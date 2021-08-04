/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
import Scene.Lights.Lights;
import Scene.Shading;
import Scene.Raytracing;
import Utils.Helpers;

struct DebugInfo{
    float3 rayOrig;
    bool isHit;

    float3 rayDir;
    float hitT;

    float3 viewDir;
    float rotate;

    float2 UV;
    uint gID;
    float r;

    float3 cameraVD;
};

StructuredBuffer<float> rotatePool;
RWStructuredBuffer<float3> directViewDirBuffer : register(u3);
RWStructuredBuffer<DebugInfo> dBuffer : register(u4);

static const int kSpriteHitKind = 0x01;

cbuffer PerFrameCB
{
    float4x4 invView;
    float2 viewportDims;
    float tanHalfFovY;
    uint sampleIndex;
    bool useDOF;
};

struct PrimaryRayData
{
    float4 color;
    uint depth;
    float hitT;
};

struct ShadowRayData
{
    bool hit;
};



[shader("miss")]
void shadowMiss(inout ShadowRayData hitData)
{
    hitData.hit = false;
}

[shader("anyhit")]
void shadowAnyHit(inout ShadowRayData hitData, in BuiltInTriangleIntersectionAttributes attribs)
{
    hitData.hit = true;
}

[shader("miss")]
void primaryMiss(inout PrimaryRayData hitData)
{
    hitData.color = float4(0.38f, 0.52f, 0.10f, 1);
    hitData.hitT = -1;
}

bool checkLightHit(uint lightIndex, float3 origin)
{
    LightData light = gScene.lights[lightIndex];
    RayDesc ray = getShadowRay(light, origin);
    ray.TMin = max(ray.TMin, 0.001);
    
    ShadowRayData rayData;
    rayData.hit = true;
    TraceRay(gRtScene, RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH, 0xFF, 1 /* ray index */, hitProgramCount, 1, ray, rayData);
    return rayData.hit;
}

uint getDispatchIndex(){
    uint3 idx = DispatchRaysIndex();
    uint3 dim = DispatchRaysDimensions();
    return idx.x + idx.y * dim.x + idx.z * dim.x * dim.y;
}

float3 getCameraViewDir(){
    return normalize(gScene.camera.data.target - gScene.camera.data.posW);
}

float3 getReflectionColor(float3 worldOrigin, VertexData v, float3 worldRayDir, uint hitDepth)
{
    float3 reflectColor = float3(0, 0, 0);
    if (hitDepth == 0)
    {
        PrimaryRayData secondaryRay;
        secondaryRay.depth.r = 1;
        RayDesc ray;
        ray.Origin = worldOrigin;
        ray.Direction = reflect(worldRayDir, v.normalW);
        float3 lastViewDir = directViewDirBuffer[getDispatchIndex()];
        directViewDirBuffer[getDispatchIndex()] = reflect(lastViewDir, v.normalW);
        ray.TMin = 0.001;
        ray.TMax = 100000;
        TraceRay(gRtScene, 0 /*rayFlags*/, 0xFF, 0 /* ray index*/, hitProgramCount, 0, ray, secondaryRay);
        reflectColor = secondaryRay.hitT == -1 ? 0 : secondaryRay.color.rgb;
        float falloff = max(1, (secondaryRay.hitT * secondaryRay.hitT));
        reflectColor *= 20 / falloff;
    }
    return reflectColor;
}

[shader("closesthit")]
void primaryClosestHit(
    uniform HitShaderParams hitParams,
    inout PrimaryRayData hitData, in BuiltInTriangleIntersectionAttributes attribs)
{
    // Get the hit-point data
    float3 rayOrigW = WorldRayOrigin();
    float3 rayDirW = WorldRayDirection();
    float hitT = RayTCurrent();
    uint triangleIndex = PrimitiveIndex();

    float3 posW = rayOrigW + hitT * rayDirW;
    // prepare the shading data
    VertexData v = getVertexData(hitParams, triangleIndex, attribs);
    uint materialID = gScene.getMaterialID(hitParams.getGlobalHitID());
    ShadingData sd = prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], -rayDirW, 0);


    // Shoot a reflection ray
    float3 reflectColor = float3(0.0);
    if (HitKind() != kSpriteHitKind){
        reflectColor = getReflectionColor(posW, v, rayDirW, hitData.depth.r);
    }
    float3 color = 0;

    if (HitKind() == kSpriteHitKind){
        color = float3(0,0,1);
    }
    else{
        for (int i = 0; i < gScene.getLightCount(); i++)
        {
            if (checkLightHit(i, posW) == false)
            {
                color += evalMaterial(sd, gScene.getLight(i), 1).color;
            }
        }
    }

    hitData.color.rgb = color;

    hitData.hitT = hitT;
    // A very non-PBR inaccurate way to do reflections
    float alpha = min(0.5, max(1e-8, sd.ggxAlpha));
    hitData.color.rgb += sd.specular * reflectColor * (alpha * alpha);
    hitData.color.rgb += sd.emissive;
    hitData.color.a = 1;
}

[shader("anyhit")]
void primaryAnyHit(
    uniform HitShaderParams                 hitParams,
    inout PrimaryRayData                    rayData,
    BuiltInTriangleIntersectionAttributes   attribs)
{
    // Alpha test for non-opaque geometry.
    VertexData v = getVertexData(hitParams, PrimitiveIndex(), attribs);
    uint materialID = gScene.getMaterialID(hitParams.getGlobalHitID());
    if (alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f))
    {
        IgnoreHit();
    }
}

[shader("raygeneration")]
void rayGen(
    uniform RWTexture2D<float4> gOutput)
{
    uint3 launchIndex = DispatchRaysIndex();
    uint randSeed = rand_init(launchIndex.x + launchIndex.y * viewportDims.x, sampleIndex, 16);

    RayDesc ray;
    if (!useDOF)
    {
        ray = gScene.camera.computeRayPinhole(launchIndex.xy, viewportDims).toRayDesc();
    }
    else
    {
        float2 u = float2(rand_next(randSeed), rand_next(randSeed));
        ray = gScene.camera.computeRayThinlens(launchIndex.xy, viewportDims, u).toRayDesc();
    }

    PrimaryRayData hitData;
    hitData.depth = 0;
    directViewDirBuffer[getDispatchIndex()] = getCameraViewDir();
    TraceRay(gRtScene, 0 /*rayFlags*/, 0xFF, 0 /* ray index*/, hitProgramCount, 0, ray, hitData);
    gOutput[launchIndex.xy] = hitData.color;
}

float3x3 rotateMatFromUtoV(float3 u, float3 v)
{
    float c = dot(u,v);
    float3 axis = cross(u,v);
    float s = length(axis);
    if (abs(s) < 1e-3){
        return float3x3(1,0,0,
                        0,1,0,
                        0,0,1);
    }
    axis = normalize(axis);

    float t = 1 - c;
    float x = axis.x;
    float y = axis.y;
    float z = axis.z;

    return float3x3(
        t * x * x + c,      t * x * y - s * z,  t * x * z + s * y,
        t * x * y + s * z,  t * y * y + c,      t * y * z - s * x,
        t * x * z - s * y,  t * y * z + s * x,  t * z * z + c
    );
}


struct SpriteIntersectionAttributes
{
    float2 UV;
};

[shader("intersection")]
void spriteIntersection(uniform HitShaderParams hitParams)
{
    float3 rayOrigW = WorldRayOrigin();
    float3 rayDirW = WorldRayDirection();
    float THit = RayTCurrent();
    AABB aabbData = gScene.getProceduralPrimitiveAABB(InstanceID(), hitParams.geometryIndex);


    // rotate
    float rot = - rotatePool[hitParams.geometryIndex];
    float cosTheta = cos(rot);
    float sinTheta = sin(rot);
    float2x2 rotToUni = float2x2(cosTheta, sinTheta, -sinTheta, cosTheta);

    float3 center = aabbData.center();
    float radius = aabbData.extent().x * 0.25 * sqrt(2.0); // * 0.5 / sqrt2

    float3 viewDir = directViewDirBuffer[getDispatchIndex()];

    // rotate to sprite space (face to view)
    float3 originViewDir = float3(0, 0, 1);
    float3x3 rotFromWtoV =  rotateMatFromUtoV(viewDir, originViewDir);

    float3 rotRayDir = mul(rotFromWtoV,rayDirW);

    float3 rotRayOrig = mul(rotFromWtoV,rayOrigW - center);

    rotRayOrig += center; // rotate around center

    // DebugInfo
    bool isHit = false;
    float hittt = 0;
    float2 UV;
    // DebugInfo

    if (abs(rotRayDir.z) > 1e-3){
        float nowHitT = (center.z - rotRayOrig.z) / rotRayDir.z;
        float2 nowHitP = (rotRayOrig + nowHitT * rotRayDir).xy;

        // map hitPos to [-1,1] x [-1,1] (with rotate)
        UV = (nowHitP - center.xy) / float2(radius);
        UV = mul(rotToUni,UV);

        if (all(abs(UV) < float2(1.0))){
            UV = UV * float2(0.5) + float2(0.5);
            BuiltInTriangleIntersectionAttributes attr = {UV};
            // DebugInfo
            isHit = true;
            hittt = nowHitT;
            // DebugInfo
            ReportHit(nowHitT, kSpriteHitKind, attr);
        }
    }

    // DebugInfo

    DebugInfo dinfo = {
    rayOrigW,
    isHit,
    rayDirW, 
    hittt,
    viewDir,
    rot,
    UV,
    hitParams.geometryIndex,
    radius,
    getCameraViewDir(),
    };


    dBuffer[getDispatchIndex()] = dinfo;
    // DebugInfo

    
}
