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
import Scene.HitInfo;

#ifndef MAX_AABB_CNT_PERRAY
#define MAX_AABB_CNT_PERRAY 10
#endif

#ifndef MAX_OIT_CNT_PERRAY
#define MAX_OIT_CNT_PERRAY 2
#endif

#ifdef _DEBUG
struct DebugInfo{
    uint begin;
    uint size;
};
#endif

struct Particle{
    float3 center;
    float scale;
    float rotate;
}

struct Range{
    uint offset;
    uint size;
}

struct AHS_RGS_Payload_t{
    uint cnt;
    uint idx[MAX_AABB_CNT_PERRAY];
}

StructuredBuffer<float> rotatePool;
RWStructuredBuffer<AHS_RGS_Payload_t> directViewDirBuffer : register(u3);
#ifdef _DEBUG
RWStructuredBuffer<DebugInfo> dBuffer : register(u4);
#endif
StructuredBuffer<Particle> particlePool;
StructuredBuffer<Range> particleSystemRanges;

static const int kSpriteHitKind = 0x01;

cbuffer PerFrameCB
{
    float4x4 invView;
    float2 viewportDims;
    float tanHalfFovY;
    uint sampleIndex;
    bool useDOF;
    //uint numParticleSystemsCount;
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

/** Payload for scatter ray.
*/

struct ScatterRayData
{
    PackedHitInfo packedHitInfo;        ///< Packed HitInfo data, or kInvalidIndex in the first component if ray missed.
    uint hitKind;
    float hitT;
    uint aabbHitCnt;
    uint aabbIdx[MAX_AABB_CNT_PERRAY];
};



[shader("miss")]
void shadowMiss(uniform HitShaderParams hitParams, inout ShadowRayData hitData)
{
    hitData.hit = false;
}

[shader("anyhit")]
void shadowAnyHit(uniform HitShaderParams hitParams, inout ShadowRayData hitData, in BuiltInTriangleIntersectionAttributes attribs)
{
    if (kSpriteHitKind == HitKind()){
        hitData.hit = false;
        IgnoreHit();
    }
    else{
        hitData.hit = true;
    }

}

[shader("miss")]
void primaryMiss(inout PrimaryRayData hitData)
{
    hitData.color = float4(0.38f, 0.52f, 0.10f, 1);
    hitData.hitT = -1;
}

[shader("miss")]
void primaryMissIR(inout ScatterRayData rayData : SV_RayPayload)
{
    rayData.packedHitInfo.x = HitInfo::kInvalidIndex;
    rayData.hitT = -1;
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
        float3 lastViewDir = float3(0.0);//directViewDirBuffer[getDispatchIndex()];
        //directViewDirBuffer[getDispatchIndex()] = reflect(lastViewDir, v.normalW);
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

[shader("closesthit")]
void primaryClosestHitIR(
    uniform HitShaderParams hitParams,
    inout ScatterRayData rayData : SV_RayPayload, in BuiltInTriangleIntersectionAttributes attribs)
{
    HitInfo hit;
    hit.type = InstanceType::TriangleMesh;
    hit.instanceID = hitParams.getGlobalHitID();
    hit.primitiveIndex = PrimitiveIndex();
    hit.barycentrics = attribs.barycentrics;
    rayData.packedHitInfo = hit.encode();
    rayData.hitKind = HitKind();
    rayData.hitT = RayTCurrent();
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

[shader("anyhit")]
void primaryAnyHitIR(
    uniform HitShaderParams                 hitParams,
    inout ScatterRayData rayData : SV_RayPayload,
    BuiltInTriangleIntersectionAttributes   attribs)
{
    if (HitKind() == kSpriteHitKind){
        if (directViewDirBuffer[getDispatchIndex()].cnt < MAX_AABB_CNT_PERRAY){
            uint idx = directViewDirBuffer[getDispatchIndex()].cnt++;
            directViewDirBuffer[getDispatchIndex()].idx[idx] = asuint(attribs.barycentrics.x);
        }
        IgnoreHit();
    }
    else{
        // Alpha test for non-opaque geometry.
        VertexData v = getVertexData(hitParams, PrimitiveIndex(), attribs);
        uint materialID = gScene.getMaterialID(hitParams.getGlobalHitID());
        if (alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f))
        {
            IgnoreHit();
        }
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
    //directViewDirBuffer[getDispatchIndex()] = getCameraViewDir();
    TraceRay(gRtScene, 0 /*rayFlags*/, 0xFF, 0 /* ray index*/, hitProgramCount, 0, ray, hitData);
    gOutput[launchIndex.xy] = hitData.color;
}

struct HitPoint{
    float3 p;
    float t;
    float hitT;
}

void dealWithSpriteAndBlend(float3 vDir, RayDesc ray, ScatterRayData rayData, inout float3 color){

    HitPoint arrayHitT[MAX_OIT_CNT_PERRAY + 1];

    float tMax = rayData.packedHitInfo.x == HitInfo::kInvalidIndex ?  100000 : rayData.hitT;

    for (uint i = 0; i < MAX_OIT_CNT_PERRAY + 1; ++i){
        arrayHitT[i].hitT = tMax;
        arrayHitT[i].p = float3(0);
        arrayHitT[i].t = 1;
    }

    float3 viewDir = vDir;

    float3 right = normalize(getOrthoVector(viewDir));

    float3 up = cross(viewDir,right);

    float4x4 rotWorld2View = getNewBasisMat(right,up,viewDir);

    AHS_RGS_Payload_t RGS_info = directViewDirBuffer[getDispatchIndex()];

    for (uint psIdx = 0; psIdx < RGS_info.cnt; ++psIdx){
        uint gIdx = RGS_info.idx[psIdx];

        //

        float3 rayOrigW = ray.Origin;
        float3 rayDirW = ray.Direction;


        Range range = particleSystemRanges[gIdx];

        uint IdxEnd = range.size + range.offset;

        bool isHit = false;

        float2 hitUV = float2(0.0);
#ifdef _DEBUG

        DebugInfo dbinfo = {
            RGS_info.cnt,
            gIdx,
        };

        dBuffer[getDispatchIndex()] = dbinfo;
#endif

        for (uint particle_ID = range.offset; particle_ID < IdxEnd; ++particle_ID){
            Particle pd = particlePool[particle_ID];
            float rot = -pd.rotate;

            float cosTheta = cos(rot);
            float sinTheta = sin(rot);

            float3 center = pd.center;
            float radius = pd.scale;

            float4x4 translateMat = getTranslateMat(-center);
            float4x4 rotAroundZ = float4x4(float4(cosTheta, sinTheta,0,0), float4(-sinTheta, cosTheta, 0, 0), float4(0, 0, 1, 0), float4(0, 0, 0, 1));
            float4x4 worldToSpriteLocalMat = mul(rotAroundZ, mul(rotWorld2View, translateMat));

            float4 rotRayDir = mul(worldToSpriteLocalMat,float4(rayDirW, 0));

            float4 rotRayOrig = mul(worldToSpriteLocalMat, float4(rayOrigW, 1));

            if (abs(rotRayDir.z) > 1e-3){
                float nowHitT = - rotRayOrig.z / rotRayDir.z;
                float2 nowHitP = (rotRayOrig + nowHitT * rotRayDir).xy;

                float2 UV = nowHitP / float2(radius);

                if (nowHitT >= ray.TMin && nowHitT < tMax && all(abs(UV) < float2(1.0))){
                    UV = UV * float2(0.5) + float2(0.5);
                    isHit = true;
                    hitUV = UV;
                    float4 spriteColor = float4((gIdx & 2) >> 1, gIdx & 1, gIdx & 1, 0.5);
                    //HitPoint lastVal = {spriteColor.rgb * spriteColor.a, 1 - spriteColor.a, nowHitT};
                    HitPoint lastVal;
                    lastVal.p = spriteColor.rgb * spriteColor.a;
                    lastVal.t = 1.0 - spriteColor.a;
                    lastVal.hitT = nowHitT;
                    for (uint i = 0;i < MAX_OIT_CNT_PERRAY + 1; ++i){
                        if (arrayHitT[i].hitT > nowHitT){
                            HitPoint tmp = lastVal;
                            lastVal = arrayHitT[i];
                            arrayHitT[i] = tmp;
                        }
                    }
                    HitPoint merge;
                    merge.p = arrayHitT[MAX_OIT_CNT_PERRAY - 1].p + (arrayHitT[MAX_OIT_CNT_PERRAY].p * arrayHitT[MAX_OIT_CNT_PERRAY - 1].t);
                    merge.t = arrayHitT[MAX_OIT_CNT_PERRAY - 1].t * arrayHitT[MAX_OIT_CNT_PERRAY].t;
                    merge.hitT = arrayHitT[MAX_OIT_CNT_PERRAY - 1].hitT;
                    arrayHitT[MAX_OIT_CNT_PERRAY - 1] = merge;
                }
            }
        }
    }

    for (uint i = 0;i < MAX_OIT_CNT_PERRAY; ++i){
        uint idx = MAX_OIT_CNT_PERRAY - i - 1;
        if (arrayHitT[idx].hitT < tMax){
            color = arrayHitT[idx].p + (color * arrayHitT[idx].t);
        }
    }
}

void updateRayState(VertexData v, ShadingData sd, ScatterRayData rayData, bool isPrimaryRay, inout float3 specularCoefficient, inout RayDesc ray, inout float3 vDir){
    // A very non-PBR inaccurate way to do reflections
    float alpha = min(0.5, max(1e-8, sd.ggxAlpha));

    // if not primary ray, calc the falloff of coef
    if (!isPrimaryRay){
        specularCoefficient = rayData.hitT == -1 ? 0 : specularCoefficient;
        float falloff = max(1, (rayData.hitT * rayData.hitT));
        specularCoefficient *= 20 / falloff;
    }
    specularCoefficient *= sd.specular * (alpha * alpha);

    // reflection
    RayDesc tmpRay;
    tmpRay.Origin = ray.Origin + ray.Direction * rayData.hitT;
    tmpRay.Direction = reflect(ray.Direction, v.normalW);
    // float3 lastViewDir = directViewDirBuffer[getDispatchIndex()];
    // directViewDirBuffer[getDispatchIndex()] = reflect(lastViewDir, v.normalW);
    vDir = reflect(vDir, v.normalW);
    tmpRay.TMin = 0.001;
    tmpRay.TMax = 100000;
    ray = tmpRay;
}

[shader("raygeneration")]
void rayGenIR(
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

    ScatterRayData rayData;
    // directViewDirBuffer[getDispatchIndex()] = getCameraViewDir();
    float3 vDir = getCameraViewDir();
    const uint maxDepth = 2;
    float4 accColor = float4(0.0,0.0,0.0,1.0);
    float3 specularCoefficient = float3(1.0);
    float3 color = 0.0;

    for (uint depth = 0; depth < maxDepth; ++depth){
        ShadingData sd;
        VertexData v;
        rayData.packedHitInfo = { HitInfo::kInvalidIndex };
        rayData.aabbHitCnt = 0;
        directViewDirBuffer[getDispatchIndex()].cnt = 0;
        TraceRay(gRtScene, 0 /*rayFlags*/, 0xFF, 0 /* ray index*/, hitProgramCount, 0, ray, rayData);
        bool bShouldUpdateState = false;
        if (rayData.packedHitInfo.x != HitInfo::kInvalidIndex){
            bShouldUpdateState = true;
            HitInfo tmpHit;
            tmpHit.decode(rayData.packedHitInfo);
            v = gScene.getVertexData(tmpHit);
            //
            uint materialID = gScene.getMaterialID(tmpHit.instanceID);
            sd = prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], -ray.Direction, 0);

            color = float3(0);

            float3 posW = ray.Origin + ray.Direction * rayData.hitT;

            if (rayData.hitKind == kSpriteHitKind){
                color = float3(0,0,1);
                bShouldUpdateState = false;
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

            color += sd.emissive;
            color *= specularCoefficient;

            accColor.rgb += color;
        }
        else{
            color = specularCoefficient * float3(0.01);
        }

        dealWithSpriteAndBlend(vDir, ray, rayData, color);

        
        accColor.rgb += color;

        if (bShouldUpdateState){
            updateRayState(v, sd, rayData, depth == 0, specularCoefficient, ray, vDir);
        }
        else{
            break;
        }

    }

    gOutput[launchIndex.xy] = accColor;
}


struct SpriteIntersectionAttributes
{
    float2 UV;
};

float4x4 getTranslateMat(float3 offset){
    return float4x4(float4(1, 0, 0, offset.x), float4(0, 1, 0, offset.y), float4(0, 0, 1, offset.z), float4(0, 0, 0, 1));
}

float4x4 getNewBasisMat(float3 x,float3 y, float3 z){
    return float4x4(float4(x, 0) , float4(y, 0), float4(z, 0), float4(0, 0, 0, 1));
}

float3 getOrthoVector(float3 vec){
    return cross(vec,float3(0, 1, 0));
}

[shader("intersection")]
void spriteIntersection(uniform HitShaderParams hitParams)
{
    float3 rayOrigW = WorldRayOrigin();
    float3 rayDirW = WorldRayDirection();
    float THit = RayTCurrent();
    AABB aabbData = gScene.getProceduralPrimitiveAABB(InstanceID(), hitParams.geometryIndex);


    // rotate
    float rot = -rotatePool[hitParams.geometryIndex];
    float cosTheta = cos(rot);
    float sinTheta = sin(rot);

    float3 center = aabbData.center();
    float radius = aabbData.extent().x * 0.25 * sqrt(2.0); // * 0.5 / sqrt2

    float3 viewDir = float3(0.0);//directViewDirBuffer[getDispatchIndex()];

    float3 right = normalize(getOrthoVector(viewDir));

    float3 up = cross(viewDir,right);

    float4x4 rotWorld2View = getNewBasisMat(right,up,viewDir);
    float4x4 translateMat = getTranslateMat(-center);
    float4x4 rotAroundZ = float4x4(float4(cosTheta, sinTheta,0,0), float4(-sinTheta, cosTheta, 0, 0), float4(0, 0, 1, 0), float4(0, 0, 0, 1));
    float4x4 worldToSpriteLocalMat = mul(rotAroundZ, mul(rotWorld2View, translateMat));

    float4 rotRayDir = mul(worldToSpriteLocalMat,float4(rayDirW,0));

    float4 rotRayOrig = mul(worldToSpriteLocalMat, float4(rayOrigW, 1));

    // DebugInfo
    bool isHit = false;
    float hittt = 0;
    float2 UV = float2(0.0);
    // DebugInfo

    if (abs(rotRayDir.z) > 1e-3){
        float nowHitT = - rotRayOrig.z / rotRayDir.z;
        float2 nowHitP = (rotRayOrig + nowHitT * rotRayDir).xy;

        UV = nowHitP / float2(radius);

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

    // DebugInfo dinfo = {
    // rayOrigW,
    // isHit,
    // rayDirW, 
    // hittt,
    // viewDir,
    // rot,
    // UV,
    // hitParams.geometryIndex,
    // radius,
    // getCameraViewDir(),
    // };


    // dBuffer[getDispatchIndex()] = dinfo;
    // DebugInfo

    
}

[shader("intersection")]
void spriteEmitterAABBIntersection(uniform HitShaderParams hitParams)
{
    float3 rayOrigW = WorldRayOrigin();
    float3 rayDirW = WorldRayDirection();
    float THit = RayTCurrent();


    // rotate
    Range range = particleSystemRanges[hitParams.geometryIndex];

    uint IdxEnd = range.size + range.offset;

    float3 viewDir = float3(0);// directViewDirBuffer[getDispatchIndex()];

    float3 right = normalize(getOrthoVector(viewDir));

    float3 up = cross(viewDir,right);

    float4x4 rotWorld2View = getNewBasisMat(right,up,viewDir);

    bool isHit = false;

    float2 hitUV = float2(0.0);

    for (uint particle_ID = range.offset; particle_ID < IdxEnd; ++particle_ID){
        Particle pd = particlePool[particle_ID];
        float rot = -pd.rotate;

        float cosTheta = cos(rot);
        float sinTheta = sin(rot);

        float3 center = pd.center;
        float radius = pd.scale;

        float4x4 translateMat = getTranslateMat(-center);
        float4x4 rotAroundZ = float4x4(float4(cosTheta, sinTheta,0,0), float4(-sinTheta, cosTheta, 0, 0), float4(0, 0, 1, 0), float4(0, 0, 0, 1));
        float4x4 worldToSpriteLocalMat = mul(rotAroundZ, mul(rotWorld2View, translateMat));

        float4 rotRayDir = mul(worldToSpriteLocalMat,float4(rayDirW, 0));

        float4 rotRayOrig = mul(worldToSpriteLocalMat, float4(rayOrigW, 1));

        if (abs(rotRayDir.z) > 1e-3){
            float nowHitT = - rotRayOrig.z / rotRayDir.z;
            float2 nowHitP = (rotRayOrig + nowHitT * rotRayDir).xy;

            float2 UV = nowHitP / float2(radius);

            if (nowHitT >= RayTMin() && nowHitT < THit && all(abs(UV) < float2(1.0))){
                UV = UV * float2(0.5) + float2(0.5);
                BuiltInTriangleIntersectionAttributes attr = {UV};
                isHit = true;
                hitUV = UV;
                THit = nowHitT;
            }
        }

    }
    BuiltInTriangleIntersectionAttributes attr = {hitUV};

    if (isHit){
        ReportHit(THit, kSpriteHitKind, attr);
    }
    
}


[shader("intersection")]
void spriteEmitterAABBIntersectionSimple(uniform HitShaderParams hitParams)
{
    BuiltInTriangleIntersectionAttributes attr = {float2(0.0)};
    attr.barycentrics.x = asfloat(hitParams.geometryIndex);
    ReportHit(RayTCurrent(), kSpriteHitKind, attr);
    
}