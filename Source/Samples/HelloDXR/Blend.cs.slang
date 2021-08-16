#include "RayData.slangh"

#ifndef MAX_SPRITE_PER_AABB_BIT_LSF16
#define MAX_SPRITE_PER_AABB_BIT_LSF16 0
#endif

#ifndef MAX_SPRITE_PER_AABB
#define MAX_SPRITE_PER_AABB (16 << MAX_SPRITE_PER_AABB_BIT_LSF16)
#endif

SamplerState gSampler;
Texture2D<float4> gTex;
cbuffer CSCB{
    uint gDispatchX;
};
RWTexture2D<float4> gOutput : register(u1);
RWStructuredBuffer<uint> unormFBO : register(u4);
RWStructuredBuffer<uint> lockBuffer : register(u2);
StructuredBuffer<RGS2CS_t> RGS2CSBuffer : register(u3);
#ifdef _DEBUG
RWStructuredBuffer<DebugInfo> dBuffer : register(u5);
struct BlendInfoArray{
    float ba[MAX_SPRITE_PER_AABB];
    float color[MAX_SPRITE_PER_AABB];
};
RWStructuredBuffer<BlendInfoArray> badBuffer : register(u6);
#endif
StructuredBuffer<Particle> particlePool;
StructuredBuffer<Range> particleSystemRanges;
// [numthreads(16,16,1)]
// void doBlendCS(uint3 groupID : SV_DispatchThreadID){
//     uint idx = gDispatchX * groupID.y + groupID.x;
//     RGS2CS_t input = RGS2CSBuffer[idx];
//     if (input.needBlend){
//         SpriteRayHitInfo sHitInfo;
//         sHitInfo.decode(input.packedSHitInfo);
//         float3 color = dealWithSpriteAndBlend(input.vDir, input.ray, sHitInfo,
//             input.color, gTex, gSampler, particlePool, particleSystemRanges,idx / MAX_DEPTH);
//         color *= input.specularCoefficient;
//         uint3 srcColor = round(saturate(color) * 255.0);
//         uint screenIdx = idx / MAX_DEPTH;
//         uint address = screenIdx * 3;
//         // uint lockAvailable;
//         // do{
//         //      InterlockedCompareExchange(lockBuffer[screenIdx], 0, 1, lockAvailable);
//         // } while(lockAvailable);
//         InterlockedAdd(unormFBO[address + 0], srcColor.x);
//         InterlockedAdd(unormFBO[address + 1], srcColor.y);
//         InterlockedAdd(unormFBO[address + 2], srcColor.z);
//         // InterlockedExchange(lockBuffer[screenIdx], 0, lockAvailable);
//     }
// }
uint InsertOneBit( uint Value, uint OneBitMask )
{
    uint Mask = OneBitMask - 1;
    return (Value & ~Mask) << 1 | (Value & Mask) | OneBitMask;
}

struct BlendInfo{
    float4 color;
    float hitT;
}
groupshared BlendInfo blendArray[MAX_SPRITE_PER_AABB][MAX_DEPTH];
groupshared float3 colors[MAX_DEPTH];
[numthreads(MAX_SPRITE_PER_AABB, MAX_DEPTH, 1)]
void doBlendCSParallex(uint3 groupID : SV_GroupID, uint3 gTID : SV_GroupThreadID){
    uint idx = gDispatchX * groupID.y + groupID.x;
    uint gpIdx = gTID.x;
    uint depth = gTID.y;
    idx = MAX_DEPTH * idx + gTID.y;
    RGS2CS_t input = RGS2CSBuffer[idx];
    colors[depth] = float3(0);
    blendArray[gpIdx][depth].color = float4(0.0);
    blendArray[gpIdx][depth].hitT = -1;
    if (input.needBlend){

        if (gpIdx < input.cnt){
            SpriteRayHitInfo rayData;

            rayData.decode(input.packedSHitInfo);

            float3 vDir = input.vDir;

            RayDesc ray = input.ray;

            float tMax = rayData.hitT == -1 ?  100000 : rayData.hitT;

            float3 viewDir = vDir;

            float3 right = normalize(getOrthoVector(viewDir));

            float3 up = cross(viewDir,right);

            float3x3 rotWorld2View = getNewBasisMat(right,up,viewDir);

            uint preSum = 0;

            uint particle_ID = ~0;

            for (uint psIdx = 0; psIdx < rayData.aabbHitCnt; ++psIdx){
                uint gIdx = rayData.aabbIdx[psIdx];
                Range range = particleSystemRanges[gIdx];
                if (gpIdx >= preSum && gpIdx < preSum + range.size){
                    particle_ID = gpIdx - preSum + range.offset;
                    break;
                }
                preSum += range.size;
            }
                
            float3 rayOrigW = ray.Origin;
            float3 rayDirW = ray.Direction;

            bool isHit = false;

            float2 hitUV = float2(0.0);

            Particle pd = particlePool[particle_ID];

            float3 center = pd.center;
            float radius = pd.scale;

            float3 rotRayDir = rotateAroundZ(mul(rotWorld2View ,rayDirW), pd.cosRot, pd.sinRot);
            float3 rotRayOrig = rotateAroundZ(mul(rotWorld2View ,rayOrigW - center), pd.cosRot, pd.sinRot);

            if (abs(rotRayDir.z) > 1e-3){

                float nowHitT = - rotRayOrig.z / rotRayDir.z;
                float2 nowHitP = (rotRayOrig + nowHitT * rotRayDir).xy;

                float2 UV = nowHitP / float2(radius);

                if (nowHitT >= ray.TMin && nowHitT < tMax && all(abs(UV) < float2(1.0))){
                    UV = UV * float2(0.5) + float2(0.5);
                    isHit = true;
                    hitUV = UV;
                    float4 spriteColor = proceduralTexture(gTex, gSampler, UV);
                    blendArray[gpIdx][depth].color = spriteColor;
                    blendArray[gpIdx][depth].hitT = nowHitT;

                }

            }
        }
        

        GroupMemoryBarrierWithGroupSync();

        [unroll]
        for (uint k = 2; k <= MAX_SPRITE_PER_AABB; k <<= 1){
            for (uint j = k / 2; j > 0; j /= 2){
                uint Index2 = InsertOneBit(gpIdx, j);
                uint Index1 = Index2 ^ (k == 2 * j ? k - 1 : j);
                BlendInfo A = blendArray[Index1][depth];
                BlendInfo B = blendArray[Index2][depth];
                if (A.hitT < B.hitT){
                    blendArray[Index2][depth] = A;
                    blendArray[Index1][depth] = B;
                }
                GroupMemoryBarrierWithGroupSync();
            }
        }
#ifdef _DEBUG
        badBuffer[idx].ba[gpIdx] = blendArray[gpIdx][depth].hitT;
#endif
        if (gpIdx == 0){
            static float3 color = input.color;
            [unroll]
            for (uint i = 0; i < MAX_SPRITE_PER_AABB; ++i){
                if (blendArray[i][depth].hitT == -1){
                    color = color;
                }
                else{
                    color = lerp(color, blendArray[i][depth].color.rgb, blendArray[i][depth].color.a);
                }

            }
            color *= input.specularCoefficient;
            colors[depth] = color;
        }
    }
    GroupMemoryBarrierWithGroupSync();
    if (depth == 0){
        gOutput[input.getPos()] = float4(0.0);
        for (uint d = 0; d < MAX_DEPTH;++d){
            gOutput[input.getPos()].rgb += colors[d];
        }
    }
}