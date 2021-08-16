#include "RayData.slangh"
SamplerState gSampler;
Texture2D gTex;
cbuffer CSCB{
    uint gDispatchX;
};
RWStructuredBuffer<uint> unormFBO : register(u4);
RWStructuredBuffer<uint> lockBuffer : register(u2);
StructuredBuffer<RGS2CS_t> RGS2CSBuffer : register(u3);
#ifdef _DEBUG
RWStructuredBuffer<DebugInfo> dBuffer : register(u5);
#endif
StructuredBuffer<Particle> particlePool;
StructuredBuffer<Range> particleSystemRanges;
[numthreads(16,16,1)]
void doBlendCS(uint3 groupID : SV_DispatchThreadID){
    uint idx = gDispatchX * groupID.y + groupID.x;
    RGS2CS_t input = RGS2CSBuffer[idx];
    if (input.needBlend){
        SpriteRayHitInfo sHitInfo;
        sHitInfo.decode(input.packedSHitInfo);
        float3 color = dealWithSpriteAndBlend(input.vDir, input.ray, sHitInfo,
            input.color, gTex, gSampler, particlePool, particleSystemRanges,idx / MAX_DEPTH);
        color *= input.specularCoefficient;
        uint3 srcColor = round(saturate(color) * 255.0);
        uint screenIdx = idx / MAX_DEPTH;
        uint address = screenIdx * 3;
        // uint lockAvailable;
        // do{
        //      InterlockedCompareExchange(lockBuffer[screenIdx], 0, 1, lockAvailable);
        // } while(lockAvailable);
        InterlockedAdd(unormFBO[address + 0], srcColor.x);
        InterlockedAdd(unormFBO[address + 1], srcColor.y);
        InterlockedAdd(unormFBO[address + 2], srcColor.z);
        // InterlockedExchange(lockBuffer[screenIdx], 0, lockAvailable);
    }
}