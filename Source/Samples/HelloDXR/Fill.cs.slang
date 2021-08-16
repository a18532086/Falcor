#include "RayData.slangh"
StructuredBuffer<RangeAndIndex> SpriteIndexRangeBuffer : register(u1);
StructuredBuffer<RGS2CS_t> RGS2CSBuffer : register(u2);
StructuredBuffer<Range> particleSystemRanges : register(u3);
RWStructuredBuffer<SpriteColorInfo> SpriteColorInfoBuffer : register(u4);
[numthreads(1024,1,1)]
void main(uint3 groupID : SV_GroupID, uint gpIdx : SV_GroupIndex){
    RangeAndIndex SpriteIndexRange = SpriteIndexRangeBuffer[groupID.x];
    if (gpIdx < SpriteIndexRange.size){
        SpriteColorInfo sColorInfo = {
            float4(0),
            0,
        };
        uint rayID = SpriteIndexRange.RGS_ID;
        sColorInfo.setRGSIndex(rayID);

        RGS2CS_t input = RGS2CSBuffer[rayID];
        SpriteRayHitInfo sHitInfo;
        sHitInfo.decode(input.packedSHitInfo);
        uint preSum = 0;
        uint particleIdx = ~0;
        // for (uint psIdx = 0; psIdx < sHitInfo.aabbHitCnt; ++psIdx){
        //     uint gIdx = sHitInfo.aabbIdx[psIdx];
        //     Range range = particleSystemRanges[gIdx];
        //     if (gpIdx >= preSum && gpIdx < preSum + range.size){
        //         particleIdx = range.offset + gpIdx - preSum;
        //     }
        //     preSum += range.size;
        // }
        sColorInfo.setParticleIndex(gpIdx);
        SpriteColorInfoBuffer[SpriteIndexRange.offset + gpIdx] = sColorInfo;
    }
}