#include "RayData.slangh"
StructuredBuffer<uint> RGSIndexCntBuffer;
RWByteAddressBuffer gDispatchArgsBuffer;

[numthreads(MAX_POW_TWO,1,1)]
void main(uint gpIdx : SV_GroupIndex){
    uint offset = gpIdx * 12;
    uint Cnt = RGSIndexCntBuffer[gpIdx];
    gDispatchArgsBuffer.Store3(offset,uint3(min(Cnt, MAX_GROUPSIZE_X), (Cnt == 0 ? 0 : Cnt - 1) / MAX_GROUPSIZE_X + 1, 1));
}