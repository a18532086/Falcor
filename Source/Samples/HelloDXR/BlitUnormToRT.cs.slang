cbuffer CSCB{
    uint gDispatchX;
};
RWTexture2D<float4> gOutput : register(u1);
RWStructuredBuffer<uint> unormFBO : register(u2);
[numthreads(32,8,1)]
void main(uint3 groupID : SV_DispatchThreadID){
    uint idx = gDispatchX * groupID.y + groupID.x;
    uint address = idx * 3;
    uint3 unormColor = uint3(unormFBO[address + 0], unormFBO[address + 1], unormFBO[address + 2]);
    float3 color = saturate(unormColor / 255.0);
    uint2 pos = uint2(idx % gDispatchX, idx / gDispatchX);
    gOutput[pos] = float4(color, 1.0);
}