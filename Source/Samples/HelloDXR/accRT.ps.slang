Texture2D<float4> gOutput_0 : register(u2);
Texture2D<float4> gOutput_1 : register(u3);
SamplerState gSampler;
float4 main(float2 texC : TEXCOORD, float4 posS : SV_POSITION) : SV_TARGET0
{
    float3 color1 = gOutput_0.Sample(gSampler, texC).rgb;
    float3 color2 = gOutput_1.Sample(gSampler, texC).rgb;
    return float4(color1 + color2, 1.0);
}