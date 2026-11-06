Shader "UnityROSML/RobotContourComposite"
{
    Properties
    {
        _ContourColor ("Contour color", Color) = (0.2, 0.9, 1, 0.9)
        _ContourWidth ("Contour width in pixels", Float) = 2
        _ContourDepthRange ("Packed eye depth range", Float) = 100
    }
    SubShader
    {
        Tags { "RenderPipeline"="UniversalPipeline" "RenderType"="Transparent" "Queue"="Transparent" }
        Pass
        {
            Name "Visible robot contours only"
            Cull Off
            ZWrite Off
            ZTest Always
            Blend SrcAlpha OneMinusSrcAlpha, One OneMinusSrcAlpha
            HLSLPROGRAM
            #pragma target 3.5
            #pragma vertex Vert
            #pragma fragment ContourFragment
            #pragma multi_compile_instancing
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.core/Runtime/Utilities/Blit.hlsl"
            float4 _ContourColor;
            float4 _ContourTexelSize;
            float _ContourWidth;
            float _ContourDepthRange;

            float DecodeDepth(float4 value)
            {
                return (round(value.b * 255) * 256 + round(value.a * 255)) * (_ContourDepthRange / 65535);
            }
            float3 DecodeNormal(float2 oct)
            {
                oct = oct * 2 - 1;
                float3 value = float3(oct, 1 - abs(oct.x) - abs(oct.y));
                float correction = saturate(-value.z);
                value.xy += float2(value.x >= 0 ? -correction : correction, value.y >= 0 ? -correction : correction);
                return normalize(value);
            }
            float ContourPlaneValue(float depth)
            {
                // Linear eye depth is affine for orthographic projection; inverse
                // eye depth is affine for perspective projection.
                return unity_OrthoParams.w > 0.5 ? depth : rcp(depth);
            }
            float ContourPlaneSlope(float4 before, float4 after, float centerValue, float3 normal)
            {
                float beforeDepth = DecodeDepth(before), afterDepth = DecodeDepth(after);
                if (beforeDepth <= 0 || afterDepth <= 0 ||
                    dot(normal, DecodeNormal(before.rg)) < 0.99 || dot(normal, DecodeNormal(after.rg)) < 0.99) return 0;
                float left = centerValue - ContourPlaneValue(beforeDepth);
                float right = ContourPlaneValue(afterDepth) - centerValue;
                // Minmod takes the continuous side of a depth jump, including a
                // jump between parallel faces. It never treats that jump as a plane.
                return left * right > 0 ? sign(left) * min(abs(left), abs(right)) : 0;
            }
            float4 ContourFragment(Varyings input) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);
                float4 center = SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_PointClamp, input.texcoord);
                float depth = DecodeDepth(center);
                if (depth <= 0) discard;
                float3 normal = DecodeNormal(center.rg);
                float2 stepUV = _ContourTexelSize.xy * max(1, _ContourWidth);
                // Compare against that plane rather than its centre depth: a grazing
                // face can otherwise exceed the depth-jump threshold at every pixel.
                float planeValue = ContourPlaneValue(depth);
                float2 planeSlope = float2(
                    ContourPlaneSlope(
                        SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_PointClamp, input.texcoord - float2(stepUV.x, 0)),
                        SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_PointClamp, input.texcoord + float2(stepUV.x, 0)), planeValue, normal),
                    ContourPlaneSlope(
                        SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_PointClamp, input.texcoord - float2(0, stepUV.y)),
                        SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_PointClamp, input.texcoord + float2(0, stepUV.y)), planeValue, normal));
                float edge = 0;
                const float2 offsets[8] = {
                    float2(-1, 0), float2(1, 0), float2(0, -1), float2(0, 1),
                    float2(-1, -1), float2(-1, 1), float2(1, -1), float2(1, 1)
                };
                [unroll] for (int i = 0; i < 8; i++)
                {
                    float4 adjacent = SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_PointClamp, input.texcoord + offsets[i] * stepUV);
                    float otherDepth = DecodeDepth(adjacent);
                    float expectedValue = planeValue + dot(planeSlope, offsets[i]);
                    float expectedDepth = expectedValue > 0 ? ContourPlaneValue(expectedValue) : depth;
                    if (otherDepth <= 0 || abs(otherDepth - expectedDepth) > max(0.012, depth * 0.015))
                    {
                        edge = 1;
                        break;
                    }
                }
                if (edge == 0)
                {
                    // Creases cover both sides of an edge, so use half the silhouette
                    // radius. A 60-degree threshold rejects smooth CAD curvature.
                    float2 normalStepUV = _ContourTexelSize.xy * max(0.5, _ContourWidth * 0.5);
                    [unroll] for (int i = 0; i < 8; i++)
                    {
                        float4 adjacent = SAMPLE_TEXTURE2D_X(_BlitTexture, sampler_PointClamp,
                            input.texcoord + offsets[i] * normalStepUV);
                        if (DecodeDepth(adjacent) > 0 && dot(normal, DecodeNormal(adjacent.rg)) < 0.5)
                        {
                            edge = 1;
                            break;
                        }
                    }
                }
                if (edge == 0) discard;
                return _ContourColor;
            }
            ENDHLSL
        }
    }
    Fallback Off
}
