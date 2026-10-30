Shader "UnityROSML/RobotContourDepthNormals"
{
    SubShader
    {
        Tags { "RenderPipeline"="UniversalPipeline" "RenderType"="Opaque" }
        Pass
        {
            Name "Private robot normals and depth"
            Cull Back
            ZWrite On
            ZTest LEqual
            Blend Off
            HLSLPROGRAM
            #pragma target 3.5
            #pragma vertex ContourDepthVertex
            #pragma fragment ContourDepthFragment
            #pragma multi_compile_instancing
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            float _ContourDepthRange;
            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };
            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 normalWS : TEXCOORD0;
                float eyeDepth : TEXCOORD1;
                UNITY_VERTEX_OUTPUT_STEREO
            };
            Varyings ContourDepthVertex(Attributes input)
            {
                Varyings output;
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);
                float3 world = TransformObjectToWorld(input.positionOS.xyz);
                // Use the same native per-eye matrices as the filled robot.
                // URP late-latches these after CPU pass preparation on XR.
                output.positionCS = TransformWorldToHClip(world);
                output.normalWS = TransformObjectToWorldNormal(input.normalOS);
                output.eyeDepth = -TransformWorldToView(world).z;
                return output;
            }
            float2 EncodeNormal(float3 value)
            {
                value /= max(abs(value.x) + abs(value.y) + abs(value.z), 1e-6);
                float2 oct = value.xy;
                if (value.z < 0)
                    oct = (1 - abs(oct.yx)) * float2(oct.x >= 0 ? 1 : -1, oct.y >= 0 ? 1 : -1);
                return oct * 0.5 + 0.5;
            }
            float4 ContourDepthFragment(Varyings input) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);
                // Reserve all-zero depth for untouched background. Two UNorm8
                // channels provide <=1.53 mm resolution even with a 100 m far range.
                float depth = clamp(round(saturate(input.eyeDepth / _ContourDepthRange) * 65535), 1, 65535);
                float2 packedDepth = float2(floor(depth / 256), depth - floor(depth / 256) * 256) / 255;
                return float4(EncodeNormal(normalize(input.normalWS)), packedDepth);
            }
            ENDHLSL
        }
    }
    Fallback Off
}
