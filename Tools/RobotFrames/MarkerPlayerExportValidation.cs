using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEditor.Rendering;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

// Copied into the dedicated validation project's Assets/Editor by the preparer.
// No callback edits a shader variant list or changes the source project.
public sealed class MarkerShaderVariantAudit : IPreprocessShaders
{
    public static readonly List<MarkerPlayerExportValidation.VariantPass> Passes = new List<MarkerPlayerExportValidation.VariantPass>();
    public int callbackOrder => int.MaxValue;
    public void OnProcessShader(Shader shader, ShaderSnippetData snippet, IList<ShaderCompilerData> variants)
    {
        if (shader.name != "Universal Render Pipeline/Unlit") return;
        var item = new MarkerPlayerExportValidation.VariantPass { pass = snippet.passName, stage = snippet.shaderType.ToString(), count = variants.Count };
        foreach (ShaderCompilerData variant in variants)
        {
            bool transparent = variant.shaderKeywordSet.IsEnabled(new ShaderKeyword(shader, "_SURFACE_TYPE_TRANSPARENT"));
            if (transparent) item.transparent++; else item.opaque++;
            if (variant.shaderKeywordSet.IsEnabled(new ShaderKeyword("INSTANCING_ON"))) item.instanced++;
            if (variant.shaderKeywordSet.IsEnabled(new ShaderKeyword("STEREO_INSTANCING_ON")) ||
                variant.shaderKeywordSet.IsEnabled(new ShaderKeyword("STEREO_MULTIVIEW_ON"))) item.stereo++;
        }
        Passes.Add(item);
    }
}

public static class MarkerPlayerExportValidation
{
    [Serializable] public sealed class VariantPass { public string pass, stage; public int count, opaque, transparent, instanced, stereo; }
    [Serializable] private sealed class MaterialState {
        public string path, shader, renderType; public bool transparentKeyword, instancing;
        public int queue; public float surface, srcBlend, dstBlend, srcBlendAlpha, dstBlendAlpha, zWrite;
    }
    [Serializable] private sealed class Packed { public string path, type; public ulong bytes; }
    [Serializable] private sealed class Result {
        public bool success; public string completedUtc, unityVersion, urpVersion, buildResult, buildOptions, error;
        public MaterialState[] materials; public Packed[] packed; public VariantPass[] shaderPasses;
        public bool scriptsOnly = false, gradleInvoked = false, deviceDeployed = false, xrRuntimeTested = false;
        public string limitation = "Isolated Android player-data export with actual marker assets and URP17.0.4. No headset, Vulkan stereo rendering or full application behavior was exercised.";
    }

    public static void Export()
    {
        string project = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        Result result = new Result { unityVersion = Application.unityVersion, urpVersion = "17.0.4" };
        try
        {
            result.materials = new[] { Inspect(false), Inspect(true) };
            var clone = TargetMarkerMaterials.Create(new Color(0.2f, 0.8f, 0.1f, 0.4f), true);
            if (clone.shader.name != "Universal Render Pipeline/Unlit" || Mathf.Abs(clone.GetColor("_BaseColor").a - 0.4f) > 0.0001f)
                throw new InvalidOperationException("Runtime factory did not preserve the transparent resource template.");
            UnityEngine.Object.DestroyImmediate(clone);

            var pipeline = AssetDatabase.LoadAssetAtPath<UniversalRenderPipelineAsset>("Assets/Settings/Mobile_RPAsset.asset");
            if (pipeline == null) throw new InvalidOperationException("Missing exact app mobile URP asset.");
            GraphicsSettings.defaultRenderPipeline = pipeline;
            QualitySettings.renderPipeline = pipeline;
            PlayerSettings.SetPreloadedAssets(Array.Empty<UnityEngine.Object>());
            PlayerSettings.productName = "UnityROSML Marker Validation";
            PlayerSettings.SetApplicationIdentifier(NamedBuildTarget.Android, "com.buildwise.unityrosml.markervalidation");
            PlayerSettings.SetScriptingBackend(NamedBuildTarget.Android, ScriptingImplementation.Mono2x);
            PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARMv7;
            PlayerSettings.Android.minSdkVersion = AndroidSdkVersions.AndroidApiLevel29;
            PlayerSettings.Android.targetSdkVersion = AndroidSdkVersions.AndroidApiLevel29;
            PlayerSettings.Android.useCustomKeystore = false;
            EditorUserBuildSettings.exportAsGoogleAndroidProject = true;
            EditorUserBuildSettings.buildScriptsOnly = false;
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            new GameObject("Validation camera", typeof(Camera));
            string scenePath = "Assets/MarkerValidation.unity";
            EditorSceneManager.SaveScene(EditorSceneManager.GetActiveScene(), scenePath);
            AssetDatabase.SaveAssets();
            MarkerShaderVariantAudit.Passes.Clear();
            BuildOptions options = BuildOptions.AcceptExternalModificationsToPlayer | BuildOptions.DetailedBuildReport | BuildOptions.CleanBuildCache;
            result.buildOptions = options.ToString();
            BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
                scenes = new[] { scenePath }, target = BuildTarget.Android,
                locationPathName = Path.Combine(project, "marker-export"), options = options
            });
            result.buildResult = report.summary.result.ToString();
            if (report.summary.result != BuildResult.Succeeded) throw new InvalidOperationException("Android player-data export failed; inspect marker-editor.log.");
            List<Packed> packed = new List<Packed>();
            foreach (PackedAssets asset in report.packedAssets)
                foreach (PackedAssetInfo item in asset.contents)
                    if (item.sourceAssetPath.Contains("RobotTargetMarkers/") || item.sourceAssetPath.EndsWith("/Shaders/Unlit.shader", StringComparison.Ordinal))
                        packed.Add(new Packed { path = item.sourceAssetPath, bytes = item.packedSize, type = item.type != null ? item.type.FullName : "" });
            result.packed = packed.ToArray();
            foreach (string required in new[] { "Assets/Resources/RobotTargetMarkers/Opaque.mat", "Assets/Resources/RobotTargetMarkers/Transparent.mat",
                                               "Packages/com.unity.render-pipelines.universal/Shaders/Unlit.shader" })
                if (!packed.Any(item => item.path == required && item.bytes > 0))
                    throw new InvalidOperationException("BuildReport does not retain " + required);
            result.shaderPasses = MarkerShaderVariantAudit.Passes.ToArray();
            var forward = result.shaderPasses.Where(item => item.pass == "Unlit").ToArray();
            if (!forward.Any(item => item.opaque > 0) || !forward.Any(item => item.transparent > 0))
                throw new InvalidOperationException("Both opaque and transparent Unlit forward variants must survive stripping.");
            result.success = true;
            Debug.Log("MarkerPlayerValidation: both Resources materials, Unlit shader and opaque/transparent forward variants retained in full Android player data.");
        }
        catch (Exception exception) { result.error = exception.ToString(); Debug.LogError(exception); }
        finally
        {
            result.completedUtc = DateTime.UtcNow.ToString("O");
            if (result.shaderPasses == null) result.shaderPasses = MarkerShaderVariantAudit.Passes.ToArray();
            File.WriteAllText(Path.Combine(project, "marker-validation-result.json"), JsonUtility.ToJson(result, true), new UTF8Encoding(false));
        }
        if (!result.success) EditorApplication.Exit(1);
    }

    private static MaterialState Inspect(bool transparent)
    {
        string path = "Assets/Resources/RobotTargetMarkers/" + (transparent ? "Transparent" : "Opaque") + ".mat";
        Material material = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (material == null || material.shader == null || material.shader.name != "Universal Render Pipeline/Unlit")
            throw new InvalidOperationException("Missing or wrong shader on " + path);
        MaterialState state = new MaterialState {
            path = path, shader = material.shader.name, renderType = material.GetTag("RenderType", false),
            transparentKeyword = material.IsKeywordEnabled("_SURFACE_TYPE_TRANSPARENT"), instancing = material.enableInstancing,
            queue = material.renderQueue, surface = material.GetFloat("_Surface"), srcBlend = material.GetFloat("_SrcBlend"),
            dstBlend = material.GetFloat("_DstBlend"), srcBlendAlpha = material.GetFloat("_SrcBlendAlpha"),
            dstBlendAlpha = material.GetFloat("_DstBlendAlpha"), zWrite = material.GetFloat("_ZWrite")
        };
        if (state.transparentKeyword != transparent || !state.instancing || state.surface != (transparent ? 1 : 0) ||
            state.queue != (transparent ? 3000 : 2000) || state.srcBlend != (transparent ? 5 : 1) ||
            state.dstBlend != (transparent ? 10 : 0) || state.srcBlendAlpha != 1 ||
            state.dstBlendAlpha != (transparent ? 10 : 0) || state.zWrite != (transparent ? 0 : 1))
            throw new InvalidOperationException("Wrong surface/blend/instancing state on " + path);
        return state;
    }
}
