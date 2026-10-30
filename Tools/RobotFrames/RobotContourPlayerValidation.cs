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

// Copy into the prepared point-registration validation project's Assets/Editor.
// This callback observes the final variants; it never adds or removes any.
public sealed class RobotContourShaderVariantAudit : IPreprocessShaders
{
    public static readonly List<RobotContourPlayerValidation.VariantPass> Passes =
        new List<RobotContourPlayerValidation.VariantPass>();
    public int callbackOrder => int.MaxValue;

    public void OnProcessShader(Shader shader, ShaderSnippetData snippet, IList<ShaderCompilerData> variants)
    {
        if (shader.name != RobotContourPlayerValidation.DepthShader &&
            shader.name != RobotContourPlayerValidation.CompositeShader) return;
        foreach (IGrouping<ShaderCompilerPlatform, ShaderCompilerData> platform in variants.GroupBy(item => item.shaderCompilerPlatform))
        {
            var pass = new RobotContourPlayerValidation.VariantPass {
                shader = shader.name, pass = snippet.passName, stage = snippet.shaderType.ToString(),
                platform = platform.Key.ToString(), count = platform.Count()
            };
            foreach (ShaderCompilerData variant in platform)
            {
                if (variant.shaderKeywordSet.IsEnabled(new ShaderKeyword("INSTANCING_ON"))) pass.instanced++;
                if (variant.shaderKeywordSet.IsEnabled(new ShaderKeyword("STEREO_INSTANCING_ON")) ||
                    variant.shaderKeywordSet.IsEnabled(new ShaderKeyword("STEREO_MULTIVIEW_ON"))) pass.stereo++;
            }
            Passes.Add(pass);
        }
    }
}

/// <summary>Offline Android player-data validation; exporting a Gradle project never invokes Gradle.</summary>
public static class RobotContourPlayerValidation
{
    public const string DepthShader = "UnityROSML/RobotContourDepthNormals";
    public const string CompositeShader = "UnityROSML/RobotContourComposite";
    private const string ResourceRoot = "Assets/Resources/RobotContours/";

    [Serializable] public sealed class VariantPass {
        public string shader, pass, stage, platform;
        public int count, instanced, stereo;
    }
    [Serializable] private sealed class MaterialState {
        public string path, resourcePath, shader, shaderPath;
        public bool instancing, loadedByResources;
        public int queue, passes;
    }
    [Serializable] private sealed class Packed { public string path, type; public ulong bytes; }
    [Serializable] private sealed class Result {
        public bool success;
        public string completedUtc, unityVersion, urpVersion, buildResult, buildOptions, exportPath, error;
        public string mobileRenderer, graphicsApi, scriptingBackend, architecture;
        public MaterialState[] materials;
        public Packed[] packed;
        public VariantPass[] shaderPasses;
        public bool scriptsOnly = false, gradleInvoked = false, deviceDeployed = false, xrRuntimeTested = false;
        public string shaderStageConvention = "Vulkan includes all stages in the Vertex program type; each retained program contains the vertex and fragment stages.";
        public string limitation = "Full isolated Android Vulkan player-data/Gradle project export using the actual contour shaders, Resources materials and mobile URP renderer. Mono ARMv7 avoids IL2CPP and Gradle. No ML2, stereo runtime rendering, full application behavior or robot operation is exercised.";
    }

    public static void Export()
    {
        string project = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
        var result = new Result { unityVersion = Application.unityVersion, graphicsApi = "Vulkan",
            scriptingBackend = "Mono2x", architecture = "ARMv7" };
        try
        {
            RequireIsolatedHarness(project);
            RobotContourShaderVariantAudit.Passes.Clear();
            result.materials = new[] {
                Inspect("DepthNormalsMaterial", DepthShader, "RobotContourDepthNormals.shader", 2000),
                Inspect("CompositeMaterial", CompositeShader, "RobotContourComposite.shader", 3000)
            };
            var pipeline = AssetDatabase.LoadAssetAtPath<UniversalRenderPipelineAsset>("Assets/Settings/Mobile_RPAsset.asset");
            var renderer = AssetDatabase.LoadAssetAtPath<UniversalRendererData>("Assets/Settings/Mobile_Renderer.asset");
            if (pipeline == null || renderer == null) throw new InvalidOperationException("Missing exact application mobile URP assets.");
            result.mobileRenderer = AssetDatabase.GetAssetPath(renderer);
            result.urpVersion = UnityEditor.PackageManager.PackageInfo.FindForAssetPath("Packages/com.unity.render-pipelines.universal").version;
            InspectFeature(renderer, result.materials);
            GraphicsSettings.defaultRenderPipeline = pipeline;
            QualitySettings.renderPipeline = pipeline;
            PlayerSettings.SetPreloadedAssets(Array.Empty<UnityEngine.Object>());
            PlayerSettings.productName = "UnityROSML Contour Validation";
            PlayerSettings.SetApplicationIdentifier(NamedBuildTarget.Android, "com.buildwise.unityrosml.contourvalidation");
            PlayerSettings.SetScriptingBackend(NamedBuildTarget.Android, ScriptingImplementation.Mono2x);
            PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARMv7;
            PlayerSettings.Android.minSdkVersion = AndroidSdkVersions.AndroidApiLevel29;
            PlayerSettings.Android.targetSdkVersion = AndroidSdkVersions.AndroidApiLevel29;
            PlayerSettings.Android.useCustomKeystore = false;
            PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.Android, false);
            PlayerSettings.SetGraphicsAPIs(BuildTarget.Android, new[] { GraphicsDeviceType.Vulkan });
            EditorUserBuildSettings.exportAsGoogleAndroidProject = true;
            EditorUserBuildSettings.buildScriptsOnly = false;
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            new GameObject("Offline contour export camera", typeof(Camera));
            string scenePath = "Assets/RobotContourValidation.unity";
            EditorSceneManager.SaveScene(EditorSceneManager.GetActiveScene(), scenePath);
            AssetDatabase.SaveAssets();
            BuildOptions options = BuildOptions.AcceptExternalModificationsToPlayer | BuildOptions.DetailedBuildReport | BuildOptions.CleanBuildCache;
            result.buildOptions = options.ToString();
            result.exportPath = Path.Combine(project, "contour-export");
            BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
                scenes = new[] { scenePath }, target = BuildTarget.Android,
                locationPathName = result.exportPath, options = options
            });
            result.buildResult = report.summary.result.ToString();
            result.shaderPasses = RobotContourShaderVariantAudit.Passes.ToArray();
            if (report.summary.result != BuildResult.Succeeded)
                throw new InvalidOperationException("Android contour player-data export failed; inspect the Unity editor log.");
            result.packed = report.packedAssets.SelectMany(asset => asset.contents)
                .Where(item => item.sourceAssetPath.StartsWith(ResourceRoot, StringComparison.Ordinal))
                .Select(item => new Packed { path = item.sourceAssetPath, bytes = item.packedSize,
                    type = item.type != null ? item.type.FullName : "" }).ToArray();
            foreach (MaterialState material in result.materials)
            {
                foreach (string required in new[] { material.path, material.shaderPath })
                    if (!result.packed.Any(item => item.path == required && item.bytes > 0))
                        throw new InvalidOperationException("BuildReport did not retain " + required);
                // Vulkan reports the combined graphics program as Vertex, with
                // all stages included, rather than separate Fragment callbacks.
                // https://docs.unity3d.com/ScriptReference/ShaderData.Pass.PreprocessVariant.html
                if (!result.shaderPasses.Any(item => item.shader == material.shader && item.stage == "Vertex" &&
                    item.platform == "Vulkan" && item.count > 0))
                    throw new InvalidOperationException("No retained Android Vulkan graphics program variants for " + material.shader);
            }
            string library = Path.Combine(result.exportPath, "unityLibrary");
            string gradle = File.ReadAllText(Path.Combine(library, "build.gradle"));
            if (!gradle.Contains("abiFilters \"armeabi-v7a\"") ||
                !File.Exists(Path.Combine(library, "src/main/assets/bin/Data/Managed/Assembly-CSharp.dll")))
                throw new InvalidOperationException("Expected complete Mono ARMv7 Gradle player-data export artifacts.");
            result.success = true;
            Debug.Log("RobotContourPlayerValidation: both contour shaders compiled for Vulkan and were retained with their Resources materials in full Android player data.");
        }
        catch (Exception exception) { result.error = exception.ToString(); Debug.LogError(exception); }
        finally
        {
            result.completedUtc = DateTime.UtcNow.ToString("O");
            if (result.shaderPasses == null) result.shaderPasses = RobotContourShaderVariantAudit.Passes.ToArray();
            File.WriteAllText(Path.Combine(project, "contour-player-validation-result.json"), JsonUtility.ToJson(result, true), new UTF8Encoding(false));
        }
        if (!result.success) EditorApplication.Exit(1);
    }

    private static void RequireIsolatedHarness(string project)
    {
        if (new DirectoryInfo(project).Parent?.Name != "Library" ||
            !File.Exists(Path.Combine(project, "Assets/OfflineValidationWorkflowStubs.cs")) ||
            File.Exists(Path.Combine(project, "Assets/Scenes/UR16eInteraction.unity")))
            throw new InvalidOperationException("Run only from a prepared isolated workflow project under the source project's Library directory.");
    }

    private static MaterialState Inspect(string name, string shader, string shaderFile, int queue)
    {
        string path = ResourceRoot + name + ".mat";
        Material material = AssetDatabase.LoadAssetAtPath<Material>(path);
        string resource = "RobotContours/" + name;
        if (material == null || material.shader == null || material.shader.name != shader ||
            AssetDatabase.GetAssetPath(material.shader) != ResourceRoot + shaderFile || material.renderQueue != queue ||
            !material.enableInstancing || Resources.Load<Material>(resource) != material)
            throw new InvalidOperationException("Missing or incorrect direct Resources contour template: " + path);
        return new MaterialState { path = path, resourcePath = resource, shader = shader,
            shaderPath = AssetDatabase.GetAssetPath(material.shader), instancing = material.enableInstancing,
            loadedByResources = true, queue = material.renderQueue, passes = material.passCount };
    }

    private static void InspectFeature(UniversalRendererData renderer, MaterialState[] materials)
    {
        ScriptableRendererFeature feature = renderer.rendererFeatures.FirstOrDefault(item =>
            item != null && item.GetType().Name == "RobotContourRendererFeature" && item.isActive);
        if (feature == null) throw new InvalidOperationException("Mobile renderer must include the active contour feature.");
        var serialized = new SerializedObject(feature);
        string[] fields = { "_depthNormalsMaterial", "_compositeMaterial" };
        for (int index = 0; index < fields.Length; index++)
        {
            SerializedProperty property = serialized.FindProperty(fields[index]);
            if (property == null || AssetDatabase.GetAssetPath(property.objectReferenceValue) != materials[index].path)
                throw new InvalidOperationException("Mobile contour feature does not directly reference " + materials[index].path);
        }
    }
}
