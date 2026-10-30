using System;
using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>Actual rig materials respond to synthetic execution state; no ROS or XR session.</summary>
public sealed class ExecutionVisibilityTests
{
    private readonly List<GameObject> _robots = new List<GameObject>();
    private readonly List<UnityEngine.Object> _resources = new List<UnityEngine.Object>();
    private GameObject _request;
    private GameObject _otherRequest;

    [SetUp]
    public void BeginWithoutExecution()
    {
        _request = new GameObject("Synthetic execution owner");
        _otherRequest = new GameObject("Second synthetic execution owner");
        SetBusy(_request, false);
        SetBusy(_otherRequest, false);
        Assert.That(IsBusy, Is.False);
    }

    [TearDown]
    public void Cleanup()
    {
        SetBusy(_request, false);
        SetBusy(_otherRequest, false);
        for (int i = _robots.Count - 1; i >= 0; i--)
            if (_robots[i] != null)
            {
                Component rig = _robots[i].GetComponent(RuntimeType("RosRobotRig"));
                if (rig != null) Private(rig, "OnDisable");
                UnityEngine.Object.DestroyImmediate(_robots[i]);
            }
        foreach (UnityEngine.Object resource in _resources)
            if (resource != null) UnityEngine.Object.DestroyImmediate(resource);
        _robots.Clear();
        _resources.Clear();
        if (_request != null) UnityEngine.Object.DestroyImmediate(_request);
        if (_otherRequest != null) UnityEngine.Object.DestroyImmediate(_otherRequest);
    }

    [Test]
    public void ExecutionDimsTheActualRobotWithoutMutatingItsMaterialsAndRestoresTheExactAppearance()
    {
        Component rig = CreateRig(false, "kenos");
        Renderer renderer = GeneratedRenderers(rig)[0];
        Material resource = Resources.Load<Material>("RobotTargetMarkers/Transparent");
        Color resourceColor = ColorOf(resource);
        Material source = new Material(resource) { name = "Visibility test textured source" };
        _resources.Add(source);
        Color sourceColor = new Color(0.23f, 0.61f, 0.87f, 0.8f);
        source.SetColor("_BaseColor", sourceColor);
        Texture2D texture = new Texture2D(2, 2);
        _resources.Add(texture);
        source.SetTexture("_BaseMap", texture);
        source.SetTextureScale("_BaseMap", new Vector2(2f, 3f));
        source.SetTextureOffset("_BaseMap", new Vector2(0.2f, 0.4f));
        Material[] originals = renderer.sharedMaterials;
        originals[0] = source;
        renderer.sharedMaterials = originals;
        renderer.shadowCastingMode = ShadowCastingMode.On;
        renderer.receiveShadows = true;
        Dictionary<Renderer, Material[]> before = CaptureMaterials(rig);

        SetBusy(_request, true);

        Assert.That(Get<bool>(rig, "IsExecutionTransparent"), Is.True);
        AssertDimming(rig);
        Material transparent = renderer.sharedMaterials[0];
        Assert.That(transparent, Is.Not.SameAs(source));
        Assert.That(ColorOf(transparent).r, Is.EqualTo(sourceColor.r).Within(1e-6f));
        Assert.That(ColorOf(transparent).g, Is.EqualTo(sourceColor.g).Within(1e-6f));
        Assert.That(ColorOf(transparent).b, Is.EqualTo(sourceColor.b).Within(1e-6f));
        Assert.That(transparent.GetTexture("_BaseMap"), Is.SameAs(texture));
        Assert.That(transparent.GetTextureScale("_BaseMap"), Is.EqualTo(new Vector2(2f, 3f)));
        Assert.That(transparent.GetTextureOffset("_BaseMap"), Is.EqualTo(new Vector2(0.2f, 0.4f)));
        Assert.That(ColorOf(source), Is.EqualTo(sourceColor));
        Assert.That(ColorOf(resource), Is.EqualTo(resourceColor), "The shared Android material must remain unchanged.");
        Assert.That(renderer.shadowCastingMode, Is.EqualTo(ShadowCastingMode.Off));
        Assert.That(renderer.receiveShadows, Is.False);

        SetBusy(_request, false);

        Assert.That(Get<bool>(rig, "IsExecutionTransparent"), Is.False);
        AssertOriginalMaterials(before);
        Assert.That(renderer.shadowCastingMode, Is.EqualTo(ShadowCastingMode.On));
        Assert.That(renderer.receiveShadows, Is.True);
        Assert.That(transparent == null, Is.True, "Temporary execution materials must be released.");
    }

    [Test]
    public void AllLiveAndTrajectoryRobotsRemainDimUntilEveryExecutionOwnerCompletes()
    {
        Component live = CreateRig(false, "kenos_sl");
        Component ghost = CreateRig(true, "kenos_sl");
        Dictionary<Renderer, Material[]> liveBefore = CaptureMaterials(live), ghostBefore = CaptureMaterials(ghost);
        SetBusy(_request, true);
        SetBusy(_otherRequest, true);
        AssertDimming(live);
        AssertDimming(ghost);
        SetBusy(_request, false);
        Assert.That(IsBusy, Is.True);
        AssertDimming(live);
        AssertDimming(ghost);
        SetBusy(_otherRequest, false);
        Assert.That(IsBusy, Is.False);
        AssertOriginalMaterials(liveBefore);
        AssertOriginalMaterials(ghostBefore);
    }

    [Test]
    public void ToolModelReplacementDuringExecutionStaysDimAndPreservesTheOriginalOpacityPolicy()
    {
        Component rig = CreateRig(false, "none");
        bool originalGhost = Get<bool>(rig, "IsGhost");
        float originalOpacity = (float)rig.GetType().GetField("_opacityLimit", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(rig);
        SetBusy(_request, true);
        AssertDimming(rig);
        Assert.That(Call(rig, "ApplyModel", RosRobotModelCatalog.Load().Find("kenos_sl")), Is.EqualTo(true));
        Assert.That(Get<bool>(rig, "IsGhost"), Is.EqualTo(originalGhost), "Temporary 3% opacity must not redefine the model's ghost policy.");
        Assert.That((float)rig.GetType().GetField("_opacityLimit", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(rig),
            Is.EqualTo(originalOpacity), "Temporary 3% opacity must not become the model template.");
        AssertDimming(rig);
        Dictionary<Renderer, Material[]> replacementOriginals = CaptureExecutionOriginalMaterials(rig);
        var replacementAlphas = new Dictionary<Material, float>();
        foreach (Material[] materials in replacementOriginals.Values)
            foreach (Material material in materials)
                if (material != null) replacementAlphas[material] = ColorOf(material).a;
        SetBusy(_request, false);
        Assert.That(Get<bool>(rig, "IsExecutionTransparent"), Is.False);
        AssertOriginalMaterials(replacementOriginals);
        foreach (KeyValuePair<Material, float> material in replacementAlphas)
            Assert.That(ColorOf(material.Key).a, Is.EqualTo(material.Value), "Restoration must preserve even a zero-alpha source.");
    }

    [Test]
    public void DisableRestoresTheNormalAppearanceAndReenableReappliesAnUnresolvedExecution()
    {
        Component rig = CreateRig(false, "kenos");
        Dictionary<Renderer, Material[]> before = CaptureMaterials(rig);
        SetBusy(_request, true);
        rig.gameObject.SetActive(false);
        Private(rig, "OnDisable");
        Assert.That(Get<bool>(rig, "IsExecutionTransparent"), Is.False);
        AssertOriginalMaterials(before);
        rig.gameObject.SetActive(true);
        Private(rig, "OnEnable");
        AssertDimming(rig);
        SetBusy(_request, false);
        AssertOriginalMaterials(before);
    }

    [Test]
    public void ADestroyedOfflineRigCannotPoisonLaterExecutionNotifications()
    {
        GameObject owner = new GameObject("Never-active offline rig");
        owner.SetActive(false);
        Component rig = owner.AddComponent(RuntimeType("RosRobotRig"));
        Private(rig, "OnEnable");
        UnityEngine.Object.DestroyImmediate(owner);
        Assert.DoesNotThrow(() => SetBusy(_request, true));
        Assert.DoesNotThrow(() => SetBusy(_request, false));
    }

    [Test]
    public void CompletingExecutionDoesNotRevealRenderersHiddenByRegistrationOrPreviewControls()
    {
        Component rig = CreateRig(false, "kenos");
        Dictionary<Renderer, Material[]> before = CaptureMaterials(rig);
        SetBusy(_request, true);
        Renderer renderer = GeneratedRenderers(rig)[0];
        renderer.enabled = false;
        SetBusy(_request, false);
        Assert.That(renderer.enabled, Is.False, "Transparency must not override another component's visibility choice.");
        AssertOriginalMaterials(before);
    }

    [Test]
    public void ARobotCreatedDuringExecutionStartsDimAndNeverIncreasesAnAlreadyLowerOpacity()
    {
        SetBusy(_request, true);
        Component rig = CreateRig(true, "kenos");
        AssertDimming(rig);
        // Restore once so a lower-opacity source can be installed without touching an asset.
        rig.gameObject.SetActive(false);
        Private(rig, "OnDisable");
        Renderer renderer = GeneratedRenderers(rig)[0];
        Material lowerOpacity = new Material(Resources.Load<Material>("RobotTargetMarkers/Transparent"));
        _resources.Add(lowerOpacity);
        lowerOpacity.SetColor("_BaseColor", new Color(0.4f, 0.7f, 1f, 0.005f));
        Material[] originals = renderer.sharedMaterials;
        originals[0] = lowerOpacity;
        renderer.sharedMaterials = originals;
        rig.gameObject.SetActive(true);
        Private(rig, "OnEnable");
        Assert.That(ColorOf(renderer.sharedMaterials[0]).a, Is.EqualTo(0.005f).Within(1e-6f));
        SetBusy(_request, false);
        Assert.That(renderer.sharedMaterials[0], Is.SameAs(lowerOpacity));
    }

    private Component CreateRig(bool ghost, string tool)
    {
        string path = "Assets/Prefabs/ur16e_kenos/ur16e_kenos" + (ghost ? "_transparent" : "") + ".prefab";
        GameObject robot = UnityEngine.Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>(path));
        _robots.Add(robot);
        robot.SetActive(false);
        Component rig = robot.GetComponent(RuntimeType("RosRobotRig")) ?? robot.AddComponent(RuntimeType("RosRobotRig"));
        rig.GetType().GetProperty("IsGhost").SetValue(rig, ghost);
        Assert.That(Call(rig, "ApplyModel", RosRobotModelCatalog.Load().Find(tool)), Is.EqualTo(true));
        robot.SetActive(true);
        Private(rig, "OnEnable"); // Keep the offline lifecycle explicit; never invokes Start or ROS transport.
        return rig;
    }
    private static Renderer[] GeneratedRenderers(Component rig)
    {
        var visuals = (List<GameObject>)rig.GetType().GetField("_generatedVisuals", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(rig);
        var result = new List<Renderer>();
        foreach (GameObject visual in visuals) result.AddRange(visual.GetComponentsInChildren<Renderer>(true));
        Assert.That(result, Is.Not.Empty);
        return result.ToArray();
    }
    private static Dictionary<Renderer, Material[]> CaptureMaterials(Component rig)
    {
        var result = new Dictionary<Renderer, Material[]>();
        foreach (Renderer renderer in GeneratedRenderers(rig)) result.Add(renderer, renderer.sharedMaterials);
        return result;
    }
    private static Dictionary<Renderer, Material[]> CaptureExecutionOriginalMaterials(Component rig)
    {
        var saved = (System.Collections.IDictionary)rig.GetType().GetField("_executionAppearances", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(rig);
        var originals = new Dictionary<Renderer, Material[]>();
        foreach (System.Collections.DictionaryEntry item in saved)
            originals.Add((Renderer)item.Key, (Material[])item.Value.GetType().GetField("Materials").GetValue(item.Value));
        Assert.That(originals, Is.Not.Empty, "The replacement model must have an execution appearance to restore.");
        return originals;
    }
    private static void AssertOriginalMaterials(Dictionary<Renderer, Material[]> originals)
    {
        foreach (KeyValuePair<Renderer, Material[]> item in originals)
        {
            Material[] actual = item.Key.sharedMaterials;
            Assert.That(actual.Length, Is.EqualTo(item.Value.Length));
            for (int index = 0; index < actual.Length; index++) Assert.That(actual[index], Is.SameAs(item.Value[index]));
        }
    }
    private static void AssertDimming(Component rig)
    {
        foreach (Renderer renderer in GeneratedRenderers(rig))
            foreach (Material material in renderer.sharedMaterials)
                if (material != null)
                {
                    Assert.That(ColorOf(material).a, Is.LessThanOrEqualTo(0.030001f));
                    Assert.That(material.GetFloat("_ZWrite"), Is.Zero);
                    Assert.That(material.IsKeywordEnabled("_SURFACE_TYPE_TRANSPARENT"), Is.True);
                }
    }
    private static Color ColorOf(Material material) => material.HasProperty("_BaseColor") ? material.GetColor("_BaseColor") : material.color;
    private static bool IsBusy => (bool)RuntimeType("RobotExecutionState").GetProperty("IsBusy").GetValue(null);
    private static void SetBusy(object owner, bool busy) => RuntimeType("RobotExecutionState").GetMethod("SetBusy").Invoke(null, new[] { owner, (object)busy });
    private static Type RuntimeType(string name) => Type.GetType(name + ", Assembly-CSharp", true);
    private static T Get<T>(object owner, string name) => (T)owner.GetType().GetProperty(name).GetValue(owner);
    private static object Call(object owner, string name, params object[] arguments) => owner.GetType().GetMethod(name).Invoke(owner, arguments);
    private static void Private(object owner, string name) => owner.GetType().GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic).Invoke(owner, null);
}
