using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>Live-robot identity and draw suppression, without ROS, XR or a graphics device.</summary>
public sealed class RobotContourDisplayTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private GameObject _executionOwner;
    private GameObject _renderingProvider;

    [SetUp]
    public void ResetPresentation()
    {
        Static("SetEnabled", false);
        Static("SetLiveRobot", new object[] { null });
        _executionOwner = New("Synthetic contour execution");
        _renderingProvider = New("Synthetic supported contour renderer");
        Static("SetRenderingAvailable", _renderingProvider, true);
        Busy(false);
    }

    [TearDown]
    public void Cleanup()
    {
        Busy(false);
        Static("SetEnabled", false);
        Static("SetLiveRobot", new object[] { null });
        Static("SetRenderingAvailable", _renderingProvider, false);
        for (int i = _owned.Count - 1; i >= 0; i--)
            if (_owned[i] != null) UnityEngine.Object.DestroyImmediate(_owned[i]);
        _owned.Clear();
    }

    [Test]
    public void ContoursSuppressOnlyTheExplicitLiveRobotAndLeaveTrajectoryGhostsUntouched()
    {
        Component live = Rig(false, "kenos");
        Component ghost = Rig(true, "kenos");
        // Main imported prefab materials can legitimately classify as a ghost.
        // Explicit placement identity must still select this particular rig.
        live.GetType().GetProperty("IsGhost").SetValue(live, true);
        ghost.transform.position += Vector3.right * 10f;
        Dictionary<MeshRenderer, Appearance> originals = Capture(live);
        Dictionary<MeshRenderer, Appearance> ghosts = Capture(ghost);

        Static("SetLiveRobot", live);
        Static("SetEnabled", true);

        Assert.That(Enabled, Is.True);
        Assert.That(Draws().Count, Is.GreaterThan(0));
        foreach (KeyValuePair<MeshRenderer, Appearance> item in originals)
        {
            Assert.That(item.Key.forceRenderingOff, Is.True);
            Assert.That(item.Key.enabled, Is.EqualTo(item.Value.Enabled));
            AssertMaterials(item.Key, item.Value.Materials);
        }
        AssertUnchanged(ghosts);
        Static("SetEnabled", false);
        AssertUnchanged(originals);
        AssertUnchanged(ghosts);
        Assert.That(Draws(), Is.Empty);
    }

    [Test]
    public void RegistrationHiddenAndOriginallySuppressedVisualsProduceNoContourDraws()
    {
        Component live = Rig(false, "kenos_sl");
        List<MeshRenderer> renderers = Renderers(live);
        Assert.That(renderers.Count, Is.GreaterThanOrEqualTo(3));
        MeshRenderer disabled = renderers[0], inactive = renderers[1], originallyOff = renderers[2];
        disabled.enabled = false;
        inactive.gameObject.SetActive(false);
        originallyOff.forceRenderingOff = true;
        int expected = 0;
        foreach (MeshRenderer renderer in renderers)
            if (renderer.enabled && renderer.gameObject.activeInHierarchy && !renderer.forceRenderingOff)
                expected += renderer.GetComponent<MeshFilter>().sharedMesh.subMeshCount;

        Static("SetLiveRobot", live);
        Static("SetEnabled", true);

        Assert.That(Draws().Count, Is.EqualTo(expected));
        Static("SetEnabled", false);
        Assert.That(disabled.enabled, Is.False, "Contour cleanup must not reveal a registration-hidden renderer.");
        Assert.That(inactive.gameObject.activeSelf, Is.False);
        Assert.That(originallyOff.forceRenderingOff, Is.True, "An existing suppression belongs to its original owner.");
        Assert.That(disabled.forceRenderingOff, Is.False);
    }

    [Test]
    public void DrawsUseCurrentJointAndTrackingTransformsWithoutCreatingAnotherRobot()
    {
        Component live = Rig(false, "kenos");
        Static("SetLiveRobot", live);
        Static("SetEnabled", true);
        live.transform.SetPositionAndRotation(new Vector3(2f, 0.4f, -3f), Quaternion.Euler(0f, 43f, 0f));
        Assert.That(Call(live, "SetJointPositions", new[] { "ur_shoulder_pan_joint", "ur_elbow_joint" }, new[] { 0.7d, -0.6d }), Is.EqualTo(true));
        var expected = new HashSet<Matrix4x4>();
        foreach (MeshRenderer renderer in Renderers(live))
            if (renderer.enabled && renderer.gameObject.activeInHierarchy) expected.Add(renderer.localToWorldMatrix);
        IList draws = Draws();
        Assert.That(draws, Is.Not.Empty);
        foreach (object draw in draws)
        {
            Assert.That(expected.Contains((Matrix4x4)draw.GetType().GetField("LocalToWorld").GetValue(draw)), Is.True);
            Mesh mesh = (Mesh)draw.GetType().GetField("Mesh").GetValue(draw);
            int submesh = (int)draw.GetType().GetField("SubMeshIndex").GetValue(draw);
            Assert.That(mesh, Is.Not.Null);
            Assert.That(submesh, Is.InRange(0, mesh.subMeshCount - 1));
        }
        Assert.That(live.gameObject.GetComponents(RuntimeType("RosRobotRig")).Length, Is.EqualTo(1));
    }

    [Test]
    public void ToolReplacementKeepsContoursSelectedAndAppliesOnlyToTheNewLiveVisuals()
    {
        Component live = Rig(false, "none"), ghost = Rig(true, "none");
        Dictionary<MeshRenderer, Appearance> ghostOriginals = Capture(ghost);
        Static("SetLiveRobot", live);
        Static("SetEnabled", true);

        Assert.That(Call(live, "ApplyModel", RosRobotModelCatalog.Load().Find("kenos_sl")), Is.EqualTo(true));

        Assert.That(Enabled, Is.True);
        Assert.That(Draws(), Is.Not.Empty);
        foreach (MeshRenderer renderer in Renderers(live)) Assert.That(renderer.forceRenderingOff, Is.True);
        AssertUnchanged(ghostOriginals);
        Static("SetEnabled", false);
        foreach (MeshRenderer renderer in Renderers(live)) Assert.That(renderer.forceRenderingOff, Is.False);
    }

    [Test]
    public void ReplacingOrClearingThePlacedRobotRestoresItsExactPreviousDrawFlags()
    {
        Component first = Rig(false, "kenos"), second = Rig(false, "kenos_sl");
        Dictionary<MeshRenderer, Appearance> firstOriginals = Capture(first), secondOriginals = Capture(second);
        Static("SetLiveRobot", first);
        Static("SetEnabled", true);

        Static("SetLiveRobot", second);

        Assert.That(Enabled, Is.True);
        AssertUnchanged(firstOriginals);
        foreach (MeshRenderer renderer in secondOriginals.Keys) Assert.That(renderer.forceRenderingOff, Is.True);
        Static("SetLiveRobot", new object[] { null });
        AssertUnchanged(secondOriginals);
        Assert.That(Draws(), Is.Empty);
        Assert.That(Enabled, Is.True, "A new placement should retain the user's selected representation.");
    }

    [Test]
    public void DisableAndReenableRestoreAndReapplyPresentationWithoutChangingTheOption()
    {
        Component live = Rig(false, "kenos");
        Dictionary<MeshRenderer, Appearance> originals = Capture(live);
        Static("SetLiveRobot", live);
        Static("SetEnabled", true);
        live.gameObject.SetActive(false);
        Static("OnRigDisabled", live); // EditMode fixtures deliberately drive the player lifecycle.
        Assert.That(Draws(), Is.Empty);
        AssertUnchanged(originals);
        live.gameObject.SetActive(true);
        Static("OnRigEnabled", live);
        Assert.That(Enabled, Is.True);
        Assert.That(Draws(), Is.Not.Empty);
        foreach (MeshRenderer renderer in originals.Keys) Assert.That(renderer.forceRenderingOff, Is.True);
    }

    [Test]
    public void ExecutionRetainsTheExistingTransparencyPolicyForContours()
    {
        Component live = Rig(false, "kenos");
        Static("SetLiveRobot", live);
        Static("SetEnabled", true);
        Color normal = (Color)Property("LineColor");
        Busy(true);
        Assert.That(((Color)Property("LineColor")).a, Is.EqualTo(0.03f).Within(1e-6f));
        Assert.That(Draws(), Is.Not.Empty);
        Busy(false);
        Assert.That((Color)Property("LineColor"), Is.EqualTo(normal));
        Assert.That(Enabled, Is.True);
        Assert.That((float)Property("LineWidthPixels"), Is.GreaterThan(0f));
    }

    [Test]
    public void DestructionRestoresFlagsAndReleasesSourceWithoutChangingGhosts()
    {
        Component live = Rig(false, "kenos"), ghost = Rig(true, "kenos");
        Dictionary<MeshRenderer, Appearance> originals = Capture(live), ghostOriginals = Capture(ghost);
        Static("SetLiveRobot", live);
        Static("SetEnabled", true);
        Static("OnRigDestroyed", live);
        AssertUnchanged(originals);
        AssertUnchanged(ghostOriginals);
        UnityEngine.Object.DestroyImmediate(live.gameObject);
        Assert.DoesNotThrow(() => Draws());
        Assert.That(Draws(), Is.Empty);
    }

    [Test]
    public void PlacementRegistersTheLiveRobotAndTogglingDoesNotInvalidateCalibration()
    {
        Component live = Rig(false, "kenos");
        Component coordinates = New("Contour-test coordinates").AddComponent(RuntimeType("RobotCoordinateService"));
        Component state = New("Contour-test placement state").AddComponent(RuntimeType("RobotStateService"));
        Transform tracking = New("Contour-test tracking").transform;
        Static("SetEnabled", true); // Selection before the first robot placement is supported.
        Pose pose = new Pose(live.transform.position, live.transform.rotation);
        Call(state, "InitializeRobot", pose, live.gameObject, tracking);
        Assert.That((bool)state.GetType().GetProperty("IsInitialized").GetValue(state), Is.True);
        int revision = (int)coordinates.GetType().GetProperty("CalibrationRevision").GetValue(coordinates);
        Assert.That(Draws(), Is.Not.Empty);

        Static("SetEnabled", false);
        Static("SetEnabled", true);

        Assert.That((int)coordinates.GetType().GetProperty("CalibrationRevision").GetValue(coordinates), Is.EqualTo(revision));
        Assert.That((bool)coordinates.GetType().GetProperty("IsInitialized").GetValue(coordinates), Is.True);
    }

    [Test]
    public void MissingOrWithdrawnRendererFallsBackToNormalFillWhileKeepingTheUserSelection()
    {
        Component live = Rig(false, "kenos");
        Dictionary<MeshRenderer, Appearance> originals = Capture(live);
        Static("SetRenderingAvailable", _renderingProvider, false);
        Static("SetLiveRobot", live);
        Static("SetEnabled", true);
        Assert.That((bool)Property("IsRenderingAvailable"), Is.False);
        Assert.That(Enabled, Is.True);
        Assert.That(Draws(), Is.Empty);
        AssertUnchanged(originals);

        Static("SetRenderingAvailable", _renderingProvider, true);
        Assert.That(Draws(), Is.Not.Empty);
        foreach (MeshRenderer renderer in originals.Keys) Assert.That(renderer.forceRenderingOff, Is.True);

        Static("SetRenderingAvailable", _renderingProvider, false);
        Assert.That(Draws(), Is.Empty);
        Assert.That(Enabled, Is.True);
        AssertUnchanged(originals);
    }

    [Test]
    public void DestroyedRenderingProviderRestoresNormalDrawsBeforeTheNextCameraRenders()
    {
        Component live = Rig(false, "kenos");
        Dictionary<MeshRenderer, Appearance> originals = Capture(live);
        Static("SetLiveRobot", live);
        Static("SetEnabled", true);
        Assert.That(Draws(), Is.Not.Empty);
        UnityEngine.Object.DestroyImmediate(_renderingProvider);

        RuntimeType("RobotContourDisplay").GetMethod("OnBeginCameraRendering", BindingFlags.NonPublic | BindingFlags.Static)
            .Invoke(null, new object[] { default(UnityEngine.Rendering.ScriptableRenderContext), null });

        Assert.That((bool)Property("IsRenderingAvailable"), Is.False);
        Assert.That(Enabled, Is.True);
        AssertUnchanged(originals);
    }

    [Test]
    public void OneRendererWithdrawalDoesNotDisableAnotherAvailableProvider()
    {
        Component live = Rig(false, "kenos");
        GameObject other = New("Second contour renderer provider");
        Static("SetRenderingAvailable", other, true);
        Static("SetLiveRobot", live);
        Static("SetEnabled", true);
        Static("SetRenderingAvailable", _renderingProvider, false);
        Assert.That((bool)Property("IsRenderingAvailable"), Is.True);
        Assert.That(Draws(), Is.Not.Empty);
        Static("SetRenderingAvailable", other, false);
        Assert.That(Draws(), Is.Empty);
    }

    [Test]
    public void AnUnrelatedActiveProviderCannotHideTheRobotFromACameraWithoutItsContourFeature()
    {
        Component live = Rig(false, "kenos");
        Dictionary<MeshRenderer, Appearance> originals = Capture(live);
        Camera camera = New("Camera using a renderer without contours").AddComponent<Camera>();
        camera.enabled = false;
        Static("SetLiveRobot", live);
        Static("SetEnabled", true);
        Assert.That(Draws(), Is.Not.Empty, "Global provider state remains usable outside rendering.");
        CameraEvent("OnBeginCameraRendering", camera);
        try
        {
            // Create/AddRenderPasses on a different renderer cannot confirm this camera.
            Static("SetRenderingAvailable", _renderingProvider, true);
            Assert.That((bool)Property("IsRenderingAvailable"), Is.False);
            Assert.That(Enabled, Is.True);
            Assert.That(Draws(), Is.Empty);
            AssertUnchanged(originals);

            GameObject cameraProvider = New("The current camera's contour feature");
            Static("SetRenderingAvailable", cameraProvider, true);
            Static("SetCameraRenderingAvailable", cameraProvider, camera, true);
            Assert.That(Draws(), Is.Not.Empty);
            foreach (MeshRenderer renderer in originals.Keys) Assert.That(renderer.forceRenderingOff, Is.True);
            Static("SetCameraRenderingAvailable", cameraProvider, camera, false);
            Assert.That(Draws(), Is.Empty);
            AssertUnchanged(originals);
            Static("SetRenderingAvailable", cameraProvider, false);
        }
        finally { CameraEvent("OnEndCameraRendering", camera); }
        Assert.That(Draws(), Is.Not.Empty, "The camera scope must not replace global feature availability.");
    }

    [Test]
    public void NestedCameraRenderingRestoresTheEnclosingCameraConfirmation()
    {
        Component live = Rig(false, "kenos");
        Dictionary<MeshRenderer, Appearance> originals = Capture(live);
        Camera outer = New("Outer supported camera").AddComponent<Camera>();
        Camera inner = New("Nested unsupported camera").AddComponent<Camera>();
        outer.enabled = inner.enabled = false;
        Static("SetLiveRobot", live);
        Static("SetEnabled", true);
        CameraEvent("OnBeginCameraRendering", outer);
        try
        {
            Static("SetCameraRenderingAvailable", _renderingProvider, outer, true);
            Assert.That(Draws(), Is.Not.Empty);
            CameraEvent("OnBeginCameraRendering", inner);
            try
            {
                Static("SetCameraRenderingAvailable", _renderingProvider, outer, true);
                Assert.That(Draws(), Is.Empty, "Another camera's confirmation cannot authorize this camera.");
                AssertUnchanged(originals);
            }
            finally { CameraEvent("OnEndCameraRendering", inner); }
            Assert.That(Draws(), Is.Not.Empty);
            foreach (MeshRenderer renderer in originals.Keys) Assert.That(renderer.forceRenderingOff, Is.True);
        }
        finally { CameraEvent("OnEndCameraRendering", outer); }
        Assert.That((bool)Property("IsRenderingAvailable"), Is.True);
    }

    private Component Rig(bool ghost, string tool)
    {
        string path = "Assets/Prefabs/ur16e_kenos/ur16e_kenos" + (ghost ? "_transparent" : "") + ".prefab";
        GameObject robot = UnityEngine.Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>(path));
        _owned.Add(robot);
        robot.SetActive(false);
        Component rig = robot.GetComponent(RuntimeType("RosRobotRig")) ?? robot.AddComponent(RuntimeType("RosRobotRig"));
        rig.GetType().GetProperty("IsGhost").SetValue(rig, ghost);
        Assert.That(Call(rig, "ApplyModel", RosRobotModelCatalog.Load().Find(tool)), Is.EqualTo(true));
        robot.SetActive(true);
        return rig;
    }

    private static List<MeshRenderer> Renderers(Component rig)
    {
        var renderers = new List<MeshRenderer>();
        foreach (GameObject visual in (IEnumerable)rig.GetType().GetProperty("VisualRoots").GetValue(rig))
            if (visual != null) renderers.AddRange(visual.GetComponentsInChildren<MeshRenderer>(true));
        Assert.That(renderers, Is.Not.Empty);
        return renderers;
    }
    private static Dictionary<MeshRenderer, Appearance> Capture(Component rig)
    {
        var result = new Dictionary<MeshRenderer, Appearance>();
        foreach (MeshRenderer renderer in Renderers(rig)) result.Add(renderer, new Appearance
        { ForceOff = renderer.forceRenderingOff, Enabled = renderer.enabled, Materials = renderer.sharedMaterials });
        return result;
    }
    private static void AssertUnchanged(Dictionary<MeshRenderer, Appearance> before)
    {
        foreach (KeyValuePair<MeshRenderer, Appearance> entry in before)
        {
            Assert.That(entry.Key.forceRenderingOff, Is.EqualTo(entry.Value.ForceOff));
            Assert.That(entry.Key.enabled, Is.EqualTo(entry.Value.Enabled));
            AssertMaterials(entry.Key, entry.Value.Materials);
        }
    }
    private static void AssertMaterials(Renderer renderer, Material[] original)
    {
        Material[] actual = renderer.sharedMaterials;
        Assert.That(actual.Length, Is.EqualTo(original.Length));
        for (int i = 0; i < actual.Length; i++) Assert.That(actual[i], Is.SameAs(original[i]));
    }
    private sealed class Appearance { public bool ForceOff; public bool Enabled; public Material[] Materials; }
    private GameObject New(string name) { var value = new GameObject(name); _owned.Add(value); return value; }
    private void Busy(bool value) => RuntimeType("RobotExecutionState").GetMethod("SetBusy").Invoke(null, new object[] { _executionOwner, value });
    private static bool Enabled => (bool)Property("Enabled");
    private static object Property(string name) => RuntimeType("RobotContourDisplay").GetProperty(name).GetValue(null);
    private static object Static(string name, params object[] values) => RuntimeType("RobotContourDisplay").GetMethod(name).Invoke(null, values);
    private static object Call(object owner, string name, params object[] values) => owner.GetType().GetMethod(name,
        Array.ConvertAll(values, value => value.GetType())).Invoke(owner, values);
    private static Type RuntimeType(string name) => Type.GetType(name + ", Assembly-CSharp", true);
    private static void CameraEvent(string name, Camera camera) => RuntimeType("RobotContourDisplay")
        .GetMethod(name, BindingFlags.NonPublic | BindingFlags.Static)
        .Invoke(null, new object[] { default(UnityEngine.Rendering.ScriptableRenderContext), camera });
    private static IList Draws()
    {
        Type listType = typeof(List<>).MakeGenericType(RuntimeType("RobotContourDraw"));
        var draws = (IList)Activator.CreateInstance(listType);
        Static("TryGetDraws", draws);
        return draws;
    }
}
