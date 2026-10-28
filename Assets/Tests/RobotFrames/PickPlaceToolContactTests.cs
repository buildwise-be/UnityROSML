using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.TestTools;

/// <summary>Actual capture/export/depth workflow without starting XR input or ROS transport.</summary>
public sealed class PickPlaceToolContactTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private readonly List<GameObject> _previousMainCameras = new List<GameObject>();
    private Component _controller, _coordinates, _plans;
    private Transform _base, _tracking;

    [SetUp]
    public void SetupActualComponentsWithoutTransport()
    {
        _coordinates = Add("Contact coordinates", "RobotCoordinateService");
        Add("Contact robot readiness", "RobotStateService");
        _tracking = New("Contact XR tracking").transform;
        _tracking.SetPositionAndRotation(new Vector3(1.2f, 0.3f, -0.7f), Quaternion.Euler(0, 27, 0));
        ArticulationBody body = New("base_link").AddComponent<ArticulationBody>();
        _base = body.transform;
        _base.SetPositionAndRotation(new Vector3(0.6f, -0.1f, 1.4f), Quaternion.Euler(14, -33, 7));
        Call(_coordinates, "Initialize", body, _tracking);
        Component tools = Add("Matched contact tool", "ToolSelectionController");
        if (Field(tools, "_catalog").GetValue(tools) == null) Private(tools, "Awake");
        string description = Resources.Load<TextAsset>("RobotModels/kenos_sl.urdf").text;
        foreach (string service in new[] { "/robot_state_publisher/get_parameters", "/move_group/get_parameters", "/mtc_test_task_node/get_parameters" })
            Assert.That(Call(tools, "InspectDescription", service, description), Is.EqualTo(true));
        _plans = Add("Contact MTC review", "MTCPlansManager");
        if (Field(_plans, "OnSuccessfulPlan").GetValue(_plans) == null) Private(_plans, "Awake");
        _controller = Add("Contact targets", "PickPlaceController");
        Set(_controller, "_plansManager", _plans);
    }

    [TearDown]
    public void Cleanup()
    {
        TypeOf("TargetPoseContextMenu").GetMethod("CloseAll").Invoke(null, null);
        for (int index = _owned.Count - 1; index >= 0; index--)
            if (_owned[index] != null) UnityEngine.Object.DestroyImmediate(_owned[index]);
        _owned.Clear();
        foreach (GameObject previous in _previousMainCameras)
            if (previous != null) previous.tag = "MainCamera";
        _previousMainCameras.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty,
            "Tool-contact editing must never create a robot transport.");
    }

    [Test]
    public void DisplayedBallAndArrowExportTheExactContactPointAndNormal()
    {
        Pose pointer = new Pose(new Vector3(0.4f, 0.8f, -0.2f), Quaternion.Euler(27, -42, 19));
        Pose contact = RobotFrameMath.PointerToTcpPose(pointer);
        Capture(contact);
        GameObject pick = Point(true);
        AssertPose(pick.transform, contact);
        Component marker = pick.GetComponent(TypeOf("NormalTargetMarker"));
        Assert.That(marker, Is.Not.Null);
        Assert.That(pick.GetComponent(TypeOf("ToolPoseVisual")), Is.Null);
        Assert.That(Vector3.Distance(Get<Vector3>(marker, "DirectionWorld"), contact.rotation * Vector3.up), Is.LessThan(0.000001f));
        Assert.That(Vector3.Distance(Get<Vector3>(marker, "ArrowTipWorld"), contact.position + contact.rotation * Vector3.up * 0.15f), Is.LessThan(0.000001f));
        GameObject markerGeometry = (GameObject)Field(marker, "_geometry").GetValue(marker);
        Assert.That(markerGeometry.GetComponentsInChildren<Collider>(true), Is.Empty);
        Assert.That(pick.GetComponentsInChildren<SphereCollider>(), Has.Length.EqualTo(1));
        Assert.That(pick.GetComponentsInChildren<BoxCollider>(), Is.Empty);
        Assert.That(pick.GetComponentsInChildren<Transform>().Any(t => t.name.StartsWith("Tilt arrow") || t.name.StartsWith("Twist arrow")), Is.False);
        Assert.That(pick.GetComponentsInChildren<ArticulationBody>(true), Is.Empty);
        object message = Export();
        object pickPose = Field(message, "pick_pose").GetValue(message);
        AssertExport(pickPose, contact);
        object header = Field(pickPose, "header").GetValue(pickPose);
        Assert.That(Field(header, "frame_id").GetValue(header), Is.EqualTo("ur_base_link"));
        Assert.That(TypeOf("PickPlaceController").GetField("ContactPlanTopic").GetRawConstantValue(),
            Is.EqualTo("/pick_place_tcp_target"));
    }

    [Test]
    public void LegacyPlanePrefabsAreNeverInstantiatedForContactTargets()
    {
        GameObject legacy = GameObject.CreatePrimitive(PrimitiveType.Plane);
        legacy.name = "Legacy surface plane"; legacy.SetActive(false); _owned.Add(legacy);
        Set(_controller, "_pickPointPrefab", legacy);
        Set(_controller, "_placePointPrefab", legacy);
        Capture(new Pose(new Vector3(0.3f, 0.7f, 0.2f), Quaternion.Euler(12, 8, -9)));
        foreach (GameObject target in new[] { Point(true), Point(false) })
            Assert.That(target.GetComponentsInChildren<Transform>(true).Any(t => t.name.StartsWith("Legacy surface plane")), Is.False);
        Assert.That(legacy.activeSelf, Is.False);
    }

    [Test]
    public void ActualControllerKeepsBallAndArrowVisibleWhenLabelsFaceTheViewerAndTargetsAreRecaptured()
    {
        Camera viewer = CreateViewer();
        Pose captured = new Pose(new Vector3(0.3f, 0.7f, 0.2f), Quaternion.Euler(22f, -31f, 17f));
        Capture(captured);
        GameObject originalPick = Point(true);
        foreach (GameObject target in new[] { Point(true), Point(false) })
        {
            Pose before = new Pose(target.transform.position, target.transform.rotation);
            Vector3 scale = target.transform.lossyScale;
            MeshRenderer[] geometry = ContactGeometry(target);
            Bounds[] bounds = geometry.Select(renderer => renderer.bounds).ToArray();
            AssertContactGeometryVisible(target);
            viewer.transform.SetPositionAndRotation(new Vector3(-1.2f, 1.7f, 0.4f), Quaternion.Euler(12f, 65f, 9f));
            Component label = target.GetComponentInChildren(TypeOf("TargetNameLabel"));
            Assert.That(label, Is.Not.Null);
            Call(label, "Refresh", viewer);
            AssertPose(target.transform, before);
            Assert.That(target.transform.lossyScale, Is.EqualTo(scale));
            for (int index = 0; index < geometry.Length; index++)
            {
                Assert.That(Vector3.Distance(geometry[index].bounds.center, bounds[index].center), Is.LessThan(0.000001f));
                Assert.That(Vector3.Distance(geometry[index].bounds.size, bounds[index].size), Is.LessThan(0.000001f));
            }
        }

        Call(_controller, "SetTilePlanningEnabled", false);
        foreach (GameObject target in new[] { Point(true), Point(false) })
            Assert.That(ContactGeometry(target).Any(renderer => renderer.gameObject.activeInHierarchy), Is.False);
        Call(_controller, "SetTilePlanningEnabled", true);
        foreach (GameObject target in new[] { Point(true), Point(false) }) AssertContactGeometryVisible(target);

        Pose recaptured = new Pose(new Vector3(-0.2f, 1.1f, 0.4f), Quaternion.Euler(-29f, 52f, 21f));
        Call(_controller, "SetPickPlace", recaptured.position, recaptured.rotation);
        Assert.That(Point(true), Is.SameAs(originalPick));
        AssertPose(Point(true).transform, recaptured);
        AssertContactGeometryVisible(Point(true));
        Assert.That(Point(true).GetComponentsInChildren(TypeOf("TargetNameLabel"), true), Has.Length.EqualTo(1));
        AssertExport(Field(Export(), "pick_pose").GetValue(Export()), recaptured);
    }

    [Test]
    public void RecapturingALabelOnlyReferenceRebuildsTheContactGeometry()
    {
        GameObject partial = New("Interrupted pick capture with label only");
        TypeOf("TargetNameLabel").GetMethod("Attach").Invoke(null,
            new object[] { partial.transform, "Pick", Color.green });
        Set(_controller, "_pickPointInstance", partial);
        Pose contact = new Pose(new Vector3(0.25f, 0.65f, 0.4f), Quaternion.Euler(18f, 43f, -21f));
        Capture(contact);
        Assert.That(Point(true), Is.SameAs(partial));
        AssertContactGeometryVisible(partial);
        AssertPose(partial.transform, contact);
        Assert.That(partial.GetComponentsInChildren(TypeOf("TargetNameLabel"), true), Has.Length.EqualTo(1));
        AssertExport(Field(Export(), "pick_pose").GetValue(Export()), contact);
    }

    [Test]
    public void RecapturingRestoresHiddenBallAndArrowGeometryWithoutChangingTheRequestedPose()
    {
        Capture(new Pose(new Vector3(0.2f, 0.7f, -0.1f), Quaternion.Euler(12f, -18f, 9f)));
        GameObject pick = Point(true);
        foreach (MeshRenderer renderer in ContactGeometry(pick)) renderer.enabled = false;
        Component marker = pick.GetComponent(TypeOf("NormalTargetMarker"));
        GameObject geometry = (GameObject)Field(marker, "_geometry").GetValue(marker);
        geometry.SetActive(false);
        Pose next = new Pose(new Vector3(-0.3f, 0.6f, 0.2f), Quaternion.Euler(42f, 26f, -31f));
        Call(_controller, "SetPickPlace", next.position, next.rotation);
        Assert.That(Point(true), Is.SameAs(pick));
        AssertContactGeometryVisible(pick);
        AssertPose(pick.transform, next);
        Assert.That(pick.GetComponentsInChildren(TypeOf("TargetNameLabel"), true), Has.Length.EqualTo(1));
        AssertExport(Field(Export(), "pick_pose").GetValue(Export()), next);
    }

    [Test]
    public void CapturedAndLiveMaterialsUseBundledUrpVariantsWithoutRecoloringTheirTemplates()
    {
        Material opaque = Resources.Load<Material>("RobotTargetMarkers/Opaque");
        Material transparent = Resources.Load<Material>("RobotTargetMarkers/Transparent");
        Assert.That(opaque, Is.Not.Null);
        Assert.That(transparent, Is.Not.Null);
        Color opaqueBefore = opaque.GetColor("_BaseColor");
        Color transparentBefore = transparent.GetColor("_BaseColor");
        Assert.That(opaque.shader.name, Is.EqualTo("Universal Render Pipeline/Unlit"));
        Assert.That(transparent.shader, Is.SameAs(opaque.shader));
        Capture(new Pose(new Vector3(0.3f, 0.6f, 0.2f), Quaternion.Euler(12f, 31f, -19f)));
        foreach (bool pick in new[] { true, false })
            foreach (MeshRenderer renderer in ContactGeometry(Point(pick)))
            {
                Material material = renderer.sharedMaterial;
                Assert.That(material, Is.Not.SameAs(opaque));
                Assert.That(material.shader, Is.SameAs(opaque.shader));
                Assert.That(material.GetFloat("_Surface"), Is.EqualTo(0f));
                Color expected = pick ? Color.green : new Color(1f, 0.6f, 0.05f);
                Assert.That(Vector4.Distance(material.GetColor("_BaseColor"), expected), Is.LessThan(0.00001f));
            }
        Call(_controller, "SetPointerPose", new Pose(new Vector3(0.1f, 0.8f, 0.3f), Quaternion.identity), _tracking);
        GameObject live = (GameObject)Field(_controller, "_livePointInstance").GetValue(_controller);
        foreach (MeshRenderer renderer in live.GetComponentsInChildren<MeshRenderer>())
        {
            Material material = renderer.sharedMaterial;
            Assert.That(material, Is.Not.SameAs(transparent));
            Assert.That(material.shader, Is.SameAs(transparent.shader));
            Assert.That(material.GetFloat("_Surface"), Is.EqualTo(1f));
            Assert.That(material.renderQueue, Is.GreaterThanOrEqualTo((int)RenderQueue.Transparent));
        }
        Assert.That(opaque.GetColor("_BaseColor"), Is.EqualTo(opaqueBefore));
        Assert.That(transparent.GetColor("_BaseColor"), Is.EqualTo(transparentBefore));
    }

    /// <summary>Render the controller-created captured references and their labels using the real mobile URP setup.</summary>
    public static void RenderCapturedMarkerPreview()
    {
        if (SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null)
            throw new InvalidOperationException("A graphics device is required for the actual-controller marker preview.");
        Assert.That(GraphicsSettings.currentRenderPipeline, Is.Not.Null,
            "Prepare this isolated project with the application's Mobile URP settings.");
        Assert.That(AssetDatabase.GetAssetPath(GraphicsSettings.currentRenderPipeline), Does.EndWith("Mobile_RPAsset.asset"));
        PickPlaceToolContactTests preview = new PickPlaceToolContactTests();
        RenderTexture image = null;
        RenderTexture previousActive = RenderTexture.active;
        Texture2D pixels = null;
        Camera camera = null;
        try
        {
            preview.SetupActualComponentsWithoutTransport();
            preview._tracking.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            camera = preview.CreateViewer();
            camera.transform.SetPositionAndRotation(new Vector3(0f, 0.06f, -1f), Quaternion.identity);
            camera.orthographic = true;
            camera.orthographicSize = 0.23f;
            camera.aspect = 1.5f;
            camera.nearClipPlane = 0.01f;
            camera.farClipPlane = 3f;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.035f, 0.045f, 0.065f, 1f);
            Call(preview._controller, "SetPickPlace", new Vector3(-0.18f, -0.04f, 0f), Quaternion.Euler(0f, 0f, 30f));
            Call(preview._controller, "SetPlacePlace", new Vector3(0.13f, -0.04f, 0f), Quaternion.Euler(0f, 0f, -20f));
            Type fontType = TypeOf("TMPro.TMP_FontAsset");
            UnityEngine.Object font = Resources.Load("Fonts & Materials/LiberationSans SDF", fontType);
            if (font == null)
            {
                string guid = AssetDatabase.FindAssets("LiberationSans SDF t:TMP_FontAsset").FirstOrDefault();
                if (guid != null) font = AssetDatabase.LoadAssetAtPath(AssetDatabase.GUIDToAssetPath(guid), fontType);
            }
            foreach (GameObject target in new[] { preview.Point(true), preview.Point(false) })
            {
                AssertContactGeometryVisible(target);
                Component label = target.GetComponentInChildren(TypeOf("TargetNameLabel"));
                Call(label, "Refresh", camera);
                foreach (Component text in target.GetComponentsInChildren(TypeOf("TMPro.TMP_Text"), true))
                {
                    if (font != null) text.GetType().GetProperty("font").SetValue(text, font);
                    text.GetType().GetMethod("ForceMeshUpdate").Invoke(text, new object[] { true, true });
                }
            }
            image = new RenderTexture(1200, 800, 24, RenderTextureFormat.ARGB32);
            pixels = new Texture2D(image.width, image.height, TextureFormat.RGB24, false);
            camera.targetTexture = image;
            Canvas.ForceUpdateCanvases();
            camera.Render();
            RenderTexture.active = image;
            pixels.ReadPixels(new Rect(0, 0, image.width, image.height), 0, 0);
            pixels.Apply();
            string path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "normal-controller-preview.png"));
            File.WriteAllBytes(path, pixels.EncodeToPNG());
            foreach (bool pick in new[] { true, false })
                foreach (MeshRenderer renderer in ContactGeometry(preview.Point(pick)))
                    AssertRenderedMarkerPart(camera, pixels, renderer, pick);
            Debug.Log("ActualControllerMarkerPreview: " + path);
        }
        finally
        {
            if (camera != null) camera.targetTexture = null;
            RenderTexture.active = previousActive;
            if (image != null) { image.Release(); UnityEngine.Object.DestroyImmediate(image); }
            if (pixels != null) UnityEngine.Object.DestroyImmediate(pixels);
            preview.Cleanup();
        }
    }

    [Test]
    public void DepthClickMovesExportedContactFiveMillimetresAlongToolZAndRevokesReview()
    {
        Pose contact = new Pose(new Vector3(0.2f, 0.6f, 0.8f), Quaternion.Euler(31, 24, -17));
        Capture(contact);
        Component menu = Point(true).GetComponent(TypeOf("TargetPoseContextMenu"));
        Assert.That(Call(_plans, "BeginPlanningRequest"), Is.EqualTo(true));
        Set(_controller, "_hasPlanningRequestBeenSent", true);
        Assert.That(Call(menu, "NudgeDepth", 1), Is.EqualTo(true));
        Pose shifted = new Pose(contact.position + contact.rotation * Vector3.up * 0.005f, contact.rotation);
        AssertPose(Point(true).transform, shifted);
        AssertExport(Field(Export(), "pick_pose").GetValue(Export()), shifted);
        Assert.That(Get<bool>(_plans, "IsPlanning") || Get<bool>(_plans, "CanExecute"), Is.False);
        Assert.That(Field(_controller, "_hasPlanningRequestBeenSent").GetValue(_controller), Is.EqualTo(false));
        Call(menu, "ResetDepth");
        AssertPose(Point(true).transform, contact);
    }

    [Test]
    public void RecapturingResetsDepthBaselineWithoutKeepingTheOldOffset()
    {
        Pose first = new Pose(new Vector3(0.2f, 0.4f, 0.1f), Quaternion.Euler(4, 25, 3));
        Capture(first);
        Component menu = Point(true).GetComponent(TypeOf("TargetPoseContextMenu"));
        Call(menu, "NudgeDepth", 1);
        Pose next = new Pose(new Vector3(-0.3f, 0.8f, 0.5f), Quaternion.Euler(42, -18, 7));
        Call(_controller, "SetPickPlace", next.position, next.rotation);
        Assert.That(Get<int>(menu, "DepthSteps"), Is.Zero);
        Call(menu, "NudgeDepth", -1);
        AssertPose(Point(true).transform, new Pose(next.position - next.rotation * Vector3.up * 0.005f, next.rotation));
        Call(menu, "ResetDepth");
        AssertPose(Point(true).transform, next);
    }

    [Test]
    public void InactiveModeHidesNormalAimAndRejectsCaptureAndExportWhileKeepingPreviousPoints()
    {
        Pose contact = new Pose(new Vector3(0.5f, 0.8f, -0.3f), Quaternion.Euler(18, 32, 7));
        Capture(contact);
        Call(_controller, "SetPointerPose", new Pose(new Vector3(0.1f, 0.7f, 0.3f), Quaternion.identity), _tracking);
        GameObject live = (GameObject)Field(_controller, "_livePointInstance").GetValue(_controller);
        Assert.That(live.activeSelf, Is.True);
        Assert.That(live.GetComponent(TypeOf("NormalTargetMarker")), Is.Not.Null);
        Assert.That(live.GetComponent(TypeOf("ToolPoseVisual")), Is.Null);
        Call(_controller, "SetTilePlanningEnabled", false);
        Call(_controller, "SetPickPlace", Vector3.one * 8f, Quaternion.identity);
        Assert.That(Point(true).activeSelf || Point(false).activeSelf || live.activeSelf, Is.False);
        AssertPose(Point(true).transform, contact);
        object[] args = { null };
        Assert.That(CallArgs(_controller, "TryGetContactPoses", args), Is.EqualTo(false));
        Call(_controller, "SetTilePlanningEnabled", true);
        Assert.That(Point(true).activeSelf && Point(false).activeSelf, Is.True);
        AssertExport(Field(Export(), "pick_pose").GetValue(Export()), contact);
    }

    [Test]
    public void DelayedCapabilityResponseCannotRearmAfterAContactChanges()
    {
        Capture(new Pose(new Vector3(0.4f, 0.7f, 0.1f), Quaternion.identity));
        Set(_controller, "_checkingCapability", true);
        Set(_controller, "_generation", 40);
        Set(_controller, "_requestedContactPoses", Export());
        Call(_controller, "SetPickPlace", new Vector3(0.5f, 0.7f, 0.1f), Quaternion.identity);
        Private(_controller, "OnCapabilityResponse", null, 40);
        Assert.That(Get<bool>(_controller, "IsPlanning"), Is.False);
        Assert.That(Get<bool>(_plans, "IsPlanning") || Get<bool>(_plans, "CanExecute"), Is.False);
        Assert.That(Field(_controller, "_hasPlanningRequestBeenSent").GetValue(_controller), Is.EqualTo(false));
    }

    [TestCase("focus")]
    [TestCase("pause")]
    [TestCase("tracking")]
    [TestCase("disabled")]
    public void InterruptedContextBumperDoesNotBlockTheNextCapture(string interruption)
    {
        CreateViewer();
        Component input = Add("Contact input lifecycle", "URInputManager");
        Set(input, "_pickPlaceController", _controller);
        Set(input, "_trackingSpace", _tracking);
        Call(input, "UseTilePickPlace");
        Pose contact = new Pose(new Vector3(0.4f, 0.8f, 0.3f), Quaternion.Euler(12, 28, -9));
        Capture(contact);
        Call(_controller, "SetPointerPose", contact, _tracking);
        GameObject live = (GameObject)Field(_controller, "_livePointInstance").GetValue(_controller);
        Assert.That(live.activeSelf, Is.True);
        Component menu = Point(true).GetComponent(TypeOf("TargetPoseContextMenu"));
        Assert.That(Call(menu, "Open"), Is.EqualTo(true));

        MethodInfo bumper = input.GetType().GetMethod("HandleOnBumper", BindingFlags.Instance | BindingFlags.NonPublic);
        bumper.Invoke(input, new[] { Activator.CreateInstance(bumper.GetParameters()[0].ParameterType) });
        Assert.That(float.IsPositiveInfinity((float)Field(input, "_contextBumperConsumedUntil").GetValue(input)), Is.True,
            "A context bumper press consumes capture until release or an input interruption.");
        Set(input, "_robotPlacementRequested", true);

        switch (interruption)
        {
            case "focus": Private(input, "OnApplicationFocus", false); break;
            case "pause": Private(input, "OnApplicationPause", true); break;
            case "tracking": Private(input, "Update"); break;
            case "disabled": Private(input, "OnDisable"); break;
        }

        Assert.That(Field(input, "_contextBumperConsumedUntil").GetValue(input), Is.EqualTo(0f));
        Assert.That(Field(input, "_robotPlacementRequested").GetValue(input), Is.EqualTo(false));
        Assert.That((bool)TypeOf("TargetPoseContextMenu").GetProperty("IsOpen").GetValue(null), Is.False);
        Assert.That(live.activeSelf, Is.False);
        // No XR device is supplied: reaching its tracking check proves the stale context press no longer blocks capture.
        LogAssert.Expect(LogType.Warning, "Cannot capture a target without a tracked controller and a valid XR tracking frame.");
        Private(input, "Capture", true);
        AssertPose(Point(true).transform, contact);
    }

    private void Capture(Pose contact)
    {
        Call(_controller, "SetPickPlace", contact.position, contact.rotation);
        Call(_controller, "SetPlacePlace", contact.position + Vector3.right * 0.1f, contact.rotation);
    }
    private object Export()
    {
        object[] args = { null };
        Assert.That(CallArgs(_controller, "TryGetContactPoses", args), Is.EqualTo(true));
        return args[0];
    }
    private GameObject Point(bool pick) => (GameObject)Field(_controller, pick ? "_pickPointInstance" : "_placePointInstance").GetValue(_controller);
    private Camera CreateViewer()
    {
        foreach (Camera previous in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (previous.CompareTag("MainCamera")) { _previousMainCameras.Add(previous.gameObject); previous.tag = "Untagged"; }
        GameObject viewer = New("Contact target viewer");
        viewer.tag = "MainCamera";
        viewer.transform.position = new Vector3(0, 1, -2);
        return viewer.AddComponent<Camera>();
    }
    private static MeshRenderer[] ContactGeometry(GameObject target)
    {
        MeshRenderer[] all = target.GetComponentsInChildren<MeshRenderer>(true);
        return new[]
        {
            all.Single(renderer => renderer.name.StartsWith("Free rotation ball", StringComparison.Ordinal)),
            all.Single(renderer => renderer.name == "Tool +Z normal shaft"),
            all.Single(renderer => renderer.name == "Tool +Z normal arrowhead")
        };
    }
    private static void AssertContactGeometryVisible(GameObject target)
    {
        Material retained = Resources.Load<Material>("RobotTargetMarkers/Opaque");
        Assert.That(retained, Is.Not.Null, "The bundled material must retain the player's opaque marker shader variant.");
        foreach (MeshRenderer renderer in ContactGeometry(target))
        {
            Assert.That(renderer.enabled && renderer.gameObject.activeInHierarchy, Is.True, renderer.name);
            Vector3 size = renderer.bounds.size;
            Assert.That(size.x > 0.001f && size.y > 0.001f && size.z > 0.001f &&
                size.x < 0.3f && size.y < 0.3f && size.z < 0.3f, Is.True,
                renderer.name + " must retain visible dimensions in metres after label setup.");
            Assert.That(renderer.sharedMaterial, Is.Not.Null, renderer.name);
            Assert.That(renderer.sharedMaterial.shader, Is.Not.Null, renderer.name);
            Assert.That(renderer.sharedMaterial.shader.name, Is.EqualTo("Universal Render Pipeline/Unlit"), renderer.name);
            Assert.That(renderer.sharedMaterial.shader, Is.SameAs(retained.shader), renderer.name);
            Assert.That(renderer.sharedMaterial.renderQueue, Is.LessThan((int)RenderQueue.Transparent), renderer.name);
            if (SystemInfo.graphicsDeviceType != GraphicsDeviceType.Null)
                Assert.That(renderer.sharedMaterial.shader.isSupported, Is.True, renderer.name);
        }
    }
    private static void AssertRenderedMarkerPart(Camera camera, Texture2D image, MeshRenderer renderer, bool pick)
    {
        Vector3 viewport = camera.WorldToViewportPoint(renderer.bounds.center);
        Assert.That(viewport.z > 0f && viewport.x > 0f && viewport.x < 1f && viewport.y > 0f && viewport.y < 1f,
            Is.True, renderer.name + " must be within the preview camera.");
        int centerX = Mathf.RoundToInt(viewport.x * (image.width - 1));
        int centerY = Mathf.RoundToInt(viewport.y * (image.height - 1));
        int colored = 0;
        for (int y = centerY - 3; y <= centerY + 3; y++)
            for (int x = centerX - 3; x <= centerX + 3; x++)
            {
                Color color = image.GetPixel(x, y);
                if (pick ? color.g > 0.2f && color.g > color.r * 1.5f && color.g > color.b * 1.5f
                    : color.r > 0.3f && color.r > color.g * 1.1f && color.g > color.b * 1.5f) colored++;
            }
        Assert.That(colored, Is.GreaterThan(3), renderer.name + " must render colored pixels, independently of the name label.");
        Debug.Log("ActualControllerMarkerPixels: " + (pick ? "Pick/" : "Place/") + renderer.name + "=" + colored);
    }
    private void AssertExport(object stamped, Pose expected)
    {
        Pose local = RobotFrameMath.RelativeTo(new Pose(_base.position, _base.rotation), expected);
        object pose = Field(stamped, "pose").GetValue(stamped);
        object position = Field(pose, "position").GetValue(pose);
        Assert.That(Number(position, "x"), Is.EqualTo(local.position.z).Within(0.000001));
        Assert.That(Number(position, "y"), Is.EqualTo(-local.position.x).Within(0.000001));
        Assert.That(Number(position, "z"), Is.EqualTo(local.position.y).Within(0.000001));
        object rotation = Field(pose, "orientation").GetValue(pose);
        double x = Number(rotation, "x"), y = Number(rotation, "y"), z = Number(rotation, "z"), w = Number(rotation, "w");
        Vector3 normalUnity = local.rotation * Vector3.up;
        Assert.That(2*(x*z+w*y), Is.EqualTo(normalUnity.z).Within(0.000002));
        Assert.That(2*(y*z-w*x), Is.EqualTo(-normalUnity.x).Within(0.000002));
        Assert.That(1-2*(x*x+y*y), Is.EqualTo(normalUnity.y).Within(0.000002));
    }
    private static double Number(object value, string name) => Convert.ToDouble(Field(value, name).GetValue(value));
    private GameObject New(string name) { GameObject value = new GameObject(name); _owned.Add(value); return value; }
    private Component Add(string name, string type) { Component value = New(name).AddComponent(TypeOf(type)); ((Behaviour)value).enabled = false; return value; }
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType(name)).First(t => t != null);
    private static FieldInfo Field(object value, string name) => value.GetType().GetField(name, BindingFlags.Instance|BindingFlags.Static|BindingFlags.Public|BindingFlags.NonPublic);
    private static void Set(object value, string name, object data) => Field(value, name).SetValue(value, data);
    private static T Get<T>(object value, string name) => (T)value.GetType().GetProperty(name).GetValue(value);
    private static object Call(object value, string name, params object[] args) => CallArgs(value, name, args);
    private static object CallArgs(object value, string name, object[] args) => value.GetType().GetMethods().Single(m => m.Name == name && m.GetParameters().Length == args.Length).Invoke(value,args);
    private static object Private(object value, string name, params object[] args) => value.GetType().GetMethod(name,BindingFlags.Instance|BindingFlags.NonPublic).Invoke(value,args);
    private static void AssertPose(Transform actual, Pose expected)
    {
        Assert.That(Vector3.Distance(actual.position,expected.position), Is.LessThan(0.000001f));
        Assert.That(Quaternion.Angle(actual.rotation,expected.rotation), Is.LessThan(0.001f));
    }
}
