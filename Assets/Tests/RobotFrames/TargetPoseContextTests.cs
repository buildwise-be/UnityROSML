using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEditor;

public sealed class TargetPoseContextTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private readonly List<GameObject> _previousCameras = new List<GameObject>();
    private Component _menu;
    private Transform _target, _tracking;
    private int _changes;

    /// <summary>Graphics-enabled preview of the actual contact ball, normal arrow and menu, without ROS.</summary>
    public static void RenderToolContactPreview()
    {
        if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null)
            throw new InvalidOperationException("A graphics device is required for the tool-context preview.");
        TargetPoseContextTests preview = new TargetPoseContextTests();
        RenderTexture image = null;
        Texture2D pixels = null;
        Camera camera = null;
        try
        {
            preview.CreateLocalToolReferenceWithoutTransport();
            preview._tracking.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            preview._target.SetPositionAndRotation(new Vector3(-0.15f, -0.03f, 0f), Quaternion.Euler(-50f, 0f, -20f));
            Call(preview._menu, "RebaseAtCurrentPose");
            Component marker = (Component)TypeOf("NormalTargetMarker").GetMethod("Attach").Invoke(null,
                new object[] { preview._target, Color.green, 1f });
            Call(marker, "SetBallVisible", false);
            Material ballMaterial = new Material(Shader.Find("Unlit/Color")) { color = Color.green };
            Component handles = (Component)TypeOf("TargetRotationHandles").GetMethod("Attach").Invoke(null,
                new object[] { preview._target, ballMaterial, 0.035f });
            UnityEngine.Object.DestroyImmediate(ballMaterial);
            Call(handles, "ConfigureNormalInteraction");
            camera = Camera.main;
            camera.transform.SetPositionAndRotation(new Vector3(0f, 0.10f, -1f), Quaternion.identity);
            camera.orthographic = true;
            camera.orthographicSize = 0.34f;
            camera.aspect = 4f / 3f;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.08f, 0.11f, 0.15f, 1f);
            Assert.That(Call(preview._menu, "Open"), Is.EqualTo(true));
            Call(preview._menu, "NudgeDepth", 1);
            Canvas canvas = (Canvas)Field(preview._menu, "_canvas").GetValue(preview._menu);
            Type fontType = TypeOf("TMPro.TMP_FontAsset");
            UnityEngine.Object font = Resources.Load("Fonts & Materials/LiberationSans SDF", fontType);
            if (font == null)
            {
                string guid = AssetDatabase.FindAssets("LiberationSans SDF t:TMP_FontAsset").FirstOrDefault();
                if (guid != null) font = AssetDatabase.LoadAssetAtPath(AssetDatabase.GUIDToAssetPath(guid), fontType);
            }
            foreach (Component text in canvas.GetComponentsInChildren(TypeOf("TMPro.TMP_Text"), true))
            {
                if (font != null) text.GetType().GetProperty("font").SetValue(text, font);
                text.GetType().GetMethod("ForceMeshUpdate").Invoke(text, new object[] { true, true });
            }
            image = new RenderTexture(1024, 768, 24, RenderTextureFormat.ARGB32);
            pixels = new Texture2D(1024, 768, TextureFormat.RGB24, false);
            camera.targetTexture = image;
            Canvas.ForceUpdateCanvases();
            camera.Render();
            RenderTexture.active = image;
            pixels.ReadPixels(new Rect(0f, 0f, 1024f, 768f), 0, 0);
            pixels.Apply();
            string path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "normal-contact-context.png"));
            File.WriteAllBytes(path, pixels.EncodeToPNG());
            Debug.Log("ToolContactPreview: " + path);
        }
        finally
        {
            if (camera != null) camera.targetTexture = null;
            RenderTexture.active = null;
            if (image != null) { image.Release(); UnityEngine.Object.DestroyImmediate(image); }
            if (pixels != null) UnityEngine.Object.DestroyImmediate(pixels);
            preview.RemoveAllDetachedPanelsAndInteractionState();
        }
    }

    [SetUp]
    public void CreateLocalToolReferenceWithoutTransport()
    {
        _changes = 0;
        foreach (Camera camera in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (camera.CompareTag("MainCamera")) { _previousCameras.Add(camera.gameObject); camera.tag = "Untagged"; }
        GameObject view = New("Context test viewer");
        view.tag = "MainCamera";
        view.AddComponent<Camera>();
        view.transform.position = new Vector3(0f, 1.6f, -0.7f);
        _tracking = New("Context tracking").transform;
        _tracking.SetPositionAndRotation(new Vector3(1f, 0.3f, -0.4f), Quaternion.Euler(0f, 37f, 0f));
        _target = New("Pick tool reference").transform;
        _target.SetParent(_tracking, false);
        _target.SetLocalPositionAndRotation(new Vector3(0.3f, 0.8f, 0.5f), Quaternion.Euler(32f, -18f, 44f));
        _menu = Attach(_target, true);
        Call(_menu, "RebaseAtCurrentPose");
    }

    [TearDown]
    public void RemoveAllDetachedPanelsAndInteractionState()
    {
        foreach (GameObject go in _owned)
            if (go != null)
                foreach (Component menu in go.GetComponents(TypeOf("TargetPoseContextMenu")))
                {
                    Canvas canvas = (Canvas)Field(menu, "_canvas").GetValue(menu);
                    if (canvas != null) UnityEngine.Object.DestroyImmediate(canvas.gameObject);
                    Field(menu, "_canvas").SetValue(menu, null);
                }
        for (int index = _owned.Count - 1; index >= 0; index--)
            if (_owned[index] != null) UnityEngine.Object.DestroyImmediate(_owned[index]);
        _owned.Clear();
        foreach (GameObject camera in _previousCameras) if (camera != null) camera.tag = "MainCamera";
        _previousCameras.Clear();
        Assert.That((bool)TypeOf("TargetPoseContextMenu").GetProperty("IsOpen").GetValue(null), Is.False);
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
    }

    [Test]
    public void DepthStepMovesExactlyFiveMillimetresAlongPhysicalToolZ()
    {
        Vector3 original = _target.position;
        Quaternion rotation = _target.rotation;
        Vector3 toolZ = RobotFrameMath.TcpPositiveZ(new Pose(original, rotation));
        Assert.That(Call(_menu, "NudgeDepth", 1), Is.EqualTo(true));
        Assert.That(Vector3.Distance(_target.position, original + toolZ * 0.005f), Is.LessThan(0.000001f));
        Assert.That(Quaternion.Angle(_target.rotation, rotation), Is.LessThan(0.0001f));
        Assert.That(_changes, Is.EqualTo(1));
        Assert.That(Get<int>(_menu, "DepthSteps"), Is.EqualTo(1));
    }

    [Test]
    public void NegativeStepRetractsAndOppositeStepsCancel()
    {
        Vector3 original = _target.position;
        Call(_menu, "NudgeDepth", -1);
        Assert.That(Vector3.Distance(_target.position, original - _target.up * 0.005f), Is.LessThan(0.000001f));
        Call(_menu, "NudgeDepth", 1);
        Assert.That(Vector3.Distance(_target.position, original), Is.LessThan(0.000001f));
        Assert.That(Get<int>(_menu, "DepthSteps"), Is.Zero);
    }

    [Test]
    public void ResetUndoesActualTranslationsWhenOrientationChangesBetweenSteps()
    {
        Vector3 baseline = _target.localPosition;
        Call(_menu, "NudgeDepth", 1);
        _target.localRotation *= Quaternion.Euler(50f, 13f, -8f);
        Call(_menu, "NudgeDepth", -1);
        Assert.That(Vector3.Distance(_target.localPosition, baseline), Is.GreaterThan(0.001f));
        Call(_menu, "ResetDepth");
        Assert.That(Vector3.Distance(_target.localPosition, baseline), Is.LessThan(0.000001f));
        Assert.That(_changes, Is.EqualTo(3));
    }

    [Test]
    public void RecaptureRebasesWithoutMovingTheNewContactPoint()
    {
        Call(_menu, "NudgeDepth", 1);
        Vector3 recaptured = new Vector3(-0.4f, 0.6f, 0.2f);
        _target.localPosition = recaptured;
        Call(_menu, "RebaseAtCurrentPose");
        Call(_menu, "ResetDepth");
        Assert.That(_target.localPosition, Is.EqualTo(recaptured));
        Assert.That(Get<int>(_menu, "DepthSteps"), Is.Zero);
        Call(_menu, "NudgeDepth", -1);
        Call(_menu, "ResetDepth");
        Assert.That(Vector3.Distance(_target.localPosition, recaptured), Is.LessThan(0.000001f));
    }

    [Test]
    public void TrackingOriginMovementPreservesThePhysicalDepthOffset()
    {
        Vector3 baseline = _target.localPosition;
        Call(_menu, "NudgeDepth", 1);
        _tracking.SetPositionAndRotation(new Vector3(-1f, 0.1f, 2f), Quaternion.Euler(0f, -73f, 0f));
        Call(_menu, "ResetDepth");
        Assert.That(Vector3.Distance(_target.localPosition, baseline), Is.LessThan(0.000001f));
    }

    [Test]
    public void ExactTcpContextDisallowsContactDepthAdjustments()
    {
        Component exact = Attach(New("TCP A").transform, false);
        Pose original = new Pose(exact.transform.position, exact.transform.rotation);
        Assert.That(Call(exact, "NudgeDepth", 1), Is.EqualTo(false));
        Assert.That(exact.transform.position, Is.EqualTo(original.position));
        Assert.That(_changes, Is.Zero);
    }

    [Test]
    public void HiddenTargetsAndCalibrationEditsRejectPositionChanges()
    {
        Vector3 original = _target.position;
        _target.gameObject.SetActive(false);
        Assert.That(Call(_menu, "NudgeDepth", 1), Is.EqualTo(false));
        _target.gameObject.SetActive(true);
        Component coordinates = New("Context calibration gate").AddComponent(TypeOf("RobotCoordinateService"));
        Call(coordinates, "BeginCalibrationEdit");
        Assert.That(Call(_menu, "NudgeDepth", 1), Is.EqualTo(false));
        Assert.That(_target.position, Is.EqualTo(original));
        Call(coordinates, "EndCalibrationEdit");
    }

    [Test]
    public void MenuStaysStillWhenItsButtonsMoveTheToolAndClosesOnDisable()
    {
        Assert.That(Call(_menu, "Open"), Is.EqualTo(true));
        Canvas canvas = (Canvas)Field(_menu, "_canvas").GetValue(_menu);
        Assert.That(canvas.transform.parent, Is.EqualTo(_tracking));
        Vector3 position = canvas.transform.position;
        Quaternion rotation = canvas.transform.rotation;
        Call(_menu, "NudgeDepth", 1);
        Assert.That(canvas.transform.position, Is.EqualTo(position));
        Assert.That(canvas.transform.rotation, Is.EqualTo(rotation));
        Assert.That((bool)TypeOf("TargetRotationHandles").GetProperty("IsHovered").GetValue(null), Is.True);
        _target.gameObject.SetActive(false);
        Private(_menu, "OnDisable"); // Runtime lifecycle is explicit in this EditMode fixture.
        Assert.That(canvas.gameObject.activeSelf, Is.False);
    }

    [Test]
    public void OnlyOneTargetContextCanBeOpenAtOnce()
    {
        Component second = Attach(New("Place tool").transform, true);
        Assert.That(Call(_menu, "Open"), Is.EqualTo(true));
        Assert.That(Call(second, "Open"), Is.EqualTo(true));
        Canvas firstCanvas = (Canvas)Field(_menu, "_canvas").GetValue(_menu);
        Canvas secondCanvas = (Canvas)Field(second, "_canvas").GetValue(second);
        Assert.That(firstCanvas.gameObject.activeSelf, Is.False);
        Assert.That(secondCanvas.gameObject.activeSelf, Is.True);
        Private(second, "OnApplicationFocus", false);
        Assert.That(secondCanvas.gameObject.activeSelf, Is.False);
    }

    [Test]
    public void ToolHandleCoversActualGeometryWithoutRenderingTheOldBall()
    {
        Component handles = (Component)TypeOf("TargetRotationHandles").GetMethod("Attach").Invoke(null,
            new object[] { _target, null, 0.035f });
        Bounds bounds = new Bounds(new Vector3(0.02f, -0.08f, 0.01f), new Vector3(0.13f, 0.16f, 0.25f));
        Call(handles, "ConfigureToolInteraction", bounds);
        BoxCollider box = _target.GetComponentsInChildren<BoxCollider>().Single();
        Assert.That(box.center, Is.EqualTo(bounds.center));
        Assert.That(box.size, Is.EqualTo(bounds.size));
        Assert.That(box.transform.localScale, Is.EqualTo(Vector3.one));
        Assert.That(box.GetComponent<MeshRenderer>().enabled, Is.False);
        Assert.That(box.GetComponent<SphereCollider>().enabled, Is.False);
        Assert.That(_target.localPosition, Is.EqualTo(new Vector3(0.3f, 0.8f, 0.5f)));
    }

    private Component Attach(Transform target, bool depth) => (Component)TypeOf("TargetPoseContextMenu")
        .GetMethod("Attach").Invoke(null, new object[] { target, target.name, depth, (Action)(() => _changes++) });
    private GameObject New(string name) { GameObject go = new GameObject(name); _owned.Add(go); return go; }
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType(name)).First(t => t != null);
    private static FieldInfo Field(Component c, string name) => c.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic);
    private static object Call(Component c, string method, params object[] values) => c.GetType().GetMethod(method).Invoke(c, values);
    private static object Private(Component c, string method, params object[] values) => c.GetType().GetMethod(method, BindingFlags.Instance | BindingFlags.NonPublic).Invoke(c, values);
    private static T Get<T>(Component c, string property) => (T)c.GetType().GetProperty(property).GetValue(c);
}
