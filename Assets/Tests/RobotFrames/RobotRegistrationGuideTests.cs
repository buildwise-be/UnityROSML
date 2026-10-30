using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>Actual CAD guide geometry and registration inputs, without a ROS connection or an XR session.</summary>
public sealed class RobotRegistrationGuideTests
{
    private GameObject _registrationOwner;
    private GameObject _guideOwner;
    private Component _registration;
    private Component _guide;
    private Component _feedback;
    private GameObject _prefab;
    private RosRobotModel _model;
    private Dictionary<string, double> _joints;

    [SetUp]
    public void CreateWithoutRuntimeStart()
    {
        _model = RosRobotModelCatalog.Load().Find("kenos");
        RosRobotGoldenCase bent = Array.Find(_model.goldenCases, pose => pose.name == "bent");
        _joints = bent.jointNames.Select((name, index) => new { name, value = bent.jointPositions[index] })
            .ToDictionary(joint => joint.name, joint => joint.value);
        _prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab");
        Assert.That(_prefab, Is.Not.Null);
        _registrationOwner = new GameObject("Inactive guide registration fixture");
        _registrationOwner.SetActive(false);
        _registration = _registrationOwner.AddComponent(RuntimeType("RobotPointRegistration"));
        _feedback = _registrationOwner.AddComponent(RuntimeType("JointStateSubscriber"));
        Set(_registration, "_jointFeedback", _feedback);
        Set(_registration, "_fallbackModel", _model);
        Set(_registration, "<IsCollecting>k__BackingField", true);
        _guideOwner = new GameObject("Registration guide mount");
        _guideOwner.transform.localScale = Vector3.one * 0.001f;
        _guide = _guideOwner.AddComponent(RuntimeType("RobotRegistrationGuide"));
    }

    [TearDown]
    public void Cleanup()
    {
        if (_guideOwner != null) UnityEngine.Object.DestroyImmediate(_guideOwner);
        if (_registrationOwner != null) UnityEngine.Object.DestroyImmediate(_registrationOwner);
    }

    [Test]
    public void EveryStepHighlightsItsActualLinkAndDistinctCalibratedCoverCenter()
    {
        LockPose();
        RobotCoverLandmark[] landmarks = RobotCoverLandmarks.CreateDefaults();
        Assert.That(RobotCoverLandmarks.TryGetBasePoints(_model, _joints, landmarks, out Vector3[] points, out string error), Is.True, error);
        GameObject geometry = null;
        int[] rendererIds = null;
        for (int step = 0; step < 4; step++)
        {
            SetStep(step);
            Refresh();
            Assert.That(Get<bool>(_guide, "HasVisual"), Is.True, Get<string>(_guide, "PoseStatus"));
            Assert.That(Get<int>(_guide, "SelectedPointIndex"), Is.EqualTo(step));
            Assert.That(Get<string>(_guide, "CurrentPointName"), Is.EqualTo(landmarks[step].displayName));
            Assert.That(Get<string>(_guide, "HighlightedLinkName"), Is.EqualTo(landmarks[step].linkName));
            Assert.That(Vector3.Distance(Get<Vector3>(_guide, "TargetPointInModel"), points[step]), Is.LessThan(0.000001f));
            IList markers = Field<IList>(_guide, "_markers");
            for (int point = 0; point < 4; point++)
                Assert.That(Vector3.Distance(((Transform)markers[point]).localPosition, points[point]), Is.LessThan(0.000001f));
            Assert.That(((Transform)markers[step]).localScale.x, Is.GreaterThan(0.04f));
            IList visuals = Field<IList>(_guide, "_visuals");
            foreach (object visual in visuals)
            {
                string link = (string)visual.GetType().GetField("Link").GetValue(visual);
                Transform transform = (Transform)visual.GetType().GetField("Transform").GetValue(visual);
                Material[] expected = (Material[])visual.GetType().GetField(link == landmarks[step].linkName ? "Highlight" : "Dim").GetValue(visual);
                CollectionAssert.AreEqual(expected, transform.GetComponent<MeshRenderer>().sharedMaterials);
            }
            GameObject current = Field<GameObject>(_guide, "_geometry");
            int[] currentIds = current.GetComponentsInChildren<Renderer>().Select(renderer => renderer.GetInstanceID()).ToArray();
            if (geometry == null) { geometry = current; rendererIds = currentIds; }
            else { Assert.That(current, Is.SameAs(geometry)); CollectionAssert.AreEqual(rendererIds, currentIds); }
            Bounds bounds = RenderBounds(current);
            Assert.That(Mathf.Max(bounds.size.x, bounds.size.y, bounds.size.z), Is.InRange(0.18f, 0.22f));
        }
        Assert.That(Vector3.Distance(points[0], points[1]), Is.GreaterThan(0.3f),
            "Shoulder and elbow share an owning link but must identify different cover centers.");
    }

    [Test]
    public void MiniatureContainsOnlyGeometryAndCannotJoinTheLiveContourRegistryOrPhysics()
    {
        LockPose();
        var originals = _prefab.GetComponentsInChildren<Renderer>(true)
            .ToDictionary(renderer => renderer, renderer => renderer.sharedMaterials);
        Type contours = RuntimeType("RobotContourDisplay");
        bool previous = (bool)contours.GetProperty("Enabled").GetValue(null);
        contours.GetMethod("SetEnabled").Invoke(null, new object[] { true });
        try
        {
            Refresh();
            Assert.That(_guideOwner.GetComponentsInChildren<ArticulationBody>(true), Is.Empty);
            Assert.That(_guideOwner.GetComponentsInChildren<Rigidbody>(true), Is.Empty);
            Assert.That(_guideOwner.GetComponentsInChildren<Collider>(true), Is.Empty);
            Assert.That(_guideOwner.GetComponentsInChildren(RuntimeType("RosRobotRig"), true), Is.Empty);
            Assert.That(_guideOwner.GetComponentsInChildren<MonoBehaviour>(true), Is.EqualTo(new[] { _guide }));
            foreach (Renderer renderer in _guideOwner.GetComponentsInChildren<Renderer>(true))
            {
                Assert.That(renderer.forceRenderingOff, Is.False);
                Assert.That(renderer.gameObject.layer, Is.EqualTo(2));
            }
            foreach (var item in originals) CollectionAssert.AreEqual(item.Value, item.Key.sharedMaterials);
            Type connection = RuntimeType("Unity.Robotics.ROSTCPConnector.ROSConnection");
            Assert.That(Resources.FindObjectsOfTypeAll(connection), Is.Empty);
        }
        finally { contours.GetMethod("SetEnabled").Invoke(null, new object[] { previous }); }
    }

    [Test]
    public void GuideUsesFreshFeedbackBeforeCaptureAndTheLockedPoseAfterCapture()
    {
        Observe(_joints);
        Refresh();
        Assert.That(Get<string>(_guide, "PoseStatus"), Is.EqualTo("Live robot pose"));
        Vector3 before = Get<Vector3>(_guide, "TargetPointInModel");
        LockPose();
        var moved = new Dictionary<string, double>(_joints);
        moved["ur_shoulder_pan_joint"] += 0.7d;
        Observe(moved);
        Refresh();
        Assert.That(Get<string>(_guide, "PoseStatus"), Is.EqualTo("Pose locked at first point"));
        Assert.That(Vector3.Distance(before, Get<Vector3>(_guide, "TargetPointInModel")), Is.LessThan(0.000001f));
        object[] state = { null, null, null, false };
        Assert.That(Call(_registration, "GetGuideState", state), Is.EqualTo(true));
        ((Dictionary<string, double>)state[1])["ur_shoulder_pan_joint"] += 2d;
        Assert.That(Field<Dictionary<string, double>>(_registration, "_referenceJoints")["ur_shoulder_pan_joint"],
            Is.EqualTo(_joints["ur_shoulder_pan_joint"]), "A guide consumer cannot alter the registration fit pose.");
        Set(_registration, "_referenceJoints", null);
        Set(_registration, "_referenceModel", null);
        Refresh();
        Assert.That(Get<string>(_guide, "PoseStatus"), Is.EqualTo("Live robot pose"));
        Assert.That(Vector3.Distance(before, Get<Vector3>(_guide, "TargetPointInModel")), Is.GreaterThan(0.02f));
    }

    [Test]
    public void WaitingIllustrationIsExplicitAndCompletionOrDisableHidesTheGuide()
    {
        Refresh();
        Assert.That(Get<bool>(_guide, "HasVisual"), Is.True, Get<string>(_guide, "PoseStatus"));
        Assert.That(Get<string>(_guide, "PoseStatus"), Does.StartWith("Illustration"));
        GameObject geometry = Field<GameObject>(_guide, "_geometry");
        Set(_registration, "<IsCollecting>k__BackingField", false);
        Refresh();
        Assert.That(Get<bool>(_guide, "HasVisual"), Is.False);
        Assert.That(geometry.activeSelf, Is.False);
        Set(_registration, "<IsCollecting>k__BackingField", true);
        Refresh();
        Assert.That(Get<bool>(_guide, "HasVisual"), Is.True);
        _guideOwner.SetActive(false);
        Assert.That(geometry.activeInHierarchy, Is.False);
        // Ordinary runtime MonoBehaviours do not receive native lifecycle
        // callbacks in this EditMode fixture. Exercise the runtime hook explicitly.
        Call(_guide, "OnDisable");
        Assert.That(geometry.activeSelf, Is.False);
        Assert.That(Get<bool>(_guide, "HasVisual"), Is.False);
    }

    [Test]
    public void CustomCoverCoordinatesAndMissingReferenceDoNotSilentlyShowTheWrongTarget()
    {
        LockPose();
        RobotCoverLandmark[] landmarks = RobotCoverLandmarks.CreateDefaults();
        landmarks[0].pointInVisualRos += new Vector3(0.003f, 0f, 0f);
        Set(_registration, "_landmarks", landmarks);
        Assert.That(RobotCoverLandmarks.TryGetBasePoints(_model, _joints, landmarks, out Vector3[] points, out string error), Is.True, error);
        Refresh();
        Assert.That(Vector3.Distance(points[0], Get<Vector3>(_guide, "TargetPointInModel")), Is.LessThan(0.000001f));
        Call(_guide, "Refresh", _registration, null, 200f);
        Assert.That(Get<bool>(_guide, "HasVisual"), Is.False);
        Assert.That(Get<string>(_guide, "PoseStatus"), Does.Contain("reference missing"));
    }

    /// <summary>Optional graphics-enabled rendering of the four real CAD targets.</summary>
    public static void RenderRegistrationGuidePreviews()
    {
        Assert.That(SystemInfo.graphicsDeviceType, Is.Not.EqualTo(UnityEngine.Rendering.GraphicsDeviceType.Null));
        var fixture = new RobotRegistrationGuideTests();
        GameObject cameraOwner = null;
        RenderTexture target = null;
        Texture2D pixels = null;
        try
        {
            fixture.CreateWithoutRuntimeStart();
            fixture.LockPose();
            cameraOwner = new GameObject("Offline miniature preview camera", typeof(Camera));
            Camera camera = cameraOwner.GetComponent<Camera>();
            camera.transform.position = new Vector3(0f, 0f, -0.65f);
            camera.orthographic = true;
            camera.orthographicSize = 0.13f;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.045f, 0.075f, 0.1f, 1f);
            camera.cullingMask = 1 << 31;
            camera.allowHDR = false;
            camera.allowMSAA = false;
            target = new RenderTexture(600, 600, 24, RenderTextureFormat.ARGB32);
            camera.targetTexture = target;
            pixels = new Texture2D(600, 600, TextureFormat.RGB24, false);
            string directory = Path.GetFullPath("RegistrationGuidePreviews");
            Directory.CreateDirectory(directory);
            for (int step = 0; step < 4; step++)
            {
                fixture.SetStep(step);
                fixture.Refresh();
                foreach (Transform child in fixture._guideOwner.GetComponentsInChildren<Transform>(true)) child.gameObject.layer = 31;
                camera.Render();
                RenderTexture.active = target;
                pixels.ReadPixels(new Rect(0f, 0f, 600f, 600f), 0, 0);
                pixels.Apply();
                int orange = pixels.GetPixels32().Count(color => color.r > 180 && color.g > 70 && color.g < 235 && color.b < 140);
                Assert.That(orange, Is.GreaterThan(8), "The current cover-center marker must be visible for step " + step);
                string path = Path.Combine(directory, "step-" + (step + 1) + ".png");
                File.WriteAllBytes(path, pixels.EncodeToPNG());
                Debug.Log("RegistrationGuidePreview: step=" + (step + 1) + ", orangeTargetPixels=" + orange + ", " + path);
            }
        }
        finally
        {
            if (cameraOwner != null) cameraOwner.GetComponent<Camera>().targetTexture = null;
            RenderTexture.active = null;
            if (pixels != null) UnityEngine.Object.DestroyImmediate(pixels);
            if (target != null) UnityEngine.Object.DestroyImmediate(target);
            if (cameraOwner != null) UnityEngine.Object.DestroyImmediate(cameraOwner);
            fixture.Cleanup();
        }
    }

    private void Refresh() => Call(_guide, "Refresh", _registration, _prefab, 200f);
    private void LockPose()
    {
        Set(_registration, "_referenceModel", _model);
        Set(_registration, "_referenceJoints", new Dictionary<string, double>(_joints));
    }
    private void SetStep(int step)
    {
        IList points = Field<IList>(_registration, "_measuredInTracking");
        points.Clear();
        for (int index = 0; index < step; index++) points.Add(Vector3.one * index);
    }
    private void Observe(Dictionary<string, double> joints) => Call(_feedback, "ObserveJointState", joints.Keys.ToArray(), joints.Values.ToArray());
    private static Bounds RenderBounds(GameObject root)
    {
        Renderer[] renderers = root.GetComponentsInChildren<Renderer>();
        Bounds bounds = renderers[0].bounds;
        foreach (Renderer renderer in renderers) bounds.Encapsulate(renderer.bounds);
        return bounds;
    }
    private static object Call(Component target, string name, params object[] args) => target.GetType()
        .GetMethod(name, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic).Invoke(target, args);
    private static void Set(Component target, string name, object value) => target.GetType()
        .GetField(name, BindingFlags.Instance | BindingFlags.NonPublic).SetValue(target, value);
    private static T Field<T>(Component target, string name) => (T)target.GetType()
        .GetField(name, BindingFlags.Instance | BindingFlags.NonPublic).GetValue(target);
    private static T Get<T>(Component target, string name) => (T)target.GetType().GetProperty(name).GetValue(target);
    private static Type RuntimeType(string name) => AppDomain.CurrentDomain.GetAssemblies()
        .Select(assembly => assembly.GetType(name)).FirstOrDefault(type => type != null)
        ?? throw new InvalidOperationException("Missing runtime type " + name);
}
