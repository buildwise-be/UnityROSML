using System;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.Events;

/// <summary>Actual QR component policy and native prefab placement, without an OpenXR session or ROS connection.</summary>
public sealed class QrCalibrationTests
{
    private GameObject _tracking;
    private GameObject _owner;
    private Component _detector;
    private Component _stabilizer;

    [SetUp]
    public void SetUp()
    {
        _tracking = new GameObject("Offline QR tracking space");
        _tracking.transform.SetPositionAndRotation(new Vector3(2f, 1f, -3f), Quaternion.Euler(0f, 48f, 0f));
        _owner = new GameObject("Offline QR observations");
        _owner.SetActive(false);
        _stabilizer = _owner.AddComponent(RuntimeType("SimpleQRPoseStabilizer"));
        Set(_stabilizer, "_nbStabilizationFrames", 3);
        _detector = _owner.AddComponent(RuntimeType("QRDetector"));
        Set(_detector, "_trackingSpace", _tracking.transform);
        Set(_detector, "_showMarkerAxes", false);
    }

    [TearDown]
    public void TearDown()
    {
        foreach (string name in new[] { "RobotStateService", "RobotCoordinateService" })
            foreach (UnityEngine.Object component in UnityEngine.Object.FindObjectsByType(RuntimeType(name), FindObjectsInactive.Include, FindObjectsSortMode.None))
                UnityEngine.Object.DestroyImmediate(((Component)component).gameObject);
        UnityEngine.Object.DestroyImmediate(_owner);
        UnityEngine.Object.DestroyImmediate(_tracking);
    }

    [Test]
    public void IdentityPoseIsValidAndConvertedThroughTheTrackingOrigin()
    {
        Observe(new[] { "physical-tag" }, new Pose?[] { Pose.identity });
        Observe(new[] { "physical-tag" }, new Pose?[] { Pose.identity });
        Observe(new[] { "physical-tag" }, new Pose?[] { Pose.identity });
        Assert.That(Get<bool>(_detector, "HasValidatedPose"), Is.True);
        AssertPose(new Pose(_tracking.transform.position, _tracking.transform.rotation), Get<Pose>(_detector, "StabilizedPose"));
        Assert.That(Get<string>(_detector, "DetectedPayload"), Is.EqualTo("physical-tag"));
        Assert.That(Get<float>(_detector, "MarkerSizeMeters"), Is.EqualTo(0.087f));
    }

    [TestCase(false)]
    [TestCase(true)]
    public void InitialMultiplePhysicalMarkersAreAmbiguousEvenWhenOnlyOnePoseIsValid(bool samePayload)
    {
        string other = samePayload ? "A" : "B";
        Observe(new[] { "A", other }, new Pose?[] { Pose.identity, null });
        Assert.That(Samples, Is.Zero);
        Assert.That(Get<bool>(_detector, "HasValidatedPose"), Is.False);
    }

    [Test]
    public void PayloadIsPinnedAndMissingMarkerResetsItsConsecutiveWindow()
    {
        Pose selected = new Pose(new Vector3(0.3f, 0.2f, 0.8f), Quaternion.Euler(90f, 0f, 0f));
        Pose foreign = new Pose(new Vector3(8f, 7f, 6f), Quaternion.Euler(0f, 150f, 0f));
        Observe(new[] { "A" }, new Pose?[] { selected });
        Observe(new[] { "B" }, new Pose?[] { foreign });
        Assert.That(Samples, Is.Zero);
        Assert.That(Get<string>(_detector, "DetectedPayload"), Is.EqualTo("A"));
        Observe(new[] { "B", "A" }, new Pose?[] { foreign, selected });
        Observe(new[] { "A", "B" }, new Pose?[] { selected, foreign });
        Assert.That(Get<bool>(_detector, "HasValidatedPose"), Is.False);
        Observe(new[] { "B", "A" }, new Pose?[] { foreign, selected });
        Pose expected = RobotFrameMath.Compose(new Pose(_tracking.transform.position, _tracking.transform.rotation), selected);
        AssertPose(expected, Get<Pose>(_detector, "StabilizedPose"));
    }

    [Test]
    public void ExactExpectedPayloadSelectsOneMarkerWithoutGuessingHistoricalQrNames()
    {
        Assert.That(Get<string>(_detector, "ExpectedMarkerPayload"), Is.Empty);
        Call(_detector, "SetExpectedMarkerPayload", "known physical payload");
        for (int i = 0; i < 3; i++)
            Observe(new[] { "QR1", "known physical payload" }, new Pose?[] { new Pose(Vector3.one * 10f, Quaternion.identity), Pose.identity });
        Assert.That(Get<bool>(_detector, "HasValidatedPose"), Is.True);
        Assert.That(Get<string>(_detector, "DetectedPayload"), Is.EqualTo("known physical payload"));
    }

    [Test]
    public void DuplicatePinnedPayloadCopiesAndInvalidSelectedPosesResetTheWindow()
    {
        Observe(new[] { "A" }, new Pose?[] { Pose.identity });
        Observe(new[] { "A", "A" }, new Pose?[] { Pose.identity, Pose.identity });
        Assert.That(Samples, Is.Zero);
        Observe(new[] { "A" }, new Pose?[] { Pose.identity });
        Observe(new[] { "A" }, new Pose?[] { new Pose(Vector3.zero, new Quaternion()) });
        Assert.That(Samples, Is.Zero);
    }

    [TestCase(true)]
    [TestCase(false)]
    public void PositionOrRotationDiscontinuityStartsANewWindow(bool translate)
    {
        Pose first = new Pose(new Vector3(0.3f, 0.1f, 0.7f), Quaternion.Euler(90f, 0f, 0f));
        Pose next = translate ? new Pose(first.position + Vector3.right * 0.03f, first.rotation)
            : new Pose(first.position, Quaternion.AngleAxis(10f, Vector3.up) * first.rotation);
        Observe(new[] { "A" }, new Pose?[] { first });
        Observe(new[] { "A" }, new Pose?[] { first });
        Observe(new[] { "A" }, new Pose?[] { next });
        Assert.That(Samples, Is.EqualTo(1));
        Assert.That(Get<int>(_stabilizer, "DiscontinuityCount"), Is.EqualTo(1));
        Observe(new[] { "A" }, new Pose?[] { next });
        Assert.That(Get<bool>(_detector, "HasValidatedPose"), Is.False);
        Observe(new[] { "A" }, new Pose?[] { next });
        AssertPose(RobotFrameMath.Compose(new Pose(_tracking.transform.position, _tracking.transform.rotation), next), Get<Pose>(_detector, "StabilizedPose"));
    }

    [Test]
    public void WindowUsesFirstSampleAndQuaternionSignDoesNotCauseAFalseRestart()
    {
        Pose pose = new Pose(Vector3.zero, Quaternion.Euler(70f, 31f, -10f));
        Quaternion q = pose.rotation;
        Observe(new[] { "A" }, new Pose?[] { pose });
        Observe(new[] { "A" }, new Pose?[] { new Pose(Vector3.right * 0.02f, new Quaternion(-q.x, -q.y, -q.z, -q.w)) });
        Assert.That(Samples, Is.EqualTo(2));
        Observe(new[] { "A" }, new Pose?[] { new Pose(Vector3.right * 0.04f, q) });
        Assert.That(Samples, Is.EqualTo(1), "Slow drift must not extend the averaging window indefinitely.");
    }

    [Test]
    public void RescanImmediatelyInvalidatesCalibrationAndAllowsANewPayload()
    {
        GameObject coordinatesObject = new GameObject("Offline coordinate service");
        Component coordinates = coordinatesObject.AddComponent(RuntimeType("RobotCoordinateService"));
        int notifications = 0;
        coordinates.GetType().GetEvent("CalibrationChanged").AddEventHandler(coordinates, (Action)(() => notifications++));
        Observe(new[] { "A" }, new Pose?[] { Pose.identity });
        Call(_detector, "Rescan");
        Assert.That(Get<int>(coordinates, "CalibrationRevision"), Is.EqualTo(1));
        Assert.That(notifications, Is.EqualTo(1));
        Assert.That(Get<bool>(coordinates, "IsInitialized"), Is.False);
        Assert.That(Get<bool>(_detector, "HasValidatedPose"), Is.False);
        Assert.That(Get<string>(_detector, "DetectedPayload"), Is.Empty);
        for (int i = 0; i < 3; i++) Observe(new[] { "B" }, new Pose?[] { Pose.identity });
        Assert.That(Get<string>(_detector, "DetectedPayload"), Is.EqualTo("B"));
    }

    [Test]
    public void ValidatedQrAxesHaveThreeLabelledArrowsAndNoInteractionCollidersAndRescanDisposesThem()
    {
        Set(_detector, "_showMarkerAxes", true);
        for (int i = 0; i < 3; i++) Observe(new[] { "physical-tag" }, new Pose?[] { Pose.identity });
        GameObject axes = (GameObject)_detector.GetType().GetField("_markerAxesObject", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(_detector);
        Assert.That(axes, Is.Not.Null);
        Assert.That(axes.GetComponentsInChildren<LineRenderer>().Length, Is.EqualTo(3));
        Assert.That(axes.GetComponentsInChildren<Collider>().Length, Is.Zero);
        Assert.That(axes.transform.Find("QR +Z into printed face"), Is.Not.Null);
        Assert.That(axes.transform.parent, Is.EqualTo(_tracking.transform));
        Call(_detector, "Rescan");
        Assert.That(axes == null, Is.True);
    }

    [TestCase(0f, 0f)]
    [TestCase(48f, -25f)]
    [TestCase(-70f, 110f)]
    public void ActivePrefabAndNativePhysicsApplyTheQrExtrinsicExactlyOnce(float originYaw, float markerYaw)
    {
        GameObject robot = null;
        SimulationMode previous = Physics.simulationMode;
        try
        {
            Physics.simulationMode = SimulationMode.Script;
            _tracking.transform.rotation = Quaternion.Euler(0f, originYaw, 0f);
            Pose marker = new Pose(new Vector3(0.4f, -0.35f, 0.9f), Quaternion.Euler(0f, markerYaw, 0f) * Quaternion.Euler(90f, 0f, 0f));
            Pose world = RobotFrameMath.Compose(new Pose(_tracking.transform.position, _tracking.transform.rotation), marker);
            Pose expected = RobotFrameMath.Compose(world, new Pose(new Vector3(-0.419f, 0.230f, 0.505f), Quaternion.Euler(-90f, 0f, 0f)));
            GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab");
            robot = UnityEngine.Object.Instantiate(prefab, world.position, world.rotation);
            robot.transform.SetParent(_tracking.transform, true);
            Transform baseLink = Array.Find(robot.GetComponentsInChildren<Transform>(), t => t.name == "base_link");
            AssertPose(expected, new Pose(baseLink.position, baseLink.rotation));
            Component rig = robot.AddComponent(RuntimeType("RosRobotRig"));
            Assert.That(Call(rig, "ApplyModel", RosRobotModelCatalog.Load().Find("kenos_sl")), Is.EqualTo(true));
            Physics.SyncTransforms();
            Physics.Simulate(0.02f); Physics.Simulate(0.02f);
            AssertPose(expected, new Pose(baseLink.position, baseLink.rotation));
        }
        finally
        {
            if (robot != null) UnityEngine.Object.DestroyImmediate(robot);
            Physics.simulationMode = previous;
        }
    }

    [Test]
    public void ReanchorRetainsOneRobotAndItsJointFeedbackAndRevisesCalibration()
    {
        Component instantiator = _owner.AddComponent(RuntimeType("URInstantiator"));
        Set(instantiator, "_QRDetector", _detector);
        Set(instantiator, "_URPrefab", AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab"));
        int creations = 0;
        UnityEvent<GameObject, Pose> created = (UnityEvent<GameObject, Pose>)instantiator.GetType().GetField("OnURInstantiated").GetValue(instantiator);
        GameObject robot = null;
        created.AddListener((value, pose) => { creations++; robot = value; });
        try
        {
            // Precreate services so their runtime singleton fallback never calls DontDestroyOnLoad in EditMode.
            new GameObject("Offline robot state").AddComponent(RuntimeType("RobotStateService"));
            new GameObject("Offline robot coordinates").AddComponent(RuntimeType("RobotCoordinateService"));
            Pose first = new Pose(new Vector3(0.3f, -0.4f, 0.8f), Quaternion.Euler(90f, 0f, 0f));
            Call(_detector, "SetStabilizedPose", first);
            Call(instantiator, "InstantiateUR");
            Assert.That(robot, Is.Not.Null);
            Component rig = robot.GetComponent(RuntimeType("RosRobotRig"));
            string[] names = { "ur_shoulder_pan_joint" };
            Assert.That(Call(rig, "SetJointPositions", names, new double[] { 0.47 }), Is.EqualTo(true));
            Component coordinates = UnityEngine.Object.FindFirstObjectByType(RuntimeType("RobotCoordinateService")) as Component;
            int revision = Get<int>(coordinates, "CalibrationRevision");
            Call(_detector, "Rescan");
            Assert.That(Get<bool>(coordinates, "IsInitialized"), Is.False);
            object[] invalidConversion = { Pose.identity, default(Pose) };
            Assert.That(coordinates.GetType().GetMethod("TryConvertWorldPoseToLocalPose").Invoke(coordinates, invalidConversion), Is.EqualTo(false));
            Pose second = new Pose(new Vector3(-0.6f, -0.3f, 0.5f), Quaternion.Euler(0f, 35f, 0f) * Quaternion.Euler(90f, 0f, 0f));
            Call(_detector, "SetStabilizedPose", second);
            Call(instantiator, "InstantiateUR");
            Assert.That(creations, Is.EqualTo(1));
            Assert.That(Get<int>(coordinates, "CalibrationRevision"), Is.EqualTo(revision + 2));
            Transform baseLink = Get<Transform>(rig, "BaseLink");
            Pose expected = RobotFrameMath.Compose(RobotFrameMath.Compose(new Pose(_tracking.transform.position, _tracking.transform.rotation), second), Get<Pose>(instantiator, "BasePoseInMarker"));
            AssertPose(expected, new Pose(baseLink.position, baseLink.rotation));
            object[] arguments = { names[0], 0d };
            Assert.That(rig.GetType().GetMethod("TryGetJointPosition").Invoke(rig, arguments), Is.EqualTo(true));
            Assert.That((double)arguments[1], Is.EqualTo(0.47).Within(1e-10));
            Call(instantiator, "InstantiateUR");
            Assert.That(Get<int>(coordinates, "CalibrationRevision"), Is.EqualTo(revision + 2), "Repeated trigger with the same scan must be ignored.");
        }
        finally { if (robot != null) UnityEngine.Object.DestroyImmediate(robot); }
    }

    private int Samples => Get<int>(_stabilizer, "SamplesCollected");
    private void Observe(string[] payloads, Pose?[] poses)
    {
        float[] lengths = new float[payloads.Length];
        Array.Fill(lengths, 0.087f);
        Call(_detector, "ObserveMarkers", payloads, poses, lengths);
    }
    private static Type RuntimeType(string name) => Type.GetType(name + ", Assembly-CSharp", true);
    private static T Get<T>(object owner, string name) => (T)owner.GetType().GetProperty(name).GetValue(owner);
    private static void Set(object owner, string name, object value) => owner.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic).SetValue(owner, value);
    private static object Call(object owner, string name, params object[] args) => owner.GetType().GetMethod(name).Invoke(owner, args);
    private static void AssertPose(Pose expected, Pose actual)
    {
        Assert.That(Vector3.Distance(expected.position, actual.position), Is.LessThan(0.00002f));
        foreach (Vector3 axis in new[] { Vector3.right, Vector3.up, Vector3.forward })
            Assert.That(Vector3.Distance(expected.rotation * axis, actual.rotation * axis), Is.LessThan(0.00002f));
    }
}
