using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.Events;

/// <summary>Actual QR, calibration, robot and persistence components; no ROS transport or XR session.</summary>
public sealed class XrAlignmentIntegrationTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private Transform _tracking;
    private Component _coordinates;
    private Component _detector;
    private Component _instantiator;
    private GameObject _robot;
    private string _directory;
    private string _temporaryPrefix;

    [SetUp]
    public void SetUp()
    {
        _temporaryPrefix = Path.Combine(Path.GetTempPath(), "UnityROSML-XrAlignment-");
        _directory = _temporaryPrefix + Guid.NewGuid().ToString("N");
        Directory.CreateDirectory(_directory);
        _tracking = NewObject("Alignment tracking origin").transform;
        _tracking.SetPositionAndRotation(new Vector3(2f, 1f, -3f), Quaternion.Euler(0f, 48f, 0f));
        _coordinates = NewObject("Offline alignment coordinate service").AddComponent(RuntimeType("RobotCoordinateService"));
        NewObject("Offline alignment robot state").AddComponent(RuntimeType("RobotStateService"));
        CreateObservationComponents();
        ObserveAndPlace("physical-robot-tag", MarkerPose());
        Assert.That(_robot, Is.Not.Null);
    }

    [TearDown]
    public void TearDown()
    {
        for (int index = _owned.Count - 1; index >= 0; index--)
            if (_owned[index] != null) UnityEngine.Object.DestroyImmediate(_owned[index]);
        _owned.Clear();
        // Never remove a caller-supplied path: only this test's GUID directory under OS temporary storage.
        string absolute = Path.GetFullPath(_directory);
        Assert.That(absolute.StartsWith(Path.GetFullPath(_temporaryPrefix), StringComparison.OrdinalIgnoreCase), Is.True);
        if (Directory.Exists(absolute)) Directory.Delete(absolute, true);
        Type connection = Type.GetType("Unity.Robotics.ROSTCPConnector.ROSConnection, Unity.Robotics.ROSTCPConnector", false);
        if (connection != null) Assert.That(Resources.FindObjectsOfTypeAll(connection), Is.Empty, "Calibration must never establish ROS transport.");
    }

    [Test]
    public void DraftMovesAndTurnsOnlyTheVirtualBaseAroundItsOwnPivotAndBlocksCoordinateCommands()
    {
        Component rig = Rig;
        string[] names = { "ur_shoulder_pan_joint" };
        Assert.That(Call(rig, "SetJointPositions", names, new double[] { 0.47 }), Is.EqualTo(true));
        Pose original = BaseWorld;
        int revision = Get<int>(_coordinates, "CalibrationRevision");
        Call(_instantiator, "BeginCalibration");
        Assert.That(Get<bool>(_instantiator, "IsCalibrating"), Is.True);
        Assert.That(Get<bool>(_coordinates, "IsCalibrationInProgress"), Is.True);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.False);
        AssertCoordinateConversionBlocked();

        Vector3 delta = new Vector3(0.31f, 0.002f, -0.12f);
        Call(_instantiator, "NudgePosition", delta);
        Call(_instantiator, "RotateBase", Vector3.up, 27f);
        Pose expected = new Pose(original.position + delta, Quaternion.AngleAxis(27f, Vector3.up) * original.rotation);
        AssertPose(expected, BaseWorld);
        SimulatePhysics();
        InvokePrivate(_coordinates, "FixedUpdate");
        AssertPose(expected, BaseWorld);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.False);
        object[] joint = { names[0], 0d };
        Assert.That(rig.GetType().GetMethod("TryGetJointPosition").Invoke(rig, joint), Is.EqualTo(true));
        Assert.That((double)joint[1], Is.EqualTo(0.47).Within(1e-10));

        Call(_instantiator, "CancelCalibration");
        Assert.That(Get<bool>(_instantiator, "IsCalibrating"), Is.False);
        Assert.That(Get<bool>(_coordinates, "IsCalibrationInProgress"), Is.False);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.True);
        Assert.That(Get<int>(_coordinates, "CalibrationRevision"), Is.GreaterThan(revision));
        AssertPose(original, BaseWorld);
        Assert.That(Directory.GetFiles(_directory, "*.json"), Is.Empty);
    }

    [Test]
    public void SavedQrRelativePoseReloadsAfterRestartUnderADifferentTrackingOriginAndMarkerPose()
    {
        Call(_instantiator, "BeginCalibration");
        Call(_instantiator, "NudgePosition", new Vector3(0.43f, 0.018f, -0.09f));
        Call(_instantiator, "RotateBase", Vector3.up, 19f);
        Call(_instantiator, "RotateBase", Vector3.right, 3f);
        Pose savedRelative = RobotFrameMath.RelativeTo(Get<Pose>(_detector, "StabilizedPose"), BaseWorld);
        Call(_instantiator, "SaveCalibration");
        Assert.That(Get<bool>(_instantiator, "IsCalibrating"), Is.False);
        Assert.That(Get<bool>(_instantiator, "HasSavedCalibration"), Is.True);
        Assert.That(Directory.GetFiles(_directory, "*.json").Length, Is.EqualTo(1));

        UnityEngine.Object.DestroyImmediate(_robot);
        UnityEngine.Object.DestroyImmediate(_instantiator.gameObject);
        _tracking.SetPositionAndRotation(new Vector3(-4f, 0.3f, 5f), Quaternion.Euler(0f, -73f, 0f));
        CreateObservationComponents();
        Pose nextMarker = new Pose(new Vector3(-0.7f, -0.2f, 0.4f), Quaternion.Euler(0f, 34f, 0f) * Quaternion.Euler(90f, 0f, 0f));
        ObserveAndPlace("physical-robot-tag", nextMarker);
        Assert.That(Get<bool>(_instantiator, "HasSavedCalibration"), Is.True);
        AssertPose(savedRelative, Get<Pose>(_instantiator, "BasePoseInMarker"));
        AssertPose(RobotFrameMath.Compose(Get<Pose>(_detector, "StabilizedPose"), savedRelative), BaseWorld);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.True);
    }

    [Test]
    public void ANewMarkerPayloadDoesNotInheritAnotherMarkersSavedOffset()
    {
        Pose prefabRelative = Get<Pose>(_instantiator, "BasePoseInMarker");
        Call(_instantiator, "BeginCalibration");
        Call(_instantiator, "NudgePosition", Vector3.right * 0.4f);
        Call(_instantiator, "SaveCalibration");
        GameObject originalRobot = _robot;
        Call(_detector, "Rescan");
        ObserveAndPlace("different-physical-tag", MarkerPose());
        Assert.That(_robot, Is.SameAs(originalRobot));
        Assert.That(Get<bool>(_instantiator, "HasSavedCalibration"), Is.False);
        AssertPose(prefabRelative, Get<Pose>(_instantiator, "BasePoseInMarker"));
        AssertPose(RobotFrameMath.Compose(Get<Pose>(_detector, "StabilizedPose"), prefabRelative), BaseWorld);
    }

    [Test]
    public void ResetIsAnUnsavedDraftAndCancelKeepsThePreviouslySavedAlignment()
    {
        Pose prefabRelative = Get<Pose>(_instantiator, "BasePoseInMarker");
        Call(_instantiator, "BeginCalibration");
        Call(_instantiator, "NudgePosition", Vector3.forward * 0.22f);
        Call(_instantiator, "SaveCalibration");
        Pose saved = BaseWorld;
        Call(_instantiator, "BeginCalibration");
        Call(_instantiator, "RestoreDefaultCalibration");
        Assert.That(Get<bool>(_instantiator, "IsCalibrating"), Is.True);
        AssertPose(RobotFrameMath.Compose(Get<Pose>(_detector, "StabilizedPose"), prefabRelative), BaseWorld);
        Call(_instantiator, "CancelCalibration");
        AssertPose(saved, BaseWorld);
        Call(_detector, "Rescan");
        ObserveAndPlace("physical-robot-tag", MarkerPose());
        AssertPose(saved, BaseWorld);
    }

    [Test]
    public void RescanningDuringAnEditDiscardsItsDraftAndLeavesCommandsBlockedUntilPlacement()
    {
        Pose original = BaseWorld;
        GameObject originalRobot = _robot;
        Call(_instantiator, "BeginCalibration");
        Call(_instantiator, "NudgePosition", Vector3.right * 0.3f);
        Call(_detector, "Rescan");
        // The detector event is handled synchronously; no frame of editing may reenable coordinates.
        Assert.That(Get<bool>(_instantiator, "IsCalibrating"), Is.False);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.False);
        AssertCoordinateConversionBlocked();
        ObserveAndPlace("physical-robot-tag", MarkerPose());
        Assert.That(_robot, Is.SameAs(originalRobot));
        AssertPose(original, BaseWorld);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.True);
    }

    [Test]
    public void SavingRebindsCoordinateConversionToTheAlignedBaseAndRequiresFreshPlanningRevision()
    {
        Pose target = new Pose(new Vector3(3.1f, 0.7f, -1.4f), Quaternion.Euler(21f, 31f, -12f));
        int revision = Get<int>(_coordinates, "CalibrationRevision");
        Call(_instantiator, "BeginCalibration");
        Call(_instantiator, "NudgePosition", new Vector3(-0.2f, 0.03f, 0.15f));
        Call(_instantiator, "RotateBase", Vector3.up, -12f);
        AssertCoordinateConversionBlocked();
        Pose expectedLocal = RobotFrameMath.RelativeTo(BaseWorld, target);
        Call(_instantiator, "SaveCalibration");
        object[] conversion = { target, default(Pose) };
        Assert.That(_coordinates.GetType().GetMethod("TryConvertWorldPoseToLocalPose").Invoke(_coordinates, conversion), Is.EqualTo(true));
        AssertPose(expectedLocal, (Pose)conversion[1]);
        Assert.That(Get<int>(_coordinates, "CalibrationRevision"), Is.GreaterThan(revision));
    }

    [Test]
    public void InitializationCannotBypassTheCalibrationEditGate()
    {
        Call(_instantiator, "BeginCalibration");
        Transform root = Get<Transform>(Rig, "BaseLink");
        _coordinates.GetType().GetMethod("Initialize", new[] { typeof(ArticulationBody), typeof(Transform) })
            .Invoke(_coordinates, new object[] { root.GetComponent<ArticulationBody>(), _tracking });
        Assert.That(Get<bool>(_coordinates, "IsCalibrationInProgress"), Is.True);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.False);
        AssertCoordinateConversionBlocked();
        Call(_instantiator, "CancelCalibration");
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.True);
    }

    [Test]
    public void MovingTheXrOriginDuringEditingPreservesTheDraftQrRelativePoseAndCancelRestoresItsOriginalCalibration()
    {
        Pose originalRelative = Get<Pose>(_instantiator, "BasePoseInMarker");
        Call(_instantiator, "BeginCalibration");
        Call(_instantiator, "NudgePosition", new Vector3(0.17f, -0.002f, 0.06f));
        Call(_instantiator, "RotateBase", Vector3.up, 13f);
        Pose draftRelative = RobotFrameMath.RelativeTo(Get<Pose>(_detector, "StabilizedPose"), BaseWorld);
        _tracking.SetPositionAndRotation(new Vector3(-1f, 0.5f, 6f), Quaternion.Euler(0f, -58f, 0f));
        InvokePrivate(_instantiator, "LateUpdate");
        SimulatePhysics();
        InvokePrivate(_instantiator, "LateUpdate");
        AssertPose(RobotFrameMath.Compose(Get<Pose>(_detector, "StabilizedPose"), draftRelative), BaseWorld);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.False);
        Call(_instantiator, "CancelCalibration");
        AssertPose(RobotFrameMath.Compose(Get<Pose>(_detector, "StabilizedPose"), originalRelative), BaseWorld);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.True);
    }

    [Test]
    public void SaveFailureKeepsTheDraftVisibleAndCommandsBlockedSoTheUserCanRetryOrCancel()
    {
        string unwritableDirectory = Path.Combine(_directory, "a-file-instead-of-a-folder");
        File.WriteAllText(unwritableDirectory, "fixture");
        Set(_instantiator, "_calibrationDirectoryOverride", unwritableDirectory);
        Call(_instantiator, "BeginCalibration");
        Call(_instantiator, "NudgePosition", Vector3.right * 0.2f);
        Pose draft = BaseWorld;
        Call(_instantiator, "SaveCalibration");
        Assert.That(Get<bool>(_instantiator, "IsCalibrating"), Is.True);
        Assert.That(Get<bool>(_instantiator, "HasSavedCalibration"), Is.False);
        Assert.That(Get<string>(_instantiator, "AlignmentStatus"), Does.Contain("Save failed"));
        AssertPose(draft, BaseWorld);
        AssertCoordinateConversionBlocked();
        Call(_instantiator, "CancelCalibration");
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.True);
    }

    [Test]
    public void BeginningAlignmentImmediatelyAfterAnOriginMoveUsesTheAuthoritativeQrRelativeBase()
    {
        Pose relative = Get<Pose>(_instantiator, "BasePoseInMarker");
        _tracking.SetPositionAndRotation(new Vector3(-3f, 0.1f, 4f), Quaternion.Euler(0f, -65f, 0f));
        SimulatePhysics(); // Native articulation may still retain the former world pose.
        Call(_instantiator, "BeginCalibration");
        Assert.That(Get<bool>(_instantiator, "IsCalibrating"), Is.True);
        AssertPose(relative, Get<Pose>(_instantiator, "BasePoseInMarker"));
        AssertPose(RobotFrameMath.Compose(Get<Pose>(_detector, "StabilizedPose"), relative), BaseWorld);
        Call(_instantiator, "CancelCalibration");
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.True);
    }

    private Component Rig => _robot.GetComponent(RuntimeType("RosRobotRig"));
    private Pose BaseWorld
    {
        get { Transform root = Get<Transform>(Rig, "BaseLink"); return new Pose(root.position, root.rotation); }
    }

    private void CreateObservationComponents()
    {
        GameObject owner = NewObject("Inactive offline alignment components");
        owner.SetActive(false);
        Component stabilizer = owner.AddComponent(RuntimeType("SimpleQRPoseStabilizer"));
        Set(stabilizer, "_nbStabilizationFrames", 3);
        _detector = owner.AddComponent(RuntimeType("QRDetector"));
        Set(_detector, "_trackingSpace", _tracking);
        Set(_detector, "_showMarkerAxes", false);
        _instantiator = owner.AddComponent(RuntimeType("URInstantiator"));
        Set(_instantiator, "_QRDetector", _detector);
        Set(_instantiator, "_URPrefab", AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab"));
        Set(_instantiator, "_calibrationDirectoryOverride", _directory);
        // Disabled tests subscribe deliberately; Start/OnEnable must never create a hardware detector.
        InvokePrivate(_instantiator, "SubscribeToDetector");
        UnityEvent<GameObject, Pose> created = (UnityEvent<GameObject, Pose>)_instantiator.GetType().GetField("OnURInstantiated").GetValue(_instantiator);
        created.AddListener((robot, pose) => { _robot = robot; _owned.Add(robot); });
    }

    private void ObserveAndPlace(string payload, Pose pose)
    {
        for (int index = 0; index < 3; index++)
            Call(_detector, "ObserveMarkers", new[] { payload }, new Pose?[] { pose }, new[] { 0.087f });
        Assert.That(Get<bool>(_detector, "HasValidatedPose"), Is.True);
        Call(_instantiator, "InstantiateUR");
    }

    private void AssertCoordinateConversionBlocked()
    {
        object[] request = { Pose.identity, default(Pose) };
        Assert.That(_coordinates.GetType().GetMethod("TryConvertWorldPoseToLocalPose").Invoke(_coordinates, request), Is.EqualTo(false));
    }

    private static Pose MarkerPose() => new Pose(new Vector3(0.3f, -0.4f, 0.8f), Quaternion.Euler(90f, 0f, 0f));
    private GameObject NewObject(string name) { GameObject value = new GameObject(name); _owned.Add(value); return value; }
    private static Type RuntimeType(string name) => Type.GetType(name + ", Assembly-CSharp", true);
    private static T Get<T>(object owner, string name) => (T)owner.GetType().GetProperty(name).GetValue(owner);
    private static void Set(object owner, string name, object value) => owner.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic).SetValue(owner, value);
    private static object Call(object owner, string name, params object[] args) => owner.GetType().GetMethod(name).Invoke(owner, args);
    private static void InvokePrivate(object owner, string name) => owner.GetType().GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic).Invoke(owner, null);
    private static void AssertPose(Pose expected, Pose actual)
    {
        Assert.That(Vector3.Distance(expected.position, actual.position), Is.LessThan(0.00002f));
        foreach (Vector3 axis in new[] { Vector3.right, Vector3.up, Vector3.forward })
            Assert.That(Vector3.Distance(expected.rotation * axis, actual.rotation * axis), Is.LessThan(0.00002f));
    }
    private static void SimulatePhysics()
    {
        SimulationMode prior = Physics.simulationMode;
        try { Physics.simulationMode = SimulationMode.Script; Physics.SyncTransforms(); Physics.Simulate(0.02f); Physics.Simulate(0.02f); }
        finally { Physics.simulationMode = prior; }
    }
}
