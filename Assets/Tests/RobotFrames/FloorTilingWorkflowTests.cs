using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>Exercise floor planning and review with the publication boundary replaced in the offline harness.</summary>
public sealed class FloorTilingWorkflowTests
{
    private const string PlanId = "floor-plan";
    private readonly List<GameObject> _owned = new List<GameObject>();
    private Component _controller, _visualizer;
    private string _requestId;

    [SetUp]
    public void PrepareAndPlanCapturedFloorBatch()
    {
        Type offlineController = FindType("OfflineFloorTilingController");
        if (offlineController == null) Assert.Ignore("Requires the connection-free fixture and floor validation project.");

        Component coordinates = Add("Coordinates", TypeOf("RobotCoordinateService"));
        Add("Robot state", TypeOf("RobotStateService"));
        ArticulationBody robotBase = New("Robot base").AddComponent<ArticulationBody>();
        Transform tracking = New("Tracking space").transform;
        Call(coordinates, "Initialize", robotBase, tracking);

        Component tools = Add("Matched tool", TypeOf("ToolSelectionController"));
        if (Field(tools, "_catalog").GetValue(tools) == null) Call(tools, "Awake");
        string urdf = Resources.Load<TextAsset>("RobotModels/kenos_sl.urdf").text;
        foreach (string service in new[] { "/robot_state_publisher/get_parameters", "/move_group/get_parameters", "/mtc_test_task_node/get_parameters" })
            Assert.That(Call(tools, "InspectDescription", service, urdf), Is.EqualTo(true));

        _visualizer = Add("Path reviewer", TypeOf("UR16eTrajectoryVisualizer"));
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos_transparent.prefab");
        GameObject robot = UnityEngine.Object.Instantiate(prefab);
        _owned.Add(robot);
        robot.SetActive(false);
        Component rig = robot.AddComponent(TypeOf("RosRobotRig"));
        Assert.That(Call(rig, "ApplyModel", Property<object>(tools, "CurrentModel")), Is.EqualTo(true));
        Set(_visualizer, "virtualUr16ePrefab", prefab);
        Call(_visualizer, "RecordRobot", robot, Pose.identity);

        _controller = Add("Offline floor controller", offlineController);
        Set(_controller, "_visualizer", _visualizer);
        Call(_controller, "SetTaskActive", true);
        object capture = Property<object>(_controller, "SurfaceCapture");
        Call(capture, "SetTipOffset", Vector3.zero);
        Call(_controller, "BeginCapture");
        foreach (Vector3 corner in PickSurfaceGeometry.GetCanonicalCorners(PickItem.Tile))
            Assert.That(Call(capture, "CapturePointerPose", new Pose(corner + Vector3.up * .2f, Quaternion.identity), tracking),
                Is.EqualTo(true), Property<string>(capture, "StatusText"));

        // Inject an XR floor hit in tracking space. The actual measurement and command gates remain active.
        Set(_controller, "_anchorSpace", tracking);
        Set(_controller, "_anchorInTracking", new Pose(new Vector3(.3f, 0f, .5f), Quaternion.identity));
        Set(_controller, "_anchorCalibrationRevision", Property<int>(coordinates, "CalibrationRevision"));
        Call(_controller, "SetTileCount", 6);
        Call(_controller, "SetOffsetIndex", 1);
        Call(_controller, "SetMortarMillimeters", 8);
        Assert.That(Property<bool>(_controller, "CanPlan"), Is.True);
        Call(_controller, "Plan");
        Assert.That(Property<bool>(_controller, "IsPlanning"), Is.True, Property<string>(_controller, "StatusText"));
        Assert.That(Field(_controller, "PlanPublishCount").GetValue(_controller), Is.EqualTo(1));
        object request = Field(_controller, "LastRequest").GetValue(_controller);
        _requestId = (string)Field(request, "request_id").GetValue(request);
        Assert.That(_requestId, Is.Not.Empty);
        Assert.That(Field(request, "tile_count").GetValue(request), Is.EqualTo(6u));
        Assert.That(Field(request, "row_offset").GetValue(request), Is.EqualTo(1));
        Assert.That(Field(request, "mortar_mm").GetValue(request), Is.EqualTo(8));
    }

    [TearDown]
    public void CleanUpWithoutTransport()
    {
        if (_controller != null)
        {
            TypeOf("RobotExecutionState").GetMethod("SetBusy").Invoke(null, new object[] { _controller, false });
            Call(_controller, "SetTaskActive", false);
            Component capture = Property<Component>(_controller, "SurfaceCapture");
            if (capture != null) Call(capture, "OnDestroy");
            var review = (Component)Field(_controller, "_review").GetValue(_controller);
            if (review != null) Call(review, "OnDestroy");
            var source = (Component)Field(_controller, "_sourcePreview").GetValue(_controller);
            if (source != null) Call(source, "OnDestroy");
        }
        if (_visualizer != null) Call(_visualizer, "SetTrajectory", new object[] { null });
        foreach (GameObject value in _owned.AsEnumerable().Reverse())
            if (value != null) UnityEngine.Object.DestroyImmediate(value);
        _owned.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("PickSurfaceGuide")), Is.Empty);
    }

    [Test]
    public void ABackendWithoutTheDirectTaskContractReceivesNoNewPlan()
    {
        Status("failed");
        Set(_controller, "TaskVersion", 1L);
        Call(_controller, "Plan");
        Assert.That(Field(_controller, "PlanPublishCount").GetValue(_controller), Is.EqualTo(1));
        Assert.That(Property<bool>(_controller, "IsPlanning"), Is.False);
        Assert.That(Property<string>(_controller, "StatusText"), Does.Contain("Update ROS"));
    }

    [Test]
    public void ExecutionRequiresAllThreeOutputsToMatchTheCurrentRequest()
    {
        Status("ready", PlanId, "foreign-request");
        Scene("another-plan");
        Solution(PlanId);
        Assert.That(Property<bool>(_controller, "CanExecute"), Is.False);
        Status("ready");
        Assert.That(Property<bool>(_controller, "CanExecute"), Is.False);
        Scene(PlanId);
        Assert.That(Property<bool>(_controller, "CanExecute"), Is.True, Property<string>(_controller, "StatusText"));
        Assert.That(Property<int>(_controller, "ReviewStepCount"), Is.EqualTo(2));
        Call(_controller, "StepReview", 1);
        Assert.That(Property<int>(_controller, "ReviewStepIndex"), Is.EqualTo(1));
        Assert.That(ExecuteCount, Is.Zero);
    }

    [TestCase(true)]
    [TestCase(false)]
    public void MortarOrTrackingChangeRevokesReviewAndLateMessagesCannotRestoreIt(bool changeMortar)
    {
        Ready();
        if (changeMortar) Call(_controller, "SetMortarMillimeters", 9);
        else Call(_controller, "CheckTrackingSpace", New("Replacement tracking space").transform);
        Assert.That(Property<int>(_controller, "ReviewStepCount"), Is.Zero);
        Status("ready"); Scene(PlanId); Solution(PlanId);
        Assert.That(Property<bool>(_controller, "CanExecute"), Is.False);
        Call(_controller, "Execute");
        Assert.That(ExecuteCount, Is.Zero);
    }

    [Test]
    public void ExecuteConsumesTheMatchingPlanOnceAndIgnoresForeignExecutionStatus()
    {
        Ready();
        Call(_controller, "Execute");
        Call(_controller, "Execute");
        Assert.That(ExecuteCount, Is.EqualTo(1));
        Assert.That(Field(_controller, "ExecutedPlanId").GetValue(_controller), Is.EqualTo(PlanId));
        Status("succeeded", "foreign-plan");
        Assert.That(IsExecutionBusy, Is.True);
        Status("succeeded");
        Assert.That(IsExecutionBusy, Is.False);
        Assert.That(Property<bool>(_controller, "CanExecute"), Is.False);
        Assert.That(Property<bool>(_controller, "CanPlan"), Is.False, "The completed batch consumed the captured stack.");
        Assert.That(Property<string>(_controller, "StatusText"), Does.Contain("Capture the top tile"));
        Call(_controller, "Execute");
        Assert.That(ExecuteCount, Is.EqualTo(1));
    }

    private int ExecuteCount => (int)Field(_controller, "ExecutePublishCount").GetValue(_controller);

    [Test]
    public void PhysicalCentreAndSupportPointUseTheRegisteredTipAndNeedNoScan()
    {
        Ready();
        Call(_controller, "SetTileSideMeters", .32f);
        Call(_controller, "SetTileThicknessMeters", .012f);
        Component registration = Add("Direct capture contact tip", TypeOf("RobotPointRegistration"));
        Vector3 offset = new Vector3(.012f, -.023f, .041f);
        Call(registration, "SetTipOffset", offset);
        Transform tracking = _owned.Single(owner => owner.name == "Tracking space").transform;
        Quaternion pointerRotation = Quaternion.Euler(24f, -38f, 61f);
        Vector3 sourceCentre = new Vector3(.2f, .3f, .4f);

        Call(_controller, "BeginSourceCenterCapture");
        Assert.That(Property<bool>(_controller, "IsCapturingPoint"), Is.True);
        Assert.That(Call(_controller, "CapturePointerPose", new Pose(sourceCentre - pointerRotation * offset, pointerRotation), tracking), Is.EqualTo(true));
        object[] source = { Pose.identity, null };
        Assert.That(Call(_controller, "TryGetSource", source), Is.EqualTo(true));
        Assert.That(Vector3.Distance(((Pose)source[0]).position, sourceCentre), Is.LessThan(1e-6));
        Assert.That((Vector3[])source[1], Is.Empty);
        Assert.That(Property<bool>(_controller, "SourceFromCorners"), Is.False);
        Assert.That(Property<float>(_controller, "TileSideMeters"), Is.EqualTo(.32f));
        Assert.That(Property<float>(_controller, "TileThicknessMeters"), Is.EqualTo(.012f));

        Vector3 supportCorner = new Vector3(.5f, 0f, .8f);
        Call(_controller, "BeginPlacementCapture");
        Assert.That(Call(_controller, "CapturePointerPose", new Pose(supportCorner - pointerRotation * offset, pointerRotation), tracking), Is.EqualTo(true));
        object[] anchor = { Pose.identity };
        Assert.That(Call(_controller, "TryGetAnchorWorldPose", anchor), Is.EqualTo(true));
        Assert.That(Vector3.Distance(((Pose)anchor[0]).position, supportCorner), Is.LessThan(1e-6), "The anchor is the tile's bottom support surface.");
        Assert.That(Property<bool>(_controller, "IsScanRequested"), Is.False);
        Assert.That(Property<bool>(_controller, "CanPlan"), Is.True);
    }

    [Test]
    public void ThreeCornerCaptureUsesTheEditedDimensionsAndRetainsRawContacts()
    {
        Ready();
        Call(_controller, "SetTileSideMeters", .32f);
        Call(_controller, "SetTileThicknessMeters", .012f);
        Assert.That(Property<bool>(_controller, "IsSourceCurrent"), Is.False);
        object capture = Property<object>(_controller, "SurfaceCapture");
        Transform tracking = _owned.Single(owner => owner.name == "Tracking space").transform;
        var corners = new[] { new Vector3(.16f, .2f, -.16f), new Vector3(.16f, .2f, .16f), new Vector3(-.16f, .2f, .16f) };
        Call(_controller, "BeginCapture");
        foreach (Vector3 corner in corners)
            Assert.That(Call(capture, "CapturePointerPose", new Pose(corner, Quaternion.identity), tracking), Is.EqualTo(true), Property<string>(capture, "StatusText"));
        object[] source = { Pose.identity, null };
        Assert.That(Call(_controller, "TryGetSource", source), Is.EqualTo(true));
        Assert.That(Property<bool>(_controller, "SourceFromCorners"), Is.True);
        Assert.That(Vector3.Distance(((Pose)source[0]).position, new Vector3(0f, .2f, 0f)), Is.LessThan(1e-5));
        Vector3[] captured = (Vector3[])source[1];
        Assert.That(captured.Length, Is.EqualTo(3));
        for (int index = 0; index < corners.Length; index++)
            Assert.That(Vector3.Distance(captured[index], corners[index]), Is.LessThan(1e-6));
    }

    [Test]
    public void CancellingAnArmedPointPreservesTheAcceptedSourceAndDimensionsRevokeIt()
    {
        Ready();
        Call(_controller, "BeginSourceCenterCapture");
        Call(_controller, "CancelPointCapture");
        Assert.That(Property<bool>(_controller, "IsCapturingPoint"), Is.False);
        Assert.That(Property<bool>(_controller, "IsSourceCurrent"), Is.True);
        Call(_controller, "SetTileSideMeters", .3f);
        Assert.That(Property<bool>(_controller, "IsSourceCurrent"), Is.False);
        Assert.That(Property<bool>(_controller, "CanExecute"), Is.False);
        Assert.That(ExecuteCount, Is.Zero);
    }

    [Test]
    public void PlacementRechecksItsSourceAtTheTriggerAndPreservesThePreviousAnchorOnExpiry()
    {
        Ready();
        Pose previous = (Pose)Read(_controller, "_anchorInTracking");
        Transform tracking = _owned.Single(owner => owner.name == "Tracking space").transform;
        Call(_controller, "BeginPlacementCapture");
        object capture = Property<object>(_controller, "SurfaceCapture");
        Set(capture, "_surfaceTrackingInvalidated", true);
        Assert.That(Call(_controller, "CapturePointerPose", new Pose(Vector3.one, Quaternion.identity), tracking), Is.EqualTo(false));
        Assert.That(Property<bool>(_controller, "IsCapturingPoint"), Is.False);
        Assert.That(Property<string>(_controller, "StatusText"), Does.Contain("no longer current"));
        Assert.That(((Pose)Read(_controller, "_anchorInTracking")).position, Is.EqualTo(previous.position));
        Assert.That(Property<bool>(_controller, "CanExecute"), Is.False);
    }
    private static bool IsExecutionBusy => (bool)TypeOf("RobotExecutionState").GetProperty("IsBusy").GetValue(null);

    private void Ready()
    {
        Status("ready"); Scene(PlanId); Solution(PlanId);
        Assert.That(Property<bool>(_controller, "CanExecute"), Is.True, Property<string>(_controller, "StatusText"));
    }

    private void Status(string status, string planId = PlanId, string requestId = null)
    {
        string json = "{\"status\":\"" + status + "\",\"plan_id\":\"" + planId +
            "\",\"request_id\":\"" + (requestId ?? _requestId) + "\",\"reason\":\"offline test\"}";
        object message = Activator.CreateInstance(TypeOf("RosMessageTypes.Std.StringMsg"), new object[] { json });
        Call(_controller, "OnStatus", message);
    }

    private void Scene(string id) => Call(_controller, "OnScene", BuildScene(id));

    private void Solution(string id)
    {
        object solution = Message("MoveitTaskConstructor.SolutionMsg");
        Set(solution, "task_id", id);
        Set(solution, "start_scene", BuildScene(id));
        object segment = Message("MoveitTaskConstructor.SubTrajectoryMsg");
        object joint = Read(Read(segment, "trajectory"), "joint_trajectory");
        Set(joint, "joint_names", new[] { "ur_shoulder_pan_joint" });
        object first = Message("Trajectory.JointTrajectoryPointMsg"), last = Message("Trajectory.JointTrajectoryPointMsg");
        Set(first, "positions", new[] { 0.0 });
        Set(last, "positions", new[] { 0.1 });
        Set(Read(last, "time_from_start"), "sec", 1);
        SetArray(joint, "points", first, last);
        Set(Read(segment, "scene_diff"), "is_diff", true);
        SetArray(solution, "sub_trajectory", segment);
        Call(_controller, "OnSolution", solution);
    }

    private static object BuildScene(string id)
    {
        object scene = Message("Moveit.PlanningSceneMsg");
        Set(scene, "name", id);
        return scene;
    }

    private GameObject New(string name)
    {
        var result = new GameObject(name);
        result.SetActive(false);
        _owned.Add(result);
        return result;
    }

    private Component Add(string name, Type type)
    {
        Component result = New(name).AddComponent(type);
        ((Behaviour)result).enabled = false;
        result.gameObject.SetActive(true);
        return result;
    }

    private static object Message(string name) => Activator.CreateInstance(TypeOf("RosMessageTypes." + name));
    private static Type FindType(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType(name)).FirstOrDefault(t => t != null);
    private static Type TypeOf(string name) => FindType(name) ?? throw new TypeLoadException(name);
    private const BindingFlags MemberFlags = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.DeclaredOnly;

    private static FieldInfo Field(object target, string name)
    {
        for (Type type = target.GetType(); type != null; type = type.BaseType)
        {
            FieldInfo field = type.GetField(name, MemberFlags);
            if (field != null) return field;
        }
        throw new MissingFieldException(name);
    }

    private static object Call(object target, string name, params object[] args)
    {
        for (Type type = target.GetType(); type != null; type = type.BaseType)
        {
            MethodInfo method = type.GetMethods(MemberFlags).SingleOrDefault(m => m.Name == name && m.GetParameters().Length == args.Length);
            if (method != null) return method.Invoke(target, args);
        }
        throw new MissingMethodException(name);
    }

    private static object Read(object target, string name) => Field(target, name).GetValue(target);
    private static void Set(object target, string name, object value) => Field(target, name).SetValue(target, value);
    private static T Property<T>(object target, string name) => (T)target.GetType().GetProperty(name).GetValue(target);

    private static void SetArray(object target, string name, params object[] entries)
    {
        FieldInfo field = Field(target, name);
        Array array = Array.CreateInstance(field.FieldType.GetElementType(), entries.Length);
        for (int i = 0; i < entries.Length; i++) array.SetValue(entries[i], i);
        field.SetValue(target, array);
    }
}
