using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>Actual identified fixture review and Execute admission without obtaining ROS transport.</summary>
public sealed class FixtureTilingWorkflowTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private Component _controller, _visualizer, _coordinates, _tools;
    private object _scene, _solution;
    private object _publishedTimedPlan;
    private const string Request = "fixture-test-request", Plan = "fixture-test-plan";

    [SetUp]
    public void SetupActualControllerAndRobotPreview()
    {
        _coordinates = Add("Fixture-test coordinates", "RobotCoordinateService");
        Add("Fixture-test state", "RobotStateService");
        ArticulationBody body = New("Fixture-test base").AddComponent<ArticulationBody>();
        Call(_coordinates, "Initialize", body, New("Fixture-test tracking").transform);
        _tools = Add("Fixture-test matching tool", "ToolSelectionController");
        if (Read(_tools, "_catalog") == null) Private(_tools, "Awake");
        string urdf = Resources.Load<TextAsset>("RobotModels/kenos_sl.urdf").text;
        foreach (string service in new[] { "/robot_state_publisher/get_parameters", "/move_group/get_parameters", "/mtc_test_task_node/get_parameters" })
            Assert.That(Call(_tools, "InspectDescription", service, urdf), Is.EqualTo(true));
        _visualizer = Add("Fixture-test shared path", "UR16eTrajectoryVisualizer");
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos_transparent.prefab");
        GameObject robot = UnityEngine.Object.Instantiate(prefab); _owned.Add(robot); robot.SetActive(false);
        Component rig = robot.AddComponent(TypeOf("RosRobotRig"));
        Assert.That(Call(rig, "ApplyModel", Get<object>(_tools, "CurrentModel")), Is.EqualTo(true));
        Set(_visualizer, "virtualUr16ePrefab", prefab);
        Private(_visualizer, "RecordRobot", robot, Pose.identity);
        _controller = Add("Fixture-test offline controller", "OfflineFixtureTilingController");
        Set(_controller, "_visualizer", _visualizer);
        Set(_controller, "_active", true);
        _scene = Msg("Moveit.PlanningSceneMsg"); Set(_scene, "name", Plan);
        _solution = Msg("MoveitTaskConstructor.SolutionMsg"); Set(_solution, "task_id", Plan); Set(_solution, "start_scene", _scene);
        object segment = Msg("MoveitTaskConstructor.SubTrajectoryMsg");
        object joint = Read(Read(segment, "trajectory"), "joint_trajectory");
        Set(joint, "joint_names", new[] { "ur_shoulder_pan_joint" });
        object first = Msg("Trajectory.JointTrajectoryPointMsg"), last = Msg("Trajectory.JointTrajectoryPointMsg");
        Set(first, "positions", new[] { 0.0 }); Set(last, "positions", new[] { 0.1 });
        Set(Read(last, "time_from_start"), "sec", 1);
        ArrayField(joint, "points", first, last);
        Set(Read(segment, "scene_diff"), "is_diff", true);
        object settle = Msg("MoveitTaskConstructor.SubTrajectoryMsg");
        Set(Read(settle, "info"), "comment", "vacuum off; timed settling assumption");
        Set(Read(settle, "scene_diff"), "is_diff", true);
        ArrayField(_solution, "sub_trajectory", segment, settle);
        Arm();
    }

    [TearDown]
    public void CleanupAndProveNoTransport()
    {
        if (_visualizer != null) Call(_visualizer, "SetTrajectory", new object[] { null });
        foreach (GameObject value in _owned.AsEnumerable().Reverse()) if (value != null) UnityEngine.Object.DestroyImmediate(value);
        _owned.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
    }

    [TestCase("RSL")]
    [TestCase("RLS")]
    [TestCase("SRL")]
    [TestCase("SLR")]
    [TestCase("LRS")]
    [TestCase("LSR")]
    public void OnlyAllThreeMatchingOutputsArmExecutionInEveryDeliveryOrder(string order)
    {
        // R=identified ready, S=full scene, L=identified aggregate solution.
        for (int i = 0; i < order.Length; i++)
        {
            Deliver(order[i]);
            Assert.That(Get<bool>(_controller, "CanExecute"), Is.EqualTo(i == 2), order + " at " + i);
        }
        Assert.That(Get<int>(_controller, "ReviewStepCount"), Is.EqualTo(3));
        Call(_controller, "StepReview", 2);
        Assert.That(Get<string>(_controller, "ReviewStepText"), Does.Contain("settling"));
        Assert.That((int)Read(_controller, "PublishCount"), Is.Zero, "Planning/review cannot execute.");
    }

    [Test]
    public void ForeignOrMissingOutputsNeverArmAndLateCallbacksCannotRestoreEditedReview()
    {
        Status("ready", Plan, "foreign-request");
        Deliver('S'); Deliver('L'); Assert.That(Get<bool>(_controller, "CanExecute"), Is.False);
        Private(_controller, "InvalidateReview", new object[] { null }); Arm();
        Set(_scene, "name", "old-plan"); Set(_solution, "task_id", "old-plan");
        Status("ready"); Deliver('S'); Deliver('L'); Assert.That(Get<bool>(_controller, "CanExecute"), Is.False);
        Set(_scene, "name", Plan); Set(_solution, "task_id", Plan); Deliver('S'); Deliver('L');
        Assert.That(Get<bool>(_controller, "CanExecute"), Is.True);
        Call(_controller, "NotifyEdited");
        Status("ready"); Deliver('S'); Deliver('L');
        Assert.That(Get<bool>(_controller, "CanExecute") || Get<bool>(_controller, "IsPlanning"), Is.False);
    }

    [TestCase("edit")]
    [TestCase("mode")]
    [TestCase("calibration")]
    public void ChangesRevokeTheVisiblePlanAndItsExecutionPermission(string change)
    {
        Ready();
        if (change == "edit") Call(_controller, "NotifyEdited");
        else if (change == "mode") Call(_controller, "SetTaskActive", false);
        else Private(_controller, "OnRobotCalibrationChanged");
        Assert.That(Get<bool>(_controller, "CanExecute"), Is.False);
        Assert.That(Read(_controller, "_trajectory"), Is.Null);
        Call(_controller, "Execute"); Assert.That(Read(_controller, "PublishCount"), Is.EqualTo(0));
    }

    [Test]
    public void FittedSourceSurfaceSetsTheStackCenterAndYawAndRevokesPreviousReview()
    {
        Ready();
        Pose local = new Pose(new Vector3(.2f, .3f, .4f), Quaternion.Euler(0f, 30f, 0f));
        Pose world = (Pose)Call(_coordinates, "ConvertRootLocalPoseToWorldPose", local);
        Assert.That(Call(_controller, "AcceptSourceSurface", world), Is.EqualTo(true));
        object source = Read(Get<object>(_controller, "Settings"), "SourceTopPose");
        object position = Read(Read(source, "pose"), "position");
        Assert.That(Read(position, "x"), Is.EqualTo(.4).Within(1e-6));
        Assert.That(Read(position, "y"), Is.EqualTo(-.2).Within(1e-6));
        Assert.That(Read(position, "z"), Is.EqualTo(.3).Within(1e-6));
        object rotation = Read(Read(source, "pose"), "orientation");
        Assert.That(Read(rotation, "z"), Is.EqualTo(-Math.Sin(Math.PI / 12)).Within(1e-6));
        Assert.That(Get<bool>(_controller, "CanExecute"), Is.False);
        Assert.That(Get<string>(_controller, "StatusText"), Does.Contain("green stack"));
        Assert.That(Read(_controller, "PublishCount"), Is.EqualTo(0));
    }

    [Test]
    public void SteepSourceSurfaceCannotReplaceAnAcceptedStackPose()
    {
        Ready();
        Pose horizontal = (Pose)Call(_coordinates, "ConvertRootLocalPoseToWorldPose", new Pose(Vector3.up, Quaternion.identity));
        Assert.That(Call(_controller, "AcceptSourceSurface", horizontal), Is.EqualTo(true));
        object source = Read(Get<object>(_controller, "Settings"), "SourceTopPose");
        Pose tilted = (Pose)Call(_coordinates, "ConvertRootLocalPoseToWorldPose", new Pose(Vector3.zero, Quaternion.Euler(30f, 0f, 0f)));
        Assert.That(Call(_controller, "AcceptSourceSurface", tilted), Is.EqualTo(false));
        Assert.That(Read(Get<object>(_controller, "Settings"), "SourceTopPose"), Is.SameAs(source));
        Assert.That(Get<string>(_controller, "StatusText"), Does.Contain("horizontal"));
    }

    [Test]
    public void OnePointCentreCaptureSetsTheStackPositionAndShowsThePickCentre()
    {
        Ready();
        object target = Enum.Parse(TypeOf("FixtureCaptureTarget"), "SourceTop");
        Call(_controller, "BeginCapture", target);
        Assert.That(Get<string>(_controller, "CaptureInstruction"), Does.Contain("source tile top centre"));
        Pose world = (Pose)Call(_coordinates, "ConvertRootLocalPoseToWorldPose", new Pose(new Vector3(.2f, .3f, .4f), Quaternion.identity));
        Transform tracking = New("Centre contact tracking").transform;
        Assert.That(Call(_controller, "CapturePointerPose", world, tracking), Is.EqualTo(true));
        object source = Read(Get<object>(_controller, "Settings"), "SourceTopPose");
        object position = Read(Read(source, "pose"), "position");
        Assert.That(Read(position, "x"), Is.EqualTo(.4).Within(1e-6));
        Assert.That(Read(position, "y"), Is.EqualTo(-.2).Within(1e-6));
        Assert.That(Read(position, "z"), Is.EqualTo(.3).Within(1e-6));
        Component preview = (Component)Read(_controller, "_preview");
        GameObject root = (GameObject)Read(preview, "_root");
        Assert.That(Vector3.Distance(root.transform.Find("Source top centre").localPosition, new Vector3(.2f, .3f, .4f)), Is.LessThan(1e-6));
        Assert.That(root.transform.Find("Source pick centre cross X"), Is.Not.Null);
        Assert.That(root.transform.Find("Source pick centre cross Z"), Is.Not.Null);
        Assert.That(Get<bool>(_controller, "CanExecute"), Is.False);
    }

    [Test]
    public void PlacementCapturesReportAndDisplayEachPointBeforeThePatternIsComplete()
    {
        Ready();
        var points = new[] { new Vector3(.2f, .3f, .4f), new Vector3(.4f, .3f, .4f), new Vector3(.2f, .3f, .6f) };
        string[] targets = { "PlacementCorner", "PlacementX", "PlacementY" };
        string[] summaries = { "Corner ✓ · +X – · +Y –", "Corner ✓ · +X ✓ · +Y –", "Corner ✓ · +X ✓ · +Y ✓" };
        Transform tracking = New("Placement contact tracking").transform;
        for (int index = 0; index < points.Length; index++)
        {
            Call(_controller, "BeginCapture", Enum.Parse(TypeOf("FixtureCaptureTarget"), targets[index]));
            Pose world = (Pose)Call(_coordinates, "ConvertRootLocalPoseToWorldPose", new Pose(points[index], Quaternion.identity));
            Assert.That(Call(_controller, "CapturePointerPose", world, tracking), Is.EqualTo(true));
            Assert.That(Get<string>(_controller, "PlacementCaptureSummary"), Is.EqualTo(summaries[index]));
            Assert.That(Get<string>(_controller, "StatusText"), Does.StartWith("Captured"));
            Component preview = (Component)Read(_controller, "_preview");
            GameObject root = (GameObject)Read(preview, "_root");
            if (index == 0) Assert.That(root.transform.Find("Final placement corner"), Is.Not.Null);
            if (index == 1) Assert.That(root.transform.Find("Captured placement +X"), Is.Not.Null);
            if (index == 2) Assert.That(root.transform.Find("Destination tile 1"), Is.Not.Null);
        }
    }

    [TestCase("SourceTop")]
    [TestCase("PlacementCorner")]
    [TestCase("Board00")]
    public void NamedFixturePointUsesTheRegisteredTipBeforeConvertingToRobotCoordinates(string targetName)
    {
        Ready();
        Component registration = Add("Fixture contact tip", "RobotPointRegistration");
        Vector3 offset = new Vector3(.012f, -.023f, .041f);
        Call(registration, "SetTipOffset", offset);
        Transform tracking = New("Rotated fixture tracking").transform;
        tracking.SetPositionAndRotation(new Vector3(.4f, -.2f, .7f), Quaternion.Euler(11f, 47f, -6f));
        Pose desiredContact = (Pose)Call(_coordinates, "ConvertRootLocalPoseToWorldPose",
            new Pose(new Vector3(.2f, .3f, .4f), Quaternion.identity));
        Quaternion pointerRotation = Quaternion.Euler(24f, -38f, 61f);
        Pose pointer = new Pose(desiredContact.position - pointerRotation * offset, pointerRotation);
        Call(_controller, "BeginCapture", Enum.Parse(TypeOf("FixtureCaptureTarget"), targetName));
        Assert.That(Call(_controller, "CapturePointerPose", pointer, tracking), Is.EqualTo(true));
        object point;
        if (targetName == "SourceTop") point = Read(Read(Read(Get<object>(_controller, "Settings"), "SourceTopPose"), "pose"), "position");
        else if (targetName == "PlacementCorner") point = Read(_controller, "_placementOrigin");
        else point = ((Array)Read(Get<object>(_controller, "Calibration"), "BoardTopCorners")).GetValue(0);
        Assert.That(Read(point, "x"), Is.EqualTo(.4).Within(1e-6));
        Assert.That(Read(point, "y"), Is.EqualTo(-.2).Within(1e-6));
        Assert.That(Read(point, "z"), Is.EqualTo(.3).Within(1e-6));
    }

    [Test]
    public void ExecuteConsumesOnceAndUnknownAcknowledgementRemainsLockedUntilMatchingTerminal()
    {
        Ready(); Call(_controller, "Execute"); Call(_controller, "Execute");
        Assert.That(Read(_controller, "PublishCount"), Is.EqualTo(1));
        Assert.That(Read(_controller, "LastPlanId"), Is.EqualTo(Plan));
        Assert.That(Get<bool>(_controller, "CanExecute"), Is.False);
        Set(_controller, "_deadline", float.NegativeInfinity); Private(_controller, "Update");
        Assert.That(Get<bool>(_controller, "IsExecuting"), Is.True);
        Assert.That(Get<string>(_controller, "StatusText"), Does.Contain("unknown").IgnoreCase);
        Call(_controller, "NotifyEdited"); Call(_controller, "SetTaskActive", false); Call(_controller, "Execute");
        Assert.That(Get<bool>(_controller, "IsExecuting"), Is.True);
        Status("succeeded", "foreign-plan"); Assert.That(Get<bool>(_controller, "IsExecuting"), Is.True);
        Status("executing"); Status("succeeded");
        Assert.That(Get<bool>(_controller, "IsExecuting"), Is.False);
        Status("ready"); Deliver('S'); Deliver('L'); Call(_controller, "Execute");
        Assert.That(Read(_controller, "PublishCount"), Is.EqualTo(1));
    }

    [Test]
    public void IdentifiedPreflightRejectionCanReleaseTheLockWithoutARequestId()
    {
        Ready(); Call(_controller, "Execute");
        Status("rejected", "foreign-plan", ""); Assert.That(Get<bool>(_controller, "IsExecuting"), Is.True);
        Status("rejected", Plan, ""); Assert.That(Get<bool>(_controller, "IsExecuting"), Is.False);
        Assert.That(Get<string>(_controller, "StatusText"), Does.Contain("rejected"));
    }

    [Test]
    public void StopDuringFixtureCalibrationCheckDiscardsThePendingRequestAndAllowsRetry()
    {
        Assert.That(_controller.GetType().GetMethod("StopPlanning"), Is.Not.Null, "Fixture planning needs a Stop Planning action.");
        Set(_controller, "_acceptingResults", false);
        Set(_controller, "_checkingCalibration", true);
        int generation = (int)Read(_controller, "_generation");
        Call(_controller, "StopPlanning");
        Assert.That(Get<bool>(_controller, "IsPlanning"), Is.False);
        Assert.That(Read(_controller, "_requestId"), Is.Null);
        string stopped = Get<string>(_controller, "StatusText");
        Assert.That(stopped, Does.Contain("stopped").IgnoreCase);
        Private(_controller, "OnCalibrationParameters", null, generation);
        Assert.That(Get<string>(_controller, "StatusText"), Is.EqualTo(stopped));
        Assert.That(Read(_controller, "PlanPublishCount"), Is.EqualTo(0));
        Assert.That(Read(_controller, "PublishCount"), Is.EqualTo(0));
    }

    [Test]
    public void PublishedFixtureStopClearsItsReviewAndWaitsForItsOwnTerminal()
    {
        object response = CalibrationResponse("matching", true);
        Component transport = Add("Offline fixture stop messages", "Unity.Robotics.ROSTCPConnector.ROSConnection");
        Set(_controller, "_connection", transport);
        object request = Msg("Ur16eInterfaces.FixtureTilingTaskMsg");
        Set(request, "request_id", Request);
        Arm();
        Set(_controller, "_request", request);
        Set(_controller, "_acceptingResults", false);
        Set(_controller, "_checkingCalibration", true);
        Private(_controller, "OnCalibrationParameters", response, Read(_controller, "_generation"));
        Assert.That(Read(_controller, "PlanPublishCount"), Is.EqualTo(1));
        Assert.That(Get<bool>(_controller, "CanStopPlanning"), Is.True);
        Deliver('R'); Deliver('S'); Deliver('L');
        Assert.That(Get<int>(_controller, "ReviewStepCount"), Is.GreaterThan(0));
        Assert.That(Get<bool>(_controller, "CanExecute"), Is.False, "ROS still owns the planning slot.");

        Call(_controller, "StopPlanning");
        Assert.That(Get<bool>(_controller, "IsStoppingPlanning"), Is.True);
        Deliver('R'); Deliver('S'); Deliver('L');
        Assert.That(Get<int>(_controller, "ReviewStepCount"), Is.Zero);
        Assert.That(Get<bool>(_visualizer, "HasPlayback"), Is.False);
        PlanningControl("fixture/foreign-request", "cancelled");
        Assert.That(Get<bool>(_controller, "IsPlanning"), Is.True);
        PlanningControl("fixture/" + Request, "cancelled");
        Assert.That(Get<bool>(_controller, "IsPlanning") || Get<bool>(_controller, "CanExecute"), Is.False);
        Assert.That(Read(_controller, "PublishCount"), Is.EqualTo(0));
    }

    [Test]
    public void TimedFixturePublishesOneBudgetForTheWholeBatch()
    {
        Component actual = Add("Timed fixture controller", "FixtureTilingController");
        Component transport = Add("Offline fixture timed transport", "Unity.Robotics.ROSTCPConnector.ROSConnection");
        Set(actual, "_connection", transport);
        Set(actual, "_requestHasTimeout", true);
        Set(actual, "_requestedTimeoutSeconds", 85);
        object fixture = Msg("Ur16eInterfaces.FixtureTilingTaskMsg");
        Set(fixture, "request_id", Request);
        Set(fixture, "item_count", 5u);
        const string topicName = "/mtc/plan_request";
        MethodInfo register = transport.GetType().GetMethods().Single(method =>
            method.Name == "RegisterPublisher" && method.IsGenericMethodDefinition);
        object topic = register.MakeGenericMethod(TypeOf("RosMessageTypes.Ur16eInterfaces.MtcPlanRequestMsg"))
            .Invoke(transport, new object[] { topicName, null, null });
        MethodInfo subscribe = topic.GetType().GetMethod("AddSubscriber");
        MethodInfo capture = GetType().GetMethod(nameof(CaptureTimedPlan), BindingFlags.Instance | BindingFlags.NonPublic);
        subscribe.Invoke(topic, new[] { Delegate.CreateDelegate(subscribe.GetParameters()[0].ParameterType, this, capture) });
        _publishedTimedPlan = null;

        Assert.That(Private(actual, "TryPublishPlan", fixture, null), Is.EqualTo(true));

        Assert.That(_publishedTimedPlan, Is.Not.Null);
        Assert.That(Read(_publishedTimedPlan, "mode"), Is.EqualTo(3));
        Assert.That(Read(_publishedTimedPlan, "timeout_seconds"), Is.EqualTo(85));
        Assert.That(Read(_publishedTimedPlan, "fixture"), Is.SameAs(fixture));
    }

    private void CaptureTimedPlan(object message) => _publishedTimedPlan = message;

    [Test]
    public void StartingOutsideFixtureModeDoesNotRequestOrRegisterRosTransport()
    {
        Set(_controller, "_active", false);
        Private(_controller, "Start");
        Assert.That(Read(_controller, "_hasStarted"), Is.EqualTo(true));
        Assert.That(Read(_controller, "_connection"), Is.Null,
            "The offline ROS factory throws; inactive startup must never request it.");
        Assert.That(Read(_controller, "PublishCount"), Is.EqualTo(0));
        Assert.That(Read(_controller, "PlanPublishCount"), Is.EqualTo(0));
    }

    [TestCase("matching", true)]
    [TestCase("opposite quaternion", true)]
    [TestCase("disabled", false)]
    [TestCase("foreign frame", false)]
    [TestCase("different board", false)]
    [TestCase("nonfinite thickness", false)]
    public void PlanningRequiresTheLoadedRosCalibrationToMatchTheMeasuredFixture(string variant, bool expected)
    {
        Assert.That(Private(_controller, "RunningCalibrationMatches", CalibrationResponse(variant), null), Is.EqualTo(expected));
    }

    private object CalibrationResponse(string variant, bool supportsStop = false)
    {
        object calibration = Get<object>(_controller, "Calibration");
        object a = Msg("Geometry.PointMsg"), b = Msg("Geometry.PointMsg"), c = Msg("Geometry.PointMsg"), d = Msg("Geometry.PointMsg");
        Set(b, "x", 1.0); Set(c, "x", 1.0); Set(c, "y", 1.0); Set(d, "y", 1.0);
        ArrayField(calibration, "BoardTopCorners", a, b, c, d);
        object support = Msg("Geometry.PoseStampedMsg"); Set(Read(Read(support, "pose"), "orientation"), "w", 1.0);
        Set(calibration, "SupportCorner", support);
        object[] values = Enumerable.Range(0, supportsStop ? 9 : 8).Select(_ => Msg("RclInterfaces.ParameterValueMsg")).ToArray();
        byte[] kinds = { 1, 4, 8, 3, 8, 8, 3, 3 };
        for (int i = 0; i < kinds.Length; i++) Set(values[i], "type", kinds[i]);
        Set(values[0], "bool_value", variant != "disabled");
        Set(values[1], "string_value", variant == "foreign frame" ? "world" : "ur_base_link");
        Set(values[2], "double_array_value", new[] { 0.0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0 });
        if (variant == "different board") ((double[])Read(values[2], "double_array_value"))[0] = 0.01;
        Set(values[3], "double_value", variant == "nonfinite thickness" ? double.NaN : Convert.ToDouble(Read(calibration, "BoardThickness")));
        Set(values[4], "double_array_value", new[] { 0.0, 0, 0 });
        Set(values[5], "double_array_value", new[] { 0.0, 0, 0, variant == "opposite quaternion" ? -1.0 : 1.0 });
        Set(values[6], "double_value", Convert.ToDouble(Read(calibration, "Margin")));
        Set(values[7], "double_value", 60.0);
        if (supportsStop)
        {
            Set(values[8], "type", (byte)1);
            Set(values[8], "bool_value", true);
        }
        object response = Msg("RclInterfaces.GetParametersResponse"); ArrayField(response, "values", values);
        return response;
    }

    private void PlanningControl(string key, string status)
    {
        object message = Activator.CreateInstance(TypeOf("RosMessageTypes.Std.StringMsg"),
            new object[] { "{\"key\":\"" + key + "\",\"status\":\"" + status + "\",\"reason\":\"offline test\"}" });
        Private(Read(_controller, "_planningStop"), "OnStatus", message);
    }

    private void Arm()
    {
        Set(_controller, "_requestId", Request); Set(_controller, "_planId", null);
        Set(_controller, "_acceptingResults", true); Set(_controller, "_deadline", float.PositiveInfinity);
        Set(_controller, "_snapshot", Private(_controller, "CurrentSnapshot"));
        Set(_controller, "_calibrationRevision", Get<int>(_coordinates, "CalibrationRevision"));
        Set(_controller, "_requestModel", Get<object>(_tools, "CurrentModel"));
    }
    private void Ready() { Deliver('R'); Deliver('S'); Deliver('L'); Assert.That(Get<bool>(_controller, "CanExecute"), Is.True); }
    private void Deliver(char item) { if (item == 'R') Status("ready"); else Private(_controller, item == 'S' ? "OnScene" : "OnSolution", item == 'S' ? _scene : _solution); }
    private void Status(string status, string plan = Plan, string request = Request) => Private(_controller, "OnStatus", Activator.CreateInstance(TypeOf("RosMessageTypes.Std.StringMsg"), new object[] { "{\"status\":\"" + status + "\",\"plan_id\":\"" + plan + "\",\"request_id\":\"" + request + "\",\"reason\":\"fixture test\"}" }));
    private GameObject New(string name) { var value = new GameObject(name); value.SetActive(false); _owned.Add(value); return value; }
    private Component Add(string name, string type) { Component value = New(name).AddComponent(TypeOf(type)); ((Behaviour)value).enabled = false; value.gameObject.SetActive(true); return value; }
    private static object Msg(string name) => Activator.CreateInstance(TypeOf("RosMessageTypes." + name));
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType(name)).First(t => t != null);
    private static FieldInfo Field(object value, string name) { for (Type type = value.GetType(); type != null; type = type.BaseType) { var field = type.GetField(name, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.DeclaredOnly); if (field != null) return field; } throw new MissingFieldException(name); }
    private static object Read(object value, string name) => Field(value, name).GetValue(value);
    private static void Set(object value, string name, object data) => Field(value, name).SetValue(value, data);
    private static T Get<T>(object value, string name) => (T)value.GetType().GetProperty(name).GetValue(value);
    private static object Call(object value, string name, params object[] args) => value.GetType().GetMethods().Single(m => m.Name == name && m.GetParameters().Length == args.Length).Invoke(value, args);
    private static object Private(object value, string name, params object[] args) { for (Type type = value.GetType(); type != null; type = type.BaseType) { var method = type.GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.DeclaredOnly); if (method != null) return method.Invoke(value, args); } throw new MissingMethodException(name); }
    private static void ArrayField(object value, string name, params object[] entries) { var element = Field(value, name).FieldType.GetElementType(); Array array = Array.CreateInstance(element, entries.Length); for (int i = 0; i < entries.Length; i++) array.SetValue(entries[i], i); Set(value, name, array); }
}
