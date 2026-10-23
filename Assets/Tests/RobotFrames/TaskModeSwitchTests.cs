using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Actual task controllers run without Start, XR hardware or a ROS connection.
/// Marker roots are seeded; MTC preview tests use the unchanged real robot prefab,
/// model and trajectory visualizer. Separate audits cover marker geometry and UI.
/// </summary>
public sealed class TaskModeSwitchTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private Component _input, _surfaces, _tcp, _targets, _mtc, _visualizer, _coordinates, _state;
    private Transform _tracking, _tcpA, _tcpB, _aim, _pick, _place;
    private TcpPlanReview _review;

    [SetUp]
    public void CreateActualControllersWithoutStartingTransport()
    {
        _coordinates = Add("Mode-test coordinates", "RobotCoordinateService");
        _state = Add("Mode-test readiness", "RobotStateService");
        _tracking = New("Mode-test tracking").transform;
        _tracking.SetPositionAndRotation(new Vector3(1f, 0.2f, -0.7f), Quaternion.Euler(0f, 24f, 0f));
        ArticulationBody body = New("base_link").AddComponent<ArticulationBody>();
        body.transform.SetPositionAndRotation(new Vector3(0.4f, 0.3f, 0.8f), Quaternion.Euler(0f, -18f, 0f));
        Call(_coordinates, "Initialize", body, _tracking);

        Component tools = Add("Mode-test matching suction tool", "ToolSelectionController");
        if (Field(tools, "_catalog").GetValue(tools) == null) Private(tools, "Awake");
        string urdf = Resources.Load<TextAsset>("RobotModels/kenos_sl.urdf").text;
        foreach (string service in new[] { "/robot_state_publisher/get_parameters", "/move_group/get_parameters", "/mtc_test_task_node/get_parameters" })
            Assert.That(Call(tools, "InspectDescription", service, urdf), Is.EqualTo(true));

        GameObject owner = New("Mode-test task owner");
        owner.SetActive(false);
        _input = Disabled(owner, "URInputManager");
        _surfaces = Disabled(owner, "PickPlaceController");
        _tcp = Disabled(owner, "TcpPlanningController");
        _targets = Disabled(owner, "TcpTargetPreview");
        _mtc = Add("Mode-test MTC review", "MTCPlansManager");
        if (Field(_mtc, "OnSuccessfulPlan").GetValue(_mtc) == null) Private(_mtc, "Awake");
        _visualizer = Add("Mode-test shared ghost", "UR16eTrajectoryVisualizer");
        Set(_input, "_pickPlaceController", _surfaces);
        Set(_input, "_tcpPlanningController", _tcp);
        Set(_input, "_tcpTargetPreview", _targets);
        Set(_surfaces, "_plansManager", _mtc);
        Set(_mtc, "_UR16eTrajectoryVisualizer", _visualizer);
        Set(_tcp, "_visualizer", _visualizer);
        Set(_tcp, "_targets", _targets);
        _review = (TcpPlanReview)Field(_tcp, "_review").GetValue(_tcp);

        _tcpA = Marker("_pick", "Retained TCP A", new Vector3(0.8f, 0.6f, 0.1f));
        _tcpB = Marker("_place", "Retained TCP B", new Vector3(-0.2f, 0.7f, 0.9f));
        _aim = Marker("_live", "Previous live aim", new Vector3(0.2f, 0.9f, 0.3f));
        _pick = New("Retained pick surface").transform;
        _place = New("Retained place surface").transform;
        _pick.SetPositionAndRotation(new Vector3(0.2f, 0.5f, 0.4f), Quaternion.Euler(12f, 23f, -8f));
        _place.SetPositionAndRotation(new Vector3(-0.5f, 0.4f, 0.7f), Quaternion.Euler(-7f, 14f, 31f));
        Set(_surfaces, "_pickPointInstance", _pick.gameObject);
        Set(_surfaces, "_placePointInstance", _place.gameObject);
        owner.SetActive(true);
        Call(_input, "UseTilePickPlace");
    }

    [TearDown]
    public void CleanupWithoutTransportOrDeferredRuntimeDestruction()
    {
        if (_targets != null)
            foreach (string name in new[] { "_live", "_pick", "_place" }) Set(_targets, name, null);
        DestroyPreviewClonesImmediately();
        for (int index = _owned.Count - 1; index >= 0; index--)
            if (_owned[index] != null) UnityEngine.Object.DestroyImmediate(_owned[index]);
        _owned.Clear();
        Type connection = TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection");
        Assert.That(Resources.FindObjectsOfTypeAll(connection), Is.Empty, "Mode tests must not create a ROS connection.");
    }

    [Test]
    public void SwitchingModesRetainsPosesAndShowsOnlyTheSelectedTargets()
    {
        Pose tcp = PoseOf(_tcpA), pick = PoseOf(_pick);
        Assert.That(_tcpA.gameObject.activeSelf, Is.False);
        Assert.That(_pick.gameObject.activeSelf, Is.True);
        Call(_input, "UseTcpPreview");
        Assert.That(Get<bool>(_input, "IsTcpMode"), Is.True);
        Assert.That(_tcpA.gameObject.activeSelf && _tcpB.gameObject.activeSelf, Is.True);
        Assert.That(_pick.gameObject.activeSelf || _place.gameObject.activeSelf, Is.False);
        Assert.That(_aim.gameObject.activeSelf, Is.False, "Old live aim must wait for a fresh tracked controller pose.");
        Assert.That(Get<bool>(_state, "IsReadyForPlanning"), Is.False);
        Call(_input, "UseTilePickPlace");
        Assert.That(_tcpA.gameObject.activeSelf || _tcpB.gameObject.activeSelf, Is.False);
        Assert.That(_pick.gameObject.activeSelf && _place.gameObject.activeSelf, Is.True);
        AssertPose(PoseOf(_tcpA), tcp);
        AssertPose(PoseOf(_pick), pick);
        Assert.That(Get<bool>(_state, "IsReadyForPlanning"), Is.True);
    }

    [Test]
    public void ModeSwitchRevokesBothPlansAndClearsTheSharedGhostPath()
    {
        Assert.That(Call(_mtc, "BeginPlanningRequest"), Is.EqualTo(true));
        MtcStatus(true);
        _review.Begin("mode-test:1");
        _review.AcceptResult("mode-test:1", true);
        _review.AcceptPreview("mode-test:1");
        Set(_surfaces, "_hasPlanningRequestBeenSent", true);
        LineRenderer path = New("Old shared path").AddComponent<LineRenderer>();
        path.positionCount = 2;
        Set(_visualizer, "trajectoryLineRenderer", path);
        Set(_visualizer, "_trajectory", Trajectory());
        Call(_input, "UseTcpPreview");
        Assert.That(_review.CanExecute || _review.IsPending, Is.False);
        Assert.That(Get<bool>(_mtc, "HasPlanResult") || Get<bool>(_mtc, "IsPlanning"), Is.False);
        Assert.That((bool)Field(_surfaces, "_hasPlanningRequestBeenSent").GetValue(_surfaces), Is.False);
        Assert.That(Field(_visualizer, "_trajectory").GetValue(_visualizer), Is.Null);
        Assert.That(path.positionCount, Is.Zero);
        Assert.That(Get<bool>(_mtc, "CanExecute") || Get<bool>(_tcp, "CanExecute"), Is.False);
    }

    [Test]
    public void SelectingTheCurrentModePreservesItsPendingReview()
    {
        Assert.That(Call(_mtc, "BeginPlanningRequest"), Is.EqualTo(true));
        Call(_input, "UseTilePickPlace");
        Assert.That(Get<bool>(_mtc, "IsPlanning"), Is.True);
        Call(_input, "UseTcpPreview");
        _review.Begin("mode-test:2");
        Call(_input, "UseTcpPreview");
        Assert.That(_review.RequestId, Is.EqualTo("mode-test:2"));
        Assert.That(_review.IsPending, Is.True);
    }

    [Test]
    public void ModeSwitchEndsAnActiveTargetRotation()
    {
        Component handles = Add("Selected surface handle", "TargetRotationHandles");
        ((Behaviour)handles).enabled = true;
        object free = Enum.Parse(TypeOf("TargetRotationHandles+RotationHandleKind"), "Free");
        Assert.That(Call(handles, "BeginRotation", free, Quaternion.identity, Vector3.zero), Is.EqualTo(true));
        Assert.That((bool)handles.GetType().GetProperty("IsInteracting").GetValue(null), Is.True);
        Call(_input, "UseTcpPreview");
        Assert.That((bool)handles.GetType().GetProperty("IsInteracting").GetValue(null), Is.False);
    }

    [Test]
    public void HiddenTcpTargetsRejectCaptureAndExportUntilTheirModeIsRestored()
    {
        Pose before = PoseOf(_tcpA), aim = PoseOf(_aim);
        var next = new Pose(new Vector3(9f, 8f, 7f), Quaternion.Euler(42f, 33f, 21f));
        Call(_targets, "SetPointerPose", next, _tracking);
        Call(_targets, "CapturePointerPose", next, _tracking, true);
        object[] exported = { true, null };
        Assert.That(CallArgs(_targets, "TryGetCapturedRosPose", exported), Is.EqualTo(false));
        AssertPose(PoseOf(_tcpA), before);
        AssertPose(PoseOf(_aim), aim);
        Assert.That(_aim.gameObject.activeSelf, Is.False);
        Call(_input, "UseTcpPreview");
        Assert.That(CallArgs(_targets, "TryGetCapturedRosPose", exported), Is.EqualTo(true));
        Assert.That(exported[1], Is.Not.Null);
        AssertPose(PoseOf(_tcpA), before);
    }

    [Test]
    public void LiveToolAimUsesTheSameTcpPoseInBothTaskModes()
    {
        Pose pointer = new Pose(new Vector3(0.7f, 0.6f, -0.2f), Quaternion.Euler(24f, -37f, 19f));
        Call(_surfaces, "SetPointerPose", pointer, _tracking);
        GameObject liveContact = (GameObject)Field(_surfaces, "_livePointInstance").GetValue(_surfaces);
        Pose contactAim = PoseOf(liveContact.transform);
        AssertPose(contactAim, RobotFrameMath.PointerToTcpPose(pointer));
        Call(_input, "UseTcpPreview");
        Assert.That(liveContact.activeSelf, Is.False);
        Call(_targets, "SetPointerPose", pointer, _tracking);
        AssertPose(PoseOf(_aim), contactAim);
    }

    [Test]
    public void PlanAndExecuteUseOnlyTheSelectedPlanner()
    {
        Set(_surfaces, "_pickPointInstance", null);
        string initialTcp = Get<string>(_tcp, "StatusText");
        Call(_input, "PlanCurrentTask");
        Assert.That(Get<string>(_mtc, "StatusText"), Does.Contain("Capture both"));
        Assert.That(Get<string>(_tcp, "StatusText"), Is.EqualTo(initialTcp));
        Call(_input, "ExecuteCurrentTask");
        Assert.That(Get<string>(_mtc, "StatusText"), Does.Contain("MTC Execute"));
        Assert.That(Get<string>(_tcp, "StatusText"), Is.EqualTo(initialTcp));
        Call(_input, "UseTcpPreview");
        string inactiveMtc = Get<string>(_mtc, "StatusText");
        Call(_input, "PlanCurrentTask");
        Assert.That(Get<string>(_tcp, "StatusText"), Does.Contain("disconnected"));
        Call(_input, "ExecuteCurrentTask");
        Assert.That(Get<string>(_tcp, "StatusText"), Does.Contain("disconnected"));
        Assert.That(Get<string>(_mtc, "StatusText"), Is.EqualTo(inactiveMtc));
    }

    [Test]
    public void FailedOfflineMtcPublishCannotLeaveAnArmedRequest()
    {
        Call(_input, "PlanCurrentTask");
        Assert.That(Get<string>(_mtc, "StatusText"), Does.Contain("could not be sent"));
        Assert.That(Get<bool>(_mtc, "IsPlanning") || Get<bool>(_mtc, "CanExecute"), Is.False);
        Assert.That((bool)Field(_surfaces, "_hasPlanningRequestBeenSent").GetValue(_surfaces), Is.False);
        MtcStatus(true);
        Assert.That(Get<bool>(_mtc, "HasPlanResult"), Is.False);
    }

    [Test]
    public void LateMtcResultsCannotRestoreAPlanAfterSwitchingAwayAndBack()
    {
        Assert.That(Call(_mtc, "BeginPlanningRequest"), Is.EqualTo(true));
        Call(_input, "UseTcpPreview");
        MtcStatus(true);
        Private(_mtc, "OnPlannedTrajectory", Trajectory());
        Assert.That(Get<bool>(_mtc, "HasPlanResult"), Is.False);
        Call(_input, "UseTilePickPlace");
        MtcStatus(true);
        Private(_mtc, "OnPlannedTrajectory", Trajectory());
        Assert.That(Get<bool>(_mtc, "HasPlanResult") || Get<bool>(_mtc, "CanExecute"), Is.False);
        Assert.That(Field(_visualizer, "_trajectory").GetValue(_visualizer), Is.Null);
    }

    [Test]
    public void MtcSuccessWaitsForPreviewAndRejectsAnUndisplayableTrajectory()
    {
        Assert.That(Call(_mtc, "BeginPlanningRequest"), Is.EqualTo(true));
        MtcStatus(true);
        Assert.That(Get<bool>(_mtc, "HasPlanResult"), Is.True);
        Assert.That(Get<bool>(_mtc, "CanExecute"), Is.False);
        Assert.That(Get<string>(_mtc, "StatusText"), Does.Contain("Waiting"));
        Private(_mtc, "OnPlannedTrajectory", Trajectory());
        Assert.That(Get<bool>(_mtc, "CanExecute") || Get<bool>(_mtc, "IsPlanning"), Is.False);
        Assert.That(Get<string>(_mtc, "StatusText"), Does.Contain("could not be displayed"));
    }

    [Test]
    public void ReadyMtcReviewRequiresTheSameDisplayedTrajectoryAndConsumesOnce()
    {
        InstallActualGhostModel();
        Assert.That(Call(_mtc, "BeginPlanningRequest"), Is.EqualTo(true));
        MtcStatus(true);
        object trajectory = Trajectory();
        Private(_mtc, "OnPlannedTrajectory", trajectory);
        Assert.That(Get<bool>(_mtc, "CanExecute"), Is.True);
        Set(_visualizer, "_trajectory", Trajectory());
        Assert.That(Get<bool>(_mtc, "CanExecute"), Is.False);
        Set(_visualizer, "_trajectory", trajectory);
        Assert.That(Call(_mtc, "ConsumePlanForExecution"), Is.EqualTo(true));
        Assert.That(Call(_mtc, "ConsumePlanForExecution"), Is.EqualTo(false));
        Assert.That(Get<bool>(_mtc, "CanExecute"), Is.False);
    }

    [TestCase(true)]
    [TestCase(false)]
    public void InitialEmptyFalseAndReplacementNoticeAllowSuccessAndPreviewInEitherOrder(bool previewFirst)
    {
        InstallActualGhostModel();
        Assert.That(Call(_mtc, "BeginPlanningRequest"), Is.EqualTo(true));
        MtcStatus(false);
        Assert.That(Get<bool>(_mtc, "IsPlanning"), Is.True);
        Assert.That(Get<bool>(_mtc, "HasPlanResult"), Is.False);
        MtcExecutionStatus("invalidated", "New planning request replaced the previous plan");
        Assert.That(Get<bool>(_mtc, "IsPlanning"), Is.True);
        object trajectory = Trajectory();
        if (previewFirst) Private(_mtc, "OnPlannedTrajectory", trajectory);
        else MtcStatus(true);
        Assert.That(Get<bool>(_mtc, "CanExecute"), Is.False);
        if (previewFirst) MtcStatus(true);
        else Private(_mtc, "OnPlannedTrajectory", trajectory);
        Assert.That(Get<bool>(_mtc, "CanExecute"), Is.True);
        Assert.That(Get<bool>(_visualizer, "HasVisiblePreview"), Is.True);
        Assert.That(Get<string>(_mtc, "StatusText"), Does.Contain("plan ready"));
    }

    [Test]
    public void EmptyFalseFollowedByPlanningFailedTerminatesAndRejectsLateSuccess()
    {
        Assert.That(Call(_mtc, "BeginPlanningRequest"), Is.EqualTo(true));
        MtcStatus(false);
        Assert.That(Get<bool>(_mtc, "IsPlanning"), Is.True);
        MtcExecutionStatus("planning_failed", "No solution");
        Assert.That(Get<bool>(_mtc, "IsPlanning") || Get<bool>(_mtc, "CanExecute"), Is.False);
        Assert.That(Get<string>(_mtc, "StatusText"), Does.Contain("planning_failed"));
        MtcStatus(true);
        Assert.That(Get<bool>(_mtc, "HasPlanResult"), Is.False);
    }

    [Test]
    public void ReplacementNoticeRevokesAnAlreadyReviewedMtcPlan()
    {
        InstallActualGhostModel();
        Assert.That(Call(_mtc, "BeginPlanningRequest"), Is.EqualTo(true));
        MtcStatus(true);
        Private(_mtc, "OnPlannedTrajectory", Trajectory());
        Assert.That(Get<bool>(_mtc, "CanExecute"), Is.True);
        // EditMode requires immediate teardown of native clones before runtime
        // invalidation; the frozen request/result flags remain unchanged.
        DestroyPreviewClonesImmediately();
        MtcExecutionStatus("invalidated", "New planning request replaced the previous plan");
        Assert.That(Get<bool>(_mtc, "HasPlanResult") || Get<bool>(_mtc, "CanExecute"), Is.False);
        Assert.That(Get<string>(_mtc, "StatusText"), Does.Contain("invalidated"));
    }

    [Test]
    public void MtcPreviewTimeoutDisarmsTheRequestAndRejectsLateCallbacks()
    {
        Assert.That(Call(_mtc, "BeginPlanningRequest"), Is.EqualTo(true));
        Set(_mtc, "_planningDeadline", Time.realtimeSinceStartup - 1f);
        Private(_mtc, "Update");
        Assert.That(Get<bool>(_mtc, "IsPlanning") || Get<bool>(_mtc, "CanExecute"), Is.False);
        Assert.That(Get<string>(_mtc, "StatusText"), Does.Contain("No complete MTC preview"));
        MtcStatus(true);
        Assert.That(Get<bool>(_mtc, "HasPlanResult"), Is.False);
    }

    [Test]
    public void HistoricalResetListenerPreservesTheMtcBlockedReason()
    {
        GameObject panel = new GameObject("Legacy MTC feedback panel", typeof(RectTransform));
        _owned.Add(panel);
        panel.SetActive(false);
        Component text = panel.AddComponent(TypeOf("TMPro.TextMeshProUGUI"));
        Component ui = panel.AddComponent(TypeOf("MTCPlanUIManager"));
        Set(ui, "_input", _input);
        Set(ui, "_tcpPlanning", _tcp);
        Set(ui, "_MTCPlansManager", _mtc);
        Private(ui, "Start");
        Set(_surfaces, "_pickPointInstance", null);
        Call(_input, "PlanCurrentTask");
        string blocked = Get<string>(_mtc, "StatusText");
        Assert.That(blocked, Does.Contain("Capture both"));
        Call(ui, "ResetPlanStatusPanel");
        Assert.That(Get<string>(text, "text"), Is.EqualTo(blocked));
    }

    [Test]
    public void SurfaceRotationRevokesThePreviousMtcRequestAndPreview()
    {
        Assert.That(Call(_mtc, "BeginPlanningRequest"), Is.EqualTo(true));
        MtcStatus(true);
        Set(_surfaces, "_hasPlanningRequestBeenSent", true);
        Private(_surfaces, "OnTargetRotationChanged");
        Assert.That(Get<bool>(_mtc, "HasPlanResult") || Get<bool>(_mtc, "CanExecute"), Is.False);
        Assert.That((bool)Field(_surfaces, "_hasPlanningRequestBeenSent").GetValue(_surfaces), Is.False);
        MtcStatus(true);
        Assert.That(Get<bool>(_mtc, "HasPlanResult"), Is.False);
    }

    [Test]
    public void AlignmentBlocksBothModesBeforeAnyPlanningOrExecutionConnection()
    {
        Call(_coordinates, "BeginCalibrationEdit");
        Call(_input, "PlanCurrentTask");
        Assert.That(Get<string>(_mtc, "StatusText"), Does.Contain("alignment"));
        Call(_input, "ExecuteCurrentTask");
        Assert.That(Get<bool>(_mtc, "CanExecute") || Get<bool>(_mtc, "IsPlanning"), Is.False);
        Assert.That(Call(_mtc, "BeginPlanningRequest"), Is.EqualTo(false));
        Call(_input, "UseTcpPreview");
        Call(_input, "PlanCurrentTask");
        Assert.That(Get<string>(_tcp, "StatusText"), Does.Contain("alignment"));
        Call(_input, "ExecuteCurrentTask");
        Assert.That(Get<string>(_tcp, "StatusText"), Does.Contain("alignment"));
        Assert.That(Get<bool>(_tcp, "CanExecute"), Is.False);
    }

    private Transform Marker(string field, string name, Vector3 position)
    {
        Type kind = _targets.GetType().GetNestedType("Marker", BindingFlags.NonPublic);
        object marker = Activator.CreateInstance(kind);
        Transform root = New(name).transform;
        root.SetPositionAndRotation(position, Quaternion.Euler(17f, -29f, 38f));
        kind.GetField("Root").SetValue(marker, root);
        Set(_targets, field, marker);
        return root;
    }
    private void MtcStatus(bool success)
    {
        MethodInfo callback = _mtc.GetType().GetMethod("OnPlanningStatus", BindingFlags.Instance | BindingFlags.NonPublic);
        object msg = Activator.CreateInstance(callback.GetParameters()[0].ParameterType,
            new object[] { "{\"overall_success\":" + (success ? "true" : "false") + ",\"stages\":[]}" });
        callback.Invoke(_mtc, new[] { msg });
    }
    private void MtcExecutionStatus(string status, string reason)
    {
        MethodInfo callback = _mtc.GetType().GetMethod("OnExecutionStatus", BindingFlags.Instance | BindingFlags.NonPublic);
        object msg = Activator.CreateInstance(callback.GetParameters()[0].ParameterType,
            new object[] { "{\"status\":\"" + status + "\",\"reason\":\"" + reason + "\"}" });
        callback.Invoke(_mtc, new[] { msg });
    }
    private void InstallActualGhostModel()
    {
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos_transparent.prefab");
        Assert.That(prefab, Is.Not.Null);
        GameObject robot = UnityEngine.Object.Instantiate(prefab);
        _owned.Add(robot);
        robot.SetActive(false);
        Component rig = robot.AddComponent(TypeOf("RosRobotRig"));
        Component tools = _owned.SelectMany(go => go.GetComponents<Component>()).First(c => c != null && c.GetType().Name == "ToolSelectionController");
        Assert.That(Call(rig, "ApplyModel", Get<RosRobotModel>(tools, "CurrentModel")), Is.EqualTo(true));
        Set(_visualizer, "virtualUr16ePrefab", prefab);
        Private(_visualizer, "RecordRobot", robot, Pose.identity);
    }
    private void DestroyPreviewClonesImmediately()
    {
        if (_visualizer == null) return;
        IList clones = (IList)Field(_visualizer, "_clones").GetValue(_visualizer);
        foreach (Component clone in clones)
            if (clone != null) UnityEngine.Object.DestroyImmediate(clone.gameObject);
        clones.Clear();
    }
    private object Trajectory()
    {
        Type type = _mtc.GetType().GetMethod("OnPlannedTrajectory", BindingFlags.Instance | BindingFlags.NonPublic).GetParameters()[0].ParameterType;
        object value = Activator.CreateInstance(type);
        object joints = type.GetField("joint_trajectory").GetValue(value);
        joints.GetType().GetField("joint_names").SetValue(joints, new[] { "shoulder_pan_joint" });
        FieldInfo pointsField = joints.GetType().GetField("points");
        Type pointType = pointsField.FieldType.GetElementType();
        Array points = Array.CreateInstance(pointType, 1);
        object point = Activator.CreateInstance(pointType);
        pointType.GetField("positions").SetValue(point, new[] { 0d });
        points.SetValue(point, 0);
        pointsField.SetValue(joints, points);
        return value;
    }
    private GameObject New(string name) { var value = new GameObject(name); _owned.Add(value); return value; }
    private Component Add(string name, string type) => Disabled(New(name), type);
    private static Component Disabled(GameObject owner, string type)
    { Component result = owner.AddComponent(TypeOf(type)); ((Behaviour)result).enabled = false; return result; }
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType(name)).FirstOrDefault(t => t != null)
        ?? throw new InvalidOperationException("Missing runtime type " + name);
    private static FieldInfo Field(object value, string name) => value.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public)
        ?? throw new InvalidOperationException("Missing field " + value.GetType().Name + "." + name);
    private static void Set(object value, string name, object data) => Field(value, name).SetValue(value, data);
    private static T Get<T>(object value, string name) => (T)value.GetType().GetProperty(name).GetValue(value);
    private static object Call(object value, string name, params object[] arguments) => CallArgs(value, name, arguments);
    private static object CallArgs(object value, string name, object[] arguments) => value.GetType().GetMethods()
        .Single(method => method.Name == name && method.GetParameters().Length == arguments.Length)
        .Invoke(value, arguments);
    private static object Private(object value, string name, params object[] arguments) => value.GetType().GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic).Invoke(value, arguments);
    private static Pose PoseOf(Transform value) => new Pose(value.position, value.rotation);
    private static void AssertPose(Pose actual, Pose expected)
    {
        Assert.That(Vector3.Distance(actual.position, expected.position), Is.LessThan(0.000001f));
        Assert.That(Quaternion.Angle(actual.rotation, expected.rotation), Is.LessThan(0.001f));
    }
}
