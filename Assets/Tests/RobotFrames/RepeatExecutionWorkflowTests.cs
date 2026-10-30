using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>Actual public MTC/TCP actions and status callbacks; only ROS publication is substituted.</summary>
public sealed class RepeatExecutionWorkflowTests
{
    private const string PlanId = "offline-retained-reviewed-plan";
    private const string TcpRequest = "0:0";
    private MtcExecutionWorkflowTests _fixture;
    private Component _plans, _visualizer, _input, _coordinates, _feedback, _planner, _publisher;
    private string[] _names;
    private double[] _start;
    private bool _tcp;
    private bool _advertisePlanToken;

    [SetUp]
    public void CreateActualComponentsWithoutStartingRos()
    {
        _fixture = new MtcExecutionWorkflowTests();
        _fixture.CreateActualReviewAndItsMenuStatusWithoutTransport();
        _plans = Value<Component>(_fixture, "_plans");
        _visualizer = Value<Component>(_fixture, "_visualizer");
        _input = Value<Component>(_fixture, "_input");
        _coordinates = Value<Component>(_fixture, "_coordinates");
        _feedback = Add("Repeat-test named feedback", "JointStateSubscriber");
        Component tools = UnityEngine.Object.FindFirstObjectByType(TypeOf("ToolSelectionController")) as Component;
        RosRobotModel model = Get<RosRobotModel>(tools, "CurrentModel");
        _names = model.links.Where(RosRobotModelMath.IsRevolute).Select(link => link.jointName).ToArray();
        _start = new[] { 0.1d, -1.2d, 1.6d, -0.8d, -0.9d, 0.3d };
        Assert.That(_names.Length, Is.EqualTo(6));
    }

    [TearDown]
    public void CleanupWithoutTransport() => _fixture?.CleanupAndProveNoTransportWasCreated();

    [TestCase(false)]
    [TestCase(true)]
    public void PublicRepeatUsesTheRetainedPlanAndAFreshNonceAndConsumesOnlyOneClick(bool tcp)
    {
        ExecuteAndRetain(tcp);
        Execute();
        Assert.That(PublishCount, Is.EqualTo(2));
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.True);
        Assert.That(Get<bool>(_planner, "CanRepeat"), Is.False);
        RepeatPayload request = LastRepeatRequest();
        Assert.That(request.plan_id, Is.EqualTo(PlanId));
        Assert.That(Guid.TryParseExact(request.attempt_id, "N", out _), Is.True);
        if (tcp) Assert.That(Value<string>(_publisher, "LastTopic"), Is.EqualTo("/repeat_reviewed_plan"));
        else Assert.That(Value<int>(_publisher, "RepeatPublishCount"), Is.EqualTo(1), "Repeat must not resend execute_task.");
        Execute();
        Execute();
        Assert.That(PublishCount, Is.EqualTo(2), "Rapid repeated clicks must not publish duplicate motion requests.");
        Status("succeeded", true, "", PlanId);
        Status("succeeded", true, request.attempt_id, "old-plan");
        Status("succeeded", true, "old-attempt", PlanId);
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.True, "Stale completion must not release the current attempt.");
        Status("succeeded", true, request.attempt_id, PlanId);
        Feed();
        Assert.That(Get<bool>(_planner, "CanRepeat"), Is.True);
        Execute();
        RepeatPayload next = LastRepeatRequest();
        Assert.That(next.attempt_id, Is.Not.EqualTo(request.attempt_id));
        Status("succeeded", true, request.attempt_id, PlanId);
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.True);
        Assert.That(PublishCount, Is.EqualTo(3));
    }

    [TestCase(false, "away")]
    [TestCase(true, "away")]
    [TestCase(false, "full turn")]
    [TestCase(true, "full turn")]
    [TestCase(false, "stale")]
    [TestCase(true, "stale")]
    public void RepeatWaitsForFreshFeedbackAtTheExactReviewedJointWinding(bool tcp, string changed)
    {
        ExecuteAndRetain(tcp);
        if (changed == "stale")
        {
            Dictionary<string, float> times = Value<Dictionary<string, float>>(_feedback, "_receivedAt");
            foreach (string name in _names) times[name] = Time.realtimeSinceStartup - 60f;
        }
        else
        {
            double[] moved = (double[])_start.Clone();
            moved[2] += changed == "full turn" ? 2d * Math.PI : 0.03d;
            Feed(moved);
        }
        Assert.That(Get<bool>(_planner, "CanRepeat"), Is.False);
        Assert.That(Get<bool>(_planner, "HasRepeatablePlan"), Is.True, "Waiting at another pose must retain the reviewed plan.");
        Assert.That(PublishCount, Is.EqualTo(1), "Returning to the start never automatically executes the plan.");
        Feed();
        Assert.That(Get<bool>(_planner, "CanRepeat"), Is.True);
        Assert.That(PublishCount, Is.EqualTo(1));
    }

    [TestCase(false)]
    [TestCase(true)]
    public void InvalidatingReviewWhileAnExecutionIsPendingPreventsLaterSuccessFromRearmingIt(bool tcp)
    {
        Ready(tcp);
        Execute();
        if (tcp) Call(_planner, "InvalidatePlan", "Synthetic target edit");
        else Call(_planner, "InvalidatePlan");
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.True, "A review edit cannot cancel a command already sent.");
        Status("succeeded", true, "", PlanId);
        Feed();
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.False);
        Assert.That(Get<bool>(_planner, "HasRepeatablePlan"), Is.False);
        Assert.That(Get<bool>(_planner, "CanRepeat"), Is.False);
        Assert.That(PublishCount, Is.EqualTo(1));
    }

    [TestCase(false)]
    [TestCase(true)]
    public void LegacySuccessfulStatusWithoutServerRepeatSupportLeavesThePlanSpent(bool tcp)
    {
        Ready(tcp, false);
        Execute();
        Status("succeeded", false, "", null);
        Feed();
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.False);
        Assert.That(Get<bool>(_planner, "HasRepeatablePlan"), Is.False);
        Assert.That(Get<bool>(_planner, "CanExecute"), Is.False);
        Assert.That(PublishCount, Is.EqualTo(1));
    }

    [TestCase(false)]
    [TestCase(true)]
    public void InitialCompletionForAnOlderRetainedPlanCannotGrantRepeatPermission(bool tcp)
    {
        Ready(tcp);
        Execute();
        Status("succeeded", true, "", "older-plan-token");
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.True);
        Assert.That(Get<bool>(_planner, "HasRepeatablePlan"), Is.False);
        Status("succeeded", true, "", PlanId);
        Feed();
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.False);
        Assert.That(Get<bool>(_planner, "CanRepeat"), Is.True);
        Assert.That(PublishCount, Is.EqualTo(1));
    }

    [TestCase(false)]
    [TestCase(true)]
    public void RepeatPublicationFailureReleasesBusyStateWithoutRestoringSpentPermission(bool tcp)
    {
        ExecuteAndRetain(tcp);
        Set(_publisher, "PublishSucceeds", false);
        Execute();
        Assert.That(PublishCount, Is.EqualTo(2));
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.False);
        Assert.That(Get<bool>(_planner, "HasRepeatablePlan"), Is.False);
        Assert.That(Get<bool>(_planner, "CanRepeat"), Is.False);
        Assert.That(Get<string>(_planner, "StatusText"), Does.Contain("request failed"));
        Status("succeeded", true, LastRepeatRequest().attempt_id, PlanId);
        Assert.That(Get<bool>(_planner, "CanRepeat"), Is.False, "Unsolicited completion after a failed dispatch cannot rearm review.");
        Assert.That(PublishCount, Is.EqualTo(2));
    }

    [Test]
    public void AnExpiredTcpRepeatCanBeRejectedWithoutRequestIdOnlyForTheExactPlanAndAttempt()
    {
        ExecuteAndRetain(true);
        Execute();
        RepeatPayload request = LastRepeatRequest();
        Message(_planner, "OnExecutionStatus", JsonUtility.ToJson(new ExecutionStatus {
            status = "rejected", reason = "Plan is no longer retained", plan_id = PlanId, attempt_id = "older-attempt"
        }));
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.True);
        Message(_planner, "OnExecutionStatus", JsonUtility.ToJson(new ExecutionStatus {
            status = "rejected", reason = "Plan is no longer retained", plan_id = "older-plan", attempt_id = request.attempt_id
        }));
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.True);
        Message(_planner, "OnExecutionStatus", JsonUtility.ToJson(new ExecutionStatus {
            status = "rejected", reason = "Plan is no longer retained", plan_id = PlanId, attempt_id = request.attempt_id
        }));
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.False);
        Assert.That(Get<bool>(_planner, "HasRepeatablePlan"), Is.False);
        Assert.That(Get<bool>(_planner, "CanRepeat"), Is.False);
        Assert.That(Get<string>(_planner, "StatusText"), Does.Contain("no longer retained"));
        Assert.That((bool)TypeOf("RobotExecutionState").GetProperty("IsBusy").GetValue(null), Is.False);
        Assert.That(PublishCount, Is.EqualTo(2));
    }

    private void Ready(bool tcp, bool advertisePlanToken = true)
    {
        _tcp = tcp;
        _advertisePlanToken = advertisePlanToken;
        if (tcp) ReadyTcp(); else ReadyMtc();
        Feed();
        Assert.That(Get<bool>(_planner, "CanExecute"), Is.True, Get<string>(_planner, "StatusText"));
    }
    private void ReadyMtc()
    {
        Private(_fixture, "InstallOfflineContactController");
        _publisher = Value<Component>(_fixture, "_contacts");
        _planner = _plans;
        Set(_plans, "_jointFeedback", _feedback);
        Assert.That(Call(_plans, "BeginPlanningRequest"), Is.EqualTo(true));
        Message(_plans, "OnPlanningStatus", "{\"overall_success\":true,\"stages\":[]}");
        Private(_plans, "OnPlannedTrajectory", FullTrajectory());
        Status("ready", false, "", _advertisePlanToken ? PlanId : null);
    }
    private void ReadyTcp()
    {
        _publisher = _planner = Add("Repeat-test offline TCP publisher", "OfflineTcpExecutionController");
        Component targets = Add("Repeat-test TCP captures", "TcpTargetPreview");
        Set(_planner, "_targets", targets);
        Set(_planner, "_visualizer", _visualizer);
        Set(_planner, "_jointFeedback", _feedback);
        Set(_input, "_tcpPlanningController", _planner);
        Set(_input, "_tcpTargetPreview", targets);
        Call(_input, "UseTcpPreview");
        TypeOf("ToolPoseVisual").GetMethod("SetReferencePrefab", BindingFlags.Public | BindingFlags.Static)
            .Invoke(null, new[] { Field(_visualizer, "virtualUr16ePrefab").GetValue(_visualizer) });
        Material material = Resources.Load<Material>("RobotTargetMarkers/Opaque");
        Set(targets, "_markerMaterial", material);
        Transform tracking = Value<Transform>(_coordinates, "_calibrationFrame");
        Call(targets, "CapturePointerPose", new Pose(new Vector3(0.4f, 0.5f, 0.3f), Quaternion.Euler(13f, 22f, -7f)), tracking, true);
        Call(targets, "CapturePointerPose", new Pose(new Vector3(-0.2f, 0.4f, 0.7f), Quaternion.Euler(-11f, 16f, 24f)), tracking, false);
        object[] a = { true, null }, b = { false, null };
        Assert.That(Call(targets, "TryGetCapturedRosPose", a), Is.EqualTo(true));
        Assert.That(Call(targets, "TryGetCapturedRosPose", b), Is.EqualTo(true));
        Set(_planner, "_requestedA", a[1]);
        Set(_planner, "_requestedB", b[1]);
        TcpPlanReview review = Value<TcpPlanReview>(_planner, "_review");
        review.Begin(TcpRequest);
        Message(_planner, "OnStatus", "{\"request_id\":\"0:0\",\"overall_success\":true,\"phase\":\"ready\"" +
            (_advertisePlanToken ? ",\"plan_id\":\"" + PlanId + "\"" : "") + "}");
        Private(_planner, "OnTrajectory", FullTrajectory());
    }
    private object FullTrajectory()
    {
        object trajectory = Private(_fixture, "NewTrajectory");
        object joints = Field(trajectory, "joint_trajectory").GetValue(trajectory);
        Set(joints, "joint_names", (string[])_names.Clone());
        Array points = Value<Array>(joints, "points");
        Set(points.GetValue(0), "positions", (double[])_start.Clone());
        return trajectory;
    }
    private void ExecuteAndRetain(bool tcp)
    {
        Ready(tcp);
        Execute();
        Assert.That(PublishCount, Is.EqualTo(1));
        Assert.That(Get<bool>(_planner, "IsExecuting"), Is.True);
        if (tcp) Assert.That(Value<string>(_publisher, "LastTopic"), Is.EqualTo("/execute_tcp_plan"));
        else Assert.That(Value<int>(_publisher, "RepeatPublishCount"), Is.Zero);
        Status("succeeded", true, "", PlanId);
        Feed();
        Assert.That(Get<bool>(_planner, "CanRepeat"), Is.True, Get<string>(_planner, "StatusText"));
    }
    private void Execute() => Call(_publisher, _tcp ? "Execute" : "ExecutePickAndPlace");
    private void Feed(double[] positions = null) => Assert.That(Call(_feedback, "ObserveJointState", _names, positions ?? _start), Is.EqualTo(true));
    private int PublishCount => Value<int>(_publisher, "PublishCount");
    private RepeatPayload LastRepeatRequest() => JsonUtility.FromJson<RepeatPayload>(Value<string>(_publisher, _tcp ? "LastPayload" : "LastRepeatPayload"));
    private void Status(string status, bool repeatable, string attemptId, string planId) => Message(_planner, "OnExecutionStatus",
        JsonUtility.ToJson(new ExecutionStatus { request_id = _tcp ? TcpRequest : null, status = status,
            reason = "Offline execution outcome", repeatable = repeatable, plan_id = planId, attempt_id = attemptId }));
    private static void Message(object component, string callback, string json)
    {
        MethodInfo method = PrivateMethod(component, callback);
        object message = Activator.CreateInstance(method.GetParameters()[0].ParameterType, new object[] { json });
        method.Invoke(component, new[] { message });
    }
    private Component Add(string name, string type) => (Component)Private(_fixture, "Add", name, type);
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(assembly => assembly.GetType(name)).FirstOrDefault(type => type != null)
        ?? throw new InvalidOperationException("Missing runtime type " + name);
    private static FieldInfo Field(object owner, string name)
    {
        for (Type type = owner.GetType(); type != null; type = type.BaseType)
        {
            FieldInfo field = type.GetField(name, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
            if (field != null) return field;
        }
        throw new InvalidOperationException("Missing field " + name);
    }
    private static T Value<T>(object owner, string name) => (T)Field(owner, name).GetValue(owner);
    private static void Set(object owner, string name, object value) => Field(owner, name).SetValue(owner, value);
    private static T Get<T>(object owner, string name) => (T)owner.GetType().GetProperty(name).GetValue(owner);
    private static object Call(object owner, string name, params object[] args) => owner.GetType().GetMethods().Single(method => method.Name == name && method.GetParameters().Length == args.Length).Invoke(owner, args);
    private static object Private(object owner, string name, params object[] args) => PrivateMethod(owner, name).Invoke(owner, args);
    private static MethodInfo PrivateMethod(object owner, string name)
    {
        for (Type type = owner.GetType(); type != null; type = type.BaseType)
        {
            MethodInfo method = type.GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
            if (method != null) return method;
        }
        throw new InvalidOperationException("Missing private callback " + name);
    }
    [Serializable] private sealed class RepeatPayload { public string plan_id; public string attempt_id; }
    [Serializable] private sealed class ExecutionStatus { public string request_id; public string status; public string reason; public string plan_id; public string attempt_id; public bool repeatable; }
}
