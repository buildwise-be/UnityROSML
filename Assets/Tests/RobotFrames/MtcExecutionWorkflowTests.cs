using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>Exercises the actual MTC review and feedback lifecycle without Start, ROS or robot motion.</summary>
public sealed class MtcExecutionWorkflowTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private Component _plans, _visualizer, _coordinates, _input, _contacts, _statusText, _menu;
    private GameObject _pick, _place;
    private object _trajectory;
    private readonly List<string> _identifiedExecutionIds = new List<string>();

    [SetUp]
    public void CreateActualReviewAndItsMenuStatusWithoutTransport()
    {
        _coordinates = Add("Execution-test coordinates", "RobotCoordinateService");
        Add("Execution-test robot state", "RobotStateService");
        ArticulationBody body = New("Execution-test base_link").AddComponent<ArticulationBody>();
        body.transform.SetPositionAndRotation(new Vector3(0.3f, 0.2f, -0.4f), Quaternion.Euler(0f, 21f, 0f));
        Call(_coordinates, "Initialize", body, New("Execution-test tracking").transform);

        Component tools = Add("Execution-test matching tool", "ToolSelectionController");
        if (Field(tools, "_catalog").GetValue(tools) == null) Private(tools, "Awake");
        string urdf = Resources.Load<TextAsset>("RobotModels/kenos_sl.urdf").text;
        foreach (string service in new[] { "/robot_state_publisher/get_parameters", "/move_group/get_parameters", "/mtc_test_task_node/get_parameters" })
            Assert.That(Call(tools, "InspectDescription", service, urdf), Is.EqualTo(true));

        _plans = Add("Execution-test MTC manager", "MTCPlansManager");
        if (Field(_plans, "OnSuccessfulPlan").GetValue(_plans) == null) Private(_plans, "Awake");
        _visualizer = Add("Execution-test reviewed preview", "UR16eTrajectoryVisualizer");
        Set(_plans, "_UR16eTrajectoryVisualizer", _visualizer);
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos_transparent.prefab");
        Assert.That(prefab, Is.Not.Null);
        GameObject robot = UnityEngine.Object.Instantiate(prefab);
        _owned.Add(robot);
        robot.SetActive(false);
        Component rig = robot.AddComponent(TypeOf("RosRobotRig"));
        Assert.That(Call(rig, "ApplyModel", Get<RosRobotModel>(tools, "CurrentModel")), Is.EqualTo(true));
        Set(_visualizer, "virtualUr16ePrefab", prefab);
        Private(_visualizer, "RecordRobot", robot, Pose.identity);

        _input = Add("Execution-test task input", "URInputManager");
        Component tcp = Add("Execution-test TCP mode", "TcpPlanningController");
        Set(_input, "_tcpPlanningController", tcp);
        Call(_input, "UseTilePickPlace");

        GameObject panel = new GameObject("Execution-test status", typeof(RectTransform));
        _owned.Add(panel);
        panel.SetActive(false);
        _statusText = panel.AddComponent(TypeOf("TMPro.TextMeshProUGUI"));
        Component feedback = panel.AddComponent(TypeOf("MTCPlanUIManager"));
        Set(feedback, "_input", _input);
        Set(feedback, "_tcpPlanning", tcp);
        Set(feedback, "_MTCPlansManager", _plans);
        Private(feedback, "Start");

        // Subscribe the same shared menu used in the headset, in addition to the
        // historical scene status panel. No native XR selection or Start is run.
        _menu = New("Execution-test main menu").AddComponent(TypeOf("ToolSelectionMenu"));
        Set(_menu, "_input", _input);
        Set(_menu, "_tcpPlanning", tcp);
        Set(_menu, "_trajectoryVisualizer", _visualizer);
        Private(_menu, "BuildMenu");
        Private(_menu, "BindTaskControls");
    }

    [TearDown]
    public void CleanupAndProveNoTransportWasCreated()
    {
        if (_visualizer != null) Call(_visualizer, "SetTrajectory", new object[] { null });
        if (_menu != null)
        {
            Canvas canvas = (Canvas)Field(_menu, "_canvas").GetValue(_menu);
            if (canvas != null) UnityEngine.Object.DestroyImmediate(canvas.gameObject);
        }
        // Controller-owned targets are disposed by OnDestroy.
        if (_contacts != null)
        {
            Set(_contacts, "_pickPointInstance", null);
            Set(_contacts, "_placePointInstance", null);
        }
        for (int index = _owned.Count - 1; index >= 0; index--)
            if (_owned[index] != null) UnityEngine.Object.DestroyImmediate(_owned[index]);
        _owned.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty,
            "Execution regression checks must never open ROS transport or send real robot motion.");
    }

    [Test]
    public void TheConsumedReviewedPlanAcceptsExecutingAndSucceededAndUpdatesTheMenu()
    {
        ReadyPlan();
        Assert.That(Call(_plans, "ConsumePlanForExecution"), Is.EqualTo(true));
        AssertExecuting();
        Assert.That(Get<bool>(_plans, "CanExecute"), Is.False);
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("requested"));
        AssertMenuMatchesManager();
        ExecutionStatus("executing", "Executing the reviewed plan");
        AssertExecuting();
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("executing").IgnoreCase);
        AssertMenuMatchesManager();
        ExecutionStatus("succeeded", "Task completed");
        Assert.That(Get<bool>(_plans, "IsExecuting"), Is.False);
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("succeeded").IgnoreCase.And.Contain("Task completed"));
        AssertMenuMatchesManager();
        AssertSpentPlanCannotBeRearmed();
    }

    [Test]
    public void RecordedNordboExecutionRejectionIsVisibleAfterTheSuccessfulPlanWasConsumed()
    {
        string fixture = ReadFixture("mtc_execute_live_20261023.json");
        const string reason = "world collision geometry or octomap changed";
        Assert.That(fixture, Does.Contain("\"status\": \"rejected\"").And.Contain(reason));
        ReadyPlan();
        Assert.That(Call(_plans, "ConsumePlanForExecution"), Is.EqualTo(true));
        ExecutionStatus("rejected", reason);
        Assert.That(Get<bool>(_plans, "IsExecuting"), Is.False);
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("rejected").IgnoreCase.And.Contain(reason));
        AssertMenuMatchesManager();
        AssertSpentPlanCannotBeRearmed();
    }

    [TestCase("failed", "Controller refused the trajectory")]
    [TestCase("rejected", "Robot state differs from the reviewed start")]
    [TestCase("invalidated", "Robot tool changed")]
    public void ExecutionTerminalReasonsReachTheMenuAndNeverRestoreTheSpentReview(string status, string reason)
    {
        ReadyPlan();
        Assert.That(Call(_plans, "ConsumePlanForExecution"), Is.EqualTo(true));
        ExecutionStatus("executing", "Motion started");
        ExecutionStatus(status, reason);
        Assert.That(Get<bool>(_plans, "IsExecuting"), Is.False);
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain(status).IgnoreCase.And.Contain(reason));
        AssertMenuMatchesManager();
        AssertSpentPlanCannotBeRearmed();
    }

    [Test]
    public void AcknowledgementTimeoutReportsUnknownOutcomeWithoutRetryingOrRearming()
    {
        ReadyPlan();
        Assert.That(Call(_plans, "ConsumePlanForExecution"), Is.EqualTo(true));
        Set(_plans, "_executionDeadline", Time.realtimeSinceStartup - 1f);
        Private(_plans, "Update");
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("timed out").And.Contain("unknown"));
        AssertExecuting();
        Assert.That(Call(_plans, "ConsumePlanForExecution"), Is.EqualTo(false));
        Assert.That(Call(_plans, "BeginPlanningRequest"), Is.EqualTo(false));
        AssertMenuMatchesManager();
        ExecutionStatus("executing", "Delayed acknowledgement");
        AssertExecuting();
        ExecutionStatus("succeeded", "Completed after delayed acknowledgement");
        Assert.That(Get<bool>(_plans, "IsExecuting"), Is.False);
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("succeeded").IgnoreCase);
        AssertMenuMatchesManager();
    }

    [Test]
    public void DelayedPlanningAndReadyMessagesCannotOverwriteExecutionFeedback()
    {
        ReadyPlan();
        Assert.That(Call(_plans, "ConsumePlanForExecution"), Is.EqualTo(true));
        ExecutionStatus("executing", "Motion started");
        string executing = Get<string>(_plans, "StatusText");
        PlanningSuccess();
        Private(_plans, "OnPlannedTrajectory", _trajectory);
        ExecutionStatus("ready", "A successful plan is available for one execution attempt");
        ExecutionStatus("planning_failed", "Delayed previous planning notification");
        Assert.That(Get<string>(_plans, "StatusText"), Is.EqualTo(executing));
        AssertExecuting();
        Assert.That(Call(_plans, "BeginPlanningRequest"), Is.EqualTo(false));
        Assert.That(Call(_plans, "ConsumePlanForExecution"), Is.EqualTo(false));
        Assert.That(Get<string>(_plans, "StatusText"), Is.EqualTo(executing));
        AssertMenuMatchesManager();
    }

    [Test]
    public void AcknowledgedExecutionDoesNotUseThePlanningOrAcknowledgementDeadline()
    {
        ReadyPlan();
        Assert.That(Call(_plans, "ConsumePlanForExecution"), Is.EqualTo(true));
        ExecutionStatus("executing", "Motion started");
        Set(_plans, "_executionDeadline", Time.realtimeSinceStartup - 100f);
        Set(_plans, "_planningDeadline", Time.realtimeSinceStartup - 100f);
        string before = Get<string>(_plans, "StatusText");
        Private(_plans, "Update");
        AssertExecuting();
        Assert.That(Get<string>(_plans, "StatusText"), Is.EqualTo(before));
    }

    [Test]
    public void ActualExecuteActionPublishesOnceAndDisplaysTheSubsequentBackendRejection()
    {
        InstallOfflineContactController();
        ReadyPlan();
        Call(_input, "ExecuteCurrentTask");
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(1));
        AssertExecuting();
        Assert.That(Field(_contacts, "_hasPlanningRequestBeenSent").GetValue(_contacts), Is.EqualTo(false));
        string pending = Get<string>(_plans, "StatusText");
        Call(_input, "ExecuteCurrentTask");
        Call(_contacts, "SendPickAndPlacePlanningRequest");
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(1), "A second click must not resend or replace an unacknowledged command.");
        Assert.That(Get<string>(_plans, "StatusText"), Is.EqualTo(pending));
        ExecutionStatus("rejected", "world collision geometry or octomap changed");
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("rejected").And.Contain("world collision geometry or octomap changed"));
        AssertMenuMatchesManager();
        AssertSpentPlanCannotBeRearmed();
    }

    [Test]
    public void ActualExecuteTransportFailureConsumesTheReviewAndDisplaysTheReason()
    {
        InstallOfflineContactController();
        ReadyPlan();
        Set(_contacts, "PublishSucceeds", false);
        Call(_contacts, "ExecutePickAndPlace");
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(1));
        Assert.That(Get<bool>(_plans, "IsExecuting") || Get<bool>(_plans, "CanExecute"), Is.False);
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("request failed").And.Contain("Offline transport refused request"));
        AssertMenuMatchesManager();
        AssertSpentPlanCannotBeRearmed();
    }

    [Test]
    public void ActualExecuteRefusesTargetsMovedSinceTheReviewedPlanWithoutPublishing()
    {
        InstallOfflineContactController();
        ReadyPlan();
        _pick.transform.position += Vector3.right * 0.005f;
        Call(_contacts, "ExecutePickAndPlace");
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(0));
        Assert.That(Get<bool>(_plans, "IsExecuting") || Get<bool>(_plans, "CanExecute"), Is.False);
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("MTC Execute requires"));
        AssertMenuMatchesManager();
    }

    [TestCase("targets", false)]
    [TestCase("targets", true)]
    [TestCase("mode", false)]
    [TestCase("mode", true)]
    [TestCase("calibration", false)]
    [TestCase("calibration", true)]
    [TestCase("disable", false)]
    [TestCase("disable", true)]
    public void LocalReviewInvalidationPreservesTheSentCommandAndItsLateOutcome(string changed, bool timeoutFirst)
    {
        InstallOfflineContactController();
        ReadyPlan();
        Call(_input, "ExecuteCurrentTask");
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(1));
        if (timeoutFirst)
        {
            Set(_plans, "_executionDeadline", Time.realtimeSinceStartup - 1f);
            Private(_plans, "Update");
            Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("timed out").And.Contain("unknown"));
        }
        string unresolved = Get<string>(_plans, "StatusText");
        if (changed == "targets")
        {
            _pick.transform.position += Vector3.right * 0.005f;
            Private(_contacts, "OnTargetRotationChanged");
        }
        else if (changed == "mode") Call(_input, "UseTcpPreview");
        else if (changed == "calibration") { Call(_coordinates, "BeginCalibrationEdit"); Private(_plans, "Update"); }
        else Private(_plans, "OnDisable");
        AssertExecuting();
        Assert.That(Get<bool>(_visualizer, "HasVisiblePreview"), Is.False, "Local edits must revoke the old review without cancelling an already sent command.");
        Assert.That(Get<string>(_plans, "StatusText"), Is.EqualTo(unresolved));
        AssertMenuMatchesManager();

        Call(_input, "ExecuteCurrentTask");
        Call(_contacts, "ExecutePickAndPlace");
        Call(_input, "PlanCurrentTask");
        Call(_contacts, "SendPickAndPlacePlanningRequest");
        Assert.That(Call(_plans, "BeginPlanningRequest"), Is.EqualTo(false));
        Assert.That(Call(_plans, "ConsumePlanForExecution"), Is.EqualTo(false));
        Component tcp = (Component)Field(_input, "_tcpPlanningController").GetValue(_input);
        Call(tcp, "Plan");
        Call(tcp, "Execute");
        Assert.That(Get<string>(tcp, "StatusText"), Does.Contain("MTC execution request is unresolved"));
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(1), "Neither local invalidation nor a mode switch may resend or replace an unresolved command.");
        Assert.That(Get<string>(_plans, "StatusText"), Is.EqualTo(unresolved));
        PlanningSuccess();
        Private(_plans, "OnPlannedTrajectory", _trajectory);
        ExecutionStatus("ready", "Late ready notification");
        Assert.That(Get<string>(_plans, "StatusText"), Is.EqualTo(unresolved));
        AssertMenuMatchesManager();

        ExecutionStatus("executing", "Late motion notification");
        AssertExecuting();
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("Late motion notification"));
        AssertMenuMatchesManager();
        ExecutionStatus("succeeded", "Late completion");
        Assert.That(Get<bool>(_plans, "IsExecuting"), Is.False);
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("succeeded").And.Contain("Late completion"));
        AssertMenuMatchesManager();
        AssertSpentPlanCannotBeRearmed();

        // A deliberate new action may select another workflow after the outcome
        // is known. Its disconnected validation proves admission without ROS.
        if (changed == "calibration") Call(_coordinates, "EndCalibrationEdit");
        Call(_input, "UseTcpPreview");
        Call(_input, "PlanCurrentTask");
        Assert.That(Get<string>(tcp, "StatusText"), Does.Contain("ROS is disconnected"));
        Assert.That(Get<bool>(_plans, "HasExecutionFeedback"), Is.False);
        Assert.That(Get<bool>((Component)Field(_menu, "_planButton").GetValue(_menu), "interactable"), Is.True);
    }

    [Test]
    public void ARejectionAfterModeSwitchAndUnknownTimeoutStillExplainsTheSentCommand()
    {
        InstallOfflineContactController();
        ReadyPlan();
        Call(_input, "ExecuteCurrentTask");
        Call(_input, "UseTcpPreview");
        Set(_plans, "_executionDeadline", Time.realtimeSinceStartup - 1f);
        Private(_plans, "Update");
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("timed out").And.Contain("unknown"));
        AssertExecuting();
        ExecutionStatus("rejected", "world collision geometry or octomap changed");
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("rejected").And.Contain("world collision geometry or octomap changed"));
        Assert.That(Get<bool>(_plans, "IsExecuting"), Is.False);
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(1));
        AssertMenuMatchesManager();
        AssertSpentPlanCannotBeRearmed();
    }

    [Test]
    public void StopPlanningButtonCancelsThePendingCapabilityCheckAndAllowsAnotherPlan()
    {
        InstallOfflineContactController();
        Assert.That(_contacts.GetType().GetMethod("StopPlanning"), Is.Not.Null, "A pending pick plan needs a Stop Planning action.");
        Set(_contacts, "_checkingCapability", true);
        int generation = (int)Field(_contacts, "_generation").GetValue(_contacts);
        Private(_menu, "BindTaskControls"); Private(_menu, "RefreshTask");
        Component stop = (Component)Field(_menu, "_stopPlanningButton").GetValue(_menu);
        Component plan = (Component)Field(_menu, "_planButton").GetValue(_menu);
        Component execute = (Component)Field(_menu, "_executeButton").GetValue(_menu);
        Assert.That(stop.gameObject.activeSelf && Get<bool>(stop, "interactable"), Is.True);
        Assert.That(Get<bool>(plan, "interactable"), Is.False);
        Assert.That(execute.gameObject.activeSelf, Is.False);
        object clicked = Get<object>(stop, "onClick"); clicked.GetType().GetMethod("Invoke").Invoke(clicked, null);
        Assert.That(Get<bool>(_contacts, "IsPlanning"), Is.False);
        Assert.That(stop.gameObject.activeSelf, Is.False);
        Assert.That(Get<bool>(plan, "interactable"), Is.True);
        Assert.That(Get<bool>(execute, "interactable"), Is.False);
        string stopped = Get<string>(_plans, "StatusText");
        Assert.That(stopped, Does.Contain("stopped").IgnoreCase);
        Private(_contacts, "OnCapabilityResponse", null, generation);
        Assert.That(Get<string>(_plans, "StatusText"), Is.EqualTo(stopped), "A late capability response cannot send a stopped request.");
    }

    [Test]
    public void ACorrelatedPickPlanCannotReviewAnotherRequestsTrajectoryOrReadiness()
    {
        Assert.That(_plans.GetType().GetMethod("BeginCorrelatedPlanningRequest"), Is.Not.Null,
            "A retried pick plan needs request correlation before it can accept results.");
        Component transport = Add("Offline pick planning messages", "Unity.Robotics.ROSTCPConnector.ROSConnection");
        Assert.That(Call(_plans, "BeginCorrelatedPlanningRequest", transport, "102:4"), Is.EqualTo(true));
        Message("OnExecutionStatus", "{\"status\":\"ready\",\"plan_id\":\"old-plan\"}");
        Message("OnPlanningStatus", "{\"request_id\":\"101:4\",\"overall_success\":true,\"plan_id\":\"old-plan\"}");
        Private(_plans, "OnPlannedTrajectory", StampedTrajectory(101));
        Assert.That(Get<bool>(_plans, "HasPlanResult"), Is.False);
        Message("OnPlanningStatus", "{\"request_id\":\"102:4\",\"overall_success\":true,\"plan_id\":\"new-plan\"}");
        Assert.That(Get<bool>(_plans, "HasPlanResult"), Is.True);
        Assert.That(Get<bool>(_plans, "CanExecute"), Is.False);
        Private(_plans, "OnPlannedTrajectory", StampedTrajectory(102));
        Assert.That(Get<bool>(_plans, "CanExecute"), Is.False, "Wait for ROS to release the planner slot.");
        PlanningControl("pick/102:4", "finished");
        Assert.That(Get<bool>(_plans, "CanExecute"), Is.True);
    }

    [Test]
    public void StopDiscardsPickPreviewAndWaitsForTheMatchingTerminalBeforeRetry()
    {
        InstallOfflineContactController();
        Assert.That(_plans.GetType().GetMethod("BeginCorrelatedPlanningRequest"), Is.Not.Null,
            "Stopping a published plan requires a correlated ROS terminal result.");
        Component transport = Add("Offline pick cancellation messages", "Unity.Robotics.ROSTCPConnector.ROSConnection");
        Assert.That(Call(_plans, "BeginCorrelatedPlanningRequest", transport, "102:4"), Is.EqualTo(true));
        Message("OnPlanningStatus", "{\"request_id\":\"102:4\",\"overall_success\":true,\"plan_id\":\"cancelled-plan\"}");
        Private(_plans, "OnPlannedTrajectory", StampedTrajectory(102));
        Assert.That(Get<bool>(_visualizer, "HasPlayback"), Is.True);
        Call(_contacts, "StopPlanning");
        Assert.That(Get<bool>(_contacts, "IsStoppingPlanning"), Is.True);
        Assert.That(Get<bool>(_visualizer, "HasPlayback"), Is.False);
        Call(_plans, "InvalidatePlan");
        PlanningControl("pick/101:4", "cancelled");
        Assert.That(Call(_plans, "BeginPlanningRequest"), Is.EqualTo(false));
        PlanningControl("pick/102:4", "cancelled");
        Assert.That(Get<bool>(_contacts, "IsStoppingPlanning"), Is.False);
        Assert.That(Get<bool>(_contacts, "IsPlanning"), Is.False);
        Assert.That(Call(_plans, "BeginCorrelatedPlanningRequest", transport, "103:4"), Is.EqualTo(true));
        Message("OnPlanningStatus", "{\"request_id\":\"102:4\",\"overall_success\":true,\"plan_id\":\"cancelled-plan\"}");
        Private(_plans, "OnPlannedTrajectory", StampedTrajectory(102));
        Assert.That(Get<bool>(_plans, "HasPlanResult") || Get<bool>(_plans, "CanExecute"), Is.False);
    }

    [Test]
    public void AFinishedPickRequestCanDiscardItsStillPendingPreviewLocally()
    {
        InstallOfflineContactController();
        Component transport = Add("Offline pick preview messages", "Unity.Robotics.ROSTCPConnector.ROSConnection");
        Assert.That(Call(_plans, "BeginCorrelatedPlanningRequest", transport, "102:4"), Is.EqualTo(true));
        Message("OnPlanningStatus", "{\"request_id\":\"102:4\",\"overall_success\":true,\"plan_id\":\"ready-plan\"}");
        PlanningControl("pick/102:4", "finished");
        Assert.That(Get<bool>(_contacts, "IsPlanning") && Get<bool>(_contacts, "CanStopPlanning"), Is.True);
        Call(_contacts, "StopPlanning");
        Assert.That(Get<bool>(_contacts, "IsPlanning") || Get<bool>(_contacts, "IsStoppingPlanning"), Is.False);
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("preview discarded").IgnoreCase);
        Private(_plans, "OnPlannedTrajectory", StampedTrajectory(102));
        Assert.That(Get<bool>(_plans, "CanExecute") || Get<bool>(_visualizer, "HasPlayback"), Is.False);
    }

    [Test]
    public void CorrelatedPickSuccessWithoutAPlanIdCannotEnableExecute()
    {
        Component transport = Add("Offline unidentified pick result", "Unity.Robotics.ROSTCPConnector.ROSConnection");
        Assert.That(Call(_plans, "BeginCorrelatedPlanningRequest", transport, "102:4"), Is.EqualTo(true));
        Message("OnPlanningStatus", "{\"request_id\":\"102:4\",\"overall_success\":true}");
        Private(_plans, "OnPlannedTrajectory", StampedTrajectory(102));
        PlanningControl("pick/102:4", "finished");
        Assert.That(Get<bool>(_plans, "CanExecute"), Is.False, "A correlated review must identify the exact plan to execute.");
    }

    [Test]
    public void CorrelatedPickExecutePublishesOnlyTheReviewedPlanId()
    {
        InstallOfflineContactController();
        Component transport = Add("Offline identified pick execution", "Unity.Robotics.ROSTCPConnector.ROSConnection");
        Type type = transport.GetType();
        const string topicName = "/execute_pick_place_plan";
        MethodInfo register = type.GetMethods().Single(method => method.Name == "RegisterPublisher" && method.IsGenericMethodDefinition);
        object topic = register.MakeGenericMethod(TypeOf("RosMessageTypes.Std.StringMsg"))
            .Invoke(transport, new object[] { topicName, null, null });
        MethodInfo subscribe = topic.GetType().GetMethod("AddSubscriber");
        MethodInfo capture = GetType().GetMethod(nameof(CaptureIdentifiedExecution), BindingFlags.Instance | BindingFlags.NonPublic);
        subscribe.Invoke(topic, new object[] { Delegate.CreateDelegate(subscribe.GetParameters()[0].ParameterType, this, capture) });
        _identifiedExecutionIds.Clear();
        Assert.That(Call(_plans, "BeginCorrelatedPlanningRequest", transport, "102:4"), Is.EqualTo(true));
        Message("OnPlanningStatus", "{\"request_id\":\"102:4\",\"overall_success\":true,\"plan_id\":\"reviewed-plan\"}");
        Private(_plans, "OnPlannedTrajectory", StampedTrajectory(102));
        PlanningControl("pick/102:4", "finished");
        Assert.That(Get<bool>(_plans, "CanExecute"), Is.True);

        using var offlineConnectionFlag = new CancellationTokenSource();
        FieldInfo connectionError = type.GetField("m_HasConnectionError", BindingFlags.Static | BindingFlags.NonPublic);
        bool previousError = (bool)connectionError.GetValue(null);
        try
        {
            // Observe the real local publisher without connecting to ROS.
            connectionError.SetValue(null, false);
            Set(transport, "m_ConnectionThreadCancellation", offlineConnectionFlag);
            Set(_contacts, "_connection", transport);
            Call(_contacts, "ExecutePickAndPlace");
            Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(0), "A correlated plan cannot use the legacy Bool topic.");
            Assert.That(_identifiedExecutionIds, Is.EqualTo(new[] { "reviewed-plan" }));
            Message("OnExecutionStatus", "{\"status\":\"rejected\",\"plan_id\":\"foreign-plan\"}");
            Assert.That(Get<bool>(_plans, "IsExecuting"), Is.True);
            Message("OnExecutionStatus", "{\"status\":\"rejected\",\"plan_id\":\"reviewed-plan\"}");
            Assert.That(Get<bool>(_plans, "IsExecuting"), Is.False);
        }
        finally
        {
            Set(transport, "m_ConnectionThreadCancellation", null);
            connectionError.SetValue(null, previousError);
        }
    }

    private void CaptureIdentifiedExecution(object message) => _identifiedExecutionIds.Add((string)Field(message, "data").GetValue(message));

    private object StampedTrajectory(int seconds)
    {
        object trajectory = NewTrajectory();
        object joints = Field(trajectory, "joint_trajectory").GetValue(trajectory);
        object header = Field(joints, "header").GetValue(joints);
        object stamp = Field(header, "stamp").GetValue(header);
        Set(stamp, "sec", Convert.ChangeType(seconds, Field(stamp, "sec").FieldType));
        Set(stamp, "nanosec", 4u);
        return trajectory;
    }

    private void PlanningControl(string key, string status)
    {
        object helper = Field(_plans, "_planningStop").GetValue(_plans);
        object msg = Activator.CreateInstance(TypeOf("RosMessageTypes.Std.StringMsg"),
            new object[] { "{\"key\":\"" + key + "\",\"status\":\"" + status + "\",\"reason\":\"offline test\"}" });
        Private(helper, "OnStatus", msg);
    }

    [TestCase(PickItem.Tile, "tile")]
    [TestCase(PickItem.Brick, "brick")]
    public void RepeatPromptsForThePlacedItemAndOnlyTheCurrentYesCanPublish(PickItem item, string label)
    {
        RetainPickForConfirmation(item);
        Call(_menu, "ExecuteTask");
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(1), "Opening the repeat prompt cannot request motion.");
        Transform first = RepeatDialog();
        Assert.That(first, Is.Not.Null, "A retained pick repeat needs a placed-item confirmation.");
        Assert.That(ReadLabel(first, "Repeat confirmation title"), Is.EqualTo("Are you sure?"));
        Assert.That(ReadLabel(first, "Repeat confirmation message"), Is.EqualTo(
            "The previous run should have placed a " + label + " at the place location, re-executing will thus end up in a collision"));
        object staleYes = ButtonClick(first, "Yes, I am sure, I removed it");
        InvokeClick(ButtonClick(first, "No, cancel"));
        Assert.That(RepeatDialog(), Is.Null);
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(1));

        Call(_input, "ExecuteCurrentTask");
        Transform current = RepeatDialog();
        Assert.That(current, Is.Not.Null, "Serialized Execute actions use the same prompt.");
        Assert.That(ReferenceEquals(first, current), Is.False);
        InvokeClick(staleYes);
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(1));
        InvokeClick(ButtonClick(current, "Yes, I am sure, I removed it"));
        Assert.That(RepeatDialog(), Is.Null);
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(2));
        Assert.That(Field(_contacts, "RepeatPublishCount").GetValue(_contacts), Is.EqualTo(1));
        Assert.That((string)Field(_contacts, "LastRepeatPayload").GetValue(_contacts), Does.Contain("\"plan_id\":\"offline-confirmation-plan\""));
    }

    [TestCase("invalidate")]
    [TestCase("mode")]
    public void ChangingTheReviewContextClosesTheRepeatPromptAndRejectsItsOldYes(string change)
    {
        RetainPickForConfirmation(PickItem.Tile);
        Call(_input, "ExecuteCurrentTask");
        Transform dialog = RepeatDialog();
        Assert.That(dialog, Is.Not.Null, "The repeat must remain pending until the user confirms.");
        object yes = ButtonClick(dialog, "Yes, I am sure, I removed it");
        if (change == "invalidate") Call(_plans, "InvalidatePlan");
        else Call(_input, "UseTcpPreview");
        Private(_menu, "RefreshTask");
        Assert.That(RepeatDialog(), Is.Null);
        InvokeClick(yes);
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(1));
    }

    private void RetainPickForConfirmation(PickItem item)
    {
        InstallOfflineContactController();
        Assert.That(Call(_contacts, "SetSelectedItem", item), Is.EqualTo(true));
        object[] exported = { null, null };
        Assert.That(Call(_contacts, "TryGetTaskRequest", exported), Is.EqualTo(true));
        Set(_contacts, "_requestedTask", exported[0]);
        Set(_contacts, "_requestedContactPoses", Field(exported[0], "contacts").GetValue(exported[0]));
        Set(_contacts, "_hasPlanningRequestBeenSent", true);
        Private(_menu, "BindTaskControls");

        Component feedback = Add("Repeat confirmation joint feedback", "JointStateSubscriber");
        Set(_plans, "_jointFeedback", feedback);
        Component tools = (Component)UnityEngine.Object.FindFirstObjectByType(TypeOf("ToolSelectionController"));
        RosRobotModel model = Get<RosRobotModel>(tools, "CurrentModel");
        string[] names = model.links.Where(RosRobotModelMath.IsRevolute).Select(link => link.jointName).ToArray();
        double[] positions = { .1d, -1.2d, 1.6d, -.8d, -.9d, .3d };
        object trajectory = NewTrajectory();
        object joints = Field(trajectory, "joint_trajectory").GetValue(trajectory);
        Set(joints, "joint_names", names);
        Set(((Array)Field(joints, "points").GetValue(joints)).GetValue(0), "positions", positions);
        Assert.That(Call(_plans, "BeginPlanningRequest"), Is.EqualTo(true));
        Message("OnPlanningStatus", "{\"overall_success\":true,\"plan_id\":\"offline-confirmation-plan\"}");
        Private(_plans, "OnPlannedTrajectory", trajectory);
        Call(_menu, "ExecuteTask");
        Assert.That(RepeatDialog(), Is.Null, "The initial execution remains direct.");
        Assert.That(Field(_contacts, "PublishCount").GetValue(_contacts), Is.EqualTo(1));
        Message("OnExecutionStatus", "{\"status\":\"succeeded\",\"repeatable\":true,\"placed_item_confirmation_required\":true,\"plan_id\":\"offline-confirmation-plan\"}");
        Assert.That(Call(feedback, "ObserveJointState", names, positions), Is.EqualTo(true));
        Assert.That(Get<bool>(_plans, "CanRepeat"), Is.True);
    }

    private Transform RepeatDialog() => ((RectTransform)Field(_menu, "_panel").GetValue(_menu)).Find("Repeat execution confirmation");
    private static string ReadLabel(Transform parent, string name) => Get<string>(parent.Find(name).GetComponent(TypeOf("TMPro.TextMeshProUGUI")), "text");
    private static object ButtonClick(Transform parent, string name) => Get<object>(parent.Find(name).GetComponent(TypeOf("UnityEngine.UI.Button")), "onClick");
    private static void InvokeClick(object click) => click.GetType().GetMethod("Invoke").Invoke(click, null);

    private void ReadyPlan()
    {
        Assert.That(Call(_plans, "BeginPlanningRequest"), Is.EqualTo(true));
        PlanningSuccess();
        _trajectory = NewTrajectory();
        Private(_plans, "OnPlannedTrajectory", _trajectory);
        ExecutionStatus("ready", "A successful plan is available for one execution attempt");
        Assert.That(Get<bool>(_plans, "CanExecute"), Is.True);
        AssertMenuMatchesManager();
    }

    private void AssertExecuting()
    {
        Assert.That(Get<bool>(_plans, "IsExecuting"), Is.True);
        Assert.That(Get<bool>(_plans, "IsPlanning") || Get<bool>(_plans, "CanExecute") || Get<bool>(_plans, "HasPlanResult"), Is.False);
    }

    private void AssertSpentPlanCannotBeRearmed()
    {
        string terminal = Get<string>(_plans, "StatusText");
        Assert.That(Call(_plans, "ConsumePlanForExecution"), Is.EqualTo(false));
        PlanningSuccess();
        Private(_plans, "OnPlannedTrajectory", _trajectory);
        ExecutionStatus("ready", "Delayed ready status");
        ExecutionStatus("executing", "Delayed duplicate execution status");
        Assert.That(Get<bool>(_plans, "CanExecute") || Get<bool>(_plans, "IsExecuting") || Get<bool>(_plans, "HasPlanResult"), Is.False);
        Assert.That(Get<string>(_plans, "StatusText"), Is.EqualTo(terminal));
        AssertMenuMatchesManager();
    }

    private void AssertMenuMatchesManager()
    {
        string expected = Get<string>(_plans, "StatusText");
        Assert.That(Get<string>(_statusText, "text"), Is.EqualTo(expected));
        Component taskStatus = (Component)Field(_menu, "_taskStatus").GetValue(_menu);
        Assert.That(Get<string>(taskStatus, "text"), Is.EqualTo(expected), "The actual headset menu must show the backend lifecycle and rejection reason.");
        Component execute = (Component)Field(_menu, "_executeButton").GetValue(_menu);
        Assert.That(Get<bool>(execute, "interactable"), Is.EqualTo(Get<bool>(_plans, "CanExecute")));
        Component plan = (Component)Field(_menu, "_planButton").GetValue(_menu);
        Assert.That(Get<bool>(plan, "interactable"), Is.EqualTo(!Get<bool>(_plans, "IsExecuting")), "No workflow can replace an unresolved MTC execution command.");
    }

    private void InstallOfflineContactController()
    {
        // prepare_unity_validation.py emits this tiny runtime subclass outside
        // player Assets because the test assembly cannot reference Assembly-CSharp.
        // It overrides only transport; the production Execute method stays intact.
        _contacts = Add("Execution-test offline publisher", "OfflineMtcExecutionController");
        Set(_contacts, "_plansManager", _plans);
        Set(_input, "_pickPlaceController", _contacts);
        _pick = New("Execution-test pick");
        _place = New("Execution-test place");
        _pick.transform.SetPositionAndRotation(new Vector3(0.4f, 0.5f, 0.3f), Quaternion.Euler(13f, 22f, -7f));
        _place.transform.SetPositionAndRotation(new Vector3(-0.2f, 0.4f, 0.7f), Quaternion.Euler(-11f, 16f, 24f));
        Set(_contacts, "_pickPointInstance", _pick);
        Set(_contacts, "_placePointInstance", _place);
        object[] exported = { null };
        MethodInfo export = _contacts.GetType().GetMethod("TryGetContactPoses");
        Assert.That(export.Invoke(_contacts, exported), Is.EqualTo(true));
        Set(_contacts, "_requestedContactPoses", exported[0]);
        Set(_contacts, "_hasPlanningRequestBeenSent", true);
    }

    private void PlanningSuccess() => Message("OnPlanningStatus", "{\"overall_success\":true,\"stages\":[]}");
    private void ExecutionStatus(string status, string reason) => Message("OnExecutionStatus", JsonUtility.ToJson(new ExecutionMessage { status = status, reason = reason }));
    private void Message(string callback, string json)
    {
        MethodInfo method = _plans.GetType().GetMethod(callback, BindingFlags.Instance | BindingFlags.NonPublic);
        object message = Activator.CreateInstance(method.GetParameters()[0].ParameterType, new object[] { json });
        method.Invoke(_plans, new[] { message });
    }

    private object NewTrajectory()
    {
        Type type = _plans.GetType().GetMethod("OnPlannedTrajectory", BindingFlags.Instance | BindingFlags.NonPublic).GetParameters()[0].ParameterType;
        object value = Activator.CreateInstance(type);
        object joints = Field(value, "joint_trajectory").GetValue(value);
        Set(joints, "joint_names", new[] { "ur_shoulder_pan_joint" });
        FieldInfo pointsField = Field(joints, "points");
        Type pointType = pointsField.FieldType.GetElementType();
        Array points = Array.CreateInstance(pointType, 1);
        object point = Activator.CreateInstance(pointType);
        Set(point, "positions", new[] { 0d });
        points.SetValue(point, 0);
        pointsField.SetValue(joints, points);
        return value;
    }

    private static string ReadFixture(string name)
    {
        DirectoryInfo directory = new DirectoryInfo(Application.dataPath);
        while (directory != null)
        {
            string path = Path.Combine(directory.FullName, "Tools", "RobotFrames", name);
            if (File.Exists(path)) return File.ReadAllText(path);
            directory = directory.Parent;
        }
        throw new FileNotFoundException("Recorded ROS execution fixture is missing: " + name);
    }

    [Serializable] private sealed class ExecutionMessage { public string status; public string reason; }
    private GameObject New(string name) { var value = new GameObject(name); value.SetActive(false); _owned.Add(value); return value; }
    private Component Add(string name, string type)
    { Component value = New(name).AddComponent(TypeOf(type)); ((Behaviour)value).enabled = false; value.gameObject.SetActive(true); return value; }
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(assembly => assembly.GetType(name)).FirstOrDefault(type => type != null)
        ?? throw new InvalidOperationException("Missing runtime type " + name);
    private static FieldInfo Field(object value, string name)
    {
        for (Type type = value.GetType(); type != null; type = type.BaseType)
        {
            FieldInfo field = type.GetField(name, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.DeclaredOnly);
            if (field != null) return field;
        }
        throw new InvalidOperationException("Missing field " + value.GetType().Name + "." + name);
    }
    private static void Set(object value, string name, object data) => Field(value, name).SetValue(value, data);
    private static T Get<T>(object value, string name) => (T)value.GetType().GetProperty(name).GetValue(value);
    private static object Call(object value, string name, params object[] arguments) => value.GetType().GetMethods()
        .Single(method => method.Name == name && method.GetParameters().Length == arguments.Length).Invoke(value, arguments);
    private static object Private(object value, string name, params object[] arguments)
    {
        for (Type type = value.GetType(); type != null; type = type.BaseType)
        {
            MethodInfo method = type.GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
            if (method != null) return method.Invoke(value, arguments);
        }
        throw new InvalidOperationException("Missing private callback " + name);
    }
}
