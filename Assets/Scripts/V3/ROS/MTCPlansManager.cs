using RosMessageTypes.Moveit;
using RosMessageTypes.Std;
using UnityEngine;
using UnityEngine.Events;
using Unity.Robotics.ROSTCPConnector;

public class MTCPlansManager : MonoBehaviour
{
    [SerializeField]
    private UR16eTrajectoryVisualizer _UR16eTrajectoryVisualizer;
    [SerializeField, Min(10f)] private float _planningTimeout = 120f;
    [SerializeField, Min(1f)] private float _executionAcknowledgementTimeout = 10f;
    private bool _isLastPlanSuccessful = false;
    private RobotCoordinateService _coordinates;
    private ROSConnection _connection;
    private bool _tilePlanningEnabled = true;
    private bool _hasPlanningRequest;
    private bool _acceptingResults;
    private string _planningRequestId;
    private MtcPlanningStop _planningStop;
    private MtcPlanningStop PlanningStop => _planningStop ??=
        new MtcPlanningStop("pick", RefreshPlanningStopFeedback, OnPlanningStopFinished);
    private bool _receivedInitialExecutionReset;
    private bool _receivedExecutionReady;
    private int _requestCalibrationRevision;
    private RobotTrajectoryMsg _pendingTrajectory;
    private float _planningDeadline;
    private bool _executionRequested;
    private bool _awaitingExecutionAcknowledgement;
    private float _executionDeadline;
    [SerializeField] private JointStateSubscriber _jointFeedback;
    private readonly RobotPlanRepeatReview _repeat = new RobotPlanRepeatReview();
    private string _executionAttemptId = "";
    private string _executionPlanId;
    private string _reviewedPlanId;
    private bool? _repeatStartReady;
    public string RepeatExecutionPayload { get; private set; }
    public string IdentifiedExecutionPlanId { get; private set; }
    public bool HasRepeatablePlan => _repeat.HasRetainedPlan;
    public bool CanRepeat => !RobotExecutionState.IsBusy && IsReviewContextValid && _repeat.HasRetainedPlan &&
        _UR16eTrajectoryVisualizer != null && _UR16eTrajectoryVisualizer.IsShowingTrajectory(_pendingTrajectory) &&
        RepeatStartMatches(out _);
    internal string RepeatConfirmationKey => CanRepeat ? _repeat.PlanId + "/" + _repeat.AttemptId : null;
    public bool HasPlanResult { get; private set; }
    public bool IsPlanning => _planningStop?.IsActive == true || (IsCurrentRequest && _acceptingResults);
    public bool CanStopPlanning => !_executionRequested && (_planningStop?.CanRequestStop == true ||
        (_planningStop?.IsActive != true && !string.IsNullOrEmpty(_planningRequestId) && IsCurrentRequest && _acceptingResults));
    public bool IsStoppingPlanning => _planningStop?.StopRequested == true;
    public bool IsExecuting => _executionRequested;
    public bool HasExecutionFeedback { get; private set; }
    public static MTCPlansManager ActiveExecution
    {
        get
        {
            foreach (MTCPlansManager plans in FindObjectsByType<MTCPlansManager>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                if (plans.IsExecuting) return plans;
            return null;
        }
    }

    public static void DismissCompletedExecutionFeedback()
    {
        foreach (MTCPlansManager plans in FindObjectsByType<MTCPlansManager>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (plans.IsExecuting || !plans.HasExecutionFeedback) continue;
            plans.HasExecutionFeedback = false;
            plans.OnSuccessfulPlan?.Invoke(false);
        }
    }
    public bool CanExecute => !IsPlanning && !RobotExecutionState.IsBusy && (CanRepeat || (ActiveExecution == null && IsCurrentRequest && HasPlanResult && _isLastPlanSuccessful &&
        (string.IsNullOrEmpty(_planningRequestId) || !string.IsNullOrEmpty(_reviewedPlanId)) &&
        _UR16eTrajectoryVisualizer != null && _UR16eTrajectoryVisualizer.IsShowingTrajectory(_pendingTrajectory)));
    public string StatusText { get; private set; } = "Capture pick and place tool contacts, then Plan.";
    public UnityEvent<bool> OnSuccessfulPlan;

    public bool IsLastPlanSuccessful 
    { 
        get { return _isLastPlanSuccessful; } 
    }

    public void InvalidatePlan()
    {
        ClearPlan();
        SetReviewStatus(_tilePlanningEnabled ? "Targets, tool or calibration changed. Plan MTC pick / place again." : "MTC pick / place is inactive.");
    }

    private void ClearPlan()
    {
        _repeat.Invalidate();
        _repeatStartReady = null;
        _reviewedPlanId = null;
        _hasPlanningRequest = false;
        _acceptingResults = false;
        _planningRequestId = null;
        // Editing a target, changing mode or disabling the component revokes
        // review permission, but cannot cancel a command already sent to ROS.
        if (!_executionRequested)
        {
            _receivedInitialExecutionReset = false;
            _receivedExecutionReady = false;
            HasExecutionFeedback = false;
        }
        HasPlanResult = false;
        _isLastPlanSuccessful = false;
        _UR16eTrajectoryVisualizer?.ClearTrajectoryIfMatches(_pendingTrajectory);
        _pendingTrajectory = null;
    }

    private void SetReviewStatus(string status)
    {
        if (_executionRequested) SetStatus(StatusText);
        else if (IsStoppingPlanning) RefreshPlanningStopFeedback();
        else SetStatus(status);
    }

    public void SetTilePlanningEnabled(bool enabled)
    {
        if (_tilePlanningEnabled == enabled) return;
        _tilePlanningEnabled = enabled;
        ClearPlan();
        SetReviewStatus(enabled ? "MTC pick / place selected. Capture tool contacts, then Plan." : "MTC pick / place is inactive.");
    }

    /// <summary>Arm callbacks only for a deliberate local tile-plan request. Does not publish anything.</summary>
    public bool BeginPlanningRequest() => BeginPlanningRequestCore(null, null, _planningTimeout);

    public bool BeginCorrelatedPlanningRequest(ROSConnection connection, string requestId) =>
        BeginPlanningRequestCore(connection, requestId, _planningTimeout);

    internal bool BeginTimedPlanningRequest(ROSConnection connection, string requestId, int timeoutSeconds, bool stopSupported) =>
        BeginPlanningRequestCore(connection, requestId, timeoutSeconds + MtcPlanningStop.ResultGraceSeconds, stopSupported);

    private bool BeginPlanningRequestCore(ROSConnection connection, string requestId, float resultTimeout, bool stopSupported = true)
    {
        if (RobotExecutionState.IsBusy || ActiveExecution != null) return false;
        if (IsPlanning) return false;
        ClearPlan();
        if (!CanUseTilePlanner)
        {
            SetStatus("MTC planning requires pick / place mode, a matching suction tool and valid robot alignment.");
            return false;
        }
        _hasPlanningRequest = true;
        _acceptingResults = true;
        _planningRequestId = requestId;
        _requestCalibrationRevision = RobotCoordinateService.Instance.CalibrationRevision;
        _planningDeadline = Time.realtimeSinceStartup + resultTimeout;
        if (!string.IsNullOrEmpty(requestId)) PlanningStop.Begin(connection, requestId, stopSupported, _planningDeadline);
        SetStatus("Planning MTC pick / place from tool contact poses...");
        return true;
    }

    public bool RequestPlanningStop()
    {
        if (!CanStopPlanning) return false;
        ClearPlan();
        if (_planningStop?.IsActive == true) return _planningStop.RequestStop();
        SetStatus("MTC preview discarded. Plan again.");
        return true;
    }

    public void ForgetUnsentPlanningRequest() => _planningStop?.ForgetUnsent();

    private void RefreshPlanningStopFeedback()
    {
        if (IsStoppingPlanning && _hasPlanningRequest) ClearPlan();
        SetStatus(_planningStop?.Feedback ?? StatusText);
    }

    private void OnPlanningStopFinished(string status, string reason, bool stopRequested)
    {
        if (status == "cancelled") RejectPlanningRequest("MTC planning stopped. Plan again.");
        else if (status == "timed_out" || status == "unconfirmed") RejectPlanningRequest(reason + " Plan again.");
        else if (status == "rejected") RejectPlanningRequest("ROS rejected MTC planning: " + reason);
        else if (stopRequested) RejectPlanningRequest("MTC preview discarded. Plan again.");
        else SetStatus(StatusText);
    }

    public void RejectPlanningRequest(string reason)
    {
        ClearPlan();
        SetReviewStatus(reason);
    }

    public bool ConsumePlanForExecution() => ConsumeExecution(false);

    internal bool ConsumeConfirmedRepeat(string confirmationKey) =>
        !string.IsNullOrEmpty(confirmationKey) && confirmationKey == RepeatConfirmationKey && ConsumeExecution(true);

    private bool ConsumeExecution(bool confirmedRepeat)
    {
        if (!CanExecute || _repeat.HasRetainedPlan != confirmedRepeat) return false;
        RepeatExecutionPayload = null;
        IdentifiedExecutionPlanId = null;
        _executionPlanId = _reviewedPlanId;
        _executionAttemptId = "";
        if (confirmedRepeat)
        {
            if (!RepeatStartMatches(out _) || !_repeat.TryBeginAttempt(out _executionPlanId, out _executionAttemptId)) return false;
            RepeatExecutionPayload = JsonUtility.ToJson(new RepeatRequest
            {
                plan_id = _executionPlanId, attempt_id = _executionAttemptId, placed_item_removed = true
            });
        }
        else if (!string.IsNullOrEmpty(_planningRequestId)) IdentifiedExecutionPlanId = _executionPlanId;
        // Consuming permission must not stop listening for the outcome of the
        // command. ROS can reject its start-state/scene preflight without motion.
        _executionRequested = true;
        HasExecutionFeedback = true;
        _awaitingExecutionAcknowledgement = true;
        _executionDeadline = Time.realtimeSinceStartup + _executionAcknowledgementTimeout;
        _acceptingResults = false;
        _hasPlanningRequest = false;
        HasPlanResult = false;
        _isLastPlanSuccessful = false;
        RobotExecutionState.SetBusy(this, true);
        SetStatus("MTC execution requested. Waiting for ROS acknowledgement...");
        return true;
    }

    public void RejectExecutionRequest(string reason)
    {
        if (!_executionRequested) return;
        FinishExecution("request failed", reason);
    }

    private void FinishExecution(string status, string reason, bool repeatable = false, string planId = null,
        string attemptId = null, bool removalConfirmationSupported = false)
    {
        _repeat.Complete(status == "succeeded", repeatable && removalConfirmationSupported &&
            !string.IsNullOrEmpty(_executionPlanId) && _executionPlanId == planId, planId, attemptId);
        if (!IsReviewContextValid || _pendingTrajectory == null) _repeat.Invalidate();
        _hasPlanningRequest = false;
        _executionRequested = false;
        IdentifiedExecutionPlanId = null;
        _awaitingExecutionAcknowledgement = false;
        _executionDeadline = 0f;
        _acceptingResults = false;
        HasPlanResult = false;
        _isLastPlanSuccessful = false;
        RobotExecutionState.SetBusy(this, false);
        _repeatStartReady = null;
        string next = " Plan again before another execution.";
        if (_repeat.HasRetainedPlan) next = " Return to the initial joints to review the removal confirmation.";
        else if (status == "succeeded" && repeatable && !removalConfirmationSupported)
            next = " Update ROS to repeat with placed-item removal confirmation.";
        SetStatus("MTC execution " + status + (string.IsNullOrEmpty(reason) ? "." : ": " + reason) + next);
    }

    private bool RepeatStartMatches(out string reason)
    {
        reason = null;
        if (_jointFeedback == null) _jointFeedback = FindFirstObjectByType<JointStateSubscriber>();
        RosRobotModel model = ToolSelectionController.Instance != null ? ToolSelectionController.Instance.CurrentModel : null;
        if (_jointFeedback == null) { reason = "Waiting for fresh ROS joint feedback before repeating."; return false; }
        if (!_jointFeedback.TryGetFreshJointPositions(model, 1f, out var positions, out reason)) return false;
        return _repeat.TryMatchStart(positions, 0.01d, out reason);
    }

    private bool CanUseTilePlanner => _tilePlanningEnabled &&
        !(FindFirstObjectByType<URInputManager>() is URInputManager input && input.IsTcpMode) &&
        !RobotCoordinateService.IsCalibrationEditActive && RobotCoordinateService.Instance.IsInitialized &&
        ToolSelectionController.CanPlanPickPlace;

    private bool IsReviewContextValid => CanUseTilePlanner &&
        RobotCoordinateService.Instance.CalibrationRevision == _requestCalibrationRevision;
    private bool IsCurrentRequest => _hasPlanningRequest && IsReviewContextValid;

    private void SetStatus(string status)
    {
        if (StatusText != status) Debug.Log("MTC: " + status, this);
        StatusText = status;
        OnSuccessfulPlan?.Invoke(HasPlanResult && _isLastPlanSuccessful);
    }

    private void Awake()
    {
        OnSuccessfulPlan = new UnityEvent<bool>();
    }

    void Start()
    {
        _coordinates = RobotCoordinateService.Instance;
        _coordinates.CalibrationChanged += InvalidatePlan;
        _connection = ROSService.Instance.GetROSConnection();
        PlanningStop.Bind(_connection);
        _connection.Subscribe<StringMsg>("/mtc/planning_status", OnPlanningStatus);
        _connection.Subscribe<RobotTrajectoryMsg>("/mtc/planned_trajectory", OnPlannedTrajectory);
        _connection.Subscribe<StringMsg>("/mtc/execution_status", OnExecutionStatus);
    }

    private void Update()
    {
        _planningStop?.Tick();
        if (IsStoppingPlanning)
        {
            if (StatusText != _planningStop.Feedback) RefreshPlanningStopFeedback();
            return;
        }
        if (_executionRequested)
        {
            if (_pendingTrajectory != null && !IsReviewContextValid) InvalidatePlan();
            if (_awaitingExecutionAcknowledgement && Time.realtimeSinceStartup > _executionDeadline)
            {
                _awaitingExecutionAcknowledgement = false;
                SetStatus("MTC execution acknowledgement timed out. The outcome is unknown; inspect the robot and ROS logs. Execute will not be resent.");
            }
            return;
        }
        if (_repeat.HasRetainedPlan)
        {
            if (!IsReviewContextValid || _UR16eTrajectoryVisualizer == null || !_UR16eTrajectoryVisualizer.IsShowingTrajectory(_pendingTrajectory))
            { InvalidatePlan(); return; }
            bool ready = RepeatStartMatches(out string reason);
            if (_repeatStartReady != ready)
            {
                _repeatStartReady = ready;
                SetStatus(ready ? "MTC ready to execute again. Initial joints match; ROS will recheck the full scene." : reason);
            }
            return;
        }
        if (!_hasPlanningRequest) return;
        if (!IsCurrentRequest) { InvalidatePlan(); return; }
        if (_acceptingResults && Time.realtimeSinceStartup > _planningDeadline)
            RejectPlanningRequest(HasPlanResult && _isLastPlanSuccessful
                ? "ROS MTC planning succeeded, but Unity did not receive and display its trajectory preview. Check the Unity robot model and ROS connection, then Plan again."
                : "No complete MTC preview received. Check ROS planning logs, then Plan again.");
        else if (!_acceptingResults && (_UR16eTrajectoryVisualizer == null ||
            !_UR16eTrajectoryVisualizer.IsShowingTrajectory(_pendingTrajectory)))
            RejectPlanningRequest("The reviewed MTC trajectory is no longer displayed. Plan again.");
        else if (_acceptingResults && HasPlanResult && _isLastPlanSuccessful && _pendingTrajectory != null)
            ShowPreviewIfComplete();
    }

    private void OnPlanningStatus(StringMsg message)
    {
        // Keep legacy zero-stamped requests compatible. Correlated requests only
        // accept their own status, even after an earlier plan has been stopped.
        if (!IsCurrentRequest || !_acceptingResults) return;
        PlanningStatus status;
        try { status = JsonUtility.FromJson<PlanningStatus>(message.data); }
        catch (System.Exception) { return; }
        if (status == null || (status.request_id ?? "") != (_planningRequestId ?? "")) return;
        // A freshly constructed MTC task may report its empty root container,
        // rather than an empty stages array. Neither is a terminal failure. ROS
        // publishes an error message / failed stage counts or execution_status
        // planning_failed for a real failure.
        if (IsPlanningReset(status))
        {
            if (!HasPlanResult) SetStatus("Planning MTC pick / place from tool contact poses...");
            return;
        }
        HasPlanResult = true;
        _isLastPlanSuccessful = status.overall_success;
        if (_isLastPlanSuccessful && !string.IsNullOrEmpty(_planningRequestId) && string.IsNullOrEmpty(status.plan_id))
        {
            RejectPlanningRequest("ROS did not identify the MTC plan. Plan again.");
            return;
        }
        if (_isLastPlanSuccessful && !string.IsNullOrEmpty(status.plan_id))
        {
            if (!string.IsNullOrEmpty(_reviewedPlanId) && _reviewedPlanId != status.plan_id)
            { RejectPlanningRequest("MTC plan identifiers do not match. Plan again."); return; }
            _reviewedPlanId = status.plan_id;
        }
        if (!_isLastPlanSuccessful)
        {
            _hasPlanningRequest = false;
            _acceptingResults = false;
            _pendingTrajectory = null;
            if (_UR16eTrajectoryVisualizer != null) _UR16eTrajectoryVisualizer.SetTrajectory(null);
            SetStatus(string.IsNullOrEmpty(status.message)
                ? "ROS MTC pick / place planning failed. Adjust tool targets and plan again."
                : "ROS MTC pick / place planning failed: " + status.message);
            return;
        }
        ShowPreviewIfComplete();
    }

    private static bool IsPlanningReset(PlanningStatus status)
    {
        if (status.overall_success || !string.IsNullOrEmpty(status.message)) return false;
        if (status.stages != null)
            foreach (StageStatus stage in status.stages)
                if (stage != null && (stage.solved != 0 || stage.failed != 0)) return false;
        return true;
    }

    private void OnExecutionStatus(StringMsg message)
    {
        // Execution outcomes belong to the command already sent, independently
        // of the currently selected mode, edited targets or calibration.
        if (!_executionRequested && !IsCurrentRequest) return;
        MtcExecutionStatus status;
        try { status = JsonUtility.FromJson<MtcExecutionStatus>(message.data); }
        catch (System.Exception) { return; }
        if (status == null) return;
        if (!_executionRequested && !string.IsNullOrEmpty(_planningRequestId) && status.request_id != _planningRequestId) return;
        if (_executionRequested && ((_executionAttemptId ?? "") != (status.attempt_id ?? "") ||
            (!string.IsNullOrEmpty(_executionPlanId) && _executionPlanId != status.plan_id))) return;
        if (status.status == "ready")
        {
            if (!_executionRequested && !string.IsNullOrEmpty(status.plan_id))
            {
                if (!string.IsNullOrEmpty(_reviewedPlanId) && _reviewedPlanId != status.plan_id) return;
                _reviewedPlanId = status.plan_id;
            }
            // A ready callback can arrive after the separate result and preview
            // topics, even after Execute. It must never re-arm a consumed plan.
            _receivedExecutionReady = true;
            return;
        }
        // Three independent topics can deliver the planning reset after the
        // result/preview, even if Execute has already consumed the permission.
        // One reset before ready is harmless; later invalidations revoke it.
        bool initialReset = status.status == "invalidated" && !_receivedInitialExecutionReset && !_receivedExecutionReady &&
            status.reason == "New planning request replaced the previous plan";
        if (initialReset) { _receivedInitialExecutionReset = true; return; }
        if (_executionRequested)
        {
            if (status.status == "executing")
            {
                _awaitingExecutionAcknowledgement = false;
                SetStatus("MTC executing" + (string.IsNullOrEmpty(status.reason) ? "." : ": " + status.reason));
            }
            else if (status.status == "succeeded" || status.status == "failed" ||
                status.status == "rejected" || status.status == "invalidated")
                FinishExecution(status.status, status.reason, status.repeatable, status.plan_id, status.attempt_id,
                    status.placed_item_confirmation_required);
            return;
        }
        if (status.status == "planning_failed" || status.status == "rejected" || status.status == "failed")
            RejectPlanningRequest("MTC planning " + status.status + ": " + status.reason);
        else if (status.status == "invalidated")
            RejectPlanningRequest("MTC plan invalidated: " + status.reason + ". Plan again.");
    }

    private void OnPlannedTrajectory(RobotTrajectoryMsg trajectory)
    {
        if (!IsCurrentRequest || !_acceptingResults || trajectory?.joint_trajectory?.points == null || trajectory.joint_trajectory.points.Length == 0) return;
        var stamp = trajectory.joint_trajectory.header?.stamp;
        if (!string.IsNullOrEmpty(_planningRequestId) && (stamp == null || MtcPlanningStop.RequestId(stamp) != _planningRequestId)) return;
        _pendingTrajectory = trajectory;
        ShowPreviewIfComplete();
    }

    private void ShowPreviewIfComplete()
    {
        if (!HasPlanResult) { SetStatus("MTC trajectory received. Waiting for its planning result..."); return; }
        if (_pendingTrajectory == null) { SetStatus("MTC plan succeeded. Waiting for its trajectory preview..."); return; }
        if (_UR16eTrajectoryVisualizer == null)
        { RejectPlanningRequest("ROS MTC planning succeeded, but the Unity trajectory visualizer is missing. Execution is blocked."); return; }
        if (!_UR16eTrajectoryVisualizer.IsReadyForPreview)
        {
            SetStatus("ROS MTC plan succeeded. Waiting for the Unity robot model to display its trajectory preview...");
            return;
        }
        _UR16eTrajectoryVisualizer.SetTrajectory(_pendingTrajectory);
        if (!_UR16eTrajectoryVisualizer.IsShowingTrajectory(_pendingTrajectory))
        { RejectPlanningRequest("ROS MTC planning succeeded, but its trajectory could not be displayed in Unity. Execution is blocked."); return; }
        _acceptingResults = false;
        _repeat.CaptureStart(_pendingTrajectory.joint_trajectory.joint_names, _pendingTrajectory.joint_trajectory.points[0].positions);
        SetStatus("MTC pick / place plan ready. Review the ghost path, then Execute.");
    }

    private void OnDisable() => InvalidatePlan();

    private void OnDestroy()
    {
        _planningStop?.Dispose();
        RobotExecutionState.SetBusy(this, false);
        if (_coordinates != null) _coordinates.CalibrationChanged -= InvalidatePlan;
        if (_connection != null)
        {
            _connection.Unsubscribe("/mtc/planning_status");
            _connection.Unsubscribe("/mtc/planned_trajectory");
            _connection.Unsubscribe("/mtc/execution_status");
        }
    }

    [System.Serializable]
    private sealed class MtcExecutionStatus
    {
        public string status;
        public string reason;
        public string request_id;
        public string plan_id;
        public string attempt_id;
        public bool repeatable;
        public bool placed_item_confirmation_required;
    }
    [System.Serializable] private sealed class RepeatRequest
    {
        public string plan_id, attempt_id;
        public bool placed_item_removed;
    }
}

[System.Serializable]
public class StageStatus
{
    public string name;
    public int depth;
    public int solved;
    public int failed;
}

[System.Serializable]
public class PlanningStatus
{
    public bool overall_success;
    public StageStatus[] stages;
    public string message;
    public string request_id;
    public string plan_id;
}
