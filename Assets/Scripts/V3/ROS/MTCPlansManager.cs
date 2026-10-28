using RosMessageTypes.Moveit;
using RosMessageTypes.Std;
using UnityEngine;
using UnityEngine.Events;

public class MTCPlansManager : MonoBehaviour
{
    [SerializeField]
    private UR16eTrajectoryVisualizer _UR16eTrajectoryVisualizer;
    [SerializeField, Min(10f)] private float _planningTimeout = 120f;
    [SerializeField, Min(1f)] private float _executionAcknowledgementTimeout = 10f;
    private bool _isLastPlanSuccessful = false;
    private RobotCoordinateService _coordinates;
    private bool _tilePlanningEnabled = true;
    private bool _hasPlanningRequest;
    private bool _acceptingResults;
    private bool _receivedInitialExecutionReset;
    private bool _receivedExecutionReady;
    private int _requestCalibrationRevision;
    private RobotTrajectoryMsg _pendingTrajectory;
    private float _planningDeadline;
    private bool _executionRequested;
    private bool _awaitingExecutionAcknowledgement;
    private float _executionDeadline;
    public bool HasPlanResult { get; private set; }
    public bool IsPlanning => IsCurrentRequest && _acceptingResults;
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
    public bool CanExecute => ActiveExecution == null && IsCurrentRequest && HasPlanResult && _isLastPlanSuccessful &&
        _UR16eTrajectoryVisualizer != null && _UR16eTrajectoryVisualizer.IsShowingTrajectory(_pendingTrajectory);
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
        _hasPlanningRequest = false;
        _acceptingResults = false;
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
        _pendingTrajectory = null;
        if (_UR16eTrajectoryVisualizer != null) _UR16eTrajectoryVisualizer.SetTrajectory(null);
    }

    private void SetReviewStatus(string status) => SetStatus(_executionRequested ? StatusText : status);

    public void SetTilePlanningEnabled(bool enabled)
    {
        if (_tilePlanningEnabled == enabled) return;
        _tilePlanningEnabled = enabled;
        ClearPlan();
        SetReviewStatus(enabled ? "MTC pick / place selected. Capture tool contacts, then Plan." : "MTC pick / place is inactive.");
    }

    /// <summary>Arm callbacks only for a deliberate local tile-plan request. Does not publish anything.</summary>
    public bool BeginPlanningRequest()
    {
        if (ActiveExecution != null) return false;
        if (IsPlanning) return false;
        ClearPlan();
        if (!CanUseTilePlanner)
        {
            SetStatus("MTC planning requires pick / place mode, a matching suction tool and valid robot alignment.");
            return false;
        }
        _hasPlanningRequest = true;
        _acceptingResults = true;
        _requestCalibrationRevision = RobotCoordinateService.Instance.CalibrationRevision;
        _planningDeadline = Time.realtimeSinceStartup + _planningTimeout;
        SetStatus("Planning MTC pick / place from tool contact poses...");
        return true;
    }

    public void RejectPlanningRequest(string reason)
    {
        ClearPlan();
        SetReviewStatus(reason);
    }

    public bool ConsumePlanForExecution()
    {
        if (!CanExecute) return false;
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
        SetStatus("MTC execution requested. Waiting for ROS acknowledgement...");
        return true;
    }

    public void RejectExecutionRequest(string reason)
    {
        if (!_executionRequested) return;
        FinishExecution("request failed", reason);
    }

    private void FinishExecution(string status, string reason)
    {
        _hasPlanningRequest = false;
        _executionRequested = false;
        _awaitingExecutionAcknowledgement = false;
        _executionDeadline = 0f;
        _acceptingResults = false;
        HasPlanResult = false;
        _isLastPlanSuccessful = false;
        SetStatus("MTC execution " + status + (string.IsNullOrEmpty(reason) ? "." : ": " + reason) +
            " Plan again before another execution.");
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
        ROSService.Instance.Subscribe<StringMsg>("/mtc/planning_status", OnPlanningStatus);
        ROSService.Instance.Subscribe<RobotTrajectoryMsg>("/mtc/planned_trajectory", OnPlannedTrajectory);
        ROSService.Instance.Subscribe<StringMsg>("/mtc/execution_status", OnExecutionStatus);
    }

    private void Update()
    {
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
        // Legacy MTC messages have no request ID. Ignore them until a tile request
        // is explicitly sent, and revoke that window whenever mode/targets change.
        if (!IsCurrentRequest || !_acceptingResults) return;
        PlanningStatus status;
        try { status = JsonUtility.FromJson<PlanningStatus>(message.data); }
        catch (System.Exception) { return; }
        if (status == null) return;
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
        if (status.status == "ready")
        {
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
                FinishExecution(status.status, status.reason);
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
        SetStatus("MTC pick / place plan ready. Review the ghost path, then Execute.");
    }

    private void OnDisable() => InvalidatePlan();

    private void OnDestroy()
    {
        if (_coordinates != null) _coordinates.CalibrationChanged -= InvalidatePlan;
    }

    [System.Serializable]
    private sealed class MtcExecutionStatus
    {
        public string status;
        public string reason;
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
}
