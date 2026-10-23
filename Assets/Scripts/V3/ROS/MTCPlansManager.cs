using RosMessageTypes.Moveit;
using RosMessageTypes.Std;
using UnityEngine;
using UnityEngine.Events;

public class MTCPlansManager : MonoBehaviour
{
    [SerializeField]
    private UR16eTrajectoryVisualizer _UR16eTrajectoryVisualizer;
    [SerializeField, Min(10f)] private float _planningTimeout = 120f;
    private bool _isLastPlanSuccessful = false;
    private RobotCoordinateService _coordinates;
    private bool _tilePlanningEnabled = true;
    private bool _hasPlanningRequest;
    private bool _acceptingResults;
    private int _requestCalibrationRevision;
    private RobotTrajectoryMsg _pendingTrajectory;
    private float _planningDeadline;
    public bool HasPlanResult { get; private set; }
    public bool IsPlanning => IsCurrentRequest && _acceptingResults;
    public bool CanExecute => IsCurrentRequest && HasPlanResult && _isLastPlanSuccessful &&
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
        SetStatus(_tilePlanningEnabled ? "Targets, tool or calibration changed. Plan MTC pick / place again." : "MTC pick / place is inactive.");
    }

    private void ClearPlan()
    {
        _hasPlanningRequest = false;
        _acceptingResults = false;
        HasPlanResult = false;
        _isLastPlanSuccessful = false;
        _pendingTrajectory = null;
        if (_UR16eTrajectoryVisualizer != null) _UR16eTrajectoryVisualizer.SetTrajectory(null);
    }

    public void SetTilePlanningEnabled(bool enabled)
    {
        if (_tilePlanningEnabled == enabled) return;
        _tilePlanningEnabled = enabled;
        ClearPlan();
        SetStatus(enabled ? "MTC pick / place selected. Capture tool contacts, then Plan." : "MTC pick / place is inactive.");
    }

    /// <summary>Arm callbacks only for a deliberate local tile-plan request. Does not publish anything.</summary>
    public bool BeginPlanningRequest()
    {
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
        SetStatus(reason);
    }

    public bool ConsumePlanForExecution()
    {
        if (!CanExecute) return false;
        _hasPlanningRequest = false;
        HasPlanResult = false;
        _isLastPlanSuccessful = false;
        SetStatus("MTC execution requested. Plan again before another execution.");
        return true;
    }

    private bool CanUseTilePlanner => _tilePlanningEnabled &&
        !(FindFirstObjectByType<URInputManager>() is URInputManager input && input.IsTcpMode) &&
        !RobotCoordinateService.IsCalibrationEditActive && RobotCoordinateService.Instance.IsInitialized &&
        ToolSelectionController.CanPlanPickPlace;

    private bool IsCurrentRequest => _hasPlanningRequest && CanUseTilePlanner &&
        RobotCoordinateService.Instance.CalibrationRevision == _requestCalibrationRevision;

    private void SetStatus(string status)
    {
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
        if (!_hasPlanningRequest) return;
        if (!IsCurrentRequest) { InvalidatePlan(); return; }
        if (_acceptingResults && Time.realtimeSinceStartup > _planningDeadline)
            RejectPlanningRequest("No complete MTC preview received. Check ROS planning logs, then Plan again.");
        else if (!_acceptingResults && (_UR16eTrajectoryVisualizer == null ||
            !_UR16eTrajectoryVisualizer.IsShowingTrajectory(_pendingTrajectory)))
            RejectPlanningRequest("The reviewed MTC trajectory is no longer displayed. Plan again.");
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
        // createTask publishes false with no stages when clearing the previous
        // task before planning. A terminal empty-task failure is reported on
        // execution_status; do not mistake this initial reset for plan failure.
        if (!status.overall_success && (status.stages == null || status.stages.Length == 0))
        { SetStatus("Planning MTC pick / place from tool contact poses..."); return; }
        HasPlanResult = true;
        _isLastPlanSuccessful = status.overall_success;
        if (!_isLastPlanSuccessful)
        {
            _hasPlanningRequest = false;
            _acceptingResults = false;
            _pendingTrajectory = null;
            if (_UR16eTrajectoryVisualizer != null) _UR16eTrajectoryVisualizer.SetTrajectory(null);
            SetStatus("MTC pick / place planning failed. Adjust tool targets and plan again.");
            return;
        }
        ShowPreviewIfComplete();
    }

    private void OnExecutionStatus(StringMsg message)
    {
        if (!IsCurrentRequest) return;
        MtcExecutionStatus status;
        try { status = JsonUtility.FromJson<MtcExecutionStatus>(message.data); }
        catch (System.Exception) { return; }
        if (status == null) return;
        if (status.status == "planning_failed" || status.status == "rejected" || status.status == "failed")
            RejectPlanningRequest("MTC planning " + status.status + ": " + status.reason);
        else if (status.status == "invalidated")
        {
            bool initialReset = _acceptingResults && !HasPlanResult &&
                status.reason == "New planning request replaced the previous plan";
            if (!initialReset) RejectPlanningRequest("MTC plan invalidated: " + status.reason + ". Plan again.");
        }
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
        { RejectPlanningRequest("MTC trajectory visualizer is missing. Execution is blocked."); return; }
        _UR16eTrajectoryVisualizer.SetTrajectory(_pendingTrajectory);
        if (!_UR16eTrajectoryVisualizer.IsShowingTrajectory(_pendingTrajectory))
        { RejectPlanningRequest("MTC trajectory could not be displayed. Execution is blocked."); return; }
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
}
