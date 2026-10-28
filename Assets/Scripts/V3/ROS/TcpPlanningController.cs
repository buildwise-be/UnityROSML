using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Moveit;
using RosMessageTypes.RclInterfaces;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>Exact TCP A/B planning. Plan never sends a motion or vacuum command.</summary>
public sealed class TcpPlanningController : MonoBehaviour
{
    private const string PlanTopic = "/tcp_poses_plan_target";
    private const string ExecuteTopic = "/execute_tcp_plan";
    private const string CapabilityService = "/mtc_test_task_node/get_parameters";
    [SerializeField] private TcpTargetPreview _targets;
    [SerializeField] private UR16eTrajectoryVisualizer _visualizer;
    [SerializeField, Min(1f)] private float _capabilityTimeout = 10f;
    [SerializeField, Min(10f)] private float _planningTimeout = 120f;

    private readonly TcpPlanReview _review = new TcpPlanReview();
    private ROSConnection _connection;
    private RobotCoordinateService _coordinates;
    private PoseStampedMsg _requestedA, _requestedB;
    private RobotTrajectoryMsg _pendingPreview;
    private string _executionRequestId;
    private bool _checkingCapability;
    private float _deadline;
    private int _generation;
    private static long _lastRequestTicks;
    public string StatusText { get; private set; } = "Capture TCP A (hold bumper) and B (double bumper), then Plan.";
    public bool IsPlanning => _checkingCapability || (_review.IsPending && !_review.CanExecute);
    public bool CanExecute => _review.CanExecute && _visualizer != null && _visualizer.IsShowingTrajectory(_pendingPreview) &&
        TryValidate(out _) && TargetsMatchRequest();
    public event Action StateChanged;

    private void Start()
    {
        if (_targets == null) _targets = GetComponent<TcpTargetPreview>();
        if (_visualizer == null) _visualizer = FindFirstObjectByType<UR16eTrajectoryVisualizer>();
        if (_targets != null) _targets.TargetsChanged += OnTargetsChanged;
        _coordinates = RobotCoordinateService.Instance;
        _coordinates.CalibrationChanged += OnCalibrationChanged;
        _connection = ROSService.Instance.GetROSConnection();
        _connection.RegisterPublisher<PoseArrayMsg>(PlanTopic);
        _connection.RegisterPublisher<StringMsg>(ExecuteTopic);
        _connection.RegisterRosService<GetParametersRequest, GetParametersResponse>(CapabilityService);
        _connection.Subscribe<StringMsg>("/mtc/tcp_planning_status", OnStatus);
        _connection.Subscribe<RobotTrajectoryMsg>("/mtc/tcp_planned_trajectory", OnTrajectory);
        _connection.Subscribe<StringMsg>("/mtc/tcp_execution_status", OnExecutionStatus);
    }

    private void Update()
    {
        if (!_checkingCapability && !_review.IsPending) return;
        if (!TryValidate(out string reason)) { InvalidatePlan(reason); return; }
        if (!TargetsMatchRequest()) { InvalidatePlan("Targets or calibration changed. Plan again."); return; }
        if (_review.CanExecute && (_visualizer == null || !_visualizer.IsShowingTrajectory(_pendingPreview)))
        { InvalidatePlan("The reviewed trajectory is no longer displayed. Plan again."); return; }
        if (IsPlanning && Time.realtimeSinceStartup > _deadline)
            InvalidatePlan(_checkingCapability
                ? "ROS TCP planner did not answer. Check that the updated MTC node is running."
                : "No complete TCP preview received. Check ROS planning logs, then Plan again.");
    }

    public void Plan()
    {
        if (MTCPlansManager.ActiveExecution != null)
        { SetStatus("An MTC execution request is unresolved. Wait for its ROS outcome before planning another task."); return; }
        MTCPlansManager.DismissCompletedExecutionFeedback();
        if (IsPlanning) { SetStatus("A TCP plan is already pending. Change a target to cancel it, or wait for the result."); return; }
        InvalidatePlan(null);
        if (!TryValidate(out string reason)) { SetStatus(reason); return; }
        if (_targets == null || !_targets.TryGetCapturedRosPose(true, out _requestedA) ||
            !_targets.TryGetCapturedRosPose(false, out _requestedB))
        { SetStatus("Capture both TCP A (hold bumper) and B (double bumper) first."); return; }

        int generation = _generation;
        _checkingCapability = true;
        _deadline = Time.realtimeSinceStartup + _capabilityTimeout;
        SetStatus("Checking ROS plan-only TCP support...");
        _connection.SendServiceMessage<GetParametersResponse>(CapabilityService,
            new GetParametersRequest(new[] { "tcp_pose_planning_enabled" }), response =>
            {
                if (this == null || generation != _generation || !_checkingCapability) return;
                _checkingCapability = false;
                if (!TryValidate(out string blocked)) { InvalidatePlan(blocked); return; }
                if (!TargetsMatchRequest()) { InvalidatePlan("Targets or calibration changed. Plan again."); return; }
                if (response?.values == null || response.values.Length != 1 ||
                    response.values[0].type != 1 || !response.values[0].bool_value)
                { InvalidatePlan("ROS needs the plan-only TCP update. Existing direct-TCP topics execute immediately."); return; }
                TimeMsg stamp = NewRequestStamp();
                _review.Begin(RequestId(stamp));
                _deadline = Time.realtimeSinceStartup + _planningTimeout;
                _connection.Publish(PlanTopic, new PoseArrayMsg(
                    new HeaderMsg { frame_id = _requestedA.header.frame_id, stamp = stamp },
                    new[] { _requestedA.pose, _requestedB.pose }));
                SetStatus("Planning exact TCP A → B. No motion requested.");
            });
    }

    public void Execute()
    {
        if (!TryValidate(out string reason)) { InvalidatePlan(reason); return; }
        if (!TargetsMatchRequest()) { InvalidatePlan("Targets or calibration changed. Plan again."); return; }
        if (_review.CanExecute && (_visualizer == null || !_visualizer.IsShowingTrajectory(_pendingPreview)))
        { InvalidatePlan("The reviewed trajectory is no longer displayed. Plan again."); return; }
        if (!_review.TryConsume(out string requestId))
        { SetStatus("Execute requires a successful TCP plan and its visible trajectory preview."); return; }
        // A token names the reviewed frozen ROS solution; it cannot execute another client's plan.
        _executionRequestId = requestId;
        _connection.Publish(ExecuteTopic, new StringMsg(requestId));
        SetStatus("Execution requested for the reviewed TCP plan. Plan again before another execution.");
    }

    public void InvalidatePlan(string reason = "Robot model or calibration changed. Plan again.")
    {
        _generation++;
        _checkingCapability = false;
        _review.Invalidate();
        _pendingPreview = null;
        _executionRequestId = null;
        _requestedA = _requestedB = null;
        if (_visualizer != null) _visualizer.SetTrajectory(null);
        if (reason != null) SetStatus(reason);
    }

    private void OnTargetsChanged() => InvalidatePlan("TCP target changed. Plan again to preview its new pose.");
    private void OnCalibrationChanged() => InvalidatePlan("QR calibration changed. Confirm alignment, then Plan again.");

    private void OnStatus(StringMsg message)
    {
        TcpStatus status;
        try { status = JsonUtility.FromJson<TcpStatus>(message.data); }
        catch (Exception) { return; }
        if (status != null && _review.IsPending && status.phase == "planning" && status.request_id == _review.RequestId)
        { SetStatus("Planning exact TCP A → B. No motion requested."); return; }
        if (status == null || !_review.AcceptResult(status.request_id, status.overall_success)) return;
        if (!status.overall_success)
        { InvalidatePlan("TCP planning failed or invalidated. " + status.message); return; }
        ShowPreviewIfComplete();
    }

    private void OnTrajectory(RobotTrajectoryMsg trajectory)
    {
        if (trajectory?.joint_trajectory?.header?.stamp == null || trajectory.joint_trajectory.points == null ||
            trajectory.joint_trajectory.points.Length == 0 ||
            !_review.AcceptPreview(RequestId(trajectory.joint_trajectory.header.stamp))) return;
        _pendingPreview = trajectory;
        ShowPreviewIfComplete();
    }

    private void OnExecutionStatus(StringMsg message)
    {
        ExecutionStatus status;
        try { status = JsonUtility.FromJson<ExecutionStatus>(message.data); }
        catch (Exception) { return; }
        if (status == null || _executionRequestId == null || status.request_id != _executionRequestId) return;
        SetStatus("TCP execution " + status.status + ": " + status.reason);
    }

    private void ShowPreviewIfComplete()
    {
        if (!_review.CanExecute)
        {
            SetStatus(_review.HasPreview
                ? "TCP trajectory received. Waiting for the matching planning result..."
                : "TCP plan succeeded. Waiting for its matching trajectory preview...");
            return;
        }
        if (!TryValidate(out string reason)) { InvalidatePlan(reason); return; }
        if (!TargetsMatchRequest()) { InvalidatePlan("Targets or calibration changed. Plan again."); return; }
        if (_visualizer == null) { InvalidatePlan("Trajectory visualizer is missing. Execution is blocked."); return; }
        _visualizer.SetTrajectory(_pendingPreview);
        if (!_visualizer.HasVisiblePreview)
        { InvalidatePlan("The returned trajectory could not be displayed. Execution is blocked; check Unity logs."); return; }
        SetStatus("TCP plan ready. Inspect the ghost path and A/B orientations, then Execute when ready.");
    }

    private bool TryValidate(out string reason)
    {
        reason = null;
        if (MTCPlansManager.ActiveExecution != null)
            reason = "An MTC execution request is unresolved. Wait for its ROS outcome before planning or executing another task.";
        else if (RobotCoordinateService.IsCalibrationEditActive)
            reason = "Finish or cancel robot alignment before planning or executing.";
        else if (_connection == null || !_connection.HasConnectionThread || _connection.HasConnectionError)
            reason = "ROS is disconnected. Start the ROS stack and reconnect.";
        else if (TargetPoseContextMenu.IsOpen) reason = "Close the target menu before planning or executing.";
        else if (TargetRotationHandles.IsInteracting) reason = "Release the tool rotation handle first.";
        else if (!RobotCoordinateService.Instance.IsInitialized) reason = "Scan the robot QR and confirm robot placement first.";
        else if (ToolSelectionController.Instance == null || !ToolSelectionController.Instance.IsRosModelMatched)
            reason = "Waiting for matching robot, MoveIt and MTC models. Check the tool status.";
        else if (GetComponent<URInputManager>() is URInputManager input && !input.IsTcpMode)
            reason = "Select exact TCP capture mode before planning TCP A/B.";
        return reason == null;
    }

    private bool TargetsMatchRequest() => _requestedA != null && _requestedB != null && _targets != null &&
        _targets.TryGetCapturedRosPose(true, out PoseStampedMsg a) && _targets.TryGetCapturedRosPose(false, out PoseStampedMsg b) &&
        SamePose(_requestedA, a) && SamePose(_requestedB, b);

    private static bool SamePose(PoseStampedMsg a, PoseStampedMsg b)
    {
        if (a.header.frame_id != b.header.frame_id) return false;
        PointMsg p = a.pose.position, q = b.pose.position;
        QuaternionMsg r = a.pose.orientation, s = b.pose.orientation;
        double dot = r.x * s.x + r.y * s.y + r.z * s.z + r.w * s.w;
        double norm = Math.Sqrt((r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w) *
            (s.x * s.x + s.y * s.y + s.z * s.z + s.w * s.w));
        return Math.Abs(p.x - q.x) < 0.00001 && Math.Abs(p.y - q.y) < 0.00001 &&
            Math.Abs(p.z - q.z) < 0.00001 && norm > 0 && Math.Abs(dot / norm) > 0.9999999;
    }

    private void SetStatus(string status)
    {
        if (StatusText == status) { StateChanged?.Invoke(); return; }
        StatusText = status;
        Debug.Log("TCP planning: " + status, this);
        StateChanged?.Invoke();
    }

    private static TimeMsg NewRequestStamp()
    {
        long ticks = DateTime.UtcNow.Ticks - 621355968000000000L;
        ticks = Math.Max(ticks, _lastRequestTicks + 1);
        _lastRequestTicks = ticks;
#if ROS2
        int seconds = (int)(ticks / 10000000L);
#else
        uint seconds = (uint)(ticks / 10000000L);
#endif
        return new TimeMsg(seconds, (uint)((ticks % 10000000L) * 100L));
    }

    private static string RequestId(TimeMsg stamp) => stamp.sec + ":" + stamp.nanosec;
    [Serializable] private sealed class TcpStatus
    { public string request_id; public bool overall_success; public string message; public string phase; }
    [Serializable] private sealed class ExecutionStatus
    { public string request_id; public string status; public string reason; }

    private void OnDisable() { InvalidatePlan("TCP planning paused. Plan again after enabling it."); }
    private void OnDestroy()
    {
        _generation++;
        if (_targets != null) _targets.TargetsChanged -= OnTargetsChanged;
        if (_coordinates != null) _coordinates.CalibrationChanged -= OnCalibrationChanged;
        if (_connection != null)
        {
            _connection.Unsubscribe("/mtc/tcp_planning_status");
            _connection.Unsubscribe("/mtc/tcp_planned_trajectory");
            _connection.Unsubscribe("/mtc/tcp_execution_status");
        }
        StateChanged = null;
    }
}
