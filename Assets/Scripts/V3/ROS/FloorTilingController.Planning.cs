using System;
using System.Collections.Generic;
using RosMessageTypes.Moveit;
using RosMessageTypes.MoveitTaskConstructor;
using RosMessageTypes.RclInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Ur16eInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public partial class FloorTilingController
{
    private const string PlanTopic = "/tile_floor_plan_target";
    private const string ExecuteTopic = "/execute_tile_floor_plan";
    private const string StatusTopic = "/mtc/tile_floor/status";
    private const string SceneTopic = "/mtc/tile_floor/preview_scene";
    private const string SolutionTopic = "/mtc/tile_floor/solution";
    private const string ParameterService = "/mtc_test_task_node/get_parameters";
    private const int RequiredTaskVersion = 2;
    private const float ExecutionAcknowledgementSeconds = 15f;

    private ROSConnection _connection;
    private UR16eTrajectoryVisualizer _visualizer;
    private FixtureTilingPreview _review;
    private RobotTrajectoryMsg _trajectory;
    private RosRobotModel _planModel;
    private long _commandRevision;
    private string _requestId, _planId, _planStatus;
    private bool _checkingContract, _waitingForPreview, _ready, _executing, _awaitingAcknowledgement;
    private float _deadline;
    private readonly Dictionary<string, PlanningSceneMsg> _scenes = new Dictionary<string, PlanningSceneMsg>();
    private readonly Dictionary<string, SolutionMsg> _solutions = new Dictionary<string, SolutionMsg>();

    public bool CanPlan => CanEdit && IsSourceCurrent && !IsCapturingPoint && HasAnchor && ToolSelectionController.CanPlanPickPlace;
    public bool CanExecute => _ready && CanPlan && PlanContextIsCurrent() && _review?.HasReview == true &&
        _visualizer != null && _visualizer.IsShowingTrajectory(_trajectory);
    public bool IsPlanning => _checkingContract || _waitingForPreview;
    public bool HasExecutionFeedback { get; private set; }
    public int ReviewStepCount => _review != null ? _review.StepCount : 0;
    public int ReviewStepIndex => _review != null ? _review.StepIndex : 0;
    public string ReviewStepText => _review != null ? _review.StepText : string.Empty;

    public void Plan()
    {
        if (!CanPlan || !TryGetSource(out Pose source, out Vector3[] corners) || !TryGetAnchorWorldPose(out Pose floor)) return;
        MTCPlansManager.DismissCompletedExecutionFeedback();
        RobotExecutionState.NotifyPlanningStarted();
        int budget = MtcPlanningSettings.TimeoutSeconds;
        string requestId = Guid.NewGuid().ToString("N");
        InvalidatePlan();
        if (!FloorTilingRequest.TryBuild(requestId, budget, source, corners, floor, Pattern, out TileFloorTaskMsg request))
        {
            SetPlanStatus("Check that the source tile and destination are level with the robot base, then plan again.");
            return;
        }

        StopScan();
        try
        {
            BindTransport();
            if (!Connected)
            {
                SetPlanStatus("Reconnect ROS before planning the floor batch.");
                return;
            }

            _requestId = requestId;
            _planModel = ToolSelectionController.Instance?.CurrentModel;
            _commandRevision = RobotExecutionState.CommandRevision;
            _checkingContract = true;
            _deadline = Time.realtimeSinceStartup + ExecutionAcknowledgementSeconds;
            SetPlanStatus("Checking that ROS supports direct repetitive placement.");
            RequestTaskVersion(version => OnTaskVersion(request, version));
        }
        catch (Exception error)
        {
            InvalidatePlan();
            SetPlanStatus("Floor planning request failed: " + error.Message);
        }
    }

    private void OnTaskVersion(TileFloorTaskMsg request, long version)
    {
        if (this == null || !_checkingContract || _requestId != request.request_id) return;
        if (!PlanContextIsCurrent()) { RejectPlan("The job, tool, alignment or ROS connection changed. Plan again."); return; }
        if (version != RequiredTaskVersion)
        {
            RejectPlan("Update ROS to the direct repetitive placement backend, then plan again.");
            return;
        }
        _checkingContract = false;
        _waitingForPreview = true;
        _deadline = Time.realtimeSinceStartup + request.timeout_seconds + MtcPlanningStop.ResultGraceSeconds;
        try
        {
            PublishPlan(request);
            SetPlanStatus("Planning direct pickup and placement for the full batch.");
        }
        catch (Exception error) { RejectPlan("Batch planning request failed: " + error.Message); }
    }

    public void Execute()
    {
        if (!CanExecute) return;
        // An unknown delivery outcome must never rearm a plan or release the common execution lock.
        _ready = false;
        _executing = _awaitingAcknowledgement = true;
        HasExecutionFeedback = true;
        _deadline = Time.realtimeSinceStartup + ExecutionAcknowledgementSeconds;
        RobotExecutionState.SetBusy(this, true);
        SetPlanStatus("Floor batch execution requested. Waiting for ROS acknowledgement.");
        try { PublishExecution(_planId); }
        catch (Exception)
        {
            SetPlanStatus("Floor execution delivery is unknown. Inspect ROS. Robot commands remain blocked.");
        }
    }

    public void StepReview(int delta)
    {
        if (_review?.HasReview != true || _executing) return;
        _review.Step(delta);
        if (_visualizer != null && _visualizer.HasPlayback)
        {
            _visualizer.Play();
            _visualizer.Seek(_visualizer.DurationSeconds > 0f ? _review.StepMotionTime / _visualizer.DurationSeconds : 0f);
        }
        Changed?.Invoke();
    }

    protected virtual bool Connected => _connection != null && _connection.HasConnectionThread && !_connection.HasConnectionError;

    protected virtual void BindTransport()
    {
        if (_connection != null) return;
        _connection = ROSService.Instance.GetROSConnection();
        _connection.RegisterRosService<GetParametersRequest, GetParametersResponse>(ParameterService);
        _connection.RegisterPublisher<StringMsg>(ExecuteTopic);
        _connection.Subscribe<StringMsg>(StatusTopic, OnStatus);
        _connection.Subscribe<PlanningSceneMsg>(SceneTopic, OnScene);
        _connection.Subscribe<SolutionMsg>(SolutionTopic, OnSolution);
        // The standalone trajectory has no plan ID. The preview uses the identified aggregate solution.
    }

    protected virtual void RequestTaskVersion(Action<long> completed) =>
        _connection.SendServiceMessage<GetParametersResponse>(ParameterService,
            new GetParametersRequest(new[] { "tile_floor_task_version" }), response =>
            {
                var value = response?.values != null && response.values.Length == 1 ? response.values[0] : null;
                completed(value != null && value.type == 2 ? value.integer_value : 0);
            });

    protected virtual void PublishPlan(TileFloorTaskMsg request)
    {
        if (_connection.GetTopic(PlanTopic)?.IsPublisher != true)
            _connection.RegisterPublisher<TileFloorTaskMsg>(PlanTopic);
        _connection.Publish(PlanTopic, request);
    }
    protected virtual void PublishExecution(string planId) => _connection.Publish(ExecuteTopic, new StringMsg(planId));

    private void UnbindTransport()
    {
        if (_connection == null) return;
        _connection.Unsubscribe(StatusTopic);
        _connection.Unsubscribe(SceneTopic);
        _connection.Unsubscribe(SolutionTopic);
        _connection = null;
    }

    private void Update()
    {
        if (_executing)
        {
            if (_awaitingAcknowledgement && Time.realtimeSinceStartup > _deadline)
            {
                _awaitingAcknowledgement = false;
                SetPlanStatus("Floor execution acknowledgement timed out. Inspect ROS. Robot commands remain blocked.");
            }
            return;
        }
        if (!IsPlanning && !_ready) return;
        if (!PlanContextIsCurrent())
        {
            RejectPlan("Floor plan expired because the job, tool, alignment or ROS connection changed.");
            return;
        }
        if (IsPlanning && Time.realtimeSinceStartup > _deadline)
        {
            RejectPlan(_checkingContract ? "ROS did not confirm support for direct repetitive placement."
                : "Batch planning or preview timed out. Check ROS and plan again.");
            return;
        }
        if (_ready && (_visualizer == null || !_visualizer.IsShowingTrajectory(_trajectory) || _review?.HasReview != true))
        {
            RejectPlan("The reviewed floor path is no longer displayed. Plan again.");
            return;
        }
        if (_waitingForPreview && _planId != null) ShowPreviewIfComplete();
    }

    private bool PlanContextIsCurrent() => _taskActive && HasAnchor && IsSourceCurrent && !IsCapturingPoint &&
        ToolSelectionController.CanPlanPickPlace && ReferenceEquals(_planModel, ToolSelectionController.Instance?.CurrentModel) &&
        _commandRevision == RobotExecutionState.CommandRevision && Connected;

    [Serializable]
    private sealed class FloorStatus
    {
        public string status, reason, request_id, plan_id;
        public int tile_index, completed_tiles;
    }

    private void OnStatus(StringMsg message)
    {
        if (message == null || string.IsNullOrEmpty(_requestId)) return;
        FloorStatus status;
        try { status = JsonUtility.FromJson<FloorStatus>(message.data); }
        catch (ArgumentException) { return; }
        if (status == null) return;

        if (_executing)
        {
            bool admissionRejection = status.status == "rejected" && string.IsNullOrEmpty(status.request_id);
            if (status.plan_id != _planId || (status.request_id != _requestId && !admissionRejection)) return;
            if (status.status == "executing")
            {
                _awaitingAcknowledgement = false;
                SetPlanStatus("Floor batch executing · " + status.completed_tiles + "/" + TileCount + " tiles placed. " + status.reason);
            }
            else if (IsTerminalFailure(status.status) || status.status == "succeeded")
            {
                if (!admissionRejection)
                {
                    _requiresRecapture = true;
                    _surfaceCapture.SetItemPresentationEnabled(false);
                }
                _executing = _awaitingAcknowledgement = false;
                _requestId = _planId = null;
                RobotExecutionState.SetBusy(this, false);
                SetPlanStatus("Floor execution " + status.status + " · " + status.completed_tiles + "/" + TileCount +
                    " tiles placed. " + status.reason + (_requiresRecapture ? " Capture the top tile before the next batch." : ""));
            }
            return;
        }

        if (status.request_id != _requestId || (!_waitingForPreview && !_ready)) return;
        if (IsTerminalFailure(status.status))
        {
            RejectPlan("Floor planning " + status.status + ": " + status.reason);
            return;
        }
        if (status.status == "ready")
        {
            if (!FixtureTilingGeometry.ValidToken(status.plan_id) || (_planId != null && _planId != status.plan_id))
            {
                RejectPlan("ROS returned an invalid or conflicting floor plan ID. Plan again.");
                return;
            }
            _planId = status.plan_id;
            ShowPreviewIfComplete();
        }
        else if (_waitingForPreview && _planId == null && status.status == "planning")
            SetPlanStatus("Planning floor tile " + (status.tile_index + 1) + "/" + TileCount + ". " + status.reason);
    }

    private static bool IsTerminalFailure(string status) => status == "failed" || status == "rejected" ||
        status == "invalidated" || status == "timed_out" || status == "cancelled";

    private void OnScene(PlanningSceneMsg scene)
    {
        if (!_waitingForPreview || scene == null || !AcceptArtifact(scene.name)) return;
        if (_scenes.Count >= 4) _scenes.Clear();
        _scenes[scene.name] = scene;
        ShowPreviewIfComplete();
    }

    private void OnSolution(SolutionMsg solution)
    {
        if (!_waitingForPreview || solution == null || !AcceptArtifact(solution.task_id)) return;
        if (_solutions.Count >= 4) _solutions.Clear();
        _solutions[solution.task_id] = solution;
        ShowPreviewIfComplete();
    }

    private bool AcceptArtifact(string planId) => FixtureTilingGeometry.ValidToken(planId) &&
        (_planId == null || planId == _planId);

    private void ShowPreviewIfComplete()
    {
        if (!_waitingForPreview || _planId == null) return;
        if (!PlanContextIsCurrent()) { RejectPlan("Floor measurements changed. Plan again."); return; }
        if (!_scenes.TryGetValue(_planId, out PlanningSceneMsg scene) || !_solutions.TryGetValue(_planId, out SolutionMsg solution))
        {
            const string waiting = "Waiting for the matching floor scene and trajectory.";
            if (_planStatus != waiting) SetPlanStatus(waiting);
            return;
        }
        if (_visualizer == null) _visualizer = FindFirstObjectByType<UR16eTrajectoryVisualizer>();
        if (_visualizer == null || !_visualizer.IsReadyForPreview)
        {
            const string waiting = "Waiting for the robot model to display the floor batch.";
            if (_planStatus != waiting) SetPlanStatus(waiting);
            return;
        }

        try
        {
            _trajectory = FixtureSolutionPreview.Flatten(solution);
            _visualizer.SetTrajectory(_trajectory);
            if (!_visualizer.IsShowingTrajectory(_trajectory)) throw new ArgumentException("The robot path could not be displayed.");
            if (_review == null)
            {
                GameObject review = new GameObject("Floor batch review");
                review.transform.SetParent(transform, false);
                _review = review.AddComponent<FixtureTilingPreview>();
            }
            _review.SetReview(scene, solution, _planModel);
            if (!_review.HasReview) throw new ArgumentException("The batch scene could not be displayed.");
            _scenes.Clear();
            _solutions.Clear();
            _waitingForPreview = false;
            _ready = true;
            _anchorPreview?.Hide();
            SetPlanStatus("Batch ready. Review the path and scene steps, then execute once.");
        }
        catch (Exception error) { RejectPlan("Floor preview rejected: " + error.Message); }
    }

    private void InvalidatePlan()
    {
        _checkingContract = _waitingForPreview = _ready = false;
        _scenes.Clear();
        _solutions.Clear();
        _visualizer?.ClearTrajectoryIfMatches(_trajectory);
        _trajectory = null;
        _review?.ClearReview();
        if (_executing) return;
        _requestId = _planId = _planStatus = null;
        HasExecutionFeedback = false;
    }

    private void RejectPlan(string reason)
    {
        InvalidatePlan();
        RefreshAnchorPreview();
        SetPlanStatus(reason);
    }

    private void SetPlanStatus(string value)
    {
        _planStatus = value;
        Changed?.Invoke();
    }
}
