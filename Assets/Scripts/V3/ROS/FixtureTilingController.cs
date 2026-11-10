using System;
using System.Collections.Generic;
using System.IO;
using RosMessageTypes.Geometry;
using RosMessageTypes.Moveit;
using RosMessageTypes.MoveitTaskConstructor;
using RosMessageTypes.RclInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Ur16eInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public enum FixtureCaptureTarget
{
    SourceTop, PlacementCorner, PlacementX, PlacementY, ReleaseCenter,
    Board00, Board10, Board11, Board01, SupportCorner, SupportX, SupportY
}

/// <summary>Calibrated square-tile jobs. Only a complete, identified, displayed batch can execute once.</summary>
public class FixtureTilingController : MonoBehaviour
{
    public const string PlanTopic = "/fixture_tiling_plan_target";
    public const string ExecuteTopic = "/execute_fixture_plan";
    public const string CalibrationService = "/save_fixture_calibration";
    private const string ParameterService = "/mtc_test_task_node/get_parameters";
    private static readonly string[] CalibrationParameters = {
        "fixture.enabled", "fixture.frame_id", "fixture.board_top_corners", "fixture.board_thickness",
        "fixture.support_corner_xyz", "fixture.support_corner_xyzw", "fixture.margin", "fixture.planning_timeout"
    };
    [SerializeField] private FixtureTilingSettings _settings = new FixtureTilingSettings();
    [SerializeField] private FixtureCalibrationData _calibration = new FixtureCalibrationData();
    [SerializeField] private UR16eTrajectoryVisualizer _visualizer;
    private FixtureTilingPreview _preview;
    private ROSConnection _connection;
    private RobotCoordinateService _coordinates;
    private MtcPlanningStop _planningStop;
    private MtcPlanningStop PlanningStop => _planningStop ??=
        new MtcPlanningStop("fixture", RefreshPlanningStopFeedback, OnPlanningStopFinished);
    private bool _active, _checkingCalibration, _acceptingResults, _ready, _executionRequested, _awaitingAcknowledgement;
    private bool _savingCalibration, _receivedPlanningTerminal;
    private bool _hasStarted;
    private float _deadline, _saveDeadline;
    private int _generation, _saveGeneration, _calibrationRevision;
    private string _requestId, _planId, _snapshot, _saveSnapshot;
    private RosRobotModel _requestModel;
    private FixtureTilingTaskMsg _request;
    private RobotTrajectoryMsg _trajectory;
    private readonly Dictionary<string, PlanningSceneMsg> _scenes = new Dictionary<string, PlanningSceneMsg>();
    private readonly Dictionary<string, SolutionMsg> _solutions = new Dictionary<string, SolutionMsg>();
    private FixtureCaptureTarget? _capture;
    private PointMsg _supportOrigin, _supportX, _supportY, _placementOrigin, _placementX, _placementY;
    public FixtureTilingSettings Settings => _settings;
    public FixtureCalibrationData Calibration => _calibration;
    public bool IsActive => _active;
    public bool IsPlanning => _checkingCalibration || _acceptingResults || _planningStop?.IsActive == true;
    public bool CanStopPlanning => !_executionRequested && (_checkingCalibration || _planningStop?.CanRequestStop == true ||
        (_receivedPlanningTerminal && _acceptingResults));
    public bool IsStoppingPlanning => _planningStop?.StopRequested == true;
    public bool IsExecuting => _executionRequested;
    public bool HasExecutionFeedback { get; private set; }
    public bool IsCapturing => _capture.HasValue;
    public string CaptureInstruction => _capture.HasValue ? "Aim at " + CaptureName(_capture.Value) + "; press trigger to capture." : "Select a point to capture, then aim and press trigger.";
    public string StatusText { get; private set; } = "Set up a square-tile stack, fixture and placement pattern.";
    public string CalibrationStatus { get; private set; } = "Capture four plank corners and the alignment corner/axes, then Save to ROS.";
    public int ReviewStepCount => _preview != null ? _preview.StepCount : 0;
    public int ReviewStepIndex => _preview != null ? _preview.StepIndex : 0;
    public string ReviewStepText => _preview != null ? _preview.StepText : "No batch scene preview.";
    public event Action Changed;
    public bool CanPlan => !IsPlanning && !_executionRequested && ContextValid() &&
        FixtureTilingGeometry.TryBuildRequest(_settings, "validation", out _, out _) &&
        FixtureTilingGeometry.TryBuildCalibration(_calibration, out _, out _);
    public bool CanExecute => _ready && !IsPlanning && !_executionRequested && ContextValid() && SnapshotMatches() &&
        _visualizer != null && _visualizer.IsShowingTrajectory(_trajectory) && _preview != null && _preview.HasReview;

    private void Start()
    {
        _coordinates = RobotCoordinateService.Instance;
        _coordinates.CalibrationChanged += OnRobotCalibrationChanged;
        if (_visualizer == null) _visualizer = FindFirstObjectByType<UR16eTrajectoryVisualizer>();
        _preview = GetComponent<FixtureTilingPreview>();
        if (_preview == null) _preview = gameObject.AddComponent<FixtureTilingPreview>();
        LoadProfile();
        _hasStarted = true;
        if (_active) EnsureTransport();
        RefreshSetupPreview();
    }

    private void EnsureTransport()
    {
        if (_connection != null) return;
        _connection = ROSService.Instance.GetROSConnection();
        PlanningStop.Bind(_connection);
        _connection.RegisterPublisher<FixtureTilingTaskMsg>(PlanTopic);
        _connection.RegisterPublisher<StringMsg>(ExecuteTopic);
        _connection.RegisterRosService<SaveFixtureCalibrationRequest, SaveFixtureCalibrationResponse>(CalibrationService);
        _connection.RegisterRosService<GetParametersRequest, GetParametersResponse>(ParameterService);
        _connection.Subscribe<StringMsg>("/mtc/fixture/status", OnStatus);
        _connection.Subscribe<PlanningSceneMsg>("/mtc/fixture/preview_scene", OnScene);
        _connection.Subscribe<SolutionMsg>("/mtc/fixture/solution", OnSolution);
        // The separate flattened trajectory has no plan ID. Build our robot path
        // from the matching aggregate solution instead of correlating by arrival time.
    }

    public void SetTaskActive(bool active)
    {
        if (_active == active) return;
        _active = active;
        if (_active && _hasStarted) EnsureTransport();
        CancelCapture();
        InvalidateReview(active ? "Fixture tiling selected. Configure the stack, release and pattern." : null);
        RefreshSetupPreview();
    }

    public void NotifyEdited()
    {
        InvalidateReview("Tile job or fixture changed. Plan the full batch again.");
        SaveProfile();
        RefreshSetupPreview();
    }

    public void RotateSourceYaw(float degrees)
    {
        if (_executionRequested || _settings.SourceTopPose == null || !float.IsFinite(degrees)) return;
        QuaternionMsg q = _settings.SourceTopPose.pose.orientation;
        double yaw = 2 * Math.Atan2(q.z, q.w) + degrees * Math.PI / 180;
        _settings.SourceTopPose.pose.orientation = new QuaternionMsg(0, 0, Math.Sin(yaw / 2), Math.Cos(yaw / 2));
        NotifyEdited();
    }

    /// <summary>Fit noisy XR measurements to the backend's exact rectangle, with an explicit residual check.</summary>
    public void FitCalibration()
    {
        if (!CaptureAllowed()) return;
        if (!FixtureTilingGeometry.TryFitBoard(_calibration.BoardTopCorners, .01f,
            out PointMsg[] fitted, out float residual, out string reason))
        { CalibrationStatus = reason; Changed?.Invoke(); return; }
        _calibration.BoardTopCorners = fitted;
        try
        {
            if (_supportOrigin != null && _supportX != null && _supportY != null)
                _calibration.SupportCorner = FixtureTilingGeometry.ProjectFrameToBoard(_calibration, _supportOrigin, _supportX, _supportY);
            else if (_calibration.SupportCorner != null)
                _calibration.SupportCorner = FixtureTilingGeometry.ProjectFrameToBoard(_calibration,
                    _calibration.SupportCorner.pose.position,
                    FixtureTilingGeometry.PointInFrame(_calibration.SupportCorner, .1, 0, 0),
                    FixtureTilingGeometry.PointInFrame(_calibration.SupportCorner, 0, .1, 0));
            CalibrationStatus = "Plank fitted; maximum corner adjustment " + (residual*1000).ToString("F1") +
                " mm. Inspect the ghost plank before saving. Alignment axes are projected onto its surface.";
            NotifyEdited();
        }
        catch (ArgumentException error) { _calibration.SupportCorner = null; CalibrationStatus = error.Message; NotifyEdited(); }
    }

    public void BeginCapture(FixtureCaptureTarget target)
    {
        if (!CaptureAllowed()) { SetStatus("Finish robot alignment and any active execution before capturing fixture points."); return; }
        InvalidateReview(null);
        TargetPoseContextMenu.CloseAll();
        TargetRotationHandles.CancelAllInteractions();
        _capture = target;
        Changed?.Invoke();
    }

    public void CancelCapture() { _capture = null; Changed?.Invoke(); }

    public bool CapturePointerPose(Pose worldPointer, Transform trackingSpace)
    {
        if (!_capture.HasValue || !CaptureAllowed() || !RobotFrameMath.IsRigidFrame(trackingSpace) ||
            !RobotCoordinateService.Instance.TryConvertWorldPoseToLocalPose(worldPointer, out Pose local)) return false;
        PointMsg point = local.position.To<FLU>();
        FixtureCaptureTarget target = _capture.Value;
        try
        {
            switch (target)
            {
                case FixtureCaptureTarget.SourceTop:
                    QuaternionMsg yaw = _settings.SourceTopPose?.pose.orientation ?? new QuaternionMsg(0, 0, 0, 1);
                    _settings.SourceTopPose = Stamped(point, yaw); break;
                case FixtureCaptureTarget.PlacementCorner:
                    _placementOrigin = point; _placementX = _placementY = null; _settings.PlacementCorner = null; break;
                case FixtureCaptureTarget.PlacementX: _placementX = point; TryPlacementFrame(); break;
                case FixtureCaptureTarget.PlacementY: _placementY = point; TryPlacementFrame(); break;
                case FixtureCaptureTarget.SupportCorner:
                    _supportOrigin = point; _supportX = _supportY = null; _calibration.SupportCorner = null; break;
                case FixtureCaptureTarget.SupportX: _supportX = point; TrySupportFrame(); break;
                case FixtureCaptureTarget.SupportY: _supportY = point; TrySupportFrame(); break;
                case FixtureCaptureTarget.ReleaseCenter:
                    if (_calibration.SupportCorner == null) throw new ArgumentException("Capture the alignment corner and both axes first.");
                    PoseMsg support = _calibration.SupportCorner.pose;
                    Quaternion rotation = new Quaternion((float)support.orientation.x, (float)support.orientation.y,
                        (float)support.orientation.z, (float)support.orientation.w);
                    Vector3 relative = Quaternion.Inverse(rotation) * new Vector3((float)(point.x-support.position.x),
                        (float)(point.y-support.position.y), (float)(point.z-support.position.z));
                    _settings.ReleaseCenterX = relative.x; _settings.ReleaseCenterY = relative.y; break;
                default:
                    _calibration.BoardTopCorners[(int)target - (int)FixtureCaptureTarget.Board00] = point; break;
            }
            _capture = null;
            NotifyEdited();
            string captureStatus = "Captured " + CaptureName(target) + ". " + NextCaptureHint(target);
            if (target >= FixtureCaptureTarget.Board00) CalibrationStatus = captureStatus;
            SetStatus(captureStatus);
            return true;
        }
        catch (ArgumentException error)
        {
            NotifyEdited();
            if (target >= FixtureCaptureTarget.Board00) CalibrationStatus = error.Message;
            SetStatus(error.Message); return false;
        }
    }

    private void TryPlacementFrame()
    {
        _settings.PlacementCorner = null;
        if (_placementOrigin == null) throw new ArgumentException("Capture the placement corner first.");
        if (_placementX != null && _placementY != null)
            _settings.PlacementCorner = FixtureTilingGeometry.FrameFromPoints(_placementOrigin, _placementX, _placementY);
    }
    private void TrySupportFrame()
    {
        _calibration.SupportCorner = null;
        if (_supportOrigin == null) throw new ArgumentException("Capture the alignment corner first.");
        if (_supportX != null && _supportY != null)
            _calibration.SupportCorner = FixtureTilingGeometry.ProjectFrameToBoard(_calibration, _supportOrigin, _supportX, _supportY);
    }

    public void SaveCalibration()
    {
        if (_savingCalibration || !CaptureAllowed()) return;
        if (!FixtureTilingGeometry.TryBuildCalibration(_calibration, out SaveFixtureCalibrationRequest request, out string reason))
        { CalibrationStatus = reason; Changed?.Invoke(); return; }
        try
        {
            EnsureTransport();
            if (!Connected) { CalibrationStatus = "Reconnect ROS before saving fixture calibration."; Changed?.Invoke(); return; }
            _savingCalibration = true; _saveDeadline = Time.realtimeSinceStartup + 15;
            _saveSnapshot = JsonUtility.ToJson(_calibration);
            int generation = ++_saveGeneration;
            CalibrationStatus = "Saving measured calibration to ROS for its next startup...";
            Changed?.Invoke();
            _connection.SendServiceMessage<SaveFixtureCalibrationResponse>(CalibrationService, request, response => {
                if (this == null || generation != _saveGeneration || !_savingCalibration) return;
                _savingCalibration = false;
                CalibrationStatus = response != null && response.success
                    ? "Saved to ROS. Restart/reload MTC to activate it; an existing plank may require planning-scene reset."
                    : "ROS did not save calibration: " + (response?.message ?? "empty response");
                if (_saveSnapshot != JsonUtility.ToJson(_calibration)) CalibrationStatus += " Measurements changed since this save; save again.";
                Changed?.Invoke();
            });
        }
        catch (Exception error) { _savingCalibration = false; CalibrationStatus = "Calibration save failed: " + error.Message; Changed?.Invoke(); }
    }

    public void Plan()
    {
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null || IsPlanning) return;
        RobotExecutionState.NotifyPlanningStarted();
        if (!ContextValid()) { SetStatus("Select fixture tiling, finish alignment/capture and match the ROS suction-tool model."); return; }
        if (!FixtureTilingGeometry.TryBuildCalibration(_calibration, out _, out string reason)) { SetStatus(reason); return; }
        string requestId = Guid.NewGuid().ToString("N");
        if (!FixtureTilingGeometry.TryBuildRequest(_settings, requestId, out FixtureTilingTaskMsg request, out reason))
        { SetStatus(reason); return; }
        InvalidateReview(null);
        _requestId = requestId; _request = request;
        _snapshot = CurrentSnapshot(); _calibrationRevision = RobotCoordinateService.Instance.CalibrationRevision;
        _requestModel = ToolSelectionController.Instance?.CurrentModel;
        try
        {
            EnsureTransport();
            if (!Connected) { InvalidateReview("Reconnect ROS before planning fixture tiling."); return; }
            _checkingCalibration = true; _deadline = Time.realtimeSinceStartup + 15;
            int generation = _generation;
            SetStatus("Checking the running ROS fixture calibration...");
            MtcPlanningStop.RequestCapabilities(_connection, ParameterService, CalibrationParameters,
                () => this != null && generation == _generation && _checkingCalibration,
                response => OnCalibrationParameters(response, generation));
        }
        catch (Exception error) { InvalidateReview("Fixture planning request failed: " + error.Message); }
    }

    private void OnCalibrationParameters(GetParametersResponse response, int generation)
    {
        if (this == null || generation != _generation || !_checkingCalibration) return;
        if (!ContextValid() || !SnapshotMatches()) { InvalidateReview("Job, tool or alignment changed. Plan again."); return; }
        if (!RunningCalibrationMatches(response, out string reason)) { InvalidateReview(reason); return; }
        _checkingCalibration = false; _acceptingResults = true;
        double timeout = response.values[7].double_value;
        _deadline = Time.realtimeSinceStartup + (float)(Math.Max(60, timeout) * _settings.ItemCount + 30);
        try
        {
            if (MtcPlanningStop.IsSupported(response.values, 8))
                PlanningStop.Begin(_connection, _requestId);
            if (!TryPublishPlan(_request, out reason))
            {
                _planningStop?.ForgetUnsent();
                InvalidateReview(reason);
                return;
            }
        }
        catch (Exception error)
        {
            _planningStop?.ForgetUnsent();
            InvalidateReview("Fixture planning request failed: " + error.Message);
            return;
        }
        SetStatus("Planning every tile cycle. No execution requested.");
    }

    public void StopPlanning()
    {
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null || !CanStopPlanning) return;
        if (_checkingCalibration)
        {
            InvalidateReview("Fixture planning stopped. Plan again.");
            return;
        }
        if (_receivedPlanningTerminal)
        {
            InvalidateReview("Fixture preview discarded. Plan again.");
            return;
        }
        _planningStop.RequestStop();
    }

    private void RefreshPlanningStopFeedback()
    {
        string feedback = _planningStop?.Feedback;
        if (IsStoppingPlanning && (_checkingCalibration || _acceptingResults || _ready)) InvalidateReview(feedback);
        else if (feedback != null) SetStatus(feedback);
        else Changed?.Invoke();
    }

    private void OnPlanningStopFinished(string status, string reason, bool stopRequested)
    {
        if (status == "cancelled") InvalidateReview("Fixture planning stopped. Plan again.");
        else if (status == "rejected") InvalidateReview("ROS rejected fixture planning: " + reason);
        else if (stopRequested) InvalidateReview("Fixture preview discarded. Plan again.");
        else _receivedPlanningTerminal = true;
    }

    private bool RunningCalibrationMatches(GetParametersResponse response, out string reason)
    {
        reason = "Running ROS fixture calibration differs or is disabled. Save the measured calibration, restart/reload MTC and plan again.";
        ParameterValueMsg[] v = response?.values;
        if (v == null || v.Length < 8 || v[0].type != 1 || !v[0].bool_value || v[1].type != 4 ||
            v[1].string_value != "ur_base_link" || v[2].type != 8 || v[3].type != 3 ||
            v[4].type != 8 || v[5].type != 8 || v[6].type != 3 || v[7].type != 3 ||
            !double.IsFinite(v[3].double_value) || !double.IsFinite(v[6].double_value) ||
            !double.IsFinite(v[7].double_value) || v[7].double_value <= 0 || v[7].double_value > 600) return false;
        var corners = new List<double>();
        foreach (PointMsg p in _calibration.BoardTopCorners) { corners.Add(p.x); corners.Add(p.y); corners.Add(p.z); }
        PoseMsg support = _calibration.SupportCorner.pose;
        if (!Near(v[2].double_array_value, corners.ToArray()) || Math.Abs(v[3].double_value-_calibration.BoardThickness) > 1e-6 ||
            !Near(v[4].double_array_value, new[] {support.position.x,support.position.y,support.position.z}) ||
            Math.Abs(v[6].double_value-_calibration.Margin) > 1e-6) return false;
        double[] q = {support.orientation.x,support.orientation.y,support.orientation.z,support.orientation.w};
        double[] opposite = Array.ConvertAll(q, value => -value);
        return Near(v[5].double_array_value,q) || Near(v[5].double_array_value,opposite);
    }
    private static bool Near(double[] a, double[] b)
    {
        if (a == null || a.Length != b.Length) return false;
        for (int i = 0; i < a.Length; i++) if (!double.IsFinite(a[i]) || Math.Abs(a[i]-b[i]) > 1e-6) return false;
        return true;
    }

    protected virtual bool TryPublishPlan(FixtureTilingTaskMsg request, out string reason)
    { _connection.Publish(PlanTopic, request); reason = null; return true; }
    protected virtual bool TryPublishExecution(string planId, out string reason)
    { _connection.Publish(ExecuteTopic, new StringMsg(planId)); reason = null; return true; }

    public void Execute()
    {
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return;
        if (!CanExecute || !Connected) { SetStatus("Execute requires the current complete batch and its visible preview. Plan again if inputs changed."); return; }
        // Consume before publication. Missing acknowledgement is an unknown outcome,
        // never permission to resend or create a competing robot command.
        _ready = false; _acceptingResults = false; _executionRequested = _awaitingAcknowledgement = true;
        HasExecutionFeedback = true; _deadline = Time.realtimeSinceStartup + 15;
        RobotExecutionState.SetBusy(this, true);
        SetStatus("Fixture execution requested. Waiting for ROS acknowledgement...");
        try
        {
            if (!TryPublishExecution(_planId, out string reason)) FinishExecution("rejected", reason);
        }
        catch (Exception) { SetStatus("Execution publication outcome is unknown. Inspect ROS; this plan will not be resent."); }
    }

    private void OnStatus(StringMsg message)
    {
        FixtureStatus status;
        try { status = JsonUtility.FromJson<FixtureStatus>(message.data); }
        catch (Exception) { return; }
        if (status == null || string.IsNullOrEmpty(_requestId)) return;
        if (_executionRequested)
        {
            // ROS cannot recover a request ID after another task has replaced the
            // prepared slot. Its admission rejection still identifies the exact
            // plan ID we sent; all other outcomes require the request ID too.
            bool admissionRejection = status.status == "rejected" && string.IsNullOrEmpty(status.request_id);
            if (status.plan_id != _planId || (status.request_id != _requestId && !admissionRejection)) return;
            if (status.status == "executing")
            { _awaitingAcknowledgement = false; SetStatus("Executing tile " + (status.tile_index+1) + ": " + status.reason); }
            else if (status.status == "succeeded" || status.status == "failed" || status.status == "rejected" || status.status == "invalidated")
                FinishExecution(status.status, status.reason);
            return;
        }
        if (status.request_id != _requestId) return;
        if (!_acceptingResults && !_ready) return;
        if (status.status == "failed" || status.status == "rejected" || status.status == "invalidated")
        { InvalidateReview("Fixture batch " + status.status + ": " + status.reason); return; }
        if (status.status == "ready" && !string.IsNullOrEmpty(status.plan_id))
        {
            if (_planId != null && _planId != status.plan_id) { InvalidateReview("Conflicting fixture plan IDs. Plan again."); return; }
            _planId = status.plan_id;
            ShowPreviewIfComplete();
        }
        else if (status.status == "planning" && _planId == null)
            SetStatus("Planning tile " + (status.tile_index+1) + ": " + status.reason);
    }

    private void OnScene(PlanningSceneMsg scene)
    {
        if (!_acceptingResults || scene == null || string.IsNullOrEmpty(scene.name)) return;
        if (_scenes.Count >= 4) _scenes.Clear();
        _scenes[scene.name] = scene; ShowPreviewIfComplete();
    }
    private void OnSolution(SolutionMsg solution)
    {
        if (!_acceptingResults || solution == null || string.IsNullOrEmpty(solution.task_id)) return;
        if (_solutions.Count >= 4) _solutions.Clear();
        _solutions[solution.task_id] = solution; ShowPreviewIfComplete();
    }
    private void ShowPreviewIfComplete()
    {
        if (_planId == null || !_acceptingResults) return;
        if (!_scenes.TryGetValue(_planId, out PlanningSceneMsg scene) || !_solutions.TryGetValue(_planId, out SolutionMsg solution))
        { SetStatus("ROS batch ready. Waiting for its matching solution and scene..."); return; }
        if (!ContextValid() || !SnapshotMatches()) { InvalidateReview("Job, tool or alignment changed. Plan again."); return; }
        if (_visualizer == null || !_visualizer.IsReadyForPreview) { SetStatus("Waiting for the robot model to display the complete batch..."); return; }
        try
        {
            _trajectory = FixtureSolutionPreview.Flatten(solution);
            _visualizer.SetTrajectory(_trajectory);
            if (!_visualizer.IsShowingTrajectory(_trajectory)) throw new ArgumentException("Robot path could not be displayed.");
            if (_preview == null) _preview = gameObject.AddComponent<FixtureTilingPreview>();
            _preview.SetReview(scene, solution, _settings, _requestModel);
            if (!_preview.HasReview) throw new ArgumentException("Batch scene effects could not be displayed.");
            _acceptingResults = false; _ready = true;
            SetStatus("Batch ready. Review the robot path and tile/settling steps, then Execute once.");
        }
        catch (Exception error) { InvalidateReview("Batch preview rejected: " + error.Message); }
    }
    public void StepReview(int delta)
    {
        if (_preview != null)
        {
            _preview.Step(delta);
            if (_visualizer != null && _visualizer.HasPlayback)
            {
                _visualizer.Play();
                _visualizer.Seek(_visualizer.DurationSeconds > 0 ? _preview.StepMotionTime/_visualizer.DurationSeconds : 0);
            }
        }
        Changed?.Invoke();
    }

    private void FinishExecution(string status, string reason)
    {
        _executionRequested = _awaitingAcknowledgement = false;
        RobotExecutionState.SetBusy(this, false);
        _ready = _acceptingResults = false;
        SetStatus("Fixture execution " + status + ": " + reason + " Inspect inventory and scene before starting another job.");
    }
    private void Update()
    {
        _planningStop?.Tick();
        if (_savingCalibration && Time.realtimeSinceStartup > _saveDeadline)
        { _savingCalibration = false; ++_saveGeneration; CalibrationStatus = "Calibration save timed out; inspect ROS before saving again."; Changed?.Invoke(); }
        if (_executionRequested)
        {
            if (_awaitingAcknowledgement && Time.realtimeSinceStartup > _deadline)
            { _awaitingAcknowledgement = false; SetStatus("Fixture execution acknowledgement timed out. Outcome unknown; inspect ROS. Other commands remain blocked."); }
            return;
        }
        if (IsStoppingPlanning)
        {
            if (StatusText != _planningStop.Feedback) RefreshPlanningStopFeedback();
            return;
        }
        if (_checkingCalibration || _acceptingResults || _ready)
        {
            if (!ContextValid() || !SnapshotMatches()) { InvalidateReview("Job, tool, connection or alignment changed. Plan again."); return; }
            if ((_checkingCalibration || _acceptingResults) && Time.realtimeSinceStartup > _deadline)
            { InvalidateReview("Fixture planning/preview timed out. Inspect ROS, then plan again."); return; }
            if (_ready && (_visualizer == null || !_visualizer.IsShowingTrajectory(_trajectory)))
            { InvalidateReview("The reviewed batch path is no longer displayed. Plan again."); return; }
            if (_acceptingResults && _planId != null) ShowPreviewIfComplete();
        }
    }

    protected virtual bool Connected => _connection != null && _connection.HasConnectionThread && !_connection.HasConnectionError;
    private bool CaptureAllowed() => _active && !RobotExecutionState.IsBusy && MTCPlansManager.ActiveExecution == null &&
        !RobotCoordinateService.IsCalibrationEditActive && RobotCoordinateService.Instance.IsInitialized &&
        RobotCoordinateService.Instance.RosBaseFrameId == "ur_base_link";
    private bool ContextValid() => CaptureAllowed() && !IsCapturing && !TargetRotationHandles.IsInteracting &&
        !TargetPoseContextMenu.IsOpen && ToolSelectionController.Instance != null &&
        ToolSelectionController.Instance.IsRosModelMatched && ToolSelectionController.Instance.SelectedTool != "none";
    private string CurrentSnapshot() => JsonUtility.ToJson(_settings) + "\n" + JsonUtility.ToJson(_calibration);
    private bool SnapshotMatches() => _snapshot == CurrentSnapshot() &&
        RobotCoordinateService.Instance.CalibrationRevision == _calibrationRevision &&
        ReferenceEquals(_requestModel, ToolSelectionController.Instance?.CurrentModel) && Connected;
    private void OnRobotCalibrationChanged() { CancelCapture(); InvalidateReview("Robot alignment changed. Review fixture measurements and plan again."); RefreshSetupPreview(); }
    private void InvalidateReview(string reason)
    {
        unchecked { ++_generation; }
        _checkingCalibration = _acceptingResults = _ready = false;
        _receivedPlanningTerminal = false;
        _scenes.Clear(); _solutions.Clear();
        _request = null;
        if (!_executionRequested) { _requestId = _planId = null; HasExecutionFeedback = false; }
        _visualizer?.ClearTrajectoryIfMatches(_trajectory);
        _trajectory = null;
        if (_preview != null) _preview.ClearReview();
        if (!_executionRequested && IsStoppingPlanning) SetStatus(_planningStop.Feedback);
        else if (!_executionRequested && reason != null) SetStatus(reason);
        else Changed?.Invoke();
    }
    private void RefreshSetupPreview()
    {
        if (_preview == null) _preview = GetComponent<FixtureTilingPreview>();
        if (_preview != null) _preview.ShowSetup(_settings, _calibration, _active);
    }
    private void SetStatus(string text) { StatusText = text; Changed?.Invoke(); }
    private static PoseStampedMsg Stamped(PointMsg position, QuaternionMsg rotation) => new PoseStampedMsg(
        new HeaderMsg { frame_id = "ur_base_link" }, new PoseMsg(position, rotation));
    private static string CaptureName(FixtureCaptureTarget target) => target.ToString().Replace("Board", "plank P").Replace("Support", "alignment ").Replace("Placement", "placement ");
    private static string NextCaptureHint(FixtureCaptureTarget target) =>
        target == FixtureCaptureTarget.PlacementCorner || target == FixtureCaptureTarget.SupportCorner ? "Capture +X and +Y along the edges into the receiving area." : "";
    [Serializable] private sealed class Profile { public FixtureTilingSettings settings; public FixtureCalibrationData calibration; }
    private string ProfilePath => Path.Combine(Application.persistentDataPath, "fixture-tiling.json");
    private void SaveProfile()
    {
        if (!Application.isPlaying) return;
        try { File.WriteAllText(ProfilePath, JsonUtility.ToJson(new Profile { settings = _settings, calibration = _calibration }, true)); }
        catch (Exception error) { Debug.LogWarning("Fixture setup could not be saved locally: " + error.Message, this); }
    }
    private void LoadProfile()
    {
        try
        {
            if (!File.Exists(ProfilePath)) return;
            Profile profile = JsonUtility.FromJson<Profile>(File.ReadAllText(ProfilePath));
            if (profile?.settings != null) _settings = profile.settings;
            if (profile?.calibration?.BoardTopCorners?.Length == 4) _calibration = profile.calibration;
        }
        catch (Exception error) { Debug.LogWarning("Fixture setup could not be loaded: " + error.Message, this); }
    }
    private void OnDisable() { CancelCapture(); InvalidateReview(null); }
    private void OnDestroy()
    {
        _planningStop?.Dispose();
        if (_coordinates != null) _coordinates.CalibrationChanged -= OnRobotCalibrationChanged;
        if (_connection != null) { _connection.Unsubscribe("/mtc/fixture/status"); _connection.Unsubscribe("/mtc/fixture/preview_scene"); _connection.Unsubscribe("/mtc/fixture/solution"); }
        RobotExecutionState.SetBusy(this, false);
    }
    [Serializable] private sealed class FixtureStatus { public string status, reason, request_id, plan_id; public int tile_index; }
}
