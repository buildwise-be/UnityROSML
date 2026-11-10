using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.RclInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Ur16eInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

/// <summary>Desired tool-contact poses for MTC pick/place. Plan never executes motion.</summary>
public class PickPlaceController : MonoBehaviour
{
    public const string ContactPlanTopic = "/pick_place_tcp_target";
    public const string TaskPlanTopic = "/pick_place_task_target";
    public const string IdentifiedExecuteTopic = "/execute_pick_place_plan";
    private const string CapabilityService = "/mtc_test_task_node/get_parameters";
    private const string CapabilityParameter = "pick_place_task_planning_enabled";
    // Retain old scene fields during migration; tool poses never use the old surface API or planes.
    [SerializeField, HideInInspector] private string _pickPlaceTopicName = ContactPlanTopic;
    [SerializeField, HideInInspector] private GameObject _pickPointPrefab;
    [SerializeField, HideInInspector] private GameObject _placePointPrefab;
    [SerializeField] private RemoteExecutionController _remoteExecutionController;
    [SerializeField] private MTCPlansManager _plansManager;
    [SerializeField, Min(1f)] private float _capabilityTimeout = 10f;
    private GameObject _pickPointInstance, _placePointInstance, _livePointInstance;
    private TargetRotationHandles _pickRotationHandles, _placeRotationHandles;
    private Material _pickHandleMaterial, _placeHandleMaterial;
    private bool _hasPlanningRequestBeenSent;
    private bool _tilePlanningEnabled = true;
    private bool _checkingCapability;
    private int _requestedTimeoutSeconds = MtcPlanningSettings.DefaultTimeoutSeconds;
    private int _generation, _requestCalibrationRevision;
    private float _capabilityDeadline;
    private PickPlacePosesMsg _requestedContactPoses;
    private PickPlaceTaskMsg _requestedTask;
    [SerializeField] private int _descentSpeedMmPerSecond = 5;
    [SerializeField] private bool _pickBySurface;
    [SerializeField] private PickItem _selectedItem = PickItem.Tile;
    private PickSurfaceCapture _surfaceCapture;
    private bool _applyingSurface;
    private RobotCoordinateService _coordinates;
    private ROSConnection _connection;
    public bool IsTilePlanningEnabled => _tilePlanningEnabled;
    public bool IsPlanning => _checkingCapability || (PlansManager != null && PlansManager.IsPlanning);
    public bool CanStopPlanning => _checkingCapability || (PlansManager != null && PlansManager.CanStopPlanning);
    public bool IsStoppingPlanning => PlansManager != null && PlansManager.IsStoppingPlanning;
    public int DescentSpeedMmPerSecond => _descentSpeedMmPerSecond;
    public bool PickBySurface => _pickBySurface;
    public PickItem SelectedItem => _selectedItem;
    public PickSurfaceCapture SurfaceCapture => _surfaceCapture != null ? _surfaceCapture : GetComponent<PickSurfaceCapture>();
    public bool HasValidSurface => SurfaceCapture != null && SurfaceCapture.HasSurface &&
        SurfaceCapture.SurfaceItem == _selectedItem && !SurfaceCapture.IsCapturing &&
        SurfaceCapture.TryGetWorldTopSurfacePose(out _);
    public event Action SettingsChanged;
    private MTCPlansManager PlansManager
    {
        get { if (_plansManager == null) _plansManager = FindFirstObjectByType<MTCPlansManager>(); return _plansManager; }
    }

    private void Start()
    {
        _coordinates = RobotCoordinateService.Instance;
        _coordinates.CalibrationChanged += InvalidatePlanningRequest;
        _connection = ROSService.Instance.GetROSConnection();
        _pickPlaceTopicName = TaskPlanTopic;
        _connection.RegisterPublisher<PickPlaceTaskMsg>(_pickPlaceTopicName);
        _connection.RegisterPublisher<BoolMsg>("execute_task");
        _connection.RegisterPublisher<StringMsg>(IdentifiedExecuteTopic);
        _connection.RegisterPublisher<StringMsg>("/repeat_reviewed_plan");
        _connection.RegisterRosService<GetParametersRequest, GetParametersResponse>(CapabilityService);
        if (_pickPointPrefab != null || _placePointPrefab != null)
            Debug.Log("PickPlaceController: Using contact balls and normal arrows instead of legacy plane markers.", this);
    }

    private void Update()
    {
        if (!_checkingCapability) return;
        if (!CanKeepCapabilityRequest())
        { RejectPlanningRequest("Tool targets, mode, model or alignment changed. Plan pick/place again."); return; }
        if (Time.realtimeSinceStartup > _capabilityDeadline)
            RejectPlanningRequest("ROS pick/place tool-pose planner did not answer. Update the MTC backend and reconnect.");
    }

    private void CancelCapabilityRequest()
    {
        unchecked { _generation++; }
        _checkingCapability = false;
        _requestedContactPoses = null;
        _requestedTask = null;
    }

    public bool SetDescentSpeedMmPerSecond(int speed)
    {
        if ((speed != 5 && speed != 10 && speed != 20) || RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return false;
        if (_descentSpeedMmPerSecond == speed) return true;
        _descentSpeedMmPerSecond = speed;
        InvalidatePlanningRequest();
        SettingsChanged?.Invoke();
        return true;
    }

    public bool SetPickBySurface(bool surface)
    {
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return false;
        if (_pickBySurface == surface) return true;
        SurfaceCapture?.Cancel();
        _pickBySurface = surface;
        RefreshSurfacePresentation();
        if (!surface && _pickPointInstance != null)
            SetContactPose(true, new Pose(_pickPointInstance.transform.position, _pickPointInstance.transform.rotation));
        else if (surface && HasValidSurface) OnSurfaceCaptured();
        InvalidatePlanningRequest();
        SettingsChanged?.Invoke();
        return true;
    }

    public bool SetSelectedItem(PickItem item)
    {
        if ((item != PickItem.Tile && item != PickItem.Brick) || RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return false;
        if (_selectedItem == item) return true;
        SurfaceCapture?.Cancel();
        _selectedItem = item;
        RefreshSurfacePresentation();
        InvalidatePlanningRequest();
        SettingsChanged?.Invoke();
        return true;
    }

    public PickSurfaceCapture GetOrCreateSurfaceCapture()
    {
        PickSurfaceCapture capture = SurfaceCapture;
        if (capture == null) capture = gameObject.AddComponent<PickSurfaceCapture>();
        if (_surfaceCapture != capture)
        {
            if (_surfaceCapture != null) { _surfaceCapture.SurfaceCaptured -= OnSurfaceCaptured; _surfaceCapture.Changed -= OnSurfaceChanged; }
            _surfaceCapture = capture;
            _surfaceCapture.SurfaceCaptured += OnSurfaceCaptured;
            _surfaceCapture.Changed += OnSurfaceChanged;
        }
        RobotPointRegistration registration = FindFirstObjectByType<RobotPointRegistration>();
        if (registration != null) capture.SetTipOffset(registration.TipOffsetInPointer);
        RefreshSurfacePresentation();
        return capture;
    }

    private void RefreshSurfacePresentation()
    {
        PickSurfaceCapture capture = SurfaceCapture;
        if (capture != null) capture.SetItemPresentationEnabled(isActiveAndEnabled && _tilePlanningEnabled &&
            _pickBySurface && _selectedItem == capture.SurfaceItem);
    }

    private void OnSurfaceChanged()
    {
        RefreshSurfacePresentation();
        InvalidatePlanningRequest();
        SettingsChanged?.Invoke();
    }

    private void OnSurfaceCaptured()
    {
        if (!_pickBySurface || !HasValidSurface || !_surfaceCapture.TryGetWorldPickTcpPose(out Pose pick)) return;
        _applyingSurface = true;
        try { SetPickPlace(pick.position, pick.rotation); }
        finally { _applyingSurface = false; }
        SettingsChanged?.Invoke();
    }

    public void InvalidatePlanningRequest()
    {
        CancelCapabilityRequest();
        _hasPlanningRequestBeenSent = false;
        if (PlansManager != null) PlansManager.InvalidatePlan();
        UpdatePlanningReadiness();
    }

    public void SetTilePlanningEnabled(bool enabled)
    {
        if (!enabled) SurfaceCapture?.Cancel();
        bool changed = _tilePlanningEnabled != enabled;
        _tilePlanningEnabled = enabled;
        RefreshSurfacePresentation();
        if (changed) { CancelCapabilityRequest(); _hasPlanningRequestBeenSent = false; SetTrackingLost(); }
        if (PlansManager != null) PlansManager.SetTilePlanningEnabled(enabled);
        if (_pickPointInstance != null) _pickPointInstance.SetActive(enabled);
        if (_placePointInstance != null) _placePointInstance.SetActive(enabled);
        UpdatePlanningReadiness();
    }

    private void UpdatePlanningReadiness()
    {
        RobotStateService.Instance.IsReadyForPlanning = _tilePlanningEnabled &&
            (!_pickBySurface || HasValidSurface) &&
            ToolSelectionController.CanPlanPickPlace && RobotCoordinateService.Instance.IsInitialized &&
            _pickPointInstance != null && _placePointInstance != null;
        if (_remoteExecutionController != null)
            _remoteExecutionController.SetPoseSources(
                _tilePlanningEnabled && _pickPointInstance != null ? _pickPointInstance.transform : null,
                _tilePlanningEnabled && _placePointInstance != null ? _placePointInstance.transform : null);
    }

    public void SetPointerPose(Pose pointerWorld, Transform trackingSpace)
    {
        if (!_tilePlanningEnabled || !RobotFrameMath.IsValidPose(pointerWorld) || !RobotFrameMath.IsRigidFrame(trackingSpace)) return;
        if (_livePointInstance == null)
        {
            _livePointInstance = new GameObject("Pick/place normal aim preview");
            NormalTargetMarker.Attach(_livePointInstance.transform, new Color(0.1f, 0.9f, 1f), 0.65f);
        }
        _livePointInstance.transform.SetParent(trackingSpace, true);
        Pose target = RobotFrameMath.PointerToTcpPose(pointerWorld);
        _livePointInstance.transform.SetPositionAndRotation(target.position, target.rotation);
        _livePointInstance.SetActive(true);
    }

    public void SetTrackingLost() { if (_livePointInstance != null) _livePointInstance.SetActive(false); }
    // Inputs are canonical TCP contact poses. Pointer conversion belongs to URInputManager.
    public void SetPickPlace(Vector3 position, Quaternion rotation)
    {
        if (_pickBySurface && !_applyingSurface) return;
        SetContactPose(true, new Pose(position, rotation));
    }
    public void SetPlacePlace(Vector3 position, Quaternion rotation) => SetContactPose(false, new Pose(position, rotation));

    private void SetContactPose(bool pick, Pose pose)
    {
        if (!_tilePlanningEnabled || !RobotFrameMath.IsValidPose(pose)) return;
        GameObject point = pick ? _pickPointInstance : _placePointInstance;
        Color tint = pick ? Color.green : new Color(1f, 0.6f, 0.05f);
        if (point == null)
        {
            point = new GameObject(pick ? "Pick — tool contact" : "Place — tool contact");
            if (pick) _pickPointInstance = point; else _placePointInstance = point;
        }
        // Reapply visuals on recapture as well, including after an interrupted first capture.
        NormalTargetMarker marker = NormalTargetMarker.Attach(point.transform, tint);
        if (pick && _pickBySurface)
        {
            // A measured surface owns the pick centre and orientation. Its marker is read-only.
            marker.SetBallVisible(true);
            if (_pickRotationHandles != null) { _pickRotationHandles.enabled = false; Unbind(_pickRotationHandles); Dispose(_pickRotationHandles); _pickRotationHandles = null; }
            TargetPoseContextMenu oldContext = point.GetComponent<TargetPoseContextMenu>();
            if (oldContext != null) { oldContext.enabled = false; Dispose(oldContext); }
            Transform surfaceTracking = null;
            if (XRTrackingSpaceResolver.TryResolve(ref surfaceTracking)) point.transform.SetParent(surfaceTracking, true);
            point.transform.SetPositionAndRotation(pose.position, pose.rotation);
            TargetNameLabel.Attach(point.transform, "Pick", tint);
            point.SetActive(_tilePlanningEnabled);
            InvalidatePlanningRequest();
            return;
        }
        // The rotation handle supplies the visible, highlightable ball and its collider.
        marker.SetBallVisible(false);
        TargetRotationHandles handles = pick ? _pickRotationHandles : _placeRotationHandles;
        if (handles == null)
        {
            Material material = MakeHandleMaterial(tint);
            handles = TargetRotationHandles.Attach(point.transform, material, 0.035f);
            handles.RotationStarted += OnTargetRotationChanged;
            handles.RotationChanged += OnTargetRotationChanged;
            if (pick)
            {
                _pickHandleMaterial = material; _pickRotationHandles = handles;
            }
            else
            {
                _placeHandleMaterial = material; _placeRotationHandles = handles;
            }
        }
        handles.enabled = true;
        handles.ConfigureNormalInteraction();
        Transform trackingSpace = null;
        if (XRTrackingSpaceResolver.TryResolve(ref trackingSpace)) point.transform.SetParent(trackingSpace, true);
        point.transform.SetPositionAndRotation(pose.position, pose.rotation);
        TargetNameLabel.Attach(point.transform, pick ? "Pick" : "Place", tint);
        TargetPoseContextMenu context = TargetPoseContextMenu.Attach(point.transform,
            pick ? "Pick contact" : "Place contact", true, OnTargetRotationChanged);
        point.SetActive(_tilePlanningEnabled);
        context.RebaseAtCurrentPose();
        InvalidatePlanningRequest();
    }

    private static Material MakeHandleMaterial(Color color)
    {
        return TargetMarkerMaterials.Create(color);
    }
    private void OnTargetRotationChanged() => InvalidatePlanningRequest();

    /// <summary>Export the displayed TCP contact poses without an object-surface or flange offset.</summary>
    public bool TryGetContactPoses(out PickPlacePosesMsg message)
    {
        message = null;
        // Keep the exported poses identical to the displayed constrained contacts, even before LateUpdate.
        if (_pickPointInstance != null)
            _pickPointInstance.GetComponent<TargetPoseContextMenu>()?.ApplyOrientationConstraint();
        if (_placePointInstance != null)
            _placePointInstance.GetComponent<TargetPoseContextMenu>()?.ApplyOrientationConstraint();
        if (!_tilePlanningEnabled || _pickPointInstance == null || _placePointInstance == null ||
            !RobotFrameMath.IsRigidFrame(_pickPointInstance.transform) || !RobotFrameMath.IsRigidFrame(_placePointInstance.transform) ||
            !RobotCoordinateService.Instance.TryConvertWorldPoseToLocalPose(
                new Pose(_pickPointInstance.transform.position, _pickPointInstance.transform.rotation), out Pose pick) ||
            !RobotCoordinateService.Instance.TryConvertWorldPoseToLocalPose(
                new Pose(_placePointInstance.transform.position, _placePointInstance.transform.rotation), out Pose place)) return false;
        string frame = RobotCoordinateService.Instance.RosBaseFrameId;
        message = new PickPlacePosesMsg(GeneratePoseStamped(pick, frame), GeneratePoseStamped(place, frame));
        return true;
    }

    public bool TryGetTaskRequest(out PickPlaceTaskMsg task, out string reason)
    {
        task = null;
        reason = null;
        if (_descentSpeedMmPerSecond != 5 && _descentSpeedMmPerSecond != 10 && _descentSpeedMmPerSecond != 20)
        { reason = "Choose a Cartesian descent speed of 5, 10 or 20 mm/s."; return false; }
        if (_selectedItem != PickItem.Tile && _selectedItem != PickItem.Brick)
        { reason = "Choose the tile or brick item."; return false; }
        if (_pickBySurface && !HasValidSurface)
        { reason = "Capture all three top-surface corners of the selected item first."; return false; }
        if (!TryGetContactPoses(out PickPlacePosesMsg contacts))
        { reason = "Capture both pick and place contacts with a valid robot placement."; return false; }
        task = new PickPlaceTaskMsg {
            contacts = contacts,
            cartesian_descent_speed_mps = _descentSpeedMmPerSecond / 1000d,
            item_type = _selectedItem == PickItem.Tile ? "tile" : "brick",
            pick_from_surface = _pickBySurface
        };
        if (_pickBySurface)
        {
            Vector3[] corners = SurfaceCapture.GetWorldCorners();
            if (corners == null || corners.Length != 3) { task = null; reason = "Surface corners are unavailable."; return false; }
            for (int i = 0; i < 3; i++)
            {
                if (!RobotCoordinateService.Instance.TryConvertWorldPoseToLocalPose(new Pose(corners[i], Quaternion.identity), out Pose local))
                { task = null; reason = "Surface corners cannot be converted to the robot frame."; return false; }
                task.top_corners[i] = local.position.To<FLU>();
            }
        }
        return true;
    }

    public void SendPickAndPlacePlanningRequest()
    {
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return;
        if (IsPlanning) return;
        RobotExecutionState.NotifyPlanningStarted();
        if (!ValidatePlanningRequest()) return;
        InvalidatePlanningRequest();
        if (!TryGetTaskRequest(out PickPlaceTaskMsg task, out string requestReason))
        { RejectPlanningRequest(requestReason); return; }
        try
        {
            if (_connection == null) _connection = ROSService.Instance.GetROSConnection();
            if (_connection == null || !_connection.HasConnectionThread || _connection.HasConnectionError)
            { RejectPlanningRequest("ROS is disconnected. Reconnect before planning MTC pick/place."); return; }
            _requestedContactPoses = task.contacts;
            _requestedTask = task;
            _requestedTimeoutSeconds = MtcPlanningSettings.TimeoutSeconds;
            _requestCalibrationRevision = RobotCoordinateService.Instance.CalibrationRevision;
            _checkingCapability = true;
            _capabilityDeadline = Time.realtimeSinceStartup + _capabilityTimeout;
            int generation = _generation;
            // Show progress without arming callbacks for a planning request that has not been sent.
            PlansManager.RejectPlanningRequest("Checking ROS item/speed pick support...");
            MtcPlanningStop.RequestCapabilities(_connection, CapabilityService, new[] { CapabilityParameter },
                () => this != null && generation == _generation && _checkingCapability,
                response => OnCapabilityResponse(response, generation));
        }
        catch (Exception)
        { RejectPlanningRequest("MTC planning request could not be sent. Check the ROS connection, then Plan again."); }
    }

    private void OnCapabilityResponse(GetParametersResponse response, int generation)
    {
        if (this == null || generation != _generation || !_checkingCapability) return;
        if (!CanKeepCapabilityRequest())
        { RejectPlanningRequest("Tool targets, mode, model or alignment changed. Plan pick/place again."); return; }
        if (response?.values == null || response.values.Length == 0 || response.values[0].type != 1 || !response.values[0].bool_value)
        { RejectPlanningRequest("ROS needs the item/speed pick update before using these settings. Update the MTC backend."); return; }
        bool timed = MtcPlanningStop.IsSupported(response.values, 2);
        if (!timed && _requestedTimeoutSeconds != MtcPlanningSettings.DefaultTimeoutSeconds)
        { RejectPlanningRequest(MtcPlanningStop.TimeoutUnsupported); return; }
        PickPlaceTaskMsg message = _requestedTask;
        _checkingCapability = false;
        MTCPlansManager plans = PlansManager;
        if (plans == null) return;
        try
        {
            string requestId = null;
            bool stopSupported = MtcPlanningStop.IsSupported(response.values, 1);
            if (timed || stopSupported)
            {
                var stamp = MtcPlanningStop.NewRequestStamp();
                message.contacts.pick_pose.header.stamp = stamp;
                requestId = MtcPlanningStop.RequestId(stamp);
            }
            bool started;
            if (timed) started = plans.BeginTimedPlanningRequest(_connection, requestId, _requestedTimeoutSeconds, stopSupported);
            else started = requestId == null ? plans.BeginPlanningRequest() : plans.BeginCorrelatedPlanningRequest(_connection, requestId);
            if (!started) return;
            if (timed)
                MtcPlanningStop.PublishTimedPlan(_connection, new MtcPlanRequestMsg
                {
                    mode = MtcPlanRequestMsg.PICK, timeout_seconds = (uint)_requestedTimeoutSeconds, pick = message
                });
            else _connection.Publish(TaskPlanTopic, message);
            _hasPlanningRequestBeenSent = true;
        }
        catch (Exception)
        {
            plans.ForgetUnsentPlanningRequest();
            RejectPlanningRequest("MTC planning request could not be sent. Check the ROS connection, then Plan again.");
        }
    }

    public void StopPlanning()
    {
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return;
        if (_checkingCapability)
        {
            RejectPlanningRequest("MTC planning stopped. Plan again.");
            return;
        }
        if (PlansManager == null || !PlansManager.RequestPlanningStop()) return;
        CancelCapabilityRequest();
        _hasPlanningRequestBeenSent = false;
    }

    private bool CanKeepCapabilityRequest() => _tilePlanningEnabled && _connection != null &&
        _connection.HasConnectionThread && !_connection.HasConnectionError &&
        !TargetPoseContextMenu.IsOpen &&
        !RobotCoordinateService.IsCalibrationEditActive && RobotCoordinateService.Instance.IsInitialized &&
        RobotCoordinateService.Instance.CalibrationRevision == _requestCalibrationRevision &&
        ToolSelectionController.CanPlanPickPlace && TargetsMatchRequest();
    private bool TargetsMatchRequest()
    {
        PickPlacePosesMsg requested = _requestedContactPoses;
        if (requested == null || !TryGetContactPoses(out PickPlacePosesMsg current) || _requestedContactPoses != requested ||
            !SamePose(requested.pick_pose, current.pick_pose) || !SamePose(requested.place_pose, current.place_pose)) return false;
        // Legacy offline review fixtures have no additive request snapshot.
        if (_requestedTask == null) return true;
        if (!TryGetTaskRequest(out PickPlaceTaskMsg task, out _) ||
            task.cartesian_descent_speed_mps != _requestedTask.cartesian_descent_speed_mps ||
            task.item_type != _requestedTask.item_type || task.pick_from_surface != _requestedTask.pick_from_surface) return false;
        for (int i = 0; i < 3; i++)
            if (Math.Abs(task.top_corners[i].x - _requestedTask.top_corners[i].x) >= 0.00001 ||
                Math.Abs(task.top_corners[i].y - _requestedTask.top_corners[i].y) >= 0.00001 ||
                Math.Abs(task.top_corners[i].z - _requestedTask.top_corners[i].z) >= 0.00001) return false;
        return true;
    }
    private static bool SamePose(PoseStampedMsg a, PoseStampedMsg b)
    {
        if (a.header.frame_id != b.header.frame_id) return false;
        PointMsg p = a.pose.position, q = b.pose.position;
        QuaternionMsg r = a.pose.orientation, s = b.pose.orientation;
        double norm = Math.Sqrt((r.x*r.x+r.y*r.y+r.z*r.z+r.w*r.w)*(s.x*s.x+s.y*s.y+s.z*s.z+s.w*s.w));
        double dot = r.x*s.x+r.y*s.y+r.z*s.z+r.w*s.w;
        return Math.Abs(p.x-q.x) < 0.00001 && Math.Abs(p.y-q.y) < 0.00001 &&
            Math.Abs(p.z-q.z) < 0.00001 && norm > 0 && Math.Abs(dot/norm) > 0.9999999;
    }

    private bool ValidatePlanningRequest()
    {
        if (PlansManager == null)
        { Debug.LogWarning("MTC planning status controller is missing. Planning is blocked.", this); return false; }
        if (TargetPoseContextMenu.IsOpen)
        { RejectPlanningRequest("Close the target menu before planning MTC pick/place."); return false; }
        if (!_tilePlanningEnabled)
        { RejectPlanningRequest("Select MTC pick/place mode to plan from tool contact points."); return false; }
        if (TargetRotationHandles.IsInteracting)
        { RejectPlanningRequest("Release the tool target before planning MTC pick/place."); return false; }
        if (RobotCoordinateService.IsCalibrationEditActive || !RobotCoordinateService.Instance.IsInitialized)
        { RejectPlanningRequest("Scan the robot QR and finish robot alignment before MTC planning."); return false; }
        if (ToolSelectionController.Instance != null && ToolSelectionController.Instance.SelectedTool == "none")
        { RejectPlanningRequest("MTC pick/place requires a suction tool. Select Kenos or Kenos SL and match ROS."); return false; }
        if (!ToolSelectionController.CanPlanPickPlace)
        { RejectPlanningRequest("MTC pick/place is blocked until Unity and ROS use the same suction-tool model."); return false; }
        if (_pickPointInstance == null || _placePointInstance == null)
        { RejectPlanningRequest("Capture both pick and place tool contact points before MTC planning."); return false; }
        return true;
    }
    private void RejectPlanningRequest(string reason)
    {
        CancelCapabilityRequest(); _hasPlanningRequestBeenSent = false;
        if (PlansManager != null) PlansManager.RejectPlanningRequest(reason); else Debug.LogWarning(reason, this);
    }

    public void SendFakePickAndPlacePlanningRequest()
    {
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return;
        if (!_tilePlanningEnabled || TargetRotationHandles.IsInteracting || !ToolSelectionController.CanPlanPickPlace ||
            !RobotCoordinateService.Instance.IsInitialized)
        { RejectPlanningRequest("Fake MTC planning requires pick/place mode, a matching suction tool and valid robot alignment."); return; }
        QuaternionMsg orientation = new QuaternionMsg(0.707107, 0.707107, 0, 0);
        Pose pick = new Pose(new PointMsg(0, 0.754485, 0.098).From<FLU>(), orientation.From<FLU>());
        Pose place = new Pose(new PointMsg(-0.4, 0.254485, 0.098).From<FLU>(), orientation.From<FLU>());
        pick = RobotCoordinateService.Instance.ConvertRootLocalPoseToWorldPose(pick);
        place = RobotCoordinateService.Instance.ConvertRootLocalPoseToWorldPose(place);
        SetPickPlace(pick.position, pick.rotation); SetPlacePlace(place.position, place.rotation);
        SendPickAndPlacePlanningRequest();
    }
    private static PoseStampedMsg GeneratePoseStamped(Pose localPose, string frame) =>
        new PoseStampedMsg(new HeaderMsg { frame_id = frame }, new PoseMsg(localPose.position.To<FLU>(), localPose.rotation.To<FLU>()));

    public void ExecutePickAndPlace()
    {
        // Repeated clicks must not clear the acknowledgement/outcome window or
        // publish another command while the first request is unresolved.
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return;
        if (TargetPoseContextMenu.IsOpen)
        { RejectPlanningRequest("Close the target menu before executing MTC pick/place."); return; }
        if (_checkingCapability || TargetRotationHandles.IsInteracting || !ToolSelectionController.CanPlanPickPlace ||
            !_tilePlanningEnabled || !RobotCoordinateService.Instance.IsInitialized ||
            (!_hasPlanningRequestBeenSent && !(PlansManager != null && PlansManager.CanRepeat)) ||
            !TargetsMatchRequest() || PlansManager == null || !PlansManager.CanExecute)
        { RejectPlanningRequest("MTC Execute requires a new successful pick/place plan and its visible trajectory preview."); return; }
        try
        {
            if (!PlansManager.ConsumePlanForExecution()) return;
            _hasPlanningRequestBeenSent = false;
            string repeat = PlansManager.RepeatExecutionPayload;
            string planId = PlansManager.IdentifiedExecutionPlanId;
            string reason;
            bool published;
            if (repeat != null) published = TryPublishRepeatExecutionRequest(repeat, out reason);
            else if (planId != null) published = TryPublishIdentifiedExecutionRequest(planId, out reason);
            else published = TryPublishExecutionRequest(out reason);
            if (!published) PlansManager.RejectExecutionRequest(reason);
        }
        catch (Exception exception)
        {
            Debug.LogWarning("MTC execution publication failed: " + exception.Message, this);
            PlansManager.RejectExecutionRequest("The execution request could not be sent. Check ROS and plan again before retrying.");
        }
    }

    // This transport boundary also permits offline tests of the actual public
    // Execute action without opening a connection or sending robot commands.
    protected virtual bool TryPublishExecutionRequest(out string reason)
    {
        if (_connection == null) _connection = ROSService.Instance.GetROSConnection();
        if (_connection == null || !_connection.HasConnectionThread || _connection.HasConnectionError)
        {
            reason = "ROS is disconnected. Reconnect and plan MTC pick/place again.";
            return false;
        }
        _connection.Publish("execute_task", new BoolMsg(true));
        Debug.Log("MTC execution command sent on execute_task; waiting for /mtc/execution_status.", this);
        reason = null;
        return true;
    }
    protected virtual bool TryPublishRepeatExecutionRequest(string payload, out string reason)
    {
        if (_connection == null) _connection = ROSService.Instance.GetROSConnection();
        if (_connection == null || !_connection.HasConnectionThread || _connection.HasConnectionError)
        { reason = "ROS is disconnected. Plan again after reconnecting."; return false; }
        _connection.Publish("/repeat_reviewed_plan", new StringMsg(payload));
        reason = null;
        return true;
    }

    private bool TryPublishIdentifiedExecutionRequest(string planId, out string reason)
    {
        if (_connection == null) _connection = ROSService.Instance.GetROSConnection();
        if (_connection == null || !_connection.HasConnectionThread || _connection.HasConnectionError)
        { reason = "ROS is disconnected. Reconnect and plan MTC pick/place again."; return false; }
        _connection.Publish(IdentifiedExecuteTopic, new StringMsg(planId));
        reason = null;
        return true;
    }
    private void OnEnable() => RefreshSurfacePresentation();
    private void OnDisable() { SurfaceCapture?.Cancel(); SurfaceCapture?.SetItemPresentationEnabled(false); CancelCapabilityRequest(); _hasPlanningRequestBeenSent = false; SetTrackingLost(); }
    private void OnDestroy()
    {
        if (_coordinates != null) _coordinates.CalibrationChanged -= InvalidatePlanningRequest;
        if (_surfaceCapture != null) { _surfaceCapture.SurfaceCaptured -= OnSurfaceCaptured; _surfaceCapture.Changed -= OnSurfaceChanged; }
        Unbind(_pickRotationHandles); Unbind(_placeRotationHandles);
        Dispose(_livePointInstance); Dispose(_pickPointInstance); Dispose(_placePointInstance);
        Dispose(_pickHandleMaterial); Dispose(_placeHandleMaterial);
    }
    private void Unbind(TargetRotationHandles handles)
    {
        if (handles == null) return;
        handles.RotationStarted -= OnTargetRotationChanged; handles.RotationChanged -= OnTargetRotationChanged;
    }
    private static void Dispose(UnityEngine.Object value)
    { if (value != null) { if (Application.isPlaying) Destroy(value); else DestroyImmediate(value); } }
}
