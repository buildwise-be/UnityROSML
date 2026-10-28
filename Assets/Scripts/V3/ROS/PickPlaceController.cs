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
    private const string CapabilityService = "/mtc_test_task_node/get_parameters";
    private const string CapabilityParameter = "pick_place_tcp_planning_enabled";
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
    private int _generation, _requestCalibrationRevision;
    private float _capabilityDeadline;
    private PickPlacePosesMsg _requestedContactPoses;
    private RobotCoordinateService _coordinates;
    private ROSConnection _connection;
    public bool IsTilePlanningEnabled => _tilePlanningEnabled;
    public bool IsPlanning => _checkingCapability || (PlansManager != null && PlansManager.IsPlanning);
    private MTCPlansManager PlansManager
    {
        get { if (_plansManager == null) _plansManager = FindFirstObjectByType<MTCPlansManager>(); return _plansManager; }
    }

    private void Start()
    {
        _coordinates = RobotCoordinateService.Instance;
        _coordinates.CalibrationChanged += InvalidatePlanningRequest;
        _connection = ROSService.Instance.GetROSConnection();
        _pickPlaceTopicName = ContactPlanTopic;
        _connection.RegisterPublisher<PickPlacePosesMsg>(_pickPlaceTopicName);
        _connection.RegisterPublisher<BoolMsg>("execute_task");
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
        bool changed = _tilePlanningEnabled != enabled;
        _tilePlanningEnabled = enabled;
        if (changed) { CancelCapabilityRequest(); _hasPlanningRequestBeenSent = false; SetTrackingLost(); }
        if (PlansManager != null) PlansManager.SetTilePlanningEnabled(enabled);
        if (_pickPointInstance != null) _pickPointInstance.SetActive(enabled);
        if (_placePointInstance != null) _placePointInstance.SetActive(enabled);
        UpdatePlanningReadiness();
    }

    private void UpdatePlanningReadiness()
    {
        RobotStateService.Instance.IsReadyForPlanning = _tilePlanningEnabled &&
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
    public void SetPickPlace(Vector3 position, Quaternion rotation) => SetContactPose(true, new Pose(position, rotation));
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
        context.RebaseAtCurrentPose();
        point.SetActive(_tilePlanningEnabled);
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

    public void SendPickAndPlacePlanningRequest()
    {
        if (MTCPlansManager.ActiveExecution != null) return;
        if (!ValidatePlanningRequest() || IsPlanning) return;
        InvalidatePlanningRequest();
        if (!TryGetContactPoses(out PickPlacePosesMsg message))
        { RejectPlanningRequest("Cannot plan MTC pick/place without valid tool targets and robot alignment."); return; }
        try
        {
            if (_connection == null) _connection = ROSService.Instance.GetROSConnection();
            if (_connection == null || !_connection.HasConnectionThread || _connection.HasConnectionError)
            { RejectPlanningRequest("ROS is disconnected. Reconnect before planning MTC pick/place."); return; }
            _requestedContactPoses = message;
            _requestCalibrationRevision = RobotCoordinateService.Instance.CalibrationRevision;
            _checkingCapability = true;
            _capabilityDeadline = Time.realtimeSinceStartup + _capabilityTimeout;
            int generation = _generation;
            // Show progress without arming callbacks for a planning request that has not been sent.
            PlansManager.RejectPlanningRequest("Checking ROS pick/place tool-pose support...");
            _connection.SendServiceMessage<GetParametersResponse>(CapabilityService,
                new GetParametersRequest(new[] { CapabilityParameter }), response => OnCapabilityResponse(response, generation));
        }
        catch (Exception)
        { RejectPlanningRequest("MTC planning request could not be sent. Check the ROS connection, then Plan again."); }
    }

    private void OnCapabilityResponse(GetParametersResponse response, int generation)
    {
        if (this == null || generation != _generation || !_checkingCapability) return;
        if (!CanKeepCapabilityRequest())
        { RejectPlanningRequest("Tool targets, mode, model or alignment changed. Plan pick/place again."); return; }
        if (response?.values == null || response.values.Length != 1 || response.values[0].type != 1 || !response.values[0].bool_value)
        { RejectPlanningRequest("ROS needs the pick/place tool-pose update. Update the MTC backend; surface-pose fallback is disabled."); return; }
        PickPlacePosesMsg message = _requestedContactPoses;
        _checkingCapability = false;
        try
        {
            if (PlansManager == null || !PlansManager.BeginPlanningRequest()) return;
            _connection.Publish(ContactPlanTopic, message);
            _hasPlanningRequestBeenSent = true;
        }
        catch (Exception)
        { RejectPlanningRequest("MTC planning request could not be sent. Check the ROS connection, then Plan again."); }
    }

    private bool CanKeepCapabilityRequest() => _tilePlanningEnabled && _connection != null &&
        _connection.HasConnectionThread && !_connection.HasConnectionError &&
        !TargetPoseContextMenu.IsOpen &&
        !RobotCoordinateService.IsCalibrationEditActive && RobotCoordinateService.Instance.IsInitialized &&
        RobotCoordinateService.Instance.CalibrationRevision == _requestCalibrationRevision &&
        ToolSelectionController.CanPlanPickPlace && TargetsMatchRequest();
    private bool TargetsMatchRequest() => _requestedContactPoses != null && TryGetContactPoses(out PickPlacePosesMsg current) &&
        SamePose(_requestedContactPoses.pick_pose, current.pick_pose) && SamePose(_requestedContactPoses.place_pose, current.place_pose);
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
        if (MTCPlansManager.ActiveExecution != null) return;
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
        if (MTCPlansManager.ActiveExecution != null) return;
        if (TargetPoseContextMenu.IsOpen)
        { RejectPlanningRequest("Close the target menu before executing MTC pick/place."); return; }
        if (_checkingCapability || TargetRotationHandles.IsInteracting || !ToolSelectionController.CanPlanPickPlace ||
            !_tilePlanningEnabled || !RobotCoordinateService.Instance.IsInitialized || !_hasPlanningRequestBeenSent ||
            !TargetsMatchRequest() || PlansManager == null || !PlansManager.CanExecute)
        { RejectPlanningRequest("MTC Execute requires a new successful pick/place plan and its visible trajectory preview."); return; }
        try
        {
            if (!PlansManager.ConsumePlanForExecution()) return;
            _hasPlanningRequestBeenSent = false;
            if (!TryPublishExecutionRequest(out string reason)) PlansManager.RejectExecutionRequest(reason);
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
    private void OnDisable() { CancelCapabilityRequest(); _hasPlanningRequestBeenSent = false; SetTrackingLost(); }
    private void OnDestroy()
    {
        if (_coordinates != null) _coordinates.CalibrationChanged -= InvalidatePlanningRequest;
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
