using UnityEngine;
using UnityEngine.InputSystem;

public class URInputManager : MonoBehaviour
{
    public enum PoseCaptureMode
    {
        [InspectorName("Pick/place tool contact poses")] TileSurface,
        [InspectorName("Exact TCP poses (plan preview)")] TcpPreview
    }

    [SerializeField] private URInstantiator _URInstantiator;
    [SerializeField] private PickPlaceController _pickPlaceController;
    [SerializeField] private TcpTargetPreview _tcpTargetPreview;
    [SerializeField] private TcpPlanningController _tcpPlanningController;
    [SerializeField, Tooltip("OpenXR tracking-space transform. Automatically uses the single XR Origin's Camera Offset when empty.")]
    private Transform _trackingSpace;
    [SerializeField, Tooltip("Both modes capture the desired tool contact pose. Exact TCP fixes full orientation; MTC pick/place allows tool twist about +Z.")]
    private PoseCaptureMode _poseCaptureMode = PoseCaptureMode.TileSurface;

    private MagicLeapOpenXRInput _magicLeapInputs;
    private MagicLeapOpenXRInput.ControllerActions _controllerActions;
    private PoseCaptureMode? _appliedMode;
    private bool _robotPlacementRequested;
    private float _contextBumperConsumedUntil;
    private RobotPointRegistration PointRegistration => _URInstantiator != null
        ? _URInstantiator.GetComponent<RobotPointRegistration>() : null;
    public bool IsTcpMode => _poseCaptureMode == PoseCaptureMode.TcpPreview;

    private void Start()
    {
        if (_tcpTargetPreview == null)
            _tcpTargetPreview = GetComponent<TcpTargetPreview>();
        if (_tcpPlanningController == null)
            _tcpPlanningController = GetComponent<TcpPlanningController>();
        _magicLeapInputs = new MagicLeapOpenXRInput();
        _controllerActions = new MagicLeapOpenXRInput.ControllerActions(_magicLeapInputs);
        _controllerActions.Trigger.performed += HandleOnTrigger;
        _controllerActions.Bumper.performed += HandleOnBumper;
        _controllerActions.Bumper.canceled += HandleOnBumperReleased;
        _controllerActions.DoubleBumper.performed += HandleOnDoubleBumper;
        _controllerActions.HoldBumper.performed += HandleOnHoldBumper;
        _magicLeapInputs.Enable();
        ApplyCaptureMode();
    }

    private void Update()
    {
        ApplyCaptureMode();
        RobotPointRegistration registration = PointRegistration;
        if (registration != null && registration.IsCollecting)
        {
            if (_tcpTargetPreview != null) _tcpTargetPreview.SetTrackingLost();
            if (_pickPlaceController != null) _pickPlaceController.SetTrackingLost();
            if (!ToolSelectionMenu.IsHovered && TryGetPointerWorldPose(out Pose contactPointer))
                registration.SetPointerPose(contactPointer, _trackingSpace);
            else registration.SetTrackingLost();
            return;
        }
        if (RobotCoordinateService.IsCalibrationEditActive)
        {
            _robotPlacementRequested = false;
            if (_tcpTargetPreview != null) _tcpTargetPreview.SetTrackingLost();
            if (_pickPlaceController != null) _pickPlaceController.SetTrackingLost();
            return;
        }
        if (!TryGetPointerWorldPose(out Pose pointerWorld))
        {
            ResetPendingInteraction();
            return;
        }
        bool showAim = !ToolSelectionMenu.IsHovered && !TargetPoseContextMenu.IsOpen &&
            !TargetPoseContextMenu.IsHovered && !TargetRotationHandles.IsHovered && !TargetRotationHandles.IsInteracting;
        if (showAim && _poseCaptureMode == PoseCaptureMode.TcpPreview && _tcpTargetPreview != null)
            _tcpTargetPreview.SetPointerPose(pointerWorld, _trackingSpace);
        else
            if (_tcpTargetPreview != null) _tcpTargetPreview.SetTrackingLost();
        if (showAim && _poseCaptureMode == PoseCaptureMode.TileSurface && _pickPlaceController != null)
            _pickPlaceController.SetPointerPose(pointerWorld, _trackingSpace);
        else
            if (_pickPlaceController != null) _pickPlaceController.SetTrackingLost();
    }

    private void ApplyCaptureMode()
    {
        if (_appliedMode == _poseCaptureMode) return;
        bool modeChanged = _appliedMode.HasValue;
        _appliedMode = _poseCaptureMode;
        if (modeChanged)
        {
            _robotPlacementRequested = false;
            TargetPoseContextMenu.CloseAll();
            TargetRotationHandles.CancelAllInteractions();
            if (_tcpPlanningController != null) _tcpPlanningController.InvalidatePlan("Task mode changed. Plan again in the selected mode.");
            foreach (MTCPlansManager plans in FindObjectsByType<MTCPlansManager>(FindObjectsSortMode.None))
                plans.InvalidatePlan();
        }
        if (_tcpTargetPreview != null) _tcpTargetPreview.SetTargetsVisible(IsTcpMode);
        if (_pickPlaceController != null)
            _pickPlaceController.SetTilePlanningEnabled(!IsTcpMode);
    }

    public void UseTcpPreview() { _poseCaptureMode = PoseCaptureMode.TcpPreview; ApplyCaptureMode(); }
    public void UseTilePickPlace() { _poseCaptureMode = PoseCaptureMode.TileSurface; ApplyCaptureMode(); }

    public void PlanCurrentTask()
    {
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return;
        MTCPlansManager.DismissCompletedExecutionFeedback();
        ApplyCaptureMode();
        if (IsTcpMode)
        {
            if (_tcpPlanningController != null) _tcpPlanningController.Plan();
            else Debug.LogError("Exact TCP planner is missing from this scene.", this);
        }
        else if (_pickPlaceController != null) _pickPlaceController.SendPickAndPlacePlanningRequest();
    }

    public void ExecuteCurrentTask()
    {
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return;
        ApplyCaptureMode();
        if (IsTcpMode)
        {
            if (_tcpPlanningController != null) _tcpPlanningController.Execute();
        }
        else if (_pickPlaceController != null) _pickPlaceController.ExecutePickAndPlace();
    }

    private bool TryGetPointerWorldPose(out Pose pointerWorld)
    {
        pointerWorld = default;
        if (_magicLeapInputs == null || !_controllerActions.IsTracked.IsPressed() ||
            (_controllerActions.Status.ReadValue<int>() & 3) != 3 ||
            _controllerActions.PointerPosition.activeControl == null ||
            _controllerActions.PointerRotation.activeControl == null ||
            !XRTrackingSpaceResolver.TryResolve(ref _trackingSpace)) return false;
        Pose trackingPose = new Pose(_controllerActions.PointerPosition.ReadValue<Vector3>(),
            _controllerActions.PointerRotation.ReadValue<Quaternion>());
        return RobotFrameMath.TryTrackingToWorld(_trackingSpace, trackingPose, out pointerWorld);
    }

    private void HandleOnTrigger(InputAction.CallbackContext context)
    {
        // XRI processes this same trigger to select handles during Update.
        // Wait until LateUpdate before deciding whether it requests robot placement.
        if (context.ReadValueAsButton()) _robotPlacementRequested = true;
    }

    private void LateUpdate()
    {
        if (!_robotPlacementRequested) return;
        _robotPlacementRequested = false;
        RobotPointRegistration registration = PointRegistration;
        if (registration != null && registration.IsCollecting)
        {
            if (!ToolSelectionMenu.IsHovered && !TargetPoseContextMenu.IsOpen && !TargetPoseContextMenu.IsHovered &&
                !TargetRotationHandles.IsHovered && !TargetRotationHandles.IsInteracting &&
                TryGetPointerWorldPose(out Pose pointerWorld))
                registration.CapturePointerPose(pointerWorld, _trackingSpace);
            return;
        }
        if (!RobotCoordinateService.IsCalibrationEditActive &&
            !ToolSelectionMenu.IsHovered && !TargetPoseContextMenu.IsOpen && !TargetPoseContextMenu.IsHovered &&
            !TargetRotationHandles.IsHovered && !TargetRotationHandles.IsInteracting &&
            _URInstantiator != null && TryGetPointerWorldPose(out _))
            _URInstantiator.InstantiateUR();
    }

    private void HandleOnHoldBumper(InputAction.CallbackContext context) { Capture(true); }
    private void HandleOnDoubleBumper(InputAction.CallbackContext context) { Capture(false); }

    private void HandleOnBumper(InputAction.CallbackContext context)
    {
        if (PointRegistration != null && PointRegistration.IsCollecting) return;
        if (TargetPoseContextMenu.TryOpenHovered() || TargetPoseContextMenu.IsOpen || TargetPoseContextMenu.IsHovered)
            _contextBumperConsumedUntil = float.PositiveInfinity;
    }

    private void HandleOnBumperReleased(InputAction.CallbackContext context)
    {
        if (float.IsPositiveInfinity(_contextBumperConsumedUntil))
            // A context-menu bumper press must not complete a delayed DoubleBumper capture.
            _contextBumperConsumedUntil = Time.realtimeSinceStartup + 0.5f;
    }

    private void Capture(bool isPick)
    {
        if (Time.realtimeSinceStartup < _contextBumperConsumedUntil ||
            TargetPoseContextMenu.IsOpen || TargetPoseContextMenu.IsHovered ||
            RobotCoordinateService.IsCalibrationEditActive || ToolSelectionMenu.IsHovered ||
            TargetRotationHandles.IsHovered || TargetRotationHandles.IsInteracting) return;
        if (!TryGetPointerWorldPose(out Pose pointerWorld))
        {
            Debug.LogWarning("Cannot capture a target without a tracked controller and a valid XR tracking frame.");
            return;
        }
        ApplyCaptureMode();
        if (_poseCaptureMode == PoseCaptureMode.TcpPreview)
        {
            if (_tcpTargetPreview != null)
                _tcpTargetPreview.CapturePointerPose(pointerWorld, _trackingSpace, isPick);
            return;
        }
        if (_pickPlaceController == null) return;
        Pose contactPose = RobotFrameMath.PointerToTcpPose(pointerWorld);
        if (isPick) _pickPlaceController.SetPickPlace(contactPose.position, contactPose.rotation);
        else _pickPlaceController.SetPlacePlace(contactPose.position, contactPose.rotation);
    }

    private void OnEnable() { _magicLeapInputs?.Enable(); }
    private void OnDisable()
    {
        // Disabling an action can synchronously report a canceled bumper; clear its latch afterwards.
        _magicLeapInputs?.Disable();
        ResetPendingInteraction();
    }

    private void OnApplicationFocus(bool hasFocus)
    {
        if (!hasFocus) ResetPendingInteraction();
    }

    private void OnApplicationPause(bool isPaused)
    {
        if (isPaused) ResetPendingInteraction();
    }

    private void ResetPendingInteraction()
    {
        _robotPlacementRequested = false;
        if (PointRegistration != null) PointRegistration.SetTrackingLost();
        _contextBumperConsumedUntil = 0f;
        TargetPoseContextMenu.CloseAll();
        TargetRotationHandles.CancelAllInteractions();
        ToolSelectionMenu.CancelAllInteractions();
        if (_tcpTargetPreview != null) _tcpTargetPreview.SetTrackingLost();
        if (_pickPlaceController != null) _pickPlaceController.SetTrackingLost();
    }

    private void OnDestroy()
    {
        if (_magicLeapInputs == null) return;
        _controllerActions.Trigger.performed -= HandleOnTrigger;
        _controllerActions.Bumper.performed -= HandleOnBumper;
        _controllerActions.Bumper.canceled -= HandleOnBumperReleased;
        _controllerActions.DoubleBumper.performed -= HandleOnDoubleBumper;
        _controllerActions.HoldBumper.performed -= HandleOnHoldBumper;
        _magicLeapInputs.Dispose();
    }
}
