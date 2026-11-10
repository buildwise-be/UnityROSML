using UnityEngine;
using UnityEngine.InputSystem;

public class URInputManager : MonoBehaviour
{
    public enum WorkflowMode { Unrestricted = 0, SimplePickAndPlace = 1, RepetitivePickAndPlace = 2 }
    public enum PoseCaptureMode
    {
        [InspectorName("Pick/place tool contact poses")] TileSurface,
        [InspectorName("Exact TCP poses (plan preview)")] TcpPreview,
        [InspectorName("Fixture regrasp tile batch")] FixtureTiling
    }

    [SerializeField] private URInstantiator _URInstantiator;
    [SerializeField] private PickPlaceController _pickPlaceController;
    [SerializeField] private TcpTargetPreview _tcpTargetPreview;
    [SerializeField] private TcpPlanningController _tcpPlanningController;
    [SerializeField] private FixtureTilingController _fixtureTilingController;
    [SerializeField] private WorkflowMode _workflowMode;
    [SerializeField, Tooltip("OpenXR tracking-space transform. Automatically uses the single XR Origin's Camera Offset when empty.")]
    private Transform _trackingSpace;
    [SerializeField, Tooltip("TCP fixes full tool orientation; pick/place captures contact normals; fixture tiling captures item and fixture points.")]
    private PoseCaptureMode _poseCaptureMode = PoseCaptureMode.TileSurface;

    private MagicLeapOpenXRInput _magicLeapInputs;
    private MagicLeapOpenXRInput.ControllerActions _controllerActions;
    private PoseCaptureMode? _appliedMode;
    private bool _robotPlacementRequested;
    private float _contextBumperConsumedUntil;
    private RobotPointRegistration PointRegistration => _URInstantiator != null
        ? _URInstantiator.GetComponent<RobotPointRegistration>() : null;
    public bool IsTcpMode => _workflowMode != WorkflowMode.RepetitivePickAndPlace && _poseCaptureMode == PoseCaptureMode.TcpPreview;
    public bool IsFixtureMode => _workflowMode == WorkflowMode.RepetitivePickAndPlace ||
        (_workflowMode == WorkflowMode.Unrestricted && _poseCaptureMode == PoseCaptureMode.FixtureTiling);
    public WorkflowMode SelectedWorkflow => _workflowMode;
    public PickPlaceController PickPlaceController => _pickPlaceController;
    private PickSurfaceCapture SurfaceCapture => _pickPlaceController != null ? _pickPlaceController.SurfaceCapture : null;
    public bool IsCapturingPickSurface => SurfaceCapture != null && SurfaceCapture.IsCapturing;
    public FixtureTilingController FixtureController
    {
        get
        {
            if (_workflowMode == WorkflowMode.SimplePickAndPlace) return null;
            if (_fixtureTilingController == null) _fixtureTilingController = GetComponent<FixtureTilingController>();
            if (_fixtureTilingController == null) _fixtureTilingController = gameObject.AddComponent<FixtureTilingController>();
            return _fixtureTilingController;
        }
    }

    private void Start()
    {
        ApplyWorkflowConstraint();
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
            CancelPickSurfaceCapture();
            if (_tcpTargetPreview != null) _tcpTargetPreview.SetTrackingLost();
            if (_pickPlaceController != null) _pickPlaceController.SetTrackingLost();
            if (!ToolSelectionMenu.IsHovered && TryGetPointerWorldPose(out Pose contactPointer))
                registration.SetPointerPose(contactPointer, _trackingSpace);
            else registration.SetTrackingLost();
            return;
        }
        if (RobotCoordinateService.IsCalibrationEditActive)
        {
            CancelPickSurfaceCapture();
            _robotPlacementRequested = false;
            if (_tcpTargetPreview != null) _tcpTargetPreview.SetTrackingLost();
            if (_pickPlaceController != null) _pickPlaceController.SetTrackingLost();
            return;
        }
        if (IsCapturingPickSurface)
        {
            if (_tcpTargetPreview != null) _tcpTargetPreview.SetTrackingLost();
            if (_pickPlaceController != null) _pickPlaceController.SetTrackingLost();
            if (!IsTcpMode && !IsFixtureMode && !ToolSelectionMenu.IsHovered &&
                !TargetPoseContextMenu.IsOpen && !TargetPoseContextMenu.IsHovered &&
                !TargetRotationHandles.IsHovered && !TargetRotationHandles.IsInteracting &&
                TryGetPointerWorldPose(out Pose surfacePointer))
                SurfaceCapture.SetPointerPose(surfacePointer, _trackingSpace);
            else SurfaceCapture.SetTrackingLost();
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
        ApplyWorkflowConstraint();
        if (_appliedMode == _poseCaptureMode) return;
        bool modeChanged = _appliedMode.HasValue;
        _appliedMode = _poseCaptureMode;
        if (modeChanged)
        {
            CancelPickSurfaceCapture();
            _robotPlacementRequested = false;
            TargetPoseContextMenu.CloseAll();
            TargetRotationHandles.CancelAllInteractions();
            if (_tcpPlanningController != null) _tcpPlanningController.InvalidatePlan("Task mode changed. Plan again in the selected mode.");
            foreach (MTCPlansManager plans in FindObjectsByType<MTCPlansManager>(FindObjectsSortMode.None))
                plans.InvalidatePlan();
            FindFirstObjectByType<UR16eTrajectoryVisualizer>()?.SetTrajectory(null);
        }
        if (_tcpTargetPreview != null) _tcpTargetPreview.SetTargetsVisible(IsTcpMode);
        if (_pickPlaceController != null)
            _pickPlaceController.SetTilePlanningEnabled(_poseCaptureMode == PoseCaptureMode.TileSurface);
        if (_workflowMode != WorkflowMode.SimplePickAndPlace) FixtureController.SetTaskActive(IsFixtureMode);
        else if (_fixtureTilingController != null) _fixtureTilingController.SetTaskActive(false);
    }

    public void ConfigureWorkflow(WorkflowMode workflow)
    {
        _workflowMode = workflow;
        ApplyWorkflowConstraint();
        ApplyCaptureMode();
    }

    private void ApplyWorkflowConstraint()
    {
        if (_workflowMode == WorkflowMode.RepetitivePickAndPlace) _poseCaptureMode = PoseCaptureMode.FixtureTiling;
        else if (_workflowMode == WorkflowMode.SimplePickAndPlace && _poseCaptureMode == PoseCaptureMode.FixtureTiling)
            _poseCaptureMode = PoseCaptureMode.TcpPreview;
    }

    public void UseTcpPreview() => SelectMode(PoseCaptureMode.TcpPreview);
    public void UseTilePickPlace() => SelectMode(PoseCaptureMode.TileSurface);
    public void UseFixtureTiling() => SelectMode(PoseCaptureMode.FixtureTiling);
    private void SelectMode(PoseCaptureMode mode)
    {
        if ((_workflowMode == WorkflowMode.SimplePickAndPlace && mode == PoseCaptureMode.FixtureTiling) ||
            (_workflowMode == WorkflowMode.RepetitivePickAndPlace && mode != PoseCaptureMode.FixtureTiling)) return;
        // A programmatic mode change revokes local review even while an earlier
        // command awaits its outcome. Plan/Execute retain the global command lock.
        _poseCaptureMode = mode;
        ApplyCaptureMode();
    }

    public void PlanCurrentTask()
    {
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return;
        MTCPlansManager.DismissCompletedExecutionFeedback();
        ApplyCaptureMode();
        if (IsFixtureMode) FixtureController.Plan();
        else if (IsTcpMode)
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
        if (IsFixtureMode) FixtureController.Execute();
        else if (IsTcpMode)
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
        if (IsCapturingPickSurface)
        {
            if (!IsTcpMode && !IsFixtureMode && !RobotCoordinateService.IsCalibrationEditActive &&
                !ToolSelectionMenu.IsHovered && !TargetPoseContextMenu.IsOpen && !TargetPoseContextMenu.IsHovered &&
                !TargetRotationHandles.IsHovered && !TargetRotationHandles.IsInteracting &&
                TryGetPointerWorldPose(out Pose surfacePointer))
                SurfaceCapture.CapturePointerPose(surfacePointer, _trackingSpace);
            return;
        }
        if (IsFixtureMode)
        {
            if (!RobotCoordinateService.IsCalibrationEditActive && !ToolSelectionMenu.IsHovered &&
                !TargetPoseContextMenu.IsOpen && !TargetPoseContextMenu.IsHovered &&
                !TargetRotationHandles.IsHovered && !TargetRotationHandles.IsInteracting &&
                FixtureController.IsCapturing && TryGetPointerWorldPose(out Pose fixturePointer))
                FixtureController.CapturePointerPose(fixturePointer, _trackingSpace);
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
        if (IsFixtureMode || IsCapturingPickSurface) return;
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
        if (IsFixtureMode || IsCapturingPickSurface || (PointRegistration != null && PointRegistration.IsCollecting)) return;
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
        if (isPick && _pickPlaceController.PickBySurface) return;
        Pose contactPose = RobotFrameMath.PointerToTcpPose(pointerWorld);
        if (isPick) _pickPlaceController.SetPickPlace(contactPose.position, contactPose.rotation);
        else _pickPlaceController.SetPlacePlace(contactPose.position, contactPose.rotation);
    }

    private void OnEnable() { _magicLeapInputs?.Enable(); }
    private void OnDisable()
    {
        // Disabling an action can synchronously report a canceled bumper; clear its latch afterwards.
        _magicLeapInputs?.Disable();
        CancelPickSurfaceCapture();
        if (_fixtureTilingController != null) _fixtureTilingController.CancelCapture();
        ResetPendingInteraction();
    }

    private void OnApplicationFocus(bool hasFocus)
    {
        if (!hasFocus)
        {
            CancelPickSurfaceCapture();
            if (_fixtureTilingController != null) _fixtureTilingController.CancelCapture();
            ResetPendingInteraction();
        }
    }

    private void OnApplicationPause(bool isPaused)
    {
        if (isPaused)
        {
            CancelPickSurfaceCapture();
            if (_fixtureTilingController != null) _fixtureTilingController.CancelCapture();
            ResetPendingInteraction();
        }
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
        if (SurfaceCapture != null) SurfaceCapture.SetTrackingLost();
    }

    public void CancelPickSurfaceCapture()
    {
        if (SurfaceCapture != null && SurfaceCapture.IsCapturing) SurfaceCapture.Cancel();
    }

    private void OnDestroy()
    {
        CancelPickSurfaceCapture();
        if (_magicLeapInputs == null) return;
        _controllerActions.Trigger.performed -= HandleOnTrigger;
        _controllerActions.Bumper.performed -= HandleOnBumper;
        _controllerActions.Bumper.canceled -= HandleOnBumperReleased;
        _controllerActions.DoubleBumper.performed -= HandleOnDoubleBumper;
        _controllerActions.HoldBumper.performed -= HandleOnHoldBumper;
        _magicLeapInputs.Dispose();
    }
}
