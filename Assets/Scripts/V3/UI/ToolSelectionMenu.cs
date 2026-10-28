using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Filtering;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

/// <summary>Shared XR controls for tool matching, TCP planning and marker alignment.</summary>
public sealed class ToolSelectionMenu : MonoBehaviour
{
    [SerializeField] private ToolSelectionController _controller;
    [SerializeField] private URInputManager _input;
    [SerializeField] private TcpPlanningController _tcpPlanning;
    [SerializeField] private QRDetector _qrDetector;
    [SerializeField] private URInstantiator _instantiator;
    [SerializeField] private UR16eTrajectoryVisualizer _trajectoryVisualizer;
    [SerializeField, Tooltip("Existing transparent app logo used in the task and alignment headers.")]
    private Texture2D _appLogo;
    [SerializeField, Tooltip("Optional canvas dedicated to this menu. A world-space canvas is created when empty.")]
    private Canvas _canvas;

    [Header("Initial and recentered placement")]
    [SerializeField, Min(0.6f)] private float _initialDistance = 1f;
    [SerializeField, Range(-40f, 40f)] private float _initialSideAngle = 20f;
    [SerializeField] private float _initialHeightOffset = -0.15f;

    private sealed class Choice
    {
        public string Tool;
        public string Title;
        public Button Button;
        public TextMeshProUGUI Label;
        public Outline Border;
        public UnityAction Click;
    }

    private static readonly HashSet<ToolSelectionMenu> HoveredMenus = new HashSet<ToolSelectionMenu>();
    private static readonly HashSet<ToolSelectionMenu> ActiveMenus = new HashSet<ToolSelectionMenu>();
    public static bool IsHovered => HoveredMenus.Count != 0;

    /// <summary>Release menu grabs when the controller loses tracking or the app is suspended.</summary>
    public static void CancelAllInteractions()
    {
        foreach (ToolSelectionMenu menu in new List<ToolSelectionMenu>(ActiveMenus))
            if (menu != null) menu.ClearPointerState();
    }

    private readonly List<Choice> _choices = new List<Choice>();
    private readonly HashSet<int> _hoveringPointers = new HashSet<int>();
    private readonly HashSet<int> _pressedPointers = new HashSet<int>();
    private ToolSelectionController _subscribedController;
    private TcpPlanningController _subscribedPlanning;
    private QRDetector _subscribedQr;
    private URInstantiator _subscribedInstantiator;
    private MTCPlansManager _legacyPlans;
    private RectTransform _panel;
    private RectTransform _taskView;
    private RectTransform _alignmentView;
    private TextMeshProUGUI _status;
    private Button _tcpTaskButton;
    private Button _pickPlaceTaskButton;
    private TextMeshProUGUI _taskHelp;
    private TextMeshProUGUI _taskStatus;
    private TextMeshProUGUI _qrStatus;
    private Button _planButton;
    private Button _executeButton;
    private Button _rescanButton;
    private Button _recenterButton;
    private Button _alignButton;
    private Button _playbackPlayPauseButton;
    private TextMeshProUGUI _playbackPlayPauseLabel;
    private Button _playbackReplayButton;
    private Slider _playbackTimeline;
    private TextMeshProUGUI _playbackTime;
    private readonly List<Button> _playbackSpeedButtons = new List<Button>();
    private readonly List<UnityAction> _playbackSpeedActions = new List<UnityAction>();
    private readonly float[] _playbackSpeeds = { 0.25f, 0.5f, 1f, 2f };
    private readonly HashSet<int> _playbackTimelinePointers = new HashSet<int>();
    private Slider _pathThicknessSlider;
    private TextMeshProUGUI _pathThicknessValue;
    private const float MinimumPathThickness = 0.005f;
    private const float MaximumPathThickness = 0.05f;
    private TextMeshProUGUI _alignmentStatus;
    private readonly List<Button> _translationStepButtons = new List<Button>();
    private readonly List<Button> _rotationStepButtons = new List<Button>();
    private readonly List<Button> _alignmentActionButtons = new List<Button>();
    private readonly float[] _translationSteps = { 0.1f, 0.01f, 0.001f };
    private readonly float[] _rotationSteps = { 5f, 1f, 0.1f };
    private int _translationStepIndex = 1;
    private int _rotationStepIndex = 1;
    private Vector3 _alignmentRight = Vector3.right;
    private Vector3 _alignmentForward = Vector3.forward;
    private Transform _alignmentTrackingSpace;
    private Vector3 _alignmentRightInTracking;
    private Vector3 _alignmentForwardInTracking;
    private bool _ownsCanvas;
    private bool _hasPositioned;
    private Vector3 _lastPlacementForward = Vector3.forward;
    private XRSimpleInteractable _grabHandle;
    private Image _grabImage;
    private XRSelectFilterDelegate _grabFilter;
    private readonly HashSet<IXRHoverInteractor> _grabHovering = new HashSet<IXRHoverInteractor>();
    private IXRSelectInteractor _grabInteractor;
    private bool _draggingMenu;
    private Vector3 _grabPointLocal;
    private float _grabDistance;
    private readonly Color _grabColor = new Color(0.08f, 0.34f, 0.43f, 1f);
    private bool? _lastTcpMode;
    private bool? _lastQrValidated;
    private bool? _lastTargetRotation;
    private string _localNotice;

    public bool IsVisible => isActiveAndEnabled && _panel != null && _panel.gameObject.activeInHierarchy;

    private void Start()
    {
        BuildMenu();
        BindController();
        BindTaskControls();
        RefreshMenu();
        if (!_hasPositioned) RecenterMenu();
    }

    private void LateUpdate()
    {
        if (!IsVisible || _canvas == null) return;
        Camera viewer = Camera.main;
        if (viewer != null && RobotFrameMath.IsValidPose(new Pose(viewer.transform.position, viewer.transform.rotation)))
        {
            _canvas.worldCamera = viewer;
            if (!_hasPositioned) RecenterMenu();
            FaceCamera(viewer.transform);
        }
        if (!_draggingMenu || _grabInteractor == null) return;
        if (!InteractorIsTracked(_grabInteractor) || !TryInteractorPose(_grabInteractor, out Pose pointer))
        {
            CancelMenuGrab();
            return;
        }
        MoveMenuDrag(pointer);
    }

    private Vector3 InitialPosition(Transform camera)
    {
        Vector3 horizontal = Vector3.ProjectOnPlane(camera.forward, Vector3.up);
        if (horizontal.sqrMagnitude > 0.000001f) _lastPlacementForward = horizontal.normalized;
        Vector3 direction = Quaternion.AngleAxis(_initialSideAngle, Vector3.up) * _lastPlacementForward;
        return camera.position + direction * Mathf.Max(0.6f, _initialDistance) + Vector3.up * _initialHeightOffset;
    }

    private bool BeginMenuDrag(Pose pointer, Vector3 grabbedWorldPoint)
    {
        if (!IsVisible || _canvas == null || _draggingMenu || !RobotFrameMath.IsValidPose(pointer) ||
            !RobotFrameMath.IsValidPose(new Pose(grabbedWorldPoint, Quaternion.identity))) return false;
        _grabDistance = Vector3.Dot(grabbedWorldPoint - pointer.position, pointer.rotation * Vector3.forward);
        if (_grabDistance < 0.05f || float.IsInfinity(_grabDistance)) return false;
        _grabPointLocal = _canvas.transform.InverseTransformPoint(grabbedWorldPoint);
        _draggingMenu = true;
        _hasPositioned = true;
        RefreshHoverState();
        return true;
    }

    private void MoveMenuDrag(Pose pointer)
    {
        if (!_draggingMenu || !RobotFrameMath.IsValidPose(pointer)) return;
        Vector3 grabbedWorldPoint = pointer.position + pointer.rotation * Vector3.forward * _grabDistance;
        // Keep the selected point under the ray while the panel keeps facing the viewer.
        // Updating the offset as we turn prevents a corner grab from snapping to the centre.
        Camera viewer = Camera.main;
        for (int pass = 0; pass < 4; pass++)
        {
            _canvas.transform.position = grabbedWorldPoint - _canvas.transform.TransformVector(_grabPointLocal);
            if (viewer != null) FaceCamera(viewer.transform);
        }
    }

    private void EndMenuDrag()
    {
        _draggingMenu = false;
        _grabInteractor = null;
        RefreshHoverState();
    }

    private void CancelMenuGrab()
    {
        EndMenuDrag();
        if (_grabHandle != null && _grabHandle.interactionManager != null)
            _grabHandle.interactionManager.CancelInteractableSelection((IXRSelectInteractable)_grabHandle);
    }

    private static bool InteractorIsTracked(IXRSelectInteractor interactor)
    {
        return !(interactor is Behaviour behaviour) || (behaviour != null && behaviour.isActiveAndEnabled);
    }

    private bool TryInteractorPose(IXRSelectInteractor interactor, out Pose pointer)
    {
        Transform origin;
        if (interactor is XRRayInteractor ray)
            origin = ray.rayOriginTransform != null ? ray.rayOriginTransform : ray.transform;
        else origin = interactor.GetAttachTransform(_grabHandle);
        pointer = origin != null ? new Pose(origin.position, origin.rotation) : default;
        return origin != null && RobotFrameMath.IsValidPose(pointer);
    }

    private void GrabEntered(SelectEnterEventArgs args)
    {
        if (_grabInteractor != null || !TryInteractorPose(args.interactorObject, out Pose pointer)) return;
        Vector3 point = _grabHandle.GetComponent<BoxCollider>().bounds.center;
        if (args.interactorObject is XRRayInteractor ray && ray.TryGetCurrent3DRaycastHit(out RaycastHit hit)) point = hit.point;
        if (!BeginMenuDrag(pointer, point)) return;
        _grabInteractor = args.interactorObject;
    }

    private void GrabExited(SelectExitEventArgs args)
    {
        if (args.interactorObject == _grabInteractor) EndMenuDrag();
    }

    private void GrabHoverEntered(HoverEnterEventArgs args)
    {
        if (!isActiveAndEnabled) return;
        _grabHovering.Add(args.interactorObject);
        RefreshHoverState();
    }

    private void GrabHoverExited(HoverExitEventArgs args)
    {
        _grabHovering.Remove(args.interactorObject);
        RefreshHoverState();
    }

    private void RefreshHoverState()
    {
        bool engaged = isActiveAndEnabled && (_hoveringPointers.Count != 0 || _pressedPointers.Count != 0 ||
            _grabHovering.Count != 0 || _draggingMenu);
        if (engaged) HoveredMenus.Add(this);
        else HoveredMenus.Remove(this);
        if (_grabImage != null)
            _grabImage.color = Color.Lerp(_grabColor, Color.white, _draggingMenu ? 0.5f : _grabHovering.Count != 0 ? 0.25f : 0f);
    }

    private void Update()
    {
        if (_controller == null) BindController();
        if (_input == null || _tcpPlanning == null || _qrDetector == null || _instantiator == null) BindTaskControls();
        bool tcp = _input == null || _input.IsTcpMode;
        bool qr = _qrDetector != null && _qrDetector.HasValidatedPose;
        if (_lastTcpMode != tcp || _lastQrValidated != qr || _lastTargetRotation != TargetRotationHandles.IsInteracting) RefreshTask();
        RefreshPlayback();
        RefreshPathThickness();
    }

    private void RefreshPlayback()
    {
        if (_playbackTimeline == null) return;
        if (_trajectoryVisualizer == null) _trajectoryVisualizer = FindFirstObjectByType<UR16eTrajectoryVisualizer>();
        bool available = _trajectoryVisualizer != null && _trajectoryVisualizer.HasPlayback && !IsCalibrating;
        _playbackPlayPauseButton.interactable = available;
        _playbackReplayButton.interactable = available;
        _playbackTimeline.interactable = available;
        _playbackPlayPauseLabel.text = available && _trajectoryVisualizer.IsPlaying ? "Pause" : "Play";
        _playbackTime.text = available
            ? _trajectoryVisualizer.PlaybackTimeSeconds.ToString("0.0") + " / " + _trajectoryVisualizer.DurationSeconds.ToString("0.0") + " s"
            : "Plan to enable";
        if (!available) _playbackTimelinePointers.Clear();
        if (_playbackTimelinePointers.Count == 0)
            _playbackTimeline.SetValueWithoutNotify(available ? _trajectoryVisualizer.PlaybackNormalized : 0f);
        for (int index = 0; index < _playbackSpeedButtons.Count; index++)
        {
            Button button = _playbackSpeedButtons[index];
            button.interactable = available;
            bool selected = _trajectoryVisualizer != null && Mathf.Approximately(_trajectoryVisualizer.PlaybackSpeed, _playbackSpeeds[index]);
            button.image.color = selected
                ? new Color(0.08f, 0.42f, 0.52f, 1f) : new Color(0.12f, 0.17f, 0.23f, 1f);
        }
    }

    private void TogglePlayback()
    {
        if (_trajectoryVisualizer == null || !_trajectoryVisualizer.HasPlayback || IsCalibrating) return;
        if (_trajectoryVisualizer.IsPlaying) _trajectoryVisualizer.Pause();
        else _trajectoryVisualizer.Play();
        RefreshPlayback();
    }

    private void ReplayPlayback()
    {
        if (_trajectoryVisualizer == null || !_trajectoryVisualizer.HasPlayback || IsCalibrating) return;
        _trajectoryVisualizer.Replay();
        RefreshPlayback();
    }

    private void SeekPlayback(float normalized)
    {
        if (_trajectoryVisualizer == null || !_trajectoryVisualizer.HasPlayback || IsCalibrating) return;
        _trajectoryVisualizer.Seek(normalized);
        RefreshPlayback();
    }

    private void SelectPlaybackSpeed(float speed)
    {
        if (_trajectoryVisualizer == null || !_trajectoryVisualizer.HasPlayback || IsCalibrating) return;
        _trajectoryVisualizer.SetPlaybackSpeed(speed);
        RefreshPlayback();
    }

    private void RefreshPathThickness()
    {
        if (_pathThicknessSlider == null) return;
        if (_trajectoryVisualizer == null) _trajectoryVisualizer = FindFirstObjectByType<UR16eTrajectoryVisualizer>();
        LineRenderer path = _trajectoryVisualizer != null ? _trajectoryVisualizer.trajectoryLineRenderer : null;
        _pathThicknessSlider.interactable = path != null;
        if (path == null) return;
        float width = Mathf.Clamp(path.widthMultiplier, MinimumPathThickness, MaximumPathThickness);
        _pathThicknessSlider.SetValueWithoutNotify(width);
        _pathThicknessValue.text = Mathf.RoundToInt(width * 1000f) + " mm";
    }

    private void SetPathThickness(float width)
    {
        if (_trajectoryVisualizer == null || _trajectoryVisualizer.trajectoryLineRenderer == null ||
            float.IsNaN(width) || float.IsInfinity(width)) return;
        _trajectoryVisualizer.trajectoryLineRenderer.widthMultiplier = Mathf.Clamp(width, MinimumPathThickness, MaximumPathThickness);
        RefreshPathThickness();
    }

    public void SelectTool(string tool)
    {
        if (_instantiator != null && _instantiator.IsCalibrating) return;
        if (tool != "none" && tool != "kenos" && tool != "kenos_sl") return;
        BindController();
        if (_controller != null) _controller.SelectTool(tool);
        RefreshMenu();
    }

    public void RefreshMenu()
    {
        string selected = _controller != null ? _controller.SelectedTool : string.Empty;
        foreach (Choice choice in _choices)
        {
            bool active = choice.Tool == selected;
            choice.Button.interactable = _controller != null && !IsCalibrating;
            choice.Button.image.color = active
                ? new Color(0.08f, 0.42f, 0.52f, 1f) : new Color(0.12f, 0.17f, 0.23f, 1f);
            choice.Border.effectColor = active
                ? new Color(0.4f, 0.9f, 1f, 1f) : new Color(0.3f, 0.37f, 0.44f, 0.8f);
            choice.Label.text = active
                ? choice.Title + "\n<size=11><color=#B2EDF8>Selected</color></size>" : choice.Title;
        }
        if (_status != null)
        {
            _status.text = _controller != null
                ? _controller.StatusText : "Tool profiles are not available yet.";
        }
        RefreshTask();
    }

    public void PlanTask()
    {
        BindTaskControls();
        if (IsCalibrating) return;
        _localNotice = null;
        if (_input != null)
        {
            _input.PlanCurrentTask();
        }
        else if (_tcpPlanning != null) _tcpPlanning.Plan();
        else _localNotice = "Planning controls are unavailable. Reopen the robot interaction scene.";
        RefreshTask();
    }

    public void SelectTcpTask() => SelectTaskMode(true);
    public void SelectPickPlaceTask() => SelectTaskMode(false);

    private void SelectTaskMode(bool tcp)
    {
        BindTaskControls();
        if (IsCalibrating || TargetRotationHandles.IsInteracting) return;
        if (_input == null)
        {
            _localNotice = "Task controls are unavailable. Open the robot interaction scene.";
            RefreshTask();
            return;
        }
        if (tcp) _input.UseTcpPreview();
        else _input.UseTilePickPlace();
        _localNotice = null;
        RefreshTask();
    }

    public void ExecuteTask()
    {
        BindTaskControls();
        if (IsCalibrating) return;
        if (_input != null) _input.ExecuteCurrentTask();
        else if (_tcpPlanning != null) _tcpPlanning.Execute();
        RefreshTask();
    }

    public void RescanQr()
    {
        BindTaskControls();
        if (IsCalibrating) return;
        if (_qrDetector != null) _qrDetector.Rescan();
        RefreshTask();
    }

    private bool IsCalibrating => _instantiator != null && _instantiator.IsCalibrating;

    public void BeginAlignment()
    {
        BindTaskControls();
        _localNotice = null;
        if (_instantiator == null)
        {
            _localNotice = "Place the virtual robot from its QR before aligning it.";
            RefreshTask();
            return;
        }
        if (!IsCalibrating) CaptureAlignmentFrame();
        _instantiator.BeginCalibration();
        if (!IsCalibrating) _localNotice = _instantiator.AlignmentStatus;
        RefreshMenu();
    }

    public void SaveAlignment()
    {
        if (!IsCalibrating) return;
        _instantiator.SaveCalibration();
        if (!IsCalibrating) _localNotice = _instantiator.AlignmentStatus;
        RefreshMenu();
    }

    public void CancelAlignment()
    {
        if (!IsCalibrating) return;
        _instantiator.CancelCalibration();
        _localNotice = _instantiator.AlignmentStatus;
        RefreshMenu();
    }

    public void ResetAlignment()
    {
        if (!IsCalibrating) return;
        _instantiator.RestoreDefaultCalibration();
        RefreshAlignmentControls();
    }

    public void SetTranslationStep(int index)
    {
        if (index < 0 || index >= _translationSteps.Length) return;
        _translationStepIndex = index;
        RefreshAlignmentControls();
    }

    public void SetRotationStep(int index)
    {
        if (index < 0 || index >= _rotationSteps.Length) return;
        _rotationStepIndex = index;
        RefreshAlignmentControls();
    }

    private void CaptureAlignmentFrame()
    {
        Transform camera = Camera.main != null ? Camera.main.transform : null;
        Vector3 forward = camera != null ? camera.forward : _canvas != null ? _canvas.transform.forward : Vector3.forward;
        forward = Vector3.ProjectOnPlane(forward, Vector3.up);
        if (forward.sqrMagnitude > 0.000001f)
        {
            _alignmentForward = forward.normalized;
            _alignmentRight = Vector3.Cross(Vector3.up, _alignmentForward).normalized;
        }
        else
        {
            Vector3 right = Vector3.ProjectOnPlane(camera != null ? camera.right : Vector3.right, Vector3.up);
            _alignmentRight = right.sqrMagnitude > 0.000001f ? right.normalized : Vector3.right;
            _alignmentForward = Vector3.Cross(_alignmentRight, Vector3.up).normalized;
        }
        // Retain physical starting-view axes across an XR Origin recenter, while
        // later head turns do not redefine left/right or towards/away.
        _alignmentTrackingSpace = _qrDetector != null ? _qrDetector.TrackingSpace : null;
        if (RobotFrameMath.IsRigidFrame(_alignmentTrackingSpace))
        {
            _alignmentRightInTracking = _alignmentTrackingSpace.InverseTransformDirection(_alignmentRight);
            _alignmentForwardInTracking = _alignmentTrackingSpace.InverseTransformDirection(_alignmentForward);
        }
        else _alignmentTrackingSpace = null;
    }

    private Vector3 AlignmentRightWorld => _alignmentTrackingSpace != null
        ? _alignmentTrackingSpace.TransformDirection(_alignmentRightInTracking).normalized : _alignmentRight;
    private Vector3 AlignmentForwardWorld => _alignmentTrackingSpace != null
        ? _alignmentTrackingSpace.TransformDirection(_alignmentForwardInTracking).normalized : _alignmentForward;

    private void MoveAlignment(Vector3 direction)
    {
        if (IsCalibrating) _instantiator.NudgePosition(direction * _translationSteps[_translationStepIndex]);
    }

    private void RotateAlignment(Vector3 axis, float sign)
    {
        if (IsCalibrating) _instantiator.RotateBase(axis, sign * _rotationSteps[_rotationStepIndex]);
    }

    private void OnCalibrationStateChanged()
    {
        if (!IsCalibrating && _instantiator != null)
            _localNotice = _instantiator.AlignmentStatus;
        RefreshMenu();
    }

    private void OnPlanningStateChanged()
    {
        _localNotice = null;
        RefreshTask();
    }

    private void BindTaskControls()
    {
        if (_input == null) _input = FindFirstObjectByType<URInputManager>();
        if (_tcpPlanning == null) _tcpPlanning = _input != null ? _input.GetComponent<TcpPlanningController>() : FindFirstObjectByType<TcpPlanningController>();
        if (_qrDetector == null) _qrDetector = FindFirstObjectByType<QRDetector>();
        if (_instantiator == null) _instantiator = FindFirstObjectByType<URInstantiator>();
        if (_subscribedInstantiator != _instantiator)
        {
            if (_subscribedInstantiator != null) _subscribedInstantiator.CalibrationStateChanged -= OnCalibrationStateChanged;
            _subscribedInstantiator = _instantiator;
            if (_subscribedInstantiator != null) _subscribedInstantiator.CalibrationStateChanged += OnCalibrationStateChanged;
            if (_instantiator != null && !IsCalibrating && _instantiator.AlignmentStatus != null &&
                _instantiator.AlignmentStatus.StartsWith("Saved alignment could not", System.StringComparison.Ordinal))
                _localNotice = _instantiator.AlignmentStatus;
        }
        if (_subscribedQr != _qrDetector)
        {
            if (_subscribedQr != null) _subscribedQr.StateChanged -= RefreshTask;
            _subscribedQr = _qrDetector;
            if (_subscribedQr != null) _subscribedQr.StateChanged += RefreshTask;
        }
        if (_subscribedPlanning != _tcpPlanning)
        {
            if (_subscribedPlanning != null) _subscribedPlanning.StateChanged -= OnPlanningStateChanged;
            _subscribedPlanning = _tcpPlanning;
            if (_subscribedPlanning != null) _subscribedPlanning.StateChanged += OnPlanningStateChanged;
        }
        if (_legacyPlans == null)
        {
            _legacyPlans = FindFirstObjectByType<MTCPlansManager>();
            if (_legacyPlans != null) _legacyPlans.OnSuccessfulPlan.AddListener(OnLegacyPlanResult);
        }
    }

    private void OnLegacyPlanResult(bool success)
    {
        _localNotice = _legacyPlans != null && _legacyPlans.HasExecutionFeedback ? _legacyPlans.StatusText : null;
        RefreshTask();
    }

    private void RefreshTask()
    {
        bool tcp = _input == null || _input.IsTcpMode;
        MTCPlansManager execution = MTCPlansManager.ActiveExecution;
        _lastTcpMode = tcp;
        _lastQrValidated = _qrDetector != null && _qrDetector.HasValidatedPose;
        _lastTargetRotation = TargetRotationHandles.IsInteracting;
        bool canSelectMode = _input != null && !IsCalibrating && !TargetRotationHandles.IsInteracting;
        RefreshTaskChoice(_tcpTaskButton, tcp, canSelectMode);
        RefreshTaskChoice(_pickPlaceTaskButton, !tcp, canSelectMode);
        if (_taskHelp != null) _taskHelp.text = tcp
            ? "Hold bumper: TCP A. Double press: TCP B.\nPlace the tool preview; drag it to set exact orientation."
            : "Hold/double bumper: pick/place contact balls.\nArrow = normal. Drag ball to turn; bumper: depth ±5 mm.";
        if (_taskStatus != null)
            _taskStatus.text = execution != null ? execution.StatusText : _localNotice ?? (tcp
                ? (_tcpPlanning != null ? _tcpPlanning.StatusText : "Capture TCP A and B to prepare a preview.")
                : (_legacyPlans != null ? _legacyPlans.StatusText : "Capture pick and place tool contacts, then Plan preview."));
        if (_planButton != null) _planButton.interactable = !IsCalibrating && execution == null;
        if (_executeButton != null) _executeButton.interactable = !IsCalibrating && execution == null && (tcp
            ? _tcpPlanning != null && _tcpPlanning.CanExecute
            : _legacyPlans != null && _legacyPlans.CanExecute && ToolSelectionController.CanPlanPickPlace);
        if (_rescanButton != null) _rescanButton.interactable = _qrDetector != null && !IsCalibrating;
        // Keep the entry usable so the core can explain which placement is missing.
        if (_alignButton != null) _alignButton.interactable = !IsCalibrating;
        if (_qrStatus != null)
        {
            string qr = _qrDetector != null ? _qrDetector.StatusText : "QR scanner unavailable.";
            if (_qrDetector != null)
            {
                qr += "\nMarker: " + (_qrDetector.MarkerSizeMeters * 1000f).ToString("0.#") + " mm";
                qr += string.IsNullOrEmpty(_qrDetector.ExpectedMarkerPayload) ? " · ID not pinned" : " · expected " + ShortPayload(_qrDetector.ExpectedMarkerPayload);
                if (!string.IsNullOrEmpty(_qrDetector.DetectedPayload)) qr += "\nDetected: " + ShortPayload(_qrDetector.DetectedPayload);
            }
            if (_instantiator != null) qr += "\n" + _instantiator.CalibrationSummary;
            if (_instantiator != null && _instantiator.HasSavedCalibration) qr += "\nSaved on this headset for this QR.";
            _qrStatus.text = qr;
        }
        RefreshPlayback();
        RefreshAlignmentControls();
    }

    private static void RefreshTaskChoice(Button button, bool selected, bool enabled)
    {
        if (button == null) return;
        button.interactable = enabled;
        button.image.color = selected ? new Color(0.08f, 0.42f, 0.52f, 1f) : new Color(0.12f, 0.17f, 0.23f, 1f);
    }

    private void RefreshAlignmentControls()
    {
        bool editing = IsCalibrating;
        if (_taskView != null) _taskView.gameObject.SetActive(!editing);
        if (_alignmentView != null) _alignmentView.gameObject.SetActive(editing);
        if (_alignmentStatus != null)
            _alignmentStatus.text = _instantiator != null
                ? "QR: " + (_qrDetector != null ? ShortPayload(_qrDetector.DetectedPayload, 26) : "unknown") +
                    " · " + (_instantiator.HasSavedCalibration ? "saved on headset" : "new on headset") +
                    "\n" + _instantiator.AlignmentStatus + "\n" + _instantiator.CalibrationSummary
                : "Scan the robot QR and place the virtual robot first.";
        RefreshStepButtons(_translationStepButtons, _translationStepIndex, editing);
        RefreshStepButtons(_rotationStepButtons, _rotationStepIndex, editing);
        foreach (Button button in _alignmentActionButtons)
            if (button != null) button.interactable = editing;
    }

    private static void RefreshStepButtons(List<Button> buttons, int selectedIndex, bool editing)
    {
        for (int index = 0; index < buttons.Count; index++)
        {
            buttons[index].interactable = editing;
            buttons[index].image.color = index == selectedIndex
                ? new Color(0.08f, 0.42f, 0.52f, 1f) : new Color(0.12f, 0.17f, 0.23f, 1f);
        }
    }

    private static string ShortPayload(string value, int maximum = 55)
    {
        string oneLine = value.Replace('\r', ' ').Replace('\n', ' ');
        return oneLine.Length > maximum ? oneLine.Substring(0, maximum - 1) + "…" : oneLine;
    }

    public void RecenterMenu()
    {
        if (_panel == null) BuildMenu();
        if (_canvas == null || Camera.main == null) return;
        CancelMenuGrab();
        Transform camera = Camera.main.transform;
        if (!RobotFrameMath.IsValidPose(new Pose(camera.position, camera.rotation))) return;
        _canvas.transform.position = InitialPosition(camera);
        FaceCamera(camera);
        _canvas.worldCamera = Camera.main;
        _hasPositioned = true;
    }

    /// <summary>Robot placement never relocates a menu the viewer has already placed.</summary>
    public void PositionAt(Pose baseWorld)
    {
        if (IsCalibrating) return;
        if (_panel == null) BuildMenu();
        if (_canvas == null || !RobotFrameMath.IsValidPose(baseWorld)) return;
        if (_hasPositioned || Camera.main != null)
        {
            if (!_hasPositioned) RecenterMenu();
            RefreshTask();
            return;
        }
        _canvas.transform.position = baseWorld.position + Vector3.up * 0.65f + baseWorld.rotation * Vector3.right * 0.35f;
        if (Camera.main != null)
        {
            FaceCamera(Camera.main.transform);
            _canvas.worldCamera = Camera.main;
        }
        else
        {
            _canvas.transform.rotation = baseWorld.rotation;
        }
        _hasPositioned = true;
        RefreshTask();
    }

    private void FaceCamera(Transform camera)
    {
        if (TryFacingRotation(camera, out Quaternion facing)) _canvas.transform.rotation = facing;
    }

    private bool TryFacingRotation(Transform camera, out Quaternion facing)
    {
        facing = _canvas.transform.rotation;
        Vector3 awayFromCamera = _canvas.transform.position - camera.position;
        if (awayFromCamera.sqrMagnitude < 0.0001f ||
            Vector3.ProjectOnPlane(awayFromCamera, Vector3.up).sqrMagnitude < 0.000001f) return false;
        facing = Quaternion.LookRotation(awayFromCamera, Vector3.up);
        return true;
    }

    private void BuildMenu()
    {
        if (_panel != null) return;
        if (_canvas == null)
        {
            GameObject canvasObject = new GameObject("Robot tool menu", typeof(RectTransform), typeof(Canvas));
            canvasObject.layer = 0;
            canvasObject.transform.SetParent(transform, false);
            _canvas = canvasObject.GetComponent<Canvas>();
            _ownsCanvas = true;
        }
        _canvas.renderMode = RenderMode.WorldSpace;
        // Stay where the user leaves it; head movement only changes facing.
        _canvas.transform.SetParent(null, true);
        RectTransform canvasRect = (RectTransform)_canvas.transform;
        canvasRect.sizeDelta = new Vector2(440f, 756f);
        canvasRect.localScale = Vector3.one * 0.001f;
        CanvasScaler scaler = _canvas.GetComponent<CanvasScaler>();
        if (scaler == null) scaler = _canvas.gameObject.AddComponent<CanvasScaler>();
        scaler.dynamicPixelsPerUnit = 10f;
        if (_canvas.GetComponent<TrackedDeviceGraphicRaycaster>() == null)
            _canvas.gameObject.AddComponent<TrackedDeviceGraphicRaycaster>();

        _panel = Rect("Robot task controls", canvasRect, Vector2.zero, new Vector2(440f, 756f));
        _panel.anchorMin = _panel.anchorMax = new Vector2(0.5f, 0.5f);
        _panel.pivot = new Vector2(0.5f, 0.5f);
        Image background = _panel.gameObject.AddComponent<Image>();
        background.color = new Color(0.035f, 0.06f, 0.09f, 0.97f);
        background.raycastTarget = true;
        Outline panelBorder = _panel.gameObject.AddComponent<Outline>();
        panelBorder.effectColor = new Color(0.2f, 0.44f, 0.53f, 0.8f);
        panelBorder.effectDistance = new Vector2(1f, -1f);
        AddHoverTracking(_panel.gameObject);

        _taskView = Rect("Task view", _panel, Vector2.zero, new Vector2(440f, 756f));
        _alignmentView = Rect("Alignment view", _panel, Vector2.zero, new Vector2(440f, 756f));
        BuildHeader(_taskView, "Title", "Robot tool");
        Text("Subtitle", _taskView, new Vector2(16f, -44f), new Vector2(408f, 29f),
            "Match the mounted tool and ROS setup.", 14f).color = new Color(0.73f, 0.81f, 0.87f);
        CreateChoice("none", "No tool", 16f);
        CreateChoice("kenos", "Kenos", 155f);
        CreateChoice("kenos_sl", "Kenos SL", 294f);
        _status = Text("Tool status", _taskView, new Vector2(16f, -138f), new Vector2(408f, 55f), string.Empty, 15f);
        _status.color = new Color(0.83f, 0.9f, 0.94f);
        _tcpTaskButton = CreateButton("TCP A → B", _taskView, new Vector2(16f, -204f), new Vector2(198f, 32f), out TextMeshProUGUI tcpLabel);
        tcpLabel.fontSize = 16f;
        _tcpTaskButton.onClick.AddListener(SelectTcpTask);
        _pickPlaceTaskButton = CreateButton("MTC pick / place", _taskView, new Vector2(226f, -204f), new Vector2(198f, 32f), out TextMeshProUGUI pickPlaceLabel);
        pickPlaceLabel.fontSize = 16f;
        _pickPlaceTaskButton.onClick.AddListener(SelectPickPlaceTask);
        _taskHelp = Text("Target instructions", _taskView, new Vector2(16f, -239f), new Vector2(408f, 45f), string.Empty, 14f);
        _taskHelp.color = new Color(0.73f, 0.81f, 0.87f);
        _taskStatus = Text("Task request status", _taskView, new Vector2(16f, -292f), new Vector2(408f, 94f), string.Empty, 17f);
        _taskStatus.richText = false;
        _taskStatus.enableAutoSizing = true;
        _taskStatus.fontSizeMin = 14f;
        _taskStatus.fontSizeMax = 17f;
        _planButton = CreateButton("Plan preview", _taskView, new Vector2(16f, -394f), new Vector2(198f, 42f), out _);
        _planButton.onClick.AddListener(PlanTask);
        _executeButton = CreateButton("Execute plan", _taskView, new Vector2(226f, -394f), new Vector2(198f, 42f), out _);
        _executeButton.onClick.AddListener(ExecuteTask);
        BuildPlaybackControls();
        BuildPathThicknessSlider();
        _recenterButton = CreateButton("Recenter menu", _taskView, new Vector2(16f, -625f), new Vector2(130f, 28f), out TextMeshProUGUI recenterLabel);
        recenterLabel.text = "Recenter menu";
        recenterLabel.fontSize = 13f;
        _recenterButton.onClick.AddListener(RecenterMenu);
        _rescanButton = CreateButton("Rescan QR", _taskView, new Vector2(155f, -625f), new Vector2(130f, 28f), out TextMeshProUGUI rescanLabel);
        rescanLabel.fontSize = 13f;
        _rescanButton.onClick.AddListener(RescanQr);
        _alignButton = CreateButton("Align robot", _taskView, new Vector2(294f, -625f), new Vector2(130f, 28f), out TextMeshProUGUI alignLabel);
        alignLabel.fontSize = 13f;
        _alignButton.onClick.AddListener(BeginAlignment);
        _qrStatus = Text("QR placement status", _taskView, new Vector2(16f, -663f), new Vector2(408f, 80f), string.Empty, 12f);
        _qrStatus.color = new Color(0.65f, 0.74f, 0.8f);
        _qrStatus.richText = false;
        _qrStatus.enableAutoSizing = true;
        _qrStatus.fontSizeMin = 10f;
        _qrStatus.fontSizeMax = 12f;
        BuildAlignmentView();
        BuildGrabHandle(canvasRect);
        RefreshAlignmentControls();
        _panel.gameObject.SetActive(isActiveAndEnabled);
    }

    private void BuildPlaybackControls()
    {
        Text("Arm preview heading", _taskView, new Vector2(16f, -448f), new Vector2(236f, 24f),
            "Arm + tool preview", 14f).color = new Color(0.73f, 0.81f, 0.87f);
        _playbackTime = Text("Arm preview time", _taskView, new Vector2(254f, -448f), new Vector2(170f, 24f),
            "Plan to enable", 13f);
        _playbackTime.alignment = TextAlignmentOptions.MidlineRight;
        _playbackPlayPauseButton = CreateButton("Play preview", _taskView, new Vector2(16f, -478f),
            new Vector2(82f, 30f), out _playbackPlayPauseLabel);
        _playbackPlayPauseLabel.text = "Play";
        _playbackPlayPauseLabel.fontSize = 14f;
        _playbackPlayPauseButton.onClick.AddListener(TogglePlayback);
        _playbackReplayButton = CreateButton("Replay preview", _taskView, new Vector2(106f, -478f),
            new Vector2(82f, 30f), out TextMeshProUGUI replayLabel);
        replayLabel.text = "Replay";
        replayLabel.fontSize = 14f;
        _playbackReplayButton.onClick.AddListener(ReplayPlayback);
        for (int index = 0; index < _playbackSpeeds.Length; index++)
        {
            float speed = _playbackSpeeds[index];
            string caption = speed.ToString("0.##") + "×";
            Button button = CreateButton("Preview speed " + caption, _taskView,
                new Vector2(196f + index * 59f, -478f), new Vector2(51f, 30f), out TextMeshProUGUI speedLabel);
            speedLabel.text = caption;
            speedLabel.fontSize = 12f;
            UnityAction click = () => SelectPlaybackSpeed(speed);
            button.onClick.AddListener(click);
            _playbackSpeedActions.Add(click);
            _playbackSpeedButtons.Add(button);
        }

        RectTransform root = Rect("Arm preview timeline", _taskView, new Vector2(16f, -516f), new Vector2(408f, 30f));
        root.gameObject.AddComponent<Image>().color = new Color(0f, 0f, 0f, 0f);
        RectTransform track = Rect("Preview track", root, new Vector2(14f, -12f), new Vector2(380f, 6f));
        track.gameObject.AddComponent<Image>().color = new Color(0.15f, 0.23f, 0.29f);
        RectTransform fillArea = Rect("Preview fill area", root, new Vector2(14f, -12f), new Vector2(380f, 6f));
        RectTransform fill = Rect("Preview fill", fillArea, Vector2.zero, Vector2.zero);
        fill.anchorMin = Vector2.zero; fill.anchorMax = Vector2.one;
        fill.pivot = new Vector2(0f, 0.5f);
        fill.gameObject.AddComponent<Image>().color = new Color(0.08f, 0.56f, 0.68f);
        RectTransform handleArea = Rect("Preview handle area", root, new Vector2(14f, 0f), new Vector2(380f, 30f));
        RectTransform handle = Rect("Preview handle", handleArea, Vector2.zero, new Vector2(24f, -6f));
        handle.anchorMin = Vector2.zero; handle.anchorMax = Vector2.up;
        handle.pivot = new Vector2(0.5f, 0.5f);
        Image handleImage = handle.gameObject.AddComponent<Image>();
        handleImage.color = new Color(0.55f, 0.91f, 1f);
        _playbackTimeline = root.gameObject.AddComponent<Slider>();
        _playbackTimeline.direction = Slider.Direction.LeftToRight;
        _playbackTimeline.minValue = 0f;
        _playbackTimeline.maxValue = 1f;
        _playbackTimeline.fillRect = fill;
        _playbackTimeline.handleRect = handle;
        _playbackTimeline.targetGraphic = handleImage;
        Navigation navigation = _playbackTimeline.navigation;
        navigation.mode = Navigation.Mode.None;
        _playbackTimeline.navigation = navigation;
        _playbackTimeline.onValueChanged.AddListener(SeekPlayback);
        AddPressTracking(root.gameObject);
        EventTrigger trigger = root.GetComponent<EventTrigger>();
        EventTrigger.Entry pressed = new EventTrigger.Entry { eventID = EventTriggerType.PointerDown };
        pressed.callback.AddListener(data =>
        {
            if (!(data is PointerEventData pointer) || !_playbackTimeline.interactable) return;
            _playbackTimelinePointers.Add(pointer.pointerId);
            if (_trajectoryVisualizer != null) _trajectoryVisualizer.Pause();
        });
        EventTrigger.Entry released = new EventTrigger.Entry { eventID = EventTriggerType.PointerUp };
        released.callback.AddListener(data =>
        {
            if (data is PointerEventData pointer) _playbackTimelinePointers.Remove(pointer.pointerId);
        });
        trigger.triggers.Add(pressed);
        trigger.triggers.Add(released);
        RefreshPlayback();
    }

    private void BuildPathThicknessSlider()
    {
        Text("Ghost path thickness", _taskView, new Vector2(16f, -560f), new Vector2(300f, 24f),
            "Ghost path thickness", 14f).color = new Color(0.73f, 0.81f, 0.87f);
        _pathThicknessValue = Text("Ghost path thickness value", _taskView, new Vector2(326f, -560f),
            new Vector2(98f, 24f), "25 mm", 14f);
        _pathThicknessValue.alignment = TextAlignmentOptions.MidlineRight;
        RectTransform root = Rect("Ghost path thickness slider", _taskView, new Vector2(16f, -586f), new Vector2(408f, 30f));
        // The full row accepts the laser, including the space around the thin track.
        root.gameObject.AddComponent<Image>().color = new Color(0f, 0f, 0f, 0f);
        RectTransform track = Rect("Thickness track", root, new Vector2(14f, -12f), new Vector2(380f, 6f));
        track.gameObject.AddComponent<Image>().color = new Color(0.15f, 0.23f, 0.29f);
        RectTransform fillArea = Rect("Thickness fill area", root, new Vector2(14f, -12f), new Vector2(380f, 6f));
        RectTransform fill = Rect("Thickness fill", fillArea, Vector2.zero, Vector2.zero);
        fill.anchorMin = Vector2.zero; fill.anchorMax = Vector2.one;
        fill.pivot = new Vector2(0f, 0.5f);
        fill.gameObject.AddComponent<Image>().color = new Color(0.08f, 0.56f, 0.68f);
        RectTransform handleArea = Rect("Thickness handle area", root, new Vector2(14f, 0f), new Vector2(380f, 30f));
        RectTransform handle = Rect("Thickness handle", handleArea, Vector2.zero, new Vector2(24f, -6f));
        // Slider stretches the handle vertically; inset it to keep a square 24 mm grip.
        handle.anchorMin = Vector2.zero;
        handle.anchorMax = Vector2.up;
        handle.pivot = new Vector2(0.5f, 0.5f);
        Image handleImage = handle.gameObject.AddComponent<Image>();
        handleImage.color = new Color(0.55f, 0.91f, 1f);
        _pathThicknessSlider = root.gameObject.AddComponent<Slider>();
        _pathThicknessSlider.direction = Slider.Direction.LeftToRight;
        _pathThicknessSlider.minValue = MinimumPathThickness;
        _pathThicknessSlider.maxValue = MaximumPathThickness;
        _pathThicknessSlider.fillRect = fill;
        _pathThicknessSlider.handleRect = handle;
        _pathThicknessSlider.targetGraphic = handleImage;
        Navigation navigation = _pathThicknessSlider.navigation;
        navigation.mode = Navigation.Mode.None;
        _pathThicknessSlider.navigation = navigation;
        _pathThicknessSlider.SetValueWithoutNotify(0.025f);
        _pathThicknessSlider.onValueChanged.AddListener(SetPathThickness);
        AddPressTracking(root.gameObject);
        RefreshPathThickness();
    }

    private void BuildGrabHandle(RectTransform canvas)
    {
        // A separate strip above the panel leaves every UI button unobstructed.
        RectTransform bar = Rect("Grab menu - hold trigger and move", canvas, new Vector2(16f, 38f), new Vector2(408f, 30f));
        _grabImage = bar.gameObject.AddComponent<Image>();
        _grabImage.color = _grabColor;
        _grabImage.raycastTarget = false;
        TextMeshProUGUI hint = Text("Grab menu hint", bar, new Vector2(4f, -2f), new Vector2(400f, 26f),
            "Grab menu · hold trigger and move", 15f);
        hint.alignment = TextAlignmentOptions.Center;
        BoxCollider hit = bar.gameObject.AddComponent<BoxCollider>();
        hit.center = new Vector3(204f, -15f, 0f);
        hit.size = new Vector3(408f, 30f, 14f);
        _grabHandle = bar.gameObject.AddComponent<XRSimpleInteractable>();
        _grabHandle.selectMode = InteractableSelectMode.Single;
        _grabHandle.interactionLayers = 1;
        _grabHandle.colliders.Clear();
        _grabHandle.colliders.Add(hit);
        _grabFilter = new XRSelectFilterDelegate((interactor, target) =>
            isActiveAndEnabled && IsVisible && !TargetRotationHandles.IsInteracting &&
            (!_draggingMenu || _grabInteractor == interactor));
        _grabHandle.selectFilters.Add(_grabFilter);
        _grabHandle.hoverEntered.AddListener(GrabHoverEntered);
        _grabHandle.hoverExited.AddListener(GrabHoverExited);
        _grabHandle.selectEntered.AddListener(GrabEntered);
        _grabHandle.selectExited.AddListener(GrabExited);
        bar.gameObject.SetActive(isActiveAndEnabled);
    }

    private void BuildAlignmentView()
    {
        BuildHeader(_alignmentView, "Alignment title", "Align virtual robot");
        Text("Alignment help", _alignmentView, new Vector2(16f, -48f), new Vector2(408f, 40f),
            "Keep the real robot still; match its base and links.\nOnly the virtual overlay moves. Planning is paused.", 14f).color = new Color(0.73f, 0.81f, 0.87f);
        _alignmentStatus = Text("Alignment draft status", _alignmentView, new Vector2(16f, -97f), new Vector2(408f, 78f), string.Empty, 14f);
        _alignmentStatus.richText = false;
        _alignmentStatus.enableAutoSizing = true;
        _alignmentStatus.fontSizeMin = 10f;
        _alignmentStatus.fontSizeMax = 14f;

        Text("Move heading", _alignmentView, new Vector2(16f, -177f), new Vector2(194f, 27f), "Move position", 19f);
        CreateStepButtons(new[] { "100 mm", "10 mm", "1 mm" }, -176f, _translationStepButtons, SetTranslationStep);
        AlignmentButton("Left", new Vector2(16f, -214f), () => MoveAlignment(-AlignmentRightWorld));
        AlignmentButton("Up", new Vector2(155f, -214f), () => MoveAlignment(Vector3.up));
        AlignmentButton("Right", new Vector2(294f, -214f), () => MoveAlignment(AlignmentRightWorld));
        AlignmentButton("Towards you", new Vector2(16f, -258f), () => MoveAlignment(-AlignmentForwardWorld));
        AlignmentButton("Down", new Vector2(155f, -258f), () => MoveAlignment(Vector3.down));
        AlignmentButton("Away from you", new Vector2(294f, -258f), () => MoveAlignment(AlignmentForwardWorld));

        Text("Rotate heading", _alignmentView, new Vector2(16f, -305f), new Vector2(194f, 27f), "Rotate about base", 19f);
        CreateStepButtons(new[] { "5°", "1°", "0.1°" }, -304f, _rotationStepButtons, SetRotationStep);
        AlignmentButton("Yaw −", new Vector2(16f, -342f), () => RotateAlignment(Vector3.up, -1f));
        AlignmentButton("Pitch −", new Vector2(155f, -342f), () => RotateAlignment(AlignmentRightWorld, -1f));
        AlignmentButton("Roll −", new Vector2(294f, -342f), () => RotateAlignment(AlignmentForwardWorld, -1f));
        AlignmentButton("Yaw +", new Vector2(16f, -386f), () => RotateAlignment(Vector3.up, 1f));
        AlignmentButton("Pitch +", new Vector2(155f, -386f), () => RotateAlignment(AlignmentRightWorld, 1f));
        AlignmentButton("Roll +", new Vector2(294f, -386f), () => RotateAlignment(AlignmentForwardWorld, 1f));

        Text("Alignment frame hint", _alignmentView, new Vector2(16f, -432f), new Vector2(408f, 50f),
            "Moves follow your starting view; up/down is vertical.\nYaw: up; pitch: right; roll: forward. Pivot: base.\nSave keeps this QR's alignment on this headset.", 12f).color = new Color(0.73f, 0.81f, 0.87f);
        AlignmentButton("Save & finish", new Vector2(16f, -491f), SaveAlignment, new Vector2(198f, 42f));
        AlignmentButton("Cancel changes", new Vector2(226f, -491f), CancelAlignment, new Vector2(198f, 42f));
        AlignmentButton("Reset draft to prefab", new Vector2(16f, -542f), ResetAlignment, new Vector2(198f, 28f), 13f);
        AlignmentButton("Recenter alignment menu", new Vector2(226f, -542f), RecenterMenu, new Vector2(198f, 28f), 13f, "Recenter menu");
    }

    private void BuildHeader(Transform parent, string name, string title)
    {
        bool hasLogo = _appLogo != null;
        if (hasLogo)
        {
            RectTransform rect = Rect("Application logo", parent, new Vector2(16f, -5f), new Vector2(40f, 40f));
            RawImage logo = rect.gameObject.AddComponent<RawImage>();
            logo.texture = _appLogo;
            // The existing transparent artwork has a large empty outer border.
            // Sample a square around the arm without modifying or duplicating it.
            logo.uvRect = new Rect(0.268f, 0.179f, 0.554f, 0.554f);
            logo.raycastTarget = false;
        }
        Text(name, parent, new Vector2(hasLogo ? 66f : 16f, -12f),
            new Vector2(hasLogo ? 358f : 408f, 30f), title, 24f);
    }

    private void CreateStepButtons(string[] labels, float y, List<Button> buttons, System.Action<int> select)
    {
        for (int index = 0; index < labels.Length; index++)
        {
            int selected = index;
            Button button = CreateButton(labels[index] + " step", _alignmentView,
                new Vector2(216f + index * 72f, y), new Vector2(64f, 30f), out TextMeshProUGUI label);
            label.text = labels[index];
            label.fontSize = 13f;
            button.onClick.AddListener(() => select(selected));
            buttons.Add(button);
        }
    }

    private void AlignmentButton(string title, Vector2 position, UnityAction click,
        Vector2? size = null, float fontSize = 16f, string labelText = null)
    {
        Button button = CreateButton(title, _alignmentView, position, size ?? new Vector2(130f, 36f), out TextMeshProUGUI label);
        label.text = labelText ?? title;
        label.fontSize = fontSize;
        button.onClick.AddListener(click);
        _alignmentActionButtons.Add(button);
    }

    private void CreateChoice(string tool, string title, float x)
    {
        Button button = CreateButton(title, _taskView, new Vector2(x, -82f), new Vector2(130f, 44f), out TextMeshProUGUI label);
        label.fontSize = 18f;
        Outline border = button.gameObject.AddComponent<Outline>();
        border.effectDistance = new Vector2(1.5f, -1.5f);
        Choice choice = new Choice { Tool = tool, Title = title, Button = button, Label = label, Border = border };
        choice.Click = () => SelectTool(tool);
        button.onClick.AddListener(choice.Click);
        _choices.Add(choice);
    }

    private Button CreateButton(string name, Transform parent, Vector2 position, Vector2 size, out TextMeshProUGUI label)
    {
        RectTransform rect = Rect(name, parent, position, size);
        Image image = rect.gameObject.AddComponent<Image>();
        image.color = new Color(0.12f, 0.17f, 0.23f, 1f);
        Button button = rect.gameObject.AddComponent<Button>();
        button.targetGraphic = image;
        ColorBlock colors = button.colors;
        colors.normalColor = Color.white;
        colors.highlightedColor = new Color(0.8f, 0.95f, 1f);
        colors.pressedColor = new Color(0.55f, 0.8f, 0.95f);
        colors.selectedColor = Color.white;
        colors.disabledColor = new Color(0.5f, 0.5f, 0.5f, 0.7f);
        button.colors = colors;
        Navigation navigation = button.navigation;
        navigation.mode = Navigation.Mode.None;
        button.navigation = navigation;
        label = Text(name + " label", rect, new Vector2(4f, -2f), size - new Vector2(8f, 4f), name, 16f);
        label.alignment = TextAlignmentOptions.Center;
        AddPressTracking(rect.gameObject);
        return button;
    }

    private static RectTransform Rect(string name, Transform parent, Vector2 position, Vector2 size)
    {
        GameObject child = new GameObject(name, typeof(RectTransform));
        child.layer = 0;
        RectTransform rect = (RectTransform)child.transform;
        rect.SetParent(parent, false);
        rect.anchorMin = rect.anchorMax = new Vector2(0f, 1f);
        rect.pivot = new Vector2(0f, 1f);
        rect.anchoredPosition = position;
        rect.sizeDelta = size;
        return rect;
    }

    private static TextMeshProUGUI Text(string name, Transform parent, Vector2 position, Vector2 size, string value, float fontSize)
    {
        RectTransform rect = Rect(name, parent, position, size);
        TextMeshProUGUI text = rect.gameObject.AddComponent<TextMeshProUGUI>();
        text.text = value;
        text.fontSize = fontSize;
        text.color = Color.white;
        text.alignment = TextAlignmentOptions.MidlineLeft;
        text.textWrappingMode = TextWrappingModes.Normal;
        text.overflowMode = TextOverflowModes.Truncate;
        text.raycastTarget = false;
        return text;
    }

    private void AddHoverTracking(GameObject panel)
    {
        EventTrigger trigger = panel.AddComponent<EventTrigger>();
        EventTrigger.Entry entered = new EventTrigger.Entry { eventID = EventTriggerType.PointerEnter };
        entered.callback.AddListener(data =>
        {
            if (!isActiveAndEnabled || !(data is PointerEventData pointer)) return;
            _hoveringPointers.Add(pointer.pointerId);
            HoveredMenus.Add(this);
        });
        EventTrigger.Entry exited = new EventTrigger.Entry { eventID = EventTriggerType.PointerExit };
        exited.callback.AddListener(data =>
        {
            if (data is PointerEventData pointer) _hoveringPointers.Remove(pointer.pointerId);
            RefreshHoverState();
        });
        trigger.triggers.Add(entered);
        trigger.triggers.Add(exited);
        AddPressTracking(panel);
    }

    private void AddPressTracking(GameObject target)
    {
        EventTrigger trigger = target.GetComponent<EventTrigger>();
        if (trigger == null) trigger = target.AddComponent<EventTrigger>();
        EventTrigger.Entry pressed = new EventTrigger.Entry { eventID = EventTriggerType.PointerDown };
        pressed.callback.AddListener(data =>
        {
            if (!isActiveAndEnabled || !(data is PointerEventData pointer)) return;
            _pressedPointers.Add(pointer.pointerId);
            HoveredMenus.Add(this);
        });
        EventTrigger.Entry released = new EventTrigger.Entry { eventID = EventTriggerType.PointerUp };
        released.callback.AddListener(data =>
        {
            if (data is PointerEventData pointer) _pressedPointers.Remove(pointer.pointerId);
            RefreshHoverState();
        });
        trigger.triggers.Add(pressed);
        trigger.triggers.Add(released);
    }

    private void BindController()
    {
        if (_controller == null) _controller = ToolSelectionController.Instance;
        if (_subscribedController == _controller) return;
        Unsubscribe();
        _subscribedController = _controller;
        if (_subscribedController != null) _subscribedController.StateChanged += RefreshMenu;
        RefreshMenu();
    }

    private void Unsubscribe()
    {
        if (_subscribedController != null) _subscribedController.StateChanged -= RefreshMenu;
        _subscribedController = null;
    }

    private void UnsubscribeTaskControls()
    {
        if (_subscribedPlanning != null) _subscribedPlanning.StateChanged -= OnPlanningStateChanged;
        _subscribedPlanning = null;
        if (_subscribedQr != null) _subscribedQr.StateChanged -= RefreshTask;
        _subscribedQr = null;
        if (_legacyPlans != null) _legacyPlans.OnSuccessfulPlan.RemoveListener(OnLegacyPlanResult);
        _legacyPlans = null;
        if (_subscribedInstantiator != null) _subscribedInstantiator.CalibrationStateChanged -= OnCalibrationStateChanged;
        _subscribedInstantiator = null;
    }

    private void OnEnable()
    {
        ActiveMenus.Add(this);
        BindController();
        if (_panel != null) BindTaskControls();
        if (_panel != null) _panel.gameObject.SetActive(true);
        if (_grabHandle != null) _grabHandle.gameObject.SetActive(true);
        RefreshMenu();
    }

    private void OnApplicationFocus(bool focused)
    {
        if (!focused) ClearPointerState();
    }

    private void OnApplicationPause(bool paused)
    {
        if (paused) ClearPointerState();
    }

    private void ClearPointerState()
    {
        // A suspended XR input module may never send the matching PointerUp.
        _hoveringPointers.Clear();
        _pressedPointers.Clear();
        _playbackTimelinePointers.Clear();
        _grabHovering.Clear();
        CancelMenuGrab();
        if (_grabHandle != null && _grabHandle.interactionManager != null)
            _grabHandle.interactionManager.CancelInteractableHover((IXRHoverInteractable)_grabHandle);
        HoveredMenus.Remove(this);
    }

    private void OnDisable()
    {
        ActiveMenus.Remove(this);
        Unsubscribe();
        UnsubscribeTaskControls();
        if (IsCalibrating) _instantiator.CancelCalibration();
        ClearPointerState();
        if (_panel != null) _panel.gameObject.SetActive(false);
        if (_grabHandle != null) _grabHandle.gameObject.SetActive(false);
    }

    private void OnDestroy()
    {
        ActiveMenus.Remove(this);
        Unsubscribe();
        UnsubscribeTaskControls();
        HoveredMenus.Remove(this);
        if (_grabHandle != null)
        {
            CancelMenuGrab();
            _grabHandle.hoverEntered.RemoveListener(GrabHoverEntered);
            _grabHandle.hoverExited.RemoveListener(GrabHoverExited);
            _grabHandle.selectEntered.RemoveListener(GrabEntered);
            _grabHandle.selectExited.RemoveListener(GrabExited);
            _grabHandle.selectFilters.Remove(_grabFilter);
            if (!_ownsCanvas) Destroy(_grabHandle.gameObject);
        }
        foreach (Choice choice in _choices)
            if (choice.Button != null) choice.Button.onClick.RemoveListener(choice.Click);
        if (_recenterButton != null) _recenterButton.onClick.RemoveListener(RecenterMenu);
        if (_planButton != null) _planButton.onClick.RemoveListener(PlanTask);
        if (_executeButton != null) _executeButton.onClick.RemoveListener(ExecuteTask);
        if (_rescanButton != null) _rescanButton.onClick.RemoveListener(RescanQr);
        if (_alignButton != null) _alignButton.onClick.RemoveListener(BeginAlignment);
        if (_pathThicknessSlider != null) _pathThicknessSlider.onValueChanged.RemoveListener(SetPathThickness);
        if (_playbackPlayPauseButton != null) _playbackPlayPauseButton.onClick.RemoveListener(TogglePlayback);
        if (_playbackReplayButton != null) _playbackReplayButton.onClick.RemoveListener(ReplayPlayback);
        if (_playbackTimeline != null) _playbackTimeline.onValueChanged.RemoveListener(SeekPlayback);
        for (int index = 0; index < _playbackSpeedButtons.Count; index++)
            if (_playbackSpeedButtons[index] != null) _playbackSpeedButtons[index].onClick.RemoveListener(_playbackSpeedActions[index]);
        if (_tcpTaskButton != null) _tcpTaskButton.onClick.RemoveListener(SelectTcpTask);
        if (_pickPlaceTaskButton != null) _pickPlaceTaskButton.onClick.RemoveListener(SelectPickPlaceTask);
        foreach (Button button in _alignmentActionButtons) if (button != null) button.onClick.RemoveAllListeners();
        foreach (Button button in _translationStepButtons) if (button != null) button.onClick.RemoveAllListeners();
        foreach (Button button in _rotationStepButtons) if (button != null) button.onClick.RemoveAllListeners();
        if (_ownsCanvas && _canvas != null) Destroy(_canvas.gameObject);
        else if (_panel != null) Destroy(_panel.gameObject);
    }
}
