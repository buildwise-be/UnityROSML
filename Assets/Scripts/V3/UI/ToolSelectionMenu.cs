using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>Shared XR controls for tool matching, TCP planning and marker alignment.</summary>
public sealed class ToolSelectionMenu : MonoBehaviour
{
    [SerializeField] private ToolSelectionController _controller;
    [SerializeField] private URInputManager _input;
    [SerializeField] private TcpPlanningController _tcpPlanning;
    [SerializeField] private QRDetector _qrDetector;
    [SerializeField] private URInstantiator _instantiator;
    [SerializeField, Tooltip("Existing transparent app logo used in the task and alignment headers.")]
    private Texture2D _appLogo;
    [SerializeField, Tooltip("Optional canvas dedicated to this menu. A world-space canvas is created when empty.")]
    private Canvas _canvas;

    [Header("Loose viewer follow")]
    [SerializeField] private bool _followUser = true;
    [SerializeField, Min(0.6f)] private float _followDistance = 1f;
    [SerializeField, Range(-40f, 40f)] private float _followSideAngle = 20f;
    [SerializeField] private float _followHeightOffset = -0.15f;
    [SerializeField, Range(20f, 90f)] private float _followAngularDeadZone = 50f;
    [SerializeField, Min(0.1f), Tooltip("Seconds for the remaining follow distance to halve.")]
    private float _followHalfLife = 1.1f;
    [SerializeField, Min(0.05f)] private float _followMaxSpeed = 0.6f;
    [SerializeField, Min(0f)] private float _followDelay = 0.65f;
    [SerializeField, Min(0f)] private float _followInteractionHold = 0.8f;
    [SerializeField, Min(1f)] private float _followTurnSpeed = 35f;

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
    public static bool IsHovered => HoveredMenus.Count != 0;

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
    private bool _followActive;
    private float _outsideComfortTime;
    private float _interactionHoldRemaining;
    private Vector3 _lastFollowForward = Vector3.forward;
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

    private void LateUpdate() => UpdateFollow(Time.unscaledDeltaTime);

    private void UpdateFollow(float deltaTime)
    {
        if (!_followUser || !IsVisible || _canvas == null || deltaTime <= 0f ||
            float.IsNaN(deltaTime) || float.IsInfinity(deltaTime)) return;
        if (_hoveringPointers.Count != 0 || _pressedPointers.Count != 0 || IsCalibrating ||
            RobotCoordinateService.IsCalibrationEditActive || TargetRotationHandles.IsHovered || TargetRotationHandles.IsInteracting)
        {
            ResetFollowMotion();
            _interactionHoldRemaining = _followInteractionHold;
            return;
        }
        Camera viewer = Camera.main;
        if (viewer == null || !RobotFrameMath.IsValidPose(new Pose(viewer.transform.position, viewer.transform.rotation)))
        { ResetFollowMotion(); return; }
        _canvas.worldCamera = viewer;
        if (!_hasPositioned) { RecenterMenu(); return; }
        if (_interactionHoldRemaining > 0f)
        {
            _interactionHoldRemaining = Mathf.Max(0f, _interactionHoldRemaining - deltaTime);
            return;
        }

        Transform camera = viewer.transform;
        Vector3 goal = ComfortPosition(camera);
        Vector3 fromViewer = _canvas.transform.position - camera.position;
        Vector3 horizontal = Vector3.ProjectOnPlane(fromViewer, Vector3.up);
        float angle = horizontal.sqrMagnitude > 0.000001f
            ? Vector3.Angle(_lastFollowForward, horizontal) : 0f;
        float distance = fromViewer.magnitude;
        float desiredDistance = Mathf.Max(0.6f, _followDistance);
        float heightError = Mathf.Abs(_canvas.transform.position.y - goal.y);
        bool outside = distance < desiredDistance * 0.65f || distance > desiredDistance * 1.45f ||
            angle > _followAngularDeadZone || heightError > 0.45f;
        bool comfortable = distance >= desiredDistance * 0.85f && distance <= desiredDistance * 1.2f &&
            angle <= _followAngularDeadZone * 0.65f && heightError <= 0.25f;
        if (_followActive && comfortable) ResetFollowMotion();
        if (!_followActive)
        {
            _outsideComfortTime = outside ? _outsideComfortTime + deltaTime : 0f;
            if (outside && _outsideComfortTime >= _followDelay) _followActive = true;
        }
        if (_followActive)
        {
            // Resuming after a long frame must not jump across the user's view.
            float motionTime = Mathf.Min(deltaTime, 0.1f);
            float blend = 1f - Mathf.Pow(0.5f, motionTime / Mathf.Max(0.1f, _followHalfLife));
            // Follow around the viewer: a large head turn must not send a straight
            // path through their face on the way to the opposite peripheral side.
            Vector3 goalOffset = goal - camera.position;
            Vector3 goalHorizontal = Vector3.ProjectOnPlane(goalOffset, Vector3.up);
            float radius = horizontal.magnitude;
            Vector3 direction = radius > 0.0001f ? horizontal / radius : goalHorizontal.normalized;
            float turn = Vector3.SignedAngle(direction, goalHorizontal, Vector3.up) * blend;
            direction = Quaternion.AngleAxis(turn, Vector3.up) * direction;
            Vector3 nextOffset = direction * Mathf.Lerp(radius, goalHorizontal.magnitude, blend) +
                Vector3.up * Mathf.Lerp(fromViewer.y, goalOffset.y, blend);
            Vector3 step = camera.position + nextOffset - _canvas.transform.position;
            _canvas.transform.position += Vector3.ClampMagnitude(step, Mathf.Max(0.05f, _followMaxSpeed) * motionTime);
        }
        if (TryFacingRotation(camera, out Quaternion facing))
        {
            float angleError = Quaternion.Angle(_canvas.transform.rotation, facing);
            if (angleError > 4f)
            {
                float motionTime = Mathf.Min(deltaTime, 0.1f);
                float blend = 1f - Mathf.Pow(0.5f, motionTime / 0.3f);
                _canvas.transform.rotation = Quaternion.RotateTowards(_canvas.transform.rotation, facing,
                    Mathf.Min(angleError * blend, Mathf.Max(1f, _followTurnSpeed) * motionTime));
            }
        }
    }

    private Vector3 ComfortPosition(Transform camera)
    {
        Vector3 horizontal = Vector3.ProjectOnPlane(camera.forward, Vector3.up);
        if (horizontal.sqrMagnitude > 0.000001f) _lastFollowForward = horizontal.normalized;
        Vector3 direction = Quaternion.AngleAxis(_followSideAngle, Vector3.up) * _lastFollowForward;
        return camera.position + direction * Mathf.Max(0.6f, _followDistance) + Vector3.up * _followHeightOffset;
    }

    private void ResetFollowMotion()
    {
        _followActive = false;
        _outsideComfortTime = 0f;
    }

    private void Update()
    {
        if (_controller == null) BindController();
        if (_input == null || _tcpPlanning == null || _qrDetector == null || _instantiator == null) BindTaskControls();
        bool tcp = _input == null || _input.IsTcpMode;
        bool qr = _qrDetector != null && _qrDetector.HasValidatedPose;
        if (_lastTcpMode != tcp || _lastQrValidated != qr || _lastTargetRotation != TargetRotationHandles.IsInteracting) RefreshTask();
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
        _localNotice = null;
        RefreshTask();
    }

    private void RefreshTask()
    {
        bool tcp = _input == null || _input.IsTcpMode;
        _lastTcpMode = tcp;
        _lastQrValidated = _qrDetector != null && _qrDetector.HasValidatedPose;
        _lastTargetRotation = TargetRotationHandles.IsInteracting;
        bool canSelectMode = _input != null && !IsCalibrating && !TargetRotationHandles.IsInteracting;
        RefreshTaskChoice(_tcpTaskButton, tcp, canSelectMode);
        RefreshTaskChoice(_pickPlaceTaskButton, !tcp, canSelectMode);
        if (_taskHelp != null) _taskHelp.text = tcp
            ? "Hold bumper: TCP A. Double press: TCP B.\nPlace the tool preview; drag it to set exact orientation."
            : "Hold/double bumper: pick/place tool contacts.\nBumper over a tool: depth ±5 mm. MTC chooses twist.";
        if (_taskStatus != null)
            _taskStatus.text = _localNotice ?? (tcp
                ? (_tcpPlanning != null ? _tcpPlanning.StatusText : "Capture TCP A and B to prepare a preview.")
                : (_legacyPlans != null ? _legacyPlans.StatusText : "Capture pick and place tool contacts, then Plan preview."));
        if (_planButton != null) _planButton.interactable = !IsCalibrating;
        if (_executeButton != null) _executeButton.interactable = !IsCalibrating && (tcp
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
        Transform camera = Camera.main.transform;
        if (!RobotFrameMath.IsValidPose(new Pose(camera.position, camera.rotation))) return;
        _canvas.transform.position = ComfortPosition(camera);
        FaceCamera(camera);
        _canvas.worldCamera = Camera.main;
        _hasPositioned = true;
        ResetFollowMotion();
    }

    /// <summary>Keep a following panel with its viewer when QR placement changes.</summary>
    public void PositionAt(Pose baseWorld)
    {
        if (IsCalibrating) return;
        if (_panel == null) BuildMenu();
        if (_canvas == null || !RobotFrameMath.IsValidPose(baseWorld)) return;
        if (_followUser && Camera.main != null)
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
        // Keep world space independent of the rig; loose following moves it only when needed.
        _canvas.transform.SetParent(null, true);
        RectTransform canvasRect = (RectTransform)_canvas.transform;
        canvasRect.sizeDelta = new Vector2(440f, 580f);
        canvasRect.localScale = Vector3.one * 0.001f;
        CanvasScaler scaler = _canvas.GetComponent<CanvasScaler>();
        if (scaler == null) scaler = _canvas.gameObject.AddComponent<CanvasScaler>();
        scaler.dynamicPixelsPerUnit = 10f;
        if (_canvas.GetComponent<TrackedDeviceGraphicRaycaster>() == null)
            _canvas.gameObject.AddComponent<TrackedDeviceGraphicRaycaster>();

        _panel = Rect("Robot task controls", canvasRect, Vector2.zero, new Vector2(440f, 580f));
        _panel.anchorMin = _panel.anchorMax = new Vector2(0.5f, 0.5f);
        _panel.pivot = new Vector2(0.5f, 0.5f);
        Image background = _panel.gameObject.AddComponent<Image>();
        background.color = new Color(0.035f, 0.06f, 0.09f, 0.97f);
        background.raycastTarget = true;
        Outline panelBorder = _panel.gameObject.AddComponent<Outline>();
        panelBorder.effectColor = new Color(0.2f, 0.44f, 0.53f, 0.8f);
        panelBorder.effectDistance = new Vector2(1f, -1f);
        AddHoverTracking(_panel.gameObject);

        _taskView = Rect("Task view", _panel, Vector2.zero, new Vector2(440f, 580f));
        _alignmentView = Rect("Alignment view", _panel, Vector2.zero, new Vector2(440f, 580f));
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
        _recenterButton = CreateButton("Recenter menu", _taskView, new Vector2(16f, -449f), new Vector2(130f, 28f), out TextMeshProUGUI recenterLabel);
        recenterLabel.text = "Recenter menu";
        recenterLabel.fontSize = 13f;
        _recenterButton.onClick.AddListener(RecenterMenu);
        _rescanButton = CreateButton("Rescan QR", _taskView, new Vector2(155f, -449f), new Vector2(130f, 28f), out TextMeshProUGUI rescanLabel);
        rescanLabel.fontSize = 13f;
        _rescanButton.onClick.AddListener(RescanQr);
        _alignButton = CreateButton("Align robot", _taskView, new Vector2(294f, -449f), new Vector2(130f, 28f), out TextMeshProUGUI alignLabel);
        alignLabel.fontSize = 13f;
        _alignButton.onClick.AddListener(BeginAlignment);
        _qrStatus = Text("QR placement status", _taskView, new Vector2(16f, -487f), new Vector2(408f, 80f), string.Empty, 12f);
        _qrStatus.color = new Color(0.65f, 0.74f, 0.8f);
        _qrStatus.richText = false;
        _qrStatus.enableAutoSizing = true;
        _qrStatus.fontSizeMin = 10f;
        _qrStatus.fontSizeMax = 12f;
        BuildAlignmentView();
        RefreshAlignmentControls();
        _panel.gameObject.SetActive(isActiveAndEnabled);
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
            if (_hoveringPointers.Count == 0 && _pressedPointers.Count == 0) HoveredMenus.Remove(this);
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
            if (_hoveringPointers.Count == 0 && _pressedPointers.Count == 0) HoveredMenus.Remove(this);
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
        BindController();
        if (_panel != null) BindTaskControls();
        if (_panel != null) _panel.gameObject.SetActive(true);
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
        HoveredMenus.Remove(this);
        ResetFollowMotion();
        _interactionHoldRemaining = _followInteractionHold;
    }

    private void OnDisable()
    {
        Unsubscribe();
        UnsubscribeTaskControls();
        if (IsCalibrating) _instantiator.CancelCalibration();
        ClearPointerState();
        if (_panel != null) _panel.gameObject.SetActive(false);
    }

    private void OnDestroy()
    {
        Unsubscribe();
        UnsubscribeTaskControls();
        HoveredMenus.Remove(this);
        foreach (Choice choice in _choices)
            if (choice.Button != null) choice.Button.onClick.RemoveListener(choice.Click);
        if (_recenterButton != null) _recenterButton.onClick.RemoveListener(RecenterMenu);
        if (_planButton != null) _planButton.onClick.RemoveListener(PlanTask);
        if (_executeButton != null) _executeButton.onClick.RemoveListener(ExecuteTask);
        if (_rescanButton != null) _rescanButton.onClick.RemoveListener(RescanQr);
        if (_alignButton != null) _alignButton.onClick.RemoveListener(BeginAlignment);
        if (_tcpTaskButton != null) _tcpTaskButton.onClick.RemoveListener(SelectTcpTask);
        if (_pickPlaceTaskButton != null) _pickPlaceTaskButton.onClick.RemoveListener(SelectPickPlaceTask);
        foreach (Button button in _alignmentActionButtons) if (button != null) button.onClick.RemoveAllListeners();
        foreach (Button button in _translationStepButtons) if (button != null) button.onClick.RemoveAllListeners();
        foreach (Button button in _rotationStepButtons) if (button != null) button.onClick.RemoveAllListeners();
        if (_ownsCanvas && _canvas != null) Destroy(_canvas.gameObject);
        else if (_panel != null) Destroy(_panel.gameObject);
    }
}
