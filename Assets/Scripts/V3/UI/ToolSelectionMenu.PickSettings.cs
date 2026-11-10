using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>Single-item MTC pickup settings and the three-corner surface workflow.</summary>
public sealed partial class ToolSelectionMenu
{
    private PickPlaceController _pickSettingsController;
    private PickPlaceController _subscribedPickSettings;
    private PickSurfaceCapture _subscribedPickSurface;
    private RectTransform _pickSettingsEntry, _pickSettingsView;
    private TextMeshProUGUI _pickSettingsSummary, _pickSettingsStatus, _pickCaptureProgress, _pickCaptureHelp;
    private Button _pickSettingsButton, _pickCenterButton, _pickSurfaceButton, _pickTileButton, _pickBrickButton;
    private Button _pickSlowButton, _pickMediumButton, _pickFastButton;
    private Button _pickCaptureUndo, _pickCaptureRestart, _pickCaptureCancel;
    private bool _showPickSettings;
    private bool IsMtcPickMode => _workflowMode != URInputManager.WorkflowMode.RepetitivePickAndPlace &&
        _input != null && !_input.IsTcpMode && !_input.IsFixtureMode;
    private bool IsSurfaceCapturing => _pickSettingsController != null &&
        _pickSettingsController.SurfaceCapture != null && _pickSettingsController.SurfaceCapture.IsCapturing;
    private bool IsShowingPickSettings => _showPickSettings && IsMtcPickMode &&
        !IsCalibrating && !IsPointCollecting && !IsPositionAdjusting;

    private void BuildPickSettings()
    {
        if (_workflowMode == URInputManager.WorkflowMode.RepetitivePickAndPlace) return;
        _pickSettingsEntry = Rect("MTC pick settings entry", _taskView,
            new Vector2(456f, -450f), new Vector2(300f, 112f));
        Image background = _pickSettingsEntry.gameObject.AddComponent<Image>();
        background.color = new Color(0.035f, 0.06f, 0.09f, 0.97f);
        Outline border = _pickSettingsEntry.gameObject.AddComponent<Outline>();
        border.effectColor = new Color(0.2f, 0.44f, 0.53f, 0.8f);
        border.effectDistance = new Vector2(1f, -1f);
        AddHoverTracking(_pickSettingsEntry.gameObject);
        _pickSettingsSummary = Text("Pick settings summary", _pickSettingsEntry, new Vector2(12f, -10f),
            new Vector2(276f, 43f), "Pick by center · 5 mm/s", 14f);
        _pickSettingsSummary.richText = false;
        _pickSettingsButton = PickSettingButton("Pick settings", _pickSettingsEntry, 12f, 63f, 276f, 36f, OpenPickSettings);
        _pickSettingsEntry.gameObject.SetActive(false);

        _pickSettingsView = Rect("MTC pick settings view", _panel, Vector2.zero, new Vector2(440f, 756f));
        BuildHeader(_pickSettingsView, "Pick settings title", "Pick settings");
        Text("Pick settings intro", _pickSettingsView, new Vector2(16f, -52f), new Vector2(408f, 44f),
            "Choose a pickup method and descent speed.\nPlace is captured with a double bumper press.", 14f);
        Text("Pickup method title", _pickSettingsView, new Vector2(16f, -106f), new Vector2(408f, 25f),
            "Pickup method", 17f);
        _pickCenterButton = PickSettingButton("Pick by center", _pickSettingsView, 16f, 139f, 198f, 42f,
            () => SelectPickMethod(false), "Center");
        _pickSurfaceButton = PickSettingButton("Pick by surface", _pickSettingsView, 226f, 139f, 198f, 42f,
            () => SelectPickMethod(true), "Item surface");
        Text("Pick item title", _pickSettingsView, new Vector2(16f, -202f), new Vector2(408f, 25f),
            "Item · select to capture its top face", 17f);
        _pickTileButton = PickSettingButton("Pick tile", _pickSettingsView, 16f, 236f, 198f, 72f,
            () => SelectPickItem(PickItem.Tile), "Tile\n20 × 20 × 0.7 cm · ≈0.67 kg");
        _pickBrickButton = PickSettingButton("Pick brick", _pickSettingsView, 226f, 236f, 198f, 72f,
            () => SelectPickItem(PickItem.Brick), "Brick\n28 × 18.5 × 13.5 cm\n≈11.2 kg estimated");
        Text("Descent speed title", _pickSettingsView, new Vector2(16f, -330f), new Vector2(408f, 25f),
            "Pick descent speed", 17f);
        _pickSlowButton = PickSettingButton("Pick descent 5 mm per second", _pickSettingsView, 16f, 363f, 130f, 40f,
            () => SelectPickDescentSpeed(5), "5 mm/s");
        _pickMediumButton = PickSettingButton("Pick descent 10 mm per second", _pickSettingsView, 155f, 363f, 130f, 40f,
            () => SelectPickDescentSpeed(10), "10 mm/s");
        _pickFastButton = PickSettingButton("Pick descent 20 mm per second", _pickSettingsView, 294f, 363f, 130f, 40f,
            () => SelectPickDescentSpeed(20), "20 mm/s");
        _pickCaptureProgress = Text("Pick surface capture progress", _pickSettingsView,
            new Vector2(16f, -430f), new Vector2(408f, 31f), "Center pickup", 18f);
        _pickCaptureProgress.richText = false;
        _pickCaptureHelp = Text("Pick surface capture instructions", _pickSettingsView,
            new Vector2(16f, -470f), new Vector2(408f, 79f), "Hold bumper to capture the pick contact center.", 15f);
        _pickCaptureHelp.richText = false;
        _pickSettingsStatus = Text("Pick settings status", _pickSettingsView,
            new Vector2(16f, -559f), new Vector2(408f, 59f), string.Empty, 14f);
        _pickSettingsStatus.richText = false;
        _pickSettingsStatus.enableAutoSizing = true;
        _pickSettingsStatus.fontSizeMin = 12f;
        _pickSettingsStatus.fontSizeMax = 14f;
        _pickCaptureUndo = PickSettingButton("Undo pick corner", _pickSettingsView, 16f, 630f, 130f, 34f,
            UndoPickSurface, "Undo corner");
        _pickCaptureRestart = PickSettingButton("Restart pick surface", _pickSettingsView, 155f, 630f, 130f, 34f,
            RestartPickSurface, "Recapture");
        _pickCaptureCancel = PickSettingButton("Cancel pick surface", _pickSettingsView, 294f, 630f, 130f, 34f,
            CancelPickSurface, "Cancel capture");
        PickSettingButton("Back to pick task", _pickSettingsView, 16f, 700f, 408f, 38f, ClosePickSettings, "Back to task");
        _pickSettingsView.gameObject.SetActive(false);
    }

    private Button PickSettingButton(string name, Transform parent, float x, float y, float width, float height,
        UnityEngine.Events.UnityAction action, string label = null)
    {
        Button button = CreateButton(name, parent, new Vector2(x, -y), new Vector2(width, height), out TextMeshProUGUI text);
        text.text = label ?? name;
        text.fontSize = height > 50f ? 13f : 15f;
        text.richText = false;
        button.onClick.AddListener(action);
        return button;
    }

    private void BindPickSettingsControls()
    {
        if (_workflowMode == URInputManager.WorkflowMode.RepetitivePickAndPlace)
        {
            UnbindPickSettingsControls();
            _pickSettingsController = null;
            return;
        }
        if (_input != null) _pickSettingsController = _input.PickPlaceController;
        else if (_pickSettingsController == null) _pickSettingsController = FindFirstObjectByType<PickPlaceController>();
        if (_subscribedPickSettings != _pickSettingsController)
        {
            if (_subscribedPickSettings != null)
            {
                _subscribedPickSettings.SettingsChanged -= OnPickSettingsChanged;
                _subscribedPickSettings.RepeatConfirmationRequested -= OpenRepeatConfirmation;
            }
            _subscribedPickSettings = _pickSettingsController;
            if (_subscribedPickSettings != null)
            {
                _subscribedPickSettings.SettingsChanged += OnPickSettingsChanged;
                _subscribedPickSettings.RepeatConfirmationRequested += OpenRepeatConfirmation;
            }
        }
        PickSurfaceCapture capture = _pickSettingsController != null ? _pickSettingsController.SurfaceCapture : null;
        if (_subscribedPickSurface == capture) return;
        if (_subscribedPickSurface != null) _subscribedPickSurface.Changed -= OnPickSettingsChanged;
        _subscribedPickSurface = capture;
        if (_subscribedPickSurface != null) _subscribedPickSurface.Changed += OnPickSettingsChanged;
    }

    private void UnbindPickSettingsControls()
    {
        if (_subscribedPickSettings != null)
        {
            _subscribedPickSettings.SettingsChanged -= OnPickSettingsChanged;
            _subscribedPickSettings.RepeatConfirmationRequested -= OpenRepeatConfirmation;
        }
        if (_subscribedPickSurface != null) _subscribedPickSurface.Changed -= OnPickSettingsChanged;
        _subscribedPickSettings = null;
        _subscribedPickSurface = null;
    }

    private void OnPickSettingsChanged() { _localNotice = null; RefreshTask(); }

    public void OpenPickSettings()
    {
        BindTaskControls();
        if (!IsMtcPickMode || IsCalibrating || IsPointCollecting || IsPositionAdjusting ||
            RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return;
        _showPickSettings = true;
        TargetPoseContextMenu.CloseAll();
        RefreshTask();
    }

    public void ClosePickSettings()
    {
        CancelPickSurface();
        _showPickSettings = false;
        RefreshTask();
    }

    public void SelectPickMethod(bool surface)
    {
        if (!CanEditPickSettings || _pickSettingsController == null) return;
        _pickSettingsController.SetPickBySurface(surface);
        BindPickSettingsControls();
        RefreshTask();
    }

    public void SelectPickItem(PickItem item)
    {
        if (!CanEditPickSettings || _pickSettingsController == null) return;
        if (!_pickSettingsController.SetSelectedItem(item)) return;
        if (_pickSettingsController.PickBySurface)
            _pickSettingsController.GetOrCreateSurfaceCapture().BeginCapture(item);
        BindPickSettingsControls();
        RefreshTask();
    }

    public void SelectPickDescentSpeed(int millimetersPerSecond)
    {
        if (CanEditPickSettings && _pickSettingsController != null)
            _pickSettingsController.SetDescentSpeedMmPerSecond(millimetersPerSecond);
        RefreshTask();
    }

    private bool CanEditPickSettings => IsMtcPickMode && !RobotExecutionState.IsBusy &&
        MTCPlansManager.ActiveExecution == null && !IsCalibrating && !IsPointCollecting && !IsPositionAdjusting;

    public void UndoPickSurface()
    {
        if (CanEditPickSettings && _pickSettingsController != null) _pickSettingsController.SurfaceCapture?.Undo();
        RefreshTask();
    }

    public void RestartPickSurface()
    {
        if (!CanEditPickSettings || _pickSettingsController == null || !_pickSettingsController.PickBySurface) return;
        _pickSettingsController.GetOrCreateSurfaceCapture().BeginCapture(_pickSettingsController.SelectedItem);
        BindPickSettingsControls();
        RefreshTask();
    }

    public void CancelPickSurface()
    {
        if (_pickSettingsController != null && _pickSettingsController.SurfaceCapture != null &&
            _pickSettingsController.SurfaceCapture.IsCapturing) _pickSettingsController.SurfaceCapture.Cancel();
    }

    private void RefreshPickSettingsMenu()
    {
        if (_pickSettingsView == null) return;
        if (!IsMtcPickMode) _showPickSettings = false;
        _pickSettingsView.gameObject.SetActive(IsShowingPickSettings);
        if (_pickSettingsEntry != null) _pickSettingsEntry.gameObject.SetActive(IsMtcPickMode);
        bool editable = CanEditPickSettings && _pickSettingsController != null;
        bool surface = _pickSettingsController != null && _pickSettingsController.PickBySurface;
        PickItem item = _pickSettingsController != null ? _pickSettingsController.SelectedItem : PickItem.Tile;
        int speed = _pickSettingsController != null ? _pickSettingsController.DescentSpeedMmPerSecond : 5;
        PickSurfaceCapture capture = _pickSettingsController != null ? _pickSettingsController.SurfaceCapture : null;
        bool capturing = capture != null && capture.IsCapturing;
        bool ready = _pickSettingsController != null && _pickSettingsController.HasValidSurface;
        _pickSettingsSummary.text = (surface ? "Pick by surface · " + item : "Pick by center") + "\nDescent " + speed + " mm/s";
        _pickSettingsButton.interactable = editable && !IsPlacementBusy;
        RefreshTaskChoice(_pickCenterButton, !surface, editable);
        RefreshTaskChoice(_pickSurfaceButton, surface, editable);
        RefreshTaskChoice(_pickTileButton, item == PickItem.Tile, editable && surface);
        RefreshTaskChoice(_pickBrickButton, item == PickItem.Brick, editable && surface);
        RefreshTaskChoice(_pickSlowButton, speed == 5, editable);
        RefreshTaskChoice(_pickMediumButton, speed == 10, editable);
        RefreshTaskChoice(_pickFastButton, speed == 20, editable);
        _pickCaptureUndo.interactable = editable && capturing && capture.CapturedCount > 0;
        _pickCaptureRestart.interactable = editable && surface;
        _pickCaptureCancel.interactable = editable && capturing;
        _pickCaptureProgress.text = !surface ? "Center pickup" : capturing
            ? "Top face · " + capture.CapturedCount + " / 3 corners captured" : ready ? "Top face ready" : "Select an item to capture its top face";
        _pickCaptureHelp.text = !surface ? "Hold bumper to capture the pick contact center.\nDouble press bumper to capture the place contact."
            : "Touch three consecutive top corners with the controller tip; press trigger at each.\nFirst corner → along length → along width.";
        _pickSettingsStatus.text = surface && capture != null ? capture.StatusText
            : surface ? "Select Tile or Brick to show the floating corner guide."
            : "Center pickup is the default. Descent speed applies to the pick approach.";
    }
}
