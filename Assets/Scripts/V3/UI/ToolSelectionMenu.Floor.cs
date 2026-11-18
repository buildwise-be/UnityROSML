using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

public sealed partial class ToolSelectionMenu
{
    private static readonly string[] FloorOffsetLabels = { "Zero", "⅓ tile", "½ tile" };
    private static readonly int[] FloorCountSteps = { -10, -1, 1, 10 };
    private readonly Button[] _floorCountButtons = new Button[FloorCountSteps.Length];
    private FloorTilingController _floorTiling;
    private RectTransform _floorView;
    private Button _floorSetupButton, _floorCaptureButton;
    private Button _floorScanButton, _floorPlan, _floorExecute, _floorFixture;
    private Button _floorRotateLeft, _floorRotateRight, _floorPreviousStep, _floorNextStep;
    private TextMeshProUGUI _floorCaptureHelp, _floorCountValue, _floorOffsetValue, _floorMortarValue;
    private TextMeshProUGUI _floorAngleValue, _floorScanLabel, _floorStatus;
    private Slider _floorOffsetSlider, _floorMortarSlider;
    private bool _showFloorSetup;

    private bool IsShowingFloorSetup => _showFloorSetup && _input != null && _input.IsFloorMode && !IsPlacementBusy;

    private void BuildFloorEntry()
    {
        if (_workflowMode != URInputManager.WorkflowMode.RepetitivePickAndPlace) return;
        _floorSetupButton = PickSettingButton("Floor tiling", _taskView, 16f, 204f, 130f, 32f, OpenFloorSetup);
        _floorPreviousStep = PickSettingButton("Previous floor scene step", _taskView, 294f, 204f, 62f, 32f,
            () => _floorTiling?.StepReview(-1), "‹ Step");
        _floorNextStep = PickSettingButton("Next floor scene step", _taskView, 362f, 204f, 62f, 32f,
            () => _floorTiling?.StepReview(1), "Step ›");
        RefreshFloorReview(false, false);
    }

    private void RefreshFloorReview(bool visible, bool enabled)
    {
        if (_floorPreviousStep == null) return;
        _floorPreviousStep.gameObject.SetActive(visible);
        _floorNextStep.gameObject.SetActive(visible);
        _floorPreviousStep.interactable = visible && enabled && _floorTiling.ReviewStepIndex > 0;
        _floorNextStep.interactable = visible && enabled && _floorTiling.ReviewStepIndex < _floorTiling.ReviewStepCount - 1;
    }

    private void BuildFloorView()
    {
        if (_workflowMode != URInputManager.WorkflowMode.RepetitivePickAndPlace) return;
        _floorView = Rect("Floor tiling setup view", _panel, Vector2.zero, new Vector2(440f, 756f));
        BuildHeader(_floorView, "Floor tiling title", "Floor tiling");

        _floorCaptureButton = FloorButton("Capture first tile", 16f, 56f, 408f, 42f, () => _floorTiling?.BeginCapture(),
            "Capture stack top · 3 corners");
        _floorCaptureHelp = FloorText("First tile instructions", 16f, 103f, 408f, 45f, 14f);
        FloorText("Batch count instructions", 16f, 155f, 408f, 42f, 14f).text =
            "Set how many tiles to take from this stack. The preview shows their floor layout.";

        _floorCountValue = FloorText("Tile count and grid size", 16f, 206f, 198f, 42f, 16f);
        for (int index = 0; index < FloorCountSteps.Length; index++)
        {
            int step = FloorCountSteps[index];
            string label = step < 0 ? "−" + -step : "+" + step;
            _floorCountButtons[index] = FloorButton("Change tile count " + label, 216f + index * 53f, 210f, 49f, 34f,
                () => ChangeFloorTileCount(step), label);
        }

        _floorOffsetValue = FloorText("Row offset value", 16f, 257f, 408f, 22f, 15f);
        _floorOffsetSlider = CreateFloorSlider("Row offset slider", 283f, 0, 2,
            value => _floorTiling?.SetOffsetIndex(Mathf.RoundToInt(value)));
        _floorMortarValue = FloorText("Mortar thickness value", 16f, 325f, 408f, 22f, 15f);
        _floorMortarSlider = CreateFloorSlider("Mortar thickness slider", 351f,
            FloorTilePattern.MinimumMortarMillimeters, FloorTilePattern.MaximumMortarMillimeters,
            value => _floorTiling?.SetMortarMillimeters(Mathf.RoundToInt(value)));

        _floorAngleValue = FloorText("Pattern angle value", 16f, 395f, 244f, 34f, 15f);
        _floorRotateLeft = FloorButton("Rotate floor pattern left", 268f, 395f, 74f, 34f,
            () => _floorTiling?.RotatePattern(-15f), "−15°");
        _floorRotateRight = FloorButton("Rotate floor pattern right", 350f, 395f, 74f, 34f,
            () => _floorTiling?.RotatePattern(15f), "+15°");

        _floorScanButton = CreateButton("Scan floor", _floorView, new Vector2(16f, -444f),
            new Vector2(408f, 42f), out _floorScanLabel);
        _floorScanButton.onClick.AddListener(ToggleFloorScan);
        FloorText("Floor anchor instructions", 16f, 491f, 408f, 39f, 14f).text =
            "Look across the floor until a green outline appears. Aim inside it and press trigger to place the pattern.";
        _floorStatus = FloorText("Floor setup status", 16f, 532f, 408f, 63f, 15f);
        _floorPlan = FloorButton("Plan floor preview", 16f, 605f, 198f, 42f, PlanFloorTask, "Plan preview");
        _floorExecute = FloorButton("Execute floor plan", 226f, 605f, 198f, 42f, ExecuteTask, "Execute plan");

        FloorButton("Open robot controls", 16f, 700f, 198f, 38f, CloseFloorSetup, "Robot controls");
        _floorFixture = FloorButton("Open fixture setup", 226f, 700f, 198f, 38f, OpenFixtureSetup, "Fixture setup");
        _floorView.gameObject.SetActive(false);
    }

    private Button FloorButton(string name, float x, float y, float width, float height, UnityAction action, string label = null)
        => PickSettingButton(name, _floorView, x, y, width, height, action, label);

    private TextMeshProUGUI FloorText(string name, float x, float y, float width, float height, float size)
    {
        TextMeshProUGUI text = Text(name, _floorView, new Vector2(x, -y), new Vector2(width, height), string.Empty, size);
        text.richText = false;
        return text;
    }

    private Slider CreateFloorSlider(string name, float y, int minimum, int maximum, UnityAction<float> change)
    {
        RectTransform root = Rect(name, _floorView, new Vector2(16f, -y), new Vector2(408f, 30f));
        root.gameObject.AddComponent<Image>().color = Color.clear;
        RectTransform track = Rect("Track", root, new Vector2(14f, -12f), new Vector2(380f, 6f));
        track.gameObject.AddComponent<Image>().color = new Color(0.15f, 0.23f, 0.29f);
        RectTransform fillArea = Rect("Fill area", root, new Vector2(14f, -12f), new Vector2(380f, 6f));
        RectTransform fill = Rect("Fill", fillArea, Vector2.zero, Vector2.zero);
        fill.anchorMin = Vector2.zero;
        fill.anchorMax = Vector2.one;
        fill.pivot = new Vector2(0f, 0.5f);
        fill.gameObject.AddComponent<Image>().color = new Color(0.08f, 0.56f, 0.68f);
        RectTransform handleArea = Rect("Handle area", root, new Vector2(14f, 0f), new Vector2(380f, 30f));
        RectTransform handle = Rect("Handle", handleArea, Vector2.zero, new Vector2(24f, -6f));
        handle.anchorMin = Vector2.zero;
        handle.anchorMax = Vector2.up;
        handle.pivot = new Vector2(0.5f, 0.5f);
        Image handleImage = handle.gameObject.AddComponent<Image>();
        handleImage.color = new Color(0.55f, 0.91f, 1f);

        Slider slider = root.gameObject.AddComponent<Slider>();
        slider.minValue = minimum;
        slider.maxValue = maximum;
        slider.wholeNumbers = true;
        slider.fillRect = fill;
        slider.handleRect = handle;
        slider.targetGraphic = handleImage;
        slider.navigation = new Navigation { mode = Navigation.Mode.None };
        slider.onValueChanged.AddListener(change);
        AddPressTracking(root.gameObject);
        return slider;
    }

    private void BindFloorControls()
    {
        FloorTilingController controller = _workflowMode == URInputManager.WorkflowMode.RepetitivePickAndPlace && _input != null
            ? _input.FloorController : null;
        if (_floorTiling == controller) return;
        UnbindFloorControls();
        _floorTiling = controller;
        if (_floorTiling != null) _floorTiling.Changed += RefreshTask;
    }

    private void UnbindFloorControls()
    {
        if (_floorTiling != null) _floorTiling.Changed -= RefreshTask;
        _floorTiling = null;
    }

    private void OpenFloorSetup()
    {
        BindTaskControls();
        if (_input == null || IsPlacementBusy || TargetRotationHandles.IsInteracting || RobotExecutionState.IsBusy ||
            MTCPlansManager.ActiveExecution != null || _fixtureTiling?.IsPlanning == true || _floorTiling?.IsPlanning == true) return;
        _input.UseFloorTiling();
        if (!_input.IsFloorMode) return;
        _showFloorSetup = true;
        _localNotice = null;
        RefreshTask();
    }

    private void CloseFloorSetup()
    {
        _showFloorSetup = false;
        EndFloorSetupInteractions();
        RefreshTask();
    }

    private void EndFloorSetupInteractions()
    {
        if (_floorTiling == null) return;
        if (_floorTiling.SurfaceCapture?.IsCapturing == true) _floorTiling.SurfaceCapture.Cancel();
        _floorTiling.StopScan();
        HideFloorMenuPreview();
    }

    private void ChangeFloorTileCount(int step)
    {
        if (_floorTiling != null) _floorTiling.SetTileCount(Mathf.Clamp(_floorTiling.TileCount + step,
            FloorTilePattern.MinimumTileCount, FloorTilePattern.MaximumTileCount));
    }

    private void ToggleFloorScan()
    {
        if (_floorTiling == null) return;
        if (_floorTiling.IsScanRequested) _floorTiling.StopScan();
        else if (_floorTiling.CanScan) _floorTiling.StartScan();
        RefreshFloorAvailability();
    }

    private void PlanFloorTask()
    {
        if (_floorTiling?.CanPlan != true) return;
        CloseFloorSetup();
        PlanTask();
    }

    private void RefreshFloorMenu()
    {
        if (_floorView == null) return;
        if (_input == null || !_input.IsFloorMode) _showFloorSetup = false;
        _floorView.gameObject.SetActive(IsShowingFloorSetup);
        if (!IsShowingFloorSetup) HideFloorMenuPreview();
        if (_floorTiling == null) return;

        _floorCountValue.text = $"{_floorTiling.TileCount} tiles\n{_floorTiling.Rows} rows × {_floorTiling.Columns} columns";
        _floorOffsetValue.text = "Row offset · " + FloorOffsetLabels[_floorTiling.OffsetIndex];
        _floorMortarValue.text = $"Mortar thickness · {_floorTiling.MortarMillimeters} mm";
        _floorAngleValue.text = $"Pattern angle · {_floorTiling.YawDegrees:0}°";
        _floorOffsetSlider.SetValueWithoutNotify(_floorTiling.OffsetIndex);
        _floorMortarSlider.SetValueWithoutNotify(_floorTiling.MortarMillimeters);
        RefreshFloorAvailability();
    }

    private void RefreshFloorAvailability()
    {
        if (_floorTiling == null || !IsShowingFloorSetup || _floorView == null) return;
        PickSurfaceCapture capture = _floorTiling.SurfaceCapture;
        bool capturing = capture?.IsCapturing == true;
        bool canEdit = _floorTiling.CanEdit;
        bool canEditPattern = canEdit && !capturing;
        _floorCaptureButton.interactable = canEditPattern;
        for (int index = 0; index < FloorCountSteps.Length; index++)
        {
            int step = FloorCountSteps[index];
            _floorCountButtons[index].interactable = canEditPattern && (step < 0
                ? _floorTiling.TileCount > FloorTilePattern.MinimumTileCount
                : _floorTiling.TileCount < FloorTilePattern.MaximumTileCount);
        }
        _floorOffsetSlider.interactable = _floorMortarSlider.interactable = canEditPattern;
        _floorRotateLeft.interactable = _floorRotateRight.interactable = canEditPattern;
        _floorScanButton.interactable = _floorTiling.IsScanRequested || _floorTiling.CanScan;
        _floorScanLabel.text = _floorTiling.IsScanRequested ? "Stop scan" : "Scan floor";
        _floorPlan.interactable = _floorTiling.CanPlan;
        _floorExecute.interactable = _floorTiling.CanExecute;
        _floorFixture.interactable = canEdit && !_floorTiling.IsPlanning;
        _floorCaptureHelp.text = capturing ? capture.StatusText : capture?.IsSurfaceCurrent == true
            ? "Stack top captured. Set the pattern, then select Scan floor."
            : "Touch three consecutive top corners of the source tile, pressing trigger at each. This sets the pick centre.";
        _floorStatus.text = capturing ? string.Empty : _floorTiling.StatusText;
    }

    private void PositionFloorMenuPreview()
    {
        if (_floorTiling == null || _floorView == null || !_floorView.gameObject.activeInHierarchy) return;
        RectTransform menu = (RectTransform)_canvas.transform;
        float tileSide = (float)PickSurfaceGeometry.GetPreset(PickItem.Tile).Width;
        int rows = _floorTiling.Rows;
        float fullDepth = rows * tileSide + (rows - 1) * _floorTiling.MortarMillimeters * 0.001f;
        Vector3 top = menu.TransformPoint(new Vector3(0f, menu.rect.yMax, 0f));
        Vector3 worldCenter = top + menu.up * (0.08f + fullDepth * 0.25f);
        _floorTiling.ShowMenuPreview(menu, worldCenter, true);
    }

    private void HideFloorMenuPreview() => _floorTiling?.ShowMenuPreview(null, Vector3.zero, false);
}
