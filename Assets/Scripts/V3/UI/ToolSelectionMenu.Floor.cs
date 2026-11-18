using System;
using System.Collections.Generic;
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
    private Button _floorSetupButton, _floorCaptureButton, _floorCentreCaptureButton, _floorPlacementButton, _floorCancelPoint;
    private Button _floorScanButton, _floorPlan, _floorExecute, _floorSourceLeft, _floorSourceRight;
    private readonly List<Button> _floorDimensionButtons = new List<Button>();
    private Button _floorRotateLeft, _floorRotateRight, _floorPreviousStep, _floorNextStep;
    private TextMeshProUGUI _floorCaptureHelp, _floorCountValue, _floorOffsetValue, _floorMortarValue;
    private TextMeshProUGUI _floorAngleValue, _floorScanLabel, _floorStatus, _floorSideValue, _floorThicknessValue, _floorSourceYawValue;
    private Slider _floorOffsetSlider, _floorMortarSlider;
    private bool _showFloorSetup;

    private bool IsShowingFloorSetup => _showFloorSetup && _input != null && _input.IsFloorMode && !IsPlacementBusy;

    private void BuildFloorEntry()
    {
        if (_workflowMode != URInputManager.WorkflowMode.RepetitivePickAndPlace) return;
        _floorSetupButton = PickSettingButton("Floor tiling", _taskView, 16f, 204f, 130f, 32f, OpenFloorSetup, "Batch setup");
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
        BuildHeader(_floorView, "Floor tiling title", "Repetitive Pick and Place");
        _floorSideValue = FloorDimensionRow("Tile side", 56f, () => _floorTiling.TileSideMeters, value => _floorTiling.SetTileSideMeters(value));
        _floorThicknessValue = FloorDimensionRow("Tile thickness", 96f, () => _floorTiling.TileThicknessMeters,
            value => _floorTiling.SetTileThicknessMeters(value));
        _floorCentreCaptureButton = FloorButton("Capture source centre", 16f, 143f, 198f, 38f,
            () => _floorTiling?.BeginSourceCenterCapture(), "Pick centre · 1 point");
        _floorCaptureButton = FloorButton("Capture first tile", 226f, 143f, 198f, 38f,
            () => _floorTiling?.BeginCapture(), "Pick · 3 corners");
        _floorCaptureHelp = FloorText("First tile instructions", 16f, 185f, 408f, 32f, 14f);
        _floorSourceYawValue = FloorText("Source yaw", 16f, 225f, 244f, 32f, 15f);
        _floorSourceLeft = FloorButton("Rotate source left", 268f, 225f, 74f, 32f, () => _floorTiling?.RotateSource(-5f), "−5°");
        _floorSourceRight = FloorButton("Rotate source right", 350f, 225f, 74f, 32f, () => _floorTiling?.RotateSource(5f), "+5°");

        _floorCountValue = FloorText("Tile count and grid size", 16f, 272f, 198f, 42f, 16f);
        for (int index = 0; index < FloorCountSteps.Length; index++)
        {
            int step = FloorCountSteps[index];
            string label = step < 0 ? "−" + -step : "+" + step;
            _floorCountButtons[index] = FloorButton("Change tile count " + label, 216f + index * 53f, 276f, 49f, 34f,
                () => ChangeFloorTileCount(step), label);
        }

        _floorOffsetValue = FloorText("Row offset value", 16f, 325f, 408f, 22f, 15f);
        _floorOffsetSlider = CreateFloorSlider("Row offset slider", 349f, 0, 2,
            value => _floorTiling?.SetOffsetIndex(Mathf.RoundToInt(value)));
        _floorMortarValue = FloorText("Mortar thickness value", 16f, 387f, 408f, 22f, 15f);
        _floorMortarSlider = CreateFloorSlider("Mortar thickness slider", 411f,
            FloorTilePattern.MinimumMortarMillimeters, FloorTilePattern.MaximumMortarMillimeters,
            value => _floorTiling?.SetMortarMillimeters(Mathf.RoundToInt(value)));

        _floorPlacementButton = FloorButton("Capture final support surface", 16f, 454f, 198f, 38f,
            () => _floorTiling?.BeginPlacementCapture(), "Place surface · 1 point");
        _floorScanButton = CreateButton("Scan floor", _floorView, new Vector2(226f, -454f),
            new Vector2(198f, 38f), out _floorScanLabel);
        _floorScanButton.onClick.AddListener(ToggleFloorScan);
        _floorAngleValue = FloorText("Pattern angle value", 16f, 504f, 244f, 34f, 15f);
        _floorRotateLeft = FloorButton("Rotate floor pattern left", 268f, 504f, 74f, 34f,
            () => _floorTiling?.RotatePattern(-15f), "−15°");
        _floorRotateRight = FloorButton("Rotate floor pattern right", 350f, 504f, 74f, 34f,
            () => _floorTiling?.RotatePattern(15f), "+15°");

        _floorStatus = FloorText("Floor setup status", 16f, 551f, 408f, 63f, 15f);
        _floorPlan = FloorButton("Plan floor preview", 16f, 625f, 198f, 42f, PlanFloorTask, "Plan preview");
        _floorExecute = FloorButton("Execute floor plan", 226f, 625f, 198f, 42f, ExecuteTask, "Execute plan");

        FloorButton("Open robot controls", 16f, 700f, 198f, 38f, CloseFloorSetup, "Robot controls");
        _floorCancelPoint = FloorButton("Cancel point capture", 226f, 700f, 198f, 38f,
            () => _floorTiling?.CancelPointCapture(), "Cancel capture");
        _floorView.gameObject.SetActive(false);
    }

    private Button FloorButton(string name, float x, float y, float width, float height, UnityAction action, string label = null)
        => PickSettingButton(name, _floorView, x, y, width, height, action, label);

    private TextMeshProUGUI FloorDimensionRow(string name, float y, Func<float> get, Action<float> set)
    {
        TextMeshProUGUI value = FloorText(name + " value", 16f, y, 198f, 34f, 14f);
        for (int index = 0; index < FloorCountSteps.Length; index++)
        {
            int millimeters = FloorCountSteps[index];
            string caption = millimeters < 0 ? "−" + -millimeters : "+" + millimeters;
            _floorDimensionButtons.Add(FloorButton(name + " " + caption + " mm", 216f + index * 53f, y, 49f, 34f,
                () => set(Mathf.Max(.001f, get() + millimeters * .001f)), caption));
        }
        return value;
    }

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
            MTCPlansManager.ActiveExecution != null || _floorTiling?.IsPlanning == true) return;
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
        _floorTiling.CancelPointCapture();
        if (_floorTiling.SurfaceCapture?.IsGuideOpen == true) _floorTiling.SurfaceCapture.Cancel();
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

        _floorCountValue.text = $"Tiles in stack · {_floorTiling.TileCount}\nLayout · {_floorTiling.Rows} × {_floorTiling.Columns}";
        _floorSideValue.text = $"Tile side · {_floorTiling.TileSideMeters * 1000f:0.#} mm";
        _floorThicknessValue.text = $"Thickness · {_floorTiling.TileThicknessMeters * 1000f:0.#} mm";
        _floorSourceYawValue.text = !_floorTiling.IsSourceCurrent ? "Capture source to show orientation"
            : _floorTiling.SourceFromCorners ? "Source edges from corners" : $"Source yaw · {_floorTiling.SourceYawDegrees:0}°";
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
        bool capturing = capture?.IsGuideOpen == true || _floorTiling.IsCapturingPoint;
        bool canEdit = _floorTiling.CanEdit;
        bool canEditPattern = canEdit && !capturing;
        _floorCaptureButton.interactable = canEditPattern;
        _floorCentreCaptureButton.interactable = canEditPattern;
        _floorPlacementButton.interactable = canEditPattern && _floorTiling.IsSourceCurrent;
        foreach (Button button in _floorDimensionButtons) button.interactable = canEditPattern;
        _floorSourceLeft.interactable = _floorSourceRight.interactable = canEditPattern && _floorTiling.IsSourceCurrent && !_floorTiling.SourceFromCorners;
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
        _floorCancelPoint.interactable = _floorTiling.IsCapturingPoint;
        _floorCaptureHelp.text = _floorTiling.IsSourceCurrent ? "Green stack and cyan cross show the source pick centre." : "Pick on the tile top. Place on the final support surface.";
        _floorStatus.text = _floorTiling.StatusText;
    }

    private void PositionFloorMenuPreview()
    {
        if (_floorTiling == null || _floorView == null || !_floorView.gameObject.activeInHierarchy) return;
        RectTransform menu = (RectTransform)_canvas.transform;
        float tileSide = _floorTiling.TileSideMeters;
        int rows = _floorTiling.Rows;
        float fullDepth = rows * tileSide + (rows - 1) * _floorTiling.MortarMillimeters * 0.001f;
        Vector3 top = menu.TransformPoint(new Vector3(0f, menu.rect.yMax, 0f));
        Vector3 worldCenter = top + menu.up * (0.08f + fullDepth * 0.25f);
        _floorTiling.ShowMenuPreview(menu, worldCenter, true);
    }

    private void HideFloorMenuPreview() => _floorTiling?.ShowMenuPreview(null, Vector3.zero, false);
}
