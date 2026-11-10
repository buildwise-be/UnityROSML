using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

/// <summary>Pages for the measured fixture, square-tile batch and scene-step review.</summary>
public sealed partial class ToolSelectionMenu
{
    private FixtureTilingController _fixtureTiling;
    private FixtureTilingController _subscribedFixtureTiling;
    private Button _fixtureTaskButton, _fixtureSetupButton;
    private Button _fixtureReviewButton;
    private RectTransform _fixtureView, _fixturePageContent;
    private TextMeshProUGUI _fixturePageTitle, _fixtureStatus, _fixtureCaptureHelp;
    private bool _showFixtureSetup;
    private int _fixturePage;
    private float _fixtureLengthStep = 0.001f;
    private readonly List<Action> _fixtureRefreshValues = new List<Action>();
    private readonly List<Button> _fixtureEditButtons = new List<Button>();
    private readonly string[] _fixturePageTitles =
    {
        "Tile dimensions", "Source stack", "Destination pattern", "Release region",
        "Retreat and settling", "Plank top corners", "Alignment corner", "Review task steps"
    };

    private void BindFixtureControls()
    {
        if (_workflowMode == URInputManager.WorkflowMode.SimplePickAndPlace ||
            (_input != null && _input.SelectedWorkflow == URInputManager.WorkflowMode.SimplePickAndPlace))
        {
            UnbindFixtureControls();
            _fixtureTiling = null;
            return;
        }
        if (_fixtureTiling == null && _input != null) _fixtureTiling = _input.FixtureController;
        if (_subscribedFixtureTiling == _fixtureTiling) return;
        UnbindFixtureControls();
        _subscribedFixtureTiling = _fixtureTiling;
        if (_subscribedFixtureTiling != null) _subscribedFixtureTiling.Changed += OnFixtureChanged;
        if (_fixtureView != null) BuildFixturePage();
    }

    private void UnbindFixtureControls()
    {
        if (_subscribedFixtureTiling != null) _subscribedFixtureTiling.Changed -= OnFixtureChanged;
        _subscribedFixtureTiling = null;
    }

    private void OnFixtureChanged()
    {
        RefreshFixtureMenu();
        RefreshTask();
    }

    private void BuildFixtureEntry()
    {
        if (_workflowMode == URInputManager.WorkflowMode.SimplePickAndPlace) return;
        if (_workflowMode == URInputManager.WorkflowMode.RepetitivePickAndPlace)
        {
            _fixtureSetupButton = CreateButton("Configure repetitive task", _taskView, new Vector2(16f, -204f),
                new Vector2(198f, 32f), out TextMeshProUGUI configure);
            configure.text = "Configure task";
            configure.fontSize = 16f;
            _fixtureSetupButton.onClick.AddListener(OpenFixtureSetup);
            _fixtureReviewButton = CreateButton("Review repetitive task", _taskView, new Vector2(226f, -204f),
                new Vector2(198f, 32f), out TextMeshProUGUI review);
            review.text = "Review task steps";
            review.fontSize = 16f;
            _fixtureReviewButton.onClick.AddListener(OpenFixtureReview);
            return;
        }
        _fixtureTaskButton = CreateButton("Fixture tiling", _taskView, new Vector2(294f, -204f),
            new Vector2(130f, 32f), out TextMeshProUGUI title);
        title.fontSize = 14f;
        _fixtureTaskButton.onClick.AddListener(SelectFixtureTask);
        _fixtureSetupButton = CreateButton("Fixture setup", _taskView, new Vector2(294f, -246f),
            new Vector2(130f, 32f), out TextMeshProUGUI setup);
        setup.fontSize = 14f;
        _fixtureSetupButton.onClick.AddListener(OpenFixtureSetup);
        _fixtureSetupButton.gameObject.SetActive(false);
    }

    public void OpenFixtureSetup()
    {
        if (_workflowMode == URInputManager.WorkflowMode.SimplePickAndPlace) return;
        BindTaskControls();
        if (IsPlacementBusy || _input == null) return;
        if (!_input.IsFixtureMode && !RobotExecutionState.IsBusy) _input.UseFixtureTiling();
        if (!_input.IsFixtureMode) return;
        _showFixtureSetup = true;
        BuildFixturePage();
        RefreshTask();
    }

    public void OpenFixtureReview()
    {
        _fixturePage = 7;
        OpenFixtureSetup();
    }

    private System.Collections.IEnumerator OpenRepetitiveSetupAfterStart()
    {
        // A controller created during Start loads its saved profile before the
        // next frame; build form callbacks against that loaded settings object.
        yield return null;
        if (_workflowMode == URInputManager.WorkflowMode.RepetitivePickAndPlace)
        {
            // Initial robot placement can still own the canvas. Keep setup
            // selected so it appears as soon as that placement is finished.
            _showFixtureSetup = true;
            BuildFixturePage();
            RefreshTask();
        }
    }

    public void CloseFixtureSetup()
    {
        if (_fixtureTiling != null) _fixtureTiling.CancelCapture();
        _showFixtureSetup = false;
        RefreshTask();
    }

    public void ChangeFixturePage(int delta)
    {
        _fixturePage = Mathf.Clamp(_fixturePage + delta, 0, _fixturePageTitles.Length - 1);
        BuildFixturePage();
        RefreshFixtureMenu();
    }

    private void BuildFixtureView()
    {
        if (_workflowMode == URInputManager.WorkflowMode.SimplePickAndPlace) return;
        _fixtureView = Rect("Fixture tiling setup view", _panel, Vector2.zero, new Vector2(440f, 756f));
        BuildHeader(_fixtureView, "Fixture setup title", _workflowMode == URInputManager.WorkflowMode.RepetitivePickAndPlace
            ? "Repetitive Pick and Place" : "Fixture tile batch");
        _fixturePageTitle = Text("Fixture page title", _fixtureView, new Vector2(64f, -57f),
            new Vector2(312f, 32f), string.Empty, 19f);
        _fixturePageTitle.alignment = TextAlignmentOptions.Center;
        FixtureButton("Previous page", _fixtureView, 16f, 54f, 42f, 36f, () => ChangeFixturePage(-1), "‹");
        FixtureButton("Next page", _fixtureView, 382f, 54f, 42f, 36f, () => ChangeFixturePage(1), "›");
        _fixtureStatus = Text("Fixture setup status", _fixtureView, new Vector2(16f, -543f),
            new Vector2(408f, 67f), string.Empty, 14f);
        _fixtureStatus.richText = false;
        _fixtureCaptureHelp = Text("Fixture capture instructions", _fixtureView, new Vector2(16f, -620f),
            new Vector2(408f, 68f), string.Empty, 15f);
        _fixtureCaptureHelp.richText = false;
        FixtureButton("Back to task", _fixtureView, 16f, 700f, 198f, 38f, CloseFixtureSetup);
        FixtureButton("Cancel point capture", _fixtureView, 226f, 700f, 198f, 38f,
            () => { if (_fixtureTiling != null) _fixtureTiling.CancelCapture(); RefreshFixtureMenu(); }, "Cancel capture");
        BuildFixturePage();
        _fixtureView.gameObject.SetActive(false);
    }

    private void BuildFixturePage()
    {
        if (_fixtureView == null) return;
        if (_fixturePageContent != null)
        {
            _fixturePageContent.gameObject.SetActive(false);
            if (Application.isPlaying) Destroy(_fixturePageContent.gameObject);
            else DestroyImmediate(_fixturePageContent.gameObject);
        }
        _fixtureRefreshValues.Clear();
        _fixtureEditButtons.Clear();
        _fixturePageContent = Rect("Fixture page content", _fixtureView, new Vector2(16f, -103f), new Vector2(408f, 428f));
        _fixturePageTitle.text = (_fixturePage + 1) + " / " + _fixturePageTitles.Length + " · " + _fixturePageTitles[_fixturePage];
        FixtureTilingSettings s = _fixtureTiling != null ? _fixtureTiling.Settings : new FixtureTilingSettings();
        FixtureCalibrationData c = _fixtureTiling != null ? _fixtureTiling.Calibration : new FixtureCalibrationData();
        switch (_fixturePage)
        {
            case 0:
                FixtureHint(0, "Square tiles · centred suction on the top face.\nSet measured dimensions before planning.");
                FixtureLength("Tile side", 64, () => s.ItemSize, v => s.ItemSize = v, 0.001f, 2f);
                FixtureLength("Tile thickness", 111, () => s.ItemThickness, v => s.ItemThickness = v, 0.001f, 2f);
                FixtureCount("Tiles in source", 158, () => s.SourceCount, v => s.SourceCount = v);
                FixtureCount("Tiles to place", 205, () => s.ItemCount, v => s.ItemCount = v);
                FixtureLength("Stack pitch (0 = thickness)", 252, () => s.StackPitch, v => s.StackPitch = v, 0, 2f);
                FixtureHint(303, "Top face is the square face; other pickup faces and rectangular bricks are not supported in this version.");
                FixtureSteps(373);
                break;
            case 1:
                FixtureHint(0, "Capture the centre of the top tile's upper face.\nThe stack is horizontal; yaw follows tile edges.");
                FixtureCapture("Capture stack top centre", FixtureCaptureTarget.SourceTop, 66);
                FixtureDynamicText("Stack capture status", 115, () => s.SourceTopPose == null
                    ? "Top centre: not captured" : "Top centre: captured in robot coordinates");
                FixtureHint(166, "Rotate the ghost pile until its edges match the real stack. This yaw is sent to ROS for collisions.");
                FixtureEdit("Yaw −45°", 0, 227, 96, () => _fixtureTiling.RotateSourceYaw(-45));
                FixtureEdit("Yaw −5°", 104, 227, 96, () => _fixtureTiling.RotateSourceYaw(-5));
                FixtureEdit("Yaw +5°", 208, 227, 96, () => _fixtureTiling.RotateSourceYaw(5));
                FixtureEdit("Yaw +45°", 312, 227, 96, () => _fixtureTiling.RotateSourceYaw(45));
                FixtureHint(292, "Lower contacts descend by the stack pitch. The initial scene keeps every source tile, including tiles left unused.");
                break;
            case 2:
                FixtureCount("Columns", 0, () => s.Columns, v => s.Columns = v);
                FixtureLength("Mortar gap", 47, () => s.Gap, v => s.Gap = v, 0, 2);
                FixtureStepper("Odd-row shift", 94, () => (s.RowOffsetFraction * 100).ToString("0") + "% pitch",
                    delta => s.RowOffsetFraction = Mathf.Clamp01(s.RowOffsetFraction + delta * 0.05f));
                FixtureCapture("Capture first tile corner", FixtureCaptureTarget.PlacementCorner, 147);
                FixtureCapture("Capture point along +X edge", FixtureCaptureTarget.PlacementX, 192);
                FixtureCapture("Capture point along +Y edge", FixtureCaptureTarget.PlacementY, 237);
                FixtureDynamicText("Pattern status", 284, () => s.PlacementCorner == null ? "Pattern frame: not captured" : "Pattern frame: captured");
                FixtureHint(326, "Corner is on the FINAL TOP FACE. X/Y follow pattern edges. Half-pitch shift includes the mortar gap; the last row may be partial.", 55);
                FixtureSteps(387);
                break;
            case 3:
                FixtureCapture("Capture release centre on plank", FixtureCaptureTarget.ReleaseCenter, 0);
                FixtureLength("Centre X from stop", 47, () => s.ReleaseCenterX, v => s.ReleaseCenterX = v, -10, 10);
                FixtureLength("Centre Y from stop", 94, () => s.ReleaseCenterY, v => s.ReleaseCenterY = v, -10, 10);
                FixtureLength("Region width", 141, () => s.ReleaseWidth, v => s.ReleaseWidth = v, 0, 10);
                FixtureLength("Region depth", 188, () => s.ReleaseDepth, v => s.ReleaseDepth = v, 0, 10);
                FixtureLength("Tile-bottom clearance", 235, () => s.ReleaseClearance, v => s.ReleaseClearance = v, 0, 1);
                FixtureCount("Yaw alternatives", 282, () => s.ReleaseYawSamples, v => s.ReleaseYawSamples = v, 1, 12);
                FixtureHint(329, "Region bounds tile CENTRES in fixture X/Y. Clearance follows the plank normal. ROS checks full footprints and the slide envelope.", 55);
                FixtureSteps(387);
                break;
            case 4:
                FixtureLength("Retreat distance", 0, () => s.RetreatDistance, v => s.RetreatDistance = v, 0.001f, 1);
                FixtureStepper("Settling wait", 47, () => s.SettlingDelay.ToString("0.0") + " s",
                    delta => s.SettlingDelay = Mathf.Clamp(s.SettlingDelay + delta * 0.5f, 0, 60));
                FixtureEdit("Toggle cycle home", 0, 102, 408, () => { s.ReturnHomeEachTile = !s.ReturnHomeEachTile; _fixtureTiling.NotifyEdited(); });
                FixtureDynamicText("Cycle home setting", 150, () => s.ReturnHomeEachTile ? "Return to configured home after every tile: ON" : "Return home after every tile: OFF");
                FixtureHint(212, "Release vacuum → withdraw along plank normal → wait → exact regrasp → place. The extra wait starts AFTER withdrawal.");
                FixtureHint(291, "Automatic continuation assumes the tile has seated in the corner. There is no alignment sensing. Home returns use the updated scene.");
                FixtureSteps(373);
                break;
            case 5:
                FixtureHint(0, "After robot alignment, capture four TOP corners around the plank: P00 → P10 → P11 → P01.");
                FixtureCapture("1 · P00", FixtureCaptureTarget.Board00, 68, 0, 198);
                FixtureCapture("2 · P10", FixtureCaptureTarget.Board10, 68, 210, 198);
                FixtureCapture("3 · P11", FixtureCaptureTarget.Board11, 113, 0, 198);
                FixtureCapture("4 · P01", FixtureCaptureTarget.Board01, 113, 210, 198);
                FixtureDynamicText("Board capture status", 164, () => "Top corners captured: " + CapturedBoardCorners(c) + " / 4");
                FixtureLength("Plank thickness", 214, () => c.BoardThickness, v => c.BoardThickness = v, 0.001f, 1);
                FixtureLength("Board-edge margin", 261, () => c.Margin, v => c.Margin = v, 0, 1);
                FixtureEdit("Fit plank rectangle", 0, 310, 408, () => _fixtureTiling.FitCalibration());
                FixtureHint(355, "Pointer tip marks the top. Fit and inspect its error before capturing the alignment corner.", 32);
                FixtureSteps(387);
                break;
            case 6:
                FixtureHint(0, "Capture the stop's INNER corner on the plank support surface, then a point along each arm into the tray.");
                FixtureCapture("Capture alignment corner", FixtureCaptureTarget.SupportCorner, 68);
                FixtureCapture("Capture point along +X arm", FixtureCaptureTarget.SupportX, 113);
                FixtureCapture("Capture point along +Y arm", FixtureCaptureTarget.SupportY, 158);
                FixtureDynamicText("Support capture status", 206, () => c.SupportCorner == null ? "Alignment frame: not captured" : "Alignment frame: captured");
                FixtureHint(256, "Gravity must pull toward BOTH stop arms. The metal stop is an alignment reference; the plank is the modeled fixture obstacle.");
                FixtureEdit("Save fixture to ROS", 0, 330, 408, () => _fixtureTiling.SaveCalibration());
                FixtureHint(376, "Save persists ROS calibration for the next MTC startup. An operator must restart/reload the backend before it takes effect.", 52);
                break;
            case 7:
                FixtureHint(0, "Robot path omits vacuum/waits; inspect steps.\nReview initial inventory, release, settled rest, attachments and final placement.");
                FixtureButton("Previous task step", _fixturePageContent, 0, 75, 198, 38,
                    () => { if (_fixtureTiling != null) _fixtureTiling.StepReview(-1); });
                FixtureButton("Next task step", _fixturePageContent, 210, 75, 198, 38,
                    () => { if (_fixtureTiling != null) _fixtureTiling.StepReview(1); });
                FixtureDynamicText("Task step detail", 128, () => _fixtureTiling == null ? "Plan a batch to review its steps." : _fixtureTiling.ReviewStepText, 188);
                Button plan = FixtureButton("Plan fixture batch", _fixturePageContent, 0, 334, 198, 42, PlanTask, "Plan preview");
                Button execute = FixtureButton("Execute fixture batch", _fixturePageContent, 210, 334, 198, 42, ExecuteTask, "Execute plan");
                Button stop = FixtureButton("Stop fixture planning", _fixturePageContent, 210, 334, 198, 42, StopPlanningTask, "Stop Planning");
                _fixtureRefreshValues.Add(() =>
                {
                    bool motionFree = !RobotExecutionState.IsBusy && MTCPlansManager.ActiveExecution == null;
                    plan.interactable = _fixtureTiling != null && !_fixtureTiling.IsPlanning && motionFree && !IsPlacementBusy;
                    execute.interactable = _fixtureTiling != null && _fixtureTiling.CanExecute && motionFree && !IsPlacementBusy;
                    RefreshStopPlanningButton(stop, execute, motionFree && _fixtureTiling?.CanStopPlanning == true,
                        _fixtureTiling?.IsStoppingPlanning == true);
                });
                FixtureHint(386, "Single-use execution. Check inventory and scene after an attempt before creating a new job.", 42);
                break;
        }
        RefreshFixtureMenu();
    }

    private static int CapturedBoardCorners(FixtureCalibrationData data)
    {
        int count = 0;
        if (data.BoardTopCorners != null) foreach (var point in data.BoardTopCorners) if (point != null) count++;
        return count;
    }

    private void FixtureHint(float y, string message, float height = 61)
    {
        Text("Fixture instruction", _fixturePageContent, new Vector2(0, -y), new Vector2(408, height), message, 14f)
            .color = new Color(0.73f, 0.81f, 0.87f);
    }

    private void FixtureDynamicText(string name, float y, Func<string> value, float height = 40)
    {
        TextMeshProUGUI text = Text(name, _fixturePageContent, new Vector2(0, -y), new Vector2(408, height), value(), 15f);
        text.richText = false;
        _fixtureRefreshValues.Add(() => text.text = value());
    }

    private Button FixtureButton(string name, Transform parent, float x, float y, float width, float height, UnityAction action, string label = null)
    {
        Button button = CreateButton(name, parent, new Vector2(x, -y), new Vector2(width, height), out TextMeshProUGUI text);
        text.text = label ?? name;
        text.fontSize = 15f;
        button.onClick.AddListener(action);
        return button;
    }

    private void FixtureEdit(string name, float x, float y, float width, UnityAction action)
    {
        _fixtureEditButtons.Add(FixtureButton(name, _fixturePageContent, x, y, width, 38, () =>
        {
            if (!CanEditFixture()) return;
            action();
            RefreshFixtureMenu();
        }));
    }

    private void FixtureCapture(string name, FixtureCaptureTarget target, float y, float x = 0, float width = 408)
        => FixtureEdit(name, x, y, width, () => _fixtureTiling.BeginCapture(target));

    private void FixtureLength(string name, float y, Func<float> value, Action<float> set, float min, float max)
        => FixtureStepper(name, y, () => (value() * 1000).ToString("0.#") + " mm",
            delta => set(Mathf.Clamp(value() + delta * _fixtureLengthStep, min, max)));

    private void FixtureCount(string name, float y, Func<int> value, Action<int> set, int min = 1, int max = 64)
        => FixtureStepper(name, y, () => value().ToString(), delta => set(Mathf.Clamp(value() + delta, min, max)));

    private void FixtureStepper(string name, float y, Func<string> value, Action<int> change)
    {
        Text(name + " heading", _fixturePageContent, new Vector2(0, -y), new Vector2(202, 38), name, 14f);
        TextMeshProUGUI current = Text(name + " value", _fixturePageContent, new Vector2(204, -y), new Vector2(108, 38), value(), 15f);
        current.alignment = TextAlignmentOptions.Center;
        _fixtureRefreshValues.Add(() => current.text = value());
        for (int sign = -1; sign <= 1; sign += 2)
        {
            int delta = sign;
            FixtureEdit(name + (sign < 0 ? " −" : " +"), sign < 0 ? 316 : 364, y, 44, () =>
            {
                change(delta);
                _fixtureTiling.NotifyEdited();
            });
            Button button = _fixtureEditButtons[_fixtureEditButtons.Count - 1];
            button.GetComponentInChildren<TextMeshProUGUI>().text = sign < 0 ? "−" : "+";
        }
    }

    private void FixtureSteps(float y)
    {
        Text("Length increment", _fixturePageContent, new Vector2(0, -y), new Vector2(126, 32), "Length step", 14f);
        float[] steps = { 0.0001f, 0.001f, 0.01f, 0.1f };
        for (int i = 0; i < steps.Length; i++)
        {
            float step = steps[i];
            Button button = FixtureButton((step * 1000).ToString("0.#") + " mm step", _fixturePageContent,
                132 + i * 70, y, 66, 32, () => { _fixtureLengthStep = step; RefreshFixtureMenu(); }, (step * 1000).ToString("0.#") + " mm");
            _fixtureRefreshValues.Add(() => button.image.color = Mathf.Approximately(_fixtureLengthStep, step)
                ? new Color(0.08f, 0.42f, 0.52f, 1) : new Color(0.12f, 0.17f, 0.23f, 1));
        }
    }

    private bool CanEditFixture() => _fixtureTiling != null && _input != null && _input.IsFixtureMode &&
        !IsPlacementBusy && !RobotExecutionState.IsBusy && MTCPlansManager.ActiveExecution == null;

    private void RefreshFixtureMenu()
    {
        bool visible = _showFixtureSetup && _input != null && _input.IsFixtureMode && !IsPlacementBusy;
        if (_fixtureView != null) _fixtureView.gameObject.SetActive(visible);
        if (_showFixtureSetup && (_input == null || !_input.IsFixtureMode)) _showFixtureSetup = false;
        foreach (Action refresh in _fixtureRefreshValues) refresh();
        foreach (Button button in _fixtureEditButtons) if (button != null) button.interactable = CanEditFixture();
        if (_fixtureStatus != null) _fixtureStatus.text = _fixtureTiling != null
            ? (_fixturePage == 5 || _fixturePage == 6 ? _fixtureTiling.CalibrationStatus : _fixtureTiling.StatusText)
            : "Select Fixture tiling in the robot task menu.";
        if (_fixtureCaptureHelp != null) _fixtureCaptureHelp.text = _fixtureTiling != null && _fixtureTiling.IsCapturing
            ? _fixtureTiling.CaptureInstruction + "\nMove the pointer tip to the real point; press trigger."
            : "Choose a capture button, point at the real location, then press trigger. Robot alignment must be valid.";
    }
}
