using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>Recovery from a rejected contact fit and bounded, translation-only overlay correction.</summary>
public sealed partial class ToolSelectionMenu
{
    private RectTransform _fitFailureView, _positionAdjustmentView;
    private TextMeshProUGUI _fitFailureReason, _positionAdjustmentStatus, _positionAdjustmentValue;
    private readonly List<Button> _positionStepButtons = new List<Button>();
    private readonly List<Button> _positionNudgeButtons = new List<Button>();
    private readonly float[] _positionStepsMeters = { .0001f, .001f, .005f };
    private int _positionStepIndex = 1;
    private Button _positionResetButton, _positionDoneButton;

    public void BeginPositionAdjustment()
    {
        BindTaskControls();
        if (_pointRegistration == null || RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null) return;
        _localNotice = null;
        _pointRegistration.BeginPositionAdjustment();
        RefreshMenu();
    }

    public void EndPositionAdjustment()
    {
        if (_pointRegistration != null) _pointRegistration.EndPositionAdjustment();
        RefreshMenu();
    }

    private void NudgePosition(int axis, float sign)
    {
        if (!IsPositionAdjusting) return;
        Vector3 rosDelta = axis == 0 ? Vector3.right : axis == 1 ? Vector3.up : Vector3.forward;
        _pointRegistration.NudgeRobotPosition(rosDelta * sign * _positionStepsMeters[_positionStepIndex]);
        RefreshMenu();
    }

    private void ResetPositionAdjustment()
    {
        if (IsPositionAdjusting) _pointRegistration.ResetPositionAdjustment();
        RefreshMenu();
    }

    private void BuildPointRegistrationPanels()
    {
        _fitFailureView = Rect("Rejected point-fit recovery", _panel, Vector2.zero, new Vector2(440, 756));
        BuildHeader(_fitFailureView, "Rejected fit title", "Robot fit rejected");
        Text("Rejected fit introduction", _fitFailureView, new Vector2(16,-60), new Vector2(408,56),
            "The measured points did not produce an accepted robot alignment.", 19);
        _fitFailureReason = Text("Rejected fit reason", _fitFailureView, new Vector2(16,-133),
            new Vector2(408,190), string.Empty, 19);
        _fitFailureReason.richText = false;
        _fitFailureReason.enableAutoSizing = true;
        _fitFailureReason.fontSizeMin = 14;
        _fitFailureReason.fontSizeMax = 19;
        _fitFailureReason.color = new Color(1f,.67f,.45f);
        Text("Rejected fit recovery instructions", _fitFailureView, new Vector2(16,-341), new Vector2(408,107),
            "Check the five targets and their order. Use the verified controller tip and keep the robot still. Undo the last point to recapture it, or restart all five.", 17);
        RegistrationButton("Undo last point", _fitFailureView, new Vector2(16,-475), new Vector2(198,42), UndoPointPlacement);
        RegistrationButton("Restart 5 points", _fitFailureView, new Vector2(226,-475), new Vector2(198,42), BeginPointPlacement);
        RegistrationButton("Cancel placement", _fitFailureView, new Vector2(16,-538), new Vector2(198,42), CancelPointPlacement);
        RegistrationButton("Use QR instead", _fitFailureView, new Vector2(226,-538), new Vector2(198,42), UseQrPlacement);
        Text("Rejected fit planning block", _fitFailureView, new Vector2(16,-611), new Vector2(408,65),
            "Planning stays blocked until robot alignment is valid. A rejected fit does not place the robot.", 16);
        RegistrationButton("Recenter menu", _fitFailureView, new Vector2(16,-689), new Vector2(408,30), RecenterMenu);

        _positionAdjustmentView = Rect("Robot position adjustment", _panel, Vector2.zero, new Vector2(440,756));
        BuildHeader(_positionAdjustmentView, "Robot position title", "Fine-tune robot position");
        Text("Robot position introduction", _positionAdjustmentView, new Vector2(16,-57), new Vector2(408,60),
            "Move the XR robot overlay along its fitted ROS base axes. Planning is paused while you adjust.", 17);
        _positionAdjustmentStatus = Text("Robot position status", _positionAdjustmentView, new Vector2(16,-129),
            new Vector2(408,73), string.Empty, 16);
        _positionAdjustmentStatus.richText = false;
        _positionAdjustmentStatus.enableAutoSizing = true;
        _positionAdjustmentStatus.fontSizeMin = 13;
        _positionAdjustmentStatus.fontSizeMax = 16;
        _positionAdjustmentValue = Text("Robot position correction", _positionAdjustmentView, new Vector2(16,-215),
            new Vector2(408,58), string.Empty, 18);
        Text("Robot position step heading", _positionAdjustmentView, new Vector2(16,-281), new Vector2(408,25), "Move increment", 17);
        string[] labels = { "0.1 mm", "1 mm", "5 mm" };
        for (int index = 0; index < labels.Length; index++)
        {
            int selected = index;
            Button button = RegistrationButton(labels[index] + " position step", _positionAdjustmentView,
                new Vector2(16 + index * 139,-313), new Vector2(130,32), () =>
                { _positionStepIndex = selected; RefreshPointRegistrationPanels(); }, labels[index]);
            _positionStepButtons.Add(button);
        }
        string[] axes = { "X", "Y", "Z" };
        for (int axis = 0; axis < axes.Length; axis++)
        {
            int selected = axis;
            float y = -366 - axis * 53;
            Text("Robot base " + axes[axis] + " label", _positionAdjustmentView, new Vector2(16,y), new Vector2(128,40),
                "Robot base " + axes[axis], 17);
            _positionNudgeButtons.Add(RegistrationButton(axes[axis] + " minus", _positionAdjustmentView,
                new Vector2(155,y), new Vector2(130,40), () => NudgePosition(selected,-1), axes[axis] + " −"));
            _positionNudgeButtons.Add(RegistrationButton(axes[axis] + " plus", _positionAdjustmentView,
                new Vector2(294,y), new Vector2(130,40), () => NudgePosition(selected,1), axes[axis] + " +"));
        }
        Text("Robot position limit", _positionAdjustmentView, new Vector2(16,-531), new Vector2(408,70),
            "Maximum correction: 20 mm total from the original fit, across all adjustment sessions. Reset returns to that fit. Rotation stays fixed.", 16);
        _positionResetButton = RegistrationButton("Reset to point fit", _positionAdjustmentView,
            new Vector2(16,-624), new Vector2(198,42), ResetPositionAdjustment);
        _positionDoneButton = RegistrationButton("Done adjusting", _positionAdjustmentView,
            new Vector2(226,-624), new Vector2(198,42), EndPositionAdjustment);
        RegistrationButton("Recenter menu", _positionAdjustmentView, new Vector2(16,-689), new Vector2(408,30), RecenterMenu);
        RefreshPointRegistrationPanels();
    }

    private Button RegistrationButton(string name, Transform parent, Vector2 position, Vector2 size,
        UnityEngine.Events.UnityAction action, string title = null)
    {
        Button button = CreateButton(name,parent,position,size,out TextMeshProUGUI text);
        text.text = title ?? name;
        text.fontSize = 16;
        button.onClick.AddListener(action);
        return button;
    }

    private void RefreshPointRegistrationPanels()
    {
        bool failure = IsPointCollecting && _pointRegistration != null && _pointRegistration.HasFitFailure;
        bool adjusting = IsPositionAdjusting;
        if (_fitFailureView != null) _fitFailureView.gameObject.SetActive(failure && !adjusting);
        if (_positionAdjustmentView != null) _positionAdjustmentView.gameObject.SetActive(adjusting);
        if (_fitFailureReason != null) _fitFailureReason.text = failure ? _pointRegistration.FitFailureText : string.Empty;
        if (_positionAdjustmentStatus != null) _positionAdjustmentStatus.text = adjusting ? _pointRegistration.StatusText : string.Empty;
        if (_positionAdjustmentValue != null)
        {
            Vector3 delta = _pointRegistration != null ? _pointRegistration.PositionAdjustmentInRosMeters * 1000 : Vector3.zero;
            _positionAdjustmentValue.text = "X " + delta.x.ToString("+0.0;-0.0;0.0") + " mm   Y " +
                delta.y.ToString("+0.0;-0.0;0.0") + " mm   Z " + delta.z.ToString("+0.0;-0.0;0.0") +
                " mm\nTotal correction: " + delta.magnitude.ToString("0.0") + " / 20.0 mm";
        }
        bool canMove = adjusting && !RobotExecutionState.IsBusy && MTCPlansManager.ActiveExecution == null;
        RefreshStepButtons(_positionStepButtons,_positionStepIndex,canMove);
        foreach (Button button in _positionNudgeButtons) if (button != null) button.interactable = canMove;
        if (_positionResetButton != null) _positionResetButton.interactable = canMove;
        if (_positionDoneButton != null) _positionDoneButton.interactable = canMove;
        if (_alignButton != null)
        {
            bool pointPlaced = _pointRegistration != null && _pointRegistration.HasPlacement;
            TextMeshProUGUI label = _alignButton.GetComponentInChildren<TextMeshProUGUI>();
            if (label != null) label.text = pointPlaced ? "Adjust XYZ" : "Align robot";
            if (pointPlaced) _alignButton.interactable = !IsPlacementBusy && !RobotExecutionState.IsBusy &&
                MTCPlansManager.ActiveExecution == null && _pointRegistration.CanAdjustPosition;
        }
    }
}
