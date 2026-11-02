using UnityEngine;
using UnityEngine.UI;
using TMPro;

/// <summary>Scene-specific presentation and a guarded route back to the welcome scene.</summary>
public sealed partial class ToolSelectionMenu
{
    private Button _welcomeButton;

    /// <summary>Configure before Start so each scene builds only its workflow's controls.</summary>
    public void ConfigureWorkflow(URInputManager.WorkflowMode workflow)
    {
        _workflowMode = workflow;
        if (_input != null) _input.ConfigureWorkflow(workflow);
        if (_panel != null) RefreshTask();
    }

    private void BuildSceneNavigation()
    {
        if (_workflowMode == URInputManager.WorkflowMode.Unrestricted) return;
        // Keep the full calibration/status area available for measured values.
        // A compact header route leaves every lower control in its usual place.
        TextMeshProUGUI title = _taskView.Find("Title")?.GetComponent<TextMeshProUGUI>();
        if (title != null)
        {
            title.rectTransform.sizeDelta = new Vector2(_appLogo != null ? 262f : 312f, 30f);
            title.fontSize = 20f;
        }
        _welcomeButton = CreateButton("Return to welcome", _taskView, new Vector2(338f, -12f),
            new Vector2(86f, 28f), out var label);
        label.text = "Welcome";
        label.fontSize = 14f;
        _welcomeButton.onClick.AddListener(ReturnToWelcome);
        RefreshSceneNavigation();
    }

    private void RefreshSceneNavigation()
    {
        if (_welcomeButton != null) _welcomeButton.interactable = RobotAppNavigation.CanChangeScene;
    }

    public void ReturnToWelcome()
    {
        if (!RobotAppNavigation.CanChangeScene) return;
        RobotAppNavigation.ReturnToWelcome();
    }
}
