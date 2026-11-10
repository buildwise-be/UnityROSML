using TMPro;
using UnityEngine;
using UnityEngine.UI;

public sealed partial class ToolSelectionMenu
{
    private RectTransform _repeatConfirmationView;
    private PickPlaceController _repeatConfirmationController;
    private string _repeatConfirmationKey;

    private bool CanShowRepeatConfirmation(PickPlaceController controller) =>
        IsMtcPickMode && !IsPlacementBusy && !IsShowingPickSettings && controller != null &&
        _input.PickPlaceController == controller;

    private void OpenRepeatConfirmation()
    {
        PickPlaceController controller = _subscribedPickSettings;
        if (_panel == null || !CanShowRepeatConfirmation(controller)) return;
        string key = controller.RepeatConfirmationKey;
        if (string.IsNullOrEmpty(key)) return;
        if (_repeatConfirmationView != null && _repeatConfirmationController == controller && _repeatConfirmationKey == key) return;
        ClearRepeatConfirmation();

        // A fresh view gives each prompt its own click target and captured key.
        RectTransform view = Rect("Repeat execution confirmation", _panel, Vector2.zero, new Vector2(440f, 756f));
        _repeatConfirmationView = view;
        _repeatConfirmationController = controller;
        _repeatConfirmationKey = key;
        BuildHeader(view, "Repeat confirmation title", "Are you sure?");
        TextMeshProUGUI message = Text("Repeat confirmation message", view, new Vector2(16f, -64f), new Vector2(408f, 124f),
            "The previous run should have placed a " + controller.RepeatItemLabel +
            " at the place location, re-executing will thus end up in a collision", 20f);
        message.color = new Color(1f, 0.84f, 0.57f);
        message.richText = false;
        Button yes = CreateButton("Yes, I am sure, I removed it", view, new Vector2(16f, -206f), new Vector2(408f, 56f), out var yesLabel);
        yesLabel.fontSize = 18f;
        yes.onClick.AddListener(() => ConfirmRepeatFromView(view, controller, key));
        Button no = CreateButton("No, cancel", view, new Vector2(16f, -278f), new Vector2(408f, 48f), out var noLabel);
        noLabel.fontSize = 18f;
        no.onClick.AddListener(() =>
        {
            if (_repeatConfirmationView == view) CancelRepeatConfirmation();
        });
        RefreshTask();
    }

    private void ConfirmRepeatFromView(RectTransform view, PickPlaceController controller, string key)
    {
        if (view == null || _repeatConfirmationView != view) return;
        bool current = CanShowRepeatConfirmation(controller) && key == controller.RepeatConfirmationKey;
        ClearRepeatConfirmation();
        if (current) controller.ConfirmRepeatExecution(key);
        RefreshTask();
    }

    private bool CloseStaleRepeatConfirmation()
    {
        if (_repeatConfirmationView == null || (CanShowRepeatConfirmation(_repeatConfirmationController) &&
            _repeatConfirmationKey == _repeatConfirmationController.RepeatConfirmationKey)) return false;
        ClearRepeatConfirmation();
        return true;
    }

    private void CancelRepeatConfirmation()
    {
        if (_repeatConfirmationView == null) return;
        ClearRepeatConfirmation();
        RefreshTask();
    }

    private void ClearRepeatConfirmation()
    {
        RectTransform view = _repeatConfirmationView;
        _repeatConfirmationView = null;
        _repeatConfirmationController = null;
        _repeatConfirmationKey = null;
        if (view == null) return;
        view.gameObject.SetActive(false);
        if (Application.isPlaying) Destroy(view.gameObject);
        else DestroyImmediate(view.gameObject);
    }
}
