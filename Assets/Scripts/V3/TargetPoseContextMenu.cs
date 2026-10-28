using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>Local target editing only. Bumper opens the hovered target's menu; no ROS publication.</summary>
public sealed class TargetPoseContextMenu : MonoBehaviour
{
    public const float DepthStepMetres = 0.005f;
    private static TargetPoseContextMenu _active;
    public static bool IsOpen => _active != null && _active._canvas != null && _active._canvas.gameObject.activeSelf;
    public static bool IsHovered => IsOpen && (_active._hovering.Count != 0 || _active._pressing.Count != 0);

    private string _title;
    private bool _allowDepth;
    private Action _changed;
    private Canvas _canvas;
    private TextMeshProUGUI _depthLabel;
    private readonly HashSet<int> _hovering = new HashSet<int>();
    private readonly HashSet<int> _pressing = new HashSet<int>();
    private Vector3 _depthShiftInParent;
    private Transform _offsetParent;
    private int _depthSteps;
    private float _lastPressTime;

    public int DepthSteps => _depthSteps;
    public float DepthOffsetMetres => _depthSteps * DepthStepMetres;
    public bool AllowsDepthAdjustment => _allowDepth;

    public static TargetPoseContextMenu Attach(Transform target, string title,
        bool allowDepthAdjustment, Action changed)
    {
        if (target == null) return null;
        TargetPoseContextMenu menu = target.GetComponent<TargetPoseContextMenu>();
        if (menu == null) menu = target.gameObject.AddComponent<TargetPoseContextMenu>();
        menu._title = title;
        menu._allowDepth = allowDepthAdjustment;
        menu._changed = changed;
        return menu;
    }

    /// <summary>Call after assigning a new captured pose, so old depth offsets never leak into a recapture.</summary>
    public void RebaseAtCurrentPose()
    {
        Close();
        _offsetParent = transform.parent;
        _depthShiftInParent = Vector3.zero;
        _depthSteps = 0;
        RefreshDepthLabel();
    }

    public static bool TryOpenHovered()
    {
        if (IsOpen || RobotCoordinateService.IsCalibrationEditActive || TargetRotationHandles.IsInteracting)
            return IsOpen;
        if (!TargetRotationHandles.TryGetHoveredTarget(out Transform target)) return false;
        TargetPoseContextMenu menu = target.GetComponent<TargetPoseContextMenu>();
        return menu != null && menu.Open();
    }

    public bool Open()
    {
        if (!isActiveAndEnabled || RobotCoordinateService.IsCalibrationEditActive ||
            TargetRotationHandles.IsInteracting || !RobotFrameMath.IsRigidFrame(transform)) return false;
        if (_active != null && _active != this) _active.Close();
        if (_canvas == null) BuildMenu();
        Camera viewer = Camera.main;
        if (viewer == null) return false;
        _active = this;
        _canvas.worldCamera = viewer;
        // The panel stays still while its buttons move the target. It follows the XR origin, not the target.
        _canvas.transform.SetParent(transform.parent, true);
        Vector3 side = Vector3.ProjectOnPlane(viewer.transform.right, Vector3.up).normalized;
        if (side.sqrMagnitude < 0.1f) side = Vector3.right;
        _canvas.transform.position = transform.position + side * 0.24f + Vector3.up * 0.17f;
        Vector3 facing = _canvas.transform.position - viewer.transform.position;
        if (facing.sqrMagnitude < 0.0001f) facing = viewer.transform.forward;
        _canvas.transform.rotation = Quaternion.LookRotation(facing, Vector3.up);
        _canvas.gameObject.SetActive(true);
        RefreshDepthLabel();
        return true;
    }

    public void Close()
    {
        if (_canvas != null) _canvas.gameObject.SetActive(false);
        _hovering.Clear();
        _pressing.Clear();
        if (_active == this) _active = null;
    }

    public static void CloseAll() { if (_active != null) _active.Close(); }

    public bool NudgeDepth(int steps)
    {
        if (steps == 0 || !_allowDepth || !isActiveAndEnabled || RobotCoordinateService.IsCalibrationEditActive ||
            TargetRotationHandles.IsInteracting || !RobotFrameMath.IsRigidFrame(transform)) return false;
        if (_offsetParent != transform.parent) RebaseAtCurrentPose();
        // A click is one step, even when this public entry point receives an arbitrary integer.
        int direction = Math.Sign(steps);
        if ((_depthSteps == int.MaxValue && direction > 0) || (_depthSteps == int.MinValue && direction < 0)) return false;
        Vector3 shift = RobotFrameMath.TcpPositiveZ(new Pose(transform.position, transform.rotation)) *
            (direction * DepthStepMetres);
        Pose next = new Pose(transform.position + shift, transform.rotation);
        if (!RobotFrameMath.IsValidPose(next)) return false;
        Vector3 localShift = transform.parent != null ? transform.parent.InverseTransformVector(shift) : shift;
        transform.position = next.position;
        _depthShiftInParent += localShift;
        _depthSteps += direction;
        RefreshDepthLabel();
        _changed?.Invoke();
        return true;
    }

    /// <summary>Undo context-menu translations, including when the tool was rotated between steps.</summary>
    public void ResetDepth()
    {
        if (!_allowDepth || !isActiveAndEnabled || RobotCoordinateService.IsCalibrationEditActive ||
            TargetRotationHandles.IsInteracting || _depthSteps == 0 && _depthShiftInParent == Vector3.zero) return;
        if (_offsetParent != transform.parent) { RebaseAtCurrentPose(); return; }
        if (!RobotFrameMath.IsRigidFrame(transform)) return;
        transform.localPosition -= _depthShiftInParent;
        _depthShiftInParent = Vector3.zero;
        _depthSteps = 0;
        RefreshDepthLabel();
        _changed?.Invoke();
    }

    private void RefreshDepthLabel()
    {
        if (_depthLabel == null) return;
        _depthLabel.text = _allowDepth
            ? "Shift from capture: " + (_depthShiftInParent.magnitude * 1000f).ToString("0") +
                " mm\n+Z pushes through the suction face; −Z backs off."
            : "Exact TCP position and rotation.\nClose, then trigger-drag the tool or curved arrows to turn it.";
    }

    private void BuildMenu()
    {
        GameObject go = new GameObject(_title + " context", typeof(RectTransform), typeof(Canvas),
            typeof(CanvasScaler), typeof(TrackedDeviceGraphicRaycaster));
        _canvas = go.GetComponent<Canvas>();
        _canvas.renderMode = RenderMode.WorldSpace;
        RectTransform panel = (RectTransform)go.transform;
        panel.sizeDelta = new Vector2(360f, _allowDepth ? 270f : 200f);
        panel.pivot = new Vector2(0.5f, 0.5f);
        panel.localScale = Vector3.one * 0.001f;
        Image background = go.AddComponent<Image>();
        background.color = new Color(0.035f, 0.08f, 0.11f, 0.97f);
        TrackPointer(go);
        Text(panel, "Title", _title, new Vector2(16f, -12f), new Vector2(328f, 34f), 23f);
        _depthLabel = Text(panel, "Depth", "", new Vector2(16f, -51f), new Vector2(328f, 68f), 17f);
        if (_allowDepth)
        {
            Button(panel, "−Z  5 mm", new Vector2(16f, -126f), new Vector2(158f, 43f), () => NudgeDepth(-1));
            Button(panel, "+Z  5 mm", new Vector2(186f, -126f), new Vector2(158f, 43f), () => NudgeDepth(1));
            Button(panel, "Reset depth", new Vector2(16f, -181f), new Vector2(158f, 40f), ResetDepth);
            Button(panel, "Close", new Vector2(186f, -181f), new Vector2(158f, 40f), Close);
            Text(panel, "Hint", "Close to turn the ball; depth steps follow the arrow.",
                new Vector2(16f, -228f), new Vector2(328f, 30f), 14f);
        }
        else Button(panel, "Close", new Vector2(16f, -135f), new Vector2(328f, 43f), Close);
        go.SetActive(false);
    }

    private void Button(Transform parent, string label, Vector2 position, Vector2 size, Action clicked)
    {
        RectTransform rect = Rect(parent, label, position, size);
        Image image = rect.gameObject.AddComponent<Image>();
        image.color = new Color(0.12f, 0.25f, 0.31f, 1f);
        UnityEngine.UI.Button button = rect.gameObject.AddComponent<UnityEngine.UI.Button>();
        button.targetGraphic = image;
        button.onClick.AddListener(() => { if (!RobotCoordinateService.IsCalibrationEditActive) clicked(); });
        TextMeshProUGUI text = Text(rect, "Label", label, Vector2.zero, size, 20f);
        text.alignment = TextAlignmentOptions.Center;
        TrackPointer(rect.gameObject);
    }

    private static RectTransform Rect(Transform parent, string name, Vector2 position, Vector2 size)
    {
        RectTransform rect = new GameObject(name, typeof(RectTransform)).GetComponent<RectTransform>();
        rect.SetParent(parent, false);
        rect.anchorMin = rect.anchorMax = rect.pivot = new Vector2(0f, 1f);
        rect.anchoredPosition = position;
        rect.sizeDelta = size;
        return rect;
    }

    private static TextMeshProUGUI Text(Transform parent, string name, string value,
        Vector2 position, Vector2 size, float fontSize)
    {
        TextMeshProUGUI text = Rect(parent, name, position, size).gameObject.AddComponent<TextMeshProUGUI>();
        text.text = value;
        text.fontSize = fontSize;
        text.color = Color.white;
        text.textWrappingMode = TextWrappingModes.Normal;
        text.overflowMode = TextOverflowModes.Ellipsis;
        text.raycastTarget = false;
        return text;
    }

    private void TrackPointer(GameObject go)
    {
        EventTrigger trigger = go.AddComponent<EventTrigger>();
        Add(trigger, EventTriggerType.PointerEnter, data => _hovering.Add(data.pointerId));
        Add(trigger, EventTriggerType.PointerExit, data => _hovering.Remove(data.pointerId));
        Add(trigger, EventTriggerType.PointerDown, data => { _pressing.Add(data.pointerId); _lastPressTime = Time.unscaledTime; });
        Add(trigger, EventTriggerType.PointerUp, data => _pressing.Remove(data.pointerId));
    }

    private static void Add(EventTrigger trigger, EventTriggerType type, Action<PointerEventData> action)
    {
        EventTrigger.Entry entry = new EventTrigger.Entry { eventID = type };
        entry.callback.AddListener(data => { if (data is PointerEventData pointer) action(pointer); });
        trigger.triggers.Add(entry);
    }

    private void Update()
    {
        if (_active != this) return;
        if (RobotCoordinateService.IsCalibrationEditActive || Camera.main == null) Close();
        // Lost pointer-up events after a tracking interruption must not leave a permanent press latch.
        if (_pressing.Count != 0 && Time.unscaledTime - _lastPressTime > 10f) _pressing.Clear();
    }

    private void OnApplicationFocus(bool focus) { if (!focus) Close(); }
    private void OnApplicationPause(bool paused) { if (paused) Close(); }
    private void OnDisable() => Close();
    private void OnDestroy()
    {
        Close();
        if (_canvas != null)
        {
            if (Application.isPlaying) Destroy(_canvas.gameObject);
            else DestroyImmediate(_canvas.gameObject);
        }
        _changed = null;
    }
}
