using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using TMPro;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

/// <summary>
/// Local TCP aim/capture visualization. Never publishes a robot command.
/// A canonical Unity pose uses local +Y for ROS tool +Z and local +Z for ROS tool +X.
/// </summary>
public class TcpTargetPreview : MonoBehaviour
{
    public event System.Action TargetsChanged;
    [SerializeField] private Material _markerMaterial;
    [SerializeField, Min(0.01f)] private float _sphereDiameter = 0.035f;
    [SerializeField, Tooltip("Optional target-point offset in the controller's pointer frame, in metres. This is not a flange-to-TCP compensation.")]
    private Vector3 _pointerToTargetOffsetMeters;

    private sealed class Marker
    {
        public Transform Root;
        public TextMeshPro Label;
        public string Caption;
        public TargetRotationHandles RotationHandles;
        public ToolPoseVisual ToolVisual;
        public TargetPoseContextMenu ContextMenu;
        public System.Action VisualChanged;
    }

    private Marker _live;
    private Marker _pick;
    private Marker _place;
    private Material _liveMaterial;
    private Material _pickMaterial;
    private Material _placeMaterial;
    private bool _targetsVisible = true;

    /// <summary>Keep captured poses while exposing only the active task's markers and handles.</summary>
    public void SetTargetsVisible(bool visible)
    {
        _targetsVisible = visible;
        // A live aim must be refreshed from a tracked controller, including after a mode switch.
        SetTrackingLost();
        if (_pick != null) _pick.Root.gameObject.SetActive(visible);
        if (_place != null) _place.Root.gameObject.SetActive(visible);
    }

    public void SetPointerPose(Pose pointerWorld, Transform trackingSpace)
    {
        if (!_targetsVisible || !RobotFrameMath.IsValidPose(pointerWorld) || !RobotFrameMath.IsRigidFrame(trackingSpace)) return;
        if (_live == null) _live = CreateMarker("TCP tool aim", new Color(0.1f, 0.9f, 1f), out _liveMaterial);
        UpdateMarker(_live, TargetPose(pointerWorld), trackingSpace);
        _live.Root.gameObject.SetActive(true);
    }

    public void CapturePointerPose(Pose pointerWorld, Transform trackingSpace, bool isPick)
    {
        if (!_targetsVisible || !RobotFrameMath.IsValidPose(pointerWorld) || !RobotFrameMath.IsRigidFrame(trackingSpace)) return;
        Marker marker;
        if (isPick)
        {
            if (_pick == null) _pick = CreateMarker("TCP A — exact tool pose", Color.green, out _pickMaterial);
            marker = _pick;
        }
        else
        {
            if (_place == null) _place = CreateMarker("TCP B — exact tool pose", new Color(1f, 0.6f, 0.05f), out _placeMaterial);
            marker = _place;
        }
        if (marker.RotationHandles == null)
        {
            marker.RotationHandles = TargetRotationHandles.Attach(marker.Root,
                isPick ? _pickMaterial : _placeMaterial, _sphereDiameter);
            marker.RotationHandles.RotationStarted += OnTargetChanged;
            marker.RotationHandles.RotationChanged += OnTargetChanged;
            marker.VisualChanged = () => marker.RotationHandles.ConfigureToolInteraction(marker.ToolVisual.TcpLocalBounds);
            marker.ToolVisual.VisualChanged += marker.VisualChanged;
            marker.VisualChanged();
            marker.ContextMenu = TargetPoseContextMenu.Attach(marker.Root, isPick ? "TCP A" : "TCP B", false, OnTargetChanged);
        }
        UpdateMarker(marker, TargetPose(pointerWorld), trackingSpace);
        marker.ContextMenu?.RebaseAtCurrentPose();
        OnTargetChanged();
        if (TryGetCapturedRosPose(isPick, out PoseStampedMsg target))
            Debug.Log($"{marker.Caption}: {target.header.frame_id} TCP ({target.pose.position.x:F4}, {target.pose.position.y:F4}, {target.pose.position.z:F4}) m. No command sent.");
    }

    private void OnTargetChanged() => TargetsChanged?.Invoke();

    public void SetTrackingLost()
    {
        if (_live != null) _live.Root.gameObject.SetActive(false);
    }

    /// <summary>Build a TCP target for inspection/export. The ROS direct-TCP topic executes motion; this method only returns data.</summary>
    public bool TryGetCapturedRosPose(bool isPick, out PoseStampedMsg target)
    {
        target = null;
        Marker marker = isPick ? _pick : _place;
        if (!_targetsVisible || marker == null || !RobotFrameMath.IsRigidFrame(marker.Root) ||
            !RobotCoordinateService.Instance.TryConvertWorldPoseToLocalPose(
                new Pose(marker.Root.position, marker.Root.rotation), out Pose localPose)) return false;

        HeaderMsg header = new HeaderMsg { frame_id = RobotCoordinateService.Instance.RosBaseFrameId };
        // A captured target is static in the calibrated robot base; zero requests latest TF.
        target = new PoseStampedMsg(header, new PoseMsg(localPose.position.To<FLU>(), localPose.rotation.To<FLU>()));
        return true;
    }

    private Pose TargetPose(Pose pointerWorld)
    {
        Pose target = RobotFrameMath.PointerToTcpPose(pointerWorld);
        target.position += pointerWorld.rotation * _pointerToTargetOffsetMeters;
        return target;
    }

    private static void UpdateMarker(Marker marker, Pose worldPose, Transform trackingSpace)
    {
        if (marker.Root.parent != trackingSpace) marker.Root.SetParent(trackingSpace, true);
        marker.Root.SetPositionAndRotation(worldPose.position, worldPose.rotation);
    }

    private void LateUpdate()
    {
        UpdateLabel(_live);
        UpdateLabel(_pick);
        UpdateLabel(_place);
    }

    private static void UpdateLabel(Marker marker)
    {
        if (marker == null || marker.Label == null || !marker.Root.gameObject.activeInHierarchy) return;
        string coordinates = "Scan QR and place robot to inspect ROS coordinates";
        if (RobotCoordinateService.Instance.TryConvertWorldPoseToLocalPose(
                new Pose(marker.Root.position, marker.Root.rotation), out Pose local))
        {
            PointMsg ros = local.position.To<FLU>();
            coordinates = $"{RobotCoordinateService.Instance.RosBaseFrameId}: {ros.x:F3}, {ros.y:F3}, {ros.z:F3} m";
        }
        string editing = marker.RotationHandles != null ? "\nTrigger-drag tool / rings to rotate; bumper: menu" : "";
        string geometry = marker.ToolVisual != null && !marker.ToolVisual.HasVisual ? "\n" + marker.ToolVisual.StatusText : "";
        marker.Label.text = $"{marker.Caption}\n{coordinates}{geometry}{editing}";
        marker.Label.transform.position = marker.Root.position + Vector3.up * (marker.RotationHandles != null ? 0.17f : 0.09f);
        if (Camera.main != null) marker.Label.transform.rotation = Camera.main.transform.rotation;
    }

    private Marker CreateMarker(string caption, Color color, out Material material)
    {
        Shader shader = _markerMaterial != null ? _markerMaterial.shader : Shader.Find("Universal Render Pipeline/Unlit");
        material = _markerMaterial != null ? new Material(_markerMaterial) : new Material(shader);
        if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
        if (material.HasProperty("_Color")) material.SetColor("_Color", color);
        GameObject root = new GameObject(caption);
        root.layer = 2;
        ToolPoseVisual visual = ToolPoseVisual.Attach(root.transform, color,
            caption == "TCP tool aim" ? 0.35f : 0.72f);
        GameObject labelObject = new GameObject("Target label", typeof(TextMeshPro));
        labelObject.layer = 2;
        labelObject.transform.SetParent(root.transform, false);
        labelObject.transform.localScale = Vector3.one * 0.05f;
        TextMeshPro label = labelObject.GetComponent<TextMeshPro>();
        label.fontSize = 2f;
        label.color = color;
        label.alignment = TextAlignmentOptions.Center;
        label.rectTransform.sizeDelta = new Vector2(12f, 3f);
        return new Marker { Root = root.transform, Label = label, Caption = caption, ToolVisual = visual };
    }
    private void OnDestroy()
    {
        foreach (Marker marker in new[] { _pick, _place })
            if (marker?.RotationHandles != null)
            {
                marker.RotationHandles.RotationStarted -= OnTargetChanged;
                marker.RotationHandles.RotationChanged -= OnTargetChanged;
                if (marker.ToolVisual != null && marker.VisualChanged != null)
                    marker.ToolVisual.VisualChanged -= marker.VisualChanged;
            }
        TargetsChanged = null;
        if (_live != null && _live.Root != null) Dispose(_live.Root.gameObject);
        if (_pick != null && _pick.Root != null) Dispose(_pick.Root.gameObject);
        if (_place != null && _place.Root != null) Dispose(_place.Root.gameObject);
        if (_liveMaterial != null) Dispose(_liveMaterial);
        if (_pickMaterial != null) Dispose(_pickMaterial);
        if (_placeMaterial != null) Dispose(_placeMaterial);
    }

    private static void Dispose(Object value)
    {
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }
}
