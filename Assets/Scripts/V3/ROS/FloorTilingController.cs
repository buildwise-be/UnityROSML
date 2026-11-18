using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

/// <summary>Captures the first tile and places the batch preview in metre-scale tracking space.</summary>
[DisallowMultipleComponent]
public partial class FloorTilingController : MonoBehaviour
{
    private const float MenuScale = 0.5f;
    private PickSurfaceCapture _surfaceCapture;
    private FloorSurfaceScanner _scanner;
    private FloorTilePatternVisual _menuPreview, _aimPreview, _anchorPreview;
    private readonly List<XRInputSubsystem> _trackingSubsystems = new List<XRInputSubsystem>();
    private Transform _anchorSpace;
    private Pose _anchorInTracking;
    private int _anchorCalibrationRevision;
    private bool _taskActive;
    private bool _requiresRecapture;
    private string _notice;

    public int TileCount { get; private set; } = FloorTilePattern.MinimumTileCount;
    public int OffsetIndex { get; private set; }
    public int MortarMillimeters { get; private set; } = FloorTilePattern.MinimumMortarMillimeters;
    public float YawDegrees { get; private set; }
    public int Rows => Pattern.Rows;
    public int Columns => Pattern.Columns;
    public bool IsScanning => _scanner != null && _scanner.IsScanning;
    public bool IsScanRequested => _scanner != null && _scanner.IsScanRequested;
    public PickSurfaceCapture SurfaceCapture => _surfaceCapture;
    public bool CanEdit => _taskActive && !IsPlanning && !RobotCoordinateService.IsCalibrationEditActive &&
        !RobotExecutionState.IsBusy && MTCPlansManager.ActiveExecution == null;
    public bool CanScan => CanEdit && !_requiresRecapture && _surfaceCapture != null && !_surfaceCapture.IsGuideOpen &&
        _surfaceCapture.IsSurfaceCurrent;
    public bool HasAnchor => _anchorSpace != null && TryGetAnchorWorldPose(out _);
    public event Action Changed;

    public string StatusText
    {
        get
        {
            if (!string.IsNullOrEmpty(_planStatus)) return _planStatus;
            if (_surfaceCapture == null) return string.Empty;
            if (_surfaceCapture.IsCapturing || !_surfaceCapture.IsSurfaceCurrent)
                return _surfaceCapture.StatusText;
            if (IsScanRequested && !IsScanning) return _scanner.StatusText;
            if (HasAnchor)
                return IsScanning
                    ? "Floor pattern anchored. Aim and press trigger to move it, or plan the batch."
                    : "Floor pattern anchored. Plan the batch to review its motion and scene.";
            if (!string.IsNullOrEmpty(_notice)) return _notice;
            return _scanner != null && !string.IsNullOrEmpty(_scanner.StatusText)
                ? _scanner.StatusText : "Set the tile count and pattern, then scan the floor.";
        }
    }

    private FloorTilePattern Pattern
    {
        get
        {
            PickItemPreset preset = PickSurfaceGeometry.GetPreset(PickItem.Tile);
            return new FloorTilePattern(TileCount, (float)preset.Width, (float)preset.Thickness,
                OffsetIndex, MortarMillimeters);
        }
    }

    private void EnsureCapture()
    {
        if (_surfaceCapture != null) return;
        // The simple planner listens to its own capture. Use the same interaction in a separate instance.
        GameObject capture = new GameObject("First tile surface capture");
        capture.transform.SetParent(transform, false);
        _surfaceCapture = capture.AddComponent<PickSurfaceCapture>();
        _surfaceCapture.Changed += OnCaptureChanged;
        _surfaceCapture.SurfaceCaptured += OnSurfaceCaptured;
    }

    public void SetTaskActive(bool active)
    {
        if (_taskActive == active) return;
        _taskActive = active;
        EnsureCapture();
        _surfaceCapture.SetItemPresentationEnabled(active && !_requiresRecapture);
        if (!active)
        {
            InvalidatePlan();
            _surfaceCapture.Cancel();
            StopScan();
            _menuPreview?.Hide();
            _anchorPreview?.Hide();
        }
        else RefreshAnchorPreview();
        Changed?.Invoke();
    }

    public void BeginCapture()
    {
        if (!CanEdit || !_surfaceCapture.OpenGuide(PickItem.Tile, PickSurfaceGeometry.GetPreset(PickItem.Tile))) return;
        InvalidatePlan();
        StopScan();
        Changed?.Invoke();
    }

    public void SetTileCount(int count)
    {
        if (!CanEdit || TileCount == count ||
            count < FloorTilePattern.MinimumTileCount || count > FloorTilePattern.MaximumTileCount) return;
        TileCount = count;
        OnPatternChanged();
    }

    public void SetOffsetIndex(int index)
    {
        if (!CanEdit || OffsetIndex == index || index < 0 || index > 2) return;
        OffsetIndex = index;
        OnPatternChanged();
    }

    public void SetMortarMillimeters(int millimeters)
    {
        if (!CanEdit || MortarMillimeters == millimeters || millimeters < FloorTilePattern.MinimumMortarMillimeters ||
            millimeters > FloorTilePattern.MaximumMortarMillimeters) return;
        MortarMillimeters = millimeters;
        OnPatternChanged();
    }

    public void RotatePattern(float degrees)
    {
        if (!CanEdit || !float.IsFinite(degrees)) return;
        float nextYaw = Mathf.Repeat(YawDegrees + degrees, 360f);
        if (Mathf.Approximately(nextYaw, YawDegrees)) return;
        if (HasAnchor)
            _anchorInTracking.rotation *= Quaternion.AngleAxis(nextYaw - YawDegrees, Vector3.up);
        YawDegrees = nextYaw;
        OnPatternChanged();
    }

    public void StartScan()
    {
        if (!CanScan) return;
        if (_scanner == null) _scanner = gameObject.AddComponent<FloorSurfaceScanner>();
        InvalidatePlan();
        _notice = null;
        RefreshTrackingSubscriptions();
        _scanner.StartScan();
        Changed?.Invoke();
    }

    public void StopScan()
    {
        if (_scanner == null) return;
        _scanner.StopScan();
        ClearAim();
        Changed?.Invoke();
    }

    public void ShowMenuPreview(Transform menu, Vector3 worldCenter, bool visible)
    {
        if (!_taskActive || !visible || menu == null)
        {
            _menuPreview?.Hide();
            return;
        }
        FloorTilePattern pattern = Pattern;
        Quaternion rotation = Quaternion.LookRotation(menu.up, -menu.forward);
        Vector2 center = pattern.BoundsCenter * MenuScale;
        Vector3 corner = worldCenter - rotation * new Vector3(center.x, 0f, center.y);
        if (_menuPreview == null) _menuPreview = CreatePreview("Half-scale tile pattern");
        _menuPreview.Show(pattern, new Pose(corner, rotation), MenuScale);
    }

    public void UpdateAim(Ray ray, float maxDistance)
    {
        if (!TryGetCandidate(ray, maxDistance, out Pose pose) ||
            (TryGetAnchorWorldPose(out Pose anchor) && (anchor.position - pose.position).sqrMagnitude < 0.0001f))
        {
            ClearAim();
            return;
        }
        if (_aimPreview == null) _aimPreview = CreatePreview("Floor tile pattern aim");
        _aimPreview.Show(Pattern, pose, 1f);
    }

    public void ClearAim() => _aimPreview?.Hide();

    public bool TrySetAnchor(Ray ray, float maxDistance, Transform trackingSpace)
    {
        if (!RobotFrameMath.IsRigidFrame(trackingSpace) || !TryGetCandidate(ray, maxDistance, out Pose world)) return false;
        InvalidatePlan();
        _anchorSpace = trackingSpace;
        _anchorInTracking = RobotFrameMath.RelativeTo(new Pose(trackingSpace.position, trackingSpace.rotation), world);
        _anchorCalibrationRevision = RobotCoordinateService.Instance.CalibrationRevision;
        ClearAim();
        RefreshAnchorPreview();
        Changed?.Invoke();
        return true;
    }

    public void CheckTrackingSpace(Transform trackingSpace)
    {
        if (_anchorSpace != null && _anchorSpace != trackingSpace)
            InvalidateAnchor("XR tracking space changed. Scan the floor again.");
    }

    private bool TryGetAnchorWorldPose(out Pose pose)
    {
        pose = default;
        return _anchorSpace != null && _surfaceCapture.IsSurfaceCurrent &&
            RobotCoordinateService.Instance.CalibrationRevision == _anchorCalibrationRevision &&
            RobotFrameMath.TryTrackingToWorld(_anchorSpace, _anchorInTracking, out pose);
    }

    private bool TryGetCandidate(Ray ray, float maxDistance, out Pose pose)
    {
        pose = default;
        if (!CanScan || !IsScanning || !_scanner.TryGetFloorHit(ray, maxDistance, out Vector3 point, out Vector3 normal) ||
            !RobotCoordinateService.Instance.TryGetRobotBaseWorldPose(out Pose robotBase)) return false;
        if (!FloorTilingRequest.IsLevel(normal, robotBase.rotation * Vector3.up))
        {
            _notice = "Aim at a floor plane level with the robot base.";
            return false;
        }
        _notice = null;
        // At zero yaw, columns follow ROS base +X and rows follow ROS base +Y.
        Vector3 rows = Vector3.ProjectOnPlane(robotBase.rotation * Vector3.left, normal);
        if (rows.sqrMagnitude < 0.0001f) return false;
        Quaternion rotation = Quaternion.LookRotation(rows, normal) * Quaternion.AngleAxis(YawDegrees, Vector3.up);
        pose = new Pose(point, rotation);
        return RobotFrameMath.IsValidPose(pose);
    }

    private void LateUpdate()
    {
        if (_taskActive && _anchorSpace != null) RefreshAnchorPreview();
    }

    private void OnPatternChanged()
    {
        InvalidatePlan();
        RefreshAnchorPreview();
        Changed?.Invoke();
    }

    private void OnCaptureChanged()
    {
        if (!_surfaceCapture.IsSurfaceCurrent && _anchorSpace != null)
        {
            InvalidateAnchor("Capture the top tile again, then rescan the floor.");
            return;
        }
        Changed?.Invoke();
    }

    private void OnSurfaceCaptured()
    {
        InvalidatePlan();
        _requiresRecapture = false;
        _surfaceCapture.SetItemPresentationEnabled(_taskActive);
        _notice = null;
        if (_anchorSpace != null)
            InvalidateAnchor("Top tile recaptured. Scan the floor to place this batch.");
    }

    private void RefreshAnchorPreview()
    {
        if (!_taskActive || _surfaceCapture?.IsGuideOpen == true || _review?.HasReview == true || !TryGetAnchorWorldPose(out Pose world))
        {
            _anchorPreview?.Hide();
            return;
        }
        if (_anchorPreview == null) _anchorPreview = CreatePreview("Anchored floor tile pattern");
        _anchorPreview.Show(Pattern, world, 1f);
    }

    private FloorTilePatternVisual CreatePreview(string name)
    {
        GameObject preview = new GameObject(name) { layer = 2 };
        preview.transform.SetParent(transform, false);
        return preview.AddComponent<FloorTilePatternVisual>();
    }

    private void RefreshTrackingSubscriptions()
    {
        ClearTrackingSubscriptions();
        SubsystemManager.GetSubsystems(_trackingSubsystems);
        foreach (XRInputSubsystem subsystem in _trackingSubsystems)
            subsystem.trackingOriginUpdated += OnTrackingOriginUpdated;
    }

    private void ClearTrackingSubscriptions()
    {
        foreach (XRInputSubsystem subsystem in _trackingSubsystems)
            subsystem.trackingOriginUpdated -= OnTrackingOriginUpdated;
        _trackingSubsystems.Clear();
    }

    private void OnTrackingOriginUpdated(XRInputSubsystem subsystem) =>
        InvalidateAnchor("XR tracking origin reset. Capture the tile and scan the floor again.");

    public void SuspendInteraction()
    {
        if (_surfaceCapture != null && _surfaceCapture.IsCapturing) _surfaceCapture.Cancel();
        StopScan();
    }

    private void InvalidateAnchor(string reason)
    {
        _anchorSpace = null;
        InvalidatePlan();
        _notice = reason;
        _anchorPreview?.Hide();
        StopScan();
        Changed?.Invoke();
    }

    private void OnDisable()
    {
        InvalidatePlan();
        SuspendInteraction();
        ClearTrackingSubscriptions();
        _menuPreview?.Hide();
        _anchorPreview?.Hide();
    }

    private void OnApplicationPause(bool isPaused)
    {
        if (isPaused) InvalidateAnchor("App paused. Capture the tile and scan the floor again.");
    }

    private void OnDestroy()
    {
        UnbindTransport();
        if (_surfaceCapture == null) return;
        _surfaceCapture.Changed -= OnCaptureChanged;
        _surfaceCapture.SurfaceCaptured -= OnSurfaceCaptured;
    }
}
