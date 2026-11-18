using System;
using System.IO;
using UnityEngine;

public partial class FloorTilingController
{
    private enum PointCapture { SourceCentre, PlacementCorner }
    private PointCapture? _pointCapture;
    private Vector3 _pointTipOffset;
    private Transform _sourceSpace;
    private Pose _sourceInTracking;
    private int _sourceCalibrationRevision;
    private RosRobotModel _sourceModel;
    private string _sourceTool;
    private RepetitiveSourcePreview _sourcePreview;
    private float _sourceYawDegrees;
    private const string DimensionsPreference = "UnityROSML.RepetitiveTileDimensions";
    private bool _dimensionsLoaded;

    [Serializable] private sealed class SavedDimensions { public float ItemSize, ItemThickness; }
    [Serializable] private sealed class LegacyDimensionsProfile { public SavedDimensions settings; }

    public float TileSideMeters { get; private set; } = (float)PickSurfaceGeometry.GetPreset(PickItem.Tile).Width;
    public float TileThicknessMeters { get; private set; } = (float)PickSurfaceGeometry.GetPreset(PickItem.Tile).Thickness;
    public bool SourceFromCorners { get; private set; } = true;
    public bool IsCapturingPoint => _pointCapture.HasValue;
    public int CaptureStartFrame { get; private set; } = -1;
    public bool CanCaptureArmedPoint => IsCapturingPoint && CanEdit &&
        (!Application.isPlaying || Time.frameCount > CaptureStartFrame);
    public float SourceYawDegrees => _sourceYawDegrees;
    public string CaptureInstruction => _pointCapture == PointCapture.SourceCentre
        ? "Touch the top tile's centre with the controller tip and press trigger."
        : "Touch the final support-surface corner with the controller tip and press trigger. The tile bottom sits here.";

    public bool IsSourceCurrent
    {
        get
        {
            if (_requiresRecapture || _surfaceCapture?.IsGuideOpen == true) return false;
            if (SourceFromCorners) return _surfaceCapture != null && _surfaceCapture.IsSurfaceCurrent;
            ToolSelectionController tool = ToolSelectionController.Instance;
            return RobotFrameMath.IsRigidFrame(_sourceSpace) && RobotCoordinateService.Instance.IsInitialized &&
                RobotCoordinateService.Instance.CalibrationRevision == _sourceCalibrationRevision &&
                ReferenceEquals(_sourceModel, tool?.CurrentModel) && _sourceTool == tool?.SelectedTool;
        }
    }

    public bool TryGetSource(out Pose worldTop, out Vector3[] worldCorners)
    {
        worldTop = Pose.identity;
        worldCorners = Array.Empty<Vector3>();
        if (!IsSourceCurrent) return false;
        if (SourceFromCorners)
            return _surfaceCapture.TryGetWorldTopSurfacePose(out worldTop) && _surfaceCapture.TryGetWorldCorners(out worldCorners);
        return RobotFrameMath.TryTrackingToWorld(_sourceSpace, _sourceInTracking, out worldTop);
    }

    public void SetTileSideMeters(float side)
    {
        if (!CanEdit || !float.IsFinite(side) || side < .001f || side > 2f || side < TileThicknessMeters || side == TileSideMeters) return;
        TileSideMeters = side;
        DimensionsChanged();
    }

    public void SetTileThicknessMeters(float thickness)
    {
        if (!CanEdit || !float.IsFinite(thickness) || thickness < .001f || thickness > TileSideMeters || thickness == TileThicknessMeters) return;
        TileThicknessMeters = thickness;
        DimensionsChanged();
    }

    private void DimensionsChanged()
    {
        SaveTileDimensions();
        _requiresRecapture = true;
        CancelPointCapture();
        InvalidateAnchor("Tile dimensions changed. Capture the source again, then the final support surface.");
        RefreshSourcePreview();
    }

    private void LoadTileDimensions()
    {
        if (!Application.isPlaying || _dimensionsLoaded) return;
        _dimensionsLoaded = true;
        try
        {
            SavedDimensions dimensions = null;
            if (PlayerPrefs.HasKey(DimensionsPreference))
                dimensions = JsonUtility.FromJson<SavedDimensions>(PlayerPrefs.GetString(DimensionsPreference));
            else
            {
                string path = Path.Combine(Application.persistentDataPath, "fixture-tiling.json");
                if (File.Exists(path))
                    dimensions = JsonUtility.FromJson<LegacyDimensionsProfile>(File.ReadAllText(path))?.settings;
            }
            if (dimensions != null && float.IsFinite(dimensions.ItemSize) && float.IsFinite(dimensions.ItemThickness) &&
                dimensions.ItemSize >= .001f && dimensions.ItemSize <= 2f &&
                dimensions.ItemThickness >= .001f && dimensions.ItemThickness <= dimensions.ItemSize)
            {
                TileSideMeters = dimensions.ItemSize;
                TileThicknessMeters = dimensions.ItemThickness;
            }
        }
        catch (Exception error) { Debug.LogWarning("Saved tile dimensions could not be loaded: " + error.Message, this); }
        SaveTileDimensions();
    }

    private void SaveTileDimensions()
    {
        if (!Application.isPlaying) return;
        PlayerPrefs.SetString(DimensionsPreference, JsonUtility.ToJson(new SavedDimensions
        {
            ItemSize = TileSideMeters,
            ItemThickness = TileThicknessMeters
        }));
        PlayerPrefs.Save();
    }

    public void BeginSourceCenterCapture() => BeginPointCapture(PointCapture.SourceCentre);
    public void BeginPlacementCapture() => BeginPointCapture(PointCapture.PlacementCorner);

    private void BeginPointCapture(PointCapture target)
    {
        if (!CanEdit || _surfaceCapture?.IsGuideOpen == true) return;
        if (!RobotCoordinateService.Instance.IsInitialized || !ToolSelectionController.CanPlanPickPlace)
        {
            _notice = "Finish robot alignment and match the suction tool before capturing a physical point.";
            Changed?.Invoke();
            return;
        }
        if (target == PointCapture.PlacementCorner && !IsSourceCurrent)
        {
            _notice = "Capture the source tile before its final placement.";
            Changed?.Invoke();
            return;
        }
        InvalidatePlan();
        StopScan();
        RobotPointRegistration registration = FindFirstObjectByType<RobotPointRegistration>();
        _pointTipOffset = registration != null ? registration.TipOffsetInPointer : Vector3.zero;
        _pointCapture = target;
        CaptureStartFrame = Time.frameCount;
        RefreshTrackingSubscriptions();
        TargetPoseContextMenu.CloseAll();
        TargetRotationHandles.CancelAllInteractions();
        Changed?.Invoke();
    }

    public void CancelPointCapture()
    {
        if (!_pointCapture.HasValue) return;
        _pointCapture = null;
        Changed?.Invoke();
    }

    public bool CapturePointerPose(Pose worldPointer, Transform trackingSpace)
    {
        if (!CanCaptureArmedPoint || !RobotFrameMath.IsRigidFrame(trackingSpace) ||
            !RobotFrameMath.IsValidPose(worldPointer) ||
            !RobotCoordinateService.Instance.TryGetRobotBaseWorldPose(out Pose robotBase)) return false;
        if (!ToolSelectionController.CanPlanPickPlace || (_pointCapture == PointCapture.PlacementCorner && !IsSourceCurrent))
        {
            _pointCapture = null;
            _notice = "The source capture or tool is no longer current. Resolve the tool match and source capture, then choose placement again.";
            Changed?.Invoke();
            return false;
        }
        Vector3 contact = worldPointer.position + worldPointer.rotation * _pointTipOffset;
        if (!RobotFrameMath.IsValidPose(new Pose(contact, Quaternion.identity))) return false;

        PointCapture target = _pointCapture.Value;
        _pointCapture = null;
        if (target == PointCapture.SourceCentre)
        {
            Pose top = new Pose(contact, robotBase.rotation * Quaternion.AngleAxis(_sourceYawDegrees, Vector3.up));
            _sourceSpace = trackingSpace;
            _sourceInTracking = RobotFrameMath.RelativeTo(new Pose(trackingSpace.position, trackingSpace.rotation), top);
            _sourceCalibrationRevision = RobotCoordinateService.Instance.CalibrationRevision;
            _sourceModel = ToolSelectionController.Instance?.CurrentModel;
            _sourceTool = ToolSelectionController.Instance?.SelectedTool;
            SourceFromCorners = false;
            _requiresRecapture = false;
            InvalidateAnchor("Source centre captured. Align the green stack's yaw, then capture the final support-surface corner.");
        }
        else
        {
            Pose placement = new Pose(contact, PlacementRotation(robotBase, robotBase.rotation * Vector3.up) * Quaternion.AngleAxis(YawDegrees, Vector3.up));
            SetAnchor(placement, trackingSpace);
            _notice = "Final support surface captured. Inspect the pattern and plan the whole job.";
        }
        RefreshSourcePreview();
        Changed?.Invoke();
        return true;
    }

    public void RotateSource(float degrees)
    {
        if (!CanEdit || SourceFromCorners || !IsSourceCurrent || !float.IsFinite(degrees)) return;
        _sourceYawDegrees = Mathf.Repeat(_sourceYawDegrees + degrees, 360f);
        _sourceInTracking.rotation *= Quaternion.AngleAxis(degrees, Vector3.up);
        InvalidatePlan();
        RefreshSourcePreview();
        Changed?.Invoke();
    }

    private static Quaternion PlacementRotation(Pose robotBase, Vector3 normal)
    {
        Vector3 rows = Vector3.ProjectOnPlane(robotBase.rotation * Vector3.left, normal).normalized;
        return Quaternion.LookRotation(rows, normal);
    }

    private void RefreshSourcePreview()
    {
        Pose top = Pose.identity;
        bool visible = _taskActive && _review?.HasReview != true && TryGetSource(out top, out _);
        if (!visible)
        {
            _sourcePreview?.Hide();
            return;
        }
        if (_sourcePreview == null)
        {
            var owner = new GameObject("Source stack preview");
            owner.transform.SetParent(transform, false);
            _sourcePreview = owner.AddComponent<RepetitiveSourcePreview>();
        }
        _sourcePreview.Show(TileCount, TileSideMeters, TileThicknessMeters, top);
    }
}
