using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.XR;

/// <summary>Three ordered physical contacts defining an item's top surface; never publishes ROS commands.</summary>
[DisallowMultipleComponent]
public sealed class PickSurfaceCapture : MonoBehaviour
{
    [SerializeField] private Vector3 _tipOffsetInPointer;
    [SerializeField, Min(.0001f)] private float _maximumRmsErrorMeters = PickSurfaceGeometry.DefaultMaximumRms;
    [SerializeField, Min(.0001f)] private float _maximumPointErrorMeters = PickSurfaceGeometry.DefaultMaximumPointError;
    private bool _explicitTipOffset;
    private readonly List<Vector3> _points = new List<Vector3>();
    private readonly List<GameObject> _pointVisuals = new List<GameObject>();
    private readonly List<Material> _pointMaterials = new List<Material>();
    private readonly List<XRInputSubsystem> _subsystems = new List<XRInputSubsystem>();
    private Transform _captureSpace, _surfaceSpace;
    private int _captureRevision, _surfaceRevision;
    private RosRobotModel _captureModel, _surfaceModel;
    private string _captureTool, _surfaceTool;
    private Pose _surfaceInTracking;
    private Vector3[] _surfaceCorners = Array.Empty<Vector3>();
    private bool _hasSurface, _surfaceTrackingInvalidated, _lastSurfaceCurrent;
    private GameObject _tipVisual;
    private Material _tipMaterial;
    private PickSurfaceGuide _guide;
    private GameObject _itemVisual;
    private MeshRenderer _itemRenderer;
    private Material _itemMaterial;
    private bool _itemPresentationEnabled = true;

    public bool IsCapturing { get; private set; }
    public int CapturedCount => _points.Count;
    public PickItem Item { get; private set; }
    public PickItem SurfaceItem { get; private set; }
    public bool HasSurface => _hasSurface;
    public float RmsErrorMeters { get; private set; }
    public float MaxErrorMeters { get; private set; }
    public string StatusText { get; private set; } = "Choose an item, then capture its three consecutive top corners.";
    public string CurrentPointName => CornerName(Mathf.Min(CapturedCount, 2));
    public Pose WorldTopSurfacePose => TryGetWorldTopSurfacePose(out Pose pose) ? pose : Pose.identity;
    public bool IsSurfaceCurrent => _hasSurface && !_surfaceTrackingInvalidated && RobotFrameMath.IsRigidFrame(_surfaceSpace) &&
        RobotCoordinateService.Instance.IsInitialized && RobotCoordinateService.Instance.CalibrationRevision == _surfaceRevision &&
        SameTool(_surfaceModel, _surfaceTool);
    public event Action Changed;
    public event Action SurfaceCaptured;

    public void SetTipOffset(Vector3 offsetInPointer)
    {
        if (!RobotFrameMath.IsValidPose(new Pose(offsetInPointer, Quaternion.identity)))
            throw new ArgumentException("The physical controller-tip offset must be finite.");
        _explicitTipOffset = true;
        if (_tipOffsetInPointer == offsetInPointer) return;
        if (IsCapturing) Cancel();
        _tipOffsetInPointer = offsetInPointer;
        if (_hasSurface)
        {
            _surfaceTrackingInvalidated = true;
            SetStatus("The controller contact tip changed. Capture the surface again.");
        }
    }

    public bool BeginCapture(PickItem item)
    {
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null || RobotCoordinateService.IsCalibrationEditActive ||
            !RobotCoordinateService.Instance.IsInitialized || !ToolSelectionController.CanPlanPickPlace)
        { SetStatus("Finish robot alignment, match the suction-tool model and resolve any execution before capturing item corners."); return false; }
        try { PickSurfaceGeometry.GetPreset(item); }
        catch (ArgumentOutOfRangeException) { SetStatus("Choose the tile or brick preset."); return false; }
        if (!_explicitTipOffset)
        {
            RobotPointRegistration registration = FindFirstObjectByType<RobotPointRegistration>();
            if (registration != null) _tipOffsetInPointer = registration.TipOffsetInPointer;
        }
        ClearPointVisuals(); _points.Clear(); _captureSpace = null;
        Item = item; _captureRevision = RobotCoordinateService.Instance.CalibrationRevision;
        ToolSelectionController tool = ToolSelectionController.Instance;
        _captureModel = tool?.CurrentModel; _captureTool = tool?.SelectedTool;
        IsCapturing = true;
        TargetPoseContextMenu.CloseAll(); TargetRotationHandles.CancelAllInteractions();
        SubscribeTracking();
        RefreshGuide();
        _guide?.Recenter();
        SetStatus("Touch 1/3: " + CornerName(0) + "; press trigger. Use the verified controller contact tip.");
        return true;
    }

    public bool CapturePointerPose(Pose pointerWorld, Transform trackingSpace)
    {
        if (!IsCapturing) return false;
        if (!CaptureContextValid(out string reason)) { CancelWithReason(reason); return false; }
        if (CapturedCount >= 3) { SetStatus("Undo the last corner or restart to retry the rejected surface fit."); return false; }
        if (!TryTip(pointerWorld, trackingSpace, out Vector3 tipWorld))
        { SetStatus("A tracked controller and a valid metre-scale tracking frame are required."); return false; }
        if (_captureSpace != null && _captureSpace != trackingSpace)
        { CancelWithReason("The XR tracking frame changed. Capture all three corners again."); return false; }
        if (_captureSpace == null) _captureSpace = trackingSpace;
        Vector3 local = trackingSpace.InverseTransformPoint(tipWorld);
        foreach (Vector3 previous in _points)
            if (Vector3.Distance(previous, local) < .010f)
            { SetStatus("This corner repeats an earlier contact. Touch " + CurrentPointName + "."); return false; }
        _points.Add(local); AddPointVisual(local, _points.Count - 1);
        if (CapturedCount < 3)
        {
            RefreshGuide();
            SetStatus("Touch " + (CapturedCount + 1) + "/3: " + CurrentPointName + "; press trigger.");
            return true;
        }
        if (!PickSurfaceGeometry.TryFit(Item, _points, out Pose surface, out _, out float rms, out float worst,
            out reason, _maximumRmsErrorMeters, _maximumPointErrorMeters))
        { RefreshGuide(); SetStatus(reason + " Undo the last corner or restart; the previous pick remains unchanged."); return false; }
        _surfaceInTracking = surface; _surfaceCorners = _points.ToArray(); _surfaceSpace = _captureSpace;
        _surfaceRevision = _captureRevision; _surfaceModel = _captureModel; _surfaceTool = _captureTool;
        SurfaceItem = Item; RmsErrorMeters = rms; MaxErrorMeters = worst;
        _hasSurface = true; _surfaceTrackingInvalidated = false; _lastSurfaceCurrent = true;
        IsCapturing = false; SetTrackingLost(); _guide?.Hide();
        ClearPointVisuals();
        UpdateAcceptedItemVisual();
        StatusText = "Surface captured. RMS " + (rms * 1000f).ToString("F1") + " mm; worst " + (worst * 1000f).ToString("F1") + " mm.";
        SurfaceCaptured?.Invoke(); Changed?.Invoke();
        return true;
    }

    public bool TryGetWorldTopSurfacePose(out Pose pose)
    {
        pose = Pose.identity;
        if (!IsSurfaceCurrent) return false;
        pose = RobotFrameMath.Compose(new Pose(_surfaceSpace.position, _surfaceSpace.rotation), _surfaceInTracking);
        return true;
    }

    /// <summary>The measured body is a pick snapshot; it follows the pick workflow's visibility.</summary>
    public void SetItemPresentationEnabled(bool enabled)
    {
        _itemPresentationEnabled = enabled;
        RefreshItemVisibility();
    }

    private void UpdateAcceptedItemVisual()
    {
        if (_itemVisual == null)
        {
            _itemVisual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            _itemVisual.name = "Captured item body"; _itemVisual.layer = 2;
            Collider collider = _itemVisual.GetComponent<Collider>(); collider.enabled = false; Dispose(collider);
            _itemRenderer = _itemVisual.GetComponent<MeshRenderer>();
            _itemMaterial = TargetMarkerMaterials.Create(new Color(.35f, .9f, 1f));
            _itemRenderer.sharedMaterial = _itemMaterial;
            _itemRenderer.shadowCastingMode = ShadowCastingMode.Off; _itemRenderer.receiveShadows = false;
            RobotContourDisplay.RegisterItem(_itemRenderer);
        }
        PickItemPreset preset = PickSurfaceGeometry.GetPreset(SurfaceItem);
        _itemVisual.transform.SetParent(_surfaceSpace, false);
        _itemVisual.transform.localPosition = _surfaceInTracking.position +
            _surfaceInTracking.rotation * Vector3.down * (float)(preset.Thickness / 2d);
        _itemVisual.transform.localRotation = _surfaceInTracking.rotation;
        _itemVisual.transform.localScale = new Vector3((float)preset.Width, (float)preset.Thickness, (float)preset.Length);
        RefreshItemVisibility();
    }

    private void RefreshItemVisibility()
    {
        if (_itemVisual != null) _itemVisual.SetActive(isActiveAndEnabled && _itemPresentationEnabled && IsSurfaceCurrent);
    }

    public bool TryGetWorldPickTcpPose(out Pose pose)
    {
        pose = Pose.identity;
        if (!TryGetWorldTopSurfacePose(out Pose surface)) return false;
        pose = PickSurfaceGeometry.GetPickTcpPose(surface);
        return true;
    }

    /// <summary>Accepted raw measured contacts, in the original consecutive P0/P1/P2 order.</summary>
    public Vector3[] GetWorldCorners()
    {
        if (!IsSurfaceCurrent) return Array.Empty<Vector3>();
        return Array.ConvertAll(_surfaceCorners, p => _surfaceSpace.TransformPoint(p));
    }

    public void Undo()
    {
        if (!IsCapturing || _points.Count == 0) return;
        _points.RemoveAt(_points.Count - 1); RemovePointVisual(); RefreshGuide();
        SetStatus("Touch " + (CapturedCount + 1) + "/3: " + CurrentPointName + "; press trigger.");
    }
    public void Restart() { if (IsCapturing) BeginCapture(Item); }
    public void Cancel() => CancelWithReason("Corner capture canceled. The previous pick and place remain unchanged.");
    private void CancelWithReason(string reason)
    {
        if (!IsCapturing) return;
        IsCapturing = false; _points.Clear(); _captureSpace = null; ClearPointVisuals();
        SetTrackingLost(); _guide?.Hide(); SetStatus(reason);
    }

    public void SetPointerPose(Pose pointerWorld, Transform trackingSpace)
    {
        if (!IsCapturing || !TryTip(pointerWorld, trackingSpace, out Vector3 tipWorld)) { SetTrackingLost(); return; }
        if (_tipVisual == null)
        {
            _tipMaterial = TargetMarkerMaterials.Create(new Color(.95f, .85f, .3f));
            _tipVisual = MakeSphere("Object corner contact tip", _tipMaterial, .006f);
        }
        _tipVisual.transform.SetParent(trackingSpace, true);
        _tipVisual.transform.position = tipWorld;
        _tipVisual.SetActive(true);
    }
    public void SetTrackingLost() { if (_tipVisual != null) _tipVisual.SetActive(false); }

    private bool TryTip(Pose pointerWorld, Transform trackingSpace, out Vector3 tipWorld)
    {
        tipWorld = Vector3.zero;
        if (!RobotFrameMath.IsValidPose(pointerWorld) || !RobotFrameMath.IsRigidFrame(trackingSpace) ||
            !RobotFrameMath.IsValidPose(new Pose(_tipOffsetInPointer, Quaternion.identity))) return false;
        tipWorld = pointerWorld.position + pointerWorld.rotation * _tipOffsetInPointer;
        return true;
    }
    private bool CaptureContextValid(out string reason)
    {
        reason = null;
        if (RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null || RobotCoordinateService.IsCalibrationEditActive ||
            !RobotCoordinateService.Instance.IsInitialized || RobotCoordinateService.Instance.CalibrationRevision != _captureRevision ||
            !ToolSelectionController.CanPlanPickPlace || !SameTool(_captureModel, _captureTool))
            reason = "Robot alignment, tool model or execution state changed. Capture all three corners again.";
        return reason == null;
    }
    private static bool SameTool(RosRobotModel model, string tool)
    {
        ToolSelectionController current = ToolSelectionController.Instance;
        return current == null ? model == null && tool == null : ReferenceEquals(current.CurrentModel, model) && current.SelectedTool == tool;
    }
    private static string CornerName(int index) => index == 0 ? "first top corner (P0)" : index == 1
        ? "next corner along the length edge (P1)" : "next corner along the width edge (P2)";
    private void RefreshGuide()
    {
        if (_guide == null)
        {
            GameObject guide = new GameObject("Object surface capture miniature");
            _guide = guide.AddComponent<PickSurfaceGuide>();
        }
        _guide.Refresh(Item, Mathf.Min(CapturedCount, 2));
    }
    private void AddPointVisual(Vector3 local, int index)
    {
        Color[] colors = { new Color(1f, .45f, .08f), new Color(.2f, .85f, .7f), new Color(.7f, .4f, 1f) };
        Material material = TargetMarkerMaterials.Create(colors[index]);
        GameObject point = MakeSphere("Object top corner P" + index, material, .010f);
        point.transform.SetParent(_captureSpace, false); point.transform.localPosition = local;
        _pointVisuals.Add(point); _pointMaterials.Add(material);
    }
    private static GameObject MakeSphere(string name, Material material, float diameter)
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere); sphere.name = name; sphere.layer = 2;
        Collider collider = sphere.GetComponent<Collider>(); collider.enabled = false; Dispose(collider);
        sphere.GetComponent<Renderer>().sharedMaterial = material;
        sphere.transform.localScale = Vector3.one * diameter;
        return sphere;
    }
    private void RemovePointVisual()
    {
        int index = _pointVisuals.Count - 1; if (index < 0) return;
        if (_pointVisuals[index] != null) _pointVisuals[index].SetActive(false);
        Dispose(_pointVisuals[index]); Dispose(_pointMaterials[index]);
        _pointVisuals.RemoveAt(index); _pointMaterials.RemoveAt(index);
    }
    private void ClearPointVisuals() { while (_pointVisuals.Count > 0) RemovePointVisual(); }
    private void SetStatus(string status) { StatusText = status; RefreshItemVisibility(); Changed?.Invoke(); }
    private void Update()
    {
        if (IsCapturing && !CaptureContextValid(out string reason)) CancelWithReason(reason);
        bool current = IsSurfaceCurrent;
        RefreshItemVisibility();
        if (_lastSurfaceCurrent != current) { _lastSurfaceCurrent = current; Changed?.Invoke(); }
    }
    private void OnEnable() { SubscribeTracking(); RefreshItemVisibility(); }
    private void SubscribeTracking()
    {
        foreach (XRInputSubsystem subsystem in _subsystems) subsystem.trackingOriginUpdated -= OnTrackingOriginUpdated;
        _subsystems.Clear(); SubsystemManager.GetSubsystems(_subsystems);
        foreach (XRInputSubsystem subsystem in _subsystems) subsystem.trackingOriginUpdated += OnTrackingOriginUpdated;
    }
    private void OnTrackingOriginUpdated(XRInputSubsystem subsystem) => InvalidateTracking("XR tracking origin reset. Capture the surface again.");
    private void InvalidateTracking(string reason)
    {
        _surfaceTrackingInvalidated = true; CancelWithReason(reason); SetTrackingLost();
        if (_guide != null) _guide.Hide();
        SetStatus(reason);
    }
    private void OnApplicationPause(bool paused) { if (paused) InvalidateTracking("App paused. Capture the surface again."); }
    private void OnApplicationFocus(bool focus) { if (!focus && IsCapturing) CancelWithReason("App lost focus. Capture all three corners again."); }
    private void OnDisable()
    {
        Cancel(); SetTrackingLost(); _guide?.Hide();
        if (_itemVisual != null) _itemVisual.SetActive(false);
        foreach (XRInputSubsystem subsystem in _subsystems) subsystem.trackingOriginUpdated -= OnTrackingOriginUpdated;
        _subsystems.Clear();
    }
    private void OnDestroy()
    {
        ClearPointVisuals(); Dispose(_tipVisual); Dispose(_tipMaterial);
        RobotContourDisplay.UnregisterItem(_itemRenderer);
        if (_itemVisual != null) _itemVisual.SetActive(false);
        Dispose(_itemVisual); Dispose(_itemMaterial);
        if (_guide != null) Dispose(_guide.gameObject);
        Changed = null; SurfaceCaptured = null;
    }
    private static void Dispose(UnityEngine.Object value)
    {
        if (value == null) return;
        if (Application.isPlaying) Destroy(value); else DestroyImmediate(value);
    }
}
