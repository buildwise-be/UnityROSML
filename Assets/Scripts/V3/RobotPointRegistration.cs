using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

/// <summary>Session-local, ordered contact registration of the UR rear joint covers.</summary>
[DisallowMultipleComponent]
public sealed class RobotPointRegistration : MonoBehaviour
{
    [SerializeField] private URInstantiator _instantiator;
    [SerializeField] private JointStateSubscriber _jointFeedback;
    [SerializeField] private Transform _trackingSpace;
    [SerializeField] private RobotCoverLandmark[] _landmarks = RobotCoverLandmarks.CreateDefaults();
    [SerializeField, Tooltip("Physical contact tip relative to the ML2 OpenXR POINTER pose, in Unity local metres. Zero uses the pointer origin; verify/measure on the headset.")]
    private Vector3 _tipOffsetInPointer = Vector3.zero;
    [SerializeField, Min(0.001f)] private float _maximumRmsErrorMeters = 0.01f;
    [SerializeField, Min(0.001f)] private float _maximumPointErrorMeters = 0.02f;
    [SerializeField, Min(0.1f)] private float _maximumJointAgeSeconds = 1f;
    [SerializeField, Min(0.01f)] private float _maximumJointMovementDegrees = 0.5f;
    [SerializeField, Min(0.001f)] private float _capturedPointDiameterMeters = 0.012f;

    private readonly List<Vector3> _measuredInTracking = new List<Vector3>();
    private readonly List<GameObject> _capturedPointVisuals = new List<GameObject>();
    private readonly List<Material> _capturedPointMaterials = new List<Material>();
    private static readonly Color[] CapturedPointColors = {
        new Color(0.3f, 1f, 0.5f), new Color(1f, 0.5f, 0.1f),
        new Color(0.85f, 0.4f, 1f), new Color(0.2f, 0.85f, 1f)
    };
    private readonly Dictionary<Renderer, bool> _hiddenRenderers = new Dictionary<Renderer, bool>();
    private Dictionary<string, double> _referenceJoints;
    private RosRobotModel _referenceModel;
    private RosRobotModel _fallbackModel;
    private Vector3[] _modelPoints;
    private Transform _captureTrackingSpace;
    private QRDetector _detector;
    private bool _detectorWasEnabled;
    private bool _hadUsablePlacement;
    private Pose _previousBaseInTracking;
    private Transform _previousTrackingSpace;
    private GameObject _tipVisual;
    private Material _tipMaterial;
    private RosRobotRig _hiddenRig;
    private readonly List<XRInputSubsystem> _inputSubsystems = new List<XRInputSubsystem>();

    public bool IsCollecting { get; private set; }
    public bool HasPlacement { get; private set; }
    public int CapturedPointCount => _measuredInTracking.Count;
    public float RmsErrorMeters { get; private set; }
    public float MaxErrorMeters { get; private set; }
    public string StatusText { get; private set; } = "Place the robot by touching four rear blue cover centers.";
    public event Action StateChanged;
    public string[] PointNames
    {
        get
        {
            if (_landmarks == null) return Array.Empty<string>();
            string[] names = new string[_landmarks.Length];
            for (int i = 0; i < names.Length; i++) names[i] = _landmarks[i].displayName;
            return names;
        }
    }
    public string CurrentPointName => _landmarks != null && CapturedPointCount < _landmarks.Length
        ? _landmarks[CapturedPointCount].displayName : "All four points collected";

    public void SetTipOffset(Vector3 offsetInPointer)
    {
        if (IsCollecting || !RobotFrameMath.IsValidPose(new Pose(offsetInPointer, Quaternion.identity))) return;
        _tipOffsetInPointer = offsetInPointer;
    }

    public void BeginRegistration()
    {
        if (_instantiator == null) _instantiator = GetComponent<URInstantiator>();
        if (_instantiator == null) { SetStatus("Robot placement component is missing."); return; }
        if (_instantiator.IsCalibrating || RobotExecutionState.IsBusy || MTCPlansManager.ActiveExecution != null)
        { SetStatus("Finish the current alignment or robot execution before picking points."); return; }
        if (_landmarks == null || _landmarks.Length != 4)
        { SetStatus("Configure four rear-cover landmarks before point placement."); return; }
        foreach (RobotCoverLandmark landmark in _landmarks)
            if (landmark == null || string.IsNullOrWhiteSpace(landmark.displayName))
            { SetStatus("Each rear-cover landmark needs a name and contact definition."); return; }
        SubscribeTrackingOrigin();
        if (!IsCollecting)
        {
            RobotCoordinateService coordinates = RobotCoordinateService.Instance;
            _hadUsablePlacement = coordinates.TryGetRobotBaseWorldPose(out Pose previousBase) &&
                XRTrackingSpaceResolver.TryResolve(ref _trackingSpace);
            _previousTrackingSpace = _hadUsablePlacement ? _trackingSpace : null;
            if (_hadUsablePlacement)
                _previousBaseInTracking = RobotFrameMath.RelativeTo(new Pose(_trackingSpace.position, _trackingSpace.rotation), previousBase);
            _detector = _instantiator.MarkerDetector;
            _detectorWasEnabled = _detector != null && _detector.enabled;
            if (_detector != null) _detector.enabled = false;
            IsCollecting = true;
            coordinates.BeginCalibrationEdit();
            RobotStateService.Instance.IsReadyForPlanning = false;
            TargetPoseContextMenu.CloseAll();
            TargetRotationHandles.CancelAllInteractions();
            HideRobot();
        }
        ClearPicks();
        SetStatus("Keep the robot still. Touch 1/4: " + CurrentPointName + "; press trigger.");
    }

    /// <summary>Called only for a tracked pointer pose, after XR UI has consumed its trigger.</summary>
    public bool CapturePointerPose(Pose pointerWorld, Transform trackingSpace)
    {
        if (!IsCollecting) return false;
        if (!RobotFrameMath.IsValidPose(pointerWorld) || !RobotFrameMath.IsRigidFrame(trackingSpace) ||
            !RobotFrameMath.IsValidPose(new Pose(_tipOffsetInPointer, Quaternion.identity)))
        { SetStatus("A tracked controller and a valid metre-scale tracking frame are required."); return false; }
        if (CapturedPointCount == 4) { SetStatus("Undo the last point or restart to retry the fit."); return false; }
        if (_captureTrackingSpace != null && _captureTrackingSpace != trackingSpace)
        { RestartPicks("The XR tracking frame changed. Pick all four points again."); return false; }
        RosRobotModel model = CurrentModel();
        if (_jointFeedback == null) _jointFeedback = FindFirstObjectByType<JointStateSubscriber>();
        if (_jointFeedback == null)
        { SetStatus("Waiting for the ROS joint-state subscriber."); return false; }
        if (!_jointFeedback.TryGetFreshJointPositions(model, _maximumJointAgeSeconds,
            out Dictionary<string, double> joints, out string error))
        { SetStatus(error); return false; }
        if (_referenceModel != null && _referenceModel != model)
        { RestartPicks("The robot model changed. Pick all four points again."); return false; }
        if (_referenceJoints != null && JointsMoved(joints))
        { RestartPicks("The robot moved. Keep it still and pick all four points again."); return false; }
        if (CapturedPointCount == 0)
        {
            if (!RobotCoverLandmarks.TryGetBasePoints(model, joints, _landmarks, out _modelPoints, out error))
            { SetStatus(error); return false; }
            _referenceJoints = joints;
            _referenceModel = model;
            _captureTrackingSpace = trackingSpace;
        }
        Vector3 tipWorld = pointerWorld.position + pointerWorld.rotation * _tipOffsetInPointer;
        Vector3 measured = trackingSpace.InverseTransformPoint(tipWorld);
        foreach (Vector3 previous in _measuredInTracking)
            if (Vector3.Distance(previous, measured) < 0.015f)
            { SetStatus("This point is too close to an earlier pick. Touch " + CurrentPointName + "."); return false; }
        _measuredInTracking.Add(measured);
        AddCapturedPointVisual(trackingSpace, measured, CapturedPointCount - 1);
        if (CapturedPointCount < 4)
        { SetStatus("Touch " + (CapturedPointCount + 1) + "/4: " + CurrentPointName + "; press trigger."); return true; }
        return TryFinish();
    }

    private RosRobotModel CurrentModel()
    {
        ToolSelectionController tools = ToolSelectionController.Instance;
        return tools != null ? tools.CurrentModel : _fallbackModel ?? (_fallbackModel = RosRobotModelCatalog.Load().Find("kenos"));
    }

    private bool JointsMoved(Dictionary<string, double> current)
    {
        foreach (KeyValuePair<string, double> joint in _referenceJoints)
            if (!current.TryGetValue(joint.Key, out double angle) ||
                Math.Abs(Math.IEEERemainder(angle - joint.Value, Math.PI * 2d)) * Mathf.Rad2Deg > _maximumJointMovementDegrees)
                return true;
        return false;
    }

    private bool TryFinish()
    {
        if (!RobotPointRegistrationMath.TryFit(_modelPoints, _measuredInTracking,
            out Pose baseInTracking, out float rms, out float worst, out string error))
        { SetStatus(error + " Undo a point or restart."); return false; }
        RmsErrorMeters = rms;
        MaxErrorMeters = worst;
        if (!(_maximumRmsErrorMeters > 0f) || !(_maximumPointErrorMeters > 0f) ||
            float.IsInfinity(_maximumRmsErrorMeters) || float.IsInfinity(_maximumPointErrorMeters) ||
            rms > _maximumRmsErrorMeters || worst > _maximumPointErrorMeters)
        {
            SetStatus("Fit rejected: RMS " + (rms * 1000f).ToString("F1") + " mm; worst " +
                (worst * 1000f).ToString("F1") + " mm. Check point order and tip offset; undo or restart.");
            return false;
        }
        Pose baseWorld = RobotFrameMath.Compose(new Pose(_captureTrackingSpace.position, _captureTrackingSpace.rotation), baseInTracking);
        RobotCoordinateService coordinates = RobotCoordinateService.Instance;
        coordinates.EndCalibrationEdit();
        if (!_instantiator.PlaceFromPointRegistration(baseWorld, _captureTrackingSpace, _referenceJoints))
        {
            coordinates.BeginCalibrationEdit();
            SetStatus("The fit is valid, but the robot could not be placed. Restart to retry.");
            return false;
        }
        IsCollecting = false;
        HasPlacement = true;
        RestoreRobotVisibility();
        HideTip();
        SetStatus("Placed from 4 points. RMS " + (rms * 1000f).ToString("F1") + " mm; worst " +
            (worst * 1000f).ToString("F1") + " mm. Check the overlay before planning.");
        return true;
    }

    public void UndoLastPoint()
    {
        if (!IsCollecting || CapturedPointCount == 0) return;
        _measuredInTracking.RemoveAt(CapturedPointCount - 1);
        RemoveLastCapturedPointVisual();
        if (CapturedPointCount == 0) ClearPicks();
        SetStatus("Touch " + (CapturedPointCount + 1) + "/4: " + CurrentPointName + "; press trigger.");
    }

    public void CancelRegistration()
    {
        if (!IsCollecting) return;
        IsCollecting = false;
        RobotCoordinateService coordinates = FindFirstObjectByType<RobotCoordinateService>();
        if (coordinates != null) coordinates.EndCalibrationEdit();
        if (_hadUsablePlacement && _instantiator != null && _instantiator.RobotInstance != null &&
            RobotFrameMath.IsRigidFrame(_previousTrackingSpace))
        {
            Pose world = RobotFrameMath.Compose(new Pose(_previousTrackingSpace.position, _previousTrackingSpace.rotation), _previousBaseInTracking);
            RosRobotRig rig = _instantiator.RobotInstance.GetComponent<RosRobotRig>();
            if (rig != null) rig.SetBaseWorldPose(world);
            RobotStateService state = FindFirstObjectByType<RobotStateService>();
            if (state != null) state.InitializeRobot(world, _instantiator.RobotInstance, _previousTrackingSpace);
        }
        if (_detector != null) _detector.enabled = _detectorWasEnabled;
        RestoreRobotVisibility();
        HideTip();
        ClearPicks();
        SetStatus("Point placement cancelled." + (_hadUsablePlacement ? " Previous placement restored." : " Choose four points or scan QR."));
    }

    public void UseQrPlacement()
    {
        CancelRegistration();
        ClearPicks();
        HasPlacement = false;
        if (_instantiator == null) _instantiator = GetComponent<URInstantiator>();
        if (_instantiator != null) _instantiator.SelectQrPlacement();
        QRDetector detector = _instantiator != null ? _instantiator.MarkerDetector : null;
        if (detector != null) { detector.enabled = true; detector.Rescan(); }
        SetStatus("QR placement selected. Scan the marker, then trigger to place the robot.");
    }

    public void NotifyQrPlacement() { ClearPicks(); HideTip(); HasPlacement = false; StateChanged?.Invoke(); }

    /// <summary>Native reference-space changes invalidate saved contacts; moving the XR Origin transform does not.</summary>
    public void InvalidateTrackingSession(string reason)
    {
        if (!HasPlacement && !IsCollecting) return;
        HasPlacement = false;
        _hadUsablePlacement = false;
        RobotCoordinateService coordinates = FindFirstObjectByType<RobotCoordinateService>();
        if (coordinates != null) coordinates.InvalidateCalibration();
        RobotStateService state = FindFirstObjectByType<RobotStateService>();
        if (state != null) state.IsReadyForPlanning = false;
        if (!IsCollecting) BeginRegistration();
        RestartPicks(reason + " Pick all four points again.");
    }

    private void OnEnable() => SubscribeTrackingOrigin();
    private void SubscribeTrackingOrigin()
    {
        foreach (XRInputSubsystem subsystem in _inputSubsystems) subsystem.trackingOriginUpdated -= OnTrackingOriginUpdated;
        _inputSubsystems.Clear();
        SubsystemManager.GetSubsystems(_inputSubsystems);
        foreach (XRInputSubsystem subsystem in _inputSubsystems) subsystem.trackingOriginUpdated += OnTrackingOriginUpdated;
    }
    private void OnTrackingOriginUpdated(XRInputSubsystem subsystem) => InvalidateTrackingSession("XR tracking origin reset.");

    public void SetPointerPose(Pose pointerWorld, Transform trackingSpace)
    {
        if (!IsCollecting || !RobotFrameMath.IsValidPose(pointerWorld) || !RobotFrameMath.IsRigidFrame(trackingSpace))
        { HideTip(); return; }
        if (_tipVisual == null)
        {
            _tipVisual = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            _tipVisual.name = "Registration contact tip (verify against physical controller)";
            Collider collider = _tipVisual.GetComponent<Collider>();
            collider.enabled = false;
            _tipVisual.transform.localScale = Vector3.one * 0.006f;
            _tipMaterial = TargetMarkerMaterials.Create(new Color(0.55f, 0.95f, 1f));
            _tipVisual.GetComponent<Renderer>().sharedMaterial = _tipMaterial;
        }
        _tipVisual.SetActive(true);
        _tipVisual.transform.SetParent(trackingSpace, true);
        _tipVisual.transform.position = pointerWorld.position + pointerWorld.rotation * _tipOffsetInPointer;
    }

    public void SetTrackingLost() => HideTip();
    private void HideTip() { if (_tipVisual != null) _tipVisual.SetActive(false); }
    private void RestartPicks(string reason) { ClearPicks(); SetStatus(reason + " Start at " + CurrentPointName + "."); }
    private void ClearPicks()
    {
        ClearCapturedPointVisuals();
        _measuredInTracking.Clear();
        _referenceJoints = null;
        _referenceModel = null;
        _modelPoints = null;
        _captureTrackingSpace = null;
        RmsErrorMeters = MaxErrorMeters = 0f;
    }
    private void AddCapturedPointVisual(Transform trackingSpace, Vector3 measured, int index)
    {
        Material material = TargetMarkerMaterials.Create(CapturedPointColors[index % CapturedPointColors.Length]);
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.name = "Registration point " + (index + 1) + " - " + _landmarks[index].displayName;
        marker.GetComponent<Collider>().enabled = false;
        marker.GetComponent<Renderer>().sharedMaterial = material;
        marker.transform.SetParent(trackingSpace, false);
        marker.transform.localPosition = measured;
        marker.transform.localRotation = Quaternion.identity;
        float diameter = _capturedPointDiameterMeters > 0f && !float.IsInfinity(_capturedPointDiameterMeters)
            ? _capturedPointDiameterMeters : 0.012f;
        marker.transform.localScale = Vector3.one * diameter;
        _capturedPointVisuals.Add(marker);
        _capturedPointMaterials.Add(material);
    }
    private void RemoveLastCapturedPointVisual()
    {
        if (_capturedPointVisuals.Count == 0) return;
        int index = _capturedPointVisuals.Count - 1;
        GameObject marker = _capturedPointVisuals[index];
        if (marker != null) marker.SetActive(false);
        DisposeVisual(marker);
        DisposeVisual(_capturedPointMaterials[index]);
        _capturedPointVisuals.RemoveAt(index);
        _capturedPointMaterials.RemoveAt(index);
    }
    private void ClearCapturedPointVisuals()
    {
        while (_capturedPointVisuals.Count != 0) RemoveLastCapturedPointVisual();
    }
    private static void DisposeVisual(UnityEngine.Object value)
    {
        if (value == null) return;
        if (Application.isPlaying) Destroy(value); else DestroyImmediate(value);
    }
    private void HideRobot()
    {
        _hiddenRenderers.Clear();
        if (_instantiator.RobotInstance == null) return;
        _hiddenRig = _instantiator.RobotInstance.GetComponent<RosRobotRig>();
        if (_hiddenRig != null) _hiddenRig.ModelChanged += HideCurrentRenderers;
        HideCurrentRenderers();
    }
    private void HideCurrentRenderers()
    {
        if (!IsCollecting || _instantiator == null || _instantiator.RobotInstance == null) return;
        foreach (Renderer renderer in _instantiator.RobotInstance.GetComponentsInChildren<Renderer>(true))
        {
            if (!_hiddenRenderers.ContainsKey(renderer)) _hiddenRenderers[renderer] = renderer.enabled;
            renderer.enabled = false;
        }
    }
    private void RestoreRobotVisibility()
    {
        if (_hiddenRig != null) _hiddenRig.ModelChanged -= HideCurrentRenderers;
        _hiddenRig = null;
        foreach (KeyValuePair<Renderer, bool> entry in _hiddenRenderers)
            if (entry.Key != null) entry.Key.enabled = entry.Value;
        _hiddenRenderers.Clear();
    }
    private void SetStatus(string status) { StatusText = status; StateChanged?.Invoke(); }
    private void LateUpdate() { if (IsCollecting) HideCurrentRenderers(); }
    private void OnDisable()
    {
        CancelRegistration();
        foreach (XRInputSubsystem subsystem in _inputSubsystems) subsystem.trackingOriginUpdated -= OnTrackingOriginUpdated;
        _inputSubsystems.Clear();
    }
    private void OnDestroy()
    {
        ClearCapturedPointVisuals();
        DisposeVisual(_tipVisual);
        DisposeVisual(_tipMaterial);
    }
    private void OnApplicationPause(bool paused) { if (paused) InvalidateTrackingSession("App paused."); }
    private void OnApplicationFocus(bool focus) { if (!focus && IsCollecting) RestartPicks("App lost focus; pick all four points again."); }
}
