using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

/// <summary>Session-local registration from five rear blue covers, ordered from base to tool.</summary>
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
        new Color(0.85f, 0.4f, 1f), new Color(0.2f, 0.85f, 1f), new Color(1f, 0.9f, 0.2f)
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
    private Pose _fittedBaseInTracking;
    private Transform _fitTrackingSpace;
    private Vector3 _positionAdjustmentInRosMeters;
    private Vector3 _adjustmentBeforeEdit;
    private float _previousFitRms, _previousFitMax;
    public const float MaximumPositionAdjustmentMeters = 0.02f;

    public int RequiredPointCount => 5;
    public bool IsCollecting { get; private set; }
    public bool HasPlacement { get; private set; }
    public bool HasFitFailure { get; private set; }
    public string FitFailureText { get; private set; } = "";
    public bool IsAdjustingPosition { get; private set; }
    public Vector3 PositionAdjustmentInRosMeters => _positionAdjustmentInRosMeters;
    public bool CanAdjustPosition => HasPlacement && !IsCollecting && _instantiator != null &&
        _instantiator.IsPointPlaced && _instantiator.RobotInstance != null && !_instantiator.IsCalibrating &&
        !RobotExecutionState.IsBusy && MTCPlansManager.ActiveExecution == null &&
        RobotFrameMath.IsRigidFrame(_fitTrackingSpace) && RobotFrameMath.IsValidPose(_fittedBaseInTracking) &&
        (IsAdjustingPosition ? RobotCoordinateService.IsCalibrationEditActive : RobotCoordinateService.Instance.IsInitialized);
    public int CapturedPointCount => _measuredInTracking.Count;
    public float RmsErrorMeters { get; private set; }
    public float MaxErrorMeters { get; private set; }
    public string StatusText { get; private set; } = "Place the robot using five rear blue cover centres, starting at the base and working towards the tool.";
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
        ? _landmarks[CapturedPointCount].displayName : "All five points collected";

    /// <summary>Read-only miniature inputs. A captured registration always uses its first-touch joint pose.</summary>
    public bool GetGuideState(out RosRobotModel model, out Dictionary<string, double> joints,
        out RobotCoverLandmark[] landmarks, out bool poseLocked)
    {
        model = _referenceModel ?? CurrentModel();
        landmarks = _landmarks;
        poseLocked = _referenceJoints != null;
        if (poseLocked)
        {
            joints = new Dictionary<string, double>(_referenceJoints);
            return true;
        }
        if (_jointFeedback == null) _jointFeedback = FindFirstObjectByType<JointStateSubscriber>();
        if (_jointFeedback != null && _jointFeedback.TryGetFreshJointPositions(model,
            _maximumJointAgeSeconds, out joints, out _)) return true;
        joints = null;
        return false;
    }

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
        if (IsAdjustingPosition) CancelPositionAdjustment();
        _landmarks = RobotCoverLandmarks.UpgradeDefaults(_landmarks);
        if (_landmarks == null || _landmarks.Length != RequiredPointCount)
        { SetStatus("Configure five blue cover landmarks, ordered from base to tool, before point placement."); return; }
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
            _previousFitRms = RmsErrorMeters; _previousFitMax = MaxErrorMeters;
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
        SetStatus("Keep the robot still. Touch 1/5: " + CurrentPointName + "; press trigger.");
    }

    /// <summary>Called only for a tracked pointer pose, after XR UI has consumed its trigger.</summary>
    public bool CapturePointerPose(Pose pointerWorld, Transform trackingSpace)
    {
        if (!IsCollecting) return false;
        if (!RobotFrameMath.IsValidPose(pointerWorld) || !RobotFrameMath.IsRigidFrame(trackingSpace) ||
            !RobotFrameMath.IsValidPose(new Pose(_tipOffsetInPointer, Quaternion.identity)))
        { SetStatus("A tracked controller and a valid metre-scale tracking frame are required."); return false; }
        if (CapturedPointCount >= RequiredPointCount) { SetStatus(HasFitFailure ? FitFailureText : "Undo the last point or restart to retry the fit."); return false; }
        if (_captureTrackingSpace != null && _captureTrackingSpace != trackingSpace)
        { RestartPicks("The XR tracking frame changed. Pick all five points again."); return false; }
        RosRobotModel model = CurrentModel();
        if (_jointFeedback == null) _jointFeedback = FindFirstObjectByType<JointStateSubscriber>();
        if (_jointFeedback == null)
        { SetStatus("Waiting for the ROS joint-state subscriber."); return false; }
        if (!_jointFeedback.TryGetFreshJointPositions(model, _maximumJointAgeSeconds,
            out Dictionary<string, double> joints, out string error))
        { SetStatus(error); return false; }
        if (_referenceModel != null && _referenceModel != model)
        { RestartPicks("The robot model changed. Pick all five points again."); return false; }
        if (_referenceJoints != null && JointsMoved(joints))
        { RestartPicks("The robot moved. Keep it still and pick all five points again."); return false; }
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
        ClearFitFailure();
        if (CapturedPointCount < RequiredPointCount)
        { SetStatus("Touch " + (CapturedPointCount + 1) + "/5: " + CurrentPointName + "; press trigger."); return true; }
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
        { FailFit("Five-point fit FAILED. " + error + " No new placement was applied; planning remains blocked. Restart and capture all five points in order."); return false; }
        RmsErrorMeters = rms;
        MaxErrorMeters = worst;
        if (!(_maximumRmsErrorMeters > 0f) || !(_maximumPointErrorMeters > 0f) ||
            float.IsInfinity(_maximumRmsErrorMeters) || float.IsInfinity(_maximumPointErrorMeters) ||
            rms > _maximumRmsErrorMeters || worst > _maximumPointErrorMeters)
        {
            int worstIndex = 0; float largest = -1f;
            for (int i = 0; i < _modelPoints.Length; i++)
            {
                float mismatch = Vector3.Distance(baseInTracking.position + baseInTracking.rotation * _modelPoints[i], _measuredInTracking[i]);
                if (mismatch > largest) { largest = mismatch; worstIndex = i; }
            }
            FailFit("Five-point fit FAILED — fit rejected. No new XR placement was applied; planning remains blocked.\n" +
                "RMS " + (rms * 1000f).ToString("F1") + " mm (limit " + (_maximumRmsErrorMeters*1000f).ToString("F1") +
                "); worst " + (worst * 1000f).ToString("F1") + " mm (limit " + (_maximumPointErrorMeters*1000f).ToString("F1") +
                "). Largest mismatch: point " + (worstIndex+1) + " " + _landmarks[worstIndex].displayName + ".\n" +
                "Undo recaptures point 5. For an earlier point, Restart. Check point order, contact centres and controller tip offset; keep the real robot still.");
            return false;
        }
        Pose baseWorld = RobotFrameMath.Compose(new Pose(_captureTrackingSpace.position, _captureTrackingSpace.rotation), baseInTracking);
        RobotCoordinateService coordinates = RobotCoordinateService.Instance;
        coordinates.EndCalibrationEdit();
        if (!_instantiator.PlaceFromPointRegistration(baseWorld, _captureTrackingSpace, _referenceJoints))
        {
            coordinates.BeginCalibrationEdit();
            FailFit("Five-point fit FAILED to apply. The measured fit is valid, but the XR robot could not be placed. Planning remains blocked. Check the robot prefab/model, then Restart.");
            return false;
        }
        IsCollecting = false;
        HasPlacement = true;
        _fittedBaseInTracking = baseInTracking;
        _fitTrackingSpace = _captureTrackingSpace;
        _positionAdjustmentInRosMeters = Vector3.zero;
        ClearFitFailure();
        RestoreRobotVisibility();
        HideTip();
        SetStatus("Placed from 5 points. RMS " + (rms * 1000f).ToString("F1") + " mm; worst " +
            (worst * 1000f).ToString("F1") + " mm. Check the overlay; use Adjust position for small robot-XYZ corrections.");
        return true;
    }

    /// <summary>Session-only translation of the XR overlay. Never sends robot motion or changes joint angles.</summary>
    public void BeginPositionAdjustment()
    {
        if (IsAdjustingPosition) return;
        if (!CanAdjustPosition)
        { SetStatus("Complete a valid five-point placement and finish any robot execution before adjusting the XR position."); return; }
        _adjustmentBeforeEdit = _positionAdjustmentInRosMeters;
        IsAdjustingPosition = true;
        TargetPoseContextMenu.CloseAll();
        TargetRotationHandles.CancelAllInteractions();
        RobotCoordinateService.Instance.BeginCalibrationEdit();
        RobotStateService.Instance.IsReadyForPlanning = false;
        SetStatus("Adjust the XR overlay in robot XYZ. Planning is blocked until Done. Maximum total correction: 20 mm from the five-point fit.");
    }

    public bool NudgeRobotPosition(Vector3 deltaInRosMeters)
    {
        if (!IsAdjustingPosition || !CanAdjustPosition)
        { SetStatus("Position adjustment requires an active five-point alignment edit and no unresolved execution."); return false; }
        if (!RobotFrameMath.IsValidPose(new Pose(deltaInRosMeters, Quaternion.identity)))
        { SetStatus("The requested XYZ correction is not finite."); return false; }
        Vector3 candidate = _positionAdjustmentInRosMeters + deltaInRosMeters;
        if (!RobotFrameMath.IsValidPose(new Pose(candidate, Quaternion.identity)) ||
            candidate.magnitude > MaximumPositionAdjustmentMeters + 0.0000001f)
        { SetStatus("20 mm total correction limit reached. Reduce another axis, Reset, or repeat the five-point fit for a larger error."); return false; }
        if (!ApplyPositionOffset(candidate)) return false;
        _positionAdjustmentInRosMeters = candidate;
        SetStatus("XR position correction: " + AdjustmentDescription() + ". Robot axes; original fit residuals are unchanged. Select Done before planning.");
        return true;
    }

    public void ResetPositionAdjustment()
    {
        if (!IsAdjustingPosition || !CanAdjustPosition || !ApplyPositionOffset(Vector3.zero)) return;
        _positionAdjustmentInRosMeters = Vector3.zero;
        SetStatus("Original five-point fit position restored. Select Done to finish adjustment.");
    }

    public void EndPositionAdjustment()
    {
        if (!IsAdjustingPosition) return;
        RobotCoordinateService coordinates = RobotCoordinateService.Instance;
        // Scene navigation resets coordinates before unloading this component.
        // Never reinitialize the outgoing robot after that reset.
        if (!coordinates.IsCalibrationInProgress)
        { IsAdjustingPosition = false; SetStatus("Robot alignment changed. Confirm alignment again before planning."); return; }
        if (!CanAdjustPosition || !ApplyPositionOffset(_positionAdjustmentInRosMeters))
        { SetStatus("Cannot finish position adjustment while execution or tracking is unresolved. Keep planning blocked and restore tracking first."); return; }
        FinishPositionEdit(coordinates, "Five-point alignment adjusted by " + AdjustmentDescription() + ". Review the overlay and plan again.");
    }

    public void CancelPositionAdjustment()
    {
        if (!IsAdjustingPosition) return;
        RobotCoordinateService coordinates = FindFirstObjectByType<RobotCoordinateService>();
        _positionAdjustmentInRosMeters = _adjustmentBeforeEdit;
        if (coordinates != null && coordinates.IsCalibrationInProgress &&
            HasPlacement && RobotFrameMath.IsRigidFrame(_fitTrackingSpace) &&
            _instantiator != null && _instantiator.RobotInstance != null && ApplyPositionOffset(_adjustmentBeforeEdit))
            FinishPositionEdit(coordinates, "Position adjustment cancelled. Previous XR position restored.");
        else { IsAdjustingPosition = false; StateChanged?.Invoke(); }
    }

    private bool ApplyPositionOffset(Vector3 offsetRos)
    {
        RosRobotRig rig = _instantiator?.RobotInstance != null ? _instantiator.RobotInstance.GetComponent<RosRobotRig>() : null;
        if (rig == null || rig.BaseLink == null || !RobotFrameMath.IsRigidFrame(_fitTrackingSpace))
        { SetStatus("The XR robot or tracking frame is unavailable. Repeat robot placement."); return false; }
        Vector3 localDelta = RosRobotModelMath.FromRosVector(new[] { offsetRos.x, offsetRos.y, offsetRos.z });
        Pose adjusted = new Pose(_fittedBaseInTracking.position + _fittedBaseInTracking.rotation * localDelta,
            _fittedBaseInTracking.rotation);
        Pose world = RobotFrameMath.Compose(new Pose(_fitTrackingSpace.position, _fitTrackingSpace.rotation), adjusted);
        if (!RobotFrameMath.IsValidPose(world)) return false;
        rig.SetBaseWorldPose(world);
        return true;
    }

    private void FinishPositionEdit(RobotCoordinateService coordinates, string status)
    {
        RosRobotRig rig = _instantiator.RobotInstance.GetComponent<RosRobotRig>();
        Pose world = new Pose(rig.BaseLink.position, rig.BaseLink.rotation);
        coordinates.EndCalibrationEdit();
        RobotStateService.Instance.InitializeRobot(world, _instantiator.RobotInstance, _fitTrackingSpace);
        IsAdjustingPosition = false;
        if (!coordinates.IsInitialized)
        { HasPlacement = false; SetStatus("XR position correction could not be applied. Repeat the five-point placement before planning."); return; }
        RobotStateService.Instance.IsReadyForPlanning = false;
        SetStatus(status);
    }
    private string AdjustmentDescription() => "X " + (_positionAdjustmentInRosMeters.x*1000f).ToString("F1") +
        ", Y " + (_positionAdjustmentInRosMeters.y*1000f).ToString("F1") +
        ", Z " + (_positionAdjustmentInRosMeters.z*1000f).ToString("F1") + " mm";
    private void ClearFitFailure() { HasFitFailure = false; FitFailureText = ""; }
    private void FailFit(string reason) { HasFitFailure = true; FitFailureText = reason; SetStatus(reason); }

    public void UndoLastPoint()
    {
        if (!IsCollecting || CapturedPointCount == 0) return;
        _measuredInTracking.RemoveAt(CapturedPointCount - 1);
        ClearFitFailure();
        RemoveLastCapturedPointVisual();
        if (CapturedPointCount == 0) ClearPicks();
        SetStatus("Touch " + (CapturedPointCount + 1) + "/5: " + CurrentPointName + "; press trigger.");
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
        if (_hadUsablePlacement) { RmsErrorMeters = _previousFitRms; MaxErrorMeters = _previousFitMax; }
        SetStatus("Point placement cancelled." + (_hadUsablePlacement ? " Previous placement restored." : " Choose five points or scan QR."));
    }

    public void UseQrPlacement()
    {
        if (IsAdjustingPosition) CancelPositionAdjustment();
        CancelRegistration();
        ClearPicks();
        HasPlacement = false;
        _fitTrackingSpace = null; _positionAdjustmentInRosMeters = Vector3.zero;
        if (_instantiator == null) _instantiator = GetComponent<URInstantiator>();
        if (_instantiator != null) _instantiator.SelectQrPlacement();
        QRDetector detector = _instantiator != null ? _instantiator.MarkerDetector : null;
        if (detector != null) { detector.enabled = true; detector.Rescan(); }
        SetStatus("QR placement selected. Scan the marker, then trigger to place the robot.");
    }

    public void NotifyQrPlacement() { ClearPicks(); HideTip(); HasPlacement = false; IsAdjustingPosition = false; _fitTrackingSpace = null; _positionAdjustmentInRosMeters = Vector3.zero; StateChanged?.Invoke(); }

    /// <summary>Native reference-space changes invalidate saved contacts; moving the XR Origin transform does not.</summary>
    public void InvalidateTrackingSession(string reason)
    {
        if (!HasPlacement && !IsCollecting) return;
        HasPlacement = false;
        IsAdjustingPosition = false;
        _fitTrackingSpace = null;
        _positionAdjustmentInRosMeters = _adjustmentBeforeEdit = Vector3.zero;
        _hadUsablePlacement = false;
        RobotCoordinateService coordinates = FindFirstObjectByType<RobotCoordinateService>();
        if (coordinates != null) coordinates.InvalidateCalibration();
        RobotStateService state = FindFirstObjectByType<RobotStateService>();
        if (state != null) state.IsReadyForPlanning = false;
        if (!IsCollecting) BeginRegistration();
        RestartPicks(reason + " Pick all five points again.");
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
        ClearFitFailure();
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
        if (IsAdjustingPosition) CancelPositionAdjustment();
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
    private void OnApplicationFocus(bool focus) { if (!focus && IsCollecting) RestartPicks("App lost focus; pick all five points again."); }
}
