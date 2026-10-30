using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class URInstantiator : MonoBehaviour
{
    [SerializeField]
    private QRDetector _QRDetector;

    [SerializeField]
    private GameObject _URPrefab;

    [SerializeField, Tooltip("Use an explicitly measured QR-to-base pose instead of the prefab calibration. Distances are metres in the detected QR's Unity axes.")]
    private bool _overridePrefabCalibration;
    [SerializeField] private Vector3 _basePositionInMarker = new Vector3(-0.419f, 0.230f, 0.505f);
    [SerializeField] private Vector3 _baseEulerInMarker = new Vector3(-90f, 0f, 0f);
    [SerializeField, Tooltip("Stable identity for this physical robot's per-QR saved alignment; independent of its selected tool.")]
    private string _calibrationProfileId = "ur16e";
    [SerializeField, Tooltip("Physical ML2 contact tip relative to its OpenXR pointer pose, in local Unity metres. Measure/verify on the headset. Used when the point-registration component is created automatically.")]
    private Vector3 _controllerTipOffsetInPointer = Vector3.zero;
    // Isolated tests inject a temporary directory. Normal applications use persistentDataPath.
    private string _calibrationDirectoryOverride;

    /// <summary>
    /// Arguments are the UR instance and the world pose of the QR location.
    /// </summary>
    public UnityEvent<GameObject, Pose> OnURInstantiated = new UnityEvent<GameObject, Pose>();

    private QRDetector _subscribedDetector;
    private bool _hasValidatedPose;
    private GameObject _robotInstance;
    private int _appliedValidationRevision = -1;
    private int _loadedValidationRevision = -1;
    private string _loadedPayload;
    private RobotCalibrationProfile _savedCalibration;
    private bool _hasActiveCalibration;
    private Pose _activeCalibration;
    private Pose _draftCalibration;
    private Pose _calibrationBeforeEdit;
    private Pose _markerInTrackingAtEdit;
    private Pose _baseWorldBeforeEdit;
    private Transform _editingTrackingSpace;
    private int _editingValidationRevision;
    private string _editingPayload;
    private float _editingMarkerSize;
    private bool _placedWithPoints;
    public GameObject RobotInstance => _robotInstance;
    public GameObject RobotPrefab => _URPrefab;
    public bool IsPointPlaced => _placedWithPoints;
    public QRDetector MarkerDetector => _QRDetector;

    public bool HasRobot => _robotInstance != null;
    public bool IsCalibrating { get; private set; }
    public bool HasSavedCalibration => _savedCalibration != null;
    public string AlignmentStatus { get; private set; } = "Scan the robot QR, place the robot, then align its base.";
    public event Action CalibrationStateChanged;
    public bool CanCalibrate => !_placedWithPoints && _robotInstance != null && _QRDetector != null && _QRDetector.HasValidatedPose &&
        !string.IsNullOrWhiteSpace(_QRDetector.DetectedPayload) &&
        _appliedValidationRevision == _QRDetector.ValidationRevision &&
        RobotFrameMath.IsRigidFrame(_QRDetector.TrackingSpace) &&
        _robotInstance.GetComponent<RosRobotRig>() is RosRobotRig rig && rig.BaseLink != null &&
        RobotFrameMath.IsValidPose(new Pose(rig.BaseLink.position, rig.BaseLink.rotation));
    public Pose BasePoseInMarker => TryGetCalibration(out Pose pose) ? pose : default;
    public string CalibrationSummary
    {
        get
        {
            if (_placedWithPoints) return GetComponent<RobotPointRegistration>()?.HasPlacement == true
                ? "Robot placed from four rear-cover centers (this XR session)."
                : "Point placement is pending. Pick the four covers again.";
            if (!TryGetCalibration(out Pose pose)) return "QR-to-base calibration is unavailable.";
            Vector3 euler = pose.rotation.eulerAngles;
            string source = IsCalibrating ? "Unsaved alignment" : _hasActiveCalibration ? "Saved alignment" :
                _overridePrefabCalibration ? "Configured alignment" : "Prefab calibration";
            return source +
                ": QR to base (" + pose.position.x.ToString("F3") + ", " + pose.position.y.ToString("F3") + ", " +
                pose.position.z.ToString("F3") + ") m; Euler (" + Mathf.DeltaAngle(0f, euler.x).ToString("F1") + ", " +
                Mathf.DeltaAngle(0f, euler.y).ToString("F1") + ", " + Mathf.DeltaAngle(0f, euler.z).ToString("F1") + ") deg.";
        }
    }

    private void Awake()
    {
        if (OnURInstantiated == null) OnURInstantiated = new UnityEvent<GameObject, Pose>();
        ToolPoseVisual.SetReferencePrefab(_URPrefab);
        if (GetComponent<RobotPointRegistration>() == null)
            gameObject.AddComponent<RobotPointRegistration>().SetTipOffset(_controllerTipOffsetInPointer);
    }

    private void OnEnable()
    {
        SubscribeToDetector();
    }

    private void Start()
    {
        SubscribeToDetector();
        GetComponent<RobotPointRegistration>().BeginRegistration();
    }

    private void SubscribeToDetector()
    {
        if (_subscribedDetector != null)
        {
            _subscribedDetector.OnQRValidated.RemoveListener(HandleQRValidated);
            _subscribedDetector.StateChanged -= HandleDetectorStateChanged;
        }
        _subscribedDetector = _QRDetector;
        _hasValidatedPose = _QRDetector != null && _QRDetector.HasValidatedPose;
        if (_subscribedDetector != null)
        {
            _subscribedDetector.OnQRValidated.AddListener(HandleQRValidated);
            _subscribedDetector.StateChanged += HandleDetectorStateChanged;
        }
    }

    private void HandleQRValidated(Pose worldPose)
    {
        _hasValidatedPose = RobotFrameMath.IsValidPose(worldPose);
        if (IsCalibrating && !EditingContextIsValid()) CancelCalibration();
        if (_hasValidatedPose) LoadSavedCalibration();
    }

    public void InstantiateUR()
    {
        RobotPointRegistration points = GetComponent<RobotPointRegistration>();
        if (_placedWithPoints || (points != null && (points.IsCollecting || points.HasPlacement))) return;
        if (IsCalibrating || _QRDetector == null || (_robotInstance != null && _appliedValidationRevision == _QRDetector.ValidationRevision)) return;
        // Also supports a detector validated before this component subscribed.
        if (!_hasValidatedPose && _QRDetector.HasValidatedPose) HandleQRValidated(_QRDetector.StabilizedPose);
        if (!_hasValidatedPose || !_QRDetector.HasValidatedPose) return;
        LoadSavedCalibration();

        Transform trackingSpace = _QRDetector.TrackingSpace;
        Pose qrPose = _QRDetector.StabilizedPose;
        if (!RobotFrameMath.IsRigidFrame(trackingSpace) || !RobotFrameMath.IsValidPose(qrPose) ||
            _URPrefab == null || !TryGetCalibration(out Pose baseInMarker))
        {
            Debug.LogError("URInstantiator: A valid QR pose and unit-scale tracking frame and robot prefab are required.", this);
            return;
        }

        bool firstPlacement = _robotInstance == null;
        GameObject robotInstance = _robotInstance;
        RosRobotRig rig;
        if (firstPlacement)
        {
            robotInstance = Instantiate(_URPrefab, qrPose.position, qrPose.rotation);
            robotInstance.transform.SetParent(trackingSpace, true);
            rig = robotInstance.GetComponent<RosRobotRig>();
            if (rig == null) rig = robotInstance.AddComponent<RosRobotRig>();
            ToolSelectionController tools = ToolSelectionController.Instance;
            bool configured = tools != null ? tools.ConfigureRobot(rig) : rig.ApplyModel(RosRobotModelCatalog.Load().Find("kenos"));
            if (!configured)
            {
                Debug.LogError("URInstantiator: The ROS robot model could not be applied.", this);
                Destroy(robotInstance);
                return;
            }
        }
        else rig = robotInstance.GetComponent<RosRobotRig>();
        if (rig == null || rig.BaseLink == null) return;

        // Apply the measured extrinsic once. Reanchoring retains the robot, tool and joint feedback.
        robotInstance.transform.SetParent(trackingSpace, true);
        rig.SetBaseWorldPose(RobotFrameMath.Compose(qrPose, baseInMarker));
        RobotStateService.Instance.InitializeRobot(qrPose, robotInstance, trackingSpace);
        if (!RobotCoordinateService.Instance.IsInitialized) return;
        _robotInstance = robotInstance;
        _placedWithPoints = false;
        if (points != null) points.NotifyQrPlacement();
        _appliedValidationRevision = _QRDetector.ValidationRevision;
        Pose baseWorld = new Pose(rig.BaseLink.position, rig.BaseLink.rotation);
        Debug.Log("URInstantiator: " + (firstPlacement ? "placed" : "realigned") + " robot. " + CalibrationSummary +
            " World base position=" + baseWorld.position.ToString("F6") + ", quaternion=" + baseWorld.rotation.ToString("F6") + ".", this);
        if (firstPlacement) OnURInstantiated.Invoke(robotInstance, qrPose);
        ToolSelectionMenu menu = FindFirstObjectByType<ToolSelectionMenu>();
        if (menu != null) menu.PositionAt(RobotStateService.Instance.GetRobotRootPose());
        if (!AlignmentStatus.StartsWith("Saved alignment could not", StringComparison.Ordinal))
            SetAlignmentStatus(_hasActiveCalibration ? "Saved alignment applied for this QR." : "Default alignment applied. Use Align robot to adjust it.");
    }

    /// <summary>Place directly in tracking space; no QR extrinsic is involved.</summary>
    public bool PlaceFromPointRegistration(Pose baseWorld, Transform trackingSpace, IDictionary<string, double> joints)
    {
        if (IsCalibrating || RobotCoordinateService.IsCalibrationEditActive ||
            !RobotFrameMath.IsValidPose(baseWorld) || !RobotFrameMath.IsRigidFrame(trackingSpace) ||
            _URPrefab == null || joints == null || joints.Count != 6) return false;
        bool firstPlacement = _robotInstance == null;
        GameObject robot = _robotInstance;
        if (firstPlacement) robot = Instantiate(_URPrefab, trackingSpace);
        RosRobotRig rig = robot.GetComponent<RosRobotRig>();
        if (rig == null) rig = robot.AddComponent<RosRobotRig>();
        ToolSelectionController tools = ToolSelectionController.Instance;
        bool configured = rig.Model != null || (tools != null
            ? tools.ConfigureRobot(rig) : rig.ApplyModel(RosRobotModelCatalog.Load().Find("kenos")));
        if (!configured || rig.BaseLink == null ||
            !rig.SetJointPositions(new List<string>(joints.Keys).ToArray(), new List<double>(joints.Values).ToArray()))
        {
            if (firstPlacement) Destroy(robot);
            return false;
        }
        robot.transform.SetParent(trackingSpace, true);
        rig.SetBaseWorldPose(baseWorld);
        RobotStateService.Instance.InitializeRobot(baseWorld, robot, trackingSpace);
        if (!RobotCoordinateService.Instance.IsInitialized)
        { if (firstPlacement) Destroy(robot); return false; }
        _robotInstance = robot;
        _placedWithPoints = true;
        _appliedValidationRevision = -1;
        RobotStateService.Instance.IsReadyForPlanning = false;
        if (firstPlacement) OnURInstantiated.Invoke(robot, baseWorld);
        SetAlignmentStatus("Four-point placement applied. Review the overlay before planning.");
        return true;
    }

    public void SelectQrPlacement() { _placedWithPoints = false; }

    private bool TryGetCalibration(out Pose pose)
    {
        if (IsCalibrating) { pose = _draftCalibration; return RobotFrameMath.IsValidPose(pose); }
        if (_hasActiveCalibration) { pose = _activeCalibration; return RobotFrameMath.IsValidPose(pose); }
        return TryGetDefaultCalibration(out pose);
    }

    private bool TryGetDefaultCalibration(out Pose pose)
    {
        pose = default;
        if (_overridePrefabCalibration)
        {
            pose = new Pose(_basePositionInMarker, Quaternion.Euler(_baseEulerInMarker));
            return RobotFrameMath.IsValidPose(pose);
        }
        if (_URPrefab == null || !RobotFrameMath.IsRigidFrame(_URPrefab.transform)) return false;
        foreach (ArticulationBody body in _URPrefab.GetComponentsInChildren<ArticulationBody>(true))
        {
            if (body.name != "base_link") continue;
            if (!RobotFrameMath.IsRigidFrame(body.transform)) return false;
            pose = RobotFrameMath.RelativeTo(new Pose(_URPrefab.transform.position, _URPrefab.transform.rotation),
                new Pose(body.transform.position, body.transform.rotation));
            return RobotFrameMath.IsValidPose(pose);
        }
        return false;
    }

    private void OnDisable()
    {
        if (IsCalibrating) CancelCalibration();
        if (_subscribedDetector != null)
        {
            _subscribedDetector.OnQRValidated.RemoveListener(HandleQRValidated);
            _subscribedDetector.StateChanged -= HandleDetectorStateChanged;
            _subscribedDetector = null;
        }
    }

    private RobotCalibrationStore CalibrationStore() => new RobotCalibrationStore(
        string.IsNullOrWhiteSpace(_calibrationDirectoryOverride)
            ? Path.Combine(Application.persistentDataPath, "RobotCalibrations") : _calibrationDirectoryOverride);

    private void LoadSavedCalibration()
    {
        if (IsCalibrating || _QRDetector == null || !_QRDetector.HasValidatedPose) return;
        string payload = _QRDetector.DetectedPayload;
        if (_loadedValidationRevision == _QRDetector.ValidationRevision && _loadedPayload == payload) return;
        _loadedValidationRevision = _QRDetector.ValidationRevision;
        _loadedPayload = payload;
        _savedCalibration = null;
        _hasActiveCalibration = false;
        if (string.IsNullOrWhiteSpace(payload))
        { SetAlignmentStatus("Scan an identified QR marker before saving alignment."); return; }
        string error;
        try
        {
            if (CalibrationStore().TryLoad(_calibrationProfileId, payload, _QRDetector.MarkerSizeMeters, out RobotCalibrationProfile profile, out error))
            {
                _savedCalibration = profile;
                _activeCalibration = profile.PoseInMarker;
                _hasActiveCalibration = true;
                SetAlignmentStatus("Saved alignment loaded. Trigger places or realigns the robot.");
                return;
            }
        }
        catch (Exception exception) when (exception is IOException || exception is UnauthorizedAccessException || exception is ArgumentException)
        { error = exception.Message; }
        if (error == null) SetAlignmentStatus("No saved alignment for this QR. Place the robot, then Align robot.");
        else
        {
            SetAlignmentStatus("Saved alignment could not be loaded. Using the default; check and save a new alignment.");
            Debug.LogWarning("URInstantiator: " + error, this);
        }
    }

    private void HandleDetectorStateChanged()
    {
        if (_QRDetector == null || _QRDetector.HasValidatedPose) return;
        bool hadCalibration = _loadedValidationRevision >= 0 || _hasValidatedPose || IsCalibrating;
        if (IsCalibrating) CancelCalibration();
        _hasValidatedPose = false;
        _loadedValidationRevision = -1;
        _loadedPayload = null;
        _savedCalibration = null;
        _hasActiveCalibration = false;
        if (hadCalibration) SetAlignmentStatus("Scan the QR again, then trigger to place or realign the robot.");
    }

    public void BeginCalibration()
    {
        if (RobotExecutionState.IsBusy) return;
        RobotPointRegistration points = GetComponent<RobotPointRegistration>();
        if (points != null && points.IsCollecting) return;
        if (IsCalibrating) return;
        if (!CanCalibrate)
        { SetAlignmentStatus("Scan one identified QR and trigger to place the robot before alignment."); return; }
        RobotCoordinateService coordinates = RobotCoordinateService.Instance;
        if (!coordinates.TryGetRobotBaseWorldPose(out _baseWorldBeforeEdit))
        { SetAlignmentStatus("Confirm QR placement again before alignment."); return; }
        Pose marker = _QRDetector.StabilizedPose;
        _calibrationBeforeEdit = RobotFrameMath.RelativeTo(marker, _baseWorldBeforeEdit);
        _draftCalibration = _calibrationBeforeEdit;
        _editingTrackingSpace = _QRDetector.TrackingSpace;
        _markerInTrackingAtEdit = RobotFrameMath.RelativeTo(new Pose(_editingTrackingSpace.position, _editingTrackingSpace.rotation), marker);
        _editingValidationRevision = _QRDetector.ValidationRevision;
        _editingPayload = _QRDetector.DetectedPayload;
        _editingMarkerSize = _QRDetector.MarkerSizeMeters;
        IsCalibrating = true;
        TargetRotationHandles.CancelAllInteractions();
        coordinates.BeginCalibrationEdit();
        RobotStateService.Instance.IsReadyForPlanning = false;
        ApplyDraftPose();
        SetAlignmentStatus("Adjust the virtual base and links to overlap the real robot, then Save.");
    }

    public void NudgePosition(Vector3 worldDelta)
    {
        if (!RequireEditingContext()) return;
        if (!RobotFrameMath.IsValidPose(new Pose(worldDelta, Quaternion.identity)))
        { SetAlignmentStatus("The requested position adjustment is invalid."); return; }
        Pose marker = _QRDetector.StabilizedPose;
        Pose world = RobotFrameMath.Compose(marker, _draftCalibration);
        world.position += worldDelta;
        if (!RobotFrameMath.IsValidPose(world)) { SetAlignmentStatus("The requested position adjustment is invalid."); return; }
        Pose proposed = RobotFrameMath.RelativeTo(marker, world);
        if (!RobotFrameMath.IsValidPose(proposed)) { SetAlignmentStatus("The requested position adjustment is invalid."); return; }
        _draftCalibration = proposed;
        ApplyDraftPose();
        SetAlignmentStatus("Unsaved alignment. Match the base and two separated links, then Save.");
    }

    public void RotateBase(Vector3 worldAxis, float degrees)
    {
        if (!RequireEditingContext()) return;
        if (!RobotFrameMath.IsValidPose(new Pose(worldAxis, Quaternion.identity)) || worldAxis.sqrMagnitude < 0.000001f ||
            float.IsNaN(degrees) || float.IsInfinity(degrees))
        { SetAlignmentStatus("The requested rotation adjustment is invalid."); return; }
        Pose marker = _QRDetector.StabilizedPose;
        Pose world = RobotFrameMath.Compose(marker, _draftCalibration);
        world.rotation = Quaternion.AngleAxis(degrees, worldAxis.normalized) * world.rotation;
        Pose proposed = RobotFrameMath.RelativeTo(marker, world);
        if (!RobotFrameMath.IsValidPose(proposed)) { SetAlignmentStatus("The requested rotation adjustment is invalid."); return; }
        _draftCalibration = proposed;
        ApplyDraftPose();
        SetAlignmentStatus("Unsaved rotation about the robot base. Check several links, then Save.");
    }

    public void RestoreDefaultCalibration()
    {
        if (!RequireEditingContext()) return;
        if (!TryGetDefaultCalibration(out Pose fallback))
        { SetAlignmentStatus("The configured default alignment is invalid."); return; }
        _draftCalibration = fallback;
        ApplyDraftPose();
        SetAlignmentStatus("Default alignment previewed. Save to keep it, or Cancel to undo.");
    }

    public void SaveCalibration()
    {
        if (!RequireEditingContext()) return;
        RobotCalibrationProfile profile = null;
        string error;
        bool saved;
        try
        {
            profile = RobotCalibrationProfile.Create(_calibrationProfileId, _editingPayload, _editingMarkerSize, _draftCalibration);
            saved = CalibrationStore().TrySave(profile, out error);
        }
        catch (Exception exception) when (exception is IOException || exception is UnauthorizedAccessException || exception is ArgumentException)
        { saved = false; error = exception.Message; }
        if (!saved)
        {
            SetAlignmentStatus("Save failed. Check device storage; alignment remains unsaved.");
            Debug.LogWarning("URInstantiator: " + error, this);
            return;
        }
        _savedCalibration = profile;
        _activeCalibration = _draftCalibration;
        _hasActiveCalibration = true;
        FinishEditing(_activeCalibration, true);
        SetAlignmentStatus("Alignment saved for this QR and restored on future scans.");
    }

    public void CancelCalibration()
    {
        if (!IsCalibrating) return;
        bool valid = EditingContextIsValid();
        FinishEditing(_calibrationBeforeEdit, valid);
        SetAlignmentStatus(valid ? "Alignment cancelled. Previous alignment restored." : "Alignment cancelled. Scan QR and confirm placement again.");
    }

    private void FinishEditing(Pose calibration, bool initialize)
    {
        Pose marker = initialize ? _QRDetector.StabilizedPose : RobotFrameMath.IsRigidFrame(_editingTrackingSpace)
            ? RobotFrameMath.Compose(new Pose(_editingTrackingSpace.position, _editingTrackingSpace.rotation), _markerInTrackingAtEdit)
            : default;
        RosRobotRig rig = _robotInstance != null ? _robotInstance.GetComponent<RosRobotRig>() : null;
        Pose world = RobotFrameMath.IsValidPose(marker) ? RobotFrameMath.Compose(marker, calibration) : _baseWorldBeforeEdit;
        if (rig != null && rig.BaseLink != null && RobotFrameMath.IsValidPose(world)) rig.SetBaseWorldPose(world);
        IsCalibrating = false;
        // Do not recreate singleton services during scene teardown or application shutdown.
        RobotCoordinateService coordinates = FindFirstObjectByType<RobotCoordinateService>();
        RobotStateService state = FindFirstObjectByType<RobotStateService>();
        if (coordinates != null) coordinates.EndCalibrationEdit();
        if (initialize && coordinates != null && state != null && rig != null && rig.BaseLink != null)
            state.InitializeRobot(marker, _robotInstance, _QRDetector.TrackingSpace);
        else _appliedValidationRevision = -1;
        _editingTrackingSpace = null;
    }

    private bool EditingContextIsValid() => IsCalibrating && CanCalibrate &&
        _editingTrackingSpace == _QRDetector.TrackingSpace &&
        _editingValidationRevision == _QRDetector.ValidationRevision && _editingPayload == _QRDetector.DetectedPayload &&
        Mathf.Abs(_editingMarkerSize - _QRDetector.MarkerSizeMeters) <= 0.000001f;

    private bool RequireEditingContext()
    {
        if (!IsCalibrating) { SetAlignmentStatus("Choose Align robot before adjusting or saving alignment."); return false; }
        if (EditingContextIsValid()) return true;
        CancelCalibration();
        return false;
    }

    private void FixedUpdate() { SynchronizeDraft(); }
    private void LateUpdate() { SynchronizeDraft(); }
    private void SynchronizeDraft()
    {
        if (!IsCalibrating) return;
        if (!RequireEditingContext()) return;
        ApplyDraftPose();
    }

    private void ApplyDraftPose()
    {
        RosRobotRig rig = _robotInstance.GetComponent<RosRobotRig>();
        Pose desired = RobotFrameMath.Compose(_QRDetector.StabilizedPose, _draftCalibration);
        if ((rig.BaseLink.position - desired.position).sqrMagnitude > 0.000000000001f ||
            Quaternion.Angle(rig.BaseLink.rotation, desired.rotation) > 0.001f)
            rig.SetBaseWorldPose(desired);
    }

    private void SetAlignmentStatus(string status)
    {
        AlignmentStatus = status;
        CalibrationStateChanged?.Invoke();
    }
}
