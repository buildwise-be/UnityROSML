using UnityEngine;

/// <summary>
/// Singleton service for converting coordinates between Unity world space and robot base link frame.
/// Consolidates coordinate conversion logic from RobotCoordinatesConverter and CoordinatesConverter.
/// </summary>
public class RobotCoordinateService : MonoBehaviour
{
    private static RobotCoordinateService _instance;
    public static RobotCoordinateService Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindFirstObjectByType<RobotCoordinateService>();
                if (_instance == null)
                {
                    GameObject go = new GameObject("RobotCoordinateService");
                    _instance = go.AddComponent<RobotCoordinateService>();
                    DontDestroyOnLoad(go);
                }
            }
            return _instance;
        }
    }

    private ArticulationBody _robotRootArticulation = null;
    private Transform _calibrationFrame;
    private Pose _baseInCalibrationFrame;
    private bool _isInitialized = false;
    public int CalibrationRevision { get; private set; }
    public bool IsCalibrationInProgress { get; private set; }
    // Inspect an existing service without creating a singleton during UI/offline checks.
    public static bool IsCalibrationEditActive
    {
        get
        {
            RobotCoordinateService service = _instance != null ? _instance : FindFirstObjectByType<RobotCoordinateService>();
            return service != null && service.IsCalibrationInProgress;
        }
    }
    public event System.Action CalibrationChanged;
    public bool IsInitialized => !IsCalibrationInProgress && _isInitialized && RobotFrameMath.IsRigidFrame(
        _robotRootArticulation != null ? _robotRootArticulation.transform : null) &&
        (_calibrationFrame == null || RobotFrameMath.IsRigidFrame(_calibrationFrame));

    [SerializeField, Tooltip("ROS frame represented by the imported robot's base_link. This is distinct from UR's teach-pendant base frame.")]
    private string _rosBaseFrameId = "ur_base_link";
    public string RosBaseFrameId => _rosBaseFrameId;

    private void Awake()
    {
        if (_instance == null)
        {
            _instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else if (_instance != this)
        {
            Destroy(gameObject);
        }
    }

    /// <summary>
    /// Initialize the service with the robot's root articulation body.
    /// Called when robot is instantiated and placed.
    /// </summary>
    public void Initialize(ArticulationBody robotRootArticulation)
    {
        Initialize(robotRootArticulation, robotRootArticulation != null ? robotRootArticulation.transform.parent : null);
    }

    public void Initialize(ArticulationBody robotRootArticulation, Transform trackingSpace)
    {
        _robotRootArticulation = robotRootArticulation;
        _calibrationFrame = trackingSpace;
        _isInitialized = !IsCalibrationInProgress && robotRootArticulation != null &&
            RobotFrameMath.IsRigidFrame(robotRootArticulation.transform) &&
            (trackingSpace == null || RobotFrameMath.IsRigidFrame(trackingSpace));
        if (!_isInitialized)
        {
            NotifyCalibrationChanged();
            if (IsCalibrationInProgress) return;
            Debug.LogError("RobotCoordinateService: A valid robot base at metre scale is required.");
            return;
        }
        Pose worldBase = new Pose(robotRootArticulation.transform.position, robotRootArticulation.transform.rotation);
        _baseInCalibrationFrame = trackingSpace != null
            ? RobotFrameMath.RelativeTo(new Pose(trackingSpace.position, trackingSpace.rotation), worldBase)
            : worldBase;
        NotifyCalibrationChanged();
    }

    /// <summary>Block all coordinate-dependent commands while a new QR alignment is pending.</summary>
    public void InvalidateCalibration()
    {
        _isInitialized = false;
        NotifyCalibrationChanged();
    }

    /// <summary>Release the authoritative root while editing the virtual QR-to-base extrinsic.</summary>
    public void BeginCalibrationEdit()
    {
        IsCalibrationInProgress = true;
        InvalidateCalibration();
    }

    /// <summary>Ending an edit never rearms commands; Initialize must confirm the final pose.</summary>
    public void EndCalibrationEdit()
    {
        if (!IsCalibrationInProgress) return;
        IsCalibrationInProgress = false;
        InvalidateCalibration();
    }

    private void NotifyCalibrationChanged()
    {
        unchecked { CalibrationRevision++; }
        CalibrationChanged?.Invoke();
    }

    public bool TryGetRobotBaseWorldPose(out Pose worldBase)
    {
        worldBase = default;
        if (!IsInitialized) return false;
        worldBase = _calibrationFrame != null
            ? RobotFrameMath.Compose(new Pose(_calibrationFrame.position, _calibrationFrame.rotation), _baseInCalibrationFrame)
            : _baseInCalibrationFrame;
        return RobotFrameMath.IsValidPose(worldBase);
    }

    // PhysX can retain a root body's old world pose when its Transform parent moves.
    // The QR calibration is authoritative; a physics update must never redefine it.
    private void FixedUpdate() { SynchronizeRobotRoot(); }
    private void LateUpdate() { SynchronizeRobotRoot(); }

    private void SynchronizeRobotRoot()
    {
        if (!TryGetRobotBaseWorldPose(out Pose desired) || !_robotRootArticulation.isRoot) return;
        Transform body = _robotRootArticulation.transform;
        if ((body.position - desired.position).sqrMagnitude > 1e-10f ||
            Quaternion.Angle(body.rotation, desired.rotation) > 0.001f)
            _robotRootArticulation.TeleportRoot(desired.position, desired.rotation);
    }

    /// <summary>
    /// Transforms a world pose to a pose relative to the robot base link.
    /// </summary>
    /// <param name="worldTransform">The world transform to convert</param>
    /// <returns>The local pose relative to robot base link</returns>
    public Pose ConvertWorldTransformToLocalPose(Transform worldTransform)
    {
        return ConvertWorldPoseToLocalPose(new Pose(worldTransform.position, worldTransform.rotation));
    }

    /// <summary>
    /// Transforms a world pose to a pose relative to the robot base link.
    /// </summary>
    /// <param name="worldPose">The world pose to convert</param>
    /// <returns>The local pose relative to robot base link</returns>
    public Pose ConvertWorldPoseToLocalPose(Pose worldPose)
    {
        if (!TryConvertWorldPoseToLocalPose(worldPose, out Pose localPose))
            throw new System.InvalidOperationException("Robot base calibration or target pose is invalid.");
        return localPose;
    }

    public bool TryConvertWorldPoseToLocalPose(Pose worldPose, out Pose localPose)
    {
        localPose = default;
        if (!TryGetRobotBaseWorldPose(out Pose worldBase) || !RobotFrameMath.IsValidPose(worldPose) || string.IsNullOrWhiteSpace(_rosBaseFrameId))
            return false;
        localPose = RobotFrameMath.RelativeTo(worldBase, worldPose);
        return true;
    }

    public Pose ConvertRootLocalPoseToWorldPose(Pose localPose)
    {
        if (!TryGetRobotBaseWorldPose(out Pose worldBase) || !RobotFrameMath.IsValidPose(localPose))
            throw new System.InvalidOperationException("Robot base calibration or target pose is invalid.");
        return RobotFrameMath.Compose(worldBase, localPose);
    }
}

