using UnityEngine;

/// <summary>
/// Singleton service for managing robot instance and state.
/// Provides robot pose/state information and tracks robot lifecycle.
/// </summary>
public class RobotStateService : MonoBehaviour
{
    private static RobotStateService _instance;
    public static RobotStateService Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindFirstObjectByType<RobotStateService>();
                if (_instance == null)
                {
                    GameObject go = new GameObject("RobotStateService");
                    _instance = go.AddComponent<RobotStateService>();
                    DontDestroyOnLoad(go);
                }
            }
            return _instance;
        }
    }

    private GameObject _robotInstance = null;
    private ArticulationBody _robotRootArticulation = null;
    private Pose _robotPose = new Pose();
    private bool _isInitialized = false;
    public bool IsInitialized => _isInitialized;
    private bool _isReadyForPlanning = false;
    public bool IsReadyForPlanning
    {
        get {  return _isReadyForPlanning; }
        set { _isReadyForPlanning = value; }
    }
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
    /// Initialize the service with the robot instance and pose.
    /// Called when robot is instantiated and placed via QR code detection.
    /// </summary>
    public void InitializeRobot(Pose robotPose, GameObject robotInstance)
    {
        _robotInstance = robotInstance;
        _robotPose = robotPose;

        // Find the base_link articulation body
        foreach (ArticulationBody articulationBody in _robotInstance.GetComponentsInChildren<ArticulationBody>())
        {
            if (articulationBody.gameObject.name == "base_link")
            {
                _robotRootArticulation = articulationBody;
                break;
            }
        }

        if (_robotRootArticulation == null)
        {
            Debug.LogError("RobotStateService: Could not find robot base_link!");
            return;
        }

        _isInitialized = true;

        // Initialize coordinate service
        RobotCoordinateService.Instance.Initialize(_robotRootArticulation);
    }

    /// <summary>
    /// Get the robot GameObject instance.
    /// </summary>
    public GameObject GetRobotInstance()
    {
        return _robotInstance;
    }

    /// <summary>
    /// Get the robot's root articulation body (base_link).
    /// </summary>
    public ArticulationBody GetRobotRootArticulation()
    {
        return _robotRootArticulation;
    }

    /// <summary>
    /// Get the robot's root's current pose.
    /// </summary>
    public Pose GetRobotRootPose()
    {
        if (_robotRootArticulation != null)
        {
            return new Pose(_robotRootArticulation.transform.position, _robotRootArticulation.transform.rotation);
        }
        return _robotPose;
    }
}

