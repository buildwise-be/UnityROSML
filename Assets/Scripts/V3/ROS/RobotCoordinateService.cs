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
    private bool _isInitialized = false;
    public bool IsInitialized => _isInitialized;

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
        _robotRootArticulation = robotRootArticulation;
        _isInitialized = true;
    }

    /// <summary>
    /// Transforms a world pose to a pose relative to the robot base link.
    /// </summary>
    /// <param name="worldTransform">The world transform to convert</param>
    /// <returns>The local pose relative to robot base link</returns>
    public Pose ConvertWorldTransformToLocalPose(Transform worldTransform)
    {
        if (!_isInitialized || _robotRootArticulation == null)
        {
            Debug.LogError("RobotCoordinateService: Not initialized! Robot base link not found.");
            return new Pose();
        }

        Vector3 rootRelativePos = _robotRootArticulation.transform.InverseTransformPoint(worldTransform.position);
        Quaternion rootRelativeRot = Quaternion.Inverse(_robotRootArticulation.transform.rotation) * worldTransform.rotation;
        
        return new Pose(rootRelativePos, rootRelativeRot);
    }

    /// <summary>
    /// Transforms a world pose to a pose relative to the robot base link.
    /// </summary>
    /// <param name="worldPose">The world pose to convert</param>
    /// <returns>The local pose relative to robot base link</returns>
    public Pose ConvertWorldPoseToLocalPose(Pose worldPose)
    {
        if (!_isInitialized || _robotRootArticulation == null)
        {
            Debug.LogError("RobotCoordinateService: Not initialized! Robot base link not found.");
            return new Pose();
        }

        Vector3 rootRelativePos = _robotRootArticulation.transform.InverseTransformPoint(worldPose.position);
        Quaternion rootRelativeRot = Quaternion.Inverse(_robotRootArticulation.transform.rotation) * worldPose.rotation;
        
        return new Pose(rootRelativePos, rootRelativeRot);
    }

    public Pose ConvertRootLocalPoseToWorldPose(Pose localPose)
    {
        if (!_isInitialized || _robotRootArticulation == null)
        {
            Debug.LogError("RobotCoordinateService: Not initialized! Robot base link not found.");
            return new Pose();
        }
        Vector3 worldPosition = _robotRootArticulation.transform.TransformPoint(localPose.position);
        Quaternion worldQuaternion = _robotRootArticulation.transform.rotation * localPose.rotation;

        return new Pose(worldPosition, worldQuaternion);
    }
}

