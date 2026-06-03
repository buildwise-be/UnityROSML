using UnityEngine;
using UnityEngine.Events;

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

    [HideInInspector]
    public UnityEvent OnRobotPositionRecorded;

    private void Awake()
    {
        if (_instance == null)
        {
            _instance = this;
            DontDestroyOnLoad(gameObject);
            OnRobotPositionRecorded = new UnityEvent();
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
        OnRobotPositionRecorded?.Invoke();
        Debug.Log("RobotCoordinateService: Initialized with robot base link");
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

    /// <summary>
    /// Gets world-aligned dimensions of a primitive obstacle.
    /// </summary>
    public Vector3 GetWorldAlignedDimensions(Transform primitiveObstacle)
    {
        if (primitiveObstacle == null)
        {
            Debug.LogError("RobotCoordinateService: Primitive obstacle transform is null!");
            return Vector3.zero;
        }
        
        var meshFilter = primitiveObstacle.GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.mesh == null)
        {
            Debug.LogError("RobotCoordinateService: MeshFilter or mesh not found on primitive obstacle!");
            return Vector3.zero;
        }
        
        Vector3[] vertices = meshFilter.mesh.vertices;
        Matrix4x4 localToWorld = primitiveObstacle.localToWorldMatrix;

        Quaternion rotation = primitiveObstacle.rotation;
        Quaternion inverseRotation = Quaternion.Inverse(rotation);

        Vector3 min = Vector3.positiveInfinity;
        Vector3 max = Vector3.negativeInfinity;

        foreach (Vector3 vertex in vertices)
        {
            Vector3 worldVertex = localToWorld.MultiplyPoint3x4(vertex);
            Vector3 alignedVertex = inverseRotation * worldVertex;

            min = Vector3.Min(min, alignedVertex);
            max = Vector3.Max(max, alignedVertex);
        }

        return max - min;
    }

    public bool IsInitialized => _isInitialized;
}

