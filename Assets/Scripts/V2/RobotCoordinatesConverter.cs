using UnityEngine;
using UnityEngine.Events;

public class RobotCoordinatesConverter : MonoBehaviour
{
    private GameObject _ur16eInstance = null;
    private ArticulationBody _ur16eRootArticulation = null;
    [HideInInspector]
    public UnityEvent OnRobotPositionRecorded;

    private void Start()
    {
        OnRobotPositionRecorded = new UnityEvent();
        MarkerUnderstander mu = FindAnyObjectByType<MarkerUnderstander>();
        if (mu != null)
        {
            mu.OnUR16eInstantiatedAndPlaced.AddListener(RecordURInstancePosition); 
        }
    }

    private void RecordURInstancePosition(Pose arg0, GameObject arg1)
    {
        _ur16eInstance = arg1;
        foreach (ArticulationBody articulationBody in _ur16eInstance.GetComponentsInChildren<ArticulationBody>())
        {
            if (articulationBody.gameObject.name == "base_link")
            {
                _ur16eRootArticulation = articulationBody;
                break;
            }
        }
        OnRobotPositionRecorded.Invoke();
    }

    /// <summary>
    /// Transforms a world pose to a pose relative to the UR16e (ur_)base_link
    /// </summary>
    /// <param name="worldTransform">The world transform to be.. transformed!</param>
    /// <returns>The local transform</returns>
    public Pose ConvertWorldTransformToLocalPose(Transform worldTransform)
    {
        if (_ur16eRootArticulation == null)
        {
            Debug.LogError("No root articulation body found!");
            return new Pose();
        }
        Vector3 rootRelativePos = _ur16eRootArticulation.transform.InverseTransformPoint(worldTransform.position);
        Quaternion rootRelativeRot = Quaternion.Inverse(_ur16eRootArticulation.transform.rotation) * (worldTransform.rotation);
        Pose pose = new Pose();
        pose.position = rootRelativePos;
        pose.rotation = rootRelativeRot;
        return pose;
    }

    public Vector3 GetWorldAlignedDimensions(Transform primitiveObstacle)
    {
        if (primitiveObstacle == null)
        {
            Debug.LogError("Primitive obstacle transform is null!");
            return Vector3.zero;
        }
        
        var meshFilter = primitiveObstacle.GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.mesh == null)
        {
            Debug.LogError("MeshFilter or mesh not found on primitive obstacle!");
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
            // Transform vertex to world space
            Vector3 worldVertex = localToWorld.MultiplyPoint3x4(vertex);

            // Undo rotation to align with world axes
            Vector3 alignedVertex = inverseRotation * worldVertex;

            min = Vector3.Min(min, alignedVertex);
            max = Vector3.Max(max, alignedVertex);
        }

        Vector3 worldAlignedSize = max - min;
        return worldAlignedSize;
    }
}