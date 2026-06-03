using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Ur16eUnityInterfaces;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using System;

public class PlanningSceneUpdater : MonoBehaviour
{
    ROSConnection ros;
    const string ServiceName = "add_primitive_object";
    private string session_id = string.Empty;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<AddPrimitiveRequest, AddPrimitiveResponse>(ServiceName);
        session_id = Guid.NewGuid().ToString();
    }

    /// <summary>
    /// Add/remove a box by passing in the object's Pose in local UR16e coordinate system.
    /// </summary>
    public void UpdateBox(Pose URLocalPose, Vector3 dimensions, int id, bool remove = false)
    {
        var pos = URLocalPose.position.To<FLU>();
        var rot = URLocalPose.rotation.To<FLU>();
        var dim = new Vector3(dimensions.z, dimensions.x, dimensions.y);

        PoseMsg poseMsg = new PoseMsg(pos, rot);

        var req = new AddPrimitiveRequest
        {
            session_id = session_id,
            type = "BOX",
            remove = remove,
            pose = poseMsg,
            dimensions = new Vector3Msg(dim.x, dim.y, dim.z),
            id = id.ToString()
        };
        ros.SendServiceMessage<AddPrimitiveResponse>(ServiceName, req, ResponseCallback);
    }

    /// <summary>
    /// Add/remove a plane (as a thin box) via its Transform.
    /// </summary>
    public void UpdatePlane(Pose URLocalPose, Vector3 dimensions, int id, bool remove = false)
    {
        var pos = URLocalPose.position.To<FLU>();
        var rot = URLocalPose.rotation.To<FLU>();
        var dim = new Vector3(dimensions.z, dimensions.x, dimensions.y);

        PoseMsg poseMsg = new PoseMsg(pos, rot);

        var req = new AddPrimitiveRequest
        {
            session_id = session_id,
            type = "PLANE",
            remove = remove,
            pose = poseMsg,
            dimensions = new Vector3Msg(dim.x, dim.y, 0f),
            id = id.ToString()
        };

        ros.SendServiceMessage<AddPrimitiveResponse>(ServiceName, req, ResponseCallback);
    }
    
    void ResponseCallback(AddPrimitiveResponse resp)
    {
        if (resp.success)
        {
            Debug.Log($"[PlanningSceneUpdater] Success: {resp.message}");
        }
        else
            Debug.LogError($"[PlanningSceneUpdater] Failed: {resp.message}");
    }
    /*
    public List<GameObject> meshObstacles;
    private ROSConnection ros;

    public string serviceName = "update_planning_scene";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterRosService<UpdatePlanningSceneRequest, UpdatePlanningSceneResponse>(serviceName);
    }

    void SendMeshToScene(GameObject obj)
    {
        var mesh = obj.GetComponent<MeshFilter>().mesh;
        var pose = obj.transform.To<FLU>();

        // Build MeshMsg
        List<MeshTriangleMsg> triangles = new List<MeshTriangleMsg>();
        for (int i = 0; i < mesh.triangles.Length; i += 3)
        {
            triangles.Add(new MeshTriangleMsg(new uint[] {
                (uint)mesh.triangles[i],
                (uint)mesh.triangles[i + 1],
                (uint)mesh.triangles[i + 2]
            }));
        }


        foreach (GameObject obstacle in meshObstacles)
        {
            var meshes = obstacle.GetComponentsInChildren<MeshFilter>();
            foreach (MeshFilter mesh in meshes)
            {
                Mesh m = mesh.mesh;

                var allMeshTriangleMsgs = new List<MeshTriangleMsg>();
                var allMeshVerticesMsgs = new List<PointMsg>();
                for (int i = 0; i < m.triangles.Length; i+=3)
                {
                    int index1 = m.triangles[i];
                    int index2 = m.triangles[i+1];
                    int index3 = m.triangles[i+2];
                    UInt32[] vertex_indices = { (uint)index1, (uint)index2, (uint)index3 };
                    var meshTrianglesMsg = new MeshTriangleMsg(vertex_indices);
                    allMeshTriangleMsgs.Add(meshTrianglesMsg);

                    var p1 = new PointMsg(m.vertices[index1].x, m.vertices[index1].y, m.vertices[index1].z);
                    var p2 = new PointMsg(m.vertices[index2].x, m.vertices[index2].y, m.vertices[index2].z);
                    var p3 = new PointMsg(m.vertices[index3].x, m.vertices[index3].y, m.vertices[index3].z);
                    allMeshVerticesMsgs.Add(p1);
                    allMeshVerticesMsgs.Add(p2);
                    allMeshVerticesMsgs.Add(p3);
                }
                MeshMsg meshMsg = new MeshMsg(allMeshTriangleMsgs.ToArray(), allMeshVerticesMsgs.ToArray());
                ros.Publish(MeshUpdateSceneTopicName, meshMsg);
            }

        }
    }
    */
}

public enum PrimitiveObstacleTypes
{
    Box,
    Plane
}