using RosMessageTypes.Ur16eInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class AddBoxToScene : MonoBehaviour
{
    [Header("Debug")]
    [SerializeField]
    private bool debug = false;

    [Header("ROS Message Configuration")]
    [Tooltip("Must exactly match the ROS2-side message name.")]
    public string MessageName = "scene_setup_box";

    [SerializeField]
    [Tooltip("The color to apply to the cube in the Unity Scene once it has been sent to the ROS Scene.")]
    private Color sentColor;
    
    private bool sent = false;
    private ROSConnection ros;

    void Start()
    {
        // Get (or create) the ROSConnection singleton
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<BoxMsg>(MessageName);
    }

    public void CallSceneBoxTopic(GameObject box)
    {
        if (sent)
        {
            return;
        }
        
        // Transform that world pose into a local to base_link
        Pose boxPoseLocal = Transform.FindAnyObjectByType<RobotCoordinatesConverter>().ConvertWorldTransformToLocalPose(box.transform);

        // Unity's world coordinates -> ROS coordinates:
        // Here we assume Unity world == MoveIt base_link. If you have a TF offset, apply it.
        Vector3 pos;
        Quaternion rot;
        Vector3 scale;
        if (debug)
        {
            pos = new Vector3(1.0f, 0.0f, 0.0f);
            rot = Quaternion.identity;
            scale = box.transform.localScale;
        }
        else
        {
            pos = boxPoseLocal.position;
            rot = boxPoseLocal.rotation;
            scale = box.transform.localScale;
        }

        PointMsg rosPos = pos.To<FLU>();
        QuaternionMsg rosRot = rot.To<FLU>();
        Debug.Log($"Requesting to create box at (Unity Coordinates) {pos.x}, {pos.y}, {pos.z}");
        Debug.Log($"Requesting to create box at (ROS Coordinates) {rosPos.x}, {rosPos.y}, {rosPos.z}");


        HeaderMsg header = new HeaderMsg();
        header.frame_id = "ur_base_link";
        PoseMsg pose = new PoseMsg(rosPos, rosRot);
        PoseStampedMsg poseStamped = new PoseStampedMsg(header, pose);
        BoxMsg boxMsg = new BoxMsg(scale.z, scale.x, scale.y, poseStamped);

        ros.Publish(MessageName, boxMsg);
        Debug.Log("Box message sent");

        box.GetComponent<MeshRenderer>().material.color = sentColor;
        sent = true;

        GameObject primitiveRotationMenu = GameObject.FindAnyObjectByType<AddBoxToScene>().transform.parent.gameObject;
        primitiveRotationMenu.SetActive(false);
    }
}
