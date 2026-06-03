using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Ur16eInterfaces;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using System;

/// <summary>
/// Refactored from PickAndPlaceManager.
/// Focuses on pick/place task orchestration using services and tile system.
/// </summary>
public class PickPlaceController : MonoBehaviour
{
    [SerializeField]
    [Tooltip("Reference to TileManager for accessing tiles")]
    private TileManager _tileManager;

    [SerializeField]
    [Tooltip("Reference to FaceSelectionController for face selection")]
    private FaceSelectionController _faceSelectionController;

    [SerializeField]
    [Tooltip("ROS topic name for pick/place poses")]
    private string _pickPlaceTopicName = "pick_place_poses_target";

    private bool _hasPlanningRequestBeenSent = false;

    private void Start()
    {
        // Register ROS publishers via ROSService
        ROSService.Instance.RegisterPublisher<PickPlacePosesMsg>(_pickPlaceTopicName);
        ROSService.Instance.RegisterPublisher<BoolMsg>("execute_task");
        ROSService.Instance.RegisterPublisher<BoxMsg>("scene_setup_box");
    }

    /// <summary>
    /// Send pick and place planning request to ROS.
    /// Gets tiles from TileManager and extracts poses using selected faces.
    /// </summary>
    public void SendPickAndPlacePlanningRequest()
    {
        if (_tileManager == null)
        {
            Debug.LogError("PickPlaceController: TileManager not assigned!");
            return;
        }

        Tile pickTile = _tileManager.GetReadyPickTile();
        Tile placeTile = _tileManager.GetReadyPlaceTile();

        if (pickTile == null)
        {
            Debug.LogError("PickPlaceController: No ready pick tile found!");
            return;
        }

        if (placeTile == null)
        {
            Debug.LogError("PickPlaceController: No ready place tile found!");
            return;
        }

        // Get poses from tiles
        Pose pickPoseWorld = pickTile.GetPoseForFace(true);
        Pose placePoseWorld = placeTile.GetPoseForFace(false);

        // Convert to robot-local poses
        Pose pickPoseLocal = RobotCoordinateService.Instance.ConvertWorldPoseToLocalPose(pickPoseWorld);
        Pose placePoseLocal = RobotCoordinateService.Instance.ConvertWorldPoseToLocalPose(placePoseWorld);

        // Generate ROS messages
        PoseStampedMsg pickPose = GeneratePoseStamped(pickPoseLocal);
        PoseStampedMsg placePose = GeneratePoseStamped(placePoseLocal);

        // Add box to MoveIt scene (using pick pose for obstacle)
        AddBoxToMoveitScene(pickPose, 0.1f, 0.1f, 0.004f);

        PickPlacePosesMsg msg = new PickPlacePosesMsg(pickPose, placePose);

        // Debug log before sending
        Debug.Log($"PickPlaceController: Sending PickPlacePosesMsg:\n" +
            $"Pick Pose - Header: frame_id={pickPose.header.frame_id}, stamp={pickPose.header.stamp}, " +
            $"Position: ({pickPose.pose.position.x:F4}, {pickPose.pose.position.y:F4}, {pickPose.pose.position.z:F4}), " +
            $"Orientation: ({pickPose.pose.orientation.x:F4}, {pickPose.pose.orientation.y:F4}, {pickPose.pose.orientation.z:F4}, {pickPose.pose.orientation.w:F4})\n" +
            $"Place Pose - Header: frame_id={placePose.header.frame_id}, stamp={placePose.header.stamp}, " +
            $"Position: ({placePose.pose.position.x:F4}, {placePose.pose.position.y:F4}, {placePose.pose.position.z:F4}), " +
            $"Orientation: ({placePose.pose.orientation.x:F4}, {placePose.pose.orientation.y:F4}, {placePose.pose.orientation.z:F4}, {placePose.pose.orientation.w:F4})");

        // Send via ROSService
        ROSService.Instance.Publish(_pickPlaceTopicName, msg);
        _hasPlanningRequestBeenSent = true;
    }

    /// <summary>
    /// Generate a PoseStampedMsg from a local pose.
    /// </summary>
    private PoseStampedMsg GeneratePoseStamped(Pose localPose)
    {
        // Convert Unity coordinates to ROS FLU coordinates
        Vector3 pos = localPose.position;
        Quaternion rot = localPose.rotation;

        PointMsg rosPos = pos.To<FLU>();
        QuaternionMsg rosRot = rot.To<FLU>();

        HeaderMsg header = new HeaderMsg();
        header.frame_id = "ur_base_link";
        
        // Set the timestamp to current ROS time (Unix epoch)
        DateTime epochStart = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        TimeSpan timeSinceEpoch = DateTime.UtcNow - epochStart;
        double totalSeconds = timeSinceEpoch.TotalSeconds;
        int sec = (int)totalSeconds;
        uint nanosec = (uint)((totalSeconds - sec) * 1e9);
        header.stamp = new TimeMsg(sec, nanosec);
        
        PoseMsg pose = new PoseMsg(rosPos, rosRot);
        PoseStampedMsg poseStamped = new PoseStampedMsg(header, pose);

        return poseStamped;
    }

    /// <summary>
    /// Executes the pick and place task by sending a Bool message to the execute_task topic.
    /// This method can only be called after SendPickAndPlacePlanningRequest has been called.
    /// </summary>
    public void ExecutePickAndPlace()
    {
        if (!_hasPlanningRequestBeenSent)
        {
            Debug.LogWarning("PickPlaceController: Cannot execute task. SendPickAndPlacePlanningRequest must be called first.");
            return;
        }

        BoolMsg executeMsg = new BoolMsg(true);
        ROSService.Instance.Publish("execute_task", executeMsg);
        Debug.Log("PickPlaceController: Sent execute_task message with value: true");
    }

    /// <summary>
    /// Adds a box to the MoveIt scene by creating a BoxMsg from the provided pose and dimensions,
    /// then publishing it to the scene_setup_box topic.
    /// </summary>
    /// <param name="pose">The PoseStampedMsg representing the position and orientation of the box</param>
    /// <param name="dimension_x">The x dimension of the box</param>
    /// <param name="dimension_y">The y dimension of the box</param>
    /// <param name="dimension_z">The z dimension of the box</param>
    public void AddBoxToMoveitScene(PoseStampedMsg pose, float dimension_x, float dimension_y, float dimension_z)
    {
        BoxMsg boxMsg = new BoxMsg(dimension_x, dimension_y, dimension_z, pose);
        ROSService.Instance.Publish("scene_setup_box", boxMsg);
        Debug.Log($"PickPlaceController: Published box with dimensions ({dimension_x}, {dimension_y}, {dimension_z}) to scene_setup_box topic");
    }
}

