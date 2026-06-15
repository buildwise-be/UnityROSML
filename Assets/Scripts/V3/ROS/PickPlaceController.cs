using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Ur16eInterfaces;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using System;
using Unity.XR.CoreUtils;

/// <summary>
/// Refactored from PickAndPlaceManager.
/// Focuses on pick/place task orchestration using services and tile system.
/// </summary>
public class PickPlaceController : MonoBehaviour
{
    [SerializeField]
    [Tooltip("ROS topic name for pick/place poses")]
    private string _pickPlaceTopicName = "pick_place_poses_target";

    [SerializeField]
    private GameObject _pickPointPrefab;
    private GameObject _pickPointInstance;

    [SerializeField]
    private GameObject _placePointPrefab;
    private GameObject _placePointInstance;

    private bool _hasPlanningRequestBeenSent = false;

    private void Start()
    {
        // Register ROS publishers via ROSService
        ROSService.Instance.RegisterPublisher<PickPlacePosesMsg>(_pickPlaceTopicName);
        ROSService.Instance.RegisterPublisher<BoolMsg>("execute_task");
    }

    private void UpdatePlanningReadiness()
    {
        if (_pickPointInstance != null && _placePointInstance != null)
        {
            RobotStateService.Instance.IsReadyForPlanning = true;
        }
        else
        {
            RobotStateService.Instance.IsReadyForPlanning = false;
        }
    }

    public void SetPickPlace(Vector3 position, Quaternion rotation)
    {
        if (_pickPointInstance == null)
        {
            _pickPointInstance = Instantiate(_pickPointPrefab, position, rotation);
        }
        else
        {
            _pickPointInstance.transform.position = position;
            _pickPointInstance.transform.rotation = rotation;
        }
        UpdatePlanningReadiness();
    }

    public void SetPlacePlace(Vector3 position, Quaternion rotation)
    {
        if (_placePointInstance == null)
        {
            _placePointInstance = Instantiate(_placePointPrefab, position, rotation);
        }
        else
        {
            _placePointInstance.transform.position = position;
            _placePointInstance.transform.rotation = rotation;
        }
        UpdatePlanningReadiness();
    }

    /// <summary>
    /// Send pick and place planning request to ROS.
    /// Gets tiles from TileManager and extracts poses using selected faces.
    /// </summary>
    public void SendPickAndPlacePlanningRequest()
    {
        Debug.Log("SendPickAndPlacePlanningRequest called");
        if (RobotStateService.Instance.IsReadyForPlanning)
        {
            Debug.Log("Robot is ready, now sending the request");
            // Get poses from tiles
            Pose pickPoseWorld = _pickPointInstance.transform.GetWorldPose();
            Pose placePoseWorld = _placePointInstance.transform.GetWorldPose();

            // Convert to robot-local poses
            Pose pickPoseLocal = RobotCoordinateService.Instance.ConvertWorldPoseToLocalPose(pickPoseWorld);
            Pose placePoseLocal = RobotCoordinateService.Instance.ConvertWorldPoseToLocalPose(placePoseWorld);

            // Generate ROS messages
            string header = "ur_base_link";
            PoseStampedMsg pickPose = GeneratePoseStamped(pickPoseLocal, header);
            PoseStampedMsg placePose = GeneratePoseStamped(placePoseLocal, header);

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
    }

    public void SendFakePickAndPlacePlanningRequest()
    {
        Debug.Log("SendFakePickAndPlacePlanningRequest called");

        // Convert to robot-local poses
        // These hardcoded coordinates are directly in ROS (FLU) coordinates

        // mir_base_link coordinates:
        /*
        Vector3 pickPostion = new Vector3(1.0f, 0.0f, 0.4f);
        Quaternion pickRotation = new Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
        Vector3 placePostion = new Vector3(0.5f, 0.4f, 0.4f);
        Quaternion placeRotation = new Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
        */

        // ur_base_link coordinates:
        Vector3 pickPostionFLU = new Vector3(0.0f, 0.754485f, 0.098f);
        Quaternion pickRotationFLU = new Quaternion(0.707107f, 0.707107f, 0.0f, 0.0f);
        Vector3 placePostionFLU = new Vector3(-0.4f, 0.254485f, 0.098f);
        Quaternion placeRotatioFLU = new Quaternion(0.707107f, 0.707107f, 0.0f, 0.0f);

        // To place these in Unity, we need them in RUF
        // For that we need PointMsg and QuaternionMsg
        PointMsg PickPointMsgFLU = new PointMsg(pickPostionFLU.x, pickPostionFLU.y, pickPostionFLU.z);
        QuaternionMsg PickQuaternionMsgFLU = new QuaternionMsg(pickRotationFLU.x, pickRotationFLU.y, pickRotationFLU.z, pickRotationFLU.w);
        PointMsg PlacePointMsgFLU = new PointMsg(placePostionFLU.x, placePostionFLU.y, placePostionFLU.z);
        QuaternionMsg PlaceQuaternionMsgFLU = new QuaternionMsg(placeRotatioFLU.x, placeRotatioFLU.y, placeRotatioFLU.z, placeRotatioFLU.w);

        // Then convert in RUF
        Vector3 PickPositionRUF = PickPointMsgFLU.From<FLU>();
        Quaternion PickRotationRUF = PickQuaternionMsgFLU.From<FLU>();
        Vector3 PlacePositionRUF = PlacePointMsgFLU.From<FLU>();
        Quaternion PlaceRotationRUF = PlaceQuaternionMsgFLU.From<FLU>();

        // Make local Poses out of that
        Pose pickPoseLocal = new Pose(PickPositionRUF, PickRotationRUF);
        Pose placePoseLocal = new Pose(PlacePositionRUF, PlaceRotationRUF);

        // And then convert to world poses
        Pose worldPickPosition = RobotCoordinateService.Instance.ConvertRootLocalPoseToWorldPose(pickPoseLocal);
        Pose worldPlacePosition = RobotCoordinateService.Instance.ConvertRootLocalPoseToWorldPose(placePoseLocal);
        SetPickPlace(worldPickPosition.position, worldPickPosition.rotation);
        SetPlacePlace(worldPlacePosition.position, worldPlacePosition.rotation);

        // Generate ROS messages
        string header = "ur_base_link";
        PoseStampedMsg pickPose = GenerateFakePoseStamped(new Pose(pickPostionFLU, pickRotationFLU), header);
        PoseStampedMsg placePose = GenerateFakePoseStamped(new Pose(placePostionFLU, placeRotatioFLU), header);

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
    /// Fake (for debug) Pose Stamped generation.
    /// This is meant to be used with ROS-type (FLU) coordinates."
    /// </summary>
    /// <param name="localPose">The local to "ur_base_link", ROS-type (FLU) coordinates.</param>
    /// <param name="header">The ROS link name, defining the coordinate system in which the local pose is defined.</param>
    /// <returns></returns>
    private PoseStampedMsg GenerateFakePoseStamped(Pose localPose, string header)
    {
        // No need to convert between Unity and ROS conventions, since fake pose is already in ROS coordinates, local to (ur_)base_link
        PointMsg rosPos = new PointMsg(localPose.position.x, localPose.position.y, localPose.position.z);
        QuaternionMsg rosRot = new QuaternionMsg(localPose.rotation.x, localPose.rotation.y, localPose.rotation.z, localPose.rotation.w);

        HeaderMsg headerMsg = new HeaderMsg();
        headerMsg.frame_id = header;

        // Set the timestamp to current ROS time (Unix epoch)
        DateTime epochStart = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        TimeSpan timeSinceEpoch = DateTime.UtcNow - epochStart;
        double totalSeconds = timeSinceEpoch.TotalSeconds;
        int sec = (int)totalSeconds;
        uint nanosec = (uint)((totalSeconds - sec) * 1e9);
        headerMsg.stamp = new TimeMsg(sec, nanosec);

        PoseMsg pose = new PoseMsg(rosPos, rosRot);
        PoseStampedMsg poseStamped = new PoseStampedMsg(headerMsg, pose);

        return poseStamped;
    }

    /// <summary>
    /// Generate a PoseStampedMsg from a local pose.
    /// </summary>
    private PoseStampedMsg GeneratePoseStamped(Pose localPose, string header)
    {
        // Convert Unity coordinates to ROS FLU coordinates
        Vector3 pos = localPose.position;
        Quaternion rot = localPose.rotation;

        PointMsg rosPos = pos.To<FLU>();
        QuaternionMsg rosRot = rot.To<FLU>();

        HeaderMsg headerMsg = new HeaderMsg();
        headerMsg.frame_id = header;
        
        // Set the timestamp to current ROS time (Unix epoch)
        DateTime epochStart = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        TimeSpan timeSinceEpoch = DateTime.UtcNow - epochStart;
        double totalSeconds = timeSinceEpoch.TotalSeconds;
        int sec = (int)totalSeconds;
        uint nanosec = (uint)((totalSeconds - sec) * 1e9);
        headerMsg.stamp = new TimeMsg(sec, nanosec);
        
        PoseMsg pose = new PoseMsg(rosPos, rosRot);
        PoseStampedMsg poseStamped = new PoseStampedMsg(headerMsg, pose);

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

}

