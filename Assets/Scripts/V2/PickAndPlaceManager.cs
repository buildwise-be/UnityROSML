using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Ur16eInterfaces;
using RosMessageTypes.BuiltinInterfaces;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.UI;
using System;

public class PickAndPlaceManager : MonoBehaviour
{
    [SerializeField]
    [Tooltip("The prefab instantiated to set the pick and place locations.")]
    private GameObject _pickOrPlaceLocationPrefab;

    [SerializeField]
    [Tooltip("The material to assign to the instantiated location prefab for the pick type.")]
    private Material _pickMaterial;

    [SerializeField]
    [Tooltip("The material to assign to the instantiated location prefab for the place type.")]
    private Material _placeMaterial;

    [SerializeField]
    private RobotCoordinatesConverter _robotCoordinatesConverter;

    [SerializeField]
    private string _pickPlaceTopicName = "pick_place_poses_target";

    private GameObject[] _pickOrPlaceLocationInstances;

    private int _nbPrefabsInstantiated = 0;

    private ROSConnection _ros;

    private bool _hasPlanningRequestBeenSent = false;

    private void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<PickPlacePosesMsg>(_pickPlaceTopicName);
        _ros.RegisterPublisher<BoolMsg>("execute_task");
        _ros.RegisterPublisher<BoxMsg>("scene_setup_box");
        _pickOrPlaceLocationInstances = new GameObject[2];
    }

    /// <summary>
    /// A LocationType can be either "pick" or "place". When we define a location, the pick and place
    /// manager needs to know whether that location is meant to be the pick or place location.
    /// </summary>
    /// <param name="go">The GameObject to which the LocationType is to be attached to</param>
    /// <param name="type">Pick or Place</param>
    private void AssignLocationType(GameObject go, LocationType type)
    {
        go.GetComponent<PickOrPlaceLocationInstance>().LocationType = type;
        foreach (var item in _pickOrPlaceLocationInstances)
        {
            if (item == null || item == go)
            {
                continue;
            }
            else
            {
                var all = LocationType.Unset | LocationType.Place | LocationType.Pick;
                item.GetComponent<PickOrPlaceLocationInstance>().LocationType = all & ~(LocationType.Unset | type);
                if (type == LocationType.Pick)
                {
                    item.GetComponent<MeshRenderer>().material = _placeMaterial;
                    foreach (var text in item.GetComponentsInChildren<TextMeshProUGUI>(true))
                    {
                        if (text.name == "LocationTypeText")
                        {
                            text.text = "Place";
                            break;
                        }
                    }
                }
                else if (type == LocationType.Place)
                {
                    item.GetComponent<MeshRenderer>().material = _pickMaterial;
                    foreach (var text in item.GetComponentsInChildren<TextMeshProUGUI>(true))
                    {
                        if (text.name == "LocationTypeText")
                        {
                            text.text = "Pick";
                            break;
                        }
                    }
                }
                break;
            }
        }
    }

    /// <summary>
    /// This method is called when the user places, via a prefab, a location in the scene,
    /// that will hold the reference to the pick or place pose.
    /// </summary>
    public void CreatePickOrPlaceLocationInstance()
    {
        if (_nbPrefabsInstantiated == 2)
        {
            return;
        }
        var go = Instantiate(_pickOrPlaceLocationPrefab);
        go.transform.position = Camera.main.transform.position + 1.0f * Camera.main.transform.forward;

        foreach (Button b in go.GetComponentsInChildren<Button>(true))
        {
            if (b.name == "PickTypeButton")
            {
                b.onClick.AddListener(() => AssignLocationType(go, LocationType.Pick));
            }
            else if (b.name == "PlaceTypeButton")
            {
                b.onClick.AddListener(() => AssignLocationType(go, LocationType.Place));
            }
        }

        _pickOrPlaceLocationInstances[_nbPrefabsInstantiated] = go;
        _nbPrefabsInstantiated++;
    }

    public void SendPickAndPlacePlanningRequest()
    {
        if (_nbPrefabsInstantiated != 2) { return; }
        PoseStampedMsg pickPose = new PoseStampedMsg();
        PoseStampedMsg placePose = new PoseStampedMsg();
        foreach (var pose in _pickOrPlaceLocationInstances)
        {
            if (pose.GetComponent<PickOrPlaceLocationInstance>().LocationType == LocationType.Pick)
            {
                pickPose = GeneratePoseStamped(pose);
            }
            else if (pose.GetComponent<PickOrPlaceLocationInstance>().LocationType == LocationType.Place)
            {
                placePose = GeneratePoseStamped(pose);
            }
            else
            {
                Debug.LogError("Unset location type");
                return;
            }
        }
        AddBoxToMoveitScene(pickPose, 0.1f, 0.1f, 0.004f);

        PickPlacePosesMsg msg = new PickPlacePosesMsg(pickPose, placePose);

        // Debug log before sending
        Debug.Log($"Sending PickPlacePosesMsg:\n" +
            $"Pick Pose - Header: frame_id={pickPose.header.frame_id}, stamp={pickPose.header.stamp}, " +
            $"Position: ({pickPose.pose.position.x:F4}, {pickPose.pose.position.y:F4}, {pickPose.pose.position.z:F4}), " +
            $"Orientation: ({pickPose.pose.orientation.x:F4}, {pickPose.pose.orientation.y:F4}, {pickPose.pose.orientation.z:F4}, {pickPose.pose.orientation.w:F4})\n" +
            $"Place Pose - Header: frame_id={placePose.header.frame_id}, stamp={placePose.header.stamp}, " +
            $"Position: ({placePose.pose.position.x:F4}, {placePose.pose.position.y:F4}, {placePose.pose.position.z:F4}), " +
            $"Orientation: ({placePose.pose.orientation.x:F4}, {placePose.pose.orientation.y:F4}, {placePose.pose.orientation.z:F4}, {placePose.pose.orientation.w:F4})");

        //Now send it...
        _ros.Publish(_pickPlaceTopicName, msg);
        _hasPlanningRequestBeenSent = true;
    }

    private PoseStampedMsg GeneratePoseStamped(GameObject pickOrPlaceLocationInstance)
    {
        var worldTransform = pickOrPlaceLocationInstance.transform;
        var localPose = _robotCoordinatesConverter.ConvertWorldTransformToLocalPose(worldTransform);

        // Change Coordinate system
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
        /* // DEBUGGING
        if (!_hasPlanningRequestBeenSent)
        {
            Debug.LogWarning("ExecutePickAndPlace: Cannot execute task. SendPickAndPlacePlanningRequest must be called first.");
            return;
        }
        */

        BoolMsg executeMsg = new BoolMsg(true);
        _ros.Publish("execute_task", executeMsg);
        Debug.Log("ExecutePickAndPlace: Sent execute_task message with value: true");
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
        _ros.Publish("scene_setup_box", boxMsg);
        Debug.Log($"AddBoxToMoveitScene: Published box with dimensions ({dimension_x}, {dimension_y}, {dimension_z}) to scene_setup_box topic");
    }
}
