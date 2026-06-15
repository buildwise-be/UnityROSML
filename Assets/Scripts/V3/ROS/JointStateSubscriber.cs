using RosMessageTypes.Sensor;
using UnityEngine;
using Unity.Robotics.UrdfImporter;
using System.Collections.Generic;

public class JointStateSubscriber : MonoBehaviour
{
    public ArticulationBody[] jointArticulations;
    
    [SerializeField]
    [Tooltip("The prefix used ROS-side for the UR16e URDF links.")]
    private string _rosUR16ePrefix = "ur_";

    [SerializeField]
    private URInstantiator _URInstantiator;

    const int k_NumRobotJoints = 6;
    public static readonly string[] LinkNames =
       { "world/base_link/base_link_inertia/shoulder_link", "/upper_arm_link", "/forearm_link", "/wrist_1_link", "/wrist_2_link", "/wrist_3_link" };

    public Dictionary<string, string> ROSNameToUnityName;

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    private bool _hasUR16eBeenInstantiated = false;

    void Start()
    {
        InitializeROSNameToUnityNameDict();
        
        // Use ROSService for subscription
        ROSService.Instance.Subscribe<JointStateMsg>("/joint_states", OnJointStateReceived);
        Debug.Log("Subscribed to /joint_states");

        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        // Listen to RobotStateService instead of MarkerUnderstander directly
        //RobotStateService.Instance.OnRobotInstantiatedAndPlaced.AddListener(RecordInstantiatedUR16e);
        if (_URInstantiator.OnURInstantiated != null)
        {
            _URInstantiator.OnURInstantiated.AddListener(StoreArticulationsBodies);
            Debug.Log("StoreArticulationBodies added as listener.");
        }
    }

    private void StoreArticulationsBodies(GameObject go, Pose p)
    {
        Debug.Log("StoreArticulationBodies called");
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            Transform child = go.transform.Find(linkName);
            m_JointArticulationBodies[i] = child.GetComponent<UrdfJointRevolute>();
        }
        _hasUR16eBeenInstantiated = true;
        Debug.Log("JointStateSubscriber: Robot instance recorded and ready for joint updates");
    }

    private void InitializeROSNameToUnityNameDict()
        {
        ROSNameToUnityName = new Dictionary<string, string>(){
        { _rosUR16ePrefix + "shoulder_pan_joint",   "shoulder_link" },
        { _rosUR16ePrefix + "shoulder_lift_joint",  "upper_arm_link" },
        { _rosUR16ePrefix + "elbow_joint",          "forearm_link" },
        { _rosUR16ePrefix + "wrist_1_joint",        "wrist_1_link" },
        { _rosUR16ePrefix + "wrist_2_joint",        "wrist_2_link" },
        { _rosUR16ePrefix + "wrist_3_joint",        "wrist_3_link" }
        };
    }

    /*
    private void RecordInstantiatedUR16e(Pose p, GameObject instantiatedUR16e)
    {
        m_UR16e = instantiatedUR16e;
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            Transform child = m_UR16e.transform.Find(linkName);
            m_JointArticulationBodies[i] = child.GetComponent<UrdfJointRevolute>();
        }
        _hasUR16eBeenInstantiated = true;
        Debug.Log("JointStateSubscriber: Robot instance recorded and ready for joint updates");
    }
    */

    void OnJointStateReceived(JointStateMsg msg)
    {
        if (!_hasUR16eBeenInstantiated)
        {
            return;
        }
        for (int i = 0; i < m_JointArticulationBodies.Length; i++)
        {
            float angleDeg = Mathf.Rad2Deg * (float)msg.position[i];  // convert radians to degrees
            string go_name = ROSNameToUnityName[msg.name[i]];

            foreach (var item in m_JointArticulationBodies)
            {
                if (item.gameObject.name == go_name)
                {
                    var drive = item.gameObject.GetComponent<ArticulationBody>().xDrive;
                    drive.target = angleDeg;
                    item.gameObject.GetComponent<ArticulationBody>().xDrive = drive;
                    break;
                }
            }
        }
    }

    public void PrintCurrentJointStates()
    {
        foreach (var item in m_JointArticulationBodies)
        {
            string rosName = "";
            foreach (var dictVal in ROSNameToUnityName)
            {
                if (dictVal.Value == item.gameObject.name)
                {
                    rosName = dictVal.Key;
                }
            }
            Debug.Log($"Gameobject {item.gameObject.name} (ROS link {rosName}) has: " +
                $"\n\t angle value {item.gameObject.GetComponent<ArticulationBody>().xDrive.target}" +
                $"\n\t rad value {Mathf.Deg2Rad * item.gameObject.GetComponent<ArticulationBody>().xDrive.target}");
        }
    }
}