using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>Matches incoming UR joints by name; mobile-base joints may share this topic.</summary>
public class JointStateSubscriber : MonoBehaviour
{
    public ArticulationBody[] jointArticulations;
    [SerializeField] private URInstantiator _URInstantiator;
    private RosRobotRig _rig;
    private ROSConnection _connection;

    private void Start()
    {
        _connection = ROSService.Instance.GetROSConnection();
        _connection.Subscribe<JointStateMsg>("/joint_states", OnJointStateReceived);
        if (_URInstantiator != null) _URInstantiator.OnURInstantiated.AddListener(StoreArticulationBodies);
        if (RobotStateService.Instance.IsInitialized)
            StoreArticulationBodies(RobotStateService.Instance.GetRobotInstance(), Pose.identity);
    }

    private void StoreArticulationBodies(GameObject robot, Pose qrPose)
    {
        _rig = robot != null ? robot.GetComponent<RosRobotRig>() : null;
        jointArticulations = robot != null ? robot.GetComponentsInChildren<ArticulationBody>() : null;
    }

    private void OnJointStateReceived(JointStateMsg message)
    {
        if (_rig != null && !_rig.SetJointPositions(message.name, message.position))
            Debug.LogWarning("JointStateSubscriber: Ignored malformed joint feedback.", this);
    }

    public void PrintCurrentJointStates()
    {
        if (_rig == null || _rig.Model == null) return;
        foreach (RosRobotLink link in _rig.Model.links)
            if (RosRobotModelMath.IsRevolute(link) && _rig.TryGetJointPosition(link.jointName, out double radians))
                Debug.Log(link.jointName + ": " + radians + " radians", this);
    }

    private void OnDestroy()
    {
        if (_URInstantiator != null) _URInstantiator.OnURInstantiated.RemoveListener(StoreArticulationBodies);
        if (_connection != null) _connection.Unsubscribe("/joint_states");
    }
}
