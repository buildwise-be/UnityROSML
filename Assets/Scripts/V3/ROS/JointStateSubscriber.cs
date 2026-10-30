using RosMessageTypes.Sensor;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>Matches incoming UR joints by name; mobile-base joints may share this topic.</summary>
public class JointStateSubscriber : MonoBehaviour
{
    public ArticulationBody[] jointArticulations;
    [SerializeField] private URInstantiator _URInstantiator;
    private RosRobotRig _rig;
    private ROSConnection _connection;
    private readonly Dictionary<string, double> _latestPositions = new Dictionary<string, double>();
    private readonly Dictionary<string, float> _receivedAt = new Dictionary<string, float>();

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
        if (_rig != null && _rig.Model != null &&
            TryGetFreshJointPositions(_rig.Model, 1f, out Dictionary<string, double> positions, out _))
            _rig.SetJointPositions(new List<string>(positions.Keys).ToArray(), new List<double>(positions.Values).ToArray());
    }

    private void OnJointStateReceived(JointStateMsg message)
    {
        if (message == null || !ObserveJointState(message.name, message.position)) return;
        if (_rig != null && !_rig.SetJointPositions(message.name, message.position))
            Debug.LogWarning("JointStateSubscriber: Ignored malformed joint feedback.", this);
    }

    // Retain feedback even before there is a visible robot. Never substitute zero joints.
    public bool ObserveJointState(string[] names, double[] positions)
    {
        if (names == null || positions == null || names.Length != positions.Length) return false;
        HashSet<string> seen = new HashSet<string>();
        for (int i = 0; i < names.Length; i++)
            if (string.IsNullOrWhiteSpace(names[i]) || !seen.Add(names[i]) ||
                double.IsNaN(positions[i]) || double.IsInfinity(positions[i])) return false;
        for (int i = 0; i < names.Length; i++)
        {
            _latestPositions[names[i]] = positions[i];
            _receivedAt[names[i]] = Time.realtimeSinceStartup;
        }
        return true;
    }

    public bool TryGetFreshJointPositions(RosRobotModel model, float maximumAgeSeconds,
        out Dictionary<string, double> positions, out string error)
    {
        positions = new Dictionary<string, double>();
        error = null;
        if (model == null || model.links == null || !(maximumAgeSeconds > 0f) || float.IsInfinity(maximumAgeSeconds))
        { error = "A robot model and valid feedback age are required."; return false; }
        foreach (RosRobotLink link in model.links)
        {
            if (!RosRobotModelMath.IsRevolute(link)) continue;
            if (!_latestPositions.TryGetValue(link.jointName, out double value) ||
                !_receivedAt.TryGetValue(link.jointName, out float time) ||
                Time.realtimeSinceStartup - time > maximumAgeSeconds)
            { error = "Waiting for fresh ROS joint feedback: " + link.jointName + "."; return false; }
            positions.Add(link.jointName, value);
        }
        if (positions.Count != 6) { error = "Fresh feedback for all six UR arm joints is required."; return false; }
        return true;
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
