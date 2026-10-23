using System.Collections.Generic;
using RosMessageTypes.Moveit;
using UnityEngine;

/// <summary>Trajectory replicas share the live robot's calibrated ROS model and TCP.</summary>
public class UR16eTrajectoryVisualizer : MonoBehaviour
{
    public GameObject virtualUr16ePrefab;
    [SerializeField] private URInstantiator _URInstantiator;
    [SerializeField, Range(0f, 100f)] private float _percentageToShow;
    public float PercentageToShow => _percentageToShow;
    public LineRenderer trajectoryLineRenderer;
    public int resolution = 10;
    private readonly List<RosRobotRig> _clones = new List<RosRobotRig>();
    private readonly List<Pose> _plannedTcp = new List<Pose>();
    private RosRobotRig _activeRig;
    private RobotTrajectoryMsg _trajectory;
    private float _previousPercentage = -1f;
    public bool HasVisiblePreview => _trajectory != null && _clones.Count > 0 && _plannedTcp.Count > 0;
    public bool IsShowingTrajectory(RobotTrajectoryMsg trajectory) => HasVisiblePreview && ReferenceEquals(_trajectory, trajectory);

    private void Start()
    {
        if (_URInstantiator != null) _URInstantiator.OnURInstantiated.AddListener(RecordRobot);
        if (RobotStateService.Instance.IsInitialized)
            RecordRobot(RobotStateService.Instance.GetRobotInstance(), Pose.identity);
    }

    private void RecordRobot(GameObject robot, Pose qrPose)
    {
        if (_activeRig != null) _activeRig.ModelChanged -= OnModelChanged;
        _activeRig = robot != null ? robot.GetComponent<RosRobotRig>() : null;
        if (_activeRig != null) _activeRig.ModelChanged += OnModelChanged;
        if (_trajectory != null) RedistributeClones();
    }

    private void OnModelChanged() => SetTrajectory(null);

    private void LateUpdate()
    {
        if (_trajectory != null && !Mathf.Approximately(_previousPercentage, _percentageToShow)) RedistributeClones();
        if (_activeRig == null || _activeRig.BaseLink == null) return;
        Pose basePose = RobotStateService.Instance.GetRobotRootPose();
        foreach (RosRobotRig clone in _clones) if (clone != null) clone.SetBaseWorldPose(basePose);
        if (_plannedTcp.Count > 0) ComputeTrajectorySpline();
    }

    public void UpdatePercentageToShow(float percentage) => _percentageToShow = Mathf.Clamp01(percentage) * 100f;

    public void SetTrajectory(RobotTrajectoryMsg trajectory)
    {
        _trajectory = trajectory;
        RedistributeClones();
    }

    public void ClearAllClones()
    {
        foreach (RosRobotRig clone in _clones)
            if (clone != null) { clone.gameObject.SetActive(false); Destroy(clone.gameObject); }
        _clones.Clear();
        _plannedTcp.Clear();
        if (trajectoryLineRenderer != null) trajectoryLineRenderer.positionCount = 0;
    }

    private void RedistributeClones()
    {
        ClearAllClones();
        _previousPercentage = _percentageToShow;
        if (_trajectory == null || _activeRig == null || _activeRig.Model == null || virtualUr16ePrefab == null) return;
        var trajectory = _trajectory.joint_trajectory;
        if (trajectory == null || trajectory.joint_names == null || trajectory.joint_names.Length == 0 ||
            trajectory.points == null || trajectory.points.Length == 0) { _trajectory = null; return; }
        Dictionary<string, double> positions = new Dictionary<string, double>();
        foreach (RosRobotLink link in _activeRig.Model.links)
            if (RosRobotModelMath.IsRevolute(link) && _activeRig.TryGetJointPosition(link.jointName, out double angle))
                positions[link.jointName] = angle;
        string[] initialNames = new string[positions.Count];
        double[] initialAngles = new double[positions.Count];
        int initialIndex = 0;
        foreach (var joint in positions)
        {
            initialNames[initialIndex] = joint.Key;
            initialAngles[initialIndex++] = joint.Value;
        }
        foreach (var point in trajectory.points)
        {
            if (point == null || point.positions == null || point.positions.Length != trajectory.joint_names.Length)
            {
                ClearAllClones();
                _trajectory = null;
                return;
            }
            HashSet<string> seen = new HashSet<string>();
            for (int index = 0; index < trajectory.joint_names.Length; index++)
            {
                string name = trajectory.joint_names[index];
                if (name == null || double.IsNaN(point.positions[index]) || double.IsInfinity(point.positions[index]) || !seen.Add(name))
                {
                    ClearAllClones();
                    _trajectory = null;
                    return;
                }
                if (positions.ContainsKey(name)) positions[name] = point.positions[index];
            }
            _plannedTcp.Add(RosRobotModelMath.ForwardKinematics(_activeRig.Model, positions)["tcp"]);
        }
        int count = trajectory.points.Length;
        int intermediateCount = Mathf.CeilToInt(Mathf.Clamp01(_percentageToShow / 100f) * (count - 1));
        SortedSet<int> indices = new SortedSet<int>();
        for (int index = 0; index < intermediateCount; index++)
            indices.Add(Mathf.Clamp(Mathf.FloorToInt(index * (count - 1f) / intermediateCount), 0, count - 2));
        indices.Add(count - 1);
        Pose basePose = RobotStateService.Instance.GetRobotRootPose();
        foreach (int index in indices)
        {
            GameObject instance = Instantiate(virtualUr16ePrefab);
            instance.SetActive(false);
            RosRobotRig rig = instance.GetComponent<RosRobotRig>();
            if (rig == null) rig = instance.AddComponent<RosRobotRig>();
            rig.IsGhost = true;
            if (!rig.ApplyModel(_activeRig.Model) ||
                !rig.SetJointPositions(initialNames, initialAngles) ||
                !rig.SetJointPositions(trajectory.joint_names, trajectory.points[index].positions))
            {
                Destroy(instance);
                Debug.LogWarning("Trajectory contains an unsupported robot model or invalid joint values.", this);
                ClearAllClones();
                _trajectory = null;
                return;
            }
            rig.SetBaseWorldPose(basePose);
            instance.SetActive(true);
            rig.SetBaseWorldPose(basePose);
            _clones.Add(rig);
        }
        ComputeTrajectorySpline();
    }

    // Keep the public method used by existing UI events. The line follows actual
    // waypoint TCP samples; interpolation must not imply an unplanned robot path.
    public void ComputeTrajectorySpline()
    {
        if (trajectoryLineRenderer == null) return;
        List<Vector3> points = new List<Vector3>();
        Pose basePose = RobotStateService.Instance.GetRobotRootPose();
        foreach (Pose tcp in _plannedTcp) points.Add(RobotFrameMath.Compose(basePose, tcp).position);
        trajectoryLineRenderer.useWorldSpace = true;
        trajectoryLineRenderer.positionCount = points.Count;
        trajectoryLineRenderer.SetPositions(points.ToArray());
    }

    private void OnDestroy()
    {
        if (_URInstantiator != null) _URInstantiator.OnURInstantiated.RemoveListener(RecordRobot);
        if (_activeRig != null) _activeRig.ModelChanged -= OnModelChanged;
        ClearAllClones();
    }
}
