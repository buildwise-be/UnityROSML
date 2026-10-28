using System;
using System.Collections.Generic;
using RosMessageTypes.Moveit;
using UnityEngine;
using UnityEngine.Rendering;

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
    private RosRobotRig _playbackRig;
    private Material _playbackMaterial;
    private RobotTrajectoryMsg _trajectory;
    private string[] _jointNames = Array.Empty<string>();
    private string[] _initialJointNames = Array.Empty<string>();
    private double[] _initialJointAngles = Array.Empty<double>();
    private double[][] _jointSamples = Array.Empty<double[]>();
    private double[] _sampleTimes = Array.Empty<double>();
    private double[] _interpolatedAngles = Array.Empty<double>();
    private double _playbackTime;
    private float _playbackSpeed = 1f;
    private bool _isPlaying;
    private float _previousPercentage = -1f;

    public bool IsReadyForPreview => _activeRig != null && _activeRig.Model != null && virtualUr16ePrefab != null;
    public bool HasPlayback => _trajectory != null && _playbackRig != null && _sampleTimes.Length > 0;
    public bool IsPlaying => HasPlayback && _isPlaying;
    public float PlaybackTimeSeconds => (float)_playbackTime;
    public float DurationSeconds => _sampleTimes.Length > 0 ? (float)_sampleTimes[_sampleTimes.Length - 1] : 0f;
    public float PlaybackNormalized => DurationSeconds > 0f ? Mathf.Clamp01(PlaybackTimeSeconds / DurationSeconds) : 0f;
    public float PlaybackSpeed => _playbackSpeed;
    public bool HasVisiblePreview => _trajectory != null && _plannedTcp.Count > 0 && (_clones.Count > 0 || HasPlayback);
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
        if (_trajectory != null)
        {
            RobotTrajectoryMsg pending = _trajectory;
            ClearAllClones();
            SetTrajectory(pending);
        }
    }

    private void OnModelChanged() => SetTrajectory(null);

    private void LateUpdate()
    {
        if (_trajectory != null && !Mathf.Approximately(_previousPercentage, _percentageToShow)) RedistributeClones();
        if (_activeRig == null || _activeRig.BaseLink == null) return;
        Pose basePose = RobotStateService.Instance.GetRobotRootPose();
        foreach (RosRobotRig clone in _clones) if (clone != null) clone.SetBaseWorldPose(basePose);
        if (_playbackRig != null) _playbackRig.SetBaseWorldPose(basePose);
        AdvancePlayback(Time.unscaledDeltaTime);
        if (_plannedTcp.Count > 0) ComputeTrajectorySpline();
    }

    public void UpdatePercentageToShow(float percentage) => _percentageToShow = Mathf.Clamp01(percentage) * 100f;

    public void SetTrajectory(RobotTrajectoryMsg trajectory)
    {
        if (ReferenceEquals(_trajectory, trajectory) && HasPlayback) return;
        ClearAllClones();
        _trajectory = trajectory;
        if (_trajectory == null || !IsReadyForPreview) return;
        if (!CacheTrajectory() || !CreatePlaybackRig())
        {
            ClearAllClones();
            return;
        }
        // Prepare the first pose without overlaying the live robot until Play.
        ApplyPlaybackPose();
        RedistributeClones();
    }

    public void Play()
    {
        if (!HasPlayback) return;
        _playbackRig.gameObject.SetActive(true);
        if (_playbackTime >= _sampleTimes[_sampleTimes.Length - 1])
        {
            _playbackTime = 0d;
            ApplyPlaybackPose();
        }
        _isPlaying = DurationSeconds > 0f;
    }

    public void Pause() => _isPlaying = false;

    public void Replay()
    {
        if (!HasPlayback) return;
        _playbackRig.gameObject.SetActive(true);
        _playbackTime = 0d;
        ApplyPlaybackPose();
        _isPlaying = DurationSeconds > 0f;
    }

    /// <summary>Scrubbing pauses virtual playback. It never changes the reviewed ROS plan.</summary>
    public void Seek(float normalized)
    {
        if (!Finite(normalized) || !HasPlayback) return;
        Pause();
        _playbackTime = Mathf.Clamp01(normalized) * _sampleTimes[_sampleTimes.Length - 1];
        ApplyPlaybackPose();
    }

    public void SetPlaybackSpeed(float speed)
    {
        if (Finite(speed)) _playbackSpeed = Mathf.Clamp(speed, 0.25f, 2f);
    }

    private void AdvancePlayback(float deltaSeconds)
    {
        if (!IsPlaying || !Finite(deltaSeconds) || deltaSeconds <= 0f) return;
        double end = _sampleTimes[_sampleTimes.Length - 1];
        _playbackTime = Math.Min(end, _playbackTime + deltaSeconds * (double)_playbackSpeed);
        ApplyPlaybackPose();
        if (_playbackTime >= end) Pause();
    }

    private void ApplyPlaybackPose()
    {
        if (!HasPlayback) return;
        int last = _sampleTimes.Length - 1;
        int lower = 0;
        int upper = _sampleTimes.Length;
        // Upper bound selects the final copy of a repeated MTC segment boundary.
        while (lower < upper)
        {
            int middle = lower + (upper - lower) / 2;
            if (_sampleTimes[middle] <= _playbackTime) lower = middle + 1;
            else upper = middle;
        }
        int before = Mathf.Clamp(lower - 1, 0, last);
        int after = Mathf.Clamp(lower, 0, last);
        double span = _sampleTimes[after] - _sampleTimes[before];
        double fraction = span > 0d ? Math.Max(0d, Math.Min(1d, (_playbackTime - _sampleTimes[before]) / span)) : 0d;
        for (int index = 0; index < _interpolatedAngles.Length; index++)
        {
            double start = _jointSamples[before][index];
            // ROS values are already unwrapped. Angle wrapping would invent a different arm motion.
            _interpolatedAngles[index] = start + (_jointSamples[after][index] - start) * fraction;
        }
        _playbackRig.SetJointPositions(_jointNames, _interpolatedAngles);
    }

    private bool CacheTrajectory()
    {
        var trajectory = _trajectory.joint_trajectory;
        if (trajectory == null || trajectory.joint_names == null || trajectory.joint_names.Length == 0 ||
            trajectory.points == null || trajectory.points.Length == 0) return false;
        Dictionary<string, double> positions = new Dictionary<string, double>();
        foreach (RosRobotLink link in _activeRig.Model.links)
            if (RosRobotModelMath.IsRevolute(link) && _activeRig.TryGetJointPosition(link.jointName, out double angle))
                positions[link.jointName] = angle;
        HashSet<string> seen = new HashSet<string>();
        foreach (string name in trajectory.joint_names)
            if (string.IsNullOrEmpty(name) || !positions.ContainsKey(name) || !seen.Add(name)) return false;
        _initialJointNames = new string[positions.Count];
        _initialJointAngles = new double[positions.Count];
        int initialIndex = 0;
        foreach (var joint in positions)
        {
            _initialJointNames[initialIndex] = joint.Key;
            _initialJointAngles[initialIndex++] = joint.Value;
        }
        _jointNames = (string[])trajectory.joint_names.Clone();
        _jointSamples = new double[trajectory.points.Length][];
        _sampleTimes = new double[trajectory.points.Length];
        _interpolatedAngles = new double[_jointNames.Length];
        for (int pointIndex = 0; pointIndex < trajectory.points.Length; pointIndex++)
        {
            var point = trajectory.points[pointIndex];
            if (point == null || point.positions == null || point.positions.Length != _jointNames.Length ||
                point.time_from_start == null || point.time_from_start.sec < 0 ||
                (double)point.time_from_start.nanosec < 0d || point.time_from_start.nanosec >= 1000000000) return false;
            double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9d;
            if (pointIndex > 0 && time < _sampleTimes[pointIndex - 1]) return false;
            _sampleTimes[pointIndex] = time;
            _jointSamples[pointIndex] = (double[])point.positions.Clone();
            for (int jointIndex = 0; jointIndex < _jointNames.Length; jointIndex++)
            {
                double value = _jointSamples[pointIndex][jointIndex];
                if (double.IsNaN(value) || double.IsInfinity(value)) return false;
                // MTC concatenation repeats its seam timestamps with small numerical IK differences.
                // Accept up to 0.001 rad at that instant; larger jumps have no valid playback timing.
                if (pointIndex > 0 && time == _sampleTimes[pointIndex - 1] &&
                    Math.Abs(value - _jointSamples[pointIndex - 1][jointIndex]) > 0.001d) return false;
                positions[_jointNames[jointIndex]] = value;
            }
            _plannedTcp.Add(RosRobotModelMath.ForwardKinematics(_activeRig.Model, positions)["tcp"]);
        }
        return true;
    }

    private bool CreatePlaybackRig()
    {
        _playbackRig = CreateReplica(_jointSamples[0]);
        if (_playbackRig == null) return false;
        _playbackRig.name = "Planned arm and tool preview";
        _playbackRig.gameObject.SetActive(false);
        try
        {
            // Serialized Resources materials retain this shader and its XR transparent variant in Android builds.
            _playbackMaterial = TargetMarkerMaterials.Create(new Color(0.12f, 0.85f, 1f, 0.48f), true);
            _playbackMaterial.name = "Planned arm and tool cyan preview";
            foreach (Renderer renderer in _playbackRig.GetComponentsInChildren<Renderer>(true))
            {
                Material[] materials = renderer.sharedMaterials;
                for (int index = 0; index < materials.Length; index++) materials[index] = _playbackMaterial;
                renderer.sharedMaterials = materials;
                renderer.shadowCastingMode = ShadowCastingMode.Off;
                renderer.receiveShadows = false;
            }
            return true;
        }
        catch (Exception exception)
        {
            Debug.LogWarning("Trajectory playback material could not be loaded: " + exception.Message, this);
            return false;
        }
    }

    private RosRobotRig CreateReplica(double[] angles)
    {
        GameObject instance = Instantiate(virtualUr16ePrefab);
        instance.SetActive(false);
        RosRobotRig rig = instance.GetComponent<RosRobotRig>();
        if (rig == null) rig = instance.AddComponent<RosRobotRig>();
        rig.IsGhost = true;
        if (!rig.ApplyModel(_activeRig.Model) || !rig.SetJointPositions(_initialJointNames, _initialJointAngles) ||
            !rig.SetJointPositions(_jointNames, angles))
        {
            Dispose(instance);
            Debug.LogWarning("Trajectory contains an unsupported robot model or invalid joint values.", this);
            return null;
        }
        Pose basePose = RobotStateService.Instance.GetRobotRootPose();
        rig.SetBaseWorldPose(basePose);
        instance.SetActive(true);
        rig.SetBaseWorldPose(basePose);
        return rig;
    }

    public void ClearAllClones()
    {
        ClearStaticClones();
        if (_playbackRig != null)
        {
            _playbackRig.gameObject.SetActive(false);
            Dispose(_playbackRig.gameObject);
        }
        _playbackRig = null;
        if (_playbackMaterial != null) Dispose(_playbackMaterial);
        _playbackMaterial = null;
        _trajectory = null;
        _plannedTcp.Clear();
        _jointNames = Array.Empty<string>();
        _initialJointNames = Array.Empty<string>();
        _initialJointAngles = Array.Empty<double>();
        _jointSamples = Array.Empty<double[]>();
        _sampleTimes = Array.Empty<double>();
        _interpolatedAngles = Array.Empty<double>();
        _playbackTime = 0d;
        _isPlaying = false;
        if (trajectoryLineRenderer != null) trajectoryLineRenderer.positionCount = 0;
    }

    private void ClearStaticClones()
    {
        foreach (RosRobotRig clone in _clones)
            if (clone != null) { clone.gameObject.SetActive(false); Dispose(clone.gameObject); }
        _clones.Clear();
    }

    private void RedistributeClones()
    {
        ClearStaticClones();
        _previousPercentage = _percentageToShow;
        if (_trajectory == null || !IsReadyForPreview || _jointSamples.Length == 0) return;
        int count = _jointSamples.Length;
        int intermediateCount = Mathf.CeilToInt(Mathf.Clamp01(_percentageToShow / 100f) * (count - 1));
        SortedSet<int> indices = new SortedSet<int>();
        for (int index = 0; index < intermediateCount; index++)
            indices.Add(Mathf.Clamp(Mathf.FloorToInt(index * (count - 1f) / intermediateCount), 0, count - 2));
        indices.Add(count - 1);
        foreach (int index in indices)
        {
            RosRobotRig rig = CreateReplica(_jointSamples[index]);
            if (rig == null) { ClearAllClones(); return; }
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

    private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    private static void Dispose(UnityEngine.Object value)
    {
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }

    private void OnDisable() => Pause();

    private void OnApplicationFocus(bool focused)
    {
        if (!focused) Pause();
    }

    private void OnApplicationPause(bool paused)
    {
        if (paused) Pause();
    }

    private void OnDestroy()
    {
        if (_URInstantiator != null) _URInstantiator.OnURInstantiated.RemoveListener(RecordRobot);
        if (_activeRig != null) _activeRig.ModelChanged -= OnModelChanged;
        ClearAllClones();
    }
}
