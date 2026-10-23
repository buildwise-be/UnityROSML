using UnityEngine;
using UnityEngine.Events;

public class SimpleQRPoseStabilizer : MonoBehaviour, IQRPoseStabilizer
{
    [SerializeField]
    [Min(1)]
    [Tooltip("The number of valid tracking-space poses to average.")]
    private int _nbStabilizationFrames = 20;

    [SerializeField, Min(0.001f), Tooltip("Restart the averaging window if the selected QR moves this far from its first sample.")]
    private float _maximumPositionSpreadMeters = 0.025f;
    [SerializeField, Range(0.1f, 45f), Tooltip("Restart the averaging window if the selected QR turns this far from its first sample.")]
    private float _maximumAngleSpreadDegrees = 5f;

    private Pose[] poses;
    private int _posesFed;
    private bool _isPoseStabilized;
    private Pose _windowReference;

    public int SamplesCollected => _posesFed;
    public int RequiredSamples => Mathf.Max(1, _nbStabilizationFrames);
    public int DiscontinuityCount { get; private set; }

    private readonly UnityEvent<Pose> onPoseStabilized = new UnityEvent<Pose>();
    public UnityEvent<Pose> PoseStabilized => onPoseStabilized;

    public void FeedPoseStabilizationAlgorithm(Pose? p)
    {
        if (_isPoseStabilized) return;
        if (!p.HasValue || !RobotFrameMath.IsValidPose(p.Value))
        {
            ResetStabilization();
            return;
        }

        EnsureBuffer();
        if (_posesFed > 0 && !IsWithinWindow(_windowReference, p.Value))
        {
            _posesFed = 0;
            DiscontinuityCount++;
        }
        if (_posesFed == 0) _windowReference = p.Value;
        poses[_posesFed++] = p.Value;
        if (_posesFed == poses.Length)
        {
            StabilizePose(poses);
        }
    }

    public void StabilizePose(Pose[] samples)
    {
        if (_isPoseStabilized || samples == null || samples.Length == 0)
        {
            return;
        }

        double positionX = 0, positionY = 0, positionZ = 0;
        Vector4 rotationSum = Vector4.zero;
        Quaternion referenceRotation = Quaternion.identity;
        int validCount = 0;
        foreach (Pose sample in samples)
        {
            if (!RobotFrameMath.IsValidPose(sample)) continue;

            Quaternion source = sample.rotation;
            double magnitude = System.Math.Sqrt((double)source.x * source.x + (double)source.y * source.y +
                (double)source.z * source.z + (double)source.w * source.w);
            Quaternion rotation = new Quaternion((float)(source.x / magnitude), (float)(source.y / magnitude),
                (float)(source.z / magnitude), (float)(source.w / magnitude));
            if (validCount == 0) referenceRotation = rotation;
            float sign = Quaternion.Dot(referenceRotation, rotation) < 0f ? -1f : 1f;
            rotationSum += sign * new Vector4(rotation.x, rotation.y, rotation.z, rotation.w);
            positionX += sample.position.x;
            positionY += sample.position.y;
            positionZ += sample.position.z;
            validCount++;
        }

        if (validCount == 0) return;
        Quaternion averagedRotation = new Quaternion(rotationSum.x, rotationSum.y, rotationSum.z, rotationSum.w).normalized;
        Vector3 averagedPosition = new Vector3((float)(positionX / validCount),
            (float)(positionY / validCount), (float)(positionZ / validCount));
        Pose averagedPose = new Pose(averagedPosition, averagedRotation);
        if (!RobotFrameMath.IsValidPose(averagedPose)) return;

        _isPoseStabilized = true;
        onPoseStabilized.Invoke(averagedPose);
    }

    public void ResetStabilization()
    {
        _isPoseStabilized = false;
        _posesFed = 0;
        EnsureBuffer();
        System.Array.Clear(poses, 0, poses.Length);
    }

    private bool IsWithinWindow(Pose first, Pose next)
    {
        float distance = float.IsNaN(_maximumPositionSpreadMeters) || float.IsInfinity(_maximumPositionSpreadMeters)
            ? 0.025f : Mathf.Max(0.001f, _maximumPositionSpreadMeters);
        float angle = float.IsNaN(_maximumAngleSpreadDegrees) || float.IsInfinity(_maximumAngleSpreadDegrees)
            ? 5f : Mathf.Clamp(_maximumAngleSpreadDegrees, 0.1f, 45f);
        // Compare complete rotations, treating q and -q as the same orientation.
        Quaternion a = RobotFrameMath.Compose(Pose.identity, new Pose(Vector3.zero, first.rotation)).rotation;
        Quaternion b = RobotFrameMath.Compose(Pose.identity, new Pose(Vector3.zero, next.rotation)).rotation;
        float dot = Mathf.Clamp01(Mathf.Abs(Quaternion.Dot(a, b)));
        return (first.position - next.position).sqrMagnitude <= distance * distance &&
            dot >= Mathf.Cos(angle * 0.5f * Mathf.Deg2Rad);
    }

    private void EnsureBuffer()
    {
        int frameCount = Mathf.Max(1, _nbStabilizationFrames);
        if (poses == null || poses.Length != frameCount)
        {
            poses = new Pose[frameCount];
            _posesFed = 0;
        }
    }
}
