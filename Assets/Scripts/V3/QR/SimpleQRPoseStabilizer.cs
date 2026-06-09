using UnityEngine;
using UnityEngine.Events;

public class SimpleQRPoseStabilizer : MonoBehaviour, IQRPoseStabilizer
{
    [SerializeField]
    [Tooltip("Adjust the number of frames over which the stabilization occurs. Defaults to 20.")]
    private int _nbStabilizationFrames = 20;

    private Pose[] poses;
    private int _posesFed = 0;
    private bool _isPoseStabilized = false;

    private UnityEvent<Pose> onPoseStabilized;
    public UnityEvent<Pose> PoseStabilized => onPoseStabilized;

    private void Awake()
    {
        onPoseStabilized = new UnityEvent<Pose>();
    }

    public void Start()
    {
        poses = new Pose[_nbStabilizationFrames];
    }

    public void FeedPoseStabilizationAlgorithm(Pose? p) 
    {
        Pose validatedPose;
        if (p.Value != null)
        {
            validatedPose = p.Value;
        }
        else return;
        if (!_isPoseStabilized)
        {
            poses[_posesFed] = validatedPose;
            _posesFed += 1;
            Debug.Log($"Pose {_posesFed} fed.");
        }
        if (_posesFed == _nbStabilizationFrames)
        {
            StabilizePose(poses);
        }
    }

    public void StabilizePose(Pose[] poses)
    {
        Pose stabilizedPose;
        if (poses.Length == 0)
        {
            stabilizedPose = Pose.identity;
        }
        else
        {
            stabilizedPose = poses[poses.Length - 1];
        }
        onPoseStabilized?.Invoke(stabilizedPose);
        _isPoseStabilized = true;
        Debug.Log("Pose is stabilized");
    }

    public void ResetStabilization()
    {
        _isPoseStabilized = false;
        _posesFed = 0;
        for (int i = 0; i < poses.Length; i++)
        {
            poses[i] = Pose.identity;
        }
    }
}
