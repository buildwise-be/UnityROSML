using UnityEngine;
using UnityEngine.Events;

public interface IQRPoseStabilizer
{
    public void StabilizePose(Pose[] poses);
    public void FeedPoseStabilizationAlgorithm(Pose? p);

    public UnityEvent<Pose> PoseStabilized { get; }

    public void ResetStabilization();
}
