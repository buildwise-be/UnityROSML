using System;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// Scene-facing controller for triggering a remote executable with PoseStamped data.
/// Hook <see cref="TriggerRemoteExecution"/> to a UI Button's OnClick.
///
/// The poses are taken from the Transforms assigned in <see cref="_poseSources"/>,
/// converted from Unity world space to the robot base link frame (via
/// RobotCoordinateService) and then to ROS FLU convention — the same pipeline used
/// by PickPlaceController — before being shipped to the remote machine by
/// RemoteExecutorClient. If you need to send poses that are computed elsewhere,
/// call <see cref="TriggerRemoteExecution(PoseStampedMsg[])"/> directly.
/// </summary>
public class RemoteExecutionController : MonoBehaviour
{
    [SerializeField]
    [Tooltip("Logical command name sent to the launcher server; it decides which executable to run.")]
    private string _command = "run_task";

    [SerializeField]
    [Tooltip("Frame id stamped on each pose (coordinate frame the poses are expressed in).")]
    private string _frameId = "ur_base_link";

    [SerializeField]
    [Tooltip("Transforms whose poses are sent to the remote executable, in order. E.g. pick/place markers.")]
    private List<Transform> _poseSources = new List<Transform>();

    [SerializeField]
    [Tooltip("If true, world poses are converted to the robot base link frame via RobotCoordinateService. If false, raw Unity world poses are used (still converted to FLU).")]
    private bool _convertToRobotFrame = true;

    [Tooltip("Invoked on the main thread when the remote execution finished successfully.")]
    public UnityEvent OnRemoteExecutionSucceeded;

    [Tooltip("Invoked on the main thread with an error description when the remote execution failed.")]
    public UnityEvent<string> OnRemoteExecutionFailed;

    /// <summary>
    /// Replace the pose sources at runtime (e.g. called by PickPlaceController when
    /// the pick/place markers are created or moved). Null entries are ignored.
    /// </summary>
    public void SetPoseSources(params Transform[] sources)
    {
        _poseSources.Clear();
        if (sources == null)
        {
            return;
        }

        foreach (Transform source in sources)
        {
            if (source != null)
            {
                _poseSources.Add(source);
            }
        }
    }

    /// <summary>
    /// Entry point for UI buttons. Collects poses from _poseSources and sends them.
    /// </summary>
    public void TriggerRemoteExecution()
    {
        if (_poseSources == null || _poseSources.Count == 0)
        {
            Debug.LogError("RemoteExecutionController: No pose sources assigned.");
            OnRemoteExecutionFailed?.Invoke("No pose sources assigned");
            return;
        }

        var poses = new List<PoseStampedMsg>(_poseSources.Count);
        foreach (Transform source in _poseSources)
        {
            if (source == null)
            {
                Debug.LogWarning("RemoteExecutionController: Skipping null pose source.");
                continue;
            }

            Pose worldPose = new Pose(source.position, source.rotation);
            Pose localPose = _convertToRobotFrame
                ? RobotCoordinateService.Instance.ConvertWorldPoseToLocalPose(worldPose)
                : worldPose;

            poses.Add(GeneratePoseStamped(localPose, _frameId));
        }

        TriggerRemoteExecution(poses.ToArray());
    }

    /// <summary>
    /// Send already-built PoseStamped messages to the remote executable.
    /// </summary>
    public void TriggerRemoteExecution(PoseStampedMsg[] poses)
    {
        RemoteExecutorClient.Instance.Execute(_command, poses, result =>
        {
            if (result.Success)
            {
                OnRemoteExecutionSucceeded?.Invoke();
            }
            else
            {
                OnRemoteExecutionFailed?.Invoke(result.Error);
            }
        });
    }

    /// <summary>
    /// Convenience overload for a single pose.
    /// </summary>
    public void TriggerRemoteExecution(PoseStampedMsg pose)
    {
        TriggerRemoteExecution(new[] { pose });
    }

    /// <summary>
    /// Generate a PoseStampedMsg from a robot-local Unity pose (RUF), converting to ROS FLU.
    /// Mirrors PickPlaceController.GeneratePoseStamped.
    /// </summary>
    private PoseStampedMsg GeneratePoseStamped(Pose localPose, string frameId)
    {
        PointMsg rosPos = localPose.position.To<FLU>();
        QuaternionMsg rosRot = localPose.rotation.To<FLU>();

        HeaderMsg headerMsg = new HeaderMsg();
        headerMsg.frame_id = frameId;

        DateTime epochStart = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        double totalSeconds = (DateTime.UtcNow - epochStart).TotalSeconds;
        // TimeMsg.sec is int in ROS2 mode and uint in ROS1 mode
#if ROS2
        int sec = (int)totalSeconds;
#else
        uint sec = (uint)totalSeconds;
#endif
        uint nanosec = (uint)((totalSeconds - sec) * 1e9);
        headerMsg.stamp = new TimeMsg(sec, nanosec);

        return new PoseStampedMsg(headerMsg, new PoseMsg(rosPos, rosRot));
    }
}
