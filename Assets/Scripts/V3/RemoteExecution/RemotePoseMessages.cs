using System;
using RosMessageTypes.Geometry;

/// <summary>
/// Plain serializable DTOs (Data Transfer Objects) describing a PoseStamped,
/// used to build the JSON payload sent to the remote execution server.
/// Kept independent from ROS message classes so the wire format stays stable
/// and trivially parseable on the C++ side.
/// </summary>
[Serializable]
public class RemoteVector3d
{
    public double x;
    public double y;
    public double z;
}

[Serializable]
public class RemoteQuaterniond
{
    public double x;
    public double y;
    public double z;
    public double w;
}

[Serializable]
public class RemotePoseStamped
{
    public string frame_id;
    public uint sec;
    public uint nanosec;
    public RemoteVector3d position;
    public RemoteQuaterniond orientation;

    public static RemotePoseStamped FromMsg(PoseStampedMsg msg)
    {
        return new RemotePoseStamped
        {
            frame_id = msg.header.frame_id,
            sec = (uint)msg.header.stamp.sec,
            nanosec = msg.header.stamp.nanosec,
            position = new RemoteVector3d
            {
                x = msg.pose.position.x,
                y = msg.pose.position.y,
                z = msg.pose.position.z
            },
            orientation = new RemoteQuaterniond
            {
                x = msg.pose.orientation.x,
                y = msg.pose.orientation.y,
                z = msg.pose.orientation.z,
                w = msg.pose.orientation.w
            }
        };
    }
}

/// <summary>
/// Top-level JSON payload sent to the remote server.
/// 'command' lets one server dispatch to different executables/behaviors later on.
/// </summary>
[Serializable]
public class RemoteExecutionPayload
{
    public string command;
    public RemotePoseStamped[] poses;
}

/// <summary>
/// Outcome of a remote execution request, delivered on the Unity main thread.
/// </summary>
public struct RemoteExecutionResult
{
    public bool Success;
    /// <summary>Raw response line sent back by the server (e.g. {"ok":true,"exit_code":0}).</summary>
    public string Response;
    public string Error;
}
