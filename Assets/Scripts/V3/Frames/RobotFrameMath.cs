using System;
using UnityEngine;

/// <summary>
/// Rigid poses in Unity's right/up/forward convention. Convert to ROS axes only
/// after expressing a pose relative to its intended ROS reference frame.
/// </summary>
public static class RobotFrameMath
{
    private const float FrameTolerance = 0.0001f;

    public static Pose Compose(Pose parent, Pose local)
    {
        RequireValid(parent, nameof(parent));
        RequireValid(local, nameof(local));
        Quaternion parentRotation = Normalize(parent.rotation);
        return new Pose(parent.position + parentRotation * local.position,
            Normalize(parentRotation * Normalize(local.rotation)));
    }

    public static Pose Inverse(Pose pose)
    {
        RequireValid(pose, nameof(pose));
        Quaternion inverseRotation = Quaternion.Inverse(Normalize(pose.rotation));
        return new Pose(inverseRotation * -pose.position, inverseRotation);
    }

    /// <summary>Express a world pose in the coordinates of a rigid frame.</summary>
    public static Pose RelativeTo(Pose frame, Pose world)
    {
        return Compose(Inverse(frame), world);
    }

    public static bool IsValidPose(Pose pose)
    {
        Quaternion q = pose.rotation;
        return IsFinite(pose.position) && IsFinite(q.x) && IsFinite(q.y) &&
            IsFinite(q.z) && IsFinite(q.w) &&
            ((double)q.x * q.x + (double)q.y * q.y +
             (double)q.z * q.z + (double)q.w * q.w) > 0.0;
    }

    /// <summary>
    /// Require a finite, unit-scale, proper rigid frame. Scaling or shearing a
    /// tracking/robot frame changes metres and cannot represent a ROS TF frame.
    /// </summary>
    public static bool IsRigidFrame(Transform frame)
    {
        if (frame == null)
            return false;

        Matrix4x4 matrix = frame.localToWorldMatrix;
        Vector3 right = matrix.GetColumn(0);
        Vector3 up = matrix.GetColumn(1);
        Vector3 forward = matrix.GetColumn(2);
        Vector3 position = matrix.GetColumn(3);
        if (!IsFinite(right) || !IsFinite(up) || !IsFinite(forward) || !IsFinite(position))
            return false;

        if (Mathf.Abs(right.sqrMagnitude - 1f) > FrameTolerance ||
            Mathf.Abs(up.sqrMagnitude - 1f) > FrameTolerance ||
            Mathf.Abs(forward.sqrMagnitude - 1f) > FrameTolerance ||
            Mathf.Abs(Vector3.Dot(right, up)) > FrameTolerance ||
            Mathf.Abs(Vector3.Dot(right, forward)) > FrameTolerance ||
            Mathf.Abs(Vector3.Dot(up, forward)) > FrameTolerance ||
            Vector3.Dot(Vector3.Cross(right, up), forward) < 1f - FrameTolerance)
            return false;

        Pose framePose = new Pose(position, frame.rotation);
        if (!IsValidPose(framePose))
            return false;

        // Also reject negative-scale transforms whose matrix axes disagree with
        // Transform.rotation, even when two reflections give determinant +1.
        Quaternion rotation = Normalize(frame.rotation);
        float toleranceSquared = FrameTolerance * FrameTolerance;
        return (right - rotation * Vector3.right).sqrMagnitude <= toleranceSquared &&
            (up - rotation * Vector3.up).sqrMagnitude <= toleranceSquared &&
            (forward - rotation * Vector3.forward).sqrMagnitude <= toleranceSquared;
    }

    /// <summary>
    /// Convert an OpenXR tracking-space pose through the XR camera-floor-offset
    /// frame. Pass that frame, not the tracked camera transform itself.
    /// </summary>
    public static bool TryTrackingToWorld(Transform trackingSpace, Pose trackingPose, out Pose worldPose)
    {
        worldPose = default;
        if (!IsRigidFrame(trackingSpace) || !IsValidPose(trackingPose))
            return false;

        worldPose = Compose(new Pose(trackingSpace.position, trackingSpace.rotation), trackingPose);
        return IsValidPose(worldPose);
    }

    /// <summary>
    /// Make a canonical TCP pose whose ROS +Z points along the controller aim
    /// (+Z in Unity). ROSGeometry maps ROS +Z to local Unity +Y, so the TCP
    /// frame needs this fixed 90-degree local-X rotation before conversion.
    /// </summary>
    public static Pose PointerToTcpPose(Pose pointerWorld)
    {
        RequireValid(pointerWorld, nameof(pointerWorld));
        return new Pose(pointerWorld.position,
            Normalize(Normalize(pointerWorld.rotation) * Quaternion.AngleAxis(90f, Vector3.right)));
    }

    /// <summary>The physical ROS TCP +Z direction, expressed in Unity world space.</summary>
    public static Vector3 TcpPositiveZ(Pose tcpWorld)
    {
        RequireValid(tcpWorld, nameof(tcpWorld));
        return Normalize(tcpWorld.rotation) * Vector3.up;
    }

    private static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private static Quaternion Normalize(Quaternion q)
    {
        double magnitude = Math.Sqrt((double)q.x * q.x + (double)q.y * q.y +
            (double)q.z * q.z + (double)q.w * q.w);
        return new Quaternion((float)(q.x / magnitude), (float)(q.y / magnitude),
            (float)(q.z / magnitude), (float)(q.w / magnitude));
    }

    private static void RequireValid(Pose pose, string parameterName)
    {
        if (!IsValidPose(pose))
            throw new ArgumentException("A pose needs finite coordinates and a nonzero quaternion.", parameterName);
    }
}
