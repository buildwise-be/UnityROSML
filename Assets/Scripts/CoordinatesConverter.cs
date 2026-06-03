using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// DEPRECATED: This static utility class has been replaced by RobotCoordinateService.
/// Use RobotCoordinateService.Instance for coordinate conversions.
/// This file is kept for backward compatibility but should not be used in new code.
/// </summary>
[System.Obsolete("Use RobotCoordinateService instead")]
public static class CoordinatesConverter
{
    public static Dictionary<string, string> ROSFrameToUnityName = new Dictionary<string, string>
        {
            { "ur_shoulder_pan_joint", "shoulder_link" },
            { "ur_shoulder_lift_joint",  "upper_arm_link" },
            { "ur_elbow_joint",          "forearm_link" },
            { "ur_wrist_1_joint",        "wrist_1_link" },
            { "ur_wrist_2_joint",        "wrist_2_link" },
            { "ur_wrist_3_joint",        "wrist_3_link" }
        };

    /// <summary>
    /// Transforms a world pose to a pose relative to the robot root link (most likely (ur_)base_link).
    /// </summary>
    /// <param name="worldTransform">The world transform to be.. transformed!</param>
    /// <param name="robotRootArticulation">The root link to use.</param>
    /// <returns>The local transform, in root link frame.</returns>
    public static Pose ConvertWorldTransformToLocalPose(Transform worldTransform, ArticulationBody robotRootArticulation)
    {
        if (robotRootArticulation == null)
        {
            Debug.LogError("No root articulation body found!");
            return new Pose();
        }
        Vector3 rootRelativePos = robotRootArticulation.transform.InverseTransformPoint(worldTransform.position);
        Quaternion rootRelativeRot = Quaternion.Inverse(robotRootArticulation.transform.rotation) * (worldTransform.rotation);
        Pose pose = new Pose();
        pose.position = rootRelativePos;
        pose.rotation = rootRelativeRot;
        return pose;
    }

    /// <summary>
    /// Transforms a world pose to a pose relative to the robot root link (most likely (ur_)base_link).
    /// </summary>
    /// <param name="worldTransform">The world transform to be.. transformed!</param>
    /// <param name="frame">The frame to use as the local reference frame.</param>
    /// <returns>The local transform, in the local provided frame.</returns>
    public static Pose ConvertWorldTransformToLocalPose(Transform worldTransform, string frame)
    {
        var referenceTransform = GameObject.Find(frame).transform;
        if (referenceTransform == null)
        {
            Debug.LogError($"No object found with name {frame}");
            return new Pose();
        }
        Vector3 rootRelativePos = referenceTransform.InverseTransformPoint(worldTransform.position);
        Quaternion rootRelativeRot = Quaternion.Inverse(referenceTransform.rotation) * (worldTransform.rotation);
        Pose pose = new Pose();
        pose.position = rootRelativePos;
        pose.rotation = rootRelativeRot;
        return pose;
    }
}