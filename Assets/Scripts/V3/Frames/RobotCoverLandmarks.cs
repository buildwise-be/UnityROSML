using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>A physical contact point, in metres in the ROS axes of one visual mesh.</summary>
[Serializable]
public sealed class RobotCoverLandmark
{
    public string displayName;
    public string linkName;
    public int visualIndex;
    public Vector3 pointInVisualRos;
    [Tooltip("Require this mesh suffix so landmarks are not silently reused with a different arm geometry.")]
    public string expectedMeshSuffix;
}

/// <summary>UR16e blue-cover surface centres. Defaults are CAD-derived, not physical measurements.</summary>
public static class RobotCoverLandmarks
{
    /// <summary>
    /// Order follows the supplied photo from base to tool: shoulder, elbow, wrist 2, wrist 3.
    /// The wrist 1 cover is on the opposite side of the forearm and is omitted.
    /// Both large covers belong to the upper-arm visual; the small covers belong to the
    /// wrist-1 and wrist-2 visuals. Joint origins are not the contact surfaces.
    /// See Tools/RobotFrames/controller_point_landmarks.md for the mesh derivation.
    /// </summary>
    public static RobotCoverLandmark[] CreateDefaults() => new[]
    {
        Cover("Shoulder", "ur_upper_arm_link", new Vector3(-0.000001f, 0.087242f, -0.000297f), "/ur16e/visual/upperarm.dae"),
        Cover("Elbow", "ur_upper_arm_link", new Vector3(0f, 0.065129f, 0.477982f), "/ur16e/visual/upperarm.dae"),
        Cover("Wrist 2", "ur_wrist_1_link", new Vector3(-0.000001f, 0.135142f, -0.054024f), "/ur10e/visual/wrist1.dae"),
        Cover("Wrist 3", "ur_wrist_2_link", new Vector3(0.000001f, -0.053495f, 0.119459f), "/ur10e/visual/wrist2.dae")
    };

    private static RobotCoverLandmark Cover(string name, string link, Vector3 point, string mesh) =>
        new RobotCoverLandmark { displayName = name, linkName = link, visualIndex = 0,
            pointInVisualRos = point, expectedMeshSuffix = mesh };

    /// <summary>Uses the current model's calibrated FK and visual origins, never a physics-frame snapshot.</summary>
    public static bool TryGetBasePoints(RosRobotModel model, IDictionary<string, double> joints,
        RobotCoverLandmark[] landmarks, out Vector3[] points, out string error)
    {
        points = null;
        error = null;
        if (landmarks == null || landmarks.Length < 3)
        { error = "At least three cover landmarks are required."; return false; }
        try
        {
            RosRobotModelMath.Validate(model);
            foreach (RosRobotLink link in model.links)
            {
                if (!RosRobotModelMath.IsRevolute(link)) continue;
                if (joints == null || !joints.TryGetValue(link.jointName, out double angle) ||
                    double.IsNaN(angle) || double.IsInfinity(angle))
                { error = "Complete finite joint feedback is required: " + link.jointName + "."; return false; }
            }
            Dictionary<string, Pose> poses = RosRobotModelMath.ForwardKinematics(model, joints);
            Vector3[] result = new Vector3[landmarks.Length];
            for (int index = 0; index < landmarks.Length; index++)
            {
                RobotCoverLandmark landmark = landmarks[index];
                if (landmark == null || string.IsNullOrEmpty(landmark.linkName) ||
                    !RobotFrameMath.IsValidPose(new Pose(landmark.pointInVisualRos, Quaternion.identity)))
                { error = "A cover landmark is missing or invalid."; return false; }
                RosRobotLink link = model.FindLink(landmark.linkName);
                if (link == null || link.visuals == null || landmark.visualIndex < 0 ||
                    landmark.visualIndex >= link.visuals.Length)
                { error = "Cover visual is unavailable: " + landmark.linkName + "."; return false; }
                RosRobotGeometry geometry = link.visuals[landmark.visualIndex];
                string mesh = (geometry.meshUri ?? "").Replace('\\', '/');
                if (geometry.kind != "mesh" || (!string.IsNullOrEmpty(landmark.expectedMeshSuffix) &&
                    !mesh.EndsWith(landmark.expectedMeshSuffix, StringComparison.OrdinalIgnoreCase)))
                { error = "Cover landmark does not match the current arm mesh: " + landmark.displayName + "."; return false; }
                Vector3 point = landmark.pointInVisualRos;
                if (geometry.scale != null)
                    point = Vector3.Scale(point, new Vector3(geometry.scale[0], geometry.scale[1], geometry.scale[2]));
                Vector3 inVisual = RosRobotModelMath.FromRosVector(new[] { point.x, point.y, point.z });
                Pose visualInBase = RobotFrameMath.Compose(poses[link.name], RosRobotModelMath.GeometryPose(geometry));
                result[index] = visualInBase.position + visualInBase.rotation * inVisual;
                if (!RobotFrameMath.IsValidPose(new Pose(result[index], Quaternion.identity)))
                { error = "Cover position is invalid: " + landmark.displayName + "."; return false; }
            }
            points = result;
            return true;
        }
        catch (ArgumentException exception)
        {
            error = "Cover positions could not be calculated: " + exception.Message;
            return false;
        }
    }
}
