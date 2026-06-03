using System;
using UnityEngine;
using RosMessageTypes.Moveit;
using RosMessageTypes.Trajectory;
using System.Collections.Generic;

[Serializable]
public class UR16eEndPoseVisualizer : MonoBehaviour
{
    [Tooltip("A UR16e prefab imported via URDF Importer, set up with ArticulationBodies.")]
    public GameObject ur16ePrefab;

    [Tooltip("Uniform scale to apply to the instantiated UR16e.")]
    public float scale = 1.0f;

    private static readonly Dictionary<string, string> RosNameToUnityName = new Dictionary<string, string>()
    {
        { "shoulder_pan_joint",   "world/base_link/base_link_inertia/shoulder_link" },
        { "shoulder_lift_joint",  "world/base_link/base_link_inertia/shoulder_link/upper_arm_link" },
        { "elbow_joint",          "world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link" },
        { "wrist_1_joint",        "world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link" },
        { "wrist_2_joint",        "world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link" },
        { "wrist_3_joint",        "world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link" }
    };

    // You still need the mapping from ROS joint-names to the correct ArticulationBody index in the prefab
    // (exactly as before). For simplicity, this example assumes the prefab’s joint‐GameObject names
    // exactly match the ROS joint names, so we can do a Find() rather than an index‐array.
    //
    // If you instead have an array m_JointArticulationBodies[], you can reuse the Dictionary pattern
    // from before to convert jointName→arrayIndex, and then drive m_JointArticulationBodies[index].
    //

    /// <summary>
    /// Call this once you have a RobotTrajectoryMsg back and only want the final pose.
    /// </summary>
    public void VisualizeFinalPose(RobotTrajectoryMsg trajectoryMsg)
    {
        if (ur16ePrefab == null)
        {
            Debug.LogError("[UR16eEndPoseVisualizer] ur16ePrefab is null. Assign in Inspector.");
            return;
        }

        // Grab the joint_trajectory from the MoveIt message
        var jointTrajectory = trajectoryMsg.joint_trajectory;
        string[] jointNames = jointTrajectory.joint_names;
        JointTrajectoryPointMsg[] points = jointTrajectory.points;

        if (jointNames == null || jointNames.Length == 0)
        {
            Debug.LogError("[UR16eEndPoseVisualizer] Received empty joint_names array.");
            return;
        }
        if (points == null || points.Length == 0)
        {
            Debug.LogWarning("[UR16eEndPoseVisualizer] Received zero waypoints.");
            return;
        }

        // Take only the last waypoint:
        int lastIndex = points.Length - 1;
        JointTrajectoryPointMsg lastPoint = points[lastIndex];
        double[] finalPositions = lastPoint.positions;

        if (finalPositions == null || finalPositions.Length != jointNames.Length)
        {
            Debug.LogError(
                $"[UR16eEndPoseVisualizer] Last waypoint has {finalPositions?.Length} positions, " +
                $"but joint_names.Length = {jointNames.Length}.");
            return;
        }

        // 1) Instantiate a single UR16e clone
        GameObject instance = Instantiate(ur16ePrefab, Vector3.zero, Quaternion.identity);
        instance.transform.localScale = Vector3.one * scale;

        // 2) For each joint: find its ArticulationBody by name and set its target angle
        foreach (string rosJointName in jointNames)
        {
            string unityName = RosNameToUnityName[rosJointName];
            //Debug.Log($"Unity name for joint {rosJointName}: {unityName}");
            // Find the child GameObject whose name matches the ROS joint (exact match required).
            // E.g. rosJointName == "shoulder_pan_joint" → find "shoulder_pan_joint" in the prefab hierarchy.
            Transform jointTf = instance.transform.Find(unityName);
            if (jointTf == null)
            {
                Debug.LogWarning($"[UR16eEndPoseVisualizer] Could not find child Transform '{unityName}'.");
                continue;
            }

            ArticulationBody ab = jointTf.GetComponent<ArticulationBody>();
            if (ab == null)
            {
                Debug.LogWarning($"[UR16eEndPoseVisualizer] No ArticulationBody on '{unityName}'.");
                continue;
            }

            // Get index in jointNames to pull the correct angle:
            int idx = Array.IndexOf(jointNames, rosJointName);
            if (idx < 0 || idx >= finalPositions.Length)
            {
                Debug.LogWarning($"[UR16eEndPoseVisualizer] '{unityName}' not found in jointNames array.");
                continue;
            }

            // Convert rad→deg and apply
            float angleDeg = (float)(finalPositions[idx] * Mathf.Rad2Deg);
            var drive = ab.xDrive;
            drive.target = angleDeg;
            ab.xDrive = drive;
        }

        Debug.Log($"[UR16eEndPoseVisualizer] Instantiated final‐pose UR16e at waypoint #{lastIndex}.");
    }
}