using System;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Moveit;
using RosMessageTypes.MoveitTaskConstructor;
using RosMessageTypes.Std;
using RosMessageTypes.Trajectory;

/// <summary>Build the motion preview exclusively from the identified aggregate solution.</summary>
public static class FixtureSolutionPreview
{
    /// <summary>Concatenate every motion segment. IO, waits and scene-only steps have no motion.</summary>
    public static RobotTrajectoryMsg Flatten(SolutionMsg solution)
    {
        Require(solution != null && FixtureTilingGeometry.ValidToken(solution.task_id), "The batch solution has no valid plan ID.");
        Require(solution.sub_trajectory != null && solution.sub_trajectory.Length > 0, "The batch solution has no ordered steps.");
        var output = new RobotTrajectoryMsg();
        var points = new List<JointTrajectoryPointMsg>();
        string[] names = null;
        long offset = 0, previousTime = -1;
        double[] previousPositions = null;
        foreach (SubTrajectoryMsg segment in solution.sub_trajectory)
        {
            Require(segment?.trajectory != null, "A batch step has no trajectory container.");
            RobotTrajectoryMsg robot = segment.trajectory;
            Require(robot.multi_dof_joint_trajectory?.points == null || robot.multi_dof_joint_trajectory.points.Length == 0,
                "A multi-DOF batch trajectory cannot be displayed by this robot preview.");
            JointTrajectoryMsg joint = robot.joint_trajectory;
            Require(joint != null && joint.points != null, "A batch step has an incomplete joint trajectory.");
            if (joint.points.Length == 0) continue;
            Require(joint.joint_names != null && joint.joint_names.Length > 0, "A motion segment has no joint names.");
            var indices = new Dictionary<string, int>();
            for (int index = 0; index < joint.joint_names.Length; index++)
                Require(!string.IsNullOrEmpty(joint.joint_names[index]) && indices.TryAdd(joint.joint_names[index], index),
                    "A motion segment has missing or duplicate joint names.");
            if (names == null)
            {
                names = (string[])joint.joint_names.Clone();
                output.joint_trajectory.joint_names = names;
                Require(joint.header?.stamp != null, "A motion segment has no header.");
                output.joint_trajectory.header = new HeaderMsg {
                    frame_id = joint.header.frame_id,
                    stamp = new TimeMsg(joint.header.stamp.sec, joint.header.stamp.nanosec)
                };
            }
            Require(indices.Count == names.Length, "Motion segments do not contain the same joints.");
            var reorder = new int[names.Length];
            for (int index = 0; index < names.Length; index++)
                Require(indices.TryGetValue(names[index], out reorder[index]), "A motion segment omits a required joint.");
            long localPrevious = -1, localLast = 0;
            foreach (JointTrajectoryPointMsg point in joint.points)
            {
                Require(point != null && point.positions != null && point.positions.Length == names.Length,
                    "A motion waypoint omits joint positions.");
                Require(point.time_from_start != null && point.time_from_start.sec >= 0 && point.time_from_start.nanosec < 1000000000,
                    "A motion waypoint has an invalid time.");
                ValidateOptional(point.velocities, names.Length); ValidateOptional(point.accelerations, names.Length); ValidateOptional(point.effort, names.Length);
                long localTime = point.time_from_start.sec * 1000000000L + point.time_from_start.nanosec;
                Require(localTime >= localPrevious, "A motion segment runs backward in time.");
                long time = checked(offset + localTime);
                Require(time / 1000000000L <= int.MaxValue, "The batch preview exceeds the ROS duration range.");
                var positions = new double[names.Length];
                for (int index = 0; index < names.Length; index++)
                {
                    double value = point.positions[reorder[index]];
                    Require(Finite(value), "A motion waypoint has a nonfinite joint position.");
                    Require(previousTime != time || Math.Abs(previousPositions[index]-value) <= 0.001,
                        "A batch seam jumps between joint positions at the same time.");
                    positions[index] = value;
                }
                points.Add(new JointTrajectoryPointMsg {
                    positions = positions,
                    time_from_start = new DurationMsg((int)(time / 1000000000L), (uint)(time % 1000000000L))
                });
                previousTime = time; previousPositions = positions;
                localPrevious = localLast = localTime;
            }
            offset = checked(offset + localLast);
        }
        Require(points.Count > 0, "The batch solution contains no robot motion to review.");
        output.joint_trajectory.points = points.ToArray();
        return output;
    }

    private static void ValidateOptional(double[] values, int joints)
    {
        Require(values != null && (values.Length == 0 || values.Length == joints), "A motion waypoint has incomplete derivative/effort data.");
        foreach (double value in values) Require(Finite(value), "A motion waypoint has nonfinite derivative/effort data.");
    }
    private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    private static void Require(bool condition, string reason) { if (!condition) throw new ArgumentException(reason); }
}
