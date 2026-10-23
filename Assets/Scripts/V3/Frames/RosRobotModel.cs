using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public sealed class RosRobotModelCatalog
{
    public int schemaVersion;
    public string baseLink;
    public RosModelSource[] sourceFiles;
    public RosRobotModel[] models;

    public RosRobotModel Find(string tool)
    {
        if (models != null) foreach (RosRobotModel model in models) if (model.tool == tool) return model;
        return null;
    }

    public static RosRobotModelCatalog Load()
    {
        TextAsset data = Resources.Load<TextAsset>("RobotModels/Models");
        if (data == null) throw new InvalidOperationException("The ROS robot-model catalog is missing.");
        RosRobotModelCatalog catalog = JsonUtility.FromJson<RosRobotModelCatalog>(data.text);
        if (catalog == null || catalog.schemaVersion != 1 || catalog.baseLink != "ur_base_link" ||
            catalog.Find("none") == null || catalog.Find("kenos") == null || catalog.Find("kenos_sl") == null)
            throw new InvalidOperationException("The ROS robot-model catalog is invalid.");
        foreach (RosRobotModel model in catalog.models) RosRobotModelMath.Validate(model);
        return catalog;
    }
}

[Serializable]
public sealed class RosModelSource { public string path; public string sha256; }

[Serializable]
public sealed class RosRobotModel
{
    public string tool;
    public string calibrationHash;
    public RosRobotLink[] links;
    public RosRobotGoldenCase[] goldenCases;

    public RosRobotLink FindLink(string name)
    {
        if (links != null) foreach (RosRobotLink link in links) if (link.name == name) return link;
        return null;
    }
}

[Serializable]
public sealed class RosRobotLink
{
    public string name;
    public string parent;
    public string jointName;
    public string jointType;
    public float[] position;
    public float[] rotation;
    public float[] axis;
    public float lower;
    public float upper;
    public RosRobotGeometry[] visuals;
    public RosRobotGeometry[] collisions;
}

[Serializable]
public sealed class RosRobotGeometry
{
    public string kind;
    public float[] position;
    public float[] rotation;
    public float[] size;
    public float radius;
    public float length;
    public string meshUri;
    public float[] scale;
    public float[] color;
}

[Serializable]
public sealed class RosRobotGoldenCase
{
    public string name;
    public string[] jointNames;
    public double[] jointPositions;
    public RosRobotLinkPose[] links;
}

[Serializable]
public sealed class RosRobotLinkPose { public string name; public float[] position; public float[] rotation; }

/// <summary>ROS FLU metres/radians to Unity RUF. Unity rotations reverse handedness.</summary>
public static class RosRobotModelMath
{
    public static Vector3 FromRosVector(float[] value)
    {
        if (value == null || value.Length != 3) throw new ArgumentException("Expected a ROS vector with three values.");
        Vector3 vector = new Vector3(-value[1], value[2], value[0]);
        if (!Finite(vector.x) || !Finite(vector.y) || !Finite(vector.z)) throw new ArgumentException("Nonfinite ROS vector.");
        return vector;
    }

    public static Quaternion FromRosRotation(float[] value)
    {
        if (value == null || value.Length != 4) throw new ArgumentException("Expected a ROS quaternion with four values.");
        Quaternion rotation = new Quaternion(-value[1], value[2], value[0], -value[3]);
        if (!RobotFrameMath.IsValidPose(new Pose(Vector3.zero, rotation))) throw new ArgumentException("Invalid ROS quaternion.");
        return rotation.normalized;
    }

    public static Pose LocalPose(RosRobotLink link) => new Pose(FromRosVector(link.position), FromRosRotation(link.rotation));
    public static Pose GeometryPose(RosRobotGeometry geometry) => new Pose(FromRosVector(geometry.position), FromRosRotation(geometry.rotation));
    public static Vector3 Axis(RosRobotLink link) => FromRosVector(link.axis).normalized;
    public static float[] ToRosVector(Vector3 value) => new[] { value.z, -value.x, value.y };
    public static float[] ToRosRotation(Quaternion value) => new[] { value.z, -value.x, value.y, -value.w };

    public static Dictionary<string, Pose> ForwardKinematics(RosRobotModel model, IDictionary<string, double> jointPositions = null)
    {
        Validate(model);
        Dictionary<string, Pose> poses = new Dictionary<string, Pose>();
        foreach (RosRobotLink link in model.links)
        {
            Pose local = LocalPose(link);
            if (IsRevolute(link) && jointPositions != null && jointPositions.TryGetValue(link.jointName, out double radians))
            {
                if (double.IsNaN(radians) || double.IsInfinity(radians)) throw new ArgumentException("Invalid joint angle.");
                local.rotation = local.rotation * Quaternion.AngleAxis((float)(-radians * Mathf.Rad2Deg), Axis(link));
            }
            poses.Add(link.name, string.IsNullOrEmpty(link.parent) ? local : RobotFrameMath.Compose(poses[link.parent], local));
        }
        return poses;
    }

    public static bool IsRevolute(RosRobotLink link) => link.jointType == "revolute" || link.jointType == "continuous";

    public static void Validate(RosRobotModel model)
    {
        if (model == null || model.links == null || model.links.Length == 0) throw new ArgumentException("Robot model has no links.");
        HashSet<string> names = new HashSet<string>();
        HashSet<string> joints = new HashSet<string>();
        foreach (RosRobotLink link in model.links)
        {
            if (link == null || string.IsNullOrEmpty(link.name) || !names.Add(link.name)) throw new ArgumentException("Duplicate or empty robot link.");
            LocalPose(link);
            if (string.IsNullOrEmpty(link.parent))
            {
                if (names.Count != 1 || link.name != "ur_base_link" ||
                    FromRosVector(link.position).sqrMagnitude > 1e-12f ||
                    Quaternion.Angle(FromRosRotation(link.rotation), Quaternion.identity) > 0.001f)
                    throw new ArgumentException("Robot model must start at ur_base_link with an identity pose.");
            }
            else
            {
                if (link.parent == link.name || !names.Contains(link.parent) || string.IsNullOrEmpty(link.jointName) || !joints.Add(link.jointName))
                    throw new ArgumentException("Robot links must form a topologically ordered tree with unique joints.");
                if (link.jointType != "fixed" && !IsRevolute(link)) throw new ArgumentException("Unsupported robot joint type.");
                if (IsRevolute(link) && FromRosVector(link.axis).sqrMagnitude < 1e-10f) throw new ArgumentException("Robot joint has no rotation axis.");
            }
            if (link.visuals != null) foreach (RosRobotGeometry visual in link.visuals) ValidateGeometry(visual);
            if (link.collisions != null) foreach (RosRobotGeometry collision in link.collisions) ValidateGeometry(collision);
        }
        if (!names.Contains("ur_flange") || !names.Contains("tcp")) throw new ArgumentException("Robot model must include flange and TCP frames.");
    }

    public static void ValidateGeometry(RosRobotGeometry geometry)
    {
        GeometryPose(geometry);
        switch (geometry.kind)
        {
            case "box":
                Vector3 size = FromRosVector(geometry.size);
                if (size.x >= 0f || size.y <= 0f || size.z <= 0f) throw new ArgumentException("Invalid box size.");
                break;
            case "cylinder":
                if (!Finite(geometry.length) || geometry.length <= 0f) throw new ArgumentException("Invalid cylinder length.");
                goto case "sphere";
            case "sphere":
                if (!Finite(geometry.radius) || geometry.radius <= 0f) throw new ArgumentException("Invalid sphere radius.");
                break;
            case "mesh":
                if (string.IsNullOrEmpty(geometry.meshUri)) throw new ArgumentException("Robot visual mesh has no URI.");
                if (geometry.scale != null)
                {
                    Vector3 scale = FromRosVector(geometry.scale);
                    if (scale.x >= 0f || scale.y <= 0f || scale.z <= 0f) throw new ArgumentException("Invalid mesh scale.");
                }
                break;
            default: throw new ArgumentException("Unsupported robot visual geometry.");
        }
    }

    private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
}
