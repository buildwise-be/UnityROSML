using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Xml;
using UnityEngine;

/// <summary>Read the expanded description actually loaded by ROS, including custom TCP parameters.</summary>
public static class RosRobotDescription
{
    public static bool TryParse(string xml, out RosRobotModel model, out string error)
    {
        model = null;
        error = null;
        try
        {
            if (string.IsNullOrWhiteSpace(xml)) throw new FormatException("ROS robot description is empty.");
            XmlDocument document = new XmlDocument { XmlResolver = null };
            XmlReaderSettings settings = new XmlReaderSettings
            {
                DtdProcessing = DtdProcessing.Prohibit, XmlResolver = null, MaxCharactersInDocument = 4000000
            };
            using (StringReader input = new StringReader(xml))
            using (XmlReader reader = XmlReader.Create(input, settings)) document.Load(reader);
            XmlElement robot = document.DocumentElement;
            if (robot == null || robot.Name != "robot") throw new FormatException("Expected an expanded URDF robot.");
            Dictionary<string, XmlElement> links = new Dictionary<string, XmlElement>();
            Dictionary<string, List<XmlElement>> children = new Dictionary<string, List<XmlElement>>();
            Dictionary<string, float[]> colors = new Dictionary<string, float[]>();
            foreach (XmlNode node in robot.ChildNodes)
            {
                if (!(node is XmlElement element)) continue;
                if (element.Name == "link") links.Add(element.GetAttribute("name"), element);
                else if (element.Name == "joint")
                {
                    string parent = RequiredChild(element, "parent").GetAttribute("link");
                    if (!children.TryGetValue(parent, out List<XmlElement> joints)) children.Add(parent, joints = new List<XmlElement>());
                    joints.Add(element);
                }
                else if (element.Name == "material" && element["color"] != null)
                    colors[element.GetAttribute("name")] = Numbers(element["color"].GetAttribute("rgba"), 4, null);
            }
            if (!links.ContainsKey("ur_base_link")) throw new FormatException("ROS model has no ur_base_link.");
            List<RosRobotLink> ordered = new List<RosRobotLink>();
            AddLink("ur_base_link", null, "", links, children, colors, ordered, new HashSet<string>());
            string tool = ordered.Exists(link => link.name == "kenos_sl_base") ? "kenos_sl" :
                ordered.Exists(link => link.name == "kenos_base") ? "kenos" : "none";
            model = new RosRobotModel { tool = tool, links = ordered.ToArray() };
            RosRobotModelMath.Validate(model);
            if (tool == "none" && (model.FindLink("tcp").parent != "ur_flange" ||
                model.FindLink("tcp").jointName != "none_tool_joint"))
                throw new FormatException("Unsupported ROS tool: expected none, kenos or kenos_sl.");
            return true;
        }
        catch (Exception exception) when (exception is XmlException || exception is FormatException ||
            exception is ArgumentException || exception is InvalidOperationException)
        {
            error = exception.Message;
            model = null;
            return false;
        }
    }

    private static void AddLink(string name, XmlElement joint, string parent, Dictionary<string, XmlElement> links,
        Dictionary<string, List<XmlElement>> children, Dictionary<string, float[]> colors,
        List<RosRobotLink> ordered, HashSet<string> visited)
    {
        if (!visited.Add(name) || !links.TryGetValue(name, out XmlElement source)) throw new FormatException("Invalid URDF link tree.");
        ReadOrigin(joint?["origin"], out float[] position, out float[] rotation);
        XmlElement limit = joint?["limit"];
        RosRobotLink link = new RosRobotLink
        {
            name = name, parent = parent, jointName = joint?.GetAttribute("name") ?? "",
            jointType = joint?.GetAttribute("type") ?? "fixed", position = position, rotation = rotation,
            axis = Numbers(joint?["axis"]?.GetAttribute("xyz"), 3, new[] { 1f, 0f, 0f }),
            lower = Number(limit?.GetAttribute("lower"), 0f), upper = Number(limit?.GetAttribute("upper"), 0f),
            visuals = ReadGeometry(source, "visual", colors), collisions = ReadGeometry(source, "collision", colors)
        };
        ordered.Add(link);
        if (children.TryGetValue(name, out List<XmlElement> joints)) foreach (XmlElement childJoint in joints)
            AddLink(RequiredChild(childJoint, "child").GetAttribute("link"), childJoint, name,
                links, children, colors, ordered, visited);
    }

    private static RosRobotGeometry[] ReadGeometry(XmlElement link, string tag, Dictionary<string, float[]> colors)
    {
        List<RosRobotGeometry> result = new List<RosRobotGeometry>();
        foreach (XmlNode node in link.ChildNodes)
        {
            if (!(node is XmlElement visual) || visual.Name != tag) continue;
            XmlElement shape = null;
            foreach (XmlNode candidate in RequiredChild(visual, "geometry").ChildNodes)
                if (candidate is XmlElement element)
                {
                    if (shape != null) throw new FormatException("URDF visual has multiple geometries.");
                    shape = element;
                }
            if (shape == null) throw new FormatException("URDF visual has no geometry.");
            ReadOrigin(visual["origin"], out float[] position, out float[] rotation);
            float[] color = new[] { 0.65f, 0.65f, 0.65f, 1f };
            XmlElement material = visual["material"];
            if (material?["color"] != null) color = Numbers(material["color"].GetAttribute("rgba"), 4, color);
            else if (material != null && colors.TryGetValue(material.GetAttribute("name"), out float[] namedColor)) color = namedColor;
            result.Add(new RosRobotGeometry
            {
                kind = shape.Name, position = position, rotation = rotation, color = color,
                size = shape.Name == "box" ? Numbers(shape.GetAttribute("size"), 3, null) : null,
                radius = Number(shape.GetAttribute("radius"), 0f), length = Number(shape.GetAttribute("length"), 0f),
                meshUri = shape.GetAttribute("filename"),
                scale = shape.Name == "mesh" ? Numbers(shape.GetAttribute("scale"), 3, new[] { 1f, 1f, 1f }) : null
            });
        }
        return result.ToArray();
    }

    private static XmlElement RequiredChild(XmlElement parent, string name) => parent[name] ?? throw new FormatException("Missing URDF " + name + ".");

    private static void ReadOrigin(XmlElement origin, out float[] position, out float[] rotation)
    {
        position = Numbers(origin?.GetAttribute("xyz"), 3, new[] { 0f, 0f, 0f });
        float[] rpy = Numbers(origin?.GetAttribute("rpy"), 3, new[] { 0f, 0f, 0f });
        double cr = Math.Cos(rpy[0] / 2.0), sr = Math.Sin(rpy[0] / 2.0);
        double cp = Math.Cos(rpy[1] / 2.0), sp = Math.Sin(rpy[1] / 2.0);
        double cy = Math.Cos(rpy[2] / 2.0), sy = Math.Sin(rpy[2] / 2.0);
        rotation = new[] { (float)(sr * cp * cy - cr * sp * sy), (float)(cr * sp * cy + sr * cp * sy),
            (float)(cr * cp * sy - sr * sp * cy), (float)(cr * cp * cy + sr * sp * sy) };
    }

    private static float[] Numbers(string text, int count, float[] fallback)
    {
        if (string.IsNullOrWhiteSpace(text)) return fallback ?? throw new FormatException("Missing URDF vector.");
        string[] parts = text.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
        if (parts.Length != count) throw new FormatException("Invalid URDF vector size.");
        float[] result = new float[count];
        for (int i = 0; i < count; i++) result[i] = Number(parts[i], 0f);
        return result;
    }

    private static float Number(string text, float fallback)
    {
        if (string.IsNullOrWhiteSpace(text)) return fallback;
        if (!float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out float number) ||
            float.IsNaN(number) || float.IsInfinity(number)) throw new FormatException("Invalid URDF number.");
        return number;
    }

    /// <summary>Compare every arm/tool link, axis and geometry through tcp, independent of XML formatting.</summary>
    public static bool Equivalent(RosRobotModel expected, RosRobotModel actual, out string reason)
    {
        reason = null;
        if (expected == null || actual == null || expected.tool != actual.tool || expected.links.Length != actual.links.Length)
        { reason = "Tool or link set differs."; return false; }
        foreach (RosRobotLink link in expected.links)
        {
            RosRobotLink other = actual.FindLink(link.name);
            if (other == null || link.parent != other.parent || link.jointName != other.jointName || link.jointType != other.jointType ||
                !SamePose(link.position, link.rotation, other.position, other.rotation) ||
                (RosRobotModelMath.IsRevolute(link) &&
                    (Vector3.Distance(RosRobotModelMath.Axis(link), RosRobotModelMath.Axis(other)) > 1e-6f ||
                    Mathf.Abs(link.lower - other.lower) > 1e-6f || Mathf.Abs(link.upper - other.upper) > 1e-6f)) ||
                !SameGeometry(link.visuals, other.visuals) || !SameGeometry(link.collisions, other.collisions))
            { reason = "ROS model differs at " + link.name + "."; return false; }
        }
        return true;
    }

    private static bool SamePose(float[] p, float[] q, float[] p2, float[] q2)
    {
        if (Vector3.Distance(RosRobotModelMath.FromRosVector(p), RosRobotModelMath.FromRosVector(p2)) > 1e-6f) return false;
        Quaternion a = RosRobotModelMath.FromRosRotation(q), b = RosRobotModelMath.FromRosRotation(q2);
        if (Quaternion.Dot(a, b) < 0f) b = new Quaternion(-b.x, -b.y, -b.z, -b.w);
        return new Vector4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w).sqrMagnitude < 1e-10f;
    }

    private static bool SameGeometry(RosRobotGeometry[] geometries, RosRobotGeometry[] others)
    {
        int count = geometries?.Length ?? 0;
        if (count != (others?.Length ?? 0)) return false;
        for (int i = 0; i < count; i++)
        {
            RosRobotGeometry a = geometries[i], b = others[i];
            if (a.kind != b.kind || !SamePose(a.position, a.rotation, b.position, b.rotation)) return false;
            if (a.kind == "box" && Vector3.Distance(RosRobotModelMath.FromRosVector(a.size), RosRobotModelMath.FromRosVector(b.size)) > 1e-6f) return false;
            if ((a.kind == "sphere" || a.kind == "cylinder") && Mathf.Abs(a.radius - b.radius) > 1e-6f) return false;
            if (a.kind == "cylinder" && Mathf.Abs(a.length - b.length) > 1e-6f) return false;
            if (a.kind == "mesh" && (MeshKey(a.meshUri) != MeshKey(b.meshUri) ||
                Vector3.Distance(RosRobotModelMath.FromRosVector(a.scale), RosRobotModelMath.FromRosVector(b.scale)) > 1e-6f)) return false;
        }
        return true;
    }

    private static string MeshKey(string uri)
    {
        string path = (uri ?? "").Replace('\\', '/');
        int meshes = path.IndexOf("/meshes/", StringComparison.Ordinal);
        return meshes < 0 ? path : path.Substring(meshes);
    }
}
