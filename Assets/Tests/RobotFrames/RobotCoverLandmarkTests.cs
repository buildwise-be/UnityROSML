using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Xml;
using NUnit.Framework;
using UnityEngine;

public sealed class RobotCoverLandmarkTests
{
    [Test]
    public void DefaultsUseTheVisibleCoverSurfacesRatherThanJointOrigins()
    {
        RobotCoverLandmark[] landmarks = RobotCoverLandmarks.CreateDefaults();
        CollectionAssert.AreEqual(new[] { "Shoulder", "Elbow", "Wrist 2", "Wrist 3" }, landmarks.Select(point => point.displayName));
        CollectionAssert.AreEqual(new[] { "ur_upper_arm_link", "ur_upper_arm_link", "ur_wrist_1_link", "ur_wrist_2_link" },
            landmarks.Select(point => point.linkName));
        // Defaults must be fresh inspector-editable data, not a shared mutable array.
        landmarks[0].pointInVisualRos = Vector3.zero;
        Assert.That(RobotCoverLandmarks.CreateDefaults()[0].pointInVisualRos.magnitude, Is.GreaterThan(0.08f));
    }

    [TestCase(0, "ur16e/visual/upperarm.dae", "eSeries_UR10e_004", 1, true)]
    [TestCase(1, "ur16e/visual/upperarm.dae", "eSeries_UR10e_005", 1, true)]
    [TestCase(2, "ur10e/visual/wrist1.dae", "eSeries_UR10e_021", 2, false)]
    [TestCase(3, "ur10e/visual/wrist2.dae", "eSeries_UR10e_030", 1, false)]
    public void DefaultContactIsOnTheActualBlueMeshOuterSurface(int index, string meshPath,
        string nodeId, int normalAxis, bool outwardPositive)
    {
        // Read source CAD geometry independently of Unity's FK and mesh importer.
        string path = Path.Combine(Application.dataPath,
            "Prefabs/ur16e_kenos/meshes/ur_description/meshes", meshPath);
        XmlDocument document = new XmlDocument();
        document.Load(path);
        XmlNamespaceManager ns = new XmlNamespaceManager(document.NameTable);
        ns.AddNamespace("c", "http://www.collada.org/2005/11/COLLADASchema");
        XmlNode node = document.SelectSingleNode("//c:visual_scene/c:node[@id='" + nodeId + "']", ns);
        Assert.That(node, Is.Not.Null);
        Assert.That(node.SelectSingleNode(".//c:instance_material[contains(@target,'URBlue')]", ns), Is.Not.Null);
        string geometryId = node.SelectSingleNode("c:instance_geometry", ns).Attributes["url"].Value.Substring(1);
        XmlNode mesh = document.SelectSingleNode("//c:geometry[@id='" + geometryId + "']/c:mesh", ns);
        string sourceId = mesh.SelectSingleNode("c:vertices/c:input[@semantic='POSITION']", ns).Attributes["source"].Value.Substring(1);
        double[] source = Numbers(mesh.SelectSingleNode("c:source[@id='" + sourceId + "']/c:float_array", ns).InnerText);
        double[] matrix = Numbers(node.SelectSingleNode("c:matrix", ns).InnerText);
        double[][] vertices = new double[source.Length / 3][];
        for (int vertex = 0; vertex < vertices.Length; vertex++)
        {
            vertices[vertex] = new double[3];
            for (int row = 0; row < 3; row++)
                vertices[vertex][row] = matrix[row * 4 + 3] + matrix[row * 4] * source[vertex * 3] +
                    matrix[row * 4 + 1] * source[vertex * 3 + 1] + matrix[row * 4 + 2] * source[vertex * 3 + 2];
        }
        Vector3 point = RobotCoverLandmarks.CreateDefaults()[index].pointInVisualRos;
        double outer = outwardPositive ? double.NegativeInfinity : double.PositiveInfinity;
        foreach (XmlNode primitive in mesh.ChildNodes)
        {
            if (primitive.LocalName != "triangles" && primitive.LocalName != "polylist") continue;
            XmlNode[] inputs = primitive.SelectNodes("c:input", ns).Cast<XmlNode>().ToArray();
            int stride = inputs.Max(input => int.Parse(input.Attributes["offset"].Value, CultureInfo.InvariantCulture)) + 1;
            int offset = int.Parse(inputs.Single(input => input.Attributes["semantic"].Value == "VERTEX").Attributes["offset"].Value,
                CultureInfo.InvariantCulture);
            int[] packed = Numbers(primitive.SelectSingleNode("c:p", ns).InnerText).Select(value => (int)value).ToArray();
            int[] indices = Enumerable.Range(0, packed.Length / stride).Select(value => packed[value * stride + offset]).ToArray();
            XmlNode countsNode = primitive.SelectSingleNode("c:vcount", ns);
            int[] counts = countsNode == null ? Enumerable.Repeat(3, indices.Length / 3).ToArray() :
                Numbers(countsNode.InnerText).Select(value => (int)value).ToArray();
            int start = 0;
            foreach (int count in counts)
            {
                for (int corner = 1; corner < count - 1; corner++)
                {
                    if (!TryIntersect(vertices[indices[start]], vertices[indices[start + corner]],
                        vertices[indices[start + corner + 1]], point, normalAxis, out double distance)) continue;
                    outer = outwardPositive ? Math.Max(outer, distance) : Math.Min(outer, distance);
                }
                start += count;
            }
        }
        Assert.That(double.IsInfinity(outer), Is.False, "Contact ray must intersect the cover's blue CAD surface.");
        Assert.That(Math.Abs(outer - point[normalAxis]), Is.LessThan(0.00002), "Default must touch the exterior face within 0.02 mm.");
    }

    [Test]
    public void MissingFeedbackAndDifferentArmMeshesCannotSilentlyProduceLandmarks()
    {
        RosRobotModel model = RosRobotModelCatalog.Load().Find("none");
        Dictionary<string, double> joints = HomeJoints(model);
        joints.Remove("ur_wrist_3_joint");
        Assert.That(RobotCoverLandmarks.TryGetBasePoints(model, joints, RobotCoverLandmarks.CreateDefaults(), out _, out string error), Is.False);
        StringAssert.Contains("ur_wrist_3_joint", error);
        joints = HomeJoints(model);
        RobotCoverLandmark[] landmarks = RobotCoverLandmarks.CreateDefaults();
        landmarks[0].expectedMeshSuffix = "/different_robot.dae";
        Assert.That(RobotCoverLandmarks.TryGetBasePoints(model, joints, landmarks, out _, out error), Is.False);
        StringAssert.Contains("mesh", error);
    }

    [Test]
    public void ToolChangesKeepArmLandmarksAndWristMotionDoesNotMoveUpperArmCovers()
    {
        RosRobotModelCatalog catalog = RosRobotModelCatalog.Load();
        RosRobotModel model = catalog.Find("none");
        Dictionary<string, double> joints = HomeJoints(model);
        RobotCoverLandmark[] landmarks = RobotCoverLandmarks.CreateDefaults();
        Assert.That(RobotCoverLandmarks.TryGetBasePoints(model, joints, landmarks, out Vector3[] original, out string error), Is.True, error);
        foreach (string tool in new[] { "kenos", "kenos_sl" })
        {
            Assert.That(RobotCoverLandmarks.TryGetBasePoints(catalog.Find(tool), joints, landmarks, out Vector3[] other, out error), Is.True, error);
            for (int i = 0; i < original.Length; i++) Assert.That(Vector3.Distance(original[i], other[i]), Is.LessThan(0.000001f));
        }
        joints["ur_wrist_1_joint"] += 0.7;
        Assert.That(RobotCoverLandmarks.TryGetBasePoints(model, joints, landmarks, out Vector3[] moved, out error), Is.True, error);
        Assert.That(Vector3.Distance(original[0], moved[0]), Is.LessThan(0.000001f));
        Assert.That(Vector3.Distance(original[1], moved[1]), Is.LessThan(0.000001f));
        Assert.That(Vector3.Distance(original[2], moved[2]), Is.GreaterThan(0.02f));
        Assert.That(Vector3.Distance(original[3], moved[3]), Is.GreaterThan(0.02f));
    }

    private static Dictionary<string, double> HomeJoints(RosRobotModel model)
    {
        RosRobotGoldenCase home = model.goldenCases.Single(sample => sample.name == "home");
        return home.jointNames.Select((name, index) => new { name, value = home.jointPositions[index] })
            .ToDictionary(joint => joint.name, joint => joint.value);
    }

    private static double[] Numbers(string text) => text.Split((char[])null, StringSplitOptions.RemoveEmptyEntries)
        .Select(value => double.Parse(value, CultureInfo.InvariantCulture)).ToArray();

    private static bool TryIntersect(double[] a, double[] b, double[] c, Vector3 point, int normalAxis, out double value)
    {
        int first = normalAxis == 0 ? 1 : 0;
        int second = normalAxis == 2 ? 1 : 2;
        double denominator = (b[second] - c[second]) * (a[first] - c[first]) +
            (c[first] - b[first]) * (a[second] - c[second]);
        value = 0;
        if (Math.Abs(denominator) < 1e-15) return false;
        double u = ((b[second] - c[second]) * (point[first] - c[first]) +
            (c[first] - b[first]) * (point[second] - c[second])) / denominator;
        double v = ((c[second] - a[second]) * (point[first] - c[first]) +
            (a[first] - c[first]) * (point[second] - c[second])) / denominator;
        double w = 1 - u - v;
        if (Math.Min(u, Math.Min(v, w)) < -1e-7) return false;
        value = u * a[normalAxis] + v * b[normalAxis] + w * c[normalAxis];
        return true;
    }
}
