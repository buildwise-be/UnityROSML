using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml;
using NUnit.Framework;
using UnityEngine;

public sealed class RosRobotModelTests
{
    private const float PositionTolerance = 0.00002f;
    private const float DirectionTolerance = 0.00002f;
    private static readonly string[] Tools = { "none", "kenos", "kenos_sl" };
    private static readonly string[] GoldenNames =
    {
        "zero", "home", "bent", "joint_1_quarter_turn", "joint_2_quarter_turn", "joint_3_quarter_turn",
        "joint_4_quarter_turn", "joint_5_quarter_turn", "joint_6_quarter_turn",
        "random_01", "random_02", "random_03", "random_04", "random_05", "random_06",
        "random_07", "random_08", "random_09", "random_10", "random_11", "random_12"
    };
    private RosRobotModelCatalog _catalog;

    [OneTimeSetUp]
    public void LoadCatalog()
    {
        _catalog = RosRobotModelCatalog.Load();
    }

    public static IEnumerable<TestCaseData> GoldenConfigurations()
    {
        // Keep Unity resource access out of NUnit discovery: it runs in test setup instead.
        foreach (string tool in Tools)
            foreach (string name in GoldenNames)
                yield return new TestCaseData(tool, name).SetName("ROS_FK_" + tool + "_" + name);
    }

    [Test]
    public void CatalogContainsAllThreeToolsAndExactlyThe63CheckedConfigurations()
    {
        CollectionAssert.AreEquivalent(Tools, _catalog.models.Select(model => model.tool).ToArray());
        foreach (RosRobotModel model in _catalog.models)
        {
            CollectionAssert.AreEquivalent(GoldenNames, model.goldenCases.Select(sample => sample.name).ToArray(), model.tool);
            Assert.That(model.links.Count(link => string.IsNullOrEmpty(link.parent)), Is.EqualTo(1), model.tool);
            Assert.That(model.links[0].name, Is.EqualTo("ur_base_link"));
            Assert.That(model.calibrationHash, Is.Not.Empty, model.tool);
        }
        Assert.That(_catalog.models.Sum(model => model.goldenCases.Length), Is.EqualTo(63));
    }

    [TestCaseSource(nameof(GoldenConfigurations))]
    public void ForwardKinematicsMatchesIndependentRosGoldensForEveryLink(string tool, string configuration)
    {
        RosRobotModel model = _catalog.Find(tool);
        RosRobotGoldenCase golden = model.goldenCases.Single(sample => sample.name == configuration);
        Assert.That(golden.jointNames.Length, Is.EqualTo(6));
        Assert.That(golden.jointPositions.Length, Is.EqualTo(6));
        Dictionary<string, double> joints = new Dictionary<string, double>();
        for (int i = 0; i < golden.jointNames.Length; i++) joints.Add(golden.jointNames[i], golden.jointPositions[i]);
        Dictionary<string, Pose> actual = RosRobotModelMath.ForwardKinematics(model, joints);
        CollectionAssert.AreEquivalent(model.links.Select(link => link.name).ToArray(), golden.links.Select(link => link.name).ToArray());
        Assert.That(actual.Count, Is.EqualTo(golden.links.Length));
        foreach (RosRobotLinkPose expected in golden.links)
        {
            string context = tool + "/" + configuration + "/" + expected.name;
            Pose pose = actual[expected.name];
            AssertVector(RosVector(expected.position), pose.position, PositionTolerance, context + " position");
            Quaternion rosRotation = new Quaternion(expected.rotation[0], expected.rotation[1], expected.rotation[2], expected.rotation[3]).normalized;
            // Independently map the physical ROS-rotated axes, rather than reusing FromRosRotation.
            AssertVector(RosVector(rosRotation * -Vector3.up), pose.rotation * Vector3.right, DirectionTolerance, context + " right");
            AssertVector(RosVector(rosRotation * Vector3.forward), pose.rotation * Vector3.up, DirectionTolerance, context + " up");
            AssertVector(RosVector(rosRotation * Vector3.right), pose.rotation * Vector3.forward, DirectionTolerance, context + " forward");
        }
    }

    [TestCase("none")]
    [TestCase("kenos")]
    [TestCase("kenos_sl")]
    public void ExpandedUrdfMatchesItsCatalogIncludingVisualsAndCollisions(string tool)
    {
        Assert.That(RosRobotDescription.TryParse(Urdf(tool), out RosRobotModel parsed, out string error), Is.True, error);
        Assert.That(parsed.tool, Is.EqualTo(tool));
        Assert.That(RosRobotDescription.Equivalent(_catalog.Find(tool), parsed, out string difference), Is.True, difference);
        Assert.That(RosRobotDescription.Equivalent(parsed, _catalog.Find(tool), out difference), Is.True, difference);
    }

    [TestCase("none")]
    [TestCase("kenos")]
    [TestCase("kenos_sl")]
    public void EquivalentQuaternionSignsDoNotCauseFalseModelMismatches(string tool)
    {
        RosRobotModel expected = _catalog.Find(tool);
        RosRobotModel actual = Clone(expected);
        foreach (RosRobotLink link in actual.links)
        {
            Negate(link.rotation);
            foreach (RosRobotGeometry geometry in link.visuals ?? Array.Empty<RosRobotGeometry>()) Negate(geometry.rotation);
            foreach (RosRobotGeometry geometry in link.collisions ?? Array.Empty<RosRobotGeometry>()) Negate(geometry.rotation);
        }
        Assert.That(RosRobotDescription.Equivalent(expected, actual, out string reason), Is.True, reason);
    }

    [Test]
    public void ChangedTcpOffsetCannotBeAcceptedAsTheSameModel()
    {
        RosRobotModel actual = Clone(_catalog.Find("kenos_sl"));
        actual.FindLink("tcp").position[2] += 0.01f;
        AssertMismatch(_catalog.Find("kenos_sl"), actual, "tcp");
    }

    [Test]
    public void ChangedJointAxisCannotBeAcceptedAsTheSameModel()
    {
        RosRobotModel actual = Clone(_catalog.Find("none"));
        actual.FindLink("ur_shoulder_link").axis = new[] { 1f, 0f, 0f };
        AssertMismatch(_catalog.Find("none"), actual, "ur_shoulder_link");
    }

    [TestCase(false)]
    [TestCase(true)]
    public void ChangedGeometryOriginCannotBeAcceptedAsTheSameModel(bool collision)
    {
        RosRobotModel actual = Clone(_catalog.Find("kenos_sl"));
        RosRobotLink tool = actual.FindLink("kenos_sl_base");
        (collision ? tool.collisions : tool.visuals)[0].position[0] += 0.01f;
        AssertMismatch(_catalog.Find("kenos_sl"), actual, "kenos_sl_base");
    }

    [Test]
    public void ChangedVisualMeshCannotBeAcceptedAsTheSameModel()
    {
        RosRobotModel actual = Clone(_catalog.Find("kenos"));
        actual.FindLink("kenos_base").visuals[0].meshUri = "package://kenos_flexigrip/meshes/visual/different_tool.dae";
        AssertMismatch(_catalog.Find("kenos"), actual, "kenos_base");
    }

    [TestCase(false)]
    [TestCase(true)]
    public void MissingOrExtraLinksCannotBeAcceptedAsTheSameModel(bool extra)
    {
        RosRobotModel expected = _catalog.Find("none");
        RosRobotModel actual = Clone(expected);
        actual.links = extra
            ? actual.links.Concat(new[] { FixedLink("unexpected_link", "ur_base_link", "unexpected_joint") }).ToArray()
            : actual.links.Where(link => link.name != "tcp").ToArray();
        Assert.That(RosRobotDescription.Equivalent(expected, actual, out string reason), Is.False);
        Assert.That(reason, Is.Not.Empty);
    }

    [Test]
    public void CustomSafeLightMountAndTcpParametersSurviveLiveUrdfParsing()
    {
        XmlDocument document = Document("kenos_sl");
        SetOrigin(document, "kenos_sl_tool_joint", "0 0 0.08", "0 0 1.5707963267948966");
        SetOrigin(document, "kenos_sl_tcp_fixed", "0.010 -0.020 0.100", "0 1.5707963267948966 0");
        foreach (XmlElement cylinder in document.SelectNodes("/robot/link[@name='kenos_sl_mount']/*/geometry/cylinder"))
        {
            cylinder.SetAttribute("length", "0.08");
            cylinder.SetAttribute("radius", "0.045");
        }
        foreach (XmlElement origin in document.SelectNodes("/robot/link[@name='kenos_sl_mount']/*/origin"))
            origin.SetAttribute("xyz", "0 0 0.04");
        Assert.That(RosRobotDescription.TryParse(document.OuterXml, out RosRobotModel model, out string error), Is.True, error);
        Assert.That(model.tool, Is.EqualTo("kenos_sl"));
        CollectionAssert.AreEqual(new[] { 0.01f, -0.02f, 0.1f }, model.FindLink("tcp").position);
        Dictionary<string, Pose> poses = RosRobotModelMath.ForwardKinematics(model);
        Pose flangeToTcp = RobotFrameMath.RelativeTo(poses["ur_flange"], poses["tcp"]);
        // Independent URDF composition: Ry(pi/2) * [Rz(pi/2) * (0.01,-0.02,0.1) + (0,0,0.08)].
        AssertVector(new Vector3(-0.01f, -0.02f, 0.18f), flangeToTcp.position, PositionTolerance, "custom flange TCP");
        AssertVector(Vector3.left, flangeToTcp.rotation * Vector3.up, DirectionTolerance, "custom ROS TCP +Z");
        Assert.That(RosRobotDescription.Equivalent(_catalog.Find("kenos_sl"), model, out _), Is.False);
    }

    [TestCase(null)]
    [TestCase("")]
    [TestCase("  ")]
    [TestCase("<robot>")]
    [TestCase("<not_a_robot/>")]
    [TestCase("<robot name='missing_base'><link name='tcp'/></robot>")]
    [TestCase("<!DOCTYPE robot [<!ENTITY x SYSTEM 'file:///not-a-real-model-file'>]><robot name='model'>&x;</robot>")]
    public void InvalidOrExternalEntityXmlIsRejectedWithoutReturningPartialModels(string xml)
    {
        AssertRejected(xml);
    }

    [TestCase("duplicate_link")]
    [TestCase("missing_child_link")]
    [TestCase("missing_parent_element")]
    [TestCase("missing_child_element")]
    [TestCase("cycle")]
    [TestCase("two_parents")]
    [TestCase("duplicate_joint")]
    [TestCase("unsupported_joint")]
    [TestCase("nonfinite_origin")]
    public void InvalidReachableUrdfTreesAreRejected(string defect)
    {
        XmlDocument document = new XmlDocument { XmlResolver = null };
        document.LoadXml(MinimalRobot);
        XmlElement tcpJoint = (XmlElement)document.SelectSingleNode("/robot/joint[@name='tcp_joint']");
        switch (defect)
        {
            case "duplicate_link": document.DocumentElement.AppendChild(document.SelectSingleNode("/robot/link[@name='tcp']").CloneNode(true)); break;
            case "missing_child_link": document.DocumentElement.RemoveChild(document.SelectSingleNode("/robot/link[@name='tcp']")); break;
            case "missing_parent_element": tcpJoint.RemoveChild(tcpJoint["parent"]); break;
            case "missing_child_element": tcpJoint.RemoveChild(tcpJoint["child"]); break;
            case "cycle": tcpJoint["child"].SetAttribute("link", "ur_base_link"); break;
            case "two_parents":
                XmlElement additional = (XmlElement)tcpJoint.CloneNode(true);
                additional.SetAttribute("name", "second_parent_joint");
                additional["parent"].SetAttribute("link", "ur_base_link");
                document.DocumentElement.AppendChild(additional);
                break;
            case "duplicate_joint": tcpJoint.SetAttribute("name", "flange_joint"); break;
            case "unsupported_joint": tcpJoint.SetAttribute("type", "prismatic"); break;
            case "nonfinite_origin":
                XmlElement origin = document.CreateElement("origin");
                origin.SetAttribute("xyz", "NaN 0 0");
                tcpJoint.AppendChild(origin);
                break;
        }
        AssertRejected(document.OuterXml);
    }

    [TestCase("missing_root")]
    [TestCase("second_root")]
    [TestCase("duplicate_link")]
    [TestCase("duplicate_joint")]
    [TestCase("self_parent")]
    [TestCase("parent_after_child")]
    [TestCase("zero_axis")]
    public void InvalidJsonModelTreesAreRejected(string defect)
    {
        RosRobotModel model = Clone(_catalog.Find("none"));
        RosRobotLink tcp = model.FindLink("tcp");
        switch (defect)
        {
            case "missing_root": model.links[0].name = "wrong_root"; break;
            case "second_root": tcp.parent = ""; break;
            case "duplicate_link": tcp.name = "ur_flange"; break;
            case "duplicate_joint": tcp.jointName = "ur_shoulder_pan_joint"; break;
            case "self_parent": tcp.parent = "tcp"; break;
            case "parent_after_child":
                RosRobotLink first = model.links[1]; model.links[1] = model.links[3]; model.links[3] = first;
                break;
            case "zero_axis": model.FindLink("ur_shoulder_link").axis = new[] { 0f, 0f, 0f }; break;
        }
        Assert.Throws<ArgumentException>(() => RosRobotModelMath.Validate(model));
    }

    [TestCase(false)]
    [TestCase(true)]
    public void InvalidVisualAndCollisionDimensionsAreRejected(bool collision)
    {
        XmlDocument document = Document("kenos_sl");
        XmlElement box = (XmlElement)document.SelectSingleNode("/robot/link[@name='kenos_sl_base']/" +
            (collision ? "collision" : "visual") + "/geometry/box");
        box.SetAttribute("size", "0.2 -0.06 0.032");
        AssertRejected(document.OuterXml);
    }

    [Test]
    public void NonfiniteJointFeedbackIsRejectedBeforeForwardKinematics()
    {
        Dictionary<string, double> invalid = new Dictionary<string, double> { { "ur_shoulder_pan_joint", double.NaN } };
        Assert.Throws<ArgumentException>(() => RosRobotModelMath.ForwardKinematics(_catalog.Find("none"), invalid));
        invalid["ur_shoulder_pan_joint"] = double.PositiveInfinity;
        Assert.Throws<ArgumentException>(() => RosRobotModelMath.ForwardKinematics(_catalog.Find("none"), invalid));
    }

    private static readonly string MinimalRobot = "<robot name='test'><link name='ur_base_link'/><link name='ur_flange'/><link name='tcp'/>" +
        "<joint name='flange_joint' type='fixed'><parent link='ur_base_link'/><child link='ur_flange'/></joint>" +
        "<joint name='tcp_joint' type='fixed'><parent link='ur_flange'/><child link='tcp'/></joint></robot>";

    private static RosRobotLink FixedLink(string name, string parent, string joint)
    {
        return new RosRobotLink { name = name, parent = parent, jointName = joint, jointType = "fixed",
            position = new[] { 0f, 0f, 0f }, rotation = new[] { 0f, 0f, 0f, 1f }, axis = new[] { 1f, 0f, 0f } };
    }

    private static string Urdf(string tool)
    {
        TextAsset asset = Resources.Load<TextAsset>("RobotModels/" + tool + ".urdf");
        Assert.That(asset, Is.Not.Null, "Missing expanded URDF resource for " + tool);
        return asset.text;
    }

    private static XmlDocument Document(string tool)
    {
        XmlDocument document = new XmlDocument { XmlResolver = null };
        document.LoadXml(Urdf(tool));
        return document;
    }

    private static void SetOrigin(XmlDocument document, string joint, string xyz, string rpy)
    {
        XmlElement origin = (XmlElement)document.SelectSingleNode("/robot/joint[@name='" + joint + "']/origin");
        Assert.That(origin, Is.Not.Null, joint);
        origin.SetAttribute("xyz", xyz);
        origin.SetAttribute("rpy", rpy);
    }

    private static RosRobotModel Clone(RosRobotModel model) => JsonUtility.FromJson<RosRobotModel>(JsonUtility.ToJson(model));
    private static Vector3 RosVector(float[] value) => new Vector3(-value[1], value[2], value[0]);
    private static Vector3 RosVector(Vector3 value) => new Vector3(-value.y, value.z, value.x);
    private static void Negate(float[] quaternion) { for (int i = 0; i < quaternion.Length; i++) quaternion[i] = -quaternion[i]; }

    private static void AssertRejected(string xml)
    {
        Assert.That(RosRobotDescription.TryParse(xml, out RosRobotModel model, out string error), Is.False);
        Assert.That(model, Is.Null);
        Assert.That(error, Is.Not.Empty);
    }

    private static void AssertMismatch(RosRobotModel expected, RosRobotModel actual, string link)
    {
        Assert.That(RosRobotDescription.Equivalent(expected, actual, out string reason), Is.False);
        Assert.That(reason, Does.Contain(link));
    }

    private static void AssertVector(Vector3 expected, Vector3 actual, float tolerance, string context)
    {
        Assert.That(Vector3.Distance(expected, actual), Is.LessThan(tolerance), context + ": expected " + expected + ", got " + actual);
    }
}
