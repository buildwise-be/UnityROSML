using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

public class RosRobotRigPrefabTests
{
    private const string PrefabDirectory = "Assets/Prefabs/ur16e_kenos/";
    private static Type RigType => AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType("RosRobotRig"))
        .FirstOrDefault(type => type != null) ?? throw new InvalidOperationException("RosRobotRig was not compiled.");

    [TestCase(false, "none")]
    [TestCase(false, "kenos")]
    [TestCase(false, "kenos_sl")]
    [TestCase(true, "none")]
    [TestCase(true, "kenos")]
    [TestCase(true, "kenos_sl")]
    public void ImportedPrefabMatchesEveryRosGoldenLinkAfterPhysics(bool transparent, string tool)
    {
        GameObject robot = Instantiate(transparent);
        SimulationMode previous = Physics.simulationMode;
        try
        {
            Physics.simulationMode = SimulationMode.Script;
            RosRobotModel model = RosRobotModelCatalog.Load().Find(tool);
            Component rig = robot.AddComponent(RigType);
            Assert.That(Invoke(rig, "ApplyModel", model), Is.EqualTo(true));
            Pose basePose = new Pose(new Vector3(2.1f, -0.8f, 3.4f), Quaternion.Euler(31f, -73f, 14f));
            Invoke(rig, "SetBaseWorldPose", basePose);
            robot.SetActive(true);
            Invoke(rig, "SetBaseWorldPose", basePose);
            float maximumPositionError = 0f;
            float maximumDirectionError = 0f;
            foreach (RosRobotGoldenCase sample in model.goldenCases)
            {
                Assert.That(Invoke(rig, "SetJointPositions", sample.jointNames, sample.jointPositions), Is.EqualTo(true));
                Physics.SyncTransforms();
                Physics.Simulate(0.02f);
                Physics.Simulate(0.02f);
                foreach (RosRobotLinkPose link in sample.links)
                {
                    object[] arguments = { link.name, null };
                    Assert.That(Invoke(rig, "TryGetLink", arguments), Is.EqualTo(true), link.name);
                    Transform actual = (Transform)arguments[1];
                    Pose expected = RobotFrameMath.Compose(basePose, new Pose(
                        RosRobotModelMath.FromRosVector(link.position), RosRobotModelMath.FromRosRotation(link.rotation)));
                    float positionError = Vector3.Distance(actual.position, expected.position);
                    float directionError = DirectionError(actual.rotation, expected.rotation);
                    maximumPositionError = Mathf.Max(maximumPositionError, positionError);
                    maximumDirectionError = Mathf.Max(maximumDirectionError, directionError);
                    Assert.That(positionError, Is.LessThan(0.00002f),
                        tool + "/" + sample.name + "/" + link.name + " position");
                    Assert.That(directionError, Is.LessThan(0.00002f),
                        tool + "/" + sample.name + "/" + link.name + " basis direction");
                }
            }
            Debug.Log("RosRigValidation " + (transparent ? "transparent/" : "main/") + tool +
                ": maxPositionMetres=" + maximumPositionError.ToString("R") +
                ", maxBasisVectorError=" + maximumDirectionError.ToString("R"));
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(robot);
            Physics.simulationMode = previous;
        }
    }

    [TestCase(false)]
    [TestCase(true)]
    public void SwitchingToolsPreservesBaseAndJointsAndRebuildsExactGeometry(bool transparent)
    {
        GameObject robot = Instantiate(transparent);
        SimulationMode previous = Physics.simulationMode;
        try
        {
            Physics.simulationMode = SimulationMode.Script;
            Component rig = robot.AddComponent(RigType);
            RosRobotModelCatalog catalog = RosRobotModelCatalog.Load();
            Pose basePose = new Pose(new Vector3(4f, 3f, -2f), Quaternion.Euler(14f, 25f, 37f));
            Assert.That(Invoke(rig, "ApplyModel", catalog.Find("kenos")), Is.EqualTo(true));
            Invoke(rig, "SetBaseWorldPose", basePose);
            robot.SetActive(true);
            Invoke(rig, "SetBaseWorldPose", basePose);
            RosRobotGoldenCase initial = catalog.Find("kenos").goldenCases[2];
            Assert.That(Invoke(rig, "SetJointPositions", initial.jointNames, initial.jointPositions), Is.EqualTo(true));
            foreach (string tool in new[] { "kenos", "kenos_sl", "none", "kenos" })
            {
                RosRobotModel model = catalog.Find(tool);
                Assert.That(Invoke(rig, "ApplyModel", model), Is.EqualTo(true));
                Physics.SyncTransforms();
                Physics.Simulate(0.02f);
                foreach (RosRobotLinkPose pose in model.goldenCases[2].links)
                {
                    object[] arguments = { pose.name, null };
                    Assert.That(Invoke(rig, "TryGetLink", arguments), Is.EqualTo(true));
                    Transform node = (Transform)arguments[1];
                    Pose expected = RobotFrameMath.Compose(basePose, new Pose(RosRobotModelMath.FromRosVector(pose.position),
                        RosRobotModelMath.FromRosRotation(pose.rotation)));
                    Assert.That(Vector3.Distance(node.position, expected.position), Is.LessThan(0.00002f), tool + "/" + pose.name);
                    Assert.That(DirectionError(node.rotation, expected.rotation), Is.LessThan(0.00002f), tool + "/" + pose.name);
                }
                foreach (RosRobotLink link in model.links)
                {
                    object[] arguments = { link.name, null };
                    Assert.That(Invoke(rig, "TryGetLink", arguments), Is.EqualTo(true));
                    Transform node = (Transform)arguments[1];
                    Transform[] visuals = node.Cast<Transform>().Where(t => t.name.StartsWith("ROS visual:")).ToArray();
                    if (link.name == "kenos_sl_base" && Resources.Load<TextAsset>("RobotModels/KenosSLVisual") != null)
                    {
                        Assert.That(visuals.Length, Is.EqualTo(1));
                        Assert.That(visuals[0].localPosition, Is.EqualTo(Vector3.zero));
                        Assert.That(visuals[0].localRotation, Is.EqualTo(Quaternion.identity));
                        Assert.That(visuals[0].localScale, Is.EqualTo(Vector3.one));
                        Mesh detail = visuals[0].GetComponent<MeshFilter>().sharedMesh;
                        Assert.That(detail, Is.Not.Null);
                        Assert.That(detail.vertexCount, Is.GreaterThan(0));
                        Assert.That(detail.bounds.size.sqrMagnitude, Is.GreaterThan(0.001f));
                        Assert.That(detail.normals.Length, Is.EqualTo(detail.vertexCount));
                        Assert.That(detail.normals.Any(n => n.sqrMagnitude > 0.9f), Is.True);
                        continue;
                    }
                    Assert.That(visuals.Length, Is.EqualTo(link.visuals.Length), link.name);
                    for (int index = 0; index < visuals.Length; index++)
                    {
                        Pose expected = RosRobotModelMath.GeometryPose(link.visuals[index]);
                        Assert.That(Vector3.Distance(visuals[index].localPosition, expected.position), Is.LessThan(0.000001f));
                        Assert.That(DirectionError(visuals[index].localRotation, expected.rotation), Is.LessThan(0.00002f));
                        if (link.visuals[index].kind == "mesh")
                        {
                            Assert.That(visuals[index].GetComponentsInChildren<MeshFilter>(true).Length, Is.GreaterThan(0));
                            Assert.That(visuals[index].GetComponentsInChildren<MeshFilter>(true).All(m => m.sharedMesh != null), Is.True);
                        }
                    }
                }
                Assert.That(robot.GetComponentsInChildren<Collider>().All(c => !c.enabled), Is.True);
                if (transparent)
                {
                    Assert.That(robot.GetComponentsInChildren<Renderer>().All(r => r.sharedMaterials.All(m => m.color.a < 0.99f)),
                        Is.True, "Every ghost visual must retain its transparent material alpha.");
                    Assert.That(robot.GetComponentsInChildren<Renderer>().All(r => r.sharedMaterials.All(m => m.renderQueue >= 3000)),
                        Is.True, "Ghost materials need transparent rendering, including embedded imported materials.");
                }
            }
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(robot);
            Physics.simulationMode = previous;
        }
    }

    [Test]
    public void HelperShaderWithoutColorOrTexturePropertiesDoesNotBreakRigOrGhostMaterials()
    {
        GameObject robot = Instantiate(false);
        Material helperMaterial = null;
        try
        {
            Shader shader = AssetDatabase.LoadAssetAtPath<Shader>("Assets/Tests/RobotFrames/Fixtures/NoMaterialProperties.shader");
            Assert.That(shader, Is.Not.Null);
            helperMaterial = new Material(shader);
            Assert.That(helperMaterial.HasProperty("_Color"), Is.False);
            Assert.That(helperMaterial.HasProperty("_BaseColor"), Is.False);
            Assert.That(helperMaterial.HasProperty("_MainTex"), Is.False);
            Assert.That(helperMaterial.HasProperty("_BaseMap"), Is.False);
            GameObject helper = new GameObject("No tint root helper", typeof(MeshRenderer));
            helper.transform.SetParent(robot.transform, false);
            helper.transform.SetAsFirstSibling();
            helper.GetComponent<Renderer>().sharedMaterial = helperMaterial;
            Transform upperArm = Array.Find(robot.GetComponentsInChildren<Transform>(true), t => t.name == "upper_arm_link");
            GameObject meshHelper = new GameObject("No tint mesh helper", typeof(MeshRenderer));
            meshHelper.transform.SetParent(upperArm.Find("Visuals").GetChild(0), false);
            meshHelper.GetComponent<Renderer>().sharedMaterial = helperMaterial;
            Component rig = robot.AddComponent(RigType);
            RigType.GetProperty("IsGhost").SetValue(rig, true);
            Assert.That(Invoke(rig, "ApplyModel", RosRobotModelCatalog.Load().Find("kenos_sl")), Is.EqualTo(true));
            object[] arguments = { "ur_upper_arm_link", null };
            Assert.That(Invoke(rig, "TryGetLink", arguments), Is.EqualTo(true));
            Transform node = (Transform)arguments[1];
            Transform visual = node.Cast<Transform>().First(t => t.name.StartsWith("ROS visual:"));
            Renderer copied = visual.GetComponentsInChildren<Renderer>(true).First(r => r.name == "No tint mesh helper");
            Assert.That(copied.sharedMaterial, Is.Not.SameAs(helperMaterial));
            Assert.That(copied.sharedMaterial.renderQueue, Is.GreaterThanOrEqualTo(3000));
            Assert.That(copied.sharedMaterial.HasProperty("_Color") || copied.sharedMaterial.HasProperty("_BaseColor"), Is.True);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(robot);
            if (helperMaterial != null) UnityEngine.Object.DestroyImmediate(helperMaterial);
        }
    }

    [Test]
    public void ExtraFixedFrameUsesNearestArticulationAnchorAndCanBeRemovedAgain()
    {
        GameObject robot = Instantiate(false);
        SimulationMode previous = Physics.simulationMode;
        try
        {
            Physics.simulationMode = SimulationMode.Script;
            RosRobotModelCatalog catalog = RosRobotModelCatalog.Load();
            RosRobotModel model = JsonUtility.FromJson<RosRobotModel>(JsonUtility.ToJson(catalog.Find("none")));
            List<RosRobotLink> links = new List<RosRobotLink>(model.links);
            RosRobotLink upperArm = links.Find(l => l.name == "ur_upper_arm_link");
            RosRobotLink extra = new RosRobotLink
            {
                name = "inspection_frame", parent = upperArm.parent, jointName = "inspection_frame_joint", jointType = "fixed",
                position = new[] { 0.013f, -0.021f, 0.008f }, rotation = new[] { 0f, 0f, 0.38268343f, 0.9238795f },
                visuals = Array.Empty<RosRobotGeometry>(), collisions = Array.Empty<RosRobotGeometry>()
            };
            links.Insert(links.IndexOf(upperArm), extra);
            upperArm.parent = extra.name;
            model.links = links.ToArray();
            Component rig = robot.AddComponent(RigType);
            Assert.That(Invoke(rig, "ApplyModel", model), Is.EqualTo(true));
            Pose basePose = new Pose(new Vector3(-2f, 1f, 3f), Quaternion.Euler(-17f, 41f, 9f));
            Invoke(rig, "SetBaseWorldPose", basePose);
            robot.SetActive(true);
            Invoke(rig, "SetBaseWorldPose", basePose);
            RosRobotGoldenCase sample = model.goldenCases[2];
            Dictionary<string, double> positions = new Dictionary<string, double>();
            for (int i = 0; i < sample.jointNames.Length; i++) positions[sample.jointNames[i]] = sample.jointPositions[i];
            Assert.That(Invoke(rig, "SetJointPositions", sample.jointNames, sample.jointPositions), Is.EqualTo(true));
            Physics.SyncTransforms();
            Physics.Simulate(0.02f);
            foreach (var link in RosRobotModelMath.ForwardKinematics(model, positions))
            {
                object[] arguments = { link.Key, null };
                Assert.That(Invoke(rig, "TryGetLink", arguments), Is.EqualTo(true));
                Transform node = (Transform)arguments[1];
                Pose expected = RobotFrameMath.Compose(basePose, link.Value);
                Assert.That(Vector3.Distance(node.position, expected.position), Is.LessThan(0.00002f), link.Key);
                Assert.That(DirectionError(node.rotation, expected.rotation), Is.LessThan(0.00002f), link.Key);
            }
            Assert.That(Invoke(rig, "ApplyModel", catalog.Find("none")), Is.EqualTo(true));
            Physics.SyncTransforms();
            Physics.Simulate(0.02f);
            object[] restored = { "ur_upper_arm_link", null };
            Assert.That(Invoke(rig, "TryGetLink", restored), Is.EqualTo(true));
            Assert.That(((Transform)restored[1]).GetComponent<ArticulationBody>(), Is.Not.Null);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(robot);
            Physics.simulationMode = previous;
        }
    }

    [Test]
    public void InvalidJointFeedbackIsAtomicAndUnrelatedMobileJointsAreIgnored()
    {
        GameObject robot = Instantiate(false);
        try
        {
            Component rig = robot.AddComponent(RigType);
            RosRobotModel model = RosRobotModelCatalog.Load().Find("kenos");
            Assert.That(Invoke(rig, "ApplyModel", model), Is.EqualTo(true));
            string name = model.goldenCases[0].jointNames[0];
            Assert.That(Invoke(rig, "SetJointPositions", new[] { "mir_left_wheel_joint", name }, new[] { 3.0, 0.4 }), Is.EqualTo(true));
            Assert.That(Invoke(rig, "SetJointPositions", new[] { name, name }, new[] { 0.2, 0.3 }), Is.EqualTo(false));
            Assert.That(Invoke(rig, "SetJointPositions", new[] { name }, new[] { double.NaN }), Is.EqualTo(false));
            object[] arguments = { name, 0d };
            Assert.That(Invoke(rig, "TryGetJointPosition", arguments), Is.EqualTo(true));
            Assert.That((double)arguments[1], Is.EqualTo(0.4));
        }
        finally { UnityEngine.Object.DestroyImmediate(robot); }
    }

    private static GameObject Instantiate(bool transparent)
    {
        string name = transparent ? "ur16e_kenos_transparent.prefab" : "ur16e_kenos.prefab";
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabDirectory + name);
        Assert.That(prefab, Is.Not.Null);
        GameObject robot = UnityEngine.Object.Instantiate(prefab);
        robot.SetActive(false);
        return robot;
    }

    private static object Invoke(Component rig, string method, params object[] arguments) =>
        RigType.GetMethod(method, BindingFlags.Public | BindingFlags.Instance).Invoke(rig, arguments);

    private static float DirectionError(Quaternion actual, Quaternion expected) => Mathf.Max(
        Vector3.Distance(actual * Vector3.right, expected * Vector3.right),
        Vector3.Distance(actual * Vector3.up, expected * Vector3.up),
        Vector3.Distance(actual * Vector3.forward, expected * Vector3.forward));
}
