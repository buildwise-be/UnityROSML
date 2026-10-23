using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>Actual tool meshes and TCP frames are inspected without a ROS connection.</summary>
public sealed class ToolPoseVisualTests
{
    private readonly List<GameObject> _objects = new List<GameObject>();
    private const string PrefabPath = "Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab";

    [TearDown]
    public void Cleanup()
    {
        Static("ToolPoseVisual", "SetReferencePrefab", new object[] { null });
        for (int index = _objects.Count - 1; index >= 0; index--)
            if (_objects[index] != null) UnityEngine.Object.DestroyImmediate(_objects[index]);
        _objects.Clear();
    }

    [TestCase("none")]
    [TestCase("kenos")]
    [TestCase("kenos_sl")]
    public void LiveToolCopiesOnlyGeometryAtTheIdenticalTcpPose(string tool)
    {
        Component rig = Robot(tool);
        Pose sourcePose = new Pose(rig.transform.position, rig.transform.rotation);
        Transform tcp = Get<Transform>(rig, "Tcp");
        Component view = View(new Pose(new Vector3(-1.4f, 0.7f, 2.3f), Quaternion.Euler(37f, -64f, 18f)));
        Assert.That(Call(view, "Refresh", rig, null), Is.EqualTo(true));
        Assert.That(Get<string>(view, "Tool"), Is.EqualTo(tool));
        Assert.That(Get<bool>(view, "HasVisual"), Is.True);
        Assert.That(view.GetComponentsInChildren<ArticulationBody>(true), Is.Empty);
        Assert.That(view.GetComponentsInChildren<Rigidbody>(true), Is.Empty);
        Assert.That(view.GetComponentsInChildren<Collider>(true), Is.Empty);
        Assert.That(view.GetComponentsInChildren<MonoBehaviour>(true), Has.Length.EqualTo(1),
            "A ghost may contain only its view component, not copied importer/controller scripts.");
        MeshFilter[] sourceMeshes = rig.GetComponentsInChildren<MeshFilter>();
        MeshFilter[] targetMeshes = view.GetComponentsInChildren<MeshFilter>(true);
        Assert.That(targetMeshes.Length, Is.GreaterThan(0));
        foreach (MeshFilter targetMesh in targetMeshes)
        {
            MeshFilter[] matches = sourceMeshes.Where(m => m.sharedMesh == targetMesh.sharedMesh).ToArray();
            Assert.That(matches, Is.Not.Empty, "The ghost must use an actual source mesh asset.");
            float bestError = float.PositiveInfinity;
            foreach (MeshFilter sourceMesh in matches)
            {
                float error = 0f;
                Matrix4x4 expected = view.transform.localToWorldMatrix * tcp.worldToLocalMatrix * sourceMesh.transform.localToWorldMatrix;
                foreach (Vector3 point in Corners(targetMesh.sharedMesh.bounds))
                    error = Mathf.Max(error, Vector3.Distance(expected.MultiplyPoint3x4(point),
                        targetMesh.transform.TransformPoint(point)));
                bestError = Mathf.Min(bestError, error);
            }
            Assert.That(bestError, Is.LessThan(0.00002f), targetMesh.name + " must retain its TCP-relative origin.");
            foreach (Material material in targetMesh.GetComponent<MeshRenderer>().sharedMaterials)
            {
                Assert.That(material.renderQueue, Is.EqualTo(3000));
                if (material.HasProperty("_BaseColor")) Assert.That(material.GetColor("_BaseColor").a, Is.EqualTo(0.72f).Within(0.0001f));
                else if (material.HasProperty("_Color")) Assert.That(material.GetColor("_Color").a, Is.EqualTo(0.72f).Within(0.0001f));
            }
        }
        Bounds localBounds = Get<Bounds>(view, "TcpLocalBounds");
        Assert.That(localBounds.size.magnitude, Is.GreaterThan(0.03f));
        Assert.That(Vector3.Distance(rig.transform.position, sourcePose.position), Is.LessThan(0.000001f));
        Assert.That(Quaternion.Angle(rig.transform.rotation, sourcePose.rotation), Is.LessThan(0.001f));
    }

    [TestCase("none")]
    [TestCase("kenos")]
    [TestCase("kenos_sl")]
    public void PrefabAndCadFallbackMatchesLiveToolBoundsBeforeQrPlacement(string tool)
    {
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        Static("ToolPoseVisual", "SetReferencePrefab", prefab);
        Component rig = Robot(tool);
        Component live = View(Pose.identity);
        Assert.That(Call(live, "Refresh", rig, null), Is.EqualTo(true));
        Bounds expected = Get<Bounds>(live, "TcpLocalBounds");
        int expectedMeshes = live.GetComponentsInChildren<MeshFilter>(true).Length;
        rig.gameObject.SetActive(false);
        Component fallback = View(new Pose(new Vector3(2f, -0.1f, -1f), Quaternion.Euler(26f, 18f, -31f)));
        Assert.That(Call(fallback, "Refresh", null, RosRobotModelCatalog.Load().Find(tool)), Is.EqualTo(true));
        Bounds actual = Get<Bounds>(fallback, "TcpLocalBounds");
        Assert.That(Vector3.Distance(expected.center, actual.center), Is.LessThan(0.00003f), tool + " centre");
        Assert.That(Vector3.Distance(expected.size, actual.size), Is.LessThan(0.00003f), tool + " dimensions");
        Assert.That(fallback.GetComponentsInChildren<MeshFilter>(true), Has.Length.EqualTo(expectedMeshes));
        Assert.That(fallback.GetComponentsInChildren<Collider>(true), Is.Empty);
    }

    [Test]
    public void ModelChangesRebuildGhostWithoutMovingItsTarget()
    {
        Component rig = Robot("kenos");
        Component view = View(new Pose(new Vector3(0.4f, 1.2f, -0.8f), Quaternion.Euler(17f, 44f, -6f)));
        Pose target = new Pose(view.transform.position, view.transform.rotation);
        int changes = 0;
        Action onChanged = () => changes++;
        view.GetType().GetEvent("VisualChanged").AddEventHandler(view, onChanged);
        Assert.That(Call(view, "Refresh", rig, null), Is.EqualTo(true));
        Assert.That(changes, Is.EqualTo(1));
        Assert.That(Call(view, "Refresh", null, null), Is.EqualTo(true));
        Assert.That(changes, Is.EqualTo(1), "An unchanged model must reuse its meshes and materials.");
        Assert.That(Call(rig, "ApplyModel", RosRobotModelCatalog.Load().Find("kenos_sl")), Is.EqualTo(true));
        Assert.That(Call(view, "Refresh", null, null), Is.EqualTo(true));
        Assert.That(Get<string>(view, "Tool"), Is.EqualTo("kenos_sl"));
        Assert.That(changes, Is.EqualTo(2));
        Assert.That(view.GetComponentsInChildren<MeshFilter>(true).Any(m => m.sharedMesh.vertexCount > 1000), Is.True,
            "Kenos SL must use the detailed CAD mesh rather than a sphere or an envelope only.");
        Assert.That(Vector3.Distance(target.position, view.transform.position), Is.LessThan(0.000001f));
        Assert.That(Quaternion.Angle(target.rotation, view.transform.rotation), Is.LessThan(0.001f));
    }

    [Test]
    public void PrefabFallbackRejectsAnUnsupportedToolMeshInsteadOfSubstitutingTheReferenceMesh()
    {
        Static("ToolPoseVisual", "SetReferencePrefab", AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath));
        RosRobotModel model = RosRobotModelCatalog.Load().Find("kenos");
        RosRobotLink toolLink = model.links.First(link => link.name.StartsWith("kenos", StringComparison.Ordinal) &&
            link.visuals != null && link.visuals.Any(geometry => geometry.kind == "mesh"));
        RosRobotGeometry mesh = toolLink.visuals.First(geometry => geometry.kind == "mesh");
        mesh.meshUri = "package://different_tool/meshes/unsupported_tool.dae";
        Component view = View(Pose.identity);
        LogAssert.Expect(LogType.Warning,
            "ToolPoseVisual: Tool preview unavailable: Unsupported tool visual mesh for " + toolLink.name + ".");
        Assert.That(Call(view, "Refresh", null, model), Is.EqualTo(false));
        Assert.That(Get<bool>(view, "HasVisual"), Is.False);
        Assert.That(Get<string>(view, "StatusText"), Does.Contain("Unsupported tool visual mesh"));
        Assert.That(view.GetComponentsInChildren<MeshFilter>(true), Is.Empty,
            "A rejected model must not retain a partial mount or silently display the reference tool.");
    }

    private Component Robot(string tool)
    {
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        Assert.That(prefab, Is.Not.Null);
        GameObject robot = UnityEngine.Object.Instantiate(prefab);
        _objects.Add(robot);
        robot.SetActive(false);
        Component rig = robot.AddComponent(TypeOf("RosRobotRig"));
        RosRobotModel model = RosRobotModelCatalog.Load().Find(tool);
        Assert.That(Call(rig, "ApplyModel", model), Is.EqualTo(true));
        Pose basePose = new Pose(new Vector3(3.2f, 0.5f, -1.1f), Quaternion.Euler(12f, -28f, 6f));
        robot.SetActive(true);
        Call(rig, "SetBaseWorldPose", basePose);
        RosRobotGoldenCase sample = model.goldenCases[3];
        Assert.That(Call(rig, "SetJointPositions", sample.jointNames, sample.jointPositions), Is.EqualTo(true));
        return rig;
    }

    private Component View(Pose pose)
    {
        GameObject owner = new GameObject("Offline TCP tool preview test");
        _objects.Add(owner);
        owner.SetActive(false);
        owner.transform.SetPositionAndRotation(pose.position, pose.rotation);
        Component view = owner.AddComponent(TypeOf("ToolPoseVisual"));
        Call(view, "Configure", Color.green, 0.72f);
        return view;
    }

    private static IEnumerable<Vector3> Corners(Bounds bounds)
    {
        for (int corner = 0; corner < 8; corner++)
            yield return bounds.center + new Vector3((corner & 1) == 0 ? -bounds.extents.x : bounds.extents.x,
                (corner & 2) == 0 ? -bounds.extents.y : bounds.extents.y,
                (corner & 4) == 0 ? -bounds.extents.z : bounds.extents.z);
    }

    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies()
        .Select(assembly => assembly.GetType(name)).First(type => type != null);
    private static object Call(Component target, string name, params object[] args) =>
        target.GetType().GetMethod(name, BindingFlags.Public | BindingFlags.Instance).Invoke(target, args);
    private static object Static(string type, string name, params object[] args) =>
        TypeOf(type).GetMethod(name, BindingFlags.Public | BindingFlags.Static).Invoke(null, args);
    private static T Get<T>(Component target, string name) => (T)target.GetType().GetProperty(name).GetValue(target);
}
