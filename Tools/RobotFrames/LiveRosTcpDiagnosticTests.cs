using System;
using System.IO;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

// Isolated offline diagnostic only. This file is outside the application's Assets.
public sealed class LiveRosTcpDiagnosticTests
{
    [Serializable] public sealed class Fixture
    {
        public string robotDescription;
        public string[] jointNames;
        public double[] jointPositions;
        public string[] fkLinkNames;
        public LinkPose[] fkPoses;
        public string reportPath;
    }
    [Serializable] public sealed class LinkPose { public float[] position; public float[] rotation; }
    [Serializable] public sealed class Report
    {
        public string completedUtc;
        public string tool;
        public float maximumPositionErrorMetres;
        public float maximumBasisError;
        public float tcpPositionErrorMetres;
        public int physicsSteps;
        public string scope;
    }

    [Test]
    public void ActualLiveRspUrdfAndJointsMatchMoveItTcpAfterNativePhysics()
    {
        Fixture data = JsonUtility.FromJson<Fixture>(File.ReadAllText(
            Path.Combine(Application.dataPath, "Tests/RobotFrames/Fixtures/LiveRosTcpDiagnostic.json")));
        Assert.That(RosRobotDescription.TryParse(data.robotDescription, out RosRobotModel model, out string error), Is.True, error);
        Assert.That(model.tool, Is.EqualTo("kenos_sl"));
        Type rigType = RuntimeType("RosRobotRig");
        Type coordinateType = RuntimeType("RobotCoordinateService");
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab");
        Assert.That(prefab, Is.Not.Null);
        GameObject robot = UnityEngine.Object.Instantiate(prefab);
        robot.SetActive(false);
        GameObject tracking = new GameObject("Offline live ROS diagnostic tracking frame");
        GameObject coordinateOwner = new GameObject("Offline live ROS diagnostic coordinate service");
        SimulationMode previous = Physics.simulationMode;
        try
        {
            Component rig = robot.AddComponent(rigType);
            Assert.That(Call(rig, "ApplyModel", model), Is.EqualTo(true));
            Pose expectedBase = PoseFrom(data.fkPoses[Array.IndexOf(data.fkLinkNames, "ur_base_link")]);
            Call(rig, "SetBaseWorldPose", expectedBase);
            robot.SetActive(true);
            Call(rig, "SetBaseWorldPose", expectedBase);
            Assert.That(Call(rig, "SetJointPositions", data.jointNames, data.jointPositions), Is.EqualTo(true));
            Component coordinates = coordinateOwner.AddComponent(coordinateType);
            Transform baseLink = (Transform)rigType.GetProperty("BaseLink").GetValue(rig);
            coordinateType.GetMethod("Initialize", new[] {typeof(ArticulationBody),typeof(Transform)})
                .Invoke(coordinates,new object[] {baseLink.GetComponent<ArticulationBody>(),tracking.transform});
            Physics.simulationMode = SimulationMode.Script;
            float maxPosition = 0f, maxBasis = 0f, tcpError = 0f;
            for (int step = 0; step < 122; step++)
            {
                coordinateType.GetMethod("FixedUpdate",BindingFlags.Instance|BindingFlags.NonPublic).Invoke(coordinates,null);
                Physics.SyncTransforms();
                Physics.Simulate(0.02f);
                if (step != 1 && step != 121) continue;
                for (int index = 0; index < data.fkLinkNames.Length; index++)
                {
                    object[] args = {data.fkLinkNames[index],null};
                    Assert.That(Call(rig,"TryGetLink",args),Is.EqualTo(true));
                    Transform actual = (Transform)args[1];
                    Pose expected = PoseFrom(data.fkPoses[index]);
                    float positionError = Vector3.Distance(actual.position,expected.position);
                    float basisError = Mathf.Max(Vector3.Distance(actual.right,expected.rotation*Vector3.right),
                        Vector3.Distance(actual.up,expected.rotation*Vector3.up),
                        Vector3.Distance(actual.forward,expected.rotation*Vector3.forward));
                    maxPosition = Mathf.Max(maxPosition,positionError);
                    maxBasis = Mathf.Max(maxBasis,basisError);
                    if (data.fkLinkNames[index] == "tcp") tcpError = positionError;
                    Assert.That(positionError,Is.LessThan(0.00002f),data.fkLinkNames[index]+" position at step "+step);
                    Assert.That(basisError,Is.LessThan(0.00002f),data.fkLinkNames[index]+" basis at step "+step);
                }
            }
            Report report = new Report {completedUtc=DateTime.UtcNow.ToString("o"), tool=model.tool,
                maximumPositionErrorMetres=maxPosition,maximumBasisError=maxBasis,
                tcpPositionErrorMetres=tcpError,physicsSteps=122,
                scope="Actual current RSP URDF, six current joint positions, actual main Unity prefab, native PhysX after two and 122 steps, authoritative coordinate FixedUpdate; compared to six MoveIt FK frames from live read-only snapshot. No ROS connection or commands."};
            File.WriteAllText(data.reportPath,JsonUtility.ToJson(report,true));
            Debug.Log("LIVE_ROS_TCP_DIAGNOSTIC "+JsonUtility.ToJson(report));
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(coordinateOwner);
            UnityEngine.Object.DestroyImmediate(tracking);
            Physics.simulationMode = previous;
        }
    }
    private static Pose PoseFrom(LinkPose value) => new Pose(RosRobotModelMath.FromRosVector(value.position),
        RosRobotModelMath.FromRosRotation(value.rotation));
    private static Type RuntimeType(string name) => AppDomain.CurrentDomain.GetAssemblies()
        .Select(a=>a.GetType(name)).First(t=>t!=null);
    private static object Call(Component owner,string method,params object[] args) => owner.GetType()
        .GetMethod(method,BindingFlags.Public|BindingFlags.Instance).Invoke(owner,args);
}
