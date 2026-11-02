using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>Scene transitions release robot state while retaining one disabled, connection-free transport.</summary>
public sealed class WorkflowServiceLifecycleTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();

    [TearDown]
    public void Cleanup()
    {
        for (int index = _owned.Count-1; index >= 0; index--)
            if (_owned[index] != null) UnityEngine.Object.DestroyImmediate(_owned[index]);
        _owned.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
    }

    [Test]
    public void SceneResetReleasesRobotReferencesAndReadinessWithoutDestroyingTheOldRobot()
    {
        Component coordinates = Add("Lifecycle coordinates", "RobotCoordinateService");
        Component state = Add("Lifecycle state", "RobotStateService");
        GameObject robot = New("base_link"); robot.AddComponent<ArticulationBody>();
        Transform tracking = New("Lifecycle tracking").transform;
        Call(state, "InitializeRobot", Pose.identity, robot, tracking);
        SetProperty(state, "IsReadyForPlanning", true);
        Assert.That(Get<bool>(state, "IsInitialized"), Is.True);
        int revision = Get<int>(coordinates, "CalibrationRevision");
        int notifications = 0;
        coordinates.GetType().GetEvent("CalibrationChanged").AddEventHandler(coordinates, (Action)(() => notifications++));
        Call(state, "ResetForSceneChange");
        Call(coordinates, "ResetForSceneChange");
        Assert.That(Get<bool>(state, "IsInitialized"), Is.False);
        Assert.That(Get<bool>(state, "IsReadyForPlanning"), Is.False);
        Assert.That(Call(state, "GetRobotInstance"), Is.Null);
        Assert.That(Call(state, "GetRobotRootArticulation"), Is.Null);
        Assert.That(Get<bool>(coordinates, "IsInitialized"), Is.False);
        Assert.That(Get<int>(coordinates, "CalibrationRevision"), Is.EqualTo(revision+1));
        Assert.That(notifications, Is.EqualTo(1));
        Assert.That(robot != null, Is.True, "The outgoing scene owns destruction of its robot.");
        Assert.That(Field(coordinates, "_robotRootArticulation").GetValue(coordinates), Is.Null);
        Assert.That(Field(coordinates, "_calibrationFrame").GetValue(coordinates), Is.Null);
    }

    [Test]
    public void ResetEndsAnOldCalibrationEditAndAllowsFreshAlignmentInTheNewWorkflow()
    {
        Component coordinates = Add("Lifecycle edited coordinates", "RobotCoordinateService");
        ArticulationBody oldRoot = New("Old lifecycle base").AddComponent<ArticulationBody>();
        Call(coordinates, "Initialize", oldRoot, New("Old lifecycle tracking").transform);
        Call(coordinates, "BeginCalibrationEdit");
        Assert.That(Get<bool>(coordinates, "IsCalibrationInProgress"), Is.True);
        Call(coordinates, "ResetForSceneChange");
        Assert.That(Get<bool>(coordinates, "IsCalibrationInProgress"), Is.False);
        Assert.That(Get<bool>(coordinates, "IsInitialized"), Is.False);
        ArticulationBody nextRoot = New("New lifecycle base").AddComponent<ArticulationBody>();
        Call(coordinates, "Initialize", nextRoot, New("New lifecycle tracking").transform);
        Assert.That(Get<bool>(coordinates, "IsInitialized"), Is.True);
    }

    [Test]
    public void ASceneDestroyedRobotCannotLeaveStateReportingInitialized()
    {
        Add("Lifecycle destroyed coordinates", "RobotCoordinateService");
        Component state = Add("Lifecycle destroyed state", "RobotStateService");
        GameObject robot = New("base_link"); robot.AddComponent<ArticulationBody>();
        Call(state, "InitializeRobot", Pose.identity, robot);
        Assert.That(Get<bool>(state, "IsInitialized"), Is.True);
        UnityEngine.Object.DestroyImmediate(robot);
        Assert.That(Get<bool>(state, "IsInitialized"), Is.False);
    }

    [UnityTest]
    public IEnumerator ServiceReusesTheConnectorPreservesHudEditsAndRecoversAReplacement()
    {
        if (TypeOf("ROSService").GetField("_rosIPAddress", BindingFlags.NonPublic|BindingFlags.Instance) == null)
            Assert.Ignore("Run the scene workflow harness to exercise the real ROSService; the baseline harness deliberately stubs it.");
        // DontDestroyOnLoad is a real PlayMode operation. Both components stay
        // inactive, and no Start or Connect method is invoked during this check.
        yield return new EnterPlayMode();
        string failure = null;
        try
        {
            Type serviceType = TypeOf("ROSService");
            Component connector = DisabledConnector("Lifecycle persistent connector");
            GameObject serviceObject = New("Lifecycle ROS service"); serviceObject.SetActive(false);
            Component service = serviceObject.AddComponent(serviceType);
            Field(service, "_rosIPAddress").SetValue(service, "10.200.101.144");
            Field(service, "_rosPort").SetValue(service, 10000);
            Private(service, "Awake");
            Assert.That(Call(service, "GetROSConnection"), Is.SameAs(connector));
            Assert.That(serviceObject.scene.name, Is.EqualTo("DontDestroyOnLoad"));
            Assert.That(connector.gameObject.scene.name, Is.EqualTo("DontDestroyOnLoad"));
            Assert.That(Get<string>(connector, "RosIPAddress"), Is.EqualTo("10.200.101.144"));
            Assert.That(Get<int>(connector, "RosPort"), Is.EqualTo(10000));
            SetProperty(connector, "RosIPAddress", "10.200.101.151");
            Assert.That(Call(service, "GetROSConnection"), Is.SameAs(connector));
            Assert.That(Get<string>(connector, "RosIPAddress"), Is.EqualTo("10.200.101.151"), "Routine accesses must preserve the user's HUD endpoint.");
            Assert.That(Get<bool>(connector, "HasConnectionThread"), Is.False);
            UnityEngine.Object.DestroyImmediate(connector.gameObject);
            Assert.That(Get<bool>(service, "IsInitialized"), Is.False);
            Component replacement = DisabledConnector("Lifecycle replacement connector");
            Assert.That(Call(service, "GetROSConnection"), Is.SameAs(replacement));
            Assert.That(Get<bool>(service, "IsInitialized"), Is.True);
            Assert.That(replacement.gameObject.scene.name, Is.EqualTo("DontDestroyOnLoad"));
            Assert.That(Get<bool>(replacement, "HasConnectionThread"), Is.False);
        }
        catch (Exception error) { failure = error.ToString(); }
        finally
        {
            try { Cleanup(); }
            catch (Exception error) { if (failure == null) failure = error.ToString(); }
        }
        yield return new ExitPlayMode();
        Assert.That(failure, Is.Null, failure);
    }

    [Test]
    public void MtcManagerDestructionCleanupUnsubscribesAllThreeTopicsFromTheRetainedConnector()
    {
        Component connector = DisabledConnector("Lifecycle subscriber connector");
        string[] topics = { "/mtc/planning_status", "/mtc/planned_trajectory", "/mtc/execution_status" };
        string[] messages = { "std_msgs/String", "moveit_msgs/RobotTrajectory", "std_msgs/String" };
        for (int index = 0; index < topics.Length; index++)
        {
            Call(connector, "SubscribeByMessageName", topics[index], messages[index], (Action<object>)(_ => { }));
            Assert.That(Call(connector, "HasSubscriber", topics[index]), Is.EqualTo(true));
        }
        Component manager = Add("Lifecycle MTC subscriber", "MTCPlansManager");
        Field(manager, "_connection").SetValue(manager, connector);
        // EditMode does not run Awake/OnDestroy for a newly added runtime-only
        // component. Exercise its actual teardown method without starting ROS.
        Private(manager, "Awake");
        Private(manager, "OnDestroy");
        foreach (string topic in topics) Assert.That(Call(connector, "HasSubscriber", topic), Is.EqualTo(false));
        UnityEngine.Object.DestroyImmediate(manager.gameObject);
        foreach (string topic in topics) Assert.That(Call(connector, "HasSubscriber", topic), Is.EqualTo(false));
        Assert.That(Get<bool>(connector, "HasConnectionThread"), Is.False);
    }

    private Component DisabledConnector(string name)
    {
        GameObject value = New(name); value.SetActive(false);
        Component connector = value.AddComponent(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection"));
        ((Behaviour)connector).enabled = false;
        SetProperty(connector, "ConnectOnStart", false); SetProperty(connector, "ShowHud", false);
        Private(connector, "Awake");
        return connector;
    }
    private GameObject New(string name) { var value = new GameObject(name); _owned.Add(value); return value; }
    private Component Add(string name,string type) => New(name).AddComponent(TypeOf(type));
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(assembly => assembly.GetType(name, false)).First(type => type != null);
    private static FieldInfo Field(object value,string name) => value.GetType().GetField(name, BindingFlags.Instance|BindingFlags.NonPublic|BindingFlags.Public);
    private static T Get<T>(object value,string name) => (T)value.GetType().GetProperty(name).GetValue(value);
    private static void SetProperty(object value,string name,object data) => value.GetType().GetProperty(name).SetValue(value,data);
    private static object Call(object value,string method,params object[] arguments) => value.GetType().GetMethods(BindingFlags.Public|BindingFlags.Instance)
        .Single(candidate => candidate.Name == method && candidate.GetParameters().Length == arguments.Length &&
            candidate.GetParameters().Select((parameter,index) => arguments[index] == null || parameter.ParameterType.IsInstanceOfType(arguments[index])).All(compatible => compatible))
        .Invoke(value,arguments);
    private static void Private(object value,string method) => value.GetType().GetMethod(method, BindingFlags.NonPublic|BindingFlags.Instance).Invoke(value,null);
}
