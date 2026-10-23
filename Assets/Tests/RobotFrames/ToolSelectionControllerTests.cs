using System;
using System.Collections.Generic;
using System.Reflection;
using System.Xml;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>Offline policy and UI tests. The controller stays disabled, so Start cannot contact ROS.</summary>
public sealed class ToolSelectionControllerTests
{
    private static readonly string[] Services =
    {
        "/robot_state_publisher/get_parameters", "/move_group/get_parameters", "/mtc_test_task_node/get_parameters"
    };
    private readonly List<GameObject> _objects = new List<GameObject>();
    private Type _controllerType;
    private Component _controller;

    [SetUp]
    public void CreateDisabledOfflineController()
    {
        _controllerType = Type.GetType("ToolSelectionController, Assembly-CSharp", true);
        GameObject owner = NewObject("Disabled offline tool controller");
        owner.SetActive(false);
        _controller = owner.AddComponent(_controllerType);
        ((Behaviour)_controller).enabled = false;
        owner.SetActive(true);
        if (_controllerType.GetField("_catalog", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(_controller) == null)
            InvokePrivate(_controller, "Awake");
        Assert.That(Get<bool>("IsRosModelMatched"), Is.False);
    }

    [TearDown]
    public void Cleanup()
    {
        // Menus own detached world-space canvases. Remove their transient UI
        // objects first while running these tests outside play mode.
        foreach (GameObject owner in _objects)
            if (owner != null)
                foreach (Component component in owner.GetComponents<Component>())
                    if (component != null && component.GetType().Name == "ToolSelectionMenu")
                    {
                        Canvas canvas = (Canvas)component.GetType().GetField("_canvas", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(component);
                        if (canvas != null) UnityEngine.Object.DestroyImmediate(canvas.gameObject);
                    }
        for (int i = _objects.Count - 1; i >= 0; i--)
            if (_objects[i] != null) UnityEngine.Object.DestroyImmediate(_objects[i]);
        _objects.Clear();
        Type connection = Type.GetType("Unity.Robotics.ROSTCPConnector.ROSConnection, Unity.Robotics.ROSTCPConnector", false);
        if (connection != null) Assert.That(Resources.FindObjectsOfTypeAll(connection), Is.Empty, "Offline inspection must not create a ROS connection.");
    }

    [TestCase("none")]
    [TestCase("kenos")]
    [TestCase("kenos_sl")]
    public void ReadinessRequiresAllThreeIndependentRosDescriptions(string tool)
    {
        string xml = Description(tool);
        Assert.That(Inspect(Services[0], xml), Is.True);
        Assert.That(Get<bool>("IsRosModelMatched"), Is.False);
        Assert.That(Inspect(Services[1], xml), Is.True);
        Assert.That(Get<bool>("IsRosModelMatched"), Is.False);
        Assert.That(Inspect(Services[2], xml), Is.True);
        Assert.That(Get<bool>("IsRosModelMatched"), Is.True);
        Assert.That(Get<string>("SelectedTool"), Is.EqualTo(tool));
        Assert.That(Static<bool>("CanSendRobotCommands"), Is.True);
        Assert.That(Static<bool>("CanPlanPickPlace"), Is.EqualTo(tool != "none"));
        Invoke(_controller, "SelectTool", tool == "kenos" ? "kenos_sl" : "kenos");
        Assert.That(Get<bool>("IsRosModelMatched"), Is.False);
        Assert.That(Static<bool>("CanSendRobotCommands"), Is.False);
    }

    [Test]
    public void DifferentConsumerTcpBlocksThenMatchingActualCustomModelsBecomeAuthority()
    {
        string nominal = Description("kenos_sl");
        foreach (string service in Services) Assert.That(Inspect(service, nominal), Is.True);
        XmlDocument document = new XmlDocument();
        document.LoadXml(nominal);
        ((XmlElement)document.SelectSingleNode("/robot/joint[@name='kenos_sl_tcp_fixed']/origin"))
            .SetAttribute("xyz", "0.045 -0.01 0.12");
        string custom = document.OuterXml;
        Assert.That(Inspect(Services[1], custom), Is.True);
        Assert.That(Get<bool>("IsRosModelMatched"), Is.False);
        Assert.That(Static<bool>("CanPlanPickPlace"), Is.False);
        Assert.That(Inspect(Services[0], custom), Is.True);
        Assert.That(Get<bool>("IsRosModelMatched"), Is.False);
        Assert.That(Inspect(Services[2], custom), Is.True);
        Assert.That(Get<bool>("IsRosModelMatched"), Is.True);
        RosRobotModel model = Get<RosRobotModel>("CurrentModel");
        CollectionAssert.AreEqual(new[] { 0.045f, -0.01f, 0.12f }, model.FindLink("tcp").position);
    }

    [Test]
    public void FailedOrExpiredInspectionRevokesReadiness()
    {
        foreach (string service in Services) Assert.That(Inspect(service, Description("kenos")), Is.True);
        Assert.That(Get<bool>("IsRosModelMatched"), Is.True);
        Assert.That(Inspect(Services[1], "<robot>"), Is.False);
        Assert.That(Get<bool>("IsRosModelMatched"), Is.False);
        Assert.That(Inspect(Services[1], Description("kenos")), Is.True);
        Assert.That(Get<bool>("IsRosModelMatched"), Is.True);
        Array inspections = (Array)_controllerType.GetField("_inspections", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(_controller);
        object inspection = inspections.GetValue(2);
        inspection.GetType().GetField("ReceivedAt").SetValue(inspection, Time.realtimeSinceStartup - 100f);
        InvokePrivate(_controller, "RefreshStatus");
        Assert.That(Get<bool>("IsRosModelMatched"), Is.False);
    }

    [Test]
    public void AnUnansweredInspectionExpiresAndSupersedesItsRequestGeneration()
    {
        // Construct only a disabled component, with no connection thread or
        // Start call. No service request is sent while exercising expiry.
        GameObject owner = NewObject("Inactive connection metadata fixture");
        owner.SetActive(false);
        Type connectionType = Type.GetType("Unity.Robotics.ROSTCPConnector.ROSConnection, Unity.Robotics.ROSTCPConnector", true);
        Component connection = owner.AddComponent(connectionType);
        ((Behaviour)connection).enabled = false;
        Assert.That(connectionType.GetProperty("HasConnectionThread").GetValue(connection), Is.EqualTo(false));
        _controllerType.GetField("_connection", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(_controller, connection);
        _controllerType.GetField("_nextRefresh", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(_controller, float.PositiveInfinity);
        Array inspections = (Array)_controllerType.GetField("_inspections", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(_controller);
        object inspection = inspections.GetValue(0);
        Type inspectionType = inspection.GetType();
        inspectionType.GetField("Pending").SetValue(inspection, true);
        inspectionType.GetField("RequestedAt").SetValue(inspection, Time.realtimeSinceStartup - 100f);
        inspectionType.GetField("RequestSerial").SetValue(inspection, 7);
        InvokePrivate(_controller, "Update");
        Assert.That(inspectionType.GetField("Pending").GetValue(inspection), Is.EqualTo(false));
        Assert.That(inspectionType.GetField("RequestSerial").GetValue(inspection), Is.EqualTo(8));
        Assert.That(Get<bool>("IsRosModelMatched"), Is.False);
        _controllerType.GetField("_connection", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(_controller, null);
    }

    [Test]
    public void LocalSelectionUpdatesAnActualRigWithoutGrantingCommandReadiness()
    {
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab");
        GameObject robot = UnityEngine.Object.Instantiate(prefab);
        _objects.Add(robot);
        robot.SetActive(false);
        Component rig = robot.AddComponent(Type.GetType("RosRobotRig, Assembly-CSharp", true));
        Assert.That(Invoke(_controller, "ConfigureRobot", rig), Is.EqualTo(true));
        // ApplySelection enumerates active rigs, matching the scene's visible robot.
        robot.SetActive(true);
        Invoke(_controller, "SelectTool", "kenos_sl");
        Assert.That(((RosRobotModel)rig.GetType().GetProperty("Model").GetValue(rig)).tool, Is.EqualTo("kenos_sl"));
        Assert.That(Get<bool>("IsRosModelMatched"), Is.False);
        Assert.That(Static<bool>("CanPlanPickPlace"), Is.False);
    }

    [Test]
    public void ActualWorldMenuButtonsChooseEveryProfileAndShowTheSelection()
    {
        GameObject camera = NewObject("Menu test camera");
        camera.tag = "MainCamera";
        camera.AddComponent<Camera>();
        GameObject owner = NewObject("Actual tool menu");
        Type menuType = Type.GetType("ToolSelectionMenu, Assembly-CSharp", true);
        Component menu = owner.AddComponent(menuType);
        menuType.GetField("_controller", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(menu, _controller);
        InvokePrivate(menu, "Start");
        Canvas canvas = (Canvas)menuType.GetField("_canvas", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(menu);
        Assert.That(canvas.renderMode, Is.EqualTo(RenderMode.WorldSpace));
        Assert.That(canvas.transform.parent, Is.Null);
        Type buttonType = Type.GetType("UnityEngine.UI.Button, UnityEngine.UI", true);
        foreach (string tool in new[] { "none", "kenos", "kenos_sl" })
        {
            string title = tool == "none" ? "No tool" : tool == "kenos" ? "Kenos" : "Kenos SL";
            Component button = Array.Find(canvas.GetComponentsInChildren(buttonType), item => item.name == title);
            Assert.That(button, Is.Not.Null);
            object click = buttonType.GetProperty("onClick").GetValue(button);
            click.GetType().GetMethod("Invoke").Invoke(click, null);
            Assert.That(Get<string>("SelectedTool"), Is.EqualTo(tool));
            Assert.That(Static<bool>("CanSendRobotCommands"), Is.False);
        }
    }

    private GameObject NewObject(string name) { GameObject value = new GameObject(name); _objects.Add(value); return value; }
    private static string Description(string tool) => Resources.Load<TextAsset>("RobotModels/" + tool + ".urdf").text;
    private bool Inspect(string service, string xml) => (bool)Invoke(_controller, "InspectDescription", service, xml);
    private T Get<T>(string name) => (T)_controllerType.GetProperty(name).GetValue(_controller);
    private T Static<T>(string name) => (T)_controllerType.GetProperty(name).GetValue(null);
    private static object Invoke(Component component, string name, params object[] arguments) => component.GetType().GetMethod(name).Invoke(component, arguments);
    private static void InvokePrivate(Component component, string name) => component.GetType().GetMethod(name, BindingFlags.NonPublic | BindingFlags.Instance).Invoke(component, null);
}
