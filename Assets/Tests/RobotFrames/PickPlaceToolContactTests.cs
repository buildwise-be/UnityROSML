using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>Actual capture/export/depth workflow without starting XR input or ROS transport.</summary>
public sealed class PickPlaceToolContactTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private Component _controller, _coordinates, _plans;
    private Transform _base, _tracking;

    [SetUp]
    public void SetupActualComponentsWithoutTransport()
    {
        _coordinates = Add("Contact coordinates", "RobotCoordinateService");
        Add("Contact robot readiness", "RobotStateService");
        _tracking = New("Contact XR tracking").transform;
        _tracking.SetPositionAndRotation(new Vector3(1.2f, 0.3f, -0.7f), Quaternion.Euler(0, 27, 0));
        ArticulationBody body = New("base_link").AddComponent<ArticulationBody>();
        _base = body.transform;
        _base.SetPositionAndRotation(new Vector3(0.6f, -0.1f, 1.4f), Quaternion.Euler(14, -33, 7));
        Call(_coordinates, "Initialize", body, _tracking);
        Component tools = Add("Matched contact tool", "ToolSelectionController");
        if (Field(tools, "_catalog").GetValue(tools) == null) Private(tools, "Awake");
        string description = Resources.Load<TextAsset>("RobotModels/kenos_sl.urdf").text;
        foreach (string service in new[] { "/robot_state_publisher/get_parameters", "/move_group/get_parameters", "/mtc_test_task_node/get_parameters" })
            Assert.That(Call(tools, "InspectDescription", service, description), Is.EqualTo(true));
        _plans = Add("Contact MTC review", "MTCPlansManager");
        if (Field(_plans, "OnSuccessfulPlan").GetValue(_plans) == null) Private(_plans, "Awake");
        _controller = Add("Contact targets", "PickPlaceController");
        Set(_controller, "_plansManager", _plans);
    }

    [TearDown]
    public void Cleanup()
    {
        TypeOf("TargetPoseContextMenu").GetMethod("CloseAll").Invoke(null, null);
        for (int index = _owned.Count - 1; index >= 0; index--)
            if (_owned[index] != null) UnityEngine.Object.DestroyImmediate(_owned[index]);
        _owned.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty,
            "Tool-contact editing must never create a robot transport.");
    }

    [Test]
    public void DisplayedToolContactIsExportedExactlyWithoutSurfaceOrFlangeOffsets()
    {
        Pose pointer = new Pose(new Vector3(0.4f, 0.8f, -0.2f), Quaternion.Euler(27, -42, 19));
        Pose contact = RobotFrameMath.PointerToTcpPose(pointer);
        Capture(contact);
        GameObject pick = Point(true);
        AssertPose(pick.transform, contact);
        Assert.That(pick.GetComponent(TypeOf("ToolPoseVisual")), Is.Not.Null);
        Assert.That(pick.GetComponentsInChildren<ArticulationBody>(true), Is.Empty);
        object message = Export();
        object pickPose = Field(message, "pick_pose").GetValue(message);
        AssertExport(pickPose, contact);
        object header = Field(pickPose, "header").GetValue(pickPose);
        Assert.That(Field(header, "frame_id").GetValue(header), Is.EqualTo("ur_base_link"));
        Assert.That(TypeOf("PickPlaceController").GetField("ContactPlanTopic").GetRawConstantValue(),
            Is.EqualTo("/pick_place_tcp_target"));
    }

    [Test]
    public void LegacyPlanePrefabsAreNeverInstantiatedForContactTargets()
    {
        GameObject legacy = GameObject.CreatePrimitive(PrimitiveType.Plane);
        legacy.name = "Legacy surface plane"; legacy.SetActive(false); _owned.Add(legacy);
        Set(_controller, "_pickPointPrefab", legacy);
        Set(_controller, "_placePointPrefab", legacy);
        Capture(new Pose(new Vector3(0.3f, 0.7f, 0.2f), Quaternion.Euler(12, 8, -9)));
        foreach (GameObject target in new[] { Point(true), Point(false) })
            Assert.That(target.GetComponentsInChildren<Transform>(true).Any(t => t.name.StartsWith("Legacy surface plane")), Is.False);
        Assert.That(legacy.activeSelf, Is.False);
    }

    [Test]
    public void DepthClickMovesExportedContactFiveMillimetresAlongToolZAndRevokesReview()
    {
        Pose contact = new Pose(new Vector3(0.2f, 0.6f, 0.8f), Quaternion.Euler(31, 24, -17));
        Capture(contact);
        Component menu = Point(true).GetComponent(TypeOf("TargetPoseContextMenu"));
        Assert.That(Call(_plans, "BeginPlanningRequest"), Is.EqualTo(true));
        Set(_controller, "_hasPlanningRequestBeenSent", true);
        Assert.That(Call(menu, "NudgeDepth", 1), Is.EqualTo(true));
        Pose shifted = new Pose(contact.position + contact.rotation * Vector3.up * 0.005f, contact.rotation);
        AssertPose(Point(true).transform, shifted);
        AssertExport(Field(Export(), "pick_pose").GetValue(Export()), shifted);
        Assert.That(Get<bool>(_plans, "IsPlanning") || Get<bool>(_plans, "CanExecute"), Is.False);
        Assert.That(Field(_controller, "_hasPlanningRequestBeenSent").GetValue(_controller), Is.EqualTo(false));
        Call(menu, "ResetDepth");
        AssertPose(Point(true).transform, contact);
    }

    [Test]
    public void RecapturingResetsDepthBaselineWithoutKeepingTheOldOffset()
    {
        Pose first = new Pose(new Vector3(0.2f, 0.4f, 0.1f), Quaternion.Euler(4, 25, 3));
        Capture(first);
        Component menu = Point(true).GetComponent(TypeOf("TargetPoseContextMenu"));
        Call(menu, "NudgeDepth", 1);
        Pose next = new Pose(new Vector3(-0.3f, 0.8f, 0.5f), Quaternion.Euler(42, -18, 7));
        Call(_controller, "SetPickPlace", next.position, next.rotation);
        Assert.That(Get<int>(menu, "DepthSteps"), Is.Zero);
        Call(menu, "NudgeDepth", -1);
        AssertPose(Point(true).transform, new Pose(next.position - next.rotation * Vector3.up * 0.005f, next.rotation));
        Call(menu, "ResetDepth");
        AssertPose(Point(true).transform, next);
    }

    [Test]
    public void InactiveModeHidesToolAimAndRejectsCaptureAndExportWhileKeepingPreviousPoints()
    {
        Pose contact = new Pose(new Vector3(0.5f, 0.8f, -0.3f), Quaternion.Euler(18, 32, 7));
        Capture(contact);
        Call(_controller, "SetPointerPose", new Pose(new Vector3(0.1f, 0.7f, 0.3f), Quaternion.identity), _tracking);
        GameObject live = (GameObject)Field(_controller, "_livePointInstance").GetValue(_controller);
        Assert.That(live.activeSelf, Is.True);
        Call(_controller, "SetTilePlanningEnabled", false);
        Call(_controller, "SetPickPlace", Vector3.one * 8f, Quaternion.identity);
        Assert.That(Point(true).activeSelf || Point(false).activeSelf || live.activeSelf, Is.False);
        AssertPose(Point(true).transform, contact);
        object[] args = { null };
        Assert.That(CallArgs(_controller, "TryGetContactPoses", args), Is.EqualTo(false));
        Call(_controller, "SetTilePlanningEnabled", true);
        Assert.That(Point(true).activeSelf && Point(false).activeSelf, Is.True);
        AssertExport(Field(Export(), "pick_pose").GetValue(Export()), contact);
    }

    [Test]
    public void DelayedCapabilityResponseCannotRearmAfterAContactChanges()
    {
        Capture(new Pose(new Vector3(0.4f, 0.7f, 0.1f), Quaternion.identity));
        Set(_controller, "_checkingCapability", true);
        Set(_controller, "_generation", 40);
        Set(_controller, "_requestedContactPoses", Export());
        Call(_controller, "SetPickPlace", new Vector3(0.5f, 0.7f, 0.1f), Quaternion.identity);
        Private(_controller, "OnCapabilityResponse", null, 40);
        Assert.That(Get<bool>(_controller, "IsPlanning"), Is.False);
        Assert.That(Get<bool>(_plans, "IsPlanning") || Get<bool>(_plans, "CanExecute"), Is.False);
        Assert.That(Field(_controller, "_hasPlanningRequestBeenSent").GetValue(_controller), Is.EqualTo(false));
    }

    [TestCase("focus")]
    [TestCase("pause")]
    [TestCase("tracking")]
    [TestCase("disabled")]
    public void InterruptedContextBumperDoesNotBlockTheNextCapture(string interruption)
    {
        GameObject viewer = New("Contact context viewer");
        viewer.AddComponent<Camera>();
        viewer.tag = "MainCamera";
        viewer.transform.position = new Vector3(0, 1, -2);
        Component input = Add("Contact input lifecycle", "URInputManager");
        Set(input, "_pickPlaceController", _controller);
        Set(input, "_trackingSpace", _tracking);
        Call(input, "UseTilePickPlace");
        Pose contact = new Pose(new Vector3(0.4f, 0.8f, 0.3f), Quaternion.Euler(12, 28, -9));
        Capture(contact);
        Call(_controller, "SetPointerPose", contact, _tracking);
        GameObject live = (GameObject)Field(_controller, "_livePointInstance").GetValue(_controller);
        Assert.That(live.activeSelf, Is.True);
        Component menu = Point(true).GetComponent(TypeOf("TargetPoseContextMenu"));
        Assert.That(Call(menu, "Open"), Is.EqualTo(true));

        MethodInfo bumper = input.GetType().GetMethod("HandleOnBumper", BindingFlags.Instance | BindingFlags.NonPublic);
        bumper.Invoke(input, new[] { Activator.CreateInstance(bumper.GetParameters()[0].ParameterType) });
        Assert.That(float.IsPositiveInfinity((float)Field(input, "_contextBumperConsumedUntil").GetValue(input)), Is.True,
            "A context bumper press consumes capture until release or an input interruption.");
        Set(input, "_robotPlacementRequested", true);

        switch (interruption)
        {
            case "focus": Private(input, "OnApplicationFocus", false); break;
            case "pause": Private(input, "OnApplicationPause", true); break;
            case "tracking": Private(input, "Update"); break;
            case "disabled": Private(input, "OnDisable"); break;
        }

        Assert.That(Field(input, "_contextBumperConsumedUntil").GetValue(input), Is.EqualTo(0f));
        Assert.That(Field(input, "_robotPlacementRequested").GetValue(input), Is.EqualTo(false));
        Assert.That((bool)TypeOf("TargetPoseContextMenu").GetProperty("IsOpen").GetValue(null), Is.False);
        Assert.That(live.activeSelf, Is.False);
        // No XR device is supplied: reaching its tracking check proves the stale context press no longer blocks capture.
        LogAssert.Expect(LogType.Warning, "Cannot capture a target without a tracked controller and a valid XR tracking frame.");
        Private(input, "Capture", true);
        AssertPose(Point(true).transform, contact);
    }

    private void Capture(Pose contact)
    {
        Call(_controller, "SetPickPlace", contact.position, contact.rotation);
        Call(_controller, "SetPlacePlace", contact.position + Vector3.right * 0.1f, contact.rotation);
    }
    private object Export()
    {
        object[] args = { null };
        Assert.That(CallArgs(_controller, "TryGetContactPoses", args), Is.EqualTo(true));
        return args[0];
    }
    private GameObject Point(bool pick) => (GameObject)Field(_controller, pick ? "_pickPointInstance" : "_placePointInstance").GetValue(_controller);
    private void AssertExport(object stamped, Pose expected)
    {
        Pose local = RobotFrameMath.RelativeTo(new Pose(_base.position, _base.rotation), expected);
        object pose = Field(stamped, "pose").GetValue(stamped);
        object position = Field(pose, "position").GetValue(pose);
        Assert.That(Number(position, "x"), Is.EqualTo(local.position.z).Within(0.000001));
        Assert.That(Number(position, "y"), Is.EqualTo(-local.position.x).Within(0.000001));
        Assert.That(Number(position, "z"), Is.EqualTo(local.position.y).Within(0.000001));
        object rotation = Field(pose, "orientation").GetValue(pose);
        double x = Number(rotation, "x"), y = Number(rotation, "y"), z = Number(rotation, "z"), w = Number(rotation, "w");
        Vector3 normalUnity = local.rotation * Vector3.up;
        Assert.That(2*(x*z+w*y), Is.EqualTo(normalUnity.z).Within(0.000002));
        Assert.That(2*(y*z-w*x), Is.EqualTo(-normalUnity.x).Within(0.000002));
        Assert.That(1-2*(x*x+y*y), Is.EqualTo(normalUnity.y).Within(0.000002));
    }
    private static double Number(object value, string name) => Convert.ToDouble(Field(value, name).GetValue(value));
    private GameObject New(string name) { GameObject value = new GameObject(name); _owned.Add(value); return value; }
    private Component Add(string name, string type) { Component value = New(name).AddComponent(TypeOf(type)); ((Behaviour)value).enabled = false; return value; }
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType(name)).First(t => t != null);
    private static FieldInfo Field(object value, string name) => value.GetType().GetField(name, BindingFlags.Instance|BindingFlags.Static|BindingFlags.Public|BindingFlags.NonPublic);
    private static void Set(object value, string name, object data) => Field(value, name).SetValue(value, data);
    private static T Get<T>(object value, string name) => (T)value.GetType().GetProperty(name).GetValue(value);
    private static object Call(object value, string name, params object[] args) => CallArgs(value, name, args);
    private static object CallArgs(object value, string name, object[] args) => value.GetType().GetMethods().Single(m => m.Name == name && m.GetParameters().Length == args.Length).Invoke(value,args);
    private static object Private(object value, string name, params object[] args) => value.GetType().GetMethod(name,BindingFlags.Instance|BindingFlags.NonPublic).Invoke(value,args);
    private static void AssertPose(Transform actual, Pose expected)
    {
        Assert.That(Vector3.Distance(actual.position,expected.position), Is.LessThan(0.000001f));
        Assert.That(Quaternion.Angle(actual.rotation,expected.rotation), Is.LessThan(0.001f));
    }
}
