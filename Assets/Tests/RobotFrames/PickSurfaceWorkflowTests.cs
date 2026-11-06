using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading;
using NUnit.Framework;
using UnityEngine;

/// <summary>Actual pick settings, corner capture, menus and ROS wire contract without robot or XR input.</summary>
public sealed class PickSurfaceWorkflowTests
{
    private const BindingFlags Instance = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
    private readonly List<GameObject> _owned = new List<GameObject>();
    private readonly List<GameObject> _busyOwners = new List<GameObject>();
    private readonly List<GameObject> _previousMainCameras = new List<GameObject>();
    private Component _controller, _plans, _coordinates, _tools, _fakeConnector;
    private GameObject _contourProvider;
    private Transform _tracking;
    private CancellationTokenSource _fakeCancellation;
    private bool _previousConnectionError;

    [SetUp]
    public void SetupActualPickControllerWithoutTransport()
    {
        Contour("SetEnabled", false); Contour("SetLiveRobot", new object[] { null });
        _contourProvider = New("Surface-test supported contour renderer");
        Contour("SetRenderingAvailable", _contourProvider, true);
        _coordinates = Add("Surface-test coordinates", "RobotCoordinateService");
        Add("Surface-test robot readiness", "RobotStateService");
        _tracking = New("Surface-test tracking").transform;
        _tracking.SetPositionAndRotation(new Vector3(1.2f, .3f, -.7f), Quaternion.Euler(0f, 27f, 0f));
        ArticulationBody body = New("Surface-test robot base").AddComponent<ArticulationBody>();
        body.transform.SetPositionAndRotation(new Vector3(.6f, -.1f, 1.4f), Quaternion.Euler(14f, -33f, 7f));
        Call(_coordinates, "Initialize", body, _tracking);
        _tools = Add("Surface-test matching suction tool", "ToolSelectionController");
        if (Read(_tools, "_catalog") == null) Call(_tools, "Awake");
        string description = Resources.Load<TextAsset>("RobotModels/kenos_sl.urdf").text;
        foreach (string service in new[] { "/robot_state_publisher/get_parameters", "/move_group/get_parameters", "/mtc_test_task_node/get_parameters" })
            Assert.That(Call(_tools, "InspectDescription", service, description), Is.EqualTo(true));
        _plans = Add("Surface-test MTC review", "MTCPlansManager");
        if (Read(_plans, "OnSuccessfulPlan") == null) Call(_plans, "Awake");
        _controller = Add("Surface-test pick contacts", "PickPlaceController");
        Set(_controller, "_plansManager", _plans);
    }

    [TearDown]
    public void Cleanup()
    {
        TypeOf("TargetPoseContextMenu").GetMethod("CloseAll").Invoke(null, null);
        foreach (GameObject owner in _busyOwners) if (owner != null) SetBusy(owner, false);
        _busyOwners.Clear();
        if (_fakeConnector != null)
        {
            Set(_fakeConnector, "m_ConnectionThreadCancellation", null);
            TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection").GetField("m_HasConnectionError", BindingFlags.Static | BindingFlags.NonPublic)
                .SetValue(null, _previousConnectionError);
        }
        _fakeCancellation?.Dispose(); _fakeCancellation = null;
        Component capture = _controller != null ? Get<Component>(_controller, "SurfaceCapture") : null;
        if (capture != null) { Call(capture, "OnDisable"); Call(capture, "OnDestroy"); }
        Assert.That(ItemDraws(), Is.Empty, "Destroying the capture must release its contour registration.");
        Contour("SetRenderingAvailable", _contourProvider, false);
        if (_controller != null) Call(_controller, "OnDestroy");
        foreach (GameObject value in _owned.AsEnumerable().Reverse())
            if (value != null) UnityEngine.Object.DestroyImmediate(value);
        _owned.Clear();
        foreach (GameObject camera in _previousMainCameras) if (camera != null) camera.tag = "MainCamera";
        _previousMainCameras.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty,
            "Pick settings, corner capture and guide rendering must never start a robot connection.");
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("PickSurfaceGuide")), Is.Empty);
    }

    [TestCase(5, .005d)]
    [TestCase(10, .010d)]
    [TestCase(20, .020d)]
    public void CenterPickupExportsTheSelectedCartesianDescentSpeedInMetresPerSecond(int speed, double metresPerSecond)
    {
        Assert.That(Get<int>(_controller, "DescentSpeedMmPerSecond"), Is.EqualTo(5));
        CaptureCenterPair();
        Assert.That(Call(_controller, "SetDescentSpeedMmPerSecond", speed), Is.EqualTo(true));
        object task = TaskRequest();
        Assert.That(Read(task, "cartesian_descent_speed_mps"), Is.EqualTo(metresPerSecond));
        Assert.That(Read(task, "item_type"), Is.EqualTo("tile"));
        Assert.That(Read(task, "pick_from_surface"), Is.EqualTo(false));
        Assert.That(((Array)Read(task, "top_corners")).Length, Is.EqualTo(3));
        object[] contacts = { null };
        Assert.That(Call(_controller, "TryGetContactPoses", contacts), Is.EqualTo(true));
        foreach (string target in new[] { "pick_pose", "place_pose" })
        {
            object actual = Read(Read(Read(task, "contacts"), target), "pose");
            object expected = Read(Read(contacts[0], target), "pose");
            AssertRosPoint(Read(actual, "position"), Read(expected, "position"));
            foreach (string axis in new[] { "x", "y", "z", "w" })
                Assert.That(Read(Read(actual, "orientation"), axis), Is.EqualTo(Read(Read(expected, "orientation"), axis)));
        }
    }

    [TestCase(-5)]
    [TestCase(0)]
    [TestCase(7)]
    [TestCase(100)]
    public void UnsupportedSpeedsAndBusySettingsCannotChangeTheRequestedTask(int invalidSpeed)
    {
        Assert.That(Call(_controller, "SetDescentSpeedMmPerSecond", invalidSpeed), Is.EqualTo(false));
        Assert.That(Get<int>(_controller, "DescentSpeedMmPerSecond"), Is.EqualTo(5));
        Assert.That(Call(_controller, "SetSelectedItem", (PickItem)99), Is.EqualTo(false));
        GameObject owner = New("Surface-test execution lock"); _busyOwners.Add(owner); SetBusy(owner, true);
        Assert.That(Call(_controller, "SetDescentSpeedMmPerSecond", 20), Is.EqualTo(false));
        Assert.That(Call(_controller, "SetPickBySurface", true), Is.EqualTo(false));
        Assert.That(Call(_controller, "SetSelectedItem", PickItem.Brick), Is.EqualTo(false));
        Assert.That(Get<int>(_controller, "DescentSpeedMmPerSecond"), Is.EqualTo(5));
        Assert.That(Get<bool>(_controller, "PickBySurface"), Is.False);
        Assert.That(Call(Capture(), "BeginCapture", PickItem.Tile), Is.EqualTo(false));
    }

    [Test]
    public void ChangingSpeedRevokesPendingAndRetainedReviewPermission()
    {
        CaptureCenterPair(); object task = TaskRequest();
        Set(_controller, "_checkingCapability", true); Set(_controller, "_requestedTask", task);
        Set(_controller, "_requestedContactPoses", Read(task, "contacts")); Set(_controller, "_hasPlanningRequestBeenSent", true);
        Set(_plans, "_hasPlanningRequest", true); Set(_plans, "_acceptingResults", true);
        object repeat = Read(_plans, "_repeat");
        Call(repeat, "CaptureStart", new[] { "a", "b", "c", "d", "e", "f" }, new double[6]);
        Call(repeat, "Complete", true, true, "surface-test-retained-plan", "");
        Assert.That(Get<bool>(_plans, "HasRepeatablePlan"), Is.True);
        int generation = (int)Read(_controller, "_generation");
        Assert.That(Call(_controller, "SetDescentSpeedMmPerSecond", 10), Is.EqualTo(true));
        Assert.That((int)Read(_controller, "_generation"), Is.GreaterThan(generation));
        Assert.That(Read(_controller, "_requestedTask"), Is.Null);
        Assert.That(Read(_controller, "_checkingCapability"), Is.EqualTo(false));
        Assert.That(Read(_controller, "_hasPlanningRequestBeenSent"), Is.EqualTo(false));
        Assert.That(Get<bool>(_plans, "IsPlanning"), Is.False);
        Assert.That(Get<bool>(_plans, "HasRepeatablePlan"), Is.False);
    }

    [Test]
    public void MerelySelectingSurfaceModeDoesNotStartOrDisplayACornerGuide()
    {
        Assert.That(Call(_controller, "SetPickBySurface", true), Is.EqualTo(true));
        Assert.That(Get<Component>(_controller, "SurfaceCapture"), Is.Null);
        Component capture = Capture();
        Assert.That(Get<bool>(capture, "IsCapturing"), Is.False);
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("PickSurfaceGuide")), Is.Empty);
    }

    [TestCase(PickItem.Tile)]
    [TestCase(PickItem.Brick)]
    public void ThreeConsecutiveCornersReplaceOnlyThePickAndExportTheirMeasuredCoordinates(PickItem item)
    {
        CaptureCenterPair(); Pose place = PointPose(false);
        BeginSurface(item); Component capture = Capture();
        Vector3[] corners = WorldCorners(item);
        for (int i = 0; i < 3; i++)
        {
            Assert.That(Call(capture, "CapturePointerPose", new Pose(corners[i], Quaternion.identity), _tracking), Is.EqualTo(true), Get<string>(capture, "StatusText"));
            AssertPose(PointPose(false), place);
            if (i < 2) Assert.That(Get<bool>(_controller, "HasValidSurface"), Is.False);
        }
        Assert.That(Get<bool>(capture, "IsCapturing"), Is.False);
        Assert.That(Get<bool>(_controller, "HasValidSurface"), Is.True);
        AssertPose(PointPose(true), PickSurfaceGeometry.GetPickTcpPose(SurfaceWorld));
        object task = TaskRequest();
        Assert.That(Read(task, "pick_from_surface"), Is.EqualTo(true));
        Assert.That(Read(task, "item_type"), Is.EqualTo(item == PickItem.Tile ? "tile" : "brick"));
        Array exported = (Array)Read(task, "top_corners");
        for (int i = 0; i < 3; i++)
        {
            object[] local = { new Pose(corners[i], Quaternion.identity), Pose.identity };
            Assert.That(Call(_coordinates, "TryConvertWorldPoseToLocalPose", local), Is.EqualTo(true));
            Vector3 actual = RosUnityPoint(exported.GetValue(i));
            Assert.That(Vector3.Distance(actual, ((Pose)local[1]).position), Is.LessThan(1e-5f));
        }
    }

    [TestCase(PickItem.Tile, .200f, .007f, .200f)]
    [TestCase(PickItem.Brick, .185f, .135f, .280f)]
    public void AcceptedItemDrawsItsFullBodyAtTheFittedPhysicalPoseWithoutEnablingRobotContours(
        PickItem item, float width, float thickness, float length)
    {
        BeginSurface(item); Component capture = Capture();
        Vector3[] corners = WorldCorners(item);
        Assert.That(ItemDraws(), Is.Empty);
        for (int index = 0; index < 3; index++)
        {
            Assert.That(Call(capture, "CapturePointerPose", new Pose(corners[index], Quaternion.identity), _tracking), Is.EqualTo(true));
            if (index < 2) Assert.That(ItemDraws(), Is.Empty, "Partial contacts must not display an accepted item.");
        }
        IList draws = ItemDraws();
        Assert.That(draws.Count, Is.EqualTo(1), "The accepted item must render even with no live robot and its contour option off.");
        Assert.That(ContourProperty("Enabled"), Is.EqualTo(false));
        Matrix4x4 matrix = DrawMatrix(draws[0]);
        Assert.That(Vector3.Distance(matrix.MultiplyPoint3x4(Vector3.up * .5f), SurfaceWorld.position), Is.LessThan(1e-5f));
        Assert.That(Vector3.Distance(matrix.MultiplyVector(Vector3.right), SurfaceWorld.rotation * Vector3.right * width), Is.LessThan(1e-5f));
        Assert.That(Vector3.Distance(matrix.MultiplyVector(Vector3.up), SurfaceWorld.rotation * Vector3.up * thickness), Is.LessThan(1e-5f));
        Assert.That(Vector3.Distance(matrix.MultiplyVector(Vector3.forward), SurfaceWorld.rotation * Vector3.forward * length), Is.LessThan(1e-5f));
        Assert.That(_tracking.GetComponentsInChildren<Collider>(true), Is.Empty);
        Assert.That(_tracking.GetComponentsInChildren<ArticulationBody>(true), Is.Empty);
        _tracking.position += new Vector3(.3f, .2f, -.1f);
        Assert.That(Vector3.Distance(DrawMatrix(ItemDraws()[0]).MultiplyPoint3x4(Vector3.up * .5f), SurfaceWorld.position), Is.LessThan(1e-5f),
            "The item follows the same tracking parent as the accepted contacts.");
    }

    [TestCase("method")]
    [TestCase("item")]
    [TestCase("workflow")]
    [TestCase("disable")]
    public void AcceptedItemHidesWhenThePickTargetIsInapplicableAndReturnsWithTheSameSettings(string change)
    {
        CompleteSurface(PickItem.Tile); Component capture = Capture();
        Assert.That(ItemDraws(), Is.Not.Empty);
        if (change == "method") Call(_controller, "SetPickBySurface", false);
        else if (change == "item") Call(_controller, "SetSelectedItem", PickItem.Brick);
        else if (change == "workflow") Call(_controller, "SetTilePlanningEnabled", false);
        else Call(capture, "OnDisable");
        Assert.That(ItemDraws(), Is.Empty);
        if (change == "method") Call(_controller, "SetPickBySurface", true);
        else if (change == "item") Call(_controller, "SetSelectedItem", PickItem.Tile);
        else if (change == "workflow") Call(_controller, "SetTilePlanningEnabled", true);
        else { Call(capture, "OnEnable"); Call(capture, "Update"); }
        Assert.That(ItemDraws(), Is.Not.Empty);
    }

    [Test]
    public void CancelAndRejectedReplacementFitsPreserveThePreviouslyAcceptedPickAndPlace()
    {
        CaptureCenterPair(); CompleteSurface(PickItem.Tile);
        Pose pick = PointPose(true), place = PointPose(false);
        Assert.That(ItemDraws(), Is.Not.Empty);
        Matrix4x4 acceptedBody = DrawMatrix(ItemDraws()[0]);
        Component capture = Capture(); BeginSurface(PickItem.Tile);
        Vector3[] corners = WorldCorners(PickItem.Tile);
        Assert.That(Call(capture, "CapturePointerPose", new Pose(corners[0]+Vector3.right, Quaternion.identity), _tracking), Is.EqualTo(true));
        Call(capture, "Cancel");
        Assert.That(DrawMatrix(ItemDraws()[0]), Is.EqualTo(acceptedBody));
        AssertPose(PointPose(true), pick); AssertPose(PointPose(false), place);
        Assert.That(Get<bool>(_controller, "HasValidSurface"), Is.True);
        BeginSurface(PickItem.Tile);
        Assert.That(Call(capture, "CapturePointerPose", new Pose(corners[0], Quaternion.identity), _tracking), Is.EqualTo(true));
        Assert.That(Call(capture, "CapturePointerPose", new Pose(corners[1], Quaternion.identity), _tracking), Is.EqualTo(true));
        Assert.That(Call(capture, "CapturePointerPose", new Pose(corners[2]+Vector3.up*.15f, Quaternion.identity), _tracking), Is.EqualTo(false));
        AssertPose(PointPose(true), pick); AssertPose(PointPose(false), place);
        Assert.That(Get<bool>(capture, "IsCapturing"), Is.True);
        Assert.That(Get<bool>(_controller, "HasValidSurface"), Is.False);
        Assert.That(DrawMatrix(ItemDraws()[0]), Is.EqualTo(acceptedBody), "A rejected replacement must retain the last accepted full body.");
        Call(capture, "Cancel"); Assert.That(Get<bool>(_controller, "HasValidSurface"), Is.True);
        AssertPose(PointPose(true), pick);
    }

    [Test]
    public void FloatingGuideHighlightsTheNextCornerAndContainsNoCollisionOrRobotComponents()
    {
        BeginSurface(PickItem.Brick); Component capture = Capture();
        Vector3[] corners = WorldCorners(PickItem.Brick);
        Component guide = (Component)Read(capture, "_guide");
        Assert.That(Get<bool>(guide, "HasVisual"), Is.True);
        for (int i = 0; i < 3; i++)
        {
            Assert.That(Get<int>(guide, "SelectedPointIndex"), Is.EqualTo(i));
            GameObject geometry = (GameObject)Read(guide, "_geometry");
            Renderer[] marks = Enumerable.Range(1, 3).Select(index => geometry.transform.Find("Capture corner " + index).GetComponent<Renderer>()).ToArray();
            Assert.That(marks[i].sharedMaterial, Is.Not.SameAs(marks[(i+1)%3].sharedMaterial));
            Assert.That(guide.GetComponentsInChildren<Collider>(true), Is.Empty);
            Assert.That(guide.GetComponentsInChildren<ArticulationBody>(true), Is.Empty);
            Assert.That(Call(capture, "CapturePointerPose", new Pose(corners[i], Quaternion.identity), _tracking), Is.EqualTo(true));
        }
        Assert.That(Get<bool>(guide, "HasVisual"), Is.False);
    }

    [TestCase("model")]
    [TestCase("calibration")]
    [TestCase("tracking")]
    public void AChangedReferenceContextInvalidatesAnAcceptedSurfaceTask(string change)
    {
        CaptureCenterPair(); CompleteSurface(PickItem.Tile); Component capture = Capture();
        Assert.That(Get<bool>(_controller, "HasValidSurface"), Is.True);
        if (change == "model") Call(_tools, "SelectTool", "kenos");
        else if (change == "calibration") Call(_coordinates, "BeginCalibrationEdit");
        else Call(capture, "OnTrackingOriginUpdated", new object[] { null });
        Call(capture, "Update");
        Assert.That(ItemDraws(), Is.Empty, "Invalid calibration, tool or tracking must remove the item outline.");
        Assert.That(Get<bool>(_controller, "HasValidSurface"), Is.False);
        object[] request = { null, null };
        Assert.That(Call(_controller, "TryGetTaskRequest", request), Is.EqualTo(false));
        Assert.That(request[0], Is.Null);
    }

    [Test]
    public void PhysicalTipOffsetIsAppliedBeforeStoringSurfaceContacts()
    {
        Call(_controller, "SetPickBySurface", true); Component capture = Capture();
        Vector3 offset = new Vector3(.012f, -.035f, .041f); Call(capture, "SetTipOffset", offset);
        Assert.That(Call(capture, "BeginCapture", PickItem.Tile), Is.EqualTo(true));
        Quaternion pointerRotation = Quaternion.Euler(15f, -28f, 9f);
        Vector3[] corners = WorldCorners(PickItem.Tile);
        foreach (Vector3 corner in corners)
            Assert.That(Call(capture, "CapturePointerPose", new Pose(corner-pointerRotation*offset, pointerRotation), _tracking), Is.EqualTo(true));
        Vector3[] actual = (Vector3[])Call(capture, "GetWorldCorners");
        for (int i = 0; i < 3; i++) Assert.That(Vector3.Distance(actual[i], corners[i]), Is.LessThan(1e-5f));
    }

    [Test]
    public void UnsupportedTaskCapabilityFailsClosedWithoutPublishingALegacyOrNewPlan()
    {
        CaptureCenterPair(); object task = TaskRequest();
        Assert.That(TypeOf("PickPlaceController").GetField("TaskPlanTopic").GetRawConstantValue(), Is.EqualTo("/pick_place_task_target"));
        Assert.That(TypeOf("PickPlaceController").GetField("CapabilityParameter", BindingFlags.Static | BindingFlags.NonPublic).GetRawConstantValue(),
            Is.EqualTo("pick_place_task_planning_enabled"));
        // A nonnull cancellation token makes the real connector's connection
        // predicate true; no Connect, Start, socket or network thread is run.
        _fakeConnector = Add("Surface-test disabled capability transport", "Unity.Robotics.ROSTCPConnector.ROSConnection", false);
        ((Behaviour)_fakeConnector).enabled = false;
        _fakeConnector.GetType().GetProperty("ConnectOnStart").SetValue(_fakeConnector, false);
        _fakeConnector.GetType().GetProperty("ShowHud").SetValue(_fakeConnector, false); Call(_fakeConnector, "Awake");
        FieldInfo error = _fakeConnector.GetType().GetField("m_HasConnectionError", BindingFlags.Static | BindingFlags.NonPublic);
        _previousConnectionError = (bool)error.GetValue(null); error.SetValue(null, false);
        _fakeCancellation = new CancellationTokenSource(); Set(_fakeConnector, "m_ConnectionThreadCancellation", _fakeCancellation);
        Set(_controller, "_connection", _fakeConnector); Set(_controller, "_checkingCapability", true);
        Set(_controller, "_requestedTask", task); Set(_controller, "_requestedContactPoses", Read(task, "contacts"));
        Set(_controller, "_requestCalibrationRevision", Get<int>(_coordinates, "CalibrationRevision"));
        object response = Msg("RclInterfaces.GetParametersResponse"), value = Msg("RclInterfaces.ParameterValueMsg");
        Set(value, "type", (byte)1); Set(value, "bool_value", false); ArrayField(response, "values", value);
        Call(_controller, "OnCapabilityResponse", response, (int)Read(_controller, "_generation"));
        Assert.That(Read(_controller, "_checkingCapability"), Is.EqualTo(false));
        Assert.That(Read(_controller, "_hasPlanningRequestBeenSent"), Is.EqualTo(false));
        Assert.That(Read(_controller, "_requestedTask"), Is.Null);
        Assert.That(Get<string>(_plans, "StatusText"), Does.Contain("item/speed pick update"));
        Assert.That(Get<bool>(_plans, "IsPlanning"), Is.False);
        Assert.That(Call(_fakeConnector, "GetTopic", "/pick_place_task_target"), Is.Null);
        Assert.That(Call(_fakeConnector, "GetTopic", "/pick_place_tcp_target"), Is.Null);
    }

    [Test]
    public void SettingsMenuBuildsWithoutTransportAndItemSelectionStartsTheActualGuide()
    {
        Component menu = BuildSettingsMenu();
        RectTransform view = (RectTransform)Read(menu, "_pickSettingsView");
        foreach (string name in new[] { "Pick by center", "Pick by surface", "Pick tile", "Pick brick",
            "Pick descent 5 mm per second", "Pick descent 10 mm per second", "Pick descent 20 mm per second" })
            Assert.That(view.Find(name), Is.Not.Null, name);
        Assert.That(Get<Component>(_controller, "SurfaceCapture"), Is.Null);
        Call(menu, "OpenPickSettings"); Call(menu, "SelectPickMethod", true);
        Click((Component)Read(menu, "_pickBrickButton"));
        Component capture = Capture();
        Assert.That(Get<bool>(capture, "IsCapturing"), Is.True);
        Assert.That(Get<PickItem>(capture, "Item"), Is.EqualTo(PickItem.Brick));
        Assert.That(Get<bool>((Component)Read(capture, "_guide"), "HasVisual"), Is.True);
        Click((Component)Read(menu, "_pickMediumButton"));
        Assert.That(Get<int>(_controller, "DescentSpeedMmPerSecond"), Is.EqualTo(10));
        Call(menu, "CancelPickSurface");
        Click((Component)Read(menu, "_pickTileButton"));
        Assert.That(Get<PickItem>(capture, "Item"), Is.EqualTo(PickItem.Tile));
    }

    [Serializable] private sealed class CdrDocument { public CdrFixture[] fixtures; }
    [Serializable] private sealed class CdrFixture
    {
        public string name, base64, canonicalBase64, item_type;
        public int[] paddingOffsets;
        public double cartesian_descent_speed_mps;
        public bool pick_from_surface;
        public CdrPoint[] top_corners;
    }
    [Serializable] private sealed class CdrPoint { public double x, y, z; }

    [Test]
    public void TaskMessageInteroperatesWithRosCdrAndCarriesExactlyThreeUnprefixedCorners()
    {
        TextAsset resource = Resources.Load<TextAsset>("PickPlaceTaskCdr"); Assert.That(resource, Is.Not.Null);
        CdrDocument document = JsonUtility.FromJson<CdrDocument>(resource.text);
        Assert.That(document.fixtures, Has.Length.EqualTo(4));
        foreach (CdrFixture fixture in document.fixtures)
        {
            byte[] wire = Convert.FromBase64String(fixture.base64);
            object reader = Activator.CreateInstance(TypeOf("Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageDeserializer"));
            Call(reader, "InitWithBuffer", wire);
            object task = TypeOf("RosMessageTypes.Ur16eInterfaces.PickPlaceTaskMsg")
                .GetMethod("Deserialize", BindingFlags.Public | BindingFlags.Static | BindingFlags.DeclaredOnly).Invoke(null, new[] { reader });
            Assert.That(Read(task, "cartesian_descent_speed_mps"), Is.EqualTo(fixture.cartesian_descent_speed_mps), fixture.name);
            Assert.That(Read(task, "item_type"), Is.EqualTo(fixture.item_type));
            Assert.That(Read(task, "pick_from_surface"), Is.EqualTo(fixture.pick_from_surface));
            Array corners = (Array)Read(task, "top_corners"); Assert.That(corners.Length, Is.EqualTo(3));
            for (int i = 0; i < 3; i++)
                foreach (string axis in new[] { "x", "y", "z" })
                    // JSON metadata can round a decimal by one ULP. The CDR byte comparison below remains exact.
                    Assert.That((double)Read(corners.GetValue(i), axis), Is.EqualTo((double)Read(fixture.top_corners[i], axis)).Within(1e-12));
            Assert.That(Read(reader, "offset"), Is.EqualTo(wire.Length));
            object writer = Activator.CreateInstance(TypeOf("Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageSerializer"));
            Call(writer, "SerializeMessage", task); byte[] actual = (byte[])Call(writer, "GetBytes");
            byte[] expected = Convert.FromBase64String(fixture.canonicalBase64);
            Assert.That(actual.Length, Is.EqualTo(expected.Length));
            foreach (int offset in fixture.paddingOffsets) actual[offset] = 0;
            CollectionAssert.AreEqual(expected, actual, fixture.name + ": C# must match ROS's fixed-array CDR layout.");
            for (int i = 0; i < 9; i++)
                Assert.That(BitConverter.ToDouble(actual, actual.Length-72+i*8), Is.EqualTo(i%3 == 0
                    ? fixture.top_corners[i/3].x : i%3 == 1 ? fixture.top_corners[i/3].y : fixture.top_corners[i/3].z).Within(1e-12));
        }
    }

    [Test]
    public void TaskSerializerRejectsMissingOrVariableLengthCornerArrays()
    {
        object task = Msg("Ur16eInterfaces.PickPlaceTaskMsg");
        object writer = Activator.CreateInstance(TypeOf("Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageSerializer"));
        ArrayField(task, "top_corners", Msg("Geometry.PointMsg"), Msg("Geometry.PointMsg"));
        TargetInvocationException error = Assert.Throws<TargetInvocationException>(() => Call(writer, "SerializeMessage", task));
        Assert.That(error.InnerException, Is.TypeOf<InvalidOperationException>());
    }

    /// <summary>Actual settings and floating guide with synthetic contacts, no input/transport Start calls.</summary>
    public static void RenderPickSurfacePreviews()
    {
        var fixture = new PickSurfaceWorkflowTests();
        RenderTexture target = null, previous = RenderTexture.active; Texture2D pixels = null;
        try
        {
            fixture.SetupActualPickControllerWithoutTransport();
            foreach (Camera prior in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                if (prior.CompareTag("MainCamera")) { fixture._previousMainCameras.Add(prior.gameObject); prior.gameObject.tag = "Untagged"; }
            Camera camera = fixture.New("Pick surface preview camera").AddComponent<Camera>(); camera.tag = "MainCamera";
            camera.transform.SetPositionAndRotation(new Vector3(.158f, 0f, -1f), Quaternion.identity);
            camera.orthographic = true; camera.orthographicSize = .415f; camera.aspect = 1f;
            camera.clearFlags = CameraClearFlags.SolidColor; camera.backgroundColor = new Color(.08f, .11f, .15f);
            target = new RenderTexture(1152, 1152, 24, RenderTextureFormat.ARGB32);
            pixels = new Texture2D(1152, 1152, TextureFormat.RGB24, false); camera.targetTexture = target;
            Component menu = fixture.BuildSettingsMenu(); Canvas canvas = (Canvas)Read(menu, "_canvas");
            canvas.worldCamera = camera; canvas.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            ((RectTransform)Read(menu, "_panel")).gameObject.SetActive(true); Call(menu, "OpenPickSettings");
            string directory = Path.GetFullPath("PickSurfacePreviews"); Directory.CreateDirectory(directory);
            Type textType = TypeOf("TMPro.TMP_Text"), fontType = TypeOf("TMPro.TMP_FontAsset");
            UnityEngine.Object font = Resources.Load("Fonts & Materials/LiberationSans SDF", fontType);
            foreach (PickItem item in new[] { PickItem.Tile, PickItem.Brick })
            {
                Call(menu, "SelectPickMethod", true); Call(menu, "SelectPickItem", item);
                Component capture = fixture.Capture(), guide = (Component)Read(capture, "_guide");
                guide.transform.SetPositionAndRotation(new Vector3(.36f, -.02f, 0f), Quaternion.identity);
                Set(guide, "_positioned", true);
                for (int index = 0; index < 3; index++)
                {
                    if (index > 0) Call(capture, "CapturePointerPose", new Pose(fixture.WorldCorners(item)[index-1], Quaternion.identity), fixture._tracking);
                    Call(menu, "RefreshTask"); Call(guide, "LateUpdate");
                    Component[] labels = canvas.GetComponentsInChildren(textType, true).Concat(guide.GetComponentsInChildren(textType, true)).ToArray();
                    foreach (Component label in labels) if (font != null) textType.GetProperty("font").SetValue(label, font);
                    Canvas.ForceUpdateCanvases();
                    foreach (Component label in labels)
                    {
                        textType.GetMethod("ForceMeshUpdate").Invoke(label, new object[] { true, true });
                        if (label.gameObject.activeInHierarchy && (bool)textType.GetProperty("isTextOverflowing").GetValue(label))
                            throw new InvalidOperationException("Pick surface preview text overflow: " + label.name);
                    }
                    camera.Render(); RenderTexture.active = target;
                    pixels.ReadPixels(new Rect(0f, 0f, 1152f, 1152f), 0, 0); pixels.Apply();
                    File.WriteAllBytes(Path.Combine(directory, item.ToString().ToLowerInvariant()+"-corner-"+(index+1)+".png"), pixels.EncodeToPNG());
                }
                Call(menu, "CancelPickSurface");
            }
            camera.targetTexture = null;
        }
        finally
        {
            RenderTexture.active = previous;
            if (pixels != null) UnityEngine.Object.DestroyImmediate(pixels);
            if (target != null) { target.Release(); UnityEngine.Object.DestroyImmediate(target); }
            fixture.Cleanup();
        }
    }

    private Component BuildSettingsMenu()
    {
        Component input = Add("Surface-test input routing", "URInputManager");
        Set(input, "_pickPlaceController", _controller); Set(input, "_trackingSpace", _tracking);
        Component menu = Add("Surface-test pick menu", "ToolSelectionMenu");
        Set(menu, "_input", input); Set(menu, "_controller", _tools); Set(menu, "_legacyPlans", _plans);
        Type mode = TypeOf("URInputManager").GetNestedType("WorkflowMode");
        Call(menu, "ConfigureWorkflow", Enum.Parse(mode, "SimplePickAndPlace"));
        Call(menu, "BuildMenu"); _owned.Add(((Canvas)Read(menu, "_canvas")).gameObject);
        Call(menu, "BindTaskControls"); Call(menu, "RefreshTask");
        return menu;
    }
    private void CaptureCenterPair()
    {
        Call(_controller, "SetPickPlace", new Vector3(.4f, .7f, -.2f), Quaternion.Euler(21f, -42f, 8f));
        Call(_controller, "SetPlacePlace", new Vector3(.7f, .6f, .3f), Quaternion.Euler(12f, 17f, -3f));
    }
    private Component Capture() => (Component)Call(_controller, "GetOrCreateSurfaceCapture");
    private void BeginSurface(PickItem item)
    {
        Assert.That(Call(_controller, "SetPickBySurface", true), Is.EqualTo(true));
        Assert.That(Call(_controller, "SetSelectedItem", item), Is.EqualTo(true));
        Component capture = Capture(); Call(capture, "SetTipOffset", Vector3.zero);
        Assert.That(Call(capture, "BeginCapture", item), Is.EqualTo(true), Get<string>(capture, "StatusText"));
    }
    private void CompleteSurface(PickItem item)
    {
        BeginSurface(item); Component capture = Capture();
        foreach (Vector3 point in WorldCorners(item))
            Assert.That(Call(capture, "CapturePointerPose", new Pose(point, Quaternion.identity), _tracking), Is.EqualTo(true), Get<string>(capture, "StatusText"));
    }
    internal MeshRenderer CreateAcceptedItemForRendering(PickItem item)
    {
        CompleteSurface(item);
        return _tracking.GetComponentsInChildren<MeshRenderer>(true).Single(renderer => renderer.name == "Captured item body");
    }
    private Pose SurfaceWorld => RobotFrameMath.Compose(new Pose(_tracking.position, _tracking.rotation),
        new Pose(new Vector3(.4f, .5f, .8f), Quaternion.Euler(15f, 33f, -9f)));
    private Vector3[] WorldCorners(PickItem item) => PickSurfaceGeometry.GetCanonicalCorners(item)
        .Select(point => SurfaceWorld.position+SurfaceWorld.rotation*point).ToArray();
    private Pose PointPose(bool pick)
    {
        Transform point = ((GameObject)Read(_controller, pick ? "_pickPointInstance" : "_placePointInstance")).transform;
        return new Pose(point.position, point.rotation);
    }
    private object TaskRequest()
    {
        object[] result = { null, null }; Assert.That(Call(_controller, "TryGetTaskRequest", result), Is.EqualTo(true), result[1] as string);
        return result[0];
    }
    private Component Add(string name, string type, bool active = true) => New(name, active).AddComponent(TypeOf(type));
    private GameObject New(string name, bool active = true) { var value = new GameObject(name); value.SetActive(active); _owned.Add(value); return value; }
    private static void SetBusy(GameObject owner, bool busy) => TypeOf("RobotExecutionState").GetMethod("SetBusy").Invoke(null, new object[] { owner, busy });
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(assembly => assembly.GetType(name, false)).First(type => type != null);
    private static object Msg(string name) => Activator.CreateInstance(TypeOf("RosMessageTypes."+name));
    private static object Read(object value, string field) => value.GetType().GetField(field, Instance).GetValue(value);
    private static object Contour(string name, params object[] values) => TypeOf("RobotContourDisplay").GetMethod(name).Invoke(null, values);
    private static object ContourProperty(string name) => TypeOf("RobotContourDisplay").GetProperty(name).GetValue(null);
    private static IList ItemDraws()
    {
        var draws = (IList)Activator.CreateInstance(typeof(List<>).MakeGenericType(TypeOf("RobotContourDraw")));
        Contour("TryGetDraws", draws); return draws;
    }
    private static Matrix4x4 DrawMatrix(object draw) => (Matrix4x4)draw.GetType().GetField("LocalToWorld").GetValue(draw);
    private static void Set(object value, string field, object data) => value.GetType().GetField(field, Instance).SetValue(value, data);
    private static T Get<T>(object value, string name) => (T)value.GetType().GetProperty(name).GetValue(value);
    private static object Call(object value, string method, params object[] args) => value.GetType().GetMethods(Instance)
        .Single(candidate => candidate.Name == method && candidate.GetParameters().Length == args.Length &&
            candidate.GetParameters().Select((parameter, i) => args[i] == null ||
                (parameter.ParameterType.IsByRef ? parameter.ParameterType.GetElementType() : parameter.ParameterType).IsInstanceOfType(args[i])).All(compatible => compatible))
        .Invoke(value, args);
    private static void ArrayField(object value, string field, params object[] items)
    {
        Type element = value.GetType().GetField(field).FieldType.GetElementType(); Array array = Array.CreateInstance(element, items.Length);
        for (int i = 0; i < items.Length; i++) array.SetValue(items[i], i); Set(value, field, array);
    }
    private static void Click(Component button)
    {
        object clicked = button.GetType().GetProperty("onClick").GetValue(button); clicked.GetType().GetMethod("Invoke").Invoke(clicked, null);
    }
    private static Vector3 RosUnityPoint(object point) => PickSurfaceGeometry.FromRosPoint((double)Read(point, "x"), (double)Read(point, "y"), (double)Read(point, "z"));
    private static void AssertRosPoint(object actual, object expected)
    {
        foreach (string axis in new[] { "x", "y", "z" }) Assert.That(Read(actual, axis), Is.EqualTo(Read(expected, axis)));
    }
    private static void AssertPose(Pose actual, Pose expected)
    {
        Assert.That(Vector3.Distance(actual.position, expected.position), Is.LessThan(1e-5f));
        Assert.That(Vector3.Distance(actual.rotation*Vector3.up, expected.rotation*Vector3.up), Is.LessThan(1e-5f));
        Assert.That(Vector3.Distance(actual.rotation*Vector3.forward, expected.rotation*Vector3.forward), Is.LessThan(1e-5f));
    }
}
