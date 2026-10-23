using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>Advances the actual world menu's follow loop without ROS, XR hardware or runtime Start.</summary>
public sealed class MenuFollowTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private readonly List<GameObject> _previousMainCameras = new List<GameObject>();
    private Component _menu;
    private Camera _camera;
    private Transform _canvas;
    private GameObject _panel;

    [SetUp]
    public void CreateAnIndependentCameraAndTheActualWorldCanvas()
    {
        foreach (Camera existing in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (existing.CompareTag("MainCamera"))
            {
                _previousMainCameras.Add(existing.gameObject);
                existing.gameObject.tag = "Untagged";
            }
        GameObject view = New("Follow-test user view");
        _camera = view.AddComponent<Camera>();
        view.tag = "MainCamera";
        view.transform.SetPositionAndRotation(new Vector3(0f, 1.6f, 0f), Quaternion.identity);
        _camera.fieldOfView = 60f;
        _camera.aspect = 1.5f;
        Assert.That(Camera.main, Is.EqualTo(_camera));
        GameObject owner = New("Follow-test menu owner");
        owner.SetActive(false);
        _menu = owner.AddComponent(TypeOf("ToolSelectionMenu"));
        Private("BuildMenu");
        _canvas = Get<Canvas>("_canvas").transform;
        _panel = Get<RectTransform>("_panel").gameObject;
        owner.SetActive(true);
        _panel.SetActive(true);
        Call("RecenterMenu");
        Assert.That(GetProperty<bool>(_menu, "IsVisible"), Is.True);
    }

    [TearDown]
    public void CleanupDetachedCanvasAndRestoreThePreviousCameraTags()
    {
        if (_canvas != null) UnityEngine.Object.DestroyImmediate(_canvas.gameObject);
        for (int index = _owned.Count - 1; index >= 0; index--)
            if (_owned[index] != null) UnityEngine.Object.DestroyImmediate(_owned[index]);
        _owned.Clear();
        foreach (GameObject previous in _previousMainCameras)
            if (previous != null) previous.tag = "MainCamera";
        _previousMainCameras.Clear();
        Type connection = TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection");
        Assert.That(Resources.FindObjectsOfTypeAll(connection), Is.Empty, "Following the user must not request ROS transport.");
    }

    [Test]
    public void RecenterPlacesAnUprightMenuOneMetreAwayAndTwentyDegreesToTheRight()
    {
        _camera.transform.rotation = Quaternion.Euler(25f, 40f, 22f);
        Call("RecenterMenu");
        Vector3 offset = _canvas.position - _camera.transform.position;
        Vector3 horizontalView = Vector3.ProjectOnPlane(_camera.transform.forward, Vector3.up).normalized;
        Assert.That(Vector3.ProjectOnPlane(offset, Vector3.up).magnitude, Is.EqualTo(1f).Within(0.00001f));
        Assert.That(offset.y, Is.EqualTo(-0.15f).Within(0.00001f));
        Assert.That(Vector3.SignedAngle(horizontalView, Vector3.ProjectOnPlane(offset, Vector3.up), Vector3.up), Is.EqualTo(20f).Within(0.001f));
        Assert.That(Vector3.Dot(_canvas.forward, offset.normalized), Is.GreaterThan(0.99999f));
        Assert.That(Mathf.Abs(_canvas.right.y), Is.LessThan(0.00001f), "The panel must not inherit head roll.");
        Assert.That(_canvas.parent, Is.Null, "Following remains a world-space operation.");
    }

    [Test]
    public void SmallHeadMotionInsideTheComfortZoneLeavesTheMenuStationary()
    {
        Pose original = PoseOf(_canvas);
        _camera.transform.position += new Vector3(0.02f, 0.01f, 0f);
        _camera.transform.rotation = Quaternion.Euler(0f, 15f, 0f);
        Advance(4f, 0.02f);
        AssertPose(PoseOf(_canvas), original);
    }

    [Test]
    public void ABriefExcursionDoesNotStartPositionFollowing()
    {
        Vector3 original = _canvas.position;
        _camera.transform.position += Vector3.back * 3f;
        Advance(0.6f, 0.01f);
        Assert.That(Vector3.Distance(_canvas.position, original), Is.LessThan(0.000001f));
        Advance(0.1f, 0.01f);
        Assert.That(Vector3.Distance(_canvas.position, original), Is.GreaterThan(0.001f));
        Assert.That(Vector3.Distance(_canvas.position, original), Is.LessThan(0.04f));
    }

    [Test]
    public void FarFollowingAndFacingAreGradualAndBoundedEveryFrame()
    {
        Vector3 original = _canvas.position;
        _camera.transform.SetPositionAndRotation(_camera.transform.position + new Vector3(-3f, 0f, -1f), Quaternion.Euler(0f, 80f, 0f));
        for (int frame = 0; frame < 150; frame++)
        {
            Pose before = PoseOf(_canvas);
            Follow(0.02f);
            Assert.That(Vector3.Distance(before.position, _canvas.position), Is.LessThanOrEqualTo(0.01201f), "Position is capped at 0.6 m/s.");
            Assert.That(Quaternion.Angle(before.rotation, _canvas.rotation), Is.LessThanOrEqualTo(0.705f), "Facing is capped at 35 degrees/s.");
        }
        Assert.That(Vector3.Distance(original, _canvas.position), Is.GreaterThan(0.1f));
        Assert.That(Vector3.Distance(original, _canvas.position), Is.LessThan(1.5f));
        Assert.That(Vector3.Dot(_canvas.forward, (_canvas.position - _camera.transform.position).normalized), Is.GreaterThan(0.99f));
    }

    [TestCase("hover")]
    [TestCase("pressed")]
    [TestCase("alignment")]
    [TestCase("target drag")]
    public void InteractionFreezesPositionAndFacingThenResumesGently(string interaction)
    {
        Action release;
        if (interaction == "hover")
        {
            PointerEvent("PointerEnter");
            Assert.That((bool)_menu.GetType().GetProperty("IsHovered").GetValue(null), Is.True);
            release = () => PointerEvent("PointerExit");
        }
        else if (interaction == "pressed")
        {
            GameObject button = _panel.GetComponentsInChildren(TypeOf("UnityEngine.UI.Button"), true)
                .Single(component => component.name == "Plan preview").gameObject;
            PointerEvent("PointerEnter");
            PointerEvent("PointerDown", button);
            PointerEvent("PointerExit"); // A pressed pointer may leave the panel before release.
            Assert.That((bool)_menu.GetType().GetProperty("IsHovered").GetValue(null), Is.True);
            release = () => PointerEvent("PointerUp", button);
        }
        else if (interaction == "alignment")
        {
            Component coordinates = New("Follow-test alignment service").AddComponent(TypeOf("RobotCoordinateService"));
            coordinates.GetType().GetMethod("BeginCalibrationEdit").Invoke(coordinates, null);
            release = () => coordinates.GetType().GetMethod("EndCalibrationEdit").Invoke(coordinates, null);
        }
        else
        {
            Component handles = New("Follow-test selected target").AddComponent(TypeOf("TargetRotationHandles"));
            object free = Enum.Parse(TypeOf("TargetRotationHandles+RotationHandleKind"), "Free");
            Assert.That(handles.GetType().GetMethod("BeginRotation").Invoke(handles, new[] { free, (object)Quaternion.identity, Vector3.zero }), Is.EqualTo(true));
            release = () => handles.GetType().GetMethod("EndRotation").Invoke(handles, null);
        }
        Pose original = PoseOf(_canvas);
        _camera.transform.SetPositionAndRotation(_camera.transform.position + new Vector3(-3f, 0.3f, -1f), Quaternion.Euler(0f, 90f, 0f));
        Advance(3f, 0.02f);
        AssertPose(PoseOf(_canvas), original);
        release();
        Advance(0.7f, 0.02f);
        AssertPose(PoseOf(_canvas), original);
        Advance(1f, 0.02f);
        Assert.That(Vector3.Distance(_canvas.position, original.position), Is.GreaterThan(0.001f));
        Assert.That(Vector3.Distance(_canvas.position, original.position), Is.LessThan(0.3f));
    }

    [Test]
    public void AHiddenMenuStaysStillAndReenablingDoesNotSnapIt()
    {
        Pose original = PoseOf(_canvas);
        PointerEvent("PointerDown");
        Private("OnDisable");
        Assert.That((bool)_menu.GetType().GetProperty("IsHovered").GetValue(null), Is.False, "Disabling clears unfinished pointer presses.");
        _camera.transform.position += Vector3.back * 3f;
        Advance(2f, 0.02f);
        AssertPose(PoseOf(_canvas), original);
        Private("OnEnable");
        Follow(0.02f);
        Assert.That(Vector3.Distance(_canvas.position, original.position), Is.LessThan(0.01201f));
        Advance(2f, 0.02f);
        Assert.That(Vector3.Distance(_canvas.position, original.position), Is.GreaterThan(0.01f));
        Assert.That(Vector3.Distance(_canvas.position, original.position), Is.LessThan(1f));
    }

    [Test]
    public void LookingStraightUpOrDownKeepsAValidHorizontalPlacement()
    {
        foreach (float pitch in new[] { -90f, 90f })
        {
            _camera.transform.rotation = Quaternion.Euler(pitch, 73f, 0f);
            Call("RecenterMenu");
            Vector3 offset = _canvas.position - _camera.transform.position;
            Assert.That(RobotFrameMath.IsValidPose(PoseOf(_canvas)), Is.True);
            Assert.That(Vector3.ProjectOnPlane(offset, Vector3.up).magnitude, Is.EqualTo(1f).Within(0.00001f));
            Assert.That(offset.y, Is.EqualTo(-0.15f).Within(0.00001f));
            Advance(1f, 0.02f);
            Assert.That(RobotFrameMath.IsValidPose(PoseOf(_canvas)), Is.True);
        }
    }

    [Test]
    public void ExplicitRecenterMovesImmediatelyWhileAutomaticFollowingDoesNotSnap()
    {
        Vector3 original = _canvas.position;
        _camera.transform.SetPositionAndRotation(new Vector3(10f, 1.8f, -5f), Quaternion.Euler(0f, 140f, 0f));
        Follow(0.02f);
        Assert.That(Vector3.Distance(_canvas.position, original), Is.LessThan(0.01201f));
        Call("RecenterMenu");
        Assert.That(Vector3.Distance(_canvas.position, original), Is.GreaterThan(5f));
        Assert.That(Vector3.ProjectOnPlane(_canvas.position - _camera.transform.position, Vector3.up).magnitude, Is.EqualTo(1f).Within(0.00001f));
    }

    [Test]
    public void RobotPlacementDoesNotMoveAnAlreadyPositionedViewerMenu()
    {
        Pose original = PoseOf(_canvas);
        Call("PositionAt", new Pose(new Vector3(100f, -10f, 40f), Quaternion.Euler(0f, 120f, 0f)));
        AssertPose(PoseOf(_canvas), original);
    }

    [Test]
    public void FollowingAtThirtyAndOneHundredTwentyFramesPerSecondEndsInTheSameComfortZone()
    {
        Vector3 cameraStart = _camera.transform.position;
        _camera.transform.position = cameraStart + Vector3.back * 3f;
        Advance(8f, 1f / 30f);
        Vector3 atThirty = _canvas.position;
        _camera.transform.position = cameraStart;
        Call("RecenterMenu");
        _camera.transform.position = cameraStart + Vector3.back * 3f;
        Advance(8f, 1f / 120f);
        Vector3 atOneTwenty = _canvas.position;
        float difference = Vector3.Distance(atThirty, atOneTwenty);
        Assert.That(difference, Is.LessThan(0.025f));
        Assert.That(Vector3.ProjectOnPlane(atThirty - _camera.transform.position, Vector3.up).magnitude, Is.InRange(0.85f, 1.21f));
        Assert.That(Vector3.ProjectOnPlane(atOneTwenty - _camera.transform.position, Vector3.up).magnitude, Is.InRange(0.85f, 1.21f));
        Debug.Log("MenuFollowValidation: frame-rate-position-difference-metres=" + difference.ToString("R", System.Globalization.CultureInfo.InvariantCulture));
    }

    [Test]
    public void TurningToLookAtTheMenuStopsThePositionChase()
    {
        _camera.transform.rotation = Quaternion.Euler(0f, 110f, 0f);
        Advance(0.8f, 0.02f);
        Assert.That(Vector3.Distance(_canvas.position, _camera.transform.position), Is.InRange(0.85f, 1.2f));
        Vector3 horizontal = Vector3.ProjectOnPlane(_canvas.position - _camera.transform.position, Vector3.up);
        _camera.transform.rotation = Quaternion.LookRotation(horizontal.normalized, Vector3.up);
        Vector3 original = _canvas.position;
        Advance(4f, 0.02f);
        Assert.That(Vector3.Distance(_canvas.position, original), Is.LessThan(0.000001f));
    }

    [Test]
    public void TurningAroundMovesTheMenuAroundTheViewerAndKeepsClearOfTheirFace()
    {
        _camera.transform.rotation = Quaternion.Euler(0f, 180f, 0f);
        float closest = float.PositiveInfinity;
        Vector3 original = _canvas.position;
        for (int frame = 0; frame < 300; frame++)
        {
            Follow(0.02f);
            float clearance = Vector3.ProjectOnPlane(_canvas.position - _camera.transform.position, Vector3.up).magnitude;
            closest = Mathf.Min(closest, clearance);
            Assert.That(clearance, Is.GreaterThanOrEqualTo(0.85f), "A 180-degree turn must arc around the viewer rather than travel through their face.");
        }
        Assert.That(Vector3.Distance(_canvas.position, original), Is.GreaterThan(0.5f));
        Debug.Log("MenuFollowValidation: turn-clearance-minimum-metres=" + closest.ToString("R", System.Globalization.CultureInfo.InvariantCulture));
    }

    [TestCase("focus")]
    [TestCase("pause")]
    public void ApplicationSuspensionClearsAnUnreleasedButtonAndResumesGently(string signal)
    {
        GameObject button = _panel.GetComponentsInChildren(TypeOf("UnityEngine.UI.Button"), true)
            .Single(component => component.name == "Plan preview").gameObject;
        PointerEvent("PointerEnter");
        PointerEvent("PointerDown", button);
        PointerEvent("PointerExit");
        Assert.That((bool)_menu.GetType().GetProperty("IsHovered").GetValue(null), Is.True);
        if (signal == "focus")
        {
            Private("OnApplicationFocus", false);
            Private("OnApplicationFocus", true);
        }
        else
        {
            Private("OnApplicationPause", true);
            Private("OnApplicationPause", false);
        }
        Assert.That((bool)_menu.GetType().GetProperty("IsHovered").GetValue(null), Is.False);
        Assert.That(GetProperty<int>(Field(_menu, "_pressedPointers").GetValue(_menu), "Count"), Is.Zero);
        Pose original = PoseOf(_canvas);
        _camera.transform.position += new Vector3(-3f, 0f, -1f);
        Advance(0.7f, 0.02f);
        AssertPose(PoseOf(_canvas), original);
        Advance(1f, 0.02f);
        Assert.That(Vector3.Distance(_canvas.position, original.position), Is.GreaterThan(0.001f));
        Assert.That(Vector3.Distance(_canvas.position, original.position), Is.LessThan(0.3f));
    }

    [Test]
    public void InvalidDurationsAreIgnoredAndLongFrameHitchesCannotSnapTheMenu()
    {
        Pose original = PoseOf(_canvas);
        _camera.transform.position += new Vector3(-3f, 0f, -1f);
        foreach (float delta in new[] { 0f, -1f, float.NaN, float.PositiveInfinity })
        {
            Follow(delta);
            AssertPose(PoseOf(_canvas), original);
        }
        Follow(10f);
        Assert.That(Vector3.Distance(_canvas.position, original.position), Is.LessThanOrEqualTo(0.06001f));
        Assert.That(Quaternion.Angle(_canvas.rotation, original.rotation), Is.LessThanOrEqualTo(3.505f));
    }

    [Test]
    public void MissingCameraLeavesTheWorldMenuUntouched()
    {
        Pose original = PoseOf(_canvas);
        _camera.gameObject.tag = "Untagged";
        _camera.transform.position += new Vector3(-3f, 0f, -1f);
        Advance(3f, 0.02f);
        AssertPose(PoseOf(_canvas), original);
    }

    [Test]
    public void DisablingFollowingKeepsTheWorldMenuStationary()
    {
        Field(_menu, "_followUser").SetValue(_menu, false);
        Pose original = PoseOf(_canvas);
        _camera.transform.SetPositionAndRotation(_camera.transform.position + Vector3.back * 3f, Quaternion.Euler(0f, 120f, 0f));
        Advance(3f, 0.02f);
        AssertPose(PoseOf(_canvas), original);
    }

    private void Follow(float deltaTime) => Private("UpdateFollow", deltaTime);
    private void Advance(float duration, float deltaTime)
    {
        int count = Mathf.RoundToInt(duration / deltaTime);
        for (int frame = 0; frame < count; frame++) Follow(deltaTime);
    }
    private void PointerEvent(string eventName, GameObject target = null)
    {
        Type triggerType = TypeOf("UnityEngine.EventSystems.EventTrigger");
        object trigger = (target != null ? target : _panel).GetComponent(triggerType);
        IList entries = (IList)triggerType.GetProperty("triggers").GetValue(trigger);
        object entry = entries.Cast<object>().Single(value => value.GetType().GetField("eventID").GetValue(value).ToString() == eventName);
        object callback = entry.GetType().GetField("callback").GetValue(entry);
        Type pointerType = TypeOf("UnityEngine.EventSystems.PointerEventData");
        object pointer = Activator.CreateInstance(pointerType, new object[] { null });
        pointerType.GetProperty("pointerId").SetValue(pointer, 41);
        callback.GetType().GetMethod("Invoke").Invoke(callback, new[] { pointer });
    }
    private GameObject New(string name) { var value = new GameObject(name); _owned.Add(value); return value; }
    private T Get<T>(string name) => (T)Field(_menu, name).GetValue(_menu);
    private static FieldInfo Field(object value, string name) => value.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public)
        ?? throw new InvalidOperationException("Missing field " + value.GetType().Name + "." + name);
    private object Call(string name, params object[] args) => _menu.GetType().GetMethod(name).Invoke(_menu, args);
    private object Private(string name, params object[] args) => _menu.GetType().GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic).Invoke(_menu, args);
    private static T GetProperty<T>(object value, string name) => (T)value.GetType().GetProperty(name).GetValue(value);
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType(name)).FirstOrDefault(t => t != null)
        ?? throw new InvalidOperationException("Missing runtime type " + name);
    private static Pose PoseOf(Transform value) => new Pose(value.position, value.rotation);
    private static void AssertPose(Pose actual, Pose expected)
    {
        Assert.That(Vector3.Distance(actual.position, expected.position), Is.LessThan(0.000001f));
        Assert.That(Quaternion.Angle(actual.rotation, expected.rotation), Is.LessThan(0.001f));
    }
}
