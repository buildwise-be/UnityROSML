using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>The actual world menu and native XRI selection, without ROS or XR hardware.</summary>
public sealed class MenuFollowTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private readonly List<GameObject> _previousMainCameras = new List<GameObject>();
    private Component _menu;
    private Camera _camera;
    private Transform _canvas;
    private GameObject _panel;
    private Component _interactionManager;
    private readonly List<Component> _rays = new List<Component>();

    [SetUp]
    public void CreateTheActualWorldMenu()
    {
        foreach (Camera existing in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (existing.CompareTag("MainCamera"))
            {
                _previousMainCameras.Add(existing.gameObject);
                existing.gameObject.tag = "Untagged";
            }
        GameObject view = New("Menu-test user view");
        _camera = view.AddComponent<Camera>();
        view.tag = "MainCamera";
        view.transform.SetPositionAndRotation(new Vector3(0f, 1.6f, 0f), Quaternion.identity);
        GameObject owner = New("Menu-test owner");
        owner.SetActive(false);
        _menu = owner.AddComponent(TypeOf("ToolSelectionMenu"));
        Private("BuildMenu");
        _canvas = Get<Canvas>("_canvas").transform;
        _panel = Get<RectTransform>("_panel").gameObject;
        owner.SetActive(true);
        // EditMode has no player lifecycle. Run the real callbacks explicitly,
        // including XRI registration, rather than silently bypassing native selection.
        Private("OnEnable");
        _interactionManager = New("Menu-test XRI manager").AddComponent(TypeOf("UnityEngine.XR.Interaction.Toolkit.XRInteractionManager"));
        Lifecycle(_interactionManager, "Awake");
        Lifecycle(_interactionManager, "OnEnable");
        Component grab = Get<Component>("_grabHandle");
        grab.GetType().GetProperty("interactionManager").SetValue(grab, _interactionManager);
        Lifecycle(grab, "Awake");
        Lifecycle(grab, "OnEnable");
        Call("RecenterMenu");
        Physics.SyncTransforms();
        Assert.That(grab.gameObject.activeInHierarchy, Is.True, "Runtime OnEnable must show the grab bar.");
        Assert.That(grab.GetType().GetProperty("interactionManager").GetValue(grab), Is.EqualTo(_interactionManager));
    }

    [TearDown]
    public void Cleanup()
    {
        if (_menu != null) Private("OnDisable");
        foreach (Component ray in _rays)
            if (ray != null) Lifecycle(ray, "OnDisable");
        _rays.Clear();
        Component grab = _menu != null ? Get<Component>("_grabHandle") : null;
        if (grab != null) Lifecycle(grab, "OnDisable");
        if (_interactionManager != null) Lifecycle(_interactionManager, "OnDisable");
        if (_canvas != null) UnityEngine.Object.DestroyImmediate(_canvas.gameObject);
        for (int index = _owned.Count - 1; index >= 0; index--)
            if (_owned[index] != null) UnityEngine.Object.DestroyImmediate(_owned[index]);
        _owned.Clear();
        foreach (GameObject previous in _previousMainCameras)
            if (previous != null) previous.tag = "MainCamera";
        _previousMainCameras.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
        Assert.That(Hovered, Is.False);
    }

    [Test]
    public void RecenterStartsOneMetreAwayAndTwentyDegreesRight()
    {
        _camera.transform.rotation = Quaternion.Euler(25f, 40f, 22f);
        Call("RecenterMenu");
        Vector3 offset = _canvas.position - _camera.transform.position;
        Vector3 horizontal = Vector3.ProjectOnPlane(_camera.transform.forward, Vector3.up).normalized;
        Assert.That(Vector3.ProjectOnPlane(offset, Vector3.up).magnitude, Is.EqualTo(1f).Within(0.00001f));
        Assert.That(offset.y, Is.EqualTo(-0.15f).Within(0.00001f));
        Assert.That(Vector3.SignedAngle(horizontal, Vector3.ProjectOnPlane(offset, Vector3.up), Vector3.up), Is.EqualTo(20f).Within(0.001f));
        AssertFacing();
        Assert.That(_canvas.parent, Is.Null);
    }

    [Test]
    public void WalkingFarAwayOrTurningAroundNeverMovesTheMenuButItAlwaysFacesTheViewer()
    {
        Vector3 placed = _canvas.position;
        foreach (Vector3 position in new[] { new Vector3(-3f, 1.9f, -2f), new Vector3(4f, 2.1f, 5f), new Vector3(0.1f, 0.5f, 0.1f) })
        {
            _camera.transform.SetPositionAndRotation(position, Quaternion.Euler(45f, 180f, 30f));
            for (int frame = 0; frame < 200; frame++) Private("LateUpdate");
            Assert.That(_canvas.position, Is.EqualTo(placed));
            AssertFacing();
        }
    }

    [Test]
    public void OnlyTheSeparateVisibleBarHasAGrabCollider()
    {
        Component grab = Get<Component>("_grabHandle");
        Collider[] colliders = _canvas.GetComponentsInChildren<Collider>(true);
        Assert.That(colliders.Length, Is.EqualTo(1));
        BoxCollider hit = grab.GetComponent<BoxCollider>();
        Assert.That(hit, Is.EqualTo(colliders[0]));
        Assert.That(hit.size.x * Mathf.Abs(hit.transform.lossyScale.x), Is.EqualTo(0.408f).Within(0.00001f));
        Assert.That(hit.bounds.size.y, Is.GreaterThan(0.025f));
        Assert.That(hit.bounds.size.z, Is.GreaterThan(0.01f));
        Component image = grab.GetComponent(TypeOf("UnityEngine.UI.Image"));
        Assert.That((bool)image.GetType().GetProperty("raycastTarget").GetValue(image), Is.False);
        RectTransform bar = (RectTransform)grab.transform;
        Assert.That(bar.anchoredPosition.y - bar.sizeDelta.y, Is.GreaterThan(0f), "The bar stays entirely above the panel.");
        foreach (Component button in _panel.GetComponentsInChildren(TypeOf("UnityEngine.UI.Button"), true))
            Assert.That(button.GetComponent<Collider>(), Is.Null, button.name);
    }

    [Test]
    public void NativeRaySelectionMovesWithoutASnapAndReleaseRetainsWorldPosition()
    {
        Component ray = SelectGrabWithRay();
        Vector3 before = _canvas.position;
        Private("LateUpdate");
        Assert.That(Vector3.Distance(_canvas.position, before), Is.LessThan(0.00001f));
        ray.transform.position += new Vector3(0.3f, 0.12f, -0.2f);
        Private("LateUpdate");
        Assert.That(Vector3.Distance(_canvas.position, before), Is.GreaterThan(0.2f));
        AssertFacing();
        NativeEvent("SelectExit", ray);
        Assert.That(Get<bool>("_draggingMenu"), Is.False);
        Vector3 placed = _canvas.position;
        ray.transform.position += Vector3.right * 3f;
        _camera.transform.position += new Vector3(3f, 0f, -3f);
        for (int frame = 0; frame < 100; frame++) Private("LateUpdate");
        Assert.That(_canvas.position, Is.EqualTo(placed));
        AssertFacing();
    }

    [Test]
    public void SelectedMenuStillSuppressesTargetCaptureAfterHoverExit()
    {
        Component ray = SelectGrabWithRay();
        NativeEvent("HoverEnter", ray);
        NativeEvent("HoverExit", ray);
        Assert.That(Hovered, Is.True, "URInputManager suppresses bumper capture and robot placement using this flag.");
        NativeEvent("SelectExit", ray);
        Assert.That(Hovered, Is.False);
    }

    [TestCase("focus")]
    [TestCase("pause")]
    [TestCase("disable")]
    [TestCase("tracking loss")]
    public void InterruptedGrabsReleaseNativeSelectionAndDoNotResume(string interruption)
    {
        Component ray = SelectGrabWithRay();
        if (interruption == "focus") Private("OnApplicationFocus", false);
        else if (interruption == "pause") Private("OnApplicationPause", true);
        else if (interruption == "disable")
        {
            ((Behaviour)_menu).enabled = false;
            Private("OnDisable");
        }
        else
        {
            GameObject inputOwner = New("Untracked controller input");
            inputOwner.SetActive(false);
            Component input = inputOwner.AddComponent(TypeOf("URInputManager"));
            input.GetType().GetMethod("ResetPendingInteraction", BindingFlags.NonPublic | BindingFlags.Instance).Invoke(input, null);
        }
        Assert.That(Get<bool>("_draggingMenu"), Is.False);
        Assert.That(Hovered, Is.False);
        Component grab = Get<Component>("_grabHandle");
        Assert.That((bool)grab.GetType().GetProperty("isSelected").GetValue(grab), Is.False);
        Vector3 placed = _canvas.position;
        ray.transform.position += Vector3.right;
        ((Behaviour)_menu).enabled = true;
        Private("OnEnable");
        Private("LateUpdate");
        Assert.That(_canvas.position, Is.EqualTo(placed));
    }

    [Test]
    public void ExplicitRecenterWorksAndQrPlacementDoesNotRelocateTheMenu()
    {
        Vector3 placed = _canvas.position;
        Call("PositionAt", new Pose(new Vector3(100f, -10f, 40f), Quaternion.Euler(0f, 120f, 0f)));
        Assert.That(_canvas.position, Is.EqualTo(placed));
        _camera.transform.position += Vector3.right * 5f;
        Private("LateUpdate");
        Assert.That(_canvas.position, Is.EqualTo(placed));
        Call("RecenterMenu");
        Assert.That(Vector3.Distance(_canvas.position, placed), Is.GreaterThan(4f));
        AssertFacing();
    }

    [Test]
    public void VerticalViewingAndCoincidentViewerPreserveAValidPose()
    {
        foreach (float pitch in new[] { -90f, 90f })
        {
            _camera.transform.rotation = Quaternion.Euler(pitch, 73f, 0f);
            Call("RecenterMenu");
            Private("LateUpdate");
            Assert.That(RobotFrameMath.IsValidPose(new Pose(_canvas.position, _canvas.rotation)), Is.True);
        }
        _camera.transform.position = _canvas.position;
        Private("LateUpdate");
        Assert.That(RobotFrameMath.IsValidPose(new Pose(_canvas.position, _canvas.rotation)), Is.True);
    }

    private Component SelectGrabWithRay()
    {
        Component grab = Get<Component>("_grabHandle");
        Vector3 point = grab.GetComponent<BoxCollider>().bounds.center;
        GameObject rayOwner = New("Menu-test laser");
        rayOwner.SetActive(false);
        rayOwner.transform.SetPositionAndRotation(point - Vector3.forward, Quaternion.identity);
        Component ray = rayOwner.AddComponent(TypeOf("UnityEngine.XR.Interaction.Toolkit.Interactors.XRRayInteractor"));
        ray.GetType().GetProperty("interactionManager").SetValue(ray, _interactionManager);
        ray.GetType().GetProperty("enableUIInteraction").SetValue(ray, false);
        Lifecycle(ray, "Awake");
        rayOwner.SetActive(true);
        Lifecycle(ray, "OnEnable");
        _rays.Add(ray);
        Physics.SyncTransforms();
        NativeEvent("SelectEnter", ray);
        Assert.That(Get<bool>("_draggingMenu"), Is.True, "Actual XRI selectEntered must begin movement.");
        Assert.That(Hovered, Is.True);
        return ray;
    }

    private void NativeEvent(string method, Component ray)
    {
        Component grab = Get<Component>("_grabHandle");
        object manager = grab.GetType().GetProperty("interactionManager").GetValue(grab);
        bool hover = method.StartsWith("Hover", StringComparison.Ordinal);
        MethodInfo native = manager.GetType().GetMethod(method, new[] {
            TypeOf("UnityEngine.XR.Interaction.Toolkit.Interactors." + (hover ? "IXRHoverInteractor" : "IXRSelectInteractor")),
            TypeOf("UnityEngine.XR.Interaction.Toolkit.Interactables." + (hover ? "IXRHoverInteractable" : "IXRSelectInteractable")) });
        native.Invoke(manager, new object[] { ray, grab });
    }

    private bool Hovered => (bool)_menu.GetType().GetProperty("IsHovered").GetValue(null);
    private static void Lifecycle(Component component, string method) => component.GetType()
        .GetMethod(method, BindingFlags.NonPublic | BindingFlags.Instance).Invoke(component, null);
    private void AssertFacing()
    {
        Assert.That(Vector3.Dot(_canvas.forward, (_canvas.position - _camera.transform.position).normalized), Is.GreaterThan(0.99999f));
        Assert.That(Mathf.Abs(_canvas.right.y), Is.LessThan(0.00001f));
    }
    private GameObject New(string name) { var value = new GameObject(name); _owned.Add(value); return value; }
    private T Get<T>(string name) => (T)_menu.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic).GetValue(_menu);
    private object Call(string name, params object[] args) => _menu.GetType().GetMethod(name).Invoke(_menu, args);
    private object Private(string name, params object[] args) => _menu.GetType().GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic).Invoke(_menu, args);
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType(name)).FirstOrDefault(t => t != null)
        ?? throw new InvalidOperationException("Missing runtime type " + name);
}
