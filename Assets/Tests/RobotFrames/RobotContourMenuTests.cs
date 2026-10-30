using System;
using System.Collections;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>Actual XR canvas checkbox and presentation-only routing, without Start or ROS transport.</summary>
public sealed class RobotContourMenuTests
{
    private GameObject _owner;
    private Component _menu;
    private Component _toggle;

    [SetUp]
    public void CreateMenuBeforeRobotPlacement()
    {
        SetContours(false);
        _owner = new GameObject("Offline contour menu");
        _owner.SetActive(false);
        _menu = _owner.AddComponent(RuntimeType("ToolSelectionMenu"));
        Invoke(_menu, "BuildMenu");
        _toggle = Field<Component>(_menu, "_robotContoursToggle");
        // The generated canvas is a separate world-space object. It can exercise
        // native UI click behavior while the menu owner cannot run Start/ROS.
        Field<RectTransform>(_menu, "_panel").gameObject.SetActive(true);
    }

    [TearDown]
    public void Cleanup()
    {
        if (_menu != null) Invoke(_menu, "OnDisable");
        SetContours(false);
        if (_menu != null)
        {
            Canvas canvas = Field<Canvas>(_menu, "_canvas");
            if (canvas != null) UnityEngine.Object.DestroyImmediate(canvas.gameObject);
        }
        if (_owner != null) UnityEngine.Object.DestroyImmediate(_owner);
    }

    /// <summary>Use the existing graphics fixture to inspect the checked main-menu toggle.</summary>
    public static void RenderContourMenuPreviews()
    {
        bool previous = ContoursEnabled;
        SetContours(true);
        try { ToolCalibrationMenuTests.RenderCalibrationMenuPreviews(); }
        finally { SetContours(previous); }
    }

    [Test]
    public void TheActualCheckboxWorksBeforePlacementAndReflectsExternalSelection()
    {
        Assert.That(_toggle, Is.Not.Null);
        Assert.That(_toggle.GetType(), Is.EqualTo(RuntimeType("UnityEngine.UI.Toggle")));
        Assert.That(Get<bool>(_toggle, "interactable"), Is.True);
        Assert.That(Get<bool>(_toggle, "isOn"), Is.False);
        ClickToggle();
        Assert.That(ContoursEnabled, Is.True);
        Assert.That(Get<bool>(_toggle, "isOn"), Is.True);
        SetContours(false);
        Assert.That(Get<bool>(_toggle, "isOn"), Is.False, "External presentation changes must update the visible check state.");
        SetContours(true);
        ClickToggle();
        Assert.That(ContoursEnabled, Is.False);
        AssertNoRosTransport();
    }

    [Test]
    public void CheckboxFitsBesideTheSubtitleAndRetainsXrPressConsumption()
    {
        RectTransform rect = (RectTransform)_toggle.transform;
        Rect bounds = new Rect(rect.anchoredPosition.x, -rect.anchoredPosition.y, rect.sizeDelta.x, rect.sizeDelta.y);
        RectTransform task = Field<RectTransform>(_menu, "_taskView");
        Assert.That(bounds.xMin, Is.GreaterThanOrEqualTo(0f));
        Assert.That(bounds.xMax, Is.LessThanOrEqualTo(task.sizeDelta.x));
        Assert.That(bounds.yMin, Is.GreaterThanOrEqualTo(42f), "Header and checkbox must remain separate.");
        Assert.That(bounds.yMax, Is.LessThan(82f), "The checkbox must not overlap the tool buttons.");
        Component subtitle = task.Find("Subtitle").GetComponent(RuntimeType("TMPro.TextMeshProUGUI"));
        RectTransform subtitleRect = (RectTransform)subtitle.transform;
        Assert.That(subtitleRect.anchoredPosition.x + subtitleRect.sizeDelta.x, Is.LessThan(bounds.xMin));
        Component label = rect.Find("Robot contours label").GetComponent(RuntimeType("TMPro.TextMeshProUGUI"));
        Assert.That(Get<string>(label, "text"), Is.EqualTo("Robot contours"));
        Assert.That(Get<float>(label, "fontSize"), Is.GreaterThanOrEqualTo(13f));
        Component trigger = rect.GetComponent(RuntimeType("UnityEngine.EventSystems.EventTrigger"));
        Assert.That(trigger, Is.Not.Null);
        IList entries = Get<IList>(trigger, "triggers");
        string[] events = entries.Cast<object>().Select(entry => Get<object>(entry, "eventID").ToString()).ToArray();
        Assert.That(events, Does.Contain("PointerDown"));
        Assert.That(events, Does.Contain("PointerUp"));
        Assert.That(Field<RectTransform>(_menu, "_pointView").Find("Robot contours"), Is.Null,
            "The control belongs to the main task view, not the contact-picking workflow.");
    }

    [Test]
    public void ChangingContourStyleDoesNotInvalidateCalibrationTaskModeOrGhostPreview()
    {
        Component coordinates = _owner.AddComponent(RuntimeType("RobotCoordinateService"));
        Component input = _owner.AddComponent(RuntimeType("URInputManager"));
        Component preview = _owner.AddComponent(RuntimeType("UR16eTrajectoryVisualizer"));
        SetField(_menu, "_input", input);
        SetField(_menu, "_trajectoryVisualizer", preview);
        SetField(preview, "_percentageToShow", 37f);
        SetField(preview, "_playbackSpeed", 1.25f);
        FieldInfo trajectory = preview.GetType().GetField("_trajectory", BindingFlags.NonPublic | BindingFlags.Instance);
        object reviewedPath = Activator.CreateInstance(trajectory.FieldType);
        trajectory.SetValue(preview, reviewedPath);
        int revision = Get<int>(coordinates, "CalibrationRevision");
        bool mode = Get<bool>(input, "IsTcpMode");

        ClickToggle();

        Assert.That(ContoursEnabled, Is.True);
        Assert.That(Get<int>(coordinates, "CalibrationRevision"), Is.EqualTo(revision));
        Assert.That(Get<bool>(input, "IsTcpMode"), Is.EqualTo(mode));
        Assert.That(trajectory.GetValue(preview), Is.SameAs(reviewedPath));
        Assert.That(Field<float>(preview, "_percentageToShow"), Is.EqualTo(37f));
        Assert.That(Field<float>(preview, "_playbackSpeed"), Is.EqualTo(1.25f));
        Assert.That(Field<bool>(input, "_robotPlacementRequested"), Is.False);
        AssertNoRosTransport();
    }

    [Test]
    public void DisabledMenuUnsubscribesAndResynchronizesWhenEnabledAgain()
    {
        Invoke(_menu, "OnDisable");
        SetContours(true);
        Assert.That(Get<bool>(_toggle, "isOn"), Is.False, "A disabled menu should not retain a presentation-state subscription.");
        Invoke(_menu, "OnEnable");
        Assert.That(Get<bool>(_toggle, "isOn"), Is.True);
        SetContours(false);
        Assert.That(Get<bool>(_toggle, "isOn"), Is.False);
        AssertNoRosTransport();
    }

    private void ClickToggle()
    {
        Type pointerType = RuntimeType("UnityEngine.EventSystems.PointerEventData");
        object pointer = Activator.CreateInstance(pointerType, new object[] { null });
        _toggle.GetType().GetMethod("OnPointerClick").Invoke(_toggle, new[] { pointer });
    }
    private static bool ContoursEnabled => (bool)RuntimeType("RobotContourDisplay").GetProperty("Enabled").GetValue(null);
    private static void SetContours(bool enabled) => RuntimeType("RobotContourDisplay").GetMethod("SetEnabled").Invoke(null, new object[] { enabled });
    private static void AssertNoRosTransport()
    {
        Assert.That(Resources.FindObjectsOfTypeAll(RuntimeType("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
    }
    private static T Get<T>(object value, string name)
    {
        MemberInfo member = value.GetType().GetMember(name, BindingFlags.Public | BindingFlags.Instance).Single();
        return (T)(member is PropertyInfo property ? property.GetValue(value) : ((FieldInfo)member).GetValue(value));
    }
    private static T Field<T>(object value, string name) => (T)value.GetType().GetField(name, BindingFlags.NonPublic | BindingFlags.Instance).GetValue(value);
    private static void SetField(object value, string name, object data) => value.GetType().GetField(name, BindingFlags.NonPublic | BindingFlags.Instance).SetValue(value, data);
    private static void Invoke(object value, string name) => value.GetType().GetMethod(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance).Invoke(value, null);
    private static Type RuntimeType(string name) => AppDomain.CurrentDomain.GetAssemblies()
        .Select(assembly => assembly.GetType(name)).FirstOrDefault(type => type != null)
        ?? throw new InvalidOperationException("Missing runtime type " + name);
}
