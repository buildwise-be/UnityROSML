using System;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>The dedicated scenes expose only their workflow, without starting XR or ROS.</summary>
public sealed class WorkflowMenuTests
{
    private GameObject _owner;
    private Component _input, _menu;
    private const BindingFlags Fields = BindingFlags.Instance | BindingFlags.NonPublic;

    [TearDown]
    public void Cleanup()
    {
        if (_menu != null)
        {
            Canvas canvas = Read<Canvas>(_menu, "_canvas");
            if (canvas != null) UnityEngine.Object.DestroyImmediate(canvas.gameObject);
        }
        if (_owner != null) UnityEngine.Object.DestroyImmediate(_owner);
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
    }

    private void Create(string workflow)
    {
        _owner = new GameObject("Offline dedicated workflow menu");
        _owner.SetActive(false);
        _input = _owner.AddComponent(TypeOf("URInputManager"));
        _menu = _owner.AddComponent(TypeOf("ToolSelectionMenu"));
        Set(_menu, "_input", _input);
        Type mode = _input.GetType().GetNestedType("WorkflowMode");
        _menu.GetType().GetMethod("ConfigureWorkflow").Invoke(_menu, new[] { Enum.Parse(mode, workflow) });
        Invoke(_menu, "BuildMenu");
        Invoke(_menu, "BindTaskControls");
        Invoke(_menu, "RefreshTask");
    }

    [Test]
    public void SimpleWorkflowRetainsBothSimpleModesAndNeverCreatesFixtureBindings()
    {
        Create("SimplePickAndPlace");
        Assert.That(Read<Component>(_menu, "_tcpTaskButton"), Is.Not.Null);
        Assert.That(Read<Component>(_menu, "_pickPlaceTaskButton"), Is.Not.Null);
        Assert.That(Read<Component>(_menu, "_fixtureTaskButton"), Is.Null);
        Assert.That(Read<Component>(_menu, "_fixtureSetupButton"), Is.Null);
        Assert.That(Read<RectTransform>(_menu, "_fixtureView"), Is.Null);
        Assert.That(_input.GetType().GetProperty("FixtureController").GetValue(_input), Is.Null);
        _input.GetType().GetMethod("UseFixtureTiling").Invoke(_input, null);
        Assert.That(Property<bool>(_input, "IsFixtureMode"), Is.False);
        _input.GetType().GetMethod("UseTcpPreview").Invoke(_input, null);
        Assert.That(Property<bool>(_input, "IsTcpMode"), Is.True);
        _input.GetType().GetMethod("UseTilePickPlace").Invoke(_input, null);
        Assert.That(Property<bool>(_input, "IsTcpMode"), Is.False);
        Invoke(_menu, "BindTaskControls");
        Assert.That(_owner.GetComponents(TypeOf("FixtureTilingController")), Is.Empty);
    }

    [Test]
    public void RepetitiveWorkflowHasSetupAndReviewWithoutOldModeControls()
    {
        Create("RepetitivePickAndPlace");
        Assert.That(Read<Component>(_menu, "_tcpTaskButton"), Is.Null);
        Assert.That(Read<Component>(_menu, "_pickPlaceTaskButton"), Is.Null);
        Assert.That(Read<Component>(_menu, "_fixtureTaskButton"), Is.Null);
        Assert.That(Read<Component>(_menu, "_fixtureSetupButton"), Is.Not.Null);
        Assert.That(Read<Component>(_menu, "_fixtureReviewButton"), Is.Not.Null);
        Assert.That(Read<RectTransform>(_menu, "_fixtureView"), Is.Not.Null);
        _input.GetType().GetMethod("UseTcpPreview").Invoke(_input, null);
        _input.GetType().GetMethod("UseTilePickPlace").Invoke(_input, null);
        Assert.That(Property<bool>(_input, "IsFixtureMode"), Is.True);
        Assert.That(Property<bool>(_input, "IsTcpMode"), Is.False);
        Assert.That(_owner.GetComponents(TypeOf("FixtureTilingController")).Length, Is.EqualTo(1));
        _menu.GetType().GetMethod("OpenFixtureReview").Invoke(_menu, null);
        Assert.That(Read<int>(_menu, "_fixturePage"), Is.EqualTo(7));
        Assert.That(Read<RectTransform>(_menu, "_fixtureView").gameObject.activeSelf, Is.True);
    }

    [TestCase("SimplePickAndPlace")]
    [TestCase("RepetitivePickAndPlace")]
    public void DedicatedWorkflowNavigationFitsAndStaysLockedWhileACommandIsUnresolved(string workflow)
    {
        Create(workflow);
        Component welcome = Read<Component>(_menu, "_welcomeButton");
        RectTransform rect = welcome.GetComponent<RectTransform>();
        RectTransform status = Read<Component>(_menu, "_qrStatus").GetComponent<RectTransform>();
        RectTransform title = Read<RectTransform>(_menu, "_taskView").Find("Title").GetComponent<RectTransform>();
        Assert.That(title.anchoredPosition.x + title.sizeDelta.x, Is.LessThan(rect.anchoredPosition.x));
        Assert.That(-rect.anchoredPosition.y + rect.sizeDelta.y, Is.LessThan(-status.anchoredPosition.y));
        Assert.That(status.sizeDelta.y, Is.EqualTo(80f), "Navigation must retain the full measured calibration/status area.");
        Assert.That(-rect.anchoredPosition.y + rect.sizeDelta.y, Is.LessThanOrEqualTo(756));
        Type execution = TypeOf("RobotExecutionState");
        execution.GetMethod("SetBusy").Invoke(null, new object[] { _input, true });
        try
        {
            Invoke(_menu, "RefreshSceneNavigation");
            Assert.That(Property<bool>(welcome, "interactable"), Is.False);
            _menu.GetType().GetMethod("ReturnToWelcome").Invoke(_menu, null);
            Assert.That((bool)execution.GetProperty("IsBusy").GetValue(null), Is.True);
        }
        finally { execution.GetMethod("SetBusy").Invoke(null, new object[] { _input, false }); }
    }

    [Test]
    public void FixtureReviewShowsStopInTheExecuteSlotAndCancelsThePendingCheck()
    {
        Create("RepetitivePickAndPlace");
        Component controller = Property<Component>(_input, "FixtureController");
        Assert.That(controller.GetType().GetMethod("StopPlanning"), Is.Not.Null, "The fixture Review page needs a planning stop action.");
        Set(controller, "_checkingCalibration", true);
        _menu.GetType().GetMethod("OpenFixtureReview").Invoke(_menu, null);
        Invoke(_menu, "RefreshTask");
        RectTransform page = Read<RectTransform>(_menu, "_fixturePageContent");
        Component plan = page.Find("Plan fixture batch").GetComponent(TypeOf("UnityEngine.UI.Button"));
        Component stop = page.Find("Stop fixture planning").GetComponent(TypeOf("UnityEngine.UI.Button"));
        Component execute = page.Find("Execute fixture batch").GetComponent(TypeOf("UnityEngine.UI.Button"));
        Assert.That(stop.gameObject.activeSelf && Property<bool>(stop, "interactable"), Is.True);
        Assert.That(Property<bool>(plan, "interactable"), Is.False);
        Assert.That(execute.gameObject.activeSelf, Is.False);
        object clicked = Property<object>(stop, "onClick"); clicked.GetType().GetMethod("Invoke").Invoke(clicked, null);
        Assert.That(stop.gameObject.activeSelf, Is.False);
        Assert.That(execute.gameObject.activeSelf, Is.True);
        Assert.That(Property<bool>(execute, "interactable"), Is.False);
        Assert.That(Property<bool>(plan, "interactable"), Is.True);
    }

    private static void Set(object target, string name, object value) => target.GetType().GetField(name, Fields).SetValue(target, value);
    private static T Read<T>(object target, string name) => (T)target.GetType().GetField(name, Fields).GetValue(target);
    private static T Property<T>(object target, string name) => (T)target.GetType().GetProperty(name).GetValue(target);
    private static void Invoke(object target, string name) => target.GetType().GetMethod(name, Fields).Invoke(target, null);
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(assembly => assembly.GetType(name)).First(type => type != null);
}
