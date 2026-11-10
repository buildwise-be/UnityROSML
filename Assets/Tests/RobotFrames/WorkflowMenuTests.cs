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

    [TestCase("SimplePickAndPlace")]
    [TestCase("RepetitivePickAndPlace")]
    public void PlanningTimeIsSharedAcrossTaskMenusAndClampedToSupportedSeconds(string workflow)
    {
        const string preference = "UnityROSML.MtcPlanningTimeoutSeconds";
        bool hadPreference = PlayerPrefs.HasKey(preference);
        int saved = PlayerPrefs.GetInt(preference);
        try
        {
            PlayerPrefs.DeleteKey(preference);
            Create(workflow);
            RectTransform task = Read<RectTransform>(_menu, "_taskView");
            Transform control = task.Find("Planning time control");
            Assert.That(control, Is.Not.Null, "The task menu needs a shared planning time control.");
            Assert.That(PlanningTimeLabel(control), Is.EqualTo("60 s"));
            ClickPlanningTimeButton(control, "Increase planning time by 30 seconds");
            Assert.That(PlayerPrefs.GetInt(preference), Is.EqualTo(90));
            Assert.That(PlanningTimeLabel(control), Is.EqualTo("90 s"));

            if (workflow == "SimplePickAndPlace")
            {
                _input.GetType().GetMethod("UseTcpPreview").Invoke(_input, null);
                Invoke(_menu, "RefreshTask");
                Assert.That(PlanningTimeLabel(control), Is.EqualTo("90 s"));
            }
            else
            {
                _menu.GetType().GetMethod("OpenFixtureReview").Invoke(_menu, null);
                Transform reviewControl = Read<RectTransform>(_menu, "_fixturePageContent").Find("Planning time control");
                Assert.That(reviewControl, Is.Not.Null);
                Assert.That(PlanningTimeLabel(reviewControl), Is.EqualTo("90 s"));
                ClickPlanningTimeButton(reviewControl, "Decrease planning time by 1 second");
                Invoke(_menu, "RefreshTask");
                Assert.That(PlanningTimeLabel(control), Is.EqualTo("89 s"));
                _menu.GetType().GetMethod("CloseFixtureSetup").Invoke(_menu, null);
            }

            PropertyInfo timeout = TypeOf("MtcPlanningSettings").GetProperty("TimeoutSeconds");
            timeout.SetValue(null, 599);
            Invoke(_menu, "RefreshTask");
            ClickPlanningTimeButton(control, "Increase planning time by 30 seconds");
            Assert.That(PlanningTimeLabel(control), Is.EqualTo("600 s"));
            Assert.That(PlayerPrefs.GetInt(preference), Is.EqualTo(600));
            timeout.SetValue(null, 2);
            Invoke(_menu, "RefreshTask");
            ClickPlanningTimeButton(control, "Decrease planning time by 30 seconds");
            Assert.That(PlanningTimeLabel(control), Is.EqualTo("1 s"));
            Assert.That(PlayerPrefs.GetInt(preference), Is.EqualTo(1));
        }
        finally
        {
            if (hadPreference) PlayerPrefs.SetInt(preference, saved);
            else PlayerPrefs.DeleteKey(preference);
            PlayerPrefs.Save();
        }
    }

    private static string PlanningTimeLabel(Transform control) => Property<string>(
        control.Find("Planning time value").GetComponent(TypeOf("TMPro.TextMeshProUGUI")), "text");

    private static void ClickPlanningTimeButton(Transform control, string name)
    {
        Component button = control.Find(name).GetComponent(TypeOf("UnityEngine.UI.Button"));
        Assert.That(Property<bool>(button, "interactable"), Is.True);
        object click = Property<object>(button, "onClick");
        click.GetType().GetMethod("Invoke").Invoke(click, null);
    }

    private static void Set(object target, string name, object value) => target.GetType().GetField(name, Fields).SetValue(target, value);
    private static T Read<T>(object target, string name) => (T)target.GetType().GetField(name, Fields).GetValue(target);
    private static T Property<T>(object target, string name) => (T)target.GetType().GetProperty(name).GetValue(target);
    private static void Invoke(object target, string name) => target.GetType().GetMethod(name, Fields).Invoke(target, null);
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(assembly => assembly.GetType(name)).First(type => type != null);
}
