using System;
using System.IO;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
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
    public void SimpleWorkflowStartsWithContactPickupAndRetainsOptionalTcpPreview()
    {
        Create("SimplePickAndPlace");
        Assert.That(Read<Component>(_menu, "_tcpTaskButton"), Is.Not.Null);
        Assert.That(Read<Component>(_menu, "_pickPlaceTaskButton"), Is.Not.Null);
        Assert.That(Property<bool>(_input, "IsTcpMode"), Is.False);
        _input.GetType().GetMethod("UseTcpPreview").Invoke(_input, null);
        Assert.That(Property<bool>(_input, "IsTcpMode"), Is.True);
        _input.GetType().GetMethod("UseTilePickPlace").Invoke(_input, null);
        Assert.That(Property<bool>(_input, "IsTcpMode"), Is.False);
        Assert.That(_owner.GetComponents(TypeOf("FixtureTilingController")), Is.Empty);
    }

    [Test]
    public void RepetitiveWorkflowOpensDirectFloorSetupAndHasNoFixtureRoute()
    {
        Create("RepetitivePickAndPlace");
        Assert.That(Read<Component>(_menu, "_tcpTaskButton"), Is.Null);
        Assert.That(Read<Component>(_menu, "_pickPlaceTaskButton"), Is.Null);
        RectTransform task = Read<RectTransform>(_menu, "_taskView");
        Transform entry = task.Find("Floor tiling");
        Assert.That(entry, Is.Not.Null);
        Assert.That(Property<bool>(_input, "IsFloorMode"), Is.True);
        Assert.That(task.Find("Previous floor scene step")?.gameObject.activeSelf, Is.False);
        Assert.That(task.Find("Next floor scene step")?.gameObject.activeSelf, Is.False);
        Click(entry.GetComponent(TypeOf("UnityEngine.UI.Button")));
        RectTransform floor = Read<RectTransform>(_menu, "_floorView");
        Assert.That(floor.gameObject.activeSelf, Is.True);
        AssertDiscreteSlider(floor.Find("Row offset slider"), 0, 2);
        AssertDiscreteSlider(floor.Find("Mortar thickness slider"), 5, 15);
        Assert.That(Property<bool>(floor.Find("Execute floor plan").GetComponent(TypeOf("UnityEngine.UI.Button")), "interactable"), Is.False);
        Assert.That(floor.Find("Open fixture setup"), Is.Null);
        Assert.That(_owner.GetComponents(TypeOf("FixtureTilingController")), Is.Empty);
        Assert.That(_input.GetType().GetMethod("UseFixtureTiling"), Is.Null);
        Assert.That(_input.GetType().GetProperty("FixtureController"), Is.Null);
        _input.GetType().GetMethod("UseTcpPreview").Invoke(_input, null);
        _input.GetType().GetMethod("UseTilePickPlace").Invoke(_input, null);
        Assert.That(Property<bool>(_input, "IsFloorMode"), Is.True);
        Assert.That(Property<bool>(_input, "IsTcpMode"), Is.False);
    }

    [TestCase("SimplePickAndPlace", 0)]
    [TestCase("Unrestricted", 0)]
    [TestCase("RepetitivePickAndPlace", 3)]
    public void SerializedFormerFixtureModeResolvesToTheDirectWorkflow(string workflow, int expectedMode)
    {
        Create(workflow);
        Type capture = _input.GetType().GetNestedType("PoseCaptureMode");
        Set(_input, "_poseCaptureMode", Enum.ToObject(capture, 2));
        Type mode = _input.GetType().GetNestedType("WorkflowMode");
        _input.GetType().GetMethod("ConfigureWorkflow").Invoke(_input, new[] { Enum.Parse(mode, workflow) });
        Assert.That(Convert.ToInt32(Read<object>(_input, "_poseCaptureMode")), Is.EqualTo(expectedMode));
        Assert.That(_owner.GetComponents(TypeOf("FixtureTilingController")), Is.Empty);
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

    private static void AssertDiscreteSlider(Transform root, float min, float max)
    {
        Assert.That(root, Is.Not.Null);
        Component slider = root.GetComponent(TypeOf("UnityEngine.UI.Slider"));
        Assert.That(Property<bool>(slider, "wholeNumbers"), Is.True);
        Assert.That(Property<float>(slider, "minValue"), Is.EqualTo(min));
        Assert.That(Property<float>(slider, "maxValue"), Is.EqualTo(max));
    }

    private static void Click(Component button)
    {
        Assert.That(Property<bool>(button, "interactable"), Is.True);
        object clicked = Property<object>(button, "onClick");
        clicked.GetType().GetMethod("Invoke").Invoke(clicked, null);
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

    /// <summary>Explicit graphics export of the direct simple and floor workflow menus.</summary>
    public static void RenderDedicatedWorkflowPreviews()
    {
        if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null)
            throw new InvalidOperationException("Workflow menu previews require a graphics-enabled Unity process.");
        foreach (string workflow in new[] { "SimplePickAndPlace", "RepetitivePickAndPlace" })
        {
            var preview = new WorkflowMenuTests();
            GameObject viewer = null;
            RenderTexture target = null;
            Texture2D pixels = null;
            RenderTexture previous = RenderTexture.active;
            try
            {
                preview.Create(workflow);
                Canvas canvas = Read<Canvas>(preview._menu, "_canvas");
                canvas.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
                Read<RectTransform>(preview._menu, "_panel").gameObject.SetActive(true);
                if (workflow == "RepetitivePickAndPlace") Invoke(preview._menu, "OpenFloorSetup");
                viewer = new GameObject("Offline direct workflow viewer", typeof(Camera));
                Camera camera = viewer.GetComponent<Camera>();
                camera.transform.position = new Vector3(0f, .02f, -1f);
                camera.orthographic = true;
                camera.orthographicSize = .43f;
                camera.clearFlags = CameraClearFlags.SolidColor;
                camera.backgroundColor = new Color(.08f, .11f, .15f, 1f);
                camera.allowHDR = camera.allowMSAA = false;
                canvas.worldCamera = camera;
                Type textType = TypeOf("TMPro.TMP_Text"), fontType = TypeOf("TMPro.TMP_FontAsset");
                UnityEngine.Object font = Resources.Load("Fonts & Materials/LiberationSans SDF", fontType);
                if (font == null)
                {
                    string guid = AssetDatabase.FindAssets("LiberationSans SDF t:TMP_FontAsset").FirstOrDefault();
                    if (guid != null) font = AssetDatabase.LoadAssetAtPath(AssetDatabase.GUIDToAssetPath(guid), fontType);
                }
                if (font == null) throw new FileNotFoundException("A font is required for workflow menu previews.");
                foreach (Component text in canvas.GetComponentsInChildren(textType, true))
                    textType.GetProperty("font").SetValue(text, font);
                Canvas.ForceUpdateCanvases();
                foreach (Component text in canvas.GetComponentsInChildren(textType, true))
                    textType.GetMethod("ForceMeshUpdate").Invoke(text, new object[] { true, true });
                target = new RenderTexture(600, 960, 24, RenderTextureFormat.ARGB32);
                camera.targetTexture = target;
                camera.Render();
                RenderTexture.active = target;
                pixels = new Texture2D(600, 960, TextureFormat.RGB24, false);
                pixels.ReadPixels(new Rect(0f, 0f, 600f, 960f), 0, 0);
                pixels.Apply();
                string directory = Path.GetFullPath(Path.Combine("WorkflowMenuPreviews", workflow));
                Directory.CreateDirectory(directory);
                string output = Path.Combine(directory, "direct-workflow.png");
                File.WriteAllBytes(output, pixels.EncodeToPNG());
                Debug.Log("WorkflowMenuPreview: " + output);
            }
            finally
            {
                if (viewer != null) viewer.GetComponent<Camera>().targetTexture = null;
                RenderTexture.active = previous;
                if (pixels != null) UnityEngine.Object.DestroyImmediate(pixels);
                if (target != null) { target.Release(); UnityEngine.Object.DestroyImmediate(target); }
                if (viewer != null) UnityEngine.Object.DestroyImmediate(viewer);
                preview.Cleanup();
            }
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
