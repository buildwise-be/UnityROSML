using System;
using System.Linq;
using System.IO;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>Offline checks for the paged XR form, without connecting ROS or sending commands.</summary>
public sealed class FixtureTilingMenuTests
{
    private GameObject _owner;
    private Component _menu;
    private const BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;

    [SetUp]
    public void BuildMenuWithoutRuntimeStart()
    {
        _owner = new GameObject("Offline fixture menu");
        _owner.SetActive(false);
        _menu = _owner.AddComponent(RuntimeType("ToolSelectionMenu"));
        Invoke("BuildMenu");
    }

    [TearDown]
    public void Cleanup()
    {
        Canvas canvas = Field<Canvas>("_canvas");
        if (canvas != null) UnityEngine.Object.DestroyImmediate(canvas.gameObject);
        if (_owner != null) UnityEngine.Object.DestroyImmediate(_owner);
    }

    [Test]
    public void ThirdTaskChoiceFitsWithExistingModesAndSetupStartsHidden()
    {
        RectTransform tcp = Field<Component>("_tcpTaskButton").GetComponent<RectTransform>();
        RectTransform legacy = Field<Component>("_pickPlaceTaskButton").GetComponent<RectTransform>();
        RectTransform fixture = Field<Component>("_fixtureTaskButton").GetComponent<RectTransform>();
        Assert.That(tcp.anchoredPosition.x + tcp.sizeDelta.x, Is.LessThan(legacy.anchoredPosition.x));
        Assert.That(legacy.anchoredPosition.x + legacy.sizeDelta.x, Is.LessThan(fixture.anchoredPosition.x));
        Assert.That(fixture.anchoredPosition.x + fixture.sizeDelta.x, Is.LessThanOrEqualTo(440));
        Assert.That(Field<RectTransform>("_fixtureView").gameObject.activeSelf, Is.False);
        Assert.That(Field<Component>("_fixtureSetupButton").gameObject.activeSelf, Is.False);
    }

    [TestCase(0)]
    [TestCase(1)]
    [TestCase(2)]
    [TestCase(3)]
    [TestCase(4)]
    [TestCase(5)]
    [TestCase(6)]
    [TestCase(7)]
    public void AllFixturePagesFitAbovePersistentStatusAndCaptureInstructions(int page)
    {
        _menu.GetType().GetField("_fixturePage", Private).SetValue(_menu, page);
        Invoke("BuildFixturePage");
        RectTransform content = Field<RectTransform>("_fixturePageContent");
        foreach (RectTransform control in content)
        {
            float x = control.anchoredPosition.x;
            float y = -control.anchoredPosition.y;
            Assert.That(x, Is.GreaterThanOrEqualTo(0), control.name);
            Assert.That(y, Is.GreaterThanOrEqualTo(0), control.name);
            Assert.That(x + control.sizeDelta.x, Is.LessThanOrEqualTo(content.sizeDelta.x + 0.1f), control.name);
            Assert.That(y + control.sizeDelta.y, Is.LessThanOrEqualTo(content.sizeDelta.y + 0.1f), control.name);
        }
        Assert.That(-content.anchoredPosition.y + content.sizeDelta.y,
            Is.LessThan(-Field<Component>("_fixtureStatus").GetComponent<RectTransform>().anchoredPosition.y));
        foreach (Component text in content.GetComponentsInChildren(RuntimeType("TMPro.TextMeshProUGUI"), true))
            Assert.That((float)text.GetType().GetProperty("fontSize").GetValue(text), Is.GreaterThanOrEqualTo(14));
    }

    [Test]
    public void PagingClampsAndDoesNotCreateRosConnection()
    {
        _menu.GetType().GetMethod("ChangeFixturePage").Invoke(_menu, new object[] { 100 });
        Assert.That(Field<int>("_fixturePage"), Is.EqualTo(7));
        _menu.GetType().GetMethod("ChangeFixturePage").Invoke(_menu, new object[] { -100 });
        Assert.That(Field<int>("_fixturePage"), Is.EqualTo(0));
        Assert.That(Resources.FindObjectsOfTypeAll(RuntimeType("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
    }

    [TestCase(false)]
    [TestCase(true)]
    public void FixtureModeCreatesOneControllerEvenWhenChangedListenerResolvesItAgain(bool executionPending)
    {
        Component input = _owner.AddComponent(RuntimeType("URInputManager"));
        PropertyInfo controllerProperty = input.GetType().GetProperty("FixtureController");
        object controller = controllerProperty.GetValue(input);
        int callbacks = 0;
        Action changed = () =>
        {
            callbacks++;
            Assert.That(controllerProperty.GetValue(input), Is.SameAs(controller));
        };
        EventInfo changedEvent = controller.GetType().GetEvent("Changed");
        changedEvent.AddEventHandler(controller, changed);
        try
        {
            input.GetType().GetMethod("UseFixtureTiling").Invoke(input, null);
            Assert.That((bool)input.GetType().GetProperty("IsFixtureMode").GetValue(input), Is.True);
            if (executionPending)
            {
                controller.GetType().GetField("_ready", Private).SetValue(controller, true);
                controller.GetType().GetField("_executionRequested", Private).SetValue(controller, true);
                controller.GetType().GetField("_requestId", Private).SetValue(controller, "retained-request");
                controller.GetType().GetField("_planId", Private).SetValue(controller, "retained-plan");
                RuntimeType("RobotExecutionState").GetMethod("SetBusy").Invoke(null, new[] { controller, (object)true });
            }
            input.GetType().GetMethod("UseTcpPreview").Invoke(input, null);
            Assert.That((bool)input.GetType().GetProperty("IsFixtureMode").GetValue(input), Is.False);
            Assert.That((bool)controller.GetType().GetField("_ready", Private).GetValue(controller), Is.False,
                "Switching modes revokes the previous local review.");
            if (executionPending)
            {
                Assert.That((bool)controller.GetType().GetProperty("IsExecuting").GetValue(controller), Is.True);
                Assert.That((bool)RuntimeType("RobotExecutionState").GetProperty("IsBusy").GetValue(null), Is.True);
                Assert.That(controller.GetType().GetField("_requestId", Private).GetValue(controller), Is.EqualTo("retained-request"));
                Assert.That(controller.GetType().GetField("_planId", Private).GetValue(controller), Is.EqualTo("retained-plan"));
                input.GetType().GetMethod("PlanCurrentTask").Invoke(input, null);
                input.GetType().GetMethod("ExecuteCurrentTask").Invoke(input, null);
            }
            Assert.That(callbacks, Is.GreaterThan(0));
            Assert.That(_owner.GetComponents(RuntimeType("FixtureTilingController")).Length, Is.EqualTo(1));
            Assert.That(Resources.FindObjectsOfTypeAll(RuntimeType("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
        }
        finally
        {
            changedEvent.RemoveEventHandler(controller, changed);
            RuntimeType("RobotExecutionState").GetMethod("SetBusy").Invoke(null, new[] { controller, (object)false });
            controller.GetType().GetField("_executionRequested", Private).SetValue(controller, false);
        }
    }

    /// <summary>Explicit graphics export; not part of the normal test suite.</summary>
    public static void RenderFixtureTilingPreviews()
    {
        if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null)
            throw new InvalidOperationException("Fixture menu preview needs a graphics-enabled Unity process.");
        string directory = Environment.GetEnvironmentVariable("FIXTURE_TILING_PREVIEW_DIR");
        if (string.IsNullOrEmpty(directory)) directory = Path.GetFullPath("FixtureTilingPreviews");
        Directory.CreateDirectory(directory);
        var fixture = new FixtureTilingMenuTests();
        GameObject cameraOwner = null;
        RenderTexture target = null;
        Texture2D sheet = null;
        Texture2D pixels = null;
        try
        {
            fixture.BuildMenuWithoutRuntimeStart();
            Component controller = fixture._owner.AddComponent(RuntimeType("FixtureTilingController"));
            fixture._menu.GetType().GetField("_fixtureTiling", Private).SetValue(fixture._menu, controller);
            Canvas canvas = fixture.Field<Canvas>("_canvas");
            canvas.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            fixture.Field<RectTransform>("_panel").gameObject.SetActive(true);
            fixture.Field<RectTransform>("_alignmentView").gameObject.SetActive(false);
            fixture.Field<RectTransform>("_pointView").gameObject.SetActive(false);
            fixture.Field<Component>("_grabHandle").gameObject.SetActive(true);
            cameraOwner = new GameObject("Fixture menu offline preview camera", typeof(Camera));
            Camera camera = cameraOwner.GetComponent<Camera>();
            camera.transform.position = new Vector3(0, 0.02f, -1);
            camera.orthographic = true;
            camera.orthographicSize = 0.43f;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.08f, 0.11f, 0.15f, 1);
            camera.allowHDR = camera.allowMSAA = false;
            canvas.worldCamera = camera;
            target = new RenderTexture(600, 960, 24, RenderTextureFormat.ARGB32);
            target.Create();
            camera.targetTexture = target;
            pixels = new Texture2D(600, 960, TextureFormat.RGB24, false);
            sheet = new Texture2D(1800, 2880, TextureFormat.RGB24, false);
            Type textType = RuntimeType("TMPro.TMP_Text");
            Type fontType = RuntimeType("TMPro.TMP_FontAsset");
            UnityEngine.Object font = Resources.Load("Fonts & Materials/LiberationSans SDF", fontType);
            if (font == null)
            {
                string guid = AssetDatabase.FindAssets("LiberationSans SDF t:TMP_FontAsset").FirstOrDefault();
                if (guid != null) font = AssetDatabase.LoadAssetAtPath(AssetDatabase.GUIDToAssetPath(guid), fontType);
            }
            if (font == null) throw new FileNotFoundException("No font is available for fixture previews.");
            for (int image = 0; image < 9; image++)
            {
                fixture.Field<RectTransform>("_taskView").gameObject.SetActive(image == 0);
                if (image == 0)
                {
                    fixture.Invoke("RefreshTask");
                    fixture.Field<Component>("_fixtureSetupButton").gameObject.SetActive(true);
                    fixture.SetPreviewText("_status", "Kenos SL selected · matching ROS model");
                    fixture.SetPreviewText("_taskHelp", "Set tile, stack, pattern and fixture.\nChoose a point, then trigger to capture.");
                    fixture.Field<Component>("_taskHelp").GetComponent<RectTransform>().sizeDelta = new Vector2(268, 45);
                    fixture.SetPreviewText("_taskStatus", "Fixture tiling selected. Set up measured tile dimensions, the stack, release region and placement pattern.");
                    fixture.SetPreviewText("_qrStatus", "Robot alignment valid · calibrated coordinates in ur_base_link.\nRepresentative offline UI state; no ROS connection.");
                    fixture.SetPreviewButtonColor("_tcpTaskButton", false);
                    fixture.SetPreviewButtonColor("_pickPlaceTaskButton", false);
                    fixture.SetPreviewButtonColor("_fixtureTaskButton", true);
                }
                else
                {
                    fixture._menu.GetType().GetField("_fixturePage", Private).SetValue(fixture._menu, image - 1);
                    fixture.Invoke("BuildFixturePage");
                }
                fixture.Field<RectTransform>("_fixtureView").gameObject.SetActive(image > 0);
                foreach (Component text in canvas.GetComponentsInChildren(textType, true))
                    textType.GetProperty("font").SetValue(text, font);
                foreach (Component button in canvas.GetComponentsInChildren(RuntimeType("UnityEngine.UI.Button"), true))
                    button.GetType().GetProperty("interactable").SetValue(button, true);
                Canvas.ForceUpdateCanvases();
                foreach (Component text in canvas.GetComponentsInChildren(textType, true))
                    textType.GetMethod("ForceMeshUpdate").Invoke(text, new object[] { true, true });
                Canvas.ForceUpdateCanvases();
                RectTransform inspected = fixture.Field<RectTransform>(image == 0 ? "_taskView" : "_fixtureView");
                foreach (Component text in inspected.GetComponentsInChildren(textType, false))
                    if ((bool)textType.GetProperty("isTextOverflowing").GetValue(text))
                        Debug.LogWarning("FixturePreviewTextOverflow: image=" + image + " object=" + text.name +
                            " text=" + textType.GetProperty("text").GetValue(text));
                camera.Render();
                RenderTexture.active = target;
                pixels.ReadPixels(new Rect(0, 0, target.width, target.height), 0, 0);
                pixels.Apply();
                string name = image == 0 ? "fixture-task-entry" : "fixture-page-" + image;
                File.WriteAllBytes(Path.Combine(directory, name + ".png"), pixels.EncodeToPNG());
                sheet.SetPixels((image % 3) * 600, (2 - image / 3) * 960, 600, 960, pixels.GetPixels());
            }
            sheet.Apply();
            string output = Path.Combine(directory, "fixture-menu-contact-sheet.png");
            File.WriteAllBytes(output, sheet.EncodeToPNG());
            Assert.That(Resources.FindObjectsOfTypeAll(RuntimeType("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
            Debug.Log("FixtureTilingMenuPreviews: " + output);
        }
        finally
        {
            if (cameraOwner != null) cameraOwner.GetComponent<Camera>().targetTexture = null;
            RenderTexture.active = null;
            if (pixels != null) UnityEngine.Object.DestroyImmediate(pixels);
            if (sheet != null) UnityEngine.Object.DestroyImmediate(sheet);
            if (target != null) { target.Release(); UnityEngine.Object.DestroyImmediate(target); }
            fixture.Cleanup();
            if (cameraOwner != null) UnityEngine.Object.DestroyImmediate(cameraOwner);
        }
    }

    private void SetPreviewText(string field, string value)
    {
        Component text = Field<Component>(field);
        text.GetType().GetProperty("text").SetValue(text, value);
    }

    private void SetPreviewButtonColor(string field, bool selected)
    {
        Component button = Field<Component>(field);
        object image = button.GetType().GetProperty("image").GetValue(button);
        image.GetType().GetProperty("color").SetValue(image, selected
            ? new Color(0.08f, 0.42f, 0.52f, 1) : new Color(0.12f, 0.17f, 0.23f, 1));
    }

    private void Invoke(string name) => _menu.GetType().GetMethod(name, Private).Invoke(_menu, null);
    private T Field<T>(string name) => (T)_menu.GetType().GetField(name, Private).GetValue(_menu);
    private static Type RuntimeType(string name) => AppDomain.CurrentDomain.GetAssemblies()
        .Select(assembly => assembly.GetType(name)).First(type => type != null);
}
