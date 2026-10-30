using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>Offline layout and pre-placement access checks for the four-point menu.</summary>
public sealed class PointPlacementMenuTests
{
    private GameObject _owner;
    private Component _menu;
    private Component _guideRegistration;

    [SetUp]
    public void CreateWithoutRuntimeStart() => CreateMenu(false);

    private void CreateMenu(bool includeLogo)
    {
        _owner = new GameObject("Offline point placement menu");
        _owner.SetActive(false);
        _menu = _owner.AddComponent(RuntimeType("ToolSelectionMenu"));
        if (includeLogo)
        {
            Texture2D logo = AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Branding/UnityROSML/UnityROSML-Logo-Transparent.png");
            if (logo == null) throw new FileNotFoundException("The app logo is required for the menu preview.");
            _menu.GetType().GetField("_appLogo", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(_menu, logo);
        }
        Call("BuildMenu");
    }

    [TearDown]
    public void Cleanup()
    {
        if (_guideRegistration != null) SetCollecting(_guideRegistration, false);
        Canvas canvas = Field<Canvas>("_canvas");
        if (canvas != null) UnityEngine.Object.DestroyImmediate(canvas.gameObject);
        if (_owner != null) UnityEngine.Object.DestroyImmediate(_owner);
    }

    [TestCase(0, "Shoulder")]
    [TestCase(1, "Elbow")]
    [TestCase(2, "Wrist 2")]
    [TestCase(3, "Wrist 3")]
    public void MiniatureFollowsTheCurrentCaptureStepAndHidesAfterLeavingPlacement(int step, string cover)
    {
        Component guide = Field<Component>("_pointGuide");
        Assert.That(guide, Is.Not.Null);
        Assert.That((bool)guide.GetType().GetProperty("HasVisual").GetValue(guide), Is.False);
        ShowGuideForStep(step);
        Assert.That((bool)guide.GetType().GetProperty("HasVisual").GetValue(guide), Is.True);
        Assert.That((int)guide.GetType().GetProperty("SelectedPointIndex").GetValue(guide), Is.EqualTo(step));
        Assert.That((string)guide.GetType().GetProperty("CurrentPointName").GetValue(guide), Is.EqualTo(cover));
        Assert.That(Field<object>("_pointGuideTitle").GetType().GetProperty("text")
            .GetValue(Field<object>("_pointGuideTitle")), Is.EqualTo((step + 1) + " / 4 · " + cover));
        Assert.That(guide.GetComponentInParent<Canvas>(), Is.SameAs(Field<Canvas>("_canvas")));
        Assert.That(guide.transform.IsChildOf(Field<RectTransform>("_pointView")), Is.True);

        SetCollecting(_guideRegistration, false);
        Call("RefreshPointPlacementControls");
        Assert.That(Field<RectTransform>("_pointView").gameObject.activeSelf, Is.False);
        Assert.That((bool)guide.GetType().GetProperty("HasVisual").GetValue(guide), Is.False);
        Assert.That(Resources.FindObjectsOfTypeAll(RuntimeType("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
    }

    private void ShowGuideForStep(int step)
    {
        Field<RectTransform>("_panel").gameObject.SetActive(true);
        Field<RectTransform>("_taskView").gameObject.SetActive(false);
        Field<RectTransform>("_alignmentView").gameObject.SetActive(false);
        if (_guideRegistration == null)
        {
            Component instantiator = _owner.AddComponent(RuntimeType("URInstantiator"));
            GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab");
            instantiator.GetType().GetField("_URPrefab", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(instantiator, prefab);
            _guideRegistration = _owner.AddComponent(RuntimeType("RobotPointRegistration"));
            _guideRegistration.GetType().GetField("_instantiator", BindingFlags.NonPublic | BindingFlags.Instance)
                .SetValue(_guideRegistration, instantiator);
            SetMenuField("_instantiator", instantiator);
            SetMenuField("_pointRegistration", _guideRegistration);
        }
        var points = (System.Collections.IList)_guideRegistration.GetType()
            .GetField("_measuredInTracking", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(_guideRegistration);
        points.Clear();
        for (int index = 0; index < step; index++) points.Add(Vector3.right * index * 0.1f);
        SetCollecting(_guideRegistration, true);
        Call("RefreshPointPlacementControls");
    }

    [Test]
    public void FloatingMiniatureStaysCenteredOnItsCardForPerspectiveViewers()
    {
        GameObject cameraOwner = new GameObject("Miniature perspective viewer", typeof(Camera));
        try
        {
            Camera camera = cameraOwner.GetComponent<Camera>();
            camera.enabled = false;
            camera.orthographic = false;
            camera.fieldOfView = 46f;
            Vector3[] viewpoints = { new Vector3(0f, 0f, -1f), new Vector3(-0.15f, 0.1f, -1f), new Vector3(0.2f, -0.08f, -0.8f) };
            for (int step = 0; step < 4; step++)
            {
                ShowGuideForStep(step);
                Component guide = Field<Component>("_pointGuide");
                Transform anchor = Field<Transform>("_pointGuideAnchor");
                foreach (Vector3 viewpoint in viewpoints)
                {
                    camera.transform.position = viewpoint;
                    camera.transform.LookAt(Vector3.zero);
                    _menu.GetType().GetMethod("PositionPointGuide", BindingFlags.NonPublic | BindingFlags.Instance)
                        .Invoke(_menu, new object[] { camera });
                    Vector3 onCard = camera.WorldToViewportPoint(anchor.position);
                    Vector3 floating = camera.WorldToViewportPoint(guide.transform.position);
                    Assert.That(Vector2.Distance(onCard, floating), Is.LessThan(0.00001f),
                        "Forward depth must not push the guide sideways away from its sidebar.");
                    Assert.That(guide.transform.localScale.x, Is.LessThan(1f),
                        "Compensate perspective magnification of the floating model.");
                    Assert.That(floating.z, Is.LessThan(onCard.z));
                }
            }
        }
        finally { UnityEngine.Object.DestroyImmediate(cameraOwner); }
    }

    private static void SetCollecting(Component registration, bool collecting) => registration.GetType()
        .GetField("<IsCollecting>k__BackingField", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(registration, collecting);

    private void SetMenuField(string name, object value) => _menu.GetType()
        .GetField(name, BindingFlags.NonPublic | BindingFlags.Instance).SetValue(_menu, value);

    [Test]
    public void PointViewGivesOrderedPhysicalCaptureInstructionsAndAllActionsFit()
    {
        RectTransform pointView = Field<RectTransform>("_pointView");
        Assert.That(pointView.gameObject.activeSelf, Is.False);
        Type textType = RuntimeType("TMPro.TMP_Text");
        string[] labels = pointView.GetComponentsInChildren(textType, true)
            .Select(text => (string)textType.GetProperty("text").GetValue(text)).ToArray();
        Assert.That(labels.Any(text => text.Contains("controller tip") && text.Contains("trigger")), Is.True);
        Assert.That(labels.Any(text => text.StartsWith("1  Shoulder")), Is.True);
        Assert.That(labels.Any(text => text.StartsWith("2  Elbow")), Is.True);
        Assert.That(labels.Any(text => text.StartsWith("3  Wrist 2")), Is.True);
        Assert.That(labels.Any(text => text.StartsWith("4  Wrist 3")), Is.True);
        Type buttonType = RuntimeType("UnityEngine.UI.Button");
        Component[] buttons = pointView.GetComponentsInChildren(buttonType, true);
        foreach (string name in new[] { "Undo last point", "Restart 4 points", "Cancel placement", "Use QR instead", "Recenter point menu" })
            Assert.That(buttons.Any(button => button.name == name), Is.True, name);
        var occupied = new List<Rect>();
        foreach (Component button in buttons)
        {
            RectTransform rect = (RectTransform)button.transform;
            Rect bounds = new Rect(rect.anchoredPosition.x, -rect.anchoredPosition.y, rect.sizeDelta.x, rect.sizeDelta.y);
            Assert.That(bounds.xMin, Is.GreaterThanOrEqualTo(0f), button.name);
            Assert.That(bounds.yMin, Is.GreaterThanOrEqualTo(0f), button.name);
            Assert.That(bounds.xMax, Is.LessThanOrEqualTo(pointView.sizeDelta.x), button.name);
            Assert.That(bounds.yMax, Is.LessThanOrEqualTo(pointView.sizeDelta.y), button.name);
            foreach (Rect other in occupied) Assert.That(bounds.Overlaps(other), Is.False, button.name);
            occupied.Add(bounds);
        }
    }

    /// <summary>One offline entry point for the contour fix, loop menu and miniature guide images.</summary>
    public static void RenderInteractionPreviews()
    {
        RobotContourRenderingTests.RenderRobotContourPreviews();
        RenderPointPlacementPreview();
        RobotRegistrationGuideTests.RenderRegistrationGuidePreviews();
        Debug.Log("RobotInteractionPreviews passed: contours, playback menu and four registration guide steps.");
    }

    /// <summary>Graphics-enabled, connection-free preview of the actual generated placement UI.</summary>
    public static void RenderPointPlacementPreview()
    {
        if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null)
            throw new InvalidOperationException("Point-placement preview requires a graphics-enabled Unity process.");
        var preview = new PointPlacementMenuTests();
        GameObject cameraOwner = null;
        RenderTexture target = null;
        Texture2D pixels = null;
        try
        {
            preview.CreateMenu(true);
            cameraOwner = new GameObject("Offline point menu preview camera", typeof(Camera));
            Camera camera = cameraOwner.GetComponent<Camera>();
            camera.transform.position = new Vector3(0.1185f, 0f, -1f);
            camera.orthographic = false;
            camera.fieldOfView = 46f;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.08f, 0.11f, 0.15f, 1f);
            Canvas canvas = preview.Field<Canvas>("_canvas");
            canvas.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            canvas.worldCamera = camera;
            preview.Field<RectTransform>("_panel").gameObject.SetActive(true);
            preview.Field<RectTransform>("_taskView").gameObject.SetActive(false);
            preview.Field<RectTransform>("_alignmentView").gameObject.SetActive(false);
            preview.Field<RectTransform>("_pointView").gameObject.SetActive(true);
            preview.ShowGuideForStep(1);
            preview._menu.GetType().GetMethod("PositionPointGuide", BindingFlags.NonPublic | BindingFlags.Instance)
                .Invoke(preview._menu, new object[] { camera });
            preview.Field<Component>("_grabHandle").gameObject.SetActive(true);
            preview.SetText("_pointProgress", "1 / 4 points captured\nNext: Elbow rear blue cover");
            preview.SetText("_pointStatus", "Keep the robot still. Touch 2/4: the elbow cover center, then press trigger.");
            Type textType = RuntimeType("TMPro.TMP_Text");
            Type fontType = RuntimeType("TMPro.TMP_FontAsset");
            UnityEngine.Object font = Resources.Load("Fonts & Materials/LiberationSans SDF", fontType);
            if (font == null)
            {
                string fontId = AssetDatabase.FindAssets("LiberationSans SDF t:TMP_FontAsset").FirstOrDefault();
                if (fontId != null) font = AssetDatabase.LoadAssetAtPath(AssetDatabase.GUIDToAssetPath(fontId), fontType);
            }
            if (font == null) throw new FileNotFoundException("No font available for the menu preview.");
            foreach (Component text in canvas.GetComponentsInChildren(textType, true)) textType.GetProperty("font").SetValue(text, font);
            foreach (Component button in preview.Field<RectTransform>("_pointView").GetComponentsInChildren(RuntimeType("UnityEngine.UI.Button"), true))
                button.GetType().GetProperty("interactable").SetValue(button, true);
            System.Collections.IList labels = preview.Field<System.Collections.IList>("_pointLabels");
            for (int index = 0; index < labels.Count; index++)
            {
                object label = labels[index];
                label.GetType().GetProperty("color").SetValue(label, index == 0
                    ? new Color(0.45f, 0.92f, 0.68f) : index == 1 ? new Color(0.55f, 0.91f, 1f) : new Color(0.73f, 0.81f, 0.87f));
            }
            target = new RenderTexture(1600, 1600, 24, RenderTextureFormat.ARGB32);
            camera.targetTexture = target;
            pixels = new Texture2D(1600, 1600, TextureFormat.RGB24, false);
            Canvas.ForceUpdateCanvases();
            foreach (Component text in canvas.GetComponentsInChildren(textType, true))
                text.GetType().GetMethod("ForceMeshUpdate").Invoke(text, new object[] { true, true });
            Canvas.ForceUpdateCanvases();
            camera.Render();
            RenderTexture.active = target;
            pixels.ReadPixels(new Rect(0f, 0f, target.width, target.height), 0, 0);
            pixels.Apply();
            string output = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "point-placement-menu.png"));
            File.WriteAllBytes(output, pixels.EncodeToPNG());
            Debug.Log("PointPlacementMenuPreview: " + output);
        }
        finally
        {
            if (cameraOwner != null) cameraOwner.GetComponent<Camera>().targetTexture = null;
            RenderTexture.active = null;
            if (pixels != null) UnityEngine.Object.DestroyImmediate(pixels);
            if (target != null) { target.Release(); UnityEngine.Object.DestroyImmediate(target); }
            preview.Cleanup();
            if (cameraOwner != null) UnityEngine.Object.DestroyImmediate(cameraOwner);
        }
    }

    private void SetText(string field, string text)
    {
        object target = Field<object>(field);
        target.GetType().GetProperty("text").SetValue(target, text);
    }

    [Test]
    public void EntryIsAvailableBeforeRobotPlacementAndExplainsMissingSceneComponent()
    {
        object entry = Field<object>("_pointPlacementButton");
        Assert.That(entry, Is.Not.Null);
        Call("BeginPointPlacement");
        Assert.That(Field<string>("_localNotice"), Does.Contain("Four-point placement is unavailable"));
        Assert.That(Field<RectTransform>("_taskView").gameObject.activeSelf, Is.True);
        Assert.That(Field<RectTransform>("_pointView").gameObject.activeSelf, Is.False);
        Type connectionType = RuntimeType("Unity.Robotics.ROSTCPConnector.ROSConnection");
        Assert.That(Resources.FindObjectsOfTypeAll(connectionType), Is.Empty);
    }

    private T Field<T>(string name) => (T)_menu.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic).GetValue(_menu);
    private void Call(string name) => _menu.GetType().GetMethod(name, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic).Invoke(_menu, null);
    private static Type RuntimeType(string name) => AppDomain.CurrentDomain.GetAssemblies()
        .Select(assembly => assembly.GetType(name)).FirstOrDefault(type => type != null)
        ?? throw new InvalidOperationException("Missing runtime type " + name);
}
