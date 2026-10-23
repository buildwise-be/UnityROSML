using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>Menu layout and viewer-relative adjustment controls, without XR tracking or ROS.</summary>
public sealed class ToolCalibrationMenuTests
{
    private GameObject _owner;
    private GameObject _camera;
    private Component _menu;
    private readonly List<GameObject> _previousMainCameras = new List<GameObject>();

    /// <summary>Optional graphics-enabled batch preview; no runtime Start or ROS transport is invoked.</summary>
    public static void RenderCalibrationMenuPreviews()
    {
        if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null)
            throw new InvalidOperationException("Menu rendering requires a graphics-enabled Unity batch process.");
        var preview = new ToolCalibrationMenuTests();
        preview.CreateMenu(true);
        RenderTexture target = null;
        Texture2D pixels = null;
        try
        {
            preview.CreateCamera(Quaternion.identity);
            Camera camera = preview._camera.GetComponent<Camera>();
            camera.transform.position = new Vector3(0f, 0f, -1f);
            camera.orthographic = true;
            camera.orthographicSize = 0.32f;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.08f, 0.11f, 0.15f, 1f);
            Canvas canvas = preview.Get<Canvas>("_canvas");
            canvas.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            canvas.worldCamera = camera;
            preview.Get<RectTransform>("_panel").gameObject.SetActive(true);
            target = new RenderTexture(880, 1160, 24, RenderTextureFormat.ARGB32);
            camera.targetTexture = target;
            pixels = new Texture2D(880, 1160, TextureFormat.RGB24, false);

            Type textType = RuntimeType("TMPro.TMP_Text");
            Type fontType = RuntimeType("TMPro.TMP_FontAsset");
            UnityEngine.Object font = Resources.Load("Fonts & Materials/LiberationSans SDF", fontType);
            if (font == null)
            {
                string fontId = AssetDatabase.FindAssets("LiberationSans SDF t:TMP_FontAsset").FirstOrDefault();
                if (fontId != null) font = AssetDatabase.LoadAssetAtPath(AssetDatabase.GUIDToAssetPath(fontId), fontType);
            }
            foreach (Component text in canvas.GetComponentsInChildren(textType, true))
                if (font != null) textType.GetProperty("font").SetValue(text, font);

            Component input = preview._owner.AddComponent(RuntimeType("URInputManager"));
            preview._menu.GetType().GetField("_input", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(preview._menu, input);
            input.GetType().GetMethod("UseTcpPreview").Invoke(input, null);
            preview.Invoke("RefreshTask");
            preview.SetText("_status", "Offline menu preview. Live ROS model status appears here.");
            preview.SetText("_taskStatus", "Capture TCP A and B, then Plan preview. Inspect the ghost path before Execute.");
            preview.SetText("_qrStatus", "Marker: 87 mm · robot-lab\nSaved QR-to-base alignment is kept on this headset.");
            preview.RenderPreview(camera, target, pixels, "calibration-menu-task.png");

            input.GetType().GetMethod("UseTilePickPlace").Invoke(input, null);
            preview.Invoke("RefreshTask");
            preview.SetText("_taskStatus", "Capture pick and place top surfaces. Plan and inspect the approach, grasp, transfer and release before Execute.");
            preview.SetText("_qrStatus", "Marker: 87 mm · robot-lab\nSaved QR-to-base alignment is kept on this headset.");
            preview.RenderPreview(camera, target, pixels, "calibration-menu-pick-place.png");

            preview.Get<RectTransform>("_taskView").gameObject.SetActive(false);
            preview.Get<RectTransform>("_alignmentView").gameObject.SetActive(true);
            foreach (Component button in canvas.GetComponentsInChildren(RuntimeType("UnityEngine.UI.Button"), true))
                button.GetType().GetProperty("interactable").SetValue(button, true);
            preview.SetText("_alignmentStatus", "QR: robot-lab · saved on headset\nMatch the base and separated links, then Save.\nUnsaved alignment: QR to base (−0.419, 0.230, 0.505) m; Euler (−90, 0, 0) deg.");
            preview.RenderPreview(camera, target, pixels, "calibration-menu-alignment.png");
        }
        finally
        {
            if (preview._camera != null) preview._camera.GetComponent<Camera>().targetTexture = null;
            RenderTexture.active = null;
            if (pixels != null) UnityEngine.Object.DestroyImmediate(pixels);
            if (target != null) { target.Release(); UnityEngine.Object.DestroyImmediate(target); }
            preview.Cleanup();
        }
    }

    private void SetText(string field, string text)
    {
        object component = Get<object>(field);
        component.GetType().GetProperty("text").SetValue(component, text);
    }

    private void RenderPreview(Camera camera, RenderTexture target, Texture2D pixels, string name)
    {
        Canvas.ForceUpdateCanvases();
        foreach (Component text in Get<Canvas>("_canvas").GetComponentsInChildren(RuntimeType("TMPro.TMP_Text"), true))
            text.GetType().GetMethod("ForceMeshUpdate").Invoke(text, new object[] { true, true });
        Canvas.ForceUpdateCanvases();
        camera.Render();
        RenderTexture.active = target;
        pixels.ReadPixels(new Rect(0f, 0f, target.width, target.height), 0, 0);
        pixels.Apply();
        string output = Path.GetFullPath(Path.Combine(Application.dataPath, "..", name));
        File.WriteAllBytes(output, pixels.EncodeToPNG());
        Debug.Log("CalibrationMenuPreview: " + output);
    }

    [SetUp]
    public void CreateMenuWithoutRuntimeStart() => CreateMenu(false);

    private void CreateMenu(bool includeLogo)
    {
        Texture2D logo = includeLogo ? AssetDatabase.LoadAssetAtPath<Texture2D>(
            "Assets/Branding/UnityROSML/UnityROSML-Logo-Transparent.png") : null;
        if (includeLogo && logo == null)
            throw new FileNotFoundException("The existing app logo is required for the branded menu preview.");
        _owner = new GameObject("Offline calibration menu");
        _owner.SetActive(false);
        _menu = _owner.AddComponent(RuntimeType("ToolSelectionMenu"));
        if (includeLogo) _menu.GetType().GetField("_appLogo", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(_menu, logo);
        Invoke("BuildMenu");
    }

    [TearDown]
    public void Cleanup()
    {
        Canvas canvas = Get<Canvas>("_canvas");
        if (canvas != null) UnityEngine.Object.DestroyImmediate(canvas.gameObject);
        UnityEngine.Object.DestroyImmediate(_owner);
        if (_camera != null) UnityEngine.Object.DestroyImmediate(_camera);
        foreach (GameObject existing in _previousMainCameras)
            if (existing != null) existing.tag = "MainCamera";
        _previousMainCameras.Clear();
    }

    [Test]
    public void AlignmentActionsFitTheExistingPanelAndOnlyTaskViewStartsVisible()
    {
        RectTransform task = Get<RectTransform>("_taskView");
        RectTransform alignment = Get<RectTransform>("_alignmentView");
        Assert.That(task.gameObject.activeSelf, Is.True);
        Assert.That(alignment.gameObject.activeSelf, Is.False);
        Type buttonType = RuntimeType("UnityEngine.UI.Button");
        Component[] actions = alignment.GetComponentsInChildren(buttonType, true);
        foreach (string name in new[] { "Save & finish", "Cancel changes", "Reset draft to prefab", "Left", "Right", "Up", "Down", "Towards you", "Away from you", "Yaw −", "Yaw +", "Pitch −", "Pitch +", "Roll −", "Roll +" })
            Assert.That(actions.Any(action => action.name == name), Is.True, name);

        foreach (RectTransform view in new[] { task, alignment })
        {
            var rectangles = new List<Rect>();
            foreach (Component button in view.GetComponentsInChildren(buttonType, true))
            {
                RectTransform rect = (RectTransform)button.transform;
                Rect bounds = new Rect(rect.anchoredPosition.x, -rect.anchoredPosition.y, rect.sizeDelta.x, rect.sizeDelta.y);
                Assert.That(bounds.xMin, Is.GreaterThanOrEqualTo(0f), button.name);
                Assert.That(bounds.yMin, Is.GreaterThanOrEqualTo(0f), button.name);
                Assert.That(bounds.xMax, Is.LessThanOrEqualTo(view.sizeDelta.x), button.name);
                Assert.That(bounds.yMax, Is.LessThanOrEqualTo(view.sizeDelta.y), button.name);
                foreach (Rect other in rectangles) Assert.That(bounds.Overlaps(other), Is.False, "Calibration buttons must not overlap.");
                rectangles.Add(bounds);
            }
        }
    }

    [Test]
    public void MovementBasisUsesHorizontalStartingViewAndDoesNotFollowLaterHeadTurns()
    {
        CreateCamera(Quaternion.Euler(25f, 90f, 0f));
        Invoke("CaptureAlignmentFrame");
        Vector3 forward = Get<Vector3>("_alignmentForward");
        Vector3 right = Get<Vector3>("_alignmentRight");
        Assert.That(Vector3.Distance(forward, Vector3.right), Is.LessThan(0.00001f));
        Assert.That(Vector3.Distance(right, Vector3.back), Is.LessThan(0.00001f));
        _camera.transform.rotation = Quaternion.Euler(0f, 180f, 0f);
        Assert.That(Get<Vector3>("_alignmentForward"), Is.EqualTo(forward));
        Assert.That(Get<Vector3>("_alignmentRight"), Is.EqualTo(right));
    }

    [Test]
    public void LookingStraightUpStillProducesRigidHorizontalMovementAxes()
    {
        CreateCamera(Quaternion.Euler(-90f, 37f, 0f));
        Invoke("CaptureAlignmentFrame");
        Vector3 forward = Get<Vector3>("_alignmentForward");
        Vector3 right = Get<Vector3>("_alignmentRight");
        Assert.That(forward.magnitude, Is.EqualTo(1f).Within(0.00001f));
        Assert.That(right.magnitude, Is.EqualTo(1f).Within(0.00001f));
        Assert.That(Mathf.Abs(forward.y), Is.LessThan(0.00001f));
        Assert.That(Mathf.Abs(right.y), Is.LessThan(0.00001f));
        Assert.That(Mathf.Abs(Vector3.Dot(forward, right)), Is.LessThan(0.00001f));
    }

    [Test]
    public void AlignmentEntryExplainsMissingPlacementWithoutHidingTaskControls()
    {
        _menu.GetType().GetMethod("BeginAlignment").Invoke(_menu, null);
        Assert.That(Get<string>("_localNotice"), Does.Contain("QR"));
        Assert.That(Get<RectTransform>("_taskView").gameObject.activeSelf, Is.True);
        Assert.That(Get<RectTransform>("_alignmentView").gameObject.activeSelf, Is.False);
        Type connectionType = RuntimeType("Unity.Robotics.ROSTCPConnector.ROSConnection");
        Assert.That(Resources.FindObjectsOfTypeAll(connectionType), Is.Empty, "Opening alignment must not start ROS.");
    }

    private void CreateCamera(Quaternion rotation)
    {
        // Graphics-enabled editor runs may retain the default scene camera.
        // Make the fixture's starting view unambiguous without deleting it.
        foreach (Camera existing in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (existing.CompareTag("MainCamera"))
            {
                _previousMainCameras.Add(existing.gameObject);
                existing.gameObject.tag = "Untagged";
            }
        _camera = new GameObject("Calibration starting view", typeof(Camera));
        _camera.tag = "MainCamera";
        _camera.transform.rotation = rotation;
        Assert.That(Camera.main, Is.EqualTo(_camera.GetComponent<Camera>()), "The fixture must own the starting view.");
    }

    private T Get<T>(string name) => (T)_menu.GetType().GetField(name, BindingFlags.NonPublic | BindingFlags.Instance).GetValue(_menu);
    private void Invoke(string name) => _menu.GetType().GetMethod(name, BindingFlags.NonPublic | BindingFlags.Instance).Invoke(_menu, null);
    private static Type RuntimeType(string name) => AppDomain.CurrentDomain.GetAssemblies()
        .Select(assembly => assembly.GetType(name)).FirstOrDefault(type => type != null)
        ?? throw new InvalidOperationException("Missing runtime type " + name);
}
