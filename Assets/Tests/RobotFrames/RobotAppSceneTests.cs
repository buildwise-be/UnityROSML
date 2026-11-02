using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

/// <summary>Exact serialized scene contracts and real guarded navigation, without loading XR hardware scenes.</summary>
public sealed class RobotAppSceneTests
{
    [Serializable] private sealed class SceneEntry { public string name, path, resource, sha256, guid; }
    [Serializable] private sealed class ScriptEntry { public string name, path, guid; }
    [Serializable] private sealed class BuildEntry { public string resource, sha256; }
    [Serializable] private sealed class Manifest { public SceneEntry[] scenes; public ScriptEntry[] scripts; public BuildEntry buildSettings; }
    private readonly List<GameObject> _owned = new List<GameObject>();
    private Manifest _manifest;
    private static Type Navigation => Runtime("RobotAppNavigation");

    [SetUp]
    public void ResetNavigationAndLoadExactSceneSnapshots()
    {
        Static("Reset");
        TextAsset manifest = Resources.Load<TextAsset>("RobotAppScenes/manifest");
        if (manifest != null) _manifest = JsonUtility.FromJson<Manifest>(manifest.text);
    }

    [TearDown]
    public void Cleanup()
    {
        Static("Reset");
        foreach (GameObject value in _owned.AsEnumerable().Reverse())
            if (value != null) UnityEngine.Object.DestroyImmediate(value);
        _owned.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(Runtime("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
    }

    [Test]
    public void BuildStartsAtWelcomeAndContainsExactlyTheTwoDedicatedWorkflows()
    {
        RequireSnapshots();
        string build = Resource(_manifest.buildSettings.resource);
        AssertHash(build, _manifest.buildSettings.sha256);
        MatchCollection scenes = Regex.Matches(build, @"(?m)^  - enabled: (\d+)\r?\n    path: (.+)\r?\n    guid: ([0-9a-f]{32})");
        Assert.That(scenes.Count, Is.EqualTo(3));
        string[] expected = { "Welcome", "SimplePickAndPlace", "RepetitivePickAndPlace" };
        for (int i = 0; i < expected.Length; i++)
        {
            SceneEntry entry = _manifest.scenes.Single(value => value.name == expected[i]);
            Assert.That(scenes[i].Groups[1].Value, Is.EqualTo("1"));
            Assert.That(scenes[i].Groups[2].Value.Trim(), Is.EqualTo(entry.path));
            Assert.That(scenes[i].Groups[3].Value, Is.EqualTo(entry.guid));
            AssertHash(Resource(entry.resource), entry.sha256);
        }
    }

    [Test]
    public void WelcomeContainsOnlyTheWelcomeWorkflowAndTheSharedXrRig()
    {
        RequireSnapshots();
        string scene = SceneText("Welcome");
        Assert.That(Components(scene, "RobotWelcomeMenu").Length, Is.EqualTo(1));
        foreach (string type in new[] { "URInputManager", "PickPlaceController", "TcpPlanningController", "MTCPlansManager", "FixtureTilingController", "ROSService" })
            Assert.That(Components(scene, type), Is.Empty, "Welcome must not instantiate " + type);
        Assert.That(Regex.Matches(scene, @"m_SourcePrefab: \{fileID: 100100000, guid: 2ad7d02c3ee2a447abc9de300646685d").Count, Is.EqualTo(1));
        Assert.That(scene, Does.Not.Contain("bb85235baaf618b44b9592f3ac6ca0ab"), "No auto-connecting ROS prefab belongs in Welcome.");
    }

    [TestCase("Welcome")]
    [TestCase("SimplePickAndPlace")]
    [TestCase("RepetitivePickAndPlace")]
    public void SerializedSceneHasUniqueDocumentsAndNoDanglingLocalReferences(string name)
    {
        RequireSnapshots();
        string scene = SceneText(name);
        string[] ids = Regex.Matches(scene, @"(?m)^--- !u!\d+ &(-?\d+)").Cast<Match>().Select(match => match.Groups[1].Value).ToArray();
        Assert.That(ids.Distinct().Count(), Is.EqualTo(ids.Length));
        var declared = new HashSet<string>(ids);
        foreach (Match reference in Regex.Matches(scene, @"\{fileID: (-?\d+)\}"))
            if (reference.Groups[1].Value != "0") Assert.That(declared.Contains(reference.Groups[1].Value), Is.True, "Missing local fileID " + reference.Groups[1].Value);
    }

    [TestCase("SimplePickAndPlace", 1, 1)]
    [TestCase("RepetitivePickAndPlace", 2, 2)]
    public void DedicatedSceneSerializesItsWorkflowAndActualControllerReferences(string name, int workflow, int capture)
    {
        RequireSnapshots();
        string scene = SceneText(name), input = Components(scene, "URInputManager").Single(), menu = Components(scene, "ToolSelectionMenu").Single();
        Assert.That(IntField(input, "_workflowMode"), Is.EqualTo(workflow));
        Assert.That(IntField(menu, "_workflowMode"), Is.EqualTo(workflow));
        Assert.That(IntField(input, "_poseCaptureMode"), Is.EqualTo(capture));
        AssertReference(scene, input, "_URInstantiator", "URInstantiator");
        AssertReference(scene, menu, "_input", "URInputManager");
        AssertReference(scene, menu, "_controller", "ToolSelectionController");
        AssertReference(scene, menu, "_instantiator", "URInstantiator");
        AssertReference(scene, menu, "_trajectoryVisualizer", "UR16eTrajectoryVisualizer");
        if (workflow == 1)
        {
            AssertReference(scene, input, "_pickPlaceController", "PickPlaceController");
            AssertReference(scene, input, "_tcpTargetPreview", "TcpTargetPreview");
            AssertReference(scene, input, "_tcpPlanningController", "TcpPlanningController");
            Assert.That(Components(scene, "FixtureTilingController"), Is.Empty);
            Assert.That(Components(scene, "MTCPlansManager").Length, Is.EqualTo(1));
        }
        else
        {
            AssertReference(scene, input, "_fixtureTilingController", "FixtureTilingController");
            foreach (string type in new[] { "PickPlaceController", "TcpTargetPreview", "TcpPlanningController", "MTCPlansManager" })
                Assert.That(Components(scene, type), Is.Empty, "Repetitive scene must not retain " + type);
        }
        string service = Components(scene, "ROSService").Single();
        Assert.That(service, Does.Contain("_rosIPAddress: 10.200.101.144"));
        Assert.That(IntField(service, "_rosPort"), Is.EqualTo(10000));
        Assert.That(scene, Does.Not.Contain("bb85235baaf618b44b9592f3ac6ca0ab"));
        Assert.That(Regex.Matches(scene, @"m_SourcePrefab: \{fileID: 100100000, guid: 2ad7d02c3ee2a447abc9de300646685d").Count, Is.EqualTo(1));
    }

    [TestCase("Welcome")]
    [TestCase("SimplePickAndPlace")]
    [TestCase("RepetitivePickAndPlace")]
    public void NavigationRequestsOneKnownSceneAndRevokesOldAlignmentBeforeLoading(string target)
    {
        Component coordinates = AddInactive("Navigation coordinates", "RobotCoordinateService");
        Component state = AddInactive("Navigation robot state", "RobotStateService");
        ArticulationBody body = New("base_link").AddComponent<ArticulationBody>();
        Call(coordinates, "Initialize", body, New("Navigation tracking").transform);
        state.GetType().GetProperty("IsReadyForPlanning").SetValue(state, true);
        int calls = 0;
        bool accepted = Navigate(target, name => {
            calls++; Assert.That(name, Is.EqualTo(target));
            Assert.That(Get<bool>(coordinates, "IsInitialized"), Is.False);
            Assert.That(Get<bool>(state, "IsReadyForPlanning"), Is.False);
        });
        Assert.That(accepted, Is.True); Assert.That(calls, Is.EqualTo(1));
        Assert.That(CanNavigate, Is.False, "Another click cannot race the scene transition.");
        Assert.That(Navigate(target, _ => calls++), Is.False);
        Assert.That(calls, Is.EqualTo(1));
        Static("OnSceneLoaded", default(Scene), LoadSceneMode.Single);
        Assert.That(CanNavigate, Is.True);
    }

    [TestCase(false)]
    [TestCase(true)]
    public void PendingOrUnknownExecutionBlocksNavigationBeforeAnyReset(bool legacyManagerOnly)
    {
        Component coordinates = AddInactive("Busy navigation coordinates", "RobotCoordinateService");
        ArticulationBody body = New("base_link").AddComponent<ArticulationBody>();
        Call(coordinates, "Initialize", body, New("Busy navigation tracking").transform);
        int revision = Get<int>(coordinates, "CalibrationRevision");
        GameObject owner = New("Pending robot command");
        if (legacyManagerOnly)
        {
            Component manager = owner.AddComponent(Runtime("MTCPlansManager"));
            manager.GetType().GetField("_executionRequested", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(manager, true);
        }
        else Runtime("RobotExecutionState").GetMethod("SetBusy").Invoke(null, new object[] { owner, true });
        int calls = 0;
        Assert.That(CanNavigate, Is.False);
        Assert.That(Navigate("Welcome", _ => calls++), Is.False);
        Assert.That(calls, Is.Zero);
        Assert.That(Get<int>(coordinates, "CalibrationRevision"), Is.EqualTo(revision));
        Assert.That(Get<bool>(coordinates, "IsInitialized"), Is.True);
    }

    [Test]
    public void UnknownDestinationAndLoaderFailureCannotLeaveAStuckNavigationLock()
    {
        int calls = 0;
        Assert.That(Navigate("UR16eInteraction", _ => calls++), Is.False);
        Assert.That(Navigate("Welcome", null), Is.False); Assert.That(calls, Is.Zero);
        LogAssert.Expect(LogType.Error, "Could not open the selected workflow: Offline loader failure");
        Assert.That(Navigate("Welcome", _ => throw new InvalidOperationException("Offline loader failure")), Is.False);
        Assert.That(CanNavigate, Is.True);
    }

    [Test]
    public void WelcomeBuildIsIdempotentAndHasExactlyTheTwoRequestedActionsWithoutTransport()
    {
        Component menu = AddInactive("Offline welcome", "RobotWelcomeMenu");
        Private(menu, "BuildMenu"); Private(menu, "BuildMenu");
        Component[] buttons = menu.GetComponentsInChildren(Runtime("UnityEngine.UI.Button"), true);
        CollectionAssert.AreEquivalent(new[] { "Simple Pick and Place", "Repetitive Pick and Place" }, buttons.Select(button => button.name).ToArray());
        Assert.That(menu.GetComponentsInChildren<Canvas>(true).Length, Is.EqualTo(1));
        Assert.That(Resources.FindObjectsOfTypeAll(Runtime("URInputManager")), Is.Empty);
        Type service = Runtime("ROSService");
        if (typeof(UnityEngine.Object).IsAssignableFrom(service)) Assert.That(Resources.FindObjectsOfTypeAll(service), Is.Empty);
        GameObject lockOwner = New("Welcome command lock");
        Runtime("RobotExecutionState").GetMethod("SetBusy").Invoke(null, new object[] { lockOwner, true });
        Private(menu, "LateUpdate");
        foreach (Component button in buttons)
        {
            Assert.That(Get<bool>(button, "interactable"), Is.False);
            ((UnityEngine.Events.UnityEvent)button.GetType().GetProperty("onClick").GetValue(button)).Invoke();
        }
        Assert.That(menu != null, Is.True, "Blocked welcome actions cannot unload the current scene.");
    }

    /// <summary>Actual welcome and both dedicated menus under the application's mobile URP renderer.</summary>
    public static void RenderSceneWorkflowPreviews()
    {
        var fixture = new RobotAppSceneTests();
        var previousMainCameras = new List<GameObject>();
        RenderTexture target = null, previous = RenderTexture.active;
        Texture2D pixels = null;
        Camera camera = null;
        try
        {
            foreach (Camera existing in UnityEngine.Object.FindObjectsByType<Camera>(FindObjectsInactive.Include, FindObjectsSortMode.None))
                if (existing.CompareTag("MainCamera")) { previousMainCameras.Add(existing.gameObject); existing.tag = "Untagged"; }
            GameObject viewer = fixture.New("Welcome preview viewer"); viewer.tag = "MainCamera";
            camera = viewer.AddComponent<Camera>(); viewer.SetActive(true);
            camera.transform.SetPositionAndRotation(new Vector3(0, 0, -1), Quaternion.identity);
            camera.orthographic = true; camera.orthographicSize = .25f; camera.aspect = 1.3f;
            camera.clearFlags = CameraClearFlags.SolidColor; camera.backgroundColor = new Color(.08f,.11f,.15f,1);
            Component menu = fixture.AddInactive("Welcome preview", "RobotWelcomeMenu");
            menu.GetType().GetField("_appLogo", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(menu,
                UnityEditor.AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Branding/UnityROSML/UnityROSML-Logo-Transparent.png"));
            Private(menu, "BuildMenu"); menu.gameObject.SetActive(true);
            menu.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            Canvas canvas = menu.GetComponentInChildren<Canvas>(); canvas.worldCamera = camera;
            Type textType = Runtime("TMPro.TMP_Text"), fontType = Runtime("TMPro.TMP_FontAsset");
            UnityEngine.Object font = Resources.Load("Fonts & Materials/LiberationSans SDF", fontType);
            foreach (Component text in canvas.GetComponentsInChildren(textType, true))
            {
                if (font != null) textType.GetProperty("font").SetValue(text, font);
                textType.GetMethod("ForceMeshUpdate").Invoke(text, new object[] { true, true });
                if ((bool)textType.GetProperty("isTextOverflowing").GetValue(text))
                    throw new InvalidOperationException("Welcome preview text overflow: " + text.name);
            }
            target = new RenderTexture(1040, 800, 24, RenderTextureFormat.ARGB32);
            pixels = new Texture2D(1040, 800, TextureFormat.RGB24, false);
            camera.targetTexture = target; Canvas.ForceUpdateCanvases(); camera.Render();
            RenderTexture.active = target; pixels.ReadPixels(new Rect(0,0,1040,800),0,0); pixels.Apply();
            string directory = Path.GetFullPath("WorkflowMenuPreviews"); Directory.CreateDirectory(directory);
            string output = Path.Combine(directory, "Welcome.png"); File.WriteAllBytes(output, pixels.EncodeToPNG());
            Debug.Log("RobotWelcomeMenuPreview: " + output);
        }
        finally
        {
            if (camera != null) camera.targetTexture = null;
            RenderTexture.active = previous;
            if (target != null) { target.Release(); UnityEngine.Object.DestroyImmediate(target); }
            if (pixels != null) UnityEngine.Object.DestroyImmediate(pixels);
            foreach (GameObject value in fixture._owned.AsEnumerable().Reverse()) if (value != null) UnityEngine.Object.DestroyImmediate(value);
            foreach (GameObject value in previousMainCameras) if (value != null) value.tag = "MainCamera";
        }
        FixtureTilingMenuTests.RenderDedicatedWorkflowPreviews();
    }

    private void RequireSnapshots() { if (_manifest == null) Assert.Ignore("Run the dedicated scene-workflow preparer for exact scene snapshots."); }
    private string SceneText(string name) => Resource(_manifest.scenes.Single(entry => entry.name == name).resource);
    private static string Resource(string path) { TextAsset value = Resources.Load<TextAsset>(path); Assert.That(value, Is.Not.Null, path); return value.text; }
    private string[] Components(string scene, string type) => Documents(scene).Where(block => block.Contains("m_Script: {fileID: 11500000, guid: " + _manifest.scripts.Single(entry => entry.name == type).guid)).ToArray();
    private static string[] Documents(string scene) => Regex.Split(scene, @"(?m)(?=^--- !u!)");
    private static int IntField(string block, string name) { Match value = Regex.Match(block, @"(?m)^  " + Regex.Escape(name) + @": (-?\d+)\s*$"); Assert.That(value.Success, Is.True, name); return int.Parse(value.Groups[1].Value); }
    private void AssertReference(string scene, string owner, string field, string expectedType)
    {
        Match reference = Regex.Match(owner, @"(?m)^  " + Regex.Escape(field) + @": \{fileID: (-?\d+)\}");
        Assert.That(reference.Success, Is.True, field); Assert.That(reference.Groups[1].Value, Is.Not.EqualTo("0"), field);
        string target = Components(scene, expectedType).Single(block => Regex.IsMatch(block, @"^--- !u!114 &" + reference.Groups[1].Value + @"(?:\s|$)"));
        Assert.That(target, Is.Not.Empty);
    }
    private static void AssertHash(string text, string expected) { using (SHA256 sha = SHA256.Create()) Assert.That(BitConverter.ToString(sha.ComputeHash(Encoding.UTF8.GetBytes(text))).Replace("-", "").ToLowerInvariant(), Is.EqualTo(expected)); }
    private GameObject New(string name) { GameObject value = new GameObject(name); value.SetActive(false); _owned.Add(value); return value; }
    private Component AddInactive(string name, string type) => New(name).AddComponent(Runtime(type));
    private static Type Runtime(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(assembly => assembly.GetType(name)).First(type => type != null);
    private static bool CanNavigate => (bool)Navigation.GetProperty("CanChangeScene").GetValue(null);
    private static bool Navigate(string target, Action<string> load) => (bool)Static("TryNavigate", target, load);
    private static object Static(string name, params object[] arguments) => Navigation.GetMethod(name, BindingFlags.Static | BindingFlags.NonPublic).Invoke(null, arguments);
    private static T Get<T>(object value, string name) => (T)value.GetType().GetProperty(name).GetValue(value);
    private static object Private(object value, string method, params object[] arguments) => value.GetType().GetMethod(method, BindingFlags.Instance | BindingFlags.NonPublic).Invoke(value, arguments);
    private static object Call(object value, string name, params object[] arguments) => value.GetType().GetMethods().Single(method => method.Name == name && method.GetParameters().Length == arguments.Length).Invoke(value, arguments);
}
