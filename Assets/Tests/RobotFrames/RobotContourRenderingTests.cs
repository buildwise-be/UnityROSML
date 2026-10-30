using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>Pixel checks through the installed renderer feature, rather than a substitute line renderer.</summary>
public sealed class RobotContourRenderingTests
{
    [TestCase("Mobile", false)]
    [TestCase("Mobile", true)]
    [TestCase("PC", false)]
    [TestCase("PC", true)]
    public void PrivateRobotDepthRemovesRearEdgesWithoutOccludingTheGhost(string renderer, bool compatibility)
    {
        if (SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null)
            Assert.Ignore("Pixel validation needs a graphics device; run RenderRobotContourPreviews without -nographics.");
        using (var fixture = new Fixture(renderer, compatibility)) fixture.Validate(null);
    }

    [TestCase("Mobile", false)]
    [TestCase("Mobile", true)]
    [TestCase("PC", false)]
    [TestCase("PC", true)]
    public void ContoursStayOnTheFilledRobotAsThePerspectiveCameraMoves(string renderer, bool compatibility)
    {
        if (SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null)
            Assert.Ignore("World-anchoring pixel validation needs a graphics device.");
        using (var fixture = new Fixture(renderer, compatibility)) fixture.ValidateWorldAnchoring();
    }

    [Test]
    public void DepthShaderUsesTheRenderTimeNativeViewInsteadOfPreparedMaterialMatrices()
    {
        if (SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null)
            Assert.Ignore("Render-time view validation needs a graphics device.");
        using (var fixture = new Fixture("Mobile", false)) fixture.ValidateRenderTimeView();
    }

    /// <summary>Also callable with Unity -executeMethod RobotContourRenderingTests.RenderRobotContourPreviews.</summary>
    public static void RenderRobotContourPreviews()
    {
        Assert.That(SystemInfo.graphicsDeviceType, Is.Not.EqualTo(GraphicsDeviceType.Null),
            "Omit -nographics when checking contour shaders and pixels.");
        string directory = Environment.GetEnvironmentVariable("ROBOT_CONTOUR_PREVIEW_DIR");
        if (string.IsNullOrEmpty(directory)) directory = Path.GetFullPath("RobotContourPreviews");
        Directory.CreateDirectory(directory);
        foreach (string renderer in new[] { "Mobile", "PC" })
            foreach (bool compatibility in new[] { false, true })
                using (var fixture = new Fixture(renderer, compatibility))
                {
                    fixture.Validate(directory);
                    fixture.ValidateWorldAnchoring();
                    fixture.RenderRobotPreviews(directory);
                }
        RobotContourMenuTests.RenderContourMenuPreviews();
        Debug.Log("RobotContourGraphicsValidation passed: hidden edges, visible edges, transparent interior, " +
            "unchanged ghost pixels/materials, execution opacity, both renderer assets and both render modes. Images: " + directory);
    }

    private sealed class Fixture : IDisposable
    {
        private const int Size = 512;
        private readonly string _renderer;
        private readonly RenderPipelineAsset _previousPipeline;
        private readonly RenderPipelineAsset _previousQualityPipeline;
        private readonly object _renderGraphSettings;
        private readonly PropertyInfo _compatibilityProperty;
        private readonly bool _previousCompatibility;
        private readonly List<UnityEngine.Object> _owned = new List<UnityEngine.Object>();
        private readonly GameObject _root;
        private readonly Camera _camera;
        private readonly RenderTexture _target;
        private readonly MeshRenderer _front;
        private readonly MeshRenderer _rear;
        private readonly MeshRenderer _ghost;
        private readonly Material _liveMaterial;
        private readonly Material _ghostMaterial;

        public Fixture(string renderer, bool compatibility)
        {
            _renderer = renderer + (compatibility ? "-Compatibility" : "-RenderGraph");
            _previousPipeline = GraphicsSettings.defaultRenderPipeline;
            _previousQualityPipeline = QualitySettings.renderPipeline;
            try
            {
                var pipeline = AssetDatabase.LoadAssetAtPath<RenderPipelineAsset>("Assets/Settings/" + renderer + "_RPAsset.asset");
                Assert.That(pipeline, Is.Not.Null, renderer + " pipeline is part of the fixture.");
                GraphicsSettings.defaultRenderPipeline = pipeline;
                QualitySettings.renderPipeline = pipeline;
                Type settingsType = PackageType("UnityEngine.Rendering.Universal.RenderGraphSettings");
                MethodInfo settingsGetter = Array.Find(typeof(GraphicsSettings).GetMethods(BindingFlags.Public | BindingFlags.Static),
                    method => method.Name == "GetRenderPipelineSettings" && method.IsGenericMethodDefinition && method.GetParameters().Length == 0);
                _renderGraphSettings = settingsGetter.MakeGenericMethod(settingsType).Invoke(null, null);
                Assert.That(_renderGraphSettings, Is.Not.Null, "URP global render graph settings must be registered.");
                _compatibilityProperty = settingsType.GetProperty("enableRenderCompatibilityMode");
                _previousCompatibility = (bool)_compatibilityProperty.GetValue(_renderGraphSettings);
                // This fixture changes memory only, restores it below, and never saves project settings.
                _compatibilityProperty.SetValue(_renderGraphSettings, compatibility);
                Contour("SetEnabled", false);
                Contour("SetLiveRobot", new object[] { null });
                _root = Own(new GameObject("Contour pixel fixture " + renderer));
                _root.layer = 31;
                Component rig = _root.AddComponent(RuntimeType("RosRobotRig"));
                rig.GetType().GetField("<Model>k__BackingField", BindingFlags.Instance | BindingFlags.NonPublic)
                    .SetValue(rig, RosRobotModelCatalog.Load().Find("none"));
                _liveMaterial = Own(new Material(Resources.Load<Material>("RobotTargetMarkers/Opaque")));
                _ghostMaterial = Own(new Material(Resources.Load<Material>("RobotTargetMarkers/Transparent")));
                SetColor(_liveMaterial, new Color(0.15f, 0.25f, 0.65f, 1f));
                SetColor(_ghostMaterial, new Color(1f, 0.1f, 0.05f, 0.72f));
                _front = Box("Front live link", Vector3.zero, new Vector3(1.6f, 1.6f, 0.3f), _liveMaterial);
                _rear = Box("Rear live link", new Vector3(0.1f, 0.05f, 0.6f), new Vector3(0.6f, 0.6f, 0.25f), _liveMaterial);
                _ghost = Box("Trajectory ghost (not registered)", new Vector3(0f, 0f, 0.9f),
                    new Vector3(0.4f, 0.4f, 0.1f), _ghostMaterial);
                var visuals = (IList)rig.GetType().GetField("_generatedVisuals", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(rig);
                visuals.Add(_front.gameObject);
                visuals.Add(_rear.gameObject);
                Contour("SetLiveRobot", rig);
                _camera = Own(new GameObject("Contour validation camera")).AddComponent<Camera>();
                _camera.transform.position = new Vector3(0f, 0f, -4f);
                _camera.transform.rotation = Quaternion.identity;
                _camera.orthographic = true;
                _camera.orthographicSize = 1.5f;
                _camera.nearClipPlane = 0.1f;
                _camera.farClipPlane = 20f;
                _camera.clearFlags = CameraClearFlags.SolidColor;
                _camera.backgroundColor = Color.black;
                _camera.cullingMask = 1 << 31;
                _camera.allowHDR = false;
                _camera.allowMSAA = false;
                _camera.enabled = false;
                _target = Own(new RenderTexture(Size, Size, 24, RenderTextureFormat.ARGB32));
                _target.Create();
                _camera.targetTexture = _target;
                // Initialize the pipeline, feature materials and availability before measuring pixels.
                _camera.Render();
                _camera.Render();
                Assert.That((bool)PackageType("UnityEngine.Rendering.Universal.UniversalRenderPipeline")
                    .GetField("useRenderGraph", BindingFlags.Static | BindingFlags.NonPublic).GetValue(null), Is.EqualTo(!compatibility),
                    "The graphics checks must execute the requested renderer path.");
                foreach (string shaderName in new[] { "RobotContourDepthNormals", "RobotContourComposite" })
                {
                    Shader shader = AssetDatabase.LoadAssetAtPath<Shader>("Assets/Resources/RobotContours/" + shaderName + ".shader");
                    Assert.That(shader, Is.Not.Null);
                    Assert.That(shader.isSupported, Is.True, shader.name);
                    foreach (var message in ShaderUtil.GetShaderMessages(shader))
                        Assert.That(message.severity.ToString(), Is.Not.EqualTo("Error"), shader.name + ": " + message.message);
                }
            }
            catch { Dispose(); throw; }
        }

        public void RenderRobotPreviews(string directory)
        {
            _camera.orthographic = true;
            Contour("SetEnabled", false);
            Contour("SetLiveRobot", new object[] { null });
            _root.SetActive(false);
            var previewRoot = Own(new GameObject("Actual UR16e contour preview"));
            previewRoot.SetActive(false);
            Component live = PreviewRig(previewRoot.transform, false);
            Component ghost = PreviewRig(previewRoot.transform, true);
            RosRobotModel model = RosRobotModelCatalog.Load().Find("kenos_sl");
            RosRobotGoldenCase pose = model.goldenCases[2];
            live.GetType().GetMethod("SetJointPositions").Invoke(live, new object[] { pose.jointNames, pose.jointPositions });
            double[] ghostJoints = (double[])pose.jointPositions.Clone();
            ghostJoints[0] += 0.45;
            ghostJoints[2] += 0.2;
            ghost.GetType().GetMethod("SetJointPositions").Invoke(ghost, new object[] { pose.jointNames, ghostJoints });
            previewRoot.SetActive(true);
            Physics.SyncTransforms();
            Bounds bounds = VisualBounds(live);
            bounds.Encapsulate(VisualBounds(ghost));
            float radius = Mathf.Max(0.25f, bounds.extents.magnitude);
            _camera.transform.position = bounds.center + new Vector3(1.5f, 0.9f, -2f).normalized * radius * 3f;
            _camera.transform.LookAt(bounds.center);
            _camera.orthographicSize = radius * 1.08f;
            _camera.backgroundColor = new Color(0.025f, 0.035f, 0.055f, 1f);
            var light = Own(new GameObject("UR16e preview light")).AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.4f;
            light.transform.rotation = Quaternion.Euler(35f, -40f, 0f);
            ghost.gameObject.SetActive(false);
            Contour("SetLiveRobot", live);
            Save(directory, "ur16e-normal", Capture());
            Contour("SetEnabled", true);
            Save(directory, "ur16e-contours", Capture());
            ghost.gameObject.SetActive(true);
            Save(directory, "ur16e-contours-and-path-ghost", Capture());
            Contour("SetEnabled", false);
            Contour("SetLiveRobot", new object[] { null });
        }

        private Component PreviewRig(Transform parent, bool ghost)
        {
            string path = "Assets/Prefabs/ur16e_kenos/ur16e_kenos" + (ghost ? "_transparent" : "") + ".prefab";
            var robot = UnityEngine.Object.Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>(path), parent);
            robot.SetActive(false);
            Component rig = robot.GetComponent(RuntimeType("RosRobotRig")) ?? robot.AddComponent(RuntimeType("RosRobotRig"));
            rig.GetType().GetProperty("IsGhost").SetValue(rig, ghost);
            Assert.That(rig.GetType().GetMethod("ApplyModel").Invoke(rig,
                new object[] { RosRobotModelCatalog.Load().Find("kenos_sl") }), Is.EqualTo(true));
            var generated = new HashSet<Renderer>();
            foreach (GameObject visual in (IEnumerable)rig.GetType().GetProperty("VisualRoots").GetValue(rig))
                foreach (Renderer renderer in visual.GetComponentsInChildren<Renderer>(true)) generated.Add(renderer);
            foreach (Renderer renderer in robot.GetComponentsInChildren<Renderer>(true))
                if (!generated.Contains(renderer)) renderer.enabled = false;
            foreach (Transform child in robot.GetComponentsInChildren<Transform>(true)) child.gameObject.layer = 31;
            rig.GetType().GetMethod("SetBaseWorldPose").Invoke(rig, new object[] { new Pose(Vector3.zero, Quaternion.identity) });
            robot.SetActive(true);
            return rig;
        }

        public void ValidateWorldAnchoring()
        {
            _camera.orthographic = false;
            _camera.fieldOfView = 55f;
            _front.transform.localPosition = new Vector3(0.3f, 0.15f, 0.05f);
            _front.transform.localScale = new Vector3(0.7f, 0.55f, 0.5f);
            _front.transform.localRotation = Quaternion.Euler(12f, 29f, 11f);
            _rear.enabled = false;
            _ghost.enabled = false;
            var positions = new[] { new Vector3(0f, 0f, -4f), new Vector3(0.42f, 0.33f, -3.4f), new Vector3(-0.4f, 0.24f, -3.6f) };
            var rotations = new[] { Quaternion.identity, Quaternion.Euler(6f, 9f, 3f), Quaternion.Euler(-5f, -7f, -2f) };
            for (int pose = 0; pose < positions.Length; pose++)
            {
                _camera.transform.SetPositionAndRotation(positions[pose], rotations[pose]);
                Contour("SetEnabled", false);
                Color32[] filled = Capture();
                Contour("SetEnabled", true);
                Color32[] contours = Capture();
                RectInt solidBounds = PixelBounds(filled, false), contourBounds = PixelBounds(contours, true);
                Assert.That(solidBounds.width * solidBounds.height, Is.GreaterThan(500));
                Assert.That(CyanPixels(contours), Is.GreaterThan(100));
                Assert.That(Math.Abs(solidBounds.xMin - contourBounds.xMin), Is.LessThanOrEqualTo(2));
                Assert.That(Math.Abs(solidBounds.xMax - contourBounds.xMax), Is.LessThanOrEqualTo(2));
                Assert.That(Math.Abs(solidBounds.yMin - contourBounds.yMin), Is.LessThanOrEqualTo(2),
                    "Contour Y must match the normal robot, rather than reflecting about the headset's screen centre.");
                Assert.That(Math.Abs(solidBounds.yMax - contourBounds.yMax), Is.LessThanOrEqualTo(2));
            }
            // XR can update native view/VP globals after the CPU has prepared a
            // render pass. Deliberately stale legacy material arrays must not win.
            ValidateLateNativeView(positions[0], rotations[0], positions[1], rotations[1]);
            UnityEngine.Object feature = CurrentFeature();
            object pass = feature.GetType().GetField("_pass", BindingFlags.Instance | BindingFlags.NonPublic).GetValue(feature);
            Assert.That((bool)pass.GetType().GetProperty("requiresIntermediateTexture").GetValue(pass), Is.True,
                "A camera texture keeps private and scene projections in the same orientation; URP performs the final XR/backbuffer flip.");
            Contour("SetEnabled", false);
            Debug.Log("RobotContourWorldAnchoring " + _renderer + ": perspectiveCameraPoses=3, filledContourBoundsMatch=true, lateNativeViewUpdate=true");
        }

        public void ValidateRenderTimeView()
        {
            _camera.orthographic = false;
            _camera.fieldOfView = 55f;
            ValidateLateNativeView(new Vector3(0f, 0f, -4f), Quaternion.identity,
                new Vector3(0.42f, 0.33f, -3.4f), Quaternion.Euler(6f, 9f, 3f));
        }

        private void ValidateLateNativeView(Vector3 preparedPosition, Quaternion preparedRotation, Vector3 trackedPosition, Quaternion trackedRotation)
        {
            var material = Own(new Material(Resources.Load<Material>("RobotContours/DepthNormalsMaterial")));
            material.SetFloat("_ContourDepthRange", 20f);
            _camera.transform.SetPositionAndRotation(preparedPosition, preparedRotation);
            Matrix4x4 preparedView = _camera.worldToCameraMatrix;
            Matrix4x4 projection = _camera.projectionMatrix;
            Matrix4x4 preparedVP = GL.GetGPUProjectionMatrix(projection, true) * preparedView;
            material.SetMatrixArray("_RobotContourView", new[] { preparedView, preparedView });
            material.SetMatrixArray("_RobotContourViewProjection", new[] { preparedVP, preparedVP });
            Color32[] prepared = DrawPrivateWithNativeView(material, preparedView, projection);
            _camera.transform.SetPositionAndRotation(trackedPosition, trackedRotation);
            Matrix4x4 trackedView = _camera.worldToCameraMatrix;
            Color32[] tracked = DrawPrivateWithNativeView(material, trackedView, projection);
            Assert.That(RegionDifference(prepared, tracked, 0, 0, Size, Size), Is.GreaterThan(200),
                "Updating native camera matrices after CPU pass preparation must move the projected robot. Stale material matrices produce a head-relative image.");
        }

        private Color32[] DrawPrivateWithNativeView(Material material, Matrix4x4 view, Matrix4x4 projection)
        {
            var command = new CommandBuffer { name = "Simulated render-time tracked view update" };
            try
            {
                command.SetRenderTarget(_target);
                command.SetViewport(new Rect(0f, 0f, Size, Size));
                command.ClearRenderTarget(true, true, Color.clear);
                command.SetViewProjectionMatrices(view, projection);
                command.DrawMesh(_front.GetComponent<MeshFilter>().sharedMesh, _front.localToWorldMatrix, material, 0, 0);
                Graphics.ExecuteCommandBuffer(command);
                return ReadTarget();
            }
            finally { command.Release(); }
        }

        private static RectInt PixelBounds(Color32[] pixels, bool cyan)
        {
            int minX = Size, minY = Size, maxX = -1, maxY = -1;
            for (int y = 0; y < Size; y++) for (int x = 0; x < Size; x++)
            {
                Color32 pixel = pixels[y * Size + x];
                if (cyan ? !IsCyan(pixel) : pixel.r + pixel.g + pixel.b < 15) continue;
                minX = Math.Min(minX, x); minY = Math.Min(minY, y);
                maxX = Math.Max(maxX, x); maxY = Math.Max(maxY, y);
            }
            return maxX < 0 ? new RectInt() : new RectInt(minX, minY, maxX - minX + 1, maxY - minY + 1);
        }

        private static Bounds VisualBounds(Component rig)
        {
            bool found = false;
            Bounds bounds = default;
            foreach (GameObject visual in (IEnumerable)rig.GetType().GetProperty("VisualRoots").GetValue(rig))
                foreach (Renderer renderer in visual.GetComponentsInChildren<Renderer>())
                {
                    if (!renderer.enabled) continue;
                    if (!found) { bounds = renderer.bounds; found = true; }
                    else bounds.Encapsulate(renderer.bounds);
                }
            Assert.That(found, Is.True);
            return bounds;
        }

        public void Validate(string directory)
        {
            _front.enabled = false;
            _rear.enabled = false;
            Color32[] ghostBaseline = Capture();
            _front.enabled = true;
            Contour("SetEnabled", true);
            Color32[] frontOnly = Capture();
            Assert.That((bool)RuntimeType("RobotContourDisplay").GetProperty("IsRenderingAvailable").GetValue(null), Is.True);
            Assert.That(_front.forceRenderingOff, Is.True, "Normal fill must be suppressed while the feature draws contours.");
            Assert.That(CyanPixels(frontOnly), Is.GreaterThan(500), "The installed feature must draw a visible silhouette.");
            Assert.That(RegionDifference(frontOnly, ghostBaseline, 245, 245, 22, 22), Is.EqualTo(0),
                "An unchanged trajectory ghost behind the live robot must remain visible through its unfilled interior.");
            Assert.That(frontOnly[256 * Size + 180].r + frontOnly[256 * Size + 180].g + frontOnly[256 * Size + 180].b,
                Is.LessThan(8), "The contour representation must leave the robot interior unfilled.");

            _rear.enabled = true;
            Color32[] hiddenRear = Capture();
            Assert.That(RegionDifference(frontOnly, hiddenRear, 0, 0, Size, Size), Is.EqualTo(0),
                "A rear link fully hidden by the front live link must contribute no lines.");
            _rear.transform.localPosition = new Vector3(0.95f, 0.2f, 0.6f);
            Color32[] visibleRear = Capture();
            Assert.That(RegionDifference(frontOnly, visibleRear, 0, 0, Size, Size), Is.GreaterThan(200),
                "Moving the same rear link into view must reveal its visible contours.");

            Assert.That(_front.sharedMaterial, Is.SameAs(_liveMaterial));
            Assert.That(_rear.sharedMaterial, Is.SameAs(_liveMaterial));
            Assert.That(_ghost.sharedMaterial, Is.SameAs(_ghostMaterial));
            Assert.That(_ghost.enabled && !_ghost.forceRenderingOff, Is.True);
            Assert.That(_ghostMaterial.GetColor("_BaseColor").a, Is.EqualTo(0.72f).Within(0.0001f));
            ValidateVisibleCrease();
            Color32[] fallback = ValidateDisabledFeatureFallback();
            RuntimeType("RobotExecutionState").GetMethod("SetBusy").Invoke(null, new object[] { _camera, true });
            Color32[] executing = Capture();
            Assert.That(CyanPixels(executing), Is.LessThan(CyanPixels(visibleRear) / 4),
                "Execution should reduce contour opacity to the live-robot opacity limit.");
            RuntimeType("RobotExecutionState").GetMethod("SetBusy").Invoke(null, new object[] { _camera, false });
            Contour("SetEnabled", false);
            Capture();
            Assert.That(_front.forceRenderingOff || _rear.forceRenderingOff, Is.False);
            Assert.That(_front.sharedMaterial, Is.SameAs(_liveMaterial));
            Assert.That(_ghost.sharedMaterial, Is.SameAs(_ghostMaterial));
            if (directory != null)
            {
                Save(directory, "hidden-rear", hiddenRear);
                Save(directory, "visible-rear", visibleRear);
                Save(directory, "execution", executing);
                Save(directory, "disabled-feature-fallback", fallback);
            }
            Debug.Log("RobotContourPixels " + _renderer + ": silhouettePixels=" + CyanPixels(frontOnly) +
                ", hiddenRearPixelDifference=0, ghostInteriorPixelDifference=0, revealedRearChangedPixels=" +
                RegionDifference(frontOnly, visibleRear, 0, 0, Size, Size));
        }

        private void ValidateVisibleCrease()
        {
            Quaternion rotation = _front.transform.localRotation;
            bool rearEnabled = _rear.enabled, ghostEnabled = _ghost.enabled;
            try
            {
                _rear.enabled = false;
                _ghost.enabled = false;
                _front.transform.localRotation = Quaternion.Euler(0f, 35f, 0f);
                Color32[] image = Capture();
                Vector3 crease = _camera.WorldToScreenPoint(_front.transform.TransformPoint(new Vector3(0.5f, 0f, -0.5f)));
                bool found = false;
                for (int y = (int)crease.y - 4; y <= (int)crease.y + 4; y++)
                    for (int x = (int)crease.x - 4; x <= (int)crease.x + 4; x++)
                        if (x >= 0 && y >= 0 && x < Size && y < Size && IsCyan(image[y * Size + x])) found = true;
                Assert.That(found, Is.True, "A visible 90-degree crease inside the silhouette must survive the CAD smoothing threshold.");
            }
            finally
            {
                _front.transform.localRotation = rotation;
                _rear.enabled = rearEnabled;
                _ghost.enabled = ghostEnabled;
            }
        }

        private Color32[] ValidateDisabledFeatureFallback()
        {
            UnityEngine.Object feature = CurrentFeature();
            Assert.That(feature, Is.Not.Null);
            MethodInfo setActive = feature.GetType().GetMethod("SetActive");
            bool previousActive = (bool)feature.GetType().GetProperty("isActive").GetValue(feature);
            var unrelated = Own(new GameObject("Unrelated available contour provider"));
            Contour("SetRenderingAvailable", unrelated, true);
            try
            {
                setActive.Invoke(feature, new object[] { false });
                Color32[] fallback = Capture();
                Color32 interior = fallback[256 * Size + 180];
                Assert.That(interior.r + interior.g + interior.b, Is.GreaterThan(60),
                    "A disabled camera feature must restore normal robot fill before culling, even with an unrelated available provider.");
                Assert.That(CyanPixels(fallback), Is.LessThan(50), "No contour lines should be drawn by a disabled feature.");
                return fallback;
            }
            finally
            {
                setActive.Invoke(feature, new object[] { previousActive });
                Contour("SetRenderingAvailable", unrelated, false);
            }
        }

        private UnityEngine.Object CurrentFeature()
        {
            string rendererName = _renderer.StartsWith("Mobile") ? "Mobile" : "PC";
            return Array.Find(AssetDatabase.LoadAllAssetsAtPath("Assets/Settings/" + rendererName + "_Renderer.asset"),
                asset => asset != null && asset.GetType().Name == "RobotContourRendererFeature");
        }

        private Color32[] Capture()
        {
            _camera.Render();
            return ReadTarget();
        }

        private Color32[] ReadTarget()
        {
            RenderTexture previous = RenderTexture.active;
            var texture = new Texture2D(Size, Size, TextureFormat.RGBA32, false);
            try
            {
                RenderTexture.active = _target;
                texture.ReadPixels(new Rect(0f, 0f, Size, Size), 0, 0);
                texture.Apply();
                return texture.GetPixels32();
            }
            finally { RenderTexture.active = previous; UnityEngine.Object.DestroyImmediate(texture); }
        }

        private void Save(string directory, string state, Color32[] pixels)
        {
            var texture = new Texture2D(Size, Size, TextureFormat.RGBA32, false);
            try
            {
                texture.SetPixels32(pixels); texture.Apply();
                File.WriteAllBytes(Path.Combine(directory, _renderer + "-" + state + ".png"), texture.EncodeToPNG());
            }
            finally { UnityEngine.Object.DestroyImmediate(texture); }
        }

        private MeshRenderer Box(string name, Vector3 position, Vector3 scale, Material material)
        {
            var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = name; box.layer = 31;
            box.transform.SetParent(_root.transform, false);
            box.transform.localPosition = position; box.transform.localScale = scale;
            UnityEngine.Object.DestroyImmediate(box.GetComponent<Collider>());
            var renderer = box.GetComponent<MeshRenderer>();
            renderer.sharedMaterial = material;
            renderer.shadowCastingMode = ShadowCastingMode.Off; renderer.receiveShadows = false;
            return renderer;
        }

        private T Own<T>(T value) where T : UnityEngine.Object { _owned.Add(value); return value; }
        public void Dispose()
        {
            if (_camera != null) RuntimeType("RobotExecutionState").GetMethod("SetBusy")
                .Invoke(null, new object[] { _camera, false });
            Contour("SetEnabled", false);
            Contour("SetLiveRobot", new object[] { null });
            if (_camera != null) _camera.targetTexture = null;
            for (int i = _owned.Count - 1; i >= 0; i--) if (_owned[i] != null) UnityEngine.Object.DestroyImmediate(_owned[i]);
            if (_renderGraphSettings != null && _compatibilityProperty != null)
                _compatibilityProperty.SetValue(_renderGraphSettings, _previousCompatibility);
            GraphicsSettings.defaultRenderPipeline = _previousPipeline;
            QualitySettings.renderPipeline = _previousQualityPipeline;
        }
    }

    private static int CyanPixels(Color32[] image)
    {
        int count = 0;
        foreach (Color32 pixel in image)
            if (IsCyan(pixel)) count++;
        return count;
    }
    private static bool IsCyan(Color32 pixel) => pixel.g > 90 && pixel.b > 100 && pixel.g > pixel.b * 0.8f && pixel.b > pixel.r * 1.15f;
    private static int RegionDifference(Color32[] a, Color32[] b, int x, int y, int width, int height)
    {
        int count = 0;
        for (int py = y; py < y + height; py++) for (int px = x; px < x + width; px++)
        {
            int index = py * 512 + px;
            if (Math.Abs(a[index].r - b[index].r) + Math.Abs(a[index].g - b[index].g) + Math.Abs(a[index].b - b[index].b) > 3) count++;
        }
        return count;
    }
    private static void SetColor(Material material, Color value)
    {
        if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", value);
        if (material.HasProperty("_Color")) material.SetColor("_Color", value);
    }
    private static object Contour(string name, params object[] values) => RuntimeType("RobotContourDisplay").GetMethod(name).Invoke(null, values);
    private static Type RuntimeType(string name) => Type.GetType(name + ", Assembly-CSharp", true);
    private static Type PackageType(string name)
    {
        foreach (Assembly assembly in AppDomain.CurrentDomain.GetAssemblies())
        {
            Type type = assembly.GetType(name);
            if (type != null) return type;
        }
        throw new TypeLoadException(name);
    }
}
