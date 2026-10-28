using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>Runs the actual calibrated arm/tool preview without Start, XR hardware or ROS transport.</summary>
public sealed class TrajectoryPlaybackTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private Component _visualizer, _live, _coordinates;
    private RosRobotModel _model;
    private string[] _names;
    private readonly double[] _initial = { 0.2, -1.1, 1.2, -0.9, 1.3, 0.2 };
    private readonly Pose _base = new Pose(new Vector3(0.4f, 0.2f, -0.3f), Quaternion.Euler(0f, 27f, 0f));

    [SetUp]
    public void CreateTheActualModelAndDisabledVisualizer()
    {
        _coordinates = Add("Playback-test coordinates", "RobotCoordinateService");
        Add("Playback-test robot state", "RobotStateService");
        _model = RosRobotModelCatalog.Load().Find("kenos_sl");
        _names = _model.links.Where(RosRobotModelMath.IsRevolute).Select(link => link.jointName).ToArray();
        Assert.That(_names.Length, Is.EqualTo(_initial.Length));
        GameObject robot = UnityEngine.Object.Instantiate(Prefab(false));
        _owned.Add(robot);
        robot.SetActive(false);
        _live = robot.AddComponent(TypeOf("RosRobotRig"));
        Assert.That(Call(_live, "ApplyModel", _model), Is.EqualTo(true));
        Assert.That(Call(_live, "SetJointPositions", _names, _initial), Is.EqualTo(true));
        Call(_live, "SetBaseWorldPose", _base);
        Transform baseLink = Get<Transform>(_live, "BaseLink");
        Call(_coordinates, "Initialize", baseLink.GetComponent<ArticulationBody>(), New("Playback tracking").transform);
        _visualizer = Add("Playback-test visualizer", "UR16eTrajectoryVisualizer");
        Set(_visualizer, "virtualUr16ePrefab", Prefab(true));
        Private(_visualizer, "RecordRobot", robot, Pose.identity);
    }

    [TearDown]
    public void Cleanup()
    {
        if (_visualizer != null) Call(_visualizer, "SetTrajectory", new object[] { null });
        for (int index = _owned.Count - 1; index >= 0; index--)
            if (_owned[index] != null) UnityEngine.Object.DestroyImmediate(_owned[index]);
        _owned.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(TypeOf("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty,
            "Virtual playback must not create a ROS connection or publish robot motion.");
    }

    [Test]
    public void NonuniformTimingInterpolatesEveryJointAndTheSelectedToolTcpWithoutMovingTheLiveRig()
    {
        double[] middle = Offset(0.4), last = Offset(1.0);
        object trajectory = Trajectory(new[] { 0d, 1d, 4d }, _initial, middle, last);
        Call(_visualizer, "SetTrajectory", trajectory);
        Assert.That(Get<bool>(_visualizer, "HasPlayback"), Is.True);
        Assert.That(Get<float>(_visualizer, "DurationSeconds"), Is.EqualTo(4).Within(0.000001));
        Call(_visualizer, "Seek", 0.625f); // t=2.5, halfway through the three-second segment.
        AssertPreviewJoints(Offset(0.7));
        AssertPreviewTcp(Offset(0.7));
        AssertLiveJoints();
        AssertPose(PoseOf(Get<Transform>(_live, "BaseLink")), _base);
        Assert.That(Get<bool>(_visualizer, "IsShowingTrajectory", trajectory), Is.True);
    }

    [Test]
    public void JointInterpolationPreservesRosUnwrappedAnglesAcrossPi()
    {
        double[] start = (double[])_initial.Clone(), end = (double[])_initial.Clone();
        start[0] = 3.0;
        end[0] = -3.0;
        Call(_visualizer, "SetTrajectory", Trajectory(new[] { 0d, 2d }, start, end));
        Call(_visualizer, "Seek", 0.5f);
        double[] expected = (double[])_initial.Clone();
        expected[0] = 0d; // ROS requested the long sweep; shortest-arc quaternion interpolation would give pi.
        AssertPreviewJoints(expected);
        AssertPreviewTcp(expected);
    }

    [TestCase("Play")]
    [TestCase("Replay")]
    public void AnimatedRobotStaysHiddenUntilPlaybackAndEachNewPlanStartsHidden(string action)
    {
        object trajectory = Trajectory(new[] { 0d, 4d }, _initial, Offset(0.8));
        Call(_visualizer, "SetTrajectory", trajectory);
        AssertAnimatedRobotVisible(false);
        Assert.That(Get<bool>(_visualizer, "HasPlayback"), Is.True, "Play must be available while its robot is hidden.");
        Assert.That(Get<bool>(_visualizer, "IsShowingTrajectory", trajectory), Is.True,
            "The existing static waypoint preview still permits reviewing the plan.");

        Call(_visualizer, "Seek", 0.5f);
        Call(_visualizer, "SetPlaybackSpeed", 2f);
        Call(_visualizer, "UpdatePercentageToShow", 1f);
        Private(_visualizer, "RedistributeClones");
        Call(_visualizer, "SetTrajectory", trajectory);
        AssertAnimatedRobotVisible(false);
        AssertPreviewJoints(Offset(0.4));

        Call(_visualizer, action);
        AssertAnimatedRobotVisible(true);
        Assert.That(Get<bool>(_visualizer, "IsPlaying"), Is.True);
        Advance(0.5f);
        Call(_visualizer, "Pause");
        Call(_visualizer, "Seek", 0.75f);
        AssertAnimatedRobotVisible(true);
        AssertPreviewJoints(Offset(0.6));
        Assert.That(Get<bool>(_visualizer, "IsPlaying"), Is.False);

        Component preview = PlaybackRig;
        Call(_visualizer, "SetTrajectory", trajectory);
        Assert.That(PlaybackRig, Is.EqualTo(preview));
        AssertAnimatedRobotVisible(true);

        Call(_visualizer, "SetTrajectory", Trajectory(new[] { 0d, 2d }, _initial, Offset(0.3)));
        AssertAnimatedRobotVisible(false);
        AssertTime(0d);
        Assert.That(Get<bool>(_visualizer, "IsPlaying"), Is.False);
        AssertLiveJoints();
    }

    [Test]
    public void PauseSpeedSeekEndAndReplayAffectOnlyVirtualPlaybackAndPreserveTheReviewedTrajectory()
    {
        object trajectory = Trajectory(new[] { 0d, 4d }, _initial, Offset(0.8));
        Call(_visualizer, "SetTrajectory", trajectory);
        Call(_visualizer, "Replay");
        Call(_visualizer, "SetPlaybackSpeed", 2f);
        Advance(0.5f);
        AssertTime(1d);
        Call(_visualizer, "Pause");
        Advance(2f);
        AssertTime(1d);
        Call(_visualizer, "Seek", 0.75f);
        AssertTime(3d);
        Assert.That(Get<bool>(_visualizer, "IsPlaying"), Is.False);
        Call(_visualizer, "Play");
        Advance(1f);
        AssertTime(4d);
        Assert.That(Get<bool>(_visualizer, "IsPlaying"), Is.False);
        AssertPreviewJoints(Offset(0.8));
        Call(_visualizer, "Replay");
        AssertTime(0d);
        Assert.That(Get<bool>(_visualizer, "IsPlaying"), Is.True);
        Assert.That(Get<bool>(_visualizer, "IsShowingTrajectory", trajectory), Is.True);
        AssertLiveJoints();
    }

    [Test]
    public void HoldsAndDuplicateTimestampsHaveDeterministicSampling()
    {
        Call(_visualizer, "SetTrajectory", Trajectory(new[] { 0d, 1d, 1d, 3d, 4d },
            _initial, Offset(0.4), Offset(0.4), Offset(0.4), Offset(0.8)));
        foreach (float normalized in new[] { 0.25f, 0.5f, 0.75f })
        {
            Call(_visualizer, "Seek", normalized);
            AssertPreviewJoints(Offset(0.4));
        }
        Call(_visualizer, "Seek", 0.875f);
        AssertPreviewJoints(Offset(0.6));
    }

    [Test]
    public void AStaticOnePointPlanRetainsItsVisibleReviewWithoutPlaying()
    {
        object trajectory = Trajectory(new[] { 0d }, Offset(0.5));
        Call(_visualizer, "SetTrajectory", trajectory);
        Assert.That(Get<bool>(_visualizer, "HasVisiblePreview"), Is.True);
        Assert.That(Get<bool>(_visualizer, "HasPlayback"), Is.True);
        Assert.That(Get<float>(_visualizer, "DurationSeconds"), Is.Zero);
        AssertAnimatedRobotVisible(false);
        Call(_visualizer, "Play");
        AssertAnimatedRobotVisible(true);
        Advance(10f);
        Assert.That(Get<bool>(_visualizer, "IsPlaying"), Is.False);
        AssertPreviewJoints(Offset(0.5));
        Assert.That(Get<bool>(_visualizer, "IsShowingTrajectory", trajectory), Is.True);
    }

    [TestCase("negative time")]
    [TestCase("decreasing time")]
    [TestCase("noncanonical nanoseconds")]
    [TestCase("nonfinite position")]
    [TestCase("unknown joint")]
    [TestCase("instantaneous jump")]
    public void InvalidTrajectoryCannotProduceAnExecutableLookingArmPreview(string invalid)
    {
        object trajectory = Trajectory(new[] { 0d, 2d }, _initial, Offset(0.5));
        object joints = Field(trajectory, "joint_trajectory").GetValue(trajectory);
        Array points = (Array)Field(joints, "points").GetValue(joints);
        if (invalid == "negative time") Set(Field(points.GetValue(0), "time_from_start").GetValue(points.GetValue(0)), "sec", -1);
        else if (invalid == "decreasing time")
        {
            Set(Field(points.GetValue(0), "time_from_start").GetValue(points.GetValue(0)), "sec", 3);
        }
        else if (invalid == "noncanonical nanoseconds")
            Set(Field(points.GetValue(1), "time_from_start").GetValue(points.GetValue(1)), "nanosec", 1000000000u);
        else if (invalid == "nonfinite position") ((double[])Field(points.GetValue(1), "positions").GetValue(points.GetValue(1)))[2] = double.NaN;
        else if (invalid == "unknown joint") Set(joints, "joint_names", new[] { "unsupported_joint" }.Concat(_names.Skip(1)).ToArray());
        else Set(Field(points.GetValue(1), "time_from_start").GetValue(points.GetValue(1)), "sec", 0);
        Call(_visualizer, "SetTrajectory", trajectory);
        Assert.That(Get<bool>(_visualizer, "HasPlayback") || Get<bool>(_visualizer, "HasVisiblePreview"), Is.False);
        Assert.That(Field(_visualizer, "_playbackRig").GetValue(_visualizer), Is.Null);
        AssertLiveJoints();
    }

    [TestCase("clear")]
    [TestCase("model")]
    [TestCase("mode")]
    public void ClearingOrChangingTheRobotModelStopsAndDisposesTheOwnedPreview(string reason)
    {
        Call(_visualizer, "SetTrajectory", Trajectory(new[] { 0d, 2d }, _initial, Offset(0.5)));
        Call(_visualizer, "Play");
        Component preview = PlaybackRig;
        if (reason == "model") Assert.That(Call(_live, "ApplyModel", RosRobotModelCatalog.Load().Find("none")), Is.EqualTo(true));
        else if (reason == "mode")
        {
            Component planner = Add("Playback-test TCP planner", "TcpPlanningController");
            Set(planner, "_visualizer", _visualizer);
            Component input = Add("Playback-test task input", "URInputManager");
            Set(input, "_tcpPlanningController", planner);
            Call(input, "UseTilePickPlace");
            Call(input, "UseTcpPreview");
        }
        else Call(_visualizer, "SetTrajectory", new object[] { null });
        Assert.That(Get<bool>(_visualizer, "HasPlayback") || Get<bool>(_visualizer, "IsPlaying"), Is.False);
        Assert.That(preview == null, Is.True, "The preview owns and disposes its separate cloned robot.");
        Assert.That(Field(_visualizer, "_playbackRig").GetValue(_visualizer), Is.Null);
    }

    [Test]
    public void SubsetTrajectoryHoldsTheRemainingLiveJointsAtTheirCapturedInitialValues()
    {
        object trajectory = Trajectory(new[] { 0d, 2d }, new[] { 0.2d }, new[] { 0.8d });
        Set(Field(trajectory, "joint_trajectory").GetValue(trajectory), "joint_names", new[] { _names[0] });
        Call(_visualizer, "SetTrajectory", trajectory);
        // Incoming live feedback must not change the already reviewed missing joints.
        Call(_live, "SetJointPositions", _names, Offset(0.3));
        Call(_visualizer, "Seek", 0.5f);
        double[] expected = (double[])_initial.Clone();
        expected[0] = 0.5d;
        AssertPreviewJoints(expected);
        AssertPreviewTcp(expected);
    }

    [Test]
    public void RecordedSuccessfulMtcPlanReplaysAllTimedSamplesWithTheKenosSlTcp()
    {
        object trajectory = ReadRecordedMtcTrajectory();
        object joints = Field(trajectory, "joint_trajectory").GetValue(trajectory);
        Array points = (Array)Field(joints, "points").GetValue(joints);
        string[] names = (string[])Field(joints, "joint_names").GetValue(joints);
        Assert.That(points.Length, Is.EqualTo(97));
        Call(_visualizer, "SetTrajectory", trajectory);
        Assert.That(Get<bool>(_visualizer, "HasPlayback"), Is.True);
        double duration = Get<float>(_visualizer, "DurationSeconds");
        Assert.That(duration, Is.EqualTo(8.569338209d).Within(0.000001));
        for (int index = 0; index < points.Length; index++)
        {
            object point = points.GetValue(index);
            double time = TimeOf(point);
            int expectedIndex = index;
            while (expectedIndex + 1 < points.Length && Math.Abs(TimeOf(points.GetValue(expectedIndex + 1)) - time) < 0.000000001)
                expectedIndex++;
            // Exercise the exact ROS boundary, including tiny duplicate-time IK seams.
            Set(_visualizer, "_playbackTime", time);
            Private(_visualizer, "ApplyPlaybackPose");
            double[] values = (double[])Field(points.GetValue(expectedIndex), "positions").GetValue(points.GetValue(expectedIndex));
            var expected = names.Select((name, joint) => new { name, value = values[joint] }).ToDictionary(pair => pair.name, pair => pair.value);
            AssertPreviewTcp(_names.Select(name => expected[name]).ToArray(), 0.0001f);
        }
        AssertLiveJoints();
        Assert.That(Get<bool>(_visualizer, "IsShowingTrajectory", trajectory), Is.True);
    }

    [Test]
    public void ReattachingTheSameReviewedResultKeepsPlaybackProgressAndItsOwnedArm()
    {
        object trajectory = Trajectory(new[] { 0d, 4d }, _initial, Offset(0.8));
        Call(_visualizer, "SetTrajectory", trajectory);
        Call(_visualizer, "Seek", 0.5f);
        Call(_visualizer, "Play");
        Component preview = PlaybackRig;
        Call(_visualizer, "SetTrajectory", trajectory);
        Assert.That(PlaybackRig, Is.EqualTo(preview));
        AssertTime(2d);
        Assert.That(Get<bool>(_visualizer, "IsPlaying"), Is.True);
        Assert.That(Get<bool>(_visualizer, "IsShowingTrajectory", trajectory), Is.True);
    }

    [Test]
    public void ActualMenuEnablesPlaybackControlsAfterPlanningAndScrubsOnlyThePreview()
    {
        Component menu = Add("Playback-test menu", "ToolSelectionMenu");
        Set(menu, "_trajectoryVisualizer", _visualizer);
        Private(menu, "BuildMenu");
        Canvas canvas = (Canvas)Field(menu, "_canvas").GetValue(menu);
        _owned.Add(canvas.gameObject);
        Private(menu, "RefreshPlayback");
        Component play = (Component)Field(menu, "_playbackPlayPauseButton").GetValue(menu);
        Component timeline = (Component)Field(menu, "_playbackTimeline").GetValue(menu);
        Assert.That(Get<bool>(play, "interactable") || Get<bool>(timeline, "interactable"), Is.False);
        Call(_visualizer, "SetTrajectory", Trajectory(new[] { 0d, 4d }, _initial, Offset(0.8)));
        Private(menu, "RefreshPlayback");
        Assert.That(Get<bool>(play, "interactable") && Get<bool>(timeline, "interactable"), Is.True);
        AssertAnimatedRobotVisible(false);
        object click = play.GetType().GetProperty("onClick").GetValue(play);
        Call(click, "Invoke");
        AssertAnimatedRobotVisible(true);
        Assert.That(Get<bool>(_visualizer, "IsPlaying"), Is.True);
        timeline.GetType().GetProperty("value").SetValue(timeline, 0.75f);
        AssertTime(3d);
        Assert.That(Get<bool>(_visualizer, "IsPlaying"), Is.False);
        AssertPreviewJoints(Offset(0.6));
        AssertLiveJoints();
        Component instantiator = Add("Playback-test calibration state", "URInstantiator");
        Set(menu, "_instantiator", instantiator);
        // Native calibration tests cover QR editing; here exercise the menu against its actual editing state.
        instantiator.GetType().GetProperty("IsCalibrating").GetSetMethod(true).Invoke(instantiator, new object[] { true });
        Private(menu, "RefreshPlayback");
        Assert.That(Get<bool>(play, "interactable") || Get<bool>(timeline, "interactable"), Is.False);
        instantiator.GetType().GetProperty("IsCalibrating").GetSetMethod(true).Invoke(instantiator, new object[] { false });
    }

    /// <summary>Render the real arm/tool preview at three recorded plan times using the mobile URP project.</summary>
    public static void RenderPlaybackPreview()
    {
        var fixture = new TrajectoryPlaybackTests();
        fixture.CreateTheActualModelAndDisabledVisualizer();
        try
        {
            Call(fixture._visualizer, "SetTrajectory", fixture.ReadRecordedMtcTrajectory());
            Call(fixture._visualizer, "Play");
            Call(fixture._visualizer, "Pause");
            Component preview = fixture.PlaybackRig;
            Assert.That(preview, Is.Not.Null);
            foreach (Component staticGhost in (IList)Field(fixture._visualizer, "_clones").GetValue(fixture._visualizer))
                staticGhost.gameObject.SetActive(false);
            GameObject cameraOwner = fixture.New("Playback preview camera");
            Camera camera = cameraOwner.AddComponent<Camera>();
            cameraOwner.SetActive(true);
            camera.backgroundColor = new Color(0.055f, 0.075f, 0.095f);
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.nearClipPlane = 0.01f;
            camera.farClipPlane = 20f;
            camera.fieldOfView = 48f;
            cameraOwner.transform.position = fixture._base.position + new Vector3(2f, 1.4f, -2.2f);
            cameraOwner.transform.LookAt(fixture._base.position + new Vector3(0f, 0.5f, 0f));
            GameObject lightOwner = fixture.New("Playback preview light");
            Light light = lightOwner.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 2f;
            light.transform.rotation = Quaternion.Euler(40f, -25f, 0f);
            lightOwner.SetActive(true);
            RenderTexture target = new RenderTexture(1200, 1000, 24);
            Texture2D pixels = new Texture2D(1200, 1000, TextureFormat.RGB24, false);
            RenderTexture previous = RenderTexture.active;
            try
            {
                target.Create();
                camera.targetTexture = target;
                foreach (int percent in new[] { 0, 50, 100 })
                {
                    Call(fixture._visualizer, "Seek", percent / 100f);
                    camera.Render();
                    RenderTexture.active = target;
                    pixels.ReadPixels(new Rect(0, 0, target.width, target.height), 0, 0);
                    pixels.Apply();
                    File.WriteAllBytes(Path.Combine(Directory.GetParent(Application.dataPath).FullName,
                        "arm-playback-" + percent + ".png"), pixels.EncodeToPNG());
                }
                Debug.Log("ActualArmToolPlaybackPreview: rendered 0, 50 and 100 percent of the recorded 97-point MTC plan.");
            }
            finally
            {
                camera.targetTexture = null;
                RenderTexture.active = previous;
                UnityEngine.Object.DestroyImmediate(pixels);
                target.Release();
                UnityEngine.Object.DestroyImmediate(target);
            }
        }
        finally { fixture.Cleanup(); }
    }

    private Component PlaybackRig => (Component)Field(_visualizer, "_playbackRig").GetValue(_visualizer);
    private void AssertAnimatedRobotVisible(bool visible)
    {
        Assert.That(PlaybackRig, Is.Not.Null);
        Assert.That(PlaybackRig.gameObject.activeSelf, Is.EqualTo(visible));
        Renderer[] renderers = PlaybackRig.GetComponentsInChildren<Renderer>(true);
        Assert.That(renderers, Is.Not.Empty, "Check the real arm/tool geometry rather than an empty replica.");
        Assert.That(renderers.Any(renderer => renderer.enabled && renderer.gameObject.activeInHierarchy), Is.EqualTo(visible));
    }
    private void Advance(float seconds) => Private(_visualizer, "AdvancePlayback", seconds);
    private double[] Offset(double amount) => _initial.Select(value => value + amount).ToArray();
    private void AssertTime(double time) => Assert.That(Get<float>(_visualizer, "PlaybackTimeSeconds"), Is.EqualTo(time).Within(0.00001));
    private void AssertLiveJoints() => AssertJoints(_live, _initial);
    private void AssertPreviewJoints(double[] expected) => AssertJoints(PlaybackRig, expected);
    private void AssertJoints(Component rig, double[] expected)
    {
        Assert.That(rig, Is.Not.Null);
        for (int index = 0; index < _names.Length; index++)
        {
            object[] arguments = { _names[index], 0d };
            Assert.That(Call(rig, "TryGetJointPosition", arguments), Is.EqualTo(true));
            Assert.That((double)arguments[1], Is.EqualTo(expected[index]).Within(0.00001), _names[index]);
        }
    }
    private void AssertPreviewTcp(double[] expected, float tolerance = 0.00003f)
    {
        Dictionary<string, double> positions = _names.Select((name, index) => new { name, value = expected[index] })
            .ToDictionary(pair => pair.name, pair => pair.value);
        Pose tcp = RobotFrameMath.Compose(_base, RosRobotModelMath.ForwardKinematics(_model, positions)["tcp"]);
        AssertPose(PoseOf(Get<Transform>(PlaybackRig, "Tcp")), tcp, tolerance);
    }
    private object Trajectory(double[] times, params double[][] values)
    {
        Assert.That(times.Length, Is.EqualTo(values.Length));
        Type trajectoryType = TypeOf("UR16eTrajectoryVisualizer").GetMethod("SetTrajectory").GetParameters()[0].ParameterType;
        object trajectory = Activator.CreateInstance(trajectoryType);
        object joints = Field(trajectory, "joint_trajectory").GetValue(trajectory);
        Set(joints, "joint_names", _names);
        FieldInfo pointsField = Field(joints, "points");
        Type pointType = pointsField.FieldType.GetElementType();
        Array points = Array.CreateInstance(pointType, times.Length);
        for (int index = 0; index < times.Length; index++)
        {
            object point = Activator.CreateInstance(pointType);
            Set(point, "positions", (double[])values[index].Clone());
            object duration = Field(point, "time_from_start").GetValue(point);
            if (duration == null) { duration = Activator.CreateInstance(Field(point, "time_from_start").FieldType); Set(point, "time_from_start", duration); }
            Set(duration, "sec", (int)Math.Floor(times[index]));
            Set(duration, "nanosec", (uint)Math.Round((times[index] - Math.Floor(times[index])) * 1000000000d));
            points.SetValue(point, index);
        }
        pointsField.SetValue(joints, points);
        return trajectory;
    }
    private object ReadRecordedMtcTrajectory()
    {
        DirectoryInfo directory = new DirectoryInfo(Application.dataPath);
        while (directory != null)
        {
            string path = Path.Combine(directory.FullName, "Tools/RobotFrames/mtc_success_live_20261023.json");
            if (File.Exists(path))
            {
                string snapshot = File.ReadAllText(path);
                string key = "\"/mtc/planned_trajectory\"";
                int start = snapshot.IndexOf(':', snapshot.IndexOf(key, StringComparison.Ordinal)) + 1;
                while (char.IsWhiteSpace(snapshot[start])) start++;
                bool quoted = false, escaped = false;
                int depth = 0;
                for (int index = start; index < snapshot.Length; index++)
                {
                    char value = snapshot[index];
                    if (quoted)
                    {
                        if (escaped) escaped = false;
                        else if (value == '\\') escaped = true;
                        else if (value == '"') quoted = false;
                    }
                    else if (value == '"') quoted = true;
                    else if (value == '{' || value == '[') depth++;
                    else if ((value == '}' || value == ']') && --depth == 0)
                    {
                        string json = snapshot.Substring(start, index - start + 1);
                        Type type = TypeOf("UR16eTrajectoryVisualizer").GetMethod("SetTrajectory").GetParameters()[0].ParameterType;
                        object result = JsonUtility.FromJson(json, type);
                        RestoreRecordedDurations(result, json);
                        return result;
                    }
                }
                Assert.Fail("Recorded MTC trajectory has invalid JSON.");
            }
            directory = directory.Parent;
        }
        throw new FileNotFoundException("The successful Nordbo MTC trajectory fixture is missing.");
    }
    // The connector's handwritten DurationMsg is not [Serializable]. ROS binary
    // transport populates it, but Unity's JSON fixture reader silently skips it.
    [Serializable] private sealed class RecordedTrajectory { public RecordedJoints joint_trajectory; }
    [Serializable] private sealed class RecordedJoints { public RecordedPoint[] points; }
    [Serializable] private sealed class RecordedPoint { public RecordedDuration time_from_start; }
    [Serializable] private sealed class RecordedDuration { public int sec; public uint nanosec; }
    private static void RestoreRecordedDurations(object trajectory, string json)
    {
        RecordedTrajectory recorded = JsonUtility.FromJson<RecordedTrajectory>(json);
        object joints = Field(trajectory, "joint_trajectory").GetValue(trajectory);
        Array points = (Array)Field(joints, "points").GetValue(joints);
        Assert.That(recorded.joint_trajectory.points.Length, Is.EqualTo(points.Length));
        for (int index = 0; index < points.Length; index++)
        {
            object point = points.GetValue(index);
            FieldInfo durationField = Field(point, "time_from_start");
            object duration = durationField.GetValue(point) ?? Activator.CreateInstance(durationField.FieldType);
            RecordedDuration saved = recorded.joint_trajectory.points[index].time_from_start;
            Assert.That(saved, Is.Not.Null);
            Set(duration, "sec", saved.sec);
            Set(duration, "nanosec", saved.nanosec);
            durationField.SetValue(point, duration);
        }
    }
    private static double TimeOf(object point)
    {
        object duration = Field(point, "time_from_start").GetValue(point);
        return Convert.ToDouble(Field(duration, "sec").GetValue(duration)) + Convert.ToDouble(Field(duration, "nanosec").GetValue(duration)) * 0.000000001d;
    }
    private static GameObject Prefab(bool ghost) => AssetDatabase.LoadAssetAtPath<GameObject>(
        "Assets/Prefabs/ur16e_kenos/ur16e_kenos" + (ghost ? "_transparent" : "") + ".prefab");
    private GameObject New(string name) { var result = new GameObject(name); result.SetActive(false); _owned.Add(result); return result; }
    private Component Add(string name, string type) { Component result = New(name).AddComponent(TypeOf(type)); ((Behaviour)result).enabled = false; result.gameObject.SetActive(true); return result; }
    private static Type TypeOf(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(assembly => assembly.GetType(name)).FirstOrDefault(type => type != null)
        ?? throw new InvalidOperationException("Missing runtime type " + name);
    private static FieldInfo Field(object value, string name) => value.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public)
        ?? throw new InvalidOperationException("Missing field " + value.GetType().Name + "." + name);
    private static void Set(object value, string name, object data) => Field(value, name).SetValue(value, data);
    private static T Get<T>(object value, string name) => (T)value.GetType().GetProperty(name).GetValue(value);
    private static T Get<T>(object value, string method, params object[] arguments) => (T)Call(value, method, arguments);
    private static object Call(object value, string method, params object[] arguments) => value.GetType().GetMethods()
        .Single(candidate => candidate.Name == method && candidate.GetParameters().Length == arguments.Length).Invoke(value, arguments);
    private static object Private(object value, string method, params object[] arguments) => value.GetType()
        .GetMethod(method, BindingFlags.Instance | BindingFlags.NonPublic).Invoke(value, arguments);
    private static Pose PoseOf(Transform value) => new Pose(value.position, value.rotation);
    private static void AssertPose(Pose actual, Pose expected, float tolerance = 0.00003f)
    {
        Assert.That(Vector3.Distance(actual.position, expected.position), Is.LessThan(tolerance));
        foreach (Vector3 basis in new[] { Vector3.right, Vector3.up, Vector3.forward })
            Assert.That(Vector3.Distance(actual.rotation * basis, expected.rotation * basis), Is.LessThan(tolerance));
    }
}
