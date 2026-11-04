using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>Real wrench admission and stale-data presentation without a ROS connection or robot.</summary>
public sealed class ForceTorqueSubscriberTests
{
    private const BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;
    private readonly List<GameObject> _owned = new List<GameObject>();
    private readonly List<GameObject> _executionOwners = new List<GameObject>();
    private Component _subscriber;

    [SetUp]
    public void CreateInactiveSubscriber()
    {
        _subscriber = New("Offline force subscriber").AddComponent(Runtime("ForceTorqueSubscriber"));
        // EditMode never invokes Start; enabling only exercises OnEnable's
        // pre-start branch and lets freshness retain its actual enabled policy.
        _subscriber.gameObject.SetActive(true);
        Invoke(_subscriber, "OnEnable");
    }

    [TearDown]
    public void Cleanup()
    {
        foreach (GameObject owner in _executionOwners)
            if (owner != null) SetBusy(owner, false);
        _executionOwners.Clear();
        if (_subscriber != null) Invoke(_subscriber, "OnDisable");
        foreach (GameObject value in _owned.AsEnumerable().Reverse())
            if (value != null) UnityEngine.Object.DestroyImmediate(value);
        _owned.Clear();
        Assert.That(Resources.FindObjectsOfTypeAll(Runtime("Unity.Robotics.ROSTCPConnector.ROSConnection")), Is.Empty);
    }

    [Test]
    public void WaitingDoesNotPretendAZeroWrenchHasBeenMeasured()
    {
        Assert.That(Get<bool>("HasSample"), Is.False);
        Assert.That(Get<bool>("IsFresh"), Is.False);
        Assert.That(double.IsPositiveInfinity(Get<double>("SampleAgeSeconds")), Is.True);
        Assert.That(Get<string>("StatusText").ToLowerInvariant(), Does.Contain("waiting"));
        Assert.That(Get<string>("ReadingText"), Does.Contain("N"));
    }

    [Test]
    public void CopiesSixSignedRosAxesWithoutUnityCoordinateConversion()
    {
        object message = Wrench();
        Assert.That(Observe(message), Is.True);
        Assert.That(Get<bool>("HasSample"), Is.True);
        Assert.That(Get<bool>("IsFresh"), Is.True);
        Assert.That(Get<double>("ForceX"), Is.EqualTo(12.3));
        Assert.That(Get<double>("ForceY"), Is.EqualTo(-4.5));
        Assert.That(Get<double>("ForceZ"), Is.EqualTo(6.7));
        Assert.That(Get<double>("TorqueX"), Is.EqualTo(-0.12));
        Assert.That(Get<double>("TorqueY"), Is.EqualTo(0.34));
        Assert.That(Get<double>("TorqueZ"), Is.EqualTo(-0.56));
        Assert.That(Get<double>("ForceMagnitude"), Is.EqualTo(Math.Sqrt(12.3*12.3+4.5*4.5+6.7*6.7)).Within(1e-10));
        Assert.That(Get<double>("TorqueMagnitude"), Is.EqualTo(Math.Sqrt(.12*.12+.34*.34+.56*.56)).Within(1e-10));
        Assert.That(Get<string>("FrameId"), Is.EqualTo("ur_tool0"));
        Assert.That(Get<int>("StampSeconds"), Is.EqualTo(123));
        Assert.That(Get<uint>("StampNanoseconds"), Is.EqualTo(456u));
        object force = Field(Field(message, "wrench"), "force");
        SetField(force, "x", 999.0);
        SetField(Field(message, "header"), "frame_id", "changed_source");
        Assert.That(Get<double>("ForceX"), Is.EqualTo(12.3), "Telemetry must own a snapshot, not a mutable message reference.");
        Assert.That(Get<string>("FrameId"), Is.EqualTo("ur_tool0"));
    }

    [Test]
    public void DisplayUsesInvariantUnitsAndThePublishedReferenceFrame()
    {
        CultureInfo previous = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("fr-BE");
            Assert.That(Observe(Wrench()), Is.True);
            string text = Get<string>("ReadingText");
            Assert.That(text, Does.Contain("12.3"));
            Assert.That(text, Does.Contain("-4.5"));
            Assert.That(text, Does.Contain("Nm"));
            Assert.That(text, Does.Contain("ur_tool0"));
        }
        finally { CultureInfo.CurrentCulture = previous; }
    }

    [Test]
    public void ReceiptTimeoutHidesOldValuesAndTheNextValidSampleRecovers()
    {
        Assert.That(Observe(Wrench()), Is.True);
        SetField(_subscriber, "_receivedAt", Time.realtimeSinceStartupAsDouble-2.0);
        Assert.That(Get<bool>("HasSample"), Is.True);
        Assert.That(Get<bool>("IsFresh"), Is.False);
        Assert.That(Get<double>("SampleAgeSeconds"), Is.GreaterThanOrEqualTo(2));
        Assert.That(Get<string>("StatusText").ToLowerInvariant(), Does.Contain("stale"));
        Assert.That(Get<string>("ReadingText"), Does.Not.Contain("12.3"));
        Assert.That(Observe(Wrench()), Is.True);
        Assert.That(Get<bool>("IsFresh"), Is.True);
        Assert.That(Get<string>("ReadingText"), Does.Contain("12.3"));
    }

    [TestCase("null")]
    [TestCase("header")]
    [TestCase("stamp")]
    [TestCase("wrench")]
    [TestCase("force")]
    [TestCase("torque")]
    [TestCase("frame-empty")]
    [TestCase("frame-space")]
    [TestCase("frame-control")]
    [TestCase("seconds")]
    [TestCase("nanoseconds")]
    public void MalformedSampleNeverRefreshesOrReplacesTheLastAcceptedSnapshot(string problem)
    {
        Assert.That(Observe(Wrench()), Is.True);
        double received = (double)Field(_subscriber, "_receivedAt");
        object message = Wrench();
        object header = Field(message, "header");
        switch (problem)
        {
            case "null": message = null; break;
            case "header": SetField(message, "header", null); break;
            case "stamp": SetField(header, "stamp", null); break;
            case "wrench": SetField(message, "wrench", null); break;
            case "force": SetField(Field(message, "wrench"), "force", null); break;
            case "torque": SetField(Field(message, "wrench"), "torque", null); break;
            case "frame-empty": SetField(header, "frame_id", ""); break;
            case "frame-space": SetField(header, "frame_id", "ur tool0"); break;
            case "frame-control": SetField(header, "frame_id", "ur_tool0\n"); break;
            case "seconds": SetField(Field(header, "stamp"), "sec", -1); break;
            case "nanoseconds": SetField(Field(header, "stamp"), "nanosec", 1000000000u); break;
        }
        Assert.That(Observe(message), Is.False, problem);
        Assert.That((double)Field(_subscriber, "_receivedAt"), Is.EqualTo(received));
        Assert.That(Get<double>("ForceX"), Is.EqualTo(12.3));
        Assert.That(Get<string>("FrameId"), Is.EqualTo("ur_tool0"));
        Assert.That(Get<bool>("IsFresh"), Is.False, "Invalid telemetry must not remain labelled live.");
        Assert.That(Get<string>("ReadingText"), Does.Not.Contain("12.3"));
        Assert.That(Observe(Wrench()), Is.True);
        Assert.That(Get<bool>("IsFresh"), Is.True);
    }

    [TestCase("force", "x", double.NaN)]
    [TestCase("force", "y", double.PositiveInfinity)]
    [TestCase("force", "z", double.NegativeInfinity)]
    [TestCase("torque", "x", double.NaN)]
    [TestCase("torque", "y", double.PositiveInfinity)]
    [TestCase("torque", "z", double.NegativeInfinity)]
    public void RejectsNonfiniteValuesOnEveryAxis(string vector, string axis, double value)
    {
        object message = Wrench();
        SetField(Field(Field(message, "wrench"), vector), axis, value);
        Assert.That(Observe(message), Is.False);
        Assert.That(Get<bool>("HasSample"), Is.False);
        Assert.That(Get<bool>("IsFresh"), Is.False);
    }

    [Test]
    public void DisableDiscardsTheOldSampleBeforeTheWorkflowCanBeReenabled()
    {
        Assert.That(Observe(Wrench()), Is.True);
        Invoke(_subscriber, "OnDisable");
        Assert.That(Get<bool>("HasSample"), Is.False);
        Assert.That(Get<bool>("IsFresh"), Is.False);
        Assert.That(Get<string>("ReadingText"), Does.Not.Contain("12.3"));
        Invoke(_subscriber, "OnEnable");
        Assert.That(Get<bool>("IsFresh"), Is.False);
    }

    [Test]
    public void DisconnectedTransportHidesAndDiscardsAnOtherwiseFreshReading()
    {
        Component connector = DisabledConnector();
        SetField(_subscriber, "_connection", connector);
        Assert.That(Observe(Wrench()), Is.True);
        Assert.That(Get<bool>("IsFresh"), Is.False);
        Assert.That(Get<string>("StatusText").ToLowerInvariant(), Does.Contain("disconnected"));
        Assert.That(Get<string>("ReadingText"), Does.Not.Contain("12.3"));
        Invoke(_subscriber, "Update");
        Assert.That(Get<bool>("HasSample"), Is.False);
        Assert.That((bool)connector.GetType().GetProperty("HasConnectionThread").GetValue(connector), Is.False);
    }

    [Test]
    public void SceneTeardownUnsubscribesTheForceTopicFromTheRetainedConnector()
    {
        Component connector = DisabledConnector();
        Runtime("RosMessageTypes.Geometry.WrenchStampedMsg").GetMethod("Register", BindingFlags.Static | BindingFlags.Public).Invoke(null, null);
        string topic = Get<string>("TopicName");
        connector.GetType().GetMethod("SubscribeByMessageName").Invoke(connector,
            new object[] { topic, "geometry_msgs/WrenchStamped", (Action<object>)(_ => { }) });
        Assert.That(connector.GetType().GetMethod("HasSubscriber").Invoke(connector, new object[] { topic }), Is.EqualTo(true));
        SetField(_subscriber, "_connection", connector);
        SetField(_subscriber, "_subscribedTopic", topic);
        Invoke(_subscriber, "OnDisable");
        Assert.That(connector.GetType().GetMethod("HasSubscriber").Invoke(connector, new object[] { topic }), Is.EqualTo(false));
        Invoke(_subscriber, "OnDestroy");
        Assert.That((bool)connector.GetType().GetProperty("HasConnectionThread").GetValue(connector), Is.False);
    }

    [Test]
    public void MagnitudesAvoidIntermediateOverflowButRejectAnUnrepresentableNorm()
    {
        object message = Wrench();
        object force = Field(Field(message, "wrench"), "force");
        SetField(force, "x", double.MaxValue); SetField(force, "y", 0d); SetField(force, "z", 0d);
        Assert.That(Observe(message), Is.True);
        Assert.That(Get<double>("ForceMagnitude"), Is.EqualTo(double.MaxValue));
        SetField(force, "y", double.MaxValue);
        Assert.That(Observe(message), Is.False);
        Assert.That(Get<bool>("IsFresh"), Is.False);
    }

    [Test]
    public void PeakStartsEmptyAndDoesNotSeedFromTheReadingBeforeExecute()
    {
        Assert.That(Observe(MagnitudeWrench(80d, 8d)), Is.True);
        Assert.That(Get<bool>("HasExecutionPeak"), Is.False);
        GameObject owner = ExecutionOwner();
        SetBusy(owner, true);
        Assert.That(Get<bool>("IsRecordingExecutionPeak"), Is.True);
        Assert.That(Get<bool>("HasExecutionPeak"), Is.False, "The previous idle reading is outside this execution.");
        Assert.That(Observe(MagnitudeWrench(3d, .4d)), Is.True);
        AssertPeak(3d, .4d);
    }

    [Test]
    public void EveryDeliveredSamplePushesIndependentForceAndTorquePeaks()
    {
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(3d, 4d)), Is.True);
        Assert.That(Observe(MagnitudeWrench(30d, 2d)), Is.True);
        Assert.That(Observe(MagnitudeWrench(5d, 9d)), Is.True);
        Assert.That(Observe(MagnitudeWrench(2d, 1d)), Is.True);
        // No menu update occurs between these samples: a short spike must not
        // disappear just because the gauges are redrawn at a lower frequency.
        AssertPeak(30d, 9d);
        SetBusy(owner, true);
        AssertPeak(30d, 9d, "Repeated busy/executing feedback is not a new execution.");
    }

    [Test]
    public void TerminalExecutionFreezesTheMarkUntilAnotherPlanOrExecute()
    {
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(12d, .8d)), Is.True);
        SetBusy(owner, false);
        Assert.That(Get<bool>("IsRecordingExecutionPeak"), Is.False);
        Assert.That(Observe(MagnitudeWrench(90d, 9d)), Is.True);
        AssertPeak(12d, .8d, "Idle samples after success, failure or rejection must not raise the finished execution's peak.");
        PlanningStarted();
        Assert.That(Get<bool>("HasExecutionPeak"), Is.False);
        Assert.That(Get<bool>("IsRecordingExecutionPeak"), Is.False);
        Assert.That(Observe(MagnitudeWrench(50d, 5d)), Is.True);
        Assert.That(Get<bool>("HasExecutionPeak"), Is.False, "Planning does not collect an execution peak.");
    }

    [Test]
    public void RepeatExecuteClearsTheOldMarkAndWaitsForItsOwnFirstSample()
    {
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(20d, 2d)), Is.True);
        SetBusy(owner, false); SetBusy(owner, true);
        Assert.That(Get<bool>("HasExecutionPeak"), Is.False);
        Assert.That(Get<bool>("IsRecordingExecutionPeak"), Is.True);
        Assert.That(Observe(MagnitudeWrench(2d, .2d)), Is.True);
        AssertPeak(2d, .2d);
    }

    [Test]
    public void MeasuredZeroCreatesARealPeakMarkDuringExecution()
    {
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Get<bool>("HasExecutionPeak"), Is.False);
        Assert.That(Observe(MagnitudeWrench(0d, 0d)), Is.True);
        AssertPeak(0d, 0d);
    }

    [Test]
    public void InvalidAndDisconnectedSamplesCannotRaiseRecordedPeaks()
    {
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(5d, .5d)), Is.True);
        object invalid = MagnitudeWrench(50d, 5d);
        SetField(Field(Field(invalid, "wrench"), "torque"), "z", double.NaN);
        Assert.That(Observe(invalid), Is.False);
        AssertPeak(5d, .5d);
        Component connector = DisabledConnector();
        SetField(_subscriber, "_connection", connector);
        Assert.That(Observe(MagnitudeWrench(100d, 10d)), Is.True);
        Assert.That(Get<bool>("IsFresh"), Is.False);
        AssertPeak(5d, .5d);
        Invoke(_subscriber, "Update");
        AssertPeak(5d, .5d, "Clearing a disconnected live reading must preserve the historical execution peak.");
    }

    [Test]
    public void StaleAndDisabledReadingsPreserveTheFinishedPeak()
    {
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(10d, 1d)), Is.True);
        SetBusy(owner, false);
        SetField(_subscriber, "_receivedAt", Time.realtimeSinceStartupAsDouble-2d);
        Assert.That(Get<bool>("IsFresh"), Is.False);
        AssertPeak(10d, 1d);
        Invoke(_subscriber, "OnDisable");
        Assert.That(Get<bool>("IsRecordingExecutionPeak"), Is.False);
        AssertPeak(10d, 1d);
        Invoke(_subscriber, "OnEnable"); Invoke(_subscriber, "OnEnable");
        AssertPeak(10d, 1d);
        PlanningStarted();
        Assert.That(Get<bool>("HasExecutionPeak"), Is.False);
        SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(1d, .1d)), Is.True);
        AssertPeak(1d, .1d);
    }

    [Test]
    public void ReenablingDuringTheSameExecutionResumesItsExistingPeak()
    {
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(20d, 2d)), Is.True);
        Invoke(_subscriber, "OnDisable");
        Assert.That(Get<bool>("IsRecordingExecutionPeak"), Is.False);
        Invoke(_subscriber, "OnEnable"); Invoke(_subscriber, "OnEnable");
        Assert.That(Get<bool>("IsRecordingExecutionPeak"), Is.True);
        AssertPeak(20d, 2d);
        Assert.That(Observe(MagnitudeWrench(5d, .5d)), Is.True);
        AssertPeak(20d, 2d);
        Assert.That(Observe(MagnitudeWrench(25d, .3d)), Is.True);
        AssertPeak(25d, 2d);
    }

    [TestCase(false)]
    [TestCase(true)]
    public void ANewPlanOrExecuteWhileDisabledClearsTheOldPeakOnReenable(bool execute)
    {
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(20d, 2d)), Is.True);
        SetBusy(owner, false); Invoke(_subscriber, "OnDisable");
        if (execute) { SetBusy(owner, true); SetBusy(owner, false); }
        else PlanningStarted();
        Invoke(_subscriber, "OnEnable");
        Assert.That(Get<bool>("HasExecutionPeak"), Is.False);
        Assert.That(Get<bool>("IsRecordingExecutionPeak"), Is.False);
    }

    [TestCase("TcpPlanningController", "Plan")]
    [TestCase("FixtureTilingController", "Plan")]
    [TestCase("PickPlaceController", "SendPickAndPlacePlanningRequest")]
    public void EveryPlanningEntryClearsTheFinishedPeakEvenWhenLocalValidationRejectsIt(string type, string method)
    {
        Component planner = InertPlanner(type);
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(10d, 1d)), Is.True); SetBusy(owner, false);
        planner.GetType().GetMethod(method).Invoke(planner, null);
        Assert.That(Get<bool>("HasExecutionPeak"), Is.False);
        Assert.That(Get<bool>("IsRecordingExecutionPeak"), Is.False);
    }

    [TestCase("TcpPlanningController", "Plan")]
    [TestCase("FixtureTilingController", "Plan")]
    [TestCase("PickPlaceController", "SendPickAndPlacePlanningRequest")]
    public void BusyGuardedPlanningDoesNotResetTheRunningExecutionPeak(string type, string method)
    {
        Component planner = InertPlanner(type);
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(10d, 1d)), Is.True);
        planner.GetType().GetMethod(method).Invoke(planner, null);
        AssertPeak(10d, 1d);
        Assert.That(Get<bool>("IsRecordingExecutionPeak"), Is.True);
    }

    [TestCase("SimplePickAndPlace")]
    [TestCase("RepetitivePickAndPlace")]
    public void BuildingEitherMenuShowsTheForceCardWithoutStartingTelemetry(string workflow)
    {
        GameObject owner = New("Offline force workflow menu");
        Component menu = owner.AddComponent(Runtime("ToolSelectionMenu"));
        Type mode = Runtime("URInputManager").GetNestedType("WorkflowMode");
        menu.GetType().GetMethod("ConfigureWorkflow").Invoke(menu, new[] { Enum.Parse(mode, workflow) });
        Invoke(menu, "BuildMenu");
        Canvas canvas = (Canvas)Field(menu, "_canvas");
        _owned.Add(canvas.gameObject);
        RectTransform task = (RectTransform)Field(menu, "_taskView");
        Assert.That(task.Find("UR16e force readout"), Is.Not.Null);
        Assert.That(owner.GetComponent(Runtime("ForceTorqueSubscriber")), Is.Null);
        Assert.That(Field(menu, "_forceStatus"), Is.Not.Null);
        Assert.That(Field(menu, "_forceReading"), Is.Not.Null);
        Assert.That((bool)Field(menu, "_forceReading").GetType().GetProperty("richText").GetValue(Field(menu, "_forceReading")), Is.False);
        Assert.That(task.Find("UR16e force readout/Force magnitude gauge"), Is.Not.Null);
        Assert.That(task.Find("UR16e force readout/Torque magnitude gauge"), Is.Not.Null);
    }

    [TestCase("SimplePickAndPlace")]
    [TestCase("RepetitivePickAndPlace")]
    public void GaugesShowOnlyMagnitudesAndRetainPeakMarksWhenTheLiveReadingGoesStale(string workflow)
    {
        Component menu = BuildMenu(workflow);
        object forceGauge = Field(menu, "_forceMagnitudeGauge"), torqueGauge = Field(menu, "_torqueMagnitudeGauge");
        RefreshGauges(menu);
        Assert.That(((RectTransform)Field(forceGauge, "PeakMarker")).gameObject.activeSelf, Is.False);
        Assert.That(((RectTransform)Field(torqueGauge, "PeakMarker")).gameObject.activeSelf, Is.False);
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(30d, 3d)), Is.True);
        Assert.That(Observe(MagnitudeWrench(5d, .5d)), Is.True);
        SetBusy(owner, false); RefreshGauges(menu);
        Assert.That(Text(Field(forceGauge, "CurrentValue")), Does.Contain("5.00"));
        Assert.That(Text(Field(torqueGauge, "CurrentValue")), Does.Contain("0.50"));
        Assert.That(Text(Field(forceGauge, "PeakValue")), Does.Contain("30.00"));
        Assert.That(Text(Field(torqueGauge, "PeakValue")), Does.Contain("3.00"));
        Assert.That(((RectTransform)Field(forceGauge, "PeakMarker")).gameObject.activeSelf, Is.True);
        Assert.That(((RectTransform)Field(torqueGauge, "PeakMarker")).gameObject.activeSelf, Is.True);
        Assert.That(Text(Field(menu, "_forceReading")), Does.Not.Contain(" X "));
        SetField(_subscriber, "_receivedAt", Time.realtimeSinceStartupAsDouble-2d); RefreshGauges(menu);
        Assert.That(((RectTransform)Field(forceGauge, "Fill")).gameObject.activeSelf, Is.False);
        Assert.That(((RectTransform)Field(torqueGauge, "Fill")).gameObject.activeSelf, Is.False);
        Assert.That(((RectTransform)Field(forceGauge, "PeakMarker")).gameObject.activeSelf, Is.True);
        Assert.That(Text(Field(forceGauge, "PeakValue")), Does.Contain("30.00"));
        PlanningStarted(); RefreshGauges(menu);
        Assert.That(((RectTransform)Field(forceGauge, "PeakMarker")).gameObject.activeSelf, Is.False);
        Assert.That(((RectTransform)Field(torqueGauge, "PeakMarker")).gameObject.activeSelf, Is.False);
    }

    [Test]
    public void GaugeRangesStayFixedAndClampOnlyTheGraphicsWhileKeepingActualReadingsAndPeaks()
    {
        Component menu = BuildMenu("SimplePickAndPlace");
        object forceGauge = Field(menu, "_forceMagnitudeGauge"), torqueGauge = Field(menu, "_torqueMagnitudeGauge");
        Assert.That((double)Field(forceGauge, "Maximum"), Is.EqualTo(100d));
        Assert.That((double)Field(torqueGauge, "Maximum"), Is.EqualTo(20d));
        GameObject owner = ExecutionOwner(); SetBusy(owner, true);
        Assert.That(Observe(MagnitudeWrench(1000d, 30d)), Is.True); RefreshGauges(menu);
        Assert.That(Text(Field(forceGauge, "CurrentValue")), Does.Contain("1000.00"));
        Assert.That(Text(Field(torqueGauge, "CurrentValue")), Does.Contain("30.00"));
        Assert.That(Observe(MagnitudeWrench(400d, 100d)), Is.True); RefreshGauges(menu);
        AssertPeak(1000d, 100d);
        Assert.That(Text(Field(forceGauge, "CurrentValue")), Does.Contain("400.00"));
        Assert.That(Text(Field(torqueGauge, "CurrentValue")), Does.Contain("100.00"));
        Assert.That(Text(Field(forceGauge, "PeakValue")), Does.Contain("1000.00"));
        Assert.That(Text(Field(torqueGauge, "PeakValue")), Does.Contain("100.00"));
        foreach (object gauge in new[] { forceGauge, torqueGauge })
        {
            RectTransform fill = (RectTransform)Field(gauge, "Fill"), marker = (RectTransform)Field(gauge, "PeakMarker");
            float trackWidth = ((RectTransform)fill.parent).rect.width;
            Assert.That(fill.sizeDelta.x, Is.InRange(0f, trackWidth));
            Assert.That(marker.anchoredPosition.x, Is.GreaterThanOrEqualTo(0f));
            Assert.That(marker.anchoredPosition.x+marker.sizeDelta.x, Is.LessThanOrEqualTo(trackWidth));
        }
        Assert.That((double)Field(forceGauge, "Maximum"), Is.EqualTo(100d));
        Assert.That((double)Field(torqueGauge, "Maximum"), Is.EqualTo(20d));
        SetBusy(owner, false);
        Assert.That(Observe(MagnitudeWrench(1d, .1d)), Is.True); RefreshGauges(menu);
        Assert.That((double)Field(forceGauge, "Maximum"), Is.EqualTo(100d));
        Assert.That((double)Field(torqueGauge, "Maximum"), Is.EqualTo(20d));
        AssertPeak(1000d, 100d);
    }

    /// <summary>Explicit full-width preview export, including the card outside the main menu panel.</summary>
    public static void RenderForceReadoutPreviews()
    {
        var fixture = new ForceTorqueSubscriberTests();
        RenderTexture previous = RenderTexture.active, target = null;
        Texture2D pixels = null;
        try
        {
            fixture.CreateInactiveSubscriber();
            GameObject viewer = fixture.New("Force readout preview camera");
            Camera camera = viewer.AddComponent<Camera>(); viewer.SetActive(true);
            camera.transform.SetPositionAndRotation(new Vector3(.158f, 0f, -1f), Quaternion.identity);
            camera.orthographic = true; camera.orthographicSize = .415f; camera.aspect = 1f;
            camera.clearFlags = CameraClearFlags.SolidColor; camera.backgroundColor = new Color(.08f, .11f, .15f, 1f);
            target = new RenderTexture(1152, 1152, 24, RenderTextureFormat.ARGB32);
            pixels = new Texture2D(1152, 1152, TextureFormat.RGB24, false);
            camera.targetTexture = target;
            string directory = System.IO.Path.GetFullPath("ForceReadoutPreviews");
            System.IO.Directory.CreateDirectory(directory);
            Type textType = Runtime("TMPro.TMP_Text"), fontType = Runtime("TMPro.TMP_FontAsset");
            UnityEngine.Object font = Resources.Load("Fonts & Materials/LiberationSans SDF", fontType);
            foreach (string workflow in new[] { "SimplePickAndPlace", "RepetitivePickAndPlace" })
            {
                Component menu = fixture.New("Force preview " + workflow).AddComponent(Runtime("ToolSelectionMenu"));
                Type mode = Runtime("URInputManager").GetNestedType("WorkflowMode");
                menu.GetType().GetMethod("ConfigureWorkflow").Invoke(menu, new[] { Enum.Parse(mode, workflow) });
                Invoke(menu, "BuildMenu");
                Canvas canvas = (Canvas)Field(menu, "_canvas"); fixture._owned.Add(canvas.gameObject);
                canvas.worldCamera = camera;
                canvas.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
                ((RectTransform)Field(menu, "_panel")).gameObject.SetActive(true);
                SetField(menu, "_forceTorque", fixture._subscriber);
                foreach (Component text in canvas.GetComponentsInChildren(textType, true))
                    if (font != null) textType.GetProperty("font").SetValue(text, font);
                foreach (string state in new[] { "waiting", "live", "peak", "stale", "invalid", "planning-cleared", "repeat-start" })
                {
                    foreach (GameObject owner in fixture._executionOwners) SetBusy(owner, false);
                    Invoke(fixture._subscriber, "OnDisable");
                    Invoke(fixture._subscriber, "OnEnable");
                    PlanningStarted();
                    if (state != "waiting") fixture.Observe(MagnitudeWrench(12.3d, .67d));
                    if (state == "peak" || state == "stale" || state == "invalid" || state == "planning-cleared" || state == "repeat-start")
                    {
                        GameObject owner = fixture.ExecutionOwner(); SetBusy(owner, true);
                        fixture.Observe(MagnitudeWrench(38d, 3.5d));
                        fixture.Observe(MagnitudeWrench(12.3d, .67d));
                        SetBusy(owner, false);
                        if (state == "planning-cleared") PlanningStarted();
                        if (state == "repeat-start") SetBusy(owner, true);
                    }
                    if (state == "stale") SetField(fixture._subscriber, "_receivedAt", Time.realtimeSinceStartupAsDouble-2d);
                    if (state == "invalid") fixture.Observe(null);
                    SetField(menu, "_nextForceReadoutAt", -1f); Invoke(menu, "RefreshForceReadout");
                    Canvas.ForceUpdateCanvases();
                    foreach (Component text in canvas.GetComponentsInChildren(textType, true))
                    {
                        textType.GetMethod("ForceMeshUpdate").Invoke(text, new object[] { true, true });
                        if (text.gameObject.activeInHierarchy && (bool)textType.GetProperty("isTextOverflowing").GetValue(text))
                            throw new InvalidOperationException("Force preview text overflow: " + text.name);
                    }
                    camera.Render(); RenderTexture.active = target;
                    pixels.ReadPixels(new Rect(0, 0, 1152, 1152), 0, 0); pixels.Apply();
                    string path = System.IO.Path.Combine(directory, workflow + "-" + state + ".png");
                    System.IO.File.WriteAllBytes(path, pixels.EncodeToPNG());
                    Debug.Log("ForceReadoutPreview: " + path);
                }
                canvas.gameObject.SetActive(false);
            }
            camera.targetTexture = null;
        }
        finally
        {
            RenderTexture.active = previous;
            if (pixels != null) UnityEngine.Object.DestroyImmediate(pixels);
            if (target != null) { target.Release(); UnityEngine.Object.DestroyImmediate(target); }
            fixture.Cleanup();
        }
    }

    private GameObject New(string name) { var value = new GameObject(name); value.SetActive(false); _owned.Add(value); return value; }
    private Component DisabledConnector()
    {
        Component connector = New("Offline disabled force transport").AddComponent(Runtime("Unity.Robotics.ROSTCPConnector.ROSConnection"));
        ((Behaviour)connector).enabled = false;
        connector.GetType().GetProperty("ConnectOnStart").SetValue(connector, false);
        connector.GetType().GetProperty("ShowHud").SetValue(connector, false);
        Invoke(connector, "Awake");
        return connector;
    }
    private T Get<T>(string name) => (T)_subscriber.GetType().GetProperty(name).GetValue(_subscriber);
    private bool Observe(object message) => (bool)_subscriber.GetType().GetMethod("ObserveWrench").Invoke(_subscriber, new[] { message });
    private void AssertPeak(double force, double torque, string reason = null)
    {
        Assert.That(Get<bool>("HasExecutionPeak"), Is.True, reason);
        Assert.That(Get<double>("PeakForceMagnitude"), Is.EqualTo(force).Within(1e-10), reason);
        Assert.That(Get<double>("PeakTorqueMagnitude"), Is.EqualTo(torque).Within(1e-10), reason);
    }
    private GameObject ExecutionOwner()
    {
        GameObject owner = New("Offline force execution owner");
        _executionOwners.Add(owner);
        return owner;
    }
    private Component BuildMenu(string workflow)
    {
        Component menu = New("Offline magnitude gauge menu").AddComponent(Runtime("ToolSelectionMenu"));
        Type mode = Runtime("URInputManager").GetNestedType("WorkflowMode");
        menu.GetType().GetMethod("ConfigureWorkflow").Invoke(menu, new[] { Enum.Parse(mode, workflow) });
        Invoke(menu, "BuildMenu");
        Canvas canvas = (Canvas)Field(menu, "_canvas");
        _owned.Add(canvas.gameObject);
        SetField(menu, "_forceTorque", _subscriber);
        return menu;
    }
    private Component InertPlanner(string type)
    {
        Component planner = New("Offline planning trigger " + type).AddComponent(Runtime(type));
        if (type == "PickPlaceController")
        {
            SetField(planner, "_plansManager", New("Offline planning status").AddComponent(Runtime("MTCPlansManager")));
            SetField(planner, "_tilePlanningEnabled", false);
        }
        return planner;
    }
    private static void SetBusy(GameObject owner, bool busy) => Runtime("RobotExecutionState").GetMethod("SetBusy").Invoke(null, new object[] { owner, busy });
    private static void PlanningStarted() => Runtime("RobotExecutionState").GetMethod("NotifyPlanningStarted").Invoke(null, null);
    private static void RefreshGauges(Component menu) { SetField(menu, "_nextForceReadoutAt", -1f); Invoke(menu, "RefreshForceReadout"); }
    private static string Text(object label) => (string)label.GetType().GetProperty("text").GetValue(label);
    private static object Field(object owner, string name) => owner.GetType().GetField(name, BindingFlags.Public | Private).GetValue(owner);
    private static void SetField(object owner, string name, object value) => owner.GetType().GetField(name, BindingFlags.Public | Private).SetValue(owner, value);
    private static void Invoke(object owner, string name) => owner.GetType().GetMethod(name, Private).Invoke(owner, null);
    private static Type Runtime(string name) => AppDomain.CurrentDomain.GetAssemblies().Select(assembly => assembly.GetType(name, false)).First(type => type != null);
    private static object Wrench()
    {
        object message = Activator.CreateInstance(Runtime("RosMessageTypes.Geometry.WrenchStampedMsg"));
        object header = Field(message, "header"), stamp = Field(header, "stamp");
        SetField(header, "frame_id", "ur_tool0");
        SetField(stamp, "sec", 123); SetField(stamp, "nanosec", 456u);
        object wrench = Field(message, "wrench"), force = Field(wrench, "force"), torque = Field(wrench, "torque");
        SetField(force, "x", 12.3); SetField(force, "y", -4.5); SetField(force, "z", 6.7);
        SetField(torque, "x", -.12); SetField(torque, "y", .34); SetField(torque, "z", -.56);
        return message;
    }
    private static object MagnitudeWrench(double forceMagnitude, double torqueMagnitude)
    {
        object message = Wrench();
        object wrench = Field(message, "wrench"), force = Field(wrench, "force"), torque = Field(wrench, "torque");
        SetField(force, "x", -forceMagnitude); SetField(force, "y", 0d); SetField(force, "z", 0d);
        SetField(torque, "x", 0d); SetField(torque, "y", -torqueMagnitude); SetField(torque, "z", 0d);
        return message;
    }
}
