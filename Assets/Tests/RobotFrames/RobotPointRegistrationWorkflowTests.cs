using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.Events;

/// <summary>Actual registration, subscriber, placement, native rig and coordinate gates; no ROS or XR session.</summary>
public sealed class RobotPointRegistrationWorkflowTests
{
    private readonly List<GameObject> _owned = new List<GameObject>();
    private Transform _tracking;
    private Component _coordinates;
    private Component _instantiator;
    private Component _registration;
    private Component _feedback;
    private RosRobotModel _model;
    private string[] _jointNames;
    private double[] _angles;
    private Vector3[] _modelPoints;
    private Pose _baseInTracking;
    private GameObject _robot;

    [SetUp]
    public void SetUp()
    {
        _tracking = NewObject("Point registration tracking origin").transform;
        _tracking.SetPositionAndRotation(new Vector3(2f, 1f, -3f), Quaternion.Euler(0f, 48f, 0f));
        _coordinates = NewObject("Point registration coordinates").AddComponent(RuntimeType("RobotCoordinateService"));
        NewObject("Point registration robot state").AddComponent(RuntimeType("RobotStateService"));
        GameObject owner = NewObject("Inactive point registration workflow");
        owner.SetActive(false); // Start cannot request a ROS connection or an XR hardware detector.
        _instantiator = owner.AddComponent(RuntimeType("URInstantiator"));
        Set(_instantiator, "_URPrefab", AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab"));
        InvokePrivate(_instantiator, "Awake");
        _registration = owner.GetComponent(RuntimeType("RobotPointRegistration"));
        Assert.That(_registration, Is.Not.Null);
        _feedback = owner.AddComponent(RuntimeType("JointStateSubscriber"));
        Set(_registration, "_instantiator", _instantiator);
        Set(_registration, "_jointFeedback", _feedback);
        Set(_registration, "_trackingSpace", _tracking);
        UnityEvent<GameObject, Pose> created = (UnityEvent<GameObject, Pose>)_instantiator.GetType().GetField("OnURInstantiated").GetValue(_instantiator);
        created.AddListener((robot, pose) => { _robot = robot; _owned.Add(robot); });

        _model = RosRobotModelCatalog.Load().Find("kenos");
        RosRobotGoldenCase bent = Array.Find(_model.goldenCases, pose => pose.name == "bent");
        Assert.That(bent, Is.Not.Null, "A nonstraight calibration pose is required by this fixture.");
        _jointNames = (string[])bent.jointNames.Clone();
        _angles = (double[])bent.jointPositions.Clone();
        RecomputeModelPoints();
        _baseInTracking = new Pose(new Vector3(0.4f, -0.6f, 1.2f), Quaternion.Euler(13f, -34f, 7f));
    }

    [TearDown]
    public void TearDown()
    {
        for (int i = _owned.Count - 1; i >= 0; i--)
            if (_owned[i] != null) UnityEngine.Object.DestroyImmediate(_owned[i]);
        _owned.Clear();
        Type connection = Type.GetType("Unity.Robotics.ROSTCPConnector.ROSConnection, Unity.Robotics.ROSTCPConnector", false);
        if (connection != null)
            Assert.That(Resources.FindObjectsOfTypeAll(connection), Is.Empty, "Point registration must never obtain ROS transport during offline validation.");
    }

    [Test]
    public void FifthTouchPlacesTheActualRobotAtTheRigidFitAndRetainsAllLiveJointAngles()
    {
        BeginWithFeedback();
        for (int i = 0; i < 4; i++)
        {
            Assert.That(Capture(i), Is.True, Status);
            Assert.That(Get<int>(_registration, "CapturedPointCount"), Is.EqualTo(i + 1));
            Assert.That(Get<bool>(_instantiator, "HasRobot"), Is.False, "The overlay must first appear after all five points.");
            AssertCoordinatesBlocked();
        }
        Assert.That(Capture(4), Is.True, Status);
        Assert.That(_robot, Is.Not.Null);
        Assert.That(Get<bool>(_registration, "IsCollecting"), Is.False);
        Assert.That(Get<bool>(_registration, "HasPlacement"), Is.True);
        Assert.That(Get<bool>(_instantiator, "IsPointPlaced"), Is.True);
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.True);
        Assert.That(Get<float>(_registration, "RmsErrorMeters"), Is.LessThan(0.00001f));
        AssertPose(ExpectedBaseWorld, BaseWorld);
        Component rig = _robot.GetComponent(RuntimeType("RosRobotRig"));
        for (int i = 0; i < _jointNames.Length; i++)
        {
            object[] query = { _jointNames[i], 0d };
            Assert.That(Call(rig, "TryGetJointPosition", query), Is.EqualTo(true));
            Assert.That((double)query[1], Is.EqualTo(_angles[i]).Within(1e-10));
        }
        SimulatePhysics();
        AssertPose(ExpectedBaseWorld, BaseWorld);
    }

    [Test]
    public void CollectionBlocksCoordinateCommandsAndConcurrentInitializationCannotRearmThem()
    {
        PlaceAllFive();
        Component rig = _robot.GetComponent(RuntimeType("RosRobotRig"));
        ArticulationBody body = Get<Transform>(rig, "BaseLink").GetComponent<ArticulationBody>();
        Call(_registration, "BeginRegistration");
        Assert.That(Get<bool>(_coordinates, "IsCalibrationInProgress"), Is.True);
        AssertCoordinatesBlocked();
        _coordinates.GetType().GetMethod("Initialize", new[] { typeof(ArticulationBody), typeof(Transform) })
            .Invoke(_coordinates, new object[] { body, _tracking });
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.False);
        AssertCoordinatesBlocked();
        Assert.That((bool)RuntimeType("ToolSelectionController").GetProperty("CanSendRobotCommands").GetValue(null), Is.False);
    }

    [Test]
    public void CancellingBeforeAnyPlacementEndsCollectionButLeavesCoordinatesUnavailable()
    {
        BeginWithFeedback();
        Assert.That(Capture(0), Is.True, Status);
        Call(_registration, "CancelRegistration");
        Assert.That(Get<bool>(_registration, "IsCollecting"), Is.False);
        Assert.That(Get<int>(_registration, "CapturedPointCount"), Is.Zero);
        Assert.That(Get<bool>(_coordinates, "IsCalibrationInProgress"), Is.False);
        Assert.That(Get<bool>(_instantiator, "HasRobot"), Is.False);
        AssertCoordinatesBlocked();
    }

    [Test]
    public void CancellingAReplacementRestoresThePriorPlacementAndRendererStatesAfterOriginMovement()
    {
        PlaceAllFive();
        GameObject priorRobot = _robot;
        Renderer[] renderers = _robot.GetComponentsInChildren<Renderer>(true);
        Assert.That(renderers.Length, Is.GreaterThan(1));
        renderers[0].enabled = false;
        Dictionary<Renderer, bool> states = new Dictionary<Renderer, bool>();
        foreach (Renderer renderer in renderers) states.Add(renderer, renderer.enabled);
        Call(_registration, "BeginRegistration");
        foreach (Renderer renderer in renderers) Assert.That(renderer.enabled, Is.False);
        Assert.That(Capture(0), Is.True, Status);
        _tracking.SetPositionAndRotation(new Vector3(-3f, 0.4f, 2f), Quaternion.Euler(0f, -71f, 0f));
        Call(_registration, "CancelRegistration");
        Assert.That(Get<GameObject>(_instantiator, "RobotInstance"), Is.SameAs(priorRobot));
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.True);
        AssertPose(ExpectedBaseWorld, BaseWorld);
        foreach (KeyValuePair<Renderer, bool> state in states)
            Assert.That(state.Key.enabled, Is.EqualTo(state.Value));
    }

    [Test]
    public void MissingOrPartialFeedbackCannotCaptureAPointOrSubstituteZeroJointAngles()
    {
        Call(_registration, "BeginRegistration");
        Assert.That(Capture(0), Is.False);
        string[] partialNames = { _jointNames[0], _jointNames[1], _jointNames[2] };
        double[] partialAngles = { _angles[0], _angles[1], _angles[2] };
        Assert.That(Call(_feedback, "ObserveJointState", partialNames, partialAngles), Is.EqualTo(true));
        Assert.That(Capture(0), Is.False);
        Assert.That(Status, Does.Contain("joint feedback"));
        Assert.That(Get<int>(_registration, "CapturedPointCount"), Is.Zero);
        Assert.That(Get<bool>(_instantiator, "HasRobot"), Is.False);
        AssertCoordinatesBlocked();
    }

    [Test]
    public void StaleFeedbackCannotCaptureAPoint()
    {
        BeginWithFeedback();
        Dictionary<string, float> received = Field<Dictionary<string, float>>(_feedback, "_receivedAt");
        foreach (string name in _jointNames) received[name] = Time.realtimeSinceStartup - 60f;
        Assert.That(Capture(0), Is.False);
        Assert.That(Status, Does.Contain("fresh ROS joint feedback"));
        Assert.That(Get<int>(_registration, "CapturedPointCount"), Is.Zero);
        AssertCoordinatesBlocked();
    }

    [Test]
    public void RobotMovementDiscardsEarlierTouchesAndRequiresACompleteNewSet()
    {
        BeginWithFeedback();
        Assert.That(Capture(0), Is.True, Status);
        double[] moved = (double[])_angles.Clone();
        moved[0] += 0.1d;
        Assert.That(Call(_feedback, "ObserveJointState", _jointNames, moved), Is.EqualTo(true));
        Assert.That(Capture(1), Is.False);
        Assert.That(Status, Does.Contain("robot moved"));
        Assert.That(Get<int>(_registration, "CapturedPointCount"), Is.Zero);
        Assert.That(Get<bool>(_registration, "IsCollecting"), Is.True);
        AssertCoordinatesBlocked();
    }

    [Test]
    public void PoorFitStaysBlockedAndUndoingTheBadFifthPointAllowsACorrectedFit()
    {
        BeginWithFeedback();
        for (int i = 0; i < 4; i++) Assert.That(Capture(i), Is.True, Status);
        Pose badFifth = ContactPose(4);
        badFifth.position += new Vector3(0.17f, -0.11f, 0.14f);
        Assert.That(Call(_registration, "CapturePointerPose", badFifth, _tracking), Is.EqualTo(false));
        Assert.That(Status, Does.Contain("FAILED"));
        Assert.That(Get<bool>(_registration, "HasFitFailure"), Is.True);
        string failure = Get<string>(_registration, "FitFailureText");
        Assert.That(failure, Does.Contain("FAILED").And.Contain("planning remains blocked").And.Contain("RMS").And.Contain("limit").And.Contain("Largest mismatch"));
        Assert.That(Get<string[]>(_registration,"PointNames").Any(name => failure.Contains(name)),Is.True,"The rejected fit must identify an actual contact by name.");
        Assert.That(Get<int>(_registration, "CapturedPointCount"), Is.EqualTo(5));
        Assert.That(Get<bool>(_registration, "IsCollecting"), Is.True);
        Assert.That(Get<bool>(_instantiator, "HasRobot"), Is.False);
        AssertCoordinatesBlocked();
        Call(_registration, "UndoLastPoint");
        Assert.That(Get<int>(_registration, "CapturedPointCount"), Is.EqualTo(4));
        Assert.That(Get<bool>(_registration, "HasFitFailure"), Is.False);
        Assert.That(Get<string>(_registration, "FitFailureText"), Is.Empty);
        Assert.That(Capture(4), Is.True, Status);
        AssertPose(ExpectedBaseWorld, BaseWorld);
    }

    [Test]
    public void UndoingTheOnlyPointReleasesItsJointPoseSnapshot()
    {
        BeginWithFeedback();
        Assert.That(Capture(0), Is.True, Status);
        Call(_registration, "UndoLastPoint");
        Assert.That(Get<int>(_registration, "CapturedPointCount"), Is.Zero);
        _angles[0] += 0.15d;
        Feed();
        RecomputeModelPoints();
        for (int i = 0; i < 5; i++) Assert.That(Capture(i), Is.True, Status);
        Assert.That(Get<bool>(_registration, "IsCollecting"), Is.False);
        AssertPose(ExpectedBaseWorld, BaseWorld);
    }

    [Test]
    public void MeasuredTipOffsetRotatesWithEachPointerPoseBeforeTheRigidFit()
    {
        Vector3 offset = new Vector3(0.01f, -0.02f, 0.045f);
        Set(_registration, "_tipOffsetInPointer", offset);
        BeginWithFeedback();
        for (int i = 0; i < 5; i++)
        {
            Pose pointer = ContactPose(i);
            pointer.rotation = Quaternion.Euler(21f + 17f * i, -44f + 39f * i, 63f - 31f * i);
            pointer.position -= pointer.rotation * offset;
            Assert.That(Call(_registration, "CapturePointerPose", pointer, _tracking), Is.EqualTo(true), Status);
        }
        AssertPose(ExpectedBaseWorld, BaseWorld);
        Assert.That(Get<float>(_registration, "MaxErrorMeters"), Is.LessThan(0.00001f));
    }

    [Test]
    public void APlacementTriggerCannotReplaceThePointFitWithAPreviouslyValidatedQrPose()
    {
        PlaceAllFive();
        GameObject priorRobot = _robot;
        Pose priorBase = BaseWorld;
        int priorRevision = Get<int>(_coordinates, "CalibrationRevision");
        Component detector = _instantiator.gameObject.AddComponent(RuntimeType("QRDetector"));
        // Supports the inert harness QR and the actual hardware component's
        // retained tracking-local pose, without starting any marker detection.
        SetPropertyOrField(detector, "TrackingSpace", "_trackingSpace", _tracking);
        Pose syntheticMarkerInTracking = new Pose(new Vector3(-1.3f, 0.2f, -0.7f), Quaternion.Euler(90f, 15f, 0f));
        Pose syntheticMarkerWorld = RobotFrameMath.Compose(new Pose(_tracking.position, _tracking.rotation), syntheticMarkerInTracking);
        PropertyInfo poseProperty = detector.GetType().GetProperty("StabilizedPose");
        if (poseProperty.GetSetMethod(true) != null) poseProperty.SetValue(detector, syntheticMarkerWorld);
        else Set(detector, "_stabilizedTrackingPose", syntheticMarkerInTracking);
        SetPropertyOrField(detector, "HasValidatedPose", "<HasValidatedPose>k__BackingField", true);
        SetPropertyOrField(detector, "ValidationRevision", "<ValidationRevision>k__BackingField", 43);
        SetPropertyOrField(detector, "DetectedPayload", "_lockedPayload", "offline synthetic QR");
        Set(_instantiator, "_QRDetector", detector);
        Set(_instantiator, "_hasValidatedPose", true);
        Set(_instantiator, "_loadedValidationRevision", 43);
        Set(_instantiator, "_loadedPayload", "offline synthetic QR");

        Call(_instantiator, "InstantiateUR");

        Assert.That(Get<GameObject>(_instantiator, "RobotInstance"), Is.SameAs(priorRobot));
        AssertPose(priorBase, BaseWorld);
        Assert.That(Get<bool>(_instantiator, "IsPointPlaced"), Is.True);
        Assert.That(Get<bool>(_registration, "HasPlacement"), Is.True);
        Assert.That(Get<int>(_coordinates, "CalibrationRevision"), Is.EqualTo(priorRevision));
    }

    [Test]
    public void RegeneratedToolRenderersRemainHiddenThroughoutPointCollection()
    {
        PlaceAllFive();
        Component rig = _robot.GetComponent(RuntimeType("RosRobotRig"));
        HashSet<int> oldRenderers = new HashSet<int>();
        foreach (Renderer renderer in _robot.GetComponentsInChildren<Renderer>(true)) oldRenderers.Add(renderer.GetInstanceID());
        Call(_registration, "BeginRegistration");
        Assert.That(Call(rig, "ApplyModel", RosRobotModelCatalog.Load().Find("kenos_sl")), Is.EqualTo(true));
        Renderer[] regenerated = _robot.GetComponentsInChildren<Renderer>(true);
        bool hasNewRenderer = false;
        foreach (Renderer renderer in regenerated)
        {
            hasNewRenderer |= !oldRenderers.Contains(renderer.GetInstanceID());
            Assert.That(renderer.enabled, Is.False, "Model refresh must not reveal the robot during point collection.");
        }
        Assert.That(hasNewRenderer, Is.True, "This scenario must exercise newly generated geometry.");
        Assert.That(Get<bool>(_registration, "IsCollecting"), Is.True);
        AssertCoordinatesBlocked();
    }

    [Test]
    public void PausingAfterACompletedFitInvalidatesTheSessionAndCancelCannotRestoreIt()
    {
        PlaceAllFive();
        InvokePrivate(_registration, "OnApplicationPause", true);
        Assert.That(Get<bool>(_registration, "HasPlacement"), Is.False);
        Assert.That(Get<bool>(_registration, "IsCollecting"), Is.True);
        Assert.That(Get<int>(_registration, "CapturedPointCount"), Is.Zero);
        AssertCoordinatesBlocked();
        Call(_registration, "CancelRegistration");
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.False, "An XR-session-invalidated placement must not be restored by Cancel.");
        AssertCoordinatesBlocked();
    }

    [Test]
    public void NativeTrackingResetDiscardsPicksAndThePriorPlacementWithoutAllowingCancelToRearmIt()
    {
        PlaceAllFive();
        Call(_registration, "BeginRegistration");
        Assert.That(Capture(0), Is.True, Status);
        Call(_registration, "InvalidateTrackingSession", "Synthetic native tracking-space reset.");
        Assert.That(Get<bool>(_registration, "HasPlacement"), Is.False);
        Assert.That(Get<bool>(_registration, "IsCollecting"), Is.True);
        Assert.That(Get<int>(_registration, "CapturedPointCount"), Is.Zero);
        AssertCoordinatesBlocked();
        Call(_registration, "CancelRegistration");
        Assert.That(Get<bool>(_coordinates, "IsInitialized"), Is.False);
        AssertCoordinatesBlocked();
    }

    [Test]
    public void CapturedSpheresMarkTheMeasuredTipAndRemainAttachedToTrackingSpace()
    {
        Vector3 offset = new Vector3(0.01f, -0.02f, 0.045f);
        Call(_registration, "SetTipOffset", offset);
        BeginWithFeedback();
        Pose pointer = ContactPose(0);
        pointer.rotation = Quaternion.Euler(22f, -31f, 64f);
        pointer.position -= pointer.rotation * offset;
        Call(_registration, "SetPointerPose", pointer, _tracking);
        Assert.That(Call(_registration, "CapturePointerPose", pointer, _tracking), Is.EqualTo(true), Status);
        GameObject sphere = CapturedSpheres[0];
        Vector3 measuredInTracking = _tracking.InverseTransformPoint(ContactPose(0).position);
        Assert.That(sphere.transform.parent, Is.SameAs(_tracking));
        Assert.That(Vector3.Distance(sphere.transform.localPosition, measuredInTracking), Is.LessThan(0.000001f));
        Assert.That(sphere.transform.localScale.x, Is.EqualTo(0.012f).Within(0.000001f));
        Assert.That(sphere.GetComponent<Collider>().enabled, Is.False);
        Material template = Resources.Load<Material>("RobotTargetMarkers/Opaque");
        Assert.That(sphere.GetComponent<Renderer>().sharedMaterial.shader, Is.SameAs(template.shader));
        GameObject tip = Field<GameObject>(_registration, "_tipVisual");
        Assert.That(tip.GetComponent<Renderer>().sharedMaterial.shader, Is.SameAs(template.shader));
        Assert.That(Capture(1), Is.True, Status);
        Assert.That(CapturedSpheres[0].GetComponent<Renderer>().sharedMaterial.GetColor("_BaseColor"),
            Is.Not.EqualTo(CapturedSpheres[1].GetComponent<Renderer>().sharedMaterial.GetColor("_BaseColor")));
        _tracking.SetPositionAndRotation(new Vector3(-1f, 0.8f, 2f), Quaternion.Euler(0f, -80f, 0f));
        Assert.That(Vector3.Distance(sphere.transform.position, _tracking.TransformPoint(measuredInTracking)), Is.LessThan(0.000001f));
    }

    [Test]
    public void DuplicateTouchesDoNotAddSpheresAndUndoRemovesOnlyTheLastSphere()
    {
        BeginWithFeedback();
        Assert.That(Capture(0), Is.True, Status);
        GameObject first = CapturedSpheres[0];
        Assert.That(Capture(0), Is.False, "A repeated contact must not produce extra feedback.");
        Assert.That(CapturedSpheres.Count, Is.EqualTo(1));
        Assert.That(Capture(1), Is.True, Status);
        GameObject second = CapturedSpheres[1];
        Material secondMaterial = second.GetComponent<Renderer>().sharedMaterial;
        Call(_registration, "UndoLastPoint");
        Assert.That(CapturedSpheres.Count, Is.EqualTo(1));
        Assert.That(CapturedSpheres[0], Is.SameAs(first));
        Assert.That(second == null, Is.True);
        Assert.That(secondMaterial == null, Is.True);
        Call(_registration, "CancelRegistration");
        Assert.That(CapturedSpheres, Is.Empty);
        Assert.That(first == null, Is.True);
    }

    [Test]
    public void CapturedSpheresSurviveTheFifthTouchAndClearForQrOrANewRegistration()
    {
        PlaceAllFive();
        Assert.That(CapturedSpheres.Count, Is.EqualTo(5));
        foreach (GameObject sphere in CapturedSpheres) Assert.That(sphere.activeSelf, Is.True);
        GameObject[] completed = CapturedSpheres.ToArray();
        Call(_registration, "BeginRegistration");
        Assert.That(CapturedSpheres, Is.Empty);
        foreach (GameObject sphere in completed) Assert.That(sphere == null, Is.True);
        for (int i = 0; i < 5; i++) Assert.That(Capture(i), Is.True, Status);
        Assert.That(CapturedSpheres.Count, Is.EqualTo(5));
        Call(_registration, "UseQrPlacement");
        Assert.That(CapturedSpheres, Is.Empty);
        Assert.That(Get<int>(_registration, "CapturedPointCount"), Is.Zero);
    }

    [Test]
    public void RejectedFitKeepsItsFiveFeedbackSpheresAndTrackingResetClearsThem()
    {
        BeginWithFeedback();
        for (int i = 0; i < 4; i++) Assert.That(Capture(i), Is.True, Status);
        Pose badFifth = ContactPose(4);
        badFifth.position += new Vector3(0.17f, -0.11f, 0.14f);
        Assert.That(Call(_registration, "CapturePointerPose", badFifth, _tracking), Is.EqualTo(false));
        Assert.That(CapturedSpheres.Count, Is.EqualTo(5), "All sampled contacts must remain inspectable after a rejected fit.");
        GameObject[] rejected = CapturedSpheres.ToArray();
        Call(_registration, "InvalidateTrackingSession", "Synthetic tracking reset");
        Assert.That(CapturedSpheres, Is.Empty);
        foreach (GameObject sphere in rejected) Assert.That(sphere == null, Is.True);
    }

    [TestCase(0.003f,0f,0f)]
    [TestCase(0f,0.003f,0f)]
    [TestCase(0f,0f,0.003f)]
    public void PositionNudgeUsesRobotRosAxesWithRotatedBaseAndTrackingAndPreservesThePhysicalSamples(float x,float y,float z)
    {
        PlaceAllFive();
        Pose original = BaseWorld;
        Vector3[] samples = CapturedSpheres.Select(point => point.transform.localPosition).ToArray();
        float rms = Get<float>(_registration,"RmsErrorMeters"), worst = Get<float>(_registration,"MaxErrorMeters");
        int revision = Get<int>(_coordinates,"CalibrationRevision");
        Call(_registration,"BeginPositionAdjustment");
        Assert.That(Get<bool>(_registration,"IsAdjustingPosition"),Is.True);
        AssertCoordinatesBlocked();
        Assert.That((bool)RuntimeType("ToolSelectionController").GetProperty("CanSendRobotCommands").GetValue(null),Is.False);
        Assert.That((bool)RuntimeType("ToolSelectionController").GetProperty("CanPlanPickPlace").GetValue(null),Is.False);
        Vector3 delta = new Vector3(x,y,z);
        Assert.That(Call(_registration,"NudgeRobotPosition",delta),Is.EqualTo(true),Status);
        AssertPose(WithRosOffset(original,delta),BaseWorld);
        Assert.That(Get<Vector3>(_registration,"PositionAdjustmentInRosMeters"),Is.EqualTo(delta));
        AssertLiveJointsUnchanged();
        CollectionAssert.AreEqual(samples,CapturedSpheres.Select(point => point.transform.localPosition).ToArray());
        Assert.That(Get<float>(_registration,"RmsErrorMeters"),Is.EqualTo(rms));
        Assert.That(Get<float>(_registration,"MaxErrorMeters"),Is.EqualTo(worst));
        AssertCoordinatesBlocked();
        Call(_registration,"EndPositionAdjustment");
        Assert.That(Get<bool>(_registration,"IsAdjustingPosition"),Is.False);
        Assert.That(Get<bool>(_coordinates,"IsInitialized"),Is.True);
        Assert.That(Get<int>(_coordinates,"CalibrationRevision"),Is.GreaterThan(revision));
        AssertPose(WithRosOffset(original,delta),BaseWorld);
        SimulatePhysics();
        AssertPose(WithRosOffset(original,delta),BaseWorld);
        AssertLiveJointsUnchanged();
    }

    [Test]
    public void PositionLimitIsEuclideanFromTheOriginalFitAcrossCompletedEditsAndResetReturnsToThatFit()
    {
        PlaceAllFive();
        Pose original = BaseWorld;
        Call(_registration,"BeginPositionAdjustment");
        Assert.That(Call(_registration,"NudgeRobotPosition",new Vector3(.015f,0,0)),Is.EqualTo(true),Status);
        Call(_registration,"EndPositionAdjustment");
        Call(_registration,"BeginPositionAdjustment");
        Pose first = BaseWorld;
        Assert.That(Call(_registration,"NudgeRobotPosition",new Vector3(0,.015f,0)),Is.EqualTo(false));
        Assert.That(Status,Does.Contain("20 mm"));
        AssertPose(first,BaseWorld);
        Assert.That(Get<Vector3>(_registration,"PositionAdjustmentInRosMeters"),Is.EqualTo(new Vector3(.015f,0,0)));
        Assert.That(Call(_registration,"NudgeRobotPosition",new Vector3(0,.010f,0)),Is.EqualTo(true),Status);
        Vector3 combined = new Vector3(.015f,.010f,0);
        AssertPose(WithRosOffset(original,combined),BaseWorld);
        Call(_registration,"EndPositionAdjustment");
        Call(_registration,"BeginPositionAdjustment");
        Assert.That(Call(_registration,"NudgeRobotPosition",new Vector3(0,0,.010f)),Is.EqualTo(false));
        Assert.That(Get<Vector3>(_registration,"PositionAdjustmentInRosMeters"),Is.EqualTo(combined));
        Call(_registration,"ResetPositionAdjustment");
        Assert.That(Get<Vector3>(_registration,"PositionAdjustmentInRosMeters"),Is.EqualTo(Vector3.zero));
        AssertPose(original,BaseWorld);
        AssertCoordinatesBlocked();
        Call(_registration,"EndPositionAdjustment");
        AssertPose(original,BaseWorld);
        Assert.That(Get<bool>(_coordinates,"IsInitialized"),Is.True);
    }

    [Test]
    public void PositionAdjustmentRetainsNewerLiveJointsInsteadOfRestoringTheFirstTouchSnapshot()
    {
        PlaceAllFive();
        Component rig = _robot.GetComponent(RuntimeType("RosRobotRig"));
        double[] latest = Array.ConvertAll(_angles,value => value+0.1d);
        Assert.That(Call(rig,"SetJointPositions",_jointNames,latest),Is.EqualTo(true));
        _angles = latest;
        Pose original = BaseWorld;
        Vector3 delta = new Vector3(.003f,-.002f,.001f);
        Call(_registration,"BeginPositionAdjustment");
        Assert.That(Call(_registration,"NudgeRobotPosition",delta),Is.EqualTo(true),Status);
        AssertLiveJointsUnchanged();
        Call(_registration,"EndPositionAdjustment");
        AssertLiveJointsUnchanged();
        AssertPose(WithRosOffset(original,delta),BaseWorld);
    }

    [Test]
    public void NonfinitePositionNudgesNeverChangeTheOverlayOrItsStoredOffset()
    {
        PlaceAllFive();
        Call(_registration,"BeginPositionAdjustment");
        Pose original = BaseWorld;
        foreach(float invalid in new[] {float.NaN,float.PositiveInfinity,float.NegativeInfinity})
        {
            Assert.That(Call(_registration,"NudgeRobotPosition",new Vector3(invalid,0,0)),Is.EqualTo(false));
            Assert.That(Status,Does.Contain("not finite"));
            AssertPose(original,BaseWorld);
            Assert.That(Get<Vector3>(_registration,"PositionAdjustmentInRosMeters"),Is.EqualTo(Vector3.zero));
        }
        Call(_registration,"CancelPositionAdjustment");
    }

    [Test]
    public void PositionAdjustmentIsRejectedBeforePlacementAndDuringUnresolvedExecution()
    {
        Call(_registration,"BeginPositionAdjustment");
        Assert.That(Get<bool>(_registration,"IsAdjustingPosition"),Is.False);
        Assert.That(Call(_registration,"NudgeRobotPosition",Vector3.one*.001f),Is.EqualTo(false));
        PlaceAllFive();
        Pose original = BaseWorld;
        int revision = Get<int>(_coordinates,"CalibrationRevision");
        SetExecutionBusy(true);
        try
        {
            Call(_registration,"BeginPositionAdjustment");
            Assert.That(Get<bool>(_registration,"IsAdjustingPosition"),Is.False);
            Assert.That(Call(_registration,"NudgeRobotPosition",Vector3.one*.001f),Is.EqualTo(false));
            Assert.That(Get<int>(_coordinates,"CalibrationRevision"),Is.EqualTo(revision));
            AssertPose(original,BaseWorld);
        }
        finally { SetExecutionBusy(false); }
        Call(_registration,"BeginPositionAdjustment");
        SetExecutionBusy(true);
        try
        {
            Assert.That(Call(_registration,"NudgeRobotPosition",Vector3.one*.001f),Is.EqualTo(false));
            Call(_registration,"EndPositionAdjustment");
            Assert.That(Get<bool>(_registration,"IsAdjustingPosition"),Is.True);
            AssertCoordinatesBlocked();
            AssertPose(original,BaseWorld);
        }
        finally { SetExecutionBusy(false); }
        Call(_registration,"CancelPositionAdjustment");
    }

    [TestCase(false)]
    [TestCase(true)]
    public void CancelAndDisableRevertToThePreviouslyCompletedPositionCorrection(bool disable)
    {
        PlaceAllFive();
        Pose original = BaseWorld;
        Vector3 prior = new Vector3(.005f,0,0);
        Call(_registration,"BeginPositionAdjustment");
        Assert.That(Call(_registration,"NudgeRobotPosition",prior),Is.EqualTo(true),Status);
        Call(_registration,"EndPositionAdjustment");
        Call(_registration,"BeginPositionAdjustment");
        Assert.That(Call(_registration,"NudgeRobotPosition",new Vector3(0,.006f,0)),Is.EqualTo(true),Status);
        if(disable) InvokePrivate(_registration,"OnDisable");
        else Call(_registration,"CancelPositionAdjustment");
        Assert.That(Get<bool>(_registration,"IsAdjustingPosition"),Is.False);
        Assert.That(Get<Vector3>(_registration,"PositionAdjustmentInRosMeters"),Is.EqualTo(prior));
        Assert.That(Get<bool>(_coordinates,"IsInitialized"),Is.True);
        AssertPose(WithRosOffset(original,prior),BaseWorld);
        AssertLiveJointsUnchanged();
    }

    [Test]
    public void SceneResetBeforeDisableCannotReinitializeTheOutgoingRobot()
    {
        PlaceAllFive();
        Call(_registration,"BeginPositionAdjustment");
        Assert.That(Call(_registration,"NudgeRobotPosition",new Vector3(.005f,0,0)),Is.EqualTo(true),Status);
        Component state = UnityEngine.Object.FindFirstObjectByType(RuntimeType("RobotStateService")) as Component;
        Call(state,"ResetForSceneChange");
        Call(_coordinates,"ResetForSceneChange");
        int revision = Get<int>(_coordinates,"CalibrationRevision");
        InvokePrivate(_registration,"OnDisable");
        Assert.That(Get<bool>(_registration,"IsAdjustingPosition"),Is.False);
        Assert.That(Get<bool>(_coordinates,"IsInitialized"),Is.False);
        Assert.That(Get<int>(_coordinates,"CalibrationRevision"),Is.EqualTo(revision));
        Assert.That(Get<bool>(state,"IsInitialized"),Is.False);
        Assert.That(Call(state,"GetRobotInstance"),Is.Null);
        AssertCoordinatesBlocked();
    }

    [Test]
    public void TrackingResetDuringAdjustmentDropsTheCorrectionAndRequiresAllFiveContactsAgain()
    {
        PlaceAllFive();
        Call(_registration,"BeginPositionAdjustment");
        Assert.That(Call(_registration,"NudgeRobotPosition",new Vector3(.005f,0,0)),Is.EqualTo(true),Status);
        Call(_registration,"InvalidateTrackingSession","Synthetic adjustment tracking reset.");
        Assert.That(Get<bool>(_registration,"HasPlacement"),Is.False);
        Assert.That(Get<bool>(_registration,"IsAdjustingPosition"),Is.False);
        Assert.That(Get<bool>(_registration,"IsCollecting"),Is.True);
        Assert.That(Get<int>(_registration,"CapturedPointCount"),Is.Zero);
        Assert.That(Get<Vector3>(_registration,"PositionAdjustmentInRosMeters"),Is.EqualTo(Vector3.zero));
        AssertCoordinatesBlocked();
        Feed();
        for(int index=0;index<4;index++)
        {
            Assert.That(Capture(index),Is.True,Status);
            Assert.That(Get<bool>(_registration,"HasPlacement"),Is.False);
            AssertCoordinatesBlocked();
        }
        Assert.That(Capture(4),Is.True,Status);
        Assert.That(Get<Vector3>(_registration,"PositionAdjustmentInRosMeters"),Is.EqualTo(Vector3.zero));
        AssertPose(ExpectedBaseWorld,BaseWorld);
    }

    [Test]
    public void OwnedVisualCleanupDisposesCapturedAndLiveTipObjectsAndMaterials()
    {
        BeginWithFeedback();
        Call(_registration, "SetPointerPose", ContactPose(0), _tracking);
        Assert.That(Capture(0), Is.True, Status);
        GameObject sphere = CapturedSpheres[0];
        Material sphereMaterial = sphere.GetComponent<Renderer>().sharedMaterial;
        GameObject tip = Field<GameObject>(_registration, "_tipVisual");
        Material tipMaterial = tip.GetComponent<Renderer>().sharedMaterial;
        // This inactive EditMode fixture deliberately never enters the player lifecycle.
        // Invoke its resource-cleanup callback explicitly, as with Awake elsewhere in this fixture.
        InvokePrivate(_registration, "OnDestroy");
        UnityEngine.Object.DestroyImmediate(_registration);
        Assert.That(sphere == null, Is.True);
        Assert.That(tip == null, Is.True);
        Assert.That(sphereMaterial == null, Is.True);
        Assert.That(tipMaterial == null, Is.True);
        Assert.That(Resources.Load<Material>("RobotTargetMarkers/Opaque"), Is.Not.Null);
    }

    private List<GameObject> CapturedSpheres => Field<List<GameObject>>(_registration, "_capturedPointVisuals");
    private void BeginWithFeedback() { Feed(); Call(_registration, "BeginRegistration"); }
    private void Feed() => Assert.That(Call(_feedback, "ObserveJointState", _jointNames, _angles), Is.EqualTo(true));
    private void PlaceAllFive()
    {
        BeginWithFeedback();
        for (int i = 0; i < 5; i++) Assert.That(Capture(i), Is.True, Status);
        Feed(); // Keep replacement-session scenarios independent of prefab construction time.
    }
    private bool Capture(int point) => (bool)Call(_registration, "CapturePointerPose", ContactPose(point), _tracking);
    private Pose ContactPose(int point) => new Pose(_tracking.TransformPoint(_baseInTracking.position + _baseInTracking.rotation * _modelPoints[point]), Quaternion.identity);
    private Pose ExpectedBaseWorld => RobotFrameMath.Compose(new Pose(_tracking.position, _tracking.rotation), _baseInTracking);
    private static Pose WithRosOffset(Pose original,Vector3 offset) => new Pose(
        original.position + original.rotation*new Vector3(-offset.y,offset.z,offset.x),original.rotation);
    private void SetExecutionBusy(bool busy) => RuntimeType("RobotExecutionState").GetMethod("SetBusy").Invoke(null,new object[] {_registration,busy});
    private void AssertLiveJointsUnchanged()
    {
        Component rig = _robot.GetComponent(RuntimeType("RosRobotRig"));
        for(int index=0;index<_jointNames.Length;index++)
        {
            object[] query = {_jointNames[index],0d};
            Assert.That(Call(rig,"TryGetJointPosition",query),Is.EqualTo(true));
            Assert.That((double)query[1],Is.EqualTo(_angles[index]).Within(1e-10));
        }
    }
    private Pose BaseWorld
    {
        get
        {
            Transform root = Get<Transform>(_robot.GetComponent(RuntimeType("RosRobotRig")), "BaseLink");
            return new Pose(root.position, root.rotation);
        }
    }
    private string Status => Get<string>(_registration, "StatusText");
    private void RecomputeModelPoints()
    {
        Dictionary<string, double> joints = new Dictionary<string, double>();
        for (int i = 0; i < _jointNames.Length; i++) joints.Add(_jointNames[i], _angles[i]);
        Assert.That(RobotCoverLandmarks.TryGetBasePoints(_model, joints, RobotCoverLandmarks.CreateDefaults(), out _modelPoints, out string error), Is.True, error);
    }
    private void AssertCoordinatesBlocked()
    {
        object[] request = { Pose.identity, default(Pose) };
        Assert.That(_coordinates.GetType().GetMethod("TryConvertWorldPoseToLocalPose").Invoke(_coordinates, request), Is.EqualTo(false));
    }
    private GameObject NewObject(string name) { GameObject value = new GameObject(name); _owned.Add(value); return value; }
    private static Type RuntimeType(string name) => Type.GetType(name + ", Assembly-CSharp", true);
    private static T Get<T>(object owner, string name) => (T)owner.GetType().GetProperty(name).GetValue(owner);
    private static T Field<T>(object owner, string name) => (T)owner.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic).GetValue(owner);
    private static void Set(object owner, string name, object value) => owner.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic).SetValue(owner, value);
    private static object Call(object owner, string name, params object[] args) => owner.GetType().GetMethod(name).Invoke(owner, args);
    private static void InvokePrivate(object owner, string name, params object[] args) => owner.GetType().GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic).Invoke(owner, args);
    private static void SetPropertyOrField(object owner, string property, string field, object value)
    {
        PropertyInfo info = owner.GetType().GetProperty(property);
        if (info != null && info.GetSetMethod(true) != null) info.SetValue(owner, value);
        else Set(owner, field, value);
    }
    private static void AssertPose(Pose expected, Pose actual)
    {
        Assert.That(Vector3.Distance(expected.position, actual.position), Is.LessThan(0.00002f));
        foreach (Vector3 axis in new[] { Vector3.right, Vector3.up, Vector3.forward })
            Assert.That(Vector3.Distance(expected.rotation * axis, actual.rotation * axis), Is.LessThan(0.00002f));
    }
    private static void SimulatePhysics()
    {
        SimulationMode prior = Physics.simulationMode;
        try { Physics.simulationMode = SimulationMode.Script; Physics.SyncTransforms(); Physics.Simulate(0.02f); Physics.Simulate(0.02f); }
        finally { Physics.simulationMode = prior; }
    }
}
