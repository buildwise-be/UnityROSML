using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Threading;
using NUnit.Framework;
using UnityEngine;

/// <summary>Exercise the actual ROS-status callback without a connection or robot commands.</summary>
public class TcpPlanningControllerTests
{
    private GameObject _owner;
    private Component _controller;
    private TcpPlanReview _review;
    private readonly List<string> _sentStopKeys = new List<string>();

    [SetUp]
    public void SetUp()
    {
        _owner = new GameObject("Offline TCP callback test");
        _owner.SetActive(false); // Start must not establish a ROS connection.
        _controller = _owner.AddComponent(RuntimeType("TcpPlanningController"));
        _review = (TcpPlanReview)Field(_controller, "_review").GetValue(_controller);
        _review.Begin("123:456");
    }

    [TearDown]
    public void TearDown() => UnityEngine.Object.DestroyImmediate(_owner);

    [Test]
    public void PlanningProgressRetainsTokenAndReadyResultWaitsForMatchingPreview()
    {
        Status("123:456", false, "planning", "Planning started");
        Assert.That(_review.IsPending, Is.True);
        Assert.That(_review.RequestId, Is.EqualTo("123:456"));
        Assert.That(_review.HasSuccessfulResult, Is.False);
        Assert.That(_review.CanExecute, Is.False);

        Status("123:456", true, "ready", "Ready");
        Assert.That(_review.IsPending, Is.True);
        Assert.That(_review.RequestId, Is.EqualTo("123:456"));
        Assert.That(_review.HasSuccessfulResult, Is.True);
        Assert.That(_review.CanExecute, Is.False, "A result without its visible preview cannot authorize execution.");
        Assert.That((bool)_controller.GetType().GetProperty("CanExecute").GetValue(_controller), Is.False);
    }

    [TestCase("failed")]
    [TestCase("invalidated")]
    public void MatchingFailureConsumesTokenAndAbsorbsLateSuccess(string phase)
    {
        Status("123:456", false, "planning", "Started");
        Status("123:456", false, phase, "Changed or failed");
        Assert.That(_review.IsPending, Is.False);
        Assert.That(_review.RequestId, Is.Null);

        Status("123:456", true, "ready", "Late result");
        Assert.That(_review.IsPending, Is.False);
        Assert.That(_review.HasSuccessfulResult, Is.False);
        Assert.That(_review.CanExecute, Is.False);
    }

    [Test]
    public void OtherClientsStatusCannotChangePendingReview()
    {
        Status("other:789", false, "failed", "Another client");
        Status("other:789", true, "ready", "Another client");
        Assert.That(_review.RequestId, Is.EqualTo("123:456"));
        Assert.That(_review.IsPending, Is.True);
        Assert.That(_review.HasSuccessfulResult, Is.False);
    }

    [Test]
    public void StopDuringCapabilityCheckRevokesThePendingLocalRequest()
    {
        Field(_controller, "_checkingCapability").SetValue(_controller, true);
        int generation = (int)Field(_controller, "_generation").GetValue(_controller);

        StopPlanning();

        Assert.That((int)Field(_controller, "_generation").GetValue(_controller), Is.GreaterThan(generation));
        Assert.That(GetMember(_controller, "IsPlanning"), Is.False);
        Status("123:456", true, "ready", "Late result");
        Assert.That(_review.HasSuccessfulResult, Is.False);
    }

    [TestCase(false)]
    [TestCase(true)]
    public void StopRevokesPreviewAndWaitsForTheMatchingPlannerRelease(bool disconnectBeforeStop)
    {
        var transport = new GameObject("Offline TCP stop transport");
        transport.SetActive(false);
        using var offlineConnectionFlag = new CancellationTokenSource();
        Component connection = null;
        FieldInfo connectionError = null;
        bool previousError = false;
        object stop = null;
        try
        {
            connection = transport.AddComponent(RuntimeType("Unity.Robotics.ROSTCPConnector.ROSConnection"));
            var request = _controller.GetType().GetProperty("StopRequest", BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(request, Is.Not.Null, "TCP needs separate ROS stop state that survives clearing its review.");
            stop = request.GetValue(_controller);
            stop.GetType().GetMethod("Begin").Invoke(stop, new object[] { connection, "123:456" });
            // Observe the real connector's local publish callback without starting a connection thread.
            object topic = connection.GetType().GetMethod("GetTopic").Invoke(connection, new object[] { "/mtc/cancel_planning" });
            MethodInfo subscribe = topic.GetType().GetMethod("AddSubscriber");
            MethodInfo capture = GetType().GetMethod(nameof(CaptureStopKey), BindingFlags.Instance | BindingFlags.NonPublic);
            subscribe.Invoke(topic, new object[] { Delegate.CreateDelegate(subscribe.GetParameters()[0].ParameterType, this, capture) });
            connectionError = connection.GetType().GetField("m_HasConnectionError", BindingFlags.Static | BindingFlags.NonPublic);
            previousError = (bool)connectionError.GetValue(null);
            connectionError.SetValue(null, disconnectBeforeStop);
            Field(connection, "m_ConnectionThreadCancellation").SetValue(connection, offlineConnectionFlag);
            _sentStopKeys.Clear();

            StopPlanning();
            Status("123:456", true, "ready", "Late result");
            Assert.That(_review.HasSuccessfulResult, Is.False);
            Assert.That(GetMember(_controller, "IsPlanning"), Is.True);
            if (disconnectBeforeStop)
            {
                Assert.That(_sentStopKeys, Is.Empty);
                Assert.That(GetMember(_controller, "CanStopPlanning"), Is.False);
                connectionError.SetValue(null, false);
                stop.GetType().GetField("_retryAt", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(stop, 0f);
                StopPlanning();
            }
            Assert.That(_sentStopKeys, Is.EqualTo(new[] { "tcp/123:456" }), "Stop can arrive before admission.");
            ControlStatus(stop, "tcp/other", "started");
            ControlStatus(stop, "tcp/123:456", "started");
            Assert.That(_sentStopKeys, Is.EqualTo(new[] { "tcp/123:456" }));
            ControlStatus(stop, "tcp/other", "cancelled");
            Assert.That(GetMember(_controller, "IsPlanning"), Is.True);

            ControlStatus(stop, "tcp/123:456", "cancelled");
            Assert.That(GetMember(_controller, "IsPlanning"), Is.False);
            Assert.That(GetMember(_controller, "CanExecute"), Is.False);
            Assert.That((string)GetMember(_controller, "StatusText"), Does.Contain("stopped"));
        }
        finally
        {
            if (connection != null) Field(connection, "m_ConnectionThreadCancellation").SetValue(connection, null);
            connectionError?.SetValue(null, previousError);
            (stop as IDisposable)?.Dispose();
            UnityEngine.Object.DestroyImmediate(transport);
        }
    }

    private void CaptureStopKey(object message) => _sentStopKeys.Add((string)GetMember(message, "data"));

    [TestCase(null)]
    [TestCase("")]
    public void UnsolicitedProgressWithNoRequestCannotReplaceCurrentStatus(string requestId)
    {
        _review.Invalidate();
        string previous = (string)_controller.GetType().GetProperty("StatusText").GetValue(_controller);
        Status(requestId, false, "planning", "No active request");
        Assert.That((string)_controller.GetType().GetProperty("StatusText").GetValue(_controller), Is.EqualTo(previous));
        Assert.That(_review.IsPending, Is.False);
    }

    [TestCase(1d)]
    [TestCase(-1d)]
    public void PoseComparisonNormalizesFloatQuaternionsButRejectsChangedTargets(double sign)
    {
        MethodInfo same = _controller.GetType().GetMethod("SamePose", BindingFlags.Static | BindingFlags.NonPublic);
        Type poseType = same.GetParameters()[0].ParameterType;
        object a = Activator.CreateInstance(poseType), b = Activator.CreateInstance(poseType);
        SetMember(GetMember(a, "header"), "frame_id", "ur_base_link");
        SetMember(GetMember(b, "header"), "frame_id", "ur_base_link");
        object poseA = GetMember(a, "pose"), poseB = GetMember(b, "pose");
        object rotationA = GetMember(poseA, "orientation"), rotationB = GetMember(poseB, "orientation");
        const double scale = 0.99999994d;
        SetMember(rotationA, "w", scale);
        SetMember(rotationB, "w", scale * sign);
        Assert.That((bool)same.Invoke(null, new[] { a, b }), Is.True,
            "An identical Unity float rotation, including its negative quaternion, must retain the plan.");

        object positionB = GetMember(poseB, "position");
        SetMember(positionB, "x", 0.00002d);
        Assert.That((bool)same.Invoke(null, new[] { a, b }), Is.False, "A target translated by 20 micrometres changed.");
        SetMember(positionB, "x", 0d);
        SetMember(rotationB, "z", Math.Sin(Math.PI / 360d) * scale * sign);
        SetMember(rotationB, "w", Math.Cos(Math.PI / 360d) * scale * sign);
        Assert.That((bool)same.Invoke(null, new[] { a, b }), Is.False, "A target rotated by one degree changed.");
    }

    [Test]
    public void HistoricalResetListenerCannotOverwriteVisibleTcpBlockedReason()
    {
        Component input = _owner.AddComponent(RuntimeType("URInputManager"));
        Field(input, "_tcpPlanningController").SetValue(input, _controller);
        input.GetType().GetMethod("UseTcpPreview").Invoke(input, null);
        GameObject panel = new GameObject("Legacy task status", typeof(RectTransform));
        panel.SetActive(false);
        try
        {
            Type textType = RuntimeType("TMPro.TextMeshProUGUI");
            Component text = panel.AddComponent(textType);
            Component ui = panel.AddComponent(RuntimeType("MTCPlanUIManager"));
            Field(ui, "_input").SetValue(ui, input);
            Field(ui, "_tcpPlanning").SetValue(ui, _controller);
            Invoke(ui, "Start");
            _review.Invalidate();
            _controller.GetType().GetMethod("Plan").Invoke(_controller, null);
            string blocked = (string)_controller.GetType().GetProperty("StatusText").GetValue(_controller);
            Assert.That(blocked, Does.Contain("disconnected"));
            ui.GetType().GetMethod("ResetPlanStatusPanel").Invoke(ui, null);
            Assert.That((string)textType.GetProperty("text").GetValue(text), Is.EqualTo(blocked));
        }
        finally { UnityEngine.Object.DestroyImmediate(panel); }
    }

    private void Status(string id, bool success, string phase, string message)
    {
        MethodInfo callback = _controller.GetType().GetMethod("OnStatus", BindingFlags.Instance | BindingFlags.NonPublic);
        Type messageType = callback.GetParameters()[0].ParameterType;
        string json = "{\"request_id\":" + (id == null ? "null" : "\"" + id + "\"") + ",\"overall_success\":" + (success ? "true" : "false") +
            ",\"phase\":\"" + phase + "\",\"message\":\"" + message + "\"}";
        object value = Activator.CreateInstance(messageType, new object[] { json });
        callback.Invoke(_controller, new[] { value });
    }

    private void StopPlanning()
    {
        MethodInfo stop = _controller.GetType().GetMethod("StopPlanning");
        Assert.That(stop, Is.Not.Null, "TCP needs a planning stop action.");
        stop.Invoke(_controller, null);
    }

    private static void ControlStatus(object stop, string key, string status)
    {
        MethodInfo callback = stop.GetType().GetMethod("OnStatus", BindingFlags.Instance | BindingFlags.NonPublic);
        Type messageType = callback.GetParameters()[0].ParameterType;
        object message = Activator.CreateInstance(messageType,
            new object[] { "{\"key\":\"" + key + "\",\"status\":\"" + status + "\",\"reason\":\"Stopped\"}" });
        callback.Invoke(stop, new[] { message });
    }

    private static Type RuntimeType(string name) => AppDomain.CurrentDomain.GetAssemblies()
        .Select(assembly => assembly.GetType(name)).FirstOrDefault(type => type != null)
        ?? throw new InvalidOperationException("Missing runtime type " + name);
    private static FieldInfo Field(Component component, string name) => component.GetType().GetField(name, BindingFlags.Instance | BindingFlags.NonPublic);
    private static void Invoke(Component component, string name) => component.GetType().GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic).Invoke(component, null);
    private static object GetMember(object instance, string name) => instance.GetType().GetField(name) is FieldInfo field
        ? field.GetValue(instance) : instance.GetType().GetProperty(name).GetValue(instance);
    private static void SetMember(object instance, string name, object value)
    {
        if (instance.GetType().GetField(name) is FieldInfo field) field.SetValue(instance, value);
        else instance.GetType().GetProperty(name).SetValue(instance, value);
    }
}
