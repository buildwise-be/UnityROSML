using System;
using System.Globalization;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.RclInterfaces;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>Tracks a ROS planning job separately from permission to review its result.</summary>
internal sealed class MtcPlanningStop : IDisposable
{
    public const string CapabilityParameter = "mtc_planning_stop_enabled";
    private const string CommandTopic = "/mtc/cancel_planning";
    private static long _lastRequestTicks;

    private readonly string _scope, _statusTopic;
    private readonly Action _changed;
    private readonly Action<string, string, bool> _finished;
    private ROSConnection _connection;
    private string _key;
    private bool _sendFailed;
    private float _retryAt;

    public bool IsActive => _key != null;
    public bool StopRequested { get; private set; }
    public bool CanRequestStop => IsActive && (!StopRequested || (Connected && Time.realtimeSinceStartup >= _retryAt));
    private bool Connected => _connection != null && _connection.HasConnectionThread && !_connection.HasConnectionError;

    public string Feedback
    {
        get
        {
            if (!StopRequested) return null;
            if (!Connected) return "ROS disconnected. Reconnect to confirm the planning stop.";
            if (_sendFailed) return "Could not send the stop request. Press Stop Planning to retry.";
            return "Stopping planning. Waiting for ROS to release the planner...";
        }
    }

    public MtcPlanningStop(string scope, Action changed, Action<string, string, bool> finished)
    {
        _scope = scope;
        _statusTopic = "/mtc/planning_control/" + scope;
        _changed = changed;
        _finished = finished;
    }

    public void Bind(ROSConnection connection)
    {
        if (connection == null) throw new ArgumentNullException(nameof(connection));
        if (_connection == connection) return;
        if (_connection != null) _connection.Unsubscribe(_statusTopic);
        if (connection.GetTopic(CommandTopic)?.IsPublisher != true)
            connection.RegisterPublisher<StringMsg>(CommandTopic);
        connection.Subscribe<StringMsg>(_statusTopic, OnStatus);
        _connection = connection;
    }

    public void Begin(ROSConnection connection, string requestId)
    {
        if (string.IsNullOrEmpty(requestId)) throw new ArgumentException("Planning request ID is required.", nameof(requestId));
        if (IsActive) throw new InvalidOperationException("A ROS planning request is already tracked.");
        Bind(connection);
        _key = _scope + "/" + requestId;
    }

    public bool RequestStop()
    {
        if (!CanRequestStop) return false;
        StopRequested = true;
        Send();
        _changed?.Invoke();
        return true;
    }

    public void Tick()
    {
        if (!StopRequested || _retryAt <= 0f || Time.realtimeSinceStartup < _retryAt) return;
        _retryAt = 0f;
        _changed?.Invoke();
    }

    private void Send()
    {
        _retryAt = Time.realtimeSinceStartup + 5f;
        _sendFailed = false;
        if (!Connected) return;
        try { _connection.Publish(CommandTopic, new StringMsg(_key)); }
        catch (Exception) { _sendFailed = true; }
    }

    private void OnStatus(StringMsg message)
    {
        if (!IsActive || message == null) return;
        ControlStatus status;
        try { status = JsonUtility.FromJson<ControlStatus>(message.data); }
        catch (Exception) { return; }
        if (status == null || status.key != _key) return;
        switch (status.status)
        {
            case "stopping":
                StopRequested = true;
                _retryAt = Time.realtimeSinceStartup + 5f;
                _sendFailed = false;
                _changed?.Invoke();
                return;
            case "cancelled":
            case "finished":
            case "rejected":
                bool wasRequested = StopRequested;
                Reset();
                _finished?.Invoke(status.status, status.reason, wasRequested);
                _changed?.Invoke();
                return;
        }
    }

    /// <summary>Use only if publishing the tracked plan request failed.</summary>
    public void ForgetUnsent() => Reset();

    private void Reset()
    {
        _key = null;
        StopRequested = _sendFailed = false;
        _retryAt = 0;
    }

    public void Dispose()
    {
        if (_connection != null) _connection.Unsubscribe(_statusTopic);
        _connection = null;
        Reset();
    }

    public static bool IsSupported(ParameterValueMsg[] values, int index) =>
        values != null && index >= 0 && index < values.Length &&
        values[index] != null && values[index].type == 1 && values[index].bool_value;

    public static void RequestCapabilities(ROSConnection connection, string service, string[] required,
        Func<bool> isCurrent, Action<GetParametersResponse> completed)
    {
        var parameters = new string[required.Length + 1];
        Array.Copy(required, parameters, required.Length);
        parameters[required.Length] = CapabilityParameter;

        connection.SendServiceMessage<GetParametersResponse>(service, new GetParametersRequest(parameters), response =>
        {
            if (!isCurrent()) return;
            if (response?.values != null && response.values.Length != 0)
            {
                completed(response);
                return;
            }

            // Older ROS nodes may return an empty result for the whole request when one name is undeclared.
            connection.SendServiceMessage<GetParametersResponse>(service, new GetParametersRequest(required), legacyResponse =>
            {
                if (isCurrent()) completed(legacyResponse);
            });
        });
    }

    public static TimeMsg NewRequestStamp()
    {
        long ticks = Math.Max(DateTime.UtcNow.Ticks - 621355968000000000L, _lastRequestTicks + 1);
        _lastRequestTicks = ticks;
#if ROS2
        int seconds = (int)(ticks / 10000000L);
#else
        uint seconds = (uint)(ticks / 10000000L);
#endif
        return new TimeMsg(seconds, (uint)((ticks % 10000000L) * 100L));
    }

    public static string RequestId(TimeMsg stamp) => stamp == null ? null :
        stamp.sec.ToString(CultureInfo.InvariantCulture) + ":" + stamp.nanosec.ToString(CultureInfo.InvariantCulture);

    [Serializable]
    private sealed class ControlStatus { public string key, status, reason; }
}
