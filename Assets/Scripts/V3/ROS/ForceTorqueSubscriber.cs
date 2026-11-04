using System;
using System.Globalization;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>Read-only wrench telemetry, preserving the source ROS axes and reference frame.</summary>
[DisallowMultipleComponent]
public sealed class ForceTorqueSubscriber : MonoBehaviour
{
    public const string DefaultTopicName = "/force_torque_sensor_broadcaster/wrench";
    [SerializeField, Tooltip("geometry_msgs/WrenchStamped topic from the UR driver.")]
    private string _topicName = DefaultTopicName;
    [SerializeField, Min(0.05f), Tooltip("Hide readings when no valid sample has arrived within this many seconds.")]
    private float _maximumSampleAgeSeconds = 1f;

    private ROSConnection _connection;
    private string _subscribedTopic;
    private bool _started;
    private bool _invalidSample;
    private double _receivedAt;
    private bool _observingCommands;
    private long _observedCommandRevision;

    public string TopicName => _topicName;
    public bool HasSample { get; private set; }
    public string FrameId { get; private set; } = "";
    public int StampSeconds { get; private set; }
    public uint StampNanoseconds { get; private set; }
    public double ForceX { get; private set; }
    public double ForceY { get; private set; }
    public double ForceZ { get; private set; }
    public double TorqueX { get; private set; }
    public double TorqueY { get; private set; }
    public double TorqueZ { get; private set; }
    public double ForceMagnitude => Magnitude(ForceX, ForceY, ForceZ);
    public double TorqueMagnitude => Magnitude(TorqueX, TorqueY, TorqueZ);
    public bool HasExecutionPeak { get; private set; }
    public bool IsRecordingExecutionPeak { get; private set; }
    public double PeakForceMagnitude { get; private set; }
    public double PeakTorqueMagnitude { get; private set; }
    public double SampleAgeSeconds => HasSample ? Math.Max(0d, Time.realtimeSinceStartupAsDouble - _receivedAt) : double.PositiveInfinity;
    private double MaximumAge => _maximumSampleAgeSeconds > 0f && !float.IsInfinity(_maximumSampleAgeSeconds)
        ? _maximumSampleAgeSeconds : 1d;
    private bool TransportUnavailable => _connection != null && (!_connection.HasConnectionThread || _connection.HasConnectionError);
    public bool IsFresh => isActiveAndEnabled && HasSample && !_invalidSample && !TransportUnavailable && SampleAgeSeconds <= MaximumAge;

    public string StatusText
    {
        get
        {
            if (!isActiveAndEnabled) return "Force readout paused";
            if (TransportUnavailable) return "ROS disconnected";
            if (_invalidSample) return "Invalid force / torque sample";
            if (!HasSample) return "Waiting for force sensor";
            return IsFresh ? "Live" : "Stale · " + SampleAgeSeconds.ToString("0.0", CultureInfo.InvariantCulture) + " s since sample";
        }
    }

    public string ReadingText => IsFresh
        ? string.Format(CultureInfo.InvariantCulture,
            "F [N]   X {0:0.00}  Y {1:0.00}  Z {2:0.00}\nT [Nm]  X {3:0.00}  Y {4:0.00}  Z {5:0.00}\n|F| {6:0.00} N    |T| {7:0.00} Nm\nFrame: {8}",
            ForceX, ForceY, ForceZ, TorqueX, TorqueY, TorqueZ, ForceMagnitude, TorqueMagnitude, FrameId)
        : "F [N]   X —  Y —  Z —\nT [Nm]  X —  Y —  Z —\n|F| — N    |T| — Nm\nFrame: —";

    private void Start()
    {
        _started = true;
        Subscribe();
    }

    private void OnEnable()
    {
        if (!_observingCommands)
        {
            RobotExecutionState.Changed += SynchronizeExecutionPeak;
            RobotExecutionState.PlanningStarted += SynchronizeExecutionPeak;
            _observingCommands = true;
        }
        SynchronizeExecutionPeak();
        if (_started) Subscribe();
    }

    private void SynchronizeExecutionPeak()
    {
        if (_observedCommandRevision != RobotExecutionState.CommandRevision)
        {
            // A disabled observer also notices Plan/Execute starts it missed.
            _observedCommandRevision = RobotExecutionState.CommandRevision;
            HasExecutionPeak = false;
            PeakForceMagnitude = PeakTorqueMagnitude = 0d;
        }
        IsRecordingExecutionPeak = isActiveAndEnabled && RobotExecutionState.IsBusy;
    }

    private void Update()
    {
        SynchronizeExecutionPeak();
        // A changed Inspector topic or a replaced transport starts a new stream.
        if (_started && (_subscribedTopic != _topicName || _connection == null)) Subscribe();
        if (TransportUnavailable) ClearReading();
    }

    private void Subscribe()
    {
        Unsubscribe();
        ClearReading();
        if (!ValidName(_topicName)) return;
        _connection = ROSService.Instance.GetROSConnection();
        _subscribedTopic = _topicName;
        _connection.Subscribe<WrenchStampedMsg>(_subscribedTopic, OnWrenchReceived);
    }

    private void OnWrenchReceived(WrenchStampedMsg message)
    {
        if (isActiveAndEnabled) ObserveWrench(message);
    }

    /// <summary>Copy a sample without changing ROS coordinates, zeroing the sensor or commanding the robot.</summary>
    public bool ObserveWrench(WrenchStampedMsg message)
    {
        if (message?.header?.stamp == null || message.wrench?.force == null || message.wrench.torque == null ||
            !ValidName(message.header.frame_id) || message.header.frame_id.Length > 128 ||
            message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000 ||
            !Finite(message.wrench.force.x) || !Finite(message.wrench.force.y) || !Finite(message.wrench.force.z) ||
            !Finite(message.wrench.torque.x) || !Finite(message.wrench.torque.y) || !Finite(message.wrench.torque.z) ||
            !Finite(Magnitude(message.wrench.force.x, message.wrench.force.y, message.wrench.force.z)) ||
            !Finite(Magnitude(message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z)))
        {
            _invalidSample = true;
            return false;
        }

        ForceX = message.wrench.force.x;
        ForceY = message.wrench.force.y;
        ForceZ = message.wrench.force.z;
        TorqueX = message.wrench.torque.x;
        TorqueY = message.wrench.torque.y;
        TorqueZ = message.wrench.torque.z;
        FrameId = message.header.frame_id;
        StampSeconds = (int)message.header.stamp.sec;
        StampNanoseconds = message.header.stamp.nanosec;
        _receivedAt = Time.realtimeSinceStartupAsDouble;
        _invalidSample = false;
        HasSample = true;
        // Capture at sensor delivery rate rather than at the gauge refresh rate.
        // The pre-execution cached reading never seeds a new execution's peak.
        if (isActiveAndEnabled) SynchronizeExecutionPeak();
        if (IsRecordingExecutionPeak && IsFresh)
        {
            PeakForceMagnitude = HasExecutionPeak ? Math.Max(PeakForceMagnitude, ForceMagnitude) : ForceMagnitude;
            PeakTorqueMagnitude = HasExecutionPeak ? Math.Max(PeakTorqueMagnitude, TorqueMagnitude) : TorqueMagnitude;
            HasExecutionPeak = true;
        }
        return true;
    }

    private static bool ValidName(string value)
    {
        if (string.IsNullOrWhiteSpace(value) || value == "/") return false;
        foreach (char c in value) if (char.IsWhiteSpace(c) || char.IsControl(c)) return false;
        return true;
    }

    private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    private static double Magnitude(double x, double y, double z)
    {
        double scale = Math.Max(Math.Abs(x), Math.Max(Math.Abs(y), Math.Abs(z)));
        if (scale == 0d) return 0d;
        return scale * Math.Sqrt((x / scale) * (x / scale) + (y / scale) * (y / scale) + (z / scale) * (z / scale));
    }

    private void ClearReading()
    {
        HasSample = false;
        _invalidSample = false;
        FrameId = "";
    }

    private void Unsubscribe()
    {
        if (_connection != null && _subscribedTopic != null) _connection.Unsubscribe(_subscribedTopic);
        _subscribedTopic = null;
        _connection = null;
    }

    private void StopObservingCommands()
    {
        if (_observingCommands)
        {
            RobotExecutionState.Changed -= SynchronizeExecutionPeak;
            RobotExecutionState.PlanningStarted -= SynchronizeExecutionPeak;
            _observingCommands = false;
        }
        IsRecordingExecutionPeak = false;
    }

    private void OnApplicationPause(bool paused) { if (paused) ClearReading(); }
    private void OnDisable() { StopObservingCommands(); Unsubscribe(); ClearReading(); }
    private void OnDestroy() { StopObservingCommands(); Unsubscribe(); ClearReading(); }
}
