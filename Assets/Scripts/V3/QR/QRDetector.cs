using MagicLeap.OpenXR.Features.MarkerUnderstanding;
using UnityEngine;
using UnityEngine.XR.OpenXR;
using UnityEngine.Events;
using System;
using System.Collections.Generic;

public class QRDetector : MonoBehaviour
{
    [SerializeField]
    [Tooltip("The width and length of the marker expected, in meters.")]
    private float _markerSize = 0.087f;

    [SerializeField, Tooltip("Optional exact decoded QR payload. Empty requires one unambiguous visible QR, then pins that payload until Rescan.")]
    private string _expectedMarkerPayload = "";

    [SerializeField]
    [Tooltip("The GameObject to instantiate upon stable QR detected. Can be left empty.")]
    private GameObject _markerPositionPrefab;

    [SerializeField]
    [Tooltip("Optional tracking-space transform. Defaults to the XR Origin camera offset.")]
    private Transform _trackingSpace;

    [SerializeField, Tooltip("Show labelled SDK marker axes after validation. Blue +Z points into the printed face.")]
    private bool _showMarkerAxes = true;

    private AudioSource _qrAudio;
    private GameObject _markerPositionObject;
    private GameObject _markerAxesObject;
    private IQRPoseStabilizer _QRStabilizer;
    private MagicLeapMarkerUnderstandingFeature markerFeature;
    private MarkerDetector _markerDetector;
    private Pose _stabilizedTrackingPose = Pose.identity;
    private bool _isQRDetectingRequired = true;
    private bool _hasStarted;
    private string _lockedPayload;
    private string _visiblePayloads = "";
    private float _reportedMarkerSize;
    private int _acceptedSamples;
    public UnityEvent<Pose> OnQRValidated = new UnityEvent<Pose>();
    public event Action StateChanged;

    public string StatusText { get; private set; } = "Looking for QR.";
    public string DetectedPayload => _lockedPayload ?? "";
    public string ExpectedMarkerPayload => _expectedMarkerPayload ?? "";
    public string VisiblePayloads => _visiblePayloads;
    public float MarkerSizeMeters => _markerSize;
    public float ReportedMarkerSizeMeters => _reportedMarkerSize;
    public int ValidationRevision { get; private set; }

    public bool HasValidatedPose { get; private set; }
    public Transform TrackingSpace
    {
        get
        {
            XRTrackingSpaceResolver.TryResolve(ref _trackingSpace);
            return _trackingSpace;
        }
    }

    // Re-evaluate from the retained tracking-space pose if the XR Origin moves.
    public Pose StabilizedPose => HasValidatedPose &&
        RobotFrameMath.TryTrackingToWorld(TrackingSpace, _stabilizedTrackingPose, out Pose worldPose)
        ? worldPose : new Pose();

    private void Awake()
    {
        if (OnQRValidated == null) OnQRValidated = new UnityEvent<Pose>();
        _qrAudio = GetComponent<AudioSource>();
    }

    private void Start()
    {
        _hasStarted = true;
        InitializeDetection();
    }

    private void OnEnable()
    {
        if (_hasStarted) InitializeDetection();
    }

    private void InitializeDetection()
    {
        EnsureStabilizer();
        if (_QRStabilizer == null) return;
        SetStatus(WaitingStatus());
        if (_isQRDetectingRequired && _markerDetector == null) CreateDetector();
    }

    private void EnsureStabilizer()
    {
        IQRPoseStabilizer previous = _QRStabilizer;
        _QRStabilizer = GetComponent<IQRPoseStabilizer>();
        if (previous != null && previous != _QRStabilizer) previous.PoseStabilized.RemoveListener(SetStabilizedPose);
        if (_QRStabilizer == null)
        {
            Debug.LogError("QRDetector: No QR stabilizer specified.", this);
            return;
        }

        _QRStabilizer.PoseStabilized.RemoveListener(SetStabilizedPose);
        _QRStabilizer.PoseStabilized.AddListener(SetStabilizedPose);
    }

    // The SDK and stabilizer supply tracking-local poses; listeners receive world poses.
    public void SetStabilizedPose(Pose trackingPose)
    {
        if (!_isQRDetectingRequired || !XRTrackingSpaceResolver.TryResolve(ref _trackingSpace) ||
            !RobotFrameMath.TryTrackingToWorld(_trackingSpace, trackingPose, out Pose worldPose))
        {
            _QRStabilizer?.ResetStabilization();
            SetStatus("QR pose cannot be placed: a valid unit-scale tracking frame is required.");
            return;
        }

        _stabilizedTrackingPose = trackingPose;
        HasValidatedPose = true;
        _isQRDetectingRequired = false;
        unchecked { ValidationRevision++; }
        if (_markerPositionPrefab != null)
        {
            _markerPositionObject = Instantiate(_markerPositionPrefab, worldPose.position, worldPose.rotation);
            _markerPositionObject.transform.SetParent(_trackingSpace, true);
        }
        if (_showMarkerAxes)
        {
            _markerAxesObject = new GameObject("Validated QR axes");
            _markerAxesObject.transform.SetPositionAndRotation(worldPose.position, worldPose.rotation);
            _markerAxesObject.transform.SetParent(_trackingSpace, true);
            _markerAxesObject.AddComponent<QRMarkerAxes>().Build();
        }
        Debug.Log("QRDetector: accepted payload='" + DisplayPayload(DetectedPayload) + "', configured side=" +
            _markerSize.ToString("F6") + " m, reported side=" + _reportedMarkerSize.ToString("F6") +
            " m, tracking position=" + trackingPose.position.ToString("F6") + ", tracking quaternion=" +
            trackingPose.rotation.ToString("F6") + ", world position=" + worldPose.position.ToString("F6") +
            ", world quaternion=" + worldPose.rotation.ToString("F6") + ".", this);
        if (_qrAudio != null) _qrAudio.Play();
        SetStatus("QR '" + DisplayPayload(DetectedPayload) + "' ready. Trigger places or realigns the robot.");
        OnQRValidated.Invoke(worldPose);
    }

    public void ResetDetection()
    {
        RobotCoordinateService coordinates = FindFirstObjectByType<RobotCoordinateService>();
        if (coordinates != null) coordinates.InvalidateCalibration();
        RobotStateService state = FindFirstObjectByType<RobotStateService>();
        if (state != null) state.IsReadyForPlanning = false;
        HasValidatedPose = false;
        _stabilizedTrackingPose = Pose.identity;
        _isQRDetectingRequired = true;
        _lockedPayload = null;
        _visiblePayloads = "";
        _reportedMarkerSize = 0f;
        _acceptedSamples = 0;
        _QRStabilizer?.ResetStabilization();
        DisposeVisualization(_markerPositionObject);
        _markerPositionObject = null;
        DisposeVisualization(_markerAxesObject);
        _markerAxesObject = null;
        ReleaseDetector();
        SetStatus(WaitingStatus());
        if (_hasStarted && isActiveAndEnabled) CreateDetector();
    }

    public void Rescan() => ResetDetection();

    public void SetExpectedMarkerPayload(string payload)
    {
        _expectedMarkerPayload = payload ?? "";
        ResetDetection();
    }

    /// <summary>Process one detector snapshot. Also permits hardware-free identity/window regression tests.</summary>
    public bool ObserveMarkers(string[] payloads, Pose?[] trackingPoses, float[] reportedLengths)
    {
        if (!_isQRDetectingRequired) return false;
        EnsureStabilizer();
        if (_QRStabilizer == null) return false;
        if (payloads == null || trackingPoses == null || reportedLengths == null ||
            payloads.Length != trackingPoses.Length || payloads.Length != reportedLengths.Length)
        {
            ResetWindow();
            SetStatus("QR observation data is incomplete.");
            return false;
        }

        List<string> visible = new List<string>();
        List<int> valid = new List<int>();
        for (int i = 0; i < payloads.Length; i++)
        {
            if (string.IsNullOrEmpty(payloads[i])) continue;
            if (!visible.Contains(payloads[i])) visible.Add(payloads[i]);
            if (trackingPoses[i].HasValue && RobotFrameMath.IsValidPose(trackingPoses[i].Value)) valid.Add(i);
        }
        visible.Sort(StringComparer.Ordinal);
        _visiblePayloads = string.Join(", ", visible.ConvertAll(DisplayPayload));
        string desired = !string.IsNullOrEmpty(_expectedMarkerPayload) ? _expectedMarkerPayload : _lockedPayload;
        if (string.IsNullOrEmpty(desired))
        {
            if (valid.Count != 1 || visible.Count != 1)
            {
                ResetWindow();
                SetStatus(valid.Count > 1 || visible.Count > 1 ? "Several QR markers are visible. Show one marker or set an expected payload. Seen: " + _visiblePayloads : WaitingStatus());
                return false;
            }
            desired = payloads[valid[0]];
            _lockedPayload = desired;
        }
        List<int> matches = valid.FindAll(index => payloads[index] == desired);
        int visibleCopies = Array.FindAll(payloads, payload => payload == desired).Length;
        if (matches.Count != 1 || visibleCopies != 1)
        {
            ResetWindow();
            SetStatus(visibleCopies > 1 ? "Several copies of QR '" + DisplayPayload(desired) + "' are visible. Show one copy." : WaitingStatus());
            return false;
        }
        _lockedPayload = desired;
        int selected = matches[0];
        _reportedMarkerSize = reportedLengths[selected];
        SimpleQRPoseStabilizer simple = _QRStabilizer as SimpleQRPoseStabilizer;
        int restarts = simple != null ? simple.DiscontinuityCount : 0;
        _QRStabilizer.FeedPoseStabilizationAlgorithm(trackingPoses[selected]);
        _acceptedSamples = simple != null ? simple.SamplesCollected : _acceptedSamples + 1;
        if (HasValidatedPose) return true;
        string progress = simple != null ? _acceptedSamples + "/" + simple.RequiredSamples : _acceptedSamples.ToString();
        SetStatus((simple != null && simple.DiscontinuityCount != restarts ? "QR moved; restarting. " : "Stabilizing ") +
            "'" + DisplayPayload(desired) + "' (" + progress + "). Configured side: " + (_markerSize * 1000f).ToString("F0") + " mm.");
        return true;
    }

    private void ResetWindow()
    {
        _QRStabilizer?.ResetStabilization();
        _acceptedSamples = 0;
    }

    private string WaitingStatus()
    {
        string desired = !string.IsNullOrEmpty(_expectedMarkerPayload) ? _expectedMarkerPayload : _lockedPayload;
        string status = string.IsNullOrEmpty(desired) ? "Looking for one QR marker" : "Waiting for QR '" + DisplayPayload(desired) + "'";
        return status + ". Configured side: " + (_markerSize * 1000f).ToString("F0") + " mm." +
            (string.IsNullOrEmpty(_visiblePayloads) ? "" : " Seen: " + _visiblePayloads);
    }

    private static string DisplayPayload(string payload)
    {
        string text = (payload ?? "").Replace('<', '‹').Replace('>', '›').Replace('\r', ' ').Replace('\n', ' ');
        return text.Length <= 64 ? text : text.Substring(0, 61) + "...";
    }

    private void SetStatus(string status)
    {
        if (StatusText == status) return;
        StatusText = status;
        StateChanged?.Invoke();
    }

    private void CreateDetector()
    {
        if (!(_markerSize > 0f) || float.IsInfinity(_markerSize))
        {
            SetStatus("QR scanning is blocked: marker side length must be positive and finite.");
            Debug.LogError("QRDetector: Marker size must be a positive number of meters.", this);
            return;
        }

        OpenXRSettings settings = OpenXRSettings.Instance;
        markerFeature = settings != null ? settings.GetFeature<MagicLeapMarkerUnderstandingFeature>() : null;
        if (markerFeature == null || !markerFeature.enabled)
        {
            SetStatus("QR scanning is unavailable: enable the OpenXR marker understanding feature.");
            Debug.LogError("QRDetector: No enabled marker understanding feature found.", this);
            return;
        }

        MarkerDetectorSettings detectorSettings = new MarkerDetectorSettings();
        detectorSettings.MarkerDetectorProfile = MarkerDetectorProfile.Default;
        detectorSettings.MarkerType = MarkerType.QR;
        detectorSettings.QRSettings.QRLength = _markerSize;
        detectorSettings.QRSettings.EstimateQRLength = false;
        _markerDetector = markerFeature.CreateMarkerDetector(detectorSettings);
    }

    private void Update()
    {
        if (!markerFeature || !_isQRDetectingRequired || _QRStabilizer == null || _markerDetector == null ||
            !XRTrackingSpaceResolver.TryResolve(ref _trackingSpace) || !RobotFrameMath.IsRigidFrame(_trackingSpace) ||
            markerFeature.MarkerDetectors == null || markerFeature.MarkerDetectors.Count == 0)
        {
            return;
        }

        markerFeature.UpdateMarkerDetectors();
        if (_markerDetector.Status != MarkerDetectorStatus.Ready)
        {
            // Pending is normal between asynchronous detector snapshots, not a lost observation.
            if (_markerDetector.Status == MarkerDetectorStatus.Error)
            {
                ResetWindow();
                SetStatus("QR detector reported an error. " + WaitingStatus());
            }
            return;
        }
        var data = _markerDetector.Data;
        int count = data != null ? data.Count : 0;
        string[] payloads = new string[count];
        Pose?[] poses = new Pose?[count];
        float[] lengths = new float[count];
        for (int i = 0; i < count; i++)
        {
            payloads[i] = data[i].MarkerString;
            poses[i] = data[i].MarkerPose;
            lengths[i] = data[i].MarkerLength;
        }
        ObserveMarkers(payloads, poses, lengths);
    }

    private void OnDisable()
    {
        _QRStabilizer?.PoseStabilized.RemoveListener(SetStabilizedPose);
        ReleaseDetector();
    }

    private void ReleaseDetector()
    {
        if (_markerDetector != null && markerFeature != null)
        {
            // Avoid releasing a detector already destroyed by the OpenXR feature.
            var detectors = markerFeature.MarkerDetectors;
            if (detectors != null)
            {
                for (int i = 0; i < detectors.Count; i++)
                {
                    if (detectors[i] == _markerDetector)
                    {
                        markerFeature.DestroyMarkerDetector(_markerDetector);
                        break;
                    }
                }
            }
        }
        _markerDetector = null;
    }

    private void OnDestroy()
    {
        DisposeVisualization(_markerPositionObject);
        DisposeVisualization(_markerAxesObject);
    }

    private static void DisposeVisualization(GameObject visual)
    {
        if (visual == null) return;
        if (Application.isPlaying) Destroy(visual);
        else DestroyImmediate(visual);
    }
}
