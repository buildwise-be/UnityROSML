using MagicLeap.OpenXR.Features.MarkerUnderstanding;
using UnityEngine;
using UnityEngine.XR.OpenXR;
using UnityEngine.Events;

public class QRDetector : MonoBehaviour
{
    [SerializeField]
    [Tooltip("The width and length of the marker expected, in meters.")]
    private float _markerSize = 0.087f;

    [SerializeField]
    [Tooltip("The GameObject to instantiate upon stable QR detected. Can be left empty.")]
    private GameObject _markerPositionPrefab;

    private AudioSource _qrAudio;

    private GameObject _markerPositionObject = null;

    private IQRPoseStabilizer _QRStabilizer;

    private MagicLeapMarkerUnderstandingFeature markerFeature;
    private MarkerDetectorSettings markerDetectorSettings;
    private MarkerDetector _markerDetector;
    private Pose _stabilizedPose = Pose.identity;
    private bool _isQRDetectingRequired = true;
    public UnityEvent<Pose> OnQRValidated;

    public Pose StabilizedPose { get { return _stabilizedPose; } private set { } }

    private void Awake()
    {
        OnQRValidated = new UnityEvent<Pose>();
    }

    void Start()
    {
        _qrAudio = GetComponent<AudioSource>();
        _QRStabilizer = GetComponent<IQRPoseStabilizer>();
        if (_QRStabilizer == null)
        {
            Debug.LogError("No QR stabilizer specified. Aborting QR detection.");
            return;
        }
        markerFeature = OpenXRSettings.Instance.GetFeature<MagicLeapMarkerUnderstandingFeature>();
        CreateDetector();

        _QRStabilizer.PoseStabilized.AddListener(SetStabilizedPose);
    }

    public void SetStabilizedPose(Pose p)
    {
        Debug.Log("Calling SetStabilizedPose");
        _stabilizedPose = p;
        _isQRDetectingRequired = false;
        InstantiateQRPoseHelperObject();
    }

    private void InstantiateQRPoseHelperObject()
    {
        if (_markerPositionPrefab != null)
        {
            Debug.Log($"Instantiating prefab at coordinates {_stabilizedPose.position}");
            Debug.Log($"Headset current position is {Camera.main.transform.position}");
            _markerPositionObject = GameObject.Instantiate(_markerPositionPrefab, _stabilizedPose.position, _stabilizedPose.rotation);
            
            if (_qrAudio != null) _qrAudio.Play();

            OnQRValidated?.Invoke(_stabilizedPose);
        }
    }

    public void ResetDetection()
    {
        _stabilizedPose = Pose.identity;
        _isQRDetectingRequired = true;
        _QRStabilizer.ResetStabilization();
        GameObject.Destroy(_markerPositionObject);
    }

    private void CreateDetector()
    {
        //Create a detector based on a subset of settings. For fully customizable detector
        //creation, see the Marker Tracking scene in our Examples project. 
        markerDetectorSettings.MarkerDetectorProfile = MarkerDetectorProfile.Default;
        markerDetectorSettings.MarkerType = MarkerType.QR;
        markerDetectorSettings.QRSettings.QRLength = _markerSize;
        // Do not use length estimation as it can fuck up pose detection
        // (see https://forum.magicleap.cloud/t/scanned-markers-pose-at-world-origin/6026)
        markerDetectorSettings.QRSettings.EstimateQRLength = false;
        if (markerFeature != null)
        {
            _markerDetector = markerFeature.CreateMarkerDetector(markerDetectorSettings);
        }
        else
        {
            Debug.LogError("No marker feature found. Cannot create marker detector!");
        }
        Debug.Log("Detector created");
    }

    void Update()
    {
        if (!markerFeature || !_isQRDetectingRequired)
        {
            return;
        }
        markerFeature.UpdateMarkerDetectors();
        if (markerFeature.MarkerDetectors.Count == 0)
        {
            Debug.LogWarning("Zero MarkerDetectors");
            return;
        }

        if (_markerDetector.Data[0].MarkerPose != null && _markerDetector.Status == MarkerDetectorStatus.Ready)
        {
            Debug.Log("Feeding pose for stabilization");
            _QRStabilizer.FeedPoseStabilizationAlgorithm(_markerDetector.Data[0].MarkerPose);
        }
    }
}