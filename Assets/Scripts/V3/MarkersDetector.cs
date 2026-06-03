using MagicLeap.OpenXR.Features.MarkerUnderstanding;
using System;
using UnityEditorInternal;
using UnityEngine;
using UnityEngine.XR.OpenXR;

public class MarkersDetector : MonoBehaviour
{
    [SerializeField]
    [Tooltip("The width and length of the marker expected, in meters.")]
    private float _markerSize = 0.087f;

    private MagicLeapMarkerUnderstandingFeature markerFeature;
    private MarkerDetectorSettings markerDetectorSettings;
    private MarkerDetector markerDetector;
    private MarkerPose[] markerPoses;

    void Start()
    {
        markerFeature = OpenXRSettings.Instance.GetFeature<MagicLeapMarkerUnderstandingFeature>();
        markerPoses = new MarkerPose[2];
#if !UNITY_EDITOR
        CreateDetector();
#endif
    }

    void Update()
    {
        if (!markerFeature)
        {
            return;
        }
        markerFeature.UpdateMarkerDetectors();
        if (markerFeature.MarkerDetectors.Count == 0)
        {
            return;
        }

        for (int i = 0, n = markerDetector.Data.Count; i < n; i++)
        {
            if (markerDetector.Data[i].MarkerPose != null && markerDetector.Status == MarkerDetectorStatus.Ready)
            {
                UpdateMarkerPose(i);
            }
        }
    }

    private void UpdateMarkerPose(int i)
    {
        throw new NotImplementedException();
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
            markerDetector = markerFeature.CreateMarkerDetector(markerDetectorSettings);
        }
    }  
}

public class MarkerPose
{
    public Vector3 Position
    {
        get
        {
            return _position;
        }
        private set
        {
            _position = value;
        }
    }
    public Quaternion Rotation
    {
        get
        {
            return _rotation;
        }
        private set
        {
            _rotation = value;
        }
    }

    private Vector3 _position;
    private Quaternion _rotation;

    public MarkerPose()
    {
        Position = Vector3.zero;
        Rotation = Quaternion.identity;
    }
}
