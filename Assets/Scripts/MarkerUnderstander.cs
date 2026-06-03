using MagicLeap.OpenXR.Features.MarkerUnderstanding;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using TMPro;
using UnityEditor;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.Serialization;
using UnityEngine.XR.OpenXR;

public class MarkerUnderstander : MonoBehaviour
{
    [Header("Debug")]
    [SerializeField]
    [Tooltip("If debugging, some UI info will show.")]
    private bool debugging = false;
    [SerializeField]
    [Tooltip("The Canvas to show debug info.")]
    private Canvas debugMenu;
    private TextMeshProUGUI markerFeatureDetails; 
    private TextMeshProUGUI markerDetails;
    private TextMeshProUGUI PrefabInstantiatedTextBox;
    private TextMeshProUGUI PrefabPlacedTextBox;

    [Header("Detection")]
    [SerializeField]
    [Tooltip("The visual that shows when a QR is detected, to get a visual confirmation of the QR detection.")]
    private GameObject markerVisualPrefab;


    [HideInInspector]
    public UnityEvent<Pose, GameObject> OnUR16eInstantiatedAndPlaced;
    private bool _hasUR16eBeenInstantiated = false;
    private bool _hasUR16eBeenPlaced = false;
    [SerializeField]
    private GameObject _ur16ePrefab;
    private GameObject UR16eInstance;

    private HashSet<GameObject> detectorVisuals = new();
    private MagicLeapMarkerUnderstandingFeature markerFeature;
    private MarkerDetectorSettings markerDetectorSettings;
    private MarkerDetector currentDetector;

    [Header("Marker Detection")]
    [SerializeField]
    [Tooltip("Number of frames to average the QR code pose over for smoother detection.")]
    private int poseAveragingFrameCount = 50;
    [SerializeField]
    [Tooltip("The width and length of the marker expected, in meters.")]
    private float _markerSize = 0.087f;
    [SerializeField]
    [Tooltip("The offset between the base_link and the center of the QR1, in meters and in base_link reference frame.")]
    private Vector3 _marker1Offset = new Vector3(0.4194f, 0.5f, -0.2305f);
    [SerializeField]
    [Tooltip("The offset between the base_link and the center of the QR2, in meters and in base_link reference frame.")]
    private Vector3 _marker2Offset = new Vector3(0.60031f, 0.5f, 0.17385f);

    [SerializeField]
    [Tooltip("Reference to the main camera. If null, will try to find Camera.main.")]
    private Camera mainCamera;
    // Store world-space poses for averaging (MarkerPose is already in world coordinates)
    private Queue<Pose> poseHistory = new Queue<Pose>();

    [Header("Debug")]
    [SerializeField]
    [Tooltip("The textbox used for debugging purpose.")]
    private TextMeshProUGUI _debugText;

    private GameObject markerInstance;

    

    private void Awake()
    {
        OnUR16eInstantiatedAndPlaced = new UnityEvent<Pose, GameObject>();
    }

    private void Start()
    {
        InitDebugMenu();

        markerFeature = OpenXRSettings.Instance.GetFeature<MagicLeapMarkerUnderstandingFeature>();
        detectorVisuals = new HashSet<GameObject>();
        
        // Find main camera if not assigned
        if (mainCamera == null)
        {
            mainCamera = Camera.main;
            if (mainCamera == null)
            {
                Debug.LogWarning("MarkerUnderstander: Main camera not found. Pose averaging may not work correctly.");
            }
        }
        
#if !UNITY_EDITOR
        CreateDetector();
#endif
    }

    private void InitDebugMenu()
    {
        debugMenu.gameObject.SetActive(debugging);

        var textBoxes = debugMenu.gameObject.GetComponentsInChildren<TextMeshProUGUI>();
        foreach (var textBox in textBoxes)
        {
            if (textBox.name == "MarkerFeatureDetails")
            {
                markerFeatureDetails = textBox;
            }
            else if (textBox.name == "MarkerDetails")
            {
                markerDetails = textBox;
            }
            else if (textBox.name == "PrefabInstanceStatus")
            {
                PrefabInstantiatedTextBox = textBox;
            }
            else if (textBox.name == "PrefabPlacementStatus")
            {
                PrefabPlacedTextBox = textBox;
            }
        }

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
            markerFeature.CreateMarkerDetector(markerDetectorSettings);
        }
    }

    private void ClearVisuals()
    {
        foreach (var visual in detectorVisuals)
            Destroy(visual.gameObject);
        detectorVisuals.Clear();
    }

    private void AddPoseToHistory(Pose worldSpacePose)
    {
        // MarkerPose is already in world coordinates, so we can store it directly
        poseHistory.Enqueue(worldSpacePose);
        
        while (poseHistory.Count > poseAveragingFrameCount)
        {
            poseHistory.Dequeue();
        }
    }

    /// <summary>
    /// Gets the averaged pose by averaging all stored world-space poses.
    /// Since MarkerPose is already in world coordinates, we can average directly.
    /// </summary>
    private Pose? GetAveragedPose()
    {
        if (poseHistory.Count == 0)
        {
            return null;
        }

        // Average position in world space
        Vector3 averagedPosition = Vector3.zero;
        foreach (var pose in poseHistory)
        {
            averagedPosition += pose.position;
        }
        averagedPosition /= poseHistory.Count;

        // Average rotation in world space (proper quaternion averaging)
        Quaternion averagedRotation = AverageQuaternions(poseHistory.Select(p => p.rotation).ToList());

        return new Pose(averagedPosition, averagedRotation);
    }

    private Quaternion AverageQuaternions(List<Quaternion> quaternions)
    {
        if (quaternions.Count == 0)
        {
            return Quaternion.identity;
        }

        if (quaternions.Count == 1)
        {
            return quaternions[0];
        }

        // Normalize all quaternions and ensure they're on the same hemisphere
        List<Quaternion> normalizedQuaternions = new List<Quaternion>();
        Quaternion firstQuaternion = quaternions[0];
        normalizedQuaternions.Add(firstQuaternion);

        for (int i = 1; i < quaternions.Count; i++)
        {
            Quaternion q = quaternions[i];
            // Flip quaternion if it's on the opposite hemisphere
            if (Quaternion.Dot(firstQuaternion, q) < 0)
            {
                q = new Quaternion(-q.x, -q.y, -q.z, -q.w);
            }
            normalizedQuaternions.Add(q);
        }

        // Average quaternion components
        float x = 0, y = 0, z = 0, w = 0;
        foreach (var q in normalizedQuaternions)
        {
            x += q.x;
            y += q.y;
            z += q.z;
            w += q.w;
        }

        float count = normalizedQuaternions.Count;
        Quaternion averaged = new Quaternion(x / count, y / count, z / count, w / count);
        return averaged.normalized;
    }

    private void UpdateDebugMenu()
    {
        if (markerFeature != null)
        {
            StringBuilder markerFeatureDetailsText = new();
            var markerDetectors = markerFeature.MarkerDetectors;
            MarkerDetectorStatus[] statuses = new MarkerDetectorStatus[markerDetectors.Count];
            markerFeatureDetailsText.Append("Marker detector(s) info:");
            for (int i = 0; i < markerDetectors.Count; i++)
            {
                markerFeatureDetailsText.Append($"\nMarker {i+1}");
                markerFeatureDetailsText.Append("\nType: " + markerDetectors[i].Settings.MarkerType);
                markerFeatureDetailsText.Append("\nProfile: " + markerDetectors[i].Settings.MarkerDetectorProfile);
                markerFeatureDetailsText.Append("\nStatus: " + markerDetectors[i].Status);
            }
            markerFeatureDetails.text = markerFeatureDetailsText.ToString();

            if (markerFeature.MarkerDetectors != null && currentDetector !=null)
            {
                StringBuilder markerDetailsText = new();
                markerDetailsText.Append($"There are {markerFeature.MarkerDetectors.Count} Marker Detectors.");
                try
                {
                    var pose = currentDetector.Data[0].MarkerPose;
                    if (currentDetector.Data[0].MarkerPose != null)
                    {
                        markerDetailsText.Append("\nRaw marker pose: " + currentDetector.Data[0].MarkerPose.Value);
                        markerDetailsText.Append("\nMarker string: " + currentDetector.Data[0].MarkerString);
                        markerDetailsText.Append("\nMarker reprojection error: " + currentDetector.Data[0].ReprojectionErrorMeters);
                        
                        // Show averaged pose info
                        var averagedPose = GetAveragedPose();
                        if (averagedPose.HasValue)
                        {
                            markerDetailsText.Append($"\nAveraged pose (over {poseHistory.Count}/{poseAveragingFrameCount} frames): " + averagedPose.Value);
                        }
                        else
                        {
                            markerDetailsText.Append($"\nAveraged pose: Collecting frames ({poseHistory.Count}/{poseAveragingFrameCount})");
                        }
                    }
                }
                catch 
                {
                    markerDetailsText.Append("\nNo Marker Detector Data yet.");
                }
                markerDetails.text = markerDetailsText.ToString();
            }

            StringBuilder prefabInstanceStatusText = new();
            StringBuilder prefabPlacementStatusText = new();
            if (_hasUR16eBeenInstantiated)
            {
                prefabInstanceStatusText.Append("UR16e has been instantiated!");
                PrefabInstantiatedTextBox.color = Color.green;
            }
            else
            {
                prefabInstanceStatusText.Append("UR16e hasn't been instantiated yet!");
                PrefabInstantiatedTextBox.color = Color.red;
            }
            if (_hasUR16eBeenPlaced)
            {
                prefabPlacementStatusText.Append("UR16e has been placed!");
                PrefabPlacedTextBox.color = Color.green;
            }
            else
            {
                prefabPlacementStatusText.Append("UR16e hasn't been placed yet!");
                PrefabPlacedTextBox.color = Color.red;
            }
            PrefabInstantiatedTextBox.text = prefabInstanceStatusText.ToString();
            PrefabPlacedTextBox.text = prefabPlacementStatusText.ToString();
        }
    }

    private void Update()
    {
        if (debugging)
        {
            UpdateDebugMenu();
        }

        if (!markerFeature)
        {
            return;
        }
        markerFeature.UpdateMarkerDetectors();
        if (markerFeature.MarkerDetectors.Count == 0)
        {
            // Clear pose history when marker is lost
            poseHistory.Clear();
            return;
        }
        currentDetector = markerFeature.MarkerDetectors[0];

        bool markerDetected = false;
        for (int i = 0; i < currentDetector.Data.Count; i++)
        {
            if (currentDetector.Data[i].MarkerPose != null)
            {
                markerDetected = true;
                // Add current pose to history for averaging
                AddPoseToHistory(currentDetector.Data[i].MarkerPose.Value);
                InstantiateOrPlaceUR16e();
            }
        }

        // Clear pose history if marker is not detected in this frame
        if (!markerDetected)
        {
            poseHistory.Clear();
        }
    }

    private void InstantiateOrPlaceUR16e()
    {
        // Get averaged pose if we have enough frames
        Pose? averagedPose = GetAveragedPose();
        if (!averagedPose.HasValue)
        {
            return; // Not enough frames collected yet
        }
        Pose markerPose = averagedPose.Value;

        if (!_hasUR16eBeenInstantiated)
        {
            if (!_ur16ePrefab)
            {
                return;
            }
            if (currentDetector.Status == MarkerDetectorStatus.Ready && poseHistory.Count >= poseAveragingFrameCount)
            {
                UR16eInstance = Instantiate(_ur16ePrefab);
                _hasUR16eBeenInstantiated = true;
                UR16eInstance.SetActive(false);
                UR16eInstance.transform.SetPositionAndRotation(markerPose.position, markerPose.rotation);
                UR16eInstance.SetActive(true);
            }
        }
        else if (!_hasUR16eBeenPlaced)
        {
            Debug.Log("!_hasUR16eBeenPlaced");
            Debug.Log($"Averaged marker pose: {markerPose}");
            if (markerPose.position != Vector3.zero && poseHistory.Count >= poseAveragingFrameCount)
            {
                Debug.Log("markerPose.position != Vector3.zero");
                Quaternion correction = Quaternion.Euler(-90, 0, 0);  // Y+ becomes Z+
                Quaternion finalRotation = markerPose.rotation * correction;

                if (!markerInstance)
                {
                    markerInstance = Instantiate(markerVisualPrefab);
                }
                markerInstance.transform.position = markerPose.position;// - markerPose.forward * 0.01f;
                //markerInstance.transform.rotation = finalRotation;
                markerInstance.transform.rotation = markerPose.rotation;

                /*
                Vector3 markerOffset = -markerPose.right * (0.19f/2.0f + 0.282f + 0.087f/2.0f) 
                + markerPose.up * (0.182f - 0.087f/2.0f + 0.19f/2.0f) + markerPose.forward * 0.5f;
                */

                Vector3 markerOffset = markerPose.right * _marker1Offset.x
                + markerPose.up * _marker1Offset.z + markerPose.forward * _marker1Offset.y;

                foreach (ArticulationBody articulationBody in UR16eInstance.GetComponentsInChildren<ArticulationBody>())
                {
                    if (articulationBody.gameObject.name == "base_link")
                    {
                        articulationBody.TeleportRoot(markerPose.position + markerOffset, finalRotation);
                        break;
                    }
                }
                Pose computedPose = new Pose(markerPose.position + markerOffset, finalRotation);

                OnUR16eInstantiatedAndPlaced?.Invoke(computedPose, UR16eInstance);
                
                // Notify RobotStateService
                RobotStateService.Instance.InitializeRobot(computedPose, UR16eInstance);
                
                //_targetPose.transform.SetPositionAndRotation(markerPose.position + markerOffset + new Vector3(0.5f,1,0.5f), Quaternion.identity);
                _hasUR16eBeenPlaced = true;
            }
        }
    }

    private void OnDestroy()
    {
        ClearVisuals();
        markerFeature.DestroyAllMarkerDetectors();
    }
}