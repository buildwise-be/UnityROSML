using System.Collections.Generic;
using MagicLeap.Android;
using MagicLeap.OpenXR.Subsystems;
using Unity.Collections;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.XR;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.ARSubsystems;
using UnityEngine.XR.Management;

/// <summary>Shows recognized surface boundaries and resolves controller rays against the floor.</summary>
[DisallowMultipleComponent]
public sealed class FloorSurfaceScanner : MonoBehaviour
{
    private const string IdleStatus = "Select Scan floor to recognize the placement surface.";
    private const string SearchingStatus = "Look around the floor until its outline appears.";
    private const string ReadyStatus = "Floor recognized. Aim the controller at the floor.";
    private const float MinimumUpDot = .996f;
    private readonly Dictionary<TrackableId, LineRenderer> _cues = new Dictionary<TrackableId, LineRenderer>();
    private readonly List<XRInputSubsystem> _inputSubsystems = new List<XRInputSubsystem>();
    private XROrigin _origin;
    private ARPlaneManager _manager;
    private bool _managerWasEnabled;
    private PlaneDetectionMode _previousDetectionMode;
    private MLXrPlaneSubsystem.PlanesQuery _previousMagicLeapQuery;
    private Material _floorMaterial;
    private Material _surfaceMaterial;
    private bool _permissionPending;
    private int _scanGeneration;

    public bool IsScanning => _manager != null && _manager.isActiveAndEnabled && _manager.subsystem?.running == true;
    public bool IsScanRequested => _permissionPending || IsScanning;
    public string StatusText { get; private set; } = IdleStatus;

    public void StartScan()
    {
        if (!isActiveAndEnabled || IsScanRequested) return;
        XRPlaneSubsystem subsystem = XRGeneralSettings.Instance?.Manager?.activeLoader?.GetLoadedSubsystem<XRPlaneSubsystem>();
        XRPlaneSubsystemDescriptor descriptor = subsystem?.subsystemDescriptor;
        // Magic Leap SDK 2.6.0 omits supportsHorizontalPlaneDetection from its descriptor,
        // but its OpenXR provider accepts horizontal planes and returns horizontal alignments.
        bool magicLeap = subsystem is MLXrPlaneSubsystem;
        if (descriptor == null || (!magicLeap && !descriptor.supportsHorizontalPlaneDetection) ||
            !descriptor.supportsBoundaryVertices || !descriptor.supportsClassification)
        {
            StatusText = "Classified floor scanning is unavailable on this XR runtime.";
            return;
        }

        XROrigin[] origins = FindObjectsByType<XROrigin>(FindObjectsSortMode.None);
        if (origins.Length != 1 || origins[0].Camera == null || !RobotFrameMath.IsRigidFrame(origins[0].TrackablesParent))
        {
            StatusText = "Floor scanning needs one XR origin with a tracked camera and metre-scale tracking.";
            return;
        }

        _origin = origins[0];
        if (Application.platform == RuntimePlatform.Android && magicLeap &&
            !Permissions.CheckPermission(Permissions.SpatialMapping))
        {
            int generation = ++_scanGeneration;
            _permissionPending = true;
            StatusText = "Allow spatial mapping on the headset to scan the floor.";
            Permissions.RequestPermission(Permissions.SpatialMapping,
                _ => CompletePermissionRequest(generation, true),
                _ => CompletePermissionRequest(generation, false),
                _ => CompletePermissionRequest(generation, false));
            return;
        }

        EnablePlaneManager();
    }

    private void CompletePermissionRequest(int generation, bool granted)
    {
        if (this == null || !isActiveAndEnabled || generation != _scanGeneration || !_permissionPending) return;
        _permissionPending = false;
        if (granted) EnablePlaneManager();
        else StatusText = "Spatial mapping permission was denied. Allow it in headset settings to scan the floor.";
    }

    private void EnablePlaneManager()
    {
        if (_origin == null || !RobotFrameMath.IsRigidFrame(_origin.TrackablesParent))
        {
            StatusText = "XR tracking changed. Select Scan floor again.";
            return;
        }

        _manager = _origin.GetComponent<ARPlaneManager>();
        _managerWasEnabled = _manager != null && _manager.enabled;
        if (_manager == null) _manager = _origin.gameObject.AddComponent<ARPlaneManager>();
        _previousDetectionMode = _manager.requestedDetectionMode;
        _manager.requestedDetectionMode = PlaneDetectionMode.Horizontal | PlaneDetectionMode.Vertical;
        _manager.trackablesChanged.AddListener(OnPlanesChanged);
        _manager.enabled = true;
        if (!IsScanning)
        {
            StopScan();
            StatusText = "The XR plane subsystem could not start. Try Scan floor again.";
            return;
        }

        StatusText = SearchingStatus;
        SubsystemManager.GetSubsystems(_inputSubsystems);
        foreach (XRInputSubsystem subsystem in _inputSubsystems)
            subsystem.trackingOriginUpdated += OnTrackingOriginChanged;
        ConfigureMagicLeapQuery();
    }

    public void StopScan()
    {
        ++_scanGeneration;
        _permissionPending = false;
        foreach (XRInputSubsystem subsystem in _inputSubsystems)
            subsystem.trackingOriginUpdated -= OnTrackingOriginChanged;
        _inputSubsystems.Clear();
        if (_manager != null)
        {
            _manager.trackablesChanged.RemoveListener(OnPlanesChanged);
            _manager.requestedDetectionMode = _previousDetectionMode;
            _manager.enabled = _managerWasEnabled;
        }
        // The SDK cannot clear its static query override. Restore a previous explicit query if present.
        if (_previousMagicLeapQuery.MaxResults > 0) MLXrPlaneSubsystem.Query = _previousMagicLeapQuery;
        _previousMagicLeapQuery = default;
        _manager = null;
        _origin = null;
        foreach (LineRenderer cue in _cues.Values) DisposeCue(cue);
        _cues.Clear();
        StatusText = IdleStatus;
    }

    public bool TryGetFloorHit(Ray ray, float maxDistance, out Vector3 point, out Vector3 normal)
    {
        point = default;
        normal = default;
        if (!IsScanning || _origin == null || _cues.Count == 0 || !(maxDistance > 0f)) return false;

        Transform trackables = _origin.TrackablesParent;
        if (!RobotFrameMath.IsRigidFrame(trackables)) return false;
        Ray sessionRay = new Ray(trackables.InverseTransformPoint(ray.origin), trackables.InverseTransformDirection(ray.direction));
        using var hits = _manager.Raycast(sessionRay, TrackableType.PlaneWithinPolygon, Allocator.Temp);
        ARPlane nearest = null;
        Vector3 nearestPoint = default;
        float nearestDistance = maxDistance;
        foreach (XRRaycastHit hit in hits)
        {
            if (!(hit.distance > 0f && hit.distance <= nearestDistance) || !_cues.ContainsKey(hit.trackableId)) continue;
            ARPlane plane = _manager.GetPlane(hit.trackableId);
            if (!HasTrackedBoundary(plane)) continue;
            nearest = plane;
            nearestPoint = trackables.TransformPoint(hit.pose.position);
            nearestDistance = hit.distance;
        }

        if (!IsFloor(nearest)) return false;
        Vector3 up = nearest.normal;
        if (Vector3.Dot(up, Vector3.up) < 0f) up = -up;
        if (Vector3.Dot(ray.direction, up) >= 0f) return false;
        point = nearestPoint;
        normal = up;
        return true;
    }

    private static bool HasTrackedBoundary(ARPlane plane) => plane != null &&
        plane.trackingState == TrackingState.Tracking && plane.subsumedBy == null && plane.boundary.Length >= 3;

    private static bool IsFloor(ARPlane plane) => plane != null && plane.alignment == PlaneAlignment.HorizontalUp &&
        (plane.classifications & PlaneClassifications.Floor) != 0 && Mathf.Abs(Vector3.Dot(plane.normal, Vector3.up)) >= MinimumUpDot;

    private void OnPlanesChanged(ARTrackablesChangedEventArgs<ARPlane> changes)
    {
        foreach (ARPlane plane in changes.added) UpdateCue(plane);
        foreach (ARPlane plane in changes.updated) UpdateCue(plane);
        foreach (KeyValuePair<TrackableId, ARPlane> plane in changes.removed) RemoveCue(plane.Key);

        StatusText = SearchingStatus;
        foreach (TrackableId id in _cues.Keys)
        {
            if (!IsFloor(_manager.GetPlane(id))) continue;
            StatusText = ReadyStatus;
            break;
        }
    }

    private void UpdateCue(ARPlane plane)
    {
        if (!HasTrackedBoundary(plane))
        {
            if (plane != null) RemoveCue(plane.trackableId);
            return;
        }

        if (!_cues.TryGetValue(plane.trackableId, out LineRenderer cue) || cue == null)
        {
            GameObject marker = new GameObject("Recognized surface outline") { layer = 2 };
            marker.transform.SetParent(plane.transform, false);
            cue = marker.AddComponent<LineRenderer>();
            cue.useWorldSpace = false;
            cue.loop = true;
            cue.shadowCastingMode = ShadowCastingMode.Off;
            cue.receiveShadows = false;
            _cues[plane.trackableId] = cue;
        }

        if (_floorMaterial == null) _floorMaterial = TargetMarkerMaterials.Create(new Color(.25f, .95f, .72f, .9f), true);
        if (_surfaceMaterial == null) _surfaceMaterial = TargetMarkerMaterials.Create(new Color(.55f, .7f, .78f, .45f), true);
        bool floor = IsFloor(plane);
        cue.sharedMaterial = floor ? _floorMaterial : _surfaceMaterial;
        cue.widthMultiplier = floor ? .006f : .004f;
        var boundary = plane.boundary;
        cue.positionCount = boundary.Length;
        for (int i = 0; i < boundary.Length; ++i)
            cue.SetPosition(i, new Vector3(boundary[i].x, .002f, boundary[i].y));
    }

    private void RemoveCue(TrackableId id)
    {
        if (_cues.TryGetValue(id, out LineRenderer cue)) DisposeCue(cue);
        _cues.Remove(id);
    }

    private void Update()
    {
        if (_manager != null && !IsScanning)
        {
            StopScan();
            StatusText = "XR plane tracking stopped. Select Scan floor again.";
        }
    }

    private void ConfigureMagicLeapQuery()
    {
        if (_manager.subsystem is not MLXrPlaneSubsystem || _managerWasEnabled || _origin.Camera == null) return;
        _previousMagicLeapQuery = MLXrPlaneSubsystem.Query;
        // The provider's default minimum area is 10 m², which misses small floors.
        MLXrPlaneSubsystem.Query = new MLXrPlaneSubsystem.PlanesQuery
        {
            Flags = MLXrPlaneSubsystem.MLPlanesQueryFlags.Horizontal | MLXrPlaneSubsystem.MLPlanesQueryFlags.Vertical |
                MLXrPlaneSubsystem.MLPlanesQueryFlags.SemanticAll,
            BoundsCenter = _origin.TrackablesParent.InverseTransformPoint(_origin.Camera.transform.position),
            BoundsRotation = Quaternion.identity,
            BoundsExtents = new Vector3(20f, 20f, 20f),
            MaxResults = 100,
            MinPlaneArea = .25f
        };
    }

    private void OnTrackingOriginChanged(XRInputSubsystem subsystem)
    {
        StopScan();
        StatusText = "XR tracking origin changed. Select Scan floor again.";
    }

    private void OnApplicationPause(bool paused)
    {
        if (paused && IsScanning) StopScan();
    }

    private void OnDisable() => StopScan();

    private void OnDestroy()
    {
        Dispose(_floorMaterial);
        Dispose(_surfaceMaterial);
    }

    private static void DisposeCue(LineRenderer cue)
    {
        if (cue == null) return;
        cue.gameObject.SetActive(false);
        Dispose(cue.gameObject);
    }

    private static void Dispose(Object value)
    {
        if (value == null) return;
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }
}
