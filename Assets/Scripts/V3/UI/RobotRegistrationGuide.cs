using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Geometry-only registration miniature. No cloned scripts, articulation,
/// colliders, ROS transport, or connection to the placed robot's presentation.
/// </summary>
[DisallowMultipleComponent]
public sealed class RobotRegistrationGuide : MonoBehaviour
{
    private sealed class Visual
    {
        public string Link;
        public RosRobotGeometry Geometry;
        public Pose MeshInVisual;
        public Transform Transform;
        public Mesh Mesh;
        public Material[] Dim;
        public Material[] Highlight;
    }

    private sealed class MaterialPair { public Material Dim; public Material Highlight; }
    private readonly List<Visual> _visuals = new List<Visual>();
    private readonly List<Material> _materials = new List<Material>();
    private readonly Dictionary<Material, MaterialPair> _materialPairs = new Dictionary<Material, MaterialPair>();
    private readonly List<Transform> _markers = new List<Transform>();
    private readonly Dictionary<string, double> _lastJoints = new Dictionary<string, double>();
    private RobotPointRegistration _registration;
    private GameObject _referencePrefab;
    private RosRobotModel _model;
    private GameObject _geometry;
    private Material _targetMaterial;
    private Material _otherPointMaterial;
    private MaterialPair _defaultPair;
    private float _maximumLocalSize = 200f;
    private float _nextInspection;
    private bool _lastIllustration;
    private int _lastStep = -1;

    // The measured covers' outward surface normals, in their visual's ROS axes.
    // A step-specific view keeps its rear-cover center visible in the miniature.
    private static readonly Vector3[] CoverNormalsRos = { Vector3.up, Vector3.up, Vector3.back, Vector3.down };
    private static readonly Color TargetColor = new Color(1f, 0.5f, 0.08f);
    private const float TargetDiameterInModel = 0.05f;

    public bool HasVisual { get; private set; }
    public int SelectedPointIndex { get; private set; } = -1;
    public string CurrentPointName { get; private set; } = string.Empty;
    public string HighlightedLinkName { get; private set; } = string.Empty;
    public string PoseStatus { get; private set; } = "Waiting for registration";
    public Vector3 TargetPointInModel { get; private set; }

    /// <summary>Mount on a menu-relative transform. 200 local units become 20 cm on the .001-scale canvas.</summary>
    public void Refresh(RobotPointRegistration registration, GameObject referencePrefab, float maximumLocalSize = 200f)
    {
        _registration = registration;
        if (registration == null || !registration.IsCollecting)
        {
            Hide();
            return;
        }
        if (referencePrefab == null)
        {
            HideGeometry();
            PoseStatus = "Miniature unavailable: robot mesh reference missing";
            return;
        }
        _maximumLocalSize = maximumLocalSize > 0f && !float.IsInfinity(maximumLocalSize)
            ? maximumLocalSize : 200f;
        bool hasJoints = registration.GetGuideState(out RosRobotModel model, out Dictionary<string, double> joints,
            out RobotCoverLandmark[] landmarks, out bool locked);
        if (!hasJoints) joints = IllustrationJoints(model);
        int step = Mathf.Clamp(registration.CapturedPointCount, 0, 3);
        try
        {
            if (_model != model || _referencePrefab != referencePrefab || _geometry == null)
            {
                ClearGeometry();
                _model = model;
                _referencePrefab = referencePrefab;
                BuildGeometry();
            }
            if (landmarks == null || landmarks.Length != 4 || landmarks[step] == null)
                throw new ArgumentException("Four valid cover definitions are required.");
            if (step != _lastStep || !SameJoints(joints) || _lastIllustration != !hasJoints || !HasVisual)
                PoseGeometry(joints, landmarks, step);
            HasVisual = _visuals.Count > 0;
            _geometry.SetActive(HasVisual);
            SelectedPointIndex = step;
            CurrentPointName = landmarks[step].displayName;
            HighlightedLinkName = landmarks[step].linkName;
            PoseStatus = !hasJoints ? "Illustration · waiting for joint feedback" :
                locked ? "Pose locked at first point" : "Live robot pose";
            _lastIllustration = !hasJoints;
            _nextInspection = Time.unscaledTime + 0.25f;
        }
        catch (Exception exception)
        {
            ClearGeometry();
            HideGeometry();
            PoseStatus = "Miniature unavailable: " + exception.Message;
        }
    }

    public void Hide()
    {
        _registration = null;
        HideGeometry();
        PoseStatus = "Waiting for registration";
    }

    private void HideGeometry()
    {
        HasVisual = false;
        SelectedPointIndex = -1;
        if (_geometry != null) _geometry.SetActive(false);
    }

    private void LateUpdate()
    {
        if (_registration == null) return;
        if (!_registration.IsCollecting) { Hide(); return; }
        if (Time.unscaledTime >= _nextInspection)
            Refresh(_registration, _referencePrefab, _maximumLocalSize);
        if (HasVisual && SelectedPointIndex >= 0 && SelectedPointIndex < _markers.Count)
            _markers[SelectedPointIndex].localScale = Vector3.one * TargetDiameterInModel *
                (1.08f + 0.08f * Mathf.Sin(Time.unscaledTime * 4f));
    }

    private void BuildGeometry()
    {
        RosRobotModelMath.Validate(_model);
        _geometry = new GameObject("UR16e registration miniature geometry");
        _geometry.layer = 2; // Ignore Raycast, including the target-center spheres.
        _geometry.transform.SetParent(transform, false);
        RosRobotModel reference = RosRobotModelCatalog.Load().Find("kenos");
        var imported = new Dictionary<string, Transform>();
        foreach (Transform node in _referencePrefab.GetComponentsInChildren<Transform>(true))
            if (!imported.ContainsKey(node.name)) imported.Add(node.name, node);
        foreach (RosRobotLink link in _model.links)
        {
            if (!link.name.StartsWith("ur_", StringComparison.Ordinal) || link.visuals == null) continue;
            for (int index = 0; index < link.visuals.Length; index++)
            {
                RosRobotGeometry geometry = link.visuals[index];
                if (geometry.kind != "mesh") continue;
                RosRobotLink known = reference.FindLink(link.name);
                if (known?.visuals == null || index >= known.visuals.Length ||
                    MeshKey(known.visuals[index].meshUri) != MeshKey(geometry.meshUri))
                    throw new InvalidOperationException("Unsupported arm mesh: " + link.name);
                string importedName = link.name.Substring(3);
                Transform visuals = imported.TryGetValue(importedName, out Transform node) ? node.Find("Visuals") : null;
                if (visuals == null || visuals.childCount != 1) continue;
                Transform origin = visuals.GetChild(0);
                Vector3 geometryScale = geometry.scale == null ? Vector3.one : Abs(RosRobotModelMath.FromRosVector(geometry.scale));
                foreach (MeshFilter filter in origin.GetComponentsInChildren<MeshFilter>(true))
                {
                    MeshRenderer source = filter.GetComponent<MeshRenderer>();
                    if (filter.sharedMesh == null || source == null) continue;
                    var item = new GameObject("Guide link: " + link.name, typeof(MeshFilter), typeof(MeshRenderer));
                    item.layer = 2;
                    item.transform.SetParent(_geometry.transform, false);
                    item.transform.localScale = Vector3.Scale(Divide(filter.transform.lossyScale, origin.lossyScale), geometryScale);
                    item.GetComponent<MeshFilter>().sharedMesh = filter.sharedMesh;
                    Material[] dim = new Material[filter.sharedMesh.subMeshCount];
                    Material[] highlight = new Material[dim.Length];
                    Material[] sourceMaterials = source.sharedMaterials;
                    for (int materialIndex = 0; materialIndex < dim.Length; materialIndex++)
                    {
                        MaterialPair pair = Materials(materialIndex < sourceMaterials.Length ? sourceMaterials[materialIndex] : null);
                        dim[materialIndex] = pair.Dim;
                        highlight[materialIndex] = pair.Highlight;
                    }
                    MeshRenderer renderer = item.GetComponent<MeshRenderer>();
                    renderer.sharedMaterials = dim;
                    renderer.shadowCastingMode = ShadowCastingMode.Off;
                    renderer.receiveShadows = false;
                    _visuals.Add(new Visual {
                        Link = link.name, Geometry = geometry, Mesh = filter.sharedMesh, Transform = item.transform,
                        MeshInVisual = new Pose(Vector3.Scale(origin.InverseTransformPoint(filter.transform.position), geometryScale),
                            Quaternion.Inverse(origin.rotation) * filter.transform.rotation), Dim = dim, Highlight = highlight
                    });
                }
            }
        }
        if (_visuals.Count == 0) throw new InvalidOperationException("UR16e arm meshes are unavailable.");
        _targetMaterial = Own(TargetMarkerMaterials.Create(TargetColor));
        _otherPointMaterial = Own(TargetMarkerMaterials.Create(new Color(0.2f, 0.48f, 0.62f)));
        for (int index = 0; index < 4; index++)
        {
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.name = "Guide cover center " + (index + 1);
            marker.layer = 2;
            marker.transform.SetParent(_geometry.transform, false);
            Collider collider = marker.GetComponent<Collider>();
            collider.enabled = false;
            Dispose(collider);
            marker.GetComponent<Renderer>().shadowCastingMode = ShadowCastingMode.Off;
            marker.GetComponent<Renderer>().receiveShadows = false;
            _markers.Add(marker.transform);
        }
    }

    private void PoseGeometry(Dictionary<string, double> joints, RobotCoverLandmark[] landmarks, int step)
    {
        Dictionary<string, Pose> poses = RosRobotModelMath.ForwardKinematics(_model, joints);
        if (!RobotCoverLandmarks.TryGetBasePoints(_model, joints, landmarks, out Vector3[] points, out string error))
            throw new ArgumentException(error);
        foreach (Visual visual in _visuals)
        {
            Pose pose = RobotFrameMath.Compose(RobotFrameMath.Compose(poses[visual.Link],
                RosRobotModelMath.GeometryPose(visual.Geometry)), visual.MeshInVisual);
            visual.Transform.SetLocalPositionAndRotation(pose.position, pose.rotation);
            visual.Transform.GetComponent<MeshRenderer>().sharedMaterials = visual.Link == landmarks[step].linkName
                ? visual.Highlight : visual.Dim;
        }
        for (int index = 0; index < _markers.Count; index++)
        {
            _markers[index].localPosition = points[index];
            _markers[index].localScale = Vector3.one * (index == step ? TargetDiameterInModel : 0.018f);
            _markers[index].GetComponent<Renderer>().sharedMaterial = index == step ? _targetMaterial : _otherPointMaterial;
        }
        RobotCoverLandmark target = landmarks[step];
        RosRobotGeometry geometry = _model.FindLink(target.linkName).visuals[target.visualIndex];
        Vector3 normal = poses[target.linkName].rotation * RosRobotModelMath.GeometryPose(geometry).rotation *
            RosRobotModelMath.FromRosVector(ToArray(CoverNormalsRos[step]));
        Vector3 up = Mathf.Abs(Vector3.Dot(normal.normalized, Vector3.up)) > 0.94f ? Vector3.forward : Vector3.up;
        Quaternion view = Quaternion.Inverse(Quaternion.LookRotation(-normal, up));
        Bounds bounds = RotatedBounds(view);
        float scale = _maximumLocalSize / Mathf.Max(bounds.size.x, bounds.size.y, bounds.size.z, 0.001f);
        _geometry.transform.localRotation = view;
        _geometry.transform.localScale = Vector3.one * scale;
        _geometry.transform.localPosition = -bounds.center * scale;
        TargetPointInModel = points[step];
        _lastStep = step;
        _lastJoints.Clear();
        foreach (KeyValuePair<string, double> joint in joints) _lastJoints[joint.Key] = joint.Value;
    }

    private Bounds RotatedBounds(Quaternion rotation)
    {
        bool found = false;
        Bounds total = default;
        foreach (Visual item in _visuals)
        {
            Bounds mesh = item.Mesh.bounds;
            Matrix4x4 matrix = Matrix4x4.Rotate(rotation) * Matrix4x4.TRS(item.Transform.localPosition,
                item.Transform.localRotation, item.Transform.localScale);
            for (int corner = 0; corner < 8; corner++)
            {
                Vector3 point = matrix.MultiplyPoint3x4(mesh.center + Vector3.Scale(mesh.extents,
                    new Vector3((corner & 1) == 0 ? -1f : 1f, (corner & 2) == 0 ? -1f : 1f, (corner & 4) == 0 ? -1f : 1f)));
                if (!found) { total = new Bounds(point, Vector3.zero); found = true; }
                else total.Encapsulate(point);
            }
        }
        return total;
    }

    private MaterialPair Materials(Material source)
    {
        if (source == null && _defaultPair != null) return _defaultPair;
        if (source != null && _materialPairs.TryGetValue(source, out MaterialPair existing)) return existing;
        Color original = source != null && source.HasProperty("_BaseColor") ? source.GetColor("_BaseColor") :
            source != null && source.HasProperty("_Color") ? source.GetColor("_Color") : Color.gray;
        var pair = new MaterialPair {
            Dim = Own(TargetMarkerMaterials.Create(Color.Lerp(original, new Color(0.11f, 0.19f, 0.24f), 0.75f))),
            Highlight = Own(TargetMarkerMaterials.Create(Color.Lerp(original, new Color(0.3f, 0.85f, 1f), 0.75f)))
        };
        if (source == null) _defaultPair = pair; else _materialPairs.Add(source, pair);
        return pair;
    }

    private bool SameJoints(Dictionary<string, double> joints)
    {
        if (joints == null || joints.Count != _lastJoints.Count) return false;
        foreach (KeyValuePair<string, double> joint in joints)
            if (!_lastJoints.TryGetValue(joint.Key, out double previous) || Math.Abs(joint.Value - previous) > 0.00001d) return false;
        return true;
    }

    private static Dictionary<string, double> IllustrationJoints(RosRobotModel model)
    {
        var joints = new Dictionary<string, double>();
        if (model?.goldenCases != null)
            foreach (RosRobotGoldenCase candidate in model.goldenCases)
                if (candidate.name == "bent")
                {
                    for (int index = 0; index < candidate.jointNames.Length; index++) joints[candidate.jointNames[index]] = candidate.jointPositions[index];
                    return joints;
                }
        // A recognizable bent UR pose for live descriptions without golden cases.
        double[] pose = { 0.6d, -1.2d, 1.5d, -0.8d, 1.1d, -0.4d };
        int jointIndex = 0;
        if (model?.links != null)
            foreach (RosRobotLink link in model.links)
                if (RosRobotModelMath.IsRevolute(link)) joints[link.jointName] = pose[Mathf.Min(jointIndex++, pose.Length - 1)];
        return joints;
    }

    private Material Own(Material material) { _materials.Add(material); return material; }
    private static string MeshKey(string uri) => (uri ?? "").Replace('\\', '/').Replace("package://", "");
    private static float[] ToArray(Vector3 value) => new[] { value.x, value.y, value.z };
    private static Vector3 Abs(Vector3 value) => new Vector3(Mathf.Abs(value.x), Mathf.Abs(value.y), Mathf.Abs(value.z));
    private static Vector3 Divide(Vector3 a, Vector3 b) => new Vector3(a.x / b.x, a.y / b.y, a.z / b.z);

    private void ClearGeometry()
    {
        if (_geometry != null) { _geometry.SetActive(false); Dispose(_geometry); }
        _geometry = null;
        foreach (Material material in _materials) Dispose(material);
        _materials.Clear();
        _materialPairs.Clear();
        _markers.Clear();
        _visuals.Clear();
        _lastJoints.Clear();
        _defaultPair = null;
        _lastStep = -1;
        HasVisual = false;
    }

    private static void Dispose(UnityEngine.Object value)
    {
        if (value == null) return;
        if (Application.isPlaying) Destroy(value); else DestroyImmediate(value);
    }

    private void OnDisable() => Hide();
    private void OnDestroy() => ClearGeometry();
}
