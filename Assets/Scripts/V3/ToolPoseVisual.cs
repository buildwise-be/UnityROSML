using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Geometry-only view of the selected tool, anchored at its ROS TCP. It never
/// instantiates an articulation, an imported controller, or a ROS connection.
/// </summary>
public sealed class ToolPoseVisual : MonoBehaviour
{
    private static GameObject _referencePrefab;
    private static int _referenceRevision;
    private static Mesh _kenosSlDetail;
    private static RosRobotModel _meshReference;
    private readonly List<Material> _materials = new List<Material>();
    private GameObject _geometry;
    private RosRobotRig _source;
    private RosRobotModel _model;
    private RosRobotRig _explicitSource;
    private RosRobotModel _explicitModel;
    private Color _tint = Color.cyan;
    private float _opacity = 0.65f;
    private float _nextInspection;
    private int _appliedReferenceRevision = -1;
    private bool _dirty = true;

    public event Action VisualChanged;
    public bool HasVisual { get; private set; }
    public string Tool => _model != null ? _model.tool : "";
    public string StatusText { get; private set; } = "Waiting for tool model";
    public Bounds TcpLocalBounds { get; private set; } = new Bounds(Vector3.zero, Vector3.one * 0.04f);

    public static void SetReferencePrefab(GameObject prefab)
    {
        if (_referencePrefab == prefab) return;
        _referencePrefab = prefab;
        unchecked { _referenceRevision++; }
    }

    public static ToolPoseVisual Attach(Transform tcpTarget, Color tint, float opacity = 0.65f)
    {
        if (tcpTarget == null) return null;
        ToolPoseVisual view = tcpTarget.GetComponent<ToolPoseVisual>();
        if (view == null) view = tcpTarget.gameObject.AddComponent<ToolPoseVisual>();
        view.Configure(tint, opacity);
        view.Refresh();
        return view;
    }

    public void Configure(Color tint, float opacity = 0.65f)
    {
        float alpha = float.IsNaN(opacity) || float.IsInfinity(opacity) ? 0.65f : Mathf.Clamp(opacity, 0.15f, 0.9f);
        if (_tint != tint || !Mathf.Approximately(_opacity, alpha)) _dirty = true;
        _tint = tint;
        _opacity = alpha;
    }

    /// <summary>Optional explicit inputs support geometry inspection without a live robot.</summary>
    public bool Refresh(RosRobotRig source = null, RosRobotModel model = null)
    {
        if (source != null) _explicitSource = source;
        if (model != null) _explicitModel = model;
        RosRobotRig selectedSource = _explicitSource != null ? _explicitSource : FindLiveSource();
        RosRobotModel selectedModel = _explicitModel ?? ToolSelectionController.Instance?.CurrentModel ?? selectedSource?.Model;
        if (selectedModel == null)
        {
            if (HasVisual) ClearGeometry();
            StatusText = "Waiting for selected tool model";
            return false;
        }
        if (selectedSource != null && (selectedSource.Model == null || selectedSource.Model.tool != selectedModel.tool))
            selectedSource = null;
        if (_source != selectedSource)
        {
            if (_source != null) _source.ModelChanged -= SourceModelChanged;
            _source = selectedSource;
            if (_source != null) _source.ModelChanged += SourceModelChanged;
            _dirty = true;
        }
        if (_model != selectedModel || _appliedReferenceRevision != _referenceRevision) _dirty = true;
        _model = selectedModel;
        if (!_dirty) return HasVisual;
        _dirty = false;
        _appliedReferenceRevision = _referenceRevision;
        ClearGeometry();
        _geometry = new GameObject("Selected tool at target TCP");
        _geometry.layer = 2;
        _geometry.transform.SetParent(transform, false);
        try
        {
            RosRobotModelMath.Validate(_model);
            bool copied = _source != null && CopyLiveGeometry(_source);
            if (!copied)
            {
                ClearGeometry();
                _geometry = new GameObject("Selected tool at target TCP");
                _geometry.layer = 2;
                _geometry.transform.SetParent(transform, false);
                BuildModelGeometry(_model);
            }
            HasVisual = _geometry.GetComponentsInChildren<MeshRenderer>(true).Length > 0;
            TcpLocalBounds = CalculateBounds();
            StatusText = HasVisual ? (_model.tool == "none" ? "No tool — flange TCP" : _model.tool + " at TCP")
                : "Tool geometry is not available yet";
        }
        catch (Exception exception)
        {
            ClearGeometry();
            StatusText = "Tool preview unavailable: " + exception.Message;
            Debug.LogWarning("ToolPoseVisual: " + StatusText, this);
        }
        VisualChanged?.Invoke();
        return HasVisual;
    }

    private void OnEnable() { _dirty = true; Refresh(); }
    private void SourceModelChanged() => _dirty = true;
    private void LateUpdate()
    {
        if (!_dirty && Time.unscaledTime < _nextInspection) return;
        _nextInspection = Time.unscaledTime + 0.25f;
        Refresh();
    }

    private static RosRobotRig FindLiveSource()
    {
        foreach (RosRobotRig rig in FindObjectsByType<RosRobotRig>(FindObjectsSortMode.None))
            if (!rig.IsGhost && rig.isActiveAndEnabled && rig.Model != null && rig.Tcp != null) return rig;
        return null;
    }

    private static HashSet<string> ToolLinks(RosRobotModel model)
    {
        HashSet<string> names = new HashSet<string>();
        if (model.tool == "none") names.Add("ur_wrist_3_link");
        else
            foreach (RosRobotLink link in model.links)
                if ((link.parent == "ur_flange" && link.name != "ur_tool0") || names.Contains(link.parent ?? ""))
                    names.Add(link.name);
        return names;
    }

    private bool CopyLiveGeometry(RosRobotRig source)
    {
        if (!RobotFrameMath.IsRigidFrame(source.Tcp)) return false;
        HashSet<string> toolLinks = ToolLinks(_model);
        Dictionary<Transform, string> knownLinks = new Dictionary<Transform, string>();
        foreach (RosRobotLink link in source.Model.links)
            if (source.TryGetLink(link.name, out Transform node)) knownLinks[node] = link.name;
        int count = 0;
        foreach (MeshFilter mesh in source.GetComponentsInChildren<MeshFilter>())
        {
            MeshRenderer renderer = mesh.GetComponent<MeshRenderer>();
            if (mesh.sharedMesh == null || renderer == null || !renderer.enabled) continue;
            Transform owner = mesh.transform;
            while (owner != null && !knownLinks.ContainsKey(owner)) owner = owner.parent;
            if (owner == null || !toolLinks.Contains(knownLinks[owner])) continue;
            Pose local = new Pose(source.Tcp.InverseTransformPoint(mesh.transform.position),
                Quaternion.Inverse(source.Tcp.rotation) * mesh.transform.rotation);
            AddMesh(mesh.sharedMesh, local, mesh.transform.lossyScale, renderer.sharedMaterials,
                "Tool mesh: " + knownLinks[owner]);
            count++;
        }
        return count > 0;
    }

    private void BuildModelGeometry(RosRobotModel model)
    {
        Dictionary<string, Pose> poses = RosRobotModelMath.ForwardKinematics(model);
        Pose tcp = poses["tcp"];
        HashSet<string> links = ToolLinks(model);
        foreach (RosRobotLink link in model.links)
        {
            if (!links.Contains(link.name)) continue;
            Pose linkInTcp = RobotFrameMath.RelativeTo(tcp, poses[link.name]);
            if (link.name == "kenos_sl_base")
            {
                Mesh detail = KenosSlDetail();
                if (detail != null)
                {
                    AddMesh(detail, linkInTcp, Vector3.one, null, "Kenos SL CAD tool");
                    continue;
                }
            }
            if (link.visuals == null) continue;
            foreach (RosRobotGeometry geometry in link.visuals)
            {
                Pose visualInTcp = RobotFrameMath.Compose(linkInTcp, RosRobotModelMath.GeometryPose(geometry));
                if (geometry.kind == "mesh") CopyReferenceMesh(link.name, geometry, visualInTcp);
                else AddPrimitive(geometry, visualInTcp);
            }
        }
    }

    private void CopyReferenceMesh(string linkName, RosRobotGeometry geometry, Pose visualInTcp)
    {
        if (_referencePrefab == null) return;
        RosRobotLink reference = (_meshReference ?? (_meshReference = RosRobotModelCatalog.Load().Find("kenos"))).FindLink(linkName);
        bool supported = false;
        if (reference?.visuals != null)
            foreach (RosRobotGeometry known in reference.visuals)
                if (known.kind == "mesh" && MeshKey(known.meshUri) == MeshKey(geometry.meshUri)) supported = true;
        if (!supported) throw new InvalidOperationException("Unsupported tool visual mesh for " + linkName + ".");
        string importedName = linkName.StartsWith("ur_", StringComparison.Ordinal) ? linkName.Substring(3) : linkName;
        Transform referenceLink = null;
        foreach (Transform node in _referencePrefab.GetComponentsInChildren<Transform>(true))
            if (node.name == importedName) { referenceLink = node; break; }
        Transform visuals = referenceLink != null ? referenceLink.Find("Visuals") : null;
        if (visuals == null || visuals.childCount != 1) return;
        // Match RosRobotRig's template handling: replace the importer geometry
        // root's pose/scale with the selected URDF geometry, retain its mesh children.
        Transform origin = visuals.GetChild(0);
        Vector3 geometryScale = geometry.scale != null ? Abs(RosRobotModelMath.FromRosVector(geometry.scale)) : Vector3.one;
        foreach (MeshFilter filter in origin.GetComponentsInChildren<MeshFilter>(true))
        {
            MeshRenderer renderer = filter.GetComponent<MeshRenderer>();
            if (filter.sharedMesh == null || renderer == null) continue;
            Pose meshLocal = new Pose(origin.InverseTransformPoint(filter.transform.position),
                Quaternion.Inverse(origin.rotation) * filter.transform.rotation);
            meshLocal.position = Vector3.Scale(meshLocal.position, geometryScale);
            Vector3 scale = Vector3.Scale(Divide(filter.transform.lossyScale, origin.lossyScale), geometryScale);
            AddMesh(filter.sharedMesh, RobotFrameMath.Compose(visualInTcp, meshLocal), scale,
                renderer.sharedMaterials, "Tool mesh: " + linkName);
        }
    }

    private void AddPrimitive(RosRobotGeometry geometry, Pose local)
    {
        PrimitiveType kind = geometry.kind == "box" ? PrimitiveType.Cube :
            geometry.kind == "sphere" ? PrimitiveType.Sphere : PrimitiveType.Cylinder;
        GameObject node = GameObject.CreatePrimitive(kind);
        node.name = "Tool geometry: " + geometry.kind;
        node.layer = 2;
        node.transform.SetParent(_geometry.transform, false);
        node.transform.SetLocalPositionAndRotation(local.position, local.rotation);
        node.transform.localScale = geometry.kind == "box" ? Abs(RosRobotModelMath.FromRosVector(geometry.size)) :
            geometry.kind == "sphere" ? Vector3.one * geometry.radius * 2f :
            new Vector3(geometry.radius * 2f, geometry.length * 0.5f, geometry.radius * 2f);
        Collider collider = node.GetComponent<Collider>();
        if (collider != null) { collider.enabled = false; Dispose(collider); }
        MeshRenderer renderer = node.GetComponent<MeshRenderer>();
        renderer.sharedMaterial = GhostMaterial(null);
        renderer.shadowCastingMode = ShadowCastingMode.Off;
        renderer.receiveShadows = false;
    }

    private void AddMesh(Mesh mesh, Pose local, Vector3 scale, Material[] sourceMaterials, string title)
    {
        GameObject node = new GameObject(title, typeof(MeshFilter), typeof(MeshRenderer));
        node.layer = 2;
        node.transform.SetParent(_geometry.transform, false);
        node.transform.SetLocalPositionAndRotation(local.position, local.rotation);
        node.transform.localScale = scale;
        node.GetComponent<MeshFilter>().sharedMesh = mesh;
        MeshRenderer renderer = node.GetComponent<MeshRenderer>();
        int count = Mathf.Max(1, mesh.subMeshCount);
        Material[] materials = new Material[count];
        for (int index = 0; index < count; index++)
            materials[index] = GhostMaterial(sourceMaterials != null && index < sourceMaterials.Length ? sourceMaterials[index] : null);
        renderer.sharedMaterials = materials;
        renderer.shadowCastingMode = ShadowCastingMode.Off;
        renderer.receiveShadows = false;
    }

    private Material GhostMaterial(Material source)
    {
        // A source Shader Graph can bake an opaque render state. Use a known
        // transparent-capable lit shader while preserving source surface detail.
        Shader shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        if (shader == null) throw new InvalidOperationException("A tool preview shader is required.");
        Material material = new Material(shader);
        if (source != null)
        {
            foreach (string texture in new[] { "_BaseMap", "_MainTex", "_BumpMap" })
                if (source.HasProperty(texture) && material.HasProperty(texture))
                    material.SetTexture(texture, source.GetTexture(texture));
            foreach (string value in new[] { "_Metallic", "_Smoothness", "_Glossiness", "_BumpScale" })
                if (source.HasProperty(value) && material.HasProperty(value))
                    material.SetFloat(value, source.GetFloat(value));
            if (material.HasProperty("_BumpMap") && material.GetTexture("_BumpMap") != null)
                material.EnableKeyword("_NORMALMAP");
        }
        Color original = source != null && source.HasProperty("_BaseColor") ? source.GetColor("_BaseColor") :
            source != null && source.HasProperty("_Color") ? source.GetColor("_Color") : Color.gray;
        Color color = Color.Lerp(original, _tint, 0.65f);
        color.a = _opacity;
        if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
        if (material.HasProperty("_Color")) material.SetColor("_Color", color);
        material.SetOverrideTag("RenderType", "Transparent");
        if (material.HasProperty("_Surface")) material.SetFloat("_Surface", 1f);
        if (material.HasProperty("_Mode")) material.SetFloat("_Mode", 3f);
        if (material.HasProperty("_SrcBlend")) material.SetFloat("_SrcBlend", (float)BlendMode.SrcAlpha);
        if (material.HasProperty("_DstBlend")) material.SetFloat("_DstBlend", (float)BlendMode.OneMinusSrcAlpha);
        if (material.HasProperty("_SrcBlendAlpha")) material.SetFloat("_SrcBlendAlpha", (float)BlendMode.One);
        if (material.HasProperty("_DstBlendAlpha")) material.SetFloat("_DstBlendAlpha", (float)BlendMode.OneMinusSrcAlpha);
        if (material.HasProperty("_ZWrite")) material.SetFloat("_ZWrite", 0f);
        material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        material.EnableKeyword("_ALPHABLEND_ON");
        material.DisableKeyword("_ALPHATEST_ON");
        material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
        material.renderQueue = (int)RenderQueue.Transparent;
        _materials.Add(material);
        return material;
    }

    private Bounds CalculateBounds()
    {
        Bounds result = new Bounds(Vector3.zero, Vector3.zero);
        bool initialized = false;
        foreach (MeshFilter filter in _geometry.GetComponentsInChildren<MeshFilter>(true))
        {
            if (filter.sharedMesh == null) continue;
            Bounds mesh = filter.sharedMesh.bounds;
            for (int corner = 0; corner < 8; corner++)
            {
                Vector3 offset = new Vector3((corner & 1) == 0 ? -mesh.extents.x : mesh.extents.x,
                    (corner & 2) == 0 ? -mesh.extents.y : mesh.extents.y,
                    (corner & 4) == 0 ? -mesh.extents.z : mesh.extents.z);
                Vector3 point = transform.InverseTransformPoint(filter.transform.TransformPoint(mesh.center + offset));
                if (!initialized) { result = new Bounds(point, Vector3.zero); initialized = true; }
                else result.Encapsulate(point);
            }
        }
        return initialized ? result : new Bounds(Vector3.zero, Vector3.one * 0.04f);
    }

    private static Mesh KenosSlDetail()
    {
        if (_kenosSlDetail != null) return _kenosSlDetail;
        TextAsset asset = Resources.Load<TextAsset>("RobotModels/KenosSLVisual");
        if (asset == null) return null;
        using (BinaryReader reader = new BinaryReader(new MemoryStream(asset.bytes, false)))
        {
            if (new string(reader.ReadChars(4)) != "KSL1") throw new InvalidDataException("Invalid Kenos SL mesh header.");
            int vertexCount = reader.ReadInt32(), indexCount = reader.ReadInt32();
            if (vertexCount <= 0 || indexCount <= 0 || indexCount % 3 != 0 ||
                asset.bytes.LongLength != 12L + vertexCount * 12L + indexCount * 4L)
                throw new InvalidDataException("Invalid Kenos SL mesh length.");
            Vector3[] vertices = new Vector3[vertexCount];
            for (int index = 0; index < vertexCount; index++)
            {
                vertices[index] = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
                if (!RobotFrameMath.IsValidPose(new Pose(vertices[index], Quaternion.identity)))
                    throw new InvalidDataException("Invalid Kenos SL mesh vertex.");
            }
            int[] triangles = new int[indexCount];
            for (int index = 0; index < indexCount; index++)
            {
                triangles[index] = reader.ReadInt32();
                if (triangles[index] < 0 || triangles[index] >= vertexCount)
                    throw new InvalidDataException("Invalid Kenos SL mesh index.");
            }
            _kenosSlDetail = new Mesh { name = "Kenos SL target CAD geometry", indexFormat =
                vertexCount > 65535 ? IndexFormat.UInt32 : IndexFormat.UInt16 };
            _kenosSlDetail.vertices = vertices;
            _kenosSlDetail.triangles = triangles;
            _kenosSlDetail.RecalculateNormals();
            _kenosSlDetail.RecalculateBounds();
        }
        return _kenosSlDetail;
    }

    private static Vector3 Abs(Vector3 value) => new Vector3(Mathf.Abs(value.x), Mathf.Abs(value.y), Mathf.Abs(value.z));
    private static Vector3 Divide(Vector3 a, Vector3 b) => new Vector3(a.x / b.x, a.y / b.y, a.z / b.z);
    private static string MeshKey(string uri)
    {
        string path = (uri ?? "").Replace('\\', '/');
        int meshes = path.IndexOf("/meshes/", StringComparison.Ordinal);
        return meshes >= 0 ? path.Substring(meshes) : path;
    }

    private void ClearGeometry()
    {
        HasVisual = false;
        TcpLocalBounds = new Bounds(Vector3.zero, Vector3.one * 0.04f);
        if (_geometry != null) { _geometry.SetActive(false); Dispose(_geometry); _geometry = null; }
        foreach (Material material in _materials) if (material != null) Dispose(material);
        _materials.Clear();
    }

    private static void Dispose(UnityEngine.Object value)
    {
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }

    private void OnDestroy()
    {
        if (_source != null) _source.ModelChanged -= SourceModelChanged;
        ClearGeometry();
        VisualChanged = null;
    }
}
