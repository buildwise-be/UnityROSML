using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>A visual replica of the ROS model. ROS joint feedback owns its pose, never Unity dynamics.</summary>
public sealed class RosRobotRig : MonoBehaviour
{
    private readonly Dictionary<string, Transform> _links = new Dictionary<string, Transform>();
    private readonly Dictionary<string, ArticulationBody> _bodies = new Dictionary<string, ArticulationBody>();
    private readonly Dictionary<string, double> _positions = new Dictionary<string, double>();
    private readonly Dictionary<string, GameObject> _meshTemplates = new Dictionary<string, GameObject>();
    private readonly List<GameObject> _legacyToolBranches = new List<GameObject>();
    private readonly List<GameObject> _legacyVisuals = new List<GameObject>();
    private readonly List<GameObject> _generatedVisuals = new List<GameObject>();
    private readonly List<GameObject> _generatedFrames = new List<GameObject>();
    private readonly List<Material> _ownedMaterials = new List<Material>();
    private static RosRobotModel _meshReference;
    private static Mesh _kenosSlDetail;
    private static bool _checkedKenosSlDetail;
    private GameObject _templateRoot;
    private GameObject _toolRoot;
    private Material _materialTemplate;
    private ArticulationBody _baseBody;
    private float _opacityLimit = 1f;

    public RosRobotModel Model { get; private set; }
    public bool IsGhost { get; set; }
    public string LastError { get; private set; }
    public Transform BaseLink => _links.TryGetValue("ur_base_link", out Transform link) ? link : null;
    public Transform Tcp => _links.TryGetValue("tcp", out Transform link) ? link : null;
    public event Action ModelChanged;

    public bool TryGetLink(string name, out Transform link) => _links.TryGetValue(name, out link);
    public bool TryGetJointPosition(string name, out double radians) => _positions.TryGetValue(name, out radians);

    public bool ApplyModel(RosRobotModel model)
    {
        bool wasActive = gameObject.activeSelf;
        try
        {
            RosRobotModelMath.Validate(model);
            Dictionary<string, Transform> imported = ImportedLinks();
            if (!imported.TryGetValue("ur_base_link", out Transform baseLink) ||
                !imported.TryGetValue("ur_flange", out Transform flange))
                throw new ArgumentException("The imported robot needs base_link and flange frames.");
            if (baseLink.GetComponent<ArticulationBody>() == null)
                throw new ArgumentException("The robot base needs an articulation body.");
            CacheToolTemplates(flange);
            HashSet<string> toolLinks = ToolLinks(model);
            RosRobotModel reference = _meshReference ?? (_meshReference = RosRobotModelCatalog.Load().Find("kenos"));
            foreach (RosRobotLink link in model.links)
            {
                if (!toolLinks.Contains(link.name) && !imported.ContainsKey(link.name) &&
                    (link.name == "ur_base_link" || RosRobotModelMath.IsRevolute(link)))
                    throw new ArgumentException("The imported robot is missing " + link.name + ".");
                if (RosRobotModelMath.IsRevolute(link) &&
                    (!imported.TryGetValue(link.name, out Transform joint) || joint.GetComponent<ArticulationBody>() == null))
                    throw new ArgumentException("No articulation for " + link.jointName + ".");
                if (link.visuals != null)
                    for (int index = 0; index < link.visuals.Length; index++)
                    {
                        RosRobotGeometry geometry = link.visuals[index];
                        if (geometry.kind != "mesh") continue;
                        RosRobotLink known = reference.FindLink(link.name);
                        if (!_meshTemplates.ContainsKey(link.name) || known == null || known.visuals == null ||
                            known.visuals.Length != link.visuals.Length || known.visuals[index].kind != "mesh" ||
                            MeshKey(known.visuals[index].meshUri) != MeshKey(geometry.meshUri))
                            throw new ArgumentException("Unsupported visual mesh or mesh count for " + link.name + ".");
                    }
            }

            Pose savedBase = new Pose(baseLink.position, baseLink.rotation);
            gameObject.SetActive(false);
            DisableImporterControllers();
            foreach (GameObject branch in _legacyToolBranches) branch.SetActive(false);
            foreach (GameObject visuals in _legacyVisuals) visuals.SetActive(false);
            // A live description may insert a fixed TF frame before an imported joint.
            // Keep the real articulation when replacing those generated frame nodes.
            foreach (Transform node in imported.Values)
                foreach (GameObject frame in _generatedFrames)
                    if (frame != null && node.IsChildOf(frame.transform)) node.SetParent(baseLink, true);
            ClearGeneratedTool();
            _links.Clear();
            _bodies.Clear();
            Model = model;
            _toolRoot = new GameObject("Selected ROS tool - " + model.tool);
            _toolRoot.transform.SetParent(flange, false);
            foreach (RosRobotLink link in model.links)
            {
                Transform node;
                if (toolLinks.Contains(link.name))
                {
                    node = new GameObject(link.name).transform;
                    node.SetParent(link.parent == "ur_flange" ? _toolRoot.transform : _links[link.parent], false);
                }
                else if (!imported.TryGetValue(link.name, out node))
                {
                    node = new GameObject(link.name).transform;
                    node.SetParent(_links[link.parent], false);
                    _generatedFrames.Add(node.gameObject);
                }
                _links.Add(link.name, node);
                if (link.name != "ur_base_link")
                {
                    if (!toolLinks.Contains(link.name)) node.SetParent(_links[link.parent], false);
                    Pose origin = RosRobotModelMath.LocalPose(link);
                    node.SetLocalPositionAndRotation(origin.position, origin.rotation);
                    node.localScale = Vector3.one;
                    ArticulationBody body = node.GetComponent<ArticulationBody>();
                    if (body != null) ConfigureJoint(body, link, origin);
                }
                if (RosRobotModelMath.IsRevolute(link))
                {
                    ArticulationBody body = node.GetComponent<ArticulationBody>();
                    if (body == null) throw new ArgumentException("No articulation for " + link.jointName + ".");
                    _bodies.Add(link.jointName, body);
                    if (!_positions.ContainsKey(link.jointName)) _positions.Add(link.jointName, 0d);
                }
                if (link.name == "kenos_sl_base" && TryCreateKenosSlDetail(node, link)) continue;
                if (link.visuals != null)
                    foreach (RosRobotGeometry geometry in link.visuals) CreateVisual(node, link.name, geometry);
            }
            _baseBody = baseLink.GetComponent<ArticulationBody>();
            foreach (ArticulationBody body in GetComponentsInChildren<ArticulationBody>(true)) body.useGravity = false;
            ApplyGhostMaterials();
            // Both the live replica and trajectory ghosts are read-only displays of ROS state.
            foreach (Collider collider in GetComponentsInChildren<Collider>(true)) collider.enabled = false;
            baseLink.SetPositionAndRotation(savedBase.position, savedBase.rotation);
            gameObject.SetActive(wasActive);
            if (_baseBody != null && _baseBody.isActiveAndEnabled) _baseBody.TeleportRoot(savedBase.position, savedBase.rotation);
            ApplyJointCoordinates();
            LastError = null;
            ModelChanged?.Invoke();
            return true;
        }
        catch (Exception exception)
        {
            gameObject.SetActive(wasActive);
            LastError = exception.Message;
            Debug.LogError("RosRobotRig: " + LastError, this);
            return false;
        }
    }

    public bool SetJointPositions(string[] names, double[] radians)
    {
        if (Model == null || names == null || radians == null || names.Length != radians.Length) return false;
        HashSet<string> seen = new HashSet<string>();
        for (int index = 0; index < names.Length; index++)
        {
            if (names[index] == null) return false;
            if (!_bodies.ContainsKey(names[index])) continue; // /joint_states also contains the mobile base.
            double value = radians[index];
            if (!seen.Add(names[index]) || double.IsNaN(value) || double.IsInfinity(value)) return false;
        }
        for (int index = 0; index < names.Length; index++)
            if (_bodies.ContainsKey(names[index])) _positions[names[index]] = radians[index];
        ApplyJointCoordinates();
        return true;
    }

    public void SetBaseWorldPose(Pose pose)
    {
        if (BaseLink == null || !RobotFrameMath.IsValidPose(pose)) return;
        Pose rootToBase = RobotFrameMath.RelativeTo(new Pose(transform.position, transform.rotation),
            new Pose(BaseLink.position, BaseLink.rotation));
        Pose root = RobotFrameMath.Compose(pose, RobotFrameMath.Inverse(rootToBase));
        transform.SetPositionAndRotation(root.position, root.rotation);
        if (_baseBody != null && _baseBody.isActiveAndEnabled) _baseBody.TeleportRoot(pose.position, pose.rotation);
        else BaseLink.SetPositionAndRotation(pose.position, pose.rotation);
        ApplyJointCoordinates();
    }

    private void OnEnable()
    {
        if (Model != null) ApplyJointCoordinates();
    }

    private void ApplyJointCoordinates()
    {
        if (Model == null) return;
        foreach (RosRobotLink link in Model.links)
        {
            if (link.name == "ur_base_link" || !_links.TryGetValue(link.name, out Transform node)) continue;
            Pose local = RosRobotModelMath.LocalPose(link);
            if (RosRobotModelMath.IsRevolute(link))
            {
                double radians = _positions[link.jointName];
                local.rotation *= Quaternion.AngleAxis((float)(-radians * Mathf.Rad2Deg), RosRobotModelMath.Axis(link));
                ArticulationBody body = _bodies[link.jointName];
                ArticulationDrive drive = body.xDrive;
                drive.target = (float)(radians * Mathf.Rad2Deg);
                body.xDrive = drive;
                if (body.isActiveAndEnabled && body.dofCount == 1)
                {
                    body.jointPosition = new ArticulationReducedSpace((float)radians);
                    body.jointVelocity = new ArticulationReducedSpace(0f);
                }
            }
            node.SetLocalPositionAndRotation(local.position, local.rotation);
        }
    }

    private static void ConfigureJoint(ArticulationBody body, RosRobotLink link, Pose origin)
    {
        bool revolute = RosRobotModelMath.IsRevolute(link);
        body.jointType = revolute ? ArticulationJointType.RevoluteJoint : ArticulationJointType.FixedJoint;
        body.matchAnchors = false;
        Quaternion childAnchor = revolute
            ? Quaternion.FromToRotation(Vector3.right, -RosRobotModelMath.Axis(link)) : Quaternion.identity;
        body.anchorPosition = Vector3.zero;
        body.anchorRotation = childAnchor;
        ArticulationBody parent = body.transform.parent != null ? body.transform.parent.GetComponentInParent<ArticulationBody>(true) : null;
        body.parentAnchorPosition = parent != null ? parent.transform.InverseTransformPoint(body.transform.position) : origin.position;
        body.parentAnchorRotation = parent != null
            ? Quaternion.Inverse(parent.transform.rotation) * body.transform.rotation * childAnchor : origin.rotation * childAnchor;
        if (revolute)
        {
            body.twistLock = link.jointType == "continuous" ? ArticulationDofLock.FreeMotion : ArticulationDofLock.LimitedMotion;
            ArticulationDrive drive = body.xDrive;
            drive.lowerLimit = link.lower * Mathf.Rad2Deg;
            drive.upperLimit = link.upper * Mathf.Rad2Deg;
            body.xDrive = drive;
        }
    }

    private Dictionary<string, Transform> ImportedLinks()
    {
        Dictionary<string, Transform> links = new Dictionary<string, Transform>();
        foreach (Transform node in GetComponentsInChildren<Transform>(true))
        {
            if ((_templateRoot != null && node.IsChildOf(_templateRoot.transform)) ||
                (_toolRoot != null && node.IsChildOf(_toolRoot.transform))) continue;
            if (node.name == "base_link" || node.name == "base_link_inertia" || node.name == "shoulder_link" ||
                node.name == "upper_arm_link" || node.name == "forearm_link" || node.name == "wrist_1_link" ||
                node.name == "wrist_2_link" || node.name == "wrist_3_link" || node.name == "flange" ||
                node.name == "tool0" || node.name == "ft_frame" || node.name == "base")
                links["ur_" + node.name] = node;
        }
        return links;
    }

    private static HashSet<string> ToolLinks(RosRobotModel model)
    {
        HashSet<string> result = new HashSet<string>();
        foreach (RosRobotLink link in model.links)
            if ((link.parent == "ur_flange" && link.name != "ur_tool0") || result.Contains(link.parent ?? "")) result.Add(link.name);
        return result;
    }

    private void CacheToolTemplates(Transform flange)
    {
        if (_templateRoot != null) return;
        foreach (Renderer renderer in GetComponentsInChildren<Renderer>(true))
            foreach (Material material in renderer.sharedMaterials)
            {
                if (material == null) continue;
                if (_materialTemplate == null || (!HasMaterialColor(_materialTemplate) && HasMaterialColor(material)))
                    _materialTemplate = material;
                Color color = MaterialColor(material);
                if (HasMaterialColor(material) && color.a < _opacityLimit)
                {
                    _opacityLimit = color.a;
                    _materialTemplate = material;
                }
            }
        IsGhost = IsGhost || _opacityLimit < 0.99f || name.IndexOf("transparent", StringComparison.OrdinalIgnoreCase) >= 0;
        if (IsGhost && _opacityLimit >= 0.99f) _opacityLimit = 0.15686275f;
        _templateRoot = new GameObject("ROS tool visual templates");
        _templateRoot.transform.SetParent(transform, false);
        _templateRoot.SetActive(false);
        foreach (Transform branch in flange)
        {
            if (!branch.name.StartsWith("kenos", StringComparison.Ordinal)) continue;
            _legacyToolBranches.Add(branch.gameObject);
        }
        foreach (Transform node in GetComponentsInChildren<Transform>(true))
        {
            if (node.IsChildOf(_templateRoot.transform)) continue;
            Transform visuals = node.Find("Visuals");
            if (visuals == null) continue;
            _legacyVisuals.Add(visuals.gameObject);
            string key = node.name.StartsWith("kenos", StringComparison.Ordinal) ? node.name : "ur_" + node.name;
            // The importer stores the URDF geometry origin on this immediate child.
            // Clone that node, then replace its origin from the selected ROS description.
            if (visuals.childCount != 1 || _meshTemplates.ContainsKey(key)) continue;
            GameObject clone = Instantiate(visuals.GetChild(0).gameObject, _templateRoot.transform, false);
            clone.transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);
            clone.transform.localScale = Vector3.one;
            _meshTemplates.Add(key, clone);
        }
    }

    private void CreateVisual(Transform link, string linkName, RosRobotGeometry geometry)
    {
        GameObject visual;
        if (geometry.kind == "mesh")
        {
            visual = Instantiate(_meshTemplates[linkName], link, false);
            visual.SetActive(true);
            Vector3 scale = geometry.scale == null ? Vector3.one : RosRobotModelMath.FromRosVector(geometry.scale);
            visual.transform.localScale = Abs(scale);
        }
        else
        {
            PrimitiveType primitive = geometry.kind == "box" ? PrimitiveType.Cube :
                geometry.kind == "sphere" ? PrimitiveType.Sphere : PrimitiveType.Cylinder;
            visual = GameObject.CreatePrimitive(primitive);
            visual.transform.SetParent(link, false);
            visual.transform.localScale = geometry.kind == "box" ? Abs(RosRobotModelMath.FromRosVector(geometry.size)) :
                geometry.kind == "sphere" ? Vector3.one * (2f * geometry.radius) :
                new Vector3(2f * geometry.radius, geometry.length / 2f, 2f * geometry.radius);
            Collider collider = visual.GetComponent<Collider>();
            if (collider != null) collider.enabled = false;
            Renderer renderer = visual.GetComponent<Renderer>();
            if (_materialTemplate != null)
            {
                renderer.sharedMaterial = CreateMaterial(geometry.color);
            }
        }
        visual.name = "ROS visual: " + geometry.kind;
        _generatedVisuals.Add(visual);
        Pose pose = RosRobotModelMath.GeometryPose(geometry);
        visual.transform.SetLocalPositionAndRotation(pose.position, pose.rotation);
    }

    private static Vector3 Abs(Vector3 value) => new Vector3(Mathf.Abs(value.x), Mathf.Abs(value.y), Mathf.Abs(value.z));

    private Material CreateMaterial(float[] rgba)
    {
        if (_materialTemplate == null) return null;
        Material material = new Material(_materialTemplate);
        if (rgba != null && rgba.Length == 4)
        {
            Color color = new Color(rgba[0], rgba[1], rgba[2], rgba[3]);
            color.a = Mathf.Min(color.a, MaterialColor(material).a, _opacityLimit);
            if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
            if (material.HasProperty("_Color")) material.SetColor("_Color", color);
        }
        if (IsGhost)
        {
            material.SetOverrideTag("RenderType", "Transparent");
            if (material.HasProperty("_Surface")) material.SetFloat("_Surface", 1f);
            if (material.HasProperty("_Mode")) material.SetFloat("_Mode", 2f);
            if (material.HasProperty("_SrcBlend")) material.SetFloat("_SrcBlend", (float)BlendMode.SrcAlpha);
            if (material.HasProperty("_DstBlend")) material.SetFloat("_DstBlend", (float)BlendMode.OneMinusSrcAlpha);
            if (material.HasProperty("_SrcBlendAlpha")) material.SetFloat("_SrcBlendAlpha", (float)BlendMode.One);
            if (material.HasProperty("_DstBlendAlpha")) material.SetFloat("_DstBlendAlpha", (float)BlendMode.OneMinusSrcAlpha);
            if (material.HasProperty("_ZWrite")) material.SetFloat("_ZWrite", 0f);
            material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            material.EnableKeyword("_ALPHABLEND_ON");
            material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            material.DisableKeyword("_ALPHATEST_ON");
            material.renderQueue = (int)RenderQueue.Transparent;
        }
        _ownedMaterials.Add(material);
        return material;
    }

    private void ApplyGhostMaterials()
    {
        if (!IsGhost || _materialTemplate == null) return;
        Dictionary<Material, Material> copies = new Dictionary<Material, Material>();
        foreach (GameObject visual in _generatedVisuals)
            foreach (Renderer renderer in visual.GetComponentsInChildren<Renderer>(true))
            {
                Material[] materials = renderer.sharedMaterials;
                for (int index = 0; index < materials.Length; index++)
                {
                    Material source = materials[index];
                    if (source == null || _ownedMaterials.Contains(source)) continue;
                    if (!copies.TryGetValue(source, out Material copy))
                    {
                        Color color = MaterialColor(source);
                        copy = CreateMaterial(new[] { color.r, color.g, color.b, Mathf.Min(color.a, _opacityLimit) });
                        foreach (string property in new[] { "_BaseMap", "_MainTex" })
                            if (source.HasProperty(property) && copy.HasProperty(property))
                                copy.SetTexture(property, source.GetTexture(property));
                        copies.Add(source, copy);
                    }
                    materials[index] = copy;
                }
                renderer.sharedMaterials = materials;
                renderer.shadowCastingMode = ShadowCastingMode.Off;
                renderer.receiveShadows = false;
            }
    }

    private static bool HasMaterialColor(Material material) => material != null &&
        (material.HasProperty("_BaseColor") || material.HasProperty("_Color"));

    private static Color MaterialColor(Material material)
    {
        if (material != null && material.HasProperty("_BaseColor")) return material.GetColor("_BaseColor");
        if (material != null && material.HasProperty("_Color")) return material.GetColor("_Color");
        return Color.white; // Helper/interaction Shader Graphs need not expose a tint.
    }

    private bool TryCreateKenosSlDetail(Transform link, RosRobotLink definition)
    {
        Mesh mesh = GetKenosSlDetail();
        if (mesh == null) return false;
        GameObject visual = new GameObject("ROS visual: Kenos SL CAD detail");
        visual.transform.SetParent(link, false);
        visual.AddComponent<MeshFilter>().sharedMesh = mesh;
        Renderer renderer = visual.AddComponent<MeshRenderer>();
        float[] color = definition.visuals != null && definition.visuals.Length > 0 ? definition.visuals[0].color : null;
        renderer.sharedMaterial = CreateMaterial(color);
        _generatedVisuals.Add(visual);
        return true;
    }

    private static Mesh GetKenosSlDetail()
    {
        if (_checkedKenosSlDetail) return _kenosSlDetail;
        _checkedKenosSlDetail = true;
        TextAsset asset = Resources.Load<TextAsset>("RobotModels/KenosSLVisual");
        if (asset == null) return null;
        Mesh mesh = null;
        try
        {
            byte[] bytes = asset.bytes;
            using (BinaryReader reader = new BinaryReader(new MemoryStream(bytes, false)))
            {
                if (new string(reader.ReadChars(4)) != "KSL1") throw new InvalidDataException("Invalid KSL mesh header.");
                int vertexCount = reader.ReadInt32();
                int indexCount = reader.ReadInt32();
                if (vertexCount <= 0 || indexCount <= 0 || indexCount % 3 != 0 ||
                    bytes.LongLength != 12L + vertexCount * 12L + indexCount * 4L)
                    throw new InvalidDataException("Invalid KSL mesh length.");
                Vector3[] vertices = new Vector3[vertexCount];
                for (int i = 0; i < vertices.Length; i++)
                {
                    vertices[i] = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
                    if (!RobotFrameMath.IsValidPose(new Pose(vertices[i], Quaternion.identity)))
                        throw new InvalidDataException("Nonfinite KSL vertex.");
                }
                int[] indices = new int[indexCount];
                for (int i = 0; i < indices.Length; i++)
                {
                    indices[i] = reader.ReadInt32();
                    if (indices[i] < 0 || indices[i] >= vertexCount) throw new InvalidDataException("Invalid KSL mesh index.");
                }
                mesh = new Mesh { name = "Kenos SL CAD detail", hideFlags = HideFlags.HideAndDontSave,
                    indexFormat = vertexCount > 65535 ? IndexFormat.UInt32 : IndexFormat.UInt16 };
                mesh.vertices = vertices;
                mesh.triangles = indices;
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();
            }
            _kenosSlDetail = mesh;
            Application.quitting -= ResetSharedResources;
            Application.quitting += ResetSharedResources;
            return mesh;
        }
        catch (Exception exception)
        {
            if (mesh != null) Dispose(mesh);
            Debug.LogWarning("RosRobotRig: Using ROS primitive visuals; " + exception.Message);
            return null;
        }
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void ResetSharedResources()
    {
        if (_kenosSlDetail != null) Dispose(_kenosSlDetail);
        _kenosSlDetail = null;
        _checkedKenosSlDetail = false;
        _meshReference = null;
        Application.quitting -= ResetSharedResources;
    }

    private static string MeshKey(string uri)
    {
        string normalized = (uri ?? "").Replace('\\', '/');
        int meshes = normalized.IndexOf("/meshes/", StringComparison.Ordinal);
        return meshes >= 0 ? normalized.Substring(meshes) : normalized;
    }

    private void ClearGeneratedTool()
    {
        foreach (GameObject visual in _generatedVisuals)
        {
            if (visual == null) continue;
            visual.SetActive(false);
            visual.transform.SetParent(null);
            Dispose(visual);
        }
        _generatedVisuals.Clear();
        foreach (GameObject frame in _generatedFrames)
        {
            if (frame == null) continue;
            frame.SetActive(false);
            frame.transform.SetParent(null);
            Dispose(frame);
        }
        _generatedFrames.Clear();
        if (_toolRoot != null)
        {
            _toolRoot.SetActive(false);
            _toolRoot.transform.SetParent(null);
            Dispose(_toolRoot);
        }
        _toolRoot = null;
        foreach (Material material in _ownedMaterials) Dispose(material);
        _ownedMaterials.Clear();
    }

    private void DisableImporterControllers()
    {
        foreach (MonoBehaviour component in GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (component == null) continue; // Legacy prefabs can retain a removed importer/helper script.
            string type = component.GetType().FullName;
            if (type == "Unity.Robotics.UrdfImporter.Control.Controller" || type == "JointControl" ||
                type == "Unity.Robotics.UrdfImporter.Control.FKRobot") component.enabled = false;
        }
    }

    private static void Dispose(UnityEngine.Object value)
    {
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }

    private void OnDestroy()
    {
        foreach (Material material in _ownedMaterials) Dispose(material);
        ModelChanged = null;
    }
}
