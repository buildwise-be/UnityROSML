using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Filtering;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

/// <summary>Local orientation editing for captured targets. Never moves the pivot or publishes a command.</summary>
public sealed class TargetRotationHandles : MonoBehaviour
{
    public enum RotationHandleKind { Free, Tilt, Twist }

    private sealed class Handle
    {
        public RotationHandleKind Kind;
        public XRSimpleInteractable Interactable;
        public Material Material;
        public Color Color;
        public XRSelectFilterDelegate Filter;
        public readonly HashSet<IXRHoverInteractor> Hovering = new HashSet<IXRHoverInteractor>();
        public UnityAction<HoverEnterEventArgs> HoverEntered;
        public UnityAction<HoverExitEventArgs> HoverExited;
        public UnityAction<SelectEnterEventArgs> SelectEntered;
        public UnityAction<SelectExitEventArgs> SelectExited;
    }

    private static readonly HashSet<TargetRotationHandles> HoveredTargets = new HashSet<TargetRotationHandles>();
    private static readonly HashSet<TargetRotationHandles> SelectedTargets = new HashSet<TargetRotationHandles>();
    public static bool IsHovered => HoveredTargets.Count != 0 || TargetPoseContextMenu.IsOpen;
    public static bool IsInteracting => SelectedTargets.Count != 0;
    public event Action RotationStarted;
    public event Action RotationChanged;

    public static bool TryGetHoveredTarget(out Transform target)
    {
        target = null;
        foreach (TargetRotationHandles handles in HoveredTargets)
        {
            if (handles == null || !handles.isActiveAndEnabled) continue;
            // Overlapping target hit volumes must not choose an arbitrary pick/place reference.
            if (target != null && target != handles.transform) { target = null; return false; }
            target = handles.transform;
        }
        return target != null;
    }

    /// <summary>Use the visible tool as the free-rotation handle, rather than a ball at its TCP.</summary>
    public void ConfigureToolInteraction(Bounds tcpLocalBounds)
    {
        foreach (Handle handle in _handles)
        {
            if (handle.Kind != RotationHandleKind.Free || handle.Interactable == null) continue;
            bool wasEnabled = handle.Interactable.enabled;
            // XRI caches collider ownership on registration; re-register after replacing the ball collider.
            if (wasEnabled) handle.Interactable.enabled = false;
            GameObject tool = handle.Interactable.gameObject;
            tool.name = "Tool rotation handle - trigger drag; bumper for target menu";
            tool.transform.localScale = Vector3.one;
            MeshRenderer sphere = tool.GetComponent<MeshRenderer>();
            if (sphere != null) sphere.enabled = false;
            SphereCollider sphereCollider = tool.GetComponent<SphereCollider>();
            if (sphereCollider != null) sphereCollider.enabled = false;
            BoxCollider box = tool.GetComponent<BoxCollider>();
            if (box == null) box = tool.AddComponent<BoxCollider>();
            Vector3 size = tcpLocalBounds.size;
            box.center = tcpLocalBounds.center;
            box.size = new Vector3(Mathf.Max(0.025f, size.x), Mathf.Max(0.025f, size.y), Mathf.Max(0.025f, size.z));
            handle.Interactable.colliders.Clear();
            handle.Interactable.colliders.Add(box);
            if (wasEnabled) handle.Interactable.enabled = true;
            return;
        }
    }

    private readonly List<Handle> _handles = new List<Handle>();
    private readonly List<Mesh> _ownedMeshes = new List<Mesh>();
    private Transform _visualRoot;
    private Handle _activeHandle;
    private IXRSelectInteractor _activeInteractor;
    private bool _dragging;
    private RotationHandleKind _dragKind;
    private Vector3 _pivotWorld;
    private Vector3 _worldAxis;
    private Vector3 _previousRadial;
    private Quaternion _initialTargetRotation;
    private Quaternion _initialControllerRotation;
    private float _accumulatedDegrees;
    private Transform _parentAtBegin;
    private Pose _parentPoseAtBegin;

    public static void CancelAllInteractions()
    {
        HashSet<TargetRotationHandles> targets = new HashSet<TargetRotationHandles>(SelectedTargets);
        targets.UnionWith(HoveredTargets);
        foreach (TargetRotationHandles target in targets)
        {
            if (target == null) continue;
            target.AbortRotation();
            foreach (Handle handle in target._handles)
            {
                if (handle.Interactable != null && handle.Interactable.interactionManager != null)
                    handle.Interactable.interactionManager.CancelInteractableHover((IXRHoverInteractable)handle.Interactable);
                handle.Hovering.Clear();
            }
            target.RefreshHoverState();
        }
    }

    public static TargetRotationHandles Attach(Transform target, Material source, float sphereDiameter)
    {
        if (target == null) return null;
        TargetRotationHandles handles = target.GetComponent<TargetRotationHandles>();
        if (handles == null) handles = target.gameObject.AddComponent<TargetRotationHandles>();
        handles.Configure(source, sphereDiameter);
        handles.enabled = true;
        return handles;
    }

    public void Configure(Material source, float sphereDiameter)
    {
        EndRotation();
        if (_visualRoot != null) return;
        float diameter = float.IsNaN(sphereDiameter) || float.IsInfinity(sphereDiameter)
            ? 0.035f : Mathf.Clamp(sphereDiameter, 0.015f, 0.08f);
        _visualRoot = new GameObject("Captured target rotation handles").transform;
        _visualRoot.SetParent(transform, false);
        _visualRoot.gameObject.layer = 0;
        Color baseColor = source != null ? source.color : new Color(0.85f, 0.9f, 1f);
        // Slightly cover an existing marker sphere to avoid coincident rendered surfaces.
        CreateBall(diameter * 1.08f, source, baseColor);
        float radius = Mathf.Max(0.055f, diameter * 1.7f);
        CreateRing(RotationHandleKind.Tilt, radius, Vector3.right, Vector3.up,
            source, new Color(1f, 0.65f, 0.16f));
        CreateRing(RotationHandleKind.Twist, radius * 1.13f, Vector3.forward, Vector3.right,
            source, new Color(0.55f, 0.65f, 1f));
        _visualRoot.gameObject.SetActive(isActiveAndEnabled);
    }

    // These methods also allow isolated verification without an XR device.
    public bool BeginRotation(RotationHandleKind kind, Quaternion controllerWorldRotation, Vector3 radialWorld)
    {
        if (RobotCoordinateService.IsCalibrationEditActive || TargetPoseContextMenu.IsOpen || _dragging || OtherTargetSelected() || !isActiveAndEnabled || !RobotFrameMath.IsRigidFrame(transform) ||
            !RobotFrameMath.IsValidPose(new Pose(Vector3.zero, controllerWorldRotation))) return false;
        if (kind != RotationHandleKind.Free && kind != RotationHandleKind.Tilt && kind != RotationHandleKind.Twist) return false;
        _pivotWorld = transform.position;
        _parentAtBegin = transform.parent;
        _parentPoseAtBegin = _parentAtBegin != null
            ? new Pose(_parentAtBegin.position, _parentAtBegin.rotation) : Pose.identity;
        _initialTargetRotation = transform.rotation;
        _initialControllerRotation = controllerWorldRotation;
        _dragKind = kind;
        _worldAxis = _initialTargetRotation * (kind == RotationHandleKind.Tilt ? Vector3.forward : Vector3.up);
        if (kind != RotationHandleKind.Free &&
            !TcpTargetRotationMath.TrySignedPlaneAngle(radialWorld, radialWorld, _worldAxis, out _)) return false;
        _previousRadial = radialWorld;
        _accumulatedDegrees = 0f;
        _dragging = true;
        SelectedTargets.Add(this);
        RotationStarted?.Invoke();
        return true;
    }

    public bool UpdateRotation(Quaternion controllerWorldRotation, Vector3 radialWorld)
    {
        if (!_dragging) return false;
        if (RobotCoordinateService.IsCalibrationEditActive || ParentMoved() || !RobotFrameMath.IsRigidFrame(transform))
        {
            AbortRotation();
            return false;
        }
        if (!RobotFrameMath.IsValidPose(new Pose(Vector3.zero, controllerWorldRotation))) return false;
        Quaternion rotation;
        if (_dragKind == RotationHandleKind.Free)
        {
            rotation = TcpTargetRotationMath.ApplyControllerDelta(
                _initialTargetRotation, _initialControllerRotation, controllerWorldRotation);
        }
        else
        {
            if (!TcpTargetRotationMath.TrySignedPlaneAngle(_previousRadial, radialWorld, _worldAxis, out float delta))
                return false;
            _accumulatedDegrees += delta;
            _previousRadial = radialWorld;
            rotation = TcpTargetRotationMath.ApplyWorldAxisDelta(_initialTargetRotation, _worldAxis, _accumulatedDegrees);
        }
        bool changed = RotationDiffers(transform.rotation, rotation);
        transform.SetPositionAndRotation(_pivotWorld, rotation);
        if (changed) RotationChanged?.Invoke();
        return true;
    }

    public void EndRotation()
    {
        _dragging = false;
        _activeHandle = null;
        _activeInteractor = null;
        SelectedTargets.Remove(this);
        RefreshColors();
    }

    private void Update()
    {
        if (!_dragging || _activeHandle == null || _activeInteractor == null) return;
        if (ParentMoved() || (_activeInteractor is Behaviour behaviour && (behaviour == null || !behaviour.isActiveAndEnabled)))
        {
            AbortRotation();
            return;
        }
        Transform attach = _activeInteractor.GetAttachTransform(_activeHandle.Interactable);
        if (attach == null) { EndRotation(); return; }
        Quaternion controllerRotation = ControllerRotation(_activeInteractor, attach);
        Vector3 radial = _previousRadial;
        if (_dragKind != RotationHandleKind.Free && !TryGetRadial(_activeInteractor, attach, false, out radial)) return;
        UpdateRotation(controllerRotation, radial);
    }

    private bool OtherTargetSelected()
    {
        return SelectedTargets.Count != 0 && (SelectedTargets.Count != 1 || !SelectedTargets.Contains(this));
    }

    private bool ParentMoved()
    {
        if (transform.parent != _parentAtBegin) return true;
        if (_parentAtBegin == null) return false;
        return (_parentAtBegin.position - _parentPoseAtBegin.position).sqrMagnitude > 0.0000000001f ||
            RotationDiffers(_parentAtBegin.rotation, _parentPoseAtBegin.rotation);
    }

    private static bool RotationDiffers(Quaternion a, Quaternion b)
    {
        float sign = Quaternion.Dot(a, b) < 0f ? -1f : 1f;
        Vector4 difference = new Vector4(a.x - sign * b.x, a.y - sign * b.y,
            a.z - sign * b.z, a.w - sign * b.w);
        return difference.sqrMagnitude > 0.000000000001f;
    }

    private void AbortRotation()
    {
        Handle selected = _activeHandle;
        EndRotation();
        if (selected != null && selected.Interactable != null && selected.Interactable.interactionManager != null)
            selected.Interactable.interactionManager.CancelInteractableSelection((IXRSelectInteractable)selected.Interactable);
    }

    private void Selected(Handle handle, SelectEnterEventArgs args)
    {
        if (RobotCoordinateService.IsCalibrationEditActive || _dragging || !isActiveAndEnabled) return;
        Transform attach = args.interactorObject.GetAttachTransform(handle.Interactable);
        if (attach == null) return;
        _pivotWorld = transform.position;
        _worldAxis = transform.rotation * (handle.Kind == RotationHandleKind.Tilt ? Vector3.forward : Vector3.up);
        Vector3 radial = Vector3.zero;
        if (handle.Kind != RotationHandleKind.Free && !TryGetRadial(args.interactorObject, attach, true, out radial)) return;
        if (!BeginRotation(handle.Kind, ControllerRotation(args.interactorObject, attach), radial)) return;
        _activeHandle = handle;
        _activeInteractor = args.interactorObject;
        RefreshColors();
    }

    private static Quaternion ControllerRotation(IXRSelectInteractor interactor, Transform attach)
    {
        // Ray attach transforms may be manipulated or relocated to a hit point by XRI.
        if (interactor is XRRayInteractor ray)
        {
            Transform origin = ray.rayOriginTransform != null ? ray.rayOriginTransform : ray.transform;
            return origin.rotation;
        }
        return attach.rotation;
    }

    private bool TryGetRadial(IXRSelectInteractor interactor, Transform attach, bool beginning, out Vector3 radial)
    {
        if (interactor is XRRayInteractor ray)
        {
            if (beginning && ray.TryGetCurrent3DRaycastHit(out RaycastHit hit))
            {
                radial = hit.point - _pivotWorld;
                if (TcpTargetRotationMath.TrySignedPlaneAngle(radial, radial, _worldAxis, out _)) return true;
            }
            Transform origin = ray.rayOriginTransform != null ? ray.rayOriginTransform : ray.transform;
            if (TcpTargetRotationMath.TryRayPlaneIntersection(
                new Ray(origin.position, origin.forward), _pivotWorld, _worldAxis, out Vector3 point))
            {
                radial = point - _pivotWorld;
                return TcpTargetRotationMath.TrySignedPlaneAngle(radial, radial, _worldAxis, out _);
            }
        }
        radial = attach.position - _pivotWorld;
        return TcpTargetRotationMath.TrySignedPlaneAngle(radial, radial, _worldAxis, out _);
    }

    private void CreateBall(float diameter, Material source, Color color)
    {
        GameObject ball = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        ball.name = "Free rotation ball - select and turn controller";
        ball.layer = 0;
        ball.transform.SetParent(_visualRoot, false);
        ball.transform.localScale = Vector3.one * diameter;
        Handle handle = CreateHandle(ball, RotationHandleKind.Free, source, color);
        ball.GetComponent<MeshRenderer>().sharedMaterial = handle.Material;
    }

    private void CreateRing(RotationHandleKind kind, float radius, Vector3 u, Vector3 v, Material source, Color color)
    {
        const int segments = 24;
        const int sides = 8;
        const float start = -140f * Mathf.Deg2Rad;
        const float sweep = 280f * Mathf.Deg2Rad;
        const float thickness = 0.0032f;
        GameObject ring = new GameObject(kind == RotationHandleKind.Tilt ? "Tilt arrow - ROS X" : "Twist arrow - ROS Z");
        ring.layer = 0;
        ring.transform.SetParent(_visualRoot, false);
        Vector3 normal = Vector3.Cross(u, v).normalized;
        Vector3[] vertices = new Vector3[(segments + 1) * sides];
        int[] triangles = new int[segments * sides * 6];
        for (int i = 0; i <= segments; i++)
        {
            float angle = start + sweep * i / segments;
            Vector3 radial = Mathf.Cos(angle) * u + Mathf.Sin(angle) * v;
            for (int s = 0; s < sides; s++)
            {
                float around = s * 2f * Mathf.PI / sides;
                vertices[i * sides + s] = radius * radial + thickness * (Mathf.Cos(around) * radial + Mathf.Sin(around) * normal);
                if (i == segments) continue;
                int next = (s + 1) % sides;
                int t = (i * sides + s) * 6;
                int a = i * sides + s, b = i * sides + next, c = (i + 1) * sides + s, d = (i + 1) * sides + next;
                triangles[t] = a; triangles[t + 1] = c; triangles[t + 2] = b;
                triangles[t + 3] = b; triangles[t + 4] = c; triangles[t + 5] = d;
            }
        }
        Mesh mesh = new Mesh { name = kind + " curved rotation arrow", vertices = vertices, triangles = triangles };
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        _ownedMeshes.Add(mesh);
        ring.AddComponent<MeshFilter>().sharedMesh = mesh;
        MeshRenderer renderer = ring.AddComponent<MeshRenderer>();
        // Colliders follow only the thin arc, leaving its centre and open gap unobstructed.
        for (int i = 0; i < segments; i++)
        {
            float a = start + sweep * i / segments, b = start + sweep * (i + 1) / segments;
            Vector3 p = radius * (Mathf.Cos(a) * u + Mathf.Sin(a) * v);
            Vector3 q = radius * (Mathf.Cos(b) * u + Mathf.Sin(b) * v);
            GameObject segment = new GameObject("Arc hit segment");
            segment.layer = 0;
            segment.transform.SetParent(ring.transform, false);
            segment.transform.localPosition = (p + q) * 0.5f;
            segment.transform.localRotation = Quaternion.FromToRotation(Vector3.up, q - p);
            CapsuleCollider collider = segment.AddComponent<CapsuleCollider>();
            collider.radius = 0.006f;
            collider.height = (q - p).magnitude + collider.radius * 2f;
            collider.direction = 1;
            collider.isTrigger = false;
        }
        float end = start + sweep;
        Vector3 tangent = -Mathf.Sin(end) * u + Mathf.Cos(end) * v;
        GameObject head = new GameObject("Curved arrowhead", typeof(MeshFilter), typeof(MeshRenderer), typeof(SphereCollider));
        head.layer = 0;
        head.transform.SetParent(ring.transform, false);
        head.transform.localPosition = radius * (Mathf.Cos(end) * u + Mathf.Sin(end) * v);
        head.transform.localRotation = Quaternion.FromToRotation(Vector3.up, tangent);
        Mesh cone = CreateCone();
        _ownedMeshes.Add(cone);
        head.GetComponent<MeshFilter>().sharedMesh = cone;
        head.GetComponent<SphereCollider>().radius = 0.009f;
        head.GetComponent<SphereCollider>().center = Vector3.up * 0.008f;
        Handle handle = CreateHandle(ring, kind, source, color);
        renderer.sharedMaterial = handle.Material;
        head.GetComponent<MeshRenderer>().sharedMaterial = handle.Material;
    }

    private Handle CreateHandle(GameObject go, RotationHandleKind kind, Material source, Color color)
    {
        Shader shader = source != null ? source.shader : Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null) shader = Shader.Find("Unlit/Color");
        Material material = source != null ? new Material(source) : new Material(shader);
        material.name = "Owned " + kind + " rotation handle material";
        material.color = color;
        XRSimpleInteractable interactable = go.AddComponent<XRSimpleInteractable>();
        interactable.selectMode = InteractableSelectMode.Single;
        interactable.interactionLayers = 1;
        Handle handle = new Handle { Kind = kind, Interactable = interactable, Material = material, Color = color };
        handle.Filter = new XRSelectFilterDelegate((interactor, target) =>
            !RobotCoordinateService.IsCalibrationEditActive && !TargetPoseContextMenu.IsOpen && isActiveAndEnabled && !OtherTargetSelected() &&
            (!_dragging || (_activeHandle == handle && _activeInteractor == interactor)));
        interactable.selectFilters.Add(handle.Filter);
        handle.HoverEntered = args =>
        {
            if (!isActiveAndEnabled) return;
            handle.Hovering.Add(args.interactorObject);
            HoveredTargets.Add(this);
            RefreshColors();
        };
        handle.HoverExited = args =>
        {
            handle.Hovering.Remove(args.interactorObject);
            RefreshHoverState();
        };
        handle.SelectEntered = args => Selected(handle, args);
        handle.SelectExited = args =>
        {
            if (_activeHandle == handle && _activeInteractor == args.interactorObject) EndRotation();
        };
        interactable.hoverEntered.AddListener(handle.HoverEntered);
        interactable.hoverExited.AddListener(handle.HoverExited);
        interactable.selectEntered.AddListener(handle.SelectEntered);
        interactable.selectExited.AddListener(handle.SelectExited);
        _handles.Add(handle);
        return handle;
    }

    private static Mesh CreateCone()
    {
        const int sides = 16;
        Vector3[] vertices = new Vector3[sides + 2];
        int[] triangles = new int[sides * 6];
        vertices[sides] = Vector3.up * 0.02f;
        for (int i = 0; i < sides; i++)
        {
            float angle = i * 2f * Mathf.PI / sides;
            vertices[i] = new Vector3(Mathf.Cos(angle) * 0.009f, 0f, Mathf.Sin(angle) * 0.009f);
            int next = (i + 1) % sides, t = i * 6;
            triangles[t] = i; triangles[t + 1] = sides; triangles[t + 2] = next;
            triangles[t + 3] = sides + 1; triangles[t + 4] = i; triangles[t + 5] = next;
        }
        Mesh mesh = new Mesh { name = "Rotation arrowhead", vertices = vertices, triangles = triangles };
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        return mesh;
    }

    private void RefreshHoverState()
    {
        bool hovering = false;
        foreach (Handle handle in _handles) hovering |= handle.Hovering.Count != 0;
        if (hovering && isActiveAndEnabled) HoveredTargets.Add(this);
        else HoveredTargets.Remove(this);
        RefreshColors();
    }

    private void RefreshColors()
    {
        foreach (Handle handle in _handles)
        {
            if (handle.Material == null) continue;
            float amount = _activeHandle == handle ? 0.65f : handle.Hovering.Count != 0 ? 0.35f : 0f;
            handle.Material.color = Color.Lerp(handle.Color, Color.white, amount);
        }
    }

    private void OnEnable()
    {
        if (_visualRoot != null) _visualRoot.gameObject.SetActive(true);
        RefreshColors();
    }

    private void OnDisable()
    {
        EndRotation();
        HoveredTargets.Remove(this);
        foreach (Handle handle in _handles) handle.Hovering.Clear();
        RefreshColors();
        if (_visualRoot != null) _visualRoot.gameObject.SetActive(false);
    }

    private void OnDestroy()
    {
        SelectedTargets.Remove(this);
        HoveredTargets.Remove(this);
        foreach (Handle handle in _handles)
        {
            if (handle.Interactable != null)
            {
                handle.Interactable.hoverEntered.RemoveListener(handle.HoverEntered);
                handle.Interactable.hoverExited.RemoveListener(handle.HoverExited);
                handle.Interactable.selectEntered.RemoveListener(handle.SelectEntered);
                handle.Interactable.selectExited.RemoveListener(handle.SelectExited);
                handle.Interactable.selectFilters.Remove(handle.Filter);
            }
            Dispose(handle.Material);
        }
        if (_visualRoot != null) Dispose(_visualRoot.gameObject);
        foreach (Mesh mesh in _ownedMeshes) Dispose(mesh);
        RotationStarted = null;
        RotationChanged = null;
    }

    private static void Dispose(UnityEngine.Object value)
    {
        if (value == null) return;
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }
}
