using System;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// A TCP point and its physical contact normal. Canonical Unity local +Y is
/// ROS tool +Z; this view never changes the target pose or creates interaction.
/// </summary>
[DisallowMultipleComponent]
public sealed class NormalTargetMarker : MonoBehaviour, IDisposable
{
    public const float BallDiameter = 0.035f;
    public const float ArrowLength = 0.15f;
    private const float ShaftDiameter = 0.006f;
    private const float HeadLength = 0.025f;
    private const float HeadRadius = 0.01f;

    private GameObject _geometry;
    private GameObject _ball;
    private Material _material;
    private Mesh _coneMesh;
    private Color _color = Color.cyan;
    private float _opacity = 1f;
    private bool _ballVisible = true;

    public Vector3 DirectionWorld => transform.up;
    public Vector3 ArrowTipWorld => transform.TransformPoint(Vector3.up * ArrowLength);

    public static NormalTargetMarker Attach(Transform target, Color color, float opacity = 1f)
    {
        if (target == null) return null;
        NormalTargetMarker marker = target.GetComponent<NormalTargetMarker>();
        if (marker == null) marker = target.gameObject.AddComponent<NormalTargetMarker>();
        marker.Configure(color, opacity);
        return marker;
    }

    public void Configure(Color color, float opacity = 1f)
    {
        _color = color;
        _opacity = float.IsNaN(opacity) || float.IsInfinity(opacity) ? 1f : Mathf.Clamp01(opacity);
        EnsureGeometry();
        _geometry.SetActive(true);
        foreach (MeshRenderer renderer in _geometry.GetComponentsInChildren<MeshRenderer>(true))
            renderer.enabled = true;
        ApplyMaterial();
    }

    /// <summary>Captured targets can use their existing highlightable handle ball.</summary>
    public void SetBallVisible(bool visible)
    {
        _ballVisible = visible;
        if (_ball != null) _ball.SetActive(visible);
    }

    private void OnEnable()
    {
        EnsureGeometry();
        ApplyMaterial();
    }

    private void EnsureGeometry()
    {
        if (_geometry != null) return;
        _material = TargetMarkerMaterials.Create(new Color(_color.r, _color.g, _color.b, _opacity), _opacity < 0.999f);
        _material.name = "TCP contact normal marker material";
        _geometry = new GameObject("TCP point and +Z contact normal");
        _geometry.layer = 2;
        _geometry.transform.SetParent(transform, false);

        _ball = Primitive(PrimitiveType.Sphere, "Exact TCP point", Vector3.zero,
            Vector3.one * BallDiameter);
        _ball.SetActive(_ballVisible);
        float shaftStart = BallDiameter * 0.5f;
        float shaftEnd = ArrowLength - HeadLength;
        Primitive(PrimitiveType.Cylinder, "Tool +Z normal shaft",
            Vector3.up * ((shaftStart + shaftEnd) * 0.5f),
            new Vector3(ShaftDiameter, (shaftEnd - shaftStart) * 0.5f, ShaftDiameter));

        GameObject head = new GameObject("Tool +Z normal arrowhead", typeof(MeshFilter), typeof(MeshRenderer));
        head.layer = 2;
        head.transform.SetParent(_geometry.transform, false);
        head.transform.localPosition = Vector3.up * shaftEnd;
        _coneMesh = CreateConeMesh();
        head.GetComponent<MeshFilter>().sharedMesh = _coneMesh;
        ConfigureRenderer(head.GetComponent<MeshRenderer>());
    }

    private GameObject Primitive(PrimitiveType kind, string title, Vector3 position, Vector3 scale)
    {
        GameObject node = GameObject.CreatePrimitive(kind);
        node.name = title;
        node.layer = 2;
        node.transform.SetParent(_geometry.transform, false);
        node.transform.localPosition = position;
        node.transform.localScale = scale;
        Collider collider = node.GetComponent<Collider>();
        if (collider != null)
        {
            collider.enabled = false;
            DisposeOwned(collider);
        }
        ConfigureRenderer(node.GetComponent<MeshRenderer>());
        return node;
    }

    private void ConfigureRenderer(MeshRenderer renderer)
    {
        renderer.sharedMaterial = _material;
        renderer.shadowCastingMode = ShadowCastingMode.Off;
        renderer.receiveShadows = false;
    }

    private void ApplyMaterial()
    {
        if (_material == null) return;
        Color color = new Color(_color.r, _color.g, _color.b, _opacity);
        if (_material.HasProperty("_BaseColor")) _material.SetColor("_BaseColor", color);
        if (_material.HasProperty("_Color")) _material.SetColor("_Color", color);
        bool transparent = _opacity < 0.999f;
        _material.SetOverrideTag("RenderType", transparent ? "Transparent" : "Opaque");
        if (_material.HasProperty("_Surface")) _material.SetFloat("_Surface", transparent ? 1f : 0f);
        if (_material.HasProperty("_SrcBlend")) _material.SetFloat("_SrcBlend", (float)(transparent ? BlendMode.SrcAlpha : BlendMode.One));
        if (_material.HasProperty("_DstBlend")) _material.SetFloat("_DstBlend", (float)(transparent ? BlendMode.OneMinusSrcAlpha : BlendMode.Zero));
        if (_material.HasProperty("_SrcBlendAlpha")) _material.SetFloat("_SrcBlendAlpha", (float)BlendMode.One);
        if (_material.HasProperty("_DstBlendAlpha")) _material.SetFloat("_DstBlendAlpha", (float)(transparent ? BlendMode.OneMinusSrcAlpha : BlendMode.Zero));
        if (_material.HasProperty("_ZWrite")) _material.SetFloat("_ZWrite", transparent ? 0f : 1f);
        if (transparent) _material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        else _material.DisableKeyword("_SURFACE_TYPE_TRANSPARENT");
        _material.DisableKeyword("_ALPHATEST_ON");
        _material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
        _material.renderQueue = (int)(transparent ? RenderQueue.Transparent : RenderQueue.Geometry);
    }

    private static Mesh CreateConeMesh()
    {
        const int sides = 24;
        Vector3[] vertices = new Vector3[sides * 3 + 1];
        int[] triangles = new int[sides * 6];
        for (int index = 0; index < sides; index++)
        {
            float angle = 2f * Mathf.PI * index / sides;
            Vector3 radial = new Vector3(Mathf.Cos(angle) * HeadRadius, 0f, Mathf.Sin(angle) * HeadRadius);
            vertices[index] = radial;
            vertices[sides + index] = Vector3.up * HeadLength;
            vertices[sides * 2 + index] = radial;
            int next = (index + 1) % sides;
            int triangle = index * 6;
            triangles[triangle] = index;
            triangles[triangle + 1] = sides + index;
            triangles[triangle + 2] = next;
            triangles[triangle + 3] = sides * 3;
            triangles[triangle + 4] = sides * 2 + index;
            triangles[triangle + 5] = sides * 2 + next;
        }
        Mesh mesh = new Mesh { name = "Tool +Z contact normal cone", vertices = vertices, triangles = triangles };
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        return mesh;
    }

    public void Dispose()
    {
        if (_geometry != null)
        {
            _geometry.SetActive(false);
            DisposeOwned(_geometry);
            _geometry = null;
        }
        _ball = null;
        if (_material != null) { DisposeOwned(_material); _material = null; }
        if (_coneMesh != null) { DisposeOwned(_coneMesh); _coneMesh = null; }
    }

    private static void DisposeOwned(UnityEngine.Object value)
    {
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }

    private void OnDestroy() => Dispose();
}
