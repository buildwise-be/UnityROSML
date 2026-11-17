using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

/// <summary>Render-only tiles sharing one mesh and material per pattern visual.</summary>
public sealed class FloorTilePatternVisual : MonoBehaviour
{
    private static readonly Color GhostColor = new Color(1f, .63f, .14f, .58f);
    private static readonly int[] BoxTriangles =
    {
        0, 2, 1, 0, 3, 2,
        4, 5, 6, 4, 6, 7,
        0, 1, 5, 0, 5, 4,
        1, 2, 6, 1, 6, 5,
        2, 3, 7, 2, 7, 6,
        3, 0, 4, 3, 4, 7
    };

    private readonly List<Vector3> _vertices = new List<Vector3>(8 * FloorTilePattern.MaximumTileCount);
    private readonly List<int> _indices = new List<int>(BoxTriangles.Length * FloorTilePattern.MaximumTileCount);
    private MeshRenderer _renderer;
    private Mesh _mesh;
    private Material _material;
    private FloorTilePattern _renderedPattern;

    public void Show(FloorTilePattern pattern, Pose pose, float physicalScale)
    {
        if (!isActiveAndEnabled || pattern.TileCount == 0 || physicalScale <= 0f ||
            float.IsNaN(physicalScale) || float.IsInfinity(physicalScale))
        {
            Hide();
            return;
        }

        if (_renderer == null) CreateRenderer();
        if (pattern.TileCount != _renderedPattern.TileCount || pattern.TileSide != _renderedPattern.TileSide ||
            pattern.TileThickness != _renderedPattern.TileThickness || pattern.OffsetIndex != _renderedPattern.OffsetIndex ||
            pattern.MortarMillimeters != _renderedPattern.MortarMillimeters)
        {
            RebuildMesh(pattern);
            _renderedPattern = pattern;
        }
        _renderer.transform.SetPositionAndRotation(pose.position, pose.rotation);
        _renderer.transform.localScale = Vector3.one * physicalScale;
        _renderer.enabled = true;
    }

    public void Hide()
    {
        if (_renderer != null) _renderer.enabled = false;
    }

    private void CreateRenderer()
    {
        _material = TargetMarkerMaterials.Create(GhostColor, transparent: true);
        // A scene root keeps world geometry in metres even beside a scaled menu canvas.
        var root = new GameObject("Floor tile pattern", typeof(MeshFilter), typeof(MeshRenderer));
        root.layer = 2;
        if (gameObject.scene.IsValid() && root.scene != gameObject.scene)
            SceneManager.MoveGameObjectToScene(root, gameObject.scene);
        _mesh = new Mesh { name = "Floor tile boxes" };
        root.GetComponent<MeshFilter>().sharedMesh = _mesh;
        _renderer = root.GetComponent<MeshRenderer>();
        _renderer.sharedMaterial = _material;
        _renderer.shadowCastingMode = ShadowCastingMode.Off;
        _renderer.receiveShadows = false;
    }

    private void RebuildMesh(FloorTilePattern pattern)
    {
        _vertices.Clear();
        _indices.Clear();
        float halfSide = pattern.TileSide * .5f;
        float top = pattern.TileThickness;
        for (int index = 0; index < pattern.TileCount; index++)
        {
            Vector3 center = pattern.GetTileCenter(index);
            float left = center.x - halfSide;
            float right = center.x + halfSide;
            float near = center.z - halfSide;
            float far = center.z + halfSide;
            int start = _vertices.Count;
            _vertices.Add(new Vector3(left, 0f, near));
            _vertices.Add(new Vector3(left, 0f, far));
            _vertices.Add(new Vector3(right, 0f, far));
            _vertices.Add(new Vector3(right, 0f, near));
            _vertices.Add(new Vector3(left, top, near));
            _vertices.Add(new Vector3(left, top, far));
            _vertices.Add(new Vector3(right, top, far));
            _vertices.Add(new Vector3(right, top, near));
            foreach (int triangleIndex in BoxTriangles) _indices.Add(start + triangleIndex);
        }
        _mesh.Clear();
        _mesh.SetVertices(_vertices);
        _mesh.SetTriangles(_indices, 0);
    }

    private void OnDisable() => Hide();

    private void OnDestroy()
    {
        if (_renderer != null) Dispose(_renderer.gameObject);
        Dispose(_mesh);
        Dispose(_material);
    }

    private static void Dispose(Object instance)
    {
        if (instance == null) return;
        if (Application.isPlaying) Destroy(instance);
        else DestroyImmediate(instance);
    }
}
