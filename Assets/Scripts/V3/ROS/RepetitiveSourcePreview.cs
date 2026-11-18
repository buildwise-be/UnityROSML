using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>World-scale source stack and the exact top-face contact centre.</summary>
public sealed class RepetitiveSourcePreview : MonoBehaviour
{
    private readonly List<Transform> _tiles = new List<Transform>();
    private GameObject _root;
    private Material _tileMaterial, _centreMaterial;
    private LineRenderer _crossX, _crossZ;

    public void Show(int count, float side, float thickness, Pose worldTop)
    {
        if (_root == null) CreateRoot();
        _root.transform.SetPositionAndRotation(worldTop.position, worldTop.rotation);
        _root.SetActive(true);
        while (_tiles.Count < count)
        {
            GameObject tile = Primitive("Source tile " + (_tiles.Count + 1), PrimitiveType.Cube, _tileMaterial);
            _tiles.Add(tile.transform);
        }
        for (int index = 0; index < _tiles.Count; index++)
        {
            Transform tile = _tiles[index];
            tile.gameObject.SetActive(index < count);
            tile.localPosition = Vector3.down * (thickness * (index + .5f));
            tile.localScale = new Vector3(side, thickness, side);
        }
        float radius = Mathf.Min(.025f, side * .2f);
        _crossX.SetPosition(0, Vector3.left * radius);
        _crossX.SetPosition(1, Vector3.right * radius);
        _crossZ.SetPosition(0, Vector3.back * radius);
        _crossZ.SetPosition(1, Vector3.forward * radius);
    }

    public void Hide()
    {
        if (_root != null) _root.SetActive(false);
    }

    private void CreateRoot()
    {
        _root = new GameObject("Captured source stack") { layer = 2 };
        _root.transform.SetParent(transform, false);
        _tileMaterial = TargetMarkerMaterials.Create(new Color(.15f, .9f, .4f, .35f), true);
        _centreMaterial = TargetMarkerMaterials.Create(Color.cyan);
        GameObject centre = Primitive("Source pick centre", PrimitiveType.Sphere, _centreMaterial);
        centre.transform.localScale = Vector3.one * .004f;
        TargetNameLabel.Attach(centre.transform, "Pick centre", Color.cyan);
        _crossX = Cross("Pick centre cross X");
        _crossZ = Cross("Pick centre cross Z");
    }

    private GameObject Primitive(string name, PrimitiveType type, Material material)
    {
        GameObject result = GameObject.CreatePrimitive(type);
        result.name = name;
        result.layer = 2;
        result.transform.SetParent(_root.transform, false);
        Collider collider = result.GetComponent<Collider>();
        collider.enabled = false;
        Dispose(collider);
        Renderer renderer = result.GetComponent<Renderer>();
        renderer.sharedMaterial = material;
        renderer.shadowCastingMode = ShadowCastingMode.Off;
        renderer.receiveShadows = false;
        return result;
    }

    private LineRenderer Cross(string name)
    {
        var owner = new GameObject(name, typeof(LineRenderer)) { layer = 2 };
        owner.transform.SetParent(_root.transform, false);
        LineRenderer line = owner.GetComponent<LineRenderer>();
        line.useWorldSpace = false;
        line.positionCount = 2;
        line.startWidth = line.endWidth = .002f;
        line.sharedMaterial = _centreMaterial;
        line.shadowCastingMode = ShadowCastingMode.Off;
        line.receiveShadows = false;
        return line;
    }

    private void OnDisable() => Hide();
    private void OnDestroy()
    {
        Dispose(_root);
        Dispose(_tileMaterial);
        Dispose(_centreMaterial);
    }

    private static void Dispose(Object value)
    {
        if (value == null) return;
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }
}
