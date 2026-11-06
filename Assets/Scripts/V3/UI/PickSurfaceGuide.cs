using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>Floating geometry-only illustration of the three consecutive item corners.</summary>
[DisallowMultipleComponent]
public sealed class PickSurfaceGuide : MonoBehaviour
{
    private GameObject _geometry;
    private readonly List<GameObject> _markers = new List<GameObject>();
    private readonly List<TextMeshPro> _labels = new List<TextMeshPro>();
    private readonly List<Material> _materials = new List<Material>();
    private TextMeshPro _caption;
    private Material _targetMaterial, _otherMaterial;
    private PickItem? _item;
    private bool _positioned;
    private float _illustrationScale;
    public bool HasVisual => _geometry != null && gameObject.activeInHierarchy;
    public int SelectedPointIndex { get; private set; } = -1;
    public string CurrentPointName { get; private set; } = "";

    public void Refresh(PickItem item, int cornerIndex)
    {
        if (_item != item || _geometry == null) Build(item);
        SelectedPointIndex = Mathf.Clamp(cornerIndex, 0, 2);
        CurrentPointName = SelectedPointIndex == 0 ? "First top corner" : SelectedPointIndex == 1
            ? "Next corner along length" : "Next corner along width";
        for (int i = 0; i < _markers.Count; i++)
            _markers[i].GetComponent<Renderer>().sharedMaterial = i == SelectedPointIndex ? _targetMaterial : _otherMaterial;
        PickItemPreset preset = PickSurfaceGeometry.GetPreset(item);
        _caption.text = preset.Name + " · " + (SelectedPointIndex + 1) + "/3\n" + CurrentPointName;
        gameObject.SetActive(true);
        PositionIfNeeded();
    }

    public void Hide() { SelectedPointIndex = -1; gameObject.SetActive(false); }
    public void Recenter() { _positioned = false; PositionIfNeeded(); }

    private void Build(PickItem item)
    {
        Clear(); _item = item;
        gameObject.layer = 2;
        PickItemPreset preset = PickSurfaceGeometry.GetPreset(item);
        _geometry = new GameObject("Item corner guide geometry"); _geometry.layer = 2;
        _geometry.transform.SetParent(transform, false);
        _illustrationScale = .20f / (float)System.Math.Max(preset.Length, preset.Width);
        _geometry.transform.localScale = Vector3.one * _illustrationScale;
        _geometry.transform.localRotation = Quaternion.Euler(-25f, -35f, 0f);
        Material bodyMaterial = Own(TargetMarkerMaterials.Create(item == PickItem.Tile
            ? new Color(.48f, .72f, .85f) : new Color(.66f, .31f, .18f)));
        _targetMaterial = Own(TargetMarkerMaterials.Create(new Color(1f, .48f, .06f)));
        _otherMaterial = Own(TargetMarkerMaterials.Create(new Color(.32f, .4f, .46f)));
        GameObject body = Primitive("Nominal item body", PrimitiveType.Cube, bodyMaterial, _geometry.transform);
        body.transform.localPosition = Vector3.down * (float)(preset.Thickness / 2d);
        // Canonical FLU/Unity mapping: ROS length X is Unity Z, width Y is Unity -X.
        body.transform.localScale = new Vector3((float)preset.Width, (float)preset.Thickness, (float)preset.Length);
        Vector3[] corners = PickSurfaceGeometry.GetCanonicalCorners(item);
        for (int i = 0; i < 3; i++)
        {
            GameObject marker = Primitive("Capture corner " + (i + 1), PrimitiveType.Sphere, _otherMaterial, _geometry.transform);
            marker.transform.localPosition = corners[i] + Vector3.up * .003f;
            marker.transform.localScale = Vector3.one * (.016f / _illustrationScale);
            _markers.Add(marker);
            TextMeshPro label = Label("Corner number " + (i + 1), _geometry.transform, (i + 1).ToString());
            label.transform.localPosition = corners[i] + Vector3.up * (.032f / _illustrationScale);
            label.transform.localScale = Vector3.one * (.007f / _illustrationScale);
            label.fontSize = 26f; label.rectTransform.sizeDelta = new Vector2(4f, 4f);
            _labels.Add(label);
        }
        _caption = Label("Item capture instruction", transform, "");
        _caption.transform.localPosition = new Vector3(0f, .155f, 0f);
        _caption.transform.localScale = Vector3.one * .01f;
        _caption.fontSize = 21f; _caption.rectTransform.sizeDelta = new Vector2(34f, 8f);
        _labels.Add(_caption);
    }

    private static TextMeshPro Label(string name, Transform parent, string text)
    {
        GameObject node = new GameObject(name, typeof(TextMeshPro)); node.layer = 2;
        node.transform.SetParent(parent, false);
        TextMeshPro label = node.GetComponent<TextMeshPro>();
        label.text = text; label.richText = false; label.raycastTarget = false;
        label.alignment = TextAlignmentOptions.Center;
        label.textWrappingMode = TextWrappingModes.Normal;
        label.color = new Color(.94f, .96f, .98f);
        return label;
    }

    private static GameObject Primitive(string name, PrimitiveType type, Material material, Transform parent)
    {
        GameObject node = GameObject.CreatePrimitive(type); node.name = name; node.layer = 2;
        Collider collider = node.GetComponent<Collider>(); collider.enabled = false; Dispose(collider);
        node.transform.SetParent(parent, false);
        Renderer renderer = node.GetComponent<Renderer>(); renderer.sharedMaterial = material;
        renderer.shadowCastingMode = ShadowCastingMode.Off; renderer.receiveShadows = false;
        return node;
    }

    private void PositionIfNeeded()
    {
        Camera viewer = Camera.main;
        if (_positioned || viewer == null) return;
        Transform tracking = null;
        if (XRTrackingSpaceResolver.TryResolve(ref tracking)) transform.SetParent(tracking, true);
        Vector3 forward = Vector3.ProjectOnPlane(viewer.transform.forward, Vector3.up).normalized;
        if (forward.sqrMagnitude < .1f) forward = viewer.transform.forward;
        Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;
        transform.SetPositionAndRotation(viewer.transform.position + forward * .8f - right * .30f - Vector3.up * .10f,
            Quaternion.LookRotation(forward, Vector3.up));
        _positioned = true;
    }
    private void LateUpdate()
    {
        PositionIfNeeded(); Camera viewer = Camera.main;
        if (viewer != null) foreach (TextMeshPro label in _labels) if (label != null) label.transform.rotation = viewer.transform.rotation;
        if (SelectedPointIndex >= 0 && SelectedPointIndex < _markers.Count)
            _markers[SelectedPointIndex].transform.localScale = Vector3.one * (.016f / _illustrationScale) *
                (1.06f + .06f * Mathf.Sin(Time.unscaledTime * 4f));
    }
    private Material Own(Material material) { _materials.Add(material); return material; }
    private void Clear()
    {
        if (_geometry != null) { _geometry.SetActive(false); Dispose(_geometry); }
        if (_caption != null) { _caption.gameObject.SetActive(false); Dispose(_caption.gameObject); }
        _geometry = null; _caption = null; _markers.Clear(); _labels.Clear();
        foreach (Material material in _materials) Dispose(material);
        _materials.Clear();
    }
    private void OnDestroy() => Clear();
    private static void Dispose(UnityEngine.Object value)
    {
        if (value == null) return;
        if (Application.isPlaying) Destroy(value); else DestroyImmediate(value);
    }
}
