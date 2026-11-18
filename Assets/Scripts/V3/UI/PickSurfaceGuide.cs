using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Rendering;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// <summary>Anchored, double-size explanation of consecutive counterclockwise item contacts.</summary>
[DisallowMultipleComponent]
public sealed class PickSurfaceGuide : MonoBehaviour
{
    private const float IllustrationScale = 2f;
    private const float SequenceSeconds = 10f;
    private const float MarkerDiameter = .018f;
    private static readonly HashSet<PickSurfaceGuide> HoveredGuides = new HashSet<PickSurfaceGuide>();
    private static int _lastControlFrame = -1;
    private readonly List<GameObject> _markers = new List<GameObject>();
    private readonly List<TextMeshPro> _labels = new List<TextMeshPro>();
    private readonly List<Material> _materials = new List<Material>();
    private GameObject _geometry, _roundArrow, _normalArrow, _controls;
    private LineRenderer _roundLine, _roundHead;
    private TextMeshPro _caption;
    private Material _targetMaterial, _otherMaterial;
    private PickItem? _item;
    private PickItemPreset _preset;
    private bool _positioned;
    private float _elapsedSeconds, _arrowRadius;
    private int _lastCueCorner = -1;
    private int _capturedCount;
    private GuideChime _chime;
    private string _captureStatus;
    private Mesh _bodyMesh, _normalHeadMesh;
    private float _viewDistance = 1f;

    public bool HasVisual => _geometry != null && gameObject.activeInHierarchy;
    public int SelectedPointIndex { get; private set; } = -1;
    public string CurrentPointName { get; private set; } = string.Empty;
    public Bounds LayoutBounds { get; private set; }
    public static bool IsPointerOverControls => HoveredGuides.Count != 0 || _lastControlFrame == Time.frameCount;
    public event Action DismissRequested;

    public void Refresh(PickItem item, int cornerIndex) => Refresh(item, cornerIndex, PickSurfaceGeometry.GetPreset(item));

    public void Refresh(PickItem item, int cornerIndex, PickItemPreset preset)
    {
        bool changed = _item != item || _preset.Length != preset.Length || _preset.Width != preset.Width ||
            _preset.Thickness != preset.Thickness || _preset.Name != preset.Name;
        bool opening = !gameObject.activeSelf || changed || _geometry == null;
        if (changed || _geometry == null) Build(item, preset);
        SelectedPointIndex = Mathf.Clamp(cornerIndex, 0, 2);
        _capturedCount = Mathf.Clamp(cornerIndex, 0, 3);
        CurrentPointName = SelectedPointIndex == 0 ? "First top corner" : SelectedPointIndex == 1
            ? "Next corner along length" : "Next corner along width";
        gameObject.SetActive(true);
        if (opening)
        {
            _elapsedSeconds = 0f;
            _lastCueCorner = -1;
            _captureStatus = null;
            _lastControlFrame = Time.frameCount;
        }
        PositionIfNeeded();
        PresentSequence(false);
    }

    public void Hide()
    {
        SelectedPointIndex = -1;
        HoveredGuides.Remove(this);
        gameObject.SetActive(false);
    }

    public void Recenter()
    {
        _positioned = false;
        PositionIfNeeded();
    }

    public void Confirm()
    {
        _lastControlFrame = Time.frameCount;
        Hide();
        DismissRequested?.Invoke();
    }

    public void SetCaptureStatus(string message)
    {
        _captureStatus = message;
        if (_geometry != null) PresentSequence(false);
    }

    private void Build(PickItem item, PickItemPreset preset)
    {
        Clear();
        _item = item;
        _preset = preset;
        gameObject.layer = 2;
        _geometry = new GameObject("Item corner guide geometry") { layer = 2 };
        _geometry.transform.SetParent(transform, false);
        _geometry.transform.localScale = Vector3.one * IllustrationScale;
        Color sideColor = item == PickItem.Tile ? new Color(.23f, .31f, .35f) : new Color(.29f, .23f, .20f);
        Color topColor = item == PickItem.Tile ? new Color(.56f, .66f, .70f) : new Color(.58f, .46f, .38f);
        Material bodyMaterial = Own(TargetMarkerMaterials.Create(sideColor));
        Material endMaterial = Own(TargetMarkerMaterials.Create(sideColor * .8f));
        Material topMaterial = Own(TargetMarkerMaterials.Create(topColor));
        _targetMaterial = Own(TargetMarkerMaterials.Create(new Color(.95f, .57f, .20f)));
        _otherMaterial = Own(TargetMarkerMaterials.Create(new Color(.32f, .4f, .46f)));
        _bodyMesh = BuildBodyMesh();
        GameObject body = MeshObject("Nominal item body", _bodyMesh, _geometry.transform,
            bodyMaterial, endMaterial, topMaterial);
        body.transform.localPosition = Vector3.down * (float)(preset.Thickness / 2d);
        body.transform.localScale = new Vector3((float)preset.Width, (float)preset.Thickness, (float)preset.Length);
        Material edgeMaterial = Own(TargetMarkerMaterials.Create(Color.Lerp(topColor, Color.white, .22f)));
        LineRenderer edge = Line("Top face perimeter", _geometry.transform, edgeMaterial, .0009f);
        float halfWidth = (float)preset.Width * .5f;
        float halfLength = (float)preset.Length * .5f;
        edge.loop = true;
        edge.positionCount = 4;
        edge.SetPositions(new[] {
            new Vector3(-halfWidth, 0f, -halfLength), new Vector3(-halfWidth, 0f, halfLength),
            new Vector3(halfWidth, 0f, halfLength), new Vector3(halfWidth, 0f, -halfLength)
        });
        Vector3[] corners = PickSurfaceGeometry.GetCanonicalCorners(preset);
        for (int i = 0; i < corners.Length; i++)
        {
            GameObject marker = Primitive("Capture corner " + (i + 1), PrimitiveType.Sphere, _otherMaterial, _geometry.transform);
            marker.transform.localPosition = corners[i] + Vector3.up * .003f;
            marker.transform.localScale = Vector3.one * (MarkerDiameter / IllustrationScale);
            _markers.Add(marker);
            TextMeshPro label = Label("Corner number " + (i + 1), _geometry.transform, (i + 1).ToString());
            label.transform.localPosition = corners[i] + Vector3.up * .022f;
            label.transform.localScale = Vector3.one * .0035f;
            label.fontSize = 26f;
            label.rectTransform.sizeDelta = new Vector2(4f, 4f);
            _labels.Add(label);
        }
        _arrowRadius = (float)Math.Min(preset.Length, preset.Width) * .30f;
        BuildRoundArrow();
        BuildNormalArrow();
        _caption = Label("Item capture instruction", transform, string.Empty);
        _caption.transform.localPosition = new Vector3(0f, .28f, 0f);
        _caption.transform.localScale = Vector3.one * .01f;
        _caption.fontSize = 21f;
        _caption.rectTransform.sizeDelta = new Vector2(52f, 15f);
        _caption.enableAutoSizing = true;
        _caption.fontSizeMin = 14f;
        _caption.fontSizeMax = 21f;
        BuildControls();
        Layout(Camera.main);
    }

    private static Mesh BuildBodyMesh()
    {
        var mesh = new Mesh { name = "Item guide box with distinct top and sides", subMeshCount = 3 };
        mesh.vertices = new[] {
            new Vector3(-.5f, -.5f, -.5f), new Vector3(-.5f, -.5f, .5f),
            new Vector3(.5f, -.5f, .5f), new Vector3(.5f, -.5f, -.5f),
            new Vector3(-.5f, .5f, -.5f), new Vector3(-.5f, .5f, .5f),
            new Vector3(.5f, .5f, .5f), new Vector3(.5f, .5f, -.5f)
        };
        mesh.SetTriangles(new[] { 0, 2, 1, 0, 3, 2, 0, 1, 5, 0, 5, 4, 2, 3, 7, 2, 7, 6 }, 0);
        mesh.SetTriangles(new[] { 1, 2, 6, 1, 6, 5, 3, 0, 4, 3, 4, 7 }, 1);
        mesh.SetTriangles(new[] { 4, 5, 6, 4, 6, 7 }, 2);
        mesh.RecalculateNormals();
        return mesh;
    }

    private static GameObject MeshObject(string name, Mesh mesh, Transform parent, params Material[] materials)
    {
        var node = new GameObject(name, typeof(MeshFilter), typeof(MeshRenderer)) { layer = 2 };
        node.transform.SetParent(parent, false);
        node.GetComponent<MeshFilter>().sharedMesh = mesh;
        MeshRenderer renderer = node.GetComponent<MeshRenderer>();
        renderer.sharedMaterials = materials;
        renderer.shadowCastingMode = ShadowCastingMode.Off;
        renderer.receiveShadows = false;
        return node;
    }

    private void BuildRoundArrow()
    {
        _roundArrow = new GameObject("Counterclockwise face arrow") { layer = 2 };
        _roundArrow.transform.SetParent(_geometry.transform, false);
        _roundLine = Line("Round arrow arc", _roundArrow.transform, _targetMaterial, .0025f);
        _roundLine.positionCount = 49;
        _roundHead = Line("Round arrow head", _roundArrow.transform, _targetMaterial, .003f);
        _roundHead.positionCount = 3;
    }

    private void BuildNormalArrow()
    {
        _normalArrow = new GameObject("Downward surface normal") { layer = 2 };
        _normalArrow.transform.SetParent(_geometry.transform, false);
        LineRenderer shaft = Line("Downward normal shaft", _normalArrow.transform, _targetMaterial, .003f);
        shaft.positionCount = 2;
        shaft.SetPositions(new[] { new Vector3(0f, .105f, 0f), new Vector3(0f, .037f, 0f) });
        const int segments = 16;
        var vertices = new Vector3[segments + 2];
        var triangles = new int[segments * 6];
        vertices[segments + 1] = Vector3.up * .025f;
        for (int i = 0; i < segments; i++)
        {
            float angle = i * 2f * Mathf.PI / segments;
            vertices[i + 1] = new Vector3(Mathf.Cos(angle) * .014f, .025f, Mathf.Sin(angle) * .014f);
            int next = (i + 1) % segments + 1;
            int start = i * 6;
            triangles[start] = 0;
            triangles[start + 1] = i + 1;
            triangles[start + 2] = next;
            triangles[start + 3] = segments + 1;
            triangles[start + 4] = next;
            triangles[start + 5] = i + 1;
        }
        _normalHeadMesh = new Mesh { name = "Downward normal cone", vertices = vertices, triangles = triangles };
        _normalHeadMesh.RecalculateNormals();
        GameObject head = MeshObject("Downward normal head", _normalHeadMesh, _normalArrow.transform, _targetMaterial);
        head.transform.localPosition = Vector3.up * .012f;
    }

    private void Layout(Camera viewer)
    {
        float radius = IllustrationScale * .5f * Mathf.Sqrt((float)(_preset.Width * _preset.Width + _preset.Length * _preset.Length));
        float tilt = 35f * Mathf.Deg2Rad;
        float minimumY = -radius * Mathf.Sin(tilt) - IllustrationScale * (float)_preset.Thickness * Mathf.Cos(tilt);
        float maximumY = Mathf.Max(radius * Mathf.Sin(tilt) + .08f, .22f);
        float halfHeight = (maximumY - minimumY) * .5f;
        float halfDepth = radius + Mathf.Max(IllustrationScale * (float)_preset.Thickness * Mathf.Sin(tilt), .12f) + .04f;
        float verticalAngle = Mathf.Clamp((viewer != null ? viewer.fieldOfView : 50f) * .4f, 15f, 32f) * Mathf.Deg2Rad;
        float horizontalAngle = Mathf.Atan(Mathf.Tan(verticalAngle) * (viewer != null ? Mathf.Max(.75f, viewer.aspect) : 1f));
        _viewDistance = Mathf.Max(1f,
            (halfHeight + halfDepth * Mathf.Tan(verticalAngle)) / (Mathf.Tan(verticalAngle) - .144f),
            halfDepth + (radius + .04f) / Mathf.Tan(horizontalAngle));
        _geometry.transform.localPosition = Vector3.up * (-(minimumY + maximumY) * .5f);
        _caption.transform.localScale = Vector3.one * (.01f * _viewDistance);
        _caption.transform.localPosition = Vector3.up * (halfHeight + .115f * _viewDistance);
        _controls.transform.localScale = Vector3.one * (.001f * _viewDistance);
        _controls.transform.localPosition = new Vector3(0f, -halfHeight - .064f * _viewDistance, -.04f);
        float top = halfHeight + .19f * _viewDistance;
        float bottom = -halfHeight - .088f * _viewDistance;
        float halfWidth = Mathf.Max(radius + .04f, .26f * _viewDistance);
        LayoutBounds = new Bounds(new Vector3(0f, (top + bottom) * .5f, 0f),
            new Vector3(halfWidth * 2f, top - bottom, halfDepth * 2f));
    }

    private void BuildControls()
    {
        _controls = new GameObject("Item guide controls", typeof(RectTransform), typeof(Canvas), typeof(TrackedDeviceGraphicRaycaster));
        _controls.transform.SetParent(transform, false);
        _controls.transform.localPosition = new Vector3(0f, -.32f, -.04f);
        _controls.transform.localScale = Vector3.one * .001f;
        Canvas canvas = _controls.GetComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvas.worldCamera = Camera.main;
        canvas.sortingOrder = 30;
        ((RectTransform)_controls.transform).sizeDelta = new Vector2(160f, 48f);

        var node = new GameObject("OK", typeof(RectTransform), typeof(Image), typeof(Button), typeof(EventTrigger));
        RectTransform rect = node.GetComponent<RectTransform>();
        rect.SetParent(_controls.transform, false);
        rect.anchorMin = Vector2.zero;
        rect.anchorMax = Vector2.one;
        rect.offsetMin = rect.offsetMax = Vector2.zero;
        Image image = node.GetComponent<Image>();
        image.color = new Color(.06f, .36f, .46f, 1f);
        Button button = node.GetComponent<Button>();
        button.targetGraphic = image;
        button.onClick.AddListener(Confirm);
        EventTrigger events = node.GetComponent<EventTrigger>();
        var enter = new EventTrigger.Entry { eventID = EventTriggerType.PointerEnter };
        enter.callback.AddListener(_ => HoveredGuides.Add(this));
        events.triggers.Add(enter);
        var exit = new EventTrigger.Entry { eventID = EventTriggerType.PointerExit };
        exit.callback.AddListener(_ => HoveredGuides.Remove(this));
        events.triggers.Add(exit);

        var labelNode = new GameObject("OK label", typeof(RectTransform), typeof(TextMeshProUGUI));
        RectTransform labelRect = labelNode.GetComponent<RectTransform>();
        labelRect.SetParent(rect, false);
        labelRect.anchorMin = Vector2.zero;
        labelRect.anchorMax = Vector2.one;
        labelRect.offsetMin = labelRect.offsetMax = Vector2.zero;
        TextMeshProUGUI label = labelNode.GetComponent<TextMeshProUGUI>();
        label.text = "OK";
        label.alignment = TextAlignmentOptions.Center;
        label.fontSize = 22f;
        label.raycastTarget = false;
    }

    private void PresentSequence(bool sound)
    {
        float time = Mathf.Repeat(_elapsedSeconds, SequenceSeconds);
        _geometry.transform.localRotation = Quaternion.AngleAxis(-35f, Vector3.right) *
            Quaternion.AngleAxis(-25f + _elapsedSeconds * 12f, Vector3.up);
        int corner = time >= 2f && time < 6.5f ? Mathf.Clamp((int)((time - 2f) / 1.5f), 0, 2) : -1;
        for (int i = 0; i < _markers.Count; i++)
        {
            bool visible = corner >= 0 ? i <= corner : time >= 6.5f;
            _markers[i].SetActive(visible);
            _markers[i].GetComponent<Renderer>().sharedMaterial = i == corner ? _targetMaterial : _otherMaterial;
            float pulse = i == corner ? 1.12f + .10f * Mathf.Sin(time * 8f) : 1f;
            _markers[i].transform.localScale = Vector3.one * (MarkerDiameter / IllustrationScale * pulse);
            _labels[i].gameObject.SetActive(visible);
            if (visible) _labels[i].transform.rotation = transform.rotation;
        }
        if (sound && corner >= 0 && corner != _lastCueCorner)
        {
            if (_chime == null) _chime = gameObject.AddComponent<GuideChime>();
            _chime.PlayCue();
        }
        if (sound) _lastCueCorner = corner;
        _roundArrow.SetActive(time >= 6.5f && time < 8.5f);
        _normalArrow.SetActive(time >= 8.5f);
        if (_roundArrow.activeSelf) DrawRoundArrow((time - 6.5f) / 2f);
        string instruction = time < 2f ? "Capture the top face" : corner >= 0
            ? "Touch corner " + (corner + 1) + " and press trigger"
            : time < 8.5f ? "Three consecutive corners · counterclockwise" : "The pick approaches into the top face";
        _caption.text = _preset.Name + " · " + _capturedCount + "/3 captured\n" +
            (string.IsNullOrEmpty(_captureStatus) ? instruction : _captureStatus);
    }

    private void DrawRoundArrow(float progress)
    {
        float endAngle = Mathf.Lerp(-45f, 270f, progress) * Mathf.Deg2Rad;
        for (int i = 0; i < _roundLine.positionCount; i++)
        {
            float angle = Mathf.Lerp(-45f * Mathf.Deg2Rad, endAngle, i / (float)(_roundLine.positionCount - 1));
            _roundLine.SetPosition(i, new Vector3(Mathf.Cos(angle) * _arrowRadius, .022f, Mathf.Sin(angle) * _arrowRadius));
        }
        Vector3 tip = new Vector3(Mathf.Cos(endAngle) * _arrowRadius, .022f, Mathf.Sin(endAngle) * _arrowRadius);
        Vector3 tangent = new Vector3(-Mathf.Sin(endAngle), 0f, Mathf.Cos(endAngle));
        Vector3 side = new Vector3(Mathf.Cos(endAngle), 0f, Mathf.Sin(endAngle));
        _roundHead.SetPositions(new[] { tip - tangent * .012f + side * .008f, tip, tip - tangent * .012f - side * .008f });
    }

    private static LineRenderer Line(string name, Transform parent, Material material, float width)
    {
        var node = new GameObject(name, typeof(LineRenderer)) { layer = 2 };
        node.transform.SetParent(parent, false);
        LineRenderer line = node.GetComponent<LineRenderer>();
        line.useWorldSpace = false;
        line.sharedMaterial = material;
        line.startWidth = line.endWidth = width;
        line.shadowCastingMode = ShadowCastingMode.Off;
        line.receiveShadows = false;
        return line;
    }

    private static TextMeshPro Label(string name, Transform parent, string text)
    {
        GameObject node = new GameObject(name, typeof(TextMeshPro)) { layer = 2 };
        node.transform.SetParent(parent, false);
        TextMeshPro label = node.GetComponent<TextMeshPro>();
        label.text = text;
        label.richText = false;
        label.raycastTarget = false;
        label.alignment = TextAlignmentOptions.Center;
        label.textWrappingMode = TextWrappingModes.Normal;
        label.color = new Color(.94f, .96f, .98f);
        return label;
    }

    private static GameObject Primitive(string name, PrimitiveType type, Material material, Transform parent)
    {
        GameObject node = GameObject.CreatePrimitive(type);
        node.name = name;
        node.layer = 2;
        Collider collider = node.GetComponent<Collider>();
        collider.enabled = false;
        Dispose(collider);
        node.transform.SetParent(parent, false);
        Renderer renderer = node.GetComponent<Renderer>();
        renderer.sharedMaterial = material;
        renderer.shadowCastingMode = ShadowCastingMode.Off;
        renderer.receiveShadows = false;
        return node;
    }

    private void PositionIfNeeded()
    {
        Camera viewer = Camera.main;
        if (_positioned || viewer == null) return;
        Transform tracking = null;
        if (XRTrackingSpaceResolver.TryResolve(ref tracking)) transform.SetParent(tracking, true);
        Layout(viewer);
        Quaternion facing = viewer.transform.rotation;
        Vector3 center = viewer.transform.position + viewer.transform.forward * _viewDistance;
        transform.SetPositionAndRotation(center - facing * LayoutBounds.center, facing);
        transform.localScale = Vector3.one;
        _positioned = true;
    }

    private void LateUpdate()
    {
        PositionIfNeeded();
        if (_geometry == null || SelectedPointIndex < 0) return;
        _elapsedSeconds += Time.unscaledDeltaTime;
        PresentSequence(true);
    }

    private Material Own(Material material)
    {
        _materials.Add(material);
        return material;
    }

    private void Clear()
    {
        if (_geometry != null) { _geometry.SetActive(false); Dispose(_geometry); }
        if (_caption != null) { _caption.gameObject.SetActive(false); Dispose(_caption.gameObject); }
        if (_controls != null) { _controls.SetActive(false); Dispose(_controls); }
        _geometry = _roundArrow = _normalArrow = _controls = null;
        _caption = null;
        _markers.Clear();
        _labels.Clear();
        foreach (Material material in _materials) Dispose(material);
        _materials.Clear();
        Dispose(_bodyMesh);
        Dispose(_normalHeadMesh);
        _bodyMesh = _normalHeadMesh = null;
        HoveredGuides.Remove(this);
    }

    private void OnDisable() => HoveredGuides.Remove(this);
    private void OnDestroy() => Clear();

    private static void Dispose(UnityEngine.Object value)
    {
        if (value == null) return;
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }
}
