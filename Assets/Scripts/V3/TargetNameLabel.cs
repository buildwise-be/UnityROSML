using TMPro;
using UnityEngine;

/// <summary>A small camera-facing name beside a captured target, independent of its normal.</summary>
[DisallowMultipleComponent]
public sealed class TargetNameLabel : MonoBehaviour
{
    private Transform _target;
    private TextMeshPro _text;
    private Material _outlineMaterial;

    public static TargetNameLabel Attach(Transform target, string caption, Color color)
    {
        if (target == null) return null;
        TargetNameLabel label = target.GetComponentInChildren<TargetNameLabel>(true);
        if (label == null)
        {
            GameObject node = new GameObject("Captured target name", typeof(TextMeshPro), typeof(TargetNameLabel));
            node.layer = 2;
            node.transform.SetParent(target, false);
            label = node.GetComponent<TargetNameLabel>();
        }
        label._target = target;
        label.Configure(caption, color);
        label.Refresh();
        return label;
    }

    private void Configure(string caption, Color color)
    {
        if (_text == null) _text = GetComponent<TextMeshPro>();
        gameObject.layer = 2;
        transform.localScale = Vector3.one * 0.01f;
        _text.text = caption ?? string.Empty;
        _text.fontSize = 30f;
        _text.isOrthographic = false;
        _text.enableAutoSizing = false;
        _text.textWrappingMode = TextWrappingModes.NoWrap;
        _text.overflowMode = TextOverflowModes.Overflow;
        _text.alignment = TextAlignmentOptions.Center;
        _text.raycastTarget = false;
        _text.color = color;
        _text.rectTransform.sizeDelta = new Vector2(18f, 4f);
        if (_outlineMaterial == null && _text.fontSharedMaterial != null)
        {
            _outlineMaterial = _text.fontMaterial;
            _outlineMaterial.SetColor("_OutlineColor", Color.black);
            _outlineMaterial.SetFloat("_OutlineWidth", 0.08f);
            _outlineMaterial.EnableKeyword("OUTLINE_ON");
            _text.UpdateMeshPadding();
        }
    }

    public void Refresh(Camera viewer = null)
    {
        if (_target == null) return;
        if (viewer == null) viewer = Camera.main;
        Vector3 right = viewer != null ? viewer.transform.right : Vector3.right;
        Vector3 up = viewer != null ? viewer.transform.up : Vector3.up;
        transform.SetPositionAndRotation(_target.position + right * 0.10f + up * 0.06f,
            viewer != null ? viewer.transform.rotation : Quaternion.identity);
    }

    private void LateUpdate() => Refresh();

    private void OnDestroy()
    {
        if (_outlineMaterial == null) return;
        if (Application.isPlaying) Destroy(_outlineMaterial);
        else DestroyImmediate(_outlineMaterial);
    }
}
