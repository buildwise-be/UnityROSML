using TMPro;
using UnityEngine;

/// <summary>Visible OpenXR marker axes after the SDK's Unity basis conversion.</summary>
public sealed class QRMarkerAxes : MonoBehaviour
{
    private readonly Material[] _materials = new Material[3];
    private readonly TextMeshPro[] _labels = new TextMeshPro[3];

    public void Build()
    {
        Vector3[] directions = { Vector3.right, Vector3.up, Vector3.forward };
        Color[] colors = { new Color(1f, 0.2f, 0.2f), new Color(0.2f, 1f, 0.2f), new Color(0.25f, 0.65f, 1f) };
        string[] captions = { "QR +X", "QR +Y", "QR +Z into printed face" };
        Shader shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color");
        for (int i = 0; i < 3; i++)
        {
            Material material = new Material(shader) { color = colors[i] };
            _materials[i] = material;
            GameObject arrow = new GameObject(captions[i], typeof(LineRenderer));
            arrow.layer = 2;
            arrow.transform.SetParent(transform, false);
            LineRenderer line = arrow.GetComponent<LineRenderer>();
            line.useWorldSpace = false;
            line.sharedMaterial = material;
            line.widthMultiplier = 0.0025f;
            Vector3 direction = directions[i];
            Vector3 wing = directions[(i + 1) % 3] * 0.009f;
            Vector3 tip = direction * 0.12f;
            Vector3 neck = direction * 0.102f;
            line.positionCount = 5;
            line.SetPositions(new[] { Vector3.zero, tip, neck + wing, tip, neck - wing });

            GameObject label = new GameObject(captions[i] + " label", typeof(TextMeshPro));
            label.layer = 2;
            label.transform.SetParent(transform, false);
            label.transform.localPosition = direction * 0.14f;
            TextMeshPro text = label.GetComponent<TextMeshPro>();
            text.text = captions[i];
            text.fontSize = 0.11f;
            text.color = colors[i];
            text.alignment = TextAlignmentOptions.Center;
            text.textWrappingMode = TextWrappingModes.NoWrap;
            text.rectTransform.sizeDelta = new Vector2(0.65f, 0.04f);
            _labels[i] = text;
        }
    }

    private void LateUpdate()
    {
        Camera camera = Camera.main;
        if (camera == null) return;
        foreach (TextMeshPro label in _labels)
            if (label != null) label.transform.rotation = camera.transform.rotation;
    }

    private void OnDestroy()
    {
        foreach (Material material in _materials)
            if (material != null)
            {
                if (Application.isPlaying) Destroy(material);
                else DestroyImmediate(material);
            }
    }
}
