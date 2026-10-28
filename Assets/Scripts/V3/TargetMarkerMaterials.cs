using System;
using UnityEngine;

/// <summary>Clones bundled materials that retain opaque and transparent XR shader variants.</summary>
public static class TargetMarkerMaterials
{
    public static Material Create(Color color, bool transparent = false)
    {
        string resource = "RobotTargetMarkers/" + (transparent ? "Transparent" : "Opaque");
        Material template = Resources.Load<Material>(resource);
        if (template == null || template.shader == null)
            throw new InvalidOperationException("Bundled XR target marker material '" + resource +
                "' is missing or invalid. Include Assets/Resources/RobotTargetMarkers and rebuild the app.");
        Material material = new Material(template)
        {
            name = transparent ? "XR target marker transparent instance" : "XR target marker opaque instance",
            enableInstancing = true
        };
        if (!transparent) color.a = 1f;
        material.SetColor("_BaseColor", color);
        material.SetColor("_Color", color);
        return material;
    }
}
