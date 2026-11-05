using System;
using System.Collections.Generic;
using UnityEngine;

public enum PickItem { Tile = 0, Brick = 1 }

/// <summary>Nominal item properties. Mass is a volume/density estimate, not a measurement.</summary>
public readonly struct PickItemPreset
{
    public string Name { get; }
    public double Length { get; }
    public double Width { get; }
    public double Thickness { get; }
    public double Density { get; }
    public double VoidFraction { get; }
    public double MassKg => Length * Width * Thickness * Density * (1d - VoidFraction);
    public PickItemPreset(string name, double length, double width, double thickness, double density, double voidFraction)
    {
        Name = name; Length = length; Width = width; Thickness = thickness;
        Density = density; VoidFraction = voidFraction;
    }
}

/// <summary>Rigid top-surface fit in canonical Unity/FLU axes, with no ROS transport dependencies.</summary>
public static class PickSurfaceGeometry
{
    public const float DefaultMaximumRms = 0.010f;
    public const float DefaultMaximumPointError = 0.015f;

    public static PickItemPreset GetPreset(PickItem item)
    {
        switch (item)
        {
            case PickItem.Tile: return new PickItemPreset("Ceramic tile", .200d, .200d, .007d, 2400d, 0d);
            case PickItem.Brick: return new PickItemPreset("Clay brick", .280d, .185d, .135d, 2000d, .20d);
            default: throw new ArgumentOutOfRangeException(nameof(item), "Choose the tile or brick preset.");
        }
    }

    /// <summary>P0, P1, P2 are consecutive top corners: along length, then along width.</summary>
    public static Vector3[] GetCanonicalCorners(PickItem item)
    {
        PickItemPreset p = GetPreset(item);
        return new[] {
            FromRosPoint(-p.Length / 2d, -p.Width / 2d, 0d),
            FromRosPoint( p.Length / 2d, -p.Width / 2d, 0d),
            FromRosPoint( p.Length / 2d,  p.Width / 2d, 0d)
        };
    }

    /// <summary>ROS object X is length, Y is width, Z is outward from the top surface.</summary>
    public static Vector3 FromRosPoint(double x, double y, double z) => new Vector3((float)-y, (float)z, (float)x);

    /// <summary>Keep tool +X along item length and direct TCP +Z into its top surface.</summary>
    public static Pose GetPickTcpPose(Pose topSurface)
    {
        if (!RobotFrameMath.IsValidPose(topSurface)) throw new ArgumentException("A valid top surface pose is required.");
        return new Pose(topSurface.position, (topSurface.rotation.normalized * Quaternion.AngleAxis(180f, Vector3.forward)).normalized);
    }

    public static bool TryFit(PickItem item, IReadOnlyList<Vector3> measuredPoints,
        out Pose topSurface, out Pose pickTcp, out float rms, out float worst, out string error,
        float maximumRms = DefaultMaximumRms, float maximumPointError = DefaultMaximumPointError)
    {
        topSurface = pickTcp = Pose.identity; rms = worst = 0f; error = null;
        Vector3[] model;
        try { model = GetCanonicalCorners(item); }
        catch (ArgumentOutOfRangeException) { error = "Choose the tile or brick preset."; return false; }
        if (measuredPoints == null || measuredPoints.Count != 3)
        { error = "Capture exactly three consecutive top corners: first corner, along length, then along width."; return false; }
        if (!(maximumRms > 0f) || !(maximumPointError > 0f) || float.IsInfinity(maximumRms) || float.IsInfinity(maximumPointError))
        { error = "Surface-fit tolerances must be finite and positive."; return false; }
        for (int i = 0; i < 3; i++)
        {
            if (!RobotFrameMath.IsValidPose(new Pose(measuredPoints[i], Quaternion.identity)))
            { error = "Corner coordinates must be finite."; return false; }
            for (int j = 0; j < i; j++)
                if (Vector3.Distance(measuredPoints[i], measuredPoints[j]) < .010f)
                { error = "Corners overlap. Touch three different corners in the shown order."; return false; }
        }
        Vector3 lengthEdge = measuredPoints[1] - measuredPoints[0], widthEdge = measuredPoints[2] - measuredPoints[1];
        if (Vector3.Cross(lengthEdge, widthEdge).magnitude <= lengthEdge.magnitude * widthEdge.magnitude * .01f)
        { error = "The three corners are nearly collinear. Follow the length edge, then the width edge."; return false; }
        PickItemPreset preset = GetPreset(item);
        double diagonal = Math.Sqrt(preset.Length * preset.Length + preset.Width * preset.Width);
        if (Math.Abs(lengthEdge.magnitude - preset.Length) > .015d || Math.Abs(widthEdge.magnitude - preset.Width) > .015d ||
            Math.Abs(Vector3.Distance(measuredPoints[0], measuredPoints[2]) - diagonal) > .015d ||
            Math.Abs(Vector3.Dot(lengthEdge.normalized, widthEdge.normalized)) > .15f)
        { error = "Corner distances or the right angle do not match the selected item. Follow the length edge, then the width edge."; return false; }
        if (!RobotPointRegistrationMath.TryFit(model, measuredPoints, out Pose fit, out rms, out worst, out string fitError))
        { error = "The surface corners could not be fitted: " + fitError; return false; }
        if (rms > maximumRms || worst > maximumPointError)
        {
            error = "Surface fit rejected. Check the selected item, corner order and contact tip. RMS " +
                (rms * 1000f).ToString("F1") + " mm; worst " + (worst * 1000f).ToString("F1") + " mm.";
            return false;
        }
        topSurface = fit;
        pickTcp = GetPickTcpPose(fit);
        return true;
    }
}
