using System;
using NUnit.Framework;
using UnityEngine;

public sealed class PickSurfaceGeometryTests
{
    [TestCase(PickItem.Tile, .200, .200, .007, .672)]
    [TestCase(PickItem.Brick, .280, .185, .135, 11.1888)]
    public void PresetsHaveNominalDimensionsAndEstimatedMass(PickItem item, double length, double width, double thickness, double mass)
    {
        PickItemPreset preset = PickSurfaceGeometry.GetPreset(item);
        Assert.That(preset.Length, Is.EqualTo(length)); Assert.That(preset.Width, Is.EqualTo(width));
        Assert.That(preset.Thickness, Is.EqualTo(thickness)); Assert.That(preset.MassKg, Is.EqualTo(mass).Within(1e-10));
        Assert.That(preset.VoidFraction, Is.EqualTo(item == PickItem.Brick ? .2d : 0d));
    }

    [TestCase(PickItem.Tile)]
    [TestCase(PickItem.Brick)]
    public void ConsecutiveCornersRecoverFullPoseAndInwardToolNormal(PickItem item)
    {
        Pose expected = new Pose(new Vector3(.43f, .27f, -.18f), Quaternion.Euler(37f, -61f, 24f));
        Vector3[] measured = Array.ConvertAll(PickSurfaceGeometry.GetCanonicalCorners(item),
            p => expected.position + expected.rotation * p);
        Assert.That(PickSurfaceGeometry.TryFit(item, measured, out Pose surface, out Pose tcp, out float rms, out float worst, out string error), Is.True, error);
        Assert.That(Vector3.Distance(surface.position, expected.position), Is.LessThan(1e-6f));
        Assert.That(Quaternion.Angle(surface.rotation, expected.rotation), Is.LessThan(.02f));
        Assert.That(rms, Is.LessThan(1e-6f)); Assert.That(worst, Is.LessThan(1e-6f));
        Assert.That(Vector3.Distance(tcp.rotation * Vector3.up, -(surface.rotation * Vector3.up)), Is.LessThan(1e-6f));
        Assert.That(Vector3.Distance(tcp.rotation * Vector3.forward, surface.rotation * Vector3.forward), Is.LessThan(1e-6f));
    }

    [Test]
    public void BrickCornerOrderAndDimensionsAreNotInterchangeable()
    {
        Vector3[] corners = PickSurfaceGeometry.GetCanonicalCorners(PickItem.Brick);
        Assert.That(Vector3.Distance(corners[0], corners[1]), Is.EqualTo(.280f).Within(1e-7f));
        Assert.That(Vector3.Distance(corners[1], corners[2]), Is.EqualTo(.185f).Within(1e-7f));
        Assert.That(PickSurfaceGeometry.TryFit(PickItem.Tile, corners, out _, out _, out _, out _, out _), Is.False);
        Assert.That(PickSurfaceGeometry.TryFit(PickItem.Brick, new[] { corners[0], corners[2], corners[1] }, out _, out _, out _, out _, out _), Is.False);
    }

    [Test]
    public void RejectsRepeatedCollinearNonfiniteAndWrongCountPoints()
    {
        Vector3[] p = PickSurfaceGeometry.GetCanonicalCorners(PickItem.Tile);
        foreach (Vector3[] invalid in new[] {
            new[] { p[0], p[0], p[2] }, new[] { Vector3.zero, Vector3.right, Vector3.right * 2f },
            new[] { p[0], p[1], new Vector3(float.NaN, 0f, 0f) }, new[] { p[0], p[1] }
        }) Assert.That(PickSurfaceGeometry.TryFit(PickItem.Tile, invalid, out _, out _, out _, out _, out _), Is.False);
    }

    [Test]
    public void FitDoesNotScaleToAcceptIncorrectDimensions()
    {
        Vector3[] stretched = Array.ConvertAll(PickSurfaceGeometry.GetCanonicalCorners(PickItem.Tile), p => p * 1.5f);
        Assert.That(PickSurfaceGeometry.TryFit(PickItem.Tile, stretched, out _, out _, out _, out _, out _), Is.False);
    }
}
