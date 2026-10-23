using System;
using NUnit.Framework;
using UnityEngine;

public class TcpTargetRotationMathTests
{
    private const float Tolerance = 0.00001f;

    [Test]
    public void PlaneAngleKeepsItsSignAndIgnoresMotionAlongTheAxis()
    {
        Assert.That(TcpTargetRotationMath.TrySignedPlaneAngle(new Vector3(2f, 0f, 7f),
            new Vector3(0f, 3f, -4f), Vector3.forward, out float angle), Is.True);
        Assert.That(angle, Is.EqualTo(90f).Within(Tolerance));
        Assert.That(TcpTargetRotationMath.TrySignedPlaneAngle(Vector3.up, Vector3.right,
            Vector3.forward, out angle), Is.True);
        Assert.That(angle, Is.EqualTo(-90f).Within(Tolerance));
    }

    [Test]
    public void ReversingThePlaneNormalReversesTheRotation()
    {
        Assert.That(TcpTargetRotationMath.TrySignedPlaneAngle(Vector3.right, Vector3.up,
            Vector3.back, out float angle), Is.True);
        Assert.That(angle, Is.EqualTo(-90f).Within(Tolerance));
    }

    [Test]
    public void OppositePlaneDirectionsProduceAHalfTurn()
    {
        Assert.That(TcpTargetRotationMath.TrySignedPlaneAngle(Vector3.right, Vector3.left,
            Vector3.forward, out float angle), Is.True);
        Assert.That(Mathf.Abs(angle), Is.EqualTo(180f).Within(Tolerance));
    }

    [Test]
    public void UndefinedPlaneAnglesAreRejected()
    {
        Assert.That(TcpTargetRotationMath.TrySignedPlaneAngle(Vector3.right, Vector3.up, Vector3.zero, out _), Is.False);
        Assert.That(TcpTargetRotationMath.TrySignedPlaneAngle(Vector3.forward, Vector3.up, Vector3.forward, out _), Is.False);
        Assert.That(TcpTargetRotationMath.TrySignedPlaneAngle(Vector3.zero, Vector3.right, Vector3.forward, out _), Is.False);
        Assert.That(TcpTargetRotationMath.TrySignedPlaneAngle(new Vector3(float.NaN, 0f, 0f),
            Vector3.right, Vector3.forward, out _), Is.False);
    }

    [Test]
    public void RayPlaneIntersectionUsesTheRayOriginAndPlaneOffset()
    {
        Ray ray = new Ray(new Vector3(3f, 4f, -2f), Vector3.forward);
        Assert.That(TcpTargetRotationMath.TryRayPlaneIntersection(ray, new Vector3(0f, 0f, 5f),
            Vector3.back, out Vector3 hit), Is.True);
        AssertVector(new Vector3(3f, 4f, 5f), hit);
    }

    [Test]
    public void ParallelAndBackwardIntersectionsAreRejected()
    {
        Ray ray = new Ray(Vector3.zero, Vector3.forward);
        Assert.That(TcpTargetRotationMath.TryRayPlaneIntersection(ray, Vector3.right, Vector3.right, out _), Is.False);
        Assert.That(TcpTargetRotationMath.TryRayPlaneIntersection(ray, Vector3.back, Vector3.forward, out _), Is.False);
        Assert.That(TcpTargetRotationMath.TryRayPlaneIntersection(ray, Vector3.zero, Vector3.zero, out _), Is.False);
        Assert.That(TcpTargetRotationMath.TryRayPlaneIntersection(new Ray(Vector3.zero, Vector3.zero),
            Vector3.forward, Vector3.forward, out _), Is.False);
    }

    [Test]
    public void NonfiniteRayPlaneInputsAreRejected()
    {
        Ray ray = new Ray(new Vector3(float.PositiveInfinity, 0f, 0f), Vector3.forward);
        Assert.That(TcpTargetRotationMath.TryRayPlaneIntersection(ray, Vector3.zero, Vector3.forward, out _), Is.False);
        Assert.That(TcpTargetRotationMath.TryRayPlaneIntersection(new Ray(Vector3.zero, Vector3.forward),
            new Vector3(0f, float.NaN, 0f), Vector3.forward, out _), Is.False);
    }

    [Test]
    public void AWorldAxisDeltaIsAppliedBeforeTheInitialRotation()
    {
        Quaternion initial = Quaternion.Euler(0f, 90f, 0f);
        Quaternion result = TcpTargetRotationMath.ApplyWorldAxisDelta(initial, Vector3.forward, 90f);
        AssertVector(Vector3.up, result * Vector3.forward);
    }

    [TestCase(0f, 0f, 0f)]
    [TestCase(30f, -45f, 70f)]
    [TestCase(-65f, 120f, 25f)]
    public void TwistingAroundTheTcpNormalPreservesItsDirection(float x, float y, float z)
    {
        Quaternion initial = Quaternion.Euler(x, y, z);
        Vector3 normal = initial * Vector3.up;
        Quaternion result = TcpTargetRotationMath.ApplyWorldAxisDelta(initial, normal, 65f);
        AssertVector(normal, result * Vector3.up);
        Assert.That(Vector3.Angle(initial * Vector3.forward, result * Vector3.forward),
            Is.EqualTo(65f).Within(0.0001f));
    }

    [TestCase(0f, 0f, 0f)]
    [TestCase(30f, -45f, 70f)]
    public void TiltUsesTheTargetsCapturedLocalXAxisInWorldSpace(float x, float y, float z)
    {
        Quaternion initial = Quaternion.Euler(x, y, z);
        // Canonical Unity +Z is ROS tool +X, the local tilt-ring axis.
        Quaternion result = TcpTargetRotationMath.ApplyWorldAxisDelta(initial, initial * Vector3.forward, 90f);
        AssertVector(-(initial * Vector3.right), result * Vector3.up);
    }

    [Test]
    public void ControllerDeltaIsRelativeToTheCapturedControllerPose()
    {
        Quaternion initialTarget = Quaternion.Euler(25f, -15f, 40f);
        Quaternion initialController = Quaternion.Euler(10f, 20f, 30f);
        Quaternion delta = Quaternion.AngleAxis(45f, Vector3.up);
        Quaternion result = TcpTargetRotationMath.ApplyControllerDelta(initialTarget,
            initialController, delta * initialController);
        AssertVector(delta * (initialTarget * Vector3.up), result * Vector3.up);
        AssertVector(delta * (initialTarget * Vector3.forward), result * Vector3.forward);
    }

    [Test]
    public void NoControllerRotationMeansNoTargetRotation()
    {
        Quaternion initialTarget = Quaternion.Euler(25f, -15f, 40f);
        Quaternion controller = Quaternion.Euler(10f, 20f, 30f);
        Quaternion result = TcpTargetRotationMath.ApplyControllerDelta(initialTarget, controller, controller);
        Assert.That(Mathf.Abs(Quaternion.Dot(initialTarget, result)), Is.GreaterThan(1f - Tolerance));
    }

    [Test]
    public void ValidNonunitControllerQuaternionsAreNormalized()
    {
        Quaternion initial = Quaternion.Euler(15f, 25f, 35f);
        Quaternion previous = Quaternion.Euler(-10f, 40f, 20f);
        Quaternion current = Quaternion.Euler(50f, -20f, 70f);
        Quaternion unitResult = TcpTargetRotationMath.ApplyControllerDelta(initial, previous, current);
        Quaternion result = TcpTargetRotationMath.ApplyControllerDelta(Scaled(initial, 2f),
            Scaled(previous, 3f), Scaled(current, 4f));
        Assert.That(Mathf.Abs(Quaternion.Dot(unitResult, result)), Is.GreaterThan(1f - Tolerance));
        Assert.That(Quaternion.Dot(result, result), Is.EqualTo(1f).Within(Tolerance));
    }

    [Test]
    public void InvalidRotationRequestsAreRejected()
    {
        Assert.Throws<ArgumentException>(() => TcpTargetRotationMath.ApplyWorldAxisDelta(Quaternion.identity, Vector3.zero, 20f));
        Assert.Throws<ArgumentException>(() => TcpTargetRotationMath.ApplyWorldAxisDelta(Quaternion.identity, Vector3.up, float.NaN));
        Assert.Throws<ArgumentException>(() => TcpTargetRotationMath.ApplyControllerDelta(Quaternion.identity, default, Quaternion.identity));
    }

    private static Quaternion Scaled(Quaternion q, float factor)
    {
        return new Quaternion(q.x * factor, q.y * factor, q.z * factor, q.w * factor);
    }

    private static void AssertVector(Vector3 expected, Vector3 actual)
    {
        Assert.That(Vector3.Distance(expected, actual), Is.LessThan(Tolerance),
            "Expected " + expected + ", received " + actual);
    }
}
