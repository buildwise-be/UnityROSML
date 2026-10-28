using System;
using NUnit.Framework;
using UnityEngine;

public sealed class RobotPointRegistrationMathTests
{
    private static readonly Vector3[] Model = {
        new Vector3(0.04f, 0.13f, 0.02f), new Vector3(-0.18f, 0.42f, 0.08f),
        new Vector3(0.14f, 0.66f, -0.12f), new Vector3(0.28f, 0.72f, 0.17f)
    };

    [TestCase(0f, 0f, 0f)]
    [TestCase(180f, 0f, 0f)]
    [TestCase(0f, 180f, 0f)]
    [TestCase(0f, 0f, 180f)]
    [TestCase(37f, -114f, 68f)]
    public void RecoversProperRotationAndTranslation(float x, float y, float z)
    {
        Pose expected = new Pose(new Vector3(2.1f, -0.4f, 3.7f), Quaternion.Euler(x, y, z));
        Vector3[] measured = Transform(Model, expected);
        Assert.That(RobotPointRegistrationMath.TryFit(Model, measured, out Pose fit, out float rms, out float maximum, out string error), Is.True, error);
        Assert.That(Vector3.Distance(fit.position, expected.position), Is.LessThan(0.000002f));
        Assert.That(Vector3.Distance(fit.rotation * Vector3.right, expected.rotation * Vector3.right), Is.LessThan(0.000002f));
        Assert.That(Vector3.Distance(fit.rotation * Vector3.up, expected.rotation * Vector3.up), Is.LessThan(0.000002f));
        Assert.That(rms, Is.LessThan(0.000002f));
        Assert.That(maximum, Is.LessThan(0.000002f));
    }

    [Test]
    public void ThreeNoncollinearPointsInAPlaneAreSufficient()
    {
        Vector3[] planar = { Vector3.zero, Vector3.right * 0.3f, Vector3.up * 0.6f };
        Pose expected = new Pose(new Vector3(-2f, 1f, 4f), Quaternion.Euler(123f, -47f, 12f));
        Assert.That(RobotPointRegistrationMath.TryFit(planar, Transform(planar, expected), out Pose fit, out float rms, out _, out string error), Is.True, error);
        Assert.That(Vector3.Distance(fit.rotation * Vector3.forward, expected.rotation * Vector3.forward), Is.LessThan(0.000002f));
        Assert.That(rms, Is.LessThan(0.000002f));
    }

    [Test]
    public void NoisyOverdeterminedFitUsesAllPairsAndReportsActualResiduals()
    {
        Pose expected = new Pose(new Vector3(1.1f, 0.5f, -2.3f), Quaternion.Euler(12f, 73f, -25f));
        Vector3[] measured = Transform(Model, expected);
        measured[0] += new Vector3(0.004f, -0.002f, 0.001f);
        measured[1] += new Vector3(-0.003f, 0.001f, -0.002f);
        measured[2] += new Vector3(0.002f, 0.003f, 0.001f);
        measured[3] += new Vector3(-0.002f, -0.001f, 0.003f);
        Assert.That(RobotPointRegistrationMath.TryFit(Model, measured, out Pose fit, out float rms, out float maximum, out string error), Is.True, error);
        float actualSquared = 0f, actualMaximum = 0f, firstThreeSquared = 0f;
        Vector3[] firstThreeModel = { Model[0], Model[1], Model[2] };
        Vector3[] firstThreeMeasured = { measured[0], measured[1], measured[2] };
        Assert.That(RobotPointRegistrationMath.TryFit(firstThreeModel, firstThreeMeasured, out Pose threeFit, out _, out _, out error), Is.True, error);
        for (int i = 0; i < Model.Length; i++)
        {
            float residual = Vector3.Distance(fit.position + fit.rotation * Model[i], measured[i]);
            actualSquared += residual * residual;
            actualMaximum = Mathf.Max(actualMaximum, residual);
            firstThreeSquared += (threeFit.position + threeFit.rotation * Model[i] - measured[i]).sqrMagnitude;
        }
        Assert.That(rms, Is.EqualTo(Mathf.Sqrt(actualSquared / Model.Length)).Within(0.000001f));
        Assert.That(maximum, Is.EqualTo(actualMaximum).Within(0.000001f));
        Assert.That(actualSquared, Is.LessThan(firstThreeSquared));
        Assert.That(rms, Is.InRange(0.001f, 0.006f));
    }

    [Test]
    public void ScaleMismatchIsVisibleInResidualAndDoesNotRescaleRobot()
    {
        Vector3[] measured = Array.ConvertAll(Model, point => point * 1.2f + Vector3.one);
        Assert.That(RobotPointRegistrationMath.TryFit(Model, measured, out Pose fit, out float rms, out _, out string error), Is.True, error);
        Assert.That((fit.rotation * (Model[1] - Model[0])).magnitude, Is.EqualTo((Model[1] - Model[0]).magnitude).Within(0.000001f));
        Assert.That(rms, Is.GreaterThan(0.03f));
    }

    [Test]
    public void ReflectedNonplanarPointsCannotBeSolvedWithAReflection()
    {
        Vector3[] tetrahedron = { Vector3.zero, Vector3.right, Vector3.up, Vector3.forward };
        Vector3[] measured = Array.ConvertAll(tetrahedron, point => new Vector3(-point.x, point.y, point.z));
        Assert.That(RobotPointRegistrationMath.TryFit(tetrahedron, measured, out Pose fit, out float rms, out _, out string error), Is.True, error);
        Assert.That(Vector3.Dot(Vector3.Cross(fit.rotation * Vector3.right, fit.rotation * Vector3.up), fit.rotation * Vector3.forward), Is.EqualTo(1f).Within(0.00001f));
        Assert.That(rms, Is.GreaterThan(0.4f));
    }

    [Test]
    public void RejectsMissingMismatchedOrInsufficientPairs()
    {
        AssertRejected(null, Model);
        AssertRejected(Model, null);
        AssertRejected(Model, new[] { Vector3.zero });
        AssertRejected(new[] { Vector3.zero, Vector3.right }, new[] { Vector3.zero, Vector3.right });
    }

    [Test]
    public void RejectsNonfiniteModelAndMeasuredCoordinates()
    {
        foreach (float invalid in new[] { float.NaN, float.PositiveInfinity, float.NegativeInfinity })
        {
            Vector3[] bad = (Vector3[])Model.Clone();
            bad[2] = new Vector3(invalid, 0f, 0f);
            AssertRejected(bad, Model);
            AssertRejected(Model, bad);
        }
    }

    [Test]
    public void RejectsCoincidentCollinearAndNearlyCollinearGeometryOnEitherSide()
    {
        foreach (Vector3[] bad in new[] {
            new[] { Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero },
            new[] { Vector3.zero, Vector3.right, Vector3.right * 2f, Vector3.right * 3f },
            new[] { Vector3.zero, Vector3.right, new Vector3(2f, 0.0001f, 0f), Vector3.right * 3f }
        })
        {
            AssertRejected(bad, Model);
            AssertRejected(Model, bad);
        }
    }

    private static Vector3[] Transform(Vector3[] points, Pose pose) => Array.ConvertAll(points, point => pose.position + pose.rotation * point);

    private static void AssertRejected(Vector3[] model, Vector3[] measured)
    {
        Assert.That(RobotPointRegistrationMath.TryFit(model, measured, out Pose fit, out float rms, out float maximum, out string error), Is.False);
        Assert.That(error, Is.Not.Null.And.Not.Empty);
        Assert.That(fit.position, Is.EqualTo(Vector3.zero));
        Assert.That(fit.rotation, Is.EqualTo(Quaternion.identity));
        Assert.That(rms, Is.Zero);
        Assert.That(maximum, Is.Zero);
    }
}
