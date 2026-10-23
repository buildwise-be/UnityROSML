using System;
using NUnit.Framework;
using UnityEngine;

public class RobotFrameMathTests
{
    private const float Tolerance = 0.00001f;

    [Test]
    public void ComposeRotatesTheOffsetBeforeAddingTheOrigin()
    {
        Pose frame = new Pose(new Vector3(10f, 20f, 30f), Quaternion.Euler(0f, 90f, 0f));
        Pose result = RobotFrameMath.Compose(frame, new Pose(new Vector3(1f, 2f, 3f), Quaternion.identity));

        AssertVector(new Vector3(13f, 22f, 29f), result.position);
        AssertRotation(frame.rotation, result.rotation);
    }

    [Test]
    public void RelativeToUsesTheRobotOriginAndItsInverseRotation()
    {
        Pose robot = new Pose(new Vector3(3f, 0f, 5f), Quaternion.Euler(0f, 90f, 0f));
        Quaternion localRotation = Quaternion.Euler(30f, 0f, 0f);
        Pose world = new Pose(new Vector3(5f, 1f, 4f), robot.rotation * localRotation);

        Pose relative = RobotFrameMath.RelativeTo(robot, world);
        AssertVector(new Vector3(1f, 1f, 2f), relative.position);
        AssertRotation(localRotation, relative.rotation);

        Pose reconstructed = RobotFrameMath.Compose(robot, relative);
        AssertVector(world.position, reconstructed.position);
        AssertRotation(world.rotation, reconstructed.rotation);
    }

    [Test]
    public void MarkerExtrinsicIncludesRotationAndTranslation()
    {
        Pose worldBase = new Pose(new Vector3(2f, 3f, 4f), Quaternion.Euler(20f, 50f, -10f));
        Pose baseToMarker = new Pose(new Vector3(0.4f, -0.2f, 0.6f), Quaternion.Euler(-90f, 0f, 15f));
        Pose worldMarker = RobotFrameMath.Compose(worldBase, baseToMarker);

        Pose recoveredBase = RobotFrameMath.Compose(worldMarker, RobotFrameMath.Inverse(baseToMarker));
        AssertVector(worldBase.position, recoveredBase.position);
        AssertRotation(worldBase.rotation, recoveredBase.rotation);
    }

    [Test]
    public void MarkerOffsetIsNotSubtractedInWorldAxes()
    {
        Pose worldMarker = new Pose(new Vector3(5f, 2f, 3f), Quaternion.Euler(0f, 90f, 0f));
        Pose baseToMarker = new Pose(Vector3.right, Quaternion.identity);
        Pose worldBase = RobotFrameMath.Compose(worldMarker, RobotFrameMath.Inverse(baseToMarker));

        AssertVector(new Vector3(5f, 2f, 4f), worldBase.position);
        AssertRotation(worldMarker.rotation, worldBase.rotation);
    }

    [Test]
    public void TrackingPoseUsesCameraOffsetFrameInsteadOfAssumingWorldSpace()
    {
        GameObject trackingObject = new GameObject("Tracking space test");
        try
        {
            trackingObject.transform.SetPositionAndRotation(new Vector3(4f, 1f, -2f), Quaternion.Euler(0f, 90f, 0f));
            Pose trackingPose = new Pose(new Vector3(1f, 2f, 3f), Quaternion.Euler(15f, 20f, 30f));

            Assert.That(RobotFrameMath.TryTrackingToWorld(trackingObject.transform, trackingPose, out Pose world), Is.True);
            AssertVector(new Vector3(7f, 3f, -3f), world.position);
            AssertVector(trackingObject.transform.rotation * (trackingPose.rotation * Vector3.forward),
                world.rotation * Vector3.forward);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(trackingObject);
        }
    }

    [TestCase(2f, 1f, 1f)]
    [TestCase(0.001f, 0.001f, 0.001f)]
    [TestCase(-1f, 1f, 1f)]
    [TestCase(-1f, -1f, 1f)]
    public void ScaledAndReflectedTrackingFramesAreRejected(float x, float y, float z)
    {
        GameObject trackingObject = new GameObject("Invalid tracking frame");
        try
        {
            trackingObject.transform.localScale = new Vector3(x, y, z);
            Assert.That(RobotFrameMath.IsRigidFrame(trackingObject.transform), Is.False);
            Assert.That(RobotFrameMath.TryTrackingToWorld(trackingObject.transform, Pose.identity, out _), Is.False);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(trackingObject);
        }
    }

    [Test]
    public void ShearedHierarchyIsRejected()
    {
        GameObject parent = new GameObject("Scaled parent");
        GameObject child = new GameObject("Rotated child");
        try
        {
            child.transform.SetParent(parent.transform, false);
            parent.transform.localScale = new Vector3(2f, 1f, 1f);
            child.transform.localRotation = Quaternion.Euler(0f, 0f, 45f);
            Assert.That(RobotFrameMath.IsRigidFrame(child.transform), Is.False);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(parent);
        }
    }

    [Test]
    public void MissingOrInvalidTrackingInputsAreRejected()
    {
        GameObject trackingObject = new GameObject("Tracking frame");
        try
        {
            Assert.That(RobotFrameMath.TryTrackingToWorld(null, Pose.identity, out _), Is.False);
            Assert.That(RobotFrameMath.TryTrackingToWorld(trackingObject.transform,
                new Pose(Vector3.zero, new Quaternion(0f, 0f, 0f, 0f)), out _), Is.False);
            Assert.That(RobotFrameMath.TryTrackingToWorld(trackingObject.transform,
                new Pose(new Vector3(float.NaN, 0f, 0f), Quaternion.identity), out _), Is.False);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(trackingObject);
        }
    }

    [Test]
    public void PoseValidationRejectsNonfiniteAndZeroQuaternions()
    {
        Assert.That(RobotFrameMath.IsValidPose(Pose.identity), Is.True);
        Assert.That(RobotFrameMath.IsValidPose(default), Is.False);
        Assert.That(RobotFrameMath.IsValidPose(new Pose(Vector3.zero, new Quaternion(float.NaN, 0f, 0f, 1f))), Is.False);
        Assert.That(RobotFrameMath.IsValidPose(new Pose(new Vector3(0f, float.PositiveInfinity, 0f), Quaternion.identity)), Is.False);
        Assert.Throws<ArgumentException>(() => RobotFrameMath.Compose(Pose.identity, default));
    }

    [Test]
    public void CompositionNormalizesValidNonunitQuaternions()
    {
        Quaternion rotation = Quaternion.Euler(0f, 90f, 0f);
        Quaternion scaledRotation = new Quaternion(rotation.x * 2f, rotation.y * 2f, rotation.z * 2f, rotation.w * 2f);
        Pose result = RobotFrameMath.Compose(new Pose(Vector3.zero, scaledRotation), new Pose(Vector3.forward, Quaternion.identity));

        AssertVector(Vector3.right, result.position);
        Assert.That(Mathf.Abs(Quaternion.Dot(result.rotation, result.rotation) - 1f), Is.LessThan(Tolerance));
    }

    [TestCase(0f, 0f, 0f)]
    [TestCase(0f, 90f, 0f)]
    [TestCase(30f, -45f, 70f)]
    [TestCase(-90f, 180f, -120f)]
    public void TcpPositiveZAndRosMessageNormalBothFollowControllerAim(float x, float y, float z)
    {
        Pose pointer = new Pose(new Vector3(0.3f, 0.2f, -0.4f), Quaternion.Euler(x, y, z));
        Pose tcp = RobotFrameMath.PointerToTcpPose(pointer);
        Vector3 visualNormal = RobotFrameMath.TcpPositiveZ(tcp);

        AssertVector(pointer.position, tcp.position);
        AssertVector(pointer.rotation * Vector3.forward, visualNormal);

        // ROSGeometry's established RUF->FLU basis maps (x,y,z)->(z,-x,y).
        // Verify ROS TCP +Z against the visible normal, rather than quaternion
        // components (q and -q describe exactly the same physical rotation).
        Quaternion q = tcp.rotation;
        Quaternion rosRotation = new Quaternion(q.z, -q.x, q.y, -q.w);
        Vector3 rosNormal = rosRotation * Vector3.forward;
        AssertVector(new Vector3(visualNormal.z, -visualNormal.x, visualNormal.y), rosNormal);
    }

    private static void AssertVector(Vector3 expected, Vector3 actual)
    {
        Assert.That(Vector3.Distance(expected, actual), Is.LessThan(Tolerance),
            "Expected " + expected + ", received " + actual);
    }

    private static void AssertRotation(Quaternion expected, Quaternion actual)
    {
        Assert.That(Mathf.Abs(Quaternion.Dot(expected.normalized, actual.normalized)),
            Is.GreaterThan(1f - Tolerance));
    }
}
