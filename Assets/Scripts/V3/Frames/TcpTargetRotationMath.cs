using System;
using UnityEngine;

/// <summary>Rotation-only handle calculations. Target translation is never an output.</summary>
public static class TcpTargetRotationMath
{
    private const float DirectionTolerance = 0.000001f;

    public static bool TrySignedPlaneAngle(Vector3 previous, Vector3 current, Vector3 axis, out float degrees)
    {
        degrees = 0f;
        if (!TryUnitVector(axis, out Vector3 normal) || !IsFinite(previous) || !IsFinite(current))
            return false;

        Vector3 projectedPrevious = previous - normal * Vector3.Dot(previous, normal);
        Vector3 projectedCurrent = current - normal * Vector3.Dot(current, normal);
        if (!TryUnitVector(projectedPrevious, out Vector3 from) || !TryUnitVector(projectedCurrent, out Vector3 to))
            return false;

        degrees = Mathf.Atan2(Vector3.Dot(normal, Vector3.Cross(from, to)), Vector3.Dot(from, to)) * Mathf.Rad2Deg;
        return IsFinite(degrees);
    }

    /// <summary>Intersect a forward ray with a handle's captured world-space rotation plane.</summary>
    public static bool TryRayPlaneIntersection(Ray ray, Vector3 planePoint, Vector3 planeNormal, out Vector3 hit)
    {
        hit = default;
        if (!IsFinite(ray.origin) || !IsFinite(planePoint) ||
            !TryUnitVector(ray.direction, out Vector3 direction) || !TryUnitVector(planeNormal, out Vector3 normal))
            return false;

        float denominator = Vector3.Dot(direction, normal);
        if (Mathf.Abs(denominator) < DirectionTolerance)
            return false;

        float distance = Vector3.Dot(planePoint - ray.origin, normal) / denominator;
        if (!IsFinite(distance) || distance < 0f)
            return false;

        hit = ray.origin + direction * distance;
        return IsFinite(hit);
    }

    /// <summary>Rotate the initial target around an axis captured in world space.</summary>
    public static Quaternion ApplyWorldAxisDelta(Quaternion initial, Vector3 worldAxis, float degrees)
    {
        RequireValidRotation(initial, nameof(initial));
        if (!TryUnitVector(worldAxis, out Vector3 axis))
            throw new ArgumentException("A rotation axis must be finite and nonzero.", nameof(worldAxis));
        if (!IsFinite(degrees))
            throw new ArgumentException("A rotation angle must be finite.", nameof(degrees));

        return Normalize(Quaternion.AngleAxis(degrees, axis) * Normalize(initial));
    }

    /// <summary>Apply the controller's world-space rotation delta to the initial target.</summary>
    public static Quaternion ApplyControllerDelta(Quaternion initialTargetRotation,
        Quaternion initialControllerRotation, Quaternion currentControllerRotation)
    {
        RequireValidRotation(initialTargetRotation, nameof(initialTargetRotation));
        RequireValidRotation(initialControllerRotation, nameof(initialControllerRotation));
        RequireValidRotation(currentControllerRotation, nameof(currentControllerRotation));
        Quaternion delta = Normalize(currentControllerRotation) * Quaternion.Inverse(Normalize(initialControllerRotation));
        return Normalize(delta * Normalize(initialTargetRotation));
    }

    private static bool TryUnitVector(Vector3 value, out Vector3 unit)
    {
        unit = default;
        if (!IsFinite(value)) return false;
        double magnitude = Math.Sqrt((double)value.x * value.x + (double)value.y * value.y + (double)value.z * value.z);
        if (magnitude < DirectionTolerance) return false;
        unit = new Vector3((float)(value.x / magnitude), (float)(value.y / magnitude), (float)(value.z / magnitude));
        return true;
    }

    private static bool IsFinite(Vector3 value)
    {
        return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private static Quaternion Normalize(Quaternion q)
    {
        double magnitude = Math.Sqrt((double)q.x * q.x + (double)q.y * q.y + (double)q.z * q.z + (double)q.w * q.w);
        return new Quaternion((float)(q.x / magnitude), (float)(q.y / magnitude),
            (float)(q.z / magnitude), (float)(q.w / magnitude));
    }

    private static void RequireValidRotation(Quaternion rotation, string parameterName)
    {
        if (!RobotFrameMath.IsValidPose(new Pose(Vector3.zero, rotation)))
            throw new ArgumentException("A rotation must be finite and nonzero.", parameterName);
    }
}
