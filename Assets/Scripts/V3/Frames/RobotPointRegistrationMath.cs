using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>Least-squares proper rigid registration of corresponding points, in metres.</summary>
public static class RobotPointRegistrationMath
{
    // A line does not constrain rotation about itself. Require measurable transverse
    // spread and reject configurations whose second principal spread is under 1% of the first.
    private const double MinimumSecondVariance = 0.000001; // (1 mm)^2
    private const double MinimumVarianceRatio = 0.0001;

    /// <summary>
    /// Fits measured[i] = rotation * model[i] + translation using all ordered pairs.
    /// Never estimates scale or reflection. A high residual is returned for the caller
    /// to assess against its measurement tolerance; it is not silently discarded.
    /// </summary>
    public static bool TryFit(IReadOnlyList<Vector3> modelPoints, IReadOnlyList<Vector3> measuredPoints,
        out Pose modelToMeasured, out float rmsErrorMeters, out float maxErrorMeters, out string error)
    {
        modelToMeasured = Pose.identity;
        rmsErrorMeters = maxErrorMeters = 0f;
        error = null;
        if (modelPoints == null || measuredPoints == null || modelPoints.Count != measuredPoints.Count || modelPoints.Count < 3)
        {
            error = "At least three corresponding model and measured points are required.";
            return false;
        }

        int count = modelPoints.Count;
        double[] sourceCenter = new double[3], targetCenter = new double[3];
        for (int i = 0; i < count; i++)
        {
            if (!Finite(modelPoints[i]) || !Finite(measuredPoints[i]))
            {
                error = "Registration points must contain finite coordinates.";
                return false;
            }
            for (int axis = 0; axis < 3; axis++)
            {
                sourceCenter[axis] += modelPoints[i][axis] / (double)count;
                targetCenter[axis] += measuredPoints[i][axis] / (double)count;
            }
        }

        double[,] cross = new double[3, 3];
        double[,] sourceCovariance = new double[3, 3], targetCovariance = new double[3, 3];
        for (int i = 0; i < count; i++)
            for (int row = 0; row < 3; row++)
                for (int column = 0; column < 3; column++)
                {
                    double sx = modelPoints[i][row] - sourceCenter[row];
                    double tx = measuredPoints[i][row] - targetCenter[row];
                    cross[row, column] += sx * (measuredPoints[i][column] - targetCenter[column]) / count;
                    sourceCovariance[row, column] += sx * (modelPoints[i][column] - sourceCenter[column]) / count;
                    targetCovariance[row, column] += tx * (measuredPoints[i][column] - targetCenter[column]) / count;
                }
        if (!HasPlaneSpread(sourceCovariance) || !HasPlaneSpread(targetCovariance))
        {
            error = "Points are coincident or nearly collinear. Use a bent robot pose with separated cover centers.";
            return false;
        }

        // Horn's symmetric quaternion matrix. Its largest algebraic eigenvalue gives
        // the best proper rotation, including planar three-point configurations.
        double xx = cross[0, 0], xy = cross[0, 1], xz = cross[0, 2];
        double yx = cross[1, 0], yy = cross[1, 1], yz = cross[1, 2];
        double zx = cross[2, 0], zy = cross[2, 1], zz = cross[2, 2];
        double[,] horn = {
            { xx + yy + zz, yz - zy, zx - xz, xy - yx },
            { yz - zy, xx - yy - zz, xy + yx, zx + xz },
            { zx - xz, xy + yx, -xx + yy - zz, yz + zy },
            { xy - yx, zx + xz, yz + zy, -xx - yy + zz }
        };
        Diagonalize(horn, out double[,] eigenvectors);
        int best = 0;
        for (int i = 1; i < 4; i++) if (horn[i, i] > horn[best, best]) best = i;
        Quaternion rotation = new Quaternion((float)eigenvectors[1, best], (float)eigenvectors[2, best],
            (float)eigenvectors[3, best], (float)eigenvectors[0, best]).normalized;

        double[,] matrix = RotationMatrix(rotation);
        double[] translation = new double[3];
        for (int row = 0; row < 3; row++)
        {
            translation[row] = targetCenter[row];
            for (int column = 0; column < 3; column++) translation[row] -= matrix[row, column] * sourceCenter[column];
        }
        Pose candidate = new Pose(new Vector3((float)translation[0], (float)translation[1], (float)translation[2]), rotation);
        if (!Finite(candidate.position))
        {
            error = "The fitted translation exceeds the supported coordinate range.";
            return false;
        }

        double squaredSum = 0.0, maximumSquared = 0.0;
        for (int i = 0; i < count; i++)
        {
            double squared = 0.0;
            for (int row = 0; row < 3; row++)
            {
                double difference = candidate.position[row] - (double)measuredPoints[i][row];
                for (int column = 0; column < 3; column++) difference += matrix[row, column] * modelPoints[i][column];
                squared += difference * difference;
            }
            squaredSum += squared / count;
            maximumSquared = Math.Max(maximumSquared, squared);
        }
        float rms = (float)Math.Sqrt(squaredSum), maximum = (float)Math.Sqrt(maximumSquared);
        if (!Finite(rms) || !Finite(maximum))
        {
            error = "The fitted residual exceeds the supported coordinate range.";
            return false;
        }
        modelToMeasured = candidate;
        rmsErrorMeters = rms;
        maxErrorMeters = maximum;
        return true;
    }

    private static bool HasPlaneSpread(double[,] covariance)
    {
        Diagonalize(covariance, out _);
        double[] variance = { covariance[0, 0], covariance[1, 1], covariance[2, 2] };
        Array.Sort(variance);
        return variance[1] >= MinimumSecondVariance && variance[1] >= variance[2] * MinimumVarianceRatio;
    }

    // Jacobi rotations diagonalize a small real symmetric matrix without the
    // identity/180-degree blind spots and eigenvalue-sign issues of power iteration.
    private static void Diagonalize(double[,] matrix, out double[,] vectors)
    {
        int size = matrix.GetLength(0);
        vectors = new double[size, size];
        for (int i = 0; i < size; i++) vectors[i, i] = 1.0;
        for (int iteration = 0; iteration < 100; iteration++)
        {
            int p = 0, q = 1;
            double largest = 0.0, scale = 0.0;
            for (int row = 0; row < size; row++)
            {
                scale = Math.Max(scale, Math.Abs(matrix[row, row]));
                for (int column = row + 1; column < size; column++)
                    if (Math.Abs(matrix[row, column]) > largest)
                    {
                        largest = Math.Abs(matrix[row, column]);
                        p = row;
                        q = column;
                    }
            }
            if (largest <= Math.Max(scale * 1e-14, 1e-30)) return;
            double angle = 0.5 * Math.Atan2(2.0 * matrix[p, q], matrix[q, q] - matrix[p, p]);
            double cosine = Math.Cos(angle), sine = Math.Sin(angle);
            double pp = matrix[p, p], qq = matrix[q, q], pq = matrix[p, q];
            matrix[p, p] = cosine * cosine * pp - 2.0 * cosine * sine * pq + sine * sine * qq;
            matrix[q, q] = sine * sine * pp + 2.0 * cosine * sine * pq + cosine * cosine * qq;
            matrix[p, q] = matrix[q, p] = 0.0;
            for (int row = 0; row < size; row++)
            {
                if (row != p && row != q)
                {
                    double rp = matrix[row, p], rq = matrix[row, q];
                    matrix[row, p] = matrix[p, row] = cosine * rp - sine * rq;
                    matrix[row, q] = matrix[q, row] = sine * rp + cosine * rq;
                }
                double vp = vectors[row, p], vq = vectors[row, q];
                vectors[row, p] = cosine * vp - sine * vq;
                vectors[row, q] = sine * vp + cosine * vq;
            }
        }
    }

    private static double[,] RotationMatrix(Quaternion quaternion)
    {
        double x = quaternion.x, y = quaternion.y, z = quaternion.z, w = quaternion.w;
        // Normalize again in double precision for the residual calculation.
        double factor = 2.0 / (x * x + y * y + z * z + w * w);
        return new double[,] {
            { 1.0 - factor * (y * y + z * z), factor * (x * y - z * w), factor * (x * z + y * w) },
            { factor * (x * y + z * w), 1.0 - factor * (x * x + z * z), factor * (y * z - x * w) },
            { factor * (x * z - y * w), factor * (y * z + x * w), 1.0 - factor * (x * x + y * y) }
        };
    }

    private static bool Finite(Vector3 value) => Finite(value.x) && Finite(value.y) && Finite(value.z);
    private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
}
