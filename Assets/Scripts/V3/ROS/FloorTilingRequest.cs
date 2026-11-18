using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Ur16eInterfaces;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

/// <summary>Builds an immutable snapshot of the measured stack top and floor pattern in robot coordinates.</summary>
public static class FloorTilingRequest
{
    private static readonly float MinimumLevelDot = Mathf.Cos(0.05f);

    public static bool IsLevel(Vector3 surfaceNormal, Vector3 baseUp) =>
        Vector3.Dot(surfaceNormal, baseUp) >= MinimumLevelDot;

    public static bool TryBuild(string requestId, int timeoutSeconds, Pose sourceWorld, Vector3[] measuredCorners,
        Pose floorWorld, FloorTilePattern pattern, out TileFloorTaskMsg request)
    {
        request = null;
        bool fromCorners = measuredCorners != null && measuredCorners.Length != 0;
        if (!FixtureTilingGeometry.ValidToken(requestId) ||
            timeoutSeconds < MtcPlanningSettings.MinTimeoutSeconds || timeoutSeconds > MtcPlanningSettings.MaxTimeoutSeconds ||
            pattern.TileCount < FloorTilePattern.MinimumTileCount ||
            (fromCorners && measuredCorners.Length != 3))
            return false;

        RobotCoordinateService coordinates = RobotCoordinateService.Instance;
        if (coordinates.RosBaseFrameId != FixtureTilingGeometry.FrameId ||
            !coordinates.TryConvertWorldPoseToLocalPose(sourceWorld, out Pose sourceLocal) ||
            !IsLevel(sourceLocal.rotation * Vector3.up, Vector3.up) ||
            !coordinates.TryConvertWorldPoseToLocalPose(floorWorld, out Pose floorLocal))
            return false;
        if (!IsLevel(floorLocal.rotation * Vector3.up, Vector3.up)) return false;

        var source = new PointMsg[3];
        for (int i = 0; i < source.Length; i++)
        {
            if (!fromCorners) { source[i] = new PointMsg(); continue; }
            if (!coordinates.TryConvertWorldPoseToLocalPose(new Pose(measuredCorners[i], Quaternion.identity), out Pose corner))
                return false;
            source[i] = corner.position.To<FLU>();
        }

        // FLU converts local Unity +Z, -X, +Y to ROS +X, +Y, +Z.
        // Rotate once so ROS +X follows the visible columns and ROS +Y follows the rows.
        Quaternion floorRotation = floorLocal.rotation * Quaternion.AngleAxis(90f, Vector3.up);
        request = new TileFloorTaskMsg
        {
            request_id = requestId,
            timeout_seconds = (uint)timeoutSeconds,
            tile_side = pattern.TileSide,
            tile_thickness = pattern.TileThickness,
            source_from_surface = fromCorners,
            source_top = StaticPose(sourceLocal.position, sourceLocal.rotation),
            source_top_corners = source,
            floor_corner = StaticPose(floorLocal.position, floorRotation),
            tile_count = (uint)pattern.TileCount,
            row_offset = (byte)pattern.OffsetIndex,
            mortar_mm = (byte)pattern.MortarMillimeters
        };
        return true;
    }

    private static PoseStampedMsg StaticPose(Vector3 position, Quaternion rotation) => new PoseStampedMsg(
        new HeaderMsg { frame_id = FixtureTilingGeometry.FrameId, stamp = new TimeMsg(0, 0) },
        new PoseMsg(position.To<FLU>(), rotation.To<FLU>()));
}
