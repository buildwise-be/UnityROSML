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

    public static bool TryBuild(string requestId, int timeoutSeconds, PickSurfaceCapture capture,
        Pose floorWorld, FloorTilePattern pattern, out TileFloorTaskMsg request)
    {
        request = null;
        if (!FixtureTilingGeometry.ValidToken(requestId) ||
            timeoutSeconds < MtcPlanningSettings.MinTimeoutSeconds || timeoutSeconds > MtcPlanningSettings.MaxTimeoutSeconds ||
            pattern.TileCount < FloorTilePattern.MinimumTileCount ||
            capture == null || capture.IsCapturing || capture.SurfaceItem != PickItem.Tile ||
            !capture.TryGetWorldCorners(out Vector3[] corners) ||
            !capture.TryGetWorldTopSurfacePose(out Pose sourceTop))
            return false;

        PickItemPreset tile = PickSurfaceGeometry.GetPreset(PickItem.Tile);
        if (pattern.TileSide != (float)tile.Width || pattern.TileThickness != (float)tile.Thickness)
            return false;

        RobotCoordinateService coordinates = RobotCoordinateService.Instance;
        if (coordinates.RosBaseFrameId != FixtureTilingGeometry.FrameId ||
            !coordinates.TryGetRobotBaseWorldPose(out Pose baseWorld) ||
            !IsLevel(sourceTop.rotation * Vector3.up, baseWorld.rotation * Vector3.up) ||
            !coordinates.TryConvertWorldPoseToLocalPose(floorWorld, out Pose floorLocal))
            return false;
        if (!IsLevel(floorLocal.rotation * Vector3.up, Vector3.up)) return false;

        var source = new PointMsg[3];
        for (int i = 0; i < source.Length; i++)
        {
            if (!coordinates.TryConvertWorldPoseToLocalPose(new Pose(corners[i], Quaternion.identity), out Pose corner))
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
            source_top_corners = source,
            floor_corner = new PoseStampedMsg(
                new HeaderMsg { frame_id = FixtureTilingGeometry.FrameId, stamp = new TimeMsg(0, 0) },
                new PoseMsg(floorLocal.position.To<FLU>(), floorRotation.To<FLU>())),
            tile_count = (uint)pattern.TileCount,
            row_offset = (byte)pattern.OffsetIndex,
            mortar_mm = (byte)pattern.MortarMillimeters
        };
        return true;
    }
}
