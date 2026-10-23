using Unity.XR.CoreUtils;
using UnityEngine;

/// <summary>OpenXR poses are local to the XR Origin's camera-floor offset, not its camera.</summary>
public static class XRTrackingSpaceResolver
{
    public static bool TryResolve(ref Transform trackingSpace)
    {
        if (trackingSpace == null)
        {
            XROrigin[] origins = Object.FindObjectsByType<XROrigin>(FindObjectsSortMode.None);
            if (origins.Length != 1)
                return false;

            XROrigin origin = origins[0];
            trackingSpace = origin.CameraFloorOffsetObject != null
                ? origin.CameraFloorOffsetObject.transform
                : origin.Origin != null ? origin.Origin.transform : origin.transform;
        }

        return RobotFrameMath.IsRigidFrame(trackingSpace);
    }
}
