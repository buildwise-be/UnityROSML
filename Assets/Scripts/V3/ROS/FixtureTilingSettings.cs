using System;
using RosMessageTypes.Geometry;

/// <summary>Measured job inputs. Poses are static ROS ur_base_link object frames, not TCP frames.</summary>
[Serializable]
public sealed class FixtureTilingSettings
{
    public float ItemSize = 0.2f, ItemThickness = 0.01f, StackPitch;
    public float Gap = 0.005f, RowOffsetFraction = 0.5f;
    public float ReleaseCenterX = 0.25f, ReleaseCenterY = 0.25f;
    public float ReleaseWidth = 0.04f, ReleaseDepth = 0.04f, ReleaseClearance = 0.01f;
    public float RetreatDistance = 0.15f, SettlingDelay = 3f;
    public int SourceCount = 4, ItemCount = 4, Columns = 2, ReleaseYawSamples = 4;
    public bool ReturnHomeEachTile = true;
    public PoseStampedMsg SourceTopPose;
    public PoseStampedMsg PlacementCorner;
}

/// <summary>Board-top measurements and corner frame saved by ROS for its next startup.</summary>
[Serializable]
public sealed class FixtureCalibrationData
{
    public PointMsg[] BoardTopCorners = new PointMsg[4];
    public float BoardThickness = 0.02f, Margin = 0.005f;
    public PoseStampedMsg SupportCorner;
}
