using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

// Use the CreateAssetMenu attribute to allow creating instances of this ScriptableObject from the Unity Editor.
[CreateAssetMenu(fileName = "PickPlacePose", menuName = "ScriptableObjects/PickAndPlaceScriptableObject", order = 1)]
public class PickAndPlaceScriptableObject : ScriptableObject
{
    private PoseStampedMsg _pickPose;
    private PoseStampedMsg _placePose;
    public PoseStampedMsg PickPose { get => _pickPose; }
    public PoseStampedMsg PlacePose { get => _placePose; }

    /// <summary>
    /// Generate the PoseStampedMsg for the pick point based on a GameObject and the desired normal vector to be used to approach it.
    /// </summary>
    /// <param name="referenceObject"></param>
    /// <param name="objectNormal">The vector along which the pick should take place. If this vector is up for instance, 
    /// the approach will come from the top of the object, up to its center + shift value along this normal vector.</param>
    /// <param name="shift">The value in meters to shift the pick point from the referenceObject's center, along the objectNormal vector. 
    /// This can allow to pick at the surface of the object, not its center.</param>
    /// <param name="frame">The frame (robot link) in which the pose is defined </param>
    /// <param name="pickOrPlace">Specifies wheter the pose specified is a pick or place pose</param>
    public void SetPose(GameObject referenceObject, Vector3 objectNormal, float shift, string frame, PoseType pickOrPlace)
    {
        Vector3 shiftedPosition = objectNormal * shift;
        referenceObject.transform.Translate(shiftedPosition);
        Pose localPose = CoordinatesConverter.ConvertWorldTransformToLocalPose(referenceObject.transform, frame);

        Vector3 pos = localPose.position;
        Quaternion rot = localPose.rotation;
        Vector3 scale = referenceObject.transform.localScale;

        PointMsg rosPos = pos.To<FLU>();
        QuaternionMsg rosRot = rot.To<FLU>();

        HeaderMsg header = new HeaderMsg();
        header.frame_id = "ur_" + frame;
        PoseMsg pose = new PoseMsg(rosPos, rosRot);
        PoseStampedMsg poseStamped = new PoseStampedMsg(header, pose);
        if (pickOrPlace == PoseType.PickPose)
        {
            _pickPose = poseStamped;
        }
        else if (pickOrPlace == PoseType.PlacePose)
        {
            _placePose = poseStamped;
        }
    }
}
