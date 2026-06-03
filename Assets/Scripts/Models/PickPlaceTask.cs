using UnityEngine;

/// <summary>
/// ScriptableObject for storing pick and place task data.
/// Can be serialized and saved for persistence.
/// </summary>
[CreateAssetMenu(fileName = "PickPlaceTask", menuName = "ScriptableObjects/PickPlaceTask", order = 1)]
public class PickPlaceTask : ScriptableObject
{
    [Header("Pick Pose")]
    [SerializeField]
    private Vector3 _pickPosition = Vector3.zero;
    [SerializeField]
    private Quaternion _pickRotation = Quaternion.identity;

    [Header("Place Pose")]
    [SerializeField]
    private Vector3 _placePosition = Vector3.zero;
    [SerializeField]
    private Quaternion _placeRotation = Quaternion.identity;

    [Header("Task State")]
    [SerializeField]
    private bool _isPlanningRequestSent = false;
    [SerializeField]
    private bool _isTaskExecuted = false;

    public Vector3 PickPosition
    {
        get => _pickPosition;
        set => _pickPosition = value;
    }

    public Quaternion PickRotation
    {
        get => _pickRotation;
        set => _pickRotation = value;
    }

    public Vector3 PlacePosition
    {
        get => _placePosition;
        set => _placePosition = value;
    }

    public Quaternion PlaceRotation
    {
        get => _placeRotation;
        set => _placeRotation = value;
    }

    public bool IsPlanningRequestSent
    {
        get => _isPlanningRequestSent;
        set => _isPlanningRequestSent = value;
    }

    public bool IsTaskExecuted
    {
        get => _isTaskExecuted;
        set => _isTaskExecuted = value;
    }

    public Pose GetPickPose()
    {
        return new Pose(_pickPosition, _pickRotation);
    }

    public Pose GetPlacePose()
    {
        return new Pose(_placePosition, _placeRotation);
    }

    public void SetPickPose(Pose pose)
    {
        _pickPosition = pose.position;
        _pickRotation = pose.rotation;
    }

    public void SetPlacePose(Pose pose)
    {
        _placePosition = pose.position;
        _placeRotation = pose.rotation;
    }

    public void Reset()
    {
        _pickPosition = Vector3.zero;
        _pickRotation = Quaternion.identity;
        _placePosition = Vector3.zero;
        _placeRotation = Quaternion.identity;
        _isPlanningRequestSent = false;
        _isTaskExecuted = false;
    }
}

