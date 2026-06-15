using UnityEngine;
using UnityEngine.Events;

public class URInstantiator : MonoBehaviour
{
    [SerializeField]
    private QRDetector _QRDetector;

    [SerializeField]
    private GameObject _URPrefab;

    /// <summary>
    /// Event triggered when the UR is instantiated. Arguments are the UR instance and the pose of the QR location.
    /// </summary>
    public UnityEvent<GameObject, Pose> OnURInstantiated;

    private Pose _stabilizedPose = Pose.identity;
    private bool _hadURBeenInstantiated = false;

    private void Awake()
    {
        OnURInstantiated = new UnityEvent<GameObject, Pose>();
    }

    void Start()
    {
        _QRDetector.OnQRValidated.AddListener( p => _stabilizedPose = p );
    }

    
    public void InstantiateUR()
    {
        if ((_stabilizedPose != Pose.identity) && !_hadURBeenInstantiated)
        {
            Debug.Log($"Now instantiating UR with pose {_stabilizedPose.position}, {_stabilizedPose.rotation}");
            GameObject UR16eInstance = GameObject.Instantiate(_URPrefab, _stabilizedPose.position, _stabilizedPose.rotation);
            /*
            foreach (ArticulationBody articulationBody in UR16eInstance.GetComponentsInChildren<ArticulationBody>())
            {
                if (articulationBody.gameObject.name == "base_link")
                {
                    articulationBody.TeleportRoot(markerPose.position + markerOffset, finalRotation);
                    break;
                }
            }
            */
            _hadURBeenInstantiated = true;
            OnURInstantiated?.Invoke(UR16eInstance, _stabilizedPose );
            RobotStateService.Instance.InitializeRobot(_stabilizedPose, UR16eInstance);
        }
    }
}
