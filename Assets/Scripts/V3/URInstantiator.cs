using UnityEngine;
using UnityEngine.Events;

public class URInstantiator : MonoBehaviour
{
    [SerializeField]
    private QRDetector _QRDetector;

    [SerializeField]
    private GameObject _URPrefab;

    public UnityEvent<Pose> OnURInstantiated;

    private Pose _stabilizedPose = Pose.identity;

    void Start()
    {
        _QRDetector.OnQRValidated.AddListener( p => _stabilizedPose = p );
    }

    
    public void InstantiateUR()
    {
        if (_stabilizedPose != Pose.identity)
        {
            Debug.Log($"Now instantiating UR with pose {_stabilizedPose.position}, {_stabilizedPose.rotation}");
            GameObject.Instantiate(_URPrefab, _stabilizedPose.position, _stabilizedPose.rotation);
            OnURInstantiated?.Invoke( _stabilizedPose );
        }
    }
}
