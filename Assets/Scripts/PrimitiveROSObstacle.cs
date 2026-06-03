using TMPro;
using UnityEngine;

public class PrimitiveROSObstacle : MonoBehaviour
{
    public PrimitiveObstacleTypes ObstacleType;
    [SerializeField]
    private bool _enableObstacleOnRobotPlacement = true;
    private PlanningSceneUpdater _planningSceneUpdater;

    private RobotCoordinatesConverter _robotCoordinatesConverter;
    [SerializeField]
    private TextMeshProUGUI _debugText;

    private void OnEnable()
    {
        _planningSceneUpdater = FindAnyObjectByType<PlanningSceneUpdater>();
        _robotCoordinatesConverter = FindAnyObjectByType<RobotCoordinatesConverter>();
        if (_enableObstacleOnRobotPlacement)
        {
            _robotCoordinatesConverter.OnRobotPositionRecorded.AddListener(AddObstacleToMoveitScene);
        }

        if (_debugText != null)
            _debugText.text = $"Found _planningSceneUpdater {_planningSceneUpdater} and _robotCoordinatesConverter {_robotCoordinatesConverter}";
    }

    public void AddObstacleToMoveitScene()
    {
        if (_debugText != null)
            _debugText.text += "\nAdding PrimitiveROSObstacle";
        if (_robotCoordinatesConverter == null || _planningSceneUpdater == null)
        {
            if (_debugText != null)
                _debugText.text += "\nCouldn't add PrimitiveROSObstacle!";
            return;
        }
        Pose localPose = _robotCoordinatesConverter.ConvertWorldTransformToLocalPose(transform);
        //Vector3 dimensions = new Vector3(transform.localScale.x, transform.localScale.y, transform.localScale.z);
        Vector3 worldDimensions = _robotCoordinatesConverter.GetWorldAlignedDimensions(transform);
        //Vector3 dimensions = transform.InverseTransformDirection(worldDimensions);
        Vector3 dimensions = worldDimensions;
        if (ObstacleType == PrimitiveObstacleTypes.Box)
        {
            if (_debugText != null)
                _debugText.text += $"\nCalling UpdateBox with dimensions {dimensions}";
            _planningSceneUpdater.UpdateBox(localPose, dimensions, gameObject.GetInstanceID());
        }
        else if (ObstacleType == PrimitiveObstacleTypes.Plane)
        {
            //TODO: improve, this is wrong. Not always z axis is the thin one.
            //dimensions = GetComponent<MeshFilter>().mesh.bounds.max - GetComponent<MeshFilter>().mesh.bounds.min;
            //Vector3 worldDimensions = transform.TransformVector(dimensions); // Now includes scale


            /*
            float[] dimz = { dimensions.x, dimensions.y, dimensions.z };
            Vector2 planeDims = new Vector2();
            for (int i=0; i<3; i++ )
            {
                if (dimz[i] < 0.01f)
                {
                    dimz[i] = 0.01f;
                }
                else
                {

                }
            }
            */
            int thinAxis = 0;
            float minSize = dimensions[0];
            for (int i = 1; i < 3; i++)
            {
                if (dimensions[i] < minSize)
                {
                    minSize = dimensions[i];
                    thinAxis = i;
                }
            }
            Vector3 planeDimensions = new Vector3(dimensions.x, dimensions.y, dimensions.z);
            planeDimensions[thinAxis] = 0.01f;  // Impose thinness on smallest axis

            if (_debugText != null)
                _debugText.text += $"\nCalling UpdatePlane with dimensions {planeDimensions}";
            _planningSceneUpdater.UpdateBox(localPose, planeDimensions, gameObject.GetInstanceID());
        }
    }
}