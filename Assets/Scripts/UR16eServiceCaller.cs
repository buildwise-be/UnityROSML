using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using RosMessageTypes.Geometry;
using RosMessageTypes.Ur16eUnityInterfaces;
using UnityEngine.Events;
using TMPro;
using RosMessageTypes.Std;
using UnityEngine.Serialization;

[Serializable]
public class Ur16EServiceCaller : MonoBehaviour
{
    [Header("ROS Service Configuration")]
    [Tooltip("Must exactly match the ROS2-side service name.")]
    public string planSingleTargetServiceName = "ur16e_mover_service";
    [Tooltip("Must exactly match the ROS2-side service name.")]
    public string executeSingleTargetServiceName = "ur16e_execute_trajectory";
    [Tooltip("The name of the service to plan a multiple points path.")]
    public string planPathServiceName = "ur16e_path_plan";
    [Tooltip("The name of the service to plan a multiple points path.")]
    public string executePathServiceName = "ur16e_path_execute";

    [Header("Trajectory Visualization")]
    [Tooltip("The GameObject with the UR16eEndPoseVisualizer script.")]
    public UR16eTrajectoryVisualizer trajectoryVisualizer;
    public UR16eEndPoseVisualizer endPoseVisualizer;

    private Vector3 _positionCorrection;
    private Quaternion _rotationCorrection;

    public VisualizationStyle visualizationStyle = VisualizationStyle.FullTrajectory;

    [HideInInspector]
    public UnityEvent onSingleTargetPlanServiceCalled;
    [HideInInspector]
    public UnityEvent<UR16eMoverServiceResponse> onSingleTargetPlanResponseReceived;
    [HideInInspector]
    public UnityEvent<UR16ePathPlanRequest> onPathPlanServiceCalled;
    [HideInInspector]
    public UnityEvent<UR16ePathPlanResponse> onPathPlanReceived;

    [FormerlySerializedAs("_markerUnderstander")] [SerializeField]
    private MarkerUnderstander markerUnderstander;

    [FormerlySerializedAs("_debugText")]
    [Header("Debug")]
    [Tooltip("The textbox used for debugging purpose.")]
    [SerializeField]
    private TextMeshProUGUI debugText;
    [FormerlySerializedAs("_debugEnabled")] [SerializeField]
    private bool debugEnabled = false;
    [SerializeField]
    private GameObject debugTargetPrefab;
    private GameObject debugTarget;


    private GameObject _ur16EInstance;

    [FormerlySerializedAs("_confirmationCanvas")] [SerializeField]
    private GameObject confirmationCanvas;

    // Cached ROS connection
    private ROSConnection _ros;
    private bool _useCartesian = false;
    public bool UseCartesian { get => _useCartesian; set => _useCartesian = value; }
    private bool _closedPath = true;
    public bool ClosedPath { get => _closedPath; set => _closedPath = value; }
    
    private TargetsManager _targetsManager;

    private void Awake()
    {
        onSingleTargetPlanServiceCalled = new UnityEvent();
        onSingleTargetPlanResponseReceived = new UnityEvent<UR16eMoverServiceResponse>();
        onPathPlanServiceCalled  = new UnityEvent<UR16ePathPlanRequest>();
        onPathPlanReceived = new UnityEvent<UR16ePathPlanResponse>();
        _targetsManager = FindAnyObjectByType<TargetsManager>();
    }

    void Start()
    {
        // Get (or create) the ROSConnection singleton
        _ros = ROSConnection.GetOrCreateInstance();

        // Register the custom service type so Unity can serialize messages automatically
        _ros.RegisterRosService<UR16eMoverServiceRequest, UR16eMoverServiceResponse>(planSingleTargetServiceName);
        _ros.RegisterRosService<SetBoolRequest, SetBoolResponse>(executeSingleTargetServiceName);
        _ros.RegisterRosService<UR16ePathPlanRequest, UR16ePathPlanResponse>(planPathServiceName);
        _ros.RegisterRosService<SetBoolRequest, SetBoolResponse>(executePathServiceName);

        markerUnderstander.OnUR16eInstantiatedAndPlaced.AddListener(RetrieveUrPosition);
    }

    private void RetrieveUrPosition(Pose arg0, GameObject arg1)
    {
        _positionCorrection = arg0.position;
        _rotationCorrection = arg0.rotation;
        _ur16EInstance = arg1;
    }
    
    #region path_planning
    /// <summary>
    /// Call this method (e.g. via a UI button or any trigger) to send the target pose
    /// to the ROS 2 service and asynchronously handle the response.
    /// </summary>
    public void SendPlanPathServiceRequest()
    {
        if (_targetsManager == null)
            return; // TODO: log that in some in-game panel

        // 1) Build the service request
        UR16ePathPlanRequest req = new UR16ePathPlanRequest
        {
            use_cartesian = true,
            closed_path = _closedPath
        };

        List<PoseMsg> targetPoses = new List<PoseMsg>();
        bool firstTarget = true;
        if (debugTarget != null)
            Destroy(debugTarget);
        foreach (var target in _targetsManager.targets)
        {
            Vector3 rootRelPos = Vector3.zero;
            Quaternion rootRelRot = Quaternion.identity;

            foreach (ArticulationBody articulationBody in _ur16EInstance.GetComponentsInChildren<ArticulationBody>())
            {
                if (articulationBody.gameObject.name == "base_link")
                {
                    rootRelPos = articulationBody.transform.InverseTransformPoint(target.transform.position);
                    rootRelRot = Quaternion.Inverse(articulationBody.transform.rotation) * (target.transform.rotation);
                    if (firstTarget)
                    {
                        if (debugTarget == null)
                        {
                            debugTarget = Instantiate(debugTargetPrefab);
                            debugTarget.transform.SetParent(articulationBody.gameObject.transform);
                            debugTarget.transform.localPosition = rootRelPos;
                            firstTarget = false;
                        }
                    }
                    break;
                }
            }
            //Quaternion rot = Quaternion.Inverse(target.transform.rotation) * target.transform.rotation;
            PointMsg rosPos = rootRelPos.To<FLU>();
            QuaternionMsg rosRot = rootRelRot.To<FLU>();
            PoseMsg pose = new PoseMsg(rosPos, rosRot);
            targetPoses.Add(pose);
        }
        
        req.target_poses =  targetPoses.ToArray();

        // 2) Send the service request and specify callback
        _ros.SendServiceMessage<UR16ePathPlanResponse>(
            planSingleTargetServiceName, req, PlanPathResponseHandler);
        onPathPlanServiceCalled?.Invoke(req);
    }

    private void PlanPathResponseHandler(UR16ePathPlanResponse resp)
    {
        onPathPlanReceived?.Invoke(resp);

        // Planning succeeded: pass the trajectory to the visualizer
        if (visualizationStyle == VisualizationStyle.FullTrajectory)
        {
            if (trajectoryVisualizer != null)
            {
                trajectoryVisualizer.SetTrajectory(resp.trajectory);
            }
        }
        else if (visualizationStyle == VisualizationStyle.EndPosition)
        {
            if (endPoseVisualizer != null)
            {
                endPoseVisualizer.VisualizeFinalPose(resp.trajectory);
            }
        }
        else
        {
            Debug.LogError("[UR16eServiceCaller] No Visualizer. Assign in Inspector.");
            return;
        }
        confirmationCanvas.SetActive(true);
    }
    
    public void ExecutePath()
    {
        SetBoolRequest request = new SetBoolRequest();
        request.data = true;
        _ros.SendServiceMessage<SetBoolResponse>(executePathServiceName, request, CheckPathExecutionResponse);
    }

    private void CheckPathExecutionResponse(SetBoolResponse response)
    {
        string errormsg = response.message;
        bool success = response.success;
        if (!success)
        {
            Debug.LogWarning(errormsg); //TODO: to log menu
        }    
    }

    #endregion
    

    #region single_target
    /// <summary>
    /// Call this method (e.g. via a UI button or any trigger) to send the target pose
    /// to the ROS 2 service and asynchronously handle the response.
    /// </summary>
    public void SendPlanSingleTargetServiceRequest()
    {
        // 1) Build the service request
        UR16eMoverServiceRequest req = new UR16eMoverServiceRequest();

        Vector3 rootRelativePos = Vector3.zero;
        Quaternion rootRelativeRot = Quaternion.identity;

        Target target = _targetsManager.targets[0];
        
        foreach (ArticulationBody articulationBody in _ur16EInstance.GetComponentsInChildren<ArticulationBody>())
        {
            if (articulationBody.gameObject.name == "base_link")
            {
                rootRelativePos = articulationBody.transform.InverseTransformPoint(target.transform.position);
                rootRelativeRot = Quaternion.Inverse(articulationBody.transform.rotation) * (target.transform.rotation);
                break;
            }
        }

        // Unity's world coordinates -> ROS coordinates:
        // Here we assume Unity world == MoveIt base_link. If you have a TF offset, apply it.
        Quaternion rot = Quaternion.Inverse(target.transform.rotation) * target.transform.rotation; //TODO: remove?
        PointMsg rosPos = rootRelativePos.To<FLU>();
        QuaternionMsg rosRot = rootRelativeRot.To<FLU>();

        req.target_pose = new PoseMsg(
            rosPos,
            rosRot
        );

        req.use_cartesian = _useCartesian;

        // 2) Send the service request and specify callback
        _ros.SendServiceMessage<UR16eMoverServiceResponse>(
            planSingleTargetServiceName, req, SingleTargetResponseHandler);
        onSingleTargetPlanServiceCalled?.Invoke();
    }
    /// <summary>
    /// Called when the service response arrives.
    /// </summary>
    private void SingleTargetResponseHandler(UR16eMoverServiceResponse resp)
    {
        onSingleTargetPlanResponseReceived?.Invoke(resp);

        // Planning succeeded: pass the trajectory to the visualizer
        if (visualizationStyle == VisualizationStyle.FullTrajectory)
        {
            if (trajectoryVisualizer != null)
            {
                trajectoryVisualizer.SetTrajectory(resp.trajectory);
            }
        }
        else if (visualizationStyle == VisualizationStyle.EndPosition)
        {
            if (endPoseVisualizer != null)
            {
                endPoseVisualizer.VisualizeFinalPose(resp.trajectory);
            }
        }
        else
        {
            Debug.LogError("[UR16eServiceCaller] No Visualizer. Assign in Inspector.");
            return;
        }
        confirmationCanvas.SetActive(true);
    }
    public void ExecuteSingleTarget()
    {
        SetBoolRequest request = new SetBoolRequest();
        request.data = true;
        _ros.SendServiceMessage<SetBoolResponse>(executeSingleTargetServiceName, request, CheckSingleTargetExecutionResponse);
    }
    private void CheckSingleTargetExecutionResponse(SetBoolResponse response)
    {
        string errormsg = response.message;
        bool success = response.success;
        if (!success)
        {
            Debug.LogWarning(errormsg);
        }
    }
    #endregion
}

public enum VisualizationStyle
{
    FullTrajectory,
    EndPosition
}
