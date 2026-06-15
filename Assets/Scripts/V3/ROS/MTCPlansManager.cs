using RosMessageTypes.Moveit;
using RosMessageTypes.Std;
using UnityEngine;
using UnityEngine.Events;

public class MTCPlansManager : MonoBehaviour
{
    [SerializeField]
    private UR16eTrajectoryVisualizer _UR16eTrajectoryVisualizer;
    private bool _isLastPlanSuccessful = false;
    public UnityEvent<bool> OnSuccessfulPlan;

    public bool IsLastPlanSuccessful 
    { 
        get { return _isLastPlanSuccessful; } 
    }

    private void Awake()
    {
        OnSuccessfulPlan = new UnityEvent<bool>();
    }

    void Start()
    {
        ROSService.Instance.Subscribe<StringMsg>("/mtc/planning_status", msg =>
        {
            var status = JsonUtility.FromJson<PlanningStatus>(msg.data); // [Serializable] DTO matching the schema
                                                                         // render status.stages (indent by depth, color by solved/failed); gate the Execute button on status.overall_success
            if (status != null)
            {
                _isLastPlanSuccessful = status.overall_success ? true : false;
                OnSuccessfulPlan?.Invoke(_isLastPlanSuccessful);
            }
        });
        ROSService.Instance.Subscribe<RobotTrajectoryMsg>("/mtc/planned_trajectory", traj =>
        {
            Debug.Log("planned_trajectory received");
            _UR16eTrajectoryVisualizer.SetTrajectory(traj);   // existing ghost clones + spline
                                                        // and/or feed your new looping TrajectoryPlaybackAnimator
        });
    }
}

[System.Serializable]
public class StageStatus
{
    public string name;
    public int depth;
    public int solved;
    public int failed;
}

[System.Serializable]
public class PlanningStatus
{
    public bool overall_success;
    public StageStatus[] stages;
}