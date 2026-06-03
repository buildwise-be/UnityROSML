using System;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Moveit;       // For RobotTrajectoryMsg
using RosMessageTypes.Trajectory;   // For JointTrajectoryPointMsg
using System.Linq;
using TMPro;
using UnityEngine.EventSystems;

[Serializable]
public class UR16eTrajectoryVisualizer : MonoBehaviour
{
    [Header("Prefab & Scaling")]
    [Tooltip("A UR16e prefab imported via URDF Importer, with ArticulationBodies at each joint.")]
    public GameObject virtualUr16ePrefab;
    [SerializeField]
    private MarkerUnderstander _markerUnderstander;
    private Pose _ur16ePose;
    private bool _hasUR16eBeenInstantiated = false;

    [Header("Visualization Settings")]
    [Range(0f, 100f)]
    [Tooltip("Percentage (0–100) of intermediate trajectory waypoints to display. " +
             "Final waypoint is always displayed even if this is 0.")]
    public float percentageToShow = 0f;

    [Header("Trajectory Line")]
    public LineRenderer trajectoryLineRenderer;
    public int resolution = 10;
    private List<Vector3> trajectoryControlPoints;

    [Header("Debug")]
    [Tooltip("The textbox used for debugging purpose.")]
    [SerializeField]
    private TextMeshProUGUI _debugText;
    [SerializeField]
    private bool _debugEnabled = false;

    // A copy of the last trajectory we visualized
    private RobotTrajectoryMsg currentTrajectory;

    // Keep track of all spawned clones so we can clear them later
    private readonly List<GameObject> spawnedClones = new List<GameObject>();

    // Remember the previous percentage so we can detect changes at runtime
    private float prevPercentage = -1f;

    // Map ROS joint names → Unity hierarchy path (relative to prefab root)
    private static readonly Dictionary<string, string> RosNameToUnityName = new Dictionary<string, string>()
    {
        { "shoulder_pan_joint",   "world/base_link/base_link_inertia/shoulder_link" },
        { "shoulder_lift_joint",  "world/base_link/base_link_inertia/shoulder_link/upper_arm_link" },
        { "elbow_joint",          "world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link" },
        { "wrist_1_joint",        "world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link" },
        { "wrist_2_joint",        "world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link" },
        { "wrist_3_joint",        "world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link" }
    };

    private GameObject _ur16eInstance;

    void Start()
    {
        trajectoryControlPoints = new List<Vector3>();
        // Initialize prevPercentage so that the first SetTrajectory call always spawns clones
        prevPercentage = -1f;
        _markerUnderstander.OnUR16eInstantiatedAndPlaced.AddListener(RecordUR16ePose);
    }

    void Update()
    {
        // If the user changed percentageToShow in the Inspector (or via a UI slider),
        // and we have an existing trajectory, re-spread clones.
        if (currentTrajectory != null && !Mathf.Approximately(prevPercentage, percentageToShow))
        {
            prevPercentage = percentageToShow;
            ClearAllClones();
            RedistributeClones();
        }
    }

    private void RecordUR16ePose(Pose pose, GameObject go)
    {
        _ur16ePose = pose;
        _hasUR16eBeenInstantiated = true;
        _ur16eInstance = go;
    }

    /// <summary>
    /// Call this when a new trajectory arrives. It will clear any existing clones
    /// and spawn according to the current percentageToShow.
    /// </summary>
    public void SetTrajectory(RobotTrajectoryMsg trajectoryMsg)
    {
        if (_debugEnabled && _debugText != null)
        {
            _debugText.text += "\nSetting trajectory";
        }
        // Replace stored trajectory
        currentTrajectory = trajectoryMsg;
        // Reset prevPercentage to force a full re‐draw
        prevPercentage = -1f;
        // Clear out old clones
        ClearAllClones();
        // Spawn new clones
        RedistributeClones();
    }

    /// <summary>
    /// Clears all UR16e clones from the scene.
    /// </summary>
    public void ClearAllClones()
    {
        foreach (var go in spawnedClones)
        {
            if (go) Destroy(go);
        }
        spawnedClones.Clear();
        trajectoryControlPoints.Clear();
    }

    /// <summary>
    /// Recomputes which trajectory indices should be shown (based on percentageToShow),
    /// and instantiates clones at those joint configurations.
    /// </summary>
    private void RedistributeClones()
    {
        if (currentTrajectory == null)
            return;

        var jointTrajectory = currentTrajectory.joint_trajectory;
        string[] jointNames = jointTrajectory.joint_names;
        JointTrajectoryPointMsg[] points = jointTrajectory.points;

        if (jointNames == null || jointNames.Length == 0)
        {
            Debug.LogError("[UR16eTrajectoryVisualizer] Received empty joint_names.");
            return;
        }
        if (points == null || points.Length == 0)
        {
            Debug.LogWarning("[UR16eTrajectoryVisualizer] Received zero waypoints.");
            return;
        }

        int N = points.Length; // total waypoints
        if (N == 0) return;

        // 1) Compute how many intermediate points (0..N-2) to show,
        //    rounding up: if percentageToShow=10% of 178, (10/100)*(177)=17.7 → Ceil → 18
        int intermediateCount = Mathf.CeilToInt((percentageToShow / 100f) * (N - 1));

        // 2) Build a list of indices to show
        List<int> indicesToShow = new List<int>();

        if (intermediateCount > 0)
        {
            // Evenly space 'intermediateCount' indices in [0 .. N-2]
            float step = (N - 1f) / (intermediateCount + 1f);
            for (int i = 1; i <= intermediateCount; ++i)
            {
                int idx = Mathf.RoundToInt(i * step);
                // Clamp just in case rounding pushes outside [0..N-2]
                idx = Mathf.Clamp(idx, 0, N - 2);
                indicesToShow.Add(idx);
            }
        }

        // 3) Always add the last index (N-1)
        indicesToShow.Add(N - 1);

        // Sort them so clones appear in increasing‐time order
        indicesToShow.Sort();

        // 4) Instantiate clones for each selected index
        foreach (int idx in indicesToShow)
        {
            InstantiateCloneAtPoint(jointNames, points[idx].positions);
        }
        trajectoryLineRenderer.positionCount = 0;
        System.Collections.IEnumerator routine = ComputeTrajectorySplineCoroutine();
        StartCoroutine(routine);
    }

    /// <summary>
    /// Instantiates a UR16e prefab, sets all 6 joint angles according to 'positionsRad'.
    /// 'jointNames' must align with the order of 'positionsRad'.
    /// </summary>
    private void InstantiateCloneAtPoint(string[] jointNames, double[] positionsRad)
    {
        if (virtualUr16ePrefab == null || !_hasUR16eBeenInstantiated)
        {
            Debug.LogError("[UR16eTrajectoryVisualizer] ur16ePrefab is null.");
            return;
        }

        // 1) Instantiate
        GameObject instance = Instantiate(
            virtualUr16ePrefab,
            _ur16ePose.position,
            _ur16ePose.rotation
        );

        // 2) For each joint, find the Unity Transform via RosNameToUnityName, set articulation target
        for (int j = 0; j < jointNames.Length; ++j)
        {
            string rosJointName = jointNames[j];
            if (!RosNameToUnityName.TryGetValue(rosJointName, out string unityPath))
            {
                Debug.LogWarning($"[UR16eTrajectoryVisualizer] No Unity path for ROS joint '{rosJointName}'.");
                continue;
            }

            // Find child Transform by path (e.g. "world/base_link/.../wrist_3_link")
            Transform jointTf = instance.transform.Find(unityPath);
            if (jointTf == null)
            {
                Debug.LogWarning($"[UR16eTrajectoryVisualizer] Could not find '{unityPath}' in prefab.");
                continue;
            }

            var ab = jointTf.GetComponent<ArticulationBody>();
            if (ab == null)
            {
                Debug.LogWarning($"[UR16eTrajectoryVisualizer] No ArticulationBody on '{unityPath}'.");
                continue;
            }

            // Convert from radians to degrees
            float angleDeg = (float)(positionsRad[j] * Mathf.Rad2Deg);

            // Set the xDrive target
            var drive = ab.xDrive;
            drive.target = angleDeg;
            ab.xDrive = drive;
        }

        // 3) Keep track of the spawned clone so we can destroy it later
        spawnedClones.Add(instance);
    }
    
    #region trajectory_spline
    private System.Collections.IEnumerator ComputeTrajectorySplineCoroutine()
    {
        yield return new WaitForSeconds( 5.0f );
        ComputeTrajectorySpline();
        yield return null;
    }
    public void ComputeTrajectorySpline()
    {
        trajectoryControlPoints.Clear();

        Vector3 currentFlangePosition = _ur16eInstance.transform.Find("world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link/flange").transform.position;
        trajectoryControlPoints.Add(currentFlangePosition);

        //Physics.SyncTransforms();
        foreach (GameObject instance in spawnedClones)
        {
            Vector3 flangePos = instance.transform.Find("world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link/flange").transform.position;
            if (_debugEnabled && _debugText != null)
                _debugText.text += flangePos.ToString();
            trajectoryControlPoints.Add(flangePos);
        }
        var splinePoints = GetSplinePoints(trajectoryControlPoints, resolution);
        trajectoryLineRenderer.positionCount = splinePoints.Count;
        trajectoryLineRenderer.SetPositions(splinePoints.ToArray());
    }
    private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        // Standard Catmull-Rom spline formula
        return 0.5f * (
            2f * p1 +
            (-p0 + p2) * t +
            (2f * p0 - 5f * p1 + 4f * p2 - p3) * t * t +
            (-p0 + 3f * p1 - 3f * p2 + p3) * t * t * t
        );
    }
    List<Vector3> GetSplinePoints(List<Vector3> controlPoints, int resolution)
    {
        List<Vector3> splinePoints = new List<Vector3>();

        int count = controlPoints.Count;

        if (count < 2)
            return splinePoints;

        for (int i = 0; i < count - 1; i++)
        {
            // Get surrounding points
            Vector3 p0 = (i == 0)
                ? controlPoints[i] + (controlPoints[i] - controlPoints[i + 1]) // Extrapolate before first
                : controlPoints[i - 1];

            Vector3 p1 = controlPoints[i];
            Vector3 p2 = controlPoints[i + 1];

            Vector3 p3 = (i + 2 >= count)
                ? controlPoints[i + 1] + (controlPoints[i + 1] - controlPoints[i]) // Extrapolate after last
                : controlPoints[i + 2];

            for (int j = 0; j <= resolution; j++)
            {
                float t = j / (float)resolution;
                splinePoints.Add(CatmullRom(p0, p1, p2, p3, t));
            }
        }

        return splinePoints;
    }
    #endregion
}