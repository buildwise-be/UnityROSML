using System;
using System.Collections.Generic;
using RosMessageTypes.RclInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>Tool visual selection and read-only verification against all three ROS model consumers.</summary>
public sealed class ToolSelectionController : MonoBehaviour
{
    [SerializeField] private string _selectedTool = "kenos";
    [SerializeField] private string[] _modelServices =
    {
        "/robot_state_publisher/get_parameters", "/move_group/get_parameters", "/mtc_test_task_node/get_parameters"
    };
    [SerializeField, Min(1f)] private float _refreshSeconds = 5f;
    [SerializeField, Min(1f)] private float _requestTimeoutSeconds = 10f;
    private static ToolSelectionController _instance;
    public static ToolSelectionController Instance
    {
        get
        {
            if (_instance == null) _instance = FindFirstObjectByType<ToolSelectionController>();
            return _instance;
        }
    }

    private sealed class Inspection
    {
        public string Service;
        public bool Pending;
        public float RequestedAt;
        public int RequestSerial;
        public float ReceivedAt = float.NegativeInfinity;
        public RosRobotModel Model;
        public string Error;
    }
    private RosRobotModelCatalog _catalog;
    private readonly Dictionary<string, RosRobotModel> _knownModels = new Dictionary<string, RosRobotModel>();
    private Inspection[] _inspections;
    private ROSConnection _connection;
    private bool _wasConnected;
    private bool _userSelected;
    private bool _rigsValid = true;
    private int _connectionEpoch;
    private float _nextRefresh;
    private string _status = "Loading tool models...";

    public string SelectedTool => _selectedTool;
    public RosRobotModel CurrentModel => _knownModels.TryGetValue(_selectedTool, out RosRobotModel model)
        ? model : _catalog?.Find(_selectedTool);
    public string StatusText => _status;
    public bool IsRosModelMatched { get; private set; }
    public event Action StateChanged;

    // Legacy scenes without this menu retain their existing behavior.
    public static bool CanSendRobotCommands => !RobotCoordinateService.IsCalibrationEditActive &&
        (Instance == null || Instance.IsRosModelMatched);
    public static bool CanPlanPickPlace => !RobotCoordinateService.IsCalibrationEditActive &&
        (Instance == null || (Instance.IsRosModelMatched && Instance.SelectedTool != "none"));

    private void Awake()
    {
        _instance = this;
        try
        {
            _catalog = RosRobotModelCatalog.Load();
            if (_catalog.Find(_selectedTool) == null) _selectedTool = "kenos";
            if (_modelServices == null || _modelServices.Length != 3)
                throw new InvalidOperationException("Configure robot-state publisher, MoveIt and MTC model services.");
            _inspections = new Inspection[_modelServices.Length];
            for (int i = 0; i < _inspections.Length; i++) _inspections[i] = new Inspection { Service = _modelServices[i] };
            _status = "Local preview. Waiting for ROS models.";
        }
        catch (Exception exception)
        {
            _status = "Robot model could not be loaded.";
            Debug.LogError(exception, this);
        }
    }

    private void Start()
    {
        if (_inspections == null) return;
        _connection = ROSService.Instance.GetROSConnection();
        foreach (Inspection inspection in _inspections)
            _connection.RegisterRosService<GetParametersRequest, GetParametersResponse>(inspection.Service);
        StateChanged?.Invoke();
    }

    private void Update()
    {
        if (_connection == null || _inspections == null) return;
        bool connected = _connection.HasConnectionThread && !_connection.HasConnectionError;
        if (connected != _wasConnected)
        {
            _wasConnected = connected;
            _connectionEpoch++;
            foreach (Inspection inspection in _inspections)
            {
                inspection.Pending = false; inspection.Model = null;
                inspection.RequestSerial++;
                inspection.ReceivedAt = float.NegativeInfinity; inspection.Error = null;
            }
            _nextRefresh = Time.realtimeSinceStartup;
            RefreshStatus();
        }
        foreach (Inspection inspection in _inspections)
        {
            if (!inspection.Pending || Time.realtimeSinceStartup - inspection.RequestedAt <= _requestTimeoutSeconds) continue;
            // The connector has no service timeout; a node starting late can
            // otherwise leave this inspection pending forever. Supersede it.
            inspection.Pending = false;
            inspection.RequestSerial++;
        }
        if (connected && Time.realtimeSinceStartup >= _nextRefresh)
        {
            _nextRefresh = Time.realtimeSinceStartup + _refreshSeconds;
            foreach (Inspection inspection in _inspections) if (!inspection.Pending) Inspect(inspection);
        }
        RefreshStatus();
    }

    public void SelectTool(string tool)
    {
        if (_catalog?.Find(tool) == null) return;
        _userSelected = true;
        if (_selectedTool == tool) { RefreshStatus(); return; }
        _selectedTool = tool;
        ApplySelection();
        RefreshStatus();
        StateChanged?.Invoke();
    }

    private void ApplySelection()
    {
        IsRosModelMatched = false;
        _rigsValid = true;
        foreach (UR16eTrajectoryVisualizer visualizer in FindObjectsByType<UR16eTrajectoryVisualizer>(FindObjectsSortMode.None))
            visualizer.SetTrajectory(null);
        foreach (RosRobotRig rig in FindObjectsByType<RosRobotRig>(FindObjectsSortMode.None))
            if (rig != null && rig.isActiveAndEnabled && !rig.ApplyModel(CurrentModel)) _rigsValid = false;
        InvalidatePlans();
    }

    public bool ConfigureRobot(RosRobotRig rig)
    {
        bool valid = rig != null && CurrentModel != null && rig.ApplyModel(CurrentModel);
        if (!valid) _rigsValid = false;
        RefreshStatus();
        return valid;
    }

    private void Inspect(Inspection inspection)
    {
        inspection.Pending = true;
        inspection.RequestedAt = Time.realtimeSinceStartup;
        int serial = ++inspection.RequestSerial;
        int epoch = _connectionEpoch;
        _connection.SendServiceMessage<GetParametersResponse>(inspection.Service,
            new GetParametersRequest(new[] { "robot_description" }), response =>
            {
                if (this == null || epoch != _connectionEpoch || serial != inspection.RequestSerial) return;
                inspection.Pending = false;
                string xml = response?.values != null && response.values.Length == 1 && response.values[0].type == 4
                    ? response.values[0].string_value : null;
                AcceptDescription(inspection, xml);
            });
    }

    private void AcceptDescription(Inspection inspection, string xml)
    {
        RosRobotModel old = inspection.Model;
        bool valid = RosRobotDescription.TryParse(xml, out RosRobotModel model, out string error);
        inspection.Model = model; inspection.Error = error; inspection.ReceivedAt = Time.realtimeSinceStartup;
        if (inspection == _inspections[0] && valid &&
            (old == null || !RosRobotDescription.Equivalent(old, model, out _)))
        {
            _knownModels[model.tool] = model;
            if (!_userSelected) _selectedTool = model.tool;
            ApplySelection();
        }
        RefreshStatus();
    }

    /// <summary>Also supports inspecting exported descriptions without opening a ROS connection.</summary>
    public bool InspectDescription(string service, string xml)
    {
        if (_inspections == null) return false;
        foreach (Inspection inspection in _inspections) if (inspection.Service == service)
        {
            AcceptDescription(inspection, xml);
            return inspection.Model != null;
        }
        return false;
    }

    private void RefreshStatus()
    {
        bool matched = _rigsValid && CurrentModel != null && _inspections != null;
        string status = "Local preview. Waiting for ROS models.";
        if (_inspections != null)
        {
            foreach (Inspection inspection in _inspections)
            {
                if (inspection.Model == null || Time.realtimeSinceStartup - inspection.ReceivedAt > _refreshSeconds * 3f)
                {
                    matched = false;
                    status = inspection.Error == null ? "Local preview. Waiting for ROS models." : "ROS model could not be verified.";
                    break;
                }
                if (inspection.Model.tool != _selectedTool)
                {
                    matched = false;
                    status = "ROS uses " + Title(inspection.Model.tool) + ". Restart ROS with tool:=" + _selectedTool + ".";
                    break;
                }
                if (!RosRobotDescription.Equivalent(CurrentModel, inspection.Model, out _))
                {
                    matched = false;
                    status = "ROS components use different models. Planning is blocked.";
                    break;
                }
            }
        }
        if (!_rigsValid) { matched = false; status = "The selected robot visual could not be loaded."; }
        if (matched) status = _selectedTool == "none"
            ? "Matches ROS: No tool. TCP verified; vacuum pick/place unavailable."
            : "Matches ROS: " + Title(_selectedTool) + ". Arm, tool and TCP verified.";
        bool matchChanged = IsRosModelMatched != matched;
        bool changed = IsRosModelMatched != matched || _status != status;
        IsRosModelMatched = matched;
        _status = status;
        if (matchChanged) InvalidatePlans();
        if (changed) StateChanged?.Invoke();
    }

    private static string Title(string tool) => tool == "kenos_sl" ? "Kenos SL" : tool == "kenos" ? "Kenos" : "No tool";

    private static void InvalidatePlans()
    {
        foreach (PickPlaceController controller in FindObjectsByType<PickPlaceController>(FindObjectsSortMode.None))
            controller.InvalidatePlanningRequest();
        foreach (MTCPlansManager manager in FindObjectsByType<MTCPlansManager>(FindObjectsSortMode.None))
            manager.InvalidatePlan();
        foreach (TcpPlanningController controller in FindObjectsByType<TcpPlanningController>(FindObjectsSortMode.None))
            controller.InvalidatePlan();
    }

    private void OnDestroy()
    {
        _connectionEpoch++;
        if (_instance == this) _instance = null;
        StateChanged = null;
    }
}
