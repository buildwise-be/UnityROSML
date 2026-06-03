using System;
using UnityEngine;
using UnityEngine.UI;

[Obsolete]
public class TrajectoryLineEnabler : MonoBehaviour
{
    private UR16eTrajectoryVisualizer _visualizer;
    private Slider _slider;
    private Ur16EServiceCaller _serviceCaller;
   
    private void OnEnable()
    {
        _visualizer = FindAnyObjectByType<UR16eTrajectoryVisualizer>();
        _serviceCaller = FindAnyObjectByType<Ur16EServiceCaller>();
        _slider = GetComponentInChildren<Slider>();

        Toggle[] cartesianTrajectoryToggle = GetComponentsInChildren<Toggle>();
        foreach (Toggle toggle in cartesianTrajectoryToggle)
        {
            if (toggle.name == "CartesianTrajectoryToggle")
            {
                toggle.onValueChanged.AddListener(value => _serviceCaller.UseCartesian = value);
                break;
            }
        }
    }

    public void ToggleTrajectoryLine(bool enable)
    {
        if (_visualizer != null)
        {
            _visualizer.ComputeTrajectorySpline();
            _visualizer.GetComponent<LineRenderer>().enabled = enable;
            _visualizer.GetComponent<LineRenderer>().widthMultiplier = _slider.value;
        }
    }

    public void ChangeLineThickness(float width)
    {
        _visualizer.GetComponent<LineRenderer>().widthMultiplier = width;
    }
}