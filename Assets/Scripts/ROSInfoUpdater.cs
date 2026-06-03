using RosMessageTypes.Ur16eUnityInterfaces;
using TMPro;
using UnityEngine;

public class ROSInfoUpdater : MonoBehaviour
{
    [SerializeField]
    private Ur16EServiceCaller _uR16EServiceCaller;

    private TextMeshProUGUI _textInfo;
    private int _callCount = 0;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        _textInfo = GetComponentInChildren<TextMeshProUGUI>();
        _uR16EServiceCaller.onSingleTargetPlanServiceCalled.AddListener(UpdateCallInfoText);
        _uR16EServiceCaller.onSingleTargetPlanResponseReceived.AddListener(UpdateInfoText);
    }

    void UpdateCallInfoText()
    {
        _callCount += 1;
        _textInfo.text += $"\nNew trajectory request sent... ({_callCount})";
    }

    void UpdateInfoText(UR16eMoverServiceResponse resp)
    {
        if (!resp.success)
        {
            _textInfo.text += $"\nError while computing trajectory: {resp.error_message}";
        }
        else
        {
            _textInfo.text += $"\nSuccess!";
        }
    }
}
