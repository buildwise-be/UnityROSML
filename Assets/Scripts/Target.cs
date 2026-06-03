using TMPro;
using UnityEngine;

public class Target : MonoBehaviour
{
    public int TargetNumber
    {
        get => _targetNumber;
        set
        {
            _targetNumber = value;
            //OnTargetNumberAssigned();
        }
    }
    private int _targetNumber;
    public bool FacingDown { get; set; }
    
    private void Update()
    {
        transform.up = FacingDown ?  -Vector3.up : transform.up;
    }

    private void OnTargetNumberAssigned()
    {
        TextMeshProUGUI textMeshProUGUI = gameObject.GetComponentInChildren<TextMeshProUGUI>();
        if (textMeshProUGUI != null)
        {
            textMeshProUGUI.text = _targetNumber.ToString();
        }
    }
}