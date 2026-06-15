using System;
using TMPro;
using UnityEngine;

public class MTCPlanUIManager : MonoBehaviour
{
    [SerializeField]
    private MTCPlansManager _MTCPlansManager;

    private TextMeshProUGUI _textBox;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        _MTCPlansManager.OnSuccessfulPlan.AddListener(UpdatePlanStatusPanel);
        _textBox = GetComponentInChildren<TextMeshProUGUI>();
    }

    private void UpdatePlanStatusPanel(bool success)
    {
        if (success)
        {
            _textBox.text = "Last plan: Success";
        }
        else
        {
            _textBox.text = "Last plan: Failure";
        }
    }

    public void ResetPlanStatusPanel()
    {
        _textBox.text = "Last plan: Waiting...";
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
