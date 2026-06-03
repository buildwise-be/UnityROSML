using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Serialization;

public class TargetsManager : MonoBehaviour
{
    [SerializeField]
    private GameObject targetPrefab;

    public int TargetCount => _targetCount;
    private int _targetCount = 0;
    [HideInInspector]
    public List<Target> targets;
    private bool _facingDownTargets = false;
    
    [SerializeField]
    private GameObject targetNumberPrefab;
    private List<GameObject> _targetNumbersInstances;

    private void Start()
    {
        targets = new List<Target>();
        _targetNumbersInstances = new List<GameObject>();
    }

    private void Update()
    {
        foreach (var target in targets)
        {
            int nb = target.TargetNumber;
            _targetNumbersInstances[nb-1].transform.position = target.transform.position + 0.1f * Vector3.up;
        }
    }

    public void CreateNewTarget()
    {
        GameObject target = Instantiate(targetPrefab);
        target.AddComponent<Target>();
        Target targetScript = target.GetComponent<Target>();
        targetScript.TargetNumber = ++_targetCount;
        targetScript.FacingDown = _facingDownTargets;
        targets.Add(targetScript);
        target.transform.position = Camera.main.transform.position + 0.5f * Camera.main.transform.forward;
        
        var targetNumberInstance = Instantiate(targetNumberPrefab);
        _targetNumbersInstances.Add(targetNumberInstance);
        targetNumberInstance.transform.position = 0.1f * Camera.main.transform.up + 0.5f * Camera.main.transform.forward;
        TextMeshProUGUI textMeshProUGUI = targetNumberInstance.GetComponentInChildren<TextMeshProUGUI>();
        if (textMeshProUGUI != null)
        {
            textMeshProUGUI.text = _targetCount.ToString();
        }
    }

    public void ClearAllTargets()
    {
        foreach (Target target in targets)
            Destroy(target.gameObject);
        foreach (var target in _targetNumbersInstances)
            Destroy(target.gameObject);
        
        targets.Clear();
        _targetNumbersInstances.Clear();
        _targetCount = 0;
    }

    public void ForceFacingDownPath(bool forceDown)
    {
        _facingDownTargets = forceDown;
        foreach (Target targetScript in targets)
        {
            targetScript.FacingDown = _facingDownTargets;
        }
    }
}