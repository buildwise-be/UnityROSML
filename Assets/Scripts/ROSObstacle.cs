using UnityEngine;

public class ROSObstacle : MonoBehaviour
{
    private bool _isObstacleEnabled = false;
    public string guid = "-1";

    public bool IsObstacleEnabled { 
        get { return _isObstacleEnabled; }  
        set { _isObstacleEnabled = value; UpdateObstacleVisual(); } 
    }

    private Color _initialColor;

    private void OnEnable()
    {
        if (TryGetComponent<MeshRenderer>(out MeshRenderer mr))
        {
            _initialColor = mr.material.color;
        }
    }

    private void UpdateObstacleVisual()
    {
        if (TryGetComponent<MeshRenderer>(out MeshRenderer mr))
        {
            if (_isObstacleEnabled)
            {
                mr.material.color = Color.green;
            }
            else
            {
                mr.material.color = _initialColor;
            }
        }
    }
}
