using System.Collections;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class ConnectionChecker : MonoBehaviour
{
    [SerializeField]
    private URInstantiator _URInstantiator;
    [SerializeField]
    private float _initialDelaySeconds = 5.0f;
    [SerializeField]
    private float _retryDelaySeconds = 2.0f;

    private bool _connectionEstablished = false;
    private TextMeshProUGUI _textBox;
    private Image _image;

    void Start()
    {
        _textBox = GetComponentInChildren<TextMeshProUGUI>();
        _image = GetComponent<Image>();

        if (_textBox == null)
        {
            Debug.LogWarning("ConnectionChecker: TextMeshProUGUI child was not found.");
        }

        if (_image == null)
        {
            Debug.LogWarning("ConnectionChecker: Image component was not found.");
        }

        if (_URInstantiator != null)
        {
            _URInstantiator.OnURInstantiated.AddListener(PlaceAndShowPanel);
        }
        else
        {
            Debug.LogWarning("ConnectionChecker: URInstantiator is not assigned.");
        }

        StartCoroutine(CheckConnectionCoroutine());
    }

    private void OnDestroy()
    {
        if (_URInstantiator != null)
        {
            _URInstantiator.OnURInstantiated.RemoveListener(PlaceAndShowPanel);
        }
    }

    private IEnumerator CheckConnectionCoroutine()
    {
        yield return new WaitForSeconds(_initialDelaySeconds);

        while (!_connectionEstablished)
        {
            bool checkCompleted = false;

            ROSService.Instance.CheckConnection(isConnected =>
            {
                if (isConnected)
                {
                    Debug.Log("Unity can talk to ROS.");
                    if (_textBox != null)
                    {
                        _textBox.text = "Connection : OK!";
                    }

                    _connectionEstablished = true;
                }
                else
                {
                    Debug.LogWarning("Unity cannot reach ROS.");
                }

                checkCompleted = true;
            });

            while (!checkCompleted)
            {
                yield return null;
            }

            if (!_connectionEstablished)
            {
                yield return new WaitForSeconds(_retryDelaySeconds);
            }
        }
    }

    void PlaceAndShowPanel(GameObject go, Pose p)
    {
        Transform panelRoot = transform.parent.parent != null ? transform.parent.parent : transform;
        panelRoot.position = p.position + Vector3.up * 0.3f;

        if (_image != null)
        {
            _image.enabled = true;
        }

        if (_textBox != null)
        {
            _textBox.enabled = true;
        }
    }
}
