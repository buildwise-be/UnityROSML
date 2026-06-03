using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.Samples.SpatialKeyboard;

public class RosConnectionChecker : MonoBehaviour
{
    [SerializeField]
    private Image rosHasConnectionThreadImage;
    [SerializeField]
    private Image rosHasConnectionErrorImage;
    [SerializeField]
    private TextMeshProUGUI IPText;
    private ROSConnection ros;

    void Start()
    {
        // Get (or create) the ROSConnection singleton
        ros = ROSConnection.GetOrCreateInstance();
    }

    private void Update()
    {
        if (ros.HasConnectionThread)
        {
            rosHasConnectionThreadImage.color = Color.green;
        }
        else
        {
            rosHasConnectionThreadImage.color = Color.red;
        }
        if (!ros.HasConnectionError)
        {
            rosHasConnectionErrorImage.color = Color.green;
        }
        else
        {
            rosHasConnectionErrorImage.color = Color.red;
        }
        IPText.text = ros.RosIPAddress + ":" + ros.RosPort;
    }

    public void SetROSIPString(KeyboardTextEventArgs args)
    {
        ros.RosIPAddress = args.keyboardText;
    }
}
