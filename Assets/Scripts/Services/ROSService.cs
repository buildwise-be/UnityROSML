using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

/// <summary>
/// Singleton service for centralized ROS communication.
/// Wraps ROSConnection singleton and manages all ROS publishers/subscribers.
/// </summary>
public class ROSService : MonoBehaviour
{
    private static ROSService _instance;
    public static ROSService Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindFirstObjectByType<ROSService>();
                if (_instance == null)
                {
                    GameObject go = new GameObject("ROSService");
                    _instance = go.AddComponent<ROSService>();
                    DontDestroyOnLoad(go);
                }
            }
            return _instance;
        }
    }

    private ROSConnection _rosConnection;
    private bool _isInitialized = false;

    private void Awake()
    {
        if (_instance == null)
        {
            _instance = this;
            DontDestroyOnLoad(gameObject);
            Initialize();
        }
        else if (_instance != this)
        {
            Destroy(gameObject);
        }
    }

    private void Initialize()
    {
        _rosConnection = ROSConnection.GetOrCreateInstance();
        _isInitialized = true;
        Debug.Log("ROSService: Initialized");
    }

    /// <summary>
    /// Register a ROS publisher for a topic.
    /// </summary>
    public void RegisterPublisher<T>(string topicName) where T : Message, new()
    {
        if (!_isInitialized)
        {
            Debug.LogError("ROSService: Not initialized!");
            return;
        }
        _rosConnection.RegisterPublisher<T>(topicName);
    }

    /// <summary>
    /// Publish a message to a ROS topic.
    /// </summary>
    public void Publish<T>(string topicName, T message) where T : Message
    {
        if (!_isInitialized)
        {
            Debug.LogError("ROSService: Not initialized!");
            return;
        }
        _rosConnection.Publish(topicName, message);
    }

    /// <summary>
    /// Subscribe to a ROS topic.
    /// </summary>
    public void Subscribe<T>(string topicName, System.Action<T> callback) where T : Message, new()
    {
        if (!_isInitialized)
        {
            Debug.LogError("ROSService: Not initialized!");
            return;
        }
        _rosConnection.Subscribe<T>(topicName, callback);
    }

    /// <summary>
    /// Get the underlying ROSConnection instance if needed for advanced operations.
    /// </summary>
    public ROSConnection GetROSConnection()
    {
        return _rosConnection;
    }

    public bool IsInitialized => _isInitialized;
    public bool IsConnected => _isInitialized && _rosConnection != null && _rosConnection.HasConnectionThread;
}

