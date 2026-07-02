using System;
using System.Collections;
using System.Collections.Generic;
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
    public bool IsConnected => _isInitialized && _rosConnection != null && _rosConnection.HasConnectionThread &&
    !_rosConnection.HasConnectionError;

    [SerializeField] private float timeoutSeconds = 2f;

    public void CheckConnection(Action<bool> onResult)
    {
        StartCoroutine(CheckConnectionCoroutine(onResult));
    }

    private IEnumerator CheckConnectionCoroutine(Action<bool> onResult)
    {
        ROSConnection ros = ROSConnection.GetOrCreateInstance();

        bool gotReply = false;
        Dictionary<string, string> topics = null;

        ros.GetTopicAndTypeList(reply =>
        {
            gotReply = true;
            topics = reply;
        });

        float deadline = Time.realtimeSinceStartup + timeoutSeconds;

        while (!gotReply && Time.realtimeSinceStartup < deadline)
        {
            yield return null;
        }

        bool connected =
            gotReply &&
            ros.HasConnectionThread &&
            !ros.HasConnectionError;

        onResult?.Invoke(connected);

        if (connected)
        {
            Debug.Log($"ROS connected. Found {topics.Count} topics.");
        }
        else
        {
            Debug.LogWarning("ROS connection check timed out or connection error occurred.");
        }
    }
}

