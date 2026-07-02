using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using RosMessageTypes.Geometry;
using UnityEngine;

/// <summary>
/// Singleton client that sends PoseStamped data to a remote machine over plain TCP,
/// where a small C++ "launcher" server receives it and starts the target executable.
///
/// Wire protocol (deliberately trivial so the C++ side stays dependency-free):
///   - Client connects to host:port.
///   - Client sends ONE line of UTF-8 JSON terminated by '\n' (see RemoteExecutionPayload).
///   - Server replies with ONE line of JSON, e.g. {"ok":true,"exit_code":0}, then closes.
///
/// This is independent from the ROS TCP connector; nothing here touches ROS topics.
/// </summary>
public class RemoteExecutorClient : MonoBehaviour
{
    private static RemoteExecutorClient _instance;
    public static RemoteExecutorClient Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = FindFirstObjectByType<RemoteExecutorClient>();
                if (_instance == null)
                {
                    GameObject go = new GameObject("RemoteExecutorClient");
                    _instance = go.AddComponent<RemoteExecutorClient>();
                    DontDestroyOnLoad(go);
                }
            }
            return _instance;
        }
    }

    [SerializeField]
    [Tooltip("IP or hostname of the machine running the C++ launcher server")]
    private string _serverHost = "192.168.1.100";

    [SerializeField]
    [Tooltip("TCP port the C++ launcher server listens on")]
    private int _serverPort = 6001;

    [SerializeField]
    [Tooltip("Timeout for connecting to the server, in seconds")]
    private float _connectTimeoutSeconds = 3f;

    [SerializeField]
    [Tooltip("Timeout for the server reply, in seconds. The server replies once the executable has been launched (or has finished, depending on server config), so size this accordingly.")]
    private float _responseTimeoutSeconds = 15f;

    public string ServerHost { get => _serverHost; set => _serverHost = value; }
    public int ServerPort { get => _serverPort; set => _serverPort = value; }

    // Callbacks queued from the worker task, invoked on the Unity main thread in Update().
    private readonly ConcurrentQueue<Action> _mainThreadQueue = new ConcurrentQueue<Action>();

    private void Awake()
    {
        if (_instance == null)
        {
            _instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else if (_instance != this)
        {
            Destroy(gameObject);
        }
    }

    private void Update()
    {
        while (_mainThreadQueue.TryDequeue(out Action action))
        {
            action();
        }
    }

    /// <summary>
    /// Send a single PoseStamped to the remote launcher.
    /// </summary>
    public void Execute(string command, PoseStampedMsg pose, Action<RemoteExecutionResult> onResult = null)
    {
        Execute(command, new[] { pose }, onResult);
    }

    /// <summary>
    /// Send a list of PoseStamped to the remote launcher, which will start the
    /// configured executable with the poses as input.
    /// </summary>
    /// <param name="command">Logical command name; the server can use it to pick which executable to run.</param>
    /// <param name="poses">Poses to forward as arguments/input to the executable.</param>
    /// <param name="onResult">Invoked on the Unity main thread with the outcome.</param>
    public void Execute(string command, PoseStampedMsg[] poses, Action<RemoteExecutionResult> onResult = null)
    {
        if (poses == null || poses.Length == 0)
        {
            Debug.LogError("RemoteExecutorClient: No poses provided.");
            DispatchResult(onResult, new RemoteExecutionResult { Success = false, Error = "No poses provided" });
            return;
        }

        var payload = new RemoteExecutionPayload
        {
            command = command,
            poses = Array.ConvertAll(poses, RemotePoseStamped.FromMsg)
        };
        string json = JsonUtility.ToJson(payload);

        string host = _serverHost;
        int port = _serverPort;
        float connectTimeout = _connectTimeoutSeconds;
        float responseTimeout = _responseTimeoutSeconds;

        Debug.Log($"RemoteExecutorClient: Sending '{command}' with {poses.Length} pose(s) to {host}:{port}\n{json}");

        Task.Run(async () =>
        {
            RemoteExecutionResult result = await SendAsync(host, port, json, connectTimeout, responseTimeout);
            DispatchResult(onResult, result);
        });
    }

    private void DispatchResult(Action<RemoteExecutionResult> onResult, RemoteExecutionResult result)
    {
        _mainThreadQueue.Enqueue(() =>
        {
            if (result.Success)
            {
                Debug.Log($"RemoteExecutorClient: Success. Server replied: {result.Response}");
            }
            else
            {
                Debug.LogError($"RemoteExecutorClient: Failed. {result.Error}");
            }
            onResult?.Invoke(result);
        });
    }

    private static async Task<RemoteExecutionResult> SendAsync(
        string host, int port, string jsonLine, float connectTimeoutSeconds, float responseTimeoutSeconds)
    {
        try
        {
            using (var client = new TcpClient())
            {
                Task connectTask = client.ConnectAsync(host, port);
                Task connectTimeout = Task.Delay(TimeSpan.FromSeconds(connectTimeoutSeconds));
                if (await Task.WhenAny(connectTask, connectTimeout) == connectTimeout)
                {
                    return new RemoteExecutionResult
                    {
                        Success = false,
                        Error = $"Connection to {host}:{port} timed out after {connectTimeoutSeconds}s"
                    };
                }
                await connectTask; // rethrow connection exceptions if any

                NetworkStream stream = client.GetStream();

                byte[] data = Encoding.UTF8.GetBytes(jsonLine + "\n");
                await stream.WriteAsync(data, 0, data.Length);
                await stream.FlushAsync();

                // Read the single-line JSON reply.
                Task<string> readTask = ReadLineAsync(stream);
                Task responseTimeout = Task.Delay(TimeSpan.FromSeconds(responseTimeoutSeconds));
                if (await Task.WhenAny(readTask, responseTimeout) == responseTimeout)
                {
                    return new RemoteExecutionResult
                    {
                        Success = false,
                        Error = $"No reply from {host}:{port} within {responseTimeoutSeconds}s"
                    };
                }

                string response = await readTask;
                if (string.IsNullOrEmpty(response))
                {
                    return new RemoteExecutionResult
                    {
                        Success = false,
                        Error = "Server closed the connection without replying"
                    };
                }

                // Minimal success check without needing a JSON parser for the reply.
                bool ok = response.Contains("\"ok\":true") || response.Contains("\"ok\": true");
                return new RemoteExecutionResult
                {
                    Success = ok,
                    Response = response,
                    Error = ok ? null : $"Server reported failure: {response}"
                };
            }
        }
        catch (Exception e)
        {
            return new RemoteExecutionResult
            {
                Success = false,
                Error = $"{e.GetType().Name}: {e.Message}"
            };
        }
    }

    private static async Task<string> ReadLineAsync(Stream stream)
    {
        var buffer = new byte[1];
        var sb = new StringBuilder();
        while (true)
        {
            int read = await stream.ReadAsync(buffer, 0, 1);
            if (read == 0)
            {
                break; // connection closed
            }
            char c = (char)buffer[0];
            if (c == '\n')
            {
                break;
            }
            sb.Append(c);
        }
        return sb.ToString().TrimEnd('\r');
    }
}
