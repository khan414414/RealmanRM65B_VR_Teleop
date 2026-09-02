using System;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;

public sealed class VRControllerSender : MonoBehaviour
{
    private const int ReceiveBufferSize = 4096;
    private const int MaximumBufferedJointStateBytes = 64 * 1024;
    private const int MaximumReadPassesPerFrame = 8;

    [Header("Python connection")]
    [SerializeField] private string ipAddress = "127.0.0.1";
    [SerializeField] private int port = 5555;

    [Header("Quest input")]
    [SerializeField] private GameObject controller;
    [SerializeField] private InputActionProperty trackingButton;

    [Header("AUBO i7 digital twin")]
    [SerializeField] private AuboI7Visualizer auboI7Visualizer;

    private readonly object connectionLock = new object();
    private readonly byte[] receiveBytes = new byte[ReceiveBufferSize];
    private readonly StringBuilder receivedText = new StringBuilder();

    private TcpListener server;
    private TcpClient client;
    private NetworkStream stream;
    private TcpClient receiverClient;
    private bool isAwaitingConnection;
    private bool isShuttingDown;

    private long sequenceId;
    private long lastJointCommandSequence = -1;
    private int malformedJointStateWarnings;
    private int samplesInRateWindow;
    private double rateWindowStartedAt;

    private void Start()
    {
        if (auboI7Visualizer == null)
            auboI7Visualizer = FindAnyObjectByType<AuboI7Visualizer>();

        StartServer();
        rateWindowStartedAt = Time.realtimeSinceStartupAsDouble;
    }

    private void StartServer()
    {
        try
        {
            IPAddress ip = IPAddress.Parse(ipAddress);
            server = new TcpListener(ip, port);
            server.Start();
            Debug.Log($"Server started at {ipAddress}:{port}");
            WaitForClientConnection();
        }
        catch (Exception exception)
        {
            Debug.LogError($"Server failed to start: {exception.Message}");
        }
    }

    private void WaitForClientConnection()
    {
        TcpListener activeServer;
        lock (connectionLock)
        {
            if (
                isShuttingDown
                || server == null
                || client != null
                || isAwaitingConnection
            )
            {
                return;
            }

            isAwaitingConnection = true;
            activeServer = server;
        }

        try
        {
            Debug.Log("Waiting for connection...");
            activeServer.BeginAcceptTcpClient(OnClientConnected, activeServer);
        }
        catch (ObjectDisposedException)
        {
            lock (connectionLock)
                isAwaitingConnection = false;
        }
        catch (Exception exception)
        {
            lock (connectionLock)
                isAwaitingConnection = false;
            Debug.LogError($"Failed to wait for a Python connection: {exception.Message}");
        }
    }

    private void OnClientConnected(IAsyncResult asyncResult)
    {
        TcpClient acceptedClient = null;
        try
        {
            TcpListener acceptingServer = (TcpListener)asyncResult.AsyncState;
            acceptedClient = acceptingServer.EndAcceptTcpClient(asyncResult);
            acceptedClient.NoDelay = true;
            acceptedClient.SendTimeout = 20;

            lock (connectionLock)
            {
                isAwaitingConnection = false;
                if (isShuttingDown || server != acceptingServer || client != null)
                {
                    acceptedClient.Close();
                    return;
                }

                client = acceptedClient;
                stream = acceptedClient.GetStream();
            }

            Debug.Log("Python client connected; AUBO joint mirroring is active.");
        }
        catch (ObjectDisposedException)
        {
            acceptedClient?.Close();
            lock (connectionLock)
                isAwaitingConnection = false;
        }
        catch (Exception exception)
        {
            acceptedClient?.Close();
            lock (connectionLock)
                isAwaitingConnection = false;
            Debug.LogError($"Error accepting Python connection: {exception.Message}");
            WaitForClientConnection();
        }
    }

    private void OnDisable()
    {
        lock (connectionLock)
        {
            isShuttingDown = true;
            stream?.Close();
            client?.Close();
            stream = null;
            client = null;
            server?.Stop();
            server = null;
            isAwaitingConnection = false;
        }
    }

    private void Update()
    {
        TcpClient activeClient;
        NetworkStream activeStream;
        lock (connectionLock)
        {
            activeClient = client;
            activeStream = stream;
        }

        if (activeClient == null || activeStream == null)
        {
            WaitForClientConnection();
            return;
        }

        if (receiverClient != activeClient)
        {
            receiverClient = activeClient;
            receivedText.Clear();
            lastJointCommandSequence = -1;
            malformedJointStateWarnings = 0;
            samplesInRateWindow = 0;
            rateWindowStartedAt = Time.realtimeSinceStartupAsDouble;
        }

        // TCP is full duplex. Read Python's accepted AUBO joint command first,
        // then publish this frame's Quest input on the same connection.
        if (!ReceiveJointStates(activeClient, activeStream))
            return;

        SendControllerSample(activeClient, activeStream);
    }

    private bool ReceiveJointStates(
        TcpClient activeClient,
        NetworkStream activeStream
    )
    {
        try
        {
            int readPasses = 0;
            while (
                activeStream.DataAvailable
                && readPasses++ < MaximumReadPassesPerFrame
            )
            {
                int count = activeStream.Read(receiveBytes, 0, receiveBytes.Length);
                if (count == 0)
                {
                    DisconnectClient(activeClient, "Python closed the TCP connection.");
                    return false;
                }

                receivedText.Append(Encoding.ASCII.GetString(receiveBytes, 0, count));
                if (receivedText.Length > MaximumBufferedJointStateBytes)
                {
                    receivedText.Clear();
                    WarnAboutJointState(
                        "Joint-state receive buffer exceeded its safety limit and was reset."
                    );
                }
            }

            ParseCompleteJointStateLines();
            return true;
        }
        catch (Exception exception) when (
            exception is IOException
            || exception is SocketException
            || exception is ObjectDisposedException
        )
        {
            DisconnectClient(
                activeClient,
                $"Joint-state receive failed: {exception.Message}"
            );
            return false;
        }
    }

    private void ParseCompleteJointStateLines()
    {
        string buffered = receivedText.ToString();
        int lineStart = 0;
        int newline;
        while ((newline = buffered.IndexOf('\n', lineStart)) >= 0)
        {
            string line = buffered.Substring(lineStart, newline - lineStart).TrimEnd('\r');
            lineStart = newline + 1;
            if (line.Length != 0)
                TryApplyJointState(line);
        }

        if (lineStart == 0)
            return;

        receivedText.Clear();
        if (lineStart < buffered.Length)
            receivedText.Append(buffered, lineStart, buffered.Length - lineStart);
    }

    private void TryApplyJointState(string line)
    {
        // Protocol J1:
        // J1,command_seq,source_unity_seq,source_unity_time,python_time,
        // mode,q1,q2,q3,q4,q5,q6\n  (joint angles are radians)
        string[] fields = line.Split(',');
        if (fields.Length != 12 || fields[0] != "J1")
        {
            WarnAboutJointState($"Ignoring unsupported joint-state record: {line}");
            return;
        }

        NumberStyles integerStyle = NumberStyles.Integer;
        NumberStyles floatStyle = NumberStyles.Float;
        CultureInfo culture = CultureInfo.InvariantCulture;

        if (
            !long.TryParse(fields[1], integerStyle, culture, out long commandSequence)
            || commandSequence < 0
            || !long.TryParse(
                fields[2],
                integerStyle,
                culture,
                out long sourceSequence
            )
            || sourceSequence < -1
            || !double.TryParse(fields[3], floatStyle, culture, out double sourceTime)
            || !double.TryParse(fields[4], floatStyle, culture, out double pythonTime)
            || !IsFinite(sourceTime)
            || sourceTime < -1.0
            || !IsFinite(pythonTime)
            || pythonTime < 0.0
        )
        {
            WarnAboutJointState($"Ignoring invalid joint-state metadata: {line}");
            return;
        }

        if (commandSequence <= lastJointCommandSequence)
            return;

        float[] jointsRadians = new float[AuboI7Visualizer.JointCount];
        for (int index = 0; index < jointsRadians.Length; index++)
        {
            if (
                !float.TryParse(
                    fields[index + 6],
                    floatStyle,
                    culture,
                    out jointsRadians[index]
                )
                || !IsFinite(jointsRadians[index])
            )
            {
                WarnAboutJointState($"Ignoring invalid AUBO joint angle: {line}");
                return;
            }
        }

        if (auboI7Visualizer == null)
            auboI7Visualizer = FindAnyObjectByType<AuboI7Visualizer>();
        if (auboI7Visualizer == null)
        {
            WarnAboutJointState("No AuboI7Visualizer exists in the active scene.");
            return;
        }

        auboI7Visualizer.ApplyJointState(
            jointsRadians,
            commandSequence,
            pythonTime,
            fields[5]
        );
        lastJointCommandSequence = commandSequence;
    }

    private void SendControllerSample(
        TcpClient activeClient,
        NetworkStream activeStream
    )
    {
        if (controller == null)
        {
            Debug.LogError("VRControllerSender requires a Quest controller object.");
            enabled = false;
            return;
        }

        // Convert Unity's controller pose to the existing Python-side convention.
        Vector3 position = controller.transform.position;
        Quaternion rotation = controller.transform.rotation;
        position = new Vector3(position.z, -position.x, position.y);
        rotation = new Quaternion(rotation.z, -rotation.x, rotation.y, -rotation.w);

        bool triggerHeld = trackingButton.action?.IsPressed() ?? false;
        const bool gripHeld = false;

        long currentSequence = sequenceId++;
        double sampleTime = Time.realtimeSinceStartupAsDouble;
        string data = string.Format(
            CultureInfo.InvariantCulture,
            "{0:F6},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6},{6:F6},{7},{8},{9},{10:F6}\n",
            position.x,
            position.y,
            position.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
            triggerHeld ? 1 : 0,
            gripHeld ? 1 : 0,
            currentSequence,
            sampleTime
        );

        try
        {
            byte[] message = Encoding.ASCII.GetBytes(data);
            activeStream.Write(message, 0, message.Length);
            samplesInRateWindow++;
            double elapsed = sampleTime - rateWindowStartedAt;
            if (elapsed >= 3.0)
            {
                Debug.Log(
                    $"Meta/Unity controller stream: "
                    + $"{(samplesInRateWindow / elapsed):F1} Hz"
                );
                samplesInRateWindow = 0;
                rateWindowStartedAt = sampleTime;
            }
        }
        catch (Exception exception) when (
            exception is IOException
            || exception is SocketException
            || exception is ObjectDisposedException
        )
        {
            DisconnectClient(
                activeClient,
                $"Controller send failed: {exception.Message}"
            );
        }
    }

    private void DisconnectClient(TcpClient expectedClient, string reason)
    {
        bool disconnected = false;
        lock (connectionLock)
        {
            if (client != expectedClient)
                return;

            stream?.Close();
            client?.Close();
            stream = null;
            client = null;
            disconnected = true;
        }

        if (disconnected)
        {
            Debug.LogWarning(reason);
            WaitForClientConnection();
        }
    }

    private void WarnAboutJointState(string message)
    {
        if (malformedJointStateWarnings < 3)
            Debug.LogWarning(message);
        else if (malformedJointStateWarnings == 3)
            Debug.LogWarning("Further malformed joint-state warnings are suppressed.");
        malformedJointStateWarnings++;
    }

    private static bool IsFinite(double value)
    {
        return !double.IsNaN(value) && !double.IsInfinity(value);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
