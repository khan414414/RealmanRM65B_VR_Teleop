using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;

public class VRControllerSender : MonoBehaviour
{
    [SerializeField] private string ipAddress = "127.0.0.1";
    [SerializeField] private int port = 5555;

    [SerializeField] private GameObject controller;
    [SerializeField] private InputActionProperty trackingButton;

    private TcpListener server;
    private TcpClient client;
    private NetworkStream stream;

    private bool isAwaitingConnection = false; // Track if waiting for a connection
    private long sequenceId = 0;
    private int samplesInRateWindow = 0;
    private double rateWindowStartedAt = 0.0;

    void Start()
    {
        StartServer();
        rateWindowStartedAt = Time.realtimeSinceStartupAsDouble;
    }

    void StartServer()
    {
        try
        {
            IPAddress ip = IPAddress.Parse(ipAddress);
            server = new TcpListener(ip, port);
            server.Start();
            Debug.Log($"Server started at {ipAddress}:{port}");
            WaitForClientConnection();
        }
        catch (Exception e)
        {
            Debug.LogError($"Server failed to start: {e.Message}");
        }
    }

    void WaitForClientConnection()
    {
        if (isAwaitingConnection) return;
        
        Debug.Log("Waiting for connection...");
        isAwaitingConnection = true;

        server.BeginAcceptTcpClient(OnClientConnected, null);
    }

    private void OnClientConnected(IAsyncResult ar)
    {
        isAwaitingConnection = false; // Mark that we're no longer awaiting a connection

        try
        {
            client = server.EndAcceptTcpClient(ar);
            client.NoDelay = true;
            stream = client.GetStream();
            samplesInRateWindow = 0;
            rateWindowStartedAt = 0.0;
            Debug.Log("Client connected!");
        }
        catch (ObjectDisposedException)
        {
            // Debug.LogWarning("Server has been stopped. Ignoring client connection.");
        }
        catch (Exception e)
        {
            Debug.LogError($"Error accepting client connection: {e.Message}");
            WaitForClientConnection(); // Retry waiting for a new client
        }
    }

    void OnDisable()
    {
        stream?.Close();
        client?.Close();
        server?.Stop();
    }

    void Update()
    {
        if (client == null || !client.Connected)
        {
            if (client != null)
            {
                client.Close();
                stream = null;
            }
            WaitForClientConnection();
            return;
        }
        // Send exactly one sample for each Unity/XR Dynamic Update.  The old
        // 10 ms gate aliased a 120 Hz Meta stream down to roughly 60 Hz and
        // therefore did not produce the advertised 100 Hz.  A sequence ID and
        // monotonic Unity timestamp let Python resample these native XR frames
        // onto the robot's fixed-rate command clock without guessing.
        // Convert to regular coordinate from Unity coordinate
        Vector3 position = controller.transform.position;
        Quaternion rotation = controller.transform.rotation;
        position = new Vector3(position.z, -position.x, position.y);
        rotation = new Quaternion(rotation.z, -rotation.x, rotation.y, -rotation.w);

        // Get button states
        // Send the Trigger's held state so tracking is active only while pressed.
        bool button1 = trackingButton.action?.IsPressed() ?? false;
        // Keep the ninth protocol field for compatibility, but disable right-hand Grip.
        const bool button2 = false;

        long currentSequence = sequenceId++;
        double sampleTime = Time.realtimeSinceStartupAsDouble;
        if (rateWindowStartedAt <= 0.0)
            rateWindowStartedAt = sampleTime;
        string FormatFloat(float value) =>
            value.ToString("F6", CultureInfo.InvariantCulture);
        string sampleTimeText =
            sampleTime.ToString("F6", CultureInfo.InvariantCulture);
        string data =
            $"{FormatFloat(position.x)},{FormatFloat(position.y)}," +
            $"{FormatFloat(position.z)},{FormatFloat(rotation.x)}," +
            $"{FormatFloat(rotation.y)},{FormatFloat(rotation.z)}," +
            $"{FormatFloat(rotation.w)},{(button1 ? 1 : 0)}," +
            $"{(button2 ? 1 : 0)},{currentSequence}," +
            $"{sampleTimeText}\n";

        try
        {
            // Send data to the Python server
            byte[] message = Encoding.UTF8.GetBytes(data);
            stream.Write(message, 0, message.Length);
            samplesInRateWindow++;
            double elapsed = sampleTime - rateWindowStartedAt;
            if (elapsed >= 3.0)
            {
                Debug.Log(
                    $"Meta/Unity controller stream: " +
                    $"{(samplesInRateWindow / elapsed):F1} Hz");
                samplesInRateWindow = 0;
                rateWindowStartedAt = sampleTime;
            }
        }
        catch (Exception e)
        {
            Debug.Log($"Error writing to stream. Connection may lose: {e.Message}");
            client?.Close();
            stream = null; // Reset stream to handle reconnection
            WaitForClientConnection();
        }

    }
}
