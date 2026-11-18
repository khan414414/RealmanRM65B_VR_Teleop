using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.Interaction.Toolkit;

public class VRControllerSender : MonoBehaviour
{
    [SerializeField] private string ipAddress = "127.0.0.1";
    [SerializeField] private int port = 5555;

    [SerializeField] private GameObject controller;
    [SerializeField] private InputActionProperty trackingButton;
    [SerializeField] private bool emitPoseDiagnostics = true;

    private TcpListener server;
    private TcpClient client;
    private NetworkStream stream;

    private bool isAwaitingConnection = false; // Track if waiting for a connection
    private long sequenceId = 0;
    private int samplesInRateWindow = 0;
    private double rateWindowStartedAt = 0.0;
    private double nextPoseDiagnosticTime = 0.0;

    [Serializable]
    private class InputSourceDiagnostic
    {
        public string action;
        public string control_path;
        public int device_id;
        public string device_name;
        public string device_layout;
        public bool is_tracked;
        public int tracking_state;
        public Vector3 device_position;
    }

    private static InputSourceDiagnostic DescribeInputSource(InputAction action)
    {
        var control = action?.activeControl;
        return DescribeDevice(control?.device, action?.name ?? "", control?.path ?? "");
    }

    private static InputSourceDiagnostic DescribeDevice(InputDevice device, string action, string controlPath)
    {
        var tracked = device as TrackedDevice;
        return new InputSourceDiagnostic
        {
            action = action,
            control_path = controlPath,
            device_id = device?.deviceId ?? -1,
            device_name = device?.displayName ?? "",
            device_layout = device?.layout ?? "",
            is_tracked = tracked != null && tracked.isTracked.isPressed,
            tracking_state = tracked != null ? tracked.trackingState.ReadValue() : 0,
            device_position = tracked != null ? tracked.devicePosition.ReadValue() : Vector3.zero,
        };
    }

    private static InputSourceDiagnostic[] DescribeRightHandDevices()
    {
        var devices = new List<InputSourceDiagnostic>();
        foreach (var device in InputSystem.devices)
            foreach (var usage in device.usages)
                if (usage == UnityEngine.InputSystem.CommonUsages.RightHand)
                {
                    devices.Add(DescribeDevice(device, "", ""));
                    break;
                }
        return devices.ToArray();
    }

    [Serializable]
    private class PoseDiagnostic
    {
        public int schema = 1;
        public long sequence_id;
        public double unity_time_s;
        public string project_path;
        public string controller_path;
        public string device_name;
        public bool raw_position_valid;
        public bool tracking_flag_valid;
        public bool is_tracked;
        public int tracking_state;
        public Vector3 raw_position;
        public Vector3 local_position;
        public Vector3 world_position;
        public Vector3 sent_position;
        public Vector3 parent_scale;
        public Vector3 world_scale;
        // XRNode.RightHand can name a different device from the action driving
        // the Transform. Record the actual active controls before diagnosing it.
        public InputSourceDiagnostic position_source;
        public InputSourceDiagnostic rotation_source;
        public InputSourceDiagnostic tracked_source;
        public InputSourceDiagnostic tracking_state_source;
        public InputSourceDiagnostic trigger_source;
        public InputSourceDiagnostic[] right_hand_devices;
        public bool motion_record_valid;
        public string input_invalid_reason;
    }

    private static bool IsRightTouchController(InputDevice device)
    {
        if (device == null) return false;
        bool rightHand = false;
        foreach (var usage in device.usages)
            if (usage == UnityEngine.InputSystem.CommonUsages.RightHand)
                rightHand = true;
        if (!rightHand) return false;
        // Include OpenXR layouts and the native Oculus XRInputV1 layout seen
        // on the actual headset. HandInteraction also derives from XRController,
        // so a generic XRController check does not distinguish hands from Touch.
        foreach (string layout in new[] { "OculusTouchController", "QuestTouchPlusController", "QuestProTouchController" })
            if (device.layout.IndexOf(layout, StringComparison.OrdinalIgnoreCase) >= 0 ||
                InputSystem.IsFirstLayoutBasedOnSecond(device.layout, layout))
                return true;
        return false;
    }

    private string GetInputInvalidReason()
    {
        var actions = controller.GetComponent<ActionBasedController>();
        if (actions == null) return "missing_action_based_controller";
        var positionDevice = actions.positionAction.action?.activeControl?.device;
        var rotationDevice = actions.rotationAction.action?.activeControl?.device;
        if (!IsRightTouchController(positionDevice)) return "position_source_is_not_right_touch";
        if (rotationDevice != positionDevice) return "position_rotation_device_mismatch";
        var tracked = positionDevice as TrackedDevice;
        if (tracked == null || !tracked.isTracked.isPressed) return "right_touch_not_tracked";
        if ((tracked.trackingState.ReadValue() & 3) != 3) return "right_touch_position_or_rotation_invalid";
        if ((trackingButton.action?.IsPressed() ?? false) &&
            trackingButton.action.activeControl?.device != positionDevice)
            return "trigger_device_mismatch";
        return "";
    }

    private string BuildPoseDiagnostic(long sequence, double sampleTime, Vector3 sentPosition, string invalidReason)
    {
        Transform pose = controller.transform;
        var device = UnityEngine.XR.InputDevices.GetDeviceAtXRNode(UnityEngine.XR.XRNode.RightHand);
        bool hasPosition = device.TryGetFeatureValue(UnityEngine.XR.CommonUsages.devicePosition, out Vector3 rawPosition);
        bool hasTracked = device.TryGetFeatureValue(UnityEngine.XR.CommonUsages.isTracked, out bool isTracked);
        device.TryGetFeatureValue(UnityEngine.XR.CommonUsages.trackingState, out UnityEngine.XR.InputTrackingState trackingState);
        var actionController = controller.GetComponent<ActionBasedController>();
        string path = pose.name;
        for (Transform parent = pose.parent; parent != null; parent = parent.parent)
            path = parent.name + "/" + path;
        var diagnostic = new PoseDiagnostic
        {
            sequence_id = sequence,
            unity_time_s = sampleTime,
            project_path = Application.dataPath,
            controller_path = path,
            device_name = device.name ?? "",
            raw_position_valid = hasPosition,
            tracking_flag_valid = hasTracked,
            is_tracked = isTracked,
            tracking_state = (int)trackingState,
            raw_position = rawPosition,
            local_position = pose.localPosition,
            world_position = pose.position,
            sent_position = sentPosition,
            parent_scale = pose.parent != null ? pose.parent.lossyScale : Vector3.one,
            world_scale = pose.lossyScale,
            position_source = DescribeInputSource(actionController != null ? actionController.positionAction.action : null),
            rotation_source = DescribeInputSource(actionController != null ? actionController.rotationAction.action : null),
            tracked_source = DescribeInputSource(actionController != null ? actionController.isTrackedAction.action : null),
            tracking_state_source = DescribeInputSource(actionController != null ? actionController.trackingStateAction.action : null),
            trigger_source = DescribeInputSource(trackingButton.action),
            right_hand_devices = DescribeRightHandDevices(),
            motion_record_valid = invalidReason.Length == 0,
            input_invalid_reason = invalidReason,
        };
        return "#VR_DIAG " + JsonUtility.ToJson(diagnostic) + "\n";
    }

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

        // Do not feed hand-tracking, inferred, or cached poses to calibration
        // or robot tracking. Python clears its latest target on this record and
        // requires a Trigger release before tracking can resume.
        string invalidReason = GetInputInvalidReason();
        if (invalidReason.Length != 0)
            data = "#VR_INPUT_INVALID " + invalidReason + "\n";

        // Keep the existing 11-field motion record unchanged. These occasional
        // side-channel records let the read-only Python diagnostic compare XR
        // metres with the scene transform, without touching the robot.
        if (emitPoseDiagnostics && sampleTime >= nextPoseDiagnosticTime)
        {
            data += BuildPoseDiagnostic(currentSequence, sampleTime, position, invalidReason);
            nextPoseDiagnosticTime = sampleTime + 0.25;
        }

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
