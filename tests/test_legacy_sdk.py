"""Exercise the real 4.2.8 DLL against a localhost fake controller, never hardware."""

import ctypes as ct
import json
import queue
import socketserver
import sys
import threading
import unittest
from unittest.mock import MagicMock, patch

from Realman_Control.legacy_sdk import LegacyArm


class ControllerHandler(socketserver.StreamRequestHandler):
    def handle(self):
        buffer = b""
        for line in self.rfile:
            buffer += line
            try:
                request = json.loads(buffer)
            except json.JSONDecodeError:
                continue  # This SDK pretty-prints some requests over several lines.
            buffer = b""
            self.server.requests.put(request)
            command = request["command"]
            if command == "get_arm_software_version":
                response = {
                    "state": "arm_software_version", "Ctrl_version": 7012676,
                    "Plan_version": 7012675, "Real-time_Kernal1": 7012676,
                    "Real-time_Kernal2": 7012676, "Product_version": "RM65-BI",
                }
            elif command == "get_joint_degree":
                response = {"state": "joint_degree", "joint": [1000, -2000, 3000, 0, 0, 0]}
            elif command == "get_current_tool_frame":
                response = {"state": "current_tool_frame", "tool_name": "Arm_Tip",
                            "pose": [0] * 6, "payload": 0, "position": [0] * 3}
            elif command == "get_current_work_frame":
                response = {"state": "current_work_frame", "frame_name": "World", "pose": [0] * 6}
            elif command == "get_install_pose":
                response = {"state": "install_pose", "roll": 0, "pitch": 0, "yaw": 0}
            elif command == "get_realtime_push":
                response = {"command": "get_realtime_push", "cycle": 1, "enable": False,
                            "ip": "127.0.0.1", "port": 8089, "force_coordinate": 0}
            elif command == "get_joint_max_speed":
                response = {"state": "joint_max_speed", "joint_max_speed": [180000] * 6}
            elif command == "set_realtime_push":
                response = {"command": "set_realtime_push", "state": True}
            else:
                # CANFD is fire-and-forget on the I-series controller.
                continue
            self.wfile.write(json.dumps(response).encode() + b"\r\n")
            self.wfile.flush()


class LocalServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


@unittest.skipUnless(sys.platform == "win32" and ct.sizeof(ct.c_void_p) == 8,
                     "Bundled SDK supports 64-bit Windows")
class NativeLegacySDKTests(unittest.TestCase):
    def setUp(self):
        self.server = LocalServer(("127.0.0.1", 0), ControllerHandler)
        self.server.requests = queue.Queue()
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.stop_server)
        self.arm = LegacyArm("127.0.0.1", self.server.server_address[1])
        self.addCleanup(self.arm.close)

    def stop_server(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def receive(self, command):
        for _ in range(20):
            message = self.server.requests.get(timeout=2)
            self.assertNotEqual(message["command"], "get_arm_software_info")
            if message["command"] == command:
                return message
        self.fail(f"Missing command {command}")

    def test_real_dll_uses_old_handshake_and_parses_degree_scale(self):
        self.receive("get_arm_software_version")
        self.assertEqual(self.arm.software_info["Product_version"], "RM65-BI")
        code, joints = self.arm.Get_Joint_Degree()
        self.assertEqual(code, 0)
        self.assertEqual(joints, [1, -2, 3, 0, 0, 0])

    def test_real_dll_joint_stream_wire_values_and_unsupported_fields(self):
        self.assertEqual(self.arm.Movej_CANFD([1.25, -2.5, 3, 0, 0, 0], False), 0)
        command = self.receive("movej_canfd")
        self.assertEqual(command["joint"], [1250, -2500, 3000, 0, 0, 0])
        self.assertFalse(command["follow"])
        self.assertNotIn("trajectory_mode", command)
        self.assertNotIn("radio", command)

    def test_real_dll_pose_stream_wire_units(self):
        self.assertEqual(self.arm.Movep_CANFD([0.1, -0.2, 0.3, 0, 0, 0.5], False), 0)
        command = self.receive("movep_canfd")
        self.assertEqual(command["pose"], [100000, -200000, 300000, 0, 0, 500])

    def test_real_dll_udp_period_is_in_five_ms_ticks(self):
        config = {"cycle": 4, "port": 9999, "enable": True,
                  "force_coordinate": -1, "ip": "127.0.0.1"}
        self.assertEqual(self.arm.Set_Realtime_Push(config), 0)
        command = self.receive("set_realtime_push")
        self.assertEqual(command["cycle"], 4)
        self.assertEqual(command["port"], 9999)
        self.assertNotIn("custom", command)


class LegacyLifetimeTests(unittest.TestCase):
    def setUp(self):
        self.lib = MagicMock()
        self.lib.RM_API_Init.return_value = 0
        self.lib.Arm_Socket_Start.return_value = 123
        self.lib.Arm_Socket_State.return_value = 0
        self.state = patch.multiple(LegacyArm, _library=self.lib, _users=0,
                                    _callback_keepalive=None)
        self.state.start()
        self.addCleanup(self.state.stop)

    def connect(self):
        with patch.object(LegacyArm, "Get_Arm_Software_Version",
                          return_value=(0, {"Product_version": "RM65-BI"})):
            return LegacyArm("127.0.0.1")

    def test_last_connection_uninitializes_sdk_and_close_is_idempotent(self):
        first, second = self.connect(), self.connect()
        self.lib.RM_API_Init.assert_called_once()
        first.close()
        self.lib.RM_API_UnInit.assert_not_called()
        second.close()
        second.close()
        self.assertEqual(self.lib.Arm_Socket_Close.call_count, 2)
        self.lib.RM_API_UnInit.assert_called_once()
        self.assertEqual(LegacyArm._users, 0)

    def test_failed_connection_releases_sdk(self):
        self.lib.Arm_Socket_State.return_value = 5
        with self.assertRaises(ConnectionError):
            self.connect()
        self.lib.RM_API_UnInit.assert_called_once()
        self.assertEqual(LegacyArm._users, 0)

    def test_failed_version_query_disconnects_and_releases_sdk(self):
        with patch.object(LegacyArm, "Get_Arm_Software_Version", return_value=(7, {})):
            with self.assertRaises(ConnectionError):
                LegacyArm("127.0.0.1")
        self.lib.Arm_Socket_Close.assert_called_once_with(123)
        self.lib.RM_API_UnInit.assert_called_once()
