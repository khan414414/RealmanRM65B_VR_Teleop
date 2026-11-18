import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from Realman_Control.legacy_sdk import RobotStatus
from Realman_Control.rm65_state_logger import RM65StateLogger


class LegacyLoggerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.logger = RM65StateLogger(
            robot_ip="192.168.1.18", robot_port=8080, local_ip="192.168.1.100",
            udp_port=8089, cycle_ms=20, output_dir=Path(self.temp.name), queue_size=100,
        )
        self.arm = MagicMock()
        self.original = {"cycle": 1, "port": 8089, "ip": "192.168.1.10",
                         "force_coordinate": 0, "enable": True}
        self.arm.Get_Realtime_Push.return_value = 0, self.original.copy()
        self.arm.Set_Realtime_Push.return_value = 0
        self.robot = MagicMock(arm=self.arm)

    def connect(self):
        with patch("Realman_Control.rm65_state_logger.RM65B", return_value=self.robot):
            self.logger.connect_and_start()

    def test_legacy_udp_configuration_uses_5_ms_ticks_and_restores_original(self):
        self.connect()
        configured = self.arm.Set_Realtime_Push.call_args.args[0]
        self.assertEqual(configured["cycle"], 4)
        self.assertEqual(configured["ip"], "192.168.1.100")
        self.logger.stop_receiving()
        self.arm.Set_Realtime_Push.assert_called_with(self.original)
        self.logger.close()
        self.robot.close.assert_called_once()
        self.robot.stop_script.assert_not_called()

    def test_failed_config_read_does_not_change_robot_settings(self):
        self.arm.Get_Realtime_Push.return_value = 7, {}
        with self.assertRaises(RuntimeError):
            self.connect()
        self.arm.Set_Realtime_Push.assert_not_called()

    def test_failed_set_reply_still_restores_original_configuration(self):
        self.arm.Set_Realtime_Push.side_effect = [7, 0]
        with self.assertRaises(RuntimeError):
            self.connect()
        self.logger.stop_receiving()
        self.arm.Set_Realtime_Push.assert_called_with(self.original)

    def test_callback_copies_legacy_layout_and_marks_missing_status_unknown(self):
        self.logger._accept_callbacks = True
        state = RobotStatus()
        state.joint_status.joint_position[0] = 90.0
        state.waypoint.position.x = 0.25
        state.waypoint.quaternion.w = 1.0
        self.logger._on_realtime_state(state)
        sample = self.logger._queue.get_nowait()
        self.assertAlmostEqual(sample.joint_rad[0], math.pi / 2)
        self.assertEqual(sample.tcp_position_base_m, (0.25, 0, 0))
        self.assertEqual(sample.tcp_quaternion_base_xyzw, (0, 0, 0, 1))
        self.assertEqual(sample.arm_current_status, -1)
