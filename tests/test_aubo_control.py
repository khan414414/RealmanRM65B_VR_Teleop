import math
import runpy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from Aubo_Control.aubo_control import (
    AuboControl,
    AuboControlError,
    AuboModelError,
)


class FakeConfig:
    def __init__(self, robot_type="AUBO", subtype="i7"):
        self.robot_type = robot_type
        self.subtype = subtype
        self.minimum = [-2.9, -2.8, -2.7, -2.6, -2.5, -6.0]
        self.maximum = [2.9, 2.8, 2.7, 2.6, 2.5, 6.0]
        self.max_speeds = [2.0, 2.1, 2.2, 2.3, 2.4, 2.5]
        self.max_accelerations = [3.0, 3.1, 3.2, 3.3, 3.4, 3.5]
        self.tcp = [0.0, 0.0, 0.2, 0.0, 0.0, 0.1]
        self.set_tcp_calls = []

    def getRobotType(self):
        return self.robot_type

    def getRobotSubType(self):
        return self.subtype

    def getJointMinPositions(self):
        return list(self.minimum)

    def getJointMaxPositions(self):
        return list(self.maximum)

    def getJointMaxSpeeds(self):
        return list(self.max_speeds)

    def getJointMaxAccelerations(self):
        return list(self.max_accelerations)

    def getTcpOffset(self):
        return list(self.tcp)

    def setTcpOffset(self, tcp):
        self.set_tcp_calls.append(list(tcp))
        self.tcp = list(tcp)
        return 0


class FakeState:
    def __init__(self):
        self.joints = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        self.speeds = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]
        self.pose = [0.3, -0.2, 0.4, 0.1, -0.2, 0.3]

    def getJointPositions(self):
        return list(self.joints)

    def getJointSpeeds(self):
        return list(self.speeds)

    def getTcpPose(self):
        return list(self.pose)

    def isSteady(self):
        return True

    def isCollisionOccurred(self):
        return False

    def isWithinSafetyLimits(self):
        return True


class FakeMotion:
    def __init__(self):
        self.servo_mode = False
        self.enable_transition = True
        self.set_servo_calls = []
        self.move_calls = []
        self.servo_calls = []
        self.stop_calls = []
        self.servo_results = []

    def moveJoint(self, *args):
        self.move_calls.append(args)
        return 0

    def servoJoint(self, *args):
        self.servo_calls.append(args)
        if self.servo_results:
            return self.servo_results.pop(0)
        return 0

    def setServoMode(self, enabled):
        self.set_servo_calls.append(enabled)
        if self.enable_transition:
            self.servo_mode = enabled
        return 0

    def isServoModeEnabled(self):
        return self.servo_mode

    def stopJoint(self, acceleration):
        self.stop_calls.append(acceleration)
        return 0


class FakeRobot:
    def __init__(self, config=None):
        self.config = config or FakeConfig()
        self.state = FakeState()
        self.motion = FakeMotion()

    def getRobotConfig(self):
        return self.config

    def getRobotState(self):
        return self.state

    def getMotionControl(self):
        return self.motion


class FakeRpcClient:
    def __init__(self, robots=None):
        self.robots = robots or {"robot-one": FakeRobot()}
        self.connect_calls = []
        self.login_calls = []
        self.interface_calls = []
        self.logout_calls = 0
        self.disconnect_calls = 0

    def connect(self, host, port):
        self.connect_calls.append((host, port))
        return 0

    def login(self, username, password):
        self.login_calls.append((username, password))
        return 0

    def getRobotNames(self):
        return list(self.robots)

    def getRobotInterface(self, name):
        self.interface_calls.append(name)
        return self.robots[name]

    def logout(self):
        self.logout_calls += 1
        return 0

    def disconnect(self):
        self.disconnect_calls += 1
        return 0


def fake_sdk(client):
    return SimpleNamespace(RpcClient=lambda: client)


class AuboControlTests(unittest.TestCase):
    def make_control(self, **kwargs):
        client = kwargs.pop("client", FakeRpcClient())
        kwargs.setdefault("servo_retry_interval", 0.0)
        control = AuboControl(sdk_module=fake_sdk(client), **kwargs)
        return control, client, client.robots[control.robot_name]

    def test_connects_logs_in_and_selects_first_robot_by_default(self):
        control, client, _ = self.make_control()
        self.addCleanup(control.close)

        self.assertEqual(client.connect_calls, [("192.168.1.100", 30004)])
        self.assertEqual(client.login_calls, [("aubo", "123456")])
        self.assertEqual(client.interface_calls, ["robot-one"])
        self.assertEqual(control.robot_name, "robot-one")

    def test_selects_requested_robot(self):
        client = FakeRpcClient(
            {"left": FakeRobot(), "right": FakeRobot()}
        )
        control, _, _ = self.make_control(client=client, robot_name="right")
        self.addCleanup(control.close)

        self.assertEqual(control.robot_name, "right")
        self.assertEqual(client.interface_calls, ["right"])

    def test_model_check_rejects_non_i7_and_can_be_disabled(self):
        wrong_client = FakeRpcClient(
            {"robot-one": FakeRobot(FakeConfig(subtype="i10"))}
        )
        with self.assertRaises(AuboModelError):
            self.make_control(client=wrong_client)
        self.assertEqual(wrong_client.logout_calls, 1)
        self.assertEqual(wrong_client.disconnect_calls, 1)

        disabled_client = FakeRpcClient(
            {"robot-one": FakeRobot(FakeConfig(subtype="i10"))}
        )
        control, _, _ = self.make_control(
            client=disabled_client, expected_model=None
        )
        self.addCleanup(control.close)
        self.assertIsNone(control.robot_type)

    def test_state_pose_tcp_and_limits_preserve_si_values(self):
        control, _, robot = self.make_control()
        self.addCleanup(control.close)

        self.assertEqual(control.get_joint_values(), robot.state.joints)
        self.assertEqual(control.get_joint_speed(), robot.state.speeds)
        self.assertEqual(control.get_tool_pose(), robot.state.pose)
        self.assertEqual(control.get_tcp_offset(), robot.config.tcp)
        self.assertEqual(
            control.get_joint_limits()[0],
            (robot.config.minimum[0], robot.config.maximum[0]),
        )

        tcp = [0.01, 0.02, 0.3, -0.1, 0.2, -0.3]
        self.assertEqual(control.set_tool_pose(tcp), 0)
        self.assertEqual(robot.config.set_tcp_calls, [tcp])

    def test_controller_position_limits_are_authoritative(self):
        control, _, robot = self.make_control()
        self.addCleanup(control.close)
        robot.config.maximum[0] = 0.5
        control.get_joint_limits()

        with self.assertRaisesRegex(ValueError, "joint 1"):
            control.move_joint([0.6, 0, 0, 0, 0, 0])
        self.assertEqual(robot.motion.move_calls, [])

    def test_move_joint_maps_a_v_blend_and_duration_in_sdk_order(self):
        control, _, robot = self.make_control()
        self.addCleanup(control.close)
        target = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]

        control.move_joint(
            target, speed=0.4, acceleration=0.7, blend=0.02, duration=1.5
        )

        self.assertEqual(
            robot.motion.move_calls,
            [(target, 0.7, 0.4, 0.02, 1.5)],
        )

    def test_move_parameters_cannot_exceed_controller_limits(self):
        control, _, robot = self.make_control()
        self.addCleanup(control.close)

        with self.assertRaisesRegex(ValueError, "speed"):
            control.move_joint([0] * 6, speed=2.01)
        with self.assertRaisesRegex(ValueError, "acceleration"):
            control.move_joint([0] * 6, acceleration=3.01)
        self.assertEqual(robot.motion.move_calls, [])

    def test_servo_mode_is_explicit_and_servo_defaults_match_arcs(self):
        control, _, robot = self.make_control()
        self.addCleanup(control.close)

        with self.assertRaisesRegex(RuntimeError, "start_servo_mode"):
            control.servo_joint([0] * 6, time=0.008)
        control.start_servo_mode()
        control.servo_joint([0] * 6, time=0.008)

        self.assertEqual(robot.motion.set_servo_calls, [True])
        self.assertEqual(
            robot.motion.servo_calls,
            [([0.0] * 6, 0.2, 0.2, 0.008, 0.1, 200)],
        )
        control.stop_servo_mode()
        self.assertFalse(robot.motion.servo_mode)

    def test_queue_full_is_retried_only_up_to_configured_limit(self):
        control, _, robot = self.make_control(servo_queue_retries=2)
        self.addCleanup(control.close)
        control.start_servo_mode()
        robot.motion.servo_results = [2, 2, 0]

        self.assertEqual(control.servo_joint([0] * 6), 0)
        self.assertEqual(len(robot.motion.servo_calls), 3)

        robot.motion.servo_calls.clear()
        robot.motion.servo_results = [2, 2, 2]
        with self.assertRaises(AuboControlError) as context:
            control.servo_joint([0] * 6)
        self.assertEqual(context.exception.code, 2)
        self.assertEqual(len(robot.motion.servo_calls), 3)

    def test_servo_mode_wait_has_a_timeout(self):
        control, _, robot = self.make_control()
        self.addCleanup(control.close)
        robot.motion.enable_transition = False

        with self.assertRaises(TimeoutError):
            control.start_servo_mode(timeout=0.0)

    def test_stop_uses_stop_joint_and_close_is_idempotent(self):
        control, client, robot = self.make_control()
        control.start_servo_mode()

        self.assertEqual(control.stop(a=1.25), 0)
        self.assertEqual(robot.motion.stop_calls, [1.25])
        self.assertEqual(control.close(), 0)
        self.assertEqual(control.close(), 0)
        self.assertEqual(robot.motion.set_servo_calls, [True, False])
        self.assertEqual(client.logout_calls, 1)
        self.assertEqual(client.disconnect_calls, 1)

    def test_stop_script_stops_and_leaves_servo_without_disconnect(self):
        control, client, robot = self.make_control()
        self.addCleanup(control.close)
        control.start_servo_mode()

        self.assertEqual(control.stop_script(), 0)
        self.assertEqual(robot.motion.stop_calls, [2.0])
        self.assertFalse(robot.motion.servo_mode)
        self.assertEqual(client.disconnect_calls, 0)

    def test_vectors_reject_wrong_size_and_non_finite_values(self):
        control, _, _ = self.make_control()
        self.addCleanup(control.close)

        with self.assertRaisesRegex(ValueError, "exactly 6"):
            control.set_tool_pose([0] * 5)
        with self.assertRaisesRegex(ValueError, "finite"):
            control.set_tool_pose([0, 0, 0, 0, math.nan, 0])

    def test_import_does_not_require_pyaubo_sdk(self):
        sys.modules.pop("pyaubo_sdk", None)
        with mock.patch("importlib.import_module") as importer:
            namespace = runpy.run_path(
                str(
                    Path(__file__).parents[1]
                    / "Aubo_Control"
                    / "aubo_control.py"
                )
            )

        importer.assert_not_called()
        self.assertIn("AuboControl", namespace)


if __name__ == "__main__":
    unittest.main()
