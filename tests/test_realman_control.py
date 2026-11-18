import math
import unittest
from unittest.mock import MagicMock, patch

import Realman_Control.realman_control as control_module
from Realman_Control.realman_control import RealmanControl, RealManError
from Realman_Control.legacy_sdk import Pose, Frame


class FakeArm:
    def Get_Joint_Min_Pos(self):
        return 0, [-170.0, -120.0, -125.0, -170.0, -118.0, -340.0]

    def Get_Joint_Max_Pos(self):
        return 0, [170.0, 120.0, 125.0, 170.0, 118.0, 340.0]


class ControllerJointLimitTests(unittest.TestCase):
    def setUp(self):
        self.robot = object.__new__(RealmanControl)
        self.robot._closed = False
        self.robot._arm = FakeArm()
        self.robot._joint_limits = tuple(RealmanControl.JOINT_LIMITS)
        self.robot._algo = None

    def test_controller_limits_are_converted_to_radians_and_cached(self):
        limits = self.robot.get_joint_limits()

        self.assertAlmostEqual(limits[0][0], math.radians(-170.0))
        self.assertAlmostEqual(limits[5][1], math.radians(340.0))
        self.assertEqual(self.robot._joint_limits, limits)

    def test_motion_validation_uses_cached_controller_limits(self):
        self.robot.get_joint_limits()

        with self.assertRaisesRegex(ValueError, "joint 1"):
            self.robot._joint_targets(
                [math.radians(175.0), 0.0, 0.0, 0.0, 0.0, 0.0]
            )


class LegacyControlTests(unittest.TestCase):
    def setUp(self):
        self.arm = MagicMock()
        self.arm.Get_Current_Tool_Frame.return_value = (0, Frame(name=b"Arm_Tip", pose=Pose()))
        for name in ("Movej_CANFD", "Movep_CANFD", "Movej_Cmd", "Movel_Cmd",
                     "Move_Slow_Stop_Cmd", "close", "Manual_Set_Tool_Frame",
                     "Change_Tool_Frame"):
            getattr(self.arm, name).return_value = 0
        self.factory = patch("Realman_Control.realman_control.LegacyArm", return_value=self.arm)
        self.factory.start()
        self.addCleanup(self.factory.stop)
        self.robot = RealmanControl()

    def test_constructor_never_creates_an_api2_robot_connection(self):
        with patch("Realman_Control.realman_control._rm") as modern:
            robot = RealmanControl(port=9000, timeout_ms=250)
        self.assertIs(robot.arm, self.arm)
        modern.RoboticArm.assert_not_called()
        modern.Algo.assert_not_called()

    def test_joint_stream_converts_radians_to_degrees(self):
        self.robot.servo_joint([0, math.pi / 2, 0, 0, 0, -math.pi])
        args = self.arm.Movej_CANFD.call_args.args
        self.assertEqual(args[0], [0, 90, 0, 0, 0, -180])
        self.assertFalse(args[1])

    def test_tool_stream_converts_rotation_vector_to_euler(self):
        self.robot.servo_tool([0.1, 0.2, 0.3, 0, 0, math.pi / 2])
        pose, follow = self.arm.Movep_CANFD.call_args.args
        self.assertEqual(pose[:3], [0.1, 0.2, 0.3])
        self.assertAlmostEqual(pose[5], math.pi / 2)
        self.assertFalse(follow)

    def test_unsupported_stream_filters_fail_before_sending(self):
        for options in ({"trajectory_mode": 1}, {"radio": 50}):
            with self.assertRaises(NotImplementedError):
                self.robot.servo_joint([0] * 6, **options)
        self.arm.Movej_CANFD.assert_not_called()

    def test_nan_or_out_of_range_joint_never_reaches_sdk(self):
        for value in (float("nan"), 100.0):
            with self.assertRaises(ValueError):
                self.robot.servo_joint([value, 0, 0, 0, 0, 0])
        self.arm.Movej_CANFD.assert_not_called()

    def test_joint_and_tool_moves_preserve_connect_and_block_argument_order(self):
        self.robot.move_joint([0] * 6, 12, asynchronous=True)
        self.arm.Movej_Cmd.assert_called_once_with([0] * 6, 12, 0, 0, 0)
        self.robot.move_tool([0] * 6, 15)
        self.arm.Movel_Cmd.assert_called_once_with([0] * 6, 15, 0, 0, 1)

    def test_connected_joint_trajectory_only_blocks_on_last_point(self):
        self.robot.move_joint_trajectory([[0] * 6, [0.1] * 6])
        calls = self.arm.Movej_Cmd.call_args_list
        self.assertEqual(calls[0].args[-3:], (0, 1, 0))
        self.assertEqual(calls[1].args[-3:], (0, 0, 1))

    def test_nonzero_blend_and_velocity_mode_fail_explicitly(self):
        with self.assertRaises(NotImplementedError):
            self.robot.move_joint_trajectory([[0] * 6], blend_percent=10)
        with self.assertRaises(NotImplementedError):
            self.robot.speed_tool([0] * 6)
        self.arm.Movej_Cmd.assert_not_called()

    def test_joint_state_and_tcp_state_conversion(self):
        self.arm.Get_Joint_Degree.return_value = (0, [90] * 6)
        self.arm.Get_Current_Arm_State.return_value = (0, [0] * 6, [0.1, 0.2, 0.3, 0, 0, math.pi / 2], 0, 0)
        for value in self.robot.get_joint_values():
            self.assertAlmostEqual(value, math.pi / 2)
        pose = self.robot.get_tool_pose()
        self.assertEqual(pose[:3], [0.1, 0.2, 0.3])
        self.assertAlmostEqual(pose[5], math.pi / 2)

    def test_tool_frame_uses_legacy_structure(self):
        frame = Frame(pose=Pose.from_euler([0.1, 0.2, 0.3, 0, 0, 0]))
        self.arm.Get_Current_Tool_Frame.return_value = 0, frame
        self.assertAlmostEqual(self.robot.get_active_tool_frame_pose_euler()[2], 0.3)

    def test_numeric_sdk_errors_are_preserved(self):
        self.arm.Movej_CANFD.return_value = 7
        with self.assertRaises(RealManError) as raised:
            self.robot.servo_joint([0] * 6)
        self.assertEqual(raised.exception.code, 7)
        self.assertEqual(raised.exception.operation, "Movej_CANFD")

    def test_close_is_idempotent_and_sends_no_stop(self):
        self.robot.close()
        self.robot.close()
        self.arm.close.assert_called_once()
        self.arm.Move_Slow_Stop_Cmd.assert_not_called()
        with self.assertRaises(RuntimeError):
            self.robot.get_joint_values()

    def test_cleanup_disconnects_even_when_stop_fails(self):
        self.arm.Move_Slow_Stop_Cmd.return_value = 7
        with self.assertRaises(RealManError):
            self.robot.stop_script()
        self.arm.close.assert_called_once()

    def test_offline_algo_preserves_singularity_and_collision_rejections(self):
        algo = MagicMock()
        self.robot._algo = algo
        algo.rm_algo_universal_singularity_analyse.return_value = -1
        algo.rm_algo_kin_robot_singularity_analyse.return_value = (-1, 0.0245)
        with self.assertRaisesRegex(ValueError, "shoulder singularity"):
            self.robot.validate_joint_target([0] * 6)
        algo.rm_algo_universal_singularity_analyse.return_value = 0
        algo.rm_algo_safety_robot_self_collision_detection.return_value = 1
        with self.assertRaisesRegex(ValueError, "self-colliding"):
            self.robot.validate_joint_target([0] * 6)

    def test_safety_initialization_uses_offline_algo_only(self):
        with patch("Realman_Control.realman_control._rm") as modern:
            self.robot._safety_algo()
        modern.Algo.assert_called_once()
        modern.RoboticArm.assert_not_called()

    def test_missing_safety_library_fails_closed(self):
        with patch("Realman_Control.realman_control._rm", None):
            with self.assertRaisesRegex(RuntimeError, "offline Algo"):
                self.robot.validate_joint_target([0] * 6)
        self.arm.Movej_CANFD.assert_not_called()

    @unittest.skipIf(control_module._rm is None, "Offline safety SDK is not installed")
    def test_real_offline_algo_accepts_nominal_pose_and_rejects_singularity(self):
        nominal = [math.radians(v) for v in (0, -75, 60, 0, 60, 0)]
        self.assertEqual(self.robot.validate_joint_target(nominal), 0)
        with self.assertRaisesRegex(ValueError, "singularity"):
            self.robot.validate_joint_target([0] * 6)
        self.arm.Movej_CANFD.assert_not_called()


if __name__ == "__main__":
    unittest.main()
