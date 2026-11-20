import math
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

from Realman_Control.realman_control import RealmanControl
from Realman_Control.legacy_sdk import Frame, Pose
from teleop.official_position_ik import OfficialPositionIK
from teleop.rm65_decoupled_ik import (
    IKOptions, RM65_NOMINAL_POSTURE_RAD, transform_from_xyz_euler,
)


REACHABILITY_SEED_DEG = (152.425, 31.729, 7.985, 70.519, -56.344, -306.180)
REACHABILITY_LIMITS_DEG = ((-183, 183), (-135, 135), (-140, 140),
                           (-360, 360), (-133, 133), (-365, 365))
REACHABILITY_TARGET = (0.30861222743988037, -0.16676518321037292, 0.6949556469917297)


def offline_robot(tool=(0, 0, 0, 0, 0, 0)):
    """Create only an Algo owner: no constructor, socket, or motion methods."""
    robot = object.__new__(RealmanControl)
    robot._closed = False
    robot._algo = None
    robot._tool_frame = Frame(name=b"TestTCP", pose=Pose.from_euler(tool))
    robot._joint_limits = RealmanControl.JOINT_LIMITS
    return robot


class OfficialWrapperTests(unittest.TestCase):
    def test_units_and_official_failure_does_not_consume_undefined_output(self):
        robot = offline_robot()
        robot._algo = Mock()
        robot._algo.rm_algo_inverse_kinematics.return_value = (0, [0, 90, 0, 0, 0, -180])
        pose = [0.2, 0.1, 0.4, 0, 0.2, 0]
        with patch("Realman_Control.realman_control._rm") as sdk:
            code, joints = robot.official_inverse_kinematics([0, math.pi / 2, 0, 0, 0, 0], pose)
            sdk.rm_inverse_kinematics_params_t.assert_called_once_with([0, 90, 0, 0, 0, 0], pose, 1)
        self.assertEqual(code, 0)
        self.assertAlmostEqual(joints[1], math.pi / 2)
        robot._algo.rm_algo_inverse_kinematics.return_value = (1, [float("nan")] * 6)
        with patch("Realman_Control.realman_control._rm"):
            self.assertEqual(robot.official_inverse_kinematics([0] * 6, pose), (1, None))

    def test_no_local_solver_fallback_on_sdk_failure(self):
        backend = Mock()
        backend.official_forward_kinematics.side_effect = lambda q: [*q[:3], 0, 0, q[5]]
        backend.official_inverse_kinematics.return_value = (1, None)
        solver = OfficialPositionIK(backend)
        with patch("teleop.rm65_decoupled_ik.RM65DecoupledIK.solve", side_effect=AssertionError("fallback")):
            result = solver.solve([0.01, 0, 0], np.zeros(6), wrist_joint_6_target_rad=0.05)
        self.assertFalse(result.converged)
        np.testing.assert_array_equal(result.joints_rad[:5], 0)
        self.assertAlmostEqual(result.joints_rad[5], 0.05)
        self.assertLessEqual(backend.official_inverse_kinematics.call_count, solver.MAX_SDK_CALLS)

    def test_production_backend_must_provide_official_api(self):
        with self.assertRaisesRegex(ValueError, "official IK backend requires"):
            OfficialPositionIK(object())

    def test_sdk_success_is_not_position_success_after_restoring_j6(self):
        backend = Mock()
        backend.official_forward_kinematics.side_effect = (
            lambda q: [q[0] + 0.1 * q[5], q[1], q[2], q[3] + q[5], q[4], 0])

        def full_pose_solution(seed, pose):
            # The SDK redistributes rotation between J4 and J6. Its original
            # full-pose solution is exact, but changing J6 displaces the TCP.
            return 0, [pose[0] - 0.05, pose[1], pose[2], pose[3] - 0.5, pose[4], 0.5]

        backend.official_inverse_kinematics.side_effect = full_pose_solution
        result = OfficialPositionIK(backend).solve(
            [0.02, 0, 0], np.zeros(6), wrist_joint_6_target_rad=0.05)
        self.assertFalse(result.converged)
        self.assertGreater(result.position_error_m, 0.001)
        self.assertAlmostEqual(result.joints_rad[5], 0.05)

    def test_orientation_search_rejects_unsafe_candidate_and_keeps_sdk_budget(self):
        backend = Mock()
        backend.official_forward_kinematics.side_effect = lambda q: [*q[:3], q[3], q[4], q[5]]

        def sdk(seed, pose):
            # Two postures reach the same XYZ. The first one is prohibited by
            # the tracker safety callback; a nearby orientation has a safe one.
            return 0, [*pose[:3], 0.1 if pose[3] >= 0 else -0.01, 0, pose[5]]

        backend.official_inverse_kinematics.side_effect = sdk
        solver = OfficialPositionIK(backend)

        def validate(q):
            if q[3] > 0:
                raise ValueError("unsafe posture")

        solver.candidate_validator = validate
        result = solver.solve([0.01, 0, 0], np.zeros(6))
        self.assertTrue(result.converged)
        self.assertLess(result.joints_rad[3], 0)
        self.assertLessEqual(result.iterations, solver.MAX_SDK_CALLS)

    def test_soft_time_budget_returns_without_consuming_failed_sdk_output(self):
        backend = Mock()
        backend.official_forward_kinematics.side_effect = lambda q: [*q[:3], 0, 0, 0]
        backend.official_inverse_kinematics.return_value = (1, None)
        solver = OfficialPositionIK(backend)
        solver.max_solve_time_s = 0.006
        with patch("teleop.official_position_ik.time.perf_counter", side_effect=[0, 0] + [0.1] * 100):
            result = solver.solve([0.01, 0, 0], np.zeros(6))
        self.assertEqual(result.iterations, 1)
        self.assertFalse(result.converged)
        np.testing.assert_array_equal(result.joints_rad, 0)


@unittest.skipUnless(sys.platform == "win32", "official DLL integration requires Windows")
class OfficialDLLTests(unittest.TestCase):
    def test_reaches_xyz_when_start_orientation_has_no_official_full_pose_solution(self):
        from teleop.vr_tracker import VRTracker
        robot = offline_robot()
        robot._joint_limits = tuple(map(tuple, np.radians(REACHABILITY_LIMITS_DEG)))
        seed = np.radians(REACHABILITY_SEED_DEG)
        target = np.array(REACHABILITY_TARGET)
        original_pose = np.asarray(robot.official_forward_kinematics(seed))
        fixed_orientation_pose = original_pose.copy()
        fixed_orientation_pose[:3] = target
        self.assertNotEqual(robot.official_inverse_kinematics(seed, fixed_orientation_pose)[0], 0)

        robot._simulated = seed.copy()
        robot.get_joint_values = lambda: robot._simulated.copy()
        robot.get_joint_limits = lambda: robot._joint_limits
        robot.get_active_tool_frame_pose_euler = lambda: [0] * 6
        commands = [seed.copy()]

        def servo(q, **kwargs):
            robot._simulated = np.asarray(q).copy()
            commands.append(robot._simulated.copy())
            return 0

        robot.servo_joint = servo
        clock = [0.0]
        with patch("teleop.vr_tracker.time.monotonic", lambda: clock[0]):
            tracker = VRTracker(robot, None, rate=0.01, smooth_step=0.15, wrist_smooth_step=0.15)
            tracker.ik.max_solve_time_s = None  # Deterministic numerical regression, not a timing benchmark.
            tracker.resume([0, 0, 0, 0, 0, 0, 1])
            hand = (target - tracker.target_position) / 1.5
            for tick in range(1, 301):
                clock[0] = tick * 0.01
                tracker.track([*hand, 0, 0, 0, 1])
        self.assertLess(np.linalg.norm(tracker.target_position - target), 2e-5)
        self.assertGreater(abs(tracker.command_joints[4] - seed[4]), math.radians(1))
        np.testing.assert_allclose(np.asarray(commands)[:, 5], seed[5], atol=1e-12)
        self.assertLessEqual(np.max(np.abs(np.diff(commands, axis=0))), math.radians(1.8) + 1e-12)
        self.assertLess(np.linalg.norm(tracker.target_position - original_pose[:3]), 0.25)
        initial_z = transform_from_xyz_euler(original_pose)[:3, 2]
        final_z = tracker.ik.forward_kinematics(tracker.command_joints)[:3, 2]
        self.assertGreater(np.linalg.norm(final_z - initial_z), 0.02)

    def test_position_and_wrist_constraints_with_on_axis_and_off_axis_tcp(self):
        for tool in ((0, 0, 0, 0, 0, 0), (0.03, 0.02, 0.10, 0, 0, 0)):
            with self.subTest(tool=tool):
                robot = offline_robot(tool)
                seed = np.asarray(RM65_NOMINAL_POSTURE_RAD)
                solver = OfficialPositionIK(robot, tcp_transform=transform_from_xyz_euler(tool),
                    options=IKOptions(position_tolerance_m=0.0002,
                                      max_solution_delta_rad=math.radians(1.8),
                                      max_wrist_step_rad=math.radians(1.8)))
                target = np.asarray(robot.official_forward_kinematics(seed)[:3]) + (0.001, -0.001, 0.001)
                result = solver.solve(target, seed, wrist_joint_6_target_rad=0.02)
                self.assertTrue(result.converged, result)
                self.assertAlmostEqual(result.joints_rad[5], 0.02, places=12)
                actual = robot.official_forward_kinematics(result.joints_rad)
                self.assertLess(np.linalg.norm(np.asarray(actual[:3]) - target), 0.0002)
                initial_rotation = solver.forward_kinematics(seed)[:3, :3]
                final_rotation = solver.forward_kinematics(result.joints_rad)[:3, :3]
                self.assertGreater(np.linalg.norm(final_rotation - initial_rotation), 0.001)
                self.assertLessEqual(np.max(np.abs(result.joints_rad - seed)), math.radians(1.8) + 1e-12)

    def test_tracker_uses_official_default_without_robot_connection(self):
        from teleop.vr_tracker import VRTracker, TeleoperationSafetyWarning
        robot = offline_robot()
        seed = np.asarray(RM65_NOMINAL_POSTURE_RAD)
        robot.get_joint_values = Mock(return_value=seed.copy())
        robot.get_joint_limits = Mock(return_value=robot._joint_limits)
        robot.get_active_tool_frame_pose_euler = Mock(return_value=[0] * 6)
        robot.validate_joint_target = Mock(return_value=0)
        robot.servo_joint = Mock(return_value=0)
        tracker = VRTracker(robot, Mock(), rate=0.01, smooth_step=1)
        tracker.ik.max_solve_time_s = None
        self.assertIsInstance(tracker.ik, OfficialPositionIK)
        tracker.resume([0, 0, 0, 0, 0, 0, 1])
        tracker.track([0.001, 0, 0, 0, 0, 0, 1])
        robot.servo_joint.assert_called_once()
        self.assertGreater(np.linalg.norm(tracker.command_joints[:5] - seed[:5]), 0)
        held_position = tracker.target_position.copy()
        robot.validate_joint_target.side_effect = ValueError("near a shoulder singularity")
        outward = [0.04, 0, 0, 0, 0, 0, 1]
        with self.assertRaises(TeleoperationSafetyWarning):
            tracker.track(outward)
        self.assertFalse(tracker.paused)
        self.assertEqual(robot.servo_joint.call_count, 1)
        np.testing.assert_array_equal(tracker.requested_position, held_position)
        robot.validate_joint_target.side_effect = None
        outward[0] -= 0.002
        tracker.track(outward)
        self.assertEqual(robot.servo_joint.call_count, 2)
        self.assertLess(tracker.target_position[0], held_position[0])


if __name__ == "__main__":
    unittest.main()
