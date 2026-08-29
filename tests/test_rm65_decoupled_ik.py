import math
import unittest

import numpy as np

from teleop.rm65_decoupled_ik import (
    ContinuousControllerTwist,
    ControllerTwistError,
    IKOptions,
    RM65DecoupledIK,
    RM65_JOINT_LIMITS_RAD,
    RM65_NOMINAL_POSTURE_RAD,
    continuous_wrist_roll_target,
    forward_kinematics,
    geometric_jacobian,
    position_jacobian,
)


class RM65KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.nominal = np.asarray(RM65_NOMINAL_POSTURE_RAD)

    def test_nominal_posture_has_full_geometric_rank(self):
        jacobian = geometric_jacobian(self.nominal)
        self.assertEqual(np.linalg.matrix_rank(jacobian, tol=1e-9), 6)
        self.assertEqual(
            np.linalg.matrix_rank(jacobian[:3, :5], tol=1e-9), 3
        )

    def test_position_jacobian_matches_central_difference(self):
        joints = np.radians((15.0, -65.0, 72.0, 18.0, 55.0, -30.0))
        analytic = position_jacobian(joints)
        numeric = np.empty((3, 6))
        step = 1.0e-7
        for index in range(6):
            positive = joints.copy()
            negative = joints.copy()
            positive[index] += step
            negative[index] -= step
            numeric[:, index] = (
                forward_kinematics(positive)[:3, 3]
                - forward_kinematics(negative)[:3, 3]
            ) / (2.0 * step)
        np.testing.assert_allclose(analytic, numeric, atol=2e-8, rtol=2e-7)

    def test_j6_does_not_move_urdf_link6_origin(self):
        before = forward_kinematics(self.nominal)[:3, 3]
        rolled = self.nominal.copy()
        rolled[5] += math.radians(90.0)
        after = forward_kinematics(rolled)[:3, 3]
        np.testing.assert_allclose(before, after, atol=1e-12)
        np.testing.assert_allclose(
            position_jacobian(self.nominal)[:, 5], 0.0, atol=1e-12
        )

    def test_off_axis_tcp_reveals_j6_position_coupling(self):
        tcp = np.eye(4)
        tcp[:3, 3] = (0.03, 0.02, 0.10)
        j6_column = position_jacobian(self.nominal, tcp)[:, 5]
        self.assertGreater(np.linalg.norm(j6_column), 0.02)


class RM65DecoupledIKTests(unittest.TestCase):
    def setUp(self):
        self.seed = np.asarray(RM65_NOMINAL_POSTURE_RAD)
        self.solver = RM65DecoupledIK(
            options=IKOptions(
                max_solution_delta_rad=math.radians(20.0),
                max_wrist_step_rad=math.radians(10.0),
            )
        )

    def test_local_position_target_converges_while_j6_is_independent(self):
        target_configuration = self.seed + np.radians(
            (3.0, -4.0, 5.0, 2.0, -3.0, 0.0)
        )
        target_position = forward_kinematics(target_configuration)[:3, 3]
        wrist_target = math.radians(7.0)
        result = self.solver.solve(
            target_position,
            self.seed,
            wrist_joint_6_target_rad=wrist_target,
            posture_reference_rad=RM65_NOMINAL_POSTURE_RAD,
        )
        self.assertTrue(result.converged, result)
        self.assertLess(result.position_error_m, 2.0e-5)
        self.assertAlmostEqual(result.joints_rad[5], wrist_target, places=12)
        self.assertTrue(result.wrist_target_reached)

    def test_solver_never_exceeds_limits_or_local_trust_region(self):
        far_configuration = np.radians((80.0, -30.0, 100.0, 50.0, 80.0, 0.0))
        far_position = forward_kinematics(far_configuration)[:3, 3]
        result = self.solver.solve(far_position, self.seed)
        limits = np.asarray(RM65_JOINT_LIMITS_RAD)
        self.assertTrue(np.all(result.joints_rad >= limits[:, 0]))
        self.assertTrue(np.all(result.joints_rad <= limits[:, 1]))
        self.assertLessEqual(
            np.max(np.abs(result.joints_rad[:5] - self.seed[:5])),
            math.radians(20.0) + 1e-12,
        )

    def test_custom_controller_limits_are_authoritative(self):
        limits = np.asarray(RM65_JOINT_LIMITS_RAD, dtype=np.float64)
        limits[0] = (-0.1, 0.1)
        solver = RM65DecoupledIK(joint_limits_rad=limits)
        outside_seed = self.seed.copy()
        outside_seed[0] = 0.2

        with self.assertRaisesRegex(ValueError, "configured joint limits"):
            solver.solve(
                forward_kinematics(outside_seed)[:3, 3], outside_seed
            )

    def test_wrist_roll_preserves_unwrapped_angle(self):
        command = continuous_wrist_roll_target(
            0.0,
            math.radians(181.0),
            math.radians(179.0),
        )
        self.assertAlmostEqual(math.degrees(command), 181.0, places=10)

    def test_wrist_roll_is_not_reduced_modulo_two_pi(self):
        command = continuous_wrist_roll_target(
            0.0,
            math.radians(270.0),
            0.0,
        )
        self.assertAlmostEqual(math.degrees(command), 270.0, places=10)

    def test_wrist_roll_step_is_limited(self):
        previous = math.radians(179.0)
        command = continuous_wrist_roll_target(
            0.0,
            math.radians(181.0),
            previous,
            max_step_rad=math.radians(1.0),
        )
        self.assertAlmostEqual(math.degrees(command), 180.0, places=10)


class ControllerTwistTests(unittest.TestCase):
    @staticmethod
    def axis_quaternion(axis, angle_degrees):
        axis = np.asarray(axis, dtype=np.float64)
        axis /= np.linalg.norm(axis)
        half = math.radians(angle_degrees) / 2.0
        return np.concatenate((axis * math.sin(half), [math.cos(half)]))

    def test_twist_unwraps_across_180_degrees(self):
        tracker = ContinuousControllerTwist(max_step_rad=None)
        tracker.reset((0.0, 0.0, 0.0, 1.0))
        self.assertAlmostEqual(
            math.degrees(tracker.update(self.axis_quaternion((1, 0, 0), 170))),
            170.0,
            places=10,
        )
        self.assertAlmostEqual(
            math.degrees(tracker.update(self.axis_quaternion((1, 0, 0), 190))),
            190.0,
            places=10,
        )

    def test_quaternion_sign_flip_does_not_change_twist(self):
        tracker = ContinuousControllerTwist()
        tracker.reset((0.0, 0.0, 0.0, 1.0))
        quaternion = self.axis_quaternion((1, 0, 0), 20.0)
        first = tracker.update(quaternion)
        second = tracker.update(-quaternion)
        self.assertAlmostEqual(first, second, places=12)

    def test_large_twist_jump_is_consumed_over_multiple_updates(self):
        tracker = ContinuousControllerTwist(max_step_rad=math.radians(45.0))
        tracker.reset((0.0, 0.0, 0.0, 1.0))
        quaternion = self.axis_quaternion((1, 0, 0), 100.0)

        angles = [
            math.degrees(tracker.update(quaternion))
            for _ in range(3)
        ]

        np.testing.assert_allclose(angles, (45.0, 90.0, 100.0), atol=1.0e-10)

    def test_orthogonal_swing_has_no_twist(self):
        tracker = ContinuousControllerTwist()
        tracker.reset((0.0, 0.0, 0.0, 1.0))
        angle = tracker.update(self.axis_quaternion((0, 1, 0), 90.0))
        self.assertAlmostEqual(angle, 0.0, places=12)

    def test_orthogonal_180_degree_swing_is_rejected(self):
        tracker = ContinuousControllerTwist()
        tracker.reset((0.0, 0.0, 0.0, 1.0))
        with self.assertRaises(ControllerTwistError):
            tracker.update(self.axis_quaternion((0, 1, 0), 180.0))


if __name__ == "__main__":
    unittest.main()
