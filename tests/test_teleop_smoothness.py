"""Continuous-trajectory regressions using the real offline official SDK."""

import math
import sys
import unittest
from unittest.mock import patch

import numpy as np

from tests.test_official_position_ik import (
    offline_robot, REACHABILITY_SEED_DEG, REACHABILITY_LIMITS_DEG, REACHABILITY_TARGET,
)
from teleop.vr_tracker import VRTracker


@unittest.skipUnless(sys.platform == "win32", "official DLL requires Windows")
class SmoothnessTests(unittest.TestCase):
    def trajectory(self, hand_position, *, seed_deg=(0, -75, 60, 0, 60, 0),
                   limits_deg=None, ticks=300):
        robot = offline_robot()
        if limits_deg is not None:
            robot._joint_limits = tuple(map(tuple, np.radians(limits_deg)))
        robot._simulated_joints = np.radians(seed_deg)
        robot.get_joint_values = lambda: robot._simulated_joints.copy()
        robot.get_joint_limits = lambda: robot._joint_limits
        robot.get_active_tool_frame_pose_euler = lambda: [0] * 6

        def servo(joints, **kwargs):
            robot._simulated_joints = np.asarray(joints).copy()
            return 0

        robot.servo_joint = servo
        clock = [0.0]
        with patch("teleop.vr_tracker.time.monotonic", lambda: clock[0]):
            tracker = VRTracker(robot, None, rate=0.01, smooth_step=0.15, wrist_smooth_step=0.15)
            tracker.ik.max_solve_time_s = None
            tracker.resume([0, 0, 0, 0, 0, 0, 1])
            joints = [tracker.command_joints.copy()]
            positions = [tracker.target_position.copy()]
            for tick in range(1, ticks + 1):
                clock[0] = tick * 0.01
                native_time = math.floor(clock[0] * 72) / 72
                hand = np.asarray(hand_position(native_time))
                position = (float(hand), 0, 0) if hand.ndim == 0 else hand.tolist()
                tracker.track([*position, 0, 0, 0, 1])
                joints.append(tracker.command_joints.copy())
                positions.append(tracker.target_position.copy())
        return np.asarray(joints), np.asarray(positions)

    def test_fast_motion_does_not_alternate_free_wrist_postures_or_hold(self):
        joints, positions = self.trajectory(lambda t: 0.05 * math.sin(2 * math.pi * t))
        velocity = np.diff(np.degrees(joints), axis=0) / 0.01
        acceleration = np.diff(velocity, axis=0) / 0.01
        self.assertLess(np.max(np.abs(velocity[:, 3])), 0.05)
        # The old adapter produced 18,000 deg/s^2 and near-zero repeated steps.
        self.assertLess(np.max(np.abs(acceleration)), 10000)
        self.assertLessEqual(np.max(np.abs(np.diff(joints, axis=0))), math.radians(1.8) + 1e-12)
        steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        self.assertLessEqual(np.max(steps), 0.006 + 1e-12)
        self.assertLess(np.mean(steps[40:] < 1e-6), 0.02)

    def test_slow_72hz_input_produces_continuous_100hz_motion(self):
        joints, positions = self.trajectory(lambda t: 0.006 * t)
        speeds = np.linalg.norm(np.diff(positions, axis=0), axis=1)[40:] / 0.01
        self.assertAlmostEqual(float(np.mean(speeds)), 0.009, delta=0.0002)
        self.assertLess(float(np.std(speeds)), 0.002)
        self.assertLess(np.mean(speeds < 0.0001), 0.02)

    def test_released_orientation_stays_continuous_on_out_and_back_motion(self):
        robot = offline_robot()
        seed = np.radians(REACHABILITY_SEED_DEG)
        origin = np.asarray(robot.official_forward_kinematics(seed)[:3])
        target = np.asarray(REACHABILITY_TARGET)

        def hand(t):
            fraction = 0.5 * (1 - math.cos(2 * math.pi * min(t, 4) / 4))
            return fraction * (target - origin) / 1.5

        joints, positions = self.trajectory(hand, seed_deg=REACHABILITY_SEED_DEG,
                                            limits_deg=REACHABILITY_LIMITS_DEG, ticks=500)
        velocity = np.diff(np.degrees(joints), axis=0) / 0.01
        acceleration = np.diff(velocity, axis=0) / 0.01
        self.assertLess(np.max(np.abs(acceleration)), 8000)
        self.assertLess(np.linalg.norm(positions[200] - target), 0.002)
        self.assertLess(np.linalg.norm(positions[-1] - origin), 2e-5)
        np.testing.assert_allclose(joints[:, 5], seed[5], atol=1e-12)
        self.assertGreater(np.ptp(joints[:, 4]), math.radians(1))
        self.assertLessEqual(np.max(np.abs(np.diff(joints, axis=0))), math.radians(1.8) + 1e-12)
        self.assertLess(np.max(np.linalg.norm(positions - origin, axis=1)), 0.25)


if __name__ == "__main__":
    unittest.main()
