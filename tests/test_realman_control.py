import math
import unittest

from Realman_Control.realman_control import RealmanControl


class FakeArm:
    def rm_get_joint_min_pos(self):
        return 0, [-170.0, -120.0, -125.0, -170.0, -118.0, -340.0]

    def rm_get_joint_max_pos(self):
        return 0, [170.0, 120.0, 125.0, 170.0, 118.0, 340.0]


class ControllerJointLimitTests(unittest.TestCase):
    def setUp(self):
        self.robot = object.__new__(RealmanControl)
        self.robot._closed = False
        self.robot._arm = FakeArm()
        self.robot._joint_limits = tuple(RealmanControl.JOINT_LIMITS)

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


if __name__ == "__main__":
    unittest.main()
