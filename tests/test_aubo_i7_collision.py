import math
import unittest

import numpy as np

from teleop.aubo_i7_collision import (
    AuboCollisionError,
    AuboI7CollisionChecker,
    segment_distance_m,
)
from teleop.aubo_i7_decoupled_ik import AUBO_I7_NOMINAL_POSTURE_RAD


class SegmentDistanceTests(unittest.TestCase):
    def test_crossing_segments_have_zero_distance(self):
        self.assertAlmostEqual(
            segment_distance_m(
                (-1.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
                (0.0, 1.0, 0.0),
            ),
            0.0,
        )

    def test_parallel_segments_report_their_separation(self):
        self.assertAlmostEqual(
            segment_distance_m(
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 0.25, 0.0),
                (1.0, 0.25, 0.0),
            ),
            0.25,
        )


class AuboI7CollisionCheckerTests(unittest.TestCase):
    def test_new_nominal_posture_is_clear_of_self_and_plane(self):
        AuboI7CollisionChecker().validate(AUBO_I7_NOMINAL_POSTURE_RAD)

    def test_folded_posture_is_rejected_as_self_collision(self):
        joints = np.radians((-167.2, 5.4, -12.2, 150.2, 46.5, 5.1))
        checker = AuboI7CollisionChecker(plane_z_m=-10.0)
        with self.assertRaisesRegex(
            AuboCollisionError, "self-collision risk between"
        ):
            checker.validate(joints)

    def test_old_low_posture_is_rejected_by_base_plane(self):
        old_posture = tuple(
            math.radians(value)
            for value in (0.0, -60.0, 75.0, 0.0, 75.0, 0.0)
        )
        with self.assertRaisesRegex(AuboCollisionError, "plane Z=0.000 m"):
            AuboI7CollisionChecker().validate(old_posture)

    def test_active_tcp_extension_is_included_as_a_tool_capsule(self):
        tcp = np.eye(4)
        tcp[2, 3] = 0.25
        names = {
            body.name
            for body in AuboI7CollisionChecker().capsules(
                AUBO_I7_NOMINAL_POSTURE_RAD,
                tcp,
            )
        }
        self.assertIn("tool", names)


if __name__ == "__main__":
    unittest.main()
