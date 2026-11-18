import math
import tempfile
import unittest
from collections import deque
from pathlib import Path

import numpy as np

from teleop.vr_calibration import (
    CalibrationError,
    ConsoleKeyReader,
    GuidedPositionCalibrator,
    load_calibration,
    save_calibration_atomic,
    solve_position_mapping,
)


class PositionCalibrationTests(unittest.TestCase):
    @staticmethod
    def known_samples():
        angle = math.radians(35.0)
        rotation = np.asarray(
            (
                (math.cos(angle), -math.sin(angle), 0.0),
                (math.sin(angle), math.cos(angle), 0.0),
                (0.0, 0.0, 1.0),
            ),
            dtype=np.float64,
        )
        center = np.asarray((0.4, -0.2, 1.1), dtype=np.float64)
        measured_directions = rotation.T
        endpoints = tuple(
            center + 0.12 * measured_directions[:, index]
            for index in range(3)
        )
        return rotation, center, endpoints

    def test_solver_recovers_rotation_and_preserves_one_to_one_distance(self):
        expected, center, endpoints = self.known_samples()

        result = solve_position_mapping(center, *endpoints)

        np.testing.assert_allclose(
            result.rotation_vr_to_robot, expected, atol=1.0e-12
        )
        displacement = np.asarray((0.03, -0.07, 0.02))
        mapped = result.rotation_vr_to_robot @ displacement
        self.assertAlmostEqual(
            np.linalg.norm(mapped), np.linalg.norm(displacement), places=12
        )

    def test_solver_rejects_left_handed_samples(self):
        center = np.zeros(3)
        with self.assertRaisesRegex(CalibrationError, "right-handed"):
            solve_position_mapping(
                center,
                (0.12, 0.0, 0.0),
                (0.0, 0.12, 0.0),
                (0.0, 0.0, -0.12),
            )

    def test_mirrored_and_swapped_axes_explain_handedness_failure(self):
        _, center, endpoints = self.known_samples()
        for bad_endpoints in (
            (endpoints[0], center - (endpoints[1] - center), endpoints[2]),
            (endpoints[1], endpoints[0], endpoints[2]),
        ):
            with self.subTest(endpoints=bad_endpoints):
                with self.assertRaises(CalibrationError) as caught:
                    solve_position_mapping(center, *bad_endpoints)
                message = str(caught.exception)
                self.assertIn("right-handed", message)
                self.assertIn("三重积=-1.000", message)
                self.assertIn("左手系", message)
                self.assertIn("机器人基坐标", message)

    def test_calibration_start_explains_robot_axes_and_common_handedness_mistake(self):
        message = GuidedPositionCalibrator().start().message
        self.assertIn("机器人基坐标", message)
        self.assertIn("+X×+Y=+Z", message)
        self.assertIn("+Y 应向左", message)

    def test_guided_sampling_rejects_motion_then_completes(self):
        expected, center, endpoints = self.known_samples()
        calibrator = GuidedPositionCalibrator(
            sample_count=4,
            max_sample_rms_m=0.003,
            max_sample_deviation_m=0.008,
        )
        calibrator.start()
        calibrator.request_capture()
        unstable = (
            center + (0.01, 0.0, 0.0),
            center - (0.01, 0.0, 0.0),
            center + (0.01, 0.0, 0.0),
            center - (0.01, 0.0, 0.0),
        )
        update = None
        for sample in unstable:
            update = calibrator.observe(sample)
        self.assertIn("采样失败", update.message)
        self.assertFalse(calibrator.capture_requested)

        for point in (center, *endpoints):
            calibrator.request_capture()
            update = None
            for _ in range(calibrator.sample_count):
                update = calibrator.observe(point)

        self.assertTrue(calibrator.awaiting_confirmation)
        self.assertIsNotNone(update.result)
        np.testing.assert_allclose(
            update.result.rotation_vr_to_robot, expected, atol=1.0e-12
        )

    def test_guided_sampling_reports_3d_positions_and_retries_bad_axis(self):
        _, center, endpoints = self.known_samples()
        calibrator = GuidedPositionCalibrator(sample_count=2)
        calibrator.start()

        calibrator.request_capture()
        update = calibrator.observe(center)
        update = calibrator.observe(center)
        self.assertIn("原点三维位置", update.message)
        self.assertIn("x=+0.4000", update.message)
        self.assertIn("y=-0.2000", update.message)
        self.assertIn("z=+1.1000", update.message)

        too_short = center + np.asarray((0.04, 0.0, 0.0))
        calibrator.request_capture()
        calibrator.observe(too_short)
        update = calibrator.observe(too_short)
        self.assertIn("+X 采样不合格", update.message)
        self.assertIn("距离为 4.0 cm（过短）", update.message)
        self.assertIn("dx=+4.0", update.message)

        for point in endpoints:
            calibrator.request_capture()
            calibrator.observe(point)
            update = calibrator.observe(point)

        self.assertTrue(calibrator.awaiting_confirmation)
        self.assertIn("VR 三维采样位置", update.message)
        self.assertIn("距离 12.0 cm", update.message)

    def test_solver_distance_error_reports_each_measured_axis(self):
        center = np.zeros(3)
        with self.assertRaisesRegex(
            CalibrationError,
            r"\+X=4\.0 cm, \+Y=12\.0 cm, \+Z=30\.0 cm",
        ):
            solve_position_mapping(
                center,
                (0.04, 0.0, 0.0),
                (0.0, 0.12, 0.0),
                (0.0, 0.0, 0.30),
            )

    def test_quality_failure_reports_all_sampled_3d_positions(self):
        center = np.asarray((0.4, -0.2, 1.1))
        left_handed_points = (
            center + (0.10, 0.0, 0.0),
            center + (0.0, 0.12, 0.0),
            center + (0.0, 0.0, -0.14),
        )
        calibrator = GuidedPositionCalibrator(sample_count=2)
        calibrator.start()

        update = None
        for point in (center, *left_handed_points):
            calibrator.request_capture()
            calibrator.observe(point)
            update = calibrator.observe(point)

        self.assertIn("标定质量检查失败", update.message)
        self.assertIn("right-handed", update.message)
        self.assertIn("原点: [x=+0.4000, y=-0.2000, z=+1.1000] m", update.message)
        self.assertIn("+X:", update.message)
        self.assertIn("+Y:", update.message)
        self.assertIn("+Z:", update.message)
        self.assertIn("距离 10.0 cm", update.message)
        self.assertIn("距离 12.0 cm", update.message)
        self.assertIn("距离 14.0 cm", update.message)
        self.assertFalse(calibrator.awaiting_confirmation)

    def test_calibration_json_round_trip(self):
        _, center, endpoints = self.known_samples()
        result = solve_position_mapping(center, *endpoints)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "vr_position_calibration.json"
            save_calibration_atomic(path, result)
            loaded = load_calibration(path)

        np.testing.assert_allclose(
            loaded.rotation_vr_to_robot, result.rotation_vr_to_robot
        )
        self.assertEqual(loaded.axis_lengths_m, result.axis_lengths_m)


class FakeConsoleBackend:
    def __init__(self, characters):
        self.characters = deque(characters)

    def kbhit(self):
        return bool(self.characters)

    def getwch(self):
        return self.characters.popleft()


class ConsoleKeyReaderTests(unittest.TestCase):
    def test_nonblocking_reader_handles_controls_and_extended_keys(self):
        backend = FakeConsoleBackend(("q", "\r", "\xe0", "K", "y"))
        reader = ConsoleKeyReader(backend)

        self.assertEqual(reader.poll(), ("q", "\r", "y"))

    def test_ctrl_c_remains_keyboard_interrupt(self):
        reader = ConsoleKeyReader(FakeConsoleBackend(("\x03",)))
        with self.assertRaises(KeyboardInterrupt):
            reader.poll()


if __name__ == "__main__":
    unittest.main()
