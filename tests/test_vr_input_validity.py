import unittest
from unittest.mock import Mock

from teleop.vr_calibration import GuidedPositionCalibrator
from teleop.vr_teleop import reject_invalid_controller_input


class InputValidityTests(unittest.TestCase):
    def test_tracking_loss_pauses_and_discards_partial_calibration(self):
        tracker = Mock(paused=False)
        calibrator = GuidedPositionCalibrator(sample_count=2)
        calibrator.start()
        calibrator.request_capture()
        calibrator.observe((0.0, 0.0, 0.0))
        message = reject_invalid_controller_input(tracker, calibrator)
        tracker.pause.assert_called_once_with()
        self.assertIsNotNone(message)
        self.assertFalse(calibrator.active)
        self.assertFalse(calibrator.capture_requested)
        self.assertIsNone(calibrator.observe((0.12, 0.0, 0.0)))
        self.assertFalse(calibrator.awaiting_confirmation)

    def test_repeated_invalid_input_does_not_restart_or_repause(self):
        tracker = Mock(paused=True)
        calibrator = GuidedPositionCalibrator()
        self.assertIsNone(reject_invalid_controller_input(tracker, calibrator))
        tracker.pause.assert_not_called()
        self.assertFalse(calibrator.active)


if __name__ == "__main__":
    unittest.main()
