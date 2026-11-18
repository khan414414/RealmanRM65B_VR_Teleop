import json
import math
import io
from pathlib import Path
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import MagicMock, patch

from teleop.vr_pose_diagnostics import compare_pose_displacements, diagnostic_matches_sample, main
from teleop.vr_protocol import DIAGNOSTIC_PREFIX, parse_controller_line, parse_pose_diagnostic


def vector(x=0, y=0, z=0):
    return {"x": x, "y": y, "z": z}


def diagnostic():
    return {"schema": 1, "sequence_id": 10, "raw_position_valid": True,
            "tracking_flag_valid": True, "is_tracked": True, "tracking_state": 3,
            "device_name": "Oculus Touch Controller OpenXR",
            "raw_position": vector(), "local_position": vector(),
            "world_position": vector(), "sent_position": vector(),
            "parent_scale": vector(1, 1, 1), "world_scale": vector(1, 1, 1)}


class PoseDiagnosticTests(unittest.TestCase):
    def test_detects_twelve_cm_raw_but_three_cm_world_scaling(self):
        anchor, current = diagnostic(), diagnostic()
        current["raw_position"] = vector(z=0.12)
        current["local_position"] = vector(z=0.12)
        current["world_position"] = vector(z=0.03)
        current["sent_position"] = vector(x=0.03)
        result = compare_pose_displacements(anchor, current)
        self.assertAlmostEqual(result["raw_position"]["distance_m"], 0.12)
        self.assertAlmostEqual(result["sent_position"]["distance_m"], 0.03)
        self.assertAlmostEqual(result["world_to_raw_distance_ratio"], 0.25)

    def test_axis_permutation_and_sign_preserve_distance(self):
        anchor, current = diagnostic(), diagnostic()
        current["raw_position"] = vector(0.03, 0.04, 0.12)
        current["world_position"] = vector(0.03, 0.04, 0.12)
        current["sent_position"] = vector(0.12, -0.03, 0.04)
        result = compare_pose_displacements(anchor, current)
        self.assertAlmostEqual(result["raw_position"]["distance_m"], 0.13)
        self.assertAlmostEqual(result["sent_position"]["distance_m"], 0.13)
        self.assertAlmostEqual(result["world_to_raw_distance_ratio"], 1)

    def test_unavailable_raw_tracking_is_not_reported_as_a_zero_distance(self):
        anchor, current = diagnostic(), diagnostic()
        current["raw_position_valid"] = False
        result = compare_pose_displacements(anchor, current)
        self.assertNotIn("raw_position", result)
        self.assertNotIn("world_to_raw_distance_ratio", result)

    def test_readable_but_untracked_or_position_invalid_raw_pose_is_rejected(self):
        for changes in ({"is_tracked": False}, {"tracking_flag_valid": False},
                        {"tracking_state": 2}):
            with self.subTest(changes=changes):
                anchor, current = diagnostic(), diagnostic()
                current.update(changes)
                current["raw_position"] = vector(z=0.12)
                result = compare_pose_displacements(anchor, current)
                self.assertNotIn("raw_position", result)
                self.assertNotIn("world_to_raw_distance_ratio", result)

    def test_changing_native_device_does_not_produce_a_scale_ratio(self):
        anchor, current = diagnostic(), diagnostic()
        current["device_name"] = "Hand Interaction OpenXR"
        current["raw_position"] = vector(z=0.12)
        self.assertNotIn("world_to_raw_distance_ratio", compare_pose_displacements(anchor, current))

    def test_metadata_matches_same_frame_csv_and_detects_changed_payload(self):
        payload = diagnostic()
        sample = parse_controller_line("0,0,0,0,0,0,1,0,0,10,3.0")
        self.assertTrue(diagnostic_matches_sample(payload, sample))
        payload["sent_position"] = vector(0.03, 0, 0)
        self.assertFalse(diagnostic_matches_sample(payload, sample))
        payload["sequence_id"] = 11
        self.assertIsNone(diagnostic_matches_sample(payload, sample))

    def test_diagnostic_rejects_invalid_schema_and_nonfinite_position(self):
        payload = diagnostic()
        parsed = parse_pose_diagnostic(DIAGNOSTIC_PREFIX + json.dumps(payload))
        self.assertEqual(parsed["sequence_id"], 10)
        payload["world_position"] = vector(math.nan, 0, 0)
        with self.assertRaises(ValueError):
            parse_pose_diagnostic(DIAGNOSTIC_PREFIX + json.dumps(payload))
        with self.assertRaises(ValueError):
            parse_pose_diagnostic(DIAGNOSTIC_PREFIX + '{"schema":2}')

    def test_legacy_and_timestamped_motion_records_keep_original_units(self):
        for suffix in ("", ",5,1.25"):
            sample = parse_controller_line("0.12,-0.03,0.04,0,0,0,1,0,0" + suffix)
            self.assertEqual(sample.position, (0.12, -0.03, 0.04))

    def test_collector_logs_invalid_input_and_recovers_to_matching_motion(self):
        invalid, valid = diagnostic(), diagnostic()
        invalid["motion_record_valid"] = False
        valid["motion_record_valid"] = True
        wire = ("#VR_INPUT_INVALID right_touch_not_tracked\n"
                + DIAGNOSTIC_PREFIX + json.dumps(invalid) + "\n"
                + "0,0,0,0,0,0,1,0,0,10,3.0\n"
                + DIAGNOSTIC_PREFIX + json.dumps(valid) + "\n").encode()
        connection = MagicMock()
        sock = connection.__enter__.return_value
        sock.recv.side_effect = [wire, b""]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture.jsonl"
            with (patch("teleop.vr_pose_diagnostics.socket.create_connection", return_value=connection),
                  patch("teleop.vr_pose_diagnostics.select.select", return_value=([sock], [], [])),
                  redirect_stdout(io.StringIO())):
                result = main(["--duration", "1", "--output", str(output)])
            rows = [json.loads(line) for line in output.read_text().splitlines()]
        self.assertEqual(result, 0)
        self.assertEqual([row["kind"] for row in rows],
                         ["input_invalid", "layers", "received", "layers"])
        self.assertEqual(rows[0]["reason"], "right_touch_not_tracked")
        self.assertIsNone(rows[1]["sample_matches"])
        self.assertTrue(rows[-1]["sample_matches"])


if __name__ == "__main__":
    unittest.main()
