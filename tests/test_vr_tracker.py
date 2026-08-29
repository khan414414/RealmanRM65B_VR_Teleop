import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from teleop.rm65_decoupled_ik import (
    RM65_JOINT_LIMITS_RAD,
    RM65_NOMINAL_POSTURE_RAD,
    forward_kinematics,
)
from teleop.vr_teleop import (
    ControllerSample,
    FixedRateScheduler,
    StreamFrequencyReporter,
    parse_controller_line,
    process_controller_sample,
)
from teleop.vr_tracker import TeleoperationSafetyWarning, VRTracker


class FakeRobot:
    def __init__(self):
        self.joints = np.asarray(RM65_NOMINAL_POSTURE_RAD, dtype=np.float64)
        self.limits = np.asarray(RM65_JOINT_LIMITS_RAD, dtype=np.float64)
        self.limit_reads = 0
        self.commands = []
        self.stop_calls = 0

    def get_active_tool_frame_pose_euler(self):
        return [0.0] * 6

    def get_joint_values(self):
        return self.joints.copy()

    def get_joint_limits(self):
        self.limit_reads += 1
        return self.limits.copy()

    def validate_joint_target(self, joints, *, singular_value_limit):
        del joints, singular_value_limit
        return 0

    def servo_joint(self, joints, *, time):
        self.joints = np.asarray(joints, dtype=np.float64).copy()
        self.commands.append((self.joints.copy(), float(time)))
        return 0

    def stop(self):
        self.stop_calls += 1
        return 0


class FakeGripper:
    def open_gripper(self):
        pass

    def close_gripper(self):
        pass


class DelayedFeedbackRobot(FakeRobot):
    def servo_joint(self, joints, *, time):
        command = np.asarray(joints, dtype=np.float64).copy()
        self.commands.append((command, float(time)))
        return 0


class FakeClock:
    def __init__(self, now=0.0):
        self.now = float(now)

    def __call__(self):
        return self.now


class VRTrackerTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobot()
        self.tracker = VRTracker(
            self.robot,
            FakeGripper(),
            rate=0.01,
            smooth_step=0.15,
            pos_mapping=(-1.0, -1.0, 1.0),
            wrist_roll_sign=1.0,
        )
        self.anchor = np.asarray((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
        self.tracker.resume(self.anchor)

    @staticmethod
    def mapped_axis_quaternion(axis, angle_degrees):
        axis = np.asarray(axis, dtype=np.float64)
        axis /= np.linalg.norm(axis)
        half = math.radians(angle_degrees) / 2.0
        return np.concatenate((axis * math.sin(half), [math.cos(half)]))

    def test_translation_changes_j1_to_j5_but_not_j6(self):
        start_joints = self.robot.joints.copy()
        start_position = forward_kinematics(self.robot.joints)[:3, 3]
        controller = self.anchor.copy()
        controller[0] += 0.01
        command = self.tracker.track(controller)
        end_position = forward_kinematics(command)[:3, 3]

        self.assertGreater(np.linalg.norm(command[:5] - start_joints[:5]), 0.0)
        self.assertAlmostEqual(command[5], self.anchor_joint_6, places=12)
        self.assertLess(end_position[0], start_position[0])
        self.assertAlmostEqual(
            end_position[0] - start_position[0], -0.0015, delta=0.0002
        )

    def test_translation_ratio_converges_one_to_one(self):
        start_position = forward_kinematics(self.robot.joints)[:3, 3]
        controller = self.anchor.copy()
        controller[0] += 0.01

        for _ in range(80):
            command = self.tracker.track(controller)

        end_position = forward_kinematics(command)[:3, 3]
        self.assertAlmostEqual(
            end_position[0] - start_position[0], -0.01, delta=0.00025
        )

    def test_controller_left_maps_to_robot_left(self):
        start_position = forward_kinematics(self.robot.joints)[:3, 3]
        controller = self.anchor.copy()
        # Unity left (-X) has already become Python +Y in the sender.
        controller[1] += 0.01

        for _ in range(80):
            command = self.tracker.track(controller)

        end_position = forward_kinematics(command)[:3, 3]
        self.assertAlmostEqual(
            end_position[1] - start_position[1], -0.01, delta=0.00025
        )

    def test_full_rotation_matrix_maps_controller_xyz(self):
        self.tracker.pause()
        mapping = np.asarray(
            ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        )
        self.tracker.set_position_mapping(mapping)
        self.tracker.resume(self.anchor)
        start_position = forward_kinematics(self.robot.joints)[:3, 3]
        controller = self.anchor.copy()
        controller[0] += 0.01

        for _ in range(80):
            command = self.tracker.track(controller)

        end_position = forward_kinematics(command)[:3, 3]
        self.assertAlmostEqual(
            end_position[1] - start_position[1], 0.01, delta=0.00025
        )

    def test_mapping_cannot_change_during_active_tracking(self):
        with self.assertRaisesRegex(RuntimeError, "only change while tracking"):
            self.tracker.set_position_mapping(np.eye(3))

    def test_controller_joint_limits_are_used_by_the_solver(self):
        self.assertEqual(
            self.tracker.joint_limit_source, "controller/teach pendant"
        )
        self.assertEqual(self.robot.limit_reads, 1)
        np.testing.assert_allclose(
            self.tracker.ik.joint_limits_rad, self.robot.limits
        )

    def test_all_six_joint_outputs_are_clipped_to_safe_limits(self):
        safe_limits = self.tracker.safe_joint_limits_rad
        raw_target = np.empty(6, dtype=np.float64)
        raw_target[::2] = safe_limits[::2, 0] - 1.0
        raw_target[1::2] = safe_limits[1::2, 1] + 1.0

        clipped = self.tracker._clip_joint_target(raw_target)

        np.testing.assert_allclose(clipped[::2], safe_limits[::2, 0])
        np.testing.assert_allclose(clipped[1::2], safe_limits[1::2, 1])

    def test_clipped_candidate_is_checked_then_sent(self):
        safe_limits = self.tracker.safe_joint_limits_rad
        previous = self.tracker.command_joints.copy()
        previous[0] = safe_limits[0, 1] - 0.5 * self.tracker._maximum_joint_step
        self.tracker.command_joints = previous.copy()
        self.robot.joints = previous.copy()
        self.tracker.target_position = forward_kinematics(previous)[:3, 3]
        self.tracker._next_following_error_check_at = math.inf

        raw_candidate = previous.copy()
        raw_candidate[0] = safe_limits[0, 1] + math.radians(1.0)
        self.tracker.ik.solve = lambda *args, **kwargs: SimpleNamespace(
            joints_rad=raw_candidate,
            converged=True,
            status="converged",
            position_error_m=0.0,
            minimum_singular_value=1.0,
        )
        self.tracker._model_singularity_check = lambda joints: None
        self.tracker._candidate_makes_progress = (
            lambda candidate, candidate_position, planned_position: True
        )
        self.tracker._rate_limit_cartesian_target = (
            lambda joints, maximum_step_m: (
                joints,
                forward_kinematics(joints)[:3, 3],
                False,
            )
        )

        command = self.tracker.track(self.anchor)

        self.assertAlmostEqual(command[0], safe_limits[0, 1], places=12)
        np.testing.assert_allclose(self.robot.commands[-1][0], command)
        self.assertLessEqual(
            np.max(np.abs(command - previous)),
            self.tracker._maximum_joint_step + 1.0e-12,
        )

    def test_j6_request_saturates_instead_of_faulting(self):
        safe_upper = self.tracker.safe_joint_limits_rad[5, 1]
        captured = {}
        original_solve = self.tracker.ik.solve

        def capture_wrist_target(*args, **kwargs):
            captured["wrist_target"] = kwargs["wrist_joint_6_target_rad"]
            return original_solve(*args, **kwargs)

        self.tracker.twist.update = lambda quaternion: math.radians(1000.0)
        self.tracker.ik.solve = capture_wrist_target

        command = self.tracker.track(self.anchor)

        self.assertAlmostEqual(captured["wrist_target"], safe_upper, places=12)
        self.assertLessEqual(command[5], safe_upper)
        self.assertIsNone(self.tracker.last_warning_reason)

    @property
    def anchor_joint_6(self):
        return float(RM65_NOMINAL_POSTURE_RAD[5])

    def test_mapped_unity_roll_moves_only_j6(self):
        controller = self.anchor.copy()
        # Unity +Z roll maps to a negative rotation about Python-local +X.
        controller[3:] = self.mapped_axis_quaternion((-1, 0, 0), 5.0)
        command = self.tracker.track(controller)

        np.testing.assert_allclose(
            command[:5], np.asarray(RM65_NOMINAL_POSTURE_RAD[:5]), atol=1.0e-12
        )
        self.assertLess(command[5], self.anchor_joint_6)
        self.assertLessEqual(
            abs(command[5] - self.anchor_joint_6),
            math.radians(0.9) + 1.0e-12,
        )

    def test_large_wrist_frame_is_retained_and_j6_approaches_gradually(self):
        controller = self.anchor.copy()
        controller[3:] = self.mapped_axis_quaternion((1, 0, 0), 100.0)
        commands = []

        for _ in range(130):
            commands.append(self.tracker.track(controller))

        joint_6 = np.asarray([command[5] for command in commands])
        steps = np.diff(
            np.concatenate(([self.anchor_joint_6], joint_6))
        )
        self.assertTrue(np.all(steps >= -1.0e-12))
        self.assertLessEqual(
            np.max(np.abs(steps)),
            self.tracker._maximum_joint_step + 1.0e-12,
        )
        self.assertAlmostEqual(
            math.degrees(joint_6[-1] - self.anchor_joint_6),
            100.0,
            delta=0.01,
        )

    def test_controller_swing_does_not_drive_j6(self):
        controller = self.anchor.copy()
        controller[3:] = self.mapped_axis_quaternion((0, 1, 0), 10.0)
        command = self.tracker.track(controller)
        self.assertAlmostEqual(command[5], self.anchor_joint_6, places=12)

    def test_workspace_target_is_projected_and_approached(self):
        anchor_position = self.tracker.tool_anchor_position.copy()
        controller = self.anchor.copy()
        controller[0] = 1.0

        command = self.tracker.track(controller)

        self.assertFalse(self.tracker.paused)
        self.assertIsNotNone(command)
        self.assertEqual(len(self.robot.commands), 1)
        self.assertAlmostEqual(
            np.linalg.norm(self.tracker.requested_position - anchor_position),
            self.tracker.max_position_offset_m,
            places=12,
        )
        self.assertLessEqual(
            np.linalg.norm(self.tracker.target_position - anchor_position),
            self.tracker._maximum_cartesian_step + 1.0e-12,
        )
        self.assertIn("projected workspace boundary", self.tracker.last_warning_reason)

        replacement = self.tracker.track(self.anchor)

        self.assertIsNotNone(replacement)
        self.assertEqual(len(self.robot.commands), 2)
        np.testing.assert_allclose(
            self.tracker.requested_position, anchor_position, atol=1.0e-12
        )
        self.assertNotIn(
            "projected workspace boundary",
            self.tracker.last_warning_reason or "",
        )

    def test_nonconverged_partial_ik_is_sent_and_retained_goal_remains(self):
        controller = self.anchor.copy()
        controller[0] += 0.01
        original_solve = self.tracker.ik.solve

        def report_safe_partial(*args, **kwargs):
            result = original_solve(*args, **kwargs)
            return SimpleNamespace(
                joints_rad=result.joints_rad,
                converged=False,
                status="trust_region_or_limit_reached",
                position_error_m=result.position_error_m,
                minimum_singular_value=result.minimum_singular_value,
            )

        self.tracker.ik.solve = report_safe_partial
        requested_position = (
            self.tracker.tool_anchor_position
            + self.tracker.position_mapping_matrix
            @ (controller[:3] - self.anchor[:3])
        )

        command = self.tracker.track(controller)

        self.assertEqual(len(self.robot.commands), 1)
        self.assertGreater(
            np.linalg.norm(command - np.asarray(RM65_NOMINAL_POSTURE_RAD)),
            0.0,
        )
        np.testing.assert_allclose(
            self.tracker.requested_position, requested_position, atol=1.0e-12
        )
        np.testing.assert_allclose(
            self.tracker.target_position,
            forward_kinematics(command)[:3, 3],
            atol=1.0e-12,
        )
        self.assertIn("approaching", self.tracker.last_warning_reason)

    def test_nonconverged_candidate_obeys_actual_cartesian_step_limit(self):
        previous_joints = self.tracker.command_joints.copy()
        previous_position = self.tracker.target_position.copy()
        raw_candidate = previous_joints.copy()
        raw_candidate[1] += self.tracker._maximum_joint_step
        requested_position = forward_kinematics(raw_candidate)[:3, 3]
        controller = self.anchor.copy()
        controller[:3] += np.linalg.solve(
            self.tracker.position_mapping_matrix,
            requested_position - self.tracker.tool_anchor_position,
        )
        planned_step = min(
            self.tracker._maximum_cartesian_step,
            self.tracker.smooth_step
            * np.linalg.norm(requested_position - previous_position),
        )
        self.tracker.ik.solve = lambda *args, **kwargs: SimpleNamespace(
            joints_rad=raw_candidate,
            converged=False,
            status="trust_region_or_limit_reached",
            position_error_m=0.001,
            minimum_singular_value=1.0,
        )
        self.tracker._model_singularity_check = lambda joints: None

        command = self.tracker.track(controller)
        actual_step = np.linalg.norm(
            forward_kinematics(command)[:3, 3] - previous_position
        )

        self.assertLessEqual(actual_step, planned_step + 1.0e-12)
        self.assertLess(
            np.linalg.norm(
                self.tracker.requested_position - self.tracker.target_position
            ),
            np.linalg.norm(requested_position - previous_position),
        )
        self.assertIn("approaching", self.tracker.last_warning_reason)

    def test_no_controller_goal_does_not_allow_null_space_joint_drift(self):
        previous = self.tracker.command_joints.copy()
        drifting_candidate = previous.copy()
        drifting_candidate[2] += math.radians(0.01)
        self.tracker.ik.solve = lambda *args, **kwargs: SimpleNamespace(
            joints_rad=drifting_candidate,
            converged=False,
            status="stagnated",
            position_error_m=0.0,
            minimum_singular_value=1.0,
        )
        self.tracker._model_singularity_check = lambda joints: None

        with self.assertRaisesRegex(
            TeleoperationSafetyWarning, "made no progress"
        ):
            self.tracker.track(self.anchor)

        self.assertEqual(self.robot.commands, [])
        np.testing.assert_allclose(self.tracker.command_joints, previous)

    def test_new_controller_frame_replaces_retained_goal_without_fifo(self):
        first = self.anchor.copy()
        first[0] += 0.05
        self.tracker.track(first)

        second = self.anchor.copy()
        second[0] -= 0.05
        second_requested = (
            self.tracker.tool_anchor_position
            + self.tracker.position_mapping_matrix
            @ (second[:3] - self.anchor[:3])
        )
        error_before = np.linalg.norm(
            second_requested - self.tracker.target_position
        )
        self.tracker.track(second)

        np.testing.assert_allclose(
            self.tracker.requested_position, second_requested, atol=1.0e-12
        )
        self.assertLess(
            np.linalg.norm(second_requested - self.tracker.target_position),
            error_before,
        )

    def test_joint_rate_limit_scales_the_whole_delta_vector(self):
        previous = self.tracker.command_joints.copy()
        maximum = self.tracker._maximum_joint_step
        raw = previous.copy()
        raw[0] += 2.0 * maximum
        raw[1] -= 4.0 * maximum

        limited, was_limited = self.tracker._rate_limit_joint_target(raw)

        self.assertTrue(was_limited)
        self.assertAlmostEqual(limited[0] - previous[0], 0.5 * maximum)
        self.assertAlmostEqual(limited[1] - previous[1], -maximum)
        np.testing.assert_allclose(limited[2:], previous[2:])

    def test_singular_full_candidate_uses_one_slow_validated_step(self):
        controller = self.anchor.copy()
        controller[0] += 0.01
        previous = self.tracker.command_joints.copy()
        original_solve = self.tracker.ik.solve
        captured = {}
        validation_calls = []

        def capture_result(*args, **kwargs):
            result = original_solve(*args, **kwargs)
            captured["result"] = result
            return result

        def reject_only_full_step(joints):
            validation_calls.append(np.asarray(joints).copy())
            if len(validation_calls) == 1:
                raise ValueError(
                    "joint target is near a shoulder singularity"
                )

        self.tracker.ik.solve = capture_result
        self.tracker._model_singularity_check = reject_only_full_step

        command = self.tracker.track(controller)

        clipped = self.tracker._clip_joint_target(captured["result"].joints_rad)
        full_delta = clipped - previous
        largest = np.max(np.abs(full_delta))
        if largest > self.tracker._maximum_joint_step:
            full_delta *= self.tracker._maximum_joint_step / largest
        expected = previous + self.tracker.safety_approach_scale * full_delta
        np.testing.assert_allclose(command, expected, atol=1.0e-12)
        self.assertEqual(len(validation_calls), 2)
        self.assertEqual(len(self.robot.commands), 1)
        self.assertIn("validated 2.5% step", self.tracker.last_warning_reason)

    def test_unsafe_target_is_retained_when_even_slow_step_is_invalid(self):
        controller = self.anchor.copy()
        controller[0] += 0.01
        target_before = self.tracker.target_position.copy()

        def reject_all_candidates(joints):
            del joints
            raise ValueError(
                "joint target is near a shoulder singularity"
            )

        self.tracker._model_singularity_check = reject_all_candidates
        with self.assertRaisesRegex(
            TeleoperationSafetyWarning, "target retained at the last safe"
        ):
            self.tracker.track(controller)

        self.assertEqual(self.robot.commands, [])
        np.testing.assert_allclose(self.tracker.target_position, target_before)
        self.assertIsNotNone(self.tracker.requested_position)
        retained = self.tracker.requested_position.copy()

        self.tracker._model_singularity_check = lambda joints: None
        command = self.tracker.track(controller)

        self.assertIsNotNone(command)
        self.assertEqual(len(self.robot.commands), 1)
        np.testing.assert_allclose(self.tracker.requested_position, retained)

    def test_following_error_is_time_aligned_then_catches_up_with_keepalive(self):
        clock = FakeClock()
        robot = DelayedFeedbackRobot()
        with patch("teleop.vr_tracker.time.monotonic", clock):
            tracker = VRTracker(
                robot,
                FakeGripper(),
                rate=0.01,
                smooth_step=1.0,
                max_following_error_rad=math.radians(0.1),
                following_error_check_period_s=0.005,
                following_error_settle_time_s=0.015,
            )
            tracker.resume(self.anchor)
            q0 = robot.joints.copy()
            q1 = q0.copy()
            q1[5] += math.radians(0.5)
            tracker.ik.solve = lambda *args, **kwargs: SimpleNamespace(
                joints_rad=q1,
                converged=True,
                status="converged",
                position_error_m=0.0,
                minimum_singular_value=1.0,
            )
            tracker._model_singularity_check = lambda joints: None
            tracker._candidate_makes_progress = (
                lambda candidate, candidate_position, planned_position: True
            )

            clock.now = 0.010
            tracker.track(self.anchor)
            self.assertEqual(len(robot.commands), 1)

            # q1 is still too new at 20 ms.  Low-follow feedback at q0 is
            # correctly compared with the mature q0 anchor, not latest q1.
            clock.now = 0.020
            tracker.track(self.anchor)
            self.assertFalse(tracker._following_error_blocked)

            # Once q1 has had its settle window, unchanged feedback is a real
            # following error.  The main loop can repeat q1 without advancing.
            clock.now = 0.030
            with self.assertRaisesRegex(
                TeleoperationSafetyWarning, "J6 following error"
            ):
                tracker.track(self.anchor)
            commands_before_keepalive = len(robot.commands)
            repeated = tracker.keepalive()
            np.testing.assert_allclose(repeated, q1)
            self.assertEqual(len(robot.commands), commands_before_keepalive + 1)
            self.assertEqual(robot.stop_calls, 0)

            robot.joints = q1.copy()
            clock.now = 0.040
            command = tracker.track(self.anchor)
            self.assertIsNotNone(command)
            self.assertFalse(tracker._following_error_blocked)

    def test_following_feedback_transport_error_propagates(self):
        self.tracker._next_following_error_check_at = 0.0

        def fail_feedback():
            raise RuntimeError("feedback transport failed")

        self.robot.get_joint_values = fail_feedback
        with self.assertRaisesRegex(RuntimeError, "feedback transport failed"):
            self.tracker.track(self.anchor)

    def test_following_hold_captures_latest_goal_without_advancing_command(self):
        command_before = self.tracker.command_joints.copy()
        target_before = self.tracker.target_position.copy()
        controller = self.anchor.copy()
        controller[0] += 0.01
        expected_requested = (
            self.tracker.tool_anchor_position
            + self.tracker.position_mapping_matrix
            @ (controller[:3] - self.anchor[:3])
        )
        self.tracker._following_error_blocked = True
        self.tracker._following_error_reason = "J2 following error is 12.0 deg"
        self.tracker._next_following_error_check_at = math.inf

        with self.assertRaisesRegex(
            TeleoperationSafetyWarning, "target retained while command is held"
        ):
            self.tracker.track(controller)

        np.testing.assert_allclose(
            self.tracker.requested_position, expected_requested, atol=1.0e-12
        )
        np.testing.assert_allclose(self.tracker.command_joints, command_before)
        np.testing.assert_allclose(self.tracker.target_position, target_before)
        self.assertEqual(self.robot.commands, [])

    def test_keepalive_repeats_last_safe_target_without_advancing_planner(self):
        controller = self.anchor.copy()
        controller[0] += 0.01
        command = self.tracker.track(controller)
        target_before = self.tracker.target_position.copy()
        history_before = len(self.tracker._command_history)

        repeated = self.tracker.keepalive()

        np.testing.assert_allclose(repeated, command)
        np.testing.assert_allclose(self.robot.commands[-1][0], command)
        np.testing.assert_allclose(self.tracker.target_position, target_before)
        self.assertEqual(len(self.tracker._command_history), history_before)

    def test_trigger_release_pauses_commands_without_calling_robot_stop(self):
        process_controller_sample(
            self.tracker,
            self.anchor[:3].tolist(),
            self.anchor[3:].tolist(),
            False,
        )

        self.assertTrue(self.tracker.paused)
        self.assertIsNone(self.tracker.requested_position)
        self.assertIsNone(self.tracker.requested_wrist)
        self.assertEqual(self.robot.stop_calls, 0)

    def test_transport_error_is_not_downgraded_to_a_safety_warning(self):
        def fail_servo(joints, *, time):
            del joints, time
            raise RuntimeError("transport failed")

        self.robot.servo_joint = fail_servo

        with self.assertRaisesRegex(RuntimeError, "transport failed"):
            self.tracker.track(self.anchor)
        self.assertFalse(self.tracker.paused)


class ControllerProtocolTests(unittest.TestCase):
    def test_parse_controller_line(self):
        sample = parse_controller_line(
            "1,2,3,0,0,0,1,1,0"
        )
        self.assertEqual(sample.position, (1.0, 2.0, 3.0))
        self.assertEqual(sample.rotation, (0.0, 0.0, 0.0, 1.0))
        self.assertTrue(sample.trigger_held)
        self.assertIsNone(sample.sequence_id)

    def test_parse_timestamped_controller_line(self):
        sample = parse_controller_line(
            "1,2,3,0,0,0,1,1,0,42,12.345678"
        )
        self.assertEqual(sample.sequence_id, 42)
        self.assertAlmostEqual(sample.unity_time_s, 12.345678)

    def test_parse_controller_line_rejects_partial_record(self):
        with self.assertRaises(ValueError):
            parse_controller_line("1,2,3")


class FrequencySynchronizationTests(unittest.TestCase):
    def test_scheduler_skips_missed_ticks_without_bursting(self):
        scheduler = FixedRateScheduler(100.0, start_time_s=0.0)
        self.assertEqual(scheduler.consume_due(0.009), (False, 0))
        self.assertEqual(scheduler.consume_due(0.010), (True, 0))
        self.assertAlmostEqual(scheduler.next_deadline_s, 0.020)

        self.assertEqual(scheduler.consume_due(0.045), (True, 2))
        self.assertAlmostEqual(scheduler.next_deadline_s, 0.050)
        self.assertEqual(scheduler.consume_due(0.045), (False, 0))

    def test_frequency_reporter_uses_unity_sequence_and_timestamp(self):
        reporter = StreamFrequencyReporter(1.0, start_time_s=0.0)
        samples = (
            ControllerSample((0, 0, 0), (0, 0, 0, 1), False, 0, 0.00),
            ControllerSample((0, 0, 0), (0, 0, 0, 1), False, 1, 0.01),
            ControllerSample((0, 0, 0), (0, 0, 0, 1), False, 3, 0.03),
        )
        for sample in samples:
            reporter.observe_sample(sample)
        for _ in range(100):
            reporter.observe_robot_command()
        reporter.observe_missed_robot_ticks(2)

        message = reporter.maybe_report(1.0, 0.004)

        self.assertIn("Meta/Unity 100.0 Hz", message)
        self.assertIn("robot commands 100.0 Hz", message)
        self.assertIn("dropped Meta samples 1", message)
        self.assertIn("skipped robot ticks 2", message)


if __name__ == "__main__":
    unittest.main()
