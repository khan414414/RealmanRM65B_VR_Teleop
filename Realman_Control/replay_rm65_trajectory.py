r"""Safely inspect and replay an RM65-B state log as a joint trajectory.

Dry-run validation is the default.  The robot is contacted only when
``--execute`` is supplied, and motion still requires an interactive ``REPLAY``
confirmation.  NPZ is preferred because it preserves integer timestamps and
float arrays without CSV formatting loss.

Examples::

    python -m Realman_Control.replay_rm65_trajectory C:\RM65Logs\RM65_state.npz

    python -m Realman_Control.replay_rm65_trajectory \
        C:\RM65Logs\RM65_state.npz --execute --move-to-start
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from Realman_Control.realman_control import RM65B


JOINT_COUNT = 6
DEFAULT_OUTPUT_HZ = 50.0

ARM_STATUS_NAMES = {
    0: "idle",
    1: "move_l",
    2: "move_j",
    3: "move_c",
    4: "move_s",
    5: "joint_passthrough",
    6: "pose_passthrough",
    7: "force_pose_passthrough",
    8: "current_passthrough",
    9: "stop",
    10: "slow_stop",
    11: "pause",
    12: "current_drag",
    13: "sensor_drag",
    14: "teach_demonstration",
    15: "trajectory_reproduction",
    16: "move_initial_position",
    17: "cartesian_velocity_passthrough",
}


class ReplayValidationError(ValueError):
    """The recorded trajectory is not safe or complete enough to replay."""


@dataclass(frozen=True, slots=True)
class RecordedTrajectory:
    source_path: Path
    time_s: np.ndarray
    joint_rad: np.ndarray
    sdk_error_code: np.ndarray
    arm_current_status: np.ndarray
    merged_duplicate_timestamps: int

    @property
    def duration_s(self) -> float:
        return float(self.time_s[-1] - self.time_s[0])


@dataclass(frozen=True, slots=True)
class PreparedTrajectory:
    source_path: Path
    time_s: np.ndarray
    joint_rad: np.ndarray
    source_start_index: int
    source_end_index: int
    clipped_value_count: int
    max_limit_excess_rad: float

    @property
    def duration_s(self) -> float:
        return float(self.time_s[-1])


def _parse_csv_integer(value: str, field: str, row_number: int) -> int:
    text = value.strip()
    try:
        return int(text)
    except ValueError:
        try:
            numeric = float(text)
        except ValueError as exc:
            raise ReplayValidationError(
                f"CSV row {row_number}: {field} is not numeric"
            ) from exc
        if not math.isfinite(numeric):
            raise ReplayValidationError(
                f"CSV row {row_number}: {field} is not finite"
            )
        return int(round(numeric))


def _build_recorded_trajectory(
    path: Path,
    time_s: np.ndarray,
    joints: np.ndarray,
    sdk_errors: np.ndarray,
    arm_status: np.ndarray,
) -> RecordedTrajectory:
    sample_count = len(time_s)
    if joints.ndim != 2 or joints.shape[0] != sample_count:
        raise ReplayValidationError(
            "Joint sample count does not match the timestamp count"
        )
    if sdk_errors.shape != (sample_count,):
        raise ReplayValidationError(
            "SDK error sample count does not match the timestamp count"
        )
    if arm_status.shape != (sample_count,):
        raise ReplayValidationError(
            "Arm-status sample count does not match the timestamp count"
        )
    if len(time_s) and np.any(np.diff(time_s) < 0.0):
        raise ReplayValidationError("Trajectory timestamps move backwards")

    keep = np.ones(len(time_s), dtype=np.bool_)
    if len(time_s) > 1:
        # Keep the newest state when Windows gives consecutive callbacks the
        # same monotonic-clock value.
        keep[:-1] = time_s[:-1] != time_s[1:]
    merged = int(np.count_nonzero(~keep))
    time_s = time_s[keep]
    joints = joints[keep]
    sdk_errors = sdk_errors[keep]
    arm_status = arm_status[keep]
    if len(time_s):
        time_s = time_s - time_s[0]
    return RecordedTrajectory(
        path,
        time_s,
        joints,
        sdk_errors,
        arm_status,
        merged,
    )


def _load_npz(path: Path) -> RecordedTrajectory:
    try:
        with np.load(path, allow_pickle=False) as data:
            required = {"joint_rad", "sdk_error_code", "arm_current_status"}
            missing = sorted(required.difference(data.files))
            if missing:
                raise ReplayValidationError(
                    f"NPZ is missing required fields: {', '.join(missing)}"
                )

            joints = np.asarray(data["joint_rad"], dtype=np.float64).copy()
            sdk_errors = np.asarray(
                data["sdk_error_code"], dtype=np.int32
            ).copy()
            arm_status = np.asarray(
                data["arm_current_status"], dtype=np.int32
            ).copy()

            if "mono_ns" in data.files:
                time_ns = np.asarray(data["mono_ns"], dtype=np.int64).copy()
            elif "device_time_ns" in data.files:
                time_ns = np.asarray(
                    data["device_time_ns"], dtype=np.int64
                ).copy()
            else:
                raise ReplayValidationError(
                    "NPZ contains neither mono_ns nor device_time_ns"
                )
    except OSError as exc:
        raise ReplayValidationError(f"Could not read NPZ: {exc}") from exc

    if time_ns.ndim != 1 or not len(time_ns):
        raise ReplayValidationError("NPZ contains no one-dimensional timestamps")
    time_s = (time_ns - time_ns[0]).astype(np.float64) / 1_000_000_000.0
    return _build_recorded_trajectory(
        path, time_s, joints, sdk_errors, arm_status
    )


def _load_csv(path: Path) -> RecordedTrajectory:
    joint_fields = [f"joint_{index}_rad" for index in range(1, 7)]
    times: list[int] = []
    joints: list[list[float]] = []
    sdk_errors: list[int] = []
    arm_status: list[int] = []

    try:
        with path.open("r", newline="", encoding="utf-8-sig") as csv_file:
            reader = csv.DictReader(csv_file)
            fields = set(reader.fieldnames or [])
            required = set(joint_fields) | {
                "sdk_error_code",
                "arm_current_status",
            }
            missing = sorted(required.difference(fields))
            if missing:
                raise ReplayValidationError(
                    f"CSV is missing required columns: {', '.join(missing)}"
                )

            if "timestamp_relative_ns" in fields:
                time_field = "timestamp_relative_ns"
            elif "device_time_ns" in fields:
                time_field = "device_time_ns"
            else:
                raise ReplayValidationError(
                    "CSV contains neither timestamp_relative_ns nor device_time_ns"
                )

            for row_number, row in enumerate(reader, start=2):
                try:
                    joint_row = [float(row[field]) for field in joint_fields]
                except (TypeError, ValueError) as exc:
                    raise ReplayValidationError(
                        f"CSV row {row_number}: invalid joint value"
                    ) from exc
                times.append(
                    _parse_csv_integer(row[time_field], time_field, row_number)
                )
                joints.append(joint_row)
                sdk_errors.append(
                    _parse_csv_integer(
                        row["sdk_error_code"], "sdk_error_code", row_number
                    )
                )
                arm_status.append(
                    _parse_csv_integer(
                        row["arm_current_status"],
                        "arm_current_status",
                        row_number,
                    )
                )
    except OSError as exc:
        raise ReplayValidationError(f"Could not read CSV: {exc}") from exc

    if not times:
        raise ReplayValidationError("CSV contains no state samples")
    time_ns = np.asarray(times, dtype=np.int64)
    time_s = (time_ns - time_ns[0]).astype(np.float64) / 1_000_000_000.0
    return _build_recorded_trajectory(
        path,
        time_s,
        np.asarray(joints, dtype=np.float64),
        np.asarray(sdk_errors, dtype=np.int32),
        np.asarray(arm_status, dtype=np.int32),
    )


def load_recorded_trajectory(path: Path) -> RecordedTrajectory:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise ReplayValidationError(f"Trajectory file does not exist: {resolved}")
    suffix = resolved.suffix.lower()
    if suffix == ".npz":
        trajectory = _load_npz(resolved)
    elif suffix == ".csv":
        trajectory = _load_csv(resolved)
    else:
        raise ReplayValidationError("Trajectory must be an .npz or .csv file")
    validate_recorded_trajectory(trajectory)
    return trajectory


def validate_recorded_trajectory(trajectory: RecordedTrajectory) -> None:
    sample_count = len(trajectory.time_s)
    if sample_count < 2:
        raise ReplayValidationError("Trajectory must contain at least two samples")
    if trajectory.joint_rad.shape != (sample_count, JOINT_COUNT):
        raise ReplayValidationError(
            "joint_rad must have shape (sample_count, 6); got "
            f"{trajectory.joint_rad.shape}"
        )
    if trajectory.sdk_error_code.shape != (sample_count,):
        raise ReplayValidationError("sdk_error_code length does not match joints")
    if trajectory.arm_current_status.shape != (sample_count,):
        raise ReplayValidationError(
            "arm_current_status length does not match joints"
        )
    if not np.all(np.isfinite(trajectory.time_s)):
        raise ReplayValidationError("Trajectory timestamps contain non-finite values")
    if not np.all(np.isfinite(trajectory.joint_rad)):
        raise ReplayValidationError("Trajectory joints contain non-finite values")
    if np.any(np.diff(trajectory.time_s) <= 0.0):
        raise ReplayValidationError(
            "Trajectory timestamps must be strictly increasing"
        )
    if trajectory.duration_s <= 0.0:
        raise ReplayValidationError("Trajectory duration must be positive")
    nonzero_errors = np.flatnonzero(trajectory.sdk_error_code != 0)
    if len(nonzero_errors):
        first = int(nonzero_errors[0])
        code = int(trajectory.sdk_error_code[first])
        raise ReplayValidationError(
            f"Recorded SDK error code {code} at sample {first}; refusing replay"
        )


def _trim_idle_indices(
    trajectory: RecordedTrajectory,
    threshold_rad: float,
    padding_s: float,
) -> tuple[int, int]:
    step_magnitude = np.max(np.abs(np.diff(trajectory.joint_rad, axis=0)), axis=1)
    moving_edges = np.flatnonzero(step_magnitude >= threshold_rad)
    if not len(moving_edges):
        raise ReplayValidationError(
            "No motion exceeds the idle-trimming threshold"
        )

    motion_start_time = trajectory.time_s[int(moving_edges[0])]
    motion_end_time = trajectory.time_s[int(moving_edges[-1]) + 1]
    start_time = max(float(trajectory.time_s[0]), motion_start_time - padding_s)
    end_time = min(float(trajectory.time_s[-1]), motion_end_time + padding_s)
    start_index = int(np.searchsorted(trajectory.time_s, start_time, side="left"))
    end_index = int(np.searchsorted(trajectory.time_s, end_time, side="right"))
    return start_index, max(start_index + 2, end_index)


def _apply_joint_limits(
    joints: np.ndarray,
    tolerance_rad: float,
) -> tuple[np.ndarray, int, float]:
    limits = np.asarray(RM65B.JOINT_LIMITS, dtype=np.float64)
    lower = limits[:, 0]
    upper = limits[:, 1]
    lower_excess = np.maximum(lower - joints, 0.0)
    upper_excess = np.maximum(joints - upper, 0.0)
    max_excess = float(max(np.max(lower_excess), np.max(upper_excess)))
    if max_excess > tolerance_rad:
        row, joint = np.unravel_index(
            int(np.argmax(np.maximum(lower_excess, upper_excess))), joints.shape
        )
        value = float(joints[row, joint])
        raise ReplayValidationError(
            f"Sample {row}, joint {joint + 1} is {value:.6f} rad, exceeding "
            f"the configured limit by more than {tolerance_rad:.6f} rad"
        )

    clipped = np.clip(joints, lower, upper)
    clipped_count = int(np.count_nonzero(clipped != joints))
    return clipped, clipped_count, max_excess


def prepare_trajectory(
    trajectory: RecordedTrajectory,
    *,
    keep_idle: bool,
    idle_threshold_rad: float,
    idle_padding_s: float,
    limit_tolerance_rad: float,
    output_hz: float,
    speed_scale: float,
    max_joint_speed_rad_s: float,
    max_joint_step_rad: float,
) -> PreparedTrajectory:
    if keep_idle:
        start_index, end_index = 0, len(trajectory.time_s)
    else:
        start_index, end_index = _trim_idle_indices(
            trajectory, idle_threshold_rad, idle_padding_s
        )

    source_time = trajectory.time_s[start_index:end_index].copy()
    source_time -= source_time[0]
    source_joints = trajectory.joint_rad[start_index:end_index].copy()
    source_joints, clipped_count, max_excess = _apply_joint_limits(
        source_joints, limit_tolerance_rad
    )

    source_duration = float(source_time[-1])
    replay_duration = source_duration / speed_scale
    command_period_s = 1.0 / output_hz
    if replay_duration * output_hz > 1_000_000:
        raise ReplayValidationError(
            "Prepared trajectory would exceed 1,000,000 commands; increase "
            "--speed-scale or reduce --output-hz"
        )
    regular_count = int(math.floor(replay_duration / command_period_s)) + 1
    replay_time = np.arange(regular_count, dtype=np.float64) * command_period_s
    if replay_time[-1] < replay_duration:
        replay_time = np.append(replay_time, replay_duration)
    source_query_time = np.minimum(replay_time * speed_scale, source_duration)

    replay_joints = np.empty((len(replay_time), JOINT_COUNT), dtype=np.float64)
    for joint_index in range(JOINT_COUNT):
        replay_joints[:, joint_index] = np.interp(
            source_query_time,
            source_time,
            source_joints[:, joint_index],
        )

    time_step = np.diff(replay_time)
    joint_step = np.abs(np.diff(replay_joints, axis=0))
    joint_speed = joint_step / time_step[:, None]
    observed_max_speed = float(np.max(joint_speed))
    observed_max_step = float(np.max(joint_step))
    if observed_max_speed > max_joint_speed_rad_s + 1e-12:
        raise ReplayValidationError(
            f"Prepared trajectory reaches {observed_max_speed:.4f} rad/s, "
            f"above --max-joint-speed-deg-s "
            f"({math.degrees(max_joint_speed_rad_s):.2f} deg/s)"
        )
    if observed_max_step > max_joint_step_rad + 1e-12:
        raise ReplayValidationError(
            f"Prepared command step reaches {observed_max_step:.5f} rad, "
            f"above --max-joint-step-deg "
            f"({math.degrees(max_joint_step_rad):.2f} deg)"
        )

    return PreparedTrajectory(
        source_path=trajectory.source_path,
        time_s=replay_time,
        joint_rad=replay_joints,
        source_start_index=start_index,
        source_end_index=end_index - 1,
        clipped_value_count=clipped_count,
        max_limit_excess_rad=max_excess,
    )


def _format_vector_degrees(joints: np.ndarray) -> str:
    values = np.degrees(np.asarray(joints, dtype=np.float64))
    return "[" + ", ".join(f"{value:8.3f}" for value in values) + "] deg"


def print_summary(
    recorded: RecordedTrajectory,
    prepared: PreparedTrajectory,
    speed_scale: float,
) -> None:
    unique_status, status_counts = np.unique(
        recorded.arm_current_status, return_counts=True
    )
    status_text = ", ".join(
        f"{int(status)}:{ARM_STATUS_NAMES.get(int(status), 'unknown')}="
        f"{int(count)}"
        for status, count in zip(unique_status, status_counts)
    )

    dt = np.diff(prepared.time_s)
    joint_delta = np.diff(prepared.joint_rad, axis=0)
    max_speed = np.max(np.abs(joint_delta) / dt[:, None], axis=0)
    max_step = np.max(np.abs(joint_delta), axis=0)

    print(f"Source:                 {recorded.source_path}")
    print(f"Recorded samples:       {len(recorded.time_s)}")
    print(
        "Merged duplicate times: "
        f"{recorded.merged_duplicate_timestamps}"
    )
    print(f"Recorded duration:      {recorded.duration_s:.3f} s")
    print(f"Recorded arm statuses:  {status_text}")
    print(
        "Selected source range:  "
        f"{prepared.source_start_index}..{prepared.source_end_index}"
    )
    print(f"Replay samples:         {len(prepared.time_s)}")
    print(f"Replay duration:        {prepared.duration_s:.3f} s")
    print(f"Speed scale:            {speed_scale:.3f}x")
    print(f"Start joints:           {_format_vector_degrees(prepared.joint_rad[0])}")
    print(f"End joints:             {_format_vector_degrees(prepared.joint_rad[-1])}")
    print(f"Max speed per joint:    {_format_vector_degrees(max_speed)}/s")
    print(f"Max step per command:   {_format_vector_degrees(max_step)}")
    print(f"Clipped joint values:   {prepared.clipped_value_count}")
    if prepared.clipped_value_count:
        print(
            "Maximum limit excess:  "
            f"{prepared.max_limit_excess_rad:.6f} rad "
            f"({math.degrees(prepared.max_limit_excess_rad):.4f} deg)"
        )


def _wait_until(deadline_ns: int) -> int:
    while True:
        remaining_ns = deadline_ns - time.perf_counter_ns()
        if remaining_ns <= 0:
            return -remaining_ns
        if remaining_ns > 2_000_000:
            time.sleep((remaining_ns - 1_000_000) / 1_000_000_000.0)
        else:
            time.sleep(0)


def _require_replay_confirmation(
    current_joints: np.ndarray,
    start_joints: np.ndarray,
    will_move_to_start: bool,
) -> None:
    delta = np.abs(start_joints - current_joints)
    print()
    print("REAL ROBOT EXECUTION REQUESTED")
    print(f"Current joints: {_format_vector_degrees(current_joints)}")
    print(f"Target start:   {_format_vector_degrees(start_joints)}")
    print(f"Absolute delta: {_format_vector_degrees(delta)}")
    if will_move_to_start:
        print("The robot will first run a planned low-speed move to this start pose.")
    else:
        print("The current pose is already within the configured start tolerance.")
    print("Stop every other robot-control process and keep the E-stop accessible.")
    response = input("Type REPLAY to authorize physical motion: ").strip()
    if response != "REPLAY":
        raise RuntimeError("Replay cancelled; confirmation text did not match")


def execute_replay(
    prepared: PreparedTrajectory,
    *,
    robot_ip: str,
    robot_port: int,
    move_to_start: bool,
    approach_speed_percent: int,
    start_tolerance_rad: float,
    output_hz: float,
    max_schedule_lag_s: float,
    hold_seconds: float,
) -> None:
    robot: Optional[RM65B] = None
    motion_started = False
    primary_error: Optional[BaseException] = None
    try:
        robot = RM65B(robot_ip, port=robot_port)
        current_joints = np.asarray(robot.get_joint_values(), dtype=np.float64)
        start_joints = prepared.joint_rad[0]
        start_delta = np.max(np.abs(start_joints - current_joints))
        requires_approach = bool(start_delta > start_tolerance_rad)
        if requires_approach and not move_to_start:
            raise RuntimeError(
                "Current robot pose is not at the recorded start pose: maximum "
                f"joint difference is {math.degrees(start_delta):.3f} deg. "
                "Inspect the dry run, then add --move-to-start if the planned "
                "approach is safe."
            )

        _require_replay_confirmation(
            current_joints, start_joints, requires_approach
        )
        motion_started = True
        if requires_approach:
            robot.move_joint(
                start_joints,
                speed_percent=approach_speed_percent,
                asynchronous=False,
            )
            reached = np.asarray(robot.get_joint_values(), dtype=np.float64)
            reached_error = float(np.max(np.abs(reached - start_joints)))
            if reached_error > start_tolerance_rad:
                raise RuntimeError(
                    "Robot did not reach the recorded start pose: maximum "
                    f"remaining error is {math.degrees(reached_error):.3f} deg"
                )

        print("Replay begins in 3 seconds. Use the hardware E-stop if necessary.")
        for remaining in (3, 2, 1):
            print(f"{remaining}...")
            time.sleep(1.0)

        command_period_s = 1.0 / output_hz
        start_ns = time.perf_counter_ns()
        for index, (scheduled_s, joints) in enumerate(
            zip(prepared.time_s, prepared.joint_rad)
        ):
            deadline_ns = start_ns + int(round(float(scheduled_s) * 1e9))
            lag_ns = _wait_until(deadline_ns)
            if lag_ns > int(max_schedule_lag_s * 1e9):
                raise RuntimeError(
                    f"Replay scheduler lagged by {lag_ns / 1e6:.1f} ms at "
                    f"command {index}; stopping"
                )
            robot.servo_joint(joints, time=command_period_s)

        hold_count = int(math.ceil(hold_seconds * output_hz))
        final_joints = prepared.joint_rad[-1]
        for _ in range(hold_count):
            robot.servo_joint(final_joints, time=command_period_s)
            time.sleep(command_period_s)
        print("Replay completed.")
    except KeyboardInterrupt as exc:
        primary_error = exc
        print("\nReplay interrupted; requesting a controlled slow stop.")
        raise
    except BaseException as exc:
        primary_error = exc
        raise
    finally:
        if robot is not None:
            try:
                if motion_started:
                    robot.stop_script()
                else:
                    robot.close()
            except BaseException as cleanup_error:
                if primary_error is None:
                    raise
                print(
                    f"Warning: robot cleanup failed: {cleanup_error}",
                    file=sys.stderr,
                )


def positive_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("value must be a positive number")
    return result


def nonnegative_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return result


def percentage(value: str) -> int:
    result = int(value)
    if not 1 <= result <= 100:
        raise argparse.ArgumentTypeError("value must be in 1..100")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate and replay an RM65-B CSV/NPZ state log. Dry run is the "
            "default; --execute and interactive confirmation are required for "
            "physical motion."
        )
    )
    parser.add_argument("trajectory", type=Path, help="RM65_state .npz or .csv")
    parser.add_argument("--robot-ip", default="192.168.1.18")
    parser.add_argument("--robot-port", type=int, default=8080)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Connect to the robot and request physical replay.",
    )
    parser.add_argument(
        "--move-to-start",
        action="store_true",
        help="Allow a planned low-speed move to the recorded start pose.",
    )
    parser.add_argument(
        "--speed-scale",
        type=positive_float,
        default=0.25,
        help="Replay speed relative to the recording (default: 0.25).",
    )
    parser.add_argument(
        "--output-hz",
        type=positive_float,
        default=DEFAULT_OUTPUT_HZ,
        help="Interpolated CAN-FD command rate (default: 50 Hz).",
    )
    parser.add_argument(
        "--keep-idle",
        action="store_true",
        help="Keep stationary time at the beginning and end of the log.",
    )
    parser.add_argument(
        "--idle-threshold-deg",
        type=positive_float,
        default=0.1,
        help="Joint step used to detect and trim stationary log sections.",
    )
    parser.add_argument(
        "--idle-padding",
        type=nonnegative_float,
        default=0.5,
        help="Seconds retained before and after detected motion.",
    )
    parser.add_argument(
        "--limit-tolerance-deg",
        type=nonnegative_float,
        default=0.25,
        help="Small recorded limit excess allowed before clipping.",
    )
    parser.add_argument(
        "--max-joint-speed-deg-s",
        type=positive_float,
        default=20.0,
        help="Dry-run and execution speed ceiling per joint.",
    )
    parser.add_argument(
        "--max-joint-step-deg",
        type=positive_float,
        default=1.0,
        help="Maximum allowed change in one interpolated command.",
    )
    parser.add_argument(
        "--approach-speed-percent",
        type=percentage,
        default=5,
        help="Planned speed percentage for --move-to-start.",
    )
    parser.add_argument(
        "--start-tolerance-deg",
        type=positive_float,
        default=0.25,
        help="Maximum start-pose error allowed without --move-to-start.",
    )
    parser.add_argument(
        "--max-schedule-lag-ms",
        type=positive_float,
        default=100.0,
        help="Abort if the command scheduler falls this far behind.",
    )
    parser.add_argument(
        "--hold-seconds",
        type=nonnegative_float,
        default=0.5,
        help="Hold the final target after replay (default: 0.5 s).",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if args.speed_scale > 1.0:
        raise ReplayValidationError(
            "--speed-scale above 1.0 is disabled for safety"
        )
    if args.output_hz > 100.0:
        raise ReplayValidationError("--output-hz above 100 is disabled for safety")
    if not 0 < int(args.robot_port) <= 65535:
        raise ReplayValidationError("--robot-port must be in 1..65535")

    recorded = load_recorded_trajectory(args.trajectory)
    prepared = prepare_trajectory(
        recorded,
        keep_idle=args.keep_idle,
        idle_threshold_rad=math.radians(args.idle_threshold_deg),
        idle_padding_s=args.idle_padding,
        limit_tolerance_rad=math.radians(args.limit_tolerance_deg),
        output_hz=args.output_hz,
        speed_scale=args.speed_scale,
        max_joint_speed_rad_s=math.radians(args.max_joint_speed_deg_s),
        max_joint_step_rad=math.radians(args.max_joint_step_deg),
    )
    print_summary(recorded, prepared, args.speed_scale)

    if not args.execute:
        print()
        print("DRY RUN ONLY: the robot was not contacted and no motion was sent.")
        print("Review the summary, then add --execute when the workspace is safe.")
        if not args.move_to_start:
            print(
                "If the robot is not already at the recorded start pose, "
                "--move-to-start will also be required."
            )
        return 0

    execute_replay(
        prepared,
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        move_to_start=args.move_to_start,
        approach_speed_percent=args.approach_speed_percent,
        start_tolerance_rad=math.radians(args.start_tolerance_deg),
        output_hz=args.output_hz,
        max_schedule_lag_s=args.max_schedule_lag_ms / 1000.0,
        hold_seconds=args.hold_seconds,
    )
    return 0


if __name__ == "__main__":
    try:
        exit_code = main()
    except ReplayValidationError as exc:
        print(f"Trajectory validation failed: {exc}", file=sys.stderr)
        raise SystemExit(2)
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"Replay failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
    else:
        raise SystemExit(exit_code)
