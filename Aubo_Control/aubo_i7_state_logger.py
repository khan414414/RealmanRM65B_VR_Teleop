"""Record AUBO i7 realtime state to CSV and compressed NPZ files.

The robot controller exposes state through synchronous ARCS RPC queries, so
this logger polls ``getJointPositions``/``getTcpPose``/``getJointSpeeds`` at a
configurable cycle instead of subscribing to a UDP push.  Each sample stores
both a monotonic and a wall-clock timestamp; ``device_time_ns`` is derived from
the sample index and the configured cycle, mirroring the schema used by the
previous RM65 logs so downstream tooling keeps working.

Run from the repository root, for example::

    python -m Aubo_Control.aubo_i7_state_logger
    python -m Aubo_Control.aubo_i7_state_logger --duration 60
    python -m Aubo_Control.aubo_i7_state_logger --robot-ip 192.168.1.100

Press Ctrl+C to stop and save.  The script only reads robot state; it does not
send motion commands.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np

from Aubo_Control.aubo_control import AuboI7


JOINT_COUNT = 6
DEVICE_TIME_SOURCE = "estimated_from_poll_cycle"


CSV_COLUMNS = (
    "device_time_ns",
    "device_time_is_estimated",
    "device_sample_index",
    "estimated_missing_cycles",
    "mono_ns",
    "system_time_ns",
    "timestamp_system_ns",
    "timestamp_monotonic_ns",
    "timestamp_relative_ns",
    "poll_error",
    "joint_1_rad",
    "joint_2_rad",
    "joint_3_rad",
    "joint_4_rad",
    "joint_5_rad",
    "joint_6_rad",
    "joint_speed_1_rad_s",
    "joint_speed_2_rad_s",
    "joint_speed_3_rad_s",
    "joint_speed_4_rad_s",
    "joint_speed_5_rad_s",
    "joint_speed_6_rad_s",
    "tcp_base_x_m",
    "tcp_base_y_m",
    "tcp_base_z_m",
    "tcp_base_qx",
    "tcp_base_qy",
    "tcp_base_qz",
    "tcp_base_qw",
)


def _rotation_vector_to_quaternion_xyzw(
    rotation_vector: Sequence[float],
) -> tuple[float, float, float, float]:
    """Convert an ARCS rotation vector to an xyzw quaternion."""

    vector = np.asarray(rotation_vector, dtype=np.float64)
    angle = float(np.linalg.norm(vector))
    if angle < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    axis = vector / angle
    half = 0.5 * angle
    return (
        float(axis[0] * math.sin(half)),
        float(axis[1] * math.sin(half)),
        float(axis[2] * math.sin(half)),
        float(math.cos(half)),
    )


@dataclass(frozen=True, slots=True)
class StoredStateSample:
    """One polled state sample with estimated device timing."""

    device_time_ns: int
    device_sample_index: int
    estimated_missing_cycles: int
    mono_ns: int
    system_time_ns: int
    poll_error: bool
    joint_rad: tuple[float, ...]
    joint_speed_rad_s: tuple[float, ...]
    tcp_position_base_m: tuple[float, float, float]
    tcp_quaternion_base_xyzw: tuple[float, float, float, float]


class AuboI7StateLogger:
    """Poll ARCS state at a fixed cycle and write CSV plus NPZ outputs."""

    def __init__(
        self,
        robot_ip: str,
        port: int,
        cycle_ms: int,
        output_dir: Path,
        queue_size: int = 10_000,
        flush_every: int = 50,
        robot_name: Optional[str] = None,
        username: str = AuboI7.DEFAULT_USERNAME,
        password: str = AuboI7.DEFAULT_PASSWORD,
    ) -> None:
        if cycle_ms <= 0:
            raise ValueError("cycle_ms must be positive")
        if flush_every <= 0:
            raise ValueError("flush_every must be positive")
        if queue_size <= 0:
            raise ValueError("queue_size must be positive")

        self.robot_ip = robot_ip
        self.port = int(port)
        self.cycle_ms = int(cycle_ms)
        self.cycle_ns = self.cycle_ms * 1_000_000
        self.output_dir = output_dir
        self.flush_every = int(flush_every)
        self.robot_name = robot_name
        self.username = username
        self.password = password
        self.max_samples = int(queue_size)

        self._robot: Optional[AuboI7] = None
        self._samples: list[StoredStateSample] = []
        self._poll_error_count = 0
        self._first_mono_ns: Optional[int] = None
        self._previous_mono_ns: Optional[int] = None
        self._device_time_ns = 0

        self.output_dir.mkdir(parents=True, exist_ok=True)
        started_ns = time.time_ns()
        started = datetime.fromtimestamp(started_ns // 1_000_000_000)
        suffix = started_ns % 1_000_000_000
        stem = f"AUBO_i7_state_{started:%Y%m%d_%H%M%S}_{suffix:09d}"
        self.csv_path = self.output_dir / f"{stem}.csv"
        self.npz_path = self.output_dir / f"{stem}.npz"

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    @property
    def poll_error_count(self) -> int:
        return self._poll_error_count

    def _poll_once(self) -> tuple[bool, tuple[float, ...], tuple[float, ...], tuple[float, ...], tuple[float, float, float, float]]:
        """Return ``(ok, joints, speeds, position, quaternion)`` for one poll.

        A failed poll still advances the sample index with the previous values
        repeated and ``poll_error`` set, so gaps stay visible in the log.
        """

        try:
            joints = tuple(
                float(value) for value in self._robot.get_joint_values()
            )
            speeds = tuple(
                float(value) for value in self._robot.get_joint_speed()
            )
            pose = tuple(float(value) for value in self._robot.get_tool_pose())
            quaternion = _rotation_vector_to_quaternion_xyzw(pose[3:])
            return (
                True,
                joints,
                speeds,
                (pose[0], pose[1], pose[2]),
                quaternion,
            )
        except (RuntimeError, ValueError) as exc:
            self._poll_error_count += 1
            print(
                f"Warning: state poll failed ({exc}); sample marked invalid.",
                file=sys.stderr,
            )
            if self._samples:
                previous = self._samples[-1]
                return (
                    False,
                    previous.joint_rad,
                    previous.joint_speed_rad_s,
                    previous.tcp_position_base_m,
                    previous.tcp_quaternion_base_xyzw,
                )
            return (
                False,
                (0.0,) * JOINT_COUNT,
                (0.0,) * JOINT_COUNT,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )

    def _add_timing(
        self,
        sample_index: int,
        mono_ns: int,
        system_time_ns: int,
    ) -> tuple[int, int]:
        """Advance the estimated device clock; return (device_ns, missing)."""

        if self._first_mono_ns is None:
            self._first_mono_ns = mono_ns
            self._previous_mono_ns = mono_ns
            missing_cycles = 0
            self._device_time_ns = 0
        else:
            assert self._previous_mono_ns is not None
            elapsed_ns = max(0, mono_ns - self._previous_mono_ns)
            elapsed_cycles = max(1, round(elapsed_ns / self.cycle_ns))
            missing_cycles = max(0, elapsed_cycles - 1)
            self._device_time_ns += elapsed_cycles * self.cycle_ns
            self._previous_mono_ns = mono_ns
        return self._device_time_ns, missing_cycles

    def _csv_row(self, sample: StoredStateSample) -> tuple[Any, ...]:
        assert self._first_mono_ns is not None
        return (
            sample.device_time_ns,
            1,
            sample.device_sample_index,
            sample.estimated_missing_cycles,
            sample.mono_ns,
            sample.system_time_ns,
            sample.system_time_ns,
            sample.mono_ns,
            sample.mono_ns - self._first_mono_ns,
            int(sample.poll_error),
            *sample.joint_rad,
            *sample.joint_speed_rad_s,
            *sample.tcp_position_base_m,
            *sample.tcp_quaternion_base_xyzw,
        )

    def connect_and_start(self) -> None:
        """Connect to the controller and log the resolved configuration."""

        self._robot = AuboI7(
            self.robot_ip,
            port=self.port,
            robot_name=self.robot_name,
            username=self.username,
            password=self.password,
        )
        try:
            limits = self._robot.get_joint_limits()
            print(f"Joint limits (rad): {limits}")
        except (RuntimeError, ValueError) as exc:
            print(f"Warning: could not read joint limits ({exc}).", file=sys.stderr)

    def record(
        self,
        duration: Optional[float],
        startup_timeout: float,
        poll_timeout: float = 2.0,
    ) -> None:
        """Poll state until Ctrl+C or the optional duration elapses."""

        if self._robot is None:
            raise RuntimeError("call connect_and_start() before record()")

        cycle_s = self.cycle_ms / 1000.0
        started_mono = time.monotonic()
        sample_index = 0

        with self.csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(CSV_COLUMNS)
            csv_file.flush()

            while True:
                next_deadline = started_mono + (sample_index + 1) * cycle_s
                sleep_s = next_deadline - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)

                mono_ns = time.monotonic_ns()
                system_time_ns = time.time_ns()
                ok, joints, speeds, position, quaternion = self._poll_once()
                device_time_ns, missing_cycles = self._add_timing(
                    sample_index, mono_ns, system_time_ns
                )
                sample = StoredStateSample(
                    device_time_ns=device_time_ns,
                    device_sample_index=sample_index,
                    estimated_missing_cycles=missing_cycles,
                    mono_ns=mono_ns,
                    system_time_ns=system_time_ns,
                    poll_error=not ok,
                    joint_rad=joints,
                    joint_speed_rad_s=speeds,
                    tcp_position_base_m=position,
                    tcp_quaternion_base_xyzw=quaternion,
                )
                self._samples.append(sample)
                writer.writerow(self._csv_row(sample))
                if len(self._samples) % self.flush_every == 0:
                    csv_file.flush()

                sample_index += 1
                if len(self._samples) >= self.max_samples:
                    break
                if (
                    duration is not None
                    and mono_ns - int(started_mono * 1_000_000_000)
                    >= int(duration * 1_000_000_000)
                ):
                    break
                if time.monotonic() - started_mono > startup_timeout and not self._samples:
                    raise TimeoutError(
                        "No state could be read from the controller. Check "
                        "--robot-ip, ARCS availability and credentials."
                    )
                del poll_timeout

            csv_file.flush()

    def save_npz(self) -> None:
        """Save arrays with keys and dtypes compatible with prior NPZ logs."""

        if not self.csv_path.exists():
            with self.csv_path.open("w", newline="", encoding="utf-8") as csv_file:
                csv.writer(csv_file).writerow(CSV_COLUMNS)

        samples = self._samples
        joint_rad = np.asarray(
            [s.joint_rad for s in samples], dtype=np.float32
        ).reshape(len(samples), JOINT_COUNT)
        joint_speed = np.asarray(
            [s.joint_speed_rad_s for s in samples], dtype=np.float32
        ).reshape(len(samples), JOINT_COUNT)
        tcp_position = np.asarray(
            [s.tcp_position_base_m for s in samples], dtype=np.float32
        ).reshape(len(samples), 3)
        tcp_quaternion = np.asarray(
            [s.tcp_quaternion_base_xyzw for s in samples], dtype=np.float32
        ).reshape(len(samples), 4)

        np.savez_compressed(
            self.npz_path,
            device_time_ns=np.asarray(
                [s.device_time_ns for s in samples], dtype=np.int64
            ),
            device_time_is_estimated=np.ones(len(samples), dtype=np.uint8),
            device_sample_index=np.asarray(
                [s.device_sample_index for s in samples], dtype=np.int64
            ),
            estimated_missing_cycles=np.asarray(
                [s.estimated_missing_cycles for s in samples], dtype=np.int32
            ),
            mono_ns=np.asarray([s.mono_ns for s in samples], dtype=np.int64),
            system_time_ns=np.asarray(
                [s.system_time_ns for s in samples], dtype=np.int64
            ),
            poll_error=np.asarray(
                [s.poll_error for s in samples], dtype=np.uint8
            ),
            joint_rad=joint_rad,
            joint_speed_rad_s=joint_speed,
            tcp_position_base_m=tcp_position,
            tcp_quaternion_base_xyzw=tcp_quaternion,
            device_push_cycle_ns=np.asarray(self.cycle_ns, dtype=np.int64),
            device_time_source=np.asarray(DEVICE_TIME_SOURCE, dtype="U40"),
            native_controller_timestamp_available=np.asarray(False, dtype=np.bool_),
            poll_error_count=np.asarray(self._poll_error_count, dtype=np.int64),
        )

    def close(self) -> None:
        if self._robot is not None:
            try:
                self._robot.close()
            finally:
                self._robot = None


def positive_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("value must be a positive number")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record AUBO i7 joint/TCP state to CSV and compressed NPZ."
        )
    )
    parser.add_argument("--robot-ip", default=AuboI7.DEFAULT_IP)
    parser.add_argument("--port", type=int, default=AuboI7.DEFAULT_PORT)
    parser.add_argument("--robot-name", default=None)
    parser.add_argument("--username", default=AuboI7.DEFAULT_USERNAME)
    parser.add_argument("--password", default=AuboI7.DEFAULT_PASSWORD)
    parser.add_argument(
        "--cycle-ms",
        type=int,
        default=20,
        help="State poll cycle in milliseconds.",
    )
    parser.add_argument(
        "--duration",
        type=positive_float,
        help="Recording duration in seconds; otherwise record until Ctrl+C.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(r"C:\AUBOLogs"),
    )
    parser.add_argument("--queue-size", type=int, default=10_000)
    parser.add_argument("--startup-timeout", type=positive_float, default=5.0)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)

    logger = AuboI7StateLogger(
        robot_ip=args.robot_ip,
        port=args.port,
        cycle_ms=args.cycle_ms,
        output_dir=args.output_dir.expanduser().resolve(),
        queue_size=args.queue_size,
        robot_name=args.robot_name,
        username=args.username,
        password=args.password,
    )

    print(f"Robot:     {args.robot_ip}:{args.port}")
    print(f"Cycle:     {args.cycle_ms} ms")
    print(f"CSV:       {logger.csv_path}")
    print(f"NPZ:       {logger.npz_path}")

    exit_code = 0
    try:
        logger.connect_and_start()
        print("Recording robot state. Press Ctrl+C to stop and save.")
        logger.record(args.duration, args.startup_timeout)
    except KeyboardInterrupt:
        print("\nStopping recording...")
    except BaseException as exc:
        exit_code = 1
        print(f"Logger error: {exc}", file=sys.stderr)
    finally:
        try:
            logger.save_npz()
        except BaseException as exc:
            exit_code = 1
            print(f"Failed to save NPZ: {exc}", file=sys.stderr)
        finally:
            logger.close()

    if logger.sample_count:
        print(f"Saved {logger.sample_count} samples.")
        print(f"Poll errors: {logger.poll_error_count}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
