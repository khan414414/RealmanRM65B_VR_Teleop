"""Record RM65-B realtime state to CSV and compressed NPZ files.

The output schema matches the supplied ``RM65_state_*.csv/.npz`` examples.
The robot controller does not expose a native timestamp in this realtime-push
structure, so ``device_time_ns`` is estimated from the configured push cycle.

Run from the repository root, for example::

    python -m Realman_Control.rm65_state_logger
    python -m Realman_Control.rm65_state_logger --duration 60
    python -m Realman_Control.rm65_state_logger --local-ip 192.168.1.100

Press Ctrl+C to stop and save.  The script only reads robot state; it does not
send motion commands.  It temporarily changes the controller's UDP realtime
push destination and restores the previous configuration before disconnecting.
"""

from __future__ import annotations

import argparse
import csv
import math
import queue
import socket
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np
from Realman_Control.legacy_sdk import RealtimeCallback

from Realman_Control.realman_control import RM65B


JOINT_COUNT = 6
DEVICE_TIME_SOURCE = "estimated_from_realtime_push_cycle"

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
    "sdk_error_code",
    "arm_current_status",
    "joint_1_rad",
    "joint_2_rad",
    "joint_3_rad",
    "joint_4_rad",
    "joint_5_rad",
    "joint_6_rad",
    "tcp_base_x_m",
    "tcp_base_y_m",
    "tcp_base_z_m",
    "tcp_base_qx",
    "tcp_base_qy",
    "tcp_base_qz",
    "tcp_base_qw",
)


@dataclass(frozen=True, slots=True)
class RawStateSample:
    """One callback sample copied out of the SDK-owned ctypes structure."""

    sample_index: int
    mono_ns: int
    system_time_ns: int
    sdk_error_code: int
    arm_current_status: int
    joint_rad: tuple[float, float, float, float, float, float]
    tcp_position_base_m: tuple[float, float, float]
    tcp_quaternion_base_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class StoredStateSample:
    """A sample with the estimated controller time fields added."""

    device_time_ns: int
    device_sample_index: int
    estimated_missing_cycles: int
    mono_ns: int
    system_time_ns: int
    sdk_error_code: int
    arm_current_status: int
    joint_rad: tuple[float, float, float, float, float, float]
    tcp_position_base_m: tuple[float, float, float]
    tcp_quaternion_base_xyzw: tuple[float, float, float, float]


class RM65StateLogger:
    """Collect realtime UDP state callbacks and write the reference formats."""

    def __init__(
        self,
        robot_ip: str,
        robot_port: int,
        local_ip: str,
        udp_port: int,
        cycle_ms: int,
        output_dir: Path,
        queue_size: int,
        flush_every: int = 50,
    ) -> None:
        if cycle_ms <= 0 or cycle_ms % 5 != 0:
            raise ValueError("cycle_ms must be a positive multiple of 5")
        if not 0 < robot_port <= 65535 or not 0 < udp_port <= 65535:
            raise ValueError("robot_port and udp_port must be in 1..65535")
        if queue_size <= 0:
            raise ValueError("queue_size must be positive")

        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.local_ip = local_ip
        self.udp_port = udp_port
        self.cycle_ms = cycle_ms
        self.cycle_ns = cycle_ms * 1_000_000
        self.output_dir = output_dir
        self.flush_every = max(1, int(flush_every))

        self._queue: queue.Queue[RawStateSample] = queue.Queue(queue_size)
        self._callback_count = 0
        self._queue_overflow_count = 0
        self._callback_error_count = 0
        self._last_callback_error: Optional[str] = None
        self._accept_callbacks = False
        self._callback: Any = None
        self._robot: Optional[RM65B] = None
        self._previous_push_config: Optional[dict[str, Any]] = None
        self._configured_push = False

        self._samples: list[StoredStateSample] = []
        self._first_mono_ns: Optional[int] = None
        self._previous_mono_ns: Optional[int] = None
        self._device_time_ns = 0

        self.output_dir.mkdir(parents=True, exist_ok=True)
        started_ns = time.time_ns()
        started = datetime.fromtimestamp(started_ns // 1_000_000_000)
        suffix = started_ns % 1_000_000_000
        stem = f"RM65_state_{started:%Y%m%d_%H%M%S}_{suffix:09d}"
        self.csv_path = self.output_dir / f"{stem}.csv"
        self.npz_path = self.output_dir / f"{stem}.npz"

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    @property
    def queue_overflow_count(self) -> int:
        return self._queue_overflow_count

    def _on_realtime_state(self, state: Any) -> None:
        """Copy the legacy by-value callback; do not let exceptions cross ctypes."""

        if not self._accept_callbacks:
            return

        sample_index = self._callback_count
        self._callback_count += 1
        mono_ns = time.monotonic_ns()
        system_time_ns = time.time_ns()

        try:
            joint_degrees = state.joint_status.joint_position
            joint_rad = tuple(
                math.radians(float(joint_degrees[index]))
                for index in range(JOINT_COUNT)
            )
            position = state.waypoint.position
            quaternion = state.waypoint.quaternion
            sample = RawStateSample(
                sample_index=sample_index,
                mono_ns=mono_ns,
                system_time_ns=system_time_ns,
                sdk_error_code=int(state.errCode),
                arm_current_status=-1,  # Unavailable in the API 4.2.8 callback ABI.
                joint_rad=joint_rad,  # type: ignore[arg-type]
                tcp_position_base_m=(
                    float(position.x),
                    float(position.y),
                    float(position.z),
                ),
                # The SDK exposes wxyz; the reference files store xyzw.
                tcp_quaternion_base_xyzw=(
                    float(quaternion.x),
                    float(quaternion.y),
                    float(quaternion.z),
                    float(quaternion.w),
                ),
            )
            self._queue.put_nowait(sample)
        except queue.Full:
            self._queue_overflow_count += 1
        except BaseException as exc:
            self._callback_error_count += 1
            self._last_callback_error = f"{type(exc).__name__}: {exc}"

    @staticmethod
    def _check_sdk_code(code: int, operation: str) -> None:
        if int(code) != 0:
            raise RuntimeError(f"{operation} failed with RealMan SDK code {code}")

    @staticmethod
    def _push_config_from_dict(config: dict[str, Any]) -> dict[str, Any]:
        ip = config["ip"]
        if isinstance(ip, bytes):
            ip = ip.decode("ascii")
        return {
            "cycle": int(config["cycle"]),
            "enable": bool(config["enable"]),
            "port": int(config["port"]),
            "force_coordinate": int(config.get("force_coordinate", -1)),
            "ip": str(ip),
        }

    def connect_and_start(self) -> None:
        """Preserve the legacy UDP config and register the 4.2.8 callback ABI."""

        self._robot = RM65B(self.robot_ip, port=self.robot_port)
        arm = self._robot.arm
        old_code, old_config = arm.Get_Realtime_Push()
        self._check_sdk_code(old_code, "Get_Realtime_Push")
        self._previous_push_config = self._push_config_from_dict(old_config)

        self._callback = RealtimeCallback(self._on_realtime_state)
        self._accept_callbacks = True
        arm.Realtime_Arm_Joint_State(self._callback)

        # The wire protocol cycle is a count of 5 ms ticks, not milliseconds.
        push_config = {
            "cycle": self.cycle_ms // 5,
            "enable": True,
            "port": self.udp_port,
            "force_coordinate": -1,
            "ip": self.local_ip,
        }
        # Restore even if the controller applied the setting but its reply was lost.
        self._configured_push = True
        self._check_sdk_code(arm.Set_Realtime_Push(push_config), "Set_Realtime_Push")

    def _add_timing(self, sample: RawStateSample) -> StoredStateSample:
        if self._first_mono_ns is None:
            self._first_mono_ns = sample.mono_ns
            self._previous_mono_ns = sample.mono_ns
            missing_cycles = 0
            self._device_time_ns = 0
        else:
            assert self._previous_mono_ns is not None
            elapsed_ns = max(0, sample.mono_ns - self._previous_mono_ns)
            elapsed_cycles = max(1, round(elapsed_ns / self.cycle_ns))
            missing_cycles = max(0, elapsed_cycles - 1)
            self._device_time_ns += elapsed_cycles * self.cycle_ns
            self._previous_mono_ns = sample.mono_ns

        return StoredStateSample(
            device_time_ns=self._device_time_ns,
            device_sample_index=sample.sample_index,
            estimated_missing_cycles=missing_cycles,
            mono_ns=sample.mono_ns,
            system_time_ns=sample.system_time_ns,
            sdk_error_code=sample.sdk_error_code,
            arm_current_status=sample.arm_current_status,
            joint_rad=sample.joint_rad,
            tcp_position_base_m=sample.tcp_position_base_m,
            tcp_quaternion_base_xyzw=sample.tcp_quaternion_base_xyzw,
        )

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
            sample.sdk_error_code,
            sample.arm_current_status,
            *sample.joint_rad,
            *sample.tcp_position_base_m,
            *sample.tcp_quaternion_base_xyzw,
        )

    def record(self, duration: Optional[float], startup_timeout: float) -> None:
        """Stream CSV rows until Ctrl+C or the optional duration elapses."""

        wait_started = time.monotonic()
        recording_started_mono_ns: Optional[int] = None

        with self.csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(CSV_COLUMNS)
            csv_file.flush()

            while True:
                if self._callback_error_count:
                    raise RuntimeError(
                        "Realtime callback failed: "
                        f"{self._last_callback_error or 'unknown error'}"
                    )

                try:
                    raw_sample = self._queue.get(timeout=0.2)
                except queue.Empty:
                    if (
                        recording_started_mono_ns is None
                        and time.monotonic() - wait_started >= startup_timeout
                    ):
                        raise TimeoutError(
                            "No realtime state arrived. Check --local-ip, "
                            "--udp-port, the robot realtime-push setting, and "
                            "the Windows firewall."
                        )
                    if (
                        duration is not None
                        and recording_started_mono_ns is not None
                        and time.monotonic_ns() - recording_started_mono_ns
                        >= int(duration * 1_000_000_000)
                    ):
                        break
                    continue

                if recording_started_mono_ns is None:
                    recording_started_mono_ns = raw_sample.mono_ns

                sample = self._add_timing(raw_sample)
                self._samples.append(sample)
                writer.writerow(self._csv_row(sample))
                if len(self._samples) % self.flush_every == 0:
                    csv_file.flush()

                if (
                    duration is not None
                    and raw_sample.mono_ns - recording_started_mono_ns
                    >= int(duration * 1_000_000_000)
                ):
                    break

            csv_file.flush()

    def stop_receiving(self) -> None:
        """Stop accepting callbacks and restore the previous controller config."""

        self._accept_callbacks = False
        if self._robot is None:
            return

        try:
            arm = self._robot.arm
            if self._configured_push and self._previous_push_config is not None:
                code = arm.Set_Realtime_Push(self._previous_push_config)
                self._configured_push = False
                if int(code) != 0:
                    print(
                        "Warning: failed to restore the realtime-push "
                        f"configuration (SDK code {code}).",
                        file=sys.stderr,
                    )
        except BaseException as exc:
            print(
                f"Warning: failed to restore realtime-push configuration: {exc}",
                file=sys.stderr,
            )

    def drain_queue(self) -> None:
        """Preserve callback samples that were queued immediately before stop."""

        drained: list[StoredStateSample] = []
        while True:
            try:
                raw_sample = self._queue.get_nowait()
            except queue.Empty:
                break
            sample = self._add_timing(raw_sample)
            self._samples.append(sample)
            drained.append(sample)

        if not drained:
            return

        csv_exists = self.csv_path.exists() and self.csv_path.stat().st_size > 0
        with self.csv_path.open("a", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            if not csv_exists:
                writer.writerow(CSV_COLUMNS)
            writer.writerows(self._csv_row(sample) for sample in drained)
            csv_file.flush()

    def save_npz(self) -> None:
        """Save arrays with the same keys and dtypes as the reference NPZ."""

        if not self.csv_path.exists():
            with self.csv_path.open("w", newline="", encoding="utf-8") as csv_file:
                csv.writer(csv_file).writerow(CSV_COLUMNS)

        samples = self._samples
        joint_rad = np.asarray([s.joint_rad for s in samples], dtype=np.float32)
        tcp_position = np.asarray(
            [s.tcp_position_base_m for s in samples], dtype=np.float32
        )
        tcp_quaternion = np.asarray(
            [s.tcp_quaternion_base_xyzw for s in samples], dtype=np.float32
        )
        if not samples:
            joint_rad = np.empty((0, JOINT_COUNT), dtype=np.float32)
            tcp_position = np.empty((0, 3), dtype=np.float32)
            tcp_quaternion = np.empty((0, 4), dtype=np.float32)

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
            sdk_error_code=np.asarray(
                [s.sdk_error_code for s in samples], dtype=np.int32
            ),
            arm_current_status=np.asarray(
                [s.arm_current_status for s in samples], dtype=np.int32
            ),
            joint_rad=joint_rad,
            tcp_position_base_m=tcp_position,
            tcp_quaternion_base_xyzw=tcp_quaternion,
            device_push_cycle_ns=np.asarray(self.cycle_ns, dtype=np.int64),
            device_time_source=np.asarray(DEVICE_TIME_SOURCE, dtype="U40"),
            native_controller_timestamp_available=np.asarray(False, dtype=np.bool_),
            queue_overflow_count=np.asarray(
                self._queue_overflow_count, dtype=np.int64
            ),
        )

    def close(self) -> None:
        if self._robot is not None:
            try:
                # Do not call stop_script(): this logger never owns robot motion.
                self._robot.close()
            finally:
                self._robot = None


def detect_local_ip(robot_ip: str, robot_port: int) -> str:
    """Return the local IPv4 address Windows routes toward the robot."""

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((robot_ip, robot_port))
        local_ip = str(probe.getsockname()[0])
    finally:
        probe.close()
    if local_ip == "0.0.0.0":
        raise RuntimeError(
            "Could not detect the local robot-network IP; pass --local-ip"
        )
    return local_ip


def positive_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("value must be a positive number")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record RM65-B joint/TCP realtime output to CSV and compressed NPZ."
        )
    )
    parser.add_argument("--robot-ip", default="192.168.1.18")
    parser.add_argument("--robot-port", type=int, default=8080)
    parser.add_argument(
        "--local-ip",
        help=(
            "PC IPv4 address on the robot network. By default it is detected "
            "from the route to --robot-ip."
        ),
    )
    parser.add_argument("--udp-port", type=int, default=8089)
    parser.add_argument(
        "--cycle-ms",
        type=int,
        default=20,
        help="Requested realtime push cycle in milliseconds (multiple of 5).",
    )
    parser.add_argument(
        "--duration",
        type=positive_float,
        help="Recording duration in seconds; otherwise record until Ctrl+C.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(r"C:\RM65Logs"),
    )
    parser.add_argument("--queue-size", type=int, default=10_000)
    parser.add_argument("--startup-timeout", type=positive_float, default=5.0)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    local_ip = args.local_ip or detect_local_ip(args.robot_ip, args.robot_port)

    logger = RM65StateLogger(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        local_ip=local_ip,
        udp_port=args.udp_port,
        cycle_ms=args.cycle_ms,
        output_dir=args.output_dir.expanduser().resolve(),
        queue_size=args.queue_size,
    )

    print(f"Robot:     {args.robot_ip}:{args.robot_port}")
    print(f"UDP target: {local_ip}:{args.udp_port}")
    print(f"CSV:       {logger.csv_path}")
    print(f"NPZ:       {logger.npz_path}")

    exit_code = 0
    try:
        logger.connect_and_start()
        print("Recording real robot state. Press Ctrl+C to stop and save.")
        logger.record(args.duration, args.startup_timeout)
    except KeyboardInterrupt:
        print("\nStopping recording...")
    except BaseException as exc:
        exit_code = 1
        print(f"Logger error: {exc}", file=sys.stderr)
    finally:
        logger.stop_receiving()
        logger.drain_queue()
        try:
            logger.save_npz()
        except BaseException as exc:
            exit_code = 1
            print(f"Failed to save NPZ: {exc}", file=sys.stderr)
        finally:
            logger.close()

    print(
        f"Saved {logger.sample_count} samples "
        f"(queue overflows: {logger.queue_overflow_count})."
    )
    print(f"CSV: {logger.csv_path}")
    print(f"NPZ: {logger.npz_path}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
