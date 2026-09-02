import math
import select
import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from teleop.aubo_i7_collision import AuboI7CollisionChecker
from teleop.aubo_i7_decoupled_ik import AUBO_I7_NOMINAL_POSTURE_RAD
from teleop.gripper_request import GripperRequest
from teleop.vr_calibration import (
    CalibrationError,
    ConsoleKeyReader,
    GuidedPositionCalibrator,
    default_calibration_path,
    load_calibration,
    save_calibration_atomic,
)
from teleop.vr_tracker import TeleoperationSafetyWarning, VRTracker
from Aubo_Control.aubo_control import AuboI7


ROBOT_COMMAND_RATE_HZ = 100.0
MAX_FRESH_SAMPLE_AGE_S = 0.050
MAX_JOINT_SPEED_RAD_S = math.radians(180.0)
MAX_CARTESIAN_SPEED_M_S = 0.5
UNITY_PLANE_ROBOT_Z_M = -0.05
LEGACY_POSITION_MAPPING = (-1.0, -1.0, 1.0)
# Hand-to-robot motion ratio.  The controller displacement is multiplied by
# this scale after axis alignment, so 1.5 means a 1 cm hand motion moves the
# TCP 1.5 cm.  The direction mapping itself stays calibrated at unit length.
VR_TO_ROBOT_MOTION_SCALE = 1.5


def align_front_back_and_left_right(position_mapping, scale=1.0):
    """Route controller front/back and left/right to the matching robot axes.

    ``scale`` multiplies the aligned mapping uniformly, setting the hand-to-
    robot displacement ratio without changing any direction.
    """

    mapping = np.asarray(position_mapping, dtype=np.float64)
    if mapping.shape == (3,):
        mapping = np.diag(mapping)
    if mapping.shape != (3, 3) or not np.all(np.isfinite(mapping)):
        raise ValueError("position mapping must be a finite length-3 or 3x3 value")
    scale = float(scale)
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("position mapping scale must be positive")
    aligned_mapping = mapping.copy()
    # Keep the requested horizontal direction signs, then exchange the robot
    # X/Y output rows so front/back and left/right no longer drive each other.
    aligned_mapping[:2, :] *= -1.0
    aligned_mapping[[0, 1], :] = aligned_mapping[[1, 0], :]
    # The physical/visual lateral axis is opposite to the controller's lateral
    # convention.  Reverse only that output; front/back and vertical stay put.
    aligned_mapping[0, :] *= -1.0
    return aligned_mapping * scale


class SimulatedAuboI7:
    """Minimal joint-level plant used until the operator connects hardware.

    The simulator accepts the same ``servo_joint`` calls as :class:`AuboI7`
    and treats each accepted command as immediate joint feedback.  Unity still
    receives the checked joint targets through ``UnityJointStatePublisher``;
    no ARCS SDK connection is opened by this class.
    """

    def __init__(self, initial_joints=AUBO_I7_NOMINAL_POSTURE_RAD) -> None:
        self._joints = self._joint_vector(initial_joints)

    @staticmethod
    def _joint_vector(joints: Sequence[float]) -> list[float]:
        try:
            values = [float(value) for value in joints]
        except (TypeError, ValueError) as exc:
            raise ValueError("joint target must contain six finite angles") from exc
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError("joint target must contain six finite angles")
        return values

    def get_joint_values(self) -> list[float]:
        return self._joints.copy()

    def servo_joint(self, joints: Sequence[float], *, time: float) -> int:
        period = float(time)
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("servo period must be positive")
        self._joints = self._joint_vector(joints)
        return 0

    def close(self) -> None:
        pass


@dataclass(frozen=True)
class ControllerSample:
    """One native Unity/XR frame, optionally carrying protocol-v2 timing."""

    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]
    trigger_held: bool
    sequence_id: Optional[int] = None
    unity_time_s: Optional[float] = None


def format_joint_state_line(
    command_sequence: int,
    joints_rad: Sequence[float],
    *,
    source_sequence: Optional[int] = None,
    source_unity_time_s: Optional[float] = None,
    python_time_s: Optional[float] = None,
    mode: str = "track",
) -> str:
    """Serialize one accepted AUBO command for Unity protocol ``J1``.

    The six angles are radians in controller J1..J6 order.  Missing timing
    metadata from a legacy Unity sender is represented by ``-1``.
    """

    if (
        isinstance(command_sequence, bool)
        or not isinstance(command_sequence, int)
        or command_sequence < 0
    ):
        raise ValueError("command_sequence must be a non-negative integer")

    if source_sequence is None:
        unity_sequence = -1
    else:
        if (
            isinstance(source_sequence, bool)
            or not isinstance(source_sequence, int)
            or source_sequence < 0
        ):
            raise ValueError("source_sequence must be a non-negative integer")
        unity_sequence = source_sequence

    unity_time = (
        -1.0 if source_unity_time_s is None else float(source_unity_time_s)
    )
    python_time = (
        time.monotonic() if python_time_s is None else float(python_time_s)
    )
    if (
        not math.isfinite(unity_time)
        or unity_time < -1.0
        or not math.isfinite(python_time)
        or python_time < 0.0
    ):
        raise ValueError("joint-state timestamps are invalid")

    if (
        not isinstance(mode, str)
        or not mode
        or len(mode) > 32
        or not all(
            character.isalnum() or character == "_" for character in mode
        )
        or not mode.isascii()
    ):
        raise ValueError("mode must be a short ASCII identifier")

    try:
        joints = tuple(float(value) for value in joints_rad)
    except (TypeError, ValueError) as exc:
        raise ValueError("joints_rad must contain six finite angles") from exc
    if len(joints) != 6 or not all(math.isfinite(value) for value in joints):
        raise ValueError("joints_rad must contain six finite angles")

    joint_text = ",".join(f"{value:.9f}" for value in joints)
    return (
        f"J1,{command_sequence},{unity_sequence},{unity_time:.6f},"
        f"{python_time:.6f},{mode},{joint_text}\n"
    )


class UnityJointStatePublisher:
    """Non-blocking latest-value publisher on the existing Unity TCP socket.

    Robot control must never wait for Unity rendering or TCP backpressure.  A
    daemon worker owns socket writes and a capacity-one slot replaces any
    unsent older command with the newest accepted command.
    """

    def __init__(self, transport) -> None:
        if not callable(getattr(transport, "sendall", None)):
            raise TypeError("transport must provide sendall(bytes)")
        self._transport = transport
        self._condition = threading.Condition()
        self._latest_payload: Optional[bytes] = None
        self._next_sequence = 0
        self._closed = False
        self._failure: Optional[OSError] = None
        self._thread = threading.Thread(
            target=self._send_loop,
            name="unity-aubo-joint-publisher",
            daemon=True,
        )
        self._thread.start()

    @property
    def failure(self) -> Optional[OSError]:
        with self._condition:
            return self._failure

    def publish(
        self,
        joints_rad: Sequence[float],
        *,
        source_sequence: Optional[int] = None,
        source_unity_time_s: Optional[float] = None,
        python_time_s: Optional[float] = None,
        mode: str = "track",
    ) -> Optional[int]:
        with self._condition:
            if self._closed:
                return None
            command_sequence = self._next_sequence
            payload = format_joint_state_line(
                command_sequence,
                joints_rad,
                source_sequence=source_sequence,
                source_unity_time_s=source_unity_time_s,
                python_time_s=python_time_s,
                mode=mode,
            ).encode("ascii")
            self._next_sequence += 1
            self._latest_payload = payload
            self._condition.notify()
            return command_sequence

    def _send_loop(self) -> None:
        while True:
            with self._condition:
                while self._latest_payload is None and not self._closed:
                    self._condition.wait()
                if self._closed:
                    return
                payload = self._latest_payload
                self._latest_payload = None

            try:
                self._transport.sendall(payload)
            except OSError as exc:
                with self._condition:
                    self._failure = exc
                    self._latest_payload = None
                    self._closed = True
                    self._condition.notify_all()
                return

    def close(self, join_timeout_s: float = 0.2) -> None:
        timeout = float(join_timeout_s)
        if not math.isfinite(timeout) or timeout < 0.0:
            raise ValueError("join_timeout_s must be non-negative")
        with self._condition:
            self._closed = True
            self._latest_payload = None
            self._condition.notify_all()
        self._thread.join(timeout)


def parse_controller_line(line):
    """Parse a legacy 9-field or timestamped 11-field controller record."""

    fields = line.strip().split(",")
    if len(fields) not in (9, 11):
        raise ValueError(
            f"expected 9 or 11 controller fields, received {len(fields)}"
        )
    values = [float(field) for field in fields[:9]]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("controller record contains a non-finite value")
    sequence_id = None
    unity_time_s = None
    if len(fields) == 11:
        sequence_id = int(fields[9])
        unity_time_s = float(fields[10])
        if sequence_id < 0 or not math.isfinite(unity_time_s) or unity_time_s < 0.0:
            raise ValueError("controller timing fields are invalid")
    return ControllerSample(
        position=tuple(values[:3]),
        rotation=tuple(values[3:7]),
        trigger_held=bool(values[7]),
        sequence_id=sequence_id,
        unity_time_s=unity_time_s,
    )


class FixedRateScheduler:
    """Absolute-deadline scheduler that skips missed ticks instead of bursting."""

    def __init__(self, rate_hz=ROBOT_COMMAND_RATE_HZ, *, start_time_s=None):
        self.rate_hz = float(rate_hz)
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")
        self.period_s = 1.0 / self.rate_hz
        start = time.monotonic() if start_time_s is None else float(start_time_s)
        self.next_deadline_s = start + self.period_s

    def timeout(self, now_s):
        return max(0.0, self.next_deadline_s - float(now_s))

    def consume_due(self, now_s):
        now = float(now_s)
        if now < self.next_deadline_s:
            return False, 0
        missed = max(0, int((now - self.next_deadline_s) // self.period_s))
        self.next_deadline_s += (missed + 1) * self.period_s
        return True, missed


class StreamFrequencyReporter:
    """Measure native Meta/Unity input and actual robot-command frequencies."""

    def __init__(self, report_interval_s=3.0, *, start_time_s=None):
        self.report_interval_s = float(report_interval_s)
        self._window_started_at = (
            time.monotonic() if start_time_s is None else float(start_time_s)
        )
        self._last_sequence = None
        self._last_unity_time = None
        self._meta_rate_hz = None
        self._robot_commands = 0
        self._dropped_samples = 0
        self._missed_robot_ticks = 0

    @property
    def meta_rate_hz(self):
        return self._meta_rate_hz

    @property
    def dropped_samples(self):
        return self._dropped_samples

    def observe_sample(self, sample):
        sequence = sample.sequence_id
        unity_time = sample.unity_time_s
        if sequence is None or unity_time is None:
            return
        if self._last_sequence is not None and sequence > self._last_sequence:
            sequence_delta = sequence - self._last_sequence
            time_delta = unity_time - self._last_unity_time
            if time_delta > 0.0:
                measured = sequence_delta / time_delta
                if 1.0 <= measured <= 1000.0:
                    if self._meta_rate_hz is None:
                        self._meta_rate_hz = measured
                    else:
                        self._meta_rate_hz += 0.1 * (
                            measured - self._meta_rate_hz
                        )
            self._dropped_samples += max(0, sequence_delta - 1)
        self._last_sequence = sequence
        self._last_unity_time = unity_time

    def observe_robot_command(self):
        self._robot_commands += 1

    def observe_missed_robot_ticks(self, count):
        self._missed_robot_ticks += int(count)

    def maybe_report(self, now_s, latest_sample_age_s):
        now = float(now_s)
        elapsed = now - self._window_started_at
        if elapsed < self.report_interval_s:
            return None
        robot_rate = self._robot_commands / elapsed
        meta_text = (
            "legacy/unknown"
            if self._meta_rate_hz is None
            else f"{self._meta_rate_hz:.1f} Hz"
        )
        age_text = (
            "n/a"
            if latest_sample_age_s is None
            else f"{latest_sample_age_s * 1000.0:.1f} ms"
        )
        message = (
            f"FREQUENCY: Meta/Unity {meta_text}; robot commands "
            f"{robot_rate:.1f} Hz; latest sample age {age_text}; "
            f"dropped Meta samples {self._dropped_samples}; skipped robot ticks "
            f"{self._missed_robot_ticks}."
        )
        self._window_started_at = now
        self._robot_commands = 0
        self._dropped_samples = 0
        self._missed_robot_ticks = 0
        return message


class SafetyWarningReporter:
    """Rate-limit repeated warnings from held or gradually limited targets."""

    def __init__(self, repeat_interval_s=1.0):
        self.repeat_interval_s = float(repeat_interval_s)
        self._last_message = None
        self._last_printed_at = 0.0

    def report(self, warning):
        message = str(warning)
        now = time.monotonic()
        if (
            message != self._last_message
            or now - self._last_printed_at >= self.repeat_interval_s
        ):
            print(f"SAFETY WARNING: {message}")
            self._last_message = message
            self._last_printed_at = now

    def clear(self):
        self._last_message = None


def process_controller_sample(teleop, position, rotation, trigger_held):
    """Process one controller sample without issuing a robot stop command."""

    controller = position + rotation
    if trigger_held:
        if teleop.paused:
            teleop.resume(controller)
        return teleop.track(controller)
    if not teleop.paused:
        teleop.pause()
    return None


def create_tracker(robot, gripper, position_mapping):
    """Build one tracker for either the simulator or the real AUBO backend."""

    return VRTracker(
        robot,
        gripper,
        rate=1.0 / ROBOT_COMMAND_RATE_HZ,
        # smooth_step=0.15,
        smooth_step=1,
        # Direction selection and the configured motion scale are applied
        # before tracker construction.
        pos_mapping=position_mapping,
        # Calibrate at low speed so J6 follows the operator's turn.
        wrist_roll_axis=(1.0, 0.0, 0.0),
        wrist_roll_sign=1.0,
        max_joint_speed_rad_s=MAX_JOINT_SPEED_RAD_S,
        max_cartesian_speed_m_s=MAX_CARTESIAN_SPEED_M_S,
        # No per-direction workspace sphere: the TCP follows the hand offset
        # without a maximum-travel cap, bounded only by joint/speed limits.
        max_position_offset_m=None,
        collision_checker=AuboI7CollisionChecker(
            plane_z_m=UNITY_PLANE_ROBOT_Z_M,
        ),
    )


def main():
    # Create a TCP socket to listen to Unity
    unity_ip = "127.0.0.1"
    port = 5555

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((unity_ip, port))
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Connected to Unity server")

            # Start with a local plant.  Hardware is connected only after the
            # operator explicitly presses L in this terminal.
            robot = SimulatedAuboI7()
            real_robot_connected = False
            joint_publisher = UnityJointStatePublisher(client_socket)
            try:
                gripper = GripperRequest("130.215.216.42")
                calibration_path = default_calibration_path()
                try:
                    saved_calibration = load_calibration(calibration_path)
                    position_mapping = align_front_back_and_left_right(
                        saved_calibration.rotation_vr_to_robot,
                        VR_TO_ROBOT_MOTION_SCALE,
                    )
                    print(f"Loaded VR calibration: {calibration_path}")
                except FileNotFoundError:
                    position_mapping = align_front_back_and_left_right(
                        LEGACY_POSITION_MAPPING,
                        VR_TO_ROBOT_MOTION_SCALE,
                    )
                    print(
                        "No saved VR calibration; using aligned front/back and "
                        "left/right legacy mapping."
                    )
                except CalibrationError as exc:
                    position_mapping = align_front_back_and_left_right(
                        LEGACY_POSITION_MAPPING,
                        VR_TO_ROBOT_MOTION_SCALE,
                    )
                    print(
                        "CALIBRATION WARNING: saved calibration is invalid; "
                        "using aligned front/back and left/right legacy mapping: "
                        f"{exc}"
                    )
                teleop = create_tracker(robot, gripper, position_mapping)
                initial_joints = robot.get_joint_values()
                joint_publisher.publish(initial_joints, mode="sim_initial")
                print(
                    "SIMULATION MODE: no real AUBO connection has been opened."
                )
                print(
                    "Press L in this terminal to connect the real AUBO i7; "
                    "release the Trigger before switching."
                )
                limits_deg = [
                    (round(math.degrees(lower), 3), round(math.degrees(upper), 3))
                    for lower, upper in teleop.joint_limits_rad
                ]
                print(
                    f"Using {teleop.joint_limit_source} joint limits (deg): "
                    f"{limits_deg}"
                )
                warning_reporter = SafetyWarningReporter()
                calibrator = GuidedPositionCalibrator()
                key_reader = ConsoleKeyReader()
                calibration_waiting_for_release = False
                tracking_requires_release = False
                latest_trigger_state = None
                if key_reader.available:
                    print(
                        "Press Q in this terminal to start VR calibration "
                        "(Enter=capture, Y=save, R=redo, Esc=cancel)."
                    )
                else:
                    print(
                        "HOTKEY WARNING: console key input is unavailable; "
                        "L hardware connection and Q calibration cannot be "
                        "started in this terminal."
                    )

                receive_buffer = b""
                scheduler = FixedRateScheduler(ROBOT_COMMAND_RATE_HZ)
                frequency_reporter = StreamFrequencyReporter()
                latest_sample = None
                latest_sample_received_at = None
                publisher_failure_reported = False
                while True:
                    for key in key_reader.poll():
                        lower_key = key.lower()
                        if lower_key == "q":
                            if calibrator.active or calibration_waiting_for_release:
                                print("VR calibration is already active.")
                                continue
                            if not teleop.paused:
                                teleop.pause()
                                print("Pausing command generation for calibration...")
                            tracking_requires_release = True
                            if latest_trigger_state is False:
                                print(calibrator.start().message)
                            else:
                                calibration_waiting_for_release = True
                                print(
                                    "VR calibration requested. Release the Trigger "
                                    "to enter calibration mode."
                                )
                        elif key in ("\r", "\n"):
                            if calibrator.active:
                                print(calibrator.request_capture().message)
                        elif lower_key == "y" and calibrator.awaiting_confirmation:
                            result = calibrator.pending_result
                            try:
                                save_calibration_atomic(calibration_path, result)
                                teleop.set_position_mapping(
                                    align_front_back_and_left_right(
                                        result.rotation_vr_to_robot,
                                        VR_TO_ROBOT_MOTION_SCALE,
                                    )
                                )
                            except (OSError, CalibrationError, RuntimeError) as exc:
                                print(
                                    "CALIBRATION WARNING: could not save/apply "
                                    f"calibration: {exc}"
                                )
                            else:
                                calibrator.finish()
                                tracking_requires_release = True
                                print(
                                    "VR calibration saved and enabled. Release the "
                                    "Trigger once before resuming tracking."
                                )
                        elif lower_key == "r" and calibrator.active:
                            print(calibrator.redo().message)
                        elif key == "\x1b" and (
                            calibrator.active or calibration_waiting_for_release
                        ):
                            calibration_waiting_for_release = False
                            if calibrator.active:
                                print(calibrator.cancel().message)
                            else:
                                print("VR calibration request cancelled.")
                            tracking_requires_release = True
                        elif lower_key == "l":
                            if real_robot_connected:
                                print("The real AUBO i7 is already connected.")
                                continue
                            if calibrator.active or calibration_waiting_for_release:
                                print(
                                    "Finish or cancel VR calibration before "
                                    "connecting the real AUBO i7."
                                )
                                continue

                            # Freeze command generation before any potentially
                            # blocking network handshake or backend replacement.
                            if not teleop.paused:
                                teleop.pause()
                            tracking_requires_release = True
                            print(
                                "Connecting real AUBO i7 at "
                                "192.168.127.128:30004..."
                            )
                            candidate_robot = None
                            try:
                                candidate_robot = AuboI7(
                                    "192.168.127.128", port=30004
                                )
                                candidate_robot.start_servo_mode()
                                candidate_joints = (
                                    candidate_robot.get_joint_values()
                                )
                                candidate_teleop = create_tracker(
                                    candidate_robot,
                                    gripper,
                                    teleop.position_mapping_matrix,
                                )
                            except Exception as exc:
                                if candidate_robot is not None:
                                    try:
                                        candidate_robot.close()
                                    except Exception as close_exc:
                                        print(
                                            "AUBO CLEANUP WARNING: "
                                            f"{close_exc}"
                                        )
                                print(
                                    "AUBO CONNECTION FAILED: remaining in "
                                    f"simulation mode: {exc}"
                                )
                            else:
                                previous_robot = robot
                                robot = candidate_robot
                                teleop = candidate_teleop
                                real_robot_connected = True
                                previous_robot.close()
                                joint_publisher.publish(
                                    candidate_joints,
                                    mode="real_initial",
                                )
                                warning_reporter.clear()
                                limits_deg = [
                                    (
                                        round(math.degrees(lower), 3),
                                        round(math.degrees(upper), 3),
                                    )
                                    for lower, upper in teleop.joint_limits_rad
                                ]
                                print(
                                    "REAL ROBOT MODE: AUBO servo mode enabled. "
                                    "Release the Trigger, then press it again "
                                    "to begin tracking."
                                )
                                print(
                                    f"Using {teleop.joint_limit_source} joint "
                                    f"limits (deg): {limits_deg}"
                                )

                    now = time.monotonic()
                    readable, _, _ = select.select(
                        [client_socket], [], [], scheduler.timeout(now)
                    )
                    if readable:
                        chunk = client_socket.recv(4096)
                        if not chunk:
                            break

                        # TCP packets do not preserve message boundaries.  Parse
                        # every complete record into a latest-value slot, but
                        # never issue burst robot commands to catch up.
                        receive_buffer += chunk
                        records = receive_buffer.split(b"\n")
                        receive_buffer = records.pop()
                        for raw_record in records:
                            try:
                                sample = parse_controller_line(
                                    raw_record.decode("ascii")
                                )
                            except (UnicodeDecodeError, ValueError) as exc:
                                print(
                                    f"Ignoring malformed controller record: {exc}"
                                )
                                continue

                            received_at = time.monotonic()
                            frequency_reporter.observe_sample(sample)
                            latest_trigger_state = sample.trigger_held
                            if calibration_waiting_for_release:
                                if not sample.trigger_held:
                                    calibration_waiting_for_release = False
                                    print(calibrator.start().message)
                                continue

                            if calibrator.active:
                                if sample.trigger_held:
                                    tracking_requires_release = True
                                try:
                                    update = calibrator.observe(sample.position)
                                except CalibrationError as exc:
                                    print(
                                        "CALIBRATION WARNING: sample ignored: "
                                        f"{exc}"
                                    )
                                else:
                                    if update is not None:
                                        print(update.message)
                                continue

                            latest_sample = sample
                            latest_sample_received_at = received_at

                    now = time.monotonic()
                    due, missed_ticks = scheduler.consume_due(now)
                    if not due:
                        continue
                    frequency_reporter.observe_missed_robot_ticks(missed_ticks)
                    sample_age = (
                        None
                        if latest_sample_received_at is None
                        else max(0.0, now - latest_sample_received_at)
                    )
                    frequency_message = frequency_reporter.maybe_report(
                        now, sample_age
                    )
                    if frequency_message is not None:
                        print(frequency_message)
                    publisher_failure = joint_publisher.failure
                    if (
                        publisher_failure is not None
                        and not publisher_failure_reported
                    ):
                        print(
                            "UNITY MIRROR WARNING: joint-state publishing failed; "
                            f"robot control will continue: {publisher_failure}"
                        )
                        publisher_failure_reported = True

                    if (
                        calibration_waiting_for_release
                        or calibrator.active
                        or latest_sample is None
                    ):
                        continue

                    if tracking_requires_release:
                        if latest_sample.trigger_held:
                            continue
                        tracking_requires_release = False

                    was_paused = teleop.paused
                    if not latest_sample.trigger_held:
                        process_controller_sample(
                            teleop,
                            latest_sample.position,
                            latest_sample.rotation,
                            False,
                        )
                        if not was_paused:
                            print("Pausing command generation...")
                        warning_reporter.clear()
                        continue

                    if sample_age is None or sample_age > MAX_FRESH_SAMPLE_AGE_S:
                        warning_reporter.report(
                            "controller sample is stale; repeating the last safe "
                            "joint target without advancing VR tracking"
                        )
                        repeated = teleop.keepalive()
                        if repeated is not None:
                            frequency_reporter.observe_robot_command()
                            joint_publisher.publish(
                                repeated,
                                source_sequence=latest_sample.sequence_id,
                                source_unity_time_s=latest_sample.unity_time_s,
                                mode="keepalive_stale",
                            )
                        continue

                    try:
                        command = process_controller_sample(
                            teleop,
                            latest_sample.position,
                            latest_sample.rotation,
                            True,
                        )
                        if was_paused and not teleop.paused:
                            print("Resuming decoupled joint teleoperation...")
                        if command is not None:
                            frequency_reporter.observe_robot_command()
                            joint_publisher.publish(
                                command,
                                source_sequence=latest_sample.sequence_id,
                                source_unity_time_s=latest_sample.unity_time_s,
                                mode="track",
                            )
                        if teleop.last_warning_reason is None:
                            warning_reporter.clear()
                        else:
                            warning_reporter.report(teleop.last_warning_reason)
                    except TeleoperationSafetyWarning as warning:
                        warning_reporter.report(warning)
                        # AUBO servoJoint expects a continuous stream.  A
                        # held VR target must not create the old self-lock
                        # where low-follow feedback can never reach the last
                        # accepted target because command transmission stopped.
                        repeated = teleop.keepalive()
                        if repeated is not None:
                            frequency_reporter.observe_robot_command()
                            joint_publisher.publish(
                                repeated,
                                source_sequence=latest_sample.sequence_id,
                                source_unity_time_s=latest_sample.unity_time_s,
                                mode="keepalive_safety",
                            )
            finally:
                # Disconnect only.  The teleoperation application deliberately
                # does not request a slow stop; controller/teach-pendant safety
                # logic remains authoritative.
                joint_publisher.close()
                robot.close()

    except KeyboardInterrupt:
        print("Closing connection...")

if __name__ == "__main__":
    main()
