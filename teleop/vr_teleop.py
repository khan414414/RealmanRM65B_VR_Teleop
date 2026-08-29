import math
import select
import socket
import time
from dataclasses import dataclass
from typing import Optional

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
from Realman_Control.realman_control import RM65B


ROBOT_COMMAND_RATE_HZ = 100.0
MAX_FRESH_SAMPLE_AGE_S = 0.050


@dataclass(frozen=True)
class ControllerSample:
    """One native Unity/XR frame, optionally carrying protocol-v2 timing."""

    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]
    trigger_held: bool
    sequence_id: Optional[int] = None
    unity_time_s: Optional[float] = None


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


def main():
    # Create a TCP socket to listen to Unity
    unity_ip = "127.0.0.1"
    port = 5555

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((unity_ip, port))
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Connected to Unity server")

            # Initialize the teleoperation class
            # rtde = RTDE("192.168.1.102")
            rtde = RM65B("192.168.1.18", port=8080)
            try:
                gripper = GripperRequest("130.215.216.42")
                calibration_path = default_calibration_path()
                try:
                    saved_calibration = load_calibration(calibration_path)
                    position_mapping = saved_calibration.rotation_vr_to_robot
                    print(f"Loaded VR calibration: {calibration_path}")
                except FileNotFoundError:
                    position_mapping = (-1.0, -1.0, 1.0)
                    print("No saved VR calibration; using legacy XYZ mapping.")
                except CalibrationError as exc:
                    position_mapping = (-1.0, -1.0, 1.0)
                    print(
                        "CALIBRATION WARNING: saved calibration is invalid; "
                        f"using legacy XYZ mapping: {exc}"
                    )
                teleop = VRTracker(
                    rtde,
                    gripper,
                    rate=1.0 / ROBOT_COMMAND_RATE_HZ,
                    #smooth_step=0.15,
                    smooth_step=1,
                    # 1:1 distance; flip horizontal axes to match the operator's
                    # forward/left directions with the RM65 base frame.
                    pos_mapping=position_mapping,
                    # Calibrated so the RM65 wrist follows the operator's turn.
                    wrist_roll_axis=(1.0, 0.0, 0.0),
                    wrist_roll_sign=1.0,
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
                        "CALIBRATION WARNING: console key input is unavailable; "
                        "Q calibration cannot be started in this terminal."
                    )

                receive_buffer = b""
                scheduler = FixedRateScheduler(ROBOT_COMMAND_RATE_HZ)
                frequency_reporter = StreamFrequencyReporter()
                latest_sample = None
                latest_sample_received_at = None
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
                                    result.rotation_vr_to_robot
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
                        if teleop.last_warning_reason is None:
                            warning_reporter.clear()
                        else:
                            warning_reporter.report(teleop.last_warning_reason)
                    except TeleoperationSafetyWarning as warning:
                        warning_reporter.report(warning)
                        # CAN-FD pass-through expects a continuous stream.  A
                        # held VR target must not create the old self-lock
                        # where low-follow feedback can never reach the last
                        # accepted target because command transmission stopped.
                        repeated = teleop.keepalive()
                        if repeated is not None:
                            frequency_reporter.observe_robot_command()
            finally:
                # Disconnect only.  The teleoperation application deliberately
                # does not request a slow stop; controller/teach-pendant safety
                # logic remains authoritative.
                rtde.close()

    except KeyboardInterrupt:
        print("Closing connection...")

if __name__ == "__main__":
    main()
