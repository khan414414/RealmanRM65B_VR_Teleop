"""AUBO i7 controller adapter built on the AUBO ARCS Python SDK.

The adapter intentionally exposes the same small, SI-unit control surface used
by the teleoperation code:

* joint positions and speeds are radians and radians/second;
* poses and TCP offsets are ``[x, y, z, rx, ry, rz]`` in metres plus a
  rotation vector in radians;
* controller-reported position, velocity and acceleration limits are used
  directly.  No model limits are substituted in the client.

``pyaubo_sdk`` is an optional runtime dependency.  It is imported lazily so
documentation, static analysis and fake-SDK tests work on machines without the
vendor package installed.
"""

from __future__ import annotations

import importlib
import math
import re
import time as _time
from typing import Any, Optional, Sequence


class AuboControlError(RuntimeError):
    """A non-zero result returned by an AUBO ARCS SDK operation."""

    def __init__(self, operation: str, code: Any):
        self.operation = operation
        self.code = code
        super().__init__(
            f"{operation} failed with AUBO ARCS SDK error code {code}"
        )


class AuboModelError(RuntimeError):
    """The connected robot does not match the requested model."""


# A short compatibility name for callers that used ``*Error`` in older robot
# adapters.
AuboError = AuboControlError


class AuboControl:
    """Control an AUBO i7 over the ARCS RPC service.

    Parameters
    ----------
    robot_ip:
        Controller address.  AUBO controllers commonly use ``192.168.1.100``.
    port:
        ARCS RPC port; the documented default is 30004.
    username, password:
        ARCS login credentials.  Factory defaults are ``aubo``/``123456``.
    robot_name:
        Optional name returned by ``RpcClient.getRobotNames()``.  The first
        configured robot is selected when omitted.
    expected_model:
        Expected model designation, normally ``"i7"``.  Set to ``None`` to
        deliberately disable model verification (for example on an older
        controller that does not expose model strings).
    sdk_module:
        Optional module-like object exposing ``RpcClient``.  This is primarily
        useful for deterministic tests without hardware.

    Notes
    -----
    Servo mode is never entered implicitly.  Call :meth:`start_servo_mode`
    before :meth:`servo_joint`, and call :meth:`stop_servo_mode` when the stream
    ends.  Both transitions use bounded waits.
    """

    JOINT_COUNT = 6
    DEFAULT_IP = "192.168.1.100"
    DEFAULT_PORT = 30004
    DEFAULT_USERNAME = "aubo"
    DEFAULT_PASSWORD = "123456"
    QUEUE_FULL_CODE = 2

    def __init__(
        self,
        robot_ip: str = DEFAULT_IP,
        port: int = DEFAULT_PORT,
        username: str = DEFAULT_USERNAME,
        password: str = DEFAULT_PASSWORD,
        *,
        robot_name: Optional[str] = None,
        expected_model: Optional[str] = "i7",
        sdk_module: Any = None,
        servo_mode_timeout: float = 1.0,
        servo_poll_interval: float = 0.01,
        servo_queue_retries: int = 3,
        servo_retry_interval: float = 0.001,
    ) -> None:
        self.robot_ip = self._nonempty_string(robot_ip, "robot_ip")
        self.port = self._port(port)
        self.username = self._nonempty_string(username, "username")
        if not isinstance(password, str):
            raise TypeError("password must be a string")
        self._password = password
        if robot_name is not None:
            robot_name = self._nonempty_string(robot_name, "robot_name")
        if expected_model is not None:
            expected_model = self._nonempty_string(
                expected_model, "expected_model"
            )

        self._servo_mode_timeout = self._nonnegative_scalar(
            servo_mode_timeout, "servo_mode_timeout"
        )
        self._servo_poll_interval = self._positive_scalar(
            servo_poll_interval, "servo_poll_interval"
        )
        self._servo_queue_retries = self._nonnegative_integer(
            servo_queue_retries, "servo_queue_retries"
        )
        self._servo_retry_interval = self._nonnegative_scalar(
            servo_retry_interval, "servo_retry_interval"
        )

        sdk = sdk_module if sdk_module is not None else self._load_sdk()
        rpc_type = getattr(sdk, "RpcClient", None)
        if rpc_type is None or not callable(rpc_type):
            raise TypeError("sdk_module must expose a callable RpcClient")

        self._client = rpc_type()
        self._robot: Any = None
        self._config: Any = None
        self._state: Any = None
        self._motion: Any = None
        self.robot_name: Optional[str] = None
        self.robot_type: Any = None
        self.robot_subtype: Any = None
        self._joint_limits: tuple[tuple[float, float], ...] = ()
        self._joint_max_speeds: tuple[float, ...] = ()
        self._joint_max_accelerations: tuple[float, ...] = ()
        self._connected = False
        self._logged_in = False
        self._closed = True

        try:
            self._check_result(
                self._client.connect(self.robot_ip, self.port),
                "RpcClient.connect",
            )
            self._connected = True
            self._check_result(
                self._client.login(self.username, self._password),
                "RpcClient.login",
            )
            self._logged_in = True

            names = self._robot_names(self._client.getRobotNames())
            if robot_name is None:
                selected_name = names[0]
            elif robot_name in names:
                selected_name = robot_name
            else:
                raise LookupError(
                    f"robot {robot_name!r} is not configured; available robots: "
                    f"{', '.join(names)}"
                )

            robot = self._client.getRobotInterface(selected_name)
            if robot is None:
                raise RuntimeError(
                    f"getRobotInterface({selected_name!r}) returned no interface"
                )
            self.robot_name = selected_name
            self._robot = robot
            self._config = robot.getRobotConfig()
            self._state = robot.getRobotState()
            self._motion = robot.getMotionControl()
            if any(
                interface is None
                for interface in (self._config, self._state, self._motion)
            ):
                raise RuntimeError(
                    "robot interface did not provide config, state and motion APIs"
                )

            if expected_model is not None:
                self.robot_type = self._config.getRobotType()
                self.robot_subtype = self._config.getRobotSubType()
                if not self._model_matches(
                    expected_model, self.robot_type, self.robot_subtype
                ):
                    raise AuboModelError(
                        f"expected AUBO {expected_model}, but controller reports "
                        f"type={self.robot_type!r}, subtype={self.robot_subtype!r}"
                    )

            # From this point the public query helpers may be used.  Read every
            # motion bound from RobotConfig before accepting a target.
            self._closed = False
            self.get_joint_limits()
            self.get_joint_max_speeds()
            self.get_joint_max_accelerations()
        except BaseException:
            self._closed = True
            self._cleanup_failed_initialization()
            raise

    @staticmethod
    def _load_sdk() -> Any:
        try:
            return importlib.import_module("pyaubo_sdk")
        except ImportError as exc:
            raise RuntimeError(
                "AUBO ARCS Python SDK is not installed. Install/configure "
                "`pyaubo_sdk`, or inject sdk_module for testing."
            ) from exc

    @staticmethod
    def _nonempty_string(value: Any, name: str) -> str:
        if not isinstance(value, str):
            raise TypeError(f"{name} must be a string")
        if not value.strip():
            raise ValueError(f"{name} must not be empty")
        return value

    @staticmethod
    def _port(value: Any) -> int:
        if isinstance(value, bool):
            raise TypeError("port must be an integer")
        try:
            result = int(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise TypeError("port must be an integer") from exc
        if result != value or not 1 <= result <= 65535:
            raise ValueError("port must be in the range 1..65535")
        return result

    @staticmethod
    def _scalar(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be a real number")
        try:
            result = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise TypeError(f"{name} must be a real number") from exc
        if not math.isfinite(result):
            raise ValueError(f"{name} must be finite")
        return result

    @classmethod
    def _positive_scalar(cls, value: Any, name: str) -> float:
        result = cls._scalar(value, name)
        if result <= 0.0:
            raise ValueError(f"{name} must be positive")
        return result

    @classmethod
    def _nonnegative_scalar(cls, value: Any, name: str) -> float:
        result = cls._scalar(value, name)
        if result < 0.0:
            raise ValueError(f"{name} must be non-negative")
        return result

    @staticmethod
    def _nonnegative_integer(value: Any, name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
        if value < 0:
            raise ValueError(f"{name} must be non-negative")
        return value

    @classmethod
    def _vector(
        cls, values: Sequence[float], length: int, name: str
    ) -> list[float]:
        if isinstance(values, (str, bytes)):
            raise TypeError(f"{name} must be a numeric sequence")
        try:
            original = list(values)
        except TypeError as exc:
            raise TypeError(f"{name} must be a numeric sequence") from exc
        if len(original) != length:
            raise ValueError(f"{name} must contain exactly {length} values")
        if any(isinstance(value, bool) for value in original):
            raise TypeError(f"{name} must be a numeric sequence")
        try:
            result = [float(value) for value in original]
        except (TypeError, ValueError, OverflowError) as exc:
            raise TypeError(f"{name} must be a numeric sequence") from exc
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"{name} must contain only finite values")
        return result

    @classmethod
    def _positive_vector(
        cls, values: Sequence[float], length: int, name: str
    ) -> tuple[float, ...]:
        result = cls._vector(values, length, name)
        if any(value <= 0.0 for value in result):
            raise RuntimeError(f"controller returned non-positive {name}")
        return tuple(result)

    @staticmethod
    def _return_code(result: Any, operation: str) -> int:
        if result is None:
            return 0
        if isinstance(result, bool):
            return 0 if result else 1
        try:
            code = int(result)
        except (TypeError, ValueError, OverflowError) as exc:
            raise RuntimeError(
                f"{operation} returned an invalid status {result!r}"
            ) from exc
        if code != result:
            raise RuntimeError(
                f"{operation} returned a non-integral status {result!r}"
            )
        return code

    @classmethod
    def _check_result(cls, result: Any, operation: str) -> int:
        code = cls._return_code(result, operation)
        if code != 0:
            raise AuboControlError(operation, code)
        return 0

    @staticmethod
    def _robot_names(values: Any) -> list[str]:
        if isinstance(values, (str, bytes)):
            raise RuntimeError("getRobotNames returned an invalid string value")
        try:
            names = list(values)
        except TypeError as exc:
            raise RuntimeError("getRobotNames returned no robot list") from exc
        if not names:
            raise RuntimeError("controller reports no configured robots")
        if any(not isinstance(name, str) or not name for name in names):
            raise RuntimeError("getRobotNames returned an invalid robot name")
        return names

    @staticmethod
    def _model_matches(expected: str, robot_type: Any, subtype: Any) -> bool:
        expected_compact = re.sub(r"[^a-z0-9]+", "", expected.lower())
        reported = f"{robot_type} {subtype}".lower()
        reported_compact = re.sub(r"[^a-z0-9]+", "", reported)
        if expected_compact == reported_compact:
            return True

        # Model designations such as i7 must be compared as complete numeric
        # tokens so an i7 check cannot accidentally accept i70.
        expected_i_models = set(re.findall(r"i\d+", expected_compact))
        reported_i_models = set(re.findall(r"i\d+", reported_compact))
        if expected_i_models:
            return expected_i_models.issubset(reported_i_models)
        return reported_compact.endswith(expected_compact)

    def _cleanup_failed_initialization(self) -> None:
        if self._logged_in:
            try:
                self._client.logout()
            except BaseException:
                pass
            self._logged_in = False
        if self._connected:
            try:
                self._client.disconnect()
            except BaseException:
                pass
            self._connected = False

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("AUBO RPC connection is closed")

    @property
    def rpc_client(self) -> Any:
        """Return the connected ``RpcClient`` for advanced ARCS operations."""

        self._require_open()
        return self._client

    @property
    def robot(self) -> Any:
        """Return the selected ARCS robot interface."""

        self._require_open()
        return self._robot

    def _controller_vector(self, method: Any, name: str) -> list[float]:
        self._require_open()
        return self._vector(method(), self.JOINT_COUNT, name)

    def get_joint_values(self) -> list[float]:
        """Return actual joint positions in radians."""

        return self._controller_vector(
            self._state.getJointPositions, "joint positions"
        )

    def get_joint_speed(self) -> list[float]:
        """Return actual joint speeds in radians/second."""

        return self._controller_vector(self._state.getJointSpeeds, "joint speeds")

    def get_joint_limits(self) -> tuple[tuple[float, float], ...]:
        """Read controller-configured joint position limits in radians.

        The returned limits replace the validation cache immediately; there is
        intentionally no hard-coded i7 fallback.
        """

        minimum = self._controller_vector(
            self._config.getJointMinPositions, "joint minimum positions"
        )
        maximum = self._controller_vector(
            self._config.getJointMaxPositions, "joint maximum positions"
        )
        limits = tuple(zip(minimum, maximum))
        for index, (lower, upper) in enumerate(limits, 1):
            if lower >= upper:
                raise RuntimeError(
                    f"controller returned invalid position limits for J{index}: "
                    f"{lower}..{upper} rad"
                )
        self._joint_limits = limits
        return tuple(limits)

    def get_joint_max_speeds(self) -> tuple[float, ...]:
        """Return and cache controller maximum joint speeds in rad/s."""

        values = self._positive_vector(
            self._config.getJointMaxSpeeds(),
            self.JOINT_COUNT,
            "joint maximum speeds",
        )
        self._joint_max_speeds = values
        return tuple(values)

    def get_joint_max_accelerations(self) -> tuple[float, ...]:
        """Return and cache controller maximum accelerations in rad/s²."""

        values = self._positive_vector(
            self._config.getJointMaxAccelerations(),
            self.JOINT_COUNT,
            "joint maximum accelerations",
        )
        self._joint_max_accelerations = values
        return tuple(values)

    def get_tool_pose(self) -> list[float]:
        """Return TCP pose as metres plus a rotation vector in radians."""

        self._require_open()
        return self._vector(self._state.getTcpPose(), 6, "TCP pose")

    def get_tcp_offset(self) -> list[float]:
        """Return the flange-to-TCP offset in AUBO's SI pose convention."""

        self._require_open()
        return self._vector(self._config.getTcpOffset(), 6, "TCP offset")

    def set_tool_pose(self, tcp: Sequence[float]) -> int:
        """Set the flange-to-TCP offset without moving the robot."""

        self._require_open()
        offset = self._vector(tcp, 6, "tcp")
        return self._check_result(
            self._config.setTcpOffset(offset), "RobotConfig.setTcpOffset"
        )

    def is_steady(self) -> bool:
        """Return the controller's current steady-state flag."""

        self._require_open()
        return bool(self._state.isSteady())

    def is_collision_occurred(self) -> bool:
        """Return the controller's collision flag."""

        self._require_open()
        return bool(self._state.isCollisionOccurred())

    def is_within_safety_limits(self) -> bool:
        """Return the controller's current safety-limit flag."""

        self._require_open()
        return bool(self._state.isWithinSafetyLimits())

    def _joint_target(
        self, values: Sequence[float], name: str = "joint_values"
    ) -> list[float]:
        self._require_open()
        joints = self._vector(values, self.JOINT_COUNT, name)
        if len(self._joint_limits) != self.JOINT_COUNT:
            raise RuntimeError("controller joint limits are not initialized")
        for index, (value, (lower, upper)) in enumerate(
            zip(joints, self._joint_limits), 1
        ):
            if not lower <= value <= upper:
                raise ValueError(
                    f"{name} joint {index} is {value:.6f} rad; controller range "
                    f"is {lower:.6f}..{upper:.6f} rad"
                )
        return joints

    def _motion_parameter(
        self, value: Any, name: str, maximums: tuple[float, ...], *, allow_zero: bool
    ) -> float:
        result = (
            self._nonnegative_scalar(value, name)
            if allow_zero
            else self._positive_scalar(value, name)
        )
        if not maximums:
            raise RuntimeError(f"controller {name} limits are not initialized")
        maximum = min(maximums)
        if result > maximum:
            raise ValueError(
                f"{name} {result} exceeds the controller limit {maximum}"
            )
        return result

    def move_joint(
        self,
        joint_values: Sequence[float],
        speed: float = 0.2,
        acceleration: float = 0.2,
        blend: float = 0.0,
        duration: float = 0.0,
    ) -> int:
        """Execute ``moveJoint(q, a, v, blend, duration)`` in SI units."""

        joints = self._joint_target(joint_values)
        velocity = self._motion_parameter(
            speed, "speed", self._joint_max_speeds, allow_zero=False
        )
        accel = self._motion_parameter(
            acceleration,
            "acceleration",
            self._joint_max_accelerations,
            allow_zero=False,
        )
        blend_value = self._nonnegative_scalar(blend, "blend")
        duration_value = self._nonnegative_scalar(duration, "duration")
        return self._check_result(
            self._motion.moveJoint(
                joints, accel, velocity, blend_value, duration_value
            ),
            "MotionControl.moveJoint",
        )

    def _servo_enabled(self) -> bool:
        self._require_open()
        value = self._motion.isServoModeEnabled()
        if not isinstance(value, (bool, int)):
            raise RuntimeError(
                f"isServoModeEnabled returned an invalid value {value!r}"
            )
        return bool(value)

    def _wait_for_servo_mode(
        self, enabled: bool, timeout: float, poll_interval: float
    ) -> None:
        deadline = _time.monotonic() + timeout
        while True:
            if self._servo_enabled() is enabled:
                return
            remaining = deadline - _time.monotonic()
            if remaining <= 0.0:
                state = "enable" if enabled else "disable"
                raise TimeoutError(
                    f"AUBO servo mode did not {state} within {timeout:.3f} s"
                )
            _time.sleep(min(poll_interval, remaining))

    def start_servo_mode(
        self,
        *,
        timeout: Optional[float] = None,
        poll_interval: Optional[float] = None,
    ) -> int:
        """Explicitly enable servo mode and wait for it for a bounded time."""

        self._require_open()
        wait = (
            self._servo_mode_timeout
            if timeout is None
            else self._nonnegative_scalar(timeout, "timeout")
        )
        interval = (
            self._servo_poll_interval
            if poll_interval is None
            else self._positive_scalar(poll_interval, "poll_interval")
        )
        self._check_result(
            self._motion.setServoMode(True), "MotionControl.setServoMode(True)"
        )
        self._wait_for_servo_mode(True, wait, interval)
        return 0

    def stop_servo_mode(
        self,
        *,
        timeout: Optional[float] = None,
        poll_interval: Optional[float] = None,
    ) -> int:
        """Explicitly disable servo mode and wait for it for a bounded time."""

        self._require_open()
        wait = (
            self._servo_mode_timeout
            if timeout is None
            else self._nonnegative_scalar(timeout, "timeout")
        )
        interval = (
            self._servo_poll_interval
            if poll_interval is None
            else self._positive_scalar(poll_interval, "poll_interval")
        )
        self._check_result(
            self._motion.setServoMode(False),
            "MotionControl.setServoMode(False)",
        )
        self._wait_for_servo_mode(False, wait, interval)
        return 0

    def servo_joint(
        self,
        joint_values: Sequence[float],
        speed: float = 0.2,
        acceleration: float = 0.2,
        time: float = 0.01,
        lookahead_time: float = 0.1,
        gain: int = 200,
        *,
        queue_retries: Optional[int] = None,
    ) -> int:
        """Send one ``servoJoint(q, a, v, t, lookahead, gain)`` sample.

        SDK result code 2 means the controller queue is full.  It is retried a
        bounded number of times and then raised as :class:`AuboControlError`.
        """

        joints = self._joint_target(joint_values)
        if not self._servo_enabled():
            raise RuntimeError(
                "AUBO servo mode is disabled; call start_servo_mode() first"
            )
        velocity = self._motion_parameter(
            speed, "speed", self._joint_max_speeds, allow_zero=True
        )
        accel = self._motion_parameter(
            acceleration,
            "acceleration",
            self._joint_max_accelerations,
            allow_zero=True,
        )
        sample_time = self._positive_scalar(time, "time")
        lookahead = self._nonnegative_scalar(lookahead_time, "lookahead_time")
        if isinstance(gain, bool) or not isinstance(gain, int):
            raise TypeError("gain must be an integer")
        if gain <= 0:
            raise ValueError("gain must be positive")
        retries = (
            self._servo_queue_retries
            if queue_retries is None
            else self._nonnegative_integer(queue_retries, "queue_retries")
        )

        operation = "MotionControl.servoJoint"
        for attempt in range(retries + 1):
            result = self._motion.servoJoint(
                joints, accel, velocity, sample_time, lookahead, gain
            )
            code = self._return_code(result, operation)
            if code == 0:
                return 0
            if code != self.QUEUE_FULL_CODE or attempt == retries:
                raise AuboControlError(operation, code)
            if self._servo_retry_interval:
                _time.sleep(self._servo_retry_interval)
        raise AssertionError("unreachable servo retry state")

    def stop(self, a: float = 2.0, asynchronous: bool = False) -> int:
        """Request a controlled joint stop using ``stopJoint(acc)``."""

        self._require_open()
        if not isinstance(asynchronous, bool):
            raise TypeError("asynchronous must be a bool")
        acceleration = self._motion_parameter(
            a,
            "acceleration",
            self._joint_max_accelerations,
            allow_zero=False,
        )
        return self._check_result(
            self._motion.stopJoint(acceleration), "MotionControl.stopJoint"
        )

    def stop_script(self) -> int:
        """Compatibility cleanup for streamed control, without disconnecting.

        ARCS RPC motion does not upload a UR-style controller script.  Its safe
        equivalent is a controlled joint stop followed by leaving servo mode.
        """

        if self._closed:
            return 0
        stop_error: Optional[BaseException] = None
        try:
            self.stop()
        except BaseException as exc:
            stop_error = exc
        servo_error: Optional[BaseException] = None
        try:
            if self._servo_enabled():
                self.stop_servo_mode()
        except BaseException as exc:
            servo_error = exc
        if stop_error is not None:
            raise stop_error
        if servo_error is not None:
            raise servo_error
        return 0

    def close(self) -> int:
        """Leave servo mode, log out and disconnect; repeated calls are safe."""

        if self._closed:
            return 0

        errors: list[BaseException] = []
        try:
            if self._servo_enabled():
                self.stop_servo_mode()
        except BaseException as exc:
            errors.append(exc)

        if self._logged_in:
            try:
                logout = getattr(self._client, "logout", None)
                if logout is not None:
                    self._check_result(logout(), "RpcClient.logout")
            except BaseException as exc:
                errors.append(exc)
            finally:
                self._logged_in = False

        if self._connected:
            try:
                disconnect = getattr(self._client, "disconnect", None)
                if disconnect is not None:
                    self._check_result(disconnect(), "RpcClient.disconnect")
            except BaseException as exc:
                errors.append(exc)
            finally:
                self._connected = False

        self._closed = True
        if errors:
            raise errors[0]
        return 0

    def __enter__(self) -> "AuboControl":
        self._require_open()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> bool:
        try:
            self.close()
        except BaseException:
            if exc_type is None:
                raise
        return False


# Model-oriented aliases keep application imports concise while retaining one
# implementation and one tested behaviour surface.
AuboI7 = AuboControl
AUBOI7 = AuboControl
AUBO_i7 = AuboControl


__all__ = (
    "AUBOI7",
    "AUBO_i7",
    "AuboControl",
    "AuboControlError",
    "AuboError",
    "AuboI7",
    "AuboModelError",
)
