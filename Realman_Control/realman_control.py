"""RM65-B control wrapper built on RealMan Python API2.

The public pose and joint conventions intentionally match ``UR10_RTDE/rtde``
so this class can be used by the existing teleoperation layer:

* joints: radians
* Cartesian pose: ``[x, y, z, rx, ry, rz]`` in metres and rotation-vector
  radians

RealMan API2 uses joint degrees and XYZ Euler angles.  This module performs the
conversion at the boundary.  Planned moves use RealMan's percentage speed,
while online joint/pose servoing uses the CAN-FD pass-through interfaces.

Official API reference:
https://develop.realman-robotics.com/robot/apipython/getStarted/
https://develop.realman-robotics.com/robot/apipython/classes/movePlan/
"""

from __future__ import annotations

import math
from typing import Any, Optional, Sequence

try:
    from Robotic_Arm import rm_robot_interface as _rm
except ImportError as exc:  # Allow the module to be inspected without the SDK.
    _rm = None
    _SDK_IMPORT_ERROR: Optional[ImportError] = exc
else:
    _SDK_IMPORT_ERROR = None


class RealManError(RuntimeError):
    """An error code returned by RealMan API2."""

    def __init__(self, operation: str, code: int):
        self.operation = operation
        self.code = code
        super().__init__(f"{operation} failed with RealMan API2 error code {code}")


class RealmanControl:
    """Control a six-axis RealMan RM65-B through Python API2.

    Parameters
    ----------
    robot_ip:
        IP address of the robot controller.
    port:
        API2 controller port.  RealMan examples use port 8080.
    log_level:
        SDK log level: 0 debug, 1 info, 2 warning, 3 error.
    thread_mode:
        Optional SDK ``rm_thread_mode_e`` value.  Triple-thread mode is used by
        default so asynchronous SDK data can be received.
    """

    JOINT_COUNT = 6
    DEFAULT_PORT = 8080
    MAX_LINEAR_SPEED = 0.6
    MAX_ANGULAR_SPEED = 1.2
    # Limits from the supplied RM65-B/urdf/RM65-B.urdf reference, in radians.
    JOINT_LIMITS = (
        (-3.1060, 3.1060),
        (-2.2689, 2.2689),
        (-2.3560, 2.3560),
        (-3.1060, 3.1060),
        (-2.2340, 2.2340),
        (-6.2800, 6.2800),
    )

    def __init__(
        self,
        robot_ip: str = "192.168.1.18",
        port: int = DEFAULT_PORT,
        log_level: int = 3,
        thread_mode: Any = None,
    ) -> None:
        if _rm is None:
            raise RuntimeError(
                "RealMan Python API2 is not installed. Install it with "
                "`pip install Robotic_Arm`."
            ) from _SDK_IMPORT_ERROR

        if not robot_ip:
            raise ValueError("robot_ip must not be empty")
        if not 0 < int(port) <= 65535:
            raise ValueError("port must be in the range 1..65535")
        if int(log_level) not in range(4):
            raise ValueError("log_level must be 0, 1, 2, or 3")

        if thread_mode is None:
            thread_mode = _rm.rm_thread_mode_e.RM_TRIPLE_MODE_E

        self.robot_ip = robot_ip
        self.port = int(port)
        self._arm = _rm.RoboticArm(thread_mode)
        self._handle = self._arm.rm_create_robot_arm(
            self.robot_ip, self.port, int(log_level)
        )
        self._closed = False
        self._velocity_stream_config: Optional[tuple[int, int, int]] = None
        self._joint_limits = tuple(self.JOINT_LIMITS)

        handle_id = getattr(self._handle, "id", -1)
        if handle_id < 0:
            self._closed = True
            try:
                self._arm.rm_delete_robot_arm()
            finally:
                raise ConnectionError(
                    f"Failed to connect to RM65-B at {self.robot_ip}:{self.port}; "
                    f"SDK handle id is {handle_id}"
                )

    @property
    def arm(self) -> Any:
        """Return the underlying ``RoboticArm`` object for advanced API2 use."""

        self._require_open()
        return self._arm

    @staticmethod
    def _vector(values: Sequence[float], length: int, name: str) -> list[float]:
        if isinstance(values, (str, bytes)):
            raise TypeError(f"{name} must be a numeric sequence")
        try:
            result = [float(value) for value in values]
        except (TypeError, ValueError) as exc:
            raise TypeError(f"{name} must be a numeric sequence") from exc
        if len(result) != length:
            raise ValueError(f"{name} must contain exactly {length} values")
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"{name} must contain only finite values")
        return result

    @staticmethod
    def _percentage(value: float, name: str) -> int:
        result = int(round(float(value)))
        if not 1 <= result <= 100:
            raise ValueError(f"{name} must be in the range 1..100")
        return result

    @staticmethod
    def _blend_percentage(value: float) -> int:
        result = int(round(float(value)))
        if not 0 <= result <= 100:
            raise ValueError("blend_percent must be in the range 0..100")
        return result

    def _joint_targets(
        self, values: Sequence[float], name: str = "joint_values"
    ) -> list[float]:
        joints = self._vector(values, self.JOINT_COUNT, name)
        for index, (value, limits) in enumerate(
            zip(joints, self._joint_limits), 1
        ):
            lower, upper = limits
            if not lower <= value <= upper:
                raise ValueError(
                    f"{name} joint {index} is {value:.6f} rad; expected "
                    f"{lower:.4f}..{upper:.4f} rad"
                )
        return joints

    @staticmethod
    def _euler_xyz_to_rotvec(euler: Sequence[float]) -> list[float]:
        """Convert fixed-axis XYZ roll/pitch/yaw to a rotation vector."""

        roll, pitch, yaw = RealmanControl._vector(euler, 3, "euler")
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        # q and -q describe the same rotation.  Positive w gives the principal
        # rotation vector with magnitude no greater than pi.
        if qw < 0.0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        vector_norm = math.sqrt(qx * qx + qy * qy + qz * qz)
        if vector_norm < 1e-12:
            return [2.0 * qx, 2.0 * qy, 2.0 * qz]
        angle = 2.0 * math.atan2(vector_norm, qw)
        scale = angle / vector_norm
        return [qx * scale, qy * scale, qz * scale]

    @staticmethod
    def _rotvec_to_euler_xyz(rotvec: Sequence[float]) -> list[float]:
        """Convert a rotation vector to fixed-axis XYZ roll/pitch/yaw."""

        rx, ry, rz = RealmanControl._vector(rotvec, 3, "rotation vector")
        angle = math.sqrt(rx * rx + ry * ry + rz * rz)
        if angle < 1e-12:
            qx, qy, qz, qw = rx / 2.0, ry / 2.0, rz / 2.0, 1.0
        else:
            scale = math.sin(angle / 2.0) / angle
            qx, qy, qz = rx * scale, ry * scale, rz * scale
            qw = math.cos(angle / 2.0)

        sin_roll = 2.0 * (qw * qx + qy * qz)
        cos_roll = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sin_roll, cos_roll)

        sin_pitch = 2.0 * (qw * qy - qz * qx)
        pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

        sin_yaw = 2.0 * (qw * qz + qx * qy)
        cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(sin_yaw, cos_yaw)
        return [roll, pitch, yaw]

    @classmethod
    def _to_realman_pose(cls, pose: Sequence[float]) -> list[float]:
        pose_vector = cls._vector(pose, 6, "pose")
        return pose_vector[:3] + cls._rotvec_to_euler_xyz(pose_vector[3:])

    @classmethod
    def _from_realman_pose(cls, pose: Sequence[float]) -> list[float]:
        pose_vector = cls._vector(pose, 6, "RealMan pose")
        return pose_vector[:3] + cls._euler_xyz_to_rotvec(pose_vector[3:])

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("RM65-B connection is closed")

    @staticmethod
    def _check_code(code: int, operation: str) -> int:
        code = int(code)
        if code != 0:
            raise RealManError(operation, code)
        return code

    def _checked_call(self, operation: str, method: Any, *args: Any) -> int:
        self._require_open()
        return self._check_code(method(*args), operation)

    def get_joint_values(self) -> list[float]:
        """Return the six joint positions in radians."""

        self._require_open()
        code, joint_degrees = self._arm.rm_get_joint_degree()
        self._check_code(code, "rm_get_joint_degree")
        joints = self._vector(joint_degrees, self.JOINT_COUNT, "joint state")
        return [math.radians(value) for value in joints]

    def get_joint_limits(self) -> tuple[tuple[float, float], ...]:
        """Read and cache the controller's configured joint position limits.

        These are the software limits shown/configured through the RealMan
        teach pendant.  API2 returns degrees; the wrapper exposes radians.
        Subsequent motion validation uses this cached controller value rather
        than the model fallback in :attr:`JOINT_LIMITS`.
        """

        self._require_open()
        min_code, minimum_degrees = self._arm.rm_get_joint_min_pos()
        self._check_code(min_code, "rm_get_joint_min_pos")
        max_code, maximum_degrees = self._arm.rm_get_joint_max_pos()
        self._check_code(max_code, "rm_get_joint_max_pos")
        minimum = self._vector(
            minimum_degrees, self.JOINT_COUNT, "controller joint minimums"
        )
        maximum = self._vector(
            maximum_degrees, self.JOINT_COUNT, "controller joint maximums"
        )
        limits = tuple(
            (math.radians(lower), math.radians(upper))
            for lower, upper in zip(minimum, maximum)
        )
        for index, (lower, upper) in enumerate(limits, 1):
            if lower >= upper:
                raise RuntimeError(
                    f"controller returned invalid limits for J{index}: "
                    f"{minimum[index - 1]:.3f}..{maximum[index - 1]:.3f} deg"
                )
        self._joint_limits = limits
        return tuple(limits)

    def get_joint_speed(self) -> list[float]:
        """Explain why synchronous API2 cannot provide joint velocity here."""

        raise NotImplementedError(
            "RealMan API2 exposes joint speed through the configured UDP realtime "
            "push callback (joint_status.joint_speed), not through the synchronous "
            "state query used by this wrapper."
        )

    def get_tool_pose(self) -> list[float]:
        """Return TCP pose as metres plus a rotation vector in radians."""

        self._require_open()
        code, state = self._arm.rm_get_current_arm_state()
        self._check_code(code, "rm_get_current_arm_state")
        if not isinstance(state, dict) or "pose" not in state:
            raise RuntimeError("rm_get_current_arm_state returned no pose")
        return self._from_realman_pose(state["pose"])

    def get_active_tool_frame_pose_euler(self) -> list[float]:
        """Return the active flange-to-TCP offset as XYZ plus XYZ Euler angles.

        This deliberately preserves RealMan's native Euler convention because
        the decoupled RM65 kinematics uses it to construct a TCP transform.
        Position is in metres and angles are in radians.
        """

        self._require_open()
        code, frame = self._arm.rm_get_current_tool_frame()
        self._check_code(code, "rm_get_current_tool_frame")
        if not isinstance(frame, dict) or "pose" not in frame:
            raise RuntimeError("rm_get_current_tool_frame returned no pose")
        return self._vector(frame["pose"], 6, "active tool frame pose")

    def validate_joint_target(
        self,
        joint_values: Sequence[float],
        *,
        singular_value_limit: float = 0.01,
        check_self_collision: bool = True,
    ) -> int:
        """Reject joint targets that are singular, out of range or self-colliding.

        This is a read-only algorithm-layer check; it does not send a motion
        command.  Joint input uses the wrapper's public radians convention.
        """

        self._require_open()
        joints = self._joint_targets(joint_values)
        threshold = float(singular_value_limit)
        if not math.isfinite(threshold) or not 0.0 < threshold < 1.0:
            raise ValueError("singular_value_limit must be in (0, 1)")
        joint_degrees = [math.degrees(value) for value in joints]

        singular = int(
            self._arm.rm_algo_universal_singularity_analyse(
                joint_degrees, threshold
            )
        )
        if singular == -1:
            kind, distance = self._arm.rm_algo_kin_robot_singularity_analyse(
                joint_degrees
            )
            labels = {-1: "shoulder", -2: "elbow", -3: "wrist"}
            label = labels.get(int(kind), "Jacobian")
            raise ValueError(
                f"joint target is near a {label} singularity "
                f"(shoulder distance {float(distance):.4f} m)"
            )
        if singular != 0:
            raise RuntimeError(
                "rm_algo_universal_singularity_analyse failed with "
                f"code {singular}"
            )

        if check_self_collision:
            collision = int(
                self._arm.rm_algo_safety_robot_self_collision_detection(
                    joint_degrees
                )
            )
            if collision == 1:
                raise ValueError("joint target is self-colliding")
            if collision != 0:
                raise RuntimeError(
                    "rm_algo_safety_robot_self_collision_detection failed with "
                    f"code {collision}"
                )
        return 0

    def get_tool_speed(self) -> list[float]:
        """Report that API2 has no equivalent synchronous TCP-speed query."""

        raise NotImplementedError(
            "RealMan API2 does not provide a synchronous TCP-speed query equivalent "
            "to UR RTDE getActualTCPSpeed()."
        )

    def set_tool_pose(
        self,
        tcp: Sequence[float],
        tool_name: str = "teleop_tcp",
        payload: float = 0.0,
        center_of_mass: Sequence[float] = (0.0, 0.0, 0.0),
        activate: bool = True,
    ) -> int:
        """Create a named tool frame from a rotation-vector TCP offset.

        This is the RealMan equivalent of configuring a UR TCP; it does not move
        the robot.  ``payload`` is in kilograms and ``center_of_mass`` in metres.
        """

        self._require_open()
        if not tool_name or len(tool_name.encode("utf-8")) > 10:
            raise ValueError("tool_name must contain 1..10 UTF-8 bytes")
        if float(payload) < 0.0:
            raise ValueError("payload must be non-negative")
        cog = self._vector(center_of_mass, 3, "center_of_mass")
        frame = _rm.rm_frame_t(
            tool_name,
            self._to_realman_pose(tcp),
            float(payload),
            cog[0],
            cog[1],
            cog[2],
        )
        self._checked_call(
            "rm_set_manual_tool_frame", self._arm.rm_set_manual_tool_frame, frame
        )
        if activate:
            self._checked_call(
                "rm_change_tool_frame", self._arm.rm_change_tool_frame, tool_name
            )
        return 0

    def move_joint(
        self,
        joint_values: Sequence[float],
        speed_percent: float = 20,
        acceleration: Optional[float] = None,
        asynchronous: bool = False,
    ) -> int:
        """Run a planned joint move.

        Joint targets are radians.  RealMan accepts a 1..100 speed/acceleration
        percentage rather than separate physical speed and acceleration values;
        therefore ``acceleration`` is accepted for interface compatibility but
        has no separate API2 mapping.
        """

        del acceleration
        joints = self._joint_targets(joint_values)
        joint_degrees = [math.degrees(value) for value in joints]
        return self._checked_call(
            "rm_movej",
            self._arm.rm_movej,
            joint_degrees,
            self._percentage(speed_percent, "speed_percent"),
            0,
            0,
            0 if asynchronous else 1,
        )

    def move_joint_trajectory(
        self,
        path: Sequence[Sequence[float]],
        speed_percent: float = 20,
        blend_percent: float = 0,
        asynchronous: bool = False,
    ) -> int:
        """Run connected planned joint-space waypoints in radians."""

        points = list(path)
        if not points:
            raise ValueError("path must contain at least one waypoint")
        speed = self._percentage(speed_percent, "speed_percent")
        blend = self._blend_percentage(blend_percent)
        for index, point in enumerate(points):
            joints = self._joint_targets(point, f"path[{index}]")
            joint_degrees = [math.degrees(value) for value in joints]
            is_last = index == len(points) - 1
            self._checked_call(
                "rm_movej",
                self._arm.rm_movej,
                joint_degrees,
                speed,
                0 if is_last else blend,
                0 if is_last else 1,
                0 if asynchronous or not is_last else 1,
            )
        return 0

    def speed_joint(
        self,
        speeds: Sequence[float],
        acceleration: float = 0.5,
        time: float = 0.0,
    ) -> int:
        """Reject unsafe emulation of UR's joint-velocity command."""

        del speeds, acceleration, time
        raise NotImplementedError(
            "RM65-B API2 has no direct equivalent of UR speedJ. Use planned "
            "joint targets or generate safe position targets for servo_joint()."
        )

    def move_tool(
        self,
        pose: Sequence[float],
        speed_percent: float = 20,
        acceleration: Optional[float] = None,
        asynchronous: bool = False,
    ) -> int:
        """Run a planned linear TCP move using a rotation-vector pose."""

        del acceleration
        return self._checked_call(
            "rm_movel",
            self._arm.rm_movel,
            self._to_realman_pose(pose),
            self._percentage(speed_percent, "speed_percent"),
            0,
            0,
            0 if asynchronous else 1,
        )

    def move_tool_trajectory(
        self,
        path: Sequence[Sequence[float]],
        speed_percent: float = 20,
        blend_percent: float = 0,
        asynchronous: bool = False,
    ) -> int:
        """Run connected linear TCP waypoints using rotation-vector poses."""

        points = list(path)
        if not points:
            raise ValueError("path must contain at least one waypoint")
        speed = self._percentage(speed_percent, "speed_percent")
        blend = self._blend_percentage(blend_percent)
        for index, point in enumerate(points):
            is_last = index == len(points) - 1
            self._checked_call(
                "rm_movel",
                self._arm.rm_movel,
                self._to_realman_pose(point),
                speed,
                0 if is_last else blend,
                0 if is_last else 1,
                0 if asynchronous or not is_last else 1,
            )
        return 0

    @staticmethod
    def _validate_stream_options(trajectory_mode: int, radio: int) -> tuple[int, int]:
        mode = int(trajectory_mode)
        parameter = int(radio)
        if mode not in (0, 1, 2):
            raise ValueError("trajectory_mode must be 0, 1, or 2")
        upper_limit = 1000 if mode == 2 else 100
        if not 0 <= parameter <= upper_limit:
            raise ValueError(f"radio must be in the range 0..{upper_limit}")
        return mode, parameter

    def speed_tool(
        self,
        speeds: Sequence[float],
        acceleration: float = 0.25,
        time: float = 0.01,
        *,
        avoid_singularity: bool = True,
        frame_type: int = 1,
        high_follow: bool = False,
        trajectory_mode: int = 0,
        radio: int = 0,
    ) -> int:
        """Send one Cartesian-velocity pass-through sample.

        Call this method repeatedly at ``time`` intervals.  Linear values are
        m/s and angular values rad/s.  API2 limits are enforced before sending.
        ``frame_type`` is 1 for the work frame and 0 for the tool frame.
        """

        del acceleration
        velocity = self._vector(speeds, 6, "speeds")
        if any(abs(value) > self.MAX_LINEAR_SPEED for value in velocity[:3]):
            raise ValueError(
                f"linear speed magnitude per axis must not exceed "
                f"{self.MAX_LINEAR_SPEED} m/s"
            )
        if any(abs(value) > self.MAX_ANGULAR_SPEED for value in velocity[3:]):
            raise ValueError(
                f"angular speed magnitude per axis must not exceed "
                f"{self.MAX_ANGULAR_SPEED} rad/s"
            )
        if time <= 0.0:
            raise ValueError("time must be positive")
        if int(frame_type) not in (0, 1):
            raise ValueError("frame_type must be 0 (tool) or 1 (work)")
        dt_ms = max(1, int(round(time * 1000.0)))
        mode, parameter = self._validate_stream_options(trajectory_mode, radio)
        config = (int(bool(avoid_singularity)), int(frame_type), dt_ms)
        if config != self._velocity_stream_config:
            self._checked_call(
                "rm_set_movev_canfd_init",
                self._arm.rm_set_movev_canfd_init,
                config[0],
                config[1],
                config[2],
            )
            self._velocity_stream_config = config
        return self._checked_call(
            "rm_movev_canfd",
            self._arm.rm_movev_canfd,
            velocity,
            bool(high_follow),
            mode,
            parameter,
        )

    def servo_joint(
        self,
        joint_values: Sequence[float],
        speed: float = 0,
        acceleration: float = 0,
        time: float = 0.01,
        lookahead_time: float = 0.1,
        gain: int = 300,
        *,
        high_follow: bool = False,
        trajectory_mode: int = 1,
        radio: int = 50,
    ) -> int:
        """Send one CAN-FD joint-position pass-through sample.

        The caller owns timing; API2 does not consume ``speed``, ``acceleration``,
        ``lookahead_time`` or ``gain``.  Set ``high_follow=True`` only when the
        caller can reliably keep the command period at or below 10 ms.
        """

        del speed, acceleration, lookahead_time, gain
        if time <= 0.0:
            raise ValueError("time must be positive")
        if high_follow and time > 0.010:
            raise ValueError("high-follow CAN-FD servoing requires time <= 0.010 s")
        mode, parameter = self._validate_stream_options(trajectory_mode, radio)
        joints = self._joint_targets(joint_values)
        joint_degrees = [math.degrees(value) for value in joints]
        return self._checked_call(
            "rm_movej_canfd",
            self._arm.rm_movej_canfd,
            joint_degrees,
            bool(high_follow),
            0.0,
            mode,
            parameter,
        )

    def servo_tool(
        self,
        pose: Sequence[float],
        speed: float = 0,
        acceleration: float = 0,
        time: float = 0.01,
        lookahead_time: float = 0.1,
        gain: int = 300,
        *,
        high_follow: bool = False,
        trajectory_mode: int = 1,
        radio: int = 50,
    ) -> int:
        """Send one CAN-FD TCP-pose pass-through sample.

        This is the RM65-B equivalent used by ``VRTracker.track()``.  The input
        pose uses metres and a rotation vector, then is converted to the Euler
        pose required by ``rm_movep_canfd``.
        """

        del speed, acceleration, lookahead_time, gain
        if time <= 0.0:
            raise ValueError("time must be positive")
        if high_follow and time > 0.010:
            raise ValueError("high-follow CAN-FD servoing requires time <= 0.010 s")
        mode, parameter = self._validate_stream_options(trajectory_mode, radio)
        return self._checked_call(
            "rm_movep_canfd",
            self._arm.rm_movep_canfd,
            self._to_realman_pose(pose),
            bool(high_follow),
            mode,
            parameter,
        )

    def stop(self, a: float = 2.0, asynchronous: bool = False) -> int:
        """Request a controlled slow stop of the active trajectory."""

        del a, asynchronous
        return self._checked_call(
            "rm_set_arm_slow_stop", self._arm.rm_set_arm_slow_stop
        )

    def close(self) -> int:
        """Disconnect this robot instance.  Calling it repeatedly is safe."""

        if self._closed:
            return 0
        self._closed = True
        return self._check_code(
            self._arm.rm_delete_robot_arm(), "rm_delete_robot_arm"
        )

    def stop_script(self) -> int:
        """Compatibility cleanup: controlled stop, then disconnect API2."""

        if self._closed:
            return 0
        stop_error: Optional[BaseException] = None
        try:
            self.stop()
        except BaseException as exc:  # Always attempt to release the connection.
            stop_error = exc
        close_error: Optional[BaseException] = None
        try:
            self.close()
        except BaseException as exc:
            close_error = exc
        if stop_error is not None:
            raise stop_error
        if close_error is not None:
            raise close_error
        return 0

    def __enter__(self) -> "RealmanControl":
        self._require_open()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> bool:
        try:
            self.stop_script()
        except BaseException:
            if exc_type is None:
                raise
        return False


# Short model-specific alias for callers that prefer the robot model name.
RM65B = RealmanControl
