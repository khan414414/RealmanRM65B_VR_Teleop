"""VR-to-AUBO-i7 tracking with position-only IK and an independent wrist."""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Optional, Sequence

import numpy as np

from teleop.aubo_i7_collision import AuboI7CollisionChecker
from teleop.aubo_i7_decoupled_ik import (
    AUBO_I7_JOINT_LIMITS_RAD,
    AuboI7DecoupledIK,
    ContinuousControllerTwist,
    IKOptions,
    forward_kinematics,
    position_jacobian,
    transform_from_xyz_rotvec,
)


class TeleoperationSafetyWarning(RuntimeError):
    """Tracking could not advance without requesting a robot stop."""


class VRTracker:
    """Map controller XYZ to J1..J5 and controller twist exclusively to J6.

    Targets are sent through ``servo_joint`` instead of ``servo_tool``, so the
    robot controller cannot redistribute wrist roll between J4, J5 and J6.
    """

    def __init__(
        self,
        rtde,
        gripper,
        rate: float = 0.008,
        smooth_step: float = 0.1,
        pos_mapping: Sequence[float] = (1.0, 1.0, 1.0),
        *,
        wrist_roll_axis: Sequence[float] = (1.0, 0.0, 0.0),
        wrist_roll_sign: float = 1.0,
        wrist_roll_gain: float = 1.0,
        max_joint_speed_rad_s: float = math.radians(90.0),
        max_cartesian_speed_m_s: float = 0.15,
        safety_approach_scale: float = 0.025,
        max_position_offset_m: Optional[float] = 0.25,
        joint_limit_margin_rad: float = math.radians(5.0),
        wrist_limit_margin_rad: float = math.radians(15.0),
        singular_value_limit: float = 0.01,
        max_following_error_rad: float = math.radians(10.0),
        following_error_check_period_s: float = 0.1,
        following_error_settle_time_s: float = 0.15,
        ik_solver: Optional[AuboI7DecoupledIK] = None,
        collision_checker: Optional[AuboI7CollisionChecker] = None,
    ) -> None:
        self.rtde = rtde
        self.gripper = gripper
        self.rate = float(rate)
        self.smooth_step = float(smooth_step)
        self._position_mapping_matrix = self._mapping_matrix(pos_mapping)
        self.wrist_roll_sign = float(wrist_roll_sign)
        self.wrist_roll_gain = float(wrist_roll_gain)
        self.max_joint_speed_rad_s = float(max_joint_speed_rad_s)
        self.max_cartesian_speed_m_s = float(max_cartesian_speed_m_s)
        self.safety_approach_scale = float(safety_approach_scale)
        # None (or a non-finite value) disables the workspace sphere entirely;
        # the target is then only bounded by joint limits and speed limits.
        if max_position_offset_m is None:
            self.max_position_offset_m = math.inf
        else:
            self.max_position_offset_m = float(max_position_offset_m)
        self.joint_limit_margin_rad = float(joint_limit_margin_rad)
        self.wrist_limit_margin_rad = float(wrist_limit_margin_rad)
        self.singular_value_limit = float(singular_value_limit)
        self.max_following_error_rad = float(max_following_error_rad)
        self.following_error_check_period_s = float(
            following_error_check_period_s
        )
        self.following_error_settle_time_s = float(
            following_error_settle_time_s
        )

        if not math.isfinite(self.rate) or self.rate <= 0.0:
            raise ValueError("rate must be positive")
        if not 0.0 < self.smooth_step <= 1.0:
            raise ValueError("smooth_step must be in (0, 1]")
        if self.wrist_roll_sign not in (-1.0, 1.0):
            raise ValueError("wrist_roll_sign must be +1 or -1")
        if not 0.0 < self.safety_approach_scale < 1.0:
            raise ValueError("safety_approach_scale must be in (0, 1)")
        positive = (
            self.wrist_roll_gain,
            self.max_joint_speed_rad_s,
            self.max_cartesian_speed_m_s,
            self.max_position_offset_m,
            self.joint_limit_margin_rad,
            self.wrist_limit_margin_rad,
            self.singular_value_limit,
            self.max_following_error_rad,
            self.following_error_check_period_s,
            self.following_error_settle_time_s,
        )
        # math.inf (workspace sphere disabled) is an allowed upper bound.
        if not all(
            (math.isfinite(value) or math.isinf(value) and value > 0.0)
            and value > 0.0
            for value in positive
        ):
            raise ValueError("tracking gains, limits and thresholds must be positive")

        self._maximum_joint_step = self.max_joint_speed_rad_s * self.rate
        self._maximum_cartesian_step = self.max_cartesian_speed_m_s * self.rate
        if ik_solver is None:
            joint_limits = np.asarray(
                AUBO_I7_JOINT_LIMITS_RAD, dtype=np.float64
            )
            self.joint_limit_source = "AUBO i7_30 model fallback"
            get_joint_limits = getattr(rtde, "get_joint_limits", None)
            if get_joint_limits is not None:
                joint_limits = np.asarray(
                    get_joint_limits(), dtype=np.float64
                )
                self.joint_limit_source = "controller/teach pendant"
            tcp_transform = np.eye(4, dtype=np.float64)
            get_tcp_offset = getattr(rtde, "get_tcp_offset", None)
            if get_tcp_offset is not None:
                tcp_transform = transform_from_xyz_rotvec(get_tcp_offset())
            ik_solver = AuboI7DecoupledIK(
                tcp_transform=tcp_transform,
                joint_limits_rad=joint_limits,
                options=IKOptions(
                    position_tolerance_m=2.0e-4,
                    max_solution_delta_rad=self._maximum_joint_step,
                    max_wrist_step_rad=self._maximum_joint_step,
                    joint_limit_margin_rad=self.joint_limit_margin_rad,
                ),
            )
        else:
            self.joint_limit_source = "provided IK solver"
        self.ik = ik_solver
        self.collision_checker = (
            AuboI7CollisionChecker()
            if collision_checker is None
            else collision_checker
        )
        if not callable(getattr(self.collision_checker, "validate", None)):
            raise TypeError("collision_checker must provide validate(joints, tcp)")
        self.joint_limits_rad = self.ik.joint_limits_rad
        margins = np.full(6, self.joint_limit_margin_rad, dtype=np.float64)
        margins[5] = self.wrist_limit_margin_rad
        safe_lower = self.joint_limits_rad[:, 0] + margins
        safe_upper = self.joint_limits_rad[:, 1] - margins
        if np.any(
            safe_lower >= safe_upper
        ):
            raise ValueError("joint safety margins leave no usable joint range")
        self._safe_joint_limits_rad = np.column_stack((safe_lower, safe_upper))
        self.twist = ContinuousControllerTwist(axis=wrist_roll_axis)

        self.paused = True
        self.last_warning_reason: Optional[str] = None
        self.gripper_open = True
        self.input_anchor: Optional[np.ndarray] = None
        self.tool_anchor_position: Optional[np.ndarray] = None
        self.target_position: Optional[np.ndarray] = None
        self.requested_position: Optional[np.ndarray] = None
        self.requested_wrist: Optional[float] = None
        self.joint_anchor: Optional[np.ndarray] = None
        self.command_joints: Optional[np.ndarray] = None
        self._next_following_error_check_at: Optional[float] = None
        self._following_error_blocked = False
        self._following_error_reason: Optional[str] = None
        self._command_history: deque[tuple[float, np.ndarray]] = deque(
            maxlen=4096
        )

    @staticmethod
    def _controller_input(user_input: Sequence[float]) -> np.ndarray:
        result = np.asarray(user_input, dtype=np.float64)
        if result.shape != (7,) or not np.all(np.isfinite(result)):
            raise ValueError("controller input must contain seven finite values")
        quaternion_norm = float(np.linalg.norm(result[3:]))
        if quaternion_norm < 1.0e-9:
            raise ValueError("controller quaternion has near-zero norm")
        result = result.copy()
        result[3:] /= quaternion_norm
        return result

    @staticmethod
    def _mapping_matrix(pos_mapping: Sequence[float]) -> np.ndarray:
        result = np.asarray(pos_mapping, dtype=np.float64)
        if result.shape == (3,):
            result = np.diag(result)
        if result.shape != (3, 3) or not np.all(np.isfinite(result)):
            raise ValueError("pos_mapping must be a finite length-3 or 3x3 value")
        return result.copy()

    @property
    def position_mapping_matrix(self) -> np.ndarray:
        """Return the current VR-position to AUBO-base mapping."""

        return self._position_mapping_matrix.copy()

    def set_position_mapping(self, pos_mapping: Sequence[float]) -> None:
        """Apply a new mapping only while tracking is safely paused."""

        if not self.paused:
            raise RuntimeError("position mapping can only change while tracking is paused")
        self._position_mapping_matrix = self._mapping_matrix(pos_mapping)

    def _warning(self, reason: str) -> TeleoperationSafetyWarning:
        self.last_warning_reason = reason
        return TeleoperationSafetyWarning(reason)

    @property
    def safe_joint_limits_rad(self) -> np.ndarray:
        """Return the teach-pendant limits after reserving safety margins."""

        return self._safe_joint_limits_rad.copy()

    def _clip_joint_target(self, joints: Sequence[float]) -> np.ndarray:
        """Project one VR-generated command into the safe joint range."""

        target = np.asarray(joints, dtype=np.float64)
        if target.shape != (6,) or not np.all(np.isfinite(target)):
            raise ValueError("joint target must contain six finite values")
        return np.clip(
            target,
            self._safe_joint_limits_rad[:, 0],
            self._safe_joint_limits_rad[:, 1],
        )

    def clear_warning(self) -> None:
        self.last_warning_reason = None

    def _joint_margin_check(self, joints: np.ndarray) -> None:
        if np.any(joints < self._safe_joint_limits_rad[:, 0]) or np.any(
            joints > self._safe_joint_limits_rad[:, 1]
        ):
            raise ValueError("joint target entered the configured limit margin")

    def _model_singularity_check(self, joints: np.ndarray) -> None:
        jacobian = position_jacobian(joints, self.ik.tcp_transform)[:, :5]
        minimum = float(np.linalg.svd(jacobian, compute_uv=False)[-1])
        if minimum < self.singular_value_limit:
            raise ValueError(
                "J1..J5 position Jacobian is ill-conditioned "
                f"(minimum singular value {minimum:.5f})"
            )
        validate = getattr(self.rtde, "validate_joint_target", None)
        if validate is not None:
            validate(joints, singular_value_limit=self.singular_value_limit)

    def _candidate_safety_check(self, joints: np.ndarray) -> None:
        """Apply joint, singularity/controller, self and plane checks."""

        self._joint_margin_check(joints)
        self._model_singularity_check(joints)
        self.collision_checker.validate(joints, self.ik.tcp_transform)

    def _capture_requested_target(self, controller: np.ndarray) -> bool:
        """Store the newest finite controller intent; newer frames replace it.

        The workspace is a hard sphere around the Trigger anchor unless
        ``max_position_offset_m`` is None/infinite, in which case the mapped
        offset is used as-is and only joint/speed limits bound the motion.
        A target outside that sphere is projected to its surface instead of
        throwing the whole frame away.  The projected target is then approached
        through the normal Cartesian and joint rate limits.
        """

        if (
            self.input_anchor is None
            or self.tool_anchor_position is None
            or self.joint_anchor is None
        ):
            raise ValueError("tracking anchors are not initialized")

        relative_translation = self._position_mapping_matrix @ (
            controller[:3] - self.input_anchor[:3]
        )
        translation_norm = float(np.linalg.norm(relative_translation))
        workspace_limited = (
            math.isfinite(self.max_position_offset_m)
            and translation_norm > self.max_position_offset_m
        )
        if workspace_limited:
            relative_translation *= self.max_position_offset_m / translation_norm
        requested_position = self.tool_anchor_position + relative_translation

        controller_twist = self.twist.update(controller[3:])
        safe_wrist_limits = (
            self._safe_joint_limits_rad[5, 0],
            self._safe_joint_limits_rad[5, 1],
        )
        requested_wrist = (
            self.joint_anchor[5]
            + self.wrist_roll_sign * self.wrist_roll_gain * controller_twist
        )
        requested_wrist = float(np.clip(requested_wrist, *safe_wrist_limits))

        # Commit both values together so an invalid quaternion cannot leave a
        # half-updated pending target behind.
        self.requested_position = requested_position.copy()
        self.requested_wrist = requested_wrist
        return workspace_limited

    def _rate_limit_joint_target(
        self, joints: Sequence[float]
    ) -> tuple[np.ndarray, bool]:
        """Scale a six-joint delta as one vector to preserve its direction."""

        if self.command_joints is None:
            raise ValueError("joint command anchor is not initialized")
        target = self._clip_joint_target(joints)
        delta = target - self.command_joints
        largest_step = float(np.max(np.abs(delta)))
        limited = largest_step > self._maximum_joint_step
        if limited:
            delta *= self._maximum_joint_step / largest_step
        return self.command_joints + delta, limited

    def _rate_limit_cartesian_target(
        self,
        joints: np.ndarray,
        maximum_step_m: float,
    ) -> tuple[np.ndarray, np.ndarray, bool]:
        """Shrink a joint delta until its actual FK displacement is bounded."""

        if self.command_joints is None or self.target_position is None:
            raise ValueError("tracking command is not initialized")
        maximum = float(maximum_step_m)
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise ValueError("maximum Cartesian step must be positive")

        delta = np.asarray(joints, dtype=np.float64) - self.command_joints
        candidate = self.command_joints + delta
        candidate_position = forward_kinematics(
            candidate, self.ik.tcp_transform
        )[:3, 3]
        displacement = float(
            np.linalg.norm(candidate_position - self.target_position)
        )
        tolerance = 1.0e-12
        if displacement <= maximum + tolerance:
            return candidate, candidate_position, False

        # The joint delta is already at most one normal command step.  Local
        # FK is therefore smooth enough for conservative ratio backoff; repeat
        # in case its Cartesian response is not perfectly linear.
        scale = maximum / displacement
        for _ in range(12):
            candidate = self.command_joints + scale * delta
            candidate_position = forward_kinematics(
                candidate, self.ik.tcp_transform
            )[:3, 3]
            displacement = float(
                np.linalg.norm(candidate_position - self.target_position)
            )
            if displacement <= maximum + tolerance:
                return candidate, candidate_position, True
            scale *= 0.999999 * maximum / displacement
        raise ValueError("candidate could not satisfy the Cartesian speed limit")

    def _candidate_makes_progress(
        self,
        candidate: np.ndarray,
        candidate_position: np.ndarray,
        planned_position: np.ndarray,
    ) -> bool:
        """Return whether a partial command advances the retained latest goal."""

        if (
            self.command_joints is None
            or self.target_position is None
            or self.requested_position is None
            or self.requested_wrist is None
        ):
            raise ValueError("tracking targets are not initialized")

        tolerance = float(self.ik.options.position_tolerance_m)
        epsilon = 1.0e-10
        current_position_error = float(
            np.linalg.norm(self.requested_position - self.target_position)
        )
        candidate_position_error = float(
            np.linalg.norm(self.requested_position - candidate_position)
        )
        current_planned_error = float(
            np.linalg.norm(planned_position - self.target_position)
        )
        candidate_planned_error = float(
            np.linalg.norm(planned_position - candidate_position)
        )
        current_wrist_error = abs(
            self.requested_wrist - float(self.command_joints[5])
        )
        candidate_wrist_error = abs(
            self.requested_wrist - float(candidate[5])
        )

        position_active = current_position_error > tolerance
        wrist_active = current_wrist_error > epsilon
        if position_active:
            if candidate_position_error >= current_position_error - epsilon:
                return False
            if candidate_planned_error >= current_planned_error - epsilon:
                return False
        elif candidate_position_error > tolerance + epsilon:
            return False

        if wrist_active and candidate_wrist_error > current_wrist_error + epsilon:
            return False
        if not position_active and wrist_active:
            return candidate_wrist_error < current_wrist_error - epsilon
        if not position_active and not wrist_active:
            return bool(
                np.allclose(
                    candidate,
                    self.command_joints,
                    rtol=0.0,
                    atol=1.0e-12,
                )
            )
        return True

    def _validated_approach_candidate(
        self,
        candidate: np.ndarray,
        planned_position: np.ndarray,
        maximum_cartesian_step_m: float,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Choose the largest checked step toward a possibly unsafe target.

        A full rate-limited candidate is tried first.  If its endpoint is near
        a singularity or self-collision, one deliberately slow joint-space
        interpolation is tried.  No invalid endpoint is ever sent.
        """

        if self.command_joints is None:
            raise ValueError("joint command anchor is not initialized")
        delta = candidate - self.command_joints
        full_position = forward_kinematics(
            candidate, self.ik.tcp_transform
        )[:3, 3]
        if not self._candidate_makes_progress(
            candidate, full_position, planned_position
        ):
            raise ValueError(
                "candidate made no progress toward the retained target"
            )
        try:
            self._candidate_safety_check(candidate)
        except ValueError as full_error:
            # Only a safety-check failure activates the very slow fallback.
            # A numerically unrelated/non-improving IK result must not be made
            # sendable merely by shrinking it until its error is hard to see.
            slow_candidate = (
                self.command_joints + self.safety_approach_scale * delta
            )
            slow_candidate, slow_position, _ = self._rate_limit_cartesian_target(
                slow_candidate,
                maximum_cartesian_step_m,
            )
            if not self._candidate_makes_progress(
                slow_candidate, slow_position, planned_position
            ):
                raise ValueError(str(full_error)) from full_error
            try:
                self._candidate_safety_check(slow_candidate)
            except ValueError as slow_error:
                raise ValueError(str(slow_error)) from slow_error
            return slow_candidate, slow_position, self.safety_approach_scale
        return candidate, full_position, 1.0

    def _validated_wrist_only_candidate(
        self,
    ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """Advance J6 safely while J1..J5 remain at their last command."""

        if (
            self.command_joints is None
            or self.target_position is None
            or self.requested_wrist is None
        ):
            return None
        current = self.command_joints
        wrist_error = self.requested_wrist - float(current[5])
        if abs(wrist_error) <= 1.0e-12:
            return None

        candidate = current.copy()
        candidate[5] += float(
            np.clip(
                wrist_error,
                -self._maximum_joint_step,
                self._maximum_joint_step,
            )
        )
        candidate = self._clip_joint_target(candidate)
        candidate, accepted_position, _ = self._rate_limit_cartesian_target(
            candidate,
            self._maximum_cartesian_step,
        )
        if abs(float(candidate[5]) - float(current[5])) <= 1.0e-12:
            return None
        self._candidate_safety_check(candidate)
        return candidate, accepted_position

    def _record_sent_command(self, joints: np.ndarray, sent_at: float) -> None:
        """Record when a distinct target first entered the servo stream."""

        command = np.asarray(joints, dtype=np.float64).copy()
        if self._command_history and np.array_equal(
            command, self._command_history[-1][1]
        ):
            return
        self._command_history.append((float(sent_at), command))

    def _mature_command(self, now: float) -> Optional[np.ndarray]:
        """Return the newest command old enough for low-follow feedback."""

        cutoff = float(now) - self.following_error_settle_time_s
        for sent_at, command in reversed(self._command_history):
            if sent_at <= cutoff:
                return command
        return None

    def _command_envelope(
        self, now: float
    ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """Bound commands consistent with any lag up to the settle time."""

        cutoff = float(now) - self.following_error_settle_time_s
        mature = self._mature_command(now)
        if mature is None:
            return None
        commands = [mature]
        commands.extend(
            command
            for sent_at, command in self._command_history
            if sent_at > cutoff
        )
        if self.command_joints is not None:
            commands.append(self.command_joints)
        stacked = np.stack(commands)
        return np.min(stacked, axis=0), np.max(stacked, axis=0)

    def _following_error_check(self) -> None:
        """Hold commands while feedback is far from the accepted command.

        The synchronous AUBO RPC state query is deliberately throttled so it
        does not run in every servo sample.  J6 is compared as a continuous joint
        angle: reducing its error modulo 2*pi would hide a full-turn mismatch.
        Once blocked, command advancement remains held between feedback checks;
        the newest controller intent is still retained and this module never
        requests a robot stop.
        """

        now = time.monotonic()
        if (
            self._next_following_error_check_at is not None
            and now < self._next_following_error_check_at
        ):
            if self._following_error_blocked:
                raise ValueError(self._following_error_reason)
            return
        actual = np.asarray(self.rtde.get_joint_values(), dtype=np.float64)
        if actual.shape != (6,) or not np.all(np.isfinite(actual)):
            self._following_error_blocked = True
            self._following_error_reason = "robot returned an invalid joint state"
            self._next_following_error_check_at = (
                now + self.following_error_check_period_s
            )
            raise ValueError("robot returned an invalid joint state")
        if self._following_error_blocked:
            # Once a real lag has been detected, require the robot to catch up
            # with the frozen latest target before advancing the VR trajectory.
            error = np.abs(actual - self.command_joints)
        else:
            # Depending on controller tuning, actual feedback can match any
            # command between zero lag and the configured low-follow delay.
            # Test against that recent command envelope: comparing with only
            # the mature endpoint falsely flags a fast-following joint, while
            # comparing with only the latest falsely flags a delayed joint.
            envelope = self._command_envelope(now)
            if envelope is None:
                self._next_following_error_check_at = (
                    now + self.following_error_check_period_s
                )
                return
            lower, upper = envelope
            error = np.maximum(
                np.maximum(lower - actual, actual - upper),
                0.0,
            )
        threshold = self.max_following_error_rad
        if self._following_error_blocked:
            threshold *= 0.8
        if np.any(error > threshold):
            joint = int(np.argmax(error)) + 1
            self._following_error_blocked = True
            self._following_error_reason = (
                f"J{joint} following error is "
                f"{math.degrees(error[joint - 1]):.1f} deg "
                "(outside the recent command envelope for "
                f"{self.following_error_settle_time_s * 1000.0:.0f} ms)"
            )
            self._next_following_error_check_at = (
                now + self.following_error_check_period_s
            )
            raise ValueError(self._following_error_reason)
        self._following_error_blocked = False
        self._following_error_reason = None
        self._next_following_error_check_at = (
            now + self.following_error_check_period_s
        )

    def resume(self, input_anchor: Sequence[float]) -> None:
        """Anchor the hand and the current joint configuration on Trigger press."""

        try:
            controller = self._controller_input(input_anchor)
            joints = np.asarray(self.rtde.get_joint_values(), dtype=np.float64)
            if joints.shape != (6,) or not np.all(np.isfinite(joints)):
                raise ValueError("robot returned an invalid joint state")
            self._candidate_safety_check(joints)

            tcp_position = forward_kinematics(
                joints, self.ik.tcp_transform
            )[:3, 3]
            self.twist.reset(controller[3:])
            self.input_anchor = controller
            self.tool_anchor_position = tcp_position.copy()
            self.target_position = tcp_position.copy()
            self.requested_position = tcp_position.copy()
            self.requested_wrist = float(joints[5])
            self.joint_anchor = joints.copy()
            self.command_joints = joints.copy()
            now = time.monotonic()
            self._next_following_error_check_at = (
                now + self.following_error_check_period_s
            )
            self._following_error_blocked = False
            self._following_error_reason = None
            self._command_history.clear()
            self._record_sent_command(joints, now)
            self.last_warning_reason = None
            self.paused = False
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.paused = True
            raise self._warning(f"cannot start tracking: {exc}") from exc

    def pause(self) -> None:
        self.paused = True
        self.requested_position = None
        self.requested_wrist = None
        self.last_warning_reason = None

    def keepalive(self):
        """Repeat the last safe target without advancing the VR trajectory."""

        if self.paused or self.command_joints is None:
            return None
        self.rtde.servo_joint(self.command_joints, time=self.rate)
        return self.command_joints.copy()

    def track(self, user_input: Sequence[float]):
        """Retain the latest goal and send one checked step toward it."""

        if self.paused:
            return None
        if any(
            value is None
            for value in (
                self.input_anchor,
                self.tool_anchor_position,
                self.target_position,
                self.joint_anchor,
                self.command_joints,
            )
        ):
            self.paused = True
            raise self._warning("tracking anchors are not initialized")

        try:
            controller = self._controller_input(user_input)
            workspace_limited = self._capture_requested_target(controller)
        except (ValueError, np.linalg.LinAlgError) as exc:
            raise self._warning(f"tracking input rejected: {exc}") from exc

        try:
            # Capture the newest intent before checking plant lag.  If feedback
            # is still catching up, the command remains frozen but the latest
            # (not FIFO) target is available as soon as the lag clears.
            self._following_error_check()
        except ValueError as exc:
            raise self._warning(
                f"tracking target retained while command is held: {exc}"
            ) from exc

        try:
            requested_position = self.requested_position
            requested_wrist = self.requested_wrist
            if requested_position is None or requested_wrist is None:
                raise ValueError("latest tracking target is not initialized")
            position_step = self.smooth_step * (
                requested_position - self.target_position
            )
            step_norm = float(np.linalg.norm(position_step))
            remaining_position_error = float(
                np.linalg.norm(requested_position - self.target_position)
            )
            position_tolerance = float(self.ik.options.position_tolerance_m)
            # With exponential smoothing, the requested sub-step can become
            # smaller than the IK tolerance while the retained final target is
            # still outside tolerance.  Make that last step large enough for
            # the solver to observe, without exceeding the velocity limit.
            minimum_solver_step = min(
                self._maximum_cartesian_step,
                1.05 * position_tolerance,
            )
            if (
                remaining_position_error > position_tolerance
                and step_norm < minimum_solver_step
            ):
                position_step = (
                    requested_position - self.target_position
                ) * min(
                    1.0,
                    minimum_solver_step / remaining_position_error,
                )
                step_norm = float(np.linalg.norm(position_step))
            if step_norm > self._maximum_cartesian_step:
                position_step *= self._maximum_cartesian_step / step_norm
            next_target_position = self.target_position + position_step

            result = self.ik.solve(
                next_target_position,
                self.command_joints,
                wrist_joint_6_target_rad=requested_wrist,
                posture_reference_rad=self.joint_anchor,
            )
            candidate, joint_limited = self._rate_limit_joint_target(
                result.joints_rad
            )
            planned_cartesian_step = float(
                np.linalg.norm(next_target_position - self.target_position)
            )
            allowed_cartesian_step = min(
                self._maximum_cartesian_step,
                max(position_tolerance, planned_cartesian_step),
            )
            candidate, _, cartesian_limited = self._rate_limit_cartesian_target(
                candidate,
                allowed_cartesian_step,
            )
            candidate, accepted_position, approach_scale = (
                self._validated_approach_candidate(
                    candidate,
                    next_target_position,
                    allowed_cartesian_step,
                )
            )

            self.rtde.servo_joint(candidate, time=self.rate)
            self.command_joints = candidate.copy()
            self._record_sent_command(self.command_joints, time.monotonic())
            # The planner follows what was actually sent.  A partial IK result
            # must not pretend that the full Cartesian sub-target was reached.
            self.target_position = accepted_position.copy()

            warning_reasons = []
            if workspace_limited:
                warning_reasons.append(
                    "controller target exceeds the tracking workspace; "
                    "approaching the projected workspace boundary"
                )
            if not result.converged or joint_limited or cartesian_limited:
                warning_reasons.append(
                    "target exceeds one-cycle IK/joint capacity; approaching "
                    "the retained target gradually"
                )
            if approach_scale < 1.0:
                warning_reasons.append(
                    "full step crossed a kinematic or collision threshold; "
                    f"using a validated {approach_scale * 100.0:.1f}% step"
                )
            self.last_warning_reason = (
                "; ".join(warning_reasons) if warning_reasons else None
            )
            return candidate.copy()
        except (ValueError, np.linalg.LinAlgError) as exc:
            try:
                wrist_only = self._validated_wrist_only_candidate()
            except (ValueError, np.linalg.LinAlgError) as wrist_error:
                raise self._warning(
                    "latest tracking target retained at the last safe command: "
                    f"{exc}; J6-only fallback was rejected: {wrist_error}"
                ) from wrist_error
            if wrist_only is None:
                raise self._warning(
                    "latest tracking target retained at the last safe command: "
                    f"{exc}"
                ) from exc

            candidate, accepted_position = wrist_only
            self.rtde.servo_joint(candidate, time=self.rate)
            self.command_joints = candidate.copy()
            self._record_sent_command(self.command_joints, time.monotonic())
            self.target_position = accepted_position.copy()
            self.last_warning_reason = (
                "J1..J5 target is held at its last safe position; advancing "
                f"J6 independently ({exc})"
            )
            return candidate.copy()

    def trigger_gripper(self) -> None:
        if self.gripper_open:
            self.gripper.close_gripper()
        else:
            self.gripper.open_gripper()
        self.gripper_open = not self.gripper_open


__all__ = ("TeleoperationSafetyWarning", "VRTracker")
