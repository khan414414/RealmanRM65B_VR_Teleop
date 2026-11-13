"""Offline RM65-6F-V kinematics and position-only, roll-decoupled IK.

The joint-frame convention follows the original RM65-B URDF, with the terminal
link length replaced by RealMan's RM65-6F-V MDH value, d6 = 184 mm:
https://develop.realman-robotics.com/robot4th/robotParameter/RM65OntologyParameters/
The sensor/vision end structure is included in d6.  An active tool offset is
relative to that flange and must not include the same structure a second time.
Joints 1..5 solve only the TCP position task.  Joint 6 is held at a separately
provided wrist-roll target and is never used by the position pseudoinverse.

This split is intentional, but it is not a full pose IK: joints 1..5 are free
to change TCP orientation while reaching XYZ.  Also, an off-axis active tool
offset makes J6 affect TCP position; in that case J1..5 compensate for the
chosen J6 angle rather than treating its motion as position-neutral.

The companion SolidWorks CSV describes joint 6 with the opposite axis sign,
while the URDF and RealMan API2 forward kinematics agree on +Z.  This module
follows the URDF/API2 convention.  The controller-to-J6 ``roll_sign`` remains a
separate input-device calibration and must be checked at low speed.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence

import numpy as np


# Official RM65-6F-V d6; the original RM65-B model used 0.144 m.
RM65_D6_M = 0.184


RM65_JOINT_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-3.1060, 3.1060),
    (-2.2689, 2.2689),
    (-2.3560, 2.3560),
    (-3.1060, 3.1060),
    (-2.2340, 2.2340),
    (-6.2800, 6.2800),
)

# A bent, non-singular posture that can be supplied explicitly as a secondary
# objective.  It is not imposed automatically because collision-free posture
# depends on the physical cell and active tool.
RM65_NOMINAL_POSTURE_RAD: tuple[float, ...] = tuple(
    math.radians(value) for value in (0.0, -75.0, 60.0, 0.0, 60.0, 0.0)
)

_JOINT_ORIGIN_XYZ = np.asarray(
    (
        (0.0, 0.0, 0.2405),
        (0.0, 0.0, 0.0),
        (0.256, 0.0, 0.0),
        (0.0, -0.210, 0.0),
        (0.0, 0.0, 0.0),
        # In these URDF joint frames, MDH d6 is along joint 5's local -Y.
        (0.0, -RM65_D6_M, 0.0),
    ),
    dtype=np.float64,
)
_JOINT_ORIGIN_RPY = np.asarray(
    (
        (0.0, 0.0, 0.0),
        (math.pi / 2.0, -math.pi / 2.0, 0.0),
        (0.0, 0.0, math.pi / 2.0),
        (math.pi / 2.0, 0.0, 0.0),
        (-math.pi / 2.0, 0.0, 0.0),
        (math.pi / 2.0, 0.0, 0.0),
    ),
    dtype=np.float64,
)
_JOINT_AXES = np.tile(np.asarray((0.0, 0.0, 1.0)), (6, 1))


def _vector(values: Sequence[float], length: int, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},)")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result.copy()


def _joint_limits(
    value: Optional[Sequence[Sequence[float]]],
) -> np.ndarray:
    if value is None:
        result = np.asarray(RM65_JOINT_LIMITS_RAD, dtype=np.float64)
    else:
        result = np.asarray(value, dtype=np.float64)
    if result.shape != (6, 2) or not np.all(np.isfinite(result)):
        raise ValueError("joint_limits_rad must be a finite 6x2 array")
    if np.any(result[:, 0] >= result[:, 1]):
        raise ValueError("each joint limit minimum must be below its maximum")
    return result.copy()


def _transform(value: Optional[Sequence[Sequence[float]]]) -> np.ndarray:
    if value is None:
        return np.eye(4, dtype=np.float64)
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (4, 4) or not np.all(np.isfinite(result)):
        raise ValueError("tcp_transform must be a finite 4x4 matrix")
    if not np.allclose(result[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
        raise ValueError("tcp_transform must be homogeneous")
    rotation = result[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7):
        raise ValueError("tcp_transform rotation must be orthonormal")
    if np.linalg.det(rotation) < 0.0:
        raise ValueError("tcp_transform rotation must be right-handed")
    return result.copy()


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    """Return the URDF fixed-axis roll-pitch-yaw rotation matrix."""

    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    ry = np.asarray(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    rz = np.asarray(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    return rz @ ry @ rx


def transform_from_xyz_euler(pose: Sequence[float]) -> np.ndarray:
    """Build a homogeneous transform from ``[x, y, z, rx, ry, rz]``.

    RealMan tool-frame offsets use metres and fixed-axis XYZ Euler angles, the
    same convention used by URDF ``rpy`` origins.
    """

    value = _vector(pose, 6, "pose")
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = _rpy_matrix(value[3:])
    result[:3, 3] = value[:3]
    return result


def _axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.asarray(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))
    identity = np.eye(3)
    return identity + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (
        skew @ skew
    )


def _joint_frames(
    joints_rad: Sequence[float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    joints = _vector(joints_rad, 6, "joints_rad")
    transform = np.eye(4, dtype=np.float64)
    origins = np.empty((6, 3), dtype=np.float64)
    axes = np.empty((6, 3), dtype=np.float64)

    for index in range(6):
        origin = np.eye(4, dtype=np.float64)
        origin[:3, :3] = _rpy_matrix(_JOINT_ORIGIN_RPY[index])
        origin[:3, 3] = _JOINT_ORIGIN_XYZ[index]
        transform = transform @ origin
        origins[index] = transform[:3, 3]
        axes[index] = transform[:3, :3] @ _JOINT_AXES[index]

        rotation = np.eye(4, dtype=np.float64)
        rotation[:3, :3] = _axis_angle_matrix(
            _JOINT_AXES[index], joints[index]
        )
        transform = transform @ rotation

    return transform, origins, axes


def forward_kinematics(
    joints_rad: Sequence[float],
    tcp_transform: Optional[Sequence[Sequence[float]]] = None,
) -> np.ndarray:
    """Return the base-to-TCP homogeneous transform for six joint angles."""

    link_6, _, _ = _joint_frames(joints_rad)
    return link_6 @ _transform(tcp_transform)


def geometric_jacobian(
    joints_rad: Sequence[float],
    tcp_transform: Optional[Sequence[Sequence[float]]] = None,
) -> np.ndarray:
    """Return the 6x6 base-frame geometric Jacobian ``[linear; angular]``."""

    link_6, origins, axes = _joint_frames(joints_rad)
    tcp_position = (link_6 @ _transform(tcp_transform))[:3, 3]
    jacobian = np.empty((6, 6), dtype=np.float64)
    for index in range(6):
        jacobian[:3, index] = np.cross(
            axes[index], tcp_position - origins[index]
        )
        jacobian[3:, index] = axes[index]
    return jacobian


def position_jacobian(
    joints_rad: Sequence[float],
    tcp_transform: Optional[Sequence[Sequence[float]]] = None,
) -> np.ndarray:
    """Return the 3x6 TCP position Jacobian in the base frame."""

    return geometric_jacobian(joints_rad, tcp_transform)[:3]


def unwrap_angle_near(angle_rad: float, reference_rad: float) -> float:
    """Return the 2*pi equivalent of ``angle_rad`` nearest ``reference_rad``."""

    angle = float(angle_rad)
    reference = float(reference_rad)
    if not math.isfinite(angle) or not math.isfinite(reference):
        raise ValueError("angles must be finite")
    return angle + 2.0 * math.pi * round((reference - angle) / (2.0 * math.pi))


def _normalize_quaternion_xyzw(
    quaternion: Sequence[float], name: str = "quaternion"
) -> np.ndarray:
    result = _vector(quaternion, 4, name)
    norm = float(np.linalg.norm(result))
    if norm < 1.0e-9:
        raise ValueError(f"{name} has near-zero norm")
    return result / norm


def _quaternion_multiply_xyzw(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.asarray(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ),
        dtype=np.float64,
    )


def _wrap_to_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class ControllerTwistError(ValueError):
    """The controller quaternion cannot produce a safe continuous twist."""


class ContinuousControllerTwist:
    """Extract and unwrap local swing-twist rotation from controller poses.

    The Unity sender changes both handedness and local basis before publishing
    the quaternion.  Its Aim ``+Z`` axis is represented as local ``+X`` in the
    Python-side quaternion, hence the default axis below.  ``update`` returns an
    unwrapped angle relative to the quaternion supplied to ``reset``.  A
    configured ``max_step_rad`` slews large input jumps over repeated updates;
    it does not discard the latest orientation.
    """

    def __init__(
        self,
        axis: Sequence[float] = (1.0, 0.0, 0.0),
        *,
        max_step_rad: Optional[float] = math.radians(45.0),
    ) -> None:
        axis_vector = _vector(axis, 3, "axis")
        norm = float(np.linalg.norm(axis_vector))
        if norm < 1.0e-9:
            raise ValueError("axis has near-zero norm")
        self.axis = axis_vector / norm
        if max_step_rad is not None:
            maximum = float(max_step_rad)
            if not math.isfinite(maximum) or maximum <= 0.0:
                raise ValueError("max_step_rad must be positive")
            self.max_step_rad: Optional[float] = maximum
        else:
            self.max_step_rad = None
        self._anchor: Optional[np.ndarray] = None
        self._previous_quaternion: Optional[np.ndarray] = None
        self._previous_wrapped = 0.0
        self._continuous = 0.0

    @property
    def angle_rad(self) -> float:
        return self._continuous

    def reset(self, anchor_quaternion_xyzw: Sequence[float]) -> None:
        anchor = _normalize_quaternion_xyzw(
            anchor_quaternion_xyzw, "anchor_quaternion_xyzw"
        )
        self._anchor = anchor
        self._previous_quaternion = anchor.copy()
        self._previous_wrapped = 0.0
        self._continuous = 0.0

    def update(self, quaternion_xyzw: Sequence[float]) -> float:
        if self._anchor is None or self._previous_quaternion is None:
            raise RuntimeError("reset must be called before update")

        current = _normalize_quaternion_xyzw(quaternion_xyzw, "quaternion_xyzw")
        # q and -q are identical rotations.  Keep the sampled representation
        # continuous before forming the anchor-local relative quaternion.
        if float(np.dot(current, self._previous_quaternion)) < 0.0:
            current = -current

        anchor_inverse = self._anchor.copy()
        anchor_inverse[:3] *= -1.0
        relative = _quaternion_multiply_xyzw(anchor_inverse, current)
        relative = _normalize_quaternion_xyzw(relative, "relative_quaternion")

        projected = float(np.dot(relative[:3], self.axis))
        twist_norm = math.hypot(relative[3], projected)
        if twist_norm < 1.0e-7:
            raise ControllerTwistError(
                "controller twist is undefined near a 180-degree orthogonal swing"
            )

        wrapped = _wrap_to_pi(2.0 * math.atan2(projected, relative[3]))
        delta = _wrap_to_pi(wrapped - self._previous_wrapped)
        if self.max_step_rad is not None and abs(delta) > self.max_step_rad:
            # Keep the latest orientation as a pending virtual target.  By
            # advancing _previous_wrapped only by the capped amount, repeated
            # processing of the same latest quaternion consumes the remainder
            # over subsequent control ticks instead of rejecting it forever.
            delta = math.copysign(self.max_step_rad, delta)

        self._continuous += delta
        self._previous_wrapped = _wrap_to_pi(self._previous_wrapped + delta)
        self._previous_quaternion = current
        return self._continuous


def nearest_equivalent_angle_within_limits(
    angle_rad: float,
    reference_rad: float,
    limits_rad: tuple[float, float] = RM65_JOINT_LIMITS_RAD[5],
) -> float:
    """Choose the in-limit 2*pi equivalent closest to the previous command."""

    angle = float(angle_rad)
    reference = float(reference_rad)
    lower, upper = (float(value) for value in limits_rad)
    if not all(math.isfinite(value) for value in (angle, reference, lower, upper)):
        raise ValueError("angles and limits must be finite")
    if lower >= upper:
        raise ValueError("limits_rad must be increasing")

    turns_low = math.ceil((lower - angle) / (2.0 * math.pi))
    turns_high = math.floor((upper - angle) / (2.0 * math.pi))
    if turns_low <= turns_high:
        nearest_turn = round((reference - angle) / (2.0 * math.pi))
        nearest_turn = min(max(nearest_turn, turns_low), turns_high)
        return angle + 2.0 * math.pi * nearest_turn

    # The RM65 J6 interval is fractionally narrower than two complete turns at
    # each side.  A requested equivalence can therefore fall in a tiny gap.
    return min(max(unwrap_angle_near(angle, reference), lower), upper)


def continuous_wrist_roll_target(
    anchor_joint_6_rad: float,
    controller_roll_delta_rad: float,
    previous_joint_6_rad: float,
    *,
    roll_sign: float = 1.0,
    max_step_rad: Optional[float] = None,
    limits_rad: tuple[float, float] = RM65_JOINT_LIMITS_RAD[5],
) -> float:
    """Map an unwrapped controller-roll delta to a continuous J6 command."""

    if roll_sign not in (-1.0, 1.0):
        raise ValueError("roll_sign must be +1 or -1")
    desired = float(anchor_joint_6_rad) + roll_sign * float(
        controller_roll_delta_rad
    )
    if not math.isfinite(desired):
        raise ValueError("controller_roll_delta_rad must be finite")
    command = desired
    if max_step_rad is not None:
        maximum = float(max_step_rad)
        if not math.isfinite(maximum) or maximum <= 0.0:
            raise ValueError("max_step_rad must be positive")
        delta = min(
            max(command - previous_joint_6_rad, -maximum), maximum
        )
        command = float(previous_joint_6_rad) + delta
    lower, upper = limits_rad
    return min(max(command, lower), upper)


@dataclass(frozen=True)
class IKOptions:
    """Numerical and continuity limits for position-only IK."""

    max_iterations: int = 80
    position_tolerance_m: float = 2.0e-5
    damping_min: float = 1.0e-4
    damping_max: float = 5.0e-2
    singular_value_threshold: float = 4.0e-2
    max_iteration_step_rad: float = math.radians(4.0)
    max_solution_delta_rad: float = math.radians(20.0)
    max_wrist_step_rad: float = math.radians(5.0)
    posture_gain_rad: float = math.radians(0.20)
    limit_avoidance_gain_rad: float = math.radians(1.0)
    limit_activation_fraction: float = 0.12
    joint_limit_margin_rad: float = math.radians(0.25)
    minimum_line_search_scale: float = 1.0 / 128.0

    def __post_init__(self) -> None:
        positive = (
            self.position_tolerance_m,
            self.damping_min,
            self.damping_max,
            self.singular_value_threshold,
            self.max_iteration_step_rad,
            self.max_solution_delta_rad,
            self.max_wrist_step_rad,
            self.minimum_line_search_scale,
        )
        if self.max_iterations <= 0 or not all(
            math.isfinite(value) and value > 0.0 for value in positive
        ):
            raise ValueError(
                "IK iteration, tolerance, damping and step limits must be positive"
            )
        if self.damping_min > self.damping_max:
            raise ValueError("damping_min cannot exceed damping_max")
        if not 0.0 < self.limit_activation_fraction < 0.5:
            raise ValueError("limit_activation_fraction must be in (0, 0.5)")
        if self.posture_gain_rad < 0.0 or self.limit_avoidance_gain_rad < 0.0:
            raise ValueError("secondary-objective gains cannot be negative")
        if self.joint_limit_margin_rad < 0.0:
            raise ValueError("joint_limit_margin_rad cannot be negative")
        if self.minimum_line_search_scale > 1.0:
            raise ValueError("minimum_line_search_scale cannot exceed one")


@dataclass(frozen=True)
class IKResult:
    """An IK candidate that may be a useful bounded partial result.

    A false ``converged`` value means the position tolerance was not reached;
    callers may use the candidate only after independently proving monotonic
    progress, joint/Cartesian step limits, joint margins, singularity safety,
    and collision safety.
    """

    joints_rad: np.ndarray
    converged: bool
    iterations: int
    position_error_m: float
    minimum_singular_value: float
    damping: float
    status: str
    wrist_target_reached: bool


class RM65DecoupledIK:
    """Local, continuity-preserving RM65-6F-V position IK with independent J6."""

    def __init__(
        self,
        *,
        tcp_transform: Optional[Sequence[Sequence[float]]] = None,
        joint_limits_rad: Optional[Sequence[Sequence[float]]] = None,
        options: Optional[IKOptions] = None,
    ) -> None:
        self.tcp_transform = _transform(tcp_transform)
        self._limits = _joint_limits(joint_limits_rad)
        self.options = options or IKOptions()
        spans = self._limits[:5, 1] - self._limits[:5, 0]
        if np.any(2.0 * self.options.joint_limit_margin_rad >= spans):
            raise ValueError("joint_limit_margin_rad leaves no J1..J5 workspace")

    @property
    def joint_limits_rad(self) -> np.ndarray:
        """Return the configured controller/model limits as a defensive copy."""

        return self._limits.copy()

    def _adaptive_damping(self, minimum_singular_value: float) -> float:
        options = self.options
        ratio = max(
            0.0,
            1.0 - minimum_singular_value / options.singular_value_threshold,
        )
        return options.damping_min + (
            options.damping_max - options.damping_min
        ) * ratio * ratio

    def _limit_direction(self, joints_1_to_5: np.ndarray) -> np.ndarray:
        lower = self._limits[:5, 0]
        upper = self._limits[:5, 1]
        span = upper - lower
        lower_fraction = (joints_1_to_5 - lower) / span
        upper_fraction = (upper - joints_1_to_5) / span
        activation = self.options.limit_activation_fraction
        direction = np.zeros(5, dtype=np.float64)

        lower_active = lower_fraction < activation
        upper_active = upper_fraction < activation
        direction[lower_active] += (
            (activation - lower_fraction[lower_active]) / activation
        ) ** 2
        direction[upper_active] -= (
            (activation - upper_fraction[upper_active]) / activation
        ) ** 2
        return np.clip(direction, -1.0, 1.0)

    def _bounds_near_seed(
        self, seed_1_to_5: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        margin = self.options.joint_limit_margin_rad
        soft_lower = self._limits[:5, 0] + margin
        soft_upper = self._limits[:5, 1] - margin

        # A seed already inside the reserved margin may stay where it is or
        # move inward, but the solver may not move it farther toward the limit.
        lower = np.where(seed_1_to_5 < soft_lower, seed_1_to_5, soft_lower)
        upper = np.where(seed_1_to_5 > soft_upper, seed_1_to_5, soft_upper)
        trust = self.options.max_solution_delta_rad
        lower = np.maximum(lower, seed_1_to_5 - trust)
        upper = np.minimum(upper, seed_1_to_5 + trust)
        return lower, upper

    def solve(
        self,
        target_position_m: Sequence[float],
        seed_joints_rad: Sequence[float],
        *,
        wrist_joint_6_target_rad: Optional[float] = None,
        posture_reference_rad: Optional[Sequence[float]] = None,
    ) -> IKResult:
        """Solve XYZ with J1..J5 while rate-limiting an independent J6 target.

        ``seed_joints_rad`` should be the previous accepted command, not a fixed
        home pose.  That local seed, the per-iteration cap and the per-solve
        trust region prevent IK branch jumps.  The optional posture reference
        may contain five or six joints; only its first five values are used in
        the exact position-task null space.
        """

        target = _vector(target_position_m, 3, "target_position_m")
        seed = _vector(seed_joints_rad, 6, "seed_joints_rad")
        if np.any(seed < self._limits[:, 0]) or np.any(
            seed > self._limits[:, 1]
        ):
            raise ValueError("seed_joints_rad violates the configured joint limits")

        if posture_reference_rad is None:
            reference = seed[:5].copy()
        else:
            supplied = np.asarray(posture_reference_rad, dtype=np.float64)
            if supplied.shape not in ((5,), (6,)) or not np.all(np.isfinite(supplied)):
                raise ValueError("posture_reference_rad must contain five or six finite values")
            reference = supplied[:5].copy()
            reference = np.clip(
                reference, self._limits[:5, 0], self._limits[:5, 1]
            )

        requested_wrist = seed[5]
        if wrist_joint_6_target_rad is not None:
            requested_wrist = float(wrist_joint_6_target_rad)
            if not math.isfinite(requested_wrist):
                raise ValueError("wrist_joint_6_target_rad must be finite")
            if not self._limits[5, 0] <= requested_wrist <= self._limits[5, 1]:
                raise ValueError("wrist_joint_6_target_rad violates the J6 limits")
        wrist_delta = requested_wrist - seed[5]
        wrist_step = self.options.max_wrist_step_rad
        commanded_wrist = seed[5] + min(max(wrist_delta, -wrist_step), wrist_step)
        commanded_wrist = float(
            np.clip(commanded_wrist, self._limits[5, 0], self._limits[5, 1])
        )
        wrist_reached = math.isclose(
            commanded_wrist, requested_wrist, abs_tol=1e-12
        )

        joints = seed.copy()
        joints[5] = commanded_wrist
        lower, upper = self._bounds_near_seed(seed[:5])
        status = "max_iterations"
        iterations = 0

        for iterations in range(1, self.options.max_iterations + 1):
            position = forward_kinematics(joints, self.tcp_transform)[:3, 3]
            error = target - position
            error_norm = float(np.linalg.norm(error))
            if error_norm <= self.options.position_tolerance_m:
                status = "converged"
                break

            jacobian = position_jacobian(joints, self.tcp_transform)[:, :5]
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            minimum_singular = float(singular_values[-1])
            damping = self._adaptive_damping(minimum_singular)
            gram = jacobian @ jacobian.T + damping * damping * np.eye(3)
            damped_inverse = jacobian.T @ np.linalg.solve(gram, np.eye(3))
            task_step = damped_inverse @ error

            # Use the undamped Moore-Penrose projector for an actual first-order
            # position null space; damping is reserved for the primary task.
            projector = np.eye(5) - np.linalg.pinv(jacobian, rcond=1e-8) @ jacobian
            half_range = (
                self._limits[:5, 1] - self._limits[:5, 0]
            ) / 2.0
            posture_direction = (reference - joints[:5]) / half_range
            secondary = (
                self.options.posture_gain_rad * posture_direction
                + self.options.limit_avoidance_gain_rad
                * self._limit_direction(joints[:5])
            )
            combined_step = task_step + projector @ secondary

            def cap_iteration_step(step: np.ndarray) -> np.ndarray:
                result = step.copy()
                largest = float(np.max(np.abs(result)))
                if largest > self.options.max_iteration_step_rad:
                    result *= self.options.max_iteration_step_rad / largest
                return result

            combined_step = cap_iteration_step(combined_step)
            task_step = cap_iteration_step(task_step)

            accepted = False
            for direction in (combined_step, task_step):
                scale = 1.0
                while scale >= self.options.minimum_line_search_scale:
                    trial = joints.copy()
                    trial[:5] = np.clip(
                        joints[:5] + scale * direction, lower, upper
                    )
                    trial_error = target - forward_kinematics(
                        trial, self.tcp_transform
                    )[:3, 3]
                    if float(np.linalg.norm(trial_error)) < error_norm - 1e-12:
                        joints = trial
                        accepted = True
                        break
                    scale *= 0.5
                if accepted:
                    break

            if not accepted:
                status = "stagnated"
                break

        final_position = forward_kinematics(joints, self.tcp_transform)[:3, 3]
        final_error = float(np.linalg.norm(target - final_position))
        final_jacobian = position_jacobian(joints, self.tcp_transform)[:, :5]
        final_singular = float(np.linalg.svd(final_jacobian, compute_uv=False)[-1])
        final_damping = self._adaptive_damping(final_singular)
        converged = final_error <= self.options.position_tolerance_m
        if converged:
            status = "converged"
        elif np.any(np.isclose(joints[:5], lower, atol=1e-10)) or np.any(
            np.isclose(joints[:5], upper, atol=1e-10)
        ):
            status = "trust_region_or_limit_reached"

        return IKResult(
            joints_rad=joints.copy(),
            converged=converged,
            iterations=iterations,
            position_error_m=final_error,
            minimum_singular_value=final_singular,
            damping=final_damping,
            status=status,
            wrist_target_reached=wrist_reached,
        )


__all__ = (
    "ContinuousControllerTwist",
    "ControllerTwistError",
    "IKOptions",
    "IKResult",
    "RM65DecoupledIK",
    "RM65_D6_M",
    "RM65_JOINT_LIMITS_RAD",
    "RM65_NOMINAL_POSTURE_RAD",
    "continuous_wrist_roll_target",
    "forward_kinematics",
    "geometric_jacobian",
    "nearest_equivalent_angle_within_limits",
    "position_jacobian",
    "transform_from_xyz_euler",
    "unwrap_angle_near",
)
