"""XYZ-only task with independent J6, using official RealMan IK and FK.

The SDK solves full poses. This adapter searches nearby orientations when the
current orientation prevents XYZ progress. Orientation is a soft continuity
preference, never an equality constraint. All joint candidates come from the
official IK, and restoring J6 is followed by official FK and position checks.
"""

from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

from teleop.rm65_decoupled_ik import (
    IKOptions, IKResult, _joint_limits, _transform, _vector,
    transform_from_xyz_euler, position_jacobian,
)


@dataclass
class _Candidate:
    joints: np.ndarray
    error: float
    safe: bool
    score: float
    orientation: np.ndarray


class _SearchBudgetExhausted(Exception):
    pass


class OfficialPositionIK:
    MAX_SDK_CALLS = 16

    def __init__(self, backend, *, tcp_transform=None, joint_limits_rad=None, options=None):
        for name in ("official_forward_kinematics", "official_inverse_kinematics"):
            if not callable(getattr(backend, name, None)):
                raise ValueError(f"official IK backend requires {name}")
        self.backend = backend
        self.tcp_transform = _transform(tcp_transform if tcp_transform is not None else np.eye(4))
        self._limits = _joint_limits(joint_limits_rad)
        self.options = options or IKOptions()
        self.candidate_validator = None
        self.max_solve_time_s = None
        self._last_seed = None
        self._orientation_released = False

    @property
    def joint_limits_rad(self):
        return self._limits.copy()

    def reset_continuity(self):
        self._last_seed = None
        self._orientation_released = False

    def _pose(self, joints):
        return _vector(self.backend.official_forward_kinematics(joints), 6, "official FK pose")

    def forward_kinematics(self, joints):
        return transform_from_xyz_euler(self._pose(joints))

    def solve(self, target_position_m, seed_joints_rad, *,
              wrist_joint_6_target_rad=None, posture_reference_rad=None):
        target = _vector(target_position_m, 3, "target_position_m")
        seed = _vector(seed_joints_rad, 6, "seed_joints_rad")
        if np.any(seed < self._limits[:, 0]) or np.any(seed > self._limits[:, 1]):
            raise ValueError("seed_joints_rad violates the configured joint limits")
        wrist = seed[5] if wrist_joint_6_target_rad is None else float(wrist_joint_6_target_rad)
        if not math.isfinite(wrist) or not self._limits[5, 0] <= wrist <= self._limits[5, 1]:
            raise ValueError("invalid independent J6 target")
        commanded_wrist = seed[5] + np.clip(wrist - seed[5],
                                          -self.options.max_wrist_step_rad,
                                          self.options.max_wrist_step_rad)
        step = self.options.max_solution_delta_rad
        lower = np.maximum(self._limits[:5, 0] + self.options.joint_limit_margin_rad, seed[:5] - step)
        upper = np.minimum(self._limits[:5, 1] - self.options.joint_limit_margin_rad, seed[:5] + step)
        if np.any(lower > upper):
            raise ValueError("seed outside the usable joint margin")
        previous_delta = np.zeros(5)
        if self._last_seed is not None:
            previous_delta = seed[:5] - self._last_seed[:5]
            if np.max(np.abs(previous_delta)) > 1.5 * step:
                previous_delta[:] = 0
        self._last_seed = seed.copy()
        reference = seed[:5] if posture_reference_rad is None else np.asarray(posture_reference_rad)[:5]
        if reference.shape != (5,) or not np.all(np.isfinite(reference)):
            raise ValueError("posture_reference_rad must contain finite joint values")

        working = seed.copy()
        working[5] = commanded_wrist
        initial_pose = self._pose(working)
        initial_error = float(np.linalg.norm(target - initial_pose[:3]))
        tolerance = self.options.position_tolerance_m
        budget = min(self.options.max_iterations, self.MAX_SDK_CALLS)
        attempts = 0
        status = "official_ik_position_not_reached"
        deadline = (None if self.max_solve_time_s is None else
                    time.perf_counter() + self.max_solve_time_s)

        def has_budget():
            return attempts < budget and (deadline is None or time.perf_counter() < deadline)

        def validate_candidate(q):
            if self.candidate_validator is not None:
                try:
                    self.candidate_validator(q)
                except ValueError:
                    return False
            return True

        def trial(current, orientation, sdk_seed=None, capture=None, validate=True):
            nonlocal attempts, status
            if not has_budget():
                return None
            code, output = self.backend.official_inverse_kinematics(
                current if sdk_seed is None else sdk_seed, np.r_[target, orientation])
            attempts += 1
            if code != 0:
                status = f"official_ik_code_{code}"
                return None
            candidate = _vector(output, 6, "official IK joints")
            candidate[:5] += 2 * math.pi * np.round((seed[:5] - candidate[:5]) / (2 * math.pi))
            if capture is not None:
                capture["raw"] = candidate.copy()
            delta = candidate[:5] - seed[:5]
            largest = float(np.max(np.abs(delta)))
            if largest > step:
                delta *= step / largest
            candidate[:5] = np.clip(seed[:5] + delta, lower, upper)
            candidate[5] = commanded_wrist
            current_error = float(np.linalg.norm(target - self._pose(current)[:3]))
            for scale in (1.0, 0.5, 0.25, 0.125):
                q = current + scale * (candidate - current)
                q[5] = commanded_wrist
                error = float(np.linalg.norm(target - self._pose(q)[:3]))
                if error >= current_error - 1e-9:
                    continue
                safe = validate and validate_candidate(q)
                dq = (q[:5] - seed[:5]) / step
                dv = dq - previous_delta / step
                # Position dominates; the remaining terms are soft preferences.
                score = ((error / max(initial_error, tolerance)) ** 2
                         + 0.02 * float(dq @ dq) + 1.0 * float(dv @ dv)
                         + 0.0001 * float(np.sum((q[:5] - reference) ** 2))
                         + 0.01 * (0.15 / max(abs(math.sin(q[2])), 0.01)) ** 2)
                return _Candidate(q, error, safe, score, np.asarray(orientation).copy())
            return None

        # A failed full-pose query is not proof that XYZ is unreachable.
        error = initial_error
        unsafe = None
        primary = None
        for _ in range(min(8, budget)):
            if error <= tolerance:
                break
            candidate = trial(working, self._pose(working)[3:])
            if candidate is None:
                break
            if not candidate.safe:
                unsafe = candidate
                break
            working, error = candidate.joints, candidate.error
            primary = candidate
            if np.max(np.abs(working[:5] - seed[:5])) >= step * (1 - 1e-6):
                break

        released = False
        velocity_change = np.max(np.abs(working[:5] - seed[:5] - previous_delta)) / step
        needs_release = error >= initial_error - 1e-9
        needs_continuity = self._orientation_released and velocity_change > 0.2
        if initial_error > tolerance and (needs_release or needs_continuity):
            rotation = Rotation.from_euler("xyz", initial_pose[3:])
            # Orientation freedom must shrink with the requested XYZ step.
            # A fixed-degree search near the end of a move can redistribute
            # several degrees of elbow angle for only micrometres of motion.
            radius = min(step, math.radians(6), max(2e-4, initial_error / 0.15))
            selected = primary
            search_seed = seed.copy()
            search_seed[5] = commanded_wrist
            predicted = search_seed.copy()
            predicted[:5] = np.clip(seed[:5] + 0.7 * previous_delta, lower, upper)
            if np.max(np.abs(previous_delta)) > 1e-8:
                candidate = trial(search_seed, self._pose(predicted)[3:], predicted)
                if candidate is not None and candidate.safe:
                    if selected is None or candidate.score < selected.score:
                        selected = candidate
            # Search small neighborhoods first and compare joint continuity,
            # rather than accepting the first +/-wrist posture that works.
            for factor in (() if selected is not None else (0.5, 1.0, 2.0)):
                for axis in range(3):
                    for sign in (-1, 1):
                        offset = np.zeros(3)
                        offset[axis] = sign * factor * radius
                        orientation = (Rotation.from_rotvec(offset) * rotation).as_euler("xyz")
                        candidate = trial(search_seed, orientation, predicted)
                        if candidate is None:
                            continue
                        if not candidate.safe:
                            if unsafe is None or candidate.score < unsafe.score:
                                unsafe = candidate
                            continue
                        if selected is None or candidate.score < selected.score:
                            selected = candidate
                if selected is not None:
                    break
            if selected is not None and has_budget():
                # Optimize only the three free orientation parameters. Each
                # residual evaluation obtains joints from official IK; this is
                # not a local joint-space inverse solver or an IK fallback.
                angle_bound = min(4 * radius, math.radians(20))
                start_offset = (Rotation.from_euler("xyz", selected.orientation)
                                * rotation.inv()).as_rotvec()
                # Center variables at 1 so relative finite-difference steps
                # remain resolvable by the SDK's float32 joint representation.
                x0 = 1 + np.clip(start_offset, -angle_bound, angle_bound)

                def orientation_residual(x):
                    nonlocal selected
                    if not has_budget():
                        raise _SearchBudgetExhausted
                    offset = x - 1
                    orientation = (Rotation.from_rotvec(offset) * rotation).as_euler("xyz")
                    capture = {}
                    candidate = trial(search_seed, orientation, predicted, capture, validate=False)
                    if candidate is not None and candidate.score < selected.score:
                        if validate_candidate(candidate.joints):
                            candidate.safe = True
                            selected = candidate
                    if "raw" not in capture:
                        return np.full(17, 100.0)
                    raw = capture["raw"].copy()
                    raw[5] = commanded_wrist
                    try:
                        position_error = (self._pose(raw)[:3] - target) / tolerance
                    except ValueError:
                        return np.full(17, 100.0)
                    return np.r_[
                        (raw[:5] - predicted[:5]) / step,
                        position_error,
                        0.001 * offset / step,
                        0.001 * (raw[:5] - reference),
                        0.1 * 0.15 / max(abs(math.sin(raw[2])), 0.01),
                    ]

                try:
                    least_squares(orientation_residual, x0,
                                  bounds=(np.full(3, 1 - angle_bound),
                                          np.full(3, 1 + angle_bound)),
                                  diff_step=1e-4, max_nfev=6,
                                  ftol=1e-4, xtol=1e-4, gtol=1e-4)
                except _SearchBudgetExhausted:
                    pass
            if selected is not None:
                working, error = selected.joints, selected.error
                released = True
                self._orientation_released = True
                while error > tolerance and has_budget():
                    candidate = trial(working, self._pose(working)[3:])
                    if (candidate is None or not candidate.safe
                            or candidate.score > selected.score + 1e-12):
                        break
                    working, error = candidate.joints, candidate.error
                    selected = candidate
            elif unsafe is not None:
                # Preserve VRTracker's existing 2.5% fallback / safety hold.
                working, error = unsafe.joints, unsafe.error

        converged = error <= tolerance
        minimum = float(np.linalg.svd(position_jacobian(working, self.tcp_transform)[:, :5],
                                      compute_uv=False)[-1])
        if converged:
            status = "official_ik_orientation_free" if released else "official_ik_converged"
        return IKResult(
            joints_rad=working.copy(), converged=converged, iterations=attempts,
            position_error_m=error, minimum_singular_value=minimum, damping=0.0,
            status=status, wrist_target_reached=math.isclose(working[5], wrist, abs_tol=1e-12))
