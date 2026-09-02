"""Conservative capsule collision model for the AUBO i7_30.

The model follows the same joint origins and simplified visual radii used by
the Unity digital twin.  It checks non-neighbouring robot bodies against each
other and checks every moving body against a horizontal base-frame plane.
It is deliberately a safety pre-filter; controller collision protection and a
site-specific cell model remain authoritative.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence

import numpy as np

from teleop.aubo_i7_decoupled_ik import collision_key_points


JOINT_RADII_M = (0.076, 0.074, 0.068, 0.058, 0.052, 0.047)
LINK_RADII_M = (0.057, 0.052, 0.047, 0.039, 0.034)
BASE_RADIUS_M = 0.099
BASE_HEIGHT_M = 0.122
FLANGE_RADIUS_M = 0.048
DEFAULT_TOOL_RADIUS_M = 0.030


class AuboCollisionError(ValueError):
    """A candidate intersects the robot itself or the configured plane."""


@dataclass(frozen=True)
class Capsule:
    name: str
    start: np.ndarray
    end: np.ndarray
    radius_m: float
    first_node: int
    last_node: int
    kind: str


def _point(values: Sequence[float], name: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite coordinates")
    return result.copy()


def segment_distance_m(
    start_a: Sequence[float],
    end_a: Sequence[float],
    start_b: Sequence[float],
    end_b: Sequence[float],
) -> float:
    """Return the shortest distance between two closed 3-D segments."""

    p1 = _point(start_a, "start_a")
    q1 = _point(end_a, "end_a")
    p2 = _point(start_b, "start_b")
    q2 = _point(end_b, "end_b")
    d1 = q1 - p1
    d2 = q2 - p2
    offset = p1 - p2
    a = float(d1 @ d1)
    e = float(d2 @ d2)
    f = float(d2 @ offset)
    epsilon = 1.0e-15

    if a <= epsilon and e <= epsilon:
        return float(np.linalg.norm(p1 - p2))
    if a <= epsilon:
        s = 0.0
        t = float(np.clip(f / e, 0.0, 1.0))
    else:
        c = float(d1 @ offset)
        if e <= epsilon:
            t = 0.0
            s = float(np.clip(-c / a, 0.0, 1.0))
        else:
            b = float(d1 @ d2)
            denominator = a * e - b * b
            if denominator > epsilon:
                s = float(np.clip((b * f - c * e) / denominator, 0.0, 1.0))
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = float(np.clip(-c / a, 0.0, 1.0))
            elif t > 1.0:
                t = 1.0
                s = float(np.clip((b - c) / a, 0.0, 1.0))

    closest_a = p1 + s * d1
    closest_b = p2 + t * d2
    return float(np.linalg.norm(closest_a - closest_b))


class AuboI7CollisionChecker:
    """Validate i7 commands against self-collision and a horizontal plane."""

    def __init__(
        self,
        *,
        plane_z_m: float = 0.0,
        safety_margin_m: float = 0.005,
        tool_radius_m: float = DEFAULT_TOOL_RADIUS_M,
    ) -> None:
        self.plane_z_m = float(plane_z_m)
        self.safety_margin_m = float(safety_margin_m)
        self.tool_radius_m = float(tool_radius_m)
        if not math.isfinite(self.plane_z_m):
            raise ValueError("plane_z_m must be finite")
        if not math.isfinite(self.safety_margin_m) or self.safety_margin_m < 0.0:
            raise ValueError("safety_margin_m must be finite and non-negative")
        if not math.isfinite(self.tool_radius_m) or self.tool_radius_m <= 0.0:
            raise ValueError("tool_radius_m must be finite and positive")

    def capsules(
        self,
        joints_rad: Sequence[float],
        tcp_transform: Optional[Sequence[Sequence[float]]] = None,
    ) -> tuple[Capsule, ...]:
        origins, flange_face, tcp_position = collision_key_points(
            joints_rad, tcp_transform
        )
        bodies: list[Capsule] = [
            Capsule(
                "base",
                np.asarray((0.0, 0.0, 0.0)),
                np.asarray((0.0, 0.0, BASE_HEIGHT_M)),
                BASE_RADIUS_M,
                -1,
                0,
                "base",
            )
        ]
        for index, (origin, radius) in enumerate(zip(origins, JOINT_RADII_M)):
            bodies.append(
                Capsule(
                    f"joint_{index + 1}",
                    origin.copy(),
                    origin.copy(),
                    radius,
                    index,
                    index,
                    "joint",
                )
            )
        for index, radius in enumerate(LINK_RADII_M):
            bodies.append(
                Capsule(
                    f"link_{index + 1}",
                    origins[index].copy(),
                    origins[index + 1].copy(),
                    radius,
                    index,
                    index + 1,
                    "link",
                )
            )
        bodies.append(
            Capsule(
                "flange",
                origins[5].copy(),
                flange_face.copy(),
                FLANGE_RADIUS_M,
                5,
                6,
                "flange",
            )
        )
        tcp_outside_flange = segment_distance_m(
            tcp_position,
            tcp_position,
            origins[5],
            flange_face,
        ) > 1.0e-9
        if tcp_outside_flange:
            bodies.append(
                Capsule(
                    "tool",
                    flange_face.copy(),
                    tcp_position.copy(),
                    self.tool_radius_m,
                    6,
                    7,
                    "tool",
                )
            )
        return tuple(bodies)

    @staticmethod
    def _topology_gap(first: Capsule, second: Capsule) -> int:
        if first.last_node < second.first_node:
            return second.first_node - first.last_node
        if second.last_node < first.first_node:
            return first.first_node - second.last_node
        return 0

    @classmethod
    def _pair_is_structurally_connected(
        cls, first: Capsule, second: Capsule
    ) -> bool:
        gap = cls._topology_gap(first, second)
        if gap == 0:
            return True
        if first.kind == "base" or second.kind == "base":
            moving_body = second if first.kind == "base" else first
            # The shoulder and upper-arm bodies are mounted into the base
            # housing and intentionally overlap its conservative capsule.
            return moving_body.first_node <= 1
        # A joint housing intentionally overlaps the links mounted immediately
        # on either side.  Those construction overlaps are not collisions.
        if gap == 1 and (first.kind == "joint" or second.kind == "joint"):
            return True
        return False

    def validate(
        self,
        joints_rad: Sequence[float],
        tcp_transform: Optional[Sequence[Sequence[float]]] = None,
    ) -> None:
        bodies = self.capsules(joints_rad, tcp_transform)

        for body in bodies:
            if body.kind == "base":
                continue
            clearance = (
                min(float(body.start[2]), float(body.end[2]))
                - body.radius_m
                - self.plane_z_m
            )
            if clearance < self.safety_margin_m:
                raise AuboCollisionError(
                    f"{body.name} is too close to the plane Z="
                    f"{self.plane_z_m:.3f} m (clearance {clearance:.3f} m, "
                    f"required {self.safety_margin_m:.3f} m)"
                )

        for first_index, first in enumerate(bodies):
            for second in bodies[first_index + 1 :]:
                if self._pair_is_structurally_connected(first, second):
                    continue
                distance = segment_distance_m(
                    first.start,
                    first.end,
                    second.start,
                    second.end,
                )
                clearance = distance - first.radius_m - second.radius_m
                if clearance < self.safety_margin_m:
                    raise AuboCollisionError(
                        f"self-collision risk between {first.name} and "
                        f"{second.name} (clearance {clearance:.3f} m, required "
                        f"{self.safety_margin_m:.3f} m)"
                    )


__all__ = (
    "AuboCollisionError",
    "AuboI7CollisionChecker",
    "Capsule",
    "segment_distance_m",
)
