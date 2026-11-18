"""Unity controller protocol; standard library only, with no robot SDK imports."""

import json
import math
from dataclasses import dataclass
from typing import Optional


DIAGNOSTIC_PREFIX = "#VR_DIAG "
INPUT_INVALID_PREFIX = "#VR_INPUT_INVALID "


@dataclass(frozen=True)
class ControllerSample:
    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]
    trigger_held: bool
    sequence_id: Optional[int] = None
    unity_time_s: Optional[float] = None


def parse_controller_line(line):
    """Parse a legacy 9-field or timestamped 11-field controller record."""

    fields = line.strip().split(",")
    if len(fields) not in (9, 11):
        raise ValueError(f"expected 9 or 11 controller fields, received {len(fields)}")
    values = [float(field) for field in fields[:9]]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("controller record contains a non-finite value")
    sequence_id = None
    unity_time_s = None
    if len(fields) == 11:
        sequence_id = int(fields[9])
        unity_time_s = float(fields[10])
        if sequence_id < 0 or not math.isfinite(unity_time_s) or unity_time_s < 0:
            raise ValueError("controller timing fields are invalid")
    return ControllerSample(tuple(values[:3]), tuple(values[3:7]), bool(values[7]),
                            sequence_id, unity_time_s)


def diagnostic_vector(payload, key):
    vector = tuple(float(payload[key][axis]) for axis in "xyz")
    if not all(math.isfinite(value) for value in vector):
        raise ValueError(f"non-finite diagnostic {key}")
    return vector


def parse_pose_diagnostic(line):
    if not line.startswith(DIAGNOSTIC_PREFIX):
        raise ValueError("not a pose diagnostic record")
    payload = json.loads(line[len(DIAGNOSTIC_PREFIX):])
    if not isinstance(payload, dict) or payload.get("schema") != 1:
        raise ValueError("unsupported pose diagnostic schema")
    for key in ("local_position", "world_position", "sent_position",
                "parent_scale", "world_scale"):
        diagnostic_vector(payload, key)
    if payload.get("raw_position_valid"):
        diagnostic_vector(payload, "raw_position")
    return payload
