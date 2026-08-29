"""Guided VR-position calibration for the RM65 base coordinate frame."""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Sequence

import numpy as np


SCHEMA_VERSION = 1
INPUT_FRAME = "unity_sender_z_minus_x_y_v1"
OUTPUT_FRAME = "rm65_base"


class CalibrationError(ValueError):
    """A calibration sample or saved calibration is invalid."""


def _vector3(value: Sequence[float], name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise CalibrationError(f"{name} must contain three finite values")
    return result.copy()


def _rotation_matrix(value: Sequence[Sequence[float]]) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise CalibrationError("rotation_vr_to_robot must be a finite 3x3 matrix")
    orthogonality_error = float(
        np.linalg.norm(result @ result.T - np.eye(3), ord="fro")
    )
    determinant = float(np.linalg.det(result))
    if orthogonality_error > 1.0e-6 or abs(determinant - 1.0) > 1.0e-6:
        raise CalibrationError(
            "rotation_vr_to_robot must be an orthonormal right-handed rotation"
        )
    return result.copy()


def _finite_tuple(value, length: int, name: str) -> tuple[float, ...]:
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(
            f"{name} must contain {length} finite values"
        ) from exc
    if len(result) != length or not all(math.isfinite(item) for item in result):
        raise CalibrationError(f"{name} must contain {length} finite values")
    return result


@dataclass(frozen=True)
class CalibrationResult:
    """A validated 1:1 VR-position mapping and its quality diagnostics."""

    rotation_vr_to_robot: np.ndarray
    axis_lengths_m: tuple[float, float, float]
    pairwise_angles_deg: tuple[float, float, float]
    axis_fit_errors_deg: tuple[float, float, float]
    rms_fit_error_deg: float
    created_at_utc: str


@dataclass(frozen=True)
class CalibrationUpdate:
    """One user-facing update from the guided calibration state machine."""

    message: str
    result: Optional[CalibrationResult] = None


def solve_position_mapping(
    center: Sequence[float],
    positive_x: Sequence[float],
    positive_y: Sequence[float],
    positive_z: Sequence[float],
    *,
    min_axis_displacement_m: float = 0.08,
    max_axis_displacement_m: float = 0.25,
    min_pairwise_angle_deg: float = 75.0,
    max_pairwise_angle_deg: float = 105.0,
    min_right_handed_triple_product: float = 0.75,
    max_condition_number: float = 1.5,
    max_axis_fit_error_deg: float = 12.0,
    max_rms_fit_error_deg: float = 8.0,
) -> CalibrationResult:
    """Fit the closest proper rotation from three measured VR directions."""

    origin = _vector3(center, "center")
    endpoints = (
        _vector3(positive_x, "positive_x"),
        _vector3(positive_y, "positive_y"),
        _vector3(positive_z, "positive_z"),
    )
    vectors = np.column_stack(tuple(point - origin for point in endpoints))
    lengths = np.linalg.norm(vectors, axis=0)
    if np.any(lengths < min_axis_displacement_m) or np.any(
        lengths > max_axis_displacement_m
    ):
        measured = ", ".join(
            f"+{axis}={length * 100.0:.1f} cm"
            for axis, length in zip("XYZ", lengths)
        )
        raise CalibrationError(
            "each axis motion must be between "
            f"{min_axis_displacement_m * 100.0:.0f} and "
            f"{max_axis_displacement_m * 100.0:.0f} cm; "
            f"measured from the original center: {measured}"
        )

    directions = vectors / lengths
    pair_indices = ((0, 1), (0, 2), (1, 2))
    pairwise_angles = tuple(
        math.degrees(
            math.acos(
                float(
                    np.clip(
                        np.dot(directions[:, first], directions[:, second]),
                        -1.0,
                        1.0,
                    )
                )
            )
        )
        for first, second in pair_indices
    )
    if any(
        angle < min_pairwise_angle_deg or angle > max_pairwise_angle_deg
        for angle in pairwise_angles
    ):
        raise CalibrationError(
            "sampled +X/+Y/+Z motions are not sufficiently perpendicular"
        )

    triple_product = float(np.linalg.det(directions))
    if triple_product < min_right_handed_triple_product:
        raise CalibrationError(
            "sampled axes are not a clear right-handed +X/+Y/+Z frame"
        )
    condition_number = float(np.linalg.cond(directions))
    if condition_number > max_condition_number:
        raise CalibrationError("sampled axes are ill-conditioned")

    # For A=[ux uy uz], solve min ||R A-I||_F over SO(3).  Since the desired
    # directions form I, the Procrustes cross-covariance is A.T.
    left, _, right_transpose = np.linalg.svd(directions.T)
    correction = np.eye(3, dtype=np.float64)
    correction[2, 2] = (
        1.0 if np.linalg.det(left @ right_transpose) >= 0.0 else -1.0
    )
    rotation = left @ correction @ right_transpose
    rotation = _rotation_matrix(rotation)

    mapped_directions = rotation @ directions
    axis_errors = tuple(
        math.degrees(
            math.acos(float(np.clip(mapped_directions[index, index], -1.0, 1.0)))
        )
        for index in range(3)
    )
    rms_fit_error = math.sqrt(
        sum(error * error for error in axis_errors) / len(axis_errors)
    )
    if max(axis_errors) > max_axis_fit_error_deg or (
        rms_fit_error > max_rms_fit_error_deg
    ):
        raise CalibrationError("calibration direction fit error is too large")

    return CalibrationResult(
        rotation_vr_to_robot=rotation,
        axis_lengths_m=tuple(float(value) for value in lengths),
        pairwise_angles_deg=pairwise_angles,
        axis_fit_errors_deg=axis_errors,
        rms_fit_error_deg=float(rms_fit_error),
        created_at_utc=datetime.now(timezone.utc).isoformat(),
    )


class GuidedPositionCalibrator:
    """Collect stable center/+X/+Y/+Z samples without moving the robot."""

    _STAGES = (
        (
            "center",
            "将手柄放在舒适的标定原点，保持静止，然后按 Enter。",
        ),
        (
            "positive_x",
            "从同一原点沿机器人基坐标 +X 移动 10～15 cm，保持静止后按 Enter。",
        ),
        (
            "positive_y",
            "回到同一原点，再沿机器人基坐标 +Y 移动 10～15 cm，保持静止后按 Enter。",
        ),
        (
            "positive_z",
            "回到同一原点，再沿机器人基坐标 +Z 移动 10～15 cm，保持静止后按 Enter。",
        ),
    )

    _STAGE_LABELS = {
        "center": "原点",
        "positive_x": "+X",
        "positive_y": "+Y",
        "positive_z": "+Z",
    }

    def __init__(
        self,
        *,
        sample_count: int = 40,
        max_sample_rms_m: float = 0.003,
        max_sample_deviation_m: float = 0.008,
        min_axis_displacement_m: float = 0.08,
        max_axis_displacement_m: float = 0.25,
    ) -> None:
        if not isinstance(sample_count, int) or sample_count < 2:
            raise ValueError("sample_count must be an integer of at least two")
        positive = (
            max_sample_rms_m,
            max_sample_deviation_m,
            min_axis_displacement_m,
            max_axis_displacement_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("calibration sampling thresholds must be positive")
        if max_sample_rms_m >= max_sample_deviation_m:
            raise ValueError("RMS threshold must be below maximum deviation")
        if min_axis_displacement_m >= max_axis_displacement_m:
            raise ValueError("minimum axis displacement must be below maximum")

        self.sample_count = sample_count
        self.max_sample_rms_m = float(max_sample_rms_m)
        self.max_sample_deviation_m = float(max_sample_deviation_m)
        self.min_axis_displacement_m = float(min_axis_displacement_m)
        self.max_axis_displacement_m = float(max_axis_displacement_m)
        self._stage_index: Optional[int] = None
        self._capture_requested = False
        self._samples: list[np.ndarray] = []
        self._points: dict[str, np.ndarray] = {}
        self._pending_result: Optional[CalibrationResult] = None

    @property
    def active(self) -> bool:
        return self._stage_index is not None or self._pending_result is not None

    @property
    def awaiting_confirmation(self) -> bool:
        return self._pending_result is not None

    @property
    def capture_requested(self) -> bool:
        return self._capture_requested

    @property
    def pending_result(self) -> CalibrationResult:
        if self._pending_result is None:
            raise RuntimeError("calibration is not awaiting confirmation")
        return self._pending_result

    @property
    def prompt(self) -> str:
        if self._pending_result is not None:
            return "按 Y 保存并启用，按 R 重新标定，按 Esc 取消。"
        if self._stage_index is None:
            return "按 Q 开始 VR 坐标标定。"
        return self._STAGES[self._stage_index][1]

    def _reset_sampling(self) -> None:
        self._stage_index = 0
        self._capture_requested = False
        self._samples = []
        self._points = {}
        self._pending_result = None

    def _reset_idle(self) -> None:
        self._stage_index = None
        self._capture_requested = False
        self._samples = []
        self._points = {}
        self._pending_result = None

    @staticmethod
    def _format_position(position: np.ndarray) -> str:
        return (
            f"[x={position[0]:+.4f}, y={position[1]:+.4f}, "
            f"z={position[2]:+.4f}] m"
        )

    @staticmethod
    def _format_displacement(displacement: np.ndarray) -> str:
        return (
            f"[dx={displacement[0] * 100.0:+.1f}, "
            f"dy={displacement[1] * 100.0:+.1f}, "
            f"dz={displacement[2] * 100.0:+.1f}] cm"
        )

    def _point_diagnostics(self) -> str:
        center = self._points.get("center")
        lines = ["VR 三维采样位置（Unity 发送坐标，单位 m）："]
        for stage_name, _ in self._STAGES:
            point = self._points.get(stage_name)
            if point is None:
                continue
            label = self._STAGE_LABELS[stage_name]
            line = f"  {label}: {self._format_position(point)}"
            if center is not None and stage_name != "center":
                displacement = point - center
                line += (
                    f"；相对原点 {self._format_displacement(displacement)}；"
                    f"距离 {np.linalg.norm(displacement) * 100.0:.1f} cm"
                )
            lines.append(line)
        return "\n".join(lines)

    def start(self) -> CalibrationUpdate:
        self._reset_sampling()
        return CalibrationUpdate("VR 坐标标定已开始。" + self.prompt)

    def request_capture(self) -> CalibrationUpdate:
        if not self.active:
            return CalibrationUpdate("标定尚未开始，请先按 Q。")
        if self.awaiting_confirmation:
            return CalibrationUpdate(self.prompt)
        if self._capture_requested:
            return CalibrationUpdate("正在采样，请保持手柄静止。")
        self._samples = []
        self._capture_requested = True
        return CalibrationUpdate(
            f"开始采集 {self.sample_count} 帧，请保持手柄静止。"
        )

    def observe(self, position: Sequence[float]) -> Optional[CalibrationUpdate]:
        sample = _vector3(position, "controller position")
        if not self._capture_requested or self._stage_index is None:
            return None
        self._samples.append(sample)
        if len(self._samples) < self.sample_count:
            return None

        samples = np.stack(self._samples, axis=0)
        mean = np.mean(samples, axis=0)
        deviations = np.linalg.norm(samples - mean, axis=1)
        rms = float(math.sqrt(np.mean(deviations * deviations)))
        maximum = float(np.max(deviations))
        self._capture_requested = False
        self._samples = []
        if rms > self.max_sample_rms_m or maximum > self.max_sample_deviation_m:
            return CalibrationUpdate(
                "采样失败：手柄移动或抖动过大（"
                f"RMS {rms * 1000.0:.1f} mm，最大 {maximum * 1000.0:.1f} mm）。"
                f"本次 {self.sample_count} 帧平均三维位置："
                f"{self._format_position(mean)}。"
                "请保持静止并重新按 Enter。"
            )

        stage_name = self._STAGES[self._stage_index][0]
        stage_label = self._STAGE_LABELS[stage_name]
        if stage_name != "center":
            center = self._points["center"]
            displacement = mean - center
            distance = float(np.linalg.norm(displacement))
            if (
                distance < self.min_axis_displacement_m
                or distance > self.max_axis_displacement_m
            ):
                relation = (
                    "过短"
                    if distance < self.min_axis_displacement_m
                    else "过长"
                )
                return CalibrationUpdate(
                    f"{stage_label} 采样不合格：相对最初原点的距离"
                    f"为 {distance * 100.0:.1f} cm（{relation}），允许范围为 "
                    f"{self.min_axis_displacement_m * 100.0:.0f}～"
                    f"{self.max_axis_displacement_m * 100.0:.0f} cm。\n"
                    f"  原点: {self._format_position(center)}\n"
                    f"  {stage_label}: {self._format_position(mean)}\n"
                    f"  位移: {self._format_displacement(displacement)}\n"
                    "当前点未保存；请从同一个原点移动 10～15 cm，"
                    "保持静止后重新按 Enter。"
                )

        self._points[stage_name] = mean
        if self._stage_index < len(self._STAGES) - 1:
            self._stage_index += 1
            if stage_name == "center":
                sample_text = (
                    f"采样成功：原点三维位置 {self._format_position(mean)}。\n"
                )
            else:
                displacement = mean - self._points["center"]
                sample_text = (
                    f"采样成功：{stage_label} 三维位置 "
                    f"{self._format_position(mean)}；相对原点 "
                    f"{self._format_displacement(displacement)}；"
                    f"距离 {np.linalg.norm(displacement) * 100.0:.1f} cm。\n"
                )
            return CalibrationUpdate(sample_text + self.prompt)

        try:
            result = solve_position_mapping(
                self._points["center"],
                self._points["positive_x"],
                self._points["positive_y"],
                self._points["positive_z"],
                min_axis_displacement_m=self.min_axis_displacement_m,
                max_axis_displacement_m=self.max_axis_displacement_m,
            )
        except CalibrationError as exc:
            diagnostics = self._point_diagnostics()
            self._reset_sampling()
            return CalibrationUpdate(
                f"标定质量检查失败：{exc}。\n{diagnostics}\n"
                "旧标定未改变，请重新采样。"
                + self.prompt
            )

        self._stage_index = None
        self._pending_result = result
        matrix_text = np.array2string(
            result.rotation_vr_to_robot, precision=6, suppress_small=True
        )
        return CalibrationUpdate(
            "标定求解完成；距离比例保持 1:1。\n"
            f"{self._point_diagnostics()}\n"
            f"VR→机器人基坐标矩阵：\n{matrix_text}\n"
            f"轴向拟合误差：{result.axis_fit_errors_deg} deg；"
            f"RMS {result.rms_fit_error_deg:.2f} deg。\n"
            + self.prompt,
            result=result,
        )

    def redo(self) -> CalibrationUpdate:
        if not self.active:
            return CalibrationUpdate("标定尚未开始，请先按 Q。")
        self._reset_sampling()
        return CalibrationUpdate("已清除本轮样本。" + self.prompt)

    def cancel(self) -> CalibrationUpdate:
        self._reset_idle()
        return CalibrationUpdate("标定已取消；原有映射保持不变。")

    def finish(self) -> CalibrationResult:
        result = self.pending_result
        self._reset_idle()
        return result


def default_calibration_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "vr_position_calibration.json"


def save_calibration_atomic(path: Path | str, result: CalibrationResult) -> None:
    """Atomically save one validated calibration next to its temporary file."""

    target = Path(path)
    rotation = _rotation_matrix(result.rotation_vr_to_robot)
    payload = {
        "schema_version": SCHEMA_VERSION,
        "created_at_utc": str(result.created_at_utc),
        "input_frame": INPUT_FRAME,
        "output_frame": OUTPUT_FRAME,
        "units": "m",
        "distance_scale": 1.0,
        "rotation_vr_to_robot": rotation.tolist(),
        "quality": {
            "axis_lengths_m": list(result.axis_lengths_m),
            "pairwise_angles_deg": list(result.pairwise_angles_deg),
            "axis_fit_errors_deg": list(result.axis_fit_errors_deg),
            "rms_fit_error_deg": float(result.rms_fit_error_deg),
        },
    }
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    except Exception:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise


def load_calibration(path: Path | str) -> CalibrationResult:
    """Load a calibration without silently repairing incompatible data."""

    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CalibrationError(f"cannot read calibration file: {exc}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != SCHEMA_VERSION:
        raise CalibrationError("unsupported calibration schema version")
    if payload.get("input_frame") != INPUT_FRAME:
        raise CalibrationError("calibration input frame does not match Unity sender")
    if payload.get("output_frame") != OUTPUT_FRAME or payload.get("units") != "m":
        raise CalibrationError("calibration output frame or units are incompatible")
    try:
        distance_scale = float(payload.get("distance_scale", math.nan))
    except (TypeError, ValueError) as exc:
        raise CalibrationError("calibration distance scale must be 1:1") from exc
    if not math.isclose(distance_scale, 1.0):
        raise CalibrationError("calibration distance scale must be 1:1")

    quality = payload.get("quality")
    if not isinstance(quality, dict):
        raise CalibrationError("calibration quality metadata is missing")
    rotation = _rotation_matrix(payload.get("rotation_vr_to_robot"))
    axis_lengths = _finite_tuple(quality.get("axis_lengths_m", ()), 3, "axis lengths")
    pairwise_angles = _finite_tuple(
        quality.get("pairwise_angles_deg", ()), 3, "pairwise angles"
    )
    axis_errors = _finite_tuple(
        quality.get("axis_fit_errors_deg", ()), 3, "axis fit errors"
    )
    try:
        rms_error = float(quality.get("rms_fit_error_deg", math.nan))
    except (TypeError, ValueError) as exc:
        raise CalibrationError(
            "RMS fit error must be finite and non-negative"
        ) from exc
    if not math.isfinite(rms_error) or rms_error < 0.0:
        raise CalibrationError("RMS fit error must be finite and non-negative")
    created_at = payload.get("created_at_utc")
    if not isinstance(created_at, str) or not created_at:
        raise CalibrationError("created_at_utc is missing")
    return CalibrationResult(
        rotation_vr_to_robot=rotation,
        axis_lengths_m=axis_lengths,
        pairwise_angles_deg=pairwise_angles,
        axis_fit_errors_deg=axis_errors,
        rms_fit_error_deg=rms_error,
        created_at_utc=created_at,
    )


class ConsoleKeyReader:
    """Non-blocking Windows console keys, with an injectable test backend."""

    def __init__(self, backend=None) -> None:
        if backend is None:
            try:
                import msvcrt as backend  # type: ignore[no-redef]
            except ImportError:
                backend = None
        self._backend = backend

    @property
    def available(self) -> bool:
        return self._backend is not None

    def poll(self) -> tuple[str, ...]:
        if self._backend is None:
            return ()
        keys: list[str] = []
        try:
            while self._backend.kbhit():
                key = self._backend.getwch()
                if key == "\x03":
                    raise KeyboardInterrupt
                if key in ("\x00", "\xe0"):
                    if self._backend.kbhit():
                        self._backend.getwch()
                    continue
                keys.append(key)
        except OSError:
            self._backend = None
            return ()
        return tuple(keys)


__all__ = (
    "CalibrationError",
    "CalibrationResult",
    "CalibrationUpdate",
    "ConsoleKeyReader",
    "GuidedPositionCalibrator",
    "default_calibration_path",
    "load_calibration",
    "save_calibration_atomic",
    "solve_position_mapping",
)
