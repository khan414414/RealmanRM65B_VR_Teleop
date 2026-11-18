"""Read-only Unity pose diagnostics. No robot SDK or motion code is imported."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import select
import socket
import sys
import time

from teleop.vr_protocol import (
    DIAGNOSTIC_PREFIX,
    INPUT_INVALID_PREFIX,
    diagnostic_vector,
    parse_controller_line,
    parse_pose_diagnostic,
)


def displacement(start, end):
    delta = tuple(b - a for a, b in zip(start, end))
    return delta, math.sqrt(sum(value * value for value in delta))


def raw_tracking_valid(payload):
    """A readable XR position may still be cached after tracking is lost."""
    return bool(payload.get("raw_position_valid")
                and payload.get("tracking_flag_valid")
                and payload.get("is_tracked")
                and int(payload.get("tracking_state", 0)) & 1)


def compare_pose_displacements(anchor, current):
    """Compare raw tracking metres and each scene-transform stage."""
    result = {}
    for key in ("raw_position", "local_position", "world_position", "sent_position"):
        if key == "raw_position" and not (
            raw_tracking_valid(anchor) and raw_tracking_valid(current)
            and anchor.get("device_name") == current.get("device_name")
        ):
            continue
        delta, length = displacement(diagnostic_vector(anchor, key),
                                     diagnostic_vector(current, key))
        result[key] = {"delta_m": delta, "distance_m": length}
    raw_length = result.get("raw_position", {}).get("distance_m", 0)
    if raw_length >= 0.02:
        result["world_to_raw_distance_ratio"] = result["world_position"]["distance_m"] / raw_length
    return result


def diagnostic_matches_sample(payload, sample):
    if sample is None or payload.get("sequence_id") != sample.sequence_id:
        return None
    sent = diagnostic_vector(payload, "sent_position")
    return max(abs(a - b) for a, b in zip(sent, sample.position)) <= 0.000002


def format_cm(position):
    return "[" + ", ".join(f"{value * 100:+.2f}" for value in position) + "] cm"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--duration", type=float, help="自动结束的秒数；默认持续运行")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("--duration 必须为正数")
    output = args.output or Path(".log_analysis") / (
        "vr_pose_diagnostics_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".jsonl"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    print("本程序只连接 Unity，不连接机器人。请先退出 teleop.vr_teleop，保持 Unity Play。")
    print("手柄停在起点后按 Enter（或 R）归零，再用尺子平移 12 cm，保持静止约 2 秒。")
    print("Ctrl+C 结束。诊断文件：", output.resolve())

    keyboard = None
    if sys.platform == "win32" and sys.stdin.isatty():
        import msvcrt
        keyboard = msvcrt
    start = time.monotonic()
    last_received = start
    last_activity = start
    next_report = start + 1
    latest_sample = None
    sample_anchor = None
    latest_diagnostic = None
    diagnostic_anchor = None
    records = 0
    previous_sequence = None
    repeated_sequences = 0
    buffer = b""
    announced_source = None
    announced_input = None
    invalid_input_reason = None
    with output.open("w", encoding="utf-8") as log:
        try:
            with socket.create_connection((args.host, args.port), timeout=3) as sock:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                while args.duration is None or time.monotonic() - start < args.duration:
                    if keyboard is not None:
                        while keyboard.kbhit():
                            key = keyboard.getwch()
                            if key == "\x03":
                                raise KeyboardInterrupt
                            if key in ("\r", "\n", "r", "R"):
                                sample_anchor = None
                                diagnostic_anchor = None
                                print("测量起点已重置，使用下一帧新数据。")
                    readable, _, _ = select.select([sock], [], [], 0.05)
                    if readable:
                        chunk = sock.recv(65536)
                        if not chunk:
                            print("Unity 已断开连接。")
                            break
                        buffer += chunk
                        last_activity = time.monotonic()
                        lines = buffer.split(b"\n")
                        buffer = lines.pop()
                        for raw in lines:
                            line = raw.decode("utf-8")
                            try:
                                if line.startswith(INPUT_INVALID_PREFIX):
                                    invalid_input_reason = line[len(INPUT_INVALID_PREFIX):].strip()
                                    latest_sample = None
                                    sample_anchor = None
                                    diagnostic_anchor = None
                                    log.write(json.dumps({"kind": "input_invalid", "host_time": time.time(),
                                                         "reason": invalid_input_reason}) + "\n")
                                elif line.startswith(DIAGNOSTIC_PREFIX):
                                    latest_diagnostic = parse_pose_diagnostic(line)
                                    if (diagnostic_anchor is None and raw_tracking_valid(latest_diagnostic)
                                            and latest_diagnostic.get("motion_record_valid", True)):
                                        diagnostic_anchor = latest_diagnostic
                                    match = diagnostic_matches_sample(latest_diagnostic, latest_sample)
                                    log.write(json.dumps({"kind": "layers", "host_time": time.time(),
                                                         "sample_matches": match,
                                                         "data": latest_diagnostic}) + "\n")
                                    source = latest_diagnostic.get("project_path")
                                    if source != announced_source:
                                        announced_source = source
                                        print("Unity 实际项目：", source)
                                        print("手柄对象：", latest_diagnostic.get("controller_path"))
                                        print("原始 XR 设备：", latest_diagnostic.get("device_name"))
                                    active = latest_diagnostic.get("position_source", {})
                                    identity = (active.get("device_id"), active.get("device_layout"))
                                    if identity != announced_input:
                                        announced_input = identity
                                        print("实际位置输入：", active)
                                else:
                                    latest_sample = parse_controller_line(line)
                                    invalid_input_reason = None
                                    records += 1
                                    sequence = latest_sample.sequence_id
                                    if sequence is not None and previous_sequence is not None and sequence <= previous_sequence:
                                        repeated_sequences += 1
                                    previous_sequence = sequence
                                    if sample_anchor is None:
                                        sample_anchor = latest_sample.position
                                    last_received = time.monotonic()
                                    log.write(json.dumps({"kind": "received", "host_time": time.time(),
                                                         "position": latest_sample.position,
                                                         "rotation": latest_sample.rotation,
                                                         "sequence_id": sequence,
                                                         "unity_time_s": latest_sample.unity_time_s}) + "\n")
                            except (ValueError, KeyError, TypeError) as exc:
                                print("忽略无效诊断记录：", exc)
                    now = time.monotonic()
                    if now >= next_report:
                        next_report = now + 1
                        log.flush()
                        print(f"\n收到 {records} 帧；重复/倒序帧 {repeated_sequences}；最新数据年龄 {(now-last_received)*1000:.0f} ms")
                        if invalid_input_reason is not None:
                            print("手柄输入已拒绝：", invalid_input_reason)
                        if latest_sample is not None and sample_anchor is not None:
                            delta, length = displacement(sample_anchor, latest_sample.position)
                            print(f"Python 收到位移：{format_cm(delta)}；距离 {length*100:.2f} cm")
                        if latest_diagnostic is not None and diagnostic_anchor is not None:
                            comparisons = compare_pose_displacements(diagnostic_anchor, latest_diagnostic)
                            for key, label in (("raw_position", "XR 原始"), ("local_position", "Unity 局部"),
                                               ("world_position", "Unity 世界"), ("sent_position", "Unity 发送")):
                                if key in comparisons:
                                    item = comparisons[key]
                                    print(f"{label}：{format_cm(item['delta_m'])}；距离 {item['distance_m']*100:.2f} cm")
                            print("父级累计缩放：", diagnostic_vector(latest_diagnostic, "parent_scale"))
                            print("XR tracking：", {key: latest_diagnostic.get(key) for key in
                                                        ("raw_position_valid", "is_tracked", "tracking_state")})
                            if not raw_tracking_valid(latest_diagnostic):
                                print("原始 XR 跟踪无效；不使用缓存位置计算原始位移。")
                            if "world_to_raw_distance_ratio" in comparisons:
                                print(f"世界/原始位移比例：{comparisons['world_to_raw_distance_ratio']:.3f}")
                        elif latest_diagnostic is None:
                            print("尚无分层诊断。需要 Unity 编译新版发送脚本，并重新进入 Play。")
                        else:
                            print("等待有效右手柄跟踪；当前无效位置不作为测量起点。")
                    if now - last_activity > 5:
                        print("超过 5 秒没有新手柄帧；请检查 Unity Play 和其他 Python 客户端是否已退出。")
                        break
        except KeyboardInterrupt:
            print("\n诊断结束。")
        except OSError as exc:
            print(f"无法读取 Unity：{exc}")
            print("请确认当前项目已进入 Play、Console 显示 Waiting for connection，且其他 Python 客户端已退出。")
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
