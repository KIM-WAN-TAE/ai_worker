#!/usr/bin/env python3
"""Create synchronized command/tracking summaries from one inspector MCAP."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from logic import DEFAULT_LIMITS, finite


COMMAND_TOPICS = {
    "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory",
    "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory",
}
CONTROLLER_TOPICS = {
    "/arm_l_controller/controller_state",
    "/arm_r_controller/controller_state",
}
DYNAMIC_TOPIC = "/dynamic_joint_states"


def percentile(values: list[float], ratio: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(ratio * len(ordered)) - 1))
    return ordered[index]


def degrees(value: float | None) -> float | None:
    return math.degrees(value) if value is not None else None


def load_context(session: Path) -> dict[str, Any]:
    sidecar = session / "motion_inspector.json"
    if not sidecar.exists():
        return {"limits": DEFAULT_LIMITS, "events": [], "command_source": {"mode": "UNKNOWN"}}
    try:
        payload = json.loads(sidecar.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return {"limits": DEFAULT_LIMITS, "events": [], "command_source": {"mode": "UNKNOWN"}}
    payload.setdefault("limits", DEFAULT_LIMITS)
    payload.setdefault("events", [])
    payload.setdefault("command_source", {"mode": "UNKNOWN"})
    return payload


def analyze_session(session: Path, write_files: bool = True) -> dict[str, Any]:
    session = session.resolve()
    if not (session / "metadata.yaml").exists():
        raise ValueError(f"not a completed rosbag session: {session}")
    context = load_context(session)
    limits = context["limits"]
    commands: dict[str, list[tuple[int, float]]] = defaultdict(list)
    tracking: dict[str, list[tuple[int, float, float, float]]] = defaultdict(list)
    currents: dict[str, list[float]] = defaultdict(list)
    hardware_errors: dict[str, int] = defaultdict(int)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(session), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = COMMAND_TOPICS | CONTROLLER_TOPICS | {DYNAMIC_TOPIC}
    first_stamp: int | None = None
    last_stamp: int | None = None

    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic not in wanted or topic not in types:
            continue
        first_stamp = stamp if first_stamp is None else min(first_stamp, stamp)
        last_stamp = stamp if last_stamp is None else max(last_stamp, stamp)
        msg = deserialize_message(raw, get_message(types[topic]))
        if topic in COMMAND_TOPICS:
            names = list(msg.joint_names)
            for point in msg.points:
                for name, value in zip(names, point.positions):
                    number = finite(value)
                    if number is not None:
                        commands[name].append((stamp, number))
        elif topic in CONTROLLER_TOPICS:
            names = list(msg.joint_names)
            references = list(msg.reference.positions)
            feedbacks = list(msg.feedback.positions)
            errors = list(msg.error.positions)
            for index, name in enumerate(names):
                reference = finite(references[index]) if index < len(references) else None
                feedback = finite(feedbacks[index]) if index < len(feedbacks) else None
                error = finite(errors[index]) if index < len(errors) else None
                if reference is None or feedback is None:
                    continue
                if error is None:
                    error = reference - feedback
                tracking[name].append((stamp, reference, feedback, error))
        elif topic == DYNAMIC_TOPIC:
            for dxl_name, interface in zip(msg.joint_names, msg.interface_values):
                joint = next(
                    (name for name, item in limits.items() if item.get("dxl") == dxl_name),
                    None,
                )
                if not joint:
                    continue
                values = dict(zip(interface.interface_names, interface.values))
                current = finite(values.get("Present Current"))
                if current is not None:
                    currents[joint].append(current)
                error = finite(values.get("Hardware Error Status"))
                if error:
                    hardware_errors[joint] += 1

    joints = sorted(set(commands) | set(tracking))
    rows: list[dict[str, Any]] = []
    warn_rad = math.radians(float(context.get("thresholds", {}).get("tracking_warn_deg", 2.0)))
    for joint in joints:
        errors = [abs(item[3]) for item in tracking.get(joint, [])]
        command_values = [item[1] for item in commands.get(joint, [])]
        outside_count = 0
        configured = limits.get(joint)
        if configured and command_values:
            low, high = configured["hardware"]
            outside_count = sum(value < low or value > high for value in command_values)
        sample_periods = []
        samples = tracking.get(joint, [])
        for previous, current in zip(samples, samples[1:]):
            sample_periods.append(max(0.0, (current[0] - previous[0]) / 1e9))
        warning_duration = sum(
            period
            for period, sample in zip(sample_periods, samples)
            if abs(sample[3]) > warn_rad
        )
        row = {
            "joint": joint,
            "samples": len(errors),
            "command_min_deg": degrees(min(command_values)) if command_values else None,
            "command_max_deg": degrees(max(command_values)) if command_values else None,
            "mae_deg": degrees(sum(errors) / len(errors)) if errors else None,
            "rmse_deg": degrees(math.sqrt(sum(value * value for value in errors) / len(errors))) if errors else None,
            "p95_deg": degrees(percentile(errors, 0.95)),
            "max_error_deg": degrees(max(errors)) if errors else None,
            "warning_duration_s": warning_duration,
            "outside_dxl_samples": outside_count,
            "outside_dxl_ratio": outside_count / len(command_values) if command_values else None,
            "max_abs_current": max((abs(value) for value in currents.get(joint, [])), default=None),
            "hardware_error_samples": hardware_errors.get(joint, 0),
        }
        rows.append(row)

    event_counts: dict[str, int] = defaultdict(int)
    for event in context.get("events", []):
        event_counts[str(event.get("type") or "SYSTEM")] += 1
    configured_limits = []
    for joint, item in sorted(limits.items()):
        urdf_low, urdf_high = item["urdf"]
        dxl_low, dxl_high = item["hardware"]
        configured_limits.append({
            "joint": joint,
            "model": item.get("model"),
            "ticks_per_rev": item.get("ticks_per_rev"),
            "ticks": item.get("ticks"),
            "urdf_deg": [degrees(urdf_low), degrees(urdf_high)],
            "dxl_config_deg": [degrees(dxl_low), degrees(dxl_high)],
            "uncovered_low_deg": degrees(max(0.0, dxl_low - urdf_low)),
            "uncovered_high_deg": degrees(max(0.0, urdf_high - dxl_high)),
            "source": item.get("source", "unknown"),
        })
    summary = {
        "schema_version": 1,
        "session": session.name,
        "duration_s": ((last_stamp - first_stamp) / 1e9) if first_stamp is not None and last_stamp is not None else 0.0,
        "source_mode": context.get("command_source", {}).get("mode", "UNKNOWN"),
        "config_digest": context.get("config_digest", "unknown"),
        "event_counts": dict(sorted(event_counts.items())),
        "configured_limits": configured_limits,
        "joints": rows,
    }
    if write_files:
        write_report_files(session, summary)
    return summary


def _cell(value: Any) -> str:
    if value is None:
        return "—"
    if isinstance(value, float):
        return f"{value:.4f}"
    return str(value)


def write_report_files(session: Path, summary: dict[str, Any]) -> None:
    (session / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    fields = list(summary["joints"][0].keys()) if summary["joints"] else ["joint"]
    with (session / "joint_metrics.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(summary["joints"])

    lines = [
        f"# Motion Inspector Report — {summary['session']}",
        "",
        f"- Duration: `{summary['duration_s']:.3f} s`",
        f"- Command source: `{summary['source_mode']}`",
        f"- Runtime config: `{summary['config_digest']}`",
        "",
        "## Configured arm limits",
        "",
        "> Configuration snapshot from ros2_control; this is not a DXL EEPROM read-back.",
        "",
        "| Joint | Model | URDF [deg] | DXL config [deg] | Uncovered low/high [deg] | Tick limit |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for item in summary.get("configured_limits", []):
        ticks = item.get("ticks") or [None, None]
        lines.append(
            "| {joint} | {model} | {ul:.2f} ~ {uh:.2f} | {dl:.2f} ~ {dh:.2f} | "
            "{gl:.2f} / {gh:.2f} | {tl} ~ {th} |".format(
                joint=item["joint"],
                model=item.get("model") or "unknown",
                ul=item["urdf_deg"][0],
                uh=item["urdf_deg"][1],
                dl=item["dxl_config_deg"][0],
                dh=item["dxl_config_deg"][1],
                gl=item["uncovered_low_deg"],
                gh=item["uncovered_high_deg"],
                tl=ticks[0],
                th=ticks[1],
            )
        )
    lines.extend([
        "",
        "## Joint metrics",
        "",
        "| Joint | MAE [deg] | P95 [deg] | Max [deg] | DXL outside | Warn time [s] | HW errors |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ])
    for row in summary["joints"]:
        lines.append(
            "| {joint} | {mae} | {p95} | {maximum} | {outside} | {warn} | {errors} |".format(
                joint=row["joint"],
                mae=_cell(row["mae_deg"]),
                p95=_cell(row["p95_deg"]),
                maximum=_cell(row["max_error_deg"]),
                outside=row["outside_dxl_samples"],
                warn=_cell(row["warning_duration_s"]),
                errors=row["hardware_error_samples"],
            )
        )
    lines.extend(["", "## Events", ""])
    if summary["event_counts"]:
        lines.extend(f"- `{key}`: {value}" for key, value in summary["event_counts"].items())
    else:
        lines.append("- No inspector events recorded.")
    (session / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    parser.add_argument("--no-write", action="store_true")
    args = parser.parse_args()
    summary = analyze_session(args.session, write_files=not args.no_write)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
