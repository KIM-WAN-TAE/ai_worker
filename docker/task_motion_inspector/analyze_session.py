#!/usr/bin/env python3
"""Print a synchronized J7 command/controller/hardware summary for one session."""

from __future__ import annotations

import argparse
import math
from collections import defaultdict
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from logic import LIMITS


LEFT_COMMAND = "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory"
LEFT_CONTROLLER = "/arm_l_controller/controller_state"
JOINT_STATES = "/joint_states"
DYNAMIC_STATES = "/dynamic_joint_states"
DXL_POSITION_UNIT = 0.000010342691863670101


def append_point(series, key, stamp, value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return
    if math.isfinite(number):
        series[key].append((stamp, number))


def describe(name, points, origin):
    if not points:
        print(f"{name:20s}: no data")
        return
    values = [value for _, value in points]
    minimum_stamp, minimum = min(points, key=lambda item: item[1])
    maximum_stamp, maximum = max(points, key=lambda item: item[1])
    print(
        f"{name:20s}: {math.degrees(minimum):9.3f}° .. {math.degrees(maximum):9.3f}°  "
        f"min@{(minimum_stamp-origin)/1e9:7.3f}s max@{(maximum_stamp-origin)/1e9:7.3f}s n={len(points)}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.session), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = {LEFT_COMMAND, LEFT_CONTROLLER, JOINT_STATES, DYNAMIC_STATES}
    series = defaultdict(list)
    origin = None

    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic not in wanted:
            continue
        origin = stamp if origin is None else min(origin, stamp)
        msg = deserialize_message(raw, get_message(types[topic]))
        if topic == LEFT_COMMAND and "arm_l_joint7" in msg.joint_names:
            index = list(msg.joint_names).index("arm_l_joint7")
            for point in msg.points:
                if index < len(point.positions):
                    append_point(series, "trajectory", stamp, point.positions[index])
        elif topic == LEFT_CONTROLLER and "arm_l_joint7" in msg.joint_names:
            index = list(msg.joint_names).index("arm_l_joint7")
            for key in ("reference", "output", "feedback"):
                values = getattr(msg, key).positions
                if index < len(values):
                    append_point(series, key, stamp, values[index])
        elif topic == JOINT_STATES and "arm_l_joint7" in msg.name:
            index = list(msg.name).index("arm_l_joint7")
            if index < len(msg.position):
                append_point(series, "joint_states", stamp, msg.position[index])
        elif topic == DYNAMIC_STATES and "dxl37" in msg.joint_names:
            index = list(msg.joint_names).index("dxl37")
            interface = msg.interface_values[index]
            values = dict(zip(interface.interface_names, interface.values))
            if "Present Position" in values:
                value = float(values["Present Position"])
                value = value * DXL_POSITION_UNIT if abs(value) > 100.0 else value
                append_point(series, "dxl_present", stamp, value)

    if origin is None:
        raise SystemExit("No selected topics found")
    print(f"session: {args.session.name}")
    for key in ("trajectory", "reference", "output", "feedback", "joint_states", "dxl_present"):
        describe(key, series[key], origin)

    hardware_low, hardware_high = LIMITS["arm_l_joint7"]["hardware"]
    outside = [(stamp, value) for stamp, value in series["trajectory"] if value < hardware_low or value > hardware_high]
    if outside:
        first = (outside[0][0] - origin) / 1e9
        last = (outside[-1][0] - origin) / 1e9
        ratio = 100.0 * len(outside) / len(series["trajectory"])
        print(
            f"outside DXL range   : {len(outside)}/{len(series['trajectory'])} ({ratio:.1f}%), "
            f"first={first:.3f}s last={last:.3f}s span={last-first:.3f}s"
        )
    else:
        print("outside DXL range   : 0")


if __name__ == "__main__":
    main()
