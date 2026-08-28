"""ROS-independent diagnostics for the motion inspector.

Keep parsing, limit conversion, and alert state machines in this module so they
can be unit-tested without a running ROS graph.
"""

from __future__ import annotations

import copy
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any


Y_SERIES_TICKS_PER_REV = 524_288
PH42_TICKS_PER_REV = 607_500
Y_SERIES_RAD_PER_TICK = 2.0 * math.pi / Y_SERIES_TICKS_PER_REV
PH42_RAD_PER_TICK = 2.0 * math.pi / PH42_TICKS_PER_REV
CLAMP_EPS_RAD = math.radians(0.15)
TRACKING_WARN_RAD = math.radians(2.0)


def _arm_limit(
    spec: dict[str, Any],
    urdf: tuple[float, float],
    ticks: tuple[int, int],
    source: str,
) -> dict[str, Any]:
    rad_per_tick = 2.0 * math.pi / int(spec["ticks_per_rev"])
    return {
        "joint": spec["joint"],
        "urdf": list(urdf),
        "hardware": [ticks[0] * rad_per_tick, ticks[1] * rad_per_tick],
        "ticks": list(ticks),
        "dxl": spec["dxl"],
        "dxl_id": spec["dxl_id"],
        "model": spec["model"],
        "ticks_per_rev": spec["ticks_per_rev"],
        "rad_per_tick": rad_per_tick,
        "model_source": "F2 follower ping/model mapping",
        "source": source,
    }


_MODEL_BY_INDEX = {
    1: ("YM080-230-R099", Y_SERIES_TICKS_PER_REV),
    2: ("YM080-230-R099", Y_SERIES_TICKS_PER_REV),
    3: ("YM070-210-R099", Y_SERIES_TICKS_PER_REV),
    4: ("YM070-210-R099", Y_SERIES_TICKS_PER_REV),
    5: ("YM070-210-R099", Y_SERIES_TICKS_PER_REV),
    6: ("YM070-210-R099", Y_SERIES_TICKS_PER_REV),
    7: ("PH42-020-S300-R", PH42_TICKS_PER_REV),
}


def _motor_spec(side: str, index: int) -> dict[str, Any]:
    dxl_id = index if side == "r" else 30 + index
    model, ticks_per_rev = _MODEL_BY_INDEX[index]
    return {
        "joint": f"arm_{side}_joint{index}",
        "dxl": f"dxl{dxl_id}",
        "dxl_id": dxl_id,
        "model": model,
        "ticks_per_rev": ticks_per_rev,
    }


# The model mapping is based on the actual F2 follower ping results:
# J1/J2 YM080, J3-J6 YM070, and J7 PH42 on both arms.
ARM_DXL_MAP = {
    spec["dxl"]: spec
    for side in ("r", "l")
    for index in range(1, 8)
    for spec in (_motor_spec(side, index),)
}

_FALLBACK_URDF = {
    "arm_r_joint1": (-3.14, 3.14),
    "arm_r_joint2": (-3.14, 0.0),
    "arm_r_joint3": (-3.14, 3.14),
    "arm_r_joint4": (-2.9361, 1.0786),
    "arm_r_joint5": (-3.14, 3.14),
    "arm_r_joint6": (-1.57, 1.57),
    "arm_r_joint7": (-1.5804, 1.8201),
    "arm_l_joint1": (-3.14, 3.14),
    "arm_l_joint2": (0.0, 3.14),
    "arm_l_joint3": (-3.14, 3.14),
    "arm_l_joint4": (-2.9361, 1.0786),
    "arm_l_joint5": (-3.14, 3.14),
    "arm_l_joint6": (-1.57, 1.57),
    "arm_l_joint7": (-1.8201, 1.5804),
}

_FALLBACK_TICKS = {
    "arm_r_joint1": (-262144, 262144),
    "arm_r_joint2": (-262144, 0),
    "arm_r_joint3": (-262144, 262144),
    "arm_r_joint4": (-245000, 90000),
    "arm_r_joint5": (-262144, 262144),
    "arm_r_joint6": (-131072, 131072),
    "arm_r_joint7": (-151875, 131875),
    "arm_l_joint1": (-262144, 262144),
    "arm_l_joint2": (0, 262144),
    "arm_l_joint3": (-262144, 262144),
    "arm_l_joint4": (-245000, 90000),
    "arm_l_joint5": (-262144, 262144),
    "arm_l_joint6": (-131072, 131072),
    "arm_l_joint7": (-151875, 131875),
}


# Safe fallback used only until the transient /robot_description arrives.
DEFAULT_LIMITS = {
    spec["joint"]: _arm_limit(
        spec,
        _FALLBACK_URDF[spec["joint"]],
        _FALLBACK_TICKS[spec["joint"]],
        "fallback configuration",
    )
    for spec in ARM_DXL_MAP.values()
}

def finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def safe_session_name(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip()).strip("._")
    return value[:80] or "motion"


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def parse_f2_limits(robot_description: str) -> dict[str, dict[str, Any]]:
    """Extract F2 arm URDF and configured DXL limits from expanded XML.

    The result describes the configuration loaded into ros2_control. It is not
    labelled as an EEPROM read-back because the standard state topics do not
    expose Max/Min Position Limit on this robot.
    """

    if not robot_description.strip():
        raise ValueError("robot_description is empty")
    root = ET.fromstring(robot_description)
    urdf: dict[str, tuple[float, float]] = {}
    gpio_ticks: dict[str, tuple[int, int]] = {}

    for element in root.iter():
        tag = _local_name(element.tag)
        if tag == "joint":
            name = element.attrib.get("name", "")
            if name not in DEFAULT_LIMITS:
                continue
            for child in element:
                if _local_name(child.tag) != "limit":
                    continue
                lower = finite(child.attrib.get("lower"))
                upper = finite(child.attrib.get("upper"))
                if lower is not None and upper is not None and lower < upper:
                    urdf[name] = (lower, upper)
        elif tag == "gpio":
            gpio_name = element.attrib.get("name", "")
            if gpio_name not in ARM_DXL_MAP:
                continue
            params: dict[str, str] = {}
            for child in element:
                if _local_name(child.tag) != "param":
                    continue
                key = re.sub(r"[_\s]+", " ", child.attrib.get("name", "").strip()).lower()
                params[key] = (child.text or "").strip()
            minimum = finite(params.get("min position limit"))
            maximum = finite(params.get("max position limit"))
            if minimum is not None and maximum is not None and minimum < maximum:
                gpio_ticks[gpio_name] = (int(minimum), int(maximum))

    result = copy.deepcopy(DEFAULT_LIMITS)
    for gpio_name, spec in ARM_DXL_MAP.items():
        joint = spec["joint"]
        fallback = DEFAULT_LIMITS[joint]
        radians = urdf.get(joint, tuple(fallback["urdf"]))
        ticks = gpio_ticks.get(gpio_name, tuple(fallback["ticks"]))
        source_parts = []
        if joint in urdf:
            source_parts.append("runtime URDF")
        if gpio_name in gpio_ticks:
            source_parts.append("runtime ros2_control")
        source = " + ".join(source_parts) or "fallback configuration"
        result[joint] = _arm_limit(spec, radians, ticks, source)
    return result


def trajectory_range_status(
    minimum: Any,
    maximum: Any,
    limits: dict[str, Any],
    margin_rad: float = CLAMP_EPS_RAD,
) -> str:
    """Classify a recorded trajectory range against configured hardware range."""
    low = finite(minimum)
    high = finite(maximum)
    if low is None or high is None or low > high:
        return "unknown"
    hardware_low, hardware_high = limits["hardware"]
    if low < hardware_low - margin_rad or high > hardware_high + margin_rad:
        return "outside"
    if low <= hardware_low + margin_rad or high >= hardware_high - margin_rad:
        return "near"
    return "ok"


def classify_joint(
    values: dict[str, Any],
    limits: dict[str, Any] | None,
    tracking_warn_rad: float = TRACKING_WARN_RAD,
) -> list[str]:
    """Return compact diagnostic flags for one latest-value snapshot."""
    flags: list[str] = []
    source = finite(values.get("source"))
    command = finite(values.get("command", values.get("trajectory")))
    reference = finite(values.get("reference"))
    output = finite(values.get("output"))
    feedback = finite(values.get("feedback"))
    dxl_present = finite(values.get("dxl_present"))
    if source is not None and command is not None and abs(source - command) > CLAMP_EPS_RAD:
        flags.append("SOURCE_TO_COMMAND")
    if command is not None and reference is not None and abs(command - reference) > CLAMP_EPS_RAD:
        flags.append("COMMAND_TO_REFERENCE")
    if reference is not None and output is not None and abs(reference - output) > CLAMP_EPS_RAD:
        flags.append("CONTROLLER_OUTPUT")

    error = finite(values.get("tracking_error"))
    if error is None and reference is not None and feedback is not None:
        error = reference - feedback
    if values.get("tracking_critical"):
        flags.append("TRACKING_CRITICAL")
    elif values.get("tracking_warning") or (error is not None and abs(error) > tracking_warn_rad):
        flags.append("TRACKING_WARNING")

    if limits:
        hw_low, hw_high = limits["hardware"]
        desired = reference if reference is not None else command
        if desired is not None and not hw_low - CLAMP_EPS_RAD <= desired <= hw_high + CLAMP_EPS_RAD:
            flags.append("OUTSIDE_DXL_RANGE")
        if values.get("hard_limit_saturation"):
            flags.append("HARD_LIMIT_SATURATION")
        elif dxl_present is not None and (
            abs(dxl_present - hw_low) <= CLAMP_EPS_RAD
            or abs(dxl_present - hw_high) <= CLAMP_EPS_RAD
        ):
            flags.append("AT_DXL_LIMIT")
    if values.get("stale"):
        flags.append("STALE_SIGNAL")
    return flags


@dataclass
class ConditionLatch:
    """Debounce a condition and report only open/close transitions."""

    on_delay_s: float
    off_delay_s: float
    active: bool = False
    pending_since: float | None = None

    def update(self, condition: bool, now_s: float) -> str | None:
        target = bool(condition)
        if target == self.active:
            self.pending_since = None
            return None
        if self.pending_since is None:
            self.pending_since = float(now_s)
            return None
        delay = self.on_delay_s if target else self.off_delay_s
        if float(now_s) - self.pending_since < delay:
            return None
        self.active = target
        self.pending_since = None
        return "opened" if target else "closed"
