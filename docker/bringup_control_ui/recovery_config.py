"""Model-specific recovery configuration loaded from bring-up YAML files."""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class RecoveryStep:
    name: str
    positions: tuple[float, ...]


@dataclass(frozen=True)
class RecoveryGroup:
    executor_name: str
    label: str
    action: str
    joints: tuple[str, ...]
    steps: tuple[RecoveryStep, ...]
    duration: float
    position_tolerance: float
    velocity_tolerance: float

    def step_for_phase(self, phase_index: int) -> RecoveryStep:
        """Hold the final YAML step when another group has more phases."""
        return self.steps[min(phase_index, len(self.steps) - 1)]


# Pose values, joint names, action topics, durations, and tolerances intentionally
# do not live here.  They are read from the selected model's YAML file.
RECOVERY_PROFILES: dict[str, dict[str, Any]] = {
    "f2": {
        "config": "ffw_bringup/config/ffw_f2_follower/ffw_f2_follower_initial_positions.yaml",
        "urdf": "ffw_description/urdf/ffw_f2_follower/ffw_f2_follower.urdf",
        "executors": {
            "arm_l_joint_trajectory_executor": "왼팔",
            "arm_r_joint_trajectory_executor": "오른팔",
            "head_joint_trajectory_executor": "헤드",
            "lift_joint_trajectory_executor": "리프트",
        },
    },
}

CUSTOM_TARGET_KEYS = {
    "arm_l_joint_trajectory_executor": "left_arm",
    "arm_r_joint_trajectory_executor": "right_arm",
    "head_joint_trajectory_executor": "head",
    "lift_joint_trajectory_executor": "lift",
}


def default_config_path(model: str) -> Path:
    profile = RECOVERY_PROFILES.get(model)
    if profile is None:
        raise ValueError(f"지원하지 않는 복구 모델입니다: {model}")
    ai_worker_root = Path(__file__).resolve().parents[2]
    return ai_worker_root / str(profile["config"])


def _load_repeated_ros_sections(path: Path) -> dict[str, dict[str, Any]]:
    """Load ROS parameter YAML while preserving repeated top-level '/**:' keys."""
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ValueError(f"복구 YAML을 읽을 수 없습니다: {path}: {exc}") from exc

    section_index = 0

    def unique_section(match: re.Match[str]) -> str:
        nonlocal section_index
        replacement = f"{match.group(1)}__ros_parameter_section_{section_index}:"
        section_index += 1
        return replacement

    normalized = re.sub(r"(?m)^(\s*)/\*\*:\s*$", unique_section, text)
    if section_index == 0:
        raise ValueError(f"복구 YAML에 '/**:' ROS parameter section이 없습니다: {path}")
    try:
        document = yaml.safe_load(normalized)
    except yaml.YAMLError as exc:
        raise ValueError(f"복구 YAML 문법 오류: {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise ValueError(f"복구 YAML 최상위 값이 mapping이 아닙니다: {path}")

    executors: dict[str, dict[str, Any]] = {}
    for section in document.values():
        if not isinstance(section, dict):
            continue
        for executor_name, node_config in section.items():
            if executor_name in executors:
                raise ValueError(f"복구 YAML executor가 중복되었습니다: {executor_name}")
            if not isinstance(node_config, dict):
                continue
            parameters = node_config.get("ros__parameters")
            if isinstance(parameters, dict):
                executors[str(executor_name)] = parameters
    return executors


def _finite_float(value: Any, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} 값이 숫자가 아닙니다: {value!r}") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} 값이 유한수가 아닙니다: {value!r}")
    return result


def load_recovery_groups(model: str, config_path: str | Path | None = None) -> tuple[RecoveryGroup, ...]:
    """Validate and return the recovery groups for ``model`` from its YAML."""
    model = model.strip().lower()
    profile = RECOVERY_PROFILES.get(model)
    if profile is None:
        raise ValueError(f"지원하지 않는 복구 모델입니다: {model}")
    path = Path(config_path) if config_path is not None else default_config_path(model)
    executors = _load_repeated_ros_sections(path)

    groups: list[RecoveryGroup] = []
    for executor_name, label in profile["executors"].items():
        params = executors.get(executor_name)
        if params is None:
            raise ValueError(f"복구 YAML에 executor가 없습니다: {executor_name}")

        joints_raw = params.get("joint_names")
        steps_raw = params.get("step_names")
        action = params.get("action_topic")
        if not isinstance(joints_raw, list) or not joints_raw or not all(isinstance(item, str) and item for item in joints_raw):
            raise ValueError(f"{executor_name}.joint_names가 올바르지 않습니다.")
        if len(set(joints_raw)) != len(joints_raw):
            raise ValueError(f"{executor_name}.joint_names에 중복 joint가 있습니다.")
        if not isinstance(steps_raw, list) or not steps_raw or not all(isinstance(item, str) and item for item in steps_raw):
            raise ValueError(f"{executor_name}.step_names가 올바르지 않습니다.")
        if not isinstance(action, str) or not action.startswith("/"):
            raise ValueError(f"{executor_name}.action_topic이 올바르지 않습니다.")

        steps: list[RecoveryStep] = []
        for step_name in steps_raw:
            positions_raw = params.get(step_name)
            if not isinstance(positions_raw, list) or len(positions_raw) != len(joints_raw):
                raise ValueError(
                    f"{executor_name}.{step_name} 길이가 joint_names({len(joints_raw)})와 일치하지 않습니다."
                )
            positions = tuple(
                _finite_float(value, f"{executor_name}.{step_name}[{index}]")
                for index, value in enumerate(positions_raw)
            )
            steps.append(RecoveryStep(step_name, positions))

        duration = _finite_float(params.get("duration"), f"{executor_name}.duration")
        position_tolerance = _finite_float(
            params.get("position_tolerance"), f"{executor_name}.position_tolerance"
        )
        velocity_tolerance = _finite_float(
            params.get("velocity_tolerance"), f"{executor_name}.velocity_tolerance"
        )
        if duration <= 0:
            raise ValueError(f"{executor_name}.duration은 0보다 커야 합니다.")
        if position_tolerance < 0 or velocity_tolerance < 0:
            raise ValueError(f"{executor_name} tolerance는 0 이상이어야 합니다.")

        groups.append(
            RecoveryGroup(
                executor_name=executor_name,
                label=str(label),
                action=action,
                joints=tuple(joints_raw),
                steps=tuple(steps),
                duration=duration,
                position_tolerance=position_tolerance,
                velocity_tolerance=velocity_tolerance,
            )
        )
    return tuple(groups)


def _joint_limits(model: str, joints: set[str]) -> dict[str, tuple[float, float]]:
    profile = RECOVERY_PROFILES.get(model)
    if profile is None:
        raise ValueError(f"지원하지 않는 복구 모델입니다: {model}")
    urdf_path = Path(__file__).resolve().parents[2] / str(profile["urdf"])
    try:
        root = ET.parse(urdf_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ValueError(f"URDF joint limit을 읽을 수 없습니다: {urdf_path}: {exc}") from exc

    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = str(joint.get("name") or "")
        if name not in joints:
            continue
        limit = joint.find("limit")
        if limit is None or limit.get("lower") is None or limit.get("upper") is None:
            raise ValueError(f"{name}의 URDF joint limit이 없습니다.")
        lower = _finite_float(limit.get("lower"), f"{name}.lower")
        upper = _finite_float(limit.get("upper"), f"{name}.upper")
        if lower > upper:
            raise ValueError(f"{name}의 URDF joint limit 순서가 잘못되었습니다.")
        limits[name] = (lower, upper)

    missing = sorted(joints - set(limits))
    if missing:
        raise ValueError(f"URDF joint limit을 찾을 수 없습니다: {', '.join(missing)}")
    return limits


def load_custom_recovery_groups(
    model: str,
    targets: dict[str, Any],
    *,
    duration: Any = 8.0,
    config_path: str | Path | None = None,
) -> tuple[RecoveryGroup, ...]:
    """Build one validated whole-body target from operator-entered lists."""
    model = model.strip().lower()
    groups = load_recovery_groups(model, config_path)
    if not isinstance(targets, dict):
        raise ValueError("사용자 초기 위치는 group별 list 형식이어야 합니다.")

    move_duration = _finite_float(duration, "duration")
    if not 5.0 <= move_duration <= 30.0:
        raise ValueError("duration은 안전을 위해 5초 이상 30초 이하여야 합니다.")

    required_keys = {
        CUSTOM_TARGET_KEYS[group.executor_name]
        for group in groups
    }
    missing_keys = sorted(required_keys - set(targets))
    if missing_keys:
        raise ValueError(f"초기 위치 list가 누락되었습니다: {', '.join(missing_keys)}")

    all_joints = {joint for group in groups for joint in group.joints}
    limits = _joint_limits(model, all_joints)
    custom_groups: list[RecoveryGroup] = []
    for group in groups:
        key = CUSTOM_TARGET_KEYS[group.executor_name]
        raw = targets.get(key)
        if not isinstance(raw, list) or len(raw) != len(group.joints):
            raise ValueError(
                f"{key}는 {len(group.joints)}개 숫자 list여야 합니다."
            )
        positions = tuple(
            _finite_float(value, f"{key}[{index}]")
            for index, value in enumerate(raw)
        )
        for joint, value in zip(group.joints, positions):
            lower, upper = limits[joint]
            if value < lower or value > upper:
                raise ValueError(
                    f"{joint}={value:.6g}가 허용 범위 [{lower:.6g}, {upper:.6g}]를 벗어났습니다."
                )
        custom_groups.append(replace(
            group,
            steps=(RecoveryStep("custom", positions),),
            duration=move_duration,
        ))
    return tuple(custom_groups)
