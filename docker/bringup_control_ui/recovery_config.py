"""Model-specific recovery configuration loaded from bring-up YAML files."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
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
        "executors": {
            "arm_l_joint_trajectory_executor": "왼팔",
            "arm_r_joint_trajectory_executor": "오른팔",
            "head_joint_trajectory_executor": "헤드",
            "lift_joint_trajectory_executor": "리프트",
        },
    },
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
