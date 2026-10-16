#!/usr/bin/env python3
"""Local web UI bridge for waypoint_telep."""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time
from typing import Any
from urllib.error import HTTPError
from urllib.error import URLError
from urllib.parse import parse_qs
from urllib.parse import unquote
from urllib.parse import urlsplit
from urllib.request import Request
from urllib.request import urlopen
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import QuaternionStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from std_msgs.msg import UInt32
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


ACTION_NAMES = {
    'capture_current',
    'remove_last',
    'clear',
    'select_linear',
    'select_joint',
    'preview',
    'execute_slow_start',
    'prepare_teleop',
    'select_teleop',
    'execute',
    'stop',
}
STOP_BUTTON_CONFIRMATION = 'STOP_BUTTON_CLICK'

ARMS = ('left', 'right')
MAX_TOOL_PATH_CORRECTION_MM = 50.0
CARTESIAN_JOG_DIRECTIONS = {
    'front': (1.0, 0.0, 0.0),
    'back': (-1.0, 0.0, 0.0),
    'left': (0.0, 1.0, 0.0),
    'right': (0.0, -1.0, 0.0),
    'up': (0.0, 0.0, 1.0),
    'down': (0.0, 0.0, -1.0),
}
F2_POSITION_LIMITS = {
    'left': [
        (-3.14, 3.14), (0.0, 3.14), (-3.14, 3.14),
        (-2.9361, 1.0785802414820915),
        (-3.14, 3.14), (-1.57, 1.57),
        (-1.8201, 1.5804), (0.0, 1.1),
    ],
    'right': [
        (-3.14, 3.14), (-3.14, 0.0), (-3.14, 3.14),
        (-2.9361, 1.0785802414820915),
        (-3.14, 3.14), (-1.57, 1.57),
        (-1.5804, 1.8201), (0.0, 1.1),
    ],
}


def arm_joint_names(arm: str) -> list[str]:
    if arm not in {'left', 'right'}:
        raise ValueError("arm must be 'left' or 'right'")
    prefix = 'arm_l_joint' if arm == 'left' else 'arm_r_joint'
    return [f'{prefix}{index}' for index in range(1, 8)]


def command_joint_names(arm: str) -> list[str]:
    gripper = 'gripper_l_joint1' if arm == 'left' else 'gripper_r_joint1'
    return arm_joint_names(arm) + [gripper]


def parse_joint_values(payload: dict[str, Any], count: int) -> list[float]:
    has_degrees = 'positions_deg' in payload
    has_radians = 'positions_rad' in payload
    if has_degrees == has_radians:
        raise ValueError('관절값 단위는 degree 또는 radian 중 하나만 지정해 주세요.')
    key = 'positions_rad' if has_radians else 'positions_deg'
    values = payload.get(key)
    if not isinstance(values, list) or len(values) != count:
        raise ValueError(f'관절값 {count}개를 모두 입력해 주세요.')
    positions = []
    for value in values:
        if isinstance(value, bool):
            raise ValueError('관절값은 유한한 숫자여야 합니다.')
        try:
            numeric = float(value)
        except (TypeError, ValueError) as exception:
            raise ValueError('관절값은 유한한 숫자여야 합니다.') from exception
        if not math.isfinite(numeric):
            raise ValueError('관절값은 유한한 숫자여야 합니다.')
        positions.append(numeric if has_radians else math.radians(numeric))
    return positions


def parse_waypoint_index(payload: dict[str, Any]) -> int:
    value = payload.get('index')
    if isinstance(value, bool):
        raise ValueError('waypoint 번호는 1 이상의 정수여야 합니다.')
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exception:
        raise ValueError('waypoint 번호는 1 이상의 정수여야 합니다.') from exception
    if not math.isfinite(numeric) or numeric < 1.0 or not numeric.is_integer():
        raise ValueError('waypoint 번호는 1 이상의 정수여야 합니다.')
    return int(numeric)


def parse_waypoint_mode(payload: dict[str, Any]) -> tuple[str, int]:
    value = str(payload.get('mode', '')).strip().upper()
    if value == 'LINEAR':
        return value, 0
    if value == 'JOINT':
        return value, 1
    raise ValueError('보간 방식은 LINEAR 또는 JOINT여야 합니다.')


def parse_waypoint_name(payload: dict[str, Any]) -> str:
    value = payload.get('name')
    if not isinstance(value, str):
        raise ValueError('waypoint 이름을 문자열로 입력해 주세요.')
    name = ' '.join(value.strip().split())
    if not name:
        raise ValueError('waypoint 이름을 입력해 주세요.')
    if len(name) > 60:
        raise ValueError('waypoint 이름은 60자 이하여야 합니다.')
    return name


def parse_reorder_indices(payload: dict[str, Any]) -> tuple[int, int]:
    try:
        source = parse_waypoint_index({'index': payload.get('from_index')})
        target = parse_waypoint_index({'index': payload.get('to_index')})
    except ValueError as exception:
        raise ValueError('이동 전·후 waypoint 번호가 올바르지 않습니다.') from exception
    return source, target


def parse_execution_step(payload: dict[str, Any]) -> dict[str, Any]:
    arm = str(payload.get('arm', ''))
    if arm not in ARMS:
        raise ValueError('실행 구간의 팔은 left 또는 right여야 합니다.')
    start = parse_waypoint_index({'index': payload.get('start_index')})
    end = parse_waypoint_index({'index': payload.get('end_index')})
    return {'arm': arm, 'start_index': start, 'end_index': end}


def parse_tool_path_correction(
    payload: dict[str, Any], waypoint_count: int
) -> dict[str, Any]:
    full_path_correction = payload.get('full_path_correction', False)
    if not isinstance(full_path_correction, bool):
        raise ValueError('전체 경로 보정 옵션은 true 또는 false여야 합니다.')
    if full_path_correction:
        grasp_complete = parse_waypoint_index({
            'index': payload.get('grasp_complete_waypoint')})
        approach_start = 1
        grasp = 1
        reference_grasp = grasp_complete
    else:
        approach_start = parse_waypoint_index({
            'index': payload.get('approach_start_waypoint')})
        grasp = parse_waypoint_index({'index': payload.get('grasp_waypoint')})
        grasp_complete = parse_waypoint_index({
            'index': payload.get('grasp_complete_waypoint', grasp)})
        reference_grasp = grasp
        if approach_start > grasp:
            raise ValueError(
                '접근 시작 waypoint는 그리퍼 닫기 시작점보다 클 수 없습니다.')
        if grasp > grasp_complete:
            raise ValueError(
                '파지 완료 waypoint는 그리퍼 닫기 시작점보다 작을 수 없습니다.')
    if grasp_complete > waypoint_count:
        raise ValueError('파지 구간이 현재 경로 범위를 벗어났습니다.')
    raw_offset = payload.get('local_offset_mm')
    if not isinstance(raw_offset, list) or len(raw_offset) != 3:
        raise ValueError('로컬 보정량은 X, Y, Z 3개를 모두 입력해 주세요.')
    offset_mm = []
    for value in raw_offset:
        if isinstance(value, bool):
            raise ValueError('로컬 보정량은 유한한 숫자여야 합니다.')
        try:
            numeric = float(value)
        except (TypeError, ValueError) as exception:
            raise ValueError('로컬 보정량은 유한한 숫자여야 합니다.') from exception
        if not math.isfinite(numeric):
            raise ValueError('로컬 보정량은 유한한 숫자여야 합니다.')
        offset_mm.append(numeric)
    norm_mm = math.sqrt(sum(value * value for value in offset_mm))
    if norm_mm > MAX_TOOL_PATH_CORRECTION_MM:
        raise ValueError(
            f'보정량의 합성 크기는 {MAX_TOOL_PATH_CORRECTION_MM:g} mm 이하여야 합니다.')
    return {
        'approach_start_waypoint': approach_start,
        'grasp_waypoint': grasp,
        'reference_grasp_waypoint': reference_grasp,
        'grasp_complete_waypoint': grasp_complete,
        'full_path_correction': full_path_correction,
        'local_offset_mm': offset_mm,
        'norm_mm': norm_mm,
    }


def parse_relative_reference(payload: dict[str, Any]) -> int:
    value = payload.get('reference', 'last')
    if value in (None, '', 'last'):
        return -1
    if value == 'current':
        return 0
    if isinstance(value, bool):
        raise ValueError('상대 이동 계산 기준이 올바르지 않습니다.')
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exception:
        raise ValueError('상대 이동 계산 기준이 올바르지 않습니다.') from exception
    if not math.isfinite(numeric) or numeric < 1.0 or not numeric.is_integer():
        raise ValueError('기준 waypoint 번호는 1 이상의 정수여야 합니다.')
    return int(numeric)


def parse_dwell_seconds(payload: dict[str, Any], maximum: float) -> float:
    value = payload.get('dwell_s', 0.0)
    if isinstance(value, bool):
        raise ValueError('대기 시간은 유한한 숫자여야 합니다.')
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exception:
        raise ValueError('대기 시간은 유한한 숫자여야 합니다.') from exception
    if not math.isfinite(numeric) or numeric < 0.0 or numeric > maximum:
        raise ValueError(f'대기 시간은 0초 이상 {maximum:g}초 이하여야 합니다.')
    return numeric


def parse_speed_scale(
    payload: dict[str, Any], maximum: float = 3.0
) -> float:
    legacy_percent = False
    if 'speed_scale' in payload:
        value = payload['speed_scale']
    elif 'speed_percent' in payload:
        value = payload['speed_percent']
        legacy_percent = True
    else:
        value = 1.0
    if isinstance(value, bool):
        raise ValueError('동작 속도는 유한한 숫자여야 합니다.')
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exception:
        raise ValueError('동작 속도는 유한한 숫자여야 합니다.') from exception
    if legacy_percent:
        numeric /= 100.0
    if not math.isfinite(numeric) or numeric < 0.01 or numeric > maximum:
        raise ValueError(
            f'동작 속도 배율은 0.01 이상 {maximum:g} 이하여야 합니다.')
    return numeric


def smooth_joint_transition(
    start: list[float], target: list[float], maximum_step: float
) -> list[list[float]]:
    """Create a quintic transition whose adjacent joint steps stay bounded."""
    if len(start) != len(target) or not start:
        raise ValueError('관절 전환 시작값과 목표값의 크기가 올바르지 않습니다.')
    if not math.isfinite(maximum_step) or maximum_step <= 0.0:
        raise ValueError('관절 전환 간격 제한이 올바르지 않습니다.')
    if any(not math.isfinite(value) for value in [*start, *target]):
        raise ValueError('관절 전환값은 유한한 숫자여야 합니다.')
    maximum_delta = max(abs(goal - current) for current, goal in zip(start, target))
    if maximum_delta <= maximum_step:
        return []

    step_count = max(2, math.ceil(maximum_delta / maximum_step))
    while True:
        points = []
        for step in range(1, step_count + 1):
            progress = step / step_count
            progress3 = progress * progress * progress
            blend = progress3 * (
                10.0 - 15.0 * progress + 6.0 * progress * progress)
            points.append([
                current + blend * (goal - current)
                for current, goal in zip(start, target)
            ])
        if all(
            max(abs(value - old) for value, old in zip(point, previous))
            <= maximum_step + 1.0e-12
            for previous, point in zip([start, *points[:-1]], points)
        ):
            return points
        step_count += 1


def parse_relative_pose(payload: dict[str, Any]) -> tuple[list[float], list[float]]:
    """Parse a gripper-local XYZ (mm) and RPY (degree) delta."""
    groups = (
        ('translation_mm', '상대 이동값', 0.001),
        ('rotation_deg', '상대 회전값', math.pi / 180.0),
    )
    parsed = []
    for key, label, scale in groups:
        values = payload.get(key)
        if not isinstance(values, list) or len(values) != 3:
            raise ValueError(f'{label} 3개를 모두 입력해 주세요.')
        converted = []
        for value in values:
            if isinstance(value, bool):
                raise ValueError(f'{label}은 유한한 숫자여야 합니다.')
            try:
                numeric = float(value)
            except (TypeError, ValueError) as exception:
                raise ValueError(f'{label}은 유한한 숫자여야 합니다.') from exception
            if not math.isfinite(numeric):
                raise ValueError(f'{label}은 유한한 숫자여야 합니다.')
            converted.append(numeric * scale)
        parsed.append(converted)
    return parsed[0], parsed[1]


def parse_cartesian_jog(
    payload: dict[str, Any], maximum_speed_mmps: float, period_s: float
) -> tuple[str, list[float], float]:
    direction = str(payload.get('direction', '')).strip().lower()
    if direction not in CARTESIAN_JOG_DIRECTIONS:
        raise ValueError('이동 방향은 앞·뒤·왼쪽·오른쪽·위·아래 중 하나여야 합니다.')
    value = payload.get('speed_mmps', 5.0)
    if isinstance(value, bool):
        raise ValueError('키보드 이동 속도는 유한한 숫자여야 합니다.')
    try:
        speed_mmps = float(value)
    except (TypeError, ValueError) as exception:
        raise ValueError('키보드 이동 속도는 유한한 숫자여야 합니다.') from exception
    if (
        not math.isfinite(speed_mmps)
        or speed_mmps <= 0.0
        or speed_mmps > maximum_speed_mmps
    ):
        raise ValueError(
            f'키보드 이동 속도는 0보다 크고 {maximum_speed_mmps:g} mm/s 이하여야 합니다.'
        )
    distance_m = speed_mmps * period_s * 0.001
    vector = [component * distance_m for component in CARTESIAN_JOG_DIRECTIONS[direction]]
    return direction, vector, speed_mmps


def _parse_payload_vector(
    payload: dict[str, Any], key: str, label: str, scale: float
) -> list[float]:
    values = payload.get(key)
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f'{label} 3개를 모두 입력해 주세요.')
    converted = []
    for value in values:
        if isinstance(value, bool):
            raise ValueError(f'{label}은 유한한 숫자여야 합니다.')
        try:
            numeric = float(value)
        except (TypeError, ValueError) as exception:
            raise ValueError(f'{label}은 유한한 숫자여야 합니다.') from exception
        if not math.isfinite(numeric):
            raise ValueError(f'{label}은 유한한 숫자여야 합니다.')
        converted.append(numeric * scale)
    return converted


def parse_absolute_target(
    payload: dict[str, Any]
) -> tuple[str, list[float] | None, list[float] | None]:
    """Parse an absolute target and only require the selected components."""
    mode = payload.get('target_mode', 'pose')
    if mode not in {'position', 'orientation', 'pose'}:
        raise ValueError('절대 목표 적용 범위가 올바르지 않습니다.')
    position = None
    rotation = None
    if mode in {'position', 'pose'}:
        position = _parse_payload_vector(payload, 'position_m', '절대 위치', 1.0)
    if mode in {'orientation', 'pose'}:
        rotation = _parse_payload_vector(
            payload, 'orientation_deg', '절대 회전값', math.pi / 180.0)
    return mode, position, rotation


def parse_absolute_pose(payload: dict[str, Any]) -> tuple[list[float], list[float]]:
    """Parse a complete world-frame XYZ/RPY pose for compatibility."""
    complete_payload = dict(payload)
    complete_payload['target_mode'] = 'pose'
    _, position, rotation = parse_absolute_target(complete_payload)
    assert position is not None and rotation is not None
    return position, rotation


def rpy_to_quaternion(rpy: list[float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def browser_urls(host: str, port: int, public_host: str = '') -> list[str]:
    """Return browser-usable URLs instead of an unspecified bind address."""
    if public_host.strip():
        return [f'http://{public_host.strip()}:{port}']
    if host not in {'0.0.0.0', '::', ''}:
        display_host = '127.0.0.1' if host == 'localhost' else host
        return [f'http://{display_host}:{port}']

    addresses = {'127.0.0.1'}
    try:
        for result in socket.getaddrinfo(
            socket.gethostname(), None, family=socket.AF_INET
        ):
            address = result[4][0]
            if address and address != '0.0.0.0':
                addresses.add(address)
    except socket.gaierror:
        pass
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(('192.0.2.1', 80))
            addresses.add(probe.getsockname()[0])
        finally:
            probe.close()
    except OSError:
        pass

    ordered = sorted(
        addresses,
        key=lambda address: (address.startswith('127.'), address),
    )
    return [f'http://{address}:{port}' for address in ordered]


def _parse_vector(text: str | None, default: tuple[float, float, float]) -> list[float]:
    if text is None or not text.strip():
        return list(default)
    try:
        values = [float(value) for value in text.split()]
    except ValueError as exception:
        raise ValueError(f'URDF vector is not numeric: {text}') from exception
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f'URDF vector must contain three finite values: {text}')
    return values


def parse_urdf_chain(
    robot_description: str, base_link: str, tip_link: str
) -> dict[str, Any]:
    """Extract the serial kinematic chain needed by the browser simulator."""
    try:
        root = ElementTree.fromstring(robot_description)
    except ElementTree.ParseError as exception:
        raise ValueError('failed to parse /robot_description XML') from exception
    if root.tag != 'robot':
        raise ValueError('/robot_description root element must be <robot>')

    joints_by_child: dict[str, dict[str, Any]] = {}
    for element in root.findall('joint'):
        parent = element.find('parent')
        child = element.find('child')
        if parent is None or child is None:
            continue
        parent_link = parent.get('link', '')
        child_link = child.get('link', '')
        if not parent_link or not child_link:
            continue
        origin = element.find('origin')
        axis = element.find('axis')
        limit = element.find('limit')
        joint_type = element.get('type', 'fixed')
        lower = None
        upper = None
        if joint_type == 'continuous':
            lower, upper = -math.pi, math.pi
        elif limit is not None:
            try:
                lower = float(limit.get('lower')) if limit.get('lower') else None
                upper = float(limit.get('upper')) if limit.get('upper') else None
            except ValueError as exception:
                raise ValueError(
                    f'URDF joint limit is not numeric: {element.get("name", "")}'
                ) from exception
        joint = {
            'name': element.get('name', ''),
            'type': joint_type,
            'parent': parent_link,
            'child': child_link,
            'origin_xyz': _parse_vector(
                origin.get('xyz') if origin is not None else None,
                (0.0, 0.0, 0.0),
            ),
            'origin_rpy': _parse_vector(
                origin.get('rpy') if origin is not None else None,
                (0.0, 0.0, 0.0),
            ),
            'axis': _parse_vector(
                axis.get('xyz') if axis is not None else None,
                (1.0, 0.0, 0.0),
            ),
            'limit_lower': lower,
            'limit_upper': upper,
        }
        if child_link in joints_by_child:
            raise ValueError(f'URDF link has multiple parents: {child_link}')
        joints_by_child[child_link] = joint

    chain = []
    current_link = tip_link
    visited = set()
    while current_link != base_link:
        if current_link in visited:
            raise ValueError(f'URDF chain contains a cycle at {current_link}')
        visited.add(current_link)
        joint = joints_by_child.get(current_link)
        if joint is None:
            raise ValueError(f'URDF chain not found: {base_link} -> {tip_link}')
        chain.append(joint)
        current_link = joint['parent']
    chain.reverse()

    movable_types = {'continuous', 'revolute', 'prismatic'}
    movable_joint_names = [
        joint['name'] for joint in chain if joint['type'] in movable_types
    ]
    if not movable_joint_names:
        raise ValueError(f'URDF chain has no movable joints: {base_link} -> {tip_link}')
    return {
        'status': 'ready',
        'robot_name': root.get('name', 'F2'),
        'base_link': base_link,
        'tip_link': tip_link,
        'movable_joint_names': movable_joint_names,
        'chain': chain,
    }


def parse_equal_segment_time(payload: dict[str, Any], count: int) -> tuple[float, list[int]]:
    total_mode = 'total_duration_s' in payload
    if total_mode and 'duration_s' in payload:
        raise ValueError('전체 목표시간과 구간당 시간 중 하나만 지정해 주세요.')
    seconds = payload.get('total_duration_s' if total_mode else 'duration_s', 0)
    if (isinstance(seconds, bool) or not isinstance(seconds, (int, float))
            or not math.isfinite(seconds) or seconds < 0 or (total_mode and seconds == 0)):
        raise ValueError('전체 목표시간은 0보다 큰 숫자여야 합니다. 구간당 시간의 0은 자동 계산입니다.')
    if count < 2:
        raise ValueError('시간을 배정하려면 waypoint가 두 개 이상 필요합니다.')
    excluded = payload.get('excluded_waypoints', [])
    if (not isinstance(excluded, list) or any(
            isinstance(index, bool) or not isinstance(index, int)
            or not 2 <= index <= count for index in excluded)):
        raise ValueError(f'제외할 도착 waypoint 번호는 2–{count} 범위의 정수여야 합니다.')
    excluded = sorted(set(excluded))
    if len(excluded) >= count - 1:
        raise ValueError('제외 후 시간을 배정할 구간이 하나 이상 남아야 합니다.')
    return float(seconds), excluded


def parse_joint_timing(value: Any, count: int) -> dict[str, Any] | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError('joint_timing은 객체여야 합니다.')
    speed = value.get('speed_mps')
    start = value.get('preserve_start_waypoint')
    end = value.get('preserve_end_waypoint')
    if (isinstance(speed, bool) or not isinstance(speed, (int, float))
            or not math.isfinite(speed) or not 0 < speed <= 0.1
            or isinstance(start, bool) or not isinstance(start, int)
            or isinstance(end, bool) or not isinstance(end, int)
            or not 2 <= start <= end <= count):
        raise ValueError('등속 설정의 속도 또는 삽입 보존 구간이 올바르지 않습니다.')
    return dict(speed_mps=float(speed), preserve_start_waypoint=start,
                preserve_end_waypoint=end)


def parse_path_refinement(value: Any) -> dict[str, Any]:
    defaults = dict(enabled=False, subdivisions=10, smoothing_percent=100.0,
                    position_tolerance_mm=1.0, orientation_tolerance_deg=1.0,
                    joint_tolerance_rad=0.01, max_refinements=5)
    if value is None:
        return defaults
    if not isinstance(value, dict):
        raise ValueError('보간 설정은 객체여야 합니다.')
    result = {**defaults, **value}
    if not isinstance(result['enabled'], bool):
        raise ValueError('보간 사용 여부는 true 또는 false여야 합니다.')
    limits = dict(subdivisions=(1, 256), smoothing_percent=(0, 100),
                  position_tolerance_mm=(0, 100), orientation_tolerance_deg=(0, 180),
                  joint_tolerance_rad=(0, 1), max_refinements=(0, 8))
    for key, (lower, upper) in limits.items():
        number = result[key]
        if (isinstance(number, bool) or not isinstance(number, (int, float))
                or not math.isfinite(number) or number < lower or number > upper
                or ('tolerance' in key and number == 0)
                or (key in ('subdivisions', 'max_refinements') and int(number) != number)):
            raise ValueError(f'{key}: 허용 범위는 {lower}~{upper}입니다. 오차는 0보다 커야 합니다.')
    return {key: result[key] for key in defaults}


class WaypointUiBridge(Node):

    def __init__(self) -> None:
        super().__init__('waypoint_telep_ui')
        configured_arm = str(self.declare_parameter('arm', 'left').value)
        waypoint_namespace = str(
            self.declare_parameter('waypoint_namespace', '/waypoint_telep').value
        ).rstrip('/')
        if configured_arm not in {'left', 'right', 'both'}:
            raise ValueError("arm must be 'left', 'right', or 'both'")
        self.available_arms = (
            ['left', 'right'] if configured_arm == 'both' else [configured_arm]
        )
        self.default_arm = self.available_arms[0]
        self.host = self.declare_parameter('host', '127.0.0.1').value
        self.port = int(self.declare_parameter('port', 8765).value)
        self.public_host = str(
            self.declare_parameter('public_host', '').value)
        self.publish_enabled = bool(
            self.declare_parameter('publish_enabled', False).value)
        self.command_arbiter_namespace = str(self.declare_parameter(
            'command_arbiter_namespace', '/command_source_arbiter').value
        ).rstrip('/')
        controller_url = str(self.declare_parameter(
            'intervention_controller_url', 'http://127.0.0.1:8779').value
        ).rstrip('/')
        self.intervention_controller_url = (
            '' if controller_url.lower() in {'', 'disabled', 'none'}
            else controller_url
        )
        self.intervention_controller_token = str(self.declare_parameter(
            'intervention_controller_token', '').value)
        self.intervention_controller_timeout_s = float(self.declare_parameter(
            'intervention_controller_timeout_s', 35.0).value)
        self.teleop_action_topics = {
            'left': str(self.declare_parameter(
                'teleop_action_left_topic', '/intervention/teleop/arm_left').value),
            'right': str(self.declare_parameter(
                'teleop_action_right_topic', '/intervention/teleop/arm_right').value),
        }
        self.final_action_topics = {
            'left': str(self.declare_parameter(
                'final_action_left_topic',
                '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
            ).value),
            'right': str(self.declare_parameter(
                'final_action_right_topic',
                '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
            ).value),
        }
        self.max_start_velocity_radps = float(self.declare_parameter(
            'max_start_velocity_radps', 0.02).value)
        self.max_joint_step_rad = float(self.declare_parameter(
            'max_joint_step_rad', 0.05).value)
        self.control_rate_hz = float(self.declare_parameter(
            'control_rate_hz', 100.0).value)
        self.handoff_stop_timeout_s = float(self.declare_parameter(
            'handoff_stop_timeout_s', 2.0).value)
        self.completion_timeout_s = float(self.declare_parameter(
            'completion_timeout_s', 2.0).value)
        self.completion_tolerance_rad = float(self.declare_parameter(
            'completion_tolerance_rad', 0.05).value)
        self.max_schedule_lag_s = float(self.declare_parameter(
            'max_schedule_lag_s', 0.05).value)
        self.max_waypoint_dwell_s = float(self.declare_parameter(
            'max_waypoint_dwell_s', 30.0).value)
        self.max_waypoint_speed_scale = float(self.declare_parameter(
            'max_waypoint_speed_scale', 3.0).value)
        self.keyboard_jog_period_s = float(self.declare_parameter(
            'keyboard_jog_period_s', 0.05).value)
        self.keyboard_jog_max_speed_mmps = float(self.declare_parameter(
            'keyboard_jog_max_speed_mmps', 50.0).value)
        self.keyboard_deadman_timeout_s = float(self.declare_parameter(
            'keyboard_deadman_timeout_s', 1.00).value)
        self.teleop_ready_timeout_s = float(self.declare_parameter(
            'teleop_ready_timeout_s', 10.0).value)
        self.base_link = str(
            self.declare_parameter('base_link', 'arm_base_link').value)
        self.world_frame = str(
            self.declare_parameter('world_frame', 'world').value)
        configured_tip = str(
            self.declare_parameter('tip_link', '').value)
        configured_path_mode = str(
            self.declare_parameter('path_mode', 'linear').value
        ).upper()
        if configured_path_mode not in {'LINEAR', 'JOINT'}:
            raise ValueError("path_mode must be 'linear' or 'joint'")
        if (
            self.max_start_velocity_radps < 0.0
            or self.max_joint_step_rad <= 0.0
            or not math.isfinite(self.control_rate_hz)
            or self.control_rate_hz <= 0.0
            or self.handoff_stop_timeout_s <= 0.0
            or self.completion_timeout_s <= 0.0
            or self.completion_tolerance_rad <= 0.0
            or self.max_schedule_lag_s <= 0.0
            or self.max_waypoint_dwell_s <= 0.0
            or not math.isfinite(self.max_waypoint_speed_scale)
            or self.max_waypoint_speed_scale < 1.0
            or self.keyboard_jog_period_s <= 0.0
            or self.keyboard_jog_max_speed_mmps <= 0.0
            or self.keyboard_deadman_timeout_s <= self.keyboard_jog_period_s * 2.0
            or not math.isfinite(self.teleop_ready_timeout_s)
            or self.teleop_ready_timeout_s <= 0.0
            or not math.isfinite(self.intervention_controller_timeout_s)
            or self.intervention_controller_timeout_s <= 0.0
        ):
            raise ValueError('waypoint 실행 안전 파라미터가 올바르지 않습니다.')
        self.lock = threading.RLock()
        self.all_current_positions: dict[str, float] = {}
        self.all_current_velocities: dict[str, float] = {}
        self.joint_state_stamp = 0.0
        self.previous_action_positions: dict[str, dict[str, list[float] | None]] = {
            source: {arm: None for arm in ARMS}
            for source in ('final', 'teleop')
        }
        self.arm_configs: dict[str, dict[str, Any]] = {}
        self.arm_states: dict[str, dict[str, Any]] = {}
        for selected_arm in self.available_arms:
            default_tip = (
                'end_effector_l_link'
                if selected_arm == 'left' else 'end_effector_r_link'
            )
            tip_link = (
                configured_tip
                if len(self.available_arms) == 1 and configured_tip else default_tip
            )
            namespace = (
                f'{waypoint_namespace}/{selected_arm}'
                if configured_arm == 'both' else waypoint_namespace
            )
            self.arm_configs[selected_arm] = {
                'namespace': namespace,
                'tip_link': tip_link,
                'arm_joint_names': arm_joint_names(selected_arm),
                'joint_names': command_joint_names(selected_arm),
            }
            self.arm_states[selected_arm] = {
                'current_positions': {},
                'current_stamp': 0.0,
                'status': 'UI 연결 대기 중',
                'status_sequence': 0,
                'execution_events': [],
                'plan_revision': 0,
                'path_mode': configured_path_mode,
                'joint_smoothing_enabled': False,
                'path_refinement': parse_path_refinement(None),
                'path_refinement_events': [],
                'joint_timing': None,
                'waypoint_count': 0,
                'waypoints': [],
                'current_world_pose': None,
                'world_root_pose': None,
                'world_waypoints': [],
                'waypoint_dwells': [],
                'waypoint_speed_scales': [],
                'speed_scales_sequence': 0,
                'waypoint_grippers': [],
                'waypoint_modes': [],
                'waypoint_joint_values': [],
                'waypoints_update_sequence': 0,
                'joint_values_update_sequence': 0,
                'start_waypoint': 1,
                'end_waypoint': 1,
                'waypoint_names': [],
                'waypoint_targets': [],
                'trajectory': None,
                'trajectory_sequence': 0,
                'robot_model': {
                    'status': 'waiting',
                    'message': '/robot_description 대기 중',
                    'base_link': self.base_link,
                    'tip_link': tip_link,
                },
            }
        self.robot_description_xml = ''
        self.service_clients: dict[tuple[str, str], Any] = {}
        self.arbiter_clients: dict[str, Any] = {}
        self.service_lock = threading.Lock()
        self.planner_locks = {arm: threading.Lock() for arm in self.available_arms}
        self.joint_waypoint_publishers: dict[str, Any] = {}
        self.absolute_waypoint_publishers: dict[str, Any] = {}
        self.absolute_position_publishers: dict[str, Any] = {}
        self.absolute_orientation_publishers: dict[str, Any] = {}
        self.relative_waypoint_publishers: dict[str, Any] = {}
        self.frame_relative_waypoint_publishers: dict[str, Any] = {}
        self.relative_reference_publishers: dict[str, Any] = {}
        self.cartesian_jog_publishers: dict[str, Any] = {}
        self.waypoint_dwell_publishers: dict[str, Any] = {}
        self.waypoint_speed_scale_publishers: dict[str, Any] = {}
        self.equal_segment_time_publishers: dict[str, Any] = {}
        self.equal_total_time_publishers: dict[str, Any] = {}
        self.waypoint_gripper_publishers: dict[str, Any] = {}
        self.waypoint_mode_publishers: dict[str, Any] = {}
        self.joint_smoothing_publishers: dict[str, Any] = {}
        self.path_refinement_publishers: dict[str, Any] = {}
        self.joint_timing_publishers: dict[str, Any] = {}
        self.start_waypoint_publishers: dict[str, Any] = {}
        self.end_waypoint_publishers: dict[str, Any] = {}
        self.reorder_waypoint_publishers: dict[str, Any] = {}
        self.remove_waypoint_publishers: dict[str, Any] = {}
        self.tool_path_correction_publishers: dict[str, Any] = {}
        self.waypoint_command_publishers = {
            arm: self.create_publisher(
                JointTrajectory, f'/intervention/waypoint/arm_{arm}', 10)
            for arm in ARMS
        }
        self.execution_lock = threading.Lock()
        self.execution_thread: threading.Thread | None = None
        self.execution_stop = threading.Event()
        self.execution_arm: str | None = None
        self.isolated_streaming_enabled = True
        self.execution_sequence: list[dict[str, Any]] = []
        self.arbiter_mode = 'UNKNOWN'
        self.arbiter_fault = ''
        self.arbiter_intervention_enabled = False
        self.arbiter_teleop_ready = False
        self.arbiter_teleop_reason = '상태를 아직 받지 못했습니다.'
        self.keyboard_condition = threading.Condition(self.lock)
        self.keyboard_jog_lock = threading.Lock()
        self.keyboard_jog_targets = {
            arm: {'sequence': 0, 'positions': None} for arm in self.available_arms
        }
        self.keyboard_active = False
        self.keyboard_hold: dict[str, list[float]] | None = None
        self.keyboard_last_heartbeat = 0.0
        self.keyboard_shutdown = threading.Event()

        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # UI feedback must not back-pressure controller/arbiter publishers.
        # Keep only the latest sample; command freshness/limit checks remain.
        feedback = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, feedback)
        self.create_subscription(
            String,
            '/robot_description',
            self._robot_description_callback,
            transient,
        )
        self.create_subscription(
            String,
            f'{self.command_arbiter_namespace}/status',
            self._arbiter_status_callback,
            transient,
        )
        for action_source, topics in (
            ('final', self.final_action_topics),
            ('teleop', self.teleop_action_topics),
        ):
            for selected_arm in ARMS:
                self.create_subscription(
                    JointTrajectory,
                    topics[selected_arm],
                    lambda message, source=action_source, arm=selected_arm:
                    self._previous_action_callback(source, arm, message),
                    feedback,
                )
        for selected_arm, config in self.arm_configs.items():
            namespace = config['namespace']
            self.create_subscription(
                String,
                f'{namespace}/status',
                lambda message, arm=selected_arm: self._status_callback(arm, message),
                transient,
            )
            self.create_subscription(
                JointTrajectory,
                f'{namespace}/planned_trajectory',
                lambda message, arm=selected_arm: self._trajectory_callback(arm, message),
                transient,
            )
            self.create_subscription(
                PoseArray,
                f'{namespace}/waypoints',
                lambda message, arm=selected_arm: self._waypoints_callback(arm, message),
                transient,
            )
            self.create_subscription(
                PoseStamped,
                f'{namespace}/current_world_pose',
                lambda message, arm=selected_arm: self._current_world_pose_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                PoseStamped,
                f'{namespace}/world_root_pose',
                lambda message, arm=selected_arm: self._world_root_pose_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                PoseArray,
                f'{namespace}/world_waypoints',
                lambda message, arm=selected_arm: self._world_waypoints_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                Float64MultiArray,
                f'{namespace}/waypoint_dwells',
                lambda message, arm=selected_arm: self._waypoint_dwells_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                Float64MultiArray,
                f'{namespace}/waypoint_speed_scales',
                lambda message, arm=selected_arm: self._waypoint_speed_scales_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                Float64MultiArray,
                f'{namespace}/waypoint_grippers',
                lambda message, arm=selected_arm: self._waypoint_grippers_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                Int32MultiArray,
                f'{namespace}/waypoint_modes',
                lambda message, arm=selected_arm: self._waypoint_modes_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                Float64MultiArray, f'{namespace}/joint_timing_state',
                lambda message, arm=selected_arm: self._joint_timing_callback(arm, message),
                transient,
            )
            self.create_subscription(
                Bool,
                f'{namespace}/joint_smoothing_enabled',
                lambda message, arm=selected_arm: self._joint_smoothing_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                Float64MultiArray,
                f'{namespace}/waypoint_joint_values',
                lambda message, arm=selected_arm: self._waypoint_joint_values_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                UInt32,
                f'{namespace}/selected_start_waypoint',
                lambda message, arm=selected_arm: self._start_waypoint_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                UInt32,
                f'{namespace}/selected_end_waypoint',
                lambda message, arm=selected_arm: self._end_waypoint_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                Float64MultiArray,
                f'{namespace}/planned_waypoint_targets',
                lambda message, arm=selected_arm: self._waypoint_targets_callback(
                    arm, message),
                transient,
            )
            self.create_subscription(
                JointState,
                f'{namespace}/cartesian_jog_target',
                lambda message, arm=selected_arm: self._cartesian_jog_target_callback(
                    arm, message),
                10,
            )
            self.joint_waypoint_publishers[selected_arm] = self.create_publisher(
                JointState, f'{namespace}/joint_waypoint', 10)
            self.absolute_waypoint_publishers[selected_arm] = self.create_publisher(
                PoseStamped, f'{namespace}/waypoint_pose', 10)
            self.absolute_position_publishers[selected_arm] = self.create_publisher(
                PointStamped, f'{namespace}/absolute_position', 10)
            self.absolute_orientation_publishers[selected_arm] = self.create_publisher(
                QuaternionStamped, f'{namespace}/absolute_orientation', 10)
            self.relative_waypoint_publishers[selected_arm] = self.create_publisher(
                PoseStamped, f'{namespace}/relative_waypoint', 10)
            self.frame_relative_waypoint_publishers[selected_arm] = self.create_publisher(
                PoseStamped, f'{namespace}/frame_relative_waypoint', 10)
            self.relative_reference_publishers[selected_arm] = self.create_publisher(
                Int32, f'{namespace}/relative_reference', 10)
            self.cartesian_jog_publishers[selected_arm] = self.create_publisher(
                PointStamped, f'{namespace}/cartesian_jog', 10)
            self.waypoint_dwell_publishers[selected_arm] = self.create_publisher(
                Float64MultiArray, f'{namespace}/waypoint_dwell', 10)
            self.waypoint_speed_scale_publishers[selected_arm] = self.create_publisher(
                Float64MultiArray, f'{namespace}/waypoint_speed_scale', 10)
            self.equal_segment_time_publishers[selected_arm] = self.create_publisher(
                Float64MultiArray, f'{namespace}/equal_segment_time', 10)
            self.equal_total_time_publishers[selected_arm] = self.create_publisher(
                Float64MultiArray, f'{namespace}/equal_total_time', 10)
            self.waypoint_gripper_publishers[selected_arm] = self.create_publisher(
                Float64MultiArray, f'{namespace}/waypoint_gripper', 10)
            self.waypoint_mode_publishers[selected_arm] = self.create_publisher(
                Int32MultiArray, f'{namespace}/waypoint_mode', 10)
            self.joint_timing_publishers[selected_arm] = self.create_publisher(
                Float64MultiArray, f'{namespace}/joint_timing', 10)
            self.joint_smoothing_publishers[selected_arm] = self.create_publisher(
                Bool, f'{namespace}/joint_smoothing', 10)
            self.path_refinement_publishers[selected_arm] = self.create_publisher(
                Float64MultiArray, f'{namespace}/path_refinement', 10)
            self.create_subscription(
                Float64MultiArray, f'{namespace}/path_refinement_state',
                lambda msg, arm=selected_arm: self._path_refinement_callback(arm, msg), transient)
            self.create_subscription(
                String, f'{namespace}/path_refinement_diagnostics',
                lambda msg, arm=selected_arm: self._path_refinement_diagnostic_callback(arm, msg),
                QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.start_waypoint_publishers[selected_arm] = self.create_publisher(
                UInt32, f'{namespace}/start_waypoint', 10)
            self.end_waypoint_publishers[selected_arm] = self.create_publisher(
                UInt32, f'{namespace}/end_waypoint', 10)
            self.reorder_waypoint_publishers[selected_arm] = self.create_publisher(
                Int32MultiArray, f'{namespace}/reorder_waypoint', 10)
            self.remove_waypoint_publishers[selected_arm] = self.create_publisher(
                UInt32, f'{namespace}/remove_waypoint', 10)
            self.tool_path_correction_publishers[selected_arm] = self.create_publisher(
                Float64MultiArray, f'{namespace}/tool_path_correction', 10)
        self.keyboard_thread = threading.Thread(
            target=self._keyboard_keepalive_loop,
            name='waypoint-keyboard-keepalive',
            daemon=True,
        )
        self.keyboard_thread.start()

    def _joint_state_callback(self, message: JointState) -> None:
        positions = {
            name: float(value)
            for name, value in zip(message.name, message.position)
            if math.isfinite(float(value))
        }
        if not positions:
            return
        velocities = {
            name: float(value)
            for name, value in zip(message.name, message.velocity)
            if math.isfinite(float(value))
        }
        with self.lock:
            self.all_current_positions.update(positions)
            self.all_current_velocities.update(velocities)
            self.joint_state_stamp = time.monotonic()
            for selected_arm, config in self.arm_configs.items():
                selected = {
                    name: float(positions[name])
                    for name in config['joint_names']
                    if name in positions and math.isfinite(float(positions[name]))
                }
                arm_state = self.arm_states[selected_arm]
                arm_state['current_positions'].update(selected)
                if len(selected) == len(config['joint_names']):
                    arm_state['current_stamp'] = time.monotonic()

    def _previous_action_callback(
        self, source: str, arm: str, message: JointTrajectory
    ) -> None:
        expected = command_joint_names(arm)
        if list(message.joint_names) != expected or len(message.points) != 1:
            return
        values = [float(value) for value in message.points[0].positions]
        try:
            self._validate_arm_command(arm, values)
        except RuntimeError:
            return
        with self.lock:
            self.previous_action_positions[source][arm] = values

    def _status_callback(self, arm: str, message: String) -> None:
        with self.lock:
            arm_state = self.arm_states[arm]
            arm_state['status'] = message.data
            arm_state['status_sequence'] += 1
            previous_path_mode = arm_state['path_mode']
            if 'DEFAULT_PATH_MODE_SELECTED: LINEAR' in message.data:
                arm_state['path_mode'] = 'LINEAR'
            elif 'DEFAULT_PATH_MODE_SELECTED: JOINT' in message.data:
                arm_state['path_mode'] = 'JOINT'
            if arm_state['path_mode'] != previous_path_mode:
                arm_state['plan_revision'] += 1
            if 'count=' in message.data:
                try:
                    arm_state['waypoint_count'] = int(
                        message.data.rsplit('count=', 1)[1])
                except ValueError:
                    pass
            elif message.data == 'WAYPOINTS_CLEARED':
                arm_state['waypoint_count'] = 0

    def _arbiter_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        self._store_arbiter_status(payload)

    def _store_arbiter_status(self, payload: dict[str, Any]) -> None:
        with self.lock:
            self.arbiter_mode = str(payload.get('mode', 'UNKNOWN'))
            self.arbiter_fault = str(payload.get('fault', ''))
            self.arbiter_intervention_enabled = bool(
                payload.get('intervention_enabled', False))
            teleop = payload.get('sources', {}).get('teleop', {})
            self.arbiter_teleop_ready = teleop.get('ready') is True
            self.arbiter_teleop_reason = str(
                teleop.get('reason') or '상태를 아직 받지 못했습니다.')

    def _cartesian_jog_target_callback(
        self, arm: str, message: JointState
    ) -> None:
        expected = command_joint_names(arm)
        indexes = {name: index for index, name in enumerate(message.name)}
        if any(name not in indexes for name in expected):
            return
        positions = []
        for name in expected:
            index = indexes[name]
            if index >= len(message.position):
                return
            value = float(message.position[index])
            if not math.isfinite(value):
                return
            positions.append(value)
        reference = {
            side: list(values)
            for side, values in (self.keyboard_hold or {}).items()
        }
        if set(reference) != set(ARMS):
            return
        reference[arm] = positions
        try:
            self._validate_command_pair(reference)
        except RuntimeError as exception:
            self._set_status(arm, f'ERROR: Cartesian jog target rejected: {exception}')
            return
        with self.keyboard_condition:
            target = self.keyboard_jog_targets[arm]
            target['positions'] = positions
            target['sequence'] += 1
            self.keyboard_condition.notify_all()

    def _trajectory_callback(self, arm: str, message: JointTrajectory) -> None:
        with self.lock:
            self.arm_states[arm]['trajectory'] = message
            self.arm_states[arm]['trajectory_sequence'] += 1
            self.arm_states[arm]['plan_revision'] += 1

    def _waypoints_callback(self, arm: str, message: PoseArray) -> None:
        waypoints = []
        for index, pose in enumerate(message.poses):
            waypoints.append({
                'index': index + 1,
                'position_m': [
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                ],
                'orientation_xyzw': [
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                ],
            })
        with self.lock:
            arm_state = self.arm_states[arm]
            names = arm_state['waypoint_names']
            if len(names) < len(waypoints):
                names.extend(
                    self._default_waypoint_name(arm, index)
                    for index in range(len(names) + 1, len(waypoints) + 1)
                )
            elif len(names) > len(waypoints):
                del names[len(waypoints):]
            arm_state['waypoints'] = waypoints
            arm_state['waypoint_count'] = len(waypoints)
            arm_state['waypoints_update_sequence'] += 1
            arm_state['plan_revision'] += 1

    @staticmethod
    def _pose_snapshot(pose: Any) -> dict[str, list[float]]:
        return {
            'position_m': [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ],
            'orientation_xyzw': [
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ],
        }

    def _current_world_pose_callback(
        self, arm: str, message: PoseStamped
    ) -> None:
        with self.lock:
            self.arm_states[arm]['current_world_pose'] = self._pose_snapshot(
                message.pose)

    def _world_root_pose_callback(
        self, arm: str, message: PoseStamped
    ) -> None:
        with self.lock:
            self.arm_states[arm]['world_root_pose'] = self._pose_snapshot(
                message.pose)

    def _world_waypoints_callback(
        self, arm: str, message: PoseArray
    ) -> None:
        poses = [self._pose_snapshot(pose) for pose in message.poses]
        with self.lock:
            self.arm_states[arm]['world_waypoints'] = poses

    def _waypoint_dwells_callback(
        self, arm: str, message: Float64MultiArray
    ) -> None:
        dwells = [float(value) for value in message.data]
        if not all(math.isfinite(value) and value >= 0.0 for value in dwells):
            return
        with self.lock:
            self.arm_states[arm]['waypoint_dwells'] = dwells
            self.arm_states[arm]['plan_revision'] += 1

    def _waypoint_speed_scales_callback(
        self, arm: str, message: Float64MultiArray
    ) -> None:
        scales = [float(value) for value in message.data]
        if not all(math.isfinite(value) for value in scales):
            return
        with self.lock:
            self.arm_states[arm]['waypoint_speed_scales'] = scales
            self.arm_states[arm]['speed_scales_sequence'] += 1
            self.arm_states[arm]['plan_revision'] += 1

    def _waypoint_grippers_callback(
        self, arm: str, message: Float64MultiArray
    ) -> None:
        values = [float(value) for value in message.data]
        if not all(math.isfinite(value) for value in values):
            return
        with self.lock:
            self.arm_states[arm]['waypoint_grippers'] = values
            self.arm_states[arm]['plan_revision'] += 1

    def _waypoint_modes_callback(
        self, arm: str, message: Int32MultiArray
    ) -> None:
        values = [int(value) for value in message.data]
        if not all(value in (0, 1) for value in values):
            return
        with self.lock:
            self.arm_states[arm]['waypoint_modes'] = [
                'LINEAR' if value == 0 else 'JOINT' for value in values
            ]
            self.arm_states[arm]['plan_revision'] += 1

    def _joint_timing_callback(self, arm: str, message: Float64MultiArray) -> None:
        data = list(message.data)
        if len(data) != 3:
            return
        with self.lock:
            self.arm_states[arm]['joint_timing'] = (
                dict(speed_mps=data[0], preserve_start_waypoint=int(data[1]),
                     preserve_end_waypoint=int(data[2])) if data[0] > 0 else None)
            self.arm_states[arm]['plan_revision'] += 1

    def _joint_smoothing_callback(self, arm: str, message: Bool) -> None:
        with self.lock:
            self.arm_states[arm]['joint_smoothing_enabled'] = bool(message.data)
            self.arm_states[arm]['plan_revision'] += 1

    def _start_waypoint_callback(self, arm: str, message: UInt32) -> None:
        index = int(message.data)
        if index < 1:
            return
        with self.lock:
            self.arm_states[arm]['start_waypoint'] = index
            self.arm_states[arm]['plan_revision'] += 1

    def _end_waypoint_callback(self, arm: str, message: UInt32) -> None:
        index = int(message.data)
        if index < 1:
            return
        with self.lock:
            self.arm_states[arm]['end_waypoint'] = index
            self.arm_states[arm]['plan_revision'] += 1

    @staticmethod
    def _default_waypoint_name(arm: str, index: int) -> str:
        label = '왼팔' if arm == 'left' else '오른팔'
        return f'{label} 지점 {index}'

    def _execution_sequence_snapshot(self) -> list[dict[str, Any]]:
        result = []
        for index, step in enumerate(self.execution_sequence):
            arm = step['arm']
            state = self.arm_states[arm]
            start = step['start_index']
            end = step['end_index']
            names = state['waypoint_names']
            result.append({
                **step,
                'index': index + 1,
                'arm_label': '왼팔' if arm == 'left' else '오른팔',
                'start_name': names[start - 1] if start <= len(names) else f'Waypoint {start}',
                'end_name': names[end - 1] if end <= len(names) else f'Waypoint {end}',
            })
        return result

    def _clear_sequence_for_arm(self, arm: str) -> None:
        self.execution_sequence = [
            step for step in self.execution_sequence if step['arm'] != arm
        ]

    def _waypoint_joint_values_callback(
        self, arm: str, message: Float64MultiArray
    ) -> None:
        width = len(self.arm_configs[arm]['joint_names'])
        values = [float(value) for value in message.data]
        if len(values) % width != 0:
            return
        rows = []
        for offset in range(0, len(values), width):
            row = values[offset:offset + width]
            rows.append(row if all(math.isfinite(value) for value in row) else None)
        with self.lock:
            self.arm_states[arm]['waypoint_joint_values'] = rows
            self.arm_states[arm]['joint_values_update_sequence'] += 1
            self.arm_states[arm]['plan_revision'] += 1

    def _waypoint_targets_callback(
        self, arm: str, message: Float64MultiArray
    ) -> None:
        width = len(self.arm_configs[arm]['joint_names']) + 1
        values = [float(value) for value in message.data]
        if len(values) % width != 0 or not all(math.isfinite(value) for value in values):
            return
        targets = []
        for offset in range(0, len(values), width):
            targets.append({
                'time_s': values[offset],
                'positions_rad': values[offset + 1:offset + width],
            })
        with self.lock:
            self.arm_states[arm]['waypoint_targets'] = targets
            self.arm_states[arm]['plan_revision'] += 1

    def _robot_description_callback(self, message: String) -> None:
        if not message.data:
            return
        try:
            models = {}
            for selected_arm, config in self.arm_configs.items():
                model = parse_urdf_chain(
                    message.data, self.base_link, config['tip_link'])
                expected = set(config['arm_joint_names'])
                received = set(model['movable_joint_names'])
                if expected != received:
                    raise ValueError(
                        f'{selected_arm} arm URDF movable joints do not match: '
                        f'expected={sorted(expected)}, received={sorted(received)}'
                    )
                models[selected_arm] = model
        except ValueError as exception:
            models = {
                selected_arm: {
                    'status': 'error',
                    'message': str(exception),
                    'base_link': self.base_link,
                    'tip_link': config['tip_link'],
                }
                for selected_arm, config in self.arm_configs.items()
            }
            self.get_logger().warning(
                f'URDF simulator models rejected: {exception}')
        with self.lock:
            for selected_arm, model in models.items():
                self.arm_states[selected_arm]['robot_model'] = model
            if any(model['status'] == 'ready' for model in models.values()):
                self.robot_description_xml = message.data

    def _select_arm(self, arm: Any = None) -> str:
        selected = self.default_arm if arm in (None, '') else str(arm)
        if selected not in self.available_arms:
            raise ValueError(f'사용할 수 없는 팔입니다: {selected}')
        return selected

    def robot_model_snapshot(self, arm: Any = None) -> dict[str, Any]:
        selected_arm = self._select_arm(arm)
        with self.lock:
            return json.loads(json.dumps(
                self.arm_states[selected_arm]['robot_model']))

    def robot_description_snapshot(self) -> str:
        with self.lock:
            return self.robot_description_xml

    def snapshot(self, arm: Any = None) -> dict[str, Any]:
        selected_arm = self._select_arm(arm)
        # Callbacks replace plan arrays. Capture their references and mutable
        # metadata briefly, then format the large response outside the command
        # stream's lock. A slow /api/state client must not stall publishing.
        with self.lock:
            config = self.arm_configs[selected_arm]
            arm_state = {
                key: value.copy() if isinstance(value, (dict, list)) else value
                for key, value in self.arm_states[selected_arm].items()
            }
            current_positions = dict(self.all_current_positions)
            execution_sequence = self._execution_sequence_snapshot()
        age = (
            time.monotonic() - arm_state['current_stamp']
            if arm_state['current_stamp'] > 0.0 else None
        )
        trajectory = self._trajectory_snapshot(arm_state['trajectory'])
        waypoints = json.loads(json.dumps(arm_state['waypoints']))
        for index, waypoint in enumerate(waypoints):
            waypoint['name'] = (
                arm_state['waypoint_names'][index]
                if index < len(arm_state['waypoint_names'])
                else self._default_waypoint_name(selected_arm, index + 1)
            )
            waypoint['dwell_s'] = round(
                arm_state['waypoint_dwells'][index], 3
            ) if index < len(arm_state['waypoint_dwells']) else 0.0
            waypoint['speed_scale'] = (
                arm_state['waypoint_speed_scales'][index]
                if index < len(arm_state['waypoint_speed_scales']) else 1.0
            )
            gripper = (
                arm_state['waypoint_grippers'][index]
                if index < len(arm_state['waypoint_grippers']) else None
            )
            waypoint['gripper_rad'] = (
                round(gripper, 7) if gripper is not None else None
            )
            waypoint['gripper_deg'] = (
                round(math.degrees(gripper), 3)
                if gripper is not None else None
            )
            waypoint['interpolation_mode'] = (
                arm_state['waypoint_modes'][index]
                if index < len(arm_state['waypoint_modes'])
                else arm_state['path_mode']
            )
            waypoint_number = index + 1
            range_start = arm_state['start_waypoint']
            range_end = arm_state['end_waypoint']
            target_index = (
                abs(waypoint_number - range_start)
                if min(range_start, range_end) <= waypoint_number
                <= max(range_start, range_end)
                else -1
            )
            target = (
                arm_state['waypoint_targets'][target_index]
                if 0 <= target_index < len(arm_state['waypoint_targets'])
                else None
            )
            waypoint['trajectory_time_s'] = (
                round(target['time_s'], 4) if target is not None else None
            )
            stored_values = (
                arm_state['waypoint_joint_values'][index]
                if index < len(arm_state['waypoint_joint_values']) else None
            )
            display_values = stored_values or (
                target['positions_rad'] if target is not None else None
            )
            waypoint['joint_rad'] = (
                [round(value, 7) for value in display_values]
                if display_values is not None else None
            )
            waypoint['joint_deg'] = (
                [round(math.degrees(value), 3) for value in display_values]
                if display_values is not None else None
            )
            if index < len(arm_state['world_waypoints']):
                waypoint['world_pose'] = arm_state['world_waypoints'][index]
        return {
            'arm': selected_arm,
            'available_arms': list(self.available_arms),
            'tip_link': config['tip_link'],
            'world_frame': self.world_frame,
            'current_world_pose': arm_state['current_world_pose'],
            'world_root_pose': arm_state.get('world_root_pose'),
            'joint_names': config['joint_names'],
            'current_deg': [
                round(math.degrees(arm_state['current_positions'][name]), 3)
                if name in arm_state['current_positions'] else None
                for name in config['joint_names']
            ],
            'robot_joint_state': {
                'name': list(current_positions),
                'position': list(current_positions.values()),
            },
            'joint_state_age_s': round(age, 3) if age is not None else None,
            'joint_state_fresh': age is not None and age <= 0.25,
            'status': arm_state['status'],
            'execution_events': list(arm_state.get('execution_events', [])),
            'plan_revision': arm_state['plan_revision'],
            'path_mode': arm_state['path_mode'],
            'joint_smoothing_enabled': arm_state['joint_smoothing_enabled'],
            'path_refinement': arm_state.get('path_refinement', parse_path_refinement(None)),
            'path_refinement_events': list(arm_state.get('path_refinement_events', [])),
            'joint_timing': arm_state['joint_timing'],
            'start_waypoint': arm_state['start_waypoint'],
            'end_waypoint': arm_state['end_waypoint'],
            'waypoint_count': arm_state['waypoint_count'],
            'waypoints': waypoints,
            'execution_sequence': execution_sequence,
            'max_waypoint_dwell_s': self.max_waypoint_dwell_s,
            'max_waypoint_speed_scale': self.max_waypoint_speed_scale,
            'equal_segment_time_supported': True,
            'equal_total_time_supported': True,
            'publish_enabled': self.publish_enabled,
            'execution': {
                'active': self.execution_thread is not None
                and self.execution_thread.is_alive(),
                'arm': self.execution_arm,
                'command_source': 'WAYPOINT',
            },
            'keyboard_control': {
                'active': self.keyboard_active,
                'frame': self.world_frame,
                'period_s': self.keyboard_jog_period_s,
                'max_speed_mmps': self.keyboard_jog_max_speed_mmps,
                'deadman_timeout_s': self.keyboard_deadman_timeout_s,
            },
            'arbiter': {
                'mode': self.arbiter_mode,
                'fault': self.arbiter_fault,
                'intervention_enabled': self.arbiter_intervention_enabled,
                'teleop_ready': self.arbiter_teleop_ready,
                'teleop_reason': self.arbiter_teleop_reason,
            },
            'trajectory': trajectory,
        }

    def runtime_snapshot(self, arm: Any = None) -> dict[str, Any]:
        """Return frequently changing UI data without serializing the plan."""
        selected_arm = self._select_arm(arm)
        with self.lock:
            config = self.arm_configs[selected_arm]
            arm_state = self.arm_states[selected_arm]
            age = (
                time.monotonic() - arm_state['current_stamp']
                if arm_state['current_stamp'] > 0.0 else None
            )
            return {
                'arm': selected_arm,
                'available_arms': list(self.available_arms),
                'current_deg': [
                    round(math.degrees(arm_state['current_positions'][name]), 3)
                    if name in arm_state['current_positions'] else None
                    for name in config['joint_names']
                ],
                'robot_joint_state': {
                    'name': list(self.all_current_positions),
                    'position': list(self.all_current_positions.values()),
                },
                'joint_state_age_s': round(age, 3) if age is not None else None,
                'joint_state_fresh': age is not None and age <= 0.25,
                'status': arm_state['status'],
                'path_refinement_events': list(arm_state.get('path_refinement_events', [])),
                'execution_events': list(arm_state.get('execution_events', [])),
                'current_world_pose': arm_state.get('current_world_pose'),
                'world_root_pose': arm_state.get('world_root_pose'),
                'plan_revision': arm_state['plan_revision'],
                'execution_sequence': self._execution_sequence_snapshot(),
                'execution': {
                    'active': self.execution_thread is not None
                    and self.execution_thread.is_alive(),
                    'arm': self.execution_arm,
                    'command_source': 'WAYPOINT',
                },
                'keyboard_control': {
                    'active': self.keyboard_active,
                    'frame': self.world_frame,
                    'period_s': self.keyboard_jog_period_s,
                    'max_speed_mmps': self.keyboard_jog_max_speed_mmps,
                    'deadman_timeout_s': self.keyboard_deadman_timeout_s,
                },
                'arbiter': {
                    'mode': self.arbiter_mode,
                    'fault': self.arbiter_fault,
                    'intervention_enabled': self.arbiter_intervention_enabled,
                    'teleop_ready': self.arbiter_teleop_ready,
                    'teleop_reason': self.arbiter_teleop_reason,
                },
            }

    def _trajectory_snapshot(
        self, message: JointTrajectory | None
    ) -> dict[str, Any] | None:
        if message is None or not message.points:
            return None
        stride = max(1, math.ceil(len(message.points) / 240))
        selected_points = list(message.points[::stride])
        if selected_points[-1] is not message.points[-1]:
            selected_points.append(message.points[-1])
        samples = []
        for point in selected_points:
            duration = point.time_from_start
            samples.append({
                'time_s': round(duration.sec + duration.nanosec * 1.0e-9, 4),
                'positions_deg': [
                    round(math.degrees(value), 3) for value in point.positions
                ],
            })
        final_duration = message.points[-1].time_from_start
        return {
            'joint_names': list(message.joint_names),
            'sample_count': len(message.points),
            'duration_s': round(
                final_duration.sec + final_duration.nanosec * 1.0e-9, 4),
            'samples': samples,
        }

    def add_joint_waypoint(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 waypoint를 수정할 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        dwell_s = parse_dwell_seconds(payload, self.max_waypoint_dwell_s)
        speed_scale = parse_speed_scale(payload, self.max_waypoint_speed_scale)
        config = self.arm_configs[arm]
        positions = parse_joint_values(payload, len(config['joint_names']))
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(config['joint_names'])
        message.position = positions
        with self.lock:
            sequence = self.arm_states[arm]['status_sequence']
        self.joint_waypoint_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        if not status.startswith('JOINT_WAYPOINT_ADDED:'):
            raise RuntimeError('waypoint 노드의 입력 확인 응답이 없습니다.')
        return self._apply_added_options(arm, status, dwell_s, speed_scale)

    def add_relative_waypoint(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 waypoint를 수정할 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        dwell_s = parse_dwell_seconds(payload, self.max_waypoint_dwell_s)
        speed_scale = parse_speed_scale(payload, self.max_waypoint_speed_scale)
        reference_index = parse_relative_reference(payload)
        config = self.arm_configs[arm]
        translation, rotation = parse_relative_pose(payload)
        quaternion = rpy_to_quaternion(rotation)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = config['tip_link']
        message.pose.position.x = translation[0]
        message.pose.position.y = translation[1]
        message.pose.position.z = translation[2]
        message.pose.orientation.x = quaternion[0]
        message.pose.orientation.y = quaternion[1]
        message.pose.orientation.z = quaternion[2]
        message.pose.orientation.w = quaternion[3]
        with self.lock:
            waypoint_count = self.arm_states[arm]['waypoint_count']
            sequence = self.arm_states[arm]['status_sequence']
        if reference_index > waypoint_count:
            raise ValueError('선택한 기준 waypoint가 현재 목록에 없습니다.')
        reference_message = Int32()
        reference_message.data = reference_index
        self.relative_reference_publishers[arm].publish(reference_message)
        reference_status = self._wait_for_status(arm, sequence)
        if reference_status.startswith('ERROR:'):
            raise ValueError(reference_status.removeprefix('ERROR:').strip())
        if not reference_status.startswith('RELATIVE_REFERENCE_SELECTED:'):
            raise RuntimeError('상대 이동 계산 기준의 확인 응답이 없습니다.')
        with self.lock:
            sequence = self.arm_states[arm]['status_sequence']
        self.relative_waypoint_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        if not status.startswith('RELATIVE_WAYPOINT_ADDED:'):
            raise RuntimeError('상대 waypoint의 IK 계산 응답이 없습니다.')
        return self._apply_added_options(arm, status, dwell_s, speed_scale)

    def add_frame_relative_waypoint(
        self, payload: dict[str, Any]
    ) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 waypoint를 수정할 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        dwell_s = parse_dwell_seconds(payload, self.max_waypoint_dwell_s)
        speed_scale = parse_speed_scale(payload, self.max_waypoint_speed_scale)
        reference_index = parse_relative_reference(payload)
        translation, rotation = parse_relative_pose(payload)
        quaternion = rpy_to_quaternion(rotation)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.world_frame
        message.pose.position.x = translation[0]
        message.pose.position.y = translation[1]
        message.pose.position.z = translation[2]
        message.pose.orientation.x = quaternion[0]
        message.pose.orientation.y = quaternion[1]
        message.pose.orientation.z = quaternion[2]
        message.pose.orientation.w = quaternion[3]
        with self.lock:
            waypoint_count = self.arm_states[arm]['waypoint_count']
            sequence = self.arm_states[arm]['status_sequence']
        if reference_index > waypoint_count:
            raise ValueError('선택한 기준 waypoint가 현재 목록에 없습니다.')
        reference_message = Int32()
        reference_message.data = reference_index
        self.relative_reference_publishers[arm].publish(reference_message)
        reference_status = self._wait_for_status(arm, sequence)
        if reference_status.startswith('ERROR:'):
            raise ValueError(reference_status.removeprefix('ERROR:').strip())
        if not reference_status.startswith('RELATIVE_REFERENCE_SELECTED:'):
            raise RuntimeError('절대 frame 상대 이동 기준의 확인 응답이 없습니다.')
        with self.lock:
            sequence = self.arm_states[arm]['status_sequence']
        self.frame_relative_waypoint_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        if not status.startswith('FRAME_RELATIVE_WAYPOINT_ADDED:'):
            raise RuntimeError('절대 frame 상대 waypoint의 IK 계산 응답이 없습니다.')
        return self._apply_added_options(arm, status, dwell_s, speed_scale)

    def add_absolute_waypoint(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 waypoint를 수정할 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        dwell_s = parse_dwell_seconds(payload, self.max_waypoint_dwell_s)
        speed_scale = parse_speed_scale(payload, self.max_waypoint_speed_scale)
        mode, position, rotation = parse_absolute_target(payload)
        with self.lock:
            sequence = self.arm_states[arm]['status_sequence']
        if mode == 'position':
            assert position is not None
            message = PointStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.world_frame
            message.point.x, message.point.y, message.point.z = position
            self.absolute_position_publishers[arm].publish(message)
        elif mode == 'orientation':
            assert rotation is not None
            quaternion = rpy_to_quaternion(rotation)
            message = QuaternionStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.world_frame
            message.quaternion.x = quaternion[0]
            message.quaternion.y = quaternion[1]
            message.quaternion.z = quaternion[2]
            message.quaternion.w = quaternion[3]
            self.absolute_orientation_publishers[arm].publish(message)
        else:
            assert position is not None and rotation is not None
            quaternion = rpy_to_quaternion(rotation)
            message = PoseStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.world_frame
            message.pose.position.x = position[0]
            message.pose.position.y = position[1]
            message.pose.position.z = position[2]
            message.pose.orientation.x = quaternion[0]
            message.pose.orientation.y = quaternion[1]
            message.pose.orientation.z = quaternion[2]
            message.pose.orientation.w = quaternion[3]
            self.absolute_waypoint_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        expected_status = {
            'position': 'ABSOLUTE_POSITION_WAYPOINT_ADDED:',
            'orientation': 'ABSOLUTE_ORIENTATION_WAYPOINT_ADDED:',
            'pose': 'ABSOLUTE_WAYPOINT_ADDED:',
        }[mode]
        if not status.startswith(expected_status):
            raise RuntimeError('world 기준 절대 waypoint의 IK 계산 응답이 없습니다.')
        return self._apply_added_options(arm, status, dwell_s, speed_scale)

    def _apply_added_options(
        self, arm: str, added_status: str, dwell_s: float, speed_scale: float
    ) -> dict[str, Any]:
        try:
            index = int(added_status.rsplit('count=', 1)[1])
        except (IndexError, ValueError) as exception:
            raise RuntimeError('추가된 waypoint 번호를 확인할 수 없습니다.') from exception
        result: dict[str, Any] = {'success': True, 'message': added_status}
        details = []
        if dwell_s > 0.0:
            result['dwell'] = self.update_waypoint_dwell({
                'arm': arm, 'index': index, 'dwell_s': dwell_s,
            })
            details.append(f'도착 후 {dwell_s:g}초 대기')
        if speed_scale != 1.0:
            result['speed'] = self.update_waypoint_speed({
                'arm': arm, 'index': index, 'speed_scale': speed_scale,
            })
            details.append(f'동작 속도 배율 {speed_scale:g}')
        if details:
            result['message'] = f'{added_status} · ' + ' · '.join(details)
        return result

    def update_waypoint_speed(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 waypoint 동작 속도를 바꿀 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        index = parse_waypoint_index(payload)
        speed_scale = parse_speed_scale(payload, self.max_waypoint_speed_scale)
        with self.lock:
            state = self.arm_states[arm]
            if index > state['waypoint_count']:
                raise ValueError('속도를 수정할 waypoint 번호가 목록 밖입니다.')
            sequence = state['status_sequence']
        message = Float64MultiArray()
        message.data = [float(index), speed_scale]
        self.waypoint_speed_scale_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        if not status.startswith(f'WAYPOINT_SPEED_UPDATED: index={index},'):
            raise RuntimeError('waypoint 동작 속도 변경 응답이 없습니다.')
        return {
            'success': True,
            'message': f'Waypoint {index}의 동작 속도 배율을 {speed_scale:g}(으)로 변경했습니다.',
        }

    def equalize_segment_times(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active() or self.keyboard_active:
            raise RuntimeError('실행 또는 키보드 제어 중에는 구간 시간을 배정할 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        with self.lock:
            state = self.arm_states[arm]
            seconds, excluded = parse_equal_segment_time(payload, state['waypoint_count'])
            sequence = state['status_sequence']
            scales_sequence = state['speed_scales_sequence']
            trajectory_sequence = state['trajectory_sequence']
        total_mode = 'total_duration_s' in payload
        publisher = (self.equal_total_time_publishers if total_mode
                     else self.equal_segment_time_publishers)[arm]
        if publisher.get_subscription_count() == 0:
            raise RuntimeError('자동 시간 배정을 지원하는 waypoint 노드를 먼저 실행해 주세요.')
        message = Float64MultiArray()
        message.data = [seconds] + [float(index) for index in excluded]
        publisher.publish(message)
        # The native node validates the complete trajectory before committing.
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            with self.lock:
                status = self.arm_states[arm]['status']
                changed = self.arm_states[arm]['status_sequence'] > sequence
            if changed and (status.startswith('WAYPOINT_TIMES_VERIFIED:')
                            or status.startswith('ERROR:')):
                break
            time.sleep(0.02)
        else:
            raise RuntimeError('시간 배정 검증 응답이 지연됩니다. 상태를 새로 확인해 주세요.')
        if status.startswith('ERROR:'):
            raise ValueError('시간 배정 검증에 실패해 기존 설정을 유지했습니다: '
                             + status.removeprefix('ERROR:').strip())
        # Status and trajectory arrive on different ROS topics. Wait for the
        # committed values before the browser captures its verification state.
        sync_deadline = time.monotonic() + 2.0
        while time.monotonic() < sync_deadline:
            with self.lock:
                state = self.arm_states[arm]
                synced = (state['speed_scales_sequence'] > scales_sequence
                          and state['trajectory_sequence'] > trajectory_sequence
                          and state['trajectory'] is not None
                          and state['joint_timing'] is None)
            if synced:
                break
            time.sleep(0.01)
        else:
            raise RuntimeError('시간 배정 검증은 완료했으나 UI 데이터 수신이 지연됩니다. 새로고침해 주세요.')
        fields = dict(item.strip().split('=', 1)
                      for item in status.split(':', 1)[1].split(','))
        result = {
            'duration_s': float(fields['duration_s']),
            'updated': int(fields['updated']),
            'excluded': int(fields['excluded']),
            'max_error_s': float(fields['max_error_s']),
            'reference': fields['reference'],
            'verified': True,
        }
        for key in ('total_s', 'target_total_s', 'total_error_s', 'alignment_s',
                    'execution_total_s', 'fixed_s'):
            result[key] = float(fields[key])
        result['mode'] = fields['mode']
        if total_mode:
            return {'success': True, **result, 'message': (
                f"전체 목표시간 검증 완료 · 목표 {result['target_total_s']:.6f}초 / "
                f"계획 {result['total_s']:.6f}초 · 오차 {result['total_error_s'] * 1000:.6f} ms · "
                f"{result['updated']}개 구간 각각 {result['duration_s']:.6f}초 · "
                f"{result['excluded']}개 제외 · 첫 지점 정렬 {result['alignment_s']:.6f}초 별도 · "
                + ('첫 waypoint 기준 시뮬레이션' if result['reference'] == 'first_waypoint'
                   else '현재 관절 상태 기준 시뮬레이션'))}
        return {'success': True, **result, 'message': (
            f"시간 배정 검증 완료 · {result['updated']}개 구간 각각 "
            f"{result['duration_s']:.6f}초 · {result['excluded']}개 제외 · "
            f"최대 오차 {result['max_error_s'] * 1000:.6f} ms · "
            + ('첫 waypoint 기준 시뮬레이션' if result['reference'] == 'first_waypoint'
               else '현재 관절 상태 기준 시뮬레이션'))}

    def update_waypoint_dwell(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 waypoint 대기 시간을 바꿀 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        index = parse_waypoint_index(payload)
        dwell_s = parse_dwell_seconds(payload, self.max_waypoint_dwell_s)
        with self.lock:
            state = self.arm_states[arm]
            if index > state['waypoint_count']:
                raise ValueError('삭제하거나 수정할 waypoint 번호가 목록 밖입니다.')
            sequence = state['status_sequence']
        message = Float64MultiArray()
        message.data = [float(index), dwell_s]
        self.waypoint_dwell_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        expected = f'WAYPOINT_DWELL_UPDATED: index={index},'
        if not status.startswith(expected):
            raise RuntimeError('waypoint 대기 시간 변경 응답이 없습니다.')
        return {'success': True, 'message': status}

    def update_waypoint_mode(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 waypoint 보간 방식을 바꿀 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        index = parse_waypoint_index(payload)
        mode, mode_code = parse_waypoint_mode(payload)
        with self.lock:
            state = self.arm_states[arm]
            if index > state['waypoint_count']:
                raise ValueError('삭제하거나 수정할 waypoint 번호가 목록 밖입니다.')
            sequence = state['status_sequence']
        message = Int32MultiArray()
        message.data = [index, mode_code]
        self.waypoint_mode_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        expected = f'WAYPOINT_MODE_UPDATED: index={index}, mode={mode}'
        if status != expected:
            raise RuntimeError('waypoint 보간 방식 변경 응답이 없습니다.')
        return {'success': True, 'message': status}

    def _set_joint_timing(self, arm: str, timing: dict[str, Any] | None) -> None:
        message = Float64MultiArray()
        message.data = ([timing['speed_mps'], float(timing['preserve_start_waypoint']),
                         float(timing['preserve_end_waypoint'])] if timing else [0., 0., 0.])
        status = self._publish_planner_setting(
            arm, self.joint_timing_publishers[arm], message, 'JOINT_TIMING_UPDATED')
        if status != 'JOINT_TIMING_UPDATED':
            raise RuntimeError(f'등속 시간 배정 설정에 실패했습니다: {status}')

    def _path_refinement_callback(self, arm: str, message: Float64MultiArray) -> None:
        d = message.data
        if len(d) != 7:
            return
        settings = parse_path_refinement(dict(
            enabled=bool(d[0]), subdivisions=int(d[1]), smoothing_percent=d[2] * 100,
            position_tolerance_mm=d[3] * 1000, orientation_tolerance_deg=math.degrees(d[4]),
            joint_tolerance_rad=d[5], max_refinements=int(d[6])))
        with self.lock:
            self.arm_states[arm]['path_refinement'] = settings
            self.arm_states[arm]['plan_revision'] += 1

    def _path_refinement_diagnostic_callback(self, arm: str, message: String) -> None:
        try:
            event = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not isinstance(event, dict):
            return
        with self.lock:
            events = self.arm_states[arm]['path_refinement_events']
            if event.get('stage') in ('idle',) or (
                    event.get('stage') == 'generating' and event.get('attempt') == 1):
                events.clear()
            events.append(event)
            del events[:-32]

    def set_path_refinement(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active() or self._keyboard_is_active():
            raise RuntimeError('로봇 제어 중에는 보간 설정을 바꿀 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        settings = parse_path_refinement(payload.get('path_refinement'))
        with self.lock:
            if settings['enabled'] and self.arm_states[arm]['joint_timing']:
                raise ValueError('등속 시간 배정이 설정되어 있습니다. 이를 해제한 뒤 보간을 켜세요.')
        message = Float64MultiArray()
        message.data = [float(settings['enabled']), float(settings['subdivisions']),
                        settings['smoothing_percent'] / 100, settings['position_tolerance_mm'] / 1000,
                        math.radians(settings['orientation_tolerance_deg']),
                        settings['joint_tolerance_rad'], float(settings['max_refinements'])]
        status = self._publish_planner_setting(
            arm, self.path_refinement_publishers[arm], message, 'PATH_REFINEMENT_UPDATED')
        if status != 'PATH_REFINEMENT_UPDATED':
            raise RuntimeError(f'보간 설정을 적용하지 못했습니다: {status or "planner 응답 없음"}')
        return {'success': True, 'message': '보간 설정을 적용했습니다. 경로 계산 시 오차를 검증합니다.'}

    def set_joint_smoothing(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 JOINT 연속 통과 설정을 바꿀 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        enabled = payload.get('enabled')
        if not isinstance(enabled, bool):
            raise ValueError('JOINT 연속 통과 설정은 true 또는 false여야 합니다.')
        with self.lock:
            sequence = self.arm_states[arm]['status_sequence']
        message = Bool()
        message.data = enabled
        self.joint_smoothing_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        expected = f'JOINT_SMOOTHING_UPDATED: enabled={str(enabled).lower()}'
        if status != expected:
            raise RuntimeError('JOINT 연속 통과 설정 응답이 없습니다.')
        return {
            'success': True,
            'message': (
                'JOINT 연속 통과를 켰습니다. 중간 waypoint에서는 정지하지 않습니다.'
                if enabled else
                'JOINT 연속 통과를 껐습니다. 기존 구간별 보간을 사용합니다.'
            ),
        }

    def update_waypoint_gripper(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 waypoint 그리퍼 값을 바꿀 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        index = parse_waypoint_index(payload)
        try:
            value = float(payload.get('gripper_rad'))
        except (TypeError, ValueError) as exception:
            raise ValueError('그리퍼 값은 유한한 radian 숫자여야 합니다.') from exception
        if not math.isfinite(value):
            raise ValueError('그리퍼 값은 유한한 radian 숫자여야 합니다.')
        with self.lock:
            state = self.arm_states[arm]
            if index > state['waypoint_count']:
                raise ValueError('수정할 waypoint 번호가 목록 밖입니다.')
            sequence = state['status_sequence']
        message = Float64MultiArray()
        message.data = [float(index), value]
        self.waypoint_gripper_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        if not status.startswith(f'WAYPOINT_GRIPPER_UPDATED: index={index},'):
            raise RuntimeError('waypoint 그리퍼 값 변경 응답이 없습니다.')
        return {'success': True, 'message': status}

    def rename_waypoint(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 waypoint 이름을 바꿀 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        index = parse_waypoint_index(payload)
        name = parse_waypoint_name(payload)
        with self.lock:
            state = self.arm_states[arm]
            if index > state['waypoint_count']:
                raise ValueError('이름을 바꿀 waypoint 번호가 목록 밖입니다.')
            while len(state['waypoint_names']) < state['waypoint_count']:
                state['waypoint_names'].append(
                    self._default_waypoint_name(arm, len(state['waypoint_names']) + 1))
            state['waypoint_names'][index - 1] = name
            state['plan_revision'] += 1
        return {'success': True, 'message': f'Waypoint {index} 이름을 “{name}”(으)로 변경했습니다.'}

    def reorder_waypoint(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 waypoint 순서를 바꿀 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        source, target = parse_reorder_indices(payload)
        with self.lock:
            state = self.arm_states[arm]
            count = state['waypoint_count']
            if source > count or target > count:
                raise ValueError('이동할 waypoint 번호가 목록 밖입니다.')
            if source == target:
                return {'success': True, 'message': '순서가 변경되지 않았습니다.'}
            names = list(state['waypoint_names'])
            sequence = state['status_sequence']
        message = Int32MultiArray()
        message.data = [source, target]
        self.reorder_waypoint_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        if not status.startswith(f'WAYPOINT_REORDERED: from={source}, to={target}'):
            raise RuntimeError('waypoint 순서 변경 응답이 없습니다.')
        moved = names.pop(source - 1)
        names.insert(target - 1, moved)
        with self.lock:
            self.arm_states[arm]['waypoint_names'] = names
            self.arm_states[arm]['plan_revision'] += 1
            self._clear_sequence_for_arm(arm)
        return {'success': True, 'message': status}

    def select_start_waypoint(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 시작 waypoint를 바꿀 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        index = parse_waypoint_index(payload)
        return self._select_start_waypoint(arm, index)

    def select_end_waypoint(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 종료 waypoint를 바꿀 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        index = parse_waypoint_index(payload)
        return self._select_end_waypoint(arm, index)

    def _select_start_waypoint(self, arm: str, index: int) -> dict[str, Any]:
        with self.lock:
            state = self.arm_states[arm]
            if index > state['waypoint_count']:
                raise ValueError('시작 waypoint 번호가 목록 밖입니다.')
            sequence = state['status_sequence']
        message = UInt32()
        message.data = index
        self.start_waypoint_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        if status != f'START_WAYPOINT_SELECTED: index={index}':
            raise RuntimeError('시작 waypoint 변경 응답이 없습니다.')
        return {'success': True, 'message': status}

    def _select_end_waypoint(self, arm: str, index: int) -> dict[str, Any]:
        with self.lock:
            state = self.arm_states[arm]
            if index > state['waypoint_count']:
                raise ValueError('종료 waypoint 번호가 목록 밖입니다.')
            sequence = state['status_sequence']
        message = UInt32()
        message.data = index
        self.end_waypoint_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        if status != f'END_WAYPOINT_SELECTED: index={index}':
            raise RuntimeError('종료 waypoint 변경 응답이 없습니다.')
        return {'success': True, 'message': status}

    def remove_waypoint(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 waypoint를 삭제할 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        index = parse_waypoint_index(payload)
        with self.lock:
            state = self.arm_states[arm]
            if index > state['waypoint_count']:
                raise ValueError('삭제하거나 수정할 waypoint 번호가 목록 밖입니다.')
            sequence = state['status_sequence']
        message = UInt32()
        message.data = index
        self.remove_waypoint_publishers[arm].publish(message)
        status = self._wait_for_status(arm, sequence)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        expected = f'WAYPOINT_REMOVED: index={index},'
        if not status.startswith(expected):
            raise RuntimeError('지정한 waypoint 삭제 응답이 없습니다.')
        with self.lock:
            names = self.arm_states[arm]['waypoint_names']
            if index <= len(names):
                names.pop(index - 1)
                self.arm_states[arm]['plan_revision'] += 1
            self._clear_sequence_for_arm(arm)
        return {'success': True, 'message': status}

    def export_waypoints(self, arm_value: Any = None) -> dict[str, Any]:
        arm = self._select_arm(arm_value)
        snapshot = self.snapshot(arm)
        return {
            'schema': 'ai_worker_waypoints/v1',
            'arm': arm,
            'joint_smoothing_enabled': snapshot['joint_smoothing_enabled'],
            'path_refinement': snapshot['path_refinement'],
            'start_waypoint': snapshot['start_waypoint'],
            'end_waypoint': snapshot['end_waypoint'],
            **({'joint_timing': snapshot['joint_timing']} if snapshot['joint_timing'] else {}),
            'waypoints': [
                {
                    'name': waypoint['name'],
                    'position_m': waypoint['position_m'],
                    'orientation_xyzw': waypoint['orientation_xyzw'],
                    'joint_positions_rad': waypoint['joint_rad'],
                    'gripper_rad': waypoint['gripper_rad'],
                    'dwell_s': waypoint['dwell_s'],
                    'speed_scale': waypoint['speed_scale'],
                    'interpolation_mode': waypoint['interpolation_mode'],
                }
                for waypoint in snapshot['waypoints']
            ],
        }

    def generate_tool_path(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 보정 경로를 생성할 수 없습니다.')
        arm = self._select_arm(payload.get('arm'))
        with self.lock:
            state = self.arm_states[arm]
            correction = parse_tool_path_correction(
                payload, state['waypoint_count'])
            status_sequence = state['status_sequence']
            waypoint_sequence = state['waypoints_update_sequence']
            joint_sequence = state['joint_values_update_sequence']

        message = Float64MultiArray()
        message.data = [
            float(correction['approach_start_waypoint']),
            float(correction['grasp_waypoint']),
            *[value / 1000.0 for value in correction['local_offset_mm']],
        ]
        self.tool_path_correction_publishers[arm].publish(message)
        status = self._wait_for_status(arm, status_sequence, timeout_s=60.0)
        if status.startswith('ERROR:'):
            raise ValueError(status.removeprefix('ERROR:').strip())
        if not status.startswith('TOOL_PATH_GENERATED:'):
            raise RuntimeError('보정 경로 생성 응답이 없습니다.')

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with self.lock:
                state = self.arm_states[arm]
                updated = (
                    state['waypoints_update_sequence'] > waypoint_sequence
                    and state['joint_values_update_sequence'] > joint_sequence
                    and len(state['waypoint_joint_values']) == state['waypoint_count']
                    and all(
                        row is not None for row in state['waypoint_joint_values'])
                )
            if updated:
                break
            time.sleep(0.01)
        else:
            raise RuntimeError('생성된 pose와 IK 관절값을 확인할 수 없습니다.')

        with self.lock:
            self._clear_sequence_for_arm(arm)
        generated = self.export_waypoints(arm)
        generated['generator'] = {
            'algorithm': 'relative_grasp_compensation/v1',
            'approach_start_waypoint': correction['approach_start_waypoint'],
            'reference_grasp_waypoint': correction[
                'reference_grasp_waypoint'],
            'grasp_complete_waypoint': correction['grasp_complete_waypoint'],
            'full_path_correction': correction['full_path_correction'],
            'local_offset_mm': correction['local_offset_mm'],
            'correction_norm_mm': round(correction['norm_mm'], 6),
        }
        if correction['full_path_correction']:
            message = (
                f'{arm} 기준 경로를 보정했습니다. '
                '첫 waypoint부터 경로 끝까지 '
                '같은 상대 보정을 적용했습니다.'
            )
        else:
            message = (
                f'{arm} 기준 경로를 보정했습니다. '
                f'Waypoint {correction["grasp_waypoint"]}부터 '
                '경로의 마지막 waypoint까지 '
                '보정량을 유지합니다.'
            )
        return {
            'success': True,
            'message': message,
            'waypoint_file': generated,
        }

    def import_waypoints(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active() or self._keyboard_is_active():
            raise RuntimeError('실행 또는 키보드 제어 중에는 waypoint 파일을 불러올 수 없습니다.')
        if payload.get('schema') != 'ai_worker_waypoints/v1':
            raise ValueError('지원하지 않는 waypoint 파일 형식입니다.')
        arm = self._select_arm(payload.get('arm'))
        raw_waypoints = payload.get('waypoints')
        if not isinstance(raw_waypoints, list):
            raise ValueError('waypoint 파일의 waypoints는 목록이어야 합니다.')
        timing = parse_joint_timing(payload.get('joint_timing'), len(raw_waypoints))
        refinement = parse_path_refinement(payload.get('path_refinement'))
        if timing and refinement['enabled']:
            raise ValueError('등속 시간 배정과 오차 검증 보간을 함께 사용할 수 없습니다.')
        parsed = []
        for number, raw in enumerate(raw_waypoints, start=1):
            if not isinstance(raw, dict):
                raise ValueError(f'Waypoint {number} 데이터가 올바르지 않습니다.')
            try:
                name = parse_waypoint_name({'name': raw.get('name')})
                positions = parse_joint_values(
                    {'positions_rad': raw.get('joint_positions_rad')}, 8)
                dwell = parse_dwell_seconds(
                    {'dwell_s': raw.get('dwell_s', 0.0)}, self.max_waypoint_dwell_s)
                speed_scale = parse_speed_scale(
                    {
                        'speed_scale': raw['speed_scale']
                    } if 'speed_scale' in raw else {
                        'speed_percent': raw.get('speed_percent', 100.0)
                    },
                    self.max_waypoint_speed_scale,
                )
                mode, _ = parse_waypoint_mode({
                    'mode': raw.get('interpolation_mode', 'LINEAR')})
            except ValueError as exception:
                raise ValueError(f'Waypoint {number}: {exception}') from exception
            for joint_name, value, (lower, upper) in zip(
                command_joint_names(arm), positions, F2_POSITION_LIMITS[arm]
            ):
                if value < lower or value > upper:
                    raise ValueError(
                        f'Waypoint {number}: {joint_name}={value:.6f}이(가) '
                        f'제한 [{lower:.6f}, {upper:.6f}] 밖입니다.')
            parsed.append({
                'name': name,
                'positions_rad': positions,
                'dwell_s': dwell,
                'speed_scale': speed_scale,
                'mode': mode,
            })
        if timing and (not payload.get('joint_smoothing_enabled') or
                       any(w['mode'] != 'JOINT' for w in parsed)):
            raise ValueError('등속 시간 배정에는 모든 지점의 JOINT 연속 통과 설정이 필요합니다.')
        if not parsed:
            self._set_joint_timing(arm, None)
            self.set_path_refinement({'arm': arm, 'path_refinement': refinement})
            cleared = self._call_planner_action(arm, 'clear')
            if not cleared['success']:
                raise RuntimeError(cleared['message'])
            with self.lock:
                self.arm_states[arm]['waypoint_names'] = []
                self.arm_states[arm]['plan_revision'] += 1
                self._clear_sequence_for_arm(arm)
            if 'joint_smoothing_enabled' in payload:
                self.set_joint_smoothing({
                    'arm': arm,
                    'enabled': payload['joint_smoothing_enabled'],
                })
            return {
                'success': True,
                'message': f'{arm} waypoint 목록을 파일대로 비웠습니다.',
            }
        start = parse_waypoint_index({'index': payload.get('start_waypoint', 1)})
        end = parse_waypoint_index({
            'index': payload.get('end_waypoint', len(parsed))})
        if start > len(parsed) or end > len(parsed):
            raise ValueError('파일의 시작·종료 waypoint 구간이 올바르지 않습니다.')

        self._set_joint_timing(arm, None)
        self.set_path_refinement({'arm': arm, 'path_refinement': refinement})
        cleared = self._call_planner_action(arm, 'clear')
        if not cleared['success']:
            raise RuntimeError(cleared['message'])
        with self.lock:
            self.arm_states[arm]['waypoint_names'] = []
            self.arm_states[arm]['plan_revision'] += 1
            self._clear_sequence_for_arm(arm)
        for waypoint in parsed:
            self.add_joint_waypoint({
                'arm': arm,
                'positions_rad': waypoint['positions_rad'],
                'dwell_s': waypoint['dwell_s'],
                'speed_scale': waypoint['speed_scale'],
            })
            with self.lock:
                index = self.arm_states[arm]['waypoint_count']
            self.rename_waypoint({'arm': arm, 'index': index, 'name': waypoint['name']})
            self.update_waypoint_mode({
                'arm': arm, 'index': index, 'mode': waypoint['mode']})
        self._select_start_waypoint(arm, start)
        self._select_end_waypoint(arm, end)
        if 'joint_smoothing_enabled' in payload:
            self.set_joint_smoothing({
                'arm': arm,
                'enabled': payload['joint_smoothing_enabled'],
            })
        self._set_joint_timing(arm, timing)
        return {
            'success': True,
            'message': f'{arm} waypoint {len(parsed)}개를 파일에서 적용했습니다.',
        }

    def add_execution_step(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 순차 실행 목록을 바꿀 수 없습니다.')
        step = parse_execution_step(payload)
        if step['arm'] not in self.available_arms:
            raise ValueError('현재 실행 구성에서 사용할 수 없는 팔입니다.')
        with self.lock:
            count = self.arm_states[step['arm']]['waypoint_count']
            if max(step['start_index'], step['end_index']) > count:
                raise ValueError('순차 실행 구간이 waypoint 목록 밖입니다.')
            self.execution_sequence.append(step)
            sequence = self._execution_sequence_snapshot()
        return {'success': True, 'message': '실행 구간을 추가했습니다.', 'sequence': sequence}

    def remove_execution_step(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 순차 실행 목록을 바꿀 수 없습니다.')
        index = parse_waypoint_index(payload)
        with self.lock:
            if index > len(self.execution_sequence):
                raise ValueError('삭제할 실행 단계가 목록 밖입니다.')
            self.execution_sequence.pop(index - 1)
            sequence = self._execution_sequence_snapshot()
        return {'success': True, 'message': '실행 단계를 삭제했습니다.', 'sequence': sequence}

    def reorder_execution_step(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 순차 실행 목록을 바꿀 수 없습니다.')
        source, target = parse_reorder_indices(payload)
        with self.lock:
            if source > len(self.execution_sequence) or target > len(self.execution_sequence):
                raise ValueError('이동할 실행 단계가 목록 밖입니다.')
            moved = self.execution_sequence.pop(source - 1)
            self.execution_sequence.insert(target - 1, moved)
            sequence = self._execution_sequence_snapshot()
        return {'success': True, 'message': '실행 단계 순서를 변경했습니다.', 'sequence': sequence}

    def clear_execution_sequence(self) -> dict[str, Any]:
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 순차 실행 목록을 바꿀 수 없습니다.')
        with self.lock:
            self.execution_sequence.clear()
        return {'success': True, 'message': '순차 실행 목록을 비웠습니다.', 'sequence': []}

    def _keyboard_is_active(self) -> bool:
        with self.lock:
            return self.keyboard_active

    def _deactivate_keyboard(self) -> bool:
        with self.keyboard_condition:
            was_active = self.keyboard_active
            self.keyboard_active = False
            self.keyboard_hold = None
            self.keyboard_last_heartbeat = 0.0
            self.keyboard_condition.notify_all()
        return was_active

    def start_keyboard_control(self, payload: dict[str, Any]) -> dict[str, Any]:
        arm = self._select_arm(payload.get('arm'))
        if payload.get('confirm') != 'KEYBOARD':
            raise ValueError('키보드 WAYPOINT 제어 시작 확인이 필요합니다.')
        if not self.publish_enabled:
            raise RuntimeError('publish_enabled=false이므로 키보드 제어를 시작할 수 없습니다.')
        if self._execution_is_active():
            raise RuntimeError('waypoint trajectory 실행 중에는 키보드 제어를 시작할 수 없습니다.')
        if self._keyboard_is_active():
            self.keyboard_heartbeat()
            return {'success': True, 'message': '키보드 WAYPOINT 제어가 이미 활성화되어 있습니다.'}

        return_mode = self._arbiter_mode()
        hold, gripper_actions = self._keyboard_handoff_positions()
        self._publish_waypoint_pair(hold)
        time.sleep(0.02)
        self._publish_waypoint_pair(hold)
        try:
            self._call_arbiter('select_waypoint')
            hold = self._wait_until_stationary(gripper_actions)
        except Exception:
            self._call_arbiter('select_none')
            if return_mode == 'TELEOP':
                self._call_arbiter('select_teleop')
            raise

        with self.keyboard_condition:
            self.keyboard_hold = {
                side: list(values) for side, values in hold.items()
            }
            self.keyboard_last_heartbeat = time.monotonic()
            self.keyboard_active = True
            self.keyboard_condition.notify_all()
        self._set_status(arm, 'KEYBOARD_CONTROL_ACTIVE: world Cartesian jog')
        return {
            'success': True,
            'message': 'WAYPOINT 키보드 제어를 시작했습니다. 키를 누르는 동안만 이동합니다.',
        }

    def select_teleop_control(self, payload: dict[str, Any]) -> dict[str, Any]:
        arm = self._select_arm(payload.get('arm'))
        if payload.get('confirm') != 'TELEOP':
            raise ValueError('텔레옵 시작 확인이 필요합니다.')
        if not self.publish_enabled:
            return {
                'success': False,
                'message': 'publish_enabled=false이므로 텔레옵을 시작할 수 없습니다.',
            }
        self._deactivate_keyboard()
        try:
            result = self._select_relative_teleop_route()
        except Exception as exception:
            try:
                if self.intervention_controller_url:
                    self._call_intervention_controller('teleop_stop')
                else:
                    self._call_arbiter('select_none')
            except RuntimeError:
                pass
            self.get_logger().error(f'상대 TELEOP 연결 실패: {exception}')
            raise
        self._set_status(arm, 'TELEOP_CONTROL_ACTIVE')
        self.get_logger().info('상대 TELEOP 연결 성공: 제어권 TELEOP')
        return {
            'success': True,
            'message': (
                f"{result['message']} · 상대 TELEOP 준비 완료 · "
                '움직일 팔의 Mini leader 물리 버튼을 눌러 기준 자세를 설정하세요.'
            ),
        }

    def prepare_teleop_control(self, payload: dict[str, Any]) -> dict[str, Any]:
        arm = self._select_arm(payload.get('arm'))
        if payload.get('confirm') != 'PREPARE_TELEOP':
            raise ValueError('텔레옵 미리 연결 확인이 필요합니다.')
        if not self.publish_enabled:
            return {
                'success': False,
                'message': 'publish_enabled=false이므로 텔레옵을 준비할 수 없습니다.',
            }
        if self._keyboard_is_active():
            raise RuntimeError(
                '키보드 WAYPOINT 제어 중에는 텔레옵을 미리 연결할 수 없습니다.')
        self._set_status(arm, 'TELEOP_STANDBY_CONNECTING: Mini Leader 연결 중')
        self.get_logger().info('상대 TELEOP 미리 연결 시작: Mini Leader standby 준비')
        try:
            if self.intervention_controller_url:
                self._call_intervention_controller('standby')
            else:
                self._call_arbiter('prepare_intervention')
            self._wait_for_teleop_ready()
        except Exception as exception:
            self._set_status(arm, f'TELEOP_STANDBY_ERROR: {exception}')
            self.get_logger().error(f'상대 TELEOP 미리 연결 실패: {exception}')
            if not self.intervention_controller_url:
                self._call_arbiter('select_none')
            raise
        self._set_status(arm, 'TELEOP_STANDBY_READY: relative input connected')
        self.get_logger().info(
            '상대 TELEOP 미리 연결 성공: Mini Leader 준비, 제어권 NONE')
        return {
            'success': True,
            'message': (
                '상대 TELEOP 입력을 미리 연결했습니다. 제어권은 NONE이며, '
                '상대 리더 TELEOP을 누르기 전에는 로봇으로 전달되지 않습니다.'
            ),
        }

    def _select_relative_teleop_route(self) -> dict[str, Any]:
        if self.intervention_controller_url:
            status = self._read_arbiter_status()
            teleop = status.get('sources', {}).get('teleop', {})
            if not (
                status.get('intervention_enabled') is True
                and teleop.get('ready') is True
            ):
                self._call_intervention_controller('standby')
            self._call_intervention_controller('teleop_start')
            status = self._read_arbiter_status()
            if status.get('mode') != 'TELEOP':
                raise RuntimeError('상대 TELEOP 연결 후 제어권이 TELEOP이 아닙니다.')
            return {'success': True, 'message': 'Mini Leader 상대 TELEOP 연결 성공'}
        status = self._read_arbiter_status()
        teleop = status.get('sources', {}).get('teleop', {})
        if not (
            status.get('intervention_enabled') is True
            and teleop.get('ready') is True
        ):
            self._call_arbiter('prepare_intervention')
            self._wait_for_teleop_ready()
        return self._call_arbiter('select_teleop')

    def _read_arbiter_status(self) -> dict[str, Any]:
        result = self._call_arbiter('get_status')
        try:
            status = json.loads(result['message'])
        except (json.JSONDecodeError, KeyError, TypeError) as exception:
            raise RuntimeError(
                'command_source_arbiter 상태가 올바르지 않습니다.'
            ) from exception
        fault = str(status.get('fault', ''))
        if fault:
            raise RuntimeError(f'command_source_arbiter fault: {fault}')
        self._store_arbiter_status(status)
        return status

    def _wait_for_teleop_ready(self) -> None:
        """Wait until both isolated Mini leader streams are fresh."""
        deadline = time.monotonic() + self.teleop_ready_timeout_s
        last_reason = '입력을 아직 받지 못했습니다.'
        while time.monotonic() < deadline:
            status = self._read_arbiter_status()
            teleop = status.get('sources', {}).get('teleop', {})
            if teleop.get('ready') is True:
                return
            last_reason = str(teleop.get('reason') or last_reason)
            time.sleep(0.05)
        raise RuntimeError(
            'Mini leader 상대 TELEOP 입력 준비 시간이 초과되었습니다: '
            f'{last_reason}'
        )

    def keyboard_heartbeat(self) -> dict[str, Any]:
        with self.keyboard_condition:
            if not self.keyboard_active:
                raise RuntimeError('키보드 WAYPOINT 제어가 활성화되어 있지 않습니다.')
            if self.arbiter_fault or self.arbiter_mode != 'WAYPOINT':
                self.keyboard_active = False
                self.keyboard_hold = None
                raise RuntimeError(
                    f'키보드 제어권이 없습니다: {self.arbiter_fault or self.arbiter_mode}'
                )
            self.keyboard_last_heartbeat = time.monotonic()
        return {'success': True, 'message': 'keyboard heartbeat'}

    def keyboard_jog(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not self.keyboard_jog_lock.acquire(blocking=False):
            raise RuntimeError('이전 키보드 이동 요청을 처리 중입니다.')
        try:
            return self._keyboard_jog_locked(payload)
        finally:
            self.keyboard_jog_lock.release()

    def _keyboard_jog_locked(self, payload: dict[str, Any]) -> dict[str, Any]:
        arm = self._select_arm(payload.get('arm'))
        direction, vector, speed_mmps = parse_cartesian_jog(
            payload, self.keyboard_jog_max_speed_mmps, self.keyboard_jog_period_s)
        if self._execution_is_active():
            raise RuntimeError('waypoint trajectory 실행 중에는 키보드 제어를 사용할 수 없습니다.')
        with self.keyboard_condition:
            if not self.keyboard_active:
                raise RuntimeError('먼저 WAYPOINT 키보드 제어권을 선택해 주세요.')
            if self.arbiter_fault or self.arbiter_mode != 'WAYPOINT':
                self.keyboard_active = False
                self.keyboard_hold = None
                raise RuntimeError(
                    f'키보드 제어권이 없습니다: {self.arbiter_fault or self.arbiter_mode}'
                )
            previous_sequence = self.keyboard_jog_targets[arm]['sequence']
            self.keyboard_last_heartbeat = time.monotonic()

        message = PointStamped()
        message.header.frame_id = self.world_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.point.x, message.point.y, message.point.z = vector
        self.cartesian_jog_publishers[arm].publish(message)

        deadline = time.monotonic() + 0.30
        with self.keyboard_condition:
            while self.keyboard_jog_targets[arm]['sequence'] <= previous_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    status = self.arm_states[arm]['status']
                    raise RuntimeError(f'Cartesian jog 목표를 받지 못했습니다: {status}')
                self.keyboard_condition.wait(remaining)
            positions = list(self.keyboard_jog_targets[arm]['positions'])
            if not self.keyboard_active or self.keyboard_hold is None:
                raise RuntimeError('키보드 제어가 중지되었습니다.')
            pair = {
                side: list(values) for side, values in self.keyboard_hold.items()
            }
            # The planner's jog target contains the measured gripper state.
            # Keep the command-side gripper target captured at handoff instead.
            positions[-1] = pair[arm][-1]
            pair[arm] = positions
            self._validate_command_pair(pair)
            self.keyboard_hold = pair
            self.keyboard_last_heartbeat = time.monotonic()
        self._publish_waypoint_pair(pair)
        return {
            'success': True,
            'message': f'{arm} {direction} · {speed_mmps:g} mm/s',
        }

    def _keyboard_keepalive_loop(self) -> None:
        while not self.keyboard_shutdown.wait(0.02):
            timed_out = False
            hold = None
            with self.keyboard_condition:
                if self.keyboard_active:
                    timed_out = (
                        time.monotonic() - self.keyboard_last_heartbeat
                        > self.keyboard_deadman_timeout_s
                    )
                    if timed_out:
                        self.keyboard_active = False
                        self.keyboard_hold = None
                    elif self.keyboard_hold is not None:
                        hold = {
                            side: list(values)
                            for side, values in self.keyboard_hold.items()
                        }
            if timed_out:
                try:
                    self._call_arbiter('select_none')
                except RuntimeError as exception:
                    self.get_logger().error(
                        f'키보드 deadman 정지 실패: {exception}')
                for arm in self.available_arms:
                    self._set_status(arm, 'STOPPED: keyboard heartbeat timeout')
                continue
            if hold is not None:
                try:
                    self._publish_waypoint_pair(hold)
                except RuntimeError as exception:
                    self._deactivate_keyboard()
                    try:
                        self._call_arbiter('select_none')
                    except RuntimeError:
                        pass
                    self.get_logger().error(f'키보드 command 유지 실패: {exception}')

    def call_action(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        arm = self._select_arm(payload.get('arm'))
        if action not in ACTION_NAMES:
            raise ValueError(f'지원하지 않는 동작입니다: {action}')
        if action == 'stop':
            if payload.get('confirm') != STOP_BUTTON_CONFIRMATION:
                raise ValueError(
                    '정지 요청은 UI의 정지 요청 버튼을 '
                    '마우스나 터치로 직접 눌러야 합니다.')
            # STOP must not wait for execution_lock while initial planning is running.
            self.execution_stop.set()
            self._deactivate_keyboard()
            if self.intervention_controller_url:
                self._call_intervention_controller('teleop_stop')
                response = {
                    'success': True,
                    'message': '정지 요청 완료: 모든 arm command source를 차단했습니다.',
                }
            else:
                response = self._call_arbiter('select_none')
            self._set_status(
                arm, 'STOP_REQUESTED: all arm command sources disabled')
            return response
        if self._execution_is_active():
            raise RuntimeError('실행 중에는 STOP 이외의 동작을 요청할 수 없습니다.')
        if action == 'select_teleop':
            return self.select_teleop_control(payload)
        if action == 'prepare_teleop':
            return self.prepare_teleop_control(payload)
        if action == 'capture_current':
            dwell_s = parse_dwell_seconds(payload, self.max_waypoint_dwell_s)
            speed_scale = parse_speed_scale(
                payload, self.max_waypoint_speed_scale)
            with self.lock:
                previous_count = self.arm_states[arm]['waypoint_count']
            result = self._call_planner_action(arm, action)
            if not result['success']:
                return result
            deadline = time.monotonic() + 2.0
            index = previous_count + 1
            while time.monotonic() < deadline:
                with self.lock:
                    if self.arm_states[arm]['waypoint_count'] >= index:
                        break
                time.sleep(0.01)
            else:
                raise RuntimeError('추가된 현재 자세 waypoint를 확인할 수 없습니다.')
            details = []
            if dwell_s > 0.0:
                self.update_waypoint_dwell({
                    'arm': arm, 'index': index, 'dwell_s': dwell_s,
                })
                details.append(f'도착 후 {dwell_s:g}초 대기')
            if speed_scale != 1.0:
                self.update_waypoint_speed({
                    'arm': arm, 'index': index, 'speed_scale': speed_scale,
                })
                details.append(f'동작 속도 배율 {speed_scale:g}')
            return {
                'success': True,
                'message': result['message'] + (
                    ' · ' + ' · '.join(details) if details else ''),
            }
        if action == 'execute':
            if payload.get('confirm') != 'EXECUTE':
                raise ValueError('실행 확인이 필요합니다.')
            if not self.publish_enabled:
                return {
                    'success': False,
                    'message': 'publish_enabled=false이므로 Preview만 가능합니다.',
                }
            return self._start_waypoint_execution(arm)
        if action == 'execute_slow_start':
            if payload.get('confirm') != 'EXECUTE':
                raise ValueError('실행 확인이 필요합니다.')
            if not self.publish_enabled:
                return {
                    'success': False,
                    'message': 'publish_enabled=false이므로 Preview만 가능합니다.',
                }
            return self._start_waypoint_execution(arm, slow_start=True)
        result = self._call_planner_action(arm, action)
        if result['success'] and action in {'remove_last', 'clear'}:
            with self.lock:
                names = self.arm_states[arm]['waypoint_names']
                if action == 'clear':
                    names.clear()
                elif names:
                    names.pop()
                self.arm_states[arm]['plan_revision'] += 1
                self._clear_sequence_for_arm(arm)
        return result

    def _call_planner_action(self, arm: str, action: str) -> dict[str, Any]:
        client_key = (arm, action)
        # Long preview refinement must not hold the arbiter's stop/service lock.
        with self.planner_locks[arm]:
            client = self.service_clients.get(client_key)
            if client is None:
                namespace = self.arm_configs[arm]['namespace']
                client = self.create_client(Trigger, f'{namespace}/{action}')
                self.service_clients[client_key] = client
            response = self._call_trigger(
                client,
                'waypoint 노드 서비스에 연결할 수 없습니다.',
                'waypoint 노드 응답 시간이 초과되었습니다.',
                timeout_s=60.0 if self.arm_states[arm]['path_refinement']['enabled'] else 5.0,
            )
        return {'success': response.success, 'message': response.message}

    def _call_arbiter(self, action: str) -> dict[str, Any]:
        with self.service_lock:
            client = self.arbiter_clients.get(action)
            if client is None:
                client = self.create_client(
                    Trigger, f'{self.command_arbiter_namespace}/{action}')
                self.arbiter_clients[action] = client
            response = self._call_trigger(
                client,
                'command_source_arbiter 서비스에 연결할 수 없습니다.',
                'command_source_arbiter 응답 시간이 초과되었습니다.',
            )
        if not response.success:
            raise RuntimeError(response.message)
        expected_modes = {
            'select_none': 'NONE',
            'select_policy': 'POLICY',
            'select_teleop': 'TELEOP',
            'select_waypoint': 'WAYPOINT',
        }
        if action in expected_modes:
            with self.lock:
                self.arbiter_mode = expected_modes[action]
        return {'success': True, 'message': response.message}

    def _call_intervention_controller(self, action: str) -> dict[str, Any]:
        if not self.intervention_controller_url:
            raise RuntimeError('intervention controller URL이 설정되지 않았습니다.')
        body = json.dumps({'model': 'mini'}).encode('utf-8')
        headers = {
            'Content-Type': 'application/json',
            'X-Control-Client': 'waypoint-ui',
        }
        if self.intervention_controller_token:
            headers['X-Control-Token'] = self.intervention_controller_token
        request = Request(
            f'{self.intervention_controller_url}/api/intervention/{action}',
            data=body,
            headers=headers,
            method='POST',
        )
        try:
            with urlopen(
                request, timeout=self.intervention_controller_timeout_s
            ) as response:
                payload = json.loads(response.read().decode('utf-8'))
        except HTTPError as exception:
            message = f'HTTP {exception.code}'
            try:
                error_payload = json.loads(exception.read().decode('utf-8'))
                detail = error_payload.get('detail', error_payload)
                if isinstance(detail, dict):
                    message = str(detail.get('message') or detail)
                else:
                    message = str(detail)
            except (OSError, UnicodeDecodeError, json.JSONDecodeError):
                pass
            raise RuntimeError(
                f'Mini Leader 관리 서비스 요청 실패: {message}'
            ) from exception
        except (URLError, TimeoutError, OSError) as exception:
            raise RuntimeError(
                'Mini Leader 관리 서비스(127.0.0.1:8779)에 연결할 수 없습니다.'
            ) from exception
        except json.JSONDecodeError as exception:
            raise RuntimeError(
                'Mini Leader 관리 서비스 응답이 올바르지 않습니다.'
            ) from exception
        if not isinstance(payload, dict) or payload.get('ok') is not True:
            raise RuntimeError('Mini Leader 관리 서비스가 요청을 완료하지 못했습니다.')
        return payload

    @staticmethod
    def _call_trigger(client, unavailable: str, timeout: str, timeout_s: float = 5.0):
        if not client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError(unavailable)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done() or future.result() is None:
            raise RuntimeError(timeout)
        return future.result()

    def _arbiter_mode(self) -> str:
        result = self._call_arbiter('get_status')
        try:
            status = json.loads(result['message'])
        except (json.JSONDecodeError, TypeError) as exception:
            raise RuntimeError('command_source_arbiter 상태가 올바르지 않습니다.') from exception
        mode = str(status.get('mode', ''))
        fault = str(status.get('fault', ''))
        if fault:
            raise RuntimeError(f'command_source_arbiter fault: {fault}')
        if mode not in {'NONE', 'TELEOP'}:
            raise RuntimeError(f'{mode or "UNKNOWN"} 제어 중에는 waypoint를 실행할 수 없습니다.')
        with self.lock:
            self.arbiter_mode = mode
            self.arbiter_fault = fault
        return mode

    def _execution_is_active(self) -> bool:
        with self.execution_lock:
            return self.execution_thread is not None and self.execution_thread.is_alive()

    def _set_status(self, arm: str, status: str) -> None:
        event = None
        with self.lock:
            state = self.arm_states[arm]
            state['status'] = status
            state['status_sequence'] += 1
            if status.startswith(('EXECUTING:', 'EXECUTING_SEQUENCE:',
                                  'EXECUTION_COMPLETE:', 'EXECUTION_SEQUENCE_COMPLETE:',
                                  'STOPPED:', 'ERROR:')):
                event = {
                    'time_unix_s': time.time(),
                    'status': status,
                    'progress': dict(state.get('execution_progress', {})),
                    'arbiter_mode': self.arbiter_mode,
                    'arbiter_fault': self.arbiter_fault,
                }
                events = state.setdefault('execution_events', [])
                events.append(event)
                del events[:-20]
        if event is not None:
            logger = self.get_logger()
            message = f'WAYPOINT_EXECUTION {arm}: {json.dumps(event, ensure_ascii=False)}'
            if status.startswith('ERROR:'):
                logger.error(message)
            elif status.startswith('STOPPED:'):
                logger.warning(message)
            else:
                logger.info(message)

    def _record_stream_progress(
        self, arm: str, phase: str, sample_index: int,
        sample_count: int, planned_time_s: float,
    ) -> None:
        with self.lock:
            states = getattr(self, 'arm_states', {})
            if arm in states:
                states[arm]['execution_progress'] = {
                    'phase': phase,
                    'sample_index': sample_index,
                    'sample_count': sample_count,
                    'planned_time_s': planned_time_s,
                }

    def _current_command_positions(self) -> dict[str, list[float]]:
        with self.lock:
            age = time.monotonic() - self.joint_state_stamp
            if self.joint_state_stamp <= 0.0 or age > 0.25:
                raise RuntimeError('joint state가 오래되었거나 없습니다.')
            result = {}
            for arm in ARMS:
                names = command_joint_names(arm)
                missing = [name for name in names if name not in self.all_current_positions]
                if missing:
                    raise RuntimeError(f'joint state에 {missing[0]}이(가) 없습니다.')
                result[arm] = [self.all_current_positions[name] for name in names]
        self._validate_command_pair(result)
        return result

    def _handoff_gripper_actions(
        self,
        active_arms: set[str] | None = None,
        active_start_waypoints: dict[str, int] | None = None,
    ) -> dict[str, float]:
        active = set(active_arms or ())
        start_waypoints = dict(active_start_waypoints or {})
        if not set(start_waypoints).issubset(active):
            raise ValueError('활성 팔에 포함되지 않은 시작 waypoint가 지정되었습니다.')
        with self.lock:
            result = {}
            for arm in ARMS:
                if arm in active:
                    arm_state = self.arm_states[arm]
                    start_waypoint = int(start_waypoints.get(
                        arm, arm_state['start_waypoint']))
                    grippers = arm_state['waypoint_grippers']
                    if start_waypoint < 1 or start_waypoint > len(grippers):
                        raise RuntimeError(
                            f'{arm} 팔의 시작 waypoint 그리퍼 값이 없습니다.')
                    value = float(grippers[start_waypoint - 1])
                    lower, upper = F2_POSITION_LIMITS[arm][-1]
                    if not math.isfinite(value) or value < lower or value > upper:
                        raise RuntimeError(
                            f'{arm} 팔의 시작 waypoint 그리퍼 값 '
                            f'{value:.6f}이(가) effective limit '
                            f'[{lower:.6f}, {upper:.6f}] 밖입니다.')
                    result[arm] = value
                    continue

                values = (
                    self.previous_action_positions['final'][arm]
                    or self.previous_action_positions['teleop'][arm]
                )
                if values is not None:
                    result[arm] = values[-1]
                    continue

                gripper_name = command_joint_names(arm)[-1]
                current = self.all_current_positions.get(gripper_name)
                joint_state_age = time.monotonic() - self.joint_state_stamp
                current_is_fresh = (
                    current is not None
                    and math.isfinite(float(current))
                    and self.joint_state_stamp > 0.0
                    and joint_state_age <= 0.25
                )
                # An inactive arm with no previous command keeps its fresh
                # measured gripper position during the handoff.
                if current_is_fresh:
                    result[arm] = float(current)
                    continue
                raise RuntimeError(
                    f'{arm} 그리퍼의 이전 action과 fresh joint state가 없습니다.')
        return result

    def _keyboard_handoff_positions(
        self,
        gripper_actions: dict[str, float] | None = None,
        active_arms: set[str] | None = None,
        active_start_waypoints: dict[str, int] | None = None,
    ) -> tuple[dict[str, list[float]], dict[str, float]]:
        actions = gripper_actions or self._handoff_gripper_actions(
            active_arms, active_start_waypoints)
        positions = self._current_command_positions()
        for arm in ARMS:
            positions[arm][-1] = actions[arm]
        self._validate_command_pair(positions)
        return positions, actions

    @staticmethod
    def _trajectory_time(point: JointTrajectoryPoint) -> float:
        return point.time_from_start.sec + point.time_from_start.nanosec * 1.0e-9

    def _validate_arm_command(self, arm: str, values: list[float]) -> None:
        names = command_joint_names(arm)
        if len(values) != len(names):
            raise RuntimeError(f'{arm} waypoint command 크기가 올바르지 않습니다.')
        for name, value, (lower, upper) in zip(
            names, values, F2_POSITION_LIMITS[arm]
        ):
            if not math.isfinite(value) or value < lower or value > upper:
                raise RuntimeError(
                    f'{name}={value:.6f}이(가) effective limit '
                    f'[{lower:.6f}, {upper:.6f}] 밖입니다.'
                )

    def _validate_command_pair(self, positions: dict[str, list[float]]) -> None:
        for arm in ARMS:
            self._validate_arm_command(arm, positions[arm])

    def _publish_waypoint_pair(self, positions: dict[str, list[float]]) -> None:
        self._validate_command_pair(positions)
        for arm in ARMS:
            message = JointTrajectory()
            # This is a streaming, one-point command. Leave the trajectory
            # start stamp at zero so joint_trajectory_controller starts it on
            # receipt; a non-zero now stamp with time_from_start=0 is already
            # in the past by the time the controller validates the message.
            message.joint_names = command_joint_names(arm)
            point = JointTrajectoryPoint()
            point.positions = list(positions[arm])
            message.points = [point]
            self.waypoint_command_publishers[arm].publish(message)

    def _wait_until_stationary(
        self, gripper_actions: dict[str, float] | None = None
    ) -> dict[str, list[float]]:
        deadline = time.monotonic() + self.handoff_stop_timeout_s
        while time.monotonic() < deadline:
            positions = self._current_command_positions()
            if gripper_actions is not None:
                for arm in ARMS:
                    positions[arm][-1] = gripper_actions[arm]
                self._validate_command_pair(positions)
            with self.lock:
                names = command_joint_names('left') + command_joint_names('right')
                velocities = [self.all_current_velocities.get(name) for name in names]
            if all(
                velocity is not None
                and abs(velocity) <= self.max_start_velocity_radps
                for velocity in velocities
            ):
                return positions
            self._publish_waypoint_pair(positions)
            time.sleep(0.02)
        raise RuntimeError('텔레옵 정지 후 팔 속도가 제한 안으로 들어오지 않았습니다.')

    def _fresh_plan(
        self, arm: str, slow_start: bool = False
    ) -> JointTrajectory:
        with self.lock:
            sequence = self.arm_states[arm]['trajectory_sequence']
        planner_action = 'preview_slow_start' if slow_start else 'preview'
        result = self._call_planner_action(arm, planner_action)
        if not result['success']:
            raise RuntimeError(result['message'])
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with self.lock:
                state = self.arm_states[arm]
                if state['trajectory_sequence'] > sequence and state['trajectory'] is not None:
                    return state['trajectory']
            time.sleep(0.01)
        raise RuntimeError('새 실행 trajectory를 받지 못했습니다.')

    def _validate_plan(
        self, arm: str, trajectory: JointTrajectory, start: list[float]
    ) -> None:
        expected = command_joint_names(arm)
        if list(trajectory.joint_names) != expected or not trajectory.points:
            raise RuntimeError('실행 trajectory의 관절 순서 또는 point가 올바르지 않습니다.')
        # The live state used by the planner can differ slightly from the last
        # command held by this bridge. _stream_trajectory connects that gap
        # with bounded intermediate commands before publishing the plan.
        previous: list[float] | None = None
        previous_time = 0.0
        reference_pair = self._current_command_positions()
        for point in trajectory.points:
            values = list(point.positions)
            point_time = self._trajectory_time(point)
            if len(values) != 8 or point_time <= previous_time:
                raise RuntimeError('실행 trajectory의 크기 또는 시간이 올바르지 않습니다.')
            pair = {side: list(values) for side, values in reference_pair.items()}
            pair[arm] = values
            self._validate_command_pair(pair)
            if previous is not None:
                maximum_step = max(
                    abs(value - old) for value, old in zip(values, previous))
                if maximum_step > self.max_joint_step_rad + 1.0e-9:
                    raise RuntimeError(
                        f'trajectory 내부 관절 간격 {maximum_step:.6f} rad가 '
                        '제한을 넘었습니다.')
            previous = values
            previous_time = point_time

    def _start_waypoint_execution(
        self, arm: str, slow_start: bool = False
    ) -> dict[str, Any]:
        if not self.publish_enabled:
            raise RuntimeError('publish_enabled=false이므로 Preview만 가능합니다.')
        if self._keyboard_is_active():
            raise RuntimeError(
                '키보드 제어 중에는 trajectory를 실행할 수 없습니다. '
                '먼저 리더 TELEOP 또는 정지로 전환해 주세요.')
        with self.execution_lock:
            if self.execution_thread is not None and self.execution_thread.is_alive():
                raise RuntimeError('다른 waypoint trajectory가 실행 중입니다.')
            self.execution_stop.clear()
            return_mode = self._arbiter_mode()
            hold, gripper_actions = self._keyboard_handoff_positions(
                active_arms={arm})
            self._publish_waypoint_pair(hold)
            time.sleep(0.03)
            self._publish_waypoint_pair(hold)
            try:
                # Worker discovery may pause DDS publishers. Release TELEOP
                # before starting it, then select WAYPOINT only after both
                # hold inputs have been acknowledged by the arbiter.
                self._call_arbiter('select_none')
                self._open_stream_session(hold)
                if self.execution_stop.is_set():
                    raise InterruptedError('전송 준비 중 정지가 요청되어 실행을 취소했습니다.')
                self._call_arbiter('select_waypoint')
                hold = self._wait_until_stationary(gripper_actions)
                keepalive_stop = threading.Event()
                keepalive = threading.Thread(
                    target=self._hold_waypoint_lease,
                    args=(hold, keepalive_stop),
                    name='waypoint-handoff-keepalive',
                    daemon=True,
                )
                keepalive.start()
                try:
                    trajectory = self._fresh_plan(arm, slow_start)
                    self._validate_plan(arm, trajectory, hold[arm])
                    if self.execution_stop.is_set():
                        raise InterruptedError('경로 계산 중 정지가 요청되어 실행을 취소했습니다.')
                finally:
                    keepalive_stop.set()
                    keepalive.join(timeout=0.5)
            except Exception:
                self._call_arbiter('select_none')
                self._close_stream_session()
                if return_mode == 'TELEOP' and not self.execution_stop.is_set():
                    self._call_arbiter('select_teleop')
                raise
            self.execution_arm = arm
            self.execution_thread = threading.Thread(
                target=self._run_waypoint_trajectory,
                args=(arm, trajectory, hold, return_mode),
                name=f'waypoint-{arm}-trajectory',
                daemon=True,
            )
            self.execution_thread.start()
        duration = self._trajectory_time(trajectory.points[-1])
        execution_name = 'SLOW START WAYPOINT' if slow_start else 'WAYPOINT'
        self._set_status(arm, f'EXECUTING: {execution_name} {duration:.3f} s')
        return {
            'success': True,
            'message': f'{execution_name} trajectory 실행 시작: {duration:.3f}초',
        }

    def _hold_waypoint_lease(
        self, hold: dict[str, list[float]], stop_event: threading.Event
    ) -> None:
        while not stop_event.is_set() and not self.execution_stop.is_set():
            self._publish_waypoint_pair(hold)
            stop_event.wait(0.02)

    def _open_stream_session(self, initial_pair: dict[str, list[float]]) -> None:
        if not getattr(self, 'isolated_streaming_enabled', False):
            return
        if getattr(self, '_stream_session', None) is not None:
            raise RuntimeError('독립 전송 세션이 이미 열려 있습니다.')
        worker = Path(__file__).with_name('waypoint_stream_worker')
        if not worker.exists():
            worker = worker.with_suffix('.py')
        process = subprocess.Popen(
            [sys.executable, str(worker), str(Path(__file__).resolve())],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1,
        )
        messages = queue.Queue()

        def receive():
            try:
                for line in process.stdout:
                    messages.put(json.loads(line))
            except (ValueError, OSError) as exception:
                messages.put({'type': 'error', 'message': str(exception)})
            finally:
                messages.put({'type': 'closed'})

        reader = threading.Thread(target=receive, daemon=True)
        self._stream_session = (process, messages, reader)
        reader.start()
        self._send_stream_message({
            'type': 'init', 'initial_pair': initial_pair,
            'arbiter_namespace': self.command_arbiter_namespace,
            'control_rate_hz': self.control_rate_hz,
            'max_joint_step_rad': self.max_joint_step_rad,
            'max_schedule_lag_s': self.max_schedule_lag_s,
        })
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            try:
                message = messages.get(timeout=0.02)
            except queue.Empty:
                continue
            if message['type'] == 'ready':
                return
            if message['type'] in {'error', 'closed'}:
                raise RuntimeError(message.get('message', '독립 전송 프로세스가 종료되었습니다.'))
        raise RuntimeError('독립 trajectory 전송 프로세스 준비 시간이 초과되었습니다.')

    def _send_stream_message(self, payload: dict[str, Any]) -> None:
        process = self._stream_session[0]
        process.stdin.write(json.dumps(payload, allow_nan=False) + '\n')
        process.stdin.flush()

    def _close_stream_session(self) -> None:
        session = getattr(self, '_stream_session', None)
        if session is None:
            return
        self._stream_session = None
        process, _messages, reader = session
        try:
            process.stdin.close()  # EOF stops the publisher even if the UI dies.
        except (BrokenPipeError, OSError):
            pass
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        reader.join(timeout=1.0)
        process.stdout.close()
        process.stderr.close()

    def _stream_trajectory(
        self,
        arm: str,
        trajectory: JointTrajectory,
        initial_pair: dict[str, list[float]],
    ) -> dict[str, list[float]]:
        if not getattr(self, 'isolated_streaming_enabled', False):
            return self._stream_trajectory_local(arm, trajectory, initial_pair)
        if getattr(self, '_stream_session', None) is None:
            raise RuntimeError('독립 trajectory 전송 세션이 없습니다.')
        _process, messages, _reader = self._stream_session
        self._send_stream_message({
            'type': 'plan', 'arm': arm, 'initial_pair': initial_pair,
            'points': [[self._trajectory_time(point), list(point.positions)]
                       for point in trajectory.points],
        })
        deadline = time.monotonic() + 15.0
        stop_sent = False
        while True:
            if self.execution_stop.is_set() and not stop_sent:
                self._send_stream_message({'type': 'stop'})
                stop_sent = True
                deadline = time.monotonic() + 1.0
            try:
                message = messages.get(timeout=0.02)
            except queue.Empty:
                if time.monotonic() > deadline:
                    raise RuntimeError('독립 trajectory 전송 프로세스 응답 시간이 초과되었습니다.')
                continue
            kind = message['type']
            if 'progress' in message:
                progress = message['progress']
                self._record_stream_progress(
                    arm, progress['phase'], progress['sample_index'],
                    progress['sample_count'], progress['planned_time_s'])
            if kind == 'prepared':
                if stop_sent or self.execution_stop.is_set():
                    raise InterruptedError('사용자가 waypoint 실행을 중지했습니다.')
                self._send_stream_message({'type': 'go'})
                deadline = time.monotonic() + max(
                    30.0, self._trajectory_time(trajectory.points[-1]) * 4 + 10)
            elif kind == 'done':
                if stop_sent or self.execution_stop.is_set():
                    raise InterruptedError('사용자가 waypoint 실행을 중지했습니다.')
                return message['final_pair']
            elif kind in {'error', 'closed'}:
                if stop_sent or self.execution_stop.is_set():
                    raise InterruptedError('사용자가 waypoint 실행을 중지했습니다.')
                raise RuntimeError(message.get('message', '독립 전송 프로세스가 종료되었습니다.'))

    def _stream_trajectory_local(
        self,
        arm: str,
        trajectory: JointTrajectory,
        initial_pair: dict[str, list[float]],
    ) -> dict[str, list[float]]:
        final_pair = {side: list(values) for side, values in initial_pair.items()}
        transition = smooth_joint_transition(
            final_pair[arm], list(trajectory.points[0].positions),
            self.max_joint_step_rad,
        )
        transition_start = time.monotonic()
        transition_shift = 0.0
        for index, positions in enumerate(transition, start=1):
            self._record_stream_progress(
                arm, 'start_transition', index, len(transition), index / self.control_rate_hz)
            if self.execution_stop.is_set():
                raise InterruptedError('사용자가 waypoint 실행을 중지했습니다.')
            with self.lock:
                arbiter_mode = self.arbiter_mode
                arbiter_fault = self.arbiter_fault
            if arbiter_mode != 'WAYPOINT' or arbiter_fault:
                raise RuntimeError(
                    f'WAYPOINT 제어권이 해제되었습니다: '
                    f'mode={arbiter_mode}, fault={arbiter_fault or "none"}'
                )
            target_time = transition_start + transition_shift + index / self.control_rate_hz
            while True:
                remaining = target_time - time.monotonic()
                if remaining <= 0.0:
                    break
                if self.execution_stop.wait(min(remaining, 0.02)):
                    raise InterruptedError('사용자가 waypoint 실행을 중지했습니다.')
            schedule_lag = time.monotonic() - target_time
            if schedule_lag > self.max_schedule_lag_s:
                raise RuntimeError(
                    f'trajectory 시작 연결 publish 지연 {schedule_lag:.3f}초가 '
                    '제한을 넘었습니다.')
            final_pair[arm] = positions
            self._publish_waypoint_pair(final_pair)
            # Preserve every bounded command, and move the remaining deadlines
            # by time already spent late. Catch-up bursts can exhaust the
            # scheduler budget on long, densely sampled trajectories.
            transition_shift += max(0.0, time.monotonic() - target_time)

        start_time = time.monotonic()
        schedule_shift = 0.0
        for sample_index, point in enumerate(trajectory.points, start=1):
            self._record_stream_progress(
                arm, 'trajectory', sample_index, len(trajectory.points),
                self._trajectory_time(point))
            if self.execution_stop.is_set():
                raise InterruptedError('사용자가 waypoint 실행을 중지했습니다.')
            with self.lock:
                arbiter_mode = self.arbiter_mode
                arbiter_fault = self.arbiter_fault
            if arbiter_mode != 'WAYPOINT' or arbiter_fault:
                raise RuntimeError(
                    f'WAYPOINT 제어권이 해제되었습니다: '
                    f'mode={arbiter_mode}, fault={arbiter_fault or "none"}'
                )
            target_time = start_time + schedule_shift + self._trajectory_time(point)
            while True:
                remaining = target_time - time.monotonic()
                if remaining <= 0.0:
                    break
                if self.execution_stop.wait(min(remaining, 0.02)):
                    raise InterruptedError('사용자가 waypoint 실행을 중지했습니다.')
            schedule_lag = time.monotonic() - target_time
            if schedule_lag > self.max_schedule_lag_s:
                raise RuntimeError(
                    f'trajectory publish 지연 {schedule_lag:.3f}초가 제한을 넘었습니다.')
            final_pair[arm] = list(point.positions)
            self._publish_waypoint_pair(final_pair)
            schedule_shift += max(0.0, time.monotonic() - target_time)
        return final_pair

    def _wait_for_arm_target(
        self, arm: str, final_pair: dict[str, list[float]]
    ) -> None:
        deadline = time.monotonic() + self.completion_timeout_s
        arm_names = arm_joint_names(arm)
        worst_arm_name = arm_names[0]
        worst_arm_error = math.inf
        while time.monotonic() < deadline:
            if self.execution_stop.is_set():
                raise InterruptedError('사용자가 waypoint 실행을 중지했습니다.')
            current = self._current_command_positions()[arm]
            target = final_pair[arm]
            arm_errors = [
                abs(value - expected)
                for value, expected in zip(current[:len(arm_names)], target[:len(arm_names)])
            ]
            worst_arm_index = max(range(len(arm_errors)), key=arm_errors.__getitem__)
            worst_arm_name = arm_names[worst_arm_index]
            worst_arm_error = arm_errors[worst_arm_index]
            with self.lock:
                velocities = [
                    self.all_current_velocities.get(name)
                    for name in arm_names
                ]
            stopped = all(
                velocity is not None
                and abs(velocity) <= self.max_start_velocity_radps
                for velocity in velocities
            )
            if (
                worst_arm_error <= self.completion_tolerance_rad
                and stopped
            ):
                return
            self._publish_waypoint_pair(final_pair)
            time.sleep(0.02)
        raise RuntimeError(
            'trajectory 종료 자세가 허용 오차에 도달하지 못했습니다: '
            f'{worst_arm_name}={worst_arm_error:.4f} rad '
            f'(팔 허용 {self.completion_tolerance_rad:.4f})'
        )

    def _complete_execution(self, return_mode: str) -> None:
        if return_mode == 'TELEOP':
            self._select_relative_teleop_route()
        else:
            self._call_arbiter('select_none')

    def _run_waypoint_trajectory(
        self,
        arm: str,
        trajectory: JointTrajectory,
        hold: dict[str, list[float]],
        return_mode: str,
    ) -> None:
        try:
            final_pair = self._stream_trajectory(arm, trajectory, hold)
            self._wait_for_arm_target(arm, final_pair)
            self._complete_execution(return_mode)
            self._set_status(arm, 'EXECUTION_COMPLETE: teleop handoff ready')
        except InterruptedError as exception:
            try:
                self._call_arbiter('select_none')
            finally:
                self._set_status(arm, f'STOPPED: {exception}')
        except Exception as exception:
            try:
                self._call_arbiter('select_none')
            finally:
                self._set_status(arm, f'ERROR: {exception}')
        finally:
            self._close_stream_session()
            with self.execution_lock:
                self.execution_arm = None

    def start_execution_sequence(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._start_execution_steps(payload, None)

    def start_execution_step(self, payload: dict[str, Any]) -> dict[str, Any]:
        index = parse_waypoint_index(payload)
        return self._start_execution_steps(payload, index)

    def _start_execution_steps(
        self, payload: dict[str, Any], selected_index: int | None
    ) -> dict[str, Any]:
        if payload.get('confirm') != 'EXECUTE':
            raise ValueError('실행 확인이 필요합니다.')
        if not self.publish_enabled:
            raise RuntimeError('publish_enabled=false이므로 Preview만 가능합니다.')
        if self._keyboard_is_active():
            raise RuntimeError(
                '키보드 제어 중에는 순차 실행을 시작할 수 없습니다. '
                '먼저 리더 TELEOP 또는 정지로 전환해 주세요.')
        with self.execution_lock:
            if self.execution_thread is not None and self.execution_thread.is_alive():
                raise RuntimeError('다른 waypoint trajectory가 실행 중입니다.')
            with self.lock:
                if selected_index is None:
                    steps = [dict(step) for step in self.execution_sequence]
                else:
                    if selected_index > len(self.execution_sequence):
                        raise ValueError('실행할 단계가 순차 실행 목록 밖입니다.')
                    steps = [dict(self.execution_sequence[selected_index - 1])]
                for step in steps:
                    if max(step['start_index'], step['end_index']) > self.arm_states[
                        step['arm']
                    ]['waypoint_count']:
                        raise ValueError('실행 목록에 현재 waypoint 범위를 벗어난 구간이 있습니다.')
            if not steps:
                raise ValueError('먼저 순차 실행 구간을 추가해 주세요.')
            self.execution_stop.clear()
            return_mode = self._arbiter_mode()
            first_step = steps[0]
            hold, gripper_actions = self._keyboard_handoff_positions(
                active_arms={first_step['arm']},
                active_start_waypoints={
                    first_step['arm']: first_step['start_index'],
                },
            )
            self._publish_waypoint_pair(hold)
            time.sleep(0.03)
            self._publish_waypoint_pair(hold)
            try:
                self._call_arbiter('select_none')
                self._open_stream_session(hold)
                if self.execution_stop.is_set():
                    raise InterruptedError('전송 준비 중 정지가 요청되어 실행을 취소했습니다.')
                self._call_arbiter('select_waypoint')
                hold = self._wait_until_stationary(gripper_actions)
                if self.execution_stop.is_set():
                    raise InterruptedError('순차 실행 준비 중 정지가 요청되어 실행을 취소했습니다.')
            except Exception:
                self._call_arbiter('select_none')
                self._close_stream_session()
                if return_mode == 'TELEOP' and not self.execution_stop.is_set():
                    self._call_arbiter('select_teleop')
                raise
            self.execution_arm = 'sequence'
            self.execution_thread = threading.Thread(
                target=self._run_execution_sequence,
                args=(steps, hold),
                name='waypoint-arm-sequence',
                daemon=True,
            )
            self.execution_thread.start()
        return {
            'success': True,
            'message': (
                f'순차 실행 {len(steps)}단계를 시작했습니다.'
                if selected_index is None
                else f'순차 실행 목록의 {selected_index}번 단계만 시작했습니다.'
            ),
        }

    def _run_execution_sequence(
        self,
        steps: list[dict[str, Any]],
        hold: dict[str, list[float]],
    ) -> None:
        current_pair = {side: list(values) for side, values in hold.items()}
        active_arm = steps[0]['arm']
        try:
            for number, step in enumerate(steps, start=1):
                active_arm = step['arm']
                self._set_status(
                    active_arm,
                    f'EXECUTING_SEQUENCE: step={number}/{len(steps)}, '
                    f'range={step["start_index"]}-{step["end_index"]}',
                )
                keepalive_stop = threading.Event()
                keepalive = threading.Thread(
                    target=self._hold_waypoint_lease,
                    args=(current_pair, keepalive_stop),
                    name=f'waypoint-sequence-plan-{number}',
                    daemon=True,
                )
                keepalive.start()
                try:
                    self._select_start_waypoint(active_arm, step['start_index'])
                    self._select_end_waypoint(active_arm, step['end_index'])
                    trajectory = self._fresh_plan(active_arm)
                    self._validate_plan(active_arm, trajectory, current_pair[active_arm])
                finally:
                    keepalive_stop.set()
                    keepalive.join(timeout=0.5)
                current_pair = self._stream_trajectory(
                    active_arm, trajectory, current_pair)
                self._wait_for_arm_target(active_arm, current_pair)
            # A completed sequence prepares the relative Mini leader service,
            # waits for fresh paired input, and then hands control to TELEOP.
            # Any failure closes the command route through the exception path.
            self._complete_execution('TELEOP')
            for arm in {step['arm'] for step in steps}:
                self._set_status(arm, 'EXECUTION_SEQUENCE_COMPLETE: teleop handoff ready')
        except InterruptedError as exception:
            try:
                self._call_arbiter('select_none')
            finally:
                self._set_status(active_arm, f'STOPPED: {exception}')
        except Exception as exception:
            try:
                self._call_arbiter('select_none')
            finally:
                self._set_status(active_arm, f'ERROR: {exception}')
        finally:
            self._close_stream_session()
            with self.execution_lock:
                self.execution_arm = None

    def _publish_planner_setting(
        self, arm: str, publisher: Any, message: Any, expected_status: str
    ) -> str:
        # A service can be visible before DDS matches the settings topic. Wait
        # before publishing so the first import after startup is not dropped.
        deadline = time.monotonic() + 2.0
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError('설정을 받을 waypoint 노드가 연결되지 않았습니다.')
            time.sleep(0.01)
        with self.lock:
            sequence = self.arm_states[arm]['status_sequence']
        publisher.publish(message)
        return self._wait_for_status(arm, sequence, expected_prefix=expected_status)

    def _wait_for_status(
        self, arm: str, previous_sequence: int, timeout_s: float = 2.0,
        expected_prefix: str | None = None,
    ) -> str:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self.lock:
                arm_state = self.arm_states[arm]
                status = arm_state['status']
                if (arm_state['status_sequence'] > previous_sequence and
                        (expected_prefix is None or
                         status.startswith((expected_prefix, 'ERROR:')))):
                    return status
            time.sleep(0.01)
        return ''

    def shutdown_execution(self) -> None:
        self.execution_stop.set()
        keyboard_was_active = self._deactivate_keyboard()
        self.keyboard_shutdown.set()
        thread = self.execution_thread
        if (thread is not None and thread.is_alive()) or keyboard_was_active:
            try:
                self._call_arbiter('select_none')
            except RuntimeError as exception:
                self.get_logger().error(
                    f'종료 중 WAYPOINT command 차단에 실패했습니다: {exception}')
        if self.keyboard_thread.is_alive():
            self.keyboard_thread.join(timeout=1.0)
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)


class ApiHandler(BaseHTTPRequestHandler):

    bridge: WaypointUiBridge
    static_root: Path
    robot_asset_roots: dict[str, Path]

    def do_GET(self) -> None:
        try:
            request = urlsplit(self.path)
            arm = parse_qs(request.query).get('arm', [None])[0]
            if request.path == '/api/runtime':
                self._json_response(
                    HTTPStatus.OK, self.bridge.runtime_snapshot(arm))
                return
            if request.path == '/api/state':
                self._json_response(HTTPStatus.OK, self.bridge.snapshot(arm))
                return
            if request.path == '/api/waypoints/export':
                self._json_response(
                    HTTPStatus.OK, self.bridge.export_waypoints(arm))
                return
            if request.path == '/api/robot-model':
                self._json_response(
                    HTTPStatus.OK, self.bridge.robot_model_snapshot(arm))
                return
            if request.path == '/api/robot-description.urdf':
                description = self.bridge.robot_description_snapshot()
                if not description:
                    self._json_response(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {'error': '/robot_description 대기 중'},
                    )
                    return
                self._bytes_response(
                    HTTPStatus.OK,
                    description.encode('utf-8'),
                    'application/xml; charset=utf-8',
                )
                return
            if request.path.startswith('/urdf/'):
                self._serve_robot_asset(request.path)
                return
            static_files = {
                '/': ('index.html', 'text/html; charset=utf-8'),
                '/generator': ('generator.html', 'text/html; charset=utf-8'),
                '/generator.html': ('generator.html', 'text/html; charset=utf-8'),
                '/generator.js': ('generator.js', 'text/javascript; charset=utf-8'),
                '/generator.css': ('generator.css', 'text/css; charset=utf-8'),
                '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                '/full_robot_simulator.bundle.js': (
                    'full_robot_simulator.bundle.js',
                    'text/javascript; charset=utf-8',
                ),
                '/styles.css': ('styles.css', 'text/css; charset=utf-8'),
            }
            selected = static_files.get(request.path)
            if selected is None:
                self._json_response(
                    HTTPStatus.NOT_FOUND, {'error': '찾을 수 없습니다.'})
                return
            path = self.static_root / selected[0]
            content = path.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', selected[1])
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(content)
        except ValueError as exception:
            self._json_response(HTTPStatus.BAD_REQUEST, {'error': str(exception)})

    def _serve_robot_asset(self, request_path: str) -> None:
        relative_request = unquote(request_path.removeprefix('/urdf/'))
        package_name, separator, relative_path = relative_request.partition('/')
        package_root = self.robot_asset_roots.get(package_name)
        if not separator or package_root is None or not relative_path:
            self._json_response(
                HTTPStatus.NOT_FOUND, {'error': '로봇 메쉬를 찾을 수 없습니다.'})
            return

        relative_asset = Path(relative_path)
        if relative_asset.is_absolute() or '..' in relative_asset.parts:
            self._json_response(
                HTTPStatus.BAD_REQUEST, {'error': '잘못된 로봇 메쉬 경로입니다.'})
            return
        asset_path = package_root / relative_asset
        if not asset_path.is_file():
            self._json_response(
                HTTPStatus.NOT_FOUND, {'error': '로봇 메쉬를 찾을 수 없습니다.'})
            return

        content_types = {
            '.stl': 'model/stl',
            '.dae': 'model/vnd.collada+xml',
            '.png': 'image/png',
            '.jpg': 'image/jpeg',
            '.jpeg': 'image/jpeg',
        }
        file_size = asset_path.stat().st_size
        self.send_response(HTTPStatus.OK)
        self.send_header(
            'Content-Type', content_types.get(
                asset_path.suffix.lower(), 'application/octet-stream'))
        self.send_header('Content-Length', str(file_size))
        self.send_header('Cache-Control', 'public, max-age=3600')
        self.end_headers()
        with asset_path.open('rb') as asset_file:
            while chunk := asset_file.read(1024 * 1024):
                self.wfile.write(chunk)

    def do_POST(self) -> None:
        try:
            request_path = urlsplit(self.path).path
            payload = self._read_json()
            if request_path == '/api/waypoints/joint':
                result = self.bridge.add_joint_waypoint(payload)
            elif request_path == '/api/waypoints/absolute':
                result = self.bridge.add_absolute_waypoint(payload)
            elif request_path == '/api/waypoints/relative':
                result = self.bridge.add_relative_waypoint(payload)
            elif request_path == '/api/waypoints/frame-relative':
                result = self.bridge.add_frame_relative_waypoint(payload)
            elif request_path == '/api/waypoints/dwell':
                result = self.bridge.update_waypoint_dwell(payload)
            elif request_path == '/api/waypoints/speed':
                result = self.bridge.update_waypoint_speed(payload)
            elif request_path == '/api/waypoints/equal-time':
                result = self.bridge.equalize_segment_times(payload)
            elif request_path == '/api/waypoints/mode':
                result = self.bridge.update_waypoint_mode(payload)
            elif request_path == '/api/joint-smoothing':
                result = self.bridge.set_joint_smoothing(payload)
            elif request_path == '/api/path-refinement':
                result = self.bridge.set_path_refinement(payload)
            elif request_path == '/api/waypoints/start':
                result = self.bridge.select_start_waypoint(payload)
            elif request_path == '/api/waypoints/end':
                result = self.bridge.select_end_waypoint(payload)
            elif request_path == '/api/waypoints/name':
                result = self.bridge.rename_waypoint(payload)
            elif request_path == '/api/waypoints/reorder':
                result = self.bridge.reorder_waypoint(payload)
            elif request_path == '/api/waypoints/import':
                result = self.bridge.import_waypoints(payload)
            elif request_path == '/api/generator/tool-path':
                result = self.bridge.generate_tool_path(payload)
            elif request_path == '/api/waypoints/remove':
                result = self.bridge.remove_waypoint(payload)
            elif request_path == '/api/sequence/add':
                result = self.bridge.add_execution_step(payload)
            elif request_path == '/api/sequence/remove':
                result = self.bridge.remove_execution_step(payload)
            elif request_path == '/api/sequence/reorder':
                result = self.bridge.reorder_execution_step(payload)
            elif request_path == '/api/sequence/clear':
                result = self.bridge.clear_execution_sequence()
            elif request_path == '/api/sequence/execute':
                result = self.bridge.start_execution_sequence(payload)
            elif request_path == '/api/sequence/execute-one':
                result = self.bridge.start_execution_step(payload)
            elif request_path == '/api/keyboard/start':
                result = self.bridge.start_keyboard_control(payload)
            elif request_path == '/api/keyboard/jog':
                result = self.bridge.keyboard_jog(payload)
            elif request_path == '/api/keyboard/heartbeat':
                result = self.bridge.keyboard_heartbeat()
            elif request_path.startswith('/api/actions/'):
                action = request_path.removeprefix('/api/actions/')
                if action == 'stop':
                    client_ip = self.client_address[0]
                    confirmed = (
                        payload.get('confirm') == STOP_BUTTON_CONFIRMATION)
                    self.bridge.get_logger().warning(
                        'STOP API 요청 수신: '
                        f'client={client_ip}, button_confirmed={confirmed}')
                result = self.bridge.call_action(action, payload)
                if action == 'stop':
                    self.bridge.get_logger().warning(
                        'STOP API 요청 처리 완료: '
                        f'client={self.client_address[0]}')
            else:
                self._json_response(
                    HTTPStatus.NOT_FOUND, {'error': '찾을 수 없습니다.'})
                return
            code = HTTPStatus.OK if result['success'] else HTTPStatus.CONFLICT
            self._json_response(code, result)
        except (ValueError, UnicodeDecodeError, json.JSONDecodeError) as exception:
            self._json_response(HTTPStatus.BAD_REQUEST, {'error': str(exception)})
        except RuntimeError as exception:
            self._json_response(
                HTTPStatus.SERVICE_UNAVAILABLE, {'error': str(exception)})

    def _read_json(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get('Content-Length', '0'))
        except ValueError as exception:
            raise ValueError('잘못된 Content-Length입니다.') from exception
        if length > 16_777_216:
            raise ValueError('요청 본문이 너무 큽니다.')
        raw = self.rfile.read(length)
        if not raw:
            return {}
        payload = json.loads(raw.decode('utf-8'))
        if not isinstance(payload, dict):
            raise ValueError('JSON 객체가 필요합니다.')
        return payload

    def _json_response(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        content = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self._bytes_response(
            status, content, 'application/json; charset=utf-8')

    def _bytes_response(
        self, status: HTTPStatus, content: bytes, content_type: str
    ) -> None:
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(content)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format_string: str, *args: Any) -> None:
        self.bridge.get_logger().debug(format_string % args)


class WaypointHttpServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main(args=None) -> None:
    rclpy.init(args=args)
    bridge = WaypointUiBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    server_holder = {'server': None}

    def spin() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
        finally:
            server = server_holder['server']
            if server is not None:
                server.shutdown()

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    static_root = (
        Path(get_package_share_directory('waypoint_telep')) / 'web'
    )
    robot_asset_roots = {}
    for package_name in (
        'ffw_description', 'open_manipulator_description',
        'realsense2_description',
    ):
        try:
            robot_asset_roots[package_name] = Path(
                get_package_share_directory(package_name))
        except LookupError:
            bridge.get_logger().warning(
                f'{package_name} 패키지가 없어 해당 3D 메쉬는 생략합니다.')
    handler = type(
        'WaypointApiHandler',
        (ApiHandler,),
        {
            'bridge': bridge,
            'static_root': static_root,
            'robot_asset_roots': robot_asset_roots,
        },
    )
    server = None
    startup_error = None
    try:
        server = WaypointHttpServer((bridge.host, bridge.port), handler)
        server_holder['server'] = server
        urls = browser_urls(bridge.host, bridge.port, bridge.public_host)
        bridge.get_logger().info('Waypoint Telep UI가 준비되었습니다.')
        for url in urls:
            bridge.get_logger().info(f'UI 바로 열기: {url}')
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    except OSError as exception:
        startup_error = exception
        bridge.get_logger().error(
            f'UI 주소 {bridge.host}:{bridge.port}를 열 수 없습니다: {exception}. '
            '같은 포트를 사용하는 기존 waypoint UI를 먼저 종료하십시오.'
        )
    finally:
        if server is not None:
            server.server_close()
        bridge.shutdown_execution()
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if startup_error is not None:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
