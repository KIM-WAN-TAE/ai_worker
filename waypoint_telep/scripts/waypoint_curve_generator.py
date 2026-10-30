#!/usr/bin/env python3
"""Generate selected Cartesian segments; validate on an isolated, disabled planner.

This worker never connects to the running robot's ROS domain. Input and output
are JSON over stdio; geometry helpers are also usable without initializing ROS.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np
import PyKDL as K


def integer(value, label, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f'{label}: {minimum}~{maximum} 범위의 정수가 필요합니다.')
    return value


def number(value, label, minimum, maximum):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not minimum <= value <= maximum):
        raise ValueError(f'{label}: {minimum}~{maximum} 범위의 숫자가 필요합니다.')
    return float(value)


def array(value, count, label):
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f'{label}: {count}개 값이 필요합니다.')
    return np.array([number(v, label, -1e6, 1e6) for v in value])


def joint_array(values):
    result = K.JntArray(len(values))
    for i, value in enumerate(values):
        result[i] = float(value)
    return result


def quaternion(values):
    q = array(values, 4, '자세')
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError('자세 quaternion은 0일 수 없습니다.')
    return q / norm


def slerp(a, b, u):
    a, b = quaternion(list(a)), quaternion(list(b))
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > .9995:
        q = a + u * (b - a)
        return q / np.linalg.norm(q)
    theta = math.acos(min(1., dot))
    return (math.sin((1-u)*theta)*a + math.sin(u*theta)*b) / math.sin(theta)


def parse_start_noise(value):
    if value is None:
        return dict(enabled=False)
    if not isinstance(value, dict) or not isinstance(value.get('enabled', False), bool):
        raise ValueError('시작점 노이즈 설정이 올바르지 않습니다.')
    if not value.get('enabled', False):
        return dict(enabled=False)
    mode, sampling = value.get('mode', 'position'), value.get('sampling', 'manual')
    if mode not in ('position', 'orientation', 'both') or sampling not in ('manual', 'random'):
        raise ValueError('시작점 노이즈 종류 또는 생성 방식을 확인해 주세요.')
    result = dict(enabled=True, mode=mode, sampling=sampling)
    for kind, unit, bound, active in [('translation', 'mm', 50, mode in ('position', 'both')),
                                    ('rotation', 'deg', 30, mode in ('orientation', 'both'))]:
        lower_key, upper_key = f'{kind}_min_{unit}', f'{kind}_max_{unit}'
        manual_key = 'translation_mm' if kind == 'translation' else 'rotation_rpy_deg'
        for key, default in [(lower_key, [-5.]*3), (upper_key, [5.]*3), (manual_key, [0.]*3)]:
            values = value.get(key, default)
            if not isinstance(values, list) or len(values) != 3:
                raise ValueError('시작점 노이즈는 축별로 세 개의 값을 지정해 주세요.')
            result[key] = [number(v, key, -bound, bound) for v in values] if active else [0.]*3
        for i in range(3):
            if result[lower_key][i] > result[upper_key][i]:
                raise ValueError('시작점 노이즈의 최소값은 최대값보다 클 수 없습니다.')
            if sampling == 'manual' and not result[lower_key][i] <= result[manual_key][i] <= result[upper_key][i]:
                raise ValueError('시작점 슬라이더 값은 지정한 최소·최대 범위 안에 있어야 합니다.')
    return result


def sample_start_noise(settings, seed, variant=0):
    """Identical LCG in the browser: changing sample count keeps sample #1 fixed."""
    if not settings['enabled']:
        return dict(translation_mm=[0.]*3, rotation_rpy_deg=[0.]*3)
    state = (seed + 0x9e3779b9*(variant+1)) % 2**32
    result = {}
    for kind, unit, key in [('translation', 'mm', 'translation_mm'),
                            ('rotation', 'deg', 'rotation_rpy_deg')]:
        result[key] = []
        for lower, upper, manual in zip(settings[f'{kind}_min_{unit}'],
                                        settings[f'{kind}_max_{unit}'], settings[key]):
            state = (state*1664525+1013904223) % 2**32
            result[key].append(lower+(upper-lower)*(state+.5)/2**32
                               if settings['sampling'] == 'random' else manual)
    return result


def noisy_start_pose(anchor, sample):
    position = np.array(anchor['position_m']) + np.array(sample['translation_mm'])/1000
    reference = K.Rotation.Quaternion(*quaternion(anchor['orientation_xyzw']))
    rotation = reference * K.Rotation.RPY(*np.radians(sample['rotation_rpy_deg']))
    return position, list(rotation.GetQuaternion())


def curve_position(a, b, c, u, strength, width, angle):
    """Bounded bow with optional tangent alignment; strength zero is exactly linear.

    The quintic window has zero offset, velocity and acceleration at both ends.
    A small quintic Hermite correction guides the curve towards the insertion
    direction. Both corrections are scaled together, so the requested width is
    a bound on the total departure from the A--B straight reference.
    """
    d = b - a
    direction = d / np.linalg.norm(d)
    axis = np.eye(3)[int(np.argmin(abs(direction)))]
    e1 = np.cross(direction, axis)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(direction, e1)
    lateral = math.cos(angle)*e1 + math.sin(angle)*e2
    tail = (c-b) / np.linalg.norm(c-b)
    tangent = max(0., float(d @ tail)) * tail

    def displacement(t):
        hermite = -4*t**3 + 7*t**4 - 3*t**5
        return hermite*(tangent-d) + width*64*t**3*(1-t)**3*lateral

    # One conservative normalization for the entire curve, never per point.
    peak = max(np.linalg.norm(displacement(t)) for t in np.linspace(0, 1, 129))
    scale = min(1., width / max(peak * 1.001, 1e-12))
    return a + u*d + strength*scale*displacement(u)


class Kinematics:
    def __init__(self, model, limits):
        chain = K.Chain()
        self.names = []
        lower, upper = [], []
        for j in model['chain']:
            frame = K.Frame(K.Rotation.RPY(*j['origin_rpy']), K.Vector(*j['origin_xyz']))
            if j['type'] == 'fixed':
                joint = K.Joint(j['name'], K.Joint.Fixed)
            else:
                if j['type'] not in ('revolute', 'continuous', 'prismatic'):
                    raise ValueError('지원하지 않는 URDF 관절 종류입니다.')
                joint = K.Joint(j['name'], frame.p, frame.M*K.Vector(*j['axis']),
                                K.Joint.TransAxis if j['type'] == 'prismatic' else K.Joint.RotAxis)
                i = len(self.names)
                self.names.append(j['name'])
                lower.append(max(limits[i][0], j['limit_lower']))
                upper.append(min(limits[i][1], j['limit_upper']))
            chain.addSegment(K.Segment(j['child'], joint, frame))
        if len(self.names) != 7 or any(a >= b for a, b in zip(lower, upper)):
            raise ValueError('7축 팔의 URDF 또는 관절 제한이 올바르지 않습니다.')
        self.lower, self.upper = np.array(lower), np.array(upper)
        self.gripper_limits = limits[7]
        self.chain = chain  # KDL solvers retain references to their chain.
        self.qmin, self.qmax = joint_array(lower), joint_array(upper)
        self.fk_solver = K.ChainFkSolverPos_recursive(self.chain)
        self.vel = K.ChainIkSolverVel_pinv(self.chain)
        self.ik = K.ChainIkSolverPos_NR_JL(self.chain, self.qmin, self.qmax,
                                        self.fk_solver, self.vel, 300, 1e-9)

    def fk(self, q):
        frame = K.Frame()
        if self.fk_solver.JntToCart(joint_array(q[:7]), frame) < 0:
            raise ValueError('FK 계산에 실패했습니다.')
        return frame

    def check(self, q):
        if (not np.all(np.isfinite(q)) or np.any(q[:7] < self.lower-1e-9)
                or np.any(q[:7] > self.upper+1e-9)
                or not self.gripper_limits[0] <= q[7] <= self.gripper_limits[1]):
            raise ValueError('생성된 관절값이 허용 범위 밖입니다.')

    def solve(self, position, rotation, seed, gripper):
        target = K.Frame(K.Rotation.Quaternion(*rotation), K.Vector(*position))
        result = K.JntArray(7)
        if self.ik.CartToJnt(joint_array(seed[:7]), target, result) < 0:
            raise ValueError('곡선 경로의 IK가 수렴하지 않습니다. 곡선 정도를 낮춰 주세요.')
        q = np.array([result[i] for i in range(7)] + [gripper])
        self.check(q)
        error = K.diff(self.fk(q), target)
        if error.vel.Norm() > 1e-6 or error.rot.Norm() > 1e-6:
            raise ValueError('곡선 경로의 IK 오차가 허용 범위를 넘었습니다.')
        return q


def generate(config):
    payload = config['payload']
    source = payload.get('waypoint_file')
    if not isinstance(source, dict) or source.get('schema') != 'ai_worker_waypoints/v1':
        raise ValueError('기준 waypoint JSON이 필요합니다.')
    ws = source.get('waypoints')
    if not isinstance(ws, list) or not 2 <= len(ws) <= 10000:
        raise ValueError('기준 경로에는 2개 이상, 10000개 이하의 지점이 필요합니다.')
    if source.get('arm') not in ('left', 'right') or source['arm'] != config['arm']:
        raise ValueError('기준 파일과 로봇 모델의 팔이 일치하지 않습니다.')
    segments = payload.get('segments')
    if segments is None:  # Compatibility for the three-anchor offline example.
        segments = [dict(start_waypoint=payload.get('noise_waypoint'),
                         end_waypoint=payload.get('approach_waypoint'), mode='CURVE',
                         curve_percent=payload.get('curve_percent', 100)),
                    dict(start_waypoint=payload.get('approach_waypoint'),
                         end_waypoint=payload.get('insertion_waypoint'), mode='LINEAR')]
    if not isinstance(segments, list) or not 1 <= len(segments) <= 32:
        raise ValueError('1~32개 구간을 선택해 주세요.')
    parsed = []
    for segment in segments:
        if not isinstance(segment, dict):
            raise ValueError('구간 설정이 올바르지 않습니다.')
        start = integer(segment.get('start_waypoint'), '시작점', 1, len(ws))
        end = integer(segment.get('end_waypoint'), '끝점', 1, len(ws))
        if start == end:
            raise ValueError('구간의 시작점과 끝점은 서로 달라야 합니다.')
        if parsed and start != parsed[-1]['end_waypoint']:
            raise ValueError('다음 구간은 앞 구간의 끝점에서 시작해야 합니다.')
        mode = segment.get('mode', 'CURVE')
        if mode not in ('CURVE', 'LINEAR'):
            raise ValueError('구간 방식은 곡선 또는 직선으로 선택해 주세요.')
        strength = number(segment.get('curve_percent', 100), '곡선 정도', 0, 100) if mode == 'CURVE' else 0.
        first, last = sorted((start, end))
        for internal in ws[first:last-1]:
            if not isinstance(internal, dict):
                raise ValueError('기준 경로의 지점 정보가 올바르지 않습니다.')
            if internal.get('dwell_s', 0):
                raise ValueError('구간 안에 대기 지점이 있습니다. 대기 지점을 구간 끝점으로 나눠 주세요.')
            values = internal.get('joint_positions_rad')
            initial = ws[start-1].get('joint_positions_rad')
            if (isinstance(values, list) and len(values) == 8 and isinstance(initial, list)
                    and len(initial) == 8 and abs(float(values[7])-float(initial[7])) > 1e-6):
                raise ValueError('구간 안에 그리퍼 변경 지점이 있습니다. 해당 지점에서 구간을 나눠 주세요.')
        parsed.append(dict(start_waypoint=start, end_waypoint=end, mode=mode, curve_percent=strength))
    indices = [parsed[0]['start_waypoint']] + [s['end_waypoint'] for s in parsed]
    width = number(payload.get('max_deviation_mm', 3), '최대 곡선 편차', .1, 10) / 1000
    count = integer(payload.get('count', 1), '생성 개수', 1, 30)
    seed = integer(payload.get('seed', 42), '시드', 0, 2**31-1)
    noise_settings = parse_start_noise(payload.get('start_noise'))
    kin = Kinematics(config['model'], config['limits'])
    anchors = [copy.deepcopy(ws[i-1]) for i in indices]
    for anchor in anchors:
        q = array(anchor.get('joint_positions_rad'), 8, '관절값')
        kin.check(q)
        p = array(anchor.get('position_m'), 3, '위치')
        rot = quaternion(anchor.get('orientation_xyzw'))
        frame = K.Frame(K.Rotation.Quaternion(*rot), K.Vector(*p))
        err = K.diff(kin.fk(q), frame)
        if err.vel.Norm() > 2e-5 or err.rot.Norm() > 2e-4:
            raise ValueError('기준 파일의 pose와 관절값이 현재 URDF와 맞지 않습니다.')
        number(anchor.get('dwell_s', 0), '대기시간', 0, 30)
        if abs(float(anchor.get('gripper_rad', q[7])) - q[7]) > 1e-9:
            raise ValueError('기준 파일의 그리퍼 값이 관절값과 다릅니다.')
    base_anchors = copy.deepcopy(anchors)
    positions = [np.array(w['position_m']) for w in anchors]
    if any(np.linalg.norm(b-a) < 1e-5 for a, b in zip(positions, positions[1:])):
        raise ValueError('구간마다 서로 다른 위치의 두 지점을 선택해 주세요.')
    rotations = [quaternion(w['orientation_xyzw']) for w in anchors]
    initial_angle = ((seed*1664525+1013904223) % 2**32) / 2**32 * 2*math.pi
    files = []
    for variant in range(count):
        anchors = copy.deepcopy(base_anchors)
        sample = sample_start_noise(noise_settings, seed, variant)
        if noise_settings['enabled'] and any(abs(v) > 1e-14 for values in sample.values() for v in values):
            position, rotation = noisy_start_pose(anchors[0], sample)
            try:
                q = kin.solve(position, rotation, anchors[0]['joint_positions_rad'],
                              anchors[0]['joint_positions_rad'][7])
            except ValueError as exc:
                raise ValueError(f'{variant+1}번 시작점의 IK 검사에 실패했습니다. 위치·방향 노이즈 범위를 줄여 주세요. {exc}') from exc
            anchors[0].update(position_m=position.tolist(), orientation_xyzw=rotation,
                              joint_positions_rad=q.tolist())
        positions = [np.array(w['position_m']) for w in anchors]
        rotations = [quaternion(w['orientation_xyzw']) for w in anchors]
        if any(np.linalg.norm(b-a) < 1e-5 for a, b in zip(positions, positions[1:])):
            raise ValueError('노이즈 시작점이 다음 지점과 겹칩니다. 위치 노이즈 범위를 줄여 주세요.')
        angle = initial_angle + 2*math.pi*variant/count
        generated = []
        q_previous = np.array(anchors[0]['joint_positions_rad'])
        anchor_indices = [1]
        max_offset = 0.
        segment_offsets = []
        for segment, spec in enumerate(parsed):
            strength = spec['curve_percent']/100
            a, b = positions[segment:segment+2]
            c = positions[segment+2] if segment+2 < len(positions) else b+(b-a)
            curved = spec['mode'] == 'CURVE'
            start, end = anchors[segment:segment+2]
            length = np.linalg.norm(np.array(end['position_m'])-start['position_m'])
            rotation_angle = 2*math.acos(min(1., abs(float(rotations[segment] @ rotations[segment+1]))))
            steps = max(12 if curved else 1,
                        math.ceil((length + (4*width if curved else 0))/.002),
                        math.ceil(rotation_angle / math.radians(1)))
            if steps > 512:
                raise ValueError('선택 구간이 너무 깁니다. 더 가까운 지점을 선택해 주세요.')
            max_offset = 0.
            for j in range(0 if segment == 0 else 1, steps+1):
                u = j / steps
                p = (curve_position(a, b, c, u, strength, width, angle) if curved
                     else a + u*(b-a))
                rot = slerp(rotations[segment], rotations[segment+1], u)
                gripper = float((1-u)*start['joint_positions_rad'][7] + u*end['joint_positions_rad'][7])
                if j in (0, steps):
                    w = copy.deepcopy(start if j == 0 else end)
                    q = np.array(w['joint_positions_rad'])
                else:
                    q = kin.solve(p, rot, q_previous, gripper)
                    w = dict(name=f'구간 {segment+1} {"곡선" if curved else "직선"} 중간점 {j}',
                             position_m=p.tolist(), orientation_xyzw=rot.tolist(),
                             joint_positions_rad=q.tolist(), gripper_rad=gripper, dwell_s=0.)
                w['is_interpolated'] = j not in (0, steps)
                # A different IK branch at an exact anchor must never be hidden.
                if generated and max(abs(q[:7]-q_previous[:7])) > .15:
                    raise ValueError('기준 지점과 IK 관절 해가 불연속입니다. 다른 구간을 선택해 주세요.')
                # Cartesian construction ends at IK. Saved execution points
                # use their verified joint solutions, including straight spans.
                w['interpolation_mode'] = 'JOINT'
                w['speed_scale'] = min(float(config['max_speed_scale']), 5.)
                generated.append(w)
                q_previous = q
                max_offset = max(max_offset, float(np.linalg.norm(p-(a+u*(b-a)))))
            anchor_indices.append(len(generated))
            segment_offsets.append(max_offset*1000)
            if max_offset > width*strength + 1e-8:
                raise ValueError('곡선 편차가 지정 범위를 넘었습니다.')
        if len(generated) > 2048:
            raise ValueError('생성할 중간점이 너무 많습니다. 구간을 나눠 생성해 주세요.')
        preserve = payload.get('preserve_outside', False)
        if not isinstance(preserve, bool):
            raise ValueError('구간 밖 경로 유지 설정은 참 또는 거짓이어야 합니다.')
        if preserve:
            if any(s['start_waypoint'] >= s['end_waypoint'] for s in parsed):
                raise ValueError('전체 경로를 유지할 때는 앞에서 뒤 순서로 구간을 선택해 주세요.')
            prefix = copy.deepcopy(ws[:indices[0]-1])
            suffix = copy.deepcopy(ws[indices[-1]:])
            anchor_indices = [i+len(prefix) for i in anchor_indices]
            generated = prefix+generated+suffix
        output = dict(schema='ai_worker_waypoints/v1', arm=source['arm'],
                      joint_smoothing_enabled=True, start_waypoint=1, end_waypoint=len(generated),
                      path_refinement=dict(enabled=True, automatic_subdivisions=True,
                          subdivisions=10, smoothing_percent=100, position_tolerance_mm=1.,
                          orientation_tolerance_deg=1., joint_tolerance_rad=.01, max_refinements=5),
                      waypoints=generated,
                      generator=dict(algorithm='selected_segments_bounded_curve_and_line/v1',
                          source_sha256=hashlib.sha256(json.dumps(source, sort_keys=True).encode()).hexdigest(),
                          source_anchor_indices=indices, generated_anchor_indices=anchor_indices,
                          segments=parsed, max_deviation_mm=width*1000,
                          sampled_segment_deviation_mm=segment_offsets, variant=variant+1, count=count, seed=seed,
                          output_scope='full_path_with_selected_segments_replaced' if preserve else 'selected_segments',
                          physical_robot_executed=False, live_ui_applied=False))
        if source.get('interpolated_provenance_version') == 2:
            output['interpolated_provenance_version'] = 2
        if noise_settings['enabled']:
            output['generator']['start_noise'] = dict(noise_settings, **sample,
                translation_frame=config['model']['base_link'], rotation_frame='local_end_effector',
                reference_waypoint=base_anchors[0],
                rotation_geodesic_deg=K.diff(K.Rotation.Quaternion(*base_anchors[0]['orientation_xyzw']),
                    K.Rotation.Quaternion(*anchors[0]['orientation_xyzw'])).Norm()*180/math.pi)
        files.append(output)
    return kin, files


class NativePreview:
    """Reuse the production bridge + C++ planner on a private DDS domain."""
    def __init__(self, config, ui_path):
        import fcntl
        import importlib.machinery
        import importlib.util
        import os
        import subprocess
        import tempfile
        import threading
        import time
        self.process = self.bridge = self.executor = None
        self.thread = None
        self.domain_lock = None
        for domain in range(221, 230):
            lock = open(Path(tempfile.gettempdir()) / f'waypoint_curve_domain_{domain}.lock', 'a')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                lock.close()
                continue
            self.domain_lock = lock
            self.domain = domain
            break
        if self.domain_lock is None:
            raise RuntimeError('다른 경로를 검사 중입니다. 잠시 후 다시 생성해 주세요.')
        os.environ.update(ROS_DOMAIN_ID=str(self.domain), RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                          ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        os.environ.pop('ROS_DISCOVERY_SERVER', None)
        loader = importlib.machinery.SourceFileLoader('curve_ui', str(ui_path))
        spec = importlib.util.spec_from_loader(loader.name, loader)
        ui = importlib.util.module_from_spec(spec)
        loader.exec_module(ui)
        self.ui, self.config = ui, config
        ui.rclpy.init(args=['--ros-args', '-p', f'arm:={config["arm"]}',
                           '-p', 'publish_enabled:=false', '-p', 'intervention_controller_url:=disabled',
                           '-p', f'max_waypoint_speed_scale:={float(config["max_speed_scale"])}',
                           '-p', f'base_link:={config["model"]["base_link"]}',
                           '-p', f'tip_link:={config["model"]["tip_link"]}'])
        try:
            self.bridge = ui.WaypointUiBridge()
            self.executor = ui.SingleThreadedExecutor()
            self.executor.add_node(self.bridge)
            self.description = self.bridge.create_publisher(ui.String, '/robot_description',
                ui.QoSProfile(depth=1, durability=ui.DurabilityPolicy.TRANSIENT_LOCAL))
            self.joints = self.bridge.create_publisher(ui.JointState, '/joint_states', 10)
            payload = config['payload']
            first = (1 if payload.get('preserve_outside') else
                     payload['segments'][0]['start_waypoint'] if 'segments' in payload else payload['noise_waypoint'])
            self.positions = payload['waypoint_file']['waypoints'][first-1]['joint_positions_rad']
            self.description.publish(ui.String(data=config['robot_description']))

            def sensors():
                msg = ui.JointState()
                msg.header.stamp = self.bridge.get_clock().now().to_msg()
                msg.name = ui.command_joint_names(config['arm'])
                msg.position = self.positions
                msg.velocity = [0.]*8
                self.joints.publish(msg)

            self.bridge.create_timer(.02, sensors)
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()
            from ament_index_python.packages import get_package_prefix
            executable = Path(get_package_prefix('waypoint_telep'))/'lib/waypoint_telep/waypoint_telep_node'
            self.planner_sha256 = hashlib.sha256(executable.read_bytes()).hexdigest()
            self.log = tempfile.TemporaryFile(mode='w+')
            self.process = subprocess.Popen([str(executable), '--ros-args',
                '-p', 'publish_enabled:=false', '-p', 'direct_action_enabled:=false',
                '-p', f'base_link:={config["model"]["base_link"]}',
                '-p', f'tip_link:={config["model"]["tip_link"]}',
                '-p', f'gripper_joint:={ui.command_joint_names(config["arm"])[7]}',
                '-p', f'max_waypoint_speed_scale:={float(config["max_speed_scale"])}'],
                stdout=self.log, stderr=self.log)
            self.wait(lambda: self.bridge.snapshot(config['arm'])['joint_state_fresh'], 10)
            # import_waypoints waits for its settings topic match and acknowledgement.
        except Exception:
            self.close()
            raise

    def wait(self, predicate, timeout=5):
        import time
        deadline = time.monotonic()+timeout
        while not predicate():
            if self.process is not None and self.process.poll() is not None:
                raise RuntimeError('독립 경로 검사기가 종료되었습니다.')
            if time.monotonic() > deadline:
                raise RuntimeError('독립 경로 검사 응답 시간이 초과되었습니다.')
            time.sleep(.01)

    def validate(self, data, kin):
        arm = data['arm']
        self.positions = list(data['waypoints'][0]['joint_positions_rad'])
        names = self.ui.command_joint_names(arm)
        self.wait(lambda: all(abs(self.bridge.arm_states[arm]['current_positions'].get(name, math.inf)-q) < 1e-9
                             for name, q in zip(names, self.positions)))
        self.bridge.import_waypoints(data)
        with self.bridge.lock:
            self.bridge.arm_states[arm]['path_refinement_events'] = []
            self.bridge.arm_states[arm]['trajectory'] = None
            self.bridge.arm_states[arm]['waypoint_targets'] = []
        response = self.bridge._call_planner_action(arm, 'preview')
        if not response['success']:
            raise ValueError('Cartesian → JOINT 검증 실패: '+response['message'])
        self.wait(lambda: self.bridge.arm_states[arm]['trajectory'] is not None
            and bool(self.bridge.arm_states[arm]['path_refinement_events'])
            and self.bridge.arm_states[arm]['path_refinement_events'][-1]['stage'] == 'success')
        with self.bridge.lock:
            trajectory = self.bridge.arm_states[arm]['trajectory']
            targets = copy.deepcopy(self.bridge.arm_states[arm]['waypoint_targets'])
            diagnostic = copy.deepcopy(self.bridge.arm_states[arm]['path_refinement_events'][-1])
        points = [dict(time_s=p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
                       positions_rad=list(p.positions)) for p in trajectory.points]
        self.wait(lambda: len(self.bridge.arm_states[arm]['waypoint_targets']) == len(data['waypoints']))
        targets = self.bridge.arm_states[arm]['waypoint_targets']
        anchor_indices = [i-1 for i in data['generator']['generated_anchor_indices']]
        anchor_times = [targets[i]['time_s'] for i in anchor_indices]
        positions, straight_errors, orientation_errors = [], [], []
        previous = None
        max_step = 0.
        for point in points:
            q = np.array(point['positions_rad'])
            kin.check(q)
            if previous is not None:
                max_step = max(max_step, float(max(abs(q-previous))))
            previous = q
            frame = kin.fk(q)
            position = np.array([frame.p[i] for i in range(3)])
            segment = min(len(anchor_times)-2, max(0, sum(point['time_s'] >= t-1e-9 for t in anchor_times)-1))
            if point['time_s'] < anchor_times[0]-1e-9 or point['time_s'] > anchor_times[-1]+1e-9:
                segment = -1
            positions.append(dict(time_s=point['time_s'], position_m=position.tolist(), segment=segment))
            spec = data['generator']['segments'][segment]
            if segment >= 0 and (spec['mode'] == 'LINEAR' or spec['curve_percent'] == 0):
                first, last = anchor_indices[segment:segment+2]
                a, b = [np.array(data['waypoints'][i]['position_m']) for i in (first, last)]
                tail = b-a
                u = float(np.clip((position-a)@tail/(tail@tail), 0, 1))
                straight_errors.append(float(np.linalg.norm(position-(a+u*tail)))*1000)
                rot = slerp(data['waypoints'][first]['orientation_xyzw'],
                            data['waypoints'][last]['orientation_xyzw'], u)
                orientation_errors.append(K.diff(frame.M, K.Rotation.Quaternion(*rot)).Norm()*180/math.pi)
        if (max_step > .05+1e-9
                or max(straight_errors, default=0) > data['path_refinement']['position_tolerance_mm']+1e-6
                or max(orientation_errors, default=0) > data['path_refinement']['orientation_tolerance_deg']+1e-6):
            raise ValueError('관절 경로의 직선 오차 또는 연속성 검사에 실패했습니다. '
                f'직선 {max(straight_errors, default=0):.6f} mm / '
                f'자세 {max(orientation_errors, default=0):.6f}° / 관절 변화 {max_step:.6f} rad')
        data['generator']['validation'] = dict(passed=True, native_planner=True,
            planner_sha256=self.planner_sha256, isolated_ros_domain=self.domain,
            publish_enabled=False, direct_action_enabled=False,
            max_straight_error_mm=max(straight_errors, default=0),
            max_straight_orientation_error_deg=max(orientation_errors, default=0),
            max_command_step_rad=max_step, command_samples=len(points),
            duration_s=points[-1]['time_s'], anchor_times_s=anchor_times, refinement=diagnostic)
        raw = dict(schema='waypoint_joint_trajectory/v1', arm=arm,
                   joint_names=list(trajectory.joint_names), points=points,
                   waypoint_targets=targets, physical_robot_executed=False)
        stride = max(1, len(positions)//300)
        preview = positions[::stride]
        if preview[-1] is not positions[-1]:
            preview.append(positions[-1])
        return raw, preview

    def close(self):
        import signal
        if self.process is not None:
            self.process.send_signal(signal.SIGINT) if self.process.poll() is None else None
            try:
                self.process.wait(timeout=5)
            except Exception:
                self.process.kill()
                self.process.wait()
            self.log.close()
        if self.bridge is not None:
            self.bridge.keyboard_shutdown.set()
            self.bridge.keyboard_thread.join(timeout=1)
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2)
        if self.thread is not None:
            self.thread.join(timeout=2)
        if self.bridge is not None:
            self.bridge.destroy_node()
        if hasattr(self, 'ui') and self.ui.rclpy.ok():
            self.ui.rclpy.shutdown()
        if self.domain_lock is not None:
            self.domain_lock.close()


def run(config, ui_path):
    kin, files = generate(config)
    native = NativePreview(config, ui_path)
    archive = io.BytesIO()
    summaries, first_preview = [], None
    try:
        with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as bundle:
            for i, data in enumerate(files):
                raw, preview = native.validate(data, kin)
                if i == 0:
                    first_preview = preview
                name = f'{config["arm"]}_curve_{i+1:02d}'
                bundle.writestr(name+'.json', json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
                bundle.writestr(name+'_trajectory.json', json.dumps(raw, ensure_ascii=False, allow_nan=False))
                summaries.append(dict(file=name+'.json', **data['generator']['validation']))
            bundle.writestr('validation.json', json.dumps(dict(files=summaries,
                physical_robot_executed=False, live_ui_applied=False), ensure_ascii=False, indent=2))
    finally:
        native.close()
    return dict(success=True, waypoint_file=files[0], preview=first_preview,
                files=summaries, archive_base64=base64.b64encode(archive.getvalue()).decode(),
                message=f'{len(files)}개 경로의 곡선 생성 · IK · Cartesian → JOINT 검증을 완료했습니다.')


if __name__ == '__main__':
    try:
        result = run(json.load(sys.stdin), sys.argv[1])
    except Exception as exc:
        result = dict(success=False, error=str(exc))
    print('CURVE_RESULT_JSON:'+json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
