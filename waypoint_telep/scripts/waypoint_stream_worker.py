#!/usr/bin/env python3
"""Isolated session: hold through planning, stream, then hold through completion."""
import importlib.machinery
import importlib.util
import json
import os
import queue
import sys
import threading
import time


def main():
    loader = importlib.machinery.SourceFileLoader('waypoint_ui_math', sys.argv[1])
    ui = importlib.util.module_from_spec(importlib.util.spec_from_loader(loader.name, loader))
    loader.exec_module(ui)
    config = json.loads(sys.stdin.readline())
    assert config['type'] == 'init'
    ui.rclpy.init(args=[])
    node = ui.Node(f'waypoint_stream_worker_{os.getpid()}')
    bridge = object.__new__(ui.WaypointUiBridge)
    bridge.lock = threading.RLock()
    bridge.execution_stop = threading.Event()
    bridge.control_rate_hz = float(config['control_rate_hz'])
    bridge.max_joint_step_rad = float(config['max_joint_step_rad'])
    bridge.max_schedule_lag_s = float(config['max_schedule_lag_s'])
    bridge.arbiter_mode = 'UNKNOWN'
    bridge.arbiter_fault = ''
    bridge.waypoint_command_publishers = {
        arm: node.create_publisher(ui.JointTrajectory, f'/intervention/waypoint/arm_{arm}', 10)
        for arm in ui.ARMS
    }
    parent_pid = os.getppid()
    finished = threading.Event()
    go = threading.Event()
    plans = queue.Queue()
    failure = ['']
    status_stamp = [0.0]
    waypoint_ready_count = [0]
    last_progress = [None]
    last_report = [0.0]
    output_lock = threading.Lock()
    hold_stop = threading.Event()
    hold_thread = None
    publication = {'count': 0, 'max_call_s': 0.0, 'max_gap_s': 0.0}
    last_publish = [None]
    publish_pair = bridge._publish_waypoint_pair

    def tracked_publish(pair):
        now = time.monotonic()
        if last_publish[0] is not None:
            publication['max_gap_s'] = max(publication['max_gap_s'], now-last_publish[0])
        publish_pair(pair)
        publication['max_call_s'] = max(publication['max_call_s'], time.monotonic()-now)
        publication['count'] += 1
        last_publish[0] = now

    bridge._publish_waypoint_pair = tracked_publish

    def report(message):
        with output_lock:
            print(json.dumps(message, allow_nan=False), flush=True)

    def status(message):
        try:
            data = json.loads(message.data)
            with bridge.lock:
                bridge.arbiter_mode = data.get('mode', 'UNKNOWN')
                bridge.arbiter_fault = data.get('fault', '')
                status_stamp[0] = time.monotonic()
                if data.get('sources', {}).get('waypoint', {}).get('ready') is True:
                    waypoint_ready_count[0] += 1
                else:
                    waypoint_ready_count[0] = 0
        except (ValueError, TypeError):
            failure[0] = 'command_source_arbiter 상태가 올바르지 않습니다.'
            bridge.execution_stop.set()

    qos = ui.QoSProfile(depth=1, reliability=ui.ReliabilityPolicy.RELIABLE,
                       durability=ui.DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(ui.String, config['arbiter_namespace'] + '/status', status, qos)
    executor = ui.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    def control():
        try:
            for line in sys.stdin:
                message = json.loads(line)
                if message['type'] == 'plan':
                    plans.put(message)
                elif message['type'] == 'go':
                    go.set()
                elif message['type'] == 'stop':
                    failure[0] = '사용자가 waypoint 실행을 중지했습니다.'
                    bridge.execution_stop.set()
        except (OSError, ValueError):
            pass
        finally:
            bridge.execution_stop.set()

    def watchdog():
        while not finished.wait(0.01):
            # GUI stalls do not revoke the session; parent death/EOF still stops it.
            if os.getppid() != parent_pid:
                failure[0] = 'UI 프로세스가 종료되었습니다.'
                bridge.execution_stop.set()
            if go.is_set() and time.monotonic() - status_stamp[0] > 0.6:
                failure[0] = 'command_source_arbiter 상태 수신이 지연되었습니다.'
                bridge.execution_stop.set()

    threading.Thread(target=control, daemon=True).start()
    threading.Thread(target=watchdog, daemon=True).start()

    def progress(arm, phase, index, count, planned_time):
        last_progress[0] = dict(phase=phase, sample_index=index,
                               sample_count=count, planned_time_s=planned_time)
        if time.monotonic() - last_report[0] >= 0.1:
            report({'type': 'progress', 'progress': last_progress[0]})
            last_report[0] = time.monotonic()

    bridge._record_stream_progress = progress

    def stop_hold():
        nonlocal hold_thread
        hold_stop.set()
        if hold_thread is not None:
            hold_thread.join(timeout=1.0)
            if hold_thread.is_alive():
                raise RuntimeError('독립 hold publisher가 종료되지 않았습니다.')
            hold_thread = None

    def start_hold(pair):
        nonlocal hold_thread
        stop_hold()
        bridge._validate_command_pair(pair)
        hold_stop.clear()

        def hold():
            try:
                while not bridge.execution_stop.is_set() and not hold_stop.is_set():
                    bridge._publish_waypoint_pair(pair)
                    hold_stop.wait(0.02)
            except Exception as exception:
                failure[0] = str(exception)
                bridge.execution_stop.set()

        hold_thread = threading.Thread(target=hold, daemon=True)
        hold_thread.start()

    try:
        deadline = time.monotonic() + 10.0
        while not all(p.get_subscription_count() for p in bridge.waypoint_command_publishers.values()):
            if bridge.execution_stop.wait(0.01):
                return
            if time.monotonic() > deadline:
                raise RuntimeError('waypoint command 수신 노드를 찾지 못했습니다.')
        # Cache both inputs before the UI selects WAYPOINT and keep the lease
        # fresh while the UI computes/validates plans, including sequence joins.
        start_hold(config['initial_pair'])
        # Discovery alone does not acknowledge delivery. Require two fresh
        # arbiter status cycles to confirm both hold inputs before ownership
        # changes, including after a prior worker has closed.
        waypoint_ready_count[0] = 0
        deadline = time.monotonic() + 5.0
        while waypoint_ready_count[0] < 2:
            if bridge.execution_stop.wait(0.01):
                return
            if time.monotonic() > deadline:
                raise RuntimeError('arbiter가 양팔 hold command 수신을 확인하지 못했습니다.')
        report({'type': 'ready'})
        while not bridge.execution_stop.is_set():
            try:
                plan = plans.get(timeout=0.02)
            except queue.Empty:
                continue
            arm = plan['arm']
            assert arm in ui.ARMS
            go.clear()
            last_progress[0] = None
            trajectory = ui.JointTrajectory()
            trajectory.joint_names = ui.command_joint_names(arm)
            previous = None
            previous_time = 0.0
            for stamp, positions in plan['points']:
                bridge._validate_arm_command(arm, positions)
                assert stamp > previous_time, 'trajectory time must increase'
                if previous is not None:
                    assert max(abs(a-b) for a,b in zip(previous,positions)) <= bridge.max_joint_step_rad + 1e-9
                point = ui.JointTrajectoryPoint()
                point.positions = positions
                ticks = round(stamp * 1e9)
                point.time_from_start.sec = ticks // 1_000_000_000
                point.time_from_start.nanosec = ticks % 1_000_000_000
                trajectory.points.append(point)
                previous, previous_time = positions, stamp
            assert trajectory.points
            bridge._validate_command_pair(plan['initial_pair'])
            deadline = time.monotonic() + 2.0
            while bridge.arbiter_mode != 'WAYPOINT' or bridge.arbiter_fault:
                if bridge.arbiter_fault or time.monotonic() > deadline:
                    raise RuntimeError(f'WAYPOINT 제어권이 해제되었습니다: {bridge.arbiter_mode}, {bridge.arbiter_fault}')
                if bridge.execution_stop.wait(0.01):
                    break
            report({'type': 'prepared'})
            while not go.wait(0.01):
                if bridge.execution_stop.is_set():
                    break
            if bridge.execution_stop.is_set():
                break
            stop_hold()
            final_pair = bridge._stream_trajectory_local(arm, trajectory, plan['initial_pair'])
            start_hold(final_pair)
            report({'type': 'done', 'final_pair': final_pair, 'progress': last_progress[0],
                    'publication': dict(publication)})
        if failure[0]:
            raise RuntimeError(failure[0])
    except Exception as exception:
        report({'type': 'error', 'message': (failure[0] or str(exception)) +
                ' [publisher=' + json.dumps(publication) + ']',
                **({'progress': last_progress[0]} if last_progress[0] else {})})
    finally:
        finished.set()
        stop_hold()
        executor.shutdown(timeout_sec=1.0)
        spin.join(timeout=1.0)
        node.destroy_node()
        ui.rclpy.shutdown()


if __name__ == '__main__':
    main()
