#!/usr/bin/env python3
"""Bounded passive ROS capture. Does not publish commands or call services."""
import argparse
from collections import Counter
import json
from pathlib import Path
import queue
import threading
import time


TOPICS = (
    '/inference/trajectory_preview',
    '/command_source_arbiter/executed/arm_left',
    '/command_source_arbiter/executed/arm_right',
    '/arm_l_controller/controller_state', '/arm_r_controller/controller_state',
    '/joint_states', '/dynamic_joint_states', '/ffw_follower/dxl_state',
    '/command_source_arbiter/status',
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=30)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error('--seconds must be in [1, 300]')
    args.output.mkdir(parents=True, exist_ok=True)
    # Exclusive creation protects previous evidence from accidental overwrite.
    stream = (args.output / 'capture.jsonl').open('x')
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = rclpy.create_node('passive_control_diagnostics')
    pending = queue.Queue(maxsize=2048)
    counts, dropped, written = Counter(), Counter(), Counter()
    writer_errors = []

    def writer():
        try:
            while True:
                row = pending.get()
                if row is None:
                    break
                stream.write(json.dumps(row, separators=(',', ':')) + '\n')
                written[row['topic']] += 1
        except Exception as error:
            writer_errors.append(str(error))
        finally:
            stream.close()

    thread = threading.Thread(target=writer, daemon=True)
    thread.start()
    subscriptions = {}
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)

    def receive(topic, msg):
        received_mono, received_wall = time.monotonic_ns(), time.time_ns()
        counts[topic] += 1
        # Conversion runs in this separate diagnostic process, never in the
        # controller. Disk I/O has a bounded queue and explicit drop accounting.
        row = dict(topic=topic, receive_monotonic_ns=received_mono,
                   receive_wall_ns=received_wall, message=message_to_ordereddict(msg))
        try:
            pending.put_nowait(row)
        except queue.Full:
            dropped[topic] += 1

    start = time.monotonic()
    next_discovery = start
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds and not writer_errors:
            if time.monotonic() >= next_discovery:
                catalog = dict(node.get_topic_names_and_types())
                for topic in TOPICS:
                    if topic in catalog and topic not in subscriptions:
                        subscriptions[topic] = node.create_subscription(
                            get_message(catalog[topic][0]), topic,
                            lambda msg, topic=topic: receive(topic, msg), qos)
                next_discovery = time.monotonic() + 1
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - start
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if thread.is_alive():
            try:
                pending.put(None, timeout=2)
            except queue.Full:
                writer_errors.append('writer queue failed to drain')
            thread.join(timeout=5)
        if thread.is_alive():
            writer_errors.append('writer did not finish')
        summary = dict(passive_only=True, elapsed_s=elapsed, received=dict(counts),
                       written=dict(written), dropped=dict(dropped), writer_errors=writer_errors,
                       missing_topics=[t for t in TOPICS if not counts[t]],
                       timing='Subscriber receive times; not hardware delivery timestamps.',
                       current_units='Present Current from dynamic_joint_states retains driver units.')
        (args.output / 'capture_summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
    if writer_errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
