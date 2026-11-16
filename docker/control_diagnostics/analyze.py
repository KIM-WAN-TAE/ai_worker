#!/usr/bin/env python3
"""Compare controller reference, feedback and current without claiming causality."""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np


def analyze(rows, *, command_tolerance_rad=1e-4, motion_rms_rad=.002, torque_telemetry_trusted=True):
    samples, torque, currents, intervals, final = defaultdict(list), [], defaultdict(list), defaultdict(list), defaultdict(list)
    for row in rows:
        t = row['receive_monotonic_ns'] / 1e9
        topic, msg = row['topic'], row['message']
        intervals[topic].append(t)
        if topic == '/ffw_follower/dxl_state':
            torque.append((t, dict(zip(msg['id'], msg['torque_state']))))
        elif topic.endswith('/controller_state'):
            names = msg['joint_names']
            reference = msg.get('reference', msg.get('desired', {}))
            feedback = msg.get('feedback', msg.get('actual', {}))
            for name, ref, measured in zip(names, reference.get('positions', []), feedback.get('positions', [])):
                if name.startswith('arm_') and np.isfinite([ref, measured]).all():
                    samples[name].append((t, ref, measured))
        elif topic.startswith('/command_source_arbiter/executed/'):
            # Trajectory endpoints are evidence, not an assumption that a
            # future endpoint has already been applied by the controller.
            for point in msg.get('points', []):
                for name, position in zip(msg['joint_names'], point['positions']):
                    final[name].append((t, position))
        elif topic == '/dynamic_joint_states':
            for name, values in zip(msg['joint_names'], msg['interface_values']):
                table = dict(zip(values['interface_names'], values['values']))
                if 'Present Current' in table and np.isfinite(table['Present Current']):
                    currents[name].append((t, table['Present Current']))

    result = dict(method='Two-second windows after two-second settling exclusion; receive-time evidence only.',
                  thresholds=dict(command_tolerance_rad=command_tolerance_rad, motion_rms_rad=motion_rms_rad),
                  torque_telemetry_trusted=torque_telemetry_trusted,
                  joints={}, topic_timing={}, limitations=[
                      'Changing commands may be intentional; this is not proof of model noise.',
                      'A downstream candidate includes mechanics, contacts and load, not only motor gain.',
                      'Current uses driver units. Saturation requires verified current limits.',
                      'Missing data, torque-off and gaps cannot establish motor stability.'])
    torque.sort(key=lambda row: row[0])
    for topic, ts in intervals.items():
        gaps = np.diff(ts)
        result['topic_timing'][topic] = dict(samples=len(ts),
            p99_gap_ms=float(np.percentile(gaps, 99)*1000) if len(gaps) else None,
            max_gap_ms=float(max(gaps)*1000) if len(gaps) else None)
    for name, values in samples.items():
        data = np.array(sorted(values))
        motor_id = (30 if name.startswith('arm_l_') else 0) + int(name.split('joint')[-1])
        windows = []
        for begin in np.arange(data[0, 0]+2, data[-1, 0]-2+1e-8, 1):
            end = begin+2
            window = data[(data[:, 0] >= begin) & (data[:, 0] < end)]
            if (len(window) < 20 or window[0, 0]-begin > .05 or
                    end-window[-1, 0] > .05 or np.max(np.diff(window[:, 0])) > .05):
                continue
            preceding = [(t, states) for t, states in torque if t <= begin]
            tw = ([preceding[-1]] if preceding else []) + [(t, states) for t, states in torque if begin < t <= end]
            torque_verified = (bool(tw) and begin-tw[0][0] <= .2 and
                end-tw[-1][0] <= .2 and
                all(b[0]-a[0] <= .2 for a, b in zip(tw, tw[1:])))
            energized = torque_verified and all(states.get(motor_id) is True for _, states in tw)
            cmd_range = float(np.ptp(window[:, 1]))
            actual_rms = float(np.std(window[:, 2]))
            classification = ('inconclusive_untrusted_torque_telemetry' if not torque_telemetry_trusted else
                'inconclusive_torque_or_telemetry' if not energized else
                'changing_reference' if cmd_range > command_tolerance_rad else
                'downstream_motion_candidate' if actual_rms > motion_rms_rad else
                'quiet_under_fixed_reference')
            cs = [v for t, v in currents.get(f'dxl{motor_id}', []) if begin <= t < end]
            fs = [v for t, v in final[name] if begin <= t < end]
            windows.append(dict(start_monotonic_s=float(begin), classification=classification,
                reference_range_rad=cmd_range, feedback_rms_rad=actual_rms,
                feedback_range_rad=float(np.ptp(window[:, 2])),
                current_samples=len(cs), current_range_driver_units=float(np.ptp(cs)) if cs else None,
                final_command_points=len(fs), final_target_range_rad=float(np.ptp(fs)) if fs else None))
        result['joints'][name] = dict(samples=len(data), windows=windows)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--untrusted-torque-telemetry', action='store_true',
                        help='Retain measurements but do not infer energized state from a defective status publisher.')
    args = parser.parse_args()
    with args.capture.open() as stream:
        result = analyze((json.loads(line) for line in stream if line.strip()),
                         torque_telemetry_trusted=not args.untrusted_torque_telemetry)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
    print(args.output)


if __name__ == '__main__':
    main()
