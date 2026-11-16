#!/usr/bin/env python3
"""Receive-time tracking estimates; never interpret fitted lag as pure motor latency."""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np


def fit_tracking(command, feedback):
    """Fit feedback(t) against command(t-lag) on one common interior interval."""
    command, feedback = np.asarray(command), np.asarray(feedback)
    start = max(command[0, 0], feedback[0, 0]) + .5
    end = min(command[-1, 0], feedback[-1, 0]) - .5
    if end-start < 2:
        return None
    if np.max(np.diff(command[:, 0])) > .05 or np.max(np.diff(feedback[:, 0])) > .05:
        return dict(usable=False, reason='receive gap exceeds 50ms')
    t = np.arange(start, end, .01)
    measured = np.interp(t, feedback[:, 0], feedback[:, 1])
    lags = np.arange(0, .4001, .005)
    reference = np.interp(t, command[:, 0], command[:, 1])
    costs = [np.mean((measured-np.interp(t-lag, command[:, 0], command[:, 1]))**2)
             for lag in lags]
    best = int(np.argmin(costs))
    span = float(np.rad2deg(np.ptp(reference)))
    return dict(usable=span >= .5 and best not in (0, len(lags)-1),
                command_range_deg=span, feedback_range_deg=float(np.rad2deg(np.ptp(measured))),
                best_fit_lag_ms=float(lags[best]*1000),
                zero_lag_rmse_deg=float(np.rad2deg(np.sqrt(costs[0]))),
                fitted_rmse_deg=float(np.rad2deg(np.sqrt(costs[best]))),
                fit_at_search_boundary=best in (0, len(lags)-1),
                duration_s=float(end-start))


def analyze(rows):
    commands, references, feedback = defaultdict(list), defaultdict(list), defaultdict(list)
    for row in rows:
        t = row['receive_monotonic_ns']/1e9
        topic, msg = row['topic'], row['message']
        if topic.startswith('/command_source_arbiter/executed/'):
            # Current runtime emits one immediate position target. Reject other
            # trajectories instead of treating future endpoints as applied now.
            points = msg.get('points', [])
            if len(points) != 1 or any(points[0]['time_from_start'].values()):
                continue
            for name, q in zip(msg['joint_names'], points[0]['positions']):
                if name.startswith('arm_'):
                    commands[name].append((t, q))
        elif topic.endswith('/controller_state'):
            for name, r, q in zip(msg['joint_names'], msg['reference']['positions'], msg['feedback']['positions']):
                if name.startswith('arm_'):
                    references[name].append((t, r))
                    feedback[name].append((t, q))
    result = dict(joints={}, limitations=[
        'Subscriber arrival times, not hardware timestamps; fitted lag includes multiple pipeline stages.',
        'Fits use linear resampling at 100Hz, common interior interval, and 0..400ms lag search.',
        'Low excitation, boundary optima and receive gaps are unsuitable for a latency conclusion.',
        'Command variation can be intentional. Fit quality alone does not establish jitter causality.',
        'Torque status publisher is untrusted on the current container; no torque inference is made.'])
    for name, samples in commands.items():
        a = np.asarray(samples)
        if len(a) < 2 or not references[name]:
            continue
        segments = []
        # Split around gaps in any of the three streams. Never interpolate
        # through a missing interval just to obtain a global latency estimate.
        cuts = []
        for stream in (a, np.asarray(references[name]), np.asarray(feedback[name])):
            for i in np.where(np.diff(stream[:, 0]) > .05)[0]:
                cut = float((stream[i, 0]+stream[i+1, 0])/2)
                if a[0, 0] < cut < a[-1, 0]:
                    cuts.append(cut)
        boundaries = [float(a[0, 0]), *sorted(set(cuts)), float(a[-1, 0])]
        for begin, end in zip(boundaries, boundaries[1:]):
            seg = a[(a[:, 0] >= begin) & (a[:, 0] <= end)]
            if len(seg) < 2 or seg[-1, 0]-seg[0, 0] < 4:
                continue
            lo, hi = seg[0, 0], seg[-1, 0]
            ref = np.asarray([v for v in references[name] if lo <= v[0] <= hi])
            fb = np.asarray([v for v in feedback[name] if lo <= v[0] <= hi])
            if len(ref) < 2 or len(fb) < 2:
                continue
            segments.append(dict(start_monotonic_s=float(lo), end_monotonic_s=float(hi),
                command_to_feedback=fit_tracking(seg, fb),
                controller_reference_to_feedback=fit_tracking(ref, fb),
                command_to_controller_reference=fit_tracking(seg, ref)))
        result['joints'][name] = dict(command_samples=len(a),
            command_p99_gap_ms=float(np.percentile(np.diff(a[:, 0]), 99)*1000),
            command_max_gap_ms=float(np.max(np.diff(a[:, 0]))*1000), segments=segments)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.capture.open() as stream:
        report = analyze(json.loads(line) for line in stream if line.strip())
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
    print(args.output)


if __name__ == '__main__':
    main()
