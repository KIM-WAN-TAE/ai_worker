import json
from pathlib import Path
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

ROOT = Path(__file__).parent
URL = 'http://127.0.0.1:18767'


def request(path, data=None):
    encoded = None if data is None else json.dumps(data).encode()
    try:
        with urlopen(Request(URL + path, data=encoded, headers={
                'Content-Type': 'application/json'}), timeout=180) as response:
            return json.load(response)
    except HTTPError as error:
        raise RuntimeError(f'{path}: HTTP {error.code}: {error.read().decode()}') from error


reference = json.loads((ROOT.parent.parent / 'workspace/waypoint_total_time_check/live_backup/right.json').read_text())
reference.pop('joint_timing', None)
reference['start_waypoint'] = 1
reference['end_waypoint'] = len(reference['waypoints'])
reference['path_refinement'] = {'enabled': False}
started = time.monotonic()
request('/api/waypoints/import', reference)
generated = request('/api/generator/tool-path', {
    'arm': 'right', 'grasp_complete_waypoint': 247,
    'full_path_correction': True, 'local_offset_mm': [0.5, -0.25, 0.5],
})
assert generated['success']
settings = dict(enabled=True, subdivisions=10, smoothing_percent=60,
                position_tolerance_mm=1, orientation_tolerance_deg=1,
                joint_tolerance_rad=0.01, max_refinements=8)
request('/api/path-refinement', {'arm': 'right', 'path_refinement': settings})
preview = request('/api/actions/preview', {'arm': 'right'})
assert preview['success'], preview
state = request('/api/state?arm=right')
event = state['path_refinement_events'][-1]
assert event['stage'] == 'success', event
assert event['knot_count'] == 4921, event
assert event['joint_intermediate_points'] == 9
equal = request('/api/waypoints/equal-time', {'arm': 'right', 'duration_s': 1.5})
assert equal['verified'] and equal['max_error_s'] < 1e-6, equal
state = request('/api/state?arm=right')
arrivals = [w['trajectory_time_s'] for w in state['waypoints']]
for i in range(1, len(arrivals)):
    duration = arrivals[i] - arrivals[i - 1] - state['waypoints'][i - 1]['dwell_s']
    assert abs(duration - 1.5) < 1e-6, (i, duration)
exported = request('/api/waypoints/export?arm=right')
assert exported['path_refinement'] == settings
for generated_w, exported_w in zip(generated['waypoint_file']['waypoints'], exported['waypoints']):
    for key in ['joint_positions_rad', 'dwell_s', 'gripper_rad', 'interpolation_mode']:
        assert generated_w[key] == exported_w[key], (key, generated_w, exported_w)
result = {
    'passed': True, 'arm': 'right', 'source_waypoints': len(arrivals),
    'generated_joint_knots': event['knot_count'],
    'generation_offset_mm': [0.5, -0.25, 0.5],
    'refinement': settings, 'initial_validation': event, 'equal_time': equal,
    'final_validation': state['path_refinement_events'][-1],
    'elapsed_s': time.monotonic() - started,
    'physical_robot_commands': 0,
}
(ROOT / 'actual-reference-results.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
print(json.dumps(result, ensure_ascii=False, indent=2))
