import importlib.util
import json
from pathlib import Path
import signal
import threading
import rclpy

source = Path('/root/ros2_ws/src/ai_worker/waypoint_telep/test/test_mock_integration.py')
spec = importlib.util.spec_from_file_location('waypoint_test', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
rclpy.init()
graph = module.MockGraph()
left_base = [0.0, 0.4, 0.1, -0.5, 0.1, 0.2, 0.0, 0.1]
right_base = left_base.copy()
right_base[1] *= -1
graph.joint_names = module.LEFT_COMMAND_JOINTS + module.RIGHT_COMMAND_JOINTS
graph.positions = left_base + right_base
graph.velocities = [0.0] * 16
left = module.WaypointProcess(arm='left', quiet=True, max_waypoint_speed_scale=30.0)
right = module.WaypointProcess(arm='right', tip_link='end_effector_r_link', quiet=True,
                               max_waypoint_speed_scale=30.0)
ui = module.WaypointUiProcess(18767, arm='both', max_waypoint_speed_scale=30.0)
stopping = threading.Event()
signal.signal(signal.SIGINT, lambda *_: stopping.set())
signal.signal(signal.SIGTERM, lambda *_: stopping.set())
try:
    module.wait_for_http(ui.url)
    graph.wait_for(lambda: module.http_json(ui.url, '/api/state?arm=left')[1]['joint_state_fresh'])
    for arm, base in [('left', left_base), ('right', right_base)]:
        for offset in (0, 0.03, 0.06, 0.09):
            target = base.copy()
            target[0] += offset
            target[7] += offset
            module.http_json(ui.url, '/api/waypoints/joint', {'arm': arm, 'positions_rad': target})
        exported = module.http_json(ui.url, '/api/waypoints/export?arm=' + arm)[1]
        Path('/workspace/waypoint_refinement_check/browser_reference_' + arm + '.json').write_text(json.dumps(exported))
    print('READY http://127.0.0.1:18767', flush=True)
    stopping.wait()
finally:
    ui.stop()
    right.stop()
    left.stop()
    graph.close()
    rclpy.shutdown()
