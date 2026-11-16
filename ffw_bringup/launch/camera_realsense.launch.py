# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in
# rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_multi_camera_launch.py \
#     camera_name1:=D400 \
#     device_type1:=d4 \
#     device_type2:=l5

"""Launch realsense2_camera node."""
import copy
import math
import os
import re
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
import yaml

# Add realsense2_camera/launch to sys.path using ROS package discovery
realsense2_camera_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'),
                                            'launch')
sys.path.append(realsense2_camera_launch_dir)
import rs_launch  # noqa: E402, I100

CAMERA_ROLES = {
    1: 'left',
    2: 'right',
    3: 'head',
}

CAMERA_USB_PORTS = {
    1: '1-4.5-7',   # left wrist D405
    2: '1-4.6-8',   # right wrist D405
    3: '2-3.1-4',   # head D455
}


# Utility function to load YAML as dict
def yaml_to_dict(path_to_yaml):
    if not os.path.exists(path_to_yaml):
        return {}
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader) or {}


def get_device_info(device, camera_info):
    try:
        if device.supports(camera_info):
            return device.get_info(camera_info)
    except RuntimeError:
        pass
    return ''


def normalize_usb_port(physical_port):
    usb_ports = re.findall(r'(?<!\w)(\d+(?:-\d+(?:\.\d+)*)+)(?![\w.])', physical_port or '')
    return usb_ports[-1] if usb_ports else physical_port


def devices_from_realsense_context():
    try:
        import pyrealsense2 as rs
    except ImportError:
        return []

    try:
        context = rs.context()
        connected_devices = list(context.query_devices())
    except RuntimeError:
        # A running camera can deny enumeration during a camera-only restart.
        # Let discovery fall back to the saved serial mapping in that case.
        return []

    devices = []
    for device in connected_devices:
        serial = get_device_info(device, rs.camera_info.serial_number)
        if not serial:
            continue
        physical_port = get_device_info(device, rs.camera_info.physical_port)
        devices.append({
            'serial': serial,
            'name': get_device_info(device, rs.camera_info.name),
            'product_line': get_device_info(device, rs.camera_info.product_line),
            'usb_port': normalize_usb_port(physical_port),
            'physical_port': physical_port,
        })
    return devices


def device_from_rs_enumerate_block(block):
    serial_match = re.search(r'Serial Number\s*:\s*([0-9]+)', block)
    if not serial_match:
        return {}
    physical_port_match = re.search(r'Physical Port\s*:\s*(.+)', block)
    name_match = re.search(r'Name\s*:\s*(.+)', block)
    product_line_match = re.search(r'Product Line\s*:\s*(.+)', block)
    physical_port = physical_port_match.group(1).strip() if physical_port_match else ''
    return {
        'serial': serial_match.group(1),
        'name': name_match.group(1).strip() if name_match else '',
        'product_line': product_line_match.group(1).strip() if product_line_match else '',
        'usb_port': normalize_usb_port(physical_port),
        'physical_port': physical_port,
    }


def devices_from_rs_enumerate_devices():
    try:
        result = subprocess.run(
            ['rs-enumerate-devices'],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return []

    if result.returncode != 0:
        return []

    devices = []
    for block in re.split(r'\n\s*\n', result.stdout):
        device = device_from_rs_enumerate_block(block)
        if device:
            devices.append(device)
    return devices


def discover_realsense_devices():
    seen = set()
    devices = []
    for device in devices_from_realsense_context() or devices_from_rs_enumerate_devices():
        serial = device.get('serial')
        if serial and serial not in seen:
            seen.add(serial)
            devices.append(device)
    return devices


def format_serial_for_launch(serial):
    serial = str(serial).strip()
    if serial.startswith("'") and serial.endswith("'"):
        return serial
    return f"'{serial}'"


def unquote_serial(serial):
    return str(serial).strip().strip('"').strip("'")


def is_head_camera(device):
    text = ' '.join([
        device.get('name', ''),
        device.get('product_line', ''),
    ]).upper()
    return 'D455' in text


def camera_key(index, suffix):
    return f'camera{index}_{suffix}'


def apply_device_to_camera(serials_dict, index, device):
    role = CAMERA_ROLES.get(index, '')
    serials_dict[camera_key(index, 'serial')] = format_serial_for_launch(device.get('serial', ''))
    if role:
        serials_dict[camera_key(index, 'role')] = role
    for key in ('usb_port', 'physical_port', 'name', 'product_line'):
        value = device.get(key)
        if value:
            serials_dict[camera_key(index, key)] = value


def stable_device_sort_key(device):
    return (device.get('usb_port') or device.get('physical_port') or device.get('serial') or '')


def assign_devices_by_configured_ports(serials_dict, devices):
    devices_by_port = {
        device.get('usb_port'): device
        for device in devices
        if device.get('usb_port')
    }

    for index, usb_port in CAMERA_USB_PORTS.items():
        device = devices_by_port.get(usb_port)
        if device:
            apply_device_to_camera(serials_dict, index, device)


def camera_is_assigned(serials_dict, index):
    return bool(serials_dict.get(camera_key(index, 'serial')))


def assigned_serials(serials_dict):
    return {
        unquote_serial(serials_dict.get(camera_key(index, 'serial'), ''))
        for index in CAMERA_ROLES
        if serials_dict.get(camera_key(index, 'serial'))
    }


def assign_devices_by_default(devices):
    serials_dict = {}
    head_devices = [device for device in devices if is_head_camera(device)]
    wrist_devices = [device for device in devices if device not in head_devices]

    assign_devices_by_configured_ports(serials_dict, devices)

    assigned = assigned_serials(serials_dict)
    unassigned_wrist_devices = [
        device for device in wrist_devices if device.get('serial') not in assigned
    ]
    for index, device in zip(
        [index for index in (1, 2) if not camera_is_assigned(serials_dict, index)],
        sorted(unassigned_wrist_devices, key=stable_device_sort_key),
    ):
        apply_device_to_camera(serials_dict, index, device)

    if not camera_is_assigned(serials_dict, 3) and head_devices:
        apply_device_to_camera(serials_dict, 3, sorted(head_devices, key=stable_device_sort_key)[0])
    elif not camera_is_assigned(serials_dict, 3) and len(devices) >= 3:
        assigned = assigned_serials(serials_dict)
        for device in sorted(devices, key=stable_device_sort_key):
            if device.get('serial') not in assigned:
                apply_device_to_camera(serials_dict, 3, device)
                break

    return serials_dict


def update_serials_from_stored_ports(stored_serials, devices):
    if not devices:
        return stored_serials

    devices_by_port = {
        device.get('usb_port'): device
        for device in devices
        if device.get('usb_port')
    }
    updated_serials = dict(stored_serials)
    matched_by_port = False

    for index in CAMERA_ROLES:
        stored_port = stored_serials.get(camera_key(index, 'usb_port'))
        device = devices_by_port.get(stored_port)
        if device:
            apply_device_to_camera(updated_serials, index, device)
            matched_by_port = True

    return updated_serials if matched_by_port else stored_serials


def add_ports_to_existing_serials(stored_serials, devices):
    if not devices:
        return stored_serials

    devices_by_serial = {
        device.get('serial'): device
        for device in devices
        if device.get('serial')
    }
    updated_serials = dict(stored_serials)

    for index in CAMERA_ROLES:
        serial = unquote_serial(stored_serials.get(camera_key(index, 'serial'), ''))
        device = devices_by_serial.get(serial)
        if device:
            apply_device_to_camera(updated_serials, index, device)

    return updated_serials


def serials_have_ports(serials_dict):
    return all(
        serials_dict.get(camera_key(index, 'usb_port'))
        for index in (1, 2)
        if serials_dict.get(camera_key(index, 'serial'))
    )


def ensure_head_from_model(serials_dict, devices):
    if serials_dict.get('camera3_serial') or not devices:
        return serials_dict

    for device in devices:
        if is_head_camera(device):
            updated_serials = dict(serials_dict)
            apply_device_to_camera(updated_serials, 3, device)
            return updated_serials

    return serials_dict


def write_serials_yaml(path_to_yaml, serials_dict):
    directory = os.path.dirname(path_to_yaml)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path_to_yaml, 'w') as f:
        f.write('# Auto-generated RealSense camera mapping for this robot.\n')
        f.write('# camera1=left, camera2=right, camera3=head.\n')
        f.write('# USB ports are used as the stable camera identity across bringups.\n')
        yaml.safe_dump(serials_dict, f, sort_keys=False, default_flow_style=False)


def try_write_serials_yaml(path_to_yaml, serials_dict):
    try:
        write_serials_yaml(path_to_yaml, serials_dict)
        print(f'[camera_realsense] Saved RealSense camera mapping to {path_to_yaml}')
    except OSError as exc:
        print(f'[camera_realsense] Could not save RealSense camera mapping to '
              f'{path_to_yaml}: {exc}')


def load_realsense_serials():
    persistent_path = os.environ.get('FFW_RS_SERIAL_PATH', '/workspace/config/rs_serial.yaml')
    fallback_path = os.path.join(
        get_package_share_directory('ffw_bringup'), 'config', 'common', 'rs_serial.yaml')

    persistent_serials = yaml_to_dict(persistent_path)
    discovered_devices = discover_realsense_devices()
    if persistent_serials:
        if serials_have_ports(persistent_serials):
            serials = update_serials_from_stored_ports(persistent_serials, discovered_devices)
        else:
            serials = add_ports_to_existing_serials(persistent_serials, discovered_devices)
        serials = ensure_head_from_model(serials, discovered_devices)
        if serials != persistent_serials:
            try_write_serials_yaml(persistent_path, serials)
        print(f'[camera_realsense] Using RealSense camera mapping from {persistent_path}')
        return serials

    if discovered_devices:
        serials = assign_devices_by_default(discovered_devices)
        try_write_serials_yaml(persistent_path, serials)
        return serials

    print(f'[camera_realsense] Could not auto-detect RealSense serials. '
          f'Using fallback serials from {fallback_path}')
    return yaml_to_dict(fallback_path)


serials = load_realsense_serials()
serial1 = serials.get('camera1_serial', '')
serial2 = serials.get('camera2_serial', '')
serial3 = serials.get('camera3_serial', '')  # d455 head, only used when enable_head_camera=true

local_parameters = [{'name': 'camera_name1', 'default': 'camera_left',
                     'description': 'camera1 unique name'},
                    {'name': 'camera_name2', 'default': 'camera_right',
                     'description': 'camera2 unique name'},
                    {'name': 'camera_name3', 'default': 'camera_head',
                     'description': 'camera3 unique name'},
                    {'name': 'camera_namespace1', 'default': 'camera_left',
                     'description': 'camera1 namespace'},
                    {'name': 'camera_namespace2', 'default': 'camera_right',
                     'description': 'camera2 namespace'},
                    {'name': 'camera_namespace3', 'default': 'camera_head',
                     'description': 'camera3 namespace'},
                    {'name': 'serial_no1', 'default': serial1,
                     'description': 'choose device1 by serial number'},
                    {'name': 'serial_no2', 'default': serial2,
                     'description': 'choose device2 by serial number'},
                    {'name': 'serial_no3', 'default': serial3,
                     'description': 'choose device3 by serial number'},
                    {'name': 'depth_module.depth_profile1', 'default': '640,480,15',
                     'description': 'depth stream profile for camera1'},
                    {'name': 'depth_module.depth_profile2', 'default': '640,480,15',
                     'description': 'depth stream profile for camera2'},
                    {'name': 'depth_module.color_profile1', 'default': '640,480,15',
                     'description': 'Left color capture profile; prioritize 15fps on the current USB2 link'},
                    {'name': 'depth_module.color_profile2', 'default': '640,480,15',
                     'description': 'Depth module color stream profile for d405 camera2'},
                    {'name': 'enable_depth1', 'default': 'false',
                     'description': 'Disable unused left depth stream to reduce USB traffic'},
                    {'name': 'enable_depth2', 'default': 'false',
                     'description': 'Disable unused right depth stream to reduce USB traffic'},
                     {'name': 'depth_module.depth_profile3', 'default': '640,480,15',
                    'description': 'Depth stream profile for D455 head camera'},
                    {'name': 'rgb_camera.color_profile3', 'default': '640,480,15',
                    'description': 'Color stream profile for D455 head camera'},
                    {'name': 'colorizer.enable1', 'default': 'false',
                     'description': 'enable colorizer filter for camera1'},
                    {'name': 'colorizer.enable2', 'default': 'false',
                     'description': 'enable colorizer filter for camera2'},
                    ]


def set_configurable_parameters(local_params):
    return {param['original_name']: LaunchConfiguration(param['name'])
            for param in local_params}


def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params


def launch_left_camera(context, params):
    namespace = LaunchConfiguration('camera_namespace1').perform(context).strip('/')
    name = LaunchConfiguration('camera_name1').perform(context).strip('/')
    prefix = '/' + '/'.join(part for part in (namespace, name, 'color') if part)
    image = prefix + '/image_rect_raw'
    source_image = prefix + '/unzoomed/image_rect_raw'
    info = prefix + '/camera_info'
    source_info = prefix + '/unzoomed/camera_info'
    remaps = [SetRemap(src=info, dst=source_info)]
    for suffix in ('', '/compressed', '/compressedDepth', '/theora', '/ffmpeg', '/zstd'):
        remaps.append(SetRemap(src=image + suffix, dst=source_image + suffix))
    return [
        GroupAction(actions=remaps + [OpaqueFunction(
            function=rs_launch.launch_setup,
            kwargs={'params': params, 'param_name_suffix': '1'},
        )]),
        Node(
            package='ffw_bringup', executable='color_zoom',
            namespace=namespace, name='color_zoom', output='screen',
            respawn=True, respawn_delay=2.0,
            parameters=[{
                'input_image': source_image, 'input_info': source_info,
                'output_image': image, 'output_info': info,
                'zoom': float(LaunchConfiguration('left_color_zoom').perform(context)),
                'display_rotation': 270,
                'output_width': 640,
                'output_height': 480,
                # An optional calibrated crop can preserve the VGA field of view
                # when a different sensor profile is selected.
                'source_crop': [int(value) for value in
                                LaunchConfiguration('left_color_crop').perform(context).split(',')],
            }],
        ),
    ]


def launch_cameras_staggered(context, camera_params):
    interval = float(LaunchConfiguration('camera_startup_interval').perform(context))
    if not math.isfinite(interval) or interval < 0:
        raise ValueError('camera_startup_interval must be a finite, non-negative number')

    # RSUSB discovery opens devices before filtering by serial. Concurrent nodes
    # can therefore contend even when they request different cameras. Allow each
    # node time to initialize before the next starts on the shared USB2 hub.
    actions = []
    for index, params in enumerate(camera_params, start=1):
        role = CAMERA_ROLES[index]
        if not IfCondition(LaunchConfiguration(f'enable_{role}_camera')).evaluate(context):
            continue
        action = OpaqueFunction(
            function=launch_left_camera if index == 1 else rs_launch.launch_setup,
            kwargs={'params': params} if index == 1 else {
                'params': params, 'param_name_suffix': str(index)},
        )
        delay = len(actions) * interval
        actions.append(TimerAction(period=delay, actions=[action]) if delay else action)
    return actions


def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    params3 = duplicate_params(rs_launch.configurable_parameters, '3')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_startup_interval', default_value='20.0',
                description='Seconds between enabled RealSense camera starts to reduce '
                            'USB initialization contention; 0 starts them together.'),
            DeclareLaunchArgument('enable_left_camera', default_value='true',
                                  choices=['true', 'false']),
            DeclareLaunchArgument('enable_right_camera', default_value='true',
                                  choices=['true', 'false']),
            DeclareLaunchArgument('left_color_zoom', default_value='2.0',
                                  description='Left color stream zoom, anchored at the '
                                              'displayed bottom for 270-degree rotation.'),
            DeclareLaunchArgument('left_color_crop', default_value='0,0,0,0',
                                  description='Crop x,y,width,height; zeros use automatic zoom. '
                                              'For matching VGA framing in 720p use 108,161,532,399.'),
            DeclareLaunchArgument(
                'enable_head_camera',
                default_value='false',
                choices=['true', 'false'],
                description='Launch the d455 head camera in addition to the two d405 wrist '
                            'cameras. Set to true on f2 (3-camera setup), false on '
                            'sg2/bg2/sh5/bh5 (2-camera setup with zed head).'
            ),
        ] +
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        rs_launch.declare_configurable_parameters(params2) +
        rs_launch.declare_configurable_parameters(params3) +
        [
            OpaqueFunction(
                function=launch_cameras_staggered,
                kwargs={
                    'camera_params': [set_configurable_parameters(params)
                                      for params in (params1, params2, params3)],
                },
            ),
        ]
    )
