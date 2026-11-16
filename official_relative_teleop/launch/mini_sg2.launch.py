"""Run official relative-pose control behind the existing SG2 command arbiter."""
from pathlib import Path
import copy
import tempfile
import xml.etree.ElementTree as ET

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    flag = lambda name: value(name).lower() == 'true'
    for side in ('left','right'):
        if value(f'{side}_trajectory_output_topic') != f'/intervention/teleop/arm_{side}':
            raise ValueError('Official Mini arm outputs must pass through the command-source arbiter')
    if value('teleop_reference_mode') != 'relative':
        raise ValueError('Official Mini integration supports relative mode only')
    package = Path(share('official_relative_teleop'))
    description = Path(share('ffw_description'))
    models = Path(share('cyclo_motion_controller_models')) / 'models/ai_worker'
    generated = Path(tempfile.mkdtemp(prefix='official-relative-'))
    hardware = yaml.safe_load((package/'config/mini_hardware.yaml').read_text())
    mini = xacro.process_file(str(description/'urdf/ffw_lg2_mini_leader/ffw_lg2_mini_leader.urdf.xacro'),
                             mappings={'use_mock_hardware': value('use_mock_hardware'),
                                       'use_sensorxel': value('use_sensorxel'),
                                       'left_port_name': value('left_port_name'),
                                       'right_port_name': value('right_port_name')}).toxml()
    if flag('use_mock_hardware'):
        # Leader arm motors are read-only on hardware. Expose mock-only commands
        # so the isolated integration test can move their simulated encoders.
        mock_tree = ET.fromstring(mini)
        for joint in mock_tree.findall('./ros2_control/joint'):
            if joint.get('name').startswith(('arm_l_joint', 'arm_r_joint')):
                ET.SubElement(joint, 'command_interface', {'name':'position'})
        mini = ET.tostring(mock_tree, encoding='unicode')
    # FK-only Mini model: exclude its virtual lift by selecting arm_base_link.
    leader_model = generated/'mini_fk.urdf'
    leader_tree = ET.fromstring(mini)
    for side in ('l','r'):
        # The spring trigger rotates gripper_*_link1. A fixed tool frame at its
        # pivot prevents opening the gripper from rotating the follower wrist.
        trigger = leader_tree.find(f"joint[@name='gripper_{side}_joint1']")
        ET.SubElement(leader_tree,'link',{'name':f'mini_tool_{side}'})
        tool = ET.SubElement(leader_tree,'joint',{'name':f'mini_tool_{side}_fixed','type':'fixed'})
        ET.SubElement(tool,'parent',{'link':f'arm_{side}_link7'})
        ET.SubElement(tool,'child',{'link':f'mini_tool_{side}'})
        tool.append(copy.deepcopy(trigger.find('origin')))
    ET.ElementTree(leader_tree).write(leader_model)
    follower_tree = ET.parse(models/'ffw_sg2_follower.urdf')
    local_tree = ET.parse(description/'urdf/ffw_sg2_rev1_follower/ffw_sg2_follower.urdf')
    local_joints = {j.attrib['name']:j for j in local_tree.getroot().findall('joint')}
    for joint in follower_tree.getroot().findall('joint'):
        name = joint.attrib['name']
        if name.startswith(('arm_l_joint', 'arm_r_joint')) and name in local_joints:
            limit, installed = joint.find('limit'), local_joints[name].find('limit')
            # Never widen official constraints or the installed hardware's encoder limits.
            for key, operation in [('lower', max), ('upper', min), ('velocity', min)]:
                limit.set(key, str(operation(float(limit.get(key)),float(installed.get(key)))))
    follower_model = generated/'sg2_calculation.urdf'
    follower_tree.write(follower_model)

    follower = yaml.safe_load((package/'config/sg2_upstream.yaml').read_text())['cyclo_teleoperation']['ros__parameters']
    follower.update({
        'follower_urdf_path': str(follower_model),
        'follower_srdf_path': str(models/'ffw_sg2_follower_modified.srdf'),
        'model_action_follower_srdf_path': str(models/'ffw_sg2_follower_default.srdf'),
        'default_control_mode': 2,
        'available_control_modes': [2],
        # Pose mode receives its gripper through the clutch adapter. Upstream's
        # generic raw-joint subscriber would otherwise overwrite that target.
        'left_leader_topic': '/official_relative/unused/left_joint',
        'right_leader_topic': '/official_relative/unused/right_joint',
        'left_command_topic': '/official_relative/output/left',
        'right_command_topic': '/official_relative/output/right',
        'control_command_topic': '/official_relative/control/command',
        'control_status_topic': '/official_relative/control/status',
        'follower_left_eef_pose_topic': '/official_relative/state/left/pose',
        'follower_right_eef_pose_topic': '/official_relative/state/right/pose',
        'follower_left_eef_reference_topic': '/official_relative/action/left/pose',
        'follower_right_eef_reference_topic': '/official_relative/action/right/pose',
        'commands': {s: {'pose_topic':f'/official_relative/action/{s}/pose',
                         'joint_topic':f'/official_relative/action/{s}/gripper'} for s in ('left','right')},
    })
    original = hardware['/**']['joint_trajectory_command_broadcaster']['ros__parameters']
    follower['control_modes']['2']['initial_pose'] = {'enabled':False}
    follower['available_presets'] = [3, 4]
    follower['presets'] = {str(i): {
        'name': f'mini_saved_pose_{i}', 'step_names':['target'],
        'steps': {'target': {s: {'positions': original[f'{s}_save_pose_{i}'][:7],
                                  'gripper_position': original[f'{s}_save_pose_{i}'][7]} for s in ('left','right')}},
        'duration': original['save_pose_duration'], 'completion_tolerance': .1, 'timeout': 30.,
    } for i in (3,4)}
    controllers = hardware['/**']['controller_manager']['ros__parameters']
    controllers['joint_trajectory_command_broadcaster']['type'] = 'lg2_joint_trajectory_command_broadcaster/LG2JointTrajectoryCommandBroadcaster'
    leader = {k: original[k] for k in ('left_joints','right_joints','left_offsets','right_offsets')}
    leader.update({
        'enable_teleoperation': True, 'style':'advanced', 'default_control_mode':2,
        # The local edge-trigger node owns Mini's saved-pose gesture. Disable
        # the upstream long-hold gesture with a threshold outside encoder range.
        'trigger_threshold':-100.0,
        'default_left_preset_id':3, 'default_right_preset_id':3,
        'output_topic_suffix':'raw_joint_trajectory', 'leader_pose_reference_enabled':True,
        'leader_urdf_path':str(leader_model), 'leader_base_frame':'arm_base_link',
        'leader_left_eef':'mini_tool_l', 'leader_right_eef':'mini_tool_r',
        'leader_use_follower_reference_model':False,
        'follower_control_node':'/official_relative_controller',
        'command_source_state_topic':'/official_relative/source',
        'set_source_service':'/official_relative/set_source',
        'control_command_topic':'/official_relative/control/command',
        'control_status_topic':'/official_relative/control/status',
        'follower_left_eef_state_topic':'/official_relative/state/left/pose',
        'follower_right_eef_state_topic':'/official_relative/state/right/pose',
        'left_eef_action_topic':'/official_relative/action/left/pose',
        'right_eef_action_topic':'/official_relative/action/right/pose',
        'left_gripper_action_topic':'/official_relative/raw_gripper/left',
        'right_gripper_action_topic':'/official_relative/raw_gripper/right',
        'cartesian_scaling':{'left_translation_gain':1.,'right_translation_gain':1.},
    })
    hardware['/**']['joint_trajectory_command_broadcaster']['ros__parameters'] = leader
    if flag('use_mock_hardware'):
        controllers['mock_position_controller'] = {'type':'position_controllers/JointGroupPositionController'}
        hardware['/**']['mock_position_controller'] = {'ros__parameters': {
            'joints': original['left_joints'][:7] + original['right_joints'][:7]}}
    hardware_file = generated/'mini_controllers.yaml'
    hardware_file.write_text(yaml.safe_dump(hardware))
    follower_file = generated/'sg2.yaml'
    follower_file.write_text(yaml.safe_dump({'official_relative_controller':{'ros__parameters':follower}}))
    runtime = Node(package='cyclo_teleoperation',executable='cyclo_teleoperation_node',
                   name='official_relative_controller',parameters=[str(follower_file)],output='screen',
                   remappings=[('/source','/official_relative/source'),('/set_source','/official_relative/set_source'),
                               ('/toggle_source','/official_relative/toggle_source'),('/set_mode','/official_relative/set_mode')])
    bridge = Node(package='official_relative_teleop',executable='compatibility_bridge.py',
                  parameters=[{'auto_select_teleop':flag('auto_select_teleop')}],output='screen')
    actions = [runtime, bridge]
    if flag('launch_leader_hardware'):
        manager = Node(package='controller_manager',executable='ros2_control_node',namespace='leader',
                       parameters=[str(hardware_file),{'robot_description':mini}],output='screen')
        rsp = Node(package='robot_state_publisher',executable='robot_state_publisher',namespace='leader',
                   parameters=[{'robot_description':mini,'frame_prefix':'leader_'}],output='screen')
        controller_names = ['joint_state_broadcaster','joint_trajectory_command_broadcaster','trigger_position_controller']
        if flag('use_mock_hardware'):
            controller_names.append('mock_position_controller')
        spawner = Node(package='controller_manager',executable='spawner',namespace='leader',
                       arguments=controller_names+['--controller-manager-timeout','30'],output='screen')
        def spawned(event, _context):
            if event.returncode:
                return [EmitEvent(event=Shutdown(reason='Official Mini controller activation failed'))]
            if flag('use_mock_hardware'):
                return []
            return [ExecuteProcess(cmd=[
                'ros2','topic','pub','--times','50','--rate','50',
                '/leader/trigger_position_controller/commands','std_msgs/msg/Float64MultiArray',
                'data: [0.0, 0.0]'], output='log')]
        actions += [manager,rsp,spawner,RegisterEventHandler(OnProcessExit(target_action=spawner,on_exit=spawned))]
        actions.append(RegisterEventHandler(OnProcessExit(target_action=manager,
            on_exit=[EmitEvent(event=Shutdown(reason='Mini controller manager exited'))])))
        if flag('use_sensorxel'):
            # Preserve the existing intervention lease's auxiliary-topic
            # isolation, including the base and recording trigger.
            remaps = {
                '/leader/joystick_controller_left/joint_trajectory':value('left_joystick_output_topic'),
                '/leader/joystick_controller_right/joint_trajectory':value('right_joystick_output_topic'),
                '/cmd_vel':value('cmd_vel_output_topic'),
                '/leader/joystick_controller/tact_trigger':value('tact_trigger_output_topic'),
            }
            joystick = Node(package='controller_manager',executable='spawner',namespace='leader',
                arguments=['joystick_controller','--controller-manager-timeout','30'] +
                          [f'--controller-ros-args=-r {a}:={b}' for a,b in remaps.items()],output='screen')
            def joystick_spawned(event, _context):
                if event.returncode:
                    return [EmitEvent(event=Shutdown(reason='Mini joystick activation failed'))]
                return []
            actions += [joystick, RegisterEventHandler(OnProcessExit(
                target_action=joystick,on_exit=joystick_spawned))]
        if flag('launch_gripper_trigger'):
            actions.append(Node(package='official_relative_teleop',executable='gripper_trigger',namespace='leader',
                                parameters=[{'gripper_threshold':-.7,'save_pose_id':3,'enable_press_gestures':False,
                                             'tact_trigger_topic':value('tact_trigger_output_topic')}],output='screen'))
        if flag('launch_foot_switch') and not flag('use_mock_hardware'):
            actions.append(Node(package='ffw_bringup',executable='foot_switch_node',
                                parameters=[{'controller_config_path':str(package/'config/mini_hardware.yaml')}],output='screen'))
    # A failed runtime must not leave an apparently active leader stack behind.
    for process in (runtime, bridge):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Official relative control process exited'))])))
    return actions


def generate_launch_description():
    defaults = {
        'use_mock_hardware':'false', 'use_sensorxel':'true', 'launch_leader_hardware':'true',
        'launch_gripper_trigger':'true','auto_select_teleop':'true',
        'teleop_reference_mode':'relative',
        'left_trajectory_output_topic':'/intervention/teleop/arm_left',
        'right_trajectory_output_topic':'/intervention/teleop/arm_right',
        'left_joystick_output_topic':'/leader/joystick_controller_left/joint_trajectory',
        'right_joystick_output_topic':'/leader/joystick_controller_right/joint_trajectory',
        'cmd_vel_output_topic':'/cmd_vel',
        'tact_trigger_output_topic':'/leader/joystick_controller/tact_trigger',
        'launch_foot_switch':str(Path('/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd').exists()).lower(),
        'left_port_name':'/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_00000002-if00',
        'right_port_name':'/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_00000001-if00',
    }
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
