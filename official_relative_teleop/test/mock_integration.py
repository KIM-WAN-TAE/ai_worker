#!/usr/bin/env python3
"""Kinematic integration test; run only in an isolated, device-free container.

Exercises the actual upstream controller plugin and QP runtime, local arbiter,
and compatibility bridge. Feedback follows selected commands without physics.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, UInt8, Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotis_interfaces.msg import ControlModeStatus
from tf2_ros import StaticTransformBroadcaster

SIDES = ('left','right')
JOINTS = {s: [f'arm_{c}_joint{i}' for i in range(1,8)]+[f'gripper_{c}_joint1']
          for s,c in zip(SIDES,('l','r'))}


class Rig(Node):
    def __init__(self):
        super().__init__('official_relative_mock_test')
        self.q = {'left':[.431,.304,.152,-2.457,-.267,.686,-.948,.4],
                  'right':[.431,-.304,-.152,-2.457,.267,.686,.948,.6]}
        self.initial = {s:list(q) for s,q in self.q.items()}
        self.arbiter = {}
        self.control = ControlModeStatus()
        self.source = ''
        self.final = {s:[] for s in SIDES}
        self.poses = {}
        self.raw_gripper = {}
        self.action_gripper = {}
        self.feed_enabled = True
        self.gripper_deflection = {s:0. for s in SIDES}
        self.policy_enabled = False
        self.policy_q = {s:list(q) for s,q in self.q.items()}
        self.leader = [0.]*14
        self.leader_pub = self.create_publisher(Float64MultiArray,'/leader/mock_position_controller/commands',10)
        self.trigger_pub = self.create_publisher(Float64MultiArray,'/leader/trigger_position_controller/commands',10)
        self.feedback_pub = self.create_publisher(JointState,'/joint_states',10)
        self.buttons = {s:self.create_publisher(UInt8,f'/leader/{s}_command',10) for s in SIDES}
        self.policy = {s:self.create_publisher(JointTrajectory,f'/intervention/policy/arm_{s}',10) for s in SIDES}
        latched = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/command_source_arbiter/status',self.status,latched)
        self.create_subscription(String,'/official_relative/source',lambda m:setattr(self,'source',m.data),latched)
        self.create_subscription(ControlModeStatus,'/official_relative/control/status',lambda m:setattr(self,'control',m),latched)
        for s in SIDES:
            self.create_subscription(JointTrajectory,f'/official_relative/raw_gripper/{s}',
                                     lambda m,side=s:self.raw_gripper.update({side:list(m.points[0].positions)}),10)
            self.create_subscription(JointTrajectory,f'/official_relative/action/{s}/gripper',
                                     lambda m,side=s:self.action_gripper.update({side:list(m.points[0].positions)}),10)
            self.create_subscription(JointTrajectory,f'/leader/joint_trajectory_command_broadcaster_{s}/joint_trajectory',
                                     lambda m,side=s:self.command(side,m),10)
            self.create_subscription(PoseStamped,f'/official_relative/action/{s}/pose',
                                     lambda m,side=s:self.poses.update({side:m}),10)
        self.tf = StaticTransformBroadcaster(self)
        transform=TransformStamped()
        transform.header.frame_id='base_link'
        transform.child_frame_id='arm_base_link'
        transform.transform.translation.x=-.0199
        transform.transform.translation.z=1.4316-.3
        transform.transform.rotation.w=1.
        self.tf.sendTransform(transform)
        self.create_timer(.01,self.publish)

    def status(self,message):
        self.arbiter=json.loads(message.data)

    def command(self,side,message):
        assert message.joint_names==JOINTS[side], message.joint_names
        q=list(message.points[0].positions)
        assert len(q)==8 and all(math.isfinite(x) for x in q), q
        self.final[side].append((time.monotonic(),q))
        self.q[side]=q

    def publish(self):
        if self.feed_enabled:
            sensed={s:list(q) for s,q in self.q.items()}
            for s in SIDES:
                sensed[s][-1]+=self.gripper_deflection[s]
            self.feedback_pub.publish(JointState(header=self.header(),
                name=JOINTS['left']+JOINTS['right']+['lift_joint'],
                position=sensed['left']+sensed['right']+[-.3],velocity=[0.]*17))
        self.leader_pub.publish(Float64MultiArray(data=self.leader))
        if self.policy_enabled:
            for s in SIDES:
                self.policy[s].publish(JointTrajectory(joint_names=JOINTS[s],
                    points=[JointTrajectoryPoint(positions=self.policy_q[s])]))

    def header(self):
        from std_msgs.msg import Header
        return Header(stamp=self.get_clock().now().to_msg())

    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self,timeout_sec=.01)

    def until(self,condition,timeout=15.):
        end=time.monotonic()+timeout
        while not condition():
            assert time.monotonic()<end, f'timeout: source={self.source}, control={self.control}, arbiter={self.arbiter}'
            rclpy.spin_once(self,timeout_sec=.01)

    def service(self,name):
        client=self.create_client(Trigger,f'/command_source_arbiter/{name}')
        self.until(client.service_is_ready)
        future=client.call_async(Trigger.Request())
        self.until(future.done)
        result=future.result()
        assert result.success, result.message
        self.destroy_client(client)

    def button(self,side,value):
        self.buttons[side].publish(UInt8(data=value))

    def snapshot(self):
        return {s:list(q) for s,q in self.q.items()}


def distance(a,b,arm_only=True):
    return max(abs(x-y) for s in SIDES for x,y in zip(a[s][:(7 if arm_only else 8)],b[s]))


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='91', 'isolated ROS domain 91 required'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1', 'localhost only required'
    assert not Path('/dev/serial').exists(), 'test container must have no serial devices'
    logs=Path('/workspace/official_relative_teleop/test_results')
    logs.mkdir(exist_ok=True)
    processes=[]
    handles=[]
    results={}
    def start(name,command):
        handle=(logs/f'{name}.log').open('w')
        handles.append(handle)
        process=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
    rclpy.init()
    rig=Rig()
    try:
        start('arbiter',['python3','/local_ai_worker/ffw_bringup/ffw_bringup/command_source_arbiter.py',
                         '--ros-args','-p','robot_model:=sg2','-p','manage_leader_service:=false'])
        start('launch',['ros2','launch','official_relative_teleop','mini_sg2.launch.py',
                        'use_mock_hardware:=true','use_sensorxel:=true','launch_foot_switch:=false',
                        'left_joystick_output_topic:=/intervention/teleop/head',
                        'right_joystick_output_topic:=/intervention/teleop/lift',
                        'cmd_vel_output_topic:=/intervention/teleop/cmd_vel',
                        'tact_trigger_output_topic:=/intervention/teleop/tact_trigger'])
        rig.until(lambda:bool(rig.arbiter) and rig.source=='model_action',timeout=30.)
        rig.spin(3.)
        rig.until(lambda:all(rig.get_publishers_info_by_topic('/intervention/teleop/'+name)
                             for name in ('head','lift','cmd_vel','tact_trigger')))
        for topic in ('/cmd_vel','/leader/joystick_controller/tact_trigger',
                      '/leader/joystick_controller_left/joint_trajectory',
                      '/leader/joystick_controller_right/joint_trajectory'):
            assert not rig.get_publishers_info_by_topic(topic), f'auxiliary isolation failed: {topic}'
        results['intervention_auxiliary_topics_isolated']=True
        assert not any(rig.final.values()), 'commands before intervention was prepared'
        results['no_output_before_prepare']=True
        rig.service('prepare_intervention')
        rig.until(lambda:rig.arbiter.get('mode')=='TELEOP' and rig.source=='teleop' and rig.control.active_control_mode==2,30.)
        rig.until(lambda:all(s in rig.poses for s in SIDES),30.)
        rig.spin(1.)
        results['initial_arm_jump_rad']=distance(rig.initial,rig.q)
        assert results['initial_arm_jump_rad']<.01, results
        assert rig.control.requested_arms=='none', rig.control
        before=rig.snapshot()
        rig.button('left',1)
        rig.until(lambda:rig.control.active_arms=='left')
        rig.spin(.5)
        results['enable_jump_rad']=distance(before,rig.q)
        assert results['enable_jump_rad']<.01,results
        for i in range(40):
            rig.leader[1]=.04*(i+1)/40
            rig.spin(.025)
        rig.spin(1.)
        left_motion=max(abs(x-y) for x,y in zip(before['left'][:7],rig.q['left']))
        right_motion=max(abs(x-y) for x,y in zip(before['right'][:7],rig.q['right']))
        results.update(left_motion_rad=left_motion,disabled_right_drift_rad=right_motion)
        assert .001<left_motion<.3 and right_motion<.01,results
        grip_before=rig.q['left'][7]
        arms_before_grip=rig.snapshot()
        rig.trigger_pub.publish(Float64MultiArray(data=[0.,.25]))
        rig.spin(1.)
        results['gripper_delta']=rig.q['left'][7]-grip_before
        results['gripper_inputs']={'raw':rig.raw_gripper,'adjusted':rig.action_gripper}
        assert .15<results['gripper_delta']<.30,results
        results['gripper_only_arm_drift_rad']=distance(arms_before_grip,rig.q)
        assert results['gripper_only_arm_drift_rad']<.01,results
        rig.button('left',0)
        rig.until(lambda:rig.control.requested_arms=='none')
        rig.spin(.3)
        paused=rig.snapshot()
        rig.leader[1]=.4
        rig.spin(.7)
        results['disabled_motion_rad']=distance(paused,rig.q)
        assert results['disabled_motion_rad']<.01,results
        rig.button('left',1)
        rig.until(lambda:rig.control.active_arms=='left')
        rig.spin(.5)
        results['reanchor_jump_rad']=distance(paused,rig.q)
        assert results['reanchor_jump_rad']<.01,results
        # Policy moves both arms while the leader moves elsewhere; TELEOP must
        # restart at the new measured state with both arms disabled.
        rig.policy_q=rig.snapshot()
        rig.gripper_deflection={'left':-.05,'right':-.03}
        rig.policy_enabled=True
        rig.spin(.3)
        rig.service('select_policy')
        rig.until(lambda:rig.arbiter.get('mode')=='POLICY' and rig.source=='model_action')
        for i in range(40):
            for s in SIDES:
                rig.policy_q[s][0]+=.001
            rig.leader[1]-=.005
            rig.spin(.025)
        rig.spin(.3)
        takeover=rig.snapshot()
        rig.service('select_teleop')
        rig.until(lambda:rig.arbiter.get('mode')=='TELEOP' and rig.source=='teleop')
        rig.spin(1.)
        results['policy_to_teleop_jump_rad']=distance(takeover,rig.q)
        assert results['policy_to_teleop_jump_rad']<.01,results
        results['loaded_gripper_handoff_error']=max(abs(takeover[s][-1]-rig.q[s][-1]) for s in SIDES)
        assert results['loaded_gripper_handoff_error']<1e-6,results
        rig.gripper_deflection={s:0. for s in SIDES}
        assert rig.control.requested_arms=='none', rig.control
        # Existing saved-pose buttons remain available through official presets.
        for s in SIDES:
            rig.button(s,3)
        rig.until(lambda:rig.control.left_preset_state==rig.control.PRESET_HOLDING
                  and rig.control.right_preset_state==rig.control.PRESET_HOLDING,25.)
        results['preset_3_both_arms']=True
        rig.button('right',4)
        rig.until(lambda:rig.control.right_preset_id==4 and
                  rig.control.right_preset_state==rig.control.PRESET_HOLDING,25.)
        results['preset_4_right_arm']=True
        # Exercise the actual local gripper edge-trigger, including a long hold.
        rig.trigger_pub.publish(Float64MultiArray(data=[-.8,.25]))
        rig.until(lambda:rig.control.right_preset_id==3 and
                  rig.control.right_preset_state==rig.control.PRESET_HOLDING,25.)
        rig.spin(2.2)
        assert rig.control.right_initial_pose_state==rig.control.INITIAL_POSE_UNUSED,rig.control
        results['gripper_saved_pose_shortcut']=True
        rig.button('right',4)
        rig.until(lambda:rig.control.right_preset_id==4 and
                  rig.control.right_preset_state==rig.control.PRESET_MOVING)
        rig.button('right',0)
        rig.until(lambda:rig.source=='model_action')
        rig.until(lambda:rig.source=='teleop' and rig.control.requested_arms=='none'
                  and rig.control.right_preset_state==rig.control.PRESET_UNUSED)
        rig.spin(.2)
        stopped=rig.snapshot()
        rig.spin(.4)
        assert distance(stopped,rig.q)<.01, 'preset continued after stop'
        results['stop_cancels_preset']=True
        assert not rig.arbiter.get('publisher_conflict'),rig.arbiter
        assert not rig.arbiter.get('fault'),rig.arbiter
        results['single_final_publisher']=all(len(rig.get_publishers_info_by_topic(
            f'/leader/joint_trajectory_command_broadcaster_{s}/joint_trajectory'))==1 for s in SIDES)
        assert results['single_final_publisher']
        rig.feed_enabled=False
        rig.spin(1.)
        counts={s:len(rig.final[s]) for s in SIDES}
        rig.spin(.4)
        assert counts=={s:len(rig.final[s]) for s in SIDES}, 'output continued after stale feedback'
        assert rig.arbiter.get('mode')=='NONE',rig.arbiter
        results['stale_feedback_stops_output']=True
        results['passed']=True
    except BaseException as error:
        results.update(passed=False,error=repr(error),arbiter=rig.arbiter,control=str(rig.control))
        raise
    finally:
        (logs/'results.json').write_text(json.dumps(results,indent=2)+'\n')
        print(json.dumps(results,indent=2),flush=True)
        for process in processes:
            if process.poll() is None:
                # ros2 launch forwards SIGINT to its children itself.
                process.send_signal(signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL)
        for handle in handles: handle.close()
        rig.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
