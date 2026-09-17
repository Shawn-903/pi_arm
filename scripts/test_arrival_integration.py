#!/usr/bin/env python3
"""Adversarial end-to-end manager arrival test. Synthetic services only, domain 82.

Source an isolated install; starts ONLY manager with fake feedback/controller/arm
services. Never launches a CAN driver, opens CAN, or executes MoveIt trajectories.
"""
import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from pi_arm_interfaces.action import DirectMove
from pi_arm_interfaces.msg import HardwareState, RobotState
from pi_arm_interfaces.srv import SetMotionArmed
from rclpy.qos import qos_profile_sensor_data
import xacro
import yaml

NAMES=['joint_01','joint_02','joint_04','joint_05','joint_06','joint_09']


class Fixture(Node):
    def __init__(self):
        super().__init__('arrival_test_fixture')
        self.group=ReentrantCallbackGroup()
        self.positions=[0.]*6;self.mode='offset';self.goal=None;self.result_state=None
        self.sent_at=0.;self.disarmed_at=None;self.armed=False;self.stable_from=None
        self.pub=self.create_publisher(HardwareState,'/test/hardware_state',qos_profile_sensor_data)
        self.create_timer(.02,self.publish,callback_group=self.group)
        self.create_subscription(RobotState,'/pi_arm/state',self.record,10,callback_group=self.group)
        self.create_service(SetMotionArmed,'/pi_arm/hardware/set_motion_armed',self.arm,callback_group=self.group)
        self.server=ActionServer(self,FollowJointTrajectory,'/test/follow',execute_callback=self.follow,callback_group=self.group)
        # MoveGroupInterface waits for these servers at construction; no planning
        # or execution call is used by DirectMove in this fixture.
        self.move_server=ActionServer(self,MoveGroup,'/move_action',execute_callback=lambda h: self.unexpected(h,MoveGroup),callback_group=self.group)
        self.execute_server=ActionServer(self,ExecuteTrajectory,'/execute_trajectory',execute_callback=lambda h: self.unexpected(h,ExecuteTrajectory),callback_group=self.group)
        self.client=ActionClient(self,DirectMove,'/pi_arm/direct_move',callback_group=self.group)

    def unexpected(self,handle,action):
        handle.abort()
        raise AssertionError('Unexpected planner/execution call in arrival-only fixture')

    def record(self,msg):self.result_state=msg

    def arm(self,request,response):
        response.success=True;response.armed=request.armed;response.message='synthetic arm service'
        if request.armed:
            self.armed=True;self.disarmed_at=None
        elif self.armed:
            self.armed=False;self.disarmed_at=time.monotonic()
            if self.mode=='drift':self.positions[0]+=math.radians(1)
        return response

    def follow(self,handle):
        self.goal=handle.request
        assert len(self.goal.goal_tolerance)==6
        assert all(abs(t.position-math.radians(.2))<1e-12 for t in self.goal.goal_tolerance)
        target=list(self.goal.trajectory.points[-1].positions)
        self.positions=target
        if self.mode in ('offset','settle'):
            self.positions[0]+=math.radians(1)
        self.sent_at=time.monotonic()
        handle.succeed()  # Intentionally lie: emulate the old false-positive controller.
        return FollowJointTrajectory.Result(error_code=0,error_string='fake controller success')

    def publish(self):
        if self.mode=='settle' and self.goal and time.monotonic()-self.sent_at>.35:
            self.positions=list(self.goal.trajectory.points[-1].positions)
        m=HardwareState();m.stamp=self.get_clock().now().to_msg();m.connected=True
        m.driver_mode='CONTROL' if self.armed else 'MONITORING';m.joint_names=NAMES
        m.positions=self.positions;m.velocities=[0.]*6
        m.enabled=[True]*6;m.errors=[False]*6;m.moving=[False]*6;m.fresh=[True]*6
        m.error_codes=[0]*6;m.ages_sec=[0.]*6;m.motor_ids=list(range(1,7))
        m.motor_enabled=[True]*6;m.motor_errors=[False]*6;m.motor_moving=[False]*6
        m.motor_fresh=[True]*6;m.motor_error_codes=[0]*6;m.motor_ages_sec=[0.]*6
        m.calibrated=True;m.kinematics_valid=True;m.control_permitted=True
        m.raw_protocol_positions_deg=[0.]*6;m.raw_output_positions_rad=[0.]*6
        self.pub.publish(m)


def await_future(f,timeout=10):
    deadline=time.monotonic()+timeout
    while not f.done() and time.monotonic()<deadline:time.sleep(.01)
    assert f.done(),'future timeout'
    return f.result()


def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='82','Run only in isolated domain 82'
    rclpy.init(domain_id=82)
    node=Fixture();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    desc=Path(get_package_share_directory('pi_arm_description'))
    moveit=Path(get_package_share_directory('pi_arm_moveit_config'))
    params={
        'robot_description':xacro.process_file(str(desc/'urdf/pi_arm.urdf.xacro'),mappings={'backend':'mock','cable_kinematics_file':str(desc/'config/cable_kinematics.yaml')}).toxml(),
        'robot_description_semantic':(moveit/'config/pi_arm.srdf').read_text(),
        'robot_description_kinematics':yaml.safe_load((moveit/'config/kinematics.yaml').read_text()),
        'robot_description_planning':yaml.safe_load((moveit/'config/joint_limits.yaml').read_text()),
        'hardware_state_topic':'/test/hardware_state','follow_joint_trajectory_action':'/test/follow',
        'arrival_stable_time_sec':.2,'arrival_timeout_sec':1.0,
    }
    with tempfile.TemporaryDirectory() as tmp:
        config=Path(tmp)/'manager.yaml';config.write_text(yaml.safe_dump({'pi_arm_manager':{'ros__parameters':params}}))
        executable=Path(get_package_prefix('pi_arm_manager'))/'lib/pi_arm_manager/pi_arm_manager_node'
        log=open('/tmp/pi_arm_arrival_fake_manager.log','w')
        process=subprocess.Popen([str(executable),'--ros-args','--params-file',str(config)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            assert node.client.wait_for_server(timeout_sec=30),'manager not ready'
            for mode in ('offset','settle','drift'):
                node.mode=mode;node.goal=None;node.positions=[0.]*6;node.sent_at=0
                deadline=time.monotonic()+5
                while (node.result_state is None or node.result_state.state_name!='READY') and time.monotonic()<deadline:time.sleep(.05)
                time.sleep(.15)
                goal=DirectMove.Goal();goal.joint_names=NAMES;goal.positions=[0.]*6;goal.max_velocity=math.radians(1);goal.max_output_velocity=math.radians(1)
                handle=await_future(node.client.send_goal_async(goal));assert handle.accepted,mode
                result=await_future(handle.get_result_async(),10).result
                time.sleep(.15)
                arrival=node.result_state.task.arrival_state
                if mode=='settle':
                    assert result.success and arrival=='reached',(mode,result,arrival)
                    assert node.disarmed_at-node.sent_at>=.5, 'lease released before stable arrival'
                else:
                    assert not result.success and arrival=='not_reached',(mode,result,arrival)
                    assert 'GOAL_NOT_REACHED' in result.message,result.message
                print('PASS',mode,'success=',result.success,'arrival=',arrival,flush=True)
        finally:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
            log.close()
    executor.shutdown();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
