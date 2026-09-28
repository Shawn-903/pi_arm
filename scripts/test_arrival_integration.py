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
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from pi_arm_interfaces.action import DirectMove
from pi_arm_interfaces.msg import HardwareState, RobotState
from pi_arm_interfaces.srv import SetMotionArmed, PreviewMotion, StopMotion
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
        self.arm_requests=[];self.follow_count=0;self.hold_failed=False;self.drifted=False;self.faulted=False
        self.controller_requested=threading.Event();self.controller_canceled=threading.Event()
        self.pub=self.create_publisher(HardwareState,'/test/hardware_state',qos_profile_sensor_data)
        self.create_timer(.02,self.publish,callback_group=self.group)
        self.create_subscription(RobotState,'/pi_arm/state',self.record,10,callback_group=self.group)
        self.create_service(SetMotionArmed,'/pi_arm/hardware/set_motion_armed',self.arm,callback_group=self.group)
        self.server=ActionServer(self,FollowJointTrajectory,'/test/follow',execute_callback=self.follow,
                                goal_callback=self.accept_follow,
                                cancel_callback=lambda _: CancelResponse.ACCEPT,callback_group=self.group)
        # MoveGroupInterface waits for these servers at construction; no planning
        # or execution call is used by DirectMove in this fixture.
        self.move_server=ActionServer(self,MoveGroup,'/move_action',execute_callback=lambda h: self.unexpected(h,MoveGroup),callback_group=self.group)
        self.execute_server=ActionServer(self,ExecuteTrajectory,'/execute_trajectory',execute_callback=lambda h: self.unexpected(h,ExecuteTrajectory),callback_group=self.group)
        self.client=ActionClient(self,DirectMove,'/pi_arm/direct_move',callback_group=self.group)
        self.preview=self.create_client(PreviewMotion,'/pi_arm/preview_motion',callback_group=self.group)
        self.stop=self.create_client(StopMotion,'/pi_arm/stop_motion',callback_group=self.group)

    def accept_follow(self, _request):
        self.controller_requested.set()
        if self.mode=='late_stop':time.sleep(.7)
        return GoalResponse.ACCEPT

    def unexpected(self,handle,action):
        handle.abort()
        raise AssertionError('Unexpected planner/execution call in arrival-only fixture')

    def record(self,msg):self.result_state=msg

    def arm(self,request,response):
        self.arm_requests.append((request.armed,request.hold_position,request.lease_id))
        if not request.armed and request.hold_position and self.mode=='hold_failure':
            self.hold_failed=True
            response.success=False;response.armed=False;response.message='synthetic hold ACK loss'
            return response
        response.success=True;response.armed=request.armed;response.message='synthetic arm service'
        if request.armed:
            self.armed=True;self.disarmed_at=None
        elif self.armed:
            self.armed=False;self.disarmed_at=time.monotonic()
        return response

    def follow(self,handle):
        self.follow_count+=1
        self.goal=handle.request
        assert len(self.goal.goal_tolerance)==6
        assert all(abs(t.position-math.radians(.2))<1e-12 for t in self.goal.goal_tolerance)
        target=list(self.goal.trajectory.points[-1].positions)
        if self.mode in ('in_motion','late_stop','hold_failure'):
            self.positions=[.01]*6
            self.sent_at=time.monotonic()
            deadline=time.monotonic()+10
            while not handle.is_cancel_requested and time.monotonic()<deadline:
                time.sleep(.01)
            if handle.is_cancel_requested:
                self.controller_canceled.set();handle.canceled()
            else:handle.abort()
            return FollowJointTrajectory.Result(error_code=0,error_string='synthetic controller canceled')
        self.positions=target
        if self.mode in ('offset','settle'):
            self.positions[0]+=math.radians(1)
        self.sent_at=time.monotonic()
        handle.succeed()  # Intentionally lie: emulate the old false-positive controller.
        return FollowJointTrajectory.Result(error_code=0,error_string='fake controller success')

    def publish(self):
        # Arrival now retains the A4 loop instead of releasing it. Inject drift
        # during the held-position verification, not after a successful lease
        # release (the old stop/release-based fixture no longer models this).
        if self.mode=='drift' and not self.drifted and self.result_state and \
                self.result_state.task.arrival_state=='verifying_hold':
            self.positions[0]+=math.radians(1);self.drifted=True
        if self.mode=='settle' and self.goal and time.monotonic()-self.sent_at>.35:
            self.positions=list(self.goal.trajectory.points[-1].positions)
        m=HardwareState();m.stamp=self.get_clock().now().to_msg();m.connected=True
        m.driver_mode='CONTROL' if self.armed else 'MONITORING';m.joint_names=NAMES
        m.positions=self.positions;m.velocities=[0.]*6
        m.enabled=[True]*6;m.errors=[False]*6;m.moving=[False]*6;m.fresh=[True]*6
        m.error_codes=[0]*6;m.ages_sec=[0.]*6;m.motor_ids=list(range(1,7))
        m.motor_enabled=[True]*6;m.motor_errors=[False]*6;m.motor_moving=[False]*6
        m.motor_fresh=[True]*6;m.motor_error_codes=[0]*6;m.motor_ages_sec=[0.]*6
        if self.faulted:m.errors[0]=True;m.motor_errors[0]=True
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
            assert node.preview.wait_for_service(timeout_sec=5)
            time.sleep(.2)
            preview=PreviewMotion.Request(mode='output', positions=[0.]*6, max_velocity=math.radians(1))
            answer=await_future(node.preview.call_async(preview))
            assert answer.success and not node.armed and node.goal is None, answer
            assert len(answer.trajectory.points)==201
            print('PASS preview is read-only: no lease, controller or MoveIt execution',flush=True)
            for mode in ('offset','settle','drift'):
                node.mode=mode;node.goal=None;node.positions=[0.]*6;node.sent_at=0;node.drifted=False
                deadline=time.monotonic()+5
                while (node.result_state is None or node.result_state.state_name!='READY') and time.monotonic()<deadline:time.sleep(.05)
                time.sleep(.15)
                goal=DirectMove.Goal();goal.joint_names=NAMES;goal.positions=[0.]*6;goal.max_velocity=math.radians(1);goal.max_output_velocity=math.radians(1)
                handle=await_future(node.client.send_goal_async(goal));assert handle.accepted,mode
                if mode=='offset':
                    blocked=await_future(node.preview.call_async(preview))
                    assert not blocked.success and blocked.code==2003, blocked
                    print('PASS manager rejects preview during an active motion task',flush=True)
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

            for mode in ('in_motion','action_cancel','late_stop','hold_failure','hardware_fault'):
                node.mode='in_motion' if mode in ('action_cancel','hardware_fault') else mode
                node.armed=False;node.goal=None;node.positions=[0.]*6;node.hold_failed=False
                node.arm_requests.clear();node.controller_requested.clear();node.controller_canceled.clear()
                time.sleep(.4)
                goal=DirectMove.Goal();goal.joint_names=NAMES;goal.positions=[.1]*6
                goal.max_velocity=math.radians(1);goal.max_output_velocity=math.radians(1)
                handle=await_future(node.client.send_goal_async(goal));assert handle.accepted,mode
                assert node.controller_requested.wait(5),mode
                if mode=='hardware_fault':node.faulted=True
                elif mode=='action_cancel':
                    assert await_future(handle.cancel_goal_async()).goals_canceling
                else:
                    reply=await_future(node.stop.call_async(StopMotion.Request()))
                    assert reply.success == (mode!='hold_failure'),(mode,reply)
                result=await_future(handle.get_result_async(),10).result
                assert node.controller_canceled.wait(5), f'{mode}: controller survived Stop'
                time.sleep(.3)  # Include late result handling and the scope guard.
                holds=[r for r in node.arm_requests if not r[0]]
                assert holds and all(r[1]==(mode!='hardware_fault') and r[2]==0 for r in holds),(mode,node.arm_requests)
                if mode=='hold_failure':assert not result.success and result.code==3006,result
                elif mode=='hardware_fault':assert not result.success and result.code==3102,result
                elif mode=='action_cancel':assert result.success,result
                else:assert not result.success and result.code==3003,result
                print('PASS stop-and-hold',mode,node.arm_requests,flush=True)

            node.mode='settle';node.armed=False;node.hold_failed=False;node.faulted=False
            node.arm_requests.clear();time.sleep(.3)
            count=node.follow_count
            for _ in range(2):
                assert await_future(node.stop.call_async(StopMotion.Request())).success
            assert node.follow_count==count,'Stop generated an unwanted hold trajectory'
            assert node.arm_requests==[(False,True,0),(False,True,0)],node.arm_requests
            print('PASS idle/dwell repeated Stop holds without generating trajectories',flush=True)
        finally:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
            log.close()
    executor.shutdown();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
