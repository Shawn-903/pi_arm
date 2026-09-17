import asyncio
import json
import math
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest
from pi_arm_interfaces.msg import RobotState
from pi_arm_websocket.frames import FrameModel, pose_record, quaternion
from pi_arm_websocket.points import PointStore
from pi_arm_websocket.presets import JOINT_NAMES
from pi_arm_websocket.gateway import CommandGateway, WebSocketGateway
from pi_arm_websocket.protocol import Request, ProtocolError

ROOT = Path(__file__).resolve().parents[3]
# Independent matrix-chain oracle; production uses quaternion composition.
import xml.etree.ElementTree as ET


def reference(angles):
    root=ET.parse(ROOT/'src/pi_arm_description/urdf/pi_arm.urdf.xacro').getroot()
    joints={j.get('name'):j for j in root.findall('joint')}
    q=dict(zip(JOINT_NAMES,angles));matrices={'base_link':np.eye(4)}
    def rotation(axis,angle):
        axis=np.array(axis,dtype=float);axis/=np.linalg.norm(axis)
        x,y,z=axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)
    def angle(name):
        if name in q:return q[name]
        m=joints[name].find('mimic')
        return 0 if m is None else angle(m.get('joint'))*float(m.get('multiplier','1'))+float(m.get('offset','0'))
    pending=dict(joints)
    while pending:
        for name,j in list(pending.items()):
            parent=j.find('parent').get('link')
            if parent not in matrices:continue
            origin=j.find('origin');M=np.eye(4)
            if origin is not None:
                M[:3,3]=[float(v) for v in origin.get('xyz','0 0 0').split()]
                r,p,y=[float(v) for v in origin.get('rpy','0 0 0').split()]
                M[:3,:3]=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
            A=np.eye(4)
            if j.get('type')!='fixed':A[:3,:3]=rotation([float(v) for v in j.find('axis').get('xyz').split()],angle(name))
            matrices[j.find('child').get('link')]=matrices[parent]@M@A
            del pending[name]
    return matrices


@pytest.fixture
def model():
    return FrameModel(ROOT/'src/pi_arm_description/urdf/pi_arm.urdf.xacro',JOINT_NAMES)


def test_all_base_frames_against_independent_matrix_fk(model):
    rng=np.random.default_rng(20260917)
    for q in [[0.]*6,*rng.uniform(-.4,.4,(20,6)).tolist()]:
        result=model.forward(q);expected=reference(q)
        for frame in [*result['joints'],result['tool']]:
            matrix=expected[frame['frame_id']]
            assert frame['position_mm']==pytest.approx(matrix[:3,3]*1000,abs=1e-8)
            # Compare orientation via rotating basis vectors, not Euler representations.
            from pi_arm_websocket.frames import rotate
            for i in range(3):
                assert rotate(frame['quaternion_xyzw'],np.eye(3)[:,i])==pytest.approx(matrix[:3,i],abs=1e-10)


@pytest.mark.parametrize('pitch',[90,-90,89.999,-89.999])
def test_rpy_reconstructs_quaternion_at_singularities(pitch):
    q=quaternion([math.radians(v) for v in (27,pitch,-42)])
    result=pose_record('test',[0,0,0],q)
    reconstructed=quaternion([math.radians(v) for v in result['rpy_deg']])
    assert abs(sum(a*b for a,b in zip(q,reconstructed)))==pytest.approx(1.,abs=1e-10)


class Bridge:
    hardware_type='mock'; read_only=True; commissioning=False
    def __init__(self, model, path):
        self.frame_model=model;self.points=PointStore(path);self.state=RobotState()
        self.state.state=RobotState.DISABLED;self.state.state_name='DISABLED'
        h=self.state.hardware;h.connected=h.calibrated=h.kinematics_valid=True
        h.joint_names=JOINT_NAMES;h.positions=[.01,.02,-.03,.04,-.05,.06]
        h.fresh=h.motor_fresh=[True]*6;h.motor_moving=h.motor_enabled=h.motor_errors=[False]*6
        h.motor_error_codes=[0]*6;h.raw_protocol_positions_deg=[1.]*6;h.calibration_id='test-cal'
    def state_snapshot(self):return self.state
    async def send_goal(self,*args):raise AssertionError('teaching never moves')
    async def call_service(self,*args):raise AssertionError('teaching never changes motor power')


def call(bridge,cmd='point_save',**params):
    return asyncio.run(CommandGateway(bridge).dispatch(Request(1,cmd,params)))


def test_disabled_readonly_capture_and_disk_round_trip(model,tmp_path):
    b=Bridge(model,tmp_path/'points.json')
    data=call(b,name='repeat A',kind='joint',source='feedback',revision=0)
    p=data['points'][0]
    assert p['values']==pytest.approx([math.degrees(v) for v in b.state.hardware.positions])
    assert p['calibration_id']=='test-cal' and p['raw_protocol_positions_deg']==[1.]*6
    assert p['orientation_xyzw']==pytest.approx(model.forward(b.state.hardware.positions)['tool']['quaternion_xyzw'])
    assert PointStore(b.points.path).list()==data
    cart=call(b,name='line B',kind='cartesian',source='feedback',revision=1)['points'][1]
    assert cart['values']==pytest.approx(p['pose_mm_deg'])
    state=json.loads(WebSocketGateway(b)._state_message())
    assert state['base_frames']['valid'] and state['points_available']
    assert state['pose']['x']==pytest.approx(p['pose_mm_deg'][0])
    assert call(b,'point_delete',id=p['id'],revision=2)['revision']==3


@pytest.mark.parametrize('fault',['stale','moving','task','invalid','disconnected','motor_error','override'])
def test_capture_rejects_unusable_feedback(model,tmp_path,fault):
    b=Bridge(model,tmp_path/'points.json');h=b.state.hardware;extra={}
    if fault=='stale':h.motor_fresh=[False]*6
    if fault=='moving':h.motor_moving=[True]*6
    if fault=='task':b.state.task.status=1
    if fault=='invalid':h.kinematics_valid=False
    if fault=='disconnected':b.state=None
    if fault=='motor_error':h.motor_error_codes=[8]*6
    if fault=='override':extra['values']=[0]*6
    with pytest.raises(ProtocolError):call(b,name='bad',kind='joint',source='feedback',revision=0,**extra)
    assert not b.points.path.exists()


def test_manual_validation_conflict_and_corrupt_file(model,tmp_path):
    b=Bridge(model,tmp_path/'points.json')
    for values in ([0]*5,[0]*5+[float('nan')],[0]*5+[True]):
        with pytest.raises(ProtocolError):call(b,name='invalid',kind='joint',values=values,revision=0)
    args=dict(name='manual',kind='cartesian',values=[100,50,600,45,45,-45],revision=0)
    data=call(b,**args)
    with pytest.raises(ProtocolError,match='已变化'):call(b,**args)
    with pytest.raises(ProtocolError,match='名称已存在'):call(b,**dict(args,revision=1))
    assert b.points.list()==data
    b.points.path.write_text('{broken')
    with pytest.raises(ProtocolError,match='损坏'):call(b,**dict(args,revision=1))
    assert b.points.path.read_text()=='{broken'


def test_competing_writers_do_not_overwrite(model,tmp_path):
    path=tmp_path/'points.json'
    def writer(i):
        b=Bridge(model,path)
        try:return call(b,name=str(i),kind='joint',values=[0]*6,revision=0)
        except ProtocolError:return None
    with ThreadPoolExecutor(max_workers=2) as executor:results=list(executor.map(writer,[1,2]))
    assert sum(r is not None for r in results)==1
    assert len(PointStore(path).list()['points'])==1
