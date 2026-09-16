"""Reproduce known defects in commit 17af2f8; only sends frames on vcan0.

This is an audit snapshot, not a passing safety acceptance test. Source the
workspace install/setup.bash first. See README.md in this directory.
"""
import sys, threading, socket, struct, time, subprocess, math
import json, tempfile
from pathlib import Path
import yaml
root = Path(__file__).resolve().parents[3]
link = json.loads(subprocess.check_output(['ip', '-details', '-json', 'link', 'show', 'vcan0']))
assert link[0].get('linkinfo', {}).get('info_kind') == 'vcan', 'requires virtual CAN, never physical CAN'
temporary = tempfile.TemporaryDirectory(prefix='pi_arm_src_audit_')
work = Path(temporary.name)
probe = work / 'probe'
calibration = work / 'zero.yaml'
recorded = yaml.safe_load((root / 'src/pi_arm_hardware/config/pack_reference_calibration.R02.recorded.yaml').read_text())
recorded['calibration'].update(robot_id='VCAN-ONLY', calibration_id='VCAN-ONLY-ZERO',
    packing_pose_joint_rad=[0.0]*6, raw_output_at_packing_rad=[0.0]*6)
calibration.write_text(yaml.safe_dump(recorded))
packages = ['pi_arm_hardware', 'pi_arm_can', 'pi_arm_transmission']
subprocess.run(['g++', '-std=c++17', str(Path(__file__).with_name('probe_backend.cpp')),
    *['-I'+str(root/'src'/p/'include') for p in packages],
    *['-L'+str(root/'install'/p/'lib') for p in packages],
    *['-l'+p for p in packages], '-pthread', '-o', str(probe)], check=True)
sys.path.insert(0,str(root/'scripts'))
from vcan_motor_emulator import Motor, response, CAN_FRAME, CAN_BASE
bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
bus.bind(('vcan0',)); bus.settimeout(.1)
motors={i:Motor(enabled=True) for i in range(1,7)}
frames=[]; running=True; lock=threading.Lock()
def serve():
    while running:
        try: raw=bus.recv(16)
        except TimeoutError: continue
        can_id,n,p=CAN_FRAME.unpack(raw); i=can_id-CAN_BASE
        if i not in motors or n!=8:continue
        with lock:
            frames.append((i,p))
            reply=response(motors[i],bytes([0x9c])+bytes(7) if p[0]==0xa4 else p)
        if reply is not None:bus.send(CAN_FRAME.pack(CAN_BASE+i,8,reply))
t=threading.Thread(target=serve);t.start()
def start(mode):
    p=subprocess.Popen([str(probe),str(root),str(calibration),mode],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    assert p.stdout.readline().strip()=='READY'
    return p
def cmd(p,s):
    p.stdin.write(s+'\n');p.stdin.flush()
    out=p.stdout.readline().strip(); print(out,flush=True);return out
def finish(p):
    p.stdin.write('quit\n');p.stdin.flush();p.wait(timeout=3)
def setangles(values):
    with lock:
        for i,a in enumerate(values,1):motors[i].angle_raw=round(a*100)
def commands():
    with lock:return [(i,struct.unpack('<H',p[2:4])[0],struct.unpack('<i',p[4:8])[0]/100) for i,p in frames if p[0]==0xa4]
p=None
try:
    print('CASE 1: Feedback outside joint_05 limit (35 > 30 deg)')
    setangles([0,0,0,-6*180/math.pi*(33.94/16.25)*math.sin(math.radians(35)),0,0])
    p=start('normal');out=cmd(p,'read');assert 'kin=1 control=1' in out;finish(p);p=None
    print('CASE 2: Normal disarm does not transmit stop/hold')
    setangles([0]*6);frames.clear();p=start('normal');cmd(p,'write');time.sleep(.15)
    before=commands();out=cmd(p,'disarm');time.sleep(.2)
    after=commands()
    with lock: stop=[(i,hex(d[0])) for i,d in frames if d[0] in (0x80,0x81)]
    print('A4 before=',before,'A4 after=',after,'stop_frames=',stop,flush=True)
    assert len(before)==6 and after==before and not stop
    finish(p);p=None
    print('CASE 3: Raw motor 4 requests .05 deg/s, wire encodes 1/6 deg/s')
    frames.clear();p=start('commissioning');cmd(p,'motor');time.sleep(.1)
    a4=commands();print('A4=',a4,'output_speed_deg_s=',a4[0][1]/6,flush=True)
    assert a4[0][0]==4 and a4[0][1]==1
    cmd(p,'stop');finish(p);p=None
    print('CASE 4: Accepted joint_02 test violates held joint_05 during quantized trajectory')
    q5=math.radians(29.99)
    start4=-6*180/math.pi*(33.94/16.25)*math.sin(q5)
    setangles([0,0,0,start4,0,0]);frames.clear();p=start('commissioning')
    out=cmd(p,'joint');assert 'active=1' in out
    time.sleep(.05);a4=commands();print('A4=',a4,flush=True)
    # Model independent constant-speed motors obeying the actual integer A4 fields.
    # Inject the predicted .5 s sample; do not send another position command.
    angles=[0,0,0,round(start4*100)/100,0,0]
    for i,speed,target in a4:
        delta=target-angles[i-1]
        angles[i-1]+=math.copysign(min(abs(delta),speed*.5),delta)
    setangles(angles)
    out=cmd(p,'update')
    assert float(out.split('q5_deg=')[1].split()[0])>30 and 'active=1' in out and 'kin=1' in out
    cmd(p,'stop');finish(p);p=None
finally:
    if p is not None:p.kill();p.wait()
    running=False;t.join();bus.close();temporary.cleanup()
