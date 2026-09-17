#!/usr/bin/env python3
"""Test production RealBackend reboot on unused vcan0; never binds physical CAN.
Source the intended install/setup.bash first. Does not change calibration files.
"""
import json
import runpy
import socket
import subprocess
import tempfile
import threading
import time
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_prefix
from vcan_motor_emulator import CAN_BASE, CAN_FRAME, Motor, response

ROOT=Path(__file__).resolve().parents[1]

class Emulator:
    def __init__(self,mode):
        self.mode=mode;self.running=True;self.frames=[];self.boot_at=0
        self.motors={i:Motor(enabled=True,error=8 if i==2 else 0) for i in range(1,7)}
        self.bus=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
        self.bus.bind(('vcan0',));self.bus.settimeout(.05)
        self.thread=threading.Thread(target=self.run);self.thread.start()
    def run(self):
        while self.running:
            try: raw=self.bus.recv(CAN_FRAME.size)
            except TimeoutError: continue
            can,length,data=CAN_FRAME.unpack(raw);i=can-CAN_BASE
            if i not in self.motors or length!=8:continue
            self.frames.append((i,data))
            if i==2 and self.boot_at and (self.mode=='timeout' or time.monotonic()-self.boot_at<.2):continue
            if data[0]==7:
                self.boot_at=time.monotonic()
                response(self.motors[i],data)
                if self.mode=='jump':self.motors[i].angle_raw=360000
                if self.mode=='persistent':self.motors[i].error=8
                if self.mode=='off_ignored':self.motors[i].enabled=True
                continue # Protocol explicitly specifies NO ACK.
            if self.mode=='off_ignored' and data[0]==0x80:continue
            reply=response(self.motors[i],data)
            if reply is not None:self.bus.send(CAN_FRAME.pack(CAN_BASE+i,8,reply))
    def close(self):
        self.running=False;self.thread.join();self.bus.close()

def main():
    info=json.loads(subprocess.check_output(['ip','-details','-json','link','show','vcan0']))[0]
    assert info['linkinfo']['info_kind']=='vcan'
    with tempfile.TemporaryDirectory(prefix='pi_arm_reboot_') as folder:
        work=Path(folder);cal=work/'zero.yaml'
        data=yaml.safe_load((ROOT/'docs/calibration-history/pack_reference_calibration.R02.recorded.yaml').read_text())
        cable=ROOT/'src/pi_arm_description/config/cable_kinematics.yaml'
        candidate=runpy.run_path(str(ROOT/'src/pi_arm_hardware/scripts/pi_arm_calibration_candidate'))
        # Mirror the actual actuator mappings and current cable config.
        mapping=[dict(joint_name=name,motor_id=i,protocol_angle_ratio=36.0 if i<=3 else 6.0,motor_direction=d)
                 for i,(name,d) in enumerate(zip(candidate['JOINT_NAMES'],[1,1,-1,-1,-1,1]),1)]
        data['calibration'].update(robot_id='VCAN-ONLY',calibration_id='REBOOT-VCAN-ONLY',
                                  packing_pose_joint_rad=[0.]*6,raw_output_at_packing_rad=[0.]*6,
                                  configuration_fingerprint=candidate['configuration_fingerprint'](cable,mapping))
        cal.write_text(yaml.safe_dump(data))
        packages=['pi_arm_hardware','pi_arm_can','pi_arm_transmission'];exe=work/'probe'
        subprocess.run(['g++','-std=c++17',str(ROOT/'src/pi_arm_hardware/test/vcan_reboot_probe.cpp'),
                        *['-I'+str(ROOT/'src'/p/'include') for p in packages],
                        *['-L'+str(Path(get_package_prefix(p))/'lib') for p in packages],
                        *['-l'+p for p in packages],'-pthread','-o',str(exe)],check=True)
        for mode in ('recover','timeout','jump','persistent','off_ignored'):
            emulator=Emulator(mode)
            try:
                output=subprocess.check_output([str(exe),str(ROOT),str(cal)],text=True,timeout=15)
                assert 'success='+('1' if mode=='recover' else '0') in output,output
                assert 'armed=0' in output,output
                reboot=[i for i,d in emulator.frames if d[0]==7]
                assert reboot==[2],reboot
                assert not any(d[0] in (0xa4,0x88,0x19,0x95) for _,d in emulator.frames)
                if mode=='recover':assert 'enabled=0' in output and 'error=NONE' in output,output
                else:assert 'REBOOT_REFERENCE_UNVERIFIED' in output and 'enable_rejected=1' in output,output
                print('PASS',mode,output.strip(),flush=True)
            finally:emulator.close()

if __name__=='__main__':main()
