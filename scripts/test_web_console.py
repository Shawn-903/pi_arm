#!/usr/bin/env python3
"""Offline console interaction/layout checks. WebSockets are intercepted; no CAN/ROS."""
import argparse
import json
from pathlib import Path
from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--browser-path')
    args = parser.parse_args()
    state = dict(type='state', hardware_type='mock', state_available=True,
                 hardware_connected=True, robot_state='READY', read_only=False,
                 commissioning=False, calibrated=True, kinematics_valid=True,
                 control_permitted=True, calibration_id='MOCK-UI-TEST',
                 motor_ids=list(range(1, 7)), motor_fresh=[True]*6,
                 motor_in_motion=[False]*6, reboot_available=True,
                 motor_enabled=[True]*6, enabled=[True]*6,
                 motor_error_codes=[0]*6, motor_ages_sec=[.01]*6,
                 has_error=[False]*6, in_motion=[False]*6,
                 current_joint_angles=[10, 35, -25, 8, -12, 15], current_speeds=[0]*6,
                 model_output_positions_rad=[.1]*6, raw_output_positions_rad=[.1]*6,
                 raw_protocol_positions_deg=[1]*6, software_output_offsets_rad=[0]*6,
                 task=dict(task_id=0, status=0, progress=0, command='', message='idle'),
                 kinematics_error=dict(code='NONE', axis=0, message=''),
                 supported_motion_commands=['move','motor_move','output_move','movel'],
                 pose=dict(x=100,y=20,z=500,rx=0,ry=0,rz=0),
                 preset_poses={name:dict(available=True,reason='',joint_speed_deg_s=1,
                                        target_joint_angles_deg=target)
                               for name,target in [('zero',[0]*6),('packing',[0,100.7,-85.6,0,0,0])]})
    with sync_playwright() as p:
        browser=p.chromium.launch(headless=True, executable_path=args.browser_path)
        page=browser.new_page(viewport={'width':1440,'height':1000})
        errors,sent=[],[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        def route(socket):
            socket.on_message(lambda message:sent.append(json.loads(message)))
            socket.send(json.dumps(state))
        page.route_web_socket('**/*',route)
        page.goto((Path(__file__).resolve().parents[1]/'web/control.html').as_uri())
        assert page.locator('#globalStop').is_disabled()
        assert page.locator('[data-view-target="diagnostics"]').is_hidden()
        page.locator('#connectBtn').click()
        page.wait_for_function('lastState?.robot_state === "READY"')
        page.evaluate('''s => {window.__fixture=s; window.__ticker=setInterval(()=>handleMessage(window.__fixture),200)}''',state)
        assert 'MOCK' in page.locator('#consoleMode').inner_text()
        assert page.locator('[data-overview-axis="1"] .joint-value>span').inner_text()=='35.00'
        assert page.locator('#movePackingPose').is_enabled()
        for width,height in [(1440,1000),(1024,768),(390,844)]:
            page.set_viewport_size(dict(width=width,height=height))
            page.locator('#viewRole').select_option('developer')
            for view in ('workbench','records','diagnostics','commissioning'):
                page.locator(f'[data-view-target="{view}"]').click()
                assert page.locator(f'[data-view-panel="{view}"]').is_visible()
                assert page.evaluate('document.documentElement.scrollWidth <= innerWidth'), (width,view)
                page.locator('#viewContent').scroll_into_view_if_needed()
                assert page.locator('#globalStop').is_visible()
            page.locator('#viewRole').select_option('operator')
            assert page.locator('[data-view-panel="workbench"]').is_visible()
            assert page.locator('[data-view-target="diagnostics"]').is_hidden()
        assert sent==[], 'Navigation must not emit robot commands'
        page.set_viewport_size(dict(width=1440,height=1000))
        for tab in ('single','multi','outputControlCard'):
            page.locator(f'[data-tab="{tab}"]').click()
            assert page.locator('#'+tab).is_visible()
            assert page.locator(f'[data-tab="{tab}"]').get_attribute('aria-selected')=='true'
        page.evaluate('''() => {clearInterval(window.__ticker);lastStateAt=Date.now()-2000;refreshControlGate()}''')
        assert page.locator('[data-overview-axis="1"] .joint-value>span').inner_text()=='--'
        assert page.locator('#endX').input_value()=='--'
        assert page.locator('#moveOutputTargets').is_disabled()
        assert page.locator('#globalStop').is_enabled(), 'Stale feedback must not disable an available software stop'
        page.evaluate('s=>handleMessage({...s,robot_state:"FAULT",kinematics_valid:false,control_permitted:false,kinematics_error:{code:"ASIN_DOMAIN",axis:4,message:"offline fixture"}})',state)
        assert page.locator('#operatorNotice').get_attribute('class')=='notice error'
        assert page.locator('#moveZeroPose').is_disabled()
        page.locator('#inspectNotice').click()
        assert page.locator('[data-view-panel="diagnostics"]').is_visible()
        page.locator('#globalStop').click()
        page.wait_for_function('pending.size > 0')
        assert len(sent)==1 and sent[0]['cmd']=='stop_motion'
        page.evaluate('r=>handleMessage({type:"rsp",msg_id:r.msg_id,code:0,ms:"ok",data:{}})',sent[-1])
        page.locator('[data-view-target="workbench"]').click()
        assert page.locator('#rebootMotorBtn').is_enabled(), 'FAULT must allow recovery without kinematics'
        page.locator('#recoveryMotorId').select_option('2')
        page.locator('#rebootMotorBtn').click()
        page.wait_for_function('pending.size > 0')
        assert len(sent)==2 and sent[-1]['cmd']=='reboot_motor' and sent[-1]['params']['motor_id']==2
        assert page.locator('#rebootMotorBtn').is_disabled()
        assert page.locator('#clearMotorFaultBtn').is_disabled()
        page.evaluate('s=>handleMessage(s)',state)
        assert page.locator('#moveZeroPose').is_disabled(), 'Recovery must block new motion even with a READY sample'
        assert page.locator('button[data-cmd="enable_motor"]').first.is_disabled()
        page.evaluate('r=>handleMessage({type:"rsp",msg_id:r.msg_id,code:2002,ms:"position reference unverified",data:{}})',sent[-1])
        assert 'position reference unverified' in page.locator('#recoveryResult').inner_text()
        page.locator('#rebootMotorBtn').click()
        page.wait_for_function('pending.size > 0')
        page.evaluate('r=>handleMessage({type:"rsp",msg_id:r.msg_id,code:0,ms:"ok",data:{message:"fresh disabled feedback confirmed"}})',sent[-1])
        assert '电机保持关闭' in page.locator('#recoveryResult').inner_text()
        for changes in (dict(motor_fresh=[False]*6), dict(motor_in_motion=[True]+[False]*5),
                        dict(motor_in_motion=[]), dict(hardware_connected=False),
                        dict(task=dict(task_id=0,status=1)), dict(reboot_available=False)):
            page.evaluate('s=>handleMessage(s)',{**state,**changes})
            assert page.locator('#rebootMotorBtn').is_disabled(), changes
        page.evaluate('s=>handleMessage({...s,read_only:true,control_permitted:false})',state)
        assert page.locator('#globalStop').is_disabled()
        assert page.locator('#rebootMotorBtn').is_disabled()
        page.locator('#disconnectBtn').click()
        page.wait_for_function('connectionState === "closed"')
        assert page.locator('#globalStop').is_disabled()
        assert not errors,errors
        browser.close()
    print('PASS console navigation, responsive layouts, role views, tabs, stale/fault/read-only gates and software stop routing')


if __name__=='__main__':
    main()
