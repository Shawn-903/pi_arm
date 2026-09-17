#!/usr/bin/env python3
"""Offline drag-teaching UI regression: intercepted WebSocket, no robot connection."""
import json
from pathlib import Path
from playwright.sync_api import sync_playwright


def main():
    state = dict(type='state', hardware_type='mock', state_available=True, hardware_connected=True,
                 read_only=False, commissioning=False, robot_state='READY', control_permitted=True,
                 calibrated=True, kinematics_valid=True, motor_ids=list(range(1,7)),
                 motor_enabled=[True]*6, motor_fresh=[True]*6, motor_in_motion=[False]*6,
                 current_joint_angles=[0]*6, enabled=[True]*6, has_error=[False]*6,
                 task=dict(task_id=0,status=0), teaching_active=False,teaching_available=True,
                 supported_motion_commands=['move','movel'])
    with sync_playwright() as p:
        browser=p.chromium.launch(headless=True)
        page=browser.new_page()
        errors,sent,sockets=[],[],[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        def route(ws):
            sockets.append(ws)
            def receive(raw):
                req=json.loads(raw);sent.append(req)
                ws.send(json.dumps(dict(type='rsp',msg_id=req['msg_id'],code=0,ms='ok',data=dict(cmd=req['cmd']))))
            ws.on_message(receive)
            ws.send(json.dumps(state))
        page.route_web_socket('**/*',route)
        page.goto((Path(__file__).resolve().parents[1]/'web/control.html').as_uri())
        page.locator('#connectBtn').click()
        page.wait_for_function('lastState?.teaching_available')
        button=page.locator('#dragTeachingBtn')
        assert button.is_enabled()
        page.once('dialog',lambda d:d.dismiss())
        button.click()
        assert not sent
        page.once('dialog',lambda d:d.accept())
        button.click()
        page.wait_for_function('pending.size===0')
        assert [(x['cmd'],x['params']) for x in sent]==[('set_teaching',{'enabled':True})]
        state.update(teaching_active=True,control_permitted=False,robot_state='DISABLED',motor_enabled=[True]*6)
        sockets[0].send(json.dumps(state))
        page.wait_for_function('lastState?.teaching_active')
        assert button.get_attribute('aria-pressed')=='true'
        assert page.locator('[data-cmd="enable_motor"][data-motor-id="0"]').is_disabled()
        assert page.locator('#multiMoveBtn').is_disabled()
        button.click()
        page.wait_for_function('pending.size===0')
        assert sent[-1]['cmd']=='set_teaching' and sent[-1]['params']=={'enabled':False}
        state.update(teaching_active=False,control_permitted=True,robot_state='READY',motor_enabled=[True]*6)
        sockets[0].send(json.dumps(state))
        page.wait_for_function('lastState?.teaching_active===false')
        assert button.get_attribute('aria-pressed')=='false'
        # The stale gate is refreshed by the UI timer; allow its next tick on busy Pis.
        page.wait_for_function('document.getElementById("dragTeachingBtn").disabled', timeout=5000)
        assert page.evaluate('Date.now()-lastStateAt >= 1500')
        assert button.is_disabled(), 'stale state must forbid teaching switches'
        assert not errors,errors
        browser.close()
    print('PASS drag teaching confirmation, explicit commands, feedback state, motion/enable/stale gates')


if __name__=='__main__':main()
