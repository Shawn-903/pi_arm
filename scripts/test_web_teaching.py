#!/usr/bin/env python3
"""Teaching UI + ROS integration. Only mock; serve static assets temporarily."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time

from playwright.sync_api import sync_playwright

ROOT=Path(__file__).resolve().parents[1]
class Quiet(SimpleHTTPRequestHandler):
    def log_message(self,*args):pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ws',default='ws://127.0.0.1:8766')
    parser.add_argument('--screenshots',type=Path)
    args=parser.parse_args()
    server=ThreadingHTTPServer(('127.0.0.1',0),partial(Quiet,directory=str(ROOT/'web')))
    threading.Thread(target=server.serve_forever,daemon=True).start()
    prefix='test-'+str(time.time_ns())
    try:
        with sync_playwright() as p:
            browser=p.chromium.launch(headless=True,args=['--enable-unsafe-swiftshader'])
            page=browser.new_page(viewport=dict(width=1440,height=1100))
            errors,sent=[],[]
            page.on('pageerror',lambda e:errors.append(str(e)))
            page.on('websocket',lambda ws:ws.on('framesent',lambda raw:sent.append(json.loads(raw))))
            url=f'http://127.0.0.1:{server.server_port}/control.html?ws={args.ws}'
            page.goto(url)
            page.locator('#connectBtn').click()
            page.wait_for_function('lastState?.points_available && lastState?.base_frames?.valid')
            assert page.evaluate('lastState.hardware_type')=='mock','Only mock is allowed'
            assert page.evaluate('lastState.motor_enabled.every(v=>v===false)'), 'Start with a fresh disabled mock'
            page.wait_for_function('PiArmViewer.snapshot().ready',timeout=120000)
            assert page.locator('#baseFrameBody tr').count()==6
            assert '--' not in page.locator('#baseFrameBody').inner_text()
            page.locator('[data-view-target="points"]').click()
            page.wait_for_function('!document.getElementById("saveManualPoint").disabled')
            def card(name):return page.locator('.point-card').filter(has=page.locator('strong',has_text=prefix+name))
            def saved(name):page.wait_for_function('n=>[...document.querySelectorAll(".point-card strong")].some(x=>x.textContent===n)',arg=prefix+name)
            page.locator('#pointName').fill(prefix+' joint')
            for i,q in enumerate([.3,.2,-.1,0,0,0],1):page.locator(f'#pointValue{i}').fill(str(q))
            page.locator('#saveManualPoint').click();saved(' joint')
            page.locator('#pointName').fill(prefix+' feedback')
            page.locator('#capturePoint').click();saved(' feedback')
            page.locator('#pointKind').select_option('cartesian')
            page.locator('#pointName').fill(prefix+' cart')
            page.locator('#capturePoint').click();saved(' cart')
            assert not any(x['cmd'] in ['move','movel','motor_move','enable_motor','disable_motor'] for x in sent)
            print('PASS manual and disabled feedback capture, joint/cartesian point types, no motion/power commands',flush=True)
            page.reload();page.locator('#connectBtn').click()
            page.wait_for_function('lastState?.points_available')
            page.locator('[data-view-target="points"]').click();saved(' joint')
            card(' joint').locator('[data-point-action="load"]').click()
            assert page.locator('#multi').is_visible() and page.locator('#cartesian').is_hidden()
            assert page.locator('#multiJoint1').input_value()=='0.3'
            assert page.locator('#targetX').is_hidden()
            page.locator('[data-view-target="points"]').click()
            card(' cart').locator('[data-point-action="load"]').click()
            assert page.locator('#cartesian').is_visible() and page.locator('#multi').is_hidden()
            assert page.locator('#multiJoint1').is_hidden()
            assert not any(x['cmd'] in ['move','movel','motor_move','enable_motor','disable_motor'] for x in sent)
            print('PASS durable reload and type-specific load only, isolated motion forms',flush=True)
            # Existing finite-gesture API updates the displayed preview; saving must use
            # the full displayed pose, not potentially stale numeric single-axis inputs.
            page.wait_for_function('PiArmViewer.snapshot().valid',timeout=120000)
            page.evaluate('''() => {
              document.getElementById('target2').value='50';
              const owner=document.getElementById('jointSlider1');
              if(!PiArmInteractive.beginGesture(0,0,owner)) throw Error('gesture unavailable');
              PiArmInteractive.updateGesture(.4,owner);PiArmInteractive.endGesture(false,owner);
            }''')
            expected=page.evaluate('PiArmViewer.snapshot().previewTargetAngles')
            assert expected and expected[1]!=50
            page.locator('[data-view-target="points"]').click()
            page.locator('#pointName').fill(prefix+' model')
            page.locator('#saveModelPoint').click();saved(' model')
            card(' model').locator('[data-point-action="load"]').click()
            assert [float(page.locator(f'#multiJoint{i}').input_value()) for i in range(1,7)]==expected
            page.evaluate('q=>{targetsInitialized=false;PiArmControl.loadPointTarget("joint",q);handleMessage(lastState)}',expected)
            assert [float(page.locator(f'#multiJoint{i}').input_value()) for i in range(1,7)]==expected
            assert not any(x['cmd'] in ['move','movel','motor_move','enable_motor','disable_motor'] for x in sent)
            print('PASS model preview saves all six preview angles, not stale form values',flush=True)
            # Execute one finite mock joint target; Cartesian values must not interfere.
            page.locator('[data-cmd="enable_motor"][data-motor-id="0"]').click()
            page.wait_for_function('lastState.robot_state==="READY"')
            page.evaluate('document.getElementById("targetX").value="100000"')
            previous=page.evaluate('lastState.task.task_id')
            page.locator('#multiMoveBtn').click()
            page.wait_for_function('id=>lastState.task.task_id>id&&lastState.task.status===2',arg=previous,timeout=30000)
            assert abs(page.evaluate('lastState.current_joint_angles[0]')-.4)<.01
            request=[x for x in sent if x['cmd']=='move'][-1]
            assert request['params']['targets']==expected and 'target' not in request['params']
            print('PASS loaded mock joint execution ignores Cartesian target and follows normal task chain',flush=True)
            page.locator('[data-view-target="points"]').click()
            with page.expect_download() as download:page.locator('#exportPoints').click()
            backup=json.loads(Path(download.value.path()).read_text())
            assert any(x['name']==prefix+' model' for x in backup['points'])
            for width,height in [(1440,1100),(390,844)]:
                page.set_viewport_size(dict(width=width,height=height))
                page.evaluate('window.scrollTo(0,0);document.getElementById("pointList").scrollTop=0')
                assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
                if args.screenshots:
                    args.screenshots.mkdir(parents=True,exist_ok=True)
                    page.screenshot(path=str(args.screenshots/f'teaching-{width}.png'),full_page=True)
            for name in [' joint',' feedback',' cart',' model']:
                card(name).locator('[data-point-action="delete"]').click()
                page.wait_for_function('n=>![...document.querySelectorAll(".point-card strong")].some(x=>x.textContent===n)',arg=prefix+name)
            page.locator('[data-view-target="workbench"]').click()
            page.locator('#disconnectBtn').click()
            page.wait_for_function('connectionState==="closed"')
            assert page.locator('#baseFrameBody td').first.inner_text()=='--'
            assert not errors,errors
            browser.close()
            print('PASS export/delete, responsive layout and stale base-frame suppression',flush=True)
    finally:server.shutdown();server.server_close()


if __name__=='__main__':main()
