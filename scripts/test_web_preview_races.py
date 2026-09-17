#!/usr/bin/env python3
"""Offline adversarial preview/gesture checks. Intercepts all WS; no ROS or CAN."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from playwright.sync_api import sync_playwright
from test_web_viewer import fixture


class Quiet(SimpleHTTPRequestHandler):
    def log_message(self, *args): pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--browser-path')
    args=parser.parse_args()
    server=ThreadingHTTPServer(('127.0.0.1',0),partial(Quiet,directory=str(Path(__file__).resolve().parents[1]/'web')))
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        with sync_playwright() as pw:
            browser=pw.chromium.launch(headless=True,executable_path=args.browser_path,args=['--enable-unsafe-swiftshader'])
            page=browser.new_page(viewport=dict(width=1440,height=1100))
            errors,sent,sockets=[],[],[]
            page.on('pageerror',lambda e:errors.append(str(e)))
            def route(socket):
                sockets.append(socket)
                socket.on_message(lambda raw:sent.append(json.loads(raw)))
            page.route_web_socket('**/*',route)
            page.goto(f'http://127.0.0.1:{server.server_port}/control.html')
            page.wait_for_function('PiArmViewer.snapshot().ready',timeout=120000)
            page.locator('#connectBtn').click()
            state=fixture([0]*6);state['preview_modes']=['joint','output','ik','lin']
            page.evaluate('s=>{window.fake=s;window.tick=setInterval(()=>handleMessage(fake),100);handleMessage(s)}',state)
            page.wait_for_function('!document.getElementById("previewOutput").disabled')
            def response(request, target=1):
                angles=[target,0,0,0,0,0]
                return dict(type='rsp',msg_id=request['msg_id'],code=0,ms='ok',data=dict(
                    mode='output',joint_names=state['joint_names'],start_joint_angles_deg=[0]*6,
                    target_joint_angles_deg=angles,target_output_angles_deg=angles,
                    points=[dict(t=0,joints=[0]*6,tcp=[0,0,1]),dict(t=1,joints=angles,tcp=[.1,0,1])]))
            page.locator('#previewOutput').click()
            page.wait_for_function('pending.size===1',timeout=5000)
            page.locator('[data-tab="outputControlCard"]').click()
            page.locator('#outputTarget1').fill('2')
            sockets[0].send(json.dumps(response(sent[-1])))
            page.wait_for_function('pending.size===0')
            assert not page.evaluate('PiArmViewer.snapshot().previewVisible')
            assert page.evaluate('PiArmViewer.snapshot().pathPointCount')==0
            print('PASS late response after input edit cannot restore the old preview',flush=True)

            page.locator('#previewOutput').click()
            page.wait_for_function('pending.size===1',timeout=5000)
            sockets[0].send(json.dumps(response(sent[-1],target=2)))
            page.wait_for_function('PiArmViewer.snapshot().pathPointCount===2')
            # Copy-like programmatic input changes emit no input event.
            page.evaluate('document.getElementById("outputTarget1").value="3"')
            page.wait_for_function('PiArmViewer.snapshot().pathPointCount===0')
            page.locator('#previewOutput').click()
            page.wait_for_function('pending.size===1',timeout=5000)
            page.locator('#clearPreview').click()
            sockets[0].send(json.dumps(response(sent[-1],target=3)))
            page.wait_for_function('pending.size===0')
            assert not page.evaluate('PiArmViewer.snapshot().previewVisible')
            print('PASS programmatic edits and explicit clear invalidate path and pending response',flush=True)

            page.locator('#releaseExecute').check()
            page.evaluate("""() => {
                const a=document.getElementById('jointSlider1'), b=document.getElementById('jointSlider2');
                const model=document.querySelector('#viewerCanvas canvas');
                if(!PiArmInteractive.beginGesture(0,0,a)) throw Error('first gesture failed');
                if(PiArmInteractive.beginGesture(1,0,b)) throw Error('second pointer took over');
                PiArmInteractive.updateGesture(1,a);
                PiArmInteractive.endGesture(false,b);
                PiArmInteractive.endGesture(true);
                PiArmInteractive.beginGesture(0,0,model);
                PiArmInteractive.updateGesture(1,model);
                PiArmInteractive.endGesture(false,a);
                PiArmInteractive.endGesture(true);
            }""")
            assert not any(x['cmd']=='motor_move' for x in sent),sent
            print('PASS overlapping gestures and old control releases cannot submit another gesture',flush=True)

            page.locator('[data-tab="single"]').click()
            page.locator('#releaseExecute').check()
            page.locator('#jointSlider1').focus()
            page.keyboard.down('ArrowRight')
            page.evaluate('sendCommand("stop_motion")')
            page.locator('#jointSlider1').dispatch_event('keydown',dict(key='ArrowRight',repeat=True))
            page.keyboard.up('ArrowRight')
            assert not page.locator('#releaseExecute').is_checked()
            assert not any(x['cmd']=='motor_move' for x in sent),sent
            print('PASS Stop during a held gesture prevents a new goal on release',flush=True)

            page.evaluate('fake.read_only=true;fake.control_permitted=false;handleMessage(fake)')
            assert page.locator('#jointSlider1').is_disabled()
            assert page.locator('#previewIK').is_disabled()
            page.evaluate('fake.read_only=false;fake.control_permitted=true;handleMessage(fake)')
            page.wait_for_function('!document.getElementById("releaseExecute").disabled')
            page.locator('#releaseExecute').check()
            page.locator('#jointSlider1').focus();page.keyboard.down('ArrowRight')
            page.evaluate('clearInterval(tick);lastStateAt=Date.now()-2500;refreshControlGate()')
            page.keyboard.up('ArrowRight')
            assert not page.locator('#releaseExecute').is_checked()
            assert not any(x['cmd']=='motor_move' for x in sent),sent
            assert not errors,errors
            print('PASS read-only and stale feedback disarm gestures; no motion requests or JS errors',flush=True)
            browser.close()
    finally:server.shutdown();server.server_close()


if __name__=='__main__':main()
