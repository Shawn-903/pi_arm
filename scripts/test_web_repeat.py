#!/usr/bin/env python3
"""Offline repeatability UI integration: intercept ALL sockets; never contact ROS/CAN."""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from playwright.sync_api import sync_playwright
from test_web_viewer import fixture, NAMES

ROOT = Path(__file__).resolve().parents[1]

class Quiet(SimpleHTTPRequestHandler):
    def log_message(self, *args): pass


def main():
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(Quiet, directory=str(ROOT/'web')))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True, args=['--enable-unsafe-swiftshader'])
            page = browser.new_page(viewport=dict(width=1440, height=1100))
            errors, sent, sockets, delayed = [], [], [], []
            page.on('pageerror', lambda e: errors.append(str(e)))
            state = fixture()
            state.update(points_available=True, motor_in_motion=[False]*6,
                         base_frames=dict(valid=True, model_id='test-model', joints=[dict(
                             frame_id=n, position_mm=[0]*3, rpy_deg=[0]*3) for n in NAMES],
                             tool=dict(position_mm=[1, 2, 3], rpy_deg=[0]*3)))
            points = [dict(id=str(i), name=f'点位 {i}', kind=kind, values=[i*.1]*6,
                           hardware_type='mock', model_id='test-model', calibration_id=state['calibration_id'],
                           base_frame='base_link', tool_frame='tool0', joint_names=NAMES,
                           source='manual', created_at='2026-09-20T00:00:00Z')
                      for i, kind in enumerate(['joint', 'joint', 'cartesian', 'cartesian', 'joint'])]
            points[4]['calibration_id'] = 'OLD'
            count, defer = 0, False
            def route(socket):
                sockets.append(socket)
                def receive(raw):
                    nonlocal count
                    req = json.loads(raw); sent.append(req)
                    data = dict(cmd=req['cmd'])
                    if req['cmd'].startswith('point_'):
                        data = dict(revision=1, points=points)
                    elif req['cmd'] in ('move', 'movel'):
                        count += 1; data['task_id'] = count
                    reply = json.dumps(dict(type='rsp', msg_id=req['msg_id'], code=0, ms='ok', data=data))
                    if defer and req['cmd'] in ('move', 'movel'): delayed.append(reply)
                    else: socket.send(reply)
                socket.on_message(receive)
            page.route_web_socket('**/*', route)
            page.goto(f'http://127.0.0.1:{server.server_port}/control.html?ws=ws://offline.invalid')
            page.locator('#connectBtn').click()
            page.evaluate('s=>{window.fake=s;window.tick=setInterval(()=>handleMessage(fake),80);handleMessage(fake)}', state)
            page.locator('[data-view-target="points"]').click()
            page.wait_for_function('document.getElementById("repeatStartPoint").options.length===6')
            moves = lambda: [r for r in sent if r['cmd'] in ('move', 'movel')]
            def configure(start='0', target='1', cycles=1, dwell=0.2):
                page.locator('#repeatStartPoint').select_option(start)
                page.locator('#repeatTargetPoint').select_option(target)
                page.locator('#repeatCycles').fill(str(cycles))
                page.locator('#repeatDwell').fill(str(dwell))
            def ready():
                page.evaluate('''() => {
                  fake.robot_state='READY';fake.task={task_id:0,status:0};
                  fake.motor_in_motion=Array(6).fill(false);handleMessage(fake);
                }''')
                page.wait_for_function('acceptedTaskId===null')
            def complete(status=2, arrival='reached'):
                page.evaluate('''p=>{
                  fake.task={task_id:p.id,status:p.status,command:'direct_move',arrival_state:p.arrival,
                    target_joint_angles_deg:Array(6).fill(0),error_joint_angles_deg:Array(6).fill(.001),
                    goal_tolerance_deg:.2,message:'fixture completion'};
                  fake.robot_state='READY';fake.motor_in_motion=Array(6).fill(false);handleMessage(fake);
                }''', dict(id=count, status=status, arrival=arrival))
            def wait_moves(n):
                for _ in range(100):
                    if len(moves()) >= n: return
                    page.wait_for_timeout(30)
                raise AssertionError((n, moves(), page.locator('#repeatStatus').inner_text(), errors))
            configure(target='4')
            assert page.locator('#repeatStart').is_disabled()
            configure(target='2')
            assert page.locator('#repeatStart').is_disabled()
            configure(target='0')
            assert page.locator('#repeatStart').is_disabled()
            configure(cycles=2)
            page.locator('#repeatCycles').fill('1.5')
            page.locator('#repeatStart').click()
            assert not moves()
            page.locator('#repeatCycles').fill('2')
            page.locator('#repeatOutputSpeed').fill('10')
            page.locator('#repeatStart').click(); wait_moves(1)
            assert moves()[0]['params'] == dict(targets=[0]*6,max_speed=5,output_speed_limit=10)
            assert page.locator('#repeatCycles').is_disabled()
            page.evaluate('sendCommand("move", {targets:Array(6).fill(0),max_speed:5})')
            page.wait_for_timeout(300)
            assert len(moves()) == 1, 'Must wait for completion; no other UI motion may interleave'
            # A previous successful task must not count as arrival of our accepted task.
            page.evaluate('fake.task={task_id:0,status:2,arrival_state:"reached"};handleMessage(fake)')
            page.wait_for_timeout(250); assert len(moves()) == 1
            for expected in (2, 3, 4):
                complete(); page.wait_for_timeout(60)
                assert len(moves()) == expected-1, 'Dwell begins after arrival'
                wait_moves(expected)
            complete()
            page.wait_for_function('!PiArmRepeat.active')
            assert '已完成 2 轮' in page.locator('#repeatStatus').inner_text()
            assert [m['params']['targets'] for m in moves()] == [[0]*6, [.1]*6, [0]*6, [.1]*6]
            with page.expect_download() as item: page.locator('#repeatExport').click()
            report = json.loads(Path(item.value.path()).read_text())
            assert report['result'] == 'completed' and len(report['samples']) == 2
            assert [s['task_id'] for s in report['samples']] == [2, 4]
            assert report['config']['output_speed'] == 10
            with page.expect_download() as item: page.locator('#repeatCsv').click()
            csv = Path(item.value.path()).read_text(encoding='utf-8-sig')
            assert len(csv.splitlines()) == 3 and '0.001' in csv
            print('PASS point/model/calibration guards, finite A-B cycles, exact task matching, dwell, speed snapshot, records/exports', flush=True)
            # STOP during dwell cancels the future target and routes the real stop API.
            configure(dwell=10); page.locator('#repeatStart').click(); wait_moves(5)
            complete(); page.wait_for_function('document.getElementById("repeatStatus").textContent.includes("已到位")')
            page.locator('#repeatStop').click(); page.wait_for_timeout(300)
            assert not page.evaluate('PiArmRepeat.active') and len(moves()) == 5
            assert sent[-1]['cmd'] == 'stop_motion'
            configure(); page.locator('#repeatStart').click(); wait_moves(6)
            complete(status=4, arrival='not_reached'); page.wait_for_function('!PiArmRepeat.active')
            assert len(moves()) == 6
            # A delayed successful acceptance after cancellation must not resurrect the loop.
            ready(); defer = True
            page.locator('#repeatStart').click(); wait_moves(7)
            page.locator('#repeatStop').click()
            sockets[-1].send(delayed.pop()); defer = False
            page.wait_for_timeout(300); complete()
            assert not page.evaluate('PiArmRepeat.active') and len(moves()) == 7
            print('PASS stop during dwell, failed arrival, and acceptance/stop race; no queued legs', flush=True)
            configure('2', '3', dwell=0)
            page.locator('#repeatStart').click(); wait_moves(8)
            assert moves()[-1]['cmd'] == 'movel'
            assert 'target' in moves()[-1]['params'] and 'targets' not in moves()[-1]['params']
            complete(); wait_moves(9); complete()
            page.wait_for_function('!PiArmRepeat.active')
            configure(); page.locator('#repeatStart').click(); wait_moves(10)
            page.evaluate('clearInterval(tick);lastStateAt=Date.now()-2000;refreshControlGate()')
            assert not page.evaluate('PiArmRepeat.active')
            complete()
            page.evaluate('window.tick=setInterval(()=>handleMessage(fake),80);handleMessage(fake)')
            page.wait_for_timeout(300); assert len(moves()) == 10
            print('PASS Cartesian payload, stale-feedback abort, no automatic resume', flush=True)
            # A different client's accepted task must never be counted as this run's task.
            page.locator('#repeatStart').click(); wait_moves(11)
            page.evaluate('id=>{fake.task={task_id:id+100,status:1};handleMessage(fake)}', count)
            page.wait_for_function('!PiArmRepeat.active'); complete()
            assert '其他运动任务' in page.locator('#repeatStatus').inner_text()
            # Changed calibration and hidden page both cancel the active run.
            page.locator('#repeatStart').click(); wait_moves(12)
            page.evaluate('fake.calibration_id="CHANGED";handleMessage(fake)')
            page.wait_for_function('!PiArmRepeat.active')
            page.evaluate('id=>{fake.calibration_id=id;handleMessage(fake)}', state['calibration_id'])
            complete()
            page.locator('#repeatStart').click(); wait_moves(13)
            page.evaluate('''() => {
              Object.defineProperty(document,'hidden',{configurable:true,value:true});
              document.dispatchEvent(new Event('visibilitychange'));
              delete document.hidden;
            }''')
            page.wait_for_function('!PiArmRepeat.active'); complete()
            print('PASS foreign task, changed calibration and page-background abort', flush=True)
            # Global STOP also terminates repetition, even while its current leg is pending.
            page.locator('#repeatStart').click(); wait_moves(14)
            page.evaluate('sendCommand("stop_motion")'); complete()
            page.wait_for_timeout(300); assert not page.evaluate('PiArmRepeat.active') and len(moves()) == 14
            # Closing the socket resets freshness and cannot restart the loop on reconnect.
            page.locator('#repeatStart').click(); wait_moves(15)
            page.evaluate('ws.close()')
            page.wait_for_function('!PiArmRepeat.active')
            assert '停止指令未能发送' in page.locator('#repeatStatus').inner_text()
            for width, height in [(1440,1100), (390,844)]:
                page.set_viewport_size(dict(width=width,height=height))
                assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
            page.screenshot(path='/tmp/pi-arm-repeat-mobile.png', full_page=True)
            assert not errors, errors
            browser.close()
            print('PASS global STOP, disconnect, responsive layout, no browser errors; all sockets intercepted', flush=True)
    finally:
        server.shutdown(); server.server_close()

if __name__ == '__main__': main()
