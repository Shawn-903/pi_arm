#!/usr/bin/env python3
"""Reproduce cached teaching.js with new HTML, then verify versioned assets fix it.

All WebSockets are intercepted. The HTTP fixture intentionally gives old JS a
long cache lifetime, as on a client that retained assets across this UI upgrade.
"""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import threading
from urllib.parse import urlsplit
from playwright.sync_api import sync_playwright
from test_web_viewer import fixture, NAMES

ROOT = Path(__file__).resolve().parents[1]
requests = []

class CachedAssets(SimpleHTTPRequestHandler):
    def log_message(self, *args): pass

    def do_GET(self):
        requests.append(self.path)
        url = urlsplit(self.path)
        content = None
        if url.path == '/control.html' and url.query == 'legacy=1':
            content = re.sub(r'\?v=[^"\s]+', '', (ROOT/'web/control.html').read_text())
        elif url.path == '/assets/teaching.js' and not url.query:
            content = (ROOT/'web/assets/teaching.js').read_text().replace(
                '    window.PiArmRepeat?.setLibrary(library);\n', '')
        if content is None:
            return super().do_GET()
        raw = content.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html' if url.path.endswith('.html') else 'text/javascript')
        self.send_header('Cache-Control', 'public, max-age=86400' if url.path.endswith('.js') else 'no-store')
        self.send_header('Content-Length', str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)


def main():
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(CachedAssets, directory=str(ROOT/'web')))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    state = fixture()
    state.update(points_available=True, motor_in_motion=[False]*6, base_frames=dict(
        model_id='model', valid=True, tool=dict(position_mm=[0]*3, rpy_deg=[0]*3),
        joints=[dict(frame_id=n, position_mm=[0]*3, rpy_deg=[0]*3) for n in NAMES]))
    points = [dict(id=str(i), name=name, kind='joint', values=[i]*6, source='manual',
                   hardware_type='mock', model_id='model', calibration_id=state['calibration_id'],
                   base_frame='base_link', tool_frame='tool0', joint_names=NAMES,
                   created_at='2026-09-20T00:00:00Z')
              for i, name in enumerate(['零点A', '重复定位A', '重复点位B'])]
    library = dict(revision=3, points=points)
    sent, errors = [], []
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True, args=['--enable-unsafe-swiftshader'])
            page = browser.new_page()
            page.on('pageerror', lambda error: errors.append(str(error)))
            def route(socket):
                def receive(raw):
                    req = json.loads(raw); sent.append(req)
                    assert req['cmd'] == 'point_list', 'This test must not send motion or mutate points'
                    socket.send(json.dumps(dict(type='rsp', msg_id=req['msg_id'], code=0, ms='ok', data=library)))
                socket.on_message(receive)
            page.route_web_socket('**/*', route)
            def load(suffix):
                page.goto(f'http://127.0.0.1:{server.server_port}/control.html{suffix}')
                page.locator('#connectBtn').click()
                page.evaluate('s=>{window.fake=s;window.tick=setInterval(()=>handleMessage(fake),100);handleMessage(fake)}', state)
                page.locator('[data-view-target="points"]').click()
                page.wait_for_function('document.querySelectorAll(".point-card").length===3')
            load('?legacy=1')
            assert page.locator('#repeatStartPoint option').count() == 1
            assert page.locator('#repeatTargetPoint option').count() == 1
            print('PASS reproduced: cards contain 3 saved points but old cached teaching.js never populates selectors', flush=True)
            load('')
            for element in ('repeatStartPoint', 'repeatTargetPoint'):
                assert page.locator(f'#{element} option').all_text_contents() == [
                    '请选择点位', '零点A · 六关节', '重复定位A · 六关节', '重复点位B · 六关节']
            for asset in ('control.js', 'repeat.js', 'teaching.js'):
                assert any(url.startswith('/assets/'+asset+'?v=') for url in requests), asset
            page.locator('#repeatStartPoint').select_option('0')
            page.locator('#repeatTargetPoint').select_option('1')
            page.locator('#refreshPoints').click()
            page.wait_for_function('pending.size===0')
            assert page.locator('#repeatStartPoint').input_value() == '0'
            assert page.locator('#repeatTargetPoint').input_value() == '1'
            # New/deleted points propagate to both selectors without a page refresh.
            library['points'] = points[:2]
            library['revision'] = 4
            page.locator('#refreshPoints').click()
            page.wait_for_function('document.getElementById("repeatStartPoint").options.length===3')
            library['points'] = points
            library['revision'] = 5
            page.locator('#refreshPoints').click()
            page.wait_for_function('document.getElementById("repeatTargetPoint").options.length===4')
            assert not errors, errors
            assert all(req['cmd'] == 'point_list' for req in sent)
            browser.close()
            print('PASS versioned scripts bypass old cache; both selectors list saved points and retain selection on refresh; read-only requests only', flush=True)
    finally:
        server.shutdown(); server.server_close()

if __name__ == '__main__': main()
