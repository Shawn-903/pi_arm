#!/usr/bin/env python3
"""Offline UI speed contract: intercept every WebSocket; no ROS or CAN commands."""
import json
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
from playwright.sync_api import sync_playwright
from test_web_viewer import fixture


class Quiet(SimpleHTTPRequestHandler):
    def log_message(self, *args): pass


def main():
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(
        Quiet, directory=str(Path(__file__).resolve().parents[1] / 'web')))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True, args=['--enable-unsafe-swiftshader'])
            page = browser.new_page(viewport=dict(width=1440, height=1100))
            sent, errors = [], []
            page.on('pageerror', lambda error: errors.append(str(error)))
            def route(socket):
                def receive(raw):
                    request = json.loads(raw)
                    sent.append(request)
                    socket.send(json.dumps(dict(type='rsp', msg_id=request['msg_id'], code=0,
                                                ms='ok', data=dict(cmd=request['cmd']))))
                socket.on_message(receive)
            page.route_web_socket('**/*', route)
            page.goto(f'http://127.0.0.1:{server.server_port}/control.html')
            page.locator('#connectBtn').click()
            state = fixture([0]*6)
            page.evaluate('s=>{window.fake=s;window.tick=setInterval(()=>handleMessage(fake),100);handleMessage(s)}', state)
            page.wait_for_function('!document.getElementById("moveZeroPose").disabled')
            ids = ['outputSpeedLimit', 'multiSpeed', 'angularSpeed', 'presetSpeed',
                   'motorTestSpeed', 'jointTestSpeed', *[f'speed{i}' for i in range(1, 7)]]
            for name in ids:
                control = page.locator('#'+name)
                assert control.input_value() == '5', name
                assert control.get_attribute('max') == '10', name
            for name, speed in [('packing', 10), ('zero', 5)]:
                page.locator('#presetSpeed').fill(str(speed))
                page.locator('#outputSpeedLimit').fill('10')
                page.locator(f'[data-preset="{name}"]').click()
                page.wait_for_function('pending.size===0')
                assert sent[-1]['cmd'] == 'move_preset'
                assert sent[-1]['params'] == dict(preset=name, max_speed=speed, output_speed_limit=10)
            before = len(sent)
            page.locator('#presetSpeed').fill('10.001')
            page.locator('#moveZeroPose').click()
            assert len(sent) == before
            page.locator('#presetSpeed').fill('5')
            page.locator('#outputSpeedLimit').fill('10.001')
            page.locator('#moveZeroPose').click()
            assert len(sent) == before
            assert not errors, errors
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
    print('PASS angular inputs default 5/max 10, adjustable packing/zero payload, over-limit UI rejection; no robot commands')


if __name__ == '__main__':
    main()
