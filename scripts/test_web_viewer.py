#!/usr/bin/env python3
"""Read-only 3D tests: independent URDF FK, mock TF and browser interactions.

Default: local static HTTP + intercepted WebSocket, no ROS/CAN.
--mock-url: additionally compare a running mock backend (refuses real hardware).
"""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import gzip
import hashlib
import json
import math
from pathlib import Path
import random
import threading
import xml.etree.ElementTree as ET

import numpy as np
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
NAMES = ['joint_01', 'joint_02', 'joint_04', 'joint_05', 'joint_06', 'joint_09']


def verify_assets():
    model = ROOT / 'web/model'
    manifest = json.loads((model/'manifest.json').read_text())
    assert manifest['source_sha256'] == hashlib.sha256((ROOT/manifest['source']).read_bytes()).hexdigest()
    assert manifest['urdf_sha256'] == hashlib.sha256((model/manifest['urdf']).read_bytes()).hexdigest()
    source = ET.parse(ROOT/manifest['source']).getroot()
    exported = ET.parse(model/manifest['urdf']).getroot()
    assert exported.find('ros2_control') is None
    assert manifest['joint_names'] == [j.attrib['name'] for j in source.find('ros2_control').findall('joint')]
    for original, actual in zip(source.findall('joint'), exported.findall('joint'), strict=True):
        assert original.attrib == actual.attrib
        assert [(e.tag, e.attrib) for e in original] == [(e.tag, e.attrib) for e in actual]
    for record in manifest['meshes']:
        original = (ROOT/record['source']).read_bytes()
        assert hashlib.sha256(original).hexdigest() == record['sha256']
        assert gzip.decompress((model/record['url']).read_bytes()) == original
    print('PASS exported model matches source joints, limits/mimics and all 10 meshes byte-for-byte', flush=True)


def rotation(axis, angle):
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew)


def forward(angles):
    """Independent matrix FK from source Xacro's plain URDF joints, not JS output."""
    root = ET.parse(ROOT/'src/pi_arm_description/urdf/pi_arm.urdf.xacro').getroot()
    joints = root.findall('joint')
    by_name = {j.attrib['name']: j for j in joints}
    q = dict(zip(NAMES, np.radians(angles)))

    def angle(name):
        if name in q:
            return q[name]
        mimic = by_name[name].find('mimic')
        if mimic is None:
            return 0
        return angle(mimic.attrib['joint'])*float(mimic.get('multiplier', 1))+float(mimic.get('offset', 0))

    result = {'base_link': np.eye(4)}
    pending = list(joints)
    while pending:
        progressed = False
        for joint in list(pending):
            parent = joint.find('parent').attrib['link']
            if parent not in result:
                continue
            origin = joint.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
            roll, pitch, yaw = [float(v) for v in origin.get('rpy', '0 0 0').split()]
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = rotation([0, 0, 1], yaw)@rotation([0, 1, 0], pitch)@rotation([1, 0, 0], roll)
            if joint.attrib['type'] != 'fixed':
                axis = [float(v) for v in joint.find('axis').attrib['xyz'].split()]
                transform[:3, :3] = transform[:3, :3]@rotation(axis, angle(joint.attrib['name']))
            result[joint.find('child').attrib['link']] = result[parent]@transform
            pending.remove(joint)
            progressed = True
        assert progressed, 'URDF tree cannot be resolved'
    return result


def fixture(angles=None):
    angles = angles or [0]*6
    tip = forward(angles)['tool0']
    roll = math.atan2(tip[2, 1], tip[2, 2])
    pitch = math.atan2(-tip[2, 0], math.hypot(tip[0, 0], tip[1, 0]))
    yaw = math.atan2(tip[1, 0], tip[0, 0])
    return dict(type='state', hardware_type='mock', state_available=True,
                hardware_connected=True, robot_state='READY', read_only=False,
                commissioning=False, calibrated=True, kinematics_valid=True,
                control_permitted=True, calibration_id='MOCK-VIEWER-TEST',
                joint_names=NAMES, joint_fresh=[True]*6, motor_ids=list(range(1, 7)),
                motor_fresh=[True]*6, motor_enabled=[True]*6, enabled=[True]*6,
                motor_error_codes=[0]*6, motor_ages_sec=[.01]*6,
                has_error=[False]*6, in_motion=[False]*6,
                current_joint_angles=angles, current_speeds=[0]*6,
                model_output_positions_rad=[0]*6, raw_output_positions_rad=[0]*6,
                raw_protocol_positions_deg=[0]*6, software_output_offsets_rad=[0]*6,
                task=dict(task_id=0, status=0, progress=0, command='', message='idle'),
                kinematics_error=dict(code='NONE', axis=0, message=''),
                supported_motion_commands=['move', 'motor_move', 'output_move', 'movel'],
                pose=dict(x=tip[0, 3]*1000, y=tip[1, 3]*1000, z=tip[2, 3]*1000,
                          rx=math.degrees(roll), ry=math.degrees(pitch), rz=math.degrees(yaw)),
                preset_poses={n: dict(available=True, reason='', joint_speed_deg_s=1,
                                     target_joint_angles_deg=q) for n, q in
                              [('zero', [0]*6), ('packing', [0, 100.7, -85.6, 0, 0, 0])]})


def verify_transforms(snapshot, angles):
    reference = forward(angles)
    assert set(snapshot['links']) == set(reference)
    for name, expected in reference.items():
        actual = np.array(snapshot['links'][name]).reshape((4, 4), order='F')
        assert np.allclose(actual, expected, atol=2e-12, rtol=0), (name, angles, np.max(abs(actual-expected)))


def run_fixture(browser, url, screenshots):
    page = browser.new_page(viewport=dict(width=1440, height=1100))
    errors, sent = [], []
    page.on('pageerror', lambda e: errors.append(str(e)))
    page.route_web_socket('**/*', lambda socket: (
        socket.on_message(lambda message: sent.append(json.loads(message))), socket.send(json.dumps(fixture()))))
    page.goto(url)
    page.wait_for_function('PiArmViewer.snapshot().ready', timeout=120000)
    snapshot = page.evaluate('PiArmViewer.snapshot()')
    assert snapshot['state'] == 'reference' and not snapshot['valid']
    manifest = json.loads((ROOT/'web/model/manifest.json').read_text())
    triangle_count = 0
    for record in manifest['meshes']:
        with (ROOT/record['source']).open('rb') as stream:
            triangle_count += int.from_bytes(stream.read(84)[80:84], 'little')
    assert snapshot['meshCount'] == len(manifest['meshes']) and snapshot['triangles'] == triangle_count
    verify_transforms(snapshot, [0]*6)
    page.locator('#connectBtn').click()
    page.wait_for_function('PiArmViewer.snapshot().valid')
    page.evaluate('''s=>{window.__viewerState=s;window.__ticker=setInterval(()=>handleMessage(window.__viewerState),150)}''', fixture())

    def update(state):
        return page.evaluate('s=>{window.__viewerState=s;handleMessage(s);return PiArmViewer.snapshot()}', state)

    samples = [[0]*6, [0, 100.7, -85.6, 0, 0, 0]]
    for axis in range(6):
        for sign in (-1, 1):
            q = [0]*6
            q[axis] = 5*sign
            samples.append(q)
    rng = random.Random(42)
    samples += [[rng.uniform(-v, v) for v in [120, 95, 80, 28, 58, 120]] for _ in range(20)]
    for q in samples:
        snapshot = update(fixture(q))
        assert snapshot['valid']
        verify_transforms(snapshot, q)
    print('PASS 34 source-URDF FK poses, all 11 links, 3 mimic joints, degree/radian conversion', flush=True)

    q = [12, 40, -22, 8, -12, 25]
    state = fixture(q)
    order = [4, 2, 0, 5, 3, 1]
    state['joint_names'] = [NAMES[i] for i in order]
    state['current_joint_angles'] = [q[i] for i in order]
    verify_transforms(update(state), q)
    for bad in [dict(joint_names=[]), dict(joint_names=[NAMES[0]]*6),
                dict(current_joint_angles=[0]*5), dict(state_available=False),
                dict(joint_fresh=[True, False, True, True, True, True]),
                dict(motor_fresh=[False]*6), dict(kinematics_valid=False),
                dict(kinematics_valid=False, kinematics_error=dict(code='ASIN_DOMAIN', axis=4, message='fixture'))]:
        snapshot = update({**fixture([30]*6), **bad})
        assert not snapshot['valid'] and snapshot['state'] == 'stale'
        verify_transforms(snapshot, q)
    snapshot = update(fixture([0, 102.7, -87.6, 0, 0, 0]))
    verify_transforms(snapshot, [0, 102.7, -87.6, 0, 0, 0])
    assert page.locator('#viewerJoints .viewer-warning').count() == 2
    update({**fixture(q), 'motor_error_codes': [0, 0, 0, 4, 0, 0], 'any_error': True})
    assert page.locator('#viewerJoints .viewer-fault').count() == 1
    assert '故障' in page.locator('#viewerStatus').inner_text()
    print('PASS name reordering, invalid/missing/stale feedback, frozen last pose, unclamped tolerances and fault highlighting', flush=True)

    update(fixture(q))
    page.locator('#viewerPreview').select_option('zero')
    snapshot = page.evaluate('PiArmViewer.snapshot()')
    assert snapshot['previewVisible']
    assert all(v == [0] for n, v in snapshot['previewJoints'].items() if n != 'tool0_fixed_joint')
    verify_transforms(snapshot, q)  # cloning/mimic updates must not mutate feedback
    page.locator('#viewerPreview').select_option('packing')
    snapshot = page.evaluate('PiArmViewer.snapshot()')
    assert math.isclose(snapshot['previewJoints']['joint_03'][0], math.radians(-85.6))
    verify_transforms(snapshot, q)
    page.locator('#viewerPreview').select_option('single')
    page.locator('#target4').fill('12')
    snapshot = page.evaluate('PiArmViewer.snapshot()')
    assert math.isclose(snapshot['previewJoints']['joint_08'][0], math.radians(-12))
    page.locator('#target2').fill('110')
    assert not page.evaluate('PiArmViewer.snapshot().previewVisible')
    assert 'J2' in page.locator('#viewerPreviewStatus').inner_text()
    page.locator('#target2').fill('')
    assert not page.evaluate('PiArmViewer.snapshot().previewVisible')
    page.locator('#target2').fill('0')
    page.locator('[data-tab="multi"]').click()
    page.locator('#multiJoint5').fill('21')
    page.locator('#viewerPreview').select_option('multi')
    assert math.isclose(page.evaluate('PiArmViewer.snapshot().previewJoints.joint_07[0]'), math.radians(21))
    page.locator('#viewerPreview').select_option('none')
    assert not page.evaluate('PiArmViewer.snapshot().previewVisible')
    for name in ('front', 'side', 'top', 'iso'):
        page.locator(f'[data-camera-view="{name}"]').click()
    page.locator('#viewerAxes').uncheck()
    page.locator('#viewerAxes').check()
    page.locator('#viewerGrid').uncheck()
    page.locator('#viewerGrid').check()
    page.locator('#viewerJoints button').nth(2).click()
    assert page.evaluate('PiArmViewer.snapshot().selected') == 2
    page.locator('#viewerLocate').click()
    assert page.locator('#target3').evaluate('e=>e===document.activeElement')
    assert page.locator('#single').is_visible()
    assert sent == [], sent
    print('PASS target preview isolation, limits, preset/mimic preview, camera/selection controls; zero robot commands', flush=True)

    for width, height in [(1440, 1100), (1024, 768), (390, 844)]:
        page.set_viewport_size(dict(width=width, height=height))
        page.locator('#viewRole').select_option('developer')
        for view in ('commissioning', 'workbench', 'diagnostics', 'workbench'):
            page.locator(f'[data-view-target="{view}"]').click()
            assert page.evaluate('document.documentElement.scrollWidth<=innerWidth'), (width, view)
        page.locator('#viewerFit').click()
        if screenshots:
            screenshots.mkdir(parents=True, exist_ok=True)
            page.locator('#robotViewer').screenshot(path=str(screenshots/f'viewer-{width}.png'))
    page.set_viewport_size(dict(width=1440, height=1100))
    update({**fixture(q), 'read_only': True, 'control_permitted': False})
    assert page.evaluate('PiArmViewer.snapshot().valid')
    assert page.locator('#globalStop').is_disabled()
    page.evaluate('''()=>{clearInterval(window.__ticker);lastStateAt=Date.now()-2500;refreshControlGate()}''')
    assert not page.evaluate('PiArmViewer.snapshot().valid')
    verify_transforms(page.evaluate('PiArmViewer.snapshot()'), q)
    page.locator('#viewerPreview').select_option('packing')
    assert not page.evaluate('PiArmViewer.snapshot().previewVisible')
    page.locator('#disconnectBtn').click()
    assert '最后有效姿态' in page.locator('#viewerStatus').inner_text()
    assert sent == [], sent
    assert not errors, errors
    print('PASS responsive 1440/1024/390, commissioning panel reuse, read-only view, timeout/disconnect; zero robot commands', flush=True)
    page.evaluate('document.querySelector("#viewerCanvas canvas").getContext("webgl2").getExtension("WEBGL_lose_context").loseContext()')
    page.wait_for_function('PiArmViewer.snapshot().failed')
    assert page.locator('#connectBtn').is_enabled()
    assert not errors, errors
    print('PASS WebGL context loss isolated from connection and numeric controls', flush=True)
    page.close()

    # Asset/WebGL failures are local to visualization and do not break controls.
    for failure in ('asset', 'webgl'):
        page = browser.new_page()
        errors = []
        page.on('pageerror', lambda e: errors.append(str(e)))
        if failure == 'asset':
            page.route('**/model/manifest.json', lambda route: route.fulfill(status=404, body='fixture'))
        else:
            page.add_init_script('''const original=HTMLCanvasElement.prototype.getContext;
            HTMLCanvasElement.prototype.getContext=function(kind,...args){return kind.startsWith('webgl')?null:original.call(this,kind,...args)}''')
        page.goto(url)
        page.wait_for_function('PiArmViewer.snapshot().failed')
        assert page.locator('#connectBtn').is_enabled()
        assert '三维显示不可用' in page.locator('#viewerStatus').inner_text()
        assert not errors, errors
        page.close()
    print('PASS missing-asset and WebGL fallback, existing connection controls preserved', flush=True)


def run_mock(browser, url):
    page = browser.new_page(viewport=dict(width=1440, height=1100))
    page.goto(url)
    page.locator('#connectBtn').click()
    page.wait_for_function('lastState!==null && PiArmViewer.snapshot().ready', timeout=120000)
    assert page.evaluate('lastState.hardware_type') == 'mock', 'Real hardware is forbidden'
    page.wait_for_function('PiArmViewer.snapshot().valid && Number.isFinite(lastState.pose?.x)')
    # Only read the model + TF; no enable or motion request.
    for _ in range(5):
        data = page.evaluate('({snapshot:PiArmViewer.snapshot(),state:lastState})')
        verify_transforms(data['snapshot'], data['state']['current_joint_angles'])
        tf = data['state']['pose']
        matrix = np.array(data['snapshot']['links']['tool0']).reshape(4, 4, order='F')
        position = np.array([tf['x'], tf['y'], tf['z']])/1000
        expected_rotation = rotation([0, 0, 1], math.radians(tf['rz']))@rotation([0, 1, 0], math.radians(tf['ry']))@rotation([1, 0, 0], math.radians(tf['rx']))
        assert np.allclose(matrix[:3, 3], position, atol=1e-6, rtol=0), (matrix[:3, 3], position)
        assert np.allclose(matrix[:3, :3], expected_rotation, atol=1e-6, rtol=0)
        page.wait_for_timeout(250)
    page.close()
    print('PASS complete mock: browser tool0 position/orientation match ROS TF within 1e-6, no motion requests', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--browser-path')
    parser.add_argument('--mock-url')
    parser.add_argument('--screenshots', type=Path)
    args = parser.parse_args()
    verify_assets()
    class QuietHandler(SimpleHTTPRequestHandler):
        def log_message(self, *args):
            pass
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(QuietHandler, directory=str(ROOT/'web')))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True, executable_path=args.browser_path,
                                        args=['--enable-unsafe-swiftshader'])
            run_fixture(browser, f'http://127.0.0.1:{server.server_port}/control.html', args.screenshots)
            if args.mock_url:
                run_mock(browser, args.mock_url)
            browser.close()
    finally:
        server.shutdown()
        server.server_close()


if __name__ == '__main__':
    main()
