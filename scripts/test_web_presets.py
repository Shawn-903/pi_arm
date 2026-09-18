#!/usr/bin/env python3
"""Preset button integration against mock ROS only; rejects real hardware."""
import argparse
import json

from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--browser-path')
    parser.add_argument('--url', default='http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766')
    args = parser.parse_args()
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True, executable_path=args.browser_path)
        page = browser.new_page()
        errors, responses = [], []
        page.on('pageerror', lambda error: errors.append(str(error)))
        def observe(ws):
            def received(payload):
                data = json.loads(payload)
                if data.get('type') == 'rsp': responses.append(data)
            ws.on('framereceived', received)
        page.on('websocket', observe)
        page.goto(args.url)
        assert page.locator('#movePackingPose').is_disabled()
        assert page.locator('#moveZeroPose').is_disabled()
        page.locator('#connectBtn').click()
        page.wait_for_function('lastState?.state_available')
        assert page.evaluate('lastState.hardware_type') == 'mock', 'Only mock is allowed'
        page.locator('[data-cmd="enable_motor"][data-motor-id="0"]').click()
        page.wait_for_function('lastState.robot_state === "READY" && !document.getElementById("movePackingPose").disabled')
        assert '100.700' in page.locator('#packingPoseStatus').inner_text()
        page.locator('#target2').fill('12')  # Named pose must not use editable joint targets.
        previous = page.evaluate('lastState.task.task_id')
        assert page.locator('#presetSpeed').input_value() == '5'
        page.locator('#presetSpeed').fill('10')
        page.locator('#movePackingPose').click()
        page.wait_for_function('id => lastState.task.task_id > id && lastState.task.status === 1', arg=previous)
        assert page.locator('#movePackingPose').is_disabled()
        assert page.locator('#moveZeroPose').is_disabled()
        page.wait_for_timeout(700)
        page.locator('#presetPosePanel [data-cmd="stop_motion"]').click()
        page.wait_for_function('lastState.robot_state === "READY" && [3,4].includes(lastState.task.status)')
        page.wait_for_timeout(600)
        page.wait_for_function('lastState.robot_state === "READY" && !document.getElementById("moveZeroPose").disabled')
        packing = [r for r in responses if r.get('data', {}).get('preset') == 'packing'][-1]
        assert packing['code'] == 0
        assert packing['data']['target_joint_angles_deg'] == [0, 100.7, -85.6, 0, 0, 0]
        assert packing['data']['joint_speed_deg_s'] == 10.0
        previous = page.evaluate('lastState.task.task_id')
        page.locator('#presetSpeed').fill('5')
        page.locator('#moveZeroPose').click()
        page.wait_for_function('id => lastState.task.task_id > id && lastState.task.status === 2', arg=previous, timeout=120000)
        assert all(abs(q) < .01 for q in page.evaluate('lastState.current_joint_angles'))
        zero = [r for r in responses if r.get('data', {}).get('preset') == 'zero'][-1]
        assert zero['data']['joint_speed_deg_s'] == 5.0
        assert zero['code'] == 0 and zero['data']['target_joint_angles_deg'] == [0]*6
        page.evaluate('lastStateAt = Date.now()-2000; refreshControlGate()')
        assert page.locator('#movePackingPose').is_disabled()
        assert page.locator('#moveZeroPose').is_disabled()
        page.locator('#disconnectBtn').click()
        page.wait_for_function('connectionState === "closed"')
        assert page.locator('#movePackingPose').is_disabled()
        assert not errors, errors
        browser.close()
    print('PASS mock packing action/stop, zero completion, server targets, task lock and stale/disconnect gates')


if __name__ == '__main__':
    main()
