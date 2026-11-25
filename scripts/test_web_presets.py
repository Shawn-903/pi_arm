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
        packing_button = page.locator('.header-presets [data-preset="packing"]')
        zero_button = page.locator('.header-presets [data-preset="zero"]')
        assert packing_button.is_visible() and zero_button.is_visible()
        assert all(b.is_disabled() for b in page.locator('[data-preset]').all())
        page.locator('#connectBtn').click()
        page.locator('#teachTab-target').click()
        page.locator('[data-tab="single"]').click()
        page.locator('#targetUtilities > summary').click()
        page.wait_for_function('lastState?.state_available')
        assert page.evaluate('lastState.hardware_type') == 'mock', 'Only mock is allowed'
        page.locator('[data-cmd="enable_motor"][data-motor-id="0"]').click()
        page.wait_for_function('lastState.robot_state === "READY" && !document.getElementById("movePackingPose").disabled')
        assert '100.700' in page.locator('#packingPoseStatus').inner_text()
        page.locator('#target2').fill('12')  # Named pose must not use editable joint targets.
        previous = page.evaluate('lastState.task.task_id')
        assert page.locator('#presetSpeed').input_value() == '5'
        page.locator('#presetSpeed').fill('10')
        page.locator('#outputSpeedLimit').fill('10')
        page.locator('#teachTab-step').click()
        packing_button.click()
        page.wait_for_function('id => lastState.task.task_id > id && lastState.task.status === 1', arg=previous)
        assert all(b.is_disabled() for b in page.locator('[data-preset]').all())
        page.wait_for_timeout(700)
        page.locator('#globalStop').click()
        page.wait_for_function('lastState.robot_state === "READY" && [3,4].includes(lastState.task.status)')
        page.wait_for_timeout(600)
        page.wait_for_function('lastState.robot_state === "READY" && !document.getElementById("moveZeroPose").disabled')
        packed = [0, 100.7, -85.6, 0, 0, 0]
        def move_and_check(button, target):
            previous = page.evaluate('lastState.task.task_id')
            button.click()
            page.wait_for_function('id => lastState.task.task_id > id && lastState.task.status === 2',
                                   arg=previous, timeout=130000)
            actual = page.evaluate('lastState.current_joint_angles')
            assert len(actual) == 6 and all(abs(a - b) < .01 for a, b in zip(actual, target)), actual
        move_and_check(packing_button, packed)
        page.locator('#teachTab-target').click()
        page.locator('#presetSpeed').fill('5')
        page.locator('[data-view-target="points"]').click()
        move_and_check(zero_button, [0]*6)
        steps = [r['data'] for r in responses if r['code'] == 0
                 and r['data'].get('request_params', {}).get('preset')]
        assert [(s['request_params']['preset'], s['params']['targets'], s['params']['max_speed'])
                for s in steps] == [('packing', packed, 10), ('packing', packed, 10), ('zero', [0]*6, 5)]
        assert len({s['request_id'] for s in steps}) == 3
        assert page.evaluate('''() => {
            lastStateAt = Date.now() - 2000; refreshControlGate();
            return [...document.querySelectorAll('[data-preset]')].every(b => b.disabled)
                && !document.getElementById('globalStop').disabled;
        }''')
        page.locator('#disconnectBtn').click()
        page.wait_for_function('connectionState === "closed"')
        assert all(b.is_disabled() for b in page.locator('[data-preset]').all())
        assert not errors, errors
        browser.close()
    print('PASS header packing/zero arrival, stop, server targets, task lock and stale/disconnect gates')


if __name__ == '__main__':
    main()
