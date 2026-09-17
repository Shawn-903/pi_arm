#!/usr/bin/env python3
"""Exercise control spaces using browser clicks, exclusively mock or vcan0."""
import argparse
from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766')
    parser.add_argument('--browser-path')
    args = parser.parse_args()
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True, executable_path=args.browser_path)
        page = browser.new_page()
        errors = []
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.goto(args.url)
        page.locator('#connectBtn').click()
        page.wait_for_function('lastState?.state_available')
        assert page.evaluate('lastState.hardware_type === "mock" || lastState.can_interface === "vcan0"'), 'Physical CAN prohibited'
        page.locator('[data-cmd="enable_motor"][data-motor-id="0"]').click()
        page.wait_for_function('lastState.robot_state === "READY"')

        def finish(previous):
            page.wait_for_function('id => lastState.task.task_id > id && [2,3,4].includes(lastState.task.status)',
                                   arg=previous, timeout=150000)
            task = page.evaluate('lastState.task')
            assert task['status'] == 2, task
            page.wait_for_function('lastState.robot_state === "READY"')

        # Move to a nonsingular virtual pose before testing LIN through the UI.
        page.locator('[data-tab="multi"]').click()
        for i, value in enumerate([10,10,-10,5,5,10],1):
            page.locator(f'#multiJoint{i}').fill(str(value))
        page.locator('#multiSpeed').fill('5')
        page.locator('#outputSpeedLimit').fill('5')
        previous = page.evaluate('lastState.task.task_id')
        page.locator('#multiMoveBtn').click()
        finish(previous)
        print('PASS full joint target through normal control chain', flush=True)

        page.locator('[data-tab="outputControlCard"]').click()
        page.locator('#copyOutputTargets').click()
        value = float(page.locator('#outputTarget1').input_value())
        page.locator('#outputTarget1').fill(str(value+.2))
        page.locator('#outputSpeedLimit').fill('0.5')
        previous = page.evaluate('lastState.task.task_id')
        page.locator('#moveOutputTargets').click()
        finish(previous)
        assert abs(page.evaluate('lastState.model_output_positions_rad[0]*180/Math.PI')-value-.2) < .02
        print('PASS model output target with 0.5 deg/s ceiling', flush=True)

        # All Cartesian execution in this test is initiated via webpage clicks.
        page.locator('[data-tab="multi"]').click()
        page.locator('input[name="motionMode"][value="line"]').check()
        page.wait_for_timeout(300)
        for target, source in [('targetX','endX'),('targetY','endY'),('targetZ','endZ'),
                               ('targetRx','endRx'),('targetRy','endRy'),('targetRz','endRz')]:
            value = float(page.locator('#'+source).input_value())
            if target == 'targetZ': value -= .5
            page.locator('#'+target).fill(str(value))
        page.locator('#endSpeed').fill('1')
        page.locator('#outputSpeedLimit').fill('1')
        previous = page.evaluate('lastState.task.task_id')
        page.locator('#multiMoveBtn').click()
        finish(previous)
        print('PASS virtual Cartesian LIN via webpage, base_link/tool0', flush=True)
        assert not errors, errors
        browser.close()


if __name__ == '__main__':
    main()
