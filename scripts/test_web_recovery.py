#!/usr/bin/env python3
"""Reboot service integration from the browser. Refuses real hardware."""
import argparse
import json
from pathlib import Path

from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ws', default='ws://127.0.0.1:8766')
    args = parser.parse_args()
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page(viewport={'width': 1440, 'height': 1000})
        errors, responses = [], []
        page.on('pageerror', lambda error: errors.append(str(error)))

        def observe(ws):
            def receive(payload):
                data = json.loads(payload)
                if data.get('type') == 'rsp':
                    responses.append(data)
            ws.on('framereceived', receive)

        page.on('websocket', observe)
        page.goto((Path(__file__).resolve().parents[1] / 'web/control.html').as_uri())
        page.locator('#wsUrl').fill(args.ws)
        page.locator('#connectBtn').click()
        page.wait_for_function('lastState?.state_available && lastState?.reboot_available')
        assert page.evaluate('lastState.hardware_type') == 'mock', 'Only mock is allowed'
        page.locator('[data-cmd="enable_motor"][data-motor-id="0"]').click()
        page.wait_for_function('lastState.robot_state === "READY"')
        task = page.evaluate('lastState.task.task_id')
        page.locator('#recoveryMotorId').select_option('2')
        page.locator('#rebootMotorBtn').click()
        page.wait_for_function('document.getElementById("recoveryResult").textContent.includes("电机保持关闭")')
        page.wait_for_function('lastState.motor_enabled[1] === false')
        assert page.evaluate('lastState.motor_enabled') == [True, False, True, True, True, True]
        assert page.evaluate('lastState.task.task_id') == task
        assert page.locator('#moveZeroPose').is_disabled()
        assert any(r.get('data', {}).get('cmd') == 'reboot_motor' and r['code'] == 0 for r in responses)
        page.locator('#clearMotorFaultBtn').click()
        page.wait_for_function('document.getElementById("recoveryResult").textContent.includes("清除错误请求完成")')
        assert page.evaluate('lastState.motor_enabled[1]') is False
        assert not errors, errors
        browser.close()
    print('PASS browser → WebSocket → manager → mock hardware reboot/clear, selected motor off, no motion task')


if __name__ == '__main__':
    main()
