#!/usr/bin/env python3
"""Browser smoke test against an already running mock start_web.sh.

Requires Playwright and its Chromium browser. Refuses to control a real backend.
Usage: python3 scripts/test_web_mock.py [--browser-path /path/to/chrome]
"""
import argparse

from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8000/control.html")
    parser.add_argument("--browser-path")
    args = parser.parse_args()
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            headless=True, executable_path=args.browser_path,
        )
        page = browser.new_page(viewport={"width": 1440, "height": 1100})
        errors = []
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.goto(args.url)
        page.locator("#connectBtn").click()
        page.wait_for_function("lastState !== null")
        assert page.evaluate("lastState.hardware_type") == "mock", "Only mock is allowed"
        page.locator('[data-cmd="enable_motor"][data-motor-id="0"]').click()
        page.wait_for_function('lastState.robot_state === "READY"')
        assert page.locator("#endRx").input_value() != "--"

        def previous_task():
            return page.evaluate("lastState.task.task_id")

        def wait_task_after(task_id, statuses=(2,)):
            page.wait_for_function(
                "([id, statuses]) => lastState.task.task_id > id && statuses.includes(lastState.task.status)",
                arg=[task_id, list(statuses)], timeout=30000,
            )
            return page.evaluate("lastState.task")

        start = page.evaluate("lastState.current_joint_angles[0]")
        before = previous_task()
        page.locator("#target1").fill(str(start + 0.5))
        page.locator("#speed1").fill("0.5")
        page.locator('[data-action="move"][data-joint-id="1"]').click()
        wait_task_after(before)
        assert abs(page.evaluate("lastState.current_joint_angles[0]") - start - 0.5) < 0.01
        assert "已完成" in page.locator("#taskState").inner_text()
        print("PASS single joint, fractional speed, pose and final task result", flush=True)

        page.locator("#copyCurrentBtn").click()
        page.locator('[data-tab="multi"]').click()
        before = previous_task()
        joint2 = page.evaluate("lastState.current_joint_angles[1]")
        page.locator("#multiJoint2").fill(str(joint2 + 0.25))
        page.locator("#multiMoveBtn").click()
        wait_task_after(before)
        assert abs(page.evaluate("lastState.current_joint_angles[1]") - joint2 - 0.25) < 0.01
        page.locator("#multiSpeed").fill("")
        page.locator("#multiMoveBtn").click()
        assert "参数错误" in page.locator("#commandLog").inner_text()
        page.locator("#multiSpeed").fill("1")
        print("PASS multi joint and empty input rejection", flush=True)

        assert page.locator('#outputSpeedLimit').input_value() == '5'
        page.locator('#outputSpeedLimit').fill('0.5')
        page.locator('#copyOutputTargets').click()
        output_before = float(page.locator('#outputTarget1').input_value())
        page.locator('#outputTarget1').fill(str(output_before+.1))
        before = previous_task()
        page.locator('#moveOutputTargets').click()
        wait_task_after(before)
        output_after = page.evaluate('lastState.model_output_positions_rad[0]*180/Math.PI')
        assert abs(output_after-output_before-.1) < .01
        page.locator('#outputSpeedLimit').fill('5')
        print('PASS model output target, cable inverse and editable output speed ceiling', flush=True)

        page.locator("#copyCurrentBtn").click()
        # The top-level button must read single-joint cards, even on the multi tab.
        page.locator("#multiJoint1").fill("500")
        before = previous_task()
        page.locator("#moveAllBtn").click()
        wait_task_after(before)
        page.evaluate('sendCommand("disable_motor", {motor_id: 7})')
        page.wait_for_function('document.getElementById("commandLog").textContent.includes("命令失败")')
        print("PASS top-level target selection and visible service rejection", flush=True)

        page.locator('[data-tab="single"]').click()
        before = previous_task()
        angle = page.evaluate("lastState.current_joint_angles[0]")
        page.locator("#target1").fill(str(angle + 5))
        page.locator('[data-action="move"][data-joint-id="1"]').click()
        page.wait_for_function("lastState.task.status === 1")
        page.locator('[data-action="stop"][data-joint-id="1"]').click()
        wait_task_after(before, (3, 4))
        print("PASS stop while moving", flush=True)

        page.wait_for_function('lastState.robot_state === "READY"')
        page.locator('[data-tab="multi"]').click()
        page.locator('input[name="motionMode"][value="line"]').check()
        page.locator("#targetX").fill("100000")
        before = previous_task()
        page.locator("#multiMoveBtn").click()
        wait_task_after(before, (4,))
        assert "失败" in page.locator("#taskState").inner_text()
        print("PASS accepted action reports final planning failure on page", flush=True)

        page.wait_for_function('lastState.robot_state === "READY"')
        assert page.evaluate("formatNumber(null)") == "--"
        assert page.evaluate("""() => {
            lastStateAt = Date.now() - 2000;
            refreshControlGate();
            return document.querySelector('[data-action="move"]').disabled;
        }""")
        page.locator("#disconnectBtn").click()
        page.wait_for_function('connectionState === "closed"')
        assert page.locator('[data-action="move"][data-joint-id="1"]').is_disabled()
        assert page.locator("#endRx").input_value() == "--"
        page.locator("#wsUrl").fill("ws://]")
        page.locator("#connectBtn").click()
        assert "连接失败" in page.locator("#commandLog").inner_text()
        assert not errors, errors
        print("PASS stale/disconnected state and invalid URL; no JavaScript errors", flush=True)
        browser.close()


if __name__ == "__main__":
    main()
