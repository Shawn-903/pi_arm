#!/usr/bin/env python3
"""Browser tests against vcan commissioning, never real CAN."""
import argparse
import json
from pathlib import Path

from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--browser-path")
    parser.add_argument("--stall", action="store_true", help="emulator --freeze-motion")
    args = parser.parse_args()
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True, executable_path=args.browser_path)
        page = browser.new_page()
        errors = []
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.goto("http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766")
        page.locator("#connectBtn").click()
        page.wait_for_function("lastState !== null")
        assert page.evaluate('lastState.commissioning && lastState.can_interface === "vcan0"')
        page.locator('#viewRole').select_option('developer')
        page.locator('[data-view-target="commissioning"]').click()
        assert page.locator('#motorTestStep').input_value() == '5'
        assert page.locator('#motorTestStep').get_attribute('max') == '5'
        assert page.locator('#motorTestSpeed').input_value() == '1'
        assert page.locator('#motorTestSpeed').get_attribute('max') == '1'
        page.locator("#testMotorId").select_option("3")
        page.locator("#testEnableMotor").click()
        positive = page.locator('[data-axis-test="motor"][data-sign="1"]')
        page.wait_for_function('!document.querySelector(\'[data-axis-test="motor"][data-sign="1"]\').disabled')
        before = page.evaluate("lastState.axis_test.test_id")
        positive.click()
        if args.stall:
            page.wait_for_function("lastState.axis_test.active")
            page.wait_for_timeout(1300)
            assert page.evaluate("lastState.axis_test.active"), "browser heartbeat must keep test alive"
            page.locator("#axisTestStop").click()
        page.wait_for_function("id => lastState.axis_test.test_id > id && !lastState.axis_test.active", arg=before)
        page.wait_for_function("axisTestRecords.at(-1)?.after !== undefined")
        assert "acknowledged" in page.locator("#axisTestStatus").inner_text()
        page.locator("#axisObservation").fill("VCAN TEST: observed direction placeholder")
        page.locator("#saveAxisObservation").click()
        page.locator('[data-view-target="diagnostics"]').click()
        with page.expect_download() as download:
            page.locator("#exportDiagnosticBtn").click()
        snapshot = json.loads(Path(download.value.path()).read_text())
        record = snapshot["axis_tests"][-1]
        assert record["request"]["space"] == "motor" and record["request"]["axis_id"] == 3
        assert record['request']['delta_deg'] == 5 and record['request']['speed_deg_s'] == 1
        if not args.stall:
            assert 'succeeded' in record['after']['axis_test']['status']
            before_angles = record['before']['raw_protocol_positions_deg']
            after_angles = record['after']['raw_protocol_positions_deg']
            assert abs(after_angles[2] - before_angles[2] - 180) < .02
        assert "VCAN TEST" in record["observation"]
        assert record["after"]["axis_test"]["active"] is False
        if args.stall:
            page.locator('[data-view-target="commissioning"]').click()
            before = page.evaluate("lastState.axis_test.test_id")
            positive.click()
            page.wait_for_function("lastState.axis_test.active")
            page.locator("#disconnectBtn").click()
            page.wait_for_function('connectionState === "closed"')
            page.wait_for_timeout(1000)
            page.locator("#connectBtn").click()
            page.wait_for_function("id => lastState?.axis_test.test_id > id && !lastState.axis_test.active", arg=before)
            assert "stopped" in page.locator("#axisTestStatus").inner_text()
        assert not errors, errors
        browser.close()
    print("PASS axis UI, explicit motor selection, feedback record, observation/export" + (", heartbeat/stop/disconnect" if args.stall else ""))


if __name__ == "__main__":
    main()
