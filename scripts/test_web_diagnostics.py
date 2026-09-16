#!/usr/bin/env python3
"""Offline browser regression: synthetic fault feedback, no ROS or CAN access."""
import argparse
import json
from pathlib import Path

from playwright.sync_api import sync_playwright


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--browser-path")
    args = parser.parse_args()
    state = {
        "type": "state", "read_only": True, "hardware_type": "real",
        "state_available": True, "hardware_connected": True,
        "driver_mode": "MONITORING", "robot_state": "FAULT",
        "calibrated": True, "kinematics_valid": False, "control_permitted": False,
        "calibration_id": "TEST-R02", "motor_ids": [2, 1, 3, 4, 5, 6],
        "motor_fresh": [True] * 6, "motor_error_codes": [0] * 6,
        "motor_ages_sec": [0.01] * 6,
        "current_joint_angles": [999.0] * 6,
        "raw_protocol_positions_deg": [-319.55, 0.03, 631.27, 0.04, -0.04, 0.03],
        "raw_output_positions_rad": [0.1] * 6,
        "software_output_offsets_rad": [1.5] * 6,
        "model_output_positions_rad": [1.6, 0.0, -8.146277666983426, 0.0, 0.0, 0.0],
        "kinematics_error": {"code": "OUTPUT_POSITION_LIMIT", "axis": 3,
                             "value": -8.146277666983426,
                             "message": "model output limit exceeded at motor_03"},
        "diagnostic": "value=-8.146277666983426 rad, limit=[-7.841,7.841] rad",
        "supported_motion_commands": [],
    }
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True, executable_path=args.browser_path)
        page = browser.new_page()
        errors, sent = [], []
        page.on("pageerror", lambda error: errors.append(str(error)))

        def socket_route(socket):
            socket.on_message(lambda message: sent.append(message))
            socket.send(json.dumps(state))

        page.route_web_socket("**/*", socket_route)
        page.goto((Path(__file__).resolve().parents[1] / "web/control.html").as_uri())
        page.locator("#connectBtn").click()
        page.wait_for_function('lastState?.robot_state === "FAULT"')
        summary = page.locator("#diagnosticSummary").inner_text()
        assert "只读排查" in summary and "OUTPUT_POSITION_LIMIT" in summary
        assert "-8.146277666983426" in summary and "limit=[-7.841,7.841]" in summary
        assert page.locator('[data-status-row="2"] [data-field="angle"]').inner_text() == "--"
        assert page.locator('[data-coordinate-row="2"] [data-coordinate="raw_protocol_positions_deg"]').inner_text() == "-319.550000000"
        assert page.locator('[data-motor-row="2"] [data-field="raw-angle"]').inner_text() == "-319.550"
        assert page.locator('[data-action="move"][data-joint-id="2"]').is_disabled()
        assert page.locator('[data-cmd="enable_motor"][data-motor-id="0"]').is_disabled()
        assert page.evaluate('sendCommand("motor_move", {joint_id:2,target_angle:0,max_speed:1})') is False
        assert not sent
        with page.expect_download() as download:
            page.locator("#exportDiagnosticBtn").click()
        data = json.loads(Path(download.value.path()).read_text())
        assert data["state"]["kinematics_error"]["value"] == -8.146277666983426
        page.evaluate("lastStateAt = Date.now() - 2000; refreshControlGate()")
        assert "状态已过期" in page.locator("#diagnosticSummary").inner_text()
        assert page.locator('[data-coordinate-row="2"] [data-coordinate="fresh"]').inner_text() == "缺失/过期"
        page.evaluate('handleMessage({type:"state",read_only:true,hardware_type:"real",state_available:false,robot_state:"DISCONNECTED"})')
        assert "检查监控进程" in page.locator("#diagnosticSummary").inner_text()
        assert not errors, errors
        browser.close()
    print("PASS fault connection, raw feedback mapping, full errors, export, read-only and stale state")


if __name__ == "__main__":
    main()
