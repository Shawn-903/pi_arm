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
        page.locator('#viewRole').select_option('developer')
        page.locator('[data-view-target="diagnostics"]').click()
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
        page.evaluate('''handleMessage({...lastState, motion_stop_failed:true,
          kinematics_error:{code:"JOINT_POSITION_LIMIT",axis:4,value:0.610865,
            message:"joint limit exceeded: joint_05 value=0.610865 rad, limit=[-0.523599,0.523599] rad"}})''')
        summary = page.locator("#diagnosticSummary").inner_text()
        assert "JOINT_POSITION_LIMIT" in summary and "joint_05" in summary
        assert "停止未确认" in summary and "重试停止" in summary
        page.evaluate('''handleMessage({...lastState, read_only:false, commissioning:true,
          motor_test_permitted:false, protocol_angle_ratios:[36,36,36,6,6,6]})''')
        page.locator('[data-view-target="commissioning"]').click()
        page.locator("#testMotorId").select_option("4")
        assert abs(float(page.locator("#motorTestSpeed").get_attribute("min")) - 1/6) < 1e-10
        assert "0.166667" in page.locator("#motorTestReason").inner_text()
        assert page.locator("#axisTestStop").is_enabled()
        assert page.locator('[data-axis-test="motor"][data-sign="1"]').is_disabled()
        page.evaluate('''handleMessage({...lastState, preset_poses:{
          packing:{target_joint_angles_deg:[0,100.7,-85.6,0,0,0],available:false,
                   reason:"方向测试模式未启动整臂轨迹控制器；标定尚未确认"},
          zero:{target_joint_angles_deg:[0,0,0,0,0,0],available:false,reason:"标定尚未确认"}}})''')
        page.locator('[data-view-target="workbench"]').click()
        assert page.locator('#movePackingPose').is_visible() and page.locator('#movePackingPose').is_disabled()
        assert page.locator('#moveZeroPose').is_visible() and page.locator('#moveZeroPose').is_disabled()
        assert '100.700' in page.locator('#packingPoseStatus').inner_text()
        assert '标定尚未确认' in page.locator('#zeroPoseStatus').inner_text()
        assert page.evaluate('sendCommand("move_preset", {preset:"packing"})') is False
        page.evaluate('''handleMessage({...lastState, calibrated:true, kinematics_valid:true,
          current_joint_angles:[0,101.7,-86.6,0,0,0], current_speeds:[0,0,0,0,0,0],
          kinematics_error:{code:"JOINT_COMMAND_LIMIT_WARNING",axis:2,value:1.775,
            message:"feedback outside command limits, within feedback allowance"}})''')
        assert '仍在解算容差内' in page.locator('#diagnosticSummary').inner_text()
        assert page.locator('[data-status-row="2"] [data-field="angle"]').inner_text() == '101.700000'
        page.evaluate("lastStateAt = Date.now() - 2000; refreshControlGate()")
        assert "状态已过期" in page.locator("#diagnosticSummary").inner_text()
        assert page.locator('[data-coordinate-row="2"] [data-coordinate="fresh"]').inner_text() == "缺失/过期"
        page.evaluate('handleMessage({type:"state",read_only:true,hardware_type:"real",state_available:false,robot_state:"DISCONNECTED"})')
        assert "检查监控进程" in page.locator("#diagnosticSummary").inner_text()
        assert not errors, errors
        browser.close()
    print("PASS fault connection, raw feedback, joint limits, stop retry, minimum speed, export, read-only and stale state")


if __name__ == "__main__":
    main()
