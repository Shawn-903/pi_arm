#!/usr/bin/env python3
"""步骤编辑、暂停停止、历史、点位修改与响应布局；只连接独立 mock。"""

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from playwright.sync_api import sync_playwright


class Quiet(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws", default="ws://127.0.0.1:8766")
    parser.add_argument("--url", help="使用已部署页面，可验证局域网 HTTP 访问")
    parser.add_argument("--screenshots", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    server = ThreadingHTTPServer(("127.0.0.1", 0), partial(Quiet, directory=str(root / "web")))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    prefix = "ui-" + str(time.time_ns())
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True, args=["--enable-unsafe-swiftshader"])
            page = browser.new_page(viewport=dict(width=1440, height=1000))
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.on("dialog", lambda dialog: dialog.accept())
            page.goto(args.url or f"http://127.0.0.1:{server.server_port}/control.html?ws={args.ws}")
            page.locator("#connectBtn").click()
            page.wait_for_function('lastState?.hardware_type==="mock" && lastState.state_available')
            assert page.evaluate("lastState.hardware_type") == "mock"
            page.evaluate('PiArmControl.request("enable_motor",{motor_id:0})')
            page.wait_for_function('lastState.robot_state==="READY"')
            before = page.evaluate('lastState.task.task_id')
            angle = page.evaluate('lastState.current_joint_angles[0]')
            page.locator('#stepDelta').fill('0.1')
            page.locator('[data-step-axis="0"][data-step-sign="1"]').click()
            page.wait_for_function('id=>lastState.task.task_id>id && lastState.task.status===2', arg=before)
            assert abs(page.evaluate('lastState.current_joint_angles[0]') - angle - .1) < .01
            page.locator('#teachTab-target').click()
            target = page.locator('#multiMoveBtn').bounding_box()
            model = page.locator('#viewerStage').bounding_box()
            assert target['x'] + target['width'] <= model['x'] and target['y'] + target['height'] < 1000
            page.locator('[data-view-target="programs"]').click()
            page.locator("#programName").fill(prefix)
            page.locator("#programAdd").click()
            page.locator("#programSteps input[type=number]").fill("5")
            page.locator("#programSave").click()
            page.wait_for_function('document.getElementById("programStatus").textContent.includes("已保存")')
            page.locator("#programValidate").click()
            page.wait_for_function(
                'document.getElementById("programStatus").textContent.includes("校验通过")'
            )
            page.locator("#programStart").click()
            page.wait_for_function('lastState.execution?.phase==="wait"')
            page.locator('[data-view-target="workbench"]').click()
            assert page.locator("#multiMoveBtn").is_disabled()
            page.locator("#executionPause").click()
            page.wait_for_function('lastState.execution.status==="PAUSED"')
            page.locator('[data-view-target="programs"]').click()
            page.wait_for_function('document.querySelector("#programRunning .current-step")')
            assert page.locator("#programSteps .current-step").count() == 1
            page.locator("#programSteps input[type=number]").fill("3")
            page.wait_for_function('!document.querySelector("#programSteps .current-step")')
            assert "5 秒" in page.locator("#programRunning .current-step").inner_text()
            page.locator('[data-view-target="workbench"]').click()
            page.wait_for_timeout(5200)
            assert page.evaluate("lastState.execution.status") == "PAUSED"
            page.locator("#executionResume").click()
            page.wait_for_function('lastState.execution.status==="SUCCEEDED"', timeout=10000)
            page.locator('[data-view-target="programs"]').click()
            page.locator("#programStart").click()
            page.wait_for_function('lastState.execution.status==="RUNNING"')
            page.locator("#globalStop").click()
            page.wait_for_function('lastState.execution.status==="CANCELED"')
            assert page.evaluate("lastState.execution.stop_confirmed")
            page.locator('[data-view-target="records"]').click()
            page.locator("#historyRefresh").click()
            page.wait_for_function('document.getElementById("historyList").children.length>0')
            assert prefix in page.locator("#historyList").inner_text()
            page.locator("#historyList button").filter(has_text="详情").first.click()
            page.wait_for_function(
                'document.getElementById("historyDetail").textContent.includes("运行编号")'
            )
            page.locator('[data-view-target="points"]').click()
            page.locator("#pointName").fill(prefix)
            page.locator("#saveManualPoint").click()
            page.wait_for_function(
                'n=>[...document.querySelectorAll(".point-card strong")].some(e=>e.textContent===n)',
                arg=prefix,
            )
            page.locator("#pointSearch").fill(prefix)
            card = page.locator(".point-card").first
            identity = card.get_attribute("data-point-id")
            page.evaluate(
                """async id=>{
                const data=JSON.parse((await PiArmControl.request('point_export')).content);
                await PiArmControl.request('point_update',{id,revision:data.revision,values:Array(6).fill(.123456789)});
            }""",
                identity,
            )
            page.locator("#refreshPoints").click()
            page.wait_for_function("pending.size===0")
            card.locator('[data-point-action="edit"]').click()
            page.locator("dialog input").nth(0).fill("")
            page.locator("dialog input").nth(1).fill("保存备注")
            page.locator("dialog button").filter(has_text="保存修改").click()
            page.wait_for_function('document.querySelector("dialog [role=alert]").textContent.length>0')
            assert page.locator("dialog").is_visible()
            assert page.locator("dialog input").nth(1).input_value() == "保存备注"
            page.locator("dialog input").nth(0).fill(prefix + " edited")
            page.locator("dialog input").nth(1).fill("保存备注")
            assert ".123456789" in page.locator("dialog input").nth(2).input_value()
            page.locator("dialog input").nth(2).fill(", ".join(map(str, [0.234567891] + [0.123456789] * 5)))
            page.locator("dialog button").filter(has_text="保存修改").click()
            page.wait_for_function('document.getElementById("pointStatus").textContent.includes("已更新")')
            assert page.locator(".point-card").first.get_attribute("data-point-id") == identity
            with page.expect_download() as download:
                page.locator("#exportPoints").click()
            data = json.loads(Path(download.value.path()).read_text())
            assert any(p["id"] == identity and p["note"] == "保存备注" for p in data["points"])
            assert (
                next(p for p in data["points"] if p["id"] == identity)["values"]
                == [0.234567891] + [0.123456789] * 5
            )
            for width, height, scale in [(1440, 1000, 1), (390, 844, 1), (1024, 768, 2)]:
                page.set_viewport_size(dict(width=width, height=height))
                page.evaluate("s=>document.body.style.zoom=s", scale)
                for view in ("workbench", "programs", "points", "records"):
                    page.locator(f'[data-view-target="{view}"]').click()
                    assert page.evaluate("document.documentElement.scrollWidth<=innerWidth"), (
                        width,
                        scale,
                        view,
                    )
                    assert page.locator("#globalStop").is_visible()
                    page.evaluate('window.scrollTo(0,document.body.scrollHeight)')
                    stop = page.locator('#globalStop').bounding_box()
                    assert 0 <= stop['y'] and stop['y'] + stop['height'] <= height
                    page.evaluate('window.scrollTo(0,0)')
                    if args.screenshots and view in ("workbench", "programs"):
                        args.screenshots.mkdir(parents=True, exist_ok=True)
                        page.screenshot(
                            path=str(args.screenshots / f"{view}-{width}-{scale}.png"), full_page=False
                        )
            page.evaluate("document.body.style.zoom=1")
            page.set_viewport_size(dict(width=1440, height=1000))
            page.locator('[data-view-target="points"]').click()
            page.locator(".point-card").first.locator('[data-point-action="delete"]').click()
            page.locator('[data-view-target="programs"]').click()
            page.locator("#programDelete").click()
            assert not errors, errors
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
    print(
        "PASS program editor / server pause-stop-history / point identity-export / desktop-mobile-200% stop visibility"
    )


if __name__ == "__main__":
    main()
