#!/usr/bin/env python3
"""示教步进及程序主从编辑检查；拦截 WebSocket，只在树莓派运行。"""

from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import re
from pathlib import Path
import threading
from playwright.sync_api import sync_playwright
from test_web_viewer import fixture


class Quiet(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


def main():
    root = Path(__file__).resolve().parents[1]
    server = ThreadingHTTPServer(
        ("127.0.0.1", 0), partial(Quiet, directory=str(root / "web"))
    )
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(
                headless=True, args=["--enable-unsafe-swiftshader"]
            )
            page = browser.new_page(viewport=dict(width=1440, height=1000))
            page.add_init_script(
                "Object.defineProperty(Crypto.prototype, 'randomUUID', {value: undefined})"
            )
            sent, errors, sockets = [], [], []
            page.on("pageerror", lambda e: errors.append(str(e)))

            def route(socket):
                sockets.append(socket)
                socket.on_message(lambda raw: sent.append(json.loads(raw)))

            page.route_web_socket("**/*", route)
            page.goto(f"http://127.0.0.1:{server.server_port}/control.html")
            page.locator("#connectBtn").click()
            state = fixture([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
            page.evaluate(
                "s=>{window.fake=s;window.tick=setInterval(()=>handleMessage(fake),100);handleMessage(fake)}",
                state,
            )
            page.wait_for_function(
                '!document.querySelector("[data-step-axis]").disabled'
            )
            assert "0.20" in page.locator('[data-step-feedback="1"]').inner_text()
            for confirmed in (False, True):
                page.locator("#globalStop").click()
                page.wait_for_function("pending.size===1")
                request = sent[-1]
                assert request["cmd"] == "stop_motion"
                sockets[0].send(json.dumps(dict(
                    type="rsp", msg_id=request["msg_id"], code=0, ms="ok",
                    data=dict(stop_confirmed=confirmed, message="" if confirmed else "保持反馈超时"),
                )))
                page.wait_for_function("pending.size===0")
                warning = page.locator("#stopWarning")
                assert warning.is_visible() == (not confirmed)
                if not confirmed:
                    assert "保持反馈超时" in warning.inner_text()
                    assert "停止失败" in page.locator("#commandLog").text_content()
            for space, axis, sign, delta in [
                ("joint", 4, 1, 0.2),
                ("position", 1, -1, 0.5),
                ("orientation", 2, 1, 0.3),
            ]:
                page.locator("#stepSpace").select_option(space)
                page.locator("#stepDelta").fill(str(delta))
                page.locator(
                    f'[data-step-axis="{axis}"][data-step-sign="{sign}"]'
                ).click()
                page.wait_for_function("pending.size===1")
                assert page.locator("[data-step-axis]").first.is_disabled()
                request = sent[-1]
                assert request["cmd"] == "step_move"
                params = dict(request["params"])
                assert re.fullmatch(
                    r"[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}",
                    params.pop("request_id"),
                )
                assert params == dict(space=space, axis=axis, delta=sign * delta)
                run_id = "run-" + str(request["msg_id"])
                sockets[0].send(
                    json.dumps(
                        dict(
                            type="rsp",
                            msg_id=request["msg_id"],
                            code=0,
                            ms="ok",
                            data=dict(step_id="step-test", run_id=run_id),
                        )
                    )
                )
                page.wait_for_function("pending.size===0")
                assert page.locator("[data-step-axis]").first.is_disabled(), (
                    "Accepted motion waits for its terminal state"
                )
                page.evaluate(
                    'id=>{fake.execution={run_id:id,status:"SUCCEEDED",mode:"manual"};handleMessage(fake)}',
                    run_id,
                )
                page.wait_for_function(
                    '!document.querySelector("[data-step-axis]").disabled'
                )
            before = len(sent)
            page.locator("#stepDelta").fill("")
            page.locator('[data-step-axis="0"][data-step-sign="1"]').click()
            assert len(sent) == before
            assert page.locator("#stepDelta").get_attribute("aria-invalid") == "true"
            page.evaluate("fake.read_only=true;handleMessage(fake)")
            assert page.locator("[data-step-axis]").first.is_disabled()
            page.evaluate(
                'fake.read_only=false;fake.execution={run_id:"owner",status:"PAUSED",mode:"sdk",name:"script"};handleMessage(fake)'
            )
            assert page.locator("[data-step-axis]").first.is_disabled()
            page.evaluate(
                "delete fake.execution;clearInterval(tick);lastStateAt=Date.now()-2500;refreshControlGate()"
            )
            assert "—" in page.locator('[data-step-feedback="0"]').inner_text()
            assert page.locator("[data-step-axis]").first.is_disabled()
            assert page.locator("#globalStop").is_enabled()
            page.locator('[data-view-target="programs"]').click()
            page.locator('[data-program-add="movej"]').click()
            page.locator("#programInspector input").first.fill("12.3456789")
            page.locator("#programAdd").click()
            page.locator("#programInspector input").fill("2.5")
            page.locator('[data-program-add="repeat"]').click()
            page.locator("#programInspector input").fill("3")
            page.locator("#programInspector button").filter(
                has_text="添加循环内步骤"
            ).click()
            page.locator("#programInspector input").fill("4")
            page.locator("#programOutline .program-row").first.click()
            assert (
                page.locator("#programInspector input").first.input_value()
                == "12.3456789"
            )
            page.locator("#programInspector button").filter(has_text="下移").click()
            page.locator("#programOutline .program-row").first.click()
            assert page.locator("#programInspector input").input_value() == "2.5"
            page.locator("#programInspector input").fill("")
            page.locator("#programOutline .program-row").nth(1).click()
            page.locator("#programExport").click()
            assert (
                page.locator("#programInspector input").get_attribute("aria-invalid")
                == "true"
            )
            page.locator("#programInspector input").fill("2.5")
            with page.expect_download() as download:
                page.locator("#programExport").click()
            doc = json.loads(Path(download.value.path()).read_text())
            assert [row["type"] for row in doc["steps"]] == ["wait", "movej", "repeat"]
            assert doc["steps"][1]["params"]["target"][0] == 12.3456789
            assert (
                doc["steps"][2]["count"] == 3
                and doc["steps"][2]["steps"][1]["seconds"] == 4
            )
            assert len(sent) == before, (
                "Editing and exporting must not dispatch commands"
            )
            assert not errors, errors
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
    print(
        "PASS three-space step requests / busy-read-only-owner-stale gates / program selection, precision, reorder, nested steps and validation"
    )


if __name__ == "__main__":
    main()
