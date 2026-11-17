#!/usr/bin/env python3
"""离线核对 A/B 程序快照、五轮模板和断线行为；拦截全部 WebSocket。"""

from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from playwright.sync_api import sync_playwright
from test_web_viewer import fixture, NAMES


class Quiet(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


def main():
    root = Path(__file__).resolve().parents[1]
    server = ThreadingHTTPServer(("127.0.0.1", 0), partial(Quiet, directory=str(root / "web")))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        with sync_playwright() as pw:
            browser = pw.chromium.launch(headless=True, args=["--enable-unsafe-swiftshader"])
            page = browser.new_page(viewport=dict(width=1440, height=1100))
            errors, sent = [], []
            page.on("pageerror", lambda e: errors.append(str(e)))
            state = fixture()
            state.update(
                points_available=True,
                motor_in_motion=[False] * 6,
                base_frames=dict(
                    valid=True,
                    model_id="test",
                    joints=[dict(frame_id=n, position_mm=[0] * 3, rpy_deg=[0] * 3) for n in NAMES],
                    tool=dict(position_mm=[0] * 3, rpy_deg=[0] * 3),
                ),
            )
            points = [
                dict(
                    id=str(i),
                    name="点位 " + str(i),
                    kind="joint",
                    values=[i * 0.2] * 6,
                    hardware_type="mock",
                    model_id="test",
                    calibration_id=state["calibration_id"],
                    base_frame="base_link",
                    tool_frame="tool0",
                    joint_names=NAMES,
                    source="manual",
                    created_at="2026-11-17T00:00:00Z",
                )
                for i in range(2)
            ]
            saved = {}

            def route(socket):
                def receive(raw):
                    req = json.loads(raw)
                    sent.append(req)
                    data = {}
                    if req["cmd"] == "point_list":
                        data = dict(revision=1, points=points)
                    if req["cmd"] == "program_start":
                        saved.update(req["params"]["program"])
                        data = dict(run_id="run-one", status="RUNNING")
                        state["execution"] = dict(
                            run_id="run-one",
                            status="RUNNING",
                            name=saved["name"],
                            step_index=1,
                            step_count=20,
                            mode="program",
                        )
                    if req["cmd"] == "run_get":
                        data = dict(
                            run=dict(
                                state["execution"], snapshot=saved, steps=saved["steps"][0]["steps"] * 5
                            ),
                            steps=[],
                        )
                    if req["cmd"] == "execution_stop":
                        state["execution"]["status"] = "CANCELED"
                        data = dict(stop_confirmed=True)
                    socket.send(
                        json.dumps(dict(type="rsp", msg_id=req["msg_id"], code=0, ms="ok", data=data))
                    )
                    socket.send(json.dumps(dict(state, type="state")))

                socket.on_message(receive)

            page.route_web_socket("**/*", route)
            page.goto(f"http://127.0.0.1:{server.server_port}/control.html")
            page.locator("#connectBtn").click()
            page.evaluate(
                "s=>{window.fake=s;window.tick=setInterval(()=>handleMessage(fake),100);handleMessage(s)}",
                state,
            )
            page.locator('[data-view-target="points"]').click()
            page.wait_for_function('document.getElementById("repeatStartPoint").options.length===3')
            page.locator("#repeatStartPoint").select_option("0")
            page.locator("#repeatTargetPoint").select_option("1")
            page.locator("#repeatDwell").fill("0.2")
            page.locator("#repeatTrial").click()
            page.wait_for_function("pending.size===0")
            starts = [r for r in sent if r["cmd"] == "program_start"]
            assert len(starts) == 1
            repeat = starts[0]["params"]["program"]["steps"][0]
            assert repeat["count"] == 5 and [s["type"] for s in repeat["steps"]] == [
                "move",
                "wait",
                "move",
                "wait",
            ]
            assert repeat["steps"][0]["point_id"] == "0" and repeat["steps"][2]["point_id"] == "1"
            assert repeat["steps"][3]["capture"] is True
            assert starts[0]["params"]["request_id"]
            assert not any(r["cmd"] in ("move", "movel", "motor_move") for r in sent)
            page.evaluate("s=>{window.fake=s;handleMessage(s)}", state)
            assert page.locator("#repeatStart").is_disabled()
            before = len([r for r in sent if r["cmd"] in ("stop_motion", "execution_stop")])
            page.locator("#disconnectBtn").click()
            assert len([r for r in sent if r["cmd"] in ("stop_motion", "execution_stop")]) == before
            assert not errors, errors
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
    print(
        "PASS A/B finite program snapshot / five-round trial / no browser dispatch / disconnect does not stop program"
    )


if __name__ == "__main__":
    main()
