#!/usr/bin/env python3
"""SDK 客户端连接独立 mock；可在 Windows 或树莓派运行。"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import time
import uuid

from pi_arm_sdk import Arm, ArmError, MotionTimeout


def until(predicate, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError("state timed out")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    with Arm(args.host, port=args.port) as arm, Arm(args.host, port=args.port) as observer:
        assert arm.info["hardware_type"] == "mock", "This test only permits mock"
        arm.enable()
        with arm.control("SDK acceptance"):
            current = arm.get_state()["current_joint_angles"]
            target = list(current)
            target[0] += 0.5
            result = arm.move_direct(target)
            assert result.status == "SUCCEEDED" and result.task_id > 0
            assert abs(arm.get_state()["current_joint_angles"][0] - target[0]) < 0.2
            task = arm.move_direct(current, wait=False)
            assert task.wait().status == "SUCCEEDED"
            print("PASS direct blocking / handle result", flush=True)
            with ThreadPoolExecutor() as pool:
                waiting = pool.submit(arm.wait, 0.8)
                until(lambda: observer._rpc("execution_state").get("phase") == "wait")
                observer.pause()
                time.sleep(0.9)
                assert not waiting.done()
                try:
                    with observer.control("conflict"):
                        raise AssertionError("conflicting session accepted")
                except ArmError as error:
                    assert error.code == 2003
                observer.resume()
                assert waiting.result(timeout=5).status == "SUCCEEDED"
            print("PASS paused wait / exclusive control", flush=True)
        session = arm._accept("session_open", dict(name="dedupe"))
        arm._session = session
        params = dict(session_id=session["session_id"], request_id=uuid.uuid4().hex, seconds=0.3)
        first = arm._rpc("wait", params)
        second = arm._rpc("wait", params)
        assert first["step_id"] == second["step_id"]
        arm.stop()
        assert arm._rpc("task_get", dict(step_id=first["step_id"]))["status"] != "SUCCEEDED"
        print("PASS dedupe / cancel terminal", flush=True)
        # 浏览器连接不持有程序的续租职责。
        program = dict(name="autonomous test", schema_version=1, steps=[dict(type="wait", seconds=0.4)])
        transient = Arm(args.host, port=args.port)
        run = transient._accept("program_start", dict(program=program))
        transient.close()
        until(lambda: observer._rpc("run_get", dict(run_id=run["run_id"]))["run"]["status"] == "SUCCEEDED")
        print("PASS program survives client disconnect", flush=True)
        client = Arm(args.host, port=args.port)
        session = client._accept("session_open", dict(name="disconnect"))
        client._session = session
        handle = client._motion("wait", dict(seconds=30.0), wait=False)
        client._socket.close()  # 模拟网络丢失，不调用正常会话关闭。
        until(
            lambda: (
                observer._rpc("run_get", dict(run_id=session["run_id"]))["run"]["status"]
                in ("FAILED", "CANCELED")
            )
        )
        assert observer._rpc("task_get", dict(step_id=handle.step_id))["status"] != "SUCCEEDED"
        print("PASS SDK disconnect stop", flush=True)
        with Arm(args.host, port=args.port) as silent:
            session = silent._accept("session_open", dict(name="heartbeat expiry"))
            step = silent._accept("wait", dict(seconds=30.0, session_id=session["session_id"]))
            # 保持连接，但不登记到 SDK 心跳线程，验证服务端续租超时。
            until(
                lambda: observer._rpc("run_get", dict(run_id=session["run_id"]))["run"]["status"] == "FAILED"
            )
            record = observer._rpc("run_get", dict(run_id=session["run_id"]))["run"]
            assert record["code"] == 3102 and record["stop_confirmed"]
            assert observer._rpc("task_get", dict(step_id=step["step_id"]))["status"] == "FAILED"
        try:
            with arm.control("exception exit"):
                run_id = arm._session["run_id"]
                arm._motion("wait", dict(seconds=30.0), wait=False)
                raise ValueError("external measurement failed")
        except ValueError:
            record = observer._rpc("run_get", dict(run_id=run_id))["run"]
            assert record["status"] == "CANCELED" and record["stop_confirmed"]
        try:
            with arm.control("wait timeout"):
                arm.wait(30.0, timeout_s=0.15)
        except MotionTimeout as error:
            assert error.stop_confirmed and error.run_id and error.step_id
        else:
            raise AssertionError("wait timeout was not reported")
        print("PASS heartbeat expiry / exception exit / timeout stop confirmation", flush=True)
        name = "sdk-test-" + uuid.uuid4().hex[:8]
        arm.points.save(name, joints_deg=[0.123456789, 0, 0, 0, 0, 0])
        point = arm.points.get(name)
        assert point["values"][0] == 0.123456789
        arm.points.update(point["id"], name=name + "-renamed", note="updated")
        updated = arm.points.get(point["id"])
        assert updated["id"] == point["id"] and updated["source"] == point["source"]
        arm.points.delete(point["id"])
        print("PASS point precision / identity / revision", flush=True)
        with arm.control("MoveJ / MoveL / finite steps"):
            assert (
                arm.movej(
                    [20.0, 45.0, -60.0, 10.0, 30.0, 0.0], speed_deg_s=5.0, output_speed_deg_s=5.0
                ).status
                == "SUCCEEDED"
            )
            pose = arm.get_state()["base_frames"]["tool"]
            assert arm.movel(pose["position_mm"], pose["rpy_deg"]).status == "SUCCEEDED"
            assert arm.step_joint(0, 0.2).status == "SUCCEEDED"
            assert arm.step_position(2, -0.5).status == "SUCCEEDED"
            assert arm.step_orientation(2, 0.1).status == "SUCCEEDED"
        print("PASS MoveJ / MoveL / finite joint-position-orientation steps", flush=True)
        base = arm.get_state()["current_joint_angles"]
        prefix = "program-" + uuid.uuid4().hex[:8]
        arm.points.save(prefix + "-A", joints_deg=base)
        b = list(base)
        b[0] += 0.4
        arm.points.save(prefix + "-B", joints_deg=b)
        a_point, b_point = arm.points.get(prefix + "-A"), arm.points.get(prefix + "-B")
        motion = lambda p, leg: dict(
            type="move", point_id=p["id"], params=dict(max_speed=2.0, output_speed_limit=2.0), ab_leg=leg
        )
        document = dict(
            name="A/B acceptance",
            kind="ab",
            schema_version=1,
            steps=[
                dict(
                    type="repeat",
                    count=5,
                    steps=[
                        motion(a_point, "start"),
                        dict(type="wait", seconds=0.05),
                        motion(b_point, "target"),
                        dict(type="wait", seconds=0.05, capture=True),
                    ],
                )
            ],
        )
        run = arm._accept("program_start", dict(program=document))
        changed = list(b)
        changed[0] += 0.5
        arm.points.update(b_point["id"], values=changed)
        until(
            lambda: (
                observer._rpc("run_get", dict(run_id=run["run_id"]))["run"]["status"]
                in ("SUCCEEDED", "FAILED")
            ),
            timeout=60.0,
        )
        record = observer._rpc("run_get", dict(run_id=run["run_id"]))
        assert record["run"]["status"] == "SUCCEEDED", record["run"]
        samples = [s["ab_sample"] for s in record["steps"] if s.get("ab_sample")]
        assert len(samples) == 5 and samples[0]["target_interval_s"] is None
        assert all(s["target_interval_s"] > 0 for s in samples[1:])
        assert abs(arm.get_state()["current_joint_angles"][0] - b[0]) < 0.2
        assert all(
            s["point_snapshot"]["values"] == b for s in record["run"]["steps"] if s.get("ab_leg") == "target"
        )
        assert (
            "run_id,step_id"
            in observer._rpc("run_export", dict(run_id=run["run_id"], format="csv"))["content"]
        )
        arm.points.delete(a_point["id"])
        arm.points.delete(b_point["id"])
        print("PASS five-cycle A/B / frozen points / timing / persistent export", flush=True)
    print("SDK MOCK ACCEPTANCE PASSED", flush=True)


if __name__ == "__main__":
    main()
