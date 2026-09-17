#!/usr/bin/env python3
"""Integration test. Requires commissioning_web on vcan0; refuses real CAN."""
import argparse
import asyncio
import json
import socket
import struct

import websockets


async def main(url, joint=False, watchdog=False):
    async with websockets.connect(url) as ws:
        async def until(predicate):
            async def receive():
                while True:
                    data = json.loads(await ws.recv())
                    if predicate(data):
                        return data
            return await asyncio.wait_for(receive(), 15)

        state = await until(lambda m: m.get("type") == "state")
        assert state.get("can_interface") == "vcan0" and state.get("commissioning"), state
        seq = 0

        async def command(cmd, params):
            nonlocal seq
            seq += 1
            await ws.send(json.dumps({"type": "req", "msg_id": seq, "cmd": cmd, "params": params}))
            return await until(lambda m: m.get("type") == "rsp" and m["msg_id"] == seq)

        rsp = await command("enable_motor", {"motor_id": 0 if joint else 3})
        assert rsp["code"] == 0, rsp
        state = await until(lambda m: m.get("motor_test_permitted"))
        before = state["raw_protocol_positions_deg"]
        bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        bus.bind(("vcan0",))
        bus.setblocking(False)
        frames = []
        async def capture():
            while True:
                raw = await asyncio.get_running_loop().sock_recv(bus, 16)
                can_id, length, data = struct.unpack("=IB3x8s", raw)
                frames.append((can_id - 0x140, data))
        capture_task = asyncio.create_task(capture())
        rsp = await command("axis_test", {"space": "joint" if joint else "motor",
            "axis_id": 2 if joint else 3, "delta_deg": 0.05 if joint else 0.1, "speed_deg_s": 0.2})
        assert rsp["code"] == 0, rsp
        test_id = rsp["data"]["axis_test_id"]
        result = await until(lambda m: m.get("axis_test", {}).get("test_id") == test_id and not m["axis_test"]["active"])
        if watchdog:
            assert "heartbeat expired" in result["axis_test"]["status"], result
        else:
            assert "succeeded" in result["axis_test"]["status"], result
        capture_task.cancel()
        try:
            await capture_task
        except asyncio.CancelledError:
            pass
        bus.close()
        commanded = {motor_id for motor_id, data in frames if data[0] == 0xA4 and data[1] == 0}
        if joint:
            assert 2 in commanded and len(commanded) > 1, commanded
            assert abs(result["current_joint_angles"][1] - state["current_joint_angles"][1] - 0.05) < 0.005, result
        else:
            assert commanded == {3}, commanded
        assert any(data[0] == 0x81 for _, data in frames), "must issue protocol STOP"
        if not joint and not watchdog:
            after = result["raw_protocol_positions_deg"]
            assert abs(after[2] - before[2] - 3.6) < 0.011, (before, after)
            assert all(abs(after[i] - before[i]) < 0.011 for i in (0, 1, 3, 4, 5)), (before, after)
            if not state["kinematics_valid"]:
                denied = await command("axis_test", {"space": "joint", "axis_id": 2, "delta_deg": 0.1, "speed_deg_s": 0.2})
                assert denied["code"] != 0, denied
        denied = await command("axis_test", {"space": "motor", "axis_id": 3, "delta_deg": 5.01, "speed_deg_s": 0.2})
        assert denied["code"] != 0, denied
        print("PASS", "watchdog" if watchdog else "joint" if joint else "uncoupled motor", result["axis_test"], "CAN position IDs:", commanded)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://127.0.0.1:8766")
    parser.add_argument("--joint", action="store_true")
    parser.add_argument("--watchdog", action="store_true", help="requires emulator --freeze-motion")
    args = parser.parse_args()
    asyncio.run(main(args.url, args.joint, args.watchdog))
