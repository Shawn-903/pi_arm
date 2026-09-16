#!/usr/bin/env python3
"""Normal manager/JTC/RealBackend integration; refuses any interface except vcan0."""
import argparse
import asyncio
import json
import socket
import struct

import websockets


async def main(url):
    async with websockets.connect(url) as ws:
        async def until(predicate):
            async def receive():
                while True:
                    value = json.loads(await ws.recv())
                    if predicate(value):
                        return value
            return await asyncio.wait_for(receive(), 30)

        state = await until(lambda m: m.get('type') == 'state' and m.get('state_available') and m.get('can_interface') and all(m.get('motor_fresh', [False])))
        assert state['can_interface'] == 'vcan0' and state['hardware_type'] == 'real', state
        assert not state['commissioning'], state
        seq = 0

        async def command(name, params=None):
            nonlocal seq
            seq += 1
            await ws.send(json.dumps({'cmd': name, 'msg_id': seq, 'params': params or {}}))
            reply = await until(lambda m: m.get('type') == 'rsp' and m['msg_id'] == seq)
            assert reply['code'] == 0, reply
            return reply

        await asyncio.sleep(.5)  # manager requires three stationary feedback publications
        await command('enable_motor', {'motor_id': 0})
        await until(lambda m: m.get('robot_state') == 'READY')
        bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        bus.bind(('vcan0',))
        bus.setblocking(False)
        frames = []

        async def capture():
            while True:
                raw = await asyncio.get_running_loop().sock_recv(bus, 16)
                can_id, length, data = struct.unpack('=IB3x8s', raw)
                frames.append((can_id-0x140, data))

        task = asyncio.create_task(capture())
        try:
            accepted = await command('motor_move', {'joint_id': 1, 'target_angle': .2, 'max_speed': .2})
            task_id = accepted['data']['task_id']
            result = await until(lambda m: m.get('task', {}).get('task_id') == task_id and m['task']['status'] in (2, 3, 4))
            assert result['task']['status'] == 2, result['task']
            await until(lambda m: m.get('robot_state') == 'READY')
            assert any(data[0] == 0xa4 and data[1] == 0 for _, data in frames)
            assert {i for i, data in frames if data[0] == 0x81} == set(range(1, 7))
            print('PASS normal DirectMove terminal result and six-motor stop ACK', flush=True)
            frames.clear()
            accepted = await command('motor_move', {'joint_id': 1, 'target_angle': 2.0, 'max_speed': .2})
            task_id = accepted['data']['task_id']
            await until(lambda m: m.get('task', {}).get('task_id') == task_id and m['task']['status'] == 1)
            await asyncio.sleep(.3)
            stopped = await command('stop_motion')
            result = await until(lambda m: m.get('task', {}).get('task_id') == task_id and m['task']['status'] in (3, 4))
            await until(lambda m: m.get('robot_state') == 'READY')
            assert any(data[0] == 0xa4 and data[1] == 0 for _, data in frames), 'must stop an already transmitted motion'
            first_stop = next(n for n, (_, data) in enumerate(frames) if data[0] == 0x81)
            assert not any(data[0] == 0xa4 and data[1] == 0 for _, data in frames[first_stop+1:])
            assert {i for i, data in frames if data[0] == 0x81} == set(range(1, 7))
            print('PASS stop_motion through gateway/manager/JTC: no A4 after STOP', stopped, result['task'], flush=True)
        finally:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            bus.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='ws://127.0.0.1:8766')
    asyncio.run(main(parser.parse_args().url))
