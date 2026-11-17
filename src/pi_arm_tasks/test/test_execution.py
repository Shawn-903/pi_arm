import asyncio
from copy import deepcopy
import tempfile
import unittest
import uuid
from pathlib import Path

from pi_arm_tasks.execution.engine import Engine, ExecutionError
from pi_arm_tasks.execution.plan import validate_program
from pi_arm_tasks.execution.store import Store


class Adapter:
    def __init__(self):
        self.token = ""
        self.instance = "controller-one"
        self.moves = []
        self.release = asyncio.Event()
        self.release.set()
        self.arrive = asyncio.Event()
        self.arrive.set()
        self.stop_count = 0
        self.stop_failed = False
        self.moving = False

    async def lease(self, operation, token, name=""):
        if operation == "acquire":
            if self.token:
                raise ExecutionError(2003, "busy")
            self.token = uuid.uuid4().hex
        elif not token or token != self.token:
            raise ExecutionError(3101, "invalid lease")
        elif operation == "release":
            self.token = ""
        return dict(token=self.token, controller_instance_id=self.instance)

    async def check(self, instance, *, stationary=False):
        if instance != self.instance:
            raise ExecutionError(3104, "controller restarted")
        if stationary and self.moving:
            raise ExecutionError(2001, "motion during wait")

    async def stop(self):
        self.stop_count += 1
        self.token = ""
        self.arrive.set()
        if self.stop_failed:
            raise ExecutionError(3006, "hold failed")

    async def resolve(self, command, params):
        return (
            dict(type="wait", seconds=params["seconds"])
            if command == "wait"
            else dict(type=command, params=deepcopy(params))
        )

    async def freeze(self, steps):
        return deepcopy(steps)

    async def move(self, command, params, token):
        await self.release.wait()
        if not token or token != self.token:
            raise ExecutionError(3101, "revoked lease")
        self.moves.append(deepcopy(params))
        return len(self.moves)

    async def result(self, task_id, instance):
        await self.arrive.wait()
        return dict(status="SUCCEEDED", code=0)


async def until(condition, timeout=4.0):
    deadline = asyncio.get_running_loop().time() + timeout
    while not condition():
        if asyncio.get_running_loop().time() >= deadline:
            raise AssertionError("condition timed out")
        await asyncio.sleep(0.01)


class ExecutionTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.store = Store(Path(self.directory.name) / "test.db")
        self.adapter = Adapter()
        self.engine = Engine(self.adapter, self.store)

    async def asyncTearDown(self):
        self.adapter.release.set()
        await self.engine.close()
        await asyncio.sleep(0.05)
        self.store.close()
        self.directory.cleanup()

    async def open(self):
        result = await self.engine.command(
            "session_open", dict(name="test", request_id=uuid.uuid4().hex), "client"
        )
        return result["session_id"]

    async def submit(self, session, request=None, command="move", **params):
        data = dict(session_id=session, request_id=request or uuid.uuid4().hex)
        if command == "move":
            data.update(targets=[1.0] * 6, max_speed=2.0)
        data.update(params)
        return await self.engine.command(command, data, "client")

    async def test_duplicate_requests_and_frozen_parameters(self):
        session = await self.open()
        self.adapter.arrive.clear()
        request = uuid.uuid4().hex
        first, second = await asyncio.gather(self.submit(session, request), self.submit(session, request))
        self.assertEqual(first["step_id"], second["step_id"])
        await until(lambda: len(self.adapter.moves) == 1)
        with self.assertRaises(ExecutionError):
            await self.submit(session)
        self.adapter.arrive.set()
        await until(lambda: self.engine.active.step is None)
        self.assertEqual(self.store.get("steps", first["step_id"])["status"], "SUCCEEDED")
        self.assertEqual(len(self.adapter.moves), 1)
        with self.assertRaises(ValueError):
            await self.submit(session, request, max_speed=3.0)

    async def test_pause_wait_preserves_time_and_ownership(self):
        session = await self.open()
        step = await self.submit(session, command="wait", seconds=0.3)
        await asyncio.sleep(0.08)
        await self.engine.command("execution_pause", {})
        remaining = self.engine.active.step["remaining_s"]
        await asyncio.sleep(0.2)
        self.assertAlmostEqual(self.engine.active.step["remaining_s"], remaining, delta=0.03)
        self.assertTrue(self.adapter.token)
        with self.assertRaises(ExecutionError):
            await self.open()
        await self.engine.command("execution_resume", {})
        await until(lambda: self.engine.active.step is None)
        self.assertGreater(self.store.get("steps", step["step_id"])["elapsed_s"], 0.45)

    async def test_pause_motion_is_a_step_boundary(self):
        session = await self.open()
        self.adapter.arrive.clear()
        await self.submit(session)
        await until(lambda: bool(self.adapter.moves))
        await self.engine.command("execution_pause", {})
        self.assertEqual(self.engine.state()["status"], "PAUSING")
        self.assertEqual(self.adapter.stop_count, 0)
        self.adapter.arrive.set()
        await until(lambda: self.engine.state()["status"] == "PAUSED")
        with self.assertRaises(ExecutionError):
            await self.submit(session)
        await self.engine.command("execution_stop", {})
        with self.assertRaises(ExecutionError):
            await self.engine.command("execution_resume", {})

    async def test_late_acceptance_after_stop_cannot_dispatch(self):
        session = await self.open()
        self.adapter.release.clear()
        step = await self.submit(session)
        await until(lambda: self.engine.active.step["status"] == "RUNNING")
        stop = asyncio.create_task(self.engine.command("execution_stop", {}))
        await until(lambda: self.adapter.stop_count > 0)
        self.adapter.release.set()
        await stop
        self.assertEqual(self.adapter.moves, [])
        self.assertEqual(self.store.get("steps", step["step_id"])["status"], "CANCELED")

    async def test_stop_fences_requests_waiting_for_admission(self):
        await self.engine.lock.acquire()
        request = asyncio.create_task(
            self.engine.command("move", dict(targets=[0.0] * 6, max_speed=2.0, request_id="late-manual"))
        )
        await asyncio.sleep(0.02)
        await self.engine.command("execution_stop", {})
        self.engine.lock.release()
        with self.assertRaises(ExecutionError):
            await request
        self.assertEqual(self.adapter.moves, [])
        self.assertIsNone(self.engine.active)

    async def test_cancel_old_run_does_not_stop_next_owner(self):
        first = await self.open()
        await self.engine.command("session_close", dict(session_id=first), "client")
        second = await self.open()
        with self.assertRaises(ExecutionError):
            await self.engine.command("execution_stop", dict(run_id=first))
        self.assertEqual(self.engine.state()["session_id"], second)
        self.assertEqual(self.adapter.stop_count, 0)

    async def test_stop_during_program_freeze_prevents_start(self):
        entered, release = asyncio.Event(), asyncio.Event()

        async def freeze(steps):
            entered.set()
            await release.wait()
            return deepcopy(steps)

        self.adapter.freeze = freeze
        request = asyncio.create_task(
            self.engine.command(
                "program_start",
                dict(
                    program=dict(name="late", steps=[dict(type="wait", seconds=1)]), request_id="late-program"
                ),
            )
        )
        await entered.wait()
        await self.engine.command("execution_stop", {})
        release.set()
        with self.assertRaises(ExecutionError):
            await request
        self.assertIsNone(self.engine.active)
        self.assertEqual(self.adapter.token, "")

    async def test_disconnect_stops_sdk_but_not_program(self):
        session = await self.open()
        await self.submit(session, command="wait", seconds=10.0)
        await self.engine.command("connection_lost", {}, "client")
        self.assertEqual(self.engine.state()["code"], 3102)
        with self.assertRaises(ExecutionError):
            await self.open()
        document = dict(name="autonomous", steps=[dict(type="wait", seconds=0.15)])
        result = await self.engine.command(
            "program_start", dict(program=document, request_id="program-1"), "browser"
        )
        await self.engine.command("connection_lost", {}, "browser")
        await until(lambda: self.engine.active is None)
        self.assertEqual(self.store.get("runs", result["run_id"])["status"], "SUCCEEDED")

    async def test_stop_failure_never_advances_program(self):
        self.adapter.arrive.clear()
        move = dict(type="move", params=dict(targets=[1.0] * 6, max_speed=2.0))
        await self.engine.command(
            "program_start", dict(program=dict(name="test", steps=[move, move]), request_id="program")
        )
        await until(lambda: len(self.adapter.moves) == 1)
        self.adapter.stop_failed = True
        await self.engine.command("execution_stop", {})
        self.assertFalse(self.engine.state()["stop_confirmed"])
        self.assertEqual(self.engine.state()["status"], "FAILED")
        self.assertEqual(len(self.adapter.moves), 1)
        with self.assertRaises(ExecutionError):
            await self.open()
        self.adapter.stop_failed = False
        await self.engine.command("execution_stop", {})
        await self.open()

    async def test_heartbeat_and_controller_restart(self):
        session = await self.open()
        self.engine.heartbeat_timeout = 0.1
        await self.submit(session, command="wait", seconds=10.0)
        await until(lambda: self.engine.active is None)
        self.assertEqual(self.engine.state()["code"], 3102)
        self.engine.heartbeat_timeout = 3.0
        await self.open()
        self.adapter.instance = "controller-two"
        await until(lambda: self.engine.active is None)
        self.assertEqual(self.engine.state()["code"], 3104)

    async def test_disk_failure_prevents_motion(self):
        session = await self.open()
        original = self.store.put

        def fail(*_):
            raise OSError("disk full")

        self.store.put = fail
        with self.assertRaises(Exception):
            await self.submit(session)
        await until(lambda: self.engine.active is None)
        self.assertGreater(self.adapter.stop_count, 0)
        self.assertEqual(self.adapter.moves, [])
        self.store.put = original

    async def test_unexpected_motion_during_wait_prevents_next_step(self):
        await self.engine.command(
            "program_start",
            dict(
                program=dict(
                    name="wait guard",
                    steps=[
                        dict(type="wait", seconds=2.0),
                        dict(type="move", params=dict(targets=[0.0] * 6, max_speed=2.0)),
                    ],
                ),
                request_id="wait-guard",
            ),
        )
        await until(lambda: self.engine.state()["phase"] == "wait")
        self.adapter.moving = True
        await until(lambda: self.engine.active is None)
        self.assertEqual(self.engine.state()["status"], "FAILED")
        self.assertTrue(self.engine.state()["stop_confirmed"])
        self.assertEqual(self.adapter.moves, [])

    async def test_program_name_conflict_does_not_delete_another_program(self):
        first = self.store.program_save(dict(name="one", steps=[dict(type="wait", seconds=1)]), 0)
        with self.assertRaises(ValueError):
            self.store.program_save(dict(name="one", steps=[dict(type="wait", seconds=2)]), 0)
        self.assertEqual(self.store.program_get(first["id"]), first)

    async def test_restart_interrupts_records_and_never_replays(self):
        session = await self.open()
        step = await self.submit(session, command="wait", seconds=5.0)
        self.store.interrupt_unfinished()
        self.assertEqual(self.store.get("runs", session)["status"], "INTERRUPTED")
        self.assertEqual(self.store.get("steps", step["step_id"])["status"], "INTERRUPTED")


class PlanTests(unittest.TestCase):
    def test_single_level_bounded_repeat(self):
        result = validate_program(
            dict(name="test", steps=[dict(type="repeat", count=3, steps=[dict(type="wait", seconds=0)])])
        )
        self.assertEqual([r["cycle"] for r in result], [1, 2, 3])
        for count in (True, -1, 10001):
            with self.assertRaises(ValueError):
                validate_program(
                    dict(
                        name="bad",
                        steps=[dict(type="repeat", count=count, steps=[dict(type="wait", seconds=0)])],
                    )
                )
