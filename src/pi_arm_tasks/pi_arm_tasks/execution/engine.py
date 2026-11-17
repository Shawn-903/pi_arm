"""独占执行会话。运动终态由适配器确认，数据库写入不占用停止入口。"""

import asyncio
from copy import deepcopy
import csv
import io
import time
from types import SimpleNamespace
import uuid

from .plan import MOTIONS, number, validate_motion, validate_program
from .store import utc_now, encode

TERMINAL = {"SUCCEEDED", "CANCELED", "FAILED", "INTERRUPTED"}


class ExecutionError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code, self.message = code, message


class Engine:
    def __init__(self, adapter, store, *, heartbeat_timeout=3.0):
        self.adapter, self.store = adapter, store
        self.instance_id = uuid.uuid4().hex
        self.active = None
        self.closed_connections = set()
        self.stop_epoch = 0
        self.stop_unconfirmed = False
        self.last = {}
        self.lock = asyncio.Lock()
        self.heartbeat_timeout = heartbeat_timeout
        self.store.interrupt_unfinished()

    def state(self):
        result = deepcopy(self.active.doc if self.active else self.last)
        return dict(result, service_instance_id=self.instance_id)

    async def db(self, method, *args):
        return await asyncio.to_thread(getattr(self.store, method), *deepcopy(args))

    def owned(self, params, connection, *, required=True):
        run = self.active
        if not run or run.doc["status"] in TERMINAL:
            raise ExecutionError(3101, "没有可控制的运行")
        if required and (
            run.doc["mode"] != "sdk"
            or params.get("session_id") != run.doc["session_id"]
            or connection != run.connection
        ):
            raise ExecutionError(3101, "控制会话无效，请重新创建会话")
        return run

    async def command(self, command, params, connection=""):
        if command == "execution_state":
            return self.state()
        if command == "session_heartbeat":
            run = self.owned(params, connection)
            if run.abort.is_set():
                raise ExecutionError(3101, "会话正在停止")
            run.heartbeat = time.monotonic()
            return self.state()
        if command == "connection_lost":
            self.closed_connections.add(connection)
            run = self.active
            if run and run.doc["mode"] == "sdk" and run.connection == connection:
                await self.stop(run, "脚本连接已断开", 3102)
            return {}
        if command in ("execution_stop", "execution_pause", "execution_resume"):
            run = self.active
            if not run:
                if command == "execution_stop":
                    identifier = params.get("run_id") or params.get("session_id")
                    if identifier:
                        if identifier not in (self.last.get("run_id"), self.last.get("session_id")):
                            raise ExecutionError(3101, "本次运行已经结束")
                        return self.state()
                    self.stop_epoch += 1
                    self.stop_unconfirmed = True
                    await self.adapter.stop()
                    self.stop_unconfirmed = False
                    if self.last:
                        self.last["stop_confirmed"] = True
                        await self.db("put", "runs", self.last)
                    return dict(self.state(), stop_confirmed=True)
                raise ExecutionError(3101, "没有可控制的运行")
            if params.get("session_id"):
                self.owned(params, connection)
            if params.get("run_id") and params["run_id"] != run.doc["run_id"]:
                raise ExecutionError(3101, "本次运行已经结束")
            if command == "execution_stop":
                await self.stop(run, "用户停止")
            elif command == "execution_pause":
                if run.abort.is_set():
                    raise ExecutionError(3101, "本次运行已经停止")
                run.gate.clear()
                run.doc["status"] = "PAUSING" if run.step and run.step["type"] != "wait" else "PAUSED"
                await self.record(run)
            else:
                if run.abort.is_set():
                    raise ExecutionError(3101, "停止后不能继续")
                await self.adapter.check(
                    run.doc["controller_instance_id"], stationary=run.doc["phase"] == "wait"
                )
                await self.adapter.lease("renew", run.token)
                if run.abort.is_set():
                    raise ExecutionError(3101, "本次运行已经停止")
                run.doc["status"] = "RUNNING"
                await self.record(run)
                if not run.abort.is_set():
                    run.gate.set()
            return self.state()
        if command == "request_get":
            return dict(accepted=await self.db("request_get", params.get("request_id")))
        if command == "task_get":
            if self.active and self.active.step and self.active.step["step_id"] == params.get("step_id"):
                step = deepcopy(self.active.step)
                if step["status"] == "SUCCEEDED":
                    step["status"] = "RUNNING"
                return step
            return await self.db("get", "steps", params.get("step_id"))
        if command == "run_list":
            return dict(runs=await self.db("run_list", params.get("limit", 100)))
        if command in ("run_get", "run_export"):
            data = await self.db("run_export", params.get("run_id"))
            if command == "run_get":
                return data
            if params.get("format", "json") == "csv":
                output = io.StringIO(newline="")
                fields = [
                    "run_id",
                    "step_id",
                    "index",
                    "cycle",
                    "type",
                    "status",
                    "task_id",
                    "controller_instance_id",
                    "request_id",
                    "started_at",
                    "finished_at",
                    "elapsed_s",
                    "code",
                    "message",
                    "stop_confirmed",
                    "seconds",
                    "params",
                    "request_params",
                    "point_snapshot",
                    "sample",
                    "ab_sample",
                    "run_status",
                    "run_name",
                ]
                writer = csv.DictWriter(output, fields, extrasaction="ignore")
                writer.writeheader()
                for row in data["steps"]:
                    writer.writerow(
                        dict(
                            row,
                            run_status=data["run"]["status"],
                            run_name=data["run"]["name"],
                            **{
                                field: encode(row.get(field, {}))
                                for field in (
                                    "params",
                                    "request_params",
                                    "point_snapshot",
                                    "sample",
                                    "ab_sample",
                                )
                            },
                        )
                    )
                return dict(content=output.getvalue(), format="csv")
            return dict(content=encode(data), format="json")
        if command == "program_list":
            return dict(programs=await self.db("program_list"))
        if command == "program_get":
            return await self.db("program_get", params.get("id"))
        # 停止、续租和查询不等待这个锁；它只串行化受理及数据编辑。
        epoch = self.stop_epoch
        async with self.lock:
            if command == "program_save":
                document = deepcopy(params.get("program"))
                validate_program(document)
                return await self.db("program_save", document, params.get("revision", 0))
            if command == "program_delete":
                await self.db("program_delete", params.get("id"), params.get("revision"))
                return {}
            if command == "program_validate":
                steps = await self.adapter.freeze(validate_program(params.get("program")))
                return dict(valid=True, step_count=len(steps), steps=steps)
            if command == "session_close":
                run = self.owned(params, connection)
                if run.step or params.get("abort"):
                    await self.stop(run, "控制会话异常退出" if params.get("abort") else "会话关闭时仍有任务")
                else:
                    await self.finish(run, "SUCCEEDED")
                return deepcopy(run.doc)
            if command not in MOTIONS | {"wait", "program_start", "session_open"}:
                raise ValueError("不支持的执行命令")
            if epoch != self.stop_epoch:
                raise ExecutionError(3101, "请求等待受理时已收到停止")
            payload = dict(command=command, params=params)
            request_id = params.get("request_id")
            previous = await self.db("request", request_id, payload)
            if previous is not None:
                return previous
            if command == "session_open":
                run = await self.create("sdk", params.get("name", "Python 控制"), connection, epoch)
                run.doc["request_id"] = request_id
                response = deepcopy(run.doc)
            elif command == "program_start":
                document = (
                    await self.db("program_get", params["id"]) if params.get("id") else params.get("program")
                )
                steps = await self.adapter.freeze(validate_program(document))
                run = await self.create("program", document["name"], "", epoch)
                run.doc["request_id"] = request_id
                run.frozen = dict(snapshot=deepcopy(document), steps=steps)
                run.doc.update(program_kind=document.get("kind", "steps"), step_count=len(steps))
                response = deepcopy(run.doc)
            else:
                if command == "wait":
                    if number(params.get("seconds"), "等待时间") < 0:
                        raise ValueError("等待时间不能为负数")
                else:
                    validate_motion(command, params)
                run = self.owned(params, connection) if params.get("session_id") else None
                if run is None:
                    if command == "wait":
                        raise ExecutionError(3101, "等待需要控制会话")
                    run = await self.create(
                        "manual", "有限步进" if command == "step_move" else "单次运动", "", epoch
                    )
                if run.abort.is_set() or not run.gate.is_set():
                    raise ExecutionError(3103, "执行已暂停或停止")
                if run.step:
                    raise ExecutionError(2003, "同一会话只能执行一个任务")
                try:
                    resolved = await self.adapter.resolve(command, params)
                except Exception:
                    if run.doc["mode"] == "manual":
                        await self.stop(run, "目标校验失败", 1004)
                    raise
                if run.abort.is_set():
                    raise ExecutionError(3101, "受理过程中运行已停止")
                resolved["request_params"] = {
                    key: deepcopy(value)
                    for key, value in params.items()
                    if key not in ("request_id", "session_id")
                }
                step = self.new_step(run, resolved, request_id)
                run.step = step
                response = deepcopy(step)
            try:
                if command == "program_start":
                    await self.db("snapshot", run.doc["run_id"], run.frozen)
                await self.record(run)
                if command not in ("session_open", "program_start"):
                    await self.db("put", "steps", run.step)
                await self.db("request", request_id, payload, response)
            except Exception:
                await self.stop(run, "运行记录无法写入", 3105)
                raise
            if not run.abort.is_set():
                if command == "program_start":
                    run.worker = asyncio.create_task(self.program(run, steps))
                elif command != "session_open":
                    run.worker = asyncio.create_task(self.single(run))
            return response

    async def create(self, mode, name, connection, epoch):
        if epoch != self.stop_epoch:
            raise ExecutionError(3101, "请求受理期间已收到停止")
        if self.stop_unconfirmed:
            raise ExecutionError(3006, "上次停止未确认，请先重试停止")
        if mode == "sdk" and connection in self.closed_connections:
            raise ExecutionError(3102, "脚本连接已关闭")
        if self.active:
            raise ExecutionError(2003, "已有程序或控制会话占用机械臂")
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 80:
            raise ValueError("运行名称应为 1–80 个字符")
        identifier = uuid.uuid4().hex
        run = SimpleNamespace(
            doc=dict(
                run_id=identifier,
                session_id=identifier if mode == "sdk" else "",
                mode=mode,
                hardware_type=getattr(self.adapter, "hardware_type", "unknown"),
                name=name,
                status="STARTING",
                phase="idle",
                step_index=0,
                step_count=0,
                task_id=0,
                code=0,
                message="",
                started_at=utc_now(),
                controller_instance_id="",
                stop_confirmed=False,
            ),
            token="",
            connection=connection,
            heartbeat=time.monotonic(),
            started=time.monotonic(),
            gate=asyncio.Event(),
            abort=asyncio.Event(),
            step=None,
            step_started=None,
            worker=None,
            watcher=None,
            stop_task=None,
            round_started=0.0,
            arrived=0.0,
            arrived_at="",
            arrived_task=0,
            previous_capture=None,
            previous_arrival=None,
        )
        run.gate.set()
        self.active = run
        try:
            lease = await self.adapter.lease("acquire", "", name)
            run.token = lease["token"]
            run.doc["controller_instance_id"] = lease["controller_instance_id"]
            if run.abort.is_set() or (mode == "sdk" and connection in self.closed_connections):
                await self.adapter.stop()
                raise ExecutionError(3101, "运行在受理期间被停止")
            run.doc["status"] = "RUNNING"
            await self.record(run)
            run.watcher = asyncio.create_task(self.watch(run))
            return run
        except Exception:
            if run.token:
                await self.stop(run, "会话创建失败", 3101)
            elif self.active is run:
                self.active = None
            raise

    async def record(self, run):
        try:
            await self.db("put", "runs", run.doc)
        except Exception as error:
            # 先禁止派发，再安排停止；不等待损坏的记录介质。
            run.abort.set()
            asyncio.create_task(self.stop(run, "运行记录无法写入", 3105))
            raise ExecutionError(3105, "运行记录无法写入") from error

    async def watch(self, run):
        try:
            while self.active is run and not run.abort.is_set():
                await asyncio.sleep(0.5)
                if run.abort.is_set() or self.active is not run:
                    break
                if run.doc["mode"] == "sdk" and time.monotonic() - run.heartbeat > self.heartbeat_timeout:
                    raise ExecutionError(3102, "脚本心跳超时")
                await self.adapter.check(
                    run.doc["controller_instance_id"], stationary=run.doc["phase"] == "wait"
                )
                await self.adapter.lease("renew", run.token)
        except Exception as error:
            await self.stop(run, str(error), getattr(error, "code", 3104))

    def new_step(self, run, row, request_id=""):
        run.doc["step_index"] += 1
        return dict(
            deepcopy(row),
            step_id=uuid.uuid4().hex,
            run_id=run.doc["run_id"],
            request_id=request_id,
            index=run.doc["step_index"],
            status="PENDING",
            task_id=0,
            code=0,
            message="",
            controller_instance_id=run.doc["controller_instance_id"],
        )

    async def checkpoint(self, run):
        while not run.gate.is_set() and not run.abort.is_set():
            if run.doc["status"] != "PAUSED":
                run.doc["status"] = "PAUSED"
                await self.record(run)
            await asyncio.sleep(0.02)
        if run.abort.is_set():
            raise ExecutionError(3101, "运行已停止")

    async def perform(self, run):
        step = run.step
        await self.checkpoint(run)
        started = time.monotonic()
        run.step_started = started
        if step.get("ab_leg") == "start":
            run.round_started = started - run.started
        step.update(status="RUNNING", started_at=utc_now())
        run.doc.update(
            step_id=step["step_id"],
            phase=step["type"],
            task_id=0,
            source_index=step.get("source_index"),
            cycle=step.get("cycle"),
        )
        await self.db("put", "steps", step)
        await self.record(run)
        # 停止可能在落盘或目标受理期间到达，每个派发边界都再检查一次。
        await self.checkpoint(run)
        if step["type"] == "wait":
            remaining = step["seconds"]
            while remaining > 0:
                await self.checkpoint(run)
                tick = time.monotonic()
                await asyncio.sleep(min(0.02, remaining))
                if run.gate.is_set():
                    remaining = max(0.0, remaining - (time.monotonic() - tick))
                step["remaining_s"] = remaining
        else:
            task_id = await self.adapter.move(step["type"], step["params"], run.token)
            step["task_id"] = run.doc["task_id"] = task_id
            if run.abort.is_set():
                await self.adapter.stop()
            await self.db("put", "steps", step)
            await self.record(run)
            result = await self.adapter.result(task_id, run.doc["controller_instance_id"])
            step["sample"] = result.get("sample", {})
            if result.get("status") != "SUCCEEDED":
                raise ExecutionError(result.get("code") or 3106, result.get("message") or "运动未成功结束")
            if step.get("ab_leg") == "target":
                run.arrived, run.arrived_at, run.arrived_task = (
                    time.monotonic() - run.started,
                    utc_now(),
                    task_id,
                )
        if run.abort.is_set():
            raise ExecutionError(3101, "运行已停止")
        if step.get("capture") and run.doc.get("program_kind") == "ab":
            captured = time.monotonic() - run.started
            sample = await self.adapter.sample()
            sample.update(
                cycle=step.get("cycle"),
                task_id=run.arrived_task,
                round_started_elapsed_s=run.round_started,
                cycle_duration_s=captured - run.round_started,
                target_arrived_at=run.arrived_at,
                target_arrived_elapsed_s=run.arrived,
                captured_elapsed_s=captured,
                captured_at=utc_now(),
                target_interval_s=None if run.previous_capture is None else captured - run.previous_capture,
                arrival_interval_s=None
                if run.previous_arrival is None
                else run.arrived - run.previous_arrival,
            )
            step["ab_sample"] = sample
            run.previous_capture, run.previous_arrival = captured, run.arrived
        step.update(status="SUCCEEDED", finished_at=utc_now(), elapsed_s=time.monotonic() - started)
        await self.db("put", "steps", step)
        run.doc.update(phase="idle", step_id="", task_id=0)
        await self.record(run)
        run.step = None
        run.step_started = None
        await self.checkpoint(run)

    async def single(self, run):
        try:
            await self.perform(run)
            if run.doc["mode"] == "manual":
                await self.finish(run, "SUCCEEDED")
        except Exception as error:
            if not run.abort.is_set():
                asyncio.create_task(self.stop(run, str(error), getattr(error, "code", 3105)))

    async def program(self, run, steps):
        try:
            for row in steps:
                await self.checkpoint(run)
                run.step = self.new_step(run, row)
                await self.perform(run)
            await self.finish(run, "SUCCEEDED")
        except Exception as error:
            if not run.abort.is_set():
                asyncio.create_task(self.stop(run, str(error), getattr(error, "code", 3105)))

    async def finish(self, run, status):
        if run.abort.is_set():
            return
        if run.watcher:
            run.watcher.cancel()
        await self.adapter.lease("release", run.token)
        if run.abort.is_set():
            return
        run.doc.update(
            status=status, phase="idle", finished_at=utc_now(), elapsed_s=time.monotonic() - run.started
        )
        await self.record(run)
        self.last = deepcopy(run.doc)
        run.gate.set()
        if self.active is run:
            self.active = None

    async def stop(self, run, message, code=0):
        run.abort.set()
        run.gate.set()
        if run.stop_task is None:
            self.stop_epoch += 1
            run.doc.update(status="STOPPING", message=message, code=code)
            run.stop_task = asyncio.create_task(self._stop(run, message, code))
        await asyncio.shield(run.stop_task)

    async def _stop(self, run, message, code):
        confirmed = False
        try:
            await self.adapter.stop()
            confirmed = True
        except Exception as error:
            message = f"{message}；停止未确认：{error}"
            code = 3006
        if run.worker and not run.worker.done():
            try:
                await asyncio.wait_for(asyncio.shield(run.worker), 12.0)
            except asyncio.TimeoutError:
                run.worker.cancel()
                confirmed = False
                code, message = 3006, message + "；任务退出未确认"
        # 令牌已撤销，即使 action 的接受响应迟到也不能再触发有效派发。
        run.doc.update(
            status="CANCELED" if confirmed and not code else "FAILED",
            phase="idle",
            message=message,
            code=code,
            stop_confirmed=confirmed,
            finished_at=utc_now(),
            elapsed_s=time.monotonic() - run.started,
        )
        self.stop_unconfirmed = not confirmed
        if run.step:
            run.step.update(
                status=run.doc["status"],
                code=code,
                message=message,
                stop_confirmed=confirmed,
                finished_at=utc_now(),
                elapsed_s=0.0 if run.step_started is None else time.monotonic() - run.step_started,
            )
        try:
            if run.step:
                await self.db("put", "steps", run.step)
            await self.db("put", "runs", run.doc)
            await self.db(
                "event", run.doc["run_id"], "stop", dict(confirmed=confirmed, code=code, message=message)
            )
        except Exception:
            run.doc.update(code=3105, message=message + "；运行记录无法写入")
        self.last = deepcopy(run.doc)
        if self.active is run:
            self.active = None

    async def close(self):
        if self.active:
            await self.stop(self.active, "执行服务关闭", 3104)
