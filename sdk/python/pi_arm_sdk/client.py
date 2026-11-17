from concurrent.futures import Future, TimeoutError as FutureTimeout
from contextlib import contextmanager
from dataclasses import dataclass
import itertools
import json
import threading
import time
import uuid

from websockets.sync.client import connect

TERMINAL = {"SUCCEEDED", "CANCELED", "FAILED", "INTERRUPTED"}


class ArmError(Exception):
    def __init__(self, code, message, *, task_id=0, run_id="", step_id="", stop_confirmed=None):
        super().__init__(message)
        self.code, self.message = code, message
        self.task_id, self.run_id, self.step_id = task_id, run_id, step_id
        self.stop_confirmed = stop_confirmed


class ConnectionLost(ArmError):
    def __init__(self, message="连接已断开，控制会话不能继续"):
        super().__init__(3102, message)


class MotionTimeout(ArmError):
    pass


@dataclass(frozen=True)
class MotionResult:
    task_id: int
    run_id: str
    step_id: str
    status: str
    elapsed_s: float
    data: dict


class MotionHandle:
    def __init__(self, arm, accepted):
        self.arm, self.accepted = arm, dict(accepted)
        self.step_id, self.run_id = accepted["step_id"], accepted["run_id"]

    @property
    def task_id(self):
        return self.accepted.get("task_id", 0)

    def query(self):
        self.accepted = self.arm._rpc("task_get", dict(step_id=self.step_id))
        return dict(self.accepted)

    def done(self):
        return self.query()["status"] in TERMINAL

    def wait(self, timeout_s=None):
        timeout_s = self.arm.motion_timeout_s if timeout_s is None else timeout_s
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        while True:
            try:
                record = self.query()
            except FutureTimeout:
                confirmed = self.arm._stop_after_error(self.run_id)
                raise MotionTimeout(
                    3107,
                    "任务状态查询超时，已请求停止",
                    task_id=self.task_id,
                    run_id=self.run_id,
                    step_id=self.step_id,
                    stop_confirmed=confirmed,
                ) from None
            if record["status"] in TERMINAL:
                if record["status"] != "SUCCEEDED":
                    raise ArmError(
                        record.get("code") or 3106,
                        record.get("message") or record["status"],
                        task_id=record.get("task_id", 0),
                        run_id=self.run_id,
                        step_id=self.step_id,
                        stop_confirmed=record.get("stop_confirmed"),
                    )
                return MotionResult(
                    record.get("task_id", 0),
                    self.run_id,
                    self.step_id,
                    record["status"],
                    record.get("elapsed_s", 0.0),
                    record,
                )
            if deadline is not None and time.monotonic() >= deadline:
                confirmed = self.arm._stop_after_error(self.run_id)
                raise MotionTimeout(
                    3107,
                    "等待运动超时，已请求停止",
                    task_id=record.get("task_id", 0),
                    run_id=self.run_id,
                    step_id=self.step_id,
                    stop_confirmed=confirmed,
                )
            self.arm._closed.wait(0.05)

    def cancel(self):
        return self.arm._rpc("execution_stop", dict(run_id=self.run_id), timeout_s=20.0)


class Points:
    def __init__(self, arm):
        self.arm = arm

    def export(self):
        return json.loads(self.arm._rpc("point_export")["content"])

    def list(self, search=""):
        return [
            p
            for p in self.export()["points"]
            if search.casefold() in (p["name"] + " " + p.get("note", "")).casefold()
        ]

    def get(self, identifier):
        points = [p for p in self.list() if identifier in (p["id"], p["name"])]
        if len(points) != 1:
            raise ArmError(1004, "点位不存在或名称不唯一")
        return points[0]

    def save(
        self, name, *, joints_deg=None, position_mm=None, rpy_deg=None, note="", feedback=False, kind="joint"
    ):
        params = dict(
            name=name,
            note=note,
            revision=self.export()["revision"],
            source="feedback" if feedback else "manual",
            kind=kind,
        )
        if not feedback:
            if joints_deg is not None:
                params.update(kind="joint", values=list(joints_deg))
            else:
                params.update(kind="cartesian", values=list(position_mm or []) + list(rpy_deg or []))
        return self.arm._rpc("point_save", params)

    def update(self, identifier, *, name=None, note=None, values=None):
        point = self.get(identifier)
        params = dict(id=point["id"], revision=self.export()["revision"])
        params.update({k: v for k, v in dict(name=name, note=note, values=values).items() if v is not None})
        return self.arm._rpc("point_update", params)

    def delete(self, identifier):
        return self.arm._rpc(
            "point_delete", dict(id=self.get(identifier)["id"], revision=self.export()["revision"])
        )

    def import_points(self, document, *, conflict="skip", commit=False, revision=None):
        return self.arm._rpc(
            "point_import",
            dict(
                document=document,
                conflict=conflict,
                commit=commit,
                revision=self.export()["revision"] if revision is None else revision,
            ),
        )


class Arm:
    def __init__(self, host, *, port=8765, request_timeout_s=5.0, motion_timeout_s=300.0):
        self.request_timeout_s, self.motion_timeout_s = request_timeout_s, motion_timeout_s
        self._session = None
        self._closed = threading.Event()
        self._pending, self._lock, self._send_lock = {}, threading.Lock(), threading.Lock()
        self._counter = itertools.count(1)
        self._state = {}
        url = host if "://" in host else f"ws://{host}:{port}"
        self._socket = connect(url, open_timeout=request_timeout_s, max_size=32 * 1024 * 1024, proxy=None)
        self._receiver = threading.Thread(target=self._receive, name="pi-arm-receive", daemon=True)
        self._receiver.start()
        try:
            self.info = self._rpc("get_info")
            if self.info.get("protocol_version") != 1:
                raise ArmError(1002, "服务器协议版本不兼容")
        except Exception:
            self.close()
            raise
        self.points = Points(self)
        self._heartbeat = threading.Thread(target=self._heartbeats, name="pi-arm-heartbeat", daemon=True)
        self._heartbeat.start()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def _receive(self):
        try:
            for raw in self._socket:
                value = json.loads(raw)
                if value.get("type") == "state":
                    self._state = value
                elif value.get("type") == "rsp":
                    with self._lock:
                        pending = self._pending.pop(value["msg_id"], None)
                    if pending:
                        pending.set_result(value)
        except Exception:
            pass
        finally:
            self._closed.set()
            with self._lock:
                outstanding, self._pending = self._pending, {}
            for future in outstanding.values():
                future.set_exception(ConnectionLost())

    def _rpc(self, command, params=None, *, timeout_s=None):
        future = Future()
        with self._lock:
            if self._closed.is_set():
                raise ConnectionLost()
            identifier = next(self._counter)
            self._pending[identifier] = future
        try:
            with self._send_lock:
                self._socket.send(
                    json.dumps(
                        dict(type="req", msg_id=identifier, cmd=command, params=params or {}),
                        ensure_ascii=False,
                        allow_nan=False,
                    )
                )
            response = future.result(timeout=timeout_s or self.request_timeout_s)
            if response["code"]:
                raise ArmError(response["code"], response.get("ms", "请求失败"))
            return response["data"]
        finally:
            with self._lock:
                self._pending.pop(identifier, None)

    def _accept(self, command, params):
        params = dict(params, request_id=uuid.uuid4().hex)
        try:
            return self._rpc(command, params)
        except FutureTimeout:
            # 受理超时只查询原编号，不重新发送可能产生运动的命令。
            deadline = time.monotonic() + self.request_timeout_s
            while time.monotonic() < deadline:
                try:
                    result = self._rpc("request_get", dict(request_id=params["request_id"]))["accepted"]
                    if result is not None:
                        return result
                except FutureTimeout:
                    break
                self._closed.wait(0.1)
            confirmed = self._stop_after_error()
            raise ArmError(3107, "请求受理结果未知，已请求停止", stop_confirmed=confirmed)

    def _heartbeats(self):
        while not self._closed.wait(0.5):
            session = self._session
            if session:
                try:
                    self._rpc("session_heartbeat", dict(session_id=session["session_id"]), timeout_s=1.2)
                except ArmError as error:
                    if error.code == 3101:
                        self._session = None
                    else:
                        self._socket.close()
                except Exception:
                    self._socket.close()

    def get_state(self):
        return self._rpc("get_state")

    def _enabled(self, enabled):
        self._rpc("enable_motor" if enabled else "disable_motor", dict(motor_id=0))
        deadline = time.monotonic() + self.request_timeout_s
        while time.monotonic() < deadline:
            state = self.get_state()
            values = state.get("motor_enabled", [])
            if state.get("state_available") and len(values) == 6 and all(v == enabled for v in values):
                return state
            self._closed.wait(0.05)
        raise ArmError(2001, "使能状态未在期限内得到确认")

    def enable(self):
        return self._enabled(True)

    def disable(self):
        return self._enabled(False)

    @contextmanager
    def control(self, name="Python 控制"):
        if self._session:
            raise ArmError(2003, "控制会话不能嵌套")
        session = self._accept("session_open", dict(name=name))
        self._session = session
        try:
            yield self
        except BaseException:
            self._stop_after_error(session["run_id"])
            raise
        else:
            if self._session:
                self._rpc("session_close", dict(session_id=session["session_id"]))
        finally:
            self._session = None

    def _ready(self):
        session = self._session
        if not session:
            raise ArmError(3101, "请先进入 arm.control() 控制会话")
        while True:
            state = self._rpc("execution_state")
            if state.get("session_id") != session["session_id"] or state.get("status") in TERMINAL | {
                "STOPPING"
            }:
                raise ArmError(3101, "控制会话已经结束")
            if state.get("status") == "RUNNING":
                return session["session_id"]
            self._closed.wait(0.05)

    def _motion(self, command, params, wait=True, timeout_s=None):
        params = dict(params, session_id=self._ready())
        handle = MotionHandle(self, self._accept(command, params))
        return handle.wait(timeout_s) if wait else handle

    def movej(
        self,
        joints_deg,
        *,
        speed_deg_s=2.0,
        acceleration_deg_s2=5.0,
        output_speed_deg_s=2.0,
        wait=True,
        timeout_s=None,
    ):
        return self._motion(
            "movej",
            dict(
                target=list(joints_deg),
                vel=speed_deg_s,
                acc=acceleration_deg_s2,
                output_speed_limit=output_speed_deg_s,
            ),
            wait,
            timeout_s,
        )

    def move_direct(self, joints_deg, *, speed_deg_s=2.0, output_speed_deg_s=2.0, wait=True, timeout_s=None):
        return self._motion(
            "move",
            dict(targets=list(joints_deg), max_speed=speed_deg_s, output_speed_limit=output_speed_deg_s),
            wait,
            timeout_s,
        )

    def movel(
        self,
        position_mm,
        rpy_deg,
        *,
        speed_mm_s=10.0,
        acceleration_mm_s2=20.0,
        angular_speed_deg_s=2.0,
        angular_acceleration_deg_s2=5.0,
        output_speed_deg_s=2.0,
        wait=True,
        timeout_s=None,
    ):
        return self._motion(
            "movel",
            dict(
                target=list(position_mm) + list(rpy_deg),
                linear_vel=speed_mm_s,
                linear_acc=acceleration_mm_s2,
                angular_vel=angular_speed_deg_s,
                angular_acc=angular_acceleration_deg_s2,
                output_speed_limit=output_speed_deg_s,
            ),
            wait,
            timeout_s,
        )

    def step_joint(
        self, joint_index, delta_deg, *, speed_deg_s=2.0, output_speed_deg_s=2.0, wait=True, timeout_s=None
    ):
        return self._motion(
            "step_move",
            dict(
                space="joint",
                axis=joint_index,
                delta=delta_deg,
                max_speed=speed_deg_s,
                output_speed_limit=output_speed_deg_s,
            ),
            wait,
            timeout_s,
        )

    def step_position(
        self, axis, delta_mm, *, speed_mm_s=10.0, output_speed_deg_s=2.0, wait=True, timeout_s=None
    ):
        return self._motion(
            "step_move",
            dict(
                space="position",
                axis=axis,
                delta=delta_mm,
                linear_vel=speed_mm_s,
                output_speed_limit=output_speed_deg_s,
            ),
            wait,
            timeout_s,
        )

    def step_orientation(
        self, axis, delta_deg, *, speed_deg_s=2.0, output_speed_deg_s=2.0, wait=True, timeout_s=None
    ):
        return self._motion(
            "step_move",
            dict(
                space="orientation",
                axis=axis,
                delta=delta_deg,
                angular_vel=speed_deg_s,
                output_speed_limit=output_speed_deg_s,
            ),
            wait,
            timeout_s,
        )

    def move_to_point(
        self,
        point,
        *,
        motion="movej",
        speed_deg_s=2.0,
        acceleration_deg_s2=5.0,
        speed_mm_s=10.0,
        acceleration_mm_s2=20.0,
        output_speed_deg_s=2.0,
        wait=True,
        timeout_s=None,
    ):
        motion = "move" if motion == "direct" else motion
        parameters = (
            dict(max_speed=speed_deg_s)
            if motion == "move"
            else dict(vel=speed_deg_s, acc=acceleration_deg_s2)
        )
        if motion == "movel":
            parameters = dict(
                linear_vel=speed_mm_s,
                linear_acc=acceleration_mm_s2,
                angular_vel=speed_deg_s,
                angular_acc=acceleration_deg_s2,
            )
        return self._motion(
            "move_to_point",
            dict(point=point, motion=motion, output_speed_limit=output_speed_deg_s, **parameters),
            wait,
            timeout_s,
        )

    def wait(self, seconds, *, timeout_s=None):
        return self._motion("wait", dict(seconds=seconds), True, timeout_s)

    def pause(self):
        return self._rpc("execution_pause", self._session_params())

    def resume(self):
        return self._rpc("execution_resume", self._session_params())

    def _session_params(self):
        return dict(session_id=self._session["session_id"]) if self._session else {}

    def stop(self):
        result = self._rpc("execution_stop", self._session_params(), timeout_s=20.0)
        self._session = None
        if not result.get("stop_confirmed"):
            raise ArmError(
                result.get("code") or 3006, result.get("message", "停止未确认"), stop_confirmed=False
            )
        return result

    def _stop_after_error(self, run_id=None):
        try:
            run_id = run_id or (self._session or {}).get("run_id")
            if run_id:
                result = self._rpc("execution_stop", dict(run_id=run_id), timeout_s=20.0)
                self._session = None
                return result.get("stop_confirmed", False)
            return self.stop().get("stop_confirmed", False)
        except Exception:
            return False

    def close(self):
        if self._session and not self._closed.is_set():
            self._stop_after_error()
        self._session = None
        self._socket.close()
        self._closed.set()
        if threading.current_thread() is not self._receiver:
            self._receiver.join(timeout=2.0)
