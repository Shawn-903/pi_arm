import asyncio
import threading
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from pi_arm_interfaces.msg import RobotState, TaskState
from pi_arm_websocket.gateway import RosBridge
from pi_arm_websocket.protocol import ProtocolError


def completed(value):
    future = Future()
    future.set_result(value)
    return future


class Goal:
    accepted = True

    def __init__(self, finish_on_cancel=True):
        self.result = Future()
        self.cancel_count = 0
        self.finish_on_cancel = finish_on_cancel

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_count += 1
        if self.finish_on_cancel and not self.result.done():
            self.result.set_result(SimpleNamespace(status=5))
        return completed(SimpleNamespace(goals_canceling=[object()]))


class Client:
    def __init__(self, goal, late=False, feedback=False):
        self.goal = goal
        self.acceptance = Future() if late else completed(goal)
        self.feedback = feedback

    def wait_for_server(self, timeout):
        return True

    def send_goal_async(self, goal, feedback_callback):
        if self.feedback:
            feedback_callback(SimpleNamespace(feedback=SimpleNamespace(task_id=42)))
        return self.acceptance


class Bridge:
    wait_future = RosBridge.wait_future
    send_goal = RosBridge.send_goal
    _track_goal = RosBridge._track_goal
    _cancel_late_goal = RosBridge._cancel_late_goal
    _cancel_accepted_goal = RosBridge._cancel_accepted_goal

    def __init__(self, client):
        self.timeout = 0.02
        self.action_clients = {"direct": client}
        self._goal_lock = threading.Lock()
        self._active_goals = []

    def state_snapshot(self):
        return SimpleNamespace(state=RobotState.READY, state_name="READY",
                               task=SimpleNamespace(status=TaskState.IDLE))

    def get_logger(self):
        return SimpleNamespace(**{key: lambda message: None for key in ("error", "warning", "info")})


def test_accepted_goal_without_feedback_is_cancelled_and_terminal_result_checked():
    goal = Goal()
    bridge = Bridge(Client(goal))
    with pytest.raises(ProtocolError, match="目标已结束"):
        asyncio.run(bridge.send_goal("direct", object()))
    assert goal.cancel_count == 1
    assert not bridge._active_goals


def test_cancel_ack_without_terminal_result_reports_unconfirmed_stop():
    goal = Goal(finish_on_cancel=False)
    bridge = Bridge(Client(goal))
    with pytest.raises(ProtocolError, match="停止未确认"):
        asyncio.run(bridge.send_goal("direct", object()))
    assert goal.cancel_count == 1
    assert bridge._active_goals == [goal]


def test_late_acceptance_is_cancelled_even_after_request_event_loop_closed():
    goal = Goal()
    client = Client(goal, late=True)
    bridge = Bridge(client)
    with pytest.raises(ProtocolError, match="接受状态未知"):
        asyncio.run(bridge.send_goal("direct", object()))
    client.acceptance.set_result(goal)
    assert goal.cancel_count == 1
    assert not bridge._active_goals


def test_late_rejection_does_not_issue_cancel():
    goal = Goal()
    goal.accepted = False
    client = Client(goal, late=True)
    bridge = Bridge(client)
    with pytest.raises(ProtocolError):
        asyncio.run(bridge.send_goal("direct", object()))
    client.acceptance.set_result(goal)
    assert goal.cancel_count == 0


def test_timely_feedback_keeps_goal_active_until_result():
    goal = Goal()
    bridge = Bridge(Client(goal, feedback=True))
    assert asyncio.run(bridge.send_goal("direct", object())) == 42
    assert goal.cancel_count == 0
    assert bridge._active_goals == [goal]
    goal.result.set_result(SimpleNamespace(status=4))
    assert not bridge._active_goals
