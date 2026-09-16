import asyncio, threading
from types import SimpleNamespace
from concurrent.futures import Future
from pi_arm_websocket.gateway import RosBridge, ProtocolError
from pi_arm_interfaces.msg import RobotState, TaskState
class Logger:
    def __getattr__(self,name):return lambda msg: None
class Goal:
    accepted=True
    def __init__(self): self.result=Future(); self.cancelled=False
    def get_result_async(self):return self.result
    def cancel_goal_async(self):self.cancelled=True
class Client:
    def __init__(self):self.goal=Goal()
    def wait_for_server(self,timeout):return True
    def send_goal_async(self,*args,**kwargs):
        f=Future();f.set_result(self.goal);return f
async def main():
    client=Client()
    state=SimpleNamespace(state=RobotState.READY,state_name='READY',task=SimpleNamespace(status=TaskState.IDLE))
    bridge=SimpleNamespace(timeout=.02,action_clients={'direct_move':client},_goal_lock=threading.Lock(),_active_goals=[],get_logger=lambda:Logger(),state_snapshot=lambda:state)
    async def wait(f):return await RosBridge.wait_future(bridge,f)
    bridge.wait_future=wait
    try:await RosBridge.send_goal(bridge,'direct_move',object())
    except ProtocolError as e:
        print('gateway_error=',str(e),'goal_accepted=',client.goal.accepted,'cancel_requested=',client.goal.cancelled,'active_goal_count=',len(bridge._active_goals))
    else:raise AssertionError('expected missing feedback timeout')
    assert not client.goal.cancelled and len(bridge._active_goals)==1
asyncio.run(main())
