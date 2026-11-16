"""Task orchestration without a CAN connection or a second copy of zero offsets."""

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from pi_arm_interfaces.srv import SetZeroReference

from .client import zero_request


class TasksNode(Node):
    def __init__(self):
        super().__init__("pi_arm_tasks")
        self._busy = threading.Lock()
        self._group = ReentrantCallbackGroup()
        self._hardware = self.create_client(
            SetZeroReference, "/pi_arm/commissioning/set_zero_reference",
            callback_group=self._group,
        )
        self.create_service(
            SetZeroReference, "/pi_arm/tasks/set_zero_reference", self._set_zero,
            callback_group=self._group,
        )

    def _set_zero(self, request, response):
        try:
            zero_request(request.reference_joint_positions_rad, request.reference_pose_confirmed)
        except (TypeError, ValueError) as error:
            response.code, response.message = 1004, str(error)
            return response
        if not self._hardware.service_is_ready():
            response.code = 2001
            response.message = "software zero requires the commissioning hardware service"
            return response
        if not self._busy.acquire(blocking=False):
            response.code, response.message = 2003, "a zero-reference task is still pending"
            return response
        future = None
        try:
            completed = threading.Event()
            future = self._hardware.call_async(request)
            future.add_done_callback(lambda _: completed.set())
            if not completed.wait(4.0):
                response.code = 2002
                response.message = "zero-reference result unknown; inspect software_zero_id, do not retry automatically"
                return response
            return future.result()
        except Exception as error:
            response.code, response.message = 2002, f"zero-reference task failed: {error}"
            return response
        finally:
            if future is not None and not future.done():
                future.add_done_callback(lambda _: self._busy.release())
            else:
                self._busy.release()


def main(args=None):
    rclpy.init(args=args)
    node = TasksNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
