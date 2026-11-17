"""软件标零请求构造和 Python 客户端。"""

import math

from pi_arm_interfaces.srv import SetZeroReference


def zero_request(reference_joint_positions_rad, reference_pose_confirmed):
    values = list(reference_joint_positions_rad)
    if len(values) != 6 or any(isinstance(v, bool) or not math.isfinite(v) for v in values):
        raise ValueError("reference pose must contain six finite joint angles in radians")
    if reference_pose_confirmed is not True:
        raise ValueError("confirm the physical reference pose before setting software zero")
    return SetZeroReference.Request(
        reference_joint_positions_rad=[float(v) for v in values],
        reference_pose_confirmed=True,
    )


class ZeroReferenceClient:
    def __init__(self, node):
        self._zero = node.create_client(SetZeroReference, "/pi_arm/tasks/set_zero_reference")

    def wait_for_service(self, timeout_sec=3.0):
        return self._zero.wait_for_service(timeout_sec=timeout_sec)

    def set_zero_reference(self, reference_joint_positions_rad, *, reference_pose_confirmed):
        """Return a ROS future; spin the caller's node and inspect response.success."""
        request = zero_request(reference_joint_positions_rad, reference_pose_confirmed)
        if not self._zero.service_is_ready():
            raise RuntimeError("zero-reference service is unavailable; start commissioning first")
        return self._zero.call_async(request)
