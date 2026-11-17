# Pi Arm Python SDK

在 Windows 或树莓派上通过 WebSocket 控制机械臂。需要 Python 3.11 及以上版本，客户端无需 ROS。服务端需启动 Pi Arm 的 manager、执行服务和网关。

```sh
python -m venv .venv
# Windows：.venv\Scripts\activate；Linux：source .venv/bin/activate
python -m pip install ./sdk/python
```

连接只用于查询。运动前显式使能，并进入独占控制会话：

```python
from pi_arm_sdk import Arm

with Arm("192.168.2.110") as arm:
    print(arm.info)
    print(arm.get_state())
    arm.enable()
    with arm.control(name="往返运动"):
        for _ in range(3):
            arm.move_to_point("A", motion="movej")
            arm.wait(1.0)
            arm.move_to_point("B", motion="movej")
```

`Arm(host, port=8765, request_timeout_s=5, motion_timeout_s=300)` 建立连接；可传完整 WebSocket URL。`close()` 关闭连接。`enable()`、`disable()` 返回已核对的使能状态。`get_state()` 返回关节、末端、硬件、任务、反馈新鲜度和执行状态。

`control(name=...)` 在运动、等待、暂停期间保留控制权。正常离开会话释放控制权；异常退出或关闭仍有任务的会话会请求停止。断线或心跳过期后服务器停止并结束会话，重新连接后需创建新会话。

| 方法 | 主要参数 |
|---|---|
| `movej` | `joints_deg` 六关节角；`speed_deg_s=2`、`acceleration_deg_s2=5` |
| `move_direct` | `joints_deg` 六关节角；`speed_deg_s=2` |
| `movel` | `position_mm` 三维位置、`rpy_deg` 三维姿态；`speed_mm_s=10`、`acceleration_mm_s2=20`、`angular_speed_deg_s=2` |
| `step_joint` | `joint_index` 0–5、`delta_deg`、`speed_deg_s=2` |
| `step_position` | `axis` 0–2 对应基坐标 X/Y/Z、`delta_mm`、`speed_mm_s=10` |
| `step_orientation` | `axis` 0–2 对应绕基坐标 X/Y/Z 旋转、`delta_deg`、`speed_deg_s=2` |
| `move_to_point` | 点位 ID 或名称；`motion='movej'`，也支持 `move`、`movel` |
| `wait` | `seconds`，暂停时保存剩余时间 |

关节顺序由 `arm.info['model']['joints']` 给出，限位由模型元数据提供。末端为 `tool0`，目标使用 `base_link`。MoveL 的旋转加速度由后端规划配置决定，`angular_acceleration_deg_s2` 不构成独立约束。关节及角速度上限 10 度/秒，`output_speed_deg_s` 范围 1/6–10 度/秒。

运动默认等到后端核验到位，返回 `MotionResult`（`task_id`、`run_id`、`step_id`、`status`、`elapsed_s`、`data`）。`wait=False` 返回 `MotionHandle`，可调用 `query()`、`done()`、`wait(timeout_s=...)`、`cancel()`。同一会话只能执行一个任务；不会自动排队。`move_to_point` 可传 `speed_deg_s`、`acceleration_deg_s2`、`speed_mm_s`、`acceleration_mm_s2`、`output_speed_deg_s`，按所选运动方式使用相应参数。

`pause()` 在运动完成后暂停，等待步骤立即暂停计时。暂停后新的 SDK 运动调用等待继续，普通 Python 计算仍可执行。`resume()` 检查条件后继续；`stop()` 立即禁止下一步、取消当前运动并请求保持当前位置，返回停止确认结果。停止后的运行不能继续。

请求超时会查询原请求的受理结果，不重新发送运动。运动等待超时会请求停止，并抛出 `MotionTimeout`。取消、失败、中断抛出 `ArmError`，包含 `code`、`task_id`、`run_id`、`step_id`、`stop_confirmed`。断线抛出 `ConnectionLost`。异常处理示例见 `examples/`。

点位通过 `arm.points` 管理：`list(search='')`、`get(id_or_name)`、`save(...)`、`update(...)`、`delete(...)`、`export()`、`import_points(...)`。导入先以 `commit=False` 预检，再传预检返回的 `revision` 和 `commit=True` 提交；冲突默认 `skip`，`update` 仅更新相同 ID。`export()` 返回完整精度的存档。保存、编辑和载入点位均不会自动运动。
