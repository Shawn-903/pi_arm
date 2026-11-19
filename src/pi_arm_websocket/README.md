# pi_arm_websocket

Python WebSocket ↔ ROS 2 网关，负责网页命令、状态推送、持久化点位和模型坐标显示。普通运动请求转给 manager，维护请求转给维护节点或任务服务。坐标显示通过 URDF 正运动学（FK）计算；HTML 静态服务由根目录启动脚本另行启动。

## 文件与节点

| 模块 | 职责 |
| --- | --- |
| [gateway.py](pi_arm_websocket/gateway.py) | 进程入口、WebSocket 连接管理和状态推送 |
| [state_payload.py](pi_arm_websocket/state_payload.py) | 将同一状态快照转换为协议字段，处理单位、无反馈默认值和非有限数值 |
| [ros_bridge.py](pi_arm_websocket/ros_bridge.py) | `pi_arm_websocket` 节点、ROS 客户端、反馈快照与动作生命周期 |
| [commands.py](pi_arm_websocket/commands.py) | 命令参数/模式校验、单位转换、服务和动作分发 |
| [rotations.py](pi_arm_websocket/rotations.py) | 命令与反馈共享的角度单位、RPY/四元数换算 |
| [protocol.py](pi_arm_websocket/protocol.py) | JSON 校验、错误码、字段精度及合法命令 |
| [points.py](pi_arm_websocket/points.py) | 点位库、版本冲突校验、文件锁/fsync/原子替换 |
| [frames.py](pi_arm_websocket/frames.py) | 同帧六关节的 URDF FK，包含 mimic、各关节坐标系与 tool0 |
| [presets.py](pi_arm_websocket/presets.py) | 定义机械零位（打包姿态）的关节角，提供打包和零位运动目标及其可用状态 |
| [motion_limits.py](pi_arm_websocket/motion_limits.py) | 角速度默认 5°/s、上限 10°/s |
| [websocket.launch.py](launch/websocket.launch.py) | 独立网关入口；整体运行通常用 bringup |

默认 host `0.0.0.0`、port `8765`、推送周期 `100 ms`、状态超时 `1 s`、请求超时 `5 s`。其他参数包括 `hardware_type`、`read_only`、`commissioning`、`calibration_file`、`points_file`、`frame_model_file`、`joint_names`。

普通模式订阅 `/pi_arm/state` 并使用 manager；只读/commissioning 订阅 `/pi_arm/hardware_state`。只读模式保留状态和点位功能；commissioning 创建维护管理、AxisTest 及软件标零任务的服务客户端，使用独立维护链路。

## 协议

请求格式：`{"type":"req","msg_id":1,"cmd":"point_list","params":{}}`。
响应：`type=rsp`，通过 `msg_id` 对应请求。`code=0` 表示请求成功，`ms` 保存文字说明，`data` 保存结果数据。状态为 `type=state`，包含任务、硬件许可、坐标与能力字段，前端按能力开放功能。

编写客户端时，连接 `ws://树莓派IP:8765`，发送上面的 JSON 即可读取点位库；成功响应的 `data` 包含 `revision` 和 `points`。连接中会持续收到状态推送，因此按 `type` 区分状态与请求响应，再用 `msg_id` 对应已发送的请求。连续请求使用不同的 `msg_id`，网关返回的 `ms` 可直接显示给操作者。

| 命令 | 路由/语义 |
| --- | --- |
| `motor_move` / `move` | 单关节 / 完整六关节目标，经 DirectMove 执行 |
| `output_move` | 六个模型输出轴角，经绳索反解和 DirectMove |
| `move_preset` | 打包 / 零位，前端可设置关节速度，仍受输出轴限速约束 |
| `set_zero_reference` | 维护模式软件标零。网页以状态中的 `mechanical_zero_joints_deg` 为参考角，通过 `reference_joint_positions_deg` 传入六个关节角（°）。现场确认后将 `reference_pose_confirmed` 设为 true，采集成功返回参考编号 |
| `movej` / `movejs` / `movel` | 对应 ROS MoveJ / MoveJs / MoveL |
| `preview_motion` | joint/output/ik/lin 只读预览 |
| `enable_motor` / `disable_motor` / `reset_motor` / `reboot_motor` | 管理服务；reset 为清错，reboot 为选定电机重启 |
| `stop_motion` / `motor_stop` | 普通模式取消当前任务并请求保持当前位置；`motor_stop` 同样停止整条任务。commissioning 模式转给 AxisTest 停止服务 |
| `set_teaching` | 显式进入/退出实机拖拽示教 |
| `axis_test` / `axis_test_heartbeat` / `axis_test_stop` | commissioning 专用单轴测试 |
| `point_list` / `point_save` / `point_delete` | 点位库读取、存储和删除 |

网页目标使用 °、mm，网关转换为 ROS 动作的 rad、m。RPY 使用固定轴 XYZ，即 `Rz(yaw) Ry(pitch) Rx(roll)`，坐标系为 base_link，控制点为 tool0。返回的预览轨迹 TCP 路径使用 m，渲染时按米制读取。请求字段定义见 `protocol.py` 和 `commands.py`，调用示例可查阅 `test/`。

进程入口是 `pi_arm_websocket.gateway:main`。`gateway` 管理连接，调用 `commands` 分发请求、通过 `ros_bridge` 访问 ROS，再由 `state_payload` 将反馈快照转换为网页消息。

运动响应中的 `task_id` 用于跟踪任务，后续 `state` 提供执行进度和终态。目标接受或任务号获取超时时，网关会处理迟到响应，并请求取消已接受的目标。

## 点位数据与精度

默认文件是 `~/.local/share/pi_arm/points-real.json` / `points-mock.json`，也可指定 `points_file`；最多 500 个点、2 MiB、名称唯一。变更携带库 `revision`，并发冲突要求刷新；损坏文件保留原内容并报错。

点位包含类型、来源、values、坐标系/单位、关节顺序、模型哈希、标定 ID、时间等。选择反馈来源时，服务端采集有效、新鲜、静止的数据，并附原始协议角和 ROS 时间；选择模型预览来源时，保存当前预览目标。执行点位时由 manager 检查限位及规划条件。

点位与常用角度/XYZ/RPY 的网页输出保留最多三位小数；磁盘原始存档、ROS/CAN、弧度诊断、四元数和时间保留内部精度。`base_frames` 使用同一帧关节反馈计算 FK。

步骤程序和重复循环由树莓派执行服务调度，网页断线后继续。Python 控制会话、请求去重、运行历史与完整参数见 [执行接口](../../docs/execution.md)。点位支持 `point_update`、`point_import`（先预检再提交）和 `point_export`（返回完整精度的 JSON 文本）。网页操作见 [web/README.md](../../web/README.md)。

## 验证

`test/` 覆盖请求校验、模式门禁、单位、预览、任务超时、点位持久化/并发/损坏保护、FK 等。离线 UI 与 mock 端到端入口见 [scripts/README.md](../../scripts/README.md)。
