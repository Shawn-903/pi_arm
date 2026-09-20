# pi_arm_websocket

Python WebSocket ↔ ROS 2 网关。将网页 JSON 命令转换为 manager 服务/动作，推送状态，并管理持久点位和 URDF FK。自身不直接操作 SocketCAN；HTML 静态服务由根目录启动脚本另行启动。

## 文件与节点

| 模块 | 职责 |
| --- | --- |
| [gateway.py](pi_arm_websocket/gateway.py) | `pi_arm_websocket` 节点、ROS 客户端、连接管理、任务追踪和状态推送 |
| [protocol.py](pi_arm_websocket/protocol.py) | JSON 校验、错误码、字段精度及合法命令 |
| [points.py](pi_arm_websocket/points.py) | 点位库、版本冲突校验、文件锁/fsync/原子替换 |
| [frames.py](pi_arm_websocket/frames.py) | 同帧六关节的 URDF FK，包含 mimic、各关节坐标系与 tool0 |
| [presets.py](pi_arm_websocket/presets.py) | 由当前标定派生打包姿态，六轴零位和可用性 |
| [motion_limits.py](pi_arm_websocket/motion_limits.py) | 角速度默认 5°/s、上限 10°/s |
| [websocket.launch.py](launch/websocket.launch.py) | 独立网关入口；整体运行通常用 bringup |

默认 host `0.0.0.0`、port `8765`、推送周期 `100 ms`、状态超时 `1 s`、请求超时 `5 s`。其他参数包括 `hardware_type`、`read_only`、`commissioning`、`calibration_file`、`points_file`、`frame_model_file`、`joint_names`。

普通模式订阅 `/pi_arm/state` 并使用 manager；只读/commissioning 订阅 `/pi_arm/hardware_state`。只读不建管理/运动客户端；commissioning 只建其专属管理和 AxisTest 服务，不能借普通运动命令绕过模式。

## 协议

请求格式：`{"type":"req","msg_id":1,"cmd":"point_list","params":{}}`。
响应：`type=rsp`，对应 `msg_id`，包含 `code`、`ms`、`data`；成功 code 为 0。状态为 `type=state`，包含任务、硬件许可、坐标与能力字段，前端按能力开放功能。

| 命令 | 路由/语义 |
| --- | --- |
| `motor_move` / `move` | 单关节 / 完整六关节目标，经 DirectMove；不是裸电机协议控制 |
| `output_move` | 六个模型输出轴角，经绳索反解和 DirectMove |
| `move_preset` | 打包 / 零位，前端可设置关节速度，仍受输出轴限速约束 |
| `movej` / `movejs` / `movel` | 对应 ROS MoveJ / MoveJs / MoveL |
| `preview_motion` | joint/output/ik/lin 只读预览 |
| `enable_motor` / `disable_motor` / `reset_motor` / `reboot_motor` | 管理服务；reset 为清错，reboot 为选定电机重启 |
| `stop_motion` / `motor_stop` | 停止链，不是默认的到位保持动作 |
| `set_teaching` | 显式进入/退出实机拖拽示教 |
| `axis_test` / `axis_test_heartbeat` / `axis_test_stop` | commissioning 专用单轴测试 |
| `point_list` / `point_save` / `point_delete` | 点位库读取/存储/删除，不发运动 |

网页目标使用 °、mm；ROS 动作转为 rad、m。RPY 使用固定轴 XYZ，即 `Rz(yaw) Ry(pitch) Rx(roll)`，坐标 base_link、控制点 tool0。预览轨迹 TCP 路径使用 m，不可按 mm 再传入渲染。具体必填字段及限制以 protocol/gateway 实现和 `test/` 为准。

运动响应里的 task_id 表示“已接受”，终态见后续 state。接受后迟迟拿不到任务号会取消并确认；接受响应晚到也处理取消，不允许返回普通失败却留下无人跟踪的运动。

## 点位数据与精度

默认文件是 `~/.local/share/pi_arm/points-real.json` / `points-mock.json`，也可指定 `points_file`；最多 500 个点、2 MiB、名称唯一。变更携带库 `revision`，并发冲突要求刷新；损坏文件保留原内容并报错。

点位包含类型、来源、values、坐标系/单位、关节顺序、模型哈希、标定 ID、时间等。反馈来源要求有效新鲜静止的服务端数据，附原始协议角和 ROS 时间；模型预览不是实测。保存不代表目标可执行。

点位与常用角度/XYZ/RPY 的输出保留最多三位小数；磁盘原始存档、ROS/CAN、弧度诊断、四元数和时间不因显示精度而舍入。base_frames 使用同帧 FK，不依赖另一时刻的 TF 缓存。

完整操作与备份方式见 [web/README.md](../../web/README.md)。重复循环由浏览器调度，网关不提供后台循环队列，也没有普通任务的断线自动停止；commissioning 另有心跳。

## 验证

`test/` 覆盖请求校验、模式门禁、单位、预览、任务超时、点位持久化/并发/损坏保护、FK 等。离线 UI 与 mock 端到端入口见 [scripts/README.md](../../scripts/README.md)。当前不提供身份鉴权或 TLS，前端角色不是权限边界。
