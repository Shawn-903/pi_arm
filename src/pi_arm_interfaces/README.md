# pi_arm_interfaces

ROS 2 公共接口定义包。硬件、manager、任务包和网关通过这里的消息、服务和动作交换数据，rosidl 为它们生成 C++/Python 类型。除显式命名为 `deg` 的调试字段，ROS 运动数据使用 SI 单位：rad、rad/s、m、s。

## 消息

| 定义 | 内容及常用 Topic |
| --- | --- |
| [HardwareState](msg/HardwareState.msg) | 六轴反馈、原始协议/输出角、标定、诊断、写入许可及示教/停止状态；`/pi_arm/hardware_state` |
| [RobotState](msg/RobotState.msg) | manager 聚合整机状态、硬件和任务；`/pi_arm/state` |
| [TaskState](msg/TaskState.msg) | task_id、进度、结果、到位阶段、目标/误差；IDLE=0、RUNNING=1、SUCCEEDED=2、CANCELED=3、FAILED=4 |

## 动作

| 定义 | 正常 manager 入口 | 语义 |
| --- | --- | --- |
| [DirectMove](action/DirectMove.action) | `/pi_arm/direct_move` | 关节目标或 `output_space` 输出轴目标，受限位和限速约束的插值 |
| [MoveJ](action/MoveJ.action) | `/pi_arm/movej` | 六轴关节目标经 MoveIt 规划 |
| [MoveJs](action/MoveJs.action) | `/pi_arm/movejs` | 带时间的六关节轨迹 |
| [MoveL](action/MoveL.action) | `/pi_arm/movel` | base_link 下 tool0 目标，Pilz LIN |

调用动作后，从反馈取得任务号，通过终态和到位核验结果判断完成情况。`max_output_velocity=0` 选择默认 5°/s，非零使用 rad/s。关节顺序见 [源码索引](../README.md)。

## 服务

| 定义 | 作用 |
| --- | --- |
| [ManageMotor](srv/ManageMotor.srv) | 按服务名使能/关闭/清错/重启；通常 ID 0 表示全部，但重启只接受 1–6 |
| [SetMotionArmed](srv/SetMotionArmed.srv) | 硬件内部写入租约、输出轴限速；`armed=false, hold_position=true` 时，非零 lease 保留到位目标，零 lease 停止并保持当前编码器位置 |
| [SetTeaching](srv/SetTeaching.srv) | 显式进入拖拽示教或退出并保持当前位置 |
| [StopMotion](srv/StopMotion.srv) | 取消当前运动并保持当前位置，返回保持确认结果或失败原因 |
| [AxisTest](srv/AxisTest.srv) | commissioning 专用 start/heartbeat/stop，步进与速度单位为 °、°/s |
| [SetZeroReference](srv/SetZeroReference.srv) | 确认已知实物姿态并采集软件零点，参考关节角使用 rad。前端/Python 任务入口为 `/pi_arm/tasks/set_zero_reference`，转调维护模式的 `/pi_arm/commissioning/set_zero_reference`；调用方式见 [任务包](../pi_arm_tasks/README.md) |
| [PreviewMotion](srv/PreviewMotion.srv) | 根据 joint/output/ik/lin 请求计算目标和轨迹，供前端预览 |

`SetMotionArmed` 的租约是一次运动的临时写入许可，用 `lease_id` 关联任务与清理请求。正常到位释放该许可时保留最终目标；用户停止使用零 lease 捕获当前位置。该服务由控制链内部协调，上层程序通过动作或 `StopMotion` 发起操作。

修改字段时同步硬件发布、manager、网关与消费者测试。此包的验证以接口生成及各消费者测试为主；可用 `ros2 interface show pi_arm_interfaces/msg/RobotState` 检查安装结果。
