# pi_arm_interfaces

ROS 2 公共接口定义包，由 rosidl 生成 C++/Python 类型；自身不启动节点、不控制硬件。除显式命名为 `deg` 的调试字段，ROS 运动数据使用 SI 单位：rad、rad/s、m、s。

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

动作反馈提供任务号，接受目标并不代表执行成功；终态与到位核验共同决定结果。`max_output_velocity=0` 选择默认 5°/s，非零使用 rad/s。关节顺序见 [源码索引](../README.md)。

## 服务

| 定义 | 作用 |
| --- | --- |
| [ManageMotor](srv/ManageMotor.srv) | 按服务名使能/关闭/清错/重启；通常 ID 0 表示全部，但重启只接受 1–6 |
| [SetMotionArmed](srv/SetMotionArmed.srv) | 硬件内部写入租约、输出轴限速，成功到位释放时可 `hold_position` |
| [SetTeaching](srv/SetTeaching.srv) | 显式进入拖拽示教或退出并保持当前位置 |
| [StopMotion](srv/StopMotion.srv) | 停止当前运动并返回结果 |
| [AxisTest](srv/AxisTest.srv) | commissioning 专用 start/heartbeat/stop，步进与速度单位为 °、°/s |
| [PreviewMotion](srv/PreviewMotion.srv) | joint/output/ik/lin 只读预览，不执行；返回轨迹仅供显示 |

修改字段时同步硬件发布、manager、网关与消费者测试。此包的验证以接口生成及各消费者测试为主；可用 `ros2 interface show pi_arm_interfaces/msg/RobotState` 检查安装结果。
