# pi_arm_manager

正常控制模式的任务与运动管理节点 `pi_arm_manager_node`。它接收上层动作/服务、检查当前状态、规划或生成轨迹、向 JTC 提交执行，并独立核验到位结果。不可由网页绕过它直接写 CAN。

## 控制流程

1. 读取 `/pi_arm/hardware_state`，通过 [state_aggregator.hpp](include/pi_arm_manager/state_aggregator.hpp) 发布 `/pi_arm/state`。
2. 互斥接收 DirectMove / MoveJ / MoveJs / MoveL；起点可使用反馈容差，目标使用标准关节限位和输出轴限位。
3. `prepare_trajectory` 根据绳索 Jacobian 的绝对值上界、协议速度分辨率与关节速度上限延长轨迹时间。输出轴慢的一路决定同步运动耗时。
4. 获取本次位置写入租约，通过 `/joint_trajectory_controller/follow_joint_trajectory` 执行；用户取消/停止先关闭硬件轨迹写入并确认当前位置保持，再取消控制器目标。
5. JTC 成功后仍检查新鲜反馈、六轴目标误差及静止，满足稳定时间后以位置保持方式释放租约，再复核结果。请求接受或“电机不动”不能单独判定成功。

默认到位容差 0.2°、稳定时间 0.5 s、收敛超时 10 s，配置在 [manager.yaml](../pi_arm_bringup/config/manager.yaml)；与 [controllers.yaml](../pi_arm_bringup/config/controllers.yaml) 配合。普通任务 `motion_timeout_sec` 默认 120 s，以节点参数与轨迹执行逻辑为准；网页循环的等待上限不覆盖后端超时。

`StopMotion` 在运动中和到点等待/空闲时都使用当前位置保持，覆盖关节、输出轴、预设姿态及笛卡尔路径。停止不再生成一条被已关闭写入门禁挡住的“保持轨迹”。取消、结果回调及作用域清理共享停止策略；迟到的控制器 goal 接受回包也会取消。停止处理期间禁止接收新任务。保持失败的动作结果为 3006，不把它报告为成功取消；硬件故障/断线/失能仍有更高优先级并保留故障停止。

## 接口与文件

- [manager_node.cpp](src/manager_node.cpp)：动作、管理/停止服务、MoveIt/JTC 客户端、预览和执行主逻辑。
- [direct_move_validation.hpp](include/pi_arm_manager/direct_move_validation.hpp)：关节顺序、限位、目标完整性及轨迹校验。
- [arrival_check.hpp](include/pi_arm_manager/arrival_check.hpp)：目标误差、速度、新鲜度与稳定到位判断。
- [management_gate.hpp](include/pi_arm_manager/management_gate.hpp)、[motion_stop_reason.hpp](include/pi_arm_manager/motion_stop_reason.hpp)：管理互斥与退出原因。
- [manager.launch.py](launch/manager.launch.py)：节点入口，正常运行由 bringup 统一提供模型与规划参数。

对外动作与服务见 [interfaces](../pi_arm_interfaces/README.md)。`/pi_arm/preview_motion` 复用生产模型：joint/output 是 DirectMove 插值；ik 调用 `/compute_ik` 检查端点；lin 仅规划 Pilz 路径。预览不使能、不创建执行租约，执行前仍从最新起点重新生成。

故障恢复在标定无效时可依据原始电机反馈开放，但仍要求静止、无活动任务及可确认停止。`/pi_arm/hardware/*` 是内部管理边界，不是另一套用户运动入口。

## 验证

本包 `test/` 覆盖到位、目标、状态聚合、管理门禁和停止原因。`scripts/test_arrival_integration.py` 在隔离 ROS 域 82 用合成反馈验证到位、保持复核、运动停止、动作取消、迟到响应、保持失败、硬件故障优先级和等待时重复停止；完整链路使用 mock/vcan，见 [scripts](../../scripts/README.md)。实机机械精度与保持力矩需独立测量。
