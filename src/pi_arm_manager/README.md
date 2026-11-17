# pi_arm_manager

正常控制模式的任务与运动管理节点 `pi_arm_manager_node`。它接收上层动作/服务，检查当前状态，规划或生成轨迹，再交给关节轨迹控制器（JointTrajectoryController，简称 JTC）执行。manager 根据硬件反馈独立核验到位结果。

## 控制流程

1. 读取 `/pi_arm/hardware_state`，通过 [state_aggregator.hpp](include/pi_arm_manager/state_aggregator.hpp) 发布 `/pi_arm/state`。
2. 互斥接收 DirectMove / MoveJ / MoveJs / MoveL；起点可使用反馈容差，目标使用标准关节限位和输出轴限位。
3. `prepare_trajectory` 根据绳索雅可比矩阵（关节速度到输出轴速度的映射）、协议速度分辨率与关节速度上限延长轨迹时间。计算采用雅可比绝对值的保守上界；耗时最长的输出轴决定整段同步运动时间。
4. 获取本次位置写入租约，通过 `/joint_trajectory_controller/follow_joint_trajectory` 执行；用户取消/停止先关闭硬件轨迹写入并确认当前位置保持，再取消控制器目标。
5. JTC 成功后检查新鲜反馈、六轴目标误差及静止，满足稳定时间后以位置保持方式释放租约，再复核并返回到位结果。

默认到位容差 0.2°、稳定时间 0.5 s、收敛超时 10 s，配置在 [manager.yaml](../pi_arm_bringup/config/manager.yaml)，与 [controllers.yaml](../pi_arm_bringup/config/controllers.yaml) 配合使用。普通任务 `motion_timeout_sec` 默认 120 s，网页等待和后端执行分别计时。

`StopMotion` 在运动中、到点等待和空闲时都请求保持当前位置，适用于关节、输出轴、预设姿态及笛卡尔运动。取消、结果回调和任务清理共用停止策略，停止处理期间暂停接收新任务。迟到的控制器目标接受回包也会触发取消。保持失败返回 3006；硬件故障、断线或失能时，优先执行故障停止并报告原因。

收到 SIGINT/SIGTERM 后，manager 按以下顺序退出：

1. 拒绝新任务；有活动动作或待完成的清理时，以故障停止方式释放运动租约。
2. 取消规划/控制器目标，保持 ROS 通信可用，等待工作线程和取消响应完成。
3. 关闭 ROS，再销毁 MoveIt 资源。

退出时，活动动作返回失败码 3104；已有硬件故障保留更高优先级。预览退出会取消正在进行的规划。若停止或取消未获确认，进程返回非零退出码，并在日志中记录原因。

## 接口与文件

- [manager_node.cpp](src/manager_node.cpp)：动作、管理/停止服务、MoveIt/JTC 客户端、预览和执行主逻辑。
- [trajectory_preparation.cpp](src/trajectory_preparation.cpp)：轨迹时间校验及关节/输出轴限速计算，使用节点取得的起点快照和模型限位；锁、租约和控制器提交仍由节点管理。
- [direct_move_validation.hpp](include/pi_arm_manager/direct_move_validation.hpp)：关节顺序、限位、目标完整性及轨迹校验。
- [arrival_check.hpp](include/pi_arm_manager/arrival_check.hpp)：目标误差、速度、新鲜度与稳定到位判断。
- [management_gate.hpp](include/pi_arm_manager/management_gate.hpp)、[motion_stop_reason.hpp](include/pi_arm_manager/motion_stop_reason.hpp)：管理互斥与退出原因。
- [manager.launch.py](launch/manager.launch.py)：节点入口，正常运行由 bringup 统一提供模型与规划参数。

上层程序通过 [interfaces](../pi_arm_interfaces/README.md) 中的动作提交运动目标，跟踪任务反馈和结果；通过 `StopMotion` 服务请求停止。`/pi_arm/preview_motion` 返回计算结果供显示：joint/output 使用 DirectMove 插值，ik 调用 `/compute_ik` 求端点解，lin 规划 Pilz 路径。正式执行时从最新反馈起点重新生成轨迹。

清错服务按所选电机检查新鲜、静止反馈，其他电机暂时失联时仍可对在线电机清错；活动任务、已知运动或停止未确认时会拒绝请求。使能、失能和重启继续检查整臂反馈与静止状态。`/pi_arm/hardware/*` 是供 manager 使用的内部管理接口。

## 验证

本包 `test/` 覆盖到位、目标、状态聚合、管理门禁和停止原因。`scripts/test_arrival_integration.py` 在隔离 ROS 域 82 用合成反馈验证到位、保持复核、运动停止、动作取消、迟到响应、保持失败、硬件故障优先级和等待时重复停止；完整链路使用 mock/vcan，见 [scripts](../../scripts/README.md)。

[test_manager_shutdown.py](../../scripts/test_manager_shutdown.py) 在隔离 ROS 域 98 检查运动中退出、租约申请中退出、迟到目标、规划/预览中退出、停止确认失败和空闲退出。先加载待测工作区，例如：`ROS_DOMAIN_ID=98 python3 scripts/test_manager_shutdown.py --case active --log-dir /tmp/manager-shutdown`。
