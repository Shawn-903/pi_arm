# pi_arm_hardware

ros2_control 的 `SystemInterface` 插件包，通过六轴 Backend 将关节命令映射到电机输出轴/CAN，并将编码器反馈反解为关节状态。正常控制、只读监测和 commissioning 共用这里的模型与诊断逻辑。

## 模块

| 文件 | 职责 |
| --- | --- |
| [pi_arm_system.cpp](src/pi_arm_system.cpp) | 插件生命周期、position/velocity 接口、硬件状态发布和管理服务 |
| [backend.cpp](src/backend.cpp) | RealBackend、实机反馈有效性、租约、停止确认、保持与拖拽示教 |
| [mock_backend.cpp](src/mock_backend.cpp) | MockBackend，提供确定性的命令反馈和模拟管理操作 |
| [backend_common.hpp](src/backend_common.hpp) | 两种后端共用的配置校验、速度限幅和关节与电机依赖关系 |
| [actuator_coordinates.cpp](src/actuator_coordinates.cpp) | 协议度、减速比、安装方向与输出轴弧度转换 |
| [calibration.cpp](src/calibration.cpp) | 加载接受的参考、核验配置指纹、计算软件零偏 |
| [software_zero.cpp](src/software_zero.cpp)、[zero_reference.cpp](src/zero_reference.cpp) | 静止采样、计算整组六轴零偏，并保存和加载软件参考 |
| [axis_test.cpp](src/axis_test.cpp) | 独立电机/关节方向测试及心跳、行程、编码后路径检查 |
| [monitor_node.cpp](src/monitor_node.cpp) | `pi_arm_hardware_monitor_node`，只读或 commissioning 入口 |
| [hardware_state_message.cpp](src/hardware_state_message.cpp) | 正常控制、只读和 commissioning 共用的硬件状态消息转换 |
| [motion_write_gate.hpp](include/pi_arm_hardware/motion_write_gate.hpp) | 新控制器命令与当前运动租约匹配，隔离旧目标 |
| [motion_detector.hpp](include/pi_arm_hardware/motion_detector.hpp) | 基于新鲜编码器角度和持续速度的反馈运动判定、防抖与稳定期 |

配置、R10 参考与零点流程见 [config/README.md](config/README.md)。插件描述为 [pi_arm_hardware_plugin.xml](pi_arm_hardware_plugin.xml)，由 [bringup](../pi_arm_bringup/README.md) 加载。控制器通过 position/velocity 接口提交关节目标，后端转换后发送给 CAN 驱动；反馈沿相反方向换算并发布。

## 关键行为

- 正常运动要求反馈完整新鲜、标定有效且取得控制许可。缺少反馈或标定无效时仍发布已有原始数据和诊断原因。位置校验包括输出轴范围和反解后的关节限位。
- J2/J3 的反馈与起点容差为 3°；下发目标坚持标准限位。起点落在容差带时，生成命令会投影到最近标准边界。
- A4 是带速度上限的位置命令，速度字段按协议度/秒向下取整。普通运动的时序由控制器采样位置决定，速度字段用于约束电机跟随速度。
- 成功到位后保留最后 A4 位置目标。正常取消/网页停止关闭轨迹写入、清空旧指令，再用新鲜协议角下发 A4 保持。故障、超时、下使能、进入示教和服务退出按各自的停止策略处理。
- 进入示教通过 0x81 释放位置闭环；退出时捕获新鲜、静止的协议角，以 1°/s 输出轴上限建立当前位置保持，并确认 A4 回包和反馈。
- 0x07 重启后核验反馈恢复及电机关闭状态。输出轴参考跳变超过 0.2° 或无法核验时，记录 `REBOOT_REFERENCE_UNVERIFIED` 并要求人工核对姿态和参考。该故障标志保存在当前进程内。
- commissioning 的电机测试用于观察协议方向，关节测试用于检查绳索映射及耦合电机运动；两者按各自的行程和输出范围执行。

`/pi_arm/hardware_state` 发布反馈与诊断；`/pi_arm/hardware/*` 管理服务供 manager 调用。只读模式发布监测数据，commissioning 通过 `/pi_arm/commissioning/*` 提供维护服务。

生命周期分为“停用后等待再次激活”和“退出并释放资源”两种情况：

| 阶段 | 处理顺序与结果 |
| --- | --- |
| 停用（deactivate） | 撤销运动许可，保留 CAN 反馈读取，等待再次激活 |
| 再次激活（activate） | 恢复租约申请、位置保持和示教服务，后续运动申请新租约 |
| cleanup / shutdown / error | 先结束服务线程，再停止运动，随后关闭 CAN 线程和 socket，最后释放接口独占锁 |

插件的激活、停用与管理服务串行处理。停止应答缺失时，日志记录 `STOP UNCONFIRMED`，供操作者检查通信和停止结果。

## 停止并保持

内部 `SetMotionArmed(armed=false, hold_position=true, lease_id=0)` 表示取消轨迹并保持当前编码器位置；非零 lease 的 hold 表示正常到位后保留最终目标。停止目标直接取自原始协议位置，角度/速度反馈的有效期为 100 ms，状态反馈沿用新鲜度配置。保持请求作用于已使能电机。

保持目标的输出轴速度上限为 1°/s，用于修正采样及通信期间的微小位置差。驱动确认 A4 应答、新角度/状态、静止和位置误差后返回成功。重复停止保留同一目标；新的运动任务取得新租约后接管位置目标。

保持未确认时返回 `HOLD UNCONFIRMED`、设置 `motion_stop_failed`，并转入故障停止。处理通信或硬件故障后重试停止，确认结果后再发起运动。全部电机已下使能或处于拖拽示教时，停止请求保持原有释放状态。commissioning 的操作员停止和心跳超时会保持正在测试的电机；异常与退出使用故障停止，自动到达终点则执行测试结束流程。

## 反馈运动状态

实机的 `motor_moving` / `moving` 由各输出轴、各关节独立的 `MotionDetector` 判定，正常控制和 monitor 共用。原始协议角、关节位置和速度按实际计算值发布，运动标志单独进行时间防抖；mock 使用理想命令反馈。

- 速度阈值取 URDF `moving_speed_threshold_rad_s`，默认 0.01°/s。超过阈值的同方向速度持续 150 ms 后形成运动证据。
- 同时检查最近 500 ms 的编码器角度跨度，去掉一个最高/最低采样以忽略单点异常。跨度超过 `max(两协议角度刻度折算量, 速度阈值 × 实际时间窗)` 且证据持续 40 ms，可识别低速移动和往返振动。
- 协议角度刻度为 0.01°；两刻度在 36/6 减速比下对应输出轴约 0.000556°/0.003333°。各关节位移容差经当前位置的绳索逆雅可比计算，计入耦合、方向与腕部交换。
- 最后确认运动后需 300 ms 新鲜稳定反馈才退出运动；考虑位置窗口残留，停止显示通常有约 0.3～0.8 s 延迟。时间常量在上述头文件，只有速度阈值是现有运行配置参数。
- 每次新 CAN 回包推进一次检测。缺失/非有限反馈、时间倒退或超过 250 ms 的采样间断会重建检测器，并重新积累稳定反馈。使用运动标志时同时检查 `fresh`、连接、标定和运动学状态。

manager 有活动任务时发布 RUNNING；空闲时由反馈运动标志决定 READY/RUNNING。网页状态和重复定位的到点等待使用同一组运动标志，后端另行核验到位误差与稳定时间。

## 验证

`test/test_system_configuration.cpp` 覆盖插件配置入口；`test/test_backend.cpp`、`test/test_motion_write_gate.cpp` 覆盖后端与租约；`test/test_motion_detector.cpp` 覆盖静止噪声、低速、持续速度、稳定期与缓存/失效反馈。`test/vcan_backend_probe.cpp`、`test/vcan_reboot_probe.cpp`、`test/vcan_motion_probe.cpp` 由 [vcan 脚本](../../scripts/README.md) 编译/调用，在虚拟 CAN 上验证生产 RealBackend。

[test_backend_lifecycle_vcan.py](../../scripts/test_backend_lifecycle_vcan.py) 验证停用后的租约门禁、重新激活、插件 shutdown/error/cleanup、停止应答缺失及重复清理。启用 `BUILD_TESTING` 构建后，使用空闲的虚拟 `vcan0` 和 ROS 域 100 运行：`ROS_DOMAIN_ID=100 python3 scripts/test_backend_lifecycle_vcan.py --probe build/pi_arm_hardware/vcan_lifecycle_probe --log-dir /tmp/backend-lifecycle`。探针在生命周期回调后保留插件对象，检查 CAN socket 与独占锁是否已释放。
