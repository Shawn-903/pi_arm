# pi_arm_hardware

ros2_control 的 `SystemInterface` 插件包，通过六轴 Backend 将关节命令映射到电机输出轴/CAN，并将编码器反馈反解为关节状态。正常控制、只读监测和 commissioning 共用这里的模型与诊断逻辑。

## 模块

| 文件 | 职责 |
| --- | --- |
| [pi_arm_system.cpp](src/pi_arm_system.cpp) | 插件生命周期、position/velocity 接口、硬件状态发布和管理服务 |
| [backend.cpp](src/backend.cpp) | Real/Mock Backend、反馈有效性、租约、停止确认、保持与拖拽示教 |
| [actuator_coordinates.cpp](src/actuator_coordinates.cpp) | 协议度、减速比、安装方向与输出轴弧度转换 |
| [calibration.cpp](src/calibration.cpp) | 加载接受的参考、核验配置指纹、计算软件零偏 |
| [axis_test.cpp](src/axis_test.cpp) | 独立电机/关节方向测试及心跳、行程、编码后路径检查 |
| [monitor_node.cpp](src/monitor_node.cpp) | `pi_arm_hardware_monitor_node`，只读或 commissioning 入口 |
| [motion_write_gate.hpp](include/pi_arm_hardware/motion_write_gate.hpp) | 新控制器命令与当前运动租约匹配，隔离旧目标 |
| [motion_detector.hpp](include/pi_arm_hardware/motion_detector.hpp) | 基于新鲜编码器角度和持续速度的反馈运动判定、防抖与稳定期 |

配置、R10 参考与零点流程见 [config/README.md](config/README.md)。插件描述为 [pi_arm_hardware_plugin.xml](pi_arm_hardware_plugin.xml)，正常由 [bringup](../pi_arm_bringup/README.md) 加载，不另开同一 CAN 驱动。

## 关键行为

- 反馈必须完整新鲜；标定通过后才进行受许可的正常控制。输出轴包络检查不能替代关节限位，反解后仍逐关节校验。
- J2/J3 的反馈与起点容差为 3°；下发目标坚持标准限位。起点落在容差带时，生成命令会投影到最近标准边界。
- A4 协议速度是整数上限，向下取整，不能提高低速请求。普通运动使用控制器采样位置推进，不能用过小的整数前馈速度造成持续落后。
- 成功到位后保留最后 A4 位置闭环。正常取消/网页停止关闭轨迹写入、清空旧指令，使用新鲜原始协议角直接下发 A4 保持，不先发送释放位置环的 0x81。故障、超时、下使能、进入示教和服务退出仍使用原有故障/释放路径。
- 进入示教使用 0x81，退出时捕获新鲜静止协议角，以 1°/s 输出轴上限建立当前位置保持，确认 A4 回包和反馈；不使用 0x88 恢复旧目标。
- 0x07 重启后确认反馈恢复且保持关闭；参考跳变超过 0.2° 输出轴角或无法核验时锁定 `REBOOT_REFERENCE_UNVERIFIED`。该锁定不跨进程持久化，重启服务不能证明参考已修复。
- commissioning 的关节测试受耦合电机实际编码与输出范围约束；协议方向电机测试是独立诊断模式，不能据此认定正常关节运动学有效。

`/pi_arm/hardware_state` 发布原始数据与原因；`/pi_arm/hardware/*` 管理服务供 manager 调用。只读模式不暴露写入服务；commissioning 服务在 `/pi_arm/commissioning/*`。

## 停止并保持

内部 `SetMotionArmed(armed=false, hold_position=true, lease_id=0)` 表示取消轨迹并保持当前编码器位置；非零 lease 的 hold 仍表示正常到位后保留最终目标。停止抓取的是原始协议位置，不需要用关节反解重新构造目标。角度/速度反馈不得超过 100 ms，状态反馈沿用新鲜度配置；不对已下使能电机发送 A4，也不以 0x88 恢复旧目标。

保持目标的输出轴速度上限为 1°/s，用于修正采样及通信期间的微小位置差；这是停止保持的捕获速度，不修改正常运动速度设置。驱动确认 A4 应答、新角度/状态、静止和位置误差后才返回成功。重复停止保留同一目标，旧 action 的析构清理或残留控制器命令不能释放/覆盖保持；新运动需要新租约。

保持未确认会返回 `HOLD UNCONFIRMED`，置 `motion_stop_failed`、禁止新运动，并退回可确认的故障停止。恢复反馈/故障后重试停止；不能宣称断线或电机故障时仍有保持力矩。全部电机已下使能时停止保持关闭状态，拖拽示教中停止也不会自动重新上力。commissioning 操作员停止和心跳停止保持正在测试的电机，异常/退出保持故障停止逻辑；单轴测试自动到达终点仍沿用原有测试结束流程。

## 反馈运动状态

实机的 `motor_moving` / `moving` 由各输出轴、各关节独立的 `MotionDetector` 判定，正常控制和 monitor 共用。原始协议角、关节位置、速度反馈不做截断或滤波；mock 保留理想命令反馈。

- 速度阈值仍取 URDF `moving_speed_threshold_rad_s`，默认 0.01°/s。超过阈值的同方向速度需持续 150 ms，孤立速度尖峰不会单独触发运动。
- 同时检查最近 500 ms 的编码器角度跨度，去掉一个最高/最低采样以忽略单点异常。跨度超过 `max(两协议角度刻度折算量, 速度阈值 × 实际时间窗)` 且证据持续 40 ms，即使协议速度为零，也可识别真实缓慢移动；往返振动不会因窗口首尾位置相同而漏判。
- 协议角度刻度为 0.01°；两刻度在 36/6 减速比下对应输出轴约 0.000556°/0.003333°。关节位移容差经当前位置的绳索逆雅可比传播，计入耦合、方向与腕部交换，不能统一把六轴死区设为一个大角度。
- 最后确认运动后需 300 ms 新鲜稳定反馈才退出运动；考虑位置窗口残留，停止显示通常有约 0.3～0.8 s 延迟。时间常量在上述头文件，只有速度阈值是现有运行配置参数。
- 用 CAN 回包时间推进检测，重复读取缓存不能累计证据。缺失/非有限反馈、时间倒退或超过 250 ms 的采样间断会重建检测器，重新积累稳定反馈前按“尚不能确认静止”处理。`fresh`、连接、标定、运动学诊断仍独立生效，调用方必须一起检查，不能仅凭 moving 判断故障。

manager 的活动任务仍立即产生 RUNNING；空闲时才由反馈运动标志决定 READY/RUNNING。网页与重复定位到点等待使用这些标志，无需在前端另加隐藏运动的延迟。目标误差/原始速度到位核验、底层停止确认、标零/重启/示教保持的静止门禁均未放宽。

## 验证

`test/test_backend.cpp`、`test/test_motion_write_gate.cpp` 覆盖后端与租约；`test/test_motion_detector.cpp` 覆盖静止噪声、低速、持续速度、稳定期与缓存/失效反馈。`test/vcan_backend_probe.cpp`、`test/vcan_reboot_probe.cpp`、`test/vcan_motion_probe.cpp` 由 [vcan 脚本](../../scripts/README.md) 编译/调用，验证生产 RealBackend；不应将探针接到实机 CAN。
