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

配置、R10 参考与零点流程见 [config/README.md](config/README.md)。插件描述为 [pi_arm_hardware_plugin.xml](pi_arm_hardware_plugin.xml)，正常由 [bringup](../pi_arm_bringup/README.md) 加载，不另开同一 CAN 驱动。

## 关键行为

- 反馈必须完整新鲜；标定通过后才进行受许可的正常控制。输出轴包络检查不能替代关节限位，反解后仍逐关节校验。
- J2/J3 的反馈与起点容差为 3°；下发目标坚持标准限位。起点落在容差带时，生成命令会投影到最近标准边界。
- A4 协议速度是整数上限，向下取整，不能提高低速请求。普通运动使用控制器采样位置推进，不能用过小的整数前馈速度造成持续落后。
- 成功到位后保留最后 A4 位置闭环。异常、取消、显式停止清除待发位置指令，发送 0x81 并确认；停止未确认时保持运动禁止。
- 进入示教使用 0x81，退出时捕获新鲜静止协议角，以 1°/s 输出轴上限建立当前位置保持，确认 A4 回包和反馈；不使用 0x88 恢复旧目标。
- 0x07 重启后确认反馈恢复且保持关闭；参考跳变超过 0.2° 输出轴角或无法核验时锁定 `REBOOT_REFERENCE_UNVERIFIED`。该锁定不跨进程持久化，重启服务不能证明参考已修复。
- commissioning 的关节测试受耦合电机实际编码与输出范围约束；协议方向电机测试是独立诊断模式，不能据此认定正常关节运动学有效。

`/pi_arm/hardware_state` 发布原始数据与原因；`/pi_arm/hardware/*` 管理服务供 manager 调用。只读模式不暴露写入服务；commissioning 服务在 `/pi_arm/commissioning/*`。

## 验证

`test/test_backend.cpp`、`test/test_motion_write_gate.cpp` 覆盖后端与租约。`test/vcan_backend_probe.cpp`、`test/vcan_reboot_probe.cpp` 由 [vcan 脚本](../../scripts/README.md) 编译/调用，验证生产 RealBackend；不应将探针接到实机 CAN。
