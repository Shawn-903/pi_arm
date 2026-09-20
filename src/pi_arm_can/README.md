# pi_arm_can

SocketCAN 传输与 LK-TECH 电机协议库，不依赖 ROS 节点。上层硬件包负责标定和关节语义，本包只处理电机协议坐标与总线状态。

## 结构与协议

- [can_driver.hpp](include/pi_arm_can/can_driver.hpp) / [can_driver.cpp](src/can_driver.cpp)：收发线程、轮询、反馈缓存、指令队列、停止/保持及接口占用管理。
- [protocol.hpp](include/pi_arm_can/protocol.hpp) / [protocol.cpp](src/protocol.cpp)：小端编码、带符号多圈角、命令帧与整数速度。
- [rom_zero_cli.cpp](src/rom_zero_cli.cpp)：显式 ROM 置零工具 `pi_arm_rom_zero_cli`，默认 dry-run，记录 JSONL 审计。

CAN ID 为 `0x140 + motor_id`，六轴 ID 1–6、DLC 8。主要命令：

| 命令 | 含义 |
| --- | --- |
| 0x92 / 0x9A / 0x9C | 多圈角、状态 1、状态 2 |
| 0xA4 | 带速度上限的位置控制，协议位置单位为度 |
| 0x81 | 停止，保留运行状态/角度参考；确认停止应答 |
| 0x80 / 0x88 | 关闭 / 运行；0x80 会清除转动圈数，0x88 可能恢复旧目标 |
| 0x9B | 清错误标志，不等于重启 |
| 0x07 | 电机重启，无 ACK，必须等待后续轮询确认状态 |
| 0x19 | 将当前位置写为 ROM 零点，重新上电后核验 |

完整依据见 [原始 CAN 协议](../../docs/CAN总线通讯协议V2.6.pdf)。速度字段以协议度/秒表示、向下取整；不能把 UI 的输出轴速度直接填入协议字段。比值和方向由 [硬件坐标层](../pi_arm_hardware/config/README.md) 转换。

## ROM 工具与独占

`ros2 run pi_arm_can pi_arm_rom_zero_cli --help` 查看参数。默认仅核验六轴反馈和静止、不写 0x19；`--execute` 才请求实际写入，真实 CAN 不允许 `--yes` 跳过交互确认。同一接口不得同时运行控制服务和该工具。

标零后保留人工完整断电/上电流程，再确认反馈与姿态、生成新参考；不能仅凭六个 ACK 认定零点生效。标定步骤见 [配置 README](../pi_arm_hardware/config/README.md)。

## 验证

`test/test_pi_arm_can.cpp` 覆盖帧编码与解析；`test/test_vcan_motor_emulator.py` 检查模拟器协议。虚拟总线停止、重启与 ROM 场景见 [scripts/README.md](../../scripts/README.md)。这些结果不等价于实机停止距离、启动时间或零点保留验证。
