# pi_arm_can

SocketCAN 传输与 LK-TECH 电机协议库，供硬件后端和命令行工具调用。它将电机目标编码为 CAN 帧，并把回包解析为电机反馈。

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
| 0x9B | 清除错误标志 |
| 0x07 | 电机重启，无 ACK，必须等待后续轮询确认状态 |
| 0x19 | 将当前位置写为 ROM 零点，重新上电后核验 |

完整依据见 [原始 CAN 协议](../../docs/CAN总线通讯协议V2.6.pdf)。速度字段以协议度/秒表示并向下取整，输出轴速度经减速比和安装方向换算后写入该字段。换算方法见 [硬件坐标层](../pi_arm_hardware/config/README.md)。

在 C++ 中使用时，先用 `DriverConfig` 设置接口和电机 ID，再构造 `CanDriver` 并调用 `start()` 启动轮询。`snapshot()` 返回按电机 ID 索引的反馈缓存，角度、状态有效标志和更新时间用于判断数据是否可用；`stop()` 结束驱动线程并关闭连接。上层硬件后端通过驱动的控制模式、位置队列及管理接口发送命令，调用示例见 [backend.cpp](../pi_arm_hardware/src/backend.cpp)。

## 连接与恢复

接口尚未出现或 SocketCAN 收发失败时，驱动记录连接状态和错误，每秒尝试重新连接；上层硬件节点继续发布诊断。接口恢复后重新读取电机反馈，恢复期间保持对该 CAN 接口的独占使用。

通信故障会丢弃待发指令，使旧运动授权失效。恢复后先检查新反馈；若故障发生时正在控制电机或停止尚未确认，需要再次执行“停止”，确认停止后再发起新任务。

## ROM 工具与独占

执行 `ros2 run pi_arm_can pi_arm_rom_zero_cli --help` 查看参数。工具默认以 dry-run 模式核验六轴反馈和静止状态；指定 `--execute` 后执行 ROM 写零，真实 CAN 上还需交互确认。使用前先停止占用该接口的控制服务。

ROM 写零后完整断电、上电，再核对反馈和姿态并生成新参考。操作步骤见 [配置 README](../pi_arm_hardware/config/README.md)。

## 验证

`test/test_pi_arm_can.cpp` 覆盖帧编码与解析；`test/test_vcan_motor_emulator.py` 检查模拟器协议。虚拟总线停止、重启与 ROM 场景见 [scripts/README.md](../../scripts/README.md)。
