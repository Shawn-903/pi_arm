# vcan 修复及验证记录

针对 `17af2f8` 的四项审查发现，已完成以下修复。实物方向、减速比和 R02 标定数据未作修改。

| 审查问题 | 修复后的行为 |
| --- | --- |
| 普通控制只撤销后续写入 | 撤销租约与最终 A4 提交串行化，清除待发位置指令后发送六电机 `0x81`。故障检测只发起停止，不在控制循环等待 ACK；服务调用等待确认。显式无租约停止也发送停止帧，覆盖进程重启后未知的旧目标。 |
| 反馈缺少关节限位 | 逆解后逐关节检查 URDF 限位，报告 `JOINT_POSITION_LIMIT`、关节名称、值和边界，保留原始数据；禁止普通运动并停止正在运行的关节测试。 |
| 低速编码提高速度、耦合路径不覆盖实际过程 | 协议速度向下取整，低于 1 协议度/秒明确拒绝；对编码后的目标检查 64 个输出范围顶点，覆盖电机不同步但单调到位的组合。运行时越出已验证范围即停止，超时按编码速度重新计算。弦长模型的选定关节速度同时按最小 Jacobian 增益限制。 |
| 动作已接受但网关返回失败后继续运行 | 缺少任务 ID 反馈时请求取消，并等待动作终态；未确认时明确报错并保留跟踪。接受响应超时后注册回调，取消晚到的已接受目标，回调不依赖原 WebSocket 协程仍然存活。 |

停止超过 500 ms 未获全体所选电机应答时，驱动保持失败状态，禁止再次进入 CONTROL 或提交 A4。网页显示“停止未确认”，允许重试停止。普通 manager 的停止服务不再无条件返回成功；等待正常停止确认期间不会把正在收尾的任务误判为硬件禁用故障。正常硬件状态消息也补齐 CAN 接口、协议比值和方向字段。

`0x81` 的应答代表协议请求获得确认；真实机械的停止距离及驱动器实际响应仍需现场验证。异步路径检查基于当前绳索模型和输出轴范围，不能验证实物绳索方向是否与配置一致。

## 验证方式

- 完整工作区构建：10 个包成功。
- `colcon test`：139 项测试，0 errors / 0 failures / 0 skipped，包括新增速度编码、停止中状态聚合和网关超时回归。
- `scripts/test_motion_safety_vcan.py`：12 个生产 RealBackend 场景。包括显式闲置停止、错误租约、并发写入停止、丢失一轴 ACK 后锁定/重试、控制循环故障停止、原始反馈丢失、不可表示速度拒绝、真实整数速度下的独立电机运动、限位附近耦合请求拒绝、运行中路径偏离/关节越界、commissioning 停止失败及心跳丢失。
- 虚拟 CAN + ROS commissioning：电机单轴只下发选定 CAN ID；关节单轴下发必要的耦合电机；均收到停止应答。
- 浏览器：单轴指令、动作前后记录、观察记录与 JSON 导出、心跳续期、手动停止和断线停止通过。离线浏览器验证新增关节越界详情、停止重试入口及最小速度提示。
- 普通完整链路：WebSocket → manager → JTC → RealBackend → vcan。DirectMove 到位后任务成功并发送六轴停止；运动中 `stop_motion` 返回成功，任务终止，抓帧确认 STOP 后没有新的 A4 请求。

所有运动命令仅发送到 `vcan0`，测试 ROS 图使用独立 domain 74 / 75；`can0` 的既有实机服务未重启或切换版本。本轮没有向真实电机发送运动、使能或 ROM 写入请求。

普通链路首次测试遇到“硬件 READY 先于 JTC 激活”的启动时序，控制器明确拒绝了过早的目标。等待控制器激活后重新验证通过。测试退出时还观察到安装版 MoveIt `move_group` 在 SIGINT 清理阶段出现段错误；发生在停止测试进程之后，未纳入本次四项业务缺陷修复，应另行调查其退出行为。

## 复现入口

先构建并 source 环境，在没有其他驱动/模拟器占用的已启用 `vcan0` 上运行：

```bash
bash scripts/build.sh
source install/setup.bash
python3 scripts/test_motion_safety_vcan.py
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

完整 ROS 链路及浏览器测试需要按各脚本说明启动测试服务：

```bash
python3 scripts/test_axis_vcan.py
python3 scripts/test_axis_vcan.py --joint
python3 scripts/test_web_axis.py --browser-path /path/to/chromium
python3 scripts/test_web_axis.py --stall --browser-path /path/to/chromium
python3 scripts/test_normal_motion_vcan.py --url ws://127.0.0.1:8766
python3 scripts/test_web_diagnostics.py --browser-path /path/to/chromium
```

其中 commissioning 和普通 bringup 必须分开运行，不能同时占用 vcan0；`--stall` 使用 `vcan_motor_emulator.py --freeze-motion`。普通链路测试须等待 `joint_trajectory_controller` 激活。虚拟标定由测试生成，不能用于实机。
