# pi_arm_tasks

机械臂应用任务集合。每个任务放在独立子包中，通过已有 ROS 服务和动作组合控制流程，并提供具体的 Python 或命令行入口。

| 任务 | 用途 | 调用入口 |
| --- | --- | --- |
| [execution](../../docs/execution.md) | 独占控制会话、步骤程序、暂停停止和持久运行记录 | `pi_arm_execution`、网页、[Python SDK](../../sdk/python/README.md) |
| [zero_reference](pi_arm_tasks/zero_reference/README.md) | 采集已知姿态的编码器读数，计算并保存软件零点 | `scripts/zero_reference.sh` 一键流程；网页、`ZeroReferenceClient`、`pi_arm_zero` 接入维护服务 |
| [calibration_motion](pi_arm_tasks/calibration_motion/README.md) | 按姿态清单逐点运动、等待到位、停留并记录反馈 | `CalibrationMotionTask`、`pi_arm_calibration_motion` |

`pi_arm_tasks/` 是 Python 包目录，具体任务的客户端、节点和命令行实现在各自子目录中。软件标零服务节点使用 `pi_arm_zero_reference` 启动，由一键脚本或维护模式的 launch 装配。

标定采样任务在正常控制模式下运行，读取 JSON 姿态清单，输出带采样点名称、任务号和时间的 JSONL 记录。软件标零在维护模式下采集参考；两项任务各自的启动步骤见上表。

新任务使用具体功能命名，并在子目录 README 中说明输入、运行条件、调用方法和输出。运动请求通过 [manager](../pi_arm_manager/README.md) 的动作接口执行，硬件采样和管理操作通过对应服务完成。
