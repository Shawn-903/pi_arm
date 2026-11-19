# 标定姿态采样控制

按姿态清单依次移动机械臂，等待 manager 确认到位，保持指定时间后保存一帧反馈。采样点名称、任务号和时间用于与动捕、激光跟踪仪等外部测量记录对应。

任务使用 `/pi_arm/direct_move` 执行关节插值。运行前启动正常控制模式，使能电机并等待 READY；采样姿态和沿途空间由操作者按实验需要确定。

## 姿态清单

JSON 文件保存一个列表，每项包含名称、主动关节角和停留时间。角度单位为度，顺序为 `joint_01, joint_02, joint_04, joint_05, joint_06, joint_09`。名称在本次清单内唯一，`dwell_sec` 默认 3 秒，可设为 0。

```json
[
  {"name": "P001", "joint_positions_deg": [0, 0, 0, 0, 0, 0], "dwell_sec": 3.0},
  {"name": "P002", "joint_positions_deg": [5, 0, 0, 0, 0, 0], "dwell_sec": 3.0}
]
```

上例随包安装为 `examples/calibration_poses.json`，可先在 mock 中体验。现场使用时将实验姿态保存到自己的清单文件。

## 命令行运行

在一个终端加载 ROS 和工作区环境，启动模拟控制服务：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=81 bash scripts/start_web.sh hardware_type:=mock use_rviz:=false
```

连接网页并点击“所有电机使能”。另开终端，在相同 ROS 域运行任务：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=81
tasks_share="$(ros2 pkg prefix --share pi_arm_tasks)"
ros2 run pi_arm_tasks pi_arm_calibration_motion \
  --plan "$tasks_share/examples/calibration_poses.json" \
  --output /tmp/calibration-samples.jsonl
```

`--speed-deg-s` 设置关节速度，`--output-speed-deg-s` 设置输出轴速度上限，两者默认 5°/s。`--motion-timeout-sec` 设置每个姿态的运动等待上限，默认 180 秒。结果文件使用新路径，已有文件会保留。

每个点按“运动 → 到位核验 → 停留 → 记录”执行。全部完成后保持最后位置，命令返回采样点数和结果文件路径。`Ctrl+C` 或 SIGTERM 会结束后续采样并请求停止；运动失败、状态过期、参考变化或停留期间姿态变化也会结束任务。失败结果包含原因及停止确认情况，已完成的采样点保留在文件中。

## Python 调用

`CalibrationPose` 的角度使用弧度。`CalibrationMotionTask.run()` 在调用线程中处理 ROS 消息，返回保存的采样点数；其他线程可调用 `request_stop()`。一个实例一次执行一组清单。

```python
import math
import rclpy
from rclpy.signals import SignalHandlerOptions
from pi_arm_tasks.calibration_motion import CalibrationMotionTask, CalibrationPose

rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
task = CalibrationMotionTask()
try:
    poses = [
        CalibrationPose("P001", [0.0] * 6, dwell_sec=3.0),
        CalibrationPose("P002", [math.radians(5), 0, 0, 0, 0, 0], dwell_sec=3.0),
    ]
    count = task.run(poses, "/tmp/my-calibration-samples.jsonl")
    print(count)
finally:
    task.close()
    rclpy.try_shutdown()
```

示例保留 Python 的中断处理，使任务能在 `KeyboardInterrupt` 后完成停止通信。自行管理进程信号时，将结束请求交给 `task.request_stop()`；命令行入口已处理 SIGINT 和 SIGTERM。

## 结果文件

JSONL 每行一条 JSON 记录：

- `run`：姿态清单、速度、超时和开始时间。
- `sample`：点名称、序号、任务号、UTC 与 ROS 时间、目标/反馈关节角、原始协议角、模型输出轴角，以及当前标定信息。
- `result`：本组成功状态、已保存点数；失败时记录原因和 `stop_confirmed`。

关节数据按 `joint_names` 排列，电机数据按 `motor_ids` 排列。反馈记录为停留结束时的一帧；外部测量按点名称和时间与之关联。

返回 [任务索引](../../README.md)，运动行为见 [manager](../../../pi_arm_manager/README.md)。
