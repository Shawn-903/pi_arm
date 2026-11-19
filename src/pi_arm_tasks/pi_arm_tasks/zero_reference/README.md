# 软件标零任务

在实物处于已知关节姿态时采集编码器读数，计算并保存软件零偏。可以用一条命令完成，也可以通过网页或 Python 调用。

## 一条命令标零

机械零位是打包姿态，J1 至 J6 依次为 `[0, 100.7, -85.6, 0, 0, 0]°`。树莓派完成构建、CAN 接口就绪后，停止占用 CAN 的服务，将机械臂放到打包姿态并保持静止。在仓库根目录运行以下命令。

```bash
bash scripts/zero_reference.sh --confirm-reference
```

脚本加载 ROS 和工作区环境，启动维护与标零服务，等待六台电机反馈稳定后采集一次，保存结果并关闭本次服务。`--confirm-reference` 表示你已确认实物处于给定姿态。

默认使用 `can0`。脚本把已知关节姿态、采集时间和此刻六台电机的编码器读数保存到 `~/.local/share/pi_arm/software-zero-can0.yaml`。文件名随 CAN 接口变化。

每台电机都收到六次新的静止角度反馈后，程序才保存零点。采集期间反馈中断时，原文件保留。

修改电机方向后，在已知姿态重新执行同一条标零命令。采集成功时，程序用当前模型与本次的六轴读数原子替换软件零点文件。

**文件丢失时**，把实物重新放到同一个已知姿态并保持静止，再执行同一条采集命令。沿用该姿态的关节角，根据本次编码器读数重新计算零偏。

采集其他已知姿态时，用 `--reference-deg` 按 J1 至 J6 顺序传入角度（°）。例如，实物处于六个关节角均为 0° 的零位时，运行以下命令。

```bash
bash scripts/zero_reference.sh --confirm-reference \
  --reference-deg 0 0 0 0 0 0
```

自定义绳索模型用 `--cable-kinematics-file` 指定。服务和静止反馈的等待时间由 `--startup-timeout` 设置，默认 15 秒。

运行日志输出到终端。成功时，最后一行 JSON 给出 `success`、`software_zero_file` 和 `cleanup_success`，命令退出码为 0。失败时会说明原因并清理本次服务；若返回 `result_unknown: true`，核对保存文件中的姿态、编码器读数和现场反馈后再操作。按 `Ctrl+C` 可以结束运行，已发出的采集请求会先等待本次结果，再退出。

已经加载 ROS 和工作区环境时，也可直接执行 `ros2 run pi_arm_tasks pi_arm_zero_once --confirm-reference`。完整参数见 `bash scripts/zero_reference.sh --help`。

## 启动与网页使用

将机械臂放到参考姿态并保持静止，停止正常控制服务后，在仓库根目录启动维护模式：

```bash
bash scripts/start_web.sh --commissioning
```

该入口会启动 `pi_arm_zero_reference` 服务节点。在网页“方向测试”中核对打包姿态的参考角，确认实物姿态和静止状态后点击“采集并保存软件零点”。成功后页面显示采集结果。

## 命令行与 Python

接入已运行的维护服务时，使用 `pi_arm_zero` 客户端或 `ZeroReferenceClient`。它们调用 `/pi_arm/tasks/set_zero_reference`，复用服务的配置和保存路径。

上面的网页启动命令已经包含所需服务。独立 ROS 程序也可以先启动 `pi_arm_bringup` 的 `monitor.launch.py`，设置 `commissioning:=true`，再在另一个已加载环境的终端执行 `ros2 run pi_arm_tasks pi_arm_zero_reference`。

客户端也默认在机械零位采集。实物处于打包姿态且静止时运行以下命令。

```bash
ros2 run pi_arm_tasks pi_arm_zero --confirm-reference
```

Python 的 `set_zero_reference()` 接受以弧度表示的关节角，返回 ROS future。以下示例在打包姿态采集。

```python
import math

import rclpy
from pi_arm_tasks.zero_reference import ZeroReferenceClient
from pi_arm_websocket.presets import PACKING_JOINTS_DEG

rclpy.init()
node = rclpy.create_node("my_zero_reference_program")
try:
    client = ZeroReferenceClient(node)
    if not client.wait_for_service(timeout_sec=3.0):
        raise RuntimeError("请先启动维护模式和 pi_arm_zero_reference")
    future = client.set_zero_reference(
        [math.radians(angle) for angle in PACKING_JOINTS_DEG], reference_pose_confirmed=True,
    )
    rclpy.spin_until_future_complete(node, future, timeout_sec=6.0)
    if not future.done():
        raise RuntimeError("结果未知，请核对当前关节反馈和零点文件")
    result = future.result()
    if not result.success:
        raise RuntimeError(result.message)
    print(result.raw_protocol_positions_deg)
finally:
    node.destroy_node()
    rclpy.try_shutdown()
```

任务会校验参考姿态并串行处理标零。请求超时或连接中断时，先核对关节反馈与保存文件；硬件请求返回后才能开始下一次标零。

## 保存与后续运行

采集成功后，运行 `bash scripts/start_web.sh --monitor`，连接网页，核对关节反馈与实物参考姿态。正常控制、只读和维护模式都会自动读取当前接口的零点。自定义位置时，采集用 `--software-zero-file`，启动用 `software_zero_file:=/绝对路径/zero.yaml`。

首次采集和重新采集都使用当前的绳索模型、已知姿态和实测编码器读数。每次采集成功会覆盖同一个文件。请按新零点复核已有示教点。V0.52 的旧零点仍可搭配原基础配置加载，V0.53 的单文件可以直接读取。`--calibration-file` 可一并保存旧配置中已批准的打包运动目标。实物处于自定义姿态时，用 `--reference-deg` 指定本次采集的关节角。

加载软件零点要求电机协议角度参考与采集时一致。断电、驱动重启或 0x80 关闭后，先核对实物姿态和编码器参考，再决定是否重新采集。零偏变化后重新核验已有点位。

相关字段与参考约定见 [硬件配置](../../../pi_arm_hardware/config/README.md)，任务索引见 [pi_arm_tasks](../../README.md)。
