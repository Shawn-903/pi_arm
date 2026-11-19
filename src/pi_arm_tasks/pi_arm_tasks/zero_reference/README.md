# 软件标零任务

在实物处于已知关节姿态时采集编码器读数，计算并保存软件零偏。可以用一条命令完成，也可以通过网页或 Python 调用。

## 一条命令标零

树莓派上的项目完成构建、CAN 接口就绪后，先停止正在运行的控制或维护服务，将机械臂放到机械零位并保持静止。在仓库根目录执行：

```bash
bash scripts/zero_reference.sh --confirm-reference
```

脚本会加载 ROS 和当前工作区环境，启动硬件维护服务与标零服务，等待电机反馈就绪后采集一次零点，保存结果，再关闭本次启动的服务。`--confirm-reference` 表示你已确认实物处于给定姿态；默认参考关节角均为零。

默认使用 `can0`，将打包姿态、模型指纹和本次采集的零点一起保存到 `~/.local/share/pi_arm/software-zero-can0.yaml`。再次标零会更新同一文件。切换 CAN 接口时，文件名中的 `can0` 随接口名变化。

采集其他已知姿态时，使用 `--reference-deg` 传入角度，单位为度，顺序为 `joint_01, joint_02, joint_04, joint_05, joint_06, joint_09`：

```bash
bash scripts/zero_reference.sh --confirm-reference \
  --reference-deg 0 100.7 -85.6 0 0 0
```

该例将当前实物姿态登记为这组角度。自定义绳索模型用 `--cable-kinematics-file` 指定；服务和静止反馈的等待时间由 `--startup-timeout` 设置，默认 15 秒。

运行日志输出到终端。成功时，最后一行 JSON 给出 `success`、`reference_id`、`software_zero_file` 和 `cleanup_success`，命令退出码为 0。失败时会说明原因并清理本次服务；若返回 `result_unknown: true`，按保存文件中的参考编号核对结果后再操作。按 `Ctrl+C` 可以结束运行，已发出的采集请求会先等待本次结果，再退出。

已经加载 ROS 和工作区环境时，也可直接执行 `ros2 run pi_arm_tasks pi_arm_zero_once --confirm-reference`。完整参数见 `bash scripts/zero_reference.sh --help`。

## 启动与网页使用

将机械臂放到参考姿态并保持静止，停止正常控制服务后，在仓库根目录启动维护模式：

```bash
bash scripts/start_web.sh --commissioning
```

该入口会启动 `pi_arm_zero_reference` 服务节点。网页“方向测试”中，将实物放到六关节机械零位，勾选确认后点击“采集并保存软件零点”。成功后显示参考编号，并通过 `HardwareState.software_zero_id` 发布。

## 命令行与 Python

接入已运行的维护服务时，使用 `pi_arm_zero` 客户端或 `ZeroReferenceClient`。它们调用 `/pi_arm/tasks/set_zero_reference`，复用服务的配置和保存路径。

上面的网页启动命令已经包含所需服务。独立 ROS 程序也可以先启动 `pi_arm_bringup` 的 `monitor.launch.py`，设置 `commissioning:=true`，再在另一个已加载环境的终端执行 `ros2 run pi_arm_tasks pi_arm_zero_reference`。

客户端命令行使用度，默认参考姿态为机械零位：

```bash
ros2 run pi_arm_tasks pi_arm_zero --confirm-reference
ros2 run pi_arm_tasks pi_arm_zero --confirm-reference \
  --reference-deg 0 100.7 -85.6 0 0 0
```

Python 使用弧度，`set_zero_reference()` 返回 ROS future。调用方通过 spin 接收结果：

```python
import rclpy
from pi_arm_tasks.zero_reference import ZeroReferenceClient

rclpy.init()
node = rclpy.create_node("my_zero_reference_program")
try:
    client = ZeroReferenceClient(node)
    if not client.wait_for_service(timeout_sec=3.0):
        raise RuntimeError("请先启动维护模式和 pi_arm_zero_reference")
    future = client.set_zero_reference(
        [0.0] * 6, reference_pose_confirmed=True,
    )
    rclpy.spin_until_future_complete(node, future, timeout_sec=6.0)
    if not future.done():
        raise RuntimeError("结果未知，请核对反馈中的参考编号和零点文件")
    result = future.result()
    if not result.success:
        raise RuntimeError(result.message)
    print(result.reference_id)
finally:
    node.destroy_node()
    rclpy.try_shutdown()
```

任务会校验参考姿态并串行处理标零。请求超时或连接中断时，先核对反馈中的参考编号与保存文件；硬件请求返回后才能开始下一次标零。

## 保存与后续运行

采集成功后，运行 `bash scripts/start_web.sh --monitor`，连接网页，核对关节反馈与实物参考姿态、诊断中的 `software_zero_id` 与采集返回的 `reference_id`。正常控制、只读和维护模式都会自动读取当前接口的零点。自定义位置时，采集用 `--software-zero-file`，启动用 `software_zero_file:=/绝对路径/zero.yaml`。

首次采集会从包内 R10 读取基础配置。使用其他已批准配置时，可在首次采集时指定 `--calibration-file`。旧版零点也可读取，自定义基础配置继续用原路径指定，下次成功采集时升级为单文件。

加载软件零点要求电机协议角度参考与采集时一致。断电、驱动重启或 0x80 关闭后，先核对实物姿态和编码器参考，再决定是否重新采集。零偏变化后重新核验已有点位。

相关字段与参考约定见 [硬件配置](../../../pi_arm_hardware/config/README.md)，任务索引见 [pi_arm_tasks](../../README.md)。
