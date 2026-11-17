# 软件标零任务

在实物处于已知关节姿态时采集编码器读数，计算并保存软件零偏。网页、Python 程序和命令行共用 `/pi_arm/tasks/set_zero_reference` 服务，反馈采样与文件保存由硬件层完成。

## 启动与网页使用

将机械臂放到参考姿态并保持静止，停止正常控制服务后，在仓库根目录启动维护模式：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
hardware_share="$(ros2 pkg prefix --share pi_arm_hardware)"
bash scripts/start_web.sh --commissioning \
  calibration_file:="$hardware_share/config/pack_reference_calibration.R10.recorded.yaml" \
  software_zero_file:="$HOME/.local/share/pi_arm/software-zero-can0.yaml"
```

该入口会启动 `pi_arm_zero_reference` 服务节点。网页“方向测试”中，将实物放到六关节机械零位，勾选确认后点击“采集并保存软件零点”。成功后显示参考编号，并通过 `HardwareState.software_zero_id` 发布。

独立 ROS 程序可先启动 `pi_arm_bringup` 的 `monitor.launch.py`，设置 `commissioning:=true` 并传入相同文件参数，再在另一个已加载环境的终端执行 `ros2 run pi_arm_tasks pi_arm_zero_reference`。

## 命令行与 Python

命令行使用度，按主动关节顺序传入已确认的参考姿态；默认值为六关节零位：

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

零偏保存到 `software_zero_file`。正常控制启动时继续指定原基础 `calibration_file` 和这个软件零点文件，初始化时加载偏移，用于反馈及目标坐标换算。

加载软件零点要求电机协议角度参考与采集时一致。断电、驱动重启或 0x80 关闭后，先核对实物姿态和编码器参考，再决定是否重新采集。零偏变化后重新核验已有点位。

相关字段与参考约定见 [硬件配置](../../../pi_arm_hardware/config/README.md)，任务索引见 [pi_arm_tasks](../../README.md)。
