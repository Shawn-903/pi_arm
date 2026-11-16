# pi_arm_tasks

在已有 ROS Service / Action 上组织可复用任务。网页经 WebSocket 调用本包的公共接口，Python 程序和命令行也调用同一接口；本包不打开 CAN，不保存另一份零偏或绕过硬件检查。第一版提供软件标零，后续按实际需要增加任务。

## 软件标零

公共服务 `/pi_arm/tasks/set_zero_reference` 使用 `pi_arm_interfaces/srv/SetZeroReference`。任务检查输入并限制并发，再调用 `/pi_arm/commissioning/set_zero_reference`。反馈采样、参考校验、文件保存和零偏应用由硬件层统一完成。

软件标零只在 commissioning 维护模式执行，不自动运动到零位。操作者先将实物放到已知参考姿态并确认静止。网页入口固定为六关节均为 0° 的机械零位；Python/CLI 可以显式指定其他已知姿态。打包姿态不是六关节零位。

停止正常控制服务后，从仓库根目录启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
hardware_share="$(ros2 pkg prefix --share pi_arm_hardware)"
bash scripts/start_web.sh --commissioning \
  calibration_file:="$hardware_share/config/pack_reference_calibration.R10.recorded.yaml" \
  software_zero_file:="$HOME/.local/share/pi_arm/software-zero-can0.yaml"
```

在网页“方向测试”页面确认实物位于机械零位，勾选确认后点击“采集并保存软件零点”。成功会返回参考编号，并在 `HardwareState.software_zero_id` 中发布。不写电机 ROM/RAM 零点，不改原始 R10 测量。

不使用网页时，可改为启动 `ros2 launch pi_arm_bringup monitor.launch.py commissioning:=true ...`（传入相同两个文件参数），另一个终端运行 `ros2 run pi_arm_tasks pi_arm_tasks`；无需 WebSocket 或 HTTP 服务。

## 命令行与 Python

```bash
# 已由操作者确认实物处于六关节零位。
ros2 run pi_arm_tasks pi_arm_zero --confirm-reference

# 其他已知参考姿态；六关节顺序见源码索引，CLI 使用度。
ros2 run pi_arm_tasks pi_arm_zero --confirm-reference \
  --reference-deg 0 100.7 -85.6 0 0 0
```

Python API 使用弧度，返回 ROS future；调用方负责 spin，并检查业务结果：

```python
import rclpy
from pi_arm_tasks import TaskClient

rclpy.init()
node = rclpy.create_node("my_arm_program")
try:
    tasks = TaskClient(node)
    if not tasks.wait_for_service(timeout_sec=3.0):
        raise RuntimeError("请先启动维护模式和 pi_arm_tasks")
    result_future = tasks.set_zero_reference(
        [0.0] * 6, reference_pose_confirmed=True,
    )
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=6.0)
    if not result_future.done():
        raise RuntimeError("结果未知，核对当前参考编号；不要自动重试")
    result = result_future.result()
    if not result.success:
        raise RuntimeError(result.message)
    print(result.reference_id)
finally:
    node.destroy_node()
    rclpy.shutdown()
```

超时或失联不能当作标零失败且未生效；应核对反馈中的参考编号与持久化文件。任务不会自动重试，硬件请求未返回时拒绝开始另一项标零。

## 正常运行与参考有效性

正常控制启动时传入相同的 `calibration_file` 和 `software_zero_file`；零偏在初始化时加载并用于反馈和目标转换。留空 `software_zero_file` 表示使用原有基础标定，不会自动搜索文件。文件不存在、损坏或绑定关系不匹配会阻止正常运动。

程序重启重载的前提是电机协议角度参考未改变。断电、驱动重启、0x80 关闭等之后需重新确认或重建参考；本版本未验证断电恢复，不自动恢复多圈信息。零偏变化后重新核验旧点位，点位记录会保留 `software_zero_id`。

底层数据格式和约束见 [硬件配置](../pi_arm_hardware/config/README.md)，相关验证见 [脚本说明](../../scripts/README.md)。
