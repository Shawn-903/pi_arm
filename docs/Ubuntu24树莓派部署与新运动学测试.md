# Ubuntu 24.04 树莓派部署与新运动学测试

本文用于将 `pi-arm-main` 部署到 64 位 Ubuntu 24.04、ROS 2 Jazzy 的树莓派，并分阶段验证第二版绳索运动学。

## 1. 本版运动学边界

- 六个真实电机，CAN ID 为 1～6；
- ROS 主动关节顺序为 `joint_01, joint_02, joint_03, joint_05, joint_06, joint_09`；
- `joint_04`、`joint_07`、`joint_08` 继续由 URDF mimic 表示机械同步；
- `pi_arm_transmission` 直接接收 ROS 主动关节坐标；
- `S3=2*joint_03`、`S4=2*joint_05`、`S5=2*joint_06` 已代入库内公式，其他代码不得再次乘以 2；
- 运行参数必须使用 `cable_kinematics.model_version: 2`。

当前参数仍有待实机确认的部分：

- 六个 `motor_direction`；
- 第三至第六轴的 `coupling_sign`；
- 第六轴小绳路导向有效半径的加工或实测值；
- 2.5 mm 绳索是否确实表示直径；
- 第六轴 12.25/17.25 mm 与记录中的 12.15/17.15 mm 应采用哪组有效半径。

## 2. 平台检查

在树莓派终端执行：

```bash
uname -m
lsb_release -sc
echo "$ROS_DISTRO"
```

期望：

```text
aarch64
noble
jazzy
```

若 `ROS_DISTRO` 为空，先执行：

```bash
source /opt/ros/jazzy/setup.bash
```

## 3. 安装依赖并构建

进入工作区根目录：

```bash
cd ~/pi_arm
bash scripts/install_dependencies.sh
bash scripts/build.sh
source install/setup.bash
```

`install_dependencies.sh` 会安装 ROS 2、MoveIt、ros2_control、SocketCAN、`python3-yaml`、`python3-numpy`、`python3-matplotlib` 和测试所需依赖，并执行 `rosdep install`。

构建成功时，最后应出现：

```text
Summary: ... packages finished
构建完成。
```

如果构建失败，不要启动真实电机。保存完整终端输出，优先检查第一个报错包。

## 4. 运行自动化测试

先测试与本次修改直接相关的包：

```bash
colcon test --packages-select \
  pi_arm_transmission \
  pi_arm_description \
  pi_arm_can \
  pi_arm_hardware \
  pi_arm_validation \
  --event-handlers console_direct+

colcon test-result --verbose
```

期望最终没有失败测试：

```text
Summary: ... tests, 0 errors, 0 failures, 0 skipped
```

再运行整个工作区测试：

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

### 4.1 生成运动学可视化报告

运行独立验证包：

```bash
RESULT_DIR="$PWD/pi_arm_validation_results_$(date +%Y%m%d_%H%M%S)"

ros2 run pi_arm_validation kinematics_validation \
  --output-dir "$RESULT_DIR" \
  --samples 10000 \
  --sweep-points 101

ros2 run pi_arm_validation plot_kinematics_validation \
  --input-dir "$RESULT_DIR"
```

`RESULT_DIR` 使用当前日期和时间创建独立目录，因此不会覆盖或删除之前的报告。

成功后输出目录包含：

```text
summary.json
position_roundtrip.csv
velocity_roundtrip.csv
jacobian_error.csv
joint2_coupling.csv
axis45_nonlinear.csv
axis6_residual.csv
position_error.png
velocity_error.png
jacobian_error_heatmap.png
joint2_coupling.png
axis45_nonlinear.png
axis6_residual_heatmap.png
report.md
```

在无桌面的树莓派上不需要打开图形窗口。可以将整个目录复制到电脑查看：

```bash
scp -r <树莓派用户名>@<树莓派地址>:<树莓派工作空间>/pi_arm_validation_results_YYYYMMDD_HHMMSS .
```

## 5. Mock 验证

首先不连接真实 CAN：

```bash
source /opt/ros/jazzy/setup.bash
source ~/pi_arm/install/setup.bash

ros2 launch pi_arm_bringup bringup.launch.py \
  hardware_type:=mock \
  use_rviz:=false
```

另开终端检查：

```bash
source /opt/ros/jazzy/setup.bash
source ~/pi_arm/install/setup.bash

ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /pi_arm/hardware_state --once
```

应看到六个主动关节，并且控制器处于 `active`。

重点验证：

1. 只改变 `joint_02` 时，ROS 中其他关节目标值不应改变；
2. 绳索库内部会同时重新计算电机 2～6 的输出轴目标；
3. `joint_04`、`joint_07`、`joint_08` 仍只由 mimic 关系产生；
4. 第四、第五主动关节不要测试到接近 $\pm\pi/2$ 的位置。

## 6. vcan 协议验证

创建虚拟 CAN：

```bash
sudo bash scripts/setup_can.sh vcan vcan0
ip -details link show vcan0
```

终端 A：

```bash
source /opt/ros/jazzy/setup.bash
source ~/pi_arm/install/setup.bash
python3 scripts/vcan_motor_emulator.py vcan0
```

终端 B：

```bash
source /opt/ros/jazzy/setup.bash
source ~/pi_arm/install/setup.bash

ros2 launch pi_arm_bringup bringup.launch.py \
  hardware_type:=real \
  can_interface:=vcan0 \
  use_rviz:=false
```

检查六个虚拟电机均有新鲜状态，且没有 `model_version`、参数兼容性或反正弦定义域错误。

## 7. 真实机械臂测试前检查

连接真实机械臂前：

- 机械臂应可靠支撑，避免失控后因重力坠落；
- 急停或总电源开关必须可立即触达；
- 工作空间内不得有人；
- 首次测试不要携带负载；
- 保存原始 YAML 和实机标定记录；
- 确认六个电机 ID 为 1～6；
- 暂时不要调用 `set_zero`，该命令会写入电机 ROM，并要求重新上电；
- 不要一开始运行 MoveIt 大范围轨迹。

配置真实 CAN：

```bash
sudo bash scripts/setup_can.sh real can0 1000000
ip -details link show can0
candump can0
```

先仅观察总线，确认没有持续错误帧或总线关闭状态。

## 8. 真实运动分阶段验收

### 8.1 反馈方向

手动或极低速分别移动每个机构，记录：

- CAN 协议角度变化方向；
- ROS 主动关节状态变化方向；
- URDF/RViz 显示方向。

若方向错误，只修改对应的 `motor_direction` 或 `coupling_sign`，不要通过给半径写负数来修正方向。

### 8.2 第一、第二轴

先验证无非线性自身项的第一、第二轴：

- 指令幅值不超过 0.01～0.02 rad；
- 确认第一轴只产生电机 1 的主要运动；
- 第二轴运动时，电机 3～6 应出现用于保持后续关节角不变的耦合补偿。

### 8.3 第三至第五轴

逐轴进行小幅运动：

- 第三轴检查 `joint_03/joint_04` 齿轮同步；
- 第四轴检查 `joint_05/joint_08` 连杆同步；
- 第五轴检查 `joint_06/joint_07` 连杆同步；
- 初始角度应远离 $\pm\pi/2$；
- 观察绳索是否保持张紧、有无明显滑移或异常拉力。

### 8.4 第六轴

先保持第二轴不动，仅小幅转动第六轴，检查两条绳路是否同时保持张紧。

再保持第六轴 ROS 目标不变，小幅转动第二轴，观察电机 6 的耦合补偿。若一条绳路逐渐松弛而另一条变紧，应立即停止，重新检查：

- 两个输出轴绳轮的有效半径；
- 两个关节端绳轮的有效半径；
- 两个关节 2 导向有效半径；
- 两条绳路的耦合方向。

## 9. 测试记录

每次测试至少记录：

- 当前 YAML 文件；
- 六个 ROS 关节位置和速度；
- 六个 CAN 协议角度和速度；
- `motor_direction` 与 `coupling_sign`；
- 测试指令、幅值和持续时间；
- 绳索张紧情况；
- 是否出现定义域、奇异性、过期状态或 CAN 错误。

参数修改后重新执行：

```bash
colcon build --symlink-install
colcon test --packages-select pi_arm_transmission pi_arm_description pi_arm_hardware
colcon test-result --verbose
```

YAML 是安装资源。即使使用 `--symlink-install`，修改参数后也应重新构建并重新启动硬件生命周期，避免运行进程继续使用旧参数。
