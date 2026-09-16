# Pi Arm ROS 2 Jazzy

本工作区提供 Pi Arm 的 ROS 2 Jazzy 描述、SocketCAN 硬件、`ros2_control`、MoveIt 2、业务管理与 WebSocket 接入。主动关节顺序固定为：

`joint_01, joint_02, joint_04, joint_05, joint_06, joint_09`

`joint_03` 模仿 `joint_04`，`joint_07` 模仿 `joint_06`，`joint_08` 以 `-1` 倍模仿 `joint_05`。绳索耦合位于执行器传动层，不是新增的关节约束。MoveIt 规划组为 `pi_arm`，末端坐标系为 `tool0`。

## 环境、安装和构建

目标平台是 64 位 Ubuntu 24.04（noble）和 ROS 2 Jazzy。使用真实硬件时需要 SocketCAN；仅在运行 RViz 或滑块界面时需要支持 OpenGL 的桌面环境。全新系统应先按 ROS 2 Jazzy 官方 Ubuntu 安装流程配置 ROS 软件源，然后克隆本仓库。

### 新系统首次依赖安装

首次进入工作区时运行一次：

```bash
cd ~/pi_arm
bash scripts/install_dependencies.sh
source /opt/ros/jazzy/setup.bash
bash scripts/build.sh
source install/setup.bash
```

依赖脚本安装 C/C++ 编译工具链（`build-essential`，包含 `g++`）、ROS 包和 Python 运行依赖，并执行 `rosdep install`。脚本直接读取 `/etc/os-release` 检查 noble，不依赖 `lsb-release`。当前十包工作区不包含可发现的 `pi_arm_validation` 包，因此不再固定安装只供该包绘图使用的 NumPy 和 Matplotlib。

### 日常构建

依赖安装完成后，日常构建不需要重复运行安装脚本：

```bash
cd ~/pi_arm
source /opt/ros/jazzy/setup.bash
bash scripts/build.sh
source install/setup.bash
```

构建脚本执行 `colcon build --symlink-install --event-handlers console_direct+`。每个新终端都需要重新 source `/opt/ros/jazzy/setup.bash` 和工作区的 `install/setup.bash`。

构建后运行全部测试并查看汇总：

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

当前应由 `colcon list` 发现 10 个包；`pi_arm_validation` 已不再是可发现的 ROS 包。

## 运行

无硬件调试：

```bash
ros2 launch pi_arm_bringup bringup.launch.py hardware_type:=mock
```

无图形界面的 mock：

```bash
ros2 launch pi_arm_bringup bringup.launch.py hardware_type:=mock use_rviz:=false
```

真实 CAN 只读监控（不会启动 controller_manager、MoveIt、manager、WebSocket 或 RViz；CAN 仅轮询 `0x92/0x9A/0x9C`）：

```bash
sudo bash scripts/setup_can.sh real can0 1000000
ros2 launch pi_arm_bringup monitor.launch.py \
  can_interface:=can0 calibration_file:=/绝对路径/approved-calibration.yaml
```

真实整栈默认仍禁止位置控制。只有标定已审批、指纹匹配、运动学有效、六电机反馈健康且全部使能后，才可由现场验收人员显式加入 `allow_real_control:=true`。普通开发和诊断不得开启此参数：

```bash
ros2 launch pi_arm_bringup bringup.launch.py \
  hardware_type:=real can_interface:=can0 bitrate:=1000000 use_rviz:=false \
  calibration_file:=/绝对路径/approved-calibration.yaml allow_real_control:=false
```

仅检查模型：

```bash
ros2 launch pi_arm_description display.launch.py backend:=mock
```

仅启动 MoveIt 节点或 RViz：

```bash
ros2 launch pi_arm_moveit_config move_group.launch.py hardware_type:=mock
ros2 launch pi_arm_moveit_config moveit_rviz.launch.py hardware_type:=mock
```

统一启动参数：

- `hardware_type:=mock|real`：选择确定性软件后端或真实 SocketCAN 后端。
- `can_interface:=can0`：真实硬件使用的 SocketCAN 网卡。波特率由 `scripts/setup_can.sh` 在启动 ROS 前配置；launch 中的 `bitrate` 仅用于保持统一部署参数，驱动不会在线修改已启动的网络接口。
- `use_rviz:=true|false`：是否启动 MoveIt RViz。
- `use_manager:=true|false`、`use_websocket:=true|false`：是否启动业务管理和 WebSocket 代理。
- `cable_kinematics_file:=...`：六轴绳索几何 YAML。默认使用安装后的 `pi_arm_description/config/cable_kinematics.yaml`，一般无需手动指定。
- `calibration_file:=...`：打包姿态 ROM 零位的软件机械零位标定。模板为 `approved:false`，真实控制会失败关闭。
- `allow_real_control:=false|true`：真实位置帧的最终人工门控，默认 `false`；Mock 不受该实机门控影响。

## 输出轴离线可视化

`pi_arm_output_visualizer` 可在不打开 SocketCAN、不连接或控制电机的情况下，把六个减速器输出轴角度通过当前 `model_version: 3` 绳索模型转换成六个主动关节位置并显示在 RViz 中：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pi_arm_output_visualizer output_shaft_visualizer.launch.py
```

输入话题为 `/pi_arm/model_output_shaft_states`，表示已进入 `model_output` 坐标的减速器输出轴角，单位为弧度。ROM 置零后的原始 `rom_relative_output` 不能直接输入；必须先加载审批且指纹匹配的标定并执行 `model_output = rom_relative_output + Δ`。无滑块界面时使用 `use_gui:=false`，无 RViz 时使用 `use_rviz:=false`。不要与 `pi_arm_description display.launch.py` 同时运行，因为两者都会发布 `/joint_states`。详细输入名称和手动发布示例见 [`src/pi_arm_output_visualizer/README.md`](src/pi_arm_output_visualizer/README.md)。离线显示成功只验证模型和消息链路，不能证明实机方向、零位、张力或安全性正确。

## SocketCAN 与 vcan

真实总线默认使用标准帧、`can0` 和 1 Mbit/s。关闭并重新配置接口：

```bash
sudo bash scripts/setup_can.sh real can0 1000000
```

没有电机时可创建虚拟总线检查传输层：

```bash
sudo bash scripts/setup_can.sh vcan vcan0
ip -details link show vcan0
python3 scripts/vcan_motor_emulator.py vcan0
```

另开终端可用真实 CAN backend 验证单 Socket、协议轮询和管理命令：

```bash
ros2 launch pi_arm_bringup bringup.launch.py \
  hardware_type:=real can_interface:=vcan0 use_rviz:=false
```

`vcan` 不是 ros2_control 的 `mock` 后端：前者配合仓库内电机响应器验证 SocketCAN 和协议层；一般 MoveIt 与控制器联调应优先使用 `hardware_type:=mock`。

控制 bringup 不让 controller manager 直接订阅全局 `/robot_description`。它将本轮 xacro 结果发布并订阅到
`/pi_arm/controller_robot_description/<hardware_type>/<can_interface>`，避免残留的 transient-local
`/robot_description` 发布者抢先注入旧 backend 或 CAN 接口参数。显示用 launch 仍可使用标准
`/robot_description`；控制链排障时应以 controller manager 启动日志打印的实际订阅 topic 为准。

## ROS API

主要接口位于 `/pi_arm` 命名空间：

- 状态：`/pi_arm/hardware_state`、`/pi_arm/state`。
- 电机服务：`/pi_arm/enable_motor`、`disable_motor`、`reset_motor`。普通 manager、WebSocket、网页和 ros2_control 辅助服务均不提供 ROM 置零。
- 运动 Action：`/pi_arm/movej`、`movejs`、`movel`、`direct_move`。

`HardwareState.connected` 只表示驱动与 SocketCAN 在线；`calibrated`、`kinematics_valid` 和 `control_permitted` 分别表示标定有效、运动学输出有效和真实控制许可。原始协议角、ROM 相对输出轴角、软件偏置和模型输出轴角分别在 `raw_protocol_positions_deg`、`raw_output_positions_rad`、`software_output_offsets_rad`、`model_output_positions_rad`。`HardwareState.enabled` 按 `joint_names` 表示关节依赖聚合后的可用状态；例如关闭电机 2 会使依赖它的多个关节变为不可用，但不代表这些电机控制器也已关闭。原始控制器状态必须读取按 `motor_ids` 对齐的 `motor_enabled`、`motor_errors`、`motor_moving`、`motor_fresh`、`motor_error_codes` 和 `motor_ages_sec`。WebSocket 保留旧的关节级 `enabled` 字段并新增 `joint_enabled`；网页显示单台电机状态时必须改用 `motor_ids` 和 `motor_enabled`，不能由关节状态反推电机状态。
`control_permitted=true` 只表示标定、运动学、反馈、故障状态和人工 `allow_real_control` 门控满足，并不武装 CAN 位置写入。启动、空闲和 `enable_motor` 后驱动仍保持 `MONITORING`，不会把 JTC 的旧 idle/hold command 发送为 `0xA4`。当前只有 manager 已检查并接受的 `DirectMove` 会通过内部 `/pi_arm/hardware/set_motion_armed` 租约临时武装真实位置写入，而且必须等待武装后出现不同于旧 idle/hold command 的新控制器命令；完成、取消、`stop_motion`、任意电机管理操作或反馈/标定失效都会解除租约并清除待发送位置批次。该内部服务不是 WebSocket/网页公开控制接口，也不是安全认证边界。MoveJ、MoveJs 和 MoveL 尚未接入同一租约生命周期，因此真实 backend 对它们保持失败关闭；Mock 不受真实 CAN 写入门控影响。
`enable_motor`/`disable_motor` 的 `motor_id=0` 表示全部六台电机，`1..6` 表示指定电机。两者只在反馈完整且新鲜、无 FAULT、连续三帧静止且没有运动任务占用时执行；整机为 `READY` 或部分使能导致的 `DISABLED` 均可继续逐台操作。若新鲜原始反馈已经确认目标电机处于请求状态，服务以成功返回而不重复下发管理命令。
- 停止服务：`/pi_arm/stop_motion`。这是软件层停止请求，不是机械急停。
- MoveIt/控制器：`/joint_trajectory_controller/follow_joint_trajectory`。
- WebSocket：服务端默认监听 `0.0.0.0:8765`；其他设备上的客户端应连接 `ws://<树莓派IP>:8765`。JSON 请求由 `pi_arm_websocket` 转换到上述 ROS 接口，状态由 `/pi_arm/state` 推送。

运动 Action 的终态与结果必须同时判断。客户端主动取消返回 `CANCELED`、`success=true`、`code=0`；由 `stop_motion` 中止的原运动返回 `ABORTED`、`success=false`、`code=3003`。运行中硬件反馈失联、故障或失能分别返回 `ABORTED`、`success=false` 和 `code=3101/3102/3103`，同时将任务标记为 `FAILED`，不得把硬件故障解释为一次成功运动。`stop_motion` 服务本身成功只表示停止请求已被接受，不表示原运动成功完成。

硬件 deactivate/cleanup 会解除位置写入租约、清除待发送的控制批次并停止 CAN driver，但当前不会自动发送 `0x80`。因此 ROS 进程退出、Ctrl-C 或网络断开都不能证明电机已经失能；真实系统仍必须使用经过单独验收的 disable 流程以及独立的硬件急停和断能链路。是否在进程退出时自动发送 `0x80` 属于实机安全策略，不由普通清理路径隐式决定。

Ubuntu 24.04/Jazzy 的 MoveIt 2.12.4 存在可重复的上游退出缺陷：`move_group` 在 SIGINT/SIGTERM 清理时可能沿 `MoveItCpp`/`TrajectoryExecutionManager`/`rclcpp::CallbackGroup` 析构链以 `-11` 退出。该现象与 MoveIt 官方开放问题 [moveit2#3680](https://github.com/moveit/moveit2/issues/3680) 和 [moveit2#3721](https://github.com/moveit/moveit2/issues/3721) 一致。不要屏蔽该退出码或把它记为完整 bringup 通过；应使用 `apt-cache policy ros-jazzy-moveit ros-jazzy-moveit-ros-move-group` 记录实际版本，并在上游修复版本可用后重新执行 mock/vcan 的启动与清理测试。该上游崩溃不改变“退出前必须先解除写入 gate”的项目要求。

可用 `ros2 interface show pi_arm_interfaces/action/MoveJ`、`ros2 action list`、`ros2 service list` 和 `ros2 topic echo /pi_arm/state` 查询实际字段与运行状态。

WebSocket 服务当前没有鉴权和 TLS，只能用于隔离的可信网络。网页关闭、WebSocket 断开或网线拔出不会自动调用 `stop_motion`，也不保证正在执行的运动停止；安全停机必须依靠独立、可验证的机械急停和断能措施。

仓库包含静态网页 [`web/control.html`](web/control.html)。它不由 `colcon` 安装或托管，可通过可信局域网内的静态 HTTP 服务器打开；页面中填写 `ws://<树莓派IP>:8765` 后手动连接。页面已按六主动关节和 CAN ID 1～6 区分关节状态与原始电机状态，并在 `control_permitted=false` 时禁用运动按钮。

## 配置单一数据源

限位、绳索几何和执行器接口配置各自只有一个来源，修改时只改对应文件：

- 关节位置/速度限位、effort：`pi_arm_description/urdf/pi_arm.urdf.xacro` 的 `<limit>` 标签。硬件插件通过 ros2_control 解析的 `HardwareInfo::limits` 读取，manager 通过 MoveIt robot model 读取，两者都不再持有副本。
- 关节加速度限制：`pi_arm_moveit_config/config/joint_limits.yaml`（URDF 无法表达加速度）。bringup 会把它以 `robot_description_planning` 命名空间同时传给 move_group 和 manager。
- 笛卡尔 TCP 上限：两处须手动对齐——（1）manager MoveL 拒绝/scaling：`manager_node.cpp` 中 `kMaxTransVelMps` 等编译期常量；（2）move_group Pilz：`pilz_cartesian_limits.yaml`。当前测试值为线速度 0.1 m/s、线加速度 0.2 m/s²、角速度 10°/s。角加速度由 Pilz 推导。
- 规划管道：`ompl`（`movej`）与 `pilz_industrial_motion_planner`（`movel` 使用 `LIN`）并存。依赖安装含 `ros-jazzy-pilz-industrial-motion-planner`。
- 绳索几何和减速器输出轴限位：`pi_arm_description/config/cable_kinematics.yaml`。当前必须为 `model_version: 3`；旧版本会被加载器拒绝。`pi_arm_transmission` 的输入是 ROS 主动关节坐标，机械等效关系 `S3=2*joint_04`、`S4=2*joint_05`、`S5=2*joint_06` 已代入库内公式，调用方不得再次乘以 2。输出轴限位以整条六轴命令为单位拒绝，不逐轴夹紧。
- 执行器接口映射（`motor_id`、`protocol_angle_ratio`、`motor_direction`）：URDF `<ros2_control>` 段内每个主动关节的 `<param>`，由硬件插件逐关节解析，缺失即启动失败。
- 关节名与顺序：URDF `<ros2_control>` 段的 `<joint>` 顺序即主动关节顺序；`controllers.yaml` 中的控制器关节列表须与其保持一致（ros2_control 约定）。

绳索参数、公式、软件分层和实机标定要求见：

- [`docs/05 绳索运动学.md`](docs/05%20绳索运动学.md)
- [`docs/04 ROS2_control控制链.md`](docs/04%20ROS2_control控制链.md)

## 打包姿态 ROM 零位与软件机械零位

当前操作人员确认的软件关节边界为 `joint_02 = ±100.7°`、`joint_04 = ±85.6°`，`joint_03` 模仿 J4 的同一边界。对应打包测量姿态 `[0°, 100.7°, -85.6°, 0°, 0°, 0°]` 位于软件限位边界，没有额外机械余量。这些输入及派生的输出轴包络不是实机安全限位认证；配置变更后必须重新生成和审批指纹匹配的标定，不能复用旧标定或用软件改限位代替 ROM 基准验证。

机械模型零位仍定义为机械臂完全竖直的 `q=0`。ROM 零位可在可重复打包工装姿态 `q_pack` 写入；软件偏置只在执行器坐标层使用：

```text
Δ = cable_forward(q_pack) - r_pack
model_output = rom_relative_output + Δ
rom_relative_command = cable_forward(joint_command) - Δ
joint = cable_inverse(model_output)
```

标定唯一入口是 `calibration_file`。文件保存 `robot_id`、六主动关节顺序、CAN ID 1～6、`q_pack`、`r_pack`、`model_version: 3`、配置指纹和审批信息；偏置在加载时计算，不重复保存。配置指纹为 FNV-1a 漂移检测值，不是密码学签名或授权机制。仓库模板 [`src/pi_arm_hardware/config/pack_reference_calibration.template.yaml`](src/pi_arm_hardware/config/pack_reference_calibration.template.yaml) 未审批，不能开启真实控制。

候选文件必须从展开后的当前 URDF 和当前绳索 YAML 生成，生成器始终写入 `approved: false`，不会接触 CAN：

```bash
xacro $(ros2 pkg prefix --share pi_arm_description)/urdf/pi_arm.urdf.xacro \
  backend:=real cable_kinematics_file:=$(ros2 pkg prefix --share pi_arm_description)/config/cable_kinematics.yaml \
  calibration_file:=/tmp/candidate.yaml allow_real_control:=false > /tmp/pi_arm.urdf
ros2 run pi_arm_hardware pi_arm_calibration_candidate \
  --robot-description /tmp/pi_arm.urdf --robot-id ROBOT_ID --calibration-id CAL_ID \
  --packing-pose Q1_RAD Q2_RAD Q4_RAD Q5_RAD Q6_RAD Q9_RAD \
  --raw-output-at-packing R1_RAD R2_RAD R3_RAD R4_RAD R5_RAD R6_RAD \
  --output /tmp/candidate.yaml
```

ROM 写入只存在于本地 `pi_arm_rom_zero_cli`，默认 dry-run、独占 SocketCAN、检查六台电机反馈/错误/静止状态并写 JSONL 审计。`--yes` 只允许 `vcan*`；真实 `can0 --execute` 还必须输入交互确认短语。本阶段禁止在实机运行：

```bash
ros2 run pi_arm_can pi_arm_rom_zero_cli --interface vcan0
# 仅用于独立 vcan 验收：
ros2 run pi_arm_can pi_arm_rom_zero_cli --interface vcan0 --execute --yes
```

真实 ROM 写入成功后必须断电重启，再在工装中采集 `r_pack`。该工具不是普通控制 API，网页和 WebSocket 不接受 `set_zero`。

验收：启动日志含 `MoveL TCP caps (compile-time)`；`ros2 param list /move_group --filter cartesian_limits` 应列出 Pilz 四项。

## 安全警告

当前速度 `0.1745329 rad/s`（10 deg/s，URDF `<limit velocity>`）、加速度 `0.3490659 rad/s²`（20 deg/s²，`joint_limits.yaml`）以及 URDF effort（肩部 20 N·m、腕部 10 N·m）均为临时保守值，不是经过认证的机械、电机或减速器额定参数。上线前必须根据电机、减速器、供电、制动距离、负载和结构强度重新标定，并在硬件驱动中实施独立限幅。

真实运行前必须：

1. 确认机械急停、限位、保险和断电措施有效，并清空运动范围。
2. 校验零位、关节方向、传动比、CAN ID、模仿关节方向及每轴软硬限位。
3. 首次上电使用低电流/低速度、无负载和单轴点动；禁止直接执行未经验证的 MoveIt 轨迹。
4. WebSocket 当前默认监听所有网卡且不代表已具备认证、授权或 TLS；只允许部署在受信隔离网络，生产环境须增加反向代理、鉴权和访问控制。
5. `mock` 成功仅说明软件链路可用，不证明真实机械臂安全或动力学可行。
6. `/pi_arm/stop_motion` 不是急停；网页、WebSocket 或网线断开不会自动停止运动。人员和设备安全必须由独立于 ROS、网页和网络的硬件急停及断能链路保障。
