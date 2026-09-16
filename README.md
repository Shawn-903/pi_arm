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

### 网页测试快速入口

#### 电机输出轴与关节轴的方向核验

使用专用方向测试模式，同时显示两种单轴点动控制（不与普通 bringup 或只读监控同时运行）：

```bash
bash scripts/start_web.sh --commissioning can_interface:=can0 \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R02.recorded.yaml
```

- **电机输出轴点动**：按 CAN ID 选择一台电机，先使能该电机，再点击 `+` / `−`。只向该电机发送位置命令。步长是按配置减速比折算的输出轴角，“+”严格表示协议角增加，不使用 `motor_direction`、标定偏置或绳索反解。这使电机方向核验可以在 `kinematics_valid=false` 时进行；六台电机原始反馈仍必须新鲜、无电机错误且静止。其他电机保持原有状态，不自动使能或失能。
- **关节轴点动**：选择 `joint_01 / joint_02 / joint_04 / joint_05 / joint_06 / joint_09`，按当前绳索模型生成耦合电机目标，可能向多台电机发命令。只有标定、反解有效且全部电机使能时才开放；起点、目标和输出插补路径仍检查软件限位。反解无效时页面保留控件并说明禁用原因，不能将未知关节姿态当作控制起点。

默认步长 0.1°、速度 0.2°/s；单次步长限制 0.01～0.5°，速度 0.05～0.5°/s，所有实际电机输出轴单次目标变化不超过 0.5°。协议速度按整数 °/s 编码，小于一个协议速度单位的耦合轴使用最小单位；输出轴速度上限仍不超过 0.5°/s。各输出轴目标距离本服务首次成功测试时的位置限制为 ±2°，超出后拒绝请求，不自动重新定义参考位置。参数含义及这些初始测试范围不构成实机限位认证。

一次只能运行一个点动。网页自动发送测试心跳，切换到后台页面会请求停止；关闭连接、心跳超过 0.8s、原始反馈失效、运动超时或超出预期输出轴位移时，后端请求厂家协议 `0x81` 停止并等待应答。它不修改 ROM 零位，也不关闭电机。停止未获应答会显示 `STOP UNCONFIRMED` 并阻止下一次点动，需重试停止/检查设备；总线失联时仍依赖独立物理急停。网页普通 `stop_motion` 的语义不变，此处的心跳仅约束方向测试。

建议逐台电机小步测试正/负方向，观察所有受影响关节，固定观察方向并按 URDF 关节轴的右手正方向记录。每次点动会保存指令、动作前后六轴反馈和最终结果；在“本次现场观察”中填写实际方向并保存，再用“下载当前诊断 JSON”导出。记录保存在当前页面内，刷新前请导出。软件不会把运动学预测当作现场观察，也不会自动修改方向或耦合参数。

`scripts/test_axis_vcan.py` 和 `scripts/test_web_axis.py` 用于虚拟 CAN 的接口与浏览器回归，检查所选 CAN ID、耦合命令、限步、心跳和停止。二者在发送命令前均拒绝真实 CAN。`vcan_motor_emulator.py --freeze-motion` 可模拟未到达目标以验证停止路径。

#### 实机有故障时的只读网页排查

J2/J4 反馈或运动学异常不妨碍连接网页。此时可使用独立的只读监控链：

```bash
# 若 can0 尚未启动：
sudo bash scripts/setup_can.sh real can0
bash scripts/start_web.sh --monitor can_interface:=can0 calibration_file:=/绝对路径/R02标定.yaml
```

打开 `http://机器IP:8000/control.html` 并点击“连接”。此模式只启动 CAN 只读监控、WebSocket 和网页，不启动 manager、MoveIt、运动控制器或电机管理服务，不要求 `allow_real_control:=true`。不要和其他占用同一 CAN 的监控/控制进程同时运行。

没有可用标定文件时可省略 `calibration_file`，仍可查看协议角、原始输出轴角和电机状态；默认未审批模板会明确显示 `CALIBRATION_INVALID`，不会伪装成已完成标定。要查看有效的软件偏置和模型输出角，应提供当前配置匹配的实机标定文件。下方交接记录保留了 R02 数据。

从该交接记录恢复的历史副本位于 `src/pi_arm_hardware/config/pack_reference_calibration.R02.recorded.yaml`，可将其绝对路径传给只读入口。它保留历史审批字段，不是新标定或当前实机姿态验证；配置指纹仍由后端检查。若启动时 CAN 未启用，先启用 CAN，再重启只读入口，以清除驱动连接错误。

页面的“实机排查 / 坐标换算链”显示驱动状态、标定 ID、标定/运动学有效性、禁止运动原因、错误轴序号和完整错误详情。六台电机分别显示协议角、原始输出轴角、软件偏置、模型输出轴角、错误码、反馈年龄和新鲜度。运动学无效时关节角与末端姿态显示 `--`，原始数据继续更新；电机数据缺失时会标为过期。可点击“下载当前诊断 JSON”保存精确数值和时间戳，与现场测量配对排查。故障不会阻止网页连接；即使监控进程启动失败，网页仍能显示未收到硬件状态的提示，此时需结合启动终端中的错误。

#### mock 运动测试

构建完成后，在工作区执行：

```bash
bash scripts/start_web.sh
```

脚本默认启动 **mock**、WebSocket（8765）和网页 HTTP 服务（8000），不需要桌面或 RViz。打开 `http://localhost:8000/control.html`；在另一台电脑上访问时，将 `localhost` 换成运行本项目的机器 IP。页面自动使用该主机的 WebSocket 地址，也可直接打开 `web/control.html` 并手动填入 `ws://机器IP:8765`。HTTP 端口可通过 `PI_ARM_WEB_PORT=8001 bash scripts/start_web.sh` 修改。不要与已启动的 bringup 重复运行；Ctrl+C 会关闭脚本启动的服务。

1. 点击“连接”，等待显示 `mock / DISABLED` 和六轴状态。
2. 点击“所有电机使能”，等待 `READY`。
3. 页面首次收到有效反馈会将当前角度填入目标。将 J1 目标小幅修改，例如从 0° 改为 1°，保留默认 1°/s，点击该轴“运动”。
4. 查看“测试结果”的任务编号、最终状态及实际角度；“运动请求已接受”不代表最终成功。服务拒绝、参数错误和任务失败都直接显示在页面。
5. 多关节页的“运动”使用该页目标和速度；“按单关节目标全部运动”使用单轴卡片中的六个目标及最低速度。两者均走 DirectMove。测试后可“停止运动”及“所有电机关闭”。

关节目标是绝对角度，单位 °，速度单位 °/s；“读取当前角度为目标”可重新填入保持位置。直线插补仅在 mock 中开放，实机当前仅开放 DirectMove。网关默认按实机限制规划命令；单独启动 mock 网关时需传入 `hardware_type:=mock`。过期状态、断线或运行中的任务会禁用新运动。

可选浏览器回归：在安装了 Python Playwright 和 Chromium 的测试环境中，保持上述 mock 服务运行，执行 `python3 scripts/test_web_mock.py`（也可用 `--browser-path` 指定 Chromium）。该脚本检查单轴、多轴、停止、失败反馈和断线交互，只允许连接标记为 mock 的网关，并会改变模拟关节位置。

实机准备完成后，启动脚本可以透传 bringup 参数，例如 `hardware_type:=real can_interface:=can0 calibration_file:=/绝对路径/已验证标定.yaml allow_real_control:=true`。**本仓库下方交接记录中的第二静态姿态核对仍需完成，当前网页/mock 验证不能替代该核对；不要通过修改关节限位或标定审批标志绕过实机问题。**

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

## 2026-09-16 实机交接状态

本节是删除临时构建产物和过程日志前保留的实机交接记录。它记录已观察事实和待解决问题，不能代替原始日志，也不构成实机运动安全认证。

### ROM 基准与 R02 标定

- 机器人：`PIARM-R1-SN0001`。
- ROM 写零时机械臂固定在打包姿态 `[0°, 100.7°, -85.6°, 0°, 0°, 0°]`。
- CAN ID 1～6 各执行并确认一次 `0x19`；随后使用厂家协议 `0x07` 完成三个独立重启/只读周期。
- 三周期共 90 个只读样本，减速器输出轴角的最大跨周期变化为 `2.908882086657216e-05 rad`。
- R02 批准标定的原文件 SHA256 为 `2603d8d48737c175a22d2eb4291498b14575523aee80ec59875aa8bf1646a2ea`。临时日志目录清理后如需重建文件，内容如下；配置发生任何变化后不得继续复用该指纹：

```yaml
calibration:
  schema_version: 1
  robot_id: PIARM-R1-SN0001
  calibration_id: PIARM-R1-SN0001-PACKCAL-20260916-R02
  approved: true
  model_version: 3
  created_at_utc: '2026-09-16T12:05:56Z'
  configuration_fingerprint: fnv1a64:0a5596b5685ec6cc
  joint_names: [joint_01, joint_02, joint_04, joint_05, joint_06, joint_09]
  motor_ids: [1, 2, 3, 4, 5, 6]
  packing_pose_joint_rad:
    [0.0, 1.75754655675829, -1.494001839707146, 0.0, 0.0, 0.0]
  raw_output_at_packing_rad:
    [1.9392547244381436e-05, 7.433809777012883e-05,
     -5.8177641733144315e-05, -4.84813681109536e-05,
     -4.84813681109536e-05, 5.817764173314432e-05]
```

### R5 DirectMove 已确认事实

- 操作者请求 `joint_01` 从约 `0°` 相对运动 `+2°`，速度 `1°/s`；发送的是正确的 J1 绝对目标，不存在 J1/J2 名称映射错误。
- manager 会将单关节 DirectMove 扩展为六关节轨迹，未选关节使用当前位置保持值。Action 被接受后，backend 在首次写周期以 `position limit exceeded at joint[1]` 拒绝整批命令；`joint[1]` 是零基下标，对应 `joint_02`。
- Action 终态为 `ABORTED`，`success=false`、`code=3001`，消息为 `Current goal cancelled during deactivate transition.`。
- 该场景从接受到结束均未观察到 `0xA4`，因此没有位置命令发给任何真实电机；J1 反馈变化约 `0.00023°`，属于协议量化范围。
- 关节限位判断本身允许等于上下限，只拒绝严格小于 lower 或严格大于 upper。当前日志使用 `%.6g`，无法显示 J2 命令相对上限的实际微小超出量；不要把它简单解释为“等于上限被拒绝”。

### 网页测试与运动学故障

- `web/control.html` 的单关节“运动”调用 DirectMove，但“所有关节运动”和多关节页调用 MoveJ；真实 backend 当前只允许 DirectMove 获得位置写入租约。
- 网页把命令响应写入浏览器开发者 Console，没有明显错误弹窗；网页显示“命令成功”只代表 Action 已接受，不代表 Action 最终成功。
- 后续只读状态曾显示六台 `motor_enabled=true`、无电机错误且静止，但 `kinematics_valid=false`、`control_permitted=false`、manager 为 `FAULT`。关键数据为：

```text
raw_protocol_positions_deg = [0.03, -319.55, 631.27, 0.04, -0.04, 0.03]
model_output_positions_rad =
  [-4.848136811095359e-06, 1.6025500068619676, -8.146277666983426,
   3.082398394860722, 3.0826311054276547, 2.9045814064449265]
motor_03 configured output limit = [-7.841, 7.841] rad
```

- 协议角到减速器输出轴角已经应用一次减速比：CAN 1～3 使用 `36`，CAN 4～6 使用 `6`；读取时除以该比值并应用 `motor_direction`，写入时反向乘回。以 CAN 3 为例，`631.27° / 36 * -1 = -0.306048 rad`，与发布的 raw output 一致。输出限位本身位于减速器输出轴坐标，不应再次应用减速比。
- 在当前运动学参数下，上述输出反解约为 `joint_02=+91.819°`、`joint_04=-95.996°`。如果现场实测不是该姿态，则需依次核对具体电机实际减速比、方向、CAN ID、ROM 基准以及绳索耦合系数/符号。打包姿态单点标定只校准偏置，不能验证离开该点后的比例和耦合关系。

### 下一轮必须先处理

1. 在机械断能状态人工测量第二个静态姿态的 J2/J4，并与只读 `0x92` 反馈和运动学反解比较；未完成前不恢复实机运动。
2. 修正 manager：硬件处于 `FAULT` 时必须继续允许受控失能，不能把“禁止运动”扩展为“禁止断能”。
3. 修正网页：直接显示服务拒绝原因和 Action 最终结果，不能只写浏览器 Console。
4. 提高限位拒绝日志精度，并在武装真实位置写入前验证完整六关节控制器轨迹/保持值。
5. 所有修复先在 mock/vcan0 验证，再恢复实机单轴低速测试。

本轮收尾时，在没有其他 SocketCAN 进程占用的情况下，CAN ID 1～6 各收到一次直接 `0x80` 关闭请求并分别返回应答。ROS 进程均已退出；`can0` 仍须由现场执行 `sudo bash scripts/setup_can.sh down can0` 关闭。

## 安全警告

当前速度 `0.1745329 rad/s`（10 deg/s，URDF `<limit velocity>`）、加速度 `0.3490659 rad/s²`（20 deg/s²，`joint_limits.yaml`）以及 URDF effort（肩部 20 N·m、腕部 10 N·m）均为临时保守值，不是经过认证的机械、电机或减速器额定参数。上线前必须根据电机、减速器、供电、制动距离、负载和结构强度重新标定，并在硬件驱动中实施独立限幅。

真实运行前必须：

1. 确认机械急停、限位、保险和断电措施有效，并清空运动范围。
2. 校验零位、关节方向、传动比、CAN ID、模仿关节方向及每轴软硬限位。
3. 首次上电使用低电流/低速度、无负载和单轴点动；禁止直接执行未经验证的 MoveIt 轨迹。
4. WebSocket 当前默认监听所有网卡且不代表已具备认证、授权或 TLS；只允许部署在受信隔离网络，生产环境须增加反向代理、鉴权和访问控制。
5. `mock` 成功仅说明软件链路可用，不证明真实机械臂安全或动力学可行。
6. `/pi_arm/stop_motion` 不是急停；网页、WebSocket 或网线断开不会自动停止运动。人员和设备安全必须由独立于 ROS、网页和网络的硬件急停及断能链路保障。
