# Pi Arm · 阶段一：网页控制基线

树莓派六自由度绳索机械臂，基于 Ubuntu 24.04 ARM64、ROS 2 Jazzy、SocketCAN、ros2_control 和 MoveIt 2。
阶段版本 **v0.1.0-stage1**（2026-09-17）：操作者确认当前版本可稳定运动，冻结为前端开发的控制基线。

## 前端开发分支

重复定位测试阶段存档分支为 **`重复定位精度测试1.0`**，后续开发分支为 `feature/repeatability-test-next`。功能、验证范围及恢复步骤见 [1.0 存档记录](docs/releases/repeatability-test-1.0.md)。

三维显示已通过操作者实机只读确认，存档为 **v0.2.0-viewer-baseline**（`85974a2`）。
交互开发基线见 [交互计划](docs/frontend/INTERACTIVE_PLAN.md)；点位示教开发来自 `feature/teaching-points-and-base-frames`，对应 [点位示教计划](docs/frontend/TEACHING_PLAN.md)。
到位判定已增加每轴 0.2° 容差、连续稳定检查及停止后复核，见 [修复记录](docs/releases/arrival-accuracy.md)。
阶段一控制基线保留在标签 `v0.1.0-stage1`。
新界面说明见 [Web README](web/README.md)，截图及测试见 [前端验收](docs/frontend/REVIEW.md)。
三维模型实现见 [三维验收](docs/frontend/VIEWER_REVIEW.md)。
新增滑块、模型关节拖动、输出轴反解 / 笛卡尔 IK、路径及回放，软件验收与实机切换见 [交互验收](docs/frontend/INTERACTIVE_REVIEW.md)。
网页增加选定电机的清错与重启，FAULT 下可使用原始静止反馈恢复，见 [电机故障恢复](docs/frontend/MOTOR_RECOVERY.md)。
当前新增持久化点位库、独立关节/笛卡尔表单及 base 下各关节位姿，见 [点位示教说明](docs/frontend/TEACHING_REVIEW.md)。
正常运动到位后保持位置，手动拖动请显式进入拖拽示教；点位传输保留三位小数，见 [保持与拖拽示教](docs/frontend/POSITION_HOLD_AND_DRAG_TEACHING.md)。

## 已完成

| 功能 | 当前实现 |
| --- | --- |
| 网页诊断 | 故障时也可连接；六电机协议反馈、关节反解、标定、错误、任务结果及 JSON 导出 |
| 点位示教 | 手动 / 实机反馈 / 模型预览录入，树莓派持久化、加载、删除及 JSON 导出 |
| 重复定位测试 | 起始点/目标点循环、次数与到点等待、到位采样、JSON/CSV 导出；[使用说明](docs/frontend/REPEATABILITY_TEST.md) |
| base 位姿 | 六主动关节子坐标系及 tool0 的 XYZ/RPY，由同一帧反馈与 URDF 计算 |
| 三维显示 | URDF 实时姿态、三个随动关节、base_link/tool0 坐标轴、关节目标叠加；断线保留最后姿态并标灰 |
| 交互预览 | 滑块 / 关节模型拖动、输出轴反解、IK、关节插值与 LIN 路径、回放；松手执行须显式开启 |
| 到位核验 | 每轴默认 0.2°、稳定 0.5 s、收敛等待 10 s，停止后复核，网页显示目标与误差 |
| 输出轴目标 | 六轴模型输出角目标，绳索反解及关节限位检查后进入轨迹控制 |
| 关节空间 | 单关节与整臂目标，共用任务、取消及停止链路 |
| 笛卡尔空间 | base_link 坐标系、tool0 法兰中心，MoveIt/Pilz LIN 规划 |
| 预设姿态 | 打包及零位，关节速度网页可调，默认 **5°/s**、最高 **10°/s** |
| 输出轴限速 | 网页可调整，默认 **5°/s**、最高 **10°/s**；下一条运动生效，落实到轨迹时长和 CAN 编码 |
| 单轴测试 | 独立 commissioning 模式：协议方向电机点动、绳索模型关节点动 |
| 停止和限位 | 停止撤销写入许可并检查停止应答；J2/J3 反馈允许 3° 容差，指令使用标准限位 |

控制链：`网页 → WebSocket → manager → joint_trajectory_controller → RealBackend → CAN`。
启动不自动运动。详见 [阶段验收记录](docs/releases/v0.1.0-stage1.md)。

## 安装与构建

先配置 ROS 2 Jazzy 软件源，然后在仓库根目录执行：

```bash
bash scripts/install_dependencies.sh
bash scripts/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

工作区包含 10 个 ROS 包。日常只需 `bash scripts/build.sh`，采用 symlink-install。

## 启动和停止

实机（供电和 CAN 就绪）：

```bash
cd /home/cr/pi_arm
bash scripts/start_web.sh \
  hardware_type:=real can_interface:=can0 allow_real_control:=true \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml
```

打开 `http://树莓派IP:8000/control.html`，点击连接。WebSocket 默认端口 8765。
保持启动终端打开；**Ctrl+C** 停止。每个 CAN 接口只运行一个服务实例。
若 CAN 未启用：`sudo bash scripts/setup_can.sh real can0 1000000`。

模拟、只读和方向测试入口（分别启动，不与已有实例并行）：

```bash
bash scripts/start_web.sh hardware_type:=mock

bash scripts/start_web.sh --monitor can_interface:=can0 \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml

bash scripts/start_web.sh --commissioning can_interface:=can0 \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml
```

HTTP 端口用 `PI_ARM_WEB_PORT=8001` 设置，WebSocket 可用 `websocket_port:=8766` 指定。

## 坐标与标定

| 网页轴 | ROS 独立关节 | 协议/输出轴比值 G | 安装方向 d | 绳索整体方向 s |
| --- | --- | ---: | ---: | ---: |
| J1 | joint_01 | 36 | +1 | +1 |
| J2 | joint_02 | 36 | +1 | +1 |
| J3 | joint_04 | 36 | −1 | −1 |
| J4 | joint_05 | 6 | −1 | −1 |
| J5 | joint_06 | 6 | −1 | +1 |
| J6 | joint_09 | 6 | +1 | −1 |

统一角度单位时：`φ = F(q)`、`φ = d × p / G + b`、`b = F(q_pack) − raw_pack`。
p 是协议角，φ 是物理输出轴相对模型零位的角，q 是关节角，b 是软件零偏。
绳索整体方向作用于整条输出方程；`coupling_sign` 只改变方程中的 J2 补偿项。

- 打包实测关节角：`[0, 100.7, -85.6, 0, 0, 0]°`，打包时协议角接近 0。
- 零位：关节角和模型输出角全 0，协议角通常不为 0。
- 当前 R10 标定和 `.cable.yaml` 快照位于 `src/pi_arm_hardware/config/`。
- 2026-09-20 按现场反馈撤销 R09 腕部系数加倍试验，恢复 33.94 mm 与 R08 标定。R08 保留已确认的腕部通道交换，沿用 ROM 置零后接受的 R03 测量。R02～R05 见 [标定历史](docs/calibration-history/README.md)。
- 2026-09-20 重新张紧绳索后，按操作者确认的打包姿态重新写入六轴 ROM 零点并重上电，采集生成 R10。原始协议角接近零，保留 33.94 mm 和已确认通道映射；旧 R08/R09 不得用于此次新零点。详见 [R10 测量记录](docs/calibration-history/R10-measurement.json)。
- 阶段运动获操作者确认，不等同于完成全部绳索耦合、回差和定位精度计量。

## 配置位置

| 项目 | 文件 / 参数 |
| --- | --- |
| 绳索方向、几何、J2 耦合 | `src/pi_arm_description/config/cable_kinematics.yaml` |
| 安装方向、减速比、关节限位和反馈容差 | `src/pi_arm_description/urdf/pi_arm.urdf.xacro` |
| 打包/零位共同关节限速 | 网页「预设姿态」速度框；默认 / 上限见 `src/pi_arm_websocket/pi_arm_websocket/motion_limits.py`（5 / 10 °/s） |
| 单次输出轴限速 | 网页输入框 / API 的 `output_speed_limit`，单位 °/s |
| 轨迹控制器 | `src/pi_arm_bringup/config/controllers.yaml` |
| 到位容差、稳定时间、收敛等待 | `src/pi_arm_bringup/config/manager.yaml`（默认 0.2° / 0.5 s / 10 s） |

仅修改绳索整体方向时，先停止服务，再生成新标定并在下次启动使用其路径：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/rebind_cable_directions.py \
  --reference src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml \
  --output src/pi_arm_hardware/config/pack_reference_calibration.R11.recorded.yaml \
  --calibration-id PIARM-R1-SN0001-PACKCAL-20260920-R11
```

脚本保留原始测量，不连接 CAN、不写 ROM；拒绝几何、耦合项、限位或安装映射变化。
ROM 零点变化或重新装配后应重新测量。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 scripts/test_rebind_cable_directions.py
# 需要独立、已启用的虚拟 vcan0，脚本禁止物理 CAN：
python3 scripts/test_motion_safety_vcan.py
```

浏览器测试需 Python Playwright/Chromium：`test_web_mock.py`、`test_web_presets.py`、
`test_web_control_spaces.py` 仅允许 mock/vcan；`test_web_diagnostics.py` 使用离线合成反馈。

## 使用边界

- 当前无身份鉴权和 TLS，面向可信隔离网络；前端角色视图不代表权限隔离。
- 网页停止是软件请求，不是物理急停。普通轨迹不因浏览器断线自动停止；commissioning 有独立心跳。
- DirectMove、输出轴目标及预设姿态采用关节插值，不经过 MoveIt 避障，不保证末端直线。
- 容差区起点命令投影到标准边界；笛卡尔规划可能先经过这一小段边界过渡。
- 关节反馈来自绳索模型反解，不是独立关节传感器；模型自洽不能代替实测。
- 曾观察到系统 MoveIt `move_group` 退出时段错误，运行控制测试通过，退出问题尚未解决。

## 目录

`src/` ROS 控制与模型；`web/` 网页；`scripts/` 构建启动及正式回归；
`docs/` 原理、阶段记录、标定历史；`build/ install/ log/` 是忽略提交的本机产物。
历史审查见 [审查目录](docs/reviews/2026-09-17/README.md)，当前操作以本 README 为准。

腕部通道对应：`cable_kinematics.swap_wrist_channels: true`，CAN4 → J5、CAN5 → J4。说明见 [腕部映射](docs/frontend/WRIST_CHANNEL_SWAP.md)，已撤销的比例试验见 [R09 记录](docs/frontend/WRIST_SCALE_TRIAL.md)，后续检查见 [腕部幅度排查](docs/frontend/WRIST_DIAGNOSTICS.md)。
