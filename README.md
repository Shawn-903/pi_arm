# Pi Arm · 阶段一：网页控制基线

树莓派六自由度绳索机械臂，基于 Ubuntu 24.04 ARM64、ROS 2 Jazzy、SocketCAN、ros2_control 和 MoveIt 2。
阶段版本 **v0.1.0-stage1**（2026-09-17）：操作者确认当前版本可稳定运动，冻结为前端开发的控制基线。

## 前端开发分支

当前分支 `feature/web-console-v2` 已按 [前端计划](docs/frontend/PLAN.md) 完成首轮工作台重构。
已加入 URDF 实时三维姿态、关节选择、坐标轴及独立目标预览，软件验证完成，待实机比对。
阶段一控制基线保留在标签 `v0.1.0-stage1`。
新界面说明见 [Web README](web/README.md)，截图及测试见 [前端验收](docs/frontend/REVIEW.md)。
三维模型实现与现场确认步骤见 [三维验收](docs/frontend/VIEWER_REVIEW.md)。

## 已完成

| 功能 | 当前实现 |
| --- | --- |
| 网页诊断 | 故障时也可连接；六电机协议反馈、关节反解、标定、错误、任务结果及 JSON 导出 |
| 三维显示 | URDF 实时姿态、三个随动关节、base_link/tool0 坐标轴、关节目标叠加；断线保留最后姿态并标灰 |
| 输出轴目标 | 六轴模型输出角目标，绳索反解及关节限位检查后进入轨迹控制 |
| 关节空间 | 单关节与整臂目标，共用任务、取消及停止链路 |
| 笛卡尔空间 | base_link 坐标系、tool0 法兰中心，MoveIt/Pilz LIN 规划 |
| 预设姿态 | 打包及零位，共同关节速度上限 **1°/s** |
| 输出轴限速 | 网页可调整，默认 **5°/s**；下一条运动生效，落实到轨迹时长和 CAN 编码 |
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
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R06.recorded.yaml
```

打开 `http://树莓派IP:8000/control.html`，点击连接。WebSocket 默认端口 8765。
保持启动终端打开；**Ctrl+C** 停止。每个 CAN 接口只运行一个服务实例。
若 CAN 未启用：`sudo bash scripts/setup_can.sh real can0 1000000`。

模拟、只读和方向测试入口（分别启动，不与已有实例并行）：

```bash
bash scripts/start_web.sh hardware_type:=mock

bash scripts/start_web.sh --monitor can_interface:=can0 \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R06.recorded.yaml

bash scripts/start_web.sh --commissioning can_interface:=can0 \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R06.recorded.yaml
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
- 当前 R06 标定和 `.cable.yaml` 快照位于 `src/pi_arm_hardware/config/`。
- R06 沿用 ROM 置零后接受的 R03 测量。R02～R05 见 [标定历史](docs/calibration-history/README.md)。
- 阶段运动获操作者确认，不等同于完成全部绳索耦合、回差和定位精度计量。

## 配置位置

| 项目 | 文件 / 参数 |
| --- | --- |
| 绳索方向、几何、J2 耦合 | `src/pi_arm_description/config/cable_kinematics.yaml` |
| 安装方向、减速比、关节限位和反馈容差 | `src/pi_arm_description/urdf/pi_arm.urdf.xacro` |
| 打包/零位共同关节限速 | `src/pi_arm_websocket/pi_arm_websocket/presets.py`：`PRESET_JOINT_SPEED_DEG_S = 1.0` |
| 单次输出轴限速 | 网页输入框 / API 的 `output_speed_limit`，单位 °/s |
| 轨迹控制器 | `src/pi_arm_bringup/config/controllers.yaml` |

仅修改绳索整体方向时，先停止服务，再生成新标定并在下次启动使用其路径：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/rebind_cable_directions.py \
  --reference src/pi_arm_hardware/config/pack_reference_calibration.R06.recorded.yaml \
  --output src/pi_arm_hardware/config/pack_reference_calibration.R07.recorded.yaml \
  --calibration-id PIARM-R1-SN0001-PACKCAL-20260917-R07
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
