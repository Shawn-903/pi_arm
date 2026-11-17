# Pi Arm

Pi Arm 是树莓派六自由度绳索机械臂的控制软件，目标环境为 Ubuntu 24.04 ARM64 / ROS 2 Jazzy，使用 SocketCAN、ros2_control 和 MoveIt 2。

网页控制台提供六轴状态与三维模型显示、关节与末端运动、点位示教和重复定位测试。可按用途选择模拟运行、只读监测、单轴方向测试或实机控制。

## 文档入口

| 需求 | 文档 |
| --- | --- |
| 安装、启动、停止、恢复版本 | 本文件 |
| 网页操作、三维预览、示教、重复定位测试 | [web/README.md](web/README.md) |
| 控制架构、包索引、开发约定 | [src/README.md](src/README.md) |
| 构建、测试脚本与隔离测试环境 | [scripts/README.md](scripts/README.md) |
| CAN 原始协议、标定证据、传动模型 | [docs/README.md](docs/README.md) |
| 当前标定、坐标换算及配置维护 | [硬件配置 README](src/pi_arm_hardware/config/README.md) |
| 软件标零、标定姿态采样、Python 任务调用和命令行 | [pi_arm_tasks](src/pi_arm_tasks/README.md) |

## 安装与构建

在树莓派上安装 ROS 2 Jazzy 并配置其软件源，再在项目根目录执行。以下命令以 `/home/cr/pi_arm` 为项目路径，使用时按实际位置替换：

```bash
cd /home/cr/pi_arm
bash scripts/install_dependencies.sh
bash scripts/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

依赖脚本使用 sudo 安装系统包。构建采用 `colcon build --symlink-install`，并从 Xacro 导出网页模型。日常重新构建执行 `bash scripts/build.sh`；静态网页文件修改后刷新浏览器即可。新开终端运行 `ros2` 命令时，先执行上面的两条 `source` 命令，加载 ROS 和本项目的包。

## 选择启动模式

`start_web.sh` 同时启动 ROS 服务和网页 HTTP 服务，默认使用 mock 模拟后端。同一 CAN 接口只运行一个驱动实例，切换模式前先停止上一实例。

首次熟悉项目可从 [模拟运行](#模拟运行) 开始，体验连接、设置目标和查看任务结果。连接实物后，可先用只读监测查看反馈，再按需要进入正常控制或单轴方向测试。`commissioning` 是维护模式的名称。

### 实机正常控制

供电、CAN 硬件和接口就绪后：

```bash
bash scripts/start_web.sh \
  hardware_type:=real can_interface:=can0 allow_real_control:=true \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml
```

当前参考为 R10，打包关节角 `[0, 100.7, -85.6, 0, 0, 0]°`。R10 与 ROM 零点和绳索配置配套使用，重新装配或标零后需重新确认参考，步骤见 [硬件配置](src/pi_arm_hardware/config/README.md)。

已采集软件零点时，后续启动继续使用原基础标定，并传入保存零点的绝对路径，例如 `software_zero_file:=$HOME/.local/share/pi_arm/software-zero-can0.yaml`。该参数默认为空，指定后加载持久化零偏。采集步骤与 Python 调用见 [任务包](src/pi_arm_tasks/README.md)。

DirectMove、输出轴和预设姿态使用关节插值，执行前确认沿途空间。笛卡尔 LIN 规划末端直线路径，并按已建模场景检查碰撞。

### 只读监测

```bash
bash scripts/start_web.sh --monitor can_interface:=can0 \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml
```

用于查看电机反馈和诊断信息。标定无效时仍可查看原始电机数据。

### 单轴方向测试

```bash
bash scripts/start_web.sh --commissioning can_interface:=can0 \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml
```

用于核对电机、关节方向及执行单轴小步测试，网页操作见 [方向测试](web/README.md#方向测试)。

### 模拟运行

```bash
bash scripts/start_web.sh hardware_type:=mock
```

mock 后端在内存中生成电机反馈，用于调试控制流程和网页交互。打开 [网页控制台](#打开网页与停止) 并连接后，后端类型应显示 `mock`，六轴状态和模型随模拟反馈更新。点击“所有电机使能”，在关节控制面板设置目标并执行，可观察模型运动和任务结果。

与正在运行的实机并行开发时，应使用独立 ROS 域及端口：

```bash
ROS_DOMAIN_ID=81 PI_ARM_WEB_PORT=8001 bash scripts/start_web.sh \
  hardware_type:=mock websocket_port:=8766 \
  points_file:=/tmp/pi-arm-mock-points.json
```

对应网页：`http://树莓派IP:8001/control.html?ws=ws://树莓派IP:8766`。虚拟 CAN 的准备与测试见 [scripts/README.md](scripts/README.md)。

## 打开网页与停止

在浏览器打开 `http://树莓派IP:8000/control.html`，点击“连接”；默认 WebSocket 地址为 `ws://树莓派IP:8765`。三维模型通过该 HTTP 页面加载。实机控制时，先核对标定、六轴反馈和控制许可，再手动使能。详细操作见 [web/README.md](web/README.md)。

`PI_ARM_WEB_PORT` 设置 HTTP 端口；普通控制和方向测试模式可用 `websocket_port` 设置 WebSocket 端口。点位文件用 `points_file` 指定。

控制页面部署在可信隔离网络中，由操作者管理访问。普通轨迹由后端执行，网页断开后可能继续到目标；结束操作前先停止运动并核对结果。重复测试的断连处理见 [网页说明](web/README.md#重复定位测试)。

- **停止运动**：点击网页“停止运动”，请求取消当前任务并保持当前位置，随后核对结果；故障或失联时需按诊断处理。紧急情况下使用现场物理急停措施。
- **停止程序**：先结束测试或运动并支撑机械臂，再在启动终端按 `Ctrl+C`。服务退出会释放位置闭环。
- **更新后端**：停止旧服务，构建，再按上述命令启动。
- **更新前端**：刷新网页。关联脚本有版本化 URL；若界面混用旧资源，按 `Ctrl+F5`。重复测试记录刷新前需导出。

## CAN 与启动排查

先完成 CAN 板卡驱动及所需设备树配置，确认系统已识别 `can0`。接口需要配置时，先停止运动及占用该接口的程序，再执行：

```bash
sudo bash scripts/setup_can.sh real can0 1000000
```

该命令将接口设为 1 Mbit/s 并重启接口。读取状态可用：

```bash
ip -details -statistics link show can0
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 node list
ros2 control list_controllers
ros2 topic echo /pi_arm/state --once
```

| 现象 | 优先检查 |
| --- | --- |
| 网页无法打开 | HTTP 启动终端、主机 IP、8000 端口及网络 |
| 页面可打开但无法连接 | WebSocket 地址/8765 端口、网关进程；HTTP 与 WS 是两个服务 |
| 六轴掉线或反馈过期 | CAN 接口、供电、驱动占用与启动日志 |
| `CALIBRATION_INVALID` | R10 文件路径、approved、配置指纹和模型快照；见 [配置说明](src/pi_arm_hardware/config/README.md) |
| 有反馈但运动不可用 | 启动模式、使能、示教状态、控制许可、活动任务及关节轨迹控制器（JTC）是否 active |
| 回零看起来很慢 | 同时检查关节速度与输出轴限速，绳索耦合会延长整臂同步时间 |

启动日志在终端，ROS 日志通常在 `~/.ros/log/`，构建和测试日志在工作区 `log/`。零点与参考配置见 [硬件配置](src/pi_arm_hardware/config/README.md)。

## 备份与恢复

版本管理使用 `main` 和工作分支 `refactor/code-organization`。历史重复定位测试 1.0 的恢复标签为 `repeatability-v1.0`（`95b65be`），本轮重构前基线为 `baseline/pre-refactor-20261116`（`7010186`）。

恢复版本前，先停止服务，提交或另行保存未提交修改：

```bash
git fetch origin --tags
git switch --detach repeatability-v1.0
bash scripts/build.sh
```

恢复后按上面的 R10 命令启动，并核对当前标定与实物参考。查看历史代码也可创建独立工作区：

```bash
git worktree add --detach ../pi_arm-repeatability-1.0 repeatability-v1.0
```

返回主分支使用 `git switch main`，继续开发使用 `git switch refactor/code-organization`。新克隆后首次进入工作分支使用 `git switch --track origin/refactor/code-organization`。旧阶段标签 `v0.1.0-stage1`、`v0.2.0-viewer-baseline` 用于历史追溯。

每批改动在工作分支单独提交，相关测试通过后快进合入 `main`，用标签记录恢复节点。未跟踪的本地文件另行备份。开发约定见 [src/README.md](src/README.md)。

| 数据 | 保存位置与恢复方式 |
| --- | --- |
| 源码、配置、接受的标定 | Git；标定数据与 `.cable.yaml` 必须配套 |
| 软件零点 | 启动参数 `software_zero_file` 指定的文件；单独备份，后续启动继续传入同一路径，并核对实物参考 |
| 点位库 | 默认 `~/.local/share/pi_arm/points-real.json` / `points-mock.json`；网页导出备份，停止服务后恢复到相应 `points_file` |
| 重复测试记录 | 当前浏览器内存；关闭、刷新或新测试前导出 JSON/CSV |
| ROM 零点、绳索张紧状态 | 记录实物状态，恢复时在现场核对 |
| `build/`、`install/`、`log/`、`web/model/` | 本机构建产物，停机后按需清理和重建 |

## 开发验证与使用范围

包测试、离线浏览器和 vcan 测试入口见 [scripts/README.md](scripts/README.md)。

绳索模型、机械模型和测量资料见 [技术文档](docs/README.md)。
