# Pi Arm

树莓派六自由度绳索机械臂，目标环境为 Ubuntu 24.04 ARM64 / ROS 2 Jazzy。使用 SocketCAN、ros2_control、MoveIt 2 和网页控制台，支持模拟、只读诊断、单轴方向测试及实机运动。

恢复基线：分支 **`重复定位精度测试1.0`**，提交 **`95b65be`**。当前开发分支：`feature/repeatability-test-next`。

## 文档入口

| 需求 | 文档 |
| --- | --- |
| 安装、启动、停止、恢复版本 | 本文件 |
| 网页操作、三维预览、示教、重复定位测试 | [web/README.md](web/README.md) |
| 控制架构、包索引、开发约定 | [src/README.md](src/README.md) |
| 构建、测试脚本与隔离测试环境 | [scripts/README.md](scripts/README.md) |
| CAN 原始协议、标定证据、已知限制 | [docs/README.md](docs/README.md) |
| 当前标定、坐标换算及配置维护 | [硬件配置 README](src/pi_arm_hardware/config/README.md) |

## 安装与构建

先安装 ROS 2 Jazzy 并配置其软件源，再在工作区根目录执行：

```bash
cd /home/cr/pi_arm
bash scripts/install_dependencies.sh
bash scripts/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

依赖脚本会使用 sudo 安装系统包。构建采用 `colcon --symlink-install`，同时从源 Xacro 导出网页模型。日常重新构建只需 `bash scripts/build.sh`；静态网页文件修改无需编译，浏览器刷新即可加载。

## 选择启动模式

同一 CAN 接口只运行一个驱动实例。以下各模式择一启动，切换前先停止上一实例。`start_web.sh` 启动 ROS 服务和 HTTP 服务；默认无参数为 mock，不会自动使能或执行运动。

### 实机正常控制

供电、CAN 硬件和接口就绪后：

```bash
bash scripts/start_web.sh \
  hardware_type:=real can_interface:=can0 allow_real_control:=true \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml
```

当前参考为 R10，打包关节角 `[0, 100.7, -85.6, 0, 0, 0]°`。R10 绑定当前 ROM 零点与绳索配置，不应在重新装配、重新标零后直接复用。首次运行先核对网页中的标定、六轴反馈及控制许可。

### 只读监测

```bash
bash scripts/start_web.sh --monitor can_interface:=can0 \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml
```

只读取反馈，不启动正常轨迹控制链，不提供使能、运动或故障恢复写入。标定无效时仍可查看原始电机数据。

### 单轴方向测试

```bash
bash scripts/start_web.sh --commissioning can_interface:=can0 \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml
```

仅提供调试管理与单轴测试接口；不会同时启动正常 manager/JTC 运动链。网页操作见 [方向测试](web/README.md#方向测试)。

### 模拟运行

```bash
bash scripts/start_web.sh hardware_type:=mock
```

与正在运行的实机并行开发时，应使用独立 ROS 域及端口：

```bash
ROS_DOMAIN_ID=81 PI_ARM_WEB_PORT=8001 bash scripts/start_web.sh \
  hardware_type:=mock websocket_port:=8766 \
  points_file:=/tmp/pi-arm-mock-points.json
```

对应网页：`http://树莓派IP:8001/control.html?ws=ws://树莓派IP:8766`。mock 不连接物理 CAN；虚拟 CAN 的准备与测试见 [scripts/README.md](scripts/README.md)。

## 打开网页与停止

默认网页为 `http://树莓派IP:8000/control.html`，WebSocket 为 `ws://树莓派IP:8765`。在网页点击“连接”；浏览器不会自动使能。三维模型须通过 HTTP 打开，不能只双击本地 HTML。操作说明集中在 [web/README.md](web/README.md)。

HTTP 端口由环境变量 `PI_ARM_WEB_PORT` 指定，WebSocket 端口由 launch 参数 `websocket_port` 指定；点位文件可用 `points_file` 指定。

- **停止运动**：网页软件停止按钮，取消当前任务并保持当前位置；不是物理急停。故障/失联时不能保证保持，需查看停止结果。
- **停止程序**：先结束测试/运动并支撑机械臂，再在启动终端按 `Ctrl+C`；服务退出会释放位置闭环，区别于网页停止并保持。
- **更新后端**：停止旧服务，构建，再按上述命令启动；不要在运行中替换后端或并行占用同一 CAN。
- **更新前端**：刷新网页。关联脚本有版本化 URL；若界面混用旧资源，按 `Ctrl+F5`。重复测试记录刷新前需导出。

## CAN 与启动排查

CAN 设备需先由系统识别为 `can0`。接口尚未配置且没有程序占用时执行：

```bash
sudo bash scripts/setup_can.sh real can0 1000000
```

这会重配并重启接口，不能在运动中执行；它不负责安装 CAN HAT 驱动或修改设备树。读取状态可用：

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
| 有反馈但运动不可用 | 启动模式、使能、示教状态、控制许可、活动任务及 JTC 是否 active |
| 回零看起来很慢 | 同时检查关节速度与输出轴限速，绳索耦合会延长整臂同步时间 |

启动日志在终端，ROS 日志通常在 `~/.ros/log/`，构建/测试日志在工作区 `log/`。不要靠重新置零消除诊断错误，ROM 标零与人工重新上电流程见 [硬件配置](src/pi_arm_hardware/config/README.md)。

## 备份与恢复

代码恢复基线已推送到 Git。先停止当前服务，提交或另行保存未提交修改，再恢复：

```bash
git fetch origin
# 本地分支已存在时：
git switch '重复定位精度测试1.0'
# 新克隆仓库尚无本地分支时，改用：
# git switch --track 'origin/重复定位精度测试1.0'
bash scripts/build.sh
```

随后使用上面的 R10 实机启动命令。只检查历史代码、不切换当前工作区，可用：

```bash
git worktree add --detach ../pi_arm-repeatability-1.0 '重复定位精度测试1.0'
```

继续开发使用 `git switch feature/repeatability-test-next`。旧阶段标签 `v0.1.0-stage1`、`v0.2.0-viewer-baseline` 保留用于追溯，不能据此认为其旧标定适用于当前实机。

| 数据 | 保存位置与恢复方式 |
| --- | --- |
| 源码、配置、接受的标定 | Git；标定数据与 `.cable.yaml` 必须配套 |
| 点位库 | 默认 `~/.local/share/pi_arm/points-real.json` / `points-mock.json`；网页导出备份，停止服务后恢复到相应 `points_file` |
| 重复测试记录 | 当前浏览器内存；关闭、刷新或新测试前导出 JSON/CSV |
| ROM 零点、绳索张紧状态 | 实物状态，不能通过 Git 回滚恢复 |
| `build/`、`install/`、`log/`、`web/model/` | 本机产物，不纳入 Git；按需重建，运行期间不清除 |

## 开发验证与使用范围

包测试、离线浏览器和 vcan 测试入口见 [scripts/README.md](scripts/README.md)，开发约定见 [src/README.md](src/README.md)。

关节反馈来自编码器和绳索模型，不是独立关节测量。DirectMove、输出轴和预设姿态不经过 MoveIt 避障；笛卡尔 LIN 仅检查已建模场景。普通轨迹不因网页断线自动停止；重复循环取消后续段，但停止请求不保证送达。当前无用户鉴权/TLS，部署在可信隔离网络。腕部大角度模型与实机精度仍需实测核验，详见 [技术资料与已知限制](docs/README.md)。
