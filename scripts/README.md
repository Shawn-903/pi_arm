# 开发、构建与验证脚本

本目录放可重复运行的维护与回归工具。整体启动操作见 [根 README](../README.md)，网页功能见 [web](../web/README.md)。命令默认从仓库根目录执行。

## 工具索引

| 工具 | 作用与影响 |
| --- | --- |
| [install_dependencies.sh](install_dependencies.sh) | 安装 Ubuntu/ROS 依赖，需要 sudo 和已配置的 Jazzy 软件源 |
| [build.sh](build.sh) | source ROS、colcon symlink 构建、导出网页模型 |
| [start_web.sh](start_web.sh) | 正常/mock、`--monitor`、`--commissioning` 启动入口，管理 ROS 与 HTTP 子进程 |
| [setup_can.sh](setup_can.sh) | 配置 real/vcan 接口或通过 down 关闭接口；运行前停止占用该接口的服务 |
| [export_web_model.py](export_web_model.py) | 从 URDF/Xacro 生成网页模型与哈希清单，无损压缩网格 |
| [build_web_vendor.py](build_web_vendor.py) | 下载并校验固定版本依赖，重建前端 vendor，需要网络 |
| [rebind_cable_directions.py](rebind_cable_directions.py) | 根据指定方向/腕部通道生成新的绑定文件，保留原始测量 |
| [vcan_motor_emulator.py](vcan_motor_emulator.py) | 虚拟 CAN 电机模拟器，支持帧跟踪及冻结运动 |

配置物理 CAN 前，先安装板卡驱动和设备树配置，确认系统已识别目标接口。

## 包测试

构建后加载目标工作区环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

先按下表选择覆盖实际风险的最小集合。Python 可选具体测试文件；ament_cmake 包可用 CTest 名称筛选。以下是两种选择方式，按变更使用对应命令：

```bash
python3 -m pytest src/pi_arm_websocket/test/test_protocol.py -q
colcon test --packages-select pi_arm_hardware --ctest-args -R '^test_motion_write_gate$' --event-handlers console_direct+
```

| 修改类型 | 相关检查或测试入口 |
| --- | --- |
| 文档、注释、编辑器配置 | 静态检查差异、相对链接、参数及作用范围；注释修改核对可执行内容一致 |
| CAN 接口配置脚本 | `sudo python3 scripts/test_setup_can.py`，用临时替身记录 ip/modprobe 调用，检查无效参数处理、接口类型和配置顺序 |
| 网关命令、状态或点位逻辑 | 从 [网关测试目录](../src/pi_arm_websocket/test/) 选择对应文件；ROS 调用或共享状态变化时，再覆盖受影响的 mock 路径 |
| 网页布局、诊断、重复运动等单项功能 | 从下方离线网页测试中选择对应脚本；跨页面共用逻辑变化时覆盖相关页面 |
| commissioning 电机测试界面、请求或模拟器冻结运动 | `test_web_axis.py` 普通与 `--stall` 场景，使用独立 commissioning/vcan 服务 |
| 软件标零计算、持久化或任务入口 | 硬件 `test_backend` 中 `SoftwareZero.*` 及相关坐标测试；`test_software_zero_vcan.py` 验证生产后端；任务/网页变化选择客户端和网页专项入口 |
| manager 到位、停止或取消 | [manager 测试目录](../src/pi_arm_manager/test/) 中对应单测；涉及编排、Action 或并发时加 `test_arrival_integration.py` |
| manager 工作线程、租约或退出顺序 | `test_manager_shutdown.py` 的相关场景；共享动作收尾发生变化时加 `test_arrival_integration.py` |
| 硬件写入许可、租约或停止保持 | 对应 `test_motion_write_gate` / `test_backend`；涉及真实后端控制链时加 `test_motion_safety_vcan.py` |
| backend 或硬件插件停用、退出、资源释放 | `test_backend_lifecycle_vcan.py`，覆盖真实插件生命周期、租约门禁和 SocketCAN 资源；按影响补对应停止/断连回归 |
| 硬件插件初始化配置 | `test_system_configuration`，使用 mock 检查关节数量、有效配置导出和非法参数 |
| CAN 断线重连、接口迟到或重建 | `test_can_recovery_vcan.py`，验证反馈恢复、旧授权失效和停止未确认状态保留 |
| 反馈解码或运动状态判定 | 对应 CAN 单测 / `test_motion_detector`；涉及后端反馈链时加 `test_motion_feedback_vcan.py` |
| ROS 接口、构建或依赖 | 重建受影响包及消费者，运行对应单测和受影响集成路径；范围跨包时扩大到依赖包 |
| MoveIt 生命周期或插件加载方式 | `test_moveit_shutdown.py` 的 SIGINT/SIGTERM 路径，以及正常 mock 规划、执行和退出 |
| 网页模型导出或三维显示 | `test_web_viewer.py`；改变模型源时同时检查对应 description / transmission 测试 |

冒烟检查用于快速确认受影响的关键入口，例如启动 mock、连接网关、读取状态并正常退出。按改动选择步骤；涉及停止、写入许可等行为时，使用表中对应测试核验结果。

全量测试用于建立基线，或改动范围确实覆盖全系统时；已有合并、发布要求仍需完成：

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

相关检查通过后结束本批验证；出现新的相关代码、依赖、环境变化或问题时，再补对应检查。优先复用已有测试，为新增行为、已确认缺陷或覆盖缺口补充断言。提交记录注明测试入口、环境和结果。

## 离线网页测试

需要 Python Playwright、Chromium，部分脚本使用 NumPy 计算 FK。依赖安装在独立 Python 虚拟环境中：

```bash
python3 -m venv --system-site-packages /tmp/pi-arm-web-tests
source /tmp/pi-arm-web-tests/bin/activate
pip install playwright numpy
python -m playwright install chromium
```

若缺少 venv 或浏览器系统动态库，先补齐系统依赖。浏览器路径参数见各脚本的 `--help`。三维相关测试需先构建，或 source ROS 后运行 `python3 scripts/export_web_model.py`。

以下脚本通过 WebSocket 拦截或本地合成反馈构造测试场景：

| 脚本 | 覆盖内容 |
| --- | --- |
| [test_web_console.py](test_web_console.py) | 导航、布局、门禁、软件停止与故障恢复控件 |
| [test_web_diagnostics.py](test_web_diagnostics.py) | 故障展示和诊断导出 |
| [test_web_drag_teaching.py](test_web_drag_teaching.py) | 示教按钮、请求及保持状态 |
| [test_web_preview_races.py](test_web_preview_races.py) | 迟到预览、手势取消与竞争 |
| [test_web_speed_limits.py](test_web_speed_limits.py) | 默认 5/上限 10、预设速度和参数拒绝 |
| [test_web_repeat.py](test_web_repeat.py) | A/B 循环、五轮试跑、单调时钟周期统计（排除首轮/中止半轮）、到位等待、异常中止、导出和布局 |
| [test_web_repeat_loading.py](test_web_repeat_loading.py) | 旧脚本缓存复现、升级后的点位下拉同步 |
| [test_web_viewer.py](test_web_viewer.py) | 独立矩阵 FK、mimic、模型资源、显示降级；默认离线，`--mock-url` 另加 mock 只读对照 |

例如：

```bash
python3 scripts/test_web_repeat_loading.py
python3 scripts/test_web_repeat.py
python3 scripts/test_web_speed_limits.py
python3 scripts/test_web_console.py
```

## mock 端到端测试

[MoveIt 退出回归](test_moveit_shutdown.py) 使用已加载工作区中的 mock 规划入口。默认 ROS 域为 91，可通过 `--domain-id` 选择空闲域：

```bash
python3 scripts/test_moveit_shutdown.py
python3 scripts/test_moveit_shutdown.py --signal SIGTERM
```

脚本等待规划器就绪后发送退出信号，以 `move_group` 子进程的退出状态判断结果。

先在独立终端按根 README 启动 `ROS_DOMAIN_ID=81`、HTTP 8001、WS 8766 的 mock。根据各脚本要求，准备新启动、处于零位且未使能的实例。

```bash
python3 scripts/test_web_mock.py --url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_presets.py --url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_control_spaces.py --url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_interactive.py --url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_teaching.py --ws ws://127.0.0.1:8766
python3 scripts/test_web_recovery.py --ws ws://127.0.0.1:8766
```

这些测试发送模拟运动、使能和恢复命令，并核对后端类型。点位测试会写入点位数据，启动 mock 时指定临时 `points_file`。

## vcan 与合成 ROS 验证

单独准备空闲的虚拟接口，再 source 构建环境：

```bash
sudo bash scripts/setup_can.sh vcan vcan0
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/test_motion_safety_vcan.py
python3 scripts/test_reboot_vcan.py
python3 scripts/test_rebind_cable_directions.py
```

[CAN 重连专项检查](test_can_recovery_vcan.py) 需要独立环境且启动前不存在 `vcan0`。脚本以 sudo 创建、断开、重建并清理自己的虚拟接口，后端仍以调用 sudo 的用户运行：

```bash
sudo bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && PYTHONDONTWRITEBYTECODE=1 python3 scripts/test_can_recovery_vcan.py'
```

`test_motion_safety_vcan.py` 和 `test_reboot_vcan.py` 自行启动模拟器并调用生产 RealBackend，使用 vcan0。运行前停止占用该虚拟接口的服务，并依次执行这两个脚本。`test_rebind_cable_directions.py` 是离线文件测试。

[运动反馈回归](test_motion_feedback_vcan.py) 使用单独的 `vcan_motion`，通过虚拟查询/反馈帧核验生产 RealBackend 的静止抖动、零速度读数下的缓慢位移、腕部通道映射、掉线恢复和无效运动学。source 要验证的 install（支持隔离 overlay），再执行：

```bash
sudo ip link add vcan_motion type vcan
sudo ip link set vcan_motion up
python3 scripts/test_motion_feedback_vcan.py
sudo ip link delete vcan_motion
```

与现有服务并行开发时，可在 `/tmp` 指定独立 `colcon --log-base ... build --build-base ... --install-base ...` 编译测试硬件包。正式更新按根 README 停止服务、构建并重新启动。

[测试到位与停止集成](test_arrival_integration.py) 在独立 ROS 域 82 启动生产 manager 与合成硬件/控制器服务：`ROS_DOMAIN_ID=82 python3 scripts/test_arrival_integration.py`。覆盖到位复核、运动/等待停止、Action 取消、迟到响应、保持失败与故障优先级，运行前确认该域空闲。

[manager 退出回归](test_manager_shutdown.py) 使用同类合成服务，在隔离 ROS 域 98 检查退出时的动作终态、租约释放、目标取消和进程退出码。加载待测工作区后，按改动选择 `--case`：`active`、`arming`、`late_accept`、`expired_accept`、`planning`、`preview`、`stop_failure` 或 `idle`。例如：`ROS_DOMAIN_ID=98 python3 scripts/test_manager_shutdown.py --case active --log-dir /tmp/manager-shutdown`。

以下专项测试需先按表中条件准备 vcan 服务：

| 脚本 | 前置条件 |
| --- | --- |
| [test_axis_vcan.py](test_axis_vcan.py) | vcan0 commissioning、模拟器、WS 8766；`--joint` 测耦合关节，`--watchdog` 配合冻结模拟器 |
| [test_normal_motion_vcan.py](test_normal_motion_vcan.py) | 普通 real Backend 绑定 vcan0、模拟器、匹配的虚拟标定，等 JTC active；默认 WS 8766 |
| [test_web_axis.py](test_web_axis.py) | commissioning vcan0、HTTP 8001/WS 8766；`--stall` 配合 `--freeze-motion` |

`test_web_axis.py` 核验电机测试速度默认 5°/s、上限 10°/s，以及请求和导出记录中的实际速度。普通场景检查到达目标后的 0x81 停止确认；`--stall` 场景检查持续心跳、操作员停止保持和断线停止保持。

两个场景分别启动干净的 commissioning 服务和模拟器，再运行下面对应的命令。普通场景使用默认模拟器，停滞场景使用 `vcan_motor_emulator.py vcan0 --freeze-motion`；冻结运动只阻止编码器位置跟随目标，仍返回正确的 A4 应答。

```bash
python3 scripts/test_web_axis.py
python3 scripts/test_web_axis.py --stall
```

## 软件标零专项验证

构建并 source 待测工作区后，在独占的 vcan0 上执行 `python3 scripts/test_software_zero_vcan.py`。它固定 ROS 域 84，启动生产监测节点和已有虚拟电机模拟器，使用临时参考文件，覆盖静止采样、无电机写入、保存失败、反馈过期、活动任务拒绝及程序重启重载。

任务与网页验证使用另一个干净的 commissioning/vcan0 实例，配置临时 `software_zero_file`、虚拟基础标定、HTTP 8001 和 WS 8766。对同一测试实例分别运行：

```bash
python3 scripts/test_software_zero_clients.py
python3 scripts/test_web_software_zero.py
```

前者验证 Python `TaskClient` 和命令行的服务调用与单位转换；后者验证网页确认、返回结果、参考编号，以及操作前后原始编码器读数一致。这两个脚本在 vcan0 测试实例中更新软件参考，零点数据、测试用基础标定和一次性启动工具保存在临时目录。

## 维护规范

脚本说明应列明接口、端口、ROS 域、依赖和写入的数据。自动运动回归使用 mock 或虚拟 CAN；生成记录放在临时目录或指定导出路径。验证记录注明离线、mock、vcan 或现场环境及对应结果。
