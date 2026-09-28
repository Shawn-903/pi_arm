# 开发、构建与验证脚本

本目录放可重复运行的维护与回归工具。整体启动操作见 [根 README](../README.md)，网页功能见 [web](../web/README.md)。命令默认从仓库根目录执行。

## 工具索引

| 工具 | 作用与影响 |
| --- | --- |
| [install_dependencies.sh](install_dependencies.sh) | 安装 Ubuntu/ROS 依赖，需要 sudo 和已配置的 Jazzy 软件源 |
| [build.sh](build.sh) | source ROS、colcon symlink 构建、导出网页模型 |
| [start_web.sh](start_web.sh) | 正常/mock、`--monitor`、`--commissioning` 启动入口，管理 ROS 与 HTTP 子进程 |
| [setup_can.sh](setup_can.sh) | real/vcan/down；修改网络接口，运行中的物理 CAN 不能随意重配 |
| [export_web_model.py](export_web_model.py) | 从 URDF/Xacro 生成网页模型与哈希清单，无损压缩网格 |
| [build_web_vendor.py](build_web_vendor.py) | 固定依赖校验后重建前端 vendor，维护时需网络，正常运行无需使用 |
| [rebind_cable_directions.py](rebind_cable_directions.py) | 受限的离线方向/腕部通道重新绑定，保留原始测量、不写 CAN/ROM |
| [vcan_motor_emulator.py](vcan_motor_emulator.py) | 仅虚拟 CAN 模拟器，支持帧跟踪及冻结运动 |

CAN HAT 驱动/设备树安装不在现有脚本能力内。空的旧安装占位脚本已移除，不能把 `setup_can.sh` 当作驱动安装器。

## 包测试

构建后加载目标工作区环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

局部修改可用 `colcon test --packages-select <包名>`，接口/耦合模型变更需覆盖依赖包。测试范围随修改选择，不因文档变化重复运行实机或全部控制测试。

## 离线网页测试

需要 Python Playwright、Chromium，部分脚本复用 NumPy FK。可在独立 Python 虚拟环境安装，不改变 ROS Python 环境：

```bash
python3 -m venv --system-site-packages /tmp/pi-arm-web-tests
source /tmp/pi-arm-web-tests/bin/activate
pip install playwright numpy
python -m playwright install chromium
```

若缺少 venv 或浏览器系统动态库，先补齐对应系统依赖；不要将临时安装目录加入 Git。可支持 `--browser-path` 的脚本见各自 `--help`。三维相关测试需先构建，或 source ROS 后运行 `python3 scripts/export_web_model.py`。

以下脚本拦截全部 WebSocket 或使用本地合成反馈，不连接实机：

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

先在独立终端按根 README 启动 `ROS_DOMAIN_ID=81`、HTTP 8001、WS 8766 的 mock。根据各脚本要求使用新启动、零位且未使能的实例；不要把有前序任务/使能状态的实例当作干净环境。

```bash
python3 scripts/test_web_mock.py --url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_presets.py --url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_control_spaces.py --url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_interactive.py --url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_teaching.py --ws ws://127.0.0.1:8766
python3 scripts/test_web_recovery.py --ws ws://127.0.0.1:8766
```

这些测试会发模拟运动/使能/恢复命令，并检查后端类型；点位测试还会写入测试点位，启动 mock 时指定临时 `points_file`。不要修改检查使其指向实机。

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

前两个脚本自己建立模拟器并调用生产 RealBackend，固定/核验 vcan0，不需要另启实机服务；应顺序运行，不能与其他驱动或模拟器争用 vcan0。方向重绑定测试是离线文件测试。

[运动反馈回归](test_motion_feedback_vcan.py) 使用单独的 `vcan_motion`，核验生产 RealBackend 的静止抖动、零速度读数下的缓慢位移、腕部通道映射、掉线恢复和无效运动学。它只产生虚拟查询/反馈帧，拒绝非 vcan 接口，并断言没有使能、位置或标零命令。source 要验证的 install（支持隔离 overlay），再执行：

```bash
sudo ip link add vcan_motion type vcan
sudo ip link set vcan_motion up
python3 scripts/test_motion_feedback_vcan.py
sudo ip link delete vcan_motion
```

运行中的实机使用现有 `build/install` 时，可在 `/tmp` 指定独立 `colcon --log-base ... build --build-base ... --install-base ...` 编译测试硬件包；不要覆盖运行中的库。正式加载修复仍按根 README 停止、构建、重新启动。

[测试到位与停止集成](test_arrival_integration.py) 在隔离 ROS 域 82 启动真实 manager 与合成硬件/控制器服务，不开 CAN、不执行真实 MoveIt 轨迹：`ROS_DOMAIN_ID=82 python3 scripts/test_arrival_integration.py`。覆盖到位复核、运动/等待停止、Action 取消、迟到响应、保持失败与故障优先级。不要让其他服务占用这个域。

以下是需要预先配置对应 vcan 服务的专项入口，不是直接对当前实机运行的快捷命令：

| 脚本 | 前置条件 |
| --- | --- |
| [test_axis_vcan.py](test_axis_vcan.py) | vcan0 commissioning、模拟器、WS 8766；`--joint` 测耦合关节，`--watchdog` 配合冻结模拟器 |
| [test_normal_motion_vcan.py](test_normal_motion_vcan.py) | 普通 real Backend 绑定 vcan0、模拟器、匹配的虚拟标定，等 JTC active；默认 WS 8766 |
| [test_web_axis.py](test_web_axis.py) | commissioning vcan0、HTTP 8001/WS 8766；`--stall` 配合 `--freeze-motion` |

专项虚拟标定不能复制到实机运行。`test_web_axis.py` 仍有旧的 1°/s 默认/上限断言，与现有 5/10 设置不一致，需后续更新测试后再作为验收依据；本轮文档整理没有修改该测试逻辑。

## 维护规范

修改脚本时明确硬件/端口/ROS 域、依赖和副作用；运动回归默认拒绝物理 CAN。生成记录用临时目录或操作者指定导出路径，不覆盖实机点位和标定。测试通过记录要说明是离线、mock、vcan 还是现场，不把模拟精度当作实机测量。
