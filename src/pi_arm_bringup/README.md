# pi_arm_bringup

总启动包，装配模型、硬件插件、控制器、MoveIt、manager 与 WebSocket。整体启动命令、CAN 配置与端口设置统一见 [根 README](../../README.md)。

## 入口与配置

| 文件 | 作用 |
| --- | --- |
| [bringup.launch.py](launch/bringup.launch.py) | 正常 mock/real 控制链，包含 joint_state_broadcaster、joint_trajectory_controller |
| [monitor.launch.py](launch/monitor.launch.py) | 只读硬件监测基础入口 |
| [monitor_web.launch.py](launch/monitor_web.launch.py) | 只读监测加 WebSocket，不提供运动服务 |
| [commissioning_web.launch.py](launch/commissioning_web.launch.py) | 维护模式：方向测试、软件标零任务服务与 WebSocket |
| [controllers.yaml](config/controllers.yaml) | 控制周期 50 Hz、六轴顺序、位置/速度接口及 JTC 容差 |
| [manager.yaml](config/manager.yaml) | 默认到位容差 0.2°、连续稳定 0.5 s、收敛等待 10 s |

常用参数如下。`monitor_web` 和 `commissioning_web` 会包含 `monitor.launch.py`，可传入其中声明的硬件参数；各入口的完整参数以 `DeclareLaunchArgument` 为准。

| 参数 | 用途与适用入口 |
| --- | --- |
| `hardware_type` | 正常 bringup 的后端类型，默认 `mock`；`real` 使用实机后端 |
| `can_interface` | 实机或监测使用的 CAN 接口，默认 `can0` |
| `calibration_file` | 基础标定文件，供硬件坐标换算和参考核验使用 |
| `software_zero_file` | 持久化软件零点文件的绝对路径；默认空，已采集零点后需在后续启动中继续指定 |
| `allow_real_control` | 正常 bringup 的实机写入许可，需显式设置为 `true` |
| `use_rviz` | 正常 bringup 是否打开 RViz |
| `websocket_port` | 正常 bringup 和 commissioning 的 WebSocket 端口，默认 `8765` |
| `points_file` | 网关点位库路径；正常 bringup 和两个 web 入口均可指定 |

正常模式装配 manager 和轨迹控制器；只读与维护模式使用独立监测节点。维护模式的方向测试和软件标零通过各自服务执行。

控制器使用专用 robot_description 话题，按硬件类型/CAN 接口隔离，避免残留全局模型被控制器加载。真实控制状态与 launch 类型不能靠同域中其他显示节点提供的描述判断。

## 维护与验证

配置变更需与 URDF、MoveIt 和 manager 的六轴定义一致。控制器激活完成后才接收正常运动；只见到硬件 READY 不等于 JTC 已激活。`test/test_bringup_launch_safety.py` 检查启动门禁与模型隔离，运行方法见 [测试说明](../../scripts/README.md)。
