# pi_arm_bringup

总启动包，装配模型、硬件插件、控制器、MoveIt、manager 与 WebSocket。整体启动命令、CAN 配置与端口设置统一见 [根 README](../../README.md)。

## 入口与配置

| 文件 | 作用 |
| --- | --- |
| [bringup.launch.py](launch/bringup.launch.py) | 正常 mock/real 控制链，包含 joint_state_broadcaster、joint_trajectory_controller |
| [monitor.launch.py](launch/monitor.launch.py) | 只读硬件监测基础入口 |
| [monitor_web.launch.py](launch/monitor_web.launch.py) | 将只读硬件监测接入 WebSocket，供网页查看反馈与诊断 |
| [commissioning_web.launch.py](launch/commissioning_web.launch.py) | 维护模式：方向测试、软件标零任务服务与 WebSocket |
| [controllers.yaml](config/controllers.yaml) | 控制周期 50 Hz、六轴顺序、位置/速度接口及 JTC 容差 |
| [manager.yaml](config/manager.yaml) | 默认到位容差 0.2°、连续稳定 0.5 s、收敛等待 10 s |

加载 ROS 和工作区环境后，可用 `ros2 launch pi_arm_bringup bringup.launch.py hardware_type:=mock` 启动模拟控制链。根目录的 `scripts/start_web.sh` 会调用相应 launch，并同时启动网页 HTTP 服务。

常用参数如下。`monitor_web` 和 `commissioning_web` 会包含 `monitor.launch.py`，可传入其中声明的硬件参数。查看完整参数及默认值可执行 `ros2 launch pi_arm_bringup bringup.launch.py --show-args`，其他入口替换其中的 launch 文件名。

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

控制器从专用 `robot_description` 话题加载模型，话题按硬件类型和 CAN 接口区分，对应本次启动选择的配置。

## 维护与验证

配置中的关节顺序与 URDF、MoveIt 和 manager 保持一致。启动后可用 `ros2 control list_controllers` 查看控制器是否处于 `active`，正常运动在轨迹控制器激活后开放。`test/test_bringup_launch_safety.py` 检查启动条件与模型隔离，运行方法见 [测试说明](../../scripts/README.md)。
