# pi_arm_bringup

总启动包，装配模型、硬件插件、控制器、MoveIt、manager 与 WebSocket。整体启动命令、CAN 配置与端口设置统一见 [根 README](../../README.md)。

## 入口与配置

| 文件 | 作用 |
| --- | --- |
| [bringup.launch.py](launch/bringup.launch.py) | 正常 mock/real 控制链，包含 joint_state_broadcaster、joint_trajectory_controller |
| [monitor.launch.py](launch/monitor.launch.py) | 只读硬件监测基础入口 |
| [monitor_web.launch.py](launch/monitor_web.launch.py) | 只读监测加 WebSocket，不提供运动服务 |
| [commissioning_web.launch.py](launch/commissioning_web.launch.py) | 独立单轴方向测试，不启动正常轨迹链 |
| [controllers.yaml](config/controllers.yaml) | 控制周期 50 Hz、六轴顺序、位置/速度接口及 JTC 容差 |
| [manager.yaml](config/manager.yaml) | 默认到位容差 0.2°、连续稳定 0.5 s、收敛等待 10 s |

常用 launch 参数：`hardware_type`、`can_interface`、`calibration_file`、`allow_real_control`、`use_rviz`、`websocket_port`、`points_file`。不同入口参数以各 launch 的 `DeclareLaunchArgument` 为准。实机写入需显式授权；mock 是正常 bringup 的默认后端。

控制器使用专用 robot_description 话题，按硬件类型/CAN 接口隔离，避免残留全局模型被控制器加载。真实控制状态与 launch 类型不能靠同域中其他显示节点提供的描述判断。

## 维护与验证

配置变更需与 URDF、MoveIt 和 manager 的六轴定义一致。控制器激活完成后才接收正常运动；只见到硬件 READY 不等于 JTC 已激活。`test/test_bringup_launch_safety.py` 检查启动门禁与模型隔离，运行方法见 [测试说明](../../scripts/README.md)。
