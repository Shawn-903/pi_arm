# pi_arm_moveit_config

MoveIt 2 的语义模型、运动规划、逆运动学和执行接口配置包。模型几何来自 [pi_arm_description](../pi_arm_description/README.md)，不单独定义电机方向或绳索零偏。

## 文件与边界

| 文件 | 作用 |
| --- | --- |
| [pi_arm.srdf](config/pi_arm.srdf) | 规划组 `pi_arm`，base_link → tool0；三个 passive/mimic 关节、零位 home 和碰撞排除对 |
| [kinematics.yaml](config/kinematics.yaml) | IK 求解器及搜索配置 |
| [joint_limits.yaml](config/joint_limits.yaml) | 规划速度/加速度限制，需与 URDF/执行约束一致 |
| [ompl_planning.yaml](config/ompl_planning.yaml) | 关节空间规划管线 |
| [pilz_industrial_motion_planner_planning.yaml](config/pilz_industrial_motion_planner_planning.yaml) | Pilz 管线 |
| [pilz_cartesian_limits.yaml](config/pilz_cartesian_limits.yaml) | 笛卡尔速度与加减速约束 |
| [moveit_controllers.yaml](config/moveit_controllers.yaml) | 六轴 JTC 执行接口映射 |
| [move_group.launch.py](launch/move_group.launch.py)、[moveit_rviz.launch.py](launch/moveit_rviz.launch.py) | 规划节点与可视化入口；正常服务由 bringup 装配 |

manager 的 MoveJ 使用规划，MoveL 使用 Pilz LIN；DirectMove、输出轴和预设姿态是另一条受限插值路径，不经 MoveIt 避障。端点 IK 可解不代表 LIN 路径可行；模型碰撞检查也不包含未建模的现场障碍。

SRDF 的排除对包含相邻连杆及共享关节附近永久重叠的简化碰撞包围体。修改结构、碰撞体或工具后应重新检查，而不是通过持续扩大排除列表绕过碰撞失败。当前 tool0 是无工具的法兰中心。

## 验证与维护

常规启动见 [根 README](../../README.md)。独立启动规划/RViz 前加载构建环境并隔离 ROS 域，避免连接错误的控制器。通过 mock 的 `test_web_interactive.py`、`test_web_control_spaces.py` 和 `test_web_presets.py` 检查规划及正常控制集成，参数见 [scripts](../../scripts/README.md)。

树莓派安装的 MoveIt 2.12.4 在退出时，会先卸载控制器插件和默认能力插件，再销毁仍引用这些库的 ROS 回调对象，从而触发段错误。`move_group.launch.py` 针对该版本设置进程局部的 `LD_PRELOAD`，让这两个已安装的插件库保留到进程退出，并保留调用方原有的预加载项。该兼容处理只作用于此规划入口；其他 MoveIt 版本沿用原启动方式，升级后需重新检查退出行为。

退出回归使用独立 ROS 域，只启动 mock 配置的规划进程：`python3 scripts/test_moveit_shutdown.py`；加 `--signal SIGTERM` 检查终止信号路径。脚本核验 `move_group` 子进程正常退出，不能仅凭 `ros2 launch` 返回 0 判断通过。
