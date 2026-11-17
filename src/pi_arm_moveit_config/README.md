# pi_arm_moveit_config

提供 MoveIt 2 的规划组、逆运动学、运动规划和轨迹执行配置，机械模型来自 [pi_arm_description](../pi_arm_description/README.md)。正常启动时由 bringup 加载，manager 使用这些配置规划关节运动和末端直线运动。

## 配置文件

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

manager 的 MoveJ 调用关节空间规划，MoveL 调用 Pilz LIN 生成末端直线路径。IK 预览求解目标位姿对应的关节角；LIN 预览计算整条直线路径。规划中的碰撞检查使用 URDF 碰撞体和当前规划场景。

SRDF 配置规划组、随动关节和碰撞排除对。排除表记录相邻连杆及共享关节附近固定重叠的碰撞体组合，随机械结构和碰撞模型一同维护。规划坐标系为 `base_link`，控制点 `tool0` 位于法兰中心。

## 验证与维护

常规启动见 [根 README](../../README.md)。独立启动规划/RViz 时，先加载构建环境并设置测试用 ROS 域。mock 的 `test_web_interactive.py`、`test_web_control_spaces.py` 和 `test_web_presets.py` 检查规划及控制集成，参数见 [scripts](../../scripts/README.md)。

树莓派安装的 MoveIt 2.12.4 在退出时，会先卸载控制器插件和默认能力插件，再销毁仍引用这些库的 ROS 回调对象，从而触发段错误。`move_group.launch.py` 针对该版本设置进程局部的 `LD_PRELOAD`，让这两个已安装的插件库保留到进程退出，并保留调用方原有的预加载项。该兼容处理只作用于此规划入口；其他 MoveIt 版本沿用原启动方式，升级后需重新检查退出行为。

退出回归在独立 ROS 域启动 mock 配置的规划进程：`python3 scripts/test_moveit_shutdown.py`；加 `--signal SIGTERM` 检查终止信号路径。测试结果取自 `move_group` 子进程的退出状态。
