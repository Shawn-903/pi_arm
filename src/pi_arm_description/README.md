# pi_arm_description

机械臂模型与执行器配置的源包，供 ros2_control、MoveIt、RViz 和网页显示使用。URDF/Xacro 描述连杆几何、关节坐标、限位和硬件映射；STL 提供外观网格，URDF 中的碰撞形状用于规划检查。网页模型由这里的源文件生成。

## 文件

- [urdf/pi_arm.urdf.xacro](urdf/pi_arm.urdf.xacro)：机械结构、mimic、tool0、硬件插件参数。
- [config/cable_kinematics.yaml](config/cable_kinematics.yaml)：model_version 3 的绳索几何、方向、耦合与输出轴包络。
- `meshes/`：原始 STL；模型导出脚本无损压缩并写入 SHA-256 清单。
- [launch/display.launch.py](launch/display.launch.py)：joint_state_publisher_gui + robot_state_publisher/RViz 的离线关节显示。

Xacro 参数包括 `backend`（默认 mock）、`can_interface`、`bitrate`、`cable_kinematics_file`、`calibration_file`、`software_zero_file`、`allow_real_control`，由 bringup 填入。

## 六自由度与坐标

| 网页 | 主动 ROS joint | 标准关节范围 | 机械随动 joint |
| --- | --- | --- | --- |
| J1 | joint_01 | ±180° | 无 |
| J2 | joint_02 | ±100.7° | 无 |
| J3 | joint_04 | ±85.6° | joint_03 = joint_04 |
| J4 | joint_05 | ±30° | joint_08 = −joint_05 |
| J5 | joint_06 | ±60° | joint_07 = joint_06 |
| J6 | joint_09 | ±180° | 无 |

URDF `origin` 定义关节角为 0 时相对父坐标系的位姿，`axis` 定义关节在局部坐标系中的转轴。`visual` 设置显示网格，`collision` 设置碰撞形状。基座坐标系为 `base_link`，控制点 `tool0` 位于法兰中心。

mimic 表示随动关节：它的角度由另一关节按指定比例和偏移计算。整段机构姿态由主动关节和随动关节共同决定；腕部总弯曲角的计算见 [transmission](../pi_arm_transmission/README.md#腕部实机核验)。

电机安装方向、减速比和绳索通道的配置方法见 [硬件配置](../pi_arm_hardware/config/README.md)。

## 使用与验证

加载构建环境后，执行 `ros2 launch pi_arm_description display.launch.py` 打开关节滑块和 RViz，调整滑块查看机械臂姿态。与其他机械臂节点并行运行时设置独立的 `ROS_DOMAIN_ID`。

`test/test_description_configuration.py` 验证配置；`scripts/export_web_model.py` 生成 `web/model/`，由构建/网页启动自动调用；`scripts/test_web_viewer.py` 用独立矩阵 FK 核验各 link 和 mimic。正式操作见 [根 README](../../README.md)，网页显示见 [web](../../web/README.md)。
