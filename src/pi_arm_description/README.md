# pi_arm_description

机械臂模型与执行器配置的源包。URDF/Xacro 描述连杆几何、关节坐标、限位和 ros2_control 映射；STL 用于显示，碰撞形状由 URDF 单独定义。网页模型从这里生成，不维护另一套装配关系。

## 文件

- [urdf/pi_arm.urdf.xacro](urdf/pi_arm.urdf.xacro)：机械结构、mimic、tool0、硬件插件参数。
- [config/cable_kinematics.yaml](config/cable_kinematics.yaml)：model_version 3 的绳索几何、方向、耦合与输出轴包络。
- `meshes/`：原始 STL；模型导出脚本无损压缩并写入 SHA-256 清单。
- [launch/display.launch.py](launch/display.launch.py)：joint_state_publisher_gui + robot_state_publisher/RViz 的离线关节显示。

Xacro 参数包括 `backend`（默认 mock）、`can_interface`、`bitrate`、`cable_kinematics_file`、`calibration_file`、`allow_real_control`，由 bringup 填入。

## 六自由度与坐标

| 网页 | 主动 ROS joint | 标准关节范围 | 机械随动 joint |
| --- | --- | --- | --- |
| J1 | joint_01 | ±180° | 无 |
| J2 | joint_02 | ±100.7° | 无 |
| J3 | joint_04 | ±85.6° | joint_03 = joint_04 |
| J4 | joint_05 | ±30° | joint_08 = −joint_05 |
| J5 | joint_06 | ±60° | joint_07 = joint_06 |
| J6 | joint_09 | ±180° | 无 |

URDF `origin` 定义父到关节零位变换，`axis` 是该局部坐标系中的轴，不能按屏幕左右直接判正负。`visual` 的网格坐标与 `collision` 的碰撞形状也不能混用。基座为 `base_link`，当前控制点为法兰中心 `tool0`，没有附加工具坐标偏置。

mimic 是运动学假设，不是独立电机指令。尤其 J3/J4/J5 的主动关节角与整段机构总弯曲角不能混为同一个量。腕部总弯曲核验见 [transmission](../pi_arm_transmission/README.md#腕部实机核验)。

协议安装方向/比值与绳索通道的区别见 [硬件配置](../pi_arm_hardware/config/README.md)。仅修改模型显示方向而不核验实物，可能掩盖真正的绳索映射问题。

## 使用与验证

加载构建环境后，独立显示可执行 `ros2 launch pi_arm_description display.launch.py`；与实机并行时使用独立 ROS_DOMAIN_ID，避免 `/joint_states` 和模型描述冲突。该 GUI 不控制电机。

`test/test_description_configuration.py` 验证配置；`scripts/export_web_model.py` 生成 `web/model/`，由构建/网页启动自动调用；`scripts/test_web_viewer.py` 用独立矩阵 FK 核验各 link 和 mimic。正式操作见 [根 README](../../README.md)，网页显示见 [web](../../web/README.md)。
