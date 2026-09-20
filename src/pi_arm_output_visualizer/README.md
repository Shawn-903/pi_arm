# pi_arm_output_visualizer

输出轴角驱动的离线 RViz 显示包。将六个**模型输出轴角**经生产绳索反解转换为主动关节状态，不打开 SocketCAN、不控制电机。

```text
输出轴滑块 → /pi_arm/model_output_shaft_states
           → pi_arm_transmission 反解
           → /joint_states → robot_state_publisher → RViz
```

## 输入输出

输入为 `sensor_msgs/JointState`，名称固定为 `motor_01_output` 到 `motor_06_output`，单位 rad。输出关节顺序为 `joint_01, joint_02, joint_04, joint_05, joint_06, joint_09`。

输入必须是加过软件偏移后的 model_output：`phi = raw_output + offset`。不能直接发布 ROM 相对角或协议角。滑块范围从 [cable_kinematics.yaml](../pi_arm_description/config/cable_kinematics.yaml) 生成，是理论输出包络而非实机独立轴可达保证。

整组输入有轴超限或反解无效（如腕部 asin 定义域不满足）时拒绝整帧，保持最后有效 RViz 姿态；不独立裁剪某个电机来掩盖问题。

## 文件与使用

- [output_shaft_mapper.cpp](src/output_shaft_mapper.cpp)：名称/边界检查与反解。
- [output_shaft_visualizer_node.cpp](src/output_shaft_visualizer_node.cpp)：ROS 订阅/发布。
- [generate_output_shaft_urdf.py](scripts/generate_output_shaft_urdf.py)：按配置生成 GUI 用输出轴模型。
- [output_shaft_visualizer.launch.py](launch/output_shaft_visualizer.launch.py)：GUI、映射、模型及 RViz。

加载构建环境后，在独立 ROS 域运行：

```bash
ROS_DOMAIN_ID=82 ros2 launch pi_arm_output_visualizer output_shaft_visualizer.launch.py
```

可设置 `use_gui:=false`，由外部只读仿真源发送完整六轴状态。不要与实机或 description/display GUI 在同一域同时发布 `/joint_states`。

## 验证

`test/test_output_shaft_mapper.cpp` 检查映射与拒绝行为，`test/test_visualizer_assets.py` 检查生成资源，运行见 [scripts](../../scripts/README.md)。这套显示用于核验软件映射，不证明实际方向、绳索张力或可动范围。
