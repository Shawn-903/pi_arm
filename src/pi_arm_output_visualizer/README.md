# pi_arm_output_visualizer

用于离线查看输出轴角对应的机械臂姿态。程序通过 `pi_arm_transmission` 的绳索模型计算关节角，并在 RViz 中显示机械臂。可用滑块输入角度，也可通过 ROS 话题发送角度数据。

```text
输出轴滑块 → /pi_arm/model_output_shaft_states
           → pi_arm_transmission 反解
           → /joint_states → robot_state_publisher → RViz
```

## 输入输出

输入为 `sensor_msgs/JointState`，名称固定为 `motor_01_output` 到 `motor_06_output`，单位 rad。输出关节顺序为 `joint_01, joint_02, joint_04, joint_05, joint_06, joint_09`。

输入为加过软件偏移的模型输出轴角：`phi = raw_output + offset`。滑块范围根据 [cable_kinematics.yaml](../pi_arm_description/config/cable_kinematics.yaml) 中的模型参数生成。

每帧按完整的一组角度处理。有轴超限或反解无效（如腕部 asin 定义域不满足）时，节点报告原因，RViz 保持上一帧有效姿态。

## 文件与使用

- [output_shaft_mapper.cpp](src/output_shaft_mapper.cpp)：名称/边界检查与反解。
- [output_shaft_visualizer_node.cpp](src/output_shaft_visualizer_node.cpp)：ROS 订阅/发布。
- [generate_output_shaft_urdf.py](scripts/generate_output_shaft_urdf.py)：按配置生成 GUI 用输出轴模型。
- [output_shaft_visualizer.launch.py](launch/output_shaft_visualizer.launch.py)：GUI、映射、模型及 RViz。

加载构建环境后，在独立 ROS 域运行：

```bash
ROS_DOMAIN_ID=82 ros2 launch pi_arm_output_visualizer output_shaft_visualizer.launch.py
```

默认打开滑块和 RViz，拖动滑块即可观察对应关节姿态。设置 `use_gui:=false` 后，可由外部程序向 `/pi_arm/model_output_shaft_states` 发送完整六轴状态。与其他机械臂节点并行运行时，为本显示程序设置独立的 `ROS_DOMAIN_ID`。

## 验证

`test/test_output_shaft_mapper.cpp` 检查角度映射和无效输入处理，`test/test_visualizer_assets.py` 检查生成资源。运行方式见 [scripts](../../scripts/README.md)。
