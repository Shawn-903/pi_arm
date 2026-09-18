# J4 / J5 腕部通道交换（2026-09-18）

操作者观察到同视角下前后示教方向正确、左右方向相反，并授权交换、重启。两组截图的离线 FK 对比支持先交换 J4/J5 的电机绳路对应，物理方向仍需现场复核。

配置入口：`src/pi_arm_description/config/cable_kinematics.yaml`：

```yaml
swap_wrist_channels: true
```

启用后，CAN4 的绳索行作用于 J5（joint_06），CAN5 作用于 J4（joint_05）。保留 CAN ID、协议减速比、安装方向、output_directions 及各电机的 J2 补偿。`axes.joint_05` / `axes.joint_06` 是历史几何参数键，仍描述 motor_04 / motor_05 的绳索行；不能再根据键名推断启用交换后的关节对应。

正解、反解、速度和 Jacobian 统一使用该开关；关节使能/故障/新鲜度聚合、单轴测试的主电机速度上限同步使用新的依赖关系。ROS/网页的 J1–J6 顺序、URDF 和 DH 坐标定义不变。输出轴限位随对应关节的行程交换：CAN4 ±4.892 rad，CAN5 ±4.127 rad；标准关节限位不变。

R08 标定通过 `scripts/rebind_cable_directions.py --swap-wrist` 从 R06 派生。工具只允许切换该开关与交换两项输出轴限位，要求接受的打包 J4、J5 均为零，保留全部原始测量。因打包两轴均为零且 J2 补偿不变，软件零偏也应不变；生产 C++ 加载器仍校验配置指纹。没有执行 CAN 置零、运行、示教切换或运动命令。

启动使用：

```bash
bash scripts/start_web.sh hardware_type:=real can_interface:=can0 allow_real_control:=true \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R08.recorded.yaml
```

恢复旧映射时须同时恢复 R06 的模型快照与标定文件；不能只切换开关后继续使用 R08 指纹。旧点位库保留原数据，R06 记录的反馈点位在使用前应重新示教核验，网页会提示标定 ID 变化。

验证：完整构建、238 项单元测试、27 组生产 RealBackend 的 vcan0 场景通过；包括交换后的 J4/J5 单轴 CAN 目标与速度上限。生产标定加载器确认 R08 有效、旧 R06 对新模型失配、打包正反解及零偏不变；两张截图关节角反解只交换 J4/J5。
