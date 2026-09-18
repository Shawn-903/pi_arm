# R09 腕部比例试验（2026-09-18）

现场确认：R08 通道交换后方向正确，但三维腕部总弯曲角约为实机的两倍。
按操作者要求，将 `src/pi_arm_description/config/cable_kinematics.yaml` 中
`axes.joint_05.joint_radius_mm` 和 `axes.joint_06.joint_radius_mm` 从 33.94 改为
67.88 mm。这里暂作有效绳长系数使用，物理几何原因和比例准确性仍待现场确认。

CAN4 → J5、CAN5 → J4 的映射、方向、协议比例 6、J2 耦合项和 URDF mimic 均保留。
统一模型同时作用于位置正解/反解、速度和雅可比，网页不单独缩放显示。
在相同电机反馈下，新腕部单铰链角为 `asin(sin(q_old)/2)`；因此只在小角度时近似减半。

由原关节指令限位推导的输出轴理论包络同步更新：CAN4 ±6.701 rad、CAN5 ±5.172 rad。
这是绳索模型的理论包络，并非实测机械行程。关节指令限位和速度设置不变。

R09 标定及 `.cable.yaml` 快照位于 `src/pi_arm_hardware/config/`。
它保留 R08 原始打包角和原始输出轴测量；由于打包 J4、J5 都是零，且 J2 耦合不变，
软件零偏不变。配置指纹已更新。`approved` 沿用已接受的打包参考，
不表示本次腕部比例已经通过实机验收。未执行编码器置零。

启动：

```bash
bash scripts/start_web.sh hardware_type:=real can_interface:=can0 allow_real_control:=true \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R09.recorded.yaml
```

先通过网页拖拽示教，对比同一视角下实机与模型的总弯曲角，尽量分别测试两个腕部方向，
保持 J2 不变。记录多个弯曲幅度后再判断比例；旧点位保存的是旧模型解算结果，
需要重新确认后再使用。

回退：停止服务，把 R08 的 `.cable.yaml` 快照复制回上述活动配置，
再使用 `pack_reference_calibration.R08.recorded.yaml` 启动。
仅替换标定路径而保留新模型会因配置指纹不同被拒绝。

软件验证：10 个配置/输出轴模型测试通过；生产 C++ 加载器验证新指纹、旧配置失配拒绝、
打包零偏不变，以及 27 组姿态的位置/速度正逆解和输出限位；
27 项生产后端虚拟 CAN 场景通过，包括新比例下的目标角和速度上限。
以上不替代实机比例验收。
