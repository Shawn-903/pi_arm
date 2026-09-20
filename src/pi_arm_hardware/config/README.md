# 标定、坐标与硬件配置

整体启动命令统一在 [根 README](../../../README.md)。当前参考为 [R10](pack_reference_calibration.R10.recorded.yaml)，对应 [绳索模型快照](pack_reference_calibration.R10.recorded.cable.yaml)，标定编号 `PIARM-R1-SN0001-PACKCAL-20260920-R10`，配置指纹 `fnv1a64:7d99c13df4cb0c63`。

## 三种角度与零偏

统一换算为弧度后：

```text
p                 电机协议角，正方向由编码器/驱动定义
raw = d × p / G   原始减速器输出轴角，尚未加模型零偏
b = F(q_pack) - raw_pack
phi = raw + b     相对模型零位的输出轴角
phi = F(q)        绳索正解；q = inverse_F(phi)
p_target = G × d × (F(q_target) - b)
```

实际协议 API 使用度，必须在协议边界处理 deg/rad。`G>0` 为协议角/输出轴角比例，`d=±1` 为安装方向；绳索 `output_directions` 另作用于 F 的整行，不能与安装方向混为一项随意修改。

| CAN 电机 | G | d | 绳索整行方向 | 自身驱动主动关节（还存在 J2 耦合） |
| --- | ---: | ---: | ---: | --- |
| 1 | 36 | +1 | +1 | J1 / joint_01 |
| 2 | 36 | +1 | +1 | J2 / joint_02 |
| 3 | 36 | −1 | −1 | J3 / joint_04 |
| 4 | 6 | −1 | −1 | J5 / joint_06 |
| 5 | 6 | −1 | +1 | J4 / joint_05 |
| 6 | 6 | +1 | −1 | J6 / joint_09 |

`swap_wrist_channels=true` 已纳入正逆解、速度和依赖映射；ROS/网页六轴数组顺序不交换。几何键 `axes.joint_05/joint_06` 仍分别描述 CAN4/CAN5 绳索行。

- 打包姿态 q = `[0, 100.7, -85.6, 0, 0, 0]°`；完成 ROM 标零后 p 应在 0 附近，raw 也接近 0，但 phi 通常不是 0。
- 零位姿态 q = 六个 0，phi = 六个 0；p 通常非 0，不能因回零后协议角不为零就重新标零。
- R10 对应重新张紧后的打包参考，零点的接受不代表完整绳索几何和定位精度标定完成。不得将旧 R06/R08/R09 文件直接用于当前 ROM 零点。

## 文件归属

| 需调整的内容 | 源文件 |
| --- | --- |
| 安装方向、协议比例、关节标准限位、反馈容差 | [pi_arm.urdf.xacro](../../pi_arm_description/urdf/pi_arm.urdf.xacro) |
| 绳索半径、整行方向、J2 耦合、腕部通道、输出轴包络 | [cable_kinematics.yaml](../../pi_arm_description/config/cable_kinematics.yaml) |
| 打包测量与接受状态 | 本目录 `pack_reference_calibration.*.yaml` |
| 只读测量证据/历史参考 | [calibration-history](../../../docs/calibration-history/README.md) |
| 到位容差和稳定时间 | [manager.yaml](../../pi_arm_bringup/config/manager.yaml) |
| 运动默认/最大角速度 | [motion_limits.py](../../pi_arm_websocket/pi_arm_websocket/motion_limits.py)，实际约束同时由 manager/硬件校验 |

本目录中的历史 R06/R08/R09 及模板用于复现，不是自动回退候选。运行必须显式选择当前接受的文件，源模型和标定绑定指纹不匹配时应报错，不能跳过校验。

## 参考重建流程

1. 正常停止服务，支撑机械臂；操作者确认实物打包姿态、绳索张紧和六轴静止。
2. 如确需改变 ROM 零点，通过 [CAN 工具](../../pi_arm_can/README.md) 的显式写入流程记录审计。写零与普通软件标定不同，不作为一般故障恢复步骤。
3. 由现场人员完整断电并重新上电，期间不移动电机。读取多帧新鲜静止协议角，核对六轴接近零；仅 ACK 不足以证明生效。当前不以 0x07 替代此人工流程。
4. 保留实测 q_pack、按 d/G 转换的 raw_pack、采样范围、错误码、时间、模型/映射与标定 ID。`pi_arm_calibration_candidate` 生成候选，参数见该工具 `--help`；候选不自动代表接受。
5. 经现场确认后生成新编号的接受记录和配套 `.cable.yaml`，由生产加载器验证指纹、打包正逆映射与限位。旧测量原样保留，不原地改写。
6. 构建并指定新路径启动；先只读确认，再由操作者在网页做运动验证。旧点位重新核验，不自动改其标定 ID。

只改绳索整行方向时可使用离线重绑定工具，保留原始测量，不连接 CAN、不写 ROM：

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/rebind_cable_directions.py \
  --reference src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml \
  --output src/pi_arm_hardware/config/pack_reference_calibration.R11.recorded.yaml \
  --calibration-id PIARM-R1-SN0001-PACKCAL-R11
```

从仓库根目录执行，输出应使用尚未占用的新编号。该工具拒绝任意几何、耦合、安装映射及限位改动；`--swap-wrist` 仅支持其专门验证的腕部交换与配套包络变化，要求接受的打包腕部角为零。重新装配或 ROM 零点变化必须重新测量。

## 常见诊断

`CALIBRATION_INVALID` 优先核对路径、approved、机器人/关节顺序、模型版本及配置指纹。`ASIN_DOMAIN` 可能来自角度参考、耦合/几何或姿态，不等同于电机故障。J2/J3 反馈容差为 3°，目标仍用标准限位；容差不是允许继续扩大目标的理由。原始数据应保留用于判断，不能修改显示数值掩盖问题。
