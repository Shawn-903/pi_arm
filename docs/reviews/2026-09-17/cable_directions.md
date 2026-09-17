# 绳索输出轴方向现场修正

2026-09-17：用户反馈电机 3、4 在回零过程中方向相反，要求先反向并重启供逐轴验证。

配置：`src/pi_arm_description/config/cable_kinematics.yaml`：

```yaml
output_directions:
  motor_01: 1
  motor_02: 1
  motor_03: -1
  motor_04: -1
  motor_05: 1
  motor_06: 1
```

这是绳索公式到物理输出轴坐标的整体方向：`φ_i = output_directions[i] * F_original_i(q)`。
反向同时影响本关节运动项及 J2 补偿项，位置正解/反解、速度、雅可比和反馈限位容差使用一致的符号。
`+1` 沿用原公式，`-1` 整条公式取反。没有该配置时兼容旧文件，默认全 `+1`。
电机 1～6 对应独立关节 `joint_01 / joint_02 / joint_04 / joint_05 / joint_06 / joint_09`。

不要与另两种方向混淆：URDF 的 `motor_direction` 是编码器协议到物理输出轴的安装方向，仍为
`[1,1,-1,-1,-1,1]`；各轴的 `coupling_sign` 只控制公式内部的 J2 补偿项。
本次根据用户描述反转的是完整输出轴映射。单轴方向正确之后，仍需分别验证 J2 运动时的耦合补偿。

R04 保留已接受的 R03 打包关节角和协议零点测量，不进行新测量、不写 ROM；
启动时重新计算 `b = F_new(q_pack) - raw_pack`。原 R03 及其模型快照保留，供回退和审计。
R04 的 accepted 状态表示用户授权以修正后的方向继续测试，不代表已独立验证整段绳索精度。
当前实际姿态的关节反解可能因修正而变化；不能把它重新标记为打包姿态。

## 后续修改方向

先停止旧服务，再编辑上述 YAML。仅改变方向时，可使用以下离线命令生成下一个版本：

```bash
cd /home/cr/pi_arm
source /opt/ros/jazzy/setup.bash
python3 scripts/rebind_cable_directions.py \
  --reference src/pi_arm_hardware/config/pack_reference_calibration.R06.recorded.yaml \
  --output src/pi_arm_hardware/config/pack_reference_calibration.R07.recorded.yaml \
  --calibration-id PIARM-R1-SN0001-PACKCAL-20260917-R07

bash scripts/start_web.sh hardware_type:=real can_interface:=can0 allow_real_control:=true \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R07.recorded.yaml
```

每次使用上一版作为 reference、新编号作为 output 和 calibration-id。脚本验证旧模型快照与
原标定指纹、当前电机安装映射一致，且只有 `output_directions` 改动，生成新标定和 `.cable.yaml`
快照。它不连接 CAN，不运行机械臂，也不会覆写已有版本。几何/限位/安装映射变化需另外核验。
当前 symlink-install 的配置直接指向源码 YAML，后续只改 YAML 不需要重新编译，但必须更新标定并重启。
原始测量或电机 ROM 零点发生变化时不能用这个脚本代替重新测量。

本次重启使用 R04。默认输出轴限速仍为 5°/s；姿态按钮的关节限速仍为 0.2°/s。
服务启动不会主动回零。测试由用户通过网页发起。

## 本次验证结果

- transmission、description、hardware、manager 单元测试通过；新增测试遍历六轴 64 种方向组合，
  检查位置/速度往返、雅可比数值微分和双支路残差。方向重绑定脚本的 3 项检查通过。
- 虚拟 vcan0 上 21 个生产后端场景通过，包括 CAN3/CAN4 的 A4 目标方向、反馈恢复、
  原始打包参考保持及回零协议目标反向。日志 `/tmp/pi_arm_direction_vcan.log`。
- R04 实机服务已重启：两个控制器 active，标定/运动学有效，READY；未发起实机运动。
  网页快照 `/tmp/pi_arm_direction_real_state.json`，启动日志 `/tmp/pi_arm_direction_real.log`。
- 重启后的 J3 反解约 -86.242°，超出标准 -85.6° 约 0.642°，在既定 3° 反馈容差内，
  网页显示 `JOINT_COMMAND_LIMIT_WARNING`，控制仍可用。这是模型反馈，不是新实测角度。

## 后续 R05：关节 1、2 方向修正

用户进一步确认关节 1、2 也反向，要求编译重启。核查磁盘时两项仍为 +1，已按用户要求保存为 -1。
当前 output_directions 为 `[-1,-1,-1,-1,1,1]`，电机安装方向保持不变。
R05 沿用 R04/R03 原始打包测量，仅重新绑定输出轴方向模型；启动时重新计算软件零偏。
全部 10 个包构建成功，transmission/description 测试及 3 项重绑定检查通过。
实机服务使用 R05，READY，标定/运动学有效，控制许可有效，六轴反馈新鲜。
J2 模型反馈约 100.872°，在标准上限外约 0.172°，属于 3° 反馈容差，网页保留提示。
未下发实机运动。构建、启动、状态日志分别为 `/tmp/pi_arm_R05_build.log`、
`/tmp/pi_arm_R05_real.log`、`/tmp/pi_arm_R05_state.json`。

## 撤回关节 1、2 反向，恢复 R04

用户现场确认关节 1、2 原方向正确，要求恢复上一版本并重启。仅将 motor_01、motor_02
恢复为 +1，绳索配置与 R04 模型快照逐字节一致；保留 motor_03、motor_04 为 -1。
启动重新使用 R04 标定。R05 文件保留为测试历史，不再作为当前启动配置。
安装目录直接链接源 YAML，无需重新编译。未发起实机运动。

## R06 待用户手动启动：第 6 轴整体反向、姿态速度 1°/s

用户已将 motor_06 改为 -1，计划继续测试。当前方向 `[1,1,-1,-1,1,-1]`。
未改动 joint_09 两个分支的 coupling_sign（仍为 +1）。已从 R04 原始测量生成 R06
和模型快照，供该方向配置启动使用；不表示第 6 轴的整体或耦合方向已完成实机确认。
打包/零位共用 `presets.py` 的 `PRESET_JOINT_SPEED_DEG_S = 1.0`，单位关节 °/s，
网页的输出轴限速独立生效。服务已停止，按用户要求不自动重启、未下发运动。
