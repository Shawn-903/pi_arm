# pi_arm_transmission

C++17 绳索运动学库，可由 ROS 节点或独立程序调用。位置正解把关节角 `q` 换算为**模型输出轴角** `phi`，反解则由 `phi` 求 `q`；还提供速度换算、雅可比矩阵（Jacobian）和输出轴限位检查。位置数组使用 rad，速度数组使用 rad/s；电机协议角和零偏由上层先换算成模型输出轴角再传入。

## 模块与约定

- [cable_kinematics.hpp](include/pi_arm_transmission/cable_kinematics.hpp)、[cable_kinematics.cpp](src/cable_kinematics.cpp)：公共六维接口及公式。
- [config_loader.cpp](src/config_loader.cpp)：YAML 加载与参数验证。
- 源配置：[cable_kinematics.yaml](../pi_arm_description/config/cable_kinematics.yaml)。安装方向和零偏见 [硬件配置](../pi_arm_hardware/config/README.md)。

关节数组顺序为 `[joint_01, joint_02, joint_04, joint_05, joint_06, joint_09]`；输出数组按 CAN1–6。接口中的 q3/q4/q5 使用主动关节角，同步机构的等效总角 S3=2q3、S4=2q4、S5=2q5 已包含在公式内。

`output_directions` 乘整条绳索方程，含其 J2 补偿项；`coupling_sign` 只影响该补偿项。`swap_wrist_channels=true` 时 CAN4 行使用 q5，CAN5 行使用 q4；YAML 中 joint_05/joint_06 几何键仍表示原 CAN4/CAN5 行。

## 当前生产模型

以下为当前方向、通道和半径代入后的公式，所有角度用 rad；几何半径单位 mm，比例无量纲：

```text
A = (28.5 + 23.25) / 16.25
B = 28.5 / 16.25
C = 33.94 / 16.25
K = (20.25/12.25 + 28.5/17.25) / 2

phi1 = (19.75/16.25) * q1
phi2 = q2
phi3 = -A*q3 + B*q2
phi4 = -C*sin(q5) - B*q2
phi5 =  C*sin(q4) + B*q2
phi6 = -q6 - K*q2
```

J3 绳长由两个同步关节半径共同贡献，所以系数是半径之和；腕部采用弦长正弦模型。第六轴使用两条支路比例的平均值，配置一致性容差 1e-3 接受当前制造尺寸的微小差别，并可调用 `axis6_residual` 检查两支路绳长残差。

反解先求 q1、q2，再求耦合项：

```text
q1 = phi1 / (19.75/16.25)
q2 = phi2
q3 = (B*q2 - phi3) / A
q4 = asin(( phi5 - B*q2) / C)
q5 = asin((-phi4 - B*q2) / C)
q6 = -phi6 - K*q2
```

反解使用 asin 主分支，并检查定义域与关节限位。模型按定长绳索、无间隙传动和 URDF 随动关系计算。

速度关系为 `phi_dot = J(q) * q_dot`。Jacobian 的非零项：

```text
J11 = 19.75/16.25; J22 = 1
J32 = B;          J33 = -A
J42 = -B;         J45 = -C*cos(q5)
J52 = B;          J54 =  C*cos(q4)
J62 = -K;         J66 = -1
```

速度反解按位置反解顺序扣除 J2 项，再除自身增益；cos 接近零时返回奇异诊断。非有限输入、输出轴超限、asin 定义域与配置错误都应显式上报。

## 限位与轨迹时间

`check_output_position_limits` 检查目标输出轴角；`check_output_feedback_limits` 将关节容差经耦合传播到反馈输出包络。调用方在反解后继续检查各关节限位。

manager 用 `abs(J(0))` 作为线性插值段的保守速度界，并按协议编码后的有效输出轴速度延长时间。例如打包到零位时 CAN3 输出需走约 449.215°，输出轴上限 5°/s 时至少需要约 89.843 s，整段轨迹按该速度约束同步执行。

## 腕部实机核验

当前腕部几何半径为 33.94 mm，对应上式 `C = 33.94 / 16.25`。

单腕轴弯曲时，URDF 总弯曲约为主动角两倍；组合弯曲通过三维姿态计算。以 link_04 +X 与 tool0 +Z 的夹角为例：

| J4 | J5 | 模型总弯曲约值 |
| --- | --- | --- |
| 10° | 0° | 20.000° |
| 0° | 10° | 20.001° |
| 10° | 10° | 28.213° |
| 20° | 20° | 55.983° |
| 30° | 30° | 82.820° |

上表为模型计算值。与实物对照时，按以下步骤采集和分析：

1. 测量实际前臂轴与法兰法向的夹角，作为总弯曲角。核对 R10 参考，采集起点和回程数据以识别零偏变化。
2. 尽量单独改变 J4 或 J5，再做组合动作；保持其他自由度尽量不动。单轴正确而组合不符，检查交叉绳路与闭环几何。
3. 分别测量近端和远端铰链角，核对等幅 mimic 关系、局部转轴和零位变换。
4. 对输出轴做独立标记，检查 CAN4/5 的 `abs(delta_protocol/delta_output)=6`；用角度增量消除零偏影响。
5. 保持腕部相对前臂姿态，改变 J2，检查导向耦合；比较正反接近、负载与大角度数据，区分回差/伸长与公式非线性问题。

一组基础数据包含起点、小/中/较大角度和回到起点共五个样本。每个样本保存原始协议角、软件偏移、模型输出角、q、标定 ID，以及外部测得的总弯曲角、方向和负载。软件数据通过网页诊断导出，现场测量单独记录，按同一采样点对应保存。

## 验证

`test/test_cable_kinematics.cpp` 覆盖正反解、速度/Jacobian 有限差分、方向组合、腕部交换、定义域、奇异点和边界；`test/test_config_loader.cpp` 验证 YAML。执行方式见 [scripts](../../scripts/README.md)。
