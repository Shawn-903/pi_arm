# pi_arm_transmission

不依赖 ROS 节点的 C++17 绳索运动学库。输入/输出为六维 rad 或 rad/s，负责关节角 q 与**模型输出轴角** phi 的正逆位置、速度、Jacobian 和输出轴限位；不处理 CAN 协议度或标定零偏。

## 模块与约定

- [cable_kinematics.hpp](include/pi_arm_transmission/cable_kinematics.hpp)、[cable_kinematics.cpp](src/cable_kinematics.cpp)：公共六维接口及公式。
- [config_loader.cpp](src/config_loader.cpp)：YAML 加载与参数验证。
- 源配置：[cable_kinematics.yaml](../pi_arm_description/config/cable_kinematics.yaml)。安装方向和零偏见 [硬件配置](../pi_arm_hardware/config/README.md)。

关节数组顺序为 `[joint_01, joint_02, joint_04, joint_05, joint_06, joint_09]`；输出数组始终按 CAN1–6。同步机构的等效总角 S3=2q3、S4=2q4、S5=2q5 已代入公式，调用者不能再乘/除 2。

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

J3 绳长由两个同步关节半径共同贡献，所以系数是半径之和；腕部采用弦长正弦模型，不是全角度线性比例。第六轴使用两条支路比例的平均值，配置一致性容差 1e-3 接受当前制造尺寸的微小差别，并可调用 `axis6_residual` 检查两支路绳长残差。

反解先求 q1、q2，再求耦合项：

```text
q1 = phi1 / (19.75/16.25)
q2 = phi2
q3 = (B*q2 - phi3) / A
q4 = asin(( phi5 - B*q2) / C)
q5 = asin((-phi4 - B*q2) / C)
q6 = -phi6 - K*q2
```

使用 asin 主分支，必须核对定义域与关节限位；不能用任意裁剪掩盖无效反馈。当前几何假定绳索不可伸长、无间隙，且同步机构遵循 URDF 随动关系；它未证明大角度组合动作下实际绳路无额外耦合。

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

`check_output_position_limits` 用于目标；`check_output_feedback_limits` 将关节容差经耦合传播到反馈输出包络。反解后的关节还必须由上层独立校验，输出轴外包范围不能替代关节限位。

manager 用 `abs(J(0))` 作为线性插值段的保守速度界，并按协议编码后的有效输出轴速度延长时间。例如打包到零位时 CAN3 输出需走约 449.215°，输出轴上限 5°/s 时至少约 89.843 s，关节速度框设置 10°/s 也不会绕过这个约束。

## 腕部实机核验

当前 C 使用 33.94 mm，67.88 mm 的倍增试验已撤销。倍增半径会得到 `asin(sin(q_old)/2)`，并非简单将角度减半，小角度看似接近不能证明大角度正确。

在单腕轴弯曲下，URDF 总弯曲约为主动角两倍；两个腕轴组合不能按角度直接相加。以 link_04 +X 与 tool0 +Z 的三维夹角计算：

| J4 | J5 | 模型总弯曲约值 |
| --- | --- | --- |
| 10° | 0° | 20.000° |
| 0° | 10° | 20.001° |
| 10° | 10° | 28.213° |
| 20° | 20° | 55.983° |
| 30° | 30° | 82.820° |

这些数值只验证模型假设。排查按以下层次进行：

1. 用实际前臂轴与法兰法向夹角定义总弯曲，不用 base 下 RPY 或透视截图投影角代替。核对 R10 参考，采起点/回程以识别零偏变化。
2. 尽量单独改变 J4 或 J5，再做组合动作；保持其他自由度尽量不动。单轴正确而组合不符，检查交叉绳路与闭环几何。
3. 独立测量近端/远端铰链角，检查等幅 mimic 假设、局部轴和零位变换，不能仅凭总弯曲将 mimic 改为 0.5。
4. 对输出轴做独立标记，检查 CAN4/5 的 `abs(delta_protocol/delta_output)=6`；用角度增量消除零偏影响。
5. 保持腕部相对前臂姿态，改变 J2，检查导向耦合；比较正反接近、负载与大角度数据，区分回差/伸长与公式非线性问题。

最低一组数据为起点、小/中/较大角度、回到起点的五个样本，各自保存原始协议角、软件偏移、模型输出角、q、标定 ID 和独立实测总弯曲/方向/负载。网页可导出诊断，但不能把其反解值当作独立实测。原绳路结构示意图不在仓库中，现有 STL 不能确认全部锚点、导向接触与大角度绕线路径。

## 验证

`test/test_cable_kinematics.cpp` 覆盖正反解、速度/Jacobian 有限差分、方向组合、腕部交换、定义域、奇异点和边界；`test/test_config_loader.cpp` 验证 YAML。执行方式见 [scripts](../../scripts/README.md)。测试证明实现内部一致性，物理参数仍需上述现场数据核验。
