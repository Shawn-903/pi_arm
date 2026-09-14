# 03 URDF与Xacro模型

# 1. URDF是什么

URDF 全称为 Unified Robot Description Format，即“统一机器人描述格式”。

它使用 XML 描述机器人，主要包括：

- 机器人由哪些刚体组成；
- 刚体之间通过什么关节连接；
- 关节的旋转轴、位置和范围；
- 机器人在 RViz 中显示什么模型；
- MoveIt 使用什么碰撞模型；
- 仿真或动力学计算需要的质量、质心和惯量；
- `ros2_control` 可以控制哪些关节。

最简化的机器人可以写成：

```
<robot name="demo_robot">
  <link name="base_link"/>

  <link name="link_01"/>

  <joint name="joint_01" type="revolute">
    <parent link="base_link"/>
    <child link="link_01"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14"
           velocity="1.0" effort="10.0"/>
  </joint>
</robot>
```

可以把它理解成：

> URDF 使用 `link` 表示零件，使用 `joint` 表示零件之间的连接关系。
> 

# 2. Xacro是什么

Xacro 是一种“可以生成 URDF 的 XML 模板语言”。

直接写 URDF 时，大量内容需要重复。Xacro 在 XML 的基础上增加了：

- 参数；
- 变量；
- 数学表达式；
- 宏；
- 条件判断；
- 文件包含。

ROS 2 节点一般不直接使用 `.xacro`，而是先执行：

```
Xacro 文件 → 展开参数和宏 → URDF XML → robot_description
```

当前项目的主模型是：

[pi_arm.urdf.xacro](../src/pi_arm_description/urdf/pi_arm.urdf.xacro)

文件开头定义了四个 Xacro 参数：

```
<xacro:arg name="backend" default="mock" />
<xacro:arg name="can_interface" default="can0" />
<xacro:arg name="bitrate" default="1000000" />
<xacro:arg name="cable_kinematics_file" default="" />
```

其含义如下。

| 参数 | 默认值 | 意义 |
| --- | --- | --- |
| `backend` | `mock` | 使用模拟硬件还是实际硬件 |
| `can_interface` | `can0` | 实机通信使用的 SocketCAN 接口 |
| `bitrate` | `1000000` | CAN 配置参数，单位 bit/s |
| `cable_kinematics_file` | 空 | 绳索运动学 YAML 文件路径 |

当前文件虽然使用了 `.xacro` 扩展名，但主要使用的是参数功能，暂时没有大量定义 Xacro 宏。

# 3. Xacro文件组成详解

## Link：机械臂的刚体

每一个 `link` 通常包含三类描述：

```
<link name="link_01">
  <inertial>...</inertial>
  <visual>...</visual>
  <collision>...</collision>
</link>
```

### 3.4.1 Visual

`visual` 决定机器人在 RViz 中看起来是什么样子。

当前项目使用 STL 网格：

```
<visual>
  <geometry>
    <mesh filename="package://pi_arm_description/meshes/link_01.STL"/>
  </geometry>
</visual>
```

其中：

```
package://pi_arm_description/
```

表示从 ROS 2 包 `pi_arm_description` 的安装目录中寻找文件，而不是使用计算机上的绝对路径。

需要注意：

> STL 模型只决定外观，不决定机械臂的关节运动学。
> 

就算 STL 的摆放看起来正确，如果 `joint` 的 `origin` 或 `axis` 写错，机械臂运动仍然会出错。

### 3.4.2 Collision

`collision` 是 MoveIt 进行碰撞检测时使用的几何体。

当前项目没有直接使用复杂 STL 作为主要碰撞体，而是使用圆柱体、长方体等简单几何形状。这样可以：

- 减少碰撞检测计算量；
- 提高路径规划速度；
- 适当扩大碰撞模型，形成安全余量。

因此可能出现：

> RViz 中两个可视模型看起来没有接触，但 MoveIt 已经报告碰撞。
> 

这不一定是错误，可能是因为碰撞体比外观模型更保守。

### 3.4.3 Inertial

`inertial` 描述：

- 质量 `mass`；
- 质心位置 `origin`；
- 转动惯量矩阵 `inertia`。

例如：

```
<inertial>
  <origin xyz="..." rpy="..."/>
  <mass value="..."/>
  <inertia ixx="..." ixy="..." ixz="..."
           iyy="..." iyz="..." izz="..."/>
</inertial>
```

这些参数主要用于：

- Gazebo 等物理仿真；
- 动力学计算；
- 重力补偿；
- 力矩控制。

RViz 只负责显示，不会根据质量和惯量模拟机械臂运动。

## Joint：刚体之间的连接

以 `joint_01` 为例：

```
<joint name="joint_01" type="revolute">
  <origin xyz="0 0 0.3535" rpy="0 0 0" />
  <parent link="base_link" />
  <child link="link_01" />
  <axis xyz="0 0 1" />
  <limit lower="-3.14" upper="3.14"
         effort="20.0" velocity="0.1745329" />
</joint>
```

### 3.5.1 type

常见关节类型有：

| 类型 | 含义 |
| --- | --- |
| `fixed` | 固定连接，不允许运动 |
| `revolute` | 有角度范围的旋转关节 |
| `continuous` | 可以连续旋转的关节 |
| `prismatic` | 直线移动关节 |
| `floating` | 六自由度浮动关节 |

本项目的 `joint_01`～`joint_09` 都是 `revolute`。

`tool0_fixed_joint` 是 `fixed`。

### 3.5.2 parent 和 child

```
<parent link="base_link"/>
<child link="link_01"/>
```

表示：

```
base_link --joint_01--> link_01
```

URDF 必须形成一棵树，不能让一个 `link` 同时拥有两个父关节。

### 3.5.3 origin

```
<origin xyz="0 0 0.3535" rpy="0 0 0"/>
```

它表示关节坐标系相对于父级 `link` 坐标系的位置和姿态。

- `xyz` 单位为米；
- `rpy` 单位为弧度；
- `rpy` 分别表示绕 X、Y、Z 轴的 roll、pitch、yaw。

在关节角为零时，可以近似理解为：

```
父 link 坐标系
    ↓ origin 平移和旋转
关节/子 link 的初始坐标系
```

`origin` 决定“关节位于哪里”。

### 3.5.4 axis

```
<axis xyz="0 0 1"/>
```

表示关节围绕自身关节坐标系的 Z 轴转动。

常见写法：

| axis | 含义 |
| --- | --- |
| `1 0 0` | 绕局部 X 轴 |
| `0 1 0` | 绕局部 Y 轴 |
| `0 0 1` | 绕局部 Z 轴 |
| `0 0 -1` | 绕局部 Z 轴反方向 |

必须注意：

> `axis` 是关节局部坐标系中的方向，不一定是 RViz 世界坐标系中的方向。
> 

如果 `origin` 中存在 `rpy`，局部轴也会随着关节坐标系发生旋转。

### 3.5.5 limit

```
<limit
    lower="-1.57"
    upper="1.57"
    effort="20.0"
    velocity="0.1745329"/>
```

对于旋转关节：

- `lower`：最小角度，单位 rad；
- `upper`：最大角度，单位 rad；
- `velocity`：最大角速度，单位 rad/s；
- `effort`：关节最大力矩语义，通常单位 N·m。

当前速度限制：

\[
0.1745329\ \mathrm{rad/s}\approx10^\circ/\mathrm{s}
\]

这些是软件模型限制，不等于已经通过实机测量确认的绝对安全范围。

## Mimic 从动关节

Mimic 关节表示一个关节的位置由另一个关节的位置决定。

一般形式为：

$q_{\mathrm{mimic}}
=
kq_{\mathrm{source}}+b$

其中：

- $q_{\mathrm{source}}$：主动关节角度；
- $q_{\mathrm{mimic}}$：从动关节角度；
- $k$：`multiplier`；
- $b$：`offset`。

当前模型有三组关系。

[从SolidWorks到URDF](https://app.notion.com/p/SolidWorks-URDF-37ba622c674e8009a151ce248da1e444?pvs=21)