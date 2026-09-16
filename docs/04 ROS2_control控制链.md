# 04 ROS2_control控制链

# 1. ROS 2 Control 是什么

`ros2_control` ROS 2 中连接“控制算法”和“机器人硬件”的标准化框架。

可以把它理解为机器人控制系统中的“统一插座”：

- 控制器只面向标准关节接口读写数据；
- 硬件插件负责把标准接口转换成具体设备能够理解的数据；
- Controller Manager 负责装载、连接和周期性运行二者。

![image.png](image.png)

# 2. ROS 2 Control 的核心组成部分

## 2.1 Controller_manager

Controller Manager 是整个 ros2_control 系统的运行中心，通常由 `ros2_control_node` 提供默认进程。
它主要负责：

- 装载和管理硬件组件；
- 装载、配置、激活和停止控制器；
- 检查控制器需要的接口是否存在；
- 决定哪个控制器可以占用哪些 command interfaces；
- 以固定频率执行控制循环；
- 提供 `ros2 control ...` 命令所调用的 ROS 服务。

Controller Manager 本身通常不实现某一种电机控制算法，也不直接解析某一种 CAN 协议。它更像调度器和资源管理者。

## 2.2 Reource Manager

esource Manager 位于 Controller Manager 内部，负责硬件资源抽象。

它会：

- 从 `robot_description` 中解析 `<ros2_control>` 配置；
- 通过 pluginlib 动态加载硬件插件；
- 管理硬件组件的生命周期；
- 收集硬件提供的 state/command interfaces；
- 把接口借给已激活的控制器使用；
- 在控制循环中调用硬件的 `read()` 和 `write()`。

一般使用者不需要直接创建 Resource Manager，但理解它有助于分析“插件为什么没加载”“接口为什么不存在”“控制器为什么无法激活”等问题。

## 2.3 控制器 Controller

控制器读取 state interfaces，根据目标值和当前状态计算输出，然后写入 command interfaces。

常见控制器包括：

- `JointTrajectoryController`：执行多关节时间轨迹；
- position/velocity/effort forward controller：直接转发某类指令；
- diff drive controller：控制差速底盘；
- gripper action controller：控制夹爪；
- 自定义控制器：实现阻抗、力控或特定算法。

控制器也是 pluginlib 插件，可以被 Controller Manager 动态装载和切换。

## 2.4 Broadcaster

Broadcaster 在框架中也按照控制器管理，但它通常只读取状态，不向硬件写运动命令。

最常用的是 `joint_state_broadcaster`：

- 读取各关节 state interfaces；
- 发布 `/joint_states`；
- 也可以发布包含更多接口的 `/dynamic_joint_states`。

因此，`/joint_states` 通常不是硬件插件直接发布的，而是状态广播器把内部 state interfaces 转换成 ROS 消息后发布出来的。

## 2.5 硬件组件 Hardware Component

ros2_control 将硬件组件分为三类：

| 类型 | 典型对象 | 能力 |
| --- | --- | --- |
| `Actuator` | 一个电机、阀门 | 通常表示简单单自由度执行器，可读可写 |
| `Sensor` | 力传感器、IMU | 只提供状态读取 |
| `System` | 多关节机械臂、底盘 | 一个插件管理多个关节和统一通信通道，可实现复杂耦合 |

六轴机械臂通常选择 `SystemInterface`，因为多个关节共享一条总线、统一生命周期，而且执行器与关节之间可能存在耦合关系。

# 3. Interface： 控制器与硬件之间的共同语言

## 3.1 Interface 不是 ROS topic

`command interface` 和 `state interface` 是 Controller Manager 进程内部的标准数据接口。它们通常表现为被框架管理的数值句柄，而不是 DDS 网络上的 topic。

例如：

```
joint_01/position
joint_01/velocity
```

仅凭名称无法判断它是命令还是状态，还要看它属于哪一类接口：

- `command interface`：控制器写，硬件插件读；
- `state interface`：硬件插件写，控制器或 broadcaster 读。

/

## 3.2 Command interface

command interface 表示硬件能够接收的目标量，例如：

- `position`：目标位置；
- `velocity`：目标速度；
- `effort`：目标力矩或力；
- 自定义接口：设备特有的命令量。

“硬件声明 position command interface”只表示它接受位置类目标，不代表系统自动拥有位置闭环。真正的闭环可能在：

- ros2_control 控制器中；
- 电机驱动器内部；
- 硬件插件内部；
- 或多层共同完成。

## 3.3 State interface

state interface 表示硬件测得或估计得到的状态，例如：

- `position`：关节实际位置；
- `velocity`：关节实际速度；
- `effort`：关节力矩或电机电流换算值；
- 温度、故障码等自定义状态。

轨迹控制器通常至少需要 position state interface。没有位置反馈时，上层很难判断机械臂是否真正到达目标。

# 4. **read → update → write 控制循环**

Controller Manager 周期性执行的核心逻辑可以简化为：

```cpp
while (running) {
  hardware.read();       // 读取真实硬件，更新 state interfaces
  controllers.update(); // 根据目标和状态计算 command interfaces
  hardware.write();      // 把 command interfaces 下发到真实硬件
  wait_until_next_cycle();
}
```

### **4.1 read 阶段**

硬件插件在 `read()` 中通常完成：

1. 从总线或驱动器取得最新反馈；
2. 检查时间戳、有效性和故障状态；
3. 把编码器坐标转换为标准关节坐标；
4. 更新 position、velocity 等 state interfaces。

### **4.2 update 阶段**

所有已激活控制器执行 `update()`：

1. 读取本周期 state interfaces；
2. 取得最新参考目标；
3. 进行插值、误差计算或控制律计算；
4. 写入 command interfaces。

### **4.3 write 阶段**

硬件插件在 `write()` 中通常完成：

1. 读取 command interfaces；
2. 检查 NaN、限位和设备状态；
3. 把关节命令转换为执行器命令；
4. 编码设备协议并发送；
5. 在发送失败时返回错误。

### **4.4 update_rate**

如果 Controller Manager 设置：

```yaml
update_rate: 50
```

那么理想控制周期是：

T=150=0.02 s=20 ms*T*=501=0.02 s=20 ms

频率越高不一定越好。必须同时考虑：

- 总线带宽；
- 电机反馈频率；
- 控制计算耗时；
- 操作系统调度抖动；
- 硬件是否支持相应命令刷新率。

实时控制中应尽量避免在周期函数里做阻塞等待、大量动态内存分配、文件读写和慢速日志输出。

### **4.5 Pi Arm 真实位置写入租约**

Controller Manager 即使没有新轨迹，也会周期调用硬件 `write()`；Joint Trajectory Controller 还可能保留一份 idle/hold command。因此“六台电机已 enable”不能等价于“允许发送位置帧”。当前真实 backend 使用三层条件：

1. `control_permitted`：审批标定、运动学、六电机新鲜反馈、无故障及 `allow_real_control` 均有效；
2. manager 已检查并接受一个 `DirectMove`，取得短生命周期运动租约；
3. 武装后 command interface 出现不同于此前 idle/hold 基线的新命令。

只有三层同时成立，`RealBackend::write()` 才把驱动切换到 `CONTROL` 并调用 `CanDriver::enqueue_position_batch()`。单独调用 enable 不创建租约。完成、取消、软件停止、disable/enable/reset、反馈或标定失效会解除租约，使驱动回到 `MONITORING`，并丢弃尚未发送的位置批次。监控轮询仍只使用 `0x92`、`0x9A` 和 `0x9C`。

这一租约解决的是“空闲控制器命令被误发”为 `0xA4` 的软件门控问题，不是鉴权机制，也不构成急停。当前只有 DirectMove 接入租约；其他真实运动 Action 在完成同等生命周期集成前保持失败关闭。

停止原因不能只压缩成一个布尔量。manager 分别保留 Action cancel、`stop_motion`、硬件失能、反馈失联和硬件故障：客户端 cancel 结束为 `CANCELED`；`stop_motion` 中止原运动时 Action 为 `ABORTED`、结果 `success=false/code=3003`；反馈失联、故障和失能分别为 `ABORTED`、`success=false/code=3101/3102/3103`，对应任务状态为 `FAILED`。这些结果语义独立于写入 gate；无论哪种停止原因，gate 都必须先解除并清除待发送位置批次。

`RealBackend::deactivate()`/`cleanup()` 只撤销写入许可并停止软件驱动，不隐式发送电机关闭命令 `0x80`。这避免在析构路径中偷偷加入未经审批的总线写操作，但也意味着进程退出不等于电机失能，必须依靠显式 disable 和独立硬件断能措施。

Jazzy 二进制版 MoveIt 2.12.4 的 `move_group` 还存在上游已报告的 SIGINT/SIGTERM 析构崩溃（moveit2#3680/#3721）。必须把“写入 gate 已先解除”和“整个 launch clean exit”作为两项独立结果记录：前者通过不能掩盖后者的 `-11`。本项目不通过忽略进程退出码、强杀进程或在析构中偷偷发送 CAN 管理命令来规避该问题；升级到含上游修复的包后需要重新做关闭路径验收。
