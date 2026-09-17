# 交互控制与到位精度 · 2026-09-17

存档：`v0.2.0-viewer-baseline` / `85974a2`。到位修复独立提交：`cab6e72`。
开发分支：`feature/motion-accuracy-and-interactive-preview`。

## 交互方式

1. **单轴滑块**：每张关节卡下方可拖动目标角。默认只更新输入框和青绿色目标模型。
   可使用键盘方向键微调；仍可使用原“运动”按钮提交。
2. **输出轴目标反解**：填写六个模型输出角，点击“输出轴反解 / 路径”。后端使用当前绳索配置反解；显示六轴关节目标，悬停目标数值可看对应输出轴角。
   超出反解定义域、标准关节或输出轴限位时拒绝，并隐藏旧预览。
3. **笛卡尔 IK**：在独立的「笛卡尔控制」页签输入 base_link 下的 XYZ（mm）、RPY（°），点击“笛卡尔 IK”。
   控制点为 tool0 法兰中心，采用与执行相同的 RPY→四元数转换。IK 使用当前关节反馈作种子，检查模型场景中的端点碰撞。
   一个端点存在 IK 解不代表直线路径可达。
4. **路径**：“关节路径预览”读取当前单关节面板的六轴目标，或当前激活的多关节面板目标。
   关节/输出轴路径与 DirectMove 相同，为位置线性插值，不进行碰撞规划；“LIN 路径规划”使用与 MoveL 相同的 Pilz LIN 管线。
   橙线显示 tool0 路径，青绿色模型可拖动进度条查看。回放压缩为约 10 秒，旁边显示估计的轨迹时间；不是实时运动播放。
5. **拖动模型**：选择连杆或 J1–J6，将“鼠标操作”切换为“拖动关节”，在模型区域左右拖动；向右增加目标角，0.2°/像素。
   点击可识别的连杆会切换到对应关节。当前实现为选中关节的目标拖动，不是末端六自由度姿态手柄。
6. **松手执行**：勾选后，滑块/模型每个完整手势最多发送一个单关节绝对目标，使用该轴速度框和全局输出轴限速。
   后端仍解析其余关节保持目标、核验限位、任务互斥、控制许可并走正常 JTC 链路。
   取消、Esc、断线、失焦、页面隐藏、切换工作区、停止/下使能都会取消手势；连接变化不保留勾选。

每次编辑目标/速度、起点变化超过 0.2°、开始运动或反馈失效，旧路径作废；迟到响应不能恢复作废预览。
预览不使能、不获取运动许可、不创建运动任务。真正执行仍使用原有运动按钮，从最新反馈重新生成轨迹；显示采样点不作为可执行轨迹上传。

只读诊断 / commissioning 启动没有正常 manager，仍可使用原三维显示和关节姿态叠加，不提供新后端 IK/路径服务。
正常控制模式的电机未使能时，只要反馈有效静止，也可以计算预览。新前端连接旧后端时，新预览按钮、滑块和模型拖动按能力字段禁用；原控制仍可使用。

## 核心实现

- `PreviewMotion.srv`：只读服务 `/pi_arm/preview_motion`，mode 为 `joint/output/ik/lin`，ROS 使用 SI 单位。
- manager 复用 `CableKinematics`、URDF 标准限位、反馈容差和输出轴编码限速。IK 请求 MoveIt `/compute_ik`，LIN 仅调用 `plan()`。
- 计算与运动目标受同一互斥准入保护；规划过程中不持有停止请求的互斥锁。
- 返回起点、目标、FK、201 个位置线性轨迹显示采样及 TCP 点；IK 只返回端点。
- WebSocket `preview_motion`：角度 °、位置 mm，返回 TCP 路径使用 m；能力字段 `preview_modes`。
- `interactive.js` 负责手势与预览失效管理，`viewer.js` 负责渲染，`control.js` 保留唯一传输和运动门禁。

## 软件验收

开发构建与 ROS_DOMAIN_ID=81 mock、82 合成反馈均与实机域 0 隔离。
实机服务未重启，未向物理 CAN 或实机笛卡尔执行接口发指令。
独立构建 10 个包通过；colcon **193 项测试，0 失败**。下列脚本另行验证。

- `test_arrival_integration.py`：假控制器提前成功但位置未到、连续稳定收敛、停止后漂移。
- `test_web_interactive.py`：拒绝真实硬件；输出轴反解数值、端点 FK、路径回放、IK/LIN、多关节 mock 运动、滑块/模型拖动及取消门禁。
- `test_web_preview_races.py`：迟到响应、复制式参数更新、重叠手势归属、停止后的按键重复、只读和过期反馈门禁。
- `test_web_viewer.py`：独立 URDF FK、随动关系、目标模型隔离、限位、失效显示、响应式布局、WebGL 降级。
- `test_web_console.py` / `test_web_mock.py`：原网页控制回归。
- `src/pi_arm_websocket/test/test_preview.py`：单位、参数拒绝、无运动动作及 commissioning 隔离。

软件 WebGL 压力测试曾触发反馈新鲜度撤销预览；保留 1.5 s 门禁，不放宽超时，反馈恢复后重新计算 LIN / 回放通过。不可见画布暂停渲染；mock 脚本仅对这一明确的撤销状态做有限重试。

截图：[`interactive-desktop.png`](screenshots/interactive-desktop.png)、[`interactive-mobile.png`](screenshots/interactive-mobile.png)，均为 MOCK 规划预览。
现存系统 MoveIt 退出时段错误仍可能出现；这次未宣称修复，不能据此认定运动运行时失败。

## 实机切换与待确认

当前正在运行的服务仍是旧安装版本，必须先由操作者停止原启动终端（Ctrl+C），再编译并启动：

```bash
cd /home/cr/pi_arm
bash scripts/build.sh
bash scripts/start_web.sh \
  hardware_type:=real can_interface:=can0 allow_real_control:=true \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R06.recorded.yaml
```

浏览器打开 `http://树莓派IP:8000/control.html` 并强制刷新。

1. 先保持“松手执行”关闭：复制当前关节/输出轴目标，检查零位/输出反解及当前末端 IK 的一致性。
2. 设置单轴和输出轴限速 1°/s，以约 0.5–1° 小目标验证滑块和关节拖动的方向及停止。模型是绳索反解，不代替实机观察。
3. 从已确认安全的姿态回零，观察“收敛 / 停止后复核 / 到位”阶段，记录各轴误差。
   默认 0.2° 到位容差、0.5 s 稳定、10 s 收敛等待，配置在 `src/pi_arm_bringup/config/manager.yaml`。
   这是判定修复，物理残差是否改善需实测；若失败，请保留误差和任务信息，不通过重新置零消除显示偏差。
4. 检查模型 LIN 路径后，再由操作者从网页执行笛卡尔运动。助手未执行实机笛卡尔运动。
