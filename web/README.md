# Pi Arm Console 02

本分支对阶段一控制网页进行界面重构，静态资源无需 Node/npm 构建，随 `scripts/start_web.sh` 提供。
也可以直接打开 `control.html`，手动输入 WebSocket 地址；不会自动连接或使能。
**三维模型需要通过 HTTP 打开**；直接打开文件时，三维区给出启动提示，其余数值控制仍可使用。

## 文件职责

- `control.html`：页面结构、稳定 DOM 标识、无脚本运动行为。
- `assets/console.css`：视觉变量、控件、状态、桌面/平板/手机布局。
- `assets/control.js`：沿用阶段一的 WebSocket、请求门禁、任务、目标输入、诊断与测试逻辑。
- `assets/console.js`：导航、使用者/开发者视图、实时概览、状态与任务展示；不负责发运动指令。
- `assets/viewer.js`：本地 Three.js / URDF 三维显示、随动关节、路径、关节拖动和状态降级；不创建独立连接。
- `assets/interactive.js`：滑块 / 模型手势、后端反解 / IK / 路径预览和回放；通过 control.js 门禁提交有限目标。
- `vendor/`：固定版本的本地三维依赖及许可证；普通启动无需下载，维护构建见其 README。
- `model/`：启动/构建自动生成的显示 URDF、清单和 gzip STL，已加入 gitignore，来源为 `src/pi_arm_description`。

`control.js` 的 `refreshControlGate` 将同一份反馈及新鲜度传给展示层，不创建第二套控制状态。
视图切换不影响后端权限，也不发送运动。角色选择不是身份认证。

## 页面

操作工作台提供六关节概览、使能/停止、关节/输出轴目标、多关节页内的笛卡尔直线与预设姿态。
输出轴限速仍由页面输入，下一条运动生效。顶部软件停止在滚动时保持可达。
运行记录显示原始请求与终态；开发诊断显示原始 CAN/坐标链和标定；方向测试需独立 commissioning 服务。
工作台的「电机故障恢复」提供选定 CAN 电机的清错（0x9B）与重启（0x07），FAULT 下也可操作。
重启后保持关闭，失败或位置参考无法确认时继续禁止运动；详见 [恢复说明](../docs/frontend/MOTOR_RECOVERY.md)。

三维视图同时用于工作台和方向测试页；支持鼠标/触控旋转缩放、正视/侧视/俯视、坐标轴及关节选择。
J1–J6 显示按名称映射的后端关节角；无新鲜有效反馈时为灰色零位参考或最后有效姿态。
`tool0` 三维坐标由同一 URDF 和反馈角计算；只读/方向测试模式也可以显示，不依赖它们未提供的 TF。
半透明青绿色显示目标或路径采样姿态，橙线显示 tool0 路径。打包目标从当前标定读取。
新增输出轴反解、笛卡尔 IK、关节插值 / LIN 路径预览与回放；预览不使能或执行运动。
单轴滑块、选中关节拖动默认仅预览，勾选“松手执行”后每个手势至多提交一个单轴目标。
操作说明、碰撞检查范围与实机切换见 [交互验收记录](../docs/frontend/INTERACTIVE_REVIEW.md)。

模型是由电机编码器经绳索运动学推算的结果，不能作为独立物理位置测量。
资源或 WebGL 失败只影响三维显示；数值反馈、控制和软件停止仍沿用原有链路。
浏览器需支持 WebGL2、DecompressionStream；已在 Chromium 验证，首次需加载约 15 MiB 模型。

## 本地验证

安装 Python Playwright 和 Chromium 后：

```bash
python3 scripts/test_web_console.py       # 离线：导航、尺寸、门禁、软件停止及故障恢复
python3 scripts/test_web_diagnostics.py   # 离线：故障、诊断、导出
python3 scripts/test_web_preview_races.py # 离线：预览失效、迟到响应和手势取消
# 无 ROS/CAN 的三维测试（需 NumPy，脚本自建临时本地 HTTP）：
source /opt/ros/jazzy/setup.bash
python3 scripts/export_web_model.py
python3 scripts/test_web_viewer.py
# 独立终端启动模拟服务（测试不会控制实机）：
ROS_DOMAIN_ID=81 PI_ARM_WEB_PORT=8001 bash scripts/start_web.sh hardware_type:=mock websocket_port:=8766
python3 scripts/test_web_mock.py --url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_recovery.py --ws ws://127.0.0.1:8766 # 仅 mock：选定电机重启/清错
# 在刚启动、初始零位的独立 mock 实例执行：
python3 scripts/test_web_interactive.py --url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_viewer.py --mock-url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_presets.py
python3 scripts/test_web_control_spaces.py
```

若本机 Chromium 在自定义位置，可给支持该选项的脚本传 `--browser-path`。
浏览器需要中文字体；目标用户系统优先使用系统中文字体，页面不请求第三方字体或 CDN。
更多背景见 [计划](../docs/frontend/PLAN.md) 和 [首轮验收](../docs/frontend/REVIEW.md)。
三维验收及现场确认见 [三维视图记录](../docs/frontend/VIEWER_REVIEW.md)。
