# Pi Arm Console 02

本分支对阶段一控制网页进行界面重构，静态资源无需 Node/npm 构建，随 `scripts/start_web.sh` 提供。
也可以直接打开 `control.html`，手动输入 WebSocket 地址；不会自动连接或使能。

## 文件职责

- `control.html`：页面结构、稳定 DOM 标识、无脚本运动行为。
- `assets/console.css`：视觉变量、控件、状态、桌面/平板/手机布局。
- `assets/control.js`：沿用阶段一的 WebSocket、请求门禁、任务、目标输入、诊断与测试逻辑。
- `assets/console.js`：导航、使用者/开发者视图、实时概览、状态与任务展示；不负责发运动指令。

`control.js` 的 `refreshControlGate` 将同一份反馈及新鲜度传给展示层，不创建第二套控制状态。
视图切换不影响后端权限，也不发送运动。角色选择不是身份认证。

## 页面

操作工作台提供六关节概览、使能/停止、关节/输出轴目标、多关节页内的笛卡尔直线与预设姿态。
输出轴限速仍由页面输入，下一条运动生效。顶部软件停止在滚动时保持可达。
运行记录显示原始请求与终态；开发诊断显示原始 CAN/坐标链和标定；方向测试需独立 commissioning 服务。

## 本地验证

安装 Python Playwright 和 Chromium 后：

```bash
python3 scripts/test_web_console.py       # 离线：导航、尺寸、门禁、软件停止路由
python3 scripts/test_web_diagnostics.py   # 离线：故障、诊断、导出
# 独立终端启动模拟服务（测试不会控制实机）：
ROS_DOMAIN_ID=81 PI_ARM_WEB_PORT=8001 bash scripts/start_web.sh hardware_type:=mock websocket_port:=8766
python3 scripts/test_web_mock.py --url 'http://127.0.0.1:8001/control.html?ws=ws://127.0.0.1:8766'
python3 scripts/test_web_presets.py
python3 scripts/test_web_control_spaces.py
```

若本机 Chromium 在自定义位置，可给各脚本传 `--browser-path`。
浏览器需要中文字体；目标用户系统优先使用系统中文字体，页面不请求第三方字体或 CDN。
更多背景见 [计划](../docs/frontend/PLAN.md) 和 [首轮验收](../docs/frontend/REVIEW.md)。
