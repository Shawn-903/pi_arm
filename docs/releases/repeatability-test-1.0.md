# 重复定位精度测试 1.0 存档

恢复分支：`重复定位精度测试1.0`。后续开发从同一提交创建 `feature/repeatability-test-next`，不在恢复分支继续开发。
本阶段继承 `49f9d4c` 的 R10 标定与实机控制代码；新增内容集中在前端及文档、离线测试。存档操作不停止服务，不改变标定，不发送运动。

## 保存范围

- 点位库 A/B 循环：起始点 → 等待 → 目标点 → 等待/记录；有限循环次数，最终停在目标点保持。
- 独立运动速度及输出轴速度上限，默认 5°/s、最高 10°/s；关节点和笛卡尔点分别走原有运动接口。
- 精确匹配任务编号与到位结果；停止、故障、断连、后台切换、标定变化等情况中止循环，不自动续跑。
- JSON/CSV 记录导出；等待结束采样误差与后端最后核验误差分别记录。
- 已修复“已保存点位正常显示、起始/目标下拉框为空”的旧脚本缓存问题，关联静态资源使用 `v=20260920-repeat2`。
- 当前实机启动引用 `src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml` 及其绑定的 cable 文件，均随 Git 保存。手动电机重新上电流程保持不变。

使用细节见 [重复定位测试](../frontend/REPEATABILITY_TEST.md)。

## 已做验证与边界

以下离线浏览器测试已通过，WebSocket 全部拦截，不操作实机：

- `scripts/test_web_repeat.py`：完整循环、任务匹配、等待、停止/失败/迟到接受、状态失效、导出与响应式布局。
- `scripts/test_web_repeat_loading.py`：复现旧缓存，验证版本化资源更新，以及两个下拉框随点位列表同步并保留选择。
- `scripts/test_web_speed_limits.py`：默认/上限与预设姿态速度参数。
- `scripts/test_web_console.py`：原有导航、门禁、软件停止与布局。

另已用服务实际返回的三个点位做只读浏览器验证，两个下拉框显示正常。三个旧点关联 R06/R09，服务当前关联 R10；循环前需在当前标定下核验并重新录入，存档不自动改写点位标定。

本阶段不声称完成实机重复定位精度验收。末端显示和导出来自编码器/绳索模型，实机精度仍需外部量具。浏览器断连不能保证停止请求送达，当前单段运动可能继续到位；不新增后端断线看门狗。

## 恢复代码

未来恢复前，先结束测试并正常停止当前服务，保存后续开发中未提交的工作。然后在仓库中运行：

```bash
git fetch origin
git switch '重复定位精度测试1.0'
bash scripts/build.sh
bash scripts/start_web.sh \
  hardware_type:=real can_interface:=can0 allow_real_control:=true \
  calibration_file:=/home/cr/pi_arm/src/pi_arm_hardware/config/pack_reference_calibration.R10.recorded.yaml
```

打开 `http://树莓派IP:8000/control.html?v=20260920-repeat2`，连接后检查标定与状态。若在新克隆的仓库中恢复，可用 `git switch --track 'origin/重复定位精度测试1.0'` 创建本地跟踪分支。

仅检查存档、不影响当前开发工作区时，可以另建工作区：

```bash
git worktree add --detach ../pi_arm-repeatability-1.0 '重复定位精度测试1.0'
```

编译产物 `build/`、`install/`、运行日志、生成的 `web/model/` 不纳入 Git，构建和启动脚本会生成所需资源。点位库默认在 `/home/cr/.local/share/pi_arm/points-real.json`，独立于代码分支；切换分支不会恢复或覆盖点位库。页面中的本轮测试记录需另行导出 JSON/CSV。

后续继续开发使用：

```bash
git switch feature/repeatability-test-next
```
