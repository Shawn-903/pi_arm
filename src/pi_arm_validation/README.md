# pi_arm_validation

本包使用实际的 `pi_arm_transmission` C++ 库生成验证数据，再由 Python/Matplotlib 生成 PNG 和 Markdown 报告。绘图脚本不重复实现绳索运动学公式。

构建并 source 工作区后运行：

```bash
RESULT_DIR="$PWD/pi_arm_validation_results_$(date +%Y%m%d_%H%M%S)"

ros2 run pi_arm_validation kinematics_validation \
  --output-dir "$RESULT_DIR" \
  --samples 10000 \
  --sweep-points 101

ros2 run pi_arm_validation plot_kinematics_validation \
  --input-dir "$RESULT_DIR"
```

若不传 `--config`，数据生成器会自动使用已安装的：

```text
pi_arm_description/config/cable_kinematics.yaml
```

主要输出：

- `summary.json`
- `position_roundtrip.csv`
- `velocity_roundtrip.csv`
- `jacobian_error.csv`
- `joint2_coupling.csv`
- `axis45_nonlinear.csv`
- `axis6_residual.csv`
- `position_error.png`
- `velocity_error.png`
- `jacobian_error_heatmap.png`
- `joint2_coupling.png`
- `axis45_nonlinear.png`
- `axis6_residual_heatmap.png`
- `report.md`

工具只进行离线数学验证，不发送 CAN 指令，也不控制真实电机。
