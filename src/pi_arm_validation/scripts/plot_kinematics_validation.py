#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


PLOT_FILES = {
    "position": "position_error.png",
    "velocity": "velocity_error.png",
    "jacobian": "jacobian_error_heatmap.png",
    "coupling": "joint2_coupling.png",
    "nonlinear": "axis45_nonlinear.png",
    "axis6": "axis6_residual_heatmap.png",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render PNG and Markdown reports from pi_arm_validation CSV files."
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("validation_results"),
        help="Directory produced by kinematics_validation.",
    )
    parser.add_argument("--dpi", type=int, default=160, help="PNG resolution.")
    return parser.parse_args()


def load_table(path: Path) -> np.ndarray:
    if not path.is_file():
        raise FileNotFoundError(f"missing validation data: {path}")
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    return np.atleast_1d(data)


def save_figure(figure: plt.Figure, path: Path, dpi: int) -> None:
    figure.tight_layout()
    figure.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(figure)


def plot_roundtrip(
    input_dir: Path,
    csv_name: str,
    output_name: str,
    title: str,
    ylabel: str,
    dpi: int,
) -> None:
    data = load_table(input_dir / csv_name)
    fields = list(data.dtype.names or ())
    sample_field = fields[0]
    error_fields = fields[1:]
    figure, axis = plt.subplots(figsize=(10, 5.5))
    floor = 1.0e-18
    for index, field in enumerate(error_fields):
        values = np.maximum(np.asarray(data[field], dtype=float), floor)
        axis.semilogy(
            data[sample_field],
            values,
            linewidth=0.9,
            label=f"Axis {index + 1}",
        )
    axis.set_title(title)
    axis.set_xlabel("Random sample")
    axis.set_ylabel(ylabel)
    axis.grid(True, which="both", alpha=0.3)
    axis.legend(ncol=3)
    save_figure(figure, input_dir / output_name, dpi)


def plot_jacobian(input_dir: Path, dpi: int) -> None:
    data = load_table(input_dir / "jacobian_error.csv")
    matrix = np.zeros((6, 6), dtype=float)
    for row in data:
        matrix[int(row["row"]) - 1, int(row["column"]) - 1] = row["max_abs_error"]
    log_error = np.log10(np.maximum(matrix, 1.0e-18))

    figure, axis = plt.subplots(figsize=(7, 6))
    image = axis.imshow(log_error, cmap="magma", origin="upper", vmin=-18)
    axis.set_title("Analytic vs finite-difference Jacobian")
    axis.set_xlabel("Active joint column")
    axis.set_ylabel("Output axis row")
    axis.set_xticks(range(6), [str(index) for index in range(1, 7)])
    axis.set_yticks(range(6), [str(index) for index in range(1, 7)])
    for row in range(6):
        for column in range(6):
            axis.text(
                column,
                row,
                f"{matrix[row, column]:.1e}",
                ha="center",
                va="center",
                fontsize=7,
                color="white" if log_error[row, column] < -8 else "black",
            )
    colorbar = figure.colorbar(image, ax=axis)
    colorbar.set_label("log10(max absolute error)")
    save_figure(figure, input_dir / PLOT_FILES["jacobian"], dpi)


def plot_joint2_coupling(input_dir: Path, dpi: int) -> None:
    data = load_table(input_dir / "joint2_coupling.csv")
    q2 = data["joint_02_rad"]
    figure, axis = plt.subplots(figsize=(9, 5.5))
    for index in range(1, 7):
        axis.plot(q2, data[f"output_{index}_rad"], label=f"Output {index}")
    axis.axhline(0.0, color="black", linewidth=0.7)
    axis.axvline(0.0, color="black", linewidth=0.7)
    axis.set_title("Output-axis response to joint_02 coupling")
    axis.set_xlabel("joint_02 active coordinate [rad]")
    axis.set_ylabel("Output-axis position [rad]")
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=3)
    save_figure(figure, input_dir / PLOT_FILES["coupling"], dpi)


def plot_axis45_nonlinearity(input_dir: Path, dpi: int) -> None:
    data = load_table(input_dir / "axis45_nonlinear.csv")
    active_joint = data["active_joint_rad"]
    figure, axes = plt.subplots(2, 1, figsize=(9, 8), sharex=True)

    axes[0].plot(active_joint, data["output_4_self_rad"], label="Axis 4")
    axes[0].plot(
        active_joint,
        data["output_5_self_rad"],
        linestyle="--",
        label="Axis 5",
    )
    axes[0].set_ylabel("Self output position [rad]")
    axes[0].set_title("Axis 4/5 nonlinear position mapping")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(active_joint, data["jacobian_4"], label="Axis 4 derivative")
    axes[1].plot(
        active_joint,
        data["jacobian_5"],
        linestyle="--",
        label="Axis 5 derivative",
    )
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_xlabel("ROS active joint coordinate [rad]")
    axes[1].set_ylabel("d(output)/d(joint)")
    axes[1].set_title("Axis 4/5 local velocity gain")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    save_figure(figure, input_dir / PLOT_FILES["nonlinear"], dpi)


def plot_axis6_residual(input_dir: Path, dpi: int) -> None:
    data = load_table(input_dir / "axis6_residual.csv")
    joint2_values = np.unique(data["joint_02_rad"])
    joint6_values = np.unique(data["joint_09_rad"])
    residual = np.zeros((joint2_values.size, joint6_values.size), dtype=float)

    joint2_index = {value: index for index, value in enumerate(joint2_values)}
    joint6_index = {value: index for index, value in enumerate(joint6_values)}
    for row in data:
        residual[
            joint2_index[row["joint_02_rad"]],
            joint6_index[row["joint_09_rad"]],
        ] = max(
            abs(row["branch_a_residual_mm"]),
            abs(row["branch_b_residual_mm"]),
        )

    log_residual = np.log10(np.maximum(residual, 1.0e-18))
    figure, axis = plt.subplots(figsize=(8, 6))
    image = axis.imshow(
        log_residual,
        cmap="viridis",
        origin="lower",
        aspect="auto",
        extent=[
            joint6_values[0],
            joint6_values[-1],
            joint2_values[0],
            joint2_values[-1],
        ],
        vmin=-18,
    )
    axis.set_title("Axis 6 dual-branch closure residual")
    axis.set_xlabel("joint_09 active coordinate [rad]")
    axis.set_ylabel("joint_02 active coordinate [rad]")
    colorbar = figure.colorbar(image, ax=axis)
    colorbar.set_label("log10(max branch residual [mm])")
    save_figure(figure, input_dir / PLOT_FILES["axis6"], dpi)


def write_report(input_dir: Path, summary: dict) -> None:
    report = f"""# Pi Arm 绳索运动学离线验证报告

## 运行信息

| 项目 | 数值 |
|---|---:|
| 模型版本 | {summary["model_version"]} |
| 随机样本数 | {summary["samples"]} |
| 扫描点数 | {summary["sweep_points"]} |
| 随机种子 | {summary["seed"]} |
| 关节采样范围 | ±{summary["joint_range_rad"]} rad |
| 最大位置往返误差 | {summary["max_position_error_rad"]:.6e} rad |
| 最大速度往返误差 | {summary["max_velocity_error_rad_s"]:.6e} rad/s |
| 最大雅可比误差 | {summary["max_jacobian_error"]:.6e} |
| 最大第六轴闭合残差 | {summary["max_axis6_residual_mm"]:.6e} mm |

配置文件：

```text
{summary["config_path"]}
```

## 位置正反解误差

![Position round-trip error]({PLOT_FILES["position"]})

## 速度正反解误差

![Velocity round-trip error]({PLOT_FILES["velocity"]})

## 雅可比误差

![Jacobian error]({PLOT_FILES["jacobian"]})

## 关节 2 耦合

![Joint 2 coupling]({PLOT_FILES["coupling"]})

## 第四、第五轴非线性

![Axis 4/5 nonlinearity]({PLOT_FILES["nonlinear"]})

## 第六轴双绳路闭合残差

![Axis 6 residual]({PLOT_FILES["axis6"]})

## 解释边界

这些图验证 C++ 运动学实现的数值自洽性。机械臂没有关节端编码器，因此报告不能单独证明真实关节不存在绳索滑移、弹性伸长、回差或结构变形；真实机械误差仍需外部角度或位姿测量。
"""
    (input_dir / "report.md").write_text(report, encoding="utf-8")


def main() -> int:
    args = parse_args()
    input_dir = args.input_dir.resolve()
    if args.dpi <= 0:
        raise ValueError("--dpi must be positive")

    summary_path = input_dir / "summary.json"
    if not summary_path.is_file():
        raise FileNotFoundError(f"missing validation summary: {summary_path}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))

    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        plt.style.use("default")

    plot_roundtrip(
        input_dir,
        "position_roundtrip.csv",
        PLOT_FILES["position"],
        "Joint position forward/inverse round-trip error",
        "Absolute error [rad]",
        args.dpi,
    )
    plot_roundtrip(
        input_dir,
        "velocity_roundtrip.csv",
        PLOT_FILES["velocity"],
        "Joint velocity forward/inverse round-trip error",
        "Absolute error [rad/s]",
        args.dpi,
    )
    plot_jacobian(input_dir, args.dpi)
    plot_joint2_coupling(input_dir, args.dpi)
    plot_axis45_nonlinearity(input_dir, args.dpi)
    plot_axis6_residual(input_dir, args.dpi)
    write_report(input_dir, summary)

    print(f"PI_ARM_PLOTS_OK output={input_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
