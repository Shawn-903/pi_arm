// Diagnostic rendering and export share the control page's state and formatters.
function updateDiagnostics(state, fresh) {
  const summary = document.getElementById("diagnosticSummary");
  if (!state) {
    summary.textContent =
      "尚未收到状态。网页连接与机械臂运动许可相互独立，硬件故障时仍可连接查看。";
  } else {
    const reasons = [];
    if (!fresh) reasons.push("网页状态已过期，以下为最后收到的快照");
    if (!state.state_available)
      reasons.push("未收到新的硬件状态：检查监控进程、CAN 接口和启动终端日志");
    if (state.read_only)
      reasons.push("只读排查模式：控制服务关闭，故障不影响查看反馈");
    if (state.state_available) {
      if (state.motion_stop_pending)
        reasons.push("正在等待电机停止确认，暂不允许新运动");
      if (state.motion_stop_failed)
        reasons.push("停止未确认：运动已锁定，请重试停止并检查 CAN 反馈");
      if (!state.hardware_connected) reasons.push("硬件连接异常");
      if (!state.calibrated)
        reasons.push("标定无效：仍可查看原始电机反馈，模型角可能不可用");
      if (!state.kinematics_valid)
        reasons.push("运动学无效：关节角和末端姿态不作为有效反馈显示");
      if (
        state.kinematics_valid &&
        state.kinematics_error?.code === "JOINT_COMMAND_LIMIT_WARNING"
      ) {
        reasons.push(
          "反馈角超过运动指令限位，仍在解算容差内；显示原始反解角度，目标与运动路径仍受指令限位约束",
        );
      }
      const staleIds =
        state.motor_ids?.filter((id, i) => state.motor_fresh?.[i] !== true) ||
        [];
      if (staleIds.length)
        reasons.push(`反馈缺失或过期的 CAN ID：${staleIds.join(", ")}`);
      if (!state.control_permitted)
        reasons.push(
          state.commissioning
            ? "普通控制关闭；独立电机/关节点动权限见测试区"
            : "硬件未授予运动许可",
        );
    }
    const error = state.kinematics_error || {};
    summary.textContent = [
      `后端：${state.hardware_type} ${state.can_interface || ""} / ${state.read_only ? "只读排查" : state.commissioning ? "方向测试" : "控制模式"} / 状态：${state.robot_state}`,
      `驱动：${state.driver_mode || "--"} / 标定有效：${boolText(state.calibrated)} / 运动学有效：${boolText(state.kinematics_valid)} / 控制许可：${boolText(state.control_permitted)}`,
      `标定 ID：${state.calibration_id || "--"}`,
      `运动学诊断：${error.code || "--"} / 轴序号：${error.axis ?? "--"} / 值：${error.value ?? "--"}`,
      error.message || "",
      state.diagnostic || "",
      ...reasons,
    ]
      .filter(Boolean)
      .join("\n");
  }
  motorIds.forEach((id) => {
    const index = state?.motor_ids?.indexOf(id) ?? -1;
    const row = document.querySelector(`[data-coordinate-row="${id}"]`);
    [
      "raw_protocol_positions_deg",
      "raw_output_positions_rad",
      "software_output_offsets_rad",
      "model_output_positions_rad",
      "motor_error_codes",
      "motor_ages_sec",
    ].forEach((field) => {
      row.querySelector(`[data-coordinate="${field}"]`).textContent =
        formatNumber(
          state?.[field]?.[index],
          field === "motor_error_codes" ? 0 : 9,
        );
    });
    const valid = fresh && state?.motor_fresh?.[index] === true;
    row.querySelector("[data-coordinate='fresh']").textContent = valid
      ? "是"
      : "缺失/过期";
    row.querySelector("[data-coordinate='fresh']").className = valid
      ? "status-ok"
      : "status-error";
  });
}

function exportDiagnostics() {
  if (!lastState) return;
  const snapshot = {
    captured_at: new Date(lastStateAt).toISOString(),
    exported_at: new Date().toISOString(),
    state_age_ms: Date.now() - lastStateAt,
    websocket_url: els.wsUrl.value,
    state: lastState,
    axis_tests: axisTestRecords,
  };
  const url = URL.createObjectURL(
    new Blob([JSON.stringify(snapshot, null, 2)], { type: "application/json" }),
  );
  const link = document.createElement("a");
  link.href = url;
  link.download = `pi-arm-diagnostic-${Date.now()}.json`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
