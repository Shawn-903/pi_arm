/* Presentation only. All motion commands remain in control.js and the ROS gate. */
(() => {
  const titles = {
    workbench: "操作工作台",
    points: "点位示教",
    records: "运行记录",
    diagnostics: "开发诊断",
    commissioning: "方向测试",
  };
  const role = document.getElementById("viewRole");
  const limits = [180, 100.7, 85.6, 30, 60, 180];
  const names = [
    "joint_01",
    "joint_02",
    "joint_04",
    "joint_05",
    "joint_06",
    "joint_09",
  ];
  const overview = document.getElementById("jointOverview");
  overview.innerHTML = names
    .map(
      (name, i) =>
        `<article class="joint-tile unavailable" data-overview-axis="${i}"><header><strong>J${i + 1}</strong><span>±${limits[i]}°</span></header><div class="joint-value"><span>--</span><small>°</small></div><div class="angle-track" aria-hidden="true"><span></span></div></article>`,
    )
    .join("");
  function showView(view) {
    if (!(view in titles)) return;
    window.PiArmViewer?.setView(view);
    if (
      ["diagnostics", "commissioning"].includes(view) &&
      role.value !== "developer"
    ) {
      role.value = "developer";
      updateRole();
    }
    document.body.dataset.view = view;
    document.getElementById("pageTitle").textContent = titles[view];
    document
      .querySelectorAll("[data-view-panel]")
      .forEach((el) => (el.hidden = el.dataset.viewPanel !== view));
    document.querySelectorAll("[data-view-target]").forEach((el) => {
      const active = el.dataset.viewTarget === view;
      el.classList.toggle("active", active);
      if (active) el.setAttribute("aria-current", "page");
      else el.removeAttribute("aria-current");
    });
  }
  function updateRole() {
    document.body.dataset.role = role.value;
    document
      .querySelectorAll("[data-developer]")
      .forEach((el) => (el.hidden = role.value !== "developer"));
    if (
      role.value !== "developer" &&
      ["diagnostics", "commissioning"].includes(document.body.dataset.view)
    )
      showView("workbench");
  }
  role.addEventListener("change", updateRole);
  document.querySelectorAll("[data-view-target]").forEach((el) => {
    el.title = titles[el.dataset.viewTarget];
    el.setAttribute("aria-label", titles[el.dataset.viewTarget]);
    el.addEventListener("click", () => showView(el.dataset.viewTarget));
  });
  document
    .getElementById("inspectNotice")
    .addEventListener("click", () => showView("diagnostics"));
  document
    .getElementById("clearLog")
    .addEventListener("click", () =>
      document.getElementById("commandLog").replaceChildren(),
    );
  const text = (id, value) => (document.getElementById(id).textContent = value);
  const number = (value) => typeof value === "number" && Number.isFinite(value);
  function refresh(state, { fresh, permitted, connectionState }) {
    fresh = Boolean(fresh && state?.state_available !== false);
    window.PiArmViewer?.refresh(state, { fresh, connectionState });
    const connected = connectionState === "connected";
    const valid = Boolean(fresh && state?.kinematics_valid);
    const mode = !fresh
      ? connected
        ? "等待状态"
        : "未连接"
      : state.read_only
        ? "只读排查"
        : state.commissioning
          ? "方向测试"
          : state.hardware_type === "mock"
            ? "模拟运行 · MOCK"
            : "实机控制 · CAN";
    text("consoleMode", mode);
    const states = {
      READY: "就绪",
      DISABLED: "未使能",
      MOVING: "运动中",
      BUSY: "执行中",
      FAULT: "故障",
      ERROR: "故障",
      ESTOP: "停止锁定",
    };
    text(
      "consoleRobotState",
      fresh
        ? states[state.robot_state] || state.robot_state || "未知"
        : "等待反馈",
    );
    text(
      "consoleFreshness",
      fresh
        ? state.motor_fresh?.length === 6 &&
          state.motor_fresh.every((value) => value === true)
          ? "六轴在线"
          : "检查轴反馈"
        : connected
          ? "等待 / 已过期"
          : "暂无数据",
    );
    const problem =
      fresh &&
      state.kinematics_error?.code &&
      state.kinematics_error.code !== "NONE";
    const fault = Boolean(
      fresh &&
        (state.any_error ||
          !state.kinematics_valid ||
          state.motion_stop_failed),
    );
    const notice = document.getElementById("operatorNotice");
    notice.className =
      "notice " +
      (fault ? "error" : problem ? "warn" : permitted ? "ready" : "");
    let hint = !connected
      ? "连接机械臂后查看状态；使能并就绪后可发送运动。"
      : !fresh
        ? "状态未更新，已禁止新运动。请检查连接和后端服务。"
        : state.read_only
          ? "只读排查模式：可以查看反馈和导出诊断，不能控制运动。"
          : state.commissioning
            ? "方向测试模式：请在开发者视图中打开方向测试。"
            : state.motion_stop_failed
              ? "停止尚未确认，请检查设备并重试停止。"
              : problem
                ? `${state.kinematics_error.code === "JOINT_COMMAND_LIMIT_WARNING" ? "反馈在容差范围内，运动目标仍须满足标准限位。" : "运动学反馈异常，请查看诊断。"} ${state.kinematics_error.code}`
                : permitted
                  ? "机械臂已就绪。核对目标、速度与工作空间后执行运动。"
                  : state.task?.status === 1
                    ? "任务执行中。新的运动请求已锁定，可随时请求停止。"
                    : "当前不可发送新运动，请检查使能、标定和控制许可。";
    text("operatorHint", hint);
    text(
      "consoleCalibration",
      !fresh
        ? "等待新鲜状态"
        : state.calibrated
          ? state.calibration_id || "已校验"
          : state.kinematics_error?.code === "CALIBRATION_INVALID"
            ? `标定校验失败：${state.kinematics_error.message || "请查看详细诊断"}`
            : "标定未通过，请查看详细诊断",
    );
    text(
      "overviewState",
      valid ? "实时关节反馈 · 绳索模型反解" : "角度不可用 · 等待有效反馈",
    );
    text(
      "lastFeedbackTime",
      fresh ? "状态更新 " + new Date().toLocaleTimeString() : "暂无新鲜反馈",
    );
    document.querySelectorAll("[data-overview-axis]").forEach((el) => {
      const i = Number(el.dataset.overviewAxis),
        q = state?.current_joint_angles?.[i];
      const usable = valid && number(q);
      el.classList.toggle("unavailable", !usable);
      el.classList.toggle("warn", usable && Math.abs(q) > limits[i]);
      el.querySelector(".joint-value>span").textContent = usable
        ? q.toFixed(2)
        : "--";
      el.querySelector(".angle-track>span").style.left = usable
        ? `${Math.max(0, Math.min(100, 50 + (q / limits[i]) * 50))}%`
        : "50%";
    });
    const task = fresh ? state.task : null;
    const status = task?.status || 0;
    const badge = document.getElementById("taskBadge");
    badge.textContent = !fresh
      ? "未连接"
      : { 0: "待机", 1: "执行中", 2: "已完成", 3: "已取消", 4: "失败" }[
          status
        ] || "待机";
    badge.className =
      "pill " + (status === 4 ? "error" : status === 2 ? "success" : "");
    text(
      "taskSummary",
      !fresh
        ? "等待实时任务反馈"
        : task?.task_id
          ? `#${task.task_id} · ${task.command || "运动任务"}`
          : "暂无运动任务",
    );
    text("taskCaption", task?.message || "以控制器最终结果为准");
    const arrivalLabels = {
      planning: "规划中",
      tracking: "跟随中",
      settling: "终点收敛中",
      verifying_after_stop: "停止后核验",
      reached: "到位已核验",
      not_reached: "未到位",
      canceled: "已取消",
      failed: "任务失败",
    };
    text(
      "arrivalSummary",
      fresh && task?.arrival_state
        ? `到位状态：${arrivalLabels[task.arrival_state] || task.arrival_state} · 容差 ±${Number(task.goal_tolerance_deg).toFixed(2)}°`
        : "到位状态：尚未验证",
    );
    const arrivalErrors = document.getElementById("arrivalErrors");
    arrivalErrors.replaceChildren();
    if (fresh && task?.target_joint_angles_deg?.length === 6) {
      task.target_joint_angles_deg.forEach((target, i) => {
        const error = task.error_joint_angles_deg?.[i];
        const row = document.createElement("div");
        row.className =
          number(error) && Math.abs(error) > task.goal_tolerance_deg
            ? "status-error"
            : "";
        row.textContent = `J${i + 1} 目标 ${number(target) ? target.toFixed(2) : "--"}° · 末次核验误差 ${number(error) ? error.toFixed(3) : "--"}°`;
        arrivalErrors.appendChild(row);
      });
    }
    const progress = number(task?.progress)
      ? Math.max(0, Math.min(100, task.progress * 100))
      : 0;
    document
      .getElementById("taskProgress")
      .setAttribute("aria-valuenow", String(progress));
    document.querySelector("#taskProgress>span").style.width = progress + "%";
    document.getElementById("commissioningUnavailable").hidden = Boolean(
      fresh && state.commissioning,
    );
    // A stale/disconnected page must not keep displaying old angles as live data.
    if (!fresh) {
      document
        .querySelectorAll(
          '[data-status-row] [data-field="angle"],[data-status-row] [data-field="speed"]',
        )
        .forEach((el) => (el.textContent = "--"));
      ["endX", "endY", "endZ", "endRx", "endRy", "endRz"].forEach(
        (id) => (document.getElementById(id).value = "--"),
      );
    }
  }
  window.PiArmConsole = { refresh, showView };
})();
