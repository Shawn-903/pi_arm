const defaultHost = window.location.hostname || "127.0.0.1";
const defaultWsUrl =
  new URLSearchParams(window.location.search).get("ws") ||
  `ws://${defaultHost}:8765`;
const activeJointNames = [
  "joint_01",
  "joint_02",
  "joint_04",
  "joint_05",
  "joint_06",
  "joint_09",
];
const MOTOR_COUNT = 6;
const motorIds = Array.from({ length: MOTOR_COUNT }, (_, index) => index + 1);

let ws = null;
let msgId = 1;
let lastState = null;
let lastStateAt = 0;
let lastTaskKey = "";
let targetsInitialized = false;
let acceptedTaskId = null;
let ownAxisTest = null;
const axisTestRecords = [];
const pending = new Map();
const motionCommands = new Set([
  "motor_move",
  "move",
  "output_move",
  "movej",
  "movejs",
  "movel",
  "move_preset",
]);

const els = {
  jointStatusBody: document.getElementById("jointStatusBody"),
  motorStatusBody: document.getElementById("motorStatusBody"),
  motorList: document.getElementById("motorList"),
  multiJointGrid: document.getElementById("multiJointGrid"),
  multiSpeed: document.getElementById("multiSpeed"),
  multiMoveBtn: document.getElementById("multiMoveBtn"),
  multiStopBtn: document.getElementById("multiStopBtn"),
  moveAllBtn: document.getElementById("moveAllBtn"),
  jointParamCard: document.getElementById("jointParamCard"),
  lineParamCard: document.getElementById("lineParamCard"),
  endX: document.getElementById("endX"),
  endY: document.getElementById("endY"),
  endZ: document.getElementById("endZ"),
  endRx: document.getElementById("endRx"),
  endRy: document.getElementById("endRy"),
  endRz: document.getElementById("endRz"),
  wsUrl: document.getElementById("wsUrl"),
  connectBtn: document.getElementById("connectBtn"),
  disconnectBtn: document.getElementById("disconnectBtn"),
  safetyState: document.getElementById("safetyState"),
};

function formatNumber(value, digits = 3) {
  return value !== null && value !== undefined && Number.isFinite(Number(value))
    ? Number(value).toFixed(digits)
    : "--";
}

function boolText(value) {
  if (value === true) return "是";
  if (value === false) return "否";
  return "--";
}

function statusClass(value, badWhenTrue = false) {
  if (value === undefined || value === null) return "status-muted";
  if (badWhenTrue) return value ? "status-error" : "status-ok";
  return value ? "status-ok" : "status-muted";
}

function log(message, payload) {
  const time = new Date().toLocaleTimeString();
  const suffix = payload === undefined ? "" : ` ${JSON.stringify(payload)}`;
  console.info(`[${time}] ${message}${suffix}`);
  const entry = document.createElement("div");
  entry.textContent = `[${time}] ${message}${suffix}`;
  if (/失败|拒绝|错误|超时/.test(message)) entry.className = "status-error";
  const container = document.getElementById("commandLog");
  container.prepend(entry);
  while (container.children.length > 100) container.lastChild.remove();
}

let connectionState = "closed";

function refreshControlGate() {
  const connected = connectionState === "connected";
  const fresh = connected && lastState && Date.now() - lastStateAt < 1500;
  if (
    fresh &&
    acceptedTaskId !== null &&
    lastState.task?.task_id >= acceptedTaskId &&
    (lastState.task.task_id !== acceptedTaskId ||
      [2, 3, 4].includes(lastState.task.status))
  ) {
    acceptedTaskId = null;
  }
  const permitted =
    !lastState?.commissioning &&
    !lastState?.read_only &&
    acceptedTaskId === null &&
    fresh &&
    lastState.control_permitted === true &&
    lastState.robot_state === "READY" &&
    lastState.task?.status !== 1 &&
    ![...pending.values()].some((item) => motionCommands.has(item.cmd));
  document
    .querySelectorAll(
      "button[data-cmd], #multiStopBtn, button[data-action='stop']",
    )
    .forEach((button) => {
      button.disabled = !connected || lastState?.read_only === true;
    });
  document
    .querySelectorAll("[data-motion='true'], button[data-action='move']")
    .forEach((button) => {
      button.disabled = !permitted;
    });
  const lineMode = document.querySelector(
    "input[name='motionMode'][value='line']",
  );
  lineMode.disabled =
    !fresh || !lastState.supported_motion_commands?.includes("movel");
  if (lineMode.disabled && lineMode.checked) {
    document.querySelector("input[name='motionMode'][value='joint']").checked =
      true;
    updateMotionMode();
  }
  document.getElementById("copyCurrentBtn").disabled =
    !fresh || !lastState.kinematics_valid;
  document.getElementById("copyOutputTargets").disabled =
    !fresh || !lastState.kinematics_valid;
  document.getElementById("exportDiagnosticBtn").disabled = !lastState;
  updateDiagnostics(lastState, fresh);
  refreshAxisTests(fresh);
  refreshPresetPoses(fresh, permitted);
  els.safetyState.textContent = connected
    ? !fresh
      ? "已连接 / 等待新鲜机械臂状态"
      : `${lastState.hardware_type || "未知后端"}${lastState.read_only ? " 只读排查" : lastState.commissioning ? " 方向测试" : ""} / ${lastState.robot_state} / ${permitted ? "可发送运动" : "禁止新运动"}`
    : connectionState === "connecting"
      ? "正在连接…"
      : "未连接 / 禁止运动";
  window.PiArmConsole?.refresh(lastState, {
    fresh,
    permitted,
    connectionState,
  });
}

function setConnectionState(state) {
  connectionState = state;
  refreshControlGate();
}

function refreshPresetPoses(fresh, permitted) {
  for (const [name, statusId] of [
    ["packing", "packingPoseStatus"],
    ["zero", "zeroPoseStatus"],
  ]) {
    const preset = lastState?.preset_poses?.[name];
    const available = fresh && permitted && preset?.available === true;
    const button = document.querySelector(`[data-preset="${name}"]`);
    button.disabled = !available;
    const angles = preset?.target_joint_angles_deg;
    const target =
      Array.isArray(angles) && angles.length === 6
        ? `[${angles.map((v) => formatNumber(v, 3)).join(", ")}]°`
        : "目标未加载";
    const reason = !fresh
      ? "等待连接及新鲜状态"
      : !preset
        ? "后端尚未提供预设姿态"
        : preset.reason || (!permitted ? "等待当前运动或请求结束" : "可执行");
    button.title = reason;
    document.getElementById(statusId).textContent =
      `${name === "packing" ? "打包姿态" : "零位姿态"}：${target}；${reason}`;
  }
}

function connect() {
  if (
    ws &&
    (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)
  ) {
    return;
  }

  setConnectionState("connecting");
  const wsUrl = els.wsUrl.value.trim();
  if (!/^ws:\/\//i.test(wsUrl)) {
    log("地址必须使用 ws://；服务端不提供 TLS", { url: wsUrl });
    setConnectionState("error");
    return;
  }
  log("正在连接", { url: wsUrl });

  lastState = null;
  lastStateAt = 0;
  targetsInitialized = false;
  acceptedTaskId = null;
  try {
    ws = new WebSocket(wsUrl);
  } catch (error) {
    log("连接失败", { message: error.message });
    setConnectionState("error");
    return;
  }

  ws.addEventListener("open", () => {
    setConnectionState("connected");
    log("WebSocket 已连接");
  });

  ws.addEventListener("message", (event) => {
    try {
      const data = JSON.parse(event.data);
      handleMessage(data);
    } catch (error) {
      log("收到非法 JSON", { raw: event.data });
    }
  });

  ws.addEventListener("close", () => {
    for (const item of pending.values()) clearTimeout(item.timer);
    pending.clear();
    ownAxisTest = null;
    lastState = null;
    lastStateAt = 0;
    updateState({});
    document.getElementById("taskState").textContent =
      "连接已断开，运动结果未知";
    setConnectionState("closed");
    log("WebSocket 已关闭");
  });

  ws.addEventListener("error", () => {
    setConnectionState("error");
    log("WebSocket 连接错误");
  });
}

function sendCommand(cmd, params = {}) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    log("发送失败：WebSocket 未连接", { cmd, params });
    return false;
  }

  if (lastState?.read_only) {
    log("命令被拒绝：当前为只读排查模式");
    return false;
  }
  if (
    motionCommands.has(cmd) &&
    (!lastState ||
      Date.now() - lastStateAt >= 1500 ||
      !lastState.control_permitted ||
      lastState.robot_state !== "READY" ||
      acceptedTaskId !== null ||
      [...pending.values()].some((item) => motionCommands.has(item.cmd)))
  ) {
    log("运动被拒绝：状态未就绪或有运动请求等待响应");
    return false;
  }
  if (
    cmd === "axis_test" &&
    (ownAxisTest !== null ||
      [...pending.values()].some((item) => item.cmd === "axis_test"))
  )
    return false;
  if (
    cmd === "move_preset" &&
    lastState?.preset_poses?.[params.preset]?.available !== true
  ) {
    log("运动被拒绝：预设姿态尚不可用");
    return false;
  }
  if (motionCommands.has(cmd)) {
    try {
      params = {
        ...params,
        output_speed_limit: readPositiveFloat("outputSpeedLimit"),
      };
    } catch (error) {
      log("参数错误", { message: error.message });
      return false;
    }
  }
  const payload = {
    type: "req",
    msg_id: msgId++,
    cmd,
    params,
  };
  const timer = setTimeout(() => {
    pending.delete(payload.msg_id);
    log("请求响应超时，执行结果未知，请检查任务状态", {
      cmd,
      msg_id: payload.msg_id,
    });
    refreshControlGate();
  }, 15000);
  const record =
    cmd === "axis_test"
      ? {
          requested_at: new Date().toISOString(),
          request: { ...params },
          before: structuredClone(lastState),
          status: "请求中",
          observation: "",
        }
      : null;
  if (record) axisTestRecords.push(record);
  pending.set(payload.msg_id, { cmd, timer, record });
  ws.send(JSON.stringify(payload));
  refreshControlGate();
  if (cmd !== "axis_test_heartbeat") log("发送命令", payload);
  return true;
}

function handleMessage(data) {
  if (data.type === "state") {
    lastState = data;
    lastStateAt = Date.now();
    updateState(data);
    const test = data.axis_test;
    if (test?.test_id && !test.active) {
      const record = [...axisTestRecords]
        .reverse()
        .find((item) => item.test_id === test.test_id && !item.after);
      if (record && !record.after) {
        record.after = structuredClone(data);
        record.status = test.status;
        record.completed_at = new Date().toISOString();
        renderAxisTestRecords();
      }
      if (ownAxisTest === test.test_id) ownAxisTest = null;
    }
    refreshControlGate();
    return;
  }

  if (data.type === "rsp") {
    const item = pending.get(data.msg_id);
    if (item) clearTimeout(item.timer);
    pending.delete(data.msg_id);
    const ok = data.code === 0;
    if (item?.cmd === "axis_test_heartbeat") return;
    if (item?.record) {
      item.record.test_id = data.data?.axis_test_id;
      item.record.status = ok ? "已接受，等待结果" : `拒绝：${data.ms}`;
      if (ok) ownAxisTest = data.data.axis_test_id;
      renderAxisTestRecords();
    }
    const accepted =
      motionCommands.has(item?.cmd) && data.data?.task_id !== undefined;
    if (ok && accepted) acceptedTaskId = data.data.task_id;
    log(
      ok
        ? accepted
          ? "运动请求已接受，等待任务结果"
          : "服务执行成功"
        : "命令失败",
      {
        cmd: item?.cmd,
        msg_id: data.msg_id,
        code: data.code,
        ms: data.ms,
        data: data.data,
      },
    );
    refreshControlGate();
    return;
  }

  log("收到未知消息", data);
}

function updateState(state) {
  motorIds.forEach((motorId, index) => {
    const row = document.querySelector(`[data-status-row="${motorId}"]`);
    if (!row) return;

    const stateIndex = state.joint_names?.length
      ? state.joint_names.indexOf(activeJointNames[index])
      : index;
    const angle = state.kinematics_valid
      ? state.current_joint_angles?.[stateIndex]
      : null;
    const speed = state.kinematics_valid
      ? state.current_speeds?.[stateIndex]
      : null;
    const enabled = state.enabled?.[stateIndex];
    const hasError = state.has_error?.[stateIndex];
    const inMotion = state.in_motion?.[stateIndex];
    const updated = state.last_update_timestamps?.[stateIndex];

    row.querySelector("[data-field='angle']").textContent = formatNumber(
      angle,
      6,
    );
    row.querySelector("[data-field='angle']").title = state.kinematics_valid
      ? "运动学反解角；新鲜度请参照原始电机反馈"
      : "运动学无效，不能将旧值或缺失值解释为当前姿态";
    row.querySelector("[data-field='speed']").textContent = formatNumber(speed);
    setCell(row, "enabled", boolText(enabled), statusClass(enabled));
    setCell(row, "error", boolText(hasError), statusClass(hasError, true));
    setCell(row, "motion", boolText(inMotion), statusClass(inMotion));
    row.querySelector("[data-field='updated']").textContent = formatNumber(
      updated,
      1,
    );
  });

  motorIds.forEach((motorId) => {
    const index = state.motor_ids?.indexOf(motorId) ?? -1;
    const row = document.querySelector(`[data-motor-row="${motorId}"]`);
    if (!row) return;
    row.querySelector("[data-field='raw-angle']").textContent = formatNumber(
      state.raw_protocol_positions_deg?.[index],
    );
    setCell(
      row,
      "motor-enabled",
      boolText(state.motor_enabled?.[index]),
      statusClass(state.motor_enabled?.[index]),
    );
    setCell(
      row,
      "motor-error",
      boolText(state.motor_has_error?.[index]),
      statusClass(state.motor_has_error?.[index], true),
    );
    setCell(
      row,
      "motor-fresh",
      boolText(state.motor_fresh?.[index]),
      statusClass(state.motor_fresh?.[index]),
    );
  });

  updatePose(state.kinematics_valid ? state.pose : {});
  document.getElementById("diagnosticState").textContent = [
    state.kinematics_error?.message,
    state.diagnostic,
  ]
    .filter(Boolean)
    .join(" | ");
  const task = state.task;
  if (task?.task_id) {
    const names = ["空闲", "运动中", "已完成", "已取消", "失败"];
    document.getElementById("taskState").textContent =
      `任务 #${task.task_id} ${task.command}：${names[task.status] || "未知"} / ${formatNumber(task.progress * 100, 0)}% / ${task.message} (code=${task.code})`;
    const key = `${task.task_id}:${task.status}`;
    if (key !== lastTaskKey) {
      log(`运动任务${names[task.status] || "状态更新"}`, task);
      lastTaskKey = key;
    }
  }
  if (!task?.task_id) {
    document.getElementById("taskState").textContent =
      state.robot_state === "DISCONNECTED"
        ? "机械臂状态不可用，无法确认运动结果"
        : "暂无运动任务";
  }
  if (
    !targetsInitialized &&
    state.current_joint_angles?.length === MOTOR_COUNT &&
    state.kinematics_valid &&
    state.current_joint_angles.every(Number.isFinite)
  ) {
    copyCurrentTargets();
    targetsInitialized = true;
  }
}

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

function refreshAxisTests(fresh) {
  const commissioning = lastState?.commissioning === true;
  document.getElementById("axisTestPanel").hidden = !commissioning;
  document.getElementById("normalMotionControls").hidden = commissioning;
  const idle =
    fresh &&
    commissioning &&
    !lastState.axis_test?.active &&
    ownAxisTest === null &&
    ![...pending.values()].some((item) => item.cmd === "axis_test");
  const id = Number(document.getElementById("testMotorId").value);
  const index = lastState?.motor_ids?.indexOf(id) ?? -1;
  const enabled = lastState?.motor_enabled?.[index] === true;
  const motorOK = idle && lastState.motor_test_permitted && enabled;
  const jointOK = idle && lastState.joint_test_permitted;
  document.querySelectorAll('[data-axis-test="motor"]').forEach((button) => {
    button.disabled = !motorOK;
  });
  document.querySelectorAll('[data-axis-test="joint"]').forEach((button) => {
    button.disabled = !jointOK;
  });
  document.getElementById("testEnableMotor").disabled =
    !idle || !lastState?.motor_test_permitted;
  document.getElementById("testDisableMotor").disabled = !idle;
  document.getElementById("axisTestStop").disabled =
    connectionState !== "connected";
  const ratio = lastState?.protocol_angle_ratios?.[index];
  const direction = lastState?.motor_directions?.[index];
  const minimumSpeed = Number.isFinite(ratio) && ratio > 0 ? 1 / ratio : null;
  document.getElementById("motorTestSpeed").min =
    minimumSpeed === null ? "0.05" : String(Math.max(0.05, minimumSpeed));
  document.getElementById("motorTestReason").textContent =
    `CAN ${id} / 协议角 ${formatNumber(lastState?.raw_protocol_positions_deg?.[index], 6)}° / 配置减速比 ${ratio ?? "--"} / 当前 motor_direction ${direction ?? "--"}；` +
    `最小输出轴速度 ${formatNumber(minimumSpeed, 6)}°/s；` +
    (motorOK
      ? "可测试（不依赖运动学反解）"
      : !fresh || !lastState?.motor_test_permitted
        ? "等待六电机反馈新鲜、无错误、静止，且没有其他测试；请查看下方诊断"
        : !enabled
          ? "请先使能所选电机"
          : "等待当前测试结束");
  document.getElementById("jointTestReason").textContent = jointOK
    ? "可按当前模型测试"
    : "关节测试需有效标定、有效反解、全部电机使能且静止；输出/关节限位仍在后端检查";
  document.getElementById("axisTestStatus").textContent =
    `测试 #${lastState?.axis_test?.test_id ?? 0}：${lastState?.axis_test?.status || "尚无测试"}`;
}

function renderAxisTestRecords() {
  document.getElementById("axisTestRecords").textContent = axisTestRecords
    .slice(-20)
    .reverse()
    .map((item) => {
      const delta = item.after?.raw_protocol_positions_deg?.map((value, i) => {
        const before = item.before?.raw_protocol_positions_deg?.[i];
        return Number.isFinite(value) && Number.isFinite(before)
          ? formatNumber(value - before, 6)
          : "--";
      });
      return `#${item.test_id ?? "--"} ${item.request.space} 轴${item.request.axis_id} ${item.request.delta_deg}° / ${item.status}\n协议角变化(°)：${delta?.join(", ") ?? "等待反馈"}\n现场观察：${item.observation || "未填写"}`;
    })
    .join("\n\n");
}

function bindAxisTests() {
  document.getElementById("testMotorId").innerHTML = motorIds
    .map((id) => `<option value="${id}">CAN ${id}</option>`)
    .join("");
  document.getElementById("testJointId").innerHTML = activeJointNames
    .map((name, i) => `<option value="${i + 1}">${name}</option>`)
    .join("");
  document
    .getElementById("testMotorId")
    .addEventListener("change", refreshControlGate);
  document
    .getElementById("testEnableMotor")
    .addEventListener("click", () =>
      sendCommand("enable_motor", {
        motor_id: Number(document.getElementById("testMotorId").value),
      }),
    );
  document
    .getElementById("testDisableMotor")
    .addEventListener("click", () =>
      sendCommand("disable_motor", {
        motor_id: Number(document.getElementById("testMotorId").value),
      }),
    );
  document.querySelectorAll("[data-axis-test]").forEach((button) =>
    button.addEventListener("click", () => {
      try {
        const space = button.dataset.axisTest;
        sendCommand("axis_test", {
          space,
          axis_id: Number(
            document.getElementById(
              space === "motor" ? "testMotorId" : "testJointId",
            ).value,
          ),
          delta_deg:
            Number(button.dataset.sign) * readPositiveFloat(`${space}TestStep`),
          speed_deg_s: readPositiveFloat(`${space}TestSpeed`),
        });
      } catch (error) {
        log("参数错误", { message: error.message });
      }
    }),
  );
  document
    .getElementById("axisTestStop")
    .addEventListener("click", () => sendCommand("axis_test_stop"));
  document
    .getElementById("saveAxisObservation")
    .addEventListener("click", () => {
      const record = axisTestRecords.at(-1);
      if (!record) return;
      record.observation = document.getElementById("axisObservation").value;
      renderAxisTestRecords();
    });
  setInterval(() => {
    if (
      ownAxisTest !== null &&
      connectionState === "connected" &&
      ![...pending.values()].some((item) => item.cmd === "axis_test_heartbeat")
    ) {
      sendCommand("axis_test_heartbeat", { test_id: ownAxisTest });
    }
  }, 250);
  document.addEventListener("visibilitychange", () => {
    if (
      document.hidden &&
      (ownAxisTest !== null ||
        [...pending.values()].some((item) => item.cmd === "axis_test"))
    ) {
      sendCommand("axis_test_stop");
    }
  });
}

function updatePose(pose) {
  els.endX.value = formatNumber(pose?.x);
  els.endY.value = formatNumber(pose?.y);
  els.endZ.value = formatNumber(pose?.z);
  els.endRx.value = formatNumber(pose?.rx);
  els.endRy.value = formatNumber(pose?.ry);
  els.endRz.value = formatNumber(pose?.rz);
}

function setCell(row, field, text, className) {
  const cell = row.querySelector(`[data-field='${field}']`);
  cell.textContent = text;
  cell.className = className;
}

function buildStatusTable() {
  document.getElementById("coordinateBody").innerHTML = motorIds
    .map(
      (id) => `
        <tr data-coordinate-row="${id}"><td>${id}</td>
        ${["raw_protocol_positions_deg", "raw_output_positions_rad", "software_output_offsets_rad", "model_output_positions_rad", "motor_error_codes", "motor_ages_sec", "fresh"].map((field) => `<td data-coordinate="${field}">--</td>`).join("")}</tr>
      `,
    )
    .join("");
  els.jointStatusBody.innerHTML = motorIds
    .map(
      (motorId, index) => `
        <tr data-status-row="${motorId}">
          <td>${activeJointNames[index]}</td>
          <td data-field="angle">--</td>
          <td data-field="speed">--</td>
          <td data-field="enabled" class="status-muted">--</td>
          <td data-field="error" class="status-muted">--</td>
          <td data-field="motion" class="status-muted">--</td>
          <td data-field="updated">--</td>
        </tr>
      `,
    )
    .join("");
  els.motorStatusBody.innerHTML = motorIds
    .map(
      (motorId) => `
        <tr data-motor-row="${motorId}">
          <td>${motorId}</td><td data-field="raw-angle">--</td>
          <td data-field="motor-enabled" class="status-muted">--</td>
          <td data-field="motor-error" class="status-muted">--</td>
          <td data-field="motor-fresh" class="status-muted">--</td>
        </tr>`,
    )
    .join("");
}

function buildSingleMotorControls() {
  els.motorList.innerHTML = motorIds
    .map(
      (motorId) => `
        <div class="motor-card">
          <div class="motor-title">J${motorId} <small>${activeJointNames[motorId - 1]} · CAN ${motorId}</small></div>
          <div class="motor-controls">
            <div class="control-row">
              <span class="input-pair">
                <label for="speed${motorId}">速度(°/s)</label>
                <input id="speed${motorId}" type="number" min="0.01" max="10" step="any" value="1">
              </span>
              <button class="btn-green" data-cmd="enable_motor" data-motor-id="${motorId}">使能</button>
              <button class="btn-red" data-cmd="disable_motor" data-motor-id="${motorId}">关闭</button>
            </div>
            <div class="control-row">
              <span class="input-pair">
                <label for="target${motorId}">目标(°)</label>
                <input id="target${motorId}" type="number" step="any" value="0">
              </span>
              <button class="btn-green" data-motion="true" data-action="move" data-joint-id="${motorId}">运动</button>
              <button class="btn-red" data-action="stop" data-joint-id="${motorId}">停止</button>
            </div>
          </div>
        </div>
      `,
    )
    .join("");
}

function buildJointInputs(container, prefix) {
  container.innerHTML = motorIds
    .map(
      (motorId) => `
        <div class="joint-row">
          <label for="${prefix}${motorId}">J${motorId} 目标 (°)</label>
          <input id="${prefix}${motorId}" type="number" step="any" value="0">
        </div>
      `,
    )
    .join("");
}

function bindTabs() {
  document.querySelectorAll(".tab-button").forEach((button) => {
    button.addEventListener("keydown", (event) => {
      const tabs = [...document.querySelectorAll(".tab-button")];
      const index = tabs.indexOf(button);
      if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key))
        return;
      event.preventDefault();
      const next =
        event.key === "Home"
          ? 0
          : event.key === "End"
            ? tabs.length - 1
            : (index + (event.key === "ArrowRight" ? 1 : -1) + tabs.length) %
              tabs.length;
      tabs[next].focus();
      tabs[next].click();
    });
    button.addEventListener("click", () => {
      const target = button.dataset.tab;
      document
        .querySelectorAll(".tab-button")
        .forEach((item) => item.classList.toggle("active", item === button));
      document
        .querySelectorAll(".tab-button")
        .forEach((item) =>
          item.setAttribute("aria-selected", String(item === button)),
        );
      document
        .querySelectorAll(".tab-panel")
        .forEach((panel) =>
          panel.classList.toggle("active", panel.id === target),
        );
    });
  });
}

function bindCommands() {
  document.querySelectorAll("input[name='motionMode']").forEach((radio) => {
    radio.addEventListener("change", updateMotionMode);
  });

  document.addEventListener("click", (event) => {
    const button = event.target.closest("button");
    if (!button || button.disabled) return;

    try {
      const cmd = button.dataset.cmd;
      const motorId = Number(button.dataset.motorId);
      if (cmd) {
        sendCommand(cmd, { motor_id: motorId });
        return;
      }

      const action = button.dataset.action;
      const jointId = Number(button.dataset.jointId);
      if (action === "move") {
        const speed = readPositiveFloat(`speed${jointId}`);
        const target = readNumber(`target${jointId}`);
        sendCommand("motor_move", {
          joint_id: jointId,
          target_angle: target,
          max_speed: speed,
        });
      }

      if (action === "stop") {
        sendCommand("motor_stop", { joint_id: jointId });
      }
    } catch (error) {
      log("参数错误", { message: error.message });
    }
  });

  els.moveAllBtn.addEventListener("click", moveSingleTargets);
  els.multiMoveBtn.addEventListener("click", moveAllMotors);
  els.multiStopBtn.addEventListener("click", stopAllMotors);
  els.connectBtn.addEventListener("click", connect);
  document
    .getElementById("exportDiagnosticBtn")
    .addEventListener("click", exportDiagnostics);
  document
    .getElementById("copyCurrentBtn")
    .addEventListener("click", copyCurrentTargets);
  els.disconnectBtn.addEventListener("click", () => ws?.close());
}

function updateMotionMode() {
  const mode =
    document.querySelector("input[name='motionMode']:checked")?.value ??
    "joint";
  els.jointParamCard.hidden = mode !== "joint";
  els.lineParamCard.hidden = mode !== "line";
}

function readNumber(id) {
  const input = document.getElementById(id);
  const value = Number(input?.value);
  if (
    !input ||
    !input.value.trim() ||
    !Number.isFinite(value) ||
    !input.checkValidity()
  ) {
    throw new Error(`${id} 必须填写有效数字（检查范围）`);
  }
  return value;
}

function readPositiveFloat(id) {
  const value = readNumber(id);
  if (value <= 0) throw new Error(`${id} 必须大于 0`);
  return value;
}

function copyCurrentTargets() {
  const angles = lastState?.current_joint_angles;
  if (
    !angles ||
    angles.length !== MOTOR_COUNT ||
    !angles.every(Number.isFinite)
  ) {
    log("读取失败：当前角度不可用");
    return;
  }
  angles.forEach((angle, index) => {
    // Preserve feedback precision, particularly near a joint limit.
    document.getElementById(`target${index + 1}`).value = String(angle);
    document.getElementById(`multiJoint${index + 1}`).value = String(angle);
  });
}

function moveSingleTargets() {
  try {
    const targets = motorIds.map((id) => readNumber(`target${id}`));
    const max_speed = Math.min(
      ...motorIds.map((id) => readPositiveFloat(`speed${id}`)),
    );
    sendCommand("move", { targets, max_speed });
  } catch (error) {
    log("参数错误", { message: error.message });
  }
}

function moveAllMotors() {
  try {
    const mode =
      document.querySelector("input[name='motionMode']:checked")?.value ??
      "joint";
    if (mode === "line") {
      const target = [
        readNumber("targetX"),
        readNumber("targetY"),
        readNumber("targetZ"),
        readNumber("targetRx"),
        readNumber("targetRy"),
        readNumber("targetRz"),
      ];
      const linear_vel = readPositiveFloat("endSpeed");
      const linear_acc = readPositiveFloat("lineAccel");
      const angular_vel = readPositiveFloat("angularSpeed");
      const angular_acc = readPositiveFloat("angularAccel");
      sendCommand("movel", {
        target,
        linear_vel,
        linear_acc,
        angular_vel,
        angular_acc,
      });
      return;
    }
    const target = motorIds.map((motorId) =>
      readNumber(`multiJoint${motorId}`),
    );
    const max_speed = readPositiveFloat("multiSpeed");
    sendCommand("move", { targets: target, max_speed });
  } catch (error) {
    log("参数错误", { message: error.message });
  }
}

function stopAllMotors() {
  sendCommand("stop_motion");
}

function init() {
  els.wsUrl.value = defaultWsUrl;
  buildStatusTable();
  buildSingleMotorControls();
  buildJointInputs(els.multiJointGrid, "multiJoint");
  document.getElementById("outputTargetGrid").innerHTML = motorIds
    .map(
      (id) =>
        `<label>输出轴 ${id} (°)<input id="outputTarget${id}" type="number" step="any" value="0"></label>`,
    )
    .join("");
  document.getElementById("copyOutputTargets").addEventListener("click", () => {
    const values = lastState?.model_output_positions_rad;
    if (!values || values.length !== 6 || !values.every(Number.isFinite))
      return;
    values.forEach((value, i) => {
      document.getElementById(`outputTarget${i + 1}`).value = String(
        (value * 180) / Math.PI,
      );
    });
  });
  document.getElementById("moveOutputTargets").addEventListener("click", () => {
    try {
      sendCommand("output_move", {
        targets: motorIds.map((id) => readNumber(`outputTarget${id}`)),
      });
    } catch (error) {
      log("参数错误", { message: error.message });
    }
  });
  bindTabs();
  bindCommands();
  bindAxisTests();
  document
    .querySelectorAll("[data-preset]")
    .forEach((button) =>
      button.addEventListener("click", () =>
        sendCommand("move_preset", { preset: button.dataset.preset }),
      ),
    );
  updateMotionMode();
  setConnectionState("closed");
  setInterval(refreshControlGate, 300);
  log("页面初始化完成", { motor_count: MOTOR_COUNT, ws_url: els.wsUrl.value });
}

init();
