/* Teaching data only. Loading a point never submits a motion request. */
(() => {
  const $ = (id) => document.getElementById(id);
  const jointNames = [
    "joint_01",
    "joint_02",
    "joint_04",
    "joint_05",
    "joint_06",
    "joint_09",
  ];
  const cartAxes = ["X", "Y", "Z", "Rx", "Ry", "Rz"];
  let library = { revision: 0, points: [] },
    busy = false,
    available = false,
    state = null,
    fresh = false;
  let connectedBefore = false,
    generation = 0;
  const status = (message, error = false) => {
    $("pointStatus").textContent = message;
    $("pointStatus").className = error ? "status-error" : "";
  };
  const finite = (v) => typeof v === "number" && Number.isFinite(v);
  const fmt = (v) => (finite(v) ? v.toFixed(3) : "--");
  const inputValues = (prefix) =>
    Array.from({ length: 6 }, (_, i) => {
      const input = $(prefix + (i + 1)),
        value = Number(input.value);
      if (!input.value.trim() || !finite(value))
        throw Error("请填写六个有限数值");
      return value;
    });
  function buildInputs() {
    const joint = $("pointKind").value === "joint";
    $("pointValues").replaceChildren();
    Array.from({ length: 6 }, (_, i) => {
      const label = document.createElement("label"),
        input = document.createElement("input");
      label.textContent = joint
        ? `J${i + 1} (°)`
        : `${cartAxes[i]} (${i < 3 ? "mm" : "°"})`;
      input.id = `pointValue${i + 1}`;
      input.type = "number";
      input.step = "any";
      input.value = "0";
      label.append(input);
      $("pointValues").append(label);
    });
  }
  function compatibility(point) {
    if (point.hardware_type !== state?.hardware_type)
      return "实机与模拟点位不能混用";
    if (point.model_id !== state?.base_frames?.model_id)
      return "模型版本不同，请重新核验并录入";
    if (point.joint_names?.join() !== jointNames.join())
      return "关节顺序不匹配";
    return "";
  }
  function render() {
    $("pointCount").textContent =
      `${library.points.length} 个点位 · 版本 ${library.revision}`;
    $("pointList").replaceChildren();
    if (!library.points.length) {
      const empty = document.createElement("p");
      empty.className = "point-empty";
      empty.textContent = "尚无点位。录入测试目标，或在机械臂静止后记录反馈。";
      $("pointList").append(empty);
    }
    for (const point of library.points) {
      const row = document.createElement("article");
      row.className = "point-card";
      row.dataset.pointId = point.id;
      const title = document.createElement("strong");
      title.textContent = point.name;
      const meta = document.createElement("small");
      meta.textContent = `${point.kind === "joint" ? "六关节 · °" : "笛卡尔 · mm / °"} · ${{ manual: "手动录入", feedback: "反馈示教", model: "模型目标" }[point.source] || point.source} · ${new Date(point.created_at).toLocaleString()}`;
      const values = document.createElement("p");
      values.className = "point-vector";
      values.textContent = point.values.map(fmt).join(" / ");
      const note = document.createElement("p");
      note.textContent = point.note || "无备注";
      const buttons = document.createElement("div");
      buttons.className = "point-actions";
      const load = document.createElement("button");
      load.textContent = "加载到运动表单";
      load.dataset.pointAction = "load";
      load.disabled = !available || busy || Boolean(compatibility(point));
      load.title = compatibility(point) || "仅填入目标，不执行运动";
      load.addEventListener("click", () => loadPoint(point));
      const remove = document.createElement("button");
      remove.textContent = "删除";
      remove.dataset.pointAction = "delete";
      remove.className = "btn-secondary";
      remove.disabled = !available || busy;
      remove.addEventListener("click", () =>
        request("point_delete", { id: point.id }, `已删除 ${point.name}`),
      );
      buttons.append(load, remove);
      row.append(title, meta, values, note, buttons);
      $("pointList").append(row);
    }
  }
  function loadPoint(point) {
    const mismatch = compatibility(point);
    if (mismatch) {
      status(mismatch, true);
      return;
    }
    window.PiArmControl.loadPointTarget(point.kind, point.values);
    document.querySelector('[data-view-target="workbench"]').click();
    document
      .querySelector(
        `[data-tab="${point.kind === "joint" ? "multi" : "cartesian"}"]`,
      )
      .click();
    const changed =
      point.calibration_id && point.calibration_id !== state?.calibration_id;
    const message = `已加载「${point.name}」到${point.kind === "joint" ? "六关节" : "笛卡尔"}目标；尚未执行。${changed ? "标定 ID 已变化，请重新核验点位后再执行。" : !point.calibration_id ? "此点位未关联标定，请核验目标。" : "请预览并使用对应运动按钮执行。"}`;
    $("loadedPointStatus").hidden = false;
    $("loadedPointStatus").textContent = message;
    $("loadedPointStatus").dataset.pointId = point.id;
    status(message, Boolean(changed));
  }
  async function request(cmd, params = {}, message = "点位列表已更新") {
    if (busy || !available) return;
    const token = generation;
    busy = true;
    controls();
    render();
    try {
      const data = await window.PiArmControl.library(cmd, {
        ...params,
        revision: library.revision,
      });
      if (token !== generation) return;
      library = data;
      status(message);
    } catch (error) {
      if (token === generation) status(error.message, true);
    } finally {
      if (token === generation) {
        busy = false;
        render();
        controls();
      }
    }
  }
  function save(source) {
    try {
      let kind = $("pointKind").value;
      const params = {
        name: $("pointName").value.trim(),
        note: $("pointNote").value,
        kind,
        source,
      };
      if (!params.name) throw Error("请先填写点位名称");
      if (source === "manual") params.values = inputValues("pointValue");
      if (source === "model") {
        const preview = window.PiArmViewer?.snapshot()?.previewTargetAngles;
        if (
          !Array.isArray(preview) ||
          preview.length !== 6 ||
          !preview.every(finite)
        )
          throw Error("请先拖动模型或计算目标预览，再保存");
        params.kind = "joint";
        params.values = preview.slice();
      }
      request("point_save", params, `已保存「${params.name}」，未执行运动`);
    } catch (error) {
      status(error.message, true);
    }
  }
  function controls() {
    $("refreshPoints").disabled = !available || busy;
    $("saveManualPoint").disabled = !available || busy;
    $("capturePoint").textContent =
      state?.hardware_type === "mock" ? "记录当前模拟反馈" : "记录当前实机反馈";
    $("capturePoint").disabled =
      !available ||
      busy ||
      !fresh ||
      !state?.base_frames?.valid ||
      !state?.calibrated ||
      state?.task?.status === 1 ||
      state?.axis_test?.active ||
      state?.motion_stop_pending ||
      state?.motion_stop_failed ||
      state?.motor_in_motion?.length !== 6 ||
      state.motor_in_motion.some(Boolean) ||
      state?.motor_error_codes?.some(Boolean);
    $("saveModelPoint").disabled =
      !available ||
      busy ||
      !fresh ||
      !window.PiArmViewer?.snapshot()?.previewTargetAngles;
    $("exportPoints").disabled = busy || !library.points.length;
  }
  const frameRows = Array.from({ length: 6 }, (_, i) => {
    const row = document.createElement("tr");
    row.append(document.createElement("th"));
    for (let j = 0; j < 6; j++) row.append(document.createElement("td"));
    row.firstChild.textContent = `J${i + 1}`;
    $("baseFrameBody").append(row);
    return row;
  });
  window.PiArmTeaching = {
    refresh(next, gate) {
      state = next;
      fresh = Boolean(gate.fresh);
      available =
        gate.connectionState === "connected" && Boolean(next?.points_available);
      const good = fresh && next?.base_frames?.valid;
      frameRows.forEach((row, i) => {
        const frame = good ? next.base_frames.joints[i] : null;
        row.firstChild.textContent = `J${i + 1}${frame ? " / " + frame.frame_id : ""}`;
        const values = frame ? [...frame.position_mm, ...frame.rpy_deg] : [];
        [...row.children].slice(1).forEach((cell, j) => {
          cell.textContent = fmt(values[j]);
        });
      });
      $("baseFrameStatus").textContent = good
        ? "与当前关节反馈同帧计算 · RPY 存在欧拉角奇异性，点位数据保留坐标定义"
        : "反馈过期、未连接或解算无效；不显示旧姿态为当前姿态";
      const changed = available !== connectedBefore;
      if (changed) {
        connectedBefore = available;
        ++generation;
        busy = false;
        library = { revision: 0, points: [] };
        render();
        if (available) request("point_list");
        else status("点位库连接不可用；请连接新版服务");
      }
      controls();
    },
  };
  $("pointKind").addEventListener("change", buildInputs);
  $("saveManualPoint").addEventListener("click", () => save("manual"));
  $("capturePoint").addEventListener("click", () => save("feedback"));
  $("saveModelPoint").addEventListener("click", () => save("model"));
  $("refreshPoints").addEventListener("click", () => request("point_list"));
  $("exportPoints").addEventListener("click", () => {
    const url = URL.createObjectURL(
      new Blob([JSON.stringify(library, null, 2)], {
        type: "application/json",
      }),
    );
    const link = document.createElement("a");
    link.href = url;
    link.download = `pi-arm-points-${state?.hardware_type || "unknown"}.json`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  buildInputs();
  render();
  controls();
})();
