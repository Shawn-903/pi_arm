/* Read-only planning and opt-in finite joint gestures; no separate transport. */
(() => {
  const $ = (id) => document.getElementById(id);
  let state = null,
    gate = {},
    result = null,
    gesture = null,
    generation = 0;
  let calculating = false,
    playing = false,
    playFrame = 0,
    previousContext = "";
  let requestKey = null;
  const sliders = [];
  const status = (text, error = false) => {
    $("planningStatus").textContent = text;
    $("planningStatus").className = error ? "status-error" : "";
  };
  const number = (id) => {
    const s = $(id).value.trim(),
      v = Number(s);
    if (!s || !Number.isFinite(v)) throw new Error(`${id} 请输入有限数值`);
    return v;
  };
  const current = () => {
    const view = window.PiArmViewer?.snapshot();
    if (!view?.valid || !view.angles || !gate.fresh) return null;
    return view;
  };
  function stopPlayback() {
    playing = false;
    cancelAnimationFrame(playFrame);
    $("playPreview").textContent = "回放 · 10 s";
  }
  function clear(message = "目标已变化，请重新计算预览") {
    ++generation;
    result = null;
    requestKey = null;
    stopPlayback();
    $("pathPlayback").hidden = true;
    $("previewJointResult").replaceChildren();
    window.PiArmViewer?.setPreview(null);
    status(message);
  }
  function disarm(message) {
    $("releaseExecute").checked = false;
    endGesture(true);
    $("gestureStatus").textContent = message;
  }
  function beginGesture(axis, initial, owner) {
    if (gesture || !owner) return false;
    const view = current();
    if (
      !view ||
      !state?.preview_modes?.includes("joint") ||
      !view.limits ||
      calculating ||
      state.read_only ||
      state.commissioning ||
      state.task?.status === 1 ||
      ($("releaseExecute").checked && !gate.permitted)
    )
      return false;
    clear("单轴目标预览；未执行规划检查");
    const target = view.angles.map((q, i) =>
      Math.max(view.limits[i].lower, Math.min(view.limits[i].upper, q)),
    );
    gesture = {
      axis,
      owner,
      initial,
      value: initial,
      target,
      context: previousContext,
      execute: $("releaseExecute").checked,
      moved: false,
    };
    return true;
  }
  function updateGesture(value, owner) {
    if (!gesture || gesture.owner !== owner || !Number.isFinite(value)) return;
    const view = current();
    if (!view || (gesture.execute && !gate.permitted)) {
      endGesture(true);
      return;
    }
    const axis = gesture.axis,
      limit = view.limits[axis];
    value =
      Math.round(Math.max(limit.lower, Math.min(limit.upper, value)) * 100) /
      100;
    gesture.value = value;
    gesture.moved = Math.abs(value - gesture.initial) > 0.001;
    gesture.target[axis] = value;
    $(`target${axis + 1}`).value = value;
    sliders[axis].value = value;
    $(`jointSliderValue${axis + 1}`).textContent = `${value.toFixed(2)}°`;
    window.PiArmViewer.setPreview(
      gesture.target,
      [],
      `J${axis + 1} 目标 ${value.toFixed(2)}° · ${gesture.execute ? "松手后提交运动" : "仅预览"}`,
    );
    $("gestureStatus").textContent =
      `J${axis + 1} → ${value.toFixed(2)}° · ${gesture.execute ? "松手执行；Esc 取消" : "仅预览；执行请使用单轴运动按钮"}`;
  }
  function endGesture(cancel = false, owner = null) {
    if (owner && gesture?.owner !== owner) return;
    const done = gesture;
    gesture = null;
    if (!done) return;
    if (cancel) {
      clear("手势已取消，未下发运动");
      $("gestureStatus").textContent = "手势已取消，未下发运动";
      return;
    }
    if (!done.moved || !done.execute) return;
    if (
      !gate.permitted ||
      !current() ||
      done.context !== previousContext ||
      !$("releaseExecute").checked ||
      document.hidden ||
      !document.hasFocus()
    ) {
      disarm("状态变化，松手执行已关闭");
      return;
    }
    // Backend resolves the other five joints from feedback and repeats all checks.
    const sent = window.PiArmControl.moveJoint(done.axis, done.value);
    $("gestureStatus").textContent = sent
      ? `已提交 J${done.axis + 1} 有限目标，等待到位结果`
      : "目标未发送，请检查连接、限速及任务状态";
  }
  function parameters(mode) {
    const p = { mode, output_speed_limit: number("outputSpeedLimit") };
    if (mode === "joint") {
      const multi = document
        .getElementById("multi")
        .classList.contains("active");
      p.targets = Array.from({ length: 6 }, (_, i) =>
        number(`${multi ? "multiJoint" : "target"}${i + 1}`),
      );
      p.max_speed = multi
        ? number("multiSpeed")
        : Math.min(
            ...Array.from({ length: 6 }, (_, i) => number(`speed${i + 1}`)),
          );
    } else if (mode === "output") {
      p.targets = Array.from({ length: 6 }, (_, i) =>
        number(`outputTarget${i + 1}`),
      );
    } else {
      p.target = ["X", "Y", "Z", "Rx", "Ry", "Rz"].map((v) =>
        number(`target${v}`),
      );
      if (mode === "lin")
        Object.assign(p, {
          linear_vel: number("endSpeed"),
          linear_acc: number("lineAccel"),
          angular_vel: number("angularSpeed"),
        });
    }
    return p;
  }
  async function calculate(mode) {
    if (calculating) return;
    clear("正在计算，只读预览…");
    const token = generation;
    try {
      const params = parameters(mode);
      requestKey = JSON.stringify(params);
      calculating = true;
      refresh(state, gate);
      const data = await window.PiArmControl.preview(params);
      if (token !== generation) return;
      const view = current();
      if (
        !view ||
        data.joint_names?.join() !== view.jointNames.join() ||
        data.start_joint_angles_deg?.length !== 6 ||
        data.target_joint_angles_deg?.length !== 6 ||
        data.start_joint_angles_deg.some(
          (q, i) => Math.abs(q - view.angles[i]) > 0.2,
        )
      ) {
        throw new Error("预览起点与当前反馈不一致，请重新计算");
      }
      result = data;
      window.PiArmViewer.setPreview(
        data.target_joint_angles_deg,
        data.points,
        "后端解算目标 · 只读预览",
      );
      const explanation = {
        joint: "关节插值路径（无碰撞检查）",
        output: "输出轴反解 / 关节插值路径（无碰撞检查）",
        ik: "IK 端点已解算（场景碰撞检查）；不代表路径可达",
        lin: "Pilz LIN 规划成功；执行时重新规划",
      }[mode];
      status(
        explanation +
          (data.points.length
            ? ` · ${data.points.at(-1).t.toFixed(1)} s · ${data.points.length} 个显示采样点`
            : ""),
      );
      data.target_joint_angles_deg.forEach((q, i) => {
        const row = document.createElement("span");
        row.textContent = `J${i + 1} ${q.toFixed(2)}°`;
        row.title = `输出轴 ${data.target_output_angles_deg[i].toFixed(2)}°`;
        $("previewJointResult").appendChild(row);
      });
      $("pathPlayback").hidden = data.points.length < 2;
      $("pathScrub").max = Math.max(0, data.points.length - 1);
      $("pathScrub").value = $("pathScrub").max;
      scrub();
    } catch (error) {
      if (token === generation) {
        clear();
        status(`预览失败：${error.message}`, true);
      }
    } finally {
      calculating = false;
      refresh(state, gate);
    }
  }
  function scrub() {
    if (!result?.points.length) return;
    const index = Math.max(
      0,
      Math.min(result.points.length - 1, Number($("pathScrub").value)),
    );
    const p = result.points[index];
    window.PiArmViewer.scrub(p.joints);
    $("pathTime").textContent =
      `${p.t.toFixed(1)} / ${result.points.at(-1).t.toFixed(1)} s`;
  }
  function refresh(next, options) {
    state = next;
    gate = options;
    const view = window.PiArmViewer?.snapshot();
    const context = JSON.stringify([
      state?.hardware_type,
      state?.read_only,
      state?.commissioning,
      state?.calibration_id,
      gate.connectionState,
    ]);
    if (context !== previousContext) {
      previousContext = context;
      disarm("连接或控制模式变化，松手执行已关闭");
      if (result || calculating) clear("连接或控制模式变化，请重新计算");
    }
    if (requestKey) {
      try {
        const keyMode = JSON.parse(requestKey).mode;
        if (JSON.stringify(parameters(keyMode)) !== requestKey) clear();
      } catch {
        clear("参数无效，旧预览已作废");
      }
    }
    const usable =
      gate.fresh &&
      view?.valid &&
      !state?.read_only &&
      !state?.commissioning &&
      state?.preview_modes?.includes("joint");
    const busy = state?.task?.status === 1 || state?.robot_state === "RUNNING";
    $("releaseExecute").disabled = !usable || !gate.permitted || calculating;
    $("modelInteraction").disabled = !usable || busy || calculating;
    if (!usable || busy) {
      endGesture(true);
      if (!usable) $("releaseExecute").checked = false;
      if (result || calculating)
        clear("反馈不可用或机械臂正在运动，请静止后重新预览");
    }
    if (
      result &&
      view?.angles &&
      result.start_joint_angles_deg.some(
        (q, i) => Math.abs(q - view.angles[i]) > 0.2,
      )
    ) {
      clear("起点已变化超过 0.2°，旧路径已作废");
    }
    document.querySelectorAll("[data-preview]").forEach((button) => {
      button.disabled =
        !usable ||
        busy ||
        calculating ||
        !state?.preview_modes?.includes(button.dataset.preview);
    });
    sliders.forEach((slider, i) => {
      slider.disabled = !usable || busy || calculating;
      const lim = view?.limits?.[i];
      if (lim) {
        slider.min = lim.lower;
        slider.max = lim.upper;
      }
      if (gesture?.axis !== i) {
        slider.value = $(`target${i + 1}`).value;
        $(`jointSliderValue${i + 1}`).textContent =
          `${Number(slider.value).toFixed(2)}°`;
      }
    });
  }
  for (let i = 0; i < 6; ++i) {
    const block = document.createElement("div");
    block.className = "joint-slider";
    block.innerHTML = `<label for="jointSlider${i + 1}">拖动 J${i + 1}<output id="jointSliderValue${i + 1}">0.00°</output></label><input id="jointSlider${i + 1}" type="range" min="-180" max="180" step="0.1" value="0" disabled>`;
    $(`target${i + 1}`)
      .closest(".motor-card")
      .appendChild(block);
    const slider = block.querySelector("input");
    sliders.push(slider);
    let pointerId = null,
      gestureKey = null;
    slider.addEventListener("pointerdown", (e) => {
      if (
        e.button !== 0 ||
        pointerId !== null ||
        !beginGesture(i, Number(slider.value), slider)
      ) {
        e.preventDefault();
        return;
      }
      pointerId = e.pointerId;
      slider.setPointerCapture(e.pointerId);
    });
    slider.addEventListener("input", () =>
      updateGesture(Number(slider.value), slider),
    );
    const finishPointer = (e, cancel) => {
      if (pointerId !== e.pointerId) return;
      pointerId = null;
      endGesture(cancel, slider);
    };
    slider.addEventListener("pointerup", (e) => finishPointer(e, false));
    slider.addEventListener("pointercancel", (e) => finishPointer(e, true));
    slider.addEventListener("lostpointercapture", (e) =>
      finishPointer(e, true),
    );
    slider.addEventListener("keydown", (e) => {
      if (
        ![
          "ArrowLeft",
          "ArrowRight",
          "ArrowUp",
          "ArrowDown",
          "Home",
          "End",
          "PageUp",
          "PageDown",
        ].includes(e.key)
      )
        return;
      if (
        pointerId !== null ||
        (gestureKey && gestureKey !== e.key) ||
        (gesture && gesture.owner !== slider) ||
        (!gesture && e.repeat)
      ) {
        e.preventDefault();
        return;
      }
      if (!gestureKey) {
        if (beginGesture(i, Number(slider.value), slider)) gestureKey = e.key;
        else e.preventDefault();
      } else if (!gesture) e.preventDefault();
    });
    slider.addEventListener("keyup", (e) => {
      if (gestureKey !== e.key) return;
      gestureKey = null;
      if (pointerId === null) endGesture(false, slider);
    });
    slider.addEventListener("blur", () => {
      gestureKey = null;
      endGesture(true, slider);
    });
  }
  document
    .querySelectorAll("[data-preview]")
    .forEach((button) =>
      button.addEventListener("click", () => calculate(button.dataset.preview)),
    );
  $("clearPreview").addEventListener("click", () => clear("预览已清除"));
  $("releaseExecute").checked = false;
  $("releaseExecute").addEventListener("change", () => {
    endGesture(true);
    $("gestureStatus").textContent = $("releaseExecute").checked
      ? "松手执行已开启：每次滑块 / 连杆拖动提交一个单轴目标，速度使用该轴及输出轴限速。"
      : "仅预览，不发送运动";
  });
  $("pathScrub").addEventListener("input", () => {
    stopPlayback();
    scrub();
  });
  $("playPreview").addEventListener("click", () => {
    if (playing) {
      stopPlayback();
      return;
    }
    if (!result?.points.length) return;
    playing = true;
    $("playPreview").textContent = "暂停";
    if (Number($("pathScrub").value) >= result.points.length - 1)
      $("pathScrub").value = 0;
    const startIndex = Number($("pathScrub").value),
      startTime = performance.now();
    const tick = (time) => {
      if (!playing || !result) return;
      // Condense long trajectories to ten seconds; display physical time separately.
      $("pathScrub").value = Math.min(
        result.points.length - 1,
        startIndex +
          Math.floor(((time - startTime) / 10000) * (result.points.length - 1)),
      );
      scrub();
      if (Number($("pathScrub").value) >= result.points.length - 1)
        stopPlayback();
      else playFrame = requestAnimationFrame(tick);
    };
    playFrame = requestAnimationFrame(tick);
  });
  document.addEventListener("input", (e) => {
    if (
      /^(target[1-6XYZ]|targetR[xyz]|multiJoint[1-6]|outputTarget[1-6]|speed[1-6]|multiSpeed|outputSpeedLimit|endSpeed|lineAccel|angularSpeed|angularAccel)$/.test(
        e.target.id,
      )
    ) {
      endGesture(true);
      if (result || calculating) clear();
    }
  });
  $("viewerPreview").addEventListener("change", () => {
    if ($("viewerPreview").value !== "interactive" && (result || calculating)) {
      const source = $("viewerPreview").value;
      clear("已切换目标显示");
      $("viewerPreview").value = source;
      $("viewerPreview").dispatchEvent(new Event("change"));
    }
  });
  document.addEventListener("click", (e) => {
    if (e.target.closest("[data-view-target]"))
      disarm("切换工作区，松手执行已关闭");
  });
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") disarm("已取消手势并关闭松手执行");
  });
  window.addEventListener("blur", () => {
    disarm("窗口失焦，松手执行已关闭");
    stopPlayback();
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      disarm("页面隐藏，松手执行已关闭");
      stopPlayback();
    }
  });
  window.PiArmInteractive = {
    cancelActions(message) {
      disarm(message);
      clear(message);
    },
    refresh,
    beginGesture,
    updateGesture,
    endGesture,
  };
})();
