/* Browser-owned finite cycles. Each leg uses the existing backend motion/arrival chain. */
(() => {
  const $ = (id) => document.getElementById(id);
  const clone = (value) => structuredClone(value);
  const vector = (value) => Array.isArray(value) && value.length === 6 && value.every(Number.isFinite);
  let points = [], state = null, gate = {}, run = null;
  const inputIds = ['repeatStartPoint', 'repeatTargetPoint', 'repeatCycles', 'repeatDwell',
    'repeatSpeed', 'repeatOutputSpeed', 'repeatLinearSpeed'];
  const status = (text, error = false) => {
    $('repeatStatus').textContent = text;
    $('repeatStatus').className = error ? 'status-error' : '';
  };
  function pointError(point) {
    if (!point || !vector(point.values)) return '请选择有效点位';
    if (point.hardware_type !== state?.hardware_type) return '实机与模拟点位不能混用';
    if (!state?.base_frames?.model_id || point.model_id !== state.base_frames.model_id)
      return '模型版本不一致，请重新核验并录入点位';
    if (!state?.calibration_id || point.calibration_id !== state.calibration_id)
      return '标定编号不一致，请在当前标定下重新核验并录入点位';
    if (point.base_frame !== 'base_link' || point.tool_frame !== 'tool0' ||
        point.joint_names?.join() !== state?.joint_names?.join()) return '点位坐标系或关节顺序不匹配';
    if (!['joint', 'cartesian'].includes(point.kind)) return '不支持的点位类型';
    if (!state?.supported_motion_commands?.includes(point.kind === 'joint' ? 'move' : 'movel'))
      return '服务不支持该点位的运动方式';
    return '';
  }
  function selection() {
    const start = points.find(p => p.id === $('repeatStartPoint').value);
    const target = points.find(p => p.id === $('repeatTargetPoint').value);
    let error = pointError(start) || pointError(target);
    if (!error && (start.id === target.id || start.values.every((v, i) => v === target.values[i])))
      error = '起始点和目标点必须不同';
    if (!error && start.kind !== target.kind) error = '请选择两个同类型点位（六关节或笛卡尔）';
    return { start, target, error };
  }
  function number(id) {
    const input = $(id), value = Number(input.value);
    if (!input.value.trim() || !input.checkValidity() || !Number.isFinite(value))
      throw Error(`${input.closest('label').firstChild.textContent.trim()}超出范围或未填写`);
    return value;
  }
  function stationary() {
    return state?.motor_fresh?.length === 6 && state.motor_fresh.every(Boolean) &&
      state?.motor_in_motion?.length === 6 && state.motor_in_motion.every(v => v === false);
  }
  function render() {
    const chosen = selection(), active = Boolean(run?.active);
    inputIds.forEach(id => { $(id).disabled = active; });
    $('repeatLinearLabel').hidden = chosen.start?.kind !== 'cartesian';
    $('repeatSelection').textContent = chosen.error ||
      `${chosen.start.name} → ${chosen.target.name} · ${chosen.start.kind === 'joint' ? '六关节运动' : '笛卡尔直线'} · ${state.calibration_id}`;
    $('repeatStart').disabled = active || !gate.permitted || !stationary() || Boolean(chosen.error);
    $('repeatStop').disabled = !active;
    $('repeatExport').disabled = !run;
    $('repeatCsv').disabled = !run?.samples.length;
    $('repeatProgress').max = run?.config.cycles || 10;
    $('repeatProgress').value = run?.samples.length || 0;
    const samples = run?.samples || [];
    let summary = `目标点记录：${samples.length}${run ? ' / ' + run.config.cycles : ''} 次`;
    if (samples.length > 1) {
      const range = [0, 1, 2].map(i => {
        const values = samples.map(s => s.pose_mm_deg[i]);
        return (Math.max(...values) - Math.min(...values)).toFixed(3);
      });
      summary += ` · 解算位置极差 X / Y / Z：${range.join(' / ')} mm（非外部实测精度）`;
    }
    $('repeatSummary').textContent = summary;
  }
  function cancel(reason, stop = true) {
    if (!run?.active) return;
    run.active = false;
    run.result = 'interrupted';
    run.finished_at = new Date().toISOString();
    run.reason = reason;
    status(`${reason}；已记录 ${run.samples.length} 次，不会自动续跑。`, true);
    if (stop) {
      run.stop_requested = window.PiArmControl.stop();
      if (!run.stop_requested) status(`${reason}；停止指令未能发送，当前运动结果未知。`, true);
    }
    render();
  }
  async function sendLeg() {
    const owner = run;
    if (!owner?.active) return;
    if (!gate.permitted || !stationary()) {
      cancel('运动前检查未通过：反馈或控制许可不可用');
      return;
    }
    const point = owner.config[owner.leg];
    owner.phase = 'sending';
    owner.task_id = null;
    owner.deadline = performance.now() + 30 * 60 * 1000;
    status(`第 ${owner.samples.length + 1} / ${owner.config.cycles} 轮 · 正在前往${owner.leg === 'start' ? '起始点' : '目标点'}「${point.name}」`);
    try {
      const speed = owner.config.speed;
      const params = point.kind === 'joint'
        ? { targets: point.values, max_speed: speed }
        : { target: point.values, linear_vel: owner.config.linear_speed,
            linear_acc: 10, angular_vel: speed, angular_acc: 10 };
      params.output_speed_limit = owner.config.output_speed;
      const result = await window.PiArmControl.repeatMove(point.kind === 'joint' ? 'move' : 'movel', params);
      if (!owner.active || run !== owner) {
        // An accepted reply may race a stop request. Never schedule another leg.
        window.PiArmControl.stop();
        return;
      }
      if (!Number.isSafeInteger(result.task_id) || result.task_id <= 0) throw Error('未返回有效任务编号');
      owner.task_id = result.task_id;
      owner.phase = 'moving';
    } catch (error) {
      if (owner.active && run === owner) cancel(error.message);
    }
  }
  function start() {
    try {
      if (run?.active) return;
      const chosen = selection();
      if (chosen.error) throw Error(chosen.error);
      if (!gate.permitted || !stationary() || document.hidden) throw Error('请保持页面在前台，等待静止、使能及新鲜反馈');
      const config = { start: clone(chosen.start), target: clone(chosen.target),
        cycles: number('repeatCycles'), dwell: number('repeatDwell'),
        speed: number('repeatSpeed'), output_speed: number('repeatOutputSpeed'),
        linear_speed: number('repeatLinearSpeed') };
      window.PiArmInteractive?.cancelActions('开始重复定位测试，松手执行已关闭');
      run = { active: true, result: 'running', started_at: new Date().toISOString(),
        hardware_type: state.hardware_type, calibration_id: state.calibration_id,
        model_id: state.base_frames.model_id, config, samples: [], leg: 'start', phase: 'ready' };
      render();
      void sendLeg();
    } catch (error) { status(error.message, true); }
  }
  function sample() {
    const tool = state.base_frames?.tool;
    const pose = tool ? [...tool.position_mm, ...tool.rpy_deg] : null;
    if (!vector(state.current_joint_angles) || !vector(pose) ||
        !vector(state.task.target_joint_angles_deg) || !vector(state.task.error_joint_angles_deg) ||
        !vector(state.raw_protocol_positions_deg))
      throw Error('到位记录不完整，停止测试');
    run.samples.push({ cycle: run.samples.length + 1, task_id: run.task_id,
      captured_at: new Date().toISOString(), joint_angles_deg: clone(state.current_joint_angles),
      pose_mm_deg: pose, raw_protocol_positions_deg: clone(state.raw_protocol_positions_deg),
      target_joint_angles_deg: clone(state.task.target_joint_angles_deg),
      sampled_error_joint_angles_deg: state.task.target_joint_angles_deg.map((v, i) => Number((v - state.current_joint_angles[i]).toFixed(3))),
      verification_error_joint_angles_deg: clone(state.task.error_joint_angles_deg),
      goal_tolerance_deg: state.task.goal_tolerance_deg, arrival_state: state.task.arrival_state });
  }
  function tick() {
    if (!run?.active) return;
    if (!gate.fresh || gate.connectionState !== 'connected') { cancel('连接中断或反馈过期'); return; }
    if (state.read_only || state.commissioning || state.teaching_active || state.any_error ||
        !state.calibrated || !state.kinematics_valid || state.motion_stop_pending || state.motion_stop_failed ||
        state.motor_enabled?.length !== 6 || !state.motor_enabled.every(Boolean) || !state.base_frames?.valid ||
        pointError(run.config.start) || pointError(run.config.target)) {
      cancel('标定、模型、使能或运动状态变化'); return;
    }
    if (performance.now() > run.deadline) { cancel('单段运动或等待超过 30 分钟'); return; }
    if (run.phase === 'sending') return;
    const task = state.task;
    if (task.task_id !== run.task_id) {
      if (task.task_id > run.task_id) cancel('检测到其他运动任务，循环中止');
      return; // Wait for the accepted task to appear in state feedback.
    }
    if ([3, 4].includes(task.status)) { cancel(`运动未完成：${task.message || task.arrival_state}`); return; }
    if (task.status !== 2) return;
    if (task.arrival_state !== 'reached') { cancel('任务结束但未通过到位核验'); return; }
    if (!stationary()) {
      if (run.phase === 'dwell') cancel('等待测量期间反馈失效或电机再次运动');
      return;
    }
    if (run.phase === 'moving') {
      run.phase = 'dwell';
      run.dwell_until = performance.now() + run.config.dwell * 1000;
      // The dwell setting may be longer than the motion timeout.
      run.deadline = run.dwell_until + 30000;
    }
    const remaining = Math.max(0, (run.dwell_until - performance.now()) / 1000);
    status(`第 ${run.samples.length + 1} / ${run.config.cycles} 轮 · ${run.leg === 'start' ? '起始点' : '目标点'}已到位 · 等待 ${remaining.toFixed(1)} s`);
    if (remaining > 0 || !gate.permitted) return;
    if (run.leg === 'target') {
      try { sample(); } catch (error) { cancel(error.message); return; }
      if (run.samples.length >= run.config.cycles) {
        run.active = false;
        run.result = 'completed';
        run.finished_at = new Date().toISOString();
        status(`已完成 ${run.samples.length} 轮，停在目标点保持。请导出记录。`);
        render();
        return;
      }
    }
    run.leg = run.leg === 'start' ? 'target' : 'start';
    void sendLeg();
  }
  function download(csv) {
    if (!run) return;
    const report = { schema_version: 1, measurement_source: 'encoder_cable_model_not_external_metrology',
      base_frame: 'base_link', tool_frame: 'tool0', rpy_convention: 'Rz(yaw) Ry(pitch) Rx(roll)', ...clone(run) };
    let content = JSON.stringify(report, null, 2);
    if (csv) {
      const quote = value => '"' + String(value ?? '').replaceAll('"', '""') + '"';
      const headers = ['cycle', 'captured_at', 'task_id', 'calibration_id',
        ...Array.from({length: 6}, (_, i) => `J${i+1}_deg`), 'X_mm', 'Y_mm', 'Z_mm', 'Rx_deg', 'Ry_deg', 'Rz_deg',
        ...Array.from({length: 6}, (_, i) => `sampled_error_J${i+1}_deg`),
        ...Array.from({length: 6}, (_, i) => `verification_error_J${i+1}_deg`)];
      content = '\ufeff' + [headers, ...run.samples.map(s => [s.cycle, s.captured_at, s.task_id,
        run.calibration_id, ...s.joint_angles_deg, ...s.pose_mm_deg,
        ...s.sampled_error_joint_angles_deg, ...s.verification_error_joint_angles_deg])]
        .map(row => row.map(v => quote(typeof v === 'number' && !Number.isInteger(v) ? v.toFixed(3) : v)).join(',')).join('\r\n');
    }
    const url = URL.createObjectURL(new Blob([content], {type: csv ? 'text/csv;charset=utf-8' : 'application/json'}));
    const a = document.createElement('a'); a.href = url;
    a.download = `pi-arm-repeat-${run.started_at.replaceAll(':', '-')}.${csv ? 'csv' : 'json'}`;
    a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  }
  window.PiArmRepeat = {
    get active() { return Boolean(run?.active); },
    cancel,
    setLibrary(library) {
      points = clone(library.points);
      for (const id of ['repeatStartPoint', 'repeatTargetPoint']) {
        const select = $(id), previous = select.value;
        select.replaceChildren(new Option('请选择点位', ''));
        for (const point of points) select.add(new Option(`${point.name} · ${point.kind === 'joint' ? '六关节' : '笛卡尔'}`, point.id));
        if (points.some(p => p.id === previous)) select.value = previous;
      }
      render();
    },
    refresh(next, nextGate) { state = next; gate = nextGate; tick(); render(); },
  };
  inputIds.forEach(id => $(id).addEventListener('change', render));
  $('repeatStart').addEventListener('click', start);
  $('repeatStop').addEventListener('click', () => cancel('用户停止测试'));
  $('repeatExport').addEventListener('click', () => download(false));
  $('repeatCsv').addEventListener('click', () => download(true));
  document.addEventListener('visibilitychange', () => { if (document.hidden) cancel('页面已进入后台'); });
  window.addEventListener('pagehide', () => cancel('页面关闭或刷新'));
  window.addEventListener('beforeunload', (event) => {
    if (!run?.active) return;
    cancel('页面即将关闭或刷新'); event.preventDefault(); event.returnValue = '';
  });
  render();
})();
