/* A/B 测试由树莓派执行，网页展示持久记录。 */
(() => {
  const $ = id => document.getElementById(id),
    api = window.PiArmPrograms;
  const inputIds = ['repeatStartPoint', 'repeatTargetPoint', 'repeatCycles', 'repeatDwell', 'repeatSpeed',
    'repeatOutputSpeed', 'repeatLinearSpeed'
  ];
  let points = [],
    state = null,
    gate = {},
    runId = '',
    report = null,
    polling = false,
    nextPoll = 0;
  const active = () => Boolean(state?.execution?.run_id === runId && ['STARTING', 'RUNNING', 'PAUSED',
    'PAUSING', 'STOPPING'
  ].includes(state.execution.status));
  const samples = () => report?.steps?.filter(s => s.status === 'SUCCEEDED' && s.ab_sample).map(s => s
    .ab_sample) || [];

  function selection() {
    const a = points.find(p => p.id === $('repeatStartPoint').value),
      b = points.find(p => p.id === $('repeatTargetPoint').value);
    return {
      a,
      b,
      error: !a || !b ? '请选择两个点位' : a.id === b.id || a.values.every((v, i) => v === b.values[i]) ?
        '两个目标应不同' : a.kind !== b.kind ? '两个点位类型应一致' : ''
    };
  }

  function render() {
    const selected = selection(),
      busy = active(),
      data = samples();
    inputIds.forEach(id => $(id).disabled = busy);
    $('repeatLinearLabel').hidden = selected.a?.kind !== 'cartesian';
    $('repeatSelection').textContent = selected.error ||
    `${selected.a.name} → ${selected.b.name} · 树莓派独立执行`;
    $('repeatStart').disabled = $('repeatTrial').disabled = busy || api.occupied || !gate.permitted ||
      Boolean(selected.error);
    $('repeatStop').disabled = !busy;
    $('repeatExport').disabled = !report;
    $('repeatCsv').disabled = !data.length;
    const config = report?.run.snapshot?.ab_config;
    $('repeatProgress').max = config?.cycles || 10;
    $('repeatProgress').value = data.length;
    $('repeatSummary').textContent = `目标点记录：${data.length}${config?' / '+config.cycles:''} 次`;
    if (runId && state?.execution?.run_id === runId) $('repeatStatus').textContent =
      `${state.execution.status} · 步骤 ${state.execution.step_index}/${state.execution.step_count} · ${state.execution.message||''}`;
    const intervals = data.map(s => s.target_interval_s).filter(Number.isFinite);
    $('repeatTimingSummary').textContent = intervals.length ?
      `${intervals.length} 个完整周期 · 平均 ${(intervals.reduce((a,b)=>a+b,0)/intervals.length).toFixed(3)} s · 最短 ${Math.min(...intervals).toFixed(3)} s · 最长 ${Math.max(...intervals).toFixed(3)} s · 极差 ${(Math.max(...intervals)-Math.min(...intervals)).toFixed(3)} s` :
      '周期统计：至少完成两轮后得到第一个目标点记录间隔。';
    $('repeatTimingRows').replaceChildren();
    for (const s of data.slice(-10)) {
      const row = document.createElement('tr');
      for (const value of [s.cycle, s.cycle_duration_s, s.target_interval_s, s.target_arrived_elapsed_s, s
          .captured_elapsed_s
        ]) {
        const cell = document.createElement('td');
        cell.textContent = value === null ? '—' : typeof value === 'number' ? value.toFixed(3) : value;
        row.append(cell);
      }
      $('repeatTimingRows').append(row);
    }
  }

  function number(id) {
    const input = $(id);
    if (!input.value.trim() || !input.checkValidity()) {
      input.focus();
      throw Error('请修正 ' + input.closest('label').firstChild.textContent);
    }
    return Number(input.value);
  }
  async function start(trial) {
    try {
      const {
        a,
        b,
        error
      } = selection();
      if (error) throw Error(error);
      const cycles = trial ? 5 : number('repeatCycles'),
        dwell = number('repeatDwell'),
        speed = number('repeatSpeed'),
        output_speed = number('repeatOutputSpeed'),
        linear_speed = number('repeatLinearSpeed');
      const kind = a.kind === 'joint' ? 'move' : 'movel';
      const params = kind === 'move' ? {
        max_speed: speed,
        output_speed_limit: output_speed
      } : {
        linear_vel: linear_speed,
        linear_acc: 10,
        angular_vel: speed,
        angular_acc: 10,
        output_speed_limit: output_speed
      };
      const program = {
        schema_version: 1,
        kind: 'ab',
        name: trial ? 'A/B 五轮试跑' : 'A/B 重复定位',
        ab_config: {
          start: a,
          target: b,
          cycles,
          dwell,
          speed,
          output_speed,
          linear_speed
        },
        steps: [{
          type: 'repeat',
          count: cycles,
          steps: [{
            type: kind,
            point_id: a.id,
            params,
            ab_leg: 'start'
          }, {
            type: 'wait',
            seconds: dwell
          }, {
            type: kind,
            point_id: b.id,
            params,
            ab_leg: 'target'
          }, {
            type: 'wait',
            seconds: dwell,
            capture: true
          }]
        }]
      };
      const result = await api.start(program);
      runId = result.run_id;
      report = null;
      nextPoll = 0;
      render();
    } catch (error) {
      $('repeatStatus').textContent = error.message;
    }
  }
  async function poll() {
    if (polling || !runId || gate.connectionState !== 'connected' || Date.now() < nextPoll) return;
    polling = true;
    nextPoll = Date.now() + 1000;
    try {
      report = await api.rpc('run_get', {
        run_id: runId
      });
      render();
    } catch (error) {
      $('repeatStatus').textContent = error.message;
    } finally {
      polling = false;
    }
  }

  function statistics(data) {
    const intervals = data.map(s => s.target_interval_s).filter(Number.isFinite);
    return intervals.length ? {
      interval_count: intervals.length,
      mean_s: intervals.reduce((a, b) => a + b, 0) / intervals.length,
      min_s: Math.min(...intervals),
      max_s: Math.max(...intervals),
      spread_s: Math.max(...intervals) - Math.min(...intervals)
    } : null;
  }

  function download(csv) {
    if (!report) return;
    const data = samples();
    const result = {
      schema_version: 2,
      run_id: runId,
      measurement_source: 'encoder_cable_model_not_external_metrology',
      base_frame: 'base_link',
      tool_frame: 'tool0',
      rpy_convention: 'Rz(yaw) Ry(pitch) Rx(roll)',
      config: report.run.snapshot.ab_config,
      result: report.run.status,
      started_at: report.run.started_at,
      finished_at: report.run.finished_at,
      samples: data,
      hardware_type: report.run.snapshot.ab_config.start.hardware_type,
      calibration_id: report.run.snapshot.ab_config.start.calibration_id,
      timing: {
        source: 'server_monotonic',
        unit: 'seconds',
        interval_definition: 'previous_target_record_to_current_target_record',
        first_round_excluded: true,
        externally_synchronized: false,
        statistics: statistics(data)
      }
    };
    let content = JSON.stringify(result, null, 2);
    if (csv) {
      const headers = ['cycle', 'captured_at', 'task_id', 'calibration_id', 'cycle_duration_s',
        'target_interval_s', 'arrival_interval_s', 'target_arrived_at', 'target_arrived_elapsed_s',
        'captured_elapsed_s',
        ...Array.from({
          length: 6
        }, (_, i) => `J${i+1}_deg`), 'X_mm', 'Y_mm', 'Z_mm', 'Rx_deg', 'Ry_deg', 'Rz_deg', ...Array.from({
          length: 6
        }, (_, i) => `sampled_error_J${i+1}_deg`), ...Array.from({
          length: 6
        }, (_, i) => `verification_error_J${i+1}_deg`)
      ];
      const rows = data.map(s => [s.cycle, s.captured_at, s.task_id, result.calibration_id, s
        .cycle_duration_s, s.target_interval_s, s.arrival_interval_s, s.target_arrived_at, s
        .target_arrived_elapsed_s, s.captured_elapsed_s, ...s.joint_angles_deg, ...s.pose_mm_deg, ...s
        .sampled_error_joint_angles_deg, ...s.verification_error_joint_angles_deg
      ]);
      content = '\ufeff' + [headers, ...rows].map(row => row.map(v => '"' + String(v ?? '').replaceAll('"',
        '""') + '"').join(',')).join('\r\n');
    }
    api.download(content, `pi-arm-repeat-${runId}.${csv?'csv':'json'}`, csv ? 'text/csv' :
      'application/json');
  }
  window.PiArmRepeat = {
    get active() {
      return active();
    },
    cancel() {},
    setLibrary(library) {
      points = structuredClone(library.points);
      for (const id of ['repeatStartPoint', 'repeatTargetPoint']) {
        const old = $(id).value;
        $(id).replaceChildren(new Option('请选择点位', ''));
        points.forEach(p => $(id).add(new Option(p.name, p.id)));
        if (points.some(p => p.id === old)) $(id).value = old;
      }
      render();
    },
    refresh(next, nextGate) {
      state = next;
      gate = nextGate;
      if (!runId && next?.execution?.name?.startsWith('A/B ')) runId = next.execution.run_id;
      void poll();
      render();
    }
  };
  inputIds.forEach(id => $(id).addEventListener('change', render));
  $('repeatStart').onclick = () => start(false);
  $('repeatTrial').onclick = () => start(true);
  $('repeatStop').onclick = () => api.rpc('execution_stop', {
    run_id: runId
  }).catch(e => $('repeatStatus').textContent = e.message);
  $('repeatExport').onclick = () => download(false);
  $('repeatCsv').onclick = () => download(true);
  render();
})();
