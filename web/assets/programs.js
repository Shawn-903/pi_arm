/* 步骤编辑与服务端执行状态。 */
(() => {
  const $ = id => document.getElementById(id);
  const rpc = (cmd, params = {}) => window.PiArmControl.request(cmd, params);
  const terminal = new Set(['SUCCEEDED', 'CANCELED', 'FAILED', 'INTERRUPTED']);
  let state = null,
    gate = {},
    programs = [],
    points = [],
    runSnapshot = null,
    snapshotRequest = '',
    retrySnapshotAt = 0,
    renderedRun = '';
  let program = {
      schema_version: 1,
      name: '新程序',
      steps: []
    },
    revision = 0;
  const nav = document.createElement('button');
  nav.className = 'nav-item';
  nav.dataset.viewTarget = 'programs';
  nav.innerHTML = '<span class="nav-icon" aria-hidden="true">▤</span>步骤程序<span class="nav-index">PG</span>';
  nav.title = '步骤程序';
  nav.setAttribute('aria-label', '步骤程序');
  document.querySelector('[data-view-target="points"]').after(nav);
  nav.addEventListener('click', () => window.PiArmConsole.showView('programs'));
  const panel = document.createElement('section');
  panel.className = 'view-panel';
  panel.dataset.viewPanel = 'programs';
  panel.hidden = true;
  panel.innerHTML = `<div id="programRunning" class="card" hidden></div><div class="card program-editor"><h2>步骤程序</h2>
    <p>保存运动、等待和有限循环。启动时固定本次目标与参数，网页断线后继续执行。</p>
    <div class="program-toolbar"><label>已保存程序<select id="programSelect"><option value="">请选择</option></select></label>
      <button id="programRefresh">刷新</button><button id="programNew">新建</button><button id="programLoad">载入</button>
      <button id="programDelete">删除</button></div>
    <label>程序名称<input id="programName" maxlength="80" value="新程序"></label>
    <div id="programSteps"></div><button id="programAdd">添加步骤</button>
    <div class="program-toolbar"><button id="programSave">保存</button><button id="programValidate">校验</button>
      <button class="btn-primary" id="programStart">启动程序</button><button id="programExport">导出 JSON</button>
      <label class="file-label">导入 JSON<input id="programImport" type="file" accept=".json,application/json"></label></div>
    <p id="programStatus" role="status"></p></div>`;
  document.querySelector('[data-view-panel="points"]').after(panel);
  const execution = document.createElement('section');
  execution.className = 'execution-bar';
  execution.innerHTML = `<div><strong id="executionName">无程序或脚本运行</strong><span id="executionDetail"></span></div>
    <div><button id="executionPause" disabled>暂停</button><button id="executionResume" disabled>继续</button>
    <button id="executionStop" class="btn-red">停止</button></div>`;
  $('viewContent').before(execution);
  const jog = document.createElement('div');
  jog.className = 'card finite-step';
  jog.innerHTML =
    `<h3>有限步进</h3><p>使用最新静止反馈生成一次目标，完成后停在目标位置。</p><div class="program-toolbar">
    <label>空间<select id="stepSpace"><option value="joint">关节（°）</option><option value="position">基坐标位置（mm）</option><option value="orientation">绕基坐标轴旋转（°）</option></select></label>
    <label>轴<select id="stepAxis"></select></label><label>步长<input id="stepDelta" type="number" min="0.001" step="any" value="1"></label>
    <button id="stepMinus">− 一步</button><button id="stepPlus">+ 一步</button></div><p id="stepStatus" role="status"></p>`;
  document.querySelector('[data-view-panel="workbench"]').prepend(jog);
  const history = document.createElement('div');
  history.className = 'card';
  history.innerHTML = `<h3>树莓派运行记录</h3><button id="historyRefresh">刷新记录</button>
    <div id="historyList" class="program-history"></div><pre id="historyDetail"></pre>`;
  document.querySelector('[data-view-panel="records"]').prepend(history);
  const css = document.createElement('style');
  css.textContent =
    `.execution-bar{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:14px;background:var(--panel,#fff);border:1px solid #bfd2e0;border-radius:12px;margin-bottom:16px;flex-wrap:wrap}.execution-bar span{display:block;margin-top:6px}.program-toolbar{display:flex;gap:10px;flex-wrap:wrap;align-items:end;margin:14px 0}.program-toolbar label{display:flex;flex-direction:column;gap:5px}.program-editor{padding:20px}.program-row{padding:12px;border:1px solid #cbd5e1;border-radius:8px;margin:12px 0;min-width:0}.program-row.current-step{border:2px solid #159482;background:#e8faf5}.program-values{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}.program-values label{display:flex;flex-direction:column;min-width:0}.program-values input{width:100%;min-width:0}.program-repeat{padding-left:12px;border-left:3px solid #cbd5e1}.program-row select,.program-row input{max-width:100%}.program-history button{margin:6px}#historyDetail{white-space:pre-wrap;overflow-wrap:anywhere;max-height:420px;overflow:auto}.finite-step{padding:16px;margin-bottom:16px}.field-error{display:block;color:#b42318;font-size:12px}input[aria-invalid=true]{outline:2px solid #b42318}.file-label input{max-width:210px}@media(max-width:600px){.program-values{grid-template-columns:repeat(2,minmax(0,1fr))}.program-toolbar{align-items:stretch}.execution-bar{align-items:stretch}.program-editor{padding:10px}}`;
  document.head.append(css);
  const labels = {
    move: '关节直接运动',
    movej: 'MoveJ 关节运动',
    movel: 'MoveL 直线运动',
    wait: '等待',
    repeat: '有限循环'
  };
  const statuses = {
    STARTING: '正在启动',
    RUNNING: '运行中',
    PAUSING: '本步完成后暂停',
    PAUSED: '已暂停',
    STOPPING: '正在停止',
    SUCCEEDED: '已完成',
    CANCELED: '已停止',
    FAILED: '失败',
    INTERRUPTED: '已中断'
  };

  function message(text, error = false) {
    $('programStatus').textContent = text;
    $('programStatus').className = error ? 'status-error' : '';
  }

  function field(title, value, change, {
    min = null,
    max = null,
    step = 'any'
  } = {}) {
    const label = document.createElement('label');
    label.textContent = title;
    const input = document.createElement('input');
    input.type = 'number';
    input.step = step;
    input.value = value;
    if (min !== null) input.min = min;
    if (max !== null) input.max = max;
    const validate = () => {
      const valid = input.value.trim() && input.checkValidity();
      input.setAttribute('aria-invalid', valid ? 'false' : 'true');
    };
    validate();
    input.addEventListener('input', () => {
      validate();
      change(input.value.trim() ? Number(input.value) : NaN);
    });
    label.append(input);
    return label;
  }
  const newRow = type => type === 'wait' ? {
    type,
    seconds: 1
  } : type === 'repeat' ? {
    type,
    count: 2,
    steps: [newRow('wait')]
  } : {
    type,
    params: type === 'move' ? {
      targets: [0, 0, 0, 0, 0, 0],
      max_speed: 2,
      output_speed_limit: 2
    } : type === 'movej' ? {
      target: [0, 0, 0, 0, 0, 0],
      vel: 2,
      acc: 5,
      output_speed_limit: 2
    } : {
      target: [0, 0, 300, 0, 0, 0],
      linear_vel: 10,
      linear_acc: 20,
      angular_vel: 2,
      angular_acc: 5,
      output_speed_limit: 2
    }
  };

  function button(text, action) {
    const b = document.createElement('button');
    b.textContent = text;
    b.addEventListener('click', action);
    return b;
  }

  function renderRows(parent, list, nested = false, path = '') {
    list.forEach((row, index) => {
      const box = document.createElement('article');
      box.className = 'program-row';
      box.dataset.path = path + index;
      const head = document.createElement('div');
      head.className = 'program-toolbar';
      const select = document.createElement('select');
      for (const [type, label] of Object.entries(labels))
        if (!nested || type !== 'repeat') select.add(new Option(label, type));
      select.value = row.type;
      select.addEventListener('change', () => {
        list[index] = newRow(select.value);
        render();
      });
      head.append(select, button('上移', () => {
          if (index) {
            [list[index - 1], list[index]] = [list[index], list[index - 1]];
            render();
          }
        }),
        button('下移', () => {
          if (index + 1 < list.length) {
            [list[index + 1], list[index]] = [list[index], list[index + 1]];
            render();
          }
        }),
        button('删除步骤', () => {
          list.splice(index, 1);
          render();
        }));
      box.append(head);
      if (row.type === 'repeat') {
        box.append(field('循环次数', row.count, v => row.count = v, {
          min: 1,
          max: 10000,
          step: '1'
        }));
        const body = document.createElement('div');
        body.className = 'program-repeat';
        renderRows(body, row.steps, true, path + index + '.');
        box.append(body, button('添加循环内步骤', () => {
          row.steps.push(newRow('wait'));
          render();
        }));
      } else if (row.type === 'wait') box.append(field('等待秒数', row.seconds, v => row.seconds = v, {
        min: 0
      }));
      else {
        const target = document.createElement('select');
        target.add(new Option('手动目标', ''));
        points.filter(p => p.kind === (row.type === 'movel' ? 'cartesian' : 'joint')).forEach(p =>
          target.add(new Option(p.name, p.id)));
        if (row.point_id && !points.some(p => p.id === row.point_id)) target.add(new Option('已引用点位 ' +
          row.point_id, row.point_id));
        target.value = row.point_id || '';
        target.addEventListener('change', () => {
          if (target.value) row.point_id = target.value;
          else delete row.point_id;
          render();
        });
        box.append(target);
        row.params ||= newRow(row.type).params;
        if (!row.point_id) {
          const values = document.createElement('div');
          values.className = 'program-values';
          const key = row.type === 'move' ? 'targets' : 'target';
          row.params[key] ||= [0, 0, 0, 0, 0, 0];
          row.params[key].forEach((v, i) => values.append(field(row.type === 'movel' ? ['X mm', 'Y mm',
            'Z mm', 'Rx °', 'Ry °', 'Rz °'
          ][i] : `J${i+1} °`, v, n => row.params[key][i] = n)));
          box.append(values);
        }
        const params = document.createElement('div');
        params.className = 'program-toolbar';
        const names = row.type === 'move' ? {
          max_speed: '关节速度 °/s'
        } : row.type === 'movej' ? {
          vel: '关节速度 °/s',
          acc: '加速度 °/s²'
        } : {
          linear_vel: '直线速度 mm/s',
          linear_acc: '直线加速度 mm/s²',
          angular_vel: '角速度 °/s'
        };
        names.output_speed_limit = '输出轴限速 °/s';
        for (const [key, label] of Object.entries(names)) params.append(field(label, row.params[key] ??
          2, n => row.params[key] = n, {
            min: key === 'output_speed_limit' ? 1 / 6 : 0.001,
            max: ['max_speed', 'vel', 'angular_vel', 'output_speed_limit'].includes(key) ? 10 : null
          }));
        box.append(params);
      }
      parent.append(box);
    });
  }

  function render() {
    $('programName').value = program.name;
    $('programSteps').replaceChildren();
    renderRows($('programSteps'), program.steps);
  }

  function documentValue() {
    const invalid = panel.querySelector('[aria-invalid=true]');
    if (invalid) {
      invalid.focus();
      throw Error('请先修正标红字段');
    }
    program.name = $('programName').value.trim();
    if (!program.name) {
      $('programName').focus();
      throw Error('请填写程序名称');
    }
    return structuredClone(program);
  }
  async function refreshList() {
    const data = await rpc('program_list');
    programs = data.programs;
    $('programSelect').replaceChildren(new Option('请选择', ''));
    programs.forEach(p => $('programSelect').add(new Option(p.name, p.id)));
  }
  async function action(fn) {
    try {
      await fn();
    } catch (e) {
      message(e.message, true);
    }
  }

  function download(content, name, type = 'application/json') {
    const url = URL.createObjectURL(new Blob([content], {
      type
    }));
    const a = document.createElement('a');
    a.href = url;
    a.download = name;
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }
  async function start(document) {
    const result = await rpc('program_start', {
      program: document,
      request_id: crypto.randomUUID()
    });
    message('程序已启动 · ' + result.run_id);
    return result;
  }
  $('programName').addEventListener('input', () => {
    program.name = $('programName').value;
  });
  $('programAdd').onclick = () => {
    program.steps.push(newRow('wait'));
    render();
  };
  $('programNew').onclick = () => {
    program = {
      schema_version: 1,
      name: '新程序',
      steps: []
    };
    revision = 0;
    render();
  };
  $('programRefresh').onclick = () => action(refreshList);
  $('programLoad').onclick = () => action(async () => {
    program = await rpc('program_get', {
      id: $('programSelect').value
    });
    revision = program.revision;
    render();
  });
  $('programSave').onclick = () => action(async () => {
    program = await rpc('program_save', {
      program: documentValue(),
      revision
    });
    revision = program.revision;
    render();
    await refreshList();
    $('programSelect').value = program.id;
    message('程序已保存，版本 ' + revision);
  });
  $('programDelete').onclick = () => action(async () => {
    const p = programs.find(p => p.id === $('programSelect').value);
    if (p && confirm(`删除程序「${p.name}」？`)) {
      await rpc('program_delete', {
        id: p.id,
        revision: p.revision
      });
      await refreshList();
    }
  });
  $('programValidate').onclick = () => action(async () => {
    const result = await rpc('program_validate', {
      program: documentValue()
    });
    message(`校验通过，共 ${result.step_count} 个展开步骤`);
  });
  $('programStart').onclick = () => action(() => start(documentValue()));
  $('programExport').onclick = () => action(async () => download(JSON.stringify(documentValue(), null, 2),
    'pi-arm-program.json'));
  $('programImport').onchange = event => action(async () => {
    const file = event.target.files[0];
    if (!file) return;
    const doc = JSON.parse(await file.text());
    await rpc('program_validate', {
      program: doc
    });
    delete doc.id;
    delete doc.revision;
    program = doc;
    revision = 0;
    render();
    message('已导入编辑区，请保存后使用');
    event.target.value = '';
  });
  for (const [id, cmd] of [
      ['executionPause', 'execution_pause'],
      ['executionResume', 'execution_resume'],
      ['executionStop', 'execution_stop']
    ]) $(id).onclick = () => action(() => rpc(cmd));

  function axes() {
    const joint = $('stepSpace').value === 'joint';
    $('stepAxis').replaceChildren();
    (joint ? ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'] : ['X', 'Y', 'Z']).forEach((v, i) => $('stepAxis').add(
      new Option(v, i)));
  }
  $('stepSpace').onchange = axes;
  axes();
  for (const [id, sign] of [
      ['stepMinus', -1],
      ['stepPlus', 1]
    ]) $(id).onclick = () => action(async () => {
    const input = $('stepDelta');
    if (!input.value.trim() || !input.checkValidity()) {
      input.focus();
      throw Error('步长应大于零');
    }
    const result = await rpc('step_move', {
      space: $('stepSpace').value,
      axis: Number($('stepAxis').value),
      delta: sign * Number(input.value),
      request_id: crypto.randomUUID()
    });
    $('stepStatus').textContent = '步进已受理 · ' + result.step_id;
  });
  $('historyRefresh').onclick = () => action(async () => {
    const result = await rpc('run_list');
    $('historyList').replaceChildren();
    for (const run of result.runs) {
      const row = document.createElement('div');
      const title = document.createElement('span');
      title.textContent = `${run.started_at} · ${run.name} · ${statuses[run.status]||run.status} `;
      row.append(title, button('详情', () => action(async () => {
          const data = await rpc('run_get', {
            run_id: run.run_id
          });
          $('historyDetail').textContent =
            `${data.run.name}\n运行编号：${data.run.run_id}\n结果：${statuses[data.run.status]||data.run.status}\n停止确认：${data.run.stop_confirmed?'已确认':'无确认记录'}\n${data.run.message||''}\n\n` +
            data.steps.map(s =>
              `步骤 ${s.index} · ${labels[s.type]||s.type} · ${statuses[s.status]||s.status}\n任务 ${s.task_id||'—'} · 耗时 ${Number(s.elapsed_s||0).toFixed(3)} s\n${s.params?'目标 / 参数：'+JSON.stringify(s.params):'等待 '+s.seconds+' s'}\n${s.message||''}`
              ).join('\n\n');
        })),
        ...['json', 'csv'].map(format => button('导出 ' + format.toUpperCase(), () => action(
      async () => {
            const data = await rpc('run_export', {
              run_id: run.run_id,
              format
            });
            download(data.content, `pi-arm-${run.run_id}.${format}`, format === 'csv' ?
              'text/csv' : 'application/json');
          }))));
      $('historyList').append(row);
    }
  });

  function renderExecution(run, active) {
    if (run?.mode === 'program' && run.run_id !== runSnapshot?.run_id && !snapshotRequest && Date.now() >
      retrySnapshotAt) {
      snapshotRequest = run.run_id;
      rpc('run_get', {
          run_id: run.run_id
        }).then(data => {
          runSnapshot = data.run;
          renderExecution(state?.execution, Boolean(state?.execution?.run_id && !terminal.has(state
            .execution.status)));
        })
        .catch(() => {
          retrySnapshotAt = Date.now() + 3000;
        }).finally(() => {
          snapshotRequest = '';
        });
    }
    const frozen = runSnapshot?.run_id === run?.run_id ? runSnapshot : null;
    const current = (run?.step_index || 1) - 1;
    $('programRunning').hidden = !frozen;
    const key = frozen ? `${frozen.run_id}:${current}:${active}` : '';
    if (frozen && key !== renderedRun) {
      const title = document.createElement('h3');
      title.textContent = '本次运行：' + frozen.name;
      const note = document.createElement('p');
      note.textContent = '下面显示本次固定的步骤；编辑区的修改用于下次启动。';
      $('programRunning').replaceChildren(title, note);
      frozen.steps.slice(Math.max(0, current - 2), current + 3).forEach((step, i) => {
        const index = Math.max(0, current - 2) + i;
        const line = document.createElement('div');
        line.className = 'program-row';
        line.classList.toggle('current-step', active && index === current);
        line.textContent =
          `步骤 ${index+1} · ${labels[step.type]||step.type}${step.cycle?' · 第 '+step.cycle+' 轮':''}${step.type==='wait'?' · '+step.seconds+' 秒':''}`;
        $('programRunning').append(line);
      });
    }
    renderedRun = key;
    const sameDocument = frozen && JSON.stringify(frozen.snapshot.steps) === JSON.stringify(program.steps);
    const flattened = [];

    function flatten(list, path = '') {
      list.forEach((r, i) => {
        if (r.type === 'repeat') {
          for (let c = 0; c < Math.min(r.count, 10000); c++) flatten(r.steps, path + i + '.');
        } else flattened.push(path + i);
      });
    }
    if (sameDocument) flatten(program.steps);
    const path = flattened[current];
    $('programSteps').querySelectorAll('.program-row').forEach(el => el.classList.toggle('current-step',
      active && el.dataset.path === path));
  }
  window.PiArmPrograms = {
    rpc,
    start,
    download,
    setLibrary(library) {
      points = library.points;
      render();
    },
    get occupied() {
      return Boolean(state?.execution?.run_id && !terminal.has(state.execution.status));
    },
    refresh(next, nextGate) {
      state = next;
      gate = nextGate;
      const run = next?.execution,
        active = Boolean(run?.run_id && !terminal.has(run.status));
      $('executionName').textContent = run?.run_id ?
        `${run.name} · ${{sdk:'Python 脚本',program:'步骤程序',manual:'单次运动'}[run.mode]||'执行任务'}` : '无程序或脚本运行';
      $('executionDetail').textContent = run?.run_id ?
        `${statuses[run.status]||run.status} · 步骤 ${run.step_index||0}/${run.step_count||'—'} · ${run.message||''}${run.status==='STOPPING'?' · 等待停止确认':''}` :
        '';
      $('executionPause').disabled = !active || !['RUNNING', 'PAUSING'].includes(run.status);
      $('executionResume').disabled = !active || run.status !== 'PAUSED';
      $('executionStop').disabled = gate.connectionState !== 'connected' || next?.read_only;
      $('programStart').disabled = !gate.permitted;
      $('stepMinus').disabled = $('stepPlus').disabled = !gate.permitted;
      renderExecution(run, active);
    }
  };
  render();
})();
