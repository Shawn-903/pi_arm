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
  nav.innerHTML = '<span class="nav-icon" aria-hidden="true">▤</span>在线编程';
  nav.title = '在线编程';
  nav.setAttribute('aria-label', '在线编程');
  document.querySelector('[data-view-target="workbench"]').after(nav);
  nav.addEventListener('click', () => window.PiArmConsole.showView('programs'));
  const panel = document.createElement('section');
  panel.className = 'view-panel';
  panel.dataset.viewPanel = 'programs';
  panel.hidden = true;
  panel.innerHTML = `<div id="programRunning" class="card" hidden></div>
    <div class="card program-editor">
      <div class="program-filebar">
        <select id="programSelect" aria-label="已保存程序"><option value="">已保存程序</option></select>
        <button id="programRefresh">刷新</button><button id="programLoad">打开</button>
        <button id="programNew">新建</button><button id="programSave">保存</button><button id="programDelete">删除</button>
        <label class="file-label">导入 JSON<input id="programImport" type="file" accept=".json,application/json" aria-label="导入程序"></label>
        <button id="programExport">导出 JSON</button>
      </div>
      <div class="program-titlebar"><label>程序<input id="programName" maxlength="80" value="新程序"></label><span id="programStepCount">0 个步骤</span></div>
      <div class="program-studio">
        <aside class="instruction-palette" aria-label="添加指令"><h3>指令</h3>
          <button data-program-add="move">＋ 关节直接运动</button><button data-program-add="movej">＋ MoveJ</button>
          <button data-program-add="movel">＋ MoveL</button><button id="programAdd">＋ 等待</button>
          <button data-program-add="repeat">＋ 有限循环</button><p>点击添加指令，再选择步骤编辑参数。</p>
        </aside>
        <div id="programSteps"><div id="programOutline" class="program-outline" aria-label="程序步骤"></div>
          <section id="programInspector" class="program-inspector" aria-label="步骤参数"></section></div>
      </div>
      <div class="program-runbar"><button class="btn-primary" id="programStart">▶ 启动程序</button><button id="programValidate">校验程序</button>
        <p>启动时固定本次步骤与目标；页面断开后继续执行。</p></div>
      <p id="programStatus" role="status">从左侧添加指令，或打开已保存的程序。</p>
    </div>`;
  document.querySelector('[data-view-panel="points"]').after(panel);
  const execution = document.createElement('section');
  execution.className = 'execution-bar';
  execution.innerHTML = `<div><strong id="executionName">无程序或脚本运行</strong><span id="executionDetail"></span></div>
    <div><button id="executionPause" disabled>暂停</button><button id="executionResume" disabled>继续</button>
    <button id="executionStop" class="btn-red">停止</button></div>`;
  $('viewContent').before(execution);
  const history = document.createElement('div');
  history.className = 'card history-card';
  history.innerHTML =
    `<div class="history-heading"><h3>运行历史</h3><button id="historyRefresh">刷新记录</button></div>
    <div class="history-workspace"><div id="historyList" class="program-history"><p>刷新后显示树莓派保存的运行记录。</p></div><pre id="historyDetail">选择一条记录查看步骤、结果和停止确认。</pre></div>`;
  document.querySelector('[data-view-panel="records"]').prepend(history);
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
      renderOutline();
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

  let selectedRow = null;

  function locate(row, list = program.steps, nested = false, path = '') {
    for (let index = 0; index < list.length; index++) {
      if (list[index] === row) return {
        list,
        index,
        nested,
        path: path + index
      };
      if (list[index].type === 'repeat') {
        const found = locate(row, list[index].steps, true, path + index + '.');
        if (found) return found;
      }
    }
    return null;
  }

  function summary(row) {
    if (row.type === 'wait') return `${row.seconds} 秒`;
    if (row.type === 'repeat') return `${row.count} 次 · ${row.steps.length} 个子步骤`;
    if (row.point_id) return points.find(p => p.id === row.point_id)?.name || '已引用点位';
    return (row.params?.targets || row.params?.target || []).map(n => Number.isFinite(n) ? Number(n.toFixed(
      2)) : '—').join(' / ');
  }

  function renderOutline() {
    const outline = $('programOutline');
    outline.replaceChildren();

    function rows(list, path = '', nested = false) {
      list.forEach((row, index) => {
        const item = button('', () => {
          selectedRow = row;
          render();
        });
        item.className = 'program-row';
        item.dataset.path = path + index;
        item.dataset.nested = String(nested);
        item.setAttribute('aria-pressed', String(row === selectedRow));
        const number = document.createElement('span');
        number.className = 'step-number';
        number.textContent = nested ? '↳' : String(index + 1).padStart(2, '0');
        const body = document.createElement('span');
        const title = document.createElement('strong');
        title.textContent = labels[row.type];
        const detail = document.createElement('small');
        detail.textContent = summary(row);
        body.append(title, detail);
        item.append(number, body);
        outline.append(item);
        if (row.type === 'repeat') rows(row.steps, path + index + '.', true);
      });
    }
    rows(program.steps);
    $('programStepCount').textContent =
      `${program.steps.length} 个步骤${program.id ? ' · 版本 ' + revision : ' · 尚未保存'}`;
    if (!program.steps.length) {
      const empty = document.createElement('div');
      empty.className = 'program-empty';
      empty.innerHTML = '<strong>编排一段运动</strong>从左侧选择运动、等待或有限循环';
      outline.append(empty);
    }
  }

  function renderInspector() {
    const box = $('programInspector');
    box.replaceChildren();
    const position = locate(selectedRow);
    if (!position) {
      box.innerHTML = '<h3>步骤参数</h3><p>选择一个步骤查看和修改参数。</p>';
      return;
    }
    const {
      list,
      index,
      nested
    } = position, row = selectedRow;
    const title = document.createElement('h3');
    title.textContent = `步骤 ${position.path.split('.').map(n => Number(n) + 1).join('.')} · 参数`;
    const select = document.createElement('select');
    select.setAttribute('aria-label', '指令类型');
    for (const [type, label] of Object.entries(labels))
      if (!nested || type !== 'repeat') select.add(new Option(label, type));
    select.value = row.type;
    select.onchange = () => {
      selectedRow = list[index] = newRow(select.value);
      render();
    };
    const tools = document.createElement('div');
    tools.className = 'program-toolbar';
    const up = button('↑ 上移', () => {
      [list[index - 1], list[index]] = [list[index], list[index - 1]];
      render();
    });
    up.disabled = index === 0;
    const down = button('↓ 下移', () => {
      [list[index + 1], list[index]] = [list[index], list[index + 1]];
      render();
    });
    down.disabled = index === list.length - 1;
    tools.append(up, down, button('删除步骤', () => {
      list.splice(index, 1);
      selectedRow = list[Math.min(index, list.length - 1)] || program.steps[0];
      render();
    }));
    box.append(title, select, tools);
    if (row.type === 'repeat') {
      box.append(field('循环次数', row.count, n => row.count = n, {
        min: 1,
        max: 10000,
        step: '1'
      }));
      const add = button('＋ 添加循环内步骤', () => {
        selectedRow = newRow('wait');
        row.steps.push(selectedRow);
        render();
      });
      add.className = 'program-toolbar';
      box.append(add);
    } else if (row.type === 'wait') {
      box.append(field('等待时间（s）', row.seconds, n => row.seconds = n, {
        min: 0
      }));
      const note = document.createElement('p');
      note.textContent = '等待可暂停计时；继续后完成剩余时间。';
      box.append(note);
    } else {
      const targetLabel = document.createElement('label');
      targetLabel.textContent = '目标来源';
      const target = document.createElement('select');
      target.setAttribute('aria-label', '步骤目标来源');
      target.add(new Option('手动输入目标', ''));
      points.filter(p => p.kind === (row.type === 'movel' ? 'cartesian' : 'joint')).forEach(p => target.add(
        new Option(p.name, p.id)));
      if (row.point_id && !points.some(p => p.id === row.point_id)) target.add(new Option('点位不可用 · ' + row
        .point_id, row.point_id));
      target.value = row.point_id || '';
      target.onchange = () => {
        if (target.value) row.point_id = target.value;
        else delete row.point_id;
        render();
      };
      targetLabel.append(target);
      box.append(targetLabel);
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
      } else {
        const values = document.createElement('p');
        values.textContent = points.find(p => p.id === row.point_id)?.values.map(n => Number(n.toFixed(3)))
          .join(' / ') || '请检查点位库';
        box.append(values);
      }
      const params = document.createElement('div');
      params.className = 'program-toolbar program-params';
      const names = row.type === 'move' ? {
        max_speed: '关节速度 °/s'
      } : row.type === 'movej' ? {
        vel: '关节速度 °/s',
        acc: '加速度 °/s²'
      } : {
        linear_vel: '直线速度 mm/s',
        linear_acc: '直线加速度 mm/s²',
        angular_vel: '角速度 °/s',
        angular_acc: '角加速度 °/s²'
      };
      names.output_speed_limit = '输出轴限速 °/s';
      for (const [key, label] of Object.entries(names)) params.append(field(label, row.params[key] ?? 2,
        n => row.params[key] = n, {
          min: key === 'output_speed_limit' ? 1 / 6 : 0.001,
          max: ['max_speed', 'vel', 'angular_vel', 'output_speed_limit'].includes(key) ? 10 : null
        }));
      box.append(params);
    }
  }

  function render() {
    $('programName').value = program.name;
    if (!locate(selectedRow)) selectedRow = program.steps[0];
    renderOutline();
    renderInspector();
  }

  function documentValue() {
    const invalidRow = (list) => {
      for (const row of list) {
        if (row.type === 'repeat') {
          if (!Number.isInteger(row.count) || row.count < 1 || row.count > 10000) return row;
          const child = invalidRow(row.steps);
          if (child) return child;
        } else if (row.type === 'wait') {
          if (!Number.isFinite(row.seconds) || row.seconds < 0) return row;
        } else {
          const params = row.params || {};
          const target = params.targets || params.target;
          if (!row.point_id && (!Array.isArray(target) || target.length !== 6 || target.some(v => !Number
              .isFinite(v)))) return row;
          for (const [key, value] of Object.entries(params)) {
            if (Array.isArray(value)) continue;
            if (!Number.isFinite(value) || value <= 0) return row;
            if (['max_speed', 'vel', 'angular_vel', 'output_speed_limit'].includes(key) && value > 10)
              return row;
            if (key === 'output_speed_limit' && value < 1 / 6) return row;
          }
        }
      }
      return null;
    };
    const bad = invalidRow(program.steps);
    if (bad) {
      selectedRow = bad;
      render();
    }
    const invalid = panel.querySelector('[aria-invalid=true]');
    if (invalid) {
      invalid.focus();
      throw Error('请先修正标红字段');
    }
    if (bad) throw Error('请检查当前步骤的目标与参数');
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
      request_id: window.PiArmControl.requestId()
    });
    message('程序已启动 · ' + result.run_id);
    return result;
  }
  $('programName').addEventListener('input', () => {
    program.name = $('programName').value;
  });

  function addInstruction(type) {
    const place = locate(selectedRow);
    selectedRow = newRow(type);
    if (place && !(place.nested && type === 'repeat')) place.list.splice(place.index + 1, 0, selectedRow);
    else program.steps.push(selectedRow);
    render();
  }
  panel.querySelectorAll('[data-program-add]').forEach(b => {
    b.onclick = () => addInstruction(b.dataset.programAdd);
  });
  $('programAdd').onclick = () => {
    addInstruction('wait');
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
    $('programRunning').hidden = !frozen || !active;
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
      if (JSON.stringify(points) === JSON.stringify(library.points)) return;
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
      window.PiArmPendant?.refresh(next, gate);
      renderExecution(run, active);
    }
  };
  render();
})();
