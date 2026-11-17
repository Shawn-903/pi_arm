/* 示教页的视图切换和有限步进。 */
(() => {
  const $ = id => document.getElementById(id);
  let state = null,
    gate = {},
    sending = false;
  const spaceNames = {
    joint: ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'],
    position: ['X', 'Y', 'Z'],
    orientation: ['Rx', 'Ry', 'Rz']
  };
  $('teach-step').innerHTML = `<div class="finite-step">
    <div class="step-settings"><label>示教空间<select id="stepSpace"><option value="joint">关节</option><option value="position">基坐标位置</option><option value="orientation">绕基坐标轴旋转</option></select></label>
      <label><span id="stepUnit">步长（°）</span><input id="stepDelta" type="number" min="0.001" step="any" value="1"></label></div>
    <div class="step-caption"><span>轴</span><span>当前反馈</span><span>点击走一步</span></div>
    <div id="jogAxes"></div>
    <details class="panel-fold"><summary>指定轴步进</summary><div class="step-quick"><select id="stepAxis" aria-label="步进轴"></select><button id="stepMinus">− 一步</button><button id="stepPlus">＋ 一步</button></div></details>
    <p id="stepStatus" role="status">每次点击执行一个增量，完成后保持位置。</p>
  </div>`;

  function showTeach(tab) {
    if ($('teachTab-' + tab).getAttribute('aria-selected') !== 'true') {
      window.PiArmInteractive?.cancelActions('已切换示教方式，松手执行已关闭');
    }
    for (const button of document.querySelectorAll('[data-teach-tab]')) {
      const active = button.dataset.teachTab === tab;
      button.setAttribute('aria-selected', String(active));
      button.tabIndex = active ? 0 : -1;
      $('teach-' + button.dataset.teachTab).hidden = !active;
    }
  }
  const teachTabs = [...document.querySelectorAll('[data-teach-tab]')];
  teachTabs.forEach((button, i) => {
    button.onclick = () => showTeach(button.dataset.teachTab);
    button.onkeydown = event => {
      if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
      event.preventDefault();
      const index = event.key === 'Home' ? 0 : event.key === 'End' ? teachTabs.length - 1 :
        (i + (event.key === 'ArrowRight' ? 1 : -1) + teachTabs.length) % teachTabs.length;
      teachTabs[index].click();
      teachTabs[index].focus();
    };
  });
  document.querySelectorAll('[data-tab]').forEach(button => button.addEventListener('click', () =>
    showTeach('target')));
  $('openPoints').onclick = () => {
    window.PiArmConsole.showView('points');
    $('pointName').focus();
  };

  function renderAxes() {
    const space = $('stepSpace').value;
    $('stepUnit').textContent = space === 'position' ? '步长（mm）' : '步长（°）';
    $('stepAxis').replaceChildren();
    $('jogAxes').replaceChildren();
    spaceNames[space].forEach((name, axis) => {
      $('stepAxis').add(new Option(name, axis));
      const row = document.createElement('div');
      row.className = 'jog-row';
      row.innerHTML =
        `<strong>${name}</strong><button data-step-axis="${axis}" data-step-sign="-1" aria-label="${name} 减少一步">−</button><output data-step-feedback="${axis}">—</output><button data-step-axis="${axis}" data-step-sign="1" aria-label="${name} 增加一步">＋</button>`;
      row.querySelectorAll('button').forEach(button => button.onclick = () => step(axis, Number(button
        .dataset.stepSign)));
      $('jogAxes').append(row);
    });
    refresh(state, gate);
  }

  async function step(axis, sign) {
    if (sending || !gate.permitted) return;
    const input = $('stepDelta');
    if (!input.value.trim() || !input.checkValidity() || !Number.isFinite(Number(input.value))) {
      input.setAttribute('aria-invalid', 'true');
      input.focus();
      $('stepStatus').textContent = '请填写大于零的步长。';
      return;
    }
    input.setAttribute('aria-invalid', 'false');
    sending = true;
    refresh(state, gate);
    try {
      const result = await window.PiArmControl.request('step_move', {
        space: $('stepSpace').value,
        axis,
        delta: sign * Number(input.value),
        request_id: window.PiArmControl.requestId()
      });
      $('stepStatus').textContent = '步进已受理 · ' + result.step_id;
      $('stepStatus').className = '';
    } catch (error) {
      $('stepStatus').textContent = error.message;
      $('stepStatus').className = 'status-error';
    } finally {
      sending = false;
      refresh(state, gate);
    }
  }

  function refresh(next, nextGate) {
    state = next;
    gate = nextGate || {};
    document.body.dataset.connected = String(gate.connectionState === 'connected');
    const space = $('stepSpace').value;
    const values = next?.current_joint_angles;
    // 末端反馈沿用控制器已换算好的显示值，姿态步进仍由后端生成目标。
    const poseIds = space === 'position' ? ['endX', 'endY', 'endZ'] : ['endRx', 'endRy', 'endRz'];
    document.querySelectorAll('[data-step-feedback]').forEach(output => {
      const axis = Number(output.dataset.stepFeedback);
      const value = space === 'joint' ? values?.[axis] : Number($(poseIds[axis]).value);
      const valid = gate.fresh && next?.state_available !== false && next?.kinematics_valid &&
        next?.motor_fresh?.length === 6 && next.motor_fresh.every(Boolean) && Number.isFinite(value);
      output.replaceChildren(document.createTextNode(valid ? value.toFixed(2) : '—'));
      const unit = document.createElement('small');
      unit.textContent = space === 'position' ? 'mm' : '°';
      output.append(unit);
    });
    document.querySelectorAll('[data-step-axis],#stepMinus,#stepPlus').forEach(button => button.disabled =
      sending || !gate.permitted);
  }
  $('stepSpace').onchange = renderAxes;
  $('stepMinus').onclick = () => step(Number($('stepAxis').value), -1);
  $('stepPlus').onclick = () => step(Number($('stepAxis').value), 1);
  window.PiArmPendant = {
    showTeach,
    refresh
  };
  showTeach('step');
  renderAxes();
})();
