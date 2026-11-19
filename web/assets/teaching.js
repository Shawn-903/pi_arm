/* 点位记录、保存和加载；选中的点位填入运动表单，交由操作者预览并执行。 */
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
  const search = document.createElement('input');
  search.id = 'pointSearch'; search.placeholder = '搜索名称或备注'; search.setAttribute('aria-label', '搜索点位');
  const filter = document.createElement('select'); filter.id = 'pointFilter'; filter.setAttribute('aria-label', '筛选点位类型');
  filter.add(new Option('全部类型', '')); filter.add(new Option('关节点位', 'joint')); filter.add(new Option('末端位姿', 'cartesian'));
  const searchBar = document.createElement('div'); searchBar.className = 'point-searchbar'; searchBar.append(search, filter);
  $('pointList').before(searchBar); search.addEventListener('input', render); filter.addEventListener('change', render);
  const importer = document.createElement('input'); importer.type = 'file'; importer.accept = '.json,application/json'; importer.id = 'pointImport';
  const importLabel = document.createElement('label'); importLabel.className = 'file-label'; importLabel.textContent = '导入 JSON'; importer.setAttribute('aria-label', '导入点位 JSON'); importLabel.append(importer);
  const conflict = document.createElement('select'); conflict.id = 'pointConflict'; conflict.setAttribute('aria-label','导入冲突处理');
  conflict.add(new Option('冲突跳过','skip')); conflict.add(new Option('相同 ID 更新','update'));
  $('exportPoints').after(importLabel, conflict);
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
    const units = point.kind === "joint" ? "deg" : "mm,deg";
    if (point.units !== units) return `该点位需要使用 ${units} 单位`;
    if (point.hardware_type !== state?.hardware_type)
      return "实机与模拟点位不能混用";
    if (point.model_id !== state?.base_frames?.model_id)
      return "模型版本不同，请重新核验并录入";
    if (point.joint_names?.join() !== jointNames.join())
      return "关节顺序不匹配";
    return "";
  }
  function render() {
    window.PiArmRepeat?.setLibrary(library);
    window.PiArmPrograms?.setLibrary(library);
    $("pointCount").textContent =
      `${library.points.length} 个点位 · 版本 ${library.revision}`;
    $("pointList").replaceChildren();
    const visible = library.points.filter(point => (!filter.value || point.kind === filter.value) &&
      (!search.value || (point.name+' '+(point.note||'')).toLowerCase().includes(search.value.toLowerCase())));
    if (!visible.length) {
      const empty = document.createElement("p");
      empty.className = "point-empty";
      empty.textContent = library.points.length ? '没有匹配的点位，请调整筛选条件。' : '尚无点位。录入目标，或在机械臂静止后记录反馈。';
      $("pointList").append(empty);
    }
    for (const point of visible) {
      const row = document.createElement("article");
      row.className = "point-card";
      row.dataset.pointId = point.id;
      const title = document.createElement("strong");
      title.textContent = point.name;
      const meta = document.createElement("small");
      const unitLabel = { deg: "°", "mm,deg": "mm / °" }[point.units] || point.units || "单位未知";
      meta.textContent = `${point.kind === "joint" ? "六关节" : "笛卡尔"} · ${unitLabel} · ${{ manual: "手动录入", feedback: "反馈示教", model: "模型目标" }[point.source] || point.source} · ${new Date(point.created_at).toLocaleString()}`;
      const values = document.createElement("p");
      values.className = "point-vector";
      values.textContent = point.values.map(fmt).join(" / ");
      const note = document.createElement("p");
      note.textContent = point.note || "无备注";
      const buttons = document.createElement("div");
      buttons.className = "point-actions";
      const load = document.createElement("button");
      load.textContent = "载入运动目标";
      load.dataset.pointAction = "load";
      load.disabled = !available || busy || Boolean(compatibility(point));
      load.title = compatibility(point) || "仅填入目标，不执行运动";
      load.addEventListener("click", () => loadPoint(point));
      const remove = document.createElement("button");
      remove.textContent = "删除";
      remove.dataset.pointAction = "delete";
      remove.className = "btn-secondary";
      remove.disabled = !available || busy;
      remove.addEventListener("click", () => { if (confirm(`删除点位「${point.name}」？已保存程序中的引用可能失效。`))
        request("point_delete", { id: point.id }, `已删除 ${point.name}`); });
      const edit=document.createElement('button'); edit.textContent='修改'; edit.dataset.pointAction='edit'; edit.disabled=!available||busy;
      edit.onclick=async()=>{
        let original;
        try {
          const archive=JSON.parse((await window.PiArmControl.library('point_export')).content);
          original=archive.points.find(item=>item.id===point.id);
          if(!original)throw Error('点位已删除，请刷新列表');
          library=archive;
        } catch(error) {status(error.message,true);return;}
        const dialog=document.createElement('dialog');dialog.className='point-edit-dialog';
        const title=document.createElement('h3');title.textContent='修改点位';dialog.append(title);
        const fields={};for(const [key,label,value]of [['name','名称',original.name],['note','备注',original.note||''],['values','六个数值（逗号分隔）',original.values.join(', ')]]){
          const row=document.createElement('label');row.textContent=label;const input=document.createElement('input');input.value=value;fields[key]=input;row.append(input);dialog.append(row);
        }
        const errorText=document.createElement('p');errorText.className='field-error';errorText.setAttribute('role','alert');fields.name.after(errorText);
        const save=document.createElement('button');save.textContent='保存修改';save.onclick=async()=>{
          const params={id:point.id,name:fields.name.value,note:fields.note.value};
          if(fields.values.value!==original.values.join(', ')){
            const values=fields.values.value.split(',').map(v=>v.trim());if(values.length!==6||values.some(v=>!v||!Number.isFinite(Number(v)))){fields.values.setCustomValidity('请填写六个有限数字');fields.values.reportValidity();return;}params.values=values.map(Number);
          }
          save.disabled=true;
          const saved=await request('point_update',params,'点位已更新');
          save.disabled=false;
          if(saved){dialog.close();dialog.remove();}
          else{errorText.textContent=$('pointStatus').textContent;fields.name.focus();}
        };
        const cancel=document.createElement('button');cancel.textContent='取消';cancel.onclick=()=>{dialog.close();dialog.remove();};dialog.append(save,cancel);document.body.append(dialog);dialog.showModal();
      };
      const detail=document.createElement('details'),summary=document.createElement('summary'),body=document.createElement('pre');summary.textContent='详情与来源';
      body.textContent=`编号：${point.id}\n版本：${point.revision||1}\n运行模式：${point.hardware_type}\n坐标：${point.base_frame} → ${point.tool_frame}\n单位：${point.units}\n模型：${point.model_id}\n参考：${point.calibration_id||'未记录'}\n创建：${point.created_at||'未记录'}\n修改：${point.updated_at||'未修改'}${point.imported_at?'\n导入：'+point.imported_at:''}`;
      detail.append(summary,body);
      buttons.append(load, edit, remove);
      row.append(title, meta, values, note, buttons);
      row.append(detail);
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
      return true;
    } catch (error) {
      if (token === generation) status(error.message, true);
      return false;
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
  $("exportPoints").addEventListener("click", async () => {
    let content;
    try { content = (await window.PiArmControl.library('point_export')).content; }
    catch(error) {status(error.message,true);return;}
    const url = URL.createObjectURL(
      new Blob([content], {
        type: "application/json",
      }),
    );
    const link = document.createElement("a");
    link.href = url;
    link.download = `pi-arm-points-${state?.hardware_type || "unknown"}.json`;
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  importer.addEventListener('change',async()=>{
    try{
      const file=importer.files[0];if(!file)return;if(file.size>2*1024*1024)throw Error('点位文件不能超过 2 MiB');
      const document=JSON.parse(await file.text());
      const params={document,conflict:conflict.value,revision:library.revision,commit:false};
      const check=await window.PiArmControl.library('point_import',params);
      status(`预检通过：新增 ${check.added}，更新 ${check.updated}，跳过 ${check.skipped}`);
      if(confirm(`导入预检通过：新增 ${check.added}，更新 ${check.updated}，跳过 ${check.skipped}。提交整批点位？`)){
        library=await window.PiArmControl.library('point_import',{...params,revision:check.revision,commit:true});render();controls();status('点位已整批导入');
      }
    }catch(error){status(error.message,true);}finally{importer.value='';}
  });
  buildInputs();
  render();
  controls();
})();
