/* Model rendering and gestures. Transport and motion gating live in control.js. */
(() => {
  const $ = (id) => document.getElementById(id);
  const card = $("robotViewer");
  let latest = null,
    gate = {},
    robot,
    ghost,
    manifest,
    scene,
    camera,
    renderer,
    controls;
  let lastAngles = null,
    selected = -1,
    valid = false,
    failed = false,
    ready = false;
  let liveKey = "",
    previewKey = "",
    styleKey = "",
    onScreen = true,
    renderPending = false;
  let axisHelpers = [],
    grid,
    activeView = "iso",
    lastValidTime = "";
  const meshes = [],
    jointButtons = [];
  let externalPreview = null,
    pathLine = null;
  const RAD = Math.PI / 180;
  const finite = (v) => typeof v === "number" && Number.isFinite(v);
  const six = (v) => Array.isArray(v) && v.length === 6 && v.every(finite);
  const libs = window.PiArm3DLibs;
  const T = libs?.THREE;

  function requestRender() {
    if (renderPending || !renderer || failed || !onScreen || document.hidden)
      return;
    renderPending = true;
    requestAnimationFrame(() => {
      renderPending = false;
      if (failed || !onScreen) return;
      const host = $("viewerCanvas");
      if (!host.clientWidth || !host.clientHeight) return;
      renderer.setSize(host.clientWidth, host.clientHeight, false);
      camera.aspect = host.clientWidth / host.clientHeight;
      camera.updateProjectionMatrix();
      renderer.render(scene, camera);
    });
  }

  function fail(message) {
    failed = true;
    valid = false;
    card.dataset.state = "unavailable";
    $("viewerStatus").textContent = "三维显示不可用";
    $("viewerStatus").className = "pill error";
    $("viewerLoading").hidden = false;
    $("viewerLoading").textContent = message + "；数值反馈及原有控制仍可使用。";
    $("viewerSource").textContent = "模型未显示";
    card
      .querySelectorAll("button, select, input")
      .forEach((el) => (el.disabled = true));
  }

  function orderedAngles(state) {
    // Missing, repeated, or unknown names are not silently treated as J1..J6.
    const names = state?.joint_names;
    if (
      !Array.isArray(names) ||
      names.length !== 6 ||
      new Set(names).size !== 6 ||
      !six(state.current_joint_angles)
    )
      return null;
    const angles = manifest.joint_names.map(
      (name) => state.current_joint_angles[names.indexOf(name)],
    );
    return six(angles) ? angles : null;
  }

  function applyAngles(model, angles) {
    const values = Object.fromEntries(
      manifest.joint_names.map((name, i) => [name, angles[i] * RAD]),
    );
    model.setJointValues(values);
    model.updateMatrixWorld(true);
  }

  function axisForLink(linkName) {
    if (linkName === manifest.root) return -1;
    const joint = Object.values(robot.joints).find((j) =>
      j.children.some((child) => child.name === linkName),
    );
    return joint
      ? manifest.joint_names.indexOf(joint.mimicJoint || joint.name)
      : -1;
  }

  function paint() {
    const error = latest?.kinematics_error;
    const codes = latest?.motor_error_codes || [];
    const key = JSON.stringify([
      valid,
      selected,
      lastAngles,
      gate.fresh,
      error,
      codes,
      latest?.motor_has_error,
    ]);
    if (styleKey === key) return;
    styleKey = key;
    const faults = manifest.joint_names.map((name, i) => {
      const m = latest?.motor_ids?.indexOf(i + 1) ?? -1;
      return Boolean(
        gate.fresh &&
          ((m >= 0 && (codes[m] || latest.motor_has_error?.[m])) ||
            (error?.axis === i + 1 &&
              error.code &&
              !["NONE", "JOINT_COMMAND_LIMIT_WARNING"].includes(error.code))),
      );
    });
    const warnings = manifest.joint_names.map((name, i) => {
      const lim = manifest.limits_rad[name];
      return (
        valid &&
        (lastAngles[i] * RAD < lim.lower - 1e-8 ||
          lastAngles[i] * RAD > lim.upper + 1e-8)
      );
    });
    for (const mesh of meshes) {
      const i = mesh.userData.axis;
      const color =
        i >= 0 && faults[i]
          ? 0xd85c5c
          : i >= 0 && warnings[i]
            ? 0xda9a38
            : i === selected && selected >= 0
              ? 0x397de2
              : !valid
                ? 0xa6b0bd
                : i < 0
                  ? 0x4b5d72
                  : i % 2 === 0
                    ? 0x8197b1
                    : 0xc4d1df;
      mesh.material.color.setHex(color);
      mesh.material.emissive.setHex(
        i === selected && selected >= 0 ? 0x102a53 : 0x000000,
      );
    }
    jointButtons.forEach((button, i) => {
      button.classList.toggle("viewer-fault", faults[i]);
      button.classList.toggle("viewer-warning", warnings[i]);
      button.setAttribute("aria-pressed", String(i === selected));
      button.querySelector("small").textContent = valid
        ? lastAngles[i].toFixed(2) + "°"
        : "--";
    });
    if (selected >= 0) {
      $("viewerSelection").textContent =
        `J${selected + 1} · ${manifest.joint_names[selected]} · ` +
        (valid ? `${lastAngles[selected].toFixed(2)}°` : "等待有效反馈") +
        (faults[selected]
          ? " · 故障"
          : warnings[selected]
            ? " · 超出指令限位"
            : "");
    }
    document
      .querySelectorAll(".motor-card")
      .forEach((el, i) =>
        el.classList.toggle("viewer-selected", i === selected),
      );
    requestRender();
  }

  function selectJoint(i) {
    selected = i;
    $("viewerLocate").disabled = i < 0;
    paint();
  }

  function updatePreview() {
    if (!ghost || failed) return;
    const source = $("viewerPreview").value;
    let target = null,
      message = "预览不下发运动",
      error = false;
    if (source === "interactive") target = externalPreview?.target || null;
    if (source === "zero") target = [0, 0, 0, 0, 0, 0];
    if (source === "packing") {
      // Targets are emitted in the backend's fixed preset order, matching this model.
      target = gate.fresh
        ? latest?.preset_poses?.packing?.target_joint_angles_deg
        : null;
      if (!six(target)) {
        target = null;
        error = true;
        message = "请连接后端，读取当前标定的打包目标";
      }
    }
    if (source === "single" || source === "multi") {
      const prefix = source === "single" ? "target" : "multiJoint";
      target = manifest.joint_names.map((_, i) => {
        const value = $(prefix + (i + 1))?.value?.trim();
        return value ? Number(value) : NaN;
      });
    }
    if (target) {
      const bad = !six(target)
        ? -2
        : target.findIndex((q, i) => {
            const limit = manifest.limits_rad[manifest.joint_names[i]];
            return q * RAD < limit.lower - 1e-8 || q * RAD > limit.upper + 1e-8;
          });
      if (bad !== -1) {
        error = true;
        message =
          bad === -2
            ? "目标须为六个有限角度，预览已隐藏"
            : `J${bad + 1} 目标超出标准限位，预览已隐藏`;
        target = null;
      } else {
        message =
          source === "interactive"
            ? externalPreview?.label || "交互目标预览"
            : "青绿色为关节目标 · 仅预览，不执行";
      }
    }
    const key = JSON.stringify([source, target]);
    if (previewKey !== key) {
      previewKey = key;
      ghost.visible = Boolean(target);
      if (target) applyAngles(ghost, target);
      requestRender();
    }
    $("viewerPreviewStatus").textContent = message;
    $("viewerPreviewStatus").className = error ? "error" : "";
  }

  function refresh(state, options) {
    latest = state;
    gate = options;
    if (!ready || failed) return;
    const angles = orderedAngles(state);
    const freshAxes = state?.joint_fresh;
    // Both stream freshness and per-axis freshness matter; motion permission is separate.
    valid = Boolean(
      options.fresh &&
        state?.state_available !== false &&
        state?.kinematics_valid &&
        angles &&
        Array.isArray(freshAxes) &&
        freshAxes.length === 6 &&
        freshAxes.every((v) => v === true) &&
        state.motor_fresh?.length === 6 &&
        state.motor_fresh.every((v) => v === true),
    );
    if (valid) {
      const key = JSON.stringify(angles);
      if (key !== liveKey) {
        applyAngles(robot, angles);
        liveKey = key;
        requestRender();
      }
      lastAngles = angles;
      lastValidTime = new Date().toLocaleTimeString();
    }
    card.dataset.state = valid ? "live" : lastAngles ? "stale" : "reference";
    const error = state?.kinematics_error;
    const fault = Boolean(
      options.fresh && (!state?.kinematics_valid || state?.any_error),
    );
    $("viewerStatus").textContent = valid
      ? fault
        ? "有效反馈 · 设备故障"
        : "模型反馈"
      : lastAngles
        ? "最后有效姿态"
        : "离线零位参考";
    $("viewerStatus").className =
      "pill " + (fault ? "error" : valid ? "success" : "");
    const mode = state?.hardware_type === "mock" ? "MOCK" : "实机模型";
    $("viewerSource").textContent = valid
      ? `${mode} · 编码器 / 绳索反解`
      : lastAngles
        ? `反馈不可用 · 保留 ${lastValidTime} 的姿态`
        : "零位几何参考 · 非实时位置";
    const reason = !options.fresh
      ? "连接断开或状态过期"
      : !angles
        ? "关节名称或角度不完整"
        : !state?.kinematics_valid
          ? `解算无效 ${error?.code || ""}`
          : "轴反馈不新鲜";
    $("viewerNote").textContent = valid
      ? `模型由编码器经绳索运动学推算，请与实机姿态对照。${error?.code && error.code !== "NONE" ? error.code + " · " + (error.message || "") : $("modelInteraction")?.value === "joint" ? "拖动关节：先选择 J1–J6，再左右拖动模型区域。" : "拖动旋转 · 滚轮缩放 · 右键平移。"}`
      : `${reason}。${lastAngles ? "保留姿态已标灰，不能作为当前位置。" : "当前只显示零位几何参考。"}`;
    const p = new T.Vector3();
    robot.links[manifest.tip].getWorldPosition(p);
    $("viewerTcp").textContent = valid
      ? `模型 tool0 / mm：X ${(p.x * 1000).toFixed(1)} · Y ${(p.y * 1000).toFixed(1)} · Z ${(p.z * 1000).toFixed(1)}`
      : "模型 tool0：--（非实时）";
    paint();
    updatePreview();
  }

  function setView(view) {
    if (!["workbench", "commissioning"].includes(view)) return;
    $(
      view === "commissioning"
        ? "viewerCommissioningSlot"
        : "viewerWorkbenchSlot",
    ).appendChild(card);
    requestRender();
  }

  function snapshot() {
    // Read-only diagnostics for reproducible transform verification.
    if (!ready) return { ready: false, failed };
    robot.updateMatrixWorld(true);
    ghost?.updateMatrixWorld(true);
    return {
      ready: !failed,
      failed,
      valid,
      state: card.dataset.state,
      selected,
      jointNames: manifest.joint_names,
      sourceHash: manifest.source_sha256,
      limits: manifest.joint_names.map((n) => ({
        lower: manifest.limits_rad[n].lower / RAD,
        upper: manifest.limits_rad[n].upper / RAD,
      })),
      pathPointCount: pathLine?.geometry.attributes.position.count || 0,
      angles: lastAngles?.slice(),
      previewVisible: Boolean(ghost?.visible),
      joints: Object.fromEntries(
        Object.entries(robot.joints).map(([n, j]) => [n, j.jointValue.slice()]),
      ),
      links: Object.fromEntries(
        Object.entries(robot.links).map(([n, l]) => [
          n,
          l.matrixWorld.toArray(),
        ]),
      ),
      previewJoints: ghost
        ? Object.fromEntries(
            Object.entries(ghost.joints).map(([n, j]) => [
              n,
              j.jointValue.slice(),
            ]),
          )
        : {},
      meshCount: meshes.length,
      triangles: meshes.reduce(
        (n, mesh) => n + mesh.geometry.attributes.position.count / 3,
        0,
      ),
    };
  }
  window.PiArmViewer = {
    refresh(state, options) {
      try {
        refresh(state, options);
      } catch (error) {
        fail(error.message || "模型状态更新失败");
      }
    },
    setView,
    snapshot,
    setPreview(target, points = [], label = "交互目标预览") {
      externalPreview = target ? { target: target.slice(), label } : null;
      if (pathLine) {
        scene.remove(pathLine);
        pathLine.traverse((obj) => {
          obj.geometry?.dispose();
          obj.material?.dispose();
        });
        pathLine = null;
      }
      if (ready && !failed && points.length > 1) {
        const geometry = new T.BufferGeometry().setFromPoints(
          points.map((p) => new T.Vector3(...p.tcp)),
        );
        pathLine = new T.Line(
          geometry,
          new T.LineBasicMaterial({ color: 0xe59229, depthTest: false }),
        );
        pathLine.renderOrder = 4;
        // A small world-space tube keeps the path legible on high-DPI/mobile
        // WebGL, where LineBasicMaterial linewidth is fixed to one pixel.
        const curve = new T.CurvePath();
        for (let i = 1; i < points.length; ++i) {
          const a = new T.Vector3(...points[i - 1].tcp),
            b = new T.Vector3(...points[i].tcp);
          if (a.distanceTo(b) > 1e-9) curve.add(new T.LineCurve3(a, b));
        }
        if (curve.curves.length) {
          const tube = new T.Mesh(
            new T.TubeGeometry(curve, 200, 0.0018, 6, false),
            new T.MeshBasicMaterial({ color: 0xe59229, depthTest: false }),
          );
          tube.renderOrder = 4;
          pathLine.add(tube);
        }
        for (const [index, color] of [
          [0, 0x187b57],
          [points.length - 1, 0xe59229],
        ]) {
          const marker = new T.Mesh(
            new T.SphereGeometry(0.006, 12, 8),
            new T.MeshBasicMaterial({ color, depthTest: false }),
          );
          marker.position.set(...points[index].tcp);
          marker.renderOrder = 5;
          pathLine.add(marker);
        }
        scene.add(pathLine);
      }
      $("viewerPreview").value = target ? "interactive" : "none";
      updatePreview();
      requestRender();
    },
    scrub(target) {
      if (!externalPreview) return;
      externalPreview.target = target.slice();
      updatePreview();
    },
  };

  async function init() {
    if (location.protocol === "file:") {
      fail(
        "三维模型需通过 HTTP 加载，请运行 scripts/start_web.sh 后访问网页地址",
      );
      return;
    }
    if (!libs || !window.DecompressionStream)
      throw new Error("请使用支持 WebGL2 和 gzip 解压的现代浏览器");
    renderer = new T.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
    renderer.setClearColor(0xedf2f7);
    renderer.outputColorSpace = T.SRGBColorSpace;
    renderer.domElement.tabIndex = 0;
    renderer.domElement.setAttribute(
      "aria-label",
      "机械臂三维模型，可拖动旋转与缩放",
    );
    $("viewerCanvas").appendChild(renderer.domElement);
    renderer.domElement.addEventListener("webglcontextlost", (event) => {
      event.preventDefault();
      fail("图形上下文已丢失，请刷新网页重新加载");
    });
    scene = new T.Scene();
    camera = new T.PerspectiveCamera(38, 1, 0.01, 50);
    camera.up.set(0, 0, 1); // ROS uses Z-up. Keep model transforms in ROS coordinates.
    controls = new libs.OrbitControls(camera, renderer.domElement);
    controls.minDistance = 0.15;
    controls.maxDistance = 8;
    controls.addEventListener("change", requestRender);
    scene.add(new T.AmbientLight(0xffffff, 1.6));
    for (const [position, intensity] of [
      [[2, -3, 4], 2.5],
      [[-2, 2, 1], 1.1],
    ]) {
      const light = new T.DirectionalLight(0xffffff, intensity);
      light.position.set(...position);
      scene.add(light);
    }
    grid = new T.GridHelper(2.4, 24, 0xa7b9ce, 0xd0dce8);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -0.002;
    scene.add(grid);
    const base = new URL("model/", location.href).href;
    const fetchChecked = async (url, options = {}) => {
      const response = await fetch(url, {
        signal: AbortSignal.timeout(60000),
        ...options,
      });
      if (!response.ok)
        throw new Error(`模型资源加载失败 (${response.status})`);
      return response;
    };
    manifest = await (
      await fetchChecked(base + "manifest.json", { cache: "no-store" })
    ).json();
    if (
      manifest.schema !== 1 ||
      manifest.joint_names?.length !== 6 ||
      new Set(manifest.joint_names).size !== 6
    )
      throw new Error("模型关节清单不匹配");
    const xml = await (
      await fetchChecked(base + manifest.urdf, { cache: "no-store" })
    ).text();
    const pending = [];
    let loaded = 0;
    const loader = new libs.URDFLoader();
    loader.parseCollision = false;
    loader.loadMeshCb = (path, manager, done) => {
      pending.push(
        (async () => {
          const response = await fetchChecked(path);
          const bytes = await new Response(
            response.body.pipeThrough(new DecompressionStream("gzip")),
          ).arrayBuffer();
          const geometry = new libs.STLLoader().parse(bytes);
          done(new T.Mesh(geometry, new T.MeshStandardMaterial()));
          loaded++;
          $("viewerLoading").textContent =
            `正在加载连杆 ${loaded} / ${manifest.meshes.length}`;
        })(),
      );
    };
    robot = loader.parse(xml, base);
    await Promise.all(pending);
    if (failed) return;
    if (
      !robot.links[manifest.root] ||
      !robot.links[manifest.tip] ||
      manifest.joint_names.some((name) => !robot.joints[name])
    )
      throw new Error("模型缺少必要关节或坐标系");
    // URDF Loader clamps by default. Live feedback must retain tolerance excursions.
    Object.values(robot.joints).forEach((j) => (j.ignoreLimits = true));
    applyAngles(robot, [0, 0, 0, 0, 0, 0]);
    ghost = robot.clone();
    Object.values(ghost.joints).forEach((j) => (j.ignoreLimits = true));
    ghost.visible = false;
    for (const [linkName, link] of Object.entries(robot.links)) {
      link.traverse((obj) => {
        // Each link is traversed recursively; the nearest URDF link owns the mesh.
        if (!obj.isMesh || meshes.includes(obj)) return;
        let owner = obj.parent;
        while (owner && !owner.isURDFLink) owner = owner.parent;
        const axis = axisForLink(owner?.name || linkName);
        obj.userData.axis = axis;
        obj.material = new T.MeshStandardMaterial({
          roughness: 0.63,
          metalness: 0.12,
        });
        meshes.push(obj);
      });
    }
    ghost.traverse((obj) => {
      if (!obj.isMesh) return;
      obj.material = new T.MeshStandardMaterial({
        color: 0x08aa98,
        transparent: true,
        opacity: 0.27,
        depthWrite: false,
        roughness: 0.7,
      });
      obj.renderOrder = 1;
    });
    scene.add(robot, ghost);
    function addFrame(link, label, length) {
      const axes = new T.AxesHelper(length);
      axes.material.depthTest = false;
      axes.renderOrder = 2;
      link.add(axes);
      axisHelpers.push(axes);
      const canvas = document.createElement("canvas");
      canvas.width = 256;
      canvas.height = 64;
      const ctx = canvas.getContext("2d");
      ctx.font = "500 30px sans-serif";
      ctx.fillStyle = "#344d6b";
      ctx.fillText(label, 8, 42);
      const sprite = new T.Sprite(
        new T.SpriteMaterial({
          map: new T.CanvasTexture(canvas),
          depthTest: false,
        }),
      );
      sprite.scale.set(0.14, 0.035, 1);
      sprite.position.set(length, 0, length / 3);
      sprite.renderOrder = 3;
      link.add(sprite);
      axisHelpers.push(sprite);
    }
    addFrame(robot.links[manifest.root], "base_link", 0.13);
    addFrame(robot.links[manifest.tip], "tool0", 0.095);

    function view(name) {
      activeView = name;
      const bounds = new T.Box3().setFromObject(robot);
      if (ghost.visible) bounds.union(new T.Box3().setFromObject(ghost));
      const center = bounds.getCenter(new T.Vector3());
      const size = bounds.getSize(new T.Vector3()).length();
      const aspect =
        $("viewerCanvas").clientWidth /
        Math.max(1, $("viewerCanvas").clientHeight);
      const fov = Math.min(
        camera.fov * RAD,
        2 * Math.atan(Math.tan((camera.fov * RAD) / 2) * aspect),
      );
      const distance = Math.max(0.7, (size / (2 * Math.sin(fov / 2))) * 1.1);
      const direction = {
        iso: [1.3, -1.8, 1.0],
        front: [0, -1, 0],
        side: [1, 0, 0],
        top: [0, -0.001, 1],
      }[name];
      controls.target.copy(center);
      camera.position
        .copy(center)
        .add(new T.Vector3(...direction).normalize().multiplyScalar(distance));
      controls.update();
      card
        .querySelectorAll("[data-camera-view]")
        .forEach((button) =>
          button.setAttribute(
            "aria-pressed",
            String(button.dataset.cameraView === name),
          ),
        );
      requestRender();
    }
    card
      .querySelectorAll("[data-camera-view]")
      .forEach((button) =>
        button.addEventListener("click", () => view(button.dataset.cameraView)),
      );
    $("viewerFit").addEventListener("click", () => view(activeView));
    $("viewerAxes").addEventListener("change", (e) => {
      axisHelpers.forEach((a) => (a.visible = e.target.checked));
      requestRender();
    });
    $("viewerGrid").addEventListener("change", (e) => {
      grid.visible = e.target.checked;
      requestRender();
    });
    $("viewerPreview").addEventListener("change", updatePreview);
    document.addEventListener("input", (e) => {
      if (/^(target|multiJoint)[1-6]$/.test(e.target.id)) updatePreview();
    });
    manifest.joint_names.forEach((name, i) => {
      const button = document.createElement("button");
      button.type = "button";
      button.setAttribute("aria-label", `选择 J${i + 1} ${name}`);
      button.innerHTML = `<strong>J${i + 1}</strong><small>--</small>`;
      button.addEventListener("click", () => selectJoint(i));
      $("viewerJoints").appendChild(button);
      jointButtons.push(button);
    });
    $("viewerLocate").addEventListener("click", () => {
      if (selected < 0) return;
      window.PiArmConsole?.showView("workbench");
      document.querySelector('[data-tab="single"]').click();
      $("target" + (selected + 1)).focus();
      $("target" + (selected + 1)).scrollIntoView({
        block: "center",
        behavior: "smooth",
      });
    });
    let down = null,
      dragging = false;
    const pick = (e) => {
      const rect = renderer.domElement.getBoundingClientRect();
      const pointer = new T.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1,
      );
      const ray = new T.Raycaster();
      ray.setFromCamera(pointer, camera);
      return ray.intersectObjects(meshes, false)[0]?.object.userData.axis ?? -1;
    };
    const cancelDrag = () => {
      if (dragging)
        window.PiArmInteractive?.endGesture(true, renderer.domElement);
      dragging = false;
      down = null;
      controls.enabled = true;
    };
    renderer.domElement.addEventListener(
      "pointerdown",
      (e) => {
        if (
          e.button !== 0 ||
          (down && $("modelInteraction")?.value === "joint")
        )
          return;
        down = { x: e.clientX, y: e.clientY, id: e.pointerId };
        if ($("modelInteraction")?.value !== "joint") return;
        // Capture before OrbitControls so a joint gesture cannot also rotate the camera.
        controls.enabled = false;
        const hit = pick(e);
        const axis = hit >= 0 ? hit : selected;
        if (
          axis < 0 ||
          !valid ||
          !window.PiArmInteractive?.beginGesture(
            axis,
            lastAngles[axis],
            renderer.domElement,
          )
        ) {
          down = null;
          return;
        }
        selectJoint(axis);
        down.angle = lastAngles[axis];
        dragging = true;
        renderer.domElement.setPointerCapture(e.pointerId);
      },
      true,
    );
    renderer.domElement.addEventListener("pointermove", (e) => {
      if (!dragging || down?.id !== e.pointerId) return;
      window.PiArmInteractive?.updateGesture(
        down.angle + (e.clientX - down.x) * 0.2,
        renderer.domElement,
      );
    });
    renderer.domElement.addEventListener("pointerup", (e) => {
      if (!down || down.id !== e.pointerId) {
        if (!dragging) controls.enabled = true;
        return;
      }
      if (dragging) {
        window.PiArmInteractive?.endGesture(false, renderer.domElement);
        dragging = false;
      } else if (Math.hypot(e.clientX - down.x, e.clientY - down.y) <= 5) {
        const axis = pick(e);
        if (axis >= 0) selectJoint(axis);
      }
      down = null;
      controls.enabled = true;
    });
    renderer.domElement.addEventListener("pointercancel", cancelDrag);
    renderer.domElement.addEventListener("lostpointercapture", cancelDrag);
    window.addEventListener("blur", cancelDrag);
    $("modelInteraction")?.addEventListener("change", cancelDrag);
    new ResizeObserver(requestRender).observe($("viewerCanvas"));
    new IntersectionObserver(([entry]) => {
      onScreen = entry.isIntersecting;
      if (onScreen) requestRender();
    }).observe($("viewerCanvas"));
    document.addEventListener("visibilitychange", requestRender);
    ready = true;
    $("viewerLoading").hidden = true;
    view("iso");
    refresh(latest, gate);
  }
  init().catch((error) => fail(error.message || "模型加载失败"));
})();
