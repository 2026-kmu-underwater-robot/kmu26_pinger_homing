const state = {
  websocket: null,
  reconnectTimer: null,
  lastStatus: null,
  ekf: {
    values: [],
    stateNames: [],
    size: 15,
  },
  bag: {
    topicsLoaded: false,
    defaultTopics: [],
    topics: [],
  },
};

const $ = (id) => document.getElementById(id);

function fmt(value, digits = 2) {
  if (typeof value !== "number" || !Number.isFinite(value)) return "--";
  return value.toFixed(digits);
}

function fmtUnit(value, unit, digits = 2) {
  const text = fmt(value, digits);
  return text === "--" ? "--" : `${text} ${unit}`;
}

function inputNumber(id, fallback) {
  const value = Number($(id).value);
  return Number.isFinite(value) ? value : fallback;
}

function setPill(id, active, label) {
  const el = $(id);
  el.textContent = label;
  el.classList.toggle("good", active);
  el.classList.toggle("warn", !active);
}

function pingerPayload(dryRun) {
  return {
    dry_run: dryRun,
    confirm_live: !dryRun,
    use_hydrophone_estimator: $("pinger-use-estimator").checked,
    use_audio_capture: $("pinger-use-capture").checked,
    audio_device: $("pinger-audio-device").value.trim(),
    reference_frequency_hz: inputNumber("pinger-reference-frequency", 21164),
    tank_max_depth_m: inputNumber("pinger-tank-depth", 11),
    rate_hz: inputNumber("pinger-rate", 30),
    forward_max: inputNumber("pinger-forward-max", 0.48),
    yaw_gain: inputNumber("pinger-yaw-gain", 0.85),
    yaw_command_limit: inputNumber("pinger-yaw-limit", 0.42),
    arrival_radius_m: inputNumber("pinger-arrival-radius", 1.5),
    arrival_hold_s: inputNumber("pinger-arrival-hold", 1.0),
    max_runtime_s: inputNumber("pinger-max-runtime", 180),
    success_hold_s: inputNumber("pinger-success-hold", 0.8),
    success_range_m: inputNumber("pinger-success-range", 0),
    amplitude_range_constant: inputNumber("pinger-range-constant", 0),
  };
}

async function postJson(path, payload = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  if (!response.ok) {
    let detail = `${path} failed: ${response.status}`;
    try {
      const body = await response.json();
      if (body.detail) detail = body.detail;
    } catch (_) {
      // Keep the status-only error when the body is not JSON.
    }
    throw new Error(detail);
  }
  return response.json();
}

async function getJson(path) {
  const response = await fetch(path);
  if (!response.ok) {
    throw new Error(`${path} failed: ${response.status}`);
  }
  return response.json();
}

function bindControls() {
  $("start-stack").addEventListener("click", () => {
    postJson("/api/stack/start").catch(showError);
  });
  $("stop-stack").addEventListener("click", () => {
    postJson("/api/stack/stop").catch(showError);
  });
  $("start-pinger-dry").addEventListener("click", () => {
    postJson("/api/pinger/start", pingerPayload(true)).catch(showError);
  });
  $("pinger-preflight").addEventListener("click", () => {
    postJson("/api/pinger/preflight", pingerPayload(false))
      .then(renderPingerPreflight)
      .catch(showError);
  });
  $("pinger-set-mode").addEventListener("click", () => {
    postJson("/api/pinger/mode", { mode: $("pinger-mode").value }).catch(showError);
  });
  $("pinger-arm").addEventListener("click", () => {
    if (!window.confirm("ARM the physical vehicle? Keep the area and propellers clear.")) return;
    postJson("/api/pinger/arm", { armed: true }).catch(showError);
  });
  $("pinger-disarm").addEventListener("click", () => {
    postJson("/api/pinger/arm", { armed: false }).catch(showError);
  });
  $("start-pinger-live").addEventListener("click", () => {
    if (
      !window.confirm(
        "Enable real pinger-homing RC output? The controller will command MAVROS only while the vehicle reports ARMED.",
      )
    ) {
      return;
    }
    postJson("/api/pinger/start", pingerPayload(false)).catch(showError);
  });
  $("stop-pinger").addEventListener("click", () => {
    postJson("/api/pinger/stop").catch(showError);
  });
  $("start-bag").addEventListener("click", () => {
    startBag().catch(showError);
  });
  $("stop-bag").addEventListener("click", () => {
    postJson("/api/bag/stop").catch(showError);
  });
  $("bag-refresh-topics").addEventListener("click", () => {
    loadBagTopics().catch(showError);
  });

  document.querySelectorAll("[data-dvl-command]").forEach((button) => {
    button.addEventListener("click", () => {
      const payload = {
        command: button.dataset.dvlCommand,
        parameter_name: button.dataset.dvlParam || "",
        parameter_value: button.dataset.dvlValue || "",
      };
      postJson("/api/dvl/command", payload).catch(showError);
    });
  });

  document.querySelectorAll("[data-tab]").forEach((button) => {
    button.addEventListener("click", () => {
      showTab(button.dataset.tab);
      window.history.replaceState(null, "", `#${button.dataset.tab}`);
    });
  });

  $("ekf-reload").addEventListener("click", () => {
    loadEkfConfig().catch(showError);
  });
  $("ekf-save").addEventListener("click", () => {
    saveEkfConfig().catch(showError);
  });
}

function showTab(name) {
  document.querySelectorAll(".tab-button").forEach((button) => {
    button.classList.toggle("active", button.dataset.tab === name);
  });
  document.querySelectorAll(".tab-view").forEach((view) => {
    view.classList.toggle("active", view.id === `${name}-tab`);
  });
  if (name === "ekf" && state.ekf.values.length === 0) {
    loadEkfConfig().catch(showError);
  }
  if (name === "bag" && !state.bag.topicsLoaded) {
    loadBagTopics().catch(showError);
  }
}

function connectStatusSocket() {
  clearTimeout(state.reconnectTimer);
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  state.websocket = new WebSocket(`${protocol}//${window.location.host}/ws/status`);

  state.websocket.addEventListener("open", () => {
    $("connection-state").textContent = "Connected";
  });

  state.websocket.addEventListener("message", (event) => {
    renderStatus(JSON.parse(event.data));
  });

  state.websocket.addEventListener("close", () => {
    $("connection-state").textContent = "Disconnected. Reconnecting...";
    state.reconnectTimer = setTimeout(connectStatusSocket, 1000);
  });

  state.websocket.addEventListener("error", () => {
    $("connection-state").textContent = "Connection error";
  });
}

function renderStatus(payload) {
  state.lastStatus = payload;
  const process = payload.process || {};
  const ros = payload.ros || {};
  const topics = ros.topics || {};
  const pose = ros.pose || {};
  const velocity = ros.velocity || {};
  const depth = ros.depth || {};
  const battery = ros.battery || {};
  const joy = ros.joy || {};
  const dvlConfig = ros.dvl_config || {};
  const dvlEvents = ros.dvl_events || [];

  setPill("stack-pill", process.stack_running, process.stack_running ? "STACK ON" : "STACK OFF");
  setPill("bag-pill", process.bag_running, process.bag_running ? "BAG ON" : "BAG OFF");
  setPill(
    "mavros-pill",
    topics.mavros_state?.alive && ros.mavros_state?.connected,
    topics.mavros_state?.alive && ros.mavros_state?.connected ? "MAVROS ON" : "MAVROS OFF",
  );
  const pingerAlive = Boolean(process.pinger_running && topics.pinger_homing?.alive);
  const pingerMode = ros.pinger_homing_status?.dry_run ? "DRY" : "LIVE";
  setPill(
    "pinger-pill",
    pingerAlive,
    process.pinger_running ? `PINGER ${pingerAlive ? pingerMode : "WAIT"}` : "PINGER OFF",
  );
  setPill("joy-pill", topics.joy?.alive, topics.joy?.alive ? "JOY ON" : "JOY OFF");
  setPill("battery-pill", topics.battery?.alive, topics.battery?.alive ? "BAT ON" : "BAT OFF");

  $("pose-value").textContent = `${fmt(pose.x)}, ${fmt(pose.y)}, ${fmt(pose.z)}`;
  $("yaw-value").textContent = `${fmt((pose.yaw || 0) * 180 / Math.PI, 1)} deg`;
  $("velocity-value").textContent = `${fmt(velocity.x)}, ${fmt(velocity.y)}, ${fmt(velocity.z)}`;
  $("depth-value").textContent = `${fmt(depth.z)} m`;
  $("battery-voltage").textContent = fmtUnit(battery.voltage, "V");
  $("battery-current").textContent = fmtUnit(battery.current, "A");
  $("battery-soc").textContent =
    typeof battery.percentage === "number" && Number.isFinite(battery.percentage)
      ? `${fmt(battery.percentage * 100, 0)} %`
      : "--";
  $("battery-temp").textContent = fmtUnit(battery.temperature, "C", 1);
  $("joy-axes").textContent = JSON.stringify(joy.axes || [], null, 2);
  $("joy-buttons").textContent = JSON.stringify(joy.buttons || [], null, 2);
  renderDvl(dvlConfig, dvlEvents);
  renderBag(process);
  renderTopics(topics);
  renderPinger(process, ros);
  renderPath(ros.path || []);
  $("log-output").textContent = (process.logs || []).join("\n");
}

function renderBag(process) {
  $("bag-state").textContent = process.bag_running ? "Recording" : "Stopped";
  $("bag-output").textContent = process.bag_output || "--";
  $("bag-log-output").textContent = (process.logs || [])
    .filter((line) => line.includes("[bag]") || line.includes("ros2 bag record"))
    .join("\n");
}

async function loadBagTopics() {
  const payload = await getJson("/api/bag/topics");
  state.bag.defaultTopics = payload.default_topics || [];
  state.bag.topics = payload.topics || [];
  state.bag.topicsLoaded = true;
  renderBagTopics();
}

function renderBagTopics() {
  const defaults = new Set(state.bag.defaultTopics);
  $("bag-topic-list").innerHTML = state.bag.topics
    .map((topic) => {
      const checked = defaults.has(topic) ? "checked" : "";
      return `
        <label>
          <input type="checkbox" value="${topic}" ${checked} />
          <span>${topic}</span>
        </label>`;
    })
    .join("");
}

async function startBag() {
  if (!state.bag.topicsLoaded) {
    await loadBagTopics();
  }
  const mode = document.querySelector('input[name="bag-mode"]:checked')?.value || "selected";
  const recordAll = mode === "all";
  const topics = Array.from(document.querySelectorAll("#bag-topic-list input:checked")).map(
    (input) => input.value,
  );
  await postJson("/api/bag/start", {
    record_all: recordAll,
    topics,
  });
}

function renderDvl(config, events) {
  const hasConfig = Object.keys(config).length > 0;
  $("dvl-updated").textContent = config.updated_at || "--";
  $("dvl-range").textContent = config.range_mode || "--";
  $("dvl-acoustic").textContent =
    typeof config.acoustic_enabled === "boolean" ? (config.acoustic_enabled ? "ON" : "OFF") : "--";
  $("dvl-dark").textContent =
    typeof config.dark_mode_enabled === "boolean" ? (config.dark_mode_enabled ? "ON" : "OFF") : "--";
  $("dvl-sound").textContent =
    typeof config.speed_of_sound === "number" ? `${config.speed_of_sound} m/s` : "--";
  $("dvl-rotation").textContent =
    typeof config.mounting_rotation_offset === "number" ? `${config.mounting_rotation_offset} deg` : "--";
  $("dvl-error").textContent = config.error_message || "--";
  $("dvl-config-json").textContent = hasConfig ? JSON.stringify(dvlConfigPayload(config), null, 2) : "--";

  const last = events[events.length - 1];
  if (last) {
    const name = last.parameter_name ? `${last.command}.${last.parameter_name}` : last.command;
    $("dvl-last").textContent = `${last.success ? "OK" : "FAIL"} ${name}`;
  } else {
    $("dvl-last").textContent = "--";
  }

  $("dvl-events").textContent = events
    .slice(-12)
    .map((event) => {
      const name = event.parameter_name ? `${event.command}.${event.parameter_name}` : event.command;
      const value = event.parameter_value ? ` ${event.parameter_value}` : "";
      const result = event.success ? "OK" : `FAIL ${event.error_message || ""}`.trim();
      const label = event.type === "config" ? "config received" : event.type;
      return `${event.time} ${label} ${name}${value} ${result}`;
    })
    .join("\n");
}

function formatCommand(command) {
  if (!command || typeof command !== "object") return "--";
  return `F ${fmt(command.forward)} | L ${fmt(command.lateral)} | H ${fmt(command.heave)} | Y ${fmt(command.yaw)}`;
}

function renderPingerPreflight(result) {
  const failed = (result.checks || []).filter((check) => !check.ok);
  $("pinger-preflight-result").textContent = result.ok
    ? "READY"
    : failed.map((check) => check.detail).join(" | ") || "NOT READY";
  $("pinger-preflight-result").classList.toggle("good", Boolean(result.ok));
  $("pinger-preflight-result").classList.toggle("warn", !result.ok);
}

function renderPinger(process, ros) {
  const pinger = ros.pinger_homing_status || {};
  const topics = ros.topics || {};
  const mux = ros.rc_mux_status || {};
  const depthSafety = pinger.depth_safety || {};
  const estimate = Array.isArray(pinger.estimated_source_world)
    ? pinger.estimated_source_world.map((value) => fmt(Number(value))).join(", ")
    : "--";
  const actualRange = pinger.amplitude_distance_m ?? pinger.estimated_distance_m;

  $("pinger-process-state").textContent = process.pinger_running ? "running" : "stopped";
  $("pinger-control-mode").textContent = !process.pinger_running
    ? "STOPPED"
    : pinger.dry_run
      ? "DRY RUN · RC RELEASE"
      : pinger.control_output_active
        ? "LIVE · RC ACTIVE"
        : "LIVE · waiting for ARMED";
  $("pinger-mux-state").textContent = `${mux.owner || "unknown"} | ${
    mux.conflict ? "CONFLICT" : mux.output_enabled ? "output enabled" : "output blocked"
  } | pubs ${mux.publisher_count ?? 0}`;
  $("pinger-controller-state").textContent = pinger.state || "--";
  $("pinger-input-state").textContent = [
    `odom ${topics.odom?.alive ? "OK" : "stale"}`,
    `mavros ${topics.mavros_state?.alive && pinger.connected ? "OK" : "stale"}`,
    `audio ${pinger.audio_fresh ? "OK" : "stale"}`,
    `direction ${topics.hydrophone_direction?.alive ? "OK" : "stale"}`,
  ].join(" | ");
  $("pinger-estimate").textContent = `xyz ${estimate} | range ${fmtUnit(actualRange, "m")} | bearing ${fmtUnit(
    pinger.bearing_error_deg,
    "deg",
    1,
  )}`;
  $("pinger-quality").textContent = `locked ${pinger.source_locked ? "yes" : "no"} | residual ${fmtUnit(
    pinger.rms_residual_m,
    "m",
    3,
  )} | cond ${fmt(pinger.condition_number, 1)} | bias ${fmtUnit(pinger.bias_range_rate_mps, "m/s", 3)}`;
  $("pinger-requested-command").textContent = formatCommand(pinger.requested_command);
  $("pinger-command").textContent = formatCommand(pinger.command);
  $("pinger-depth-safety").textContent = `depth ${fmtUnit(depthSafety.vehicle_depth_m, "m")} / ${fmtUnit(
    depthSafety.max_vehicle_depth_m,
    "m",
  )} | probe ${fmt(depthSafety.probe_heave)} | limit ${depthSafety.limit_active ? "ON" : "off"} | recovery ${
    depthSafety.recovery_active ? "ON" : "off"
  }`;
  $("pinger-direction-source").textContent = pinger.control_direction_source || "--";
  $("pinger-samples").textContent = `${pinger.sample_count ?? 0} samples | probe ${pinger.probe_attempt ?? 0} / ${
    pinger.minimum_probe_legs ?? 0
  }`;
  $("pinger-result").textContent = `arrival ${pinger.arrival_complete ? "complete" : "pending"} | calibrated range ${
    pinger.range_complete ? "complete" : "pending"
  } | ${pinger.completion_reason || "running"} | runtime ${fmtUnit(pinger.active_runtime_s, "s", 1)} / ${fmtUnit(
    pinger.max_runtime_s,
    "s",
    0,
  )} | IQ K ${fmt(pinger.amplitude_range_constant, 3)}`;
  $("pinger-log-output").textContent = (process.logs || [])
    .filter(
      (line) =>
        line.includes("[pinger_homing]") ||
        line.includes("single_hydrophone_homing") ||
        line.includes("pinger_hydrophone") ||
        line.includes("rc_override_mux"),
    )
    .join("\n");
}

async function loadEkfConfig() {
  $("ekf-save-state").textContent = "Loading";
  const payload = await getJson("/api/ekf/process_noise");
  state.ekf.values = payload.values || [];
  state.ekf.stateNames = payload.state_names || [];
  state.ekf.size = payload.size || 15;
  $("ekf-path").textContent = payload.path || "--";
  $("ekf-save-state").textContent = "Loaded";
  renderEkfEditor();
}

async function saveEkfConfig() {
  const values = readEkfMatrix();
  $("ekf-save-state").textContent = "Saving";
  const payload = await postJson("/api/ekf/process_noise", { values });
  state.ekf.values = payload.values || values;
  $("ekf-save-state").textContent = "Saved";
  renderEkfEditor();
}

function renderEkfEditor() {
  const { values, stateNames, size } = state.ekf;
  if (values.length !== size * size) return;

  $("ekf-diagonal").innerHTML = stateNames
    .map((name, index) => {
      const value = values[index * size + index];
      return `
        <label>
          <span>${name}</span>
          <input type="number" step="0.001" data-ekf-row="${index}" data-ekf-col="${index}" value="${value}" />
        </label>`;
    })
    .join("");

  const header = [
    '<div class="ekf-cell ekf-corner"></div>',
    ...stateNames.map((name) => `<div class="ekf-cell ekf-heading">${name}</div>`),
  ].join("");

  const rows = [];
  for (let row = 0; row < size; row += 1) {
    rows.push(`<div class="ekf-cell ekf-heading">${stateNames[row]}</div>`);
    for (let col = 0; col < size; col += 1) {
      const value = values[row * size + col];
      rows.push(
        `<input class="ekf-cell ekf-input" type="number" step="0.001" data-ekf-row="${row}" data-ekf-col="${col}" value="${value}" />`,
      );
    }
  }
  $("ekf-matrix").innerHTML = header + rows.join("");

  document.querySelectorAll("[data-ekf-row]").forEach((input) => {
    input.addEventListener("change", syncEkfInputs);
  });
}

function syncEkfInputs(event) {
  const row = Number(event.target.dataset.ekfRow);
  const col = Number(event.target.dataset.ekfCol);
  const size = state.ekf.size;
  const value = Number(event.target.value);
  state.ekf.values[row * size + col] = Number.isFinite(value) ? value : 0;
  document.querySelectorAll(`[data-ekf-row="${row}"][data-ekf-col="${col}"]`).forEach((input) => {
    if (input !== event.target) input.value = event.target.value;
  });
}

function readEkfMatrix() {
  const { size } = state.ekf;
  const values = [...state.ekf.values];
  document.querySelectorAll("#ekf-matrix .ekf-input").forEach((input) => {
    const row = Number(input.dataset.ekfRow);
    const col = Number(input.dataset.ekfCol);
    const value = Number(input.value);
    values[row * size + col] = Number.isFinite(value) ? value : 0;
  });
  return values;
}

function dvlConfigPayload(config) {
  return {
    range_mode: config.range_mode,
    acoustic_enabled: config.acoustic_enabled,
    dark_mode_enabled: config.dark_mode_enabled,
    speed_of_sound: config.speed_of_sound,
    mounting_rotation_offset: config.mounting_rotation_offset,
    response_to: config.response_to,
    success: config.success,
    error_message: config.error_message,
    format: config.format,
    type: config.type,
    updated_at: config.updated_at,
  };
}

function renderTopics(topics) {
  const rows = Object.values(topics).map((topic) => {
    const age = typeof topic.age === "number" ? `${fmt(topic.age, 2)}s` : "--";
    const hz = `${fmt(topic.hz, 1)} Hz`;
    const cls = topic.alive ? "alive" : "stale";
    return `
      <div class="topic ${cls}">
        <strong>${topic.name}</strong>
        <span>${topic.alive ? "alive" : "stale"} · ${hz} · age ${age}</span>
      </div>`;
  });
  $("topic-list").innerHTML = rows.join("");
}

function renderPath(points) {
  const canvas = $("path-canvas");
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = "#2a3731";
  ctx.lineWidth = 1;
  for (let x = 0; x <= width; x += 40) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y <= height; y += 40) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  if (points.length < 2) return;

  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const spanX = Math.max(maxX - minX, 1);
  const spanY = Math.max(maxY - minY, 1);
  const scale = 0.82 * Math.min(width / spanX, height / spanY);
  const cx = (minX + maxX) / 2;
  const cy = (minY + maxY) / 2;

  ctx.strokeStyle = "#6ec6ff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  points.forEach((point, index) => {
    const x = width / 2 + (point.x - cx) * scale;
    const y = height / 2 - (point.y - cy) * scale;
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  const last = points[points.length - 1];
  const lx = width / 2 + (last.x - cx) * scale;
  const ly = height / 2 - (last.y - cy) * scale;
  ctx.fillStyle = "#52d273";
  ctx.beginPath();
  ctx.arc(lx, ly, 5, 0, Math.PI * 2);
  ctx.fill();
}

function showError(error) {
  const line = `${new Date().toLocaleTimeString()} ${error.message}`;
  $("log-output").textContent = `${$("log-output").textContent}\n${line}`.trim();
  $("pinger-log-output").textContent = `${$("pinger-log-output").textContent}\n${line}`.trim();
}

bindControls();
const initialTab = window.location.hash.slice(1);
if (initialTab && document.querySelector(`[data-tab="${initialTab}"]`)) {
  showTab(initialTab);
} else {
  showTab("pinger");
}
connectStatusSocket();
