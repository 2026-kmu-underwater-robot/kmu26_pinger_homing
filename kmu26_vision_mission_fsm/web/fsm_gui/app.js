const els = {};
const ids = [
  "serverState", "rcAllowed", "cameraEnabled", "reloadCamera", "cameraFeed", "cameraEmpty",
  "cameraMeta", "missionRunning", "saveConfig", "startDry", "startLive", "stopMission",
  "startPinger", "stopPinger", "startMarkers", "startRviz", "stopRviz", "stopMarkers",
  "course", "ownCourse", "boundaryX", "boundaryMargin", "boundaryStandoff", "rateHz",
  "pingerDepthZ", "tankMaxDepth",
  "transport", "poseTopic", "statusPath", "fsmState", "robotState", "targetState",
  "countState", "commandState", "robotPose", "rcStatus", "rcRelease", "rcCenter",
  "rcSendAxes", "axisForward", "axisLateral", "axisHeave", "axisYaw", "rcReadout",
  "topicRows", "buoyRows", "processLog", "missionMap", "mapMeta", "tankXMin", "tankXMax",
  "tankYMin", "tankYMax", "robotStartX", "robotStartY", "robotStartYaw", "scoreZoneX",
  "scoreZoneY", "scoreZoneRadius", "mapHud", "mapFsmChip", "mapTargetChip",
  "mapDetectionChip", "mapRcChip", "mapShowGrid", "mapShowLabels", "mapShowTrail",
  "mapRobotReadout", "mapDetectionReadout", "mapSurfaceReadout", "mapScoreReadout"
];

for (const id of ids) {
  els[id] = document.getElementById(id);
}

let firstConfigLoad = true;
let lastCameraUrl = "";
let latestStatus = null;
const robotTrail = [];
const ROBOT_TRAIL_MAX = 900;

function fmt(value, digits = 2) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return "n/a";
  return Number(value).toFixed(digits);
}

function setPill(el, text, cls = "") {
  el.textContent = text;
  el.className = `pill ${cls}`.trim();
}

function setConfigFields(config) {
  if (!firstConfigLoad || !config) return;
  els.course.value = config.course ?? "all";
  els.ownCourse.value = config.own_course ?? "a";
  els.boundaryX.value = config.course_boundary_x ?? 0;
  els.boundaryMargin.value = config.course_boundary_margin ?? 0.8;
  els.boundaryStandoff.value = config.course_boundary_standoff ?? 0.7;
  els.rateHz.value = config.rate_hz ?? 30;
  els.pingerDepthZ.value = config.pinger_depth_z ?? -8.5;
  els.tankMaxDepth.value = config.tank_max_depth_m ?? 11.0;
  els.transport.value = config.transport ?? "rc_override";
  els.poseTopic.value = config.pose_topic ?? "/odometry/filtered";
  els.tankXMin.value = config.tank_x_min ?? -12;
  els.tankXMax.value = config.tank_x_max ?? 12;
  els.tankYMin.value = config.tank_y_min ?? -8;
  els.tankYMax.value = config.tank_y_max ?? 8;
  els.robotStartX.value = config.robot_start_x ?? 0;
  els.robotStartY.value = config.robot_start_y ?? 0;
  els.robotStartYaw.value = config.robot_start_yaw_deg ?? 0;
  els.scoreZoneX.value = config.score_zone_x ?? -6.8;
  els.scoreZoneY.value = config.score_zone_y ?? 0;
  els.scoreZoneRadius.value = config.score_zone_radius ?? 1.5;
  firstConfigLoad = false;
}

function collectConfig() {
  return {
    course: els.course.value,
    own_course: els.ownCourse.value,
    course_boundary_x: Number(els.boundaryX.value),
    course_boundary_margin: Number(els.boundaryMargin.value),
    course_boundary_standoff: Number(els.boundaryStandoff.value),
    rate_hz: Number(els.rateHz.value),
    pinger_depth_z: Number(els.pingerDepthZ.value),
    tank_max_depth_m: Number(els.tankMaxDepth.value),
    transport: els.transport.value,
    pose_topic: els.poseTopic.value,
    tank_x_min: Number(els.tankXMin.value),
    tank_x_max: Number(els.tankXMax.value),
    tank_y_min: Number(els.tankYMin.value),
    tank_y_max: Number(els.tankYMax.value),
    robot_start_x: Number(els.robotStartX.value),
    robot_start_y: Number(els.robotStartY.value),
    robot_start_yaw_deg: Number(els.robotStartYaw.value),
    score_zone_x: Number(els.scoreZoneX.value),
    score_zone_y: Number(els.scoreZoneY.value),
    score_zone_radius: Number(els.scoreZoneRadius.value),
    camera_enabled: els.cameraEnabled.checked
  };
}

async function post(path, payload) {
  const res = await fetch(path, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(payload || {})
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok || data.ok === false) {
    throw new Error(data.error || `${path} failed`);
  }
  return data;
}

async function saveConfig() {
  await post("/api/config", collectConfig());
  await refreshStatus();
}

async function startProcess(kind, extra = {}) {
  await saveConfig();
  await post("/api/process/start", {kind, ...extra});
  await refreshStatus();
}

async function stopProcess(kind) {
  await post("/api/process/stop", {kind});
  await refreshStatus();
}

async function sendRc(mode) {
  const axes = {
    forward: Number(els.axisForward.value),
    lateral: Number(els.axisLateral.value),
    heave: Number(els.axisHeave.value),
    yaw: Number(els.axisYaw.value)
  };
  await post("/api/rc", {mode, axes});
  await refreshStatus();
}

function renderTopics(topics) {
  const rows = Object.entries(topics || {}).map(([key, item]) => {
    const alive = item.alive ? "<span class=\"pill ok\">yes</span>" : "<span class=\"pill bad\">no</span>";
    return `<tr>
      <td class="topic-name" title="${item.name || key}">${item.name || key}</td>
      <td>${alive}</td>
      <td>${fmt(item.hz, 1)}</td>
      <td>${item.age === null || item.age === undefined ? "n/a" : fmt(item.age, 1) + "s"}</td>
    </tr>`;
  });
  els.topicRows.innerHTML = rows.join("");
}

function renderBuoys(status) {
  const buoys = Array.isArray(status?.buoys) ? status.buoys : [];
  if (!buoys.length) {
    els.buoyRows.innerHTML = `<tr><td colspan="5">No buoy status</td></tr>`;
    return;
  }
  els.buoyRows.innerHTML = buoys.map((b) => {
    const xyz = Array.isArray(b.xyz) ? b.xyz.map((v) => fmt(v, 1)).join(", ") : "n/a";
    const flags = [
      b.processed ? "processed" : "",
      b.failed ? "failed" : "",
      b.capture ? "capture" : ""
    ].filter(Boolean).join(" ");
    return `<tr>
      <td title="${b.id || ""}">${b.id || "n/a"}</td>
      <td>${b.class_name || b.class || b.target_class || "n/a"}</td>
      <td>${b.state || "n/a"}</td>
      <td>${xyz}</td>
      <td>${flags || "-"}</td>
    </tr>`;
  }).join("");
}

function renderProcesses(processes) {
  const mission = processes?.mission?.running;
  setPill(els.missionRunning, mission ? "mission running" : "mission idle", mission ? "ok" : "");
  const lines = [];
  for (const [key, proc] of Object.entries(processes || {})) {
    lines.push(`${key}: ${proc.running ? "running" : "stopped"} pid=${proc.pid ?? "-"}`);
    if (proc.command?.length) lines.push(`  ${proc.command.join(" ")}`);
  }
  els.processLog.textContent = lines.join("\n");
}

function numberOr(value, fallback) {
  const num = Number(value);
  return Number.isFinite(num) ? num : fallback;
}

function finite(value) {
  return Number.isFinite(Number(value));
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function shortId(id) {
  return String(id || "n/a").replace(/^course_buoy_/, "").replace(/_float$/, "");
}

function getRobotPose(data) {
  const status = data?.mission_status || {};
  const telemetry = data?.telemetry || {};
  return status.robot || telemetry.pose || {};
}

function getRobotYaw(robot) {
  const yaw = Number(robot?.yaw_rad ?? robot?.yaw);
  return Number.isFinite(yaw) ? yaw : 0;
}

function bodyToWorld(robot, local) {
  if (!local || local.length < 2 || !finite(robot?.x) || !finite(robot?.y)) return null;
  const yaw = getRobotYaw(robot);
  const lx = Number(local[0]);
  const ly = Number(local[1]);
  return {
    x: Number(robot.x) + Math.cos(yaw) * lx - Math.sin(yaw) * ly,
    y: Number(robot.y) + Math.sin(yaw) * lx + Math.cos(yaw) * ly,
    z: finite(local[2]) ? Number(robot.z || 0) + Number(local[2]) : null
  };
}

function detectionWorld(data) {
  const detection = data?.mission_status?.detection;
  const robot = getRobotPose(data);
  if (!detection?.p_intake) return null;
  return bodyToWorld(robot, detection.p_intake);
}

function normalizeLimits(config, data) {
  let xMin = numberOr(config?.tank_x_min, -12);
  let xMax = numberOr(config?.tank_x_max, 12);
  let yMin = numberOr(config?.tank_y_min, -8);
  let yMax = numberOr(config?.tank_y_max, 8);
  const points = [];
  const status = data?.mission_status || {};
  const robot = getRobotPose(data);
  if (finite(robot.x) && finite(robot.y)) points.push([Number(robot.x), Number(robot.y)]);
  for (const b of Array.isArray(status.buoys) ? status.buoys : []) {
    if (Array.isArray(b.xyz) && b.xyz.length >= 2 && finite(b.xyz[0]) && finite(b.xyz[1])) {
      points.push([Number(b.xyz[0]), Number(b.xyz[1])]);
    }
  }
  const det = detectionWorld(data);
  if (det) points.push([det.x, det.y]);
  const collector = status.surface_collection?.collector_xyz;
  if (Array.isArray(collector) && collector.length >= 2 && finite(collector[0]) && finite(collector[1])) {
    points.push([Number(collector[0]), Number(collector[1])]);
  }
  const score = status.score_zone?.xyz;
  if (Array.isArray(score) && score.length >= 2 && finite(score[0]) && finite(score[1])) {
    points.push([Number(score[0]), Number(score[1])]);
  } else {
    points.push([numberOr(config?.score_zone_x, -6.8), numberOr(config?.score_zone_y, 0)]);
  }
  for (const [x, y] of points) {
    xMin = Math.min(xMin, x);
    xMax = Math.max(xMax, x);
    yMin = Math.min(yMin, y);
    yMax = Math.max(yMax, y);
  }
  const pad = Math.max(1.2, Math.min(5, Math.max(xMax - xMin, yMax - yMin) * 0.08));
  xMin -= pad;
  xMax += pad;
  yMin -= pad;
  yMax += pad;
  if (xMax <= xMin) xMax = xMin + 1;
  if (yMax <= yMin) yMax = yMin + 1;
  return {xMin, xMax, yMin, yMax};
}

function drawArrow(ctx, sx, sy, ex, ey, color, width = 2) {
  const dx = ex - sx;
  const dy = ey - sy;
  const len = Math.hypot(dx, dy);
  if (len < 0.001) return;
  const ux = dx / len;
  const uy = dy / len;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(sx, sy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex - ux * 10 - uy * 5, ey - uy * 10 + ux * 5);
  ctx.lineTo(ex - ux * 10 + uy * 5, ey - uy * 10 - ux * 5);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function drawText(ctx, text, x, y, color = "#25364a", align = "left") {
  ctx.save();
  ctx.font = "12px system-ui, sans-serif";
  ctx.textAlign = align;
  ctx.lineWidth = 3;
  ctx.strokeStyle = "rgba(255,255,255,0.85)";
  ctx.strokeText(text, x, y);
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
  ctx.restore();
}

function classColor(name) {
  const cls = String(name || "").toLowerCase();
  if (cls.includes("red")) return "#dc2626";
  if (cls.includes("yellow")) return "#eab308";
  if (cls.includes("orange")) return "#f97316";
  if (cls.includes("white")) return "#f8fafc";
  return "#8b95a1";
}

function buoyStyle(buoy, targetId) {
  const cls = buoy.class_name || buoy.class || buoy.target_class || "";
  const state = String(buoy.state || "").toUpperCase();
  const isTarget = targetId && buoy.id === targetId;
  const processed = Boolean(buoy.processed) || state === "PROCESSED" || state === "SCORED";
  const failed = Boolean(buoy.failed) || state === "FAILED";
  const netted = Boolean(buoy.netted) || state === "NETTED" || state === "SCORED";
  let fill = classColor(cls);
  let stroke = "#374151";
  let radius = 5;
  if (state === "FLOATING") {
    stroke = "#1d4ed8";
    radius = 6;
  }
  if (processed) {
    stroke = "#15803d";
    radius = 7;
  }
  if (netted) {
    stroke = "#7c3aed";
    radius = 8;
  }
  if (failed) {
    fill = "#9ca3af";
    stroke = "#4b5563";
  }
  if (isTarget) {
    stroke = "#991b1b";
    radius = 10;
  }
  return {fill, stroke, radius, line: isTarget ? 3 : 1.7, state, isTarget, failed, processed, netted};
}

function drawGrid(ctx, left, top, right, bottom, limits, toScreen, scale) {
  const span = Math.max(limits.xMax - limits.xMin, limits.yMax - limits.yMin);
  const step = span > 35 ? 5 : span > 14 ? 2 : 1;
  ctx.save();
  ctx.strokeStyle = "rgba(88, 113, 132, 0.20)";
  ctx.lineWidth = 1;
  ctx.font = "11px system-ui, sans-serif";
  ctx.fillStyle = "#60717f";
  for (let x = Math.ceil(limits.xMin / step) * step; x <= limits.xMax; x += step) {
    const p = toScreen(x, 0);
    ctx.beginPath();
    ctx.moveTo(p.x, top);
    ctx.lineTo(p.x, bottom);
    ctx.stroke();
    ctx.fillText(String(Number(x.toFixed(1))), p.x + 3, bottom - 4);
  }
  for (let y = Math.ceil(limits.yMin / step) * step; y <= limits.yMax; y += step) {
    const p = toScreen(0, y);
    ctx.beginPath();
    ctx.moveTo(left, p.y);
    ctx.lineTo(right, p.y);
    ctx.stroke();
    ctx.fillText(String(Number(y.toFixed(1))), left + 4, p.y - 3);
  }
  const origin = toScreen(0, 0);
  ctx.strokeStyle = "rgba(15, 23, 42, 0.30)";
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.moveTo(origin.x, top);
  ctx.lineTo(origin.x, bottom);
  ctx.moveTo(left, origin.y);
  ctx.lineTo(right, origin.y);
  ctx.stroke();
  ctx.restore();
}

function updateRobotTrail(robot) {
  if (!finite(robot?.x) || !finite(robot?.y)) return;
  const last = robotTrail[robotTrail.length - 1];
  const x = Number(robot.x);
  const y = Number(robot.y);
  if (!last || Math.hypot(last.x - x, last.y - y) > 0.05) {
    robotTrail.push({x, y, t: Date.now()});
    while (robotTrail.length > ROBOT_TRAIL_MAX) robotTrail.shift();
  }
}

function drawTrail(ctx, toScreen) {
  if (robotTrail.length < 2) return;
  ctx.save();
  ctx.strokeStyle = "rgba(27, 92, 158, 0.38)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  for (let i = 0; i < robotTrail.length; i += 1) {
    const p = toScreen(robotTrail[i].x, robotTrail[i].y);
    if (i === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  }
  ctx.stroke();
  ctx.restore();
}

function drawRobot(ctx, robot, toScreen, scale) {
  if (!finite(robot?.x) || !finite(robot?.y)) return null;
  const p = toScreen(Number(robot.x), Number(robot.y));
  const yaw = getRobotYaw(robot);
  const bodyL = Math.max(20, 1.25 * scale);
  const bodyW = Math.max(14, 0.75 * scale);
  const nose = [
    [bodyL * 0.62, 0],
    [-bodyL * 0.42, bodyW * 0.55],
    [-bodyL * 0.32, 0],
    [-bodyL * 0.42, -bodyW * 0.55],
  ];
  const rot = (pt) => ({
    x: p.x + Math.cos(yaw) * pt[0] + Math.sin(yaw) * pt[1],
    y: p.y - Math.sin(yaw) * pt[0] + Math.cos(yaw) * pt[1]
  });
  ctx.save();
  ctx.fillStyle = "#1b5c9e";
  ctx.strokeStyle = "#062b50";
  ctx.lineWidth = 2;
  ctx.beginPath();
  const first = rot(nose[0]);
  ctx.moveTo(first.x, first.y);
  for (const pt of nose.slice(1)) {
    const q = rot(pt);
    ctx.lineTo(q.x, q.y);
  }
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  drawArrow(ctx, p.x, p.y, p.x + Math.cos(yaw) * 42, p.y - Math.sin(yaw) * 42, "#062b50", 3);
  ctx.restore();
  return p;
}

function drawMetricRings(ctx, robot, toScreen, scale) {
  if (!finite(robot?.x) || !finite(robot?.y)) return;
  const p = toScreen(Number(robot.x), Number(robot.y));
  ctx.save();
  ctx.strokeStyle = "rgba(27, 92, 158, 0.18)";
  ctx.lineWidth = 1;
  for (const r of [1, 3, 5]) {
    ctx.beginPath();
    ctx.arc(p.x, p.y, r * scale, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.restore();
}

function drawMissionMap(data) {
  const canvas = els.missionMap;
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 20 || rect.height < 20) return;
  const dpr = window.devicePixelRatio || 1;
  const width = Math.round(rect.width * dpr);
  const height = Math.round(rect.height * dpr);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  const config = data.config || {};
  const status = data.mission_status || {};
  const limits = normalizeLimits(config, data);
  const pad = 42;
  const viewW = rect.width - pad * 2;
  const viewH = rect.height - pad * 2;
  const sx = viewW / (limits.xMax - limits.xMin);
  const sy = viewH / (limits.yMax - limits.yMin);
  const scale = Math.min(sx, sy);
  const drawW = (limits.xMax - limits.xMin) * scale;
  const drawH = (limits.yMax - limits.yMin) * scale;
  const ox = (rect.width - drawW) / 2;
  const oy = (rect.height - drawH) / 2;
  const toScreen = (x, y) => ({
    x: ox + (x - limits.xMin) * scale,
    y: oy + drawH - (y - limits.yMin) * scale
  });

  ctx.clearRect(0, 0, rect.width, rect.height);
  ctx.fillStyle = "#e9f1f7";
  ctx.fillRect(0, 0, rect.width, rect.height);

  const bx = numberOr(config.course_boundary_x, 0);
  const margin = Math.max(0, numberOr(config.course_boundary_margin, 0.8));
  const standoff = Math.max(0, numberOr(config.course_boundary_standoff, 0.7));
  const ownSide = String(config.own_course || "a").toLowerCase();
  const left = toScreen(limits.xMin, limits.yMax);
  const right = toScreen(limits.xMax, limits.yMin);
  const boundary = toScreen(bx, 0).x;
  const ownLeft = ownSide !== "b";

  ctx.fillStyle = ownLeft ? "rgba(36, 104, 162, 0.14)" : "rgba(184, 50, 50, 0.10)";
  ctx.fillRect(left.x, left.y, boundary - left.x, right.y - left.y);
  ctx.fillStyle = ownLeft ? "rgba(184, 50, 50, 0.10)" : "rgba(36, 104, 162, 0.14)";
  ctx.fillRect(boundary, left.y, right.x - boundary, right.y - left.y);

  if (els.mapShowGrid?.checked) {
    drawGrid(ctx, left.x, left.y, right.x, right.y, limits, toScreen, scale);
  }

  ctx.strokeStyle = "#486577";
  ctx.lineWidth = 1.5;
  ctx.strokeRect(left.x, left.y, right.x - left.x, right.y - left.y);

  ctx.strokeStyle = "#1aa6b8";
  ctx.lineWidth = 2;
  ctx.setLineDash([8, 5]);
  ctx.beginPath();
  ctx.moveTo(boundary, left.y);
  ctx.lineTo(boundary, right.y);
  ctx.stroke();
  ctx.setLineDash([]);

  for (const [x, color] of [[bx - margin, "#7cc3ce"], [bx + margin, "#7cc3ce"], [bx - standoff, "#b7791f"], [bx + standoff, "#b7791f"]]) {
    const px = toScreen(x, 0).x;
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 5]);
    ctx.beginPath();
    ctx.moveTo(px, left.y);
    ctx.lineTo(px, right.y);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  const labels = Boolean(els.mapShowLabels?.checked);
  if (labels) {
    drawText(ctx, `tank x ${fmt(limits.xMin, 1)}..${fmt(limits.xMax, 1)} | y ${fmt(limits.yMin, 1)}..${fmt(limits.yMax, 1)} m`, left.x, Math.max(14, left.y - 14));
    drawText(ctx, ownLeft ? "A / own" : "B / own", left.x + 8, left.y + 18, "#1f5a89");
    drawText(ctx, ownLeft ? "B / opponent" : "A / opponent", boundary + 8, left.y + 18, "#8b2b2b");
  }

  const scoreWorld = Array.isArray(status.score_zone?.xyz) && status.score_zone.xyz.length >= 2
    ? {x: Number(status.score_zone.xyz[0]), y: Number(status.score_zone.xyz[1])}
    : {x: numberOr(config.score_zone_x, -6.8), y: numberOr(config.score_zone_y, 0)};
  const score = toScreen(scoreWorld.x, scoreWorld.y);
  const scoreRadiusM = Math.max(0, numberOr(status.score_zone?.radius_m, numberOr(config.score_zone_radius, 1.5)));
  const scoreRadius = scoreRadiusM * scale;
  ctx.fillStyle = "rgba(36, 163, 106, 0.22)";
  ctx.strokeStyle = "#16824f";
  ctx.beginPath();
  ctx.arc(score.x, score.y, scoreRadius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  if (labels) drawText(ctx, "score zone", score.x + 8, score.y - 8, "#146c43");

  const start = toScreen(numberOr(config.robot_start_x, 0), numberOr(config.robot_start_y, 0));
  const startYaw = numberOr(config.robot_start_yaw_deg, 0) * Math.PI / 180;
  ctx.fillStyle = "#f6c445";
  ctx.strokeStyle = "#5e4b13";
  ctx.beginPath();
  ctx.arc(start.x, start.y, 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  drawArrow(ctx, start.x, start.y, start.x + Math.cos(startYaw) * 26, start.y - Math.sin(startYaw) * 26, "#7a5d00", 2);
  if (labels) drawText(ctx, "start", start.x + 8, start.y + 15, "#7a5d00");

  const collector = status.surface_collection?.collector_xyz;
  if (Array.isArray(collector) && collector.length >= 2 && finite(collector[0]) && finite(collector[1])) {
    const c = toScreen(Number(collector[0]), Number(collector[1]));
    const wx = Math.max(0, numberOr(status.surface_collection?.x_window_m, 0.85)) * scale;
    const wy = Math.max(0, numberOr(status.surface_collection?.y_window_m, 0.75)) * scale;
    ctx.save();
    ctx.strokeStyle = "#7c3aed";
    ctx.fillStyle = "rgba(124, 58, 237, 0.10)";
    ctx.lineWidth = 1.6;
    ctx.strokeRect(c.x - wx / 2, c.y - wy / 2, wx, wy);
    ctx.fillRect(c.x - wx / 2, c.y - wy / 2, wx, wy);
    ctx.beginPath();
    ctx.arc(c.x, c.y, 4, 0, Math.PI * 2);
    ctx.fillStyle = "#7c3aed";
    ctx.fill();
    ctx.restore();
    if (labels) drawText(ctx, "collector", c.x + 8, c.y + 16, "#5b21b6");
  }

  const buoys = Array.isArray(status.buoys) ? status.buoys : [];
  const targetId = status.target_id || "";
  for (const buoy of buoys) {
    if (!Array.isArray(buoy.xyz) || buoy.xyz.length < 2 || !finite(buoy.xyz[0]) || !finite(buoy.xyz[1])) continue;
    const p = toScreen(Number(buoy.xyz[0]), Number(buoy.xyz[1]));
    const style = buoyStyle(buoy, targetId);
    ctx.fillStyle = style.fill;
    ctx.strokeStyle = style.stroke;
    ctx.lineWidth = style.line;
    ctx.beginPath();
    ctx.arc(p.x, p.y, style.radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    if (style.failed) {
      ctx.strokeStyle = "#111827";
      ctx.beginPath();
      ctx.moveTo(p.x - 5, p.y - 5);
      ctx.lineTo(p.x + 5, p.y + 5);
      ctx.moveTo(p.x + 5, p.y - 5);
      ctx.lineTo(p.x - 5, p.y + 5);
      ctx.stroke();
    }
    if (labels && (style.isTarget || style.processed || style.netted)) {
      const label = `${shortId(buoy.id)} ${style.state.toLowerCase()}`;
      drawText(ctx, label, p.x + 10, p.y - 8, style.isTarget ? "#991b1b" : "#25364a");
    }
  }

  const robotSource = getRobotPose(data);
  const robotX = Number(robotSource.x);
  const robotY = Number(robotSource.y);
  const robotYaw = getRobotYaw(robotSource);
  updateRobotTrail(robotSource);
  if (els.mapShowTrail?.checked) drawTrail(ctx, toScreen);
  drawMetricRings(ctx, robotSource, toScreen, scale);
  const robot = drawRobot(ctx, robotSource, toScreen, scale);
  if (robot) {
    const cmd = status.command || {};
    const forward = Number(cmd.forward);
    const sway = Number(cmd.sway);
    if (Number.isFinite(forward) && Number.isFinite(sway) && Math.hypot(forward, sway) > 0.01) {
      const worldX = Math.cos(robotYaw) * forward - Math.sin(robotYaw) * sway;
      const worldY = Math.sin(robotYaw) * forward + Math.cos(robotYaw) * sway;
      drawArrow(ctx, robot.x, robot.y, robot.x + worldX * 48, robot.y - worldY * 48, "#245b2b", 2.5);
    }
    if (labels) drawText(ctx, `robot yaw ${fmt(robotYaw, 2)}`, robot.x + 12, robot.y + 22, "#062b50");
  }

  const detection = status.detection;
  const detWorld = detectionWorld(data);
  if (detection?.p_intake && detWorld && robot) {
    const p = toScreen(detWorld.x, detWorld.y);
    ctx.save();
    ctx.strokeStyle = "#8b3fb8";
    ctx.fillStyle = "rgba(139, 63, 184, 0.10)";
    ctx.lineWidth = 2;
    ctx.setLineDash([7, 5]);
    ctx.beginPath();
    ctx.moveTo(robot.x, robot.y);
    ctx.lineTo(p.x, p.y);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(p.x - 8, p.y);
    ctx.lineTo(p.x + 8, p.y);
    ctx.moveTo(p.x, p.y - 8);
    ctx.lineTo(p.x, p.y + 8);
    ctx.stroke();
    const rangePx = Math.max(0, Number(detection.distance_m || 0)) * scale;
    if (rangePx > 1) {
      ctx.beginPath();
      ctx.arc(robot.x, robot.y, rangePx, 0, Math.PI * 2);
      ctx.strokeStyle = "rgba(139, 63, 184, 0.28)";
      ctx.stroke();
    }
    ctx.restore();
    if (labels) drawText(ctx, `det ${fmt(detection.distance_m, 2)}m`, p.x + 10, p.y + 14, "#6b21a8");
  }

  const target = targetId ? shortId(targetId) : "none";
  const detText = detection ? `${shortId(detection.buoy_id)} ${fmt(detection.distance_m, 2)}m bearing ${fmt(detection.bearing_rad, 2)}` : "none";
  const cmd = status.command || {};
  const cmdMag = Math.hypot(Number(cmd.forward || 0), Number(cmd.sway || 0), Number(cmd.heave || 0), Number(cmd.yaw || 0));
  els.mapMeta.textContent = `scale ${fmt(1 / scale, 2)} m/px | boundary x=${fmt(bx, 1)} | margin ${fmt(margin, 1)} | standoff ${fmt(standoff, 1)} | buoys ${buoys.length}`;
  setPill(els.mapFsmChip, status.state || "FSM n/a", status.state ? "ok" : "");
  setPill(els.mapTargetChip, `target ${target}`, targetId ? "warn" : "");
  setPill(els.mapDetectionChip, detection ? `det ${fmt(detection.distance_m, 1)}m` : "detection none", detection ? "ok" : "");
  setPill(els.mapRcChip, `cmd ${fmt(cmdMag, 2)}`, cmdMag > 0.05 ? "warn" : "");
  els.mapRobotReadout.textContent = finite(robotX) && finite(robotY)
    ? `x ${fmt(robotX)} y ${fmt(robotY)} z ${fmt(robotSource.z)} yaw ${fmt(robotYaw)}`
    : "n/a";
  els.mapDetectionReadout.textContent = detText;
  const surface = status.surface_collection || {};
  els.mapSurfaceReadout.textContent = `rem ${surface.remaining ?? "n/a"} | gt ${surface.ground_truth_collect ?? "n/a"}`;
  const scoreState = status.score_zone || {};
  els.mapScoreReadout.textContent = `entered ${scoreState.entered ?? false} | buoys ${scoreState.collected_buoys_in_zone ?? false}`;
  els.mapHud.textContent = [
    `FSM: ${status.state || "NO_STATUS"} / ${status.robot_state_label || "n/a"}`,
    `Mode: ${status.mode || "n/a"}  elapsed: ${fmt(status.mission_elapsed_s, 1)}s`,
    `Target: ${target}`,
    `Detected: ${detText}`,
    `Counts: rem ${status.remaining_attached ?? 0} ok ${status.processed_count ?? 0} fail ${status.failed_count ?? 0} scored ${status.scored_count ?? 0}`,
    `Command: ${cmd.phase || "n/a"} f=${fmt(cmd.forward)} s=${fmt(cmd.sway)} h=${fmt(cmd.heave)} y=${fmt(cmd.yaw)}`,
    `Live: stale=${status.live_status?.stale ?? "n/a"} age=${fmt(status.live_status?.latest_age_s, 1)}s`
  ].join("\n");
}

function renderStatus(data) {
  latestStatus = data;
  setPill(els.serverState, "connected", "ok");
  setPill(els.rcAllowed, data.rc_send_allowed ? "RC send enabled" : "RC send locked", data.rc_send_allowed ? "warn" : "");
  setPill(els.rcStatus, data.rc_send_allowed ? "enabled" : "locked", data.rc_send_allowed ? "warn" : "");
  els.rcRelease.disabled = !data.rc_send_allowed;
  els.rcCenter.disabled = !data.rc_send_allowed;
  els.rcSendAxes.disabled = !data.rc_send_allowed;

  setConfigFields(data.config);
  els.cameraEnabled.checked = Boolean(data.camera?.enabled);
  const camAge = data.camera?.age;
  els.cameraEmpty.style.display = data.camera?.has_frame ? "none" : "grid";
  const vision = data.telemetry?.vision_status || {};
  els.cameraMeta.textContent = `source: ${data.camera?.source || "-"} | state: ${vision.state || "n/a"} | annotated: ${data.camera?.annotated_topic || "-"} | age: ${camAge == null ? "n/a" : fmt(camAge, 1) + "s"}`;
  if (data.camera?.enabled && lastCameraUrl === "") {
    reloadCamera();
  }

  const status = data.mission_status || {};
  els.statusPath.textContent = data.mission_status_path || "";
  els.fsmState.textContent = status.state || "NO_STATUS";
  els.robotState.textContent = status.robot_state_label || status.robot_state || "n/a";
  const target = [status.target_class, status.target_id].filter(Boolean).join(" ");
  els.targetState.textContent = target || "none";
  const rem = status.remaining_attached ?? 0;
  const ok = status.processed_count ?? 0;
  const fail = status.failed_count ?? 0;
  const scored = status.scored_count ?? 0;
  els.countState.textContent = `rem ${rem} | ok ${ok} | fail ${fail} | scored ${scored}`;
  const cmd = status.command || {};
  els.commandState.textContent = `${cmd.phase || "n/a"} f=${fmt(cmd.forward)} s=${fmt(cmd.sway)} h=${fmt(cmd.heave)} y=${fmt(cmd.yaw)}`;
  const robot = status.robot || data.telemetry?.pose || {};
  els.robotPose.textContent = `x ${fmt(robot.x)} y ${fmt(robot.y)} z ${fmt(robot.z)} yaw ${fmt(robot.yaw_rad ?? robot.yaw)}`;

  const rc = data.telemetry?.last_rc_override || [];
  const manual = data.telemetry?.last_manual_control || {};
  els.rcReadout.textContent = [
    `override: ${rc.slice(0, 8).join(" ") || "n/a"}`,
    `manual: x=${manual.x ?? "n/a"} y=${manual.y ?? "n/a"} z=${manual.z ?? "n/a"} r=${manual.r ?? "n/a"}`,
    `mavros: connected=${data.telemetry?.mavros_state?.connected} armed=${data.telemetry?.mavros_state?.armed} mode=${data.telemetry?.mavros_state?.mode || ""}`
  ].join("\n");

  renderTopics(data.topics);
  renderBuoys(status);
  drawMissionMap(data);
  renderProcesses(data.processes);
  const extraLogs = Array.isArray(data.logs) ? data.logs.slice(-80).join("\n") : "";
  if (extraLogs) {
    els.processLog.textContent = `${els.processLog.textContent}\n${extraLogs}`.trim();
  }
}

async function refreshStatus() {
  try {
    const res = await fetch("/api/status", {cache: "no-store"});
    const data = await res.json();
    renderStatus(data);
  } catch (err) {
    setPill(els.serverState, "disconnected", "bad");
  }
}

function reloadCamera() {
  lastCameraUrl = `/api/camera.mjpg?t=${Date.now()}`;
  els.cameraFeed.src = lastCameraUrl;
}

els.saveConfig.addEventListener("click", () => saveConfig().catch(alert));
els.startDry.addEventListener("click", () => startProcess("mission", {dry_run: true}).catch(alert));
els.startLive.addEventListener("click", () => startProcess("mission", {dry_run: false}).catch(alert));
els.stopMission.addEventListener("click", () => stopProcess("mission").catch(alert));
els.startPinger.addEventListener("click", () => startProcess("pinger").catch(alert));
els.stopPinger.addEventListener("click", () => stopProcess("pinger").catch(alert));
els.startMarkers.addEventListener("click", () => startProcess("markers").catch(alert));
els.stopMarkers.addEventListener("click", () => stopProcess("markers").catch(alert));
els.startRviz.addEventListener("click", () => startProcess("rviz").catch(alert));
els.stopRviz.addEventListener("click", () => stopProcess("rviz").catch(alert));
els.rcRelease.addEventListener("click", () => sendRc("release").catch(alert));
els.rcCenter.addEventListener("click", () => sendRc("center").catch(alert));
els.rcSendAxes.addEventListener("click", () => sendRc("axes").catch(alert));
els.reloadCamera.addEventListener("click", reloadCamera);
els.cameraEnabled.addEventListener("change", () => saveConfig().then(reloadCamera).catch(alert));

for (const id of [
  "tankXMin", "tankXMax", "tankYMin", "tankYMax", "robotStartX", "robotStartY",
  "robotStartYaw", "scoreZoneX", "scoreZoneY", "scoreZoneRadius", "boundaryX", "pingerDepthZ",
  "tankMaxDepth",
  "boundaryMargin", "boundaryStandoff", "ownCourse", "mapShowGrid", "mapShowLabels",
  "mapShowTrail"
]) {
  els[id].addEventListener("input", () => {
    if (!latestStatus) return;
    latestStatus.config = {...latestStatus.config, ...collectConfig()};
    drawMissionMap(latestStatus);
  });
}

window.addEventListener("resize", () => {
  if (latestStatus) drawMissionMap(latestStatus);
});

refreshStatus();
setInterval(refreshStatus, 700);
