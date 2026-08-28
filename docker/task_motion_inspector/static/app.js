const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
const HISTORY_SECONDS = 60;
const SERIES = {
  source: { color: "#8a94a6", label: "Leader target", dash: [2, 3], width: 1.4 },
  policy: { color: "#7656d8", label: "Policy preview", dash: [7, 4], width: 2.2 },
  command: { color: "#1769e0", label: "Final trajectory", dash: [], width: 2.0 },
  reference: { color: "#19a0b8", label: "Controller ref", dash: [4, 3], width: 1.6 },
  feedback: { color: "#138a5b", label: "Feedback", dash: [], width: 2.1 },
  dxl_present: { color: "#ef8a27", label: "DXL present", dash: [1, 3], width: 2.1 },
};
const POSITION_SERIES = ["command", "feedback", "source", "reference", "dxl_present", "policy"];
const LIVE_SERIES = ["source", "policy", "command", "reference", "feedback", "dxl_present", "tracking_error"];

let state = null;
let selectedJoint = "arm_l_joint7";
let filter = "all";
let historyData = { joint: "arm_l_joint7", series: {}, limits: null };
let sessions = [];
let historyRequest = 0;
let historyPollTimer = null;
let chartFrame = null;
let renderedJointNames = "";
let jointRowElements = new Map();

function esc(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;").replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;").replaceAll('"', "&quot;").replaceAll("'", "&#039;");
}

function fmtRad(value, digits = 2) {
  return Number.isFinite(value) ? `${(value * DEG).toFixed(digits)}°` : "—";
}

function fmtNumber(value, digits = 1) {
  return Number.isFinite(value) ? Number(value).toFixed(digits) : "—";
}

function toast(message, error = false) {
  const element = $("toast");
  element.textContent = message;
  element.className = `toast show${error ? " error" : ""}`;
  clearTimeout(element.timer);
  element.timer = setTimeout(() => { element.className = "toast"; }, 3500);
}

async function api(url, options = {}) {
  const response = await fetch(url, {
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    ...options,
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.detail || `HTTP ${response.status}`);
  return data;
}

function visible(name) {
  return filter === "all"
    || (filter === "left" && name.startsWith("arm_l_"))
    || (filter === "right" && name.startsWith("arm_r_"))
    || (filter === "j7" && name.endsWith("joint7"));
}

function severity(row) {
  if (row.tracking_critical || row.hard_limit_saturation
      || (row.flags || []).some((item) => ["OUTSIDE_DXL_RANGE", "HARD_LIMIT_SATURATION"].includes(item))) {
    return "critical";
  }
  if (row.tracking_warning || (row.flags || []).length) return "warning";
  return "normal";
}

function shortState(row) {
  if (row.hard_limit_saturation) return "SATURATED";
  if (row.tracking_critical) return "CRITICAL";
  if (row.tracking_warning) return "TRACKING";
  if (row.stale) return "STALE";
  if ((row.flags || []).includes("OUTSIDE_DXL_RANGE")) return "LIMIT";
  if ((row.flags || []).length) return "CHECK";
  return "NORMAL";
}

function renderJoints(joints) {
  const entries = Object.entries(joints).filter(([name]) => visible(name));
  const body = $("jointRows");
  if (!entries.length) {
    if (renderedJointNames !== "__empty__") {
      body.innerHTML = '<tr><td colspan="6" class="empty">수신된 관절 신호가 없습니다.</td></tr>';
      renderedJointNames = "__empty__";
      jointRowElements = new Map();
    }
    return;
  }
  const signature = entries.map(([name]) => name).join("|");
  if (signature !== renderedJointNames) {
    body.innerHTML = entries.map(([name]) => `<tr data-joint="${esc(name)}" tabindex="0" aria-selected="false">
      <td>${esc(name.replace("arm_", "").toUpperCase())}</td>
      <td data-field="command">—</td>
      <td data-field="reference">—</td>
      <td data-field="feedback">—</td>
      <td data-field="error">—</td>
      <td><span class="state-pill normal">NORMAL</span></td>
    </tr>`).join("");
    renderedJointNames = signature;
    jointRowElements = new Map(
      [...body.querySelectorAll("tr[data-joint]")].map((element) => [element.dataset.joint, element]),
    );
  }
  entries.forEach(([name, row]) => {
    const error = Number.isFinite(row.tracking_error)
      ? row.tracking_error
      : Number.isFinite(row.reference) && Number.isFinite(row.feedback)
        ? row.reference - row.feedback : null;
    const level = severity(row);
    const element = jointRowElements.get(name);
    if (!element) return;
    element.classList.toggle("selected", name === selectedJoint);
    element.classList.toggle("alert", level !== "normal");
    element.setAttribute("aria-selected", String(name === selectedJoint));
    element.querySelector('[data-field="command"]').textContent = fmtRad(row.command);
    element.querySelector('[data-field="reference"]').textContent = fmtRad(row.reference);
    element.querySelector('[data-field="feedback"]').textContent = fmtRad(row.feedback);
    element.querySelector('[data-field="error"]').textContent = fmtRad(error);
    const statePill = element.querySelector(".state-pill");
    statePill.className = `state-pill ${level}`;
    statePill.textContent = shortState(row);
  });
}

function selectJoint(name) {
  if (!name || name === selectedJoint) return;
  selectedJoint = name;
  historyData = { joint: name, series: {}, limits: state?.joints?.[name]?.limits || null };
  if (state) {
    renderJoints(state.joints);
    renderSelected(state.joints[name]);
  }
  loadHistory();
}

function appendPoint(key, stamp, value) {
  if (!Number.isFinite(stamp) || !Number.isFinite(value)) return;
  const points = historyData.series[key] || (historyData.series[key] = []);
  const last = points[points.length - 1];
  if (last && stamp < last[0]) return;
  if (last && stamp === last[0]) {
    last[1] = value;
  } else {
    points.push([stamp, value]);
  }
  const cutoff = Date.now() / 1000 - HISTORY_SECONDS;
  while (points.length && points[0][0] < cutoff) points.shift();
}

function appendLiveHistory(next) {
  const row = next?.joints?.[selectedJoint];
  if (!row) return;
  if (historyData.joint !== selectedJoint) {
    historyData = { joint: selectedJoint, series: {}, limits: row.limits || null };
  }
  historyData.limits = row.limits || historyData.limits;
  LIVE_SERIES.forEach((key) => {
    // snapshot.source is the active source alias. Only append it to the
    // Leader target series when it actually represents the teleop source.
    if (key === "source" && row.source_kind !== "transformed_leader") return;
    appendPoint(key, row[`${key}_stamp`], row[key]);
  });
}

function scheduleChartDraw() {
  if (chartFrame != null) return;
  chartFrame = window.requestAnimationFrame(() => {
    chartFrame = null;
    drawCharts();
  });
}

function sourceLabel(commandSource, row) {
  const mode = String(commandSource?.mode || "UNKNOWN").toUpperCase();
  const leaderTopic = selectedJoint.startsWith("arm_l_")
    ? "/robot/arm_left_leader/joint_states"
    : "/robot/arm_right_leader/joint_states";
  if (mode === "POLICY") return "Policy preview · /inference/trajectory_preview";
  if (mode === "TELEOP") return `Leader target (JointState) · ${leaderTopic}`;
  return "명령 발생원 없음 · controller state만 모니터링 중";
}

function renderSelected(row) {
  $("selectedTitle").textContent = selectedJoint.toUpperCase();
  $("selectedSource").textContent = sourceLabel(state?.command_source, row);
  const level = row ? severity(row) : "normal";
  const pill = $("selectedState");
  pill.className = `state-pill ${level}`;
  pill.textContent = row ? shortState(row) : "WAITING";
  const error = Number.isFinite(row?.tracking_error)
    ? row.tracking_error
    : Number.isFinite(row?.reference) && Number.isFinite(row?.feedback)
      ? row.reference - row.feedback : null;
  $("selectedError").textContent = Number.isFinite(error) ? `Error ${fmtRad(error)}` : "—";
  $("selectedCurrent").textContent = Number.isFinite(row?.dxl_current) ? fmtNumber(row.dxl_current, 1) : "—";

  const limits = row?.limits || historyData.limits;
  if (!limits) {
    $("selectedRange").textContent = "not configured";
    $("limitSource").textContent = "—";
    $("limitVisual").style.visibility = "hidden";
  } else {
    $("selectedRange").textContent = `${fmtRad(limits.hardware[0], 1)} ~ ${fmtRad(limits.hardware[1], 1)}`;
    const resolution = Number.isFinite(limits.ticks_per_rev)
      ? `${Number(limits.ticks_per_rev).toLocaleString()} tick/rev` : "resolution unknown";
    $("limitSource").textContent = `${limits.model || "model unknown"} · ${resolution} · ${limits.source || "runtime configuration"}`;
    $("limitVisual").style.visibility = "visible";
    const span = limits.hardware[1] - limits.hardware[0];
    const position = Number.isFinite(row?.dxl_present) ? row.dxl_present : row?.feedback;
    const percent = Number.isFinite(position) && span > 0
      ? Math.max(0, Math.min(100, 100 * (position - limits.hardware[0]) / span)) : 50;
    $("limitVisual").querySelector(".position-marker").style.left = `${percent}%`;
  }
}

function renderSummary(next) {
  const source = next.command_source || {};
  const sourceMode = String(source.mode || "UNKNOWN").toUpperCase();
  $("summarySource").textContent = sourceMode;
  $("sourceDetail").textContent = source.fault || (source.inferred ? "topic activity에서 추정" : "command arbiter");
  $("sourceMode").textContent = `SOURCE · ${sourceMode}`;
  $("sourceMode").className = `badge ${sourceMode === "UNKNOWN" ? "warning" : "neutral"}`;

  const maxError = next.summary?.max_tracking_error_deg;
  $("summaryError").textContent = Number.isFinite(maxError) ? `${maxError.toFixed(2)}°` : "—";
  $("errorThreshold").textContent = `경고 ${fmtNumber(next.thresholds?.tracking_warn_deg, 1)}° · 임계 ${fmtNumber(next.thresholds?.tracking_critical_deg, 1)}°`;
  const alerts = next.summary?.alert_joints || 0;
  $("summaryAlerts").textContent = String(alerts);
  $("alertDetail").textContent = alerts ? `${alerts}개 관절 확인 필요` : "정상";

  const comm = next.dxl?.comm_state;
  $("summaryDxl").textContent = comm === 0 ? "Healthy" : comm == null ? "No data" : `Error ${comm}`;
  $("configDigest").textContent = `config ${next.summary?.config_digest || "fallback"}`;
}

function renderMode(next) {
  const labels = {
    idle: "CONNECTED",
    recording: `RECORDING · ${next.active_session || ""}`,
  };
  const badge = $("modeText");
  badge.textContent = labels[next.mode] || String(next.mode).toUpperCase();
  badge.className = "badge online";

  const recordState = $("recordState");
  recordState.className = `dot-label${next.mode === "recording" ? " recording" : ""}`;
  recordState.innerHTML = `<i></i>${next.mode === "recording" ? "Recording" : "Idle"}`;
  updateButtons();
}

function renderTopics(topics) {
  const entries = Object.entries(topics || {});
  const healthy = entries.filter(([, item]) => item.available && item.age != null && item.age < 2).length;
  $("topicSummary").textContent = `${healthy}/${entries.length} live`;
  $("topicList").innerHTML = entries.map(([name, item]) => {
    const kind = !item.available ? "missing" : item.age == null || item.age >= 2 ? "stale" : "live";
    const status = !item.available ? "missing" : item.age == null ? "no data" : `${item.age.toFixed(1)} s`;
    return `<div class="topic-item ${kind}"><code>${esc(name)}</code><span>${status}</span></div>`;
  }).join("");
}

function renderEvents(events) {
  const list = $("events");
  const visibleEvents = (events || []).slice(0, 30);
  if (!visibleEvents.length) {
    list.innerHTML = '<div class="empty">이벤트 없음</div>';
    return;
  }
  list.innerHTML = visibleEvents.map((event) => {
    const time = String(event.time || "").slice(11, 19);
    return `<div class="event-item">
      <time>${esc(time)}</time>
      <span class="event-level ${esc(event.level || "info")}">${esc((event.level || "info").toUpperCase())}</span>
      <span class="event-joint">${esc(event.joint || event.type || "SYSTEM")}</span>
      <span class="event-message" title="${esc(event.message)}">${esc(event.message)}</span>
    </div>`;
  }).join("");
}

function render(next) {
  const priorMode = state?.mode;
  state = next;
  if (!next.joints[selectedJoint]) {
    selectedJoint = Object.keys(next.joints).find((name) => name.endsWith("joint7"))
      || Object.keys(next.joints)[0] || selectedJoint;
  }
  renderSummary(next);
  renderMode(next);
  renderJoints(next.joints);
  renderSelected(next.joints[selectedJoint]);
  renderTopics(next.topics);
  renderEvents(next.events);
  appendLiveHistory(next);
  scheduleChartDraw();
  if (priorMode && priorMode !== next.mode && next.mode === "idle") loadSessions();
}

function drawGrid(ctx, width, height, padding, min, max, suffix) {
  ctx.font = "10px system-ui";
  ctx.strokeStyle = "#edf0f3";
  ctx.fillStyle = "#8a93a2";
  ctx.lineWidth = 1;
  for (let index = 0; index <= 4; index += 1) {
    const y = padding.t + (height - padding.t - padding.b) * index / 4;
    ctx.beginPath(); ctx.moveTo(padding.l, y); ctx.lineTo(width - padding.r, y); ctx.stroke();
    const value = max - (max - min) * index / 4;
    ctx.fillText(`${value.toFixed(Math.abs(max - min) < 5 ? 1 : 0)}${suffix}`, 5, y + 3);
  }
}

function prepareCanvas(canvas, height) {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(rect.width * ratio));
  canvas.height = Math.round(height * ratio);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, rect.width, height);
  return { ctx, width: rect.width, height };
}

function drawPositionChart() {
  const { ctx, width, height } = prepareCanvas($("positionChart"), 220);
  const padding = { l: 44, r: 14, t: 10, b: 20 };
  const series = historyData.series || {};
  const keys = POSITION_SERIES;
  const points = keys.flatMap((key) => (series[key] || []).filter((point) => Number.isFinite(point[1])));
  $("positionLegend").querySelectorAll("[data-series]").forEach((item) => {
    const available = (series[item.dataset.series] || []).some((point) => Number.isFinite(point[1]));
    item.classList.toggle("unavailable", !available);
    item.title = available ? `${SERIES[item.dataset.series].label} 데이터 수신 중` : "최근 60초 데이터 없음";
  });
  if (!points.length) {
    ctx.fillStyle = "#98a0ae"; ctx.font = "11px system-ui"; ctx.fillText("데이터 대기 중", padding.l, 35); return;
  }
  const end = Math.max(...points.map((point) => point[0]));
  const start = end - HISTORY_SECONDS;
  const values = points.map((point) => point[1] * DEG);
  let min = Math.min(...values); let max = Math.max(...values);
  const margin = Math.max(2, (max - min) * 0.12);
  min -= margin; max += margin;
  if (max - min < 4) { min -= 2; max += 2; }
  drawGrid(ctx, width, height, padding, min, max, "°");
  const x = (stamp) => padding.l + (width - padding.l - padding.r) * (stamp - start) / HISTORY_SECONDS;
  const y = (value) => padding.t + (height - padding.t - padding.b) * (max - value * DEG) / (max - min);
  keys.forEach((key) => {
    ctx.beginPath(); let begun = false;
    (series[key] || []).forEach(([stamp, value]) => {
      if (!Number.isFinite(value) || stamp < start) return;
      if (!begun) { ctx.moveTo(x(stamp), y(value)); begun = true; } else ctx.lineTo(x(stamp), y(value));
    });
    ctx.strokeStyle = SERIES[key].color;
    ctx.lineWidth = SERIES[key].width;
    ctx.setLineDash(SERIES[key].dash);
    ctx.stroke();
  });
  ctx.setLineDash([]);
}

function drawErrorChart() {
  const { ctx, width, height } = prepareCanvas($("errorChart"), 120);
  const padding = { l: 44, r: 14, t: 10, b: 20 };
  let points = (historyData.series?.tracking_error || []).filter((point) => Number.isFinite(point[1]));
  if (!points.length) {
    const reference = historyData.series?.reference || [];
    const feedback = historyData.series?.feedback || [];
    const feedbackByStamp = new Map(feedback.map((point) => [point[0], point[1]]));
    points = reference.filter((point) => feedbackByStamp.has(point[0])).map((point) => [point[0], point[1] - feedbackByStamp.get(point[0])]);
  }
  if (!points.length) {
    ctx.fillStyle = "#98a0ae"; ctx.font = "11px system-ui"; ctx.fillText("오차 데이터 대기 중", padding.l, 30); return;
  }
  const end = Math.max(...points.map((point) => point[0]));
  const start = end - HISTORY_SECONDS;
  const warn = Number(state?.thresholds?.tracking_warn_deg || 2);
  const peak = Math.max(warn * 1.35, ...points.map((point) => Math.abs(point[1] * DEG)));
  const min = -peak; const max = peak;
  drawGrid(ctx, width, height, padding, min, max, "°");
  const x = (stamp) => padding.l + (width - padding.l - padding.r) * (stamp - start) / HISTORY_SECONDS;
  const y = (valueDeg) => padding.t + (height - padding.t - padding.b) * (max - valueDeg) / (max - min);
  ctx.setLineDash([4, 4]); ctx.strokeStyle = "#e1a33e";
  [warn, -warn].forEach((level) => { ctx.beginPath(); ctx.moveTo(padding.l, y(level)); ctx.lineTo(width - padding.r, y(level)); ctx.stroke(); });
  ctx.setLineDash([]); ctx.beginPath(); let begun = false;
  points.forEach(([stamp, value]) => {
    if (stamp < start) return;
    if (!begun) { ctx.moveTo(x(stamp), y(value * DEG)); begun = true; } else ctx.lineTo(x(stamp), y(value * DEG));
  });
  ctx.strokeStyle = "#d23a43"; ctx.lineWidth = 1.7; ctx.stroke();
}

function drawCharts() { drawPositionChart(); drawErrorChart(); }

async function loadHistory() {
  window.clearTimeout(historyPollTimer);
  const request = ++historyRequest;
  const joint = selectedJoint;
  try {
    const data = await api(`/api/history/${encodeURIComponent(joint)}?seconds=${HISTORY_SECONDS}&max_points=800`);
    if (request !== historyRequest || joint !== selectedJoint) return;
    historyData = data;
    appendLiveHistory(state);
    $("historyWindow").textContent = `최근 ${data.seconds} s`;
    scheduleChartDraw();
  } catch (error) {
    if (request === historyRequest) console.warn("history fetch failed", error);
  } finally {
    if (request === historyRequest && joint === selectedJoint) {
      historyPollTimer = window.setTimeout(loadHistory, 5000);
    }
  }
}

function updateButtons() {
  const mode = state?.mode || "idle";
  $("recordStart").disabled = mode !== "idle";
  $("recordStop").disabled = mode !== "recording";
}

async function loadSessions(preselect = "") {
  try {
    sessions = await api("/api/sessions");
    const select = $("sessionSelect");
    const current = preselect || select.value;
    select.innerHTML = sessions.length
      ? sessions.map((item) => `<option value="${esc(item.name)}">${esc(item.name)}${item.has_report ? " · report" : ""}</option>`).join("")
      : '<option value="">세션 없음</option>';
    if (current && sessions.some((item) => item.name === current)) select.value = current;
    updateReportLink();
    updateButtons();
  } catch (error) { toast(error.message, true); }
}

function updateReportLink() {
  const selected = sessions.find((item) => item.name === $("sessionSelect").value);
  const button = $("reportLink");
  button.disabled = !selected;
  button.textContent = selected?.has_report ? "Report" : "Analyze";
}

$("recordStart").addEventListener("click", async () => {
  try {
    const result = await api("/api/record/start", { method: "POST", body: JSON.stringify({ name: $("sessionName").value }) });
    toast(`녹화 시작: ${result.session}`);
  } catch (error) { toast(error.message, true); }
});

$("recordStop").addEventListener("click", async () => {
  try {
    const result = await api("/api/record/stop", { method: "POST" });
    toast(`녹화 완료: ${result.session} · 분석 중`);
    setTimeout(() => loadSessions(result.session), 1200);
  } catch (error) { toast(error.message, true); }
});

$("reportLink").addEventListener("click", async () => {
  const selected = sessions.find((item) => item.name === $("sessionSelect").value);
  if (!selected) return;
  if (selected.has_report) {
    window.open(`/api/sessions/${encodeURIComponent(selected.name)}/report`, "_blank", "noopener");
    return;
  }
  const button = $("reportLink");
  button.disabled = true; button.textContent = "Analyzing…";
  try {
    await api(`/api/sessions/${encodeURIComponent(selected.name)}/analyze`, { method: "POST" });
    toast(`분석 완료: ${selected.name}`);
    await loadSessions(selected.name);
  } catch (error) {
    toast(error.message, true); button.disabled = false; button.textContent = "Analyze";
  }
});

$("sessionSelect").addEventListener("change", () => { updateReportLink(); updateButtons(); });
// The table updates at 10 Hz. Handle selection on the stable row container so
// a data refresh cannot replace the clicked target between pointer down/up.
$("jointRows").addEventListener("pointerdown", (event) => {
  const row = event.target.closest("tr[data-joint]");
  if (row) selectJoint(row.dataset.joint);
});
$("jointRows").addEventListener("keydown", (event) => {
  if (event.key !== "Enter" && event.key !== " ") return;
  const row = event.target.closest("tr[data-joint]");
  if (!row) return;
  event.preventDefault();
  selectJoint(row.dataset.joint);
});
$("sideFilter").querySelectorAll("button").forEach((button) => {
  button.addEventListener("click", () => {
    $("sideFilter").querySelectorAll("button").forEach((item) => item.classList.remove("active"));
    button.classList.add("active"); filter = button.dataset.filter;
    if (!state) return;
    if (!visible(selectedJoint)) {
      const firstVisible = Object.keys(state.joints).find((name) => visible(name));
      if (firstVisible) selectJoint(firstVisible);
    }
    renderJoints(state.joints);
  });
});
$("clearEvents").addEventListener("click", () => { $("events").innerHTML = '<div class="empty">이벤트 숨김</div>'; });
window.addEventListener("resize", drawCharts);

loadSessions();
loadHistory();
setInterval(() => { if (state?.mode === "idle") loadSessions(); }, 5000);
const stream = new EventSource("/api/events");
stream.onmessage = (event) => render(JSON.parse(event.data));
stream.onerror = () => {
  $("modeText").textContent = "DISCONNECTED";
  $("modeText").className = "badge offline";
};
