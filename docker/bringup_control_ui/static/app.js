const $ = (id) => document.getElementById(id);
let state = null;
let pendingAction = null;
let toastTimer = null;
let lastObservedMode = null;
const tokenKey = "ai-worker-control-token";
const clientKey = "ai-worker-control-client";
const pageKey = "ai-worker-control-page";
let clientId = sessionStorage.getItem(clientKey);
if (!clientId || !/^[\x20-\x7e]+$/.test(clientId)) {
  clientId = `client-${Date.now().toString(36).slice(-4)}${Math.random().toString(36).slice(2, 7)}`;
  sessionStorage.setItem(clientKey, clientId);
}

const pageMeta = {
  home: ["AI WORKER", "Home"],
  teleop: ["OPERATION MODE", "Teleop"],
  inference: ["OPERATION MODE", "Inference"],
  system: ["ROBOT HEALTH", "System"],
};

const actionMeta = {
  follower_start: { title: "Follower Bring-up", description: "선택한 로봇의 hardware와 controller를 시작합니다. Inference와 Leader가 완전히 종료된 상태여야 합니다." },
  leader_start: { title: "Teleop 시작", description: "Leader가 command를 발행하면 켜져 있는 Follower가 즉시 움직일 수 있습니다. 로봇 주변을 확인하십시오." },
  bring_down: { title: "F2 Bring-down", description: "F2를 공통 안전 자세를 거쳐 arms-down 자세로 이동합니다. Leader는 자동 종료되지 않습니다." },
  recover_init: { title: "Init pose 복구", description: "Teleop/Inference command가 완전히 분리된 것을 재확인한 뒤 선택 모델의 YAML init pose로 이동합니다." },
};

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>'"]/g, (c) => ({"&":"&amp;","<":"&lt;",">":"&gt;","'":"&#39;",'"':"&quot;"}[c]));
}

function showToast(message, bad=false) {
  const el = $("toast");
  el.textContent = message;
  el.className = `toast${bad ? " bad" : ""}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.add("hidden"), 5500);
}

function statusText(el, text, kind="") { el.textContent = text; el.className = kind; }
function setBadge(el, up, upText="UP", downText="DOWN") { el.textContent = up ? upText : downText; el.className = `badge ${up ? "good" : "neutral"}`; }

function setPage(page) {
  if (!pageMeta[page]) page = "home";
  document.querySelectorAll("[data-page-content]").forEach((el) => el.classList.toggle("active", el.dataset.pageContent === page));
  document.querySelectorAll(".nav-item[data-page]").forEach((el) => el.classList.toggle("active", el.dataset.page === page));
  $("page-kicker").textContent = pageMeta[page][0];
  $("page-name").textContent = pageMeta[page][1];
  sessionStorage.setItem(pageKey, page);
  window.location.hash = page;
}

function initialPage() {
  const hash = window.location.hash.replace("#", "");
  return pageMeta[hash] ? hash : pageMeta[sessionStorage.getItem(pageKey)] ? sessionStorage.getItem(pageKey) : "home";
}

document.querySelectorAll("[data-page]").forEach((button) => button.addEventListener("click", () => setPage(button.dataset.page)));
document.querySelectorAll("[data-nav-page]").forEach((link) => link.addEventListener("click", (event) => { event.preventDefault(); setPage(link.dataset.navPage); }));
window.addEventListener("hashchange", () => setPage(initialPage()));
setPage(initialPage());

function fillModels() {
  const follower = $("follower-model");
  const leader = $("leader-model");
  const syncOptions = (select, models, selected) => {
    const keys = Object.keys(models);
    const current = Array.from(select.options).map((option) => option.value);
    if (current.join("|") !== keys.join("|")) {
      const previous = select.value;
      select.replaceChildren();
      Object.entries(models).forEach(([key, model]) => select.add(new Option(model.label, key)));
      select.value = models[previous] ? previous : selected;
    }
  };
  syncOptions(follower, state.models.followers, state.selected.follower);
  syncOptions(leader, state.models.leaders, state.selected.leader);
  const transition = state.control?.current;
  if (state.safety.follower_runtime_active || transition?.action?.startsWith("follower_")) {
    follower.value = transition?.model || state.selected.follower;
  }
  if (state.safety.leader_runtime_active || transition?.action?.startsWith("leader_")) {
    leader.value = transition?.model || state.selected.leader;
  }
  const fm = state.models.followers[follower.value];
  const lm = state.models.leaders[leader.value];
  $("teleop-leader-name").textContent = `${lm?.label || leader.value.toUpperCase()} Leader`;
  follower.disabled = state.safety.follower_runtime_active || state.control?.running;
  leader.disabled = state.safety.leader_runtime_active || state.control?.running;
}

function gateButton(id, action, extra=true, extraReason="필수 안전 조건을 확인해 주세요.", supplemental=[]) {
  const gate = state.gates[action] || {ok:false, failures:["상태를 확인할 수 없습니다."]};
  const button = $(id);
  const failures = [...(gate.failures || []), ...supplemental];
  if (!extra && gate.ok) failures.push(extraReason);
  const blocked = !gate.ok || !extra;
  // Native disabled buttons do not receive clicks. aria-disabled keeps the
  // disabled appearance while allowing the UI to explain the exact reason.
  button.disabled = false;
  button.classList.toggle("is-disabled", blocked);
  button.setAttribute("aria-disabled", String(blocked));
  button.dataset.blockedReasons = blocked ? JSON.stringify([...new Set(failures)]) : "";
  button.title = "";
}

function renderMode() {
  const mode = state.mode || {name:"idle", label:"STANDBY", detail:"모드 상태 확인 중"};
  if (mode.name !== lastObservedMode) {
    const previousMode = lastObservedMode;
    const visiblePage = document.querySelector(".app-page.active")?.dataset.pageContent;
    if (mode.name === "teleop" || mode.name === "inference") setPage(mode.name);
    if (mode.name === "intervention") setPage("inference");
    if (mode.name === "conflict") setPage("system");
    if (mode.name === "idle" && (
      ["teleop", "inference", "intervention", "conflict"].includes(previousMode)
      || (previousMode === null && ["teleop", "inference"].includes(visiblePage))
    )) setPage("home");
    lastObservedMode = mode.name;
  }
  document.body.dataset.robotMode = mode.name;
  const chip = $("global-mode-chip");
  chip.className = `mode-chip ${mode.name}`;
  chip.querySelector("b").textContent = mode.label;
  const hero = $("home-mode-banner");
  hero.className = `mode-hero ${mode.name}`;
  $("home-mode-label").textContent = mode.label;

  const leaderUp = state.safety.leader_runtime_active;
  const intervention = state.intervention?.current;
  const teleopBadge = $("teleop-page-badge");
  teleopBadge.textContent = mode.name === "conflict" ? "CONFLICT" : mode.name === "intervention" ? "HAND-OFF" : leaderUp ? "ACTIVE" : "OFF";
  teleopBadge.className = `mode-page-badge ${leaderUp ? "active teleop" : ""}`;
  $("teleop-stage-title").textContent = mode.name === "conflict" ? "Command 충돌 감지" : leaderUp ? "Teleop 활성" : "Teleop 대기";

  const inference = state.ros.inference;
  const inferenceBadge = $("inference-page-badge");
  const phase = inference.phase || "UNAVAILABLE";
  inferenceBadge.textContent = intervention ? "INTERVENTION" : inference.present ? phase : "DISCONNECTED";
  inferenceBadge.className = `mode-page-badge ${inference.present ? "active inference" : ""}`;
  $("inference-stage-title").textContent = mode.name === "conflict" ? "Runtime 충돌 감지" : intervention ? `사람 조작 · ${intervention.phase}` : inference.present ? `${phase}` : "Inference 대기";
  $("inference-phase-large").textContent = intervention ? `INTERVENTION · ${intervention.phase}` : inference.present ? phase : "DISCONNECTED";
}

function renderSafety() {
  const s = state.safety;
  const r = state.ros;
  const checks = [
    [!r.inference.present && !r.inference.active, "Inference 분리", r.inference.present ? `연결됨: ${[...r.inference.services, ...(r.inference.rtc_publishers || [])].join(", ")}` : "policy runtime/service 없음"],
    [!s.leader_runtime_active && !s.leader_stream_active, "Leader command 차단", s.teleop_active ? "teleop 활성" : "Leader/command quiet"],
    [!s.command_stream_active, "Trajectory source 분리", s.command_publishers_present ? "command publisher 연결됨" : s.command_recent ? "최근 command 감지" : "외부 command 없음"],
    [s.controllers_ready, "Trajectory controllers", s.controllers_ready ? "arm/head/lift feedback 정상" : "controller state 미수신"],
    [s.feedback_fresh, "Joint feedback", s.feedback_fresh ? "최신 /joint_states" : "feedback 없음/지연"],
    [s.dxl_ok, "DXL health", s.dxl_ok ? "통신 및 hardware error 정상" : "미확인 또는 오류"],
    [!r.estop.active && (!r.estop.configured || (r.estop.signal_seen && r.estop.age !== null && r.estop.age < 2)), "E-stop", r.estop.configured ? (r.estop.active ? "ACTIVE" : r.estop.signal_seen && r.estop.age < 2 ? "최신 해제 신호" : "토픽 신호 대기/지연") : "직접 신호 미설정"],
  ];
  $("safety-checks").innerHTML = checks.map(([ok,title,detail], index) => {
    const warn = index === 6 && !r.estop.configured;
    const kind = warn ? "warn" : ok ? "good" : "bad";
    return `<div class="check-item"><span class="check-icon ${kind}">${warn ? "!" : ok ? "✓" : "×"}</span><div><strong>${escapeHtml(title)}</strong><small>${escapeHtml(detail)}</small></div></div>`;
  }).join("");
  $("safe-count").textContent = `${checks.filter((row) => row[0]).length} / ${checks.length} SAFE`;
}

function renderDiagnostics() {
  $("diagnostics").innerHTML = state.diagnostics.length
    ? state.diagnostics.map((item) => `<div class="diagnostic ${item.severity}"><strong>${escapeHtml(item.title)}</strong><p>${escapeHtml(item.guidance)}</p><code title="${escapeHtml(item.evidence)}">${escapeHtml(item.evidence)}</code></div>`).join("")
    : `<div class="empty-state">현재 로그에서 분류된 경고가 없습니다.</div>`;
  $("events").innerHTML = state.ros.events.length
    ? state.ros.events.slice(0,12).map((item) => `<div class="event ${item.level}"><span class="event-dot"></span><div><p>${escapeHtml(item.message)}</p><time>${escapeHtml(item.time)}</time></div></div>`).join("")
    : `<div class="empty-state">기록된 제어 이벤트가 없습니다.</div>`;
}

function renderOperation() {
  const op = state.operation;
  const shared = state.control?.current;
  $("operation-panel").classList.toggle("hidden", !shared && !op.running && !op.last_result);
  if (shared) {
    $("operation-badge").textContent = shared.phase;
    $("operation-badge").className = "badge amber";
    const selected = shared.model ? ` · ${shared.model.toUpperCase()}` : "";
    $("operation-content").innerHTML = `<strong>${escapeHtml(shared.actor)}의 요청${escapeHtml(selected)}</strong><p>다른 화면에도 동일하게 반영됩니다. 완료될 때까지 기다려주세요.</p><div class="operation-progress"><i></i></div>`;
    $("cancel-operation").classList.add("hidden");
  } else if (op.running) {
    $("operation-badge").textContent = "RUNNING";
    $("operation-badge").className = "badge amber";
    $("operation-content").innerHTML = `<strong>${escapeHtml(op.kind)}</strong><div class="operation-progress"><i></i></div><div class="operation-log">${escapeHtml(op.logs.slice(-10).join("\n") || "작업 출력을 기다리는 중...")}</div>`;
    $("cancel-operation").classList.remove("hidden");
  } else {
    const last = op.last_result;
    $("operation-badge").textContent = last ? (last.ok ? "DONE" : "FAILED") : "IDLE";
    $("operation-badge").className = `badge ${last ? (last.ok ? "good" : "bad") : "neutral"}`;
    $("operation-content").innerHTML = last ? `<strong>${escapeHtml(last.kind)}</strong><p>${last.ok ? "정상 완료" : `실패 · exit ${last.exit_code}`} · ${escapeHtml(last.finished_at)}</p>` : `<div class="empty-state">실행 중인 일회성 작업이 없습니다.</div>`;
    $("cancel-operation").classList.add("hidden");
  }
}

function renderCritical() {
  const critical = state.safety.critical || state.mode?.conflict;
  document.body.classList.toggle("critical", critical);
  $("critical-banner").classList.toggle("hidden", !critical);
  const reasons = [...state.safety.critical_reasons];
  if (state.mode?.conflict) reasons.push(state.mode.detail);
  $("critical-message").textContent = critical ? reasons.join(" · ") : "";
}

function renderButtons() {
  const fm = state.models.followers[$("follower-model").value];
  const restartHelp = state.safety.follower_runtime_active
    ? [fm?.bring_down
      ? "다시 시작하려면 Bring down 후 Follower control을 종료해 주세요."
      : "다시 시작하려면 Follower control을 종료해 주세요."]
    : [];
  gateButton("follower-start", "follower_start", true, "", restartHelp);
  gateButton("follower-stop", "follower_stop");
  const followerReadyForLeader = state.safety.follower_runtime_active
    && state.safety.controllers_ready
    && state.safety.feedback_fresh
    && state.safety.dxl_ok;
  gateButton("leader-start", "leader_start", followerReadyForLeader);
  gateButton("leader-stop", "leader_stop");
  gateButton("bring-down", "bring_down", !!fm?.bring_down, "선택한 Robot은 Bring down을 지원하지 않습니다.");
  [
    ["recover-init", "workspace-confirm"],
    ["recover-init-teleop", "workspace-confirm-teleop"],
    ["recover-init-inference", "workspace-confirm-inference"],
  ].forEach(([button, checkbox]) => {
    const supported = !!fm?.recover_init;
    const confirmed = $(checkbox).checked;
    const reason = supported ? "먼저 경로에 사람과 물체가 없는지 확인해 주세요." : "선택한 Robot은 Init pose 복구를 지원하지 않습니다.";
    gateButton(button, "recover_init", supported && confirmed, reason);
  });
}

function render() {
  if (!state) return;
  fillModels();
  renderMode();
  renderCritical();
  renderSafety();
  renderDiagnostics();
  renderOperation();
  const followerUp = state.safety.follower_runtime_active;
  const leaderUp = state.safety.leader_runtime_active;
  setBadge($("follower-badge"), followerUp);
  setBadge($("leader-badge"), leaderUp);
  const followerOwner = followerUp && !state.services.follower.managed ? "EXTERNAL" : state.services.follower.state.toUpperCase();
  const leaderOwner = leaderUp && !state.services.leader.managed ? "EXTERNAL" : leaderUp ? "ON" : "OFF";
  statusText($("follower-status"), `${state.models.followers[state.selected.follower]?.label || state.selected.follower} · ${followerOwner}`, followerUp ? "good-text" : "");
  statusText($("leader-status"), state.safety.teleop_active ? "TELEOP ACTIVE" : leaderUp ? `${leaderOwner} · COMMAND QUIET` : "OFF", state.safety.teleop_active ? "bad-text" : leaderUp ? "warn-text" : "");
  statusText($("inference-status"), state.ros.inference.present ? `${state.ros.inference.phase} · CONNECTED` : "DISCONNECTED", state.ros.inference.active ? "bad-text" : state.ros.inference.present ? "warn-text" : "good-text");
  statusText($("hardware-status"), state.safety.dxl_ok && state.safety.feedback_fresh ? "HEALTHY" : followerUp ? "CHECK REQUIRED" : "OFFLINE", state.safety.dxl_ok ? "good-text" : followerUp ? "bad-text" : "");
  const transition = state.control?.current;
  if (transition?.action.startsWith("follower_")) {
    $("follower-badge").textContent = transition.phase;
    $("follower-badge").className = "badge amber";
  }
  if (transition?.action.startsWith("leader_")) {
    $("leader-badge").textContent = transition.phase;
    $("leader-badge").className = "badge amber";
  }
  const es = state.ros.estop;
  const estop = $("estop-status");
  estop.textContent = es.active ? "E-STOP ACTIVE" : !es.configured ? "E-stop 미설정" : es.signal_seen ? "E-stop 해제" : "E-stop 대기";
  estop.className = `estop-chip ${es.active ? "bad-text" : !es.configured ? "warn-text" : "good-text"}`;
  $("token-button").classList.toggle("hidden", !state.safety.token_configured);
  renderButtons();
}

async function callAction(action, model="") {
  const headers = {"Content-Type":"application/json", "X-Control-Client":clientId};
  const token = sessionStorage.getItem(tokenKey);
  if (state?.safety.token_configured && token) headers["X-Control-Token"] = token;
  const response = await fetch(`/api/actions/${action}`, {method:"POST", headers, body:JSON.stringify({model})});
  let payload = {};
  try { payload = await response.json(); } catch (_) {}
  if (!response.ok) {
    const detail = payload.detail;
    const message = typeof detail === "string" ? detail : detail?.failures?.join(" · ") || detail?.message || `요청 실패 (${response.status})`;
    throw new Error(message);
  }
  if (payload.warnings?.length) showToast(payload.warnings.join(" · ")); else showToast("요청이 전달되었습니다.");
}

function openAction(action) {
  const meta = actionMeta[action];
  const gate = state.gates[action];
  pendingAction = action;
  $("modal-title").textContent = meta.title;
  $("modal-description").textContent = meta.description;
  $("modal-warnings").innerHTML = (gate.warnings || []).map((item) => `<div class="modal-warning">${escapeHtml(item)}</div>`).join("");
  $("modal-confirm").textContent = "실행";
  $("modal").classList.remove("hidden");
  $("modal-confirm").focus();
}

function showBlockedNotice(buttonId) {
  const button = $(buttonId);
  if (button.getAttribute("aria-disabled") !== "true") return false;
  let reasons = [];
  try { reasons = JSON.parse(button.dataset.blockedReasons || "[]"); } catch (_) {}
  $("blocked-reasons").innerHTML = (reasons.length ? reasons : ["현재 안전 조건을 확인할 수 없습니다."])
    .map((reason) => `<p>${escapeHtml(reason)}</p>`).join("");
  $("blocked-modal").classList.remove("hidden");
  $("blocked-ok").focus();
  return true;
}

function openActionIfAvailable(buttonId, action) {
  if (!showBlockedNotice(buttonId)) openAction(action);
}

function closeModal() { $("modal").classList.add("hidden"); pendingAction = null; }

$("follower-start").onclick = () => openActionIfAvailable("follower-start", "follower_start");
$("leader-start").onclick = () => openActionIfAvailable("leader-start", "leader_start");
$("bring-down").onclick = () => openActionIfAvailable("bring-down", "bring_down");
["recover-init", "recover-init-teleop", "recover-init-inference"].forEach((id) => { $(id).onclick = () => openActionIfAvailable(id, "recover_init"); });
$("follower-stop").onclick = async () => { if (showBlockedNotice("follower-stop")) return; if (!confirm("Follower control stack을 종료할까요? F2는 먼저 Bring-down하는 것을 권장합니다.")) return; try { await callAction("follower_stop"); } catch(e){showToast(e.message,true);} };
$("leader-stop").onclick = async () => { if (showBlockedNotice("leader-stop")) return; try { await callAction("leader_stop"); } catch(e){showToast(e.message,true);} };
$("cancel-operation").onclick = async () => { try { await callAction("operation_cancel"); } catch(e){showToast(e.message,true);} };
$("modal-confirm").onclick = async () => {
  if (!pendingAction) return;
  const action = pendingAction;
  const model = action === "follower_start" || action === "recover_init" ? $("follower-model").value : action === "leader_start" ? $("leader-model").value : "";
  try { await callAction(action, model); closeModal(); } catch(e){showToast(e.message,true);}
};
$("modal-close").onclick = $("modal-cancel").onclick = closeModal;
$("blocked-close").onclick = $("blocked-ok").onclick = () => $("blocked-modal").classList.add("hidden");
["workspace-confirm", "workspace-confirm-teleop", "workspace-confirm-inference"].forEach((id) => { $(id).onchange = render; });
$("follower-model").onchange = render;
$("leader-model").onchange = render;

$("token-button").onclick = () => { $("token-input").value = sessionStorage.getItem(tokenKey) || ""; $("token-modal").classList.remove("hidden"); };
$("token-close").onclick = () => $("token-modal").classList.add("hidden");
$("token-save").onclick = () => { sessionStorage.setItem(tokenKey, $("token-input").value); $("token-modal").classList.add("hidden"); showToast("제어 token을 현재 탭에 저장했습니다."); };

async function loadLogs() {
  const source = $("log-source").value;
  $("raw-logs").textContent = "로그를 불러오는 중...";
  try {
    const response = await fetch(`/api/logs?source=${source}`);
    const payload = await response.json();
    $("raw-logs").textContent = payload.lines.join("\n") || "로그가 없습니다.";
    $("raw-logs").scrollTop = $("raw-logs").scrollHeight;
  } catch(e) { $("raw-logs").textContent = e.message; }
}
$("open-logs").onclick = () => { $("log-drawer").classList.remove("hidden"); loadLogs(); };
$("logs-close").onclick = () => $("log-drawer").classList.add("hidden");
$("logs-refresh").onclick = loadLogs;
$("log-source").onchange = loadLogs;

function connectionState(ok) {
  $("connection").className = `status-pill ${ok ? "good" : "bad"}`;
  $("connection").innerHTML = `<i></i> ${ok ? "ROS LIVE" : "재연결 중"}`;
  $("side-connection").className = `connection-dot ${ok ? "good" : "bad"}`;
}

function connect() {
  const source = new EventSource("/api/events");
  source.onopen = () => connectionState(true);
  source.onmessage = (event) => { state = JSON.parse(event.data); render(); };
  source.onerror = () => connectionState(false);
}

fetch("/api/state").then((response) => response.json()).then((data) => { state = data; render(); }).catch(() => connectionState(false));
connect();
