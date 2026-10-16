const palette = ['#1264df', '#0a9b73', '#df7d12', '#8255d9', '#d63f69', '#18a1b8', '#606c80'];
const state = {
  activeArm: 'left',
  availableArms: ['left', 'right'],
  armData: { left: null, right: null },
  data: null,
  targets: { left: [], right: [] },
  radianDrafts: { left: '', right: '' },
  nextDwells: { left: 0, right: 0 },
  nextSpeeds: { left: 1, right: 1 },
  dwellDrafts: { left: {}, right: {} },
  speedDrafts: { left: {}, right: {} },
  equalTimes: { left: { mode: 'total', seconds: '', excluded: null, result: null }, right: { mode: 'total', seconds: '', excluded: null, result: null } },
  equalTimeBusy: false,
  refinementDrafts: { left: null, right: null },
  refinementApplied: { left: null, right: null },
  refinementBusy: false,
  waypointListCollapsed: false,
  relativeReferences: { left: 'last', right: 'last' },
  frameRelativeReferences: { left: 'last', right: 'last' },
  selectedWaypoints: { left: null, right: null },
  renderedWaypointListKey: '',
  draggedWaypoint: null,
  lastTrajectoryKey: { left: '', right: '' },
  renderedTrajectoryArm: null,
  busy: false,
  refreshRequestId: 0,
  runtimeRefreshInFlight: false,
  planRefreshPending: false,
  keyboard: {
    activeDirection: null,
    inFlight: false,
    heartbeatInFlight: false,
    leaseActive: false,
    heartbeatWorker: null,
    timer: null,
  },
  simulator: {
    models: { left: null, right: null },
    fullViewer: null,
    fullViewerLoading: false,
    yaw: -0.72,
    pitch: 0.56,
    zoom: 1,
    playhead: 0,
    playing: false,
    playbackArms: [],
    previewPoseActive: false,
    lastFrame: 0,
    drag: null,
    poseDrafts: { left: null, right: null },
    poseEditorMode: 'translate',
  },
};

const byId = (id) => document.getElementById(id);

const KEYBOARD_HEARTBEAT_INTERVAL_MS = 100;
const KEYBOARD_HEARTBEAT_REQUEST_TIMEOUT_MS = 400;
const STOP_BUTTON_CONFIRMATION = 'STOP_BUTTON_CLICK';
const STOP_POINTER_CLICK_WINDOW_MS = 2000;

const keyboardHeartbeatWorkerSource = `
let leaseActive = false;
let heartbeatInFlight = false;
let heartbeatUrl = '';

async function sendHeartbeat() {
  if (!leaseActive || heartbeatInFlight || !heartbeatUrl) return;
  heartbeatInFlight = true;
  const controller = new AbortController();
  const timeout = setTimeout(
    () => controller.abort(), ${KEYBOARD_HEARTBEAT_REQUEST_TIMEOUT_MS},
  );
  try {
    const response = await fetch(heartbeatUrl, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: '{}',
      signal: controller.signal,
    });
    if (!response.ok) throw new Error('HTTP ' + response.status);
    postMessage({ type: 'heartbeat', ok: true });
  } catch (_error) {
    postMessage({ type: 'heartbeat', ok: false });
  } finally {
    clearTimeout(timeout);
    heartbeatInFlight = false;
  }
}

onmessage = (event) => {
  if (event.data?.type === 'start') {
    heartbeatUrl = event.data.url || '';
    leaseActive = true;
    sendHeartbeat();
  } else if (event.data?.type === 'stop') {
    leaseActive = false;
  }
};

setInterval(sendHeartbeat, ${KEYBOARD_HEARTBEAT_INTERVAL_MS});
`;

function escapeHtml(value) {
  return String(value)
    .replaceAll('&', '&amp;').replaceAll('<', '&lt;').replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;').replaceAll("'", '&#039;');
}

function showToast(message, error = false) {
  const toast = byId('toast');
  toast.textContent = message;
  toast.className = `toast show${error ? ' error' : ''}`;
  window.clearTimeout(showToast.timer);
  showToast.timer = window.setTimeout(() => { toast.className = 'toast'; }, 2800);
}

async function request(path, options = {}) {
  const response = await fetch(path, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  });
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || payload.message || `HTTP ${response.status}`);
  return payload;
}

async function copyText(text) {
  if (navigator.clipboard && window.isSecureContext) {
    await navigator.clipboard.writeText(text);
    return;
  }
  const textarea = document.createElement('textarea');
  textarea.value = text;
  textarea.setAttribute('readonly', '');
  textarea.style.position = 'fixed';
  textarea.style.opacity = '0';
  document.body.appendChild(textarea);
  textarea.select();
  const copied = document.execCommand('copy');
  textarea.remove();
  if (!copied) throw new Error('브라우저가 클립보드 복사를 허용하지 않았습니다.');
}

function shortJoint(name) {
  if (name.startsWith('gripper_')) return 'Gripper';
  const match = name.match(/joint(\d+)$/);
  return match ? `Joint ${match[1]}` : name;
}

function waypointPlanMode(data) {
  if (data.path_refinement?.enabled) return 'JOINT_REFINED';
  const modes = new Set((data.waypoints || []).map(
    (waypoint) => waypoint.interpolation_mode || data.path_mode,
  ));
  const mode = modes.size === 0
    ? data.path_mode
    : modes.size === 1 ? [...modes][0] : 'MIXED';
  return mode === 'JOINT' && data.joint_smoothing_enabled
    ? 'JOINT_CONTINUOUS'
    : mode;
}

function parseRadianJointList(raw, count) {
  let values;
  try {
    values = JSON.parse(raw);
  } catch (_error) {
    throw new Error('radian 관절값을 [값1, 값2, ...] 형식으로 입력해 주세요.');
  }
  if (!Array.isArray(values) || values.length !== count) {
    throw new Error(`radian 관절값 ${count}개를 모두 입력해 주세요.`);
  }
  if (values.some((value) => typeof value !== 'number' || !Number.isFinite(value))) {
    throw new Error('radian 관절값은 모두 유한한 숫자여야 합니다.');
  }
  return values;
}

function identityRotation() {
  return [1, 0, 0, 0, 1, 0, 0, 0, 1];
}

function multiplyRotation(left, right) {
  const result = Array(9).fill(0);
  for (let row = 0; row < 3; row += 1) {
    for (let column = 0; column < 3; column += 1) {
      for (let index = 0; index < 3; index += 1) {
        result[row * 3 + column] += left[row * 3 + index] * right[index * 3 + column];
      }
    }
  }
  return result;
}

function rotateVector(rotation, vector) {
  return [
    rotation[0] * vector[0] + rotation[1] * vector[1] + rotation[2] * vector[2],
    rotation[3] * vector[0] + rotation[4] * vector[1] + rotation[5] * vector[2],
    rotation[6] * vector[0] + rotation[7] * vector[1] + rotation[8] * vector[2],
  ];
}

function composePose(left, right) {
  const translated = rotateVector(left.rotation, right.position);
  return {
    rotation: multiplyRotation(left.rotation, right.rotation),
    position: left.position.map((value, index) => value + translated[index]),
  };
}

function rpyRotation([roll, pitch, yaw]) {
  const cr = Math.cos(roll); const sr = Math.sin(roll);
  const cp = Math.cos(pitch); const sp = Math.sin(pitch);
  const cy = Math.cos(yaw); const sy = Math.sin(yaw);
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  ];
}

function rotationToRpy(rotation) {
  const pitch = Math.asin(Math.max(-1, Math.min(1, -rotation[6])));
  const cosinePitch = Math.cos(pitch);
  let roll;
  let yaw;
  if (Math.abs(cosinePitch) > 1e-7) {
    roll = Math.atan2(rotation[7], rotation[8]);
    yaw = Math.atan2(rotation[3], rotation[0]);
  } else {
    roll = 0;
    yaw = Math.atan2(-rotation[1], rotation[4]);
  }
  return [roll, pitch, yaw];
}

function quaternionToRpy([x, y, z, w]) {
  const sinRollCosPitch = 2 * (w * x + y * z);
  const cosRollCosPitch = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinRollCosPitch, cosRollCosPitch);
  const sinPitch = 2 * (w * y - z * x);
  const pitch = Math.abs(sinPitch) >= 1
    ? Math.sign(sinPitch) * Math.PI / 2
    : Math.asin(sinPitch);
  const sinYawCosPitch = 2 * (w * z + x * y);
  const cosYawCosPitch = 1 - 2 * (y * y + z * z);
  return [roll, pitch, Math.atan2(sinYawCosPitch, cosYawCosPitch)];
}

function rpyToQuaternion([roll, pitch, yaw]) {
  const cr = Math.cos(roll / 2); const sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2); const sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2); const sy = Math.sin(yaw / 2);
  return [
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  ];
}

function axisAngleRotation(axis, angle) {
  const length = Math.hypot(...axis);
  if (length < 1e-9) return identityRotation();
  const [x, y, z] = axis.map((value) => value / length);
  const cosine = Math.cos(angle); const sine = Math.sin(angle); const oneMinus = 1 - cosine;
  return [
    cosine + x * x * oneMinus, x * y * oneMinus - z * sine, x * z * oneMinus + y * sine,
    y * x * oneMinus + z * sine, cosine + y * y * oneMinus, y * z * oneMinus - x * sine,
    z * x * oneMinus - y * sine, z * y * oneMinus + x * sine, cosine + z * z * oneMinus,
  ];
}

function degreesByName(jointNames, values) {
  return Object.fromEntries(jointNames.map((name, index) => [name, Number(values[index] ?? 0)]));
}

function calculateRobotPose(model, positionsDeg) {
  let pose = { rotation: identityRotation(), position: [0, 0, 0] };
  const points = [{ name: model.base_link, type: 'base', position: [...pose.position] }];
  model.chain.forEach((joint) => {
    pose = composePose(pose, {
      rotation: rpyRotation(joint.origin_rpy),
      position: joint.origin_xyz,
    });
    points.push({ name: joint.name, type: joint.type, position: [...pose.position] });
    const value = (positionsDeg[joint.name] || 0) * Math.PI / 180;
    if (joint.type === 'continuous' || joint.type === 'revolute') {
      pose = composePose(pose, {
        rotation: axisAngleRotation(joint.axis, value),
        position: [0, 0, 0],
      });
    } else if (joint.type === 'prismatic') {
      pose = composePose(pose, {
        rotation: identityRotation(),
        position: joint.axis.map((axisValue) => axisValue * value),
      });
    }
  });
  return {
    points,
    endEffector: [...pose.position],
    endEffectorPose: {
      position: [...pose.position],
      rotation: [...pose.rotation],
      rpy: rotationToRpy(pose.rotation),
    },
  };
}

function formatPose(position, rpy) {
  const millimeters = position.map((value) => value * 1000);
  const degrees = rpy.map((value) => value * 180 / Math.PI);
  return `XYZ ${millimeters.map((value) => value.toFixed(1)).join(', ')} mm · RPY ${degrees.map((value) => value.toFixed(1)).join(', ')}°`;
}

function relativeReferenceOptions(data) {
  return [
    `<option value="last">${data.waypoint_count > 0 ? `마지막 waypoint · ${data.waypoint_count}` : '마지막 waypoint · 현재 자세'}</option>`,
    '<option value="current">현재 실제 자세</option>',
    ...data.waypoints.map((waypoint) => `<option value="${waypoint.index}">${escapeHtml(waypoint.name || `Waypoint ${waypoint.index}`)}</option>`),
  ].join('');
}

function selectedWorldReferencePose(data, selectedReference) {
  if (selectedReference === 'current' || data.waypoint_count === 0) {
    return data.current_world_pose || null;
  }
  const index = selectedReference === 'last'
    ? data.waypoint_count - 1
    : Number(selectedReference) - 1;
  return data.waypoints[index]?.world_pose || null;
}

const POSE_EDITOR_INPUT_IDS = [
  'poseEditorX', 'poseEditorY', 'poseEditorZ',
  'poseEditorRoll', 'poseEditorPitch', 'poseEditorYaw',
];

function poseDraftFromWorldPose(pose) {
  const position = pose?.position_m;
  const orientation = pose?.orientation_xyzw;
  if (!Array.isArray(position) || position.length !== 3
    || !Array.isArray(orientation) || orientation.length !== 4) return null;
  const values = [...position, ...orientation].map(Number);
  if (values.some((value) => !Number.isFinite(value))) return null;
  return {
    position_m: values.slice(0, 3),
    orientation_deg: quaternionToRpy(values.slice(3)).map(
      (value) => value * 180 / Math.PI,
    ),
  };
}

function poseDraftAsViewerPose(draft) {
  if (!draft) return null;
  return {
    position_m: [...draft.position_m],
    orientation_xyzw: rpyToQuaternion(
      draft.orientation_deg.map((value) => Number(value) * Math.PI / 180),
    ),
  };
}

function ensurePoseDraft(data) {
  if (!state.simulator.poseDrafts[data.arm]) {
    state.simulator.poseDrafts[data.arm] = poseDraftFromWorldPose(data.current_world_pose);
  }
  return state.simulator.poseDrafts[data.arm];
}

function handlePoseGizmoChange(pose) {
  const draft = poseDraftFromWorldPose(pose);
  if (!draft) return;
  state.simulator.poseDrafts[state.activeArm] = draft;
  renderPoseEditor(state.data);
}

function syncPoseEditorViewer(data) {
  const viewer = state.simulator.fullViewer;
  const draft = ensurePoseDraft(data);
  if (!viewer) return;
  viewer.setPoseEditor({
    pose: poseDraftAsViewerPose(draft),
    mode: state.simulator.poseEditorMode,
    visible: Boolean(draft) && !data.execution?.active,
    onChange: handlePoseGizmoChange,
  });
}

function renderPoseEditor(data) {
  const draft = ensurePoseDraft(data);
  const armLabel = data.arm === 'left' ? '왼팔' : '오른팔';
  byId('poseEditorTitle').textContent = `${armLabel} 목표 자세`;
  byId('poseEditorFrame').textContent = data.world_frame || 'world';
  const values = draft
    ? [...draft.position_m.map((value) => value * 1000), ...draft.orientation_deg]
    : [null, null, null, null, null, null];
  POSE_EDITOR_INPUT_IDS.forEach((id, index) => {
    const input = byId(id);
    input.disabled = !draft || Boolean(data.execution?.active);
    if (document.activeElement === input) return;
    const digits = index < 3 ? 1 : 2;
    input.value = Number.isFinite(values[index]) ? values[index].toFixed(digits) : '';
  });
  document.querySelectorAll('.pose-mode').forEach((button) => {
    button.classList.toggle('active', button.dataset.poseMode === state.simulator.poseEditorMode);
    button.disabled = !draft || Boolean(data.execution?.active);
  });
  byId('poseLoadCurrent').disabled = !poseDraftFromWorldPose(data.current_world_pose)
    || Boolean(data.execution?.active);
  const selectedIndex = state.selectedWaypoints[data.arm];
  const selectedPose = selectedIndex ? data.waypoints[selectedIndex - 1]?.world_pose : null;
  byId('poseLoadSelected').disabled = !poseDraftFromWorldPose(selectedPose)
    || Boolean(data.execution?.active);
  byId('poseAddWaypoint').disabled = !draft || Boolean(data.execution?.active);
  byId('poseGizmoHint').textContent = state.simulator.poseEditorMode === 'rotate'
    ? '3D 회전 링을 드래그해 Roll/Pitch/Yaw를 조정하세요.'
    : '3D 화살표를 드래그해 고정 frame XYZ를 조정하세요.';
  syncPoseEditorViewer(data);
}

function renderPosePanels(data) {
  const model = state.simulator.models[data.arm];
  byId('gripperFrameName').textContent = model?.tip_link || '—';
  byId('absoluteFrameName').textContent = data.world_frame || 'world';
  byId('frameRelativePoseFrame').textContent = data.world_frame || 'world';
  byId('waypointFrameName').textContent = model?.base_link || '기준 링크';
  const referenceSelect = byId('relativeReferenceSelect');
  const selectedReference = state.relativeReferences[data.arm];
  if (/^\d+$/.test(selectedReference) && Number(selectedReference) > data.waypoint_count) {
    state.relativeReferences[data.arm] = 'last';
  }
  referenceSelect.innerHTML = relativeReferenceOptions(data);
  referenceSelect.value = state.relativeReferences[data.arm];

  const frameReferenceSelect = byId('frameRelativeReferenceSelect');
  const frameSelectedReference = state.frameRelativeReferences[data.arm];
  if (/^\d+$/.test(frameSelectedReference)
    && Number(frameSelectedReference) > data.waypoint_count) {
    state.frameRelativeReferences[data.arm] = 'last';
  }
  frameReferenceSelect.innerHTML = relativeReferenceOptions(data);
  frameReferenceSelect.value = state.frameRelativeReferences[data.arm];
  const worldReference = selectedWorldReferencePose(
    data, state.frameRelativeReferences[data.arm],
  );
  byId('frameRelativePose').textContent = worldReference
    ? formatPose(
      worldReference.position_m,
      quaternionToRpy(worldReference.orientation_xyzw),
    )
    : `${data.world_frame || 'world'} 절대 pose 대기 중`;

  if (model?.status === 'ready' && data.current_deg.every((value) => value !== null)) {
    const pose = calculateRobotPose(
      model,
      degreesByName(data.joint_names, data.current_deg),
    ).endEffectorPose;
    byId('gripperPose').textContent = formatPose(pose.position, pose.rpy);
  } else {
    byId('gripperPose').textContent = 'URDF와 joint state 대기 중';
  }

  const list = byId('waypointList');
  const listRenderKey = JSON.stringify({
    arm: data.arm,
    pathMode: data.path_mode,
    executing: Boolean(data.execution?.active),
    selectedWaypoint: state.selectedWaypoints[data.arm],
    baseLink: model?.base_link || '',
    waypoints: data.waypoints || [],
  });
  // Joint state polling runs every 350 ms. Replacing unchanged list markup on
  // every poll closes an open native <select>, so only rebuild when the list's
  // actual content changes.
  if (state.renderedWaypointListKey === listRenderKey) return;
  state.renderedWaypointListKey = listRenderKey;
  if (!data.waypoints?.length) {
    list.innerHTML = '<div class="waypoint-list-empty">관절 목표, 전역 고정 frame 상대 목표, 그리퍼 상대 목표를 여러 개 추가할 수 있습니다.</div>';
    return;
  }
  list.innerHTML = data.waypoints.map((waypoint) => {
    const displayPose = waypoint.world_pose || waypoint;
    const displayFrame = waypoint.world_pose
      ? (data.world_frame || 'world')
      : (model?.base_link || 'base');
    const positionMm = displayPose.position_m.map((value) => value * 1000);
    const rpyDeg = quaternionToRpy(displayPose.orientation_xyzw)
      .map((value) => value * 180 / Math.PI);
    const draft = state.dwellDrafts[data.arm][waypoint.index];
    const dwell = draft === undefined ? Number(waypoint.dwell_s || 0) : draft;
    const speedDraft = state.speedDrafts[data.arm][waypoint.index];
    const speed = speedDraft === undefined
      ? Number(Number(waypoint.speed_scale ?? 1).toFixed(6)) : speedDraft;
    const lastNote = waypoint.index === data.waypoints.length && dwell > 0
      ? '<small>다음 waypoint가 추가되면 이 대기 시간이 적용됩니다.</small>'
      : '';
    const gripper = waypoint.gripper_rad === null || waypoint.gripper_rad === undefined
      ? '—'
      : `${Number(waypoint.gripper_rad).toFixed(4)} rad (${Number(waypoint.gripper_deg).toFixed(2)}°)`;
    const interpolationMode = waypoint.interpolation_mode || data.path_mode || 'LINEAR';
    return `
      <article class="waypoint-item${state.selectedWaypoints[data.arm] === waypoint.index ? ' selected' : ''}" data-index="${waypoint.index}" draggable="${!data.execution?.active}">
        <header><span class="drag-handle" title="드래그해 순서 변경">☰</span><strong>${escapeHtml(waypoint.name || `Waypoint ${waypoint.index}`)}</strong><span>#${waypoint.index} · ${displayFrame} 절대 자세</span></header>
        <p>XYZ · ${positionMm.map((value) => value.toFixed(1)).join(', ')} mm</p>
        <p>RPY · ${rpyDeg.map((value) => value.toFixed(1)).join(', ')}°</p>
        <p>Gripper · ${gripper}</p>
        <div class="waypoint-item-actions">
          <label class="name-label">이름 <input class="name-input" data-index="${waypoint.index}" maxlength="60" value="${escapeHtml(waypoint.name || `Waypoint ${waypoint.index}`)}"></label>
          <button class="button ghost save-name" data-index="${waypoint.index}" ${data.execution?.active ? 'disabled' : ''}>이름 저장</button>
          <label class="mode-label">이 지점까지
            <select class="mode-select" data-index="${waypoint.index}" ${data.execution?.active ? 'disabled' : ''}>
              <option value="LINEAR" ${interpolationMode === 'LINEAR' ? 'selected' : ''}>Cartesian 직선</option>
              <option value="JOINT" ${interpolationMode === 'JOINT' ? 'selected' : ''}>관절 보간</option>
            </select>
          </label>
          <label class="dwell-label">도착 후 대기 <input class="dwell-input" data-index="${waypoint.index}" type="number" min="0" max="${data.max_waypoint_dwell_s}" step="0.1" value="${dwell}"> 초</label>
          <button class="button ghost save-dwell" data-index="${waypoint.index}" ${data.execution?.active ? 'disabled' : ''}>대기 저장</button>
          <label class="speed-label">이 지점까지 속도 배율 <input class="speed-input" data-index="${waypoint.index}" type="number" min="0.01" max="${data.max_waypoint_speed_scale || 3}" step="0.1" value="${speed}"> 배</label>
          <button class="button ghost save-speed" data-index="${waypoint.index}" ${data.execution?.active ? 'disabled' : ''}>속도 저장</button>
          <button class="button danger delete-waypoint" data-index="${waypoint.index}" ${data.execution?.active ? 'disabled' : ''}>이 지점 삭제</button>
        </div>
        ${lastNote}
      </article>
    `;
  }).join('');
}

function renderExecutionSequence(data) {
  const sequence = data.execution_sequence || [];
  const container = byId('executionSequence');
  const sequenceRunDisabled = !data.publish_enabled
    || Boolean(data.execution?.active)
    || Boolean(data.keyboard_control?.active)
    || !byId('executeConsent').checked;
  container.innerHTML = sequence.length ? sequence.map((step) => `
    <div class="sequence-step" data-index="${step.index}">
      <strong>${step.index}</strong>
      <span class="sequence-arm ${step.arm}">${escapeHtml(step.arm_label)}</span>
      <span>${escapeHtml(step.start_name)} <b>→</b> ${escapeHtml(step.end_name)}</span>
      <div>
        <button class="button execute sequence-run" data-index="${step.index}" ${sequenceRunDisabled ? 'disabled' : ''}>이것만 실행</button>
        <button class="button ghost sequence-up" data-index="${step.index}" ${step.index === 1 ? 'disabled' : ''}>↑</button>
        <button class="button ghost sequence-down" data-index="${step.index}" ${step.index === sequence.length ? 'disabled' : ''}>↓</button>
        <button class="button danger sequence-delete" data-index="${step.index}">삭제</button>
      </div>
    </div>`).join('') : '<div class="waypoint-list-empty">실행할 팔과 waypoint 구간을 추가해 주세요.</div>';

  const arm = byId('sequenceArm').value;
  const armData = state.armData[arm];
  const waypoints = armData?.waypoints || [];
  const start = byId('sequenceStart');
  const end = byId('sequenceEnd');
  const previousStart = Number(start.value) || 1;
  const previousEnd = Number(end.value) || waypoints.length;
  const options = waypoints.map((waypoint) => (
    `<option value="${waypoint.index}">${escapeHtml(waypoint.name || `Waypoint ${waypoint.index}`)}</option>`
  )).join('');
  start.innerHTML = options || '<option>Waypoint 없음</option>';
  end.innerHTML = options || '<option>Waypoint 없음</option>';
  start.value = String(Math.min(previousStart, waypoints.length || 1));
  end.value = String(Math.min(previousEnd, waypoints.length || 1));
  const disabled = !waypoints.length || Boolean(data.execution?.active);
  start.disabled = disabled;
  end.disabled = disabled;
  byId('addSequenceStep').disabled = disabled;
  byId('clearSequence').disabled = !sequence.length || Boolean(data.execution?.active);
  byId('executeSequence').disabled = !data.publish_enabled || !sequence.length
    || Boolean(data.execution?.active) || Boolean(data.keyboard_control?.active)
    || !byId('executeConsent').checked;
}

function renderWaypointInspector(data) {
  let index = state.selectedWaypoints[data.arm];
  if (!Number.isInteger(index) || index < 1 || index > data.waypoints.length) {
    state.selectedWaypoints[data.arm] = null;
    index = null;
  }
  const title = byId('waypointInspectorTitle');
  const meta = byId('waypointInspectorMeta');
  const values = byId('waypointJointValues');
  const copyButton = byId('copyWaypointRadians');
  if (index === null) {
    title.textContent = 'Waypoint를 선택하세요';
    meta.textContent = '목록의 waypoint를 클릭하면 해당 자세와 8축 radian 변화량을 표시합니다.';
    values.innerHTML = '';
    copyButton.disabled = true;
    return;
  }
  const waypoint = data.waypoints[index - 1];
  title.textContent = waypoint.name || `Waypoint ${index}`;
  if (!waypoint.joint_rad) {
    meta.textContent = '정확한 관절값을 계산하는 중입니다. 경로 계산이 완료되면 표시됩니다.';
    values.innerHTML = '';
    copyButton.disabled = true;
    return;
  }
  copyButton.disabled = false;
  const reference = index > 1
    ? data.waypoints[index - 2]?.joint_rad
    : data.current_deg.map((value) => value === null ? null : value * Math.PI / 180);
  const referenceLabel = index > 1 ? `Waypoint ${index - 1}` : '현재 실제 자세';
  const trajectoryNote = waypoint.trajectory_time_s === null
    || waypoint.trajectory_time_s === undefined
    ? '현재 시작 범위에서 제외됨'
    : `trajectory ${Number(waypoint.trajectory_time_s).toFixed(3)} s 지점`;
  meta.textContent = `${referenceLabel} 대비 변화량 · ${waypoint.interpolation_mode || data.path_mode} · ${trajectoryNote}`;
  values.innerHTML = data.joint_names.map((name, jointIndex) => {
    const target = Number(waypoint.joint_rad[jointIndex]);
    const origin = reference?.[jointIndex];
    const delta = origin === null || origin === undefined ? null : target - Number(origin);
    return `
      <div class="waypoint-joint-value">
        <span>${shortJoint(name)}</span><strong>${target.toFixed(6)} rad</strong>
        <small class="${delta !== null && Math.abs(delta) >= 1e-6 ? 'changed' : ''}">${delta === null ? '변화량 —' : `Δ ${delta >= 0 ? '+' : ''}${delta.toFixed(6)} rad`}</small>
      </div>
    `;
  }).join('');
}

function trajectorySampleAt(trajectory, time) {
  if (!trajectory || !trajectory.samples.length) return null;
  if (time <= trajectory.samples[0].time_s) return trajectory.samples[0];
  const last = trajectory.samples.at(-1);
  if (time >= last.time_s) return last;
  let upperIndex = trajectory.samples.findIndex((sample) => sample.time_s >= time);
  if (upperIndex <= 0) upperIndex = 1;
  const lower = trajectory.samples[upperIndex - 1];
  const upper = trajectory.samples[upperIndex];
  const span = Math.max(upper.time_s - lower.time_s, 1e-9);
  const progress = (time - lower.time_s) / span;
  return {
    time_s: time,
    positions_deg: lower.positions_deg.map(
      (value, index) => value + (upper.positions_deg[index] - value) * progress,
    ),
  };
}

function viewPoint(point) {
  const { yaw, pitch } = state.simulator;
  const cosineYaw = Math.cos(yaw); const sineYaw = Math.sin(yaw);
  const x = cosineYaw * point[0] - sineYaw * point[1];
  const y = sineYaw * point[0] + cosineYaw * point[1];
  const cosinePitch = Math.cos(pitch); const sinePitch = Math.sin(pitch);
  return [x, cosinePitch * point[2] - sinePitch * y, sinePitch * point[2] + cosinePitch * y];
}

function makeProjector(points, width, height) {
  const viewed = points.map(viewPoint);
  const xs = viewed.map((point) => point[0]);
  const ys = viewed.map((point) => point[1]);
  let minimumX = Math.min(...xs); let maximumX = Math.max(...xs);
  let minimumY = Math.min(...ys); let maximumY = Math.max(...ys);
  if (maximumX - minimumX < 0.35) { minimumX -= 0.175; maximumX += 0.175; }
  if (maximumY - minimumY < 0.35) { minimumY -= 0.175; maximumY += 0.175; }
  const scale = Math.min(
    (width - 100) / (maximumX - minimumX),
    (height - 76) / (maximumY - minimumY),
  ) * state.simulator.zoom;
  const centerX = (minimumX + maximumX) / 2;
  const centerY = (minimumY + maximumY) / 2;
  return (point) => {
    const viewedPoint = viewPoint(point);
    return {
      x: width / 2 + (viewedPoint[0] - centerX) * scale,
      y: height / 2 - (viewedPoint[1] - centerY) * scale,
      depth: viewedPoint[2],
    };
  };
}

function simulatorPositions() {
  const trajectory = state.simulator.previewPoseActive
    ? state.data?.trajectory : null;
  const sample = trajectorySampleAt(trajectory, state.simulator.playhead);
  if (sample) return degreesByName(trajectory.joint_names, sample.positions_deg);
  if (state.data) return degreesByName(state.data.joint_names, state.data.current_deg);
  return {};
}

function endEffectorTrail(model, trajectory) {
  if (!trajectory) return [];
  return trajectory.samples.map((sample) => calculateRobotPose(
    model,
    degreesByName(trajectory.joint_names, sample.positions_deg),
  ).endEffector);
}

function drawGround(context, project, extent = 0.45) {
  context.save();
  context.strokeStyle = 'rgb(154 177 208 / 16%)';
  context.lineWidth = 1;
  for (let index = -4; index <= 4; index += 1) {
    const value = extent * index / 4;
    [[[-extent, value, 0], [extent, value, 0]], [[value, -extent, 0], [value, extent, 0]]]
      .forEach(([start, end]) => {
        const from = project(start); const to = project(end);
        context.beginPath(); context.moveTo(from.x, from.y); context.lineTo(to.x, to.y); context.stroke();
      });
  }
  context.restore();
}

function drawAxes(context, project, length = 0.13) {
  const origin = project([0, 0, 0]);
  [
    [[length, 0, 0], '#ff6673', 'X'],
    [[0, length, 0], '#4fd39e', 'Y'],
    [[0, 0, length], '#65a9ff', 'Z'],
  ].forEach(([endpoint, color, label]) => {
    const projected = project(endpoint);
    context.strokeStyle = color; context.fillStyle = color; context.lineWidth = 2;
    context.beginPath(); context.moveTo(origin.x, origin.y); context.lineTo(projected.x, projected.y); context.stroke();
    context.fillText(label, projected.x + 4, projected.y + 3);
  });
}

function drawSimulation() {
  const canvas = byId('robotSimulation');
  const model = state.simulator.models[state.activeArm];
  if (!canvas || !model || model.status !== 'ready') return;
  if (state.simulator.fullViewer) {
    const frameState = state.armData[state.activeArm] || state.data;
    state.simulator.fullViewer.setWorldTransform(
      frameState?.world_frame || 'world',
      frameState?.world_root_pose || null,
    );
    if (frameState) syncPoseEditorViewer(frameState);
    const jointState = state.data?.robot_joint_state || { name: [], position: [] };
    const liveValues = Object.fromEntries(
      jointState.name.map((name, index) => [name, Number(jointState.position[index] || 0)]),
    );
    const trajectories = Object.fromEntries(
      state.availableArms.map((arm) => [arm, state.armData[arm]?.trajectory || null]),
    );
    const waypointSets = Object.fromEntries(
      state.availableArms.map((arm) => [arm, {
        waypoints: state.armData[arm]?.waypoints || [],
        baseLink: state.simulator.models[arm]?.base_link || model.base_link,
      }]),
    );
    state.simulator.fullViewer.setTrajectories(trajectories, liveValues);
    state.simulator.fullViewer.setWaypointSets(waypointSets);
    (state.simulator.previewPoseActive ? state.simulator.playbackArms : []).forEach((arm) => {
      const trajectory = trajectories[arm];
      const sample = trajectorySampleAt(trajectory, state.simulator.playhead);
      if (!sample) return;
      trajectory.joint_names.forEach((name, index) => {
        const radians = Number(sample.positions_deg[index]) * Math.PI / 180;
        if (Number.isFinite(radians)) liveValues[name] = radians;
      });
    });
    state.simulator.fullViewer.setJointValues(liveValues);
    return;
  }
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (!width || !height) return;
  canvas.width = Math.round(width * ratio);
  canvas.height = Math.round(height * ratio);
  const context = canvas.getContext('2d');
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);

  const robotPose = calculateRobotPose(model, simulatorPositions());
  const trail = endEffectorTrail(model, state.data?.trajectory);
  const allPoints = robotPose.points.map((point) => point.position).concat(trail, [[0, 0, 0]]);
  const project = makeProjector(allPoints, width, height);
  const planarExtent = Math.max(0.35, ...allPoints.map((point) => Math.hypot(point[0], point[1])));
  drawGround(context, project, planarExtent * 1.2);
  context.font = '10px system-ui';
  drawAxes(context, project);

  if (trail.length > 1) {
    context.save();
    context.strokeStyle = '#34d5c5'; context.lineWidth = 2; context.setLineDash([5, 5]);
    context.beginPath();
    trail.forEach((point, index) => {
      const projected = project(point);
      if (index === 0) context.moveTo(projected.x, projected.y); else context.lineTo(projected.x, projected.y);
    });
    context.stroke(); context.restore();
  }

  const projectedPoints = robotPose.points.map((point) => ({ ...point, screen: project(point.position) }));
  for (let index = 1; index < projectedPoints.length; index += 1) {
    const start = projectedPoints[index - 1].screen;
    const end = projectedPoints[index].screen;
    context.lineCap = 'round';
    context.strokeStyle = 'rgb(4 10 19 / 65%)'; context.lineWidth = 12;
    context.beginPath(); context.moveTo(start.x, start.y); context.lineTo(end.x, end.y); context.stroke();
    context.strokeStyle = index % 2 ? '#e7eef8' : '#9db6d5'; context.lineWidth = 7;
    context.beginPath(); context.moveTo(start.x, start.y); context.lineTo(end.x, end.y); context.stroke();
  }
  projectedPoints.forEach((point, index) => {
    const radius = index === 0 ? 7 : 5;
    context.fillStyle = index === 0 ? '#6ea8f5' : '#24364f';
    context.strokeStyle = '#dce8f8'; context.lineWidth = 2;
    context.beginPath(); context.arc(point.screen.x, point.screen.y, radius, 0, Math.PI * 2);
    context.fill(); context.stroke();
  });
  const tool = projectedPoints.at(-1).screen;
  context.fillStyle = '#ffbd4a'; context.strokeStyle = '#fff1c8'; context.lineWidth = 2;
  context.beginPath(); context.arc(tool.x, tool.y, 7, 0, Math.PI * 2); context.fill(); context.stroke();
  context.fillStyle = '#f3f7fd'; context.font = '10px system-ui';
  context.fillText(model.tip_link, tool.x + 11, tool.y - 8);
}

function updatePlaybackUi() {
  const duration = playbackDuration();
  const playButton = byId('simulationPlay');
  const timeline = byId('simulationTimeline');
  if (duration <= 0) {
    playButton.disabled = true; timeline.disabled = true; timeline.value = 0;
    byId('simulationTime').textContent = '현재 자세';
    return;
  }
  playButton.disabled = false; timeline.disabled = false;
  playButton.textContent = state.simulator.playing ? '일시정지' : '재생';
  timeline.value = Math.round(1000 * state.simulator.playhead / duration);
  byId('simulationTime').textContent = `${state.simulator.playhead.toFixed(2)} / ${duration.toFixed(2)} s`;
}

function playbackDuration() {
  return Math.max(0, ...state.simulator.playbackArms.map(
    (arm) => Number(state.armData[arm]?.trajectory?.duration_s || 0),
  ));
}

async function loadRobotModel() {
  if (state.simulator.fullViewerLoading) return;
  try {
    const modelResults = await Promise.allSettled(['left', 'right'].map(
      (arm) => request(`/api/robot-model?arm=${arm}`),
    ));
    const models = {};
    modelResults.forEach((result, index) => {
      if (result.status === 'fulfilled') models[['left', 'right'][index]] = result.value;
    });
    const model = models[state.activeArm] || Object.values(models)[0];
    if (!model) throw new Error('팔 모델을 불러오지 못했습니다.');
    const badge = byId('modelBadge');
    if (model.status === 'ready') {
      Object.entries(models).forEach(([arm, armModel]) => {
        state.simulator.models[arm] = armModel;
      });
      if (!window.FullRobotSimulator) throw new Error('전체 로봇 WebGL bundle을 불러오지 못했습니다.');
      if (!state.simulator.fullViewer) {
        state.simulator.fullViewerLoading = true;
        badge.textContent = 'F2 mesh 로딩 중'; badge.className = 'status-chip waiting';
        byId('modelMeta').textContent = `${model.robot_name} 전체 URDF·STL mesh를 불러오는 중입니다.`;
        const viewer = new window.FullRobotSimulator(byId('robotSimulation'));
        const assetOrigin = window.location.origin;
        await viewer.load({
          urdfUrl: '/api/robot-description.urdf',
          assetOrigin,
          tipLinks: Object.fromEntries(Object.entries(models).map(
            ([arm, armModel]) => [arm, armModel.tip_link],
          )),
        });
        state.simulator.fullViewer = viewer;
        state.simulator.fullViewerLoading = false;
      }
      badge.textContent = '전체 F2 준비됨'; badge.className = 'status-chip ready';
      byId('modelMeta').textContent = `${model.robot_name} 전체 모델 · 양팔 live joint state + 좌·우 trajectory`;
      byId('simulationEmpty').style.display = 'none';
      if (state.data) renderPosePanels(state.data);
      drawSimulation();
      return;
    }
    badge.textContent = model.status === 'error' ? 'URDF 오류' : 'URDF 대기 중';
    badge.className = 'status-chip waiting';
    byId('modelMeta').textContent = model.message || '/robot_description 대기 중';
  } catch (error) {
    state.simulator.fullViewerLoading = false;
    byId('modelBadge').textContent = '전체 모델 오류';
    byId('modelBadge').className = 'status-chip waiting';
    byId('modelMeta').textContent = `전체 F2 모델 로드 실패: ${error.message}`;
  }
  window.setTimeout(loadRobotModel, 900);
}

function animateSimulation(timestamp) {
  const duration = playbackDuration();
  if (state.simulator.playing && duration > 0) {
    const delta = state.simulator.lastFrame ? (timestamp - state.simulator.lastFrame) / 1000 : 0;
    state.simulator.playhead = Math.min(duration, state.simulator.playhead + delta);
    if (state.simulator.playhead >= duration) {
      state.simulator.playing = false;
      state.simulator.previewPoseActive = false;
    }
    updatePlaybackUi(); drawSimulation();
  }
  state.simulator.lastFrame = timestamp;
  window.requestAnimationFrame(animateSimulation);
}

function buildJointRows(data) {
  const rows = byId('jointRows');
  if (rows.children.length !== data.joint_names.length) {
    rows.innerHTML = data.joint_names.map((name, index) => `
      <tr>
        <td><span>${shortJoint(name)}</span></td>
        <td><span class="current-value missing" id="current-${index}">—</span></td>
        <td><input class="joint-input" id="target-${index}" type="number" step="0.1" value="0.0" aria-label="${name} 목표값"></td>
        <td><span class="delta-value" id="delta-${index}">—</span></td>
      </tr>
    `).join('');
    document.querySelectorAll('.joint-input').forEach((input, index) => {
      input.addEventListener('input', () => {
        state.targets[state.activeArm][index] = Number(input.value);
        renderJointValues(state.data);
      });
    });
  }
  if (state.targets[data.arm].length !== data.joint_names.length) {
    state.targets[data.arm] = Array(data.joint_names.length).fill(0);
  }
  state.targets[data.arm].forEach((value, index) => {
    byId(`target-${index}`).value = Number(value).toFixed(3);
  });
}

function renderJointValues(data) {
  if (!data) return;
  data.current_deg.forEach((value, index) => {
    const current = byId(`current-${index}`);
    const delta = byId(`delta-${index}`);
    if (value === null) {
      current.textContent = '—';
      current.classList.add('missing');
      delta.textContent = '—';
      delta.classList.remove('changed');
      return;
    }
    current.textContent = `${value.toFixed(2)}°`;
    current.classList.remove('missing');
    const difference = state.targets[data.arm][index] - value;
    delta.textContent = `${difference >= 0 ? '+' : ''}${difference.toFixed(2)}°`;
    delta.classList.toggle('changed', Math.abs(difference) >= 0.01);
  });
}

function setTargets(values) {
  state.targets[state.activeArm] = values.map((value) => Number(value ?? 0));
  state.targets[state.activeArm].forEach((value, index) => {
    byId(`target-${index}`).value = value.toFixed(3);
  });
  renderJointValues(state.data);
}

function renderState(data) {
  state.data = data;
  buildJointRows(data);
  renderJointValues(data);
  const armLabel = data.arm === 'left' ? '왼팔' : '오른팔';
  byId('armBadge').textContent = 'DUAL ARM';
  byId('jointEditorTitle').textContent = `${armLabel} 관절 목표값`;
  byId('frameRelativeTitle').textContent = `${armLabel} ${data.world_frame || 'world'} 고정 frame 기준 상대 이동`;
  byId('relativeTitle').textContent = `${armLabel} 그리퍼 기준 상대 이동`;
  byId('waypointListTitle').textContent = `${armLabel} 다중 waypoint 목록`;
  byId('trajectoryTitle').textContent = `${armLabel} 관절 경로`;
  byId('execute').textContent = `${armLabel}만 실제 실행`;
  byId('executeSlowStart').textContent = `${armLabel} 저속 시작 후 실행`;
  document.querySelectorAll('.arm-tab').forEach((button) => {
    button.classList.toggle('active', button.dataset.arm === data.arm);
  });
  byId('leftWaypointCount').textContent = state.armData.left?.waypoint_count ?? 0;
  byId('rightWaypointCount').textContent = state.armData.right?.waypoint_count ?? 0;
  byId('connectionBadge').textContent = 'ROS 연결됨';
  byId('connectionBadge').className = 'badge online';
  byId('safetyBadge').textContent = data.publish_enabled ? 'POWERED ENABLED' : 'PREVIEW ONLY';
  byId('safetyBadge').className = `badge ${data.publish_enabled ? 'armed' : 'safe'}`;
  byId('waypointCount').textContent = data.waypoint_count;
  byId('nodeStatus').textContent = data.status;
  const failure = (data.execution_events || []).slice().reverse()
    .find((event) => event.status.startsWith('ERROR:'));
  byId('executionFailureRow').hidden = !failure;
  byId('executionFailure').textContent = failure
    ? `${new Date(failure.time_unix_s * 1000).toLocaleTimeString()} · ${failure.status}`
    : '';
  const startSelect = byId('startWaypointSelect');
  const rangeOptions = data.waypoints.map((waypoint) => (
    `<option value="${waypoint.index}">${escapeHtml(waypoint.name || `Waypoint ${waypoint.index}`)}</option>`
  )).join('');
  startSelect.innerHTML = data.waypoints.length
    ? rangeOptions
    : '<option value="1">Waypoint가 없습니다</option>';
  startSelect.value = String(data.start_waypoint || 1);
  startSelect.disabled = data.waypoints.length === 0 || Boolean(data.execution?.active);
  const endSelect = byId('endWaypointSelect');
  endSelect.innerHTML = data.waypoints.length
    ? rangeOptions
    : '<option value="1">Waypoint가 없습니다</option>';
  endSelect.value = String(data.end_waypoint || data.waypoint_count || 1);
  endSelect.disabled = data.waypoints.length === 0 || Boolean(data.execution?.active);
  byId('nextDwell').max = data.max_waypoint_dwell_s;
  byId('nextSpeed').max = data.max_waypoint_speed_scale || 3;
  if (document.activeElement !== byId('nextDwell')) {
    byId('nextDwell').value = state.nextDwells[data.arm];
  }
  if (document.activeElement !== byId('nextSpeed')) {
    byId('nextSpeed').value = state.nextSpeeds[data.arm];
  }
  if (document.activeElement !== byId('radianJointList')) {
    byId('radianJointList').value = state.radianDrafts[data.arm];
  }

  const arbiterMode = data.arbiter?.mode || 'UNKNOWN';
  const arbiterFault = data.arbiter?.fault || '';
  byId('arbiterMode').textContent = arbiterMode;
  byId('arbiterMode').className = arbiterMode === 'NONE'
    ? 'source-none'
    : arbiterMode === 'TELEOP'
      ? 'source-teleop'
      : arbiterMode === 'WAYPOINT'
        ? 'source-waypoint'
        : 'source-faulted';
  byId('arbiterFault').hidden = !arbiterFault;
  byId('arbiterFault').textContent = arbiterFault ? `제어권 오류 · ${arbiterFault}` : '';
  const teleopStandby = Boolean(data.arbiter?.teleop_ready)
    && arbiterMode === 'NONE';
  byId('prepareTeleop').textContent = teleopStandby
    ? '텔레옵 연결됨'
    : '텔레옵 미리 연결';
  byId('prepareTeleop').title = teleopStandby
    ? '상대 TELEOP 양팔 입력이 준비되어 있습니다.'
    : (data.arbiter?.teleop_reason || '상대 TELEOP 입력을 미리 준비합니다.');
  byId('prepareTeleop').disabled = !data.publish_enabled
    || Boolean(data.execution?.active)
    || Boolean(data.keyboard_control?.active)
    || Boolean(arbiterFault)
    || teleopStandby
    || arbiterMode === 'TELEOP'
    || arbiterMode === 'WAYPOINT';
  byId('startTeleop').disabled = !data.publish_enabled
    || Boolean(data.execution?.active)
    || Boolean(arbiterFault)
    || arbiterMode === 'TELEOP';
  const keyboardActive = Boolean(data.keyboard_control?.active)
    && arbiterMode === 'WAYPOINT' && !arbiterFault;
  byId('startKeyboard').disabled = !data.publish_enabled
    || Boolean(data.execution?.active)
    || Boolean(arbiterFault)
    || keyboardActive;
  byId('keyboardControl').classList.toggle('active', keyboardActive);
  byId('keyboardFrame').textContent = data.keyboard_control?.frame || data.world_frame || 'world';
  document.querySelectorAll('.keyboard-speed').forEach((input) => {
    input.max = data.keyboard_control?.max_speed_mmps || 50;
    input.disabled = !keyboardActive;
  });
  document.querySelectorAll('.jog-key').forEach((button) => {
    button.disabled = !keyboardActive || !data.joint_state_fresh;
  });
  byId('keyboardHint').textContent = keyboardActive
    ? `${armLabel} 제어 중 · W/S 앞뒤, A/D 좌우, R/F 위아래 · 키를 놓으면 현재 자세를 유지합니다.`
    : '키보드 WAYPOINT 제어권을 먼저 선택하세요. 입력란에 커서가 있을 때는 단축키가 동작하지 않습니다.';
  if (!keyboardActive && state.keyboard.activeDirection) stopJog();

  const freshness = byId('jointFreshness');
  freshness.textContent = data.joint_state_fresh
    ? `Joint state 정상 · ${data.joint_state_age_s.toFixed(2)} s`
    : 'Joint state가 없거나 오래됐습니다.';
  freshness.className = `status-chip ${data.joint_state_fresh ? 'ready' : 'waiting'}`;

  document.querySelectorAll('.mode-card').forEach((button) => {
    button.classList.toggle('active', button.dataset.mode.toUpperCase() === data.path_mode);
  });
  byId('jointSmoothing').checked = Boolean(data.joint_smoothing_enabled);
  byId('jointSmoothing').disabled = Boolean(data.execution?.active) || Boolean(data.path_refinement?.enabled);
  renderRefinementControls(data);
  const timing = data.joint_timing;
  renderEqualTimeControls(data);
  byId('jointTimingInfo').hidden = !timing || waypointPlanMode(data) !== 'JOINT_CONTINUOUS';
  byId('jointTimingInfo').textContent = timing
    ? `일반 이동 ${(timing.speed_mps * 1000).toFixed(1)} mm/s 기준 · 출발·정지와 관절 제약에 따라 감속 · 삽입 ${timing.preserve_start_waypoint}–${timing.preserve_end_waypoint}번의 기존 시간 유지`
    : '';

  renderPosePanels(data);
  renderPoseEditor(data);
  renderWaypointInspector(data);

  const consent = byId('executeConsent');
  const gate = byId('executionGate');
  if (!data.publish_enabled) {
    gate.classList.add('preview-only');
    gate.innerHTML = 'Preview-only로 실행 중입니다. 실제 trajectory는 전송되지 않습니다.';
    byId('execute').disabled = true;
    byId('executeSlowStart').disabled = true;
  } else {
    gate.classList.remove('preview-only');
    const label = gate.querySelector('label');
    if (label) {
      label.lastChild.textContent = data.execution?.active
        ? ' WAYPOINT trajectory가 실행 중입니다. 정지 시 제어권은 NONE으로 전환됩니다.'
        : ' WAYPOINT 제어권으로 양팔 command가 기록·전송됨을 확인했습니다.';
    }
    byId('execute').disabled = Boolean(data.execution?.active)
      || keyboardActive || !consent || !consent.checked;
    byId('executeSlowStart').disabled = Boolean(data.execution?.active)
      || keyboardActive || !consent || !consent.checked;
  }
  renderExecutionSequence(data);
  renderTrajectory(data.trajectory);
  updatePlaybackUi();
  drawSimulation();
}

function renderTrajectory(trajectory) {
  const arm = state.activeArm;
  const planMode = waypointPlanMode(state.data);
  if (!trajectory) {
    state.lastTrajectoryKey[arm] = '';
    state.renderedTrajectoryArm = arm;
    byId('trajectoryMeta').textContent = `${arm === 'left' ? '왼팔' : '오른팔'} 경로를 계산하면 관절별 보간 결과가 표시됩니다.`;
    byId('chartLegend').innerHTML = '';
    byId('chartEmpty').style.display = 'grid';
    const canvas = byId('trajectoryChart');
    canvas.getContext('2d').clearRect(0, 0, canvas.width, canvas.height);
    return;
  }
  const finalPositions = trajectory.samples.at(-1)?.positions_deg.join(',') || '';
  const key = `${planMode}:${trajectory.sample_count}:${trajectory.duration_s}:${finalPositions}`;
  if (key === state.lastTrajectoryKey[arm] && state.renderedTrajectoryArm === arm) return;
  state.lastTrajectoryKey[arm] = key;
  state.renderedTrajectoryArm = arm;
  byId('trajectoryMeta').textContent = `${trajectory.sample_count} samples · ${trajectory.duration_s.toFixed(2)} s · ${planMode}`;
  byId('chartEmpty').style.display = 'none';
  byId('chartLegend').innerHTML = trajectory.joint_names.map((name, index) => `
    <span><i style="background:${palette[index % palette.length]}"></i>${shortJoint(name)}</span>
  `).join('');
  drawChart(trajectory);
}

function drawChart(trajectory) {
  const canvas = byId('trajectoryChart');
  const ratio = window.devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  canvas.width = width * ratio;
  canvas.height = height * ratio;
  const context = canvas.getContext('2d');
  context.scale(ratio, ratio);
  context.clearRect(0, 0, width, height);

  const padding = { left: 48, right: 16, top: 14, bottom: 30 };
  const plotWidth = width - padding.left - padding.right;
  const plotHeight = height - padding.top - padding.bottom;
  const allValues = trajectory.samples.flatMap((sample) => sample.positions_deg);
  let minimum = Math.min(...allValues);
  let maximum = Math.max(...allValues);
  if (Math.abs(maximum - minimum) < 1) { minimum -= 1; maximum += 1; }
  const margin = (maximum - minimum) * 0.08;
  minimum -= margin;
  maximum += margin;
  const x = (time) => padding.left + (time / Math.max(trajectory.duration_s, 0.001)) * plotWidth;
  const y = (value) => padding.top + (maximum - value) / (maximum - minimum) * plotHeight;

  context.strokeStyle = '#e4e8ed';
  context.fillStyle = '#7a8493';
  context.font = '10px system-ui';
  context.lineWidth = 1;
  for (let index = 0; index <= 4; index += 1) {
    const py = padding.top + (plotHeight * index) / 4;
    const value = maximum - ((maximum - minimum) * index) / 4;
    context.beginPath(); context.moveTo(padding.left, py); context.lineTo(width - padding.right, py); context.stroke();
    context.fillText(`${value.toFixed(1)}°`, 5, py + 3);
  }
  context.fillText('0 s', padding.left, height - 8);
  context.fillText(`${trajectory.duration_s.toFixed(2)} s`, width - padding.right - 38, height - 8);

  trajectory.joint_names.forEach((_, jointIndex) => {
    context.beginPath();
    context.strokeStyle = palette[jointIndex % palette.length];
    context.lineWidth = 2;
    trajectory.samples.forEach((sample, sampleIndex) => {
      const px = x(sample.time_s);
      const py = y(sample.positions_deg[jointIndex]);
      if (sampleIndex === 0) context.moveTo(px, py); else context.lineTo(px, py);
    });
    context.stroke();
  });
}

async function runAction(action, body = {}, arm = state.activeArm) {
  if (state.busy && action !== 'stop') return;
  state.busy = true;
  try {
    const result = await request(`/api/actions/${action}`, {
      method: 'POST', body: JSON.stringify({ ...body, arm }),
    });
    if (action === 'clear' || action === 'remove_last') {
      state.dwellDrafts[arm] = {};
      state.speedDrafts[arm] = {};
    }
    showToast(result.message || '완료했습니다.');
    if (action === 'execute' || action === 'stop') {
      await refreshRuntime(true);
    } else {
      await refresh();
    }
    return result;
  } catch (error) {
    showToast(error.message, true);
    throw error;
  } finally {
    state.busy = false;
  }
}

async function previewAndPlay() {
  const arm = state.activeArm;
  const result = await runAction('preview', {}, arm);
  const trajectory = state.armData[arm]?.trajectory;
  if (!result?.success || !trajectory) return;
  state.simulator.playbackArms = [arm];
  state.simulator.previewPoseActive = true;
  state.simulator.playhead = 0;
  state.simulator.playing = true;
  state.simulator.lastFrame = performance.now();
  updatePlaybackUi();
  drawSimulation();
}

async function previewBothAndPlay() {
  if (state.busy) return;
  state.busy = true;
  try {
    const results = await Promise.allSettled(['left', 'right'].map((arm) =>
      request('/api/actions/preview', {
        method: 'POST', body: JSON.stringify({ arm }),
      })));
    await refresh();
    const failures = results
      .map((result, index) => ({ result, arm: ['왼팔', '오른팔'][index] }))
      .filter(({ result }) => result.status === 'rejected');
    if (failures.length) {
      const details = failures.map(({ arm, result }) => `${arm}: ${result.reason.message}`).join(' · ');
      throw new Error(details);
    }
    state.simulator.playbackArms = ['left', 'right'];
    state.simulator.previewPoseActive = true;
    state.simulator.playhead = 0;
    state.simulator.playing = true;
    state.simulator.lastFrame = performance.now();
    showToast('양팔 경로를 계산해 동시에 재생합니다.');
    updatePlaybackUi();
    drawSimulation();
  } catch (error) {
    showToast(`양팔 경로 계산 실패 · ${error.message}`, true);
  } finally {
    state.busy = false;
  }
}

async function refresh() {
  const requestId = ++state.refreshRequestId;
  const requestedArm = state.activeArm;
  try {
    const primary = await request(`/api/state?arm=${requestedArm}`);
    const availableArms = primary.available_arms || [primary.arm];
    const others = availableArms.filter((arm) => arm !== primary.arm);
    const otherData = await Promise.all(others.map(
      (arm) => request(`/api/state?arm=${arm}`),
    ));
    if (requestId !== state.refreshRequestId) return;
    state.availableArms = availableArms;
    state.armData[primary.arm] = primary;
    otherData.forEach((data) => { state.armData[data.arm] = data; });
    state.planRefreshPending = false;
    const current = state.armData[state.activeArm] || primary;
    setKeyboardLeaseActive(Boolean(current.keyboard_control?.active)
      && current.arbiter?.mode === 'WAYPOINT'
      && !current.arbiter?.fault);
    renderState(current);
  } catch (error) {
    if (requestId !== state.refreshRequestId) return;
    byId('connectionBadge').textContent = '연결 끊김';
    byId('connectionBadge').className = 'badge offline';
  }
}

async function refreshRuntime(force = false) {
  if (!state.data || state.runtimeRefreshInFlight) return;
  state.runtimeRefreshInFlight = true;
  const requestId = ++state.refreshRequestId;
  try {
    const runtimeData = await Promise.all(state.availableArms.map(
      (arm) => request(`/api/runtime?arm=${arm}`),
    ));
    if (state.busy && !force) {
      runtimeData.forEach((data) => {
        if (state.armData[data.arm]) {
          state.armData[data.arm].path_refinement_events = data.path_refinement_events || [];
        }
      });
      renderRefinementMonitor(state.armData[state.activeArm]);
      return;
    }
    if (requestId !== state.refreshRequestId) return;
    const executionActive = runtimeData.some((data) => data.execution?.active);
    const planChanged = runtimeData.some((data) => (
      state.armData[data.arm]
      && state.armData[data.arm].plan_revision !== data.plan_revision
    ));
    if (planChanged && !executionActive) {
      await refresh();
      return;
    }
    if (planChanged) state.planRefreshPending = true;
    runtimeData.forEach((data) => {
      state.armData[data.arm] = { ...state.armData[data.arm], ...data };
    });
    if (state.planRefreshPending && !executionActive) {
      await refresh();
      return;
    }
    const current = state.armData[state.activeArm];
    setKeyboardLeaseActive(Boolean(current.keyboard_control?.active)
      && current.arbiter?.mode === 'WAYPOINT'
      && !current.arbiter?.fault);
    renderState(current);
  } catch (error) {
    if (requestId !== state.refreshRequestId) return;
    // Keep an already-running older UI bridge usable until its safe restart.
    if (error.message === '찾을 수 없습니다.' || error.message === 'HTTP 404') {
      await refresh();
      return;
    }
    byId('connectionBadge').textContent = '연결 끊김';
    byId('connectionBadge').className = 'badge offline';
  } finally {
    state.runtimeRefreshInFlight = false;
  }
}

byId('copyCurrent').addEventListener('click', () => {
  if (!state.data || state.data.current_deg.some((value) => value === null)) {
    showToast('완전한 joint state를 먼저 받아야 합니다.', true); return;
  }
  setTargets(state.data.current_deg);
});
byId('nextDwell').addEventListener('input', (event) => {
  state.nextDwells[state.activeArm] = event.target.value;
});
byId('nextSpeed').addEventListener('input', (event) => {
  state.nextSpeeds[state.activeArm] = event.target.value;
});
byId('radianJointList').addEventListener('input', (event) => {
  state.radianDrafts[state.activeArm] = event.target.value;
});
byId('relativeReferenceSelect').addEventListener('change', (event) => {
  state.relativeReferences[state.activeArm] = event.target.value;
});
byId('frameRelativeReferenceSelect').addEventListener('change', (event) => {
  state.frameRelativeReferences[state.activeArm] = event.target.value;
  if (state.data) renderPosePanels(state.data);
});
byId('resetTarget').addEventListener('click', () => setTargets(
  state.targets[state.activeArm].map(() => 0),
));
byId('addTarget').addEventListener('click', async () => {
  try {
    await request('/api/waypoints/joint', {
      method: 'POST', body: JSON.stringify({
        arm: state.activeArm,
        positions_deg: state.targets[state.activeArm],
        dwell_s: Number(state.nextDwells[state.activeArm]),
        speed_scale: Number(state.nextSpeeds[state.activeArm]),
      }),
    });
    showToast('관절 waypoint를 추가했습니다.'); await refresh();
  } catch (error) { showToast(error.message, true); }
});
byId('addRadianList').addEventListener('click', async () => {
  try {
    const positions = parseRadianJointList(
      state.radianDrafts[state.activeArm],
      state.data?.joint_names.length || 7,
    );
    await request('/api/waypoints/joint', {
      method: 'POST', body: JSON.stringify({
        arm: state.activeArm,
        positions_rad: positions,
        dwell_s: Number(state.nextDwells[state.activeArm]),
        speed_scale: Number(state.nextSpeeds[state.activeArm]),
      }),
    });
    state.radianDrafts[state.activeArm] = '';
    byId('radianJointList').value = '';
    showToast('radian 리스트로 관절 waypoint를 추가했습니다.');
    await refresh();
  } catch (error) { showToast(error.message, true); }
});
byId('addFrameRelative').addEventListener('click', async () => {
  const inputIds = [
    'frameRelativeX', 'frameRelativeY', 'frameRelativeZ',
    'frameRelativeRoll', 'frameRelativePitch', 'frameRelativeYaw',
  ];
  const values = inputIds.map((id) => Number(byId(id).value));
  if (values.some((value) => !Number.isFinite(value))) {
    showToast('전역 고정 frame 상대 이동값 6개를 모두 숫자로 입력해 주세요.', true);
    return;
  }
  try {
    const result = await request('/api/waypoints/frame-relative', {
      method: 'POST',
      body: JSON.stringify({
        arm: state.activeArm,
        reference: state.frameRelativeReferences[state.activeArm],
        translation_mm: values.slice(0, 3),
        rotation_deg: values.slice(3),
        dwell_s: Number(state.nextDwells[state.activeArm]),
        speed_scale: Number(state.nextSpeeds[state.activeArm]),
      }),
    });
    showToast(result.message || '전역 고정 frame 상대 waypoint를 추가했습니다.');
    inputIds.forEach((id) => { byId(id).value = '0'; });
    await refresh();
  } catch (error) {
    showToast(`전역 고정 frame 상대 IK 계산 실패: ${error.message}`, true);
  }
});
byId('addRelative').addEventListener('click', async () => {
  const inputIds = [
    'relativeX', 'relativeY', 'relativeZ',
    'relativeRoll', 'relativePitch', 'relativeYaw',
  ];
  const values = inputIds.map((id) => Number(byId(id).value));
  if (values.some((value) => !Number.isFinite(value))) {
    showToast('상대 이동값 6개를 모두 숫자로 입력해 주세요.', true);
    return;
  }
  try {
    const result = await request('/api/waypoints/relative', {
      method: 'POST',
      body: JSON.stringify({
        arm: state.activeArm,
        reference: state.relativeReferences[state.activeArm],
        translation_mm: values.slice(0, 3),
        rotation_deg: values.slice(3),
        dwell_s: Number(state.nextDwells[state.activeArm]),
        speed_scale: Number(state.nextSpeeds[state.activeArm]),
      }),
    });
    showToast(result.message || '상대 waypoint를 추가했습니다.');
    inputIds.forEach((id) => { byId(id).value = '0'; });
    await refresh();
  } catch (error) {
    showToast(`IK 계산 실패: ${error.message}`, true);
  }
});
byId('captureCurrent').addEventListener('click', () => runAction(
  'capture_current', {
    dwell_s: Number(state.nextDwells[state.activeArm]),
    speed_scale: Number(state.nextSpeeds[state.activeArm]),
  },
).catch(() => {}));
byId('removeLast').addEventListener('click', () => runAction('remove_last').catch(() => {}));
byId('clearAll').addEventListener('click', () => runAction('clear').catch(() => {}));
byId('removeLastList').addEventListener('click', () => runAction('remove_last').catch(() => {}));
byId('clearAllList').addEventListener('click', () => runAction('clear').catch(() => {}));
byId('saveWaypointFile').addEventListener('click', async () => {
  try {
    const data = await request(`/api/waypoints/export?arm=${state.activeArm}`);
    const blob = new Blob([`${JSON.stringify(data, null, 2)}\n`], { type: 'application/json' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = `${state.activeArm}_waypoints.json`;
    link.click();
    URL.revokeObjectURL(link.href);
    showToast(`${state.activeArm === 'left' ? '왼팔' : '오른팔'} waypoint 파일을 저장했습니다.`);
  } catch (error) { showToast(error.message, true); }
});
byId('loadWaypointFile').addEventListener('click', () => byId('waypointFileInput').click());
byId('waypointFileInput').addEventListener('change', async (event) => {
  const [file] = event.target.files;
  event.target.value = '';
  if (!file) return;
  try {
    const payload = JSON.parse(await file.text());
    if (!window.confirm('파일의 waypoint로 해당 팔의 현재 목록을 교체하시겠습니까?')) return;
    const result = await request('/api/waypoints/import', {
      method: 'POST', body: JSON.stringify(payload),
    });
    if (state.availableArms.includes(payload.arm)) state.activeArm = payload.arm;
    state.renderedWaypointListKey = '';
    showToast(result.message);
    await refresh();
  } catch (error) { showToast(`파일 불러오기 실패 · ${error.message}`, true); }
});
byId('waypointList').addEventListener('input', (event) => {
  const index = Number(event.target.dataset.index);
  if (event.target.classList.contains('dwell-input')) {
    state.dwellDrafts[state.activeArm][index] = event.target.value;
  } else if (event.target.classList.contains('speed-input')) {
    state.speedDrafts[state.activeArm][index] = event.target.value;
  }
});
byId('waypointList').addEventListener('change', async (event) => {
  if (!event.target.classList.contains('mode-select')) return;
  const index = Number(event.target.dataset.index);
  const mode = event.target.value;
  try {
    const result = await request('/api/waypoints/mode', {
      method: 'POST',
      body: JSON.stringify({ arm: state.activeArm, index, mode }),
    });
    showToast(result.message || `Waypoint ${index}의 보간 방식을 변경했습니다.`);
    state.renderedWaypointListKey = '';
    await refresh();
  } catch (error) {
    showToast(error.message, true);
    state.renderedWaypointListKey = '';
    await refresh();
  }
});
byId('waypointList').addEventListener('click', async (event) => {
  const item = event.target.closest('.waypoint-item');
  const index = Number(event.target.dataset.index || item?.dataset.index);
  if (!Number.isInteger(index) || index < 1) return;
  if (event.target.classList.contains('save-name')) {
    const input = byId('waypointList').querySelector(`.name-input[data-index="${index}"]`);
    try {
      const result = await request('/api/waypoints/name', {
        method: 'POST', body: JSON.stringify({ arm: state.activeArm, index, name: input?.value }),
      });
      state.renderedWaypointListKey = '';
      showToast(result.message);
      await refresh();
    } catch (error) { showToast(error.message, true); }
    return;
  }
  if (event.target.classList.contains('save-dwell')) {
    const input = byId('waypointList').querySelector(`.dwell-input[data-index="${index}"]`);
    const dwell = Number(input?.value);
    if (!Number.isFinite(dwell)) {
      showToast('대기 시간을 숫자로 입력해 주세요.', true); return;
    }
    try {
      const result = await request('/api/waypoints/dwell', {
        method: 'POST',
        body: JSON.stringify({ arm: state.activeArm, index, dwell_s: dwell }),
      });
      delete state.dwellDrafts[state.activeArm][index];
      showToast(result.message || `Waypoint ${index}의 대기 시간을 저장했습니다.`);
      await refresh();
    } catch (error) { showToast(error.message, true); }
    return;
  }
  if (event.target.classList.contains('save-speed')) {
    const input = byId('waypointList').querySelector(`.speed-input[data-index="${index}"]`);
    const speed = Number(input?.value);
    if (!Number.isFinite(speed)) {
      showToast('동작 속도를 숫자로 입력해 주세요.', true); return;
    }
    try {
      const result = await request('/api/waypoints/speed', {
        method: 'POST',
        body: JSON.stringify({ arm: state.activeArm, index, speed_scale: speed }),
      });
      delete state.speedDrafts[state.activeArm][index];
      showToast(result.message || `Waypoint ${index}의 동작 속도를 저장했습니다.`);
      await refresh();
    } catch (error) { showToast(error.message, true); }
    return;
  }
  if (event.target.classList.contains('delete-waypoint')) {
    if (!window.confirm(`Waypoint ${index}을(를) 삭제하시겠습니까?`)) return;
    try {
      const result = await request('/api/waypoints/remove', {
        method: 'POST',
        body: JSON.stringify({ arm: state.activeArm, index }),
      });
      state.dwellDrafts[state.activeArm] = {};
      state.speedDrafts[state.activeArm] = {};
      showToast(result.message || `Waypoint ${index}을(를) 삭제했습니다.`);
      await refresh();
    } catch (error) { showToast(error.message, true); }
    return;
  }
  if (!item || event.target.closest('button, input, select, label')) return;
  state.selectedWaypoints[state.activeArm] = index;
  renderPosePanels(state.data);
  renderPoseEditor(state.data);
  renderWaypointInspector(state.data);
  try {
    const arm = state.activeArm;
    const result = await runAction('preview', {}, arm);
    const waypoint = state.armData[arm]?.waypoints[index - 1];
    if (!result?.success || !waypoint?.joint_rad) return;
    state.simulator.playbackArms = [arm];
    state.simulator.previewPoseActive = true;
    state.simulator.playhead = Number(waypoint.trajectory_time_s || 0);
    state.simulator.playing = false;
    state.simulator.lastFrame = performance.now();
    renderWaypointInspector(state.armData[arm]);
    updatePlaybackUi();
    drawSimulation();
  } catch (_error) {
    // runAction already reports the planner error.
  }
});
byId('waypointList').addEventListener('dragstart', (event) => {
  const item = event.target.closest('.waypoint-item');
  if (!item || state.data?.execution?.active) return;
  state.draggedWaypoint = Number(item.dataset.index);
  item.classList.add('dragging');
  event.dataTransfer.effectAllowed = 'move';
});
byId('waypointList').addEventListener('dragover', (event) => {
  const item = event.target.closest('.waypoint-item');
  if (!item || !state.draggedWaypoint) return;
  event.preventDefault();
  event.dataTransfer.dropEffect = 'move';
});
byId('waypointList').addEventListener('drop', async (event) => {
  const item = event.target.closest('.waypoint-item');
  const source = state.draggedWaypoint;
  const target = Number(item?.dataset.index);
  state.draggedWaypoint = null;
  if (!source || !target || source === target) return;
  event.preventDefault();
  try {
    await request('/api/waypoints/reorder', {
      method: 'POST', body: JSON.stringify({ arm: state.activeArm, from_index: source, to_index: target }),
    });
    state.dwellDrafts[state.activeArm] = {};
    state.speedDrafts[state.activeArm] = {};
    state.selectedWaypoints[state.activeArm] = null;
    state.renderedWaypointListKey = '';
    showToast(`Waypoint ${source}을(를) ${target}번 위치로 옮겼습니다.`);
    await refresh();
  } catch (error) { showToast(error.message, true); await refresh(); }
});
byId('waypointList').addEventListener('dragend', () => {
  state.draggedWaypoint = null;
  document.querySelectorAll('.waypoint-item.dragging').forEach((item) => item.classList.remove('dragging'));
});
byId('copyWaypointRadians').addEventListener('click', async () => {
  const arm = state.activeArm;
  const index = state.selectedWaypoints[arm];
  const waypoint = state.armData[arm]?.waypoints[index - 1];
  if (!waypoint?.joint_rad) {
    showToast('복사할 waypoint 관절값이 없습니다.', true);
    return;
  }
  const list = `[${waypoint.joint_rad.map((value) => Number(value).toFixed(7)).join(', ')}]`;
  try {
    await copyText(list);
    showToast(`Waypoint ${index}의 8축 radian 리스트를 복사했습니다.`);
  } catch (error) {
    showToast(error.message, true);
  }
});
byId('startWaypointSelect').addEventListener('change', async (event) => {
  const index = Number(event.target.value);
  try {
    const result = await request('/api/waypoints/start', {
      method: 'POST',
      body: JSON.stringify({ arm: state.activeArm, index }),
    });
    showToast(result.message || `Waypoint ${index}부터 시작하도록 설정했습니다.`);
    await refresh();
  } catch (error) {
    showToast(error.message, true);
    await refresh();
  }
});
byId('endWaypointSelect').addEventListener('change', async (event) => {
  const index = Number(event.target.value);
  try {
    const result = await request('/api/waypoints/end', {
      method: 'POST', body: JSON.stringify({ arm: state.activeArm, index }),
    });
    showToast(result.message || `Waypoint ${index}까지 실행하도록 설정했습니다.`);
    await refresh();
  } catch (error) { showToast(error.message, true); await refresh(); }
});
byId('sequenceArm').addEventListener('change', () => renderExecutionSequence(state.data));
byId('addSequenceStep').addEventListener('click', async () => {
  try {
    const result = await request('/api/sequence/add', {
      method: 'POST', body: JSON.stringify({
        arm: byId('sequenceArm').value,
        start_index: Number(byId('sequenceStart').value),
        end_index: Number(byId('sequenceEnd').value),
      }),
    });
    showToast(result.message); await refresh();
  } catch (error) { showToast(error.message, true); }
});
byId('clearSequence').addEventListener('click', async () => {
  try {
    const result = await request('/api/sequence/clear', { method: 'POST', body: '{}' });
    showToast(result.message); await refresh();
  } catch (error) { showToast(error.message, true); }
});
byId('executionSequence').addEventListener('click', async (event) => {
  const index = Number(event.target.dataset.index);
  if (!index) return;
  let path; let payload;
  if (event.target.classList.contains('sequence-run')) {
    if (!window.confirm('이 순차 실행 단계만 실제로 실행하시겠습니까?')) return;
    path = '/api/sequence/execute-one'; payload = { index, confirm: 'EXECUTE' };
  } else if (event.target.classList.contains('sequence-delete')) {
    path = '/api/sequence/remove'; payload = { index };
  } else if (event.target.classList.contains('sequence-up')) {
    path = '/api/sequence/reorder'; payload = { from_index: index, to_index: index - 1 };
  } else if (event.target.classList.contains('sequence-down')) {
    path = '/api/sequence/reorder'; payload = { from_index: index, to_index: index + 1 };
  } else return;
  try {
    const result = await request(path, { method: 'POST', body: JSON.stringify(payload) });
    showToast(result.message);
    if (path.includes('/execute-one')) await refreshRuntime(true); else await refresh();
  } catch (error) { showToast(error.message, true); }
});
byId('executeSequence').addEventListener('click', async () => {
  if (!window.confirm('순차 실행 목록의 모든 단계를 위에서 아래 순서로 실행하시겠습니까?')) return;
  try {
    const result = await request('/api/sequence/execute', {
      method: 'POST', body: JSON.stringify({ confirm: 'EXECUTE' }),
    });
    showToast(result.message); await refreshRuntime(true);
  } catch (error) { showToast(error.message, true); }
});
byId('preview').addEventListener('click', () => previewAndPlay().catch(() => {}));
byId('previewBoth').addEventListener('click', () => previewBothAndPlay().catch(() => {}));
async function startKeyboardControl() {
  if (state.busy) return;
  if (!window.confirm(
    '선택한 팔을 world 좌표에서 키보드로 실제 이동하시겠습니까? 키를 놓으면 정지하고, 브라우저 신호가 끊기면 제어권은 NONE으로 전환됩니다.',
  )) return;
  state.busy = true;
  try {
    const result = await request('/api/keyboard/start', {
      method: 'POST',
      body: JSON.stringify({ arm: state.activeArm, confirm: 'KEYBOARD' }),
    });
    state.refreshRequestId += 1;
    setKeyboardLeaseActive(true);
    await sendKeyboardHeartbeat();
    showToast(result.message);
    await refresh();
  } catch (error) {
    setKeyboardLeaseActive(false);
    showToast(error.message, true);
  } finally {
    state.busy = false;
  }
}

function jogIsAvailable() {
  return Boolean(state.data?.keyboard_control?.active)
    && state.data?.arbiter?.mode === 'WAYPOINT'
    && !state.data?.arbiter?.fault
    && state.data?.joint_state_fresh;
}

function stopJog() {
  state.keyboard.activeDirection = null;
  if (state.keyboard.timer !== null) {
    window.clearInterval(state.keyboard.timer);
    state.keyboard.timer = null;
  }
  document.querySelectorAll('.jog-key.active').forEach(
    (button) => button.classList.remove('active'),
  );
}

function startKeyboardHeartbeatWorker() {
  const heartbeatUrl = new URL('/api/keyboard/heartbeat', window.location.href).href;
  if (state.keyboard.heartbeatWorker) {
    state.keyboard.heartbeatWorker.postMessage({ type: 'start', url: heartbeatUrl });
    return true;
  }
  if (typeof Worker === 'undefined') return false;
  try {
    const sourceUrl = URL.createObjectURL(new Blob(
      [keyboardHeartbeatWorkerSource], { type: 'text/javascript' },
    ));
    const worker = new Worker(sourceUrl);
    URL.revokeObjectURL(sourceUrl);
    worker.addEventListener('error', () => {
      worker.terminate();
      if (state.keyboard.heartbeatWorker === worker) {
        state.keyboard.heartbeatWorker = null;
      }
    });
    worker.addEventListener('message', (event) => {
      if (event.data?.type !== 'heartbeat' || event.data.ok) return;
      worker.terminate();
      if (state.keyboard.heartbeatWorker === worker) {
        state.keyboard.heartbeatWorker = null;
        sendKeyboardHeartbeat().catch(() => {});
      }
    });
    state.keyboard.heartbeatWorker = worker;
    worker.postMessage({ type: 'start', url: heartbeatUrl });
    return true;
  } catch (_error) {
    return false;
  }
}

function setKeyboardLeaseActive(active) {
  const nextActive = Boolean(active);
  if (state.keyboard.leaseActive === nextActive) return;
  state.keyboard.leaseActive = nextActive;
  if (nextActive) {
    startKeyboardHeartbeatWorker();
    return;
  }
  stopJog();
  if (state.keyboard.heartbeatWorker) {
    state.keyboard.heartbeatWorker.postMessage({ type: 'stop' });
    state.keyboard.heartbeatWorker.terminate();
    state.keyboard.heartbeatWorker = null;
  }
}

async function sendJogStep() {
  const direction = state.keyboard.activeDirection;
  if (!direction || state.keyboard.inFlight || !jogIsAvailable()) return;
  const speedInput = document.querySelector(
    `.keyboard-speed[data-direction="${direction}"]`,
  );
  const speed = Number(speedInput?.value);
  if (!Number.isFinite(speed) || speed <= 0) {
    stopJog();
    showToast('해당 방향의 이동 속도를 0보다 큰 숫자로 입력해 주세요.', true);
    return;
  }
  state.keyboard.inFlight = true;
  try {
    await request('/api/keyboard/jog', {
      method: 'POST',
      body: JSON.stringify({
        arm: state.activeArm,
        direction,
        speed_mmps: speed,
      }),
    });
  } catch (error) {
    stopJog();
    showToast(error.message, true);
    await refresh();
  } finally {
    state.keyboard.inFlight = false;
  }
}

function startJog(direction) {
  if (!jogIsAvailable()) {
    showToast('먼저 WAYPOINT 키보드 제어권을 선택해 주세요.', true);
    return;
  }
  if (state.keyboard.activeDirection === direction) return;
  stopJog();
  state.keyboard.activeDirection = direction;
  const button = document.querySelector(`.jog-key[data-direction="${direction}"]`);
  if (button) button.classList.add('active');
  sendJogStep().catch(() => {});
  state.keyboard.timer = window.setInterval(() => {
    sendJogStep().catch(() => {});
  }, 50);
}

async function sendKeyboardHeartbeat() {
  if (!state.keyboard.leaseActive || state.keyboard.heartbeatInFlight) return;
  state.keyboard.heartbeatInFlight = true;
  const controller = new AbortController();
  const timeout = window.setTimeout(
    () => controller.abort(), KEYBOARD_HEARTBEAT_REQUEST_TIMEOUT_MS,
  );
  try {
    await request('/api/keyboard/heartbeat', {
      method: 'POST', body: '{}', signal: controller.signal,
    });
  } catch (_error) {
    stopJog();
  } finally {
    window.clearTimeout(timeout);
    state.keyboard.heartbeatInFlight = false;
  }
}

byId('startKeyboard').addEventListener('click', () => {
  startKeyboardControl().catch(() => {});
});
byId('prepareTeleop').addEventListener('click', () => {
  if (!window.confirm(
    '상대 TELEOP 입력을 미리 연결하시겠습니까?\n'
    + '준비가 끝나도 제어권은 NONE으로 유지되며 로봇은 움직이지 않습니다.',
  )) return;
  stopJog();
  setKeyboardLeaseActive(false);
  showToast('상대 TELEOP 연결을 준비하고 있습니다...');
  runAction('prepare_teleop', { confirm: 'PREPARE_TELEOP' }).catch(() => {});
});
byId('startTeleop').addEventListener('click', () => {
  if (!window.confirm(
    'Mini leader 상대 TELEOP 입력 경로로 전환하시겠습니까?\n'
    + 'UI가 intervention과 양팔 입력을 먼저 준비합니다. '
    + '각 팔은 리더의 물리 버튼으로 별도 활성화해야 하며, '
    + '활성화 시점에 relative 기준 자세가 새로 설정됩니다.',
  )) return;
  stopJog();
  setKeyboardLeaseActive(false);
  runAction('select_teleop', { confirm: 'TELEOP' }).catch(() => {});
});
const stopButton = byId('stop');
let stopPointerDownAt = Number.NEGATIVE_INFINITY;
stopButton.addEventListener('pointerdown', (event) => {
  if (event.isTrusted && event.isPrimary && event.button === 0) {
    stopPointerDownAt = performance.now();
  }
});
stopButton.addEventListener('pointercancel', () => {
  stopPointerDownAt = Number.NEGATIVE_INFINITY;
});
stopButton.addEventListener('click', (event) => {
  const pointerClick = event.isTrusted
    && event.detail > 0
    && performance.now() - stopPointerDownAt <= STOP_POINTER_CLICK_WINDOW_MS;
  stopPointerDownAt = Number.NEGATIVE_INFINITY;
  if (!pointerClick) {
    showToast('정지하려면 정지 요청 버튼을 마우스나 터치로 직접 눌러주세요.', true);
    return;
  }
  stopJog();
  setKeyboardLeaseActive(false);
  runAction('stop', { confirm: STOP_BUTTON_CONFIRMATION }).catch(() => {});
});

const jogKeyMap = {
  KeyW: 'front', KeyS: 'back', KeyA: 'left',
  KeyD: 'right', KeyR: 'up', KeyF: 'down',
};
window.addEventListener('keydown', (event) => {
  if (event.repeat || !jogKeyMap[event.code]) return;
  const target = event.target;
  if (target instanceof HTMLElement && (
    target.matches('input, textarea, select, button') || target.isContentEditable
  )) return;
  event.preventDefault();
  startJog(jogKeyMap[event.code]);
});
window.addEventListener('keyup', (event) => {
  if (jogKeyMap[event.code] === state.keyboard.activeDirection) stopJog();
});
window.addEventListener('blur', stopJog);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    stopJog();
  } else {
    sendKeyboardHeartbeat().catch(() => {});
    refresh().catch(() => {});
  }
});
document.querySelectorAll('.jog-key').forEach((button) => {
  button.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    startJog(button.dataset.direction);
  });
  for (const eventName of ['pointerup', 'pointercancel', 'lostpointercapture']) {
    button.addEventListener(eventName, stopJog);
  }
});
byId('execute').addEventListener('click', () => {
  if (!window.confirm('현재 선택한 팔만 실제 controller에 trajectory를 전송합니다. 순차 목록의 다음 단계는 실행되지 않습니다. 계속하시겠습니까?')) return;
  runAction('execute', { confirm: 'EXECUTE' }).catch(() => {});
});
byId('executeSlowStart').addEventListener('click', () => {
  if (!window.confirm('현재 자세에서 선택한 시작 waypoint까지 저속으로 정렬한 뒤 나머지 경로를 실행합니다. 계속하시겠습니까?')) return;
  runAction('execute_slow_start', { confirm: 'EXECUTE' }).catch(() => {});
});
byId('executeConsent').addEventListener('change', (event) => {
  if (state.data && state.data.publish_enabled) {
    byId('execute').disabled = !event.target.checked
      || Boolean(state.data.execution?.active)
      || Boolean(state.data.keyboard_control?.active);
    byId('executeSlowStart').disabled = !event.target.checked
      || Boolean(state.data.execution?.active)
      || Boolean(state.data.keyboard_control?.active);
    byId('executeSequence').disabled = !event.target.checked
      || !(state.data.execution_sequence || []).length
      || Boolean(state.data.execution?.active)
      || Boolean(state.data.keyboard_control?.active);
    document.querySelectorAll('.sequence-run').forEach((button) => {
      button.disabled = !event.target.checked
        || Boolean(state.data.execution?.active)
        || Boolean(state.data.keyboard_control?.active);
    });
  }
});
function equalTimeFingerprint(data) {
  return JSON.stringify([(data.waypoints || []).map((point) => [
    point.position_m, point.orientation_xyzw, point.joint_rad, point.gripper_rad,
    point.speed_scale, point.dwell_s, point.interpolation_mode,
  ]), data.joint_smoothing_enabled, data.joint_timing, data.path_refinement]);
}

function renderEqualTimeControls(data) {
  byId('equalTimePanel').hidden = !data.equal_segment_time_supported || !data.equal_total_time_supported;
  const draft = state.equalTimes[data.arm];
  const totalMode = draft.mode === 'total';
  byId('equalTimeMode').value = draft.mode;
  byId('equalTimeSecondsLabel').textContent = totalMode ? '전체 목표시간 (초)' : '구간당 시간 (초)';
  byId('equalTimeSeconds').placeholder = totalMode ? '예: 180' : '비우면 자동 계산';
  byId('equalTimeAll').textContent = totalMode ? '전체 목표시간 적용' : '모든 구간 이동시간 동일';
  if (draft.excluded === null && data.waypoints?.length) {
    draft.excluded = data.joint_timing
      ? `${data.joint_timing.preserve_start_waypoint}-${data.joint_timing.preserve_end_waypoint}` : '';
  }
  if (document.activeElement !== byId('equalTimeSeconds')) byId('equalTimeSeconds').value = draft.seconds;
  if (document.activeElement !== byId('equalTimeExcluded')) byId('equalTimeExcluded').value = draft.excluded || '';
  const disabled = state.equalTimeBusy || Boolean(data.execution?.active)
    || Boolean(data.keyboard_control?.active)
    || (data.waypoints?.length || 0) < 2;
  ['equalTimeAll', 'equalTimeExcept', 'equalTimeSeconds', 'equalTimeExcluded', 'equalTimeMode'].forEach((id) => {
    byId(id).disabled = disabled;
  });
  const result = byId('equalTimeResult');
  const outdated = draft.result?.fingerprint
    && draft.result.fingerprint !== equalTimeFingerprint(data);
  result.textContent = state.equalTimeBusy ? '시간 계산 및 궤적 검증 중…'
    : outdated ? '경로 또는 속도 설정이 변경되었습니다. 다시 적용해 검증해 주세요.'
      : draft.result?.message || '시간 배정 후 검증 결과가 여기에 표시됩니다.';
  result.classList.toggle('error', Boolean(draft.result?.error) || Boolean(outdated));
}

function parseExcludedWaypointNumbers(text, count) {
  const indices = new Set();
  if (!text.trim()) throw new Error('제외할 도착 waypoint 번호를 입력해 주세요.');
  for (const part of text.split(',')) {
    const match = part.trim().match(/^(\d+)\s*(?:[-–]\s*(\d+))?$/);
    if (!match) throw new Error('제외 번호는 225-231, 240 형식으로 입력해 주세요.');
    const first = Number(match[1]);
    const last = Number(match[2] || match[1]);
    if (!Number.isSafeInteger(first) || !Number.isSafeInteger(last)
        || first < 2 || last > count || first > last) {
      throw new Error(`제외할 도착 waypoint 번호는 2–${count} 범위로 입력해 주세요.`);
    }
    for (let i = first; i <= last; i += 1) indices.add(i);
  }
  return [...indices].sort((a, b) => a - b);
}

async function equalizeWaypointTimes(excludeSelected) {
  if (state.equalTimeBusy) return;
  const arm = state.activeArm;
  const draft = state.equalTimes[arm];
  try {
    const seconds = draft.seconds.trim() === '' ? 0 : Number(draft.seconds);
    if (!Number.isFinite(seconds) || seconds < 0) throw new Error('이동시간은 0 이상의 숫자로 입력해 주세요.');
    if (draft.mode === 'total' && seconds <= 0) throw new Error('전체 목표시간을 0보다 큰 초 단위 숫자로 입력해 주세요.');
    const excluded = excludeSelected
      ? parseExcludedWaypointNumbers(draft.excluded || '', state.armData[arm].waypoint_count) : [];
    state.equalTimeBusy = true;
    draft.result = null;
    renderEqualTimeControls(state.data);
    const result = await request('/api/waypoints/equal-time', {
      method: 'POST', body: JSON.stringify({ arm, excluded_waypoints: excluded,
        [draft.mode === 'total' ? 'total_duration_s' : 'duration_s']: seconds }),
    });
    state.speedDrafts[arm] = {};
    // Periodic runtime refreshes can supersede a full refresh while it awaits
    // the other arm. Capture the committed arm directly before saving its key.
    const verified = await request(`/api/state?arm=${arm}`);
    state.armData[arm] = verified;
    draft.result = { message: result.message, fingerprint: equalTimeFingerprint(verified) };
    if (state.activeArm === arm) renderState(verified);
    showToast(result.message);
  } catch (error) {
    draft.result = { message: error.message, error: true };
    showToast(error.message, true);
  } finally {
    state.equalTimeBusy = false;
    renderEqualTimeControls(state.data);
  }
}

byId('equalTimeSeconds').addEventListener('input', (event) => {
  state.equalTimes[state.activeArm].seconds = event.target.value;
});
byId('equalTimeMode').addEventListener('change', (event) => {
  const draft = state.equalTimes[state.activeArm];
  draft.mode = event.target.value;
  draft.seconds = '';
  renderEqualTimeControls(state.data);
});
byId('equalTimeExcluded').addEventListener('input', (event) => {
  state.equalTimes[state.activeArm].excluded = event.target.value;
});
byId('equalTimeAll').addEventListener('click', () => equalizeWaypointTimes(false));
byId('equalTimeExcept').addEventListener('click', () => equalizeWaypointTimes(true));

byId('toggleWaypointList').addEventListener('click', () => {
  state.waypointListCollapsed = !state.waypointListCollapsed;
  const collapsed = state.waypointListCollapsed;
  byId('waypointList').hidden = collapsed;
  const button = byId('toggleWaypointList');
  button.textContent = collapsed ? '▸' : '▾';
  button.setAttribute('aria-expanded', String(!collapsed));
  const label = collapsed ? 'Waypoint 목록 펼치기' : 'Waypoint 목록 접기';
  button.setAttribute('aria-label', label);
  button.title = label;
});

const refinementFields = {
  enabled: 'refinementEnabled', subdivisions: 'refinementSubdivisions',
  smoothing_percent: 'refinementStrength', position_tolerance_mm: 'refinementPosition',
  orientation_tolerance_deg: 'refinementOrientation', joint_tolerance_rad: 'refinementJoint',
  max_refinements: 'refinementRetries',
};

function updateRefinementStrength(value) {
  byId('refinementStrengthValue').textContent = `${value}%`;
  if (document.activeElement !== byId('refinementStrengthNumber')) {
    byId('refinementStrengthNumber').value = value;
  }
  byId('refinementStrengthHelp').textContent = Number(value) === 0
    ? '0%: 원본 waypoint에서 정지합니다. 생성된 중간점에서는 불필요하게 정지하지 않습니다.'
    : `${value}%: ${value <= 30 ? '통과 속도를 조금 이어 줍니다.' : value <= 70 ? '통과 속도를 적당히 이어 줍니다.' : '통과 속도를 크게 이어 줍니다.'} 숫자가 클수록 중간 정지가 줄어듭니다.`;
  document.querySelectorAll('[data-smoothing]').forEach((button) => {
    button.setAttribute('aria-pressed', String(Number(button.dataset.smoothing) === Number(value)));
  });
}

function renderRefinementMonitor(data) {
  if (!data) return;
  const events = data.path_refinement_events || [];
  const latest = events.at(-1);
  const monitor = byId('refinementMonitor');
  if (!latest) {
    monitor.textContent = '경로를 계산하면 시도별 분할 수와 위치·자세·관절 오차가 표시됩니다.';
    monitor.dataset.stage = 'idle';
    return;
  }
  const labels = { idle: '설정 적용됨', generating: '중간 관절점 생성 중', validating: '오차 검증 중',
    retry: 'Cartesian 세분화·구간 연결 완화 후 재시도', failed: '생성 실패 · 실행 차단', success: '오차 검증 통과' };
  monitor.dataset.stage = latest.stage;
  const summary = events.filter((event) => ['retry', 'failed', 'success'].includes(event.stage));
  const format = (value) => Number(value || 0).toPrecision(4);
  const reasons = {
    IK_FAILURE: '중간 목표의 IK를 풀지 못했습니다. 목표 도달 가능 여부와 KDL 오류 코드를 확인하세요.',
    JOINT_LIMIT: '관절값이 허용 범위를 벗어났습니다. 해당 관절과 원본 목표를 확인하세요.',
    JOINT_STEP: '한 번에 전송할 관절 변화량이 제한을 넘었습니다. 해당 구간의 속도를 낮추세요.',
    TOLERANCE_EXCEEDED: '허용 오차를 넘었습니다. 분할 수·재시도를 늘리거나 스무딩 강도를 낮추세요. 끝점 오차가 남으면 원본 목표를 확인하세요.',
    RESOURCE_LIMIT: '생성량 제한에 도달했습니다. 선택 구간을 줄이거나 구간을 나누어 계산하세요.',
  };
  monitor.textContent = `${labels[latest.stage] || latest.stage}${latest.attempt ? ` · ${latest.attempt}차 · 구간당 ${latest.subdivisions} Cartesian 분할` : ''}\n`
    + (latest.stage === 'success' ? `JOINT 구간당 중간점 ${latest.joint_intermediate_points}개 · 전체 경로점 ${latest.knot_count}개 · 적용 스무딩 ${format(latest.effective_smoothing_percent)}%\n` : '')
    + (latest.ik_searches ? `관절 제한 회피 탐색 ${latest.ik_searches}회 · 여유를 줄여 허용한 해 ${latest.ik_relaxed_solutions}개 · 최소 IK 여유 ${format(latest.ik_min_margin_rad)} rad\n` : '')
    + (latest.position_tolerance_mm ? `허용: 위치 ${format(latest.position_tolerance_mm)} mm / 자세 ${format(latest.orientation_tolerance_deg)}° / JOINT ${format(latest.joint_tolerance_rad)} rad\n` : '')
    + summary.map((event) => `${event.attempt}차: ${event.subdivisions}분할 · 최대 위치 ${format(event.max_position_error_mm)} mm / 자세 ${format(event.max_orientation_error_deg)}° / JOINT ${format(event.max_joint_error_rad)} rad`
      + (Number.isFinite(event.effective_smoothing_percent) ? ` · 적용 강도 ${format(event.effective_smoothing_percent)}%` : '')
      + (event.segment_end_waypoint ? ` · 도착 waypoint ${event.segment_end_waypoint}` : '')
      + (event.stage === 'success' ? '' : `\n원인: ${reasons[event.reason_code] || event.reason}\n상세: ${event.reason}`)).join('\n')
    + (latest.stage === 'success' && !data.trajectory ? '\n경로가 변경되었습니다. 다시 계산하세요.' : '');
}

function renderRefinementControls(data) {
  const defaults = { enabled: false, subdivisions: 10, smoothing_percent: 100,
    position_tolerance_mm: 1, orientation_tolerance_deg: 1, joint_tolerance_rad: 0.01, max_refinements: 5 };
  const appliedKey = JSON.stringify(data.path_refinement || defaults);
  if (!state.refinementDrafts[data.arm] || state.refinementApplied[data.arm] !== appliedKey) {
    state.refinementDrafts[data.arm] = { ...defaults, ...data.path_refinement };
    state.refinementApplied[data.arm] = appliedKey;
  }
  const draft = state.refinementDrafts[data.arm];
  Object.entries(refinementFields).forEach(([key, id]) => {
    const input = byId(id);
    if (document.activeElement !== input) {
      if (key === 'enabled') input.checked = draft[key];
      else input.value = key === 'subdivisions' ? draft[key] - 1 : draft[key];
    }
    input.disabled = Boolean(data.execution?.active) || Boolean(data.keyboard_control?.active) || state.refinementBusy;
  });
  updateRefinementStrength(draft.smoothing_percent);
  byId('refinementStrengthNumber').disabled = byId('refinementStrength').disabled;
  byId('applyRefinement').disabled = state.busy || state.refinementBusy || Boolean(data.execution?.active)
    || Boolean(data.keyboard_control?.active);
  document.querySelectorAll('[data-smoothing]').forEach((button) => {
    button.disabled = byId('refinementStrength').disabled;
  });
  renderRefinementMonitor(data);
}

Object.entries(refinementFields).forEach(([key, id]) => {
  byId(id).addEventListener('input', (event) => {
    const draft = state.refinementDrafts[state.activeArm];
    if (!draft) return;
    draft[key] = key === 'enabled' ? event.target.checked
      : Number(event.target.value) + (key === 'subdivisions' ? 1 : 0);
    if (key === 'smoothing_percent') updateRefinementStrength(draft[key]);
  });
});
byId('refinementStrengthNumber').addEventListener('input', (event) => {
  if (!event.target.checkValidity()) return;
  byId('refinementStrength').value = event.target.value;
  byId('refinementStrength').dispatchEvent(new Event('input'));
});
document.querySelectorAll('[data-smoothing]').forEach((button) => {
  button.addEventListener('click', () => {
    byId('refinementStrength').value = button.dataset.smoothing;
    byId('refinementStrength').dispatchEvent(new Event('input'));
  });
});
byId('applyRefinement').addEventListener('click', async () => {
  if (state.busy || state.refinementBusy) return;
  const arm = state.activeArm;
  const settings = { ...state.refinementDrafts[arm] };
  const invalid = [...Object.values(refinementFields), 'refinementStrengthNumber']
    .map(byId).find((input) => !input.checkValidity());
  if (invalid) { invalid.reportValidity(); return; }
  state.refinementBusy = true;
  renderRefinementControls(state.armData[arm]);
  try {
    await request('/api/path-refinement', { method: 'POST',
      body: JSON.stringify({ arm, path_refinement: settings }) });
    await refresh();
    await runAction('preview', {}, arm);
  } catch (error) {
    showToast(error.message, true);
  } finally {
    state.refinementBusy = false;
    await refresh();
  }
});
byId('downloadRefinementReport').addEventListener('click', () => {
  const data = state.armData[state.activeArm];
  const blob = new Blob([JSON.stringify({ arm: data.arm, settings: data.path_refinement,
    events: data.path_refinement_events, planner_status: data.status }, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = `${data.arm}-path-diagnostics.json`;
  link.click();
  URL.revokeObjectURL(url);
});

byId('jointSmoothing').addEventListener('change', async (event) => {
  const enabled = event.target.checked;
  try {
    const result = await request('/api/joint-smoothing', {
      method: 'POST',
      body: JSON.stringify({ arm: state.activeArm, enabled }),
    });
    showToast(result.message);
    await refresh();
  } catch (error) {
    showToast(error.message, true);
    event.target.checked = !enabled;
    await refresh();
  }
});
document.querySelectorAll('.mode-card').forEach((button) => {
  button.addEventListener('click', () => {
    const action = button.dataset.mode === 'linear' ? 'select_linear' : 'select_joint';
    runAction(action).catch(() => {});
  });
});
document.querySelectorAll('.arm-tab').forEach((button) => {
  button.addEventListener('click', () => {
    const arm = button.dataset.arm;
    if (!state.availableArms.includes(arm) || !state.armData[arm]) return;
    stopJog();
    state.activeArm = arm;
    state.simulator.playbackArms = [];
    state.simulator.previewPoseActive = false;
    state.simulator.playhead = 0;
    state.simulator.playing = false;
    renderState(state.armData[arm]);
  });
});
window.addEventListener('resize', () => {
  if (state.data && state.data.trajectory) drawChart(state.data.trajectory);
  drawSimulation();
});

document.querySelectorAll('.pose-mode').forEach((button) => {
  button.addEventListener('click', () => {
    state.simulator.poseEditorMode = button.dataset.poseMode === 'rotate'
      ? 'rotate'
      : 'translate';
    if (state.data) renderPoseEditor(state.data);
  });
});

document.querySelectorAll('.pose-editor-input').forEach((input) => {
  input.addEventListener('input', () => {
    const values = POSE_EDITOR_INPUT_IDS.map((id) => byId(id).valueAsNumber);
    if (values.some((value) => !Number.isFinite(value))) return;
    state.simulator.poseDrafts[state.activeArm] = {
      position_m: values.slice(0, 3).map((value) => value * 0.001),
      orientation_deg: values.slice(3),
    };
    if (state.data) renderPoseEditor(state.data);
  });
});

byId('poseLoadCurrent').addEventListener('click', () => {
  const draft = poseDraftFromWorldPose(state.data?.current_world_pose);
  if (!draft) {
    showToast('현재 그리퍼의 world 자세를 아직 받지 못했습니다.', true);
    return;
  }
  state.simulator.poseDrafts[state.activeArm] = draft;
  renderPoseEditor(state.data);
  showToast('현재 자세를 6축 목표로 불러왔습니다.');
});

byId('poseLoadSelected').addEventListener('click', () => {
  const index = state.selectedWaypoints[state.activeArm];
  const pose = index ? state.data?.waypoints[index - 1]?.world_pose : null;
  const draft = poseDraftFromWorldPose(pose);
  if (!draft) {
    showToast('목록에서 world pose가 있는 waypoint를 먼저 선택해 주세요.', true);
    return;
  }
  state.simulator.poseDrafts[state.activeArm] = draft;
  renderPoseEditor(state.data);
  showToast(`Waypoint ${index} 자세를 6축 목표로 불러왔습니다.`);
});

byId('poseAddWaypoint').addEventListener('click', async () => {
  const draft = state.simulator.poseDrafts[state.activeArm];
  if (!draft) {
    showToast('추가할 목표 자세를 먼저 입력해 주세요.', true);
    return;
  }
  try {
    const result = await request('/api/waypoints/absolute', {
      method: 'POST',
      body: JSON.stringify({
        arm: state.activeArm,
        target_mode: 'pose',
        position_m: draft.position_m,
        orientation_deg: draft.orientation_deg,
        dwell_s: Number(state.nextDwells[state.activeArm]),
        speed_scale: Number(state.nextSpeeds[state.activeArm]),
      }),
    });
    showToast(result.message || '6축 목표 자세를 waypoint로 추가했습니다.');
    await refresh();
  } catch (error) {
    showToast(`6축 목표 IK 계산 실패: ${error.message}`, true);
  }
});

byId('simulationPlay').addEventListener('click', () => {
  const duration = playbackDuration();
  if (duration <= 0) return;
  if (!state.simulator.playing && state.simulator.playhead >= duration) {
    state.simulator.playhead = 0;
  }
  state.simulator.playing = !state.simulator.playing;
  state.simulator.previewPoseActive = true;
  state.simulator.lastFrame = performance.now();
  updatePlaybackUi(); drawSimulation();
});
byId('simulationTimeline').addEventListener('input', (event) => {
  const duration = playbackDuration();
  if (duration <= 0) return;
  state.simulator.playing = false;
  state.simulator.previewPoseActive = true;
  state.simulator.playhead = duration * Number(event.target.value) / 1000;
  updatePlaybackUi(); drawSimulation();
});
byId('resetView').addEventListener('click', () => {
  if (state.simulator.fullViewer) {
    state.simulator.fullViewer.resetCamera();
    return;
  }
  state.simulator.yaw = -0.72;
  state.simulator.pitch = 0.56;
  state.simulator.zoom = 1;
  drawSimulation();
});

const simulationCanvas = byId('robotSimulation');
simulationCanvas.addEventListener('pointerdown', (event) => {
  if (state.simulator.fullViewer) return;
  state.simulator.drag = { x: event.clientX, y: event.clientY };
  simulationCanvas.setPointerCapture(event.pointerId);
  simulationCanvas.classList.add('dragging');
});
simulationCanvas.addEventListener('pointermove', (event) => {
  if (state.simulator.fullViewer) return;
  if (!state.simulator.drag) return;
  const deltaX = event.clientX - state.simulator.drag.x;
  const deltaY = event.clientY - state.simulator.drag.y;
  state.simulator.drag = { x: event.clientX, y: event.clientY };
  state.simulator.yaw += deltaX * 0.008;
  state.simulator.pitch = Math.max(-1.35, Math.min(1.35, state.simulator.pitch - deltaY * 0.008));
  drawSimulation();
});
function stopViewDrag() {
  state.simulator.drag = null;
  simulationCanvas.classList.remove('dragging');
}
simulationCanvas.addEventListener('pointerup', stopViewDrag);
simulationCanvas.addEventListener('pointercancel', stopViewDrag);
simulationCanvas.addEventListener('wheel', (event) => {
  if (state.simulator.fullViewer) return;
  event.preventDefault();
  state.simulator.zoom = Math.max(0.55, Math.min(2.5, state.simulator.zoom * Math.exp(-event.deltaY * 0.001)));
  drawSimulation();
}, { passive: false });

refresh();
loadRobotModel();
window.requestAnimationFrame(animateSimulation);
window.setInterval(() => { refreshRuntime().catch(() => {}); }, 350);
window.setInterval(() => {
  // A Worker is resilient to browser timer throttling. The main thread only
  // takes over when Workers are unavailable or a Worker has failed.
  if (!state.keyboard.heartbeatWorker) sendKeyboardHeartbeat().catch(() => {});
}, KEYBOARD_HEARTBEAT_INTERVAL_MS);
