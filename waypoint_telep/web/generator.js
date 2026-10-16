const state = {
  reference: null,
  generated: null,
  sourceName: '',
  busy: false,
  simulator: {
    viewer: null,
    model: null,
    runtime: null,
    arm: null,
    loading: false,
    loadToken: 0,
    selectedWaypoint: 1,
    poseSource: 'reference',
    plannedTrajectory: null,
  },
};

const byId = (id) => document.getElementById(id);

function toast(message, error = false) {
  const element = byId('toast');
  element.textContent = message;
  element.className = `toast show${error ? ' error' : ''}`;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => { element.className = 'toast'; }, 3200);
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

function finiteArray(value, length) {
  return Array.isArray(value) && value.length === length
    && value.every((item) => Number.isFinite(Number(item)));
}

function validateReference(data) {
  if (!data || data.schema !== 'ai_worker_waypoints/v1') {
    throw new Error('ai_worker_waypoints/v1 형식의 파일이 아닙니다.');
  }
  if (!['left', 'right'].includes(data.arm)) throw new Error('파일의 arm은 left 또는 right여야 합니다.');
  if (!Array.isArray(data.waypoints) || data.waypoints.length === 0) {
    throw new Error('기준 경로에 waypoint가 없습니다.');
  }
  data.waypoints.forEach((waypoint, index) => {
    if (!finiteArray(waypoint.joint_positions_rad, 8)) {
      throw new Error(`Waypoint ${index + 1}의 8축 관절값이 올바르지 않습니다.`);
    }
    if (!finiteArray(waypoint.position_m, 3) || !finiteArray(waypoint.orientation_xyzw, 4)) {
      throw new Error(`Waypoint ${index + 1}의 pose가 올바르지 않습니다.`);
    }
  });
  return data;
}

function numberValue(id) {
  const value = byId(id).value.trim();
  return value === '' ? Number.NaN : Number(value);
}

function correctionInput() {
  if (!state.reference) return null;
  const complete = numberValue('graspCompleteWaypoint');
  const offset = [numberValue('offsetX'), numberValue('offsetY'), numberValue('offsetZ')];
  if (![complete, ...offset].every(Number.isFinite)) return null;
  return { grasp: 1, complete, approach: 1, offset };
}

function correctionValid(showMessage = false) {
  const input = correctionInput();
  const count = state.reference?.waypoints.length || 0;
  let message = '';
  if (!input || !Number.isInteger(input.complete)) {
    message = '파지 완료 waypoint와 보정량을 숫자로 입력해 주세요.';
  } else if (input.complete < 1 || input.complete > count) {
    message = `파지 완료 waypoint는 1~${count} 범위로 입력해 주세요.`;
  } else if (Math.hypot(...input.offset) > 50 + 1e-9) {
    message = 'XYZ 합성 보정량은 50 mm 이하여야 합니다.';
  } else if (!byId('graspConfirmed').checked) {
    message = '기준 파지 완료 시점을 확인해 주세요.';
  }
  if (showMessage && message) toast(message, true);
  return !message;
}

function updateGenerateState() {
  byId('generateButton').disabled = state.busy || !state.reference || !correctionValid(false);
  const validationPassed = state.generated?.generator?.validation?.passed === true;
  byId('downloadButton').disabled = !validationPassed || state.busy;
  renderWaypointRows();
  renderRobotPreview();
}

function renderWaypointRows() {
  if (!state.reference) return;
  const input = correctionInput() || {
    complete: 1,
  };
  const waypoints = state.reference.waypoints;
  const indices = [];
  for (let number = Math.max(1, input.complete - 3);
    number <= Math.min(waypoints.length, input.complete + 3); number += 1) {
    indices.push(number);
  }
  if (!indices.includes(1)) indices.unshift(1);
  if (!indices.includes(waypoints.length)) indices.push(waypoints.length);
  byId('waypointRows').innerHTML = [...new Set(indices)].map((number) => {
    const waypoint = waypoints[number - 1];
    const gripper = Number(waypoint.gripper_rad ?? waypoint.joint_positions_rad[7]);
    const previous = number > 1
      ? Number(waypoints[number - 2].gripper_rad ?? waypoints[number - 2].joint_positions_rad[7])
      : gripper;
    const classes = number === input.complete ? 'grasp-row' : '';
    const labels = number === input.complete ? '파지 완료' : '';
    return `<tr class="${classes}"><td>${number}${labels ? `<br><small>${labels}</small>` : ''}</td><td>${escapeHtml(waypoint.name || `Waypoint ${number}`)}</td><td>${gripper.toFixed(4)}</td><td>${number === 1 ? '—' : `${gripper - previous >= 0 ? '+' : ''}${(gripper - previous).toFixed(4)}`}</td></tr>`;
  }).join('');
}

function escapeHtml(value) {
  return String(value).replaceAll('&', '&amp;').replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;').replaceAll('"', '&quot;').replaceAll("'", '&#039;');
}

function rotateVector(quaternion, vector) {
  let [x, y, z, w] = quaternion.map(Number);
  const norm = Math.hypot(x, y, z, w) || 1;
  [x, y, z, w] = [x / norm, y / norm, z / norm, w / norm];
  const [vx, vy, vz] = vector;
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx];
}

function predictedWaypoints() {
  if (!state.reference) return [];
  const input = correctionInput();
  if (!input || input.approach < 1 || input.grasp < input.approach) {
    return state.reference.waypoints;
  }
  const localMeters = input.offset.map((value) => value / 1000);
  return state.reference.waypoints.map((waypoint, index) => {
    const number = index + 1;
    let blend = 0;
    if (number >= input.grasp || input.grasp === input.approach) blend = number >= input.approach ? 1 : 0;
    else if (number >= input.approach) blend = (number - input.approach) / (input.grasp - input.approach);
    const translated = rotateVector(waypoint.orientation_xyzw, localMeters.map((value) => value * blend));
    return {
      ...waypoint,
      position_m: waypoint.position_m.map((value, axis) => Number(value) + translated[axis]),
      orientation_xyzw: waypoint.orientation_xyzw.map(Number),
    };
  });
}

function sampledWaypoints(waypoints, maximum = 180) {
  if (waypoints.length <= maximum) return waypoints;
  const input = correctionInput();
  const selected = new Set([0, waypoints.length - 1, state.simulator.selectedWaypoint - 1]);
  [input?.approach, input?.grasp, input?.complete].forEach((number) => {
    if (Number.isInteger(number)) selected.add(number - 1);
  });
  for (let index = 0; index < maximum; index += 1) {
    selected.add(Math.round(index * (waypoints.length - 1) / (maximum - 1)));
  }
  return [...selected].filter((index) => index >= 0 && index < waypoints.length)
    .sort((left, right) => left - right).map((index) => waypoints[index]);
}

function trajectoryFromWaypointFile(file, maximum = 480) {
  const jointNames = state.simulator.runtime?.joint_names || [];
  const waypoints = file?.waypoints || [];
  if (!jointNames.length || !waypoints.length) return null;
  const stride = Math.max(1, Math.ceil(waypoints.length / maximum));
  const sampled = waypoints.filter((_, index) => index % stride === 0);
  if (sampled.at(-1) !== waypoints.at(-1)) sampled.push(waypoints.at(-1));
  if (!sampled.every((waypoint) => finiteArray(waypoint.joint_positions_rad, jointNames.length))) {
    return null;
  }
  return {
    joint_names: jointNames,
    sample_count: sampled.length,
    duration_s: Math.max(0, sampled.length - 1),
    samples: sampled.map((waypoint, index) => ({
      time_s: index,
      positions_deg: waypoint.joint_positions_rad.map((value) => Number(value) * 180 / Math.PI),
    })),
  };
}

function liveJointValues() {
  const jointState = state.simulator.runtime?.robot_joint_state || { name: [], position: [] };
  return Object.fromEntries(
    jointState.name.map((name, index) => [name, Number(jointState.position[index] || 0)]),
  );
}

function renderRobotPreview() {
  const input = correctionInput();
  if (input) {
    const norm = Math.hypot(...input.offset);
    byId('correctionSummary').innerHTML = `파지 완료 기준은 <b>Waypoint ${input.complete}</b>입니다. 새 파지 관계를 유지하도록 <b>Waypoint 1~${state.reference.waypoints.length} 전체 경로</b>에 같은 상대 보정을 적용합니다.<br>EE 로컬 보정: X ${input.offset[0]} · Y ${input.offset[1]} · Z ${input.offset[2]} mm (합성 ${norm.toFixed(2)} mm)`;
  }
  const viewer = state.simulator.viewer;
  const model = state.simulator.model;
  if (viewer && model) {
    viewer.setWorldTransform(
      state.simulator.runtime?.keyboard_control?.frame || 'world',
      state.simulator.runtime?.world_root_pose || null,
    );
  }
  if (!state.reference) {
    if (viewer && model) {
      const jointValues = liveJointValues();
      viewer.setWaypointSets({
        left: { waypoints: [], baseLink: model.base_link },
        right: { waypoints: [], baseLink: model.base_link },
      });
      viewer.setTrajectories({ left: null, right: null }, jointValues);
      viewer.setPoseEditor({ pose: {}, visible: false, onChange: null });
      viewer.transformControls.enabled = false;
      viewer.transformControlsHelper.visible = false;
      viewer.setJointValues(jointValues);
      byId('modelMeta').textContent = '오른팔 현재 joint state를 F2 전체 URDF·STL mesh에 표시하고 있습니다.';
    }
    return;
  }

  const count = state.reference.waypoints.length;
  state.simulator.selectedWaypoint = Math.max(
    1, Math.min(count, state.simulator.selectedWaypoint),
  );
  const scrubber = byId('simulationWaypoint');
  scrubber.max = count;
  scrubber.value = state.simulator.selectedWaypoint;
  scrubber.disabled = false;
  byId('simulationWaypointLabel').textContent = `Waypoint ${state.simulator.selectedWaypoint} / ${count}`;
  byId('showCorrectedPose').disabled = !state.generated;
  if (!state.generated && state.simulator.poseSource === 'corrected') {
    state.simulator.poseSource = 'reference';
  }
  byId('showReferencePose').classList.toggle(
    'active', state.simulator.poseSource === 'reference');
  byId('showCorrectedPose').classList.toggle(
    'active', state.simulator.poseSource === 'corrected');

  if (!viewer || !model) return;
  const correctedWaypoints = state.generated?.waypoints || predictedWaypoints();
  viewer.setWaypointSets({
    left: {
      waypoints: sampledWaypoints(state.reference.waypoints),
      baseLink: model.base_link,
    },
    right: {
      waypoints: sampledWaypoints(correctedWaypoints),
      baseLink: model.base_link,
    },
  });
  const graspWaypoint = input ? correctedWaypoints[input.complete - 1] : null;
  const baseLink = viewer.robot?.links?.[model.base_link];
  if (baseLink && viewer.poseEditorTarget?.parent !== baseLink) {
    baseLink.add(viewer.poseEditorTarget);
  }
  viewer.setPoseEditor({
    pose: graspWaypoint || {},
    mode: 'translate',
    visible: Boolean(graspWaypoint),
    onChange: null,
  });
  viewer.transformControls.enabled = false;
  viewer.transformControlsHelper.visible = false;
  const graspMarker = viewer.poseEditorVisual?.children?.find((child) => child.isMesh);
  graspMarker?.material?.color?.set?.(0xf16cae);

  const jointValues = liveJointValues();
  viewer.setTrajectories({
    left: trajectoryFromWaypointFile(state.reference),
    right: state.simulator.plannedTrajectory
      || trajectoryFromWaypointFile(state.generated),
  }, jointValues);
  const poseFile = state.simulator.poseSource === 'corrected' && state.generated
    ? state.generated : state.reference;
  const poseWaypoint = poseFile.waypoints[state.simulator.selectedWaypoint - 1];
  const jointNames = state.simulator.runtime?.joint_names || [];
  if (poseWaypoint && finiteArray(poseWaypoint.joint_positions_rad, jointNames.length)) {
    jointNames.forEach((name, index) => {
      jointValues[name] = Number(poseWaypoint.joint_positions_rad[index]);
    });
  }
  viewer.setJointValues(jointValues);
  byId('modelMeta').textContent = state.generated
    ? `${poseFile.arm === 'right' ? '오른팔' : '왼팔'} ${state.simulator.poseSource === 'corrected' ? '보정' : '기준'} 로봇 자세 · 기준(청록) / 보정(분홍) 3D 경로 · 큰 XYZ축: 파지 완료 보정 pose`
    : '기준 로봇 자세와 예상 보정 pose 축을 표시합니다. 큰 XYZ축이 파지 완료 보정 pose입니다.';
}

async function ensureRobotSimulator(arm) {
  const token = state.simulator.loadToken + 1;
  state.simulator.loadToken = token;
  state.simulator.loading = true;
  byId('modelBadge').textContent = 'F2 mesh 로딩 중';
  byId('modelBadge').className = 'badge waiting';
  byId('modelMeta').textContent = `${arm === 'right' ? '오른팔' : '왼팔'} URDF·STL mesh를 불러오는 중입니다.`;
  try {
    const [model, runtime] = await Promise.all([
      request(`/api/robot-model?arm=${arm}`),
      request(`/api/runtime?arm=${arm}`),
    ]);
    if (token !== state.simulator.loadToken) return;
    if (model.status !== 'ready') throw new Error(model.message || 'URDF 모델이 준비되지 않았습니다.');
    if (!window.FullRobotSimulator) throw new Error('3D 로봇 시뮬레이터를 불러오지 못했습니다.');
    if (!state.simulator.viewer) {
      state.simulator.viewer = new window.FullRobotSimulator(byId('robotSimulation'));
    }
    if (state.simulator.arm !== arm) {
      if (state.simulator.viewer.poseEditorTarget?.parent
        !== state.simulator.viewer.robotGroup) {
        state.simulator.viewer.robotGroup.add(state.simulator.viewer.poseEditorTarget);
      }
      const assetOrigin = window.location.origin;
      await state.simulator.viewer.load({
        urdfUrl: '/api/robot-description.urdf',
        assetOrigin,
        tipLinks: { left: model.tip_link, right: model.tip_link },
      });
      if (token !== state.simulator.loadToken) return;
    }
    state.simulator.model = model;
    state.simulator.runtime = runtime;
    state.simulator.arm = arm;
    byId('simulationEmpty').style.display = 'none';
    byId('modelBadge').textContent = 'F2 3D 준비됨';
    byId('modelBadge').className = 'badge ready';
    renderRobotPreview();
  } catch (error) {
    if (token !== state.simulator.loadToken) return;
    byId('modelBadge').textContent = '3D 모델 오류';
    byId('modelBadge').className = 'badge error';
    byId('modelMeta').textContent = `F2 3D 모델을 불러오지 못했습니다: ${error.message}`;
    toast(error.message, true);
  } finally {
    if (token === state.simulator.loadToken) state.simulator.loading = false;
  }
}

async function loadReference(file) {
  try {
    const data = validateReference(JSON.parse(await file.text()));
    state.reference = data;
    state.generated = null;
    state.sourceName = file.name;
    byId('fileName').textContent = file.name;
    byId('fileSummary').className = 'file-summary';
    byId('fileSummary').textContent = `${data.arm === 'right' ? '오른팔' : '왼팔'} · ${data.waypoints.length}개 waypoint · 실행 구간 ${data.start_waypoint || 1}→${data.end_waypoint || data.waypoints.length}`;
    byId('graspCompleteWaypoint').max = data.waypoints.length;
    byId('graspCompleteWaypoint').value = '';
    byId('graspConfirmed').checked = false;
    byId('candidateHint').textContent = '자동으로 선택하지 않습니다. 영상과 gripper 값을 확인해 파지가 완료된 번호를 직접 입력하세요.';
    state.simulator.selectedWaypoint = Math.max(
      1, Math.min(data.waypoints.length, Number(data.start_waypoint || 1)),
    );
    state.simulator.poseSource = 'reference';
    state.simulator.plannedTrajectory = null;
    byId('referenceCard').classList.remove('disabled');
    byId('offsetCard').classList.remove('disabled');
    byId('resultCard').hidden = true;
    updateGenerateState();
    await ensureRobotSimulator(data.arm);
  } catch (error) {
    state.reference = null;
    byId('fileSummary').className = 'file-summary empty';
    byId('fileSummary').textContent = error.message;
    byId('referenceCard').classList.add('disabled');
    byId('offsetCard').classList.add('disabled');
    updateGenerateState();
    toast(error.message, true);
  }
}

function validateTeleopPreview(runtime, waypointFile) {
  const waypointCount = waypointFile?.waypoints?.length || 0;
  const trajectory = runtime?.trajectory;
  if (waypointCount < 1) throw new Error('검증할 최종 waypoint가 없습니다.');
  if (runtime.start_waypoint !== 1 || runtime.end_waypoint !== waypointCount) {
    throw new Error(`Teleop 검증 구간이 1→${waypointCount}로 설정되지 않았습니다.`);
  }
  const ikCount = waypointFile.waypoints.filter(
    (waypoint) => finiteArray(waypoint.joint_positions_rad, 8),
  ).length;
  if (ikCount !== waypointCount) {
    throw new Error(`Endpoint IK가 ${ikCount}/${waypointCount}개만 계산됐습니다.`);
  }
  if (!trajectory || !Number.isInteger(trajectory.sample_count)
    || trajectory.sample_count < waypointCount || !(trajectory.duration_s > 0)) {
    throw new Error('Teleop Preview trajectory가 생성되지 않았습니다.');
  }
  return {
    passed: true,
    method: 'waypoint_telep_preview',
    start_waypoint: 1,
    end_waypoint: waypointCount,
    endpoint_ik_count: ikCount,
    trajectory_sample_count: trajectory.sample_count,
    trajectory_duration_s: Number(trajectory.duration_s),
    checks: [
      'endpoint_ik',
      'interpolated_path_ik',
      'joint_limits',
      'per_sample_joint_continuity',
    ],
    collision_checked: false,
    dynamics_checked: false,
    physical_execution_checked: false,
    validated_at: new Date().toISOString(),
  };
}

async function previewFullTeleopPath(arm, waypointCount) {
  await request('/api/waypoints/start', {
    method: 'POST', body: JSON.stringify({ arm, index: 1 }),
  });
  await request('/api/waypoints/end', {
    method: 'POST', body: JSON.stringify({ arm, index: waypointCount }),
  });
  const preview = await request('/api/actions/preview', {
    method: 'POST', body: JSON.stringify({ arm }),
  });
  const [runtime, waypointFile] = await Promise.all([
    request(`/api/state?arm=${arm}`),
    request(`/api/waypoints/export?arm=${arm}`),
  ]);
  const validation = validateTeleopPreview(runtime, waypointFile);
  return {
    preview,
    runtime,
    waypointFile,
    validation,
  };
}

async function generate() {
  if (!state.reference || !correctionValid(true) || state.busy) return;
  const input = correctionInput();
  state.busy = true;
  state.generated = null;
  state.simulator.plannedTrajectory = null;
  byId('resultCard').hidden = true;
  byId('generateButton').textContent = '상대 보정 · IK · Teleop Preview 검증 중…';
  updateGenerateState();
  try {
    await request('/api/waypoints/import', {
      method: 'POST', body: JSON.stringify(state.reference),
    });
    const result = await request('/api/generator/tool-path', {
      method: 'POST',
      body: JSON.stringify({
        arm: state.reference.arm,
        approach_start_waypoint: input.approach,
        grasp_waypoint: input.grasp,
        grasp_complete_waypoint: input.complete,
        full_path_correction: true,
        local_offset_mm: input.offset,
      }),
    });
    const verified = await previewFullTeleopPath(
      state.reference.arm, result.waypoint_file.waypoints.length,
    );
    const generatorMetadata = {
      ...result.waypoint_file.generator,
      reference_grasp_waypoint: input.complete,
      grasp_complete_waypoint: input.complete,
      full_path_correction: true,
      source_file: state.sourceName,
      created_at: new Date().toISOString(),
      validation: verified.validation,
    };
    state.generated = {
      ...verified.waypointFile,
      generator: generatorMetadata,
    };
    state.simulator.runtime = verified.runtime;
    state.simulator.plannedTrajectory = verified.runtime.trajectory;
    state.simulator.poseSource = 'corrected';
    state.simulator.selectedWaypoint = input.complete;
    const sampleCount = verified.validation.trajectory_sample_count.toLocaleString('ko-KR');
    const duration = verified.validation.trajectory_duration_s.toFixed(3);
    byId('resultMessage').textContent = `파지 완료 기준 Waypoint ${input.complete} · 전체 1→${verified.validation.end_waypoint} · endpoint IK ${verified.validation.endpoint_ik_count}/${verified.validation.end_waypoint} · ${sampleCount} samples · ${duration}초`;
    byId('previewResult').textContent = 'Teleop Preview 검증 완료: 전체 보간 구간 IK, 관절 제한, 관절값 연속성을 통과했습니다. 현재 Teleop 목록에 최종 경로가 유지되며 검증된 JSON을 저장할 수 있습니다. 충돌·동역학은 검사하지 않았습니다.';
    byId('resultCard').hidden = false;
    byId('resultCard').scrollIntoView({ behavior: 'smooth', block: 'nearest' });
    renderRobotPreview();
    toast('보정 경로가 Teleop Preview 검증을 통과했습니다.');
  } catch (error) {
    state.generated = null;
    state.simulator.plannedTrajectory = null;
    try {
      await request('/api/waypoints/import', {
        method: 'POST', body: JSON.stringify(state.reference),
      });
    } catch (_rollbackError) {
      // 원래 경로 복구 실패보다 최초 검증 오류를 우선 표시한다.
    }
    toast(`검증 실패: ${error.message}`, true);
  } finally {
    state.busy = false;
    byId('generateButton').textContent = '보정 · IK · Preview 검증 후 Teleop 적용';
    updateGenerateState();
  }
}

function downloadGenerated() {
  if (!state.generated) return;
  const stem = state.sourceName.replace(/\.json$/i, '') || 'waypoints';
  const input = correctionInput();
  const suffix = `grasp_complete${input.complete}_x${input.offset[0]}_y${input.offset[1]}_z${input.offset[2]}_validated`.replaceAll('.', 'p');
  const blob = new Blob([`${JSON.stringify(state.generated, null, 2)}\n`], { type: 'application/json' });
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob);
  link.download = `${stem}_${suffix}.json`;
  link.click();
  URL.revokeObjectURL(link.href);
}

async function previewGenerated() {
  if (!state.generated) return;
  const button = byId('previewButton');
  button.disabled = true;
  byId('previewResult').textContent = '현재 실제 자세에서 전체 1→마지막 waypoint를 다시 검증하는 중입니다…';
  try {
    const verified = await previewFullTeleopPath(
      state.generated.arm, state.generated.waypoints.length,
    );
    state.generated = {
      ...verified.waypointFile,
      generator: {
        ...state.generated.generator,
        validation: verified.validation,
      },
    };
    state.simulator.runtime = verified.runtime;
    state.simulator.plannedTrajectory = verified.runtime.trajectory;
    renderRobotPreview();
    byId('previewResult').textContent = `재검증 완료: ${verified.preview.message} · 충돌·동역학은 검사하지 않았습니다.`;
    toast('Waypoint Teleop Preview 재검증을 통과했습니다.');
  } catch (error) {
    state.generated.generator.validation = {
      passed: false,
      error: error.message,
      validated_at: new Date().toISOString(),
    };
    byId('previewResult').textContent = `Preview 재검증 실패: ${error.message}`;
    toast(error.message, true);
  } finally {
    button.disabled = false;
    updateGenerateState();
  }
}

async function checkConnection() {
  try {
    const runtime = await request('/api/runtime');
    const badge = byId('connectionBadge');
    badge.textContent = `연결됨 · ${runtime.available_arms?.join(' / ') || 'Waypoint'}`;
    badge.className = 'badge ready';
  } catch (_error) {
    const badge = byId('connectionBadge');
    badge.textContent = 'Waypoint UI 연결 안 됨';
    badge.className = 'badge error';
  }
}

byId('referenceFile').addEventListener('change', (event) => {
  const [file] = event.target.files;
  if (file) loadReference(file);
});
function invalidateGeneratedPath() {
  if (!state.generated) return;
  state.generated = null;
  state.simulator.plannedTrajectory = null;
  state.simulator.poseSource = 'reference';
  byId('resultCard').hidden = true;
}

function correctionChanged() {
  invalidateGeneratedPath();
  const complete = numberValue('graspCompleteWaypoint');
  if (state.reference && Number.isInteger(complete)
    && complete >= 1 && complete <= state.reference.waypoints.length) {
    state.simulator.selectedWaypoint = complete;
  }
  updateGenerateState();
}

['graspCompleteWaypoint', 'offsetX', 'offsetY', 'offsetZ'].forEach((id) => {
  byId(id).addEventListener('input', correctionChanged);
});
byId('graspConfirmed').addEventListener('change', updateGenerateState);
document.querySelectorAll('.nudge[data-axis]').forEach((button) => {
  button.addEventListener('click', () => {
    const inputs = ['offsetX', 'offsetY', 'offsetZ'];
    const axis = Number(button.dataset.axis);
    const next = Number(byId(inputs[axis]).value || 0) + Number(button.dataset.delta);
    byId(inputs[axis]).value = Math.round(next * 1000) / 1000;
    correctionChanged();
  });
});
byId('resetOffset').addEventListener('click', () => {
  ['offsetX', 'offsetY', 'offsetZ'].forEach((id) => { byId(id).value = 0; });
  correctionChanged();
});
byId('showReferencePose').addEventListener('click', () => {
  state.simulator.poseSource = 'reference';
  renderRobotPreview();
});
byId('showCorrectedPose').addEventListener('click', () => {
  if (!state.generated) return;
  state.simulator.poseSource = 'corrected';
  renderRobotPreview();
});
byId('simulationWaypoint').addEventListener('input', (event) => {
  state.simulator.selectedWaypoint = Number(event.target.value);
  renderRobotPreview();
});
byId('reset3dView').addEventListener('click', () => {
  state.simulator.viewer?.resetCamera();
});
byId('generateButton').addEventListener('click', generate);
byId('downloadButton').addEventListener('click', downloadGenerated);
byId('previewButton').addEventListener('click', previewGenerated);

renderRobotPreview();
checkConnection();
ensureRobotSimulator('right');
