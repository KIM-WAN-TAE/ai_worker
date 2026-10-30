(() => {
const byId = (id) => document.getElementById(id);
const integrated = Boolean(byId('curveNoiseEnabled'));
const state = { locked: false, source: null, segments: [], result: null, busy: false, axes: [0, 2], noiseExtent: 10 };
const controls = ['curveFile', 'curveArm', 'loadCurrent', 'addSegment', 'curveCount', 'curveWidth', 'curveSeed'];
const add = (a, b) => a.map((v, i) => v + b[i]);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const mul = (a, k) => a.map((v) => v * k);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const norm = (a) => Math.hypot(...a);
const unit = (a) => mul(a, 1 / norm(a));
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const waypointKey = (waypoints) => JSON.stringify(waypoints.map((w) => [w.name, w.position_m,
  w.orientation_xyzw, w.joint_positions_rad, w.gripper_rad, w.dwell_s, w.speed_scale, w.interpolation_mode]));
const noiseAxes = [
  ...['X', 'Y', 'Z'].map((axis) => ({ key: `Position${axis}`, kind: 'translation', unit: 'mm', label: axis, bound: 50 })),
  ...['Roll', 'Pitch', 'Yaw'].map((axis) => ({ key: `Rotation${axis}`, kind: 'rotation', unit: 'deg', label: axis, bound: 30 })),
];

function readStartNoise() {
  if (!byId('startNoiseEnabled').checked) return { enabled: false };
  const mode = byId('startNoiseMode').value;
  const result = { enabled: true, mode, sampling: byId('startNoiseRandom').checked ? 'random' : 'manual' };
  for (const [kind, unit] of [['translation', 'mm'], ['rotation', 'deg']]) {
    const rows = noiseAxes.filter((a) => a.kind === kind);
    const active = mode === 'both' || mode === (kind === 'translation' ? 'position' : 'orientation');
    result[`${kind}_min_${unit}`] = rows.map((a) => active ? Number(byId(`startNoise${a.key}Min`).value) : 0);
    result[`${kind}_max_${unit}`] = rows.map((a) => active ? Number(byId(`startNoise${a.key}Max`).value) : 0);
    result[kind === 'translation' ? 'translation_mm' : 'rotation_rpy_deg'] = rows.map((a) => active ? Number(byId(`startNoise${a.key}`).value) : 0);
  }
  return result;
}

function sampleStartNoise(noise, seed, variant = 0) {
  if (!noise.enabled) return { translation_mm: [0, 0, 0], rotation_rpy_deg: [0, 0, 0] };
  let rng = (seed + 0x9e3779b9*(variant+1)) >>> 0;
  const result = {};
  for (const [kind, unit, key] of [['translation', 'mm', 'translation_mm'], ['rotation', 'deg', 'rotation_rpy_deg']]) {
    result[key] = noise[key].map((v, i) => {
      rng = (Math.imul(rng, 1664525) + 1013904223) >>> 0;
      return noise.sampling === 'random' ? noise[`${kind}_min_${unit}`][i]
        + (noise[`${kind}_max_${unit}`][i]-noise[`${kind}_min_${unit}`][i])*(rng+.5)/2**32 : v;
    });
  }
  return result;
}

function noiseRotation(reference, rpy) {
  const [r, p, y] = rpy.map((v) => v*Math.PI/360);
  const [cr, sr, cp, sp, cy, sy] = [Math.cos(r), Math.sin(r), Math.cos(p), Math.sin(p), Math.cos(y), Math.sin(y)];
  const [x, yy, z, w] = [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy];
  const [a, b, c, d] = reference;
  return [d*x+a*w+b*z-c*yy, d*yy-a*z+b*w+c*x, d*z+a*yy-b*x+c*w, d*w-a*x-b*yy-c*z];
}

function makeStartNoiseControls() {
  const panel = document.createElement('section'); panel.className = 'start-noise'; panel.id = 'startNoisePanel';
  panel.innerHTML = `<label class="start-noise-toggle"><input id="startNoiseEnabled" type="checkbox"> 선택 구간의 첫 시작점 바꾸기</label>
    <div id="startNoiseBody" hidden>
      <label>바꿀 항목<select id="startNoiseMode"><option value="position">위치만</option><option value="orientation">방향만</option><option value="both">위치 + 방향</option></select></label>
      <small>회색 손잡이로 최소·최대 범위, 파란 손잡이로 현재 자세를 정합니다. 파란 손잡이 위에서는 휠도 사용할 수 있습니다.</small>
      <div id="startNoisePosition"><strong>위치 · 팔 기준 XYZ (mm)</strong></div>
      <div id="startNoiseRotation"><strong>방향 · 시작 자세의 로컬 축 (°)</strong></div>
      <button id="startNoiseWider" type="button" class="button ghost">드래그 범위 넓혀 보기</button>
      <details id="startNoiseNumbers"><summary>숫자로 범위 지정 · 고급</summary><div id="startNoiseNumberRows"></div></details>
      <label class="start-noise-toggle"><input id="startNoiseRandom" type="checkbox"> 이 범위에서 무작위 시작점 생성</label>
      <label id="startNoiseCountRow" hidden>만들 경로 수<input id="startNoiseCount" type="number" min="1" max="30" step="1" value="10"></label>
      <small id="startNoiseSummary"></small>
    </div>`;
  byId('curveSegments').before(panel);
  for (const axis of noiseAxes) {
    const id = `startNoise${axis.key}`, row = document.createElement('div'); row.className = 'start-noise-row';
    row.innerHTML = `<label for="${id}">${axis.label}<output id="${id}Value">0.0</output></label>
      <div class="start-noise-track" id="${id}Track"><div class="start-noise-fill"></div>
        <input id="${id}LowerHandle" class="noise-handle noise-min" type="range" min="-10" max="10" value="-5" step="any" aria-label="${axis.label} 최소 범위 드래그">
        <input id="${id}" class="noise-handle noise-current" type="range" min="-10" max="10" value="0" step="any">
        <input id="${id}UpperHandle" class="noise-handle noise-max" type="range" min="-10" max="10" value="5" step="any" aria-label="${axis.label} 최대 범위 드래그"></div>
      <div class="start-noise-bounds"><span id="${id}MinText"></span><span id="${id}MaxText"></span></div>`;
    byId(axis.kind === 'translation' ? 'startNoisePosition' : 'startNoiseRotation').append(row);
    const numbers = document.createElement('label'); numbers.className = 'start-noise-numbers';
    numbers.innerHTML = `<span>${axis.label} (${axis.unit === 'deg' ? '°' : 'mm'})</span>
      <input id="${id}Min" type="number" min="-${axis.bound}" max="${axis.bound}" step="any" value="-5" required aria-label="${axis.label} 최소">
      <input id="${id}Max" type="number" min="-${axis.bound}" max="${axis.bound}" step="any" value="5" required aria-label="${axis.label} 최대">`;
    byId('startNoiseNumberRows').append(numbers);
    for (const suffix of ['', 'Min', 'Max']) byId(id+suffix).addEventListener('input', () => { renderStartNoise(); changed(); });
    for (const [suffix, field] of [['LowerHandle', 'Min'], ['UpperHandle', 'Max']]) byId(id+suffix).addEventListener('input', () => {
      const value = Number(byId(id+suffix).value), other = Number(byId(id+(field === 'Min' ? 'Max' : 'Min')).value);
      byId(id+field).value = field === 'Min' ? Math.min(value, other) : Math.max(value, other);
      renderStartNoise(); changed();
    });
    byId(id).addEventListener('wheel', (event) => {
      const input = byId(id); if (input.disabled || event.deltaY === 0) return;
      event.preventDefault();
      input.value = Math.max(Number(input.min), Math.min(Number(input.max), Number(input.value)+(event.deltaY < 0 ? .1 : -.1))).toFixed(1);
      input.dispatchEvent(new Event('input'));
    }, { passive: false });
  }
  panel.querySelectorAll('input[id],select[id]').forEach((input) => controls.push(input.id));
  controls.push('startNoiseWider');
  byId('startNoiseWider').addEventListener('click', () => {
    state.noiseExtent = state.noiseExtent >= 50 ? 10 : Math.min(50, state.noiseExtent*2);
    renderStartNoise();
  });
  ['startNoiseEnabled', 'startNoiseMode', 'startNoiseRandom'].forEach((id) => byId(id).addEventListener('change', () => {
    if (id === 'startNoiseRandom' && byId(id).checked && Number(byId('curveCount').value) < 2) byId('curveCount').value = '10';
    renderStartNoise(); changed();
  }));
  byId('startNoiseCount').addEventListener('input', () => {
    byId('curveCount').value = byId('startNoiseCount').value; renderStartNoise(); changed();
  });
  renderStartNoise();
}

function renderStartNoise() {
  const enabled = byId('startNoiseEnabled').checked, random = byId('startNoiseRandom').checked;
  const mode = byId('startNoiseMode').value, locked = state.busy || state.locked;
  byId('startNoiseEnabled').disabled = locked;
  byId('startNoiseBody').hidden = !enabled;
  byId('startNoiseMode').disabled = locked || !enabled;
  byId('startNoiseRandom').disabled = locked || !enabled;
  byId('startNoiseWider').disabled = locked || !enabled;
  byId('startNoiseCountRow').hidden = !random;
  byId('startNoiseCount').disabled = locked || !enabled || !random;
  byId('startNoiseCount').value = byId('curveCount').value;
  byId('startNoisePosition').hidden = mode === 'orientation';
  byId('startNoiseRotation').hidden = mode === 'position';
  let valid = true;
  for (const axis of noiseAxes) {
    const id = `startNoise${axis.key}`, input = byId(id), lower = byId(id+'Min'), upper = byId(id+'Max');
    const active = mode === 'both' || mode === (axis.kind === 'translation' ? 'position' : 'orientation');
    lower.disabled = upper.disabled = locked || !enabled || !active;
    input.disabled = locked || !enabled || !active || random;
    byId(id+'LowerHandle').disabled = byId(id+'UpperHandle').disabled = locked || !enabled || !active;
    lower.setCustomValidity(''); upper.setCustomValidity('');
    if (lower.value === '' || upper.value === '' || Number(lower.value) > Number(upper.value)) {
      lower.setCustomValidity('최소·최대 범위를 확인해 주세요.'); valid = false;
    } else {
      const extent = Math.min(axis.bound, Math.max(state.noiseExtent, Math.ceil(Math.max(Math.abs(Number(lower.value)), Math.abs(Number(upper.value))))));
      for (const handle of [input, byId(id+'LowerHandle'), byId(id+'UpperHandle')]) {
        handle.min = -extent; handle.max = extent;
      }
      input.value = Math.max(Number(lower.value), Math.min(Number(upper.value), Number(input.value)));
      byId(id+'LowerHandle').value = lower.value; byId(id+'UpperHandle').value = upper.value;
      byId(id+'Track').style.setProperty('--noise-min', `${(Number(lower.value)+extent)/(2*extent)*100}%`);
      byId(id+'Track').style.setProperty('--noise-max', `${(Number(upper.value)+extent)/(2*extent)*100}%`);
    }
    byId(id+'MinText').textContent = `최소 ${Number(lower.value).toFixed(1)}`;
    byId(id+'MaxText').textContent = `최대 ${Number(upper.value).toFixed(1)}`;
    byId(id+'Value').textContent = Number(input.value).toFixed(1);
  }
  const noise = readStartNoise(), sample = sampleStartNoise(noise, Number(byId('curveSeed').value));
  if (enabled && random && valid) noiseAxes.forEach((axis, i) => {
    const value = sample[i < 3 ? 'translation_mm' : 'rotation_rpy_deg'][i%3];
    byId(`startNoise${axis.key}`).value = value;
    byId(`startNoise${axis.key}Value`).textContent = value.toFixed(2);
  });
  byId('startNoiseSummary').textContent = enabled
    ? `미리보기 ${random ? '첫 번째 샘플' : '시작점'} · XYZ ${sample.translation_mm.map((v) => v.toFixed(2)).join(', ')} mm / RPY ${sample.rotation_rpy_deg.map((v) => v.toFixed(2)).join(', ')}°`
    : '';
}

function settings() {
  return { waypoint_file: state.source, segments: state.segments.map((s) => ({ ...s })),
    count: Number(byId('curveCount').value), max_deviation_mm: Number(byId('curveWidth').value),
    seed: Number(byId('curveSeed').value), preserve_outside: integrated, start_noise: readStartNoise() };
}

function anchors() {
  if (!state.source || !state.segments.length) return null;
  const indices = [state.segments[0].start_waypoint, ...state.segments.map((s) => s.end_waypoint)];
  if (state.segments.some((s, i) => s.start_waypoint === s.end_waypoint
      || (i && s.start_waypoint !== state.segments[i-1].end_waypoint))) return null;
  return indices.map((n) => state.source.waypoints[n-1]?.position_m);
}

function renderSegments() {
  const container = byId('curveSegments');
  container.replaceChildren();
  state.segments.forEach((segment, index) => {
    const row = document.createElement('div'); row.className = 'curve-segment';
    const heading = document.createElement('strong'); heading.textContent = `구간 ${index+1}`; row.append(heading);
    const pointSelect = (key, text) => {
      const label = document.createElement('label'); label.textContent = text;
      const select = document.createElement('select');
      state.source.waypoints.forEach((w, i) => {
        const option = document.createElement('option'); option.value = i+1;
        option.textContent = `${i+1}. ${w.name || '지점'}`; select.append(option);
      });
      select.value = segment[key]; select.disabled = state.busy || state.locked || (key === 'start_waypoint' && index > 0);
      select.addEventListener('change', () => {
        segment[key] = Number(select.value);
        if (key === 'end_waypoint' && state.segments[index+1]) state.segments[index+1].start_waypoint = segment[key];
        renderSegments(); changed();
      });
      label.append(select); row.append(label);
    };
    pointSelect('start_waypoint', index ? '앞 구간의 끝점에서 시작' : '시작점');
    pointSelect('end_waypoint', '끝점');
    const label = document.createElement('label'); label.textContent = '연결 방식';
    const mode = document.createElement('select');
    [['CURVE', '곡선'], ['LINEAR', '직선']].forEach(([value, text]) => {
      const option = document.createElement('option'); option.value = value; option.textContent = text; mode.append(option);
    });
    mode.value = segment.mode; mode.disabled = state.busy || state.locked;
    mode.addEventListener('change', () => { segment.mode = mode.value; renderSegments(); changed(); });
    label.append(mode); row.append(label);
    if (segment.mode === 'CURVE') {
      const label = document.createElement('label'); label.className = 'curve-strength';
      const output = document.createElement('output'); output.textContent = `곡선 정도 ${segment.curve_percent}%`;
      const slider = document.createElement('input'); slider.type = 'range'; slider.min = 0; slider.max = 100;
      slider.value = segment.curve_percent; slider.disabled = state.busy || state.locked;
      slider.addEventListener('input', () => {
        segment.curve_percent = Number(slider.value); output.textContent = `곡선 정도 ${segment.curve_percent}%`; changed();
      });
      label.append(output, slider); row.append(label);
      const hint = document.createElement('small'); hint.textContent = '0: 직선에 가깝게 · 100: 크게 휘기'; row.append(hint);
    }
    const remove = document.createElement('button'); remove.type = 'button'; remove.className = 'button ghost';
    remove.textContent = '이 구간 삭제'; remove.disabled = state.busy || state.locked || state.segments.length === 1;
    remove.addEventListener('click', () => {
      state.segments.splice(index, 1);
      if (index > 0 && state.segments[index]) state.segments[index].start_waypoint = state.segments[index-1].end_waypoint;
      renderSegments(); changed();
    });
    row.append(remove); container.append(row);
  });
  if (byId('startNoisePanel')) renderStartNoise();
}

function setSource(source, name) {
  if (source?.schema !== 'ai_worker_waypoints/v1' || !['left', 'right'].includes(source.arm)
      || !Array.isArray(source.waypoints) || source.waypoints.length < 2
      || source.waypoints.some((w) => !Array.isArray(w.position_m) || w.position_m.length !== 3
        || w.position_m.some((v) => typeof v !== 'number' || !Number.isFinite(v)))) {
    throw Error('2개 이상의 지점이 있는 Waypoint JSON을 선택해 주세요.');
  }
  state.source = source;
  if (!integrated) {
    byId('curveArm').value = source.arm;
    byId('curveFileName').textContent = `${name} · ${source.waypoints.length}개 지점`;
  }
  const aug = source.augmentation || {};
  const fromName = (regex) => source.waypoints.findIndex((w) => regex.test(w.name || '')) + 1;
  let defaults = [aug.noise_peak_index_1based || fromName(/노이즈/),
    aug.preinsertion_index_1based || fromName(/접근\s*직전/),
    aug.insertion_index_1based || fromName(/삽입/)];
  if (!defaults.every((n) => Number.isInteger(n) && n >= 1 && n <= source.waypoints.length)
      || !(defaults[0] < defaults[1] && defaults[1] < defaults[2])) {
    defaults = [1, Math.max(2, Math.floor(source.waypoints.length/2)), source.waypoints.length];
  }
  if (defaults[0] < defaults[1] && defaults[1] < defaults[2]) {
    state.segments = [{ start_waypoint: defaults[0], end_waypoint: defaults[1], mode: 'CURVE', curve_percent: 100 },
      { start_waypoint: defaults[1], end_waypoint: defaults[2], mode: 'LINEAR', curve_percent: 0 }];
  } else {
    state.segments = [{ start_waypoint: 1, end_waypoint: source.waypoints.length, mode: 'CURVE', curve_percent: 100 }];
  }
  renderSegments(); byId('addSegment').disabled = false;
  byId('anchorHint').textContent = integrated
    ? '선택한 구간 안의 중간 경로를 다시 만듭니다. 구간 밖의 저장 지점은 유지합니다.'
    : '추천 구간입니다. 끝점과 연결 방식을 확인해 주세요. 구간을 추가하거나 삭제할 수 있습니다. 선택한 구간만 새 경로로 저장합니다.';
  changed();
}

function syncCurrent(data) {
  if (!integrated || !byId('curveNoiseEnabled').checked || state.busy || state.locked
      || !state.source || data.arm !== state.source.arm || !Array.isArray(data.waypoints)) return;
  const waypoints = data.waypoints.map((w) => ({
    name: w.name, position_m: w.position_m, orientation_xyzw: w.orientation_xyzw,
    joint_positions_rad: w.joint_rad, gripper_rad: w.gripper_rad, dwell_s: w.dwell_s,
    speed_scale: w.speed_scale, interpolation_mode: w.interpolation_mode,
    ...(typeof w.is_interpolated === 'boolean' ? { is_interpolated: w.is_interpolated } : {}),
  }));
  if (waypoints.some((w) => !Array.isArray(w.joint_positions_rad))) return;
  if (waypointKey(waypoints) === waypointKey(state.source.waypoints)) return;
  const current = { schema: 'ai_worker_waypoints/v1', arm: data.arm,
    interpolated_provenance_version: data.interpolated_provenance_version || 1,
    joint_smoothing_enabled: data.joint_smoothing_enabled, path_refinement: data.path_refinement,
    start_waypoint: data.start_waypoint, end_waypoint: data.end_waypoint,
    ...(data.joint_timing ? { joint_timing: data.joint_timing } : {}), waypoints };
  if (waypoints.length < 2) {
    state.source = null; state.segments = []; state.result = null;
    byId('curveNoiseEnabled').checked = false; byId('curveEditor').hidden = true;
    renderSegments(); return;
  }
  // After our own Apply, original indices refer to a newly sampled list.
  // Map them to the retained anchors instead of using old intermediate indices.
  if (state.result && waypointKey(waypoints) === waypointKey(state.result.waypoint_file.waypoints)) {
    const info = state.result.waypoint_file.generator;
    const mapping = new Map(info.source_anchor_indices.map((n, i) => [n, info.generated_anchor_indices[i]]));
    state.segments = state.segments.map((s) => ({ ...s,
      start_waypoint: mapping.get(s.start_waypoint), end_waypoint: mapping.get(s.end_waypoint) }));
    // The applied pose becomes the new source. Require an explicit opt-in for
    // another perturbation instead of silently accumulating the same offset.
    if (info.start_noise?.enabled) byId('startNoiseEnabled').checked = false;
    state.source = current; renderSegments(); draw(); return;
  }
  const valid = state.segments.every((s) => [s.start_waypoint, s.end_waypoint]
    .every((n) => Number.isInteger(n) && n >= 1 && n <= waypoints.length));
  if (valid) { state.source = current; renderSegments(); changed(); }
  else setSource(current, '현재 경로');
  byId('curveStatus').textContent = `현재 ${waypoints.length}개 Waypoint 목록에 맞춰 구간 설정을 갱신했습니다.`;
}

function curvePositions(a, b, c, percent, width, angle = 0) {
  const d = sub(b, a), direction = unit(d), tail = unit(sub(c, b));
  const axis = [0, 0, 0]; axis[direction.map(Math.abs).indexOf(Math.min(...direction.map(Math.abs)))] = 1;
  const e1 = unit(cross(direction, axis)), e2 = cross(direction, e1);
  const lateral = add(mul(e1, Math.cos(angle)), mul(e2, Math.sin(angle)));
  const tangent = mul(tail, Math.max(0, dot(d, tail)));
  const bend = (u) => add(mul(sub(tangent, d), -4*u**3+7*u**4-3*u**5),
    mul(lateral, width*64*u**3*(1-u)**3));
  const peak = Math.max(...Array.from({ length: 129 }, (_, i) => norm(bend(i/128))));
  const scale = percent/100 * Math.min(1, width/Math.max(peak*1.001, 1e-12));
  return Array.from({ length: 81 }, (_, i) => add(add(a, mul(d, i/80)), mul(bend(i/80), scale)));
}

function draw() {
  const canvas = byId('curveCanvas'), ctx = canvas.getContext('2d');
  if (!ctx) return;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const original = anchors();
  if (!original || original.some((p) => !Array.isArray(p))) return;
  const selected = original.map((p) => [...p]);
  const s = settings();
  const reference = state.source.waypoints[state.segments[0].start_waypoint-1];
  const sample = sampleStartNoise(s.start_noise, s.seed);
  selected[0] = add(selected[0], mul(sample.translation_mm, .001));
  let startRotation = noiseRotation(reference.orientation_xyzw, sample.rotation_rpy_deg);
  if (state.result) {
    const file = state.result.waypoint_file, indices = file.generator.generated_anchor_indices;
    indices.forEach((n, i) => { selected[i] = [...file.waypoints[n-1].position_m]; });
    startRotation = file.waypoints[indices[0]-1].orientation_xyzw;
  }
  if (selected.some((p, i) => i && norm(sub(p, selected[i-1])) < 1e-5)) return;
  const angle = ((s.seed*1664525+1013904223) >>> 0)/2**32 * 2*Math.PI;
  const paths = state.segments.map((segment, index) => {
    const [a, b] = selected.slice(index, index+2);
    const c = selected[index+2] || add(b, sub(b, a));
    const points = segment.mode === 'CURVE'
      ? curvePositions(a, b, c, segment.curve_percent, s.max_deviation_mm/1000, angle) : [a, b];
    return { mode: segment.mode, points };
  });
  if (state.result) paths.forEach((path, i) => {
    const points = state.result.preview.filter((p) => p.segment === i).map((p) => p.position_m);
    path.points = [selected[i], ...points, selected[i+1]];
  });
  const rotate = (q, v) => {
    const u = q.slice(0, 3), uv = cross(u, v);
    return add(v, add(mul(uv, 2*q[3]), mul(cross(u, uv), 2)));
  };
  const showAxes = s.start_noise.enabled || state.result?.waypoint_file.generator.start_noise?.enabled;
  const poseAxes = showAxes ? [[1,0,0],[0,1,0],[0,0,1]].map((v) => add(selected[0], mul(rotate(startRotation, v), .005))) : [];
  const [x, y] = state.axes, all = [...paths.flatMap((p) => p.points), ...selected, ...poseAxes];
  const xs = all.map((p) => p[x]*1000), ys = all.map((p) => p[y]*1000);
  const xmin = Math.min(...xs), xmax = Math.max(...xs), ymin = Math.min(...ys), ymax = Math.max(...ys);
  const scale = Math.min((canvas.width-130)/Math.max(xmax-xmin, 5), (canvas.height-110)/Math.max(ymax-ymin, 5));
  const project = (p) => [canvas.width/2+(p[x]*1000-(xmin+xmax)/2)*scale,
    canvas.height/2-(p[y]*1000-(ymin+ymax)/2)*scale];
  const line = (points, color, dash = []) => {
    if (!points.length) return;
    ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = 3; ctx.setLineDash(dash);
    points.forEach((p, i) => { const q = project(p); if (i) ctx.lineTo(...q); else ctx.moveTo(...q); }); ctx.stroke(); ctx.setLineDash([]);
  };
  line(anchors(), '#aab5c3', [5, 6]);
  paths.forEach((p) => line(p.points, p.mode === 'CURVE' ? '#246cdd' : '#16865a'));
  poseAxes.forEach((p, i) => line([selected[0], p], ['#d63f39', '#21945a', '#2459c4'][i]));
  selected.forEach((p, i) => {
    const q = project(p); ctx.fillStyle = '#17324d'; ctx.beginPath(); ctx.arc(...q, 5, 0, Math.PI*2); ctx.fill();
    ctx.font = '15px sans-serif'; ctx.fillText(`지점 ${i+1}`, q[0]+9, q[1]+(i === 1 ? -10 : 20));
  });
  ctx.font = '13px sans-serif'; ctx.fillStyle = '#62758c';
  ctx.fillText(`${['X', 'Y', 'Z'][x]} / ${['X', 'Y', 'Z'][y]} · mm`, 18, 25);
}

function changed() {
  renderStartNoise();
  state.result = null;
  byId('curveResult').hidden = true;
  const s = settings();
  byId('curveHint').textContent = state.segments.some((s) => s.mode === 'CURVE')
    ? `곡선 정도 100%에서 최대 ${s.max_deviation_mm.toFixed(1)} mm 휘어집니다. 0%는 직선이며 스무딩은 유지됩니다.`
    : '선택한 구간을 직선으로 연결하고 스무딩을 적용합니다.';
  if (!integrated) byId('createCurves').disabled = state.busy || !anchors();
  byId('curveStatus').dataset.error = 'false';
  byId('curveStatus').textContent = state.source ? (anchors() ? '모양을 확인한 뒤 생성해 주세요.' : '각 구간의 시작점과 끝점을 다르게 선택해 주세요.') : '기준 경로를 선택해 주세요.';
  byId('previewHint').textContent = '예상 모양입니다. 생성 후 검사한 관절 경로의 FK 결과로 바뀝니다.';
  draw();
}

async function jsonRequest(path, body) {
  const response = await fetch(path, body ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {});
  const data = await response.json();
  if (!response.ok || data.success === false) throw Error(data.error || data.message || `HTTP ${response.status}`);
  return data;
}
function error(message) { byId('curveStatus').textContent = message; byId('curveStatus').dataset.error = 'true'; }
byId('curveFile')?.addEventListener('change', async (e) => {
  try { if (e.target.files[0]) setSource(JSON.parse(await e.target.files[0].text()), e.target.files[0].name); }
  catch (e) { error(e.message); }
});
byId('loadCurrent')?.addEventListener('click', async () => {
  try { setSource(await jsonRequest(`/api/waypoints/export?arm=${byId('curveArm').value}`), '현재 Teleop 경로'); }
  catch (e) { error(e.message); }
});
['curveCount', 'curveWidth', 'curveSeed'].forEach((id) => byId(id).addEventListener('input', changed));
byId('addSegment').addEventListener('click', () => {
  if (!state.source || state.busy) return;
  if (state.segments.length >= 32) { error('구간은 한 번에 32개까지 만들 수 있습니다.'); return; }
  const last = state.segments.at(-1), start = last.end_waypoint;
  if (start === state.source.waypoints.length) {
    if (last.end_waypoint-last.start_waypoint <= 1) {
      error('뒤에 추가할 지점이 없습니다. 마지막 구간의 끝점을 앞쪽 지점으로 바꾼 뒤 추가해 주세요.'); return;
    }
    const middle = Math.floor((last.start_waypoint+last.end_waypoint)/2);
    last.end_waypoint = middle;
    state.segments.push({ start_waypoint: middle, end_waypoint: start,
      mode: last.mode, curve_percent: last.curve_percent });
  } else {
    state.segments.push({ start_waypoint: start, end_waypoint: start+1, mode: 'CURVE', curve_percent: 100 });
  }
  renderSegments(); changed();
});
[['viewXZ', [0, 2]], ['viewYZ', [1, 2]], ['viewXY', [0, 1]]].forEach(([id, axes]) => byId(id).addEventListener('click', () => {
  state.axes = axes;
  ['viewXZ', 'viewYZ', 'viewXY'].forEach((name) => byId(name).setAttribute('aria-pressed', String(name === id)));
  draw();
}));
async function generateCurve(arm) {
  if (state.busy || !anchors()) throw Error('만들 구간을 먼저 선택해 주세요.');
  for (const id of ['curveCount', 'curveWidth', 'curveSeed']) {
    if (!byId(id).reportValidity()) throw Error('고급 설정의 입력 범위를 확인해 주세요.');
  }
  for (const input of byId('startNoisePanel').querySelectorAll('input')) {
    if (!input.checkValidity()) {
      byId('startNoiseNumbers').open = true; input.reportValidity();
      throw Error('시작점 노이즈의 최소·최대 범위를 확인해 주세요.');
    }
  }
  if (integrated) {
    const current = await jsonRequest(`/api/waypoints/export?arm=${arm}`);
    if (current.arm !== state.source.arm || waypointKey(current.waypoints) !== waypointKey(state.source.waypoints)) {
      setSource(current, '현재 경로');
      throw Error('경로가 변경되어 구간을 갱신했습니다. 구간을 확인하고 다시 적용해 주세요.');
    }
  }
  const sourceKey = waypointKey(state.source.waypoints);
  state.busy = true;
  state.result = null;
  byId('curveResult').hidden = true;
  controls.forEach((id) => { if (byId(id)) byId(id).disabled = true; }); renderSegments();
  if (!integrated) byId('createCurves').disabled = true;
  byId('curveStatus').dataset.error = 'false';
  byId('curveStatus').textContent = '곡선 생성 → IK → JOINT 변환 → 경로 검사 중입니다.\n여러 경로를 만들면 시간이 더 걸립니다.';
  try {
    const result = await jsonRequest('/api/generator/curve-path', settings());
    state.result = result;
    byId('curveStatus').textContent = result.message;
    byId('curveResult').hidden = false;
    byId('curveResultText').textContent = `${result.files.length}개 경로 검사 완료 · 최대 직선 오차 ${Math.max(...result.files.map((f) => f.max_straight_error_mm)).toFixed(4)} mm`
      + (integrated ? ' · 첫 번째 경로를 적용합니다.' : '');
    byId('saveCurve').textContent = result.files.length > 1 ? '첫 번째 경로 JSON 저장' : 'Waypoint JSON 저장';
    byId('saveCurveZip').textContent = `${result.files.length}개 경로 전체 저장 (ZIP)`;
    byId('curveDiagnostics').textContent = JSON.stringify(result.files, null, 2);
    byId('previewHint').textContent = '검사한 JOINT 경로의 FK 결과입니다. 여러 경로를 생성한 경우 첫 번째 경로를 표시합니다.';
    draw();
    if (integrated) {
      const current = await jsonRequest(`/api/waypoints/export?arm=${arm}`);
      if (window.getWaypointCurveArm() !== arm || waypointKey(current.waypoints) !== sourceKey) {
        throw Error('계산 중 경로 또는 팔이 변경되어 목록에 적용하지 않았습니다. 구간을 확인하고 다시 계산해 주세요.');
      }
    }
    return result.waypoint_file;
  } catch (e) { error(e.message); throw e; }
  finally {
    state.busy = false;
    controls.forEach((id) => { if (byId(id)) byId(id).disabled = state.locked; }); renderSegments();
    if (!integrated) byId('createCurves').disabled = !anchors();
  }
}
byId('createCurves')?.addEventListener('click', () => generateCurve().catch(() => {}));
function save(blob, filename) {
  const url = URL.createObjectURL(blob), a = document.createElement('a');
  a.href = url; a.download = filename; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
}
byId('saveCurve').addEventListener('click', () => {
  if (state.result) save(new Blob([JSON.stringify(state.result.waypoint_file, null, 2)], { type: 'application/json' }), `${state.source.arm}_curve_01.json`);
});
byId('saveCurveZip').addEventListener('click', () => {
  if (!state.result) return;
  const raw = atob(state.result.archive_base64);
  save(new Blob([Uint8Array.from(raw, (c) => c.charCodeAt(0))], { type: 'application/zip' }), `${state.source.arm}_curves.zip`);
});
makeStartNoiseControls();
if (integrated) {
  byId('curveNoiseEnabled').addEventListener('change', async (event) => {
    byId('curveEditor').hidden = !event.target.checked;
    if (!event.target.checked) return;
    byId('refinementEnabled').checked = true;
    byId('refinementEnabled').dispatchEvent(new Event('input'));
    try {
      const arm = window.getWaypointCurveArm();
      setSource(await jsonRequest(`/api/waypoints/export?arm=${arm}`), '현재 경로');
    } catch (e) { error(e.message); }
  });
  window.waypointCurve = {
    generate: generateCurve,
    syncCurrent,
    setLocked(locked, arm) {
      if (state.source && state.source.arm !== arm) {
        byId('curveNoiseEnabled').checked = false; byId('curveEditor').hidden = true;
        state.source = null; state.segments = []; state.result = null;
      }
      if (state.locked === locked) return;
      state.locked = locked;
      controls.forEach((id) => { if (byId(id)) byId(id).disabled = locked || state.busy; });
      renderSegments();
    },
  };
}
changed();
})();
