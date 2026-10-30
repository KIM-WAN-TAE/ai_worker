// NODE_PATH must provide jsdom. All HTTP and robot actions are mocked.
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const { JSDOM } = require('jsdom');
const root = process.argv[2] || path.join(__dirname, '../web');
const tick = () => new Promise((resolve) => setImmediate(resolve));
const source = {
  schema: 'ai_worker_waypoints/v1', arm: 'right',
  augmentation: { noise_peak_index_1based: 1, preinsertion_index_1based: 3, insertion_index_1based: 4 },
  waypoints: Array.from({ length: 6 }, (_, i) => ({ name: `지점 ${i+1}`,
    position_m: [.5-i*.001, .02, -.14-i*.01], orientation_xyzw: [0, 0, 0, 1],
    joint_positions_rad: [0, -.6, .15, -1, .2, .3, .1, .9],
    gripper_rad: .9, dwell_s: 0, speed_scale: 1, interpolation_mode: 'JOINT' })),
};

async function check(integrated) {
  const dom = new JSDOM(fs.readFileSync(path.join(root, integrated ? 'index.html' : 'curve_generator.html'), 'utf8'),
    { runScripts: 'outside-only', url: 'http://localhost/' });
  const w = dom.window, d = w.document;
  const id = (name) => d.getElementById(name);
  w.HTMLCanvasElement.prototype.getContext = () => null;
  w.getWaypointCurveArm = () => 'right';
  const calls = [];
  let exported = structuredClone(source);
  let resample = false;
  w.fetch = async (url, options) => {
    calls.push([url, options?.body ? JSON.parse(options.body) : null]);
    if (url.startsWith('/api/waypoints/export')) return { ok: true, json: async () => structuredClone(exported) };
    const body = JSON.parse(options.body);
    const file = { ...exported, generator: { validation: {},
      source_anchor_indices: [body.segments[0].start_waypoint, ...body.segments.map((s) => s.end_waypoint)],
      generated_anchor_indices: [body.segments[0].start_waypoint, ...body.segments.map((s) => s.end_waypoint)] } };
    if (resample) {
      file.waypoints = [exported.waypoints[0], { ...exported.waypoints[0], name: '중간점 A' },
        exported.waypoints[1], { ...exported.waypoints[1], name: '중간점 B' }, exported.waypoints[2]];
      file.generator.generated_anchor_indices = [1, 3, 5];
    }
    if (body.start_noise?.enabled) file.generator.start_noise = body.start_noise;
    return { ok: true, json: async () => ({ success: true, message: '검사 완료',
      waypoint_file: file, preview: [],
      files: [{ passed: true, max_straight_error_mm: .1 }], archive_base64: '' }) };
  };
  w.eval(fs.readFileSync(path.join(root, 'curve_generator.js'), 'utf8'));
  if (integrated) {
    assert.equal(id('curveEditor').hidden, true);
    assert.equal(id('curveNoiseEnabled').checked, false);
    id('curveNoiseEnabled').checked = true;
    id('curveNoiseEnabled').dispatchEvent(new w.Event('change'));
    await tick();
    assert.equal(id('refinementEnabled').checked, true);
  } else {
    id('loadCurrent').click(); await tick();
  }
  assert.equal(d.querySelectorAll('.curve-segment').length, 2);
  assert.equal(calls.filter(([url]) => url.includes('curve-path')).length, 0);
  assert.equal(id('startNoiseEnabled').checked, false);
  assert.equal(id('startNoiseBody').hidden, true);
  assert.equal(id('startNoiseNumbers').open, false, 'numeric editing stays in advanced settings');
  id('startNoiseEnabled').checked = true;
  id('startNoiseEnabled').dispatchEvent(new w.Event('change'));
  assert.equal(id('startNoiseRotation').hidden, true);
  const change = (name, value) => { id(name).value = String(value); id(name).dispatchEvent(new w.Event('input')); };
  change('startNoisePositionXLowerHandle', -2.2);
  change('startNoisePositionXUpperHandle', 3.3);
  assert.equal(Number(id('startNoisePositionXMin').value), -2.2);
  assert.equal(Number(id('startNoisePositionXMax').value), 3.3);
  change('startNoisePositionX', 1.2);
  id('startNoisePositionX').dispatchEvent(new w.WheelEvent('wheel', { deltaY: -100, cancelable: true }));
  assert.equal(Number(id('startNoisePositionX').value), 1.3);
  id('startNoiseMode').value = 'both'; id('startNoiseMode').dispatchEvent(new w.Event('change'));
  change('startNoiseRotationYaw', 2.4);
  const generate = async () => {
    if (integrated) return w.waypointCurve.generate('right');
    id('createCurves').click(); await tick(); await tick();
  };
  await generate();
  let noise = calls.findLast(([url]) => url.includes('curve-path'))[1].start_noise;
  assert.equal(noise.mode, 'both');
  assert.equal(noise.translation_mm[0], 1.3);
  assert.equal(noise.rotation_rpy_deg[2], 2.4);
  id('startNoiseRandom').checked = true; id('startNoiseRandom').dispatchEvent(new w.Event('change'));
  assert.equal(Number(id('curveCount').value), 10);
  assert.equal(id('startNoisePositionX').disabled, true, 'random sampling displays values without editing the manual sample');
  await generate();
  noise = calls.findLast(([url]) => url.includes('curve-path'))[1].start_noise;
  assert.equal(noise.sampling, 'random');
  assert.equal(noise.translation_min_mm[0], -2.2);
  assert.equal(noise.translation_max_mm[0], 3.3);
  assert.ok(id('startNoiseSummary').textContent.includes('첫 번째 샘플'));
  assert.equal(Number(id('startNoisePositionX').value).toFixed(2), id('startNoisePositionXValue').textContent);
  id('startNoiseEnabled').checked = false; id('startNoiseEnabled').dispatchEvent(new w.Event('change'));
  change('curveCount', 1);
  id('addSegment').click(); id('addSegment').click();
  assert.equal(d.querySelectorAll('.curve-segment').length, 4, 'arbitrary segment count');
  const first = d.querySelector('.curve-segment input[type="range"]');
  first.value = '0'; first.dispatchEvent(new w.Event('input'));
  if (integrated) {
    // Run the real unified Apply handler with no real HTTP/ROS side effects.
    const app = fs.readFileSync(path.join(root, 'app.js'), 'utf8');
    const block = app.slice(app.indexOf("byId('applyRefinement').addEventListener"),
      app.indexOf("byId('downloadRefinementReport').addEventListener"));
    w.eval(`
      const state = { activeArm: 'right', busy: false, refinementBusy: false,
        refinementDrafts: {right:{enabled:true,smoothing_percent:100}}, armData:{right:{}} };
      const byId = id => document.getElementById(id);
      const refinementFields = {enabled:'refinementEnabled'};
      window.applied = []; window.previews = 0; window.errors = [];
      const request = async (url, options) => { window.applied.push([url, JSON.parse(options.body)]); };
      const refresh = async () => {};
      const renderRefinementControls = () => {};
      const runAction = async () => { window.previews++; };
      const showToast = msg => window.errors.push(msg);
      ${block}
    `);
    id('applyRefinement').click(); await tick(); await tick();
    assert.equal(w.errors.length, 0, w.errors.join(';'));
    assert.deepEqual(Array.from(w.applied, (x) => x[0]), ['/api/waypoints/import', '/api/path-refinement']);
    assert.equal(w.previews, 1);
    const generation = calls.findLast(([url]) => url.includes('curve-path'))[1];
    assert.equal(generation.preserve_outside, true);
    assert.equal(generation.segments.length, 4);
    assert.equal(generation.segments[0].curve_percent, 0);
    id('curveNoiseEnabled').checked = false;
    id('curveNoiseEnabled').dispatchEvent(new w.Event('change'));
    const before = calls.length;
    w.applied = [];
    id('applyRefinement').click(); await tick();
    assert.equal(calls.length, before, 'noise off uses the original refinement flow');
    assert.deepEqual(Array.from(w.applied, (x) => x[0]), ['/api/path-refinement']);
    // Editing a previously loaded list must update segment indices before Apply.
    id('curveNoiseEnabled').checked = true;
    id('curveNoiseEnabled').dispatchEvent(new w.Event('change')); await tick();
    exported = { ...source, waypoints: source.waypoints.slice(0, 3) };
    w.waypointCurve.syncCurrent({ ...exported, waypoint_count: 3,
      waypoints: exported.waypoints.map((p) => ({ ...p, joint_rad: p.joint_positions_rad })) });
    assert.equal(d.querySelectorAll('.curve-segment').length, 2);
    assert.equal(d.querySelectorAll('.curve-segment')[0].querySelectorAll('select')[1].value, '2');
    assert.equal(d.querySelectorAll('.curve-segment')[1].querySelectorAll('select')[1].value, '3');
    assert.ok(id('curveStatus').textContent.includes('3개'));
    // Object key order is irrelevant to whether waypoints have changed.
    exported.waypoints = exported.waypoints.map((p) => Object.fromEntries(Object.entries(p).reverse()));
    await w.waypointCurve.generate('right');
    assert.equal(calls.findLast(([url]) => url.includes('curve-path'))[1].waypoint_file.waypoints.length, 3);
    resample = true;
    id('startNoiseEnabled').checked = true; id('startNoiseEnabled').dispatchEvent(new w.Event('change'));
    exported = await w.waypointCurve.generate('right');
    w.waypointCurve.syncCurrent({ ...exported, waypoint_count: 5,
      waypoints: exported.waypoints.map((p) => ({ ...p, joint_rad: p.joint_positions_rad })) });
    assert.equal(d.querySelectorAll('.curve-segment')[0].querySelectorAll('select')[1].value, '3');
    assert.equal(d.querySelectorAll('.curve-segment')[1].querySelectorAll('select')[1].value, '5');
    assert.equal(id('startNoiseEnabled').checked, false, 'applying noise must not accidentally accumulate it on the next Apply');
    w.waypointCurve.setLocked(true, 'right');
    assert.ok(Array.from(d.querySelectorAll('.curve-segment select')).every((s) => s.disabled));
    w.waypointCurve.setLocked(false, 'left');
    assert.equal(id('curveNoiseEnabled').checked, false, 'arm change clears unreviewed segment edits');
  } else {
    id('createCurves').click(); await tick(); await tick();
    assert.equal(id('curveResult').hidden, false);
    assert.equal(calls.at(-1)[1].preserve_outside, false);
    assert.equal(calls.at(-1)[1].segments.length, 4);
    assert.equal(calls.at(-1)[1].segments[0].curve_percent, 0);
  }
  dom.window.close();
}
(async () => {
  await check(false); await check(true);
  console.log('Curve UI: draggable noise bounds and pose, wheel control, random start poses, segment editing, unified Apply, source refresh, execution lock and arm switch: PASS');
})().catch((e) => { console.error(e); process.exitCode = 1; });
