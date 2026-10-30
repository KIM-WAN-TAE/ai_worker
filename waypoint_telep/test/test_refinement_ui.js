// Run with Node.js; no browser or third-party packages required.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const source = fs.readFileSync(process.argv[2] || `${__dirname}/../web/app.js`, 'utf8');
const nodes = new Map();
function element(id) {
  if (!nodes.has(id)) nodes.set(id, {
    value: '', checked: false, textContent: '', dataset: {}, listeners: {}, attributes: {},
    addEventListener(type, listener) { this.listeners[type] = listener; },
    dispatchEvent(event) { this.listeners[event.type]?.({ target: this }); },
    setAttribute(key, value) { this.attributes[key] = value; },
    checkValidity() { return true; },
  });
  return nodes.get(id);
}
const context = vm.createContext({
  assert, console, byId: element, window: {},
  document: { activeElement: null },
  Event: class { constructor(type) { this.type = type; } },
});
const block = source.slice(source.indexOf('const refinementFields ='),
  source.indexOf("byId('jointSmoothing').addEventListener"));
vm.runInContext(`
  const state = {
    activeArm: 'left', refinementDrafts: {left: null, right: null},
    refinementApplied: {left: null, right: null}, refinementBusy: false, busy: false, armData: {},
  };
  ${block}
  const left = {arm: 'left', path_refinement: {enabled: true, smoothing_percent: 60}};
  const right = {arm: 'right', path_refinement: {enabled: false, smoothing_percent: 100}};
  state.armData = {left, right};
  renderRefinementControls(left);
  assert.equal(byId('refinementAutomatic').checked, true);
  assert.equal(byId('refinementManualPoints').hidden, true);
  assert.equal(byId('refinementSubdivisions').disabled, true);
  byId('refinementAutomatic').checked = false;
  byId('refinementAutomatic').dispatchEvent(new Event('input'));
  assert.equal(state.refinementDrafts.left.automatic_subdivisions, false);
  assert.equal(byId('refinementManualPoints').hidden, false);
  assert.equal(byId('refinementSubdivisions').disabled, false);
  assert.equal(Number(byId('refinementStrength').value), 60);
  assert.equal(Number(byId('refinementSubdivisions').value), 9);
  byId('refinementSubdivisions').value = '0';
  byId('refinementSubdivisions').dispatchEvent(new Event('input'));
  assert.equal(state.refinementDrafts.left.subdivisions, 1);
  byId('refinementStrengthNumber').value = '45';
  byId('refinementStrengthNumber').dispatchEvent(new Event('input'));
  assert.equal(Number(byId('refinementStrength').value), 45);
  assert.equal(state.refinementDrafts.left.smoothing_percent, 45);
  assert.ok(byId('refinementConnectionHint').textContent.includes('45%'));
  byId('refinementStrength').value = '0';
  byId('refinementStrength').dispatchEvent(new Event('input'));
  assert.equal(byId('refinementStrengthValue').textContent, '0%');
  assert.ok(byId('refinementStrengthHelp').textContent.includes('정지'));
  assert.ok(byId('refinementConnectionHint').textContent.includes('정지'));
  state.activeArm = 'right';
  renderRefinementControls(right);
  assert.equal(Number(byId('refinementStrength').value), 100);
  assert.ok(byId('refinementConnectionHint').textContent.includes('최대한'));
  state.activeArm = 'left';
  renderRefinementControls(left);
  assert.equal(Number(byId('refinementStrength').value), 0);
  left.path_refinement.smoothing_percent = 30;
  renderRefinementControls(left);
  assert.equal(Number(byId('refinementStrength').value), 30);
  left.execution = {active: true};
  renderRefinementControls(left);
  assert.equal(byId('refinementStrength').disabled, true);
  assert.equal(byId('applyRefinement').disabled, true);
  left.path_refinement_events = [{stage: 'failed', attempt: 2, subdivisions: 20,
    max_position_error_mm: 2, max_orientation_error_deg: 0.2, max_joint_error_rad: 0.01,
    reason_code: 'JOINT_STEP', reason: 'arm_l_joint4 delta=0.2', segment_end_waypoint: 3}];
  renderRefinementMonitor(left);
  assert.equal(byId('refinementMonitor').dataset.stage, 'failed');
  assert.ok(byId('refinementMonitor').textContent.includes('속도를 낮춰'));
  assert.ok(byId('refinementMonitor').textContent.includes('3번 지점'));
  assert.ok(!byId('refinementMonitor').textContent.includes('arm_l_joint4'));
  assert.ok(byId('refinementDiagnostics').textContent.includes('arm_l_joint4'));
  left.trajectory = {sample_count: 10};
  left.path_refinement_events = [{stage: 'success', automatic_subdivisions: true,
    segment_subdivisions: [2, 15], knot_count: 18, smoothing_percent: 100, effective_smoothing_percent: 50}];
  renderRefinementMonitor(left);
  assert.ok(byId('refinementMonitor').textContent.includes('준비 완료'));
  assert.ok(byId('refinementMonitor').textContent.includes('자동으로 낮췄'));
  assert.ok(byId('refinementDiagnostics').textContent.includes('2, 15'));
  state.refinementErrors = { left: '경로가 변경되었습니다.' };
  renderRefinementMonitor(left);
  assert.equal(byId('refinementMonitor').dataset.stage, 'failed');
  assert.ok(byId('refinementMonitor').textContent.includes('경로가 변경되었습니다.'));
  renderRefinementMonitor(left);
  assert.ok(byId('refinementMonitor').textContent.includes('경로가 변경되었습니다.'));
`, context);
console.log('Refinement UI: advanced strength, custom-setting summary, per-arm drafts, import sync, execution guards, diagnostics passed.');
