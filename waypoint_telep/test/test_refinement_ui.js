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
const buttons = [0, 30, 60, 100].map((strength) => {
  const button = element(`preset${strength}`);
  button.dataset.smoothing = String(strength);
  return button;
});
const context = vm.createContext({
  assert, console, byId: element,
  document: { activeElement: null, querySelectorAll: () => buttons },
  Event: class { constructor(type) { this.type = type; } },
});
const block = source.slice(source.indexOf('const refinementFields ='),
  source.indexOf("byId('jointSmoothing').addEventListener"));
vm.runInContext(`
  const state = {
    activeArm: 'left', refinementDrafts: {left: null, right: null},
    refinementApplied: {left: null, right: null}, refinementBusy: false, busy: false,
  };
  ${block}
  const left = {arm: 'left', path_refinement: {enabled: true, smoothing_percent: 60}};
  const right = {arm: 'right', path_refinement: {enabled: false, smoothing_percent: 100}};
  renderRefinementControls(left);
  assert.equal(Number(byId('refinementStrength').value), 60);
  assert.equal(Number(byId('refinementSubdivisions').value), 9);
  byId('refinementSubdivisions').value = '0';
  byId('refinementSubdivisions').dispatchEvent(new Event('input'));
  assert.equal(state.refinementDrafts.left.subdivisions, 1);
  byId('refinementStrengthNumber').value = '45';
  byId('refinementStrengthNumber').dispatchEvent(new Event('input'));
  assert.equal(Number(byId('refinementStrength').value), 45);
  assert.equal(state.refinementDrafts.left.smoothing_percent, 45);
  document.querySelectorAll('[data-smoothing]')[2].listeners.click();
  assert.equal(state.refinementDrafts.left.smoothing_percent, 60);
  byId('refinementStrength').value = '0';
  byId('refinementStrength').dispatchEvent(new Event('input'));
  assert.equal(byId('refinementStrengthValue').textContent, '0%');
  assert.ok(byId('refinementStrengthHelp').textContent.includes('정지'));
  state.activeArm = 'right';
  renderRefinementControls(right);
  assert.equal(Number(byId('refinementStrength').value), 100);
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
  assert.ok(byId('refinementMonitor').textContent.includes('속도를 낮추'));
  assert.ok(byId('refinementMonitor').textContent.includes('waypoint 3'));
  assert.ok(byId('refinementMonitor').textContent.includes('arm_l_joint4'));
`, context);
console.log('Refinement UI: strength, presets, per-arm drafts, import sync, execution guards, diagnostics passed.');
