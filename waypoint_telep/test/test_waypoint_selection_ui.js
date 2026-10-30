// NODE_PATH must provide jsdom. Network and all robot actions are mocked.
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const { JSDOM } = require('jsdom');
const root = process.argv[2] || path.join(__dirname, '../web');
const dom = new JSDOM(fs.readFileSync(path.join(root, 'index.html'), 'utf8'),
  { runScripts: 'outside-only', url: 'http://localhost/' });
const w = dom.window, d = w.document, posts = [], errors = [];
const id = (name) => d.getElementById(name);
const tick = () => new Promise((resolve) => setImmediate(resolve));
w.HTMLCanvasElement.prototype.getContext = () => null;
w.addEventListener('error', (event) => errors.push(event.error || event.message));
w.confirm = () => true;
let finishDelete;
w.fetch = async (url, options) => {
  posts.push([url, JSON.parse(options.body)]);
  if (url === '/api/waypoints/remove-range') await new Promise((resolve) => { finishDelete = resolve; });
  return { ok: true, json: async () => ({ success: true, message: '완료' }) };
};
const source = fs.readFileSync(path.join(root, 'app.js'), 'utf8');
w.eval(source.slice(0, source.lastIndexOf('\nrefresh();')) + `
  refresh = async () => {};
  window.selectionTest = {state, renderPosePanels, renderExecutionSequence, renderExecutionRangeControls, waypointPlanMode};
`);
const api = w.selectionTest;
function data(arm) {
  return { arm, waypoint_count: 5, path_mode: 'JOINT', world_frame: 'odom',
    max_waypoint_dwell_s: 30, execution: { active: false }, keyboard_control: { active: false },
    publish_enabled: false, execution_sequence: [], current_deg: Array(8).fill(null),
    waypoints: Array.from({ length: 5 }, (_, i) => ({ index: i+1,
      name: i === 3 ? '구간 1 직선 중간점 4' : `지점 ${i+1}`,
      ...(i === 1 ? { is_interpolated: true } : {}),
      position_m: [.5, 0, i*.01], orientation_xyzw: [0, 0, 0, 1],
      joint_rad: Array(8).fill(i*.01), gripper_rad: .9, gripper_deg: 51.6,
      dwell_s: 0, speed_scale: 1, interpolation_mode: 'JOINT' })),
  };
}
function render() { api.renderPosePanels(api.state.data); api.renderExecutionSequence(api.state.data); }
function change(name, value) {
  if (typeof value === 'boolean') id(name).checked = value;
  else id(name).value = String(value);
  id(name).dispatchEvent(new w.Event('change', { bubbles: true }));
}
function indices(name) { return Array.from(id(name).options, (option) => Number(option.value)); }
function point(index) { return d.querySelector(`.waypoint-item[data-index="${index}"]`); }
(async () => {
  const left = data('left'), right = data('right');
  const mixed = data('left');
  mixed.joint_smoothing_enabled = true;
  mixed.waypoints[4].interpolation_mode = 'LINEAR';
  assert.equal(api.waypointPlanMode(mixed), 'MIXED');
  mixed.start_waypoint = 3; mixed.end_waypoint = 1;
  assert.equal(api.waypointPlanMode(mixed), 'JOINT_CONTINUOUS', 'only selected endpoints determine the active interpolation mode');
  api.state.armData = { left, right }; api.state.data = left;
  render();
  assert.equal(d.querySelectorAll('.waypoint-item').length, 5);
  id('sequenceStart').value = '2'; id('sequenceEnd').value = '4';
  change('hideInterpolatedSequence', true);
  assert.equal(id('hideInterpolatedList').checked, true);
  assert.deepEqual(indices('sequenceStart'), [1, 3, 5]);
  assert.equal(id('sequenceStart').value, '1');
  assert.equal(id('sequenceEnd').value, '5');
  assert.equal(d.querySelectorAll('.waypoint-item').length, 3);
  const option = id('sequenceStart').firstElementChild;
  render();
  assert.equal(id('sequenceStart').firstElementChild, option, 'polling must preserve an open endpoint selector');
  id('sequenceStart').value = '5'; id('sequenceEnd').value = '1';
  id('addSequenceStep').click(); await tick();
  assert.deepEqual(posts.pop(), ['/api/sequence/add', { arm: 'left', start_index: 5, end_index: 1 }]);

  id('deleteRangeList').click();
  assert.equal(id('deleteRangePanel').hidden, false);
  assert.equal(point(1).draggable, false);
  point(1).click(); point(5).click();
  assert.equal(id('deleteRangeStart').value, '1');
  assert.equal(id('deleteRangeEnd').value, '5');
  assert.equal(d.querySelectorAll('.delete-selected').length, 3);
  assert.ok(id('deleteRangeSummary').textContent.includes('5개 삭제'));
  assert.equal(posts.length, 0, 'selection must never preview or execute');
  change('deleteRangeStart', 5); change('deleteRangeEnd', 3);
  assert.ok(id('deleteRangeSummary').textContent.includes('3개 삭제'));
  w.confirm = () => false;
  id('confirmDeleteRange').click(); await tick();
  assert.equal(posts.length, 0);
  w.confirm = () => true;
  id('confirmDeleteRange').click(); id('confirmDeleteRange').click(); await tick();
  assert.deepEqual(posts, [['/api/waypoints/remove-range', { arm: 'left', start_index: 3, end_index: 5, expected_count: 5 }]]);
  assert.equal(id('confirmDeleteRange').disabled, true);
  finishDelete(); await tick(); await tick();
  assert.equal(api.state.deleteRange.left, null);
  posts.length = 0;

  render(); id('deleteRange').click(); point(3).click(); point(1).click();
  assert.ok(id('deleteRangeSummary').textContent.includes('#1–#3'));
  id('cancelDeleteRange').click();
  assert.equal(id('deleteRangePanel').hidden, true);
  id('deleteRange').click(); left.waypoints[0].name = '외부 수정'; render();
  assert.equal(id('deleteRangePanel').hidden, true, 'a changed source invalidates deletion endpoints');
  left.execution.active = true; render();
  assert.equal(id('deleteRange').disabled, true);
  assert.equal(id('addSequenceStep').disabled, true);
  left.execution.active = false;
  left.keyboard_control.active = true; render();
  assert.equal(id('deleteRangeList').disabled, true);

  api.state.activeArm = 'right'; api.state.data = right;
  right.waypoints[3].is_interpolated = false; // Explicit provenance overrides a legacy-looking name.
  id('sequenceArm').value = 'right'; render();
  assert.deepEqual(indices('sequenceStart'), [1, 3, 4, 5]);
  id('deleteRange').click(); point(5).click(); point(3).click();
  assert.equal(api.state.deleteRange.right.start, 5);
  assert.equal(api.state.deleteRange.left, null);
  change('hideInterpolatedList', false);
  assert.deepEqual(indices('sequenceStart'), [1, 2, 3, 4, 5]);
  assert.equal(id('hideInterpolatedSequence').checked, false);
  assert.equal(posts.length, 0);
  // Legacy endpoint files were previously exported with every flag False.
  api.state.activeArm = 'left'; api.state.data = left;
  id('sequenceArm').value = 'left';
  left.keyboard_control.active = false;
  left.start_waypoint = 2; left.end_waypoint = 5;
  left.waypoints[0].name = '왼팔 지점 1';
  left.waypoints[1].name = '왼팔 지점 1-보간1 (1→2)';
  left.waypoints[2].name = '왼팔 지점 1-보간2 (1→2)';
  left.waypoints[3].name = '왼팔 지점 6-직선보간1';
  left.waypoints[4].name = '왼팔 지점 7';
  left.waypoints.forEach((point) => { point.is_interpolated = false; });
  api.state.relativeReferences.left = '2';
  change('hideInterpolatedList', true);
  assert.equal(d.querySelectorAll('.waypoint-item').length, 2);
  assert.deepEqual(indices('sequenceStart'), [1, 5]);
  assert.deepEqual(indices('startWaypointSelect'), [1, 2, 5], 'keep a committed hidden start until explicitly changed');
  assert.deepEqual(indices('endWaypointSelect'), [1, 5]);
  assert.equal(id('startWaypointSelect').value, '2');
  assert.deepEqual(Array.from(id('relativeReferenceSelect').options, (o) => o.value), ['last', 'current', '1', '2', '5']);
  assert.deepEqual(Array.from(id('frameRelativeReferenceSelect').options, (o) => o.value), ['last', 'current', '1', '5']);
  const rangeOption = id('startWaypointSelect').firstElementChild;
  api.renderExecutionRangeControls(left);
  assert.equal(id('startWaypointSelect').firstElementChild, rangeOption);
  left.interpolated_provenance_version = 2;
  render(); api.renderExecutionRangeControls(left);
  assert.equal(d.querySelectorAll('.waypoint-item').length, 5, 'new explicit original flags must remain authoritative');
  left.waypoints[1].is_interpolated = true;
  left.waypoints[1].name = '사용자가 바꾼 보간점 이름';
  render();
  assert.equal(d.querySelectorAll('.waypoint-item').length, 4, 'renaming must preserve generated provenance');
  assert.equal(posts.length, 0, 'display filtering must not mutate any plan or execution range');
  assert.deepEqual(errors, []);
  console.log('Waypoint selection UI: inclusive range deletion, hidden intermediates, stable selectors, original indices, reverse sequences, per-arm state and execution guards: PASS');
})().catch((error) => { console.error(error); process.exitCode = 1; }).finally(() => dom.window.close());
