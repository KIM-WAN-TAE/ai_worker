// End-to-end UI request routing; all network and robot actions are mocked.
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const { JSDOM } = require('jsdom');
const root = process.argv[2] || path.join(__dirname, '../web');
const dom = new JSDOM(fs.readFileSync(path.join(root, 'index.html'), 'utf8'),
  { runScripts: 'outside-only', url: 'http://localhost/' });
const w = dom.window, d = w.document, posts = [];
w.HTMLCanvasElement.prototype.getContext = () => null;
let verified;
w.fetch = async (url, options) => {
  if (options?.method === 'POST') posts.push([url, JSON.parse(options.body)]);
  return { ok: true, json: async () => options?.method === 'POST'
    ? { success: true, message: '끝단 등속 시간 배정 완료' } : verified };
};
const source = fs.readFileSync(path.join(root, 'app.js'), 'utf8');
w.eval(source.slice(0, source.lastIndexOf('\nrefresh();')) + `
  renderState = () => {};
  window.uniformTest = { state, renderEqualTimeControls, equalizeWaypointTimes };
`);
const api = w.uniformTest;
const id = (name) => d.getElementById(name);
(async () => {
  verified = { arm: 'right', waypoint_count: 3, waypoints: [{}, {}, {}],
    equal_segment_time_supported: true, equal_total_time_supported: true,
    uniform_total_time_supported: true, execution: { active: false }, keyboard_control: { active: false },
    joint_timing: { speed_mps: .02, preserve_start_waypoint: 0, preserve_end_waypoint: 0 } };
  api.state.activeArm = 'right'; api.state.data = verified; api.state.armData.right = verified;
  api.renderEqualTimeControls(verified);
  assert.equal(id('equalTimeExcluded').value, '', 'whole-path timing must not produce an invalid 0-0 exclusion');
  assert.equal(id('equalTimeAll').disabled, false);
  api.state.equalTimes.right.seconds = '180';
  await api.equalizeWaypointTimes(false);
  assert.deepEqual(posts.shift(), ['/api/waypoints/equal-time', {
    arm: 'right', excluded_waypoints: [], distribution: 'uniform', total_duration_s: 180 }]);
  verified.path_refinement = { enabled: true };
  api.renderEqualTimeControls(verified);
  assert.equal(id('equalTimeAll').disabled, true, 'older servers must not claim combined support');
  assert.ok(id('equalTimeResult').textContent.includes('부드럽게 연결'));
  verified.uniform_refinement_supported = true;
  api.renderEqualTimeControls(verified);
  assert.equal(id('equalTimeAll').disabled, false, 'refined curves can be timed without disabling smoothing');
  api.state.equalTimes.right.mode = 'segment';
  api.state.equalTimes.right.seconds = '1';
  await api.equalizeWaypointTimes(false);
  assert.deepEqual(posts.shift(), ['/api/waypoints/equal-time', {
    arm: 'right', excluded_waypoints: [], duration_s: 1 }]);
  api.state.equalTimes.right.mode = 'total';
  verified.path_refinement.enabled = false;
  delete verified.uniform_total_time_supported;
  api.renderEqualTimeControls(verified);
  assert.equal(id('equalTimeAll').disabled, true, 'an old server must never silently use equal segment times');
  assert.ok(id('equalTimeResult').textContent.includes('재시작'));
  console.log('Uniform timing UI: TCP distribution routing, legacy mode, empty preservation and old-server guard: PASS');
})().catch(error => { console.error(error); process.exitCode = 1; }).finally(() => dom.window.close());
