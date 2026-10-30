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
w.confirm = () => { throw new Error('File registration should not ask to replace the plan'); };
let release, fail = false, refreshes = 0;
w.fetch = async (url, options) => {
  const payload = JSON.parse(options.body);
  posts.push([url, payload]);
  if (url.endsWith('/activate')) await new Promise((resolve) => { release = resolve; });
  return { ok: !fail, json: async () => fail ? { error: 'planner timeout' } : { success: true, message: '완료' } };
};
const source = fs.readFileSync(path.join(root, 'app.js'), 'utf8');
w.eval(source.slice(0, source.lastIndexOf('\nrefresh();')) + `
  refresh = async () => { window.onFileRefresh(); renderWaypointFiles(); };
  window.fileTest = { state, renderWaypointFiles };
`);
w.onFileRefresh = () => { refreshes += 1; };
const { state, renderWaypointFiles } = w.fileTest;
const rows = () => Array.from(d.querySelectorAll('.waypoint-file-row'));
const button = (action, fileId) => d.querySelector(`[data-file-action="${action}"][data-file-id="${fileId}"]`);
async function upload(files) {
  Object.defineProperty(id('waypointFileInput'), 'files', { configurable: true, value: files });
  id('waypointFileInput').dispatchEvent(new w.Event('change', { bubbles: true }));
  await tick(); await tick();
}
(async () => {
  const left = { arm: 'left', waypoint_files: [
    { id: '1', name: 'same.json', waypoint_count: 3, active: true },
    { id: '2', name: 'same.json', waypoint_count: 5, active: false }],
    execution: { active: false }, keyboard_control: { active: false } };
  const right = { arm: 'right', waypoint_files: [
    { id: '3', name: '<img src=x onerror="throw 1">.json', waypoint_count: 2, active: false },
    { id: '4', name: 'right.json', waypoint_count: 4, active: true }],
    execution: { active: false }, keyboard_control: { active: false } };
  state.armData = { left, right }; state.data = left;
  renderWaypointFiles();
  assert.equal(id('waypointFileInput').multiple, true);
  assert.equal(rows().length, 4);
  assert.equal(d.querySelectorAll('.waypoint-file-row.active').length, 2);
  assert.equal(d.querySelectorAll('#waypointFileLibrary img').length, 0, 'file names must be escaped');
  assert.equal(button('activate', '1').disabled, true);
  assert.equal(button('remove', '1').disabled, true);
  assert.equal(button('activate', '3').disabled, false);
  const stable = rows()[0]; renderWaypointFiles();
  assert.equal(rows()[0], stable, 'runtime polling must preserve file rows');

  await upload([{ name: 'left.json', text: async () => '{"arm":"left"}' },
    { name: 'right.json', text: async () => '{"arm":"right"}' }]);
  assert.deepEqual(posts.shift(), ['/api/waypoints/files/add', { files: [
    { name: 'left.json', waypoint_file: { arm: 'left' } },
    { name: 'right.json', waypoint_file: { arm: 'right' } }] }]);
  assert.equal(state.activeArm, 'left', 'registration must not change the active arm or plan');
  assert.equal(posts.length, 0);
  await upload([{ name: 'broken.json', text: async () => '{broken' }]);
  assert.equal(posts.length, 0, 'invalid JSON must not send a partially parsed batch');
  assert.ok(id('toast').textContent.includes('broken.json'));

  state.dwellDrafts.right = { 1: '99' };
  button('activate', '3').click();
  button('activate', '3').click();
  await tick();
  assert.deepEqual(posts.shift(), ['/api/waypoints/files/activate', { arm: 'right', id: '3' }]);
  assert.equal(posts.length, 0, 'double click must produce one activation');
  assert.equal(button('activate', '2').disabled, true);
  release(); await tick(); await tick();
  assert.equal(state.activeArm, 'right');
  assert.equal(Object.keys(state.dwellDrafts.right).length, 0, 'drafts from the previous file must be reset');
  assert.equal(state.waypointFileBusy, false);
  assert.equal(posts.length, 0, 'activation must not execute or preview robot motion');

  button('remove', '2').click(); await tick(); await tick();
  assert.deepEqual(posts.shift(), ['/api/waypoints/files/remove', { arm: 'left', id: '2' }]);
  fail = true;
  const before = refreshes;
  state.speedDrafts.left = { 1: '77' };
  button('activate', '2').click(); await tick();
  release(); await tick(); await tick();
  assert.ok(id('toast').textContent.includes('planner timeout'));
  assert.ok(refreshes > before, 'failed imports must refresh the possibly changed live plan');
  assert.equal(Object.keys(state.speedDrafts.left).length, 0);
  assert.equal(state.busy, false);

  left.execution.active = true; renderWaypointFiles();
  assert.equal(button('activate', '3').disabled, true);
  left.execution.active = false; right.keyboard_control.active = true; renderWaypointFiles();
  assert.equal(button('activate', '2').disabled, true);
  assert.equal(id('loadWaypointFile').disabled, false, 'registration does not change the running plan');
  assert.deepEqual(errors, []);
  console.log('Waypoint file library UI: multiple files, independent arms, activation, guards, drafts, errors and escaped names: PASS');
})().catch((error) => { console.error(error); process.exitCode = 1; }).finally(() => dom.window.close());
