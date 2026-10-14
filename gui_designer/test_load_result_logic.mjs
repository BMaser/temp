// Logic test of "Load result", no browser. Run: node test_load_result_logic.mjs   (exit code 0 only if every check passes)
// It cuts the pure block FLIGHT-LOGIC-BEGIN ... END out of src/track_designer.dc.html, loads src/td-core.js, and reads the real
// flight_result_forward_4.json. The expected numbers are computed HERE from the file's own `samples` and `summary`, not by the code under test.
import fs from 'node:fs';
import assert from 'node:assert/strict';

const HERE = new URL('.', import.meta.url);
const rel = p => new URL(p, HERE);
const FIXTURE = [rel('./ci_load_result/gui_designer/ref/flight_result_forward_4.json'), rel('./ref/flight_result_forward_4.json')].find(u => fs.existsSync(u));
const PAGE = [rel('./src/track_designer.dc.html')].find(u => fs.existsSync(u));
const TD = new Function('window', fs.readFileSync(rel('./src/td-core.js'), 'utf8').replace('export default TD;', '') + ';return TD')({});
const pageText = fs.readFileSync(PAGE, 'utf8').replace(/\r\n/g, '\n');
const m = pageText.match(/\/\* FLIGHT-LOGIC-BEGIN[\s\S]*?\*\/\n([\s\S]*?)\/\* FLIGHT-LOGIC-END \*\//);
assert.ok(m, 'the FLIGHT-LOGIC block was not found in the page');
const FLIGHT = new Function(m[1] + '\n;return FLIGHT;')();
const O = JSON.parse(fs.readFileSync(FIXTURE, 'utf8'));

let pass = 0, fail = 0;
function check(name, fn) {
  try { fn(); pass++; console.log('PASS  ' + name); }
  catch (e) { fail++; console.log('FAIL  ' + name + '\n      ' + String(e.message).split('\n').slice(0, 4).join(' | ')); }
}
const near = (a, b, e, what) => assert.ok(Math.abs(a - b) <= e, (what || '') + ' ' + a + ' vs ' + b + ' (tolerance ' + e + ')');
const wrap = a => { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; };

// ---- numbers computed independently from the file's own samples and summary
const SM = O.samples, N = SM.count, T = SM.time_s;
const indepPassTimes = []; for (let i = 1; i < N; i++) if (SM.passages[i] > SM.passages[i - 1]) indepPassTimes.push(T[i]);
const indepLap = T[N - 1], indepSplits = indepPassTimes.map((t, i) => t - (i ? indepPassTimes[i - 1] : 0));
const gateSeq = O.layout.route.filter(e => e.type === 'gate').map(e => e.gate);

check('the file is the real flight: 1053 rows, lap 21.04 s, 5 of 5 passages, route 1 2 3 4 2 (computed from samples)', () => {
  assert.equal(N, 1053); near(indepLap, 21.04, 1e-9, 'lap'); assert.equal(SM.passages[N - 1], 5); assert.equal(indepPassTimes.length, 5);
  assert.deepEqual(gateSeq, [1, 2, 3, 4, 2]);
});
check('format check: the real file passes; wrong files give one plain sentence, never a throw', () => {
  assert.equal(FLIGHT.checkResult(O), null);
  const bad = [null, 5, [], {}, { format: 'flight_result' }, { ...O, format: 'other' }, { ...O, version: 2 }, { ...O, layout: null }, { ...O, gui: {} },
    { ...O, gui: { ...O.gui, path: { ...O.gui.path, yaw: [1, 2] } } }, { ...O, gui: { ...O.gui, path: { ...O.gui.path, time: O.gui.path.time.slice().reverse() } } },
    { ...O, gui: { ...O.gui, run: { ...O.gui.run, id: 5 } } }];
  bad.forEach((b, i) => { const r = FLIGHT.checkResult(b); assert.equal(typeof r, 'string', 'case ' + i + ' was accepted'); assert.ok(r.length > 10); });
  assert.match(FLIGHT.checkResult({ ...O, format: 'other' }), /"other"/); assert.match(FLIGHT.checkResult({ ...O, version: 2 }), /version 2/);
});
const P = FLIGHT.loadedPath(O);
check('loaded path: end event, total, passage times equal the times found independently in the samples', () => {
  assert.equal(P.loaded, true); assert.equal(P.end.reason, 'finished'); near(P.end.t, 21.04, 1e-9); near(P.total, indepLap, 1e-9); assert.equal(P.hit, null);
  assert.equal(P.gates.length, 5); P.gates.forEach((g, i) => near(g.t, indepPassTimes[i], 1e-9, 'passage ' + (i + 1)));
  assert.deepEqual(P.gates.map(g => g.gate), gateSeq); near(P.length, O.summary.path_length_m, 1e-3, 'length');
});
check('loaded flight: every split >= 0, 5 splits, they equal the splits computed from the samples AND summary.split_times_s, and sum to the last passage time', () => {
  const sl = FLIGHT.splitList(P.gates); assert.equal(sl.length, 5);
  sl.forEach((r, i) => { assert.ok(r.split >= 0, 'negative split ' + r.split); near(r.split, indepSplits[i], 1e-9); near(r.split, O.summary.split_times_s[i], 1e-9); });
  near(sl.reduce((a, r) => a + r.split, 0), O.summary.passage_times_s[4], 1e-9); near(O.summary.passage_times_s[4], indepPassTimes[4], 1e-9);
  near(O.summary.lap_time_s, 21.04, 1e-9);
});
check('interpolation AT a sample time returns that sample (x, y, z, speed, yaw, pitch, roll; rows 0, 1, 240, 423, 700, 1051, 1052)', () => {
  [0, 1, 240, 423, 700, 1051, 1052].forEach(k => {
    const s = FLIGHT.stateAt(P, T[k]);
    near(s.x, SM.x_m[k], 1e-9, 'x@' + k); near(s.y, SM.y_m[k], 1e-9, 'y@' + k); near(s.z, SM.z_m[k], 1e-9, 'z@' + k);
    near(s.v, SM.speed_mps[k], 1e-9, 'v@' + k);
    near(wrap(s.yaw - SM.yaw_deg[k] * Math.PI / 180), 0, 2e-5, 'yaw@' + k); near(s.pitch, SM.pitch_deg[k] * Math.PI / 180, 2e-5, 'pitch@' + k); near(s.roll, SM.roll_deg[k] * Math.PI / 180, 2e-5, 'roll@' + k);
  });
});
check('interpolation HALFWAY between two samples lies between them (x, y, z, speed, pitch, roll), for 60 rows', () => {
  for (let k = 0; k < N - 1; k += 17) {
    const s = FLIGHT.stateAt(P, (T[k] + T[k + 1]) / 2), within = (v, a, b) => v >= Math.min(a, b) - 1e-9 && v <= Math.max(a, b) + 1e-9;
    assert.ok(within(s.x, SM.x_m[k], SM.x_m[k + 1]), 'x@' + k); assert.ok(within(s.y, SM.y_m[k], SM.y_m[k + 1]), 'y@' + k); assert.ok(within(s.z, SM.z_m[k], SM.z_m[k + 1]), 'z@' + k);
    assert.ok(within(s.v, SM.speed_mps[k], SM.speed_mps[k + 1]), 'v@' + k); assert.ok(within(s.pitch, P.pitch[k], P.pitch[k + 1]), 'pitch@' + k); assert.ok(within(s.roll, P.roll[k], P.roll[k + 1]), 'roll@' + k);
    near(s.x, (SM.x_m[k] + SM.x_m[k + 1]) / 2, 1e-9, 'x midpoint');
  }
});
check('time outside the flight gives the first or the last sample; the yaw goes the short way around +-pi', () => {
  near(FLIGHT.stateAt(P, -5).x, SM.x_m[0], 1e-9); near(FLIGHT.stateAt(P, 999).x, SM.x_m[N - 1], 1e-9); assert.equal(FLIGHT.stateAt(P, 999).idx, N - 1);
  const Q = { time: [0, 1], S: [{ x: 0, y: 0, z: 0 }, { x: 1, y: 0, z: 0 }], v: [0, 1], dist: [0, 1], yaw: [3.1, -3.1], pitch: [0, 0], roll: [0, 0] };
  assert.ok(Math.abs(Math.abs(FLIGHT.stateAt(Q, 0.5).yaw) - Math.PI) < 0.1, 'went through zero');
});

// ---- the negative split of a gate flown twice (route 1 2 3 4 2)
const L = TD.fromFile(O.layout);
const geo = TD.routeGeometry(L, 32), S = geo.samples;
const dist = [0]; for (let i = 1; i < S.length; i++) dist.push(dist[i - 1] + Math.hypot(S[i].x - S[i - 1].x, S[i].y - S[i - 1].y, S[i].z - S[i - 1].z));
const time = dist.map(d => d / 1.0);                         // any increasing time will do for the split logic
// OLD code, copied from the unmodified page (buildPath): the sample nearest to the gate center over the WHOLE path; the split is g.t minus the previous g.t
const oldGates = geo.knots.filter(k => k.type === 'gate').map(k => { let best = 0, bd = Infinity; S.forEach((p, i) => { const d = Math.hypot(p.x - k.x, p.y - k.y, p.z - k.z); if (d < bd) { bd = d; best = i; } }); return { gate: k.gate, passage: k.passage, idx: best, t: time[best] }; });
const oldSplits = oldGates.map((g, i) => g.t - (i ? oldGates[i - 1].t : 0));
const gi = FLIGHT.gateSamples(geo, S), newGates = geo.knots.filter(k => k.type === 'gate').map((k, j) => ({ gate: k.gate, passage: k.passage, idx: gi[j], t: time[gi[j]] }));
check('POSITIVE CONTROL: the OLD split code gives a negative split on route 1 2 3 4 2 (so this test can fail)', () => {
  assert.equal(oldGates.length, 5); assert.ok(oldSplits.some(x => x < 0), 'old splits ' + oldSplits.map(x => x.toFixed(2)).join(' '));
  assert.equal(oldGates[1].idx, oldGates[4].idx, 'the cause: passages 2 and 5 (gate 2) got the same sample');
  assert.ok(FLIGHT.runSplits([5, 3, 9]).some(x => x < 0));
});
check('NEW code: passages of a clicked route 1 2 3 4 2 get 5 different, increasing samples, each at its gate center; every split >= 0 and they sum to the last passage time', () => {
  assert.equal(new Set(gi).size, 5); for (let i = 1; i < 5; i++) assert.ok(gi[i] > gi[i - 1], 'not increasing: ' + gi.join(','));
  geo.knots.filter(k => k.type === 'gate').forEach((k, j) => assert.ok(Math.hypot(S[gi[j]].x - k.x, S[gi[j]].y - k.y, S[gi[j]].z - k.z) < 1e-9, 'passage ' + (j + 1) + ' is not at its gate'));
  const sl = FLIGHT.splitList(newGates); assert.equal(sl.length, 5); sl.forEach(r => assert.ok(r.split >= 0 && r.split > 0, 'split ' + r.split));
  near(sl.reduce((a, r) => a + r.split, 0), newGates[4].t, 1e-9);
  assert.deepEqual(FLIGHT.runSplits(newGates.map(g => g.t)).map(x => +x.toFixed(9)), sl.map(r => +r.split.toFixed(9)));
});
check('NEW code on a DRAWN path through the gates: increasing samples, each close to its crossing', () => {
  const D = (pts, step = 0.02) => { const out = [pts[0]]; for (let i = 1; i < pts.length; i++) { const a = pts[i - 1], b = pts[i], n = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / step)); for (let k = 1; k <= n; k++) out.push([a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n]); } return out; };
  const g1in = [0.144, 0.602], g1out = [0.856, -0.1], g2in = [1.402, 0.394], g2out = [0.7, 1.1], g3in = [-0.5, -0.25], g3out = [-1.5, -0.25];
  const pts = [[-1.4, 0.8], [-0.6, 0.85], g1in, [0.5, 0.25], g1out, [1.45, 0.05], g2in, [1.05, 0.75], g2out, [0.0, 0.95], [-0.4, 0.4], g3in, [-1, -0.25], g3out];
  const Ld = TD.todayLayout(); Ld.route = []; assert.ok(TD.addStroke(Ld, D(pts).map(p => TD.snapSample(Ld, p[0], p[1], false))));
  const gd = TD.pathGeometry(Ld, 32), idx = FLIGHT.gateSamples(gd, gd.samples); assert.ok(idx.length >= 3, 'gates found ' + idx.length);
  for (let i = 1; i < idx.length; i++) assert.ok(idx[i] >= idx[i - 1]);
  const kn = gd.knots.filter(k => k.type === 'gate'); kn.forEach((k, j) => assert.ok(Math.hypot(gd.samples[idx[j]].x - k.x, gd.samples[idx[j]].y - k.y) < 0.05, 'passage ' + (j + 1)));
});

// ---- the layout that a result file brings must stay as it is when the page draws it (otherwise "layout changed" would close the flight at once)
check('the loaded layout is stable: its text does not change under the page\'s own route normalisation, and the route is 1 2 3 4 2', () => {
  const t1 = TD.layoutToText(L), r = TD.parseRouteText(TD.routeToText(L), L.gates.length);
  assert.equal(r.errors.length, 0); assert.equal(TD.entriesToText(r.entries), TD.routeToText(L)); assert.equal(TD.layoutToText(L), t1);
  assert.deepEqual(TD.orderOf(L), [1, 2, 3, 4, 2]); assert.equal(L.path, null);
});

// ---- run list
check('the same file loaded twice gives ONE run row (matched on id); another id adds a row; the row says Simulator', () => {
  const r1 = FLIGHT.pushRun([], O.gui.run), r2 = FLIGHT.pushRun(r1, O.gui.run), r3 = FLIGHT.pushRun(r2, { ...O.gui.run, id: 'flight_other' });
  assert.equal(r1.length, 1); assert.equal(r2.length, 1); assert.equal(r3.length, 2); assert.equal(r2[0].id, O.gui.run.id);
  assert.equal(r2[0].source, 'simulator'); assert.equal(FLIGHT.sourceLabel(r2[0]), 'Simulator');
});
check('Source column: a run from Play (source "preview") and a stored run WITHOUT the field both read "Page preview"', () => {
  assert.equal(FLIGHT.sourceLabel({ id: 'r99', source: 'preview' }), 'Page preview'); assert.equal(FLIGHT.sourceLabel({ id: 'r98' }), 'Page preview'); assert.equal(FLIGHT.sourceLabel(null), 'Page preview');
  assert.notEqual(FLIGHT.sourceLabel({ source: 'simulator' }), FLIGHT.sourceLabel({ source: 'preview' }));
});
check('stored runs: the invented r1 r2 r3 and damaged records are dropped, good runs stay, non-arrays give an empty list', () => {
  const good = { id: 'r1699', name: 'Run 1', date: 'x', lap: 7, vmax: 2, minClear: null, ok: true, splits: [1, 2], profile: [{ d: 0, z: 0, v: 0 }], track: 't' };
  const out = FLIGHT.cleanRuns([{ ...good, id: 'r1' }, { ...good, id: 'r2' }, { ...good, id: 'r3' }, good, { id: 'r5' }, null, { ...good, id: 'r6', profile: [] }, { ...good, id: 'r7', lap: 'x' }]);
  assert.deepEqual(out.map(r => r.id), ['r1699']); assert.deepEqual(FLIGHT.cleanRuns(null), []); assert.deepEqual(FLIGHT.cleanRuns('x'), []);
  assert.deepEqual(FLIGHT.runSplits([]), []); assert.equal(Math.max(1, ...[]), 1);
});
check('the fixture run record itself is accepted by the stored-run filter (so it survives a reload)', () => { assert.equal(FLIGHT.cleanRuns([O.gui.run]).length, 1); });

// ---- what the page source contains
check('page source: the invented runs are gone; the preview banner, the simulator banner and the Source column are there; no university, lab or company name', () => {
  assert.ok(!/seedRuns/.test(pageText) && !/\['r1', 'r3'\]/.test(pageText));
  assert.ok(pageText.includes('Preview computed in this page. No simulator, no physics. Load a result file to see a real simulator flight.'));
  assert.ok(pageText.includes("'Simulator flight: ' + S.flight.name + ' (run ' + S.flight.runId + ')'"));
  assert.ok(pageText.includes('<th style="padding:6px 8px">Source</th>') && pageText.includes('Load result'));
  assert.ok(!/LSY|\bTUM\b|Munich|Crazyf|Technische|Universit/i.test(pageText));
});

console.log('\n' + (pass + fail) + ' checks: ' + pass + ' passed, ' + fail + ' failed');
process.exit(fail ? 1 : 0);
