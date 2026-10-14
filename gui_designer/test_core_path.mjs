// Logic checks of the fourth round, PORTED from test_designer.mjs lines 342-516 (the drawn flight path, the crossings, the gate order,
// save and load with a path, shaky hand, loop, side view and 3D with a path). Only the loading of the core differs:
// the core is src/td-core.js (generated from our present page). Run: node test_core_path.mjs
import fs from 'node:fs';
import assert from 'node:assert/strict';
const TD = new Function('window', fs.readFileSync(new URL('./src/td-core.js', import.meta.url), 'utf8').replace('export default TD;', '') + ';return TD')({});
let pass = 0, fail = 0;
function check(name, fn) {
  try { fn(); pass++; console.log('PASS  ' + name); }
  catch (e) { fail++; console.log('FAIL  ' + name + '\n      ' + String(e.message).split('\n').slice(0, 3).join(' | ')); }
}
const today = () => TD.todayLayout();
const near = (a, b, e = 1e-9) => Math.abs(a - b) < e;
// ================= the drawn flight path (round 4)
const densify = (pts, step = 0.02) => { const out = [pts[0]]; for (let i = 1; i < pts.length; i++) { const a = pts[i - 1], b = pts[i], n = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / step)); for (let k = 1; k <= n; k++) out.push([a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n]); } return out; };
const pen = (L, pts, alt = false, step = 0.02) => TD.addStroke(L, densify(pts, step).map(p => TD.snapSample(L, p[0], p[1], alt)));
const g1in = [0.144, 0.602], g1out = [0.856, -0.1], g2in = [1.402, 0.394], g2out = [0.7, 1.1], g3in = [-0.5, -0.25], g3out = [-1.5, -0.25];
const threeGates = [[-1.4, 0.8], [-0.6, 0.85], g1in, [0.5, 0.25], g1out, [1.45, 0.05], g2in, [1.05, 0.75], g2out, [0.0, 0.95], [-0.4, 0.4], g3in, [-1, -0.25], g3out];
const drawn = () => { const L = today(); L.route = []; assert.ok(pen(L, threeGates)); return L; };
const comp = L => TD.pathCompute(L);
const sig = c => c.openings.map(o => (o.sign < 0 ? '-' : '') + o.gate).join(',');
check('point reduction (Ramer-Douglas-Peucker, 0.05 m): a straight line keeps two points, a corner three, noise is removed, pinned points stay', () => {
  const line = densify([[0, 0], [2, 0]], 0.01).map(p => [p[0], p[1]]);
  assert.deepEqual(TD.rdpKeep(line, 0.05, []), [0, line.length - 1]);
  const corner = densify([[0, 0], [1, 0], [1, 1]], 0.01); assert.equal(TD.rdpKeep(corner, 0.05, []).length, 3);
  const shaky = densify([[0, 0], [2, 0]], 0.01).map((p, i) => [p[0], p[1] + 0.015 * Math.sin(i * 1.3)]); assert.equal(TD.rdpKeep(shaky, 0.05, []).length, 2);
  const pinned = line.map((_, i) => i === 50); assert.ok(TD.rdpKeep(line, 0.05, pinned).includes(50));
  const red = TD.reduceStroke(densify([[0, 0], [1, 0], [1, 1]], 0.01).map(p => [p[0], p[1], -1])); assert.equal(red.length, 3); assert.ok(red.length < 20);
  const circle = []; for (let i = 0; i <= 200; i++) circle.push([Math.cos(i / 200 * 6.2832) * 0.5, Math.sin(i / 200 * 6.2832) * 0.5, -1]);
  const rc = TD.reduceStroke(circle); assert.ok(rc.length >= 8 && rc.length <= 40, 'circle -> ' + rc.length);   // few dots, still round
});
check('every raw point is within 0.05 m of the reduced polyline (the reduction keeps the shape)', () => {
  const raw = densify(threeGates).map(p => [p[0], p[1], -1]), red = TD.reduceStroke(raw);
  assert.ok(red.length < raw.length / 10);
  raw.forEach(p => { let d = 9; for (let i = 0; i + 1 < red.length; i++) { const a = red[i], b = red[i + 1], dx = b[0] - a[0], dy = b[1] - a[1], l2 = dx * dx + dy * dy || 1, t = Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2)); d = Math.min(d, Math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)); } assert.ok(d <= 0.0501, 'raw point ' + d); });
});
check('snap: within 0.15 m of an opening center the pen is pulled to the center; Alt switches it off; far away nothing happens', () => {
  const L = today(), g = L.gates[0];
  assert.deepEqual(TD.snapSample(L, g.pos[0] + 0.1, g.pos[1] - 0.08, false), [g.pos[0], g.pos[1], 0]);
  assert.equal(TD.snapSample(L, g.pos[0] + 0.16, g.pos[1], false)[2], -1); assert.deepEqual(TD.snapSample(L, g.pos[0] + 0.1, g.pos[1], true), [g.pos[0] + 0.1, g.pos[1], -1]);
  assert.equal(TD.snapSample(L, 9, 9, false)[0], 3);           // held inside the box
});
check('a stroke that begins away from the start pad is joined to the start pad; the end pad is placed where the path ends', () => {
  const L = today(); L.route = []; L.end = null; assert.ok(pen(L, [[-0.5, 0.3], [0.2, -0.2], [1.2, -0.3]]));
  const P = L.path.points; assert.deepEqual(P[0].slice(0, 2), L.start.pos.slice(0, 2)); assert.ok(P.length >= 3);
  assert.deepEqual(L.end.pos.slice(0, 2), P[P.length - 1].slice(0, 2)); assert.ok(near(L.end.pos[0], 1.2, 0.01) && near(L.end.pos[1], -0.3, 0.01));
  assert.ok(!pen(L, [[0, 0], [0.02, 0.01]]), 'a tiny stroke is not a stroke');
});
check('a second stroke continues from the end of the line, and the end pad moves to the new end', () => {
  const L = today(); L.route = []; pen(L, [[-0.5, 0.3], [0.2, -0.2], [1.2, -0.3]]); const n1 = L.path.points.length;
  pen(L, [[1.5, -0.2], [1.8, 0.4], [1.9, 1.0]]); assert.ok(L.path.points.length > n1);
  assert.ok(near(L.end.pos[0], 1.9, 0.01) && near(L.end.pos[1], 1.0, 0.01)); assert.deepEqual(L.path.points[0].slice(0, 2), L.start.pos.slice(0, 2));
});
check('the curve passes exactly through its dots', () => {
  const L = drawn(), c = comp(L);
  L.path.points.forEach((p, i) => { const s = c.samples[c.cpIdx[i]]; assert.ok(near(s.x, p[0], 1e-9) && near(s.y, p[1], 1e-9), 'dot ' + i); });
  assert.equal(c.cpIdx.length, L.path.points.length);
  c.samples.forEach(s => assert.ok(Number.isFinite(s.x) && Number.isFinite(s.y) && Number.isFinite(s.z)));
});
check('the curve is smooth: no sharp corner at the dots (the direction changes gently from sample to sample)', () => {
  const L = today(); L.route = []; pen(L, [[-0.5, 0.3], [0.2, -0.2], [1.2, -0.3], [1.6, 0.5]]); const s = comp(L).samples;
  for (let i = 1; i + 1 < s.length; i++) { const a = [s[i].x - s[i - 1].x, s[i].y - s[i - 1].y], b = [s[i + 1].x - s[i].x, s[i + 1].y - s[i].y], la = Math.hypot(...a), lb = Math.hypot(...b); if (la < 1e-9 || lb < 1e-9) continue; assert.ok((a[0] * b[0] + a[1] * b[1]) / (la * lb) > 0.5, 'bend at sample ' + i); }
});
check('crossing finder: through the opening along the arrow (+), against it (-), through the frame only (red), and no crossing', () => {
  const L = today(); L.path = { points: [[0, 0, 0], [0, 0, 0]], z_locked: [false, false], closed: false };
  const g = L.gates[3], c = [g.pos[0], g.pos[1]];          // gate 4: center (0,-0.75), arrow +x, opening along y
  const run = pts => TD.findCrossings(L, pts.map(p => ({ x: p[0], y: p[1] })));
  let r = run([[c[0] - 0.5, c[1]], [c[0] + 0.5, c[1]]]).filter(x => x.gate === 4); assert.equal(r.length, 1); assert.equal(r[0].sign, 1); assert.equal(r[0].kind, 'opening');
  r = run([[c[0] + 0.5, c[1]], [c[0] - 0.5, c[1]]]).filter(x => x.gate === 4); assert.equal(r[0].sign, -1); assert.equal(r[0].kind, 'opening');
  r = run([[c[0] - 0.5, c[1] + 0.3], [c[0] + 0.5, c[1] + 0.3]]).filter(x => x.gate === 4); assert.equal(r.length, 1); assert.equal(r[0].kind, 'frame');     // 0.3 m from the center: on the frame
  r = run([[c[0] - 0.5, c[1] + 0.45], [c[0] + 0.5, c[1] + 0.45]]).filter(x => x.gate === 4); assert.equal(r.length, 0);                                    // beside the gate
  r = run([[c[0] - 0.5, c[1]], [c[0] - 0.1, c[1]]]).filter(x => x.gate === 4); assert.equal(r.length, 0);                                                  // stops before it
  r = run([[c[0] - 0.5, c[1] + 0.19], [c[0] + 0.5, c[1] + 0.19]]).filter(x => x.gate === 4); assert.equal(r[0].kind, 'opening'); r = run([[c[0] - 0.5, c[1] + 0.21], [c[0] + 0.5, c[1] + 0.21]]).filter(x => x.gate === 4); assert.equal(r[0].kind, 'frame');
  r = run([[c[0] - 0.5, c[1]], [c[0], c[1]], [c[0] + 0.5, c[1]]]).filter(x => x.gate === 4); assert.equal(r.length, 1, 'a sample exactly on the plane counts once');
});
check('the path through three gates of today\'s track passes gate 1, gate 2, gate 3, all along their arrows, and touches no frame', () => {
  const L = drawn(), c = comp(L);
  assert.equal(sig(c), '1,2,3'); assert.equal(c.frames.length, 0, 'frames ' + JSON.stringify(c.frames.map(f => [f.gate, f.u])));
  assert.deepEqual(TD.orderOf(L), [1, 2, 3]); assert.deepEqual(TD.pathNotes(L, c).lines, ['The path passes: gate 1, gate 2, gate 3.', 'Not on the path: gate 4.']);
});
check('the order follows the curve, a gate crossed twice is listed twice, a gate crossed against its arrow shows a minus', () => {
  const L = today(); L.route = [];
  pen(L, [[-1.4, 0.8], [g1in[0], g1in[1]], [0.5, 0.25], g1out, [0.9, -0.5], [0.0, -0.75 - 0.5], [0, -0.75], [0.5, -0.75], [0.9, -0.2], g1out, [0.5, 0.25], g1in]);
  const c = comp(L);
  const nums = c.openings.map(o => o.gate); assert.deepEqual(nums.slice(0, 1), [1]); assert.ok(nums.filter(g => g === 1).length >= 2, 'gate 1 twice: ' + sig(c));
  assert.ok(c.openings.some(o => o.gate === 1 && o.sign > 0) && c.openings.some(o => o.gate === 1 && o.sign < 0));
  assert.deepEqual(TD.orderOf(L), c.openings.map(o => o.sign * o.gate)); console.log('      (path list: ' + sig(c) + ')');
});
check('a line that passes beside a gate touches its frame (red) and the notes say so; a line that misses the gate is just "not on the path"', () => {
  const L = today(); L.route = []; const g = L.gates[3];
  L.path = { points: [[-1, g.pos[1] + 0.3, 0], [0, g.pos[1] + 0.3, 0], [1, g.pos[1] + 0.3, 0]], z_locked: [false, false, false], closed: false };    // a straight line 0.3 m beside the center: on the frame
  const c = comp(L); assert.ok(c.frames.some(f => f.gate === 4), 'no frame mark'); assert.ok(!c.openings.some(o => o.gate === 4));
  const n = TD.pathNotes(L, c); assert.ok(n.lines.some(t => /^Red mark: the path touches the frame of gate 4/.test(t)) && n.notOn.includes('gate 4'));
  const svg = TD.mapSvg(L, c, null); assert.ok(/class="badx"/.test(svg) && /frame of gate 4/.test(svg));
});
check('a wiggle across a gate plane (there and back within 0.3 m) is not counted as a pass', () => {
  const L = today(); L.path = { points: [[0, 0, 0], [0, 0, 0]], z_locked: [false, false], closed: false }; const c = L.gates[3].pos;
  const zig = [[c[0] - 0.3, c[1]], [c[0] + 0.05, c[1]], [c[0] - 0.05, c[1]], [c[0] + 0.3, c[1]]];
  const r = TD.findCrossings(L, densify(zig, 0.01).map(p => ({ x: p[0], y: p[1] }))).filter(x => x.gate === 4); assert.equal(r.length, 1, 'the wiggle left ' + r.length + ' crossings');
});
check('heights: floor at the start and the end, the gate center height where the path crosses a gate, smooth in between', () => {
  const L = drawn(), c = comp(L);
  assert.equal(c.samples[0].z, L.start.pos[2]); assert.equal(c.samples[c.samples.length - 1].z, L.end.pos[2]); assert.equal(L.end.pos[2], 0);
  c.openings.forEach(o => { const z = L.gates[o.gate - 1].pos[2], k = c.samples.reduce((b, s, i) => Math.hypot(s.x - o.x, s.y - o.y) < Math.hypot(c.samples[b].x - o.x, c.samples[b].y - o.y) ? i : b, 0); assert.ok(near(c.samples[k].z, z, 0.03), 'gate ' + o.gate + ' crossing at ' + c.samples[k].z + ' not ' + z); });
  for (let i = 1; i < c.samples.length; i++) assert.ok(Math.abs(c.samples[i].z - c.samples[i - 1].z) < 0.06, 'a jump in height');
  L.path.points.forEach((p, i) => { assert.equal(p[2], Math.round(c.cpz[i] * 1000) / 1000); });
  assert.ok(Math.max(...c.samples.map(s => s.z)) <= 1.2 + 1e-9 && Math.min(...c.samples.map(s => s.z)) >= -1e-9);
});
check('a dot dragged up or down keeps its height (it is locked) and the curve follows; the other dots are recomputed', () => {
  const L = drawn(); const free = L.path.points.findIndex((p, i) => i > 0 && i < L.path.points.length - 1 && TD.gateAt(L, p[0], p[1], 0.03) < 0);
  TD.dragSide(L, 'cp', free, L.path.points[free][0], 1.9, [0, 0], false); TD.pathRefresh(L);
  assert.equal(L.path.points[free][2], 1.9); assert.equal(L.path.z_locked[free], true); const c = comp(L); assert.ok(near(c.samples[c.cpIdx[free]].z, 1.9, 1e-9));
  TD.moveObject(L, 'cp', free, L.path.points[free][0] + 0.05, L.path.points[free][1]); TD.pathRefresh(L); assert.equal(L.path.points[free][2], 1.9);
});
check('the gates come from the path: after every change the route is recomputed (moving a gate off the line removes it)', () => {
  const L = drawn(); assert.deepEqual(TD.orderOf(L), [1, 2, 3]); TD.moveObject(L, 'gate', 1, 1.05, -2.0); TD.pathRefresh(L);
  assert.deepEqual(TD.orderOf(L), [1, 3]); assert.ok(TD.pathNotes(L).notOn.includes('gate 2'));
});
check('Turn the gates to the path: every gate crossed against its arrow is turned, no negative number remains (a gate crossed both ways is left alone)', () => {
  const L = today(); L.route = []; TD.turnAround(L, 'gate', 0); TD.turnAround(L, 'gate', 2);       // gates 1 and 3 now point the wrong way for the same line
  assert.ok(pen(L, threeGates)); let c = comp(L); assert.ok(c.openings.some(o => o.sign < 0), 'a minus is expected: ' + sig(c));
  const r = TD.turnGatesToPath(L); assert.ok(r.turned.length >= 1); c = comp(L); assert.ok(c.openings.every(o => o.sign > 0), 'still: ' + sig(c)); assert.deepEqual(TD.orderOf(L).filter(n => n < 0), []);
  const M = today(); M.route = []; pen(M, [[-1.4, 0.8], g1in, [0.5, 0.25], g1out, [0.9, -0.5], [0.9, -0.2], g1out, [0.5, 0.25], g1in]);
  const rm = TD.turnGatesToPath(M); assert.ok(rm.mixed.includes(1), 'mixed ' + JSON.stringify(rm));
});
check('the end pad follows the path and the path follows the end pad; the start pad too', () => {
  const L = drawn(), n = L.path.points.length - 1;
  TD.moveObject(L, 'end', 0, 0.5, -1.5); assert.deepEqual(L.path.points[n].slice(0, 2), [0.5, -1.5]);
  TD.moveObject(L, 'cp', n, -0.5, -2); assert.deepEqual(L.end.pos.slice(0, 2), [-0.5, -2]);
  TD.dropObject(L, 'end', 1.0, 1.0, false); assert.deepEqual(L.path.points[n].slice(0, 2), [1, 1]);
  TD.moveObject(L, 'start', 0, -2, -1); assert.deepEqual(L.path.points[0].slice(0, 2), [-2, -1]); TD.setField(L, 'end', 0, 'x', 2.5); assert.equal(L.path.points[n][0], 2.5);
});
check('add a dot on the curve (double-click) and delete a dot; the first and last dots (the pads) cannot be deleted', () => {
  const L = drawn(), c = comp(L), mid = c.samples[Math.floor(c.samples.length * 0.37)], n0 = L.path.points.length;
  const k = TD.cpInsert(L, mid.x, mid.y + 0.02); assert.ok(k > 0 && k < n0); assert.equal(L.path.points.length, n0 + 1); assert.equal(L.path.z_locked.length, n0 + 1);
  const c2 = comp(L); const s = c2.samples[c2.cpIdx[k]]; assert.ok(near(s.x, L.path.points[k][0], 1e-9));       // the curve passes exactly through the new dot
  assert.equal(TD.cpInsert(L, 2.9, -2.9), -1);
  assert.equal(TD.cpDelete(L, k), true); assert.equal(L.path.points.length, n0); assert.equal(TD.cpDelete(L, 0), false); assert.equal(TD.cpDelete(L, L.path.points.length - 1), false);
  TD.deleteObject(L, 'cp', 1); assert.equal(L.path.points.length, n0 - 1);
});
check('a dot at a gate center leaves along the gate direction (the line crosses the opening at right angles)', () => {
  const L = drawn(), T = TD.cpTangents(L, L.path.points);
  L.path.points.forEach((p, i) => { const gi = TD.gateAt(L, p[0], p[1], 0.03); if (gi < 0 || i === 0 || i === L.path.points.length - 1) return; const d = TD.gateDir ? TD.gateDir(L.gates[gi]) : [0, 0]; assert.ok(Math.abs(T[i][0] * d[0] + T[i][1] * d[1]) > 0.999, 'dot ' + i + ' not along gate ' + (gi + 1)); });
  assert.ok(L.path.points.some(p => TD.gateAt(L, p[0], p[1], 0.03) >= 0));
});
check('save and load with path: path, route (gate and point entries), gate_order and end are written, and read back unchanged', () => {
  const L = drawn(), f = JSON.parse(TD.layoutToText(L));
  assert.ok(Array.isArray(f.path.points) && f.path.closed === false && f.path.points.every(p => p.length === 3)); assert.deepEqual(f.gate_order, [1, 2, 3]); assert.ok(f.route.filter(e => e.type === 'gate').length === 3 && f.route.every(e => e.type === 'gate' || (e.type === 'point' && e.pos.length === 3)));
  assert.deepEqual(f.end.pos, L.end.pos); assert.ok(!('z_locked' in f.path));
  const back = TD.layoutFromText(TD.layoutToText(L)); assert.deepEqual(back, L);
  const M = drawn(); TD.dragSide(M, 'cp', 1, M.path.points[1][0], 1.5, [0, 0], false); TD.pathRefresh(M); const fm = JSON.parse(TD.layoutToText(M)); assert.ok(fm.path.z_locked[1] === true); assert.deepEqual(TD.layoutFromText(TD.layoutToText(M)), M);
});
check('a file without path still loads: the route is used as before (the old mode is unchanged)', () => {
  const o = TD.toFile(today()); assert.ok(!('path' in o)); const L = TD.fromFile(o); assert.equal(L.path, null); assert.deepEqual(TD.orderOf(L), [1, 2, 3, 4, 2]);
  const R = TD.routeGeometry(L); assert.ok(R.samples.length > 50);
  const old = TD.toFile(drawn()); delete old.path; const L2 = TD.fromFile(old); assert.equal(L2.path, null); assert.deepEqual(TD.orderOf(L2), [1, 2, 3]);       // only route and gate_order: it loads as a route
  TD.clearRoute(L2); assert.deepEqual(L2.route, []); const D = drawn(); TD.clearRoute(D); assert.equal(D.path, null);
});
check('both modes write the same fields: a click-built route has no path, a drawn path writes route and gate_order; Clear path removes both', () => {
  const L = today(); TD.clearRoute(L); TD.routeClick(L, 1); TD.routeClick(L, 2); assert.equal(L.path, null); assert.deepEqual(TD.toFile(L).gate_order, [1, 2]);
  const D = drawn(); assert.ok(TD.toFile(D).path && TD.toFile(D).gate_order.length === 3); TD.pathClear(D); assert.equal(D.path, null); assert.deepEqual(D.route, []);
});
// ----- lines that a hand really draws (seeded noise, so the test is repeatable)
const rng = seed => () => (seed = (seed * 1664525 + 1013904223) % 4294967296) / 4294967296;
const shaky = (pts, amp, seed, step = 0.004) => { const r = rng(seed); return densify(pts, step).map(p => [p[0] + (r() - 0.5) * 2 * amp, p[1] + (r() - 0.5) * 2 * amp]); };
check('a shaky hand (noise of +-1.5 cm at every mouse sample) still gives the same gates, and only a few dots', () => {
  for (const seed of [1, 2, 3, 4, 5]) {
    const L = today(); L.route = []; assert.ok(TD.addStroke(L, shaky(threeGates, 0.015, seed).map(p => TD.snapSample(L, p[0], p[1], false))));
    assert.equal(sig(comp(L)), '1,2,3', 'seed ' + seed + ': ' + sig(comp(L))); assert.ok(L.path.points.length <= 24, 'dots ' + L.path.points.length);
  }
});
check('a loop (a circle drawn by hand) gives a smooth closed-looking curve, not a crash, and the gate it goes through twice is listed twice', () => {
  const L = today(); L.route = []; const g = L.gates[3], cx = g.pos[0], cy = g.pos[1];
  const circle = []; for (let i = 0; i <= 140; i++) { const a = -Math.PI + i / 140 * 2 * Math.PI * 1.15; circle.push([cx + 0.5 * Math.cos(a) * 0 + 0.0 + (0.6 * Math.cos(a)), cy - 0.6 + 0.6 * Math.sin(a) + 0.6]); }
  assert.ok(TD.addStroke(L, circle.map(p => TD.snapSample(L, p[0], p[1], false)))); const c = comp(L); assert.ok(c.samples.every(s => Number.isFinite(s.x + s.y + s.z)));
  assert.ok(L.path.points.length >= 8 && L.path.points.length <= 40);
});
check('a line through the opening 0.17 m beside the center counts (it is outside the 0.15 m snap but inside the opening); 0.25 m beside is a frame mark; 0.40 m beside is nothing', () => {
  const run = off => { const L = today(); L.route = []; L.path = { points: [[-1.5, -0.75 + off, 0], [-0.5, -0.75 + off, 0], [0.5, -0.75 + off, 0], [1.5, -0.75 + off, 0]], z_locked: [false, false, false, false], closed: false }; return comp(L).crossings.filter(x => x.gate === 4); };
  let r = run(0.17); assert.ok(r.length === 1 && r[0].kind === 'opening'); r = run(0.25); assert.ok(r.length === 1 && r[0].kind === 'frame'); assert.equal(run(0.4).length, 0);
});
check('the path shows in the side view (dots, curve with heights) and in 3D', () => {
  const L = drawn(), c = comp(L), sv = TD.sideSvg(L, c, null, 1000, 300), n = L.path.points.length;
  assert.equal([...sv.matchAll(/data-kind="cp"/g)].length, n - 2); assert.ok(/class="s-route"/.test(sv));
  const s3 = TD.view3dSvg(L, c, TD.newCamera(), 733, 457); assert.ok((s3.match(/class="route3"/g) || []).length >= 100 && /class="pt3"/.test(s3));
  const mp = TD.mapSvg(L, c, null, undefined, { snapGate: 1, stroke: [[0, 0], [1, 1]] }); assert.ok(/class="snapring"/.test(mp) && /class="pen"/.test(mp) && /data-kind="curve"/.test(mp) && [...mp.matchAll(/data-kind="cp"/g)].length === n - 2);
  assert.ok(!/data-kind="point"/.test(mp));
});

console.log('\n' + (pass + fail) + ' checks: ' + pass + ' passed, ' + fail + ' failed');
process.exit(fail ? 1 : 0);
