// Tests track_designer_gui.html in headless Edge (no visible window) with trusted mouse input through the DevTools protocol.
//  - the built page is opened from a plain file:/// address with NO special switch, with the network off
//    (dead proxy + CDP offline mode; proven by a request to an outside address that must fail);
//  - the ORIGINAL export page is opened with --allow-file-access-from-files (network on, it needs unpkg) as
//    (a) the positive control for the name check and (b) the reference for the behavior comparison.
// Usage: node test_gui_trusted.mjs        Exit code 0 only if every check passes.
import fs from 'node:fs';
import os from 'node:os';
os.setPriority(0, os.constants.priority.PRIORITY_BELOW_NORMAL);   // the owner's PC is slow: the script and every Edge it starts run at low priority
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { launch, sleep, OUT, EXPORT, ROOT } from './lib.mjs';
import { pathToFileURL } from 'node:url';
fs.mkdirSync(ROOT + '_dl/', { recursive: true });

const BUILT = pathToFileURL(ROOT + 'track_designer_gui.html').href;
const ORIG = pathToFileURL(EXPORT + 'Track Designer.dc.html').href;
const SAMPLE = ROOT + 'ref/saved_sample_layout.json';
const sampleKeys = Object.keys(JSON.parse(fs.readFileSync(SAMPLE, 'utf8'))).sort();
const NAMES = /LSY|\bTUM\b|Munich|M\u00fcnchen|Crazyf|crazyflow|Technische|Universit/i;

const results = [];
const check = (name, ok, detail = '') => { results.push({ name, ok: !!ok, detail }); console.log((ok ? 'PASS ' : 'FAIL ') + name + (detail ? '  [' + detail + ']' : '')); };

// All text a person can see: body text, text inside every svg, and title / aria-label attributes.
const VISIBLE = `(()=>{const a=[document.body.innerText];document.querySelectorAll('svg text').forEach(t=>a.push(t.textContent));document.querySelectorAll('[title],[aria-label],[placeholder],img[alt]').forEach(e=>{['title','aria-label','placeholder','alt'].forEach(k=>{const v=e.getAttribute(k);if(v)a.push(v)})});document.querySelectorAll('textarea,input').forEach(e=>{if(e.value)a.push(e.value)});return a.join('\\n')})()`;
const ROWS = `[...document.querySelectorAll('table tbody tr')].slice(0,5).map(r=>r.innerText.replace(/\\s+/g,' '))`;
const NGATES = `document.querySelectorAll('svg[data-td=map] g[data-kind=gate][role=button]').length`;

async function ready(b, url) {
  await b.nav(url);
  return b.waitFor("document.querySelectorAll('svg[data-td=map]').length>0 && document.querySelector('svg[data-td=map]').children.length>10", 40000);
}
const ready0 = async (b, url, expr) => { await b.nav(url); return b.waitFor(expr, 40000); };
const gateCenter = (b, i) => b.ev(`(()=>{const g=document.querySelector('svg[data-td=map] [data-kind=gate][data-i="${i}"]');const r=g.getBoundingClientRect();return [r.left+r.width/2,r.top+r.height/2]})()`);
const toolCenter = (b, t) => b.ev(`(()=>{const e=document.querySelector('.tdtool[data-tool=${t}]').getBoundingClientRect();const m=document.querySelector('svg[data-td=map]').getBoundingClientRect();return [e.left+e.width/2,e.top+e.height/2,m.left+m.width*0.8,m.top+m.height*0.2,m.left+m.width*0.25,m.top+m.height*0.75]})()`);
const key = (b, k, code, vk) => b.send('Input.dispatchKeyEvent', { type: 'rawKeyDown', key: k, code, windowsVirtualKeyCode: vk }).then(() => b.send('Input.dispatchKeyEvent', { type: 'keyUp', key: k, code, windowsVirtualKeyCode: vk }));

async function saveLayout(b) {
  const dir = fs.mkdtempSync(ROOT + '_dl/gui_dl_');
  await b.send('Browser.setDownloadBehavior', { behavior: 'allow', downloadPath: dir });
  await b.clickText('button', 'Save layout');
  for (let i = 0; i < 100; i++) { const f = fs.existsSync(dir) ? fs.readdirSync(dir).filter(x => !x.endsWith('.crdownload')) : []; if (f.length) { const t = fs.readFileSync(path.join(dir, f[0]), 'utf8'); fs.rmSync(dir, { recursive: true, force: true }); return { name: f[0], text: t }; } await sleep(250); }
  return null;
}

// The Design-screen interaction script. Runs identically on the built page and on the original.
async function designRun(b) {
  const R = {};
  R.rows0 = await b.ev(ROWS); R.route0 = await b.ev(`document.getElementById('routeField').value`);
  const g0 = await gateCenter(b, 0);
  await b.drag(g0[0], g0[1], g0[0] + 60, g0[1] + 40); R.rowsDrag = await b.ev(ROWS);
  await b.clickText('button', 'Undo'); R.rowsUndo = await b.ev(ROWS);
  await b.clickText('button', 'Redo'); R.rowsRedo = await b.ev(ROWS);
  await b.clickText('button', 'Undo');
  const tb = await toolCenter(b, 'triangle');
  await b.drag(tb[0], tb[1], tb[2], tb[3]); R.afterTriangle = { gates: await b.ev(NGATES), status: await b.ev(`document.querySelector('[role=status]').textContent`) };
  const gA = await gateCenter(b, 1); await b.mm('mouseMoved', gA[0], gA[1]);
  await b.mm('mousePressed', gA[0], gA[1], { button: 'left', buttons: 1, clickCount: 1, mod: 2 });
  for (let k = 1; k <= 6; k++) await b.mm('mouseMoved', gA[0] + 10 * k, gA[1] - 10 * k, { button: 'left', buttons: 1, mod: 2 });
  await b.mm('mouseReleased', gA[0] + 60, gA[1] - 60, { button: 'left', buttons: 0, clickCount: 1, mod: 2 }); await sleep(300);
  R.afterCtrlDrag = await b.ev(NGATES);
  const gL = await gateCenter(b, R.afterCtrlDrag - 1);
  const tr = await b.ev(`(()=>{const e=document.querySelector('[data-td=trash]').getBoundingClientRect();return [e.left+e.width/2,e.top+e.height/2]})()`);
  await b.drag(gL[0], gL[1], tr[0], tr[1]); R.afterTrash = await b.ev(NGATES);
  R.rowsBeforeUndos = await b.ev(ROWS);
  for (let i = 0; i < 3; i++) await b.clickText('button', 'Undo');
  R.gatesRestored = await b.ev(NGATES); R.rowsRestored = await b.ev(ROWS);
  await b.clickText('button', 'Draw route'); await sleep(200);
  R.banner = await b.ev(`document.body.innerText.includes('Drawing the route')`);
  await key(b, 'Escape', 'Escape', 27); await sleep(200);
  R.routeAfterEsc = await b.ev(`document.getElementById('routeField').value`);
  await b.clickText('button', 'Undo'); R.routeAfterUndo = await b.ev(`document.getElementById('routeField').value`);
  R.save1 = await saveLayout(b);
  const te = await toolCenter(b, 'end'); await b.drag(te[0], te[1], te[4], te[5]);
  R.save2 = await saveLayout(b);
  return R;
}

// ---------- 1. THE BUILT PAGE: plain file:///, no switch, network off ----------
console.log('=== built page: ' + BUILT);
const b = await launch(['--proxy-server=127.0.0.1:9', '--proxy-bypass-list=<-loopback>'], 1440, 900);   // no special file switch
await b.send('Network.emulateNetworkConditions', { offline: true, latency: 0, downloadThroughput: 0, uploadThroughput: 0 });
const seenUrls = []; // every URL the page asked for, from the protocol log (set per navigation below)
await ready(b, BUILT).then(ms => check('built: page draws (map svg has content) from plain file:///', ms >= 0, ms + ' ms'));
await sleep(800);
const pageHasFlags = await b.ev(`({react: !!window.React && !!window.ReactDOM && window.React.version, td: typeof window.TD, hasLsy: document.documentElement.outerHTML.includes('LSY')})`);
check('built: React 18.3.1 present from the inlined copy, TD core present', pageHasFlags.react === '18.3.1' && pageHasFlags.td === 'object', JSON.stringify(pageHasFlags));
const screens = {
  Design: ['TOOLBOX', 'TOP VIEW', 'SIDE VIEW', 'INSPECTOR', 'Safe area', 'Safe height, 2 m', 'Draw route'],
  Simulate: ['TELEMETRY', 'SPLITS', 'Orbit', 'Playback', 'Play'],
  Runs: ['RUN HISTORY', 'Clear history', 'LAP', 'MIN CLEARANCE'],
  Settings: ['Download track_layout.json', 'Load layout', 'Safe area (real room)', 'Limits', 'Export'],
};
const shot = { Design: 'gui_design.png', Simulate: 'gui_simulate.png', Runs: 'gui_runs.png', Settings: 'gui_settings.png' };
const visibleBy = {};
let simTime = null;
const reqBefore = () => b.log.http.map(h => h.url);
for (const scr of ['Design', 'Simulate', 'Runs', 'Settings']) {
  if (scr !== 'Design') { await b.clickText('button', scr); await sleep(900); }
  if (scr === 'Simulate') {
    await b.clickText('button', 'Play'); await sleep(2500);
    const t = await b.ev(`document.body.innerText`); const m = t.match(/TIME\s+([0-9]+\.[0-9]+)/i); simTime = [m && parseFloat(m[1])];
    await sleep(1500); const t2 = (await b.ev(`document.body.innerText`)).match(/TIME\s+([0-9]+\.[0-9]+)/i); simTime.push(t2 && parseFloat(t2[1]));
    check('Simulate: after Play the time number grows', simTime[0] > 0 && simTime[1] > simTime[0], simTime.join(' -> ') + ' s');
  }
  const txt = await b.ev(`document.body.innerText`);
  const missing = screens[scr].filter(s => !txt.toLowerCase().includes(s.toLowerCase()));
  check(`${scr}: shows its own content (${screens[scr].join(' | ')})`, missing.length === 0, missing.length ? 'missing: ' + missing.join(', ') : '');
  visibleBy[scr] = await b.ev(VISIBLE);
  check(`${scr}: visible text holds none of LSY / TUM / Munich / Crazyf / Technische / Universit`, !NAMES.test(visibleBy[scr]), 'chars checked: ' + visibleBy[scr].length);
  await b.shot(shot[scr]);
}
b.resolveUrls();
const urls = b.log.http.map(h => h.url);
const outside = urls.filter(u => !/^(file:|data:|blob:|about:)/.test(u));
check('no request to any server during the four screens (all URLs are file:, data: or blob:)', outside.length === 0, urls.length + ' requests seen' + (outside.length ? '; outside: ' + outside.slice(0, 5).join(' ') : ''));
check('no failed request', b.log.failed.length === 0, b.log.failed.slice(0, 5).join(' | '));
check('no console error / warning, no exception (four screens)', b.log.console.length === 0 && b.log.exceptions.length === 0, [...b.log.console, ...b.log.exceptions].slice(0, 5).join(' | '));
const netProof = await b.ev(`fetch('https://example.com/', {mode:'no-cors', cache:'no-store'}).then(()=> 'REACHED', e => 'blocked: ' + e.message)`);
check('network is off (a request to https://example.com must fail)', netProof.startsWith('blocked'), netProof);
await b.close();

// Design interaction on a fresh load of the built page (so the earlier Play does not interfere)
const b2 = await launch(['--proxy-server=127.0.0.1:9', '--proxy-bypass-list=<-loopback>'], 1440, 900);
await b2.send('Network.emulateNetworkConditions', { offline: true, latency: 0, downloadThroughput: 0, uploadThroughput: 0 });
await ready(b2, BUILT); await sleep(800);
const RB = await designRun(b2);
b2.resolveUrls();
const out2 = b2.log.http.map(h => h.url).filter(u => !/^(file:|data:|blob:|about:)/.test(u));
check('Design run: no request to a server, no console error, no exception', out2.length === 0 && b2.log.console.length === 0 && b2.log.exceptions.length === 0 && b2.log.failed.length === 0, [...b2.log.console, ...b2.log.exceptions, ...b2.log.failed, ...out2].slice(0, 4).join(' | '));
await b2.close();

check('Design: dragging gate 1 changes its row', JSON.stringify(RB.rowsDrag) !== JSON.stringify(RB.rows0) && RB.rowsDrag[1] !== RB.rows0[1], RB.rows0[1] + '  ->  ' + RB.rowsDrag[1]);
check('Design: Undo brings the gate back', JSON.stringify(RB.rowsUndo) === JSON.stringify(RB.rows0), RB.rowsUndo[1]);
check('Design: Redo applies the move again', JSON.stringify(RB.rowsRedo) === JSON.stringify(RB.rowsDrag), RB.rowsRedo[1]);
check('Design: a triangle gate from the toolbox adds a gate (4 -> 5)', RB.afterTriangle.gates === 5, RB.afterTriangle.gates + ' gates; ' + RB.afterTriangle.status);
check('Design: Ctrl + drag copies a gate (5 -> 6)', RB.afterCtrlDrag === 6, String(RB.afterCtrlDrag));
check('Design: a drop on the trash area deletes a gate (6 -> 5)', RB.afterTrash === 5, String(RB.afterTrash));
check('Design: "Draw route" shows its banner', RB.banner === true);
check('Design: three Undo steps restore the 4 gates', RB.gatesRestored === 4);
check('Save layout (no end pad) downloads track_layout.json whose keys are the sample keys minus "end"', RB.save1 && RB.save1.name === 'track_layout.json' && JSON.stringify(Object.keys(JSON.parse(RB.save1.text)).sort()) === JSON.stringify(sampleKeys.filter(k => k !== 'end')), RB.save1 ? Object.keys(JSON.parse(RB.save1.text)).length + ' keys' : 'no download');
check('Save layout (end pad placed) downloads a file whose keys equal those of saved_sample_layout.json', RB.save2 && JSON.stringify(Object.keys(JSON.parse(RB.save2.text)).sort()) === JSON.stringify(sampleKeys), RB.save2 ? Object.keys(JSON.parse(RB.save2.text)).length + ' keys vs ' + sampleKeys.length : 'no download');

// ---------- 2. THE ORIGINAL: positive control + behavior comparison ----------
console.log('=== original export page: ' + ORIG);
const o = await launch(['--allow-file-access-from-files'], 1440, 900);
const okO = await ready(o, ORIG); await sleep(1500);
check('original: page draws with --allow-file-access-from-files (needs the internet for React)', okO >= 0, okO + ' ms');
const origVisible = await o.ev(VISIBLE);
const origHits = [...origVisible.matchAll(/.*(LSY|\bTUM\b|Munich|Crazyf|Technische|Universit).*/gi)].map(m => m[0].trim());
check('POSITIVE CONTROL: the same name check DOES find "LSY" on the original page', NAMES.test(origVisible) && /LSY/.test(origVisible), 'found: ' + [...new Set(origHits)].join(' | '));
await o.close();
const o2 = await launch(['--allow-file-access-from-files'], 1440, 900);
await ready(o2, ORIG); await sleep(800);
const RO = await designRun(o2);
await o2.close();
const same = (k, f = x => x) => JSON.stringify(f(RB[k])) === JSON.stringify(f(RO[k]));
check('same as original: rows before the drag', same('rows0'), RB.rows0[1]);
check('same as original: rows after the drag (gate positions)', same('rowsDrag'), RB.rowsDrag[1] + ' | ' + RO.rowsDrag[1]);
check('same as original: rows after Undo and after Redo', same('rowsUndo') && same('rowsRedo'));
check('same as original: route text (start, after Esc, after Undo)', same('route0') && same('routeAfterEsc') && same('routeAfterUndo'), JSON.stringify([RB.route0, RB.routeAfterEsc, RB.routeAfterUndo]));
check('same as original: gate counts after triangle / Ctrl+drag / trash / undo', same('afterCtrlDrag') && same('afterTrash') && same('gatesRestored') && RB.afterTriangle.gates === RO.afterTriangle.gates && RB.afterTriangle.status === RO.afterTriangle.status);
check('same as original: rows after trash (all 5 rows)', same('rowsBeforeUndos'));
const normV = s => s.replace(/"designer_version": "[^"]*"/, '"designer_version": "-"');
const lineDiff = (a, c) => { const x = a.split('\n'), y = c.split('\n'); const d = []; for (let i = 0; i < Math.max(x.length, y.length); i++) if (x[i] !== y[i]) d.push((x[i] || '').trim() + ' => ' + (y[i] || '').trim()); return d; };
check('same as original: saved file (no end pad) is identical except the line designer_version (3.0 there, 4.0 here: the core is our present one)', normV(RB.save1.text) === normV(RO.save1.text), 'differing lines: ' + JSON.stringify(lineDiff(RB.save1.text, RO.save1.text)));
check('same as original: saved file (end pad) is identical except the line designer_version', normV(RB.save2.text) === normV(RO.save2.text), 'differing lines: ' + JSON.stringify(lineDiff(RB.save2.text, RO.save2.text)));


// ====================================================================================================
// SLICE 2: THE PEN FOR THE FLIGHT PATH (built page: plain file:///, network off, trusted mouse input)
// ====================================================================================================
console.log('=== slice 2: the pen');
const SRC_CORE = ROOT + 'src/td-core.js';
const TDN = new Function('window', fs.readFileSync(SRC_CORE, 'utf8').replace('export default TD;', '') + ';return TD')({});
// the paths (meters, x to the right, y up) were designed with the core of today's layout; see README.txt
const g1in = [0.144, 0.602], g1out = [0.856, -0.1], g2in = [1.402, 0.394], g2out = [0.7, 1.1];
const PATHS = {
  P12: [[-1.4, 0.8], [-0.6, 0.85], g1in, [0.5, 0.25], g1out, [1.45, 0.05], g2in, [1.05, 0.75], g2out],       // gate 1 then gate 2, along their arrows
  PM1: [[-1.3, -0.9], [-0.5, -1.3], [0.5, -1.3], [1.3, -0.9], g1out, [0.5, 0.25], g1in, [-0.2, 1.0]],         // gate 1 against its arrow
  PFR: [[-1.0, 1.0], [0.0, 0.9], [0.486, 0.66], [0.91, 0.236], [1.3, -0.4]],                                  // through the FRAME of gate 1, not its opening
  PNO: [[-1.0, 1.0], [-0.5, 1.2], [0.5, 1.4]],                                                               // beside every gate
};
const densify = (pts, step = 0.02) => { const out = [pts[0]]; for (let i = 1; i < pts.length; i++) { const a = pts[i - 1], b = pts[i], n = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / step)); for (let k = 1; k <= n; k++) out.push([a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n]); } return out; };
// meters -> client pixels, for the map svg selected by `sel`
const toPx = (b, sel, pts) => b.ev(`(()=>{const m=document.querySelector(${JSON.stringify(sel)});const out=[];for(const q of ${JSON.stringify(pts)}){const pt=m.createSVGPoint();pt.x=q[0];pt.y=-q[1];const p=pt.matrixTransform(m.getScreenCTM());out.push([p.x,p.y])}return out})()`);
async function penDraw(b, sel, pts, o = {}) {
  const px = await toPx(b, sel, densify(pts));
  const inside = px.every(p => p[0] > 0 && p[1] > 0 && p[0] < b.W && p[1] < b.H);
  if (!inside) throw new Error('a point of the path lies outside the window');
  await b.mm('mouseMoved', px[0][0], px[0][1]); await b.mm('mousePressed', px[0][0], px[0][1], { button: 'left', buttons: 1, clickCount: 1 });
  const half = Math.floor(px.length / 2);
  for (let i = 1; i < px.length; i++) {
    await b.mm('mouseMoved', px[i][0], px[i][1], { button: 'left', buttons: 1 });
    if (i % 12 === 0) await sleep(8);
    if (o.midShot && i === half) { await sleep(300); await b.shot(o.midShot); }
  }
  const l = px[px.length - 1]; await b.mm('mouseReleased', l[0], l[1], { button: 'left', buttons: 0, clickCount: 1 }); await sleep(350);
}
const saveJson = async b => { let s = await saveLayout(b); if (!s) { await sleep(1500); s = await saveLayout(b); } if (!s) throw new Error('Save layout gave no file; status: ' + await b.ev(`document.querySelector('[role=status]').textContent`)); return JSON.parse(s.text); };
const gateEntries = txt => txt.split(',').map(s => s.trim()).filter(s => /^-?\d+$/.test(s)).map(Number);
const maxDiff = (A, B) => { if (!A || !B || A.length !== B.length) return NaN; let m = 0; A.forEach((p, i) => p.forEach((v, k) => { m = Math.max(m, Math.abs(v - B[i][k])); })); return m; };
const distToPoly = (p, S) => { let best = 1e9; for (let i = 0; i + 1 < S.length; i++) { const a = S[i], c = S[i + 1], dx = c.x - a.x, dy = c.y - a.y, l2 = dx * dx + dy * dy || 1e-12, t = Math.max(0, Math.min(1, ((p[0] - a.x) * dx + (p[1] - a.y) * dy) / l2)); best = Math.min(best, Math.hypot(p[0] - (a.x + t * dx), p[1] - (a.y + t * dy))); } return best; };
const MAP = 'svg[data-td=map]';
const penState = b => b.ev(`({banner: document.body.innerText.includes('Drawing the flight path'), route: document.getElementById('routeField').value, routeDisabled: document.getElementById('routeField').disabled, dots: document.querySelectorAll('svg[data-td=map] g[data-kind=cp]').length, marks: document.querySelectorAll('svg[data-td=map] .badx').length, markText: [...document.querySelectorAll('svg[data-td=map] .badt')].map(e=>e.textContent), curve: (document.querySelector('svg[data-td=map] polyline[data-kind=curve]')||{getAttribute:()=>null}).getAttribute('points'), msg: document.getElementById('routeField').parentElement.querySelector('p').textContent, btn: document.querySelector('[data-td=penBtn]').textContent})`);

const p = await launch(['--proxy-server=127.0.0.1:9', '--proxy-bypass-list=<-loopback>'], 1440, 900);
await p.send('Network.emulateNetworkConditions', { offline: true, latency: 0, downloadThroughput: 0, uploadThroughput: 0 });
await ready(p, BUILT); await sleep(800);
const S0 = await penState(p);
check('pen: the button "Draw flight path" stands beside "Draw route"', S0.btn === 'Draw flight path' && await p.ev(`(()=>{const a=[...document.querySelectorAll('button')].map(x=>x.textContent.trim());return a.indexOf('Draw flight path')===a.indexOf('Draw route')+1})()`), S0.btn);
check('pen: no banner before the pen is on', S0.banner === false);
await p.clickText('button', 'Draw flight path'); await sleep(250);
const S1 = await penState(p);
check('pen: the banner "Drawing the flight path" shows while the pen is on, and the button says Finish path', S1.banner === true && S1.btn === 'Finish path', S1.btn);
// ---- the path through gate 1 then gate 2, along their arrows (half-way picture while the button is held)
await penDraw(p, MAP, PATHS.P12, { midShot: 'gui_design_pen_drawing.png' });
const A = await penState(p); const saveA = await saveJson(p);
check('pen: a path through gate 1 then gate 2 along their arrows gives the gate order 1, 2 (saved gate_order)', saveA && JSON.stringify(saveA.gate_order) === '[1,2]', saveA && JSON.stringify(saveA.gate_order));
check('pen: the route field shows that order (gate numbers 1, 2 only) and is locked while a path exists', JSON.stringify(gateEntries(A.route)) === '[1,2]' && A.routeDisabled === true, A.route + ' | disabled=' + A.routeDisabled);
check('pen: no red mark for a path that goes through the openings', A.marks === 0 && saveA && saveA.route.every(e => e.type !== 'gate' || e.reverse === false), 'marks=' + A.marks);
const lastPt = saveA && saveA.path.points[saveA.path.points.length - 1];
check('pen: the saved file holds path (control points) and the end pad sits at the last point of the path', saveA && Array.isArray(saveA.path.points) && saveA.path.points.length >= 4 && saveA.end && Math.abs(saveA.end.pos[0] - lastPt[0]) < 1e-9 && Math.abs(saveA.end.pos[1] - lastPt[1]) < 1e-9 && Math.abs(lastPt[0] - PATHS.P12[PATHS.P12.length - 1][0]) < 0.03 && Math.abs(lastPt[1] - PATHS.P12[PATHS.P12.length - 1][1]) < 0.03, saveA && ('last point ' + JSON.stringify(lastPt) + ', end ' + JSON.stringify(saveA.end.pos)));
check('pen: the first control point is the start pad', saveA && JSON.stringify(saveA.path.points[0].slice(0, 2)) === JSON.stringify(saveA.start.pos.slice(0, 2)));
check('pen: a dot is shown for every control point between the pads', A.dots === saveA.path.points.length - 2, A.dots + ' dots, ' + saveA.path.points.length + ' points');
// ---- the file against the core: the same layout computed from the saved file gives the same gates
const LA = TDN.layoutFromText(JSON.stringify(saveA)); const cA = TDN.pathCompute(LA);
check('pen: the saved path, read back by the core, passes gates 1 and 2 only, with no frame touch', JSON.stringify(cA.openings.map(o => o.sign * o.gate)) === '[1,2]' && cA.frames.length === 0);
// ---- dragging a dot (the pen is switched off first)
const dotBefore = JSON.stringify(saveA.path.points);
await p.clickText('button', 'Finish path'); await sleep(200);
const d1 = await p.ev(`(()=>{const g=document.querySelector('svg[data-td=map] g[data-kind=cp][data-i="1"]');const r=g.getBoundingClientRect();return [r.left+r.width/2,r.top+r.height/2]})()`);
await p.drag(d1[0], d1[1], d1[0] + 45, d1[1] + 50); await sleep(300);
const B = await penState(p); const saveB = await saveJson(p);
check('pen: dragging dot 1 changes the saved path (that dot moved by about 0.36 m, 0.40 m)', saveB && JSON.stringify(saveB.path.points) !== dotBefore && Math.hypot(saveB.path.points[1][0] - saveA.path.points[1][0] - 45 / 125, saveB.path.points[1][1] - saveA.path.points[1][1] + 50 / 125) < 0.1, saveA.path.points[1] + ' -> ' + saveB.path.points[1]);
check('pen: dragging that dot changes the curve in the top view', B.curve !== A.curve);
await p.clickText('button', 'Undo'); await sleep(250);
const C = await penState(p); const saveC = await saveJson(p);
check('pen: Undo puts the dot back (the saved path equals the one before the drag)', JSON.stringify(saveC.path.points) === dotBefore && C.curve === A.curve);
await p.clickText('button', 'Redo'); await sleep(250);
const saveD = await saveJson(p);
check('pen: Redo moves the dot again', JSON.stringify(saveD.path.points) === JSON.stringify(saveB.path.points));
await p.clickText('button', 'Undo'); await p.clickText('button', 'Undo'); await sleep(250);          // dot drag, then the stroke
const E0 = await penState(p); const saveE = await saveJson(p);
check('pen: a second Undo takes the whole path away (no dots, no "path" in the file, the first route is back)', E0.dots === 0 && saveE && !('path' in saveE) && JSON.stringify(saveE.gate_order) === '[1,2,3,4,2]' && E0.routeDisabled === false, 'dots=' + E0.dots + ', gate_order ' + JSON.stringify(saveE && saveE.gate_order));
await p.clickText('button', 'Redo'); await sleep(250);
const F0 = await penState(p); const saveF = await saveJson(p);
check('pen: Redo brings the path back (same control points, same gate order 1, 2)', saveF && JSON.stringify(saveF.path.points) === dotBefore && JSON.stringify(saveF.gate_order) === '[1,2]' && F0.dots === A.dots);

// ---- Simulate follows the drawn path (the saved path is in the layout now)
const layoutForSim = TDN.layoutFromText(JSON.stringify(saveF));
const drawnCurve = TDN.pathGeometry(layoutForSim, 32).samples;
const clickRoute = (() => { const Lr = TDN.layoutFromText(JSON.stringify(saveF)); Lr.path = null; Lr.route = [{ type: 'gate', gate: 1, reverse: false }, { type: 'gate', gate: 2, reverse: false }]; return TDN.routeGeometry(Lr, 32).samples; })();
// the drawn path passes 0.07 m from pole P1 (the hit rule would stop the flight there): delete P1 with the mouse and the Delete key, so that this check follows the whole path
const pc = await p.ev(`(()=>{const g=document.querySelector('svg[data-td=map] g[data-kind=pole][data-i="0"] .pole');const r=g.getBoundingClientRect();return [r.left+r.width/2,r.top+r.height/2]})()`);
await p.click(pc[0], pc[1]); await key(p, 'Delete', 'Delete', 46); await sleep(300);
check('pen: pole P1 (0.07 m from the drawn path) is deleted with mouse and Delete key before the Simulate check', (await saveJson(p)).obstacles.length === 3);
await p.clickText('button', 'Simulate'); await sleep(700);
const posNow = async () => { const t = await p.ev(`document.body.innerText`); const m = t.match(/POSITION\s+(-?[0-9.]+),\s*(-?[0-9.]+)/i) || t.match(/Position\s+(-?[0-9.]+),\s*(-?[0-9.]+)/); return m ? [parseFloat(m[1]), parseFloat(m[2])] : null; };
await p.clickText('button', 'Play'); await sleep(300); await p.clickText('button', 'Pause'); await sleep(300);
const q0 = await posNow();
await p.clickText('button', 'Resume'); await sleep(450); await p.clickText('button', 'Pause'); await sleep(300);
const q1 = await posNow(); await p.shot('gui_simulate_pen_path.png');
await p.clickText('button', 'Resume'); await sleep(450); await p.clickText('button', 'Pause'); await sleep(300);
const q2 = await posNow();
const near = q => ({ drawn: distToPoly(q, drawnCurve), click: distToPoly(q, clickRoute) });
const ns = [q0, q1, q2].map(near);
check('Simulate: after Play with a drawn path the drone is ON the drawn path (distance to the curve at three moments, tolerance 0.02 m, and it moves)', ns.every(n => n.drawn < 0.02) && Math.hypot(q0[0] - q1[0], q0[1] - q1[1]) > 0.3 && Math.hypot(q1[0] - q2[0], q1[1] - q2[1]) > 0.3, 'positions ' + JSON.stringify([q0, q1, q2]) + '; distances ' + ns.map(n => n.drawn.toFixed(4)).join(', '));
check('Simulate: the control can fail: at least one of these positions is NOT on the old click-the-gates curve (gates 1, 2, end pad) by more than 0.04 m', Math.max(...ns.map(n => n.click)) > 0.04, ns.map(n => n.click.toFixed(3)).join(', '));
const simErr = [...p.log.console, ...p.log.exceptions];
await p.clickText('button', 'Design'); await sleep(500);

// ---- gate against its arrow, red mark, a path beside all gates (each on a cleared route)
const redo = async (name, pts, o) => { await p.clickText('button', 'Clear route'); await p.clickText('button', 'Draw flight path'); await sleep(150); await penDraw(p, MAP, pts, o); const st = await penState(p); const sv = await saveJson(p); await p.clickText('button', 'Finish path'); await sleep(150); return { st, sv }; };
const M1 = await redo('PM1', PATHS.PM1);
check('pen: the same kind of path through gate 1 against its arrow gives -1 (saved gate_order; the route entry says reverse)', M1.sv && JSON.stringify(M1.sv.gate_order) === '[-1]' && M1.sv.route.some(e => e.type === 'gate' && e.gate === 1 && e.reverse === true) && JSON.stringify(gateEntries(M1.st.route)) === '[-1]', M1.sv && JSON.stringify(M1.sv.gate_order) + ' | ' + M1.st.route);
check('pen: ... and no red mark for it', M1.st.marks === 0);
const FR = await redo('PFR', PATHS.PFR, {});
check('pen: a path that crosses the frame of gate 1 shows the red mark, named in the notes (and the gate is not in the order)', FR.st.marks === 1 && FR.st.markText.some(t => /gate 1/.test(t)) && /Red mark/.test(FR.st.msg) && JSON.stringify(FR.sv.gate_order) === '[]', 'marks=' + FR.st.marks + ' ' + JSON.stringify(FR.st.markText) + ' | ' + FR.st.msg.slice(0, 90));
await p.shot('gui_design_pen_done.png');
const NO = await redo('PNO', PATHS.PNO);
check('pen: a path that passes beside every gate shows no red mark and an empty gate order', NO.st.marks === 0 && JSON.stringify(NO.sv.gate_order) === '[]', 'marks=' + NO.st.marks);
// the pen leaves pen mode on Esc; a click on the curve does not draw
await p.clickText('button', 'Draw flight path'); await sleep(100); await key(p, 'Escape', 'Escape', 27); await sleep(200);
check('pen: Esc ends the pen (banner gone)', (await penState(p)).banner === false);
p.resolveUrls();
const outP = p.log.http.map(h => h.url).filter(u => !/^(file:|data:|blob:|about:)/.test(u));
check('pen: no request to a server, no console error, no exception during all pen steps', outP.length === 0 && p.log.console.length === 0 && p.log.exceptions.length === 0 && p.log.failed.length === 0, [...p.log.console, ...p.log.exceptions, ...p.log.failed, ...outP].slice(0, 3).join(' | '));
await p.close();

// ---- the SAME path in our present page (track_designer.html) and in the new GUI: the files are compared
const PRESENT = pathToFileURL(ROOT + 'ref/track_designer.html').href;
const pr = await launch([], 1440, 1400);
await ready0(pr, PRESENT, "document.querySelectorAll('svg#map').length>0 && document.querySelector('svg#map').children.length>10");
await sleep(600);
check('present page: it is the fourth round (has the button "Draw flight path")', await pr.ev(`!!document.getElementById('btnPen') && document.getElementById('btnPen').textContent.trim()==='Draw flight path'`));
const savePresent = async (b2) => { const dir = fs.mkdtempSync(ROOT + '_dl/gui_dl_'); await b2.send('Browser.setDownloadBehavior', { behavior: 'allow', downloadPath: dir }); await b2.ev(`document.getElementById('btnSave').click()`); for (let i = 0; i < 40; i++) { const f = fs.readdirSync(dir).filter(x => !x.endsWith('.crdownload')); if (f.length) { const t = fs.readFileSync(path.join(dir, f[0]), 'utf8'); fs.rmSync(dir, { recursive: true, force: true }); return JSON.parse(t); } await sleep(250); } return null; };
const cmpRows = [], presentFiles = {};
for (const [name, pts] of Object.entries({ P12: PATHS.P12, PM1: PATHS.PM1, PFR: PATHS.PFR })) {
  await pr.ev(`document.getElementById('btnClearPath').click()`); await sleep(200);
  await pr.ev(`document.getElementById('btnPen').click()`); await sleep(200);
  await penDraw(pr, 'svg#map', pts); await pr.ev(`document.getElementById('btnPen').click()`); await sleep(200);
  presentFiles[name] = await savePresent(pr);
}
await pr.close();                                            // only one browser at a time
for (const [name, pts] of Object.entries({ P12: PATHS.P12, PM1: PATHS.PM1, PFR: PATHS.PFR })) {
  const fp = presentFiles[name];
  const g = await launch(['--proxy-server=127.0.0.1:9', '--proxy-bypass-list=<-loopback>'], 1440, 900);
  await g.send('Network.emulateNetworkConditions', { offline: true, latency: 0, downloadThroughput: 0, uploadThroughput: 0 });
  await ready(g, BUILT); await sleep(500);
  await g.clickText('button', 'Draw flight path'); await sleep(150); await penDraw(g, MAP, pts); await g.clickText('button', 'Finish path'); await sleep(200);
  const fg = await saveJson(g); await g.close();
  const keysEq = JSON.stringify(Object.keys(fp).sort()) === JSON.stringify(Object.keys(fg).sort());
  const dPath = maxDiff(fp.path && fp.path.points, fg.path && fg.path.points);
  const dEnd = maxDiff([fp.end.pos], [fg.end.pos]);
  cmpRows.push({ name, keysEq, nPresent: fp.path.points.length, nGui: fg.path.points.length, dPath, dEnd, orderP: fp.gate_order, orderG: fg.gate_order, routeEq: JSON.stringify(fp.route.map(e => e.type + (e.gate || '') + (e.reverse ? 'r' : ''))) === JSON.stringify(fg.route.map(e => e.type + (e.gate || '') + (e.reverse ? 'r' : ''))), whole: JSON.stringify(fp) === JSON.stringify(fg) });
}
for (const r of cmpRows) {
  check(`same as our present page, path ${r.name}: same keys, same number of control points, same gate_order, same route (gates and direction)`, r.keysEq && r.nPresent === r.nGui && JSON.stringify(r.orderP) === JSON.stringify(r.orderG) && r.routeEq, `points ${r.nPresent} vs ${r.nGui}; gate_order ${JSON.stringify(r.orderP)} vs ${JSON.stringify(r.orderG)}`);
  check(`same as our present page, path ${r.name}: control points differ by at most 0.02 m (mouse pixels differ between the two pages)`, r.dPath <= 0.02 && r.dEnd <= 0.02, `max difference of path points ${r.dPath.toFixed(4)} m, of the end pad ${r.dEnd.toFixed(4)} m; whole file identical: ${r.whole}`);
}

// ---- the logic checks of test_designer.mjs that concern the path, the crossings and the gate order, run on the new core
const lc = spawnSync('node', [ROOT + 'test_core_path.mjs'], { encoding: 'utf8' });
const lm = (lc.stdout || '').match(/(\d+) checks: (\d+) passed, (\d+) failed/);
console.log((lc.stdout || '').split('\n').filter(l => /^FAIL|checks:/.test(l)).join('\n'));
check('ported logic checks (path, crossings, gate order) all pass on the new core', lm && +lm[3] === 0 && +lm[1] >= 25, lm ? lm[2] + ' of ' + lm[1] + ' pass' : 'did not run: ' + (lc.stderr || '').slice(0, 200));

// ====================================================================================================
// SLICE 2b: THE FLIGHT STOPS AT A POLE THAT THE DRONE HITS  (built page, plain file:///, network off, trusted mouse)
// ====================================================================================================
console.log('=== slice 2b: the hit rule');
const POLE_R = 0.015, DRONE_HALF = 0.07, DRONE_BELOW = 0.02, HITR = POLE_R + DRONE_HALF;
// OWN calculation (not the page's code path): the speed profile of the preview, re-written from its description (speed from the curvature
// with a lateral limit, 3.5 m/s2 up and down, 0.2 m/s at the end pad), then a brute-force walk along the samples in 1 mm steps.
function ownTiming(L, S, vmax = 3, alat = 4) {
  const n = S.length, dist = [0]; for (let i = 1; i < n; i++) dist.push(dist[i - 1] + Math.hypot(S[i].x - S[i - 1].x, S[i].y - S[i - 1].y, S[i].z - S[i - 1].z));
  const v = new Array(n).fill(vmax);
  for (let i = 1; i < n - 1; i++) { const ax = S[i].x - S[i - 1].x, ay = S[i].y - S[i - 1].y, bx = S[i + 1].x - S[i].x, by = S[i + 1].y - S[i].y, la = Math.hypot(ax, ay), lb = Math.hypot(bx, by); if (la < 1e-6 || lb < 1e-6) continue; const ang = Math.acos(Math.max(-1, Math.min(1, (ax * bx + ay * by) / (la * lb)))), k = ang / ((la + lb) / 2); if (k > 1e-6) v[i] = Math.min(vmax, Math.max(0.5, Math.sqrt(alat / k))); }
  v[0] = 0; v[n - 1] = L.end ? 0.2 : v[n - 1];
  for (let i = 1; i < n; i++) v[i] = Math.min(v[i], Math.sqrt(v[i - 1] ** 2 + 2 * 3.5 * (dist[i] - dist[i - 1])));
  for (let i = n - 2; i >= 0; i--) v[i] = Math.min(v[i], Math.sqrt(v[i + 1] ** 2 + 2 * 3.5 * (dist[i + 1] - dist[i])));
  const time = [0]; for (let i = 1; i < n; i++) time.push(time[i - 1] + (dist[i] - dist[i - 1]) / Math.max(0.15, (v[i] + v[i - 1]) / 2));
  return { time, dist };
}
function ownFirstHit(L, S, time) {
  let best = null;
  L.obstacles.forEach((pl, k) => {
    for (let i = 1; i < S.length && !(best && time[i - 1] > best.t); i++) {
      const a = S[i - 1], b = S[i], len = Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z), steps = Math.max(1, Math.ceil(len / 0.001));
      for (let s = 0; s <= steps; s++) { const u = s / steps, x = a.x + (b.x - a.x) * u, y = a.y + (b.y - a.y) * u, z = a.z + (b.z - a.z) * u;
        if (Math.hypot(x - pl.pos[0], y - pl.pos[1]) < HITR && z - DRONE_BELOW < pl.pos[2]) { const t = time[i - 1] + u * (time[i] - time[i - 1]); if (!best || t < best.t) best = { t, x, y, z, pole: k }; break; } }
      if (best && best.pole === k) break;
    }
  });
  return best;
}
const hudRead = async b => b.ev(`(()=>{const t=document.body.innerText;const tm=t.match(/TIME\\s+([0-9.]+)/i), ps=t.match(/Position\\s+(-?[0-9.]+),\\s*(-?[0-9.]+)/);const pre=document.querySelector('[data-td=preLine]');const pas=t.match(/(?:Lap [0-9.]+ s|Stopped: hit pole P\\d at [0-9.]+ s), (\\d+) passages/);return {time: tm?parseFloat(tm[1]):null, pos: ps?[parseFloat(ps[1]),parseFloat(ps[2])]:null, pre: pre?pre.textContent:'', passages: pas?parseInt(pas[1]):null, status: document.querySelector('[role=status]').textContent, body: t}})()`);
async function poleCase(label, setup, o = {}) {
  const b = await launch(['--proxy-server=127.0.0.1:9', '--proxy-bypass-list=<-loopback>'], 1440, 900);
  await b.send('Network.emulateNetworkConditions', { offline: true, latency: 0, downloadThroughput: 0, uploadThroughput: 0 });
  await ready(b, BUILT); await sleep(600);
  const baseLayout = await saveJson(b);
  const baseGeo = TDN.routeGeometry(TDN.layoutFromText(JSON.stringify(baseLayout)), 32).samples;
  await setup(b, baseGeo, baseLayout);                                          // mouse work on the Design screen
  const saved = await saveJson(b), L = TDN.layoutFromText(JSON.stringify(saved));
  const S = TDN.routeGeometry(L, 32).samples, T = ownTiming(L, S), exp = ownFirstHit(L, S, T.time);
  await b.clickText('button', 'Simulate'); await sleep(700);
  const pre = await hudRead(b);
  let st = null;
  if (o.play !== false) {
    await b.clickText('button', 'Play');
    const done = await b.waitFor(`/Stopped: hit pole|Run finished in/.test(document.querySelector('[role=status]').textContent)`, 25000);
    await sleep(400); st = await hudRead(b); st.done = done >= 0;
    if (o.shot) await b.shot(o.shot);
  }
  let runs = null;
  if (st) { await b.clickText('button', 'Runs'); await sleep(600); runs = await b.ev(`(()=>{const r=document.querySelector('table tbody tr');return r?r.innerText.replace(/\\s+/g,' '):''})()`); }
  let reset = null;
  if (st && o.reset) { await b.clickText('button', 'Simulate'); await sleep(500); await b.clickText('button', 'Reset'); await sleep(500); reset = await hudRead(b); }
  b.resolveUrls(); const errs = [...b.log.console, ...b.log.exceptions, ...b.log.failed];
  await b.close();
  return { label, saved, L, exp, pre, st, runs, reset, errs, T };
}
// a point on the default path at about 1.0 m from the start, its direction, and a side step
const pickOnPath = (geo, arc) => { let s = 0; for (let i = 1; i < geo.length; i++) { const d = Math.hypot(geo[i].x - geo[i - 1].x, geo[i].y - geo[i - 1].y); if (s + d >= arc) { const u = (arc - s) / d, x = geo[i - 1].x + (geo[i].x - geo[i - 1].x) * u, y = geo[i - 1].y + (geo[i].y - geo[i - 1].y) * u, z = geo[i - 1].z + (geo[i].z - geo[i - 1].z) * u, dx = (geo[i].x - geo[i - 1].x) / d, dy = (geo[i].y - geo[i - 1].y) / d; return { x, y, z, dx, dy }; } s += d; } return null; };
async function movePole(b, k, mx, my) {            // drag pole k (0-based) with the mouse to the meter point (mx, my)
  const c = await b.ev(`(()=>{const g=document.querySelector('svg[data-td=map] g[data-kind=pole][data-i="${k}"] .pole')||document.querySelector('svg[data-td=map] g[data-kind=pole][data-i="${k}"] .hit');const r=g.getBoundingClientRect();return [r.left+r.width/2,r.top+r.height/2]})()`);
  const tgt = (await toPx(b, MAP, [[mx, my]]))[0]; await b.drag(c[0], c[1], tgt[0], tgt[1]); await sleep(250);
  return tgt;
}
async function lowerPole(b, k, px, steps) {         // select the pole with a click, then turn the mouse wheel down over it
  await b.click(px[0], px[1]);
  for (let i = 0; i < steps; i++) { await b.mm('mouseWheel', px[0], px[1], { dy: 100 }); await sleep(15); }
  await sleep(300);
}
const ARC = 1.0;
// ---- case 1: pole P2 on the path between the start and gate 1
const C1 = await poleCase('hit', async (b, geo) => { const q = pickOnPath(geo, ARC); await movePole(b, 1, q.x, q.y); }, { shot: 'gui_simulate_hit_pole.png', reset: true });
const P2 = C1.saved.obstacles[1].pos;
console.log('      case 1: pole P2 now at', JSON.stringify(P2), '; own calculation:', JSON.stringify(C1.exp && { t: +C1.exp.t.toFixed(3), x: +C1.exp.x.toFixed(3), y: +C1.exp.y.toFixed(3), pole: C1.exp.pole }), '; page line:', C1.pre.pre);
check('hit: my own calculation (brute force, 1 mm steps) finds a hit on pole P2 on the path', C1.exp && C1.exp.pole === 1, JSON.stringify(C1.exp && { t: C1.exp.t, pole: C1.exp.pole }));
const mPre = C1.pre.pre.match(/hits pole (P\d) after ([0-9.]+) s\. The flight stops there\./);
check('hit: before Play the line names the pole and a time, and the time equals my own calculation (within 0.02 s)', !!mPre && mPre[1] === 'P2' && Math.abs(parseFloat(mPre[2]) - C1.exp.t) <= 0.02, C1.pre.pre + ' | own ' + (C1.exp && C1.exp.t.toFixed(3)));
check('hit: before Play the time number is 0.00 and the drone has not moved', C1.pre.time === 0, String(C1.pre.time));
check('hit: after Play the flight ended with a stop (status "Stopped: hit pole P2 at ... s")', C1.st.done && /^Stopped: hit pole P2 at [0-9.]+ s$/.test(C1.st.status), C1.st.status);
check('hit: the time number stops at the hit time (equal to the line before Play and to my calculation, within 0.02 s)', mPre && Math.abs(C1.st.time - parseFloat(mPre[2])) <= 0.01 && Math.abs(C1.st.time - C1.exp.t) <= 0.02, 'time ' + C1.st.time + ', line ' + (mPre && mPre[2]) + ', own ' + C1.exp.t.toFixed(3));
check('hit: the drone stands exactly at the hit point (position equals my calculation, within 0.02 m)', C1.st.pos && Math.hypot(C1.st.pos[0] - C1.exp.x, C1.st.pos[1] - C1.exp.y) <= 0.02, 'page ' + JSON.stringify(C1.st.pos) + ', own ' + JSON.stringify([+C1.exp.x.toFixed(3), +C1.exp.y.toFixed(3)]));
check('hit: no passage is counted (0 of 5): gate 1 and gate 2 are not reached', C1.st.passages === 0 && /0 of 5 passages made/.test(C1.st.body), 'passages ' + C1.st.passages);
check('hit: the Runs screen lists the run as not ok and names the pole ("Hit pole P2")', /Hit pole P2/.test(C1.runs) && !/Clean/.test(C1.runs), C1.runs);
check('hit: Reset puts the drone back at the start (time 0.00, position of the start pad)', C1.reset && C1.reset.time === 0 && Math.abs(C1.reset.pos[0] - C1.saved.start.pos[0]) < 0.01 && Math.abs(C1.reset.pos[1] - C1.saved.start.pos[1]) < 0.01, JSON.stringify(C1.reset && [C1.reset.time, C1.reset.pos]));
// ---- control 1: the pole 0.5 m beside the path
const C2 = await poleCase('beside 0.5', async (b, geo) => { const q = pickOnPath(geo, ARC); await movePole(b, 1, q.x - q.dy * 0.5, q.y + q.dx * 0.5); }, {});
const dist2 = Math.min(...(() => { const S = TDN.routeGeometry(C2.L, 32).samples, o = C2.saved.obstacles[1].pos; return S.map(p => Math.hypot(p.x - o[0], p.y - o[1])); })());
check('control 1: a pole 0.5 m beside the path (' + dist2.toFixed(2) + ' m after snapping): no hit line before Play, no stop, the lap ends with 5 of 5 passages', dist2 > 0.4 && !/hits pole/.test(C2.pre.pre) && /^Run finished in/.test(C2.st.status) && C2.st.passages === 5, C2.st.status + ' | passages ' + C2.st.passages + ' | line "' + C2.pre.pre + '"');
// ---- control 2: a pole on the path but lower than the drone's height there (the drone flies over it)
const C3 = await poleCase('low pole', async (b, geo) => { const q = pickOnPath(geo, ARC); const px = await movePole(b, 1, q.x, q.y); await lowerPole(b, 1, px, 29); }, {});
const P2low = C3.saved.obstacles[1].pos, zAt = Math.min(...TDN.routeGeometry(C3.L, 32).samples.filter(p => Math.hypot(p.x - P2low[0], p.y - P2low[1]) < 0.09).map(p => p.z));
check('control 2: a pole on the path but lower (top ' + P2low[2] + ' m) than the drone there (z at least ' + zAt.toFixed(2) + ' m): no hit line, no stop, 5 of 5 passages', P2low[2] + DRONE_BELOW < zAt && !/hits pole/.test(C3.pre.pre) && /^Run finished in/.test(C3.st.status) && C3.st.passages === 5, 'status ' + C3.st.status + ' | passages ' + C3.st.passages + ' | pole ' + JSON.stringify(P2low) + ' | line "' + C3.pre.pre + '"');
// ---- control 3: a pole 0.15 m beside the path: no stop, the near-pass line BEFORE Play
const C4 = await poleCase('beside 0.15', async (b, geo) => { const q = pickOnPath(geo, ARC); await movePole(b, 1, q.x - q.dy * 0.15, q.y + q.dx * 0.15); }, {});
const dist4 = Math.min(...(() => { const S = TDN.routeGeometry(C4.L, 32).samples, o = C4.saved.obstacles[1].pos; return S.map(p => Math.hypot(p.x - o[0], p.y - o[1])); })());
check('control 3: a pole ' + dist4.toFixed(2) + ' m beside the path: no hit line, the near-pass line shows BEFORE Play, no stop, 5 of 5 passages', dist4 > HITR + 0.02 && dist4 < 0.25 && !/hits pole/.test(C4.pre.pre) && /comes within [0-9.]+ m of pole P2/.test(C4.pre.pre) && /^Run finished in/.test(C4.st.status) && C4.st.passages === 5, 'line "' + C4.pre.pre + '" | status ' + C4.st.status);
check('hit tests: no console error, no exception, no failed request in any of the four sessions', [C1, C2, C3, C4].every(c => c.errs.length === 0), [C1, C2, C3, C4].flatMap(c => c.errs).slice(0, 3).join(' | '));

const fails = results.filter(r => !r.ok);
console.log(`\nRESULT: ${results.length - fails.length} of ${results.length} checks passed`);
fs.writeFileSync(OUT + 'test_result.txt', results.map(r => (r.ok ? 'PASS ' : 'FAIL ') + r.name + (r.detail ? '  [' + r.detail + ']' : '')).join('\n') + `\n\nRESULT: ${results.length - fails.length} of ${results.length} checks passed\n`);
process.exit(fails.length ? 1 : 0);
