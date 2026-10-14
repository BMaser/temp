// Tests the four built pages (simulator_look, drone_gt, drone_gt_180, frame_options) in headless Edge, trusted mouse input.
//  - built pages: opened from a plain file:/// address, NO file switch, network off (dead proxy + CDP offline mode, proven by a request that must fail).
//    The 3D pages use software drawing (--use-angle=swiftshader --enable-unsafe-swiftshader --ignore-gpu-blocklist); that is allowed.
//  - ORIGINAL export pages: opened with --allow-file-access-from-files and the internet on, only as the reference for the visible-text comparison.
// Usage: node pages_test_trusted.mjs        Exit code 0 only if every check passes.   Writes pages_*.png and pages_test_result.txt.
import os from 'node:os';
os.setPriority(0, os.constants.priority.PRIORITY_BELOW_NORMAL);
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { pathToFileURL } from 'node:url';
import { launch, sleep, OUT, EXPORT, BROWSER_PATH } from './pages_lib.mjs';

const FU = p => pathToFileURL(p).href;
const BUILT = n => FU(OUT + n);
const ORIG = n => FU(EXPORT + n);
const GL = ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'];
const OFF = ['--proxy-server=127.0.0.1:9', '--proxy-bypass-list=<-loopback>'];
const PAL = JSON.parse(fs.readFileSync(path.join(EXPORT, 'returned', 'palettes.json'), 'utf8'));
const LOOKS = [['magenta', 'Magenta'], ['cyan', 'Cyan'], ['red', 'Red'], ['violet', 'Violet']];

const ONLY = process.env.ONLY || '';
const run = n => !ONLY || ONLY.includes(String(n));
const lines = [];
const results = [];
const say = s => { console.log(s); lines.push(s); };
const check = (name, ok, detail = '') => { results.push({ name, ok: !!ok }); say((ok ? 'PASS ' : 'FAIL ') + name + (detail ? '  [' + detail + ']' : '')); };
const sha = buf => crypto.createHash('sha256').update(buf).digest('hex');

async function offline(extra = []) {
  const b = await launch([...OFF, ...extra], 1440, 900);
  await b.send('Network.emulateNetworkConditions', { offline: true, latency: 0, downloadThroughput: 0, uploadThroughput: 0 });
  return b;
}
async function netProof(b, tag) {
  const r = await b.ev(`fetch('https://example.com/', {mode:'no-cors', cache:'no-store'}).then(()=> 'REACHED', e => 'blocked: ' + e.message)`);
  check(tag + ': network is off (a request to https://example.com must fail)', String(r).startsWith('blocked'), r);
}
// console errors (not warnings), exceptions, and any request that is not file:/data:/blob:/about:/devtools
function netReport(b, tag, page) {
  b.resolveUrls();
  const errs = b.log.console.filter(s => /^(error|log-error)/.test(s) && !s.includes('example.com')).concat(b.log.exceptions.map(e => 'exception: ' + e));
  check(tag + ': no console error, no exception', errs.length === 0, errs.slice(0, 3).join(' | '));
  const bad = b.log.http.filter(h => !/^(file|data|blob|about|chrome|devtools):/i.test(h.url) && h.url !== 'https://example.com/');
  const files = b.log.http.filter(h => /^file:/i.test(h.url)).map(h => decodeURIComponent(h.url));
  check(tag + ': no request to any server (all requests are file:, data: or blob:)', bad.length === 0, bad.slice(0, 3).map(h => h.url.slice(0, 80)).join(' | ') || (b.log.http.length + ' requests'));
  check(tag + ': only the page file itself was requested from disk', files.every(f => f.endsWith(page)), files.length + ' file request(s)');
  const warns = b.log.console.filter(s => !/^(error|log-error)/.test(s));
  if (warns.length) say('   note ' + tag + ' warnings (not errors): ' + [...new Set(warns.map(w => w.slice(0, 110)))].slice(0, 3).join(' | '));
}
async function dl(b, action, nFiles = 1) {
  const dir = fs.mkdtempSync(path.join(OUT, '_downloads_'));   // a folder inside the test folder (removed after each download)
  await b.send('Browser.setDownloadBehavior', { behavior: 'allow', downloadPath: dir });
  await action();
  let f = [];
  for (let i = 0; i < 60; i++) { f = fs.readdirSync(dir).filter(x => !x.endsWith('.crdownload')); if (f.length >= nFiles) { await sleep(500); f = fs.readdirSync(dir).filter(x => !x.endsWith('.crdownload')); break; } await sleep(250); }
  const out = {}; for (const n of f) out[n] = fs.readFileSync(path.join(dir, n));
  try { fs.rmSync(dir, { recursive: true, force: true, maxRetries: 5, retryDelay: 300 }); } catch (e) { }
  return out;
}
const pngDims = buf => [buf.readUInt32BE(16), buf.readUInt32BE(20)];
const meanDiff = (a, b) => Math.abs(a.mean[0] - b.mean[0]) + Math.abs(a.mean[1] - b.mean[1]) + Math.abs(a.mean[2] - b.mean[2]);
const mad = (a, b) => a.reduce((s, v, i) => s + Math.abs(v - b[i]), 0) / a.length;
const cellsDiff = (a, b, t = 20) => a.reduce((s, v, i) => s + (Math.abs(v - b[i]) > t ? 1 : 0), 0);

// text a person can see (the shared navigation bar is left out), for the comparison with the original
const VISIBLE = `(()=>{const nav=document.getElementById('pages-nav');const inNav=e=>nav&&nav.contains(e);const a=[document.title,document.body.innerText];
document.querySelectorAll('svg text').forEach(t=>{if(!inNav(t))a.push(t.textContent)});
document.querySelectorAll('[title],[aria-label],[placeholder],img[alt]').forEach(e=>{if(inNav(e))return;['title','aria-label','placeholder','alt'].forEach(k=>{const v=e.getAttribute(k);if(v)a.push(k+':'+v)})});
const s=document.querySelector('three-d-stage');if(s&&s.shadowRoot)s.shadowRoot.querySelectorAll('.note,.toolbar button,.err').forEach(e=>a.push(e.textContent));
let r=a.join('\\n');if(nav&&document.body.innerText.includes(nav.innerText.trim())){r=r.replace(nav.innerText.trim(),'')}return r.replace(/\\s+/g,' ').trim()})()`;
function diffText(t1, t2) {
  if (t1 === t2) return [];
  const a = t1.split(' '), b = t2.split(' '); const d = [];
  let i = 0, j = 0;
  while ((i < a.length || j < b.length) && d.length < 6) {
    if (a[i] === b[j]) { i++; j++; continue; }
    d.push('built "' + (a[i] ?? '') + '" vs original "' + (b[j] ?? '') + '" near: ' + a.slice(Math.max(0, i - 3), i + 2).join(' '));
    i++; j++;
  }
  return d;
}

// ===================== 1. Simulator look =====================
if (run(1)) {
  const P = 'simulator_look.html';
  const b = await offline();
  await b.nav(BUILT(P));
  const ok = await b.waitFor("document.querySelectorAll('svg[data-td=vTop]').length>0 && document.querySelector('svg[data-td=vTop]').children.length>5", 40000);
  check('look: page draws from file:/// with no switch, offline', ok >= 0, ok + ' ms');
  await sleep(1500);
  await netProof(b, 'look');
  const expectedPics = {};
  const hashes = {}, frames = {}, drones = {};
  for (const [k, label] of LOOKS) {
    await b.clickText('button', label); await sleep(900);
    const st = await b.ev(`({files:(document.body.innerText.match(/returned\\/\\w+\\//)||[])[0], frame:(()=>{const e=[...document.querySelectorAll('#dc-root div')].find(d=>getComputedStyle(d).borderTopWidth==='5px'&&getComputedStyle(d).borderTopStyle==='solid');return e?getComputedStyle(e).borderTopColor:null})(), drone:document.querySelector('svg[data-td=droneScene]').innerHTML.length, droneHtml:document.querySelector('svg[data-td=droneScene]').innerHTML, rows:[...document.querySelectorAll('table tr')].map(r=>r.innerText.replace(/\\s+/g,' ').trim())})`);
    const P_ = PAL[k], f = P_.frame;
    check(`look ${k}: chosen look is shown (files path)`, st.files === `returned/${k}/`, st.files);
    check(`look ${k}: gate frame color in the scene = palettes.json frame`, st.frame === `rgb(${f.join(', ')})`, st.frame + ' vs ' + f.join(','));
    check(`look ${k}: color table shows the look's gate band`, st.rows.some(r => r.startsWith('Gate band') && r.includes(P_.gate.join(' '))), (st.rows[0] || ''));
    frames[k] = st.frame; drones[k] = st.droneHtml;
    // pictures
    const urls = await b.ev(`[...new Set([...document.querySelectorAll('#dc-root *')].map(e=>getComputedStyle(e).backgroundImage).filter(v=>v&&v.includes('blob:')).map(v=>(v.match(/url\\("?(blob:[^")]+)"?\\)/)||[])[1]).filter(Boolean))]`);
    const bytes = await b.ev(`Promise.all(${JSON.stringify(urls)}.map(u=>fetch(u).then(r=>r.blob()).then(bl=>new Promise(r=>{const f=new FileReader();f.onload=()=>r(f.result.split(',')[1]);f.readAsDataURL(bl)}))))`);
    const shown = bytes.map(x => sha(Buffer.from(x, 'base64')));
    const want = ['gate_top', 'gate_bottom', 'gate_left', 'gate_right', 'gate_post_bark', 'pad_start', 'pad_end', 'gate_triangle', 'gate_circle'].map(n => fs.readFileSync(path.join(EXPORT, 'returned', k, n + '.png')));
    const wantSha = want.map(sha);
    hashes[k] = shown;
    check(`look ${k}: the 9 shown pictures are byte-identical to the export's files`, shown.length === 9 && urls.every(u => u.startsWith('blob:')) && wantSha.every(h => shown.includes(h)), shown.length + ' distinct pictures');
    const widths = await b.ev(`Promise.all(${JSON.stringify(urls)}.map(u=>new Promise(r=>{const i=new Image();i.onload=()=>r([i.naturalWidth,i.naturalHeight]);i.onerror=()=>r([0,0]);i.src=u})))`);
    const wantDims = want.map(pngDims).map(String).sort().join(';'), gotDims = widths.map(String).sort().join(';');
    check(`look ${k}: every picture loads (width above 0) and has the export's size`, widths.every(w => w[0] > 0) && wantDims === gotDims, widths.length + ' images, min width ' + Math.min(...widths.map(w => w[0])));
    await b.shot(`pages_look_${k}.png`);
  }
  check('look: the four looks differ (picture sets all 4; drone drawing 3, because cyan and violet have the same drone colors in palettes.json; frame colors 3)', new Set(Object.values(frames)).size >= 3 && new Set(Object.values(drones)).size === 3 && new Set(Object.values(hashes).map(h => h.slice().sort().join())).size === 4,
    Object.values(frames).join(' / '));
  // colors.txt of each look: numbers equal the export's colors.txt
  const expectTriples = k => {
    const t = fs.readFileSync(path.join(EXPORT, 'returned', k, 'colors.txt'), 'utf8').split(/\r?\n/);
    const tri = (prefix, n = 0) => { const l = t.find(x => x.startsWith(prefix)); const m = [...l.replace(/\(0\.5 m squares\)/, '').matchAll(/(\d+) (\d+) (\d+)/g)][n]; return [+m[1], +m[2], +m[3]].join(' '); };
    return [tri('Gate band'), tri('Gate frame'), tri('Gate springs'), tri('Corner marks'), tri('Floor', 0), tri('Floor', 1), tri('Sky horizon'), tri('Sky top'), tri('Drone body'), tri('Drone nose'), tri('Drone rear wing')];
  };
  const parseDl = txt => txt.split(/\r?\n/).filter(l => l && !l.startsWith('#')).map(l => l.match(/(\d+) (\d+) (\d+)\s*$/).slice(1, 4).join(' '));
  for (const [k, label] of LOOKS) {
    await b.clickText('button', label); await sleep(700);
    const files = await dl(b, () => b.clickText('button', 'Download colors.txt'));
    const txt = (files['colors.txt'] || Buffer.alloc(0)).toString('utf8');
    const got = txt ? parseDl(txt) : [], want = expectTriples(k);
    check(`look ${k}: "Download colors.txt" numbers equal the export's ${k}\\colors.txt (11 colors)`, got.length === 11 && got.join('|') === want.join('|'), got.length + ' colors; header: ' + txt.split('\n')[0]);
    if (k === 'magenta') {
      const other = expectTriples('cyan');
      check('look (negative control): the magenta download does NOT equal the cyan colors.txt', got.join('|') !== other.join('|'));
    }
  }
  // color picker: change "Gate frame" on the Magenta look; the scene follows, the download carries the new value, Reset undoes it
  await b.clickText('button', 'Magenta'); await sleep(700);
  const setPicker = (idx, hex) => b.ev(`(()=>{const e=document.querySelectorAll('table input[type=color]')[${idx}];const set=Object.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').set;set.call(e,'${hex}');e.dispatchEvent(new Event('input',{bubbles:true}));return e.value})()`);
  const frameNow = () => b.ev(`(()=>{const e=[...document.querySelectorAll('#dc-root div')].find(d=>getComputedStyle(d).borderTopWidth==='5px'&&getComputedStyle(d).borderTopStyle==='solid');return getComputedStyle(e).borderTopColor})()`);
  const before = await frameNow();
  await setPicker(1, '#00ff40'); await sleep(600);
  const after = await frameNow();
  check('look: a color picker (Gate frame) changes the shown color', before === 'rgb(20, 20, 24)' && after === 'rgb(0, 255, 64)', before + ' -> ' + after);
  const rowTxt = await b.ev(`[...document.querySelectorAll('table tr')][1].innerText.replace(/\\s+/g,' ').trim()`);
  check('look: the table row shows the new numbers', rowTxt.includes('0 255 64'), rowTxt);
  const f2 = await dl(b, () => b.clickText('button', 'Download colors.txt'));
  const g2 = parseDl(f2['colors.txt'].toString('utf8')), w2 = expectTriples('magenta'); w2[1] = '0 255 64';
  check('look: the download after the change carries the new frame color and the other 10 unchanged', g2.join('|') === w2.join('|'), g2[1]);
  await b.clickText('button', 'Reset'); await sleep(600);
  check('look: Reset returns the frame color', (await frameNow()) === before);
  netReport(b, 'look', P);
  await b.close();
}

// ===================== 2. Drone GT and GT 180 =====================
const SMALL = `(()=>{const s=document.querySelector('three-d-stage');const c=s.shadowRoot.querySelector('canvas');const N=64;const t=document.createElement('canvas');t.width=c.width;t.height=c.height;const x=t.getContext('2d');x.fillStyle='rgb(243,242,242)';x.fillRect(0,0,t.width,t.height);x.drawImage(c,0,0);
const d=x.getImageData(0,0,t.width,t.height).data;let n=0,sr=0,sg=0,sb=0;for(let i=0;i<d.length;i+=4){if(Math.max(Math.abs(d[i]-243),Math.abs(d[i+1]-242),Math.abs(d[i+2]-242))>12){n++;sr+=d[i];sg+=d[i+1];sb+=d[i+2]}}
const sm=document.createElement('canvas');sm.width=N;sm.height=N;const y=sm.getContext('2d');y.drawImage(t,0,0,N,N);const e=y.getImageData(0,0,N,N).data;const g=[];for(let i=0;i<e.length;i+=4)g.push(Math.round(0.299*e[i]+0.587*e[i+1]+0.114*e[i+2]));
return {share:n/(t.width*t.height),w:c.width,h:c.height,grid:g,mean:[sr/Math.max(n,1),sg/Math.max(n,1),sb/Math.max(n,1)]}})()`;
const SHARE_LIMIT = 0.02;
const snap = async b => { await sleep(900); return b.ev(SMALL); };
const stageBtn = (b, text) => b.ev(`(()=>{const e=[...document.querySelector('three-d-stage').shadowRoot.querySelectorAll('.toolbar button')].find(x=>x.textContent.trim()==='${text}');e.scrollIntoView();const r=e.getBoundingClientRect();return [r.left+r.width/2,r.top+r.height/2,e.disabled]})()`);

if (run(2)) for (const [P, ORIGNAME, tag, base] of [['drone_gt.html', 'Drone GT.html', 'gt', 'drone_gt'], ['drone_gt_180.html', 'Drone GT 180.html', 'gt180', 'drone_gt_180']]) {
  const b = await offline(GL);
  await b.nav(BUILT(P));
  // control: before the model has loaded the buttons are disabled and the canvas is empty (read as early as possible)
  const ok = await b.waitFor("(()=>{const s=document.querySelector('three-d-stage');return !!(s&&s._object)})()", 90000);
  check(`${tag}: page draws from file:/// with no switch, offline (the 3D model exists)`, ok >= 0, ok + ' ms');
  await sleep(2000);
  await netProof(b, tag);
  const first = await b.ev(SMALL);
  check(`${tag}: the 3D view is really drawn (share of points differing from the background > ${SHARE_LIMIT})`, first.share > SHARE_LIMIT, (first.share * 100).toFixed(1) + '% of ' + first.w + 'x' + first.h);
  // control that can fail: hide the model, the same measurement must fall below the limit
  await b.ev(`(()=>{const s=document.querySelector('three-d-stage');s._object.visible=false})()`); await sleep(700);
  const empty = await b.ev(SMALL);
  check(`${tag}: control - with the model hidden the same measure FAILS the limit`, empty.share < SHARE_LIMIT, (empty.share * 100).toFixed(2) + '%');
  await b.ev(`(()=>{const s=document.querySelector('three-d-stage');s._object.visible=true})()`); await sleep(500);
  // six views
  const views = {};
  for (const [v, label] of [['iso', 'Iso'], ['front', 'Front'], ['back', 'Back'], ['top', 'Top'], ['side', 'Side'], ['fpv', 'Camera cone']]) {
    await b.clickText('#views button', label); views[v] = await snap(b);
  }
  const names = Object.keys(views); let minD = 1e9, pair = '';
  for (let i = 0; i < names.length; i++) for (let j = i + 1; j < names.length; j++) { const c = cellsDiff(views[names[i]].grid, views[names[j]].grid); if (c < minD) { minD = c; pair = names[i] + '/' + names[j]; } }
  check(`${tag}: all six views are drawn (each share > limit)`, names.every(n => views[n].share > SHARE_LIMIT), names.map(n => n + ' ' + (views[n].share * 100).toFixed(1) + '%').join(', '));
  check(`${tag}: the six views give pairwise different pictures (>= 8 of 4096 cells differ by > 20)`, minD >= 8, 'least different pair ' + pair + ': ' + minD + ' cells');
  await b.clickText('#views button', 'Iso'); const iso2 = await snap(b);
  check(`${tag}: control - the same view twice gives (almost) the same picture`, cellsDiff(views.iso.grid, iso2.grid) < 8, cellsDiff(views.iso.grid, iso2.grid) + ' cells');
  await b.shot(`pages_${tag === 'gt' ? 'drone_gt' : 'drone_gt_180'}_iso.png`, false);
  // palettes
  const bodyCol = () => b.ev(`(()=>{const s=document.querySelector('three-d-stage');let c=null;s._object.traverse(o=>{if(o.isMesh&&o.name==='body'&&!c)c=[o.material.color.r,o.material.color.g,o.material.color.b]});return c})()`);
  const pics = {};
  for (const [k] of LOOKS) {
    await b.clickText('#pal button', k[0].toUpperCase() + k.slice(1) + ' look'); pics[k] = await snap(b);
    const bc = await bodyCol(), d = PAL[k].drone;
    check(`${tag}: palette button "${k}" sets the body color to palettes.json drone color`, bc && bc.every((v, i) => Math.abs(v - d[i] / 255) < 1e-6), bc && bc.map(v => v.toFixed(3)).join(','));
  }
  const pk = Object.keys(pics); const badPal = []; const same = [];
  for (let i = 0; i < pk.length; i++) for (let j = i + 1; j < pk.length; j++) {
    const c = cellsDiff(pics[pk[i]].grid, pics[pk[j]].grid) + (meanDiff(pics[pk[i]], pics[pk[j]]) > 3 ? 8 : 0), A = PAL[pk[i]], B = PAL[pk[j]];
    const colorsDiffer = A.drone.join() !== B.drone.join() || A.nose.join() !== B.nose.join();
    if (colorsDiffer !== (c >= 8)) badPal.push(pk[i] + '/' + pk[j] + ': ' + c + ' (cells + mean-color flag), colors differ=' + colorsDiffer);
    if (!colorsDiffer) same.push(pk[i] + '/' + pk[j]);
  }
  check(`${tag}: two palette buttons give different pictures exactly when their drone/nose colors differ in palettes.json`, badPal.length === 0, badPal.join('; ') || ('identical drone+nose colors in the data, so identical pictures: ' + (same.join(', ') || 'none')));
  // color picker and the colors.txt loader
  const hex0 = await b.ev(`document.getElementById('cBody').value`);
  await b.ev(`(()=>{const e=document.getElementById('cBody');e.value='#0040ff';e.dispatchEvent(new Event('input',{bubbles:true}))})()`);
  const afterPick = await snap(b);
  check(`${tag}: the Body color picker changes the picture`, cellsDiff(pics.violet.grid, afterPick.grid) >= 8, cellsDiff(pics.violet.grid, afterPick.grid) + ' cells (was ' + hex0 + ')');
  const tmp = path.join(OUT, '_pages_colors_test.txt'); fs.writeFileSync(tmp, 'Drone GT part colors (RGB 0-255)\nbody 10 200 30\nbogus 1 2 3\n');
  const doc = await b.send('DOM.getDocument'); const q = await b.send('DOM.querySelector', { nodeId: doc.root.nodeId, selector: '#colfile' });
  await b.send('DOM.setFileInputFiles', { files: [tmp], nodeId: q.nodeId }); await sleep(700);
  const msg = await b.ev(`document.getElementById('colmsg').textContent`), bc2 = await bodyCol();
  check(`${tag}: the colors.txt loader applies "body" and names the unknown part`, /Applied body/.test(msg) && /unknown: bogus/.test(msg) && Math.abs(bc2[1] - 200 / 255) < 0.01, msg + ' ; body now ' + bc2.map(v => v.toFixed(2)).join(','));
  fs.unlinkSync(tmp);
  await b.clickText('#pal button', 'Magenta look'); await snap(b);
  // X-ray
  const before = await snap(b);
  const cbp = await b.ev(`(()=>{const e=document.getElementById('xr').getBoundingClientRect();return [e.left+e.width/2,e.top+e.height/2]})()`);
  await b.click(cbp[0], cbp[1]); const xr = await snap(b);
  const op = await b.ev(`(()=>{const s=document.querySelector('three-d-stage');let o=null;s._object.traverse(m=>{if(m.isMesh&&m.name==='body'&&o===null)o=[m.material.transparent,m.material.opacity]});return o})()`);
  check(`${tag}: X-ray changes the picture and makes the body transparent`, cellsDiff(before.grid, xr.grid) >= 8 && op[0] === true && Math.abs(op[1] - 0.22) < 1e-6, cellsDiff(before.grid, xr.grid) + ' cells, opacity ' + op[1]);
  await b.click(cbp[0], cbp[1]); const xr2 = await snap(b);
  check(`${tag}: X-ray off brings the first picture back`, cellsDiff(before.grid, xr2.grid) < 8, cellsDiff(before.grid, xr2.grid) + ' cells');
  // downloads
  let p = await stageBtn(b, 'Download OBJ + MTL');
  check(`${tag}: the "Download OBJ + MTL" button is enabled once the model exists`, p[2] === false);
  const fo = await dl(b, () => b.click(p[0], p[1]), 2);
  const obj = (fo[base + '.obj'] || Buffer.alloc(0)).toString('utf8'), mtl = (fo[base + '.mtl'] || Buffer.alloc(0)).toString('utf8');
  const nv = obj.split('\n').filter(l => l.startsWith('v ')).length, nmtl = (mtl.match(/^newmtl /gm) || []).length;
  check(`${tag}: "Download OBJ + MTL" gives ${base}.obj with more than 1000 vertices and a .mtl`, nv > 1000 && nmtl >= 5 && /^mtllib /.test(obj), nv + ' vertices, ' + nmtl + ' materials, ' + Object.keys(fo).join(','));
  p = await stageBtn(b, 'Download GLB');
  const fg = await dl(b, () => b.click(p[0], p[1]), 1);
  const glb = fg[base + '.glb'] || Buffer.alloc(0);
  let gjson = null; try { const len = glb.readUInt32LE(12); gjson = JSON.parse(glb.slice(20, 20 + len).toString('utf8')); } catch (e) { }
  check(`${tag}: "Download GLB" gives a non-empty glTF binary`, glb.length > 0 && glb.slice(0, 4).toString() === 'glTF' && gjson && gjson.nodes.length > 10, glb.length + ' bytes, ' + (gjson ? gjson.nodes.length + ' nodes, ' + gjson.meshes.length + ' meshes' : 'no json'));
  const fs2 = await dl(b, () => b.clickText('button', 'OBJ for simulator (Z-up)'), 3);
  const names2 = Object.keys(fs2), objn = names2.find(n => n.endsWith('.obj'));
  const sobj = objn ? fs2[objn].toString('utf8') : '';
  const vs = sobj.split('\n').filter(l => l.startsWith('v ')).map(l => l.split(' ').slice(1, 4).map(Number));
  const ext = [0, 1, 2].map(i => { const a = vs.map(v => v[i]); return [Math.min(...a), Math.max(...a)]; });
  const mm = ext.map(([lo, hi]) => ((hi - lo) * 1000).toFixed(1));
  check(`${tag}: "OBJ for simulator (Z-up)" gives an .obj, a .mtl and drone_colors.txt`, !!objn && vs.length > 1000 && names2.some(n => n.endsWith('.mtl')) && names2.includes('drone_colors.txt'), names2.join(',') + '; ' + vs.length + ' vertices');
  say(`   ${tag} Z-up OBJ extent (mm): x ${mm[0]} (${(ext[0][0] * 1000).toFixed(0)}..${(ext[0][1] * 1000).toFixed(0)}), y ${mm[1]}, z ${mm[2]} (${(ext[2][0] * 1000).toFixed(0)}..${(ext[2][1] * 1000).toFixed(0)}); header: ${sobj.split('\n')[0]}`);
  netReport(b, tag, P);
  await b.close();
}

// ===================== 3. Frame options =====================
const FRAME_COUNT = `({svgs:document.querySelectorAll('svg').length, svgEls:document.querySelectorAll('svg *').length, text:document.querySelectorAll('svg text').length, cards:document.querySelectorAll('[id="2a"],[id="2b"],[id="1a"],[id="1b"]').length})`;
let frameBuilt;
if (run(3)) {
  const P = 'frame_options.html';
  const b = await offline();
  await b.nav(BUILT(P));
  const ok = await b.waitFor("document.querySelectorAll('svg').length>=4", 40000);
  check('frame: page draws from file:/// with no switch, offline', ok >= 0, ok + ' ms');
  await sleep(1500);
  await netProof(b, 'frame');
  frameBuilt = await b.ev(FRAME_COUNT);
  check('frame: the drawings are there (4 cards, svg count > 0)', frameBuilt.svgs >= 4 && frameBuilt.cards === 4, JSON.stringify(frameBuilt));
  await b.shot('pages_frame_options.png');
  netReport(b, 'frame', P);
  await b.close();
}

// ===================== 4. Navigation bar =====================
if (run(4)) {
  const pages = [['simulator_look.html', 'Simulator look'], ['drone_gt.html', 'Drone GT'], ['drone_gt_180.html', 'Drone GT 180'], ['frame_options.html', 'Frame options']];
  const links = [['Track designer', 'track_designer_gui.html'], ['Simulator look', 'simulator_look.html'], ['Drone GT', 'drone_gt.html'], ['Drone GT 180', 'drone_gt_180.html'], ['Frame options', 'frame_options.html']];
  check('nav: the file the first link points to exists next to the pages', fs.existsSync(OUT + 'track_designer_gui.html'));
  const b = await offline();
  let n = 0, good = 0; const bad = [];
  for (const [P, here] of pages) {
    await b.nav(BUILT(P)); await b.waitFor("!!document.getElementById('pages-nav')", 30000); await sleep(1200);
    const cur = await b.ev(`[...document.querySelectorAll('#pages-nav a[aria-current=page]')].map(a=>a.textContent)`);
    check(`nav ${P}: exactly one link is marked, and it is "${here}"`, cur.length === 1 && cur[0] === here, cur.join());
    const texts = await b.ev(`[...document.querySelectorAll('#pages-nav a')].map(a=>a.textContent+'>'+a.getAttribute('href'))`);
    check(`nav ${P}: five links, in the agreed order, with relative file names`, texts.join('|') === links.map(l => l[0] + '>' + l[1]).join('|'), texts.join(' | '));
    for (const [label, file] of links) {
      await b.nav(BUILT(P)); await b.waitFor("!!document.getElementById('pages-nav')", 30000); await sleep(600);
      n++;
      const top = await b.ev(`(()=>{const a=[...document.querySelectorAll('#pages-nav a')].find(x=>x.textContent==='${label}');const r=a.getBoundingClientRect();return document.elementFromPoint(r.left+r.width/2,r.top+r.height/2)===a})()`);
      await b.clickText('#pages-nav a', label);
      await b.waitFor(`decodeURIComponent(location.href).endsWith('/${file}')`, 20000);
      const href = decodeURIComponent(await b.ev('location.href'));
      if (top && href.endsWith('/' + file) && href.startsWith('file:///')) good++; else bad.push(P + ' -> ' + label + ' went to ' + href.split('/').pop() + (top ? '' : ' (link covered)'));
    }
  }
  check(`nav: from each of the 4 pages each of the 5 links is on top, clickable and leads to the right file (${good} of ${n})`, good === n, bad.join('; '));
  await b.close();
}

// ===================== 5. Visible text against the originals =====================
if (run(5)) {
  // one browser at a time: first the built pages (offline), then the originals (internet on, special switch on)
  const textOf = async (br, url, readyExpr) => { await br.nav(url); await br.waitFor(readyExpr, 90000); await sleep(2500); return br.ev(VISIBLE); };
  const lookReady = "document.querySelectorAll('svg[data-td=vTop]').length>0 && document.querySelector('svg[data-td=vTop]').children.length>5";
  const gtReady = "(()=>{const s=document.querySelector('three-d-stage');return !!(s&&s._object)})()";
  const fr = "document.querySelectorAll('svg').length>=4";
  const T = { built: {}, orig: {} };
  const collect = async (br, which, files) => {
    await br.nav(files.look); await br.waitFor(lookReady, 60000); await sleep(1500);
    for (const [k, label] of LOOKS) { await br.clickText('button', label); await sleep(900); T[which]['look ' + k] = await br.ev(VISIBLE); }
    if (which === 'orig') T.oUrls = await br.ev(`[...new Set([...document.querySelectorAll('#dc-root *')].map(e=>getComputedStyle(e).backgroundImage).filter(v=>v&&v.includes('url(')&&!v.includes('gradient')).map(v=>(v.match(/url\\("?([^")]+)"?\\)/)||[])[1]).filter(Boolean))]`);
    T[which]['Frame options'] = await textOf(br, files.frame, fr); T[which].frameCount = await br.ev(FRAME_COUNT);
  };
  const collectGt = async (br, which, files) => {
    T[which]['Drone GT'] = await textOf(br, files.gt, gtReady);
    T[which]['Drone GT 180'] = await textOf(br, files.gt180, gtReady);
  };
  let b = await offline();
  await collect(b, 'built', { look: BUILT('simulator_look.html'), frame: BUILT('frame_options.html') }); await b.close();
  b = await offline(GL);
  await collectGt(b, 'built', { gt: BUILT('drone_gt.html'), gt180: BUILT('drone_gt_180.html') }); await b.close();
  let o = await launch(['--allow-file-access-from-files'], 1440, 900);
  await collect(o, 'orig', { look: ORIG('Simulator Look.dc.html'), frame: ORIG('Frame Options.dc.html') }); await o.close();
  o = await launch(['--allow-file-access-from-files', ...GL], 1440, 900);
  await collectGt(o, 'orig', { gt: ORIG('Drone GT.html'), gt180: ORIG('Drone GT 180.html') }); await o.close();
  for (const name of Object.keys(T.built)) {
    if (name === 'frameCount') continue;
    const d = diffText(T.built[name], T.orig[name]);
    check(`text: ${name}: visible text of the built page equals the original`, d.length === 0, d.length ? d.join(' || ') : T.built[name].length + ' characters');
  }
  check('text (control): the original look page shows 9 pictures from files (so the comparison page really had its pictures)', T.oUrls.length === 9 && T.oUrls.every(u => u.startsWith('file:///')), T.oUrls.length + ' urls');
  check('frame: the drawing counts equal the original (svg, svg elements, svg texts, cards)', JSON.stringify(T.orig.frameCount) === JSON.stringify(T.built.frameCount), 'original ' + JSON.stringify(T.orig.frameCount) + ' built ' + JSON.stringify(T.built.frameCount));
  check('text (control): the comparison does flag a difference when the texts differ', diffText('a b c', 'a x c').length === 1);
}

const failed = results.filter(r => !r.ok);
say(`\n${results.length - failed.length} of ${results.length} checks passed` + (failed.length ? '; FAILED: ' + failed.map(f => f.name).join(' | ') : ''));
fs.writeFileSync(OUT + 'pages_test_result.txt', lines.join('\n') + '\n');
process.exit(failed.length ? 1 : 0);
