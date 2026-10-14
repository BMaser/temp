// Browser test for feature_filter.html in headless Chrome (DevTools protocol, trusted mouse input).
// The built page is opened from a plain file:/// address with the network off (dead proxy + CDP offline mode).
// Expected numbers are computed here straight from features_data.json, not by the page.
// Usage: BROWSER=/path/to/chrome RESULT_DIR=./result node test_filter_browser.mjs      Exit code 0 only if every check passes.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { launch, sleep, OUT } from './lib.mjs';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const PAGE = pathToFileURL(path.join(HERE, 'feature_filter.html')).href;
const data = JSON.parse(fs.readFileSync(path.join(HERE, 'features_data.json'), 'utf8'));
const F92 = data.features.map(f => f.id);
const simIds = data.sims.map(s => s.id);
const ans = (s, f) => data.answers[s][f];

const results = [];
const lines = [];
const say = s => { lines.push(s); console.log(s); };
const check = (name, ok, detail = '') => { results.push({ name, ok: !!ok }); say((ok ? 'PASS ' : 'FAIL ') + name + (detail ? '  [' + detail + ']' : '')); };

const b = await launch(['--proxy-server=127.0.0.1:9', '--proxy-bypass-list=<-loopback>'], 1366, 768);
await b.send('Network.emulateNetworkConditions', { offline: true, latency: 0, downloadThroughput: 0, uploadThroughput: 0 });

// Collect console problems and requests across navigations (b.nav clears the log).
const acc = { console: [], exceptions: [], failed: [], urls: [] };
const harvest = () => { b.resolveUrls(); acc.console.push(...b.log.console); acc.exceptions.push(...b.log.exceptions); acc.failed.push(...b.log.failed); acc.urls.push(...b.log.http.map(h => h.url)); };
const text = sel => b.ev(`(document.querySelector(${JSON.stringify(sel)})||{}).innerText||''`);
const center = sel => b.ev(`(()=>{const e=document.querySelector(${JSON.stringify(sel)});if(!e)return null;e.scrollIntoView({block:'center'});const r=e.getBoundingClientRect();return [r.left+r.width/2,r.top+r.height/2]})()`);
const clickSel = async sel => { const c = await center(sel); if (!c) throw new Error('no element ' + sel); await b.click(c[0], c[1]); return c; };
const tab = async name => { await b.clickText('[role=tab]', name); await sleep(400); };

try {
  // ---- 1. the page opens and the first tab draws
  await b.nav(PAGE);
  const ms = await b.waitFor("document.querySelectorAll('#allTable tbody tr').length>0", 40000);
  check('page opens from file:/// and the first tab draws rows', ms >= 0, ms + ' ms');
  const rows0 = await b.ev(`document.querySelectorAll('#allTable tbody tr').length`);
  check('All features: 92 asked + 88 extras = 180 rows', rows0 === 180, String(rows0));
  const extraText = await b.ev(`document.querySelector('#allTable tbody tr.extra').innerText`);
  check('an extra row says "Asked of one simulator only"', /Asked of one simulator only: /.test(extraText), extraText.replace(/\s+/g, ' ').slice(0, 90));
  const how = await text('#how');
  check('the "How to read this page" box is at the top and holds the four notes', /Has it fully/.test(how) && /not the same as no/i.test(how) && /asked of one simulator only/i.test(how) && /2026-10-14/.test(how));
  const ovf = await b.ev(`({sw:document.documentElement.scrollWidth,iw:innerWidth,headPos:getComputedStyle(document.querySelector('header.top')).position})`);
  check('1366 px wide: no sideways page scroll and the header is sticky', ovf.sw <= ovf.iw + 1 && ovf.headPos === 'sticky', JSON.stringify(ovf));
  await b.shot('tab_all.png', false);

  // controls on the first tab
  await b.ev(`(()=>{const q=document.getElementById('allQ');q.value='camera';q.dispatchEvent(new Event('input',{bubbles:true}))})()`);
  const camRows = await b.ev(`document.querySelectorAll('#allTable tbody tr').length`);
  const expCam = [...data.features, ...data.extras].filter(f => ('camera' + '').split(' ').every(t => (f.name + ' ' + f.def).toLowerCase().includes(t))).length;
  check('search "camera" shows the rows whose name or definition has it', camRows === expCam && camRows > 0, camRows + ' vs ' + expCam);
  await b.ev(`(()=>{const q=document.getElementById('allQ');q.value='';q.dispatchEvent(new Event('input',{bubbles:true}))})()`);

  // ---- 2. tick two features (trusted clicks), counter, reload
  const keys2 = await b.ev(`[...document.querySelectorAll('#allTable tbody tr')].slice(0,2).map(r=>r.getAttribute('data-key'))`);
  for (const k of keys2) await clickSel(`#allTable tbody tr[data-key="${k}"] input[type=checkbox]`);
  const cnt = await text('#counter');
  check('ticking two features updates the counter', /^2 vital features chosen$/.test(cnt.trim()), cnt);
  harvest();
  await b.nav(PAGE);
  await b.waitFor("document.querySelectorAll('#allTable tbody tr').length>0", 40000);
  await sleep(300);
  const cnt2 = await text('#counter');
  const checkedAfter = await b.ev(`[...document.querySelectorAll('#allTable tbody tr input:checked')].map(i=>i.closest('tr').getAttribute('data-key'))`);
  check('the two vital features survive a reload (localStorage vital_features_v1)', /^2 vital features chosen$/.test(cnt2.trim()) && JSON.stringify(checkedAfter) === JSON.stringify(keys2), cnt2 + ' ' + checkedAfter.join());
  const stored = await b.ev(`window.localStorage.getItem('vital_features_v1')`);
  check('the stored value is a list of the two ids under the key vital_features_v1', stored && JSON.stringify(JSON.parse(stored).sort()) === JSON.stringify([...keys2].sort()), String(stored));
  await b.ev(`document.getElementById('allVitalOnly').click()`);
  const vRows = await b.ev(`document.querySelectorAll('#allTable tbody tr').length`);
  check('"Show only my vital features" shows exactly the two', vRows === 2, String(vRows));
  await b.ev(`document.getElementById('allVitalOnly').click()`);
  await b.ev(`document.getElementById('allNoneFull').click()`);
  const nfRows = await b.ev(`document.querySelectorAll('#allTable tbody tr').length`);
  const expNF = F92.filter(f => simIds.every(s => ans(s, f).s !== 'yes')).length + data.extras.filter(e => e.s !== 'yes').length;
  check('"Show only features that no simulator has fully" matches the data', nfRows === expNF, nfRows + ' vs ' + expNF);
  await b.ev(`document.getElementById('allNoneFull').click()`);

  // ---- 3. Filter tab: ranking for all 92
  await tab('Filter');
  // Question 1: which simulators have these features (the two vital ones)
  await clickSel('#q1Vital'); await sleep(300);
  const q1rows = await b.ev(`[...document.querySelectorAll('#q1Table tbody tr')].map(r=>({sim:r.getAttribute('data-sim'),has:+r.getAttribute('data-has'),all:r.getAttribute('data-hasall')==='1',sum:r.querySelector('.q1-sum').innerText}))`);
  const expQ1 = simIds.map(s => ({ sim: s, has: keys2.filter(k => ans(s, k).s === 'yes').length }));
  const q1bad = q1rows.filter(r => { const e = expQ1.find(x => x.sim === r.sim); return !e || e.has !== r.has || r.all !== (e.has === 2) || r.sum !== (e.has === 2 ? 'has all 2' : 'has ' + e.has + ' of 2'); });
  check('Question 1: 22 simulators, each "has K of M" / "has all M" equals the data, "has all" first', q1rows.length === 22 && q1bad.length === 0 && q1rows.every((r, i) => i === 0 || q1rows[i - 1].all || !r.all), q1bad.map(r => r.sim).join());
  // Question 2: default A = Crazyflow, B = all the others, in ALL of A / NONE of B
  const q2keys = await b.ev(`[...document.querySelectorAll('#q2Table tbody tr')].map(r=>r.getAttribute('data-key'))`);
  const expQ2 = F92.filter(f => ans('crazyflow', f).s === 'yes' && simIds.filter(s => s !== 'crazyflow').every(s => ans(s, f).s !== 'yes'));
  check('Question 2: Crazyflow vs all the others (all / none) lists the features the data gives (' + expQ2.length + ')', JSON.stringify(q2keys) === JSON.stringify(expQ2));
  await clickSel('#qSwap'); await sleep(300);
  const q2sw = await b.ev(`[...document.querySelectorAll('#q2Table tbody tr')].map(r=>r.getAttribute('data-key'))`);
  const expSw = F92.filter(f => simIds.filter(s => s !== 'crazyflow').every(s => ans(s, f).s === 'yes') && ans('crazyflow', f).s !== 'yes');
  check('Question 2: "Swap A and B" gives the reverse direction (' + expSw.length + ')', JSON.stringify(q2sw) === JSON.stringify(expSw));
  await b.shot('tab_filter_questions.png', false);
  await clickSel('input[name=fset][value=asked]');
  await sleep(300);
  const rank = await b.ev(`[...document.querySelectorAll('#rankTable tbody tr')].map(r=>({sim:r.getAttribute('data-sim'),yes:+r.querySelector('.n-yes').getAttribute('data-v'),part:+r.querySelector('.n-part').getAttribute('data-v'),lack:+r.querySelector('.n-lack').getAttribute('data-v')}))`);
  const expRank = simIds.map(s => ({ sim: s, yes: F92.filter(f => ans(s, f).s === 'yes').length, part: F92.filter(f => ans(s, f).s === 'partial').length, lack: F92.filter(f => ['no', 'not_found'].includes(ans(s, f).s)).length }));
  const mism = rank.filter(r => { const e = expRank.find(x => x.sim === r.sim); return !e || e.yes !== r.yes || e.part !== r.part || e.lack !== r.lack; });
  check('Filter: ranking for "All 92" has 22 rows and every count equals the data', rank.length === 22 && mism.length === 0, mism.map(m => m.sim).join());
  check('Filter: ranking is sorted by the first number', rank.every((r, i) => i === 0 || rank[i - 1].yes >= r.yes));
  const mxCells = await b.ev(`document.querySelectorAll('#matrix td.cell').length`);
  check('Filter: the matrix has 92 x 22 cells', mxCells === 92 * 22, String(mxCells));
  const mxBad = await b.ev(`[...document.querySelectorAll('#matrix td.cell')].filter(c=>c.getAttribute('data-s')==='no').length`);
  const expNo = simIds.reduce((a, s) => a + F92.filter(f => ans(s, f).s === 'no').length, 0);
  check('Filter: the matrix keeps "no" (X) apart from "not found" (?)', mxBad === expNo && expNo > 0, mxBad + ' vs ' + expNo);
  const allOf = await b.ev(`document.getElementById('allOfResult')?document.getElementById('allOfResult').getAttribute('data-count'):null`);
  const expAll = simIds.filter(s => F92.every(f => ans(s, f).s === 'yes')).length;
  check('Filter: "which simulators have ALL" count equals the data', allOf !== null && +allOf === expAll, allOf + ' vs ' + expAll);
  // toggle: count part
  await clickSel('#countPart'); await sleep(300);
  const rank2 = await b.ev(`[...document.querySelectorAll('#rankTable tbody tr')].map(r=>({sim:r.getAttribute('data-sim'),has:+r.querySelector('.n-yes').getAttribute('data-v')+ +r.querySelector('.n-part').getAttribute('data-v')}))`);
  check('Filter: with "Count part as having it" the order follows fully + part', rank2.every((r, i) => i === 0 || rank2[i - 1].has >= r.has));
  await clickSel('#countPart'); await sleep(200);
  // vital set with the two ticked features
  await clickSel('input[name=fset][value=vital]'); await sleep(300);
  const vm = await b.ev(`document.querySelectorAll('#matrix tbody tr').length`);
  check('Filter: "My vital features" gives 2 matrix rows', vm === 2, String(vm));
  await clickSel('input[name=fset][value=asked]'); await sleep(300);
  // quick compare
  await b.ev(`(()=>{const a=document.getElementById('cmpA'),c=document.getElementById('cmpB');a.value='crazyflow';a.dispatchEvent(new Event('change'));c.value='airsim';c.dispatchEvent(new Event('change'))})()`);
  await sleep(300);
  const aOnly = await b.ev(`[...document.querySelectorAll('#cmpAOnly li[data-key]')].map(l=>l.getAttribute('data-key'))`);
  const bOnly = await b.ev(`[...document.querySelectorAll('#cmpBOnly li[data-key]')].map(l=>l.getAttribute('data-key'))`);
  const expA = F92.filter(f => ans('crazyflow', f).s === 'yes' && ans('airsim', f).s !== 'yes');
  const expB = F92.filter(f => ans('airsim', f).s === 'yes' && ans('crazyflow', f).s !== 'yes');
  check('Compare Crazyflow and AirSim: "Crazyflow has, AirSim lacks" equals the logic test lists (' + expA.length + ')', JSON.stringify(aOnly) === JSON.stringify(expA));
  check('Compare Crazyflow and AirSim: "AirSim has, Crazyflow lacks" equals the logic test lists (' + expB.length + ')', JSON.stringify(bOnly) === JSON.stringify(expB));
  // a matrix cell opens the details with a quotation
  await clickSel('#matrix td.cell[data-s=yes]');
  await sleep(300);
  const q1 = await b.ev(`document.querySelectorAll('#details blockquote').length`);
  check('clicking a matrix cell opens details with a quotation', q1 > 0, String(q1));
  // "only without a graphics card"
  await b.clickText('button', 'Only without a graphics card'); await sleep(300);
  const noGpu = await b.ev(`document.querySelectorAll('#rankTable tbody tr').length`);
  check('"Only without a graphics card" leaves the simulators without a GPU need', noGpu === data.sims.filter(s => !s.gpu).length, noGpu + ' vs ' + data.sims.filter(s => !s.gpu).length);
  await b.clickText('button', 'All'); await sleep(200);
  await b.shot('tab_filter.png', false);
  await b.shot('tab_filter_full.png', true);

  // ---- 4. Graph tab
  await tab('Graph');
  const gms = await b.waitFor("document.querySelectorAll('#graph circle[data-node]').length>=120", 40000);
  const nodes = await b.ev(`document.querySelectorAll('#graph circle[data-node]').length`);
  const lines2 = await b.ev(`document.querySelectorAll('#graph line.edge').length`);
  check('Graph draws at least 120 nodes (extras off: 121) and about 1,300 edges', gms >= 0 && nodes === 121 && lines2 === 851 + 349 + 92, nodes + ' nodes, ' + lines2 + ' edges');
  const dashed = await b.ev(`document.querySelectorAll('#graph line.edge.partial').length`);
  check('Graph: partial edges are dashed (class partial, dash array) and counted', dashed === 349 && (await b.ev(`getComputedStyle(document.querySelector('#graph line.edge.partial')).strokeDasharray`)) !== 'none');
  await b.shot('tab_graph.png', false);
  // zoom with the wheel and pan by dragging the background
  const svgBox = await b.ev(`(()=>{const r=document.getElementById('graph').getBoundingClientRect();return [r.left,r.top,r.width,r.height]})()`);
  const tr0 = await b.ev(`document.getElementById('viewport').getAttribute('transform')`);
  await b.mm('mouseWheel', svgBox[0] + svgBox[2] / 2, svgBox[1] + svgBox[3] / 2, { dy: -300 }); await sleep(300);
  const tr1 = await b.ev(`document.getElementById('viewport').getAttribute('transform')`);
  check('Graph: the mouse wheel zooms', tr0 !== tr1, tr0 + ' -> ' + tr1);
  const spot = await b.ev(`(()=>{const r=document.getElementById('graph').getBoundingClientRect();for(const [fx,fy] of [[.02,.02],[.98,.02],[.02,.98],[.98,.98],[.5,.02]]){const x=r.left+r.width*fx,y=r.top+r.height*fy;const e=document.elementFromPoint(x,y);if(e&&e.id==='graph')return [x,y]}return null})()`);
  if (spot) {
    await b.drag(spot[0], spot[1], spot[0] + 40, spot[1] + 30);
    const tr2 = await b.ev(`document.getElementById('viewport').getAttribute('transform')`);
    check('Graph: dragging the background pans', tr2 !== tr1, tr1 + ' -> ' + tr2);
  } else check('Graph: dragging the background pans', false, 'no empty spot found');
  await b.clickText('button', 'Fit to window'); await sleep(300);
  // click a feature node (try nodes until a trusted click lands on one; the first hit is used)
  const candidates = await b.ev(`[...document.querySelectorAll('#graph circle.node.feature')].map(c=>c.getAttribute('data-node'))`);
  let hitNode = null;
  for (const id of candidates) {
    const c = await b.ev(`(()=>{const e=document.querySelector('#graph circle[data-node="${id}"]');const r=e.getBoundingClientRect();const x=r.left+r.width/2,y=r.top+r.height/2;const t=document.elementFromPoint(x,y);return t===e?[x,y]:null})()`);
    if (c) { await b.click(c[0], c[1]); hitNode = id; break; }
  }
  await sleep(300);
  const hitKey = hitNode && hitNode.slice(2);
  const quotes = await b.ev(`[...document.querySelectorAll('#details blockquote')].map(q=>q.innerText.replace(/\\s+/g,' ').trim())`);
  const expQ = hitKey ? simIds.map(s => ans(s, hitKey).q).filter(Boolean).map(q => ('“' + q + '”').replace(/\s+/g, ' ').trim()) : [];
  check('clicking a feature node opens details with the quotation (' + hitNode + ')', hitNode && quotes.length > 0 && quotes.every(q => expQ.includes(q)), quotes.length + ' quotes');
  const hot = await b.ev(`document.querySelectorAll('#graph line.edge.hot').length`);
  const dim = await b.ev(`document.querySelectorAll('#graph circle.node.dim').length`);
  check('clicking a node highlights its neighbors and dims the rest', hot > 0 && dim > 0, hot + ' hot edges, ' + dim + ' dim nodes');
  const srcShown = await b.ev(`document.querySelectorAll('#details code').length`);
  check('details show the source file path', srcShown > 0);
  // click a simulator node
  const sc = await b.ev(`(()=>{const e=document.querySelector('#graph circle[data-node="s:crazyflow"]');const r=e.getBoundingClientRect();const x=r.left+r.width/2,y=r.top+r.height/2;return document.elementFromPoint(x,y)===e?[x,y]:null})()`);
  if (sc) { await b.click(sc[0], sc[1]); await sleep(300); }
  const simDet = await text('#details');
  const expYes = F92.filter(f => ans('crazyflow', f).s === 'yes').length;
  check('clicking the Crazyflow node shows its counts', sc && simDet.includes(expYes + ' fully'), simDet.slice(0, 120).replace(/\s+/g, ' '));
  // extras toggle
  await clickSel('#gExtras'); await sleep(500);
  const nodesX = await b.ev(`document.querySelectorAll('#graph circle[data-node]').length`);
  check('"Show extra features" adds the 88 extras', nodesX === 121 + 88, String(nodesX));
  await clickSel('#gExtras'); await sleep(300);
  // only vital
  await clickSel('#gVital'); await sleep(400);
  const nodesV = await b.ev(`document.querySelectorAll('#graph circle.node.feature').length`);
  check('"Only my vital features" keeps the two chosen features', nodesV === 2, String(nodesV));
  await clickSel('#gVital'); await sleep(300);
  // search highlights
  await b.ev(`(()=>{const q=document.getElementById('gQ');q.value='airsim';q.dispatchEvent(new Event('input',{bubbles:true}))})()`);
  const hits = await b.ev(`document.querySelectorAll('#graph circle.node.hit').length`);
  check('the graph search highlights matching nodes', hits > 0, String(hits));
  await b.ev(`(()=>{const q=document.getElementById('gQ');q.value='';q.dispatchEvent(new Event('input',{bubbles:true}))})()`);
  // only these simulators
  await b.ev(`(()=>{const s=document.getElementById('gSims');[...s.options].forEach(o=>{o.selected=(o.value==='crazyflow'||o.value==='airsim')});s.dispatchEvent(new Event('change',{bubbles:true}))})()`);
  await sleep(500);
  const simNodes = await b.ev(`document.querySelectorAll('#graph circle.node.sim').length`);
  check('"Only these simulators" keeps the two picked', simNodes === 2, String(simNodes));
  await b.shot('tab_graph_two_sims.png', false);
  await b.clickText('button', 'All simulators'); await sleep(300);

  // ---- 5. saving and loading a choice
  await tab('All features');
  const loadOk = await b.ev(`(()=>{const t=document.getElementById('storeMsg');return typeof t.innerText})()`);
  check('Save and Load buttons are on the page', (await b.ev(`!!([...document.querySelectorAll('button')].find(x=>x.textContent==='Save my choice to a file')) && !!([...document.querySelectorAll('button')].find(x=>x.textContent==='Load a choice from a file'))`)) && loadOk === 'string');

  // ---- 6. no console errors, no network requests
  harvest();
  const bad = [...acc.console, ...acc.exceptions].filter(Boolean);
  check('no console errors or exceptions', bad.length === 0, bad.slice(0, 3).join(' | '));
  const outside = acc.urls.filter(u => !/^(file:|data:|blob:|about:)/.test(u));
  check('no network requests (every request is file:, data: or blob:)', outside.length === 0, outside.slice(0, 3).join(' '));
  check('no failed requests', acc.failed.length === 0, acc.failed.slice(0, 3).join(' | '));
  // positive control: an outside address must fail, so the "no network" claim can fail
  const ctl = await b.ev(`fetch('https://example.com/').then(()=>'reached').catch(()=>'blocked')`);
  check('control: a request to an outside address is blocked in this setup', ctl === 'blocked', ctl);
} catch (err) {
  check('test ran to the end without a script error', false, String(err && err.stack || err));
} finally {
  fs.writeFileSync(path.join(OUT, 'browser_test_result.txt'), lines.join('\n') + '\n');
  await b.close();
}
const failed = results.filter(r => !r.ok).length;
say(failed === 0 ? 'ALL ' + results.length + ' CHECKS PASS' : failed + ' of ' + results.length + ' FAILED');
fs.writeFileSync(path.join(OUT, 'browser_test_result.txt'), lines.join('\n') + '\n');
process.exit(failed === 0 ? 0 : 1);
