// Logic test for feature_filter: cuts the LOGIC block out of the template, evaluates it with plain node (no browser),
// and checks it against numbers computed here straight from features_data.json (not with the functions under test).
// A positive control runs a deliberately wrong copy of the logic and must make a check fail.
// Usage: node test_filter_logic.mjs        Exit code 0 only if every check passes and the control fails as it should.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const first = (...c) => c.find(p => fs.existsSync(p));
const TEMPLATE = first(path.join(HERE, 'feature_filter_template.html'));
const DATAFILE = first(path.join(HERE, 'features_data.json'), path.join(HERE, '..', '2026-10-14_feature_matrix', 'features_data.json'));
if (!TEMPLATE || !DATAFILE) { console.log('FAIL cannot find the template or the data'); process.exit(2); }
const tpl = fs.readFileSync(TEMPLATE, 'utf8');
const data = JSON.parse(fs.readFileSync(DATAFILE, 'utf8'));
const b = tpl.indexOf('/* LOGIC-BEGIN */'), e = tpl.indexOf('/* LOGIC-END */');
if (b < 0 || e < b) { console.log('FAIL LOGIC markers not found'); process.exit(2); }
const LOGIC = tpl.slice(b, e);

const NAMES = ['simsHaving', 'featuresInAnotB', 'rankSims', 'simsWithAll', 'lacks', 'takeFrom', 'compare', 'graphModel', 'layoutGraph', 'parseChoice', 'makeChoice', 'searchMatch', 'filterRows', 'rowInfo', 'summarizeSim', 'statusOf', 'answerFor', 'allKeys', 'orderKeys'];
const load = code => new Function(code + '\nreturn {' + NAMES.join(',') + '};')();

// ---- independent expectations, straight from the data
const F92 = data.features.map(f => f.id);
const simIds = data.sims.map(s => s.id);
const ans = (sim, f) => data.answers[sim][f];
const yesSet = sim => new Set(F92.filter(f => ans(sim, f).s === 'yes'));
const countOf = (sim, st) => F92.filter(f => ans(sim, f).s === st).length;

function runAll(L) {
  const out = [];
  const check = (name, ok, detail = '') => out.push({ name, ok: !!ok, detail });

  // 1. ranking over the 92, toggle off and on
  const r0 = L.rankSims(data, simIds, F92, false);
  check('rankSims returns one row per simulator', r0.length === 22);
  const bad0 = r0.filter(r => r.yes !== countOf(r.sim, 'yes') || r.partial !== countOf(r.sim, 'partial') || r.no !== countOf(r.sim, 'no') || r.notFound !== countOf(r.sim, 'not_found') || r.has !== countOf(r.sim, 'yes'));
  check('rankSims (part not counted): yes/partial/no/? per simulator equal the data, has = yes', bad0.length === 0, bad0.map(r => r.sim).join(','));
  check('rankSims sorted by the first number (descending)', r0.every((r, i) => i === 0 || r0[i - 1].has >= r.has));
  const r1 = L.rankSims(data, simIds, F92, true);
  const bad1 = r1.filter(r => r.has !== countOf(r.sim, 'yes') + countOf(r.sim, 'partial'));
  check('rankSims (part counted): has = yes + partial', bad1.length === 0, bad1.map(r => r.sim).join(','));
  check('rankSims totals: yes+partial+no+? = 92 for each simulator', r0.every(r => r.yes + r.partial + r.no + r.notFound === 92 && r.unasked === 0));
  const sumYes = r0.reduce((a, r) => a + r.yes, 0), sumPart = r0.reduce((a, r) => a + r.partial, 0);
  let dYes = 0, dPart = 0; for (const s of simIds) for (const f of F92) { if (ans(s, f).s === 'yes') dYes++; if (ans(s, f).s === 'partial') dPart++; }
  check('rankSims grand totals equal the data (yes ' + dYes + ', partial ' + dPart + ')', sumYes === dYes && sumPart === dPart, sumYes + '/' + sumPart);

  // 2. crazyflow vs airsim
  for (const [a, bb] of [['crazyflow', 'airsim'], ['airsim', 'crazyflow']]) {
    const c = L.compare(data, a, bb, F92);
    const expA = F92.filter(f => ans(a, f).s === 'yes' && ans(bb, f).s !== 'yes');
    const expB = F92.filter(f => ans(bb, f).s === 'yes' && ans(a, f).s !== 'yes');
    check('compare(' + a + ',' + bb + '): first-only list equals the data (' + expA.length + ')', JSON.stringify(c.aOnly.map(x => x.key)) === JSON.stringify(expA));
    check('compare(' + a + ',' + bb + '): second-only list equals the data (' + expB.length + ')', JSON.stringify(c.bOnly.map(x => x.key)) === JSON.stringify(expB));
    check('compare(' + a + ',' + bb + '): both-yes count equals the data', c.both.length === F92.filter(f => ans(a, f).s === 'yes' && ans(bb, f).s === 'yes').length);
  }
  check('compare is not empty both ways (the control would be blind otherwise)', L.compare(data, 'crazyflow', 'airsim', F92).aOnly.length > 0 && L.compare(data, 'crazyflow', 'airsim', F92).bOnly.length > 0);

  // 3. simsWithAll
  const pick2 = F92.filter(f => ans('crazyflow', f).s === 'yes').slice(0, 2);
  const exp2 = simIds.filter(s => pick2.every(f => ans(s, f).s === 'yes'));
  check('simsWithAll for two features (' + pick2.join(',') + ') equals the data: ' + exp2.join(','), JSON.stringify(L.simsWithAll(data, simIds, pick2, false)) === JSON.stringify(exp2) && exp2.includes('crazyflow'));
  // a pair where part makes a difference
  let pairP = null;
  outer: for (const f1 of F92) for (const f2 of F92) { if (f1 >= f2) continue; const strict = simIds.filter(s => ans(s, f1).s === 'yes' && ans(s, f2).s === 'yes').length; const loose = simIds.filter(s => ['yes', 'partial'].includes(ans(s, f1).s) && ['yes', 'partial'].includes(ans(s, f2).s)).length; if (loose > strict && strict > 0) { pairP = [f1, f2]; break outer; } }
  const expS = simIds.filter(s => pairP.every(f => ans(s, f).s === 'yes')), expL = simIds.filter(s => pairP.every(f => ['yes', 'partial'].includes(ans(s, f).s)));
  check('simsWithAll strict and with-part differ as the data says for ' + pairP.join(','), JSON.stringify(L.simsWithAll(data, simIds, pairP, false)) === JSON.stringify(expS) && JSON.stringify(L.simsWithAll(data, simIds, pairP, true)) === JSON.stringify(expL) && expL.length > expS.length);
  check('simsWithAll with no chosen feature is empty', L.simsWithAll(data, simIds, [], false).length === 0);
  const unionStrict = L.simsWithAll(data, simIds, F92, false);
  check('simsWithAll over all 92 equals the data', JSON.stringify(unionStrict) === JSON.stringify(simIds.filter(s => F92.every(f => ans(s, f).s === 'yes'))));

  // 4. lacks and takeFrom
  const lc = L.lacks(data, 'crazyflow', F92, false);
  const expLc = F92.filter(f => ans('crazyflow', f).s !== 'yes');
  check('lacks(crazyflow, 92, part off) equals the data (' + expLc.length + ')', JSON.stringify(lc.map(x => x.key)) === JSON.stringify(expLc));
  const lcP = L.lacks(data, 'crazyflow', F92, true);
  check('lacks(crazyflow, 92, part on) drops the partial ones', JSON.stringify(lcP.map(x => x.key)) === JSON.stringify(F92.filter(f => !['yes', 'partial'].includes(ans('crazyflow', f).s))));
  const sample = expLc.slice(0, 12).concat(expLc.slice(-3));
  check('takeFrom equals the data for 15 lacking features of crazyflow', sample.every(f => JSON.stringify(L.takeFrom(data, f, 'crazyflow')) === JSON.stringify(simIds.filter(s => s !== 'crazyflow' && ans(s, f).s === 'yes'))));
  const f0 = F92.find(f => ans('crazyflow', f).s === 'yes' && ans('airsim', f).s === 'yes');
  check('takeFrom leaves out the excepted simulator', !L.takeFrom(data, f0, 'airsim').includes('airsim') && L.takeFrom(data, f0, 'airsim').includes('crazyflow'));

  // 5. graph
  const g = L.graphModel(data, { showExtras: false });
  const types = t => g.nodes.filter(n => n.type === t).length;
  check('graph with extras off: 22 sims + 92 features + 7 groups = 121 nodes', g.nodes.length === 121 && types('sim') === 22 && types('feature') === 92 && types('group') === 7, g.nodes.length + '');
  const eY = g.edges.filter(x => x.kind === 'yes').length, eP = g.edges.filter(x => x.kind === 'partial').length, eG = g.edges.filter(x => x.kind === 'group').length;
  check('graph edges: yes ' + dYes + ', partial ' + dPart + ', group 92', eY === dYes && eP === dPart && eG === 92 && g.edges.length === dYes + dPart + 92, eY + '/' + eP + '/' + eG);
  check('graph has no edge for no or not_found', g.edges.every(x => { if (x.kind !== 'yes' && x.kind !== 'partial') return true; const s = x.from.slice(2), f = x.to.slice(2); return ans(s, f).s === x.kind; }));
  check('graph edges point to existing nodes', (() => { const ids = new Set(g.nodes.map(n => n.id)); return g.edges.every(x => ids.has(x.from) && ids.has(x.to)); })());
  const gx = L.graphModel(data, { showExtras: true });
  const exYes = data.extras.filter(x => x.s === 'yes').length, exPart = data.extras.filter(x => x.s === 'partial').length;
  check('graph with extras on: 88 more nodes, 88 extra edges to their own simulator', gx.nodes.length === 121 + data.extras.length && gx.edges.filter(x => x.kind === 'extra').length === data.extras.length && data.extras.every(x => gx.edges.some(ed => ed.kind === 'extra' && ed.from === 's:' + x.sim && ed.to === 'f:' + x.key)) && gx.edges.filter(x => x.kind === 'extra' && x.status === 'yes').length === exYes && gx.edges.filter(x => x.kind === 'extra' && x.status === 'partial').length === exPart);
  const gv = L.graphModel(data, { showExtras: false, vitalOnly: true, vital: ['A01', 'C02'] });
  check('graph only-vital keeps just the chosen features', gv.nodes.filter(n => n.type === 'feature').map(n => n.ref).join() === 'A01,C02' && gv.nodes.filter(n => n.type === 'group').length === 2);
  const gs = L.graphModel(data, { showExtras: false, simIds: ['crazyflow', 'airsim'] });
  check('graph only-these-simulators keeps 2 simulators and only their edges', gs.nodes.filter(n => n.type === 'sim').length === 2 && gs.edges.filter(x => x.kind === 'yes').length === yesSet('crazyflow').size + yesSet('airsim').size);

  // 6. layout
  const lay = L.layoutGraph(g, 300, 1000, 640), lay2 = L.layoutGraph(g, 300, 1000, 640);
  check('layout is finite and deterministic', lay.length === 121 && lay.every(p => Number.isFinite(p.x) && Number.isFinite(p.y)) && JSON.stringify(lay) === JSON.stringify(lay2));
  const simPos = lay.filter((p, i) => g.nodes[i].type === 'sim');
  let minD = 1e9; for (let i = 0; i < simPos.length; i++) for (let j = i + 1; j < simPos.length; j++) minD = Math.min(minD, Math.hypot(simPos[i].x - simPos[j].x, simPos[i].y - simPos[j].y));
  check('layout keeps simulators apart (closest pair > 30 px)', minD > 30, 'closest pair ' + minD.toFixed(1));
  const layX = L.layoutGraph(gx, 300, 1000, 640);
  check('layout with extras is finite', layX.length === gx.nodes.length && layX.every(p => Number.isFinite(p.x) && Number.isFinite(p.y)));

  // 7. choice file
  const good = L.makeChoice(['A01', 'x:crazyflow:0'], '2026-10-14');
  const pg = L.parseChoice(good);
  check('parseChoice reads a file made by makeChoice', pg.ok && pg.ids.join() === 'A01,x:crazyflow:0' && pg.saved === '2026-10-14');
  const brokenTexts = ['{"app":"vital_features","ids":[', 'not json', '[1,2]', '{"app":"other","ids":["A01"]}', '{"app":"vital_features"}', '{"app":"vital_features","ids":["A01",5]}', '', 'null'];
  check('parseChoice rejects every broken file (' + brokenTexts.length + ' cases)', brokenTexts.every(t => L.parseChoice(t).ok === false && L.parseChoice(t).error.length > 0));

  // 8. search and filter rows
  check('searchMatch: all words must match name or definition, any case', L.searchMatch({ name: 'Open-source license', def: 'The code is published' }, 'LICENSE open') && !L.searchMatch({ name: 'Open-source license', def: 'x' }, 'license camera') && L.searchMatch({ name: 'a', def: 'b' }, ''));
  const rows = L.filterRows(data, { query: '', cat: '', vitalOnly: false, noneFull: false, vital: [] });
  check('filterRows with no filter lists 92 + 88 rows', rows.length === 180 && rows.slice(0, 92).join() === F92.join());
  const none = L.filterRows(data, { noneFull: true, vital: [] });
  check('filterRows "no simulator has it fully" equals the data', JSON.stringify(none.filter(k => !k.startsWith('x:'))) === JSON.stringify(F92.filter(f => simIds.every(s => ans(s, f).s !== 'yes'))) && none.filter(k => k.startsWith('x:')).every(k => data.extras.find(x => x.key === k).s !== 'yes'));
  const catB = L.filterRows(data, { cat: 'B', vital: [] });
  check('filterRows by group equals the data', catB.length === data.features.filter(f => f.cat === 'B').length + data.extras.filter(x => x.cat === 'B').length);
  const vOnly = L.filterRows(data, { vitalOnly: true, vital: ['A01', 'x:crazyflow:0'] });
  check('filterRows only-vital', vOnly.join() === 'A01,x:crazyflow:0');

  // 8b. Question 1: simsHaving
  const q1k = ['A01', 'B03', 'C05', 'D02', 'E04'];
  const st = (s, f) => ans(s, f).s;
  for (const cp of [false, true]) {
    const rows1 = L.simsHaving(data, q1k, cp);
    const expRows = simIds.map(s => ({ sim: s, full: q1k.filter(f => st(s, f) === 'yes'), part: q1k.filter(f => st(s, f) === 'partial'), lack: q1k.filter(f => ['no', 'not_found'].includes(st(s, f))) }));
    const okRows = rows1.length === 22 && rows1.every(r => { const e = expRows.find(x => x.sim === r.sim); const has = e.full.length + (cp ? e.part.length : 0); return JSON.stringify(r.full) === JSON.stringify(e.full) && JSON.stringify(r.part) === JSON.stringify(e.part) && JSON.stringify(r.lack) === JSON.stringify(e.lack) && r.has === has && r.hasAll === (has === q1k.length) && r.total === 5; });
    check('simsHaving (part ' + (cp ? 'counted' : 'not counted') + '): lists, has and hasAll equal the data for 5 features', okRows);
    const firstNon = rows1.findIndex(r => !r.hasAll);
    check('simsHaving (part ' + (cp ? 'counted' : 'not counted') + '): "has all" simulators come first, then by count', rows1.slice(0, firstNon < 0 ? rows1.length : firstNon).every(r => r.hasAll) && rows1.slice(firstNon < 0 ? rows1.length : firstNon).every(r => !r.hasAll) && rows1.every((r, i) => i === 0 || r.hasAll !== rows1[i - 1].hasAll || rows1[i - 1].has >= r.has));
  }
  const allQ1 = L.simsHaving(data, F92.filter(f => ans('crazyflow', f).s === 'yes').slice(0, 3), false);
  check('simsHaving: at least one simulator has all 3 features of Crazyflow (itself)', allQ1[0].hasAll && allQ1.some(r => r.sim === 'crazyflow' && r.hasAll));
  check('simsHaving with no features gives nobody "has all"', L.simsHaving(data, [], false).every(r => !r.hasAll));

  // 8c. Question 2: featuresInAnotB
  const keysOf = rr => rr.map(r => r.key);
  const cmpAB = L.compare(data, 'crazyflow', 'airsim', F92).aOnly.map(x => x.key);
  check('featuresInAnotB(A=[crazyflow], B=[airsim], all, none) equals compare\'s "first has, second lacks" (' + cmpAB.length + ')', JSON.stringify(keysOf(L.featuresInAnotB(data, ['crazyflow'], ['airsim'], 'all', 'none', false, F92))) === JSON.stringify(cmpAB) && cmpAB.length > 0);
  const cmpBA = L.compare(data, 'crazyflow', 'airsim', F92).bOnly.map(x => x.key);
  check('featuresInAnotB with A and B swapped equals the reverse list (' + cmpBA.length + ')', JSON.stringify(keysOf(L.featuresInAnotB(data, ['airsim'], ['crazyflow'], 'all', 'none', false, F92))) === JSON.stringify(cmpBA));
  const A2 = ['crazyflow', 'rotorpy', 'quadrace'], B2 = simIds.filter(s => !A2.includes(s));
  const has2 = (s, f, cp) => st(s, f) === 'yes' || (cp && st(s, f) === 'partial');
  for (const cp of [false, true]) for (const mA of ['all', 'any']) for (const mB of ['none', 'notall']) {
    const exp = F92.filter(f => { const a = A2.filter(s => has2(s, f, cp)), bb = B2.filter(s => has2(s, f, cp)); return (mA === 'all' ? a.length === A2.length : a.length > 0) && (mB === 'none' ? bb.length === 0 : bb.length < B2.length); });
    const got = L.featuresInAnotB(data, A2, B2, mA, mB, cp, F92);
    const okWho = got.every(r => JSON.stringify(r.aHave) === JSON.stringify(A2.filter(s => has2(s, r.key, cp))) && JSON.stringify(r.bLack.map(x => x.sim)) === JSON.stringify(B2.filter(s => !has2(s, r.key, cp))) && r.bLack.every(x => x.status === st(x.sim, r.key)));
    check('featuresInAnotB 3 simulators vs the other 19 (' + mA + '/' + mB + ', part ' + cp + '): ' + exp.length + ' features and who-lists equal the data', JSON.stringify(keysOf(got)) === JSON.stringify(exp) && okWho);
  }
  const anyNotAll = L.featuresInAnotB(data, A2, B2, 'any', 'notall', false, F92).length, allNone = L.featuresInAnotB(data, A2, B2, 'all', 'none', false, F92).length;
  check('featuresInAnotB: the loosest setting returns more than the strictest (so the modes matter)', anyNotAll > allNone, anyNotAll + ' > ' + allNone);
  check('featuresInAnotB drops a simulator listed in both A and B from B, and returns nothing for an empty side', L.featuresInAnotB(data, ['crazyflow'], ['crazyflow'], 'all', 'none', false, F92).length === 0 && L.featuresInAnotB(data, [], ['airsim'], 'all', 'none', false, F92).length === 0);
  // extras: only when A is exactly the one simulator that has the extra
  const exk = data.extras.filter(x => x.sim === 'crazyflow').map(x => x.key);
  const exYesCf = data.extras.filter(x => x.sim === 'crazyflow' && x.s === 'yes').map(x => x.key);
  const gotEx = L.featuresInAnotB(data, ['crazyflow'], ['airsim'], 'all', 'none', false, exk);
  check('featuresInAnotB with extras: A=[crazyflow] lists the yes-extras of Crazyflow (' + exYesCf.length + '), B shown as not asked', JSON.stringify(keysOf(gotEx)) === JSON.stringify(exYesCf) && gotEx.every(r => r.bLack[0].status === 'unasked'));
  check('featuresInAnotB with extras: A of two simulators lists no extra', L.featuresInAnotB(data, ['crazyflow', 'airsim'], ['rotorpy'], 'any', 'none', false, exk).length === 0 && L.featuresInAnotB(data, ['airsim'], ['rotorpy'], 'any', 'none', false, exk).length === 0);

  // 9. extras are asked of one simulator only
  const ex = data.extras[0];
  check('extras: status is the data for its own simulator and "unasked" for the others', L.statusOf(data, ex.sim, ex.key) === ex.s && simIds.filter(s => s !== ex.sim).every(s => L.statusOf(data, s, ex.key) === 'unasked'));
  const rx = L.rankSims(data, simIds, [ex.key], false);
  check('rankSims on one extra counts 21 simulators as "not asked"', rx.filter(r => r.unasked === 1).length === 21 && rx.find(r => r.sim === ex.sim).unasked === 0);
  const sm = L.summarizeSim(data, 'crazyflow');
  check('summarizeSim counts equal the data', sm.yes.length === countOf('crazyflow', 'yes') && sm.partial.length === countOf('crazyflow', 'partial') && sm.no.length === countOf('crazyflow', 'no') && sm.not_found.length === countOf('crazyflow', 'not_found') && sm.extras.yes.length + sm.extras.partial.length === data.extras.filter(x => x.sim === 'crazyflow').length);
  return out;
}

let failed = 0;
console.log('=== real logic');
const real = runAll(load(LOGIC));
for (const r of real) { console.log((r.ok ? 'PASS ' : 'FAIL ') + r.name + (r.detail ? '  [' + r.detail + ']' : '')); if (!r.ok) failed++; }

// ---- positive control: a deliberately wrong logic (counts partial as yes when the toggle is off) must fail a check
console.log('=== positive control (deliberately wrong logic: partial counted as yes when the toggle is off)');
const GOOD = 'const has = c.yes + (countPart ? c.partial : 0);';
if (!LOGIC.includes(GOOD)) { console.log('FAIL control: the line to break was not found'); process.exit(2); }
const wrong = LOGIC.replace(GOOD, 'const has = c.yes + c.partial;');
const wrongRun = runAll(load(wrong));
const wrongFails = wrongRun.filter(r => !r.ok);
console.log('wrong logic fails ' + wrongFails.length + ' check(s): ' + wrongFails.map(r => r.name).join(' | '));
const controlOk = wrongFails.some(r => /rankSims/.test(r.name));
console.log((controlOk ? 'PASS ' : 'FAIL ') + 'positive control: the wrong logic is caught by a rankSims check');
if (!controlOk) failed++;

// second control on a different function: takeFrom that does not leave out the excepted simulator
const GOOD2 = "data.sims.filter(s => s.id !== exceptSim && statusOf(data, s.id, featureKey) === 'yes')";
if (!LOGIC.includes(GOOD2)) { console.log('FAIL control 2: the line to break was not found'); process.exit(2); }
const wrong2 = runAll(load(LOGIC.replace(GOOD2, "data.sims.filter(s => statusOf(data, s.id, featureKey) === 'yes')")));
const c2 = wrong2.some(r => !r.ok && /takeFrom/.test(r.name));
console.log((c2 ? 'PASS ' : 'FAIL ') + 'positive control 2: takeFrom that keeps the excepted simulator is caught');
if (!c2) failed++;

const total = real.length + 2;
console.log(failed === 0 ? 'ALL ' + total + ' CHECKS PASS (' + real.length + ' on the real logic, 2 controls)' : failed + ' FAILED');
process.exit(failed === 0 ? 0 : 1);
