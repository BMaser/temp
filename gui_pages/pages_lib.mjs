// Copy of lib.mjs (kept separate so the other agent can change lib.mjs). Headless Edge + DevTools protocol helper.
// record console errors and failed network requests, take full-page screenshots. Installs nothing.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { fileURLToPath, pathToFileURL } from 'node:url';

// Portable: every path is relative to the folder of this file. The reference files (the export's original pages) are in .\export\ .
const HERE = path.dirname(fileURLToPath(import.meta.url));
export const EXPORT = (process.env.EXPORT_DIR || path.join(HERE, 'export')) + path.sep;
export const OUT = HERE + path.sep;
export const sleep = ms => new Promise(r => setTimeout(r, ms));
// The browser: environment variable BROWSER, else Edge on Windows, else the first Chrome/Chromium found on the PATH (Linux).
function findBrowser() {
  if (process.env.BROWSER) return process.env.BROWSER;
  const win = ['C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe', 'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe'].find(p => fs.existsSync(p));
  if (win) return win;
  for (const n of ['google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser', 'chrome'])
    for (const d of (process.env.PATH || '').split(path.delimiter)) if (d && fs.existsSync(path.join(d, n))) return path.join(d, n);
  throw new Error('no browser found: set the environment variable BROWSER');
}
const EDGE = findBrowser();
export const BROWSER_PATH = EDGE;
const LINUX = process.platform === 'linux' ? ['--no-sandbox', '--disable-dev-shm-usage'] : [];

export async function launch(extraFlags = [], W = 1440, H = 900) {
  const port = 9300 + Math.floor(Math.random() * 600);
  const prof = fs.mkdtempSync(path.join(os.tmpdir(), 'gui_check_'));
  const proc = spawn(EDGE, ['--headless=new', '--no-first-run', ...LINUX, '--remote-debugging-port=' + port, '--remote-allow-origins=*', '--user-data-dir=' + prof, '--window-size=' + W + ',' + H, ...extraFlags, 'about:blank'], { stdio: 'ignore' });
  let wsUrl = null;
  for (let i = 0; i < 600 && !wsUrl; i++) {
    try { const l = await (await fetch('http://127.0.0.1:' + port + '/json/list')).json(); const p = l.find(t => t.type === 'page'); if (p) wsUrl = p.webSocketDebuggerUrl; } catch (e) { }
    if (!wsUrl) await sleep(200);
  }
  if (!wsUrl) { proc.kill(); throw new Error('could not reach Edge'); }
  const ws = new WebSocket(wsUrl); await new Promise(r => ws.addEventListener('open', r));
  let mid = 0; const pend = new Map(); const log = { console: [], failed: [], http: [], exceptions: [] };
  ws.addEventListener('message', e => {
    const m = JSON.parse(e.data);
    if (m.id && pend.has(m.id)) { pend.get(m.id)(m); pend.delete(m.id); return; }
    const p = m.params || {};
    if (m.method === 'Runtime.consoleAPICalled' && (p.type === 'error' || p.type === 'warning')) log.console.push(p.type + ': ' + (p.args || []).map(a => a.value !== undefined ? a.value : a.description).join(' ').slice(0, 300));
    if (m.method === 'Runtime.exceptionThrown') log.exceptions.push(((p.exceptionDetails || {}).exception || {}).description || (p.exceptionDetails || {}).text);
    if (m.method === 'Log.entryAdded' && (p.entry.level === 'error' || p.entry.level === 'warning')) log.console.push('log-' + p.entry.level + ': ' + p.entry.text.slice(0, 250) + ' ' + (p.entry.url || ''));
    if (m.method === 'Network.loadingFailed') log.failed.push((p.errorText || '') + ' ' + (p.blockedReason || '') + ' ' + (p.corsErrorStatus ? JSON.stringify(p.corsErrorStatus) : '') + ' reqId=' + p.requestId);
    if (m.method === 'Network.requestWillBeSent') log.http.push({ id: p.requestId, url: p.request.url });
    if (m.method === 'Network.responseReceived' && p.response.status >= 400) log.failed.push('HTTP ' + p.response.status + ' ' + p.response.url);
  });
  const send = (method, params = {}) => new Promise(res => { const i = ++mid; pend.set(i, res); ws.send(JSON.stringify({ id: i, method, params })); }).then(m => { if (m.error) throw new Error(method + ': ' + JSON.stringify(m.error)); return m.result; });
  const ev = async expr => { const r = await send('Runtime.evaluate', { expression: expr, returnByValue: true, awaitPromise: true }); if (r.exceptionDetails) throw new Error('page error: ' + (r.exceptionDetails.exception && r.exceptionDetails.exception.description || r.exceptionDetails.text)); return r.result.value; };
  for (const d of ['Page', 'Runtime', 'Network', 'Log']) await send(d + '.enable');
  await send('Emulation.setDeviceMetricsOverride', { width: W, height: H, deviceScaleFactor: 1, mobile: false });
  const api = { send, ev, log, proc, prof, W, H };
  api.resolveUrls = () => { const m = new Map(log.http.map(h => [h.id, h.url])); log.failed = log.failed.map(s => s.replace(/reqId=(\S+)/, (_, id) => m.get(id) || id)); };
  api.nav = async url => { log.console.length = 0; log.failed.length = 0; log.exceptions.length = 0; log.http.length = 0; await send('Page.navigate', { url }); };
  api.shot = async (name, full = true) => {
    let clip;
    if (full) {
      const lm = await send('Page.getLayoutMetrics'); const cs = lm.cssContentSize || lm.contentSize;
      clip = { x: 0, y: 0, width: Math.min(cs.width, 2400), height: Math.min(cs.height, 6000), scale: 1 };
    }
    const r = await send('Page.captureScreenshot', { format: 'png', captureBeyondViewport: full, ...(clip ? { clip } : {}) });
    fs.writeFileSync(OUT + name, Buffer.from(r.data, 'base64')); return name;
  };
  api.mm = (type, x, y, o = {}) => send('Input.dispatchMouseEvent', { type, x, y, button: o.button || 'none', buttons: o.buttons || 0, clickCount: o.clickCount || 0, modifiers: o.mod || 0, deltaX: o.dx || 0, deltaY: o.dy || 0 });
  api.click = async (x, y) => { await api.mm('mouseMoved', x, y); await api.mm('mousePressed', x, y, { button: 'left', buttons: 1, clickCount: 1 }); await api.mm('mouseReleased', x, y, { button: 'left', buttons: 0, clickCount: 1 }); await sleep(250); };
  api.drag = async (x0, y0, x1, y1) => { await api.mm('mouseMoved', x0, y0); await api.mm('mousePressed', x0, y0, { button: 'left', buttons: 1, clickCount: 1 }); for (let k = 1; k <= 8; k++) { await api.mm('mouseMoved', x0 + (x1 - x0) * k / 8, y0 + (y1 - y0) * k / 8, { button: 'left', buttons: 1 }); await sleep(25); } await api.mm('mouseReleased', x1, y1, { button: 'left', buttons: 0, clickCount: 1 }); await sleep(200); };
  // click the center of the first element of `sel` whose text equals `txt` (trusted mouse click)
  api.clickText = async (sel, txt) => {
    const r = await ev(`(()=>{const e=[...document.querySelectorAll(${JSON.stringify(sel)})].find(x=>x.textContent.trim()===${JSON.stringify(txt)});if(!e)return null;e.scrollIntoView({block:'center'});const r=e.getBoundingClientRect();return [r.left+r.width/2,r.top+r.height/2]})()`);
    if (!r) throw new Error('no ' + sel + ' with text ' + txt); await api.click(r[0], r[1]); return r;
  };
  api.waitFor = async (expr, ms = 40000) => { const t0 = Date.now(); while (Date.now() - t0 < ms) { try { if (await ev(expr)) return Date.now() - t0; } catch (e) { } await sleep(250); } return -1; };
  api.close = async () => { try { send('Browser.close').catch(() => { }); } catch (e) { } await sleep(1200); try { proc.kill(); } catch (e) { } await sleep(500); try { fs.rmSync(prof, { recursive: true, force: true }); } catch (e) { } };
  return api;
}
export const fileUrl = name => pathToFileURL(path.join(EXPORT, name)).href;
