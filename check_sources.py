"""Cloud-only check of page/reference/problem_sources.html (run on the BMaser/temp machine, never on the owner's PC).
Checks: the page loads in Chromium with no JS errors; every table has its note tool and a "How to read this table." paragraph
in its block; every external link answers HTTP 200 (others are listed with their code, 404/410 fail); every row of the fit table
names a real plan row (id and title prefix) of page/plan.html. Writes out/sources_result.json and a screenshot."""
import json
import os
import re
import sys
import urllib.request
import urllib.error
from pathlib import Path
from playwright.sync_api import sync_playwright

os.makedirs("out", exist_ok=True)
page_path = Path("page/reference/problem_sources.html").resolve()
plan_text = Path("page/plan.html").read_text(encoding="utf-8")
res = {"errors": [], "fail": []}

# plan rows: ["id","title", ...] lines inside PARTS
start = plan_text.index("const PARTS = [")
end = plan_text.index("const ROUND_OF", start)
rows = re.findall(r'^\s*\["([^"]+)","((?:[^"\\]|\\.)*)"', plan_text[start:end], re.M)
plan_rows = [(i, t.replace('\\"', '"')) for i, t in rows]
res["plan_rows"] = len(plan_rows)

UA = "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/124.0.0.0 Safari/537.36"


def code_of(url):
    for method in ("HEAD", "GET"):
        req = urllib.request.Request(url, method=method, headers={"User-Agent": UA, "Accept": "text/html,*/*"})
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                return r.status
        except urllib.error.HTTPError as e:
            last = e.code
            if method == "GET":
                return last
        except Exception as e:  # network error
            last = "ERR " + type(e).__name__
            if method == "GET":
                return last
    return last


with sync_playwright() as p:
    b = p.chromium.launch()
    ctx = b.new_context(viewport={"width": 1500, "height": 1000})
    pg = ctx.new_page()
    pg.on("pageerror", lambda e: res["errors"].append(str(e)))
    pg.on("console", lambda m: m.type == "error" and res["errors"].append(m.text))
    pg.goto(page_path.as_uri())
    pg.wait_for_selector(".blk", timeout=8000)
    pg.wait_for_timeout(400)
    res["tables"] = pg.locator("table").count()
    res["tblk"] = pg.locator(".tblk").count()
    res["note_buttons"] = pg.locator('.tblk .ttools button:has-text("+ Note")').count()
    if not (res["tables"] == res["tblk"] == res["note_buttons"] and res["tables"] > 0):
        res["fail"].append("a table lacks its note tool")
    # every block that holds a table starts with its reading note
    missing = pg.evaluate("""() => [...document.querySelectorAll('.blk')].filter(b => b.querySelector('table')
        && !/How to read this table\\./.test(b.querySelector('.c').innerText)).map(b => b.dataset.b)""")
    res["blocks_without_note_text"] = missing
    if missing:
        res["fail"].append("blocks without 'How to read this table.': " + ",".join(missing))
    # fit table rows
    fit = pg.evaluate("""() => [...document.querySelectorAll('tr[data-plan]')].map(tr => {
        const td = tr.cells[0].cloneNode(true); td.querySelectorAll('small').forEach(s => s.remove());
        return [tr.dataset.plan, td.textContent.trim()]; })""")
    res["fit_rows"] = len(fit)
    bad = []
    for pid, shown in fit:
        if not any(i == pid and t.startswith(shown) for i, t in plan_rows):
            bad.append([pid, shown])
    res["fit_rows_not_in_plan"] = bad
    if bad or not fit:
        res["fail"].append("fit rows not in plan: %s" % bad)
    # links
    links = sorted(set(pg.evaluate("() => [...document.querySelectorAll('a[href]')].map(a => a.href).filter(h => /^https?:/.test(h))")))
    res["links"] = len(links)
    pg.screenshot(path="out/sources_page.png", full_page=True)
    pg.screenshot(path="out/sources_top.png")
    for name in ("srcs", "fit", "ex-l", "cou"):
        pg.locator('.blk[data-b="%s"]' % name).first.screenshot(path="out/block_%s.png" % name)
    b.close()

codes = {}
for u in links:
    codes[u] = code_of(u)
res["codes"] = codes
res["not_200"] = {u: c for u, c in codes.items() if c != 200}
res["dead"] = {u: c for u, c in codes.items() if c in (404, 410)}
if res["dead"]:
    res["fail"].append("dead links: %s" % list(res["dead"]))
if res["errors"]:
    res["fail"].append("JS errors")
Path("out/sources_result.json").write_text(json.dumps(res, indent=1))
print(json.dumps({k: v for k, v in res.items() if k not in ("codes", "plan_rows")}, indent=1)[:4000])
sys.exit(1 if res["fail"] else 0)
