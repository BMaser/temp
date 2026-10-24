"""Check the autopilot map page (page/reference/autopilot_map.html) and the "Why you learn this" boxes of the section pages.
Run on the cloud machine only (never on the owner's PC):  python check_map.py
Checks:
 1. the map page opens with 0 console errors and 0 page errors;
 2. both tables are complete: rows row-1..row-10 (multirotor) and row-F1..row-F4 (fixed wing), each id once, 7 cells per row;
 3. every GitHub link has the form https://github.com/PX4/PX4-Autopilot/blob/d28a642/<file>[#La[-Lb]] or
    https://github.com/iamaisim/ProjectAirSim/blob/c80cff5/<file>[#La[-Lb]], the file exists at that commit (HTTP 200 on the raw file),
    and the line numbers lie inside the file;
 4. every local link of the map page resolves to a file in page/ and every #fragment to an id on the target page;
 5. every table has a "How to read this table." paragraph before it and a figure in the same block; figure captions run 1..3;
 6. each figure is photographed to out/; labels that leave the viewBox or overlap are errors;
 7. pages 2.1a and 2.1b have exactly one why block, with the same markup as 2.2a (paragraph, how-to, table, figure, "Without this page");
 8. every autopilot_map.html#row-... link on 2.1a, 2.1b, 2.2a and 2.2b points to an id that exists on the map page;
 9. the notes/marks tools work on the map page; banned word, 9.81, <sub> inside svg."""
import json
import re
import sys
import urllib.request
from pathlib import Path
from playwright.sync_api import sync_playwright

PAGE = Path("page")
MAP = (PAGE / "reference" / "autopilot_map.html").resolve()
SECT = {
    "2.1a": PAGE / "samples" / "2.1a_The_four_forces_one_at_a_time.html",
    "2.1b": PAGE / "samples" / "2.1b_Forces_together_tilt_wind_simulators.html",
    "2.2a": PAGE / "samples" / "2.2a_Position_velocity_acceleration_on_a_line.html",
    "2.2b": PAGE / "samples" / "2.2b_Three_dimensions_laps_falling_fixed_wing_simulators.html",
}
Path("out").mkdir(exist_ok=True)
res = {"errors": [], "map": {}, "sections": {}}
err = res["errors"]

CLIP_JS = """() => { const out = []; for (const s of document.querySelectorAll('figure svg')) { const vb = s.viewBox.baseVal; const root = s.getBoundingClientRect();
  const sc = root.width / vb.width;
  for (const t of s.querySelectorAll('text')) { const r = t.getBoundingClientRect(); const x0 = (r.left - root.left) / sc, x1 = (r.right - root.left) / sc, y0 = (r.top - root.top) / sc, y1 = (r.bottom - root.top) / sc;
    if (x0 < -1 || y0 < -1 || x1 > vb.width + 1 || y1 > vb.height + 1) out.push(s.id + ':' + t.textContent.slice(0, 30)); } } return out; }"""
OVERLAP_JS = """() => { const out = []; for (const s of document.querySelectorAll('figure svg')) { const ts = [...s.querySelectorAll('text')].map(t => ({t, r: t.getBoundingClientRect()}));
  for (let i = 0; i < ts.length; i++) for (let j = i + 1; j < ts.length; j++) { const a = ts[i].r, c = ts[j].r;
    const w = Math.min(a.right, c.right) - Math.max(a.left, c.left), h = Math.min(a.bottom, c.bottom) - Math.max(a.top, c.top);
    if (w > 1 && h > 1 && w * h > 0.15 * Math.min(a.width * a.height, c.width * c.height)) out.push(s.id + ': "' + ts[i].t.textContent.slice(0, 20) + '" / "' + ts[j].t.textContent.slice(0, 20) + '"'); } } return out; }"""
# text that runs over a box edge: a text box that crosses a rect border inside the svg
BOXTEXT_JS = """() => { const out = []; for (const s of document.querySelectorAll('figure svg')) { const rects = [...s.querySelectorAll('rect')].filter(r => +r.getAttribute('width') >= 60 && +r.getAttribute('height') >= 30).map(r => r.getBoundingClientRect());
  for (const t of s.querySelectorAll('text')) { const a = t.getBoundingClientRect();
    for (const r of rects) { const inter = Math.min(a.right, r.right) - Math.max(a.left, r.left) > 1 && Math.min(a.bottom, r.bottom) - Math.max(a.top, r.top) > 1;
      const inside = a.left >= r.left - 1 && a.right <= r.right + 1 && a.top >= r.top - 1 && a.bottom <= r.bottom + 1;
      if (inter && !inside && a.width < r.width * 1.6) out.push(s.id + ': "' + t.textContent.slice(0, 24) + '" crosses a box edge'); } } } return out; }"""

GH = re.compile(r"^https://github\.com/(PX4/PX4-Autopilot/blob/d28a642|iamaisim/ProjectAirSim/blob/c80cff5)/([^#\s]+?)(?:#L(\d+)(?:-L(\d+))?)?$")
banned = re.compile(r"refut", re.I)

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1400, "height": 1000})
    pg.on("pageerror", lambda e: err.append("page error: " + str(e)))
    pg.on("console", lambda m: m.type == "error" and err.append("console error: " + m.text))
    pg.goto(MAP.as_uri())
    pg.wait_for_selector(".blk")
    m = res["map"]
    m["title"] = pg.title()
    m["blocks"] = pg.locator(".blk").count()
    m["figures"] = pg.locator("figure").count()
    m["tables"] = pg.locator("main table").count()
    m["table_tools"] = pg.locator(".tblk .ttools").count()
    if m["tables"] != 2 or m["table_tools"] != 2:
        err.append(f"map: expected 2 tables with 2 table tools, got {m['tables']} / {m['table_tools']}")
    if m["figures"] != 3:
        err.append(f"map: expected 3 figures, got {m['figures']}")
    # --- rows
    ids = pg.evaluate("[...document.querySelectorAll('main table tr[id]')].map(r => r.id)")
    want = [f"row-{i}" for i in range(1, 11)] + [f"row-F{i}" for i in range(1, 5)]
    m["row_ids"] = ids
    if sorted(ids) != sorted(want):
        err.append(f"map: row ids {ids} differ from {want}")
    cells = pg.evaluate("[...document.querySelectorAll('main table tr[id]')].map(r => [r.id, r.children.length, r.innerText.trim().length])")
    for rid, n, ln in cells:
        if n != 7 or ln < 60:
            err.append(f"map: row {rid} has {n} cells and {ln} characters of text")
    allids = pg.evaluate("[...document.querySelectorAll('[id]')].map(e => e.id)")
    if len(allids) != len(set(allids)):
        err.append("map: duplicate ids on the page")
    # --- links
    hrefs = pg.evaluate("[...document.querySelectorAll('main a[href]')].map(a => a.getAttribute('href'))")
    gh = sorted({h for h in hrefs if h.startswith("http")})
    local = sorted({h for h in hrefs if not h.startswith(("http", "#"))})
    m["github_links"] = len(gh)
    m["local_links"] = local
    cache = {}
    for u in gh:
        mm = GH.match(u)
        if not mm:
            err.append(f"map: link not of the required form: {u}")
            continue
        repo, path, a, bb = mm.group(1), mm.group(2), mm.group(3), mm.group(4)
        owner_repo, _, commit = repo.split("/blob/")[0], None, repo.split("/blob/")[1]
        raw = f"https://raw.githubusercontent.com/{owner_repo}/{commit}/{path}"
        if raw not in cache:
            try:
                req = urllib.request.Request(raw, headers={"User-Agent": "Mozilla/5.0"})
                cache[raw] = urllib.request.urlopen(req, timeout=30).read().decode("utf-8", "replace").count("\n") + 1
            except Exception as e:
                cache[raw] = None
                err.append(f"map: file does not exist at that commit: {raw} ({str(e)[:60]})")
        n = cache[raw]
        if n is not None and a:
            hi = int(bb) if bb else int(a)
            if int(a) < 1 or hi > n or int(a) > hi:
                err.append(f"map: line numbers {a}-{bb} outside {path} ({n} lines)")
    m["github_files_checked"] = {k.split("/", 3)[3]: v for k, v in cache.items()}
    for h in local:
        target = (MAP.parent / h.split("#")[0]).resolve()
        if not target.exists():
            err.append(f"map: local link does not resolve: {h}")
    # course links must include the five pages that exist
    for need in ("2.1a_The_four", "2.1b_Forces", "2.2a_Position", "2.2b_Three"):
        if not any(need in h for h in local):
            err.append(f"map: no link to {need}")
    # --- blocks with tables
    bad = pg.evaluate("""() => { const out = []; for (const b of document.querySelectorAll('.blk')) { const n = b.querySelectorAll('table').length; if (!n) continue;
        const how = (b.innerText.match(/How to read this table\\./g) || []).length, figs = b.querySelectorAll('figure').length;
        if (how < n || figs < n) out.push(b.dataset.b + ': tables ' + n + ', how-to ' + how + ', figures ' + figs); } return out; }""")
    if bad:
        err.append(f"map: {bad}")
    order_bad = pg.evaluate("""() => { const out = []; for (const b of document.querySelectorAll('.blk')) { const c = b.querySelector('.c'); const kids = [...c.querySelectorAll('p, .tblk')];
        let seenHow = 0, seenTab = 0; for (const k of kids) { if (k.classList.contains('tblk')) { seenTab++; if (seenHow < seenTab) out.push(b.dataset.b + ' table ' + seenTab + ' has no how-to paragraph before it'); } else if (/How to read this table\\./.test(k.innerText)) seenHow++; } } return out; }""")
    if order_bad:
        err.append(f"map: {order_bad}")
    src = MAP.read_text(encoding="utf-8")
    labs = re.findall(r"<figcaption>Figure ([^\s.]+)\.", src)
    m["caption_labels"] = labs
    if labs != ["1", "2", "3"]:
        err.append(f"map: figure labels {labs}")
    # --- figures: photograph, clipped, overlapping
    for i in range(pg.locator("figure").count()):
        pg.locator("figure").nth(i).screenshot(path=f"out/map_fig{i + 1}.png")
    pg.screenshot(path="out/map_full.png", full_page=True)
    empty = pg.evaluate("[...document.querySelectorAll('figure svg')].filter(s => s.children.length < 5).map(s => s.id)")
    if empty:
        err.append(f"map: empty figures {empty}")
    for name, js in (("labels outside the figure", CLIP_JS), ("overlapping labels", OVERLAP_JS), ("text over a box edge", BOXTEXT_JS)):
        out = pg.evaluate(js)
        m[name] = out
        if out:
            err.append(f"map: {name}: {out}")
    # --- the ten and four rows are drawn in the figures
    fig = pg.evaluate("""() => { const t = id => document.getElementById(id).textContent; return {f1: t('fig1'), f2: t('fig2'), f3: t('fig3')}; }""")
    for i, name in enumerate(["Planner", "Position controller", "Velocity controller", "Attitude controller", "Rate controller", "Mixer", "Motors and rotors", "Physics", "Sensors", "Estimator"], 1):
        if f"{i} {name}" not in re.sub(r"\s+", " ", fig["f2"]):
            err.append(f"map: figure 2 does not draw row {i} {name}")
    for f in ("F1", "F2", "F3", "F4"):
        if f not in fig["f3"]:
            err.append(f"map: figure 3 does not draw row {f}")
    # --- marks and notes
    blk = pg.locator('.blk[data-b="goal"]')
    blk.hover(); blk.locator('button.dot[title="question"]').click()
    blk.locator("text=+ Note").click(); pg.wait_for_timeout(200)
    pg.keyboard.type("sample note on map"); pg.wait_for_timeout(600)
    pg.reload(); pg.wait_for_selector(".blk")
    if pg.locator('.blk[data-b="goal"] textarea').input_value() != "sample note on map":
        err.append("map: note did not survive reload")
    if pg.evaluate("PAGE") != "map":
        err.append("map: PAGE is not 'map'")
    tb = pg.locator(".tblk").nth(1)
    tb.hover(); tb.locator('.ttools button.dot[title="change this"]').click()
    tb.screenshot(path="out/map_table_tools.png")
    # --- anchors: open the page at a row and see it scroll to the row
    pg.goto(MAP.as_uri() + "#row-F4"); pg.wait_for_selector(".blk")
    m["target_row_F4"] = pg.evaluate("document.querySelector('tr:target') && document.querySelector('tr:target').id")
    if m["target_row_F4"] != "row-F4":
        err.append(f"map: #row-F4 does not select the row ({m['target_row_F4']})")
    text = pg.inner_text("main")
    if re.search(r"(?<![\d.])9\.81(?!\d)", text):
        err.append("map: the number 9.81 appears")
    if banned.search(text) or banned.search(src):
        err.append("map: banned word")
    if any("<sub>" in s for s in re.findall(r"<svg.*?</svg>", src, flags=re.S)):
        err.append("map: HTML <sub> inside an svg")
    map_ids = set(allids)

    # ---- the section pages
    for tag, path in SECT.items():
        r = res["sections"][tag] = {}
        sp = path.resolve()
        if not sp.exists():
            err.append(f"{tag}: file missing {path}")
            continue
        pg.goto(sp.as_uri()); pg.wait_for_selector(".blk")
        links = pg.evaluate("[...document.querySelectorAll('main a[href*=\"autopilot_map.html\"]')].map(a => a.getAttribute('href'))")
        r["map_links"] = sorted(set(links))
        for h in links:
            base, _, frag = h.partition("#")
            if not base.endswith("../reference/autopilot_map.html"):
                err.append(f"{tag}: odd map link {h}")
            if frag and frag not in map_ids:
                err.append(f"{tag}: link {h} points to an id that is not on the map page")
        if not links:
            err.append(f"{tag}: no link to the autopilot map")
        n_why = pg.locator('.blk[data-b="why"]').count()
        r["why_blocks"] = n_why
        if n_why != 1 and tag.startswith("2.1"):
            err.append(f"{tag}: {n_why} why blocks, expected exactly 1")
        if n_why == 1:
            w = pg.locator('.blk[data-b="why"]')
            wt = w.inner_text()
            first = pg.evaluate("(() => { const b = document.querySelector('.blk[data-b=\"why\"] .c'); return b.firstElementChild.tagName + ' ' + b.firstElementChild.className; })()")
            r["why_first_child"] = first
            if not wt.startswith("Why you learn this."):
                err.append(f"{tag}: why block does not start with 'Why you learn this.'")
            if "How to read this table." not in wt or "Without this page" not in wt:
                err.append(f"{tag}: why block lacks the how-to paragraph or the 'Without this page' sentence")
            if w.locator("table").count() != 1 or w.locator("figure").count() != 1:
                err.append(f"{tag}: why block must hold 1 table and 1 figure")
            if w.locator(".box").count() != 0:
                err.append(f"{tag}: why block must not use a .box wrapper (2.2a markup)")
            if pg.locator('.blk[data-b="goal"] + .blk[data-b="why"]').count() != 1:
                err.append(f"{tag}: why block is not right after the goal block")
            w.locator("figure").screenshot(path=f"out/why_{tag.replace('.', '_')}_fig.png")
            w.screenshot(path=f"out/why_{tag.replace('.', '_')}_block.png")
        if tag.startswith("2.1"):
            labs = re.findall(r"<figcaption>Figure ([^\s.]+)\.", path.read_text(encoding="utf-8"))
            r["figures_labelled"] = len(labs)
            r["first_labels"] = labs[:3]
            if labs != [str(i + 1) for i in range(len(labs))]:
                err.append(f"{tag}: figure labels do not run 1..{len(labs)} (renumber step not applied?): {labs[:4]}")
            st = path.read_text(encoding="utf-8")
            if re.search(r"Figures? W\b", st):
                err.append(f"{tag}: a 'Figure W' placeholder is left")
        if tag == "2.1b":
            nxt = pg.evaluate("[...document.querySelectorAll('.blk[data-b=\"prev\"] a')].map(a => a.getAttribute('href'))")
            r["prev_links"] = nxt
            if not any("2.2a_" in h for h in nxt):
                err.append("2.1b: 'Next in the course' does not link to 2.2a")
        for name, js in (("labels outside the figure", CLIP_JS), ("overlapping labels", OVERLAP_JS), ("text over a box edge", BOXTEXT_JS)):
            if tag.startswith("2.1"):
                # only the why figure is checked here (the older figures of 2.1 have their own checks)
                out = pg.evaluate(js.replace("document.querySelectorAll('figure svg')", "document.querySelectorAll('#fWhy')"))
                if out:
                    err.append(f"{tag}: why figure, {name}: {out}")
    b.close()

Path("out/check_map.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
print(json.dumps({"errors": err, "map": {k: v for k, v in res["map"].items() if k in ("blocks", "figures", "tables", "github_links", "caption_labels")},
                  "sections": res["sections"]}, ensure_ascii=False))
sys.exit(1 if err else 0)
