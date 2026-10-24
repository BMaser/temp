"""Check the references of the course: page/reference/references.js (the one list), page/reference/references.html (the page
that draws it) and the [n] citations on the seven course pages. Run on the cloud machine only (never on the owner's PC):
    python check_refs.py
Checks:
 1. references.js: every entry has n, key, kind (one of 7), authors, title, used_on; n is unique; the keys are unique;
    "checked" is never empty; a ISBN/DOI/year that is not known is written "?" or left empty (never a guess is tested here).
 2. references.html draws one <li id="ref-n"> for every entry, grouped by kind, numbers ascending inside a group; the search box
    works; the figure (bars by kind and page) is drawn; no label leaves the figure; marks and notes work (PAGE = "refs").
 3. every a.cite on every course page has the form <prefix>references.html#ref-n with the right relative prefix, its text is n, and n is an entry.
 4. every page with a References block (all but plan.html and sim_physics_formulas.html, which has its own small block) lists exactly the
    numbers that the page cites outside the block; the plan has no block, its sources tables carry the numbers in their first column.
 5. used_on of each entry equals the set of pages that cite it (and is never empty).
 6. no block of a section page or of the map names one of the 15 common sources (Leishman, NASA, MIT 16, UIUC, Dellaert, PX4, Gazebo,
    AirSim, DART, WGS84, EGM2008, Beard, Quan, Mahony, GMU) without a [n] in the same block; the plan is checked cell by cell
    (Reuses column, resource column of the gaps table, verdict table, sources table).
 7. the header crumb of 2.1a, 2.1b, 2.2a, 2.2b, references.html, autopilot_map.html and sim_physics_formulas.html starts with the links
    "Course home" (../../index.html, checked by its href text only) and "Course plan" (../plan.html#ov); ../plan.html exists in the tree.
 8. banned word, page errors."""
import json
import re
import sys
from pathlib import Path
from playwright.sync_api import sync_playwright

PAGE = Path("page")
PAGES = {
    "plan": "plan.html",
    "2.1a": "samples/2.1a_The_four_forces_one_at_a_time.html",
    "2.1b": "samples/2.1b_Forces_together_tilt_wind_simulators.html",
    "2.2a": "samples/2.2a_Position_velocity_acceleration_on_a_line.html",
    "2.2b": "samples/2.2b_Three_dimensions_laps_falling_fixed_wing_simulators.html",
    "2.2c": "samples/2.2c_How_PX4_and_Project_AirSim_plan_motion.html",
    "2.2d": "samples/2.2d_Real_scenarios_thinking_problems.html",
    "2.3": "samples/2.3_Dynamics_Newtons_second_law.html",
    "map": "reference/autopilot_map.html",
    "formulas": "reference/sim_physics_formulas.html",
}
PREFIX = {"plan": "reference/references.html", "2.1a": "../reference/references.html", "2.1b": "../reference/references.html",
          "2.2a": "../reference/references.html", "2.2b": "../reference/references.html", "2.2c": "../reference/references.html", "2.2d": "../reference/references.html", "2.3": "../reference/references.html", "map": "references.html", "formulas": "references.html"}
KINDS = ["book", "paper", "course", "web page", "source code", "documentation", "data/standard"]
NAMES = re.compile(r"\b(Leishman|NASA|MIT 16|UIUC|Dellaert|PX4|Gazebo|AirSim|DART|WGS84|EGM2008|Beard|Quan|Mahony|GMU)\b")
HREF = re.compile(r"^(.*)#ref-(\d+)$")
Path("out").mkdir(exist_ok=True)
res = {"errors": [], "pages": {}}
err = res["errors"]

CITES_JS = """() => {
  const blocks = [...document.querySelectorAll('[data-b="refs"]')];
  const inRefs = a => blocks.some(b => b.contains(a));
  const all = [...document.querySelectorAll('a.cite')];
  const pair = a => [a.getAttribute('href'), a.textContent.trim()];
  return {outside: all.filter(a => !inRefs(a)).map(pair), inside: all.filter(a => inRefs(a)).map(pair), n_blocks: blocks.length,
          title: blocks.length ? blocks[0].innerText.trim().slice(0, 30) : ""};
}"""
BLOCKS_JS = """() => {
  const out = [];
  for (const b of document.querySelectorAll('[data-b]')) {
    const id = b.getAttribute('data-b'); if (id === 'refs') continue;
    const c = b.querySelector(':scope > .c') || b;
    const clone = c.cloneNode(true);
    clone.querySelectorAll('svg, script, style, textarea, .note, .tools, .tbar, .tnotes').forEach(e => e.remove());
    out.push({id, text: clone.textContent, cites: c.querySelectorAll('a.cite').length});
  }
  return out;
}"""
PLAN_CELLS_JS = """() => {
  const out = [];
  const sels = ['.trow:not(.tt):not(.gp):not(.src) .re', '.trow.gp .pic', '.trow.src .n', '[data-id="ov-verdict"] td'];
  for (const s of sels) document.querySelectorAll(s).forEach(e => out.push({sel: s, text: e.textContent, cites: e.querySelectorAll('a.cite').length}));
  return out;
}"""
CLIP_JS = """() => { const out = []; for (const s of document.querySelectorAll('figure svg')) { const vb = s.viewBox.baseVal; const root = s.getBoundingClientRect();
  const sc = root.width / vb.width;
  for (const t of s.querySelectorAll('text')) { const r = t.getBoundingClientRect(); const x0 = (r.left - root.left) / sc, x1 = (r.right - root.left) / sc, y0 = (r.top - root.top) / sc, y1 = (r.bottom - root.top) / sc;
    if (x0 < -1 || y0 < -1 || x1 > vb.width + 1 || y1 > vb.height + 1) out.push(s.id + ':' + t.textContent.slice(0, 30)); } } return out; }"""
OVERLAP_JS = """() => { const out = []; for (const s of document.querySelectorAll('figure svg')) { const ts = [...s.querySelectorAll('text')].map(t => ({t, r: t.getBoundingClientRect()}));
  for (let i = 0; i < ts.length; i++) for (let j = i + 1; j < ts.length; j++) { const a = ts[i].r, c = ts[j].r;
    const w = Math.min(a.right, c.right) - Math.max(a.left, c.left), h = Math.min(a.bottom, c.bottom) - Math.max(a.top, c.top);
    if (w > 1 && h > 1 && w * h > 0.15 * Math.min(a.width * a.height, c.width * c.height)) out.push(s.id + ': "' + ts[i].t.textContent.slice(0, 20) + '" / "' + ts[j].t.textContent.slice(0, 20) + '"'); } } return out; }"""

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1400, "height": 1000})
    page_errors = []
    pg.on("pageerror", lambda e: page_errors.append(str(e)))
    pg.on("console", lambda m: m.type == "error" and page_errors.append("console: " + m.text))

    # ---------- 1 + 2: the list and its page
    refs_path = (PAGE / "reference" / "references.html").resolve()
    pg.goto(refs_path.as_uri()); pg.wait_for_selector(".refs li")
    entries = pg.evaluate("window.COURSE_REFS")
    res["n_entries"] = len(entries)
    by_n, keys, seen = {}, set(), []
    for e in entries:
        for f in ("n", "key", "kind", "authors", "title", "used_on", "checked"):
            if e.get(f) in (None, "", []):
                err.append(f"entry {e.get('n')}: field {f} is empty")
        if e.get("kind") not in KINDS:
            err.append(f"entry {e.get('n')}: kind {e.get('kind')!r} is not one of {KINDS}")
        if e.get("n") in by_n:
            err.append(f"number {e.get('n')} is used twice")
        by_n[e.get("n")] = e
        if e.get("key") in keys:
            err.append(f"key {e.get('key')} is used twice")
        keys.add(e.get("key"))
        if not str(e.get("checked", "")).startswith(("fetched 2026-10-20", "not checked", "per plan", "per page")):
            err.append(f"entry {e.get('n')}: 'checked' is {e.get('checked')!r}")
    if sorted(by_n) != list(range(1, len(entries) + 1)):
        err.append(f"numbers are not 1..{len(entries)} without gaps: {sorted(by_n)}")
    lis = pg.evaluate("[...document.querySelectorAll('.refs li')].map(li => [li.id, li.dataset.kind, Number(li.dataset.n)])")
    ids = [x[0] for x in lis]
    if sorted(ids) != sorted(f"ref-{n}" for n in by_n) or len(set(ids)) != len(ids):
        err.append(f"references.html ids differ from the list: {ids}")
    order = [KINDS.index(k) for _, k, _ in lis]
    if order != sorted(order):
        err.append("references.html: entries are not grouped by kind in the order " + ", ".join(KINDS))
    for kind in KINDS:
        ns = [n for _, k, n in lis if k == kind]
        if ns != sorted(ns):
            err.append(f"references.html: numbers of kind {kind} are not ascending: {ns}")
    res["refs_page"] = {"li": len(lis), "kinds": {k: sum(1 for _, kk, _ in lis if kk == k) for k in KINDS}}
    # search
    pg.fill("#q", "Leishman"); pg.wait_for_timeout(200)
    vis = pg.evaluate("[...document.querySelectorAll('.refs li')].filter(li => li.style.display !== 'none').map(li => li.id)")
    if vis != ["ref-1"]:
        err.append(f"references.html: search 'Leishman' shows {vis}, expected ['ref-1']")
    pg.fill("#q", "")
    pg.dispatch_event("#q", "input")
    # every entry has the title and the "Cited on" line; a cite link in the entry resolves
    for e in entries:
        t = pg.inner_text(f"#ref-{e['n']}")
        if e["title"][:20] not in t or "Cited on:" not in t or "Checked:" not in t:
            err.append(f"references.html: entry {e['n']} lacks its title, its 'Checked:' or its 'Cited on:' line")
    # figure
    nrect = pg.evaluate("document.querySelectorAll('#figKinds rect').length")
    if nrect < 10:
        err.append(f"references.html: figure has only {nrect} rectangles")
    for name, js in (("labels outside the figure", CLIP_JS), ("overlapping labels", OVERLAP_JS)):
        out = pg.evaluate(js)
        if out:
            err.append(f"references.html: {name}: {out}")
    pg.locator("#figKinds").screenshot(path="out/refs_fig.png")
    pg.screenshot(path="out/refs_full.png", full_page=True)
    # marks and notes
    blk = pg.locator('.blk[data-b="how"]')
    blk.hover(); blk.locator('button.dot[title="question"]').click()
    blk.locator("text=+ Note").click(); pg.wait_for_timeout(200)
    pg.keyboard.type("sample note on refs"); pg.wait_for_timeout(600)
    pg.reload(); pg.wait_for_selector(".blk")
    if pg.locator('.blk[data-b="how"] textarea').input_value() != "sample note on refs" or pg.evaluate("PAGE") != "refs":
        err.append("references.html: note did not survive reload or PAGE is not 'refs'")
    src = refs_path.read_text(encoding="utf-8") + (PAGE / "reference" / "references.js").read_text(encoding="utf-8")
    if re.search("refut", src, re.I):
        err.append("references: banned word")
    # the links of the 'Cited on' lines
    hrefs = pg.evaluate("[...document.querySelectorAll('.meta2 a')].map(a => a.getAttribute('href'))")
    for h in sorted(set(hrefs)):
        if not (refs_path.parent / h.split('#')[0]).exists():
            err.append(f"references.html: 'Cited on' link does not resolve: {h}")

    # ---------- 3 to 7: the course pages
    cited = {}                                    # n -> set of page ids
    crumb_pages = {"2.1a", "2.1b", "2.2a", "2.2b", "2.2c", "2.2d", "2.3", "map", "formulas", "refs"}
    crumb_files = dict(PAGES, refs="reference/references.html")
    for pid, rel in PAGES.items():
        path = (PAGE / rel).resolve()
        r = res["pages"][pid] = {}
        page_errors.clear()
        pg.goto(path.as_uri()); pg.wait_for_load_state("load"); pg.wait_for_timeout(500)
        info = pg.evaluate(CITES_JS)
        infos = [info]
        if pid == "plan":                         # the plan draws one tab at a time
            cells = []
            for h in ("ov", "p1", "p2", "p3", "p4", "p5"):
                pg.goto(path.as_uri() + "#" + h); pg.reload(); pg.wait_for_selector(".blk"); pg.wait_for_timeout(250)
                infos.append(pg.evaluate(CITES_JS))
                cells += pg.evaluate(PLAN_CELLS_JS)
            for c in cells:
                txt = re.sub(r"\[[^\]]*\]", "", c["text"])
                if NAMES.search(txt) and not c["cites"]:
                    err.append(f"plan: a cell names {sorted(set(NAMES.findall(txt)))} without [n]: {c['text'][:90]!r} ({c['sel']})")
            r["cells_checked"] = len(cells)
        outside = [x for i in infos for x in i["outside"]]
        inside = [x for i in infos for x in i["inside"]]
        ns_out = set()
        for href, text in outside + inside:
            m = HREF.match(href or "")
            if not m or m.group(1) != PREFIX[pid]:
                err.append(f"{pid}: citation link {href!r} should be {PREFIX[pid]}#ref-n")
                continue
            n = int(m.group(2))
            if n not in by_n:
                err.append(f"{pid}: [{n}] is not an entry of the list")
            if text != str(n):
                err.append(f"{pid}: the link text {text!r} differs from its number {n}")
        for href, text in outside:
            m = HREF.match(href or "")
            if m:
                ns_out.add(int(m.group(2)))
        ns_in = set(int(HREF.match(h).group(2)) for h, _ in inside if HREF.match(h or ""))
        r["cites_outside"] = len(outside); r["distinct"] = sorted(ns_out); r["refs_block"] = sorted(ns_in)
        for n in ns_out:
            cited.setdefault(n, set()).add(pid)
        if pid not in ("plan", "formulas"):
            if infos[0]["n_blocks"] != 1 or not infos[0]["title"].startswith("References"):
                err.append(f"{pid}: expected one References block (data-b=refs) titled 'References', found {infos[0]['n_blocks']} ({infos[0]['title']!r})")
            if ns_in != ns_out:
                err.append(f"{pid}: References block {sorted(ns_in)} differs from the numbers cited {sorted(ns_out)}: only in block {sorted(ns_in - ns_out)}, only in text {sorted(ns_out - ns_in)}")
        if pid == "formulas":
            if ns_in != {29, 30, 31, 32, 33, 34} or not {29, 30, 31, 32, 33, 34} <= ns_out:
                err.append(f"formulas: block {sorted(ns_in)}, cited {sorted(ns_out)}")
        # bare names, block by block (section pages and the map)
        if pid in ("2.1a", "2.1b", "2.2a", "2.2b", "2.2c", "2.2d", "2.3", "map"):
            bad = []
            for blk_ in pg.evaluate(BLOCKS_JS):
                txt = re.sub(r"\[[^\]]*\]", "", blk_["text"])
                found = sorted(set(NAMES.findall(txt)))
                if found and not blk_["cites"]:
                    bad.append((blk_["id"], found))
            r["blocks_naming_without_cite"] = bad
            for bid, found in bad:
                err.append(f"{pid}: block {bid} names {found} without a [n]")
        if pid in crumb_pages:
            pass
        r["page_errors"] = list(page_errors)
        for e_ in page_errors:
            err.append(f"{pid}: {e_}")
        txt_src = path.read_text(encoding="utf-8")
        if re.search("refut", txt_src, re.I):
            err.append(f"{pid}: banned word")

    # ---------- used_on = the pages that cite
    for n, e in by_n.items():
        got = sorted(cited.get(n, set())); want = sorted(e.get("used_on", []))
        if got != want:
            err.append(f"used_on of [{n}] {e.get('key')}: list says {want}, pages cite it on {got}")
    res["cited_on"] = {n: sorted(v) for n, v in sorted(cited.items())}

    # ---------- 7: the crumbs
    for pid in crumb_pages:
        path = (PAGE / crumb_files[pid]).resolve()
        pg.goto(path.as_uri()); pg.wait_for_timeout(200)
        links = pg.evaluate("[...document.querySelectorAll('header .crumb a')].map(a => [a.getAttribute('href'), a.textContent.trim()])")
        if len(links) < 2 or links[0] != ["../../index.html", "Course home"] or links[1] != ["../plan.html#ov", "Course plan"]:
            err.append(f"{pid}: the crumb must start with Course home (../../index.html) and Course plan (../plan.html#ov): {links[:3]}")
        if not (path.parent / "../plan.html").resolve().exists():
            err.append(f"{pid}: ../plan.html does not resolve in the tree")
    pg.goto((PAGE / PAGES["2.2a"]).resolve().as_uri()); pg.wait_for_timeout(500)
    pg.screenshot(path="out/page_2_2a_full.png", full_page=True)
    pg.goto((PAGE / PAGES["map"]).resolve().as_uri()); pg.wait_for_timeout(500)
    pg.screenshot(path="out/page_map_full.png", full_page=True)
    b.close()

Path("out/check_refs.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
print(json.dumps({"errors": err, "n_entries": res.get("n_entries"), "refs_page": res.get("refs_page"),
                  "pages": {k: {kk: vv for kk, vv in v.items() if kk in ("cites_outside", "distinct", "refs_block")} for k, v in res["pages"].items()}}, ensure_ascii=False))
sys.exit(1 if err else 0)
