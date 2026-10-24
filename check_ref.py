"""Check the reference page page/reference/sim_physics_formulas.html in headless Chromium:
every data file must load as valid JavaScript/JSON (no page errors), every row must carry the required fields,
every GitHub link must be well formed, and each tab must render. Writes out/check_ref.json and screenshots.
Run on the cloud machine (never on the owner's PC): python check_ref.py page/reference/sim_physics_formulas.html"""
import json
import re
import sys
from pathlib import Path
from playwright.sync_api import sync_playwright

page_path = Path(sys.argv[1]).resolve()
Path("out").mkdir(exist_ok=True)
REQ = ["id", "group", "quantity", "vehicle", "repo", "commit", "file", "lines", "formula"]
res = {"errors": [], "files": {}, "bad_rows": [], "tabs": {}}
for name in ["gazebo", "airsim", "px4sim", "px4ap"]:          # parse each data file as JSON, independently of the browser
    f = page_path.parent / f"sim_formulas_{name}.js"
    if not f.exists():
        res["files"][name] = "missing"; continue
    txt = f.read_text(encoding="utf-8").strip()
    m = re.match(r"^window\.SIM_FORMULAS_[A-Z0-9]+\s*=\s*(\[.*\])\s*;?\s*$", txt, re.S)
    if not m:
        res["files"][name] = "bad wrapper"; res["errors"].append(f"{name}: bad wrapper"); continue
    try:
        data = json.loads(m.group(1))
    except json.JSONDecodeError as e:
        res["files"][name] = f"JSON error: {e}"; res["errors"].append(f"{name}: {e}"); continue
    nf = sum(1 for r in data if str(r.get("formula", "")).lower().startswith("not found"))
    res["files"][name] = {"rows": len(data), "not_found": nf,
                          "by_vehicle": {v: sum(1 for r in data if str(r.get("vehicle", "")).startswith(v)) for v in ["multi", "fixed", "both"]}}
    for r in data:
        miss = [k for k in REQ if r.get(k) in (None, "")]
        if miss and not str(r.get("formula", "")).lower().startswith("not found"):
            res["bad_rows"].append({"file": name, "id": r.get("id"), "missing": miss})
with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 1500, "height": 1000})
    pg.on("pageerror", lambda e: res["errors"].append(str(e)))
    pg.goto(page_path.as_uri())
    pg.wait_for_selector("#summary table")
    res["summary_text"] = pg.inner_text("#summary")
    for k in ["multirotor", "fixed-wing", "both", "all"]:
        pg.click(f'#tabs button[data-k="{k}"]'); pg.wait_for_timeout(300)
        res["tabs"][k] = {"cards": pg.locator(".card").count(), "groups": pg.locator("h3").count()}
        pg.screenshot(path=f"out/ref_{k}.png", full_page=False)
    links = pg.eval_on_selector_all(".card .src a", "as => as.map(a => a.href)")
    res["links"] = len(links)
    res["bad_links"] = [h for h in links if not re.match(r"^https://github\.com/[^/]+/[^/]+/blob/[0-9a-f]{7,40}/.+", h)][:20]
    pg.fill("#q", "thrust"); pg.wait_for_timeout(300)
    res["search_thrust_cards"] = pg.locator(".card").count()
    b.close()
Path("out").mkdir(exist_ok=True)
Path("out/check_ref.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
print(json.dumps({"errors": res["errors"], "files": res["files"], "bad_rows": len(res["bad_rows"]), "bad_links": len(res["bad_links"]), "tabs": res["tabs"]}, ensure_ascii=False))
sys.exit(1 if res["errors"] or res["bad_rows"] or res["bad_links"] else 0)
