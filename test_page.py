"""Exercise page/plan.html in headless Chromium: render every tab, mark a row, write a quoted note,
reload, and check the note survived. Writes out/result.json and screenshots."""
import json
import os
from pathlib import Path
from playwright.sync_api import sync_playwright

os.makedirs("out", exist_ok=True)
url = Path("page/plan.html").resolve().as_uri()
res = {"errors": []}
with sync_playwright() as p:
    b = p.chromium.launch()
    ctx = b.new_context(viewport={"width": 1500, "height": 1000})
    pg = ctx.new_page()
    pg.on("pageerror", lambda e: res["errors"].append(str(e)))
    pg.on("console", lambda m: m.type == "error" and res["errors"].append(m.text))
    # seed an old-style note (as in his notes.json: "p1-ch3" on the PX4 chapter) to test the id migration
    pg.goto(url)
    pg.evaluate("""() => localStorage.setItem('ardp_v2_plan_notes', JSON.stringify({version:1, notes:{
        'p1-ch3': {label:'Ch 4 title', mark:null, items:[{q:'', t:'OLD NOTE ON PX4', at:'2026-10-20T02:54:17Z'}]}}}))""")
    pg.goto(url + "#p2")
    pg.reload()
    pg.wait_for_timeout(400)
    res["migrated_note_on_px4"] = pg.locator('.blk[data-id="ch3"] textarea').count() == 1 and \
        pg.locator('.blk[data-id="ch3"] textarea').input_value() == "OLD NOTE ON PX4"
    res["px4_chapter_tag"] = pg.locator('.blk[data-id="ch3"] .tag').inner_text()
    res["own_chapter_tag"] = pg.locator('.blk[data-id="chown"] .tag').inner_text()
    res["p2_sources"] = pg.locator(".trow.src").count()
    pg.evaluate("localStorage.clear()")
    pg.goto(url)
    try:
        pg.wait_for_selector(".blk", timeout=8000)
    except Exception as e:
        res["fatal"] = str(e)[:300]
        pg.screenshot(path="out/0_fail.png")
        Path("out/result.json").write_text(json.dumps(res, indent=1))
        raise
    res["secure_context"] = pg.evaluate("isSecureContext")
    res["has_open_picker"] = pg.evaluate("typeof window.showOpenFilePicker")
    res["status_text"] = pg.inner_text("#status")
    res["gap_rows"] = pg.locator(".trow.gp").count()
    pg.screenshot(path="out/1_overview.png", full_page=True)
    res["tabs"] = {}
    for t in ("p1", "p2", "p3", "p4", "p5"):
        pg.goto(url + "#" + t)
        pg.wait_for_timeout(300)
        res["tabs"][t] = {"blocks": pg.locator(".blk").count(), "rows": pg.locator(".trow").count()}
    pg.screenshot(path="out/2_part5.png", full_page=True)
    # mark row 7.3 red and add a note quoting words from it
    pg.goto(url + "#p2")
    pg.wait_for_timeout(300)
    row = pg.locator('.blk[data-id="s7.3"]')
    row.hover()
    row.locator('button.dot[title="change this"]').click()
    # select the words "pose out" inside the row, then + Note
    pg.evaluate("""() => { const c=document.querySelector('.blk[data-id="s7.3"] .c');
        const w=document.createTreeWalker(c, NodeFilter.SHOW_TEXT); let n;
        while((n=w.nextNode())) { const i=n.data.indexOf('pose out'); if(i>=0){ const r=document.createRange();
          r.setStart(n,i); r.setEnd(n,i+8); const s=getSelection(); s.removeAllRanges(); s.addRange(r); return; } } }""")
    row.locator("text=+ Note").click()
    pg.keyboard.type("Test note: show the message order as a sequence diagram.")
    pg.wait_for_timeout(600)
    res["row_class_after_mark"] = row.get_attribute("class")
    res["nav_count_p2"] = pg.inner_text('nav a[data-p="p2"] .count')
    pg.screenshot(path="out/3_part2_marked.png", full_page=False)
    pg.reload()
    pg.wait_for_selector(".blk")
    stored = pg.evaluate("JSON.parse(localStorage.getItem('ardp_v2_plan_notes'))")
    res["stored_note"] = stored["notes"].get("s7.3")
    res["row_class_after_reload"] = pg.locator('.blk[data-id="s7.3"]').get_attribute("class")
    res["textarea_after_reload"] = pg.locator('.blk[data-id="s7.3"] textarea').input_value()
    # propose a chapter in Part I and a section in the flight chapter
    pg.goto(url + "#p1")
    pg.wait_for_timeout(300)
    res["p1_terms_and_practice_rows"] = pg.locator(".trow.tt").count()   # 33 terms + 4 chapters x 4 practice rows = 49 (run 39472913191)
    res["chapter_problem_set_rows_p1"] = pg.locator('[data-id$=".ps"]').count()
    res["math_chapter_tag"] = pg.locator('.blk[data-id="chmath"] .tag').inner_text()
    res["sample_link_on_2_1"] = pg.locator('.blk[data-id="sfly.1"] a.sample').count()
    res["practice_totals"] = pg.locator('[data-id$="-practice"] .lead').all_inner_texts()
    res["flight_chapter_tag"] = pg.locator('.blk[data-id="chfly"] .tag').inner_text()
    pg.click("text=+ Propose a chapter in Part I")
    pg.wait_for_timeout(300)
    pg.keyboard.type("Test chapter: weather for pilots")
    pg.locator(".prop textarea").last.fill("section A\nsection B")
    pg.locator('.blk[data-id="chfly"]').scroll_into_view_if_needed()
    pg.locator("text=+ Propose a section in this chapter").nth(1).click()
    pg.wait_for_timeout(300)
    pg.keyboard.type("Test section: propeller sizes")
    pg.wait_for_timeout(600)
    pg.screenshot(path="out/5_part1_flight.png", full_page=True)
    pg.reload()
    pg.wait_for_selector(".blk")
    st = pg.evaluate("JSON.parse(localStorage.getItem('ardp_v2_plan_notes'))")
    res["proposals"] = st.get("proposals")
    res["props_rendered_after_reload"] = pg.locator(".prop").count()
    res["nav_count_p1"] = pg.inner_text('nav a[data-p="p1"] .count')
    # narrow screen
    pg.set_viewport_size({"width": 420, "height": 900})
    pg.screenshot(path="out/4_narrow.png", full_page=False)
    b.close()
print(json.dumps(res, indent=1))
Path("out/result.json").write_text(json.dumps(res, indent=1))
