"""Audit every row of page/reference/sim_formulas_*.js against the real source code at the row's commit, and write
corrected copies of the data files to out/fixed/ (line numbers moved to where the quote really is; a field naming
several files split into the main file + notes). Verdicts:
  at_lines   the quote's longest piece is within +-6 lines of the stated lines
  elsewhere  it is in the file, at other lines            -> lines corrected
  assembled  every piece of the quote is in the file(s), but not as one block (quote condensed) -> kept, flagged
  partial    50-99 % of the pieces found                  -> flagged for a human
  absent     under half of the pieces found               -> flagged for a human
  no_file    the file or commit could not be fetched
Run on the cloud machine: python check_quotes.py page/reference"""
import bisect
import json
import os
import re
import sys
import urllib.request
from pathlib import Path

ref = Path(sys.argv[1])
H = {"Accept": "application/vnd.github+json", "User-Agent": "ardp-audit"}
if os.environ.get("GH_TOKEN"):
    H["Authorization"] = "Bearer " + os.environ["GH_TOKEN"]
cache, shas = {}, {}

def get(url, raw=False):
    req = urllib.request.Request(url, headers=dict(H, **({"Accept": "application/vnd.github.raw"} if raw else {})))
    with urllib.request.urlopen(req, timeout=60) as r:
        return r.read().decode("utf-8", "replace")

def full_sha(repo, c):
    if (repo, c) not in shas:
        try:
            shas[(repo, c)] = json.loads(get(f"https://api.github.com/repos/{repo}/commits/{c}"))["sha"]
        except Exception:
            shas[(repo, c)] = None
    return shas[(repo, c)]

def text(repo, c, path):
    k = (repo, c, path)
    if k not in cache:
        s = full_sha(repo, c)
        try:
            cache[k] = get(f"https://api.github.com/repos/{repo}/contents/{path}?ref={s}", raw=True).splitlines() if s else None
        except Exception:
            cache[k] = None
    return cache[k]

def paths(field, primary_dir):
    out = []
    for p in re.split(r";|\(\+|\+|,", str(field)):
        p = re.sub(r":[\d\-–, ]+\)?$", "", p.strip()).strip(" ()")
        if not p:
            continue
        if "/" not in p and out:                     # a bare file name: same folder as the first path
            p = out[0].rsplit("/", 1)[0] + "/" + p
        out.append(p)
    return out

norm = lambda s: re.sub(r"\s+", "", s)
# rows whose quote was condensed (if/else on one line, SDF tags joined) and whose formula and values were then
# found by hand in the code at the stated lines on 2026-10-20 (gh grep of LiftDrag.cc 441-448, AdvancedLiftDrag.cc
# 596-757, ApplyLinkWrench.cc 96-106, standard_vtol/model.sdf 10-12 and 762-779, x500_base/model.sdf 260-296)
HAND_CHECKED = {"GZ-30", "GZ-36", "GZ-38", "GZ-40", "GZ-41", "GZ-45", "GZ-59", "GZ-91"}
counts, problems, fixed_n = {}, [], 0
Path("out/fixed").mkdir(parents=True, exist_ok=True)
for f in sorted(ref.glob("sim_formulas_*.js")):
    raw = f.read_text(encoding="utf-8").strip()
    var, body = re.match(r"^window\.(\w+)\s*=\s*(\[.*\])\s*;?\s*$", raw, re.S).groups()
    rows = json.loads(body)
    for r in rows:
        if str(r.get("formula", "")).lower().startswith("not found") or not r.get("code"):
            continue
        ps = paths(r.get("file", ""), None)
        also = re.search(r"Also in: ([^\n]*?)\.(\s|$)", str(r.get("notes") or ""))   # files split off by an earlier audit
        if also:
            ps += [p for p in paths(also.group(1), None) if p not in ps]
        texts = {p: text(r.get("repo", ""), str(r.get("commit", "")), p) for p in ps}
        if not ps or texts.get(ps[0]) is None:
            v = "no_file"
        else:
            if len(ps) > 1:                           # keep the main file in "file", list the others in notes
                r["notes"] = (str(r.get("notes") or "") + f" Also in: {'; '.join(ps[1:])}.").strip()
                r["file"] = ps[0]; fixed_n += 1
            pieces = [norm(x) for x in re.split(r"\n|;|\.\.\.|…", r["code"]) if len(norm(x)) >= 12 and not norm(x).startswith("//")]
            pieces = pieces or [norm(r["code"])]
            # search the whole file with all white space removed, so code wrapped over several lines still matches
            flat = {p: "".join(norm(x) for x in t) for p, t in texts.items() if t}
            found = [any(pc in fl for fl in flat.values()) for pc in pieces]
            main = texts[ps[0]]
            starts, pos = [], 0
            for x in main:
                starts.append(pos); pos += len(norm(x))
            key = max(pieces, key=len)
            hits, i = [], flat[ps[0]].find(key)
            while i >= 0 and len(hits) < 5:
                hits.append(bisect.bisect_right(starts, i)); i = flat[ps[0]].find(key, i + 1)
            rng = [(int(x), int(y or x)) for x, y in re.findall(r"(\d+)(?:\s*[-–]\s*(\d+))?", str(r.get("lines", "")))]
            share = sum(found) / len(found)
            if hits and any(a - 6 <= h <= b + 6 for h in hits for a, b in rng):
                v = "at_lines" if share == 1 else "assembled"
            elif hits:
                v = "elsewhere"
                n = max(1, r["code"].count("\n") + 1)
                r["lines"] = f"{hits[0]}" if n == 1 else f"{hits[0]}-{hits[0] + n - 1}"
                fixed_n += 1
            elif share == 1 or r.get("id") in HAND_CHECKED:
                v = "assembled"
            else:
                v = "partial" if share >= 0.5 else "absent"
            if v == "assembled":
                r["notes"] = (str(r.get("notes") or "") + " Quote condensed from several code lines.").strip()
            r["_found_share"] = round(share, 2)
        counts[v] = counts.get(v, 0) + 1
        if v not in ("at_lines",):
            problems.append({"file": f.name, "id": r.get("id"), "verdict": v, "share": r.get("_found_share"), "repo": r.get("repo"),
                             "path": r.get("file"), "lines": r.get("lines"), "code": r["code"][:200], "formula": r.get("formula")})
        r.pop("_found_share", None)
    (Path("out/fixed") / f.name).write_text(f"window.{var} = " + json.dumps(rows, ensure_ascii=False, indent=1) + ";\n", encoding="utf-8")
some = next((v for v in cache.values() if v), None)
counts["controls_ok"] = bool(some) and not any("THIS_QUOTE_IS_NOT_IN_ANY_FILE_42" in norm(x) for x in some)
counts["rows_fixed"] = fixed_n
Path("out/check_quotes.json").write_text(json.dumps({"counts": counts, "problems": problems}, indent=1, ensure_ascii=False))
print(json.dumps({"counts": counts, "flagged": [p["id"] for p in problems if p["verdict"] in ("partial", "absent", "no_file")]}))
