"""Pass rule of the Webots base lap. Run: python3 tests/webots_base_check.py --out results   (exit code 1 on failure).

All must hold:
  1. every gate of the route is passed in order (5 passages: gates 1, 2, 3, 4, 2) and the flight ended "finished";
  2. no collision with a gate, a pole or the floor during the lap (Webots contact points; the contact API must have worked);
  3. fpv.mp4, chase.mp4 and fpv_and_map.mp4 exist, and 12 sampled pictures of each have a pixel standard deviation above 5;
  4. flight_result.json is a valid result file (format and version, 16 equal-length sample lists, events, the films beside it,
     no forbidden name) and tests/test_flight_result.py passes.
Writes result.json and puts the verdict at the end of summary.md. Nothing is loosened to make a run pass.
"""

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
import sync_film  # noqa: E402

EXPECTED_SEQUENCE = [0, 1, 2, 3, 1]
STD_MIN = 5.0
FORBIDDEN = re.compile(r"\b(lsy|utias|tum|tu munich|technical university|university|universit\w+|epfl|eth zurich|bitcraze)\b", re.I)
FILMS = {"fpv.mp4": (640, 480), "chase.mp4": (640, 480), "fpv_and_map.mp4": (640 + 480, 480)}


def add(checks, name, ok, detail):
    checks.append({"name": name, "ok": bool(ok), "detail": detail})


def film_stds(path: Path, w: int, h: int):
    _, count = sync_film.read_frames_at(path, w, h, None)
    idx = sync_film.evenly_spaced_indices(count, 12)
    frames, _ = sync_film.read_frames_at(path, w, h, idx)
    return count, [float(np.asarray(frames[i], dtype=np.float64).std()) for i in idx]


def validate_result(path: Path):
    problems = []
    text = path.read_text(encoding="utf-8")
    r = json.loads(text)
    if r.get("format") != "flight_result" or r.get("version") != 1:
        problems.append("format or version wrong")
    keys = list(r.keys())
    if keys != ["format", "version", "source", "layout", "samples", "events", "summary", "gui", "films"]:
        problems.append(f"top-level keys {keys}")
    s = r.get("samples", {})
    lists = {k: v for k, v in s.items() if isinstance(v, list)}
    if len(lists) != 16 or len({len(v) for v in lists.values()}) != 1 or s.get("count") != len(next(iter(lists.values()), [])):
        problems.append("the 16 sample lists are not equal in length or do not match count")
    if s.get("simulator") not in (None,):  # no such key expected
        pass
    if r.get("source", {}).get("simulator") != "webots":
        problems.append("source.simulator is not webots")
    gui = r.get("gui", {})
    if len(gui.get("run", {}).get("profile", [])) != 61 or len(gui.get("path", {}).get("S", [])) != s.get("count"):
        problems.append("gui path or profile has the wrong length")
    for f in r.get("films", []):
        if not (path.parent / f).is_file():
            problems.append(f"film {f} missing")
    if FORBIDDEN.search(text):
        problems.append("forbidden name in the text")
    ev = [e for e in r.get("events", []) if e.get("type") == "gate_passage"]
    if [e["gate"] for e in ev] != [g + 1 for g in EXPECTED_SEQUENCE]:
        problems.append(f"passage events {[e.get('gate') for e in ev]}")
    return problems


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = Path(a.out).resolve()
    checks = []
    cj = out / "child.json"
    child = json.loads(cj.read_text()) if cj.is_file() else None
    if child is None:
        add(checks, "child_json_exists", False, "Webots wrote no child.json")
    else:
        seq = [p["gate"] for p in child.get("passage_log", [])]
        add(checks, "all_gates_passed_in_order", child.get("end") == "finished" and seq == EXPECTED_SEQUENCE and child.get("passages") == len(EXPECTED_SEQUENCE),
            f"end {child.get('end')}; passages {child.get('passages')} of {child.get('gates_total')}; gates in order {seq}; expected {EXPECTED_SEQUENCE}; traceback {child.get('traceback')}")
        add(checks, "no_collision", child.get("collision_count") == 0 and not child.get("contact_errors") and child.get("airborne") is True,
            f"Webots contacts {child.get('collision_count')} {child.get('collisions', [])[:5]}; contact API errors {child.get('contact_errors')}; airborne {child.get('airborne')}; "
            f"(not part of the rule) geometric hits within 0.05 m: {child.get('geometric_hit_count')} {child.get('geometric_hits_within_0p05m', [])[:5]}")
    for name, (w, h) in FILMS.items():
        p = out / name
        if not p.is_file() or p.stat().st_size == 0:
            add(checks, f"film_{name}", False, "missing or empty")
            continue
        try:
            count, stds = film_stds(p, w, h)
            add(checks, f"film_{name}", count > 0 and all(v > STD_MIN for v in stds),
                f"{count} pictures, {p.stat().st_size} bytes; stds of 12 sampled pictures {[round(v, 1) for v in stds]} (each must be above {STD_MIN})")
        except Exception as e:  # noqa: BLE001
            add(checks, f"film_{name}", False, f"could not decode: {e!r}")
    fr = out / "flight_result.json"
    if not fr.is_file():
        add(checks, "flight_result_valid", False, "flight_result.json does not exist")
    else:
        try:
            problems = validate_result(fr)
        except Exception as e:  # noqa: BLE001
            problems = [repr(e)]
        t = subprocess.run([sys.executable, str(REPO / "tests" / "test_flight_result.py")], capture_output=True, text=True)
        add(checks, "flight_result_valid", not problems and t.returncode == 0, f"problems {problems}; tests/test_flight_result.py exit code {t.returncode}: {t.stdout.strip().splitlines()[-1:] }")
    failed = [c["name"] for c in checks if not c["ok"]]
    result = {"passed": not failed and bool(checks), "failed": failed, "checks": checks}
    (out / "result.json").write_text(json.dumps(result, indent=2))
    sm = out / "summary.md"
    text = sm.read_text() if sm.is_file() else ""
    text = re.split(r"\n\*\*(PASS|FAIL)\*\*", text)[0].rstrip("\n") + "\n\n"
    text += f"**{'PASS' if result['passed'] else 'FAIL'}**: failed checks: {failed}\n\n| check | result | detail |\n|---|---|---|\n"
    for c in checks:
        text += f"| {c['name']} | {'pass' if c['ok'] else 'FAIL'} | {str(c['detail']).replace('|', '/')} |\n"
    sm.write_text(text)
    print(("PASS" if result["passed"] else "FAIL") + f": failed checks {failed}")
    sys.exit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
