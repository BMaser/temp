"""MOCK TEST of the AirSim flight program and the film code. Not a simulator test.

  python mock_airsim_test.py OUT_DIR
Runs airsim_flight.py against tests/mock_airsim/airsim.py (a stand-in client with a toy drone and drawn pictures), then makes the
three films and contact sheets with unreal_films.py and checks them: 3 films, at least 50 pictures each, picture contrast above 5,
the drone climbs about 3 m, flies the figure eight (farthest point about 6 m from the start), ends near the start, no picture failures.
"""

import json
import math
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import unreal_films  # noqa: E402

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
run = out / "run"
env = {**os.environ, "PYTHONPATH": str(HERE / "mock_airsim")}
r = subprocess.run([sys.executable, str(HERE.parent / "airsim_flight.py"), "--out", str(run), "--fps", "15", "--picture-wait", "10"],
                   env=env, capture_output=True, text=True)
(out / "mock_flight.log").write_text(r.stdout + r.stderr)
print(r.stdout[-1500:], r.stderr[-1500:])
flight = json.loads((run / "flight.json").read_text())
track = json.loads((run / "track.json").read_text())
info = unreal_films.make_films(out, run / "frames", track, out / "ffmpeg.log")
quality = unreal_films.film_quality(out, info)
smp = track["samples"]
p0 = smp[0]
far = max(math.hypot(s["n"] - p0["n"], s["e"] - p0["e"]) for s in smp)
high = max(-(s["d"] - p0["d"]) for s in smp)
end = math.hypot(smp[-1]["n"] - p0["n"], smp[-1]["e"] - p0["e"])
rgb = flight["probe"].get("fpv_top_rows_mean_rgb") or [1, 0, 0]
checks = {
    "exit code 0": r.returncode == 0,
    "stage done": flight.get("stage") == "done",
    "3 films": set(info["films"]) == {"fpv.mp4", "chase.mp4", "fpv_and_map.mp4"},
    "each film at least 50 pictures": all(v["pictures_decoded"] >= 50 for v in info["films"].values()),
    "contrast above 5": all(all(x > 5 for x in q["stds"]) for q in quality.values()),
    "films change from picture to picture": all(all(x > 0.5 for x in q["neighbour_mean_abs_diff"]) for q in quality.values()),
    "three contact sheets": all((out / n).is_file() for n in ("fpv_contact.png", "chase_contact.png", "fpv_and_map_contact.png")),
    "climbs about 3 m": 2.5 < high < 3.6,
    "farthest point 4 to 8 m": 4.0 < far < 8.0,
    "ends within 1.5 m of the start": end < 1.5,
    "no camera failures": flight["timing"]["camera_failures"] == 0,
    "steps = 36 s * 15 + 1": flight["timing"]["steps"] == 36 * 15 + 1,
    "front camera colour check (sky more blue than red)": rgb[2] > rgb[0],
}
res = {"checks": checks, "measured": {"high_m": high, "far_m": far, "end_m": end, "timing": flight["timing"], "film_info": info, "quality": quality}}
(out / "mock_result.json").write_text(json.dumps(res, indent=2, default=str))
for k, v in checks.items():
    print(("PASS " if v else "FAIL ") + k)
print("MOCK TEST", "PASS" if all(checks.values()) else "FAIL", {k: round(v, 2) for k, v in {"high": high, "far": far, "end": end}.items()})
sys.exit(0 if all(checks.values()) else 1)
