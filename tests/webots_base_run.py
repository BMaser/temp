"""Webots base lap: run Webots on base_lap.wbt (Xvfb + Mesa software drawing), then make the films, the result file and the summary.

  python3 tests/webots_base_run.py run  --out results     # start Webots, wait for it, then do `post`
  python3 tests/webots_base_run.py post --out results     # films, flight_result.json, summary.md, result.json (needs the files Webots wrote)

Needs: Webots R2025a installed (/usr/local/webots), xvfb-run, ffmpeg, python3 with numpy, scipy and Pillow.
The pass rule is applied by tests/webots_base_check.py.
"""

import argparse
import datetime
import json
import os
import signal
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "webots" / "controllers" / "base_lap"))
import flight_result  # noqa: E402
import lap_core as lc  # noqa: E402
import sync_film  # noqa: E402

WORLD = REPO / "webots" / "worlds" / "base_lap.wbt"
WIDTH, HEIGHT, FPS = 640, 480, 25
RUN_TIMEOUT_S = 50 * 60


def run_webots(out: Path) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    for f in ("states.npz", "child.json", "fpv.rgb", "chase.rgb"):
        (out / f).unlink(missing_ok=True)
    env = {**os.environ, "BASE_LAP_REPO": str(REPO), "BASE_LAP_OUT": str(out.resolve()), "LIBGL_ALWAYS_SOFTWARE": "1",
           "WEBOTS_HOME": os.environ.get("WEBOTS_HOME", "/usr/local/webots"), "WEBOTS_VERSION": "R2025a"}
    cmd = ["xvfb-run", "-a", "-s", "-screen 0 1280x1024x24 +extension GLX +render -noreset", "webots", "--mode=fast", "--batch",
           "--stdout", "--stderr", "--minimize", str(WORLD)]
    print("RUN:", " ".join(cmd), flush=True)
    t0 = time.perf_counter()
    status = "timeout"
    with open(out / "webots.log", "w") as lf:
        p = subprocess.Popen(cmd, env=env, stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)
        while time.perf_counter() - t0 < RUN_TIMEOUT_S:
            if p.poll() is not None:
                status = f"webots exited with code {p.returncode}"
                break
            if time.perf_counter() - t0 > 90 and "Skipped unknown" in (out / "webots.log").read_text(errors="ignore"):
                status = "the world did not load (Skipped unknown node or PROTO in the Webots log)"
                break
            if (out / "child.json").is_file():
                status = "controller finished"
                for _ in range(30):  # give Webots a moment to quit by itself
                    if p.poll() is not None:
                        break
                    time.sleep(1.0)
                break
            time.sleep(1.0)
        time.sleep(1.0)
        try:
            os.killpg(p.pid, signal.SIGTERM)  # our own process group (start_new_session)
        except ProcessLookupError:
            pass
        try:
            p.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
    return {"status": status, "wall_s": time.perf_counter() - t0}


def encode_raw(raw: Path, mp4: Path, w: int, h: int) -> int:
    n = raw.stat().st_size // (w * h * 3)
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(FPS), "-i", str(raw),
           "-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)]
    subprocess.run(cmd, check=True)
    return n


def layout_json(path: Path):
    lay = {
        "title": "Base lap, level 0 track",
        "units": "meters; angles in radians in the file, degrees on the sheet",
        "limits_box": {"x": [-3, 3], "y": [-3, 3], "z": [0, 2.5]},
        "level_file_safety_limits": {"pos_limit_low": [-2.5, -1.5, -0.001], "pos_limit_high": [2.5, 1.5, 2.0]},
        "gate_outer_m": 0.72, "gate_opening_m": 0.4,
        "obstacle_pos_z_means": "top of the obstacle, measured from the floor (the pole starts at z = 0)",
        "start": {"pos": [lc.START_XY[0], lc.START_XY[1], lc.START_Z], "yaw_rad": 0.0},
        "gates": [{"pos": [float(x) for x in g[:3]], "yaw_rad": float(g[3]), "shape": "square"} for g in lc.GATES],
        "obstacles": [{"pos": [float(x) for x in o]} for o in lc.POLES],
    }
    path.write_text(json.dumps(lay, indent=1))


def post(out: Path, run_info: dict) -> dict:
    res = {"run": run_info, "errors": []}
    child_p = out / "child.json"
    if not child_p.is_file():
        res["errors"].append("Webots wrote no child.json")
        (out / "post.json").write_text(json.dumps(res, indent=2))
        return res
    child = json.loads(child_p.read_text())
    res["child_traceback"] = child.get("traceback")
    child["controller_name"] = "cascaded lap controller (position, velocity, attitude, body rate, motors)"
    child_p.write_text(json.dumps(child, indent=2))
    npz = out / "states.npz"
    if not npz.is_file():
        res["errors"].append("no states.npz")
        (out / "post.json").write_text(json.dumps(res, indent=2))
        return res
    z = {k: v for k, v in np.load(npz, allow_pickle=False).items()}
    # films
    try:
        fpv_raw, chase_raw = out / "fpv.rgb", out / "chase.rgb"
        n_fpv = encode_raw(fpv_raw, out / "fpv.mp4", WIDTH, HEIGHT) if fpv_raw.is_file() and fpv_raw.stat().st_size else 0
        n_chase = encode_raw(chase_raw, out / "chase.mp4", WIDTH, HEIGHT) if chase_raw.is_file() and chase_raw.stat().st_size else 0
        res["pictures"] = {"fpv": n_fpv, "chase": n_chase, "pic_rows": int(len(z["pic_step"]))}
        if n_fpv and n_fpv == len(z["pic_step"]):
            frames = np.memmap(fpv_raw, dtype=np.uint8, mode="r", shape=(n_fpv, HEIGHT, WIDTH, 3))
            track = {"gates_pos": z["gates_pos"], "gates_quat": z["gates_quat"], "gate_sequence": z["gate_sequence"],
                     "gate_direction": z["gate_direction"], "obstacles_pos": z["obstacles_pos"], "pad_start": tuple(float(v) for v in z["pad_start"]),
                     "pad_end": tuple(float(v) for v in z["pad_end"]), "plan_xy": z["planned_xy"], "total_passages": int(len(z["gate_sequence"]))}
            res["sync_film"] = sync_film.make_sync_film(frames, z, track, out / "fpv_and_map.mp4", FPS)
            del frames
        else:
            res["errors"].append(f"fpv pictures {n_fpv} differ from picture rows {len(z['pic_step'])}: no map film")
        for raw in (fpv_raw, chase_raw):
            raw.unlink(missing_ok=True)
        for name, w, h in (("fpv", WIDTH, HEIGHT), ("chase", WIDTH, HEIGHT), ("fpv_and_map", WIDTH + HEIGHT, HEIGHT)):
            mp4 = out / f"{name}.mp4"
            if mp4.is_file():
                info = sync_film.film_contact_sheet(mp4, w, h, z["pic_time"], out / f"{name}_contact.png")
                res.setdefault("contact", {})[name] = info
    except Exception:  # noqa: BLE001
        res["errors"].append("films: " + traceback.format_exc()[-1500:])
    # result file
    try:
        layout_json(out / "layout.json")
        films = [f"{n}.mp4" for n in ("fpv", "chase", "fpv_and_map") if (out / f"{n}.mp4").is_file()]
        rc = flight_result.main(["--npz", str(npz), "--child", str(child_p), "--layout", str(out / "layout.json"), "--out", str(out / "flight_result.json"),
                                 "--films", *films, "--run-id", "webots_base", "--seed", "0", "--date", datetime.date.today().isoformat(),
                                 "--title", "Base lap, level 0 track"])
        res["flight_result_exit_code"] = rc
    except Exception:  # noqa: BLE001
        res["errors"].append("flight_result: " + traceback.format_exc()[-1500:])
    (out / "post.json").write_text(json.dumps(res, indent=2, default=str))
    write_summary(out, child, res)
    return res


def write_summary(out: Path, child: dict, res: dict) -> None:
    fr = out / "flight_result.json"
    s = json.loads(fr.read_text())["summary"] if fr.is_file() else {}
    L = ["### Webots base lap", ""]
    L.append(f"- Gates passed: **{child.get('passages')} of {child.get('gates_total')}** (end: {child.get('end')})")
    L.append(f"- Lap time: {s.get('lap_time_s')} s (flight time {s.get('flight_time_s')} s)")
    L.append(f"- Largest tilt: {child.get('max_tilt_deg')} deg (flight_result: {s.get('max_tilt_deg')} deg)")
    L.append(f"- Closest distance of the drone center to a gate frame (collision box): {child.get('closest_frame_distance_m')} m, gate {child.get('closest_frame_gate')}, at {child.get('closest_frame_time')} s (the drone body is about 0.05 m in radius)")
    L.append(f"- Steps per second of wall time: {child.get('steps_per_s')} (steps of {child.get('basic_time_step_ms')} ms; {child.get('steps')} steps in {child.get('wall_s')} s with 2 cameras of 640 x 480)")
    L.append(f"- Collisions seen by Webots (contact points, floor only after liftoff): {child.get('collision_count')}; geometric hits within 0.05 m of a gate box or pole: {child.get('geometric_hit_count')}")
    L.append(f"- Closest pole surface: {child.get('closest_pole_surface_distance_m')} m")
    L.append(f"- Passages: {child.get('passage_log')}")
    L.append(f"- Pictures: {res.get('pictures')}")
    L.append(f"- Contact API: {child.get('contact_api')}; errors: {child.get('contact_errors')}")
    L.append(f"- Errors while making the outputs: {res.get('errors')}")
    L.append(f"- Webots run: {res.get('run')}")
    L.append("")
    check = out / "result.json"
    if check.is_file():
        r = json.loads(check.read_text())
        L.append(f"**{'PASS' if r['passed'] else 'FAIL'}**: failed checks: {r['failed']}")
    (out / "summary.md").write_text("\n".join(L) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("command", choices=["run", "post"])
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = Path(a.out).resolve()
    info = run_webots(out) if a.command == "run" else {"status": "post only"}
    post(out, info)


if __name__ == "__main__":
    main()
