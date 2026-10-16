"""RT15: the real-time camera probe (owner's order 2026-10-16: "reading 15 frames per second is the minimum expectation").

Not a film. One nose camera only (no chase camera), no GUI (the Gazebo SERVER only, gz sim -s, Xvfb + Mesa software drawing), the world of
the films (baylands, day, Fuel trees and rocks close to the path), PX4 SITL with the simulation running free (the world asks for real time
factor 1.0, nothing is paused or stepped), the drone hovers at 2 m for 30 wall-clock seconds, the camera is asked for 30 frames per second
(the PX4 model mono_cam says update_rate 30 and is not changed), at 640 x 480 and at 320 x 240.
Measured: camera frames received by this program per wall second (mean; the slowest 5% of the gaps), the real-time factor, an estimate of the
picture latency, the CPU load. PASS for a size = at least 15 frames per wall second while the real-time factor is at least 0.95.

  python3 tests/px4_gazebo_rt15.py all --out results --px4 px4        # both sizes, each in its own process, writes results/rt15.json
  python3 tests/px4_gazebo_rt15.py size --size 640x480 --out results --px4 px4
"""

import argparse
import asyncio
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "px4_gazebo"))
sys.path.insert(0, str(REPO / "tests"))
import films_core as fc  # noqa: E402
import px4_gazebo_films_run as fr  # noqa: E402

THIS = Path(__file__).resolve()
WALL_S = 30.0
SIZES = [(640, 480), (320, 240)]


def read_cpu():
    """(busy, total) jiffies of all cores from /proc/stat."""
    try:
        v = [int(x) for x in open("/proc/stat").readline().split()[1:]]
        idle = v[3] + (v[4] if len(v) > 4 else 0)
        return sum(v) - idle, sum(v)
    except Exception:  # noqa: BLE001
        return None


async def hover(S, child, cam_log):
    from mavsdk import System
    from mavsdk.offboard import PositionNedYaw, VelocityNedYaw
    d = System()
    await d.connect(system_address="udpin://0.0.0.0:14540")

    async def connected():
        async for st in d.core.connection_state():
            if st.is_connected:
                return
    await asyncio.wait_for(connected(), 300)

    async def healthy():
        async for h in d.telemetry.health():
            if h.is_global_position_ok and h.is_home_position_ok:
                return h
    await asyncio.wait_for(healthy(), fr.HEALTHY_TIMEOUT_S)
    fr.log("healthy")
    cam_log["on"] = True
    await d.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 90.0))
    await d.action.arm()
    t_arm = S.now()
    await d.offboard.start()
    win0 = None
    t_begin = time.monotonic()
    while True:
        await asyncio.sleep(0.02)
        t = S.now()
        tt = t - t_arm
        z = min(fc.CLIMB_Z, fc.CLIMB_RATE * max(0.0, tt))
        vz = fc.CLIMB_RATE if z < fc.CLIMB_Z else 0.0
        await d.offboard.set_position_velocity_ned(PositionNedYaw(0.0, 0.0, -z, 90.0), VelocityNedYaw(0.0, 0.0, -vz, 90.0))
        if win0 is None and z >= fc.CLIMB_Z and tt > 4.0:
            win0 = time.monotonic()
            cam_log.update(window_start_wall=win0, window_start_sim=t, cpu0=read_cpu())
            child["hover_reached_sim_s"] = t
        if win0 is not None and time.monotonic() - win0 >= WALL_S:
            cam_log.update(window_end_wall=time.monotonic(), window_end_sim=t, cpu1=read_cpu())
            break
        if time.monotonic() - t_begin > 20 * 60:
            raise RuntimeError("time-out before the hover window")


def run_size(w: int, h: int, out_root: Path, px4: Path) -> dict:
    import px4_gazebo_base_run as base
    fr.wall_start = time.monotonic()
    name = f"{w}x{h}"
    out = out_root / "rt15" / name
    out.mkdir(parents=True, exist_ok=True)
    child = {"size": name, "probe": "RT15", "world": "baylands, day, Fuel trees and rocks (the films' world)", "chase_camera": False, "gui": False}
    bk = out_root / "_backup"
    wp = px4 / fr.PX4_WORLD
    wp.write_text(fc.patch_world(fr.backup_once(wp, bk), fc.CONDITIONS["day"]))
    mono, x500 = px4 / fr.PX4_MODELS / "mono_cam" / "model.sdf", px4 / fr.PX4_MODELS / "x500_mono_cam" / "model.sdf"
    mono.write_text(fc.mono_cam_text(fr.backup_once(mono, bk), w, h, base.SENSOR_WIDTH_FROM, base.SENSOR_HEIGHT_FROM))
    x500.write_text(fc.x500_model_text(fr.backup_once(x500, bk), "", False))
    S = base.Shared(out)
    cam_log = {"on": False, "frames": []}
    base.sh("rm -f /tmp/.X99-lock", 5)
    procs = base.start_stack(out, px4)
    child["scene_info_after_s"] = procs.get("scene_info_after_s")
    try:
        m, topics = base.wait_for(r"^/world/([^/]+)/clock$", base.STACK_START_TIMEOUT_S)
        world = m.group(1)
        mm, topics = base.wait_for(r"^(/world/%s/model/%s/link/camera_link/sensor/imager/image)$" % (re.escape(world), re.escape(base.MODEL)), 300)
        (out / "gz_topics.txt").write_text(topics)
        if not mm:
            raise RuntimeError("camera topic did not appear")
        base.setup_gz(S, world)
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        stats = []

        def on_stats(msg):
            stats.append((time.monotonic(), msg.sim_time.sec + msg.sim_time.nsec * 1e-9, msg.real_time_factor))
        S.node.subscribe(WorldStatistics, f"/world/{world}/stats", on_stats)

        def on_img(msg):
            wt = time.monotonic()
            if cam_log["on"]:
                cam_log["frames"].append((wt, msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9, msg.width, msg.height, len(msg.data)))
        S.node.subscribe(S.Image, mm.group(1), on_img)
        t0 = time.time()
        while S.pose is None and time.time() - t0 < 300:
            time.sleep(1)
        time.sleep(8)
        asyncio.run(hover(S, child, cam_log))
        a, b = cam_log["window_start_wall"], cam_log["window_end_wall"]
        frames = [f for f in cam_log["frames"] if a <= f[0] <= b]
        wall = b - a
        times = np.array([f[0] for f in frames])
        gaps = np.diff(times)
        child["window_wall_s"] = wall
        child["frames_in_window"] = len(frames)
        child["sizes_seen"] = sorted({(f[2], f[3]) for f in frames})
        child["frames_per_wall_second_mean"] = float((len(frames) - 1) / (times[-1] - times[0])) if len(frames) > 2 else 0.0
        if len(gaps):
            k = max(1, int(np.ceil(0.05 * len(gaps))))
            slow = np.sort(gaps)[-k:]
            child.update(gap_mean_s=float(gaps.mean()), gap_p95_s=float(np.percentile(gaps, 95)), gap_max_s=float(gaps.max()),
                         slowest_5pct_gaps_mean_s=float(slow.mean()), frames_per_wall_second_slowest_5pct=float(1.0 / slow.mean()))
        sim_adv = cam_log["window_end_sim"] - cam_log["window_start_sim"]
        child["sim_seconds_in_window"] = sim_adv
        child["real_time_factor_window"] = sim_adv / wall
        rtfs = [x[2] for x in stats if a <= x[0] <= b]
        child["real_time_factor_gz_stats_mean"] = float(np.mean(rtfs)) if rtfs else None
        if stats and frames:
            sw, ss = np.array([x[0] for x in stats]), np.array([x[1] for x in stats])
            lat = np.array([f[0] - np.interp(f[1], ss, sw) for f in frames if ss[0] <= f[1] <= ss[-1]])
            if len(lat):
                child["latency_s_mean"] = float(lat.mean())
                child["latency_s_p95"] = float(np.percentile(lat, 95))
                child["latency_note"] = ("estimate: wall time of receipt minus the wall time at which the world clock (WorldStatistics messages) reached "
                                         "the picture's time stamp; the clock messages themselves arrive late")
        c0, c1 = cam_log.get("cpu0"), cam_log.get("cpu1")
        if c0 and c1 and c1[1] > c0[1]:
            child["cpu_busy_fraction_all_cores"] = (c1[0] - c0[0]) / (c1[1] - c0[1])
        child["loadavg"] = list(os.getloadavg())
        child["cores"] = os.cpu_count()
        child["pass"] = bool(child["frames_per_wall_second_mean"] >= 15.0 and child["real_time_factor_window"] >= 0.95)
        child["end"] = "measured"
    except Exception:  # noqa: BLE001
        child["traceback"] = traceback.format_exc()
        child["pass"] = False
        child.setdefault("end", "error")
        fr.log(child["traceback"])
    finally:
        try:
            child["gz_server_log_hits"] = fr.grep_sim_log(out / "gz_server.log", 40)
        except Exception:  # noqa: BLE001
            pass
        (out / "rt15_size.json").write_text(json.dumps(child, indent=2, default=str))
        base.stop_stack({k: v for k, v in procs.items() if hasattr(v, "pid")})
        g = procs.get("gz")
        if g is not None:
            try:
                os.killpg(g.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
        time.sleep(5)
    return child


def run_all(out_root: Path, px4: Path):
    core = REPO / "px4_gazebo" / "films_core.py"
    res = {"probe": "RT15", "date": time.strftime("%Y-%m-%d"),
           "rule": "PASS for a size = at least 15 camera frames per wall-clock second (mean) while the real-time factor is at least 0.95",
           "settings": [{"what": "Gazebo server only (gz sim -r -s started by the PX4 make target, HEADLESS=1, no GUI); Xvfb + Mesa software drawing", "where": fr.find_line(REPO / "tests" / "px4_gazebo_base_run.py", "def start_stack")},
                        {"what": "probe code (one nose camera, hover 30 s wall)", "where": fr.find_line(THIS, "def run_size")},
                        {"what": "camera size patch; update_rate 30 of the PX4 model mono_cam is not changed", "where": fr.find_line(core, "def mono_cam_text")},
                        {"what": "world: baylands + Fuel trees and rocks", "where": fr.find_line(core, "SCENERY = [")}],
           "sizes": {}}
    for (w, h) in SIZES:
        d = out_root / "rt15" / f"{w}x{h}"
        d.mkdir(parents=True, exist_ok=True)
        try:
            with open(d / "run.log", "w") as lf:
                subprocess.run([sys.executable, str(THIS), "size", "--size", f"{w}x{h}", "--out", str(out_root), "--px4", str(px4)],
                               stdout=lf, stderr=subprocess.STDOUT, timeout=fr.CONDITION_TIMEOUT_S + 600)
        except subprocess.TimeoutExpired:
            fr.log(f"rt15 {w}x{h} timed out")
        p = d / "rt15_size.json"
        res["sizes"][f"{w}x{h}"] = json.loads(p.read_text()) if p.is_file() else {"end": "no result", "pass": False}
    (out_root / "rt15.json").write_text(json.dumps(res, indent=2, default=str))
    fr.log("rt15: " + json.dumps({k: (v.get("frames_per_wall_second_mean"), v.get("real_time_factor_window"), v.get("pass")) for k, v in res["sizes"].items()}))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("command", choices=["all", "size"])
    ap.add_argument("--size", default="640x480")
    ap.add_argument("--out", required=True)
    ap.add_argument("--px4", default="px4")
    a = ap.parse_args()
    out_root = Path(a.out).resolve()
    out_root.mkdir(parents=True, exist_ok=True)
    if a.command == "size":
        w, h = (int(v) for v in a.size.split("x"))
        run_size(w, h, out_root, Path(a.px4).resolve())
    else:
        run_all(out_root, Path(a.px4).resolve())


if __name__ == "__main__":
    main()
