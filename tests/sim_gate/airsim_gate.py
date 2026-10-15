"""Simulator gate, simulator 4: Microsoft AirSim (Unreal Engine 4) v1.8.1 Linux binary, python client `airsim`.

Two sub-commands (python 3.9 with the airsim client; see the workflow and airsim_run.sh):
  measure --variant V --world W   connects to the running sim (RPC port 41451), measures physics (sim time against wall
                                  time, from the vehicle-state timestamp), then pictures (simGetImages), writes
                                  results/airsim/runs/<V>__<W>.json and <V>__<W>_pic_<n>.png
  merge                           reads all runs/*.json and writes results/airsim/gate.json

Physics steps/s: AirSim's physics loop period is 3 ms (SimModeWorldBase.h: physics_loop_period_ = 3000000LL, saved in
sources/airsim/src_SimModeWorldBase.h), so steps/s = (simulated seconds per wall second) / 0.003. The sim is started with
ClockSpeed 50 so that the measurement shows what the machine can do, not the real-time cap.
"""

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

PHYSICS_PERIOD_S = 0.003
MEASURE_S = 10.0
WIDTH, HEIGHT = 640, 480


def runs_dir() -> Path:
    d = common.results_dir("airsim") / "runs"
    d.mkdir(parents=True, exist_ok=True)
    return d


def measure(variant: str, world: str) -> int:
    import numpy as np

    res = {"variant": variant, "world": world, "connected": False, "traceback": None}
    out = runs_dir()
    try:
        import airsim

        client = airsim.MultirotorClient()
        t0 = time.time()
        while True:
            try:
                client.confirmConnection()
                break
            except Exception as e:  # not up yet
                if time.time() - t0 > 240:
                    raise RuntimeError(f"no RPC connection within 240 s: {e}")
                time.sleep(3)
        res["connected"] = True
        res["client_version"] = getattr(airsim, "__version__", None)
        res["server_version"] = client.getServerVersion() if hasattr(client, "getServerVersion") else None
        client.enableApiControl(True)

        # 1. physics: simulated time against wall time (camera not used)
        s0 = client.getMultirotorState().timestamp
        w0 = time.perf_counter()
        calls = 0
        while time.perf_counter() - w0 < MEASURE_S:
            client.getMultirotorState()
            calls += 1
        s1 = client.getMultirotorState().timestamp
        wall = time.perf_counter() - w0
        rtf = (s1 - s0) / 1e9 / wall
        res["physics"] = {"wall_s": wall, "sim_s": (s1 - s0) / 1e9, "real_time_factor": rtf,
                          "physics_steps_per_s": rtf / PHYSICS_PERIOD_S, "state_rpc_calls": calls}

        # 2. pictures
        def get(cam="0"):
            r = client.simGetImages([airsim.ImageRequest(cam, airsim.ImageType.Scene, False, False)])[0]
            if r.width == 0 or r.height == 0 or len(r.image_data_uint8) == 0:
                return None
            a = np.frombuffer(r.image_data_uint8, np.uint8)
            ch = a.size // (r.width * r.height)
            a = a.reshape(r.height, r.width, ch)[:, :, :3][:, :, ::-1]  # BGR(A) to RGB
            return np.ascontiguousarray(a)

        pics, stds = [], []
        for n, (z, yaw) in enumerate(((-10.0, 0.0), (-5.0, 2.1), (-20.0, 4.2)), 1):
            client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(0, 0, z), airsim.to_quaternion(0, 0, yaw)), True)
            time.sleep(1.0)
            img = get()
            if img is None:
                stds.append(None)
                continue
            s = common.image_stats(img)
            stds.append(s["std"])
            pics.append((img, s))
            common.save_png(img, out / f"{variant}__{world}_pic_{n}.png")
        res["pictures"] = [{"std": st["std"], "shape": st["shape"]} for _, st in pics]
        res["stds"] = stds
        res["camera_size"] = list(pics[0][1]["shape"][:2][::-1]) if pics else None
        res["camera_ok"] = any(s is not None and s > common.PASS_MIN_STD for s in stds)

        # 3. camera frames per second (RPC round trips with a drawn picture each)
        if pics:
            w0, n = time.perf_counter(), 0
            while time.perf_counter() - w0 < MEASURE_S:
                if get() is not None:
                    n += 1
            res["camera"] = {"frames": n, "wall_s": time.perf_counter() - w0, "frames_per_s": n / (time.perf_counter() - w0)}
            # physics again with pictures being requested is not separated; the RTF during the picture loop:
            s2 = client.getMultirotorState().timestamp
            res["camera"]["sim_s_during_all_three_steps"] = (s2 - s0) / 1e9
    except Exception:
        res["traceback"] = traceback.format_exc()
    (out / f"{variant}__{world}.json").write_text(json.dumps(res, indent=2, default=str))
    print(json.dumps(res, indent=2, default=str))
    return 0 if res.get("camera_ok") else 1


def merge() -> int:
    d = common.results_dir("airsim")
    g = common.read_gate("airsim") if (d / "gate.json").is_file() else common.new_gate("airsim")
    runs = [json.loads(p.read_text()) for p in sorted(runs_dir().glob("*.json"))]
    note_lines = []
    for r in runs:
        ph, cam = r.get("physics") or {}, r.get("camera") or {}
        note_lines.append(
            f"[{r['variant']} / {r['world']}] connected={r['connected']} camera_ok={r.get('camera_ok')} stds={r.get('stds')} "
            f"rtf={ph.get('real_time_factor')} steps/s={ph.get('physics_steps_per_s')} cam_fps={cam.get('frames_per_s')} "
            f"error={(r.get('traceback') or '').strip().splitlines()[-1][:160] if r.get('traceback') else None}")
    g["headless_ok"] = any(r["connected"] for r in runs) if runs else None
    ok = [r for r in runs if r.get("camera_ok")]
    world_rank = {"Africa_Savannah": 0, "LandscapeMountains": 1, "Blocks": 2}
    ok.sort(key=lambda r: (world_rank.get(r["world"], 9), r["variant"]))
    nulls = [r for r in runs if r["variant"] == "nullrhi" and r.get("physics")]
    if ok:
        b = ok[0]
        g["camera_ok"] = True
        g["camera_size"] = b.get("camera_size")
        g["camera_frames_per_s"] = (b.get("camera") or {}).get("frames_per_s")
        g["physics_steps_per_s"] = b["physics"]["physics_steps_per_s"]
        g["real_time_factor"] = b["physics"]["real_time_factor"]
        g["world_used"] = f"{b['world']} (variant {b['variant']})"
        for i, p in enumerate(sorted(runs_dir().glob(f"{b['variant']}__{b['world']}_pic_*.png"))[:3], 1):
            (d / f"cam_{i}.png").write_bytes(p.read_bytes())
    else:
        g["camera_ok"] = False if runs else None
        withphys = [r for r in runs if r.get("physics")]
        if withphys:
            b = max(withphys, key=lambda r: r["physics"]["physics_steps_per_s"])
            g["physics_steps_per_s"] = b["physics"]["physics_steps_per_s"]
            g["real_time_factor"] = b["physics"]["real_time_factor"]
            g["world_used"] = f"{b['world']} (variant {b['variant']}); no picture was non-blank"
    if nulls:
        n = max(nulls, key=lambda r: r["physics"]["physics_steps_per_s"])
        note_lines.append(f"-nullrhi run ({n['world']}): rtf {n['physics']['real_time_factor']:.2f}, physics steps/s {n['physics']['physics_steps_per_s']:.0f}")
    g["notes"] = ("physics_steps_per_s = (sim seconds per wall second, from the vehicle-state timestamp, sim started with ClockSpeed 50) / 0.003 s "
                  "(AirSim's fixed physics loop period). camera_frames_per_s = simGetImages(Scene, 640x480, uncompressed) calls per second over RPC. Runs: " + " || ".join(note_lines))
    if not g["camera_ok"]:
        g["log_excerpt"] = g.get("log_excerpt") or common.excerpt(d / "log.txt")
    common.write_gate(g)
    print(json.dumps(g, indent=2, default=str))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("command", choices=["measure", "merge"])
    ap.add_argument("--variant")
    ap.add_argument("--world")
    a = ap.parse_args()
    return measure(a.variant, a.world) if a.command == "measure" else merge()


if __name__ == "__main__":
    sys.exit(main())
