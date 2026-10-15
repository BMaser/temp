"""Simulator gate, simulator 5: Project AirSim v1.0.1 (iamaisim/ProjectAirSim, Unreal Engine 5) Linux binary + python client.

Sub-commands (python 3.10 with `pip install projectairsim==1.0.1`; see projectairsim_run.sh):
  measure --variant V --world W --sim-config DIR   connects to the running sim (ports 8989/8990), loads
        scene_basic_drone.jsonc, measures physics, takes pictures, writes results/project_airsim/runs/<V>__<W>.json
  merge                                            writes results/project_airsim/gate.json from all runs

Physics: the scene's clock is "steppable" with a 3 ms step. We (a) let it run on its own and compare sim time with wall
time, (b) pause it and call continue_for_n_steps(3000) and time that (steps per wall second as fast as the machine can).
"""

import argparse
import asyncio
import json
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

SIM = "project_airsim"
MEASURE_S = 10.0


def runs_dir() -> Path:
    d = common.results_dir(SIM) / "runs"
    d.mkdir(parents=True, exist_ok=True)
    return d


async def measure(variant: str, world_name: str, sim_config: str) -> int:
    import numpy as np
    from projectairsim import Drone, ProjectAirSimClient, World
    from projectairsim.types import ImageType
    from projectairsim.utils import unpack_image

    res = {"variant": variant, "world": world_name, "connected": False, "traceback": None}
    out = runs_dir()
    client = ProjectAirSimClient()
    try:
        t0 = time.time()
        while True:
            try:
                client.connect()
                break
            except Exception as e:
                if time.time() - t0 > 300:
                    raise RuntimeError(f"no connection within 300 s: {e}")
                time.sleep(3)
        res["connected"] = True
        world = World(client, "scene_basic_drone.jsonc", delay_after_load_sec=2, sim_config_path=sim_config)
        drone = Drone(client, world, "Drone1")
        res["clock_type"] = world.get_sim_clock_type()

        # (a) as shipped: sim time against wall time
        s0, w0 = world.get_sim_time(), time.perf_counter()
        time.sleep(MEASURE_S)
        s1, w1 = world.get_sim_time(), time.perf_counter()
        res["physics_as_shipped"] = {"wall_s": w1 - w0, "sim_s": (s1 - s0) / 1e9, "real_time_factor": (s1 - s0) / 1e9 / (w1 - w0),
                                     "steps_per_s": (s1 - s0) / 3e6 / (w1 - w0)}
        # (b) as fast as possible by stepping
        try:
            world.pause()
            n = 3000
            w0 = time.perf_counter()
            s0 = world.get_sim_time()
            world.continue_for_n_steps(n, True)
            w1 = time.perf_counter()
            s1 = world.get_sim_time()
            res["physics_stepped"] = {"steps": n, "wall_s": w1 - w0, "sim_s": (s1 - s0) / 1e9, "steps_per_s": n / (w1 - w0),
                                      "real_time_factor": (s1 - s0) / 1e9 / (w1 - w0)}
            world.resume()
        except Exception:
            res["physics_stepped_error"] = traceback.format_exc()

        drone.enable_api_control()
        drone.arm()
        try:
            task = await drone.takeoff_async()
            await asyncio.wait_for(task, 60)
            task = await drone.move_by_velocity_async(v_north=0.0, v_east=0.0, v_down=-2.0, duration=3.0)
            await asyncio.wait_for(task, 30)
        except Exception:
            res["flight_error"] = traceback.format_exc()[-600:]

        def grab(sensor):
            images = drone.get_images(sensor, [ImageType.SCENE])
            img = unpack_image(list(images.values())[0])
            a = np.asarray(img)
            if a.ndim == 3 and a.shape[2] >= 3:
                a = a[:, :, :3][:, :, ::-1]  # BGR(A) to RGB
            return np.ascontiguousarray(a)

        stds, pics = {}, []
        for sensor in ("Chase", "DownCamera"):
            for k in range(3 if sensor == "Chase" else 1):
                try:
                    img = grab(sensor)
                    st = common.image_stats(img)
                    stds.setdefault(sensor, []).append(st["std"])
                    pics.append((sensor, img, st))
                    common.save_png(img, out / f"{variant}__{world_name}_{sensor}_{k + 1}.png")
                    if k == 0:
                        res.setdefault("sizes", {})[sensor] = [st["shape"][1], st["shape"][0]]
                except Exception:
                    res.setdefault("grab_errors", []).append(traceback.format_exc()[-500:])
                time.sleep(1.5)
        res["stds"] = stds
        res["camera_ok"] = any(s > common.PASS_MIN_STD for v in stds.values() for s in v)
        chase_ok = any(s > common.PASS_MIN_STD for s in stds.get("Chase", []))
        res["camera_size"] = (res.get("sizes") or {}).get("Chase" if chase_ok else "DownCamera")
        res["camera_sensor_used"] = "Chase" if chase_ok else "DownCamera"

        # camera frames per second (get_images round trips)
        if res["camera_ok"]:
            sensor = res["camera_sensor_used"]
            n, w0 = 0, time.perf_counter()
            while time.perf_counter() - w0 < MEASURE_S:
                grab(sensor)
                n += 1
            wall = time.perf_counter() - w0
            res["camera"] = {"frames": n, "wall_s": wall, "frames_per_s": n / wall}
        try:
            drone.disarm()
            drone.disable_api_control()
        except Exception:
            pass

        # (c) physics as fast as it goes: reload the scene with a copy whose clock steps every 0.1 ms of real time (3 ms of sim time per step)
        try:
            world = World(client, "scene_basic_drone_fast.jsonc", delay_after_load_sec=2, sim_config_path=sim_config)
            time.sleep(1.0)
            s0, w0 = world.get_sim_time(), time.perf_counter()
            time.sleep(MEASURE_S)
            s1, w1 = world.get_sim_time(), time.perf_counter()
            res["physics_fast_clock"] = {"wall_s": w1 - w0, "sim_s": (s1 - s0) / 1e9, "real_time_factor": (s1 - s0) / 1e9 / (w1 - w0),
                                         "steps_per_s": (s1 - s0) / 3e6 / (w1 - w0)}
        except Exception:
            res["physics_fast_error"] = traceback.format_exc()[-800:]
    except Exception:
        res["traceback"] = traceback.format_exc()
    finally:
        try:
            client.disconnect()
        except Exception:
            pass
    (out / f"{variant}__{world_name}.json").write_text(json.dumps(res, indent=2, default=str))
    print(json.dumps(res, indent=2, default=str))
    return 0 if res.get("camera_ok") else 1


def merge() -> int:
    d = common.results_dir(SIM)
    g = common.read_gate(SIM) if (d / "gate.json").is_file() else common.new_gate(SIM)
    runs = [json.loads(p.read_text()) for p in sorted(runs_dir().glob("*.json"))]
    lines = []
    for r in runs:
        cam = r.get("camera") or {}
        ps, pp = r.get("physics_stepped") or {}, r.get("physics_as_shipped") or {}
        lines.append(f"[{r['variant']} / {r['world']}] connected={r['connected']} camera_ok={r.get('camera_ok')} stds={r.get('stds')} "
                     f"rtf_shipped={pp.get('real_time_factor')} stepped_steps/s={ps.get('steps_per_s')} fast_clock_steps/s={(r.get('physics_fast_clock') or {}).get('steps_per_s')} cam_fps={cam.get('frames_per_s')} "
                     f"error={(r.get('traceback') or '').strip().splitlines()[-1][:160] if r.get('traceback') else None}")
    g["headless_ok"] = any(r["connected"] for r in runs) if runs else None
    ok = sorted([r for r in runs if r.get("camera_ok")], key=lambda r: ({"LandscapeMountains": 0, "Blocks": 1}.get(r["world"], 9), r["variant"]))
    if ok:
        b = ok[0]
        g["camera_ok"] = True
        g["camera_size"] = b.get("camera_size")
        g["camera_frames_per_s"] = (b.get("camera") or {}).get("frames_per_s")
        g["physics_steps_per_s"] = (b.get("physics_fast_clock") or b.get("physics_stepped") or {}).get("steps_per_s")
        g["real_time_factor"] = (b.get("physics_as_shipped") or {}).get("real_time_factor")
        g["world_used"] = f"{b['world']} (variant {b['variant']}), scene_basic_drone.jsonc, sensor {b.get('camera_sensor_used')}"
        for i, p in enumerate(sorted(runs_dir().glob(f"{b['variant']}__{b['world']}_{b.get('camera_sensor_used')}_*.png"))[:3], 1):
            (d / f"cam_{i}.png").write_bytes(p.read_bytes())
    else:
        g["camera_ok"] = False if runs else None
        withp = [r for r in runs if r.get("physics_stepped")]
        if withp:
            b = max(withp, key=lambda r: r["physics_stepped"]["steps_per_s"])
            g["physics_steps_per_s"] = (b.get("physics_fast_clock") or b["physics_stepped"])["steps_per_s"]
            g["real_time_factor"] = (b.get("physics_as_shipped") or {}).get("real_time_factor")
            g["world_used"] = f"{b['world']} (variant {b['variant']}); no non-blank picture"
    g["notes"] = ("physics_steps_per_s = sim steps (3 ms) per wall second when the scene clock is set to step every 0.1 ms of real time (scene_basic_drone_fast.jsonc, a copy of the shipped scene with only real-time-update-rate changed), see physics_fast_clock in runs/*.json; the shipped scene and continue_for_n_steps are both paced to ~333 steps/s; "
                  "real_time_factor = sim seconds per wall second when the sim runs on its own; camera_frames_per_s = drone.get_images(Scene) calls per second. Runs: " + " || ".join(lines))
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
    ap.add_argument("--sim-config")
    a = ap.parse_args()
    if a.command == "merge":
        return merge()
    return asyncio.run(measure(a.variant, a.world, a.sim_config))


if __name__ == "__main__":
    sys.exit(main())
