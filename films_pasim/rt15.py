"""RT15 probe (not a film): how many nose-camera pictures per WALL-CLOCK second does Project AirSim deliver to our program when the
simulation runs free at real time? One size per call (--size 640x480 or 320x240). Pass = at least 15 frames per wall second with a
real-time factor of at least 0.95 (owner's order, SPEC.md section "Real-time camera probe RT15").

Settings (all declared in rt15.json): scene Blocks, the shipped clock (step 3 ms, real-time-update-rate = step, so real time, not paused,
not stepped), ONE camera (FrontCamera on the nose, a copy of the shipped Chase camera settings with the size and capture-interval
1/30 s; Chase and DownCamera disabled; streaming-enabled false because no video feed is wanted), headless: -RenderOffScreen on Xvfb,
-AllowCPUDevices (software Vulkan, no graphics card on GitHub's free Linux machine).

Phase A (30 s wall): pictures received through the subscription to the camera topic (client.subscribe), counted with perf_counter.
Phase B (15 s wall): the request interface, drone.get_images() in a loop, timed per call (time from asking to receiving).
RTF = simulated seconds per wall second, from world.get_sim_time() at both ends of each phase. CPU load = /proc/stat deltas (all cores).
The drone hovers during both phases.
"""

import argparse
import asyncio
import json
import os
import statistics
import sys
import threading
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pasim_films import ROBOT_SHIPPED, SCENE_SHIPPED, strip_jsonc  # noqa: E402

ROBOT_RT, SCENE_RT = "robot_rt15.jsonc", "scene_rt15.jsonc"


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def cpu_times():
    try:
        v = [int(x) for x in open("/proc/stat").readline().split()[1:]]
        idle = v[3] + (v[4] if len(v) > 4 else 0)
        return sum(v), idle
    except Exception:  # noqa: BLE001
        return None


def cpu_busy(a, b):
    if a is None or b is None or b[0] == a[0]:
        return None
    return round(1.0 - (b[1] - a[1]) / (b[0] - a[0]), 3)


def prepare(sim_config: Path, w: int, h: int) -> dict:
    sim_config = Path(sim_config)
    robot = json.loads(strip_jsonc((sim_config / ROBOT_SHIPPED).read_text()))
    chase = next(s for s in robot["sensors"] if s.get("id") == "Chase")
    nose = json.loads(json.dumps(chase))
    nose["id"] = "FrontCamera"
    nose["origin"] = {"xyz": "0.3 0.0 0.0", "rpy-deg": "0 0 0"}
    nose.pop("gimbal", None)
    nose["capture-interval"] = 1.0 / 30.0                                   # asked for 30 pictures per simulated second
    cs = nose["capture-settings"][0]
    cs["width"], cs["height"], cs["streaming-enabled"] = w, h, False
    for s in robot["sensors"]:
        s["enabled"] = False                                                  # every shipped sensor off
    robot["sensors"].append(nose)
    (sim_config / ROBOT_RT).write_text(json.dumps(robot, indent=2))
    scene = json.loads(strip_jsonc((sim_config / SCENE_SHIPPED).read_text()))
    for actor in scene["actors"]:
        if actor.get("robot-config") == ROBOT_SHIPPED:
            actor["robot-config"] = ROBOT_RT
    scene["clock"]["real-time-update-rate"] = int(scene["clock"]["step-ns"])  # real time (shipped value, stated explicitly)
    scene["clock"]["pause-on-start"] = False
    (sim_config / SCENE_RT).write_text(json.dumps(scene, indent=2))
    return {"size": [w, h], "capture_interval_s": nose["capture-interval"], "streaming_enabled": False, "sensors_disabled": [s["id"] for s in robot["sensors"] if not s["enabled"]],
            "clock": scene["clock"], "written_by": f"films_pasim/rt15.py:{sys._getframe().f_lineno}"}


def gap_stats(ts):
    gaps = sorted(b - a for a, b in zip(ts[:-1], ts[1:]))
    if not gaps:
        return None
    p95 = gaps[min(len(gaps) - 1, int(0.95 * len(gaps)))]
    return {"gaps": len(gaps), "median_gap_s": round(statistics.median(gaps), 4), "p95_gap_s": round(p95, 4), "max_gap_s": round(max(gaps), 4),
            "fps_if_every_gap_were_p95": round(1.0 / p95, 2) if p95 > 0 else None}


async def amain(a) -> int:
    import numpy as np
    from PIL import Image
    from projectairsim import Drone, ProjectAirSimClient, World
    from projectairsim.types import ImageType
    from projectairsim.utils import unpack_image

    w, h = (int(x) for x in a.size.split("x"))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    res = {"probe": "RT15", "size": [w, h], "stage": "start", "traceback": None, "pass_rule": ">= 15 frames per wall second AND real-time factor >= 0.95"}

    def save():
        (out / "rt15_size.json").write_text(json.dumps(res, indent=1, default=str))

    res["config"] = prepare(a.sim_config, w, h)
    client = ProjectAirSimClient()
    t0 = time.time()
    while True:
        try:
            client.connect()
            break
        except Exception as e:  # noqa: BLE001
            if time.time() - t0 > a.connect_wait:
                res["stage"], res["error"] = "no_connection", repr(e)
                save()
                return 4
            time.sleep(3)
    rc = 0
    drone = world = None
    try:
        world = World(client, SCENE_RT, delay_after_load_sec=2, sim_config_path=a.sim_config)
        drone = Drone(client, world, "Drone1")
        drone.enable_api_control()
        drone.arm()
        # first real picture (the first one can be black)
        t1 = time.time()
        first = None
        while time.time() - t1 < 120:
            try:
                img = np.asarray(unpack_image(list(drone.get_images("FrontCamera", [ImageType.SCENE]).values())[0]))
                if float(img.std()) > 3.0:
                    first = img
                    break
            except Exception as e:  # noqa: BLE001
                res.setdefault("warmup_errors", []).append(repr(e)[-150:])
            time.sleep(0.5)
        res["first_picture_after_s"] = round(time.time() - t1, 1)
        if first is None:
            res["stage"] = "no_picture"
            save()
            return 3
        take = await drone.takeoff_async()
        tt = time.time()
        while not take.done() and time.time() - tt < 90:
            await asyncio.sleep(0.5)
        res["takeoff_wall_s"] = round(time.time() - tt, 1)
        n0 = res["takeoff_wall_s"]

        # ---------------- phase A: subscription ----------------
        stamps, sim_stamps, sample = [], [], {}
        lock = threading.Lock()

        def on_image(topic, msg):
            now = time.perf_counter()
            with lock:
                stamps.append(now)
                try:
                    sim_stamps.append(msg["time_stamp"])
                    sample["msg_keys"] = sorted(msg.keys())
                    if len(stamps) == 5:
                        arr = np.frombuffer(msg["data"], dtype="uint8").reshape(msg["height"], msg["width"], 3)
                        sample["arr"] = arr.copy()
                except Exception as e:  # noqa: BLE001
                    sample["error"] = repr(e)[-150:]

        res["stage"] = "phase_A"
        save()
        topic = drone.sensors["FrontCamera"]["scene_camera"]
        sim_a0, c0, w0 = world.get_sim_time(), cpu_times(), time.perf_counter()
        client.subscribe(topic, on_image)
        await asyncio.sleep(a.seconds)
        w1, c1, sim_a1 = time.perf_counter(), cpu_times(), world.get_sim_time()
        try:
            client.unsubscribe(topic)
        except Exception:  # noqa: BLE001
            pass
        with lock:
            ts = [t for t in stamps if w0 <= t <= w1]
            sims = list(sim_stamps)
        wall = w1 - w0
        res["phase_A_subscription"] = {
            "wall_s": round(wall, 2), "frames_received": len(ts), "frames_per_wall_second_mean": round(len(ts) / wall, 3),
            **(gap_stats(ts) or {}), "sim_seconds": round((sim_a1 - sim_a0) / 1e9, 3), "real_time_factor": round((sim_a1 - sim_a0) / 1e9 / wall, 4),
            "frames_per_sim_second": round(len(sims) / max((sims[-1] - sims[0]) / 1e9, 1e-9), 2) if len(sims) > 1 else None,
            "cpu_busy_fraction_all_cores": cpu_busy(c0, c1), "message_keys": sample.get("msg_keys"), "callback_error": sample.get("error"),
            "cores": os.cpu_count(), "loadavg": list(os.getloadavg()) if hasattr(os, "getloadavg") else None}
        if "arr" in sample:
            arr = sample["arr"]
            res["phase_A_subscription"]["sample_picture"] = {"shape": list(arr.shape), "mean": round(float(arr.mean()), 1), "std": round(float(arr.std()), 1)}
            Image.fromarray(np.ascontiguousarray(arr[:, :, ::-1])).save(out / f"rt15_{w}x{h}_sample.png")
        log("phase A", res["phase_A_subscription"])

        # ---------------- phase B: request interface ----------------
        res["stage"] = "phase_B"
        save()
        durs, ends = [], []
        sim_b0, c0, w0 = world.get_sim_time(), cpu_times(), time.perf_counter()
        fails = 0
        while time.perf_counter() - w0 < a.seconds_b:
            ta = time.perf_counter()
            try:
                drone.get_images("FrontCamera", [ImageType.SCENE])
                tb = time.perf_counter()
                durs.append(tb - ta)
                ends.append(tb)
            except Exception:  # noqa: BLE001
                fails += 1
        w1, c1, sim_b1 = time.perf_counter(), cpu_times(), world.get_sim_time()
        wall = w1 - w0
        sd = sorted(durs)
        res["phase_B_request"] = {
            "wall_s": round(wall, 2), "pictures": len(durs), "failures": fails, "pictures_per_wall_second_mean": round(len(durs) / wall, 3),
            "request_to_picture_s": {"median": round(statistics.median(sd), 4), "p95": round(sd[min(len(sd) - 1, int(0.95 * len(sd)))], 4), "max": round(sd[-1], 4)} if sd else None,
            **(gap_stats(ends) or {}), "real_time_factor": round((sim_b1 - sim_b0) / 1e9 / wall, 4), "cpu_busy_fraction_all_cores": cpu_busy(c0, c1)}
        log("phase B", res["phase_B_request"])
        A, B = res["phase_A_subscription"], res["phase_B_request"]
        best_fps = max(A["frames_per_wall_second_mean"], B["pictures_per_wall_second_mean"])
        rtf = min(A["real_time_factor"], B["real_time_factor"])
        res["summary"] = {"best_frames_per_wall_second": best_fps, "lowest_real_time_factor": rtf,
                          "PASS": bool(best_fps >= 15.0 and rtf >= 0.95),
                          "PASS_if_subscription_only": bool(A["frames_per_wall_second_mean"] >= 15.0 and A["real_time_factor"] >= 0.95)}
        res["stage"] = "done"
    except Exception:  # noqa: BLE001
        res["traceback"] = traceback.format_exc()
        res["stage"] = "error"
        log(res["traceback"])
        rc = 1
    finally:
        try:
            drone.disarm()
            drone.disable_api_control()
        except Exception:  # noqa: BLE001
            pass
        try:
            client.disconnect()
        except Exception:  # noqa: BLE001
            pass
        save()
    return rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--sim-config", required=True)
    ap.add_argument("--size", required=True)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--seconds-b", type=float, default=15.0)
    ap.add_argument("--connect-wait", type=float, default=180.0)
    a = ap.parse_args()
    return asyncio.run(amain(a))


if __name__ == "__main__":
    sys.exit(main())
