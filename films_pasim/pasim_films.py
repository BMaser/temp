"""Feature films of Project AirSim 1.0.1 (client side). Runs in a Python environment that has the `projectairsim` package.

One call makes the pictures of ONE film (F05 night, F06 clouds and rain in slice 1). The Unreal server is started by launch_film.sh.
The film itself is made by make_film.py from the pictures and from capture.json that this program writes.

How the smooth film in simulated time is made: the simulation clock runs slower than real time (real-time-update-rate = step / rtf),
because the software renderer delivers only one picture every few seconds. The simulation is NOT paused (a paused clock made
GetImages time out in an earlier test). Every film frame is one pair of pictures (drone camera + chase camera) with the simulated
time read right after it; the film plays the pairs in order, so the film time follows the simulated time step by step.
capture.json holds the simulated time of every pair, so the spacing can be checked.

Exit codes: 0 = pictures taken, 3 = no picture, 4 = no connection, 1 = error.
"""

import argparse
import asyncio
import json
import math
import re
import sys
import time
import traceback
from pathlib import Path

SCENE_SHIPPED, SCENE_FILMS = "scene_basic_drone.jsonc", "scene_films.jsonc"
ROBOT_SHIPPED, ROBOT_FILMS = "robot_quadrotor_fastphysics.jsonc", "robot_quadrotor_fastphysics_films.jsonc"
DAY = "2021-09-20"  # the date used by the shipped time_of_day.py example

# (label, caption) in the order they are filmed. The setup of each is in apply_condition().
FILM_DEFS = {
    "F05": {"short": "night", "title": "Night: day, dusk and night at the same place",
            "conditions": [("day", "12:00 (noon)"), ("dusk", "18:45 (dusk)"), ("night", "23:30 (night)")]},
    "F06": {"short": "clouds_rain", "title": "Clouds, fog, rain and snow",
            "conditions": [("clear", "no weather"), ("fog", "fog 0.6"), ("rain", "rain 0.9"), ("snow", "snow 0.9"),
                           ("overcast_try", "cloud shadows 1.0 + sun 0.5")]},
}
TIMES = {"day": f"{DAY} 12:00:00", "dusk": f"{DAY} 18:45:00", "night": f"{DAY} 23:30:00"}


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def strip_jsonc(text: str) -> str:
    out, i, n, in_str = [], 0, len(text), False
    while i < n:
        c = text[i]
        if in_str:
            out.append(c)
            if c == "\\":
                out.append(text[i + 1])
                i += 1
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True
            out.append(c)
        elif text.startswith("//", i):
            while i < n and text[i] != "\n":
                i += 1
            continue
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            i = n if j < 0 else j + 1
        else:
            out.append(c)
        i += 1
    return re.sub(r",(\s*[}\]])", r"\1", "".join(out))


def prepare_configs(sim_config: Path, rtf: float, width: int, height: int, interval: float, chase_close: bool) -> dict:
    """Copy the shipped scene and robot, add a FrontCamera, set picture sizes and the slow clock. Returns the facts."""
    sim_config = Path(sim_config)
    robot = json.loads(strip_jsonc((sim_config / ROBOT_SHIPPED).read_text()))
    chase = next(s for s in robot["sensors"] if s.get("id") == "Chase")
    facts = {"chase_size_before": [chase["capture-settings"][0].get("width"), chase["capture-settings"][0].get("height")],
             "chase_origin_before": dict(chase.get("origin") or {})}
    chase["capture-settings"][0]["width"], chase["capture-settings"][0]["height"] = width, height
    chase["capture-interval"] = interval
    if chase_close:  # 3.5 m behind, 1.5 m above, pitched down (same sign as the shipped -11.46)
        chase["origin"] = {"xyz": "-3.5 0.0 -1.5", "rpy-deg": "0 -23.0 0"}
    facts["chase_origin_after"] = dict(chase["origin"])
    front = json.loads(json.dumps(chase))
    front["id"] = "FrontCamera"
    front["origin"] = {"xyz": "0.3 0.0 0.0", "rpy-deg": "0 0 0"}
    front.pop("gimbal", None)
    robot["sensors"].append(front)
    (sim_config / ROBOT_FILMS).write_text(json.dumps(robot, indent=2))
    scene = json.loads(strip_jsonc((sim_config / SCENE_SHIPPED).read_text()))
    for actor in scene["actors"]:
        if actor.get("robot-config") == ROBOT_SHIPPED:
            actor["robot-config"] = ROBOT_FILMS
    step_ns = int(scene["clock"]["step-ns"])
    scene["clock"]["real-time-update-rate"] = int(round(step_ns / rtf))
    scene["clock"]["pause-on-start"] = False
    (sim_config / SCENE_FILMS).write_text(json.dumps(scene, indent=2))
    facts.update({"step_ns": step_ns, "real_time_update_rate_ns": scene["clock"]["real-time-update-rate"], "rtf_wanted": rtf,
                  "picture_size": [width, height], "capture_interval_sim_s": interval,
                  "drone_start": next((a.get("origin") for a in scene["actors"]), None)})
    return facts


def _get(obj, *names):
    for nme in names:
        if isinstance(obj, dict) and nme in obj:
            return obj[nme]
        if hasattr(obj, nme):
            return getattr(obj, nme)
    raise KeyError(names)


def pose_nedyaw(pose):
    t = _get(pose, "translation", "position")
    q = _get(pose, "rotation", "orientation")
    n, e, d = (float(_get(t, k)) for k in ("x", "y", "z"))
    w, x, y, z = (float(_get(q, k)) for k in ("w", "x", "y", "z"))
    return n, e, d, math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Calls:
    """Every simulator API call goes through here, so that capture.json records its name, arguments, result and the line it was called from."""

    def __init__(self):
        self.records = []

    def __call__(self, name, fn, *args, **kw):
        line = sys._getframe(1).f_lineno
        rec = {"api": name, "args": [repr(a) for a in args] + [f"{k}={v!r}" for k, v in kw.items()],
               "where": f"films_pasim/pasim_films.py:{line}", "t_wall": round(time.time(), 1)}
        try:
            rec["result"] = repr(fn(*args, **kw))[:200]
            rec["ok"] = True
        except Exception as e:  # noqa: BLE001
            rec["result"] = f"{type(e).__name__}: {e}"[:300]
            rec["ok"] = False
        self.records.append(rec)
        log("API", name, rec["args"], "->", rec["result"])
        return rec


async def amain(a) -> int:
    import numpy as np
    from PIL import Image
    from projectairsim import Drone, ProjectAirSimClient, World
    from projectairsim.drone import YawControlMode
    from projectairsim.types import ImageType, WeatherParameter
    from projectairsim.utils import unpack_image

    film = FILM_DEFS[a.film]
    out = Path(a.out)
    frames_dir = out / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    cap = {"film": a.film, "stage": "start", "conditions": [c[0] for c in film["conditions"]], "frames": [], "traceback": None,
           "args": vars(a)}

    def save():
        cap["api_calls"] = call.records
        (out / "capture.json").write_text(json.dumps(cap, indent=1, default=str))

    call = Calls()
    cap["config_facts"] = prepare_configs(a.sim_config, a.rtf, a.width, a.height, a.interval, not a.shipped_chase)
    client = ProjectAirSimClient()
    t0 = time.time()
    while True:
        try:
            client.connect()
            break
        except Exception as e:  # noqa: BLE001
            if time.time() - t0 > a.connect_wait:
                cap["stage"], cap["error"] = "no_connection", repr(e)
                save()
                return 4
            time.sleep(3)
    cap["connected_after_s"] = round(time.time() - t0, 1)
    log("connected after", cap["connected_after_s"], "s")

    def grab(drone, sensor):
        images = drone.get_images(sensor, [ImageType.SCENE])
        arr = np.asarray(unpack_image(list(images.values())[0]))
        if arr.ndim == 3 and arr.shape[2] >= 3:
            arr = arr[:, :, :3][:, :, ::-1]  # BGR -> RGB
        return np.ascontiguousarray(arr)

    rc = 0
    world = drone = None
    try:
        t_load = time.time()
        for scene in (SCENE_FILMS, SCENE_SHIPPED):
            try:
                world = World(client, scene, delay_after_load_sec=2, sim_config_path=a.sim_config)
                drone = Drone(client, world, "Drone1")
                cap["scene_used"] = scene
                break
            except Exception as e:  # noqa: BLE001
                log("scene", scene, "did not load:", repr(e)[:300])
                cap.setdefault("scene_errors", []).append(f"{scene}: {e!r}"[:400])
        if drone is None:
            cap["stage"] = "no_scene"
            save()
            return 1
        cap["scene_load_s"] = round(time.time() - t_load, 1)
        drone.enable_api_control()
        drone.arm()
        call("world.enable_weather_visual_effects", world.enable_weather_visual_effects)
        if a.film == "F05":
            call("world.set_time_of_day(day)", world.set_time_of_day, status=True, datetime=TIMES["day"], is_dst=False,
                 clock_speed=1.0, update_interval=1.0, move_sun=False)

        # ---- warm-up: the first Chase picture is black (measured); wait for a real picture of each camera ----
        cap["stage"] = "warmup"
        save()
        got, errors = {}, []
        t1 = time.time()
        while time.time() - t1 < a.picture_wait:
            for sensor in ("Chase", "FrontCamera"):
                if sensor in got:
                    continue
                try:
                    img = grab(drone, sensor)
                    if float(img.std()) > 3.0:  # a black or flat picture is thrown away
                        got[sensor] = img
                    else:
                        errors.append(f"{sensor}: flat picture (std {float(img.std()):.1f}) thrown away at {time.time() - t1:.0f} s")
                except Exception as e:  # noqa: BLE001
                    errors.append(f"{sensor}: {e!r}"[-200:])
            if len(got) == 2:
                break
            time.sleep(1)
        cap["warmup"] = {"got": sorted(got), "seconds": round(time.time() - t1, 1), "notes": errors[-6:]}
        log("warm-up:", cap["warmup"])
        if "Chase" not in got and "FrontCamera" not in got:
            cap["stage"] = "no_picture"
            save()
            return 3
        cams = {"fpv": "FrontCamera" if "FrontCamera" in got else None, "chase": "Chase" if "Chase" in got else None}

        # ---- takeoff (pictures are asked for meanwhile so that the renderer stays awake) ----
        cap["stage"] = "takeoff"
        take = await drone.takeoff_async()
        sim0 = world.get_sim_time()
        wall0 = time.perf_counter()
        t_take = time.perf_counter()
        while not take.done():
            if (world.get_sim_time() - sim0) / 1e9 > a.takeoff_sim_max or time.perf_counter() - t_take > a.takeoff_wall_max:
                cap["takeoff_note"] = "takeoff did not finish in time; continued anyway"
                break
            await asyncio.sleep(1.0)
        n0, e0, d0, yaw0 = pose_nedyaw(drone.get_ground_truth_pose())
        cap["takeoff"] = {"wall_s": round(time.perf_counter() - t_take, 1), "sim_s": round((world.get_sim_time() - sim0) / 1e9, 2),
                          "pose_ned_yaw": [n0, e0, d0, yaw0]}
        log("takeoff:", cap["takeoff"])

        # ---- the film: for each condition, N pairs of pictures while the drone hovers and swings its nose +-A degrees ----
        cap["stage"] = "film"
        save()
        sim_start = world.get_sim_time()
        wall_start = time.perf_counter()
        idx = 0
        fails = {"fpv": 0, "chase": 0}
        for ci, (label, caption) in enumerate(film["conditions"]):
            apply_condition(call, world, WeatherParameter, a.film, label)
            seg = {"label": label, "caption": caption, "first_frame": idx, "wall_start_s": round(time.perf_counter() - wall_start, 1),
                   "sim_start_s": round((world.get_sim_time() - sim_start) / 1e9, 3)}
            for k in range(a.settle_frames + a.frames):
                if time.perf_counter() - wall_start > a.max_wall:
                    cap["stop_reason"] = f"wall time limit reached in condition {label} at pair {k}"
                    break
                yaw = yaw0 + math.radians(a.yaw_amp_deg) * math.sin(2 * math.pi * k / max(1, a.frames))
                await drone.move_by_velocity_z_async(0.0, 0.0, d0, 2.0, YawControlMode.MaxDegreeOfFreedom, False, yaw)
                rec = {"i": idx, "cond": label, "k": k, "settle": k < a.settle_frames, "fpv": None, "chase": None}
                t_a = time.perf_counter()
                for cam in ("fpv", "chase"):
                    sensor = cams[cam]
                    if sensor is None:
                        continue
                    try:
                        img = grab(drone, sensor)
                    except Exception as ex:  # noqa: BLE001
                        fails[cam] += 1
                        if fails[cam] <= 5:
                            log("picture failed", sensor, repr(ex)[-160:])
                        continue
                    if not rec["settle"]:
                        name = f"{cam}_{idx:05d}.jpg"
                        Image.fromarray(img).save(frames_dir / name, quality=92)
                        rec[cam] = name
                    rec[f"{cam}_mean"], rec[f"{cam}_std"] = round(float(img.mean()), 2), round(float(img.std()), 2)
                t_sim = (world.get_sim_time() - sim_start) / 1e9
                n, e, d, yw = pose_nedyaw(drone.get_ground_truth_pose())
                rec.update({"t": t_sim, "n": n, "e": e, "d": d, "yaw": yw, "wall_s": round(time.perf_counter() - wall_start, 2),
                            "grab_s": round(time.perf_counter() - t_a, 2)})
                cap["frames"].append(rec)
                idx += 1
                if idx % 10 == 0:
                    log(f"{label} pair {k}/{a.settle_frames + a.frames} sim t={t_sim:.1f}s wall={rec['wall_s']:.0f}s failures {fails}")
                    save()
            seg.update({"last_frame": idx - 1, "wall_end_s": round(time.perf_counter() - wall_start, 1),
                        "sim_end_s": round((world.get_sim_time() - sim_start) / 1e9, 3)})
            cap.setdefault("segments", []).append(seg)
            if cap.get("stop_reason"):
                break
        cap["failures"] = fails
        cap["timing"] = {"wall_s": round(time.perf_counter() - wall_start, 1), "sim_s": round((world.get_sim_time() - sim_start) / 1e9, 3)}
        cap["timing"]["real_time_factor"] = round(cap["timing"]["sim_s"] / cap["timing"]["wall_s"], 4) if cap["timing"]["wall_s"] else None
        cap["stage"] = "done"
    except Exception:  # noqa: BLE001
        cap["traceback"] = traceback.format_exc()
        cap["stage"] = "error"
        log(cap["traceback"])
        rc = 1
    finally:
        try:
            if a.film == "F06" and world is not None:
                world.disable_weather_visual_effects()
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


def apply_condition(call, world, WP, film, label):
    """The simulator calls that make one condition. Each goes through `call` so that name, arguments, result and line are recorded."""
    if film == "F05":
        call(f"world.set_time_of_day({label})", world.set_time_of_day, status=True, datetime=TIMES[label], is_dst=False,
             clock_speed=1.0, update_interval=1.0, move_sun=False)
        return
    # F06
    call("world.reset_weather_effects", world.reset_weather_effects)
    call("world.enable_weather_visual_effects", world.enable_weather_visual_effects)
    if label == "clear":
        call("world.set_cloud_shadow_strength(0.0)", world.set_cloud_shadow_strength, 0.0)
    elif label == "fog":
        call("world.set_weather_visual_effects_param(FOG, 0.6)", world.set_weather_visual_effects_param, param=WP.FOG, value=0.6)
    elif label == "rain":
        call("world.set_weather_visual_effects_param(RAIN, 0.9)", world.set_weather_visual_effects_param, param=WP.RAIN, value=0.9)
    elif label == "snow":
        call("world.set_weather_visual_effects_param(SNOW, 0.9)", world.set_weather_visual_effects_param, param=WP.SNOW, value=0.9)
    elif label == "overcast_try":
        call("world.set_cloud_shadow_strength(1.0)", world.set_cloud_shadow_strength, 1.0)
        call("world.set_sunlight_intensity(0.5)", world.set_sunlight_intensity, 0.5)
    call("world.get_weather_visual_effects_param", world.get_weather_visual_effects_param)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--film", required=True, choices=sorted(FILM_DEFS))
    ap.add_argument("--out", required=True)
    ap.add_argument("--sim-config", required=True)
    ap.add_argument("--rtf", type=float, default=0.03)
    ap.add_argument("--frames", type=int, default=40, help="pairs of pictures per condition")
    ap.add_argument("--settle-frames", type=int, default=1, help="pairs thrown away after a condition changes")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--interval", type=float, default=0.05, help="camera capture interval in simulated seconds")
    ap.add_argument("--yaw-amp-deg", type=float, default=40.0)
    ap.add_argument("--shipped-chase", action="store_true", help="keep the shipped Chase camera position (10 m behind)")
    ap.add_argument("--connect-wait", type=float, default=180.0)
    ap.add_argument("--picture-wait", type=float, default=120.0)
    ap.add_argument("--takeoff-sim-max", type=float, default=30.0)
    ap.add_argument("--takeoff-wall-max", type=float, default=600.0)
    ap.add_argument("--max-wall", type=float, default=2700.0)
    a = ap.parse_args()
    return asyncio.run(amain(a))


if __name__ == "__main__":
    sys.exit(main())
