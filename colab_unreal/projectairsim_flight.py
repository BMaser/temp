"""Project AirSim flight program (runs in the Python 3.10 environment that has the `projectairsim` package).

prepare_configs() (called by the notebook, needs no packages) copies the shipped scene and robot files and adds a front camera
("FrontCamera", on the nose, 640 x 480) and sets the clock so that the simulation runs slower than real time (RTF = `rtf`).

main(): connects, loads the scene, checks that a picture arrives, flies the route of route.py and saves pictures and track.json.
The clock runs on its own (slowed down); each pass of the loop takes the pictures, then reads the time and the pose, then sends
the next velocity command. The film is later made at a steady rate from the saved pictures by their simulated time (unreal_films.py).

Exit codes: 0 = flight done, 3 = no picture within the wait, 4 = no connection, 1 = error during the flight.
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
import route  # noqa: E402

SCENE_FILMS = "scene_films.jsonc"
SCENE_SHIPPED = "scene_basic_drone.jsonc"
ROBOT_FILMS = "robot_quadrotor_fastphysics_films.jsonc"
ROBOT_SHIPPED = "robot_quadrotor_fastphysics.jsonc"


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def strip_jsonc(text: str) -> str:
    """Remove // and /* */ comments outside of strings, and trailing commas."""
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


def prepare_configs(sim_config: Path, rtf: float) -> dict:
    """Write scene_films.jsonc and robot_quadrotor_fastphysics_films.jsonc next to the shipped files. Returns facts."""
    sim_config = Path(sim_config)
    robot = json.loads(strip_jsonc((sim_config / ROBOT_SHIPPED).read_text()))
    sensors = robot["sensors"]
    chase = next(s for s in sensors if s.get("id") == "Chase")
    facts = {"chase_size_before": [chase["capture-settings"][0].get("width"), chase["capture-settings"][0].get("height")]}
    chase["capture-settings"][0]["width"] = 640
    chase["capture-settings"][0]["height"] = 480
    front = json.loads(json.dumps(chase))
    front["id"] = "FrontCamera"
    front["origin"] = {"xyz": "0.3 0.0 0.0", "rpy-deg": "0 0 0"}
    front.pop("gimbal", None)  # fixed to the body: looks along the nose
    sensors.append(front)
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
                  "drone_start": next((a.get("origin") for a in scene["actors"]), None)})
    return facts


def _get(obj, *names):
    """Value from a dict or an attribute object by the first name that exists."""
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
    return n, e, d, route.quat_wxyz_to_yaw(w, x, y, z)


async def amain(a) -> int:
    import numpy as np
    from PIL import Image
    from projectairsim import Drone, ProjectAirSimClient, World
    from projectairsim.types import ImageType
    from projectairsim.utils import unpack_image
    from projectairsim.drone import YawControlMode

    out = Path(a.out)
    frames = out / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    res = {"stage": "start", "test": a.test, "traceback": None}

    def save_res():
        (out / "flight.json").write_text(json.dumps(res, indent=2, default=str))

    client = ProjectAirSimClient()
    t0 = time.time()
    while True:
        try:
            client.connect()
            break
        except Exception as e:  # noqa: BLE001
            if time.time() - t0 > a.connect_wait:
                res["stage"] = "no_connection"
                res["error"] = repr(e)
                save_res()
                return 4
            time.sleep(3)
    res["connected_after_s"] = round(time.time() - t0, 1)
    log("connected after", res["connected_after_s"], "s")

    def grab(drone, sensor):
        images = drone.get_images(sensor, [ImageType.SCENE])
        img = unpack_image(list(images.values())[0])
        arr = np.asarray(img)
        if arr.ndim == 3 and arr.shape[2] >= 3:
            arr = arr[:, :, :3][:, :, ::-1]  # unpack_image gives BGR; this is RGB
        return np.ascontiguousarray(arr)

    rc = 0
    samples = []
    try:
        world = drone = None
        used_scene = None
        for scene in (SCENE_FILMS, SCENE_SHIPPED):
            try:
                world = World(client, scene, delay_after_load_sec=2, sim_config_path=a.sim_config)
                drone = Drone(client, world, "Drone1")
                used_scene = scene
                break
            except Exception as e:  # noqa: BLE001
                log("scene", scene, "did not load:", repr(e)[:300])
                res.setdefault("scene_errors", []).append(f"{scene}: {e!r}"[:400])
        if drone is None:
            res["stage"] = "no_scene"
            save_res()
            return 1
        res["scene_used"] = used_scene
        res["clock_type"] = world.get_sim_clock_type()
        drone.enable_api_control()
        drone.arm()

        # ---- picture probe: chase, then the front camera; the front camera falls back to DownCamera ----
        res["stage"] = "probe"
        got, errors = {}, []
        front_names = ["FrontCamera", "DownCamera"] if used_scene == SCENE_FILMS else ["DownCamera"]
        t1 = time.time()
        while time.time() - t1 < a.picture_wait:
            for sensor in ["Chase"] + front_names:
                if sensor in got:
                    continue
                try:
                    got[sensor] = grab(drone, sensor)
                except Exception as e:  # noqa: BLE001
                    errors.append(f"{sensor}: {e!r}"[-300:])
            if "Chase" in got and any(f in got for f in front_names):
                break
            if "Chase" in got and time.time() - t1 > 20 and got:
                break  # a front camera that never answers: go on with what works
            time.sleep(2)
        front = next((f for f in front_names if f in got), None)
        res["probe"] = {"got": sorted(got), "front_sensor": front, "seconds": round(time.time() - t1, 1), "last_errors": errors[-3:]}
        if front is not None:
            arr = got[front].astype(float)
            top = arr[: max(1, arr.shape[0] // 5)]
            res["probe"]["front_top_rows_mean_rgb"] = [round(float(x), 1) for x in top.reshape(-1, 3).mean(axis=0)]
            res["probe"]["front_std"] = round(float(arr.std()), 1)
        log("probe:", res["probe"])
        if not got and not a.test:
            res["stage"] = "no_picture"
            save_res()
            return 3
        skip_grabs = a.test and not got
        res["skip_grabs"] = skip_grabs
        res["front_sensor_used"] = front
        res["front_note"] = ("front-facing camera on the nose" if front == "FrontCamera" else
                             "DownCamera (looks down) because no front camera picture arrived" if front == "DownCamera" else "no front picture")

        # ---- flight ----
        res["stage"] = "flight"
        save_res()
        cam_for = {"fpv": front, "chase": "Chase" if "Chase" in got or not skip_grabs else None}
        take = await drone.takeoff_async()
        sim0 = world.get_sim_time()
        wall0 = time.perf_counter()
        tstart = None  # sim time (s since sim0) when the route starts
        ctl = None
        pics = {"fpv": 0, "chase": 0}
        cam_fail = 0
        last_log = 0.0
        step = 0
        max_wall = a.max_wall
        n = e = d = yaw = 0.0
        cmd_task = None
        while True:
            if time.perf_counter() - wall0 > max_wall:
                res["stop_reason"] = "wall time limit"
                break
            sample = {"i": step, "fpv": None, "chase": None}
            if not skip_grabs:
                for cam in ("fpv", "chase"):
                    sensor = cam_for[cam]
                    if sensor is None:
                        continue
                    try:
                        img = grab(drone, sensor)
                    except Exception as ex:  # noqa: BLE001
                        cam_fail += 1
                        if cam_fail <= 3:
                            log("picture failed", sensor, repr(ex)[-200:])
                        continue
                    name = f"{cam}_{step:05d}.jpg"
                    Image.fromarray(img).save(frames / name, quality=92)
                    sample[cam] = name
                    pics[cam] += 1
            t_sim = (world.get_sim_time() - sim0) / 1e9
            n, e, d, yaw = pose_nedyaw(drone.get_ground_truth_pose())
            sample.update({"t": t_sim, "n": n, "e": e, "d": d, "yaw": yaw})
            if tstart is None and take.done():
                tstart = t_sim
                ctl = route.Controller(n, e, d, yaw)
                res["start"] = {"n": n, "e": e, "d": d, "yaw": yaw, "route_starts_at_sim_s": t_sim}
                # the takeoff already put the drone about 3 m up; the route's own climb is skipped
            samples.append(sample)
            if tstart is not None:
                tr = t_sim - tstart + route.CLIMB_S  # the route clock: skip the climb phase, start at the loop
                if tr > route.TOTAL_S:
                    res["stop_reason"] = "route finished"
                    break
                vn, ve, z, ywc = ctl.command(tr, n, e)
                z_target = res["start"]["d"]  # hold the height where the takeoff ended
                cmd_task = await drone.move_by_velocity_z_async(vn, ve, z_target, 0.5, YawControlMode.MaxDegreeOfFreedom, False, ywc)
            elif t_sim > 40.0:
                res["stop_reason"] = "takeoff did not finish within 40 simulated seconds"
                break
            step += 1
            await asyncio.sleep(0.01)
            if time.perf_counter() - last_log > 20:
                last_log = time.perf_counter()
                log(f"sim t={t_sim:.1f}s wall={last_log - wall0:.0f}s pos=({n:.1f},{e:.1f},{-d:.1f} up) pictures {pics} failures {cam_fail}")
        wall = time.perf_counter() - wall0
        sim_s = samples[-1]["t"] - samples[0]["t"] if len(samples) > 1 else 0.0
        res["timing"] = {"wall_s": round(wall, 2), "sim_s": round(sim_s, 3), "real_time_factor": round(sim_s / wall, 3) if wall > 0 else None,
                         "loop_passes": len(samples), "pictures_taken": pics,
                         "pictures_per_wall_second": round((pics["fpv"] + pics["chase"]) / wall, 2) if wall > 0 else None,
                         "camera_failures": cam_fail}
        res["stage"] = "done"
    except Exception:  # noqa: BLE001
        res["traceback"] = traceback.format_exc()
        res["stage"] = "flight_error"
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
        st = res.get("start", {})
        track = {"fps": a.fps, "plan_xy": route.plan_xy(st.get("n", 0.0), st.get("e", 0.0)),
                 "start_xy": [st.get("n", 0.0), st.get("e", 0.0)], "samples": samples}
        (out / "track.json").write_text(json.dumps(track))
        save_res()
    return rc


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--sim-config", required=True)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--connect-wait", type=float, default=300.0)
    ap.add_argument("--picture-wait", type=float, default=90.0)
    ap.add_argument("--max-wall", type=float, default=900.0)
    ap.add_argument("--test", action="store_true", help="no pictures expected: fly the route anyway")
    a = ap.parse_args()
    return asyncio.run(amain(a))


if __name__ == "__main__":
    sys.exit(main())
