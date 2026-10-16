"""AirSim flight program (runs in the Python 3.9 environment that has the `airsim` package).

Connects to the running AirSim, checks that a picture arrives, flies the route of route.py while the simulation is PAUSED between
steps (pause, continueForTime(1/fps), read the state, take both pictures), and saves pictures and track.json.

Exit codes: 0 = flight done, 3 = no picture within the wait (the caller may start the simulator another way), 4 = no connection,
            1 = error during the flight (what was saved so far is kept).
Cameras: "front" = on the drone, looks along the nose (name in settings.json); "Chase" = external camera that is moved each step.
"""

import argparse
import json
import math
import sys
import time
import traceback
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import route  # noqa: E402

CHASE_BACK_M = 6.0
CHASE_UP_M = 2.5


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def to_rgb(resp):
    if resp.width == 0 or resp.height == 0 or len(resp.image_data_uint8) == 0:
        return None
    a = np.frombuffer(resp.image_data_uint8, np.uint8)
    ch = a.size // (resp.width * resp.height)
    a = a.reshape(resp.height, resp.width, ch)[:, :, :3][:, :, ::-1]  # AirSim delivers BGR(A); this is RGB
    return np.ascontiguousarray(a)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--connect-wait", type=float, default=240.0)
    ap.add_argument("--picture-wait", type=float, default=90.0)
    ap.add_argument("--test", action="store_true", help="no pictures expected: fly the route anyway")
    a = ap.parse_args()
    out = Path(a.out)
    frames = out / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    res = {"stage": "start", "test": a.test, "traceback": None}

    def save_res():
        (out / "flight.json").write_text(json.dumps(res, indent=2, default=str))

    import airsim

    client = airsim.MultirotorClient(timeout_value=120)
    t0 = time.time()
    while True:
        try:
            client.confirmConnection()
            break
        except Exception as e:  # noqa: BLE001
            if time.time() - t0 > a.connect_wait:
                res["stage"] = "no_connection"
                res["error"] = repr(e)
                save_res()
                log("no connection:", e)
                return 4
            time.sleep(3)
    res["connected_after_s"] = round(time.time() - t0, 1)
    res["client_version"] = getattr(airsim, "__version__", None)
    log("connected after", res["connected_after_s"], "s")

    def grab(which):
        if which == "fpv":
            r = client.simGetImages([airsim.ImageRequest("front", airsim.ImageType.Scene, False, False)])
        else:
            r = client.simGetImages([airsim.ImageRequest("Chase", airsim.ImageType.Scene, False, False)], external=True)
        return to_rgb(r[0]) if r else None

    # ---- picture probe ------------------------------------------------------------
    res["stage"] = "probe"
    client.enableApiControl(True)
    client.armDisarm(True)
    got = {"fpv": None, "chase": None}
    errors = []
    t1 = time.time()
    while time.time() - t1 < a.picture_wait and (got["fpv"] is None or got["chase"] is None):
        for cam in ("fpv", "chase"):
            if got[cam] is None:
                try:
                    got[cam] = grab(cam)
                except Exception as e:  # noqa: BLE001
                    errors.append(f"{cam}: {e!r}"[:300])
        if got["fpv"] is None or got["chase"] is None:
            time.sleep(2)
    res["probe"] = {"fpv": got["fpv"] is not None, "chase": got["chase"] is not None, "seconds": round(time.time() - t1, 1),
                    "last_errors": errors[-3:]}
    log("probe:", res["probe"])
    if got["fpv"] is None and got["chase"] is None and not a.test:
        res["stage"] = "no_picture"
        save_res()
        return 3
    if got["fpv"] is not None:
        arr = got["fpv"].astype(float)
        top = arr[: arr.shape[0] // 5]
        res["probe"]["fpv_top_rows_mean_rgb"] = [round(float(x), 1) for x in top.reshape(-1, 3).mean(axis=0)]
        res["probe"]["fpv_std"] = round(float(arr.std()), 1)

    skip_grabs = a.test and got["fpv"] is None and got["chase"] is None
    res["skip_grabs"] = skip_grabs
    # ---- the flight ---------------------------------------------------------------
    res["stage"] = "flight"
    save_res()
    dt = 1.0 / a.fps
    samples, rc = [], 0
    try:
        st0 = client.getMultirotorState()
        p0 = st0.kinematics_estimated.position
        sn, se, sd = p0.x_val, p0.y_val, p0.z_val
        q0 = st0.kinematics_estimated.orientation
        yaw0 = route.quat_wxyz_to_yaw(q0.w_val, q0.x_val, q0.y_val, q0.z_val)
        ctl = route.Controller(sn, se, sd, yaw0)
        res["start"] = {"n": sn, "e": se, "d": sd, "yaw": yaw0}
        ts0 = st0.timestamp
        client.simPause(True)
        nsteps = int(round(route.TOTAL_S * a.fps))
        wall0 = time.perf_counter()
        pics = {"fpv": 0, "chase": 0}
        collisions = 0
        cam_fail = 0
        n, e = sn, se
        for step in range(nsteps + 1):
            t = step * dt
            vn, ve, z, yaw = ctl.command(t, n, e)
            client.moveByVelocityZAsync(vn, ve, z, dt * 4, airsim.DrivetrainType.MaxDegreeOfFreedom,
                                        airsim.YawMode(False, math.degrees(yaw)))
            if step > 0:
                client.simContinueForTime(dt)
            st = client.getMultirotorState()
            p = st.kinematics_estimated.position
            q = st.kinematics_estimated.orientation
            n, e, d = p.x_val, p.y_val, p.z_val
            cur_yaw = route.quat_wxyz_to_yaw(q.w_val, q.x_val, q.y_val, q.z_val)
            if step % 5 == 0:
                try:
                    if client.simGetCollisionInfo().has_collided:
                        collisions += 1
                except Exception:  # noqa: BLE001
                    pass
            # the chase camera: behind the drone along its heading, above it, looking at it
            cn, ce, cd = n - CHASE_BACK_M * math.cos(cur_yaw), e - CHASE_BACK_M * math.sin(cur_yaw), d - CHASE_UP_M
            pitch = -math.atan2(CHASE_UP_M, CHASE_BACK_M)
            sample = {"i": step, "t": (st.timestamp - ts0) / 1e9, "t_nominal": t, "n": n, "e": e, "d": d, "yaw": cur_yaw,
                      "fpv": None, "chase": None}
            try:
                if skip_grabs:
                    raise RuntimeError("test run without pictures")
                client.simSetCameraPose("Chase", airsim.Pose(airsim.Vector3r(cn, ce, cd), airsim.to_quaternion(pitch, 0.0, cur_yaw)),
                                        external=True)
            except Exception as ex:  # noqa: BLE001
                if cam_fail == 0:
                    log("simSetCameraPose failed:", repr(ex)[:200])
            for cam in ("fpv", "chase"):
                if skip_grabs:
                    break
                try:
                    img = grab(cam)
                except Exception as ex:  # noqa: BLE001
                    img = None
                    if cam_fail < 3:
                        log("picture failed", cam, repr(ex)[:200])
                if img is None:
                    cam_fail += 1
                    continue
                name = f"{cam}_{step:05d}.jpg"
                Image.fromarray(img).save(frames / name, quality=92)
                sample[cam] = name
                pics[cam] += 1
            samples.append(sample)
            if step % 75 == 0:
                log(f"step {step}/{nsteps} t={t:.1f}s pos=({n:.1f},{e:.1f},{-d:.1f}m up) pictures {pics}")
        wall = time.perf_counter() - wall0
        sim_s = samples[-1]["t"] - samples[0]["t"] if len(samples) > 1 else 0.0
        res["timing"] = {"wall_s": round(wall, 2), "sim_s": round(sim_s, 3), "real_time_factor": round(sim_s / wall, 3) if wall > 0 else None,
                         "steps": len(samples), "pictures_taken": pics,
                         "pictures_per_wall_second": round((pics["fpv"] + pics["chase"]) / wall, 2) if wall > 0 else None,
                         "steps_per_wall_second": round(len(samples) / wall, 2) if wall > 0 else None,
                         "camera_failures": cam_fail, "collision_checks_with_hit": collisions}
        res["stage"] = "done"
    except Exception:  # noqa: BLE001
        res["traceback"] = traceback.format_exc()
        res["stage"] = "flight_error"
        log(res["traceback"])
        rc = 1
    finally:
        try:
            client.simPause(False)
            client.hoverAsync()
            client.armDisarm(False)
            client.enableApiControl(False)
        except Exception:  # noqa: BLE001
            pass
        track = {"fps": a.fps, "plan_xy": route.plan_xy(res.get("start", {}).get("n", 0.0), res.get("start", {}).get("e", 0.0)),
                 "start_xy": [res.get("start", {}).get("n", 0.0), res.get("start", {}).get("e", 0.0)], "samples": samples}
        (out / "track.json").write_text(json.dumps(track))
        save_res()
    return rc


if __name__ == "__main__":
    sys.exit(main())
