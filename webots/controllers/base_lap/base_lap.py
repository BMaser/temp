"""Webots supervisor controller of the Crazyflie: flies one lap through the four level 0 gates (5 passages) with our own
cascaded controller (lap_core.py), counts passages, watches for collisions, and saves the flight.

Environment: BASE_LAP_REPO = checkout (for controllers/nose_yaw.py), BASE_LAP_OUT = folder for the results.
Writes into BASE_LAP_OUT: states.npz, child.json, fpv.rgb (raw 640 x 480 RGB pictures, one every 40 ms).
The Webots Crazyflie robot (protos/Crazyflie.proto, Apache-2.0) and its example controllers are the model for the motor
names and signs; no code is copied from them except the mixing signs (see lap_core.py and webots/NOTICE.md).
"""

import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = Path(os.environ.get("BASE_LAP_REPO", HERE.parents[2]))
OUT = Path(os.environ.get("BASE_LAP_OUT", REPO / "results" / "webots_base"))
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "controllers"))
import lap_core as lc  # noqa: E402
import nose_yaw as ny  # noqa: E402

from controller import Supervisor  # noqa: E402

YAW_RATE_MAX = 4.0  # rad/s, the rate run A chose
V_MIN = 0.15
SAMPLE_EVERY = 5  # steps of 4 ms -> 50 Hz rows
PICTURE_EVERY = 10  # steps -> 25 pictures per second
AFTER_END_S = 0.4
T_LIMIT = lc.T_TOTAL + 10.0
LIFTOFF_Z = 0.06


def classify_contact(pt):
    """Name of what a contact point (world coordinates) belongs to, from its place."""
    pt = np.asarray(pt, float)
    if pt[2] < 0.004:
        return "floor"
    best = ("unknown", 1.0)
    for g in range(len(lc.GATES)):
        lp = lc.gate_local(g, pt)
        for b in lc.GATE_BOXES:
            d = lc.box_distance(lp, b)
            if d < best[1]:
                best = (f"gate_{g}", d)
    for k, (x, y, top) in enumerate(lc.POLES):
        d = max(math.hypot(pt[0] - x, pt[1] - y) - lc.POLE_RADIUS, 0.0) if pt[2] <= top else 1.0
        if d < best[1]:
            best = (f"pole_{k}", d)
    return best[0]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    child = {"tag": "webots_base_lap", "simulator": "webots", "nose_mode": "forward", "yaw_rate_max": YAW_RATE_MAX,
             "traceback": None, "end": None, "terminated_steps_dropped": 0, "versions": {"webots": os.environ.get("WEBOTS_VERSION", "R2025a")}}
    robot = None
    frames = None
    try:
        robot = Supervisor()
        dt_ms = int(robot.getBasicTimeStep())
        dt = dt_ms / 1000.0
        freq_rows = 1.0 / (SAMPLE_EVERY * dt)
        child["basic_time_step_ms"] = dt_ms
        node = robot.getSelf()
        motors = []
        for name in ("m1_motor", "m2_motor", "m3_motor", "m4_motor"):
            m = robot.getDevice(name)
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            motors.append(m)
        sign = np.array([-1.0, 1.0, -1.0, 1.0])
        cam = robot.getDevice("fpv")
        cam.enable(PICTURE_EVERY * dt_ms)
        width, height = cam.getWidth(), cam.getHeight()
        child["camera_size"] = [width, height]
        frames = open(OUT / "fpv.rgb", "wb")

        contact_api = "none"
        try:
            node.enableContactPointsTracking(dt_ms, True)
            contact_api = "getContactPoints"
        except Exception as e:  # noqa: BLE001
            child["contact_enable_error"] = repr(e)
            contact_api = "getContactPoints (no tracking call)"
        child["contact_api"] = contact_api

        def read_state():
            p = np.array(node.getPosition())
            R = np.array(node.getOrientation()).reshape(3, 3)
            vel = node.getVelocity()
            return p, np.array(vel[:3]), R, np.array(vel[3:])

        p, v, R, w_world = read_state()
        if not np.allclose(R.T @ R, np.eye(3), atol=1e-3):  # pose not valid before the first step: take one step
            robot.step(dt_ms)
            child["extra_first_step"] = True
            p, v, R, w_world = read_state()
        start = np.array([*lc.START_XY, p[2]])
        child["start_position_in_world"] = [float(x) for x in p]
        child["start_error_m"] = float(np.linalg.norm(p[:2] - np.array(lc.START_XY)))
        spline = lc.build_spline(start)
        n_plan = int(round((lc.T_TOTAL + 12.0) / dt)) + 2
        times = np.arange(n_plan) * dt
        vel_xy = spline(np.minimum(times, lc.T_TOTAL), 1)[:, :2]
        start_yaw = float(ny.yaw_from_quat_xyzw(lc.quat_xyzw_from_R(R)))
        plan = ny.plan_yaw(vel_xy, dt, YAW_RATE_MAX, V_MIN, start_yaw)
        yaw_fn = lambda t: float(plan[min(max(int(round(t / dt)), 0), len(plan) - 1)])
        ctl = lc.LapController(dt, yaw_fn, spline)
        pc = lc.PassageCounter()

        rec = {k: [] for k in ("time", "pos", "vel", "quat", "ang_vel", "passed", "cmd_yaw", "motors")}
        pic_step = []
        collisions = []  # (time, what)
        airborne = False
        min_frame = (math.inf, -1, 0.0)
        min_pole = (math.inf, -1, 0.0)
        geo_hits = []
        max_tilt = 0.0
        end, end_n = None, None
        n = 1 if child.get("extra_first_step") else 0
        wall0 = time.perf_counter()
        steps_loop = 0
        while True:
            t = n * dt
            p, v, R, w_world = read_state()
            w_body = R.T @ w_world
            # rows
            if n % SAMPLE_EVERY == 0:
                rec["time"].append(len(rec["time"]) / freq_rows)
                rec["pos"].append(p.tolist())
                rec["vel"].append(v.tolist())
                rec["quat"].append(lc.quat_xyzw_from_R(R).tolist())
                rec["ang_vel"].append(w_body.tolist())
                rec["passed"].append(int(pc.count))
                rec["cmd_yaw"].append(yaw_fn(t))
            pc.update(t, p)
            if n % SAMPLE_EVERY == 0:
                rec["passed"][-1] = int(pc.count)
            if p[2] > LIFTOFF_Z:
                airborne = True
            max_tilt = max(max_tilt, math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1)))))
            d, gi = lc.distance_to_frame(p)
            if d < min_frame[0]:
                min_frame = (d, gi, t)
            for g in range(len(lc.GATES)):
                lp = lc.gate_local(g, p)
                dd = min(lc.box_distance(lp, b) for b in lc.GATE_BOXES)
                if dd < 0.05:
                    geo_hits.append((round(t, 3), f"gate_{g}", round(dd, 4)))
            for k, (x, y, top) in enumerate(lc.POLES):
                if p[2] <= top:
                    dp = math.hypot(p[0] - x, p[1] - y) - lc.POLE_RADIUS
                    if dp < min_pole[0]:
                        min_pole = (dp, k, t)
                    if dp < 0.05:
                        geo_hits.append((round(t, 3), f"pole_{k}", round(dp, 4)))
            # contacts
            try:
                pts = node.getContactPoints(True)
            except Exception as e:  # noqa: BLE001
                pts = []
                child.setdefault("contact_errors", []).append(repr(e))
            for cp in pts:
                what = classify_contact(cp.point)
                if what == "floor" and not airborne:
                    continue
                if end is None or what != "floor":
                    collisions.append((round(t, 3), what, [round(float(x), 4) for x in cp.point]))
            # end conditions
            if end is None:
                if pc.count >= len(lc.GATE_SEQUENCE):
                    end, end_n = "finished", n
                elif collisions:
                    end, end_n = "crash", n
                elif t >= T_LIMIT:
                    end, end_n = "time", n
            if end is not None and (n - end_n) * dt >= AFTER_END_S and n % SAMPLE_EVERY == 0:
                break
            # control
            wm = ctl.step(t, p, v, R, w_body) if end is None else ctl.step(min(t, lc.T_TOTAL), p, v, R, w_body)
            for m, s, val in zip(motors, sign, wm):
                m.setVelocity(float(s * val))
            if robot.step(dt_ms) == -1:
                end = end or "time"
                break
            n += 1
            steps_loop += 1
            if n % PICTURE_EVERY == 0:
                raw = cam.getImage()
                arr = np.frombuffer(raw, np.uint8).reshape(height, width, 4)[:, :, [2, 1, 0]]
                frames.write(np.ascontiguousarray(arr).tobytes())
                pic_step.append(n // SAMPLE_EVERY)
        wall = time.perf_counter() - wall0
        frames.close()
        frames = None
        # a last row must exist for every picture row; the pictures refer to rows n // 5, which are recorded one iteration later
        rows = len(rec["time"])
        pic_step = [r for r in pic_step if r < rows]
        n_pic = len(pic_step)
        if n_pic < os.path.getsize(OUT / "fpv.rgb") // (width * height * 3):
            child["pictures_dropped_for_missing_row"] = os.path.getsize(OUT / "fpv.rgb") // (width * height * 3) - n_pic
        total = len(lc.GATE_SEQUENCE)
        arrays = {
            "time": np.array(rec["time"]), "pos": np.array(rec["pos"]), "vel": np.array(rec["vel"]), "quat": np.array(rec["quat"]),
            "ang_vel": np.array(rec["ang_vel"]), "cmd_yaw": np.array(rec["cmd_yaw"]), "passed": np.array(rec["passed"], dtype=np.int64),
            "pic_step": np.array(pic_step, dtype=np.int64), "pic_time": np.array([rec["time"][r] for r in pic_step]),
            "plan": plan, "planned_xy": spline(np.linspace(0, lc.T_TOTAL, 200))[:, :2], "start_yaw": np.array(start_yaw),
            "gates_pos": lc.GATES[:, :3].copy(),
            "gates_quat": np.array([[0.0, 0.0, math.sin(y / 2), math.cos(y / 2)] for y in lc.GATES[:, 3]]),
            "gate_sequence": np.array(lc.GATE_SEQUENCE, dtype=np.int64), "gate_direction": np.ones(total, dtype=np.int64),
            "obstacles_pos": lc.POLES.copy(), "pad_start": np.array(lc.START_XY), "pad_end": lc.POINTS_AFTER_START[-1][:2].copy(),
            "freq": np.array(int(round(freq_rows))),
        }
        np.savez(OUT / "states.npz", **arrays)
        child.update(
            end=end, rows_saved=int(rows), passages=int(rec["passed"][-1]), gates_total=total, steps=steps_loop, wall_s=wall,
            steps_per_s=steps_loop / wall if wall else None, sim_seconds=float(rec["time"][-1]), pictures=n_pic,
            max_tilt_deg=max_tilt, closest_frame_distance_m=min_frame[0], closest_frame_gate=min_frame[1], closest_frame_time=min_frame[2],
            closest_pole_surface_distance_m=min_pole[0], closest_pole=min_pole[1],
            passage_log=[{"time": a, "passage": b, "gate": c, "ly": d, "lz": e} for a, b, c, d, e in pc.log],
            wrong_crossings=[{"time": a, "gate": b, "ly": c, "lz": d} for a, b, c, d in pc.wrong],
            collisions=collisions[:50], collision_count=len(collisions), geometric_hits_within_0p05m=geo_hits[:50], geometric_hit_count=len(geo_hits),
            airborne=airborne,
        )
    except Exception:  # noqa: BLE001
        child["traceback"] = traceback.format_exc()
        if child.get("end") is None:
            child["end"] = "error"
    finally:
        if frames is not None:
            frames.close()
        (OUT / "child.json").write_text(json.dumps(child, indent=2, default=str))
        if robot is not None:
            try:
                robot.simulationQuit(0)
            except Exception:  # noqa: BLE001
                pass


main()
