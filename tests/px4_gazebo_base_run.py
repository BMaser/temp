"""PX4 + Gazebo base lap: start PX4 SITL with Gazebo Harmonic (world baylands, model x500_mono_cam plus a chase camera), put four
gate frames in the park, fly the route of the Webots lap with MAVSDK offboard setpoints, film it, and write the films, the result
file and the summary.

  python3 tests/px4_gazebo_base_run.py run  --out results [--px4 px4]   # start everything, fly, then do `post`
  python3 tests/px4_gazebo_base_run.py post --out results               # films, flight_result.json, summary.md (needs what `run` saved)

Needs (on the runner, see .github/workflows/px4-gazebo-base.yml): the PX4-Autopilot checkout built with `make px4_sitl_default`,
Gazebo Harmonic with its python bindings, xvfb, Mesa, ffmpeg, mavsdk (python), numpy, scipy, Pillow.
Time is Gazebo simulation time everywhere (the pose and picture time stamps); wall time only decides time-outs.
The pass rule is applied by tests/px4_gazebo_base_check.py.
"""

import argparse
import asyncio
import datetime
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "px4_gazebo"))
import flight_result  # noqa: E402
import px4_core as pc  # noqa: E402
import sync_film  # noqa: E402

WORLD = "baylands"
MODEL = "x500_mono_cam_0"
WIDTH, HEIGHT = 640, 480
FILM_FPS = 15
ROW_HZ = 50
SENSOR_WIDTH_FROM, SENSOR_HEIGHT_FROM = "1280", "960"
STACK_START_TIMEOUT_S = 15 * 60
FLIGHT_TIMEOUT_S = 50 * 60
CHASE_BACK_M, CHASE_UP_M = 1.5, 0.5

CHASE_LINK = f"""
    <link name="chase_cam_link">
      <pose relative_to="base_link">-{CHASE_BACK_M} 0 {CHASE_UP_M} 0 {math.atan2(CHASE_UP_M, CHASE_BACK_M):.4f} 0</pose>
      <gravity>false</gravity>
      <inertial><mass>0.001</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>
      <sensor name="chase_cam" type="camera">
        <gz_frame_id>chase_cam_link</gz_frame_id>
        <camera>
          <horizontal_fov>1.4</horizontal_fov>
          <image><width>{WIDTH}</width><height>{HEIGHT}</height></image>
          <clip><near>0.1</near><far>3000</far></clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>false</visualize>
      </sensor>
    </link>
    <joint name="ChaseCamJoint" type="fixed"><parent>base_link</parent><child>chase_cam_link</child></joint>
"""


def sh(cmd, timeout=30):
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return p.stdout + p.stderr
    except subprocess.TimeoutExpired:
        return ""


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ------------------------------------------------------------------------------------------------ models
def patch_models(px4: Path) -> dict:
    """The drone camera at 640 x 480 (4 times fewer pixels to draw in software) and a chase camera 1.5 m behind and 0.5 m above."""
    models = px4 / "Tools" / "simulation" / "gz" / "models"
    mono, x500 = models / "mono_cam" / "model.sdf", models / "x500_mono_cam" / "model.sdf"
    info = {}
    t = mono.read_text()
    t2 = t.replace(f"<width>{SENSOR_WIDTH_FROM}</width>", f"<width>{WIDTH}</width>").replace(f"<height>{SENSOR_HEIGHT_FROM}</height>", f"<height>{HEIGHT}</height>")
    info["mono_cam_changed"] = t2 != t
    mono.write_text(t2)
    t = x500.read_text()
    if "chase_cam_link" not in t:
        k = t.rindex("</model>")
        t = t[:k] + CHASE_LINK + t[k:]
    x500.write_text(t)
    info["x500_mono_cam_has_chase"] = "chase_cam_link" in x500.read_text()
    log("patched models: " + json.dumps(info))
    return info


# ------------------------------------------------------------------------------------------------ the shared state of the flight
class Shared:
    def __init__(self, out: Path):
        self.out = out
        self.lock = threading.Lock()
        self.sim_t = None            # latest simulation time, s
        self.stats_t = None
        self.pose_n = 0
        self.stamp_zero = 0
        self.pose = None             # latest (x, y, z, qx, qy, qz, qw) in the world frame
        self.poses = []              # (t, x, y, z, qx, qy, qz, qw) while recording
        self.recording = False
        self.t0 = None               # simulation time when the recording started
        self.films = {}
        self.node = None
        self.cam_topics = {}
        self.image_problems = []

    def now(self):
        with self.lock:
            return self.sim_t


class FilmRec:
    def __init__(self, path: Path):
        self.path, self.f = path, open(path, "wb")
        self.times, self.next_t, self.active, self.n_seen = [], None, False, 0

    def add(self, t, data: bytes):
        self.n_seen += 1
        if not self.active:
            return
        if self.next_t is None:
            self.next_t = t
        if t + 1e-9 < self.next_t:
            return
        self.f.write(data)
        self.times.append(t)
        self.next_t += 1.0 / FILM_FPS
        if self.next_t < t:
            self.next_t = t + 1.0 / FILM_FPS

    def close(self):
        self.f.close()


def stamp_seconds(header, fallback):
    t = header.stamp.sec + header.stamp.nsec * 1e-9
    return t if t > 0 else fallback


def setup_gz(S: Shared, world: str):
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.transport13 import Node

    S.Image = Image
    node = Node()
    S.node = node

    def on_stats(msg):
        with S.lock:
            S.stats_t = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

    def on_pose(msg):
        t = stamp_seconds(msg.header, S.stats_t)
        if t is None:
            return
        for p in msg.pose:
            if p.name == MODEL:
                row = (t, p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
                with S.lock:
                    S.pose_n += 1
                    if msg.header.stamp.sec == 0 and msg.header.stamp.nsec == 0:
                        S.stamp_zero += 1
                    S.sim_t = t
                    S.pose = row[1:]
                    if S.recording:
                        S.poses.append(row)
                break

    ok = [node.subscribe(WorldStatistics, f"/world/{world}/stats", on_stats)]
    ok.append(node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", on_pose))
    log(f"subscribed to stats and dynamic_pose: {ok}")
    return node


def subscribe_camera(S: Shared, name: str, topic: str):
    rec = S.films[name]

    def on_image(msg):
        if msg.width != WIDTH or msg.height != HEIGHT or len(msg.data) != WIDTH * HEIGHT * 3:
            if len(S.image_problems) < 5:
                S.image_problems.append(f"{name}: {msg.width}x{msg.height}, {len(msg.data)} bytes, format {msg.pixel_format_type}")
            return
        t = stamp_seconds(msg.header, S.now())
        if t is None:
            return
        rec.add(t, msg.data)

    return S.node.subscribe(S.Image, topic, on_image)


# ------------------------------------------------------------------------------------------------ start and stop the simulator
def start_stack(out: Path, px4: Path):
    procs = {}
    env = {**os.environ, "DISPLAY": ":99", "LIBGL_ALWAYS_SOFTWARE": "1", "HEADLESS": "1", "GZ_VERBOSE": "2"}
    procs["xvfb"] = subprocess.Popen(["Xvfb", ":99", "-screen", "0", "1280x1024x24", "+extension", "GLX", "+render", "-noreset"],
                                     stdout=open(out / "xvfb.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(3)
    os.environ.update(env)
    cmd = f'cd "{px4}" && sleep 4000 | setsid make px4_sitl gz_x500_mono_cam_{WORLD} > "{out / "sim.log"}" 2>&1'
    procs["px4"] = subprocess.Popen(["bash", "-c", cmd], env=env, start_new_session=True)
    return procs


def stop_stack(procs):
    for pattern in ("build/px4_sitl_default/bin/px4", "gz sim", "ruby"):
        sh(f'pkill -f "{pattern}"', 10)
    for k in ("px4", "xvfb"):
        p = procs.get(k)
        if p is not None:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass


def wait_for(pattern: str, timeout_s: float):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        out = sh("gz topic -l")
        m = re.search(pattern, out, re.M)
        if m:
            return m, out
        time.sleep(3)
    return None, sh("gz topic -l")


def spawn_gates(out: Path, ground_z: float, spawn_xy):
    """Create the four gates in the running world with the create service (SDF files in out/gates/)."""
    d = out / "gates"
    d.mkdir(exist_ok=True)
    answers = []
    for i in range(len(pc.GATES)):
        text = pc.gate_sdf(i, ground_z, spawn_xy)
        p = d / f"gate_{i + 1}.sdf"
        p.write_text(text)
        r = sh(f"gz service -s /world/{WORLD}/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 15000 "
               f"--req 'sdf_filename: \"{p}\"'", 40)
        answers.append(r.strip())
        log(f"gate {i + 1}: {r.strip()}")
    return answers


# ------------------------------------------------------------------------------------------------ the flight
async def fly(S: Shared, child: dict, ground):
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

    x0, y0, z0 = ground

    def pose_f():
        with S.lock:
            p = S.pose
        return None if p is None else (p[0] - x0, p[1] - y0, p[2] - z0, p[3:])

    d = System()
    await d.connect(system_address="udpin://0.0.0.0:14540")
    log("waiting for the connection to PX4")
    async def connected():
        async for st in d.core.connection_state():
            if st.is_connected:
                return
    await asyncio.wait_for(connected(), 300)
    log("connected; waiting for a good position estimate and home")
    async def healthy():
        async for h in d.telemetry.health():
            if h.is_global_position_ok and h.is_home_position_ok:
                return h
    await asyncio.wait_for(healthy(), 900)
    log("healthy")

    # start the recording: the cameras draw only while somebody listens, so the physics runs faster before this point
    with S.lock:
        S.t0 = S.sim_t
        S.poses = []
        S.recording = True
    for name in S.films:
        S.films[name].active = True
        child.setdefault("camera_subscribe", {})[name] = subscribe_camera(S, name, S.cam_topics[name])
    wall0 = time.monotonic()
    child["record_start_sim_s"] = S.t0
    yaw0 = pc.ned_yaw_deg(0.0)
    await d.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw0))
    await d.action.arm()
    t_arm = S.now()
    log(f"armed at sim time {t_arm}")
    try:
        await d.offboard.start()
    except OffboardError as e:
        child["offboard_error"] = str(e)
        raise
    log("offboard started")

    phase, tau0, end_reason = "climb", None, None
    settled_since = None
    while True:
        await asyncio.sleep(0.02)
        t = S.now()
        if time.monotonic() - wall0 > FLIGHT_TIMEOUT_S:
            end_reason = "time"
            break
        pf = pose_f()
        if pf is not None:
            R = pf[3]
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (R[0] ** 2 + R[1] ** 2)))))
            if tilt > 75.0 and t - t_arm > 3:
                end_reason = "crash"
                break
        if phase == "climb":
            zsp = min(pc.HOVER_Z, 0.8 * max(0.0, t - t_arm))
            vz = 0.8 if zsp < pc.HOVER_Z else 0.0
            await d.offboard.set_position_velocity_ned(PositionNedYaw(0.0, 0.0, -zsp, yaw0), VelocityNedYaw(0.0, 0.0, -vz, yaw0))
            if pf is not None and zsp >= pc.HOVER_Z:
                near = math.dist(pf[:3], (0.0, 0.0, pc.HOVER_Z)) < 0.4
                settled_since = (settled_since if settled_since is not None else t) if near else None
                if settled_since is not None and t - settled_since > 2.0:
                    phase, tau0 = "route", t
                    child["route_start_sim_s"] = t
                    log(f"route starts at sim time {t}")
            if t - t_arm > 90.0 and phase == "climb":
                child["climb_failed"] = True
                end_reason = "time"
                break
        else:
            tau = t - tau0
            if tau > pc.T_ROUTE + pc.HOLD_END_S:
                end_reason = "finished"
                break
            p, v = pc.ref_pos_vel(tau)
            yaw = pc.ned_yaw_deg(pc.ref_heading(tau))
            await d.offboard.set_position_velocity_ned(PositionNedYaw(float(p[1]), float(p[0]), float(-p[2]), yaw),
                                                      VelocityNedYaw(float(v[1]), float(v[0]), float(-v[2]), yaw))
    child["flight_end_reason"] = end_reason
    child["record_wall_s"] = time.monotonic() - wall0
    with S.lock:
        S.recording = False
        child["record_end_sim_s"] = S.sim_t
    for name in S.films:
        S.films[name].active = False
        S.node.unsubscribe(S.cam_topics[name])
    try:
        await d.action.land()
    except Exception as e:  # noqa: BLE001
        child["land_error"] = repr(e)


def run_stack(out: Path, px4: Path) -> dict:
    child = {"tag": "px4_gazebo_base_lap", "simulator": "px4_gazebo", "nose_mode": "forward", "yaw_rate_max": pc.YAW_RATE_MAX,
             "controller_name": "PX4 position controller with offboard position and velocity setpoints (MAVSDK)",
             "versions": {}, "terminated_steps_dropped": 0}
    child["model_patch"] = patch_models(px4)
    S = Shared(out)
    procs = start_stack(out, px4)
    try:
        t_start = time.time()
        m, topics = wait_for(r"^/world/([^/]+)/clock$", STACK_START_TIMEOUT_S)
        (out / "gz_topics.txt").write_text(topics)
        if not m:
            raise RuntimeError("no Gazebo world clock topic: Gazebo did not start (see sim.log)")
        world = m.group(1)
        child["world"] = world
        log(f"world {world} after {time.time() - t_start:.0f} s")
        cams = {}
        for name, sensor in (("fpv", r"link/camera_link/sensor/imager/image"), ("chase", r"link/chase_cam_link/sensor/chase_cam/image")):
            mm, topics = wait_for(r"^(/world/%s/model/%s/%s)$" % (re.escape(world), re.escape(MODEL), sensor), 300)
            (out / "gz_topics.txt").write_text(topics)
            if not mm:
                raise RuntimeError(f"camera topic for {name} did not appear")
            cams[name] = mm.group(1)
        S.cam_topics = cams
        child["camera_topics"] = cams
        setup_gz(S, world)
        t0 = time.time()
        while S.pose is None and time.time() - t0 < 300:
            time.sleep(1)
        if S.pose is None:
            raise RuntimeError("no pose of the drone model in dynamic_pose/info (topics in gz_topics.txt)")
        time.sleep(8)  # let the drone settle on the ground
        with S.lock:
            ground = (S.pose[0], S.pose[1], S.pose[2])
        child["ground_reference_world"] = list(ground)
        log(f"drone at rest at world {ground}")
        child["gate_service_answers"] = spawn_gates(out, ground[2], ground[:2])
        for name in cams:
            S.films[name] = FilmRec(out / f"{name}.rgb")
        asyncio.run(fly(S, child, ground))
        child["pose_messages"] = S.pose_n
        child["pose_messages_without_stamp"] = S.stamp_zero
        child["image_problems"] = S.image_problems
        child["frames_seen"] = {k: v.n_seen for k, v in S.films.items()}
        child["frames_kept"] = {k: len(v.times) for k, v in S.films.items()}
        for v in S.films.values():
            v.close()
        save_states(out, S, child, ground)
    except Exception:  # noqa: BLE001
        child["traceback"] = traceback.format_exc()
        log(child["traceback"])
        child.setdefault("end", "error")
        for v in S.films.values():
            try:
                v.close()
            except Exception:  # noqa: BLE001
                pass
        if S.poses and "states_saved" not in child:
            try:
                save_states(out, S, child, (S.pose[0], S.pose[1], S.pose[2]) if S.pose else (0, 0, 0))
            except Exception:  # noqa: BLE001
                child["save_traceback"] = traceback.format_exc()
    finally:
        (out / "child.json").write_text(json.dumps(child, indent=2, default=str))
        stop_stack(procs)
    return child


def resample(poses: np.ndarray, grid: np.ndarray):
    t = poses[:, 0]
    keep = np.concatenate([[True], np.diff(t) > 1e-9])
    poses, t = poses[keep], t[keep]
    pos = np.stack([np.interp(grid, t, poses[:, k]) for k in (1, 2, 3)], axis=1)
    q = poses[:, 4:8].copy()
    for i in range(1, len(q)):  # same hemisphere as the sample before
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]
    qi = np.stack([np.interp(grid, t, q[:, k]) for k in range(4)], axis=1)
    qi /= np.linalg.norm(qi, axis=1, keepdims=True)
    return pos, qi


def save_states(out: Path, S: Shared, child: dict, ground):
    x0, y0, z0 = ground
    poses = np.array(S.poses, dtype=float)
    if len(poses) < 10:
        raise RuntimeError(f"only {len(poses)} pose rows were recorded")
    t0 = S.t0 if S.t0 is not None else poses[0, 0]
    t_end = poses[-1, 0]
    rows = int((t_end - t0) * ROW_HZ)
    grid = t0 + np.arange(rows) / ROW_HZ
    grid = grid[(grid >= poses[0, 0]) & (grid <= t_end)]
    shift = grid[0] - t0
    pos, quat = resample(poses, grid)
    pos = pos - np.array([x0, y0, z0])
    time_rel = np.arange(len(grid)) / ROW_HZ
    vel = np.gradient(pos, 1.0 / ROW_HZ, axis=0)
    # passages, closest frame, tilt
    counter = pc.PassageCounter()
    passed, dist_rows, hits = np.zeros(len(grid), dtype=np.int64), np.zeros(len(grid)), []
    min_frame = (math.inf, -1, 0.0)
    max_tilt = 0.0
    for i, (tt, p) in enumerate(zip(time_rel, pos)):
        passed[i] = counter.update(float(tt), p)
        dd, gi = pc.distance_to_frame(p)
        dist_rows[i] = dd
        if dd < min_frame[0]:
            min_frame = (dd, gi, float(tt))
        if dd < pc.DRONE_RADIUS:
            hits.append((round(float(tt), 3), f"gate_{gi + 1}", round(dd, 4)))
        qx, qy = quat[i, 0], quat[i, 1]
        max_tilt = max(max_tilt, math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qx * qx + qy * qy))))))
    pic = {}
    for name, rec in S.films.items():
        times = np.array(rec.times) - t0 - shift
        steps = np.clip(np.rint(times * ROW_HZ).astype(np.int64), 0, len(grid) - 1)
        pic[name] = (steps, times)
    plan_t = np.linspace(0, pc.T_ROUTE, 200)
    plan_xy = np.array([pc.SPLINE(t)[:2] for t in plan_t])
    gates_quat = np.array([[0.0, 0.0, math.sin(y / 2), math.cos(y / 2)] for y in pc.GATES[:, 3]])
    total = len(pc.SEQUENCE)
    arrays = {
        "time": time_rel, "pos": pos, "vel": vel, "quat": quat, "passed": passed,
        "pic_step": pic["fpv"][0], "pic_time": pic["fpv"][1], "chase_pic_step": pic["chase"][0], "chase_pic_time": pic["chase"][1],
        "planned_xy": plan_xy, "gates_pos": pc.GATES[:, :3].copy(), "gates_quat": gates_quat,
        "gate_sequence": np.array(pc.SEQUENCE, dtype=np.int64), "gate_direction": np.ones(total, dtype=np.int64),
        "obstacles_pos": np.zeros((0, 3)), "pad_start": np.array([0.0, 0.0]), "pad_end": np.array(pc.to_track(pc.lc.POINTS_AFTER_START[-1])[:2]),
        "freq": np.array(ROW_HZ),
    }
    np.savez(out / "states.npz", **arrays)
    reason = child.get("flight_end_reason")
    finished = reason == "finished" and counter.count == total
    child.update(
        states_saved=True, rows_saved=int(len(grid)), passages=int(counter.count), gates_total=total, end="finished" if finished else ("crash" if reason == "crash" else "time"),
        max_tilt_deg=max_tilt, closest_frame_distance_m=min_frame[0], closest_frame_gate=min_frame[1], closest_frame_time=min_frame[2],
        frame_hit_count=len(hits), frame_hits=hits[:50], drone_radius_m=pc.DRONE_RADIUS,
        passage_log=[{"time": a, "passage": b, "gate": c, "ly": d, "lz": e} for a, b, c, d, e in counter.log],
        wrong_crossings=[{"time": a, "gate": b, "ly": c, "lz": d} for a, b, c, d in counter.wrong],
        sim_seconds=float(time_rel[-1]), route_start_row_time=(child.get("route_start_sim_s", t0) - t0 - shift),
    )
    w = child.get("record_wall_s")
    child["real_time_factor"] = float(time_rel[-1] / w) if w else None


# ------------------------------------------------------------------------------------------------ films, result file, summary
def encode_raw(raw: Path, mp4: Path, w: int, h: int) -> int:
    n = raw.stat().st_size // (w * h * 3)
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(FILM_FPS), "-i", str(raw),
           "-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)]
    subprocess.run(cmd, check=True)
    return n


def layout_json(path: Path):
    outer = pc.OPENING + 2 * pc.BAR
    lay = {
        "title": "PX4 and Gazebo base lap, level 0 gates scaled by 4",
        "units": "meters; angles in radians in the file, degrees on the sheet",
        "limits_box": {"x": [-6, 14], "y": [-9, 5], "z": [0, 8]},
        "level_file_safety_limits": {"pos_limit_low": [-6.0, -9.0, -0.001], "pos_limit_high": [14.0, 5.0, 8.0]},
        "gate_outer_m": outer, "gate_opening_m": pc.OPENING,
        "scale_vs_webots_lap": pc.SCALE,
        "frame": "x east, y north, z up; origin = the spawn point on the ground; z is measured from the height of the drone at rest",
        "start": {"pos": [0.0, 0.0, 0.0], "yaw_rad": 0.0},
        "gates": [{"pos": [float(x) for x in g[:3]], "yaw_rad": float(g[3]), "shape": "square"} for g in pc.GATES],
        "obstacles": [],
    }
    path.write_text(json.dumps(lay, indent=1))


def post(out: Path) -> dict:
    res = {"errors": []}
    child_p = out / "child.json"
    if not child_p.is_file():
        res["errors"].append("no child.json")
        (out / "post.json").write_text(json.dumps(res, indent=2))
        return res
    child = json.loads(child_p.read_text())
    npz = out / "states.npz"
    if not npz.is_file():
        res["errors"].append("no states.npz")
        (out / "post.json").write_text(json.dumps(res, indent=2))
        write_summary(out, child, res)
        return res
    z = {k: v for k, v in np.load(npz, allow_pickle=False).items()}
    try:
        outer = pc.OPENING + 2 * pc.BAR
        fpv_raw, chase_raw = out / "fpv.rgb", out / "chase.rgb"
        n_fpv = encode_raw(fpv_raw, out / "fpv.mp4", WIDTH, HEIGHT) if fpv_raw.is_file() and fpv_raw.stat().st_size else 0
        n_chase = encode_raw(chase_raw, out / "chase.mp4", WIDTH, HEIGHT) if chase_raw.is_file() and chase_raw.stat().st_size else 0
        res["pictures"] = {"fpv": n_fpv, "chase": n_chase, "fpv_rows": int(len(z["pic_step"])), "chase_rows": int(len(z["chase_pic_step"]))}
        if n_fpv and n_fpv == len(z["pic_step"]):
            frames = np.memmap(fpv_raw, dtype=np.uint8, mode="r", shape=(n_fpv, HEIGHT, WIDTH, 3))
            track = {"gates_pos": z["gates_pos"], "gates_quat": z["gates_quat"], "gate_sequence": z["gate_sequence"], "gate_direction": z["gate_direction"],
                     "obstacles_pos": z["obstacles_pos"], "pad_start": tuple(float(v) for v in z["pad_start"]), "pad_end": tuple(float(v) for v in z["pad_end"]),
                     "plan_xy": z["planned_xy"], "total_passages": int(len(z["gate_sequence"])), "gate_outer_m": outer, "gate_opening_m": pc.OPENING}
            res["sync_film"] = sync_film.make_sync_film(frames, z, track, out / "fpv_and_map.mp4", FILM_FPS)
            del frames
        else:
            res["errors"].append(f"fpv pictures {n_fpv} differ from picture rows {len(z['pic_step'])}: no map film")
        for raw in (fpv_raw, chase_raw):
            raw.unlink(missing_ok=True)
        times = {"fpv": z["pic_time"], "chase": z["chase_pic_time"], "fpv_and_map": z["pic_time"]}
        for name, w, h in (("fpv", WIDTH, HEIGHT), ("chase", WIDTH, HEIGHT), ("fpv_and_map", WIDTH + HEIGHT, HEIGHT)):
            mp4 = out / f"{name}.mp4"
            if mp4.is_file():
                res.setdefault("contact", {})[name] = sync_film.film_contact_sheet(mp4, w, h, times[name], out / f"{name}_contact.png")
    except Exception:  # noqa: BLE001
        res["errors"].append("films: " + traceback.format_exc()[-1500:])
    try:
        layout_json(out / "layout.json")
        films = [f"{n}.mp4" for n in ("fpv", "chase", "fpv_and_map") if (out / f"{n}.mp4").is_file()]
        rc = flight_result.main(["--npz", str(npz), "--child", str(child_p), "--layout", str(out / "layout.json"), "--out", str(out / "flight_result.json"),
                                 "--films", *films, "--run-id", "px4_gazebo_base", "--seed", "0", "--date", datetime.date.today().isoformat(),
                                 "--title", "PX4 and Gazebo base lap, level 0 gates scaled by 4"])
        res["flight_result_exit_code"] = rc
    except Exception:  # noqa: BLE001
        res["errors"].append("flight_result: " + traceback.format_exc()[-1500:])
    (out / "post.json").write_text(json.dumps(res, indent=2, default=str))
    write_summary(out, child, res)
    return res


def write_summary(out: Path, child: dict, res: dict) -> None:
    fr = out / "flight_result.json"
    s = json.loads(fr.read_text())["summary"] if fr.is_file() else {}
    L = ["### PX4 + Gazebo base lap", ""]
    L.append(f"- Gates passed: **{child.get('passages')} of {child.get('gates_total')}** (end: {child.get('end')}; flight end reason: {child.get('flight_end_reason')})")
    rs = child.get("route_start_row_time")
    lap = s.get("lap_time_s")
    if lap is not None and rs is not None:
        L.append(f"- Lap time: {round(lap - rs, 3)} s from the start of the route (the file's lap_time_s = {lap} s also counts the takeoff and settling before the route; the route itself is planned for {pc.T_ROUTE} s)")
    else:
        L.append(f"- Lap time: none (lap not finished); flight time {s.get('flight_time_s')} s")
    L.append(f"- Largest tilt: {child.get('max_tilt_deg')} deg (flight_result: {s.get('max_tilt_deg')} deg)")
    L.append(f"- Closest distance of the drone centre to a gate frame bar: {child.get('closest_frame_distance_m')} m, gate {None if child.get('closest_frame_gate') is None else child.get('closest_frame_gate') + 1}, "
             f"at {child.get('closest_frame_time')} s (the drone counts as touching below {child.get('drone_radius_m')} m, the radius with propellers)")
    L.append(f"- Collisions: {child.get('frame_hit_count')} samples closer than {child.get('drone_radius_m')} m to a frame bar {child.get('frame_hits', [])[:5]} (geometric rule from the Gazebo pose; no contact sensor); "
             f"drone tilted over (crash) = {child.get('flight_end_reason') == 'crash'}")
    L.append(f"- Frames per film: fpv {(res.get('pictures') or {}).get('fpv')}, chase {(res.get('pictures') or {}).get('chase')}, fpv_and_map {(res.get('sync_film') or {}).get('pictures_written')} (each film {FILM_FPS} pictures per simulated second, {WIDTH} x {HEIGHT}); "
             f"camera messages seen {child.get('frames_seen')}, kept {child.get('frames_kept')}")
    L.append(f"- Real-time factor while recording (simulated seconds per wall second, both cameras on): {child.get('real_time_factor')}; wall time recording {child.get('record_wall_s')} s; simulated {child.get('sim_seconds')} s")
    L.append(f"- Passages: {child.get('passage_log')}")
    L.append(f"- Track: the four gates of level 0, positions and heights x {pc.SCALE} and shifted so the start is the spawn point; opening {pc.OPENING} m, frame {pc.OPENING + 2 * pc.BAR} m outer")
    L.append(f"- Image problems: {child.get('image_problems')}; pose messages {child.get('pose_messages')} (without time stamp: {child.get('pose_messages_without_stamp')}); model patch {child.get('model_patch')}")
    L.append(f"- Gate create answers: {child.get('gate_service_answers')}")
    L.append(f"- Errors while making the outputs: {res.get('errors')}; traceback of the run: {child.get('traceback')}")
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
    ap.add_argument("--px4", default="px4")
    a = ap.parse_args()
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if a.command == "run":
        for f in ("states.npz", "child.json", "fpv.rgb", "chase.rgb"):
            (out / f).unlink(missing_ok=True)
        run_stack(out, Path(a.px4).resolve())
    post(out)


if __name__ == "__main__":
    main()
