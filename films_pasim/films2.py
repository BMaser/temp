"""Slice 2 feature films of Project AirSim 1.0.1 (client side): F01 F02 F03 F04 F07 F08 F09 F10 F11.

Runs in the Python environment with the `projectairsim` package; the Unreal server is started by launch_film.sh (CLIENT_SCRIPT=films2.py).
The film itself is made afterwards by make_film2.py from capture.json and the saved pictures.

Clock: slowed (real-time-update-rate = step / rtf), NOT paused, as in slice 1 (a paused clock made GetImages time out). Each "pair" is one
step of the loop: send the control, ask for the pictures, read the simulated time and the pose. capture.json holds all of it.

Every simulator call that matters goes through `call` (name, arguments, result, line) like in pasim_films.py.
Exit codes: 0 = pictures taken, 3 = no picture, 4 = no connection, 1 = error.
"""

import argparse
import asyncio
import json
import math
import random
import re
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pasim_films import Calls, log, pose_nedyaw, strip_jsonc  # noqa: E402

SCENE_SHIPPED, ROBOT_SHIPPED = "scene_basic_drone.jsonc", "robot_quadrotor_fastphysics.jsonc"
SCENE_ENV = "scene_env_actor.jsonc"
DAY = "2021-09-20"
RACE = Path(__file__).resolve().parent / "hard_race_v1.json"
NOISE_ON = {"enabled": True, "image-type": 0, "rand-contrib": 0.2, "rand-speed": 100000.0, "rand-size": 500.0, "rand-density": 2,
            "horz-wave-contrib": 0.03, "horz-wave-strength": 0.08, "horz-wave-vert-size": 1.0, "horz-wave-screen-size": 1.0,
            "horz-noise-lines-contrib": 1.0, "horz-noise-lines-density-y": 0.01, "horz-noise-lines-density-xy": 0.5,
            "horz-distortion-contrib": 1.0, "horz-distortion-strength": 0.002}  # the values of the shipped sample configs (enabled: true)


NOISE_STRONG = dict(NOISE_ON, **{"rand-contrib": 1.0, "rand-size": 30.0, "rand-density": 8, "horz-wave-contrib": 0.5, "horz-wave-strength": 0.5,
                                 "horz-noise-lines-density-y": 0.05, "horz-noise-lines-density-xy": 0.9, "horz-distortion-strength": 0.01})  # our exaggerated values


# ------------------------------------------------------------------------------------------------ configuration files
def build_robot(sim_config: Path, name: str, w=640, h=360, interval=0.05, chase=True, types=(0,), noise=None, blur=None, chase_close=True,
                max_depth=40.0) -> dict:
    """robot_<name>.jsonc: FrontCamera on the nose (one capture entry per image type), Chase camera, every other sensor disabled."""
    sim_config = Path(sim_config)
    robot = json.loads(strip_jsonc((sim_config / ROBOT_SHIPPED).read_text()))
    ch = next(s for s in robot["sensors"] if s.get("id") == "Chase")
    base = ch["capture-settings"][0]
    nose = json.loads(json.dumps(ch))
    nose["id"], nose["origin"] = "FrontCamera", {"xyz": "0.3 0.0 0.0", "rpy-deg": "0 0 0"}
    nose.pop("gimbal", None)
    nose["capture-interval"] = interval
    caps = []
    for t in types:
        c = dict(base)
        c.update({"image-type": t, "width": w, "height": h})
        if t == 4:
            c["max-depth-meters"] = max_depth
        if blur is not None and t == 0:
            c["motion-blur-amount"] = blur
        caps.append(c)
    nose["capture-settings"] = caps
    if noise:
        nose["noise-settings"] = [dict(noise)]
    ch["capture-settings"][0].update({"width": w, "height": h})
    ch["capture-interval"] = interval
    if chase_close:
        ch["origin"] = {"xyz": "-3.5 0.0 -1.5", "rpy-deg": "0 -23.0 0"}
    for s in robot["sensors"]:
        if s["id"] not in ("Chase",):
            s["enabled"] = False
    ch["enabled"] = bool(chase)
    robot["sensors"].append(nose)
    (sim_config / f"robot_{name}.jsonc").write_text(json.dumps(robot, indent=2))
    return {"robot_file": f"robot_{name}.jsonc", "picture_size": [w, h], "types": list(types), "noise": bool(noise), "blur": blur,
            "chase": chase, "capture_interval_s": interval}


def build_scene(sim_config: Path, name: str, robot_name: str, rtf: float, base=SCENE_SHIPPED, origin=None, yaw_deg=None, wind=None) -> dict:
    sim_config = Path(sim_config)
    scene = json.loads(strip_jsonc((sim_config / base).read_text()))
    for actor in scene["actors"]:
        if actor.get("type") == "robot":
            actor["robot-config"] = f"robot_{robot_name}.jsonc"
            if origin:
                actor["origin"]["xyz"] = " ".join(str(v) for v in origin)
            if yaw_deg is not None:
                actor["origin"]["rpy-deg"] = f"0 0 {yaw_deg}"
    step_ns = int(scene["clock"]["step-ns"])
    scene["clock"]["real-time-update-rate"] = int(round(step_ns / rtf))
    scene["clock"]["pause-on-start"] = False
    if wind is not None:
        scene["wind"] = {"velocity": " ".join(str(v) for v in wind)}
    (sim_config / f"scene_{name}.jsonc").write_text(json.dumps(scene, indent=2))
    return {"scene_file": f"scene_{name}.jsonc", "step_ns": step_ns, "update_rate_ns": scene["clock"]["real-time-update-rate"], "rtf_wanted": rtf,
            "actors": [(a.get("name"), a.get("origin")) for a in scene["actors"]], "env_actors": [a.get("name") for a in scene.get("environment-actors", [])]}


# ------------------------------------------------------------------------------------------------ session
class Sess:
    """One connection, one loaded scene, helper loops. All pictures and records are kept in self.cap."""

    def __init__(self, a, out: Path):
        self.a, self.out = a, out
        self.frames_dir = out / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.cap = {"film": a.film, "variant": a.variant, "stage": "start", "frames": [], "phases": [], "traceback": None, "args": vars(a)}
        self.call = Calls()
        self.idx = 0
        self.fails = {}
        self.t_wall0 = time.perf_counter()

    def save(self):
        self.cap["api_calls"] = self.call.records
        (self.out / "capture.json").write_text(json.dumps(self.cap, indent=1, default=str))

    async def connect(self):
        from projectairsim import ProjectAirSimClient
        self.client = ProjectAirSimClient()
        t0 = time.time()
        while True:
            try:
                self.client.connect()
                break
            except Exception as e:  # noqa: BLE001
                if time.time() - t0 > self.a.connect_wait:
                    self.cap["stage"], self.cap["error"] = "no_connection", repr(e)
                    self.save()
                    raise SystemExit(4)
                time.sleep(3)
        self.cap["connected_after_s"] = round(time.time() - t0, 1)

    def load(self, scene_file: str):
        from projectairsim import Drone, World
        t = time.time()
        self.world = World(self.client, scene_file, delay_after_load_sec=2, sim_config_path=self.a.sim_config)
        self.drone = Drone(self.client, self.world, "Drone1")
        self.cap.setdefault("scene_loads", []).append({"scene": scene_file, "seconds": round(time.time() - t, 1)})
        self.drone.enable_api_control()
        self.drone.arm()

    def grab(self, sensor: str, types):
        """Pictures of one camera: list of numpy arrays (RGB, or 2-D for single-channel), in the order of `types`."""
        import numpy as np
        from projectairsim.types import ImageType
        from projectairsim.utils import unpack_image
        res = self.drone.get_images(sensor, [ImageType(t) for t in types])
        vals = list(res.values())
        order = sorted(range(len(types)), key=lambda i: types[i])          # the reply holds the images sorted by type id (measured, F11 run 1)
        if len(vals) == len(types):
            vals = [vals[order.index(i)] for i in range(len(types))]
        if "image_keys" not in self.cap:
            self.cap["image_keys"] = [str(k) for k in res.keys()]
        out = []
        for v in vals:
            arr = np.asarray(unpack_image(v))
            if arr.ndim == 3 and arr.shape[2] >= 3:
                arr = arr[:, :, :3][:, :, ::-1]
            out.append(np.ascontiguousarray(arr))
        return out

    async def warmup(self, sensors):
        """Ask until every camera gives a picture that is not flat (the first Chase picture is black)."""
        got, t1 = {}, time.time()
        while time.time() - t1 < self.a.picture_wait and len(got) < len(sensors):
            for s, types in sensors.items():
                if s in got:
                    continue
                try:
                    imgs = self.grab(s, types)
                    if float(imgs[0].std()) > 3.0:
                        got[s] = True
                except Exception as e:  # noqa: BLE001
                    self.cap.setdefault("warmup_errors", []).append(f"{s}: {e!r}"[-160:])
            time.sleep(1)
        self.cap["warmup"] = {"got": sorted(got), "seconds": round(time.time() - t1, 1)}
        return got

    async def takeoff(self, wait_wall=300.0):
        sim0 = self.world.get_sim_time()
        take = await self.drone.takeoff_async()
        t = time.perf_counter()
        while not take.done() and time.perf_counter() - t < wait_wall and (self.world.get_sim_time() - sim0) / 1e9 < 40:
            await asyncio.sleep(0.5)
        pose = pose_nedyaw(self.drone.get_ground_truth_pose())
        self.cap["takeoff"] = {"wall_s": round(time.perf_counter() - t, 1), "pose_ned_yaw": list(pose)}
        return pose

    def pose(self):
        return pose_nedyaw(self.drone.get_ground_truth_pose())

    async def run_phase(self, phase: str, n: int, control, cams: dict, record_from: int = 0, extra=None):
        """n pairs. control(k, rec) is an async function that sends the commands for this pair (may fill rec). cams = {key: (sensor, [types])}.
        Pairs with k < record_from are taken but not saved (settle). Returns the records of the saved pairs."""
        from PIL import Image
        import numpy as np
        ph = {"phase": phase, "pairs": n, "first_frame": self.idx, "wall_start_s": round(time.perf_counter() - self.t_wall0, 1)}
        sim_start = self.world.get_sim_time()
        ph["sim_start_s"] = sim_start / 1e9
        saved = []
        for k in range(n):
            if time.perf_counter() - self.t_wall0 > self.a.max_wall:
                self.cap["stop_reason"] = f"wall time limit in phase {phase} at pair {k}"
                break
            rec = {"i": self.idx, "phase": phase, "k": k, "settle": k < record_from}
            await control(k, rec)
            for key, (sensor, types) in cams.items():
                try:
                    imgs = self.grab(sensor, types)
                except Exception as ex:  # noqa: BLE001
                    self.fails[key] = self.fails.get(key, 0) + 1
                    if self.fails[key] <= 5:
                        log("picture failed", key, repr(ex)[-160:])
                    continue
                for t, img in zip(types, imgs):
                    nm = f"{key}{'' if t == 0 else '_t%d' % t}"
                    rec[f"{nm}_mean"], rec[f"{nm}_std"] = round(float(img.mean()), 2), round(float(img.std()), 2)
                    if k >= record_from:
                        fn = f"{nm}_{self.idx:05d}.{'png' if t in (3, 4, 1, 2) else 'jpg'}"
                        im = img if img.ndim == 3 else np.stack([img] * 3, axis=-1) if img.dtype == np.uint8 else img
                        if im.dtype != np.uint8:
                            im = np.clip(im / max(float(im.max()), 1e-6) * 255, 0, 255).astype(np.uint8)
                            if im.ndim == 2:
                                im = np.stack([im] * 3, axis=-1)
                        Image.fromarray(im).save(self.frames_dir / fn, **({"quality": 92} if fn.endswith("jpg") else {}))
                        rec[nm] = fn
            rec["t"] = (self.world.get_sim_time() - sim_start) / 1e9 + ph["sim_start_s"] * 0
            rec["t_abs"] = self.world.get_sim_time() / 1e9
            n_, e_, d_, yw = self.pose()
            rec.update({"n": n_, "e": e_, "d": d_, "yaw": yw, "wall_s": round(time.perf_counter() - self.t_wall0, 2)})
            if extra:
                rec.update(extra(k) or {})
            self.cap["frames"].append(rec)
            if not rec["settle"]:
                saved.append(rec)
            self.idx += 1
            if self.idx % 10 == 0:
                log(f"{phase} pair {k}/{n} t_sim={rec['t_abs']:.1f}s wall={rec['wall_s']:.0f}s fails={self.fails}")
                self.save()
        ph.update({"last_frame": self.idx - 1, "wall_end_s": round(time.perf_counter() - self.t_wall0, 1),
                   "sim_end_s": round(self.world.get_sim_time() / 1e9, 3)})
        self.cap["phases"].append(ph)
        return saved


def make_pose(x, y, z, yaw=0.0, pitch=0.0, roll=0.0):
    from projectairsim.types import Pose, Quaternion, Vector3
    from projectairsim.utils import rpy_to_quaternion
    q = rpy_to_quaternion(roll, pitch, yaw)
    return Pose({"translation": Vector3({"x": x, "y": y, "z": z}), "rotation": Quaternion({"w": q[0], "x": q[1], "y": q[2], "z": q[3]}),
                 "frame_id": "DEFAULT_ID"})


def solid_png(path: Path, rgb, noise=18, size=64, seed=1):
    """A PNG of one colour with luminance noise at two scales (texture for spawned primitives; generated by our code)."""
    import numpy as np
    from PIL import Image
    rng = np.random.default_rng(seed)
    fine = rng.normal(0, noise, (size, size))
    coarse = np.kron(rng.normal(0, noise * 1.2, (max(2, size // 16), max(2, size // 16))), np.ones((16, 16)))[:size, :size]
    lum = (fine * 0.5 + coarse)[:, :, None]
    a = np.clip(np.array(rgb, dtype=float)[None, None, :] * (1 + lum / 120.0), 0, 255).astype(np.uint8)
    Image.fromarray(a).save(path)
    return str(path)


def explore(s: Sess):
    """Scene objects, assets, ground height: the facts the film notes need."""
    w, ex = s.world, {}
    for nm, fn in (("objects", lambda: w.list_objects(".*")), ("assets", lambda: w.list_assets(".*")), ("time_of_day", w.get_time_of_day)):
        try:
            v = fn()
            ex[nm] = [str(i) for i in v][:800] if nm != "time_of_day" else v
            if nm != "time_of_day":
                ex[nm + "_count"] = len(v)
        except Exception as e:  # noqa: BLE001
            ex[nm] = f"error: {type(e).__name__}: {e}"[:200]
    s.cap["scene_exploration"] = ex
    return ex


def facing_arc_positions(n0, e0, heading, count, r_min, r_max, spread_deg, seed=3):
    rnd = random.Random(seed)
    out = []
    for i in range(count):
        ang = heading + math.radians(rnd.uniform(-spread_deg, spread_deg))
        r = rnd.uniform(r_min, r_max)
        out.append((n0 + r * math.cos(ang), e0 + r * math.sin(ang)))
    return out


# ------------------------------------------------------------------------------------------------ films
def hover_swing(s, yaw0, d0, amp_deg, n_pairs):
    """control(): hover at d0, nose swings +-amp around yaw0 over n_pairs pairs."""
    from projectairsim.drone import YawControlMode

    async def control(k, rec):
        yaw = yaw0 + math.radians(amp_deg) * math.sin(2 * math.pi * k / max(1, n_pairs))
        await s.drone.move_by_velocity_z_async(0.0, 0.0, d0, 2.0, YawControlMode.MaxDegreeOfFreedom, False, yaw)
    return control


def forward_flight(s, yaw, d0, speed, ve_hold=None, e_ref=None):
    from projectairsim.drone import YawControlMode

    async def control(k, rec):
        vn, ve = speed * math.cos(yaw), speed * math.sin(yaw)
        await s.drone.move_by_velocity_z_async(vn, ve, d0, 2.0, YawControlMode.MaxDegreeOfFreedom, False, yaw)
    return control


def move_floor(s, floor, dz):
    w = s.world
    p = w.get_object_pose(floor)
    t = p["translation"] if isinstance(p, dict) else p.translation
    x, y, z = (float(t["x"]), float(t["y"]), float(t["z"])) if isinstance(t, dict) else (float(t.x), float(t.y), float(t.z))
    s.call(f"world.set_object_pose({floor[:30]}, z+{dz})", w.set_object_pose, floor, make_pose(x, y, z + dz), True)
    return (x, y, z)


async def film_nature(s, a, kind):
    """F01 (landscape) and F02 (city/neighbourhood): forward flight with nose camera (and chase camera unless variant 'light').
    F01 variants: hide (move the floor away), grass (retexture the StaticMeshActor), cover (a large spawned plane with a generated grass picture laid
    over the checkerboard), landtex (the grass picture on every Landscape object); all add trees, rocks and bushes built from the packaged primitives."""
    flags = set(a.variant.split("_"))
    s.load(f"scene_{kind}.jsonc")
    ex = explore(s)
    has_chase = "light" not in flags
    cams = {"fpv": ("FrontCamera", [0])}
    sens = {"FrontCamera": [0]}
    if has_chase:
        cams["chase"] = ("Chase", [0])
        sens["Chase"] = [0]
    await s.warmup(sens)
    n0, e0, d0, yaw0 = await s.takeoff()
    notes = {"flags": sorted(flags)}
    if kind == "F01":
        objs = [o for o in ex.get("objects", []) if isinstance(o, str)]
        floor = next((o for o in objs if o.startswith("StaticMeshActor")), None)
        notes["floor_object"] = floor
        try:
            notes["ground_elevation_at_start_m"] = s.world.get_surface_elevation_at_point(n0, e0)
        except Exception as e:  # noqa: BLE001
            notes["ground_elevation_error"] = repr(e)[:150]
        ground_z = d0 + 1.5
        tex_grass = solid_png(s.out / "grass.png", (70, 120, 45), noise=26, size=1024, seed=5)
        if floor and "hide" in flags:
            notes["floor_original_xyz"] = move_floor(s, floor, 800.0)
        elif floor and "grass" in flags:
            s.call("world.set_object_texture_from_file(floor, grass.png)", s.world.set_object_texture_from_file, floor, tex_grass)
        if "landtex" in flags:
            land = [o for o in objs if o.startswith("Landscape")]
            ok = 0
            for o in land:
                r = s.call(f"world.set_object_texture_from_file({o[:34]})", s.world.set_object_texture_from_file, o, tex_grass)
                ok += 1 if r["ok"] and s.call.last_value else 0
            notes["landscape_objects_textured"] = [ok, len(land)]
        if "cover" in flags:
            ok = spawn_prim(s, "GrassCover", "Plane", (n0 + 60.0, e0, ground_z - 0.03), (240.0, 240.0, 0.05), 0.0, tex_grass)
            notes["cover_plane_spawned"] = ok
        tex_g = solid_png(s.out / "leaf.png", (30, 105, 35), noise=22, size=128, seed=2)
        tex_b = solid_png(s.out / "bark.png", (90, 60, 35), noise=16, size=128, seed=3)
        tex_r = solid_png(s.out / "rock.png", (120, 118, 112), noise=22, size=128, seed=4)
        for i, (pn, pe) in enumerate(facing_arc_positions(n0, e0, yaw0, 14, 5.0, 24.0, 55.0)):
            h = 4.0 + (i % 3) * 1.5
            spawn_prim(s, f"Trunk{i}", "Cylinder", (pn, pe, ground_z - h / 2), (0.4, 0.4, h), 0.0, tex_b)
            spawn_prim(s, f"Crown{i}", "Sphere", (pn, pe, ground_z - h - 0.8), (3.0, 3.0, 3.4), 0.0, tex_g)
        for i, (pn, pe) in enumerate(facing_arc_positions(n0, e0, yaw0, 10, 3.0, 14.0, 60.0, seed=9)):
            spawn_prim(s, f"Rock{i}", "Sphere", (pn, pe, ground_z - 0.4), (2.0 + 0.3 * (i % 3), 1.5, 1.0), 0.0, tex_r)
            spawn_prim(s, f"Bush{i}", "Sphere", (pn + 1.5, pe + 1.0, ground_z - 0.5), (1.8, 1.8, 1.2), 0.0, tex_g)
        notes["spawned"] = {"trees": 14, "rocks": 10, "bushes": 10, "primitives": "Cylinder (trunk), Sphere (crown, rock, bush), textures generated by our code"}
    s.cap["nature_notes"] = notes
    s.cap["stage"] = "film"
    s.save()
    ctl = forward_flight(s, yaw0, d0, a.speed)
    await s.run_phase("fly", a.frames, ctl, cams, record_from=1)


async def film_moving(s, a):
    """F03: two environment actors (other drones) that fly their own trajectories from the scene file, seen from the nose camera."""
    s.load("scene_F03.jsonc")
    explore(s)
    cams = {"fpv": ("FrontCamera", [0]), "chase": ("Chase", [0])}
    await s.warmup({"FrontCamera": [0], "Chase": [0]})
    n0, e0, d0, yaw0 = await s.takeoff()
    names = ["ActorWithConfigTraj", "TiltrotorWithConfigTrajOffset"]
    ctl = hover_swing(s, 0.0, d0, 12.0, a.frames)

    def extra(k):
        out = {}
        for nm in names:
            try:
                p = s.world.get_object_pose(nm)
                t = p["translation"] if isinstance(p, dict) else p.translation
                g = (lambda key: float(t[key])) if isinstance(t, dict) else (lambda key: float(getattr(t, key)))
                out[nm] = [g("x"), g("y"), g("z")]
            except Exception as e:  # noqa: BLE001
                out[nm] = None
                s.cap.setdefault("actor_pose_errors", []).append(repr(e)[:120])
        return out
    s.cap["stage"] = "film"
    await s.run_phase("moving", a.frames, ctl, cams, record_from=1, extra=extra)


async def film_sun(s, a):
    """F04: sun low in the sky, the nose swings a full turn (glare when it faces the sun), shadows; then a spawned water plane (reflections)."""
    s.load("scene_F04.jsonc")
    explore(s)
    cams = {"fpv": ("FrontCamera", [0]), "chase": ("Chase", [0])}
    await s.warmup({"FrontCamera": [0], "Chase": [0]})
    n0, e0, d0, yaw0 = await s.takeoff()
    from projectairsim.drone import YawControlMode
    s.call("world.set_time_of_day(17:00)", s.world.set_time_of_day, status=True, datetime=f"{DAY} 17:00:00", is_dst=False, clock_speed=1.0, update_interval=1.0, move_sun=False)
    s.call("world.set_sunlight_intensity(2.75)", s.world.set_sunlight_intensity, 2.75)

    def turn(n):
        async def control(k, rec):
            yaw = yaw0 + 2 * math.pi * k / n
            await s.drone.move_by_velocity_z_async(0.0, 0.0, d0, 2.0, YawControlMode.MaxDegreeOfFreedom, False, yaw)
        return control
    s.cap["stage"] = "film"
    await s.run_phase("shadows_and_glare", a.frames, turn(a.frames), cams, record_from=1)
    try:
        s.cap["ground_pose_before"] = repr(s.world.get_object_pose("Ground"))[:200]
        s.call("world.set_object_pose(Ground, +5 m down)", s.world.set_object_pose, "Ground", make_pose(0.0, 0.0, 5.0), True)
    except Exception as e:  # noqa: BLE001
        s.cap["ground_move_error"] = repr(e)[:200]
    z_up = 0.4
    s.call(f"world.spawn_object(Water1, WaterPlane10km) {z_up} m above the floor", s.world.spawn_object, "Water1", "WaterPlane10km", make_pose(n0, e0, d0 + 3.7 - z_up), [1.0, 1.0, 1.0], False)
    await s.run_phase("water", a.frames, turn(a.frames), cams, record_from=1)


async def film_wind(s, a):
    """F07: the same flight three times: no wind, world.set_wind_velocity (the simulator's own wind), and wind made by our external force."""
    from projectairsim.drone import YawControlMode
    s.load("scene_F07.jsonc")
    explore(s)
    cams = {"fpv": ("FrontCamera", [0])}
    await s.warmup({"FrontCamera": [0]})
    n0, e0, d0, yaw0 = await s.takeoff()
    start = make_pose(n0, e0, d0, yaw0)
    WIND = (0.0, 5.0, 0.0)  # m/s, NED: 5 m/s toward the east, across the flight direction (north)
    C_DRAG = 0.5            # N per (m/s): our wind as a linear drag toward the wind velocity (donor idea: wind.py of the shipped examples, set_external_force)

    def flight(mode):
        state = {}

        async def control(k, rec):
            if mode == "force":
                n_, e_, _, _ = s.pose()
                t_ = s.world.get_sim_time() / 1e9
                last = state.get("last")
                vx, vy = ((n_ - last[0]) / max(t_ - last[2], 1e-3), (e_ - last[1]) / max(t_ - last[2], 1e-3)) if last else (0.0, 0.0)
                state["last"] = (n_, e_, t_)
                fx, fy = C_DRAG * (WIND[0] - vx), C_DRAG * (WIND[1] - vy)
                s.drone.set_external_force([fx, fy, 0.0])
                rec["ext_force"] = [fx, fy]
            await s.drone.move_by_velocity_z_async(a.speed, 0.0, d0, 2.0, YawControlMode.MaxDegreeOfFreedom, False, yaw0)
        return control

    s.cap["stage"] = "film"
    for mode in ("off", "wind_api", "force"):
        try:
            s.call("drone.set_external_force([0,0,0])", s.drone.set_external_force, [0.0, 0.0, 0.0])
            s.call("world.set_wind_velocity(%s)" % (WIND if mode == "wind_api" else (0, 0, 0)), s.world.set_wind_velocity, *(WIND if mode == "wind_api" else (0.0, 0.0, 0.0)))
            s.call("drone.set_pose(start)", s.drone.set_pose, start, True)
        except Exception as e:  # noqa: BLE001
            s.cap.setdefault("reset_errors", []).append(repr(e)[:200])
        await asyncio.sleep(1.0)
        await s.run_phase(mode, a.frames + 3, flight(mode), cams, record_from=3)
        s.save()
    s.call("world.set_wind_velocity(0,0,0)", s.world.set_wind_velocity, 0.0, 0.0, 0.0)
    s.call("drone.set_external_force([0,0,0])", s.drone.set_external_force, [0.0, 0.0, 0.0])


async def film_disturb(s, a):
    """F08: hold a point; 20 pairs without disturbance, then random force every pair (Crazyflow's disturbance_fn idea: zero-mean Gaussian force)."""
    from projectairsim.drone import YawControlMode
    s.load("scene_F08.jsonc")
    explore(s)
    cams = {"fpv": ("FrontCamera", [0]), "chase": ("Chase", [0])}
    await s.warmup({"FrontCamera": [0], "Chase": [0]})
    n0, e0, d0, yaw0 = await s.takeoff()
    rnd = random.Random(7)
    # donor: crazyflow examples/plugins/disturbance.py (MIT): force = normal(0, 0.2 N) every simulation step on a ~0.027 kg Crazyflie.
    # Scaled to this 1.0 kg drone by the mass ratio and divided by sqrt(hold time / crazyflow step 0.002 s) because we hold each sample 0.1 s (not one step).
    sigma = 0.2 * (1.0 / 0.027) / math.sqrt(0.1 / 0.002)
    s.cap["disturbance"] = {"donor": "crazyflow examples/plugins/disturbance.py (MIT license)", "std_force_N_each_axis": round(sigma, 3),
                            "formula": "0.2 N * (1.0 kg / 0.027 kg) / sqrt(0.1 s / 0.002 s)", "torque": "not ported: Project AirSim has set_external_force only (no torque call found)"}

    def hold(disturbed):
        async def control(k, rec):
            n, e, d, yw = s.pose()
            vn, ve = max(-2.0, min(2.0, -1.0 * (n - n0))), max(-2.0, min(2.0, -1.0 * (e - e0)))
            if disturbed:
                f = [rnd.gauss(0, sigma), rnd.gauss(0, sigma), rnd.gauss(0, sigma)]
                s.drone.set_external_force(f)
                rec["ext_force"] = f
            await s.drone.move_by_velocity_z_async(vn, ve, d0, 2.0, YawControlMode.MaxDegreeOfFreedom, False, yaw0)
        return control
    s.cap["stage"] = "film"
    s.cap["hold_point_ned"] = [n0, e0, d0]
    await s.run_phase("no_disturbance", a.frames // 4, hold(False), cams, record_from=1)
    await s.run_phase("random_force", a.frames, hold(True), cams)
    s.call("drone.set_external_force([0,0,0])", s.drone.set_external_force, [0.0, 0.0, 0.0])


async def film_ab(s, a, what):
    """F09 (camera noise) and F10 (motion blur): the same motion twice, the feature off and on (two scene loads, the setting is in the camera config)."""
    from projectairsim.drone import YawControlMode
    cams = {"fpv": ("FrontCamera", [0])}
    runs = [("off", f"scene_{what}_off.jsonc"), ("on", f"scene_{what}_on.jsonc")]
    for label, scene in runs:
        s.load(scene)
        if label == "off":
            explore(s)
        await s.warmup({"FrontCamera": [0]})
        n0, e0, d0, yaw0 = await s.takeoff()
        s.cap["stage"] = "film"
        if what == "F09":
            ctl = hover_swing(s, yaw0, d0, 40.0, a.frames)
        elif "spin" in a.variant.split("_"):
            async def ctl(k, rec):  # a fast turn on the spot: 3 rad/s (172 deg/s)
                await s.drone.move_by_velocity_z_async(0.0, 0.0, d0, 2.0, YawControlMode.MaxDegreeOfFreedom, True, 3.0)
        else:
            ctl = forward_flight(s, yaw0 - math.radians(45), d0, a.speed)  # a fast pass along the blocks
        await s.run_phase(label, a.frames, ctl, cams, record_from=1)
        s.save()


# ------------------------------------------------------------------------------------------------ F11
def race_geometry(scale: float, n_origin, e_origin):
    """The hard_race_v1.json course in NED metres. x forward = north, y left = west (NED east = -y), z up -> down = -z."""
    r = json.loads(RACE.read_text())

    def ned(x, y, z):
        return (n_origin + x * scale, e_origin - y * scale, -z * scale)

    def yaw_ned(deg):  # counter-clockwise (ENU) -> clockwise from north (NED) in radians
        return -math.radians(deg)
    gates = [{"id": g["id"], "center": ned(*g["center"]), "yaw": yaw_ned(g["yaw_deg"]), "center_enu": g["center"], "yaw_deg_enu": g["yaw_deg"]} for g in r["gates"]]
    obst = [{"id": o["id"], "kind": o["kind"], "xy": ned(o["center_xy"][0], o["center_xy"][1], 0)[:2], "radius": o.get("radius", 0) * scale,
             "size": o.get("size_xy", 0) * scale, "height": o["height"] * scale} for o in r["obstacles"]]
    # waypoints
    pts = [(0.0, 0.0, 0.8)]
    det = r["waypoints_hint"]["detours"]
    for g in r["gates"]:
        yaw = math.radians(g["yaw_deg"])
        cx, cy, cz = g["center"]
        for off in (-0.35, 0.0, 0.35):
            pts.append((cx + off * math.cos(yaw), cy + off * math.sin(yaw), cz))
        key = "after_" + g["id"]
        if key in det:
            pts.append(tuple(det[key]))
    pts.append((r["end_pad"][0], r["end_pad"][1], 0.8))
    wp = [ned(*p) for p in pts]
    return {"gates": gates, "obstacles": obst, "waypoints": wp, "scale": scale, "opening": r["gate_opening_m"] * scale, "outer": r["gate_frame_outer_m"] * scale}


def spawn_prim(s, name, asset, center, size_xyz, yaw, tex, pitch=0.0):
    """Spawn a packaged primitive and give it a wanted size in metres: the base size is measured with get_3d_bounding_box, then the scale is set."""
    w = s.world
    r = s.call(f"world.spawn_object({name}, {asset})", w.spawn_object, name, asset, make_pose(center[0], center[1], center[2], yaw, pitch), [1.0, 1.0, 1.0], False)
    if not r["ok"]:
        return False
    base = bbox_extent(s, name)
    scale = [size_xyz[i] / base[i] if base and base[i] > 1e-6 else size_xyz[i] for i in range(3)]
    s.call(f"world.set_object_scale({name}, {[round(v, 3) for v in scale]})", w.set_object_scale, name, scale)
    if tex:
        s.call(f"world.set_object_texture_from_file({name})", w.set_object_texture_from_file, name, tex)
    return True


def spawn_course(s, geo, tex_gate, tex_pole, tex_box):
    """Gates (4 bars each), 2 poles, 1 box from packaged primitives. The base size of each asset is measured with get_3d_bounding_box
    and the scale is corrected so that the object has the wanted size in metres."""
    w = s.world
    sc = geo["scale"]
    bar = 0.1 * sc  # bar thickness 0.4 m at scale 4
    opening, outer = geo["opening"], geo["outer"]
    made = []

    def spawn(name, asset, center, size_xyz, yaw, tex):
        ok = spawn_prim(s, name, asset, center, size_xyz, yaw, tex)
        if ok:
            made.append(name)
        return ok
    for g in geo["gates"]:
        cx, cy, cz = g["center"]
        yaw = g["yaw"]
        f = (math.cos(yaw), math.sin(yaw))          # forward (north, east)
        left = (-math.sin(yaw), math.cos(yaw))      # to the left of the direction of travel (north, east) components
        half = outer / 2
        # NED: z down. top bar above the centre: z = cz - (opening/2 + bar/2)
        gid = g["id"]
        spawn(f"Gate{gid}_top", "1M_Cube", (cx, cy, cz - (opening / 2 + bar / 2)), (bar, outer, bar), yaw, tex_gate)
        spawn(f"Gate{gid}_bot", "1M_Cube", (cx, cy, cz + (opening / 2 + bar / 2)), (bar, outer, bar), yaw, tex_gate)
        for sgn, nm in ((1, "L"), (-1, "R")):
            spawn(f"Gate{gid}_{nm}", "1M_Cube", (cx + sgn * left[0] * (opening / 2 + bar / 2), cy + sgn * left[1] * (opening / 2 + bar / 2), cz), (bar, bar, opening), yaw, tex_gate)
    for o in geo["obstacles"]:
        if o["kind"].startswith("vertical"):
            spawn(f"Pole_{o['id']}", "Cylinder", (o["xy"][0], o["xy"][1], -o["height"] / 2), (2 * o["radius"], 2 * o["radius"], o["height"]), 0.0, tex_pole)
        else:
            spawn(f"Box_{o['id']}", "1M_Cube", (o["xy"][0], o["xy"][1], -o["height"] / 2), (o["size"], o["size"], o["height"]), 0.0, tex_box)
    s.call("world.set_segmentation_id_by_name(Gate.*, 10)", w.set_segmentation_id_by_name, "Gate.*", 10, True, True)
    s.call("world.set_segmentation_id_by_name(Pole_.*, 20)", w.set_segmentation_id_by_name, "Pole_.*", 20, True, True)
    s.call("world.set_segmentation_id_by_name(Box_.*, 30)", w.set_segmentation_id_by_name, "Box_.*", 30, True, True)
    return made


def bbox_extent(s, name):
    """(dx, dy, dz) of an object's bounding box in metres, or None if the reply cannot be read (the raw reply is kept in capture.json)."""
    try:
        from projectairsim.types import BoxAlignment
        al = getattr(BoxAlignment, "OBJECT_ALIGNED", None)
        if al is None:
            al = list(BoxAlignment)[-1]
        bb = s.world.get_3d_bounding_box(name, al)
        s.cap.setdefault("bbox_raw", {})[name] = repr(bb)[:400]

        def v3(d):
            return (float(d["x"]), float(d["y"]), float(d["z"])) if isinstance(d, dict) else (float(d.x), float(d.y), float(d.z))
        if isinstance(bb, dict):
            for ka, kb in (("min", "max"), ("min_point", "max_point")):
                if ka in bb and kb in bb:
                    a_, b_ = v3(bb[ka]), v3(bb[kb])
                    return tuple(abs(b_[i] - a_[i]) for i in range(3))
            if "size" in bb:
                return v3(bb["size"])
            for k in ("extents", "extent", "half_extents"):
                if k in bb:
                    e = v3(bb[k])
                    return tuple(2 * x for x in e) if k != "extents" else e
            if "center" in bb and "size" in bb:
                return v3(bb["size"])
    except Exception as e:  # noqa: BLE001
        s.cap.setdefault("bbox_errors", []).append(f"{name}: {e!r}"[:200])
    return None


class PathFollower:
    """Pure pursuit along a polyline (NED metres): velocity command toward a point `look` metres ahead of the closest point; speed `v`."""

    def __init__(self, pts, v, look):
        self.pts, self.v, self.look = pts, v, look
        self.seg = [math.dist(a, b) for a, b in zip(pts[:-1], pts[1:])]
        self.cum = [0.0]
        for d in self.seg:
            self.cum.append(self.cum[-1] + d)
        self.s = 0.0
        self.total = self.cum[-1]

    def point_at(self, s_):
        s_ = max(0.0, min(self.total, s_))
        for i in range(len(self.seg)):
            if s_ <= self.cum[i + 1] or i == len(self.seg) - 1:
                t = (s_ - self.cum[i]) / max(self.seg[i], 1e-9)
                return tuple(self.pts[i][j] + t * (self.pts[i + 1][j] - self.pts[i][j]) for j in range(3))

    def command(self, pos):
        # advance the progress to the closest point ahead (never backwards)
        best, best_s = 1e9, self.s
        for k in range(61):
            cand = self.s + k * 0.1
            d = math.dist(self.point_at(cand), pos)
            if d < best:
                best, best_s = d, cand
        self.s = min(self.total, best_s)
        tgt = self.point_at(self.s + self.look)
        vec = [tgt[i] - pos[i] for i in range(3)]
        dist = math.sqrt(sum(c * c for c in vec))
        sp = min(self.v, 1.2 * dist)
        return [c / max(dist, 1e-9) * sp for c in vec], self.s >= self.total - 0.5


async def film_race(s, a):
    from projectairsim.drone import YawControlMode
    cams = {"fpv": ("FrontCamera", [0, 4, 3]), "chase": ("Chase", [0])}
    s.load("scene_F11.jsonc")
    ex = explore(s)
    await s.warmup({"FrontCamera": [0]})
    n0, e0, d0, yaw0 = await s.takeoff()
    scale = a.scale
    geo = race_geometry(scale, n0, e0)
    s.cap["race"] = {"scale": scale, "origin_ned": [n0, e0], "geometry": geo}
    # clear the arena: move the blocks, cones and cylinders of the Blocks scene far away (set_object_pose), keep the Ground
    moved = 0
    for o in ex.get("objects", []):
        if isinstance(o, str) and re.match(r"(TemplateCube|Cone|Cylinder|OrangeBall)", o):
            try:
                p = s.world.get_object_pose(o)
                t = p["translation"] if isinstance(p, dict) else p.translation
                x, y, z = (float(t["x"]), float(t["y"]), float(t["z"])) if isinstance(t, dict) else (float(t.x), float(t.y), float(t.z))
                s.world.set_object_pose(o, make_pose(x, y, z + 3000.0), True)
                moved += 1
            except Exception as e:  # noqa: BLE001
                s.cap.setdefault("clear_errors", []).append(f"{o}: {e!r}"[:150])
    s.cap["objects_moved_away"] = moved
    tex_g = solid_png(s.out / "gate.png", (230, 90, 20), noise=10, seed=11)
    tex_p = solid_png(s.out / "pole.png", (240, 220, 40), noise=10, seed=12)
    tex_b = solid_png(s.out / "box.png", (40, 90, 200), noise=10, seed=13)
    s.cap["course_objects"] = spawn_course(s, geo, tex_g, tex_p, tex_b)
    # flight: climb to the first waypoint height, then follow the path
    pf = PathFollower([(n0, e0, d0)] + geo["waypoints"][1:], a.speed, 1.6 * scale / 2)
    s.cap["stage"] = "film"
    yaw_state = {"yaw": yaw0}

    async def control(k, rec):
        n, e, d, yw = s.pose()
        v, done = pf.command((n, e, d))
        sp = math.hypot(v[0], v[1])
        if sp > 0.3:
            want = math.atan2(v[1], v[0])
            diff = (want - yaw_state["yaw"] + math.pi) % (2 * math.pi) - math.pi
            yaw_state["yaw"] += max(-0.35, min(0.35, diff))   # at most 0.35 rad per pair
        rec["s_along"] = pf.s
        rec["done"] = done
        await s.drone.move_by_velocity_async(v[0], v[1], v[2], 2.0, YawControlMode.MaxDegreeOfFreedom, False, yaw_state["yaw"])
    await s.run_phase("race", a.frames, control, cams, record_from=1)
    s.cap["path_progress_m"] = {"total": pf.total, "reached": pf.s}


# ------------------------------------------------------------------------------------------------ main
async def amain(a) -> int:
    out = Path(a.out)
    s = Sess(a, out)
    simcfg = Path(a.sim_config)
    f = a.film
    w, h = a.width, a.height
    rtf = a.rtf
    try:
        if f in ("F01", "F02"):
            light = "light" in a.variant.split("_")
            cfg = [build_robot(simcfg, f, 320 if light else w, 180 if light else h, chase=not light), build_scene(simcfg, f, f, rtf, yaw_deg=0)]
        elif f == "F03":
            cfg = [build_robot(simcfg, f, w, h), build_scene(simcfg, f, f, rtf, base=SCENE_ENV, origin=(-8.0, 10.0, -4.0), yaw_deg=0)]
        elif f in ("F04", "F07", "F08"):
            cfg = [build_robot(simcfg, f, w, h, chase=(f != "F07")), build_scene(simcfg, f, f, rtf, yaw_deg=0, origin=(-1.0, 8.0, -4.0))]
        elif f == "F09":
            noise = dict(NOISE_STRONG) if "strong" in a.variant.split("_") else NOISE_ON
            cfg = [build_robot(simcfg, "F09_off", w, h, chase=False), build_robot(simcfg, "F09_on", w, h, chase=False, noise=noise),
                   build_scene(simcfg, "F09_off", "F09_off", rtf, yaw_deg=0), build_scene(simcfg, "F09_on", "F09_on", rtf, yaw_deg=0)]
        elif f == "F10":
            cfg = [build_robot(simcfg, "F10_off", w, h, chase=False, blur=0.0), build_robot(simcfg, "F10_on", w, h, chase=False, blur=0.8),
                   build_scene(simcfg, "F10_off", "F10_off", rtf, yaw_deg=0), build_scene(simcfg, "F10_on", "F10_on", rtf, yaw_deg=0)]
        elif f == "F11":
            cfg = [build_robot(simcfg, f, 640, 360, types=(0, 4, 3), max_depth=60.0), build_scene(simcfg, f, f, rtf, yaw_deg=0, origin=(-1.0, 8.0, -4.0))]
        else:
            raise SystemExit(f"film {f} is not known")
        s.cap["config"] = cfg
        await s.connect()
        if f in ("F01", "F02"):
            await film_nature(s, a, f)
        elif f == "F03":
            await film_moving(s, a)
        elif f == "F04":
            await film_sun(s, a)
        elif f == "F07":
            await film_wind(s, a)
        elif f == "F08":
            await film_disturb(s, a)
        elif f in ("F09", "F10"):
            await film_ab(s, a, f)
        elif f == "F11":
            await film_race(s, a)
        s.cap["failures"] = s.fails
        s.cap["stage"] = "done"
        rc = 0
    except SystemExit as e:
        rc = int(e.code) if isinstance(e.code, int) else 1
        s.cap.setdefault("stage", "exit")
    except Exception:  # noqa: BLE001
        s.cap["traceback"] = traceback.format_exc()
        s.cap["stage"] = "error"
        log(s.cap["traceback"])
        rc = 1
    finally:
        try:
            s.drone.set_external_force([0.0, 0.0, 0.0])
            s.drone.disarm()
            s.drone.disable_api_control()
        except Exception:  # noqa: BLE001
            pass
        try:
            s.client.disconnect()
        except Exception:  # noqa: BLE001
            pass
        s.cap["wall_total_s"] = round(time.perf_counter() - s.t_wall0, 1)
        s.save()
    return rc


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--film", required=True)
    ap.add_argument("--variant", default="default")
    ap.add_argument("--out", required=True)
    ap.add_argument("--sim-config", required=True)
    ap.add_argument("--rtf", type=float, default=0.03)
    ap.add_argument("--frames", type=int, default=100)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--speed", type=float, default=1.5)
    ap.add_argument("--scale", type=float, default=4.0)
    ap.add_argument("--connect-wait", type=float, default=180.0)
    ap.add_argument("--picture-wait", type=float, default=120.0)
    ap.add_argument("--max-wall", type=float, default=5000.0)
    a = ap.parse_args()
    return asyncio.run(amain(a))


if __name__ == "__main__":
    sys.exit(main())
