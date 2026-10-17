"""PX4 + Gazebo race film FR: the hard race route (hard_race_v1.json, scale 4) in a city, flown by PX4 SITL with MAVSDK offboard setpoints,
with six timed phases (city only, wind + turbulence, rain, camera noise, motion blur, night + clouds), then the landing on the end pad.
Four pictures side by side: nose camera | depth | object labels | chase view.

  python3 tests/px4_gazebo_fr_run.py run  --out results [--px4 px4]    # start PX4 + Gazebo, build the city, fly, record the raw pictures (child.json, poses.npy, *.raw)
  python3 tests/px4_gazebo_fr_run.py post --out results                 # film, contact sheet, overview, film_result.json, summary.md (px4_gazebo/race_film.py)

Time is Gazebo simulation time everywhere (picture and pose time stamps); wall time only decides time-outs. Build and start of the stack are the code of
tests/px4_gazebo_base_run.py (the base lap, imported as `base`). The world has NO plugin in its file: PX4's server.config (physics, sensors,
scene broadcaster, ...) is only loaded when the world file brings none, so the extra systems (particle emitter, wind effects) are added to the
RUNNING world with the service entity/system/add; when that fails the whole run is repeated once with the plugin list inside the world file (variant B).
"""

import argparse
import asyncio
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "px4_gazebo"))
sys.path.insert(0, str(REPO / "tests"))
import px4_gazebo_base_run as base  # noqa: E402
import race_core as rc  # noqa: E402
import race_world as rw  # noqa: E402

THIS = Path(__file__).resolve()
PX4_WORLD = "Tools/simulation/gz/worlds/baylands.sdf"
PX4_MODELS = "Tools/simulation/gz/models"
SERVER_CONFIG = "src/modules/simulation/gz_bridge/server.config"
MODEL = base.MODEL
WORLD = rw.WORLD
FLIGHT_TIMEOUT_S = 105 * 60
HEALTHY_TIMEOUT_S = 600
FILM_FPS = rw.FPS
TURB_SIGMA_XY, TURB_SIGMA_Z, TURB_TAU = 1.8, 0.9, 1.2      # N, N, s: our Ornstein-Uhlenbeck force noise (turbulence)
TURB_DT = 0.1
EVENT_TP = dict(wind_on=10.0, turb_on=15.0, wind_off=20.0, rain_on=20.0, rain_off=30.0, noise_sub=28.5, noise_unsub=41.0, night=50.0)
POSE_DT = 0.1                                                # sim seconds between the pose updates of the moving objects
log = base.log


# ------------------------------------------------------------------------------------------------ gz helpers (python transport first, command line second)
MSG_MODULES = {"Pose_V": "pose_v_pb2", "Boolean": "boolean_pb2", "Light": "light_pb2", "Wind": "wind_pb2", "EntityWrench": "entity_wrench_pb2",
               "EntityPlugin_V": "entity_plugin_v_pb2", "EntityFactory": "entity_factory_pb2", "ParticleEmitter": "particle_emitter_pb2"}


def msg_class(name):
    import importlib
    return getattr(importlib.import_module("gz.msgs10." + MSG_MODULES[name]), name)


class Gz:
    """Service calls and topic publications. Python gz.transport first; if that raises, the `gz` command line (slower, one process per call)."""

    def __init__(self, S):
        self.S, self.pubs, self.py_fail, self.cli_calls, self.log = S, {}, 0, 0, []
        self.lock = threading.Lock()

    def service(self, service, reqtype, reptype, text, timeout_ms=5000):
        try:
            from google.protobuf import text_format
            rq, rp = msg_class(reqtype), msg_class(reptype)
            req = text_format.Parse(text, rq())
            ok, rep = self.S.node.request(service, req, rq, rp, timeout_ms)
            res = bool(ok) and (not hasattr(rep, "data") or bool(rep.data))
            self._note("py", service, res)
            return res
        except Exception as e:  # noqa: BLE001
            self.py_fail += 1
            self._note("py_error", service, repr(e)[:200])
        self.cli_calls += 1
        out = base.sh(f"gz service -s {service} --reqtype gz.msgs.{reqtype} --reptype gz.msgs.{reptype} --timeout {timeout_ms} --req '{text}'", 60)
        res = "data: true" in out
        self._note("cli", service, res if res else out.strip()[:200])
        return res

    def scene_light_ids(self, world):
        """name -> entity id of every light (world level and inside model links), from the scene/info service. Lights inside a model are found by id only."""
        ids = {}
        try:
            from google.protobuf import text_format
            from gz.msgs10.empty_pb2 import Empty
            from gz.msgs10.scene_pb2 import Scene
            ok, rep = self.S.node.request(f"/world/{world}/scene/info", Empty(), Empty, Scene, 20000)
            if not ok:
                raise RuntimeError("scene/info request failed")
            sc = rep
        except Exception as e:  # noqa: BLE001
            self._note("scene_info_error", world, repr(e)[:200])
            out = base.sh(f"gz service -s /world/{world}/scene/info --reqtype gz.msgs.Empty --reptype gz.msgs.Scene --timeout 20000 --req ''", 60)
            from google.protobuf import text_format
            from gz.msgs10.scene_pb2 import Scene
            sc = Scene()
            try:
                text_format.Parse(out, sc)
            except Exception as e2:  # noqa: BLE001
                self._note("scene_info_parse_error", world, repr(e2)[:200] + out[:100])
                return ids

        def walk_links(links):
            for lk in links:
                for l in lk.light:
                    ids[l.name] = l.id
        for l in sc.light:
            ids[l.name] = l.id
        for m in sc.model:
            walk_links(m.link)
        return ids

    def publish(self, topic, msgtype, text):
        try:
            from google.protobuf import text_format
            cls = msg_class(msgtype)
            with self.lock:
                if topic not in self.pubs:
                    self.pubs[topic] = self.S.node.advertise(topic, cls)
                    time.sleep(0.3)
                pub = self.pubs[topic]
            ok = pub.publish(text_format.Parse(text, cls()))
            self._note("py_pub", topic, ok)
            return bool(ok)
        except Exception as e:  # noqa: BLE001
            self.py_fail += 1
            self._note("py_pub_error", topic, repr(e)[:200])
        self.cli_calls += 1
        out = base.sh(f"gz topic -t {topic} -m gz.msgs.{msgtype} -p '{text}'", 30)
        self._note("cli_pub", topic, out.strip()[:100])
        return True

    def _note(self, how, what, res):
        if len(self.log) < 400:
            self.log.append((round(time.time() % 100000, 1), how, what, res))


def quat_yaw_pitch(yaw, pitch=0.0):
    """Quaternion (x, y, z, w) of Rz(yaw) * Ry(pitch)."""
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)


def pose_vector_text(items):
    return " ".join(f'pose {{name: "{n}" position {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} orientation {{x: {q[0]:.5f} y: {q[1]:.5f} z: {q[2]:.5f} w: {q[3]:.5f}}}}}'
                    for n, x, y, z, q in items)


# ------------------------------------------------------------------------------------------------ recording of the pictures
class PanelRec:
    """Raw pictures of one camera stream in a file, at most FILM_FPS per simulated second, with their simulation times."""

    def __init__(self, path: Path, kind: str, allowed):
        self.path, self.kind, self.allowed = path, kind, set(allowed)
        self.f = open(path, "wb")
        self.times, self.next_t, self.active, self.n_seen, self.nbytes, self.problems = [], None, False, 0, None, []
        self.sub = None

    def add(self, t, data: bytes):
        self.n_seen += 1
        if not self.active:
            return
        if len(data) not in self.allowed:
            if len(self.problems) < 4:
                self.problems.append(f"{self.kind}: {len(data)} bytes (allowed {sorted(self.allowed)})")
            return
        if self.nbytes is None:
            self.nbytes = len(data)
        if len(data) != self.nbytes:
            return
        if self.next_t is None:
            self.next_t = t
        if t + 1e-9 < self.next_t:
            return
        self.f.write(data)
        self.f.flush()
        self.times.append(t)
        self.next_t += 1.0 / FILM_FPS
        if self.next_t < t:
            self.next_t = t + 1.0 / FILM_FPS

    def close(self):
        try:
            self.f.close()
        except Exception:  # noqa: BLE001
            pass


def subscribe_panel(S, rec: PanelRec, topic: str):
    def on_image(msg):
        t = base.stamp_seconds(msg.header, S.now())
        if t is None:
            return
        rec.add(t, msg.data)
    ok = S.node.subscribe(S.Image, topic, on_image)
    rec.sub = topic if ok else None
    return ok


# ------------------------------------------------------------------------------------------------ files of the run
def backup_once(path: Path, backup_dir: Path) -> str:
    backup_dir.mkdir(parents=True, exist_ok=True)
    b = backup_dir / (str(path).replace("/", "__").replace("\\", "__").replace(":", "_"))
    if not b.exists():
        b.write_text(path.read_text())
    return b.read_text()


def prepare_files(px4: Path, out: Path, variant: str) -> dict:
    info = {"variant": variant}
    bk = out / "_backup"
    assets = rw.write_assets(out / "assets")
    info["assets"] = len(assets)
    wp = px4 / PX4_WORLD
    backup_once(wp, bk)
    plugins = ""
    if variant == "B":
        plugins = rw.server_plugins_from_config((px4 / SERVER_CONFIG).read_text())
    wp.write_text(rw.world_text(plugins))
    info["world_bytes"] = wp.stat().st_size
    info["world_plugins"] = plugins.count("<plugin")
    x500 = px4 / PX4_MODELS / "x500_mono_cam" / "model.sdf"
    px = backup_once(x500, bk)
    x500.write_text(rw.x500_mono_cam_text(px))
    info["x500_mono_cam_sensors"] = x500.read_text().count("<sensor")
    xb = px4 / PX4_MODELS / "x500_base" / "model.sdf"
    if xb.is_file():
        pb = backup_once(xb, bk)
        t, changed = rw.enable_wind_text(pb)
        xb.write_text(t)
        info["enable_wind_patched"] = changed
    else:
        info["enable_wind_patched"] = None
    city, counts = rw.city_model(assets)
    (out / "assets" / "city.sdf").write_text(city)
    for nm, txt in (("car", rw.car_model()), ("truck", rw.truck_model()), ("rain", rw.rain_model()), ("night_dome", rw.dome_model(assets)), *rw.person_models().items()):
        (out / "assets" / f"{nm}.sdf").write_text(txt)
    info["city_counts"] = counts
    info["city_bytes"] = len(city)
    (out / "world_used.sdf").write_text(wp.read_text())
    (out / "x500_mono_cam_used.sdf").write_text(x500.read_text())
    return info


def restore_files(px4: Path, out: Path):
    bk = out / "_backup"
    for p in (px4 / PX4_WORLD, px4 / PX4_MODELS / "x500_mono_cam" / "model.sdf", px4 / PX4_MODELS / "x500_base" / "model.sdf"):
        b = bk / (str(p).replace("/", "__").replace("\\", "__").replace(":", "_"))
        if b.exists():
            p.write_text(b.read_text())


def tail(path: Path, n=40):
    try:
        return "\n".join(path.read_text(errors="replace").splitlines()[-n:])
    except Exception:  # noqa: BLE001
        return ""


# ------------------------------------------------------------------------------------------------ the world builder, the movers and the effects
def find_service(pattern: str):
    out = base.sh("gz service -l", 30)
    return [l.strip() for l in out.splitlines() if re.search(pattern, l)]


def add_systems(gz: Gz, world: str, child: dict) -> dict:
    """Add the particle emitter and the wind effects systems to the running world. Returns {name: bool}."""
    res = {}
    cands = find_service(r"entity/system/add")
    child["system_add_services"] = cands
    svc = None
    for c in cands:
        if world in c:
            svc = c
            break
    if svc is None and cands:
        svc = cands[0]
    if svc is None:
        return {"ParticleEmitter": False, "WindEffects": False}
    for nm, fn, inner in (("ParticleEmitter", "gz-sim-particle-emitter-system", ""), ("WindEffects", "gz-sim-wind-effects-system", rw.WIND_PLUGIN_XML)):
        inner_q = inner.replace('"', '\\"')
        text = f'entity {{id: 1}} plugins {{name: "gz::sim::systems::{nm}" filename: "{fn}" innerxml: "{inner_q}"}}'
        for _ in range(3):         # run 39008289542: WindEffects answered false once and true the run before; try again
            res[nm] = gz.service(svc, "EntityPlugin_V", "Boolean", text, 15000)
            time.sleep(1.0)
            if res[nm]:
                break
    return res


def create_models(gz: Gz, out: Path, world: str, names, child: dict) -> dict:
    ans = {}
    for nm in names:
        t0 = time.time()
        ok = gz.service(f"/world/{world}/create", "EntityFactory", "Boolean", f'sdf_filename: "{out / "assets" / (nm + ".sdf")}"', 120000)
        ans[nm] = {"ok": ok, "wall_s": round(time.time() - t0, 1)}
        log(f"create {nm}: {ans[nm]}")
    return ans


class Movers(threading.Thread):
    """Moves the car, truck, person (body and swinging legs), the rain emitter (above the drone) and the night dome with set_pose_vector."""

    def __init__(self, S, gz: Gz, world: str, ground_z: float):
        super().__init__(daemon=True)
        self.S, self.gz, self.world, self.gz0 = S, gz, world, ground_z
        self.stop_flag = threading.Event()
        self.rain_on = False
        self.night_on = False
        self.n_calls = 0
        self.n_ok = 0
        self.max_gap_s = 0.0
        self.last_items = {}

    def items(self, t):
        it = []
        for nm in ("car", "truck"):
            m = rw.MOVERS[nm]
            x, y, h = rc.loop_pose(m["loop"], m["speed"], t, phase_s=7.0 if nm == "truck" else 0.0)
            it.append((nm, x, y, self.gz0 + (0.0), quat_yaw_pitch(h)))
        m = rw.MOVERS["person"]
        x, y, h = rc.loop_pose(m["loop"], m["speed"], t, phase_s=3.0)
        w = 2 * math.pi * 1.0 * t                                     # one stride per second
        bob = 0.03 * abs(math.sin(w))
        it.append(("person_body", x, y, self.gz0 + bob, quat_yaw_pitch(h)))
        it.append(("person_leg_l", x, y, self.gz0 + 0.9 + bob, quat_yaw_pitch(h, 0.55 * math.sin(w))))
        it.append(("person_leg_r", x, y, self.gz0 + 0.9 + bob, quat_yaw_pitch(h, -0.55 * math.sin(w))))
        with self.S.lock:
            p = self.S.pose
        if self.rain_on and p is not None:
            it.append(("rain", p[0], p[1], p[2] + 5.0, (0.0, 0.0, 0.0, 1.0)))
        else:
            it.append(("rain", 0.0, 0.0, -300.0, (0.0, 0.0, 0.0, 1.0)))
        if self.night_on:
            it.append(("night_dome", 0.0, 0.0, 0.0, quat_yaw_pitch(math.radians(0.8) * t)))
        else:
            it.append(("night_dome", 0.0, 0.0, -5000.0, (0.0, 0.0, 0.0, 1.0)))
        return it

    def run(self):
        last_t, last_wall = None, time.time()
        while not self.stop_flag.is_set():
            t = self.S.now()
            if t is None or (last_t is not None and t - last_t < POSE_DT):
                time.sleep(0.02)
                continue
            it = self.items(t)
            ok = self.gz.service(f"/world/{self.world}/set_pose_vector", "Pose_V", "Boolean", pose_vector_text(it), 4000)
            self.n_calls += 1
            self.n_ok += int(ok)
            if self.n_calls <= 3 or self.n_calls % 200 == 0:
                log(f"movers call {self.n_calls}: ok {ok}, t {t:.2f}, car at {it[0][1]:.1f}, {it[0][2]:.1f}")
            if last_t is not None:
                self.max_gap_s = max(self.max_gap_s, t - last_t)
            last_t = t


class Turbulence(threading.Thread):
    """Our Ornstein-Uhlenbeck force noise on the drone (persistent wrench of the ApplyLinkWrench system), updated every TURB_DT simulated seconds."""

    def __init__(self, S, gz: Gz, world: str):
        super().__init__(daemon=True)
        self.S, self.gz, self.world = S, gz, world
        self.on = False
        self.stop_flag = threading.Event()
        self.rng = np.random.default_rng(16)
        self.state = np.zeros(3)
        self.mean = np.zeros(3)      # steady force of the fallback wind (only when the WindEffects system is missing)
        self.gain = 1.0
        self.n = 0
        self.forces = []

    def run(self):
        last_t = None
        a = math.exp(-TURB_DT / TURB_TAU)
        b = math.sqrt(1 - a * a)
        sig = np.array([TURB_SIGMA_XY, TURB_SIGMA_XY, TURB_SIGMA_Z])
        while not self.stop_flag.is_set():
            t = self.S.now()
            if not self.on or t is None or (last_t is not None and t - last_t < TURB_DT):
                time.sleep(0.02)
                continue
            last_t = t
            self.state = a * self.state + b * sig * self.rng.normal(size=3)
            fx, fy, fz = self.gain * self.state + self.mean
            self.gz.publish(f"/world/{self.world}/wrench/persistent", "EntityWrench",
                            f'entity {{name: "{MODEL}" type: MODEL}} wrench {{force {{x: {fx:.3f} y: {fy:.3f} z: {fz:.3f}}}}}')
            self.n += 1
            if len(self.forces) < 2000:
                self.forces.append((round(t, 2), round(float(fx), 3), round(float(fy), 3), round(float(fz), 3)))

    def clear(self):
        self.on = False
        self.mean = np.zeros(3)
        self.gz.publish(f"/world/{self.world}/wrench/clear", "EntityWrench", f'entity {{name: "{MODEL}" type: MODEL}}')


def set_night(gz: Gz, world: str, on: bool, child: dict):
    res = {}
    ids = getattr(gz, "light_ids", {}) or {}

    def call(name, text):
        if name in ids:
            text = f"id: {ids[name]} " + text
        for _ in range(3):
            if gz.service(f"/world/{world}/light_config", "Light", "Boolean", text, 20000):
                return True
            time.sleep(1.0)
        return False
    for nm, spec in rw.LIGHTS.items():
        s = spec["night" if on else "day"]
        res[nm] = call(nm, rw.light_msg_text(nm, spec["type"], s))
    for k in range(len(rw.LAMPS)):
        res[f"lamp_{k}"] = call(f"lamp_light_{k}", rw.lamp_light_text(k, on))
    res["headlamp"] = call("headlamp", rw.headlamp_text(on))
    child.setdefault("light_config_answers", {})["night" if on else "day"] = res
    return res


class Live(threading.Thread):
    """Writes live.json and poses_live_ready.npy every few wall seconds while the film is made, so that the `early` step can look at the first pictures."""

    def __init__(self, S, child, panels, out: Path, ground):
        super().__init__(daemon=True)
        self.S, self.child, self.panels, self.out, self.ground = S, child, panels, out, ground
        self.stop_flag = threading.Event()

    def write(self):
        with self.S.lock:
            poses = np.array(self.S.poses, dtype=float)
        if len(poses):
            np.save(self.out / "poses_live_tmp.npy", poses)
            (self.out / "poses_live_tmp.npy").replace(self.out / "poses_live_ready.npy")
        d = {"arm_sim_s": self.child.get("arm_sim_s"), "route_start_sim_s": self.child.get("route_start_sim_s"), "record_start_sim_s": self.child.get("record_start_sim_s"),
             "ground_reference_world": list(self.ground), "sim_now": self.S.now(),
             "panel_times": {k: [round(t, 4) for t in v.times] for k, v in self.panels.items()}, "panel_nbytes": {k: v.nbytes for k, v in self.panels.items()},
             "events": self.child.get("events")}
        tmp = self.out / "live.tmp"
        tmp.write_text(json.dumps(d))
        tmp.replace(self.out / "live.json")

    def run(self):
        while not self.stop_flag.is_set():
            try:
                self.write()
            except Exception:  # noqa: BLE001
                pass
            time.sleep(4.0)


def night_test(S, gz: Gz, movers: Movers, world: str, topics: dict, out: Path, child: dict):
    """2.5 simulated seconds of pictures with the night settings, before the race (the night phase is the last one and the one that fails most often)."""
    res = {}
    recs = {}
    sizes = {"fpv": [rw.PW * rw.PH * 3], "chase": [rw.PW * rw.PH * 3], "depth": [rw.PW * rw.PH * 4], "seg_labels": [rw.PW * rw.PH * k for k in (1, 2, 3, 4)]}
    for nm, allowed in sizes.items():
        if topics.get(nm):
            rec = PanelRec(out / f"nt_{nm}.raw", nm, allowed)
            rec.topic = topics[nm]
            subscribe_panel(S, rec, rec.topic)
            recs[nm] = rec
    movers.night_on = True
    res["lights"] = set_night(gz, world, True, child)
    t0 = S.now()
    while S.now() < t0 + 3.0:
        time.sleep(0.2)
    for rec in recs.values():
        rec.active = True
    t1 = S.now()
    while S.now() < t1 + 2.5:
        time.sleep(0.2)
    for rec in recs.values():
        rec.active = False
        S.node.unsubscribe(rec.topic)
        rec.sub = None
        rec.close()
    movers.night_on = False
    set_night(gz, world, False, child)
    t2 = S.now()
    while S.now() < t2 + 1.5:
        time.sleep(0.2)
    res["panel_times"] = {k: [round(x, 4) for x in v.times] for k, v in recs.items()}
    res["panel_nbytes"] = {k: v.nbytes for k, v in recs.items()}
    (out / "night_test.json").write_text(json.dumps(res, default=str))
    return {k: len(v.times) for k, v in recs.items()}


# ------------------------------------------------------------------------------------------------ the flight
def tilt_deg_from_quat(q):
    qx, qy = q[0], q[1]
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qx * qx + qy * qy)))))


MAV_LOG = None          # set by run_variant: out / "mavsdk_server.log"
MAV_PORT = 50051
MAV_ADDRESS = "udpin://0.0.0.0:14540"


class MavServer:
    """Our own mavsdk_server process. MAVSDK-Python's own start (mavsdk/system.py, _start_mavsdk_server) pipes the server's output to a Python logging thread
    that drops everything below WARNING, so run 39009068651 lost every word the server said before it became <defunct>. Here the output goes to a file
    (uploaded with the results), the exit code is kept, and the server can be started again."""

    def __init__(self, logpath):
        self.logpath = Path(logpath)
        self.proc = None
        self.starts = 0
        self.exits = []                       # one dict per death: wall time, exit code, sim time, real-time factor

    def start(self, why):
        import mavsdk
        exe = os.path.join(os.path.dirname(mavsdk.__file__), "bin", "mavsdk_server")
        cmd = [exe, "-p", str(MAV_PORT), "--sysid", "245", "--compid", "190", MAV_ADDRESS]
        if os.path.exists("/usr/bin/stdbuf"):
            cmd = ["/usr/bin/stdbuf", "-oL", "-eL"] + cmd       # line-buffered, so a crash does not take the last lines with it
        with open(self.logpath, "ab") as f:
            f.write(f"--- start {self.starts} ({why}) at {time.strftime('%H:%M:%S')}: {' '.join(cmd)}\n".encode())
        lf = open(self.logpath, "ab", buffering=0)
        self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)
        lf.close()                                              # the child has its own copy of the file handle
        self.starts += 1
        log(f"mavsdk_server started ({why}): pid {self.proc.pid}, start no {self.starts}")

    def dead(self):
        return self.proc is None or self.proc.poll() is not None

    def stop(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.kill()
            try:
                self.proc.wait(5)
            except Exception:  # noqa: BLE001
                pass

    def tail(self, n=2500):
        try:
            return self.logpath.read_bytes()[-n:].decode("utf-8", "replace")
        except OSError:
            return ""


async def fly(S, child, gz: Gz, movers: Movers, turb: Turbulence, world: str, panels: dict, ground, prep=None):
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
    ex = ThreadPoolExecutor(max_workers=4)
    loop = asyncio.get_running_loop()
    x0, y0, z0 = ground

    def pose_f():
        with S.lock:
            p = S.pose
        return None if p is None else (p[0] - x0, p[1] - y0, p[2] - z0, p[3:])

    mav = MavServer(MAV_LOG if MAV_LOG is not None else Path("mavsdk_server.log"))
    mav.start("first")
    await asyncio.sleep(1.5)
    d = System(mavsdk_server_address="localhost", port=MAV_PORT)
    await d.connect()
    log("waiting for the connection to PX4")
    tasks = []

    async def connected():
        async for st in d.core.connection_state():
            if st.is_connected:
                return
    await asyncio.wait_for(connected(), 300)
    log("connected to PX4")

    async def healthy():
        last = 0.0
        async for h in d.telemetry.health():
            if h.is_global_position_ok and h.is_home_position_ok:
                return h
            if time.monotonic() - last > 20:
                last = time.monotonic()
                log(f"health: global {h.is_global_position_ok} home {h.is_home_position_ok} armable {h.is_armable} gyro {h.is_gyrometer_calibration_ok} "
                    f"accel {h.is_accelerometer_calibration_ok} mag {h.is_magnetometer_calibration_ok}; sim time {S.now()}")
    await asyncio.wait_for(healthy(), HEALTHY_TIMEOUT_S)
    log("healthy")
    child["healthy_sim_s"] = S.now()

    def server_monitor():
        # run 39009068651: the MAVSDK server was a zombie (<defunct>) after prep, which ran at about 0.12x real time. This line every 5 s wall shows WHEN it
        # dies and the real-time factor and machine load at that moment (PX4 heartbeats come in simulation time, so a slow simulation = rare heartbeats).
        prev = (time.monotonic(), S.now())
        seen_dead = False
        while not child.get("_mon_stop"):
            time.sleep(5.0)
            try:
                now = (time.monotonic(), S.now())
                rc_ = mav.proc.poll() if mav.proc is not None else "none"
                try:
                    load = open("/proc/loadavg").read().split()[0]
                except OSError:
                    load = "?"
                rtf = (now[1] - prev[1]) / max(now[0] - prev[0], 1e-6)
                log(f"mon: sim {now[1]:.1f} rtf {rtf:.2f} load {load} server pid {mav.proc.pid if mav.proc else None} exit code {rc_}")
                if rc_ not in (None, "none") and not seen_dead:
                    seen_dead = True
                    ev = {"wall": time.strftime("%H:%M:%S"), "exit_code": rc_, "sim": round(now[1], 2), "rtf": round(rtf, 3), "load": load}
                    mav.exits.append(ev)
                    child["server_deaths"] = list(mav.exits)
                    log("mon: SERVER DIED " + json.dumps(ev) + " | its last output: " + mav.tail(600).replace("\n", " / "))
                elif rc_ is None:
                    seen_dead = False
                prev = now
            except Exception as e:  # noqa: BLE001
                log(f"mon: error {e!r}")
    threading.Thread(target=server_monitor, daemon=True).start()
    state = {"armed": None, "landed": None}

    async def watch_armed():
        async for a in d.telemetry.armed():
            state["armed"] = a

    async def watch_landed():
        async for l in d.telemetry.landed_state():
            state["landed"] = str(l)

    async def new_session(why):
        """Start our own server again, connect a new System to it, wait until PX4 is connected and healthy, restart the telemetry watchers."""
        nonlocal d, tasks
        if mav.starts > 6:
            raise RuntimeError("mavsdk_server died too often: " + json.dumps(mav.exits))
        for t_ in tasks:
            t_.cancel()
        if mav.proc is not None and mav.proc.poll() is not None and not any(e.get("exit_code") == mav.proc.returncode and e.get("sim") is not None for e in mav.exits):
            mav.exits.append({"wall": time.strftime("%H:%M:%S"), "exit_code": mav.proc.returncode, "sim": round(S.now(), 2), "seen_by": why})
        child["server_deaths"] = list(mav.exits)
        log(f"mavsdk_server is gone ({why}), exit code {mav.proc.poll() if mav.proc else None}; its last output: " + mav.tail(800).replace("\n", " / "))
        mav.stop()
        await asyncio.sleep(1.0)
        mav.start(why)
        await asyncio.sleep(1.5)
        d = System(mavsdk_server_address="localhost", port=MAV_PORT)
        await d.connect()
        await asyncio.wait_for(connected(), 300)
        await asyncio.wait_for(healthy(), 300)
        log(f"new MAVSDK server ({why}): connected and healthy again")
        tasks = [asyncio.ensure_future(watch_armed()), asyncio.ensure_future(watch_landed())]

    async def ensure_server(why):
        if mav.dead():
            await new_session(why)
            return True
        return False

    async def setp(pos, vel):
        """Offboard set-point; if the server has died, bring a new one up, restart offboard mode, and send it again."""
        try:
            await d.offboard.set_position_velocity_ned(pos, vel)
        except Exception as e:  # noqa: BLE001
            child.setdefault("flight_server_losses", []).append({"sim": round(S.now(), 2), "error": repr(e)[:200], "dead": mav.dead()})
            await new_session("set-point failed in flight")
            await d.offboard.set_position_velocity_ned(pos, vel)
            await d.offboard.start()

    if prep is not None:
        await loop.run_in_executor(ex, prep)
        log(f"prep done (city, models, night test); server dead: {mav.dead()}")
    await ensure_server("after prep")
    tasks = [asyncio.ensure_future(watch_armed()), asyncio.ensure_future(watch_landed())]

    # prefill: the cameras render only while somebody listens; listen from here (not recorded) so the first recorded pictures are normal
    for nm in ("fpv", "depth", "seg_colored", "seg_labels", "chase"):
        rec = panels.get(nm)
        if rec is not None and rec.sub is None and rec.topic:
            subscribe_panel(S, rec, rec.topic)
    t_pre = S.now()
    while S.now() < t_pre + 3.0:
        await asyncio.sleep(0.2)
    with S.lock:
        S.t0 = S.sim_t
        S.poses = []
        S.recording = True
    for rec in panels.values():
        if rec.sub:
            rec.active = True
    wall0 = time.monotonic()
    child["record_start_sim_s"] = S.t0
    yaw0 = rc.ned_yaw_deg(0.0)
    t_arm = None
    for attempt in range(3):
        try:
            await ensure_server(f"before arm attempt {attempt}")
            await d.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw0))
            t_rec0 = S.now()
            while S.now() < t_rec0 + 1.0:
                await asyncio.sleep(0.05)
            await d.action.arm()
            t_arm = S.now()
            break
        except Exception as e:  # noqa: BLE001
            # run 39008289542: "Stream removed (Connection reset by peer)" on the arm call = the MAVSDK server process was gone. Say why, then start a new one.
            diag = {"attempt": attempt, "error": repr(e)[:300], "mavsdk_server_processes": base.sh("pgrep -af mavsdk_server", 10).strip()[:300], "server_exit_code": mav.proc.poll() if mav.proc else None, "server_output_tail": mav.tail(1500),
                    "free_m": base.sh("free -m", 10).strip(), "dmesg_tail": base.sh("sudo -n dmesg 2>&1 | tail -n 6", 20).strip()[-700:], "sim_now": S.now()}
            child.setdefault("arm_failures", []).append(diag)
            log("arm failed: " + json.dumps(diag))
            if attempt == 2:
                raise
            await new_session(f"arm attempt {attempt} failed")
    child["arm_sim_s"] = t_arm
    try:
        await d.offboard.start()
    except OffboardError as e:
        child["offboard_error"] = str(e)
        raise
    log(f"armed at sim time {t_arm}, offboard started")
    fired = set()
    child["events"] = {}

    def fire(name, fn):
        if name in fired:
            return
        fired.add(name)
        tp = S.now() - t_arm
        child["events"][name] = {"tp_planned": EVENT_TP.get(name), "tp_start": round(tp, 2)}
        log(f"event {name} at tp {tp:.2f}")

        def work():
            try:
                r = fn()
                child["events"][name]["result"] = r if isinstance(r, (bool, int, float, str, dict, type(None))) else str(r)
            except Exception:  # noqa: BLE001
                child["events"][name]["error"] = traceback.format_exc()[-600:]
            child["events"][name]["tp_done"] = round(S.now() - t_arm, 2)
        ex.submit(work)

    def noise_sub():
        rec = panels.get("fpvn")
        if rec is not None and rec.topic:
            subscribe_panel(S, rec, rec.topic)
            rec.active = True
        return bool(rec and rec.sub)

    def noise_unsub():
        rec = panels.get("fpvn")
        if rec is not None and rec.sub:
            rec.active = False
            S.node.unsubscribe(rec.sub)
        return True

    wind_xml = f"linear_velocity {{x: {rw.WIND_VELOCITY[0]} y: {rw.WIND_VELOCITY[1]} z: {rw.WIND_VELOCITY[2]}}} enable_wind: true"

    def wind_on():
        if not (child.get("systems_added") or {}).get("WindEffects") and not (child.get("systems_added") or {}).get("world_file"):
            turb.mean = np.array([2.4, 1.2, 0.0])       # fallback: a steady push of our own (no WindEffects system)
            turb.gain = 0.0
            turb.on = True
            child["wind_fallback"] = "our steady force 2.4, 1.2 N"
            return "fallback wind force"
        return gz.publish(f"/world/{world}/wind", "Wind", wind_xml)

    def wind_off():
        return gz.publish(f"/world/{world}/wind", "Wind", "linear_velocity {x: 0 y: 0 z: 0} enable_wind: false")

    def turb_on():
        turb.gain = 1.0
        turb.on = True
        return True

    def night():
        movers.night_on = True
        return set_night(gz, world, True, child)

    def turb_off():
        turb.clear()
        return True

    phase, tau0, end_reason, settled_since = "climb", None, None, None
    t_land_cmd = t_disarm = None
    end_t = None
    landed_flag = False
    while True:
        await asyncio.sleep(0.02)
        t = S.now()
        tp = t - t_arm
        # the phase events
        if tp >= EVENT_TP["wind_on"]:
            fire("wind_on", wind_on)
        if tp >= EVENT_TP["turb_on"]:
            fire("turb_on", turb_on)
        if tp >= EVENT_TP["wind_off"]:
            fire("wind_off", wind_off)
            fire("turb_off", turb_off)
            fire("rain_on", lambda: setattr(movers, "rain_on", True) or True)
        if tp >= EVENT_TP["noise_sub"]:
            fire("noise_sub", noise_sub)
        if tp >= EVENT_TP["rain_off"]:
            fire("rain_off", lambda: setattr(movers, "rain_on", False) or True)
        if tp >= EVENT_TP["noise_unsub"]:
            fire("noise_unsub", noise_unsub)
        if tp >= EVENT_TP["night"]:
            fire("night", night)
        if time.monotonic() - wall0 > FLIGHT_TIMEOUT_S:
            end_reason = "time"
            break
        if end_t is not None:
            if t >= end_t:
                break
            continue
        pf = pose_f()
        if pf is not None and tilt_deg_from_quat(pf[3]) > 75.0 and tp > 3:
            end_reason = "crash"
            end_t = t + 2.0
            child["crash_sim_s"] = t
            continue
        if phase == "climb":
            zsp = min(rc.HOVER_Z, rc.CLIMB_RATE * max(0.0, tp))
            vz = rc.CLIMB_RATE if zsp < rc.HOVER_Z else 0.0
            await setp(PositionNedYaw(0.0, 0.0, -zsp, yaw0), VelocityNedYaw(0.0, 0.0, -vz, yaw0))
            if pf is not None and zsp >= rc.HOVER_Z:
                near = math.dist(pf[:3], (0.0, 0.0, rc.HOVER_Z)) < 0.4
                settled_since = (settled_since if settled_since is not None else t) if near else None
                if settled_since is not None and t - settled_since > rc.HOLD_START_S:
                    phase, tau0 = "route", t
                    child["route_start_sim_s"] = t
                    log(f"route starts at sim time {t} (tp {tp:.1f})")
            if tp > 90.0 and phase == "climb":
                child["climb_failed"] = True
                end_reason = "time"
                break
        elif phase == "route":
            tau = t - tau0
            if tau > rc.T_ROUTE + rc.HOLD_END_S:
                phase, tau1 = "descend", t
                child["route_end_sim_s"] = t
                log(f"route finished at sim time {t} (tp {tp:.1f}); descending over the end pad")
                continue
            p, v = rc.ref_pos_vel(tau)
            yaw = rc.ned_yaw_deg(rc.ref_heading(tau))
            await setp(PositionNedYaw(float(p[1]), float(p[0]), float(-p[2]), yaw),
                       VelocityNedYaw(float(v[1]), float(v[0]), float(-v[2]), yaw))
        elif phase == "descend":
            yaw = rc.ned_yaw_deg(rc.ref_heading(rc.T_ROUTE))
            z = max(rc.LAND_HANDOVER_M, rc.HOVER_Z - rc.DESCENT_RATE * (t - tau1))
            vz = rc.DESCENT_RATE if z > rc.LAND_HANDOVER_M else 0.0
            await setp(PositionNedYaw(float(rc.END_PAD[1]), float(rc.END_PAD[0]), -float(z), yaw),
                       VelocityNedYaw(0.0, 0.0, float(vz), yaw))
            if z <= rc.LAND_HANDOVER_M + 1e-6 and (pf is None or abs(pf[2] - rc.LAND_HANDOVER_M) < 0.35):
                try:
                    await d.action.land()
                    t_land_cmd = t
                    child["land_cmd_sim_s"] = t
                    log(f"land command at sim time {t}")
                except Exception as e:  # noqa: BLE001
                    child["land_error"] = repr(e)
                    end_reason = "land_error"
                    end_t = t + 2.0
                phase = "landing"
            if t - tau1 > 60:
                child["descend_timeout"] = True
                end_reason = "time"
                break
        elif phase == "landing":
            if state["armed"] is False:
                t_disarm = t
                child["disarm_sim_s"] = t
                end_reason = "landed"
                end_t = t + 2.0
                landed_flag = True
                log(f"disarmed at sim time {t}; 2 s more")
            elif t - t_land_cmd > 70:
                child["landing_timeout"] = True
                end_reason = "time"
                end_t = t + 2.0
    child["flight_end_reason"] = end_reason
    child["landed"] = landed_flag
    child["record_wall_s"] = time.monotonic() - wall0
    with S.lock:
        S.recording = False
        child["record_end_sim_s"] = S.sim_t
    for rec in panels.values():
        rec.active = False
    for t_ in tasks:
        t_.cancel()
    child["_mon_stop"] = True
    child["server_starts"] = mav.starts
    child["server_deaths"] = list(mav.exits)
    mav.stop()
    ex.shutdown(wait=False)


def discover_topics(world: str, out: Path) -> dict:
    """Camera topics of the five sensors (regexes over `gz topic -l`)."""
    pre = rf"^/world/{re.escape(world)}/model/{re.escape(MODEL)}/link/"
    pats = {"fpv": pre + r"fpv_link/sensor/fpv/image$", "fpvn": pre + r"fpvn_link/sensor/fpvn/image$", "chase": pre + r"chase_link/sensor/chase/image$",
            "depth": pre + r"depth_link/sensor/depth/depth_image$", "seg_colored": pre + r"seg_link/sensor/seg/.*colored_map$", "seg_labels": pre + r"seg_link/sensor/seg/.*labels_map$"}
    found, text = {}, ""
    for _ in range(40):
        text = base.sh("gz topic -l", 30)
        found = {k: (re.search(p, text, re.M).group(0) if re.search(p, text, re.M) else None) for k, p in pats.items()}
        if all(found[k] for k in ("fpv", "chase")) and sum(1 for v in found.values() if v) >= 5:
            break
        time.sleep(3)
    (out / "gz_topics.txt").write_text(text)
    return found


def run_variant(out: Path, px4: Path, variant: str) -> dict:
    child = {"tag": "px4_gazebo_FR", "variant": variant, "simulator": "px4_gazebo", "scale": rc.SCALE, "versions": {}}
    child["files"] = prepare_files(px4, out, variant)
    log("files: " + json.dumps({k: v for k, v in child["files"].items() if k != "assets"}))
    S = base.Shared(out)
    procs = base.start_stack(out, px4)
    panels, movers, turb = {}, None, None
    need_restart = False
    try:
        t_start = time.time()
        m, topics = base.wait_for(r"^/world/([^/]+)/clock$", 12 * 60)
        (out / "gz_topics.txt").write_text(topics)
        if not m:
            raise RuntimeError("no Gazebo world clock topic: Gazebo did not start (see sim.log)")
        world = m.group(1)
        child["world"] = world
        child["world_start_wall_s"] = round(time.time() - t_start, 1)
        topics_found = discover_topics(world, out)
        child["camera_topics"] = topics_found
        if not (topics_found["fpv"] and topics_found["chase"]):
            raise RuntimeError(f"camera topics missing: {topics_found}")
        base.setup_gz(S, world)
        S.cam_topics = {}
        t0 = time.time()
        while S.pose is None and time.time() - t0 < 300:
            time.sleep(1)
        if S.pose is None:
            raise RuntimeError("no pose of the drone model in dynamic_pose/info")
        time.sleep(8)
        with S.lock:
            ground = (S.pose[0], S.pose[1], S.pose[2])
        child["ground_reference_world"] = list(ground)
        log(f"drone at rest at world {ground}")
        gz = Gz(S)
        # systems into the running world (variant A); in variant B they are in the world file
        if variant == "A":
            child["systems_added"] = add_systems(gz, world, child)
            log(f"systems added: {child['systems_added']}")
            if not child["systems_added"].get("ParticleEmitter"):
                child["systems_note"] = "service entity/system/add failed; variant B (plugins in the world file) is tried next"
                need_restart = True
                raise RuntimeError("systems could not be added to the running world")
        else:
            child["systems_added"] = {"world_file": True}
        child["particle_service_seen"] = bool(find_service(r"particle_emitters"))
        topics_now = base.sh("gz topic -l", 30)
        child["wind_topic_seen"] = bool(re.search(r"/wind$", topics_now, re.M))
        log(f"particle service seen: {child['particle_service_seen']}, wind topic seen: {child['wind_topic_seen']}")
        sizes = {"fpv": [rw.PW * rw.PH * 3], "fpvn": [rw.PW * rw.PH * 3], "chase": [rw.PW * rw.PH * 3], "depth": [rw.PW * rw.PH * 4],
                 "seg_colored": [rw.PW * rw.PH * 3, rw.PW * rw.PH * 4], "seg_labels": [rw.PW * rw.PH * k for k in (1, 2, 3, 4)]}
        for nm, topic in topics_found.items():
            rec = PanelRec(out / f"{nm}.raw", nm, sizes[nm])
            rec.topic = topic
            panels[nm] = rec
        movers = Movers(S, gz, world, 0.0)
        turb = Turbulence(S, gz, world)

        def prep():
            # PX4 is healthy; now the city, the moving things and the night test (the order of the base lap: connect first, build after)
            child["create_answers"] = create_models(gz, out, world, ["city", "car", "truck", "person_body", "person_leg_l", "person_leg_r", "night_dome", "rain"], child)
            child["topics_after_create"] = [l for l in base.sh("gz topic -l", 30).splitlines() if re.search(r"particle|wind|wrench|label|segmentation", l)]
            time.sleep(5)
            gz.light_ids = gz.scene_light_ids(world)
            child["light_ids"] = {k: v for k, v in gz.light_ids.items()}
            log(f"lights found: {sorted(gz.light_ids)}")
            movers.start()
            turb.start()
            child["night_test_frames"] = night_test(S, gz, movers, world, topics_found, out, child)
            log(f"night test: {child['night_test_frames']}")
            live = Live(S, child, panels, out, ground)
            live.start()
            child["_live"] = live
        global MAV_LOG
        MAV_LOG = out / "mavsdk_server.log"
        asyncio.run(fly(S, child, gz, movers, turb, world, panels, ground, prep))
        live = child.pop("_live", None)
        if live is not None:
            live.stop_flag.set()
        child["pose_messages"] = S.pose_n
        child["image_problems"] = {k: v.problems for k, v in panels.items() if v.problems}
        child["frames_seen"] = {k: v.n_seen for k, v in panels.items()}
        child["frames_kept"] = {k: len(v.times) for k, v in panels.items()}
        child["panel_times"] = {k: [round(t, 4) for t in v.times] for k, v in panels.items()}
        child["panel_nbytes"] = {k: v.nbytes for k, v in panels.items()}
        child["panel_topics"] = {k: v.topic for k, v in panels.items()}
        child["movers"] = {"calls": movers.n_calls, "ok": movers.n_ok, "max_gap_sim_s": movers.max_gap_s}
        child["turbulence"] = {"updates": turb.n, "force_samples": turb.forces[:600], "sigma_xy_N": TURB_SIGMA_XY, "sigma_z_N": TURB_SIGMA_Z, "tau_s": TURB_TAU}
        child["gz_calls"] = {"python_failures": gz.py_fail, "cli_calls": gz.cli_calls, "first_log": gz.log[:60]}
        poses = np.array(S.poses, dtype=float)
        np.save(out / "poses.npy", poses)
        child["pose_rows"] = int(len(poses))
        w = child.get("record_wall_s")
        if len(poses) > 2 and w:
            child["real_time_factor"] = float((poses[-1, 0] - poses[0, 0]) / w)
        child["end"] = "recorded"
    except Exception:  # noqa: BLE001
        child["traceback"] = traceback.format_exc()
        child.setdefault("end", "error")
        log(child["traceback"])
        try:
            poses = np.array(S.poses, dtype=float)
            if len(poses):
                np.save(out / "poses.npy", poses)
        except Exception:  # noqa: BLE001
            pass
    finally:
        for p in panels.values():
            p.close()
        for th in (movers, turb):
            if th is not None:
                th.stop_flag.set()
        child["sim_log_tail"] = tail(out / "sim.log", 25)
        (out / "child.json").write_text(json.dumps(child, indent=2, default=str))
        base.stop_stack({k: v for k, v in procs.items() if hasattr(v, "pid")})
        time.sleep(5)
        restore_files(px4, out)
    child["_need_restart"] = need_restart
    return child


def run(out: Path, px4: Path) -> dict:
    for f in ("child.json", "poses.npy"):
        (out / f).unlink(missing_ok=True)
    for p in out.glob("*.raw"):
        p.unlink()
    attempts = []
    child = run_variant(out, px4, "A")
    attempts.append({"variant": "A", "end": child.get("end"), "traceback": (child.get("traceback") or "")[-400:], "systems_added": child.get("systems_added")})
    if child.get("_need_restart"):
        (out / "child_variant_A.json").write_text(json.dumps(child, indent=2, default=str))
        base.sh("sleep 5; rm -f /tmp/.X99-lock", 20)
        for p in out.glob("*.raw"):
            p.unlink()
        child = run_variant(out, px4, "B")
        attempts.append({"variant": "B", "end": child.get("end"), "traceback": (child.get("traceback") or "")[-400:]})
    child["attempts"] = attempts
    child.pop("_need_restart", None)
    (out / "child.json").write_text(json.dumps(child, indent=2, default=str))
    return child


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("command", choices=["run", "post", "early", "nightsheet"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--px4", default="px4")
    a = ap.parse_args()
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if a.command == "run":
        run(out, Path(a.px4).resolve())
        sys.stdout.flush()
        os._exit(0)          # the MAVSDK server child and gRPC threads can keep a normal exit waiting
    elif a.command == "early":
        import race_film
        race_film.early_frames(out)
    elif a.command == "nightsheet":
        import race_film
        race_film.night_sheet(out)
    else:
        import race_film
        race_film.post(out)


if __name__ == "__main__":
    main()
