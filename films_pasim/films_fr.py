"""FR "The race with every feature" for Project AirSim 1.0.1 (client side). Owner's order 2026-10-16: ONE film per simulator.

One continuous flight in a packaged city scene (CityEnviron or Neighborhood, release v1.0.1): the hard_race_v1.json course at scale 4
(7 gates, 2 poles, 1 box built by our code from packaged cubes and cylinders), flown for two laps (at least 60 s in the air), then a landing on
the end pad. A person, a car and a truck move along lines beside the course during the WHOLE flight. Six phases switched by simulated time
since the start of the route: 0-10 s city only, 10-20 s wind + turbulence, 20-30 s rain, 30-40 s camera noise, 40-50 s motion blur,
50-60 s night + clouds (the last phase goes on until the landing).

Pictures: the nose camera (scene, depth visualisation, segmentation) by SUBSCRIPTION (drone.get_images gives up after 5 s and the city scenes need
more per picture). Camera noise, motion blur and (if the simulator cannot darken the scene) the night grading are added to the nose picture by
make_film_fr.py from the raw pictures; capture.json records what the simulator itself did at every phase switch.
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
import films2  # noqa: E402
from films2 import (PathFollower, Sess, bbox_extent, build_robot, build_scene, make_pose, race_geometry, solid_png,  # noqa: E402
                    spawn_course)
from pasim_films import log  # noqa: E402

TOPIC = {0: "scene_camera", 2: "depth_camera", 3: "segmentation_camera", 4: "depth_vis_camera"}
PHASES = [("city", 0, 10), ("wind_turbulence", 10, 20), ("rain", 20, 30), ("camera_noise", 30, 40), ("motion_blur", 40, 50), ("night_clouds", 50, 1e9)]
WIND_NED = (0.0, 2.0, 0.0)  # m/s toward the east (run 38988236522 used 3.0 and the drone missed gate G3 by 1.26 m: real gate bars are in the scene now)
C_DRAG = 0.5                # N per (m/s)
NIGHT_TIME = "2021-09-20 22:00:00"
LAMP_RADIUS = 3000.0
LAMP_LADDER = [2e3, 6e3, 2e4, 6e4, 2e5, 6e5, 2e6]   # light intensities tried before the flight (unit not documented; F05 used 5000 with radius 2000)
LAMP_TARGET_MEAN = 22.0                        # the lowest intensity whose nose picture has at least this mean brightness (0..255) is used


def decode_msg(msg):
    import numpy as np
    h, w = int(msg["height"]), int(msg["width"])
    buf = np.frombuffer(msg["data"], dtype=np.uint8)
    if buf.size == h * w * 3:
        return buf.reshape(h, w, 3)[:, :, ::-1].copy()
    if buf.size == h * w:
        return np.stack([buf.reshape(h, w)] * 3, axis=-1)
    if buf.size == h * w * 4:
        f = np.frombuffer(msg["data"], dtype=np.float32).reshape(h, w)
        f = np.nan_to_num(f, posinf=0.0)
        g = (255 * (1 - np.clip(f / max(float(np.percentile(f, 99)), 1e-3), 0, 1))).astype(np.uint8)
        return np.stack([g] * 3, axis=-1)
    raise ValueError(f"unknown image message size {buf.size} for {h}x{w}")


class MultiSub:
    """Latest picture of each image type of one camera, by subscription."""

    def __init__(self, sess, sensor, types):
        self.s, self.sensor, self.types = sess, sensor, list(types)
        self.latest, self.seen, self.errors, self.dead = {}, {}, [], set()
        for t in self.types:
            topic = sess.drone.sensors[sensor][TOPIC[t]]

            def cb(topic_, msg, t=t):
                try:
                    self.latest[t] = (msg["time_stamp"], decode_msg(msg))
                except Exception as e:  # noqa: BLE001
                    self.errors.append(f"type {t}: {e!r}"[:160])
            sess.client.subscribe(topic, cb)

    def next(self, wait=300.0, extra_wait=20.0):
        """Wait for a new scene picture, then give the other types a moment; returns {type: array or None}."""
        t = time.time()
        while time.time() - t < wait:
            item = self.latest.get(0)
            if item and item[0] != self.seen.get(0):
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("no new scene picture")
        self.seen[0] = self.latest[0][0]
        out = {0: self.latest[0][1]}
        t2 = time.time()
        for ty in self.types[1:]:
            if ty in self.dead:
                out[ty] = None
                continue
            while ty not in self.latest and time.time() - t2 < extra_wait:
                time.sleep(0.1)
            if ty not in self.latest:
                self.dead.add(ty)       # this image type never arrives by subscription: stop waiting for it
                self.s.cap.setdefault("dead_image_types", []).append(ty)
            out[ty] = self.latest[ty][1] if ty in self.latest else None
        return out


def next_scene(sub, n=1, wait=240.0):
    """Wait for n new scene (type 0) pictures; return the last as an array (used by the lamp calibration)."""
    img = None
    for _ in range(n):
        t = time.time()
        while time.time() - t < wait:
            item = sub.latest.get(0)
            if item and item[0] != sub.seen.get(0):
                sub.seen[0] = item[0]
                img = item[1]
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("no new scene picture")
    return img


class Mover:
    """A packaged model that walks or drives back and forth along a straight line beside the course (our script sets its pose every pair)."""

    def __init__(self, name, kind, asset, a, b, speed, ground_d, height_m):
        self.name, self.kind, self.asset, self.a, self.b, self.speed, self.ground_d, self.height_m = name, kind, asset, a, b, speed, ground_d, height_m
        self.L = math.dist(a, b)

    def state(self, t):
        s = (t * self.speed) % (2 * self.L)
        fwd = s <= self.L
        s = s if fwd else 2 * self.L - s
        u = s / self.L
        n, e = self.a[0] + u * (self.b[0] - self.a[0]), self.a[1] + u * (self.b[1] - self.a[1])
        yaw = math.atan2(self.b[1] - self.a[1], self.b[0] - self.a[0]) + (0.0 if fwd else math.pi)
        return n, e, yaw


def spawn_model(s, name, asset, center_ne, ground_d, height_m, yaw=0.0):
    """Spawn a packaged model at ground level, scaled uniformly so that its bounding box is `height_m` high (the base size is measured)."""
    w = s.world
    r = s.call(f"world.spawn_object({name}, {asset})", w.spawn_object, name, asset, make_pose(center_ne[0], center_ne[1], ground_d, yaw), [1.0, 1.0, 1.0], False)
    if not r["ok"]:
        return {"ok": False, "error": r["result"]}
    ext = bbox_extent(s, name)
    info = {"ok": True, "base_extent_m": [round(v, 3) for v in ext] if ext else None}
    if ext and ext[2] > 1e-3:
        k = height_m / ext[2]
        s.call(f"world.set_object_scale({name}, {k:.3f} uniform)", w.set_object_scale, name, [k, k, k])
        info["scale"] = round(k, 3)
        p = make_pose(center_ne[0], center_ne[1], ground_d - height_m / 2 + 0.0, yaw)  # bbox centre height: pivots are unknown, keep the ground contact by the centre
        s.call(f"world.set_object_pose({name}, centre at half height)", w.set_object_pose, name, p, True)
    return info


def night_on(s, c, WP):
    """Night: time of day, sunlight, fog and cloud-shadow calls (the same in the calibration and in the film). `c` records each call."""
    w = s.world
    c(f"world.set_time_of_day({NIGHT_TIME})", w.set_time_of_day, status=True, datetime=NIGHT_TIME, is_dst=False, clock_speed=1.0, update_interval=1.0, move_sun=False)
    c("world.set_sunlight_intensity(0.05)", w.set_sunlight_intensity, 0.05)
    c("world.enable_weather_visual_effects", w.enable_weather_visual_effects)
    c("world.set_weather_visual_effects_param(FOG, 0.3)", w.set_weather_visual_effects_param, param=WP.FOG, value=0.3)
    c("world.set_cloud_shadow_strength(1.0)", w.set_cloud_shadow_strength, 1.0)


def night_off(s, c):
    w = s.world
    c("world.reset_weather_effects", w.reset_weather_effects)
    c("world.enable_weather_visual_effects", w.enable_weather_visual_effects)
    c("world.set_cloud_shadow_strength(0.0)", w.set_cloud_shadow_strength, 0.0)
    c("world.set_sunlight_intensity(2.75)", w.set_sunlight_intensity, 2.75)
    c("world.set_time_of_day(17:00)", w.set_time_of_day, status=True, datetime="2021-09-20 17:00:00", is_dst=False, clock_speed=1.0, update_interval=1.0, move_sun=False)


def spawn_lamps(s, geo, ground_d, pad_end, wp):
    """Point lights (the simulator's own PointLightActor asset) at every gate, at the poles, at both pads, and one light that follows the drone (HeadLamp).
    All start with intensity 0 (off); the night phase switches them on at the calibrated intensity."""
    w = s.world
    spec = []
    for g in geo["gates"]:
        cn, ce, cd = g["center"]
        yaw = g["yaw"]
        f, left = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
        spec.append((f"Lamp_{g['id']}", (cn + 2.4 * left[0] - 1.5 * f[0], ce + 2.4 * left[1] - 1.5 * f[1], cd - 0.8), [1.0, 0.9, 0.75]))
    for o in geo["obstacles"]:
        vertical = o["kind"].startswith("vertical")
        spec.append((f"Lamp_{o['id']}", (o["xy"][0] + 1.0, o["xy"][1] + 1.0, ground_d - (3.0 if vertical else 2.0)), [1.0, 0.6, 0.3]))
    spec.append(("Lamp_PadStart", (wp[0][0] + 1.5, wp[0][1], ground_d - 3.0), [0.7, 0.85, 1.0]))
    spec.append(("Lamp_PadEnd", (pad_end[0] + 1.5, pad_end[1], ground_d - 3.0), [0.7, 0.85, 1.0]))
    spec.append(("HeadLamp", (wp[0][0] + 1.2, wp[0][1], wp[0][2] - 0.3), [1.0, 1.0, 1.0]))
    out = []
    for nm, (x, y, z), color in spec:
        r = s.call(f"world.spawn_object({nm}, PointLightActor)", w.spawn_object, nm, "PointLightActor", make_pose(x, y, z), [1.0, 1.0, 1.0], False)
        if not r["ok"]:
            out.append({"name": nm, "ok": False, "error": r["result"]})
            continue
        real = s.call.last_value if isinstance(s.call.last_value, str) else nm
        s.call(f"world.set_light_object_radius({nm}, {LAMP_RADIUS})", w.set_light_object_radius, real, LAMP_RADIUS)
        s.call(f"world.set_light_object_color({nm}, {color})", w.set_light_object_color, real, color)
        s.call(f"world.set_light_object_intensity({nm}, 0)", w.set_light_object_intensity, real, 0.0)
        out.append({"name": real, "wanted_name": nm, "ok": True, "ned": [round(x, 2), round(y, 2), round(z, 2)], "color": color})
    return out


def move_headlamp(s, n, e, d, yaw, lamps_name="HeadLamp"):
    try:
        s.world.set_object_pose(s.cap.get("headlamp_name", lamps_name), make_pose(n + 1.2 * math.cos(yaw), e + 1.2 * math.sin(yaw), d - 0.3), True)
    except Exception as ex_:  # noqa: BLE001
        s.cap.setdefault("headlamp_errors", []).append(repr(ex_)[:120])


async def calibrate_night(s, sub, lamps, yaw0, d0, ground_d):
    """Before the flight, with the drone hovering: day picture, night without lamps, then the lamps at rising intensity until the nose picture has a mean of at
    least LAMP_TARGET_MEAN. Then everything is switched back to day. Returns the chosen intensity (the highest of the ladder if none reached the target)."""
    from PIL import Image
    from projectairsim.drone import YawControlMode
    from projectairsim.types import WeatherParameter
    w = s.world
    cal = {"ladder": [], "target_mean": LAMP_TARGET_MEAN}
    s.cap["night_calibration"] = cal
    live = [l for l in lamps if l.get("ok")]
    for l in live:
        if l["wanted_name"] == "HeadLamp":
            s.cap["headlamp_name"] = l["name"]
    if not live:
        cal["error"] = "no lamp could be spawned"
        return LAMP_LADDER[-1]

    def c(name, fn, *a, **k):
        return s.call(name, fn, *a, **k)

    async def hover():
        await s.drone.move_by_velocity_async(0.0, 0.0, 0.0, 2.0, YawControlMode.MaxDegreeOfFreedom, False, yaw0)

    def shot(tag, img):
        Image.fromarray(img).save(s.frames_dir / f"cal_{tag}.jpg", quality=92)
        return round(float(img.mean()), 2), round(float(img.std()), 2)
    await hover()
    cal["day_mean"], cal["day_std"] = shot("day", next_scene(sub, 2))
    night_on(s, c, WeatherParameter)
    await hover()
    cal["night_no_lamps_mean"], cal["night_no_lamps_std"] = shot("night_nolamps", next_scene(sub, 3))
    chosen = LAMP_LADDER[-1]
    for lvl in LAMP_LADDER:
        for l in live:
            s.world.set_light_object_intensity(l["name"], lvl)
        n, e, d, yw = s.pose()
        move_headlamp(s, n, e, d, yw)
        await hover()
        m, sd = shot(f"lamps_{int(lvl)}", next_scene(sub, 3))
        cal["ladder"].append({"intensity": lvl, "mean": m, "std": sd})
        if m >= LAMP_TARGET_MEAN:
            chosen = lvl
            break
    for l in live:
        s.world.set_light_object_intensity(l["name"], 0.0)
    night_off(s, c)
    await hover()
    cal["day_after_mean"], cal["day_after_std"] = shot("day_after", next_scene(sub, 4))
    cal["chosen_intensity"] = chosen
    s.save()
    return chosen


async def film_fr(s, a):
    from projectairsim.drone import YawControlMode
    from projectairsim.types import WeatherParameter
    s.load("scene_FR.jsonc")
    ex = films2.explore(s)
    suns = [o for o in (ex.get("objects") or []) if isinstance(o, str) and re.search(r"sunsky|^sun|directionallight|skylight", o, re.I)]
    s.cap["sun_like_objects"] = suns
    s.cap["cube_asset"] = films2.pick_cube(ex.get("assets"))
    sub = MultiSub(s, "FrontCamera", (0, 4, 3))
    t0 = time.time()
    first = {}
    while time.time() - t0 < a.picture_wait * 3 and 0 not in first:
        try:
            first = sub.next(wait=60.0, extra_wait=30.0)
        except Exception as e:  # noqa: BLE001
            s.cap.setdefault("warmup_errors", []).append(repr(e)[:150])
    s.cap["warmup"] = {"seconds": round(time.time() - t0, 1), "types_received": [t for t, v in first.items() if v is not None], "decode_errors": sub.errors[:3]}
    if 0 not in first:
        s.cap["stage"] = "no_picture"
        s.save()
        raise SystemExit(3)
    n0, e0, d0, yaw0 = await s.takeoff()
    try:
        ground_elev = float(s.world.get_surface_elevation_at_point(n0, e0))
    except Exception as e:  # noqa: BLE001
        ground_elev = 0.0
        s.cap["ground_elevation_error"] = repr(e)[:150]
    ground_d = -ground_elev
    s.cap["ground_elevation_m"] = ground_elev
    scale = a.scale
    geo = race_geometry(scale, n0, e0)
    for g in geo["gates"]:
        g["center"] = (g["center"][0], g["center"][1], g["center"][2] + ground_d)
    wp = [(p[0], p[1], p[2] + ground_d) for p in geo["waypoints"]]
    geo["waypoints"] = wp
    s.cap["race"] = {"scale": scale, "origin_ned": [n0, e0], "geometry": geo, "ground_d": ground_d}
    tex_g = solid_png(s.out / "gate.png", (230, 90, 20), noise=10, seed=11)
    tex_p = solid_png(s.out / "pole.png", (240, 220, 40), noise=10, seed=12)
    tex_b = solid_png(s.out / "box.png", (40, 90, 200), noise=10, seed=13)
    for o in geo["obstacles"]:
        o["xy"] = tuple(o["xy"])
    s.cap["course_objects"] = _spawn_course_at_ground(s, geo, ground_d, tex_g, tex_p, tex_b)
    s.cap["course_objects_expected"] = 4 * len(geo["gates"]) + len(geo["obstacles"])
    pad_end = (n0 - 0.3 * scale, e0 - 2.0 * scale)
    for nm, (pn, pe) in (("PadStart", (n0, e0)), ("PadEnd", pad_end)):
        s.call(f"world.spawn_object({nm}, BasicLandingPad)", s.world.spawn_object, nm, "BasicLandingPad", make_pose(pn, pe, ground_d - 0.02), [scale, scale, scale], False)
    s.cap["pad_ned"] = {"start": [n0, e0], "end": list(pad_end)}

    # box around the course (north/east extent) and the lines of the moving objects, all outside it
    ns = [g["center"][0] for g in geo["gates"]] + [p[0] for p in wp]
    es = [g["center"][1] for g in geo["gates"]] + [p[1] for p in wp]
    nmin, nmax, emin, emax = min(ns) - 2.5, max(ns) + 2.5, min(es) - 2.5, max(es) + 2.5
    s.cap["course_box_ned"] = [nmin, nmax, emin, emax]
    movers = [Mover("MoverPerson", "person (walking)", "rp_manuel_rigged_001_ue4", (nmin - 2, emax + 6), (nmax + 2, emax + 6), 1.4, ground_d, 1.75),
              Mover("MoverCar", "car", "SKM_SportsCar", (nmin - 6, emin - 6), (nmax + 6, emin - 6), 5.0, ground_d, 1.4),
              Mover("MoverTruck", "truck (nearest model: off-road vehicle)", "SM_Offroad_Body", (nmax + 7, emin), (nmax + 7, emax), 3.5, ground_d, 2.4)]
    s.cap["movers"] = []
    for m in movers:
        n_, e_, yw = m.state(0.0)
        info = spawn_model(s, m.name, m.asset, (n_, e_), ground_d, m.height_m, yw)
        s.cap["movers"].append({"name": m.name, "kind": m.kind, "asset": m.asset, "line_ned": [list(m.a), list(m.b)], "speed_m_s": m.speed, **info})
    # trees close to the course (the scene's own tree assets), outside the course box
    rnd = random.Random(4)
    tree_assets = [x for x in (ex.get("assets") or []) if re.match(r"(Tree_A|Tree_B|Tree_C|SM_Tree_0|flg_tree_0|Oak_03|Fir_01|Birch_01)$", str(x))]
    s.cap["tree_assets_found"] = tree_assets
    trees = []
    for i in range(10 if tree_assets else 0):
        side = i % 4
        if side == 0:
            pos = (rnd.uniform(nmin, nmax), emin - rnd.uniform(1.5, 4))
        elif side == 1:
            pos = (rnd.uniform(nmin, nmax), emax + rnd.uniform(1.5, 4))
        elif side == 2:
            pos = (nmin - rnd.uniform(1.5, 4), rnd.uniform(emin, emax))
        else:
            pos = (nmax + rnd.uniform(1.5, 4), rnd.uniform(emin, emax))
        r = spawn_model(s, f"CityTree{i}", tree_assets[i % len(tree_assets)], pos, ground_d, 7.0 + rnd.uniform(-1, 1))
        trees.append({"name": f"CityTree{i}", "asset": tree_assets[i % len(tree_assets)], "ned": list(pos), **r})
    s.cap["trees"] = trees

    # lamps for the night phase (spawned now with intensity 0, calibrated at hover, switched on at the night phase)
    lamps = spawn_lamps(s, geo, ground_d, pad_end, wp)
    s.cap["lamps"] = lamps
    night_level = await calibrate_night(s, sub, lamps, yaw0, d0, ground_d)
    st_night = {"level": night_level}
    # the route: two laps, start of lap 1 from the hover point, then the gates again
    one_lap = wp[1:]
    path = [(n0, e0, d0)] + one_lap + one_lap + one_lap
    pf = PathFollower(path, a.speed, 1.6 * scale / 2)
    lap_ends = [pf.cum[(j + 1) * len(one_lap)] for j in range(3)]  # arc length at the end pad after lap 1, 2, 3
    s.cap["lap_end_arc_m"] = [round(x, 1) for x in lap_ends]
    rnd2 = random.Random(7)
    sigma = 0.6 * 0.2 * (1.0 / 0.027) / math.sqrt(0.25 / 0.002)  # (0.6 of the run-1b value: gate bars are real now) crazyflow disturbance_fn idea, mass-scaled, hold time ~0.25 s (see film_disturb in films2.py)
    s.cap["disturbance"] = {"donor": "crazyflow examples/plugins/disturbance.py (MIT)", "std_force_N_each_axis": round(sigma, 3), "wind_ned_m_s": WIND_NED, "drag_N_per_m_s": C_DRAG}
    st = {"yaw": yaw0, "phase": -1, "t_air0": None, "last": None, "landing": False, "events": [], "night": False}
    cams = {}

    def phase_of(t_air):
        for i, (nm, lo, hi) in enumerate(PHASES):
            if lo <= t_air < hi:
                return i
        return len(PHASES) - 1

    def switch(i, t_air):
        nm = PHASES[i][0]
        w = s.world
        ev = {"phase": nm, "t_air": round(t_air, 2), "calls": []}

        def c(name, fn, *args, **kw):
            r = s.call(name, fn, *args, **kw)
            ev["calls"].append({"api": name, "ok": r["ok"], "result": r["result"][:120], "where": r["where"]})
        # undo the previous phase
        if i >= 2:
            c("world.set_wind_velocity(0,0,0)", w.set_wind_velocity, 0.0, 0.0, 0.0)
            c("drone.set_external_force([0,0,0])", s.drone.set_external_force, [0.0, 0.0, 0.0])
        if i == 0:
            c("world.set_time_of_day(17:00)", w.set_time_of_day, status=True, datetime="2021-09-20 17:00:00", is_dst=False, clock_speed=1.0, update_interval=1.0, move_sun=False)
            c("world.enable_weather_visual_effects", w.enable_weather_visual_effects)
        elif i == 1:
            c("world.set_wind_velocity(0,3,0)", w.set_wind_velocity, *WIND_NED)
        elif i == 2:
            c("world.reset_weather_effects", w.reset_weather_effects)
            c("world.enable_weather_visual_effects", w.enable_weather_visual_effects)
            c("world.set_weather_visual_effects_param(RAIN, 1.0)", w.set_weather_visual_effects_param, param=WeatherParameter.RAIN, value=1.0)
            c("world.set_weather_visual_effects_param(ROAD_WETNESS, 1.0)", w.set_weather_visual_effects_param, param=WeatherParameter.ROAD_WETNESS, value=1.0)
            c("world.get_weather_visual_effects_param", w.get_weather_visual_effects_param)
        elif i == 3:
            c("world.reset_weather_effects", w.reset_weather_effects)
        elif i == 5:
            c("world.get_time_of_day (before)", w.get_time_of_day)
            night_on(s, c, WeatherParameter)
            for lp in lamps:
                if lp.get("ok"):
                    c(f"world.set_light_object_intensity({lp['name']}, {st_night['level']})", w.set_light_object_intensity, lp["name"], st_night["level"])
            st["night"] = True
            c("world.get_time_of_day (after)", w.get_time_of_day)
        st["events"].append(ev)
        s.cap["phase_events"] = st["events"]

    async def control(k, rec):
        t_abs = s.world.get_sim_time() / 1e9
        if st["t_air0"] is None:
            st["t_air0"] = t_abs
        t_air = t_abs - st["t_air0"]
        i = phase_of(t_air)
        if i != st["phase"]:
            for j in range(st["phase"] + 1, i + 1):
                switch(j, t_air)
            st["phase"] = i
        rec["fr_phase"], rec["t_air"] = PHASES[i][0], round(t_air, 3)
        # moving objects (our script sets their poses; the models do not animate)
        mv = {}
        for m in movers:
            n_, e_, yw = m.state(t_air)
            try:
                s.world.set_object_pose(m.name, make_pose(n_, e_, ground_d - m.height_m / 2, yw), True)
            except Exception as ex_:  # noqa: BLE001
                s.cap.setdefault("mover_errors", []).append(repr(ex_)[:120])
            mv[m.name] = [round(n_, 2), round(e_, 2)]
        rec["movers"] = mv
        n, e, d, yw = s.pose()
        v, done = pf.command((n, e, d))
        sp = math.hypot(v[0], v[1])
        if sp > 0.3:
            want = math.atan2(v[1], v[0])
            diff = (want - st["yaw"] + math.pi) % (2 * math.pi) - math.pi
            st["yaw"] += max(-0.45, min(0.45, diff))
        # wind + turbulence act on the drone in phase 1
        if i == 1:
            last = st["last"]
            vx, vy = ((n - last[0]) / max(t_abs - last[2], 1e-3), (e - last[1]) / max(t_abs - last[2], 1e-3)) if last else (0.0, 0.0)
            f = [C_DRAG * (WIND_NED[0] - vx) + rnd2.gauss(0, sigma), C_DRAG * (WIND_NED[1] - vy) + rnd2.gauss(0, sigma), rnd2.gauss(0, sigma)]
            s.drone.set_external_force(f)
            rec["ext_force"] = [round(x, 2) for x in f]
        st["last"] = (n, e, t_abs)
        if st["night"]:
            move_headlamp(s, n, e, d, st["yaw"])
        rec["s_along"], rec["path_done"] = round(pf.s, 2), done
        rec["t_air_over"] = t_air >= a.air_seconds
        await s.drone.move_by_velocity_async(v[0], v[1], v[2], 2.0, YawControlMode.MaxDegreeOfFreedom, False, st["yaw"])

    # custom loop (subscription pictures, 3 image types)
    from PIL import Image
    import numpy as np
    s.cap["stage"] = "film"
    ph = {"phase": "race", "first_frame": s.idx}
    t_start = time.perf_counter()
    k = 0
    while True:
        if time.perf_counter() - s.t_wall0 > a.max_wall:
            s.cap["stop_reason"] = "wall time limit in the race"
            break
        rec = {"i": s.idx, "phase": "race", "k": k, "settle": False}
        await control(k, rec)
        try:
            imgs = sub.next()
        except Exception as ex_:  # noqa: BLE001
            s.fails["nose"] = s.fails.get("nose", 0) + 1
            if s.fails["nose"] > 5:
                s.cap["stop_reason"] = f"no pictures: {ex_!r}"[:150]
                break
            continue
        for ty, nm in ((0, "fpv"), (4, "fpv_t4"), (3, "fpv_t3")):
            img = imgs.get(ty)
            if img is None:
                continue
            fn = f"{nm}_{s.idx:05d}.{'jpg' if ty == 0 else 'png'}"
            Image.fromarray(img).save(s.frames_dir / fn, **({"quality": 92} if ty == 0 else {}))
            rec[nm] = fn
            if ty == 0:
                rec["fpv_mean"], rec["fpv_std"] = round(float(img.mean()), 2), round(float(img.std()), 2)
        rec["t_abs"] = s.world.get_sim_time() / 1e9
        n_, e_, d_, yw_ = s.pose()
        rec.update({"n": n_, "e": e_, "d": d_, "yaw": yw_, "wall_s": round(time.perf_counter() - s.t_wall0, 2), "t": rec["t_abs"]})
        s.cap["frames"].append(rec)
        s.idx += 1
        k += 1
        if s.idx % 10 == 0:
            log(f"race pair {k} t_air={rec['t_air']:.1f}s phase={rec['fr_phase']} wall={rec['wall_s']:.0f}s s_along={rec['s_along']}/{pf.total:.0f}")
            s.save()
        # land at the end of the first lap that ends after `air_seconds` of flight (at most 3 laps)
        if rec["t_air_over"] and st.get("land_at_s") is None:
            st["land_at_s"] = next((le for le in lap_ends if le > pf.s + 1.0), lap_ends[-1])
            s.cap["landing_after_arc_m"] = round(st["land_at_s"], 1)
        if st.get("land_at_s") is not None and pf.s >= st["land_at_s"] - 0.5:
            break
        if rec["path_done"]:
            break
    ph.update({"last_frame": s.idx - 1, "wall_end_s": round(time.perf_counter() - s.t_wall0, 1)})
    s.cap["phases"].append(ph)
    s.cap["path_progress_m"] = {"total": pf.total, "reached": pf.s}
    # back above the end pad if the route was not finished, then land on the end pad
    n, e, d, _ = s.pose()
    if math.hypot(n - pad_end[0], e - pad_end[1]) > 2.0:
        pf2 = PathFollower([(n, e, d), (pad_end[0], pad_end[1], d0)], a.speed, 3.0)
        k2 = 0
        while k2 < 60:
            n, e, d, _ = s.pose()
            if st["night"]:
                move_headlamp(s, n, e, d, st["yaw"])
            v, done = pf2.command((n, e, d))
            await s.drone.move_by_velocity_async(v[0], v[1], v[2], 2.0, YawControlMode.MaxDegreeOfFreedom, False, st["yaw"])
            rec = {"i": s.idx, "phase": "to_pad", "k": k2, "settle": False, "fr_phase": PHASES[-1][0], "t_air": round(s.world.get_sim_time() / 1e9 - st["t_air0"], 3)}
            try:
                imgs = sub.next()
            except Exception:  # noqa: BLE001
                break
            for ty, nm in ((0, "fpv"), (4, "fpv_t4"), (3, "fpv_t3")):
                img = imgs.get(ty)
                if img is not None:
                    fn = f"{nm}_{s.idx:05d}.{'jpg' if ty == 0 else 'png'}"
                    Image.fromarray(img).save(s.frames_dir / fn)
                    rec[nm] = fn
                    if ty == 0:
                        rec["fpv_mean"], rec["fpv_std"] = round(float(img.mean()), 2), round(float(img.std()), 2)
            n_, e_, d_, yw_ = s.pose()
            rec.update({"t_abs": s.world.get_sim_time() / 1e9, "n": n_, "e": e_, "d": d_, "yaw": yw_, "wall_s": round(time.perf_counter() - s.t_wall0, 2)})
            rec["t"] = rec["t_abs"]
            rec["movers"] = {m.name: [round(v_, 2) for v_ in m.state(rec["t_air"])[:2]] for m in movers}
            for m in movers:
                n2, e2, yw2 = m.state(rec["t_air"])
                s.world.set_object_pose(m.name, make_pose(n2, e2, ground_d - m.height_m / 2, yw2), True)
            s.cap["frames"].append(rec)
            s.idx += 1
            k2 += 1
            if done:
                break
    # landing (the movers keep moving during the landing: the control function of the landing is the mover update)
    await _land_with_movers(s, sub, movers, st, pad_end)


async def _land_with_movers(s, sub, movers, st, pad_end, hold_s=2.0, max_pairs=150, descent=0.8):
    """Landing on the end pad by our own descent (drone.land_async never finished in the Blocks runs: the drone sat on the ground and the task stayed
    open, run 38986109324). Horizontal hold over the pad centre, vertical velocity `descent` m/s, touchdown = the height stops changing while the
    descent is still commanded; then disarm (motors stop) and keep `hold_s` simulated seconds. The movers keep moving all the time."""
    from PIL import Image
    from projectairsim.drone import YawControlMode
    try:
        s.drone.set_external_force([0.0, 0.0, 0.0])
    except Exception:  # noqa: BLE001
        pass
    n_a, e_a, d_a, _ = s.pose()
    done_t, prev, still = None, None, 0
    td_speed, max_v = None, 0.0
    for k in range(max_pairs):
        t_abs = s.world.get_sim_time() / 1e9
        t_air = t_abs - st["t_air0"]
        for m in movers:
            n2, e2, yw2 = m.state(t_air)
            try:
                s.world.set_object_pose(m.name, make_pose(n2, e2, m.ground_d - m.height_m / 2, yw2), True)
            except Exception:  # noqa: BLE001
                pass
        n, e, d, _ = s.pose()
        if st.get("night"):
            move_headlamp(s, n, e, d, st["yaw"])
        rec = {"i": s.idx, "phase": "landing", "k": k, "settle": False, "fr_phase": PHASES[-1][0], "t_air": round(t_air, 3)}
        if done_t is None:
            vn = max(-1.0, min(1.0, -0.8 * (n - pad_end[0])))
            ve = max(-1.0, min(1.0, -0.8 * (e - pad_end[1])))
            await s.drone.move_by_velocity_async(vn, ve, descent, 2.0, YawControlMode.MaxDegreeOfFreedom, False, st["yaw"])
            if prev is not None:
                v = (d - prev[0]) / max(t_abs - prev[1], 1e-3)
                max_v = max(max_v, v)
                if (d - d_a) > 0.5 and v < 0.12 and max_v > 0.3:
                    still += 1
                else:
                    still = 0
                if still >= 2:
                    done_t = t_abs
                    td_speed = round(max_v, 2)
                    r = s.call("drone.disarm() after touchdown", s.drone.disarm)
                    rec["disarm_ok"] = r["ok"]
                    rec["touchdown_detected_by"] = "height stopped changing under commanded descent"
            prev = (d, t_abs)
        rec["landing_task_done"] = done_t is not None
        try:
            imgs = sub.next()
        except Exception:  # noqa: BLE001
            break
        for ty, nm in ((0, "fpv"), (4, "fpv_t4"), (3, "fpv_t3")):
            img = imgs.get(ty)
            if img is not None:
                fn = f"{nm}_{s.idx:05d}.{'jpg' if ty == 0 else 'png'}"
                Image.fromarray(img).save(s.frames_dir / fn)
                rec[nm] = fn
                if ty == 0:
                    rec["fpv_mean"], rec["fpv_std"] = round(float(img.mean()), 2), round(float(img.std()), 2)
        n_, e_, d_, yw_ = s.pose()
        rec.update({"t_abs": s.world.get_sim_time() / 1e9, "n": n_, "e": e_, "d": d_, "yaw": yw_, "wall_s": round(time.perf_counter() - s.t_wall0, 2)})
        rec["t"] = rec["t_abs"]
        rec["movers"] = {m.name: [round(v_, 2) for v_ in m.state(t_air)[:2]] for m in movers}
        s.cap["frames"].append(rec)
        s.idx += 1
        if done_t is not None and rec["t_abs"] - done_t >= hold_s:
            break
    n_b, e_b, d_b, _ = s.pose()
    s.cap.setdefault("landings", []).append({"phase": "landing", "start_ned": [n_a, e_a, d_a], "end_ned": [n_b, e_b, d_b], "task_done": done_t is not None,
                                             "pad_ned": list(pad_end), "touchdown_speed_m_s": td_speed, "method": "own velocity descent + disarm (land_async did not finish in earlier runs)"})


def _spawn_course_at_ground(s, geo, ground_d, tex_g, tex_p, tex_b):
    """spawn_course() of films2 builds the course with z measured from NED 0; the gates and obstacles here already carry the ground offset for gates;
    poles and the box are put on the ground by this wrapper (their z is -height/2 in spawn_course, so shift by ground_d)."""
    made = spawn_course(s, geo, tex_g, tex_p, tex_b)
    if abs(ground_d) > 1e-6:
        w = s.world
        for nm in made:
            if nm.startswith(("Pole_", "Box_")):
                try:
                    p = w.get_object_pose(nm)
                    t = p["translation"] if isinstance(p, dict) else p.translation
                    x, y, z = (float(t["x"]), float(t["y"]), float(t["z"])) if isinstance(t, dict) else (float(t.x), float(t.y), float(t.z))
                    w.set_object_pose(nm, make_pose(x, y, z + ground_d), True)
                except Exception as e:  # noqa: BLE001
                    s.cap.setdefault("course_shift_errors", []).append(f"{nm}: {e!r}"[:120])
    return made


async def amain(a) -> int:
    out = Path(a.out)
    a.film, a.variant = "FR", a.variant
    s = Sess(a, out)
    simcfg = Path(a.sim_config)
    rc = 0
    try:
        cfg = [build_robot(simcfg, "FR", a.width, a.height, chase=False, types=(0, 4, 3), max_depth=80.0, blur=0.8), build_scene(simcfg, "FR", "FR", a.rtf, yaw_deg=0, origin=(-1.0, 8.0, -4.0))]
        s.cap["config"] = cfg
        await s.connect()
        await film_fr(s, a)
        s.cap["failures"] = s.fails
        s.cap["stage"] = "done"
    except SystemExit as e:
        rc = int(e.code) if isinstance(e.code, int) else 1
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
    ap.add_argument("--film", default="FR")
    ap.add_argument("--variant", default="default")
    ap.add_argument("--out", required=True)
    ap.add_argument("--sim-config", required=True)
    ap.add_argument("--rtf", type=float, default=0.012)
    ap.add_argument("--frames", type=int, default=400)
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=180)
    ap.add_argument("--speed", type=float, default=2.1)
    ap.add_argument("--scale", type=float, default=4.0)
    ap.add_argument("--air-seconds", type=float, default=62.0, help="simulated seconds on the route before the landing starts")
    ap.add_argument("--connect-wait", type=float, default=180.0)
    ap.add_argument("--picture-wait", type=float, default=120.0)
    ap.add_argument("--max-wall", type=float, default=15000.0)
    a = ap.parse_args()
    return asyncio.run(amain(a))


if __name__ == "__main__":
    sys.exit(main())
