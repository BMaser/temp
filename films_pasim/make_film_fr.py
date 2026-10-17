"""Make the film FR "The race with every feature", its contact sheets, overview row and film_result.json (Project AirSim).
usage: python make_film_fr.py <out dir> [variant]      (out dir holds capture.json and frames/ from films_fr.py)

Panels (2 x 2): nose camera | depth | object labels (segmentation) / map. Banner: phase, time, lap, gates, tag.
Camera noise, motion blur and (if the simulator did not darken the scene) the night grading are added to the NOSE panel here, from the raw pictures.
"""

import json
import math
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_film2 as m2  # noqa: E402
import unreal_films as uf  # noqa: E402

PW, PH = 480, 270
BAN, FOOT = 46, 30
PHASE_DEF = [("city", "CITY: moving objects, glare, reflections", 0, 10), ("wind_turbulence", "WIND + TURBULENCE", 10, 20), ("rain", "RAIN", 20, 30),
             ("camera_noise", "CAMERA NOISE", 30, 40), ("motion_blur", "MOTION BLUR", 40, 50), ("night_clouds", "NIGHT + CLOUDS", 50, 1e9)]
EXPOSURE_S, READOUT_S, FOV = 1 / 30, 1 / 30, 90.0


def load(p):
    try:
        return np.asarray(Image.open(p).convert("RGB"))
    except Exception:
        return None


def add_noise(img, seed):
    """Gaussian grain plus horizontal line noise (what the simulator's noise block does, as a fixed recipe of ours)."""
    rng = np.random.default_rng(seed)
    a = img.astype(np.float32) + rng.normal(0, 16, img.shape)
    rows = rng.random(img.shape[0]) < 0.06
    a[rows] += rng.normal(0, 40, (int(rows.sum()), 1, 1))
    shift = rng.integers(-3, 4, img.shape[0])
    for r in np.where(rng.random(img.shape[0]) < 0.15)[0]:
        a[r] = np.roll(a[r], int(shift[r]), axis=0)
    return np.clip(a, 0, 255).astype(np.uint8)


def night_grade(img):
    a = img.astype(np.float32) * np.array([0.16, 0.20, 0.30])[None, None, :]
    h, w = a.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w]
    vig = 1.0 - 0.55 * (((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2) / 2
    return np.clip(a * vig[:, :, None], 0, 255).astype(np.uint8)


def motion_blur(img, omega, speed):
    """Horizontal blur and rolling-shutter shear from the turn rate omega (rad/s) and the speed (m/s), pinhole camera (our recipe)."""
    h, w = img.shape[:2]
    f = (w / 2) / math.tan(math.radians(FOV) / 2)
    blur = int(max(5, min(45, abs(omega) * EXPOSURE_S * f + speed * 1.2)))
    shear = omega * READOUT_S * f
    a = img.astype(np.float32)
    k = np.ones(blur, dtype=np.float32) / blur
    pad = np.pad(a, ((0, 0), (blur // 2, blur - 1 - blur // 2), (0, 0)), mode="edge")
    out = np.zeros_like(a)
    for c in range(3):
        for r in range(h):
            out[r, :, c] = np.convolve(pad[r, :, c], k, mode="valid")
    res = out.copy()
    for r in range(h):
        s_ = int(round(shear * r / h))
        if s_ > 0:
            res[r] = np.pad(out[r], ((s_, 0), (0, 0)), mode="edge")[:w]
        elif s_ < 0:
            res[r] = np.pad(out[r], ((0, -s_), (0, 0)), mode="edge")[-s_:]
    return np.clip(res, 0, 255).astype(np.uint8), blur, round(shear, 1)


def crossings(geo, recs):
    """All gate crossings in time order: (t, gate id, lateral, vertical, inside)."""
    opening = geo["opening"]
    ev = []
    pos = [(r["n"], r["e"], r["d"]) for r in recs]
    ts = [r["t_abs"] for r in recs]
    for a, b, ta, tb in zip(pos[:-1], pos[1:], ts[:-1], ts[1:]):
        for g in geo["gates"]:
            f0, l0, z0 = m2.gate_local(g, a)
            f1, l1, z1 = m2.gate_local(g, b)
            if f0 < 0 <= f1 and f1 - f0 > 1e-9:
                u = -f0 / (f1 - f0)
                lat, dz = l0 + u * (l1 - l0), z0 + u * (z1 - z0)
                if abs(lat) < opening and abs(dz) < opening:  # within one opening width of the centre: a candidate crossing
                    ev.append((ta + u * (tb - ta), g["id"], lat, dz, abs(lat) < opening / 2 and abs(dz) < opening / 2))
    return ev


def route_lap_tables(events, flight, lap_end_arcs):
    """Gate passages per ROUTE lap (the route is one lap = the 7 gates, flown 2 to 3 times in a row). A crossing counts as PASSED when it is inside the opening,
    as MISSED when it is within one opening width of the centre but outside it (the drone clipped the frame). Returns a list of
    (time, route lap, gate id, passed) in time order. The route lap of an event comes from the path progress `s_along` of the flight at that time."""
    ts = np.array([r["t_abs"] for r in flight])
    ss = np.array([r.get("s_along", 0.0) for r in flight])
    out = []
    for t_, gid, lat, dz, inside in events:
        arc = float(np.interp(t_, ts, ss))
        lap = 1 + sum(1 for le in lap_end_arcs[:-1] if arc > le)
        out.append((t_, lap, gid, bool(inside)))
    return out


def closest(geo, recs):
    sc, opening, outer = geo["scale"], geo["opening"], geo["outer"]
    bar = 0.1 * sc
    gd = geo.get("ground_d", 0.0)
    cl = {g["id"] + "_frame": 1e9 for g in geo["gates"]}
    cl.update({o["id"]: 1e9 for o in geo["obstacles"]})
    pos = [(r["n"], r["e"], r["d"]) for r in recs]
    for a, b in zip(pos[:-1], pos[1:]):
        for sub in range(11):
            u = sub / 10.0
            p = tuple(a[i] + u * (b[i] - a[i]) for i in range(3))
            for g in geo["gates"]:
                f, lat, dz = m2.gate_local(g, p)
                bars = [((0, 0, -(opening / 2 + bar / 2)), (bar / 2, outer / 2, bar / 2)), ((0, 0, opening / 2 + bar / 2), (bar / 2, outer / 2, bar / 2)),
                        ((0, opening / 2 + bar / 2, 0), (bar / 2, bar / 2, opening / 2)), ((0, -(opening / 2 + bar / 2), 0), (bar / 2, bar / 2, opening / 2))]
                cl[g["id"] + "_frame"] = min(cl[g["id"] + "_frame"], min(m2.aabb_dist((f, lat, dz), h_, c_) for c_, h_ in bars))
            for o in geo["obstacles"]:
                if o["kind"].startswith("vertical"):
                    d = max(math.hypot(p[0] - o["xy"][0], p[1] - o["xy"][1]) - o["radius"], 0.0) if gd - p[2] < o["height"] else 1e9
                else:
                    d = m2.aabb_dist(p, (o["size"] / 2, o["size"] / 2, o["height"] / 2), (o["xy"][0], o["xy"][1], gd - o["height"] / 2))
                cl[o["id"]] = min(cl[o["id"]], d)
    return {k: round(v, 2) for k, v in cl.items()}


def map_panel(geo, recs, j, movers_cap, pads, box):
    fig, ax = plt.subplots(figsize=(PW / 100, PH / 100), dpi=100)
    for g in geo["gates"]:
        cn, ce = g["center"][0], g["center"][1]
        lat = (-math.sin(g["yaw"]), math.cos(g["yaw"]))
        half = geo["outer"] / 2
        ax.plot([ce - lat[1] * half, ce + lat[1] * half], [cn - lat[0] * half, cn + lat[0] * half], color="tab:orange", lw=3)
        ax.text(ce, cn, g["id"], fontsize=6)
    for o in geo["obstacles"]:
        ax.plot(o["xy"][1], o["xy"][0], "s" if not o["kind"].startswith("vertical") else "o", color="tab:blue", ms=5)
    for nm, (pn, pe) in pads.items():
        ax.plot(pe, pn, "D", color="tab:green", ms=6)
    ax.plot([q["e"] for q in recs[: j + 1]], [q["n"] for q in recs[: j + 1]], color="tab:red", lw=1.2)
    ax.plot(recs[j]["e"], recs[j]["n"], "ko", ms=4)
    col = {"MoverPerson": ("m", "person"), "MoverCar": ("c", "car"), "MoverTruck": ("brown", "truck")}
    for nm, (n_, e_) in (recs[j].get("movers") or {}).items():
        c_, lb = col.get(nm, ("gray", nm))
        ax.plot(e_, n_, "^", color=c_, ms=7)
        ax.text(e_, n_, lb, fontsize=6, color=c_)
    ax.set_aspect("equal", adjustable="datalim")
    ax.tick_params(labelsize=6)
    ax.grid(alpha=0.3)
    ax.set_xlabel("east (m)", fontsize=7)
    ax.set_ylabel("north (m)", fontsize=7)
    fig.tight_layout()
    return m2.fig_to_img(fig, PW, PH)


def main():
    out = Path(sys.argv[1])
    variant = sys.argv[2] if len(sys.argv) > 2 else "default"
    vtag = "" if variant == "default" else f"_{variant}"
    prefix = f"FR{vtag}"
    cap = json.loads((out / "capture.json").read_text())
    fd = out / "frames"
    recs = [r for r in cap.get("frames", []) if r.get("fpv")]
    res = {"film": "FR", "variant": variant, "simulator": "project_airsim", "version": "Project AirSim 1.0.1 (Unreal Engine 5)", "feature": "The race with every feature",
           "capture_stage": cap.get("stage"), "problems": []}
    if not recs or "race" not in cap:
        res.update({"verdict": "not_shown", "verdict_reason": f"no pictures (stage {cap.get('stage')}, error {cap.get('error') or (cap.get('traceback') or '')[-400:]})"})
        (out / "film_result.json").write_text(json.dumps(res, indent=2))
        return 1
    geo = cap["race"]["geometry"]
    geo["ground_d"] = cap["race"].get("ground_d", 0.0)
    wanted = [g["id"] for g in geo["gates"]]
    flight = [r for r in recs if r["phase"] == "race"]
    ev = crossings(geo, flight)
    lap_ends = cap.get("lap_end_arc_m") or [1e9]
    tab = route_lap_tables(ev, flight, lap_ends)
    # per-frame route lap, gates passed in this lap, gates missed in this lap
    lapk = []
    ts_f = np.array([r["t_abs"] for r in flight])
    ss_f = np.array([r.get("s_along", 0.0) for r in flight])
    last_lap = 1
    for r in recs:
        if r["phase"] == "race":
            arc = float(np.interp(r["t_abs"], ts_f, ss_f))
            last_lap = 1 + sum(1 for le in lap_ends[:-1] if arc > le)
        lap_now = last_lap
        passed = [g for (t_, lp, g, ok) in tab if lp == lap_now and ok and t_ <= r["t_abs"]]
        missed = [g for (t_, lp, g, ok) in tab if lp == lap_now and not ok and t_ <= r["t_abs"]]
        lapk.append((lap_now, len(passed), missed))
    # native night darkening measured
    def phase_means(name, which):
        v = [r["fpv_mean"] for r in recs if r.get("fr_phase") == name and "fpv_mean" in r]
        return (v[-4:] if which == "end" else v[2:6]) if v else []
    before, after = phase_means("motion_blur", "end"), phase_means("night_clouds", "start")
    ratio = round(float(np.mean(after) / max(np.mean(before), 1e-3)), 3) if before and after else None
    native_night = True  # the night is made by the simulator (time of day) and its own light actors; our grading is no longer applied (run 38988236522: night picture was black)
    # yaw rate and speed per frame
    yawrate, speed = [0.0], [0.0]
    for a_, b_ in zip(recs[:-1], recs[1:]):
        dt = max(b_["t_abs"] - a_["t_abs"], 1e-3)
        dy = (b_["yaw"] - a_["yaw"] + math.pi) % (2 * math.pi) - math.pi
        yawrate.append(dy / dt)
        speed.append(math.dist((a_["n"], a_["e"], a_["d"]), (b_["n"], b_["e"], b_["d"])) / dt)
    pads = {k: tuple(v) for k, v in cap.get("pad_ned", {}).items()}
    tags = {"city": ("NATIVE city scene; objects moved by OUR script", (60, 160, 60)), "wind_turbulence": ("ADDED BY OUR CODE (force)", (230, 140, 30)),
            "rain": ("NATIVE (weak)", (60, 160, 60)), "camera_noise": ("ADDED BY OUR CODE (picture)", (230, 140, 30)),
            "motion_blur": ("ADDED BY OUR CODE (picture)", (230, 140, 30)),
            "night_clouds": ("NATIVE night (sun off) + the simulator's own lamps, spawned by our script; clouds: fog only", (60, 160, 60))}
    sim_span = max(recs[-1]["t_abs"] - recs[0]["t_abs"], 1e-3)
    pics_per_sim_s = (len(recs) - 1) / sim_span
    fps = int(min(24, max(4, round(pics_per_sim_s))))   # playback at about real time: one recorded picture per frame
    frames, labels = [], []
    blur_info = {}
    for j, r in enumerate(recs):
        ph = r.get("fr_phase", "city")
        nose = load(fd / r["fpv"])
        nose_s = nose
        if nose is not None:
            if ph == "camera_noise":
                nose_s = add_noise(nose, j)
            elif ph == "motion_blur":
                nose_s, bpx, shear = motion_blur(nose, yawrate[j], speed[j])
                blur_info = {"blur_px_last": bpx, "shear_px_last": shear}
            elif ph == "night_clouds" and not native_night:
                nose_s = night_grade(nose)
        p1 = m2.tile(None, PW, PH, "nose camera") if nose_s is None else np.asarray(Image.fromarray(nose_s).resize((PW, PH), Image.BILINEAR))
        im = Image.fromarray(p1)
        d_ = ImageDraw.Draw(im)
        lab = "nose camera" + (" + our noise" if ph == "camera_noise" else " + our blur" if ph == "motion_blur" else " + our night grading" if ph == "night_clouds" and not native_night else "")
        b_ = d_.textbbox((6, 5), lab, font=uf.font(14))
        d_.rectangle([b_[0] - 3, b_[1] - 2, b_[2] + 3, b_[3] + 2], fill=(0, 0, 0))
        d_.text((6, 5), lab, fill=(255, 255, 255), font=uf.font(14))
        p1 = np.asarray(im)
        p2 = m2.tile(fd / r["fpv_t4"] if r.get("fpv_t4") else None, PW, PH, "depth (visualisation)")
        p3 = m2.tile(fd / r["fpv_t3"] if r.get("fpv_t3") else None, PW, PH, "object labels (segmentation)")
        p4 = map_panel(geo, recs, j, cap.get("movers"), pads, cap.get("course_box_ned"))
        grid = np.concatenate([np.concatenate([p1, p2], axis=1), np.concatenate([p3, p4], axis=1)], axis=0)
        ban = Image.new("RGB", (grid.shape[1], BAN), (15, 15, 15))
        bd = ImageDraw.Draw(ban)
        idx = [i for i, p in enumerate(PHASE_DEF) if p[0] == ph]
        title = PHASE_DEF[idx[0]][1] if idx else ph
        lapn, gk, miss = lapk[j]
        landing = r["phase"] == "landing"
        gtxt = f"gates passed {gk}/7" + (f" (missed {','.join(miss)})" if miss else "")
        bd.text((10, 6), f"FR  |  {'LANDING' if landing else title}  |  t {r['t_air']:.1f} s  |  route lap {lapn}  {gtxt}", fill=(255, 255, 255), font=uf.font(17))
        tg, colr = tags.get(ph, ("", (200, 200, 200)))
        bd.text((10, 27), tg, fill=colr, font=uf.font(14))
        foot = m2.strip(grid.shape[1], "moving: person, car, truck (poses set by our script) | recorded %.1f pictures per simulated second, film plays at %d fps (speed x%.2f) | picture %d | sim time %.2f s"
                        % (pics_per_sim_s, fps, fps / pics_per_sim_s, r["i"], r["t_abs"]))
        frames.append(np.concatenate([np.asarray(ban), grid, foot], axis=0))
        labels.append(f"{ph} t={r['t_air']:.0f}s lap {lapn} gates {gk}" + (f" missed {','.join(miss)}" if miss else ""))
    n = len(frames)
    mp4 = out / f"{prefix}_race_all_features.mp4"
    m2.write_film(mp4, frames, fps, out / "ffmpeg.log")
    seg_idx = []
    for nm, _, _, _ in PHASE_DEF:
        ii = [j for j, r in enumerate(recs) if r.get("fr_phase") == nm and r["phase"] == "race"]
        if ii:
            seg_idx.append(ii[len(ii) // 2])
    ll = [j for j, r in enumerate(recs) if r["phase"] == "landing"]
    if ll:
        seg_idx.append(ll[-1])
    ridx = m2.sheets(out, prefix, frames, labels, seg_idx, [labels[i] for i in seg_idx])
    ts = np.array([r["t_abs"] for r in recs])
    dts = np.diff(ts)
    # race numbers
    laps_tab = {}
    for t_, lp, g, ok in tab:
        laps_tab.setdefault(lp, {"passed": [], "missed": [], "first_t": None, "last_t": None})
        laps_tab[lp]["passed" if ok else "missed"].append(g)
        laps_tab[lp]["first_t"] = laps_tab[lp]["first_t"] if laps_tab[lp]["first_t"] is not None else round(t_, 1)
        laps_tab[lp]["last_t"] = round(t_, 1)
    for lp, v in laps_tab.items():
        v["passed_in_route_order"] = v["passed"] == [g for g in wanted if g in v["passed"]]
        v["all_7_passed"] = len(set(v["passed"])) == 7
    gate_offsets = {}
    for t, gid, lat, dz, inside in ev:
        phs = next((r.get("fr_phase") for r in flight if r["t_abs"] >= t), "?")
        gate_offsets.setdefault(phs, []).append(round(math.hypot(lat, dz), 2))
    cl = closest(geo, flight)
    laps_done = sum(1 for v in laps_tab.values() if v["all_7_passed"] and v["passed_in_route_order"])
    race = {"scale": cap["race"]["scale"], "laps_completed_all_7_gates": laps_done, "route_laps": laps_tab, "route_lap_end_arc_m": lap_ends,
            "crossings_note": "one entry per time the flight crosses a gate plane within one opening width of the centre; 14 entries = 2 route laps of 7 gates. The banner counts per ROUTE lap: passed = inside the opening, missed = clipped the frame.",
            "crossings": [(round(t, 1), g, round(l, 2), round(z, 2), i) for t, g, l, z, i in ev][:60],
            "closest_m_to_each_object": cl, "contact_threshold_m": 0.3, "contacts_inferred": [k for k, v in cl.items() if v < 0.3],
            "gate_centre_offset_m_by_phase": {k: {"n": len(v), "mean": round(float(np.mean(v)), 2), "max": round(max(v), 2)} for k, v in gate_offsets.items()},
            "time_in_air_s": round(max(r["t_air"] for r in flight), 1), "path_flown_m": round(sum(math.dist((a_["n"], a_["e"], a_["d"]), (b_["n"], b_["e"], b_["d"])) for a_, b_ in zip(flight[:-1], flight[1:])), 1),
            "largest_tilt": "not recorded (the program logs yaw only)"}
    res["race"] = race
    res["landing"] = m2.landing_metrics(cap, recs, ["landing"])
    for l_ in res["landing"]:
        l_["height_of_drone_centre_above_pad_plane_m"] = round(geo["ground_d"] - l_["landing_point_ned_m"][2], 2)
        li = (cap.get("landings") or [{}])[-1]
        l_["touchdown_speed_from_descent_m_s"] = li.get("touchdown_speed_m_s")
        l_["method"] = li.get("method")
    # phases
    ev_by = {e["phase"]: e for e in cap.get("phase_events", [])}
    phases = []
    for nm, title, lo, hi in PHASE_DEF:
        rr = [r for r in recs if r.get("fr_phase") == nm]
        e = ev_by.get(nm, {})
        calls_ok = [(c["api"], c["ok"]) for c in e.get("calls", [])]
        phases.append({"phase": nm, "title": title, "t_air_range_s": [lo, hi if hi < 1e8 else "until landing"], "frames": len(rr), "tag": tags[nm][0], "simulator_calls": e.get("calls", []),
                       "mean_nose_brightness": round(float(np.mean([r["fpv_mean"] for r in rr])), 1) if rr else None})
    def setv(name, verdict, reason, extra=None):
        for p in phases:
            if p["phase"] == name:
                p["verdict"], p["reason"] = verdict, reason
                if extra:
                    p.update(extra)
    setv("city", "native", f"packaged scene {cap.get('config', [{}])[1].get('scene_file', '')} (city). The person, car and truck are packaged models whose poses our script sets every pair (they slide; the person does not animate). Sun glare and reflecting surface: see notes.")
    wind_ok = [c for c in ev_by.get("wind_turbulence", {}).get("calls", []) if "wind_velocity" in c["api"]]
    setv("wind_turbulence", "added_by_our_code", "world.set_wind_velocity is accepted but moves nothing in v1.0.1 (measured in film F07: 0.0 m drift); our code applies a drag-like force toward 2 m/s east wind plus Crazyflow-style Gaussian force each pair through set_external_force.",
         {"native_call_ok": [c["ok"] for c in wind_ok], "gate_centre_offset": race["gate_centre_offset_m_by_phase"].get("wind_turbulence"), "addability": {"tried": True, "donor": "crazyflow (MIT) examples/plugins/disturbance.py", "result": "worked", "matches_check": "check F06: plug_in, small: yes"}})
    setv("rain", "native", "WeatherParameter.RAIN 1.0 and ROAD_WETNESS 1.0 (native, weak: drops and wet look, see film F06).")
    setv("camera_noise", "added_by_our_code", "The camera noise-settings block is read when the scene loads and cannot be switched during a flight (UnrealCamera.cpp TODO). Our recipe (Gaussian grain sigma 16 + line noise) is added to the nose picture. The native block was shown in film F09 with its own settings.")
    setv("motion_blur", "added_by_our_code", "motion-blur-amount (set to 0.8 for the whole flight) changes nothing in captures (film F10). Our horizontal blur and rolling-shutter shear are computed from the measured turn rate and speed.", {"last_blur": blur_info})
    cal = cap.get("night_calibration") or {}
    setv("night_clouds", "partly",
         f"Night: set_time_of_day({'22:00'}) + set_sunlight_intensity(0.05) alone made the nose picture black (mean 0.0 in run 38988236522; calibration without lamps: {cal.get('night_no_lamps_mean')}). "
         f"So the scene's own PointLightActor lamps (11 at the gates, poles and pads, plus one HeadLamp that follows the drone) are spawned by our script and set to intensity {cal.get('chosen_intensity')} "
         f"(chosen at hover before the flight: first level of a ladder with nose mean >= {cal.get('target_mean')}; ladder {cal.get('ladder')}). Nose brightness ratio night/before {ratio}. "
         "Clouds: no cloud-layer call exists; fog 0.3 and cloud-shadow strength 1.0 were set (partly).", {"night_brightness_ratio_after_over_before": ratio, "lamp_calibration": cal})
    res["phases"] = phases
    res["moving_objects"] = cap.get("movers")
    res["trees"] = {"spawned": len([t for t in cap.get("trees", []) if t.get("ok")]), "assets": cap.get("tree_assets_found")}
    res["numbers"] = {"film_file": mp4.name, "film_pictures": n, "playback_fps": fps, "film_seconds": round(n / fps, 1), "recorded_pictures_per_simulated_second": round(pics_per_sim_s, 2),
                      "playback_speed_vs_simulated_time": round(fps / pics_per_sim_s, 2), "rtf_wanted": (cap.get("config") or [{}, {}])[1].get("rtf_wanted"), "frame_size": list(frames[0].shape[1::-1]),
                      "simulated_seconds_filmed": round(float(ts[-1] - ts[0]), 1), "sim_dt_between_pairs_s": {"median": round(float(np.median(dts)), 3), "max": round(float(dts.max()), 3)},
                      "wall_seconds_total": cap.get("wall_total_s"), "wall_seconds_per_pair": round(float(np.median(np.diff([r["wall_s"] for r in recs]))), 2) if n > 2 else None,
                      "warmup": cap.get("warmup"), "picture_failures": cap.get("failures"), "stop_reason": cap.get("stop_reason"), "sun_like_objects": cap.get("sun_like_objects"),
                      "ground_elevation_m": cap.get("ground_elevation_m"), "course_objects_spawned": len(cap.get("course_objects", [])), "course_objects_expected": cap.get("course_objects_expected"), "cube_asset": cap.get("cube_asset"),
                      "night_calibration": cap.get("night_calibration"), "lamps": [(l_.get("name"), l_.get("ok")) for l_ in cap.get("lamps", [])]}
    res["config"] = cap.get("config")
    res["settings_used"] = [{"api": r["api"], "args": r["args"][:3], "ok": r["ok"], "result": r["result"][:100], "where": r["where"]} for r in cap.get("api_calls", [])][:400]
    res["verdict"] = "partly"
    res["verdict_reason"] = "one verdict per phase: see phases[]"
    res["verdict_is_automatic"] = True
    res["addability"] = {"tried": True, "note": "see phases[].addability and the F11/F08 blocks"}
    ovf = Path(__file__).resolve().parent / "verdicts.json"
    if ovf.is_file():
        ov = json.loads(ovf.read_text()).get("FR" + ("" if variant == "default" else "@" + variant))
        if ov:
            res["verdict"], res["verdict_reason"], res["verdict_is_automatic"] = ov["verdict"], ov["reason"], False
            res["notes"] = ov.get("notes")
    m2.row_png(out, prefix, frames, ridx, f"{prefix}  The race with every feature   phases: " + " / ".join(f"{p['phase']}:{p['verdict']}" for p in phases))
    try:
        pr = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=codec_name,pix_fmt,width,height,nb_frames", "-of", "json", str(mp4)], capture_output=True, text=True)
        res["film_probe"] = json.loads(pr.stdout)["streams"][0] if pr.returncode == 0 else {"error": pr.stderr[-200:]}
    except Exception as e:  # noqa: BLE001
        res["film_probe"] = {"error": repr(e)}
    (out / "film_result.json").write_text(json.dumps(res, indent=2, default=str))
    print(json.dumps({"phases": [(p["phase"], p["verdict"]) for p in phases], "race": {k: race[k] for k in ("laps_completed_all_7_gates", "route_laps", "contacts_inferred", "time_in_air_s")}, "landing": res["landing"], "numbers": res["numbers"]}, default=str)[:3000])
    return 0


if __name__ == "__main__":
    sys.exit(main())
