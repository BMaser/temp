"""Post-processing of the PX4 + Gazebo race film FR: from the raw pictures and the pose log that tests/px4_gazebo_fr_run.py recorded to the film
(four panels: nose camera | depth | object labels | chase view, banner, phase strip), the contact sheet, the overview and film_result.json.

numpy, scipy and Pillow only (no Gazebo, no MAVSDK), so the picture code can be tested anywhere (tests/test_race_film.py). ffmpeg is called only
in `post`.

What is NATIVE and what is ADDED BY OUR CODE is decided here from MEASUREMENTS of the raw pictures, never assumed: a native effect that cannot be
measured in the raw pictures is replaced by our own version of it and tagged ADDED BY OUR CODE (the owner's addability rule).
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage as ndi

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import race_core as rc  # noqa: E402
import race_world as rw  # noqa: E402
import sync_film  # noqa: E402

PW, PH = rw.PW, rw.PH
BANNER_H, STRIP_H = 60, 34
W, H = 2 * PW, BANNER_H + 2 * PH + STRIP_H
FPS = rw.FPS
TAG_COLORS = {"NATIVE": (40, 160, 70), "ADDED BY OUR CODE": (225, 130, 20), "NOT AVAILABLE": (200, 40, 40), "PARTLY": (190, 160, 20)}
CMAP = [(0.0, (10, 10, 70)), (0.25, (20, 80, 200)), (0.5, (30, 200, 120)), (0.75, (240, 220, 40)), (1.0, (240, 40, 30))]
DEPTH_NEAR, DEPTH_FAR = 0.5, 60.0
_F = {}


def font(size):
    if size not in _F:
        _F[size] = sync_film.font(size)
    return _F[size]


# ------------------------------------------------------------------------------------------------ measures on raw pictures
def gray(img):
    return img.astype(np.float32).mean(axis=2)


def hf_energy(img) -> float:
    g = gray(img)
    return float(np.abs(g - ndi.uniform_filter(g, 5)).mean())


def speck_fraction(img, thr=14.0) -> float:
    g = gray(img)
    return float(((g - ndi.uniform_filter(g, 5)) > thr).mean())


def glare_pixels(img) -> int:
    return int((img.max(axis=2) >= 250).sum())


# ------------------------------------------------------------------------------------------------ picture effects (ours)
def add_noise(img, rng, sigma=16.0):
    return np.clip(img.astype(np.float32) + rng.normal(0, sigma, img.shape), 0, 255).astype(np.uint8)


def temporal_blur(frames, weights=(0.25, 0.35, 0.40)):
    acc = np.zeros(frames[0].shape, np.float32)
    for f_, w in zip(frames, weights):
        acc += w * f_.astype(np.float32)
    return np.clip(acc / sum(weights), 0, 255).astype(np.uint8)


def add_rain_streaks(img, rng, n=240):
    h, w = img.shape[:2]
    ov = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    for _ in range(n):
        x, y = rng.uniform(0, w), rng.uniform(0, h)
        ln = rng.uniform(10, 30)
        d.line([(x, y), (x - 0.18 * ln, y + ln)], fill=(205, 215, 235, int(rng.uniform(70, 150))), width=1)
    return np.asarray(Image.alpha_composite(Image.fromarray(img).convert("RGBA"), ov).convert("RGB"))


def night_grade(img):
    return np.clip(img.astype(np.float32) * np.array([0.22, 0.26, 0.42], np.float32), 0, 255).astype(np.uint8)


def bloom(img, thr=245, sigma=9.0, gain=0.8):
    mask = (img.max(axis=2) >= thr).astype(np.float32)
    if mask.sum() < 25:
        return img, False
    b = ndi.gaussian_filter(mask, sigma)
    out = img.astype(np.float32) + gain * 255.0 * b[..., None] * np.array([1.0, 0.95, 0.8], np.float32)
    return np.clip(out, 0, 255).astype(np.uint8), True


def colorize_depth(d):
    d = np.asarray(d, np.float32)
    valid = np.isfinite(d) & (d > 0.0)
    v = np.zeros(d.shape, np.float32)
    v[valid] = np.clip(1.0 - (np.log(np.clip(d[valid], DEPTH_NEAR, DEPTH_FAR)) - math.log(DEPTH_NEAR)) / (math.log(DEPTH_FAR) - math.log(DEPTH_NEAR)), 0, 1)
    xs, cs = [c[0] for c in CMAP], np.array([c[1] for c in CMAP], np.float32)
    out = np.stack([np.interp(v, xs, cs[:, k]) for k in range(3)], axis=-1)
    out[~valid] = (0, 0, 0)
    return out.astype(np.uint8), float(valid.mean())


def decode_labels(raw, h, w):
    """Label ids from a labels_map picture (bytes per pixel 1, 2, 3 or 4); None when the size is unknown."""
    n = h * w
    b = len(raw) // n
    a = np.frombuffer(raw, dtype=np.uint8)
    if b == 1:
        return a.reshape(h, w).astype(np.int32)
    if b == 2:
        return np.frombuffer(raw, dtype="<i2").reshape(h, w).astype(np.int32)
    if b == 4:
        return np.frombuffer(raw, dtype="<i4").reshape(h, w).astype(np.int32)
    if b == 3:
        return a.reshape(h, w, 3)[..., 0].astype(np.int32)
    return None


PALETTE = np.zeros((256, 3), np.uint8)
for k, c in rw.LABEL_COLORS.items():
    PALETTE[k] = c


def colorize_labels(lab):
    idx = np.where((lab >= 0) & (lab < 256), lab, 0)
    return PALETTE[idx], idx


def label_legend(img, idx, d, top=6):
    counts = np.bincount(idx.ravel(), minlength=256)
    order = [k for k in np.argsort(-counts) if counts[k] > 0.004 * idx.size and k in rw.LABELS][:top]
    y = 6
    f = font(12)
    for k in order:
        d.rectangle([6, y, 18, y + 12], fill=tuple(int(v) for v in PALETTE[k]), outline=(0, 0, 0))
        tx = f"{rw.LABELS[k]} {100 * counts[k] / idx.size:.0f}%"
        box = d.textbbox((22, y - 1), tx, font=f)
        d.rectangle([box[0] - 1, box[1] - 1, box[2] + 2, box[3] + 1], fill=(0, 0, 0))
        d.text((22, y - 1), tx, fill=(255, 255, 255), font=f)
        y += 16
    return [rw.LABELS[k] for k in order]


# ------------------------------------------------------------------------------------------------ the picture of one film frame
def caption(d, xy, text, fill=(0, 0, 0), color=(255, 255, 255), size=13):
    f = font(size)
    box = d.textbbox(xy, text, font=f)
    d.rectangle([box[0] - 3, box[1] - 2, box[2] + 3, box[3] + 2], fill=fill)
    d.text(xy, text, fill=color, font=f)


def tag_color(tag):
    for k in ("ADDED BY OUR CODE", "NOT AVAILABLE", "PARTLY", "NATIVE"):
        if tag.startswith(k):
            return TAG_COLORS[k]
    return (90, 90, 90)


def draw_map(d, ox, oy, size, ctx):
    """Small drawn map (top view) of the course: gates, poles, box, pads, the planned route, the drone."""
    x0, x1, y0, y1 = -4.0, 26.0, -10.0, 11.0
    sc = min(size / (x1 - x0), size / (y1 - y0))
    mw, mh = int((x1 - x0) * sc), int((y1 - y0) * sc)

    def pxy(x, y):
        return ox + (x - x0) * sc, oy + mh - (y - y0) * sc
    d.rectangle([ox - 2, oy - 2, ox + mw + 2, oy + mh + 2], fill=(235, 235, 225), outline=(0, 0, 0))
    pts = [pxy(*rc.SPLINE(t)[:2]) for t in np.linspace(0, rc.T_ROUTE, 120)]
    d.line(pts, fill=(160, 160, 160), width=1)
    for o in rc.BOXES:
        a = pxy(o["xy"][0] - o["half"], o["xy"][1] - o["half"])
        b = pxy(o["xy"][0] + o["half"], o["xy"][1] + o["half"])
        d.rectangle([a[0], b[1], b[0], a[1]], fill=(150, 90, 40))
    for o in rc.POLES:
        cx, cy = pxy(*o["xy"])
        d.ellipse([cx - 3, cy - 3, cx + 3, cy + 3], fill=(220, 30, 30))
    for k in range(rc.N_GATES):
        gx, gy, _, yaw = rc.GATES[k]
        nx, ny_ = -math.sin(yaw), math.cos(yaw)                     # along the frame
        a, b = pxy(gx - nx * rc.OUTER / 2, gy - ny_ * rc.OUTER / 2), pxy(gx + nx * rc.OUTER / 2, gy + ny_ * rc.OUTER / 2)
        d.line([a, b], fill=(40, 170, 60) if k < ctx["gates"] else (255, 120, 0), width=3)
    for pad, col in ((rc.START_PAD, (240, 200, 30)), (rc.END_PAD, (40, 90, 220))):
        cx, cy = pxy(*pad[:2])
        d.rectangle([cx - 4, cy - 4, cx + 4, cy + 4], fill=col, outline=(0, 0, 0))
    cx, cy = pxy(ctx["x"], ctx["y"])
    d.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=(0, 200, 230), outline=(0, 0, 0))
    hx, hy = pxy(ctx["x"] + 2.5 * math.cos(ctx["yaw"]), ctx["y"] + 2.5 * math.sin(ctx["yaw"]))
    d.line([(cx, cy), (hx, hy)], fill=(0, 0, 0), width=2)


def compose_frame(panels: dict, ctx: dict) -> np.ndarray:
    """panels: fpv, depth, seg, chase: uint8 (PH, PW, 3). ctx: phase (dict), tag, tp, gates, speed, height, state, captions (dict), x, y, yaw."""
    canvas = np.zeros((H, W, 3), np.uint8)
    canvas[BANNER_H:BANNER_H + PH, :PW] = panels["fpv"]
    canvas[BANNER_H:BANNER_H + PH, PW:] = panels["depth"]
    canvas[BANNER_H + PH:BANNER_H + 2 * PH, :PW] = panels["seg"]
    canvas[BANNER_H + PH:BANNER_H + 2 * PH, PW:] = panels["chase"]
    img = Image.fromarray(canvas)
    d = ImageDraw.Draw(img)
    ph = ctx["phase"]
    tag = ctx["tag"]
    d.rectangle([0, 0, W, BANNER_H], fill=(20, 20, 24))
    d.text((8, 5), f"FR  PX4 + GAZEBO RACE    PHASE {ph['id']} OF 6:  {ph['name']}", fill=(255, 255, 255), font=font(21))
    tw = d.textlength(tag, font=font(17))
    d.rectangle([W - tw - 24, 4, W - 6, 30], fill=tag_color(tag))
    d.text((W - tw - 15, 7), tag, fill=(255, 255, 255), font=font(17))
    d.text((8, 34), f"t {ctx['tp']:5.1f} s    lap 1    gates {ctx['gates']} of {rc.N_GATES}    speed {ctx['speed']:3.1f} m/s    height {ctx['height']:4.1f} m    {ctx['state']}",
           fill=(255, 220, 120), font=font(17))
    d.line([(PW, BANNER_H), (PW, BANNER_H + 2 * PH)], fill=(255, 255, 255), width=1)
    d.line([(0, BANNER_H + PH), (W, BANNER_H + PH)], fill=(255, 255, 255), width=1)
    for (x, y, key) in ((6, BANNER_H + PH - 22, "fpv"), (PW + 6, BANNER_H + PH - 22, "depth"), (6, BANNER_H + 2 * PH - 22, "seg"), (PW + 6, BANNER_H + 2 * PH - 22, "chase")):
        text, tg = ctx["captions"][key]
        caption(d, (x, y), text)
        tw = d.textlength(tg, font=font(11))
        x1 = x + (PW - 12 if x < PW else PW - 12)
        d.rectangle([x1 - tw - 8, y - 1, x1, y + 17], fill=tag_color(tg))
        d.text((x1 - tw - 4, y + 2), tg, fill=(255, 255, 255), font=font(11))
    # legend of the label panel, drawn on the picture before: the caller puts it into panels["seg"]
    draw_map(d, W - 142, BANNER_H + PH + 8, 128, ctx)
    # phase strip
    y0 = BANNER_H + 2 * PH
    d.rectangle([0, y0, W, H], fill=(20, 20, 24))
    cw = W / 6
    for k, p in enumerate(rc.PHASES):
        cur = p["id"] == ph["id"]
        d.rectangle([k * cw + 2, y0 + 3, (k + 1) * cw - 2, H - 3], fill=(70, 70, 90) if cur else (36, 36, 44), outline=(255, 220, 120) if cur else (60, 60, 70))
        nm = f"{p['id']} {p['name']}"
        d.text((k * cw + 6, y0 + 5), nm, fill=(255, 255, 255) if cur else (150, 150, 160), font=font(11))
        d.text((k * cw + 6, y0 + 18), ctx["phase_tags"][p["id"]], fill=tag_color(ctx["phase_tags"][p["id"]]) if cur else (110, 110, 120), font=font(10))
    frac = min(max(ctx["tp"], 0.0) / 60.0, 1.0)
    d.rectangle([2, H - 5, 2 + frac * (W - 4), H - 2], fill=(255, 220, 120))
    return np.asarray(img)


# ------------------------------------------------------------------------------------------------ the data of a run
def nearest_index(times, t):
    i = int(np.searchsorted(times, t))
    if i <= 0:
        return 0
    if i >= len(times):
        return len(times) - 1
    return i if abs(times[i] - t) < abs(times[i - 1] - t) else i - 1


class Run:
    """Raw pictures (memory-mapped) and poses of the recorded flight."""

    def __init__(self, out: Path, child=None, poses_file="poses.npy", prefix=""):
        self.out = out
        self.child = child if child is not None else json.loads((out / "child.json").read_text())
        c = self.child
        self.ground = np.array(c.get("ground_reference_world", [0, 0, 0]), float)
        pa = out / poses_file
        self.poses = np.load(pa) if pa.is_file() else np.zeros((0, 8))
        self.t_arm = c.get("arm_sim_s")
        self.t_route = c.get("route_start_sim_s")
        self.times, self.arr, self.kind = {}, {}, {}
        for nm, ts in (c.get("panel_times") or {}).items():
            nb = (c.get("panel_nbytes") or {}).get(nm)
            p = out / f"{prefix}{nm}.raw"
            if not ts or not nb or not p.is_file():
                continue
            n = p.stat().st_size // nb
            n = min(n, len(ts))
            if n < 2:
                continue
            self.times[nm] = np.array(ts[:n])
            self.kind[nm] = nb
            if nm == "depth":
                self.arr[nm] = np.memmap(p, dtype="<f4", mode="r", shape=(n, PH, PW))
            elif nm == "seg_labels":
                self.arr[nm] = np.memmap(p, dtype=np.uint8, mode="r", shape=(n, nb))
            elif nm == "seg_colored":
                ch = nb // (PW * PH)
                self.arr[nm] = np.memmap(p, dtype=np.uint8, mode="r", shape=(n, PH, PW, ch))
            else:
                self.arr[nm] = np.memmap(p, dtype=np.uint8, mode="r", shape=(n, PH, PW, 3))

    def has(self, nm):
        return nm in self.arr

    def get(self, nm, t):
        i = nearest_index(self.times[nm], t)
        return np.asarray(self.arr[nm][i]), i, abs(self.times[nm][i] - t)


# ------------------------------------------------------------------------------------------------ flight numbers from the pose log
def flight_numbers(run: Run) -> dict:
    P = run.poses
    out = {"pose_rows": int(len(P))}
    if len(P) < 20 or run.t_arm is None:
        return out
    t = P[:, 0]
    pos = P[:, 1:4] - run.ground
    q = P[:, 4:8]
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 0] ** 2 + q[:, 1] ** 2), -1, 1)))
    out["max_tilt_deg"] = float(tilt.max())
    counter = rc.PassageCounter()
    mins = {"gate": (9.0, None, 0.0), "pole": (9.0, None, 0.0), "box": (9.0, None, 0.0)}
    contacts = {"gate": [], "pole": [], "box": []}
    for ti, p in zip(t, pos):
        counter.update(float(ti), p)
        d = rc.distances_all(p)
        for k in mins:
            if d[k][0] < mins[k][0]:
                mins[k] = (float(d[k][0]), d[k][1], float(ti - run.t_arm))
            if d[k][0] < rc.DRONE_RADIUS:
                contacts[k].append((round(float(ti - run.t_arm), 2), d[k][1], round(float(d[k][0]), 3)))
    out["gates_passed"] = counter.count
    out["gates_total"] = rc.N_GATES
    out["passage_log"] = [{"tp_s": round(a - run.t_arm, 2), "passage": b, "gate": c, "ly": round(float(d), 3), "lz": round(float(e), 3)} for a, b, c, d, e in counter.log]
    out["wrong_crossings"] = [{"tp_s": round(a - run.t_arm, 2), "gate": b, "ly": round(float(c), 3), "lz": round(float(d), 3)} for a, b, c, d in counter.wrong][:20]
    out["closest_distance_m"] = {k: {"m": round(v[0], 3), "object": v[1], "tp_s": round(v[2], 2)} for k, v in mins.items()}
    out["contacts"] = {k: len(v) for k, v in contacts.items()}
    out["contact_samples"] = {k: v[:8] for k, v in contacts.items() if v}
    out["drone_radius_m"] = rc.DRONE_RADIUS
    out["contact_rule"] = "geometric: the drone centre closer than the drone radius (0.37 m, motor arm plus propeller) to a gate frame bar, a gate post, a pole or the box; no contact sensor"
    if run.t_route is not None and counter.log:
        out["lap_times_s"] = [round(counter.log[-1][0] - run.t_route, 2)] if counter.count == rc.N_GATES else []
        out["lap_time_note"] = "from the start of the route (after take-off and settling) to the passage of gate 7; one lap"
    # tracking error and excess tilt per window, on the route
    if run.t_route is not None:
        T = t - run.t_route
        on = (T >= 0) & (T <= rc.T_ROUTE)
        ref = np.array([rc.SPLINE(x) for x in T[on]])
        err = np.linalg.norm(pos[on] - ref, axis=1)
        acc = np.array([np.linalg.norm(rc.SPLINE(x, 2)[:2]) for x in T[on]])
        exp_tilt = np.degrees(np.arctan2(acc, 9.81))
        ton = t[on] - run.t_arm
        out["tracking"] = {}
        for nm, (a, b) in {"phase1_city": (2, 10), "phase2a_wind_only": (10.5, 15), "phase2b_wind_turbulence": (15.5, 20), "phase3_rain": (20, 30), "phase4_noise": (30, 40),
                           "phase5_blur": (40, 50), "phase6_night": (50, 66)}.items():
            m = (ton >= a) & (ton < b)
            if m.sum() > 5:
                out["tracking"][nm] = {"mean_error_m": round(float(err[m].mean()), 3), "max_error_m": round(float(err[m].max()), 3),
                                       "mean_tilt_deg": round(float(tilt[on][m].mean()), 2), "mean_excess_tilt_deg": round(float((tilt[on][m] - exp_tilt[m]).mean()), 2),
                                       "mean_height_m": round(float(pos[on][m][:, 2].mean()), 2)}
    # landing
    c = run.child
    land = {"landed": bool(c.get("landed")), "land_command_tp_s": None if c.get("land_cmd_sim_s") is None else round(c["land_cmd_sim_s"] - run.t_arm, 2),
            "disarm_tp_s": None if c.get("disarm_sim_s") is None else round(c["disarm_sim_s"] - run.t_arm, 2), "end_pad_centre_m": [float(rc.END_PAD[0]), float(rc.END_PAD[1])]}
    if c.get("land_cmd_sim_s") is not None:
        m = t >= c["land_cmd_sim_s"]
        z = pos[:, 2]
        idx = np.where(m & (z <= 0.06))[0]
        if len(idx):
            i = int(idx[0])
            land["touchdown_tp_s"] = round(float(t[i] - run.t_arm), 2)
            j = int(np.searchsorted(t, t[i] - 0.3))
            land["touchdown_speed_m_s"] = round(float((z[j] - z[i]) / max(t[i] - t[j], 1e-3)), 3)
            fin = pos[-1]
            land["final_xy_m"] = [round(float(fin[0]), 3), round(float(fin[1]), 3)]
            land["distance_from_pad_centre_m"] = round(float(math.hypot(fin[0] - rc.END_PAD[0], fin[1] - rc.END_PAD[1])), 3)
            land["on_pad"] = land["distance_from_pad_centre_m"] <= rc.PAD_SIZE / 2
            land["final_height_m"] = round(float(fin[2]), 3)
    out["landing"] = land
    out["flight_time_air_s"] = None
    if land.get("touchdown_tp_s") is not None:
        liftoff = np.where(pos[:, 2] > 0.1)[0]
        if len(liftoff):
            out["flight_time_air_s"] = round(float(land["touchdown_tp_s"] - (t[liftoff[0]] - run.t_arm)), 2)
    return out


# ------------------------------------------------------------------------------------------------ the verdicts of the phases
WINDOWS = {1: (1.0, 9.5), 2: (10.5, 19.5), 3: (21.0, 29.5), 4: (31.0, 39.5), 5: (41.0, 49.5), 6: (51.0, 1e9)}


def frame_tp_list(run: Run, base_nm="chase"):
    nm = base_nm if run.has(base_nm) else ("fpv" if run.has("fpv") else None)
    if nm is None or run.t_arm is None:
        return None, np.array([])
    return nm, run.times[nm] - run.t_arm


def measure_effects(run: Run, fn: dict) -> dict:
    """Numbers from the RAW pictures that decide the tags of the phases."""
    ev = {}
    if not run.has("fpv") or run.t_arm is None:
        return {"problem": "no nose camera pictures"}
    tps = run.times["fpv"] - run.t_arm
    idx_all = np.arange(len(tps))
    rng = np.random.default_rng(3)

    def pick(a, b, n=14):
        m = np.where((tps >= a) & (tps < b))[0]
        if len(m) == 0:
            return []
        return list(m[np.linspace(0, len(m) - 1, min(n, len(m))).astype(int)])
    base_idx = pick(1.0, 9.5, 8) + pick(31.0, 49.5, 10)
    for cam in ("fpv", "chase"):
        if not run.has(cam):
            continue
        tp_c = run.times[cam] - run.t_arm

        def pk(a, b, n=14, tp_c=tp_c):
            m = np.where((tp_c >= a) & (tp_c < b))[0]
            return [] if len(m) == 0 else list(m[np.linspace(0, len(m) - 1, min(n, len(m))).astype(int)])
        b_idx = pk(1.0, 9.5, 8) + pk(31.0, 49.5, 10)
        r_idx = pk(21.0, 29.5)
        if b_idx and r_idx:
            sb = float(np.median([speck_fraction(np.asarray(run.arr[cam][i])) for i in b_idx]))
            sr = float(np.median([speck_fraction(np.asarray(run.arr[cam][i])) for i in r_idx]))
            hb = float(np.median([hf_energy(np.asarray(run.arr[cam][i])) for i in b_idx]))
            hr = float(np.median([hf_energy(np.asarray(run.arr[cam][i])) for i in r_idx]))
            ev[f"rain_{cam}"] = {"speck_fraction_rain": round(sr, 5), "speck_fraction_baseline": round(sb, 5), "speck_ratio": round(sr / max(sb, 1e-6), 2),
                                 "hf_rain": round(hr, 3), "hf_baseline": round(hb, 3), "hf_ratio": round(hr / max(hb, 1e-6), 2)}
    # camera noise: noisy twin against the clean camera at the same times
    if run.has("fpvn"):
        tn = run.times["fpvn"] - run.t_arm
        m = np.where((tn >= 31.0) & (tn < 39.5))[0]
        if len(m):
            m = m[np.linspace(0, len(m) - 1, min(10, len(m))).astype(int)]
            hn, hc = [], []
            for i in m:
                a = np.asarray(run.arr["fpvn"][i])
                b, _, _ = run.get("fpv", run.times["fpvn"][i])
                hn.append(hf_energy(a))
                hc.append(hf_energy(b))
            ev["noise"] = {"hf_noisy_camera": round(float(np.median(hn)), 3), "hf_clean_camera": round(float(np.median(hc)), 3),
                           "ratio": round(float(np.median(hn) / max(np.median(hc), 1e-6)), 2), "frames": len(m)}
    # night: mean grey
    g1 = [gray(np.asarray(run.arr["fpv"][i])).mean() for i in pick(1.0, 9.5, 6)]
    g6 = [gray(np.asarray(run.arr["fpv"][i])).mean() for i in pick(53.0, 70.0, 6)]
    if g1 and g6:
        ev["night"] = {"mean_grey_phase1": round(float(np.mean(g1)), 1), "mean_grey_phase6": round(float(np.mean(g6)), 1), "ratio": round(float(np.mean(g6) / max(np.mean(g1), 1e-6)), 2)}
    # glare: saturated area in the nose camera
    gl = [glare_pixels(np.asarray(run.arr["fpv"][i])) for i in pick(0.0, 1e9, 60)]
    ev["glare"] = {"frames_sampled": len(gl), "frames_with_saturated_area_over_200px": int(sum(1 for x in gl if x > 200)), "max_saturated_px": int(max(gl) if gl else 0)}
    # depth and labels
    if run.has("depth"):
        k = pick(0.0, 1e9, 20)
        vals = []
        for i in k:
            t_i = tps[i]
            dpt, _, _ = run.get("depth", run.t_arm + t_i)
            vals.append(float((np.isfinite(dpt) & (dpt > 0)).mean()))
        ev["depth"] = {"mean_valid_fraction": round(float(np.mean(vals)), 3) if vals else None, "frames": int(len(run.times["depth"]))}
    if run.has("seg_labels"):
        ids = set()
        for i in np.linspace(0, len(run.times["seg_labels"]) - 1, 12).astype(int):
            lab = decode_labels(bytes(run.arr["seg_labels"][i]), PH, PW)
            if lab is not None:
                ids |= set(int(v) for v in np.unique(lab))
        ev["labels"] = {"ids_seen": sorted(ids), "names_seen": [rw.LABELS.get(k, str(k)) for k in sorted(ids)]}
    elif run.has("seg_colored"):
        ev["labels"] = {"note": "colored map only (no labels map)"}
    return ev


def decide(run: Run, fn: dict, ev: dict) -> dict:
    """Per phase: tag, verdict and the reason, from the measurements."""
    c = run.child
    ph = {}
    sys_added = c.get("systems_added") or {}
    track = fn.get("tracking", {})
    # 1 city
    mv = c.get("movers") or {}
    ph[1] = dict(tag="NATIVE", verdict="native", how="the city, sun and sky are Gazebo primitives, lights and the world's own sky; pictures and meshes are generated by our code (no download); "
                 f"car, truck and a walking person move by our script (set_pose_vector {mv.get('ok')} of {mv.get('calls')} calls ok)",
                 evidence={"glare": ev.get("glare"), "movers": mv, "moving_objects_note": "person = body + two swinging legs (three models), car and truck = rigid models; none has a skeletal animation"})
    if mv.get("calls") and mv.get("ok", 0) < 0.5 * mv["calls"]:
        ph[1].update(tag="PARTLY", verdict="partly", how=ph[1]["how"] + "; MOST pose calls failed, the objects did not move as planned")
    # 2 wind
    a, b = track.get("phase1_city"), track.get("phase2a_wind_only")
    d_excess = None if not (a and b) else round(b["mean_excess_tilt_deg"] - a["mean_excess_tilt_deg"], 2)
    d_err = None if not (a and b) else round(b["mean_error_m"] - a["mean_error_m"], 3)
    t_b = track.get("phase2b_wind_turbulence")
    d_turb = None if not (b and t_b) else round(t_b["mean_error_m"] - b["mean_error_m"], 3)
    native_wind = bool(sys_added.get("WindEffects")) and (c.get("events", {}).get("wind_on", {}).get("result") in (True, None))
    wind_effect = d_excess is not None and d_excess >= 1.0
    if native_wind and wind_effect:
        ph[2] = dict(tag="NATIVE", verdict="native", how="WindEffects system (steady wind 3.0, 1.5 m/s with plugin gusts) pushes the drone; turbulence = our own Ornstein-Uhlenbeck force noise on the drone from 15 s (ADDED BY OUR CODE)")
    elif native_wind:
        ph[2] = dict(tag="PARTLY", verdict="partly", how="WindEffects system loaded and the wind message sent, but the excess tilt did not rise by 1 degree (no measurable push); turbulence = our own force noise from 15 s (ADDED BY OUR CODE)")
    else:
        ph[2] = dict(tag="ADDED BY OUR CODE", verdict="added_by_our_code", how="the WindEffects system could not be added; turbulence = our own force noise on the drone from 15 s")
    ph[2]["evidence"] = {"excess_tilt_change_deg_10to15_vs_phase1": d_excess, "tracking_error_change_m": d_err, "turbulence_extra_error_m": d_turb, "tracking": {k: track.get(k) for k in ("phase1_city", "phase2a_wind_only", "phase2b_wind_turbulence")},
                         "turbulence": {k: (c.get("turbulence") or {}).get(k) for k in ("updates", "sigma_xy_N", "sigma_z_N", "tau_s")}}
    ph[2]["turbulence_tag"] = "ADDED BY OUR CODE"
    # 3 rain
    rr = [ev.get(f"rain_{cam}") for cam in ("fpv", "chase") if ev.get(f"rain_{cam}")]
    best = max([r["speck_ratio"] for r in rr], default=0.0)
    visible = best >= 1.4 and bool(c.get("particle_service_seen") or sys_added.get("world_file"))
    if visible:
        ph[3] = dict(tag="NATIVE", verdict="native", how="Gazebo particle emitter (box emitter above the drone, ParticleEmitter system), drops seen in the raw pictures")
    else:
        ph[3] = dict(tag="ADDED BY OUR CODE", verdict="added_by_our_code",
                     how="the Gazebo particle emitter did not show up in the raw pictures (speck ratio %.2f, need 1.4), so our code draws rain streaks on the camera pictures" % best)
    ph[3]["evidence"] = {k: ev.get(k) for k in ("rain_fpv", "rain_chase")} | {"particle_service_seen": c.get("particle_service_seen"), "systems_added": sys_added}
    # 4 noise
    nz = ev.get("noise")
    if nz and nz["ratio"] >= 1.5:
        ph[4] = dict(tag="NATIVE", verdict="native", how="camera sensor noise element (gaussian, stddev %s) on a twin of the nose camera on the same mount, shown in this phase" % rw.NOISE_STDDEV)
    else:
        ph[4] = dict(tag="ADDED BY OUR CODE", verdict="added_by_our_code", how="the sensor noise of the twin camera is not measurable in the raw pictures (ratio %s), so our code adds gaussian noise" % (nz or {}).get("ratio"))
    ph[4]["evidence"] = nz
    # 5 blur
    ph[5] = dict(tag="ADDED BY OUR CODE", verdict="added_by_our_code", how="Gazebo's camera has no motion blur; our code averages three consecutive nose-camera pictures (a shutter of about 0.13 s)", evidence=None)
    # 6 night
    nt = ev.get("night")
    dark = bool(nt and nt["ratio"] < 0.7)
    if dark:
        ph[6] = dict(tag="NATIVE", verdict="partly",
                     how="night light is native (light_config service on the sun, fill, lamp and headlamp lights); the starry sky with clouds is a dome mesh our code generated and moves in (Gazebo cannot change its sky while it runs)",
                     evidence=nt)
    else:
        ph[6] = dict(tag="ADDED BY OUR CODE", verdict="added_by_our_code",
                     how="the light_config service did not make the pictures darker (grey ratio %s), so our code darkens the camera pictures; the starry sky with clouds is a dome mesh our code generated" % (nt or {}).get("ratio"), evidence=nt)
    ph[6]["evidence_light_config"] = (c.get("light_config_answers") or {}).get("night")
    return ph


# ------------------------------------------------------------------------------------------------ the film
class _NullProc:
    def __init__(self):
        import io
        self.stdin = io.BytesIO()

    def wait(self, timeout=None):
        return 0


def encode_pipe(mp4: Path):
    import os
    if os.environ.get("RACE_FILM_NO_FFMPEG"):
        return _NullProc()
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
           "-c:v", "libx264", "-preset", "veryfast", "-crf", "21", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE)


def blank_panel(text):
    img = Image.new("RGB", (PW, PH), (30, 30, 34))
    d = ImageDraw.Draw(img)
    d.text((16, PH // 2 - 12), text, fill=(255, 120, 120), font=font(18))
    return np.asarray(img)


def build_film(run: Run, fn: dict, ev: dict, ph: dict, mp4: Path, want_frames: dict, only_times=None) -> dict:
    """Write the film; returns facts. want_frames: name -> tp (s) of frames to keep for the contact sheet."""
    nm, tps = frame_tp_list(run, "chase")
    if nm is None:
        return {"problem": "no pictures"}
    times = run.times[nm]
    t0 = run.child.get("record_start_sim_s") or times[0]
    keep_k = {}
    rng = np.random.default_rng(11)
    P = run.poses
    pt = P[:, 0] if len(P) else np.array([0.0])
    pos = (P[:, 1:4] - run.ground) if len(P) else np.zeros((1, 3))
    sp = np.zeros(len(pos))
    if len(P) > 5:
        sp = np.linalg.norm(np.gradient(ndi.uniform_filter1d(pos, 15, axis=0), pt, axis=0), axis=1)
    yaw_arr = np.arctan2(2 * (P[:, 7] * P[:, 6] + P[:, 4] * P[:, 5]), 1 - 2 * (P[:, 5] ** 2 + P[:, 6] ** 2)) if len(P) else np.zeros(1)
    pass_t = [e["tp_s"] for e in fn.get("passage_log", [])]
    land = fn.get("landing", {})
    proc = _NullProc() if only_times is not None else encode_pipe(mp4)
    frames_written, max_dt = 0, {"fpv": 0.0, "depth": 0.0, "seg": 0.0, "chase": 0.0}
    tags = {k: v["tag"] for k, v in ph.items()}
    stats = {"blur_frames": 0, "noise_added_frames": 0, "rain_added_frames": 0, "native_noise_frames": 0, "bloom_frames": 0}
    prev = []
    t_end = run.child.get("record_end_sim_s") or times[-1]
    iter_times = times if only_times is None else [times[nearest_index(times, x)] for x in only_times]
    for k, tk in enumerate(iter_times):
        if tk > t_end + 1e-6:
            break
        tp = tk - run.t_arm
        phase = rc.phase_at(tp)
        pid = phase["id"]
        # panels
        pan = {}
        cap = {"fpv": ("NOSE CAMERA", "NATIVE"), "depth": ("DEPTH", "NATIVE"), "seg": ("OBJECT LABELS", "NATIVE"), "chase": (f"CHASE VIEW {rw.CHASE_BACK:g} m BEHIND", "NATIVE")}
        # nose camera, by phase
        if run.has("fpv"):
            img, _, dt = run.get("fpv", tk)
            max_dt["fpv"] = max(max_dt["fpv"], dt)
            if pid == 4 and run.has("fpvn") and ph[4]["tag"] == "NATIVE":
                img, _, dt2 = run.get("fpvn", tk)
                cap["fpv"] = ("NOSE CAMERA + SENSOR NOISE", "NATIVE")
                stats["native_noise_frames"] += 1
            elif pid == 4:
                img = add_noise(img, rng)
                cap["fpv"] = ("NOSE CAMERA + NOISE", "ADDED BY OUR CODE")
                stats["noise_added_frames"] += 1
            if pid == 3 and ph[3]["tag"] != "NATIVE":
                img = add_rain_streaks(img, rng)
                cap["fpv"] = ("NOSE CAMERA + RAIN", "ADDED BY OUR CODE")
                stats["rain_added_frames"] += 1
            if pid == 3 and ph[3]["tag"] == "NATIVE":
                cap["fpv"] = ("NOSE CAMERA + RAIN", "NATIVE")
            if pid == 5:
                i0 = nearest_index(run.times["fpv"], tk)
                fr3 = [np.asarray(run.arr["fpv"][max(i0 - 2, 0)]), np.asarray(run.arr["fpv"][max(i0 - 1, 0)]), np.asarray(run.arr["fpv"][i0])]
                img = temporal_blur(fr3)
                cap["fpv"] = ("NOSE CAMERA + MOTION BLUR", "ADDED BY OUR CODE")
                stats["blur_frames"] += 1
            if pid == 6 and ph[6]["tag"] != "NATIVE":
                img = night_grade(img)
                cap["fpv"] = ("NOSE CAMERA + NIGHT", "ADDED BY OUR CODE")
            img, did = bloom(img)
            if did:
                stats["bloom_frames"] += 1
                if pid in (1, 2, 4, 5):
                    cap["fpv"] = (cap["fpv"][0] + " + GLARE", cap["fpv"][1] if cap["fpv"][1] != "NATIVE" else "NATIVE + BLOOM ADDED")
            pan["fpv"] = np.ascontiguousarray(img)
        else:
            pan["fpv"] = blank_panel("NOSE CAMERA: NOT AVAILABLE")
            cap["fpv"] = ("NOSE CAMERA", "NOT AVAILABLE")
        # depth
        if run.has("depth"):
            dpt, _, dt = run.get("depth", tk)
            max_dt["depth"] = max(max_dt["depth"], dt)
            pan["depth"], _ = colorize_depth(dpt)
            cap["depth"] = (f"DEPTH {DEPTH_NEAR:g} to {DEPTH_FAR:g} m (red near)", "NATIVE")
        else:
            pan["depth"] = blank_panel("DEPTH: NOT AVAILABLE")
            cap["depth"] = ("DEPTH", "NOT AVAILABLE")
        # labels
        seg_img, legend = None, None
        if run.has("seg_labels"):
            raw, _, dt = run.get("seg_labels", tk)
            max_dt["seg"] = max(max_dt["seg"], dt)
            lab = decode_labels(bytes(raw), PH, PW)
            if lab is not None:
                col, idx = colorize_labels(lab)
                seg_img = Image.fromarray(col)
                dd = ImageDraw.Draw(seg_img)
                legend = label_legend(col, idx, dd)
        if seg_img is None and run.has("seg_colored"):
            raw, _, dt = run.get("seg_colored", tk)
            max_dt["seg"] = max(max_dt["seg"], dt)
            seg_img = Image.fromarray(np.ascontiguousarray(raw[..., :3]))
        if seg_img is not None:
            pan["seg"] = np.asarray(seg_img)
            cap["seg"] = ("OBJECT LABELS (semantic)", "NATIVE")
        else:
            pan["seg"] = blank_panel("OBJECT LABELS: NOT AVAILABLE")
            cap["seg"] = ("OBJECT LABELS", "NOT AVAILABLE")
        # chase
        img, _, dt = run.get("chase", tk)
        max_dt["chase"] = max(max_dt["chase"], dt)
        if pid == 6 and ph[6]["tag"] != "NATIVE":
            img = night_grade(img)
            cap["chase"] = (cap["chase"][0] + " + NIGHT", "ADDED BY OUR CODE")
        if pid == 3 and ph[3]["tag"] != "NATIVE":
            img = add_rain_streaks(img, rng, 330)
            cap["chase"] = (cap["chase"][0] + " + RAIN", "ADDED BY OUR CODE")
        elif pid == 3:
            cap["chase"] = (cap["chase"][0] + " + RAIN", "NATIVE")
        img, _ = bloom(img)
        pan["chase"] = np.ascontiguousarray(img)
        # context
        j = int(np.searchsorted(pt, tk))
        j = min(max(j, 0), len(pt) - 1)
        gates_now = sum(1 for x in pass_t if x <= tp + 1e-6)
        landed = land.get("disarm_tp_s") is not None and tp >= land["disarm_tp_s"]
        if tp < 0.2:
            state = "ON THE START PAD"
        elif landed:
            state = "LANDED, MOTORS OFF"
        elif land.get("land_command_tp_s") is not None and tp >= land["land_command_tp_s"]:
            state = "LANDING"
        elif gates_now == rc.N_GATES:
            state = "OVER THE END PAD"
        else:
            state = "FLYING THE ROUTE"
        ctx = dict(phase=phase, tag=ph[pid]["tag"] + {2: " + OUR TURBULENCE", 6: " + OUR SKY DOME"}.get(pid, ""), tp=tp, gates=gates_now, speed=float(sp[j]), height=float(pos[j, 2]), state=state,
                   captions=cap, x=float(pos[j, 0]), y=float(pos[j, 1]), yaw=float(yaw_arr[j]), phase_tags=tags)
        fr = compose_frame(pan, ctx)
        proc.stdin.write(fr.tobytes())
        frames_written += 1
        for name, want in want_frames.items():
            if name not in keep_k and tp >= want:
                keep_k[name] = (fr.copy(), tp, pid)
    if only_times is None:
        proc.stdin.close()
    rc_ = proc.wait(timeout=600)
    if only_times is not None:
        return {"kept": keep_k, "max_time_difference_s": max_dt}
    return {"frames": frames_written, "seconds": round(frames_written / FPS, 2), "ffmpeg_exit": rc_, "size": [W, H], "max_time_difference_s": {k: round(v, 3) for k, v in max_dt.items()},
            "effects_applied": stats, "kept": keep_k}


def default_phases():
    return {p["id"]: dict(tag="NATIVE", verdict="pending", how="") for p in rc.PHASES}


def _depth_stripes(d, valid):
    rows = float(np.std([d[i][valid[i]].mean() if valid[i].any() else 0.0 for i in range(0, d.shape[0], 8)]))
    cols = float(np.std([d[:, j][valid[:, j]].mean() if valid[:, j].any() else 0.0 for j in range(0, d.shape[1], 8)]))
    return rows, cols


def early_frames(out: Path) -> dict:
    """While the film is still being made: six full frames from the first 15.5 simulated seconds after the take-off command plus one frame of the
    night test shot, and cheap automatic checks. Writes out/early/early_contact.png and early_checks.json."""
    ed = out / "early"
    ed.mkdir(exist_ok=True)
    live = json.loads((out / "live.json").read_text())
    pa = out / "poses_live_ready.npy"
    if pa.is_file():
        (out / "poses_live_copy.npy").write_bytes(pa.read_bytes())
    run = Run(out, child=live, poses_file="poses_live_copy.npy")
    checks = {"problems": []}
    res = {"checks": checks}
    ta = live.get("arm_sim_s")
    labs, imgs = [], []
    if ta is None or not run.has("chase"):
        checks["problems"].append("no pictures or no arm time yet")
    else:
        marks = [1.0, 4.0, 7.0, 10.0, 13.0, 15.5]
        fn = {"passage_log": [], "landing": {}}
        r = build_film(run, fn, {}, default_phases(), out / "x.mp4", {f"m{i}": m for i, m in enumerate(marks)}, only_times=[ta + m for m in marks])
        kept = r.get("kept", {})
        for i, m in enumerate(marks):
            if f"m{i}" in kept:
                imgs.append(kept[f"m{i}"][0])
                labs.append(f"EARLY FRAME t {kept[f'm{i}'][1]:.1f} s")
        grey, depth_ok, seg_ids, stripes = [], [], set(), []
        for m in marks:
            tt = ta + m
            if run.has("fpv"):
                im, _, _ = run.get("fpv", tt)
                grey.append(float(gray(np.asarray(im)).mean()))
            if run.has("depth"):
                d, _, _ = run.get("depth", tt)
                d = np.asarray(d, np.float32)
                valid = np.isfinite(d) & (d > 0)
                v = d[valid]
                depth_ok.append({"valid_fraction": round(float(valid.mean()), 3), "distinct_values": int(len(np.unique(np.round(v, 1)))) if v.size else 0,
                                 "std_m": round(float(v.std()), 2) if v.size else 0.0})
                if v.size:
                    rw_, cl_ = _depth_stripes(d, valid)
                    stripes.append({"row_mean_std": round(rw_, 2), "col_mean_std": round(cl_, 2)})
            if run.has("seg_labels"):
                raw, _, _ = run.get("seg_labels", tt)
                lab = decode_labels(bytes(raw), PH, PW)
                if lab is not None:
                    seg_ids |= set(int(x) for x in np.unique(lab))
        checks["fpv_mean_grey"] = [round(g, 1) for g in grey]
        checks["brightness_ok"] = bool(grey) and all(40.0 <= g <= 225.0 for g in grey)
        checks["depth"] = depth_ok
        checks["depth_is_a_picture"] = bool(depth_ok) and all(x["valid_fraction"] > 0.2 and x["distinct_values"] > 20 for x in depth_ok) and bool(stripes) and all(s["row_mean_std"] > 0.3 and s["col_mean_std"] > 0.3 for s in stripes)
        checks["depth_stripe_check"] = stripes
        checks["label_ids_seen"] = sorted(seg_ids)
        checks["gates_in_view"] = 1 in seg_ids
        checks["labels_more_than_three_classes"] = len(seg_ids) > 3
        for nm in ("fpv", "depth", "seg_labels", "chase"):
            ts = live["panel_times"].get(nm) or []
            if len(ts) > 3:
                span = ts[-1] - ts[0]
                checks.setdefault("pictures_per_sim_second", {})[nm] = round((len(ts) - 1) / span, 2) if span > 0 else None
        pps = checks.get("pictures_per_sim_second", {})
        checks["picture_rate_ok"] = bool(pps) and all(v is not None and 13.0 <= v <= 16.0 for v in pps.values())
    nt = out / "night_test.json"
    if nt.is_file():
        n = json.loads(nt.read_text())
        nrun = Run(out, child={"panel_times": n["panel_times"], "panel_nbytes": n["panel_nbytes"], "arm_sim_s": 0.0}, prefix="nt_")
        if nrun.has("chase") and nrun.has("fpv"):
            tl = float(nrun.times["chase"][-1])
            nrun.t_arm = tl - 60.0
            nrun.child["record_end_sim_s"] = tl + 1
            r = build_film(nrun, {"passage_log": [], "landing": {}}, {}, default_phases(), out / "y.mp4", {"n": -1e9}, only_times=[tl])
            kept = r.get("kept", {})
            if "n" in kept:
                imgs.append(kept["n"][0])
                labs.append("NIGHT TEST SHOT (before the race, drone on the pad)")
                g = float(gray(np.asarray(nrun.get("fpv", tl)[0])).mean())
                checks["night_test_mean_grey"] = round(g, 1)
                checks["night_test_dark"] = g < 90.0
                checks["night_test_frames"] = {k: len(v) for k, v in n["panel_times"].items()}
        else:
            checks["problems"].append("night test: no pictures")
    else:
        checks["problems"].append("no night_test.json")
    if imgs:
        labelled_sheet(imgs, labs, cols=2, tile_w=800).save(ed / "early_contact.png")
    checks["all_ok"] = bool(imgs) and not checks["problems"] and all(checks.get(k) for k in ("brightness_ok", "depth_is_a_picture", "gates_in_view", "picture_rate_ok", "night_test_dark"))
    (ed / "early_checks.json").write_text(json.dumps(res, indent=2, default=str))
    return res


def night_sheet(out: Path) -> dict:
    """The night test shot alone, as soon as night_test.json exists (before the take-off, so it also exists when the take-off never happens).
    Writes out/early/night_test_contact.png (900 px wide) and out/early/night_test_checks.json; the raw pictures are deleted at the end of the job, this is what stays."""
    ed = out / "early"
    ed.mkdir(exist_ok=True)
    nt = out / "night_test.json"
    res = {"problems": []}
    if not nt.is_file():
        res["problems"].append("no night_test.json")
    else:
        n = json.loads(nt.read_text())
        nrun = Run(out, child={"panel_times": n["panel_times"], "panel_nbytes": n["panel_nbytes"], "arm_sim_s": 0.0}, prefix="nt_")
        res["frames"] = {k: len(v) for k, v in n["panel_times"].items()}
        if nrun.has("chase") and nrun.has("fpv"):
            tl = float(nrun.times["chase"][-1])
            nrun.t_arm = tl - 60.0
            nrun.child["record_end_sim_s"] = tl + 1
            r = build_film(nrun, {"passage_log": [], "landing": {}}, {}, default_phases(), out / "y_night.mp4", {"n": -1e9}, only_times=[tl])
            kept = r.get("kept", {})
            if "n" in kept:
                labelled_sheet([kept["n"][0]], ["NIGHT TEST SHOT (before the race, drone on the pad)"], cols=1, tile_w=900).save(ed / "night_test_contact.png")
                res["fpv_mean_grey"] = round(float(gray(np.asarray(nrun.get("fpv", tl)[0])).mean()), 1)
            else:
                res["problems"].append("no frame kept")
        else:
            res["problems"].append("night test: no chase or fpv pictures")
    (ed / "night_test_checks.json").write_text(json.dumps(res, indent=2, default=str))
    return res


def labelled_sheet(images, labels, cols=2, tile_w=800):
    """Grid of pictures, each with its label in a strip ABOVE it (so the label hides nothing of the banner)."""
    h, w = images[0].shape[:2]
    tile_h = int(round(h * tile_w / w))
    strip = 26
    rows = (len(images) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * tile_w, rows * (tile_h + strip)), (30, 30, 30))
    d = ImageDraw.Draw(sheet)
    for k, (im, lb) in enumerate(zip(images, labels)):
        x, y = (k % cols) * tile_w, (k // cols) * (tile_h + strip)
        d.text((x + 6, y + 4), lb, fill=(255, 255, 255), font=font(17))
        sheet.paste(Image.fromarray(im).resize((tile_w, tile_h), Image.BILINEAR), (x, y + strip))
    return sheet


def post(out: Path) -> dict:
    run = Run(out)
    c = run.child
    res = {"id": "FR", "simulator": "px4_gazebo", "errors": []}
    fn = flight_numbers(run)
    ev = measure_effects(run, fn) if run.has("fpv") else {"problem": "no nose camera pictures"}
    ph = decide(run, fn, ev) if run.has("fpv") else {}
    if not ph:
        res["errors"].append("no phase verdicts (no pictures)")
        (out / "film_result.json").write_text(json.dumps({**res, "flight": fn, "measures": ev, "child": {k: c.get(k) for k in ("traceback", "end", "attempts")}}, indent=2, default=str))
        write_summary(out, res, fn, ev, ph, {})
        return res
    mp4 = out / "FR_race.mp4"
    marks = {"takeoff": 1.0, "p1": 5.0, "p2": 16.0, "p3": 25.0, "p4": 35.0, "p5": 45.0, "p6": 55.0}
    td = fn.get("landing", {}).get("touchdown_tp_s")
    marks["landed"] = (td + 1.5) if td is not None else 78.0
    film = build_film(run, fn, ev, ph, mp4, marks)
    kept = film.pop("kept", {})
    sheet_path = out / "FR_contact.png"
    if kept:
        order = [n for n in marks if n in kept]
        imgs = [kept[n][0] for n in order]
        labels = [f"FR {n}  t {kept[n][1]:.1f} s  phase {kept[n][2]}" for n in order]
        sheet = labelled_sheet(imgs, labels, cols=2, tile_w=800)
        sheet.save(sheet_path)
        film["contact_sheet"] = sheet_path.name
        film["contact_frames"] = {n: {"tp_s": round(kept[n][1], 2), "phase": kept[n][2]} for n in order}
        # overview: one row, 4 pictures
        pick = [kept[n][0] for n in ("takeoff", "p3", "p6", "landed") if n in kept]
        if pick:
            tile_w = 480
            f_ = font(22)
            pics = [Image.fromarray(x).resize((tile_w, int(round(x.shape[0] * tile_w / x.shape[1]))), Image.BILINEAR) for x in pick]
            th = pics[0].size[1]
            row = Image.new("RGB", (4 * tile_w, th + 34), (25, 25, 25))
            dd = ImageDraw.Draw(row)
            dd.text((8, 4), f"FR  px4_gazebo  race with every feature:  " + ", ".join(f"{k} {ph[k]['verdict']}" for k in sorted(ph)), fill=(255, 255, 255), font=font(15))
            for i, p in enumerate(pics):
                row.paste(p, (i * tile_w, 34))
            row.save(out / "overview.png")
    for k in sorted(ph):
        w = rc.PHASES[k - 1]
        ph[k].update(id=k, name=w["name"], t0_s=w["t0"], t1_s=(w["t1"] if w["t1"] < 1e8 else None))
    gates_ok = fn.get("gates_passed") == rc.N_GATES and not fn.get("wrong_crossings")
    res.update({
        "title": "The race with every feature (PX4 + Gazebo)", "film": mp4.name if mp4.is_file() else None, "contact_sheet": film.get("contact_sheet"), "overview": "overview.png",
        "film_facts": film, "phases": [ph[k] for k in sorted(ph)], "flight": fn, "measures": ev,
        "race": {"route": "hard_race_v1.json at scale 4 (7 gates, 2 poles, 1 box); path %.1f m, route time %.1f s at %.2f m/s" % (rc.PATH_LEN, rc.T_ROUTE, rc.V_ROUTE),
                 "gates_in_order": gates_ok, "gates_passed": fn.get("gates_passed"), "contacts": fn.get("contacts"), "closest_distance_m": fn.get("closest_distance_m"),
                 "lap_times_s": fn.get("lap_times_s"), "max_tilt_deg": fn.get("max_tilt_deg"), "flight_time_air_s": fn.get("flight_time_air_s")},
        "landing": fn.get("landing"),
        "moving_objects": {"models": "car (body, cabin, wheels), truck (cab, cargo, wheels), person (torso, head, arms, two legs): all built from Gazebo box, cylinder and sphere primitives by px4_gazebo/race_world.py; no download, no licence",
                           "how_they_move": "our script (tests/px4_gazebo_fr_run.py, class Movers) sets their poses with the set_pose_vector service every 0.1 simulated seconds along closed loops: car and truck on the two streets, the person on the south sidewalk (legs swing as a sine); no Gazebo skeletal animation",
                           "loops": {k: {"loop": v["loop"], "speed_m_s": v["speed"]} for k, v in rw.MOVERS.items()}, "calls": c.get("movers")},
        "look": {"ground": "grass, asphalt roads, concrete sidewalks (procedural pictures generated by race_world.make_textures), no checkerboard", "objects": c.get("files", {}).get("city_counts"),
                 "source": "all generated by our code from Gazebo primitives and generated meshes (OBJ) and PNG pictures; nothing downloaded; no licence needed", "world_name": rw.WORLD + " (the file name PX4 expects; the content is our own city world)"},
        "addability": {"tried": True, "feature": "wind, turbulence, rain, camera noise, motion blur, night sky", "donor": "none: Crazyflow's disturbance code was not copied; the turbulence is a fresh Ornstein-Uhlenbeck force noise (sigma %.1f N horizontal, %.1f N vertical, tau %.1f s) applied with Gazebo's own ApplyLinkWrench system"
                       % ((c.get("turbulence") or {}).get("sigma_xy_N", 0), (c.get("turbulence") or {}).get("sigma_z_N", 0), (c.get("turbulence") or {}).get("tau_s", 0)),
                       "ported": ["turbulence force noise (our file tests/px4_gazebo_fr_run.py, class Turbulence, about 40 lines)", "motion blur and fallback noise and rain streaks (px4_gazebo/race_film.py, about 60 lines)", "night sky dome (px4_gazebo/race_world.py dome_obj and make_textures)"],
                       "cloud_runs_needed": None, "result": "see phases", "matches_check": "not compared in this run"},
        "run": {"variant": c.get("variant"), "attempts": c.get("attempts"), "real_time_factor": c.get("real_time_factor"), "record_wall_s": c.get("record_wall_s"),
                "events": c.get("events"), "systems_added": c.get("systems_added"), "create_answers": c.get("create_answers"), "gz_calls": c.get("gz_calls"),
                "image_problems": c.get("image_problems"), "frames_kept": c.get("frames_kept"), "panel_topics": c.get("panel_topics"), "end": c.get("end"), "flight_end_reason": c.get("flight_end_reason"),
                "traceback": c.get("traceback"), "files": c.get("files")},
        "versions": {k: (out / f).read_text().strip() for k, f in (("px4", "px4_version.txt"), ("gazebo", "gz_version.txt")) if (out / f).is_file()},
    })
    (out / "film_result.json").write_text(json.dumps(res, indent=2, default=str))
    write_summary(out, res, fn, ev, ph, film)
    run.arr.clear()
    for p in out.glob("*.raw"):
        try:
            p.unlink(missing_ok=True)
        except OSError:
            pass
    return res


def write_summary(out: Path, res: dict, fn: dict, ev: dict, ph: dict, film: dict):
    L = ["### PX4 + Gazebo race film FR", ""]
    L.append(f"- Film: {res.get('film')}, {film.get('frames')} pictures, {film.get('seconds')} s, size {film.get('size')}, ffmpeg exit {film.get('ffmpeg_exit')}, max time difference between panels {film.get('max_time_difference_s')}")
    L.append(f"- Gates passed in order: {fn.get('gates_passed')} of {fn.get('gates_total')}; wrong crossings {fn.get('wrong_crossings')}; contacts {fn.get('contacts')}; closest {fn.get('closest_distance_m')}")
    L.append(f"- Largest tilt {fn.get('max_tilt_deg')} deg; lap times {fn.get('lap_times_s')}; time in the air {fn.get('flight_time_air_s')} s; landing {fn.get('landing')}")
    r = (res.get("run") or {})
    L.append(f"- Run: variant {r.get('variant')}, real-time factor {r.get('real_time_factor')}, wall time recording {r.get('record_wall_s')} s, end {r.get('end')} / {r.get('flight_end_reason')}, attempts {r.get('attempts')}")
    L.append(f"- Systems added to the running world: {r.get('systems_added')}; create answers {r.get('create_answers')}")
    for k in sorted(ph):
        L.append(f"- Phase {k} {rc.PHASES[k - 1]['name']}: **{ph[k]['tag']}** ({ph[k]['verdict']}): {ph[k]['how']}")
    L.append(f"- Measures: {json.dumps(ev, default=str)[:3000]}")
    L.append(f"- Image problems: {r.get('image_problems')}; frames kept {r.get('frames_kept')}")
    L.append(f"- Errors: {res.get('errors')}; traceback: {str(r.get('traceback'))[-800:]}")
    (out / "summary.md").write_text("\n".join(L) + "\n")


if __name__ == "__main__":
    post(Path(sys.argv[1]).resolve())
