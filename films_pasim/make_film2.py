"""Make the film, contact sheets, overview row and film_result.json of a slice 2 Project AirSim feature film (F01..F04, F07..F11).

usage: python make_film2.py <film id> <out dir> [variant]      (out dir holds capture.json and frames/ from films2.py)
Writes F<nn>_<short>[_variant].mp4 (H.264 yuv420p), F<nn>..._contact.png (8 evenly spaced frames), ..._segments.png (middle frame of
every phase), ..._row.png (4 frames, for overview.png), plots (F07 drift, F08 error, F11 map), film_result.json.
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
import unreal_films as uf  # noqa: E402

TW, TH, STRIP = 480, 270, 34
SHORT = {"F01": "landscape", "F02": "city", "F03": "moving", "F04": "sun_water", "F07": "wind", "F08": "disturbance", "F09": "camera_noise",
         "F10": "motion_blur", "F11": "race"}
TITLE = {"F01": "Natural landscape", "F02": "Cities and buildings", "F03": "Moving objects", "F04": "Sun glare, reflections, shadows", "F07": "Wind",
         "F08": "Turbulence and other disturbances", "F09": "Camera noise", "F10": "Motion blur", "F11": "Hard race route"}
DOC = {
    "F01": ("E11", "yes", "README shows a mountain environment; LandscapeMountains and Forest/MountainVillage scenes appear in release files and docs.",
            "Fixed-wing swarm flying over a mountain environment", "runs/2026-10-14_main_simulator_sources/industry_sims/airsim/projectairsim_readme.md"),
    "F02": ("E12", "yes", "README shows a dense Unreal city; CityEnviron and Neighborhood are release binaries.", "An air taxi operating in a dense Unreal city environment.",
            "runs/2026-10-14_main_simulator_sources/industry_sims/airsim/projectairsim_readme.md"),
    "F03": ("E09", "yes", "Ground vehicles and other robots move beside the drones.", "Simulate UGVs and aerial vehicles together with multiple sensor integration.",
            "runs/2026-10-14_main_simulator_sources/industry_sims/airsim/projectairsim_readme.md"),
    "F04": ("D17", "partial", "Only cloud shadows have a setting; reflections and glare are not mentioned.", "Sets the strength of clouds shadows onto the scene. Disabled by default.",
            "runs/2026-10-14_feature_matrix/sources/project_airsim/docs_api.md"),
    "F07": ("B08", "partial", "Steady wind works in v1.1.0: set it in the scene config or with world.set_wind_velocity; no gusts or Dryden model; the migration page says wind is not implemented.", "",
            "runs/2026-10-15_project_airsim_features/sources/projectairsim_code/..."),
    "F08": ("F06", "partial", "A script can push the drone with set_external_force and set steady wind; no noise on the commands and no command delay were found.",
            "Set arbitrary external force on a vehicle, which can ...", "runs/2026-10-15_project_airsim_features/sources/projectairsim_code/client__python__projectairsim__src__projectairsim__robot.py"),
    "F09": ("D18", "yes", "Camera noise-settings block per image type (random noise, horizontal lines, waves, distortion); off by default; pattern noise, not a shot-noise model.", "",
            "runs/2026-10-15_project_airsim_features/sources/projectairsim_code/unreal__Blocks__Plugins__ProjectAirSim__Source__ProjectAirSim__Private__Sensors__UnrealCamera.cpp"),
    "F10": ("D19", "partial", "motion-blur-amount camera setting; the migration table says AirSim's MotionBlurAmount is not implemented. No rolling shutter.",
            "Blurs objects based on it's motion determined using velocity maps.", "runs/2026-10-14_feature_matrix/sources/project_airsim/docs_sensors_camera_capture_settings.md"),
    "F11": ("E01-E05 + D02 + D03", "mixed", "E01 gates not_found, E02 racing tracks not_found, E03 gate passing not_found; E04/E05 obstacles yes; D02 depth yes; D03 segmentation yes.", "",
            "runs/2026-10-14_feature_matrix/features_data.json"),
}


def load_img(p):
    try:
        return np.asarray(Image.open(p).convert("RGB"))
    except Exception:
        return None


def tile(path, w=TW, h=TH, label=None, fallback="no picture"):
    img = load_img(path) if path else None
    im = Image.fromarray(img).resize((w, h)) if img is not None else Image.new("RGB", (w, h), (25, 25, 25))
    d = ImageDraw.Draw(im)
    if img is None:
        d.text((16, h // 2), fallback, fill=(200, 200, 200), font=uf.font(16))
    if label:
        f = uf.font(15)
        b = d.textbbox((6, 5), label, font=f)
        d.rectangle([b[0] - 3, b[1] - 2, b[2] + 3, b[3] + 2], fill=(0, 0, 0))
        d.text((6, 5), label, fill=(255, 255, 255), font=f)
    return np.asarray(im)


def strip(width, text):
    im = Image.new("RGB", (width, STRIP), (20, 20, 20))
    ImageDraw.Draw(im).text((10, 8), text, fill=(255, 255, 255), font=uf.font(16))
    return np.asarray(im)


def fig_to_img(fig, w, h):
    fig.canvas.draw()
    arr = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
    plt.close(fig)
    return np.asarray(Image.fromarray(arr).resize((w, h)))


def even(n):
    return n - n % 2


def write_film(path, frames, fps, log):
    h, w = frames[0].shape[:2]
    wr = uf.FilmWriter(path, even(w), even(h), fps, log)
    for fr in frames:
        wr.write(np.ascontiguousarray(fr[: even(h), : even(w)]))
    wr.close()


def sheets(out, prefix, frames, labels, seg_idx, seg_labels):
    idx = uf.evenly_spaced_indices(len(frames), 8)
    uf.contact_sheet([frames[i] for i in idx], [labels[i] for i in idx], cols=2, max_width=1920).save(out / f"{prefix}_contact.png")
    if seg_idx:
        uf.contact_sheet([frames[i] for i in seg_idx], seg_labels, cols=2, max_width=1920).save(out / f"{prefix}_segments.png")
    ridx = uf.evenly_spaced_indices(len(frames), 4)
    return ridx


def row_png(out, prefix, frames, ridx, text):
    tw = 480
    tiles = [Image.fromarray(frames[i]).resize((tw, int(frames[i].shape[0] * tw / frames[i].shape[1]))) for i in ridx]
    row = Image.new("RGB", (tw * 4, tiles[0].height + 26), (0, 0, 0))
    for k, t in enumerate(tiles):
        row.paste(t, (k * tw, 26))
    ImageDraw.Draw(row).text((8, 3), text, fill=(255, 255, 255), font=uf.font(18))
    row.save(out / f"{prefix}_row.png")


# ------------------------------------------------------------------ geometry for F11
def gate_local(g, p):
    """position p (NED) in the gate frame: (forward, left_ENU_like, down-offset)."""
    yaw = g["yaw"]
    d = (p[0] - g["center"][0], p[1] - g["center"][1], p[2] - g["center"][2])
    f = d[0] * math.cos(yaw) + d[1] * math.sin(yaw)
    lat = -d[0] * math.sin(yaw) + d[1] * math.cos(yaw)
    return f, lat, d[2]


def aabb_dist(p, half, c=(0, 0, 0)):
    q = [max(abs(p[i] - c[i]) - half[i], 0.0) for i in range(3)]
    return math.sqrt(sum(v * v for v in q))


def race_metrics(cap, recs):
    geo = cap["race"]["geometry"]
    sc, opening, outer = geo["scale"], geo["opening"], geo["outer"]
    bar = 0.1 * sc
    pos = [(r["n"], r["e"], r["d"]) for r in recs]
    ts = [r["t_abs"] for r in recs]
    passed, order = {}, []
    closest = {g["id"] + "_frame": 1e9 for g in geo["gates"]}
    closest.update({o["id"]: 1e9 for o in geo["obstacles"]})
    for a, b, ta, tb in zip(pos[:-1], pos[1:], ts[:-1], ts[1:]):
        for sub in range(21):
            u = sub / 20.0
            p = tuple(a[i] + u * (b[i] - a[i]) for i in range(3))
            for g in geo["gates"]:
                f, lat, dz = gate_local(g, p)
                # frame bars as boxes in the gate frame: half extents (depth bar/2, ...)
                bars = [((0, 0, -(opening / 2 + bar / 2)), (bar / 2, outer / 2, bar / 2)), ((0, 0, opening / 2 + bar / 2), (bar / 2, outer / 2, bar / 2)),
                        ((0, opening / 2 + bar / 2, 0), (bar / 2, bar / 2, opening / 2)), ((0, -(opening / 2 + bar / 2), 0), (bar / 2, bar / 2, opening / 2))]
                dm = min(aabb_dist((f, lat, dz), half, c) for c, half in bars)
                closest[g["id"] + "_frame"] = min(closest[g["id"] + "_frame"], dm)
            for o in geo["obstacles"]:
                if o["kind"].startswith("vertical"):
                    d = max(math.hypot(p[0] - o["xy"][0], p[1] - o["xy"][1]) - o["radius"], 0.0) if -p[2] < o["height"] else 1e9
                else:
                    d = aabb_dist(p, (o["size"] / 2, o["size"] / 2, o["height"] / 2), (o["xy"][0], o["xy"][1], -o["height"] / 2))
                closest[o["id"]] = min(closest[o["id"]], d)
        for g in geo["gates"]:
            if g["id"] in passed:
                continue
            f0, l0, z0 = gate_local(g, a)
            f1, l1, z1 = gate_local(g, b)
            if f0 < 0 <= f1:
                u = -f0 / (f1 - f0)
                lat, dz = l0 + u * (l1 - l0), z0 + u * (z1 - z0)
                inside = abs(lat) < opening / 2 and abs(dz) < opening / 2
                passed[g["id"]] = {"t_sim_s": round(ta + u * (tb - ta), 2), "lateral_m": round(lat, 2), "vertical_m": round(dz, 2), "through_opening": bool(inside)}
                if inside:
                    order.append(g["id"])
    wanted = [g["id"] for g in geo["gates"]]
    in_order = []
    k = 0
    for gid in order:
        if k < len(wanted) and gid == wanted[k]:
            in_order.append(gid)
            k += 1
    return {"gates_passed_through_opening_in_order": in_order, "gates_crossed": passed, "closest_m_to_each_object": {k_: round(v, 2) for k_, v in closest.items()},
            "contact_threshold_m": 0.3, "contacts_inferred": [k_ for k_, v in closest.items() if v < 0.3],
            "lap_time_sim_s": round(ts[-1] - ts[0], 1) if ts else None, "path_flown_m": round(sum(math.dist(a, b) for a, b in zip(pos[:-1], pos[1:])), 1)}


# ------------------------------------------------------------------ main
def main():
    film, out = sys.argv[1], Path(sys.argv[2])
    variant = sys.argv[3] if len(sys.argv) > 3 else "default"
    vtag = "" if variant == "default" else f"_{variant}"
    prefix = f"{film}{vtag}"
    cap = json.loads((out / "capture.json").read_text())
    fd = out / "frames"
    recs_all = [r for r in cap.get("frames", []) if not r.get("settle")]
    res = {"film": film, "variant": variant, "simulator": "project_airsim", "version": "Project AirSim 1.0.1 (Unreal Engine 5)", "feature": TITLE[film],
           "capture_stage": cap.get("stage"), "problems": []}
    did, st, note, quote, src = DOC[film]
    res["document_claim"] = {"feature_id": did, "status": st, "note": note, "quote": quote, "source": src}
    res["settings_used"] = [{"api": r["api"], "args": r["args"][:4], "ok": r["ok"], "result": r["result"][:120], "where": r["where"]} for r in cap.get("api_calls", [])]
    if not recs_all:
        res.update({"verdict": "not_shown", "verdict_reason": f"no pictures saved (stage {cap.get('stage')}, error {cap.get('error') or (cap.get('traceback') or '')[-400:]})"})
        (out / "film_result.json").write_text(json.dumps(res, indent=2))
        print("no pictures")
        return 1
    for r in recs_all:
        r["t"] = r["t_abs"]
    log = out / "ffmpeg.log"
    mp4 = out / f"{prefix}_{SHORT[film]}.mp4"
    frames, labels, seg_idx, seg_labels = [], [], [], []
    phases = []
    for r in recs_all:
        if r["phase"] not in phases:
            phases.append(r["phase"])
    extra = {}

    if film in ("F01", "F02", "F03", "F04"):
        cap_for = {"fly": "forward flight", "moving": "other drones fly their own paths", "shadows_and_glare": "sun low, nose turns a full circle", "water": "spawned water plane, same turn"}
        for r in recs_all:
            f1 = tile(fd / r["fpv"] if r.get("fpv") else None, 640, 360, "drone camera (nose)")
            f2 = tile(fd / r["chase"] if r.get("chase") else None, 640, 360, "chase camera")
            body = np.concatenate([f1, f2], axis=1)
            frames.append(np.concatenate([body, strip(body.shape[1], f"{film} {r['phase']}: {cap_for.get(r['phase'], '')}  |  simulated time {r['t']:.2f} s  |  pair {r['i']}")], axis=0))
            labels.append(f"{r['phase']} t={r['t']:.1f}s")
    elif film in ("F07", "F09", "F10"):
        modes = phases
        by = {m: [r for r in recs_all if r["phase"] == m] for m in modes}
        n = min(len(v) for v in by.values()) if by else 0
        names = {"off": "feature OFF", "on": "feature ON", "wind_api": "wind: world.set_wind_velocity 5 m/s east", "force": "wind: our external force"}
        if film == "F07":
            base = by.get("off", [])
            ref = (base[0]["n"], base[0]["e"]) if base else (0, 0)
        for j in range(n):
            tiles = [tile(fd / by[m][j]["fpv"] if by[m][j].get("fpv") else None, TW if film == "F07" else 640, TH if film == "F07" else 360, names.get(m, m)) for m in modes]
            body = np.concatenate(tiles, axis=1)
            parts = [body]
            if film == "F07":
                fig, ax = plt.subplots(1, 2, figsize=(body.shape[1] / 100, 2.7))
                for m, colr in zip(modes, ("tab:gray", "tab:red", "tab:blue")):
                    rr = by[m][: j + 1]
                    t0 = by[m][0]["t_abs"]
                    ax[0].plot([r["t_abs"] - t0 for r in rr], [r["e"] - by[m][0]["e"] for r in rr], color=colr, label=names.get(m, m))
                    ax[1].plot([r["e"] - by[m][0]["e"] for r in rr], [r["n"] - by[m][0]["n"] for r in rr], color=colr)
                ax[0].set_xlabel("simulated time (s)")
                ax[0].set_ylabel("east drift (m)")
                ax[0].legend(fontsize=7, loc="upper left")
                ax[0].grid(alpha=0.3)
                ax[1].set_xlabel("east (m)")
                ax[1].set_ylabel("north (m)")
                ax[1].grid(alpha=0.3)
                ax[1].set_aspect("equal", adjustable="datalim")
                fig.tight_layout()
                parts.append(fig_to_img(fig, body.shape[1], 270))
            parts.append(strip(body.shape[1], f"{film}   simulated time {by[modes[0]][j]['t']:.2f} s (phase 1)   |   pair {j}"))
            frames.append(np.concatenate(parts, axis=0))
            labels.append(f"pair {j} ({by[modes[0]][j]['t']:.1f}s)")
        # numbers
        if film == "F07":
            drift = {}
            for m in modes:
                rr = by[m]
                drift[m] = {"east_drift_end_m": round(rr[-1]["e"] - rr[0]["e"], 2), "north_travel_m": round(rr[-1]["n"] - rr[0]["n"], 2),
                            "max_abs_east_drift_m": round(max(abs(r["e"] - rr[0]["e"]) for r in rr), 2), "sim_seconds": round(rr[-1]["t_abs"] - rr[0]["t_abs"], 2)}
            extra["drift"] = drift
            fig, ax = plt.subplots(1, 2, figsize=(11, 4))
            for m, colr in zip(modes, ("tab:gray", "tab:red", "tab:blue")):
                rr = by[m]
                ax[0].plot([r["t_abs"] - rr[0]["t_abs"] for r in rr], [r["e"] - rr[0]["e"] for r in rr], color=colr, label=names.get(m, m))
                ax[1].plot([r["e"] - rr[0]["e"] for r in rr], [r["n"] - rr[0]["n"] for r in rr], color=colr)
            ax[0].set_xlabel("simulated time (s)")
            ax[0].set_ylabel("east drift (m)")
            ax[0].legend(fontsize=8)
            ax[1].set_xlabel("east (m)")
            ax[1].set_ylabel("north (m)")
            ax[1].set_aspect("equal", adjustable="datalim")
            for x in ax:
                x.grid(alpha=0.3)
            fig.suptitle("F07 wind: drift across a straight flight north at 1.5 m/s")
            fig.tight_layout()
            fig.savefig(out / f"{prefix}_drift.png", dpi=110)
            plt.close(fig)
        else:
            def hf(path):
                a_ = load_img(path)
                if a_ is None:
                    return None
                g = a_.astype(float).mean(axis=2)
                lap = g[1:-1, 1:-1] * 4 - g[:-2, 1:-1] - g[2:, 1:-1] - g[1:-1, :-2] - g[1:-1, 2:]
                return float(lap.std()), float(lap.var())
            met = {}
            for m in modes:
                vals = [hf(fd / r["fpv"]) for r in by[m] if r.get("fpv")]
                vals = [v for v in vals if v]
                met[m] = {"laplacian_std_mean": round(float(np.mean([v[0] for v in vals])), 2) if vals else None,
                          "laplacian_var_mean": round(float(np.mean([v[1] for v in vals])), 1) if vals else None}
            extra["sharpness_or_noise_metric"] = met
    elif film == "F08":
        n0, e0, d0 = cap.get("hold_point_ned", [recs_all[0]["n"], recs_all[0]["e"], recs_all[0]["d"]])
        err = [math.sqrt((r["n"] - n0) ** 2 + (r["e"] - e0) ** 2 + (r["d"] - d0) ** 2) for r in recs_all]
        for j, r in enumerate(recs_all):
            f1 = tile(fd / r["fpv"] if r.get("fpv") else None, 640, 360, "drone camera (nose)")
            f2 = tile(fd / r["chase"] if r.get("chase") else None, 640, 360, "chase camera")
            fig, ax = plt.subplots(figsize=(6.4, 3.6))
            t0 = recs_all[0]["t_abs"]
            ax.plot([q["t_abs"] - t0 for q in recs_all[: j + 1]], err[: j + 1], color="tab:red")
            sw = next((q["t_abs"] - t0 for q in recs_all if q["phase"] == "random_force"), None)
            if sw is not None:
                ax.axvline(sw, color="k", linestyle="--", lw=1)
                ax.text(sw, ax.get_ylim()[1] * 0.9, " random force on", fontsize=8)
            ax.set_xlim(0, recs_all[-1]["t_abs"] - t0)
            ax.set_ylim(0, max(err) * 1.1 + 0.05)
            ax.set_xlabel("simulated time (s)")
            ax.set_ylabel("position error from hold point (m)")
            ax.grid(alpha=0.3)
            fig.tight_layout()
            f3 = fig_to_img(fig, 640, 360)
            body = np.concatenate([f1, f2, f3], axis=1)
            frames.append(np.concatenate([body, strip(body.shape[1], f"F08 {r['phase']}  |  simulated time {r['t']:.2f} s  |  error {err[j]:.2f} m")], axis=0))
            labels.append(f"{r['phase']} t={r['t']:.1f}s err {err[j]:.2f} m")
        a_ = [e for e, r in zip(err, recs_all) if r["phase"] == "no_disturbance"]
        b_ = [e for e, r in zip(err, recs_all) if r["phase"] == "random_force"]
        extra["position_error_m"] = {"no_disturbance_mean": round(float(np.mean(a_)), 3) if a_ else None, "no_disturbance_max": round(max(a_), 3) if a_ else None,
                                     "random_force_mean": round(float(np.mean(b_)), 3) if b_ else None, "random_force_max": round(max(b_), 3) if b_ else None,
                                     "random_force_rms": round(float(np.sqrt(np.mean(np.square(b_)))), 3) if b_ else None}
        extra["disturbance"] = cap.get("disturbance")
        fig, ax = plt.subplots(figsize=(8, 3.5))
        ax.plot([r["t_abs"] - recs_all[0]["t_abs"] for r in recs_all], err, color="tab:red")
        ax.set_xlabel("simulated time (s)")
        ax.set_ylabel("position error (m)")
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / f"{prefix}_error.png", dpi=110)
        plt.close(fig)
    elif film == "F11":
        geo = cap["race"]["geometry"]
        met = race_metrics(cap, recs_all)
        extra["race"] = met
        extra["race_scale"] = cap["race"]["scale"]
        extra["course_objects_spawned"] = len(cap.get("course_objects", []))
        for j, r in enumerate(recs_all):
            t1 = tile(fd / r["fpv"] if r.get("fpv") else None, TW, TH, "nose camera: scene")
            t2 = tile(fd / r["fpv_t4"] if r.get("fpv_t4") else None, TW, TH, "depth (visualisation)")
            t3 = tile(fd / r["fpv_t3"] if r.get("fpv_t3") else None, TW, TH, "segmentation (object labels)")
            t4 = tile(fd / r["chase"] if r.get("chase") else None, TW, TH, "chase camera")
            fig, ax = plt.subplots(figsize=(4.8, 2.7))
            for g in geo["gates"]:
                cn, ce = g["center"][0], g["center"][1]
                lat = (-math.sin(g["yaw"]), math.cos(g["yaw"]))
                ax.plot([ce - lat[1] * geo["outer"] / 2, ce + lat[1] * geo["outer"] / 2], [cn - lat[0] * geo["outer"] / 2, cn + lat[0] * geo["outer"] / 2], color="tab:orange", lw=3)
                ax.text(ce, cn, g["id"], fontsize=6)
            for o in geo["obstacles"]:
                ax.plot(o["xy"][1], o["xy"][0], "s" if not o["kind"].startswith("vertical") else "o", color="tab:blue", ms=5)
            ax.plot([q["e"] for q in recs_all[: j + 1]], [q["n"] for q in recs_all[: j + 1]], color="tab:red", lw=1.5)
            ax.plot(r["e"], r["n"], "ko", ms=4)
            ax.set_aspect("equal", adjustable="datalim")
            ax.set_xlabel("east (m)", fontsize=7)
            ax.set_ylabel("north (m)", fontsize=7)
            ax.tick_params(labelsize=6)
            ax.grid(alpha=0.3)
            fig.tight_layout()
            t5 = fig_to_img(fig, TW, TH)
            txt = Image.new("RGB", (TW, TH), (20, 20, 20))
            d = ImageDraw.Draw(txt)
            passed_now = [gid for gid, v in met["gates_crossed"].items() if v["through_opening"] and v["t_sim_s"] <= r["t_abs"]]
            d.text((10, 10), f"scale {cap['race']['scale']:g}   t {r['t']:.1f} s", fill=(255, 255, 255), font=uf.font(16))
            d.text((10, 40), "gates passed: " + (" ".join(passed_now) if passed_now else "-"), fill=(255, 200, 80), font=uf.font(16))
            d.text((10, 70), f"height {-r['d']:.1f} m", fill=(255, 255, 255), font=uf.font(16))
            row1 = np.concatenate([t1, t2, t3], axis=1)
            row2 = np.concatenate([t4, t5, np.asarray(txt)], axis=1)
            frames.append(np.concatenate([row1, row2, strip(row1.shape[1], f"F11 hard_race_v1 at scale {cap['race']['scale']:g}   |   simulated time {r['t']:.2f} s   |   pair {r['i']}")], axis=0))
            labels.append(f"t={r['t']:.1f}s")
    n = len(frames)
    fps = int(min(12, max(8, round(n / 14.0))))
    write_film(mp4, frames, fps, log)
    if not seg_idx:
        for ph in phases:
            idxs = [j for j, r in enumerate(recs_all) if r["phase"] == ph]
            if idxs and len(idxs) <= len(frames):
                seg_idx.append(min(idxs[len(idxs) // 2], len(frames) - 1))
        seg_labels = [f"{phases[i]}" for i in range(len(seg_idx))]
    ridx = sheets(out, prefix, frames, labels, seg_idx, seg_labels if seg_idx else [])
    ts = np.array([r["t_abs"] for r in recs_all])
    dts = np.diff(ts)
    res["numbers"] = {"film_file": mp4.name, "film_pictures": n, "playback_fps": fps, "film_seconds": round(n / fps, 1), "frame_size": list(frames[0].shape[1::-1]),
                      "simulated_seconds_filmed": round(float(ts[-1] - ts[0]), 2), "sim_dt_between_pairs_s": {"median": round(float(np.median(dts)), 3) if len(dts) else None,
                      "max": round(float(dts.max()), 3) if len(dts) else None}, "wall_seconds_total": cap.get("wall_total_s"),
                      "wall_seconds_per_pair": round(float(np.median(np.diff([r["wall_s"] for r in recs_all]))), 2) if len(recs_all) > 2 else None,
                      "picture_failures": cap.get("failures"), "phases": cap.get("phases"), "takeoff": cap.get("takeoff"), "stop_reason": cap.get("stop_reason"), **extra}
    res["config"] = cap.get("config")
    res["scene_exploration"] = {k: (v[:60] if isinstance(v, list) else v) for k, v in (cap.get("scene_exploration") or {}).items()}
    res["nature_notes"] = cap.get("nature_notes")
    res["addability"] = {"tried": False, "donor": None, "note": "see verdict_reason"}
    # ---- automatic verdict (the main session reviews the contact sheet and may correct it through verdicts.json) ----
    v, why = "not_shown", "no rule matched"
    if film == "F03":
        moved = {}
        for nm in ("ActorWithConfigTraj", "TiltrotorWithConfigTrajOffset"):
            pts = [r[nm] for r in recs_all if r.get(nm)]
            moved[nm] = round(max(math.dist(pts[0], q) for q in pts), 2) if len(pts) > 1 else None
        res["numbers"]["environment_actor_travel_m"] = moved
        ok = [m for m in moved.values() if m and m > 3.0]
        v, why = ("native", f"{len(ok)} environment actors moved by themselves (config trajectories of the scene); travel {moved}") if len(ok) >= 2 else ("partly", f"actor travel {moved}")
    elif film == "F07":
        d = extra["drift"]
        api = d.get("wind_api", {}).get("max_abs_east_drift_m", 0)
        frc = d.get("force", {}).get("max_abs_east_drift_m", 0)
        off = d.get("off", {}).get("max_abs_east_drift_m", 0)
        if api > max(1.0, 3 * off):
            v, why = "native", f"world.set_wind_velocity moved the drone {api} m across the flight (no wind: {off} m)"
        elif frc > max(1.0, 3 * off):
            v, why = "added_by_our_code", f"set_wind_velocity gave {api} m drift (no effect in v1.0.1); our external-force wind gave {frc} m (no wind: {off} m)"
            res["addability"] = {"tried": True, "donor": "none needed: shipped example wind.py uses set_external_force", "ours": "films_pasim/films2.py film_wind (linear drag force toward the wind velocity)", "result": "worked"}
        else:
            v, why = "not_shown", f"drift api {api} m, force {frc} m, none {off} m"
    elif film == "F08":
        pe = extra["position_error_m"]
        v, why = ("added_by_our_code", f"Gaussian random force every pair (port of Crazyflow's disturbance_fn idea); position error rms {pe['random_force_rms']} m vs {pe['no_disturbance_mean']} m before") if (pe["random_force_mean"] or 0) > 2 * (pe["no_disturbance_mean"] or 0.01) else ("partly", f"error {pe}")
        res["addability"] = {"tried": True, "donor": "crazyflow (MIT) examples/plugins/disturbance.py", "copied": "nothing copied; JAX code, rewritten in plain Python (about 15 lines in films2.py film_disturb)",
                             "ported_idea": "zero-mean Gaussian force each step; torque not possible (no torque call)", "cloud_runs_needed": 1, "result": "worked" if v == "added_by_our_code" else "partly",
                             "matches_check": "check project_airsim.json F06 says verdict plug_in, effort small: yes (set_external_force from a script, one call per step)"}
    elif film in ("F09", "F10"):
        m = extra["sharpness_or_noise_metric"]
        off, on = m.get("off", {}).get("laplacian_std_mean"), m.get("on", {}).get("laplacian_std_mean")
        if off and on:
            ratio = on / off
            if film == "F09":
                v, why = ("native", f"high-frequency energy x{ratio:.2f} with noise on") if ratio > 1.3 else ("not_shown", f"high-frequency energy ratio on/off {ratio:.2f}: no visible noise")
            else:
                v, why = ("native", f"picture {1 / max(ratio, 1e-6):.2f}x less sharp with blur on") if ratio < 0.85 else ("partly", f"setting accepted; sharpness ratio on/off {ratio:.2f} (no visible blur at the slowed clock)")
    elif film == "F11":
        mm = extra["race"]
        n_ok = len(mm["gates_passed_through_opening_in_order"])
        v, why = "added_by_our_code", f"gates, poles and box built from packaged cubes and cylinders by our code; {n_ok} of 7 gates passed through the opening in order; contacts (inferred, <0.3 m): {mm['contacts_inferred']}"
        res["addability"] = {"tried": True, "donor": "none (the simulator has no gate asset: checks E01 not_found)", "ours": "films_pasim/films2.py spawn_course (about 45 lines)", "result": "worked" if n_ok else "partly"}
    elif film == "F01":
        n_sp = len([r for r in cap.get("api_calls", []) if r["api"].startswith("world.spawn_object") and r["ok"]])
        v, why = ("added_by_our_code", f"checkerboard floor handled by variant '{variant}'; {n_sp} primitives spawned as trees, rocks, bushes") if n_sp else ("partly", "no objects spawned")
    elif film == "F02":
        v, why = "native", "the packaged scene shows buildings (check the contact sheet)"
    elif film == "F04":
        v, why = "partly", "shadows, low sun glare and a spawned water plane; see the contact sheet"
    res["verdict"], res["verdict_reason"], res["verdict_is_automatic"] = v, why, True
    ovf = Path(__file__).resolve().parent / "verdicts.json"
    if ovf.is_file():
        ov = json.loads(ovf.read_text()).get(film + ("" if variant == "default" else "@" + variant))
        if ov:
            res["verdict_automatic"] = {"verdict": v, "reason": why}
            res["verdict"], res["verdict_reason"], res["verdict_is_automatic"] = ov["verdict"], ov["reason"], False
            res["verdict_reviewed"] = ov.get("reviewed")
            res["notes"] = ov.get("notes")
    row_png(out, prefix, frames, ridx, f"{prefix}  {TITLE[film]}   verdict: {res['verdict']}{'' if res['verdict_is_automatic'] else ' (reviewed)'}")
    try:
        pr = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=codec_name,pix_fmt,width,height,nb_frames", "-of", "json", str(mp4)],
                            capture_output=True, text=True)
        res["film_probe"] = json.loads(pr.stdout)["streams"][0] if pr.returncode == 0 else {"error": pr.stderr[-200:]}
    except Exception as e:  # noqa: BLE001
        res["film_probe"] = {"error": repr(e)}
    (out / "film_result.json").write_text(json.dumps(res, indent=2, default=str))
    print(json.dumps({k: res[k] for k in ("verdict", "verdict_reason")}, indent=1))
    print(json.dumps(res["numbers"], default=str)[:2500])
    return 0


if __name__ == "__main__":
    sys.exit(main())
