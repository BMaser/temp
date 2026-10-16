"""Make the film, the contact sheets, the overview row and film_result.json of one Project AirSim feature film.

usage: python make_film.py <film id> <out dir>        (out dir holds capture.json and frames/ from pasim_films.py)

Writes: F<nn>_<short>.mp4 (H.264, yuv420p), F<nn>_contact.png (8 evenly spaced frames, labeled), F<nn>_segments.png (the middle frame of
every condition), F<nn>_row.png (4 frames, for overview.png), film_result.json.
The film code (FilmWriter, contact sheet, decoding) is unreal_films.py, the same as in the base laps.
"""

import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
import unreal_films as uf  # noqa: E402
from pasim_films import FILM_DEFS  # noqa: E402

STRIP = 34

# what the document says (features_data.json, answers.project_airsim)
DOC_CLAIMS = {
    "F05": {"feature_id": "D15", "status": "yes",
            "note": "set_time_of_day, set_sun_angle_from_date_time and set_sunlight_intensity.",
            "quote": "Uses the date and time to set the position of the sun in the sky",
            "source": "runs/2026-10-14_feature_matrix/sources/project_airsim/docs_api.md"},
    "F06": {"feature_id": "D16", "status": "yes", "note": "Rain, snow, leaves, dust and fog, density 0 to 1; F10 opens a weather menu.",
            "quote": "Add weather effects based on a parameter type which includes both falling and fallen (ground cover) leaves, rain, and snow as well as dust and fog",
            "source": "runs/2026-10-14_feature_matrix/sources/project_airsim/docs_api.md"},
}


def load(p):
    try:
        return np.asarray(Image.open(p).convert("RGB"))
    except Exception:
        return None


def compose(rec, frames_dir, w, h, film_id, caption):
    """One film frame: drone camera | chase camera, a caption strip below."""
    canvas = Image.new("RGB", (2 * w, h + STRIP), (20, 20, 20))
    d = ImageDraw.Draw(canvas)
    f_small, f_text = uf.font(16), uf.font(18)
    for k, (cam, title) in enumerate((("fpv", "drone camera (nose)"), ("chase", "chase camera"))):
        img = load(frames_dir / rec[cam]) if rec.get(cam) else None
        if img is not None:
            canvas.paste(Image.fromarray(img).resize((w, h)), (k * w, 0))
        else:
            d.text((k * w + 20, h // 2), "no picture", fill=(200, 200, 200), font=f_text)
        box = d.textbbox((k * w + 8, 6), title, font=f_small)
        d.rectangle([box[0] - 4, box[1] - 3, box[2] + 4, box[3] + 3], fill=(0, 0, 0))
        d.text((k * w + 8, 6), title, fill=(255, 255, 255), font=f_small)
    d.text((10, h + 8), f"{film_id}   {caption}   |   simulated time {rec['t']:6.2f} s   |   pair {rec['i']}", fill=(255, 255, 255), font=f_text)
    return np.asarray(canvas)


def region_diff(a, b):
    """Mean absolute difference of two pictures (0..255), on a 160-wide copy."""
    if a is None or b is None:
        return None
    sa = np.asarray(Image.fromarray(a).resize((160, 90))).astype(float)
    sb = np.asarray(Image.fromarray(b).resize((160, 90))).astype(float)
    return float(np.abs(sa - sb).mean())


def main():
    film_id, out = sys.argv[1], Path(sys.argv[2])
    spec = FILM_DEFS[film_id]
    cap = json.loads((out / "capture.json").read_text())
    frames_dir = out / "frames"
    caps = {c[0]: c[1] for c in spec["conditions"]}
    recs = [r for r in cap.get("frames", []) if not r.get("settle") and (r.get("fpv") or r.get("chase"))]
    res = {"film": film_id, "simulator": "project_airsim", "version": "Project AirSim 1.0.1 (Unreal Engine 5), scene Blocks",
           "feature": spec["title"], "capture_stage": cap.get("stage"), "problems": []}
    if not recs:
        res.update({"verdict": "not_shown", "verdict_reason": f"no pictures were saved (capture stage {cap.get('stage')}, "
                    f"error {cap.get('error') or cap.get('traceback')})", "document_claim": DOC_CLAIMS[film_id]})
        (out / "film_result.json").write_text(json.dumps(res, indent=2))
        print("no pictures")
        return 1
    first = load(frames_dir / (recs[0].get("fpv") or recs[0]["chase"]))
    h, w = first.shape[:2]
    w, h = w - w % 2, h - h % 2
    n = len(recs)
    fps = int(min(12, max(4, round(n / 15.0))))
    mp4 = out / f"{film_id}_{spec['short']}.mp4"
    wr = uf.FilmWriter(mp4, 2 * w, h + STRIP, fps, out / "ffmpeg.log")
    composed = []
    for r in recs:
        img = compose(r, frames_dir, w, h, film_id, f"{r['cond'].upper()}  ({caps[r['cond']]})")
        wr.write(img)
        composed.append(img)
    wr.close()
    # contact sheet: 8 evenly spaced frames, labeled
    idx = uf.evenly_spaced_indices(n, 8)
    labels = [f"{recs[i]['cond']}  t={recs[i]['t']:.1f}s  pair {recs[i]['i']}" for i in idx]
    uf.contact_sheet([composed[i] for i in idx], labels, cols=2, max_width=1920).save(out / f"{film_id}_contact.png")
    # the middle frame of every condition
    seg_idx = []
    for c in spec["conditions"]:
        members = [j for j, r in enumerate(recs) if r["cond"] == c[0]]
        if members:
            seg_idx.append(members[len(members) // 2])
    uf.contact_sheet([composed[i] for i in seg_idx], [f"{recs[i]['cond']}: {caps[recs[i]['cond']]}" for i in seg_idx], cols=2,
                     max_width=1920).save(out / f"{film_id}_segments.png")

    # statistics per condition
    stats = {}
    for c in spec["conditions"]:
        rs = [r for r in recs if r["cond"] == c[0]]
        if not rs:
            continue
        stats[c[0]] = {"pairs": len(rs), "sim_t_first": round(rs[0]["t"], 2), "sim_t_last": round(rs[-1]["t"], 2),
                       "fpv_mean": round(float(np.mean([r["fpv_mean"] for r in rs if "fpv_mean" in r] or [float("nan")])), 1),
                       "fpv_std": round(float(np.mean([r["fpv_std"] for r in rs if "fpv_std" in r] or [float("nan")])), 1),
                       "chase_mean": round(float(np.mean([r["chase_mean"] for r in rs if "chase_mean" in r] or [float("nan")])), 1),
                       "chase_std": round(float(np.mean([r["chase_std"] for r in rs if "chase_std" in r] or [float("nan")])), 1)}
    ts = np.array([r["t"] for r in recs])
    dts = np.diff(ts)
    api_ok = {rec["api"]: rec["ok"] for rec in cap.get("api_calls", [])}
    res["settings_used"] = [{"api": r["api"], "args": r["args"], "ok": r["ok"], "result": r["result"], "where": r["where"]}
                            for r in cap.get("api_calls", [])]
    res["numbers"] = {
        "film_file": mp4.name, "film_pictures": n, "playback_fps": fps, "film_seconds": round(n / fps, 1),
        "picture_size_each_camera": [w, h], "simulated_seconds_filmed": round(float(ts[-1] - ts[0]), 2),
        "sim_seconds_per_film_second": round(float((ts[-1] - ts[0]) / (n / fps)), 3) if n > 1 else None,
        "sim_dt_between_pairs_s": {"median": round(float(np.median(dts)), 3), "p95": round(float(np.percentile(dts, 95)), 3),
                                   "max": round(float(dts.max()), 3)} if len(dts) else None,
        "real_time_factor_wanted": cap.get("args", {}).get("rtf"),
        "real_time_factor_measured": (cap.get("timing") or {}).get("real_time_factor"),
        "wall_seconds_film_phase": (cap.get("timing") or {}).get("wall_s"),
        "pairs_per_wall_second": round(n / (cap["timing"]["wall_s"]), 3) if cap.get("timing") and cap["timing"].get("wall_s") else None,
        "picture_failures": cap.get("failures"), "connected_after_s": cap.get("connected_after_s"), "scene_load_s": cap.get("scene_load_s"),
        "takeoff": cap.get("takeoff"), "per_condition": stats, "warmup": cap.get("warmup"), "stop_reason": cap.get("stop_reason")}
    res["document_claim"] = DOC_CLAIMS[film_id]
    res["addability"] = {"tried": False, "donor": None,
                         "note": "nothing had to be added: the simulator's own calls were used (see settings_used)."}

    # ---- verdict, from measurements (the main session reviews the pictures afterwards and may correct it) ----
    if film_id == "F05":
        calls_ok = all(v for k, v in api_ok.items() if k.startswith("world.set_time_of_day"))
        day = stats.get("day", {})
        ratios = {c: (round(stats[c]["fpv_mean"] / day["fpv_mean"], 3) if c in stats and day and day["fpv_mean"] else None)
                  for c in ("dusk", "twilight", "night", "midnight")}
        res["numbers"]["fpv_brightness_over_day"] = ratios
        visible_dark = [c for c in ("night", "midnight") if ratios.get(c) is not None and stats[c]["fpv_std"] > 4.0]
        res["numbers"]["dark_conditions_with_visible_scene_(fpv_std>4)"] = visible_dark
        darkest = min([r for r in ratios.values() if r is not None] or [1.0])
        if calls_ok and darkest <= 0.6 and visible_dark:
            res["verdict"], why = "native", "the time-of-day calls were accepted, the picture gets darker and a night scene is still visible."
        elif calls_ok and darkest <= 0.85:
            res["verdict"], why = "partly", f"the calls were accepted and the picture gets darker (darkest ratio {darkest}), but the night pictures show no scene."
        else:
            res["verdict"], why = "not_shown", f"calls accepted={calls_ok}, darkest brightness ratio={darkest}: no clear change seen."
        res["calibration_evening"] = cap.get("calibration")
        res["lamp_objects_found_in_scene"] = cap.get("lamp_objects")
        res["scene_exploration"] = {k: (v[:40] if isinstance(v, list) else v) for k, v in (cap.get("scene_exploration") or {}).items()}
        res["lights"] = ("no lamp switched on: " + ("lamp-like objects found: %s" % cap.get("lamp_objects")[:10]
                         if cap.get("lamp_objects") else "no object with light/lamp/lantern/street in its name in the Blocks scene"))
        # contact sheet of the calibration pictures
        cal = [(row["time"], frames_dir / f"cal_{row['time'].replace(':', '')}_fpv.jpg") for row in cap.get("calibration", [])]
        cal = [(t, p_) for t, p_ in cal if p_.is_file()]
        if cal:
            uf.contact_sheet([load(p_) for _, p_ in cal], [f"calibration {t} (mean {next(r for r in cap['calibration'] if r['time'] == t).get('fpv_mean')})" for t, _ in cal],
                             cols=3, max_width=1920).save(out / f"{film_id}_calibration.png")
    else:
        clear = [r for r in recs if r["cond"] == "clear"]

        def mean_diff(rs_a, rs_b_by_k, shift=0):
            out_d = {"fpv": [], "chase": []}
            for r in rs_a:
                base = rs_b_by_k.get((r["k"] + shift) % max(1, len(rs_b_by_k)))
                if base is None:
                    continue
                for cam in ("fpv", "chase"):
                    if r.get(cam) and base.get(cam):
                        d = region_diff(load(frames_dir / r[cam]), load(frames_dir / base[cam]))
                        if d is not None:
                            out_d[cam].append(d)
            return {cam: (round(float(np.mean(v)), 2) if v else None) for cam, v in out_d.items()}

        by_k = {i: q for i, q in enumerate(clear)}
        # control: the clear pictures against the clear pictures 5 pairs later (the drone has turned a little) - what no weather change gives
        control = mean_diff(clear, by_k, shift=5)
        res["numbers"]["control_clear_vs_clear_5_pairs_later_0_255"] = control
        kinds = {}
        for c in ("fog", "rain", "snow", "storm", "overcast_try"):
            rs = [r for r in recs if r["cond"] == c]
            kinds[c] = mean_diff(rs, by_k)
        res["numbers"]["mean_abs_diff_to_clear_same_swing_phase_0_255"] = kinds
        # sky blueness (blue minus red in the top 12 % of the drone-camera picture): an overcast sky is grey
        def sky_blue(rs):
            vals = []
            for r in rs:
                if r.get("fpv"):
                    im = load(frames_dir / r["fpv"]).astype(float)
                    top = im[: max(1, im.shape[0] // 8)]
                    vals.append(float((top[..., 2] - top[..., 0]).mean()))
            return round(float(np.mean(vals)), 1) if vals else None
        sky = {c: sky_blue([r for r in recs if r["cond"] == c]) for c in ("clear", "fog", "rain", "snow", "storm", "overcast_try")}
        res["numbers"]["sky_blue_minus_red_top_eighth_fpv"] = sky
        ctl = max([v for v in control.values() if v] or [3.0])
        visible = {c: any((v or 0) > ctl * 1.15 for v in kinds[c].values()) for c in kinds}
        res["numbers"]["visible_change_over_control"] = visible
        accepted = {k: v for k, v in api_ok.items() if "set_weather_visual_effects_param" in k}
        overcast_sky = bool(sky.get("clear") and sky.get("overcast_try") is not None and sky["overcast_try"] < 0.5 * sky["clear"])
        res["numbers"]["overcast_sky_seen"] = overcast_sky
        fr_ok = all(visible.get(c) for c in ("fog", "rain", "snow")) and all(accepted.values())
        if fr_ok and overcast_sky:
            res["verdict"], why = "native", "fog, rain, snow and a grey overcast sky were each seen and every call was accepted."
        elif fr_ok:
            res["verdict"] = "partly"
            why = "fog, rain and snow changed the picture and every call was accepted; no overcast (grey) sky could be made with the documented calls."
        elif any(visible.get(c) for c in ("fog", "rain", "snow")):
            res["verdict"] = "partly"
            why = "visible change for: " + ", ".join(c for c in ("fog", "rain", "snow") if visible.get(c)) + "; not seen for the others."
        else:
            res["verdict"], why = "not_shown", "no measurable change from fog, rain or snow."
        res["overcast"] = ("the documented API has no sky-cover or cloud-cover call (docs_api.md lists set_cloud_shadow_strength, set_sunlight_intensity, "
                           "fog); condition overcast_try combines them and its sky is measured above")
    res["verdict_reason"] = why
    res["verdict_is_automatic"] = True

    # row for the overview: 4 frames
    ridx = uf.evenly_spaced_indices(n, 4)
    tile_w = 480
    tiles = []
    for i in ridx:
        im = Image.fromarray(composed[i])
        tiles.append(im.resize((tile_w, int(im.height * tile_w / im.width))))
    row = Image.new("RGB", (tile_w * 4, tiles[0].height + 26), (0, 0, 0))
    for k, t in enumerate(tiles):
        row.paste(t, (k * tile_w, 26))
    d = ImageDraw.Draw(row)
    d.text((8, 3), f"{film_id}  {spec['title']}   verdict: {res['verdict']}   (automatic)", fill=(255, 255, 255), font=uf.font(18))
    row.save(out / f"{film_id}_row.png")
    # ffprobe facts about the film
    import subprocess
    pr = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=codec_name,pix_fmt,width,height,nb_frames",
                         "-of", "json", str(mp4)], capture_output=True, text=True)
    res["film_probe"] = json.loads(pr.stdout)["streams"][0] if pr.returncode == 0 else {"error": pr.stderr[-200:]}
    (out / "film_result.json").write_text(json.dumps(res, indent=2))
    print(json.dumps({k: res[k] for k in ("verdict", "verdict_reason", "numbers")}, indent=1)[:3000])
    return 0


if __name__ == "__main__":
    sys.exit(main())
