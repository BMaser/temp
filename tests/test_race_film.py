"""Offline checks of px4_gazebo/race_film.py with a SYNTHETIC run (made-up pictures, a pose log that follows the route and lands): every function of the
post-processing runs, the composed frame has the right size and parts, the verdict logic reacts to the measurements the way it should, and the film
facts and film_result.json are written. No Gazebo, no ffmpeg (RACE_FILM_NO_FFMPEG=1).
Run: python3 tests/test_race_film.py [out_dir_for_the_preview_pngs]"""

import json
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

os.environ["RACE_FILM_NO_FFMPEG"] = "1"
REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "px4_gazebo"))
import race_core as rc  # noqa: E402
import race_film as rf  # noqa: E402
import race_world as rw  # noqa: E402

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    if ok:
        passed += 1
    else:
        failed += 1
        print(f"FAIL {name} {detail}")


def make_run(td: Path, rain_visible: bool, noise_visible: bool, dark: bool):
    rng = np.random.default_rng(1)
    PW, PH = rw.PW, rw.PH
    t_arm = 100.0
    t_route = t_arm + 6.0
    # poses at 50 Hz: climb, route, descent, landing
    rows = []
    for t in np.arange(t_arm - 3.0, t_arm + 6.0 + rc.T_ROUTE + 12.0, 0.02):
        tp = t - t_arm
        if tp < 6.0:
            p = np.array([0.0, 0.0, max(0.0, min(rc.HOVER_Z, 0.8 * tp))])
        elif tp < 6.0 + rc.T_ROUTE:
            p = rc.SPLINE(tp - 6.0)
        else:
            z = max(0.0, rc.HOVER_Z - 0.5 * (tp - 6.0 - rc.T_ROUTE))
            p = np.array([rc.END_PAD[0], rc.END_PAD[1], z])
        rows.append([t, *p, 0.0, 0.0, 0.0, 1.0])
    poses = np.array(rows)
    np.save(td / "poses.npy", poses)
    # pictures: every 0.2 s from t_arm - 1 to the end
    times = np.arange(t_arm - 1.0, poses[-1, 0], 0.2)
    n = len(times)
    nbytes = {}

    def scene(k, tp):
        img = np.zeros((PH, PW, 3), np.uint8)
        img[: PH // 2] = (120, 170, 235)
        img[PH // 2:] = (70, 110, 50)
        for j in range(8):
            x0 = (j * 53 + int(tp * 7)) % (PW - 40)
            img[PH // 2 - 30: PH // 2 + 10, x0: x0 + 30] = (150, 150, 160)
        if dark and tp > 50:
            img = (img * 0.25).astype(np.uint8)
        img[20:60, 300:340] = 255  # the sun
        return img
    f = open(td / "fpv.raw", "wb")
    fn_ = open(td / "fpvn.raw", "wb")
    fc_ = open(td / "chase.raw", "wb")
    fd = open(td / "depth.raw", "wb")
    fl = open(td / "seg_labels.raw", "wb")
    tn = []
    for tt in times:
        tp = tt - t_arm
        img = scene(0, tp)
        if rain_visible and 20 <= tp < 30:
            ys = rng.integers(0, PH, 2500)
            xs = rng.integers(0, PW, 2500)
            img[ys, xs] = 255
        f.write(img.tobytes())
        fc_.write(scene(1, tp).tobytes())
        nz = img.copy()
        if noise_visible:
            nz = np.clip(nz.astype(np.int16) + rng.normal(0, 25, nz.shape), 0, 255).astype(np.uint8)
        fn_.write(nz.tobytes())
        d = np.tile(np.linspace(1.0, 70.0, PW, dtype=np.float32), (PH, 1))
        d[:20] = np.inf
        fd.write(d.astype("<f4").tobytes())
        lab = np.zeros((PH, PW), np.int16)
        lab[PH // 2:] = 15
        lab[PH // 2 - 30: PH // 2 + 10, 100:160] = 4
        lab[PH // 2 + 20: PH // 2 + 50, 200:230] = 8
        lab[PH // 2 + 60:, 10:80] = 14
        fl.write(lab.astype("<i2").tobytes())
    for fh in (f, fn_, fc_, fd, fl):
        fh.close()
    tl = [round(float(x), 4) for x in times]
    child = {"ground_reference_world": [0.0, 0.0, 0.0], "arm_sim_s": t_arm, "route_start_sim_s": t_route, "record_start_sim_s": float(times[0]),
             "record_end_sim_s": float(times[-1]), "landed": True, "land_cmd_sim_s": t_arm + 6.0 + rc.T_ROUTE + 6.0, "disarm_sim_s": t_arm + 6.0 + rc.T_ROUTE + 10.0,
             "panel_times": {"fpv": tl, "fpvn": tl, "chase": tl, "depth": tl, "seg_labels": tl},
             "panel_nbytes": {"fpv": PW * PH * 3, "fpvn": PW * PH * 3, "chase": PW * PH * 3, "depth": PW * PH * 4, "seg_labels": PW * PH * 2},
             "systems_added": {"ParticleEmitter": True, "WindEffects": True}, "particle_service_seen": True, "movers": {"calls": 100, "ok": 100, "max_gap_sim_s": 0.15},
             "turbulence": {"updates": 100, "sigma_xy_N": 1.8, "sigma_z_N": 0.9, "tau_s": 1.2}, "files": {"city_counts": {"trees": 3}}, "variant": "A", "events": {},
             "end": "recorded", "flight_end_reason": "landed"}
    (td / "child.json").write_text(json.dumps(child))


def main():
    preview = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    # ---- small functions
    rng = np.random.default_rng(0)
    img = np.full((rw.PH, rw.PW, 3), 100, np.uint8)
    check("hf energy of a flat picture is 0", rf.hf_energy(img) == 0.0)
    check("noise raises the hf energy", rf.hf_energy(rf.add_noise(img, rng)) > 5.0)
    check("rain streaks raise the speck fraction", rf.speck_fraction(rf.add_rain_streaks(img, rng)) > rf.speck_fraction(img))
    sharp = np.zeros((rw.PH, rw.PW, 3), np.uint8)
    sharp[:, ::2] = 255
    check("temporal blur lowers the hf energy", rf.hf_energy(rf.temporal_blur([sharp, np.roll(sharp, 1, axis=1), np.roll(sharp, 2, axis=1)])) < rf.hf_energy(sharp))
    out_b, did = rf.bloom(np.where(np.arange(rw.PW)[None, :, None] > 300, 255, 40).astype(np.uint8).repeat(rw.PH, 0))
    check("bloom triggers on a saturated area and not on a dark picture", did and not rf.bloom(img)[1])
    col, valid = rf.colorize_depth(np.array([[1.0, 30.0, np.inf, 0.0]], np.float32))
    check("depth colours: near differs from far, invalid is black", tuple(col[0, 0]) != tuple(col[0, 1]) and tuple(col[0, 2]) == (0, 0, 0) and tuple(col[0, 3]) == (0, 0, 0) and abs(valid - 0.5) < 1e-6)
    lab = (np.arange(rw.PH * rw.PW) % 5).reshape(rw.PH, rw.PW).astype(np.int16)
    check("labels decode (int16)", int(rf.decode_labels(lab.astype("<i2").tobytes(), rw.PH, rw.PW).max()) == 4 and rf.decode_labels(lab.astype(np.uint8).tobytes(), rw.PH, rw.PW).shape == (rw.PH, rw.PW))
    for cases in ((True, True, True), (False, False, False)):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as td:
            td = Path(td)
            make_run(td, *cases)
            run = rf.Run(td)
            fn = rf.flight_numbers(run)
            check(f"{cases}: 7 of 7 gates, no wrong crossing", fn.get("gates_passed") == 7 and not fn.get("wrong_crossings"), f"{fn.get('gates_passed')} {fn.get('wrong_crossings')}")
            check(f"{cases}: no contact", sum(fn["contacts"].values()) == 0, f"{fn['contacts']}")
            check(f"{cases}: landing found, on the pad", fn["landing"].get("on_pad") is True and fn["landing"].get("touchdown_tp_s") is not None, f"{fn['landing']}")
            check(f"{cases}: lap time near the route time", fn.get("lap_times_s") and rc.T_ROUTE - 8.0 < fn["lap_times_s"][0] < rc.T_ROUTE, f"{fn.get('lap_times_s')} {rc.T_ROUTE}")
            check(f"{cases}: tracking per window present", {"phase1_city", "phase3_rain", "phase5_blur"} <= set(fn.get("tracking", {})), f"{list(fn.get('tracking', {}))}")
            ev = rf.measure_effects(run, fn)
            ph = rf.decide(run, fn, ev)
            rain_native, noise_native, night_native = ph[3]["tag"] == "NATIVE", ph[4]["tag"] == "NATIVE", ph[6]["tag"] == "NATIVE"
            check(f"{cases}: verdicts follow the measurements", (rain_native, noise_native, night_native) == cases, f"{(rain_native, noise_native, night_native)} {ev.get('rain_fpv')} {ev.get('noise')} {ev.get('night')}")
            check(f"{cases}: blur is always ours, six phases", ph[5]["tag"] == "ADDED BY OUR CODE" and sorted(ph) == [1, 2, 3, 4, 5, 6])
            # early frames: live files as the run writes them, and a night test shot made of the same pictures
            import shutil
            c0 = json.loads((td / "child.json").read_text())
            live = {k: c0[k] for k in ("arm_sim_s", "ground_reference_world", "panel_times", "panel_nbytes")}
            (td / "live.json").write_text(json.dumps(live))
            shutil.copy(td / "poses.npy", td / "poses_live_ready.npy")
            nt_t = {k: v[-12:] for k, v in c0["panel_times"].items() if k != "fpvn"}
            for nm in nt_t:
                nb = c0["panel_nbytes"][nm]
                data = (td / f"{nm}.raw").read_bytes()
                (td / f"nt_{nm}.raw").write_bytes(data[-12 * nb:])
            (td / "night_test.json").write_text(json.dumps({"panel_times": nt_t, "panel_nbytes": {k: c0["panel_nbytes"][k] for k in nt_t}}))
            er = rf.early_frames(td)
            ck = er["checks"]
            check(f"{cases}: early frames written, six plus the night test", (td / "early" / "early_contact.png").is_file() and ck.get("night_test_mean_grey") is not None, f"{ck}")
            check(f"{cases}: early checks see the picture and the gates in view are checked", "depth_is_a_picture" in ck and ck["depth_is_a_picture"] is True and ck["gates_in_view"] is False, f"{ck.get('depth_stripe_check')} {ck.get('depth')}")
            res = rf.post(td)
            check(f"{cases}: post wrote the result", (td / "film_result.json").is_file() and (td / "FR_contact.png").is_file() and (td / "summary.md").is_file(), f"{res.get('errors')}")
            r = json.loads((td / "film_result.json").read_text())
            check(f"{cases}: film_result has phases, race, landing, addability, moving objects", all(k in r for k in ("phases", "race", "landing", "addability", "moving_objects", "look")) and len(r["phases"]) == 6)
            check(f"{cases}: frames written for every picture", r["film_facts"]["frames"] > 300 and r["film_facts"]["size"] == [rf.W, rf.H], f"{r['film_facts']}")
            if preview and cases == (True, True, True):
                preview.mkdir(parents=True, exist_ok=True)
                (preview / "FR_contact_synthetic.png").write_bytes((td / "FR_contact.png").read_bytes())
                (preview / "overview_synthetic.png").write_bytes((td / "overview.png").read_bytes())
    print(f"{passed} of {passed + failed} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
