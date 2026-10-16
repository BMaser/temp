"""PX4 + Gazebo feature films. One job makes any subset of the films F01 to F11 (slice 1: F05 night and F06 clouds and rain).

  python3 tests/px4_gazebo_films_run.py all  --films F05,F06 --out results --px4 px4   # every condition, then the films
  python3 tests/px4_gazebo_films_run.py cond --name night      --out results --px4 px4   # ONE condition (own process: a clean MAVSDK server)
  python3 tests/px4_gazebo_films_run.py assemble --films F05,F06 --out results          # films, contact sheets, film_result.json, overview.png

A condition = PX4 SITL + Gazebo Harmonic (world baylands, model x500_mono_cam + chase camera [+ headlamp]) with the world text patched
(px4_gazebo/films_core.py), flown by MAVSDK offboard setpoints: arm, climb to 2 m, 2.5 s later fly 4 m east, 7.5 simulated seconds in all.
The drone camera and the chase camera are filmed at 15 pictures per simulated second and put side by side (1280 x 516, banner on top).
Build and start of the stack are the code of tests/px4_gazebo_base_run.py (the base lap), imported here.
Time is Gazebo simulation time everywhere; wall time only decides time-outs.
"""

import argparse
import asyncio
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "px4_gazebo"))
sys.path.insert(0, str(REPO / "tests"))
import films_core as fc  # noqa: E402

THIS = Path(__file__).resolve()
PX4_WORLD = "Tools/simulation/gz/worlds/baylands.sdf"
PX4_MODELS = "Tools/simulation/gz/models"
CONDITION_TIMEOUT_S = 25 * 60
HEALTHY_TIMEOUT_S = 600


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ------------------------------------------------------------------------------------------------ one condition
def backup_once(path: Path, backup_dir: Path) -> str:
    """Pristine text of a PX4 file: the first call saves a copy, later calls read the copy (so every condition starts from the original)."""
    backup_dir.mkdir(parents=True, exist_ok=True)
    b = backup_dir / (str(path).replace("/", "__").replace("\\", "__").replace(":", "_"))
    if not b.exists():
        b.write_text(path.read_text())
    return b.read_text()


def prepare_files(px4: Path, out_root: Path, cond: dict) -> dict:
    import px4_gazebo_base_run as base
    bk = out_root / "_backup"
    info = {}
    wp = px4 / PX4_WORLD
    pristine_world = backup_once(wp, bk)
    patched = fc.patch_world(pristine_world, cond)
    wp.write_text(patched)
    info["world_patched"] = patched != pristine_world
    info["world_bytes"] = len(patched)
    mono, x500 = px4 / PX4_MODELS / "mono_cam" / "model.sdf", px4 / PX4_MODELS / "x500_mono_cam" / "model.sdf"
    pm = backup_once(mono, bk)
    mono.write_text(fc.mono_cam_text(pm, base.WIDTH, base.HEIGHT, base.SENSOR_WIDTH_FROM, base.SENSOR_HEIGHT_FROM))
    px = backup_once(x500, bk)
    x500.write_text(fc.x500_model_text(px, base.CHASE_LINK, cond.get("headlamp", False)))
    info["mono_cam_changed"] = mono.read_text() != pm
    info["headlamp"] = bool(cond.get("headlamp"))
    return info


def grep_sim_log(path: Path, limit: int = 80) -> list:
    if not path.is_file():
        return []
    pat = re.compile(r"particle|fog|sky|shader|ogre|render|light|warn|error|fuel|download", re.I)
    hits = []
    for line in path.read_text(errors="replace").splitlines():
        if pat.search(line) and "vehicle_imu" not in line:
            hits.append(line[:300])
            if len(hits) >= limit:
                break
    return hits


async def fly_segment(S, child: dict, cond: dict, base):
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

    d = System()
    await d.connect(system_address="udpin://0.0.0.0:14540")

    async def connected():
        async for st in d.core.connection_state():
            if st.is_connected:
                return
    await asyncio.wait_for(connected(), 300)

    async def healthy():
        async for h in d.telemetry.health():
            if h.is_global_position_ok and h.is_home_position_ok:
                return h
    await asyncio.wait_for(healthy(), HEALTHY_TIMEOUT_S)
    log("healthy")
    # the cameras draw only while somebody listens: listen from here on (not recorded) so that particles fill the air before the film starts
    for name in S.films:
        child.setdefault("camera_subscribe", {})[name] = base.subscribe_camera(S, name, S.cam_topics[name])
    t_pre = S.now()
    while S.now() is None or S.now() < t_pre + cond["prefill_s"]:
        await asyncio.sleep(0.2)
        if time.monotonic() - wall_start > CONDITION_TIMEOUT_S:
            raise RuntimeError("time-out in the prefill")
    log(f"prefill of {cond['prefill_s']} simulated seconds done")

    with S.lock:
        S.t0 = S.sim_t
        S.poses = []
        S.recording = True
    for name in S.films:
        S.films[name].active = True
    wall0 = time.monotonic()
    child["record_start_sim_s"] = S.t0
    yaw = fc.YAW_EAST_NED_DEG
    await d.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, yaw))
    await d.action.arm()
    t_arm = S.now()
    child["arm_sim_s"] = t_arm
    try:
        await d.offboard.start()
    except OffboardError as e:
        child["offboard_error"] = str(e)
        raise
    log(f"armed at sim time {t_arm}, offboard started")
    end_reason = None
    while True:
        await asyncio.sleep(0.02)
        t = S.now()
        tt = t - t_arm
        if time.monotonic() - wall0 > CONDITION_TIMEOUT_S:
            end_reason = "time"
            break
        if tt >= fc.SEG_S:
            end_reason = "finished"
            break
        (n, e, dn), (vn, ve, vd) = fc.setpoint(tt)
        await d.offboard.set_position_velocity_ned(PositionNedYaw(n, e, dn, yaw), VelocityNedYaw(vn, ve, vd, yaw))
    child["flight_end_reason"] = end_reason
    child["record_wall_s"] = time.monotonic() - wall0
    with S.lock:
        S.recording = False
        child["record_end_sim_s"] = S.sim_t
    for name in S.films:
        S.films[name].active = False
        S.node.unsubscribe(S.cam_topics[name])


def run_condition(name: str, out_root: Path, px4: Path) -> dict:
    import px4_gazebo_base_run as base
    global wall_start
    wall_start = time.monotonic()
    cond = fc.CONDITIONS[name]
    out = out_root / "conds" / name
    out.mkdir(parents=True, exist_ok=True)
    child = {"condition": name, "label": cond["label"], "film": cond["film"], "prefill_s": cond["prefill_s"], "versions": {}}
    child["files"] = prepare_files(px4, out_root, cond)
    log(f"condition {name}: {json.dumps(child['files'])}")
    (out / "world_patched.sdf").write_text((px4 / PX4_WORLD).read_text())
    S = base.Shared(out)
    sh = base.sh
    sh("rm -f /tmp/.X99-lock", 5)
    procs = base.start_stack(out, px4)
    try:
        m, topics = base.wait_for(r"^/world/([^/]+)/clock$", base.STACK_START_TIMEOUT_S)
        (out / "gz_topics.txt").write_text(topics)
        if not m:
            raise RuntimeError("no Gazebo world clock topic: Gazebo did not start (see sim.log)")
        world = m.group(1)
        child["world"] = world
        cams = {}
        for cname, sensor in (("fpv", r"link/camera_link/sensor/imager/image"), ("chase", r"link/chase_cam_link/sensor/chase_cam/image")):
            mm, topics = base.wait_for(r"^(/world/%s/model/%s/%s)$" % (re.escape(world), re.escape(base.MODEL), sensor), 300)
            (out / "gz_topics.txt").write_text(topics)
            if not mm:
                raise RuntimeError(f"camera topic for {cname} did not appear")
            cams[cname] = mm.group(1)
        S.cam_topics = cams
        child["camera_topics"] = cams
        base.setup_gz(S, world)
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
        for cname in cams:
            S.films[cname] = base.FilmRec(out / f"{cname}.rgb")
        asyncio.run(fly_segment(S, child, cond, base))
        child["pose_messages"] = S.pose_n
        child["image_problems"] = S.image_problems
        child["frames_seen"] = {k: v.n_seen for k, v in S.films.items()}
        child["frames_kept"] = {k: len(v.times) for k, v in S.films.items()}
        child["times"] = {k: [round(t, 4) for t in v.times] for k, v in S.films.items()}
        for v in S.films.values():
            v.close()
        poses = np.array(S.poses, dtype=float)
        if len(poses) > 2:
            rel = poses[:, 1:4] - np.array(ground)
            child["path_rel_to_spawn"] = {"max_height_m": float(rel[:, 2].max()), "final": [float(v) for v in rel[-1]],
                                          "max_east_m": float(rel[:, 0].max()), "n_rows": int(len(poses))}
        simdur = (child.get("record_end_sim_s") or 0) - (child.get("record_start_sim_s") or 0)
        w = child.get("record_wall_s")
        child["recorded_sim_s"] = simdur
        child["real_time_factor"] = float(simdur / w) if w else None
        child["pictures_per_wall_second"] = {k: (len(v) / w if w else None) for k, v in child["times"].items()}
        child["end"] = "recorded"
    except Exception:  # noqa: BLE001
        child["traceback"] = traceback.format_exc()
        child.setdefault("end", "error")
        log(child["traceback"])
        for v in S.films.values():
            try:
                v.close()
            except Exception:  # noqa: BLE001
                pass
    finally:
        try:
            child["sim_log_hits"] = grep_sim_log(out / "sim.log")
        except Exception:  # noqa: BLE001
            pass
        (out / "child.json").write_text(json.dumps(child, indent=2, default=str))
        base.stop_stack(procs)
        time.sleep(5)
    return child


# ------------------------------------------------------------------------------------------------ films
def gray_mean(a: np.ndarray) -> float:
    return float(a.astype(np.float32).mean())


def load_segment(out_root: Path, cname: str):
    """(fpv, chase, info) for a recorded condition: memory-mapped pictures cut to equal counts, or (None, None, info) when missing."""
    d = out_root / "conds" / cname
    info = {"condition": cname, "label": fc.CONDITIONS[cname]["label"]}
    cj = d / "child.json"
    if not cj.is_file():
        info["problem"] = "no child.json"
        return None, None, info
    child = json.loads(cj.read_text())
    info["child"] = {k: child.get(k) for k in ("end", "flight_end_reason", "real_time_factor", "recorded_sim_s", "frames_seen", "frames_kept",
                                              "pictures_per_wall_second", "path_rel_to_spawn", "image_problems", "traceback", "prefill_s", "files")}
    w, h = fc.PANEL
    arr = {}
    for k in ("fpv", "chase"):
        p = d / f"{k}.rgb"
        n = p.stat().st_size // (w * h * 3) if p.is_file() else 0
        arr[k] = np.memmap(p, dtype=np.uint8, mode="r", shape=(n, h, w, 3)) if n else None
    if arr["fpv"] is None or arr["chase"] is None:
        info["problem"] = "no pictures"
        return None, None, info
    n = min(len(arr["fpv"]), len(arr["chase"]))
    tf, tc = child["times"]["fpv"][:n], child["times"]["chase"][:n]
    info["pictures"] = n
    info["max_time_difference_fpv_chase_s"] = float(max(abs(a - b) for a, b in zip(tf, tc))) if n else None
    info["sim_time_span_s"] = float(tf[-1] - tf[0]) if n > 1 else 0.0
    idx = np.linspace(0, n - 1, min(n, 12)).astype(int)
    info["mean_gray_fpv"] = [round(gray_mean(arr["fpv"][i]), 1) for i in idx]
    info["mean_gray_chase"] = [round(gray_mean(arr["chase"][i]), 1) for i in idx]
    info["mean_gray_fpv_all"] = round(float(np.mean(info["mean_gray_fpv"])), 1)
    info["mean_gray_chase_all"] = round(float(np.mean(info["mean_gray_chase"])), 1)
    # contrast: mean absolute horizontal gradient of the fpv picture (fog and rain lower or add to it)
    g = [np.abs(np.diff(arr["fpv"][i].astype(np.float32).mean(axis=2), axis=1)).mean() for i in idx]
    info["mean_abs_gradient_fpv"] = round(float(np.mean(g)), 2)
    info["times_fpv"] = tf
    return arr["fpv"][:n], arr["chase"][:n], info


BANNER = 36


def compose(fpv: np.ndarray, chase: np.ndarray, text: str, tsim: float, fid: str, font_big, font_small):
    from PIL import Image, ImageDraw
    w, h = fc.PANEL
    canvas = np.zeros((h + BANNER, 2 * w, 3), dtype=np.uint8)
    canvas[BANNER:, :w] = fpv
    canvas[BANNER:, w:] = chase
    img = Image.fromarray(canvas)
    d = ImageDraw.Draw(img)
    d.text((8, 6), f"{fid}  {text}", fill=(255, 255, 255), font=font_big)
    tt = f"sim {tsim:5.1f} s"
    d.text((2 * w - 8 - d.textlength(tt, font=font_big), 6), tt, fill=(255, 220, 120), font=font_big)
    for x, cap in ((8, "drone camera (nose)"), (w + 8, "chase view (1.5 m behind, 0.5 m up)")):
        box = d.textbbox((x, h + BANNER - 28), cap, font=font_small)
        d.rectangle([box[0] - 3, box[1] - 2, box[2] + 3, box[3] + 2], fill=(0, 0, 0))
        d.text((x, h + BANNER - 28), cap, fill=(255, 255, 255), font=font_small)
    d.line([(w, BANNER), (w, h + BANNER)], fill=(255, 255, 255), width=1)
    return np.asarray(img)


def encode_pipe(mp4: Path, w: int, h: int):
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}", "-r", str(fc.FILM_FPS), "-i", "-",
           "-c:v", "libx264", "-preset", "veryfast", "-crf", "22", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(mp4)]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE)


def find_line(path: Path, needle: str):
    try:
        for i, line in enumerate(path.read_text().splitlines(), 1):
            if needle in line:
                return f"{path.relative_to(REPO).as_posix()}:{i}"
    except Exception:  # noqa: BLE001
        pass
    return None


def build_film(out_root: Path, fid: str, cnames: list, mp4: Path, contact_png: Path, label_prefix: str = ""):
    """Make the film of a list of conditions (their segments one after the other) and its contact sheet (8 pictures). Returns a result dict."""
    import sync_film
    font_big, font_small = sync_film.font(20), sync_film.font(15)
    res = {"segments": [], "problems": []}
    segs = []
    for c in cnames:
        fpv, chase, info = load_segment(out_root, c)
        res["segments"].append(info)
        if fpv is None:
            res["problems"].append(f"{c}: {info.get('problem')}")
        else:
            segs.append((c, fpv, chase, info))
    total = sum(len(s[1]) for s in segs)
    if not total:
        return res
    wanted = sorted(set(int(round(k * (total - 1) / 7)) for k in range(8)))
    keep, labels, k0 = {}, {}, 0
    W, H = 2 * fc.PANEL[0], fc.PANEL[1] + BANNER
    proc = encode_pipe(mp4, W, H)
    t_first = None
    for c, fpv, chase, info in segs:
        short = fc.CONDITIONS[c]["label"].split(":")[0]
        tf = info["times_fpv"]
        for i in range(len(fpv)):
            fr = compose(np.asarray(fpv[i]), np.asarray(chase[i]), short, tf[i] - tf[0], fid, font_big, font_small)
            proc.stdin.write(fr.tobytes())
            if k0 + i in wanted:
                keep[k0 + i] = fr
                labels[k0 + i] = f"{fid} {short} {tf[i] - tf[0]:.1f}s"
        k0 += len(fpv)
        info.pop("times_fpv", None)
    proc.stdin.close()
    rc = proc.wait(timeout=300)
    res["ffmpeg_exit"] = rc
    res["pictures"] = total
    res["seconds"] = round(total / fc.FILM_FPS, 2)
    res["size"] = [W, H]
    order = sorted(keep)
    sheet = sync_film.contact_sheet([keep[i] for i in order], [labels[i] for i in order], cols=2, max_width=1920)
    sheet.save(contact_png)
    res["contact_indices"] = order
    res["contact_size"] = list(sheet.size)
    res["_keep"] = {i: keep[i] for i in order}
    return res


def make_overview(out_root: Path, rows: list):
    """overview.png: one row per film, 4 pictures, film id and verdict written on the row."""
    from PIL import Image, ImageDraw
    import sync_film
    if not rows:
        return
    tile_w = 480
    f = sync_film.font(22)
    images = []
    for fid, verdict, frames in rows:
        pics = [Image.fromarray(fr).resize((tile_w, int(round(fr.shape[0] * tile_w / fr.shape[1]))), Image.BILINEAR) for fr in frames]
        th = pics[0].size[1]
        row = Image.new("RGB", (4 * tile_w, th + 34), (25, 25, 25))
        d = ImageDraw.Draw(row)
        d.text((8, 4), f"{fid}  verdict: {verdict}", fill=(255, 255, 255), font=f)
        for k, p in enumerate(pics):
            row.paste(p, (k * tile_w, 34))
        images.append(row)
    sheet = Image.new("RGB", (4 * tile_w, sum(r.size[1] for r in images)), (25, 25, 25))
    y = 0
    for r in images:
        sheet.paste(r, (0, y))
        y += r.size[1]
    sheet.save(out_root / "overview.png")


def settings_used(fid: str) -> list:
    core = REPO / "px4_gazebo" / "films_core.py"
    run = THIS
    items = []

    def add(what, path, needle):
        items.append({"what": what, "where": find_line(path, needle)})
    add("scene block (ambient, background, grid, sky element, shadows, fog element)", core, "def scene_xml")
    add("directional light sunUTC: direction, colours, intensity", core, "def sun_xml")
    add("world patch: replaces the scene block and the sunUTC light, appends the extras before </world>", core, "def patch_world")
    add("conditions (the settings of every segment)", core, "CONDITIONS = {")
    add("flight program (setpoints)", core, "def setpoint")
    add("MAVSDK offboard flight and recording", run, "async def fly_segment")
    if fid == "F05":
        add("point light on a post (street lamp)", core, "def lamp_xml")
        add("spot light on the drone (headlamp)", core, "def headlamp_link")
    if fid == "F06":
        add("particle emitter model (rain, snow)", core, "def emitter_xml")
        add("rain parameters", core, "def rain_xml")
        add("snow parameters", core, "def snow_xml")
        add("Fuel model Fog Generator include", core, "def fog_xml")
        add("particle emitter system plugin", core, "PARTICLE_PLUGIN =")
    return items


def assemble(out_root: Path, film_ids: list) -> dict:
    results, rows, checks = {}, [], []
    for fid in film_ids:
        spec = fc.FILMS[fid]
        d = out_root / f"{fid}_{spec['short']}"
        d.mkdir(parents=True, exist_ok=True)
        mp4 = d / f"{fid}_{spec['short']}.mp4"
        contact = d / f"{fid}_contact.png"
        r = build_film(out_root, fid, spec["conditions"], mp4, contact)
        # probes: a film of their own each, and a still
        probes = {}
        for p in spec.get("probes", []):
            pm = d / f"probe_{p.replace('probe_', '')}.mp4"
            pr = build_film(out_root, f"{fid}-probe", [p], pm, d / f"probe_{p.replace('probe_', '')}_contact.png")
            pr.pop("_keep", None)
            probes[p] = pr
        keep = r.pop("_keep", {})
        verdict = fc.VERDICTS.get(fid, {"verdict": "pending_review", "why": ""})
        if r.get("problems") and not keep:
            verdict = {"verdict": "not_shown", "why": "no segment could be recorded: " + "; ".join(r["problems"])}
        out = {
            "id": fid, "feature": spec["feature"], "simulator": "px4_gazebo", "verdict": verdict["verdict"], "verdict_reason": verdict["why"],
            "film": mp4.name if mp4.is_file() else None, "contact_sheet": contact.name if contact.is_file() else None,
            "document_claim": {i: fc.DOC_CLAIMS[i] for i in spec["doc_ids"]},
            "settings_and_code": settings_used(fid),
            "measured": r, "probes": probes,
            "versions": {k: (out_root / f).read_text().strip() for k, f in (("px4", "px4_version.txt"), ("gazebo", "gz_version.txt")) if (out_root / f).is_file()},
        }
        (d / "film_result.json").write_text(json.dumps(out, indent=2, default=str))
        results[fid] = out
        if keep:
            order = sorted(keep)
            pick = [order[0], order[2], order[4], order[-1]] if len(order) >= 5 else order[:4]
            rows.append((fid, verdict["verdict"], [keep[i] for i in pick]))
        n = r.get("pictures", 0)
        checks.append({"film": fid, "mp4": mp4.name, "pictures": n, "seconds": r.get("seconds"), "wide_enough": r.get("size", [0])[0] >= 640,
                       "ok": bool(n >= 150 and r.get("ffmpeg_exit") == 0 and not r.get("problems"))})
    make_overview(out_root, rows)
    (out_root / "films_check.json").write_text(json.dumps(checks, indent=2))
    # raw pictures are not results
    for p in (out_root / "conds").glob("*/*.rgb"):
        p.unlink(missing_ok=True)
    return {"results": results, "checks": checks}


def write_summary(out_root: Path, res: dict):
    L = ["### PX4 + Gazebo feature films", ""]
    for fid, r in res["results"].items():
        m = r["measured"]
        L.append(f"**{fid} {r['feature']}**: verdict {r['verdict']} ({r['verdict_reason']}); film {r['film']}, {m.get('pictures')} pictures, {m.get('seconds')} s; problems {m.get('problems')}")
        for s in m.get("segments", []):
            c = s.get("child") or {}
            L.append(f"- {s['condition']}: end {c.get('end')}, flight {c.get('flight_end_reason')}, real-time factor {c.get('real_time_factor')}, "
                     f"pictures per wall second {c.get('pictures_per_wall_second')}, mean gray fpv {s.get('mean_gray_fpv_all')}, chase {s.get('mean_gray_chase_all')}, "
                     f"gradient {s.get('mean_abs_gradient_fpv')}, fpv/chase time difference {s.get('max_time_difference_fpv_chase_s')}, path {c.get('path_rel_to_spawn')}, "
                     f"traceback {str(c.get('traceback'))[-300:] if c.get('traceback') else None}")
        for p, pr in r["probes"].items():
            segs = pr.get("segments", [])
            L.append(f"- probe {p}: pictures {pr.get('pictures')}, mean gray fpv {[s.get('mean_gray_fpv_all') for s in segs]}, problems {pr.get('problems')}")
    L.append("")
    L.append("| film | mp4 | pictures | seconds | check |")
    L.append("|---|---|---|---|---|")
    for c in res["checks"]:
        L.append(f"| {c['film']} | {c['mp4']} | {c['pictures']} | {c['seconds']} | {'ok' if c['ok'] else 'FAIL'} |")
    (out_root / "summary.md").write_text("\n".join(L) + "\n")


# ------------------------------------------------------------------------------------------------ main
def needed_conditions(film_ids):
    names = []
    for fid in film_ids:
        names += fc.FILMS[fid]["conditions"] + fc.FILMS[fid].get("probes", [])
    return names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("command", choices=["all", "cond", "assemble"])
    ap.add_argument("--films", default="F05,F06")
    ap.add_argument("--name", default=None)
    ap.add_argument("--out", required=True)
    ap.add_argument("--px4", default="px4")
    a = ap.parse_args()
    out_root = Path(a.out).resolve()
    out_root.mkdir(parents=True, exist_ok=True)
    film_ids = [f for f in a.films.split(",") if f]
    if a.command == "cond":
        run_condition(a.name, out_root, Path(a.px4).resolve())
        return
    if a.command == "all":
        for name in needed_conditions(film_ids):
            d = out_root / "conds" / name
            d.mkdir(parents=True, exist_ok=True)
            log(f"=== condition {name}")
            t0 = time.time()
            try:
                with open(d / "run.log", "w") as lf:
                    subprocess.run([sys.executable, str(THIS), "cond", "--name", name, "--out", str(out_root), "--px4", a.px4],
                                   stdout=lf, stderr=subprocess.STDOUT, timeout=CONDITION_TIMEOUT_S + 600)
            except subprocess.TimeoutExpired:
                log(f"condition {name} timed out")
                import px4_gazebo_base_run as base
                base.stop_stack({})
            log(f"condition {name} done in {time.time() - t0:.0f} s; tail of its log:")
            print("\n".join((d / "run.log").read_text(errors="replace").splitlines()[-6:]), flush=True)
    res = assemble(out_root, film_ids)
    write_summary(out_root, res)
    shutil.rmtree(out_root / "_backup", ignore_errors=True)
    print((out_root / "summary.md").read_text())


if __name__ == "__main__":
    main()
