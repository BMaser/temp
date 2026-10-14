"""Drone GT: draw the new drone model "GT" on the simulated drone and prove that only the drawing changed.

The model is assets/drone_gt/drone.obj (meters, front = +x, up = +z). tools/obj_parts.py splits it into one mesh per material;
tools/drone_look.py adds those meshes to the drone body of LSY's environment before the model is compiled (no collision, no mass)
and hides the present drone's drawn shapes. The track is level 0 of lsy_drone_racing with the new square gate model and the floor
pads, as in tests/crazyflow_run_a.py, and so is the flight: controllers/nose_forward_controller.py, NOSE_MODE = "forward",
YAW_RATE_MAX = 4.0 (the setting that run A chose). The seed is 0.

Sub-commands, each writes into --out:
  stills   Close-up pictures of the GT drone on the start pad from eight sides (front, front_quarter, left, back_quarter, back, right,
           top, bottom) for each of the four looks (colors): drone_gt_<look>.png; drone_gt_looks.png (front quarter and back quarter of
           all four looks); fpv_present.png, fpv_gt.png and fpv_compare.png (the drone's own camera right after the reset, cyan look);
           drone_gt_scales.png (cyan look at drawing scale 1.0, 1.5, 2.0). Also the model checks (mass, inertia, center of mass,
           collision shapes, contact masks, no collision bits on the added shapes) and the picture checks. Writes stills.json.
           The bottom view draws the drone lifted by 0.8 m (only in the picture: the pose in the renderer's copy of the mocap data
           is changed for that one picture, as in tests/crazyflow_track_look.py), because the camera would be below the floor.
  proof    The chosen flight three times without pictures: with the present drone drawing, with the GT drone at scale 1.0 and at
           scale 2.0; position, quaternion and velocity compared step by step. Writes proof.json.
  film     The same flight with the GT drone (cyan look) filmed by the following camera of run A, at scale 1.0 and 2.0:
           chase_gt.mp4 and chase_gt_scale_2p0.mp4, each with a contact sheet of 12 pictures. Writes film.json.
  summary  summary.md and result.json from the files above. Exit code 1 if any check failed.
  check    applies the pass rule to result.json (standard library only), exit code 1 on failure.
Internal: child (one environment in its own process; the parent starts it).

Every environment starts from a fresh process with seed 0 and exactly one env.reset(), as in run A.
"""

import argparse
import hashlib
import json
import math
import os
import platform
import sys
import tempfile
import time
import traceback
from pathlib import Path

try:
    import numpy as np
except ImportError:  # the check sub-command needs only the standard library
    np = None

HERE = Path(__file__).resolve().parent
REPO_DIR = HERE.parent
TOOLS_DIR = REPO_DIR / "tools"
CONTROLLERS_DIR = REPO_DIR / "controllers"
PADS_DIR = REPO_DIR / "assets" / "pads"
ASSET_DIR = REPO_DIR / "assets" / "drone_gt"
for _p in (HERE, TOOLS_DIR, CONTROLLERS_DIR):
    sys.path.insert(0, str(_p))

import crazyflow_run_a as ra  # noqa: E402  (stdlib only at import: Runner, compare_states, ChaseCamera, add_check ...)
import crazyflow_track_look as tl  # noqa: E402  (stdlib only at import: orange, check_assets)

WIDTH, HEIGHT = ra.WIDTH, ra.HEIGHT
FPS, STEP_PER_PICTURE, SEED = ra.FPS, ra.STEP_PER_PICTURE, ra.SEED
NOSE_MODE, YAW_RATE_MAX = "forward", 4.0  # run A's choice (sweep.json, chosen.json)
LOOKS = ("magenta", "cyan", "red", "violet")
SCALES = (1.0, 1.5, 2.0)
VIEWS = ("front", "front_quarter", "left", "back_quarter", "back", "right", "top", "bottom")
# view -> (azimuth offset from the heading of the drone in degrees, elevation in degrees, camera distance in m).
# Free camera of MuJoCo 3.10.0 (engine_vis_visualize.c lines 452-473): forward = (cos el cos az, cos el sin az, sin el), the camera sits at
# lookat - distance * forward, up = (-sin el cos az, -sin el sin az, cos el). With heading psi of the nose:
#   front: camera ahead of the nose looking back (az = psi + 180); back: az = psi; left: camera on the drone's left (+y), az = psi - 90;
#   right: az = psi + 90; front quarter: camera ahead and to the left, az = psi + 225; back quarter: behind and to the left, az = psi - 45;
#   top: straight down with the nose up in the picture (az = psi, el = -90); bottom: straight up with the nose up (az = psi + 180, el = +90).
VIEW_SPEC = {
    "front": (180.0, -25.0, 0.32), "front_quarter": (225.0, -25.0, 0.32), "left": (-90.0, -25.0, 0.32), "back_quarter": (-45.0, -25.0, 0.32),
    "back": (0.0, -25.0, 0.32), "right": (90.0, -25.0, 0.32), "top": (0.0, -90.0, 0.32), "bottom": (180.0, 90.0, 0.32),
    "front_quarter_far": (225.0, -25.0, 0.5),
}
BOTTOM_LIFT_M = 0.8
DIFF_THRESHOLD = 30  # a pixel differs when some color channel differs by more than this (0..255)
MIN_GT_VS_PRESENT = 0.02  # share of all pixels
MIN_LOOK_DIFF = 0.01  # share of all pixels, two looks with different body colors
MIN_FRONT_BACK = 0.05  # share of the drone's pixels (union of the drone masks of the two views)
N_ADDED = 10  # parts of the model (materials)


def scale_tag(s: float) -> str:
    return ra.scale_tag(s)


def still_file(tag: str, view: str) -> str:
    return f"{tag}_{view}.png"


def child_tag(look: str, scale: float) -> str:
    return f"{look}_s{scale_tag(scale)}"


# ---------------------------------------------------------------------------------------------
# Measures on pictures and on fingerprints (parent side)
# ---------------------------------------------------------------------------------------------


def diff_share(a, b, mask=None, threshold: int = DIFF_THRESHOLD) -> float:
    """Share of pixels (of the mask, or of all) in which some channel differs by more than the threshold."""
    d = np.abs(a.astype(np.int32) - b.astype(np.int32)).max(axis=2) > threshold
    if mask is not None:
        n = int(mask.sum())
        return float(d[mask].sum() / n) if n else 0.0
    return float(d.mean())


def load_png(path: Path):
    from PIL import Image

    return np.asarray(Image.open(path).convert("RGB"))


def fingerprint_checks(prefix: str, base: dict, others: dict, checks: list) -> None:
    """Add the exact comparisons of the compiled models: `others` = {name: child entry}; `base` = the child entry of the unchanged drone."""
    import drone_look

    fb = base.get("fingerprint")
    if not fb:
        for n in ("bodies_mass_inertia_com", "collision_shapes", "drone_collision_shapes", "camera_and_step"):
            ra.add_check(checks, f"{prefix}_{n}", False, "the reference child (present drone) wrote no model fingerprint")
        ra.add_check(checks, f"{prefix}_counts_and_contact_masks", False, "the reference child wrote no model fingerprint")
        return
    results = {}
    for name, e in others.items():
        fo = e.get("fingerprint")
        results[name] = drone_look.compare_fingerprints(fb, fo) if fo else None
    for key in ("bodies_mass_inertia_com", "collision_shapes", "drone_collision_shapes", "camera_and_step"):
        ok = bool(results) and all(r is not None and r[key][0] for r in results.values())
        bad = [n for n, r in results.items() if r is None or not r[key][0]]
        first = next((r[key][1] for r in results.values() if r), "no child")
        ra.add_check(checks, f"{prefix}_{key}", ok, f"present drone against {sorted(results)} (exact equality): {first}; differing: {bad or 'none'}")
    # counts: the only change is more geoms, meshes and materials
    ca = fb["counts"]
    bad = []
    for name, e in others.items():
        co = (e.get("fingerprint") or {}).get("counts")
        if not co:
            bad.append(f"{name}: no counts")
            continue
        want = {**ca, "ngeom": ca["ngeom"] + N_ADDED, "nmesh": ca["nmesh"] + N_ADDED, "nmat": ca["nmat"] + N_ADDED}
        if co != want:
            bad.append(f"{name}: {co} (wanted {want})")
    mb, mo = base.get("contact_masks") or {}, {n: e.get("contact_masks") or {} for n, e in others.items()}
    mask_bad = [n for n, v in mo.items() if not v or v.get("sha256") != mb.get("sha256") or v.get("shape") != mb.get("shape")]
    ra.add_check(checks, f"{prefix}_counts_and_contact_masks", not bad and not mask_bad and bool(mb),
                 f"counts: nbody, njnt, nmocap, ncam, nsite equal and ngeom, nmesh, nmat each + {N_ADDED}: problems {bad or 'none'}; contact masks "
                 f"(race_core.py _load_contact_masks, shape {mb.get('shape')}, {mb.get('true')} slots true) identical to the present drone's in "
                 f"{len(mo) - len(mask_bad)} of {len(mo)} children: differing {mask_bad or 'none'}")


# ---------------------------------------------------------------------------------------------
# Child: one environment in its own process; pictures and/or one flight
# ---------------------------------------------------------------------------------------------


def run_child(args) -> None:
    out = Path(args.out).resolve()
    sub = out / args.subdir
    sub.mkdir(parents=True, exist_ok=True)
    tag = args.tag
    views = [v for v in args.views.split(",") if v]
    result = {
        "tag": tag, "task": args.task, "look": args.look, "drone_scale": args.drone_scale, "camera": args.camera, "views": views,
        "MUJOCO_GL": os.environ.get("MUJOCO_GL"), "versions": {}, "traceback": None, "flown": None,
    }
    env = None
    writer = None
    try:
        os.environ["SCIPY_ARRAY_API"] = "1"  # crazyflow's own camera examples set this before importing scipy
        import importlib.metadata as md

        import gymnasium
        import jax
        import jax.numpy as jnp
        import mujoco
        from gymnasium.wrappers.jax_to_numpy import JaxToNumpy
        from lsy_drone_racing.utils import load_config, load_controller

        import drone_look
        import nose_yaw as ny
        import sync_film
        import track_env

        repo, assets = Path(args.repo).resolve(), Path(args.assets).resolve()
        result["versions"] = {
            "mujoco": mujoco.__version__, "jax": jax.__version__, "crazyflow": md.version("crazyflow"),
            "lsy_drone_racing": md.version("lsy_drone_racing"), "lsy_drone_racing_commit": args.lsy_commit,
        }
        result["asset_check"] = tl.check_assets(repo, assets)  # the gate model LSY loads must be the new one

        cfg = load_config(repo / "config" / "level0.toml")
        cfg.sim.render = False
        freq = int(cfg.env.freq)
        if cfg.env.id != track_env.BASE_ENV_ID:
            raise AssertionError(f"level0.toml names the environment {cfg.env.id!r}, expected {track_env.BASE_ENV_ID!r}")
        controller_cls = load_controller(CONTROLLERS_DIR / "nose_forward_controller.py")
        controller_cls.NOSE_MODE = args.nose_mode
        controller_cls.YAW_RATE_MAX = float(args.yaw_rate)
        import nose_forward_controller as nfc  # same file, imported by name only for its list of points

        import tomllib

        with open(repo / "config" / "level0.toml", "rb") as f:
            level = tomllib.load(f)
        start_xy = [float(v) for v in level["env"]["track"]["drones"][0]["pos"][:2]]
        end_xy = [float(v) for v in nfc.POINTS_AFTER_START[-1][:2]]
        pads = {"pad_start": tuple(start_xy), "pad_end": tuple(end_xy)}

        extra, look = {}, None
        if args.look == "present":
            env_id = track_env.register_pad_env()
        else:
            look = drone_look.make_look(args.look, args.drone_scale, out / "gt_meshes", asset_dir=ASSET_DIR)
            env_id = drone_look.register_gt_env()
            extra = {"gt": look}
        result["env_id"] = env_id
        env = gymnasium.make(
            env_id,
            freq=cfg.env.freq, sim_config=cfg.sim, sensor_range=cfg.env.sensor_range, control_mode=cfg.env.control_mode,
            track=cfg.env.track, disturbances=cfg.env.get("disturbances"), randomizations=cfg.env.get("randomizations"),
            seed=SEED, pads=pads, pads_dir=str(PADS_DIR), **extra,
        )
        env = JaxToNumpy(env)
        core = env.unwrapped
        obs0, info = env.reset()  # the one and only reset of this environment
        m = core.sim.mj_model
        pos0 = [float(v) for v in obs0["pos"]]
        result["start_position"] = pos0
        result["start_pad_error_m"] = float(max(abs(pos0[0] - start_xy[0]), abs(pos0[1] - start_xy[1])))
        if result["start_pad_error_m"] > tl.START_TOL:
            raise AssertionError(f"start pad is {result['start_pad_error_m']} m away from the start position of the simulator")

        # What the physics and the collision use, read from the compiled model BEFORE any visual edit of this process.
        result["fingerprint"] = drone_look.model_fingerprint(mujoco, m)
        masks = np.asarray(core.data.contact_masks)
        result["contact_masks"] = {"shape": list(masks.shape), "true": int(masks.sum()), "sha256": hashlib.sha256(np.ascontiguousarray(masks).tobytes()).hexdigest()}
        if look is not None:
            result["gt_info"] = {k: v for k, v in (core.gt_info or {}).items() if k != "added"}
            result["gt_info"]["added_parts"] = [a["part"] for a in (core.gt_info or {}).get("added", [])]
            result["read_back"] = drone_look.read_back(mujoco, np, m, look)

        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, drone_look.DRONE_BODY)

        def sync() -> None:
            if not core.data.sim_data.core.mjx_synced:  # same sync as RaceCoreEnv.render and run A
                core.data, core.sim.mjx_data = core._render_sync(core.data, core.sim.mjx_data)

        def render(**kw):
            sync()
            img = np.ascontiguousarray(core.sim.render(mode="rgb_array", width=WIDTH, height=HEIGHT, **kw))
            if img.shape != (HEIGHT, WIDTH, 3) or img.dtype != np.uint8:
                raise ValueError(f"unexpected picture {img.shape} {img.dtype}")
            return img

        # ---- pictures ----
        if args.task == "stills":
            stills_dir = out / "stills_work"
            stills_dir.mkdir(exist_ok=True)
            if args.look == "present":  # the present drone in its hi-vis orange, as in the earlier looks
                orange_info = tl.make_drone_orange(mujoco, np, m)
                ok, detail = tl.orange_readback(mujoco, np, m, orange_info)
                result["orange"] = {"ok": ok, "detail": detail}
            from PIL import Image

            if args.fpv:  # the drone's own camera right after the reset; the viewer keeps this camera, so close it afterwards
                img = render(camera="fpv_cam:0")
                Image.fromarray(img).save(out / f"fpv_{'present' if args.look == 'present' else 'gt'}.png")
                result["fpv"] = {"file": f"fpv_{'present' if args.look == 'present' else 'gt'}.png", "mean": float(img.mean())}
                core.sim.close()

            yaw0 = float(ny.yaw_from_quat_xyzw(np.asarray(obs0["quat"], dtype=float)))
            mocap_id = int(m.body_mocapid[bid])
            if mocap_id < 0:
                raise ValueError("drone:0 is not a mocap body (sim.py sync_sim2mjx places the drone through mocap_pos)")
            drone_geoms = [g for b in drone_look.drone_body_ids(mujoco, m) for g in drone_look.geoms_of(m, b) if int(m.geom_group[g]) <= 2]
            result["drone_drawn_geoms"] = len(drone_geoms)
            seen = []

            def shot(view: str):
                sync()  # before the pose is swapped below, so the sync inside render() has nothing left to do
                az0, el, dist = VIEW_SPEC[view]
                lift = BOTTOM_LIFT_M if view == "bottom" else 0.0
                cfg_cam = {"lookat": np.asarray([pos0[0], pos0[1], pos0[2] + lift]), "distance": dist, "azimuth": math.degrees(yaw0) + az0, "elevation": el}
                if core.sim.viewer is not None and core.sim.viewer.viewer is not None:  # cam_config applies only when the viewer is made
                    cam = core.sim.viewer.viewer.cam
                    cam.lookat[:] = cfg_cam["lookat"]
                    cam.distance, cam.azimuth, cam.elevation = float(dist), float(cfg_cam["azimuth"]), float(el)
                saved = core.sim.mjx_data
                if lift:
                    p = saved.mocap_pos
                    core.sim.mjx_data = saved.replace(mocap_pos=p.at[:, mocap_id, 2].set(p[:, mocap_id, 2] + lift))
                try:
                    return render(camera=-1, cam_config=cfg_cam)
                finally:
                    core.sim.mjx_data = saved

            for view in views:
                a = shot(view)
                # the same picture without the drone: every drawn geom of the drone gets alpha 0 for this one render, then comes back
                before = np.array(m.geom_rgba[drone_geoms], copy=True)
                m.geom_rgba[drone_geoms, 3] = 0.0
                try:
                    b = shot(view)
                finally:
                    m.geom_rgba[drone_geoms] = before
                mask = np.abs(a.astype(np.int32) - b.astype(np.int32)).max(axis=2) > 0
                Image.fromarray(a).save(out / "stills_work" / still_file(tag, view))
                np.savez_compressed(out / "stills_work" / f"{tag}_{view}_mask.npz", mask=mask)
                orange = tl.share_masks(np, a)["orange"] & mask
                seen.append({"view": view, "file": still_file(tag, view), "drone_pixels": int(mask.sum()), "orange_pixels_on_drone": int(orange.sum()),
                             "mean": float(a.mean())})
            result["stills"] = seen
            result["flown"] = False

        # ---- flight ----
        if args.task == "flight":
            controller = controller_cls(obs0, info, cfg)
            total = int(obs0["gate_sequence"].shape[0])
            rec = {k: [] for k in ("time", "pos", "vel", "quat", "ang_vel", "cmd_yaw", "passed")}

            def record(obs, cmd_yaw):
                rec["time"].append(len(rec["pos"]) / freq)
                rec["pos"].append([float(v) for v in obs["pos"]])
                rec["vel"].append([float(v) for v in obs["vel"]])
                rec["quat"].append([float(v) for v in obs["quat"]])
                rec["ang_vel"].append([float(v) for v in obs["ang_vel"]])
                rec["cmd_yaw"].append(float(cmd_yaw))
                rec["passed"].append(int(obs["n_gates_passed"]))

            pic_step, picture_seconds = [], []
            chase = ra.ChaseCamera(core, obs0, freq) if args.camera == "chase" else None
            if chase is not None:
                film_name = args.film or f"{tag}.mp4"
                writer = sync_film.FilmWriter(out / film_name, WIDTH, HEIGHT, FPS, sub / f"{tag}_ffmpeg.log")
                result["film"] = film_name

            def picture() -> None:
                row = len(rec["pos"]) - 1
                t0 = time.perf_counter()
                img = render(**chase.kwargs(rec["pos"][row]))
                picture_seconds.append(time.perf_counter() - t0)
                writer.write(img)
                pic_step.append(row)

            t_start = time.perf_counter()
            obs = obs0
            record(obs, controller.planned_yaw(0))
            if writer:
                picture()
            steps, end, dropped = 0, None, 0
            # The loop of run A (tests/crazyflow_run_a.py lines 380-398): a step that ends the episode with terminated=True is not
            # recorded, and the loop stops at the step of the last passage.
            while True:
                cmd_yaw = controller.planned_yaw(steps)
                action = controller.compute_control(obs, info)
                obs, reward, terminated, truncated, info = env.step(action)
                done = controller.step_callback(action, obs, reward, terminated, truncated, info)
                steps += 1
                if terminated:
                    dropped += 1
                    end = "crash"
                    break
                record(obs, cmd_yaw)
                if writer and steps % STEP_PER_PICTURE == 0:
                    picture()
                if int(obs["n_gates_passed"]) >= total:
                    end = "finished"
                    break
                if truncated or done:
                    end = "time"
                    break
            if writer and pic_step[-1] != len(rec["pos"]) - 1:
                picture()  # the last state always gets a picture
            wall = time.perf_counter() - t_start
            arrays = {
                "time": np.array(rec["time"]), "pos": np.array(rec["pos"]), "vel": np.array(rec["vel"]), "quat": np.array(rec["quat"]),
                "ang_vel": np.array(rec["ang_vel"]), "cmd_yaw": np.array(rec["cmd_yaw"]), "passed": np.array(rec["passed"], dtype=np.int64),
                "pic_step": np.array(pic_step, dtype=np.int64), "pic_time": np.array([rec["time"][r] for r in pic_step]), "freq": np.array(freq),
            }
            padding = ra.has_padding_row(arrays["pos"])
            result.update(
                steps=steps, rows_saved=int(len(arrays["pos"])), terminated_steps_dropped=dropped, padding_rows_in_saved_positions=padding,
                passages=int(arrays["passed"][-1]), gates_total=total, end=end, sim_seconds=float((len(arrays["pos"]) - 1) / freq),
                flight_wall_seconds=wall, pictures=len(pic_step),
                seconds_per_picture_mean=float(np.mean(picture_seconds)) if picture_seconds else None,
                all_finite=bool(all(np.all(np.isfinite(v)) for v in arrays.values() if v.dtype.kind == "f")),
            )
            if padding:
                raise AssertionError(f"{padding} saved position row(s) equal the padding row (-1, -1, -1)")
            if not result["all_finite"]:
                raise AssertionError("a saved array has a value that is not a number")
            np.savez(sub / f"{tag}.npz", **arrays)
            if writer:
                writer.close()
                writer = None
                result["film_bytes"] = (out / result["film"]).stat().st_size
            result["flown"] = True
    except Exception:
        result["traceback"] = traceback.format_exc()
    finally:
        if writer is not None:
            writer.abort()
        try:
            if env is not None:
                env.close()
        except Exception:
            result["close_traceback"] = traceback.format_exc()
        (sub / f"{tag}_child.json").write_text(json.dumps(result, indent=2, default=str))


# ---------------------------------------------------------------------------------------------
# Parent
# ---------------------------------------------------------------------------------------------


class Runner(ra.Runner):
    """Run A's Runner, but the child is this file."""

    def child_command(self, job: list) -> list:
        a = self.args
        return [sys.executable, str(Path(__file__).resolve()), "child", "--out", str(self.out), "--repo", str(Path(a.repo).resolve()),
                "--assets", str(Path(a.assets).resolve()), "--lsy-commit", a.lsy_commit] + job


def child_job(task: str, look: str, scale: float = 1.0, views=(), fpv: bool = False, camera: str = "none", film: str | None = None) -> list:
    job = ["--task", task, "--look", look, "--drone-scale", repr(float(scale)), "--camera", camera,
           "--nose-mode", NOSE_MODE, "--yaw-rate", repr(float(YAW_RATE_MAX))]
    if views:
        job += ["--views", ",".join(views)]
    if fpv:
        job += ["--fpv"]
    if film:
        job += ["--film", film]
    return job


def ran_ok(entry: dict, flown: bool) -> bool:
    return entry.get("flown") is flown and not entry.get("traceback") and not entry.get("process_error")


def slim(entry: dict) -> dict:
    """The entry without the big fingerprint (its hash stays)."""
    e = dict(entry)
    fp = e.pop("fingerprint", None)
    if fp is not None:
        e["fingerprint_sha256"] = hashlib.sha256(json.dumps(fp, sort_keys=True).encode()).hexdigest()
    e.pop("stderr_tail", None) if not e.get("traceback") and not e.get("process_error") else None
    return e


def read_back_checks(prefix: str, entries: dict, checks: list) -> None:
    gt = {n: e for n, e in entries.items() if e.get("look") != "present"}
    bad = {n: (e.get("read_back") or {}).get("problems", ["no read back"]) for n, e in gt.items() if not (e.get("read_back") or {}).get("ok")}
    ra.add_check(checks, f"{prefix}_added_shapes_have_no_collision_bits_and_are_drawn", bool(gt) and not bad,
                 f"in the compiled model of {len(gt)} GT children: every added geom is a mesh geom of drone:0 with contype 0 and conaffinity 0, group 2, its "
                 f"material color equals the look's MTL color, the drawn extent equals the OBJ extent times the scale; problems: {bad or 'none'}")
    hidden = {n: (e.get("read_back") or {}).get("present_mesh_geoms_hidden") for n, e in gt.items()}
    ra.add_check(checks, f"{prefix}_present_drone_shapes_hidden_in_model", bool(gt) and all(hidden.values()),
                 f"every mesh geom of the present drone (body and propellers) has alpha 0 in the compiled model: {hidden}")


# -- stills --


def make_sheets(out: Path, work: Path, entries: dict) -> dict:
    """Build the labeled sheets from the single pictures. Returns {sheet file: [source pictures]}."""
    import sync_film

    made = {}

    def sheet(name, files, labels, cols):
        imgs = [load_png(work / f) for f in files]
        sync_film.contact_sheet(imgs, labels, cols=cols).save(out / name)
        made[name] = list(files)

    for look in LOOKS:
        tag = child_tag(look, 1.0)
        sheet(f"drone_gt_{look}.png", [still_file(tag, v) for v in VIEWS], [f"{look}: {v.replace('_', ' ')}" for v in VIEWS], 4)
    files = [still_file(child_tag(look, 1.0), "front_quarter") for look in LOOKS] + [still_file(child_tag(look, 1.0), "back_quarter") for look in LOOKS]
    labels = [f"{look}: front quarter" for look in LOOKS] + [f"{look}: back quarter" for look in LOOKS]
    sheet("drone_gt_looks.png", files, labels, 4)
    sheet("drone_gt_scales.png", [still_file(child_tag("cyan", s), "front_quarter_far") for s in SCALES], [f"cyan look, drawing scale {s:g}" for s in SCALES], 3)
    fp, fg = out / "fpv_present.png", out / "fpv_gt.png"
    if fp.is_file() and fg.is_file():
        imgs = [load_png(fp), load_png(fg)]
        sync_film.contact_sheet(imgs, ["drone's camera, present drone", "drone's camera, GT drone (cyan look)"], cols=2).save(out / "fpv_compare.png")
        made["fpv_compare.png"] = ["fpv_present.png", "fpv_gt.png"]
    return made


def cmd_stills(args) -> None:
    runner = Runner(args)
    out = runner.out
    work = out / "stills_work"
    work.mkdir(exist_ok=True)
    res = {"children": {}, "checks": [], "versions": {}, "numbers": {}}
    checks = res["checks"]
    try:
        jobs = [("present", 1.0, VIEWS, True)]
        jobs += [(look, 1.0, VIEWS + (("front_quarter_far",) if look == "cyan" else ()), look == "cyan") for look in LOOKS]
        jobs += [("cyan", s, ("front_quarter_far",), False) for s in SCALES if s != 1.0]
        entries = {}
        for look, scale, views, fpv in jobs:
            tag = child_tag(look, scale)
            entries[tag] = runner.flight(tag, "stills", child_job("stills", look, scale, views, fpv), needs_gl=True)
        res["versions"] = ra.versions_of(list(entries.values()))
        for tag, e in entries.items():
            res["children"][tag] = {k: e.get(k) for k in ("look", "drone_scale", "method", "flown", "views", "start_pad_error_m", "orange", "fpv", "stills", "gt_info",
                                                          "read_back", "contact_masks", "drone_drawn_geoms")}
            res["children"][tag]["error"] = ra.short_error(e)
        ok_children = {t: e for t, e in entries.items() if ran_ok(e, False)}
        ra.add_check(checks, "stills_all_children_ran", len(ok_children) == len(entries),
                     f"{len(ok_children)} of {len(entries)} environments made their pictures; failed: " + (", ".join(f"{t} ({ra.short_error(e)})" for t, e in entries.items() if t not in ok_children) or "none"))
        base = ok_children.get(child_tag("present", 1.0), {})
        gt_children = {t: e for t, e in ok_children.items() if e.get("look") != "present"}
        fingerprint_checks("stills", base, gt_children, checks)
        read_back_checks("stills", gt_children, checks)

        # pictures
        def img(tag, view):
            return load_png(work / still_file(tag, view))

        def mask(tag, view):
            return np.load(work / f"{tag}_{view}_mask.npz")["mask"]

        have = lambda tag, views: all((work / still_file(tag, v)).is_file() and (work / f"{tag}_{v}_mask.npz").is_file() for v in views)  # noqa: E731
        t_present, t_cyan, t_mag = child_tag("present", 1.0), child_tag("cyan", 1.0), child_tag("magenta", 1.0)
        # positive control 1: the GT drone is really drawn, and is not the present drone
        if have(t_present, ("front_quarter",)) and have(t_cyan, ("front_quarter",)):
            s1 = diff_share(img(t_present, "front_quarter"), img(t_cyan, "front_quarter"))
            res["numbers"]["gt_vs_present_front_quarter_share"] = s1
            ra.add_check(checks, "positive_control_gt_differs_from_present_drone", s1 > MIN_GT_VS_PRESENT,
                         f"same camera, front quarter: {100 * s1:.2f} % of all pixels differ (some channel by more than {DIFF_THRESHOLD}) between the present drone and the GT drone (cyan look); needs more than {100 * MIN_GT_VS_PRESENT:g} %", value=s1)
        else:
            ra.add_check(checks, "positive_control_gt_differs_from_present_drone", False, "the front quarter pictures of the present drone or of the cyan look are missing")
        # positive control 2: the colors come from the look
        if have(t_cyan, ("front_quarter",)) and have(t_mag, ("front_quarter",)):
            s2 = diff_share(img(t_cyan, "front_quarter"), img(t_mag, "front_quarter"))
            res["numbers"]["cyan_vs_magenta_front_quarter_share"] = s2
            ra.add_check(checks, "positive_control_looks_with_different_body_colors_differ", s2 > MIN_LOOK_DIFF,
                         f"same camera, front quarter: {100 * s2:.2f} % of all pixels differ between the cyan look (orange body) and the magenta look (lime body); needs more than {100 * MIN_LOOK_DIFF:g} %", value=s2)
        else:
            ra.add_check(checks, "positive_control_looks_with_different_body_colors_differ", False, "the front quarter pictures of the cyan or the magenta look are missing")
        # the present drone's hi-vis orange must be gone from the magenta look (body lime): counted on the drone's own pixels
        mag = (entries.get(t_mag) or {}).get("stills") or []
        pre = (entries.get(t_present) or {}).get("stills") or []
        n_orange_mag = sum(s["orange_pixels_on_drone"] for s in mag)
        n_orange_pre = sum(s["orange_pixels_on_drone"] for s in pre)
        n_drone_mag = sum(s["drone_pixels"] for s in mag)
        res["numbers"].update(orange_pixels_on_drone_magenta=n_orange_mag, orange_pixels_on_drone_present=n_orange_pre, drone_pixels_magenta=n_drone_mag)
        ra.add_check(checks, "present_drone_drawing_is_hidden_no_hi_vis_orange_in_magenta_look", bool(mag) and n_drone_mag > 0 and n_orange_mag == 0 and n_orange_pre > 0,
                     f"orange pixels (tests/crazyflow_track_look.py share_masks) on the drone's own pixels over the 8 views: magenta look {n_orange_mag} of {n_drone_mag} drone pixels (must be 0); "
                     f"the present drone, as control, {n_orange_pre} (must be above 0, otherwise this check could not see orange)")
        # front differs from back
        fb = {}
        for look in LOOKS:
            tag = child_tag(look, 1.0)
            if have(tag, ("front", "back")):
                union = mask(tag, "front") | mask(tag, "back")
                fb[look] = diff_share(img(tag, "front"), img(tag, "back"), union)
        res["numbers"]["front_vs_back_share_of_drone_pixels"] = fb
        ra.add_check(checks, "front_differs_from_back", len(fb) == len(LOOKS) and all(v > MIN_FRONT_BACK for v in fb.values()),
                     f"share of the drone's pixels (union of the drone masks of the two views) that differ between the front view and the back view: "
                     + ", ".join(f"{k} {100 * v:.1f} %" for k, v in fb.items()) + f"; each needs more than {100 * MIN_FRONT_BACK:g} %")
        # drawing scale: more drone pixels at a larger scale, same camera
        counts = {}
        for s in SCALES:
            e = entries.get(child_tag("cyan", s)) or {}
            far = [x for x in (e.get("stills") or []) if x["view"] == "front_quarter_far"]
            counts[s] = far[0]["drone_pixels"] if far else None
        res["numbers"]["drone_pixels_front_quarter_far_by_scale"] = {f"{k:g}": v for k, v in counts.items()}
        ra.add_check(checks, "positive_control_scale_grows_the_drawing", None not in counts.values() and counts[1.0] < counts[1.5] < counts[2.0],
                     f"drone pixels in the same far front-quarter view: scale 1.0 -> {counts[1.0]}, 1.5 -> {counts[1.5]}, 2.0 -> {counts[2.0]}; each must be larger than the one before")
        # sheets
        made = {}
        try:
            made = make_sheets(out, work, entries)
        except Exception:
            res["sheet_error"] = traceback.format_exc()
        want = [f"drone_gt_{look}.png" for look in LOOKS] + ["drone_gt_looks.png", "drone_gt_scales.png", "fpv_present.png", "fpv_gt.png", "fpv_compare.png"]
        want += [still_file(child_tag(look, 1.0), v) for look in LOOKS for v in VIEWS]
        missing = [f for f in want if not ((out / f).is_file() or (work / f).is_file())]
        ra.add_check(checks, "stills_and_sheets_exist", not missing, f"{len(want) - len(missing)} of {len(want)} files exist (sheets in {out}, single pictures in {work}); missing: {missing or 'none'}")
        if "fpv_present.png" not in missing and "fpv_gt.png" not in missing:
            res["numbers"]["fpv_share_of_pixels_that_differ"] = diff_share(load_png(out / "fpv_present.png"), load_png(out / "fpv_gt.png"))
        res["sheets"] = made
    except Exception:
        res["error"] = traceback.format_exc()
        ra.add_check(checks, "stills_step_completed", False, "the stills step stopped: " + res["error"].strip().splitlines()[-1])
    finally:
        res["child_entries"] = {t: slim(e) for t, e in locals().get("entries", {}).items()}
        (out / "stills.json").write_text(json.dumps(res, indent=2, default=str))
    print("stills done:", {c["name"]: c["ok"] for c in checks})


# -- proof --


def cmd_proof(args) -> None:
    runner = Runner(args)
    out = runner.out
    res = {"flights": {}, "checks": [], "versions": {}, "numbers": {}}
    checks = res["checks"]
    try:
        spec = [("proof_present", "present", 1.0), ("proof_gt_1p0", "cyan", 1.0), ("proof_gt_2p0", "cyan", 2.0)]
        entries, states = {}, {}
        for tag, look, scale in spec:
            entries[tag] = runner.flight(tag, "proof", child_job("flight", look, scale), needs_gl=False)
            npz = out / "proof" / f"{tag}.npz"
            if ra.child_ok(entries[tag]) and npz.is_file():
                states[tag] = ra.load_npz(npz)
        res["versions"] = ra.versions_of(list(entries.values()))
        ref = states.get("proof_present")
        worst = None
        for tag, look, scale in spec:
            e = entries[tag]
            res["flights"][tag] = {k: e.get(k) for k in ("look", "drone_scale", "steps", "rows_saved", "passages", "gates_total", "end", "sim_seconds", "flight_wall_seconds",
                                                         "start_pad_error_m")}
            res["flights"][tag]["error"] = ra.short_error(e)
            if tag != "proof_present":
                if ref is not None and tag in states:
                    cmp = ra.compare_states(ref, states[tag])
                    res["flights"][tag]["compare_with_present"] = cmp
                    diffs = [cmp["max_position_difference_m"] or 0.0, cmp["max_quaternion_difference"] or 0.0, cmp["max_velocity_difference"] or 0.0]
                    w = max(diffs)
                    worst = w if worst is None else max(worst, w)
                    ra.add_check(checks, f"proof_flight_{tag}_equals_present_flight", cmp["equal"] and cmp["max_velocity_difference"] == 0.0,
                                 f"{tag} against the flight with the present drone drawing: {cmp['rows_other']} rows against {cmp['rows_reference']}; largest absolute difference of "
                                 f"position {cmp['max_position_difference_m']} m, quaternion {cmp['max_quaternion_difference']}, velocity {cmp['max_velocity_difference']}; passes only at exactly 0.0", value=w)
                else:
                    ra.add_check(checks, f"proof_flight_{tag}_equals_present_flight", False, f"the flight {tag} or the reference flight did not run: {ra.short_error(e)}")
        res["numbers"]["largest_difference_of_any_proof_flight"] = worst
        gates = {t: (e.get("passages"), e.get("gates_total")) for t, e in entries.items()}
        ra.add_check(checks, "proof_every_flight_passes_all_5_gates", all(ra.child_ok(e) and e.get("gates_total") == 5 and e.get("passages") == 5 for e in entries.values()),
                     f"passages / gates in the course, per flight: {gates}; each must be 5 of 5")
        pads = {t: ra.has_padding_row(z["pos"]) for t, z in states.items()}
        ra.add_check(checks, "proof_no_padding_row", bool(pads) and all(v == 0 for v in pads.values()), f"rows equal to (-1, -1, -1) in the saved flights: {pads}")
        base = entries.get("proof_present", {})
        fingerprint_checks("proof", base, {t: e for t, e in entries.items() if t != "proof_present" and ra.child_ok(e)}, checks)
        read_back_checks("proof", {t: e for t, e in entries.items() if t != "proof_present" and ra.child_ok(e)}, checks)
        res["child_entries"] = {t: slim(e) for t, e in entries.items()}
    except Exception:
        res["error"] = traceback.format_exc()
        ra.add_check(checks, "proof_step_completed", False, "the proof step stopped: " + res["error"].strip().splitlines()[-1])
    finally:
        (out / "proof.json").write_text(json.dumps(res, indent=2, default=str))
    print("proof done:", {c["name"]: c["ok"] for c in checks})


# -- film --


def cmd_film(args) -> None:
    import sync_film

    runner = Runner(args)
    out = runner.out
    res = {"films": {}, "checks": [], "versions": {}}
    checks = res["checks"]
    try:
        spec = [("film_gt_1p0", 1.0, "chase_gt.mp4"), ("film_gt_2p0", 2.0, "chase_gt_scale_2p0.mp4")]
        ref_file = out / "proof" / "proof_present.npz"
        ref = ra.load_npz(ref_file) if ref_file.is_file() else None
        entries, decoded = {}, {}
        for tag, scale, film in spec:
            e = runner.flight(tag, "film", child_job("flight", "cyan", scale, camera="chase", film=film), needs_gl=True)
            entries[tag] = e
            row = {k: e.get(k) for k in ("film", "film_bytes", "pictures", "steps", "passages", "end", "method", "flight_wall_seconds", "seconds_per_picture_mean")}
            row["error"] = ra.short_error(e)
            res["films"][film] = row
            npz = out / "film" / f"{tag}.npz"
            if ra.child_ok(e) and npz.is_file():
                z = ra.load_npz(npz)
                if ref is not None:
                    cmp = ra.compare_states(ref, z)
                    row["compare_with_present_proof_flight"] = cmp
                    ra.add_check(checks, f"film_flight_scale_{scale_tag(scale)}_equals_present_flight", cmp["equal"] and cmp["max_velocity_difference"] == 0.0,
                                 f"filmed flight (GT drone, scale {scale:g}) against the proof flight with the present drone drawing: {cmp['rows_other']} rows against {cmp['rows_reference']}; "
                                 f"largest difference of position {cmp['max_position_difference_m']} m, quaternion {cmp['max_quaternion_difference']}, velocity {cmp['max_velocity_difference']} (must be exactly 0.0)",
                                 value=cmp["max_position_difference_m"])
                else:
                    ra.add_check(checks, f"film_flight_scale_{scale_tag(scale)}_equals_present_flight", False, f"{ref_file} does not exist: run the proof step first")
                film_path = out / film
                if film_path.is_file() and film_path.stat().st_size > 0:
                    try:
                        sheet = f"{Path(film).stem}_contact.png"
                        info = sync_film.film_contact_sheet(film_path, WIDTH, HEIGHT, z["pic_time"], out / sheet)
                        row["contact_sheet"] = info
                        decoded[tag] = info["pictures_decoded"]
                    except Exception:
                        row["contact_sheet_error"] = traceback.format_exc()[-800:]
            else:
                ra.add_check(checks, f"film_flight_scale_{scale_tag(scale)}_equals_present_flight", False, f"the film flight {tag} did not run: {ra.short_error(e)}")
        res["versions"] = ra.versions_of(list(entries.values()))
        written = {t: e.get("pictures") for t, e in entries.items()}
        ra.add_check(checks, "film_picture_counts_equal", len(decoded) == len(spec) and all(decoded.get(t) == written.get(t) and written.get(t) for t in decoded),
                     f"pictures written {written}; pictures decoded from the mp4 files {decoded}; they must be equal for each film")
        files = [f for _, _, f in spec] + [f"{Path(f).stem}_contact.png" for _, _, f in spec]
        missing = [f for f in files if not (out / f).is_file() or (out / f).stat().st_size == 0]
        ra.add_check(checks, "film_files_and_contact_sheets_exist", not missing, f"{len(files) - len(missing)} of {len(files)} files exist in {out}; missing: {missing or 'none'}")
        read_back_checks("film", {t: e for t, e in entries.items() if ra.child_ok(e)}, checks)
        res["child_entries"] = {t: slim(e) for t, e in entries.items()}
    except Exception:
        res["error"] = traceback.format_exc()
        ra.add_check(checks, "film_step_completed", False, "the film step stopped: " + res["error"].strip().splitlines()[-1])
    finally:
        (out / "film.json").write_text(json.dumps(res, indent=2, default=str))
    print("film done:", {c["name"]: c["ok"] for c in checks})


# -- summary and check --


def read_json(path: Path):
    return json.loads(path.read_text()) if path.is_file() else None


def cmd_summary(args) -> None:
    out = Path(args.out).resolve()
    stills, proof, film = (read_json(out / f) for f in ("stills.json", "proof.json", "film.json"))
    checks = []
    outcomes = {"obj_parts_self_test": args.selftest_outcome, "drone_look_self_test": args.look_selftest_outcome}
    if args.selftest_outcome is not None:
        ra.add_check(checks, "self_test_of_obj_parts", args.selftest_outcome == "success", f"python tools/obj_parts.py --self-test ended with outcome {args.selftest_outcome!r}")
    if args.look_selftest_outcome is not None:
        ra.add_check(checks, "self_test_of_drone_look", args.look_selftest_outcome == "success", f"python tools/drone_look.py ended with outcome {args.look_selftest_outcome!r}")
    for label, data in (("stills", stills), ("proof", proof), ("film", film)):
        if data is None:
            ra.add_check(checks, f"{label}_result_file_exists", False, f"{label}.json was not written: that step did not run or crashed before it could write")
        else:
            checks.extend(data.get("checks", []))
    failed = [c["name"] for c in checks if not c["ok"]]
    versions = {**(stills or {}).get("versions", {}), **(proof or {}).get("versions", {}), **(film or {}).get("versions", {})}
    result = {"passed": not failed and bool(checks), "n_checks": len(checks), "failed": failed, "checks": checks, "outcomes": outcomes, "versions": versions,
              "python": platform.python_version()}
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))

    L = ["### Drone GT: the new drone model drawn in the simulator", ""]
    L.append(f"**{'PASS' if result['passed'] else 'FAIL'}**: {len(checks) - len(failed)} of {len(checks)} checks passed" + (f"; failed: {', '.join(failed)}" if failed else "") + ".")
    L += ["", "#### Proof flights (no pictures; the drawing is the only difference)", ""]
    if proof:
        L += ["| flight | passages | steps | end | largest position difference to the present drawing (m) | quaternion | velocity |", "|---|---|---|---|---|---|---|"]
        for t, f in proof["flights"].items():
            c = f.get("compare_with_present") or {}
            L.append(f"| {t} | {f.get('passages')} of {f.get('gates_total')} | {f.get('steps')} | {f.get('end')} | {c.get('max_position_difference_m', '-')} | {c.get('max_quaternion_difference', '-')} | {c.get('max_velocity_difference', '-')} |")
        L.append("")
        L.append(f"Largest difference of any proof flight: {proof.get('numbers', {}).get('largest_difference_of_any_proof_flight')}.")
    else:
        L.append("The proof step did not write proof.json.")
    L += ["", "#### Pictures", ""]
    if stills:
        n = stills.get("numbers", {})
        for k, v in n.items():
            L.append(f"- {k}: {v}")
        L.append("- sheets: " + ", ".join(sorted(stills.get("sheets", {}))))
    else:
        L.append("The stills step did not write stills.json.")
    L += ["", "#### Films", ""]
    if film:
        L += ["| film | pictures | size (MB) | seconds per picture (mean) |", "|---|---|---|---|"]
        for f, e in film["films"].items():
            p = out / f
            size = p.stat().st_size / 1e6 if p.is_file() else None
            L.append(f"| {f} | {e.get('pictures')} | {size if size is None else round(size, 2)} | {e.get('seconds_per_picture_mean')} |")
    else:
        L.append("The film step did not write film.json.")
    L += ["", "#### Checks", "", "| check | result | detail |", "|---|---|---|"]
    for c in checks:
        L.append(f"| {c['name']} | {'pass' if c['ok'] else 'FAIL'} | {str(c['detail']).replace('|', '/')} |")
    L += ["", "Versions: " + ", ".join(f"{k} {v}" for k, v in versions.items()), ""]
    (out / "summary.md").write_text("\n".join(L))
    print(("PASS: " if result["passed"] else "FAIL: ") + f"{len(checks) - len(failed)} of {len(checks)} checks passed" + (f"; failed: {failed}" if failed else ""))
    sys.exit(0 if result["passed"] else 1)


def cmd_check(args) -> None:
    f = Path(args.result)
    if not f.is_file():
        print(f"FAIL: result file {f} does not exist")
        sys.exit(1)
    r = json.loads(f.read_text())
    if r.get("passed"):
        print(f"PASS: {r['n_checks']} checks passed")
        sys.exit(0)
    print(f"FAIL: failed checks: {r.get('failed')}")
    sys.exit(1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["stills", "proof", "film", "summary", "check", "child"])
    parser.add_argument("--out", help="folder for all output files")
    parser.add_argument("--repo", help="checkout of lsy_drone_racing")
    parser.add_argument("--assets", default=str(REPO_DIR / "assets" / "gates" / "square"), help="folder with the new gate model")
    parser.add_argument("--lsy-commit", default="unknown", help="commit of the checkout, only recorded")
    parser.add_argument("--selftest-outcome", help="summary: outcome of the obj_parts self-test step")
    parser.add_argument("--look-selftest-outcome", help="summary: outcome of the drone_look self-test step")
    parser.add_argument("--result", help="check: result.json")
    # internal (child)
    parser.add_argument("--tag")
    parser.add_argument("--subdir", default="states")
    parser.add_argument("--task", default="stills", choices=["stills", "flight"])
    parser.add_argument("--look", default="present", choices=["present", *LOOKS])
    parser.add_argument("--views", default="")
    parser.add_argument("--fpv", action="store_true")
    parser.add_argument("--nose-mode", default=NOSE_MODE, choices=["forward", "fixed"])
    parser.add_argument("--yaw-rate", type=float, default=YAW_RATE_MAX)
    parser.add_argument("--camera", default="none", choices=["none", "chase"])
    parser.add_argument("--drone-scale", type=float, default=1.0)
    parser.add_argument("--film")
    args = parser.parse_args()
    if args.command == "check":
        if not args.result:
            parser.error("check needs --result")
        cmd_check(args)
        return
    if not args.out:
        parser.error("--out is required")
    if args.command == "summary":
        cmd_summary(args)
        return
    if not args.repo:
        parser.error("--repo is required")
    if args.command == "child":
        run_child(args)
    elif args.command == "stills":
        cmd_stills(args)
    elif args.command == "proof":
        cmd_proof(args)
    elif args.command == "film":
        cmd_film(args)


if __name__ == "__main__":
    main()
