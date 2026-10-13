"""Draw STILL pictures of the new square gate as the simulator draws it (no flight).

The new gate model (assets/gates/square/) must be in LSY's asset folder before this test runs; the
workflow copies it there. This test checks that first, then makes the level-0 environment of
LSY Drone Racing, resets it, and draws 640 x 480 pictures:
  - the on-drone camera fpv_cam:0 after the reset,
  - the whole track with the camera settings of level0.toml,
  - ONE gate (gate 0, the first gate of the level, pose read from the simulator) from the front at
    0.7 m, 1.5 m and 3.0 m, from 45 degrees at 1.5 m, from the back at 1.5 m, a corner with its
    spring at 0.35 m (straight and at an angle), and the post at 0.6 m.
It also reads the compiled MuJoCo model back and checks colors, springs, textures and pixel shares.

Parent mode (default): runs one child per drawing method (`egl`, then `osmesa` if egl gave no
pictures), writes result.json and summary.md.
Child mode (--child --method METHOD): does the work.
Check mode (--check FILE): applies the pass rule; only the standard library is imported.
"""

import argparse
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
import time
import traceback
from pathlib import Path

METHODS = ("egl", "osmesa")
WIDTH, HEIGHT = 640, 480
SEED = 0
CHILD_TIMEOUT_S = 1500
GATE = "gate:0"  # body name after LSY attached the first gate (race_core.py line 499 uses gate:{i})
# Files of the new model; the test compares each one with the file beside the imported LSY package.
MODEL_FILES = (
    "gate.xml", "gate_top.png", "gate_bottom.png", "gate_left.png", "gate_right.png",
    "gate_post_bark.png", "spring.stl",
)
FRAME_GEOMS = ("frame_top", "frame_bottom", "frame_left", "frame_right")
SPRING_GEOMS = ("rope_top_left", "rope_top_right", "rope_bottom_left", "rope_bottom_right")
# End points of the springs in gate coordinates (the old rope end points of gate.xml).
SPRING_ENDS = {
    "rope_top_left": ((0.0, -0.34, 0.34), (0.0, -0.29, 0.29)),
    "rope_top_right": ((0.0, 0.34, 0.34), (0.0, 0.29, 0.29)),
    "rope_bottom_left": ((0.0, -0.34, -0.34), (0.0, -0.29, -0.29)),
    "rope_bottom_right": ((0.0, 0.34, -0.34), (0.0, 0.29, -0.29)),
}
POST_GEOMS = ("gate_stand", "post_bark_px", "post_bark_nx", "post_bark_py", "post_bark_ny")
NEON_RGB = (240, 255, 0)
SPRING_RGB = (214, 40, 40)
BARK_SIZE = (256, 2560)  # width, height of gate_post_bark.png
# Views of gate 0: label -> (target in gate coordinates, camera offset from the target in gate
# coordinates). Gate x axis = gate normal; the textured front faces -x, so the front camera sits at -x.
C45 = math.cos(math.radians(45))
C35, S35 = math.cos(math.radians(35)), math.sin(math.radians(35))
VIEWS = {
    "front_0p7": ((0, 0, 0), (-0.7, 0, 0)),
    "front_1p5": ((0, 0, 0), (-1.5, 0, 0)),
    "front_3p0": ((0, 0, 0), (-3.0, 0, 0)),
    "side45_1p5": ((0, 0, 0), (-1.5 * C45, 1.5 * C45, 0)),
    "back_1p5": ((0, 0, 0), (1.5, 0, 0)),
    "corner_front_0p35": ((0, -0.315, 0.315), (-0.35, 0, 0)),
    "corner_oblique_0p35": ((0, -0.315, 0.315), (-0.35 * C35, 0.35 * S35, 0)),
    "post_oblique_0p6": ((0, 0, -0.53), (-0.6 * C35, 0.6 * S35, 0)),  # -0.53 = 0.17 m above ground
}
PICTURES = ("fpv_after_reset", "track_wide") + tuple(VIEWS)
MIN_PAIRWISE_DIFF = 1.0  # smallest mean absolute difference (0..255) between any two pictures


# ---------------------------------------------------------------------------------------------
# Pass rule (used by the parent and by --check); standard library only
# ---------------------------------------------------------------------------------------------


def evaluate(result: dict, base: Path) -> tuple[bool, str]:
    problems = []
    entry = result.get("run", {})
    if entry.get("traceback") or entry.get("process_error"):
        text = (entry.get("traceback") or entry.get("process_error")).strip().splitlines()
        problems.append("run failed: " + (text[-1][:200] if text else "no detail"))
    check = entry.get("asset_check") or {}
    if not check.get("ok"):
        problems.append("asset check failed or did not run")
    for name, ok in (entry.get("model_checks") or {}).items():
        if not ok.get("ok"):
            problems.append(f"model check {name} failed: {ok.get('detail')}")
    if not entry.get("model_checks"):
        problems.append("no model checks were made")
    files = {p.get("label"): p for p in entry.get("pictures", [])}
    for label in PICTURES:
        p = files.get(label)
        if not p or not p.get("file") or not (base / p["file"]).is_file() or (base / p["file"]).stat().st_size == 0:
            problems.append(f"picture {label} missing")
    front = files.get("front_1p5", {})
    if not front.get("neon_share_center", 0) > 0:
        problems.append("front_1p5: no neon yellow pixels near the gate")
    if not front.get("spring_red_share_center", 0) > 0:
        problems.append("front_1p5: no spring red pixels near the gate")
    diff = entry.get("min_pairwise_mean_abs_diff")
    if diff is None or diff < MIN_PAIRWISE_DIFF:
        problems.append(f"pictures are too much alike (smallest mean abs difference {diff})")
    if problems:
        return False, "; ".join(problems)
    return True, (
        f"{len(PICTURES)} pictures drawn, all model checks passed; front_1p5 neon share "
        f"{front['neon_share_center']:.4%}, spring red share {front['spring_red_share_center']:.4%} (centre crop)"
    )


def check(result_file: Path) -> int:
    if not result_file.is_file():
        print(f"FAIL: result file {result_file} does not exist")
        return 1
    ok, reason = evaluate(json.loads(result_file.read_text()), result_file.parent)
    print(("PASS: " if ok else "FAIL: ") + reason)
    return 0 if ok else 1


# ---------------------------------------------------------------------------------------------
# Child
# ---------------------------------------------------------------------------------------------


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_assets(repo: Path, new_assets: Path) -> dict:
    """The model LSY loads must be the new one. Raises AssertionError with a clear text if not."""
    import lsy_drone_racing
    from lsy_drone_racing.envs.race_core import RaceCoreEnv

    package_dir = Path(lsy_drone_racing.__file__).resolve().parent
    detail = {"package_dir": str(package_dir), "gate_spec_path": str(RaceCoreEnv.gate_spec_path), "files": {}}
    # pyproject.toml line 90: lsy_drone_racing = { path = ".", editable = true }, so the imported
    # package must lie inside the checkout that the workflow copied the new model into.
    if repo.resolve() not in package_dir.parents:
        raise AssertionError(f"imported lsy_drone_racing ({package_dir}) is not inside the checkout {repo.resolve()}")
    # race_core.py line 330: gate_spec_path = Path(__file__).parent / "assets/gate.xml"
    asset_dir = Path(RaceCoreEnv.gate_spec_path).resolve().parent
    if asset_dir != (package_dir / "envs" / "assets").resolve():
        raise AssertionError(f"gate_spec_path {RaceCoreEnv.gate_spec_path} is not in {package_dir}/envs/assets")
    wrong = []
    for name in MODEL_FILES:
        new, there = new_assets / name, asset_dir / name
        a = sha256(new) if new.is_file() else None
        b = sha256(there) if there.is_file() else None
        detail["files"][name] = {"new": a, "in_lsy": b}
        if a is None or a != b:
            wrong.append(name)
    if wrong:
        raise AssertionError(
            "the files beside the imported LSY package differ from the new model (or are missing): "
            + ", ".join(wrong) + f" (asset folder {asset_dir})"
        )
    detail["ok"] = True
    return detail


def rgb_close(rgb, target, tol: float) -> bool:
    return all(abs(float(rgb[i]) * 255.0 - target[i]) <= tol for i in range(3))


def mask_neon(img):
    r, g, b = (img[..., i].astype("int16") for i in range(3))
    return (r >= 190) & (g >= 210) & (b <= 100) & (g - b >= 130)


def mask_red(img):
    r, g, b = (img[..., i].astype("int16") for i in range(3))
    # spring red (214, 40, 40) also in shade; brown bark (G about 0.6 R) stays outside
    return (r >= 90) & (g * 100 <= 45 * r) & (b * 100 <= 45 * r)


def model_checks(mujoco, np, m, d, gpos, gmat, n_gates: int) -> dict:
    """Read the compiled model back (names carry the suffix ':<gate number>')."""
    out = {}

    def name_id(kind, name):
        return mujoco.mj_name2id(m, kind, name)  # mujoco.h line 599

    def entry(name, ok, detail):
        out[name] = {"ok": bool(ok), "detail": detail}

    # springs: mesh asset, four mesh geoms without collision, red material, end points
    mesh_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, i) for i in range(m.nmesh)]  # mujoco.h line 602
    spring_meshes = [n for n in mesh_names if n and n.startswith("gate_spring")]
    mesh_id = name_id(mujoco.mjtObj.mjOBJ_MESH, "gate_spring:0")
    faces = int(m.mesh_facenum[mesh_id]) if mesh_id >= 0 else 0
    entry("spring_mesh_present", mesh_id >= 0 and faces >= 2000,
          f"{len(spring_meshes)} spring meshes for {n_gates} gates ({spring_meshes}); gate_spring:0 has {faces} faces")
    mesh_type = int(mujoco.mjtGeom.mjGEOM_MESH)  # mjtype.h line 108 is mjGEOM_MESH
    problems = []
    for i in range(n_gates):
        for name in SPRING_GEOMS:
            gid = name_id(mujoco.mjtObj.mjOBJ_GEOM, f"{name}:{i}")
            if gid < 0:
                problems.append(f"{name}:{i} not found")
                continue
            matid = int(m.geom_matid[gid])
            if int(m.geom_type[gid]) != mesh_type:
                problems.append(f"{name}:{i} is not a mesh geom")
            if int(m.geom_contype[gid]) != 0 or int(m.geom_conaffinity[gid]) != 0:
                problems.append(f"{name}:{i} has collision")
            if matid < 0 or not rgb_close(m.mat_rgba[matid][:3], SPRING_RGB, 1.5):
                problems.append(f"{name}:{i} is not spring red")
    count0 = sum(
        1 for name in SPRING_GEOMS
        if name_id(mujoco.mjtObj.mjOBJ_GEOM, f"{name}:0") >= 0
        and int(m.geom_type[name_id(mujoco.mjtObj.mjOBJ_GEOM, f"{name}:0")]) == mesh_type
    )
    entry("spring_geoms", count0 == 4 and not problems, f"{count0} spring mesh geoms in gate 0; problems: {problems}")

    ends = []
    worst = 0.0
    for name, (p0, p1) in SPRING_ENDS.items():
        gid = name_id(mujoco.mjtObj.mjOBJ_GEOM, f"{name}:0")
        mid = int(m.geom_dataid[gid])
        vadr, vnum = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
        verts = np.asarray(m.mesh_vert).reshape(-1, 3)[vadr:vadr + vnum].astype(np.float64)
        world = np.asarray(d.geom_xpos[gid]) + verts @ np.asarray(d.geom_xmat[gid]).reshape(3, 3).T
        e0 = gpos + gmat @ np.array(p0)
        e1 = gpos + gmat @ np.array(p1)
        length = float(np.linalg.norm(e1 - e0))
        axis = (e1 - e0) / length
        t = (world - e0) @ axis
        radial = np.linalg.norm((world - e0) - np.outer(t, axis), axis=1)
        err = max(abs(float(t.min())), abs(float(t.max()) - length))
        worst = max(worst, err)
        ends.append({"geom": name, "axis_min": float(t.min()), "axis_max": float(t.max()),
                     "length": length, "max_radial": float(radial.max())})
    entry("spring_ends_on_corner_points", worst < 2e-3 and all(e["max_radial"] < 0.0115 for e in ends),
          f"worst end error {worst:.2e} m; {ends}")

    # frame: yellow and glowing
    bad = []
    emission = None
    for name in FRAME_GEOMS:
        gid = name_id(mujoco.mjtObj.mjOBJ_GEOM, f"{name}:0")
        matid = int(m.geom_matid[gid]) if gid >= 0 else -1
        if matid < 0 or not rgb_close(m.mat_rgba[matid][:3], NEON_RGB, 1.5):
            bad.append(name)
        else:
            emission = float(m.mat_emission[matid])  # mjmodel.h line 695
            if emission < 0.5:
                bad.append(f"{name} emission {emission}")
    entry("frame_neon_yellow_glowing", not bad, f"emission {emission}; wrong: {bad}")

    # post: material with a texture of the bark size
    bad, info = [], {}
    for name in POST_GEOMS:
        gid = name_id(mujoco.mjtObj.mjOBJ_GEOM, f"{name}:0")
        matid = int(m.geom_matid[gid]) if gid >= 0 else -1
        if matid < 0:
            bad.append(f"{name}: no material")
            continue
        texids = [int(t) for t in np.asarray(m.mat_texid).reshape(m.nmat, -1)[matid] if int(t) >= 0]
        if not texids:
            bad.append(f"{name}: material has no texture")
            continue
        size = (int(m.tex_width[texids[0]]), int(m.tex_height[texids[0]]))
        info[name] = size
        if size != BARK_SIZE:
            bad.append(f"{name}: texture {size} is not {BARK_SIZE}")
    entry("post_material_has_bark_texture", not bad, f"texture sizes {info}; wrong: {bad}")
    return out


def run_child(method: str, repo: Path, new_assets: Path, out: Path, lsy_commit: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    result = {
        "method": method,
        "MUJOCO_GL": os.environ.get("MUJOCO_GL"),
        "versions": {},
        "asset_check": None,
        "model_checks": {},
        "gate_pose": None,
        "camera_info": {},
        "pictures": [],
        "traceback": None,
    }
    env = None
    try:
        os.environ["SCIPY_ARRAY_API"] = "1"  # crazyflow's own camera examples set this
        import gymnasium
        import imageio.v3 as iio
        import importlib.metadata as md
        import jax
        import mujoco
        import numpy as np
        from lsy_drone_racing.utils import load_config

        result["versions"] = {
            "mujoco": mujoco.__version__,
            "jax": jax.__version__,
            "crazyflow": md.version("crazyflow"),
            "lsy_drone_racing": md.version("lsy_drone_racing"),
            "lsy_drone_racing_commit": lsy_commit,
        }

        # Point 4: before the environment is made, the model LSY loads must be the new one.
        result["asset_check"] = check_assets(repo, new_assets)

        cfg = load_config(repo / "config" / "level0.toml")
        cfg.sim.render = False
        env = gymnasium.make(
            cfg.env.id,
            freq=cfg.env.freq,
            sim_config=cfg.sim,
            sensor_range=cfg.env.sensor_range,
            control_mode=cfg.env.control_mode,
            track=cfg.env.track,
            disturbances=cfg.env.get("disturbances"),
            randomizations=cfg.env.get("randomizations"),
            seed=SEED,
        )
        core = env.unwrapped
        obs, _ = env.reset()
        m, d = core.sim.mj_model, core.sim.mj_data
        n_gates = len(cfg.env.track.gates)
        result["camera_info"] = {
            "fovy_deg": float(m.vis.global_.fovy),
            "extent_m": float(m.stat.extent),
            "near_plane_m": float(m.vis.map.znear * m.stat.extent),  # mjmodel.h line 175 and visualize.c line 543
            "width": WIDTH,
            "height": HEIGHT,
        }
        images = {}

        def sync() -> None:
            # Same sync as RaceCoreEnv.render: copy gate positions into the MuJoCo data.
            if not core.data.sim_data.core.mjx_synced:
                core.data, core.sim.mjx_data = core._render_sync(core.data, core.sim.mjx_data)

        def draw(label: str, settings: dict, **render_kwargs) -> None:
            entry = {"label": label, "file": f"{label}.png", "camera_settings": settings}
            result["pictures"].append(entry)
            sync()
            t0 = time.perf_counter()
            img = core.sim.render(mode="rgb_array", width=WIDTH, height=HEIGHT, **render_kwargs)
            entry["seconds"] = time.perf_counter() - t0
            img = np.ascontiguousarray(img)
            if img.shape != (HEIGHT, WIDTH, 3) or img.dtype != np.uint8:
                raise ValueError(f"unexpected picture {img.shape} {img.dtype}")
            iio.imwrite(out / entry["file"], img)  # PNG, lossless
            images[label] = img
            entry.update(shape=list(img.shape), mean=float(img.mean()), std=float(img.std()))
            h, w = img.shape[:2]
            crop = img[h // 2 - 170:h // 2 + 170, w // 2 - 170:w // 2 + 170]
            entry["neon_share_full"] = float(mask_neon(img).mean())
            entry["spring_red_share_full"] = float(mask_red(img).mean())
            entry["neon_share_center"] = float(mask_neon(crop).mean())
            entry["spring_red_share_center"] = float(mask_red(crop).mean())

        # 1. The on-drone camera. The viewer keeps the camera it was created with, so close it
        #    afterwards to switch to the free camera (as the proven camera test does).
        draw("fpv_after_reset", {"camera": "fpv_cam:0"}, camera="fpv_cam:0")
        core.sim.close()

        # 2. The whole track, with the camera settings of level0.toml (as the proven film test).
        cam_cfg = {k: np.asarray(v) if k == "lookat" else v for k, v in cfg.sim.cam_config[0].items()}
        draw("track_wide", {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in cam_cfg.items()},
             camera=-1, cam_config=cam_cfg)

        # 3. True pose of gate 0 from the simulator (mj_kinematics ran inside sim.render).
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, GATE)
        assert bid >= 0, f"body {GATE} not found"
        gpos = np.array(d.xpos[bid], dtype=np.float64)
        gmat = np.array(d.xmat[bid], dtype=np.float64).reshape(3, 3)
        result["gate_pose"] = {"body": GATE, "xpos": gpos.tolist(), "xmat": gmat.tolist()}
        try:
            result["gate_pose"]["observed_gates_pos_0"] = np.asarray(obs["gates_pos"]).reshape(-1, 3)[0].tolist()
        except Exception:
            result["gate_pose"]["observed_gates_pos_0"] = None

        # 4. Checks that read the compiled model back.
        result["model_checks"] = model_checks(mujoco, np, m, d, gpos, gmat, n_gates)

        # 5. Free-camera views of gate 0. Free camera: forward = (cos el cos az, cos el sin az, sin el),
        #    camera position = lookat - distance * forward (visualize.c lines 450-473).
        cam = core.sim.viewer.viewer.cam  # the cam_config of sim.render only applies at viewer creation
        for label, (target_local, offset_local) in VIEWS.items():
            target = gpos + gmat @ np.array(target_local, dtype=np.float64)
            position = target + gmat @ np.array(offset_local, dtype=np.float64)
            forward = target - position
            distance = float(np.linalg.norm(forward))
            forward /= distance
            azimuth = math.degrees(math.atan2(forward[1], forward[0]))
            elevation = math.degrees(math.asin(forward[2]))
            cam.lookat[:] = target
            cam.distance = distance
            cam.azimuth = azimuth
            cam.elevation = elevation
            settings = {
                "lookat": target.tolist(), "distance": distance, "azimuth": azimuth, "elevation": elevation,
                "camera_position": position.tolist(), "target_in_gate_coordinates": list(target_local),
                "camera_offset_in_gate_coordinates": list(offset_local),
            }
            draw(label, settings, camera=-1)
            settings["cam_readback"] = {
                "lookat": np.array(cam.lookat).tolist(), "distance": float(cam.distance),
                "azimuth": float(cam.azimuth), "elevation": float(cam.elevation),
            }

        # 6. The pictures must differ from each other.
        labels = list(images)
        small = {k: v[::4, ::4].astype(np.float32) for k, v in images.items()}
        worst_pair, worst = None, None
        for i in range(len(labels)):
            for j in range(i + 1, len(labels)):
                diff = float(np.abs(small[labels[i]] - small[labels[j]]).mean())
                if worst is None or diff < worst:
                    worst, worst_pair = diff, (labels[i], labels[j])
        result["min_pairwise_mean_abs_diff"] = worst
        result["min_pairwise_pair"] = list(worst_pair) if worst_pair else None
    except Exception:
        result["traceback"] = traceback.format_exc()
    finally:
        try:
            if env is not None:
                env.close()
        except Exception:
            result["close_traceback"] = traceback.format_exc()
        (out / "child.json").write_text(json.dumps(result, indent=2))


# ---------------------------------------------------------------------------------------------
# Parent
# ---------------------------------------------------------------------------------------------


def run_one(method: str, args, out: Path) -> dict:
    cmd = [
        sys.executable, str(Path(__file__).resolve()),
        "--child", "--method", method, "--repo", str(Path(args.repo).resolve()),
        "--assets", str(Path(args.assets).resolve()), "--out", str(out), "--lsy-commit", args.lsy_commit,
    ]
    entry = {}
    (out / "child.json").unlink(missing_ok=True)  # no stale file from an earlier try
    try:
        proc = subprocess.run(
            cmd, env={**os.environ, "MUJOCO_GL": method}, capture_output=True, text=True, timeout=CHILD_TIMEOUT_S
        )
        (out / f"{method}_stdout.txt").write_text(proc.stdout)
        (out / f"{method}_stderr.txt").write_text(proc.stderr)
        entry["returncode"] = proc.returncode
        entry["stderr_tail"] = proc.stderr[-3000:]
    except subprocess.TimeoutExpired:
        entry["process_error"] = f"child timed out after {CHILD_TIMEOUT_S} s"
    child_file = out / "child.json"
    if child_file.is_file():
        entry.update(json.loads(child_file.read_text()))
    else:
        entry.setdefault("process_error", "child wrote no child json (crashed hard)")
    return entry


def write_summary(result: dict, path: Path) -> None:
    e = result["run"]
    verdict = "PASS" if result["passed"] else "FAIL"
    lines = ["### Gate look: still pictures of the new gate", "", f"**{verdict}**: {result['pass_reason']}", ""]
    ac = e.get("asset_check") or {}
    lines += [f"Asset check (the model LSY loads is the new one): {'ok' if ac.get('ok') else 'FAILED'}", ""]
    lines += ["| model check | result | detail |", "|---|---|---|"]
    for name, c in (e.get("model_checks") or {}).items():
        lines.append(f"| {name} | {'ok' if c['ok'] else 'FAIL'} | {str(c['detail'])[:300]} |")
    lines += ["", f"Camera: {e.get('camera_info')}", ""]
    lines += [
        "| picture | lookat | distance | azimuth | elevation | neon share (centre / full) | spring red share (centre / full) |",
        "|---|---|---|---|---|---|---|",
    ]
    for p in e.get("pictures", []):
        s = p.get("camera_settings", {})
        if "distance" in s:
            look = [round(v, 3) for v in s["lookat"]]
            cam = f"{look} | {s['distance']:.3f} | {s['azimuth']:.1f} | {s['elevation']:.1f}"
        else:
            cam = f"{s} | | |"
        lines.append(
            f"| {p['label']} | {cam} | {p.get('neon_share_center', 0):.4%} / {p.get('neon_share_full', 0):.4%} | "
            f"{p.get('spring_red_share_center', 0):.4%} / {p.get('spring_red_share_full', 0):.4%} |"
        )
    lines += [
        "",
        f"Smallest mean absolute difference between two pictures: {e.get('min_pairwise_mean_abs_diff')} "
        f"(pair {e.get('min_pairwise_pair')}).",
        f"Gate 0 pose: {e.get('gate_pose')}",
        "",
        "Versions: " + ", ".join(f"{k} {v}" for k, v in result["versions"].items()),
    ]
    path.write_text("\n".join(lines) + "\n")


def run_parent(args) -> None:
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    result = {"versions": {"python": platform.python_version()}, "attempts": [], "run": {}}
    for method in METHODS:
        entry = run_one(method, args, out)
        result["attempts"].append(
            {"method": method, "pictures": len(entry.get("pictures", [])),
             "error": (entry.get("traceback") or entry.get("process_error") or "").strip()[-500:]}
        )
        result["run"] = entry
        if len(entry.get("pictures", [])) >= len(PICTURES):
            break
    result["versions"].update(result["run"].get("versions", {}))
    result["passed"], result["pass_reason"] = evaluate(result, out)
    (out / "result.json").write_text(json.dumps(result, indent=2))
    write_summary(result, out / "summary.md")
    print(("PASS: " if result["passed"] else "FAIL: ") + result["pass_reason"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", help="Folder for all output files")
    parser.add_argument("--repo", help="Checkout of lsy_drone_racing (config, and the place of the new model)")
    parser.add_argument("--assets", help="Folder with the new model (assets/gates/square)")
    parser.add_argument("--lsy-commit", default="unknown", help="Commit of the checkout, only recorded")
    parser.add_argument("--child", action="store_true", help="Internal: do the work for one drawing method")
    parser.add_argument("--method", choices=METHODS, help="Internal: drawing method of the child")
    parser.add_argument("--check", help="Apply the pass rule to this result.json and exit 1 on failure")
    args = parser.parse_args()
    if args.check:
        sys.exit(check(Path(args.check)))
    if not args.out or not args.repo or not args.assets:
        parser.error("--out, --repo and --assets are required")
    if args.child:
        run_child(args.method, Path(args.repo).resolve(), Path(args.assets).resolve(), Path(args.out).resolve(), args.lsy_commit)
    else:
        run_parent(args)


if __name__ == "__main__":
    main()
