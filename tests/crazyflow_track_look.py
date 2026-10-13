"""Draw STILL pictures of the owner's track as the simulator builds it (no flight).

Inputs: a level file (made from the layout by tools/layout_to_level.py) and the layout JSON that it
was made from. The new gate model (assets/gates/square/) must be in LSY's asset folder before this
test runs; the workflow copies it there, and this test checks it first (same check as the gate look test).

The test makes the environment from the level file, resets it, and draws 640 x 480 pictures:
  - fpv_after_reset: the on-drone camera fpv_cam:0 at the start,
  - track_default_cam: the camera settings written in the level file ([[sim.cam_config]]),
  - side_from_px / side_from_nx / side_from_py / side_from_ny: the whole track from +x, -x, +y, -y,
    raised, looking at the center of the room,
  - top_down: straight down from above,
  - passage_KK_gateN_fwd|rev: for every passage of the route, 1.2 m before the gate on the side the
    drone comes from, at the height of the gate's center, looking through the gate.
It also reads the compiled model back and compares gates, poles, the start and the order of the
passages with the layout.

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
import re
import subprocess
import sys
import time
import traceback
from pathlib import Path

METHODS = ("egl", "osmesa")
WIDTH, HEIGHT = 640, 480
SEED = 0
CHILD_TIMEOUT_S = 1500
MODEL_FILES = (
    "gate.xml", "gate_top.png", "gate_bottom.png", "gate_left.png", "gate_right.png",
    "gate_post_bark.png", "spring.stl",
)
MIN_PAIRWISE_DIFF = 1.0  # smallest mean absolute difference (0..255) between two pictures with different cameras
POS_TOL = 1e-3  # gate and pole positions: 1 mm (the task); float32 storage is about 1e-7
AXIS_TOL = 1e-3  # x axis of a gate against (cos yaw, sin yaw)
# Start position: the level file written from the template has no [env.randomizations] section, so
# build_reset_fn (race_core.py lines 887-910) adds no drone randomization, and the only difference is
# float32 rounding (half a unit in the last place is about 1.2e-7 m for |x| between 1 and 2, 2.4e-7 below 4).
# 1e-4 m is 400 times that and still far below any randomization range of the other level files.
START_TOL = 1e-4
QUAT_TOL = 1e-6  # 1 - |dot| of the unit quaternions, which is about angle^2 / 8
RUN_UP_M = 1.2  # camera distance before the gate for the passage pictures
SIDE_DISTANCE_M = 8.0  # camera distance of the four side views
SIDE_ELEVATION_DEG = -30.0
ROOM_CENTER = (0.0, 0.0, 1.25)  # middle of the simulator box x, y in [-3, 3], z in [0, 2.5] (race_core.py 410-411)
TOP_HALF_EXTENT_M = 3.4  # the top view must show y from -3.4 to 3.4 m (the box plus a margin)
FIXED_PICTURES = (
    "fpv_after_reset", "track_default_cam", "side_from_px", "side_from_nx", "side_from_py", "side_from_ny",
    "top_down",
)


# ---------------------------------------------------------------------------------------------
# Layout (standard library only)
# ---------------------------------------------------------------------------------------------


def load_layout(path: Path) -> dict:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return {
        "gates": [{"pos": [float(v) for v in g["pos"]], "yaw": float(g["yaw_rad"])} for g in raw["gates"]],
        "poles": [{"pos": [float(v) for v in o["pos"]]} for o in raw["obstacles"]],
        "start": {"pos": [float(v) for v in raw["start"]["pos"]], "yaw": float(raw["start"]["yaw_rad"])},
        "order": [int(n) for n in raw["gate_order"]],
    }


def passage_label(k: int, n: int) -> str:
    return f"passage_{k:02d}_gate{abs(n)}_{'fwd' if n > 0 else 'rev'}"


def expected_pictures(layout: dict) -> list:
    return list(FIXED_PICTURES) + [passage_label(k, n) for k, n in enumerate(layout["order"], start=1)]


# ---------------------------------------------------------------------------------------------
# Pass rule (used by the parent and by --check); standard library only
# ---------------------------------------------------------------------------------------------


def evaluate(result: dict, base: Path) -> tuple:
    problems = []
    entry = result.get("run", {})
    if entry.get("traceback") or entry.get("process_error"):
        text = (entry.get("traceback") or entry.get("process_error")).strip().splitlines()
        problems.append("run failed: " + (text[-1][:200] if text else "no detail"))
    check = entry.get("asset_check") or {}
    if not check.get("ok"):
        problems.append("asset check failed or did not run")
    checks = entry.get("track_checks") or {}
    if not checks:
        problems.append("no track checks were made")
    for name, c in checks.items():
        if not c.get("ok"):
            problems.append(f"track check {name} failed: {str(c.get('detail'))[:300]}")
    wanted = result.get("expected_pictures") or []
    if not wanted:
        problems.append("result.json does not list the expected pictures")
    files = {p.get("label"): p for p in entry.get("pictures", [])}
    for label in wanted:
        p = files.get(label)
        if not p or not p.get("file") or not (base / p["file"]).is_file() or (base / p["file"]).stat().st_size == 0:
            problems.append(f"picture {label} missing")
    diff = entry.get("min_pairwise_mean_abs_diff")
    if diff is None or diff < MIN_PAIRWISE_DIFF:
        problems.append(f"pictures are too much alike (smallest mean abs difference {diff})")
    if problems:
        return False, "; ".join(problems)
    return True, (
        f"{len(wanted)} pictures drawn, {len(checks)} track checks passed; smallest difference between two "
        f"pictures with different cameras {diff:.2f}"
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
    """Same check as tests/crazyflow_gate_look.py: the model LSY loads must be the new one."""
    import lsy_drone_racing
    from lsy_drone_racing.envs.race_core import RaceCoreEnv

    package_dir = Path(lsy_drone_racing.__file__).resolve().parent
    detail = {"package_dir": str(package_dir), "gate_spec_path": str(RaceCoreEnv.gate_spec_path), "files": {}}
    if repo.resolve() not in package_dir.parents:
        raise AssertionError(f"imported lsy_drone_racing ({package_dir}) is not inside the checkout {repo.resolve()}")
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


def camera_frame(az_deg: float, el_deg: float) -> dict:
    """forward, up, right of a free camera: mujoco 3.10.0 engine_vis_visualize.c lines 452-473."""
    ca, sa = math.cos(math.radians(az_deg)), math.sin(math.radians(az_deg))
    ce, se = math.cos(math.radians(el_deg)), math.sin(math.radians(el_deg))
    return {
        "forward": [ce * ca, ce * sa, se],
        "up": [-se * ca, -se * sa, ce],
        "right": [sa, -ca, 0.0],
    }


def track_checks(mujoco, np, m, d, core, obs, layout: dict, cfg, result: dict) -> dict:
    """Compare the compiled model and the environment with the layout. Names carry the suffix ':<number>'."""
    from scipy.spatial.transform import Rotation as Rot

    out = {}

    def entry(name, ok, detail):
        out[name] = {"ok": bool(ok), "detail": detail}

    n_gates, n_poles = len(layout["gates"]), len(layout["poles"])
    body_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(m.nbody)]  # mujoco.h line 602
    gate_bodies = sorted(n for n in body_names if n and re.fullmatch(r"gate:\d+", n))
    pole_bodies = sorted(n for n in body_names if n and re.fullmatch(r"obstacle:\d+", n))
    entry("gate_body_count", len(gate_bodies) == n_gates, f"{len(gate_bodies)} gate bodies in the model, {n_gates} in the layout")
    entry("pole_body_count", len(pole_bodies) == n_poles, f"{len(pole_bodies)} pole bodies in the model, {n_poles} in the layout")
    cfg_counts = (len(cfg.env.track.gates), len(cfg.env.track.obstacles))
    entry("level_counts_equal_layout", cfg_counts == (n_gates, n_poles), f"level file: {cfg_counts}, layout: {(n_gates, n_poles)}")
    entry(
        "no_randomization_in_level",
        cfg.env.get("randomizations") is None and not cfg.env.track.randomize,
        f"env.randomizations = {cfg.env.get('randomizations')}, track.randomize = {cfg.env.track.randomize}",
    )

    def name_id(kind, name):
        return mujoco.mj_name2id(m, kind, name)  # mujoco.h line 599

    gate_rows, problems = [], []
    for i, g in enumerate(layout["gates"]):
        bid = name_id(mujoco.mjtObj.mjOBJ_BODY, f"gate:{i}")
        if bid < 0:
            problems.append(f"gate:{i} not found")
            continue
        xpos = np.array(d.xpos[bid], dtype=np.float64)
        xmat = np.array(d.xmat[bid], dtype=np.float64).reshape(3, 3)
        want_axis = np.array([math.cos(g["yaw"]), math.sin(g["yaw"]), 0.0])
        dpos = float(np.abs(xpos - np.array(g["pos"])).max())
        daxis = float(np.abs(xmat[:, 0] - want_axis).max())
        row = {
            "gate": i + 1, "layout_pos": g["pos"], "sim_pos": xpos.tolist(), "max_pos_error_m": dpos,
            "layout_yaw_deg": math.degrees(g["yaw"]), "sim_x_axis": xmat[:, 0].tolist(), "layout_x_axis": want_axis.tolist(),
            "max_axis_error": daxis, "sim_z_axis": xmat[:, 2].tolist(),
        }
        gate_rows.append(row)
        if dpos > POS_TOL:
            problems.append(f"gate {i + 1} position differs by {dpos:.2e} m")
        if daxis > AXIS_TOL:
            problems.append(f"gate {i + 1} x axis differs by {daxis:.2e}")
    result["gate_table"] = gate_rows
    entry("gate_poses_equal_layout", not problems and len(gate_rows) == n_gates,
          f"worst position error {max([r['max_pos_error_m'] for r in gate_rows] or [float('nan')]):.2e} m, "
          f"worst x-axis error {max([r['max_axis_error'] for r in gate_rows] or [float('nan')]):.2e}; problems: {problems}")

    pole_rows, problems = [], []
    for i, p in enumerate(layout["poles"]):
        bid = name_id(mujoco.mjtObj.mjOBJ_BODY, f"obstacle:{i}")
        if bid < 0:
            problems.append(f"obstacle:{i} not found")
            continue
        xpos = np.array(d.xpos[bid], dtype=np.float64)
        dpos = float(np.abs(xpos - np.array(p["pos"])).max())
        pole_rows.append({"pole": i + 1, "layout_pos": p["pos"], "sim_pos": xpos.tolist(), "max_pos_error_m": dpos})
        if dpos > POS_TOL:
            problems.append(f"pole {i + 1} position differs by {dpos:.2e} m")
    result["pole_table"] = pole_rows
    entry("pole_positions_equal_layout", not problems and len(pole_rows) == n_poles,
          f"worst position error {max([r['max_pos_error_m'] for r in pole_rows] or [float('nan')]):.2e} m; problems: {problems}")

    # What the controller will see: gates_pos in the observation (nominal values until a gate is in sensor range).
    obs_gates = np.asarray(obs["gates_pos"], dtype=np.float64).reshape(-1, 3)
    want = np.array([g["pos"] for g in layout["gates"]])
    err = float(np.abs(obs_gates - want).max()) if obs_gates.shape == want.shape else float("nan")
    entry("observed_gates_pos_equal_layout", obs_gates.shape == want.shape and err < 1e-4,
          f"observed shape {obs_gates.shape}, worst error {err:.2e} m (float32 values)")

    # Start: position after the reset, and direction.
    drone_pos = np.asarray(obs["pos"], dtype=np.float64).reshape(-1)
    drone_quat = np.asarray(obs["quat"], dtype=np.float64).reshape(-1)
    if drone_pos.size != 3 or drone_quat.size != 4:
        raise ValueError(f"expected one drone: pos {drone_pos.shape}, quat {drone_quat.shape}")
    dstart = float(np.abs(drone_pos - np.array(layout["start"]["pos"])).max())
    want_q = Rot.from_euler("xyz", [0.0, 0.0, layout["start"]["yaw"]]).as_quat()  # xyzw, as in utils.py load_track
    qdot = float(abs(np.dot(drone_quat, want_q)))
    result["start"] = {
        "layout_pos": layout["start"]["pos"], "sim_pos_after_reset": drone_pos.tolist(), "max_pos_error_m": dstart,
        "layout_yaw_deg": math.degrees(layout["start"]["yaw"]), "sim_quat_xyzw": drone_quat.tolist(),
        "layout_quat_xyzw": want_q.tolist(), "one_minus_abs_dot": 1.0 - qdot,
    }
    entry("drone_start_position", dstart <= START_TOL, f"difference {dstart:.2e} m, tolerance {START_TOL:.0e} m")
    entry("drone_start_direction", 1.0 - qdot <= QUAT_TOL, f"1 - |dot| = {1.0 - qdot:.2e}, tolerance {QUAT_TOL:.0e}")

    # Order of the passages that the environment holds (race_core.py EnvData.gate_sequence, lines 117-118).
    seq = np.asarray(core.data.gate_sequence).reshape(-1).tolist()
    dirs = np.asarray(core.data.gate_sequence_direction).reshape(-1).tolist()
    want_seq = [abs(n) - 1 for n in layout["order"]]
    want_dir = [1 if n > 0 else -1 for n in layout["order"]]
    obs_seq = np.asarray(obs["gate_sequence"]).reshape(-1).tolist()
    obs_dir = np.asarray(obs["gate_sequence_direction"]).reshape(-1).tolist()
    result["order"] = {"env_gate_sequence": seq, "env_direction": dirs, "obs_gate_sequence": obs_seq,
                       "obs_direction": obs_dir, "layout_gate_sequence": want_seq, "layout_direction": want_dir}
    entry("passage_order_equal_layout", seq == want_seq and dirs == want_dir and obs_seq == want_seq and obs_dir == want_dir,
          f"env {seq} {dirs}; observation {obs_seq} {obs_dir}; layout {want_seq} {want_dir}")
    return out


def run_child(method: str, args) -> None:
    repo, new_assets, out = Path(args.repo).resolve(), Path(args.assets).resolve(), Path(args.out).resolve()
    layout = load_layout(Path(args.layout))
    out.mkdir(parents=True, exist_ok=True)
    result = {
        "method": method,
        "MUJOCO_GL": os.environ.get("MUJOCO_GL"),
        "versions": {},
        "asset_check": None,
        "track_checks": {},
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
            "lsy_drone_racing_commit": args.lsy_commit,
        }
        result["asset_check"] = check_assets(repo, new_assets)

        cfg = load_config(Path(args.level).resolve())
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
        fovy = float(m.vis.global_.fovy)
        result["camera_info"] = {
            "fovy_deg": fovy,
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

        # 1. The on-drone camera. The viewer keeps the camera it was created with, so close it
        #    afterwards to switch to the free camera (as the proven gate look test does).
        draw("fpv_after_reset", {"camera": "fpv_cam:0"}, camera="fpv_cam:0")
        core.sim.close()

        # 2. The camera settings of the level file (as the proven film test and gate look test).
        cam_cfg = {k: np.asarray(v) if k == "lookat" else v for k, v in cfg.sim.cam_config[0].items()}
        draw("track_default_cam", {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in cam_cfg.items()},
             camera=-1, cam_config=cam_cfg)

        # 3. Checks that read the model and the environment back (mj_kinematics ran inside sim.render).
        result["track_checks"] = track_checks(mujoco, np, m, d, core, obs, layout, cfg, result)

        # 4. Free-camera views. Free camera: forward = (cos el cos az, cos el sin az, sin el), camera
        #    position = lookat - distance * forward (mujoco 3.10.0 engine_vis_visualize.c lines 452-473).
        #    The cam_config of sim.render only applies at viewer creation, so set the camera directly.
        cam = core.sim.viewer.viewer.cam
        seen = {}

        def view(label: str, lookat, distance: float, azimuth: float, elevation: float, note: str = "") -> None:
            lookat = [float(v) for v in lookat]
            cam.lookat[:] = lookat
            cam.distance = float(distance)
            cam.azimuth = float(azimuth)
            cam.elevation = float(elevation)
            frame = camera_frame(azimuth, elevation)
            position = [lookat[i] - distance * frame["forward"][i] for i in range(3)]
            settings = {"lookat": lookat, "distance": float(distance), "azimuth": float(azimuth),
                        "elevation": float(elevation), "camera_position": position, **frame, "note": note}
            key = tuple(round(v, 6) for v in lookat + [distance, azimuth, elevation])
            if key in seen:
                settings["same_camera_as"] = seen[key]
            else:
                seen[key] = label
            draw(label, settings, camera=-1)
            settings["cam_readback"] = {
                "lookat": np.array(cam.lookat).tolist(), "distance": float(cam.distance),
                "azimuth": float(cam.azimuth), "elevation": float(cam.elevation),
            }

        for label, az in (("side_from_px", 180.0), ("side_from_nx", 0.0), ("side_from_py", -90.0), ("side_from_ny", 90.0)):
            view(label, ROOM_CENTER, SIDE_DISTANCE_M, az, SIDE_ELEVATION_DEG,
                 f"camera on the {label[-2:]} side of the room, looking toward the room center")
        # Straight down. With elevation -90: forward = (0, 0, -1), up = (cos az, sin az, 0), right = (sin az, -cos az, 0)
        # (visualize.c lines 456-471), so azimuth 90 gives +x to the right and +y up in the picture.
        top_distance = TOP_HALF_EXTENT_M / math.tan(math.radians(fovy) / 2.0)
        view("top_down", (0.0, 0.0, 0.0), top_distance, 90.0, -90.0,
             "straight down; azimuth 90: +x is to the right and +y is up in the picture")

        gate_rows = {r["gate"]: r for r in result.get("gate_table", [])}
        for k, n in enumerate(layout["order"], start=1):
            row = gate_rows[abs(n)]
            center = np.array(row["sim_pos"])
            axis = np.array(row["sim_x_axis"])
            heading = (1.0 if n > 0 else -1.0) * axis  # crossing direction: +x of the gate for n > 0 (utils.py gate_passed)
            position = center - RUN_UP_M * heading
            forward = center - position
            forward /= np.linalg.norm(forward)
            azimuth = math.degrees(math.atan2(forward[1], forward[0]))
            elevation = math.degrees(math.asin(max(-1.0, min(1.0, float(forward[2])))))
            view(passage_label(k, n), center, RUN_UP_M, azimuth, elevation,
                 f"passage {k}: gate {abs(n)}, {'forward (-x to +x of the gate)' if n > 0 else 'reverse (+x to -x of the gate)'}; "
                 f"camera at {position.tolist()}")

        # 5. The pictures must differ from each other (cameras that are exactly equal are left out).
        labels = [lab for lab in images if "same_camera_as" not in next(p for p in result["pictures"] if p["label"] == lab)["camera_settings"]]
        small = {k: v[::4, ::4].astype(np.float32) for k, v in images.items()}
        worst_pair, worst = None, None
        for i in range(len(labels)):
            for j in range(i + 1, len(labels)):
                diff = float(np.abs(small[labels[i]] - small[labels[j]]).mean())
                if worst is None or diff < worst:
                    worst, worst_pair = diff, (labels[i], labels[j])
        result["min_pairwise_mean_abs_diff"] = worst
        result["min_pairwise_pair"] = list(worst_pair) if worst_pair else None
        result["pictures_compared"] = len(labels)
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
        "--level", str(Path(args.level).resolve()), "--layout", str(Path(args.layout).resolve()),
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


def fmt3(v) -> str:
    return "(" + ", ".join(f"{x:.3f}" for x in v) + ")"


def write_summary(result: dict, path: Path) -> None:
    e = result["run"]
    verdict = "PASS" if result["passed"] else "FAIL"
    lines = ["### Track look: still pictures of the owner's track", "", f"**{verdict}**: {result['pass_reason']}", ""]
    lines += [f"Level file: `{result.get('level_file')}`, layout: `{result.get('layout_file')}`", ""]
    ac = e.get("asset_check") or {}
    lines += [f"Asset check (the model LSY loads is the new one): {'ok' if ac.get('ok') else 'FAILED'}", ""]
    lines += ["| track check | result | detail |", "|---|---|---|"]
    for name, c in (e.get("track_checks") or {}).items():
        lines.append(f"| {name} | {'ok' if c['ok'] else 'FAIL'} | {str(c['detail'])[:300]} |")
    lines += ["", "Gates, layout against simulator (positions in m, after the reset):", "",
              "| gate | layout position | simulator position | position error (mm) | layout yaw (deg) | layout x axis | simulator x axis | axis error |",
              "|---|---|---|---|---|---|---|---|"]
    for r in e.get("gate_table", []):
        lines.append(
            f"| {r['gate']} | {fmt3(r['layout_pos'])} | {fmt3(r['sim_pos'])} | {r['max_pos_error_m'] * 1000:.4f} | "
            f"{r['layout_yaw_deg']:.2f} | {fmt3(r['layout_x_axis'])} | {fmt3(r['sim_x_axis'])} | {r['max_axis_error']:.1e} |"
        )
    lines += ["", "Poles (z = top):", "", "| pole | layout position | simulator position | position error (mm) |", "|---|---|---|---|"]
    for r in e.get("pole_table", []):
        lines.append(f"| {r['pole']} | {fmt3(r['layout_pos'])} | {fmt3(r['sim_pos'])} | {r['max_pos_error_m'] * 1000:.4f} |")
    s = e.get("start")
    if s:
        lines += ["", f"Start: layout {fmt3(s['layout_pos'])}, simulator after reset {fmt3(s['sim_pos_after_reset'])}, "
                      f"difference {s['max_pos_error_m']:.2e} m (tolerance {START_TOL:.0e} m); layout yaw {s['layout_yaw_deg']:.2f} deg, "
                      f"quaternion 1 - |dot| = {s['one_minus_abs_dot']:.1e}."]
    o = e.get("order")
    if o:
        lines += ["", f"Passage order (0-based gate ids, direction): environment {o['env_gate_sequence']} {o['env_direction']}, "
                      f"layout {o['layout_gate_sequence']} {o['layout_direction']}."]
    lines += ["", f"Camera: {e.get('camera_info')}", ""]
    lines += [
        "| picture | lookat | distance | azimuth | elevation | camera position | note |",
        "|---|---|---|---|---|---|---|",
    ]
    for p in e.get("pictures", []):
        cs = p.get("camera_settings", {})
        if "distance" in cs:
            note = cs.get("note", "") + (f" (same camera as {cs['same_camera_as']})" if "same_camera_as" in cs else "")
            lines.append(
                f"| {p['label']} | {fmt3(cs['lookat'])} | {cs['distance']:.3f} | {cs['azimuth']:.1f} | {cs['elevation']:.1f} | "
                f"{fmt3(cs['camera_position'])} | {note} |"
            )
        else:
            lines.append(f"| {p['label']} | {cs} | | | | | |")
    lines += [
        "",
        f"Smallest mean absolute difference between two pictures with different cameras: "
        f"{e.get('min_pairwise_mean_abs_diff')} (pair {e.get('min_pairwise_pair')}; {e.get('pictures_compared')} pictures compared).",
        "",
        "Versions: " + ", ".join(f"{k} {v}" for k, v in result["versions"].items()),
    ]
    path.write_text("\n".join(lines) + "\n")


def run_parent(args) -> None:
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    layout = load_layout(Path(args.layout))
    wanted = expected_pictures(layout)
    result = {
        "versions": {"python": platform.python_version()}, "attempts": [], "run": {},
        "expected_pictures": wanted, "level_file": str(args.level), "layout_file": str(args.layout),
    }
    for method in METHODS:
        entry = run_one(method, args, out)
        result["attempts"].append(
            {"method": method, "pictures": len(entry.get("pictures", [])),
             "error": (entry.get("traceback") or entry.get("process_error") or "").strip()[-500:]}
        )
        result["run"] = entry
        if len(entry.get("pictures", [])) >= len(wanted):
            break
    result["versions"].update(result["run"].get("versions", {}))
    result["passed"], result["pass_reason"] = evaluate(result, out)
    (out / "result.json").write_text(json.dumps(result, indent=2))
    write_summary(result, out / "summary.md")
    print(("PASS: " if result["passed"] else "FAIL: ") + result["pass_reason"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", help="Folder for all output files")
    parser.add_argument("--repo", help="Checkout of lsy_drone_racing (and the place of the new model)")
    parser.add_argument("--assets", help="Folder with the new model (assets/gates/square)")
    parser.add_argument("--level", help="Level file (TOML) made by tools/layout_to_level.py")
    parser.add_argument("--layout", help="Layout JSON that the level file was made from")
    parser.add_argument("--lsy-commit", default="unknown", help="Commit of the checkout, only recorded")
    parser.add_argument("--child", action="store_true", help="Internal: do the work for one drawing method")
    parser.add_argument("--method", choices=METHODS, help="Internal: drawing method of the child")
    parser.add_argument("--check", help="Apply the pass rule to this result.json and exit 1 on failure")
    args = parser.parse_args()
    if args.check:
        sys.exit(check(Path(args.check)))
    if not (args.out and args.repo and args.assets and args.level and args.layout):
        parser.error("--out, --repo, --assets, --level and --layout are required")
    if args.child:
        run_child(args.method, args)
    else:
        run_parent(args)


if __name__ == "__main__":
    main()
