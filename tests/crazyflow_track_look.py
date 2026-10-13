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

New in this version:
  - The drone is hi-vis orange (255, 95, 0). The test changes the VISUAL properties of the compiled MuJoCo model
    that the renderer draws from (sim.mj_model): one material and the material ids of five geoms. No Crazyflow
    file is changed and the physics does not read these fields (see make_drone_orange).
  - Flat floor pads (start pad at the layout's start, end pad at the layout's "end" if it has one) are added by
    tools/track_env.py, a subclass of LSY's DroneRaceEnv that attaches them while LSY loads the track.
  - Extra pictures: drone_pad_start_side_0p4m, drone_pad_start_1p5m, drone_pad_start_top_1p0m and, if the layout
    has an end, pad_end_top_1p0m and pad_end_side_1p5m, plus checks of the model and of the pixels.

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
START_PAD_PICTURES = ("drone_pad_start_side_0p4m", "drone_pad_start_1p5m", "drone_pad_start_top_1p0m")
END_PAD_PICTURES = ("pad_end_top_1p0m", "pad_end_side_1p5m")
REPO_DIR = Path(__file__).resolve().parent.parent
TOOLS_DIR = REPO_DIR / "tools"
PADS_DIR = REPO_DIR / "assets" / "pads"

# Hi-vis orange drone. Meshes of the drone body whose geoms become orange (crazyflow drones/cf21B_500.xml lines
# 41-60 name them): the board (the body), the motors, the prop guards (the arms and rings) and the battery
# holder, plus the battery on top. The propellers (separate bodies prop0..3, green), the connectors, the gold
# connector pins and the LEDs keep their look.
ORANGE_RGB = (255, 95, 0)
ORANGE_EMISSION = 0.5  # lit color = rgba * (emission + lights); the headlight ambient 0.5 (scene.xml) adds the rest, so the
#                        darkest side shows (255, 95, 0) and lit sides get brighter (the red channel clips, green rises)
ORANGE_SPECULAR = 0.1
ORANGE_MESHES = ("pcb", "motors", "prop-guards", "battery-holder", "battery")
ORANGE_MATERIAL_PREFIX = "gray"  # the board (pcb) uses the material "gray" (suffixed ":0" in the compiled model)
MAT_TOL = 1e-6  # float32 storage of 95/255 etc.
MIN_ORANGE_SHARE = 0.001  # share of pixels of the close picture that must be close to orange (fail below this, also at zero)
MIN_PAD_PIXEL_SHARE = 0.005  # share of pixels of a straight-down pad picture for each color that must be there
PAD_RADIUS_M = 0.3  # 0.6 m across
PAD_HALF_HEIGHT_M = 0.0005  # 1 mm high
PAD_TOL = 1e-6
HEIGHT_TOL = 1e-3  # drone height after reset against the layout's start height: 1 mm (the task)


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
        # optional, written by the track designer: {"pos": [x, y, z]}; None when the layout has no end point
        "end": {"pos": [float(v) for v in raw["end"]["pos"]]} if raw.get("end") else None,
    }


def passage_label(k: int, n: int) -> str:
    return f"passage_{k:02d}_gate{abs(n)}_{'fwd' if n > 0 else 'rev'}"


def expected_pictures(layout: dict) -> list:
    pads = list(START_PAD_PICTURES) + (list(END_PAD_PICTURES) if layout.get("end") else [])
    return list(FIXED_PICTURES) + [passage_label(k, n) for k, n in enumerate(layout["order"], start=1)] + pads


def expected_checks(layout: dict) -> list:
    """Names of the track checks that must be present in the result (a check that did not run is a failure)."""
    names = [
        "env_is_lsy_drone_race_env", "gate_body_count", "pole_body_count", "level_counts_equal_layout",
        "no_randomization_in_level", "gate_poses_equal_layout", "pole_positions_equal_layout",
        "observed_gates_pos_equal_layout", "drone_start_position", "drone_start_direction",
        "passage_order_equal_layout", "drone_orange_in_model", "drone_orange_still_in_model_at_end",
        "pad_bodies_equal_layout", "pad_geoms_flat_no_collision", "drone_height_with_pads_equals_layout_start",
        "orange_pixels_in_close_picture", "start_pad_yellow_pixels",
    ]
    if layout.get("end"):
        names.append("end_pad_black_and_white_pixels")
    return names


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
    for name in result.get("expected_checks") or []:
        if name not in checks:
            problems.append(f"track check {name} did not run")
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


def physics_snapshot(np, m) -> str:
    """Hash of the model fields that the physics and the collision use; it must not change when we color the drone."""
    h = hashlib.sha256()
    for name in ("geom_type", "geom_contype", "geom_conaffinity", "geom_size", "geom_pos", "geom_quat", "geom_friction",
                 "geom_condim", "geom_bodyid", "body_mass", "body_inertia", "body_pos", "body_quat", "body_mocapid",
                 "body_weldid", "body_parentid"):
        h.update(name.encode() + np.ascontiguousarray(getattr(m, name)).tobytes())
    h.update(np.float64(m.opt.timestep).tobytes())
    return h.hexdigest()


def drone_geoms(mujoco, np, m) -> dict:
    """The mesh geoms of the body drone:0 itself (not of the propeller bodies below it), by mesh name."""
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "drone:0")
    if bid < 0:
        raise ValueError("body drone:0 not found in the compiled model")
    first, count = int(m.body_geomadr[bid]), int(m.body_geomnum[bid])
    out = {}
    for g in range(first, first + count):
        if int(m.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_MESH):
            continue
        mesh = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, int(m.geom_dataid[g]))
        out[re.sub(r":\d+$", "", mesh or "")] = g
    return out


def make_drone_orange(mujoco, np, m) -> dict:
    """Make the drone hi-vis orange by changing VISUAL fields of the compiled model `m` (= sim.mj_model).

    Why this is the model the renderer draws from: crazyflow/sim/sim.py line 248-253 builds the MujocoRenderer
    from self.mj_model, and Gymnasium's renderer (mujoco_rendering.py) keeps that very object (self.model) and
    calls mjv_updateScene with it for every frame; mjv_updateScene -> setMaterial (engine_vis_visualize.c lines
    225-246, called for geoms at line 929) copies rgba, emission, specular, shininess and reflectance from the
    material of the geom, every frame.
    What could overwrite it: Sim.build_mjx (sim.py lines 504-509) recompiles self.mj_model from the spec. It is
    called only in RaceCoreEnv._load_track_into_sim (race_core.py line 701) and use_box_collision (sim.py line 788),
    both inside the environment constructor, i.e. before this function runs. env.reset() (race_core.py lines
    503-533, Sim.reset sim.py lines 178-186) only replaces JAX data, and Sim.close (lines 282-285) closes the viewer
    and keeps the model. The checks below read the model back at the start and again at the end.
    Physics: MJX got its own copy of the model when it was built (sim.py line 353, put_model); the step functions
    use Crazyflow's JAX dynamics, and MJX only does kinematics, camlight and collision (sim.py lines 711-724),
    which read geometry and contact fields, not colors. We also hash the physics fields before and after.
    What changes: the material of the board (gray) becomes orange with emission and low specular, and the geoms of
    the other chosen meshes get that material id and the same geom rgba.
    """
    before_physics = physics_snapshot(np, m)
    mats_before = np.array(m.mat_rgba, copy=True)
    geoms = drone_geoms(mujoco, np, m)
    missing = [n for n in ORANGE_MESHES if n not in geoms]
    if missing:
        raise ValueError(f"mesh geoms {missing} not found on drone:0; found {sorted(geoms)}")

    def mat_name(mid):
        return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MATERIAL, int(mid)) if int(mid) >= 0 else None

    old = {n: {"geom_id": geoms[n], "material_before": mat_name(m.geom_matid[geoms[n]])} for n in ORANGE_MESHES}
    orange_mat = int(m.geom_matid[geoms["pcb"]])
    if orange_mat < 0 or not (mat_name(orange_mat) or "").startswith(ORANGE_MATERIAL_PREFIX):
        raise ValueError(f"the board material is {mat_name(orange_mat)!r}, expected a name starting with {ORANGE_MATERIAL_PREFIX!r}")
    others = [g for n, g in geoms.items() if n not in ORANGE_MESHES and int(m.geom_matid[g]) == orange_mat]
    if others:
        raise ValueError(f"other geoms {others} already use the board material; they would turn orange too")
    rgba = np.array([c / 255.0 for c in ORANGE_RGB] + [1.0], dtype=np.float32)
    m.mat_rgba[orange_mat] = rgba
    m.mat_emission[orange_mat] = ORANGE_EMISSION
    m.mat_specular[orange_mat] = ORANGE_SPECULAR
    for n in ORANGE_MESHES:
        g = geoms[n]
        m.geom_matid[g] = orange_mat
        m.geom_rgba[g] = rgba
        old[n]["material_after"] = mat_name(m.geom_matid[g])
    after_physics = physics_snapshot(np, m)
    other_rows = [i for i in range(m.nmat) if i != orange_mat]
    return {
        "material_changed": mat_name(orange_mat), "material_id": orange_mat, "target_rgba": rgba.tolist(),
        "emission": ORANGE_EMISSION, "specular": ORANGE_SPECULAR,
        "geoms_made_orange": {n: old[n] for n in ORANGE_MESHES},
        "geoms_kept": sorted(n for n in geoms if n not in ORANGE_MESHES) + ["propellers (bodies prop0..prop3, material green)"],
        "other_materials_unchanged": bool(np.array_equal(mats_before[other_rows], np.array(m.mat_rgba)[other_rows])),
        "physics_hash_before": before_physics, "physics_hash_after": after_physics,
        "physics_unchanged": before_physics == after_physics,
    }


def orange_readback(mujoco, np, m, info: dict) -> tuple:
    """Read the colors back from the model. Returns (ok, detail)."""
    problems = []
    mid = info["material_id"]
    want = np.array(info["target_rgba"], dtype=np.float32)
    got = np.array(m.mat_rgba[mid], dtype=np.float32)
    if float(np.abs(got - want).max()) > MAT_TOL:
        problems.append(f"material rgba {got.tolist()} differs from {want.tolist()}")
    if abs(float(m.mat_emission[mid]) - ORANGE_EMISSION) > MAT_TOL:
        problems.append(f"material emission {float(m.mat_emission[mid])}")
    bad = []
    geoms = drone_geoms(mujoco, np, m)
    for n in ORANGE_MESHES:
        g = geoms.get(n)
        if g is None or int(m.geom_matid[g]) != mid or float(np.abs(np.array(m.geom_rgba[g]) - want).max()) > MAT_TOL:
            bad.append(n)
    if bad:
        problems.append(f"geoms not orange: {bad}")
    kept = [n for n in geoms if n not in ORANGE_MESHES and int(m.geom_matid[geoms[n]]) == mid]
    if kept:
        problems.append(f"geoms that must keep their look use the orange material: {kept}")
    rgb255 = [round(float(c) * 255.0, 3) for c in got[:3]]
    return (not problems and bool(info.get("physics_unchanged")) and bool(info.get("other_materials_unchanged")),
            f"material {info['material_changed']} rgb x255 = {rgb255}, emission {float(m.mat_emission[mid])}; "
            f"orange geoms {list(ORANGE_MESHES)}; physics hash unchanged: {info.get('physics_unchanged')}; "
            f"other materials unchanged: {info.get('other_materials_unchanged')}; problems: {problems}")


def pad_checks(mujoco, np, m, d, layout: dict, result: dict, entry) -> None:
    """The pad bodies are in the compiled model at the layout's positions, flat, 0.6 m across, without collision."""
    body_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(m.nbody)]
    found = sorted(n for n in body_names if n and re.fullmatch(r"pad_(start|end):\d+", n))
    want = {"pad_start:0": layout["start"]["pos"][:2]}
    if layout.get("end"):
        want["pad_end:0"] = layout["end"]["pos"][:2]
    rows, problems = [], []
    if found != sorted(want):
        problems.append(f"pad bodies in the model {found}, expected {sorted(want)}")
    for name, xy in want.items():
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            problems.append(f"{name} not found")
            continue
        xpos = np.array(d.xpos[bid], dtype=np.float64)
        dxy = float(np.abs(xpos[:2] - np.array(xy)).max())
        row = {"pad": name, "layout_xy": list(xy), "sim_pos": xpos.tolist(), "max_xy_error_m": dxy, "floor_z_error_m": abs(float(xpos[2])),
               "mocap": int(m.body_mocapid[bid]) >= 0, "joints": int(m.body_dofnum[bid]), "geoms": []}
        if dxy > POS_TOL or abs(float(xpos[2])) > PAD_TOL:
            problems.append(f"{name} at {xpos.tolist()}, layout xy {list(xy)}, floor z 0")
        if row["mocap"] or row["joints"]:
            problems.append(f"{name} is not a static body (mocap {row['mocap']}, dofs {row['joints']})")
        first, count = int(m.body_geomadr[bid]), int(m.body_geomnum[bid])
        if count < 1:
            problems.append(f"{name} has no geom")
        for g in range(first, first + count):
            size = np.array(m.geom_size[g], dtype=np.float64)
            top = float(d.geom_xpos[g][2]) + float(size[1])
            g_row = {
                "geom": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g), "type": int(m.geom_type[g]),
                "contype": int(m.geom_contype[g]), "conaffinity": int(m.geom_conaffinity[g]),
                "radius_m": float(size[0]), "height_m": 2.0 * float(size[1]), "top_z_m": top,
                "material": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MATERIAL, int(m.geom_matid[g])) if int(m.geom_matid[g]) >= 0 else None,
            }
            row["geoms"].append(g_row)
            if g_row["contype"] != 0 or g_row["conaffinity"] != 0:
                problems.append(f"{name} geom {g_row['geom']} can collide (contype {g_row['contype']}, conaffinity {g_row['conaffinity']})")
            if g_row["type"] != int(mujoco.mjtGeom.mjGEOM_CYLINDER):
                problems.append(f"{name} geom {g_row['geom']} is not a cylinder")
            if abs(g_row["radius_m"] - PAD_RADIUS_M) > PAD_TOL or abs(size[1] - PAD_HALF_HEIGHT_M) > PAD_TOL or abs(top - 2 * PAD_HALF_HEIGHT_M) > PAD_TOL:
                problems.append(f"{name} geom {g_row['geom']}: radius {g_row['radius_m']}, height {g_row['height_m']}, top {top}")
            if g_row["material"] is None:
                problems.append(f"{name} geom {g_row['geom']} has no material (no picture)")
        rows.append(row)
    result["pad_table"] = rows
    entry("pad_bodies_equal_layout", found == sorted(want) and all(abs(r["max_xy_error_m"]) <= POS_TOL and r["floor_z_error_m"] <= PAD_TOL for r in rows) and len(rows) == len(want),
          f"pads in model {found}; layout xy {want}; worst xy error {max([r['max_xy_error_m'] for r in rows] or [float('nan')]):.2e} m"
          + ("" if layout.get("end") else "; the layout has no end point, so there is no end pad"))
    entry("pad_geoms_flat_no_collision", not problems and bool(rows), f"{len(rows)} pad(s); problems: {problems}")


def share_masks(np, img) -> dict:
    """Boolean masks of the colors we look for in a picture (uint8, HxWx3)."""
    a = img.astype(np.int32)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    return {
        "orange": (r >= 200) & (g >= 50) & (g <= 222) & (b <= 80) & (g * 100 <= r * 88),  # (255, 95, 0) up to a lit (255, 218, 0); yellow starts at G 225
        "orange_strict": (abs(r - ORANGE_RGB[0]) <= 30) & (abs(g - ORANGE_RGB[1]) <= 30) & (b <= 30),
        "neon_yellow": (r >= 205) & (g >= 225) & (b <= 60),
        "black": a.max(axis=-1) <= 25,
        "white": a.min(axis=-1) >= 235,
    }


def pixel_checks(np, images: dict, layout: dict, result: dict, entry) -> None:
    """Checks on the pictures: orange drone, yellow ring of the start pad, black and white of the end pad."""
    shares = {}
    for label in list(START_PAD_PICTURES) + list(END_PAD_PICTURES):
        if label in images:
            masks = share_masks(np, images[label])
            shares[label] = {k: float(v.mean()) for k, v in masks.items()}
            shares[label]["orange_pixels"] = int(masks["orange"].sum())
    result["pixel_shares"] = shares
    close = shares.get("drone_pad_start_side_0p4m")
    entry("orange_pixels_in_close_picture",
          close is not None and close["orange_pixels"] > 0 and close["orange"] >= MIN_ORANGE_SHARE,
          f"drone_pad_start_side_0p4m: {None if close is None else close['orange_pixels']} pixels close to orange "
          f"({None if close is None else round(100 * close['orange'], 3)} %, strict band "
          f"{None if close is None else round(100 * close['orange_strict'], 3)} %); needs at least {100 * MIN_ORANGE_SHARE:.2f} % and not zero")
    top = shares.get("drone_pad_start_top_1p0m")
    entry("start_pad_yellow_pixels", top is not None and top["neon_yellow"] >= MIN_PAD_PIXEL_SHARE,
          f"drone_pad_start_top_1p0m: {None if top is None else round(100 * top['neon_yellow'], 3)} % of the pixels are neon yellow "
          f"(R >= 205, G >= 225, B <= 60); needs at least {100 * MIN_PAD_PIXEL_SHARE:.2f} %")
    if layout.get("end"):
        e = shares.get("pad_end_top_1p0m")
        entry("end_pad_black_and_white_pixels",
              e is not None and e["black"] >= MIN_PAD_PIXEL_SHARE and e["white"] >= MIN_PAD_PIXEL_SHARE,
              f"pad_end_top_1p0m: black (max channel <= 25) {None if e is None else round(100 * e['black'], 3)} %, white (min channel >= 235) "
              f"{None if e is None else round(100 * e['white'], 3)} %; each needs at least {100 * MIN_PAD_PIXEL_SHARE:.2f} %")


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

    # Pads: the compiled model against the layout, and the drone height with the pads in the model. The drone
    # starts at z = 0.01 m (the layout) and the pad is only 1 mm high and has no collision, so nothing may change.
    pad_checks(mujoco, np, m, d, layout, result, entry)
    dz = abs(float(drone_pos[2]) - layout["start"]["pos"][2])
    result["start"]["height_after_reset_m"] = float(drone_pos[2])
    result["start"]["height_error_m"] = dz
    entry("drone_height_with_pads_equals_layout_start", dz <= HEIGHT_TOL,
          f"drone height after reset {float(drone_pos[2]):.6f} m, layout start height {layout['start']['pos'][2]} m, "
          f"difference {dz:.2e} m, tolerance {HEIGHT_TOL:.0e} m")
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
        # The environment: LSY's DroneRaceEnv plus the pads (tools/track_env.py, see its docstring for how the
        # registration and the wrappers stay the same as for "DroneRacing-v0"). Same make arguments as before.
        sys.path.insert(0, str(TOOLS_DIR))
        import track_env
        from lsy_drone_racing.envs.drone_race import DroneRaceEnv

        if cfg.env.id != track_env.BASE_ENV_ID:
            raise AssertionError(f"the level file names the environment {cfg.env.id!r}, expected {track_env.BASE_ENV_ID!r}")
        pads = {"pad_start": tuple(layout["start"]["pos"][:2])}
        if layout.get("end"):
            pads["pad_end"] = tuple(layout["end"]["pos"][:2])
        result["pads_requested"] = {k: list(v) for k, v in pads.items()}
        result["pads_dir"] = str(Path(args.pads).resolve())
        env = gymnasium.make(
            track_env.register_pad_env(),
            freq=cfg.env.freq,
            sim_config=cfg.sim,
            sensor_range=cfg.env.sensor_range,
            control_mode=cfg.env.control_mode,
            track=cfg.env.track,
            disturbances=cfg.env.get("disturbances"),
            randomizations=cfg.env.get("randomizations"),
            seed=SEED,
            pads=pads,
            pads_dir=str(Path(args.pads).resolve()),
        )
        core = env.unwrapped
        chain, e = [], env
        while isinstance(e, gymnasium.Wrapper):
            chain.append(type(e).__name__)
            e = e.env
        chain.append(type(e).__name__)
        result["env_kind"] = {"wrapper_chain": chain, "unwrapped_class": type(core).__name__,
                              "is_DroneRaceEnv": isinstance(core, DroneRaceEnv)}
        obs, _ = env.reset()
        m, d = core.sim.mj_model, core.sim.mj_data
        # The orange drone: visual fields of the model that the renderer will draw from. Done before the first
        # render, which creates the viewer; read back again at the end.
        orange_info = make_drone_orange(mujoco, np, m)
        result["orange"] = orange_info
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
        kind = result["env_kind"]
        result["track_checks"]["env_is_lsy_drone_race_env"] = {
            "ok": bool(kind["is_DroneRaceEnv"]) and "OrderEnforcing" in kind["wrapper_chain"] and "TimeLimit" in kind["wrapper_chain"],
            "detail": f"unwrapped class {kind['unwrapped_class']} (subclass of lsy DroneRaceEnv: {kind['is_DroneRaceEnv']}); wrappers {kind['wrapper_chain']}",
        }
        ok, detail = orange_readback(mujoco, np, m, orange_info)
        result["track_checks"]["drone_orange_in_model"] = {"ok": ok, "detail": detail}

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

        # 4b. The drone on the start pad and the pads from above. The viewer cameras look along +y (azimuth 90),
        #     so a viewer who stands at the pad and looks along +y sees the H upright (assets/pads/pad_start.xml).
        #     The 1.5 m views look down at 50 degrees so that the line of sight stays clear of gate 3 of draft_01.
        sx, sy, sz = layout["start"]["pos"]
        view("drone_pad_start_side_0p4m", (sx, sy, sz + 0.02), 0.4, 90.0, -20.0,
             "0.4 m from the drone on the start pad, from the -y side and a little above (looking along +y)")
        view("drone_pad_start_1p5m", (sx, sy, 0.05), 1.5, 90.0, -50.0,
             "1.5 m from the drone on the start pad, from the -y side, 50 degrees down (looking along +y)")
        view("drone_pad_start_top_1p0m", (sx, sy, 0.0), 1.0, 90.0, -90.0,
             "straight down at 1.0 m on the start pad; azimuth 90: +x is to the right and +y is up in the picture")
        if layout.get("end"):
            ex, ey, _ = layout["end"]["pos"]
            view("pad_end_top_1p0m", (ex, ey, 0.0), 1.0, 90.0, -90.0,
                 "straight down at 1.0 m on the end pad; azimuth 90: +x is to the right and +y is up in the picture")
            view("pad_end_side_1p5m", (ex, ey, 0.05), 1.5, 90.0, -50.0,
                 "1.5 m from the end pad, from the -y side, 50 degrees down (looking along +y)")
        pixel_entries = {}

        def pixel_entry(name, ok, detail):
            pixel_entries[name] = {"ok": bool(ok), "detail": detail}

        pixel_checks(np, images, layout, result, pixel_entry)
        result["track_checks"].update(pixel_entries)
        # the orange must still be in the model after all the rendering (nothing rebuilt it)
        ok, detail = orange_readback(mujoco, np, core.sim.mj_model, orange_info)
        result["track_checks"]["drone_orange_still_in_model_at_end"] = {
            "ok": ok and core.sim.mj_model is m, "detail": detail + f"; same model object as at the start: {core.sim.mj_model is m}",
        }

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
        "--pads", str(Path(args.pads).resolve()),
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
    """Three numbers as text; anything that is not a list of numbers becomes '?' (the summary must never stop)."""
    try:
        return "(" + ", ".join(f"{float(x):.3f}" for x in v) + ")"
    except (TypeError, ValueError):
        return "?"


def num(v, spec: str = ".3f", scale: float = 1.0) -> str:
    """One number formatted with `spec` (after multiplying by `scale`); '?' when it is missing or not a number."""
    try:
        return format(float(v) * scale, spec)
    except (TypeError, ValueError):
        return "?"


def write_summary(result: dict, path: Path) -> None:
    """Write summary.md. Every field is read with dict.get, so a missing field shows '?' and never stops the run."""
    e = result.get("run") or {}
    verdict = "PASS" if result.get("passed") else "FAIL"
    lines = ["### Track look: still pictures of the owner's track, orange drone and floor pads", "",
             f"**{verdict}**: {result.get('pass_reason', 'no pass reason was recorded')}", ""]
    lines += [f"Level file: `{result.get('level_file')}`, layout: `{result.get('layout_file')}`", ""]
    ac = e.get("asset_check") or {}
    lines += [f"Asset check (the model LSY loads is the new one): {'ok' if ac.get('ok') else 'FAILED'}", ""]
    kind = e.get("env_kind") or {}
    lines += [f"Environment: unwrapped class `{kind.get('unwrapped_class')}`, wrappers {kind.get('wrapper_chain')}, "
              f"pads asked for {e.get('pads_requested')} (pad files from `{e.get('pads_dir')}`).", ""]
    lines += ["| track check | result | detail |", "|---|---|---|"]
    for name, c in (e.get("track_checks") or {}).items():
        c = c if isinstance(c, dict) else {}
        lines.append(f"| {name} | {'ok' if c.get('ok') else 'FAIL'} | {str(c.get('detail'))[:400]} |")
    missing = [n for n in (result.get("expected_checks") or []) if n not in (e.get("track_checks") or {})]
    if missing:
        lines += ["", f"Checks that did not run: {', '.join(missing)}"]
    lines += ["", "Gates, layout against simulator (positions in m, after the reset):", "",
              "| gate | layout position | simulator position | position error (mm) | layout yaw (deg) | layout x axis | simulator x axis | axis error |",
              "|---|---|---|---|---|---|---|---|"]
    for r in e.get("gate_table") or []:
        lines.append(
            f"| {r.get('gate')} | {fmt3(r.get('layout_pos'))} | {fmt3(r.get('sim_pos'))} | {num(r.get('max_pos_error_m'), '.4f', 1000)} | "
            f"{num(r.get('layout_yaw_deg'), '.2f')} | {fmt3(r.get('layout_x_axis'))} | {fmt3(r.get('sim_x_axis'))} | {num(r.get('max_axis_error'), '.1e')} |"
        )
    lines += ["", "Poles (z = top):", "", "| pole | layout position | simulator position | position error (mm) |", "|---|---|---|---|"]
    for r in e.get("pole_table") or []:
        lines.append(f"| {r.get('pole')} | {fmt3(r.get('layout_pos'))} | {fmt3(r.get('sim_pos'))} | {num(r.get('max_pos_error_m'), '.4f', 1000)} |")
    s = e.get("start")
    if s:
        lines += ["", f"Start: layout {fmt3(s.get('layout_pos'))}, simulator after reset {fmt3(s.get('sim_pos_after_reset'))}, "
                      f"difference {num(s.get('max_pos_error_m'), '.2e')} m (tolerance {START_TOL:.0e} m); layout yaw {num(s.get('layout_yaw_deg'), '.2f')} deg, "
                      f"quaternion 1 - |dot| = {num(s.get('one_minus_abs_dot'), '.1e')}; drone height after reset {num(s.get('height_after_reset_m'), '.6f')} m "
                      f"(difference from the layout {num(s.get('height_error_m'), '.2e')} m, tolerance {HEIGHT_TOL:.0e} m)."]
    o = e.get("order")
    if o:
        lines += ["", f"Passage order (0-based gate ids, direction): environment {o.get('env_gate_sequence')} {o.get('env_direction')}, "
                      f"layout {o.get('layout_gate_sequence')} {o.get('layout_direction')}."]
    pads = e.get("pad_table")
    requested = e.get("pads_requested") or {}
    lines += ["", "Floor pads (flat, 0.6 m across, 1 mm high, no collision; local +y is the top of the picture, so the H is upright for a viewer who looks along world +y):", ""]
    if pads:
        lines += ["| pad | layout xy | simulator position | xy error (mm) | static | geoms (type, contype, conaffinity, radius, height, top z) |", "|---|---|---|---|---|---|"]
        for r in pads:
            geoms = "; ".join(
                f"{g.get('geom')}: type {g.get('type')}, {g.get('contype')}, {g.get('conaffinity')}, {num(g.get('radius_m'), '.4f')} m, "
                f"{num(g.get('height_m'), '.4f')} m, {num(g.get('top_z_m'), '.4f')} m"
                for g in (r.get("geoms") or [])
            )
            lines.append(f"| {r.get('pad')} | {r.get('layout_xy')} | {fmt3(r.get('sim_pos'))} | {num(r.get('max_xy_error_m'), '.4f', 1000)} | "
                         f"{'yes' if not r.get('mocap') and not r.get('joints') else 'NO'} | {geoms} |")
    else:
        lines.append("No pad table was written (the run stopped before the pad checks).")
    if requested and "pad_end" not in requested:
        lines += ["", "The layout has no end point, so there is no end pad in this run."]
    ob = e.get("orange")
    if ob:
        made = ", ".join(f"{k} (geom {(v or {}).get('geom_id')}: {(v or {}).get('material_before')} -> {(v or {}).get('material_after')})"
                         for k, v in (ob.get("geoms_made_orange") or {}).items())
        lines += ["", f"Orange drone: material `{ob.get('material_changed')}` set to rgba {ob.get('target_rgba')} with emission {ob.get('emission')} "
                      f"and specular {ob.get('specular')}; geoms made orange: {made}; kept: {ob.get('geoms_kept')}; "
                      f"physics fields unchanged: {ob.get('physics_unchanged')}; other materials unchanged: {ob.get('other_materials_unchanged')}."]
    px = e.get("pixel_shares") or {}
    if px:
        lines += ["", "Pixel shares of the new pictures (fraction of all pixels; orange_pixels is a count):", "",
                  "| picture | orange | orange (strict) | neon yellow | pure black | pure white | orange pixels |", "|---|---|---|---|---|---|---|"]
        for label, sh in px.items():
            sh = sh if isinstance(sh, dict) else {}
            lines.append(f"| {label} | {num(sh.get('orange'), '.5f')} | {num(sh.get('orange_strict'), '.5f')} | {num(sh.get('neon_yellow'), '.5f')} | "
                         f"{num(sh.get('black'), '.5f')} | {num(sh.get('white'), '.5f')} | {sh.get('orange_pixels')} |")
    lines += ["", f"Camera: {e.get('camera_info')}", ""]
    lines += [
        "| picture | lookat | distance | azimuth | elevation | camera position | note |",
        "|---|---|---|---|---|---|---|",
    ]
    for p in e.get("pictures") or []:
        cs = p.get("camera_settings") or {}
        if "distance" in cs:
            note = str(cs.get("note", "")) + (f" (same camera as {cs['same_camera_as']})" if "same_camera_as" in cs else "")
            lines.append(
                f"| {p.get('label')} | {fmt3(cs.get('lookat'))} | {num(cs.get('distance'))} | {num(cs.get('azimuth'), '.1f')} | "
                f"{num(cs.get('elevation'), '.1f')} | {fmt3(cs.get('camera_position')) if 'camera_position' in cs else ''} | {note} |"
            )
        else:
            lines.append(f"| {p.get('label')} | {cs} | | | | | |")
    wanted, drawn = result.get("expected_pictures") or [], {p.get("label") for p in e.get("pictures") or []}
    not_drawn = [x for x in wanted if x not in drawn]
    lines += ["", f"Pictures expected {len(wanted)}, drawn {len(drawn)}" + (f"; missing: {', '.join(not_drawn)}" if not_drawn else "") + ".",
              "",
              f"Smallest mean absolute difference between two pictures with different cameras: "
              f"{e.get('min_pairwise_mean_abs_diff')} (pair {e.get('min_pairwise_pair')}; {e.get('pictures_compared')} pictures compared).",
              "",
              "Versions: " + ", ".join(f"{k} {v}" for k, v in (result.get("versions") or {}).items())]
    if e.get("traceback"):
        lines += ["", "Traceback of the run:", "", "```", str(e["traceback"])[-2500:], "```"]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_parent(args) -> None:
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    layout = load_layout(Path(args.layout))
    wanted = expected_pictures(layout)
    result = {
        "versions": {"python": platform.python_version()}, "attempts": [], "run": {},
        "expected_pictures": wanted, "expected_checks": expected_checks(layout), "level_file": str(args.level), "layout_file": str(args.layout),
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
    parser.add_argument("--pads", default=str(PADS_DIR), help="Folder with the pad models (default: assets/pads of this repository)")
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
