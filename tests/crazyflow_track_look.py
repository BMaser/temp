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

New in this version (drone drawing scale): `--drone-scales 1.0,1.5,2.0` (the default; '' switches it off). In the same run, for each
scale, the vertices of the drone's visual meshes (and the positions of those mesh geoms and of the four propeller bodies) in the
compiled model are multiplied by the scale about the drone's body origin BEFORE a fresh viewer is made, and four pictures are
drawn: scale_<s>_side.png, _top.png, _gate.png (the drone drawn 0.6 m before gate 1) and _fpv.png (camera fpv_cam:0); then one
sheet drone_scale_comparison.png. Only the drawing changes: see the block comment above run_scale_mode for the source lines, and
the checks drone_scale_* (physics hash, collision, mass, camera, position after a reset, orange share growth).

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

# Drawing scale of the drone (see set_drone_scale). The scales are an argument; an empty argument switches the mode off.
DEFAULT_DRONE_SCALES = "1.0,1.5,2.0"
SCALE_VIEWS = ("side", "top", "gate", "fpv")  # columns of the comparison sheet, in this order
SHEET_FILE = "drone_scale_comparison.png"
GATE_DRONE_BEFORE_M = 0.6  # the drone is drawn 0.6 m before the first gate of the route, on the entry side
GATE_CAMERA_BEHIND_DRONE_M = 0.8  # the camera is 0.8 m behind the drone, so the gate is 1.4 m from the camera
AREA_GROWTH_FRACTION = 0.625  # the orange share must grow at least 0.625 * (s_max / s_min)^2: 2.5 times for 1.0 -> 2.0
SCALE_READBACK_TOL = 1e-4  # drawn width at scale s over drawn width at the smallest scale, against s / s_min (float32 storage is 1e-7)
SCALE_PLACE_TOL = 1e-6  # drone position in the drawn data against the position asked for (float32 mocap storage is about 1e-7)
MASSLESS_TOL = 1e-9  # kg; the propeller bodies carry no mass, so moving them cannot change the drone's inertia
THUMB_W, THUMB_H = 320, 240  # one picture of the comparison sheet
SHEET_LABEL_W = 170


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


def parse_scales(text) -> list:
    """'1.0,1.5,2.0' -> [1.0, 1.5, 2.0] (ascending). An empty text gives an empty list, which switches the mode off."""
    scales = [float(t) for t in (text or "").split(",") if t.strip()]
    if any(not (math.isfinite(s) and s > 0.0) for s in scales):
        raise ValueError(f"drone scales must be positive numbers, got {scales}")
    if len(set(scales)) != len(scales):
        raise ValueError(f"drone scales must differ from each other, got {scales}")
    return sorted(scales)


def scale_tag(s: float) -> str:
    """1.0 -> '1p0', 1.5 -> '1p5', 1.25 -> '1p25' (used in file names)."""
    t = f"{s:g}"
    if "." not in t:
        t += ".0"
    return t.replace(".", "p")


def scale_file_name(s: float, view: str) -> str:
    return f"scale_{scale_tag(s)}_{view}.png"


def scale_files(scales: list) -> list:
    """Files that the scale mode must write (the pass rule checks that they exist)."""
    if not scales:
        return []
    return [scale_file_name(s, v) for s in scales for v in SCALE_VIEWS] + [SHEET_FILE]


def scale_check_names(scales: list) -> list:
    if not scales:
        return []
    names = [
        "drone_scale_drawn_size_equals_scale", "drone_scale_physics_hash_unchanged",
        "drone_scale_collision_mass_camera_unchanged", "drone_scale_moved_bodies_massless",
        "drone_scale_position_after_reset_unchanged", "drone_scale_gate_picture_drone_placed",
        "drone_scale_model_restored",
    ]
    if len(scales) >= 2:
        names.append("drone_scale_top_orange_share_grows")
    return names


def expected_checks(layout: dict, scales=None) -> list:
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
    return names + scale_check_names(scales or [])


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
    for name in result.get("expected_files") or []:  # files of the drone scale mode (pictures and the comparison sheet)
        f = base / name
        if not f.is_file() or f.stat().st_size == 0:
            problems.append(f"file {name} missing")
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


def physics_snapshot(np, m, blank=None) -> str:
    """Hash of the model fields that the physics and the collision use; it must not change when we color the drone.

    `blank` = {"geom_pos": [geom ids], "body_pos": [body ids]} sets those ROWS to zero in the copy that is hashed.
    The drawing scale moves exactly these rows (the positions of the drawn mesh geoms and of the four propeller
    bodies); every other number of every field above, and every other row of these two fields, is still hashed.
    """
    h = hashlib.sha256()
    for name in ("geom_type", "geom_contype", "geom_conaffinity", "geom_size", "geom_pos", "geom_quat", "geom_friction",
                 "geom_condim", "geom_bodyid", "body_mass", "body_inertia", "body_pos", "body_quat", "body_mocapid",
                 "body_weldid", "body_parentid"):
        arr = np.array(getattr(m, name), copy=True)
        if blank and name in blank and len(blank[name]):
            arr[np.asarray(sorted(blank[name]), dtype=np.int64)] = 0
        h.update(name.encode() + np.ascontiguousarray(arr).tobytes())
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


# ---------------------------------------------------------------------------------------------
# Drawing scale of the drone
# ---------------------------------------------------------------------------------------------
#
# Why this changes only the picture (sources under runs/2026-10-13_drone_model_sources and the ones before it):
#   - The drawn drone is mesh geoms of class "visual" (cf21B_500.xml lines 39-41, 55-63, 76-87): contype 0 and
#     conaffinity 0, so they are not in any collision. The collision is a separate geom (col_sphere, line 67, or col_box,
#     line 69; LSY switches to the box, see drone_parts). Flight physics is Crazyflow's own parameter file, not the geometry.
#   - mjr_makeContext -> makeMesh -> mjr_uploadMesh (render_context.c lines 198-205, 211-326) copies m->mesh_vert into an
#     OpenGL display list ONCE, when the context is made. The context is made in Gymnasium's BaseRender.__init__
#     (mujoco_rendering.py line 75), which Sim.render calls the first time it has no viewer (sim.py lines 237-253) and
#     again after Sim.close (sim.py lines 282-285). So the vertices must be changed while sim.viewer is None.
#   - The renderer draws a mesh geom by glCallList(baseMesh + dataid) (render_gl3.c lines 363-367) with no scaling by
#     geom_size, at the pose geom_xpos / geom_xmat that mj_kinematics computes from geom_pos, geom_quat, body_pos
#     and body_quat (engine_core_smooth.c lines 84-93, 214-215). MuJoCo has re-centered every mesh at compile time:
#     the vertices are moved by -CoM and rotated into the principal axes, and that pose is added to the geom's
#     pos and quat (user_mesh.cc lines 1453-1526, user_objects.cc lines 4046-4053). So a vertex is at
#         p_body = R(geom_quat) * v + geom_pos            (geom of drone:0)
#         p_body = R(body_quat) * (R(geom_quat) * v + geom_pos) + body_pos     (geom of a propeller body)
#     and a scale s about the body origin of drone:0 needs v, geom_pos and the propeller body_pos all times s.
#     The quaternions stay. Normals are unit vectors and do not change under a uniform scale.
#   - mesh_vert is a float32 array (nmeshvert, 3) of mjModel (mjxmacro.h line 533) that the Python bindings expose as a
#     NumPy array over the model's own memory (structs.cc lines 397-402, same list that already let this test write
#     mat_rgba and geom_matid).
#   - Bounding volumes: the renderer has no frustum culling of geoms (the only use of geom_rbound in the draw path is the
#     sort distance of transparent geoms, render_gl3.c lines 905-912 and visualize.c line 924), so geom_rbound of the scaled
#     geoms is scaled too and geom_aabb (read only by the box-drawing option and by collision) is left alone.
#   - The LED geoms have alpha 0 and are skipped by mjv_updateScene (visualize.c lines 962-964); they are scaled anyway.
#   - MJX got its own copy of the model when it was built (sim.py lines 351-354), so it never sees these edits.


def quat_to_rot(Rot, q):
    """MuJoCo quaternion (w, x, y, z) -> scipy rotation (which wants x, y, z, w)."""
    q = [float(v) for v in q]
    return Rot.from_quat([q[1], q[2], q[3], q[0]])


def mesh_points_in_body(np, Rot, verts, geom_pos, geom_quat, body_pos=None, body_quat=None):
    """Vertices of a mesh geom in the frame of drone:0 (see the block comment above)."""
    p = quat_to_rot(Rot, geom_quat).apply(np.asarray(verts, dtype=np.float64)) + np.asarray(geom_pos, dtype=np.float64)
    if body_pos is not None:
        p = quat_to_rot(Rot, body_quat).apply(p) + np.asarray(body_pos, dtype=np.float64)
    return p


def extent_of_points(np, point_sets) -> dict:
    p = np.concatenate(list(point_sets), axis=0)
    lo, hi = p.min(axis=0), p.max(axis=0)
    return {
        "x_min_m": float(lo[0]), "x_max_m": float(hi[0]), "y_min_m": float(lo[1]), "y_max_m": float(hi[1]),
        "z_min_m": float(lo[2]), "z_max_m": float(hi[2]),
        "width_x_m": float(hi[0] - lo[0]), "width_y_m": float(hi[1] - lo[1]),
        "drawn_width_m": float(max(hi[0] - lo[0], hi[1] - lo[1])),  # the largest horizontal extent along x or y
        "max_radius_xy_m": float(np.sqrt(p[:, 0] ** 2 + p[:, 1] ** 2).max()),  # farthest point from the body origin, seen from above
        "n_vertices": int(p.shape[0]),
    }


def drone_parts(mujoco, np, m) -> dict:
    """Ids of what the drawing scale changes, and copies of the original numbers (the scale is always set from these)."""
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "drone:0")
    if bid < 0:
        raise ValueError("body drone:0 not found in the compiled model")
    mesh_type = int(mujoco.mjtGeom.mjGEOM_MESH)
    type_names = {int(v): n for n, v in mujoco.mjtGeom.__members__.items()}
    props = [b for b in range(m.nbody) if b != bid and int(m.body_parentid[b]) == bid
             and re.fullmatch(r"prop\d+.*", mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "")]
    if len(props) != 4:
        raise ValueError(f"expected 4 propeller bodies below drone:0, found {len(props)}")

    def geoms_of(b):
        return list(range(int(m.body_geomadr[b]), int(m.body_geomadr[b]) + int(m.body_geomnum[b])))

    scaled = [g for g in geoms_of(bid) if int(m.geom_type[g]) == mesh_type] + [g for b in props for g in geoms_of(b)]
    # Collision geoms: every geom of the drone's bodies that can collide, whatever its type. Which one that is depends on the
    # run: LSY's environment calls use_box_collision(self.sim, True) after it builds the simulator (race_core.py line 395), which
    # sets contype and conaffinity of "col_sphere*" to 0 and those of "col_box*" to 1 (crazyflow sim.py lines 778-788, then
    # build_mjx recompiles the model). So in the compiled model of a running LSY environment the box collides, not the sphere.
    # frozen_geoms = the non-mesh geoms of drone:0 (sphere and box, active or not): none of them may change.
    frozen_geoms = [g for g in geoms_of(bid) if int(m.geom_type[g]) != mesh_type]
    collision = [g for b in [bid] + props for g in geoms_of(b) if int(m.geom_contype[g]) != 0 or int(m.geom_conaffinity[g]) != 0]
    for g in scaled:
        if int(m.geom_type[g]) != mesh_type or int(m.geom_contype[g]) != 0 or int(m.geom_conaffinity[g]) != 0:
            raise ValueError(f"geom {g} would be scaled but is not a visual mesh geom without collision")
    meshes = sorted({int(m.geom_dataid[g]) for g in scaled})
    scaled_set = set(scaled)
    for g in range(int(m.ngeom)):  # a mesh that is shared with a geom outside the set cannot be scaled on its own
        if int(m.geom_type[g]) == mesh_type and int(m.geom_dataid[g]) in meshes and g not in scaled_set:
            raise ValueError(f"mesh {int(m.geom_dataid[g])} is also used by geom {g}, which is not part of the drawn drone")
    if m.mesh_vert.ndim != 2 or m.mesh_vert.shape[1] != 3:
        raise ValueError(f"mesh_vert has shape {m.mesh_vert.shape}, expected (nmeshvert, 3)")
    if not collision:
        raise ValueError("no geom of drone:0 or of its propeller bodies has contype or conaffinity different from 0")
    collision_info = [{
        "geom": g, "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}", "type": type_names.get(int(m.geom_type[g]), str(int(m.geom_type[g]))),
        "size_m": [float(v) for v in m.geom_size[g]], "pos_m": [float(v) for v in m.geom_pos[g]],
        "contype": int(m.geom_contype[g]), "conaffinity": int(m.geom_conaffinity[g]),
    } for g in collision]
    ranges = {k: (int(m.mesh_vertadr[k]), int(m.mesh_vertnum[k])) for k in meshes}
    return {
        "body": bid, "prop_bodies": props, "scaled_geoms": scaled, "collision_geoms": collision, "frozen_geoms": frozen_geoms, "collision_info": collision_info,
        "meshes": meshes, "mesh_ranges": ranges,
        "geom_names": [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}" for g in scaled],
        "mesh_names": [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, k) for k in meshes],
        "orig": {
            "mesh_vert": {k: np.array(m.mesh_vert[a:a + n], copy=True) for k, (a, n) in ranges.items()},
            "geom_pos": {g: np.array(m.geom_pos[g], copy=True) for g in scaled},
            "geom_rbound": {g: float(m.geom_rbound[g]) for g in scaled},
            "body_pos": {b: np.array(m.body_pos[b], copy=True) for b in props},
        },
    }


def set_drone_scale(np, m, parts: dict, s: float) -> None:
    """Draw the drone s times larger about the origin of drone:0. Call it only while there is no viewer (see above)."""
    o = parts["orig"]
    for k, (a, n) in parts["mesh_ranges"].items():
        m.mesh_vert[a:a + n] = (o["mesh_vert"][k].astype(np.float64) * s).astype(np.float32)
    for g in parts["scaled_geoms"]:
        m.geom_pos[g] = o["geom_pos"][g] * s
        m.geom_rbound[g] = o["geom_rbound"][g] * s
    for b in parts["prop_bodies"]:
        m.body_pos[b] = o["body_pos"][b] * s


def drawn_extent(mujoco, np, Rot, m, parts: dict) -> dict:
    """Extent of the drawn drone in the frame of drone:0, measured from the vertex data that is in the model NOW."""
    sets = []
    for g in parts["scaled_geoms"]:
        mat = int(m.geom_matid[g])
        alpha = float(m.mat_rgba[mat][3]) if mat >= 0 else float(m.geom_rgba[g][3])
        if alpha == 0.0:  # not drawn (the LED diffusers)
            continue
        a, n = parts["mesh_ranges"][int(m.geom_dataid[g])]
        b = int(m.geom_bodyid[g])
        if b == parts["body"]:
            sets.append(mesh_points_in_body(np, Rot, m.mesh_vert[a:a + n], m.geom_pos[g], m.geom_quat[g]))
        else:
            sets.append(mesh_points_in_body(np, Rot, m.mesh_vert[a:a + n], m.geom_pos[g], m.geom_quat[g], m.body_pos[b], m.body_quat[b]))
    return extent_of_points(np, sets)


def frozen_arrays(np, m, parts: dict) -> dict:
    """Copies of the numbers that must not change: collision geoms, masses, inertia, cameras."""
    g = parts["frozen_geoms"] + [x for x in parts["collision_geoms"] if x not in parts["frozen_geoms"]]
    out = {name: np.array(getattr(m, name)[g], copy=True)
           for name in ("geom_type", "geom_size", "geom_pos", "geom_quat", "geom_contype", "geom_conaffinity", "geom_friction", "geom_condim")}
    for name in ("body_mass", "body_inertia", "cam_pos", "cam_quat", "cam_fovy"):
        out[name] = np.array(getattr(m, name), copy=True)
    return out


def changed_arrays(np, m, parts: dict, frozen: dict) -> list:
    now = frozen_arrays(np, m, parts)
    return [name for name in frozen if not np.array_equal(frozen[name], now[name])]


def describe_collision(info: list) -> str:
    """Plain words for the collision geoms in use: shape and size (box and sphere sizes in full length, in m)."""
    parts_ = []
    for c in info:
        sz = c["size_m"]
        if c["type"].endswith("BOX"):
            what = f"box {2 * sz[0]:.3f} x {2 * sz[1]:.3f} x {2 * sz[2]:.3f} m (half sizes {sz[0]:.3f}, {sz[1]:.3f}, {sz[2]:.3f})"
        elif c["type"].endswith("SPHERE"):
            what = f"sphere of radius {sz[0]:.3f} m ({2 * sz[0]:.3f} m across)"
        else:
            what = f"{c['type']} with size {[round(v, 4) for v in sz]} m"
        parts_.append(f"{c['name']}: {what}, contype {c['contype']}, conaffinity {c['conaffinity']}")
    return "; ".join(parts_)


def make_sheet(Image, ImageDraw, directory: Path, scales: list, widths=None) -> Path:
    """One sheet: one row for each scale (written at the left), one column for each view."""
    resample = getattr(Image, "Resampling", Image)
    heads = {"side": "side, 0.4 m", "top": "from above, 1.0 m", "gate": "before gate 1", "fpv": "drone camera"}
    head_h = 34
    sheet = Image.new("RGB", (SHEET_LABEL_W + len(SCALE_VIEWS) * THUMB_W, head_h + len(scales) * THUMB_H), (255, 255, 255))

    def text_tile(text, w, h):  # the default font is small, so draw it at half size and enlarge it
        small = Image.new("RGB", (max(1, w // 2), max(1, h // 2)), (255, 255, 255))
        ImageDraw.Draw(small).text((3, 3), text, fill=(0, 0, 0))
        return small.resize((w, h), resample.NEAREST)

    for j, view in enumerate(SCALE_VIEWS):
        sheet.paste(text_tile(heads[view], THUMB_W, head_h), (SHEET_LABEL_W + j * THUMB_W, 0))
    for i, s in enumerate(scales):
        y = head_h + i * THUMB_H
        label = f"scale {s:g}"
        if widths and widths.get(s) is not None:
            label += f"\nwidth {1000 * widths[s]:.0f} mm"
        sheet.paste(text_tile(label, SHEET_LABEL_W, THUMB_H), (0, y))
        for j, view in enumerate(SCALE_VIEWS):
            with Image.open(directory / scale_file_name(s, view)) as im:
                sheet.paste(im.convert("RGB").resize((THUMB_W, THUMB_H), resample.LANCZOS), (SHEET_LABEL_W + j * THUMB_W, y))
    path = directory / SHEET_FILE
    sheet.save(path)
    return path


def run_scale_mode(mujoco, np, iio, jnp, core, env, m, layout: dict, result: dict, scales: list, out: Path, sync, images: dict, obs0) -> dict:
    """Draw the drone at each scale, make the pictures and the sheet, and return the checks (name -> {ok, detail})."""
    from PIL import Image, ImageDraw
    from scipy.spatial.transform import Rotation as Rot

    checks = {}

    def entry(name, ok, detail):
        checks[name] = {"ok": bool(ok), "detail": detail}

    parts = drone_parts(mujoco, np, m)
    bid = parts["body"]
    mocap_id = int(m.body_mocapid[bid])
    if mocap_id < 0:
        raise ValueError("drone:0 is not a mocap body (sim.py sync_sim2mjx places the drone through mocap_pos)")
    blank = {"geom_pos": parts["scaled_geoms"], "body_pos": parts["prop_bodies"]}
    full_before, blank_before = physics_snapshot(np, m), physics_snapshot(np, m, blank)
    frozen = frozen_arrays(np, m, parts)
    obs0_pos = np.asarray(obs0["pos"], dtype=np.float64).reshape(-1)
    obs0_quat = np.asarray(obs0["quat"], dtype=np.float64).reshape(-1)
    sx, sy, sz = layout["start"]["pos"]
    collision_info = parts["collision_info"]
    collision_text = describe_collision(collision_info)
    mass = float(m.body_mass[bid])
    prop_mass = [float(m.body_mass[b]) for b in parts["prop_bodies"]]

    # Where the drone is drawn for the picture before gate 1: the first passage of the route, 0.6 m before the gate on the
    # entry side, at the height of the gate's center. The camera stands 0.8 m behind the drone on the same line.
    n0 = layout["order"][0]
    grow = {r["gate"]: r for r in result["gate_table"]}[abs(n0)]
    center = np.array(grow["sim_pos"], dtype=np.float64)
    heading = (1.0 if n0 > 0 else -1.0) * np.array(grow["sim_x_axis"], dtype=np.float64)
    heading[2] = 0.0
    heading /= np.linalg.norm(heading)
    gate_drone_pos = center - GATE_DRONE_BEFORE_M * heading
    gate_drone_pos[2] = center[2]
    yaw = math.atan2(heading[1], heading[0])
    gate_drone_quat = np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])  # w, x, y, z (MuJoCo order)
    gate_view = {"lookat": center.tolist(), "distance": GATE_DRONE_BEFORE_M + GATE_CAMERA_BEHIND_DRONE_M,
                 "azimuth": math.degrees(yaw), "elevation": 0.0}

    def shot(camera, cam_config=None, place=None):
        """One picture. `place` = (position, quaternion w x y z) puts the drone there in the data that the renderer copies:
        Sim.render copies mjx_data.mocap_pos / mocap_quat into mj_data (sim.py lines 264-266) and runs mj_kinematics, and the
        drone is a mocap body (sim.py sync_sim2mjx, lines 711-724), so we swap sim.mjx_data for one with another mocap
        pose for this call and put the original object back afterward. core.data and the physics are not touched."""
        sync()
        saved = core.sim.mjx_data
        if place is not None:
            pos, quat = place
            core.sim.mjx_data = saved.replace(
                mocap_pos=saved.mocap_pos.at[:, mocap_id, :].set(jnp.asarray(pos, dtype=saved.mocap_pos.dtype)),
                mocap_quat=saved.mocap_quat.at[:, mocap_id, :].set(jnp.asarray(quat, dtype=saved.mocap_quat.dtype)),
            )
        try:
            img = core.sim.render(mode="rgb_array", width=WIDTH, height=HEIGHT, camera=camera, cam_config=cam_config)
            drawn_at = np.array(core.sim.mj_data.xpos[bid], dtype=np.float64)
        finally:
            core.sim.mjx_data = saved
        img = np.ascontiguousarray(img)
        if img.shape != (HEIGHT, WIDTH, 3) or img.dtype != np.uint8:
            raise ValueError(f"unexpected picture {img.shape} {img.dtype}")
        return img, drawn_at

    def aim(lookat, distance, azimuth, elevation):
        cam = core.sim.viewer.viewer.cam
        cam.lookat[:] = [float(v) for v in lookat]
        cam.distance, cam.azimuth, cam.elevation = float(distance), float(azimuth), float(elevation)

    def free_cfg(lookat, distance, azimuth, elevation):
        return {"lookat": np.asarray([float(v) for v in lookat]), "distance": float(distance),
                "azimuth": float(azimuth), "elevation": float(elevation)}

    rows, hash_bad, changed_bad, reset_bad, place_bad, size_bad = [], [], [], [], [], []
    top_views = {
        "side": ((sx, sy, sz + 0.02), 0.4, 90.0, -20.0),  # as drone_pad_start_side_0p4m
        "top": ((sx, sy, 0.0), 1.0, 90.0, -90.0),  # as drone_pad_start_top_1p0m
    }
    base_width = None
    images_scale = {}
    for s in scales:
        core.sim.close()  # no viewer while the vertices change: the display lists are made when the viewer is made
        if core.sim.viewer is not None:
            raise AssertionError("a viewer is still open; the new mesh would not be uploaded")
        set_drone_scale(np, m, parts, s)
        ext = drawn_extent(mujoco, np, Rot, m, parts)
        if base_width is None:
            base_width = (s, ext["drawn_width_m"])
        want_ratio = s / base_width[0]
        got_ratio = ext["drawn_width_m"] / base_width[1]
        if abs(got_ratio / want_ratio - 1.0) > SCALE_READBACK_TOL:
            size_bad.append(f"scale {s:g}: width ratio {got_ratio:.6f}, wanted {want_ratio:.6f}")
        if physics_snapshot(np, m, blank) != blank_before:
            hash_bad.append(s)
        changed = changed_arrays(np, m, parts, frozen)
        if changed:
            changed_bad.append(f"scale {s:g}: {changed}")
        # The drone's position after a reset, with the scale set (the reset only replaces JAX data; the model keeps the scale).
        obs, _ = env.reset()
        pos = np.asarray(obs["pos"], dtype=np.float64).reshape(-1)
        quat = np.asarray(obs["quat"], dtype=np.float64).reshape(-1)
        dpos = float(np.abs(pos - obs0_pos).max())
        dq = 1.0 - float(abs(np.dot(quat, obs0_quat)))
        if dpos > START_TOL or dq > QUAT_TOL:
            reset_bad.append(f"scale {s:g}: position differs by {dpos:.2e} m, 1 - |dot| = {dq:.2e}")
        if physics_snapshot(np, m, blank) != blank_before:
            hash_bad.append(s)
        sync()
        mocap_after_reset = np.array(core.sim.mjx_data.mocap_pos, copy=True)

        files, shares = {}, {}

        def keep(view, img):
            name = scale_file_name(s, view)
            iio.imwrite(out / name, img)
            files[view] = name
            shares[view] = float(share_masks(np, img)["orange"].mean())
            images_scale[(s, view)] = img

        # 1. the drone's own camera
        img, _ = shot("fpv_cam:0")
        keep("fpv", img)
        core.sim.close()
        # 2. side view, then 3. from above, then 4. before the gate (the first free-camera shot makes the viewer)
        img, _ = shot(-1, cam_config=free_cfg(*top_views["side"]))
        keep("side", img)
        aim(*top_views["top"])
        img, _ = shot(-1)
        keep("top", img)
        aim(gate_view["lookat"], gate_view["distance"], gate_view["azimuth"], gate_view["elevation"])
        img, drawn_at = shot(-1, place=(gate_drone_pos, gate_drone_quat))
        keep("gate", img)
        perr = float(np.abs(drawn_at - gate_drone_pos).max())
        if perr > SCALE_PLACE_TOL:
            place_bad.append(f"scale {s:g}: drawn at {drawn_at.tolist()}, wanted {gate_drone_pos.tolist()}")
        if not np.array_equal(np.array(core.sim.mjx_data.mocap_pos), mocap_after_reset):
            place_bad.append(f"scale {s:g}: mocap_pos of the simulation was not put back")
        core.sim.close()
        rows.append({
            "scale": s, **ext, "lowest_point_above_floor_at_start_m": float(sz + ext["z_min_m"]),
            "reset_position_error_m": dpos, "reset_quat_one_minus_abs_dot": dq, "gate_picture_position_error_m": perr,
            "orange_share": shares, "files": files,
        })

    # Back to the original drawing, and prove it.
    core.sim.close()
    set_drone_scale(np, m, parts, 1.0)
    o = parts["orig"]
    same = all(np.array_equal(m.mesh_vert[a:a + n], o["mesh_vert"][k]) for k, (a, n) in parts["mesh_ranges"].items())
    same = same and all(np.array_equal(m.geom_pos[g], o["geom_pos"][g]) for g in parts["scaled_geoms"])
    same = same and all(np.array_equal(m.body_pos[b], o["body_pos"][b]) for b in parts["prop_bodies"])
    same = same and physics_snapshot(np, m) == full_before
    entry("drone_scale_model_restored", same,
          f"after the last scale the model was set back to 1.0: vertices, geom_pos, propeller body_pos and the full physics hash equal the original: {same}")

    smin = scales[0]
    base = rows[0]
    for r in rows:
        r["top_share_ratio_to_smallest"] = (r["orange_share"]["top"] / base["orange_share"]["top"]) if base["orange_share"]["top"] > 0 else None
        r["area_ratio_expected"] = (r["scale"] / smin) ** 2
    result["scale_mode"] = {
        "scales": scales, "rows": rows, "collision_geoms": collision_info, "collision_text": collision_text, "drone_mass_kg": mass, "propeller_body_masses_kg": prop_mass,
        "scaled_geoms": parts["geom_names"], "scaled_meshes": parts["mesh_names"], "propeller_bodies": len(parts["prop_bodies"]),
        "gate_picture": {"gate": abs(n0), "direction": "fwd" if n0 > 0 else "rev", "drone_position": gate_drone_pos.tolist(),
                         "gate_center": center.tolist(), "camera_distance_from_gate_m": gate_view["distance"]},
        "physics_hash_full_before": full_before, "physics_hash_ignoring_drawn_rows": blank_before,
    }
    sheet_path = make_sheet(Image, ImageDraw, out, scales, {r["scale"]: r["drawn_width_m"] for r in rows})
    result["scale_mode"]["sheet"] = sheet_path.name
    if 1.0 in scales and "drone_pad_start_side_0p4m" in images:  # a measurement: the same camera and model as the earlier picture
        a = images_scale[(1.0, "side")].astype(np.float32)
        b = images["drone_pad_start_side_0p4m"].astype(np.float32)
        result["scale_mode"]["scale1_side_vs_pad_picture_mean_abs_diff"] = float(np.abs(a - b).mean())

    entry("drone_scale_drawn_size_equals_scale", not size_bad,
          "drawn width (largest of the x and y extent of the scaled vertices, in the drone frame) at "
          + ", ".join(f"{r['scale']:g}: {r['drawn_width_m']:.4f} m" for r in rows) + f"; ratio to the smallest scale equals the scale within {SCALE_READBACK_TOL:g}; problems: {size_bad}")
    entry("drone_scale_physics_hash_unchanged", not hash_bad,
          f"hash of {len(rows)} scales (before and after each reset) equals the hash before any scale, with only the {len(parts['scaled_geoms'])} drawn geoms' geom_pos "
          f"and the {len(parts['prop_bodies'])} propeller bodies' body_pos left out; scales that differ: {sorted(set(hash_bad))}")
    entry("drone_scale_collision_mass_camera_unchanged", not changed_bad,
          f"collision geoms in use: {collision_text}; all non-mesh geoms of drone:0 (size, pos, quat, contype, conaffinity, friction) and body_mass (drone {mass} kg), body_inertia, cam_pos, cam_quat and "
          f"cam_fovy (fpv_cam) equal the originals at every scale; changed: {changed_bad}")
    entry("drone_scale_moved_bodies_massless", all(abs(v) <= MASSLESS_TOL for v in prop_mass),
          f"the four propeller bodies, whose body_pos moves with the scale, have masses {prop_mass} kg (limit {MASSLESS_TOL:g})")
    entry("drone_scale_position_after_reset_unchanged", not reset_bad,
          f"after env.reset() at each scale the observed position equals the one of the first reset (tolerance {START_TOL:g} m, quaternion {QUAT_TOL:g}); problems: {reset_bad}")
    entry("drone_scale_gate_picture_drone_placed", not place_bad,
          f"in the picture before gate {abs(n0)} the drone was drawn at {gate_drone_pos.tolist()} (read back from mj_data.xpos after the render; tolerance {SCALE_PLACE_TOL:g} m) "
          f"and the simulation's mocap_pos was put back; problems: {place_bad}")
    if len(scales) >= 2:
        tops = [r["orange_share"]["top"] for r in rows]
        need = AREA_GROWTH_FRACTION * (scales[-1] / smin) ** 2
        got = (tops[-1] / tops[0]) if tops[0] > 0 else float("nan")
        entry("drone_scale_top_orange_share_grows", tops[0] > 0 and all(b > a for a, b in zip(tops, tops[1:])) and got >= need,
              "orange share of the picture from above at " + ", ".join(f"{r['scale']:g}: {100 * r['orange_share']['top']:.3f} %" for r in rows)
              + f"; ratio of the largest to the smallest scale {got:.2f} (needs at least {need:.2f} = {AREA_GROWTH_FRACTION} x ({scales[-1]:g}/{smin:g})^2; "
              f"the area should grow about with the square of the scale, here {(scales[-1] / smin) ** 2:.2f}) and each share larger than the one before")
    return checks


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

        # 6. The drawing scale of the drone. Everything above is done and stays as it was; this goes last because it
        #    closes the viewer, calls env.reset() and changes the drawn model (it sets it back at the end).
        scales = parse_scales(args.drone_scales)
        if scales:
            import jax.numpy as jnp

            result["track_checks"].update(run_scale_mode(mujoco, np, iio, jnp, core, env, m, layout, result, scales, out, sync, images, obs))
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
        "--pads", str(Path(args.pads).resolve()), "--drone-scales", args.drone_scales,
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
    sm = e.get("scale_mode")
    if sm:
        lines += ["", "Drone drawing scale (pictures `scale_<scale>_side|top|gate|fpv.png` and the sheet `" + SHEET_FILE + "`, one row for each scale):", "",
                  "| scale | drawn width (m) | x extent (m) | y extent (m) | farthest point from the drone center, from above (m) | lowest drawn point above the floor at the start (m) "
                  "| orange share, side | orange share, from above | orange share, before the gate | orange share, drone camera (measured, not judged) | from above: share / share at the smallest scale (expected: scale^2 ratio) |",
                  "|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in sm.get("rows") or []:
            sh = r.get("orange_share") or {}
            lines.append(
                f"| {num(r.get('scale'), 'g')} | {num(r.get('drawn_width_m'), '.4f')} | {num(r.get('width_x_m'), '.4f')} | {num(r.get('width_y_m'), '.4f')} | "
                f"{num(r.get('max_radius_xy_m'), '.4f')} | {num(r.get('lowest_point_above_floor_at_start_m'), '.4f')} | {num(sh.get('side'), '.5f')} | {num(sh.get('top'), '.5f')} | "
                f"{num(sh.get('gate'), '.5f')} | {num(sh.get('fpv'), '.5f')} | {num(r.get('top_share_ratio_to_smallest'), '.2f')} (expected {num(r.get('area_ratio_expected'), '.2f')}) |"
            )
        lines += ["", f"The drone is only DRAWN larger: every vertex of its visual meshes ({', '.join(str(n) for n in sm.get('scaled_meshes') or [])}), the positions of those mesh geoms and the positions of "
                      f"the {sm.get('propeller_bodies')} propeller bodies are multiplied by the scale about the drone's body origin. Its collision size is unchanged: the collision shape in use ("
                      f"{sm.get('collision_text')}) keeps its size, and the mass ({num(sm.get('drone_mass_kg'), '.4f')} kg), the inertia, the camera fpv_cam and the "
                      f"physics fields are the same at every scale (see the checks above).",
                  "", f"Picture before the gate: gate {(sm.get('gate_picture') or {}).get('gate')} ({(sm.get('gate_picture') or {}).get('direction')}), the drone is drawn at "
                      f"{fmt3((sm.get('gate_picture') or {}).get('drone_position'))} (gate center {fmt3((sm.get('gate_picture') or {}).get('gate_center'))}); "
                      f"the picture at scale 1.0 from the side against the earlier close picture of the same camera: mean absolute difference {num(sm.get('scale1_side_vs_pad_picture_mean_abs_diff'), '.3f')} (0..255, expected near 0)."]
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
    scales = parse_scales(args.drone_scales)
    result = {
        "versions": {"python": platform.python_version()}, "attempts": [], "run": {},
        "expected_pictures": wanted, "expected_checks": expected_checks(layout, scales), "expected_files": scale_files(scales),
        "drone_scales": scales, "level_file": str(args.level), "layout_file": str(args.layout),
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
    parser.add_argument("--drone-scales", default=DEFAULT_DRONE_SCALES,
                        help="Drawing scales of the drone for the comparison pictures, comma separated (default %(default)s); "
                             "an empty text ('') switches this mode off. The flight physics and the collision size do not change.")
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
