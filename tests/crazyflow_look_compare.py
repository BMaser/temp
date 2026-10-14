"""Five looks of the racing simulator as still pictures, side by side, so that the owner can choose one.

Looks: today (the files of this repository now), magenta, cyan, red, violet (the four returned designs, assets/looks/).
The track is level 0 of lsy_drone_racing (commit ac7377b) with the pads (tools/track_env.py) and the floor and sky of the
look (tools/look_env.py). The seed is 0, one env.reset() per look, one child process per look.

Sub-commands (each writes into --out):
  run      for each look: build its files (tools/look_assets.py), copy its gate files over LSY's asset files in the checkout
           (--repo), start one child, then write the sheets look_compare.png and compare_<view>.png. Writes looks.json.
  summary  summary.md and result.json from looks.json. Exit code 1 if a check failed.
  check    applies the pass rule to result.json (standard library only), exit code 1 on failure.
Internal: child (one look in its own process; the parent starts it).

Why the parent copies the gate files, and why one child per look: LSY loads its gate from lsy_drone_racing/envs/assets/gate.xml
(race_core.py line 330 `gate_spec_path`, line 679 `MjSpec.from_file`), and the package is an editable install of the checkout,
so the files in that folder are what LSY loads. They must be in place BEFORE the environment is made, and a process reads them
once. So: copy, then start a child for that look, wait, then copy the next look's files. Each child checks the hashes of the
files in LSY's folder against the look's source files.

Pictures per look (640 x 480, in <out>/<look>/): gate_front_1p5m, gate_oblique_0p7m, scene_side, top_down, pad_start_drone_0p4m,
pad_end_top_1p0m, fpv_start, fpv_at_gate. The last one is taken after 3.84 s of flight (192 steps at 50 Hz) with
controllers/nose_forward_controller.py, NOSE_MODE "forward", YAW_RATE_MAX 4.0.

Checks that can fail are in result.json. The picture-based checks (floor squares in top_down, floor colors, sky gradient) measure
the pictures, not the model, so they can fail although the model read back is right.
"""

import argparse
import json
import math
import os
import platform
import re
import shutil
import subprocess
import sys
import time
import traceback
from pathlib import Path

try:
    import numpy as np
except ImportError:  # the check and summary sub-commands need only the standard library
    np = None

HERE = Path(__file__).resolve().parent
REPO_DIR = HERE.parent
TOOLS_DIR = REPO_DIR / "tools"
CONTROLLERS_DIR = REPO_DIR / "controllers"
for _p in (HERE, TOOLS_DIR, CONTROLLERS_DIR):
    sys.path.insert(0, str(_p))

import crazyflow_track_look as tl  # noqa: E402  (standard library only at import)
import look_assets as la  # noqa: E402  (standard library only)

METHODS = ("egl", "osmesa")
WIDTH, HEIGHT = 640, 480
SEED = 0
CHILD_TIMEOUT_S = 1500
VIEWS = ("gate_front_1p5m", "gate_oblique_0p7m", "scene_side", "top_down", "pad_start_drone_0p4m", "pad_end_top_1p0m", "fpv_start", "fpv_at_gate")
SHEET_VIEWS = ("gate_front_1p5m", "scene_side", "fpv_at_gate")  # one sheet each with the five pictures side by side
FLIGHT_SECONDS = 3.84
NOSE_MODE, YAW_RATE_MAX = "forward", 4.0
SQUARE_M = 0.5
SQUARE_TOLERANCE = 0.10  # the square size measured in top_down.png against 0.5 m
MAT_TOL = 1e-6  # float32 storage of a color / 255
SKY_PICTURE_TOL = 6.0  # levels (0-255) between the sky in scene_side.png and the gradient of the look
FLOOR_PICTURE_TOL = 6.0  # levels (0-255) between the dominant floor colors of top_down.png and the look's colors
SKY_ROWS = (1, 40, 80)  # rows of scene_side.png; a row is checked only when its ray is at least 1 degree above the horizon
SCENE_SIDE = {"lookat": (0.0, 0.0, 0.9), "distance": 9.5, "azimuth": 90.0, "elevation": -14.0}
GATE_FRONT_M, GATE_OBLIQUE_M = 1.5, 0.7
GATE_OBLIQUE_AZIMUTH_OFFSET_DEG, GATE_OBLIQUE_ELEVATION_DEG = 40.0, -15.0
GATE_OBLIQUE_LOCAL = (0.0, 0.22, -0.30)  # the point the oblique picture looks at, in the gate's own axes (x normal, y right, z up)
TOP_HALF_EXTENT_M = tl.TOP_HALF_EXTENT_M
LABELS = {"gate_front_1p5m": "gate 1 from 1.5 m", "gate_oblique_0p7m": "gate corner 0.7 m", "scene_side": "track from the side",
          "top_down": "from above", "pad_start_drone_0p4m": "drone on start pad", "pad_end_top_1p0m": "end pad", "fpv_start": "drone camera, start",
          "fpv_at_gate": "drone camera, 3.84 s"}


def add_check(checks: list, name: str, ok: bool, detail: str, **numbers) -> None:
    checks.append({"name": name, "ok": bool(ok), "detail": detail, **numbers})


# ---------------------------------------------------------------------------------------------
# Measures on pictures (numpy only)
# ---------------------------------------------------------------------------------------------


def dominant_two(img):
    """The most frequent exact color and the most frequent color at least 5 levels away from it: (c1, c2)."""
    flat = img.reshape(-1, 3).astype(np.int64)
    codes = (flat[:, 0] << 16) | (flat[:, 1] << 8) | flat[:, 2]
    vals, counts = np.unique(codes, return_counts=True)
    order = np.argsort(-counts, kind="stable")

    def color(code):
        return np.array([(code >> 16) & 255, (code >> 8) & 255, code & 255], dtype=float)

    c1 = color(int(vals[order[0]]))
    for j in order[1:]:
        c2 = color(int(vals[j]))
        if np.linalg.norm(c2 - c1) > 5.0:
            return c1, c2
    return c1, None


def square_spacings(img, ca, cb, row_step: int = 3, min_run: int = 3, max_gap: int = 4):
    """Distances in pixels between neighbouring color changes A<->B along the rows of a picture of the floor.

    A pixel is A, B or other (distance to the color above 0.3 times the distance between A and B, at least 3 levels). A run of at
    least `min_run` pixels of one color is solid. Two neighbouring solid runs of different colors with a gap of at most `max_gap`
    pixels between them (anti-aliasing) make one color change, at the middle of the gap. Only a run with a color change on both
    sides is a full square, and its length is the distance between the two changes. Returns (list of spacings, rows with any).
    """
    ca, cb = np.asarray(ca, dtype=float), np.asarray(cb, dtype=float)
    tol = max(3.0, 0.3 * float(np.linalg.norm(ca - cb)))
    spacings, rows_used = [], 0
    for y in range(0, img.shape[0], row_step):
        row = img[y].astype(float)
        da, db = np.linalg.norm(row - ca, axis=1), np.linalg.norm(row - cb, axis=1)
        lab = np.full(len(row), 2)
        lab[(da < tol) & (da <= db)] = 0
        lab[(db < tol) & (db < da)] = 1
        runs, i = [], 0
        while i < len(lab):
            j = i
            while j + 1 < len(lab) and lab[j + 1] == lab[i]:
                j += 1
            if lab[i] != 2 and j - i + 1 >= min_run:
                runs.append((int(lab[i]), i, j))
            i = j + 1
        edges = [None] * max(len(runs) - 1, 0)
        for k in range(len(runs) - 1):
            (l0, _, e0), (l1, s1, _) = runs[k], runs[k + 1]
            if l0 != l1 and s1 - e0 - 1 <= max_gap:
                edges[k] = (e0 + s1) / 2.0
        found = False
        for k in range(1, len(runs) - 1):
            if edges[k - 1] is not None and edges[k] is not None:
                spacings.append(edges[k] - edges[k - 1])
                found = True
        rows_used += int(found)
    return spacings, rows_used


def meters_per_pixel(fovy_deg: float, distance_m: float, height_px: int = HEIGHT) -> float:
    """Straight-down free camera at `distance_m` above the floor: the picture is `height_px` pixels for 2 * d * tan(fovy / 2)
    meters at the floor (the vertical field of view fovy, m.vis.global_.fovy; the same relation tl uses for TOP_HALF_EXTENT_M)."""
    return 2.0 * distance_m * math.tan(math.radians(fovy_deg) / 2.0) / height_px


def measure_floor_squares(img, ca, cb, mpp: float) -> dict:
    spacings, rows = square_spacings(img, ca, cb)
    out = {"n_spacings": len(spacings), "rows_with_squares": rows, "meters_per_pixel": mpp}
    if spacings:
        med = float(np.median(spacings))
        out.update(median_spacing_px=med, square_m=med * mpp, spacing_px_min=float(min(spacings)), spacing_px_max=float(max(spacings)))
    return out


def sky_expected(look: str, fovy_deg: float, elevation_deg: float, row: int, height_px: int = HEIGHT) -> list:
    """Color (0-255) that the middle column of row `row` of a picture shows when it looks at the sky of the look: the ray's
    normalized elevation (angle above the horizon / 90 degrees) picks the point between horizon and top (look_env.sky_pixels)."""
    p = la.palette(look)
    half = math.tan(math.radians(fovy_deg) / 2.0)
    above_axis = math.degrees(math.atan(half * ((height_px / 2.0) - (row + 0.5)) / (height_px / 2.0)))
    t = min(1.0, max(0.0, (above_axis + elevation_deg) / 90.0))
    return [p["skyH"][i] + t * (p["skyT"][i] - p["skyH"][i]) for i in range(3)]


def sky_rows_check(img, look: str, fovy_deg: float, elevation_deg: float) -> dict:
    """Rows of scene_side.png against the gradient. A row is checked only if its ray is above the horizon by at least 1 degree."""
    rows = {}
    worst = 0.0
    for r in SKY_ROWS:
        half = math.tan(math.radians(fovy_deg) / 2.0)
        above = math.degrees(math.atan(half * ((HEIGHT / 2.0) - (r + 0.5)) / (HEIGHT / 2.0))) + elevation_deg
        if above < 1.0:
            continue
        got = np.median(img[r, WIDTH // 2 - 10:WIDTH // 2 + 11].astype(float), axis=0)
        want = np.array(sky_expected(look, fovy_deg, elevation_deg, r))
        diff = float(np.abs(got - want).max())
        rows[str(r)] = {"picture": got.round(1).tolist(), "expected": want.round(1).tolist(), "max_abs_difference": diff, "ray_above_horizon_deg": above}
        worst = max(worst, diff)
    return {"rows": rows, "worst": worst, "n_rows": len(rows)}


def analyze_pictures(images: dict, look: str, info: dict, checks: list, result: dict) -> None:
    """Checks that read the pictures. `info` has fovy_deg, top_distance_m. Needs top_down and scene_side in `images`."""
    c = lambda name: f"{look}/{name}"  # noqa: E731
    fovy = float(info["fovy_deg"])
    top = images.get("top_down")
    if top is None:
        add_check(checks, c("floor_squares_in_picture"), False, "top_down.png was not made")
    else:
        d = float(info["top_distance_m"])
        mpp = meters_per_pixel(fovy, d)
        if look == "today":
            c1, c2 = dominant_two(top)
            colors = (c1, c2)
        else:
            a, b = la.palette(look)["floor"]
            colors = (np.array(a, dtype=float), np.array(b, dtype=float))
            c1, c2 = dominant_two(top)
            ok_a = c2 is not None and max(np.abs(c1 - colors[0]).max(), np.abs(c2 - colors[1]).max()) <= FLOOR_PICTURE_TOL
            ok_b = c2 is not None and max(np.abs(c1 - colors[1]).max(), np.abs(c2 - colors[0]).max()) <= FLOOR_PICTURE_TOL
            near = bool(ok_a or ok_b)
            add_check(checks, c("floor_colors_in_picture"), near,
                      f"the two most frequent colors of top_down.png are {c1.tolist()} and {None if c2 is None else c2.tolist()}; the look's floor colors are {a} and {b}; allowed difference {FLOOR_PICTURE_TOL:g} levels",
                      rendered=[c1.tolist(), None if c2 is None else c2.tolist()], wanted=[a, b])
        if colors[1] is None:
            add_check(checks, c("floor_squares_in_picture"), False, "top_down.png has only one floor color")
        else:
            meas = measure_floor_squares(top, colors[0], colors[1], mpp)
            result["floor_squares_measured"] = meas
            ok = meas["n_spacings"] >= 10 and abs(meas.get("square_m", 0.0) - SQUARE_M) <= SQUARE_TOLERANCE * SQUARE_M
            add_check(checks, c("floor_squares_in_picture"), ok,
                      f"size of a floor square measured in top_down.png: {meas.get('square_m')} m from {meas['n_spacings']} squares in {meas['rows_with_squares']} rows "
                      f"(median {meas.get('median_spacing_px')} px x {mpp:.6f} m per pixel, fovy {fovy:g} deg, camera {d:.4f} m above the floor); wanted {SQUARE_M} m within {SQUARE_TOLERANCE:.0%}",
                      **{k: v for k, v in meas.items() if k != "n_spacings"})
    side = images.get("scene_side")
    if look != "today":
        if side is None:
            add_check(checks, c("sky_in_picture"), False, "scene_side.png was not made")
        else:
            s = sky_rows_check(side, look, fovy, SCENE_SIDE["elevation"])
            result["sky_in_picture"] = s
            add_check(checks, c("sky_in_picture"), s["n_rows"] >= 2 and s["worst"] <= SKY_PICTURE_TOL,
                      f"{s['n_rows']} rows of scene_side.png above the horizon differ from the gradient horizon -> top of the look by at most {s['worst']:.1f} levels (allowed {SKY_PICTURE_TOL:g})",
                      rows=s["rows"])


# ---------------------------------------------------------------------------------------------
# Child: one look in its own process
# ---------------------------------------------------------------------------------------------


def run_child(args) -> None:
    out = Path(args.out).resolve()
    look = args.look
    sub = out / look
    sub.mkdir(parents=True, exist_ok=True)
    c = lambda name: f"{look}/{name}"  # noqa: E731
    checks = []
    result = {"look": look, "MUJOCO_GL": os.environ.get("MUJOCO_GL"), "versions": {}, "pictures": {}, "gl_started": False, "traceback": None, "checks": checks}
    env = None
    try:
        os.environ["SCIPY_ARRAY_API"] = "1"  # crazyflow's own camera examples set this before importing scipy
        import importlib.metadata as md
        import tomllib

        import gymnasium
        import jax
        import mujoco
        from gymnasium.wrappers.jax_to_numpy import JaxToNumpy
        from lsy_drone_racing.envs.race_core import RaceCoreEnv
        from lsy_drone_racing.utils import load_config, load_controller
        from PIL import Image
        from scipy.spatial.transform import Rotation as Rot

        import look_env
        import nose_forward_controller as nfc
        import track_env

        repo, assets, pads_dir = Path(args.repo).resolve(), Path(args.assets).resolve(), Path(args.pads_dir).resolve()
        result["versions"] = {"mujoco": mujoco.__version__, "jax": jax.__version__, "crazyflow": md.version("crazyflow"),
                              "lsy_drone_racing": md.version("lsy_drone_racing"), "lsy_drone_racing_commit": args.lsy_commit,
                              "python": platform.python_version()}

        # 1. The gate files that LSY will load are the look's files.
        asset_dir = Path(RaceCoreEnv.gate_spec_path).resolve().parent
        want = la.gate_source_hashes(look)
        got = {n: (la.sha256(asset_dir / n) if (asset_dir / n).is_file() else None) for n in la.GATE_FILES}
        wrong = [n for n in la.GATE_FILES if got[n] != want[n]]
        add_check(checks, c("gate_files_loaded_by_lsy_equal_look_files"), not wrong,
                  f"sha256 of {len(la.GATE_FILES)} files in {asset_dir} against the look's source files (gate.xml: the text look_assets builds): "
                  + ("all equal" if not wrong else f"different or missing: {wrong}"), hashes=got)
        try:
            result["asset_check"] = tl.check_assets(repo, assets)  # imported package is inside the checkout; the built copy equals LSY's files
        except AssertionError as e:
            add_check(checks, c("lsy_package_is_the_checkout"), False, str(e))

        cfg = load_config(repo / "config" / "level0.toml")
        cfg.sim.render = False
        freq = int(cfg.env.freq)
        if cfg.env.id != track_env.BASE_ENV_ID:
            raise AssertionError(f"level0.toml names the environment {cfg.env.id!r}, expected {track_env.BASE_ENV_ID!r}")
        controller_cls = load_controller(CONTROLLERS_DIR / "nose_forward_controller.py")
        controller_cls.NOSE_MODE = NOSE_MODE
        controller_cls.YAW_RATE_MAX = YAW_RATE_MAX
        with open(repo / "config" / "level0.toml", "rb") as f:
            level = tomllib.load(f)
        start_xy = [float(v) for v in level["env"]["track"]["drones"][0]["pos"][:2]]
        end_xy = [float(v) for v in nfc.POINTS_AFTER_START[-1][:2]]

        env = gymnasium.make(
            look_env.register_look_env(),
            freq=cfg.env.freq, sim_config=cfg.sim, sensor_range=cfg.env.sensor_range, control_mode=cfg.env.control_mode,
            track=cfg.env.track, disturbances=cfg.env.get("disturbances"), randomizations=cfg.env.get("randomizations"),
            seed=SEED, pads={"pad_start": tuple(start_xy), "pad_end": tuple(end_xy)}, pads_dir=str(pads_dir), look=look,
        )
        env = JaxToNumpy(env)
        core = env.unwrapped
        obs0, info = env.reset()  # the one and only reset of this look
        m = core.sim.mj_model
        if core.sim.viewer is not None:
            raise AssertionError("a viewer exists before the first picture; the sky pixels would not be uploaded")
        pos0 = [float(v) for v in obs0["pos"]]
        err = float(max(abs(pos0[0] - start_xy[0]), abs(pos0[1] - start_xy[1])))
        add_check(checks, c("start_pad_under_drone"), err <= tl.START_TOL, f"drone start {pos0} against pad {start_xy}: {err} m (limit {tl.START_TOL})")

        # 2. Gate colors read back from the compiled model.
        exp = la.expected_gate_colors(look)
        names = {i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MATERIAL, i) or "" for i in range(int(m.nmat))}
        for base in ("frame_mat", "rope_mat"):
            ids = [i for i, n in names.items() if re.fullmatch(re.escape(base) + r"(:\d+)?", n)]
            rgbs = [[round(float(v) * 255.0, 3) for v in m.mat_rgba[i][:3]] for i in ids]
            ok = bool(ids) and all(max(abs(float(m.mat_rgba[i][k]) - exp[base][k] / 255.0) for k in range(3)) <= MAT_TOL for i in ids)
            add_check(checks, c(f"{base}_color_in_model"), ok,
                      f"{len(ids)} material(s) named {base}[:n] in the compiled model, rgb x255 = {rgbs[:4]}; wanted {list(exp[base])}" + ("" if ids else f"; material names: {sorted(names.values())[:30]}"))

        # 3. Floor read back from the compiled model.
        floor = look_env.read_floor(mujoco, np, m)
        result["floor_model"] = floor
        add_check(checks, c("floor_edit_applied"), (core.floor_edit or {}).get("changed") == (look != "today"), f"floor edit record: {core.floor_edit}")
        if look == "today":
            add_check(checks, c("floor_reflectance"), floor["reflectance"] > 0.0, f"floor reflectance {floor['reflectance']} (positive control: today keeps the mirror, so it must be above 0)")
        else:
            add_check(checks, c("floor_reflectance"), floor["reflectance"] == 0.0, f"floor reflectance {floor['reflectance']} (the new looks have no mirror)")
            ok, detail = look_env.floor_matches_look(np, floor, look)
            add_check(checks, c("floor_colors_in_model"), ok, detail)
        add_check(checks, c("floor_square_m_from_model"), abs(floor["square_m"] - SQUARE_M) <= 1e-9,
                  f"2 / texrepeat {floor['texrepeat']} = {2.0 / floor['texrepeat'][0]} m per picture, 2 x 2 squares: {floor['square_m']} m per square (wanted {SQUARE_M})")

        # 4. Sky: set on the compiled model (before the first picture), then read back.
        sky_edit = look_env.apply_sky_to_model(np, m, look)
        sky = look_env.read_sky(np, m)
        result["sky_model"] = {"edit": sky_edit, "read": sky}
        if look == "today":
            add_check(checks, c("sky_in_model"), True, f"sky not changed; model pixels: horizon {sky['horizon']}, top {sky['top']} (information only)")
        else:
            p = la.palette(look)
            dh = max(abs(a - b) for a, b in zip(sky["horizon"], p["skyH"]))
            dt = max(abs(a - b) for a, b in zip(sky["top"], p["skyT"]))
            add_check(checks, c("sky_in_model"), dh <= 1 and dt <= 1,
                      f"skybox pixels next to the horizon {sky['horizon']} (look {p['skyH']}) and next to the zenith {sky['top']} (look {p['skyT']}); allowed 1 level: the pixel next to the horizon is 0.0013 of the way up, the one next to the zenith 0.998",
                      horizon=sky["horizon"], top=sky["top"])

        # 5. The drone in the look's color (visual fields of the compiled model, as tests/crazyflow_run_a.py does for orange).
        rgb = la.drone_color_255(look)
        saved = tl.ORANGE_RGB
        tl.ORANGE_RGB = tuple(rgb)  # make_drone_orange reads this module constant when it runs
        try:
            drone_info = tl.make_drone_orange(mujoco, np, m)
        finally:
            tl.ORANGE_RGB = saved
        ok, detail = tl.orange_readback(mujoco, np, m, drone_info)
        got_rgb = [round(float(v) * 255.0) for v in m.mat_rgba[drone_info["material_id"]][:3]]
        add_check(checks, c("drone_color_in_model"), ok and got_rgb == list(rgb), f"drone material rgb x255 {got_rgb}, look {list(rgb)}; {detail}")

        # 6. Pictures.
        fovy = float(m.vis.global_.fovy)
        top_distance = TOP_HALF_EXTENT_M / math.tan(math.radians(fovy) / 2.0)
        gates_pos, gates_quat = np.asarray(obs0["gates_pos"], dtype=float), np.asarray(obs0["gates_quat"], dtype=float)
        g = int(np.asarray(obs0["gate_sequence"])[0])
        direction = 1.0 if int(np.asarray(obs0["gate_sequence_direction"])[0]) >= 0 else -1.0
        center = gates_pos[g]
        rot = Rot.from_quat(gates_quat[g])  # scipy order x, y, z, w
        heading = direction * rot.apply([1.0, 0.0, 0.0])  # crossing direction of the first passage (race_core.py: 1 = positive x of the gate)
        heading_az = math.degrees(math.atan2(heading[1], heading[0]))
        oblique_at = center + rot.apply(list(GATE_OBLIQUE_LOCAL))
        sx, sy, sz = pos0
        ex, ey = end_xy
        result["camera_inputs"] = {"fovy_deg": fovy, "top_distance_m": top_distance, "gate_index": g, "gate_center": center.tolist(), "gate_heading": heading.tolist()}

        def sync() -> None:
            if not core.data.sim_data.core.mjx_synced:  # same sync as RaceCoreEnv.render: copy gate positions into the MuJoCo data
                core.data, core.sim.mjx_data = core._render_sync(core.data, core.sim.mjx_data)

        def draw(**kw):
            sync()
            img = np.ascontiguousarray(core.sim.render(mode="rgb_array", width=WIDTH, height=HEIGHT, **kw))
            if img.shape != (HEIGHT, WIDTH, 3) or img.dtype != np.uint8:
                raise ValueError(f"unexpected picture {img.shape} {img.dtype}")
            result["gl_started"] = True
            return img

        state = {"kind": None}
        images = {}

        def fpv():
            if state["kind"] != "fpv":  # the viewer keeps the camera it was made with: close it to switch (as tests/crazyflow_track_look.py)
                core.sim.close()
                state["kind"] = "fpv"
            return draw(camera="fpv_cam:0")

        def free(lookat, distance, azimuth, elevation):
            lookat = [float(v) for v in lookat]
            if state["kind"] != "free":
                core.sim.close()
                state["kind"] = "free"
                cam_cfg = {"lookat": np.asarray(lookat), "distance": float(distance), "azimuth": float(azimuth), "elevation": float(elevation)}
                return draw(camera=-1, cam_config=cam_cfg)  # cam_config applies when the viewer is made
            cam = core.sim.viewer.viewer.cam
            cam.lookat[:] = lookat
            cam.distance, cam.azimuth, cam.elevation = float(distance), float(azimuth), float(elevation)
            return draw(camera=-1)

        def keep(view: str, img, **camera) -> None:
            Image.fromarray(img).save(sub / f"{view}.png")
            images[view] = img
            result["pictures"][view] = {"file": f"{view}.png", "mean": float(img.mean()), "std": float(img.std()), **camera}

        keep("fpv_start", fpv(), camera="fpv_cam:0")
        keep("gate_front_1p5m", free(center, GATE_FRONT_M, heading_az, 0.0), lookat=center.tolist(), distance=GATE_FRONT_M, azimuth=heading_az, elevation=0.0)
        oaz = heading_az + GATE_OBLIQUE_AZIMUTH_OFFSET_DEG
        keep("gate_oblique_0p7m", free(oblique_at, GATE_OBLIQUE_M, oaz, GATE_OBLIQUE_ELEVATION_DEG), lookat=oblique_at.tolist(), distance=GATE_OBLIQUE_M, azimuth=oaz, elevation=GATE_OBLIQUE_ELEVATION_DEG)
        s = SCENE_SIDE
        keep("scene_side", free(s["lookat"], s["distance"], s["azimuth"], s["elevation"]), **s)
        keep("top_down", free((0.0, 0.0, 0.0), top_distance, 90.0, -90.0), lookat=[0, 0, 0], distance=top_distance, azimuth=90.0, elevation=-90.0)
        keep("pad_start_drone_0p4m", free((sx, sy, sz + 0.02), 0.4, 90.0, -20.0), lookat=[sx, sy, sz + 0.02], distance=0.4, azimuth=90.0, elevation=-20.0)
        keep("pad_end_top_1p0m", free((ex, ey, 0.0), 1.0, 90.0, -90.0), lookat=[ex, ey, 0.0], distance=1.0, azimuth=90.0, elevation=-90.0)

        # 7. The flight (the same for every look) and the picture of the drone's camera at the end.
        controller = controller_cls(obs0, info, cfg)
        steps_wanted = int(round(FLIGHT_SECONDS * freq))
        rec = {k: [] for k in ("pos", "vel", "quat", "ang_vel", "passed")}

        def record(obs) -> None:
            rec["pos"].append([float(v) for v in obs["pos"]])
            rec["vel"].append([float(v) for v in obs["vel"]])
            rec["quat"].append([float(v) for v in obs["quat"]])
            rec["ang_vel"].append([float(v) for v in obs["ang_vel"]])
            rec["passed"].append(int(obs["n_gates_passed"]))

        obs = obs0
        record(obs)
        steps, end = 0, "finished_steps"
        while steps < steps_wanted:
            action = controller.compute_control(obs, info)
            obs, reward, terminated, truncated, info = env.step(action)
            controller.step_callback(action, obs, reward, terminated, truncated, info)
            steps += 1
            if terminated or truncated:
                end = "terminated" if terminated else "truncated"
                break
            record(obs)
        result["flight"] = {"steps": steps, "steps_wanted": steps_wanted, "end": end, "sim_seconds": steps / freq, "freq": freq,
                            "final_position": rec["pos"][-1], "passages": rec["passed"][-1]}
        add_check(checks, c("flight_reached_3p84_s"), end == "finished_steps" and steps == steps_wanted, f"{steps} of {steps_wanted} steps ({steps / freq:.2f} s), end: {end}")
        np.savez(sub / "states.npz", **{k: np.array(v) for k, v in rec.items()})
        keep("fpv_at_gate", fpv(), camera="fpv_cam:0", sim_seconds=steps / freq, position=rec["pos"][-1])

        # 8. Checks on the pictures.
        analyze_pictures(images, look, {"fovy_deg": fovy, "top_distance_m": top_distance}, checks, result)
        missing = [v for v in VIEWS if not (sub / f"{v}.png").is_file()]
        add_check(checks, c("all_8_pictures_exist"), not missing, f"missing: {missing}" if missing else f"{len(VIEWS)} pictures of {WIDTH} x {HEIGHT} in {sub}")
        result["done"] = True
    except Exception:
        result["traceback"] = traceback.format_exc()
        add_check(checks, c("child_finished"), False, "the child stopped with an error: " + result["traceback"].strip().splitlines()[-1])
    finally:
        try:
            if env is not None:
                env.close()
        except Exception:
            result["close_traceback"] = traceback.format_exc()
        (sub / "child.json").write_text(json.dumps(result, indent=2, default=str))


# ---------------------------------------------------------------------------------------------
# Parent
# ---------------------------------------------------------------------------------------------


def install_gate_files(built_gate_dir: Path, asset_dir: Path) -> dict:
    """Copy the look's gate files over LSY's asset files and compare them. Returns {name: sha256}."""
    if not asset_dir.is_dir():
        raise FileNotFoundError(f"LSY asset folder {asset_dir} does not exist (is --repo the checkout of lsy_drone_racing?)")
    hashes = {}
    for name in la.GATE_FILES:
        shutil.copyfile(built_gate_dir / name, asset_dir / name)
        a, b = la.sha256(built_gate_dir / name), la.sha256(asset_dir / name)
        if a != b:
            raise AssertionError(f"{name} differs after the copy")
        hashes[name] = b
    return hashes


class Runner:
    """Starts the child processes and remembers which drawing method worked."""

    def __init__(self, args):
        self.args = args
        self.out = Path(args.out).resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        (self.out / "logs").mkdir(exist_ok=True)
        self.methods = list(METHODS)

    def child_command(self, look: str, built: dict) -> list:
        a = self.args
        return [sys.executable, str(Path(__file__).resolve()), "child", "--out", str(self.out), "--repo", str(Path(a.repo).resolve()),
                "--assets", built["gate_dir"], "--pads-dir", built["pads_dir"], "--lsy-commit", a.lsy_commit, "--look", look]

    def run_once(self, look: str, built: dict, method: str) -> dict:
        child_json = self.out / look / "child.json"
        child_json.unlink(missing_ok=True)  # no stale file from an earlier try
        env = {**os.environ, "MUJOCO_GL": method}
        entry = {"method": method}
        try:
            proc = subprocess.run(self.child_command(look, built), env=env, capture_output=True, text=True, timeout=CHILD_TIMEOUT_S)
            (self.out / "logs" / f"{look}_{method}_stdout.txt").write_text(proc.stdout)
            (self.out / "logs" / f"{look}_{method}_stderr.txt").write_text(proc.stderr)
            entry["returncode"] = proc.returncode
            entry["stderr_tail"] = proc.stderr[-2000:]
        except subprocess.TimeoutExpired:
            entry["process_error"] = f"child timed out after {CHILD_TIMEOUT_S} s"
        if child_json.is_file():
            entry.update(json.loads(child_json.read_text()))
        else:
            entry.setdefault("process_error", "child wrote no child json (crashed hard)")
        return entry

    def run_look(self, look: str, built: dict) -> dict:
        """One child; egl first, osmesa next. A second method is tried only when no picture was drawn (a failure of the drawing
        library), not when the child failed later (another method would fail in the same way and cost the same time)."""
        entry = {}
        for method in list(self.methods):
            entry = self.run_once(look, built, method)
            started = bool(entry.get("gl_started"))
            tb = entry.get("traceback") or ""
            drawing_library_failed = (not started) and (bool(entry.get("process_error")) or bool(re.search(r"GL|mjr_|OSMesa|display", tb)))
            if started or (entry.get("done") and not tb) or (tb and not drawing_library_failed):
                self.methods = [method] + [x for x in self.methods if x != method]
                break
        return entry


def flight_differences(out: Path, looks: list, reference: str) -> dict:
    """Largest absolute difference of the recorded states (position, velocity, quaternion, angular velocity, every step) of each
    look to the reference look. Pass only at exactly 0.0."""
    ref_file = out / reference / "states.npz"
    result = {"reference": reference, "per_look": {}, "max_abs_difference": None}
    if not ref_file.is_file():
        return result
    ref = {k: v for k, v in np.load(ref_file, allow_pickle=False).items()}
    worst = 0.0
    for look in looks:
        f = out / look / "states.npz"
        if look == reference or not f.is_file():
            continue
        cur = {k: v for k, v in np.load(f, allow_pickle=False).items()}
        d = {}
        for k in ("pos", "vel", "quat", "ang_vel"):
            if ref[k].shape != cur[k].shape:
                d[k] = float("inf")
            else:
                d[k] = float(np.abs(ref[k].astype(float) - cur[k].astype(float)).max())
        d["max"] = max(d.values())
        result["per_look"][look] = d
        worst = max(worst, d["max"])
    result["max_abs_difference"] = worst
    return result


def make_sheets(out: Path, looks: list) -> dict:
    from PIL import Image, ImageDraw, ImageFont

    try:
        font = ImageFont.load_default(size=14)
        small = ImageFont.load_default(size=12)
    except TypeError:  # old Pillow: fixed-size font
        font = small = ImageFont.load_default()

    def load(look, view):
        f = out / look / f"{view}.png"
        if f.is_file():
            return Image.open(f).convert("RGB")
        img = Image.new("RGB", (WIDTH, HEIGHT), (90, 90, 90))
        ImageDraw.Draw(img).text((20, 20), f"missing: {look}/{view}", fill=(255, 255, 255), font=font)
        return img

    written = {}
    label_w, head_h = 130, 34
    cell_w = (3200 - label_w) // len(VIEWS)
    cell_h = cell_w * HEIGHT // WIDTH
    sheet = Image.new("RGB", (label_w + cell_w * len(VIEWS), head_h + cell_h * len(looks)), (255, 255, 255))
    draw = ImageDraw.Draw(sheet)
    for j, view in enumerate(VIEWS):
        draw.text((label_w + j * cell_w + 4, 4), LABELS[view], fill=(0, 0, 0), font=small)
        draw.text((label_w + j * cell_w + 4, 18), view, fill=(110, 110, 110), font=small)
    for i, look in enumerate(looks):
        draw.text((8, head_h + i * cell_h + cell_h // 2 - 8), look, fill=(0, 0, 0), font=font)
        for j, view in enumerate(VIEWS):
            sheet.paste(load(look, view).resize((cell_w, cell_h), Image.LANCZOS), (label_w + j * cell_w, head_h + i * cell_h))
    sheet.save(out / "look_compare.png")
    written["look_compare.png"] = list(sheet.size)
    for view in SHEET_VIEWS:
        strip = 30
        pic = Image.new("RGB", (WIDTH * len(looks), strip + HEIGHT), (255, 255, 255))
        d = ImageDraw.Draw(pic)
        for i, look in enumerate(looks):
            d.text((i * WIDTH + 8, 6), f"{look}  -  {LABELS[view]}", fill=(0, 0, 0), font=font)
            pic.paste(load(look, view), (i * WIDTH, strip))
        pic.save(out / f"compare_{view}.png")
        written[f"compare_{view}.png"] = list(pic.size)
    return written


def cmd_run(args) -> None:
    out = Path(args.out).resolve()
    runner = Runner(args)
    repo = Path(args.repo).resolve()
    asset_dir = repo / "lsy_drone_racing" / "envs" / "assets"
    looks = [x for x in args.looks.split(",") if x] if args.looks else list(la.LOOKS)
    for look in looks:
        la.check_look(look)
    data = {"looks": looks, "lsy_commit": args.lsy_commit, "platform": platform.platform(), "entries": {}, "parent_checks": []}
    checks = data["parent_checks"]
    t0 = time.perf_counter()
    for look in looks:
        t1 = time.perf_counter()
        try:
            built = la.build_look_files(look, out / "_built" / look)
            installed = install_gate_files(Path(built["gate_dir"]), asset_dir)
            add_check(checks, f"{look}/gate_files_copied_into_lsy", True, f"{len(installed)} files copied to {asset_dir} and compared", hashes=installed)
        except Exception as e:  # a look that cannot be set up does not stop the others
            add_check(checks, f"{look}/gate_files_copied_into_lsy", False, f"{type(e).__name__}: {e}")
            data["entries"][look] = {"setup_error": traceback.format_exc()}
            continue
        entry = runner.run_look(look, built)
        entry["wall_seconds"] = time.perf_counter() - t1
        data["entries"][look] = entry
        print(f"{look}: {'ok' if entry.get('done') else 'FAILED'} in {entry['wall_seconds']:.0f} s (method {entry.get('method')})", flush=True)
    data["wall_seconds"] = time.perf_counter() - t0
    data["flight_differences"] = flight_differences(out, looks, "today" if "today" in looks else looks[0]) if np is not None else {}
    fd = data["flight_differences"]
    worst = fd.get("max_abs_difference")
    add_check(checks, "flights_equal_across_looks", worst == 0.0 and len(fd.get("per_look", {})) == len(looks) - 1,
              f"largest absolute difference of position, velocity, quaternion and angular velocity over all steps, each look against {fd.get('reference')}: {worst}; looks compared: {sorted(fd.get('per_look', {}))} of {len(looks) - 1} (pass only at exactly 0.0)",
              per_look=fd.get("per_look"))
    try:
        data["sheets"] = make_sheets(out, looks)
        add_check(checks, "sheets_written", all((out / n).is_file() for n in data["sheets"]), f"written: {data['sheets']}")
    except Exception:
        data["sheets"] = {}
        add_check(checks, "sheets_written", False, "could not write the sheets: " + traceback.format_exc().strip().splitlines()[-1])
    (out / "looks.json").write_text(json.dumps(data, indent=2, default=str))


# ---------------------------------------------------------------------------------------------
# summary and check (standard library only)
# ---------------------------------------------------------------------------------------------


def read_json(path: Path):
    return json.loads(path.read_text()) if path.is_file() else None


def cmd_summary(args) -> None:
    out = Path(args.out).resolve()
    data = read_json(out / "looks.json")
    checks = []
    if data is None:
        add_check(checks, "looks_result_file_exists", False, "looks.json was not written: the run step did not run or crashed before it could write")
        data = {"looks": [], "entries": {}}
    else:
        checks.extend(data.get("parent_checks", []))
        for look in data["looks"]:
            e = data["entries"].get(look, {})
            if "checks" in e:
                checks.extend(e["checks"])
            else:
                add_check(checks, f"{look}/child_result_exists", False, e.get("setup_error", e.get("process_error", "the child wrote no result"))[-300:])
    if args.unit_outcome is not None:
        add_check(checks, "local_tests", args.unit_outcome == "success", f"tools/look_assets.py --self-test ended with outcome {args.unit_outcome!r}")
    failed = [c["name"] for c in checks if not c["ok"]]
    versions = next((e["versions"] for e in data["entries"].values() if e.get("versions")), {})
    result = {"passed": not failed and bool(checks), "n_checks": len(checks), "failed": failed, "checks": checks, "versions": versions,
              "looks": data["looks"], "flight_differences": data.get("flight_differences"), "sheets": data.get("sheets")}
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))

    L = ["### Looks of the racing simulator, side by side", ""]
    L.append(f"**{'PASS' if result['passed'] else 'FAIL'}**: {len(checks) - len(failed)} of {len(checks)} checks passed" + (f"; failed: {', '.join(failed)}" if failed else "") + ".")
    L += ["", "Sheets: `look_compare.png` (rows = looks, columns = the 8 views), `compare_gate_front_1p5m.png`, `compare_scene_side.png`, `compare_fpv_at_gate.png`. "
          "The pictures of each look are in the folder of its name.", "", "#### Looks", ""]
    L += ["| look | child | drawing method | seconds | floor square: model / picture (m) | floor colors in the picture | sky in the model (horizon / top) | flight difference to today |", "|---|---|---|---|---|---|---|---|"]
    fdiff = (data.get("flight_differences") or {}).get("per_look", {})
    for look in data["looks"]:
        e = data["entries"].get(look, {})
        sq_m = (e.get("floor_model") or {}).get("square_m")
        sq_p = (e.get("floor_squares_measured") or {}).get("square_m")
        flo = next((c.get("rendered") for c in e.get("checks", []) if c["name"].endswith("floor_colors_in_picture")), None)
        sky = (e.get("sky_model") or {}).get("read") or {}
        L.append(f"| {look} | {'ok' if e.get('done') else 'FAILED'} | {e.get('method')} | {e.get('wall_seconds', 0):.0f} | {sq_m} / {None if sq_p is None else round(sq_p, 4)} | {flo} | "
                 f"{sky.get('horizon')} / {sky.get('top')} | {fdiff.get(look, {}).get('max', '-')} |")
    L += ["", "#### Checks", "", "| check | result | detail |", "|---|---|---|"]
    for c in checks:
        L.append(f"| {c['name']} | {'pass' if c['ok'] else 'FAIL'} | {str(c['detail']).replace('|', '/')[:600]} |")
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
    parser.add_argument("command", choices=["run", "summary", "check", "child"])
    parser.add_argument("--out", help="folder for all output files")
    parser.add_argument("--repo", help="checkout of lsy_drone_racing")
    parser.add_argument("--lsy-commit", default="unknown", help="commit of the checkout, only recorded")
    parser.add_argument("--looks", help="run: comma separated looks (default: all five)")
    parser.add_argument("--unit-outcome", help="summary: outcome of the local test step (success or failure)")
    parser.add_argument("--result", help="check: result.json")
    # internal (child)
    parser.add_argument("--look")
    parser.add_argument("--assets", help="child: folder with the look's gate files as built by look_assets")
    parser.add_argument("--pads-dir", help="child: folder with the look's pad files as built by look_assets")
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
        if not (args.look and args.assets and args.pads_dir):
            parser.error("child needs --look, --assets and --pads-dir")
        run_child(args)
    else:
        cmd_run(args)


if __name__ == "__main__":
    main()
