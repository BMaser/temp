"""Draw clear pictures of every drone model that the simulator Crazyflow ships, from several sides.

Purpose: the owner wants to hand the pictures to a designer ("four small Crazyflie drones pictures, and we can put
an option to choose either of them in the simulator or improve the design or replace them by my own design").

Models (the files of crazyflow/drones, drones/__init__.py lines 15-22): cf2x_L250, cf2x_P250, cf2x_T350, cf21B_500
(four small Crazyflie drones) and hb_x500 (a large drone that has no flight parameters at this version).

For each model, in its own child process (so one model that fails does not stop the others):
  - make a Crazyflow `Sim` with that drone alone (no LSY environment, no gates),
  - put the drone 0.5 m or more above the floor,
  - draw eight 1024 x 768 pictures with the free camera: front, back, left, right, top, bottom,
    front_quarter (from the front, 30 degrees above, 35 degrees to the drone's left) and
    back_quarter (from behind, 30 degrees above, 35 degrees to the drone's right),
  - draw the same eight cameras again with the drone moved far behind the camera ("empty scene") so that a picture
    without a drone in it is caught,
  - read the compiled model back: mass, drawn size, drawn parts with colors, collision shapes, camera fpv_cam.
The parent then makes one sheet for each model and one gallery sheet for all models with PIL, writes result.json and
summary.md, and applies the pass rule.

Conventions: the drone's nose is body +x, its left is +y, up is +z (hb_x500.xml comment: "x points forward ... y left,
z up"; the camera fpv_cam looks along +x, see the cf2x xml files). "left" and "right" are the drone's own left and right.

Parent mode (default): runs one child per model (MUJOCO_GL = egl, then osmesa if egl gave no pictures).
Child mode (--child --model M --method METHOD): does the work for one model.
Check mode (--check FILE): applies the pass rule; only the standard library is imported.

Source lines used (Crazyflow commit d28ec70, MuJoCo 3.10.0, Gymnasium; all read before this file was written):
  crazyflow/sim/sim.py   Sim.__init__ lines 77-176 (arguments drone, n_worlds, n_drones; the drone body is "drone:0",
                         build_mjx_spec lines 287-312); Sim.render lines 214-272 (the viewer is made on the first call
                         from `cam_config`; mocap_pos and mocap_quat are copied from mjx_data at lines 265-266; the
                         offscreen size is set from width and height at lines 245-246); the decorator at lines 52-61
                         syncs sim.data -> mjx_data only when sim.data.core.mjx_synced is False; Sim.close lines 282-285;
                         sync_sim2mjx lines 711-724 puts data.states.pos / quat into the mocap data; _reset (line 440)
                         shows `core.replace(mjx_synced=False)`.
  crazyflow/sim/data.py  SimState.quat default (0, 0, 0, 1) at line 51; SimCore.mjx_synced at line 248 / 272.
  crazyflow/dynamics/core.py  load_params lines 96-122 (raises KeyError at 112-113 when the drone has no section in
                         dynamics/<dynamics>/params.toml); supported_dynamics lines 169-182 (a drone can fly when the
                         params.toml of some dynamics model has a section for it).
  crazyflow/examples/rendering/cameras.py lines 78-88: the repo's own example makes `Sim(n_drones=1, drone="cf2x_T350")`
                         and puts the drone somewhere with `sim.data.states.pos.at[...].set(...)`.
  mujoco 3.10.0 engine_vis_visualize.c: mjv_cameraFrustum lines 508-560 (free camera: fovy = vis.global.fovy; near
                         plane = vis.map.znear * stat.extent, far = vis.map.zfar * stat.extent; read at every
                         mjv_updateScene -> mjv_updateCamera, lines 2948-2971 and 3495); setMaterial lines 225-246 and 929
                         (drawn color); geoms with alpha 0 are skipped, line 962-964.
  mujoco camera_frame (forward, up, right of the free camera): engine_vis_visualize.c lines 452-473, as in
                         tests/crazyflow_track_look.py camera_frame.
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

MODELS = ("cf2x_L250", "cf2x_P250", "cf2x_T350", "cf21B_500", "hb_x500")
REQUIRED_MODELS = MODELS[:4]  # the four small Crazyflie drones must be drawn; the large model may fail
METHODS = ("egl", "osmesa")
WIDTH, HEIGHT = 1024, 768
FILL = 2.0 / 3.0  # the drone should fill about two thirds of the picture in its larger direction
MIN_PLACE_HEIGHT_M = 0.5
# (name, azimuth, elevation) of the free camera. forward = (cos el cos az, cos el sin az, sin el), the camera stands at
# lookat - distance * forward. The drone's nose is +x, so the camera of "front" stands at +x and looks toward -x.
VIEWS = (
    ("front", 180.0, 0.0),
    ("back", 0.0, 0.0),
    ("left", -90.0, 0.0),  # camera on the drone's left (+y), looking toward -y
    ("right", 90.0, 0.0),  # camera on the drone's right (-y), looking toward +y
    ("top", 0.0, -90.0),  # straight down; nose at the top of the picture, the drone's right on the right
    ("bottom", 180.0, 90.0),  # straight up; nose at the top of the picture (the drone's left is on the right)
    ("front_quarter", 215.0, -30.0),  # camera at the front, 35 degrees toward the left, 30 degrees above
    ("back_quarter", 35.0, -30.0),  # camera behind, 35 degrees toward the right, 30 degrees above
)
VIEW_NAMES = tuple(v[0] for v in VIEWS)
GALLERY_VIEWS = ("front", "right", "top", "front_quarter")
EMPTY_BEHIND_CAMERA_M = 20.0  # for the empty-scene picture the drone's center is this far behind the camera
PIXEL_DIFF_THRESHOLD = 12  # a pixel "differs" from the empty scene when some channel differs by more than this (0..255)
MIN_DIFF_SHARE = 0.003  # share of pixels that must differ from the empty scene (0.3 %)
MIN_FILL = 0.3  # the differing pixels must span at least this share of the picture width or height
MIN_VIEW_DIFF = 2.0  # mean absolute difference (0..255) between two views of one model, over the pixels of the drone
CHILD_TIMEOUT_S = 1500
SHEET_THUMB = (512, 384)
GALLERY_THUMB = (384, 288)


def picture_name(model: str, view: str) -> str:
    return f"{model}__{view}.png"


def sheet_name(model: str) -> str:
    return f"{model}__sheet.png"


GALLERY_FILE = "drone_gallery.png"


# ---------------------------------------------------------------------------------------------
# Pass rule (used by the parent and by --check); standard library only
# ---------------------------------------------------------------------------------------------


def file_ok(base: Path, name) -> bool:
    return bool(name) and (base / name).is_file() and (base / name).stat().st_size > 0


def model_problems(name: str, run: dict, base: Path) -> list:
    """Everything that is wrong with one model's run. An empty list means the model is drawn and passes its checks."""
    problems = []
    if not isinstance(run, dict) or not run:
        return [f"{name}: no run record"]
    if run.get("traceback") or run.get("process_error"):
        text = (run.get("traceback") or run.get("process_error")).strip().splitlines()
        problems.append(f"{name}: run failed: " + (text[-1][:300] if text else "no detail"))
    pics = {p.get("view"): p for p in run.get("pictures", [])}
    for view in VIEW_NAMES:
        p = pics.get(view)
        if not p or not file_ok(base, p.get("file")):
            problems.append(f"{name}: picture {view} missing")
            continue
        share = p.get("diff_share")
        if not isinstance(share, (int, float)) or share < MIN_DIFF_SHARE:
            problems.append(f"{name}: picture {view} has too few pixels that differ from the empty scene ({share}; needs at least {MIN_DIFF_SHARE})")
        fill = p.get("fill_max")
        if not isinstance(fill, (int, float)) or fill < MIN_FILL:
            problems.append(f"{name}: in picture {view} the drone spans only {fill} of the picture (needs at least {MIN_FILL})")
    diff = run.get("min_view_pair_diff")
    if not isinstance(diff, (int, float)) or diff < MIN_VIEW_DIFF:
        problems.append(f"{name}: two views are too much alike (smallest mean absolute difference {diff}, pair {run.get('min_view_pair')}; needs at least {MIN_VIEW_DIFF})")
    mass = run.get("mass_compiled_kg")
    if not isinstance(mass, (int, float)) or not mass > 0:
        problems.append(f"{name}: no mass read from the compiled model")
    size = run.get("drawn_size_m") or {}
    if not all(isinstance(size.get(k), (int, float)) and size.get(k) > 0 for k in ("depth_x", "width_y", "height_z")):
        problems.append(f"{name}: no drawn size read from the compiled model")
    if not run.get("parts"):
        problems.append(f"{name}: no drawn parts were listed")
    if not file_ok(base, sheet_name(name)):
        problems.append(f"{name}: sheet {sheet_name(name)} missing")
    return problems


def evaluate(result: dict, base: Path) -> tuple:
    """Apply the pass rule. Returns (ok, reason). Standard library only."""
    problems, notes = [], []
    models = result.get("models") or {}
    expected = result.get("expected_models") or list(MODELS)
    drawn = {}
    for name in expected:
        run = (models.get(name) or {}).get("run")
        found = model_problems(name, run, base)
        if name in REQUIRED_MODELS:
            problems += found
        elif found:
            # the large model may fail without failing the test; a run that failed outright is reported by its error only
            notes.append(found[0] if "run failed" in found[0] or "no run record" in found[0] else "; ".join(found))
        pics = {p.get("view"): p for p in (run or {}).get("pictures", [])}
        if run and not (run.get("traceback") or run.get("process_error")) and all(v in pics and file_ok(base, pics[v].get("file")) for v in VIEW_NAMES):
            drawn[name] = run  # all eight picture files exist: they take part in the byte-identical check
    # Pictures of two different models must not be byte-identical (hashes are read from the files here).
    hashes = {}
    for name, run in drawn.items():
        for p in run.get("pictures", []):
            if file_ok(base, p.get("file")):
                digest = hashlib.sha256((base / p["file"]).read_bytes()).hexdigest()
                hashes.setdefault((p["view"], digest), []).append(name)
    for (view, _digest), names in sorted(hashes.items()):
        if len(names) > 1:
            text = f"the {view} pictures of {names} are byte-identical"
            if sum(1 for n in names if n in REQUIRED_MODELS) > 1:
                problems.append(text)
            else:
                notes.append(text)
    if not file_ok(base, GALLERY_FILE):
        problems.append(f"gallery sheet {GALLERY_FILE} missing")
    if not [n for n in REQUIRED_MODELS if n in expected]:
        problems.append("the result lists none of the four small Crazyflie models")
    result["notes"] = notes
    if problems:
        return False, "; ".join(problems)
    ok_models = [n for n in expected if n in drawn and not model_problems(n, drawn[n], base)]
    return True, (
        f"{len(ok_models)} of {len(expected)} models drawn with all checks passed ({', '.join(ok_models)}); "
        f"{len(expected) - len(ok_models)} could not be drawn or failed a check but are allowed to fail"
        + (f"; notes: {' | '.join(notes)}" if notes else "")
    )


def check(result_file: Path) -> int:
    if not result_file.is_file():
        print(f"FAIL: result file {result_file} does not exist")
        return 1
    ok, reason = evaluate(json.loads(result_file.read_text()), result_file.parent)
    print(("PASS: " if ok else "FAIL: ") + reason)
    return 0 if ok else 1


# ---------------------------------------------------------------------------------------------
# Child: geometry read back from the compiled model
# ---------------------------------------------------------------------------------------------


def camera_frame(az_deg: float, el_deg: float) -> dict:
    """forward, up, right of a free camera: mujoco 3.10.0 engine_vis_visualize.c lines 452-473."""
    ca, sa = math.cos(math.radians(az_deg)), math.sin(math.radians(az_deg))
    ce, se = math.cos(math.radians(el_deg)), math.sin(math.radians(el_deg))
    return {
        "forward": [ce * ca, ce * sa, se],
        "up": [-se * ca, -se * sa, ce],
        "right": [sa, -ca, 0.0],
    }


def drawn_rgba(np, m, g: int) -> list:
    """The color the renderer draws for geom g: setMaterial (engine_vis_visualize.c lines 225-246, called at line 929)."""
    rgba = [float(v) for v in m.geom_rgba[g]]
    mat = int(m.geom_matid[g])
    default = abs(rgba[0] - 0.5) < 1e-6 and abs(rgba[1] - 0.5) < 1e-6 and abs(rgba[2] - 0.5) < 1e-6 and abs(rgba[3] - 1.0) < 1e-6
    if mat >= 0 and default:
        return [float(v) for v in m.mat_rgba[mat]]
    return rgba


def local_box_corners(np, mujoco, m, g: int):
    """Corners (n, 3) of the box that holds geom g in its own frame."""
    t = int(m.geom_type[g])
    s = [float(v) for v in m.geom_size[g]]
    G = mujoco.mjtGeom
    if t == int(G.mjGEOM_MESH):
        a, n = int(m.mesh_vertadr[int(m.geom_dataid[g])]), int(m.mesh_vertnum[int(m.geom_dataid[g])])
        return np.asarray(m.mesh_vert[a:a + n], dtype=np.float64)
    if t == int(G.mjGEOM_BOX) or t == int(G.mjGEOM_ELLIPSOID):
        h = s[:3]
    elif t == int(G.mjGEOM_SPHERE):
        h = [s[0]] * 3
    elif t == int(G.mjGEOM_CYLINDER):
        h = [s[0], s[0], s[1]]
    elif t == int(G.mjGEOM_CAPSULE):
        h = [s[0], s[0], s[1] + s[0]]
    else:
        r = float(m.geom_rbound[g])
        h = [r] * 3
    return np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=np.float64)


def read_geometry(mujoco, np, m) -> dict:
    """Mass, drawn size, drawn parts with colors, collision shapes and camera fpv_cam of the drone, from the compiled model.

    The drone is put at the origin with no rotation in a private MjData, so that world coordinates of its geoms are
    the coordinates in the drone's own frame (x forward, y left, z up).
    """
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "drone:0")  # sim.py line 299 / 331: drone body "drone" + suffix ":0"
    if bid < 0:
        raise ValueError("body drone:0 not found in the compiled model")
    mocap = int(m.body_mocapid[bid])
    if mocap < 0:
        raise ValueError("drone:0 is not a mocap body")
    dd = mujoco.MjData(m)
    dd.mocap_pos[mocap] = [0.0, 0.0, 0.0]
    dd.mocap_quat[mocap] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_kinematics(m, dd)
    mujoco.mj_comPos(m, dd)
    mujoco.mj_camlight(m, dd)
    type_names = {int(v): n for n, v in mujoco.mjtGeom.__members__.items()}

    def name_of(kind, i):
        return mujoco.mj_id2name(m, kind, int(i)) if int(i) >= 0 else None

    bodies = [b for b in range(m.nbody) if int(m.body_rootid[b]) == bid]
    body_set = set(bodies)
    parts, hidden, active, inactive, point_sets = [], [], [], [], []
    for g in range(int(m.ngeom)):
        if int(m.geom_bodyid[g]) not in body_set:
            continue
        gname = name_of(mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}"
        mesh = re.sub(r":\d+$", "", name_of(mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g]) or "") if int(m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_MESH) else None
        mat = name_of(mujoco.mjtObj.mjOBJ_MATERIAL, m.geom_matid[g])
        mat = re.sub(r":\d+$", "", mat) if mat else None
        rgba = drawn_rgba(np, m, g)
        row = {
            "geom": re.sub(r":\d+$", "", gname), "body": re.sub(r":\d+$", "", name_of(mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or ""),
            "type": type_names.get(int(m.geom_type[g]), str(int(m.geom_type[g]))), "mesh": mesh, "material": mat,
            "rgba": [round(v, 4) for v in rgba], "rgb255": [int(round(v * 255)) for v in rgba[:3]], "group": int(m.geom_group[g]),
            "contype": int(m.geom_contype[g]), "conaffinity": int(m.geom_conaffinity[g]),
        }
        collides = row["contype"] != 0 or row["conaffinity"] != 0
        if row["group"] == 3 or collides:  # class "collision" of the xml files is group 3
            row["size_m"] = [round(float(v), 5) for v in m.geom_size[g]]
            row["pos_m"] = [round(float(v), 5) for v in dd.geom_xpos[g]]
            (active if collides else inactive).append(row)
            continue
        if row["group"] > 2 or rgba[3] <= 0.0:  # the default group flags draw groups 0-2 only; alpha 0 is skipped (visualize.c 962-964)
            row["reason"] = "alpha 0 (not drawn)" if rgba[3] <= 0.0 else f"group {row['group']} (not drawn)"
            hidden.append(row)
            continue
        local = local_box_corners(np, mujoco, m, g)
        rot = np.asarray(dd.geom_xmat[g], dtype=np.float64).reshape(3, 3)
        world = local @ rot.T + np.asarray(dd.geom_xpos[g], dtype=np.float64)
        point_sets.append(world)
        row["extent_m"] = [round(float(v), 5) for v in (world.max(axis=0) - world.min(axis=0))]
        parts.append(row)
    if not point_sets:
        raise ValueError("no drawn geom was found on the drone")
    pts = np.concatenate(point_sets, axis=0)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    cam = {}
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "fpv_cam:0")  # camera name + suffix, as in the repo example cameras.py
    if cid >= 0:
        xmat = np.asarray(dd.cam_xmat[cid], dtype=np.float64).reshape(3, 3)
        cam = {
            "position_m": [round(float(v), 5) for v in dd.cam_xpos[cid]],
            "looks_along": [round(float(v), 5) for v in -xmat[:, 2]],  # MuJoCo cameras look along their negative z axis
            "up": [round(float(v), 5) for v in xmat[:, 1]],
            "fovy_deg": float(m.cam_fovy[cid]),
        }
    return {
        "points": pts,
        "mass_compiled_kg": float(sum(float(m.body_mass[b]) for b in bodies)),
        "mass_drone_body_kg": float(m.body_mass[bid]),
        "drawn_size_m": {
            "depth_x": float(hi[0] - lo[0]), "width_y": float(hi[1] - lo[1]), "height_z": float(hi[2] - lo[2]),
            "x_range": [float(lo[0]), float(hi[0])], "y_range": [float(lo[1]), float(hi[1])], "z_range": [float(lo[2]), float(hi[2])],
        },
        "center_body_m": [float(v) for v in (lo + hi) / 2.0],
        "parts": parts, "hidden_parts": hidden, "collision_active": active, "collision_inactive": inactive,
        "fpv_cam": cam, "n_drawn_geoms": len(parts), "n_bodies": len(bodies),
    }


def choose_views(np, pts, fovy_deg: float) -> tuple:
    """Camera distance of each view so that the drone fills FILL of the picture in its limiting direction.

    A point at lateral offset r and depth z = distance + w (w = coordinate along the viewing direction, from the center)
    is drawn at r / (z * tan(fovy/2) * aspect) of the half width (up/down: no aspect). Requiring that the farthest
    such point is at FILL gives distance >= |r| / (FILL * t * aspect) - w for every point. The camera also stays
    outside the drone: distance > -min(w) + 15 % of the drone's size.
    """
    center = (pts.min(axis=0) + pts.max(axis=0)) / 2.0
    rel = pts - center
    size = float((pts.max(axis=0) - pts.min(axis=0)).max())
    t = math.tan(math.radians(fovy_deg) / 2.0)
    aspect = WIDTH / HEIGHT
    out = {}
    for name, az, el in VIEWS:
        fr = camera_frame(az, el)
        f, u, r = (np.asarray(fr[k], dtype=np.float64) for k in ("forward", "up", "right"))
        w, uu, rr = rel @ f, rel @ u, rel @ r
        need = np.maximum(np.abs(rr) / (FILL * t * aspect), np.abs(uu) / (FILL * t)) - w
        dist = max(float(need.max()), float(-w.min()) + 0.15 * size)
        out[name] = {"az": az, "el": el, "distance": dist, "nearest_depth": dist + float(w.min()), "farthest_depth": dist + float(w.max()), "frame": fr}
    return center, size, out


# ---------------------------------------------------------------------------------------------
# Child: the two ways to get a drone into a scene and draw it
# ---------------------------------------------------------------------------------------------


class SimBackend:
    """A Crazyflow Sim with one drone (sim.py lines 77-176). The drone is placed through sim.data.states (as the repo
    example cameras.py lines 85-88 does) and the flag mjx_synced is cleared so that Sim.render copies the new place into
    the mocap data (sim.py decorator lines 52-61, sync_sim2mjx lines 711-724, render lines 265-266)."""

    kind = "crazyflow_sim"

    def __init__(self, drone: str):
        import jax.numpy as jnp
        from crazyflow.drones import Drone
        from crazyflow.sim import Sim

        self.jnp = jnp
        self.sim = Sim(n_worlds=1, n_drones=1, drone=Drone(drone))
        self.m = self.sim.mj_model

    def place(self, pos) -> None:
        sim = self.sim
        st = sim.data.states
        states = st.replace(pos=st.pos.at[...].set([float(v) for v in pos]), quat=st.quat.at[...].set([0.0, 0.0, 0.0, 1.0]))
        core = sim.data.core.replace(mjx_synced=self.jnp.asarray(False))
        sim.data = sim.data.replace(states=states, core=core)

    def render(self, cam_config=None):
        return self.sim.render(mode="rgb_array", width=WIDTH, height=HEIGHT, camera=-1, cam_config=cam_config)

    def cam(self):
        return self.sim.viewer.viewer.cam

    def close(self) -> None:
        self.sim.close()


class PlainBackend:
    """Fallback for a drone that Sim cannot be made for (hb_x500 has no flight parameters): the same scene.xml and the
    same drone body, attached the same way as in Sim.build_mjx_spec (sim.py lines 287-300) and Sim._replicate_drone
    (absolute meshdir, round trip through XML, lines 325-346), drawn with the same Gymnasium renderer and the same
    steps as Sim.render (lines 232-272). No flight parameters and no JAX data are involved."""

    kind = "plain_mujoco"

    def __init__(self, drone: str):
        import mujoco
        import crazyflow.sim.sim as simmod
        from gymnasium.envs.mujoco.mujoco_rendering import MujocoRenderer

        self.mujoco, self.Renderer = mujoco, MujocoRenderer
        root = Path(simmod.__file__).parents[1]  # sim.py lines 148-149: Path(__file__).parents[1]
        scene, drone_path = root / "scene.xml", root / "drones" / f"{drone}.xml"
        spec = mujoco.MjSpec.from_file(str(scene))
        spec.copy_during_attach = True
        drone_spec = mujoco.MjSpec.from_file(str(drone_path))
        frame = spec.worldbody.add_frame(name="world")
        body = drone_spec.body("drone")
        if body is None:
            raise ValueError("Drone body not found in drone spec")
        body.mocap = True
        frame.attach_body(body, "", ":0")
        spec.meshdir = str((drone_path.parent / drone_spec.meshdir).resolve())
        self.m = mujoco.MjSpec.from_string(spec.to_xml()).compile()
        self.d = mujoco.MjData(self.m)
        self.mocap = int(self.m.body_mocapid[self.m.body("drone:0").id])
        self.viewer = None
        self.pos = [0.0, 0.0, 0.5]

    def place(self, pos) -> None:
        self.pos = [float(v) for v in pos]

    def render(self, cam_config=None):
        mujoco, m, d = self.mujoco, self.m, self.d
        if self.viewer is None:
            m.vis.global_.offwidth = WIDTH
            m.vis.global_.offheight = HEIGHT
            self.viewer = self.Renderer(m, d, max_geom=1000, default_cam_config=cam_config, height=HEIGHT, width=WIDTH, camera_id=-1)
        d.mocap_pos[self.mocap] = self.pos
        d.mocap_quat[self.mocap] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        mujoco.mj_camlight(m, d)
        return self.viewer.render("rgb_array")

    def cam(self):
        return self.viewer.viewer.cam

    def close(self) -> None:
        if self.viewer is not None:
            self.viewer.close()
        self.viewer = None


def flight_facts(model: str, np) -> dict:
    """Does Crazyflow have flight parameters for this drone? dynamics/core.py: supported_dynamics (line 169) is the list
    of dynamics models whose params.toml has a section for the drone; load_params (lines 96-122) raises KeyError otherwise."""
    facts = {"supported_dynamics": [], "has_flight_params": False, "mass_params_kg": None, "params_error": None}
    try:
        from crazyflow.drones import Drone
        from crazyflow.dynamics import Dynamics, load_params, supported_dynamics

        facts["supported_dynamics"] = [str(d) for d in supported_dynamics(Drone(model))]
        facts["has_flight_params"] = len(facts["supported_dynamics"]) > 0
        try:
            p = load_params(Dynamics.first_principles, Drone(model))
            facts["mass_params_kg"] = float(np.asarray(p["mass"]).reshape(-1)[0])
        except Exception as exc:  # no section for this drone: the expected case for hb_x500
            facts["params_error"] = f"{type(exc).__name__}: {exc}"[:300]
    except Exception:
        facts["params_error"] = traceback.format_exc()[-600:]
    return facts


def save_png(Image, np, path: Path, img) -> None:
    Image.fromarray(np.ascontiguousarray(img)).save(path)


def run_child(model: str, method: str, args) -> None:
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "empty").mkdir(exist_ok=True)
    run = {
        "model": model, "method": method, "MUJOCO_GL": os.environ.get("MUJOCO_GL"), "versions": {}, "backend": None,
        "sim_error": None, "pictures": [], "traceback": None,
    }
    backend = None
    try:
        os.environ["SCIPY_ARRAY_API"] = "1"  # crazyflow's own camera examples set this
        import importlib.metadata as md
        import mujoco
        import numpy as np
        from PIL import Image

        run["versions"] = {"mujoco": mujoco.__version__, "crazyflow": md.version("crazyflow"), "lsy_drone_racing_commit": args.lsy_commit}
        run.update(flight_facts(model, np))
        try:
            backend = SimBackend(model)
        except Exception:
            run["sim_error"] = traceback.format_exc()[-1500:]
            backend = PlainBackend(model)  # a failure here ends the child with a traceback
        run["backend"] = backend.kind
        m = backend.m
        geo = read_geometry(mujoco, np, m)
        pts = geo.pop("points")
        run.update(geo)
        fovy = float(m.vis.global_.fovy)  # free camera: mjv_cameraFrustum lines 508-521
        center, size, views = choose_views(np, pts, fovy)

        # Near plane = vis.map.znear * stat.extent (mjmodel.h line 175, engine_vis_visualize.c line 543), read at every
        # frame (mjv_updateCamera, line 3495). If the nearest point of the drone in any view is closer than twice the
        # near plane, lower znear on the compiled model before the first picture.
        extent = float(m.stat.extent)
        near_before = float(m.vis.map.znear) * extent
        min_depth = min(v["nearest_depth"] for v in views.values())
        near_after = near_before
        if near_before > 0.5 * min_depth:
            m.vis.map.znear = float(0.5 * min_depth / extent)
            near_after = float(m.vis.map.znear) * extent
        run["camera"] = {
            "fovy_deg": fovy, "extent_m": extent, "near_plane_before_m": near_before, "near_plane_after_m": near_after,
            "far_plane_m": float(m.vis.map.zfar) * extent, "nearest_drone_depth_m": min_depth,
            "farthest_drone_depth_m": max(v["farthest_depth"] for v in views.values()), "fill_target": FILL,
            "near_plane_lowered": near_after < near_before,
        }

        # Height of the drone: at least 0.5 m, and high enough that the camera of the bottom view stays 0.25 m above the
        # floor and the lowest drawn point stays 0.1 m above it.
        cz, zmin = float(center[2]), float(pts[:, 2].min())
        z = max(MIN_PLACE_HEIGHT_M, 0.25 + views["bottom"]["distance"] - cz, 0.1 - zmin)
        pos = np.array([0.0, 0.0, z])
        run["place_position_m"] = pos.tolist()
        run["place_height_raised_above_0p5"] = z > MIN_PLACE_HEIGHT_M + 1e-9

        images, blanks = {}, {}
        first = True
        for name, az, el in VIEWS:
            v = views[name]
            fwd = np.asarray(v["frame"]["forward"])
            lookat = pos + center
            cam_pos = lookat - v["distance"] * fwd
            empty_pos = lookat - (v["distance"] + EMPTY_BEHIND_CAMERA_M) * fwd - center  # drone center 20 m behind the camera
            cfg = {"lookat": np.asarray(lookat, dtype=np.float64), "distance": float(v["distance"]), "azimuth": float(az), "elevation": float(el)}
            backend.place(pos)
            if first:
                img = backend.render(cfg)  # the viewer is made here and takes cam_config (sim.py lines 232-252)
                first = False
            else:
                cam = backend.cam()
                cam.lookat[:] = [float(x) for x in lookat]
                cam.distance, cam.azimuth, cam.elevation = float(v["distance"]), float(az), float(el)
                img = backend.render(None)
            backend.place(empty_pos)
            blank = backend.render(None)
            img, blank = np.ascontiguousarray(img), np.ascontiguousarray(blank)
            if img.shape != (HEIGHT, WIDTH, 3) or img.dtype != np.uint8 or blank.shape != img.shape:
                raise ValueError(f"unexpected picture {img.shape} {img.dtype} / {blank.shape}")
            diff = np.abs(img.astype(np.int16) - blank.astype(np.int16)).max(axis=2)
            mask = diff > PIXEL_DIFF_THRESHOLD
            rows, cols = np.where(mask.any(axis=1))[0], np.where(mask.any(axis=0))[0]
            fill_h = float((rows.max() - rows.min() + 1) / HEIGHT) if rows.size else 0.0
            fill_w = float((cols.max() - cols.min() + 1) / WIDTH) if cols.size else 0.0
            fname = picture_name(model, name)
            save_png(Image, np, out / fname, img)
            save_png(Image, np, out / "empty" / f"{model}__{name}__empty.png", blank)
            images[name], blanks[name] = img, mask
            run["pictures"].append({
                "view": name, "file": fname, "empty_file": f"empty/{model}__{name}__empty.png",
                "azimuth_deg": az, "elevation_deg": el, "distance_m": v["distance"], "lookat_m": lookat.tolist(),
                "camera_position_m": cam_pos.tolist(), "forward": v["frame"]["forward"], "up": v["frame"]["up"], "right": v["frame"]["right"],
                "nearest_depth_m": v["nearest_depth"], "diff_share": float(mask.mean()), "fill_w": fill_w, "fill_h": fill_h,
                "fill_max": max(fill_w, fill_h), "sha256": hashlib.sha256((out / fname).read_bytes()).hexdigest(),
                "mean": float(img.mean()),
            })

        # The eight views of one model must not be alike: mean absolute difference over the pixels that belong to the
        # drone in either of the two pictures (the floor is the same in several views, so the whole picture would hide it).
        names = list(images)
        worst, worst_pair = None, None
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = names[i], names[j]
                sel = blanks[a] | blanks[b]
                if not sel.any():
                    d = 0.0
                else:
                    d = float(np.abs(images[a].astype(np.int16) - images[b].astype(np.int16))[sel].mean())
                if worst is None or d < worst:
                    worst, worst_pair = d, [a, b]
        run["min_view_pair_diff"], run["min_view_pair"] = worst, worst_pair
    except Exception:
        run["traceback"] = traceback.format_exc()
    finally:
        try:
            if backend is not None:
                backend.close()
        except Exception:
            run["close_traceback"] = traceback.format_exc()[-800:]
        (out / f"{model}__child.json").write_text(json.dumps(run, indent=2, default=float))


# ---------------------------------------------------------------------------------------------
# Sheets (parent; PIL)
# ---------------------------------------------------------------------------------------------


def text_on(Image, ImageDraw, ImageFont, canvas, xy, text: str, size: int, fill=(0, 0, 0)) -> None:
    """Write text on `canvas`. Pillow 10.1+ can load its default font at any size; older Pillow cannot, so the small
    default font is drawn on a mask and enlarged."""
    try:
        font = ImageFont.load_default(size=size)
    except Exception:
        font = None
    if font is not None:
        ImageDraw.Draw(canvas).text(xy, text, fill=fill, font=font)
        return
    resample = getattr(Image, "Resampling", Image)
    tile = Image.new("L", (max(8, 8 * len(text)), 16), 0)
    ImageDraw.Draw(tile).text((0, 0), text, fill=255)
    box = tile.getbbox()
    if box is None:
        return
    tile = tile.crop(box)
    k = max(1, round(size / 10.0))
    tile = tile.resize((tile.width * k, tile.height * k), resample.NEAREST)
    canvas.paste(fill, (int(xy[0]), int(xy[1])), tile)


def fnum(v, spec: str = ".1f", scale: float = 1.0) -> str:
    try:
        return format(float(v) * scale, spec)
    except (TypeError, ValueError):
        return "?"


def size_text(run: dict) -> str:
    s = run.get("drawn_size_m") or {}
    return f"width {fnum(s.get('width_y'), '.0f', 1000)} mm, height {fnum(s.get('height_z'), '.0f', 1000)} mm, depth {fnum(s.get('depth_x'), '.0f', 1000)} mm"


def make_sheets(result: dict, out: Path) -> dict:
    """One sheet for each drawn model (eight views in a grid) and the gallery (one row for each model, four views).
    Returns {file name: error text} for the sheets that could not be made."""
    from PIL import Image, ImageDraw, ImageFont

    resample = getattr(Image, "Resampling", Image)
    errors = {}
    models = result.get("models") or {}
    expected = result.get("expected_models") or list(MODELS)

    def thumb(run, view, size):
        p = {q.get("view"): q for q in run.get("pictures", [])}.get(view)
        if not p or not file_ok(out, p.get("file")):
            return None
        with Image.open(out / p["file"]) as im:
            return im.convert("RGB").resize(size, resample.LANCZOS)

    # one sheet for each model
    tw, th = SHEET_THUMB
    head, label_h, cols = 120, 30, 4
    for name in expected:
        run = (models.get(name) or {}).get("run") or {}
        if not run.get("pictures"):
            continue
        try:
            sheet = Image.new("RGB", (cols * tw, head + 2 * (th + label_h)), (255, 255, 255))
            text_on(Image, ImageDraw, ImageFont, sheet, (14, 8), name, 40)
            mass = run.get("mass_compiled_kg")
            text_on(Image, ImageDraw, ImageFont, sheet, (14, 56), f"mass {fnum(mass, '.1f', 1000)} g (compiled model)    drawn size: {size_text(run)}", 24)
            text_on(Image, ImageDraw, ImageFont, sheet, (14, 88),
                    "nose = drone +x, left = drone +y; left/right are the drone's own sides; top view: nose at the top; bottom view: seen from below", 18, (90, 90, 90))
            for i, view in enumerate(VIEW_NAMES):
                x, y = (i % cols) * tw, head + (i // cols) * (th + label_h)
                t = thumb(run, view, (tw, th))
                if t is not None:
                    sheet.paste(t, (x, y + label_h))
                text_on(Image, ImageDraw, ImageFont, sheet, (x + 10, y + 3), view, 24)
            sheet.save(out / sheet_name(name))
        except Exception:
            errors[sheet_name(name)] = traceback.format_exc()[-600:]

    # the gallery
    gw, gh = GALLERY_THUMB
    label_w, gead = 300, 50
    try:
        sheet = Image.new("RGB", (label_w + len(GALLERY_VIEWS) * gw, gead + len(expected) * gh), (255, 255, 255))
        for j, view in enumerate(GALLERY_VIEWS):
            text_on(Image, ImageDraw, ImageFont, sheet, (label_w + j * gw + 10, 10), view, 26)
        for i, name in enumerate(expected):
            run = (models.get(name) or {}).get("run") or {}
            y = gead + i * gh
            text_on(Image, ImageDraw, ImageFont, sheet, (10, y + 10), name, 30)
            if run.get("pictures"):
                text_on(Image, ImageDraw, ImageFont, sheet, (10, y + 56), f"mass {fnum(run.get('mass_compiled_kg'), '.1f', 1000)} g", 20)
                s = run.get("drawn_size_m") or {}
                text_on(Image, ImageDraw, ImageFont, sheet, (10, y + 84), f"width {fnum(s.get('width_y'), '.0f', 1000)} mm", 20)
                text_on(Image, ImageDraw, ImageFont, sheet, (10, y + 108), f"height {fnum(s.get('height_z'), '.0f', 1000)} mm", 20)
                text_on(Image, ImageDraw, ImageFont, sheet, (10, y + 132), f"depth {fnum(s.get('depth_x'), '.0f', 1000)} mm", 20)
                for j, view in enumerate(GALLERY_VIEWS):
                    t = thumb(run, view, (gw, gh))
                    if t is not None:
                        sheet.paste(t, (label_w + j * gw, y))
            else:
                err = (run.get("traceback") or run.get("process_error") or "not drawn").strip().splitlines()
                text_on(Image, ImageDraw, ImageFont, sheet, (10, y + 60), "not drawn", 24, (160, 0, 0))
                text_on(Image, ImageDraw, ImageFont, sheet, (10, y + 96), (err[-1] if err else "")[:34], 16, (160, 0, 0))
        sheet.save(out / GALLERY_FILE)
    except Exception:
        errors[GALLERY_FILE] = traceback.format_exc()[-600:]
    return errors


# ---------------------------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------------------------


def vec(v, digits: int = 3) -> str:
    try:
        return "(" + ", ".join(f"{float(x):.{digits}f}" for x in v) + ")"
    except (TypeError, ValueError):
        return "?"


def write_summary(result: dict, path: Path) -> None:
    """Write summary.md. Every field is read with dict.get, so a missing field shows '?' and never stops the run."""
    models = result.get("models") or {}
    expected = result.get("expected_models") or list(MODELS)
    lines = ["### Drone gallery: pictures of every drone model that Crazyflow ships", "",
             f"**{'PASS' if result.get('passed') else 'FAIL'}**: {result.get('pass_reason', 'no pass reason was recorded')}", "",
             f"Pictures: {WIDTH} x {HEIGHT} points, free camera, the model's own colors, drone about {MIN_PLACE_HEIGHT_M} m above the floor. "
             f"Views: {', '.join(VIEW_NAMES)}. Files: `<model>__<view>.png`, `<model>__sheet.png`, `{GALLERY_FILE}`. "
             "Left and right are the drone's own sides; the nose is the drone's +x.", ""]
    lines += ["| model | drawn with | mass, compiled (g) | mass, parameter file (g) | flight parameters | drawn width / height / depth (mm) "
              "| drawn parts | collision shape in use | camera fpv_cam (position m, looks along) | pictures | smallest share of pixels that differ from the empty scene |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
    for name in expected:
        run = (models.get(name) or {}).get("run") or {}
        s = run.get("drawn_size_m") or {}
        cam = run.get("fpv_cam") or {}
        coll = "; ".join(f"{c.get('geom')}: {c.get('type', '').replace('mjGEOM_', '')} size {c.get('size_m')}" for c in run.get("collision_active") or []) or "?"
        pics = run.get("pictures") or []
        shares = [p.get("diff_share") for p in pics if isinstance(p.get("diff_share"), (int, float))]
        flight = "yes (" + ", ".join(run.get("supported_dynamics") or []) + ")" if run.get("has_flight_params") else "no"
        lines.append(
            f"| {name} | {run.get('backend')} | {fnum(run.get('mass_compiled_kg'), '.1f', 1000)} | {fnum(run.get('mass_params_kg'), '.1f', 1000)} | {flight} | "
            f"{fnum(s.get('width_y'), '.0f', 1000)} / {fnum(s.get('height_z'), '.0f', 1000)} / {fnum(s.get('depth_x'), '.0f', 1000)} | "
            f"{len(run.get('parts') or [])} | {coll} | {vec(cam.get('position_m'))}, {vec(cam.get('looks_along'))} | {len(pics)} of {len(VIEW_NAMES)} | "
            f"{fnum(min(shares) if shares else None, '.4f')} |"
        )
    lines.append("")
    for name in expected:
        run = (models.get(name) or {}).get("run") or {}
        err = None
        if run.get("traceback") or run.get("process_error") or not run:
            err = (run.get("traceback") or run.get("process_error") or "no run record").strip()
        if err is not None:
            lines.append(f"- **{name} could not be drawn**: `{err.splitlines()[-1][:400] if err else ''}` (full text in `{name}__child.json` and the run log)")
        if run.get("sim_error"):
            lines.append(f"- {name}: `Sim` could not be made for this drone, so it was drawn with a plain MuJoCo scene instead. Error of `Sim`: `{run['sim_error'].strip().splitlines()[-1][:400]}`")
    for name, text in (result.get("sheet_errors") or {}).items():
        lines.append(f"- sheet {name} could not be made: `{text.strip().splitlines()[-1][:300] if text.strip() else ''}`")
    lines += ["", "Drawn parts with colors (the color the renderer uses; rgb 0-255, alpha 0-1):", ""]
    for name in expected:
        run = (models.get(name) or {}).get("run") or {}
        if not run.get("parts"):
            continue
        lines.append(f"- **{name}**: " + "; ".join(
            f"{p.get('mesh') or p.get('geom')} [{p.get('body')}] {p.get('material') or 'no material'} rgb{tuple(p.get('rgb255') or [])} a={(p.get('rgba') or [0, 0, 0, 0])[3]}"
            for p in run["parts"]))
        if run.get("hidden_parts"):
            lines.append(f"  - not drawn: " + "; ".join(f"{p.get('geom')} ({p.get('reason')})" for p in run["hidden_parts"]))
        if run.get("collision_inactive"):
            lines.append("  - collision shapes that exist but are switched off: " + "; ".join(
                f"{c.get('geom')} {c.get('type', '').replace('mjGEOM_', '')} size {c.get('size_m')}" for c in run["collision_inactive"]))
    lines += ["", "Cameras and pictures:", "",
              "| model | view | azimuth | elevation | distance (m) | camera position (m) | pixels that differ from the empty scene | drone spans of width / height |",
              "|---|---|---|---|---|---|---|---|"]
    for name in expected:
        run = (models.get(name) or {}).get("run") or {}
        for p in run.get("pictures") or []:
            lines.append(f"| {name} | {p.get('view')} | {fnum(p.get('azimuth_deg'), '.0f')} | {fnum(p.get('elevation_deg'), '.0f')} | {fnum(p.get('distance_m'), '.3f')} | "
                         f"{vec(p.get('camera_position_m'))} | {fnum(p.get('diff_share'), '.4f')} | {fnum(p.get('fill_w'), '.2f')} / {fnum(p.get('fill_h'), '.2f')} |")
    lines += ["", "Camera, near plane and height of the drone:", ""]
    for name in expected:
        run = (models.get(name) or {}).get("run") or {}
        c = run.get("camera") or {}
        if c:
            lines.append(f"- {name}: fovy {c.get('fovy_deg')} deg, near plane {fnum(c.get('near_plane_before_m'), '.4f')} m"
                         + (f" lowered to {fnum(c.get('near_plane_after_m'), '.4f')} m" if c.get("near_plane_lowered") else " (not lowered)")
                         + f", nearest drone point {fnum(c.get('nearest_drone_depth_m'), '.3f')} m from the camera, farthest {fnum(c.get('farthest_drone_depth_m'), '.3f')} m, far plane {fnum(c.get('far_plane_m'), '.1f')} m; "
                         f"drone placed at {vec(run.get('place_position_m'))} m")
    for n in result.get("notes") or []:
        lines.append(f"- note: {n}")
    lines += ["", f"Attempts: {[ (a.get('model'), a.get('method'), a.get('pictures'), a.get('error')) for a in result.get('attempts') or [] ]}", "",
              "Versions: " + ", ".join(f"{k} {v}" for k, v in (result.get("versions") or {}).items())]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------------------------
# Parent
# ---------------------------------------------------------------------------------------------


def run_one(model: str, method: str, args, out: Path) -> dict:
    cmd = [sys.executable, str(Path(__file__).resolve()), "--child", "--model", model, "--method", method,
           "--out", str(out), "--lsy-commit", args.lsy_commit]
    entry = {}
    child_file = out / f"{model}__child.json"
    child_file.unlink(missing_ok=True)  # no stale file from an earlier try
    try:
        proc = subprocess.run(cmd, env={**os.environ, "MUJOCO_GL": method}, capture_output=True, text=True, timeout=CHILD_TIMEOUT_S)
        (out / f"{model}__{method}_stdout.txt").write_text(proc.stdout)
        (out / f"{model}__{method}_stderr.txt").write_text(proc.stderr)
        entry["returncode"] = proc.returncode
        entry["stderr_tail"] = proc.stderr[-3000:]
    except subprocess.TimeoutExpired:
        entry["process_error"] = f"child timed out after {CHILD_TIMEOUT_S} s"
    if child_file.is_file():
        entry.update(json.loads(child_file.read_text()))
    else:
        entry.setdefault("process_error", "child wrote no child json (crashed hard)")
    return entry


def run_parent(args) -> None:
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    wanted = [m for m in (args.models.split(",") if args.models else MODELS) if m]
    unknown = [m for m in wanted if m not in MODELS]
    if unknown:
        raise SystemExit(f"unknown models {unknown}; the models are {MODELS}")
    result = {"versions": {"python": platform.python_version()}, "attempts": [], "models": {}, "expected_models": wanted}
    good_method = None
    for model in wanted:
        order = list(METHODS)
        if good_method:
            order = [good_method] + ([] if model not in REQUIRED_MODELS else [x for x in METHODS if x != good_method])
        entry = {}
        for method in order:
            t0 = time.perf_counter()
            entry = run_one(model, method, args, out)
            ok = not (entry.get("traceback") or entry.get("process_error")) and len(entry.get("pictures", [])) >= len(VIEWS)
            result["attempts"].append({"model": model, "method": method, "pictures": len(entry.get("pictures", [])), "seconds": round(time.perf_counter() - t0, 1),
                                       "error": (entry.get("traceback") or entry.get("process_error") or "").strip()[-400:]})
            if ok:
                good_method = method
                break
        result["models"][model] = {"run": entry}
        result["versions"].update(entry.get("versions", {}))
    try:
        result["sheet_errors"] = make_sheets(result, out)
    except Exception:
        result["sheet_errors"] = {"all sheets": traceback.format_exc()[-800:]}
    result["passed"], result["pass_reason"] = evaluate(result, out)
    (out / "result.json").write_text(json.dumps(result, indent=2, default=float))
    write_summary(result, out / "summary.md")
    print(("PASS: " if result["passed"] else "FAIL: ") + result["pass_reason"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", help="Folder for all output files")
    parser.add_argument("--models", default="", help="Comma separated models (default: all five)")
    parser.add_argument("--lsy-commit", default="unknown", help="Commit of the LSY checkout whose environment is used, only recorded")
    parser.add_argument("--child", action="store_true", help="Internal: do the work for one model")
    parser.add_argument("--model", choices=MODELS, help="Internal: model of the child")
    parser.add_argument("--method", choices=METHODS, help="Internal: drawing method of the child")
    parser.add_argument("--check", help="Apply the pass rule to this result.json and exit 1 on failure")
    args = parser.parse_args()
    if args.check:
        sys.exit(check(Path(args.check)))
    if not args.out:
        parser.error("--out is required")
    if args.child:
        if not (args.model and args.method):
            parser.error("--child needs --model and --method")
        run_child(args.model, args.method, args)
    else:
        run_parent(args)


if __name__ == "__main__":
    main()
