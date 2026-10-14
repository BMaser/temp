"""Draw the GT drone (tools/obj_parts.py) on the simulated drone of LSY's racing environment, without touching its physics.

What this module does, in two steps that both edit the MuJoCo MODEL SPECIFICATION before it is compiled:
  1. add_gt_drone: for every material part of the GT model, add one mesh asset, one material and one mesh geom to the body
     "drone:0". The geom has contype 0, conaffinity 0, density 0 and group 2 (the group the present drone's own drawn shapes
     use, cf21B_500.xml line 39-40).
  2. hide_present_drawing: set the alpha of every mesh geom of the present drone (on "drone:0" and on its propeller bodies) to 0.

Why before the compile (file and line in the sources named here; all are saved under
G:/My Drive/github/Comprehansive_cv/coursera_mirror/mirrors/Autonomous_Racing_Drone_Path/runs/):
  - A compiled model has fixed arrays; a geom cannot be added to it. The only place to add a geom is the spec, and the spec is
    compiled by Sim.build_mjx (crazyflow sim/sim.py lines 504-509, which calls build_mjx_model, lines 348-361).
  - LSY compiles twice in the environment constructor and both times from the same `self.sim.spec`: once at the end of
    RaceCoreEnv._load_track_into_sim (race_core.py line 699, `self.sim.build_mjx()`), and once in use_box_collision (race_core.py
    line 395; crazyflow sim.py line 766-788 ends with `sim.build_mjx()`). _load_track_into_sim runs first (race_core.py line 394).
    tools/track_env.py already overrides that one method to attach the floor pads; GTDroneRaceEnv below subclasses the pad
    environment and adds the GT shapes to the spec first, so both compiles include them and nothing is edited after a compile.
  - The spec is the one that Sim._replicate_drone returned (sim.py lines 314-346): it was re-parsed from XML text, and it holds
    the drone body under the name "drone:0" (the attach suffix ":0", sim.py line 298; race_core.py lines 833 and 508 use that name).

Why this cannot change the flight (each claim with its source line):
  - Flight physics is not MuJoCo's body model. Crazyflow reads mass from its parameter files (race_core.py line 490-492,
    `self.sim.default_data.params.mass`), and the step functions are Crazyflow's own (sim.py lines 376-440). MuJoCo/MJX is used for
    kinematics and collision (sim.py lines 711-724 sync_sim2mjx; race_core.py line 612-626 contact check).
  - Collision. use_box_collision (sim.py lines 778-788) loops over `sim.spec.geoms` and changes only geoms whose NAME starts with
    "col_sphere" or "col_box". Our geoms are named "gt_*" and are never touched. The MJX collision pairs come from
    mjx/_src/collision_driver.py geom_pairs: a body is skipped if no geom of it has contype or conaffinity different from 0
    (lines 155-157), the geoms of a pair are taken only if contype or conaffinity is nonzero (lines 150, 175-176), and a pair
    needs a matching bit (lines 189-192). With contype 0 and conaffinity 0 our geoms are in no pair. MJX also builds convex hulls
    only for mesh geoms that are in a pair (mjx/_src/io.py lines 346-366, 417-423), so our meshes cost nothing there. Note:
    MuJoCo's default for contype and conaffinity is 1, so they MUST be set to 0 explicitly; this module does, and read_back
    checks the compiled values.
  - Contact masks (race_core.py lines 824-846, _load_contact_masks): a mask row marks the contact slots in which one geom lies
    in the id range [body_geomadr, body_geomadr + body_geomnum) of the body "drone:i". Our geoms are part of that range
    (they belong to "drone:0"), so a contact of one of them WOULD count as a drone contact; but they are in no pair, so no
    slot ever holds one of them. The slot list is made from the pairs in the same order as before, so the mask has the same
    shape and values; only the numeric geom ids of later bodies move up by the number of added geoms, and the masks are
    computed from the compiled model, so they follow. The test compares the mask of the compiled environment with and without
    the GT shapes, element by element.
  - Mass, inertia, center of mass. The body drone:0 has an explicit <inertial> (cf21B_500.xml line 52). Geoms add to a body's
    mass only when the compiler infers inertia from geoms (user_objects.cc InertiaFromGeom, lines 2455-2500), and that code
    ignores geoms whose mass is not above mjEPS (line 2468). With density 0 a geom's mass is 0 (user_objects.cc lines 4094-4097,
    `if (density == 0) mass_ = 0`). So whatever the compiler setting is, our geoms add exactly 0 to mass, inertia and center of
    mass. The test reads these numbers back from the compiled models and compares them for equality.
  - Hiding. The renderer (mjv_updateScene, engine_vis_visualize.c lines 895-960): setMaterial (lines 223-246) takes the color from
    the material first and then from the geom's rgba when that differs from (0.5, 0.5, 0.5, 1) or no material is set; a geom
    whose resulting alpha is 0 is skipped (lines 961-964). Setting only the alpha of the geom's own rgba to 0 therefore hides it
    whatever its material is, and rgba is a visual field that no physics code reads.
  - Group 2 is drawn by default (XMLreference.rst, geom group: "By default, groups 0, 1 and 2 are visible"), the same group the
    present drone's drawn shapes use.

Mesh files: absolute paths. MuJoCo uses a mesh file name that is an absolute path as it is and does not put the mesh directory
in front of it (XMLreference.rst, compiler meshdir, rule (1); user_util.cc FilePath::Combine lines 1071-1076). The compiled
spec's meshdir is the present drone's asset folder (sim.py line 326); that stays valid for the present meshes.

Spec calls used (python bindings of MuJoCo 3.10.0; fields from include/mujoco/mjspec.h lines 345-382 (geom), 516-537 (mesh),
610-628 (material); usage as in python/mujoco/specs_test.py lines 175-185, 275-312, 864-870, 1036-1050, 1758-1775):
  spec.body(name), body.add_geom(name=...), body.geoms, body.bodies, spec.add_mesh(name=...), mesh.file, mesh.scale,
  spec.add_material(name=..., rgba=..., specular=..., shininess=...), geom.type/meshname/material/contype/conaffinity/group/
  density/rgba. Only numpy is imported at module level; mujoco is used through the module object that callers pass in.
"""

import json
import math
from pathlib import Path

import numpy as np

import obj_parts

GT_ENV_ID = "DroneRacingGT-v0"
GT_PREFIX = obj_parts.MESH_PREFIX  # "gt_"
DRONE_BODY = "drone:0"
GT_GROUP = 2
MAT_TOL = 1e-6  # float32 storage of a color
EXTENT_TOL = 2e-6  # m, per unit of scale: float32 vertex storage is about 1e-8 m at these sizes, the STL files are float32


# ---------------------------------------------------------------------------------------------
# The look: what to draw (JSON-able, passed to the environment as the keyword argument `gt`)
# ---------------------------------------------------------------------------------------------


def make_look(look: str, scale: float, mesh_dir, asset_dir=obj_parts.DEFAULT_ASSET_DIR, skip=()) -> dict:
    """Write the mesh files of the GT model into mesh_dir and return the description of one look.

    look: one of obj_parts.LOOKS (the colors come from assets/drone_gt/mtl/<look>.mtl).
    scale: drawing scale (1.0 = the size of the model, 0.134 m across).
    skip: material names that are not drawn (default none).
    """
    asset_dir = Path(asset_dir)
    if look not in obj_parts.LOOKS:
        raise ValueError(f"unknown look {look!r}; known: {list(obj_parts.LOOKS)}")
    if not scale > 0:
        raise ValueError(f"scale must be positive, got {scale}")
    model, parts = obj_parts.load_parts(asset_dir / "drone.obj", asset_dir / "mtl" / f"{look}.mtl")
    mtl = obj_parts.parse_mtl(asset_dir / "mtl" / f"{look}.mtl")
    files = obj_parts.write_part_meshes(parts, mesh_dir)
    unknown = sorted(set(skip) - set(parts))
    if unknown:
        raise ValueError(f"skip names {unknown} are not materials of the model {sorted(parts)}")
    lo = np.min([p.vertices.min(axis=0) for p in parts.values()], axis=0)
    hi = np.max([p.vertices.max(axis=0) for p in parts.values()], axis=0)
    return {
        "look": look, "scale": float(scale), "skip": sorted(skip),
        "mesh_files": {n: files[n]["file"] for n in parts},
        "colors": {n: [float(c) for c in parts[n].color] for n in parts},
        # OBJ Ks (specular color, one number is enough: the files use gray) and Ns (exponent, 0..1000 in OBJ, 0..1 in MuJoCo)
        "specular": {n: float(mtl[n]["ks"][0]) if mtl[n]["ks"] else 0.0 for n in parts},
        "shininess": {n: float(min(1.0, mtl[n]["ns"] / 1000.0)) if mtl[n]["ns"] is not None else 0.0 for n in parts},
        "obj_extent": {"min": lo.tolist(), "max": hi.tolist()},
    }


def material_name(part: str) -> str:
    return f"{GT_PREFIX}mat_{part}"


def mesh_name(part: str) -> str:
    return f"{GT_PREFIX}mesh_{part}"


def geom_name(part: str) -> str:
    return f"{GT_PREFIX}{part}"


# ---------------------------------------------------------------------------------------------
# Spec edits
# ---------------------------------------------------------------------------------------------


def add_gt_drone(spec, mujoco, look: dict, body_name: str = DRONE_BODY) -> dict:
    """Add the GT parts to the body `body_name` of the (not yet compiled) spec. Returns what was added."""
    body = spec.body(body_name)
    if body is None:
        raise ValueError(f"body {body_name!r} not found in the spec")
    s = float(look["scale"])
    added = []
    for part, path in look["mesh_files"].items():
        if part in look.get("skip", ()):
            continue
        if not Path(path).is_absolute() or not Path(path).is_file():
            raise FileNotFoundError(f"mesh file for part {part} is not an existing absolute path: {path}")
        r, g, b = look["colors"][part]
        mat = spec.add_material(name=material_name(part), rgba=[r, g, b, 1.0],
                                specular=look["specular"][part], shininess=look["shininess"][part])
        mesh = spec.add_mesh(name=mesh_name(part))
        mesh.file = str(path)
        mesh.scale = [s, s, s]
        geom = body.add_geom(name=geom_name(part))
        geom.type = mujoco.mjtGeom.mjGEOM_MESH
        geom.meshname = mesh_name(part)
        geom.material = material_name(part)
        geom.contype = 0  # MuJoCo's default is 1: set explicitly, no collision
        geom.conaffinity = 0
        geom.group = GT_GROUP
        geom.density = 0.0  # no mass, whatever the compiler infers
        added.append({"part": part, "geom": geom_name(part), "mesh": mesh_name(part), "material": material_name(part), "file": str(path),
                      "rgba": [r, g, b, 1.0]})
    if not added:
        raise ValueError("no part was added")
    return {"body": body_name, "scale": s, "added": added}


def hide_present_drawing(spec, body_name: str = DRONE_BODY) -> dict:
    """Set the alpha of the geom rgba of every drawn mesh geom of the present drone to 0 (the body and the bodies below it).

    A geom is a drawn mesh geom if it names a mesh and is not one of ours. The collision geoms (col_sphere, col_box) are spheres
    and boxes without a mesh and are not touched."""
    root = spec.body(body_name)
    if root is None:
        raise ValueError(f"body {body_name!r} not found in the spec")
    hidden, kept = [], []

    def visit(b):
        for g in b.geoms:
            name = g.name or ""
            if name.startswith(GT_PREFIX):
                continue
            if g.meshname:
                g.rgba[3] = 0.0  # in place, as Crazyflow does for its collision geoms (sim.py line 782)
                if g.rgba[3] != 0.0:
                    g.rgba = [float(g.rgba[0]), float(g.rgba[1]), float(g.rgba[2]), 0.0]
                hidden.append(name or g.meshname)
            else:
                kept.append(name or "(unnamed)")
        for c in b.bodies:
            visit(c)

    visit(root)
    if not hidden:
        raise ValueError("no drawn mesh geom found on the present drone: the model has another structure than expected")
    return {"hidden": hidden, "kept_without_mesh": kept}


# ---------------------------------------------------------------------------------------------
# The environment: the pad environment of tools/track_env.py plus the GT drawing
# ---------------------------------------------------------------------------------------------

_ENV_CLASS = None


def gt_env_class():
    """GTDroneRaceEnv (made on first use, because it needs lsy_drone_racing and gymnasium)."""
    global _ENV_CLASS
    if _ENV_CLASS is not None:
        return _ENV_CLASS
    import mujoco
    import track_env

    class GTDroneRaceEnv(track_env.PadDroneRaceEnv):
        """PadDroneRaceEnv whose drone is drawn as the GT model. Extra keyword argument `gt`: the dict of make_look (or None)."""

        def __init__(self, *args, gt: dict | None = None, **kwargs):
            self._gt = dict(gt) if gt else None  # before the parent __init__, which calls _load_track_into_sim
            self.gt_info = None
            super().__init__(*args, **kwargs)

        def _load_track_into_sim(self, track):
            """Edit the spec first (GT shapes, hide the present drawing), then let the pad environment and LSY attach pads,
            gates and poles and compile. The two later compiles (use_box_collision) start from the same spec."""
            if self._gt:
                added = add_gt_drone(self.sim.spec, mujoco, self._gt)
                hidden = hide_present_drawing(self.sim.spec)
                self.gt_info = {**added, **hidden, "look": self._gt["look"]}
            super()._load_track_into_sim(track)

    _ENV_CLASS = GTDroneRaceEnv
    return _ENV_CLASS


def register_gt_env() -> str:
    """Register the id of the GT environment (same settings as track_env.register_pad_env and LSY's registration)."""
    import gymnasium

    gymnasium.register(
        id=GT_ENV_ID,
        entry_point=gt_env_class(),
        max_episode_steps=1500,  # same as lsy_drone_racing/envs/__init__.py
        disable_env_checker=True,  # same as lsy_drone_racing/envs/__init__.py
    )
    return GT_ENV_ID


# ---------------------------------------------------------------------------------------------
# Reading the compiled model back
# ---------------------------------------------------------------------------------------------


def quat_to_matrix(q) -> np.ndarray:
    """MuJoCo quaternion (w, x, y, z) -> 3x3 rotation matrix."""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _name(mujoco, m, kind, idx) -> str:
    return mujoco.mj_id2name(m, kind, int(idx)) or ""


def drone_body_ids(mujoco, m, body_name: str = DRONE_BODY) -> list:
    """Id of the drone body and of every body below it."""
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if bid < 0:
        raise ValueError(f"body {body_name!r} not in the compiled model")
    ids = [bid]
    for b in range(m.nbody):
        p = int(m.body_parentid[b])
        while b != bid:
            if p == bid:
                ids.append(b)
                break
            if p == 0:
                break
            p = int(m.body_parentid[p])
    return ids


def geoms_of(m, body_id: int) -> list:
    first, count = int(m.body_geomadr[body_id]), int(m.body_geomnum[body_id])
    return list(range(first, first + count))


def model_fingerprint(mujoco, m) -> dict:
    """Numbers that physics and collision use, from the compiled model, in plain Python types (exact in JSON).

    bodies: name -> mass, inertia (3), ipos (3), iquat (4), pos (3), quat (4), subtree mass.
    colliders: every geom of the whole model with contype or conaffinity different from 0, in model order, by body name and
    geom fields (no geom id: ids move when geoms are added)."""
    bodies = {}
    for b in range(m.nbody):
        bodies[_name(mujoco, m, mujoco.mjtObj.mjOBJ_BODY, b) or f"body{b}"] = {
            "mass": float(m.body_mass[b]), "inertia": [float(v) for v in m.body_inertia[b]],
            "ipos": [float(v) for v in m.body_ipos[b]], "iquat": [float(v) for v in m.body_iquat[b]],
            "pos": [float(v) for v in m.body_pos[b]], "quat": [float(v) for v in m.body_quat[b]],
            "subtree_mass": float(m.body_subtreemass[b]),
        }
    colliders = []
    for g in range(m.ngeom):
        if int(m.geom_contype[g]) == 0 and int(m.geom_conaffinity[g]) == 0:
            continue
        colliders.append({
            "body": _name(mujoco, m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]), "name": _name(mujoco, m, mujoco.mjtObj.mjOBJ_GEOM, g),
            "type": int(m.geom_type[g]), "size": [float(v) for v in m.geom_size[g]], "pos": [float(v) for v in m.geom_pos[g]],
            "quat": [float(v) for v in m.geom_quat[g]], "contype": int(m.geom_contype[g]), "conaffinity": int(m.geom_conaffinity[g]),
            "condim": int(m.geom_condim[g]), "priority": int(m.geom_priority[g]), "friction": [float(v) for v in m.geom_friction[g]],
            "solref": [float(v) for v in m.geom_solref[g]], "solimp": [float(v) for v in m.geom_solimp[g]],
            "solmix": float(m.geom_solmix[g]), "margin": float(m.geom_margin[g]), "gap": float(m.geom_gap[g]),
        })
    drone_ids = drone_body_ids(mujoco, m)
    return {
        "counts": {"nbody": int(m.nbody), "ngeom": int(m.ngeom), "nmesh": int(m.nmesh), "nmat": int(m.nmat), "njnt": int(m.njnt),
                   "nmocap": int(m.nmocap), "ncam": int(m.ncam), "nsite": int(m.nsite)},
        "timestep": float(m.opt.timestep),
        "bodies": bodies, "colliders": colliders,
        "drone_bodies": [_name(mujoco, m, mujoco.mjtObj.mjOBJ_BODY, b) for b in drone_ids],
        "drone_geoms": [_name(mujoco, m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}" for b in drone_ids for g in geoms_of(m, b)],
        "camera_fpv": {"pos": [float(v) for v in m.cam_pos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "fpv_cam:0")]],
                       "quat": [float(v) for v in m.cam_quat[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "fpv_cam:0")]],
                       "fovy": float(m.cam_fovy[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "fpv_cam:0")])},
    }


def compare_fingerprints(a: dict, b: dict) -> dict:
    """Exact comparison of two fingerprints. Returns {name: (equal, detail)} for the checks the test reports."""
    out = {}
    names_a, names_b = sorted(a["bodies"]), sorted(b["bodies"])
    same_names = names_a == names_b
    diff = [n for n in names_a if n in b["bodies"] and a["bodies"][n] != b["bodies"][n]]
    out["bodies_mass_inertia_com"] = (same_names and not diff,
                                      f"{len(names_a)} bodies against {len(names_b)}; bodies with any difference in mass, inertia, center of mass, "
                                      f"position, orientation or subtree mass: {diff or 'none'}")
    out["collision_shapes"] = (a["colliders"] == b["colliders"],
                               f"{len(a['colliders'])} colliding geoms against {len(b['colliders'])}; all fields (type, size, pos, quat, contype, "
                               f"conaffinity, condim, priority, friction, solref, solimp, solmix, margin, gap) compared exactly, in model order")
    drone_a = [c for c in a["colliders"] if c["body"].startswith("drone")]
    drone_b = [c for c in b["colliders"] if c["body"].startswith("drone")]
    out["drone_collision_shapes"] = (drone_a == drone_b and len(drone_a) > 0, f"{len(drone_a)} against {len(drone_b)} colliding geoms on the drone: "
                                     + ", ".join(f"{c['name']} (type {c['type']}, contype {c['contype']})" for c in drone_a))
    out["camera_and_step"] = (a["camera_fpv"] == b["camera_fpv"] and a["timestep"] == b["timestep"], "fpv camera pose and field of view, and the time step")
    return out


def read_back(mujoco, np_, m, look: dict, body_name: str = DRONE_BODY) -> dict:
    """Check the compiled model against the look. Returns {"ok": bool, "problems": [...], ...numbers}."""
    problems = []
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
    parts = [p for p in look["mesh_files"] if p not in look.get("skip", ())]
    s = float(look["scale"])
    drawn_pts = []
    rows = []
    for part in parts:
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, geom_name(part))
        if g < 0:
            problems.append(f"geom {geom_name(part)} not in the compiled model")
            continue
        mat = int(m.geom_matid[g])
        want = np.array(look["colors"][part] + [1.0], dtype=np.float32)
        got = np.array(m.mat_rgba[mat], dtype=np.float32) if mat >= 0 else None
        if int(m.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_MESH):
            problems.append(f"{geom_name(part)} is not a mesh geom")
        if int(m.geom_bodyid[g]) != bid:
            problems.append(f"{geom_name(part)} is not on {body_name}")
        if int(m.geom_contype[g]) != 0 or int(m.geom_conaffinity[g]) != 0:
            problems.append(f"{geom_name(part)} has collision bits {int(m.geom_contype[g])}/{int(m.geom_conaffinity[g])}")
        if int(m.geom_group[g]) != GT_GROUP:
            problems.append(f"{geom_name(part)} is in group {int(m.geom_group[g])}")
        if got is None or _name(mujoco, m, mujoco.mjtObj.mjOBJ_MATERIAL, mat) != material_name(part) or float(np.abs(got - want).max()) > MAT_TOL:
            problems.append(f"{geom_name(part)}: material color {None if got is None else got.tolist()} differs from {want.tolist()}")
        mesh = int(m.geom_dataid[g])
        a, n = int(m.mesh_vertadr[mesh]), int(m.mesh_vertnum[mesh])
        pts = quat_to_matrix(m.geom_quat[g]) @ np.asarray(m.mesh_vert[a:a + n], dtype=np.float64).T + np.asarray(m.geom_pos[g], dtype=np.float64)[:, None]
        drawn_pts.append(pts.T)
        rows.append({"part": part, "geom": g, "contype": int(m.geom_contype[g]), "conaffinity": int(m.geom_conaffinity[g]), "group": int(m.geom_group[g]),
                     "rgba": got.tolist() if got is not None else None, "vertices": n})
    extent = None
    if drawn_pts:
        p = np.concatenate(drawn_pts)
        lo, hi = p.min(axis=0), p.max(axis=0)
        extent = {"min": lo.tolist(), "max": hi.tolist()}
        tol = EXTENT_TOL * max(s, 1.0)
        want_lo = np.array(look["obj_extent"]["min"]) * s
        want_hi = np.array(look["obj_extent"]["max"]) * s
        if not look.get("skip") and (np.abs(lo - want_lo).max() > tol or np.abs(hi - want_hi).max() > tol):
            problems.append(f"drawn extent {lo.tolist()}..{hi.tolist()} differs from the OBJ extent times scale {s}: {want_lo.tolist()}..{want_hi.tolist()}")
    # the present drone's drawn mesh geoms must be invisible (geom alpha 0), ours visible
    still_visible = []
    for b in drone_body_ids(mujoco, m, body_name):
        for g in geoms_of(m, b):
            name = _name(mujoco, m, mujoco.mjtObj.mjOBJ_GEOM, g)
            if int(m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_MESH) and not name.startswith(GT_PREFIX):
                if float(m.geom_rgba[g][3]) != 0.0:
                    still_visible.append(name or f"geom{g}")
    if still_visible:
        problems.append(f"present drone mesh geoms still visible: {still_visible}")
    return {"ok": not problems, "problems": problems, "geoms": rows, "drawn_extent": extent, "scale": s,
            "present_mesh_geoms_hidden": not still_visible}


# ---------------------------------------------------------------------------------------------
# Self-test of the spec logic with a stand-in for the MuJoCo spec (no simulator needed)
# ---------------------------------------------------------------------------------------------


class _Arr(list):
    """List that also accepts item assignment, like the NumPy view the real binding returns for rgba."""


class _FakeGeom:
    def __init__(self, name="", meshname="", rgba=(0.5, 0.5, 0.5, 1.0), gtype="sphere"):
        self.name, self.meshname, self.rgba, self.type = name, meshname, _Arr(rgba), gtype
        self.contype, self.conaffinity, self.group, self.density, self.material = 1, 1, 0, 1000.0, ""


class _FakeBody:
    def __init__(self, name):
        self.name, self.geoms, self.bodies = name, [], []

    def add_geom(self, name=""):
        g = _FakeGeom(name=name)
        if any(x.name == name and name for x in self.geoms):
            raise ValueError("repeated name")
        self.geoms.append(g)
        return g


class _FakeObj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _FakeSpec:
    def __init__(self):
        self.root = _FakeBody("drone:0")
        self.root.geoms = [_FakeGeom("", "pcb", gtype="mesh"), _FakeGeom("", "motors", gtype="mesh"), _FakeGeom("led_top", "led-diffuser", gtype="mesh"),
                           _FakeGeom("col_sphere:0", gtype="sphere"), _FakeGeom("col_box:0", gtype="box")]
        prop = _FakeBody("prop0:0")
        prop.geoms = [_FakeGeom("", "propL", gtype="mesh")]
        self.root.bodies = [prop]
        self.meshes, self.materials = [], []

    def body(self, name):
        return self.root if name == self.root.name else None

    def add_mesh(self, name=""):
        m = _FakeObj(name=name, file="", scale=[1, 1, 1])
        self.meshes.append(m)
        return m

    def add_material(self, name="", rgba=None, specular=0.0, shininess=0.0):
        m = _FakeObj(name=name, rgba=rgba, specular=specular, shininess=shininess)
        self.materials.append(m)
        return m


def self_test(work_dir=None) -> int:
    """Run the spec edits on a stand-in spec and check what they must do. Every check can fail (see the last two)."""
    import tempfile

    results = []

    def check(name, ok, detail):
        results.append((name, bool(ok)))
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")

    fake_mujoco = _FakeObj(mjtGeom=_FakeObj(mjGEOM_MESH="mesh"))
    with tempfile.TemporaryDirectory(prefix="drone_look_") as tmp:
        look = make_look("magenta", 2.0, Path(work_dir or tmp) / "meshes")
        spec = _FakeSpec()
        n_old = len(spec.root.geoms)
        added = add_gt_drone(spec, fake_mujoco, look)
        hid = hide_present_drawing(spec)
        gt = [g for g in spec.root.geoms if g.name.startswith(GT_PREFIX)]
        check("one_geom_mesh_material_per_part", len(gt) == len(look["mesh_files"]) == len(spec.meshes) == len(spec.materials) == len(added["added"]),
              f"{len(gt)} geoms, {len(spec.meshes)} meshes, {len(spec.materials)} materials for {len(look['mesh_files'])} parts")
        check("added_geoms_have_no_collision_and_no_mass", all(g.contype == 0 and g.conaffinity == 0 and g.density == 0.0 and g.group == GT_GROUP and g.type == "mesh" for g in gt),
              "contype 0, conaffinity 0, density 0, group 2, type mesh on every added geom")
        check("scale_and_files_passed_to_the_meshes", all(m.scale == [2.0, 2.0, 2.0] and Path(m.file).is_absolute() and Path(m.file).is_file() for m in spec.meshes),
              "scale [2, 2, 2] and an existing absolute file on every mesh")
        check("colors_come_from_the_mtl", {m.name: tuple(m.rgba[:3]) for m in spec.materials}["gt_mat_body"] == (0.667, 1.0, 0.0), "gt_mat_body is the magenta look's lime (0.667, 1.0, 0.0)")
        old = spec.root.geoms[:n_old] + spec.root.bodies[0].geoms
        mesh_old = [g for g in old if g.meshname]
        check("present_mesh_geoms_hidden", all(g.rgba[3] == 0.0 for g in mesh_old) and len(hid["hidden"]) == len(mesh_old) == 4,
              f"hidden: {hid['hidden']}")
        check("present_collision_geoms_untouched", all(g.rgba[3] == 1.0 and g.contype == 1 for g in old if not g.meshname) and len(hid["kept_without_mesh"]) == 2,
              f"kept: {hid['kept_without_mesh']}")
        names_ok = not any(g.name.startswith(("col_sphere", "col_box")) for g in gt)
        check("no_added_name_is_matched_by_use_box_collision", names_ok, "none of the added geom names starts with col_sphere or col_box (sim.py lines 778-785)")
        # the checks must be able to fail
        bad = _FakeSpec()
        try:
            add_gt_drone(bad, fake_mujoco, {**look, "mesh_files": {"body": str(Path(tmp) / "missing.stl")}})
            raised = False
        except FileNotFoundError:
            raised = True
        check("meta_missing_mesh_file_is_refused", raised, "add_gt_drone raises for a mesh file that does not exist")
        try:
            hide_present_drawing(_EmptyFakeSpec())
            raised = False
        except ValueError:
            raised = True
        check("meta_model_without_drawn_meshes_is_refused", raised, "hide_present_drawing raises when it finds no mesh geom to hide")
        a = {"bodies": {"drone:0": {"mass": 0.0434}}, "colliders": [{"body": "drone:0", "name": "col_box:0", "type": 6, "contype": 1}], "camera_fpv": 1, "timestep": 0.02}
        b = json.loads(json.dumps(a))
        same = compare_fingerprints(a, b)
        b["bodies"]["drone:0"]["mass"] = 0.0434 + 1e-12
        diff = compare_fingerprints(a, b)
        check("meta_fingerprint_comparison_sees_a_mass_change_of_1e-12", all(v[0] for v in same.values()) and not diff["bodies_mass_inertia_com"][0], "equal when equal, different when the mass differs by 1e-12")
    failed = [n for n, ok in results if not ok]
    print(f"{'PASS' if not failed else 'FAIL'}: {len(results) - len(failed)} of {len(results)} checks passed" + (f"; failed: {failed}" if failed else ""))
    return 0 if not failed else 1


class _EmptyFakeSpec(_FakeSpec):
    def __init__(self):
        super().__init__()
        self.root.geoms = [_FakeGeom("col_box:0", gtype="box")]
        self.root.bodies = []


if __name__ == "__main__":
    import sys

    sys.exit(self_test())
