"""LSY's single-drone racing environment with the pads (tools/track_env.py) and the floor and sky of a look.

`LookDroneRaceEnv` is a subclass of `PadDroneRaceEnv` (which is a subclass of `DroneRaceEnv`); no existing file is changed.
Extra keyword argument `look`: "today" (floor and sky stay as crazyflow's scene.xml has them) or magenta, cyan, red, violet.

What is changed, and where it is set in the sources (all read; crazyflow commit d28ec70, MuJoCo 3.10.0, lsy_drone_racing
commit ac7377b; the files are in runs/2026-10-12_gate_finder_sources and runs/2026-10-13_looks_sources):

FLOOR  crazyflow/scene.xml line 15-16 makes the 2d texture "groundplane": builtin checker, mark edge, rgb1 and rgb2 (two blues),
       markrgb (light grey edge lines), 512 x 512 pixels. Line 17 makes the material "groundplane": texuniform true, texrepeat
       2 2, reflectance 0.2. Line 22 is the geom "floor": a plane with size "0 0 0.05" (infinite).
       The reflectance 0.2 is what mirrors the gates (render_gl3.c lines 45-49, isReflective: a plane with reflectance > 0 is reflective).
       The spec of the simulation is built from this file (crazyflow sim.py lines 139-142, 287-312) and then written to XML and
       read back (sim.py lines 315-346, `_replicate_drone`), so the spec still has the texture and the material under the same
       names. `RaceCoreEnv._load_track_into_sim` ends with `self.sim.build_mjx()` (race_core.py line 701; sim.py lines
       504-509), which compiles `self.sim.spec`. This class edits that spec before that compile (override of
       `_load_track_into_sim`, as the pads do), through the MjSpec fields rgb1, rgb2, mark (mjspec.h mjsTexture, lines 578-612)
       and emission, specular, shininess, reflectance (mjsMaterial, lines 615-629):
         rgb1, rgb2 = the look's floor colors A and B     mark = none (no edge lines)     reflectance = 0 (no mirror)
         emission = 0, specular = 0, shininess = 0. The floor shows the picture colors times the light sum, and `apply_lights_to_model`
         makes that sum 1.
       Why not emission: the first cloud run (results in runs/2026-10-12_cloud_test/run_38500349252) drew the floor 1.70 times brighter
       than its picture, for emission 0 (today) and for emission 1 (the new looks) alike: the emission had no visible effect. The
       sources do set GL_EMISSION = emission * rgba (render_gl3.c lines 253-259) and GL_MODULATE (line 646), so I cannot explain
       from the source why it made no difference; the measured gain is the fact. The look's lights are therefore set so that the
       sum is 1 (see LIGHTS below) and the emission is 0.
       Squares: texrepeat and texuniform stay as they are (2 2, true). For an infinite plane the texture is mapped with
       S = 0.5 * texrepeat * x - 0.5 (render_gl3.c lines 122-123 take texrepeat, lines 135-143 multiply by size only when size > 0, lines
       146-149 set S and T; the grid of an infinite plane is in meters, render_context.c lines 147-166),
       so one picture covers 2 / texrepeat = 1 m, and the builtin checker is 2 x 2 squares (user_objects.cc `checker`,
       lines 5012-5037, and `Builtin2D`, line 5043 onward): a square is 1 / texrepeat = 0.5 m. `floor_square_m` computes this from the compiled model.
       Exact colors: MuJoCo makes the byte as (std::byte)(255 * rgb) (user_objects.cc line 5043 `RGB1[j] = (std::byte)(255*rgb1[j])`),
       which truncates. For all 256 values v / 255 happens to give v in double precision (tested), but that depends on
       rounding, so the spec gets (v + 0.5) / 255 and the byte is v whatever the rounding.

SKY    crazyflow/scene.xml line 14: skybox, builtin gradient, rgb1 (0.3 0.5 0.7), rgb2 (0 0 0), 512 wide, 3072 high (six faces of
       512 x 512). The gradient (user_objects.cc BuiltinCube lines 5112-5153 and `interp` lines 4995-5008) is
       color = alpha * rgb1 + (1 - alpha) * rgb2 with alpha = 0.5 at the horizon, 1 at the zenith. The horizon is therefore the
       MIDDLE of rgb1 and rgb2, and "horizon = skyH, top = skyT" needs rgb2 = 2 * skyH - skyT, which is above 1 (impossible) for
       magenta, red and violet. So the spec cannot give these skies, and the compiled model is changed instead:
       `apply_sky_to_model` rewrites the 6 x 512 x 512 pixels in `mjModel.tex_data` (mjmodel.h lines 682-689) with the same
       geometry as the builtin and the colors horizon -> top, linear in elevation. The renderer reads the pixels when it makes the
       OpenGL context (render_context.c makeTexture lines 1341-1358, mjr_uploadTexture lines 1364-1447), i.e. at the first render,
       so the change must be made before the first picture and it stays after sim.close() (the model is kept). The skybox is drawn
       without light and with white color (render_gl3.c lines 1309-1338: GL_LIGHTING off, glColor4f(1,1,1,1)), so the picture
       shows the pixel colors.

LIGHTS The drawn color of a surface is picture color times (emission + light sum), where the fixed-function pipeline adds, for each
       light, ambient + diffuse * cos(angle between normal and light direction) (light colors set in render_gl3.c `initLights`,
       lines 662-677; the material color follows glColor for ambient and diffuse, lines 655-656 and 255-259). The lights come from
       mjvScene lights that `mjv_updateScene` fills at EVERY frame from the model: the headlight from `m.vis.headlight`
       (visualize.c lines 2880-2904) and every model light from `m.light_ambient`, `m.light_diffuse`, `m.light_specular` (lines
       2932-2934; mjmodel.h lines 528-530). Scene values (crazyflow scene.xml line 7 and line 21): headlight ambient 0.5, diffuse
       0.6; one directional light pointing straight down with MuJoCo's default diffuse 0.7. A floor seen from straight above has
       cos = 1 for both, so the sum is 0.5 + 0.6 + 0.7 = 1.8 (measured gain: 1.70, the cause of the gap of 0.1 is not known).
       `apply_lights_to_model` sets, for the new looks only, on the compiled model (read at every frame, so no upload and no
       spec edit are needed; the readback in `read_lights` proves it): headlight ambient 0.75, diffuse 0.10, specular 0; every
       directional light ambient 0, diffuse 0.15, specular 0. Sum for a floor from above: 0.75 + 0.10 + 0.15 = 1.00; a wall seen
       head on: 0.75 + 0.10 = 0.85; nothing is brighter than 1.00 and the specular light is off. `scale_lights` multiplies the
       ambient and diffuse values by one factor; the cloud test uses it once to remove any gap that remains (a measured gain
       other than 1 on the floor), and records the factor.

Pure helper functions come first; they take `np` (numpy) and `m` (a compiled mjModel or anything with the same fields), so the
local tests can run them on a fake model.
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import gymnasium  # noqa: E402,F401  (track_env needs it; imported here so a missing install fails at import)
import look_assets as la  # noqa: E402
import track_env  # noqa: E402

LOOK_ENV_ID = "DroneRacingLook-v0"
FLOOR_TEXTURE = "groundplane"  # crazyflow scene.xml line 15
FLOOR_MATERIAL = "groundplane"  # crazyflow scene.xml line 17
FLOOR_GEOM = "floor"  # crazyflow scene.xml line 22
# Integer values of enums (mjspec.h lines 86-99, mjtype.h lines 151-155), used as numbers so no enum name has to exist in Python.
BUILTIN_CHECKER, MARK_NONE = 2, 0
TEXTURE_2D, TEXTURE_SKYBOX = 0, 2
TEXROLE_RGB = 1  # mjtTextureRole (mjtype.h line 160); mat_texid has one column per role
SQUARE_M = 0.5  # the look's floor squares (colors.txt)
SKY_FACES = 6
LIGHT_DIRECTIONAL = 1  # mjtLightType (mjtype.h lines 144-147)


def texel_float(v255: int) -> float:
    """Float for rgb1/rgb2 of a builtin texture whose byte after MuJoCo's truncation is exactly v255."""
    return (int(v255) + 0.5) / 255.0


def _by_name(items, name: str):
    found = [x for x in items if x.name == name]
    if len(found) != 1:
        raise ValueError(f"expected exactly one element named {name!r} in the spec, found {len(found)} (names: {[x.name for x in items][:10]})")
    return found[0]


def edit_floor_spec(spec, look: str) -> dict:
    """Edit the MjSpec `spec` (before it is compiled): floor colors, no edge lines, no mirror, flat light. Returns what was set."""
    la.check_look(look)
    if look == "today":
        return {"look": look, "changed": False}
    a, b = la.palette(look)["floor"]
    tex = _by_name(spec.textures, FLOOR_TEXTURE)
    mat = _by_name(spec.materials, FLOOR_MATERIAL)
    if int(tex.builtin) != BUILTIN_CHECKER:
        raise ValueError(f"texture {FLOOR_TEXTURE!r} is not a builtin checker (builtin = {int(tex.builtin)})")
    tex.rgb1 = [texel_float(v) for v in a]
    tex.rgb2 = [texel_float(v) for v in b]
    tex.mark = MARK_NONE
    mat.reflectance = 0.0
    mat.emission = 0.0  # the look's lights add up to 1 (see apply_lights_to_model); emission would add to that
    mat.specular = 0.0
    mat.shininess = 0.0
    return {"look": look, "changed": True, "rgb1": list(a), "rgb2": list(b), "mark": "none", "reflectance": 0.0, "emission": 0.0}


# ---------------------------------------------------------------------------------------------
# Read the floor back from a compiled model
# ---------------------------------------------------------------------------------------------


def floor_ids(mujoco, np, m) -> dict:
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, FLOOR_GEOM)
    if gid < 0:
        raise ValueError(f"geom {FLOOR_GEOM!r} not found in the compiled model")
    mid = int(m.geom_matid[gid])
    if mid < 0:
        raise ValueError("the floor geom has no material")
    row = np.asarray(m.mat_texid).reshape(int(m.nmat), -1)[mid]
    ids = [int(t) for t in row if int(t) >= 0]
    if len(ids) != 1:
        raise ValueError(f"the floor material has {len(ids)} textures, expected 1")
    return {"geom": int(gid), "material": mid, "texture": ids[0]}


def floor_square_m(np, m, ids: dict) -> float:
    """Side of one floor square in meters, from the compiled model (see the module text for the sources of the formula).
    Valid for an infinite plane (size 0 0 ...) with a 2d texture of a 2 x 2 checker; raises otherwise."""
    size = np.asarray(m.geom_size).reshape(-1, 3)[ids["geom"]]
    if size[0] != 0 or size[1] != 0:
        raise ValueError(f"the floor plane is not infinite (size {size.tolist()}); the formula does not hold")
    if int(m.tex_type[ids["texture"]]) != TEXTURE_2D:
        raise ValueError("the floor texture is not 2d")
    rep = np.asarray(m.mat_texrepeat).reshape(-1, 2)[ids["material"]]
    if not bool(np.asarray(m.mat_texuniform).reshape(-1)[ids["material"]]):
        raise ValueError("texuniform is false: the formula does not hold")
    if rep[0] != rep[1] or rep[0] <= 0:
        raise ValueError(f"texrepeat {rep.tolist()} is not square and positive")
    return float((2.0 / float(rep[0])) / 2.0)


def read_floor(mujoco, np, m) -> dict:
    """What the compiled model has for the floor: reflectance, emission, the colors of the picture, the square size."""
    ids = floor_ids(mujoco, np, m)
    tid = ids["texture"]
    w, h, nch = int(m.tex_width[tid]), int(m.tex_height[tid]), int(m.tex_nchannel[tid])
    adr = int(m.tex_adr[tid])
    pix = np.asarray(m.tex_data[adr:adr + w * h * nch]).reshape(h, w, nch)[:, :, :3]
    colors = np.unique(pix.reshape(-1, 3), axis=0)
    return {
        **ids,
        "reflectance": float(np.asarray(m.mat_reflectance).reshape(-1)[ids["material"]]),
        "emission": float(np.asarray(m.mat_emission).reshape(-1)[ids["material"]]),
        "texture_size": [w, h], "nchannel": nch,
        "corner_colors": {"top_left": pix[0, 0].astype(int).tolist(), "top_right": pix[0, w - 1].astype(int).tolist(),
                          "bottom_left": pix[h - 1, 0].astype(int).tolist(), "bottom_right": pix[h - 1, w - 1].astype(int).tolist()},
        "unique_colors": [c.astype(int).tolist() for c in colors[:8]], "n_unique_colors": int(len(colors)),
        "square_m": floor_square_m(np, m, ids),
        "texrepeat": np.asarray(m.mat_texrepeat).reshape(-1, 2)[ids["material"]].astype(float).tolist(),
    }


def floor_matches_look(np, floor: dict, look: str) -> tuple:
    """(ok, detail): exactly the two colors A and B in the picture, A at top-left and bottom-right, B at the other two corners."""
    a, b = (list(c) for c in la.palette(look)["floor"])
    cc = floor["corner_colors"]
    ok = (cc["top_left"] == a and cc["bottom_right"] == a and cc["top_right"] == b and cc["bottom_left"] == b
          and floor["n_unique_colors"] == 2 and sorted(floor["unique_colors"]) == sorted([a, b]))
    return ok, f"look A {a} B {b}; picture corners {cc}; {floor['n_unique_colors']} different colors in the picture"


# ---------------------------------------------------------------------------------------------
# Sky
# ---------------------------------------------------------------------------------------------


def elevation_grid(np, w: int) -> dict:
    """Normalized elevation (0 horizon, 1 zenith, -1 nadir) of every pixel of one cube face, for the side faces and the up face.
    Same formulas as user_objects.cc BuiltinCube lines 5133-5144 (x, y in [-1, 1], row 0 on top)."""
    c = np.arange(w, dtype=float)
    x = 2.0 * c[None, :] / (w - 1) - 1.0
    y = 1.0 - 2.0 * c[:, None] / (w - 1)
    x, y = np.broadcast_arrays(x, y)
    root = np.sqrt(1.0 + x * x + y * y)
    side = np.arcsin(y / root) / (0.5 * math.pi)
    up = 1.0 - np.arccos(1.0 / root) / (0.5 * math.pi)
    return {"side": side, "up": up}


def sky_pixels(np, w: int, horizon255, top255):
    """New skybox pixels, shape (6, w, w, 3) uint8, faces in the order of the cube map (right, left, up, down, front, back;
    user_objects.cc BuiltinCube lines 5145-5153). Color = horizon + (top - horizon) * elevation for elevation >= 0, horizon below."""
    el = elevation_grid(np, w)
    hz, tp = np.asarray(horizon255, dtype=float), np.asarray(top255, dtype=float)

    def paint(e):
        t = np.clip(e, 0.0, 1.0)[:, :, None]
        return np.rint(hz[None, None, :] + (tp - hz)[None, None, :] * t).astype(np.uint8)

    faces = [paint(el["side"]), paint(el["side"]), paint(el["up"]), paint(-el["up"]), paint(el["side"]), paint(el["side"])]
    return np.stack(faces)


def sky_texture_id(np, m) -> int:
    ids = [i for i in range(int(m.ntex)) if int(m.tex_type[i]) == TEXTURE_SKYBOX]
    if len(ids) != 1:
        raise ValueError(f"expected one skybox texture, found {len(ids)}")
    return ids[0]


def sky_layout(np, m, tid: int) -> tuple:
    w, h, nch = int(m.tex_width[tid]), int(m.tex_height[tid]), int(m.tex_nchannel[tid])
    if nch != 3 or h != SKY_FACES * w:
        raise ValueError(f"unexpected skybox layout: {w} x {h}, {nch} channels (expected height = 6 * width, 3 channels)")
    return w, int(m.tex_adr[tid])


def apply_sky_to_model(np, m, look: str) -> dict:
    """Rewrite the skybox pixels of the compiled model `m`. Call before the first render (before the viewer exists)."""
    la.check_look(look)
    if look == "today":
        return {"look": look, "changed": False}
    p = la.palette(look)
    tid = sky_texture_id(np, m)
    w, adr = sky_layout(np, m, tid)
    pix = sky_pixels(np, w, p["skyH"], p["skyT"])
    m.tex_data[adr:adr + SKY_FACES * w * w * 3] = pix.reshape(-1)
    return {"look": look, "changed": True, "texture": tid, "face_width": w, "horizon": p["skyH"], "top": p["skyT"]}


def read_sky(np, m) -> dict:
    """The skybox pixels in the compiled model: the pixel next to the horizon on a side face and next to the zenith on the up face."""
    tid = sky_texture_id(np, m)
    w, adr = sky_layout(np, m, tid)
    pix = np.asarray(m.tex_data[adr:adr + SKY_FACES * w * w * 3]).reshape(SKY_FACES, w, w, 3)
    mid = w // 2 - 1  # rows w/2-1 and w/2 are the two rows next to the horizon (w is even)
    return {
        "texture": tid, "face_width": w,
        "horizon": pix[0, mid, mid].astype(int).tolist(),  # right face, middle
        "top": pix[2, mid, mid].astype(int).tolist(),  # up face, middle
        "bottom_edge_of_side_face": pix[0, w - 1, mid].astype(int).tolist(),
        "top_edge_of_side_face": pix[0, 0, mid].astype(int).tolist(),
    }


# ---------------------------------------------------------------------------------------------
# Lights
# ---------------------------------------------------------------------------------------------

LIGHT_VALUES = {  # the new looks: ambient, diffuse, specular
    "headlight": {"ambient": 0.75, "diffuse": 0.10, "specular": 0.0},
    "directional": {"ambient": 0.0, "diffuse": 0.15, "specular": 0.0},
}


def _lights_of(np, m) -> list:
    return list(range(int(m.nlight)))


def read_lights(np, m) -> dict:
    """Headlight and every model light as read from the compiled model, and the light sum for a surface seen from straight above
    (headlight ambient + diffuse, plus ambient + diffuse of each directional light pointing straight down; other lights counted
    in `other_lights`)."""
    hl = {k: [float(v) for v in np.asarray(getattr(m.vis.headlight, k)).reshape(-1)[:3]] for k in ("ambient", "diffuse", "specular")}
    amb = np.asarray(m.light_ambient).reshape(-1, 3)
    dif = np.asarray(m.light_diffuse).reshape(-1, 3)
    spe = np.asarray(m.light_specular).reshape(-1, 3)
    directional = np.asarray(m.light_type).reshape(-1) == LIGHT_DIRECTIONAL  # mjmodel.h light_type, mjtype.h mjLIGHT_DIRECTIONAL = 1
    dirs = np.asarray(m.light_dir).reshape(-1, 3)
    lights, total_other = [], 0
    for i in _lights_of(np, m):
        down = bool(directional[i]) and abs(float(dirs[i][2]) + 1.0) < 1e-6
        lights.append({"index": i, "directional": bool(directional[i]), "straight_down": down, "ambient": amb[i].tolist(),
                       "diffuse": dif[i].tolist(), "specular": spe[i].tolist()})
        total_other += 0 if down else 1
    top_sum = hl["ambient"][0] + hl["diffuse"][0] + sum(l["ambient"][0] + l["diffuse"][0] for l in lights if l["straight_down"])
    return {"headlight": hl, "lights": lights, "other_lights": total_other, "sum_from_above": float(top_sum)}


def apply_lights_to_model(np, m, look: str) -> dict:
    """Set the headlight and the directional lights of the compiled model for a new look (not for 'today')."""
    la.check_look(look)
    if look == "today":
        return {"look": look, "changed": False}
    h = LIGHT_VALUES["headlight"]
    for k, v in h.items():
        getattr(m.vis.headlight, k)[:] = [v, v, v]
    d = LIGHT_VALUES["directional"]
    arrays = {"ambient": m.light_ambient, "diffuse": m.light_diffuse, "specular": m.light_specular}
    directional = np.asarray(m.light_type).reshape(-1) == LIGHT_DIRECTIONAL
    for i in _lights_of(np, m):
        if directional[i]:
            for k, arr in arrays.items():
                np.asarray(arr).reshape(-1, 3)[i, :] = d[k]
    return {"look": look, "changed": True, "values": LIGHT_VALUES}


def scale_lights(np, m, factor: float) -> None:
    """Multiply ambient and diffuse of the headlight and of every model light by `factor` (specular is 0 and stays)."""
    for k in ("ambient", "diffuse"):
        getattr(m.vis.headlight, k)[:] = np.asarray(getattr(m.vis.headlight, k)).reshape(-1)[:3] * factor
    for arr in (m.light_ambient, m.light_diffuse):
        a = np.asarray(arr).reshape(-1, 3)
        a[:, :] = a * factor


# ---------------------------------------------------------------------------------------------
# The environment
# ---------------------------------------------------------------------------------------------


class LookDroneRaceEnv(track_env.PadDroneRaceEnv):
    """PadDroneRaceEnv whose floor has the look's two flat colors (no edge lines, no mirror).

    Extra keyword argument `look` (default "today": nothing changes). The sky is changed on the compiled model after the
    constructor, by `apply_sky_to_model` (see the module text for why it cannot be done on the spec).
    """

    def __init__(self, *args, look: str = "today", **kwargs):
        la.check_look(look)
        self._look_name = look  # must exist before the parent __init__ runs, which calls _load_track_into_sim
        self.floor_edit = None
        super().__init__(*args, **kwargs)

    def _load_track_into_sim(self, track):
        """Edit the floor in the spec, then let PadDroneRaceEnv attach the pads and LSY attach gates and poles and compile once."""
        self.floor_edit = edit_floor_spec(self.sim.spec, self._look_name)
        super()._load_track_into_sim(track)


def register_look_env() -> str:
    """Register the id of the look environment (same settings as LSY's registration, see track_env.py) and return the id."""
    import gymnasium as gym

    gym.register(
        id=LOOK_ENV_ID,
        entry_point=LookDroneRaceEnv,
        max_episode_steps=1500,  # same as lsy_drone_racing/envs/__init__.py
        disable_env_checker=True,  # same as lsy_drone_racing/envs/__init__.py
    )
    return LOOK_ENV_ID
