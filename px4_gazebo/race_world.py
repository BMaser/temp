"""The city around the hard race route, as SDF text and generated files (textures and meshes) for the PX4 + Gazebo race film FR.

Pure Python + numpy + Pillow (Pillow only inside write_assets); no Gazebo import, so tests/test_race_world.py can run it anywhere.
Everything is built from Gazebo's own primitives (box, cylinder, sphere) and from meshes and pictures that THIS file generates (no Fuel
download: the earlier runs died on a Fuel model that does not exist). Frame: world = frame F of race_core (x east, y north, z up, the spawn
point is the origin, the ground top is z = 0).

Segmentation labels (the Label system plugin sits inside every visual; 0 = nothing = sky):
  1 gate  2 pole  3 box  4 building  5 tree  6 bush  7 rock  8 car  9 truck  10 person  11 pad  12 hill or mountain  13 lamp post
  14 road  15 grass  16 sidewalk  17 water (the reflecting pool)
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import race_core as rc

WORLD = "baylands"          # the world keeps the name of the PX4 world file it replaces, so PX4's make target gz_x500_mono_cam_baylands needs no new configure
LABELS = {1: "gate", 2: "pole", 3: "box", 4: "building", 5: "tree", 6: "bush", 7: "rock", 8: "car", 9: "truck", 10: "person", 11: "pad",
          12: "hill / mountain", 13: "lamp post", 14: "road", 15: "grass", 16: "sidewalk", 17: "water"}
LABEL_COLORS = {0: (120, 170, 235), 1: (255, 140, 0), 2: (230, 40, 40), 3: (160, 90, 40), 4: (150, 150, 165), 5: (20, 130, 40), 6: (110, 200, 60),
                7: (110, 90, 70), 8: (230, 30, 120), 9: (40, 90, 230), 10: (255, 230, 0), 11: (255, 255, 255), 12: (110, 70, 150), 13: (0, 220, 220),
                14: (55, 55, 60), 15: (60, 100, 40), 16: (190, 190, 150), 17: (30, 60, 140)}

# the sun: low in the east-south-east, so the nose camera looks into it while the route heads east (glare); the moon of the night phase
SUN_AZ_DEG, SUN_EL_DEG = -8.0, 14.0
MOON_AZ_DEG, MOON_EL_DEG = 170.0, 38.0
SUN_DIST = 1000.0           # beyond the night dome (radius DOME_R), so the dome hides the sun
SUN_RADIUS = 48.0           # angular diameter about 5.5 degrees
DOME_R = 800.0
GROUND_HALF = 1500.0


def unit_from_az_el(az_deg, el_deg):
    a, e = math.radians(az_deg), math.radians(el_deg)
    return np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])


SUN_TO = unit_from_az_el(SUN_AZ_DEG, SUN_EL_DEG)     # unit vector from the scene to the sun
MOON_TO = unit_from_az_el(MOON_AZ_DEG, MOON_EL_DEG)


def f(v) -> str:
    return " ".join(f"{float(x):.4g}" if not isinstance(x, str) else x for x in v)


# ------------------------------------------------------------------------------------------------ lights (the settings of every phase)
LIGHTS = {
    # name: (type, day settings, night settings); a settings dict is the full gz.msgs.Light content
    "sun": dict(type="DIRECTIONAL", day=dict(diffuse=(1.0, 0.92, 0.80), specular=(0.5, 0.45, 0.35), intensity=1.0, direction=tuple(-SUN_TO)),
                night=dict(diffuse=(0.10, 0.13, 0.26), specular=(0.03, 0.04, 0.08), intensity=0.55, direction=tuple(-MOON_TO))),
    "fill": dict(type="DIRECTIONAL", day=dict(diffuse=(0.32, 0.38, 0.50), specular=(0.0, 0.0, 0.0), intensity=1.0, direction=(0.55, 0.25, -0.8)),
                 night=dict(diffuse=(0.0, 0.0, 0.0), specular=(0.0, 0.0, 0.0), intensity=0.0, direction=(0.55, 0.25, -0.8))),
}
LAMP_X = [-8.0, 6.0, 20.0, 34.0]
LAMP_Y = [-15.3, 15.3]
LAMPS = [(x, y) for y in LAMP_Y for x in LAMP_X]


def light_msg_text(name: str, kind: str, s: dict, extra: str = "") -> str:
    d, sp, dr = s["diffuse"], s["specular"], s["direction"]
    return (f'name: "{name}" type: {kind} diffuse {{r: {d[0]} g: {d[1]} b: {d[2]} a: 1}} specular {{r: {sp[0]} g: {sp[1]} b: {sp[2]} a: 1}} '
            f'intensity: {s["intensity"]} direction {{x: {dr[0]} y: {dr[1]} z: {dr[2]}}} cast_shadows: false {extra}')


def lamp_light_text(k: int, on: bool) -> str:
    s = dict(diffuse=(1.0, 0.85, 0.55), specular=(0.3, 0.3, 0.2), intensity=3.0 if on else 0.0, direction=(0, 0, -1))
    return light_msg_text(f"lamp_light_{k}", "POINT", s, "range: 18 attenuation_constant: 0.2 attenuation_linear: 0.1 attenuation_quadratic: 0.02")


def headlamp_text(on: bool) -> str:
    s = dict(diffuse=(1.0, 0.95, 0.8), specular=(0.5, 0.5, 0.4), intensity=4.0 if on else 0.0, direction=(1, 0, -0.25))
    return light_msg_text("headlamp", "SPOT", s, "range: 30 attenuation_constant: 0.3 attenuation_linear: 0.05 attenuation_quadratic: 0.002 "
                                                   "spot_inner_angle: 0.35 spot_outer_angle: 0.8 spot_falloff: 1.0")


# ------------------------------------------------------------------------------------------------ the world file (no plugin in it, on purpose)
def world_text(server_plugins: str = "") -> str:
    """The world. `server_plugins` = '' keeps PX4's server.config in charge (it is loaded when the world file has no plugin). A non-empty
    text is the full list of plugins (the copy of server.config plus our own), used when the plugins cannot be added to the running world."""
    sun, fill = LIGHTS["sun"]["day"], LIGHTS["fill"]["day"]

    def light(name, s):
        return (f"<light name='{name}' type='directional'><pose>0 0 500 0 0 0</pose><cast_shadows>false</cast_shadows><intensity>{s['intensity']}</intensity>"
                f"<direction>{f(s['direction'])}</direction><diffuse>{f(s['diffuse'])} 1</diffuse><specular>{f(s['specular'])} 1</specular>"
                f"<attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation></light>")
    return ("<?xml version='1.0'?><sdf version='1.9'><world name='" + WORLD + "'>"
            "<physics type='ode'><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics>"
            "<gravity>0 0 -9.8</gravity><magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field><atmosphere type='adiabatic'/>"
            "<scene><ambient>0.20 0.21 0.24 1</ambient><background>0.55 0.72 0.95 1</background><grid>false</grid><sky><clouds>true</clouds></sky><shadows>0</shadows></scene>"
            "<wind><linear_velocity>0 0 0</linear_velocity></wind>"
            + light("sun", sun) + light("fill", fill) +
            "<model name='ground_collision'><static>true</static><pose>0 0 -1 0 0 0</pose><link name='link'><collision name='c'><geometry><box><size>6000 6000 2</size></box></geometry>"
            "<surface><friction><ode/></friction></surface></collision></link></model>"
            + server_plugins +
            "<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>"
            "<latitude_deg>47.397971</latitude_deg><longitude_deg>8.546164</longitude_deg><elevation>0</elevation></spherical_coordinates>"
            "</world></sdf>")


# ------------------------------------------------------------------------------------------------ generated pictures and meshes
def _noise(rng, w, h, cells, Image):
    a = rng.random((cells, cells)).astype(np.float32)
    im = Image.fromarray((a * 255).astype(np.uint8)).resize((w, h), Image.BICUBIC)
    return np.asarray(im, dtype=np.float32) / 255.0


def _fbm(rng, w, h, Image, octaves=5, base=4):
    out = np.zeros((h, w), np.float32)
    amp, tot = 1.0, 0.0
    for o in range(octaves):
        out += amp * _noise(rng, w, h, base * 2 ** o, Image)
        tot += amp
        amp *= 0.5
    return out / tot


def make_textures(Image, ImageDraw, size=1024) -> dict:
    """name -> PIL image. Procedural pictures (numpy noise); nothing is downloaded."""
    rng = np.random.default_rng(20261016)
    tex = {}
    n = _fbm(rng, size, size, Image, 6, 4)
    sp = rng.random((size, size)).astype(np.float32)
    grass = np.zeros((size, size, 3), np.float32)
    base = np.array([62, 108, 40], np.float32)
    grass[:] = base * (0.65 + 0.7 * n[..., None])
    grass[..., 1] += 25 * (sp > 0.93)
    grass[..., :] -= 18 * (sp < 0.05)[..., None]
    tex["grass"] = Image.fromarray(np.clip(grass, 0, 255).astype(np.uint8))
    n = _fbm(rng, size, size, Image, 5, 8)
    asph = np.full((size, size, 3), 58.0, np.float32) * (0.8 + 0.4 * n[..., None])
    asph += 40 * (rng.random((size, size, 1)) > 0.985)
    tex["asphalt"] = Image.fromarray(np.clip(asph, 0, 255).astype(np.uint8))
    road = np.asarray(tex["asphalt"]).copy()
    r = Image.fromarray(road)
    d = ImageDraw.Draw(r)
    d.rectangle([0, int(size * 0.04), size, int(size * 0.04) + 14], fill=(235, 235, 235))
    d.rectangle([0, int(size * 0.96) - 14, size, int(size * 0.96)], fill=(235, 235, 235))
    d.rectangle([0, size // 2 - 8, int(size * 0.375), size // 2 + 8], fill=(240, 230, 120))   # dashed centre line: 3 m of 8 m
    tex["road"] = r
    n = _fbm(rng, size, size, Image, 5, 6)
    conc = np.full((size, size, 3), 176.0, np.float32) * (0.85 + 0.3 * n[..., None])
    conc[..., 2] *= 0.97
    c = Image.fromarray(np.clip(conc, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(c)
    for k in range(0, size, size // 4):
        d.line([(k, 0), (k, size)], fill=(110, 110, 105), width=4)
        d.line([(0, k), (size, k)], fill=(110, 110, 105), width=4)
    tex["concrete"] = c

    def facade(name, wall, win, cols, rows, lit=0.12, frame=None, w=512, h=1024):
        im = Image.new("RGB", (w, h), wall)
        dd = ImageDraw.Draw(im)
        cw, rh = w / cols, h / rows
        for i in range(cols):
            for j in range(rows):
                x0, y0 = i * cw + cw * 0.18, j * rh + rh * 0.20
                x1, y1 = (i + 1) * cw - cw * 0.18, (j + 1) * rh - rh * 0.20
                col = (255, 225, 140) if rng.random() < lit else win
                shade = 0.85 + 0.3 * rng.random()
                col = tuple(int(min(255, ch * shade)) for ch in col)
                dd.rectangle([x0, y0, x1, y1], fill=col, outline=frame)
        tex[name] = im
    facade("facade_beige", (205, 190, 160), (60, 80, 105), 7, 16)
    facade("facade_gray", (150, 152, 158), (45, 60, 80), 8, 18)
    facade("facade_brick", (150, 78, 62), (50, 62, 78), 6, 12)
    facade("facade_glass", (70, 95, 120), (120, 165, 205), 10, 28, lit=0.0, frame=(40, 55, 70))
    # night dome (equirectangular, mirrored about the horizon so that it is right whichever way the mesh loader flips v)
    W, H = 2048, 1024
    top = Image.new("RGB", (W, H // 2))
    arr = np.zeros((H // 2, W, 3), np.float32)
    t = np.linspace(0.0, 1.0, H // 2)[:, None]          # 0 at the zenith row, 1 at the horizon row
    zen, hor = np.array([3, 5, 16], np.float32), np.array([26, 34, 66], np.float32)
    arr[:] = (zen[None, None, :] * (1 - t[..., None]) + hor[None, None, :] * t[..., None])
    stars = rng.random((H // 2, W)) > 0.9985
    arr[stars] = 235
    cl = _fbm(rng, W, H // 2, Image, 6, 6)
    cm = np.clip((cl - 0.52) * 5.0, 0, 1) * (0.25 + 0.75 * (1 - t))
    cloud_col = np.array([70, 80, 112], np.float32)
    arr = arr * (1 - cm[..., None]) + cloud_col * cm[..., None]
    # the moon
    mu = (MOON_AZ_DEG % 360.0) / 360.0 * W
    mv = (1.0 - MOON_EL_DEG / 90.0) * (H // 2)
    yy, xx = np.mgrid[0:H // 2, 0:W]
    dist = np.hypot((xx - mu), (yy - mv) * 2.0)
    arr += (dist < 26)[..., None] * np.array([210, 215, 190], np.float32)
    arr += np.exp(-(dist / 90.0) ** 2)[..., None] * np.array([45, 50, 60], np.float32)
    top = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    full = Image.new("RGB", (W, H))
    full.paste(top, (0, 0))
    full.paste(top.transpose(Image.FLIP_TOP_BOTTOM), (0, H // 2))
    tex["night_dome"] = full
    return tex


def _write_obj(path: Path, V, VT, VN, F):
    lines = ["# generated by px4_gazebo/race_world.py"]
    lines += [f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in V]
    lines += [f"vt {u:.5f} {v:.5f}" for u, v in VT]
    lines += [f"vn {x:.5f} {y:.5f} {z:.5f}" for x, y, z in VN]
    lines += [f"f {a + 1}/{a + 1}/{a + 1} {b + 1}/{b + 1}/{b + 1} {c + 1}/{c + 1}/{c + 1}" for a, b, c in F]
    path.write_text("\n".join(lines) + "\n")


def _vertex_normals(V, F):
    V, F = np.asarray(V), np.asarray(F)
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    vn = np.zeros_like(V)
    for k in range(3):
        np.add.at(vn, F[:, k], fn)
    nrm = np.linalg.norm(vn, axis=1, keepdims=True)
    return vn / np.maximum(nrm, 1e-12)


def plane_obj(path: Path, sx: float, sy: float, tile: float, v_across: bool = False):
    """A flat rectangle sx x sy centred on the origin, normal up, picture repeated every `tile` m (v_across: the picture spans the width once, for roads)."""
    V = [(-sx / 2, -sy / 2, 0), (sx / 2, -sy / 2, 0), (sx / 2, sy / 2, 0), (-sx / 2, sy / 2, 0)]
    if v_across:
        VT = [(0, 0), (sx / tile, 0), (sx / tile, 1), (0, 1)]
    else:
        VT = [(0, 0), (sx / tile, 0), (sx / tile, sy / tile), (0, sy / tile)]
    _write_obj(path, V, VT, [(0, 0, 1)] * 4, [(0, 1, 2), (0, 2, 3)])


def blob_obj(path: Path, rx, ry, rz, seed: int, noise: float = 0.18, nu: int = 28, nv: int = 16, flat_bottom: bool = False):
    """A lumpy ellipsoid (tree crown, bush, rock): a UV sphere whose radius is bent by a few random waves over the direction."""
    rng = np.random.default_rng(seed)
    waves = [(rng.normal(size=3), rng.random() * 6.3, rng.random() * 0.6 + 0.4) for _ in range(6)]
    V, VT = [], []
    for j in range(nv + 1):
        th = math.pi * j / nv                       # 0 at the top
        for i in range(nu + 1):
            ph = 2 * math.pi * i / nu
            d = np.array([math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th)])
            bump = sum(a * math.sin(3.0 * float(np.dot(w, d)) + p) for w, p, a in waves) / 3.0
            r = 1.0 + noise * bump
            p = np.array([rx * r * d[0], ry * r * d[1], rz * r * d[2]])
            if flat_bottom:
                p[2] = max(p[2], -0.05 * rz)
            V.append(tuple(p))
            VT.append((i / nu, 1 - j / nv))
    F = []
    w = nu + 1
    for j in range(nv):
        for i in range(nu):
            a, b, c, d = j * w + i, j * w + i + 1, (j + 1) * w + i, (j + 1) * w + i + 1
            F += [(a, c, b), (b, c, d)]
    _write_obj(path, V, VT, _vertex_normals(V, F), F)


def height_obj(path: Path, half: float, height: float, seed: int, peak: float = 1.6, ridge: float = 0.25, n: int = 56):
    """A hill or a mountain: a square height field with a round footprint, height*(1-r)^peak plus ridged noise, flat (z = 0) at the border."""
    rng = np.random.default_rng(seed)
    waves = [(rng.normal(size=2) * 3.0, rng.random() * 6.3, rng.random() * 0.8 + 0.2) for _ in range(8)]
    xs = np.linspace(-half, half, n + 1)
    V, VT = [], []
    for j, y in enumerate(xs):
        for i, x in enumerate(xs):
            r = math.hypot(x, y) / half
            base = height * max(0.0, 1.0 - r) ** peak
            rid = sum(a * abs(math.sin(float(np.dot(w, (x / half, y / half))) * 2.0 + p)) for w, p, a in waves) / 4.0
            V.append((x, y, base * (1.0 + ridge * rid) if r < 1.0 else 0.0))
            VT.append((i / n, j / n))
    F = []
    w = n + 1
    for j in range(n):
        for i in range(n):
            a, b, c, d = j * w + i, j * w + i + 1, (j + 1) * w + i, (j + 1) * w + i + 1
            F += [(a, b, c), (b, d, c)]
    _write_obj(path, V, VT, _vertex_normals(V, F), F)


def dome_obj(path: Path, R: float, nu: int = 48, nv: int = 24):
    """A sphere seen from the inside (faces and normals point to the centre); the picture is equirectangular."""
    V, VT, VN = [], [], []
    for j in range(nv + 1):
        th = math.pi * j / nv
        for i in range(nu + 1):
            ph = 2 * math.pi * i / nu
            d = (math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th))
            V.append((R * d[0], R * d[1], R * d[2]))
            VN.append((-d[0], -d[1], -d[2]))
            VT.append((i / nu, 1 - j / nv))
    F = []
    w = nu + 1
    for j in range(nv):
        for i in range(nu):
            a, b, c, d = j * w + i, j * w + i + 1, (j + 1) * w + i, (j + 1) * w + i + 1
            F += [(a, b, c), (b, d, c)]          # reversed against the outside sphere
    _write_obj(path, V, VT, VN, F)


MESHES = {  # file name -> how to write it
    "ground_grass.obj": lambda p: plane_obj(p, 2 * GROUND_HALF, 2 * GROUND_HALF, 8.0),
    "road_s.obj": lambda p: plane_obj(p, 160.0, 10.5, 8.0, v_across=True),
    "road_n.obj": lambda p: plane_obj(p, 160.0, 10.5, 8.0, v_across=True),
    "walk_s.obj": lambda p: plane_obj(p, 160.0, 3.8, 4.0),
    "walk_n.obj": lambda p: plane_obj(p, 160.0, 3.8, 4.0),
    "plaza.obj": lambda p: plane_obj(p, 8.0, 8.0, 4.0),
    "crown_a.obj": lambda p: blob_obj(p, 2.4, 2.4, 2.2, 1),
    "crown_b.obj": lambda p: blob_obj(p, 2.0, 2.0, 2.8, 2),
    "crown_c.obj": lambda p: blob_obj(p, 1.6, 1.6, 3.6, 3, noise=0.12),
    "bush_a.obj": lambda p: blob_obj(p, 0.9, 0.9, 0.7, 4, noise=0.25, flat_bottom=True),
    "bush_b.obj": lambda p: blob_obj(p, 1.2, 0.8, 0.8, 5, noise=0.25, flat_bottom=True),
    "rock_a.obj": lambda p: blob_obj(p, 0.9, 0.7, 0.55, 6, noise=0.35, nu=14, nv=9, flat_bottom=True),
    "rock_b.obj": lambda p: blob_obj(p, 1.4, 1.1, 0.8, 7, noise=0.35, nu=14, nv=9, flat_bottom=True),
    "rock_c.obj": lambda p: blob_obj(p, 0.5, 0.5, 0.45, 8, noise=0.35, nu=12, nv=8, flat_bottom=True),
    "hill_a.obj": lambda p: height_obj(p, 120.0, 38.0, 11, peak=1.2, ridge=0.08),
    "hill_b.obj": lambda p: height_obj(p, 90.0, 26.0, 12, peak=1.1, ridge=0.08),
    "mountain_a.obj": lambda p: height_obj(p, 220.0, 190.0, 13, peak=1.5, ridge=0.35),
    "mountain_b.obj": lambda p: height_obj(p, 260.0, 240.0, 14, peak=1.7, ridge=0.40),
    "dome.obj": lambda p: dome_obj(p, DOME_R),
}


def write_assets(out_dir: Path) -> dict:
    """Write textures (PNG) and meshes (OBJ) into out_dir; return name -> absolute file path."""
    from PIL import Image, ImageDraw
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = {}
    for name, im in make_textures(Image, ImageDraw).items():
        p = out_dir / f"{name}.png"
        im.save(p)
        paths[name] = str(p.resolve())
    for name, fn in MESHES.items():
        p = out_dir / name
        fn(p)
        paths[name] = str(p.resolve())
    return paths


# ------------------------------------------------------------------------------------------------ SDF pieces
def uri(path: str) -> str:
    return "file://" + path.replace("\\", "/")


def material(rgb, spec=0.05, emissive=None, albedo=None, rough=0.9, metal=0.0) -> str:
    s = f"<material><ambient>{f(rgb)} 1</ambient><diffuse>{f(rgb)} 1</diffuse><specular>{spec} {spec} {spec} 1</specular>"
    if emissive is not None:
        s += f"<emissive>{f(emissive)} 1</emissive>"
    if albedo:
        s += f"<pbr><metal><albedo_map>{uri(albedo)}</albedo_map><roughness>{rough}</roughness><metalness>{metal}</metalness></metal></pbr>"
    return s + "</material>"


def geom_box(sx, sy, sz):
    return f"<geometry><box><size>{sx:.4g} {sy:.4g} {sz:.4g}</size></box></geometry>"


def geom_cyl(r, length):
    return f"<geometry><cylinder><radius>{r:.4g}</radius><length>{length:.4g}</length></cylinder></geometry>"


def geom_sphere(r):
    return f"<geometry><sphere><radius>{r:.4g}</radius></sphere></geometry>"


def geom_mesh(path, scale=(1, 1, 1)):
    return f"<geometry><mesh><uri>{uri(path)}</uri><scale>{f(scale)}</scale></mesh></geometry>"


def label_plugin(label: int) -> str:
    return f'<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{label}</label></plugin>'


class Link:
    """Collects the visuals and collisions of one link."""

    def __init__(self):
        self.parts, self.n = [], 0

    def visual(self, geom, mat, pose=(0, 0, 0, 0, 0, 0), label=None, collide=False, cast_shadows=False):
        self.n += 1
        name = f"v{self.n}"
        s = f"<visual name='{name}'><pose>{f(pose)}</pose>{geom}{mat}"
        s += "<cast_shadows>false</cast_shadows>"
        if label is not None:
            s += label_plugin(label)
        s += "</visual>"
        if collide:
            s += f"<collision name='{name}_c'><pose>{f(pose)}</pose>{geom}</collision>"
        self.parts.append(s)

    def raw(self, text):
        self.parts.append(text)

    def text(self):
        return "".join(self.parts)


def model_sdf(name: str, link_text: str, pose=(0, 0, 0, 0, 0, 0), static=True, gravity=True) -> str:
    inertial = "" if static else "<inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>"
    return (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>{'true' if static else 'false'}</static><pose>{f(pose)}</pose>"
            f"<link name='link'><gravity>{'true' if gravity else 'false'}</gravity>{inertial}{link_text}</link></model></sdf>")


# ------------------------------------------------------------------------------------------------ placement of natural objects
def path_points():
    ts = np.arange(0.0, rc.T_ROUTE, 0.25)
    return np.array([rc.SPLINE(t)[:2] for t in ts])


def keep_clear_of_route(xy, clearance: float, pts=None) -> bool:
    pts = path_points() if pts is None else pts
    d = np.hypot(pts[:, 0] - xy[0], pts[:, 1] - xy[1]).min()
    if d < clearance:
        return False
    for g in rc.GATES:                      # gates, including their posts
        if math.hypot(xy[0] - g[0], xy[1] - g[1]) < clearance - 0.5:
            return False
    for o in rc.POLES:
        if math.hypot(xy[0] - o["xy"][0], xy[1] - o["xy"][1]) < clearance - 0.5:
            return False
    for o in rc.BOXES:
        if math.hypot(xy[0] - o["xy"][0], xy[1] - o["xy"][1]) < clearance - 0.5:
            return False
    for pad in (rc.START_PAD, rc.END_PAD):
        if math.hypot(xy[0] - pad[0], xy[1] - pad[1]) < clearance + 1.2:
            return False
    return True


NATURE_REGION = dict(x=(-9.0, 34.0), y=(-11.0, 11.0))   # the grass lawn between the two sidewalks


def place_nature(seed: int = 7):
    """(trees, bushes, rocks): lists of (x, y, yaw, variant) around the course. Trees at least 4.2 m (horizontally) from the flight path, bushes
    and rocks at least 2.6 m; trees at least 5.5 m apart. A fixed seed, so the world is the same in every run."""
    rng = np.random.default_rng(seed)
    pts = path_points()
    trees, bushes, rocks = [], [], []
    x0, x1 = NATURE_REGION["x"]
    y0, y1 = NATURE_REGION["y"]
    for _ in range(4000):
        if len(trees) >= 22:
            break
        p = (rng.uniform(x0, x1), rng.uniform(y0, y1))
        if not keep_clear_of_route(p, 4.2, pts):
            continue
        if any(math.hypot(p[0] - t[0], p[1] - t[1]) < 5.5 for t in trees):
            continue
        trees.append((p[0], p[1], rng.uniform(0, 6.28), int(rng.integers(0, 3))))
    for _ in range(6000):
        if len(bushes) >= 40:
            break
        p = (rng.uniform(x0, x1), rng.uniform(y0, y1))
        if not keep_clear_of_route(p, 2.6, pts):
            continue
        if any(math.hypot(p[0] - t[0], p[1] - t[1]) < 2.2 for t in trees + bushes):
            continue
        bushes.append((p[0], p[1], rng.uniform(0, 6.28), int(rng.integers(0, 2))))
    for _ in range(6000):
        if len(rocks) >= 16:
            break
        p = (rng.uniform(x0, x1), rng.uniform(y0, y1))
        if not keep_clear_of_route(p, 2.6, pts):
            continue
        if any(math.hypot(p[0] - t[0], p[1] - t[1]) < 2.0 for t in trees + bushes + rocks):
            continue
        rocks.append((p[0], p[1], rng.uniform(0, 6.28), int(rng.integers(0, 3))))
    return trees, bushes, rocks


BUILDINGS = [  # (cx, cy, width x, depth y, height, facade)
    (-45, -45, 34, 28, 38, "facade_gray"), (-8, -42, 26, 24, 24, "facade_beige"), (22, -46, 30, 30, 46, "facade_glass"), (58, -44, 28, 26, 30, "facade_brick"),
    (-48, 46, 30, 30, 30, "facade_brick"), (-12, 44, 28, 24, 52, "facade_gray"), (22, 46, 24, 28, 44, "facade_glass"), (56, 44, 30, 26, 26, "facade_beige"),
    (-62, 0, 24, 36, 34, "facade_beige"), (-82, -38, 24, 26, 60, "facade_glass"), (-86, 34, 26, 26, 42, "facade_gray"), (88, -46, 30, 28, 36, "facade_gray"),
]
HILLS = [  # (x, y, mesh, yaw, scale xy, scale z)
    (110, -40, "hill_a.obj", 0.3, 1.0, 1.0), (140, 60, "hill_b.obj", 1.1, 1.2, 1.1), (180, 0, "hill_a.obj", 2.0, 1.4, 1.3), (95, 95, "hill_b.obj", 0.2, 1.0, 1.0),
    (-130, -120, "hill_a.obj", 1.5, 1.2, 1.0), (-140, 110, "hill_b.obj", 0.5, 1.3, 1.2),
]
MOUNTAINS = [  # (x, y, mesh, yaw)
    (420, -120, "mountain_a.obj", 0.4), (470, 140, "mountain_b.obj", 1.2), (330, 330, "mountain_a.obj", 2.4), (-80, 520, "mountain_b.obj", 0.9),
    (-420, -150, "mountain_a.obj", 2.0), (80, -520, "mountain_b.obj", 0.1),
]
CAR_LANES = dict(loop=[(-40.0, -18.2), (70.0, -18.2), (70.0, -22.6), (-40.0, -22.6)], speed=3.2)
TRUCK_LANES = dict(loop=[(70.0, 18.2), (-40.0, 18.2), (-40.0, 22.6), (70.0, 22.6)], speed=2.6)
PERSON_LANES = dict(loop=[(-8.0, -12.6), (32.0, -12.6), (32.0, -14.2), (-8.0, -14.2)], speed=1.3)
MOVERS = dict(car=CAR_LANES, truck=TRUCK_LANES, person=PERSON_LANES)


def city_model(assets: dict) -> tuple[str, dict]:
    """The static model `city`: everything that does not move. Returns (SDF text, counts)."""
    L = Link()
    counts = {}
    # ground: grass everywhere, then roads, sidewalks and pads a little above it
    L.visual(geom_mesh(assets["ground_grass.obj"]), material((0.25, 0.42, 0.16), albedo=assets["grass"], rough=1.0), (0, 0, 0, 0, 0, 0), label=15)
    L.visual(geom_mesh(assets["road_s.obj"]), material((0.22, 0.22, 0.24), albedo=assets["road"], rough=0.85), (15, -20.4, 0.012, 0, 0, 0), label=14)
    L.visual(geom_mesh(assets["road_n.obj"]), material((0.22, 0.22, 0.24), albedo=assets["road"], rough=0.85), (15, 20.4, 0.012, 0, 0, math.pi), label=14)
    L.visual(geom_mesh(assets["walk_s.obj"]), material((0.68, 0.68, 0.64), albedo=assets["concrete"], rough=0.9), (15, -13.6, 0.025, 0, 0, 0), label=16)
    L.visual(geom_mesh(assets["walk_n.obj"]), material((0.68, 0.68, 0.64), albedo=assets["concrete"], rough=0.9), (15, 13.6, 0.025, 0, 0, 0), label=16)
    L.visual(geom_mesh(assets["plaza.obj"]), material((0.68, 0.68, 0.64), albedo=assets["concrete"], rough=0.9), (0, 0, 0.02, 0, 0, 0), label=16)
    counts["ground_pieces"] = 6
    # pads: the start pad (yellow frame) and the end pad (blue with a white H)
    sx, sy = rc.START_PAD[:2]
    ex, ey = rc.END_PAD[:2]
    L.visual(geom_box(rc.PAD_SIZE, rc.PAD_SIZE, 0.04), material((0.95, 0.75, 0.1)), (sx, sy, 0.03, 0, 0, 0), label=11)
    L.visual(geom_box(rc.PAD_SIZE - 0.5, rc.PAD_SIZE - 0.5, 0.045), material((0.25, 0.25, 0.28)), (sx, sy, 0.032, 0, 0, 0), label=11)
    L.visual(geom_box(rc.PAD_SIZE, rc.PAD_SIZE, 0.04), material((0.1, 0.25, 0.75)), (ex, ey, 0.03, 0, 0, 0), label=11)
    for dx, dy, bx, by in ((-0.6, 0.0, 0.3, 1.9), (0.6, 0.0, 0.3, 1.9), (0.0, 0.0, 0.9, 0.3)):
        L.visual(geom_box(bx, by, 0.01), material((1, 1, 1), emissive=(0.3, 0.3, 0.3)), (ex + dx, ey + dy, 0.056, 0, 0, 0), label=11)
    # gates (frames orange, post grey, with collision), poles (red and white bands), the box
    orange = material((1.0, 0.45, 0.0), emissive=(0.25, 0.10, 0.0))
    grey = material((0.35, 0.35, 0.4))
    for g in range(rc.N_GATES):
        gx, gy, gz, yaw = rc.GATES[g]
        c, Rm = rc.gate_frame(g)
        for (cx, cy, cz), (hx, hy, hz) in rc.FRAME_BOXES + [rc.post_box(g)]:
            off = Rm @ np.array([cx, cy, cz])
            is_post = (hx == rc.POST / 2)
            L.visual(geom_box(2 * hx, 2 * hy, 2 * hz), grey if is_post else orange, (gx + off[0], gy + off[1], gz + off[2], 0, 0, yaw), label=1, collide=True)
    counts["gates"] = rc.N_GATES
    for o in rc.POLES:
        bands = 5
        for k in range(bands):
            h = o["height"] / bands
            col = (0.85, 0.1, 0.1) if k % 2 == 0 else (0.95, 0.95, 0.95)
            L.visual(geom_cyl(o["radius"], h), material(col), (o["xy"][0], o["xy"][1], h * (k + 0.5), 0, 0, 0), label=2, collide=True)
    for o in rc.BOXES:
        L.visual(geom_box(2 * o["half"], 2 * o["half"], o["height"]), material((0.55, 0.36, 0.18)), (o["xy"][0], o["xy"][1], o["height"] / 2, 0, 0, 0), label=3, collide=True)
    # buildings (facade pictures; the glass towers are shiny: the reflecting surface)
    for (cx, cy, w, d, h, fac) in BUILDINGS:
        glass = fac == "facade_glass"
        L.visual(geom_box(w, d, h), material((0.7, 0.7, 0.7), spec=0.9 if glass else 0.1, albedo=assets[fac], rough=0.05 if glass else 0.8, metal=0.85 if glass else 0.0),
                 (cx, cy, h / 2, 0, 0, 0), label=4)
    counts["buildings"] = len(BUILDINGS)
    # the reflecting pool: dark water with a mirror-like surface, next to gate 5
    L.visual(geom_box(9.0, 2.4, 0.05), material((0.04, 0.10, 0.22), spec=1.0, rough=0.02, metal=0.9, albedo=None), (12.0, -10.4, 0.04, 0, 0, 0), label=17)
    # nature
    trees, bushes, rocks = place_nature()
    crown = ["crown_a.obj", "crown_b.obj", "crown_c.obj"]
    greens = [(0.14, 0.38, 0.12), (0.10, 0.30, 0.10), (0.06, 0.24, 0.12)]
    for (x, y, yaw, v) in trees:
        th = [3.2, 3.8, 3.6][v]
        L.visual(geom_cyl(0.28, th), material((0.30, 0.20, 0.10)), (x, y, th / 2, 0, 0, 0), label=5)
        cz = {0: 2.2, 1: 2.8, 2: 3.6}[v]
        L.visual(geom_mesh(assets[crown[v]]), material(greens[v]), (x, y, th + cz * 0.7, 0, 0, yaw), label=5)
    for (x, y, yaw, v) in bushes:
        L.visual(geom_mesh(assets[["bush_a.obj", "bush_b.obj"][v]]), material((0.18, 0.42, 0.14)), (x, y, 0.0, 0, 0, yaw), label=6)
    for (x, y, yaw, v) in rocks:
        L.visual(geom_mesh(assets[["rock_a.obj", "rock_b.obj", "rock_c.obj"][v]]), material((0.42, 0.40, 0.37), spec=0.1), (x, y, 0.0, 0, 0, yaw), label=7)
    counts.update(trees=len(trees), bushes=len(bushes), rocks=len(rocks))
    # street trees along both sidewalks (far from the course)
    n_street = 0
    for x in range(-34, 66, 12):
        for y in (-27.5, 27.5):
            L.visual(geom_cyl(0.22, 3.0), material((0.30, 0.20, 0.10)), (x, y, 1.5, 0, 0, 0), label=5)
            L.visual(geom_mesh(assets["crown_a.obj"]), material((0.15, 0.40, 0.14)), (x, y, 4.2, 0, 0, x), label=5)
            n_street += 1
    counts["street_trees"] = n_street
    # hills and mountains (green and blue-grey)
    for (x, y, mesh, yaw, sxy, sz) in HILLS:
        L.visual(geom_mesh(assets[mesh], (sxy, sxy, sz)), material((0.26, 0.46, 0.20), albedo=assets["grass"], rough=1.0), (x, y, -0.3, 0, 0, yaw), label=12)
    for (x, y, mesh, yaw) in MOUNTAINS:
        L.visual(geom_mesh(assets[mesh]), material((0.42, 0.48, 0.58), spec=0.05), (x, y, -0.5, 0, 0, yaw), label=12)
    counts.update(hills=len(HILLS), mountains=len(MOUNTAINS))
    # lamp posts with a point light each (off by day, on at night)
    for k, (x, y) in enumerate(LAMPS):
        L.visual(geom_cyl(0.07, 4.2), material((0.25, 0.25, 0.28)), (x, y, 2.1, 0, 0, 0), label=13)
        L.visual(geom_sphere(0.22), material((1, 0.9, 0.6), emissive=(1.0, 0.85, 0.5)), (x, y, 4.35, 0, 0, 0), label=13)
        L.raw(f"<light name='lamp_light_{k}' type='point'><pose>{x} {y} 4.35 0 0 0</pose><cast_shadows>false</cast_shadows><diffuse>1 0.85 0.55 1</diffuse>"
              f"<specular>0.3 0.3 0.2 1</specular><intensity>0</intensity><attenuation><range>18</range><constant>0.2</constant><linear>0.1</linear><quadratic>0.02</quadratic></attenuation></light>")
    counts["lamps"] = len(LAMPS)
    # the sun: a bright disc far away, in the direction of the sun light, beyond the night dome (no label)
    sp = SUN_TO * SUN_DIST
    L.visual(geom_sphere(SUN_RADIUS), material((1, 1, 0.9), emissive=(1.0, 0.97, 0.85)), (sp[0], sp[1], sp[2], 0, 0, 0))
    return model_sdf("city", L.text(), static=True), counts


def car_model(assets=None) -> str:
    L = Link()
    L.visual(geom_box(4.4, 1.85, 0.75), material((0.75, 0.08, 0.12), spec=0.4), (0, 0, 0.65, 0, 0, 0), label=8)
    L.visual(geom_box(2.3, 1.65, 0.62), material((0.08, 0.1, 0.14), spec=0.8), (-0.2, 0, 1.3, 0, 0, 0), label=8)
    for sx in (-1.4, 1.4):
        for sy in (-0.95, 0.95):
            L.visual(geom_cyl(0.34, 0.26), material((0.05, 0.05, 0.05)), (sx, sy, 0.34, math.pi / 2, 0, 0), label=8)
    L.visual(geom_box(0.05, 1.3, 0.2), material((1, 1, 0.8), emissive=(0.8, 0.8, 0.6)), (2.21, 0, 0.7, 0, 0, 0), label=8)
    return model_sdf("car", L.text(), static=False, gravity=False)


def truck_model(assets=None) -> str:
    L = Link()
    L.visual(geom_box(2.4, 2.5, 2.5), material((0.92, 0.92, 0.92), spec=0.3), (3.0, 0, 1.75, 0, 0, 0), label=9)
    L.visual(geom_box(0.1, 2.3, 1.0), material((0.1, 0.12, 0.16), spec=0.8), (4.22, 0, 2.2, 0, 0, 0), label=9)
    L.visual(geom_box(6.0, 2.6, 3.0), material((0.12, 0.30, 0.78)), (-1.0, 0, 2.0, 0, 0, 0), label=9)
    L.visual(geom_box(8.6, 1.9, 0.4), material((0.1, 0.1, 0.1)), (0.3, 0, 0.55, 0, 0, 0), label=9)
    for sx in (3.0, -0.2, -2.6):
        for sy in (-1.2, 1.2):
            L.visual(geom_cyl(0.5, 0.35), material((0.04, 0.04, 0.04)), (sx, sy, 0.5, math.pi / 2, 0, 0), label=9)
    return model_sdf("truck", L.text(), static=False, gravity=False)


def person_models() -> dict:
    """The walking person as three models: body (torso, head, arms), left leg, right leg. The legs swing (the runner sets their pose)."""
    B = Link()
    B.visual(geom_box(0.28, 0.46, 0.62), material((0.15, 0.25, 0.75)), (0, 0, 1.15, 0, 0, 0), label=10)
    B.visual(geom_sphere(0.13), material((0.9, 0.7, 0.55)), (0, 0, 1.62, 0, 0, 0), label=10)
    for sy in (-0.28, 0.28):
        B.visual(geom_cyl(0.05, 0.6), material((0.15, 0.25, 0.75)), (0, sy, 1.12, 0, 0, 0), label=10)
    out = {"person_body": model_sdf("person_body", B.text(), static=False, gravity=False)}
    for nm, sy in (("person_leg_l", 0.11), ("person_leg_r", -0.11)):
        Lg = Link()
        Lg.visual(geom_cyl(0.075, 0.84), material((0.12, 0.12, 0.16)), (0, sy, -0.42, 0, 0, 0), label=10)    # the model origin is the hip
        out[nm] = model_sdf(nm, Lg.text(), static=False, gravity=False)
    return out


def rain_model(rate: int = 5000) -> str:
    """One box particle emitter, a 14 m x 14 m sheet; the emission direction (+x of the link) points down. The runner moves the model with the drone."""
    return (f"<?xml version='1.0'?><sdf version='1.9'><model name='rain'><static>false</static><pose>0 0 -300 0 0 0</pose>"
            f"<link name='link'><gravity>false</gravity><pose>0 0 0 0 1.5708 0</pose><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>"
            f"<particle_emitter name='rain_e' type='box'><emitting>true</emitting><size>0.1 14 14</size><particle_size>0.04 0.6 0.04</particle_size>"
            f"<lifetime>1.4</lifetime><min_velocity>7</min_velocity><max_velocity>9</max_velocity><scale_rate>0</scale_rate><rate>{rate}</rate>"
            f"<color_start>0.85 0.9 1.0 0.7</color_start><color_end>0.85 0.9 1.0 0.35</color_end>"
            f"<material><diffuse>0.9 0.92 1.0</diffuse><specular>0.3 0.3 0.3</specular></material></particle_emitter></link></model></sdf>")


def dome_model(assets: dict) -> str:
    L = Link()
    L.visual(geom_mesh(assets["dome.obj"]), material((1, 1, 1), emissive=(0.9, 0.9, 0.9), albedo=assets["night_dome"], rough=1.0), (0, 0, 0, 0, 0, 0))
    return model_sdf("night_dome", L.text(), pose=(0, 0, -5000, 0, 0, 0), static=False, gravity=False)


# ------------------------------------------------------------------------------------------------ the drone model patch (cameras, headlamp, wind)
PW, PH = 384, 288
FPV_FOV, CHASE_FOV = 1.5, 1.4
FPS = 15
FPV_POSE = "0.42 0 0.04 0 -0.12 0"      # ahead of the propellers (the stock mono_cam sees the front left propeller), tilted 7 degrees up
CHASE_BACK, CHASE_UP = 3.2, 1.2
NOISE_STDDEV = 0.07


def _cam_link(name, pose, sensor_xml):
    return (f"<link name='{name}_link'><pose relative_to='base_link'>{pose}</pose><gravity>false</gravity>"
            f"<inertial><mass>0.001</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>{sensor_xml}</link>"
            f"<joint name='{name}_joint' type='fixed'><parent>base_link</parent><child>{name}_link</child></joint>")


def _sensor(name, kind, fov, w, h, extra_cam="", near=0.1, far=3000, fmt=""):
    return (f"<sensor name='{name}' type='{kind}'><gz_frame_id>{name}_link</gz_frame_id><camera><horizontal_fov>{fov}</horizontal_fov>"
            f"<image><width>{w}</width><height>{h}</height>{fmt}</image><clip><near>{near}</near><far>{far}</far></clip>{extra_cam}</camera>"
            f"<always_on>1</always_on><update_rate>{FPS}</update_rate><visualize>false</visualize></sensor>")


def drone_links(w=PW, h=PH, with_noise=True, with_depth=True, with_seg=True) -> str:
    """Extra links of the x500: nose camera, the same camera with sensor noise, depth camera, segmentation camera (all on the same mount), chase camera."""
    s = _cam_link("fpv", FPV_POSE, _sensor("fpv", "camera", FPV_FOV, w, h))
    if with_noise:
        s += _cam_link("fpvn", FPV_POSE, _sensor("fpvn", "camera", FPV_FOV, w, h, extra_cam=f"<noise><type>gaussian</type><mean>0.0</mean><stddev>{NOISE_STDDEV}</stddev></noise>"))
    if with_depth:
        s += _cam_link("depth", FPV_POSE, _sensor("depth", "depth_camera", FPV_FOV, w, h, near=0.3, far=80, fmt="<format>R_FLOAT32</format>"))
    if with_seg:
        s += _cam_link("seg", FPV_POSE, _sensor("seg", "segmentation_camera", FPV_FOV, w, h, extra_cam="<segmentation_type>semantic</segmentation_type>", far=500))
    s += _cam_link("chase", f"-{CHASE_BACK} 0 {CHASE_UP} 0 {math.atan2(CHASE_UP, CHASE_BACK):.4f} 0", _sensor("chase", "camera", CHASE_FOV, w, h))
    return s


def headlamp_link_text() -> str:
    return ("<link name='headlamp_link'><pose relative_to='base_link'>0.45 0 0.02 0 0 0</pose><gravity>false</gravity>"
            "<inertial><mass>0.001</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>"
            "<light name='headlamp' type='spot'><pose>0 0 0 0 0 0</pose><cast_shadows>false</cast_shadows>"
            "<diffuse>1 0.95 0.8 1</diffuse><specular>0.5 0.5 0.4 1</specular><intensity>0</intensity>"
            "<attenuation><range>30</range><constant>0.3</constant><linear>0.05</linear><quadratic>0.002</quadratic></attenuation>"
            "<direction>1 0 -0.25</direction><spot><inner_angle>0.35</inner_angle><outer_angle>0.8</outer_angle><falloff>1.0</falloff></spot></light></link>"
            "<joint name='HeadlampJoint' type='fixed'><parent>base_link</parent><child>headlamp_link</child></joint>")


def x500_mono_cam_text(pristine: str, **kw) -> str:
    k = pristine.rindex("</model>")
    return pristine[:k] + drone_links(**kw) + headlamp_link_text() + pristine[k:]


def enable_wind_text(pristine: str) -> tuple[str, bool]:
    """x500_base model: <enable_wind> in base_link, so that the WindEffects system pushes the drone. (text, changed)"""
    key = '<link name="base_link">'
    if "<enable_wind>" in pristine or key not in pristine:
        return pristine, False
    return pristine.replace(key, key + "<enable_wind>true</enable_wind>", 1), True


WIND_PLUGIN_XML = ("<force_approximation_scaling_factor>0.35</force_approximation_scaling_factor>"
                   "<horizontal><magnitude><time_for_rise>2</time_for_rise><sin><amplitude_percent>0.3</amplitude_percent><period>5</period></sin>"
                   "<noise type=\"gaussian\"><mean>0</mean><stddev>0.2</stddev></noise></magnitude>"
                   "<direction><time_for_rise>5</time_for_rise><sin><amplitude>15</amplitude><period>12</period></sin>"
                   "<noise type=\"gaussian\"><mean>0</mean><stddev>3</stddev></noise></direction></horizontal>"
                   "<vertical><noise type=\"gaussian\"><mean>0</mean><stddev>0.1</stddev></noise></vertical>")
WIND_VELOCITY = (3.0, 1.5, 0.0)     # m/s, steady part (world frame, east, north, up)


def server_plugins_from_config(text: str, extra: bool = True) -> str:
    """The plugin list for the world file (variant B): a copy of PX4's server.config plus the particle emitter and the wind effects systems."""
    import re
    out = []
    for m in re.finditer(r'<plugin\s+([^>]*?)(?:/>|>(.*?)</plugin>)', re.sub(r"<!--.*?-->", "", text, flags=re.S), re.S):
        attrs = dict(re.findall(r'(\w+)="([^"]*)"', m.group(1)))
        if "filename" in attrs and "name" in attrs:
            out.append(f"<plugin filename='{attrs['filename']}' name='{attrs['name']}'>{m.group(2) or ''}</plugin>")
    if extra:
        out.append("<plugin filename='gz-sim-particle-emitter-system' name='gz::sim::systems::ParticleEmitter'></plugin>")
        out.append(f"<plugin filename='gz-sim-wind-effects-system' name='gz::sim::systems::WindEffects'>{WIND_PLUGIN_XML}</plugin>")
    return "".join(out)
