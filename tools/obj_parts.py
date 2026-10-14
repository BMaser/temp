"""Read the GT drone model (Wavefront OBJ + MTL), split it into one part per material, write one mesh file per part.

Why one STL file per material (sources: MuJoCo 3.10.0, saved under
G:/My Drive/github/Comprehansive_cv/coursera_mirror/mirrors/Autonomous_Racing_Drone_Path/runs/2026-10-13_drone_gt_sources/):
  - A mesh asset is ONE mesh per file. The OBJ decoder keeps only the first shape of a file
    (plugin_obj_decoder_obj_decoder.cc line 56: `GetShapes()[0]`), so a multi-group OBJ would load one group and drop the rest.
    The STL decoder reads one binary mesh (plugin_stl_decoder_stl_decoder.cc lines 49-126).
  - STL is also what the present drone's own meshes use (cf21B_500.xml lines 24-34), so it takes the same decoder path.
  - Binary STL rules (stl_decoder.cc): at least 84 bytes (line 63); the face count is the uint32 at byte 80 and must be
    between 1 and 200000 (lines 69-76); file size must be exactly 84 + 50 * faces (line 78); the three vertices of face i are
    float32 triples at byte 84 + 50*i + 12*(j+1) (line 95), i.e. after the 12-byte normal; every coordinate must have an
    absolute value of at most 2**30 (lines 97-99). Vertices that are bit-identical are merged (lines 107-114).
  - Units: the decoder returns the numbers as they are in the file; the mesh attribute `scale` (default 1 1 1) multiplies them
    (XMLreference.rst, asset/mesh/scale). The present drone's files are in millimeters and use scale 1e-3 (cf21B_500.xml line 24).
    The GT model is in METERS (drone_notes.txt), so the files written here are in meters and need no scale.
  - MuJoCo checks every mesh when it compiles it (user_mesh.cc): at least 4 vertices (line 1672), face indices in range
    (lines 1699-1705), a nonzero surface area (lines 1448-1450) and a legacy volume of at least mjMINVAL = 1e-15
    (lines 1464-1470; mjtype.h line 27; legacy volume = sum of absolute pyramid volumes, lines 1207-1209). The self-test
    repeats these tests for every part.

Only numpy is needed. `python tools/obj_parts.py --self-test` runs the checks; `python tools/obj_parts.py --write DIR`
writes the mesh files and prints one line for each.
"""

import argparse
import math
import re
import struct
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO_DIR = Path(__file__).resolve().parent.parent
DEFAULT_ASSET_DIR = REPO_DIR / "assets" / "drone_gt"
DEFAULT_OBJ = DEFAULT_ASSET_DIR / "drone.obj"
DEFAULT_MTL = DEFAULT_ASSET_DIR / "mtl" / "cyan.mtl"
LOOKS = ("magenta", "cyan", "red", "violet")

MJMINVAL = 1e-15  # mjtype.h line 27
STL_MAX_FACES = 200000  # stl_decoder.cc line 70
STL_MAX_ABS = 2.0 ** 30  # stl_decoder.cc lines 97-99
MESH_PREFIX = "gt_"
FRONT_MATERIALS = ("nose",)  # materials whose center must be in front (positive x)
FRONT_GROUPS = ("camera",)  # OBJ groups whose center must be in front. The material "glass" is NOT used: it covers the lens (group
# camera) AND glass pieces of group body, so its center is at x = 0; the lens is the group "camera".
REAR_MATERIALS = ("wing",)  # materials whose center must be behind (negative x)

# The drone's own camera: tools use these numbers for the camera cone. Source: crazyflow drones/cf21B_500.xml line 91:
#   <camera name="fpv_cam" pos="0.03 0 0" euler="0 -90 -90" fovy="70" resolution="640 480" />
# Euler sequence "xyz" with lower-case letters = rotations about the rotating axes (XMLreference.rst, compiler eulerseq,
# default "xyz"), angles in degrees (compiler angle default). A MuJoCo camera looks along its -Z axis, +X is right and +Y is up
# (XMLreference.rst, camera section: "The camera is looking along the -Z axis of its frame. The +X axis points to the right,
# and the +Y axis points up."). fovy is the VERTICAL field of view; the horizontal one follows from the picture size
# (XMLreference.rst, camera fovy). The tests draw 640 x 480 pictures (tests/crazyflow_run_a.py WIDTH, HEIGHT).
CAMERA_POS = (0.03, 0.0, 0.0)
CAMERA_EULER_DEG = (0.0, -90.0, -90.0)
CAMERA_FOVY_DEG = 70.0
PICTURE_WIDTH, PICTURE_HEIGHT = 640, 480


# ---------------------------------------------------------------------------------------------
# Reading
# ---------------------------------------------------------------------------------------------


class Model:
    """The whole OBJ: vertices (N, 3) float64 and, for every face (any number of corners >= 3), its vertex indices (0-based),
    material and group name."""

    def __init__(self, vertices, faces, face_material, face_group, mtllib):
        self.vertices = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
        self.faces = faces
        self.face_material = face_material
        self.face_group = face_group
        self.mtllib = mtllib


def parse_obj(path) -> Model:
    """Read `v`, `f`, `g`, `o`, `usemtl`, `mtllib` lines. Faces may use v, v/vt, v//vn or v/vt/vn corners and negative indices.
    Raises ValueError for a face without a material or with fewer than 3 corners or an index out of range."""
    vertices, faces, face_material, face_group = [], [], [], []
    material, group, mtllib = None, "default", None
    for line_no, raw in enumerate(Path(path).read_text().splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        tag, *rest = line.split()
        if tag == "v":
            if len(rest) < 3:
                raise ValueError(f"{path}:{line_no}: vertex with fewer than 3 numbers")
            vertices.append([float(rest[0]), float(rest[1]), float(rest[2])])
        elif tag in ("g", "o"):
            group = rest[0] if rest else "default"
        elif tag == "usemtl":
            material = rest[0]
        elif tag == "mtllib":
            mtllib = rest[0]
        elif tag == "f":
            idx = []
            for corner in rest:
                i = int(corner.split("/")[0])
                i = i - 1 if i > 0 else len(vertices) + i  # negative = counted from the last vertex read so far
                idx.append(i)
            if len(idx) < 3:
                raise ValueError(f"{path}:{line_no}: face with fewer than 3 corners")
            if material is None:
                raise ValueError(f"{path}:{line_no}: face before any usemtl")
            faces.append(idx)
            face_material.append(material)
            face_group.append(group)
    n = len(vertices)
    for k, f in enumerate(faces):
        if min(f) < 0 or max(f) >= n:
            raise ValueError(f"face {k} has a vertex index outside 1..{n}")
    return Model(vertices, faces, face_material, face_group, mtllib)


def parse_mtl(path) -> dict:
    """{material name: {"kd": (r, g, b), "ks": (r, g, b) or None, "ns": float or None}} with colors in 0..1."""
    out, cur = {}, None
    for raw in Path(path).read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        tag, *rest = line.split()
        if tag == "newmtl":
            cur = rest[0]
            out[cur] = {"kd": None, "ks": None, "ns": None}
        elif cur is not None and tag == "Kd":
            out[cur]["kd"] = tuple(float(v) for v in rest[:3])
        elif cur is not None and tag == "Ks":
            out[cur]["ks"] = tuple(float(v) for v in rest[:3])
        elif cur is not None and tag == "Ns":
            out[cur]["ns"] = float(rest[0])
    missing = [k for k, v in out.items() if v["kd"] is None]
    if missing:
        raise ValueError(f"{path}: materials without Kd: {missing}")
    return out


def triangulate(faces: list) -> list:
    """Fan triangulation: a face (i0, i1, ..., ik) becomes (i0, i1, i2), (i0, i2, i3), ... The corner order (the winding) is kept.
    Returns a list of (triangle, source face number)."""
    out = []
    for k, f in enumerate(faces):
        for j in range(1, len(f) - 1):
            out.append(((f[0], f[j], f[j + 1]), k))
    return out


# ---------------------------------------------------------------------------------------------
# Parts
# ---------------------------------------------------------------------------------------------


class Part:
    """All triangles of one material. `vertices` holds only the vertices these triangles use; `triangles` index into it."""

    def __init__(self, name, vertices, triangles, groups, color):
        self.name = name
        self.vertices = vertices  # (n, 3) float64
        self.triangles = triangles  # (m, 3) int
        self.groups = groups  # sorted list of OBJ group names that contribute triangles
        self.color = color  # (r, g, b) 0..1 from Kd, or None when no MTL was given
        self.triangle_groups = None  # (m,) array of group names, filled by split_by_material

    @property
    def n_vertices(self) -> int:
        return int(len(self.vertices))

    @property
    def n_triangles(self) -> int:
        return int(len(self.triangles))

    def corners(self) -> np.ndarray:
        """(m, 3, 3): the three corner positions of every triangle."""
        return self.vertices[self.triangles]

    def extent(self) -> dict:
        lo, hi = self.vertices.min(axis=0), self.vertices.max(axis=0)
        return {"min": lo.tolist(), "max": hi.tolist(), "size": (hi - lo).tolist(), "center": ((lo + hi) / 2).tolist()}

    def summary(self) -> dict:
        e = self.extent()
        return {"name": self.name, "vertices": self.n_vertices, "triangles": self.n_triangles, "groups": self.groups,
                "min": e["min"], "max": e["max"], "size": e["size"], "center": e["center"],
                "color": None if self.color is None else list(self.color)}


def split_by_material(model: Model, mtl: dict | None = None) -> dict:
    """{material name: Part}, in the order the materials first appear in the OBJ."""
    tris = triangulate(model.faces)
    order, per = [], {}
    for tri, k in tris:
        m = model.face_material[k]
        if m not in per:
            per[m] = {"tri": [], "grp": []}
            order.append(m)
        per[m]["tri"].append(tri)
        per[m]["grp"].append(model.face_group[k])
    parts = {}
    for m in order:
        tri = np.asarray(per[m]["tri"], dtype=np.int64)
        used, inverse = np.unique(tri.reshape(-1), return_inverse=True)
        part = Part(m, model.vertices[used].copy(), inverse.reshape(-1, 3).astype(np.int64), sorted(set(per[m]["grp"])),
                    None if mtl is None else tuple(mtl[m]["kd"]))
        part.triangle_groups = np.asarray(per[m]["grp"])
        parts[m] = part
    if mtl is not None:
        unknown = [m for m in parts if m not in mtl]
        if unknown:
            raise ValueError(f"materials used by the OBJ but missing in the MTL: {unknown}")
    return parts


def load_parts(obj_path=DEFAULT_OBJ, mtl_path=DEFAULT_MTL) -> tuple:
    """(model, parts) for the given OBJ and MTL."""
    model = parse_obj(obj_path)
    mtl = parse_mtl(mtl_path) if mtl_path else None
    return model, split_by_material(model, mtl)


# ---------------------------------------------------------------------------------------------
# Mesh files (binary STL, meters)
# ---------------------------------------------------------------------------------------------


def stl_bytes(part: Part) -> bytes:
    """Binary STL of one part. The face normal is computed from the corner order; MuJoCo ignores it."""
    c = part.corners().astype(np.float32)
    n = len(c)
    if not 1 <= n <= STL_MAX_FACES:
        raise ValueError(f"part {part.name}: {n} faces; the STL decoder accepts 1 to {STL_MAX_FACES}")
    if float(np.abs(c).max()) > STL_MAX_ABS:
        raise ValueError(f"part {part.name}: a coordinate exceeds 2**30")
    normal = np.cross(c[:, 1].astype(np.float64) - c[:, 0], c[:, 2].astype(np.float64) - c[:, 0])
    length = np.linalg.norm(normal, axis=1, keepdims=True)
    normal = np.divide(normal, length, out=np.zeros_like(normal), where=length > 0).astype(np.float32)
    rec = np.zeros(n, dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = normal, c
    header = b"GT drone part " + part.name.encode("ascii")[:60]
    data = header.ljust(80, b" ") + struct.pack("<I", n) + rec.tobytes()
    assert len(data) == 84 + 50 * n
    return data


def read_stl(path) -> np.ndarray:
    """(n, 3, 3) float32 corner positions of a binary STL, with the same size checks as MuJoCo's decoder."""
    buf = Path(path).read_bytes()
    if len(buf) < 84:
        raise ValueError(f"{path}: shorter than the 84-byte header")
    (n,) = struct.unpack_from("<I", buf, 80)
    if not 1 <= n <= STL_MAX_FACES:
        raise ValueError(f"{path}: face count {n} outside 1..{STL_MAX_FACES}")
    if n * 50 != len(buf) - 84:
        raise ValueError(f"{path}: size {len(buf)} is not 84 + 50 * {n}")
    rec = np.frombuffer(buf, dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")], count=n, offset=84)
    return np.array(rec["v"])


def mesh_file_name(part_name: str) -> str:
    return f"{MESH_PREFIX}{part_name}.stl"


def write_part_meshes(parts: dict, out_dir) -> dict:
    """Write gt_<material>.stl for every part into out_dir. Returns {material: {"file": absolute path, **summary}}."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    info = {}
    for name, part in parts.items():
        path = (out / mesh_file_name(name)).resolve()
        path.write_bytes(stl_bytes(part))
        info[name] = {"file": str(path), **part.summary()}
    return info


# ---------------------------------------------------------------------------------------------
# Measures
# ---------------------------------------------------------------------------------------------


def group_extent(model: Model, group: str) -> dict:
    """Bounding box of the vertices used by the faces of one OBJ group."""
    ids = sorted({v for f, g in zip(model.faces, model.face_group) if g == group for v in f})
    if not ids:
        raise ValueError(f"no faces in group {group!r}")
    v = model.vertices[ids]
    lo, hi = v.min(axis=0), v.max(axis=0)
    return {"min": lo.tolist(), "max": hi.tolist(), "center": ((lo + hi) / 2).tolist(), "vertices": len(ids)}


def surface_samples(part: Part, k: int = 12) -> np.ndarray:
    """Points spread over every triangle of a part (barycentric grid with k steps per edge), to catch a triangle that crosses the
    view although all three of its corners are outside it."""
    w = np.array([(1 - i / k - j / k, i / k, j / k) for i in range(k + 1) for j in range(k + 1 - i)])
    return np.einsum("qk,mkd->mqd", w, part.corners()).reshape(-1, 3)


def whole_extent(vertices: np.ndarray) -> dict:
    lo, hi = vertices.min(axis=0), vertices.max(axis=0)
    return {"min": lo.tolist(), "max": hi.tolist(), "size": (hi - lo).tolist()}


def legacy_volume_and_area(part: Part) -> tuple:
    """What MuJoCo's compile computes for a mesh with inertia "legacy" (user_mesh.cc lines 1191-1224 and 1449): the sum of
    the absolute volumes of the pyramids from the face centroid, and the surface area. Both must be >= mjMINVAL."""
    c = part.corners()
    cross = np.cross(c[:, 1] - c[:, 0], c[:, 2] - c[:, 0])
    area_each = 0.5 * np.linalg.norm(cross, axis=1)
    area = float(area_each.sum())
    if area <= 0:
        return 0.0, 0.0
    centroid = (c.mean(axis=1) * area_each[:, None]).sum(axis=0) / area  # area-weighted centroid of the faces
    face_center = c.mean(axis=1)
    normal = cross / np.maximum(np.linalg.norm(cross, axis=1, keepdims=True), 1e-300)
    vol = np.abs(((face_center - centroid) * normal).sum(axis=1) * area_each / 3.0)
    return float(vol.sum()), area


def signed_volume(part: Part) -> float:
    """Signed volume by the divergence theorem (positive = faces wound counter-clockwise seen from outside, when closed)."""
    c = part.corners()
    return float(np.einsum("ij,ij->i", c[:, 0], np.cross(c[:, 1], c[:, 2])).sum() / 6.0)


def edge_balance(part: Part) -> dict:
    """Edges: how many are used by exactly one triangle (open boundary), and how many directed edges have no opposite partner
    (a closed, consistently wound mesh has none)."""
    seen = {}
    for a, b, c in part.triangles:
        for e in ((a, b), (b, c), (c, a)):
            seen[e] = seen.get(e, 0) + 1
    unmatched = sum(1 for (a, b), n in seen.items() if seen.get((b, a), 0) != n)
    undirected = {}
    for a, b in seen:
        key = (min(a, b), max(a, b))
        undirected[key] = undirected.get(key, 0) + seen[(a, b)]
    open_edges = sum(1 for n in undirected.values() if n == 1)
    return {"directed_edges": int(len(seen)), "unmatched_directed_edges": int(unmatched), "open_edges": int(open_edges)}


def camera_frame() -> tuple:
    """(position, rotation matrix whose columns are the camera's x, y, z axes written in the drone frame).
    Euler sequence xyz, intrinsic: R = Rx(a) Ry(b) Rz(c)."""
    a, b, c = (math.radians(v) for v in CAMERA_EULER_DEG)
    rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    rz = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1]])
    return np.array(CAMERA_POS, dtype=np.float64), rx @ ry @ rz


def camera_cone_numbers(points: np.ndarray, width: int = PICTURE_WIDTH, height: int = PICTURE_HEIGHT, margin_m: float = 0.0) -> dict:
    """For points in the drone frame: which are inside the camera's view cone (a pyramid with the drone camera at its tip).
    `margin_m` grows (positive) or shrinks (negative) the view by that many meters at the point's own depth, so a vertex that
    only touches the boundary can be told from one that is clearly inside.
    Returns boolean arrays: ahead (depth > 0), inside (ahead and within both angles) and the depth of every point."""
    pos, rot = camera_frame()
    p = (np.asarray(points, dtype=np.float64) - pos) @ rot  # coordinates in the camera frame
    depth = -p[:, 2]  # the camera looks along -z
    half_v = math.tan(math.radians(CAMERA_FOVY_DEG) / 2.0)
    half_h = half_v * width / height
    ahead = depth > 0
    safe = np.where(ahead, depth, 1.0)
    inside = ahead & (np.abs(p[:, 0]) <= half_h * safe + margin_m) & (np.abs(p[:, 1]) <= half_v * safe + margin_m)
    return {"ahead": ahead, "inside": inside, "depth": depth, "cam": p, "half_h_tan": half_h, "half_v_tan": half_v}


def cone_report(model: Model, parts: dict) -> dict:
    """Count the model's vertices inside the camera's view, per material part and per OBJ group (vertices as the faces use them)."""
    cone_all = camera_cone_numbers(model.vertices)
    by_part = {}
    for name, part in parts.items():
        c = camera_cone_numbers(part.vertices)
        grazing = camera_cone_numbers(part.vertices, margin_m=0.0005)["inside"] & ~camera_cone_numbers(part.vertices, margin_m=-0.0005)["inside"]
        by_part[name] = {"vertices": part.n_vertices, "inside": int(c["inside"].sum()), "ahead_of_lens": int(c["ahead"].sum()),
                         "within_0p5mm_of_the_boundary": int((grazing & c["ahead"]).sum()),
                         "surface_samples_inside": int(camera_cone_numbers(surface_samples(part))["inside"].sum()),
                         "min_depth_m": float(c["depth"][c["ahead"]].min()) if c["ahead"].any() else None}
    group_vertices = {}
    for f, g in zip(model.faces, model.face_group):
        group_vertices.setdefault(g, set()).update(f)
    by_group = {}
    for g, ids in group_vertices.items():
        ids = sorted(ids)
        inside = cone_all["inside"][ids]
        by_group[g] = {"vertices": len(ids), "inside": int(inside.sum()), "ahead_of_lens": int(cone_all["ahead"][ids].sum())}
    x_ahead = model.vertices[:, 0] > CAMERA_POS[0]
    return {
        "camera": {"pos": list(CAMERA_POS), "euler_deg": list(CAMERA_EULER_DEG), "fovy_deg": CAMERA_FOVY_DEG,
                   "picture": [PICTURE_WIDTH, PICTURE_HEIGHT], "looks_along_drone_axis": (-camera_frame()[1][:, 2]).round(9).tolist(),
                   "up_axis_in_drone_frame": camera_frame()[1][:, 1].round(9).tolist(),
                   "half_angle_h_deg": math.degrees(math.atan(cone_all["half_h_tan"])), "half_angle_v_deg": math.degrees(math.atan(cone_all["half_v_tan"]))},
        "total_vertices": int(len(model.vertices)), "total_inside": int(cone_all["inside"].sum()),
        "total_ahead_of_lens_depth_positive": int(cone_all["ahead"].sum()),
        "vertices_with_x_greater_than_lens_x": int(x_ahead.sum()),
        "max_z_of_vertices_ahead_of_lens_x": float(model.vertices[x_ahead, 2].max()) if x_ahead.any() else None,
        "by_part": by_part, "by_group": by_group,
    }


# ---------------------------------------------------------------------------------------------
# Self-test: every check can fail (see the meta tests at the end)
# ---------------------------------------------------------------------------------------------


def check_triangle_count(parts: dict, expected_total: int) -> bool:
    return sum(p.n_triangles for p in parts.values()) == expected_total


def check_extent_equal(parts: dict, expected: dict, tol: float = 0.0) -> bool:
    lo = np.min([p.vertices.min(axis=0) for p in parts.values()], axis=0)
    hi = np.max([p.vertices.max(axis=0) for p in parts.values()], axis=0)
    return bool(np.all(np.abs(lo - np.array(expected["min"])) <= tol) and np.all(np.abs(hi - np.array(expected["max"])) <= tol))


def raw_text_numbers(path) -> dict:
    """Independent count straight from the text (no parse_obj): vertices, faces, triangles after fan triangulation, extent."""
    v, tri, nf = [], 0, 0
    for line in Path(path).read_text().splitlines():
        if line.startswith("v "):
            v.append([float(t) for t in line.split()[1:4]])
        elif line.startswith("f "):
            nf += 1
            tri += max(len(line.split()) - 1 - 2, 0)
    a = np.array(v)
    return {"vertices": len(v), "faces": nf, "triangles": tri, "min": a.min(axis=0).tolist(), "max": a.max(axis=0).tolist()}


def self_test(obj_path=DEFAULT_OBJ, asset_dir=DEFAULT_ASSET_DIR) -> int:
    results, notes = [], []

    def check(name, ok, detail):
        results.append((name, bool(ok), detail))
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")

    obj_path, asset_dir = Path(obj_path), Path(asset_dir)
    raw = raw_text_numbers(obj_path)
    model = parse_obj(obj_path)
    mtl_files = {look: asset_dir / "mtl" / f"{look}.mtl" for look in LOOKS}
    mtls = {look: parse_mtl(p) for look, p in mtl_files.items()}
    parts = split_by_material(model, mtls["cyan"])

    # 0. the parser against the raw text
    check("parser_matches_raw_text", model.vertices.shape[0] == raw["vertices"] and len(model.faces) == raw["faces"],
          f"parse_obj: {model.vertices.shape[0]} vertices, {len(model.faces)} faces; raw text: {raw['vertices']} vertices, {raw['faces']} faces")
    # 1. triangle count
    total = sum(p.n_triangles for p in parts.values())
    check("triangle_count_of_parts_equals_whole_model", check_triangle_count(parts, raw["triangles"]),
          f"{len(parts)} parts hold {total} triangles; the whole model has {raw['triangles']} (counted from the text: faces with k corners give k-2)")
    # 2. extent
    ext = whole_extent(np.concatenate([p.vertices for p in parts.values()]))
    check("extent_of_parts_equals_extent_of_obj", check_extent_equal(parts, {"min": raw["min"], "max": raw["max"]}),
          f"parts together: x {ext['min'][0]:.4f}..{ext['max'][0]:.4f}, y {ext['min'][1]:.4f}..{ext['max'][1]:.4f}, z {ext['min'][2]:.4f}..{ext['max'][2]:.4f}; "
          f"OBJ text: x {raw['min'][0]:.4f}..{raw['max'][0]:.4f}, y {raw['min'][1]:.4f}..{raw['max'][1]:.4f}, z {raw['min'][2]:.4f}..{raw['max'][2]:.4f} (must be equal)")
    # 3. which side the parts lie on
    cx = {n: p.extent()["center"][0] for n, p in parts.items()}
    gx = {g: group_extent(model, g)["center"][0] for g in FRONT_GROUPS}
    front_ok = all(cx[n] > 0 for n in FRONT_MATERIALS) and all(v > 0 for v in gx.values())
    rear_ok = all(cx[n] < 0 for n in REAR_MATERIALS)
    check("front_parts_have_positive_x_center", front_ok,
          ", ".join(f"material {n} x = {cx[n]:+.4f}" for n in FRONT_MATERIALS) + "; " + ", ".join(f"group {g} x = {v:+.4f}" for g, v in gx.items())
          + f" (bounding-box center, meters). Not used: material glass, x = {cx['glass']:+.4f}, because it also holds the glass pieces of group body")
    check("rear_parts_have_negative_x_center", rear_ok, ", ".join(f"material {n} x = {cx[n]:+.4f}" for n in REAR_MATERIALS))
    # 4. the four MTL files cover the materials and the colors are what the files say
    for look, mtl in mtls.items():
        p = split_by_material(model, mtl)
        bad = [n for n in p if p[n].color != tuple(mtl[n]["kd"])]
        check(f"mtl_{look}_covers_all_materials", set(mtl) >= set(parts) and not bad, f"materials {sorted(parts)}; body Kd = {mtl['body']['kd']}; wrong colors: {bad or 'none'}")
    # 5. mesh files: write, read back
    with tempfile.TemporaryDirectory(prefix="gt_parts_") as tmp:
        info = write_part_meshes(parts, tmp)
        bad = []
        for name, part in parts.items():
            corners = read_stl(info[name]["file"])
            want = part.corners().astype(np.float32)
            if corners.shape != want.shape or not np.array_equal(corners, want):
                bad.append(name)
            elif {tuple(v) for v in corners.reshape(-1, 3)} != {tuple(v) for v in part.vertices.astype(np.float32)}:
                bad.append(name + " (vertex set)")
        check("mesh_files_read_back_to_the_same_vertices", not bad,
              f"{len(parts)} STL files written and read back; the {sum(p.n_triangles for p in parts.values())} triangles equal the model's, float32 exact; different: {bad or 'none'}")
        sizes = {n: Path(i["file"]).stat().st_size for n, i in info.items()}
        check("mesh_file_sizes_follow_the_decoder_rule", all(sizes[n] == 84 + 50 * parts[n].n_triangles for n in parts) and all(1 <= parts[n].n_triangles <= STL_MAX_FACES for n in parts),
              f"size = 84 + 50 * faces for all; largest file {max(sizes.values())} bytes, most faces {max(p.n_triangles for p in parts.values())} (limit {STL_MAX_FACES})")
    # 6. what MuJoCo's compile would test
    bad = []
    for name, part in parts.items():
        vol, area = legacy_volume_and_area(part)
        if part.n_vertices < 4 or not np.isfinite(part.vertices).all() or vol < MJMINVAL or area < MJMINVAL or np.abs(part.vertices).max() > STL_MAX_ABS:
            bad.append(f"{name}: vertices {part.n_vertices}, legacy volume {vol:.3e}, area {area:.3e}")
    vols = {n: legacy_volume_and_area(p)[0] for n, p in parts.items()}
    check("each_part_passes_the_mesh_tests_of_the_compiler", not bad,
          f"at least 4 vertices, finite, legacy volume >= {MJMINVAL:g} and area >= {MJMINVAL:g}: smallest legacy volume {min(vols.values()):.3e} m^3 ({min(vols, key=vols.get)}); problems: {bad or 'none'}")
    # 7. units
    size = max(ext["size"])
    check("units_are_meters", 0.05 < size < 0.30, f"largest extent {size:.4f} m (a 0.14 m drone; millimeters would be 100 times larger, kilometers 1000 times smaller)")
    # 8. the camera frame itself
    pos, rot = camera_frame()
    fwd = -rot[:, 2]
    check("camera_looks_along_drone_x", bool(np.allclose(fwd, [1, 0, 0], atol=1e-9) and np.allclose(rot[:, 1], [0, 0, 1], atol=1e-9)),
          f"euler {CAMERA_EULER_DEG} (xyz, intrinsic): looks along {fwd.round(6).tolist()}, up is {rot[:, 1].round(6).tolist()}; the file says it looks along the nose")

    # 9. meta tests: the checks above must be able to fail
    tampered = {n: Part(p.name, p.vertices.copy(), p.triangles.copy(), p.groups, p.color) for n, p in parts.items()}
    tampered["body"].triangles = tampered["body"].triangles[:-1]
    meta = [
        ("count_check_detects_a_missing_triangle", not check_triangle_count(tampered, raw["triangles"])),
        ("extent_check_detects_a_shifted_part", not check_extent_equal({**parts, "wing": Part("wing", parts["wing"].vertices + [0, 0, 1.0], parts["wing"].triangles, [], None)}, {"min": raw["min"], "max": raw["max"]})),
        ("fan_triangulation_gives_two_triangles_for_a_quad", [t for t, _ in triangulate([[0, 1, 2, 3]])] == [(0, 1, 2), (0, 2, 3)]),
        ("closed_cube_has_no_open_edges", edge_balance(Part("cube", np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)], float),
                                                              np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]), [], None))["open_edges"] == 0),
        ("an_open_triangle_has_open_edges", edge_balance(Part("tri", np.eye(3), np.array([[0, 1, 2]]), [], None))["open_edges"] == 3),
        ("stl_reader_rejects_a_wrong_size", _stl_wrong_size_is_rejected()),
        ("cone_check_sees_a_point_ahead_and_not_a_point_behind", bool(camera_cone_numbers(np.array([[0.2, 0.0, 0.0]]))["inside"][0]) and not bool(camera_cone_numbers(np.array([[-0.2, 0.0, 0.0]]))["inside"][0])
         and not bool(camera_cone_numbers(np.array([[0.2, 0.0, 0.5]]))["inside"][0])),
    ]
    for name, ok in meta:
        check("meta_" + name, ok, "the check logic gives the expected answer on a known case")

    # 10. findings (not pass rules): the camera cone and the size against drone_notes.txt
    rep = cone_report(model, parts)
    print("\nFINDINGS (reported, not pass rules)")
    cam = rep["camera"]
    print(f"camera: pos {cam['pos']}, looks along {cam['looks_along_drone_axis']}, fovy {cam['fovy_deg']} deg, picture {cam['picture']}, "
          f"half angles {cam['half_angle_h_deg']:.2f} deg (horizontal) x {cam['half_angle_v_deg']:.2f} deg (vertical)")
    print(f"vertices inside the view: {rep['total_inside']} of {rep['total_vertices']}; ahead of the lens (positive depth): {rep['total_ahead_of_lens_depth_positive']}; "
          f"with x > {CAMERA_POS[0]}: {rep['vertices_with_x_greater_than_lens_x']}, their highest z = {rep['max_z_of_vertices_ahead_of_lens_x']:.4f} m (notes: z <= -0.004)")
    print("by material part (vertices inside / ahead of lens / within 0.5 mm of the boundary / all vertices / surface sample points inside):")
    for n, r in rep["by_part"].items():
        print(f"  {n:12s} {r['inside']:4d} / {r['ahead_of_lens']:4d} / {r['within_0p5mm_of_the_boundary']:4d} / {r['vertices']:4d} / {r['surface_samples_inside']:5d}")
    print("by OBJ group (inside / ahead of lens / vertices):")
    for g, r in rep["by_group"].items():
        print(f"  {g:12s} {r['inside']:4d} / {r['ahead_of_lens']:4d} / {r['vertices']:4d}")
    lo, hi = np.array(ext["min"]), np.array(ext["max"])
    print(f"size measured: x {hi[0] - lo[0]:.4f} m, y {hi[1] - lo[1]:.4f} m, z {hi[2] - lo[2]:.4f} m; drone_notes.txt says 0.138 x 0.134 x 0.036 m")
    print(f"vertex center of the model: {(lo + hi) / 2}")
    print("part facts (triangles, vertices, signed volume m^3, open edges, unmatched directed edges):")
    for n, p in parts.items():
        eb = edge_balance(p)
        print(f"  {n:12s} {p.n_triangles:4d} {p.n_vertices:4d} {signed_volume(p):+.3e} {eb['open_edges']:4d} {eb['unmatched_directed_edges']:4d}")

    failed = [n for n, ok, _ in results if not ok]
    print(f"\n{'PASS' if not failed else 'FAIL'}: {len(results) - len(failed)} of {len(results)} checks passed" + (f"; failed: {failed}" if failed else ""))
    return 0 if not failed else 1


def _stl_wrong_size_is_rejected() -> bool:
    with tempfile.TemporaryDirectory(prefix="gt_stl_") as tmp:
        p = Path(tmp) / "bad.stl"
        part = Part("x", np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], float), np.array([[0, 1, 2], [0, 1, 3]]), [], None)
        p.write_bytes(stl_bytes(part)[:-1])
        try:
            read_stl(p)
        except ValueError:
            return True
    return False


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--self-test", action="store_true", help="run the checks")
    ap.add_argument("--write", metavar="DIR", help="write one STL per material into DIR")
    ap.add_argument("--obj", default=str(DEFAULT_OBJ))
    ap.add_argument("--mtl", default=str(DEFAULT_MTL))
    args = ap.parse_args()
    if args.self_test:
        sys.exit(self_test(args.obj, Path(args.obj).parent))
    if args.write:
        _, parts = load_parts(args.obj, args.mtl)
        for name, i in write_part_meshes(parts, args.write).items():
            print(f"{name:12s} {i['triangles']:4d} triangles {i['vertices']:4d} vertices  {i['file']}")
        return
    ap.print_help()


if __name__ == "__main__":
    main()
