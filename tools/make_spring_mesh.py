"""Make the coil spring mesh of the new gate (assets/gates/square/spring.stl) with numpy alone.

The mesh is one red coil spring: a round wire wound around a straight line. The line is the z axis
of the mesh file, from z = 0 to z = LENGTH (the distance between the two end points of a spring
in gate.xml). The wire runs straight on the axis for a short lead at each end, then spirals out to
the coil radius, winds TURNS times, and spirals back onto the axis. The ends are flat caps.

gate.xml places the mesh with `pos` = start point and `quat` = rotation of the z axis onto the line
(printed by this script). MuJoCo re-centers and re-aligns a mesh in its compiler, but composes the
offsets back into the geom pose, so the vertices land where the file coordinates say (MuJoCo
XMLreference, element asset/mesh and attribute geom/type "mesh").

Usage:  python tools/make_spring_mesh.py [--out assets/gates/square/spring.stl]
Only numpy and the standard library are imported.
"""

import argparse
import math
from pathlib import Path

import numpy as np

# End points of the four ropes in LSY's gate.xml (same for the springs), in gate coordinates [m].
ENDPOINTS = {
    "rope_top_left": ((0.0, -0.34, 0.34), (0.0, -0.29, 0.29)),
    "rope_top_right": ((0.0, 0.34, 0.34), (0.0, 0.29, 0.29)),
    "rope_bottom_left": ((0.0, -0.34, -0.34), (0.0, -0.29, -0.29)),
    "rope_bottom_right": ((0.0, 0.34, -0.34), (0.0, 0.29, -0.29)),
}

TURNS = 5
COIL_RADIUS = 0.008  # distance of the wire center from the axis [m]
WIRE_RADIUS = 0.002  # radius of the wire itself [m]
LEAD = 0.006  # straight wire on the axis at each end [m]
RAMP_ANGLE = math.pi  # the coil radius grows from 0 to COIL_RADIUS over this winding angle
STEPS_PER_TURN = 48  # path points per turn
SIDES = 12  # corners of the wire cross-section


def spring_length() -> float:
    lengths = {math.dist(a, b) for a, b in ENDPOINTS.values()}
    low, high = min(lengths), max(lengths)
    assert high - low < 1e-12, f"the four springs differ in length: {lengths}"
    return high


def smoothstep(s: np.ndarray) -> np.ndarray:
    s = np.clip(s, 0.0, 1.0)
    return s * s * (3.0 - 2.0 * s)


def centerline(length: float) -> np.ndarray:
    """Points of the wire center line: lead, spiral out, TURNS turns, spiral in, lead."""
    n = TURNS * STEPS_PER_TURN
    phi_max = 2.0 * math.pi * TURNS
    phi = np.linspace(0.0, phi_max, n + 1)
    z = LEAD + (length - 2.0 * LEAD) * phi / phi_max
    ramp = np.minimum(phi, phi_max - phi) / RAMP_ANGLE
    r = COIL_RADIUS * smoothstep(ramp)
    helix = np.stack([r * np.cos(phi), r * np.sin(phi), z], axis=1)
    first = np.array([[0.0, 0.0, 0.0]])
    last = np.array([[0.0, 0.0, length]])
    return np.concatenate([first, helix, last], axis=0)


def sweep(path: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Sweep a circle along the path with a rotation-minimizing frame. Returns vertices, faces."""
    n = len(path)
    tangent = np.zeros_like(path)
    tangent[1:-1] = path[2:] - path[:-2]
    tangent[0] = path[1] - path[0]
    tangent[-1] = path[-1] - path[-2]
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)

    normal = np.zeros_like(path)
    normal[0] = np.array([1.0, 0.0, 0.0])
    normal[0] -= tangent[0] * np.dot(normal[0], tangent[0])
    normal[0] /= np.linalg.norm(normal[0])
    for i in range(1, n):  # parallel transport: project the last normal onto the new normal plane
        v = normal[i - 1] - tangent[i] * np.dot(normal[i - 1], tangent[i])
        normal[i] = v / np.linalg.norm(v)
    binormal = np.cross(tangent, normal)

    theta = 2.0 * math.pi * np.arange(SIDES) / SIDES
    rings = (
        path[:, None, :]
        + WIRE_RADIUS * (np.cos(theta)[None, :, None] * normal[:, None, :]
                         + np.sin(theta)[None, :, None] * binormal[:, None, :])
    )
    vertices = rings.reshape(-1, 3)
    start_center, end_center = n * SIDES, n * SIDES + 1
    vertices = np.concatenate([vertices, path[:1], path[-1:]], axis=0)

    faces = []
    for i in range(n - 1):
        for j in range(SIDES):
            a, b = i * SIDES + j, i * SIDES + (j + 1) % SIDES
            c, d = (i + 1) * SIDES + j, (i + 1) * SIDES + (j + 1) % SIDES
            faces += [(a, c, d), (a, d, b)]
    for j in range(SIDES):  # caps
        a, b = j, (j + 1) % SIDES
        faces.append((start_center, a, b))  # the side faces use the ring edge b -> a here
        e, f = (n - 1) * SIDES + j, (n - 1) * SIDES + (j + 1) % SIDES
        faces.append((end_center, f, e))  # the side faces use the ring edge e -> f here
    faces = np.array(faces, dtype=np.int64)

    if signed_volume(vertices, faces) < 0:  # make the triangles face outward
        faces = faces[:, [0, 2, 1]]
    return vertices, faces


def signed_volume(vertices: np.ndarray, faces: np.ndarray) -> float:
    a, b, c = (vertices[faces[:, k]] for k in range(3))
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def write_binary_stl(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    tri = vertices[faces].astype(np.float32)
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    normal /= np.linalg.norm(normal, axis=1, keepdims=True)
    record = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    data = np.zeros(len(tri), dtype=record)
    data["n"], data["v"] = normal, tri
    with open(path, "wb") as f:
        f.write(b"coil spring for the gate, made by tools/make_spring_mesh.py".ljust(80, b" "))
        f.write(np.uint32(len(tri)).tobytes())
        f.write(data.tobytes())


def read_binary_stl(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    count = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
    assert len(raw) == 84 + 50 * count, "file size does not match the triangle count"
    record = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(raw[84:], dtype=record)["v"].astype(np.float64)


def quat_z_to(direction: np.ndarray) -> np.ndarray:
    """Quaternion (w x y z) of the shortest rotation that turns (0, 0, 1) onto `direction`."""
    z = np.array([0.0, 0.0, 1.0])
    d = direction / np.linalg.norm(direction)
    axis = np.cross(z, d)
    s = np.linalg.norm(axis)
    angle = math.atan2(s, float(np.dot(z, d)))
    if s < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0]) if np.dot(z, d) > 0 else np.array([0.0, 1.0, 0.0, 0.0])
    axis /= s
    return np.concatenate([[math.cos(angle / 2)], axis * math.sin(angle / 2)])


def rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    w, xyz = q[0], q[1:]
    return v + 2.0 * np.cross(xyz, np.cross(xyz, v) + w * v)


def report(path: Path, length: float) -> None:
    tri = read_binary_stl(path)  # read the file back, not the arrays in memory
    flat = tri.reshape(-1, 3)
    unique, inverse = np.unique(flat, axis=0, return_inverse=True)
    face = inverse.reshape(-1, 3)
    edges = {}
    for a, b, c in face:
        for u, v in ((a, b), (b, c), (c, a)):
            edges[(u, v)] = edges.get((u, v), 0) + 1
    closed = all(edges.get((v, u), 0) == n for (u, v), n in edges.items()) and set(edges.values()) == {1}
    area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    print(f"file: {path}  ({path.stat().st_size} bytes)")
    print(f"triangles: {len(tri)}  unique vertices: {len(unique)}")
    print(f"closed and consistently oriented: {closed}")
    print(f"signed volume: {signed_volume(unique, face):.4e} m^3 (positive = outward)")
    print(f"smallest triangle area: {area.min():.3e} m^2 (MuJoCo's limit is 1e-14)")
    lo, hi = unique.min(axis=0), unique.max(axis=0)
    print(f"bounding box min {np.round(lo, 5).tolist()} max {np.round(hi, 5).tolist()} [m]")
    print(f"axis length (z extent): {hi[2] - lo[2]:.6f} m, expected {length:.6f} m")
    print("placement in gate.xml (pos = start point, quat = rotation of z onto the line):")
    for name, (p0, p1) in ENDPOINTS.items():
        p0, p1 = np.array(p0), np.array(p1)
        q = quat_z_to(p1 - p0)
        back = rotate(q, np.array([0.0, 0.0, length])) + p0
        err = np.abs(back - p1).max()
        print(f"  {name}: pos=\"{p0[0]:g} {p0[1]:g} {p0[2]:g}\" quat=\"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\""
              f"  (mesh end lands within {err:.1e} m of the second end point)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    default = Path(__file__).resolve().parent.parent / "assets" / "gates" / "square" / "spring.stl"
    parser.add_argument("--out", type=Path, default=default)
    args = parser.parse_args()
    length = spring_length()
    vertices, faces = sweep(centerline(length))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_binary_stl(args.out, vertices, faces)
    report(args.out, length)


if __name__ == "__main__":
    main()
