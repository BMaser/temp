"""Pure numpy/scipy part of the PX4 + Gazebo race film FR: the hard race route (hard_race_v1.json, scale 4), the reference path, the yaw plan,
the gate-passage counter, the distance to every gate frame, pole and box, the phase table, the paths of the moving objects, and the lap
bookkeeping. No Gazebo, no MAVSDK and no PIL import, so tests/test_race_core.py can run it anywhere.

Frame F (track frame, metres): x east, y north, z up (Gazebo ENU), origin = the spot where the drone is spawned (on the ground).
hard_race_v1.json is written for a drone of Crazyflie size (scale 1); the x500 is about 0.65 m wide, so everything is multiplied by SCALE = 4
(the same scale the PX4 + Gazebo base lap used). PX4 local frame (NED) from F: north = y, east = x, down = -z. PX4 yaw (degrees from north,
clockwise) = 90 - heading in F.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "controllers"))
import nose_yaw as ny  # noqa: E402

SPEC = json.loads((Path(__file__).resolve().parent / "hard_race_v1.json").read_text())
SCALE = 4.0
OPENING = SPEC["gate_opening_m"] * SCALE          # 2.0 m
OUTER = SPEC["gate_frame_outer_m"] * SCALE        # 2.8 m
BAR = (OUTER - OPENING) / 2.0                      # 0.4 m frame bar
DEPTH = 0.2                                        # thickness of a frame along the passage direction
POST = 0.3                                         # side of the square post under a frame
DRONE_RADIUS = 0.37                                # x500: motor 0.25 m from the centre plus a propeller of 0.12 m radius
V_ROUTE = 1.1                                      # m/s along the path (the route is as long as the path, about 67 m)
HOLD_START_S = 2.0                                 # hover at the first route point before the route starts
HOLD_END_S = 1.0
YAW_RATE_MAX = 1.0                                 # rad/s
V_MIN = 0.3
PLAN_DT = 0.05
CLIMB_RATE = 0.8                                   # m/s
DESCENT_RATE = 0.6                                 # m/s, offboard descent over the end pad
LAND_HANDOVER_M = 1.0                              # above this height over the end pad PX4's own land mode takes over
GATE_IDS = [g["id"] for g in SPEC["gates"]]
N_GATES = len(GATE_IDS)


def S(v):
    return np.asarray(v, dtype=float) * SCALE


GATES = np.array([[*S(g["center"]), math.radians(g["yaw_deg"])] for g in SPEC["gates"]])  # x, y, z, yaw (rad); normal = (cos yaw, sin yaw)
POLES = [dict(id=o["id"], xy=S(o["center_xy"]), radius=o["radius"] * SCALE, height=o["height"] * SCALE) for o in SPEC["obstacles"] if o["kind"] == "vertical pole"]
BOXES = [dict(id=o["id"], xy=S(o["center_xy"]), half=o["size_xy"] * SCALE / 2.0, height=o["height"] * SCALE) for o in SPEC["obstacles"] if o["kind"].startswith("box")]
START_PAD = S(SPEC["start_pad"])
END_PAD = S(SPEC["end_pad"])
PAD_SIZE = 3.2                                     # side of a landing pad (m)


def waypoints():
    """Waypoints in frame F from the hint of the route file: take-off point, then for every gate a point before its centre along the
    passage direction, the centre and a point after, plus the detours after G1, G3, G4, then the end pad at hover height."""
    h = SPEC["waypoints_hint"]
    det = h["detours"]
    pts = [S([0.0, 0.0, 0.8])]
    for g in SPEC["gates"]:
        c = np.array(g["center"], float)
        d = np.array([math.cos(math.radians(g["yaw_deg"])), math.sin(math.radians(g["yaw_deg"])), 0.0])
        pts += [S(c - 0.35 * d), S(c), S(c + 0.35 * d)]
        if f"after_{g['id']}" in det:
            pts.append(S(det[f"after_{g['id']}"]))
    pts.append(S([SPEC["end_pad"][0], SPEC["end_pad"][1], 0.8]))
    return np.array(pts)


WP = waypoints()
HOVER_Z = float(WP[0, 2])


def build_spline():
    seg = np.linalg.norm(np.diff(WP, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    t = s / V_ROUTE
    return CubicSpline(t, WP, bc_type=((1, np.zeros(3)), (1, np.zeros(3)))), float(s[-1]), float(t[-1])


SPLINE, PATH_LEN, T_ROUTE = build_spline()


def ref_pos_vel(tau):
    """Reference position and velocity (frame F) at route time tau (clamped to [0, T_ROUTE]); at rest outside the route."""
    t = min(max(tau, 0.0), T_ROUTE)
    v = SPLINE(t, 1) if 0.0 < tau < T_ROUTE else np.zeros(3)
    return SPLINE(t), v


def yaw_plan():
    taus = np.arange(0.0, T_ROUTE + HOLD_END_S + 1e-9, PLAN_DT)
    vel = np.array([ref_pos_vel(t)[1][:2] for t in taus])
    return taus, ny.plan_yaw(vel, PLAN_DT, YAW_RATE_MAX, V_MIN, 0.0)


_TAUS, _YAWS = yaw_plan()


def ref_heading(tau):
    return float(np.interp(tau, _TAUS, _YAWS))


def ned_yaw_deg(heading_rad):
    return 90.0 - math.degrees(heading_rad)


# ------------------------------------------------------------------ gates, poles, box
def gate_frame(g):
    x, y, z, yaw = GATES[g]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x, y, z]), np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def gate_local(g, p):
    c, R = gate_frame(g)
    return R.T @ (np.asarray(p, float) - c)


_OFF = OPENING / 2 + BAR / 2
FRAME_BOXES = [  # (centre, half sizes) in the gate's own frame; x is the passage direction
    ((0.0, 0.0, _OFF), (DEPTH / 2, OUTER / 2, BAR / 2)), ((0.0, 0.0, -_OFF), (DEPTH / 2, OUTER / 2, BAR / 2)),
    ((0.0, -_OFF, 0.0), (DEPTH / 2, BAR / 2, OUTER / 2)), ((0.0, _OFF, 0.0), (DEPTH / 2, BAR / 2, OUTER / 2)),
]


def post_box(g):
    """The grey post of gate g in the gate's own frame: (centre, half sizes). From the ground (frame z = 0) up to the lower edge of the frame."""
    z = GATES[g, 2]
    top = -(OUTER / 2)                       # lower edge of the frame, gate frame
    low = -z                                 # ground, gate frame
    return ((0.0, 0.0, (top + low) / 2), (POST / 2, POST / 2, (top - low) / 2))


def box_distance(local_p, box):
    (cx, cy, cz), h = box
    d = np.abs(np.asarray(local_p) - np.array([cx, cy, cz])) - np.array(h)
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def distance_to_gates(p):
    """(distance, gate id index): smallest distance from the point to a frame bar or a post over all gates."""
    best = (math.inf, -1)
    for g in range(N_GATES):
        lp = gate_local(g, p)
        for b in FRAME_BOXES + [post_box(g)]:
            d = box_distance(lp, b)
            if d < best[0]:
                best = (d, g)
    return best


def distance_to_poles(p):
    """(distance, pole index): smallest distance from the point to the surface of a pole (a cylinder from the ground up to its height)."""
    best = (math.inf, -1)
    for i, o in enumerate(POLES):
        dxy = math.hypot(p[0] - o["xy"][0], p[1] - o["xy"][1]) - o["radius"]
        dz = max(p[2] - o["height"], -p[2], 0.0)
        d = math.hypot(max(dxy, 0.0), dz)
        if d < best[0]:
            best = (d, i)
    return best


def distance_to_boxes(p):
    best = (math.inf, -1)
    for i, o in enumerate(BOXES):
        c = (o["xy"][0], o["xy"][1], o["height"] / 2)
        d = box_distance(p, (c, (o["half"], o["half"], o["height"] / 2)))
        if d < best[0]:
            best = (d, i)
    return best


def distances_all(p):
    """Distance from the point (frame F) to the nearest solid thing of each kind: gate (frames and posts), pole, box."""
    g, gi = distance_to_gates(p)
    po, pi_ = distance_to_poles(p)
    b, bi = distance_to_boxes(p)
    return {"gate": (g, GATE_IDS[gi] if gi >= 0 else None), "pole": (po, POLES[pi_]["id"] if pi_ >= 0 else None), "box": (b, BOXES[bi]["id"] if bi >= 0 else None)}


class PassageCounter:
    """A passage = the centre goes from the back (local x < 0) to the front (local x >= 0) of the NEXT gate of the sequence G1..G7 with |local y|
    and |local z| below half the opening at the crossing. `count` is the number of gates passed in order."""

    def __init__(self, start_at=0):
        self.count, self.prev, self.log, self.wrong = start_at, None, [], []

    def update(self, t, p):
        p = np.asarray(p, float)
        if self.prev is not None:
            for g in range(N_GATES):
                a, b = gate_local(g, self.prev), gate_local(g, p)
                if a[0] < 0.0 <= b[0]:
                    u = -a[0] / (b[0] - a[0]) if b[0] != a[0] else 0.0
                    ly, lz = a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2])
                    inside = abs(ly) < OPENING / 2 and abs(lz) < OPENING / 2
                    if self.count < N_GATES and g == self.count and inside:
                        self.count += 1
                        self.log.append((t, self.count, GATE_IDS[g], ly, lz))
                    elif inside or math.hypot(ly, lz) < 1.0:
                        self.wrong.append((t, GATE_IDS[g], ly, lz))
        self.prev = p
        return self.count


# ------------------------------------------------------------------ the six phases (owner's order: switched by TIME since the take-off command)
PHASES = [
    dict(id=1, name="CITY ONLY", t0=0.0, t1=10.0, tag="NATIVE", feature="city with moving person, car and truck, sun glare, reflections"),
    dict(id=2, name="WIND + TURBULENCE", t0=10.0, t1=20.0, tag="NATIVE", feature="steady wind and gusts"),
    dict(id=3, name="RAIN", t0=20.0, t1=30.0, tag="NATIVE", feature="rain drops (particle emitter)"),
    dict(id=4, name="CAMERA NOISE", t0=30.0, t1=40.0, tag="NATIVE", feature="sensor noise of the nose camera"),
    dict(id=5, name="MOTION BLUR", t0=40.0, t1=50.0, tag="ADDED BY OUR CODE", feature="blur of the nose camera picture"),
    dict(id=6, name="NIGHT + CLOUDS", t0=50.0, t1=1e9, tag="NATIVE", feature="night light, sky dome with clouds"),
]


def phase_at(tp):
    for ph in PHASES:
        if ph["t0"] <= tp < ph["t1"]:
            return ph
    return PHASES[-1] if tp >= 0 else PHASES[0]


# ------------------------------------------------------------------ the world around the course (frame F; the world origin is the spawn point)
# The course: x from about -2 to 24, y from about -8 to 6 (+ the end pad at y = 8). The streets run along x. The paths of the moving objects are
# race_world.MOVERS (car and truck on the two streets, the person on the south sidewalk); tests/test_race_world.py checks their distance to the route.


def loop_pose(loop, speed, t, phase_s=0.0):
    """(x, y, heading rad) of an object that follows the closed polyline `loop` at `speed`, at time t (s)."""
    pts = np.array(loop + [loop[0]], float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    s = (speed * t + phase_s) % cum[-1]
    k = int(np.searchsorted(cum, s, side="right") - 1)
    k = min(k, len(seg) - 1)
    u = (s - cum[k]) / seg[k]
    p = pts[k] + u * (pts[k + 1] - pts[k])
    d = pts[k + 1] - pts[k]
    return float(p[0]), float(p[1]), math.atan2(d[1], d[0])
