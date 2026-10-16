"""Pure numpy/scipy part of the PX4 + Gazebo base lap: the track (the four gates of level 0, scaled up for the larger drone),
the reference path, the yaw plan (nose along the direction of flight), the gate-passage counter and the distance to a gate
frame, and the SDF text of a gate. No Gazebo and no MAVSDK import, so tests/test_px4_core.py can run it anywhere.

Frame F (track frame): x east, y north, z up (Gazebo ENU), origin = the spot where the drone is spawned (on the ground).
The Webots track (webots/controllers/base_lap/lap_core.py) is shifted so that its start point is the origin of F and
multiplied by SCALE in x, y and z, because the x500 drone is about 0.65 m wide and a 0.4 m opening would not let it through.
PX4 local frame (NED) from F:  north = y, east = x, down = -z.  PX4 yaw (degrees from north, clockwise) = 90 - heading in F.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "webots" / "controllers" / "base_lap"))
sys.path.insert(0, str(REPO / "controllers"))
import lap_core as lc  # noqa: E402
import nose_yaw as ny  # noqa: E402

SCALE = 4.0
OPENING = 1.6      # side of the square opening, m (Webots 0.4 m x SCALE)
BAR = 0.3          # width of a frame bar, m
DEPTH = 0.1        # thickness of the frame along the passage direction, m
POST = 0.16        # side of the square post under a frame, m
DRONE_RADIUS = 0.37  # x500: motor at 0.25 m from the centre plus a propeller of 0.12 m radius
HOVER_Z = 0.4 * SCALE  # height of the first route point (Webots 0.4 m)
T_ROUTE = 40.0     # seconds for the route (Webots: 22.5 s for a track 4 times smaller)
HOLD_END_S = 3.0   # hover at the end of the route
YAW_RATE_MAX = 1.0  # rad/s
V_MIN = 0.3        # m/s, below this speed the yaw plan holds the heading
PLAN_DT = 0.05
SEQUENCE = list(lc.GATE_SEQUENCE)  # 0, 1, 2, 3, 1
START_XY = np.array(lc.START_XY, dtype=float)


def to_track(p):
    """Webots point (x, y, z) -> frame F."""
    p = np.asarray(p, dtype=float)
    return np.array([SCALE * (p[0] - START_XY[0]), SCALE * (p[1] - START_XY[1]), SCALE * p[2]])


GATES = np.array([[*to_track(g[:3]), g[3]] for g in lc.GATES])  # x, y, z, yaw (rad); normal = (cos yaw, sin yaw)


def build_spline():
    pts = np.vstack([[0.0, 0.0, HOVER_Z]] + [to_track(p) for p in lc.POINTS_AFTER_START])
    return CubicSpline(np.linspace(0.0, T_ROUTE, len(pts)), pts, bc_type=((1, np.zeros(3)), (1, np.zeros(3))))  # at rest at both ends


SPLINE = build_spline()


def ref_pos_vel(tau):
    """Reference position and velocity (frame F) at route time tau (clamped to [0, T_ROUTE]); at rest outside the route."""
    t = min(max(tau, 0.0), T_ROUTE)
    v = SPLINE(t, 1) if 0.0 < tau < T_ROUTE else np.zeros(3)
    return SPLINE(t), v


def yaw_plan():
    """Heading plan (rad, frame F) on a grid of PLAN_DT from route time 0 to T_ROUTE + HOLD_END_S."""
    taus = np.arange(0.0, T_ROUTE + HOLD_END_S + 1e-9, PLAN_DT)
    vel = np.array([ref_pos_vel(t)[1][:2] for t in taus])
    return taus, ny.plan_yaw(vel, PLAN_DT, YAW_RATE_MAX, V_MIN, 0.0)


_TAUS, _YAWS = yaw_plan()


def ref_heading(tau):
    return float(np.interp(tau, _TAUS, _YAWS))


def ned_yaw_deg(heading_rad):
    return 90.0 - math.degrees(heading_rad)


# ------------------------------------------------------------------ gates
def gate_frame(g):
    x, y, z, yaw = GATES[g]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x, y, z]), np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def gate_local(g, p):
    c, R = gate_frame(g)
    return R.T @ (np.asarray(p, float) - c)


_OUT = OPENING + 2 * BAR
_OFF = OPENING / 2 + BAR / 2
FRAME_BOXES = [  # (centre, half sizes) in the gate's own frame; x is the passage direction
    ((0.0, 0.0, _OFF), (DEPTH / 2, _OUT / 2, BAR / 2)), ((0.0, 0.0, -_OFF), (DEPTH / 2, _OUT / 2, BAR / 2)),
    ((0.0, -_OFF, 0.0), (DEPTH / 2, BAR / 2, _OUT / 2)), ((0.0, _OFF, 0.0), (DEPTH / 2, BAR / 2, _OUT / 2)),
]


def box_distance(local_p, box):
    (cx, cy, cz), h = box
    d = np.abs(np.asarray(local_p) - np.array([cx, cy, cz])) - np.array(h)
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def distance_to_frame(p):
    """(distance, gate): smallest distance from the point to a frame bar over all gates."""
    best = (math.inf, -1)
    for g in range(len(GATES)):
        lp = gate_local(g, p)
        for b in FRAME_BOXES:
            d = box_distance(lp, b)
            if d < best[0]:
                best = (d, g)
    return best


class PassageCounter:
    """A passage = the centre goes from the back (local x < 0) to the front (local x >= 0) of the next gate of SEQUENCE
    with |local y| and |local z| below half the opening at the crossing."""

    def __init__(self):
        self.count, self.prev, self.log, self.wrong = 0, None, [], []

    def update(self, t, p):
        p = np.asarray(p, float)
        if self.prev is not None:
            for g in range(len(GATES)):
                a, b = gate_local(g, self.prev), gate_local(g, p)
                if a[0] < 0.0 <= b[0]:
                    u = -a[0] / (b[0] - a[0]) if b[0] != a[0] else 0.0
                    ly, lz = a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2])
                    inside = abs(ly) < OPENING / 2 and abs(lz) < OPENING / 2
                    if self.count < len(SEQUENCE) and g == SEQUENCE[self.count] and inside:
                        self.count += 1
                        self.log.append((t, self.count, g, ly, lz))
                    elif inside or math.hypot(ly, lz) < 1.0:
                        self.wrong.append((t, g, ly, lz))
        self.prev = p
        return self.count


def gate_sdf(index, ground_z, offset_xy=(0.0, 0.0)):
    """SDF text of gate `index` (0 based) as a static model: four orange bars and a grey post down to 6 m below the ground
    reference. Positions are in frame F plus the world offset: offset_xy is the world x, y of the origin of F and ground_z the
    world height of the ground reference."""
    x, y, z, yaw = GATES[index]
    post_top = -(OPENING / 2 + BAR)
    post_len = z + 6.0 + post_top          # from 6 m below the ground reference up to the lower edge of the frame
    post_mid = post_top - post_len / 2

    def box(name, c, h, color):
        sz = f"{2 * h[0]} {2 * h[1]} {2 * h[2]}"
        return (f'<visual name="{name}_v"><pose>{c[0]} {c[1]} {c[2]} 0 0 0</pose><geometry><box><size>{sz}</size></box></geometry>'
                f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse><emissive>{color}</emissive></material></visual>'
                f'<collision name="{name}_c"><pose>{c[0]} {c[1]} {c[2]} 0 0 0</pose><geometry><box><size>{sz}</size></box></geometry></collision>')

    orange, grey = "1 0.45 0 1", "0.35 0.35 0.4 1"
    parts = [box(f"bar{k}", c, h, orange) for k, (c, h) in enumerate(FRAME_BOXES)]
    parts.append(box("post", (0.0, 0.0, post_mid), (POST / 2, POST / 2, post_len / 2), grey))
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="gate_{index + 1}"><static>true</static>'
            f'<pose>{x + offset_xy[0]} {y + offset_xy[1]} {z + ground_z} 0 0 {yaw}</pose><link name="link">{"".join(parts)}</link></model></sdf>')
