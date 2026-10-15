"""Pure numpy/scipy part of the Webots lap: the planned path, the cascaded controller, the gate-passage counter and the
distance to a gate frame. No Webots import, so tests/webots_controller_sim.py can run it against a simple model.

Frames: world x y z with z up (Webots ENU); body x = nose, y = left, z = up; R maps body to world.
Propeller layout and signs are those of the Webots Crazyflie (Crazyflie.proto, Apache-2.0, see webots/NOTICE.md):
m1 (+x,-y), m2 (-x,-y), m3 (-x,+y), m4 (+x,+y); thrust 4e-5 * w^2 per motor; yaw reaction torque 2.4e-6 * w^2,
positive (+z on the body) for m1 and m3 (which turn clockwise, commanded with a negative speed), negative for m2, m4.
The mixing follows the signs of the Bitcraze example controller (pid_controller.py, MIT): the roll command raises
m3 and m4 (the +y side), the pitch command raises m1 and m4 (the +x side), the yaw command raises m1 and m3.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.interpolate import CubicSpline

# ----------------------------------------------------------------------------------------------- the plan
START_XY = (-1.5, 0.75)
START_Z = 0.015  # the Webots Crazyflie rests with its centre 0.015 m above the floor (bounding cylinder height 0.03)
# the 11 points after the start point: the example path of the level 0 flight (same numbers as run A)
POINTS_AFTER_START = np.array([
    [-1.0, 0.75, 0.4], [0.3, 0.35, 0.7], [1.3, -0.15, 0.9], [0.85, 0.85, 1.2], [-0.5, -0.05, 0.7], [-1.2, -0.2, 0.8],
    [-1.2, -0.2, 1.2], [-0.0, -0.7, 1.2], [1.2, -0.15, 1.2], [1.05, 0.75, 1.2], [0.25, 1.25, 1.2]])
T_TOTAL = 22.5
# gates of level 0: (x, y, z, yaw_rad). The gate normal (direction of a forward passage) is (cos yaw, sin yaw).
GATES = np.array([[0.5, 0.25, 0.7, -0.78], [1.05, 0.75, 1.2, 2.35], [-1.0, -0.25, 0.7, 3.14], [0.0, -0.75, 1.2, 0.0]])
GATE_SEQUENCE = [0, 1, 2, 3, 1]
POLES = np.array([[0.0, 0.75, 1.55], [1.0, 0.25, 1.55], [-1.5, -0.25, 1.55], [-0.5, -0.75, 1.55]])  # x, y, top z
POLE_RADIUS = 0.015
OPENING_HALF = 0.2  # opening 0.4 m
# collision boxes of a gate in its own frame (centre, half sizes), from gate.xml col_* and gate_stand
GATE_BOXES = [
    ((0.0, 0.0, 0.28), (0.01, 0.36, 0.08)), ((0.0, 0.0, -0.28), (0.01, 0.36, 0.08)),
    ((0.0, -0.28, 0.0), (0.01, 0.08, 0.36)), ((0.0, 0.28, 0.0), (0.01, 0.08, 0.36)),
    ((0.0, 0.0, -0.86), (0.05, 0.05, 0.5)),
]
FRAME_BOXES = GATE_BOXES[:4]


def build_spline(start):
    pts = np.vstack([np.asarray(start, float).reshape(1, 3), POINTS_AFTER_START])
    return CubicSpline(np.linspace(0.0, T_TOTAL, len(pts)), pts)


def gate_frame(g):
    x, y, z, yaw = GATES[g]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x, y, z]), np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # local -> world


def gate_local(g, p):
    c, R = gate_frame(g)
    return R.T @ (np.asarray(p, float) - c)


def box_distance(local_p, box):
    """Distance from a point to a solid box (0 inside)."""
    (cx, cy, cz), h = box
    d = np.abs(np.asarray(local_p) - np.array([cx, cy, cz])) - np.array(h)
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def distance_to_frame(p):
    """Smallest distance from the point to a collision box of a gate frame (4 boxes per gate), over all gates: (distance, gate)."""
    best = (math.inf, -1)
    for g in range(len(GATES)):
        lp = gate_local(g, p)
        for b in FRAME_BOXES:
            d = box_distance(lp, b)
            if d < best[0]:
                best = (d, g)
    return best


class PassageCounter:
    """Counts passages in the order of GATE_SEQUENCE. A passage = the centre of the drone goes from the back (local x < 0)
    to the front (local x >= 0) of the next gate, with |local y| < 0.2 and |local z| < 0.2 at the crossing."""

    def __init__(self):
        self.count = 0
        self.prev = None
        self.log = []  # (time, passage number, gate, ly, lz)
        self.wrong = []  # crossings in the right direction that did not count (time, gate, ly, lz)

    def update(self, t, p):
        p = np.asarray(p, float)
        if self.prev is not None:
            for g in range(len(GATES)):
                a, b = gate_local(g, self.prev), gate_local(g, p)
                if a[0] < 0.0 <= b[0]:
                    u = -a[0] / (b[0] - a[0]) if b[0] != a[0] else 0.0
                    ly, lz = a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2])
                    inside = abs(ly) < OPENING_HALF and abs(lz) < OPENING_HALF
                    if self.count < len(GATE_SEQUENCE) and g == GATE_SEQUENCE[self.count] and inside:
                        self.count += 1
                        self.log.append((t, self.count, g, ly, lz))
                    elif inside or math.hypot(ly, lz) < 0.6:
                        self.wrong.append((t, g, ly, lz))
        self.prev = p
        return self.count


# ----------------------------------------------------------------------------------------------- the controller
MASS = 0.05
G = 9.81
KT = 4e-5
KQ = 2.4e-6
ARM = 0.031
INERTIA = np.diag([3.5e-5, 3.5e-5, 6.25e-5])  # cylinder of mass 0.05 kg, radius 0.05, height 0.03 (Webots takes it from the bounding object)
W_MAX = 600.0


def vee(M):
    return np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]]) * 0.5


class LapController:
    """position P -> velocity PI -> thrust vector -> attitude P -> body-rate P -> torques -> four motor speeds."""

    def __init__(self, dt, yaw_plan_fn, spline, kp_pos=4.0, kp_vel=5.0, ki_vel=1.0, kp_att=(14.0, 14.0, 5.0), kp_rate=(45.0, 45.0, 25.0),
                 v_max=1.5, tilt_max_deg=25.0, ki_thrust=1.5, ground_tilt_deg=4.0):
        self.dt, self.yaw_plan, self.spline = dt, yaw_plan_fn, spline
        self.ki_thrust, self.s = ki_thrust, 1.0  # s scales the thrust: it learns the real hover thrust (the model may be off)
        self.ground_tilt = math.radians(ground_tilt_deg)
        self.kp_pos, self.kp_vel, self.ki_vel = kp_pos, kp_vel, ki_vel
        self.kp_att, self.kp_rate = np.array(kp_att), np.array(kp_rate)
        self.v_max, self.tilt_max = v_max, math.radians(tilt_max_deg)
        self.int = np.zeros(3)
        self.last = {}

    def reference(self, t):
        tt = min(t, T_TOTAL)
        return self.spline(tt), self.spline(tt, 1), self.spline(tt, 2)

    def step(self, t, p, v, R, w_body):
        """Returns the four motor speeds in rad/s (magnitudes; the caller applies the signs)."""
        p_ref, v_ref, a_ref = self.reference(t)
        v_sp = v_ref + self.kp_pos * (p_ref - p)
        n = np.linalg.norm(v_sp)
        if n > self.v_max:
            v_sp *= self.v_max / n
        ev = v_sp - v
        self.s = float(np.clip(self.s + self.ki_thrust * ev[2] * self.dt, 0.6, 3.0))
        self.int = np.clip(self.int + ev * self.dt, -1.0, 1.0)
        a_cmd = a_ref + self.kp_vel * ev + self.ki_vel * self.int
        F = MASS * (a_cmd + np.array([0.0, 0.0, G]))
        if F[2] < 1e-3:
            F[2] = 1e-3
        tilt_lim = self.tilt_max if p[2] > 0.12 else self.ground_tilt  # stay level while still on the floor
        horiz = np.linalg.norm(F[:2])
        if horiz > F[2] * math.tan(tilt_lim):
            F[:2] *= F[2] * math.tan(tilt_lim) / horiz
        T = max(float(F @ R[:, 2]), 0.0) * self.s
        z_des = F / np.linalg.norm(F)
        yaw = self.yaw_plan(t)
        xc = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        y_des = np.cross(z_des, xc)
        y_des /= np.linalg.norm(y_des)
        x_des = np.cross(y_des, z_des)
        R_des = np.column_stack([x_des, y_des, z_des])
        e_R = vee(R_des.T @ R - R.T @ R_des)  # body frame
        w_sp = -self.kp_att * e_R
        tau = INERTIA @ (self.kp_rate * (w_sp - w_body)) + np.cross(w_body, INERTIA @ w_body)
        w = mix(T, tau)
        self.last = {"p_ref": p_ref, "v_ref": v_ref, "T": T, "s": self.s, "yaw_cmd": yaw, "e_R": e_R, "tau": tau, "w_sp": w_sp}
        return w


def mix(T, tau):
    a = T / 4.0
    ux, uy, uz = tau[0] / (4 * ARM), tau[1] / (4 * ARM), tau[2] / (4 * (KQ / KT))
    F = np.array([a - ux - uy + uz, a - ux + uy - uz, a + ux + uy + uz, a + ux - uy - uz])
    return np.clip(np.sqrt(np.maximum(F, 0.0) / KT), 0.0, W_MAX)


def forces_from_speeds(w):
    """Inverse of mix for the offline model: thrust and body torque from the four speeds."""
    F = KT * np.asarray(w) ** 2
    T = F.sum()
    tx = ARM * (-F[0] - F[1] + F[2] + F[3])
    ty = -ARM * (F[0] - F[1] - F[2] + F[3])
    tz = (KQ / KT) * (F[0] - F[1] + F[2] - F[3])
    return T, np.array([tx, ty, tz])


def quat_xyzw_from_R(R):
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


# gain sets tried one after the other by tests/webots_base_run.py (index = BASE_LAP_ATTEMPT): the real inertia of the Webots
# drone is not known, so the first sets are the safest ones
GAIN_SETS = [
    dict(kp_pos=3.0, kp_vel=4.0, ki_vel=1.0, kp_att=(8.0, 8.0, 3.0), kp_rate=(25.0, 25.0, 12.0)),
    dict(kp_pos=2.5, kp_vel=3.0, ki_vel=0.8, kp_att=(5.0, 5.0, 2.0), kp_rate=(12.0, 12.0, 6.0)),
    dict(kp_pos=4.0, kp_vel=5.0, ki_vel=1.0, kp_att=(14.0, 14.0, 5.0), kp_rate=(45.0, 45.0, 25.0)),
    dict(kp_pos=2.0, kp_vel=2.5, ki_vel=0.5, kp_att=(3.5, 3.5, 1.5), kp_rate=(6.0, 6.0, 3.0)),
    dict(kp_pos=2.0, kp_vel=2.5, ki_vel=0.5, kp_att=(2.5, 2.5, 1.0), kp_rate=(3.0, 3.0, 2.0)),
    dict(kp_pos=1.5, kp_vel=2.0, ki_vel=0.4, kp_att=(2.0, 2.0, 1.0), kp_rate=(2.0, 2.0, 1.0)),
]
