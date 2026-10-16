"""The flight route, the same for both simulators. Pure Python, no packages.

Phases (seconds since the start of the recording):
  0 to CLIMB_S             climb straight up to HEIGHT_M above the start (nose along the start heading)
  CLIMB_S to CLIMB_S+LOOP_S   one figure eight (a Gerono lemniscate), 12 m north-south and 6 m east-west, nose along the direction of flight
  then until TOTAL_S       hover at the end of the loop

Frames: north, east, down (NED), the same as both simulators. yaw = 0 points north, positive turns toward east.
"""

import math

HEIGHT_M = 3.0
CLIMB_S = 6.0
LOOP_S = 24.0
TOTAL_S = 36.0
A_M = 6.0   # figure eight: north = A sin u        (12 m long)
B_M = 6.0   # figure eight: east  = B sin u cos u  (3 m to each side)
KP = 1.0
V_MAX = 3.0
YAW_RATE_MAX = math.radians(90.0)


def _ease(tau: float):
    """Smooth 0..1 progress with zero speed at both ends: s = tau - sin(2 pi tau) / (2 pi). Returns (s, ds/dtau)."""
    tau = min(max(tau, 0.0), 1.0)
    return tau - math.sin(2 * math.pi * tau) / (2 * math.pi), 1.0 - math.cos(2 * math.pi * tau)


def loop_point(u: float):
    """Position and derivative d/du of the figure eight at angle u, relative to the loop centre (north, east)."""
    n = A_M * math.sin(u)
    e = B_M * math.sin(u) * math.cos(u)
    dn = A_M * math.cos(u)
    de = B_M * math.cos(2 * u)
    return (n, e), (dn, de)


def plan_xy(start_n: float, start_e: float, points: int = 200):
    """The planned horizontal path for the map (north, east)."""
    pts = [[start_n, start_e]]
    for k in range(points + 1):
        (n, e), _ = loop_point(2 * math.pi * k / points)
        pts.append([start_n + n, start_e + e])
    return pts


def target(t: float, start_n: float, start_e: float, start_d: float):
    """Where the drone should be at time t: (north, east, down, v_north, v_east)."""
    z = start_d - HEIGHT_M
    if t < CLIMB_S:
        return start_n, start_e, z, 0.0, 0.0
    tau = (t - CLIMB_S) / LOOP_S
    s, ds = _ease(tau)
    u = 2 * math.pi * s
    du_dt = 2 * math.pi * ds / LOOP_S
    (n, e), (dn, de) = loop_point(u)
    return start_n + n, start_e + e, z, dn * du_dt, de * du_dt


class Controller:
    """Velocity command = feed-forward + KP * position error, clipped to V_MAX; yaw follows the velocity, rate limited."""

    def __init__(self, start_n: float, start_e: float, start_d: float, start_yaw: float):
        self.s = (start_n, start_e, start_d)
        self.yaw = start_yaw
        self.last_t = None

    def command(self, t: float, n: float, e: float):
        """Returns (v_north, v_east, z_down_target, yaw_rad) for time t and the measured north/east position."""
        tn, te, tz, fvn, fve = target(t, *self.s)
        vn = fvn + KP * (tn - n)
        ve = fve + KP * (te - e)
        sp = math.hypot(vn, ve)
        if sp > V_MAX:
            vn, ve = vn * V_MAX / sp, ve * V_MAX / sp
        if t >= CLIMB_S and math.hypot(fvn, fve) > 0.4:
            want = math.atan2(fve, fvn)  # direction of the PLANNED motion, so the nose does not wobble with the error
            dt = 0.0 if self.last_t is None else max(t - self.last_t, 0.0)
            diff = (want - self.yaw + math.pi) % (2 * math.pi) - math.pi
            step = max(-YAW_RATE_MAX * dt, min(YAW_RATE_MAX * dt, diff))
            self.yaw += step
        self.last_t = t
        return vn, ve, tz, self.yaw


def quat_wxyz_to_yaw(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
