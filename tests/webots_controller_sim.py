"""Offline check of the Webots lap controller against a simple rigid-body model of the Webots Crazyflie (no Webots needed).

Run: python tests/webots_controller_sim.py   Exit code 1 if the lap is not flown. The model approximates Webots' ODE
(same mass, inertia, thrust and torque constants; commands held for one 4 ms step; 4 sub-steps)."""
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "webots" / "controllers" / "base_lap"))
sys.path.insert(0, str(HERE.parent / "controllers"))
import lap_core as lc  # noqa: E402
import nose_yaw as ny  # noqa: E402

DT = 0.004


def hat(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def _expm(W):
    th = np.sqrt(0.5 * np.sum(W * W))
    if th < 1e-12:
        return np.eye(3) + W
    return np.eye(3) + math.sin(th) / th * W + (1 - math.cos(th)) / th ** 2 * (W @ W)


def run(rate=4.0, verbose=True, **gains):
    start = np.array([*lc.START_XY, lc.START_Z])
    sp = lc.build_spline(start)
    times = np.arange(0, lc.T_TOTAL + 5, DT)
    vel = sp(np.minimum(times, lc.T_TOTAL), 1)[:, :2]
    plan = ny.plan_yaw(vel, DT, rate, 0.15, 0.0)
    yaw_fn = lambda t: float(plan[min(int(round(t / DT)), len(plan) - 1)])
    ctl = lc.LapController(DT, yaw_fn, sp, **gains)
    p, v, R, w = start.copy(), np.zeros(3), np.eye(3), np.zeros(3)
    pc = lc.PassageCounter()
    tilt, dmin = 0.0, 9.0
    t = 0.0
    log = []
    while t < lc.T_TOTAL + 3 and pc.count < 5:
        wm = ctl.step(t, p, v, R, w)
        T, tau = lc.forces_from_speeds(wm)
        for _ in range(4):
            h = DT / 4
            acc = R[:, 2] * T / lc.MASS - np.array([0, 0, lc.G])
            if p[2] <= lc.START_Z and acc[2] < 0:
                acc[2] = 0.0
                v[2] = max(v[2], 0.0)
            p = p + v * h + 0.5 * acc * h * h
            v = v + acc * h
            wd = np.linalg.solve(lc.INERTIA, tau - np.cross(w, lc.INERTIA @ w))
            w = w + wd * h
            R = R @ _expm(hat(w * h))
        t += DT
        pc.update(t, p)
        tilt = max(tilt, math.degrees(math.acos(np.clip(R[2, 2], -1, 1))))
        dmin = min(dmin, lc.distance_to_frame(p)[0])
        log.append((t, *p))
    log = np.array(log)
    err = np.linalg.norm(log[:, 1:] - sp(np.minimum(log[:, 0], lc.T_TOTAL)), axis=1)
    out = {"passages": pc.count, "t_end": round(t, 2), "max_tilt": round(tilt, 1), "min_frame_dist": round(dmin, 3),
           "max_track_err": round(float(err.max()), 3), "passage_log": [(round(a, 2), b, c, round(d, 3), round(e, 3)) for a, b, c, d, e in pc.log]}
    if verbose:
        print(out)
    return out


if __name__ == "__main__":
    r = run()
    sys.exit(0 if r["passages"] == 5 and r["min_frame_dist"] > 0.06 else 1)
