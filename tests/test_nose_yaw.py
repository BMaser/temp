"""Tests of controllers/nose_yaw.py. Run: python tests/test_nose_yaw.py (no pytest needed).

Exit code 1 on any failure. The last line says "N of N passed".
Every check can fail: where a check could pass by accident, a control line shows the faulty variant failing.
"""

import math
import sys
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "controllers"))
import nose_yaw as ny  # noqa: E402

DT = 0.02
RESULTS = []


def check(name):
    def wrap(fn):
        try:
            fn()
            RESULTS.append((name, True, ""))
        except Exception:
            RESULTS.append((name, False, traceback.format_exc(limit=3)))
        return fn
    return wrap


def velocity_from_heading(heading, speed=1.0):
    heading = np.asarray(heading, dtype=float)
    return np.stack([speed * np.cos(heading), speed * np.sin(heading)], axis=1)


def max_abs_diff(a):
    return float(np.max(np.abs(np.diff(a)))) if len(a) > 1 else 0.0


@check("straight path gives a constant yaw")
def _():
    v = np.tile([1.0, 0.5], (100, 1))
    heading = math.atan2(0.5, 1.0)
    path = ny.path_yaw(v, 0.15, heading)
    assert np.allclose(path, heading, atol=1e-12), path[:3]
    plan = ny.plan_yaw(v, DT, 2.0, 0.15, heading)
    assert np.allclose(plan, heading, atol=1e-12)
    # a different start yaw: the plan turns to the heading at the allowed rate, then stays constant
    plan2 = ny.plan_yaw(v, DT, 2.0, 0.15, 0.0)
    assert plan2[0] == 0.0
    assert abs(plan2[-1] - heading) < 1e-12 and np.all(plan2[30:] == plan2[-1])


@check("U-turn: continuous change of pi, no 2*pi jump (the turn crosses the +pi/-pi line)")
def _():
    heading = np.linspace(0.5 * math.pi, 1.5 * math.pi, 200)
    v = velocity_from_heading(heading)
    path = ny.path_yaw(v, 0.15, heading[0])
    assert max_abs_diff(path) < 0.05, max_abs_diff(path)
    assert abs((path[-1] - path[0]) - math.pi) < 1e-9, path[-1] - path[0]
    raw = np.arctan2(v[:, 1], v[:, 0])  # control: the raw angle of the same data does jump
    assert max_abs_diff(raw) > math.pi, max_abs_diff(raw)


@check("heading across the +pi/-pi line stays continuous; raw atan2 jumps by more than pi")
def _():
    heading = np.linspace(2.9, 3.9, 120)  # crosses pi at 3.14159
    v = velocity_from_heading(heading)
    raw = np.arctan2(v[:, 1], v[:, 0])
    assert max_abs_diff(raw) > math.pi
    path = ny.path_yaw(v, 0.15, 2.9)
    assert max_abs_diff(path) < 0.02
    assert np.allclose(path, heading, atol=1e-9)
    heading_back = heading[::-1]  # and the other way, from the -pi side to the +pi side
    path_back = ny.path_yaw(velocity_from_heading(heading_back), 0.15, 3.9)
    assert np.allclose(path_back, heading_back, atol=1e-9)
    # the start yaw decides the branch: start near -pi keeps the series near -pi
    start_neg = ny.path_yaw(velocity_from_heading(np.full(5, math.pi - 0.1)), 0.15, -math.pi + 0.05)
    assert np.all(np.abs(start_neg - (-math.pi - 0.1)) < 1e-9), start_neg


@check("standstill: leading slow samples take the start yaw, later slow samples hold the last value")
def _():
    v = np.array([[0, 0]] * 3 + [[1, 0]] * 3 + [[0.01, 0.01]] * 3 + [[0, 1.0]] * 3, dtype=float)
    path = ny.path_yaw(v, 0.15, 0.7)
    assert np.all(path[:3] == 0.7), path[:3]
    assert np.allclose(path[3:6], 0.0)
    assert np.allclose(path[6:9], 0.0), "slow samples must hold the previous heading, not turn to atan2(0.01, 0.01)"
    assert np.allclose(path[9:], math.pi / 2)
    # the speed limit is "at least": exactly v_min counts as moving
    exact = ny.path_yaw(np.array([[0.15, 0.0]]), 0.15, 1.0)
    assert exact[0] == 0.0
    # not-a-number velocity holds
    nan = ny.path_yaw(np.array([[1.0, 0.0], [np.nan, np.nan], [1.0, 0.0]]), 0.15, 0.0)
    assert np.allclose(nan, 0.0)


@check("rate limit holds on a random path for several rate_max; plan[0] == start_yaw")
def _():
    rng = np.random.default_rng(1)
    for trial in range(20):
        n = 400
        heading = np.cumsum(rng.normal(0.0, 0.8, n))  # large random turns
        speed = rng.uniform(0.0, 2.0, n)
        v = velocity_from_heading(heading, 1.0) * speed[:, None]
        start = float(rng.uniform(-math.pi, math.pi))
        for rate in (0.5, 1.0, 2.0, 4.0, 8.0):
            plan = ny.plan_yaw(v, DT, rate, 0.15, start)
            assert plan[0] == start
            assert max_abs_diff(plan) <= rate * DT + 1e-12, (trial, rate, max_abs_diff(plan))
            assert np.all(np.isfinite(plan))
    # control: the unlimited path really does turn faster than the smallest rate, so the check above can fail
    path = ny.path_yaw(v, 0.15, start)
    assert max_abs_diff(path) > 0.5 * DT * 10


@check("sudden change of the path heading: the plan has reached the new value at or before the change")
def _():
    n, k0, target = 300, 100, 1.2
    heading = np.where(np.arange(n) < k0, 0.0, target)
    v = velocity_from_heading(heading)
    rate = 2.0  # 0.04 rad per sample, so the turn of 1.2 rad needs 30 samples
    plan = ny.plan_yaw(v, DT, rate, 0.15, 0.0)
    assert abs(plan[k0] - target) < 1e-12, plan[k0]
    assert plan[k0 - 1] < target - 1e-3 and plan[k0 - 29] > 0.0, "the turn is a ramp that ends at the change"
    assert plan[k0 - 31] == 0.0, "no turn earlier than needed"
    assert np.all(plan[k0:] == target)
    # control: a forward-only limit (no early start) is late, so this check can fail
    late = ny.limit_forward(ny.path_yaw(v, 0.15, 0.0), rate, DT, 0.0)
    assert late[k0] < target - 0.5, late[k0]
    # when there is too little time before the change, the plan cannot reach it: it is still turning
    plan_short = ny.plan_yaw(velocity_from_heading(np.where(np.arange(n) < 10, 0.0, target)), DT, rate, 0.15, 0.0)
    assert plan_short[10] < target - 0.5


@check("plan[0] == start_yaw exactly even when the path heading is far from it")
def _():
    v = np.tile([0.0, 1.0], (50, 1))
    for start in (0.0, 2.0, -3.0, 7.5):
        plan = ny.plan_yaw(v, DT, 1.0, 0.15, start)
        assert plan[0] == start
        assert abs(plan[1] - plan[0]) <= DT + 1e-12


@check("limit_backward and limit_forward on a hand-made series")
def _():
    y = np.array([0.0, 0.0, 0.0, 1.0, 1.0])
    back = ny.limit_backward(y, 10.0, 0.1)  # step 1.0 per sample
    assert np.allclose(back, [0.0, 0.0, 0.0, 1.0, 1.0])  # a jump of 1.0 is allowed within one step
    back = ny.limit_backward(y, 5.0, 0.1)  # step 0.5: starts one sample earlier
    assert np.allclose(back, [0.0, 0.0, 0.5, 1.0, 1.0]), back
    fwd = ny.limit_forward(y, 5.0, 0.1, 0.0)
    assert np.allclose(fwd, [0.0, 0.0, 0.0, 0.5, 1.0]), fwd
    assert ny.limit_forward(np.array([]), 1.0, 0.1, 0.0).size == 0
    for bad in ((-1.0, 0.1), (1.0, 0.0)):
        try:
            ny.limit_backward(y, *bad)
        except ValueError:
            continue
        raise AssertionError(f"no ValueError for {bad}")


@check("nose_vs_velocity_deg gives 0, 90 and 180 on hand-made cases and handles the +pi/-pi line")
def _():
    a = ny.nose_vs_velocity_deg([0.0, 0.0, 0.0], [[1, 0], [0, 1], [-1, 0]])
    assert np.allclose(a, [0.0, 90.0, 180.0]), a
    b = ny.nose_vs_velocity_deg([math.pi - 0.1], [[math.cos(-math.pi + 0.1), math.sin(-math.pi + 0.1)]])
    assert abs(b[0] - math.degrees(0.2)) < 1e-9, b  # 0.2 rad apart, not 2*pi - 0.2
    c = ny.nose_vs_velocity_deg([0.0, 0.0], [[0.0, 0.0], [1.0, 0.0]])
    assert np.isnan(c[0]) and c[1] == 0.0
    d = ny.nose_vs_velocity_deg([10.0 * math.pi], [[1.0, 0.0]])  # a multiple of 2*pi gives the same nose
    assert abs(d[0]) < 1e-9


@check("yaw_from_quat_xyzw: pure yaw, and a tilted drone")
def _():
    for yaw in (0.0, 0.5, -2.0, 3.0):
        q = [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]
        assert abs(ny.wrap_pi(ny.yaw_from_quat_xyzw(q) - yaw)) < 1e-12
    # roll 0.3 then yaw 1.0 (z-y-x order): the nose heading is still 1.0
    roll, yaw = 0.3, 1.0
    qz = np.array([0, 0, math.sin(yaw / 2), math.cos(yaw / 2)])
    qx = np.array([math.sin(roll / 2), 0, 0, math.cos(roll / 2)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    q = [w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2]
    assert abs(ny.yaw_from_quat_xyzw(q) - yaw) < 1e-12
    many = ny.yaw_from_quat_xyzw(np.array([[0, 0, 0, 1.0], [0, 0, 1.0, 0]]))
    assert np.allclose(many, [0.0, math.pi]) or np.allclose(many, [0.0, -math.pi])


def main():
    failed = [r for r in RESULTS if not r[1]]
    for name, ok, text in RESULTS:
        print(("PASS  " if ok else "FAIL  ") + name)
        if not ok:
            print(text)
    print(f"{len(RESULTS) - len(failed)} of {len(RESULTS)} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
