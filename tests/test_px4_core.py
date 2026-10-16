"""Tests of px4_gazebo/px4_core.py. Run: python tests/test_px4_core.py (exit code 1 on a failure; last line "N of N passed")."""

import math
import sys
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "px4_gazebo"))
import px4_core as pc  # noqa: E402

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


def sample(step=0.02):
    taus = np.arange(0.0, pc.T_ROUTE + pc.HOLD_END_S, step)
    return taus, np.array([pc.ref_pos_vel(t)[0] for t in taus]), np.array([pc.ref_pos_vel(t)[1] for t in taus])


def ang_between(yaws, vel):
    heading = np.arctan2(vel[:, 1], vel[:, 0])
    d = (heading - yaws + math.pi) % (2 * math.pi) - math.pi
    return np.degrees(np.abs(d))


@check("the reference path passes the gates in the order 1, 2, 3, 4, 2 (0 based 0, 1, 2, 3, 1)")
def _():
    taus, pos, _ = sample()
    c = pc.PassageCounter()
    for t, p in zip(taus, pos):
        c.update(t, p)
    assert [g for _, _, g, _, _ in c.log] == [0, 1, 2, 3, 1], c.log
    assert c.count == 5


@check("every passage leaves room for the drone (edge distance above radius + 0.15 m) and no frame bar is closer than radius + 0.1 m")
def _():
    taus, pos, _ = sample()
    c = pc.PassageCounter()
    for t, p in zip(taus, pos):
        c.update(t, p)
    for _, _, g, ly, lz in c.log:
        edge = pc.OPENING / 2 - max(abs(ly), abs(lz))
        assert edge > pc.DRONE_RADIUS + 0.15, (g, ly, lz, edge)
    d = min(pc.distance_to_frame(p)[0] for p in pos)
    assert d > pc.DRONE_RADIUS + 0.1, d


@check("the same path 3 m higher passes no opening (control: the counter can fail)")
def _():
    c = pc.PassageCounter()
    taus, pos, _ = sample()
    for t, p in zip(taus, pos):
        c.update(t, p + np.array([0.0, 0.0, 3.0]))
    assert c.count == 0, c.count


@check("speed and acceleration of the reference are gentle")
def _():
    taus, pos, vel = sample()
    sp = np.linalg.norm(vel, axis=1)
    acc = np.linalg.norm(np.diff(vel, axis=0), axis=1) / 0.02
    assert sp.max() < 3.0, sp.max()
    assert acc.max() < 3.0, acc.max()
    assert pos[:, 2].min() > 0.9 and pos[:, 2].max() < 8.0


@check("yaw plan: starts east, rate limited, nose within 30 deg of the velocity while the speed is above 0.8 m/s")
def _():
    taus, yaws = pc.yaw_plan()
    assert abs(yaws[0]) < 1e-12
    assert np.max(np.abs(np.diff(yaws))) <= pc.YAW_RATE_MAX * pc.PLAN_DT + 1e-9
    vel = np.array([pc.ref_pos_vel(t)[1][:2] for t in taus])
    sp = np.linalg.norm(vel, axis=1)
    ang = ang_between(yaws, vel)
    assert np.nanmax(ang[sp > 0.8]) < 30.0, np.nanmax(ang[sp > 0.8])
    assert np.nanmax(ang_between(yaws + 1.0, vel)[sp > 0.8]) > 30.0  # control: a nose turned by 1 rad fails the same rule


@check("NED yaw conversion: east = 90, north = 0, west = -90")
def _():
    assert pc.ned_yaw_deg(0.0) == 90.0
    assert abs(pc.ned_yaw_deg(math.pi / 2)) < 1e-9
    assert abs(pc.ned_yaw_deg(math.pi) + 90.0) < 1e-9


@check("gate SDF is valid XML with 5 visuals and 5 collisions, pose at the right height")
def _():
    for i in range(4):
        root = ET.fromstring(pc.gate_sdf(i, 0.2))
        assert len(root.findall(".//visual")) == 5 and len(root.findall(".//collision")) == 5
        pose = [float(v) for v in root.find("model/pose").text.split()]
        assert abs(pose[2] - (pc.GATES[i][2] + 0.2)) < 1e-9 and abs(pose[5] - pc.GATES[i][3]) < 1e-9


@check("opening centre is half an opening from the bars; a point on a bar is at distance 0")
def _():
    c, R = pc.gate_frame(0)
    assert abs(pc.distance_to_frame(c)[0] - pc.OPENING / 2) < 1e-9
    on_bar = c + R @ np.array([0.0, 0.0, pc.OPENING / 2 + 0.1])
    assert pc.distance_to_frame(on_bar)[0] == 0.0


if __name__ == "__main__":
    for n, ok, tb in RESULTS:
        print(("PASS " if ok else "FAIL ") + n)
        if not ok:
            print(tb)
    k = sum(1 for _, ok, _ in RESULTS if ok)
    print(f"{k} of {len(RESULTS)} passed")
    sys.exit(0 if k == len(RESULTS) else 1)
