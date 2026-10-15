"""Chase camera: a second robot that carries a camera and moves itself (it is a supervisor) to a place behind and above the
drone, looking at it. The direction "behind" is the horizontal direction of flight, smoothed (the same idea as the following
camera of run A). Saves raw 640 x 480 RGB pictures, one every 40 ms, into BASE_LAP_OUT/chase.rgb.

Environment: BASE_LAP_OUT = folder for the results.
"""

import math
import os
import traceback
from pathlib import Path

import numpy as np

from controller import Supervisor

OUT = Path(os.environ.get("BASE_LAP_OUT", "."))
PICTURE_EVERY = 10  # steps
BEHIND_M = 0.75
ABOVE_M = 0.30
ALPHA = 0.01  # smoothing of the flight direction per step (about 0.4 s)
MIN_SPEED = 0.15


def look_at_rotation(eye, target):
    """Webots camera frame: looks along +x, up is +z. Returns an axis-angle list [ax, ay, az, angle]."""
    fwd = np.asarray(target, float) - np.asarray(eye, float)
    fwd /= np.linalg.norm(fwd)
    left = np.cross([0.0, 0.0, 1.0], fwd)
    left /= np.linalg.norm(left)
    up = np.cross(fwd, left)
    R = np.column_stack([fwd, left, up])
    angle = math.acos(float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
    if angle < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    if abs(angle - math.pi) < 1e-6:
        axis = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        return [*(axis / np.linalg.norm(axis)), angle]
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * math.sin(angle))
    return [*(axis / np.linalg.norm(axis)), angle]


def main():
    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    me = robot.getSelf()
    drone = robot.getFromDef("DRONE")
    f_tr, f_rot = me.getField("translation"), me.getField("rotation")
    cam = robot.getDevice("chase")
    cam.enable(PICTURE_EVERY * dt_ms)
    w, h = cam.getWidth(), cam.getHeight()
    out = open(OUT / "chase.rgb", "wb")
    direction = np.array([1.0, 0.0])
    previous = None
    n = 0
    try:
        while True:
            p = np.array(drone.getPosition())
            if previous is not None:
                move = (p[:2] - previous[:2]) / (dt_ms / 1000.0)
                if np.linalg.norm(move) >= MIN_SPEED:
                    d = (1 - ALPHA) * direction + ALPHA * move / np.linalg.norm(move)
                    if np.linalg.norm(d) > 1e-6:
                        direction = d / np.linalg.norm(d)
            elif n == 0:
                direction = np.array([1.0, 0.0])
            previous = p
            eye = np.array([p[0] - BEHIND_M * direction[0], p[1] - BEHIND_M * direction[1], max(p[2] + ABOVE_M, 0.25)])
            f_tr.setSFVec3f([float(x) for x in eye])
            f_rot.setSFRotation([float(x) for x in look_at_rotation(eye, p)])
            if robot.step(dt_ms) == -1:
                break
            n += 1
            if n % PICTURE_EVERY == 0:
                raw = cam.getImage()
                arr = np.frombuffer(raw, np.uint8).reshape(h, w, 4)[:, :, [2, 1, 0]]
                out.write(np.ascontiguousarray(arr).tobytes())
                out.flush()
    except Exception:  # noqa: BLE001
        (OUT / "chase_error.txt").write_text(traceback.format_exc())
    finally:
        out.close()


main()
