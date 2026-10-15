"""Webots controller of the simulator gate (copied into controllers/gate_cam/gate_cam.py by webots_gate.py).

GATE_MODE=physics  camera off: step for GATE_MIN_WALL_S seconds of wall time, count steps.
GATE_MODE=camera   camera on: step, read a picture at every step, save three pictures, count pictures.
Results go to $GATE_OUT (a json file). The motors turn at a constant speed that makes the Mavic 2 Pro lift off a little
(mavic2pro.c: k_vertical_thrust = 68.5); there is no flight controller, the drone may tilt and drift.
"""

import json
import os
import time
import traceback

from controller import Robot

mode = os.environ.get("GATE_MODE", "physics")
out_path = os.environ["GATE_OUT"]
pic_dir = os.environ.get("GATE_PIC_DIR", ".")
min_wall = float(os.environ.get("GATE_MIN_WALL_S", "12"))
res = {"mode": mode, "traceback": None}

try:
    robot = Robot()
    dt = int(robot.getBasicTimeStep())
    res["basic_time_step_ms"] = dt
    for name, sign in (("front left propeller", 1), ("front right propeller", -1), ("rear left propeller", -1), ("rear right propeller", 1)):
        m = robot.getDevice(name)
        m.setPosition(float("inf"))
        m.setVelocity(sign * 68.5)
    camera = None
    if mode == "camera":
        camera = robot.getDevice("camera")
        camera.enable(dt)
        res["camera_size"] = [camera.getWidth(), camera.getHeight()]
        import numpy as np
        from PIL import Image
    # warm up: the first steps include loading
    for _ in range(20):
        if robot.step(dt) == -1:
            raise RuntimeError("simulation ended during warm-up")
    sim0, wall0 = robot.getTime(), time.perf_counter()
    steps, pics, saved, stds = 0, 0, 0, []
    pic_times = [0.5, 4.0, 8.0]  # wall seconds after the start of the loop at which a picture is kept
    while time.perf_counter() - wall0 < min_wall:
        if robot.step(dt) == -1:
            res["ended_early"] = True
            break
        steps += 1
        if camera is not None:
            raw = camera.getImage()  # BGRA bytes
            arr = np.frombuffer(raw, np.uint8).reshape(camera.getHeight(), camera.getWidth(), 4)[:, :, [2, 1, 0]]
            pics += 1
            if saved < len(pic_times) and time.perf_counter() - wall0 >= pic_times[saved]:
                s = float(arr.astype(np.float64).std())
                stds.append(s)
                saved += 1
                Image.fromarray(np.ascontiguousarray(arr)).save(os.path.join(pic_dir, f"gate_cam_{saved}.png"))
    wall = time.perf_counter() - wall0
    sim = robot.getTime() - sim0
    res.update(steps=steps, wall_s=wall, sim_s=sim, steps_per_s=steps / wall, real_time_factor=sim / wall,
               pictures=pics, pictures_per_s=(pics / wall if camera is not None else None), saved=saved, stds=stds,
               final_z=None)
except Exception:
    res["traceback"] = traceback.format_exc()
finally:
    with open(out_path, "w") as f:
        json.dump(res, f, indent=2)
