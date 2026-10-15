"""Simulator gate, simulator 3: PX4-Autopilot SITL with Gazebo Sim (Harmonic), model x500_mono_cam, HEADLESS=1.

The workflow builds PX4 and starts the simulation (make px4_sitl gz_x500_mono_cam_<world>) under Xvfb with Mesa
software rendering, then calls this script. The script talks to Gazebo with its python transport bindings:
  * finds the world, the model and the camera topic with `gz topic -l`,
  * measures the physics iterations per wall second (topic /world/<w>/stats) with no camera subscriber,
  * subscribes to the camera, counts pictures per wall second, keeps three pictures,
  * does both again after raising the real-time-factor limit (the shipped worlds are capped at real time = 250 steps/s),
  * writes results/px4_gazebo/gate.json.
Gazebo cameras draw only while somebody subscribes to them, so "camera off" = no subscriber (this is checked by the
numbers, not assumed: the picture count while unsubscribed is also recorded).
"""

import json
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

MEASURE_S = 10.0


def sh(cmd: str, timeout=30) -> str:
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return p.stdout + p.stderr
    except subprocess.TimeoutExpired:
        return ""


def wait_for(pattern: str, timeout_s: float):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        out = sh("gz topic -l")
        m = re.search(pattern, out, re.M)
        if m:
            return m, out
        time.sleep(2)
    return None, sh("gz topic -l")


def main() -> int:
    out = common.results_dir("px4_gazebo")
    g = common.new_gate("px4_gazebo")
    notes = []
    try:
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.transport13 import Node
        import numpy as np
    except Exception as e:
        g["log_excerpt"] = [f"python bindings of gz-transport/gz-msgs not importable: {e}"]
        g["notes"] = "Gazebo python bindings missing, nothing measured."
        common.write_gate(g)
        return 1

    m, topics = wait_for(r"^/world/([^/]+)/clock$", 600)
    (out / "gz_topics.txt").write_text(topics)
    if not m:
        g["headless_ok"] = False
        g["log_excerpt"] = common.excerpt(out / "sim.log")
        g["notes"] = "No Gazebo world clock topic appeared within 600 s: Gazebo did not start (see sim.log)."
        common.write_gate(g)
        return 1
    world = m.group(1)
    g["world_used"] = f"PX4 gz world '{world}', model x500_mono_cam"
    g["headless_ok"] = True
    mm = re.search(r"^/world/%s/model/([^/]*mono_cam[^/]*)/" % re.escape(world), topics, re.M)
    model = mm.group(1) if mm else None
    cam_m, topics = wait_for(r"^(/world/%s/model/[^/]+/link/[^/]+/sensor/[^/]+/image)$" % re.escape(world), 120)
    (out / "gz_topics.txt").write_text(topics)
    if not cam_m:
        g["camera_ok"] = False
        g["notes"] = "No camera image topic exists (the x500_mono_cam model did not spawn or its camera sensor did not load)."
        g["log_excerpt"] = common.excerpt(out / "sim.log")
        common.write_gate(g)
        return 1
    cam_topic = cam_m.group(1)
    notes.append(f"camera topic {cam_topic}; model {model}")

    node = Node()
    lock = threading.Lock()
    state = {"iters": None, "sim_ns": None, "stats_n": 0, "frames": 0, "last_img": None, "img_size": None, "pix": None}

    def on_stats(msg):
        with lock:
            state["iters"] = msg.iterations
            state["sim_ns"] = msg.sim_time.sec * 10**9 + msg.sim_time.nsec
            state["stats_n"] += 1

    def on_image(msg):
        arr = np.frombuffer(msg.data, np.uint8)
        with lock:
            state["frames"] += 1
            state["pix"] = int(msg.pixel_format_type)
            state["img_size"] = [int(msg.width), int(msg.height)]
            if arr.size == msg.width * msg.height * 3:
                state["last_img"] = arr.reshape(msg.height, msg.width, 3).copy()

    stats_topic = f"/world/{world}/stats"
    if not node.subscribe(WorldStatistics, stats_topic, on_stats):
        notes.append(f"could not subscribe to {stats_topic}")
    time.sleep(3)

    def measure(label, with_camera):
        with lock:
            i0, s0, f0 = state["iters"], state["sim_ns"], state["frames"]
        t0 = time.perf_counter()
        time.sleep(MEASURE_S)
        wall = time.perf_counter() - t0
        with lock:
            i1, s1, f1 = state["iters"], state["sim_ns"], state["frames"]
        if None in (i0, i1):
            return {"label": label, "error": "no stats messages"}
        return {"label": label, "wall_s": wall, "iterations": i1 - i0, "steps_per_s": (i1 - i0) / wall,
                "real_time_factor": (s1 - s0) / 1e9 / wall, "frames": f1 - f0, "frames_per_s": (f1 - f0) / wall}

    def set_rtf(x):
        r = sh(f"gz service -s /world/{world}/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean "
               f"--timeout 3000 --req 'real_time_factor: {x}'")
        return r.strip()

    def lift_and_grab(z, yaw_deg=0.0):
        """Put the model at height z (it then falls freely), wait a little, return the newest picture."""
        if model:
            import math
            q = Pose()
            q.name = model
            q.position.x, q.position.y, q.position.z = 0.0, 0.0, z
            h = math.radians(yaw_deg) / 2
            q.orientation.w, q.orientation.z = math.cos(h), math.sin(h)
            node.request(f"/world/{world}/set_pose", q, Pose, Boolean, 2000)
        time.sleep(0.4)
        with lock:
            return None if state["last_img"] is None else state["last_img"].copy()

    results = {}
    results["shipped_camera_off"] = measure("shipped RTF limit, camera not subscribed", False)
    if not node.subscribe(Image, cam_topic, on_image):
        notes.append(f"could not subscribe to {cam_topic}")
    results["shipped_camera_on"] = measure("shipped RTF limit, camera subscribed", True)
    pics = []
    for z, yaw in ((12.0, 0.0), (6.0, 120.0), (20.0, 240.0)):
        img = lift_and_grab(z, yaw)
        if img is not None:
            pics.append(img)
    for i, img in enumerate(pics, 1):
        common.save_png(img, out / f"cam_{i}.png")
    stds = [common.image_stats(p)["std"] for p in pics]
    g["camera_ok"] = bool([s for s in stds if s > common.PASS_MIN_STD])
    g["camera_size"] = state["img_size"]
    notes.append(f"picture pixel_format_type {state['pix']} (3 = RGB_INT8), stds {[round(s, 1) for s in stds]}")

    # now without the real-time cap
    notes.append("set_physics real_time_factor 100 -> " + set_rtf(100))
    time.sleep(3)
    results["unlimited_camera_on"] = measure("RTF limit raised to 100, camera subscribed", True)
    node.unsubscribe(cam_topic)
    time.sleep(2)
    results["unlimited_camera_off"] = measure("RTF limit raised to 100, camera not subscribed", False)
    notes.append("set_physics real_time_factor 1 -> " + set_rtf(1))
    (out / "measurements.json").write_text(json.dumps(results, indent=2))

    s_off, s_off_u = results["shipped_camera_off"], results["unlimited_camera_off"]
    best_off = max([s_off, s_off_u], key=lambda r: r.get("steps_per_s", 0))
    cam_u, cam_s = results["unlimited_camera_on"], results["shipped_camera_on"]
    best_cam = max([cam_u, cam_s], key=lambda r: r.get("frames_per_s", 0))
    g["physics_steps_per_s"] = best_off.get("steps_per_s")
    g["real_time_factor"] = best_off.get("real_time_factor")
    g["camera_frames_per_s"] = best_cam.get("frames_per_s")
    g["notes"] = ("physics_steps_per_s = Gazebo iterations per wall second (max step 0.004 s), best of the shipped cap (RTF 1 -> at most 250/s) and the cap raised "
                  "to 100; camera_frames_per_s = camera-topic messages per wall second, best of the same two; real_time_factor = simulated / wall seconds "
                  "in the same run as physics_steps_per_s. All four measurements: " + json.dumps(results) + ". " + " | ".join(notes))
    if not g["camera_ok"]:
        g["log_excerpt"] = common.excerpt(out / "sim.log")
    common.write_gate(g)
    print(json.dumps(g, indent=2))
    return 0 if g["camera_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
