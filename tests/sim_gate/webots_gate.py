"""Simulator gate, simulator 2: Webots R2025a with the DJI Mavic 2 Pro in the rural demo world, under Xvfb + Mesa software
rendering (LIBGL_ALWAYS_SOFTWARE=1, llvmpipe).

Webots must already be installed (the .deb, see the workflow). This script
  * builds a small project (world copied from the Webots installation, the controller replaced by gate_cam, the camera
    set to 640 x 480, the drone put at 5 m height),
  * run 1 (physics): xvfb-run webots --mode=fast --no-rendering --batch, camera off,
  * run 2 (camera):  xvfb-run webots --mode=fast --batch, camera on, three pictures,
  * writes results/webots/gate.json.
"""

import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

WEBOTS_HOME = Path(os.environ.get("WEBOTS_HOME", "/usr/local/webots"))
WORLD_SRC = WEBOTS_HOME / "projects/robots/dji/mavic/worlds/mavic_2_pro.wbt"
RUN_TIMEOUT_S = 600


def build_project(proj: Path) -> Path:
    (proj / "worlds").mkdir(parents=True, exist_ok=True)
    ctl = proj / "controllers" / "gate_cam"
    ctl.mkdir(parents=True, exist_ok=True)
    shutil.copy(Path(__file__).resolve().parent / "webots_controller_gate_cam.py", ctl / "gate_cam.py")
    text = WORLD_SRC.read_text()
    new, n = re.subn(r"Mavic2Pro \{\s*translation 0 0 0\.1(.*?)cameraSlot \[\s*Camera \{\s*width 400\s*height 240",
                     lambda m: "Mavic2Pro {\n  translation 0 0 5" + m.group(1) + "controller \"gate_cam\"\n  cameraSlot [\n    Camera {\n      width 640\n      height 480",
                     text, flags=re.S)
    if n != 1:
        raise RuntimeError(f"could not edit the Mavic2Pro node in {WORLD_SRC} (matches: {n})")
    world = proj / "worlds" / "gate.wbt"
    world.write_text(new)
    return world


def run_webots(world: Path, mode: str, out: Path, log: Path, rendering: bool) -> dict:
    result_json = out / f"controller_{mode}.json"
    result_json.unlink(missing_ok=True)
    env = {**os.environ, "GATE_MODE": mode, "GATE_OUT": str(result_json), "GATE_PIC_DIR": str(out),
           "GATE_MIN_WALL_S": "12" if mode == "physics" else "14", "LIBGL_ALWAYS_SOFTWARE": "1",
           "WEBOTS_HOME": str(WEBOTS_HOME)}
    cmd = ["xvfb-run", "-a", "-s", "-screen 0 1280x1024x24 +extension GLX +render -noreset", "webots", "--mode=fast", "--batch",
           "--stdout", "--stderr", "--minimize"] + (["--no-rendering"] if not rendering else []) + [str(world)]
    print("RUN:", " ".join(cmd), flush=True)
    t0 = time.perf_counter()
    with open(log, "w") as lf:
        p = subprocess.Popen(cmd, env=env, stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)
        status = "timeout"
        while time.perf_counter() - t0 < RUN_TIMEOUT_S:
            if result_json.is_file():
                status = "result"
                break
            if p.poll() is not None:
                status = f"webots exited with code {p.returncode}"
                break
            time.sleep(1.0)
        time.sleep(1.0)
        try:
            os.killpg(p.pid, signal.SIGTERM)  # our own process group (started with start_new_session)
        except ProcessLookupError:
            pass
        try:
            p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
    r = json.loads(result_json.read_text()) if result_json.is_file() else {}
    r["status"] = status
    r["startup_plus_run_wall_s"] = time.perf_counter() - t0
    return r


def main() -> int:
    out = common.results_dir("webots")
    g = common.new_gate("webots")
    ver = subprocess.run(["webots", "--version"], capture_output=True, text=True, env={**os.environ, "LIBGL_ALWAYS_SOFTWARE": "1"})
    g["version_or_commit"] = (ver.stdout + ver.stderr).strip().replace("\n", "; ")[:200] or "unknown"
    g["world_used"] = "projects/robots/dji/mavic/worlds/mavic_2_pro.wbt (DJI Mavic 2 PRO over a rural area: windmills, pines, cherry-tree forest, road, small manor); camera 640x480; drone started at 5 m"
    try:
        world = build_project(out / "project")
    except Exception as e:
        g["log_excerpt"] = [str(e)]
        g["notes"] = "Could not build the test project."
        common.write_gate(g)
        return 1
    phys = run_webots(world, "physics", out, out / "webots_physics.log", rendering=False)
    cam = run_webots(world, "camera", out, out / "webots_camera.log", rendering=True)
    g["headless_ok"] = bool(phys.get("steps"))
    if phys.get("steps"):
        g["physics_steps_per_s"] = phys["steps_per_s"]
        g["real_time_factor"] = phys["real_time_factor"]
    stds = cam.get("stds", [])
    pics = [out / f"gate_cam_{i}.png" for i in range(1, len(stds) + 1)]
    ok_pics = [p for p, s in zip(pics, stds) if s > common.PASS_MIN_STD]
    g["camera_ok"] = bool(ok_pics)
    if cam.get("camera_size"):
        g["camera_size"] = cam["camera_size"]
    if cam.get("pictures_per_s"):
        g["camera_frames_per_s"] = cam["pictures_per_s"]
    for i, p in enumerate(ok_pics[:3], 1):
        shutil.copy(p, out / f"cam_{i}.png")
    g["notes"] = (
        f"Physics run (--mode=fast --no-rendering, camera off, basicTimeStep {phys.get('basic_time_step_ms')} ms): status {phys.get('status')}, "
        f"{phys.get('steps')} steps in {phys.get('wall_s', 0):.1f} s wall; real_time_factor = simulated seconds / wall seconds (computed from the controller clock, "
        f"Webots does not print one in batch mode). Camera run (rendering on, Mavic camera 640x480 read at every step): status {cam.get('status')}, "
        f"{cam.get('pictures')} pictures in {cam.get('wall_s', 0):.1f} s wall, saved picture stds {[round(s, 1) for s in stds]}. "
        f"The drone has no flight controller (constant motor speed 68.5 rad/s), so the three pictures are from wherever it drifted. "
        f"Tracebacks: physics={phys.get('traceback')!r} camera={cam.get('traceback')!r}")
    if not g["camera_ok"] or not g["headless_ok"]:
        g["log_excerpt"] = common.excerpt(out / "webots_camera.log") + common.excerpt(out / "webots_physics.log")
    common.write_gate(g)
    print(json.dumps(g, indent=2))
    return 0 if g["camera_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
