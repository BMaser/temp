"""Simulator gate, simulator 1: Crazyflow (LSY, JAX + MuJoCo) through the LSY Drone Racing environment.

Run with the python of the pixi environment of lsy_drone_racing (see the workflow). The parent mode (default) starts
one child per MuJoCo drawing method (egl, osmesa), because MUJOCO_GL has to be set before MuJoCo is imported. The
first method that gives a non-blank picture is the result.

The child builds the standard level-0 race environment (as tests/crazyflow_camera.py), then
  1. physics: steps with the camera off for at least MIN_WALL_S seconds of wall time, hovering above the start;
  2. camera: steps and draws the picture of the drone camera fpv_cam:0 (640 x 480) in a loop for the same time;
  3. pictures: fpv after reset, fpv after 40 steps, outside view.
"""

import argparse
import json
import os
import subprocess
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

METHODS = ("egl", "osmesa")
WIDTH, HEIGHT = 640, 480
MIN_WALL_S = 12.0
WARMUP_STEPS = 30
CHILD_TIMEOUT_S = 1500


def run_child(method: str, repo: Path, out: Path, commit: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    res = {"method": method, "MUJOCO_GL": os.environ.get("MUJOCO_GL"), "versions": {}, "traceback": None}
    env = None
    try:
        os.environ["SCIPY_ARRAY_API"] = "1"
        import importlib.metadata as md

        import gymnasium
        import jax
        import mujoco
        import numpy as np
        from lsy_drone_racing.utils import load_config

        res["versions"] = {"mujoco": mujoco.__version__, "jax": jax.__version__, "crazyflow": md.version("crazyflow"),
                           "lsy_drone_racing": md.version("lsy_drone_racing"), "lsy_drone_racing_commit": commit}
        cfg = load_config(repo / "config" / "level0.toml")
        cfg.sim.render = False
        env = gymnasium.make(
            cfg.env.id, freq=cfg.env.freq, sim_config=cfg.sim, sensor_range=cfg.env.sensor_range,
            control_mode=cfg.env.control_mode, track=cfg.env.track, disturbances=cfg.env.get("disturbances"),
            randomizations=cfg.env.get("randomizations"), seed=0)
        core = env.unwrapped
        obs, _ = env.reset()
        env_hz, phys_hz = float(cfg.env.freq), float(cfg.sim.freq)
        res["env_freq_hz"], res["physics_freq_hz"] = env_hz, phys_hz
        res["jax_devices"] = [str(d) for d in jax.devices()]

        # State command [x y z, v(3), a(3), quat xyzw, w(3)]: hover 0.5 m above the start position.
        pos0 = np.asarray(obs["pos"], dtype=float)
        action = np.zeros(16, dtype=np.float32)
        action[:3] = pos0 + np.array([0.0, 0.0, 0.5])
        action[12] = 1.0

        def step():
            nonlocal obs
            obs, _, term, trunc, _ = env.step(action)
            np.asarray(obs["pos"])  # JAX is asynchronous: reading the result makes the step really finish
            return bool(term or trunc)

        def render(**kw):
            if not core.data.sim_data.core.mjx_synced:
                core.data, core.sim.mjx_data = core._render_sync(core.data, core.sim.mjx_data)
            return np.ascontiguousarray(core.sim.render(mode="rgb_array", width=WIDTH, height=HEIGHT, **kw))

        # 1. physics, camera off
        for _ in range(WARMUP_STEPS):  # the first steps compile the JAX functions
            step()
        steps, step_time, wall0 = 0, 0.0, time.perf_counter()
        while time.perf_counter() - wall0 < MIN_WALL_S:
            t = time.perf_counter()
            ended = step()
            step_time += time.perf_counter() - t
            steps += 1
            if ended:
                obs, _ = env.reset()
        res["physics"] = {"env_steps": steps, "step_wall_s": step_time, "env_steps_per_s": steps / step_time,
                          "physics_steps_per_s": steps / step_time * phys_hz / env_hz,
                          "real_time_factor": steps / step_time / env_hz}

        # 2. pictures (fresh environment state)
        obs, _ = env.reset()
        pics = []

        def take(label, **kw):
            e = {"label": label}
            pics.append(e)
            try:
                t = time.perf_counter()
                img = render(**kw)
                e["seconds"] = time.perf_counter() - t
                e.update(common.image_stats(img))
                e["ok"] = common.picture_ok(img)
                e["array"] = img
            except Exception:
                e["traceback"] = traceback.format_exc()

        take("fpv_after_reset", camera="fpv_cam:0")
        for _ in range(40):
            step()
        take("fpv_after_steps", camera="fpv_cam:0")
        for _ in range(60):
            step()
        take("fpv_after_more_steps", camera="fpv_cam:0")

        # 3. camera on: step and draw in a loop
        frames, wall0 = 0, time.perf_counter()
        render_s = 0.0
        while time.perf_counter() - wall0 < MIN_WALL_S:
            if step():
                obs, _ = env.reset()
            t = time.perf_counter()
            img = render(camera="fpv_cam:0")
            render_s += time.perf_counter() - t
            frames += 1
        wall = time.perf_counter() - wall0
        res["camera"] = {"frames": frames, "wall_s": wall, "frames_per_s_with_step": frames / wall,
                         "render_only_frames_per_s": frames / render_s}

        # 4. outside view for reference (not counted for camera_ok)
        core.sim.close()
        cam_cfg = {k: np.asarray(v) if k == "lookat" else v for k, v in cfg.sim.cam_config[0].items()}
        outside = {"label": "outside_view"}
        try:
            outside["array"] = render(camera=-1, cam_config=cam_cfg)
            outside.update(common.image_stats(outside["array"]))
        except Exception:
            outside["traceback"] = traceback.format_exc()
        pics.append(outside)

        for i, e in enumerate(pics):
            arr = e.pop("array", None)
            if arr is not None:
                common.save_png(arr, out / f"{method}_{e['label']}.png")
                e["file"] = f"{method}_{e['label']}.png"
        res["pictures"] = pics
    except Exception:
        res["traceback"] = traceback.format_exc()
    finally:
        try:
            if env is not None:
                env.close()
        except Exception:
            res["close_traceback"] = traceback.format_exc()
        (out / f"child_{method}.json").write_text(json.dumps(res, indent=2, default=str))


def parent(repo: Path, out: Path, commit: str) -> int:
    out.mkdir(parents=True, exist_ok=True)
    g = common.new_gate("crazyflow")
    g["version_or_commit"] = f"lsy_drone_racing {commit}"
    g["world_used"] = "LSY Drone Racing level 0 track (indoor lab room, 4 square gates, 2 posts), drone cf21B_500"
    chosen = None
    tried = []
    for method in METHODS:
        env = {**os.environ, "MUJOCO_GL": method}
        t = time.perf_counter()
        try:
            proc = subprocess.run([sys.executable, str(Path(__file__).resolve()), "child", "--method", method, "--repo", str(repo),
                                   "--out", str(out), "--commit", commit], env=env, capture_output=True, text=True, timeout=CHILD_TIMEOUT_S)
            (out / f"child_{method}_stdout.txt").write_text(proc.stdout)
            (out / f"child_{method}_stderr.txt").write_text(proc.stderr)
            print(f"[{method}] returncode {proc.returncode} in {time.perf_counter() - t:.0f} s")
            print(proc.stderr[-3000:])
        except subprocess.TimeoutExpired:
            print(f"[{method}] timed out")
        f = out / f"child_{method}.json"
        r = json.loads(f.read_text()) if f.is_file() else {"method": method, "traceback": "child wrote no json"}
        tried.append(r)
        good = [p for p in r.get("pictures", []) if p.get("label", "").startswith("fpv") and p.get("ok")]
        if not r.get("traceback") and good:
            chosen = r
            break
    g["attempts"] = int(os.environ.get("GATE_ATTEMPT", "1") or 1)
    if chosen is None:
        g["headless_ok"] = any(r.get("physics") for r in tried)
        g["camera_ok"] = False
        g["log_excerpt"] = [(r.get("traceback") or "").strip().splitlines()[-1][:300] if r.get("traceback") else f"{r['method']}: no ok picture" for r in tried]
        r0 = next((r for r in tried if r.get("physics")), None)
        if r0:
            g["physics_steps_per_s"] = r0["physics"]["physics_steps_per_s"]
        g["notes"] = "No drawing method (egl, osmesa) gave a non-blank fpv picture."
    else:
        pics = chosen["pictures"]
        good = [p for p in pics if p.get("label", "").startswith("fpv") and p.get("ok")]
        g["headless_ok"] = True
        g["camera_ok"] = True
        g["camera_size"] = [WIDTH, HEIGHT]
        g["physics_steps_per_s"] = chosen["physics"]["physics_steps_per_s"]
        g["camera_frames_per_s"] = chosen["camera"]["frames_per_s_with_step"]
        g["real_time_factor"] = chosen["physics"]["real_time_factor"]
        v = chosen["versions"]
        g["version_or_commit"] = f"lsy_drone_racing {commit}; crazyflow {v.get('crazyflow')}; mujoco {v.get('mujoco')}; jax {v.get('jax')}"
        g["notes"] = (
            f"Drawing method {chosen['method']}. Hover flight above the start, camera off: {chosen['physics']['env_steps_per_s']:.1f} environment steps/s "
            f"(env {chosen['env_freq_hz']:g} Hz, each = {chosen['physics_freq_hz'] / chosen['env_freq_hz']:g} physics steps at {chosen['physics_freq_hz']:g} Hz), "
            f"so physics_steps_per_s = env steps/s x {chosen['physics_freq_hz'] / chosen['env_freq_hz']:g}. real_time_factor is COMPUTED "
            f"(env steps/s / env Hz); Crazyflow does not report one. camera_frames_per_s = fpv_cam 640x480 pictures per second in a loop of one env step + one picture "
            f"(picture alone: {chosen['camera']['render_only_frames_per_s']:.1f}/s). JAX devices: {chosen['jax_devices']}. "
            f"Other method results: {[(r['method'], bool(r.get('traceback'))) for r in tried]}.")
        for i, p in enumerate(good[:3], 1):
            (out / f"cam_{i}.png").write_bytes((out / p["file"]).read_bytes())
    (out / "tried.json").write_text(json.dumps(tried, indent=2, default=str))
    common.write_gate(g)
    print(json.dumps(g, indent=2))
    return 0 if g["camera_ok"] else 1


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("command", nargs="?", default="run", choices=["run", "child"])
    ap.add_argument("--repo", default="lsy_drone_racing")
    ap.add_argument("--out", default=str(common.results_dir("crazyflow")))
    ap.add_argument("--commit", default="unknown")
    ap.add_argument("--method")
    a = ap.parse_args()
    if a.command == "child":
        run_child(a.method, Path(a.repo).resolve(), Path(a.out).resolve(), a.commit)
    else:
        sys.exit(parent(Path(a.repo).resolve(), Path(a.out).resolve(), a.commit))


if __name__ == "__main__":
    main()
