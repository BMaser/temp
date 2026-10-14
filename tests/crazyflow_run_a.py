"""Run A: does the drone keep its nose (and so its camera) along the direction of flight? Measure it and film it.

The controller is controllers/nose_forward_controller.py: the example path of lsy_drone_racing (commit ac7377b) with the
yaw planned from the direction of flight (controllers/nose_yaw.py). The track is level 0 of lsy_drone_racing with the new
square gate model, the start pad under the start position, the end pad under the last point of the path, and the drone in
hi-vis orange (the same environment as tests/crazyflow_track_look.py, tools/track_env.py). The seed is 0.

Sub-commands, each writes into --out:
  sweep    6 flights without pictures: yaw 0.0 ("fixed", the baseline) and the nose-forward plan at 0.5, 1, 2, 4, 8 rad/s.
           Writes sweep/<name>.npz, sweep.json, chosen.json, yaw_sweep.png.
  films    the chosen setting, filmed: fpv.mp4 (camera on the drone), chase.mp4 (following camera), fpv_and_map.mp4
           (made by tools/sync_film.py), and a contact sheet <film>_contact.png for each. Writes films.json.
  scale    the chosen setting again with the drone drawn at 1.5 and 2.0 times the size, filmed with the following
           camera (chase_scale_1p5.mp4, chase_scale_2p0.mp4). Proof that the drawing scale does not change the flight.
           Writes scale.json.
  summary  summary.md and result.json from the files above. Exit code 1 if any check failed.
  check    applies the pass rule to result.json (standard library only), exit code 1 on failure.
Internal: child (one flight in its own process; the parent starts it).

Every flight starts from a fresh environment with seed 0 and exactly one env.reset() (every further reset would draw new
noise), in its own process, as in tests/crazyflow_flight_video.py, whose film flights were identical to the last bit.
"""

import argparse
import json
import math
import os
import platform
import subprocess
import sys
import tempfile
import time
import traceback
from pathlib import Path

try:
    import numpy as np
except ImportError:  # the check sub-command needs only the standard library
    np = None

HERE = Path(__file__).resolve().parent
REPO_DIR = HERE.parent
TOOLS_DIR = REPO_DIR / "tools"
CONTROLLERS_DIR = REPO_DIR / "controllers"
PADS_DIR = REPO_DIR / "assets" / "pads"
for _p in (HERE, TOOLS_DIR, CONTROLLERS_DIR):
    sys.path.insert(0, str(_p))

import crazyflow_flight_video as fv  # noqa: E402  (stdlib only at import: chase view, constants)
import crazyflow_track_look as tl  # noqa: E402  (stdlib only at import: model checks, orange, drawing scale)

METHODS = ("egl", "osmesa")
WIDTH, HEIGHT = 640, 480
FPS = 25  # level0.toml has env.freq = 50 Hz; a picture at every 2nd step gives 25 pictures a second
STEP_PER_PICTURE = 2
SEED = 0
CHILD_TIMEOUT_S = 1500
RATES = (0.5, 1.0, 2.0, 4.0, 8.0)  # rad/s
V_MIN_MOVING = 0.3  # m/s: steps with at least this horizontal speed count for the nose-versus-velocity angle
BACKWARD_DEG = 90.0
MIN_BACKWARD_SHARE_OF_FIXED = 0.10  # positive control: the baseline must fly backward in more than 10 % of the moving steps
TIE_DEG = 1.0  # within this many degrees of the best 95th percentile the lower rate wins
PADDING_ROW = (-1.0, -1.0, -1.0)  # LSY warps a disabled drone to this position (race_core.py line 851)
SCALES = (1.0, 1.5, 2.0)
START_TOL = tl.START_TOL  # 1e-4 m
SPLINE_CHECK_TIMES = 200
SPLINE_TOL = 1e-12


def rate_tag(rate: float) -> str:
    return f"{rate:g}".replace(".", "p")


def flight_list() -> list:
    """(name, nose_mode, yaw_rate_max); the rate of the baseline is not used."""
    return [("fixed", "fixed", 2.0)] + [(f"forward_{rate_tag(r)}", "forward", r) for r in RATES]


def scale_tag(s: float) -> str:
    return rate_tag(s) if s != int(s) else f"{int(s)}p0"


# ---------------------------------------------------------------------------------------------
# Measures (numpy only)
# ---------------------------------------------------------------------------------------------


def flight_metrics(a: dict, freq: float, fixed_pos=None) -> dict:
    """Numbers of one flight from the arrays saved per step (rows = states after 0, 1, 2 ... steps).

    nose angle: the unsigned angle between the nose (heading of the body x axis from the SIMULATED quaternion) and the
    horizontal velocity, over the steps with horizontal speed >= V_MIN_MOVING; 'backward' = more than 90 degrees.
    yaw error: actual yaw minus commanded yaw, wrapped to +-pi, absolute value.
    """
    import nose_yaw as ny

    pos, vel, quat, cmd = (np.asarray(a[k], dtype=float) for k in ("pos", "vel", "quat", "cmd_yaw"))
    yaw = ny.yaw_from_quat_xyzw(quat)
    speed = np.hypot(vel[:, 0], vel[:, 1])
    moving = speed >= V_MIN_MOVING
    angle = ny.nose_vs_velocity_deg(yaw, vel[:, :2])[moving]
    err = np.degrees(np.abs(ny.wrap_pi(yaw - cmd)))
    tilt = np.degrees(np.arccos(np.clip(1.0 - 2.0 * (quat[:, 0] ** 2 + quat[:, 1] ** 2), -1.0, 1.0)))
    m = {
        "rows": int(len(pos)),
        "steps": int(len(pos) - 1),
        "sim_seconds": float((len(pos) - 1) / freq),
        "passages": int(np.max(a["passed"])) if len(pos) else 0,
        "moving_steps": int(moving.sum()),
        "nose_angle_deg_max": float(angle.max()) if angle.size else None,
        "nose_angle_deg_p95": float(np.percentile(angle, 95)) if angle.size else None,
        "nose_angle_deg_mean": float(angle.mean()) if angle.size else None,
        "backward_share": float(np.mean(angle > BACKWARD_DEG)) if angle.size else None,
        "yaw_error_deg_max": float(err.max()) if err.size else None,
        "yaw_error_deg_p95": float(np.percentile(err, 95)) if err.size else None,
        "tilt_deg_max": float(tilt.max()) if tilt.size else None,
        "max_distance_to_fixed_m": None,
    }
    if fixed_pos is not None:
        fp = np.asarray(fixed_pos, dtype=float)
        n = min(len(fp), len(pos))
        m["max_distance_to_fixed_m"] = float(np.linalg.norm(pos[:n] - fp[:n], axis=1).max())
    return m


def choose(metrics: dict, modes: dict, rates: dict, total: int) -> dict:
    """The choice rule. Only nose-forward flights can be chosen.

    Qualifying = all `total` passages and a backward share of exactly 0. Among them the smallest 95th percentile angle;
    within TIE_DEG of it the lower rate wins. If none qualifies: the most passages, then the smallest 95th percentile
    angle (qualified = False).
    """
    forward = [n for n in metrics if modes.get(n) == "forward" and metrics[n] and metrics[n].get("nose_angle_deg_p95") is not None]
    if not forward:
        return {"name": None, "qualified": False, "reason": "no nose-forward flight produced numbers"}
    qualifying = [n for n in forward if metrics[n]["passages"] >= total and metrics[n]["backward_share"] == 0.0]
    if qualifying:
        best = min(metrics[n]["nose_angle_deg_p95"] for n in qualifying)
        near = [n for n in qualifying if metrics[n]["nose_angle_deg_p95"] <= best + TIE_DEG]
        name = min(near, key=lambda n: rates[n])
        return {"name": name, "qualified": True,
                "reason": f"{len(qualifying)} flight(s) with {total} of {total} passages and backward share 0; smallest 95th percentile "
                          f"angle {best:.2f} deg; flights within {TIE_DEG:g} deg of it: {sorted(near)}; the lowest rate wins"}
    name = min(forward, key=lambda n: (-metrics[n]["passages"], metrics[n]["nose_angle_deg_p95"]))
    return {"name": name, "qualified": False,
            "reason": f"no flight has {total} passages with backward share 0; took the most passages, then the smallest 95th percentile angle"}


def compare_states(ref: dict, other: dict) -> dict:
    """Step-by-step difference of two flights (rule of compare_flights in crazyflow_flight_video.py: the largest
    position distance over the steps both flights have). Also the largest absolute difference of quaternion and velocity."""
    n = min(len(ref["pos"]), len(other["pos"]))
    pos = max((math.dist(ref["pos"][i], other["pos"][i]) for i in range(n)), default=None)
    quat = float(np.abs(np.asarray(ref["quat"][:n], dtype=float) - np.asarray(other["quat"][:n], dtype=float)).max()) if n else None
    vel = float(np.abs(np.asarray(ref["vel"][:n], dtype=float) - np.asarray(other["vel"][:n], dtype=float)).max()) if n else None
    return {"rows_reference": int(len(ref["pos"])), "rows_other": int(len(other["pos"])), "rows_compared": int(n),
            "max_position_difference_m": pos, "max_quaternion_difference": quat, "max_velocity_difference": vel,
            "equal": bool(len(ref["pos"]) == len(other["pos"]) and pos == 0.0 and quat == 0.0)}


def has_padding_row(pos) -> int:
    """Number of rows equal to the warp position (-1, -1, -1)."""
    return int(np.all(np.asarray(pos, dtype=float) == np.asarray(PADDING_ROW), axis=1).sum())


def load_npz(path: Path) -> dict:
    z = np.load(path, allow_pickle=False)
    return {k: z[k] for k in z.files}


def add_check(checks: list, name: str, ok: bool, detail: str, **numbers) -> None:
    checks.append({"name": name, "ok": bool(ok), "detail": detail, **numbers})


# ---------------------------------------------------------------------------------------------
# Child: one flight in its own process
# ---------------------------------------------------------------------------------------------


class ChaseCamera:
    """The following camera of tests/crazyflow_flight_video.py (same numbers, same rule, see chase_view there)."""

    def __init__(self, core, obs0, freq):
        self.core, self.obs0, self.freq = core, obs0, freq
        self.direction, self.previous, self.log = None, None, []

    def kwargs(self, pos) -> dict:
        if self.direction is None:
            gate = self.obs0["gates_pos"][int(self.obs0["gate_sequence"][0])]  # first gate of the order
            dx, dy = float(gate[0] - pos[0]), float(gate[1] - pos[1])
            norm = math.hypot(dx, dy)
            self.direction = (dx / norm, dy / norm) if norm > 1e-6 else (1.0, 0.0)
        elif self.previous is not None:
            mx, my = pos[0] - self.previous[0], pos[1] - self.previous[1]
            step = math.hypot(mx, my)
            if step / (STEP_PER_PICTURE / self.freq) >= fv.CHASE_MIN_SPEED_MPS:
                a = fv.CHASE_FILTER_ALPHA
                fx = (1 - a) * self.direction[0] + a * mx / step
                fy = (1 - a) * self.direction[1] + a * my / step
                norm = math.hypot(fx, fy)
                if norm > 1e-6:
                    self.direction = (fx / norm, fy / norm)
        self.previous = list(pos)
        view = fv.chase_view(pos, self.direction)
        self.log.append({"azimuth_deg": view["azimuth"], "drone_position": list(pos)})
        viewer = self.core.sim.viewer  # MujocoRenderer, None until the first render (crazyflow sim.py)
        if viewer is not None and viewer.viewer is not None:
            cam = viewer.viewer.cam
            cam.lookat[:] = view["lookat"]
            cam.distance = view["distance"]
            cam.azimuth = view["azimuth"]
            cam.elevation = view["elevation"]
        return {"camera": -1, "cam_config": {**view, "lookat": np.asarray(view["lookat"])}}


def run_child(args) -> None:
    out = Path(args.out).resolve()
    sub = out / args.subdir
    sub.mkdir(parents=True, exist_ok=True)
    tag = args.tag
    result = {
        "tag": tag, "nose_mode": args.nose_mode, "yaw_rate_max": args.yaw_rate, "camera": args.camera,
        "drone_scale": args.drone_scale, "MUJOCO_GL": os.environ.get("MUJOCO_GL"), "versions": {}, "traceback": None,
    }
    env = None
    writer = None
    frames_file = None
    try:
        os.environ["SCIPY_ARRAY_API"] = "1"  # crazyflow's own camera examples set this before importing scipy
        import importlib.metadata as md
        import tomllib

        import gymnasium
        import jax
        import mujoco
        from gymnasium.wrappers.jax_to_numpy import JaxToNumpy
        from lsy_drone_racing.utils import load_config, load_controller
        from scipy.spatial.transform import Rotation as Rot

        import nose_yaw as ny
        import sync_film
        import track_env

        repo, assets = Path(args.repo).resolve(), Path(args.assets).resolve()
        result["versions"] = {
            "mujoco": mujoco.__version__, "jax": jax.__version__, "crazyflow": md.version("crazyflow"),
            "lsy_drone_racing": md.version("lsy_drone_racing"), "lsy_drone_racing_commit": args.lsy_commit,
        }
        result["asset_check"] = tl.check_assets(repo, assets)  # the model LSY loads must be the new gate model

        cfg = load_config(repo / "config" / "level0.toml")
        cfg.sim.render = False
        freq = int(cfg.env.freq)
        if cfg.env.id != track_env.BASE_ENV_ID:
            raise AssertionError(f"level0.toml names the environment {cfg.env.id!r}, expected {track_env.BASE_ENV_ID!r}")

        # Controllers: the example (only for the spline check) and ours, both through LSY's load_controller
        # (utils.py lines 23-55: the one Controller subclass defined in the file).
        example_cls = load_controller(repo / "lsy_drone_racing" / "control" / "state_controller.py")
        controller_cls = load_controller(CONTROLLERS_DIR / "nose_forward_controller.py")
        controller_cls.NOSE_MODE = args.nose_mode
        controller_cls.YAW_RATE_MAX = float(args.yaw_rate)
        import nose_forward_controller as nfc  # same file, imported by name only for its list of points

        with open(repo / "config" / "level0.toml", "rb") as f:
            level = tomllib.load(f)
        start_xy = [float(v) for v in level["env"]["track"]["drones"][0]["pos"][:2]]
        end_xy = [float(v) for v in nfc.POINTS_AFTER_START[-1][:2]]
        pads = {"pad_start": tuple(start_xy), "pad_end": tuple(end_xy)}
        result["pads_requested"] = {k: list(v) for k, v in pads.items()}

        env = gymnasium.make(
            track_env.register_pad_env(),
            freq=cfg.env.freq, sim_config=cfg.sim, sensor_range=cfg.env.sensor_range, control_mode=cfg.env.control_mode,
            track=cfg.env.track, disturbances=cfg.env.get("disturbances"), randomizations=cfg.env.get("randomizations"),
            seed=SEED, pads=pads, pads_dir=str(PADS_DIR),
        )
        env = JaxToNumpy(env)
        core = env.unwrapped
        obs0, info = env.reset()  # the one and only reset of this flight
        m = core.sim.mj_model
        pos0 = [float(v) for v in obs0["pos"]]
        result["start_position"] = pos0
        result["start_pad_error_m"] = float(max(abs(pos0[0] - start_xy[0]), abs(pos0[1] - start_xy[1])))
        if result["start_pad_error_m"] > START_TOL:
            raise AssertionError(f"start pad is {result['start_pad_error_m']} m away from the start position of the simulator")

        # The new look: orange drone (visual fields of the compiled model), then the drawing scale, both before the first picture.
        orange_info = tl.make_drone_orange(mujoco, np, m)
        ok, detail = tl.orange_readback(mujoco, np, m, orange_info)
        result["orange"] = {"ok": ok, "detail": detail}
        if args.still or args.drone_scale != 1.0:
            parts = tl.drone_parts(mujoco, np, m)
            if args.drone_scale != 1.0:
                if core.sim.viewer is not None:
                    raise AssertionError("a viewer exists already; the scaled meshes would not be uploaded")
                tl.set_drone_scale(np, m, parts, float(args.drone_scale))
            result["drawn_extent"] = tl.drawn_extent(mujoco, np, Rot, m, parts)

        # Spline check: our spline equals the example's at 200 times.
        t_check = np.linspace(0.0, 22.5, SPLINE_CHECK_TIMES)
        ex, mine = example_cls(obs0, info, cfg), controller_cls(obs0, info, cfg)
        diff = float(np.abs(np.asarray(ex._des_pos_spline(t_check)) - np.asarray(mine._des_pos_spline(t_check))).max())
        result["spline_check"] = {"max_abs_difference_m": diff, "times": SPLINE_CHECK_TIMES, "t_total_example": float(ex._t_total),
                                  "t_total_ours": float(mine._t_total), "ok": bool(diff <= SPLINE_TOL and ex._t_total == mine._t_total)}
        del ex, mine

        def sync() -> None:
            # Same sync as RaceCoreEnv.render: copy gate positions into the MuJoCo data.
            if not core.data.sim_data.core.mjx_synced:
                core.data, core.sim.mjx_data = core._render_sync(core.data, core.sim.mjx_data)

        def render(**kw):
            sync()
            img = np.ascontiguousarray(core.sim.render(mode="rgb_array", width=WIDTH, height=HEIGHT, **kw))
            if img.shape != (HEIGHT, WIDTH, 3) or img.dtype != np.uint8:
                raise ValueError(f"unexpected picture {img.shape} {img.dtype}")
            return img

        # Fixed outside still at the start (close to the drone on its pad): counts orange pixels, for the scale proof.
        if args.still:
            sx, sy, sz = pos0
            cam_cfg = {"lookat": np.asarray([sx, sy, sz + 0.02]), "distance": 0.4, "azimuth": 90.0, "elevation": -20.0}
            still = render(camera=-1, cam_config=cam_cfg)
            from PIL import Image

            Image.fromarray(still).save(out / f"{tag}_start_still.png")
            result["start_still"] = {"file": f"{tag}_start_still.png", "orange_pixels": int(tl.share_masks(np, still)["orange"].sum())}

        if args.no_flight:
            result["flown"] = False
            return

        # Flight
        controller = controller_cls(obs0, info, cfg)
        total = int(obs0["gate_sequence"].shape[0])
        rec = {k: [] for k in ("time", "pos", "vel", "quat", "ang_vel", "cmd_yaw", "passed")}

        def record(obs, cmd_yaw):
            rec["time"].append(len(rec["pos"]) / freq)
            rec["pos"].append([float(v) for v in obs["pos"]])
            rec["vel"].append([float(v) for v in obs["vel"]])
            rec["quat"].append([float(v) for v in obs["quat"]])
            rec["ang_vel"].append([float(v) for v in obs["ang_vel"]])
            rec["cmd_yaw"].append(float(cmd_yaw))
            rec["passed"].append(int(obs["n_gates_passed"]))

        pic_step, picture_seconds = [], []
        chase = ChaseCamera(core, obs0, freq) if args.camera == "chase" else None
        if args.camera != "none":
            film_name = args.film or f"{tag}.mp4"
            writer = sync_film.FilmWriter(out / film_name, WIDTH, HEIGHT, FPS, sub / f"{tag}_ffmpeg.log")
            result["film"] = film_name
            if args.frames:
                frames_file = open(args.frames, "wb")

        def picture() -> None:
            row = len(rec["pos"]) - 1
            kwargs = chase.kwargs(rec["pos"][row]) if chase else {"camera": "fpv_cam:0"}
            t0 = time.perf_counter()
            img = render(**kwargs)
            picture_seconds.append(time.perf_counter() - t0)
            writer.write(img)
            if frames_file is not None:
                frames_file.write(img.tobytes())
            pic_step.append(row)

        t_start = time.perf_counter()
        obs = obs0
        record(obs, controller.planned_yaw(0))
        if writer:
            picture()
        steps, end = 0, None
        dropped = 0
        # Loop of scripts/sim.py (compute_control, step, step_callback), with two differences that decide what is saved:
        # a step that ends the episode with terminated=True is NOT recorded (LSY has moved a disabled drone to
        # (-1, -1, -1), race_core.py line 851, and its step is not a flight position), and the loop stops at the step of
        # the last passage instead of taking one more step (the drone is disabled one step after the last passage,
        # race_core.py line 863, which is what put the padding row at the end of an earlier position file).
        while True:
            cmd_yaw = controller.planned_yaw(steps)
            action = controller.compute_control(obs, info)
            obs, reward, terminated, truncated, info = env.step(action)
            done = controller.step_callback(action, obs, reward, terminated, truncated, info)
            steps += 1
            if terminated:
                dropped += 1
                end = "crash"
                break
            record(obs, cmd_yaw)
            if writer and steps % STEP_PER_PICTURE == 0:
                picture()
            if int(obs["n_gates_passed"]) >= total:
                end = "finished"
                break
            if truncated or done:
                end = "time"
                break
        if writer and pic_step[-1] != len(rec["pos"]) - 1:
            picture()  # the last state always gets a picture
        wall = time.perf_counter() - t_start

        arrays = {
            "time": np.array(rec["time"]), "pos": np.array(rec["pos"]), "vel": np.array(rec["vel"]), "quat": np.array(rec["quat"]),
            "ang_vel": np.array(rec["ang_vel"]), "cmd_yaw": np.array(rec["cmd_yaw"]), "passed": np.array(rec["passed"], dtype=np.int64),
            "pic_step": np.array(pic_step, dtype=np.int64), "pic_time": np.array([rec["time"][r] for r in pic_step]),
            "plan": controller.plan(), "planned_xy": controller.planned_xy(200), "start_yaw": np.array(float(ny.yaw_from_quat_xyzw(np.asarray(obs0["quat"], dtype=float)))),
            "gates_pos": np.asarray(obs0["gates_pos"], dtype=float), "gates_quat": np.asarray(obs0["gates_quat"], dtype=float),
            "gate_sequence": np.asarray(obs0["gate_sequence"], dtype=np.int64),
            "gate_direction": np.asarray(obs0["gate_sequence_direction"], dtype=np.int64),
            "obstacles_pos": np.asarray(obs0["obstacles_pos"], dtype=float),
            "pad_start": np.array(start_xy), "pad_end": np.array(end_xy), "freq": np.array(freq),
        }
        padding = has_padding_row(arrays["pos"])
        result.update(
            steps=steps, rows_saved=int(len(arrays["pos"])), terminated_steps_dropped=dropped, padding_rows_in_saved_positions=padding,
            passages=int(arrays["passed"][-1]), gates_total=total, end=end, sim_seconds=float((len(arrays["pos"]) - 1) / freq),
            flight_wall_seconds=wall, pictures=len(pic_step),
            seconds_per_picture_mean=float(np.mean(picture_seconds)) if picture_seconds else None,
            seconds_per_picture_max=float(max(picture_seconds)) if picture_seconds else None,
            plan_rate_max_seen=float(np.abs(np.diff(arrays["plan"])).max() * freq) if len(arrays["plan"]) > 1 else 0.0,
            all_finite=bool(all(np.all(np.isfinite(v)) for v in arrays.values() if v.dtype.kind == "f")),
        )
        if padding:
            raise AssertionError(f"{padding} saved position row(s) equal the padding row (-1, -1, -1)")
        if not result["all_finite"]:
            raise AssertionError("a saved array has a value that is not a number")
        np.savez(sub / f"{tag}.npz", **arrays)
        if chase:
            result["chase_log_first"] = chase.log[:3]
        if writer:
            if frames_file is not None:
                frames_file.close()
                frames_file = None
            writer.close()
            writer = None
            result["film_bytes"] = (out / result["film"]).stat().st_size
        result["flown"] = True
    except Exception:
        result["traceback"] = traceback.format_exc()
    finally:
        if writer is not None:
            writer.abort()
        if frames_file is not None:
            frames_file.close()
        try:
            if env is not None:
                env.close()
        except Exception:
            result["close_traceback"] = traceback.format_exc()
        (sub / f"{tag}_child.json").write_text(json.dumps(result, indent=2, default=str))


# ---------------------------------------------------------------------------------------------
# Parent
# ---------------------------------------------------------------------------------------------


class Runner:
    """Starts child processes and remembers which drawing method worked."""

    def __init__(self, args):
        self.args = args
        self.out = Path(args.out).resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        (self.out / "logs").mkdir(exist_ok=True)
        self.methods = list(METHODS)

    def child_command(self, job: list) -> list:
        a = self.args
        return [sys.executable, str(Path(__file__).resolve()), "child", "--out", str(self.out), "--repo", str(Path(a.repo).resolve()),
                "--assets", str(Path(a.assets).resolve()), "--lsy-commit", a.lsy_commit] + job

    def run_once(self, job: list, tag: str, subdir: str, method: str | None) -> dict:
        child_json = self.out / subdir / f"{tag}_child.json"
        child_json.unlink(missing_ok=True)  # no stale file from an earlier try
        env = {**os.environ}
        if method:
            env["MUJOCO_GL"] = method
        entry = {"method": method}
        try:
            proc = subprocess.run(self.child_command(job), env=env, capture_output=True, text=True, timeout=CHILD_TIMEOUT_S)
            suffix = f"_{method}" if method else ""
            (self.out / "logs" / f"{tag}{suffix}_stdout.txt").write_text(proc.stdout)
            (self.out / "logs" / f"{tag}{suffix}_stderr.txt").write_text(proc.stderr)
            entry["returncode"] = proc.returncode
            entry["stderr_tail"] = proc.stderr[-2000:]
        except subprocess.TimeoutExpired:
            entry["process_error"] = f"child timed out after {CHILD_TIMEOUT_S} s"
        if child_json.is_file():
            entry.update(json.loads(child_json.read_text()))
        else:
            entry.setdefault("process_error", "child wrote no child json (crashed hard)")
        return entry

    def flight(self, tag: str, subdir: str, job: list, needs_gl: bool) -> dict:
        """One child; with pictures, try egl then osmesa and keep the method that worked."""
        if not needs_gl:
            return self.run_once(job + ["--tag", tag, "--subdir", subdir], tag, subdir, None)
        entry = {}
        for method in list(self.methods):
            entry = self.run_once(job + ["--tag", tag, "--subdir", subdir], tag, subdir, method)
            if entry.get("flown") is not None and not entry.get("traceback") and not entry.get("process_error"):
                self.methods = [method] + [x for x in self.methods if x != method]
                break
        return entry


def child_job(mode: str, rate: float, camera: str = "none", scale: float = 1.0, film: str | None = None, frames: str | None = None,
              still: bool = False, no_flight: bool = False) -> list:
    job = ["--nose-mode", mode, "--yaw-rate", repr(float(rate)), "--camera", camera, "--drone-scale", repr(float(scale))]
    if film:
        job += ["--film", film]
    if frames:
        job += ["--frames", frames]
    if still:
        job += ["--still"]
    if no_flight:
        job += ["--no-flight"]
    return job


def versions_of(entries: list) -> dict:
    v = {"python": platform.python_version()}
    for e in entries:
        v.update(e.get("versions", {}) or {})
    return v


def child_ok(entry: dict) -> bool:
    return bool(entry.get("flown")) and not entry.get("traceback") and not entry.get("process_error")


def short_error(entry: dict) -> str:
    text = (entry.get("traceback") or entry.get("process_error") or "").strip()
    return text.splitlines()[-1][:200] if text else ""


# -- sweep --


def cmd_sweep(args) -> None:
    import sync_film

    runner = Runner(args)
    out = runner.out
    sweep = {"flights": {}, "checks": [], "versions": {}}
    entries, arrays = {}, {}
    modes, rates = {}, {}
    for name, mode, rate in flight_list():
        modes[name], rates[name] = mode, rate
        entry = runner.flight(name, "sweep", child_job(mode, rate), needs_gl=False)
        entries[name] = entry
        npz = out / "sweep" / f"{name}.npz"
        if child_ok(entry) and npz.is_file():
            arrays[name] = load_npz(npz)
    sweep["versions"] = versions_of(list(entries.values()))
    freq = float(next((a["freq"] for a in arrays.values()), 50.0))
    total = int(next((e.get("gates_total") for e in entries.values() if e.get("gates_total")), 5))
    fixed_pos = arrays["fixed"]["pos"] if "fixed" in arrays else None

    metrics = {}
    for name, e in entries.items():
        row = {"nose_mode": modes[name], "yaw_rate_max": rates[name] if modes[name] == "forward" else None, "ran": name in arrays,
               "end": e.get("end"), "error": short_error(e), "wall_seconds": e.get("flight_wall_seconds")}
        if name in arrays:
            row.update(flight_metrics(arrays[name], freq, fixed_pos))
            row["plan_rows"] = int(len(arrays[name]["plan"]))
        metrics[name] = row
        sweep["flights"][name] = row

    choice = choose({n: (m if m["ran"] else None) for n, m in metrics.items()}, modes, rates, total)
    chosen = {**choice, "total_passages": total}
    if choice["name"]:
        chosen.update(nose_mode=modes[choice["name"]], yaw_rate_max=rates[choice["name"]], metrics=metrics[choice["name"]])
    (out / "chosen.json").write_text(json.dumps(chosen, indent=2))

    checks = sweep["checks"]
    ran = [n for n in metrics if metrics[n]["ran"]]
    add_check(checks, "sweep_all_flights_ran", len(ran) == len(metrics),
              f"{len(ran)} of {len(metrics)} flights wrote their states; failed: " + (", ".join(f"{n} ({metrics[n]['error']})" for n in metrics if not metrics[n]["ran"]) or "none"))
    spl = {n: e.get("spline_check", {}) for n, e in entries.items() if e.get("spline_check")}
    worst = max((s.get("max_abs_difference_m", math.inf) for s in spl.values()), default=None)
    add_check(checks, "spline_equals_example_controller", bool(spl) and len(spl) == len(entries) and all(s.get("ok") for s in spl.values()),
              f"our spline against the example controller's at {SPLINE_CHECK_TIMES} times, in {len(spl)} flights: largest difference {worst} m (limit {SPLINE_TOL:g}); same flight time",
              value=worst)
    pad = {n: e.get("padding_rows_in_saved_positions") for n, e in entries.items() if e.get("flown")}
    recheck = {n: has_padding_row(a["pos"]) for n, a in arrays.items()}
    add_check(checks, "no_padding_row_in_saved_positions", bool(recheck) and all(v == 0 for v in recheck.values()) and all(v == 0 for v in pad.values()),
              f"rows equal to (-1, -1, -1) in the saved positions: {recheck}; the child counted: {pad}; terminated steps dropped: "
              f"{ {n: e.get('terminated_steps_dropped') for n, e in entries.items() if e.get('flown')} }")
    bad_plans = []
    for name in ran:
        z, mode, rate = arrays[name], modes[name], rates[name]
        plan = z["plan"]
        if mode == "fixed":
            if not np.all(plan == 0.0):
                bad_plans.append(f"{name}: baseline plan is not 0.0 everywhere")
        else:
            worst_step = float(np.abs(np.diff(plan)).max()) if len(plan) > 1 else 0.0
            if worst_step > rate / freq + 1e-12:
                bad_plans.append(f"{name}: largest step {worst_step} rad > {rate / freq + 1e-12}")
            if float(plan[0]) != float(z["start_yaw"]):
                bad_plans.append(f"{name}: plan[0] {float(plan[0])} != start yaw {float(z['start_yaw'])}")
    add_check(checks, "plans_obey_rate_limit", bool(ran) and not bad_plans,
              f"for every forward flight: largest yaw step per control step <= rate / {freq:g} Hz + 1e-12 and plan[0] equals the start yaw; the baseline plan is 0.0; problems: {bad_plans or 'none'}")
    fx = metrics.get("fixed", {})
    share = fx.get("backward_share")
    add_check(checks, "positive_control_baseline_flies_backward", share is not None and share > MIN_BACKWARD_SHARE_OF_FIXED,
              f"the baseline (yaw 0.0) flies backward (nose more than {BACKWARD_DEG:g} deg from the velocity) in {None if share is None else round(100 * share, 1)} % of its moving steps; "
              f"needs more than {100 * MIN_BACKWARD_SHARE_OF_FIXED:g} %, otherwise the measure cannot see the fault the owner saw", value=share)
    add_check(checks, "a_flight_qualifies", bool(chosen.get("qualified")),
              f"chosen: {choice['name']}; {choice['reason']}", value=choice["name"])
    (out / "sweep.json").write_text(json.dumps(sweep, indent=2, default=str))

    # yaw_sweep.png: commanded and actual yaw, and the nose-versus-velocity angle, one color per flight
    import nose_yaw as ny

    palette = {"fixed": (0, 0, 0), "forward_0p5": (230, 159, 0), "forward_1": (86, 180, 233), "forward_2": (0, 158, 115),
               "forward_4": (204, 121, 167), "forward_8": (213, 94, 0)}
    yaw_series, angle_series = [], []
    for name in ran:
        z = arrays[name]
        t = z["time"]
        yaw = ny.yaw_from_quat_xyzw(z["quat"])
        actual = z["cmd_yaw"] + ny.wrap_pi(yaw - z["cmd_yaw"])  # actual yaw drawn next to the command, no 2*pi jumps
        label = "baseline (yaw 0.0)" if name == "fixed" else f"nose forward, {rates[name]:g} rad/s"
        yaw_series.append({"label": label + " command", "color": palette.get(name, (90, 90, 90)), "x": t, "y": np.degrees(z["cmd_yaw"]), "dashed": True})
        yaw_series.append({"label": label + " actual", "color": palette.get(name, (90, 90, 90)), "x": t, "y": np.degrees(actual)})
        ang = ny.nose_vs_velocity_deg(yaw, z["vel"][:, :2])
        ang = np.where(np.hypot(z["vel"][:, 0], z["vel"][:, 1]) >= V_MIN_MOVING, ang, np.nan)
        angle_series.append({"label": label, "color": palette.get(name, (90, 90, 90)), "x": t, "y": ang})
    if yaw_series:
        sync_film.line_chart(out / "yaw_sweep.png", [
            {"title": "Yaw of the drone: command (dashed) and actual (solid)", "ylabel": "degrees", "series": yaw_series},
            {"title": "Angle between the nose and the horizontal velocity (steps with speed of at least 0.3 m/s)", "ylabel": "degrees",
             "series": angle_series, "hlines": [(BACKWARD_DEG, "backward above 90 degrees")]},
        ])
    print(f"sweep done: {len(ran)} of {len(metrics)} flights; chosen {choice['name']} (qualified {chosen.get('qualified')})")


# -- films --


def track_from_arrays(z: dict) -> dict:
    return {
        "gates_pos": z["gates_pos"], "gates_quat": z["gates_quat"], "gate_sequence": z["gate_sequence"], "gate_direction": z["gate_direction"],
        "obstacles_pos": z["obstacles_pos"], "pad_start": tuple(float(v) for v in z["pad_start"]), "pad_end": tuple(float(v) for v in z["pad_end"]),
        "plan_xy": z["planned_xy"], "total_passages": int(len(z["gate_sequence"])),
    }


def load_chosen(out: Path) -> dict:
    f = out / "chosen.json"
    if not f.is_file():
        raise FileNotFoundError(f"{f} does not exist: run the sweep first")
    chosen = json.loads(f.read_text())
    if not chosen.get("name") or chosen.get("nose_mode") != "forward":
        raise ValueError(f"no nose-forward flight was chosen: {chosen}")
    return chosen


def cmd_films(args) -> None:
    import sync_film

    runner = Runner(args)
    out = runner.out
    films = {"films": {}, "checks": [], "versions": {}}
    checks = films["checks"]
    work = Path(tempfile.mkdtemp(prefix="run_a_work_"))
    try:
        chosen = load_chosen(out)
        name, rate = chosen["name"], float(chosen["yaw_rate_max"])
        films["chosen"] = {"name": name, "yaw_rate_max": rate}
        ref = load_npz(out / "sweep" / f"{name}.npz")
        frames_path = work / "fpv_frames.rgb"
        specs = {"fpv": ("fpv.mp4", frames_path), "chase": ("chase.mp4", None)}
        entries, states = {}, {}
        for cam, (film, frames) in specs.items():
            entries[cam] = runner.flight(f"film_{cam}", "films", child_job("forward", rate, camera=cam, film=film, frames=str(frames) if frames else None), needs_gl=True)
            npz = out / "films" / f"film_{cam}.npz"
            if child_ok(entries[cam]) and npz.is_file():
                states[cam] = load_npz(npz)
        films["versions"] = versions_of(list(entries.values()))
        for cam, e in entries.items():
            films["films"][cam] = {k: e.get(k) for k in ("film", "film_bytes", "pictures", "steps", "passages", "end", "method", "flight_wall_seconds",
                                                         "seconds_per_picture_mean", "seconds_per_picture_max", "start_pad_error_m")}
            films["films"][cam]["error"] = short_error(e)
        # 1. each film flight equals the chosen sweep flight, step by step
        for cam in specs:
            if cam in states:
                cmp = compare_states(ref, states[cam])
                films["films"][cam]["compare_with_sweep"] = cmp
                add_check(checks, f"film_flight_{cam}_equals_sweep_flight", cmp["equal"],
                          f"{cam} film flight against sweep flight {name}: {cmp['rows_other']} rows against {cmp['rows_reference']}; largest position difference "
                          f"{cmp['max_position_difference_m']} m, quaternion {cmp['max_quaternion_difference']}, velocity {cmp['max_velocity_difference']} (must be exactly 0.0)",
                          value=cmp["max_position_difference_m"])
            else:
                add_check(checks, f"film_flight_{cam}_equals_sweep_flight", False, f"the {cam} film flight did not run: {short_error(entries[cam])}")
        # 2. no padding row
        pads = {cam: has_padding_row(z["pos"]) for cam, z in states.items()}
        add_check(checks, "films_no_padding_row", bool(pads) and all(v == 0 for v in pads.values()), f"rows equal to (-1, -1, -1) in the saved film flights: {pads}")
        # 3. the film with the map
        sync_info = None
        if "fpv" in states and frames_path.is_file():
            z = states["fpv"]
            n_pic = len(z["pic_step"])
            frames = np.memmap(frames_path, dtype=np.uint8, mode="r", shape=(n_pic, HEIGHT, WIDTH, 3))
            try:
                sync_info = sync_film.make_sync_film(frames, z, track_from_arrays(z), out / "fpv_and_map.mp4", FPS)
            except Exception:
                sync_info = {"error": traceback.format_exc()[-1500:]}
            del frames
        films["films"]["fpv_and_map"] = {"film": "fpv_and_map.mp4", **(sync_info or {"error": "not made: the fpv film flight is missing"})}
        # 4. contact sheets and decoded picture counts (what ffmpeg decodes, not what a writer says)
        decoded = {}
        sizes = {"fpv": (WIDTH, HEIGHT), "chase": (WIDTH, HEIGHT), "fpv_and_map": (WIDTH + HEIGHT, HEIGHT)}
        times = states["fpv"]["pic_time"] if "fpv" in states else []
        for film_key, (w, h) in sizes.items():
            path = out / f"{film_key}.mp4"
            if path.is_file() and path.stat().st_size > 0 and len(times):
                try:
                    info = sync_film.film_contact_sheet(path, w, h, times, out / f"{film_key}_contact.png")
                    films["films"][film_key]["contact_sheet"] = info
                    decoded[film_key] = info["pictures_decoded"]
                except Exception:
                    films["films"][film_key]["contact_sheet_error"] = traceback.format_exc()[-800:]
        written = {"fpv": entries["fpv"].get("pictures"), "chase": entries["chase"].get("pictures"), "fpv_and_map": (sync_info or {}).get("pictures_written")}
        counts_equal = len(decoded) == 3 and len(set(decoded.values())) == 1 and len({v for v in written.values() if v is not None}) == 1 \
            and set(decoded.values()) == {v for v in written.values() if v is not None}
        add_check(checks, "films_picture_counts_equal", counts_equal,
                  f"pictures written {written}; pictures decoded from the mp4 files {decoded}; all six numbers must be equal")
        ok_files = all((out / f"{k}.mp4").is_file() and (out / f"{k}.mp4").stat().st_size > 0 and (out / f"{k}_contact.png").is_file() for k in sizes)
        add_check(checks, "films_and_contact_sheets_exist", ok_files, f"fpv.mp4, chase.mp4, fpv_and_map.mp4 and their contact sheets in {out}")
        if sync_info and "seconds_per_picture_max" in sync_info:
            add_check(checks, "map_picture_costs_under_0p1_s", sync_info["seconds_per_picture_max"] < 0.1,
                      f"map and combined picture: mean {sync_info['seconds_per_picture_mean'] * 1000:.1f} ms, slowest {sync_info['seconds_per_picture_max'] * 1000:.1f} ms")
        orange = {c: e.get("orange", {}).get("ok") for c, e in entries.items()}
        add_check(checks, "films_drone_is_orange_and_gate_model_is_new", all(orange.values()) and all(e.get("asset_check", {}).get("ok") for e in entries.values()),
                  f"orange read back from the model: {orange}; gate model files equal to assets/gates/square (hashes): "
                  f"{ {c: e.get('asset_check', {}).get('ok') for c, e in entries.items()} }")
    except Exception:
        films["error"] = traceback.format_exc()
        add_check(checks, "films_step_completed", False, "the films step stopped: " + films["error"].strip().splitlines()[-1])
    finally:
        (out / "films.json").write_text(json.dumps(films, indent=2, default=str))
        try:
            for p in work.iterdir():
                p.unlink()
            work.rmdir()
        except Exception:
            pass
    print("films done:", {c["name"]: c["ok"] for c in checks})


# -- scale --


def cmd_scale(args) -> None:
    import sync_film

    runner = Runner(args)
    out = runner.out
    result = {"scales": {}, "checks": [], "versions": {}}
    checks = result["checks"]
    try:
        chosen = load_chosen(out)
        name, rate = chosen["name"], float(chosen["yaw_rate_max"])
        result["chosen"] = {"name": name, "yaw_rate_max": rate}
        ref = load_npz(out / "sweep" / f"{name}.npz")
        entries, states = {}, {}
        for s in SCALES:
            tag = f"scale_{scale_tag(s)}"
            if s == 1.0:  # the reference picture only: the flight at 1.0 is the sweep flight
                entries[s] = runner.flight(tag, "scale", child_job("forward", rate, scale=s, still=True, no_flight=True), needs_gl=True)
            else:
                film = f"chase_scale_{scale_tag(s)}.mp4"
                entries[s] = runner.flight(tag, "scale", child_job("forward", rate, camera="chase", scale=s, film=film, still=True), needs_gl=True)
                npz = out / "scale" / f"{tag}.npz"
                if child_ok(entries[s]) and npz.is_file():
                    states[s] = load_npz(npz)
        result["versions"] = versions_of(list(entries.values()))
        counts, widths = {}, {}
        for s, e in entries.items():
            counts[s] = (e.get("start_still") or {}).get("orange_pixels")
            widths[s] = (e.get("drawn_extent") or {}).get("drawn_width_m")
            result["scales"][f"{s:g}"] = {"orange_pixels": counts[s], "drawn_width_m": widths[s], "film": e.get("film"), "pictures": e.get("pictures"),
                                           "still": (e.get("start_still") or {}).get("file"), "error": short_error(e), "method": e.get("method")}
        # flights at 1.5 and 2.0 against the flight at 1.0
        for s in (1.5, 2.0):
            key = f"scale_{scale_tag(s)}_flight_equals_scale_1p0_flight"
            if s in states:
                cmp = compare_states(ref, states[s])
                result["scales"][f"{s:g}"]["compare_with_scale_1"] = cmp
                worst = max(cmp["max_position_difference_m"] or 0.0, cmp["max_quaternion_difference"] or 0.0, cmp["max_velocity_difference"] or 0.0)
                add_check(checks, key, cmp["equal"],
                          f"drone drawn at {s:g}: {cmp['rows_other']} rows against {cmp['rows_reference']} at 1.0; largest absolute difference of position {cmp['max_position_difference_m']} m, "
                          f"quaternion {cmp['max_quaternion_difference']}, velocity {cmp['max_velocity_difference']}; passes only at exactly 0.0", value=worst)
            else:
                add_check(checks, key, False, f"the flight at scale {s:g} did not run: {short_error(entries[s])}")
        # positive control: the scale was really active
        a, b, c = counts.get(1.0), counts.get(1.5), counts.get(2.0)
        add_check(checks, "scale_positive_control_orange_pixels_grow", None not in (a, b, c) and a < b < c,
                  f"orange pixels in one still from a fixed outside camera at the start (0.4 m from the drone): scale 1.0 -> {a}, 1.5 -> {b}, 2.0 -> {c}; each must be larger than the one before")
        w1 = widths.get(1.0)
        ratios = {s: (widths[s] / w1 if w1 and widths.get(s) else None) for s in (1.5, 2.0)}
        add_check(checks, "scale_drawn_size_equals_scale", all(r is not None and abs(r / s - 1.0) <= tl.SCALE_READBACK_TOL for s, r in ratios.items()),
                  f"drawn width of the drone read from the model's vertices {widths}; ratio to scale 1.0: {ratios} (tolerance {tl.SCALE_READBACK_TOL:g})")
        # contact sheets and picture counts of the two films
        decoded = {}
        for s in (1.5, 2.0):
            film = out / f"chase_scale_{scale_tag(s)}.mp4"
            if film.is_file() and s in states:
                info = sync_film.film_contact_sheet(film, WIDTH, HEIGHT, states[s]["pic_time"], out / f"chase_scale_{scale_tag(s)}_contact.png")
                result["scales"][f"{s:g}"]["contact_sheet"] = info
                decoded[s] = info["pictures_decoded"]
        written = {s: entries[s].get("pictures") for s in (1.5, 2.0)}
        add_check(checks, "scale_films_exist_with_all_pictures", len(decoded) == 2 and decoded == written and all(v for v in decoded.values()),
                  f"pictures written {written}, decoded from the mp4 files {decoded}; contact sheets written: {sorted(k for k in decoded)}")
        pads = {s: has_padding_row(z["pos"]) for s, z in states.items()}
        add_check(checks, "scale_no_padding_row", bool(pads) and all(v == 0 for v in pads.values()), f"rows equal to (-1, -1, -1): {pads}")
    except Exception:
        result["error"] = traceback.format_exc()
        add_check(checks, "scale_step_completed", False, "the scale step stopped: " + result["error"].strip().splitlines()[-1])
    finally:
        (out / "scale.json").write_text(json.dumps(result, indent=2, default=str))
    print("scale done:", {c["name"]: c["ok"] for c in checks})


# -- summary and check --


def read_json(path: Path):
    return json.loads(path.read_text()) if path.is_file() else None


def fmt(v, spec=".2f", dash="-"):
    return dash if v is None else format(v, spec)


def cmd_summary(args) -> None:
    out = Path(args.out).resolve()
    sweep, chosen, films, scale = (read_json(out / f) for f in ("sweep.json", "chosen.json", "films.json", "scale.json"))
    checks = []
    outcomes = {"unit_tests": args.unit_outcome}
    if args.unit_outcome is not None:
        add_check(checks, "unit_tests_of_nose_yaw", args.unit_outcome == "success", f"python tests/test_nose_yaw.py ended with outcome {args.unit_outcome!r}")
    for label, data in (("sweep", sweep), ("films", films), ("scale", scale)):
        if data is None:
            add_check(checks, f"{label}_result_file_exists", False, f"{label}.json was not written: that step did not run or crashed before it could write")
        else:
            checks.extend(data.get("checks", []))
    failed = [c["name"] for c in checks if not c["ok"]]
    result = {"passed": not failed and bool(checks), "n_checks": len(checks), "failed": failed, "checks": checks, "outcomes": outcomes,
              "chosen": chosen, "versions": {**(sweep or {}).get("versions", {}), **(films or {}).get("versions", {})}}
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))

    L = ["### Run A: nose along the direction of flight", ""]
    L.append(f"**{'PASS' if result['passed'] else 'FAIL'}**: {len(checks) - len(failed)} of {len(checks)} checks passed" + (f"; failed: {', '.join(failed)}" if failed else "") + ".")
    L += ["", "#### Sweep (6 flights of level 0, no pictures)", ""]
    if sweep:
        L += ["| flight | passages | end | sim s | nose angle max / p95 / mean (deg) | backward share | yaw error max / p95 (deg) | tilt max (deg) | largest distance to baseline (m) |",
              "|---|---|---|---|---|---|---|---|---|"]
        for n, m in sweep["flights"].items():
            if not m.get("ran"):
                L.append(f"| {n} | did not run | {m.get('error', '')} | | | | | | |")
                continue
            L.append(f"| {n} | {m['passages']} | {m['end']} | {fmt(m['sim_seconds'])} | {fmt(m['nose_angle_deg_max'], '.1f')} / {fmt(m['nose_angle_deg_p95'], '.1f')} / {fmt(m['nose_angle_deg_mean'], '.1f')} | "
                     f"{fmt(None if m['backward_share'] is None else 100 * m['backward_share'], '.1f')} % | {fmt(m['yaw_error_deg_max'], '.1f')} / {fmt(m['yaw_error_deg_p95'], '.1f')} | "
                     f"{fmt(m['tilt_deg_max'], '.1f')} | {fmt(m['max_distance_to_fixed_m'], '.3f')} |")
        L += ["", "The nose angle is counted over the steps with a horizontal speed of at least 0.3 m/s. Backward = more than 90 degrees. The picture yaw_sweep.png shows the yaw and the angle over time."]
    else:
        L.append("The sweep did not write sweep.json.")
    L += ["", "#### Choice", ""]
    if chosen and chosen.get("name"):
        L.append(f"Chosen: **{chosen['name']}** (rate {chosen['yaw_rate_max']:g} rad/s). Qualified: {chosen.get('qualified')}. {chosen.get('reason', '')}")
    else:
        L.append("Nothing was chosen." + (f" {chosen.get('reason', '')}" if chosen else ""))
    L += ["", "#### Films", ""]
    if films:
        L += ["| film | pictures | size (MB) | seconds per picture (mean / slowest) |", "|---|---|---|---|"]
        for key, e in films["films"].items():
            mp4 = out / f"{key}.mp4"
            size = mp4.stat().st_size / 1e6 if mp4.is_file() else None
            pics = e.get("pictures") if "pictures" in e else e.get("pictures_written")
            mean = e.get("seconds_per_picture_mean")
            slow = e.get("seconds_per_picture_max")
            L.append(f"| {key}.mp4 | {pics if pics is not None else '-'} | {fmt(size)} | {fmt(mean, '.3f')} / {fmt(slow, '.3f')} |")
    else:
        L.append("The films step did not write films.json.")
    L += ["", "#### Drawing scale", ""]
    if scale:
        L += ["| scale | orange pixels in the start still | drawn width (m) | largest position difference to scale 1.0 (m) | quaternion | pictures |", "|---|---|---|---|---|---|"]
        for s, e in scale["scales"].items():
            c = e.get("compare_with_scale_1") or {}
            L.append(f"| {s} | {e.get('orange_pixels')} | {fmt(e.get('drawn_width_m'), '.4f')} | {c.get('max_position_difference_m', '-')} | {c.get('max_quaternion_difference', '-')} | {e.get('pictures', '-')} |")
    else:
        L.append("The scale step did not write scale.json.")
    L += ["", "#### Checks", "", "| check | result | detail |", "|---|---|---|"]
    for c in checks:
        L.append(f"| {c['name']} | {'pass' if c['ok'] else 'FAIL'} | {str(c['detail']).replace('|', '/')} |")
    L += ["", "Versions: " + ", ".join(f"{k} {v}" for k, v in result["versions"].items()), ""]
    (out / "summary.md").write_text("\n".join(L))
    print(("PASS: " if result["passed"] else "FAIL: ") + f"{len(checks) - len(failed)} of {len(checks)} checks passed" + (f"; failed: {failed}" if failed else ""))
    sys.exit(0 if result["passed"] else 1)


def cmd_check(args) -> None:
    f = Path(args.result)
    if not f.is_file():
        print(f"FAIL: result file {f} does not exist")
        sys.exit(1)
    r = json.loads(f.read_text())
    if r.get("passed"):
        print(f"PASS: {r['n_checks']} checks passed")
        sys.exit(0)
    print(f"FAIL: failed checks: {r.get('failed')}")
    sys.exit(1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["sweep", "films", "scale", "summary", "check", "child"])
    parser.add_argument("--out", help="folder for all output files")
    parser.add_argument("--repo", help="checkout of lsy_drone_racing")
    parser.add_argument("--assets", default=str(REPO_DIR / "assets" / "gates" / "square"), help="folder with the new gate model")
    parser.add_argument("--lsy-commit", default="unknown", help="commit of the checkout, only recorded")
    parser.add_argument("--unit-outcome", help="summary: outcome of the test_nose_yaw step (success or failure)")
    parser.add_argument("--result", help="check: result.json")
    # internal (child)
    parser.add_argument("--tag")
    parser.add_argument("--subdir", default="states")
    parser.add_argument("--nose-mode", default="forward", choices=["forward", "fixed"])
    parser.add_argument("--yaw-rate", type=float, default=2.0)
    parser.add_argument("--camera", default="none", choices=["none", "fpv", "chase"])
    parser.add_argument("--drone-scale", type=float, default=1.0)
    parser.add_argument("--film")
    parser.add_argument("--frames")
    parser.add_argument("--still", action="store_true")
    parser.add_argument("--no-flight", action="store_true")
    args = parser.parse_args()
    if args.command == "check":
        if not args.result:
            parser.error("check needs --result")
        cmd_check(args)
        return
    if not args.out:
        parser.error("--out is required")
    if args.command == "summary":
        cmd_summary(args)
        return
    if not args.repo:
        parser.error("--repo is required")
    if args.command == "child":
        run_child(args)
    elif args.command == "sweep":
        cmd_sweep(args)
    elif args.command == "films":
        cmd_films(args)
    elif args.command == "scale":
        cmd_scale(args)


if __name__ == "__main__":
    main()
