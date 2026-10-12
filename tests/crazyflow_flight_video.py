"""Film a flight through the level-0 track of LSY Drone Racing with the repository's own controller.

Parent mode (default): runs one child process per film (camera "fpv" = fpv_cam:0 on the drone,
camera "outside" = the free camera), each with the same seed. A child tries the drawing method
`egl` and the parent retries with `osmesa` if no film came out. The parent compares the drone
positions of the two flights, makes a side-by-side film, and writes result.json and summary.md.

Child mode (--child CAMERA --method METHOD): flies once, pipes the pictures into ffmpeg.

Check mode (--check FILE): applies the pass rule; only the standard library is imported.
"""

import argparse
import json
import os
import platform
import subprocess
import sys
import time
import traceback
from pathlib import Path

CAMERAS = ("fpv", "outside")
METHODS = ("egl", "osmesa")
WIDTH, HEIGHT = 640, 480
FPS = 25  # level0.toml has env.freq = 50 Hz; 30 is not a divisor of 50, so use every 2nd step
STEP_PER_PICTURE = 2
N_STILLS = 6
SEED = 0
MIN_PICTURES = 60
CHILD_TIMEOUT_S = 1500


# ---------------------------------------------------------------------------------------------
# Pass rule (used by the parent and by --check)
# ---------------------------------------------------------------------------------------------


def evaluate(result: dict, base: Path) -> tuple[bool, str]:
    """Pass if both films exist with at least 60 pictures each and one gate was passed."""
    problems = []
    gates = 0
    for camera in CAMERAS:
        entry = result.get("films", {}).get(camera, {})
        film = base / entry["film"] if entry.get("film") else None
        if film is None or not film.is_file() or film.stat().st_size == 0:
            problems.append(f"no film for camera {camera}")
        elif entry.get("pictures", 0) < MIN_PICTURES:
            problems.append(f"film {camera} has only {entry.get('pictures', 0)} pictures")
        gates = max(gates, entry.get("gates_passed") or 0)
    if gates < 1:
        problems.append("no gate was passed")
    if problems:
        return False, "; ".join(problems)
    return True, f"both films exist with at least {MIN_PICTURES} pictures; most gates passed: {gates}"


def check(result_file: Path) -> int:
    if not result_file.is_file():
        print(f"FAIL: result file {result_file} does not exist")
        return 1
    ok, reason = evaluate(json.loads(result_file.read_text()), result_file.parent)
    print(("PASS: " if ok else "FAIL: ") + reason)
    return 0 if ok else 1


# ---------------------------------------------------------------------------------------------
# Child: one flight, one film
# ---------------------------------------------------------------------------------------------


def start_ffmpeg(film: Path, log_file):
    """Raw RGB pictures in on stdin, H.264 yuv420p mp4 out."""
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FPS), "-i", "-",
        "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(film),
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=log_file)


def extract_stills(film: Path, out: Path, camera: str, n_pictures: int) -> list[str]:
    """Save N_STILLS pictures spread over the film (taken from the mp4, so slightly compressed)."""
    names = []
    for k in range(N_STILLS):
        index = round(k * (n_pictures - 1) / (N_STILLS - 1))
        name = f"{camera}_still_{k}.png"
        subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{index / FPS:.4f}", "-i", str(film),
             "-frames:v", "1", str(out / name)],
            check=True,
        )
        names.append(name)
    return names


def run_child(camera: str, method: str, repo: Path, out: Path, lsy_commit: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    result = {
        "camera": camera,
        "method": method,
        "MUJOCO_GL": os.environ.get("MUJOCO_GL"),
        "versions": {},
        "film": None,
        "traceback": None,
    }
    env = None
    ffmpeg = None
    positions = []
    try:
        # crazyflow's own camera examples set this before importing scipy
        os.environ["SCIPY_ARRAY_API"] = "1"
        import gymnasium
        import importlib.metadata as md
        import jax
        import mujoco
        import numpy as np
        from gymnasium.wrappers.jax_to_numpy import JaxToNumpy
        from lsy_drone_racing.utils import load_config, load_controller

        result["versions"] = {
            "mujoco": mujoco.__version__,
            "jax": jax.__version__,
            "crazyflow": md.version("crazyflow"),
            "lsy_drone_racing": md.version("lsy_drone_racing"),
            "lsy_drone_racing_commit": lsy_commit,
        }

        # Same set-up as scripts/sim.py: the config names the controller file, the environment is
        # wrapped in JaxToNumpy. The seed is fixed; the disturbances of level0.toml stay on.
        cfg = load_config(repo / "config" / "level0.toml")
        cfg.sim.render = False
        controller_path = repo / "lsy_drone_racing" / "control" / cfg.controller.file
        controller_cls = load_controller(controller_path)
        env = gymnasium.make(
            cfg.env.id,
            freq=cfg.env.freq,
            sim_config=cfg.sim,
            sensor_range=cfg.env.sensor_range,
            control_mode=cfg.env.control_mode,
            track=cfg.env.track,
            disturbances=cfg.env.get("disturbances"),
            randomizations=cfg.env.get("randomizations"),
            seed=SEED,
        )
        env = JaxToNumpy(env)
        core = env.unwrapped
        result.update(
            controller_file=cfg.controller.file,
            env_freq=int(cfg.env.freq),
            picture_every_n_steps=STEP_PER_PICTURE,
            fps=FPS,
            seed=SEED,
        )

        render_kwargs = {"camera": "fpv_cam:0"}
        if camera == "outside":
            cam_cfg = {k: np.asarray(v) if k == "lookat" else v for k, v in cfg.sim.cam_config[0].items()}
            render_kwargs = {"camera": -1, "cam_config": cam_cfg}

        film = out / f"{camera}.mp4"
        ffmpeg_log = open(out / f"{camera}_ffmpeg.log", "wb")
        ffmpeg = start_ffmpeg(film, ffmpeg_log)
        picture_seconds = []
        picture_std = []

        def picture() -> None:
            # Same sync as RaceCoreEnv.render: copy gate positions into the MuJoCo data.
            if not core.data.sim_data.core.mjx_synced:
                core.data, core.sim.mjx_data = core._render_sync(core.data, core.sim.mjx_data)
            t0 = time.perf_counter()
            img = core.sim.render(mode="rgb_array", width=WIDTH, height=HEIGHT, **render_kwargs)
            picture_seconds.append(time.perf_counter() - t0)
            img = np.ascontiguousarray(img)
            if img.shape != (HEIGHT, WIDTH, 3) or img.dtype != np.uint8:
                raise ValueError(f"unexpected picture {img.shape} {img.dtype}")
            picture_std.append(float(img.std()))
            ffmpeg.stdin.write(img.tobytes())

        t_start = time.perf_counter()
        obs, info = env.reset()
        controller = controller_cls(obs, info, cfg)
        gates_total = int(obs["gate_sequence"].shape[0])
        positions.append([float(v) for v in obs["pos"]])
        picture()

        # Loop of scripts/sim.py: compute_control, step, step_callback.
        steps = 0
        terminated = truncated = controller_done = False
        while True:
            action = controller.compute_control(obs, info)
            obs, reward, terminated, truncated, info = env.step(action)
            controller_done = controller.step_callback(action, obs, reward, terminated, truncated, info)
            steps += 1
            positions.append([float(v) for v in obs["pos"]])
            if steps % STEP_PER_PICTURE == 0:
                picture()
            if terminated or truncated or controller_done:
                break

        gates_passed = int(obs["n_gates_passed"])
        if gates_passed == gates_total:
            end = "finished"
        elif terminated:
            end = "crash"
        else:
            end = "time"
        result.update(
            steps=steps,
            sim_seconds=steps / cfg.env.freq,
            gates_passed=gates_passed,
            gates_total=gates_total,
            end=end,
            end_detail=f"terminated={bool(terminated)} truncated={bool(truncated)} controller_done={bool(controller_done)}",
            flight_wall_seconds=time.perf_counter() - t_start,
            pictures=len(picture_seconds),
            seconds_per_picture_first=picture_seconds[0],
            seconds_per_picture_mean_without_first=float(np.mean(picture_seconds[1:])) if len(picture_seconds) > 1 else None,
            seconds_per_picture_max=max(picture_seconds),
            picture_std_min=min(picture_std),
            picture_std_mean=float(np.mean(picture_std)),
        )

        t0 = time.perf_counter()
        ffmpeg.stdin.close()
        returncode = ffmpeg.wait(timeout=300)
        ffmpeg = None
        result["ffmpeg_returncode"] = returncode
        result["encode_wait_seconds"] = time.perf_counter() - t0
        if returncode != 0 or not film.is_file() or film.stat().st_size == 0:
            raise RuntimeError(f"ffmpeg failed with return code {returncode}, see {camera}_ffmpeg.log")
        result["film"] = film.name
        result["film_bytes"] = film.stat().st_size
        result["stills"] = extract_stills(film, out, camera, len(picture_seconds))
    except Exception:
        result["traceback"] = traceback.format_exc()
    finally:
        if ffmpeg is not None:
            try:
                ffmpeg.stdin.close()
                ffmpeg.wait(timeout=60)
            except Exception:
                ffmpeg.kill()
        try:
            if env is not None:
                env.close()
        except Exception:
            result["close_traceback"] = traceback.format_exc()
        (out / f"{camera}_positions.json").write_text(json.dumps(positions))
        (out / f"{camera}_child.json").write_text(json.dumps(result, indent=2))


# ---------------------------------------------------------------------------------------------
# Parent
# ---------------------------------------------------------------------------------------------


def run_one_film(camera: str, method: str, args, out: Path) -> dict:
    cmd = [
        sys.executable, str(Path(__file__).resolve()),
        "--child", camera, "--method", method, "--repo", str(Path(args.repo).resolve()),
        "--out", str(out), "--lsy-commit", args.lsy_commit,
    ]
    entry = {}
    (out / f"{camera}_child.json").unlink(missing_ok=True)  # no stale file from an earlier try
    try:
        proc = subprocess.run(
            cmd, env={**os.environ, "MUJOCO_GL": method}, capture_output=True, text=True,
            timeout=CHILD_TIMEOUT_S,
        )
        (out / f"{camera}_{method}_stdout.txt").write_text(proc.stdout)
        (out / f"{camera}_{method}_stderr.txt").write_text(proc.stderr)
        entry["returncode"] = proc.returncode
        entry["stderr_tail"] = proc.stderr[-3000:]
    except subprocess.TimeoutExpired:
        entry["process_error"] = f"child timed out after {CHILD_TIMEOUT_S} s"
    child_file = out / f"{camera}_child.json"
    if child_file.is_file():
        entry.update(json.loads(child_file.read_text()))
    else:
        entry.setdefault("process_error", "child wrote no child json (crashed hard)")
    return entry


def compare_flights(out: Path) -> dict:
    import math

    a = json.loads((out / "fpv_positions.json").read_text())
    b = json.loads((out / "outside_positions.json").read_text())
    n = min(len(a), len(b))
    diffs = [math.dist(a[i], b[i]) for i in range(n)]
    return {
        "positions_fpv_flight": len(a),
        "positions_outside_flight": len(b),
        "positions_compared": n,
        "max_position_difference_m": max(diffs) if diffs else None,
    }


def side_by_side(out: Path) -> dict:
    entry = {"film": "side_by_side.mp4", "ok": False}
    try:
        cmd = [
            "ffmpeg", "-y", "-loglevel", "error", "-i", str(out / "fpv.mp4"), "-i", str(out / "outside.mp4"),
            "-filter_complex", "hstack=inputs=2", "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out / "side_by_side.mp4"),
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        entry["ok"] = proc.returncode == 0 and (out / "side_by_side.mp4").is_file()
        if not entry["ok"]:
            entry["error"] = proc.stderr[-1500:]
    except Exception:
        entry["error"] = traceback.format_exc()
    return entry


def write_summary(result: dict, path: Path) -> None:
    verdict = "PASS" if result["passed"] else "FAIL"
    lines = [
        "### Flight video",
        "",
        f"**{verdict}**: {result['pass_reason']}",
        "",
        "| film | method | pictures | steps | sim seconds | gates passed | ended | flight wall s | s per picture (first / mean) | error |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for camera in CAMERAS:
        e = result["films"].get(camera, {})
        text = e.get("traceback") or e.get("process_error") or ""
        last = text.strip().splitlines()[-1][:100] if text.strip() else ""
        first = e.get("seconds_per_picture_first")
        mean = e.get("seconds_per_picture_mean_without_first")
        timing = f"{first:.2f} / {mean:.2f}" if first is not None and mean is not None else ""
        gates = f"{e['gates_passed']} of {e['gates_total']}" if "gates_passed" in e else ""
        lines.append(
            f"| {camera} | {e.get('method', '')} | {e.get('pictures', '')} | {e.get('steps', '')} | "
            f"{e.get('sim_seconds', '')} | {gates} | {e.get('end', '')} | "
            f"{e.get('flight_wall_seconds', 0):.1f} | {timing} | {last} |"
        )
    fpv = result["films"].get("fpv", {})
    if "gates_passed" in fpv:
        finished = "The track was finished." if fpv.get("end") == "finished" else "The track was not finished."
        lines += ["", f"Gates passed: {fpv['gates_passed']} of {fpv['gates_total']}. {finished}"]
    diff = result.get("flight_difference", {})
    lines += [
        "",
        f"Largest position difference between the two flights: {diff.get('max_position_difference_m')} m "
        f"over {diff.get('positions_compared')} steps.",
        f"Side-by-side film: {'made' if result.get('side_by_side', {}).get('ok') else 'not made'}.",
        "",
        "Versions: " + ", ".join(f"{k} {v}" for k, v in result["versions"].items()),
    ]
    path.write_text("\n".join(lines) + "\n")


def run_parent(args) -> None:
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    result = {"versions": {"python": platform.python_version()}, "films": {}, "attempts": []}
    methods = list(METHODS)
    for camera in CAMERAS:
        for method in list(methods):
            entry = run_one_film(camera, method, args, out)
            result["attempts"].append(
                {"camera": camera, "method": method, "film": entry.get("film"),
                 "error": (entry.get("traceback") or entry.get("process_error") or "").strip()[-500:]}
            )
            result["films"][camera] = entry
            if entry.get("film"):
                methods = [method] + [m for m in methods if m != method]  # keep what worked
                break
        result["versions"].update(result["films"][camera].get("versions", {}))
    try:
        result["flight_difference"] = compare_flights(out)
    except Exception:
        result["flight_difference"] = {"traceback": traceback.format_exc()}
    if all(result["films"][c].get("film") for c in CAMERAS):
        result["side_by_side"] = side_by_side(out)
    result["passed"], result["pass_reason"] = evaluate(result, out)
    (out / "result.json").write_text(json.dumps(result, indent=2))
    write_summary(result, out / "summary.md")
    print(("PASS: " if result["passed"] else "FAIL: ") + result["pass_reason"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", help="Folder for all output files")
    parser.add_argument("--repo", help="Checkout of lsy_drone_racing (for config and controller)")
    parser.add_argument("--lsy-commit", default="unknown", help="Commit of the checkout, only recorded")
    parser.add_argument("--child", choices=CAMERAS, help="Internal: film one camera")
    parser.add_argument("--method", choices=METHODS, help="Internal: drawing method of the child")
    parser.add_argument("--check", help="Apply the pass rule to this result.json and exit 1 on failure")
    args = parser.parse_args()
    if args.check:
        sys.exit(check(Path(args.check)))
    if not args.out or not args.repo:
        parser.error("--out and --repo are required")
    if args.child:
        run_child(args.child, args.method, Path(args.repo).resolve(), Path(args.out).resolve(), args.lsy_commit)
    else:
        run_parent(args)


if __name__ == "__main__":
    main()
