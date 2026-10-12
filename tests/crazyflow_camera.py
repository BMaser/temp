"""Does Crazyflow draw the picture of the drone camera with the LSY Drone Racing gates, with no screen?

Parent mode (default): starts one child process per drawing method (egl, osmesa), collects their
results into <out>/result.json and <out>/summary.md. It never imports MuJoCo itself, because
MUJOCO_GL has to be set before MuJoCo is imported.

Child mode (--child METHOD): builds the standard level-0 race environment, takes pictures, and
writes <out>/<METHOD>/child.json.

Check mode (--check FILE): applies the pass rule to a result file and exits 1 if the job fails.
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

METHODS = ("egl", "osmesa")
WIDTH, HEIGHT = 640, 480  # same as the fpv_cam resolution in the drone XML
N_STEPS = 40  # 0.8 s of simulated time at the 50 Hz of level0.toml
CHILD_TIMEOUT_S = 900
PASS_MIN_STD = 5.0


# ---------------------------------------------------------------------------------------------
# Pass rule (used by the parent and by --check)
# ---------------------------------------------------------------------------------------------


def evaluate(result: dict) -> tuple[bool, str]:
    """Pass if one drawing method gave an on-drone picture with 3 channels and std > 5."""
    good = []
    for method, entry in result.get("methods", {}).items():
        for img in entry.get("images", []):
            if not img.get("label", "").startswith("fpv"):
                continue
            shape, std = img.get("shape"), img.get("std")
            if shape and len(shape) == 3 and shape[-1] == 3 and std is not None and std > PASS_MIN_STD:
                good.append(f"{method}/{img['label']} (std {std:.1f})")
    if good:
        return True, "on-drone pictures with 3 channels and std > 5: " + ", ".join(good)
    return False, "no on-drone picture with 3 channels and std > 5 from any drawing method"


def check(result_file: Path) -> int:
    if not result_file.is_file():
        print(f"FAIL: result file {result_file} does not exist")
        return 1
    ok, reason = evaluate(json.loads(result_file.read_text()))
    print(("PASS: " if ok else "FAIL: ") + reason)
    return 0 if ok else 1


# ---------------------------------------------------------------------------------------------
# Child: one drawing method
# ---------------------------------------------------------------------------------------------


def image_stats(img) -> dict:
    import numpy as np

    arr = np.asarray(img)
    flat = arr.astype(np.float64)
    return {
        "shape": list(arr.shape),
        "dtype": str(arr.dtype),
        "min": float(flat.min()),
        "max": float(flat.max()),
        "mean": float(flat.mean()),
        "std": float(flat.std()),
    }


def run_child(method: str, repo: Path, out: Path, lsy_commit: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    result = {
        "method": method,
        "MUJOCO_GL": os.environ.get("MUJOCO_GL"),
        "versions": {},
        "images": [],
        "traceback": None,
    }
    env = None
    try:
        # crazyflow's own camera examples set this before importing scipy
        os.environ["SCIPY_ARRAY_API"] = "1"
        import gymnasium
        import imageio.v3 as iio
        import importlib.metadata as md
        import jax
        import mujoco
        import numpy as np
        from lsy_drone_racing.utils import load_config

        result["versions"] = {
            "mujoco": mujoco.__version__,
            "jax": jax.__version__,
            "crazyflow": md.version("crazyflow"),
            "lsy_drone_racing": md.version("lsy_drone_racing"),
            "lsy_drone_racing_commit": lsy_commit,
        }

        # Same environment as scripts/sim.py, with the on-screen window switched off.
        cfg = load_config(repo / "config" / "level0.toml")
        cfg.sim.render = False
        env = gymnasium.make(
            cfg.env.id,
            freq=cfg.env.freq,
            sim_config=cfg.sim,
            sensor_range=cfg.env.sensor_range,
            control_mode=cfg.env.control_mode,
            track=cfg.env.track,
            disturbances=cfg.env.get("disturbances"),
            randomizations=cfg.env.get("randomizations"),
            seed=0,
        )
        core = env.unwrapped
        obs, _ = env.reset()

        def take(label: str, **render_kwargs) -> None:
            entry = {"label": label, "file": f"{label}.png"}
            result["images"].append(entry)
            try:
                # Same sync as RaceCoreEnv.render: copy gate positions into the MuJoCo data.
                if not core.data.sim_data.core.mjx_synced:
                    core.data, core.sim.mjx_data = core._render_sync(core.data, core.sim.mjx_data)
                t0 = time.perf_counter()
                img = core.sim.render(mode="rgb_array", width=WIDTH, height=HEIGHT, **render_kwargs)
                entry["seconds"] = time.perf_counter() - t0
                if img is None:
                    entry["error"] = "sim.render returned None"
                    return
                img = np.ascontiguousarray(img)
                entry.update(image_stats(img))
                iio.imwrite(out / entry["file"], img)
            except Exception:
                entry["traceback"] = traceback.format_exc()

        take("fpv_after_reset", camera="fpv_cam:0")

        # Fly towards the first gate: state command [x y z, v(3), a(3), quat xyzw, w(3)].
        action = np.zeros(16, dtype=np.float32)
        action[:3] = np.asarray(obs["pos"]) + np.array([1.0, -0.25, 0.7])
        action[12] = 1.0
        steps_done = 0
        for _ in range(N_STEPS):
            obs, _, terminated, truncated, _ = env.step(action)
            steps_done += 1
            if terminated or truncated:
                break
        result["steps_done"] = steps_done
        result["drone_pos_after_steps"] = [float(v) for v in np.asarray(obs["pos"])]
        take("fpv_after_steps", camera="fpv_cam:0")

        # The viewer keeps the camera it was created with, so close it to switch to the free camera.
        core.sim.close()
        cam_cfg = {k: np.asarray(v) if k == "lookat" else v for k, v in cfg.sim.cam_config[0].items()}
        take("outside_after_steps", camera=-1, cam_config=cam_cfg)
    except Exception:
        result["traceback"] = traceback.format_exc()
    finally:
        try:
            if env is not None:
                env.close()
        except Exception:
            result["close_traceback"] = traceback.format_exc()
        (out / "child.json").write_text(json.dumps(result, indent=2))


# ---------------------------------------------------------------------------------------------
# Parent
# ---------------------------------------------------------------------------------------------


def write_summary(result: dict, path: Path) -> None:
    verdict = "PASS" if result["passed"] else "FAIL"
    lines = [
        "### Camera job",
        "",
        f"**{verdict}**: {result['pass_reason']}",
        "",
        "| method | picture | shape | std | mean | seconds | error |",
        "|---|---|---|---|---|---|---|",
    ]
    for method, entry in result["methods"].items():
        images = entry.get("images", [])
        if not images:
            text = entry.get("traceback") or entry.get("process_error") or "no picture"
            lines.append(f"| {method} | none | | | | | {text.strip().splitlines()[-1][:120]} |")
        for img in images:
            text = img.get("traceback") or img.get("error") or ""
            last = text.strip().splitlines()[-1][:120] if text.strip() else ""
            if "std" in img:
                lines.append(
                    f"| {method} | {img['label']} | {img['shape']} | {img['std']:.1f} | "
                    f"{img['mean']:.1f} | {img['seconds']:.2f} | {last} |"
                )
            else:
                lines.append(f"| {method} | {img['label']} | | | | | {last or 'failed'} |")
    versions = dict(result["versions"])
    for entry in result["methods"].values():
        versions.update(entry.get("versions", {}))
    lines += ["", "Versions: " + ", ".join(f"{k} {v}" for k, v in versions.items())]
    path.write_text("\n".join(lines) + "\n")


def run_parent(args) -> None:
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    result = {"python": platform.python_version(), "versions": {"python": platform.python_version()}, "methods": {}}
    for method in METHODS:
        method_dir = out / method
        method_dir.mkdir(exist_ok=True)
        env = {**os.environ, "MUJOCO_GL": method}
        cmd = [
            sys.executable, str(Path(__file__).resolve()),
            "--child", method, "--repo", str(Path(args.repo).resolve()),
            "--out", str(method_dir), "--lsy-commit", args.lsy_commit,
        ]
        entry = {}
        try:
            proc = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=CHILD_TIMEOUT_S)
            (method_dir / "stdout.txt").write_text(proc.stdout)
            (method_dir / "stderr.txt").write_text(proc.stderr)
            entry["returncode"] = proc.returncode
            entry["stderr_tail"] = proc.stderr[-3000:]
        except subprocess.TimeoutExpired:
            entry["process_error"] = f"child timed out after {CHILD_TIMEOUT_S} s"
        child_file = method_dir / "child.json"
        if child_file.is_file():
            entry.update(json.loads(child_file.read_text()))
        else:
            entry.setdefault("process_error", "child wrote no child.json (crashed hard, for example a segfault)")
        result["methods"][method] = entry
    result["passed"], result["pass_reason"] = evaluate(result)
    (out / "result.json").write_text(json.dumps(result, indent=2))
    write_summary(result, out / "summary.md")
    print(("PASS: " if result["passed"] else "FAIL: ") + result["pass_reason"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", help="Folder for all output files")
    parser.add_argument("--repo", help="Checkout of lsy_drone_racing (for config/level0.toml)")
    parser.add_argument("--lsy-commit", default="unknown", help="Commit of the checkout, only recorded")
    parser.add_argument("--child", choices=METHODS, help="Internal: run one drawing method")
    parser.add_argument("--check", help="Apply the pass rule to this result.json and exit 1 on failure")
    args = parser.parse_args()
    if args.check:
        sys.exit(check(Path(args.check)))
    if not args.out or not args.repo:
        parser.error("--out and --repo are required")
    if args.child:
        run_child(args.child, Path(args.repo).resolve(), Path(args.out).resolve(), args.lsy_commit)
    else:
        run_parent(args)


if __name__ == "__main__":
    main()
