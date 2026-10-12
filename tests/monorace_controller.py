"""Does the published MonoRace controller M16 fly in the authors' simulator QuadRace on the CPU?

Run mode (default): loads best_models/M16, flies a few episodes in two setups, writes
<out>/result.json, <out>/summary.md and <out>/paths_top_view.png. It always exits with 0.

Check mode (--check FILE): applies the pass rule to a result file and exits 1 if the job fails.

The two setups:
  simulate_py   the environment arguments of the repository's simulate.py (its default
                options.json, gate size 0.65), except for the number of parallel episodes.
  m16_options   the environment arguments and model parameters of best_models/M16/options.json,
                the setup M16 was trained in.
The flying loop follows get_ep_info() in quad_race/utils.py. That module is not imported, because
it pulls in the OpenCV animation code, which this test does not need.
"""

import argparse
import inspect
import json
import os
import platform
import sys
import time
import traceback
from pathlib import Path

N_EPISODES = 5
MAX_STEPS = 3500  # simulate.py
SEED = 1  # simulate.py
TWO_LAPS_GATES = 22  # get_ep_info() ends an episode after 22 gates (2 laps of 11 gates)


# ---------------------------------------------------------------------------------------------
# Pass rule (used by run mode and by --check)
# ---------------------------------------------------------------------------------------------


def evaluate(result: dict) -> tuple[bool, str]:
    """Pass if the model loaded, one episode ran to its end, and one gate was passed somewhere."""
    if not result.get("model_loaded"):
        return False, "the controller file did not load"
    episodes = [e for s in result.get("setups", {}).values() for e in s.get("episodes", [])]
    finished = [e for e in episodes if e.get("end") in ("crash", "time", "two_laps")]
    if not finished:
        return False, "no episode ran to its end without an exception"
    if not any(e["gates_passed"] >= 1 for e in finished):
        return False, "episodes ran, but no gate was passed in any of them"
    best = max(e["gates_passed"] for e in finished)
    return True, f"model loaded, {len(finished)} episodes ended, most gates passed in one episode: {best}"


def check(result_file: Path) -> int:
    if not result_file.is_file():
        print(f"FAIL: result file {result_file} does not exist")
        return 1
    ok, reason = evaluate(json.loads(result_file.read_text()))
    print(("PASS: " if ok else "FAIL: ") + reason)
    return 0 if ok else 1


# ---------------------------------------------------------------------------------------------
# Flying
# ---------------------------------------------------------------------------------------------


def allow_old_jit_arguments(jax) -> bool:
    """quad_race/environment.py calls jax.jit(..., device=...). Newer JAX may have dropped it.

    Returns True if a wrapper that ignores `device` and `backend` had to be installed. On this
    CPU-only machine ignoring them changes nothing.
    """
    if "device" in inspect.signature(jax.jit).parameters:
        return False
    original = jax.jit

    def jit(*args, device=None, backend=None, **kwargs):
        return original(*args, **kwargs)

    jax.jit = jit
    return True


def fly(env, model, np) -> list[dict]:
    """Roll out env.num_envs episodes in parallel; the loop follows get_ep_info()."""
    env.reset_seed()
    env.reset()
    env.pause_if_collision = True
    n = env.num_envs
    done = np.zeros(n, dtype=bool)
    reward_sum = np.zeros(n)
    steps = np.zeros(n, dtype=int)
    gates = np.zeros(n, dtype=int)
    end = [None] * n
    paths = [[] for _ in range(n)]

    for _ in range(MAX_STEPS + 10):
        if done.all():
            break
        actions, _ = model.predict(env.states, deterministic=True)
        target_before = env.target_gates.copy()
        _, rewards, dones, _ = env.step(actions)
        active = ~done
        reward_sum[active] += rewards[active]
        steps[active] += 1
        gates[active & (target_before != env.target_gates)] += 1
        for i in np.flatnonzero(active):
            paths[i].append(env.world_states[i, 0:3].copy())
        finished_laps = gates >= TWO_LAPS_GATES
        newly_done = active & (dones | finished_laps)
        for i in np.flatnonzero(newly_done):
            if finished_laps[i]:
                end[i] = "two_laps"
            elif steps[i] >= env.max_steps:
                end[i] = "time"
            else:
                end[i] = "crash"
        done |= newly_done

    episodes = []
    for i in range(n):
        episodes.append(
            {
                "episode": i,
                "steps": int(steps[i]),
                "sim_seconds": float(steps[i] * env.dt),
                "gates_passed": int(gates[i]),
                "end": end[i] or "not_ended",
                "reward_sum": float(reward_sum[i]),
                "path_xyz_every_5th_step": np.asarray(paths[i])[::5].tolist(),
            }
        )
    return episodes


def m16_env_kwargs(qe, np, options: dict) -> dict:
    """Constructor arguments of QuadRace from the EnvironmentConfig of the M16 options.json."""
    c = options["EnvironmentConfig"]
    keys = [
        "speed_limit", "gate_size", "num_envs", "pause_if_collision", "pos_noise", "vel_noise",
        "pos_jumps_std", "vel_offset_std", "initialize_uniform", "initialize_on_ground",
        "initialize_at_random_gates", "loop_gates", "progress_reward", "gate_reward",
        "angular_rate_penalty", "gate_offset_penalty", "perception_penalty", "motor_penalty",
        "motor_penalty_threshold", "low_action_penalty", "crash_penalty", "ground_height",
        "v_ground", "gate_thickness", "disable_gate_collision",
    ]
    kwargs = {k: c[k] for k in keys}
    kwargs["cam_angle"] = c["cam_angle_degrees"] * np.pi / 180
    kwargs["flightplan"] = qe.Flightplan(c["flight_plan"])
    kwargs["num_envs"] = N_EPISODES
    kwargs["max_steps"] = MAX_STEPS
    kwargs["seed"] = SEED
    return kwargs


def plot_paths(result: dict, gates: dict, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    setups = [(name, s) for name, s in result["setups"].items() if s.get("episodes")]
    if not setups:
        return
    fig, axes = plt.subplots(len(setups), 1, figsize=(12, 4.5 * len(setups)), squeeze=False)
    gate_pos, gate_yaw, gate_size = np.array(gates["pos"]), np.array(gates["yaw"]), gates["size"]
    for ax, (name, setup) in zip(axes[:, 0], setups):
        for ep in setup["episodes"]:
            xyz = np.array(ep["path_xyz_every_5th_step"])
            if len(xyz):
                ax.plot(xyz[:, 0], xyz[:, 1], lw=1, label=f"ep {ep['episode']}: {ep['gates_passed']} gates, {ep['end']}")
        for k, (p, yaw) in enumerate(zip(gate_pos, gate_yaw)):
            half = np.array([-np.sin(yaw), np.cos(yaw)]) * gate_size / 2
            ax.plot([p[0] - half[0], p[0] + half[0]], [p[1] - half[1], p[1] + half[1]], "k-", lw=3)
            ax.annotate(str(k), p[:2], textcoords="offset points", xytext=(0, 6), ha="center")
        ax.set_title(f"setup {name}: top view (x, y in metres), black bars are gates")
        ax.set_aspect("equal")
        ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def write_summary(result: dict, path: Path) -> None:
    verdict = "PASS" if result["passed"] else "FAIL"
    lines = [
        "### Controller job",
        "",
        f"**{verdict}**: {result['pass_reason']}",
        "",
        "| setup | episode | steps | sim seconds | gates passed | end | reward sum |",
        "|---|---|---|---|---|---|---|",
    ]
    for name, setup in result.get("setups", {}).items():
        for e in setup.get("episodes", []):
            lines.append(
                f"| {name} | {e['episode']} | {e['steps']} | {e['sim_seconds']:.2f} | "
                f"{e['gates_passed']} | {e['end']} | {e['reward_sum']:.1f} |"
            )
        if setup.get("traceback"):
            lines.append(f"| {name} | failed | | | | {setup['traceback'].strip().splitlines()[-1][:100]} | |")
    if result.get("traceback"):
        lines += ["", "Error: " + result["traceback"].strip().splitlines()[-1]]
    lines += ["", "Versions: " + ", ".join(f"{k} {v}" for k, v in result["versions"].items())]
    path.write_text("\n".join(lines) + "\n")


def run(args) -> None:
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    repo = Path(args.repo).resolve()
    result = {
        "versions": {"python": platform.python_version()},
        "model_loaded": False,
        "setups": {},
        "traceback": None,
    }
    gates = None
    try:
        # quad_race/environment.py reads options.json and the flight plan relative to the
        # current folder when it is imported, so the repository root must be the current folder.
        sys.dont_write_bytecode = True
        os.chdir(repo)
        sys.path.insert(0, str(repo))
        import gymnasium
        import jax
        import numpy as np
        import stable_baselines3
        import sympy
        import torch
        from stable_baselines3 import PPO

        result["versions"].update(
            torch=torch.__version__, stable_baselines3=stable_baselines3.__version__,
            numpy=np.__version__, gymnasium=gymnasium.__version__, jax=jax.__version__,
            sympy=sympy.__version__,
        )
        result["jax_jit_device_shim_installed"] = allow_old_jit_arguments(jax)

        import quad_race.environment as qe

        model_files = sorted((repo / "best_models" / "M16").glob("*.zip"))
        result["model_files"] = [str(p.relative_to(repo)) for p in model_files]
        model = PPO.load(str(model_files[0]), device="cpu")
        result["model_loaded"] = True

        m16_options_file = repo / "best_models" / "M16" / "options.json"
        m16_options = json.loads(m16_options_file.read_text())
        setups = {
            "simulate_py": dict(
                num_envs=N_EPISODES, initialize_on_ground=True, initialize_uniform=False,
                initialize_at_random_gates=False, pause_if_collision=True, loop_gates=True,
                gate_size=0.65, pos_jumps_std=0.0, pos_jumps_prob=0.0, max_steps=MAX_STEPS,
                seed=SEED,
            ),
            "m16_options": m16_env_kwargs(qe, np, m16_options),
        }
        for name, kwargs in setups.items():
            entry = result["setups"][name] = {}
            try:
                if name == "m16_options":
                    qe.load_config(str(m16_options_file))  # model parameters and their spread
                t0 = time.perf_counter()
                env = qe.QuadRace(**kwargs)
                entry["episodes"] = fly(env, model, np)
                entry["wall_seconds"] = time.perf_counter() - t0
                gates = gates or {
                    "pos": env.gate_pos.tolist(), "yaw": env.gate_yaw.tolist(),
                    "size": float(env.gate_size[0]),
                }
            except Exception:
                entry["traceback"] = traceback.format_exc()
    except Exception:
        result["traceback"] = traceback.format_exc()

    if gates:
        try:
            plot_paths(result, gates, out / "paths_top_view.png")
        except Exception:
            result["plot_traceback"] = traceback.format_exc()
    result["passed"], result["pass_reason"] = evaluate(result)
    (out / "result.json").write_text(json.dumps(result, indent=2))
    write_summary(result, out / "summary.md")
    print(("PASS: " if result["passed"] else "FAIL: ") + result["pass_reason"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", help="Folder for all output files")
    parser.add_argument("--repo", help="Checkout of tudelft/optimal_quad_control_RL (branch a2rl2025)")
    parser.add_argument("--check", help="Apply the pass rule to this result.json and exit 1 on failure")
    args = parser.parse_args()
    if args.check:
        sys.exit(check(Path(args.check)))
    if not args.out or not args.repo:
        parser.error("--out and --repo are required")
    run(args)


if __name__ == "__main__":
    main()
