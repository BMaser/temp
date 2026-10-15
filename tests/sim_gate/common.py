"""Shared helpers of the simulator gate (tests/sim_gate).

Every simulator script writes results/<sim>/gate.json with the same fields. This module holds the field list,
the picture check (pixel standard deviation above 5), picture saving, log excerpts, and the "finalize" step that
runs at the end of every job (also after a failure) so that every job leaves a gate.json behind.

Usage as a program:  python3 tests/sim_gate/common.py finalize <sim>
"""

import json
import os
import re
import sys
import time
from pathlib import Path

PASS_MIN_STD = 5.0

FIELDS = [
    "sim", "version_or_commit", "install_ok", "install_minutes", "headless_ok", "camera_ok", "camera_size",
    "physics_steps_per_s", "camera_frames_per_s", "real_time_factor", "world_used", "attempts", "notes", "log_excerpt",
]

ERROR_PATTERN = re.compile(
    r"(error|fatal|failed|cannot|can't|not found|no such|traceback|exception|abort|segfault|segmentation|"
    r"unable|denied|timed out|timeout|vk_error|vulkan|lavapipe|llvmpipe|opengl|glx|egl)", re.I)


def results_dir(sim: str) -> Path:
    p = Path(os.environ.get("GATE_RESULTS", "results")) / sim
    p.mkdir(parents=True, exist_ok=True)
    return p


def new_gate(sim: str) -> dict:
    g = {k: None for k in FIELDS}
    g["sim"] = sim
    g["attempts"] = int(os.environ.get("GATE_ATTEMPT", "1") or 1)
    g["notes"] = ""
    g["log_excerpt"] = []
    return g


def read_gate(sim: str) -> dict:
    f = results_dir(sim) / "gate.json"
    if f.is_file():
        g = json.loads(f.read_text())
    else:
        g = new_gate(sim)
    for k in FIELDS:
        g.setdefault(k, None)
    return g


def write_gate(g: dict) -> None:
    (results_dir(g["sim"]) / "gate.json").write_text(json.dumps(g, indent=2, default=str))


def add_note(g: dict, text: str) -> None:
    g["notes"] = (g.get("notes") or "") + ("" if not g.get("notes") else " ") + text


def image_stats(img) -> dict:
    import numpy as np

    a = np.asarray(img)
    f = a.astype(np.float64)
    return {"shape": list(a.shape), "dtype": str(a.dtype), "min": float(f.min()), "max": float(f.max()),
            "mean": float(f.mean()), "std": float(f.std())}


def picture_ok(img) -> bool:
    """A non-blank RGB picture: 3 channels and a pixel standard deviation above 5."""
    s = image_stats(img)
    return len(s["shape"]) == 3 and s["shape"][-1] == 3 and s["std"] > PASS_MIN_STD


def save_png(img, path) -> None:
    import numpy as np

    a = np.ascontiguousarray(np.asarray(img))
    try:
        from PIL import Image

        Image.fromarray(a).save(str(path))
    except ImportError:
        import imageio.v3 as iio

        iio.imwrite(str(path), a)


def excerpt(path, n=25, pattern=ERROR_PATTERN, tail_if_none=15) -> list:
    """The decisive lines of a log: the last n lines that look like errors, else the last few lines."""
    p = Path(path)
    if not p.is_file():
        return [f"(no log file {p})"]
    lines = p.read_text(errors="replace").splitlines()
    hits = [ln.strip()[:300] for ln in lines if pattern.search(ln)]
    if hits:
        return hits[-n:]
    return [ln.strip()[:300] for ln in lines[-tail_if_none:]]


def timer():
    t0 = time.perf_counter()
    return lambda: time.perf_counter() - t0


def finalize(sim: str) -> None:
    d = results_dir(sim)
    g = read_gate(sim)
    g["sim"] = sim
    g["attempts"] = int(os.environ.get("GATE_ATTEMPT", "1") or 1)
    secs = d / "install_seconds.txt"
    if secs.is_file() and g.get("install_minutes") is None:
        try:
            g["install_minutes"] = round(float(secs.read_text().strip()) / 60.0, 1)
        except ValueError:
            pass
    ok = d / "install_ok.txt"
    if ok.is_file():
        g["install_ok"] = ok.read_text().strip() == "1"
    elif g.get("install_ok") is None:
        g["install_ok"] = False
    ver = d / "version.txt"
    if g.get("version_or_commit") is None and ver.is_file():
        g["version_or_commit"] = ver.read_text().strip()
    gate_written = (d / "gate.json").is_file()
    if not gate_written:
        add_note(g, "The gate script wrote no gate.json (it did not start or crashed before it could write); "
                    "all measurement fields are null.")
    if not g.get("camera_ok"):
        if not g.get("log_excerpt"):
            g["log_excerpt"] = excerpt(d / "log.txt")
    write_gate(g)
    print(json.dumps(g, indent=2, default=str))


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "finalize":
        finalize(sys.argv[2])
    else:
        print(__doc__)
        sys.exit(2)
