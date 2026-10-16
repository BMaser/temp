"""Write ONE result file (flight_result.json, format version 1) from the saved states of a simulator flight.

    python tools/flight_result.py --npz forward_4.npz --child forward_4_child.json --layout track_today.json \
        --out flight_result_forward_4.json [--films fpv.mp4 chase.mp4 ...] [--run-id ID] [--seed 0] [--date YYYY-MM-DD]

numpy and the standard library only. The format is described in docs/FLIGHT_RESULT_FORMAT.md.
A bad input stops the tool with one clear message on stderr and exit code 1; nothing is written then.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

FORMAT = "flight_result"
VERSION = 1
POLE_RADIUS_M = 0.015  # pole diameter 0.03 m (the layout note and the GUI, Track Designer.dc.html line 591)
POLE_HEIGHT_MARGIN_M = 0.1  # the GUI counts a pole only while the drone is lower than its top + 0.1 m (same line)
GUI_SAFE = {"x": (-2.5, 2.5), "y": (-1.5, 1.5), "zmax": 2.0}  # td-core.js line 7
GUI_WARN_BOX = "The path leaves the safe area of the real room."  # Track Designer.dc.html line 592
GUI_WARN_HIGH = "The path rises above the 2 m safe height."
GUI_WARN_POLE = "Pole clearance drops below 0.25 m."
FORBIDDEN_NAMES = re.compile(r"\b(lsy|utias|tum|tu munich|technical university|university|universit\w+|epfl|eth zurich|bitcraze)\b", re.I)
# digits after the decimal point, per quantity (stated in the format document)
DIGITS = {"time": 3, "pos": 4, "vel": 4, "speed": 4, "deg": 3, "quat": 6, "rad": 5, "dist": 4}


class InputError(Exception):
    """A bad input file; the message is shown to the user as it is."""


# ---------------------------------------------------------------- attitude

def quat_to_attitude_deg(q) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Roll, pitch and yaw in degrees from quaternions (n, 4) in the order x y z w (Hamilton, body to world).

    Convention = the one of the GUI (Track Designer.dc.html lines 572-576): yaw is counterclockwise seen from above,
    zero along +x; pitch is positive with the nose UP; roll is positive when the drone is banked to the LEFT
    (the thrust leans toward the left of the nose, as in a left turn).
    """
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch_about_y = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))  # right-handed about +y: positive = nose DOWN
    roll_about_x = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))  # right-handed about +x: positive = right bank
    return np.degrees(-roll_about_x), np.degrees(-pitch_about_y), np.degrees(yaw)


def tilt_deg(q) -> np.ndarray:
    """Angle between the body up axis and the world up axis, in degrees (0 = level)."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 0] ** 2 + q[:, 1] ** 2), -1.0, 1.0)))


def yaw_of_gate_quat(q) -> float:
    x, y, z, w = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


# ---------------------------------------------------------------- events and measures

def find_passage_rows(passed) -> list[tuple[int, int]]:
    """(row, passage number) for every row where the count of passages goes up; a jump of 2 gives two entries."""
    out = []
    p = np.asarray(passed)
    for i in range(1, len(p)):
        for n in range(int(p[i - 1]) + 1, int(p[i]) + 1):
            out.append((i, n))
    return out


def min_pole_clearance(pos, time, poles, radius=POLE_RADIUS_M, margin=POLE_HEIGHT_MARGIN_M):
    """Smallest horizontal distance from the flown line to a pole surface.

    The flown line is the polyline through the saved positions (a straight piece between two rows). A pole is a
    vertical cylinder of the given radius from the floor to its top (poles[k][2]). A piece counts only when both its
    rows are lower than the pole top + margin (the rule of the GUI). The distance is measured in the x-y plane,
    from the pole axis to the nearest point of the piece, minus the radius. The drone body is NOT subtracted.
    Returns (clearance_m, time_s, pole_number_from_1, axis_distance_m) or None when no piece counts.
    """
    pos = np.asarray(pos, dtype=float)
    time = np.asarray(time, dtype=float)
    best = None
    a, b = pos[:-1], pos[1:]
    ab = (b - a)[:, :2]
    ab2 = np.einsum("ij,ij->i", ab, ab)
    for k, pole in enumerate(poles):
        top = float(pole[2])
        ok = (a[:, 2] < top + margin) & (b[:, 2] < top + margin)
        if not ok.any():
            continue
        ap = np.array([pole[0], pole[1]]) - a[:, :2]
        u = np.where(ab2 > 1e-18, np.einsum("ij,ij->i", ap, ab) / np.where(ab2 > 1e-18, ab2, 1.0), 0.0)
        u = np.clip(u, 0.0, 1.0)
        d = np.linalg.norm(ap - u[:, None] * ab, axis=1)
        d = np.where(ok, d, np.inf)
        i = int(np.argmin(d))
        if best is None or d[i] < best[3]:
            best = (float(d[i]) - radius, float(time[i] + u[i] * (time[i + 1] - time[i])), k + 1, float(d[i]))
    return best


def end_event(child_end: str, time, pos, vel, freq, dropped, limits):
    """The end of the flight as (reason, time_s). A crash drops the step that ended the episode (the code that saved
    the states does not record it), so its time is the last saved time plus the dropped steps."""
    t_last = float(time[-1])
    if child_end == "finished":
        return "finished", t_last
    if child_end == "time":
        return "time_over", t_last
    t_end = t_last + dropped / freq
    nxt = np.asarray(pos[-1], dtype=float) + np.asarray(vel[-1], dtype=float) / freq
    lo, hi = limits
    outside = bool(np.any(nxt[:2] < np.asarray(lo)[:2]) or np.any(nxt[:2] > np.asarray(hi)[:2]) or nxt[2] > hi[2])
    return ("left_box" if outside else "crash"), t_end


# ---------------------------------------------------------------- input checks

def _need(cond: bool, msg: str):
    if not cond:
        raise InputError(msg)


def load_inputs(npz_path: Path, child_path: Path, layout_path: Path):
    for p in (npz_path, child_path, layout_path):
        _need(p.is_file(), f"input file not found: {p}")
    try:
        z = np.load(npz_path, allow_pickle=False)
    except Exception as e:  # noqa: BLE001
        raise InputError(f"{npz_path} is not a readable .npz file: {e}") from None
    keys = ("time", "pos", "vel", "quat", "passed", "gates_pos", "gates_quat", "gate_sequence", "gate_direction",
            "obstacles_pos", "pad_start", "pad_end", "freq")
    missing = [k for k in keys if k not in z.files]
    _need(not missing, f"{npz_path} lacks the arrays: {', '.join(missing)}")
    a = {k: np.asarray(z[k]) for k in keys}
    try:
        child = json.loads(child_path.read_text(encoding="utf-8"))
        layout = json.loads(layout_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise InputError(f"a json input is not valid json: {e}") from None
    n = len(a["time"])
    _need(n >= 2, f"{npz_path}: fewer than 2 rows")
    for k, cols in (("pos", 3), ("vel", 3), ("quat", 4)):
        _need(a[k].shape == (n, cols), f"{npz_path}: '{k}' has shape {a[k].shape}, expected ({n}, {cols})")
    _need(a["passed"].shape == (n,), f"{npz_path}: 'passed' has shape {a['passed'].shape}, expected ({n},)")
    for k in ("time", "pos", "vel", "quat"):
        _need(bool(np.all(np.isfinite(a[k]))), f"{npz_path}: '{k}' holds a value that is not a number")
    freq = float(a["freq"])
    _need(freq > 0, f"{npz_path}: freq is {freq}")
    _need(bool(np.allclose(np.diff(a["time"]), 1.0 / freq, atol=1e-6)), f"{npz_path}: time is not a steady 1/{freq:g} s grid")
    qn = np.linalg.norm(a["quat"], axis=1)
    _need(bool(np.all(np.abs(qn - 1) < 1e-3)), f"{npz_path}: a quaternion does not have length 1 (worst {np.abs(qn - 1).max():.2e})")
    _need(bool(np.all(np.diff(a["passed"]) >= 0)), f"{npz_path}: 'passed' goes down")
    ng = len(a["gates_pos"])
    _need(a["gates_pos"].shape == (ng, 3) and a["gates_quat"].shape == (ng, 4), f"{npz_path}: gate arrays have the wrong shape")
    _need(len(a["gate_sequence"]) == len(a["gate_direction"]) and len(a["gate_sequence"]) >= 1, f"{npz_path}: gate_sequence and gate_direction differ in length")
    _need(int(a["gate_sequence"].min()) >= 0 and int(a["gate_sequence"].max()) < ng, f"{npz_path}: gate_sequence names a gate that does not exist")
    _need(int(a["passed"][-1]) <= len(a["gate_sequence"]), f"{npz_path}: more passages than entries in gate_sequence")
    # child
    for k in ("tag", "end", "rows_saved", "passages", "gates_total", "terminated_steps_dropped", "versions"):
        _need(k in child, f"{child_path} lacks '{k}'")
    _need(child["end"] in ("finished", "crash", "time"), f"{child_path}: end is '{child['end']}', expected finished, crash or time")
    _need(int(child["rows_saved"]) == n, f"{child_path} says {child['rows_saved']} rows, the npz has {n}")
    _need(int(child["passages"]) == int(a["passed"][-1]), f"{child_path} says {child['passages']} passages, the npz ends at {int(a['passed'][-1])}")
    _need(int(child["gates_total"]) == len(a["gate_sequence"]), f"{child_path}: gates_total differs from the npz gate_sequence")
    if child["end"] == "finished":
        _need(int(a["passed"][-1]) == len(a["gate_sequence"]), f"{child_path} says finished but the npz did not pass every gate")
    # layout against the flight
    for k in ("gates", "obstacles", "start"):
        _need(k in layout, f"{layout_path} lacks '{k}'")
    _need(len(layout["gates"]) == ng, f"{layout_path} has {len(layout['gates'])} gates, the flight has {ng}")
    for i, g in enumerate(layout["gates"]):
        _need(np.allclose(g["pos"], a["gates_pos"][i], atol=1e-4), f"layout gate {i + 1} position differs from the flight's")
        dyaw = (g["yaw_rad"] - yaw_of_gate_quat(a["gates_quat"][i]) + math.pi) % (2 * math.pi) - math.pi
        _need(abs(dyaw) < 0.01, f"layout gate {i + 1} direction differs from the flight's by {dyaw:.3f} rad")
    no = len(a["obstacles_pos"])
    _need(len(layout["obstacles"]) == no, f"{layout_path} has {len(layout['obstacles'])} poles, the flight has {no}")
    for i, o in enumerate(layout["obstacles"]):
        _need(np.allclose(o["pos"], a["obstacles_pos"][i], atol=1e-4), f"layout pole {i + 1} position differs from the flight's")
    _need(np.allclose(layout["start"]["pos"][:2], a["pad_start"], atol=1e-4), "layout start differs from the flight's start pad")
    return a, child, layout


# ---------------------------------------------------------------- number formatting

def _r(x, nd):
    """Round a float or an array to nd digits and turn -0.0 into 0.0."""
    v = np.round(np.asarray(x, dtype=float), nd) + 0.0
    return v.tolist() if v.ndim else float(v)


def render(x, level=0) -> str:
    """JSON text: containers of plain numbers and short flat objects stay on one line, the rest is indented."""
    pad = " " * level
    if isinstance(x, (list, tuple)):
        if all(not isinstance(v, (list, tuple, dict)) for v in x):
            return json.dumps(list(x), separators=(",", ":"))
        inner = ",\n".join(" " * (level + 1) + render(v, level + 1) for v in x)
        return "[\n" + inner + "\n" + pad + "]"
    if isinstance(x, dict):
        flat = all(not isinstance(v, dict) and (not isinstance(v, (list, tuple)) or all(not isinstance(u, (list, tuple, dict)) for u in v))
                   for v in x.values())
        one = json.dumps(x, separators=(",", ":"), ensure_ascii=False) if flat else ""
        if flat and len(one) <= 160:
            return one
        inner = ",\n".join(" " * (level + 1) + json.dumps(k) + ": " + render(v, level + 1) for k, v in x.items())
        return "{\n" + inner + "\n" + pad + "}"
    return json.dumps(x, ensure_ascii=False)


# ---------------------------------------------------------------- build

def build(a, child, layout, *, films=(), run_id=None, seed=None, date=None, title="Track of the flight", date_source=None):
    time, pos, vel, quat = a["time"], a["pos"], a["vel"], a["quat"]
    passed = a["passed"].astype(int)
    n = len(time)
    freq = float(a["freq"])
    seq = [int(v) for v in a["gate_sequence"]]
    dirs = [int(v) for v in a["gate_direction"]]
    total = len(seq)
    speed = np.linalg.norm(vel, axis=1)
    roll, pitch, yaw = quat_to_attitude_deg(quat)
    tilt = tilt_deg(quat)

    # layout in the GUI's saved format (td-core.js toFile, lines 211-235)
    route = [{"type": "gate", "gate": g + 1, "reverse": d < 0} for g, d in zip(seq, dirs)]
    gate_order = [-(g + 1) if d < 0 else g + 1 for g, d in zip(seq, dirs)]
    end_pos = [float(a["pad_end"][0]), float(a["pad_end"][1]), 0.0]
    lay = {"title": title}
    for k in ("units", "limits_box", "level_file_safety_limits", "gate_outer_m", "gate_opening_m", "obstacle_pos_z_means"):
        if k in layout:
            lay[k] = layout[k]
    lay.update({
        "gate_order": gate_order,
        "gate_order_note": "only the gate passages, in order; gate numbers start at 1; a negative number means the same gate crossed in the opposite direction",
        "route": route,
        "route_note": "the full route in order: gates (reverse = crossed the other way) and free route points the line must pass through",
        "designer_version": "flight_result 1",
        "start": {"pos": [float(v) for v in layout["start"]["pos"]], "yaw_rad": float(layout["start"]["yaw_rad"])},
        "gates": [{"pos": [float(v) for v in g["pos"]], "yaw_rad": float(g["yaw_rad"]), "shape": g.get("shape", "square")} for g in layout["gates"]],
        "obstacles": [{"pos": [float(v) for v in o["pos"]]} for o in layout["obstacles"]],
        "route_points": [],
        "end": {"pos": end_pos},
    })
    limits = layout.get("level_file_safety_limits")
    if limits:
        lo, hi = limits["pos_limit_low"], limits["pos_limit_high"]
    else:
        lo, hi = [GUI_SAFE["x"][0], GUI_SAFE["y"][0], -0.001], [GUI_SAFE["x"][1], GUI_SAFE["y"][1], GUI_SAFE["zmax"]]

    # events
    events = []
    gate_pos_list = layout["gates"]
    for row, number in find_passage_rows(passed):
        gi = seq[number - 1]
        g = gate_pos_list[gi]
        normal = np.array([math.cos(g["yaw_rad"]), math.sin(g["yaw_rad"])])
        crossing = float(np.dot(pos[row, :2] - pos[row - 1, :2], normal))
        measured = 1 if crossing > 0 else -1
        events.append({"type": "gate_passage", "time_s": _r(time[row], DIGITS["time"]), "row": row, "passage": number, "gate": gi + 1,
                       "direction": "reverse" if measured < 0 else "forward", "planned_direction": "reverse" if dirs[number - 1] < 0 else "forward"})
    reason, t_end = end_event(child["end"], time, pos, vel, freq, int(child["terminated_steps_dropped"]), (lo, hi))
    events.append({"type": "end", "reason": reason, "time_s": _r(t_end, DIGITS["time"]), "row": n - 1})

    # summary
    passage_t = [e["time_s"] for e in events if e["type"] == "gate_passage"]
    splits = [round(t - s, 3) for t, s in zip(passage_t, [0.0] + passage_t[:-1])]
    poles = [o["pos"] for o in layout["obstacles"]]
    pc = min_pole_clearance(pos, time, poles)
    step_len = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    length = float(step_len.sum())
    duration = float(time[-1] - time[0])
    outside_box = bool(np.any((pos[:, 0] < GUI_SAFE["x"][0]) | (pos[:, 0] > GUI_SAFE["x"][1]) | (pos[:, 1] < GUI_SAFE["y"][0]) | (pos[:, 1] > GUI_SAFE["y"][1])))
    too_high = bool(np.any(pos[:, 2] > GUI_SAFE["zmax"]))
    warnings = []
    if outside_box:
        warnings.append(GUI_WARN_BOX)
    if too_high:
        warnings.append(GUI_WARN_HIGH)
    if pc is not None and pc[0] < 0.25:
        warnings.append(GUI_WARN_POLE)
    finished = reason == "finished"
    summary = {
        "lap_time_s": _r(time[-1], DIGITS["time"]) if finished else None,
        "flight_time_s": _r(duration, DIGITS["time"]),
        "passages": int(passed[-1]), "passages_total": total,
        "passage_times_s": passage_t, "split_times_s": splits,
        "top_speed_mps": _r(speed.max(), DIGITS["speed"]), "top_speed_time_s": _r(time[int(np.argmax(speed))], DIGITS["time"]),
        "mean_speed_mps": _r(length / duration, DIGITS["speed"]), "path_length_m": _r(length, DIGITS["pos"]),
        "max_tilt_deg": _r(tilt.max(), DIGITS["deg"]), "max_tilt_time_s": _r(time[int(np.argmax(tilt))], DIGITS["time"]),
        "min_pole_clearance": None if pc is None else {"clearance_m": _r(pc[0], DIGITS["pos"]), "axis_distance_m": _r(pc[3], DIGITS["pos"]),
                                                       "time_s": _r(pc[1], DIGITS["time"]), "pole": pc[2]},
        "max_height_m": _r(pos[:, 2].max(), DIGITS["pos"]),
        "warnings": warnings,
    }

    # gui: the path (Track Designer.dc.html buildPath, lines 555-566, plus the attitude of simState, lines 572-576)
    dist = np.concatenate([[0.0], np.cumsum(step_len)])
    gui_gates = [{"gate": e["gate"], "passage": e["passage"], "idx": e["row"], "t": e["time_s"]} for e in events if e["type"] == "gate_passage"]
    path = {
        "S": [{"x": x, "y": y, "z": z} for x, y, z in zip(_r(pos[:, 0], 4), _r(pos[:, 1], 4), _r(pos[:, 2], 4))],
        "dist": _r(dist, DIGITS["dist"]), "v": _r(speed, DIGITS["speed"]), "time": _r(time, DIGITS["time"]),
        "yaw": _r(np.radians(yaw), DIGITS["rad"]), "pitch": _r(np.radians(pitch), DIGITS["rad"]), "roll": _r(np.radians(roll), DIGITS["rad"]),
        "gates": gui_gates, "total": _r(time[-1], DIGITS["time"]), "length": _r(length, DIGITS["dist"]),
    }
    # gui: the run object (seedRuns lines 414-420 and tickSim line 596). profile = 61 points at equal TIME steps
    prof = []
    for i in range(61):
        t = time[-1] * i / 60
        prof.append({"d": _r(np.interp(t, time, dist), DIGITS["dist"]), "z": _r(np.interp(t, time, pos[:, 2]), DIGITS["pos"]),
                     "v": _r(np.interp(t, time, speed), DIGITS["speed"])})
    tag = str(child["tag"])
    run = {
        "id": "flight_" + (run_id + "_" if run_id else "") + tag, "name": "Simulator " + tag,
        "date": date or "", "lap": _r(time[-1], DIGITS["time"]), "vmax": _r(speed.max(), DIGITS["speed"]),
        "minClear": None if pc is None else _r(pc[0], DIGITS["pos"]), "ok": bool(finished and not warnings),
        "splits": passage_t, "profile": prof, "track": title,
    }
    unfilled = [
        "path.S[].seg: not used by the page; left out.",
        "path.gates[].idx is the first row where the simulator counts the passage (up to one step, about 0.1 m, after the gate plane); the page picks the nearest sample to the gate center instead.",
        "run.date: the simulator does not save a flight date; it is the --date given to the tool or the modification date of the npz file (source in source.date_source).",
        "run.ok: the page sets it from the three warnings only; here it also needs the flight to have finished.",
        "run.minClear: the page measures it at the played frames; here it is measured along the flown line between the saved rows.",
        "sim.trail, sim.warnings and the play state are made by the page while it plays.",
        "Pole clearance ignores the body of the drone (about 0.05 m half width); the real clearance of the body is smaller.",
    ]
    v = child["versions"]
    versions = {k: v[k] for k in ("mujoco", "jax", "crazyflow", "webots") if k in v}
    if "lsy_drone_racing" in v:
        versions["race_environment"] = v["lsy_drone_racing"]
    if "lsy_drone_racing_commit" in v:
        versions["race_environment_commit"] = v["lsy_drone_racing_commit"]
    out = {
        "format": FORMAT, "version": VERSION,
        "source": {
            "run_id": run_id, "tag": tag, "simulator": child.get("simulator"), "date": date, "date_source": date_source, "seed": seed,
            "controller": {"name": child.get("controller_name", "nose_forward_controller"),
                           "settings": {"nose_mode": child.get("nose_mode"), "yaw_rate_max_rad_s": child.get("yaw_rate_max"),
                                        "yaw_rate_max_used": child.get("nose_mode") == "forward"}},
            "control_rate_hz": freq, "simulator_versions": versions,
        },
        "layout": lay,
        "samples": {"count": n, "rate_hz": freq, "time_s": _r(time, DIGITS["time"]),
                    "x_m": _r(pos[:, 0], DIGITS["pos"]), "y_m": _r(pos[:, 1], DIGITS["pos"]), "z_m": _r(pos[:, 2], DIGITS["pos"]),
                    "vx_mps": _r(vel[:, 0], DIGITS["vel"]), "vy_mps": _r(vel[:, 1], DIGITS["vel"]), "vz_mps": _r(vel[:, 2], DIGITS["vel"]),
                    "speed_mps": _r(speed, DIGITS["speed"]),
                    "roll_deg": _r(roll, DIGITS["deg"]), "pitch_deg": _r(pitch, DIGITS["deg"]), "yaw_deg": _r(yaw, DIGITS["deg"]),
                    "qx": _r(quat[:, 0], DIGITS["quat"]), "qy": _r(quat[:, 1], DIGITS["quat"]),
                    "qz": _r(quat[:, 2], DIGITS["quat"]), "qw": _r(quat[:, 3], DIGITS["quat"]),
                    "passages": passed.tolist()},
        "events": events, "summary": summary,
        "gui": {"path": path, "run": run, "unfilled": unfilled},
        "films": list(films),
    }
    return out


def check_output(text: str) -> None:
    m = FORBIDDEN_NAMES.search(text)
    if m:
        raise InputError(f"the result text would contain the name '{m.group(0)}'; remove it from the inputs (for example the layout title)")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--npz", required=True, type=Path)
    ap.add_argument("--child", required=True, type=Path)
    ap.add_argument("--layout", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--films", nargs="*", default=[], help="file names of films that lie beside --out")
    ap.add_argument("--run-id")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--date", help="flight date, YYYY-MM-DD or 'YYYY-MM-DD HH:MM'; default = modification date of the npz")
    ap.add_argument("--title", default="Track of the flight")
    args = ap.parse_args(argv)
    try:
        for f in args.films:
            _need(Path(f).name == f, f"--films wants plain file names, got '{f}'")
            _need((args.out.parent / f).is_file(), f"film '{f}' does not lie beside {args.out}")
        a, child, layout = load_inputs(args.npz, args.child, args.layout)
        date, source = args.date, "--date"
        if not date:
            date = _dt.datetime.fromtimestamp(args.npz.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
            source = "modification time of the npz file (not the time of the flight)"
        res = build(a, child, layout, films=args.films, run_id=args.run_id, seed=args.seed, date=date, title=args.title, date_source=source)
        text = render(res) + "\n"
        check_output(text)
        json.loads(text)  # the file must read back
    except InputError as e:
        print(f"flight_result: {e}", file=sys.stderr)
        return 1
    args.out.write_text(text, encoding="utf-8")
    s = res["summary"]
    print(f"wrote {args.out} ({len(text.encode('utf-8'))} bytes): {res['samples']['count']} samples, lap {s['lap_time_s']} s, "
          f"{s['passages']} of {s['passages_total']} passages")
    return 0


if __name__ == "__main__":
    sys.exit(main())
