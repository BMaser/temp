"""Turn a track layout (JSON from the track designer) into a level file (TOML) for LSY Drone Racing.

Usage:
    python tools/layout_to_level.py LAYOUT.json TEMPLATE.toml OUT.toml

TEMPLATE.toml is LSY's config/level0.toml (commit ac7377b). Every line of it that does not describe
the track is copied byte for byte. Only these lines are replaced (keys as read by LSY):
  gate_order                 [env.track]            race_core.py line 401 -> utils.py load_gate_order lines 55-80
                                                    (signed, 1-based; negative = reverse crossing)
  [[env.track.gates]]  pos, rpy                     race_core.py lines 688-690 and utils.py lines 35-38
                                                    (rpy = Euler "xyz" in radians; we write [0, 0, yaw])
  [[env.track.obstacles]]  pos                      race_core.py lines 696-697 and utils.py line 45
                                                    (z = top of the pole; obstacle.xml: the pole hangs below)
  [[env.track.drones]]  pos, rpy                    utils.py lines 47-51 (drones[k] = every key of drone 0;
                                                    quat is made from rpy). So the start DIRECTION fits:
                                                    it is the yaw in rpy. vel and ang_vel stay as in the template.
Fields of the track designer that are NOT part of the level file (read, checked, and then left out of it):
  end    {"pos": [x, y, z]}  optional. Where the route ends. Must lie inside the simulator box. The level file has
                             no key for it; the test tests/crazyflow_track_look.py reads it from the layout and
                             puts the end pad there.
  route  [{"type": "gate", "gate": n, "reverse": bool} | {"type": "point", "pos": [x, y, z]}, ...]  optional.
         The gate passages must equal gate_order (the designer writes both from the same list). The route
         points are checked (3 finite numbers, inside the box) and IGNORED for the level file.
Checks that can fail (exit code 1, message starts with "ERROR"):
  - an end point outside the simulator box, a malformed end or route entry, or a route whose gate passages
    differ from gate_order
  - a gate whose shape is not "square" (only the square model exists)
  - a passage of gate_order that names a gate that does not exist (0 or |n| > number of gates)
  - a gate frame, a pole or the start outside the simulator box
    (x, y in [-3, 3] m, z in [0, 2.5] m; race_core.py lines 410-411)
  - the written file, read back with tomllib, differs from the layout in any number, or any
    line outside the replaced ones differs from the template.
Only the Python standard library is used (tomllib needs Python 3.11 or newer).
"""

import json
import math
import sys
import tomllib
from pathlib import Path

BOX_X = (-3.0, 3.0)
BOX_Y = (-3.0, 3.0)
BOX_Z = (0.0, 2.5)
GATE_HALF = 0.36  # a gate frame is 0.72 m high and wide around its center (config/level0.toml comment)


class LayoutError(Exception):
    pass


def fnum(value, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise LayoutError(f"{what} is not a finite number: {value!r}")
    return float(value)


def vec3(value, what: str) -> list:
    if not isinstance(value, list) or len(value) != 3:
        raise LayoutError(f"{what} must be a list of 3 numbers, got {value!r}")
    return [fnum(v, what) for v in value]


def fmt(x: float) -> str:
    """Shortest text that reads back as exactly the same float (Python repr); -0.0 becomes 0.0."""
    x = float(x) + 0.0
    return repr(x)


def fmt_vec(v) -> str:
    return "[" + ", ".join(fmt(x) for x in v) + "]"


def parse_layout(layout: dict) -> dict:
    """Read and validate the layout. Raises LayoutError with a clear message."""
    for key in ("gates", "obstacles", "start", "gate_order"):
        if key not in layout:
            raise LayoutError(f"the layout has no '{key}'")
    gates = []
    for i, g in enumerate(layout["gates"], start=1):
        shape = g.get("shape")
        if shape != "square":
            raise LayoutError(f"gate {i} has shape {shape!r}; only 'square' gates have a model so far")
        gates.append({"pos": vec3(g.get("pos"), f"gate {i} pos"), "yaw": fnum(g.get("yaw_rad"), f"gate {i} yaw_rad")})
    if not gates:
        raise LayoutError("the layout has no gates")
    poles = [{"pos": vec3(o.get("pos"), f"pole {i} pos")} for i, o in enumerate(layout["obstacles"], start=1)]
    start = {
        "pos": vec3(layout["start"].get("pos"), "start pos"),
        "yaw": fnum(layout["start"].get("yaw_rad"), "start yaw_rad"),
    }
    order = layout["gate_order"]
    if not isinstance(order, list) or not order:
        raise LayoutError("gate_order must be a non-empty list")
    clean = []
    for k, n in enumerate(order, start=1):
        if isinstance(n, bool) or not isinstance(n, int):
            raise LayoutError(f"passage {k} of gate_order is not an integer: {n!r}")
        if n == 0:
            raise LayoutError(f"passage {k} of gate_order is 0; gate numbers start at 1")
        if abs(n) > len(gates):
            raise LayoutError(
                f"passage {k} of gate_order is {n}, which names gate {abs(n)}, but the layout has only {len(gates)} gates"
            )
        clean.append(n)
    end = None
    if layout.get("end") is not None:
        if not isinstance(layout["end"], dict):
            raise LayoutError(f"end must be an object with a 'pos' list, got {layout['end']!r}")
        end = {"pos": vec3(layout["end"].get("pos"), "end pos")}
    route_gates, route_points = [], []
    route = layout.get("route")
    if route is not None:
        if not isinstance(route, list):
            raise LayoutError(f"route must be a list, got {route!r}")
        for k, e in enumerate(route, start=1):
            if not isinstance(e, dict) or e.get("type") not in ("gate", "point"):
                raise LayoutError(f"route entry {k} is neither a gate nor a point: {e!r}")
            if e["type"] == "gate":
                n = e.get("gate")
                if isinstance(n, bool) or not isinstance(n, int) or not (1 <= n <= len(gates)):
                    raise LayoutError(f"route entry {k} names gate {n!r}, which is not a gate of this layout (1 to {len(gates)})")
                route_gates.append(-n if e.get("reverse") else n)
            else:
                route_points.append(vec3(e.get("pos"), f"route entry {k} pos"))
        if route_gates != clean:
            raise LayoutError(f"the gate passages of route {route_gates} differ from gate_order {clean}")
    return {"gates": gates, "poles": poles, "start": start, "order": clean, "end": end, "route_points": route_points}


def check_box(track: dict) -> None:
    def in_range(v, lo_hi):
        return lo_hi[0] <= v <= lo_hi[1]

    for i, g in enumerate(track["gates"], start=1):
        x, y, z = g["pos"]
        # frame corners: center +- 0.36 m along the gate's lateral axis (-sin yaw, cos yaw), z +- 0.36 m
        lx, ly = -math.sin(g["yaw"]), math.cos(g["yaw"])
        for sign in (-1.0, 1.0):
            cx, cy = x + sign * GATE_HALF * lx, y + sign * GATE_HALF * ly
            if not (in_range(cx, BOX_X) and in_range(cy, BOX_Y)):
                raise LayoutError(
                    f"gate {i} reaches ({cx:.3f}, {cy:.3f}) m with its frame, outside the simulator box "
                    f"x {BOX_X}, y {BOX_Y}"
                )
        if not in_range(z - GATE_HALF, BOX_Z) or not in_range(z + GATE_HALF, BOX_Z):
            raise LayoutError(
                f"gate {i}: center height {z} m puts the frame from {z - GATE_HALF:.3f} to {z + GATE_HALF:.3f} m, "
                f"outside the box height {BOX_Z}"
            )
    for i, p in enumerate(track["poles"], start=1):
        x, y, z = p["pos"]
        if not (in_range(x, BOX_X) and in_range(y, BOX_Y)):
            raise LayoutError(f"pole {i} at ({x}, {y}) is outside the simulator box x {BOX_X}, y {BOX_Y}")
        if not (BOX_Z[0] < z <= BOX_Z[1]):
            raise LayoutError(f"pole {i}: top height {z} m is outside (0, {BOX_Z[1]}] m")
    x, y, z = track["start"]["pos"]
    if not (in_range(x, BOX_X) and in_range(y, BOX_Y) and in_range(z, BOX_Z)):
        raise LayoutError(f"the start ({x}, {y}, {z}) is outside the simulator box x {BOX_X}, y {BOX_Y}, z {BOX_Z}")
    if track.get("end") is not None:
        x, y, z = track["end"]["pos"]
        if not (in_range(x, BOX_X) and in_range(y, BOX_Y) and in_range(z, BOX_Z)):
            raise LayoutError(f"the end ({x}, {y}, {z}) is outside the simulator box x {BOX_X}, y {BOX_Y}, z {BOX_Z}")
    for i, (x, y, z) in enumerate(track.get("route_points", []), start=1):
        if not (in_range(x, BOX_X) and in_range(y, BOX_Y) and in_range(z, BOX_Z)):
            raise LayoutError(f"route point {i} ({x}, {y}, {z}) is outside the simulator box x {BOX_X}, y {BOX_Y}, z {BOX_Z}")


def warnings_for(track: dict, template: dict) -> list:
    """Facts that are not errors: safety limits of the template, and the side a straight line arrives on."""
    out = []
    lim = template.get("env", {}).get("track", {}).get("safety_limits")
    if lim:
        lo, hi = lim["pos_limit_low"], lim["pos_limit_high"]
        items = [(f"gate {i}", g["pos"]) for i, g in enumerate(track["gates"], 1)]
        items += [(f"pole {i}", p["pos"]) for i, p in enumerate(track["poles"], 1)]
        items += [("start", track["start"]["pos"])]
        for name, pos in items:
            if any(pos[a] < lo[a] or pos[a] > hi[a] for a in range(2)):
                out.append(
                    f"{name} at ({pos[0]}, {pos[1]}) lies outside the template's [env.track.safety_limits] "
                    f"x {lo[0]}..{hi[0]}, y {lo[1]}..{hi[1]} (in the LSY files read, only "
                    f"race_core.py lines 919-925 uses them, when track.randomize = true; deploy scripts were not read)"
                )
    prev, prev_name = track["start"]["pos"], "the start"
    for k, n in enumerate(track["order"], start=1):
        gi = abs(n) - 1
        g = track["gates"][gi]
        s = 1.0 if n > 0 else -1.0
        ax = (math.cos(g["yaw"]), math.sin(g["yaw"]))
        local_x = (prev[0] - g["pos"][0]) * ax[0] + (prev[1] - g["pos"][1]) * ax[1]
        if s * local_x > 0:
            out.append(
                f"passage {k} (gate {abs(n)}, {'forward' if s > 0 else 'reverse'}): {prev_name} lies on the EXIT side "
                f"of the gate plane (distance {abs(local_x):.2f} m), so a straight line would cross the wrong way"
            )
        prev, prev_name = g["pos"], f"gate {abs(gi) + 1}"
    return out


def build_level(track: dict, template_text: str, layout_name: str, template_name: str) -> tuple:
    """Return (text, generated_line_indices, replaced_template_indices)."""
    eol = "\r\n" if "\r\n" in template_text else "\n"
    lines = template_text.splitlines(keepends=True)
    stripped = [l.strip() for l in lines]

    def expect_one(pred, what):
        hits = [i for i, l in enumerate(lines) if pred(l)]
        if len(hits) != 1:
            raise LayoutError(f"the template should have exactly one {what}, found {len(hits)}")
        return hits[0]

    # --- find the pieces to replace in the template ---
    order_i = expect_one(lambda l: l.lstrip().startswith("gate_order") and "=" in l, "gate_order line")
    drone_i = expect_one(lambda l: l.strip() == "[[env.track.drones]]", "[[env.track.drones]] table")

    def region(header: str, keys: tuple) -> tuple:
        starts = [i for i, s in enumerate(stripped) if s == header]
        if not starts:
            raise LayoutError(f"the template has no {header}")
        a, b = starts[0], starts[0]
        while b < len(lines) and (stripped[b] == header or stripped[b].split("=")[0].strip() in keys):
            b += 1
        if len(starts) != sum(1 for i in range(a, b) if stripped[i] == header):
            raise LayoutError(f"the {header} tables of the template are not in one block")
        return a, b

    g0, g1 = region("[[env.track.gates]]", ("pos", "rpy"))
    o0, o1 = region("[[env.track.obstacles]]", ("pos",))
    pos_i = rpy_i = None
    j = drone_i + 1
    while j < len(lines) and stripped[j] and not stripped[j].startswith("["):
        key = stripped[j].split("=")[0].strip()
        if key == "pos":
            pos_i = j
        elif key == "rpy":
            rpy_i = j
        j += 1
    if pos_i is None or rpy_i is None:
        raise LayoutError("the drone table of the template has no pos and rpy lines")

    # --- generated text ---
    header = [
        f"# GENERATED by tools/layout_to_level.py from layout {layout_name} and template {template_name}. Do not edit by hand.",
        "# Everything that is not the track is the template, line for line. The start direction is the yaw in",
        "# [[env.track.drones]] rpy (the level file has a key for it, so it is not only a comment).",
    ]
    gate_lines = []
    for i, g in enumerate(track["gates"], start=1):
        gate_lines += [
            f"# gate {i} of the layout",
            "[[env.track.gates]]",
            f"pos = {fmt_vec(g['pos'])}",
            f"rpy = {fmt_vec([0.0, 0.0, g['yaw']])}",
        ]
    pole_lines = []
    for i, p in enumerate(track["poles"], start=1):
        pole_lines += [f"# pole {i} of the layout (z = top of the pole)", "[[env.track.obstacles]]", f"pos = {fmt_vec(p['pos'])}"]
    order_line = "gate_order = [" + ", ".join(str(n) for n in track["order"]) + "]"
    pos_line = f"pos = {fmt_vec(track['start']['pos'])}"
    rpy_line = f"rpy = {fmt_vec([0.0, 0.0, track['start']['yaw']])}"

    # --- assemble with tags so the byte check can find the generated lines again ---
    out = [("gen", l + eol) for l in header]
    replaced = set()
    i = 0
    while i < len(lines):
        if i == order_i:
            out.append(("gen", order_line + eol)); replaced.add(i)
        elif i == g0:
            out += [("gen", l + eol) for l in gate_lines]; replaced.update(range(g0, g1)); i = g1; continue
        elif i == o0:
            out += [("gen", l + eol) for l in pole_lines]; replaced.update(range(o0, o1)); i = o1; continue
        elif i == pos_i:
            out.append(("gen", pos_line + eol)); replaced.add(i)
        elif i == rpy_i:
            out.append(("gen", rpy_line + eol)); replaced.add(i)
        else:
            out.append(("keep", lines[i]))
        i += 1
    text = "".join(l for _, l in out)
    gen_idx = [k for k, (t, _) in enumerate(out) if t == "gen"]
    return text, gen_idx, replaced


def verify(track: dict, out_path: Path, template_text: str, gen_idx: list, replaced: set) -> list:
    """Read the written file back; return the list of passed checks. Raises LayoutError on a difference."""
    passed = []
    raw = out_path.read_bytes().decode("utf-8")
    back = tomllib.loads(raw)
    tmpl = tomllib.loads(template_text)
    tr = back["env"]["track"]

    def same(name, got, want):
        if got != want:
            raise LayoutError(f"read-back check failed: {name}: file has {got!r}, layout has {want!r}")
        passed.append(name)

    same("number of gates", len(tr["gates"]), len(track["gates"]))
    for i, g in enumerate(track["gates"], start=1):
        same(f"gate {i} pos", tr["gates"][i - 1]["pos"], g["pos"])
        same(f"gate {i} rpy", tr["gates"][i - 1]["rpy"], [0.0, 0.0, g["yaw"]])
    same("number of poles", len(tr["obstacles"]), len(track["poles"]))
    for i, p in enumerate(track["poles"], start=1):
        same(f"pole {i} pos", tr["obstacles"][i - 1]["pos"], p["pos"])
    same("number of drones", len(tr["drones"]), 1)
    same("start pos", tr["drones"][0]["pos"], track["start"]["pos"])
    same("start rpy", tr["drones"][0]["rpy"], [0.0, 0.0, track["start"]["yaw"]])
    same("gate_order", tr["gate_order"], track["order"])
    # everything that is not the track, as parsed values
    def strip_track(d):
        d = json.loads(json.dumps(d))
        t = d["env"]["track"]
        for k in ("gates", "obstacles", "gate_order"):
            t.pop(k)
        for k in ("pos", "rpy"):
            t["drones"][0].pop(k)
        return d

    same("all other settings equal the template (parsed)", strip_track(back), strip_track(tmpl))
    # and as bytes: the lines that were not replaced must be identical, in order
    out_lines = raw.splitlines(keepends=True)
    kept_out = [l for k, l in enumerate(out_lines) if k not in set(gen_idx)]
    tmpl_lines = template_text.splitlines(keepends=True)
    kept_tmpl = [l for k, l in enumerate(tmpl_lines) if k not in replaced]
    same("lines outside the track are identical to the template (bytes)", kept_out == kept_tmpl, True)
    return passed


def main(argv: list) -> int:
    if len(argv) != 4:
        print(__doc__)
        return 2
    layout_path, template_path, out_path = (Path(a) for a in argv[1:])
    try:
        if sys.version_info < (3, 11):
            raise LayoutError(f"Python 3.11 or newer is needed for tomllib, this is {sys.version.split()[0]}")
        layout = json.loads(layout_path.read_text(encoding="utf-8"))
        track = parse_layout(layout)
        check_box(track)
        template_bytes = template_path.read_bytes()
        template_text = template_bytes.decode("utf-8")
        text, gen_idx, replaced = build_level(track, template_text, layout_path.name, template_path.name)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_bytes(text.encode("utf-8"))
        passed = verify(track, out_path, template_text, gen_idx, replaced)
        notes = warnings_for(track, tomllib.loads(template_text))
    except LayoutError as e:
        print(f"ERROR: {e}")
        return 1
    except (OSError, ValueError, KeyError, tomllib.TOMLDecodeError) as e:
        print(f"ERROR: {type(e).__name__}: {e}")
        return 1
    print(f"OK: wrote {out_path} ({len(track['gates'])} gates, {len(track['poles'])} poles, "
          f"{len(track['order'])} passages)")
    print(f"read-back checks passed: {len(passed)}")
    for n in passed:
        print(f"  ok  {n}")
    for n in notes:
        print(f"NOTE: {n}")
    if track.get("end") is not None:
        print(f"NOTE: the layout has an end point {track['end']['pos']}; it is inside the box and is NOT written to the "
              f"level file (the level file has no key for it; the test puts the end pad there)")
    else:
        print("NOTE: the layout has no end point, so there is no end pad")
    if track.get("route_points"):
        print(f"NOTE: the route has {len(track['route_points'])} free route point(s); they are checked and IGNORED "
              f"for the level file (LSY has no key for them)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
