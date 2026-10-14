"""The files the simulator loads for one look, and the colors of that look.

A look is one of "today" (the files that are in this repository now, unchanged) or one of the four returned designs
(magenta, cyan, red, violet), whose pictures and colors are in assets/looks/<look>/ and assets/looks/palettes.json.

What `build_look_files(look, out_dir)` writes (the folders the cloud test copies from):
  <out_dir>/gate/gate.xml      assets/gates/square/gate.xml with ONLY the rgba of the two materials frame_mat and rope_mat
                               replaced by the look's `frame` and `spring` colors. Every other character is the same;
                               `assert_only_two_colors_changed` checks this and the self test makes it fail on purpose.
  <out_dir>/gate/gate_top.png, gate_bottom.png, gate_left.png, gate_right.png, gate_post_bark.png
                               the five gate pictures of the look (byte for byte the returned files)
  <out_dir>/gate/spring.stl    the spring mesh (the same for every look)
  <out_dir>/pads/pad_start.xml, pad_end.xml   the pad models (the same for every look), and
  <out_dir>/pads/pad_start.png, pad_end.png   the pad pictures of the look

Colors are stored as integers 0-255 in palettes.json. `floor_colors`, `sky_colors`, `drone_color` return them as floats 0-1
(value / 255). For "today" the floor and the sky are not changed (None), and the drone is the orange of the earlier tests.

Standard library only (so the parent process of the test and the self test need nothing installed). The self test also
uses Pillow for the picture sizes when it is installed.

    python tools/look_assets.py --self-test
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
import tempfile
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parent.parent
GATE_DIR = REPO_DIR / "assets" / "gates" / "square"
PADS_DIR = REPO_DIR / "assets" / "pads"
LOOKS_DIR = REPO_DIR / "assets" / "looks"
PALETTES_FILE = LOOKS_DIR / "palettes.json"
# Folder of the returned design export, only used by the self test when it exists on this PC.
RETURNED_DIR = Path(
    r"G:\My Drive\github\Comprehansive_cv\coursera_mirror\mirrors\Autonomous_Racing_Drone_Path\runs"
    r"\2026-10-13_design_pack\returned\2026-10-13_2117_claude_design_export\returned"
)

LOOKS = ("today", "magenta", "cyan", "red", "violet")
NEW_LOOKS = LOOKS[1:]
GATE_PICTURES = ("gate_top.png", "gate_bottom.png", "gate_left.png", "gate_right.png", "gate_post_bark.png")
GATE_FILES = ("gate.xml",) + GATE_PICTURES + ("spring.stl",)  # the same list as MODEL_FILES in tests/crazyflow_track_look.py
PAD_PICTURES = ("pad_start.png", "pad_end.png")
PAD_MODELS = ("pad_start.xml", "pad_end.xml")
USED_PICTURES = GATE_PICTURES + PAD_PICTURES  # the 7 pictures of a look that the simulator loads
FLAT_DRAWINGS = ("gate_triangle.png", "gate_circle.png")  # in the folder of every look, not used now
EXPECTED_SIZES = {  # (width, height) in pixels
    "gate_top.png": (2048, 341), "gate_bottom.png": (2048, 341), "gate_left.png": (341, 1366), "gate_right.png": (341, 1366),
    "gate_post_bark.png": (256, 2560), "pad_start.png": (1024, 1024), "pad_end.png": (1024, 1024),
}
TODAY_DRONE_RGB = (255, 95, 0)  # ORANGE_RGB in tests/crazyflow_track_look.py
MATERIALS = {"frame_mat": "frame", "rope_mat": "spring"}  # material in gate.xml -> key in palettes.json


def sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_look(look: str) -> None:
    if look not in LOOKS:
        raise ValueError(f"unknown look {look!r}; known: {list(LOOKS)}")


# ---------------------------------------------------------------------------------------------
# Palettes
# ---------------------------------------------------------------------------------------------


def load_palettes(path: Path = PALETTES_FILE) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def palette(look: str) -> dict:
    """The entry of palettes.json for a new look."""
    check_look(look)
    if look == "today":
        raise ValueError("the look 'today' has no palette: it is the present files")
    return load_palettes()[look]


def unit(rgb) -> tuple:
    """Integers 0-255 -> floats 0-1."""
    return tuple(float(v) / 255.0 for v in rgb)


def floor_colors(look: str):
    """((r, g, b), (r, g, b)) of the two floor colors A and B as floats 0-1, or None for 'today' (floor not changed)."""
    check_look(look)
    if look == "today":
        return None
    a, b = palette(look)["floor"]
    return unit(a), unit(b)


def sky_colors(look: str):
    """(horizon, top) as floats 0-1, or None for 'today' (sky not changed)."""
    check_look(look)
    if look == "today":
        return None
    p = palette(look)
    return unit(p["skyH"]), unit(p["skyT"])


def drone_color(look: str) -> tuple:
    """Body color of the drone as floats 0-1 (for 'today': the hi-vis orange of the earlier tests)."""
    check_look(look)
    return unit(TODAY_DRONE_RGB if look == "today" else palette(look)["drone"])


def drone_color_255(look: str) -> tuple:
    check_look(look)
    return tuple(TODAY_DRONE_RGB if look == "today" else palette(look)["drone"])


# ---------------------------------------------------------------------------------------------
# gate.xml
# ---------------------------------------------------------------------------------------------

_MATERIAL_RE = r'(<material\s+name="{name}"\s+rgba=")([^"]*)(")'


def fmt_unit(v: int) -> str:
    """One color number 0-255 as text in the style of the template ('0.941176', '1', '0'): six significant digits."""
    return format(round(v / 255.0, 6), "g")


def rgba_text(rgb255) -> str:
    return " ".join(fmt_unit(int(v)) for v in rgb255) + " 1"


def template_gate_xml() -> str:
    return (GATE_DIR / "gate.xml").read_bytes().decode("utf-8")  # no newline translation: the bytes stay the same


def material_rgba(xml: str, name: str) -> list:
    """The four numbers of the rgba of material `name` in a gate.xml text."""
    found = re.findall(_MATERIAL_RE.format(name=re.escape(name)), xml)
    if len(found) != 1:
        raise ValueError(f"expected exactly one material {name!r} with an rgba, found {len(found)}")
    return [float(t) for t in found[0][1].split()]


def replace_material_rgba(xml: str, name: str, rgb255) -> str:
    pattern = re.compile(_MATERIAL_RE.format(name=re.escape(name)))
    if len(pattern.findall(xml)) != 1:
        raise ValueError(f"expected exactly one material {name!r} with an rgba")
    return pattern.sub(lambda m: m.group(1) + rgba_text(rgb255) + m.group(3), xml, count=1)


def changed_lines(a: str, b: str) -> list:
    """[(line number from 1, old line, new line)] for every line that differs. Both texts must have the same line count."""
    la, lb = a.split("\n"), b.split("\n")
    if len(la) != len(lb):
        raise AssertionError(f"line counts differ: {len(la)} against {len(lb)}")
    return [(i + 1, x, y) for i, (x, y) in enumerate(zip(la, lb)) if x != y]


def assert_only_two_colors_changed(template: str, new: str, expected: dict) -> list:
    """`expected` = {"frame_mat": rgb255, "rope_mat": rgb255}. Raises AssertionError unless the two texts differ only in
    the rgba value of these materials (at most these two lines; a line may be unchanged when the color is the same) and the
    new rgba values are the expected ones. Returns the changed lines."""
    diffs = changed_lines(template, new)
    allowed = {}
    for name in expected:
        allowed[name] = re.compile(_MATERIAL_RE.format(name=re.escape(name)))
    for no, old, cur in diffs:
        owner = [n for n, p in allowed.items() if p.search(old)]
        if len(owner) != 1:
            raise AssertionError(f"line {no} changed but is not the rgba line of {sorted(expected)}: {cur.strip()!r}")
        # Everything outside the rgba value must be identical.
        strip = allowed[owner[0]]
        if strip.sub(lambda m: m.group(1) + "#" + m.group(3), old) != strip.sub(lambda m: m.group(1) + "#" + m.group(3), cur):
            raise AssertionError(f"line {no}: something other than the rgba value changed: {old.strip()!r} -> {cur.strip()!r}")
    for name, rgb in expected.items():
        got = material_rgba(new, name)
        want = [v / 255.0 for v in rgb] + [1.0]
        if len(got) != 4 or max(abs(x - y) for x, y in zip(got, want)) > 1e-6:
            raise AssertionError(f"material {name}: rgba {got} is not the expected {want}")
    return diffs


def expected_gate_colors(look: str) -> dict:
    """{"frame_mat": (r, g, b), "rope_mat": (r, g, b)} as integers 0-255 that the compiled model must show.
    For a new look: from palettes.json. For 'today': read from the template gate.xml."""
    check_look(look)
    if look == "today":
        xml = template_gate_xml()
        return {n: tuple(int(round(v * 255)) for v in material_rgba(xml, n)[:3]) for n in MATERIALS}
    p = palette(look)
    return {n: tuple(p[key]) for n, key in MATERIALS.items()}


def build_gate_xml(look: str) -> str:
    check_look(look)
    template = template_gate_xml()
    if look == "today":
        return template
    exp = expected_gate_colors(look)
    new = template
    for name, rgb in exp.items():
        new = replace_material_rgba(new, name, rgb)
    assert_only_two_colors_changed(template, new, exp)
    return new


# ---------------------------------------------------------------------------------------------
# Files
# ---------------------------------------------------------------------------------------------


def look_picture_source(look: str, name: str) -> Path:
    """Where the picture `name` of the look comes from."""
    check_look(look)
    if look == "today":
        return (PADS_DIR if name in PAD_PICTURES else GATE_DIR) / name
    return LOOKS_DIR / look / name


def build_look_files(look: str, out_dir) -> dict:
    """Write the gate folder and the pad folder of the look into `out_dir`. Returns {"gate_dir", "pads_dir", "files": {name: sha256}}."""
    check_look(look)
    out = Path(out_dir)
    gate_out, pads_out = out / "gate", out / "pads"
    gate_out.mkdir(parents=True, exist_ok=True)
    pads_out.mkdir(parents=True, exist_ok=True)
    files = {}
    xml = build_gate_xml(look)
    (gate_out / "gate.xml").write_bytes(xml.encode("utf-8"))
    for name in GATE_PICTURES:
        shutil.copyfile(look_picture_source(look, name), gate_out / name)
    shutil.copyfile(GATE_DIR / "spring.stl", gate_out / "spring.stl")
    for name in PAD_MODELS:
        shutil.copyfile(PADS_DIR / name, pads_out / name)
    for name in PAD_PICTURES:
        shutil.copyfile(look_picture_source(look, name), pads_out / name)
    for name in GATE_FILES:
        files[f"gate/{name}"] = sha256(gate_out / name)
    for name in PAD_MODELS + PAD_PICTURES:
        files[f"pads/{name}"] = sha256(pads_out / name)
    return {"look": look, "gate_dir": str(gate_out), "pads_dir": str(pads_out), "files": files}


def gate_source_hashes(look: str) -> dict:
    """sha256 of the gate files LSY must load for this look, computed from the SOURCES (not from a built copy):
    the look's own pictures and the gate.xml text this module builds. Used to check what LSY really loaded."""
    check_look(look)
    out = {"gate.xml": hashlib.sha256(build_gate_xml(look).encode("utf-8")).hexdigest()}
    for name in GATE_PICTURES:
        out[name] = sha256(look_picture_source(look, name))
    out["spring.stl"] = sha256(GATE_DIR / "spring.stl")
    return out


# ---------------------------------------------------------------------------------------------
# colors.txt against palettes.json
# ---------------------------------------------------------------------------------------------

_COLOR_LINES = {  # key -> (regex of the line start, how many numbers, index into the numbers)
    "frame": r"Gate frame\s+(\d+) (\d+) (\d+)\s*$",
    "spring": r"Gate springs\s+(\d+) (\d+) (\d+)\s*$",
    "skyH": r"Sky horizon\s+(\d+) (\d+) (\d+)\s*$",
    "skyT": r"Sky top\s+(\d+) (\d+) (\d+)\s*$",
    "drone": r"Drone body / center propeller\s+(\d+) (\d+) (\d+)\s*$",
    "gate": r"Gate band \(texture main color\)\s+(\d+) (\d+) (\d+)\s*$",
}
_FLOOR_LINE = r"Floor, two flat colors, no mirror\s+A (\d+) (\d+) (\d+)\s+B (\d+) (\d+) (\d+)\s+\((\S+) m squares\)"


def parse_colors_txt(path: Path) -> dict:
    text = Path(path).read_text(encoding="utf-8")
    out = {}
    for key, rx in _COLOR_LINES.items():
        m = re.search(rx, text, re.M)
        out[key] = [int(v) for v in m.groups()] if m else None
    m = re.search(_FLOOR_LINE, text)
    out["floor"] = [[int(v) for v in m.groups()[:3]], [int(v) for v in m.groups()[3:6]]] if m else None
    out["floor_square_m"] = float(m.group(7)) if m else None
    return out


def compare_colors_with_palettes() -> list:
    """Every difference between a colors.txt and palettes.json, as text lines (empty list: they agree)."""
    pal = load_palettes()
    problems = []
    for look in NEW_LOOKS:
        txt = parse_colors_txt(LOOKS_DIR / look / "colors.txt")
        for key in ("gate", "frame", "spring", "skyH", "skyT", "drone", "floor"):
            if txt[key] is None:
                problems.append(f"{look}: colors.txt has no readable line for {key}")
            elif txt[key] != pal[look][key]:
                problems.append(f"{look}: {key}: colors.txt {txt[key]} against palettes.json {pal[look][key]}")
        if txt["floor_square_m"] != 0.5:
            problems.append(f"{look}: colors.txt floor square size {txt['floor_square_m']} m, not 0.5")
    return problems


# ---------------------------------------------------------------------------------------------
# Self test
# ---------------------------------------------------------------------------------------------


def self_test() -> int:
    failures, passed = [], 0

    def check(name: str, ok: bool, detail: str = "") -> None:
        nonlocal passed
        if ok:
            passed += 1
            print(f"  ok   {name}")
        else:
            failures.append(name)
            print(f"  FAIL {name}: {detail}")

    try:
        from PIL import Image
    except ImportError:
        Image = None
        print("  (Pillow is not installed: picture sizes are not checked)")

    template = template_gate_xml()
    check("template gate.xml has the two materials", len(material_rgba(template, "frame_mat")) == 4 and len(material_rgba(template, "rope_mat")) == 4)
    with tempfile.TemporaryDirectory() as tmp:
        for look in LOOKS:
            built = build_look_files(look, Path(tmp) / look)
            gate_xml = (Path(built["gate_dir"]) / "gate.xml").read_bytes().decode("utf-8")
            if look == "today":
                check("today: gate.xml is the template, byte for byte", gate_xml.encode("utf-8") == (GATE_DIR / "gate.xml").read_bytes())
                check("today: no line differs", changed_lines(template, gate_xml) == [])
                for name in GATE_PICTURES:
                    check(f"today: {name} equals assets/gates/square", sha256(Path(built["gate_dir"]) / name) == sha256(GATE_DIR / name))
                for name in PAD_PICTURES:
                    check(f"today: {name} equals assets/pads", sha256(Path(built["pads_dir"]) / name) == sha256(PADS_DIR / name))
                continue
            exp = expected_gate_colors(look)
            diffs = changed_lines(template, gate_xml)
            check(f"{look}: exactly two lines of gate.xml differ", len(diffs) == 2, f"{len(diffs)} lines differ")
            try:
                assert_only_two_colors_changed(template, gate_xml, exp)
                ok, detail = True, ""
            except AssertionError as e:
                ok, detail = False, str(e)
            check(f"{look}: only the two rgba values changed, to the palette colors", ok, detail)
            nums = [(a.split('rgba="')[1].split('"')[0], b.split('rgba="')[1].split('"')[0]) for _, a, b in diffs]
            check(f"{look}: both changed lines still have four rgba numbers", all(len(o.split()) == 4 and len(n.split()) == 4 for o, n in nums))
            check(f"{look}: frame_mat is the frame color", [round(v * 255) for v in material_rgba(gate_xml, "frame_mat")[:3]] == list(exp["frame_mat"]))
            check(f"{look}: rope_mat is the spring color", [round(v * 255) for v in material_rgba(gate_xml, "rope_mat")[:3]] == list(exp["rope_mat"]))
            for name in USED_PICTURES:
                folder = Path(built["gate_dir"]) if name in GATE_PICTURES else Path(built["pads_dir"])
                src = LOOKS_DIR / look / name
                check(f"{look}: built {name} is byte-equal to assets/looks", (folder / name).read_bytes() == src.read_bytes())
                if RETURNED_DIR.is_dir():
                    check(f"{look}: built {name} is byte-equal to the returned file", (folder / name).read_bytes() == (RETURNED_DIR / look / name).read_bytes())
                if Image is not None:
                    with Image.open(folder / name) as im:
                        check(f"{look}: {name} is {EXPECTED_SIZES[name]}", im.size == EXPECTED_SIZES[name], f"size {im.size}")
            check(f"{look}: spring.stl and pad models are the template's",
                  sha256(Path(built["gate_dir"]) / "spring.stl") == sha256(GATE_DIR / "spring.stl")
                  and all(sha256(Path(built["pads_dir"]) / n) == sha256(PADS_DIR / n) for n in PAD_MODELS))
            fl, sk, dr = floor_colors(look), sky_colors(look), drone_color(look)
            p = palette(look)
            check(f"{look}: floor, sky and drone colors are floats 0-1 equal to palette/255",
                  fl == (unit(p["floor"][0]), unit(p["floor"][1])) and sk == (unit(p["skyH"]), unit(p["skyT"])) and dr == unit(p["drone"])
                  and all(0.0 <= v <= 1.0 for c in (*fl, *sk, dr) for v in c))
            hashes = gate_source_hashes(look)
            check(f"{look}: source hashes equal the hashes of the built files", all(hashes[n] == built["files"][f"gate/{n}"] for n in GATE_FILES))
    # Negative controls: the checks above must be able to fail.
    exp = expected_gate_colors("magenta")
    good = build_gate_xml("magenta")
    for label, bad in (
        ("an extra change (emission of frame_mat)", good.replace('reflectance="0.5" emission="1"', 'reflectance="0.5" emission="0.9"')),
        ("a wrong frame color", replace_material_rgba(good, "frame_mat", (1, 2, 3))),
        ("a changed comment line", good.replace("Square race gate, new look.", "Square race gate, newer look.")),
    ):
        try:
            assert_only_two_colors_changed(template, bad, exp)
            check(f"negative control: {label} is caught", False, "no error raised")
        except AssertionError:
            check(f"negative control: {label} is caught", True)
    check("negative control: an unknown look is refused", _raises(ValueError, check_look, "pink"))
    # colors.txt against palettes.json: reported, not a failure of this test (the task says: use palettes.json).
    problems = compare_colors_with_palettes()
    print(f"  colors.txt against palettes.json: {len(problems)} difference(s)")
    for line in problems:
        print(f"    {line}")
    print(f"look_assets self test: {passed} checks passed, {len(failures)} failed")
    return 1 if failures else 0


def _raises(exc, fn, *args) -> bool:
    try:
        fn(*args)
    except exc:
        return True
    return False


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--self-test", action="store_true", help="run the checks of this module")
    ap.add_argument("--build", metavar="LOOK", help="build the files of a look")
    ap.add_argument("--out", help="folder for --build")
    args = ap.parse_args()
    if args.self_test:
        sys.exit(self_test())
    if args.build:
        if not args.out:
            ap.error("--build needs --out")
        print(json.dumps(build_look_files(args.build, args.out), indent=2))
        return
    ap.print_help()


if __name__ == "__main__":
    main()
