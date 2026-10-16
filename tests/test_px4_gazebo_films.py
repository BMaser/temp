"""Offline checks of px4_gazebo/films_core.py (pure Python, no Gazebo): every condition patches the baylands world text, the result
is well-formed XML, the patched pieces are where they should be, and the flight setpoints are what the film needs.
Run: python3 tests/test_px4_gazebo_films.py"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "px4_gazebo"))
import films_core as fc  # noqa: E402

WORLD = (REPO / "tests" / "data" / "baylands_main.sdf").read_text()
passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    if ok:
        passed += 1
    else:
        failed += 1
        print(f"FAIL {name} {detail}")


def main():
    root0 = ET.fromstring(WORLD)
    check("original parses", root0.tag == "sdf")
    for name, cond in fc.CONDITIONS.items():
        try:
            t = fc.patch_world(WORLD, cond)
            root = ET.fromstring(t)
        except Exception as e:  # noqa: BLE001
            check(f"{name}: patch and parse", False, repr(e))
            continue
        w = root.find("world")
        check(f"{name}: one scene", len(w.findall("scene")) == 1)
        check(f"{name}: one sun", len([l for l in w.findall("light") if l.get("name") == "sunUTC"]) == 1)
        check(f"{name}: spherical coordinates and park kept", w.find("spherical_coordinates") is not None and len(w.findall("include")) >= 2)
        sc = w.find("scene")
        if cond["scene"] is not None:
            check(f"{name}: ambient", sc.findtext("ambient").split()[:3] == cond["scene"]["ambient"].split())
            check(f"{name}: background", sc.findtext("background").split()[:3] == cond["scene"]["background"].split())
            if cond["scene"]["sky"] == fc.SKY_NONE:
                check(f"{name}: no sky", sc.find("sky") is None)
        if "rain" in cond["extras"]:
            check(f"{name}: rain emitter, no world plugin (it would switch off the default systems)", len(w.findall(".//particle_emitter")) == 1 and "<plugin" not in t)
        if "fog" in cond["extras"]:
            check(f"{name}: fog include, no world plugin", "fog generator" in t and "<plugin" not in t)
        if "rain_fullplugins" in cond["extras"]:
            check(f"{name}: full plugin set", len(w.findall("plugin")) == 14 and len(w.findall(".//particle_emitter")) == 1)
        if cond["lamps"]:
            check(f"{name}: lamps", len(w.findall(".//light[@type='point']")) == len(fc.LAMP_POSITIONS))
        if cond["sun"] is not None:
            sun = [l for l in w.findall("light") if l.get("name") == "sunUTC"][0]
            check(f"{name}: sun direction", sun.findtext("direction").split() == cond["sun"]["direction"].split())
    # the model patch
    model = "<sdf><model name='x500_mono_cam'><link name='base_link'/></model></sdf>"
    m = fc.x500_model_text(model, "<link name='chase_cam_link'/>", True)
    r = ET.fromstring(m)
    check("model: chase and headlamp links", {l.get("name") for l in r.iter("link")} >= {"base_link", "chase_cam_link", "headlamp_link"})
    check("model: headlamp is a spot light", r.find(".//light").get("type") == "spot")
    check("model: no headlamp", "headlamp" not in fc.x500_model_text(model, "<link name='c'/>", False))
    # the setpoints
    (n, e, d), (vn, ve, vd) = fc.setpoint(0.0)
    check("setpoint at 0", (n, e, d) == (0.0, 0.0, 0.0) and vd < 0)
    (n, e, d), _ = fc.setpoint(fc.SEG_S)
    check("setpoint at the end", abs(e - fc.MOVE_X) < 1e-9 and abs(d + fc.CLIMB_Z) < 1e-9, f"{e} {d}")
    check("film ids", set(fc.FILMS) == {"F05", "F06"} and all(c in fc.CONDITIONS for f in fc.FILMS.values() for c in f["conditions"] + f["probes"]))
    # a broken world is refused
    try:
        fc.patch_world("<sdf><world name='x'></world></sdf>", fc.CONDITIONS["dusk"])
        check("refuse a world without scene", False)
    except ValueError:
        check("refuse a world without scene", True)
    print(f"{passed} of {passed + failed} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
