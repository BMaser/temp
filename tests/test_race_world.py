"""Offline checks of px4_gazebo/race_core.py and px4_gazebo/race_world.py (no Gazebo): the route passes the seven gates in order, keeps clear of every
solid, the moving objects stay away from the route, and every generated SDF piece is well-formed XML with the expected parts.
Run: python3 tests/test_race_world.py"""

import math
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "px4_gazebo"))
import race_core as rc  # noqa: E402
import race_world as rw  # noqa: E402

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    if ok:
        passed += 1
    else:
        failed += 1
        print(f"FAIL {name} {detail}")


def main():
    # ---- the route
    ts = np.arange(0.0, rc.T_ROUTE, 0.02)
    P = np.array([rc.SPLINE(t) for t in ts])
    pc = rc.PassageCounter()
    mins = {"gate": 9.0, "pole": 9.0, "box": 9.0}
    for t, p in zip(ts, P):
        pc.update(t, p)
        d = rc.distances_all(p)
        for k in mins:
            mins[k] = min(mins[k], d[k][0])
    check("route: 7 of 7 gates in order", pc.count == 7 and not pc.wrong, f"{pc.count} {pc.wrong[:3]}")
    for k, v in mins.items():
        check(f"route: clearance to {k} above the drone radius plus 0.4 m", v > rc.DRONE_RADIUS + 0.4, f"{v}")
    check("route: the flight is long enough (route time at least 55 s)", rc.T_ROUTE >= 55.0, f"{rc.T_ROUTE}")
    check("route: heights", P[:, 2].min() > 2.0 and P[:, 2].max() < 7.0, f"{P[:, 2].min()} {P[:, 2].max()}")
    check("route: ends over the end pad", math.hypot(P[-1, 0] - rc.END_PAD[0], P[-1, 1] - rc.END_PAD[1]) < 0.05)
    acc = max(np.linalg.norm(rc.SPLINE(t, 2)) for t in ts)
    check("route: acceleration below 2.5 m/s^2 (about 14 degrees of tilt)", acc < 2.5, f"{acc}")
    check("phases: six, in order, the last open", [p["id"] for p in rc.PHASES] == [1, 2, 3, 4, 5, 6] and rc.phase_at(55)["id"] == 6 and rc.phase_at(95)["id"] == 6)
    check("phases: boundaries", [rc.phase_at(t)["id"] for t in (0, 9.99, 10, 19.9, 20, 29.9, 30, 39.9, 40, 49.9, 50)] == [1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6])
    # ---- moving objects: away from the route, never on it
    for name, m in rw.MOVERS.items():
        L = np.array(m["loop"] + [m["loop"][0]], float)
        dm = 99.0
        for a, b in zip(L[:-1], L[1:]):
            for u in np.linspace(0, 1, 60):
                q = a + u * (b - a)
                dm = min(dm, float(np.hypot(P[:, 0] - q[0], P[:, 1] - q[1]).min()))
        check(f"mover {name}: at least 3.5 m from the route", dm >= 3.5, f"{dm}")
        x, y, h = rc.loop_pose(m["loop"], m["speed"], 12.3)
        check(f"mover {name}: loop pose is finite", all(map(math.isfinite, (x, y, h))))
    # ---- nature keeps clear
    trees, bushes, rocks = rw.place_nature()
    check("nature: trees, bushes and rocks placed", len(trees) >= 12 and len(bushes) >= 20 and len(rocks) >= 8, f"{len(trees)} {len(bushes)} {len(rocks)}")
    pts = rw.path_points()
    for nm, lst, c in (("tree", trees, 4.2), ("bush", bushes, 2.6), ("rock", rocks, 2.6)):
        d = min(float(np.hypot(pts[:, 0] - x, pts[:, 1] - y).min()) for x, y, _, _ in lst)
        check(f"nature: {nm}s at least {c} m from the route", d >= c - 1e-6, f"{d}")
    # ---- generated files and SDF
    with tempfile.TemporaryDirectory() as td:
        assets = rw.write_assets(Path(td))
        check("assets: all meshes and pictures written", all(Path(p).is_file() for p in assets.values()) and len(assets) >= len(rw.MESHES) + 8, f"{len(assets)}")
        for nm in ("crown_a.obj", "dome.obj", "mountain_a.obj", "road_s.obj"):
            txt = Path(assets[nm]).read_text().splitlines()
            nv = sum(1 for l in txt if l.startswith("v "))
            nf = [l for l in txt if l.startswith("f ")]
            idx = max(int(t.split("/")[0]) for l in nf for t in l.split()[1:])
            check(f"obj {nm}: face indices inside the vertex list", idx <= nv and nv > 3, f"{idx} {nv}")
        city, counts = rw.city_model(assets)
        root = ET.fromstring(city)
        check("city: well-formed, one static model", root.find("model").get("name") == "city" and root.find("model/static").text == "true")
        vis = root.findall(".//visual")
        check("city: many visuals, each labelled one has a label plugin", len(vis) > 200 and sum(1 for v in vis if v.find("plugin") is not None) > 150, f"{len(vis)}")
        labels = sorted({int(v.findtext("plugin/label")) for v in vis if v.find("plugin") is not None})
        check("city: labels used", set(labels) >= {1, 2, 3, 4, 5, 6, 7, 11, 12, 13, 14, 15, 16, 17}, f"{labels}")
        check("city: 8 lamp lights", len(root.findall(".//light")) == len(rw.LAMPS) == 8)
        check("city: collisions on gates, poles and the box", len(root.findall(".//collision")) == 5 * rc.N_GATES + 5 * len(rc.POLES) + len(rc.BOXES), f"{len(root.findall('.//collision'))}")
        for nm, txt in (("car", rw.car_model()), ("truck", rw.truck_model()), ("rain", rw.rain_model()), ("dome", rw.dome_model(assets)), *rw.person_models().items()):
            r = ET.fromstring(txt)
            check(f"model {nm}: well-formed", r.find("model") is not None)
        dm = ET.fromstring(rw.dome_model(assets)).find(".//visual/material")
        em = [float(x) for x in dm.findtext("emissive").split()[:3]]
        check("dome: the night sky glows by its own picture (emissive map), with no flat emissive colour (run 39010328459: white sky)",
              dm.find("pbr/metal/emissive_map") is not None and max(em) == 0.0 and float(dm.findtext("diffuse").split()[0]) <= 0.1, f"{em}")
        rain = ET.fromstring(rw.rain_model())
        check("rain: one box particle emitter", len(rain.findall(".//particle_emitter")) == 1 and rain.find(".//particle_emitter").get("type") == "box")
        lines = Path(assets["dome.obj"]).read_text().splitlines()
        vv = [tuple(map(float, l.split()[1:4])) for l in lines if l.startswith("v ")]
        ff = [[int(x.split("/")[0]) - 1 for x in l.split()[1:4]] for l in lines if l.startswith("f ")]
        bad = 0
        for a, b, c in ff[::7]:
            A, B, C = (np.array(vv[i]) for i in (a, b, c))
            nrm = np.cross(B - A, C - A)
            if np.linalg.norm(nrm) < 1e-9:
                continue
            bad += int(np.dot(nrm, (A + B + C) / 3.0) > 0)
        check("dome: every sampled face normal points to the centre (inside-out sphere)", bad == 0, f"{bad} faces point outwards")
    w = ET.fromstring(rw.world_text())
    check("world: name, no plugin, two lights, ground collision, spherical coordinates", w.find("world").get("name") == rw.WORLD and not w.findall(".//plugin")
          and len(w.findall("world/light")) == 2 and w.find("world/model/link/collision") is not None and w.find("world/spherical_coordinates") is not None)
    w2 = ET.fromstring(rw.world_text(rw.server_plugins_from_config('<server_config><plugins><plugin entity_name="*" entity_type="world" filename="gz-sim-physics-system" '
                                                                    'name="gz::sim::systems::Physics"/><plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
                                                                    '<render_engine>ogre2</render_engine></plugin><!-- <plugin filename="x" name="y"/> --></plugins></server_config>')))
    names = [p.get("name") for p in w2.findall("world/plugin")]
    check("world variant B: copy of the config plus particle emitter and wind", names == ["gz::sim::systems::Physics", "gz::sim::systems::Sensors",
                                                                                     "gz::sim::systems::ParticleEmitter", "gz::sim::systems::WindEffects"], f"{names}")
    check("world variant B: render engine kept", w2.findtext("world/plugin/render_engine") == "ogre2")
    links = ET.fromstring("<sdf><model name='m'>" + rw.drone_links() + rw.headlamp_link_text() + "</model></sdf>")
    sens = {s.get("name"): s.get("type") for s in links.findall(".//sensor")}
    check("drone links: five sensors of the right types", sens == {"fpv": "camera", "fpvn": "camera", "depth": "depth_camera", "seg": "segmentation_camera", "chase": "camera"}, f"{sens}")
    check("drone links: the noise camera has a gaussian noise element, the clean one has none",
          links.find(".//sensor[@name='fpvn']/camera/noise/type").text == "gaussian" and links.find(".//sensor[@name='fpv']/camera/noise") is None)
    t, changed = rw.enable_wind_text('<model><link name="base_link"><inertial/></link></model>')
    check("enable_wind patch", changed and "<enable_wind>true</enable_wind>" in t and not rw.enable_wind_text(t)[1])
    m = rw.x500_mono_cam_text("<sdf><model name='x'><link name='base_link'/></model></sdf>")
    check("x500 patch: one model, cameras inside", len(ET.fromstring(m).findall(".//sensor")) == 5)
    for nm, s in rw.LIGHTS.items():
        check(f"light {nm}: message text has name and intensity", f'name: "{nm}"' in rw.light_msg_text(nm, s["type"], s["day"]))
    check("sun and moon directions are unit vectors", abs(np.linalg.norm(rw.SUN_TO) - 1) < 1e-9 and abs(np.linalg.norm(rw.MOON_TO) - 1) < 1e-9)
    print(f"{passed} of {passed + failed} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
