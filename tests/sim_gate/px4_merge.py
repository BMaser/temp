"""Merge the per-world results of the PX4 + Gazebo gate (gate_baylands.json, gate_default.json) into gate.json.

The reported numbers come from the baylands world (outdoor, the more natural one) when its camera worked, else from
the default world. The other world's numbers are added to the notes, because the speed differs a lot between worlds.
"""
import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402

d = common.results_dir("px4_gazebo")
gates = {}
for w in ("baylands", "default"):
    f = d / f"gate_{w}.json"
    if f.is_file():
        gates[w] = json.loads(f.read_text())
if not gates:
    sys.exit(0)
order = [w for w in ("baylands", "default") if w in gates]
main = next((w for w in order if gates[w].get("camera_ok")), order[0])
g = dict(gates[main])
extra = []
for w in order:
    x = gates[w]
    extra.append(f"[world {w}] headless_ok={x.get('headless_ok')} camera_ok={x.get('camera_ok')} size={x.get('camera_size')} "
                 f"physics_steps_per_s={x.get('physics_steps_per_s')} rtf={x.get('real_time_factor')} camera_fps={x.get('camera_frames_per_s')}")
g["notes"] = f"Reported numbers: world {main}. " + " ".join(extra) + " || DETAILS OF THE REPORTED WORLD: " + (g.get("notes") or "")
for i in (1, 2, 3):
    src = d / f"{main}_cam_{i}.png"
    if src.is_file():
        shutil.copy(src, d / f"cam_{i}.png")
common.write_gate(g)
print(json.dumps(g, indent=2, default=str))
