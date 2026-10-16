"""Merge the per-size results of rt15.py into rt15.json. usage: python rt15_merge.py <folder with s640x480 and s320x240>"""
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
out = {"probe": "RT15", "simulator": "project_airsim", "version": "Project AirSim 1.0.1 (Unreal Engine 5), scene Blocks (the only packaged scene with a usable sky and time of day; see film notes)",
       "rule": "PASS for a size = at least 15 frames per wall-clock second while the real-time factor stays at 0.95 or more",
       "settings": {"hardware": "GitHub free Linux runner (ubuntu-latest), software Vulkan (lavapipe), no graphics card", "unreal_flags": "-RenderOffScreen -ResX=640 -ResY=480 -nosound -windowed -AllowCPUDevices (Xvfb :99)",
                    "clock": "shipped: step 3 ms, real-time-update-rate 3 ms (real time), not paused, not stepped", "camera": "one nose camera FrontCamera, other sensors disabled, streaming-enabled false, capture-interval 1/30 s",
                    "code": "films_pasim/rt15.py (prepare() writes the config; phase A subscription, phase B get_images), films_pasim/launch_film.sh (Unreal flags)",
                    "world": "Blocks: grey ground and blocks; it has NO natural ground or trees (owner's nature world was not available in Project AirSim 1.0.1 packages without a checkerboard floor)"},
       "sizes": {}}
for d in sorted(root.glob("s*x*")):
    f = d / "rt15_size.json"
    if f.is_file():
        out["sizes"][d.name[1:]] = json.loads(f.read_text())
(root / "rt15.json").write_text(json.dumps(out, indent=1))
for k, v in out["sizes"].items():
    print(k, v.get("stage"), v.get("summary"))
