"""Project AirSim client test: wait for port 8989, load the shipped scene, take a Chase picture, take off. usage: pasim_client.py <evidence dir> <sim_config dir>"""
import asyncio, json, sys, time, traceback
from pathlib import Path
out = Path(sys.argv[1]); cfg = sys.argv[2]
T0 = time.time(); log = []
def ev(k, **kw):
    e = {"t": round(time.time() - T0, 1), "kind": k, **kw}; log.append(e); print(json.dumps(e), flush=True)
    (out / "client_events.json").write_text(json.dumps(log, indent=1, default=str))
async def main():
    import numpy as np
    from PIL import Image
    from projectairsim import Drone, ProjectAirSimClient, World
    from projectairsim.types import ImageType
    from projectairsim.utils import unpack_image
    c = ProjectAirSimClient()
    while time.time() - T0 < 120:
        try:
            c.connect(); break
        except Exception as e:
            time.sleep(2)
    else:
        ev("no_connection"); return
    ev("connected")
    w = World(c, "scene_basic_drone.jsonc", delay_after_load_sec=2, sim_config_path=cfg)
    d = Drone(c, w, "Drone1"); ev("scene_loaded")
    for sensor in ("Chase", "DownCamera"):
        for k in range(2):
            try:
                im = unpack_image(list(d.get_images(sensor, [ImageType.SCENE]).values())[0])
                a = np.asarray(im)[:, :, :3][:, :, ::-1]
                Image.fromarray(np.ascontiguousarray(a)).save(out / f"pa_{sensor}_{k}.png")
                ev("picture", sensor=sensor, shape=list(a.shape), std=float(a.std()), mean=float(a.mean()))
            except Exception:
                ev("picture_fail", sensor=sensor, tb=traceback.format_exc()[-400:])
            time.sleep(1.5)
    try:
        d.enable_api_control(); d.arm()
        t = await d.takeoff_async(); await asyncio.wait_for(t, 30); ev("takeoff_done")
        im = unpack_image(list(d.get_images("Chase", [ImageType.SCENE]).values())[0])
        Image.fromarray(np.ascontiguousarray(np.asarray(im)[:, :, :3][:, :, ::-1])).save(out / "pa_Chase_after_takeoff.png")
        ev("picture_after_takeoff")
    except Exception:
        ev("flight_fail", tb=traceback.format_exc()[-500:])
try:
    asyncio.run(main())
except Exception:
    ev("client_crash", tb=traceback.format_exc()[-600:])
