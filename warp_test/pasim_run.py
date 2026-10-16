"""Project AirSim (Unreal Engine 5, Blocks v1.0.1) on the free Windows runner with the WARP software Direct3D 12 driver.

Usage (Python 3.10 with `projectairsim`):  python pasim_run.py --bin DIR --src DIR_OF_ProjectAirSim_clone --out DIR
"""
import argparse
import asyncio
import copy
import json
import os
import shutil
import socket
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import common  # noqa: E402
from common import log  # noqa: E402

SERVICES_PORT = 8990
BASE = "-nosound -ResX=640 -ResY=480 -unattended -stdout -FullStdOutLogOutput"
VARIANTS = {
    "a": f"-RenderOffscreen -dx12 -WARP {BASE}",
    "b": f"-RenderOffscreen -dx12 -WARP -AllowSoftwareRendering {BASE}",
    "c": f"-dx12 -WARP -AllowSoftwareRendering -windowed {BASE}",
}
KILL = ["*-Win64-Shipping.exe", "CrashReportClient.exe"]


def port_open(port):
    s = socket.socket()
    s.settimeout(1.0)
    try:
        s.connect(("127.0.0.1", port))
        return True
    except Exception:
        return False
    finally:
        s.close()


EXE_NAME = [""]


def ue_running():
    r = subprocess.run(["tasklist"], capture_output=True, text=True)
    return "Win64-Shipping.exe" in r.stdout or (bool(EXE_NAME[0]) and EXE_NAME[0] in r.stdout)


def prepare_config(sim_config: Path, out: Path):
    """Add a forward camera FrontCamera to the robot config (copy of DownCamera, looking forward). Returns the sensor names."""
    import commentjson
    scene_p = sim_config / "scene_basic_drone.jsonc"
    scene = commentjson.loads(scene_p.read_text())
    (out / "scene_basic_drone_used.json").write_text(json.dumps(scene, indent=2))
    info = {"scene": str(scene_p)}
    robot_name = None
    for a in scene.get("actors", []):
        if a.get("type") == "robot":
            robot_name = a.get("robot-config")
    info["robot_config"] = robot_name
    if not robot_name:
        return info
    rp = sim_config / robot_name
    robot = commentjson.loads(rp.read_text())
    sensors = robot.get("robot-config", robot).get("sensors") if isinstance(robot.get("robot-config", robot), dict) else None
    if sensors is None:
        sensors = robot.get("sensors")
    info["sensor_ids"] = [s.get("id") for s in sensors or []]
    down = next((s for s in sensors or [] if s.get("id") == "DownCamera"), None)
    if down is not None:
        info["DownCamera_config"] = down
        front = copy.deepcopy(down)
        front["id"] = "FrontCamera"
        for k in list(front.keys()):
            if isinstance(front[k], dict) and "rpy-deg" in front[k]:
                front[k]["rpy-deg"] = "0 0 0"
            if isinstance(front[k], dict) and "rpy" in front[k]:
                front[k]["rpy"] = "0 0 0"
        info["FrontCamera_config"] = front
        sensors.append(front)
        rp.write_text(json.dumps(robot, indent=2))
        info["sensor_ids_after"] = [s.get("id") for s in sensors]
    return info


def to_rgb(img):
    a = np.asarray(img)
    if a.ndim == 3 and a.shape[2] >= 3:
        a = a[:, :, :3][:, :, ::-1]
    return np.ascontiguousarray(a)


def grab(drone, sensor):
    from projectairsim.types import ImageType
    from projectairsim.utils import unpack_image
    images = drone.get_images(sensor, [ImageType.SCENE])
    return to_rgb(unpack_image(list(images.values())[0]))


class Session:
    def __init__(self, exe, out, logdir, vname, sim_config):
        self.exe, self.out, self.logdir, self.vname, self.sim_config = exe, out, logdir, vname, sim_config
        self.tag = vname
        self.info = {"variant": vname}
        self.client = self.world = self.drone = None

    def start(self, wait_s=240):
        from projectairsim import Drone, ProjectAirSimClient, World
        common.kill_names(KILL + [self.exe.name])
        time.sleep(2)
        self.t_start_epoch = time.time()
        args = VARIANTS[self.vname].split()
        sim_log = self.logdir / f"stdout_{self.tag}.log"
        log(f"start {self.tag}: {self.exe} {' '.join(args)}")
        self.proc = subprocess.Popen([str(self.exe)] + args, stdout=open(sim_log, "w"), stderr=subprocess.STDOUT, cwd=str(self.exe.parent))
        t0 = time.time()
        while time.time() - t0 < wait_s:
            if port_open(SERVICES_PORT):
                break
            if time.time() - t0 > 40 and not ue_running():
                self.info["error"] = "Unreal process is gone before the services port opened"
                return False
            time.sleep(2)
        else:
            self.info["error"] = f"services port {SERVICES_PORT} did not open within {wait_s} s"
            return False
        self.info["port_up_after_s"] = round(time.time() - t0, 1)
        self.client = ProjectAirSimClient()
        self.client.connect()
        self.world = World(self.client, "scene_basic_drone.jsonc", delay_after_load_sec=2, sim_config_path=str(self.sim_config))
        self.drone = Drone(self.client, self.world, "Drone1")
        return True

    def sensors(self):
        return self.cams

    def probe(self, wait_s=90):
        t0 = time.time()
        hist = []
        self.cams = ["Chase", "DownCamera"] + (["FrontCamera"] if self.has_front else [])
        while time.time() - t0 < wait_s:
            if not ue_running():
                self.info["error"] = "Unreal process ended while waiting for a picture"
                break
            st, ims = {}, {}
            for c in self.cams:
                try:
                    im = grab(self.drone, c)
                    ims[c] = im
                    st[c] = common.image_stats(im)["std"]
                except Exception as e:
                    st[c] = f"exception {type(e).__name__}: {str(e)[:100]}"
            hist.append(st)
            if any(isinstance(v, float) and v > common.PASS_MIN_STD for v in st.values()):
                for c, im in ims.items():
                    common.save_png(im, self.out / "pictures" / f"{self.tag}_{c}.png")
                self.info["probe_after_s"] = round(time.time() - t0, 1)
                self.info["probe_stds"] = st
                self.info["sizes"] = {c: list(im.shape[:2][::-1]) for c, im in ims.items()}
                self.good = [c for c, v in st.items() if isinstance(v, float) and v > common.PASS_MIN_STD]
                return True
            time.sleep(3)
        self.info["probe_stds_history"] = hist[-6:]
        return False

    def measure(self, n=20):
        res = {}
        cam = "Chase" if "Chase" in self.good else self.good[0]
        grab(self.drone, cam)
        t0 = time.perf_counter()
        for _ in range(n):
            grab(self.drone, cam)
        w = time.perf_counter() - t0
        res["one_camera"] = {"camera": cam, "pictures": n, "wall_s": w, "pictures_per_s": n / w}
        t0 = time.perf_counter()
        for _ in range(n):
            for c in self.good[:2]:
                grab(self.drone, c)
        w = time.perf_counter() - t0
        res["two_cameras"] = {"cameras": self.good[:2], "rounds": n, "wall_s": w, "pictures_per_s": n * len(self.good[:2]) / w}
        s0, w0 = self.world.get_sim_time(), time.perf_counter()
        time.sleep(10)
        s1, w1 = self.world.get_sim_time(), time.perf_counter()
        res["free_running"] = {"wall_s": w1 - w0, "sim_s": (s1 - s0) / 1e9, "real_time_factor": (s1 - s0) / 1e9 / (w1 - w0)}
        self.info["measure"] = res
        log(f"measure {self.tag}: {json.dumps(res)}")
        return res

    def stop(self):
        try:
            self.client.disconnect()
        except Exception:
            pass
        common.kill_names(KILL + [self.exe.name])
        time.sleep(3)

    def collect_logs(self, roots):
        got = common.collect_logs(roots, self.logdir, self.tag, self.t_start_epoch)
        self.info["unreal_logs"] = [p.name for p in got]
        f = common.log_facts(got + [self.logdir / f"stdout_{self.tag}.log"])
        self.info["log_names_warp"] = f["names_warp"]
        self.info["log_rhi_lines"] = f["rhi_lines"]
        self.info["log_error_lines"] = f["error_lines"]


def pos_of(drone):
    k = drone.get_ground_truth_kinematics()
    p = k["pose"]["position"]
    return p["x"], p["y"], p["z"]


async def fly_and_record(s: Session, out: Path, cap_s):
    from projectairsim.drone import YawControlMode
    drone, world = s.drone, s.world
    rec = out / "frames"
    rec.mkdir(parents=True, exist_ok=True)
    fpv_cam = "FrontCamera" if "FrontCamera" in s.good else "DownCamera"
    chase_cam = "Chase"
    res = {"fpv_camera": fpv_cam, "chase_camera": chase_cam, "cap_wall_s": cap_s}
    drone.enable_api_control()
    drone.arm()
    p0 = pos_of(drone)
    plan = common.Planner()
    track = []
    world.pause()
    take = await drone.takeoff_async(timeout_sec=30)
    t_sim0 = world.get_sim_time()
    t_wall0 = time.perf_counter()
    k, stopped, walls = 0, "plan finished", []
    while True:
        wall = time.perf_counter() - t_wall0
        if wall > cap_s:
            stopped = f"wall-time cap of {cap_s} s reached"
            break
        sim_t = (world.get_sim_time() - t_sim0) / 1e9
        if sim_t > 150:
            stopped = "plan did not finish within 150 simulated seconds"
            break
        p = pos_of(drone)
        n, e, d = p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]
        track.append((n, e))
        if not take.done() and sim_t < 12 and -d < 0.8:
            pass  # takeoff command still running; do not override it
        else:
            vn, ve, yaw, done = plan.step(n, e, d)
            if done:
                break
            vd = max(-1.5, min(1.5, 1.0 * ((-plan.alt) - d)))
            await drone.move_by_velocity_async(vn, ve, vd, 0.3, YawControlMode.MaxDegreeOfFreedom, False, yaw)
        ts = time.perf_counter()
        world.continue_for_sim_time(100_000_000, True)
        await asyncio.sleep(0.005)
        ims = [grab(drone, fpv_cam), grab(drone, chase_cam)]
        t_all = time.perf_counter() - ts
        common.save_jpg(ims[0], rec / f"fpv_{k:05d}.jpg")
        common.save_jpg(ims[1], rec / f"chase_{k:05d}.jpg")
        walls.append(t_all)
        k += 1
        if k % 20 == 0:
            log(f"flight step {k}: sim {sim_t:.1f} s, wall {wall:.0f} s, phase {plan.phase}, pos {n:.1f},{e:.1f},{d:.1f}")
    wall_total = time.perf_counter() - t_wall0
    sim_total = (world.get_sim_time() - t_sim0) / 1e9
    res.update({"steps": k, "stopped": stopped, "wall_s": wall_total, "sim_s": sim_total,
                "real_time_factor_while_recording": sim_total / wall_total if wall_total else None,
                "mean_wall_s_per_step": float(np.mean(walls)) if walls else None,
                "finished_plan": stopped == "plan finished"})
    try:
        world.resume()
        drone.disarm()
        drone.disable_api_control()
    except Exception:
        pass
    return res, track, plan.square


def write_summary(out, result):
    L = ["# Project AirSim on the Windows runner with WARP", ""]
    for s in result["sessions"]:
        L.append(f"## Session {s.get('variant')}")
        L.append(f"- Picture worked: {s.get('picture_ok')}")
        if "error" in s:
            L.append(f"- Error: {s['error']}")
        L.append(f"- Services port opened after (s): {s.get('port_up_after_s')}")
        L.append(f"- Log names 'Microsoft Basic Render Driver': {s.get('log_names_warp')}")
        m = s.get("measure")
        if m:
            L.append(f"- Pictures per wall second, one camera: {m['one_camera']['pictures_per_s']:.2f}")
            L.append(f"- Pictures per wall second, two cameras: {m['two_cameras']['pictures_per_s']:.2f}")
            L.append(f"- Real-time factor, free running: {m['free_running']['real_time_factor']:.3f}")
        L.append("")
    if result.get("flight"):
        L += ["## Flight and films", json.dumps(result["flight"], indent=2), json.dumps(result.get("films"), indent=2)]
    (out / "summary.md").write_text("\n".join(L))


async def amain(a):
    out = Path(a.out)
    (out / "pictures").mkdir(parents=True, exist_ok=True)
    logdir = out / "unreal_logs"
    logdir.mkdir(exist_ok=True)
    result = {"simulator": "Project AirSim v1.0.1 Blocks (Unreal Engine 5)", "sessions": [], "winner": None}
    cands = [p for p in Path(a.bin).rglob("*.exe") if not any(x in p.name.lower() for x in ("crash", "prereq", "redist", "unreal"))]
    cands.sort(key=lambda p: (len(p.parts), p.name))
    log("exe candidates: " + str([str(c) for c in cands[:8]]))
    result["exe_candidates"] = [str(c) for c in cands[:8]]
    if not cands:
        result["error"] = "no exe found"
        (out / "result.json").write_text(json.dumps(result, indent=2))
        return 1
    exe = cands[0]
    EXE_NAME[0] = exe.name
    sim_config = Path(a.src) / "client" / "python" / "example_user_scripts" / "sim_config"
    try:
        result["config"] = prepare_config(sim_config, out)
    except Exception:
        result["config_error"] = traceback.format_exc()[-1500:]
    has_front = "FrontCamera" in (result.get("config", {}).get("sensor_ids_after") or [])
    roots = [Path(a.bin), Path(os.environ.get("LOCALAPPDATA", "")) / "Blocks", Path(os.environ.get("LOCALAPPDATA", "")) / "ProjectAirSim"]
    win = None
    for v in ("a", "b", "c"):
        s = Session(exe, out, logdir, v, sim_config)
        s.has_front = has_front
        try:
            ok = s.start() and s.probe()
        except Exception:
            s.info["traceback"] = traceback.format_exc()[-1800:]
            ok = False
        s.info["picture_ok"] = ok
        if ok:
            try:
                s.measure()
            except Exception:
                s.info["measure_traceback"] = traceback.format_exc()[-1500:]
            win = s
        else:
            s.stop()
            s.collect_logs(roots)
        result["sessions"].append(s.info)
        (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
        if ok:
            break
    if not win:
        result["conclusion"] = "no variant gave a real picture"
        (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
        write_summary(out, result)
        return 1
    result["winner"] = win.vname
    try:
        fl, track, square = await fly_and_record(win, out, a.cap_min * 60)
        result["flight"] = fl
        result["films"] = common.make_films(out / "frames", track, square, out / "films")
        (out / "track.json").write_text(json.dumps(track))
    except Exception:
        result["flight_traceback"] = traceback.format_exc()[-2500:]
    win.stop()
    win.collect_logs(roots)
    fr = out / "frames"
    if fr.exists():
        keep = set(sorted(fr.glob("fpv_*.jpg"))[::25] + sorted(fr.glob("chase_*.jpg"))[::25])
        for p in fr.glob("*.jpg"):
            if p not in keep:
                p.unlink()
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    write_summary(out, result)
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--cap-min", type=float, default=60.0)
    sys.exit(asyncio.run(amain(ap.parse_args())))
