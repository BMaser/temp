"""AirSim (Unreal Engine 4.27, Blocks v1.8.1) on the free Windows runner with the WARP software Direct3D 12 driver.

Usage (Python 3.9 with the `airsim` client):  python airsim_run.py --bin DIR --out DIR
Tries the start variants a, b, c in order. The first one that gives a real picture (standard deviation above 5) wins.
Then: picture rate, free-running real-time factor, a flight with stepped recording, films.
"""
import argparse
import json
import math
import os
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

PORT = 41451
EXE_NAMES = ["Blocks.exe", "Blocks-Win64-Shipping.exe", "CrashReportClient.exe"]
BASE = "-nosound -ResX=640 -ResY=480 -unattended -stdout -FullStdOutLogOutput"
VARIANTS = {
    "a": f"-RenderOffscreen -dx12 -WARP {BASE}",
    "b": f"-RenderOffscreen -dx12 -WARP -AllowSoftwareRendering {BASE}",
    "c": f"-dx12 -WARP -AllowSoftwareRendering -windowed {BASE}",
}
CAP = {"ImageType": 0, "Width": 640, "Height": 480, "FOV_Degrees": 90}


def settings_json(view_mode):
    s = {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "ClockSpeed": 1.0,
        "Vehicles": {"Drone1": {
            "VehicleType": "SimpleFlight", "AutoCreate": True,
            "Cameras": {
                "front": {"CaptureSettings": [CAP], "X": 0.3, "Y": 0.0, "Z": 0.0, "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0},
                "chase": {"CaptureSettings": [CAP], "X": -3.5, "Y": 0.0, "Z": -1.5, "Pitch": -15.0, "Roll": 0.0, "Yaw": 0.0},
            }}},
    }
    if view_mode:
        s["ViewMode"] = view_mode
    return s


def write_settings(view_mode, out: Path, tag):
    d = Path(os.path.expanduser("~")) / "Documents" / "AirSim"
    d.mkdir(parents=True, exist_ok=True)
    txt = json.dumps(settings_json(view_mode), indent=2)
    (d / "settings.json").write_text(txt)
    (out / f"settings_{tag}.json").write_text(txt)


def port_open():
    s = socket.socket()
    s.settimeout(1.0)
    try:
        s.connect(("127.0.0.1", PORT))
        return True
    except Exception:
        return False
    finally:
        s.close()


def ue_running():
    r = subprocess.run(["tasklist"], capture_output=True, text=True)
    return "Blocks.exe" in r.stdout or "Blocks-Win64-Shipping.exe" in r.stdout


def decode(r):
    if r.width == 0 or r.height == 0 or len(r.image_data_uint8) == 0:
        return None
    a = np.frombuffer(r.image_data_uint8, np.uint8)
    ch = a.size // (r.width * r.height)
    a = a.reshape(r.height, r.width, ch)[:, :, :3][:, :, ::-1]
    return np.ascontiguousarray(a)


def grab(client, names):
    import airsim
    rs = client.simGetImages([airsim.ImageRequest(n, airsim.ImageType.Scene, False, False) for n in names])
    return [decode(r) for r in rs]


class Session:
    def __init__(self, exe, out, logdir, vname, view_mode):
        self.exe, self.out, self.logdir, self.vname, self.view_mode = exe, out, logdir, vname, view_mode
        self.tag = f"{vname}_{view_mode or 'default'}"
        self.proc = None
        self.client = None
        self.info = {"variant": vname, "view_mode": view_mode or "default"}

    def start(self, wait_s=240):
        import airsim
        common.kill_names(EXE_NAMES)
        time.sleep(2)
        write_settings(self.view_mode, self.out, self.tag)
        self.t_start_epoch = time.time()
        args = VARIANTS[self.vname].split()
        sim_log = self.logdir / f"stdout_{self.tag}.log"
        log(f"start {self.tag}: {self.exe} {' '.join(args)}")
        self.proc = subprocess.Popen([str(self.exe)] + args, stdout=open(sim_log, "w"), stderr=subprocess.STDOUT, cwd=str(self.exe.parent))
        t0 = time.time()
        while time.time() - t0 < wait_s:
            if port_open():
                break
            if time.time() - t0 > 40 and not ue_running():
                self.info["error"] = "Unreal process is gone before the RPC port opened"
                return False
            time.sleep(2)
        else:
            self.info["error"] = f"RPC port {PORT} did not open within {wait_s} s"
            return False
        self.info["rpc_up_after_s"] = round(time.time() - t0, 1)
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        return True

    def probe(self, wait_s=90):
        """Ask for a front and a chase picture until one has a standard deviation above 5."""
        t0 = time.time()
        stds = []
        while time.time() - t0 < wait_s:
            if not ue_running():
                self.info["error"] = "Unreal process ended while waiting for a picture"
                break
            try:
                ims = grab(self.client, ["front", "chase"])
            except Exception as e:
                stds.append(f"exception {type(e).__name__}: {str(e)[:100]}")
                time.sleep(3)
                continue
            st = [common.image_stats(i)["std"] if i is not None else None for i in ims]
            stds.append(st)
            if all(s is not None and s > common.PASS_MIN_STD for s in st):
                for n, i in zip(("front", "chase"), ims):
                    common.save_png(i, self.out / "pictures" / f"{self.tag}_{n}.png")
                self.info["probe_after_s"] = round(time.time() - t0, 1)
                self.info["probe_stds"] = st
                return True
            if any(s is not None and s > common.PASS_MIN_STD for s in st):
                for n, i, s in zip(("front", "chase"), ims, st):
                    if s is not None and s > common.PASS_MIN_STD:
                        common.save_png(i, self.out / "pictures" / f"{self.tag}_{n}_partial.png")
            time.sleep(2)
        self.info["probe_stds_history"] = stds[-8:]
        # keep the last (failed) pictures for the record
        try:
            ims = grab(self.client, ["front", "chase"])
            for n, i in zip(("front", "chase"), ims):
                if i is not None:
                    common.save_png(i, self.out / "pictures" / f"{self.tag}_{n}_FAILED.png")
        except Exception:
            pass
        return False

    def measure(self, n=20):
        """Pictures per wall second (front only, then front+chase in one call), and free-running real-time factor."""
        res = {}
        grab(self.client, ["front"])
        t0 = time.perf_counter()
        for _ in range(n):
            grab(self.client, ["front"])
        w = time.perf_counter() - t0
        res["front_only"] = {"pictures": n, "wall_s": w, "pictures_per_s": n / w}
        t0 = time.perf_counter()
        for _ in range(n):
            grab(self.client, ["front", "chase"])
        w = time.perf_counter() - t0
        res["front_and_chase_per_call"] = {"calls": n, "wall_s": w, "calls_per_s": n / w, "pictures_per_s": 2 * n / w}
        # real-time factor, free running, no picture requests
        s0 = self.client.getMultirotorState().timestamp
        w0 = time.perf_counter()
        time.sleep(10)
        s1 = self.client.getMultirotorState().timestamp
        w1 = time.perf_counter()
        res["free_running"] = {"wall_s": w1 - w0, "sim_s": (s1 - s0) / 1e9, "real_time_factor": (s1 - s0) / 1e9 / (w1 - w0)}
        self.info["measure"] = res
        log(f"measure {self.tag}: {json.dumps(res)}")
        return res

    def stop(self, since_epoch=None):
        common.kill_names(EXE_NAMES)
        time.sleep(3)

    def collect_logs(self, roots):
        got = common.collect_logs(roots, self.logdir, self.tag, self.t_start_epoch)
        self.info["unreal_logs"] = [p.name for p in got]
        f = common.log_facts(got + [self.logdir / f"stdout_{self.tag}.log"])
        self.info["log_names_warp"] = f["names_warp"]
        self.info["log_rhi_lines"] = f["rhi_lines"]
        self.info["log_error_lines"] = f["error_lines"]
        return f


def fly_and_record(client, out: Path, cap_s):
    import airsim
    rec = out / "frames"
    rec.mkdir(parents=True, exist_ok=True)
    res = {"cap_wall_s": cap_s}
    client.reset()
    time.sleep(1)
    client.enableApiControl(True)
    client.armDisarm(True)
    p = client.getMultirotorState().kinematics_estimated.position
    p0 = (p.x_val, p.y_val, p.z_val)
    plan = common.Planner()
    track = []
    client.simPause(True)
    t_wall0 = time.perf_counter()
    t_sim0 = client.getMultirotorState().timestamp
    k = 0
    stopped = "plan finished"
    walls = []
    while True:
        wall = time.perf_counter() - t_wall0
        if wall > cap_s:
            stopped = f"wall-time cap of {cap_s} s reached"
            break
        st = client.getMultirotorState()
        pos = st.kinematics_estimated.position
        n, e, d = pos.x_val - p0[0], pos.y_val - p0[1], pos.z_val - p0[2]
        track.append((n, e))
        vn, ve, yaw, done = plan.step(n, e, d)
        if done:
            break
        if (st.timestamp - t_sim0) / 1e9 > 120:
            stopped = "plan did not finish within 120 simulated seconds"
            break
        client.moveByVelocityZAsync(vn, ve, p0[2] - plan.alt, 0.3, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, yaw))
        ts = time.perf_counter()
        client.simContinueForTime(0.1)
        t_step = time.perf_counter() - ts
        ims = grab(client, ["front", "chase"])
        t_all = time.perf_counter() - ts
        if ims[0] is None or ims[1] is None:
            stopped = "empty picture returned"
            break
        common.save_jpg(ims[0], rec / f"fpv_{k:05d}.jpg")
        common.save_jpg(ims[1], rec / f"chase_{k:05d}.jpg")
        walls.append((t_step, t_all))
        k += 1
        if k % 20 == 0:
            log(f"flight step {k}: sim {(st.timestamp - t_sim0) / 1e9:.1f} s, wall {wall:.0f} s, phase {plan.phase}, pos {n:.1f},{e:.1f},{d:.1f}")
    wall_total = time.perf_counter() - t_wall0
    sim_total = (client.getMultirotorState().timestamp - t_sim0) / 1e9
    res.update({"steps": k, "stopped": stopped, "wall_s": wall_total, "sim_s": sim_total,
                "real_time_factor_while_recording": sim_total / wall_total if wall_total else None,
                "mean_wall_s_per_step": float(np.mean([a for _, a in walls])) if walls else None,
                "mean_wall_s_continue_only": float(np.mean([a for a, _ in walls])) if walls else None,
                "finished_plan": stopped == "plan finished"})
    try:
        client.simPause(False)
        client.armDisarm(False)
        client.enableApiControl(False)
    except Exception:
        pass
    return res, track, plan.square


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--cap-min", type=float, default=60.0)
    a = ap.parse_args()
    out = Path(a.out)
    (out / "pictures").mkdir(parents=True, exist_ok=True)
    logdir = out / "unreal_logs"
    logdir.mkdir(exist_ok=True)
    exes = sorted(Path(a.bin).rglob("Blocks.exe"), key=lambda p: (len(p.parts), str(p)))
    result = {"simulator": "AirSim v1.8.1 Blocks (Unreal Engine 4.27)", "sessions": [], "winner": None}
    roots = [Path(a.bin), Path(os.environ.get("LOCALAPPDATA", "")) / "Blocks", Path(os.environ.get("LOCALAPPDATA", "")) / "AirSim", Path(os.path.expanduser("~")) / "Documents" / "AirSim"]
    if not exes:
        result["error"] = "Blocks.exe not found"
        (out / "result.json").write_text(json.dumps(result, indent=2))
        return 1
    exe = exes[0]
    log(f"exe {exe}")
    win = None
    for v in ("a", "b", "c"):
        s = Session(exe, out, logdir, v, None)
        try:
            ok = s.start() and s.probe()
        except Exception:
            s.info["traceback"] = traceback.format_exc()[-1500:]
            ok = False
        s.info["picture_ok"] = ok
        if ok:
            try:
                s.measure()
            except Exception:
                s.info["measure_traceback"] = traceback.format_exc()[-1500:]
        s.stop()
        s.collect_logs(roots)
        result["sessions"].append(s.info)
        (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
        if ok:
            win = v
            break
    if not win:
        result["conclusion"] = "no variant gave a real picture"
        (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
        write_summary(out, result)
        return 1
    result["winner"] = win
    # second session: ViewMode NoDisplay (the main window does not draw; the camera pictures are still drawn)
    best_view, best_rate = None, result["sessions"][-1].get("measure", {}).get("front_and_chase_per_call", {}).get("pictures_per_s", 0)
    s2 = Session(exe, out, logdir, win, "NoDisplay")
    try:
        ok2 = s2.start() and s2.probe()
        s2.info["picture_ok"] = ok2
        if ok2:
            m = s2.measure()
            r2 = m["front_and_chase_per_call"]["pictures_per_s"]
            if r2 > best_rate:
                best_view, best_rate = "NoDisplay", r2
    except Exception:
        s2.info["traceback"] = traceback.format_exc()[-1500:]
        ok2 = False
    result["sessions"].append(s2.info)
    result["film_view_mode"] = best_view or "default"
    if best_view == "NoDisplay":
        sfilm = s2
    else:
        s2.stop()
        s2.collect_logs(roots)
        sfilm = Session(exe, out, logdir, win, None)
        sfilm.tag += "_film"
        if not (sfilm.start() and sfilm.probe()):
            result["film_error"] = "could not restart the winning variant for the film"
            result["sessions"].append(sfilm.info)
            (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
            write_summary(out, result)
            return 1
        result["sessions"].append(sfilm.info)
    try:
        fl, track, square = fly_and_record(sfilm.client, out, a.cap_min * 60)
        result["flight"] = fl
        result["films"] = common.make_films(out / "frames", track, square, out / "films")
        (out / "track.json").write_text(json.dumps(track))
    except Exception:
        result["flight_traceback"] = traceback.format_exc()[-2500:]
    sfilm.stop()
    sfilm.collect_logs(roots)
    # keep only a small sample of the frames in the artifact
    import shutil
    fr = out / "frames"
    if fr.exists():
        keep = sorted(fr.glob("fpv_*.jpg"))[::25] + sorted(fr.glob("chase_*.jpg"))[::25]
        for p in fr.glob("*.jpg"):
            if p not in keep:
                p.unlink()
    (out / "result.json").write_text(json.dumps(result, indent=2, default=str))
    write_summary(out, result)
    return 0


def write_summary(out, result):
    L = ["# AirSim on the Windows runner with WARP", ""]
    for s in result["sessions"]:
        L.append(f"## Session {s.get('variant')} (view mode {s.get('view_mode')})")
        L.append(f"- Picture worked: {s.get('picture_ok')}")
        if "error" in s:
            L.append(f"- Error: {s['error']}")
        L.append(f"- RPC port opened after (s): {s.get('rpc_up_after_s')}")
        L.append(f"- Log names 'Microsoft Basic Render Driver': {s.get('log_names_warp')}")
        m = s.get("measure")
        if m:
            L.append(f"- Pictures per wall second, front only, 640x480: {m['front_only']['pictures_per_s']:.2f}")
            L.append(f"- Pictures per wall second, front plus chase in one call: {m['front_and_chase_per_call']['pictures_per_s']:.2f}")
            L.append(f"- Real-time factor, free running: {m['free_running']['real_time_factor']:.3f}")
        L.append("")
    if result.get("flight"):
        f = result["flight"]
        L += ["## Flight and films", json.dumps(f, indent=2), json.dumps(result.get("films"), indent=2)]
    (out / "summary.md").write_text("\n".join(L))


if __name__ == "__main__":
    sys.exit(main())
