"""One AirSim test variant. Starts the simulator, watches its control port, and connects with a FIXED client:
a plain socket polls port 41451, and a NEW airsim.MultirotorClient is created for every try (never reuse a failed one).

usage: airsim_variant.py --name A1 --bin <dir with the start script> --out <evidence dir> --args "<unreal args>"
         [--settings minimal|notebook] [--user runner|simuser] [--cap 150]
Writes to --out: events.json, sim.log, ss_<n>s.txt, settings.json, ue_logs/, summary.txt
"""
import argparse, json, os, shutil, signal, socket, subprocess, sys, threading, time, traceback
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--name", required=True)
ap.add_argument("--bin", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--args", default="")
ap.add_argument("--settings", default="minimal")
ap.add_argument("--user", default="")
ap.add_argument("--home", required=True)
ap.add_argument("--cap", type=float, default=150.0)
ap.add_argument("--vk", default="")
a = ap.parse_args()

out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
T0 = time.time()
events = []
def ev(kind, **kw):
    e = {"t": round(time.time() - T0, 2), "kind": kind, **kw}
    events.append(e); print(json.dumps(e), flush=True)
    (out / "events.json").write_text(json.dumps(events, indent=1, default=str))

shot = {"CaptureSettings": [{"ImageType": 0, "Width": 640, "Height": 480, "FOV_Degrees": 90}]}
if a.settings == "minimal":
    settings = {"SettingsVersion": 1.2, "SimMode": "Multirotor", "ClockSpeed": 1,
                "CameraDefaults": shot, "Vehicles": {"Drone1": {"VehicleType": "SimpleFlight", "AutoCreate": True}}}
else:  # copied from notebook_source.py (Step 4)
    settings = {"SettingsVersion": 1.2, "SimMode": "Multirotor", "ClockSpeed": 1, "CameraDefaults": shot,
                "Vehicles": {"Drone1": {"VehicleType": "SimpleFlight", "AutoCreate": True,
                                        "Cameras": {"front": {**shot, "X": 0.3, "Y": 0.0, "Z": 0.0, "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}}}},
                "ExternalCameras": {"Chase": {**shot, "X": -6.0, "Y": 0.0, "Z": -2.5, "Pitch": -22.0, "Roll": 0.0, "Yaw": 0.0}}}
home = Path(a.home)
d = home / "Documents" / "AirSim"; d.mkdir(parents=True, exist_ok=True)
(d / "settings.json").write_text(json.dumps(settings, indent=2))
shutil.copyfile(d / "settings.json", out / "settings.json")
if a.user:
    subprocess.run(["chown", "-R", f"{a.user}:{a.user}", str(home)], check=False)

binp = Path(a.bin)
sh = next(iter(sorted(binp.rglob("*.sh"))), None)
for f in binp.rglob("*"):
    if f.is_file() and "Binaries/Linux" in str(f):
        os.chmod(f, 0o755)
os.chmod(sh, 0o755)
env_pairs = {"HOME": str(home), "DISPLAY": ":99", "SDL_AUDIODRIVER": "dummy"}
if a.vk:
    env_pairs["VK_ICD_FILENAMES"] = a.vk
inner = f"ulimit -c 0; exec bash {sh} {a.args}"
if a.user:
    cmd = ["runuser", "-u", a.user, "--", "env"] + [f"{k}={v}" for k, v in env_pairs.items()] + ["bash", "-c", inner]
    env = None
else:
    cmd = ["bash", "-c", inner]; env = {**os.environ, **env_pairs}
ev("start", cmd=cmd, uid=os.getuid(), settings=a.settings, args=a.args)
marker = time.time()
sim = subprocess.Popen(cmd, stdout=open(out / "sim.log", "wb"), stderr=subprocess.STDOUT, env=env, start_new_session=True, cwd=str(sh.parent))

def ss_at(sec):
    time.sleep(max(0, sec - (time.time() - T0)))
    r = subprocess.run("ss -ltnp", shell=True, capture_output=True, text=True)
    (out / f"ss_{sec}s.txt").write_text(r.stdout + r.stderr)
    listening = [l for l in r.stdout.splitlines() if "4145" in l]
    ev("ss", at=sec, listening_41451=any(":41451 " in l for l in r.stdout.splitlines()), lines=listening, sim_alive=sim.poll() is None)
for s in (2, 5, 10, 20, 40):
    threading.Thread(target=ss_at, args=(s,), daemon=True).start()
def shot_at(sec):
    time.sleep(max(0, sec - (time.time() - T0)))
    r = subprocess.run(["import", "-window", "root", str(out / f"shot_{sec}s.png")], capture_output=True, text=True,
                       env={**os.environ, "DISPLAY": ":99"})
    ev("screenshot", at=sec, rc=r.returncode, err=r.stderr[:200])
for s in (2, 5, 10):
    threading.Thread(target=shot_at, args=(s,), daemon=True).start()

import airsim
first_open = None; connected = False; tries = 0; sim_exit = None
flight_done = False
def port_open():
    try:
        with socket.create_connection(("127.0.0.1", 41451), timeout=0.3):
            return True
    except Exception:
        return False
while time.time() - T0 < a.cap:
    rc = sim.poll()
    if rc is not None and sim_exit is None:
        sim_exit = rc; ev("sim_exit", returncode=rc, signal=(-rc if rc < 0 else None), port_ever_open=first_open is not None)
    if sim_exit is not None and not connected:
        break
    if port_open():
        if first_open is None:
            first_open = round(time.time() - T0, 2); ev("port_open", seconds=first_open)
        if not connected:
            tries += 1
            try:
                c = airsim.MultirotorClient(timeout_value=15)   # NEW client for every try
                c.confirmConnection()
                connected = True; ev("confirmConnection_ok", tries=tries)
                try:
                    ev("server_version", v=str(c.getServerVersion()))
                except Exception as e:
                    ev("server_version_fail", err=repr(e)[:200])
                try:
                    c.enableApiControl(True); c.armDisarm(True); ev("armed")
                    st = c.getMultirotorState(); p = st.kinematics_estimated.position
                    ev("state_before", pos=[p.x_val, p.y_val, p.z_val], landed=str(st.landed_state))
                    c.takeoffAsync(timeout_sec=10).join(); ev("takeoff_done")
                    st = c.getMultirotorState(); p = st.kinematics_estimated.position
                    ev("state_after_takeoff", pos=[p.x_val, p.y_val, p.z_val])
                    c.moveByVelocityAsync(2, 0, 0, 2).join(); ev("move_done")
                    st = c.getMultirotorState(); p = st.kinematics_estimated.position
                    ev("state_after_move", pos=[p.x_val, p.y_val, p.z_val])
                    flight_done = True
                except Exception as e:
                    ev("flight_fail", err=repr(e)[:300], tb=traceback.format_exc()[-600:])
                break
            except Exception as e:
                ev("confirmConnection_fail", tries=tries, err=repr(e)[:300])
    time.sleep(0.4)
else:
    ev("cap_reached")
# let the simulator show whether it survives
time.sleep(3)
rc = sim.poll()
if rc is not None and sim_exit is None:
    sim_exit = rc; ev("sim_exit", returncode=rc, signal=(-rc if rc < 0 else None), port_ever_open=first_open is not None)
ev("summary", port_first_open_s=first_open, connected=connected, flight_done=flight_done, tries=tries, sim_exit=sim_exit)
try:
    os.killpg(sim.pid, signal.SIGKILL)
except Exception:
    pass
# evidence: every log file / crash file the simulator wrote
ue = out / "ue_logs"; ue.mkdir(exist_ok=True)
for root in (binp, home):
    for f in root.rglob("*"):
        try:
            if f.is_file() and f.stat().st_mtime >= marker - 1 and (f.suffix in (".log", ".txt", ".json", ".dmp", ".ini") or "Crashes" in str(f)) and f.stat().st_size < 20e6 and "Documents/AirSim/settings.json" not in str(f):
                dst = ue / (str(f).replace("/", "__"))
                shutil.copyfile(f, dst)
        except Exception:
            pass
(out / "summary.txt").write_text(json.dumps(events[-1], indent=1))
