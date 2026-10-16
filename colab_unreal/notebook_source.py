# %% [markdown]
# # Racing drone: first camera films of AirSim and Project AirSim
#
# This notebook makes the first camera films of two drone simulators: **AirSim** and **Project AirSim**. Both draw their pictures with Unreal Engine, which needs a graphics card. Colab's free T4 graphics card has one.
#
# **How to use it:** choose *Runtime > Change runtime type > T4 GPU > Save*, then *Runtime > Run all*. Allow access to Google Drive when it asks. Keep this tab open until the last cell prints **Done**. The run takes about 30 to 45 minutes.
#
# **What you get:** for each simulator, three films (the drone's own camera, a camera that follows the drone, and the drone's camera beside a map of the path), three sheets of 12 pictures from those films, a short summary in plain English, and all log files. They are saved on your Google Drive, in the folder `github/Comprehansive_cv/coursera_mirror/mirrors/Autonomous_Racing_Drone_Path/runs/2026-10-15_colab_unreal/<date_time>/`.
#
# If one simulator fails, the other one still runs. Every failure is written into that simulator's `summary.md` together with the last 60 lines of its logs.

# %% [markdown]
# ## Step 0. Settings and small helper functions (nothing to change here)

# %%
import datetime
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

TEST = os.environ.get("UNREAL_FILMS_TEST") == "1"          # test switch: no GPU, no Drive, simulators start with -nullrhi
WORK = Path(os.environ.get("UNREAL_FILMS_WORK", "/content"))  # downloads and unpacked simulators live here, never on Drive
STAMP = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d_%H%M")
DRIVE_RUN_ROOT = Path("/content/drive/MyDrive/github/Comprehansive_cv/coursera_mirror/mirrors/Autonomous_Racing_Drone_Path/runs/2026-10-15_colab_unreal")
LOCAL_RESULTS = Path(os.environ.get("UNREAL_FILMS_OUT", str(WORK / "results"))) / STAMP
DRIVE_RESULTS = None                                         # set in step 3
FPS = 15                                                     # pictures per simulated second in the films
RTF_PROJECT_AIRSIM = 1.0 if TEST else 0.1                    # Project AirSim clock speed (0.1 = ten times slower than real time)
MAX_TOTAL_MINUTES = 40                                       # no new simulator is started after this
T_START = time.time()
AIRSIM_URL = "https://github.com/microsoft/AirSim/releases/download/v1.8.1/Blocks.zip"
PASIM_URL = "https://github.com/iamaisim/ProjectAirSim/releases/download/v1.0.1/Blocks-Linux-1.0.1.zip"
IS_ROOT = hasattr(os, "geteuid") and os.geteuid() == 0
SIM_USER = "ue"                                              # Unreal Engine refuses to run as root, so the simulators run as this plain user
S = {"test": TEST, "stamp": STAMP, "vulkan": {}, "sims": {}}  # facts collected during the run


class StopRun(Exception):
    pass


def say(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def sh(cmd, log=None, timeout=None, env=None, cwd=None, quiet=True):
    """Run a shell command. Output goes to the log file (appended). Returns the exit code (124 on timeout)."""
    f = open(log, "ab") if log else subprocess.DEVNULL
    if log:
        f.write(f"\n$ {cmd}\n".encode())
        f.flush()
    try:
        r = subprocess.run(["bash", "-c", cmd], stdout=f, stderr=subprocess.STDOUT, timeout=timeout, env=env, cwd=cwd)
        return r.returncode
    except subprocess.TimeoutExpired:
        return 124
    finally:
        if log:
            f.close()


def out(cmd, timeout=120):
    """Run a shell command and return (exit code, text)."""
    try:
        r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr)
    except subprocess.TimeoutExpired:
        return 124, "timeout"


def tail(path, n=60):
    try:
        return "".join(Path(path).read_text(errors="replace").splitlines(keepends=True)[-n:])
    except Exception as e:  # noqa: BLE001
        return f"(could not read {path}: {e!r})"


def apt_install(pkgs, log):
    """Install packages; if the group fails, one by one so a missing name does not block the rest. Returns the names that failed."""
    rc = sh("DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends " + " ".join(pkgs), log)
    if rc == 0:
        return []
    failed = []
    for p in pkgs:
        if sh(f"DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends {p}", log) != 0:
            failed.append(p)
    return failed


_synced = {}


def sync_to_drive():
    """Copy new or changed result files to Drive (so a crash still leaves the logs there)."""
    if DRIVE_RESULTS is None or not LOCAL_RESULTS.exists():
        return
    for p in LOCAL_RESULTS.rglob("*"):
        if not p.is_file():
            continue
        try:
            st = p.stat()
            key = (st.st_size, int(st.st_mtime))
            if _synced.get(p) == key:
                continue
            dest = DRIVE_RESULTS / p.relative_to(LOCAL_RESULTS)
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, dest)
            _synced[p] = key
        except Exception:  # noqa: BLE001
            pass


def _sync_loop():
    while True:
        time.sleep(20)
        try:
            sync_to_drive()
        except Exception:  # noqa: BLE001
            pass


def kill_group(p):
    if p is None or p.poll() is not None:
        return
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(os.getpgid(p.pid), sig)
        except Exception:  # noqa: BLE001
            pass
        try:
            p.wait(timeout=8)
            return
        except Exception:  # noqa: BLE001
            pass


LOCAL_RESULTS.mkdir(parents=True, exist_ok=True)
(WORK / "helpers").mkdir(parents=True, exist_ok=True)
say("test mode:", TEST, "| working folder:", WORK, "| results (local):", LOCAL_RESULTS)

# %% [markdown]
# ## Step 1. Check that this computer has a graphics card

# %%
try:
    if TEST:
        say("TEST MODE: the graphics card check is skipped.")
        S["gpu"] = {"name": "none (test mode)", "driver": None}
    else:
        rc, txt = out("nvidia-smi")
        print(txt)
        if rc != 0 or "NVIDIA" not in txt:
            raise StopRun("There is no graphics card. Choose Runtime > Change runtime type > T4 GPU, then Run all again.")
        _, q = out("nvidia-smi --query-gpu=name,driver_version --format=csv,noheader")
        name, driver = [x.strip() for x in q.strip().splitlines()[0].split(",")]
        S["gpu"] = {"name": name, "driver": driver}
        say(f"Graphics card found: {name}, driver {driver}.")
except StopRun as e:
    print("\n" + str(e))
    raise

# %% [markdown]
# ## Step 2. Turn on the graphics card's Vulkan driver (and install a few helper libraries)
# Colab ships the card's compute driver but not its drawing driver. The simulators need the drawing driver (Vulkan). We install the package `libnvidia-gl-<N>` whose number matches the running driver, then check with `vulkaninfo`.

# %%
def step_vulkan():
    log = LOCAL_RESULTS / "setup_vulkan.log"
    info = {"nvidia_vulkan_device_found": False, "installed": []}
    sh("apt-get update -qq", log)
    libs = ["vulkan-tools", "libvulkan1", "xvfb", "xauth", "unzip", "curl", "git", "libgl1", "libglu1-mesa", "libsdl2-2.0-0", "libxi6",
            "libxrandr2", "libxcursor1", "libxinerama1", "libnss3", "libasound2t64", "libasound2", "libgl1-mesa-dri"]
    if shutil.which("ffmpeg") is None:
        libs.append("ffmpeg")
    info["library_install_failed"] = apt_install(libs, log)   # one of libasound2t64 / libasound2 is expected to fail
    if TEST:
        info["note"] = "TEST MODE: the graphics driver is not installed; the simulators start with -nullrhi"
        S["vulkan"] = info
        return info
    driver = S["gpu"]["driver"]
    major = int(driver.split(".")[0])
    _, names = out("apt-cache pkgnames libnvidia-gl-")
    avail = sorted({int(m.group(1)) for l in names.split() for m in [re.fullmatch(r"libnvidia-gl-(\d+)", l.strip())] if m})
    info["driver"], info["driver_major"], info["packages_available"] = driver, major, avail
    order = sorted(avail, key=lambda v: (abs(v - major), v))
    if major not in avail:
        say(f"apt has no libnvidia-gl-{major}; the nearest available are {order[:3]}")
    for cand in order[:3]:
        _, mad = out(f"apt-cache madison libnvidia-gl-{cand}")
        pins = [l.split("|")[1].strip() for l in mad.splitlines() if len(l.split("|")) > 2 and l.split("|")[1].strip().startswith(driver)]
        tries = ([f"libnvidia-gl-{cand}={pins[0]}"] if pins else []) + [f"libnvidia-gl-{cand}"]
        done = False
        for t in tries:
            if sh(f"DEBIAN_FRONTEND=noninteractive apt-get install -y -qq {t}", log) == 0:
                info["installed"].append(t)
                done = True
                break
        if done:
            break
    _, dpk = out("dpkg -l 'libnvidia-gl-*' | grep '^ii'")
    info["dpkg"] = dpk.strip()
    say("installed:", info["installed"] or "NOTHING", "|", dpk.strip() or "no libnvidia-gl package")

    def check():
        _, vk = out("vulkaninfo --summary", timeout=120)
        (LOCAL_RESULTS / "vulkaninfo_summary.txt").write_text(vk)
        return bool(re.search(r"driverName\s*=\s*NVIDIA", vk) or re.search(r"deviceName\s*=.*(NVIDIA|Tesla)", vk)), vk

    found, vk = check()
    icd = next((p for p in ("/usr/share/vulkan/icd.d/nvidia_icd.json", "/etc/vulkan/icd.d/nvidia_icd.json") if Path(p).is_file()), None)
    if not found and icd is None:
        libs_found = out("ldconfig -p | grep -i libGLX_nvidia")[1].strip()
        if libs_found:
            Path("/usr/share/vulkan/icd.d").mkdir(parents=True, exist_ok=True)
            Path("/usr/share/vulkan/icd.d/nvidia_icd.json").write_text(
                json.dumps({"file_format_version": "1.0.0", "ICD": {"library_path": "libGLX_nvidia.so.0", "api_version": "1.3.0"}}))
            icd = "/usr/share/vulkan/icd.d/nvidia_icd.json"
            info["wrote_icd_file_by_hand"] = True
            found, vk = check()
    info["nvidia_icd_file"] = icd
    info["nvidia_vulkan_device_found"] = found
    info["vulkaninfo_devices"] = [l.strip() for l in vk.splitlines() if "deviceName" in l or "driverName" in l]
    say("Vulkan lists the NVIDIA card:", found, info["vulkaninfo_devices"])
    S["vulkan"] = info
    return info


try:
    step_vulkan()
except Exception:  # noqa: BLE001
    S["vulkan"]["error"] = traceback.format_exc()
    say("Step 2 had a problem; the run goes on, but the simulators may fail:\n" + S["vulkan"]["error"][-800:])
(LOCAL_RESULTS / "vulkan.json").write_text(json.dumps(S["vulkan"], indent=2, default=str))

# %% [markdown]
# ## Step 3. Connect Google Drive and make the results folder
# Colab will ask you to allow access to your Google Drive. Please click **Connect to Google Drive** and choose your account.

# %%
if TEST:
    say("TEST MODE: Google Drive is skipped; results stay in", LOCAL_RESULTS)
else:
    from google.colab import drive  # type: ignore

    drive.mount("/content/drive")
    DRIVE_RESULTS = DRIVE_RUN_ROOT / STAMP
    for sub in ("airsim", "project_airsim"):
        (DRIVE_RESULTS / sub).mkdir(parents=True, exist_ok=True)
    say("Results folder on Drive:", DRIVE_RESULTS)
    threading.Thread(target=_sync_loop, daemon=True).start()
for sub in ("airsim", "project_airsim"):
    (LOCAL_RESULTS / sub).mkdir(parents=True, exist_ok=True)
S["drive_results"] = str(DRIVE_RESULTS) if DRIVE_RESULTS else None
sync_to_drive()

# %% [markdown]
# ## Step 3b. Write the helper programs (the flight route, the film maker, the two simulator clients)

# %%
HELPERS = {
    "route.py": r'''@@route.py@@''',
    "unreal_films.py": r'''@@unreal_films.py@@''',
    "airsim_flight.py": r'''@@airsim_flight.py@@''',
    "projectairsim_flight.py": r'''@@projectairsim_flight.py@@''',
}
for _name, _src in HELPERS.items():
    (WORK / "helpers" / _name).write_text(_src)
sys.path.insert(0, str(WORK / "helpers"))
say("helper programs written to", WORK / "helpers")

# %% [markdown]
# ## Common tools for both simulators (starting a simulator, trying graphics modes, making the films)

# %%
def ensure_sim_user():
    if not IS_ROOT:
        return Path.home()
    if sh(f"id {SIM_USER}") != 0:
        sh(f"useradd -m -s /bin/bash {SIM_USER}")
    return Path(f"/home/{SIM_USER}")


def make_venv(name, py, packages_cmds, log):
    """A separate Python environment made with uv, so old simulator clients do not clash with Colab's newer Python."""
    venv = WORK / "venvs" / name
    if (venv / "bin" / "python").exists():
        return venv
    venv.parent.mkdir(parents=True, exist_ok=True)
    sh(f"{sys.executable} -m pip install -q uv", log)
    rc = sh(f"{sys.executable} -m uv venv --seed --python {py} {venv}", log, timeout=900)
    if rc != 0:
        raise RuntimeError(f"could not make the Python {py} environment (see {log})")
    for c in packages_cmds:
        rc = sh(c.replace("PIP", f"{venv}/bin/pip"), log, timeout=1800)
        if rc != 0:
            raise RuntimeError(f"pip step failed: {c} (see {log})")
    return venv


def download_and_unpack(url, dest, log, zip_name):
    if dest.exists() and any(dest.iterdir()):
        return
    dest.mkdir(parents=True, exist_ok=True)
    z = WORK / "downloads" / zip_name
    z.parent.mkdir(parents=True, exist_ok=True)
    t = time.time()
    if sh(f"curl -fL --retry 5 -sS -o {z} {url}", log, timeout=1800) != 0:
        raise RuntimeError(f"download failed: {url}")
    say(f"downloaded {zip_name}: {z.stat().st_size / 1e9:.2f} GB in {time.time() - t:.0f} s")
    if sh(f"unzip -q -o {z} -d {dest}", log, timeout=1800) != 0:
        raise RuntimeError(f"unzip failed: {z}")
    z.unlink()
    if IS_ROOT:
        sh(f"chown -R {SIM_USER}:{SIM_USER} {dest}", log)
    sh(f"find {dest} -type f -path '*Binaries/Linux/*' -exec chmod +x {{}} + ; find {dest} -maxdepth 3 -name '*.sh' -exec chmod +x {{}} +", log)


def find_start_script(dest):
    r = subprocess.run(["bash", "-c", f"find {dest} -maxdepth 3 -name '*.sh' | head -n 1"], capture_output=True, text=True)
    p = r.stdout.strip()
    if not p:
        raise RuntimeError(f"no start script (.sh) found in {dest}")
    return p


VARIANTS = {
    "vulkan": "-RenderOffScreen -ResX=640 -ResY=480 -nosound -windowed",
    "opengl4": "-RenderOffScreen -opengl4 -ResX=640 -ResY=480 -nosound -windowed",
    "opengl": "-RenderOffScreen -opengl -ResX=640 -ResY=480 -nosound -windowed",
    "nullrhi": "-nullrhi -nosound",
}
_xvfb = {"p": None}


def ensure_xvfb():
    if _xvfb["p"] is None or _xvfb["p"].poll() is not None:
        _xvfb["p"] = subprocess.Popen(["Xvfb", ":99", "-screen", "0", "1280x1024x24", "+extension", "GLX", "+render", "-noreset"],
                                      stdout=open(LOCAL_RESULTS / "xvfb.log", "ab"), stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(3)


def start_sim(script, args, sim_home, log):
    env_pairs = {"HOME": str(sim_home), "DISPLAY": ":99", "SDL_AUDIODRIVER": "dummy"}
    icd = S["vulkan"].get("nvidia_icd_file")
    if icd and S["vulkan"].get("nvidia_vulkan_device_found"):
        env_pairs["VK_ICD_FILENAMES"] = icd   # only the graphics card, never the software fallback
    inner = f"ulimit -c 0; exec bash {script} {args}"
    if IS_ROOT:
        cmd = ["runuser", "-u", SIM_USER, "--", "env"] + [f"{k}={v}" for k, v in env_pairs.items()] + ["bash", "-c", inner]
        env = None
    else:
        cmd = ["bash", "-c", inner]
        env = {**os.environ, **env_pairs}
    f = open(log, "ab")
    return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, env=env, start_new_session=True, cwd=str(Path(script).parent))


def copy_unreal_logs(root, since, res_dir, prefix):
    """Unreal writes its own .log files inside the simulator folder; keep the ones written during this attempt."""
    n = 0
    for p in Path(root).rglob("*.log"):
        try:
            if p.stat().st_mtime >= since - 1:
                shutil.copyfile(p, res_dir / f"unreal_{prefix}__{p.name}")
                n += 1
        except Exception:  # noqa: BLE001
            pass
    return n


def try_variants(kind, script, sim_root, res_dir, client_cmd_for, variants, prepare_variant):
    """Start the simulator in each graphics mode until the flight program gets pictures and flies. Returns the list of attempts."""
    attempts = []
    for v in variants:
        att = {"variant": v, "args": VARIANTS[v]}
        attempts.append(att)
        run_dir = WORK / "runs" / kind / v
        shutil.rmtree(run_dir, ignore_errors=True)
        run_dir.mkdir(parents=True, exist_ok=True)
        say(f"[{kind}] starting the simulator with: {VARIANTS[v]}")
        prepare_variant(v)
        t0 = time.time()
        sim = start_sim(script, VARIANTS[v], ensure_sim_user(), res_dir / f"sim_{v}.log")
        clog = open(res_dir / f"client_{v}.log", "ab")
        client = subprocess.Popen(client_cmd_for(run_dir), stdout=clog, stderr=subprocess.STDOUT, start_new_session=True)
        died_at = None
        last_note = 0
        while client.poll() is None:
            time.sleep(2)
            if sim.poll() is not None:
                died_at = died_at or time.time()
                if time.time() - died_at > 12:
                    att["sim_died"] = True
                    say(f"[{kind}] the simulator process ended (exit code {sim.returncode}); stopping the flight program")
                    kill_group(client)
                    break
            if time.time() - last_note > 60:
                last_note = time.time()
                say(f"[{kind}] {v}: running for {time.time() - t0:.0f} s ...")
                sync_to_drive()
        att["client_rc"] = client.poll()
        att["sim_exit_code"] = sim.poll()
        att["seconds"] = round(time.time() - t0, 1)
        clog.close()
        att["unreal_logs_copied"] = copy_unreal_logs(sim_root, t0, res_dir, v)
        kill_group(sim)
        fj = run_dir / "flight.json"
        if fj.exists():
            shutil.copyfile(fj, res_dir / f"flight_{v}.json")
            att["stage"] = json.loads(fj.read_text()).get("stage")
        att["run_dir"] = str(run_dir)
        sync_to_drive()
        say(f"[{kind}] {v}: flight program exit code {att['client_rc']}, stage {att.get('stage')}, {att['seconds']} s")
        if att["client_rc"] == 0 or (att["client_rc"] == 1 and att.get("stage") == "flight_error"):
            att["used"] = True
            break
        time.sleep(3)
    return attempts


def color_note(rgb):
    if not rgb:
        return "no colour check (no picture)"
    r, g, b = rgb
    if b > r + 5:
        return f"the top of the front picture is bluish (red {r:.0f}, blue {b:.0f}), so the colour order looks right"
    return f"the top of the front picture is NOT bluish (red {r:.0f}, blue {b:.0f}); if the sky looks orange in the films, red and blue are swapped"


def finish_sim(kind, title, res_dir, attempts, extra_facts, camera_notes):
    """Make the films, contact sheets, summary.md and result.json for one simulator. Never claims a film that was not written."""
    import unreal_films

    used = next((a for a in attempts if a.get("used")), None)
    last = used or (attempts[-1] if attempts else None)
    result = {"simulator": title, "attempts": attempts, "facts": extra_facts, "films": {}, "problems": [], "test_mode": TEST}
    flight, info, quality = {}, {}, {}
    if last and last.get("run_dir"):
        run_dir = Path(last["run_dir"])
        try:
            flight = json.loads((run_dir / "flight.json").read_text())
        except Exception:  # noqa: BLE001
            pass
        tj = run_dir / "track.json"
        if tj.exists() and flight.get("stage") in ("done", "flight_error"):
            track = json.loads(tj.read_text())
            info = unreal_films.make_films(res_dir, run_dir / "frames", track, res_dir / "ffmpeg.log")
            quality = unreal_films.film_quality(res_dir, info)
            shutil.copyfile(tj, res_dir / "track.json")
    result["flight"] = flight
    result["film_info"] = info
    result["film_quality"] = quality
    films = list(info.get("films", {}))
    good = bool(films) and len(films) == 3 and flight.get("stage") == "done" and all(
        v.get("pictures_decoded", 0) >= 50 for v in info["films"].values()) and all(
        all(x > 5 for x in q.get("stds", [0])) for q in quality.values())
    if TEST:
        verdict = "TEST RUN WITHOUT PICTURES: the flight route was flown; pictures and films are not expected here."
    elif good:
        verdict = "WORKED: all three films were made."
    elif films:
        verdict = "PARTLY WORKED: some films were made, but not all checks passed (see below)."
    else:
        verdict = "DID NOT WORK: no film was made."
    result["verdict"] = verdict
    t = flight.get("timing", {})
    lines = [f"# {title}: first camera films", "", f"**Result: {verdict}**", ""]
    lines += ["## In plain English", ""]
    if used:
        lines.append(f"- The simulator drew its pictures in graphics mode **{used['variant']}** (modes tried, in order: "
                     + ", ".join(f"{a['variant']} ({'worked' if a.get('used') else 'failed'})" for a in attempts) + ").")
    else:
        lines.append("- No graphics mode gave pictures. Modes tried: " + (", ".join(a["variant"] for a in attempts) or "none") + ".")
    lines.append(f"- The graphics card's Vulkan driver: {'found' if S['vulkan'].get('nvidia_vulkan_device_found') else 'NOT found (only the slow software driver)'}"
                 + (f" ({S['gpu']['name']}, driver {S['gpu']['driver']})." if S.get('gpu') else "."))
    for n in camera_notes:
        lines.append(f"- {n}")
    if flight.get("probe", {}).get("fpv_top_rows_mean_rgb") or flight.get("probe", {}).get("front_top_rows_mean_rgb"):
        rgb = flight["probe"].get("fpv_top_rows_mean_rgb") or flight["probe"].get("front_top_rows_mean_rgb")
        lines.append(f"- Colour check: {color_note(rgb)}.")
    if t:
        lines.append(f"- The flight lasted {t.get('sim_s')} simulated seconds and took {t.get('wall_s')} real seconds. "
                     f"Simulated seconds per real second (the real-time factor): **{t.get('real_time_factor')}**.")
        lines.append(f"- Pictures taken: {t.get('pictures_taken')}. Pictures per real second (both cameras together): **{t.get('pictures_per_wall_second')}**. "
                     f"Camera failures: {t.get('camera_failures')}.")
    if info:
        lines.append(f"- The films are {(info.get('film_seconds') or 0):.1f} s long at {info.get('fps')} pictures per second of simulated time. "
                     f"Different pictures used: front camera {info.get('fpv_different_pictures_in_film')} "
                     f"({(info.get('fpv_pictures_per_sim_second') or 0):.1f} per simulated second), "
                     f"follow camera {info.get('chase_different_pictures_in_film')} ({(info.get('chase_pictures_per_sim_second') or 0):.1f} per simulated second). "
                     "If a number is below 15, some pictures are shown twice.")
    if flight.get("start"):
        lines.append(f"- Route: take off to about 3 m, one figure eight (12 m long, 6 m wide) with the nose along the direction of flight, then hover. "
                     f"Collision checks that reported a hit: {t.get('collision_checks_with_hit', 'not measured')}.")
    lines += ["", "## Files in this folder", ""]
    for name, f in info.get("films", {}).items():
        lines.append(f"- `{name}`: {f['pictures_decoded']} pictures, {f['bytes'] / 1e6:.1f} MB; contact sheet `{f['file']}`")
    if not info.get("films"):
        lines.append("- No films were written.")
    lines.append("- `summary.md` (this file), `result.json`, `track.json` (position of the drone for every picture), `flight_<mode>.json`, "
                 "and all logs: `sim_<mode>.log` (what the simulator printed), `client_<mode>.log` (the flight program), `unreal_<mode>__*.log` (the Unreal Engine log).")
    if quality:
        lines += ["", "## Film checks (12 pictures of each film)", ""]
        for n, q in quality.items():
            lines.append(f"- `{n}`: picture contrast (standard deviation, each should be above 5) {q.get('stds')}; "
                         f"mean change between neighbouring samples {q.get('neighbour_mean_abs_diff')}")
    problems = list(info.get("problems", []))
    if flight.get("traceback"):
        problems.append("flight program error: " + flight["traceback"].strip().splitlines()[-1])
    if flight.get("probe", {}).get("last_errors"):
        problems.append("picture errors: " + " | ".join(flight["probe"]["last_errors"]))
    if flight.get("stop_reason"):
        lines.append(f"\nThe flight stopped because: {flight['stop_reason']}.")
    if problems:
        lines += ["", "## Problems", ""] + [f"- {p}" for p in problems]
    if not good:
        lines += ["", "## Last 60 lines of the logs (because this did not fully work)", ""]
        for a in attempts:
            for pat in (f"client_{a['variant']}.log", f"sim_{a['variant']}.log"):
                lines += [f"### {pat}", "```", tail(res_dir / pat, 60).rstrip(), "```", ""]
            for p in sorted(res_dir.glob(f"unreal_{a['variant']}__*.log"))[:1]:
                lines += [f"### {p.name}", "```", tail(p, 60).rstrip(), "```", ""]
        for p in ("setup.log",):
            if (res_dir / p).exists():
                lines += [f"### {p}", "```", tail(res_dir / p, 60).rstrip(), "```", ""]
    (res_dir / "summary.md").write_text("\n".join(lines) + "\n")
    (res_dir / "result.json").write_text(json.dumps(result, indent=2, default=str))
    S["sims"][kind] = {"verdict": verdict, "films": films, "folder": str(DRIVE_RESULTS / kind if DRIVE_RESULTS else res_dir), "variant": used["variant"] if used else None}
    sync_to_drive()
    say(f"[{kind}] {verdict}")


def fail_sim(kind, title, res_dir, err):
    """A step before the flight failed (download, install, ...): write the summary with the last 60 lines of the logs."""
    lines = [f"# {title}: first camera films", "", "**Result: DID NOT WORK: the set-up stopped before the simulator could fly.**", "",
             f"Reason: `{err.strip().splitlines()[-1] if err.strip() else 'unknown'}`", "", "## Full error", "```", err.strip(), "```", "",
             "## Last 60 lines of the logs", ""]
    for p in sorted(res_dir.glob("*.log"))[:8]:
        lines += [f"### {p.name}", "```", tail(p, 60).rstrip(), "```", ""]
    (res_dir / "summary.md").write_text("\n".join(lines) + "\n")
    (res_dir / "result.json").write_text(json.dumps({"simulator": title, "verdict": "DID NOT WORK", "error": err, "test_mode": TEST}, indent=2))
    S["sims"][kind] = {"verdict": "DID NOT WORK (set-up)", "films": [], "folder": str(DRIVE_RESULTS / kind if DRIVE_RESULTS else res_dir), "variant": None}
    sync_to_drive()
    say(f"[{kind}] did not work: {err.strip().splitlines()[-1] if err.strip() else ''}")


def variants_to_try():
    return ["nullrhi"] if TEST else ["vulkan", "opengl4", "opengl"]


# %% [markdown]
# ## Step 4. AirSim: install, start, fly, record
# Downloads AirSim 1.8.1 (the "Blocks" world, Unreal Engine 4.27), installs its Python client in a Python 3.9 environment, writes `settings.json` (a camera on the drone's nose called `front`, and an external camera called `Chase` that is moved behind the drone at every step), then flies. The simulation is paused between steps, so the film time matches the simulated time even though drawing is slow.

# %%
AIRSIM_RES = LOCAL_RESULTS / "airsim"
AIRSIM_STATE = {}
try:
    if (time.time() - T_START) / 60 > MAX_TOTAL_MINUTES:
        raise RuntimeError("skipped: the time limit for starting a new simulator was reached")
    say("=== AirSim ===")
    log = AIRSIM_RES / "setup.log"
    ensure_xvfb()
    home = ensure_sim_user()
    venv = make_venv("airsim", "3.9", [
        "PIP install -q --upgrade pip",
        "PIP install -q setuptools wheel msgpack-rpc-python numpy pillow opencv-python-headless",
        "PIP install -q --no-deps --no-build-isolation airsim==1.8.1",
        "PIP list",
    ], log)
    sh(f"{venv}/bin/python -c \"import airsim; print('airsim client', airsim.__version__)\"", log)
    sim_dir = WORK / "sims" / "airsim" / "Blocks"
    download_and_unpack(AIRSIM_URL, sim_dir, log, "airsim_blocks.zip")
    script = find_start_script(sim_dir)
    say("AirSim start script:", script)
    shot = {"CaptureSettings": [{"ImageType": 0, "Width": 640, "Height": 480, "FOV_Degrees": 90}]}
    settings = {
        "SettingsVersion": 1.2, "SimMode": "Multirotor", "ClockSpeed": 1,
        "CameraDefaults": shot,
        "Vehicles": {"Drone1": {"VehicleType": "SimpleFlight", "AutoCreate": True,
                                "Cameras": {"front": {**shot, "X": 0.3, "Y": 0.0, "Z": 0.0, "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0}}}},
        "ExternalCameras": {"Chase": {**shot, "X": -6.0, "Y": 0.0, "Z": -2.5, "Pitch": -22.0, "Roll": 0.0, "Yaw": 0.0}},
    }

    def prepare_airsim(variant):
        d = home / "Documents" / "AirSim"
        d.mkdir(parents=True, exist_ok=True)
        (d / "settings.json").write_text(json.dumps(settings, indent=2))
        if IS_ROOT:
            sh(f"chown -R {SIM_USER}:{SIM_USER} {home}")
        shutil.copyfile(d / "settings.json", AIRSIM_RES / "settings_used.json")

    def airsim_client_cmd(run_dir):
        cmd = [str(venv / "bin" / "python"), str(WORK / "helpers" / "airsim_flight.py"), "--out", str(run_dir), "--fps", str(FPS)]
        return cmd + (["--test", "--picture-wait", "15", "--connect-wait", "240"] if TEST else ["--picture-wait", "90", "--connect-wait", "240"])

    attempts = try_variants("airsim", script, sim_dir, AIRSIM_RES, airsim_client_cmd, variants_to_try(), prepare_airsim)
    AIRSIM_STATE["attempts"] = attempts
    finish_sim("airsim", "AirSim 1.8.1 (Blocks, Unreal Engine 4.27)", AIRSIM_RES, attempts,
               {"settings": settings},
               ["Cameras: `front` is fixed on the drone's nose and looks along it. `Chase` is an external camera that is moved at every step to "
                "6 m behind the drone (along its heading) and 2.5 m above it, looking at it (it follows the drone)."])
except Exception:  # noqa: BLE001
    err = traceback.format_exc()
    say("AirSim stopped:", err.strip().splitlines()[-1])
    fail_sim("airsim", "AirSim 1.8.1 (Blocks, Unreal Engine 4.27)", AIRSIM_RES, err)

# %% [markdown]
# ## Step 5. Project AirSim: install, start, fly, record
# Same idea with Project AirSim 1.0.1 (the "Blocks" world, Unreal Engine 5) and its own Python client (Python 3.10 environment). We add a `FrontCamera` on the drone's nose to a copy of the sample scene. The scene's own `Chase` camera follows the drone. The simulation clock runs ten times slower than real time, so that every picture can be drawn; the film time is the simulated time.

# %%
PASIM_RES = LOCAL_RESULTS / "project_airsim"
PASIM_STATE = {}
try:
    if (time.time() - T_START) / 60 > MAX_TOTAL_MINUTES:
        raise RuntimeError("skipped: the time limit for starting a new simulator was reached")
    say("=== Project AirSim ===")
    log = PASIM_RES / "setup.log"
    ensure_xvfb()
    home = ensure_sim_user()
    venv = make_venv("pasim", "3.10", [
        "PIP install -q --upgrade pip",
        "PIP install -q projectairsim==1.0.1 numpy pillow",
        "PIP list",
    ], log)
    sh(f"{venv}/bin/python -c \"import projectairsim; print('projectairsim client ok')\"", log)
    src = WORK / "ProjectAirSim_src"
    if not src.exists():
        if sh(f"git clone --depth 1 --branch v1.0.1 https://github.com/iamaisim/ProjectAirSim.git {src}", log, timeout=900) != 0:
            raise RuntimeError("git clone of Project AirSim failed")
    sim_config = src / "client" / "python" / "example_user_scripts" / "sim_config"
    import projectairsim_flight

    facts = projectairsim_flight.prepare_configs(sim_config, RTF_PROJECT_AIRSIM)
    say("scene prepared:", facts)
    sim_dir = WORK / "sims" / "project_airsim" / "Blocks"
    download_and_unpack(PASIM_URL, sim_dir, log, "pasim_blocks.zip")
    script = find_start_script(sim_dir)
    say("Project AirSim start script:", script)

    def pasim_client_cmd(run_dir):
        cmd = [str(venv / "bin" / "python"), str(WORK / "helpers" / "projectairsim_flight.py"), "--out", str(run_dir), "--sim-config", str(sim_config),
               "--fps", str(FPS)]
        return cmd + (["--test", "--picture-wait", "15", "--connect-wait", "240", "--max-wall", "600"] if TEST else ["--picture-wait", "90", "--connect-wait", "300"])

    attempts = try_variants("project_airsim", script, sim_dir, PASIM_RES, pasim_client_cmd, variants_to_try(), lambda v: None)
    PASIM_STATE["attempts"] = attempts
    flight = {}
    try:
        used = next((a for a in attempts if a.get("used")), attempts[-1])
        flight = json.loads((Path(used["run_dir"]) / "flight.json").read_text())
    except Exception:  # noqa: BLE001
        pass
    notes = ["Cameras: " + (flight.get("front_note") or "front camera: not known (no flight data)") + ". The follow camera is the scene's own `Chase` camera "
             "(10 m behind the drone, 1 m above, it turns with the drone's heading)."]
    finish_sim("project_airsim", "Project AirSim 1.0.1 (Blocks, Unreal Engine 5)", PASIM_RES, attempts, facts, notes)
except Exception:  # noqa: BLE001
    err = traceback.format_exc()
    say("Project AirSim stopped:", err.strip().splitlines()[-1])
    fail_sim("project_airsim", "Project AirSim 1.0.1 (Blocks, Unreal Engine 5)", PASIM_RES, err)

# %% [markdown]
# ## Step 6. Summary

# %%
kill_group(_xvfb["p"])
minutes = (time.time() - T_START) / 60
top = ["# Colab run: AirSim and Project AirSim films", "", f"Run time: {minutes:.1f} minutes. Test mode: {TEST}.", ""]
print(f"\nThe run took {minutes:.1f} minutes.\n")
for kind in ("airsim", "project_airsim"):
    s = S["sims"].get(kind, {"verdict": "did not run", "films": [], "folder": "-"})
    print(f"{kind}:\n  result: {s['verdict']}\n  graphics mode that worked: {s.get('variant')}\n  folder: {s['folder']}")
    print("  films: " + (", ".join(s["films"]) if s["films"] else "none"))
    top += [f"## {kind}", f"- Result: {s['verdict']}", f"- Graphics mode that worked: {s.get('variant')}", f"- Folder: `{s['folder']}`",
            "- Films: " + (", ".join(s["films"]) if s["films"] else "none"), ""]
(LOCAL_RESULTS / "run_summary.md").write_text("\n".join(top) + "\n")
(LOCAL_RESULTS / "run.json").write_text(json.dumps(S, indent=2, default=str))
sync_to_drive()
print("\nAll results folder:", DRIVE_RESULTS or LOCAL_RESULTS)
print("Done")
