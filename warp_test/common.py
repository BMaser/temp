"""Shared helpers for the WARP test: machine report, Unreal logs, picture statistics, films."""
import re
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np

PASS_MIN_STD = 5.0
WARP_NAME = "Microsoft Basic Render Driver"


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def machine_report(out: Path):
    """Write machine.txt: CPU, memory, video controllers, dxdiag display section."""
    out.mkdir(parents=True, exist_ok=True)
    ps = (
        "Get-CimInstance Win32_ComputerSystem | Format-List Manufacturer,Model,TotalPhysicalMemory,NumberOfLogicalProcessors; "
        "Get-CimInstance Win32_Processor | Format-List Name,NumberOfCores,NumberOfLogicalProcessors; "
        "Get-CimInstance Win32_OperatingSystem | Format-List Caption,Version,BuildNumber; "
        "Get-CimInstance Win32_VideoController | Format-List Name,DriverVersion,AdapterRAM,Status,PNPDeviceID; "
        "Get-PSDrive -PSProvider FileSystem | Format-Table Name,Used,Free"
    )
    r = subprocess.run(["powershell", "-NoProfile", "-Command", ps], capture_output=True, text=True)
    text = "== PowerShell query ==\n" + r.stdout + r.stderr
    dx = out / "dxdiag.txt"
    try:
        subprocess.run(["dxdiag", "/t", str(dx)], timeout=120)
        for _ in range(60):
            if dx.exists() and dx.stat().st_size > 1000:
                break
            time.sleep(1)
        text += "\n== dxdiag ==\n"
        if dx.exists():
            t = dx.read_text(errors="replace")
            m = re.search(r"Display Devices.*?(?=\n-{10,}\s*\n\s*(Sound Devices|Sound|Input))", t, re.S)
            text += m.group(0) if m else t[:4000]
    except Exception as e:
        text += f"\ndxdiag failed: {e}\n"
    (out / "machine.txt").write_text(text)
    log("machine report written")
    return text


def kill_names(names):
    for n in names:
        subprocess.run(["taskkill", "/F", "/T", "/IM", n], capture_output=True)


def tail(path, n=40):
    try:
        return "\n".join(Path(path).read_text(errors="replace").splitlines()[-n:])
    except Exception:
        return ""


def collect_logs(search_roots, dest: Path, tag: str, since: float):
    """Copy every *.log written since `since` (epoch seconds) under the roots to dest/<tag>__<name>. Never follows links."""
    import os
    dest.mkdir(parents=True, exist_ok=True)
    got, seen = [], set()
    for root in search_roots:
        root = str(root)
        if not os.path.isdir(root):
            continue
        for dp, dn, fn in os.walk(root, followlinks=False, onerror=lambda e: None):
            dn[:] = [d for d in dn if d not in ("Application Data", "node_modules") and not os.path.islink(os.path.join(dp, d))]
            for f in fn:
                if not f.lower().endswith(".log"):
                    continue
                p = os.path.join(dp, f)
                try:
                    if os.stat(p).st_mtime >= since - 5 and p not in seen:
                        seen.add(p)
                        q = dest / f"{tag}__{f}"
                        shutil.copy(p, q)
                        got.append(q)
                except Exception:
                    pass
    return got


def log_facts(paths):
    """Facts from Unreal logs: does it name the WARP device, RHI lines, error lines."""
    facts = {"names_warp": False, "rhi_lines": [], "error_lines": []}
    for p in paths:
        t = Path(p).read_text(errors="replace")
        if WARP_NAME.lower() in t.lower():
            facts["names_warp"] = True
        for line in t.splitlines():
            l = line.lower()
            if any(k in l for k in ("d3d12", "adapter", "dxgi", "warp", "vulkan", "rhi")) and len(facts["rhi_lines"]) < 30:
                facts["rhi_lines"].append(line.strip()[:240])
            if any(k in l for k in ("fatal", "crash", "assertion", "unhandled", "device removed", "error:")) and len(facts["error_lines"]) < 30:
                facts["error_lines"].append(line.strip()[:240])
    return facts


def image_stats(a):
    a = np.asarray(a)
    return {"std": float(a.std()), "mean": float(a.mean()), "min": int(a.min()), "max": int(a.max()), "shape": list(a.shape)}


def save_png(a_rgb, path):
    import cv2
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), np.ascontiguousarray(a_rgb)[:, :, ::-1])


def save_jpg(a_rgb, path):
    import cv2
    cv2.imwrite(str(path), np.ascontiguousarray(a_rgb)[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 92])


def ffmpeg_exe():
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


def encode(frames_dir: Path, pattern: str, out_mp4: Path, fps=10):
    r = subprocess.run([ffmpeg_exe(), "-y", "-framerate", str(fps), "-i", str(frames_dir / pattern), "-c:v", "libx264",
                        "-pix_fmt", "yuv420p", "-crf", "20", str(out_mp4)], capture_output=True, text=True)
    if r.returncode != 0:
        log("ffmpeg failed: " + r.stderr[-600:])
    return r.returncode == 0


def contact_sheet(frame_paths, out_png, n=12, cols=4):
    import cv2
    if not frame_paths:
        return False
    idx = np.linspace(0, len(frame_paths) - 1, min(n, len(frame_paths))).round().astype(int)
    tiles = []
    for i in idx:
        im = cv2.imread(str(frame_paths[i]))
        im = cv2.resize(im, (320, 240))
        cv2.putText(im, f"frame {i}", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        tiles.append(im)
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[r:r + cols]) for r in range(0, len(tiles), cols)]
    cv2.imwrite(str(out_png), np.vstack(rows))
    return True


def map_panel(track, square, k, size=480):
    """Top-down map (north up, east right) of the flight so far. track = list of (north, east) in metres."""
    import cv2
    img = np.full((size, size, 3), 30, np.uint8)
    allpts = np.array(list(track) + list(square))
    lo, hi = allpts.min(0) - 3, allpts.max(0) + 3
    span = float(max(hi - lo)) or 1.0

    def px(p):
        n, e = p
        return int((e - lo[1]) / span * (size - 1)), int(size - 1 - (n - lo[0]) / span * (size - 1))

    for i in range(len(square)):
        cv2.line(img, px(square[i]), px(square[(i + 1) % len(square)]), (90, 90, 90), 1)
    pts = [px(p) for p in track[:k + 1]]
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, (0, 200, 255), 2)
    cv2.circle(img, pts[-1], 6, (0, 0, 255), -1)
    cv2.putText(img, "map (north up)", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return img


def make_films(rec_dir: Path, track, square, out_dir: Path, fps=10):
    """rec_dir holds fpv_00000.jpg and chase_00000.jpg. Writes fpv.mp4, chase.mp4, fpv_and_map.mp4 and contact sheets."""
    import cv2
    out_dir.mkdir(parents=True, exist_ok=True)
    fpv = sorted(rec_dir.glob("fpv_*.jpg"))
    chase = sorted(rec_dir.glob("chase_*.jpg"))
    res = {"fpv_frames": len(fpv), "chase_frames": len(chase)}
    if fpv:
        res["fpv_mp4"] = encode(rec_dir, "fpv_%05d.jpg", out_dir / "fpv.mp4", fps)
        contact_sheet(fpv, out_dir / "fpv_contact_sheet.png")
    if chase:
        res["chase_mp4"] = encode(rec_dir, "chase_%05d.jpg", out_dir / "chase.mp4", fps)
        contact_sheet(chase, out_dir / "chase_contact_sheet.png")
    if fpv and track:
        both = rec_dir / "both"
        both.mkdir(exist_ok=True)
        for i, p in enumerate(fpv):
            im = cv2.resize(cv2.imread(str(p)), (640, 480))
            m = map_panel(track, square, min(i, len(track) - 1))
            cv2.imwrite(str(both / f"b_{i:05d}.jpg"), np.hstack([im, m]), [cv2.IMWRITE_JPEG_QUALITY, 92])
        res["fpv_and_map_mp4"] = encode(both, "b_%05d.jpg", out_dir / "fpv_and_map.mp4", fps)
        res["fpv_and_map_frames"] = len(fpv)
        contact_sheet(sorted(both.glob("b_*.jpg")), out_dir / "fpv_and_map_contact_sheet.png", cols=2)
        shutil.rmtree(both, ignore_errors=True)
    return res


class Planner:
    """Sim-time flight plan: climb to 3 m, fly a 10 m square with the nose along the flight, hover.
    step(n, e, d) -> (v_north, v_east, v_down_cmd_alt_m, yaw_deg, done)  (positions relative to the start, d = down, negative up)"""

    def __init__(self, leg=10.0, alt=3.0, speed=2.0, hover_s=3.0):
        self.leg, self.alt, self.speed, self.hover_s = leg, alt, speed, hover_s
        self.pts = [(leg, 0.0), (leg, leg), (0.0, leg), (0.0, 0.0)]
        self.i = 0
        self.phase = "climb"
        self.hover_left = hover_s
        self.yaw = 0.0
        self.square = [(0.0, 0.0)] + self.pts[:-1]

    def step(self, n, e, d, dt=0.1):
        import math
        if self.phase == "climb":
            if -d >= self.alt - 0.3:
                self.phase = "fly"
            return 0.0, 0.0, self.yaw, False
        if self.phase == "fly":
            tn, te = self.pts[self.i]
            dn, de = tn - n, te - e
            dist = math.hypot(dn, de)
            if dist < 0.7:
                self.i += 1
                if self.i >= len(self.pts):
                    self.phase = "hover"
                return 0.0, 0.0, self.yaw, False
            self.yaw = math.degrees(math.atan2(de, dn))
            s = min(self.speed, 0.8 * dist + 0.3)
            return s * dn / dist, s * de / dist, self.yaw, False
        self.hover_left -= dt
        return 0.0, 0.0, self.yaw, self.hover_left <= 0
