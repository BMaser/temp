"""Films, contact sheets and the map for the two Unreal simulators (AirSim and Project AirSim).

Input: a folder with the pictures a flight program saved and a track.json (see below).
Output: fpv.mp4, chase.mp4, fpv_and_map.mp4, fpv_contact.png, chase_contact.png, fpv_and_map_contact.png.

track.json = {"fps": 15, "plan_xy": [[north, east], ...], "start_xy": [n, e],
              "samples": [{"t": sim seconds, "n": north m, "e": east m, "d": down m, "yaw": rad (0 = north, + toward east),
                           "fpv": "file name or null", "chase": "file name or null"}, ...]}

The film code is the same as in the PX4 + Gazebo and Webots films (branch px4-gazebo-base, tools/sync_film.py): H.264 yuv420p
through ffmpeg, a contact sheet of 12 pictures (4 by 3, first and last included) stamped with the simulated time, and a map from
above beside the camera picture. The map here has no gates: it shows the planned path (thin), the path flown so far (bold),
the drone (dot) and the direction of the nose (arrow). East is to the right, north is up.

The film is made at a steady rate: picture k of the film shows the sample whose simulated time is closest to k / fps. If the
simulator delivered fewer pictures than that, a picture is shown twice; the summary reports how many different pictures there were.

  python unreal_films.py --selftest OUT_DIR TRACK_JSON    (makes films from drawn stand-in pictures, never from a simulator)
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

BACKGROUND = (247, 247, 244)
GRID = (225, 225, 220)
GRID_TEXT = (150, 150, 145)
PLAN_COLOR = (110, 110, 215)
FLOWN_COLOR = (214, 39, 40)
DOT_COLOR = (0, 0, 0)
ARROW_COLOR = (0, 160, 70)
TEXT_COLOR = (20, 20, 20)
START_FILL = (255, 235, 59)
START_LINE = (120, 110, 0)
FOOTER_PX = 36
ARROW_LENGTH_PX = 34


def font(size: int):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow older than 10.1
        return ImageFont.load_default()


class FilmWriter:
    """Raw RGB pictures in on stdin, H.264 yuv420p mp4 out."""

    def __init__(self, path: Path, width: int, height: int, fps: int, log_path: Path | None = None):
        if width % 2 or height % 2:
            raise ValueError(f"yuv420p needs even picture sizes, got {width}x{height}")
        self.path, self.width, self.height = Path(path), int(width), int(height)
        self.count = 0
        self._log = open(log_path, "ab") if log_path else subprocess.DEVNULL
        cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
               "-r", str(fps), "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
               "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(self.path)]
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=self._log)

    def write(self, img: np.ndarray) -> None:
        if img.shape != (self.height, self.width, 3) or img.dtype != np.uint8:
            raise ValueError(f"unexpected picture {img.shape} {img.dtype}, wanted ({self.height}, {self.width}, 3) uint8")
        self._proc.stdin.write(np.ascontiguousarray(img).tobytes())
        self.count += 1

    def close(self) -> None:
        try:
            self._proc.stdin.close()
        except Exception:
            pass
        code = self._proc.wait(timeout=600)
        if hasattr(self._log, "close"):
            self._log.close()
        if code != 0 or not self.path.is_file() or self.path.stat().st_size == 0:
            raise RuntimeError(f"ffmpeg failed with return code {code} for {self.path}")


def _read_exact(stream, n: int) -> bytes:
    chunks, got = [], 0
    while got < n:
        chunk = stream.read(n - got)
        if not chunk:
            break
        chunks.append(chunk)
        got += len(chunk)
    return b"".join(chunks)


def read_frames_at(film: Path, width: int, height: int, wanted: list[int] | None = None) -> tuple[dict, int]:
    """Decode a film: ({index: picture} for the wanted indices, number of pictures ffmpeg decodes)."""
    wanted_set = set(wanted or [])
    proc = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-i", str(film), "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    size = width * height * 3
    found, index = {}, 0
    try:
        while True:
            raw = _read_exact(proc.stdout, size)
            if len(raw) < size:
                break
            if index in wanted_set:
                found[index] = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3).copy()
            index += 1
    finally:
        proc.stdout.close()
        proc.wait(timeout=120)
    return found, index


def evenly_spaced_indices(count: int, n: int = 12) -> list[int]:
    if count <= 0:
        return []
    if count <= n:
        return list(range(count))
    return [int(round(k * (count - 1) / (n - 1))) for k in range(n)]


def contact_sheet(images: list[np.ndarray], labels: list[str], cols: int = 4, max_width: int = 1920) -> Image.Image:
    h, w = images[0].shape[:2]
    tile_w = max_width // cols
    tile_h = int(round(h * tile_w / w))
    tile_w -= tile_w % 2
    rows = (len(images) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * tile_w, rows * tile_h), (30, 30, 30))
    f = font(max(12, tile_h // 14))
    for k, (img, label) in enumerate(zip(images, labels)):
        tile = Image.fromarray(img).resize((tile_w, tile_h), Image.BILINEAR)
        d = ImageDraw.Draw(tile)
        box = d.textbbox((6, 4), label, font=f)
        d.rectangle([box[0] - 4, box[1] - 3, box[2] + 4, box[3] + 3], fill=(0, 0, 0))
        d.text((6, 4), label, fill=(255, 255, 255), font=f)
        sheet.paste(tile, ((k % cols) * tile_w, (k // cols) * tile_h))
    return sheet


def film_contact_sheet(film: Path, width: int, height: int, times, out_png: Path, n: int = 12) -> dict:
    _, count = read_frames_at(film, width, height, None)
    idx = evenly_spaced_indices(count, n)
    frames, _ = read_frames_at(film, width, height, idx)
    images = [frames[i] for i in idx]
    labels = [f"{float(times[i]):.2f} s" if i < len(times) else f"picture {i}" for i in idx]
    sheet = contact_sheet(images, labels, cols=4)
    sheet.save(out_png)
    return {"pictures_decoded": count, "indices": idx, "file": Path(out_png).name, "sheet_size": list(sheet.size)}


class MapPanel:
    """Map from above, a square of `size` pixels (the lowest FOOTER_PX pixels hold one line of text). East right, north up."""

    def __init__(self, plan_ne, start_ne, all_ne, size: int = 480, margin_m: float = 2.0):
        self.size, self.footer = size, FOOTER_PX
        pts = np.asarray(list(plan_ne) + list(all_ne) + [start_ne], dtype=float).reshape(-1, 2)
        lo, hi = pts.min(axis=0) - margin_m, pts.max(axis=0) + margin_m
        area = size - self.footer
        self.scale = min(size / max(hi[1] - lo[1], 1.0), area / max(hi[0] - lo[0], 1.0))
        cx, cn = (lo[1] + hi[1]) / 2, (lo[0] + hi[0]) / 2
        self.origin = (size / 2 - self.scale * cx, area / 2 + self.scale * cn)  # pixel of east = 0, north = 0
        self.font_small, self.font_text = font(12), font(15)
        self.plan_ne, self.start_ne = np.asarray(plan_ne, dtype=float).reshape(-1, 2), np.asarray(start_ne, dtype=float)
        self.background_image = self._draw_static()

    def px(self, ne) -> tuple[float, float]:
        return (self.origin[0] + self.scale * float(ne[1]), self.origin[1] - self.scale * float(ne[0]))

    def _draw_static(self) -> Image.Image:
        img = Image.new("RGB", (self.size, self.size), BACKGROUND)
        d = ImageDraw.Draw(img)
        area = self.size - self.footer
        step = 5 if self.size / self.scale > 25 else 2
        e_lo, e_hi = math.floor((0 - self.origin[0]) / self.scale), math.ceil((self.size - self.origin[0]) / self.scale)
        for e in range(e_lo - e_lo % step, e_hi + 1, step):
            u = self.origin[0] + self.scale * e
            if 0 <= u < self.size:
                d.line([(u, 0), (u, area)], fill=GRID, width=1)
                d.text((u + 2, area - 14), f"{e}", fill=GRID_TEXT, font=self.font_small)
        n_lo, n_hi = math.floor((self.origin[1] - area) / self.scale), math.ceil(self.origin[1] / self.scale)
        for n in range(n_lo - n_lo % step, n_hi + 1, step):
            v = self.origin[1] - self.scale * n
            if 0 <= v < area:
                d.line([(0, v), (self.size, v)], fill=GRID, width=1)
                d.text((2, v - 14), f"{n}", fill=GRID_TEXT, font=self.font_small)
        d.text((self.size - 52, 4), "east >", fill=GRID_TEXT, font=self.font_small)
        d.text((4, 4), "north ^", fill=GRID_TEXT, font=self.font_small)
        if len(self.plan_ne) > 1:
            d.line([self.px(p) for p in self.plan_ne], fill=PLAN_COLOR, width=1, joint="curve")
        u, v = self.px(self.start_ne)
        r = 7
        d.ellipse([u - r, v - r, u + r, v + r], fill=START_FILL, outline=START_LINE, width=2)
        d.text((u - 12, v + r + 1), "start", fill=TEXT_COLOR, font=self.font_small)
        d.rectangle([0, area, self.size, self.size], fill=(255, 255, 255))
        d.line([(0, area), (self.size, area)], fill=GRID, width=1)
        return img

    @staticmethod
    def _arrow(d, p0, p1, color, width: int, head: int) -> None:
        d.line([p0, p1], fill=color, width=width)
        ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        for s in (-1, 1):
            a = ang + math.pi - s * math.radians(28)
            d.line([p1, (p1[0] + head * math.cos(a), p1[1] + head * math.sin(a))], fill=color, width=width)

    def render(self, flown_ne, yaw: float, time_s: float, speed_mps: float, alt_m: float) -> np.ndarray:
        img = self.background_image.copy()
        d = ImageDraw.Draw(img)
        pts = [self.px(p) for p in np.asarray(flown_ne, dtype=float).reshape(-1, 2)]
        if len(pts) > 1:
            d.line(pts, fill=FLOWN_COLOR, width=4, joint="curve")
        dot = pts[-1]
        tip = (dot[0] + ARROW_LENGTH_PX * math.sin(yaw), dot[1] - ARROW_LENGTH_PX * math.cos(yaw))
        self._arrow(d, dot, tip, ARROW_COLOR, 3, 11)
        d.ellipse([dot[0] - 6, dot[1] - 6, dot[0] + 6, dot[1] + 6], fill=DOT_COLOR)
        d.text((8, self.size - self.footer + 9), f"t {time_s:5.2f} s   speed {speed_mps:4.2f} m/s   height {alt_m:4.1f} m",
               fill=TEXT_COLOR, font=self.font_text)
        return np.asarray(img)


def _load(path: Path):
    if path is None:
        return None
    try:
        return np.asarray(Image.open(path).convert("RGB"))
    except Exception:
        return None


def make_films(out_dir: Path, frames_dir: Path, track: dict, log_path: Path | None = None) -> dict:
    """Write the three films and three contact sheets. Returns a dict of facts (never claims a film that was not written)."""
    out_dir, frames_dir = Path(out_dir), Path(frames_dir)
    fps = int(track.get("fps", 15))
    samples = [s for s in track["samples"] if s.get("t") is not None]
    info: dict = {"fps": fps, "samples": len(samples), "films": {}, "problems": []}
    if not samples:
        info["problems"].append("the flight program saved no samples")
        return info
    samples.sort(key=lambda s: s["t"])
    ts = np.array([s["t"] for s in samples])
    t_end = float(ts[-1])
    n_film = int(math.floor(t_end * fps)) + 1
    grid = np.arange(n_film) / fps
    nearest = np.abs(ts[None, :] - grid[:, None]).argmin(axis=1)
    info["film_seconds"] = n_film / fps
    all_ne = [[s["n"], s["e"]] for s in samples]
    # which cameras have pictures
    for cam in ("fpv", "chase"):
        have = [bool(s.get(cam)) and (frames_dir / s[cam]).is_file() for s in samples]
        info[f"{cam}_pictures_saved"] = int(sum(have))
        used = {int(i) for i in nearest if have[int(i)]}
        info[f"{cam}_different_pictures_in_film"] = len(used)
        info[f"{cam}_pictures_per_sim_second"] = (sum(have) / t_end) if t_end > 0 else None

    # speed from neighbouring samples
    pos = np.array([[s["n"], s["e"], s["d"]] for s in samples])
    speed = np.zeros(len(samples))
    for i in range(1, len(samples)):
        dt = max(ts[i] - ts[i - 1], 1e-6)
        speed[i] = float(np.linalg.norm(pos[i] - pos[i - 1]) / dt)

    def picture_for(idx: int, cam: str):
        """Picture of sample idx, or the nearest sample that has one (searching outward, at most 1 s away)."""
        for off in range(0, max(2, fps)):
            for j in (idx - off, idx + off):
                if 0 <= j < len(samples) and samples[j].get(cam):
                    img = _load(frames_dir / samples[j][cam])
                    if img is not None:
                        return img
        return None

    cache: dict = {}

    def cached(idx, cam):
        key = (idx, cam)
        if key not in cache:
            if len(cache) > 400:
                cache.clear()
            cache[key] = picture_for(idx, cam)
        return cache[key]

    sizes = {}
    for cam in ("fpv", "chase"):
        first = next((cached(int(i), cam) for i in nearest if cached(int(i), cam) is not None), None)
        if first is not None:
            h, w = first.shape[:2]
            sizes[cam] = (w - w % 2, h - h % 2)
    writers: dict = {}
    mp = None
    try:
        for cam, (w, h) in sizes.items():
            writers[cam] = FilmWriter(out_dir / f"{cam}.mp4", w, h, fps, log_path)
        if "fpv" in sizes:
            w, h = sizes["fpv"]
            mp = MapPanel(track.get("plan_xy", []), track.get("start_xy", all_ne[0]), all_ne, size=h)
            writers["fpv_and_map"] = FilmWriter(out_dir / "fpv_and_map.mp4", w + h, h, fps, log_path)
        for k in range(n_film):
            idx = int(nearest[k])
            for cam in ("fpv", "chase"):
                if cam in writers:
                    img = cached(idx, cam)
                    w, h = sizes[cam]
                    if img is None:
                        img = np.zeros((h, w, 3), np.uint8)
                    writers[cam].write(np.ascontiguousarray(img[:h, :w]))
            if "fpv_and_map" in writers:
                w, h = sizes["fpv"]
                img = cached(idx, "fpv")
                img = np.zeros((h, w, 3), np.uint8) if img is None else np.ascontiguousarray(img[:h, :w])
                upto = idx + 1
                mapimg = mp.render(pos[:upto, :2], samples[idx]["yaw"], float(ts[idx]), float(speed[idx]), float(-pos[idx, 2]))
                writers["fpv_and_map"].write(np.concatenate([img, mapimg], axis=1))
        for name, wr in writers.items():
            wr.close()
    except Exception as e:  # noqa: BLE001
        info["problems"].append(f"film writing failed: {e!r}")
        for wr in writers.values():
            try:
                wr._proc.kill()
            except Exception:
                pass
        writers = {}
    for name, wr in writers.items():
        w, h = (wr.width, wr.height)
        try:
            res = film_contact_sheet(out_dir / f"{name}.mp4", w, h, grid, out_dir / f"{name}_contact.png")
            res["pictures_written"] = wr.count
            res["bytes"] = (out_dir / f"{name}.mp4").stat().st_size
            info["films"][f"{name}.mp4"] = res
        except Exception as e:  # noqa: BLE001
            info["problems"].append(f"contact sheet of {name} failed: {e!r}")
    return info


def film_quality(out_dir: Path, info: dict) -> dict:
    """Pixel standard deviation of 12 pictures of each film and the mean difference between neighbours (the pass test of the other films)."""
    res = {}
    for name, f in info.get("films", {}).items():
        try:
            probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height",
                                    "-of", "csv=p=0", str(Path(out_dir) / name)], capture_output=True, text=True)
            fw, fh = (int(x) for x in probe.stdout.strip().split(","))
            idx = evenly_spaced_indices(f["pictures_decoded"], 12)
            frames, _ = read_frames_at(Path(out_dir) / name, fw, fh, idx)
            arrs = [frames[i].astype(np.float64) for i in idx]
            res[name] = {"stds": [round(float(a.std()), 1) for a in arrs],
                         "neighbour_mean_abs_diff": [round(float(np.abs(a - b).mean()), 1) for a, b in zip(arrs[:-1], arrs[1:])]}
        except Exception as e:  # noqa: BLE001
            res[name] = {"error": repr(e)}
    return res


def selftest(out_dir: Path, track_json: Path) -> int:
    """Make films from drawn stand-in pictures (a coloured ground, a horizon that tilts with the yaw, the time in big letters)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    frames = out_dir / "standin_frames"
    frames.mkdir(exist_ok=True)
    track = json.loads(Path(track_json).read_text())
    fps = int(track.get("fps", 15))
    samples = track["samples"]
    # use at most 150 samples spread over the flight so the self-test is quick
    keep = sorted(set(int(round(k)) for k in np.linspace(0, len(samples) - 1, min(len(samples), 150))))
    samples = [samples[i] for i in keep]
    f = font(40)
    for i, s in enumerate(samples):
        for cam, size in (("fpv", (640, 480)), ("chase", (640, 480))):
            img = Image.new("RGB", size, (120, 170, 230) if cam == "fpv" else (150, 150, 150))
            d = ImageDraw.Draw(img)
            horizon = int(size[1] * 0.5 + 60 * math.sin(s["yaw"]))
            d.rectangle([0, horizon, size[0], size[1]], fill=(70, 130, 60) if cam == "fpv" else (90, 90, 90))
            for g in range(0, size[0], 80):
                d.line([(g + int(40 * math.cos(s["yaw"])), horizon), (g, size[1])], fill=(40, 90, 40), width=2)
            d.text((20, 20), f"STAND-IN {cam} t={s['t']:.2f}", fill=(255, 255, 255), font=f)
            name = f"{cam}_{i:05d}.jpg"
            img.save(frames / name, quality=90)
            s[cam] = name
    track["samples"] = samples
    info = make_films(out_dir, frames, track, out_dir / "ffmpeg.log")
    q = film_quality(out_dir, info)
    (out_dir / "selftest.json").write_text(json.dumps({"info": info, "quality": q}, indent=2))
    print(json.dumps({"info": info, "quality": q}, indent=2))
    ok = (not info["problems"] and {"fpv.mp4", "chase.mp4", "fpv_and_map.mp4"} <= set(info["films"])
          and all(v.get("pictures_decoded", 0) >= 50 for v in info["films"].values())
          and all(all(x > 5 for x in v.get("stds", [0])) for v in q.values()))
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--selftest":
        sys.exit(selftest(Path(sys.argv[2]), Path(sys.argv[3])))
    print(__doc__)
