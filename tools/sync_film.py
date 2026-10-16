"""Camera picture beside a map from above, in sync, and the picture helpers of the run-A test.

Left panel: the camera picture of one step. Right panel: the map of the same step, seen from above (+x to the right,
+y up): the gates (place, opening, and an arrow for the direction in which they are passed, with the number of each
passage), the poles, the start pad and the end pad, the planned path as a thin line, the path flown so far as a bold
line, the drone as a dot, and an arrow for the direction the camera looks. A line of text under the map gives the time,
the speed and the number of passages.

The camera looks along the nose of the drone, the body x axis: crazyflow drones/cf21B_500.xml line 91 defines
`<camera name="fpv_cam" pos="0.03 0 0" euler="0 -90 -90" fovy="70" .../>` with no `eulerseq` on the compiler (line 11),
so MuJoCo's default "xyz" holds (lower case = the axes turn with the frame). MuJoCo cameras look along their -Z axis
(mujoco XMLreference.rst, element camera, "The camera is looking along the -Z axis of its frame"). The rotation
Rx(0) Ry(-90) Rz(-90) maps the camera's -Z axis to the body +x axis (checked with scipy: [1, 0, 0]) and the camera's
up axis (+Y) to the body +z axis. The arrow of the map is therefore the horizontal part of the body x axis, taken from
the SIMULATED quaternion (order x y z w) of the same step, never from the command.

The fixed parts of the map are drawn once (`MapPanel.background`); a picture only copies that image and draws the
moving parts on it with Pillow, which costs a few milliseconds.

Only numpy and Pillow are needed (matplotlib is not part of the default pixi environment of lsy_drone_racing).
ffmpeg is needed only to write or read an mp4.

  python tools/sync_film.py --self-test --out <folder>
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

FPS = 25
GATE_OUTER_M = 0.72  # tests/../level0.toml: "Gates are 0.72m wide (outer dimensions of the frame) with a 0.4m wide opening."
GATE_OPENING_M = 0.40
PAD_RADIUS_M = 0.3  # assets/pads/*.xml: radius 0.3 m
ARROW_LENGTH_PX = 34
FOOTER_PX = 36

BACKGROUND = (247, 247, 244)
GRID = (225, 225, 220)
GRID_TEXT = (150, 150, 145)
PLAN_COLOR = (110, 110, 215)
FLOWN_COLOR = (214, 39, 40)
DOT_COLOR = (0, 0, 0)
ARROW_COLOR = (0, 160, 70)
GATE_FRAME_COLOR = (70, 70, 80)
GATE_OPENING_COLOR = (70, 170, 230)
GATE_ARROW_COLOR = (150, 60, 200)
POLE_COLOR = (30, 30, 30)
START_PAD_FILL = (255, 235, 59)
START_PAD_LINE = (120, 110, 0)
END_PAD_FILL = (255, 255, 255)
END_PAD_LINE = (60, 60, 60)
TEXT_COLOR = (20, 20, 20)
DOT_RADIUS_PX = 6


def font(size: int):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow older than 10.1
        return ImageFont.load_default()


# ---------------------------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------------------------


def body_x_axis(quat_xyzw) -> np.ndarray:
    """Body x axis (the nose, the direction the camera looks) in the world, from a quaternion in the order x y z w."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w)])


def gate_axes(quat_xyzw) -> tuple[np.ndarray, np.ndarray]:
    """Horizontal unit vectors of a gate: a = local x (the direction of a forward passage), n = local y (along the frame)."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    ax = np.array([1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w)])
    ny = np.array([2.0 * (x * y - z * w), 1.0 - 2.0 * (x * x + z * z)])
    ax = ax / max(np.linalg.norm(ax), 1e-9)
    ny = ny / max(np.linalg.norm(ny), 1e-9)
    return ax, ny


class MapPanel:
    """The map from above. `track` is a dict:

    gates_pos (n, 3), gates_quat (n, 4, order x y z w), gate_sequence (m,) gate ids in the order of the passages,
    gate_direction (m,) +1 forward / -1 reverse, obstacles_pos (k, 3), pad_start (x, y), pad_end (x, y) or None,
    plan_xy (p, 2) the planned path, optional gate_outer_m and gate_opening_m.
    """

    def __init__(self, track: dict, size: int = 480, footer: int = FOOTER_PX, margin_m: float = 0.45):
        self.size, self.footer = int(size), int(footer)
        self.track = track
        self.gates_pos = np.asarray(track["gates_pos"], dtype=float).reshape(-1, 3)
        self.gates_quat = np.asarray(track["gates_quat"], dtype=float).reshape(-1, 4)
        self.sequence = [int(v) for v in np.asarray(track["gate_sequence"]).reshape(-1)]
        self.direction = [int(np.sign(v)) or 1 for v in np.asarray(track["gate_direction"]).reshape(-1)]
        self.poles = np.asarray(track.get("obstacles_pos", np.zeros((0, 3))), dtype=float).reshape(-1, 3)
        self.pad_start = tuple(track["pad_start"]) if track.get("pad_start") is not None else None
        self.pad_end = tuple(track["pad_end"]) if track.get("pad_end") is not None else None
        self.plan_xy = np.asarray(track["plan_xy"], dtype=float).reshape(-1, 2)
        self.gate_outer = float(track.get("gate_outer_m", GATE_OUTER_M))
        self.gate_opening = float(track.get("gate_opening_m", GATE_OPENING_M))

        points = [self.gates_pos[:, :2], self.poles[:, :2], self.plan_xy]
        for pad in (self.pad_start, self.pad_end):
            if pad is not None:
                points.append(np.array([[pad[0] - PAD_RADIUS_M, pad[1] - PAD_RADIUS_M], [pad[0] + PAD_RADIUS_M, pad[1] + PAD_RADIUS_M]]))
        for g in range(len(self.gates_pos)):
            _, n = gate_axes(self.gates_quat[g])
            for s in (-1, 1):
                points.append((self.gates_pos[g, :2] + s * 0.5 * self.gate_outer * n).reshape(1, 2))
        extra = track.get("extent_points")
        if extra is not None:
            points.append(np.asarray(extra, dtype=float).reshape(-1, 2))
        all_points = np.concatenate([p for p in points if len(p)], axis=0)
        lo, hi = all_points.min(axis=0) - margin_m, all_points.max(axis=0) + margin_m
        area_h = self.size - self.footer
        self.scale = min(self.size / (hi[0] - lo[0]), area_h / (hi[1] - lo[1]))  # pixels per metre, the same in x and y
        cx, cy = 0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])
        self.origin = np.array([0.5 * self.size - self.scale * cx, 0.5 * area_h + self.scale * cy])  # pixel of world (0, 0)
        self.font_small, self.font_text = font(12), font(15)
        self.background_image = self._draw_static()
        self.background = np.asarray(self.background_image).copy()

    # -- transforms --
    def px(self, xy) -> tuple[float, float]:
        return (float(self.origin[0] + self.scale * xy[0]), float(self.origin[1] - self.scale * xy[1]))

    def px_list(self, xy) -> list[tuple[float, float]]:
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        u = self.origin[0] + self.scale * xy[:, 0]
        v = self.origin[1] - self.scale * xy[:, 1]
        return list(zip(u.tolist(), v.tolist()))

    # -- fixed parts --
    def gate_segments(self, g: int) -> dict:
        """World coordinates of the drawn parts of gate g: two frame posts, the opening, and the arrows of its passages."""
        c = self.gates_pos[g, :2]
        a, n = gate_axes(self.gates_quat[g])
        half_out, half_open = 0.5 * self.gate_outer, 0.5 * self.gate_opening
        posts = [(c + s * half_open * n, c + s * half_out * n) for s in (-1, 1)]
        opening = (c - half_open * n, c + half_open * n)
        passes = [(j + 1, self.direction[j]) for j, gid in enumerate(self.sequence) if gid == g]
        arrows = [(c, c + d * 0.35 * a) for d in sorted({d for _, d in passes})]
        return {"center": c, "a": a, "n": n, "posts": posts, "opening": opening, "arrows": arrows, "passes": passes}

    def _draw_static(self) -> Image.Image:
        img = Image.new("RGB", (self.size, self.size), BACKGROUND)
        d = ImageDraw.Draw(img)
        area_h = self.size - self.footer
        # grid every metre, with the coordinate at the edge
        xs = np.arange(math.floor((0 - self.origin[0]) / self.scale), math.ceil((self.size - self.origin[0]) / self.scale) + 1)
        for x in xs:
            u = self.origin[0] + self.scale * x
            if 0 <= u < self.size:
                d.line([(u, 0), (u, area_h)], fill=GRID, width=1)
                d.text((u + 2, area_h - 14), f"{int(x)}", fill=GRID_TEXT, font=self.font_small)
        ys = np.arange(math.floor((self.origin[1] - area_h) / self.scale), math.ceil(self.origin[1] / self.scale) + 1)
        for y in ys:
            v = self.origin[1] - self.scale * y
            if 0 <= v < area_h:
                d.line([(0, v), (self.size, v)], fill=GRID, width=1)
                d.text((2, v - 14), f"{int(y)}", fill=GRID_TEXT, font=self.font_small)
        # pads
        r = PAD_RADIUS_M * self.scale
        if self.pad_start is not None:
            u, v = self.px(self.pad_start)
            d.ellipse([u - r, v - r, u + r, v + r], fill=START_PAD_FILL, outline=START_PAD_LINE, width=2)
            d.text((u - 12, v + r + 1), "start", fill=TEXT_COLOR, font=self.font_small)
        if self.pad_end is not None:
            u, v = self.px(self.pad_end)
            d.ellipse([u - r, v - r, u + r, v + r], fill=END_PAD_FILL, outline=END_PAD_LINE, width=3)
            d.ellipse([u - r * 0.5, v - r * 0.5, u + r * 0.5, v + r * 0.5], outline=END_PAD_LINE, width=2)
            d.text((u - 9, v + r + 1), "end", fill=TEXT_COLOR, font=self.font_small)
        # planned path (thin)
        if len(self.plan_xy) > 1:
            d.line(self.px_list(self.plan_xy), fill=PLAN_COLOR, width=1, joint="curve")
        # poles
        for p in self.poles:
            u, v = self.px(p[:2])
            d.ellipse([u - 4, v - 4, u + 4, v + 4], fill=POLE_COLOR, outline=(255, 255, 255))
        # gates
        for g in range(len(self.gates_pos)):
            seg = self.gate_segments(g)
            d.line([self.px(seg["opening"][0]), self.px(seg["opening"][1])], fill=GATE_OPENING_COLOR, width=2)
            for p0, p1 in seg["posts"]:
                d.line([self.px(p0), self.px(p1)], fill=GATE_FRAME_COLOR, width=6)
            for p0, p1 in seg["arrows"]:
                self._arrow(d, self.px(p0), self.px(p1), GATE_ARROW_COLOR, 2, 8)
            label = ",".join(str(j) for j, _ in seg["passes"])
            if label:
                # on the entry side, on the line of the first arrow: the arrow points from the number through the gate
                first_dir = seg["passes"][0][1]
                u, v = self.px(seg["center"] - 0.3 * first_dir * seg["a"])
                d.text((u - 4 * len(label), v - 8), label, fill=GATE_ARROW_COLOR, font=self.font_text)
        # footer
        d.rectangle([0, area_h, self.size, self.size], fill=(255, 255, 255))
        d.line([(0, area_h), (self.size, area_h)], fill=GRID, width=1)
        return img

    @staticmethod
    def _arrow(d: ImageDraw.ImageDraw, p0, p1, color, width: int, head: int) -> None:
        d.line([p0, p1], fill=color, width=width)
        ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        for s in (-1, 1):
            a = ang + math.pi - s * math.radians(28)
            d.line([p1, (p1[0] + head * math.cos(a), p1[1] + head * math.sin(a))], fill=color, width=width)

    # -- one picture --
    def camera_arrow_px(self, dot_px, quat_xyzw):
        """Pixel of the arrow tip, or None if the camera looks straight up or down."""
        ax = body_x_axis(quat_xyzw)
        h = math.hypot(ax[0], ax[1])
        if h < 1e-6:
            return None
        return (dot_px[0] + ARROW_LENGTH_PX * ax[0] / h, dot_px[1] - ARROW_LENGTH_PX * ax[1] / h)

    def render(self, flown_xy, quat_xyzw, time_s: float, speed_mps: float, passages: int, total: int) -> np.ndarray:
        """The map of one step. `flown_xy` holds the horizontal positions from the start up to the current step."""
        img = self.background_image.copy()
        d = ImageDraw.Draw(img)
        flown_xy = np.asarray(flown_xy, dtype=float).reshape(-1, 2)
        pts = self.px_list(flown_xy)
        if len(pts) > 1:
            d.line(pts, fill=FLOWN_COLOR, width=4, joint="curve")
        dot = pts[-1]
        tip = self.camera_arrow_px(dot, quat_xyzw)
        if tip is not None:
            self._arrow(d, dot, tip, ARROW_COLOR, 3, 11)
        r = DOT_RADIUS_PX
        d.ellipse([dot[0] - r, dot[1] - r, dot[0] + r, dot[1] + r], fill=DOT_COLOR)
        text = f"t {time_s:5.2f} s    speed {speed_mps:4.2f} m/s    passages {passages} of {total}"
        d.text((8, self.size - self.footer + 9), text, fill=TEXT_COLOR, font=self.font_text)
        return np.asarray(img)


# ---------------------------------------------------------------------------------------------
# Pixel checks used by the self-test (they read the picture, not the drawing code)
# ---------------------------------------------------------------------------------------------


def color_pixels(img: np.ndarray, color) -> np.ndarray:
    """(row, column) of every pixel that has exactly this color, shape (k, 2)."""
    mask = np.all(img == np.asarray(color, dtype=img.dtype), axis=-1)
    return np.argwhere(mask)


def arrow_offset_px(img: np.ndarray, dot_px) -> tuple[float, float] | None:
    """Offset (right, up) in pixels of the centre of the arrow-colored pixels from the dot, or None if there are none."""
    hits = color_pixels(img, ARROW_COLOR)
    if len(hits) == 0:
        return None
    return (float(hits[:, 1].mean() - dot_px[0]), float(dot_px[1] - hits[:, 0].mean()))


def bold_path_touches_dot(img: np.ndarray, dot_px, slack_px: float = 3.0) -> bool:
    """True if the bold-path pixel nearest to the dot is within the dot radius plus `slack_px` of the dot centre."""
    hits = color_pixels(img, FLOWN_COLOR)
    if len(hits) == 0:
        return False
    dist = np.hypot(hits[:, 1] - dot_px[0], hits[:, 0] - dot_px[1])
    return bool(dist.min() <= DOT_RADIUS_PX + slack_px)


# ---------------------------------------------------------------------------------------------
# ffmpeg: write and read films
# ---------------------------------------------------------------------------------------------


class FilmWriter:
    """Raw RGB pictures in on stdin, H.264 yuv420p mp4 out (the same settings as tests/crazyflow_flight_video.py)."""

    def __init__(self, path: Path, width: int, height: int, fps: int = FPS, log_path: Path | None = None):
        if width % 2 or height % 2:
            raise ValueError(f"yuv420p needs even picture sizes, got {width}x{height}")
        self.path, self.width, self.height = Path(path), int(width), int(height)
        self.count = 0
        self._log = open(log_path, "wb") if log_path else subprocess.DEVNULL
        cmd = [
            "ffmpeg", "-y", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}", "-r", str(fps), "-i", "-",
            "-c:v", "libx264", "-preset", "veryfast", "-crf", "23",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(self.path),
        ]
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=self._log)

    def write(self, img: np.ndarray) -> None:
        if img.shape != (self.height, self.width, 3) or img.dtype != np.uint8:
            raise ValueError(f"unexpected picture {img.shape} {img.dtype}, wanted ({self.height}, {self.width}, 3) uint8")
        self._proc.stdin.write(np.ascontiguousarray(img).tobytes())
        self.count += 1

    def close(self) -> int:
        """Finish the film. Returns the return code of ffmpeg; raises if it is not 0 or the file is empty."""
        try:
            self._proc.stdin.close()
        except Exception:
            pass
        code = self._proc.wait(timeout=600)
        if hasattr(self._log, "close"):
            self._log.close()
        if code != 0 or not self.path.is_file() or self.path.stat().st_size == 0:
            raise RuntimeError(f"ffmpeg failed with return code {code} for {self.path}")
        return code

    def abort(self) -> None:
        try:
            self._proc.stdin.close()
        except Exception:
            pass
        try:
            self._proc.kill()
        except Exception:
            pass


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
    """Decode a film and return ({index: picture} for the wanted indices, number of pictures in the film).

    With wanted=None only the number of pictures is counted. The count is what ffmpeg decodes, not what a writer says.
    """
    wanted_set = set(wanted or [])
    cmd = ["ffmpeg", "-loglevel", "error", "-i", str(film), "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
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


# ---------------------------------------------------------------------------------------------
# Contact sheet
# ---------------------------------------------------------------------------------------------


def evenly_spaced_indices(count: int, n: int = 12) -> list[int]:
    """n indices from 0 to count-1, evenly spaced, with the first and the last (fewer if the film is shorter)."""
    if count <= 0:
        return []
    if count <= n:
        return list(range(count))
    return [int(round(k * (count - 1) / (n - 1))) for k in range(n)]


def contact_sheet(images: list[np.ndarray], labels: list[str], cols: int = 4, max_width: int = 1920) -> Image.Image:
    """Grid of pictures (row by row), each stamped with its label in the top left corner; the sheet is at most max_width wide."""
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
    """12 pictures of a film, evenly spaced with the first and the last, 4 by 3, each stamped with its time.

    `times[i]` is the simulated time of picture i. Returns {"pictures_decoded": count, "indices": [...], "file": name}.
    """
    _, count = read_frames_at(film, width, height, None)
    idx = evenly_spaced_indices(count, n)
    frames, _ = read_frames_at(film, width, height, idx)
    images = [frames[i] for i in idx]
    labels = [f"{float(times[i]):.2f} s" if i < len(times) else f"picture {i}" for i in idx]
    sheet = contact_sheet(images, labels, cols=4)
    sheet.save(out_png)
    return {"pictures_decoded": count, "indices": idx, "file": Path(out_png).name, "sheet_size": list(sheet.size)}


# ---------------------------------------------------------------------------------------------
# The film: camera picture beside the map
# ---------------------------------------------------------------------------------------------


def make_sync_film(frames, arrays: dict, track: dict, out_mp4: Path | None, fps: int = FPS, save_pictures: dict | None = None) -> dict:
    """Write the film (or only render, if out_mp4 is None) and return counts and timings.

    frames: indexable, frames[k] is picture k of the camera, uint8 (H, W, 3).
    arrays: pos (N, 3), vel (N, 3), quat (N, 4) x y z w, passed (N,), time (N,), pic_step (P,) = the row of every picture.
    track: see MapPanel; plus "total_passages".
    save_pictures: {picture index: path} to write a few combined pictures as PNG (used by the self-test).
    """
    pos, vel, quat = (np.asarray(arrays[k], dtype=float) for k in ("pos", "vel", "quat"))
    passed, times = np.asarray(arrays["passed"]), np.asarray(arrays["time"], dtype=float)
    pic_step = [int(v) for v in np.asarray(arrays["pic_step"]).reshape(-1)]
    if len(frames) != len(pic_step):
        raise ValueError(f"{len(frames)} camera pictures but {len(pic_step)} picture rows")
    h, w = frames[0].shape[:2]
    panel = MapPanel(track, size=h)
    if panel.size != h:
        raise AssertionError("map and camera picture must have the same height")
    writer = FilmWriter(Path(out_mp4), w + panel.size, h, fps) if out_mp4 is not None else None
    seconds = []
    try:
        for k, row in enumerate(pic_step):
            t0 = time.perf_counter()
            speed = float(np.linalg.norm(vel[row]))
            mp = panel.render(pos[: row + 1, :2], quat[row], float(times[row]), speed, int(passed[row]), int(track["total_passages"]))
            combined = np.concatenate([np.asarray(frames[k]), mp], axis=1)
            seconds.append(time.perf_counter() - t0)
            if save_pictures and k in save_pictures:
                Image.fromarray(combined).save(save_pictures[k])
            if writer is not None:
                writer.write(combined)
        if writer is not None:
            writer.close()
            writer = None
    finally:
        if writer is not None:
            writer.abort()
    return {
        "pictures_written": len(pic_step), "width": w + panel.size, "height": h,
        "seconds_per_picture_mean": float(np.mean(seconds)), "seconds_per_picture_max": float(np.max(seconds)),
        "background_unchanged": bool(np.array_equal(panel.background, np.asarray(panel.background_image))),
    }


# ---------------------------------------------------------------------------------------------
# Line chart (Pillow only)
# ---------------------------------------------------------------------------------------------


def line_chart(path: Path, panels: list[dict], width: int = 1500, panel_height: int = 420) -> None:
    """Stacked line charts. panel = {"title", "ylabel", "series": [{"label", "color", "x", "y", "dashed"}], "hlines": [(y, text)]}.

    The x axis is shared in meaning (time in s). Non-finite values break a line.
    """
    left, right, top, bottom = 80, 300, 34, 44
    img = Image.new("RGB", (width, panel_height * len(panels)), (255, 255, 255))
    d = ImageDraw.Draw(img)
    f, fs = font(15), font(13)
    for pi, panel in enumerate(panels):
        y0 = pi * panel_height
        x_all = np.concatenate([np.asarray(s["x"], dtype=float) for s in panel["series"]])
        y_all = np.concatenate([np.asarray(s["y"], dtype=float) for s in panel["series"]])
        y_all = y_all[np.isfinite(y_all)]
        xmin, xmax = float(np.nanmin(x_all)), float(np.nanmax(x_all))
        ymin, ymax = (float(y_all.min()), float(y_all.max())) if len(y_all) else (0.0, 1.0)
        for yy, _ in panel.get("hlines", []):
            ymin, ymax = min(ymin, yy), max(ymax, yy)
        pad = 0.05 * (ymax - ymin or 1.0)
        ymin, ymax = ymin - pad, ymax + pad
        px0, px1, py0, py1 = left, width - right, y0 + top, y0 + panel_height - bottom

        def tx(x):
            return px0 + (x - xmin) / (xmax - xmin or 1.0) * (px1 - px0)

        def ty(y):
            return py1 - (y - ymin) / (ymax - ymin or 1.0) * (py1 - py0)

        d.rectangle([px0, py0, px1, py1], outline=(120, 120, 120))
        d.text((px0, y0 + 8), panel["title"], fill=(0, 0, 0), font=f)
        for xv in _nice_ticks(xmin, xmax):
            d.line([(tx(xv), py1), (tx(xv), py1 + 5)], fill=(120, 120, 120))
            d.text((tx(xv) - 12, py1 + 8), f"{xv:g}", fill=(60, 60, 60), font=fs)
        for yv in _nice_ticks(ymin, ymax):
            d.line([(px0 - 5, ty(yv)), (px0, ty(yv))], fill=(120, 120, 120))
            d.line([(px0, ty(yv)), (px1, ty(yv))], fill=(235, 235, 235))
            d.text((8, ty(yv) - 7), f"{yv:g}", fill=(60, 60, 60), font=fs)
        d.text((px1 - 70, py1 + 24), "time (s)", fill=(60, 60, 60), font=fs)
        d.text((8, y0 + 8), panel.get("ylabel", ""), fill=(60, 60, 60), font=fs)
        for yy, text in panel.get("hlines", []):
            d.line([(px0, ty(yy)), (px1, ty(yy))], fill=(200, 60, 60), width=1)
            d.text((px0 + 4, ty(yy) - 15), text, fill=(200, 60, 60), font=fs)
        for si, s in enumerate(panel["series"]):
            x, y = np.asarray(s["x"], dtype=float), np.asarray(s["y"], dtype=float)
            segment = []
            for xi, yi in zip(x, y):
                if np.isfinite(yi):
                    segment.append((tx(xi), ty(yi)))
                    continue
                if len(segment) > 1:
                    _polyline(d, segment, s["color"], bool(s.get("dashed")))
                segment = []
            if len(segment) > 1:
                _polyline(d, segment, s["color"], bool(s.get("dashed")))
            ly = y0 + top + 4 + 20 * si
            d.line([(px1 + 14, ly + 7), (px1 + 44, ly + 7)], fill=s["color"], width=3)
            d.text((px1 + 50, ly), s["label"], fill=(0, 0, 0), font=fs)
    img.save(path)


def _nice_ticks(lo: float, hi: float, n: int = 6) -> list[float]:
    """Tick values at 1, 2 or 5 times a power of ten, about n of them, inside [lo, hi]."""
    span = (hi - lo) or 1.0
    raw = span / n
    mag = 10 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 5, 10) if m * mag >= raw)
    first = math.ceil(lo / step) * step
    return [round(first + k * step, 10) for k in range(int((hi - first) / step) + 1)]


def _polyline(d: ImageDraw.ImageDraw, pts, color, dashed: bool) -> None:
    if not dashed:
        d.line(pts, fill=color, width=2, joint="curve")
        return
    for i in range(0, len(pts) - 1, 6):
        d.line(pts[i:i + 4], fill=color, width=1)


# ---------------------------------------------------------------------------------------------
# Self-test with made-up data (runs on a PC without the simulator)
# ---------------------------------------------------------------------------------------------


def _quat_yaw(yaw: float) -> list[float]:
    return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]


def made_up_track() -> dict:
    """Four gates and four poles in the places of level0, two pads, a planned path. Made up for the test, not read from the simulator."""
    gates_pos = np.array([[0.5, 0.25, 0.7], [1.05, 0.75, 1.2], [-1.0, -0.25, 0.7], [0.0, -0.75, 1.2]])
    gate_yaw = [-0.78, 2.35, 3.14, 0.0]
    t = np.linspace(0, 2 * math.pi, 200)
    plan = np.stack([1.1 * np.cos(t) - 0.1, 0.8 * np.sin(t) + 0.1], axis=1)
    return {
        "gates_pos": gates_pos, "gates_quat": np.array([_quat_yaw(y) for y in gate_yaw]),
        "gate_sequence": [0, 1, 2, 3, 1], "gate_direction": [1, 1, 1, 1, 1],
        "obstacles_pos": np.array([[0.0, 0.75, 1.55], [1.0, 0.25, 1.55], [-1.5, -0.25, 1.55], [-0.5, -0.75, 1.55]]),
        "pad_start": (-1.5, 0.75), "pad_end": (0.25, 1.25), "plan_xy": plan, "total_passages": 5,
    }


def made_up_flight(n: int = 300) -> dict:
    t = np.linspace(0, 2 * math.pi * 0.9, n)
    pos = np.stack([1.1 * np.cos(t) - 0.1, 0.8 * np.sin(t) + 0.1, 0.8 + 0.2 * np.sin(2 * t)], axis=1)
    vel = np.gradient(pos, axis=0) * 50.0
    heading = np.arctan2(vel[:, 1], vel[:, 0])
    quat = np.array([_quat_yaw(h) for h in heading])
    steps = np.arange(n)
    return {
        "pos": pos, "vel": vel, "quat": quat, "passed": np.minimum(steps // 60, 5), "time": steps / 50.0,
        "pic_step": np.arange(0, n, 2),
    }


def self_test(out: Path) -> int:
    out.mkdir(parents=True, exist_ok=True)
    results: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str) -> None:
        results.append((name, bool(ok), detail))

    track, flight = made_up_track(), made_up_flight()
    panel = MapPanel(track, size=480)
    bg_before = panel.background.copy()
    n = len(flight["pic_step"])

    # 1. three single maps and the timing
    picks = [0, n // 2, n - 1]
    seconds = []
    imgs = {}
    for k in picks:
        row = int(flight["pic_step"][k])
        t0 = time.perf_counter()
        imgs[k] = panel.render(flight["pos"][: row + 1, :2], flight["quat"][row], flight["time"][row],
                               float(np.linalg.norm(flight["vel"][row])), int(flight["passed"][row]), 5)
        seconds.append(time.perf_counter() - t0)
    check("map picture costs well under 0.1 s", max(seconds) < 0.1, f"slowest of {len(seconds)}: {max(seconds) * 1000:.1f} ms")
    check("fixed parts are drawn once and never changed by a picture", np.array_equal(bg_before, panel.background)
          and np.array_equal(bg_before, np.asarray(panel.background_image)), "background array equal before and after 3 pictures")

    # 2. the bold path ends at the dot (pixel check), with a control that must fail
    k = picks[1]
    row = int(flight["pic_step"][k])
    dot = panel.px(flight["pos"][row, :2])
    check("bold path ends at the dot", bold_path_touches_dot(imgs[k], dot), f"dot at pixel ({dot[0]:.1f}, {dot[1]:.1f})")
    wrong = (dot[0] + 45.0, dot[1] + 45.0)
    check("control: the same check says NO for a dot 64 px away", not bold_path_touches_dot(imgs[k], wrong), f"wrong dot ({wrong[0]:.1f}, {wrong[1]:.1f})")
    check("the dot has the dot color at its centre", tuple(imgs[k][int(round(dot[1])), int(round(dot[0]))]) == DOT_COLOR, "pixel at the centre")
    ahead = panel.px(flight["pos"][min(row + 60, len(flight["pos"]) - 1), :2])
    far_ahead = np.hypot(*(np.array(ahead) - np.array(dot))) > 25
    check("the path ahead of the drone is not drawn bold", far_ahead and tuple(imgs[k][int(round(ahead[1])), int(round(ahead[0]))]) != FLOWN_COLOR,
          f"pixel 60 steps ahead at ({ahead[0]:.0f}, {ahead[1]:.0f})")

    # 3. camera arrow direction from the quaternion: a drone that looks toward +y gives an arrow that points up
    spot = panel.px((0.0, 0.0))
    for name, yaw, want in (("+y", math.pi / 2, "up"), ("-y", -math.pi / 2, "down"), ("+x", 0.0, "right"), ("-x", math.pi, "left")):
        img = panel.render(np.array([[0.0, 0.0]]), _quat_yaw(yaw), 0.0, 0.0, 0, 5)
        off = arrow_offset_px(img, spot)
        if off is None:
            check(f"drone looks toward {name}: arrow points {want}", False, "no arrow pixels")
            continue
        dx, dy = off
        got = ("right" if dx > 5 else "left" if dx < -5 else "") if abs(dx) > abs(dy) else ("up" if dy > 5 else "down" if dy < -5 else "")
        check(f"drone looks toward {name}: arrow points {want}", got == want, f"arrow centre offset: right {dx:.1f} px, up {dy:.1f} px")
    # the arrow comes from the quaternion, not from the direction of motion: yaw +y while the path goes toward -x
    track_flight = np.array([[1.0, 0.0], [0.5, 0.0], [0.0, 0.0]])
    img = panel.render(track_flight, _quat_yaw(math.pi / 2), 0.0, 1.0, 0, 5)
    off = arrow_offset_px(img, panel.px((0.0, 0.0)))
    check("arrow follows the quaternion, not the motion", off is not None and off[1] > 10 and abs(off[0]) < 5, f"offset {off}")
    # a camera looking straight up has no horizontal direction: no arrow, no crash
    img = panel.render(np.array([[0.0, 0.0]]), [-math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0], 0.0, 0.0, 0, 5)
    check("camera straight up/down draws no arrow", arrow_offset_px(img, spot) is None, "no arrow-colored pixels")

    # 4. gate geometry: a gate with yaw +90 deg is passed toward +y, its frame lies along x
    seg = MapPanel(dict(track, gates_quat=np.array([_quat_yaw(math.pi / 2)] * 4)), size=480).gate_segments(0)
    p0, p1 = seg["arrows"][0]
    check("gate with yaw 90 deg: passage arrow points to +y, frame along x",
          p1[1] > p0[1] + 0.3 and abs(p1[0] - p0[0]) < 1e-9 and abs(seg["posts"][0][1][1] - seg["posts"][0][0][1]) < 1e-9,
          f"arrow {np.round(p0, 3).tolist()} -> {np.round(p1, 3).tolist()}")
    seg2 = MapPanel(dict(track, gate_direction=[-1] * 5), size=480).gate_segments(0)
    check("reverse passage draws the arrow the other way", np.dot(seg2["arrows"][0][1] - seg2["arrows"][0][0], seg2["a"]) < 0, "arrow against local x")
    check("gate 2 is used by passages 2 and 5", panel.gate_segments(1)["passes"] == [(2, 1), (5, 1)], str(panel.gate_segments(1)["passes"]))

    # 5. the 12-picture sheet from made-up camera pictures, and three combined pictures
    cam = []
    for k in range(n):
        g = np.zeros((480, 640, 3), dtype=np.uint8)
        g[..., 0] = np.linspace(40, 200, 640, dtype=np.uint8)[None, :]
        g[..., 2] = (40 + 160 * k / n)
        im = Image.fromarray(g)
        ImageDraw.Draw(im).text((40, 200), f"made-up camera picture {k}", fill=(255, 255, 255), font=font(30))
        cam.append(np.asarray(im))
    saves = {k: out / f"sync_picture_{i}.png" for i, k in enumerate(picks)}
    info = make_sync_film(cam, flight, track, None, save_pictures=saves)
    check("combined picture is camera + map side by side", info["width"] == 640 + 480 and info["height"] == 480 and all(p.is_file() for p in saves.values()),
          f"{info['width']} x {info['height']}; mean {info['seconds_per_picture_mean'] * 1000:.1f} ms, max {info['seconds_per_picture_max'] * 1000:.1f} ms per picture")
    check("combined pictures cost well under 0.1 s", info["seconds_per_picture_max"] < 0.1, f"max {info['seconds_per_picture_max'] * 1000:.1f} ms")
    check("map background unchanged after the whole film", info["background_unchanged"], "")
    idx = evenly_spaced_indices(n, 12)
    combined = []
    for k in idx:
        row = int(flight["pic_step"][k])
        mp = panel.render(flight["pos"][: row + 1, :2], flight["quat"][row], flight["time"][row], float(np.linalg.norm(flight["vel"][row])), int(flight["passed"][row]), 5)
        combined.append(np.concatenate([cam[k], mp], axis=1))
    labels = [f"{flight['time'][int(flight['pic_step'][k])]:.2f} s" for k in idx]
    sheet = contact_sheet(combined, labels, cols=4)
    sheet.save(out / "sync_contact.png")
    check("contact sheet: 12 pictures, 4 by 3, at most 1920 wide, first and last included",
          len(idx) == 12 and idx[0] == 0 and idx[-1] == n - 1 and sheet.size[0] <= 1920 and sheet.size[1] > sheet.size[0] // 4,
          f"indices {idx}; sheet {sheet.size[0]} x {sheet.size[1]}")
    check("evenly_spaced_indices handles short films", evenly_spaced_indices(5, 12) == [0, 1, 2, 3, 4] and evenly_spaced_indices(0) == [], "5 pictures -> 5 indices")

    # 6. the mp4 path, only if ffmpeg is there
    if shutil.which("ffmpeg"):
        mp4 = out / "sync_selftest.mp4"
        try:
            make_sync_film(cam[:40], dict(flight, pic_step=flight["pic_step"][:40]), track, mp4)
            _, decoded = read_frames_at(mp4, 1120, 480, None)
            sheet_info = film_contact_sheet(mp4, 1120, 480, flight["time"][flight["pic_step"][:40]], out / "sync_selftest_contact.png")
            check("mp4: ffmpeg decodes as many pictures as were written", decoded == 40 and sheet_info["pictures_decoded"] == 40, f"decoded {decoded} of 40")
        except Exception as exc:  # noqa: BLE001
            check("mp4 written and read back", False, repr(exc))
    else:
        check("mp4 written and read back", True, "skipped: no ffmpeg on this PC")

    # 7. the line chart runs
    chart = out / "line_chart_selftest.png"
    t = np.linspace(0, 20, 200)
    line_chart(chart, [{"title": "test", "ylabel": "deg", "series": [
        {"label": "a", "color": (200, 0, 0), "x": t, "y": 90 * np.sin(t / 3)},
        {"label": "b", "color": (0, 0, 200), "x": t, "y": np.where(t < 10, 40 * np.cos(t / 3), np.nan), "dashed": True}], "hlines": [(90, "90")]}])
    check("line chart written", chart.is_file() and chart.stat().st_size > 1000, str(chart.stat().st_size))

    failed = [r for r in results if not r[1]]
    for name, ok, detail in results:
        print(("PASS  " if ok else "FAIL  ") + name + (f"  [{detail}]" if detail else ""))
    print(f"{len(results) - len(failed)} of {len(results)} passed; files in {out}")
    return 1 if failed else 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--self-test", action="store_true", help="made-up data: write 3 pictures and a contact sheet, run checks")
    parser.add_argument("--out", help="folder for the self-test files")
    args = parser.parse_args()
    if args.self_test:
        if not args.out:
            parser.error("--out is required with --self-test")
        sys.exit(self_test(Path(args.out)))
    parser.error("nothing to do: use --self-test, or import make_sync_film from tests/crazyflow_run_a.py")


if __name__ == "__main__":
    main()
