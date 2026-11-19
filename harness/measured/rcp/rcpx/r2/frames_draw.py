# SPDX-License-Identifier: AGPL-3.0-only
"""Drawing for the frames package: the move_tip arrows, forward ticks and world points on a copy of a look's
annotated frame; the mark echo; the enlarged crop. Pure functions on RGB uint8 arrays and the numbers from
frames_geom; files are read and written only through load_rgb and save_rgb. Nothing here reads ground truth.
"""

import math
import os

import cv2
import numpy as np
from PIL import Image

ARROW_MM = 30.0          # a drawn arrow stands for this much tip travel, clamped to the pixel range below
ARROW_MIN_PX, ARROW_MAX_PX = 22.0, 90.0
COLORS = {"dx+ base": (255, 70, 70), "dy+ base": (60, 225, 60), "dz+ base": (80, 150, 255),
          "dx+ tool": (255, 165, 0), "dz+ tool": (205, 120, 255)}
COLOR_TICK = (255, 255, 255)
COLOR_WORLD = (0, 235, 190)
COLOR_STALE = (165, 165, 165)
COLOR_MARK = (0, 255, 0)
SHADOW = (0, 0, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def load_rgb(src):
    """An H x W x 3 uint8 array from a path or an array, or None."""
    if isinstance(src, np.ndarray):
        a = src
    elif isinstance(src, str) and os.path.exists(src):
        with Image.open(src) as im:
            a = np.asarray(im.convert("RGB"))
    elif isinstance(src, list):
        try:
            a = np.asarray(src, dtype=np.uint8)
        except (TypeError, ValueError):
            return None
    else:
        return None
    if a.ndim == 2:
        a = np.stack([a] * 3, axis=2)
    if a.ndim != 3 or a.shape[2] < 3:
        return None
    return np.ascontiguousarray(a[:, :, :3]).astype(np.uint8)


def save_rgb(img, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    Image.fromarray(np.ascontiguousarray(img[:, :, :3])).save(path)
    return path


def _pt(p):
    return int(round(float(p[0]))), int(round(float(p[1])))


class _Labels:
    """Text on a dark box, moved to the nearest free slot when it would cover an earlier label."""

    def __init__(self, img):
        self.img, self.placed = img, []   # placed: (x0, y0, x1, y1) boxes

    def _free(self, box):
        return not any(box[0] < b[2] and b[0] < box[2] and box[1] < b[3] and b[1] < box[3] for b in self.placed)

    def put(self, s, org, color, scale=0.42):
        h, w = self.img.shape[:2]
        (tw, th), base = cv2.getTextSize(s, FONT, scale, 1)
        step = th + base + 3
        best = None
        for dx, dy in ((0, 0), (0, -step), (0, step), (tw // 2 + 4, 0), (-tw // 2 - 4, 0), (0, -2 * step),
                       (0, 2 * step), (tw + 6, 0), (-tw - 6, 0)):
            x = int(min(max(org[0] + dx, 2), max(2, w - tw - 2)))
            y = int(min(max(org[1] + dy, th + 3), h - base - 2))
            box = (x - 2, y - th - 2, x + tw + 2, y + base + 1)
            if best is None:
                best = (x, y, box)
            if self._free(box):
                best = (x, y, box)
                break
        x, y, box = best
        self.placed.append(box)
        cv2.rectangle(self.img, (box[0], box[1]), (box[2], box[3]), (28, 28, 28), -1)
        cv2.putText(self.img, s, (x, y), FONT, scale, color, 1, cv2.LINE_AA)


def _border_point(start, d, w, h, inset=14.0):
    """Where a ray from start along unit d leaves the frame shrunk by inset."""
    t = []
    for k, lim in ((0, w), (1, h)):
        if d[k] > 1e-9:
            t.append((lim - inset - start[k]) / d[k])
        elif d[k] < -1e-9:
            t.append((inset - start[k]) / d[k])
    reach = max(0.0, min(t)) if t else 0.0
    return start[0] + d[0] * reach, start[1] + d[1] * reach


def draw_axes(img, g):
    """A copy of img with the five move_tip arrows at the tip, the forward ticks and the world points.

    g: frames_geom.look_geometry output. Arrows are anchored at the tip pixel, or at a legend point in the lower
    left when the tip is off frame (the directions are still the ones at the tip)."""
    out = np.ascontiguousarray(img[:, :, :3]).copy()
    h, w = out.shape[:2]
    lab = _Labels(out)
    tip = g["tip"]
    if g["tip_in_view"]:
        anchor = (tip[0], tip[1])
        tu, tv = int(tip[0]), int(tip[1])
        lab.placed += [(tu - 16, tv - 16, tu + 16, tv + 16),     # the round one tip marker and its label
                       (tu + 8, tv - 30, tu + 48, tv - 10),
                       (tu - 70, tv + 10, tu + 70, tv + 46)]     # the round one scale bar and its text
    else:
        # the round one frame writes "tip off frame" and its scale bar in the lower left: keep clear of it, and
        # put the legend in the corner farthest from where the tip is
        lab.placed.append((0, h - 64, 300, h))
        corners = [(w - 110.0, 110.0), (w - 110.0, h - 110.0), (110.0, 120.0)]
        if tip is not None:
            tu, tv = min(max(tip[0], 0.0), w - 1.0), min(max(tip[1], 0.0), h - 1.0)
            corners.sort(key=lambda c: -math.hypot(c[0] - tu, c[1] - tv))
        anchor = corners[0]
    # arrow lengths are clamped to stay visible, so the header never offers them as a ruler (the round one frame
    # carries the scale bar at the tip)
    lab.put("arrows: move_tip directions in this picture (not to scale)", (6, 16), COLOR_TICK)
    _draw_ticks(out, lab, g, anchor)
    arrows = []
    for a in g["axes_px"]:
        if a["dir_px"] is not None:
            L = min(max(a["px_per_mm"] * ARROW_MM, ARROW_MIN_PX), ARROW_MAX_PX)
            arrows.append((L, a))
    arrows.sort(key=lambda la: -la[0])   # long arrows first, so a short one sharing its direction stays on top
    for L, a in arrows:
        color = COLORS.get(a["label"], (255, 255, 255))
        d = a["dir_px"]
        end = (anchor[0] + d[0] * L, anchor[1] + d[1] * L)
        cv2.arrowedLine(out, _pt(anchor), _pt(end), SHADOW, 4, cv2.LINE_AA, tipLength=0.28)
        cv2.arrowedLine(out, _pt(anchor), _pt(end), color, 2, cv2.LINE_AA, tipLength=0.28)
    for L, a in arrows:
        color = COLORS.get(a["label"], (255, 255, 255))
        d = a["dir_px"]
        end = (anchor[0] + d[0] * L, anchor[1] + d[1] * L)
        text = a["label"] + (f" ({a['depth']})" if a["along_view"] else "")
        (tw, _), _ = cv2.getTextSize(text, FONT, 0.42, 1)
        lx = end[0] + d[0] * 6 - (tw if d[0] < -0.3 else (tw / 2 if abs(d[0]) <= 0.3 else 0))
        ly = end[1] + d[1] * 6 + (12 if d[1] > 0.3 else (4 if abs(d[1]) <= 0.3 else -2))
        lab.put(text, (lx, ly), color)
    if not g["tip_in_view"]:
        lab.put("arrows at the tip (tip off frame)" if tip is not None else "tip behind the camera",
                (anchor[0] - 100, anchor[1] + 104), COLOR_TICK)
    _draw_world(out, lab, g)
    return out


def _draw_ticks(out, lab, g, anchor):
    h, w = out.shape[:2]
    ticks = [t for t in g["ticks_px"] if t["px"] is not None]
    if not ticks or not g["tip_in_view"]:
        return
    last = ticks[-1]["px"]
    cv2.line(out, _pt(anchor), _pt(last), SHADOW, 3, cv2.LINE_AA)
    cv2.line(out, _pt(anchor), _pt(last), COLOR_TICK, 1, cv2.LINE_AA)
    for t in ticks:
        u, v = t["px"]
        du, dv = u - anchor[0], v - anchor[1]
        n = math.hypot(du, dv)
        nx, ny = (-dv / n, du / n) if n > 1e-6 else (1.0, 0.0)
        a, b = (u - 6 * nx, v - 6 * ny), (u + 6 * nx, v + 6 * ny)
        cv2.line(out, _pt(a), _pt(b), SHADOW, 3, cv2.LINE_AA)
        cv2.line(out, _pt(a), _pt(b), COLOR_TICK, 1, cv2.LINE_AA)
        if 0 <= u < w and 0 <= v < h:
            lab.put(f"{t['mm']:g}" + (" mm" if t is ticks[-1] else ""), (u + 8 * nx + 3, v + 8 * ny + 4),
                    COLOR_TICK, scale=0.38)


def _draw_world(out, lab, g):
    h, w = out.shape[:2]
    behind = 0
    for e in g["world_px"]:
        color = COLOR_STALE if e["stale"] else COLOR_WORLD
        name = f"{e['label']} ({e['age_calls']} calls ago" + (", stale)" if e["stale"] else ")")
        if e["behind"]:
            behind += 1
            continue
        u, v = e["px"]
        if e["in_view"]:
            cv2.circle(out, _pt((u, v)), 7, SHADOW, 3, cv2.LINE_AA)
            cv2.circle(out, _pt((u, v)), 7, color, 1, cv2.LINE_AA)
            lab.put(name, (u + 10, v - 8), color, scale=0.38)
        else:
            c = (w / 2.0, h / 2.0)
            d = np.array([u - c[0], v - c[1]])
            n = float(np.linalg.norm(d))
            if n < 1e-9:
                continue
            d = d / n
            end = _border_point(c, d, w, h)
            start = (end[0] - d[0] * 18, end[1] - d[1] * 18)
            cv2.arrowedLine(out, _pt(start), _pt(end), color, 1, cv2.LINE_AA, tipLength=0.4)
            lab.put(e["label"] + " (off view)", (start[0] - 20, start[1] - 6), color, scale=0.36)
    if behind:
        lab.put(f"{behind} world point(s) behind the camera", (8, 32), COLOR_STALE, scale=0.38)


# ---------------------------------------------------------------- mark echo
INSET_SRC_PX, INSET_SCALE = 40, 3


def draw_mark(img, u, v, label):
    """A copy of img with the mark: a ring and a crosshair with a gap, so the marked pixel itself stays visible,
    the label, and a 3x inset of the area around it in the corner farthest from the mark."""
    out = np.ascontiguousarray(img[:, :, :3]).copy()
    h, w = out.shape[:2]
    c = _pt((u, v))
    for color, th in ((SHADOW, 3), (COLOR_MARK, 1)):
        cv2.circle(out, c, 9, color, th, cv2.LINE_AA)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            cv2.line(out, (c[0] + 12 * dx, c[1] + 12 * dy), (c[0] + 20 * dx, c[1] + 20 * dy), color, th,
                     cv2.LINE_AA)
    lab = _Labels(out)
    lab.put(f"{label} ({u:.1f}, {v:.1f})", (c[0] + 14, c[1] - 14), COLOR_MARK)
    half = INSET_SRC_PX // 2
    u0 = int(min(max(int(math.floor(u)) - half, 0), max(0, w - INSET_SRC_PX)))
    v0 = int(min(max(int(math.floor(v)) - half, 0), max(0, h - INSET_SRC_PX)))
    src = np.ascontiguousarray(img[v0:v0 + INSET_SRC_PX, u0:u0 + INSET_SRC_PX, :3])
    if src.shape[0] < 2 or src.shape[1] < 2:
        return out
    side = INSET_SRC_PX * INSET_SCALE
    big = cv2.resize(src, (src.shape[1] * INSET_SCALE, src.shape[0] * INSET_SCALE), interpolation=cv2.INTER_NEAREST)
    bu, bv = (u - u0) * INSET_SCALE, (v - v0) * INSET_SCALE
    cv2.circle(big, _pt((bu, bv)), 2 * INSET_SCALE + 3, COLOR_MARK, 1, cv2.LINE_AA)
    ox = 4 if u > w / 2 else w - side - 4
    oy = 4 if v > h / 2 else h - side - 4
    if ox < 0 or oy < 0:
        return out
    out[oy:oy + big.shape[0], ox:ox + big.shape[1]] = big
    cv2.rectangle(out, (ox - 1, oy - 1), (ox + big.shape[1], oy + big.shape[0]), COLOR_MARK, 1)
    return out


# ---------------------------------------------------------------- crop
def crop_box(cu, cv, size, w, h):
    """(u0, v0, side): a size x size box centred on (cu, cv), shifted to stay inside a w x h frame."""
    side = int(min(size, w, h))
    u0 = int(round(cu - side / 2.0))
    v0 = int(round(cv - side / 2.0))
    u0 = min(max(u0, 0), w - side)
    v0 = min(max(v0, 0), h - side)
    return u0, v0, side


def enlarge(img, u0, v0, side, factor):
    """The square cut from img and resampled by factor (bicubic)."""
    sub = np.ascontiguousarray(img[v0:v0 + side, u0:u0 + side, :3])
    n = max(1, int(round(side * factor)))
    if n == side:
        return sub.copy()
    return cv2.resize(sub, (n, n), interpolation=cv2.INTER_CUBIC)
