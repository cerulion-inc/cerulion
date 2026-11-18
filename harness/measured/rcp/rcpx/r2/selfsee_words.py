# SPDX-License-Identifier: AGPL-3.0-only
"""The view in words, and the own-body outlines drawn on a look (package selfsee).

Pure geometry and drawing: a camera pose (from the body model at a look's joint pose), the own-body grid of
selfsee_body and the side words of rcpx/side_words.py in, words and an image out. No robot or device is named.

    camera(model, q)                  (camera position mm, Rc: columns image right, image up, toward the viewer)
    elevation_deg(Rc)                 asin(-axis_z): 90 looks straight down
    kind_of(elevation)                "top view" | "looking down at E deg" | "level view" | "looking up"
    reference(model, q, floor_z_mm)   the point the side words are read at: the floor point under the image centre,
                                      else the camera position
    roll_deg(Rc, ref, top)            the angle from image up to world up projected (top views: to "away from the
                                      robot" at the reference point), positive toward image right
    block(...)                        the view block {says, kind, elevation_deg, roll_deg, up, right, own_body,
                                      below_tip_px}, at most VIEW_BLOCK_MAX_BYTES as JSON
    draw(img, ...)                    outlines labelled own hand / own arm / own base and one arrow ("toward robot"
                                      on top views, "up" otherwise), placed outside every candidate box and the
                                      image's central share; returns (image, drawn pixel mask, placement notes)
"""

import json
import math

import cv2
import numpy as np

from rcpx import r3_limits as L
from rcpx import side_words as SW

MM = 1000.0
WORD = {"own_base": "base", "own_arm": "arm", "own_hand": "hand"}
LABEL = {"own_base": "own base", "own_arm": "own arm", "own_hand": "own hand"}
COLOUR = {"own_hand": (255, 220, 0), "own_arm": (0, 210, 255), "own_base": (255, 0, 255)}   # RGB outlines
ARROW_MM = 50.0              # the world step projected to find an arrow's direction in the image
ARROW_PX = 44                # the drawn arrow's length
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE, FONT_THICK, LABEL_PAD = 0.45, 1, 3
OUTLINE_PX = 2
ROLL_TEXT = "a standing thing can look as if it lies flat"


def camera(model, q):
    pc, Rc = model.fk(np.asarray(q, dtype=float))["camera"]
    return np.asarray(pc, dtype=float) * MM, np.asarray(Rc, dtype=float)


def elevation_deg(Rc):
    axis = -Rc[:, 2]
    return math.degrees(math.asin(max(-1.0, min(1.0, -float(axis[2])))))


def kind_of(elevation):
    if elevation >= L.TOP_VIEW_DEG:
        return "top view"
    if elevation >= L.DOWN_VIEW_DEG:
        return f"looking down at {int(round(elevation))} deg"
    if elevation >= L.LEVEL_VIEW_DEG:
        return "level view"
    return "looking up"


def reference(pc_mm, Rc, floor_z_mm):
    """The floor point under the image centre (where the optical axis meets the floor), else the camera itself."""
    axis = -Rc[:, 2]
    if axis[2] < -1e-6:
        t = (float(floor_z_mm) - float(pc_mm[2])) / float(axis[2])
        if t > 0.0:
            return pc_mm + t * axis
    return pc_mm.copy()


def roll_deg(Rc, ref_dir):
    """The angle from image up to ref_dir projected on the image plane, positive toward image right; None when
    ref_dir runs along the optical axis."""
    wx, wy = float(np.asarray(ref_dir) @ Rc[:, 0]), float(np.asarray(ref_dir) @ Rc[:, 1])
    if math.hypot(wx, wy) < 1e-6:
        return None
    return math.degrees(math.atan2(wx, wy))


def view_dir_words(h, ref_mm):
    """Image direction in eight horizontal sectors. Edges are geometric halves between the eight compass
    directions, not a tuned limit. Mostly vertical and zero directions retain the shared side words."""
    ordinary = SW.words(h, ref_mm)
    if ordinary in ("up", "down", "nowhere (a zero direction)"):
        return ordinary
    h = np.asarray(h, dtype=float).reshape(3)
    r = SW.radial(ref_mm)
    angle = math.degrees(math.atan2(float(np.cross(r, h)[2]), float(r @ h)))
    a = round(abs(angle), 12)  # remove floating point roundoff at the exact sector halves
    sector = 360.0 / 8.0
    side = "left" if angle > 0.0 else "right"
    if a <= sector / 2.0:
        return "away from the robot"
    if a < 1.5 * sector:
        return f"away from the robot and to its {side}"
    if a <= 2.5 * sector:
        return f"to the robot's {side}"
    if a < 3.5 * sector:
        return f"toward the robot and to its {side}"
    return "toward the robot"


def where_word(u, v, W, H):
    """Where a pixel lies in the frame, in words: top, middle or bottom, then left, middle or right."""
    row = "top" if v < H / 3.0 else ("bottom" if v >= 2.0 * H / 3.0 else "middle")
    col = "left" if u < W / 3.0 else ("right" if u >= 2.0 * W / 3.0 else "middle")
    return "middle" if row == col == "middle" else f"{row} {col}"


def _size(obj):
    return len(json.dumps(obj))


def block(model, q, rows, centroids, floor_z_mm, tip_mm):
    """The view block of a look at joint pose q (module text). rows: selfsee_body mask rows (group, box, share);
    centroids: {group: (u, v)}; tip_mm: the tip position (frame B)."""
    W, H = int(model.cam_w), int(model.cam_h)
    pc, Rc = camera(model, q)
    elev = elevation_deg(Rc)
    kind = kind_of(elev)
    top = elev >= L.TOP_VIEW_DEG
    ref = reference(pc, Rc, floor_z_mm)
    up = view_dir_words(Rc[:, 1], ref)
    right = view_dir_words(Rc[:, 0], ref)
    rdir = SW.radial(ref) if top else np.array([0.0, 0.0, 1.0])
    roll = roll_deg(Rc, rdir)
    below = None
    if tip_mm is not None:
        u, v, depth = model.project(q, np.array([tip_mm[0], tip_mm[1], floor_z_mm]) / MM)
        if depth > 1e-3 and math.isfinite(u) and 0.0 <= u < W and 0.0 <= v < H:
            below = [int(round(u)), int(round(v))]
    height = int(round(float(pc[2]) - float(floor_z_mm)))
    head = f"{kind} from {height} mm above the table; image up = {up}, image right = {right}"
    parts = [(WORD[g], int(round(100.0 * s)), where_word(*centroids[g], W, H)) for g, _, s in rows]
    rolled = roll is not None and abs(roll) >= L.ROLL_WARN_DEG

    def says(where=None, roll_long=True, include_right=True, compact=False, body_words=True):
        """The sentence; where: how many groups (largest first) keep their position words (None: all)."""
        n = len(parts) if where is None else where
        if parts:
            body = "own body: " + ", ".join(f"{w} {p} percent" + (f" {at}" if k < n else "")
                                            for k, (w, p, at) in enumerate(parts))
        else:
            body = "no own body in view"
        short_head = head if include_right else head.split(", image right = ", 1)[0]
        if compact:
            short_head = f"{kind}; image up = {up}"
        s = f"{short_head}; {body}" if body_words else short_head
        if rolled:
            s += f"; rolled {int(round(abs(roll)))} deg" + (f": {ROLL_TEXT}" if roll_long else "")
        return s

    def shares(boxes):
        return {WORD[g]: ([round(s, 2), b] if k < boxes else [round(s, 2)]) for k, (g, b, s) in enumerate(rows)}

    out = {"says": says(), "kind": kind, "elevation_deg": round(elev, 1),
           "roll_deg": None if roll is None else round(roll, 1), "up": up, "right": right,
           "own_body": shares(len(rows)), "below_tip_px": below}
    # within the byte bound the sentence a model reads is cut last: first the boxes leave own_body (the largest
    # group's last), then the position words leave the sentence (the largest group's last), then the roll's reason
    steps = [lambda o: o.update(own_body=shares(1)), lambda o: o.update(own_body=shares(0))]
    steps += [lambda o, k=k: o.update(says=says(where=k)) for k in range(len(parts) - 1, -1, -1)]
    steps += [lambda o: o.update(says=says(where=0, roll_long=False))]
    steps += [lambda o: o.update(says=says(where=0, roll_long=False, include_right=False))]
    # Long diagonal words on an oblique, rolled view can still overflow. The compact sentence keeps kind,
    # image up, every body share and the roll word; right stays in its key, height in the look's camera pose.
    steps += [lambda o: o.update(says=says(where=0, roll_long=False, include_right=False, compact=True))]
    # the roll word never leaves the sentence (a rolled view read as upright was the failure it is there for): the
    # body shares leave it last (they stay in own_body)
    steps += [lambda o: o.update(says=says(where=0, roll_long=False, include_right=False, compact=True,
                                           body_words=False))]
    for step in steps:
        if _size(out) <= L.VIEW_BLOCK_MAX_BYTES:
            break
        step(out)
    return out


# ---------------------------------------------------------------- drawing
def _inside(r, W, H):
    return r[0] >= 0 and r[1] >= 0 and r[2] <= W and r[3] <= H


def centre_box(W, H):
    """The image's central share (DRAW_CENTRE_FRAC of its area, the frame's own proportions)."""
    k = math.sqrt(L.DRAW_CENTRE_FRAC)
    w, h = W * k, H * k
    return [int(math.floor((W - w) / 2.0)), int(math.floor((H - h) / 2.0)), int(math.ceil((W + w) / 2.0)),
            int(math.ceil((H + h) / 2.0))]


def _text_size(text):
    (tw, th), base = cv2.getTextSize(text, FONT, FONT_SCALE, FONT_THICK)
    return tw + 2 * LABEL_PAD, th + base + 2 * LABEL_PAD


def _border_anchors(w, h, W, H, step=24):
    """Positions along the frame's border, clockwise from the top-left corner (a last resort)."""
    out = []
    for x in range(4, W - w - 3, step):
        out.append((x, 4))
    for y in range(4, H - h - 3, step):
        out.append((W - w - 4, y))
    for x in range(W - w - 4, 3, -step):
        out.append((x, H - h - 4))
    for y in range(H - h - 4, 3, -step):
        out.append((4, y))
    return out


def _label(img, r, text):
    cv2.rectangle(img, (r[0], r[1]), (r[2] - 1, r[3] - 1), (0, 0, 0), -1)
    th = r[3] - r[1] - 2 * LABEL_PAD
    cv2.putText(img, text, (r[0] + LABEL_PAD, r[1] + LABEL_PAD + th - 3), FONT, FONT_SCALE, (255, 255, 255),
                FONT_THICK, cv2.LINE_AA)


def group_masks(codes, codes_of, W, H):
    """{group: H x W bool}: each own-body group's grid cells, smoothed up to the frame's size."""
    out = {}
    for g, c in codes_of.items():
        low = (codes == c).astype(np.float32)
        if not low.any():
            continue
        out[g] = cv2.resize(low, (W, H), interpolation=cv2.INTER_LINEAR) >= 0.5
    return out


def _box_mask(boxes, W, H):
    """H x W bool: True inside any of the pixel boxes (each widened to whole pixels)."""
    out = np.zeros((H, W), dtype=bool)
    for b in boxes:
        u0, v0 = max(0, int(math.floor(b[0]))), max(0, int(math.floor(b[1])))
        u1, v1 = min(W, int(math.ceil(b[2]))), min(H, int(math.ceil(b[3])))
        if u1 > u0 and v1 > v0:
            out[v0:v1, u0:u1] = True
    return out


def _free_rect(r, blocked, W, H):
    return _inside(r, W, H) and not bool(blocked[r[1]:r[3], r[0]:r[2]].any())


def _arrow_layer(W, H, p0, p1, tr, text):
    """The arrow and its label drawn on a blank layer: (RGB layer, its drawn pixels)."""
    layer = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.arrowedLine(layer, p0, p1, (1, 1, 1), 4, cv2.LINE_AA, tipLength=0.3)
    cv2.arrowedLine(layer, p0, p1, (255, 255, 255), 2, cv2.LINE_AA, tipLength=0.3)
    _label(layer, tr, text)
    mask = np.any(layer > 0, axis=2)
    mask[tr[1]:tr[3], tr[0]:tr[2]] = True
    return layer, mask


def draw(img, masks, rows, arrow, avoid):
    """(image, drawn mask, notes): img with the own-body outlines, their labels and one arrow drawn on a copy.
    masks: {group: H x W bool}; rows: the mask rows (group, box, share), the drawing order; arrow: (text,
    (du, dv) unit image direction) or None; avoid: pixel boxes nothing may be drawn in (the candidate boxes).
    Labels and the arrow are tried at fixed anchors in order and placed at the first whose drawn pixels miss every
    avoid box, the central box and what was placed before; outline pixels inside an avoid box are left out; at the
    end every pixel inside an avoid box is the source's again, so nothing drawn covers a candidate."""
    src = np.ascontiguousarray(np.asarray(img)[:, :, :3]).astype(np.uint8)
    H, W = src.shape[:2]
    out = src.copy()
    cand = _box_mask(avoid, W, H)
    lines = np.zeros_like(out)
    drawn_line = np.zeros((H, W), dtype=bool)
    for g, _, _ in rows:
        mk = masks.get(g)
        if mk is None:
            continue
        cs, _ = cv2.findContours(mk.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        layer = np.zeros((H, W), dtype=np.uint8)
        cv2.drawContours(layer, cs, -1, 1, OUTLINE_PX)
        on = (layer > 0) & ~cand
        lines[on] = COLOUR[g]
        drawn_line |= on
    out[drawn_line] = lines[drawn_line]
    blocked = cand | _box_mask([centre_box(W, H)], W, H)
    notes = []
    for g, box, _ in rows:
        text = LABEL[g]
        w, h = _text_size(text)
        u0, v0, u1, v1 = box
        anchors = [(u0, v0 - h - 2), (u0, v1 + 2), (u1 + 2, v0), (u0 - w - 2, v0), (u1 - w, v0 - h - 2),
                   (u1 - w, v1 + 2), (u0 + 2, v0 + 2), (u1 - w - 2, v1 - h - 2)] + _border_anchors(w, h, W, H)
        r = None
        for x, y in anchors:
            cand_r = [int(round(x)), int(round(y)), int(round(x)) + w, int(round(y)) + h]
            if _free_rect(cand_r, blocked, W, H):
                r = cand_r
                break
        if r is None:
            notes.append(f"no free place for the label {text}")
            continue
        _label(out, r, text)
        blocked[max(0, r[1] - 2):r[3] + 2, max(0, r[0] - 2):r[2] + 2] = True
    if arrow is not None:
        text, (du, dv) = arrow
        tw, th = _text_size(text)
        ex, ey = du * ARROW_PX, dv * ARROW_PX
        gap = 6
        starts = [(60, 60), (W - 60, 60), (60, H - 60), (W - 60, H - 60), (W / 2.0, 50), (W / 2.0, H - 50),
                  (50, H / 2.0), (W - 50, H / 2.0)]
        placed = None
        for sx, sy in starts:
            # the text sits beside the arrow's tail, away from where it points
            tx = int(round(sx - du * (tw / 2.0 + gap) - tw / 2.0))
            ty = int(round(sy - dv * (th / 2.0 + gap) - th / 2.0))
            tr = [tx, ty, tx + tw, ty + th]
            p0, p1 = (int(round(sx)), int(round(sy))), (int(round(sx + ex)), int(round(sy + ey)))
            if not _inside(tr, W, H) or not all(0 <= x < W and 0 <= y < H for x, y in (p0, p1)):
                continue
            layer, mask = _arrow_layer(W, H, p0, p1, tr, text)
            if not bool((mask & blocked).any()):
                placed = (layer, mask)
                break
        if placed is None:
            notes.append("no free place for the arrow")
        else:
            layer, mask = placed
            out[mask] = layer[mask]
    out[cand] = src[cand]
    drawn = np.any(out != src, axis=2)
    return out, drawn, notes


def arrow_for(model, q, ref_mm, top):
    """(text, unit image direction) of the one arrow: "toward robot" on a top view (toward the base axis at the
    reference point), "up" otherwise (world up); None when the direction runs along the optical axis."""
    w = -SW.radial(ref_mm) if top else np.array([0.0, 0.0, 1.0])
    a = model.project(q, np.asarray(ref_mm, dtype=float) / MM)
    b = model.project(q, (np.asarray(ref_mm, dtype=float) + ARROW_MM * w) / MM)
    if not (a[2] > 1e-3 and b[2] > 1e-3) or not all(math.isfinite(x) for x in (a[0], a[1], b[0], b[1])):
        pc, Rc = camera(model, q)
        du, dv = float(w @ Rc[:, 0]), -float(w @ Rc[:, 1])
    else:
        du, dv = b[0] - a[0], b[1] - a[1]
    n = math.hypot(du, dv)
    if n < 1e-6:
        return None
    return ("toward robot" if top else "up"), (du / n, dv / n)


def box_px(model, q, centre_mm, size_mm, floor_z_mm, top_mm):
    """The pixel box a floor-standing box (centre [x, y], size [sx, sy], from the floor up to top_mm) covers in the
    frame at q, or None when no corner is in front of the camera."""
    cx, cy = float(centre_mm[0]), float(centre_mm[1])
    hx, hy = abs(float(size_mm[0])) / 2.0, abs(float(size_mm[1])) / 2.0
    us, vs = [], []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for z in (float(floor_z_mm), float(top_mm)):
                u, v, depth = model.project(q, np.array([cx + sx * hx, cy + sy * hy, z]) / MM)
                if depth > 1e-3 and math.isfinite(u) and math.isfinite(v):
                    us.append(u)
                    vs.append(v)
    if not us:
        return None
    return [min(us), min(vs), max(us), max(vs)]


__all__ = ["arrow_for", "block", "box_px", "camera", "centre_box", "draw", "elevation_deg", "group_masks",
           "kind_of", "reference", "roll_deg", "view_dir_words", "where_word"]
