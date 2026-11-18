# SPDX-License-Identifier: AGPL-3.0-only
"""The looks of package place (N4): the colour of what is about to be picked, read from the wrist frame at the pick
point, and after a place one look whose sentence says whether that colour shows where the object was set down.

Only the robot's own camera and its body model are read: the point is projected into the frame through the camera
pose of the joint reading (RobotModel.project), and a small patch around it is averaged. A colour match is evidence,
not proof (a table or a mark of the same colour reads the same), and the sentence says so when it matters.

Every number named below is an image numeric (pixels, colour distances, shares of a frame): none bounds a motion or a
force. A length the outline returns carries its own error: the size of one pixel at that depth (err_m).
"""

import math

import numpy as np

from rcpx import perception

PATCH_PX = 4             # numeric: half side of the patch averaged around a projected point
MATCH_RGB = 45.0         # numeric: colour distance (0 to 441) under which two patches read as the same colour
EDGE_PX = 2              # numeric: a projected point this near the frame edge reads as outside the view
FILL_TOLS = (28, 18, 12)  # numeric: flood fill: a pixel within this of the seed colour (per channel) joins the object
MAX_SHARE = 0.4          # numeric: a fill over this share of the frame is the background, not an object
WINDOW_PX = 120          # numeric: the fill stays inside this half window around the seed
MIN_PIXELS = 25          # numeric
MIN_SIDE_M = 0.006       # numeric: a filled shape narrower than this is a mark, not an object
SIDE_SPAN = 1.6          # numeric: the fill's window and its longest side, as a multiple of the jaws' opening
SIDE_SLACK_M = 0.02      # numeric: ... plus this
MIN_EDGE = 8             # numeric: outline points needed for a rectangle
BLUR = 3                 # numeric: the blur kernel's side
FILL_FLAGS = 4 | (255 << 8)   # numeric: four-way fill, mask value 255
MM = 1000.0              # unit
HALF = 0.5               # numeric: a pixel's middle
EPS = 1e-6               # numeric


def patch(img, u, v, half=PATCH_PX):
    """Mean RGB of the square patch around pixel (u, v), or None when it lies outside the frame."""
    if img is None or not (math.isfinite(u) and math.isfinite(v)):
        return None
    h, w = img.shape[:2]
    ui, vi = int(round(u)), int(round(v))
    if not (EDGE_PX <= ui < w - EDGE_PX and EDGE_PX <= vi < h - EDGE_PX):
        return None
    p = img[max(0, vi - half):vi + half + 1, max(0, ui - half):ui + half + 1, :3].reshape(-1, 3).astype(float)
    return [round(float(x), 1) for x in p.mean(axis=0)]


def colour_at(ex, q, point_m):
    """(colour, pixel, image) of the wrist frame at the projection of point_m with the arm at q, or None parts.
    A frame takes no world time."""
    img = np.asarray(ex.backend.render("wrist"))
    u, v, depth = ex.model.project(q, point_m)
    if depth <= 0:
        return None, None, img
    return patch(img, u, v), [round(u, 1), round(v, 1)], img


def footprint(ex, q, point_m, img, max_side_m):
    """What one wrist frame shows of the object at point_m: flood fill from the pixel the point projects to (the
    object's own colour), the filled pixels back-projected through the camera onto the horizontal plane through
    point_m (its top face), and the smallest rectangle around them. {"centre_m", "bearing_rad" (across the narrow
    side), "narrow_m", "wide_m", "pixels", "err_m" (the size of one pixel at that depth: the error of every
    length here)} or {"none": why}. Evidence from a look, not a measurement: a fill that touches its window's
    edge, or a side outside [MIN_SIDE_M, max_side_m * SIDE_SPAN], is not used."""
    import cv2   # here: only a look needs it
    if img is None:
        return {"none": "no frame"}
    u, v, depth = ex.model.project(q, point_m)
    h, w = img.shape[:2]
    if depth <= 0 or not (EDGE_PX <= u < w - EDGE_PX and EDGE_PX <= v < h - EDGE_PX):
        return {"none": "the point is outside the wrist view"}
    ui, vi = int(round(u)), int(round(v))
    err_m = float(depth) / float(ex.model.f_px)
    half = max(WINDOW_PX, int(SIDE_SPAN * float(max_side_m) / max(err_m, EPS)))
    x0, y0 = max(0, ui - half), max(0, vi - half)
    x1, y1 = min(w, ui + half + 1), min(h, vi + half + 1)
    sub = np.ascontiguousarray(img[y0:y1, x0:x1, :3]).astype(np.uint8)
    sub = cv2.GaussianBlur(sub, (BLUR, BLUR), 0)
    m, why = None, ""
    for tol in FILL_TOLS:          # the widest fill that stays inside its window and leaves the frame's middle
        mask = np.zeros((sub.shape[0] + 2, sub.shape[1] + 2), np.uint8)
        flags = FILL_FLAGS | cv2.FLOODFILL_FIXED_RANGE | cv2.FLOODFILL_MASK_ONLY
        cv2.floodFill(sub, mask, (ui - x0, vi - y0), (0, 0, 0), (tol,) * 3, (tol,) * 3, flags)
        mm = mask[1:-1, 1:-1] > 0
        n = int(mm.sum())
        if n < MIN_PIXELS:
            why = f"the fill found only {n} pixels"
            continue
        inner = [(mm[0, :].any() and y0 > 0), (mm[-1, :].any() and y1 < h), (mm[:, 0].any() and x0 > 0),
                 (mm[:, -1].any() and x1 < w)]
        if any(inner) or n > MAX_SHARE * h * w:
            why = "the fill ran to the edge of its window: the colour does not stand out"
            continue
        m = mm
        break
    if m is None:
        return {"none": why}
    n = int(m.sum())
    cut = bool(m[0, :].any() or m[-1, :].any() or m[:, 0].any() or m[:, -1].any())
    ys, xs = np.nonzero(m)
    edge = (cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_GRADIENT, np.ones((BLUR, BLUR), np.uint8)) > 0)
    ys, xs = np.nonzero(edge & m) if (edge & m).sum() >= MIN_EDGE else (ys, xs)
    pts = []
    zp = float(point_m[2])
    for py, px in zip(ys + y0, xs + x0):
        o, dvec = ex.model.pixel_ray(q, float(px) + HALF, float(py) + HALF)
        if abs(float(dvec[2])) < EPS:
            continue
        k = (zp - float(o[2])) / float(dvec[2])
        if k > 0:
            pts.append([(o[0] + k * dvec[0]) * MM, (o[1] + k * dvec[1]) * MM])
    if len(pts) < MIN_EDGE:
        return {"none": "the fill does not meet the plane of the point"}
    (cx, cy), (a, b), ang = cv2.minAreaRect(np.asarray(pts, np.float32))
    a, b = float(a) / MM, float(b) / MM
    th = math.radians(float(ang))
    if a > b:                      # close across the narrow side: along the side of length a's normal...
        th += HALF * math.pi
    narrow, wide = min(a, b), max(a, b)
    if narrow < MIN_SIDE_M or wide > SIDE_SPAN * float(max_side_m) + SIDE_SLACK_M:
        return {"none": f"the filled shape ({narrow * MM:.0f} by {wide * MM:.0f} mm) is not an object the jaws "
                        "could hold"}
    out = {"centre_m": [float(cx) / MM, float(cy) / MM, zp], "bearing_rad": th, "narrow_m": narrow,
           "wide_m": wide, "pixels": n, "err_m": err_m}
    if cut:     # the object runs out of the frame: its middle and size are not seen, its edges' bearing may be
        out = {"bearing_rad": th, "narrow_m": narrow, "wide_m": wide, "pixels": n, "cut_by_frame": True,
               "err_m": err_m}
    return out


def distance(a, b):
    if a is None or b is None:
        return None
    return round(float(np.linalg.norm(np.asarray(a, float) - np.asarray(b, float))), 1)


def after_place(ex, point_m, colour_before, note="after place"):
    """One look after a place: (look result, sentence, evidence). The look is the round one look (look id, frame,
    annotated, can_see); the sentence says what it shows at the set-down point."""
    lk = ex._look_impl(note)
    s = ex._sense()
    u, v, depth = ex.model.project(s.q, point_m)
    rec = ex._looks.get(lk.get("look_id")) or {}
    img = rec.get("img")
    if img is None:
        img = np.asarray(ex.backend.render("wrist"))
    seen = None if depth <= 0 else patch(img, u, v)
    d = distance(seen, colour_before)
    ev = {"look_id": lk.get("look_id"), "point_px": None if depth <= 0 else [round(u, 1), round(v, 1)],
          "colour_there": seen, "colour_picked": colour_before, "colour_distance": d,
          "match_under": MATCH_RGB}
    if lk.get("can_see") == "dark":
        return lk, f"Look {lk.get('look_id')} is dark: it shows nothing; look again when there is light.", ev
    if seen is None:
        return lk, (f"Look {lk.get('look_id')} does not see the set-down point from here (it is outside the wrist "
                    "view); a look from another pose can show it (the move there is a move's price, the look 0 control periods)."), ev
    if colour_before is None:
        return lk, (f"Look {lk.get('look_id')} shows the set-down point at pixel ({u:.0f}, {v:.0f}); no colour of "
                    "the object was read at the pick, so look at the frame to judge it."), ev
    if d is not None and d <= MATCH_RGB:
        ev["colour_match"] = True
        return lk, (f"Look {lk.get('look_id')} shows the picked object's colour at the set-down point (pixel "
                    f"({u:.0f}, {v:.0f}), colour distance {d:g}): it rests there as far as one look can say."), ev
    ev["colour_match"] = False
    return lk, (f"Look {lk.get('look_id')} does not show the picked object's colour at the set-down point (pixel "
                f"({u:.0f}, {v:.0f}), colour distance {d:g}): it may have tipped, rolled or stayed in the jaws; look "
                "before the next step."), ev


def can_see(img):
    return perception.can_see(img) if img is not None else None
