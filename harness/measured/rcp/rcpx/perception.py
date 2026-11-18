# SPDX-License-Identifier: AGPL-3.0-only
"""Perception helpers for the rcpx runtime.

Pure functions on images and geometry. Nothing here reads simulator ground truth: every input is a
rendered frame, a pose from the body model, or a stored world entry.

Images are H x W x 3 uint8 RGB, pixels u right, v down.
"""

from __future__ import annotations
import math

from rcpx import bounds_ratios as R

import cv2
import numpy as np
from PIL import Image

# CAN I SEE
#
# Brightness is the mean Rec.601 luma of the frame on the 0 to 255 scale. No level is typed as a limit: the levels
# follow the camera's own noise (the hidden level and the noise the robot read on its camera in its
# still hold, handed in as cam = {"hidden", "noise"}; a caller that has none gets the frame's own rounding).
# dark_level: a frame whose mean is not over the hidden level by more than ONE_READ x the noise (never under one
#   code step of the 8 bit frame) shows no signal: sensor noise with no usable edges, and no decision may be taken
#   from it. A lamp glowing in a dark room does not change that: the room is dark. With nothing measured: 5 of 255.
# dim_level: DIM_PER_DARK times that signal over the hidden level (with nothing measured: 40 of 255, about 2 percent
#   linear luminance through the sRGB curve). Under it edges survive but contrast is poor, colours shift and small
#   parts blur into the background: the frame can locate large things, it cannot decide a small state.
# A frame whose mean is under dim_level can still be a LIT scene seen mostly as dark surfaces (a dark machine face
#   filling the view, the black space past a table's edge). The light is read from what is lit: when at least
#   LIT_FRAC of the frame is at LIT_LEVEL or brighter, not counting compact bright blobs (a light source, see
#   bright_spots), the frame is ok. Under working light the brightest ordinary surfaces in view (a white or light
#   grey face, a textured table) print at 130 or more; under dim light they stay near 70 (measured frames,
#   the lit frames that read dim by their mean had a 98th percentile of 128 to 255, the dim setting's
#   44 to 71).
# A compact bright blob (bright_spots: at SPOT_LEVEL or brighter, at most SPOT_MAX_FRAC of the frame, filling at
#   least SPOT_FILL of its bounding box, sides within SPOT_ASPECT of each other) is a light source, not lit surface:
#   it never makes a frame ok, and it is reported so an emissive cue (a lit lamp) can be seen on a dark or dim frame
#   and a working camera told from a dead one.
# ---------------------------------------------------------------------------------------------
CODE_STEP = 1        # the rounding step of an 8 bit frame: the least noise a frame can show
DIM_PER_DARK = 8.0   # image numeric: a frame decides small states from eight times the signal that shows anything
PIXEL = 1            # a match is not placed finer than a pixel: the hidden level of a fit between two frames


def noise_level(cam=None):
    """The least signal a frame can show over its hidden level: ONE_READ x the camera's own noise, the noise never
    under one code step of the frame. cam: {"hidden", "noise"} in grey levels of 255 as the robot measured them
    on its own camera (either may be missing: 0)."""
    return R.ONE_READ * max(float((cam or {}).get("noise") or 0.0), CODE_STEP)


def dark_level(cam=None):
    """Dark = no signal over the camera's hidden level by more than noise_level (5 of 255 with nothing measured)."""
    return float((cam or {}).get("hidden") or 0.0) + noise_level(cam)


def dim_level(cam=None):
    """Small-state signal is DIM_PER_DARK times the dark signal above the camera's hidden level."""
    hidden = float((cam or {}).get("hidden") or 0.0)
    return hidden + DIM_PER_DARK * (dark_level(cam) - hidden)


LIT_LEVEL = 100
LIT_FRAC = 0.01
SPOT_LEVEL = 80
SPOT_MAX_FRAC = 0.3
SPOT_MIN_PX = 4
SPOT_FILL = 0.6
SPOT_ASPECT = 2.0
SPOTS_SHOWN = 3


def _to_gray(img: np.ndarray) -> np.ndarray:
    a = np.asarray(img)
    if a.ndim == 2:
        return a.astype(np.uint8, copy=False)
    if a.ndim == 3 and a.shape[2] >= 3:
        return cv2.cvtColor(np.ascontiguousarray(a[:, :, :3]).astype(np.uint8), cv2.COLOR_RGB2GRAY)
    raise ValueError("image must be H x W or H x W x 3")


def brightness(img: np.ndarray) -> float:
    """Mean luma on the 0 to 255 scale."""
    a = np.asarray(img)
    if a.size == 0:
        return 0.0
    if a.ndim == 3 and a.shape[2] >= 3:
        f = a[:, :, :3].astype(np.float64)
        return float((0.299 * f[:, :, 0] + 0.587 * f[:, :, 1] + 0.114 * f[:, :, 2]).mean())
    return float(a.astype(np.float64).mean())


def _components(gray):
    """(labels, [(label, area, x, y, w, h, cx, cy, compact)]) of the regions at SPOT_LEVEL or brighter."""
    mask = (gray >= SPOT_LEVEL).astype(np.uint8)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    total = gray.size
    out = []
    for i in range(1, n):
        x, y, w, h, area = (int(v) for v in stats[i])
        fill = area / float(max(1, w * h))
        aspect = max(w, h) / float(max(1, min(w, h)))
        compact = (SPOT_MIN_PX <= area <= SPOT_MAX_FRAC * total and fill >= SPOT_FILL and aspect <= SPOT_ASPECT)
        out.append((i, area, x, y, w, h, float(cents[i][0]), float(cents[i][1]), compact))
    return labels, out


def lit_fraction(img: np.ndarray) -> float:
    """Share of the frame at LIT_LEVEL or brighter that is lit surface (compact bright blobs left out)."""
    gray = _to_gray(img)
    lit = gray >= LIT_LEVEL
    if not lit.any():
        return 0.0
    labels, comps = _components(gray)
    spots = [c[0] for c in comps if c[8]]
    if spots:
        lit &= ~np.isin(labels, spots)
    return float(lit.mean())


def bright_spots(img: np.ndarray, shown: int = SPOTS_SHOWN) -> list:
    """The compact bright blobs of a frame, largest first: [{centre_px [u, v], radius_px, area_px, rgb, luma}].
    A lit lamp in a dark room is one; a lit surface is not."""
    a = np.asarray(img)
    gray = _to_gray(a)
    labels, comps = _components(gray)
    rows = sorted((c for c in comps if c[8]), key=lambda c: (-c[1], c[0]))[:shown]
    out = []
    for i, area, x, y, w, h, cx, cy, _ in rows:
        m = labels == i
        rgb = [int(round(float(v))) for v in a[m][:, :3].mean(axis=0)] if a.ndim == 3 else None
        out.append({"centre_px": [round(cx, 1), round(cy, 1)], "radius_px": round(0.5 * max(w, h), 1),
                    "area_px": area, "rgb": rgb, "luma": round(float(gray[m].mean()), 1)})
    return out


def can_see(img: np.ndarray, cam=None) -> str:
    """'dark' | 'dim' | 'ok': dark under dark_level(cam), ok at dim_level(cam) or over, and between them ok only
    when lit surface (lit_fraction) covers LIT_FRAC of the frame, else dim. cam: the camera's own noise facts."""
    b = brightness(img)
    if b < dark_level(cam):
        return "dark"
    if b >= dim_level(cam):
        return "ok"
    return "ok" if lit_fraction(img) >= LIT_FRAC else "dim"


def light(img: np.ndarray, cam=None) -> dict:
    """What a picture says of its light: brightness, can_see and, when it is not ok, bright_spots."""
    b = brightness(img)
    cs = can_see(img, cam=cam)
    out = {"brightness": round(b, 1), "can_see": cs}
    if cs != "ok":
        out["bright_spots"] = bright_spots(img)
    return out


# ---------------------------------------------------------------------------------------------
# ANNOTATED FRAMES
# ---------------------------------------------------------------------------------------------
SCALE_BAR_MM = 20.0
SCALE_BAR_FALLBACK_MM = (10.0, 5.0, 2.0, 1.0)  # used only when a 20 mm bar would not fit
SCALE_BAR_MAX_FRAC = 0.8  # a bar longer than this share of the width does not fit
COLOR_TIP = (255, 0, 255)  # magenta
COLOR_BAR = (255, 255, 0)  # yellow
COLOR_CENTRE = (0, 255, 255)  # cyan
COLOR_TEXT = (255, 255, 255)
COLOR_SHADOW = (0, 0, 0)


def _text(img, s, org, scale=0.45, color=COLOR_TEXT):
    x, y = int(org[0]), int(org[1])
    cv2.putText(img, s, (x + 1, y + 1), cv2.FONT_HERSHEY_SIMPLEX, scale, COLOR_SHADOW, 2, cv2.LINE_AA)
    cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def scale_bar(mm_per_px, width_px: int):
    """(bar_mm, bar_px) for a frame of this width, or None when the scale is unknown."""
    if mm_per_px is None:
        return None
    try:
        m = float(mm_per_px)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(m) or m <= 0.0:
        return None
    limit = SCALE_BAR_MAX_FRAC * width_px
    for mm in (SCALE_BAR_MM,) + SCALE_BAR_FALLBACK_MM:
        px = mm / m
        if px <= limit:
            return mm, px
    return SCALE_BAR_FALLBACK_MM[-1], SCALE_BAR_FALLBACK_MM[-1] / m


def annotate(img: np.ndarray, tip_px, mm_per_px, centre=None) -> np.ndarray:
    """Copy of img with the tool tip marker, a scale bar at the tip's depth and the centre cross.

    tip_px: (u, v) of the tool tip or None when it does not project in front of the camera.
    mm_per_px: millimetres per pixel at the tip's depth (None when unknown).
    centre: (u, v) of the image centre ray; defaults to the geometric centre.
    The input array is never modified.
    """
    a = np.asarray(img)
    a = np.stack([a] * 3, axis=2) if a.ndim == 2 else a[:, :, :3]
    out = np.ascontiguousarray(a).astype(np.uint8).copy()
    h, w = out.shape[:2]
    cu, cv = (w / 2.0, h / 2.0) if centre is None else (float(centre[0]), float(centre[1]))

    # centre ray cross
    c = (int(round(cu)), int(round(cv)))
    arm = max(6, min(w, h) // 30)
    cv2.line(out, (c[0] - arm, c[1]), (c[0] + arm, c[1]), COLOR_CENTRE, 1, cv2.LINE_8)
    cv2.line(out, (c[0], c[1] - arm), (c[0], c[1] + arm), COLOR_CENTRE, 1, cv2.LINE_8)

    tip_in = False
    if tip_px is not None:
        tu, tv = float(tip_px[0]), float(tip_px[1])
        if math.isfinite(tu) and math.isfinite(tv):
            tip_in = 0 <= tu < w and 0 <= tv < h
            t = (int(round(tu)), int(round(tv)))
            if tip_in:
                r = max(5, min(w, h) // 40)
                cv2.circle(out, t, r, COLOR_TIP, 2, cv2.LINE_8)
                cv2.line(out, (t[0] - 2 * r, t[1]), (t[0] - r, t[1]), COLOR_TIP, 2, cv2.LINE_8)
                cv2.line(out, (t[0] + r, t[1]), (t[0] + 2 * r, t[1]), COLOR_TIP, 2, cv2.LINE_8)
                _text(out, "tip", (t[0] + r + 4, t[1] - r - 2), color=COLOR_TIP)
            else:
                # arrow from the centre toward the off-frame tip, clipped at the border
                d = np.array([tu - cu, tv - cv])
                n = np.linalg.norm(d)
                if n > 0:
                    d = d / n
                    reach = min((w / 2.0 - 12) / max(abs(d[0]), 1e-9), (h / 2.0 - 12) / max(abs(d[1]), 1e-9))
                    end = (int(cu + d[0] * reach), int(cv + d[1] * reach))
                    start = (int(cu + d[0] * reach * 0.7), int(cv + d[1] * reach * 0.7))
                    cv2.arrowedLine(out, start, end, COLOR_TIP, 2, cv2.LINE_8, tipLength=0.3)
                _text(out, "tip off frame", (8, h - 28), color=COLOR_TIP)

    bar = scale_bar(mm_per_px, w)
    if bar is None:
        _text(out, "scale unknown", (8, h - 10))
    else:
        mm, px = bar
        n_px = max(1, int(round(px)))
        if tip_in:
            x0 = int(round(float(tip_px[0]) - px / 2.0))
            y0 = int(round(float(tip_px[1]))) + max(14, min(w, h) // 20)
        else:
            x0, y0 = 10, h - 44
        x0 = min(max(x0, 2), max(2, w - n_px - 2))
        y0 = min(max(y0, 8), h - 18)
        out[y0 - 1 : y0 + 2, x0 : x0 + n_px] = COLOR_BAR
        out[y0 - 5 : y0 + 6, x0 : x0 + 1] = COLOR_BAR
        out[y0 - 5 : y0 + 6, x0 + n_px - 1 : x0 + n_px] = COLOR_BAR
        _text(out, f"{mm:g} mm at tip", (x0, y0 + 16), color=COLOR_BAR)
    return out


def mm_per_px_at_depth(depth_m: float, fovy_deg: float, height_px: int):
    """Pinhole scale: millimetres covered by one pixel at depth_m for a vertical field of view."""
    if depth_m is None or not (depth_m > 0) or not (0 < fovy_deg < 180):
        return None
    f_px = (height_px / 2.0) / math.tan(math.radians(fovy_deg) / 2.0)
    return 1000.0 * depth_m / f_px


def annotate_model(img: np.ndarray, model, q, probe_mm: float = 10.0):
    """annotate() with tip_px and mm_per_px taken from a body model (SPEC: annotate(img, model, q)).

    Uses only model.fk(q) and model.project(q, p): the tip is projected, and the scale at its depth
    is measured by projecting a second point probe_mm away, perpendicular to the camera ray.
    Returns (image, tip_px or None, mm_per_px or None).
    """
    fk = model.fk(q)
    p_tip = np.asarray(fk["tip"][0], dtype=float)
    p_cam = np.asarray(fk["camera"][0], dtype=float)
    u, v, depth = model.project(q, p_tip)
    tip_px = None
    mmpp = None
    if depth is not None and depth > 0 and u is not None and v is not None:
        tip_px = (float(u), float(v))
        ray = p_tip - p_cam
        ray /= max(np.linalg.norm(ray), 1e-12)
        perp = np.cross(ray, [0.0, 0.0, 1.0])
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(ray, [1.0, 0.0, 0.0])
        perp /= np.linalg.norm(perp)
        u2, v2, d2 = model.project(q, p_tip + perp * probe_mm / 1000.0)
        if d2 is not None and d2 > 0:
            dpx = math.hypot(float(u2) - tip_px[0], float(v2) - tip_px[1])
            if dpx > 1e-6:
                mmpp = probe_mm / dpx
    return annotate(img, tip_px, mmpp), tip_px, mmpp


# ---------------------------------------------------------------------------------------------
# RELOCALIZE
#
# Gray, percentile stretch (so a dim frame uses the full code range), CLAHE, ORB features, a
# brute-force Hamming match with Lowe's ratio test, then a RANSAC partial affine fit (rotation,
# uniform scale, shift). yaw_deg is the rotation of the image content, counterclockwise on screen
# positive. shift_px is where the reference image centre lands in the new frame, minus the centre.
# ---------------------------------------------------------------------------------------------
ORB_FEATURES = 3000
RATIO = 0.75  # Lowe's ratio test
RANSAC_PX = 3.0  # inlier reprojection threshold
MIN_MATCHES = 12  # fewer good matches than this: no estimate, confidence 0
MIN_INLIERS = 10  # fewer RANSAC inliers than this: no estimate, confidence 0
FULL_CONF_INLIERS = 80  # inlier count at which the count term of the confidence saturates
CLAHE_CLIP = 3.0
CLAHE_TILE = 8


def _prep(img: np.ndarray) -> np.ndarray:
    g = _to_gray(img)
    lo, hi = np.percentile(g, (1.0, 99.0))
    if hi - lo >= 2.0:
        g = np.clip((g.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=(CLAHE_TILE, CLAHE_TILE))
    return clahe.apply(g)


def _no_estimate(why: str, matches: int = 0, inliers: int = 0) -> dict:
    return {
        "yaw_deg": 0.0,
        "shift_px": [0.0, 0.0],
        "scale": 1.0,
        "inliers": int(inliers),
        "matches": int(matches),
        "confidence": 0.0,
        "moved": None,
        "why": why,
    }


def relocalize(ref_img: np.ndarray, img: np.ndarray, cam=None, scatter=None) -> dict:
    """Compare a frame with its reference: {yaw_deg, shift_px, scale, inliers, confidence, moved}.

    Movement exceeds TWO_READ times the inlier fit's own error (at least a pixel), or the measured scatter of
    repeated returns when larger. moved_over records the yaw, shift and scale thresholds.
    moved is None (unknown) whenever confidence is 0: a frame that cannot be matched never says
    "nothing moved".
    """
    for name, a in (("reference", ref_img), ("current", img)):
        if can_see(a, cam=cam) == "dark":
            return _no_estimate(f"the {name} frame is dark")
    g0, g1 = _prep(ref_img), _prep(img)
    orb = cv2.ORB_create(nfeatures=ORB_FEATURES, fastThreshold=10)
    k0, d0 = orb.detectAndCompute(g0, None)
    k1, d1 = orb.detectAndCompute(g1, None)
    if d0 is None or d1 is None or len(k0) < MIN_MATCHES or len(k1) < MIN_MATCHES:
        return _no_estimate("too few features")
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d0, d1, k=2)
    good = [p[0] for p in pairs if len(p) == 2 and p[0].distance < RATIO * p[1].distance]
    if len(good) < MIN_MATCHES:
        return _no_estimate("too few matches", matches=len(good))
    src = np.float32([k0[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([k1[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    M, mask = cv2.estimateAffinePartial2D(
        src, dst, method=cv2.RANSAC, ransacReprojThreshold=RANSAC_PX, maxIters=4000, confidence=0.999,
        refineIters=20,
    )
    if M is None or mask is None:
        return _no_estimate("no consistent motion", matches=len(good))
    n_in = int(mask.sum())
    if n_in < MIN_INLIERS:
        return _no_estimate("too few inliers", matches=len(good), inliers=n_in)
    a, b = float(M[0, 0]), float(M[1, 0])
    scale = math.hypot(a, b)
    yaw = math.degrees(math.atan2(-b, a))
    h, w = g0.shape[:2]
    c = np.array([w / 2.0, h / 2.0, 1.0])
    shift = (M @ c) - c[:2]
    conf = min(1.0, n_in / FULL_CONF_INLIERS) * (n_in / len(good))
    inside = mask.ravel().astype(bool)
    source, matched = src.reshape(-1, 2)[inside], dst.reshape(-1, 2)[inside]
    fitted = source @ M[:, :2].T + M[:, 2]
    rms = float(np.sqrt(np.mean(np.sum((fitted - matched) ** 2, axis=1))))
    spread = float(np.sqrt(np.mean(np.sum((source - source.mean(axis=0)) ** 2, axis=1))))
    if spread <= 0:
        return _no_estimate("inliers have no spatial spread", matches=len(good), inliers=n_in)
    error = max(rms, PIXEL)
    fit_error = {"yaw_deg": math.degrees(error / spread), "shift_px": error, "scale": error / spread}
    over = {key: R.TWO_READ * max(value, float((scatter or {}).get(key) or 0.0)) for key, value in fit_error.items()}
    moved = (abs(yaw) > over["yaw_deg"] or float(np.linalg.norm(shift)) > over["shift_px"]
             or abs(scale - 1) > over["scale"])
    return {
        "yaw_deg": round(yaw, 3) + 0.0,  # + 0.0 turns -0.0 into 0.0
        "shift_px": [round(float(shift[0]), 2), round(float(shift[1]), 2)],
        "scale": round(scale, 4),
        "inliers": n_in,
        "matches": len(good),
        "confidence": round(float(conf), 3),
        "moved": bool(moved),
        "moved_over": {key: round(value, 3) for key, value in over.items()},
        "why": "",
    }


# ---------------------------------------------------------------------------------------------
# TWO-VIEW DEPTH: how far to trust a triangulated point
#
# The distance by which the rays miss each other says the marks are consistent, not that the depth is right: two
# rays that are nearly parallel meet at a shallow angle, and a mark one pixel off slides the meeting point a long
# way along the view. A mark px_err pixels off moves its ray sideways by e = px_err * range / f_px at the point;
# that moves the point where it meets the other ray by e / sin(angle) along the view. With both marks off the two
# shifts add, so the bound is 2 e / sin(angle).
# ---------------------------------------------------------------------------------------------
NARROW_RAY_DEG = 15.0  # under this angle between the rays, depth along the view is weak: warn


def triangulation_quality(rays, point, f_px, px_err=1.0, floor_z_m=None) -> dict:
    """What a triangulated point is worth. rays: [(origin, direction), ...] in metres, frame B, as marked;
    point: the triangulated point in metres; f_px: the camera's focal length in pixels; px_err: how far off a
    mark may be, in pixels; floor_z_m: height of the surface the robot stands on (None: not checked).

    Returns {"ray_angle_deg" (the widest angle between two rays), "range_mm" (mean distance from the cameras),
    "depth_uncertainty_mm" (how far along the view the point may be off when each mark is px_err off; None when
    the rays are parallel), "warnings": [text, ...]}.
    """
    p = np.asarray(point, dtype=float)
    dirs = [np.asarray(d, dtype=float) / max(float(np.linalg.norm(d)), 1e-12) for _, d in rays]
    origins = [np.asarray(o, dtype=float) for o, _ in rays]
    widest = 0.0
    for i in range(len(dirs)):
        for j in range(i + 1, len(dirs)):
            c = max(-1.0, min(1.0, abs(float(dirs[i] @ dirs[j]))))
            widest = max(widest, math.degrees(math.acos(c)))
    rng = float(np.mean([np.linalg.norm(p - o) for o in origins])) if origins else 0.0
    sin_a = math.sin(math.radians(widest))
    depth = None if sin_a < 1e-6 or not f_px else 2.0 * float(px_err) * (rng / float(f_px)) / sin_a * 1000.0
    warnings = []
    if depth is None:
        warnings.append("the rays are parallel: they fix no depth")
    elif widest < NARROW_RAY_DEG:
        warnings.append(f"the rays are only {widest:.1f} deg apart: the point may be off by about {depth:.0f} mm "
                        "along the view; move the camera further sideways between the looks, or touch")
    if floor_z_m is not None and float(p[2]) < float(floor_z_m) - 1e-4:
        warnings.append(f"the point lies {(float(floor_z_m) - float(p[2])) * 1000.0:.0f} mm below the surface the "
                        "robot stands on: a mark is off, or the rays are too close to fix depth")
    return {"ray_angle_deg": round(widest, 2), "range_mm": round(rng * 1000.0, 1),
            "depth_uncertainty_mm": None if depth is None else round(depth, 1), "warnings": warnings}


# ---------------------------------------------------------------------------------------------
# THIRD-PERSON PICTURE
# ---------------------------------------------------------------------------------------------
VIEWS = {"side": (0, 2, "x forward", "z up"), "top": (0, 1, "x forward", "y left")}
GRID_STEPS_MM = (10, 20, 50, 100, 200, 500)
MARGIN_MM = 60.0
COLOR_GRID = (225, 225, 225)
COLOR_AXIS = (150, 150, 150)
COLOR_LINK = (40, 70, 140)
COLOR_JOINT = (20, 20, 20)
COLOR_TIP_S = (220, 0, 0)
COLOR_CONTACT = (230, 120, 0)
COLOR_OBJECT = (0, 140, 60)
COLOR_STALE = (160, 160, 160)
COLOR_FOOT = (120, 60, 150)


def _links_list(links):
    if links is None:
        return []
    items = links.items() if isinstance(links, dict) else ((x[0], x[1:]) for x in links)
    out = []
    for name, rest in items:
        p0, p1, r = rest[0], rest[1], rest[2] if len(rest) > 2 else 0.02
        out.append((str(name), np.asarray(p0, float) * 1000.0, np.asarray(p1, float) * 1000.0, float(r) * 1000.0))
    return out


def _entry_point(e):
    p = e.get("point_mm") if isinstance(e, dict) else None
    if p is None or len(p) < 3:
        return None
    return np.asarray(p, float)


def _entry_label(e):
    s = str(e.get("label") or e.get("id") or "?")
    return s + (" (stale)" if e.get("stale") else "")


def _unconfirmed(e) -> bool:
    return e.get("confirmed") is False


def draw_schematic(view, links, contacts, objects, out_path, tip_mm=None, size=(640, 480), title=None, chain=None,
                   azimuth_deg=None, footprint_mm=None, com_mm=None):
    """Draw the arm from outside (side: x forward, z up; top: x forward, y left) to a PNG.

    links: {name: (p0, p1, radius)} or [(name, p0, p1, radius)] in metres, frame B (model.fk), in chain order.
      Each is drawn to scale, and the end of one is joined to the start of the next so the arm reads as one chain.
    contacts, objects: world entries (dicts with point_mm, label or id, optional stale, confirmed). A contact
      with confirmed false (a guard stop no touch has confirmed) is drawn hollow.
    tip_mm: tool tip in mm, frame B; defaults to the far end of the last link.
    chain: the joint anchors in order, in metres (base first, tip last): drawn as one line under the links.
    azimuth_deg: side view only: look along the horizontal direction at right angles to this bearing, so the
      horizontal axis is the distance along it (0: along x, the default; 90: along y).
    footprint_mm: the base outline as [[x, y], ...] in mm; com_mm: the arm's centre of mass [x, y, z] in mm.
    Returns {"image", "view", "grid_mm", "px_per_mm", "origin_px", "chain_px", "drawn"} or a refusal dict for an
    unknown view.
    """
    if view not in VIEWS:
        return {"refused": {"check": "view", "why": f"unknown view {view!r}", "hint": "use side or top"}}
    ia, ib, la, lb = VIEWS[view]
    w, h = int(size[0]), int(size[1])
    az = None if azimuth_deg is None or view != "side" else float(azimuth_deg)

    def flat(p):
        """A point in mm, frame B, as (horizontal, vertical) of this view."""
        p = np.asarray(p, float)
        if az is not None:
            return np.array([p[0] * math.cos(math.radians(az)) + p[1] * math.sin(math.radians(az)), p[2]])
        return np.array([p[ia], p[ib]])

    if az is not None:
        la = f"along {az:g} deg"
    L = _links_list(links)
    C = [(e, _entry_point(e)) for e in (contacts or [])]
    O = [(e, _entry_point(e)) for e in (objects or [])]
    C = [(e, p) for e, p in C if p is not None]
    O = [(e, p) for e, p in O if p is not None]
    tip = np.asarray(tip_mm, float) if tip_mm is not None else (L[-1][2] if L else None)
    anchors = [np.asarray(p, float) * 1000.0 for p in (chain or [])]
    foot = [np.array([float(q[0]), float(q[1]), 0.0]) for q in (footprint_mm or [])]
    com = None if com_mm is None else np.asarray(com_mm, float)

    pts = [np.zeros(3)]
    for _, p0, p1, _r in L:
        pts += [p0, p1]
    pts += [p for _, p in C] + [p for _, p in O] + anchors + foot
    if tip is not None:
        pts.append(tip)
    if com is not None:
        pts.append(com)
    P = np.array([flat(p) for p in pts])
    lo, hi = P.min(axis=0) - MARGIN_MM, P.max(axis=0) + MARGIN_MM
    if view == "side":
        lo[1] = min(lo[1], -MARGIN_MM / 2)
    span = np.maximum(hi - lo, 1.0)
    top_pad, bot_pad = 26, 22
    ppm = min((w - 20) / span[0], (h - top_pad - bot_pad) / span[1])
    mid = (lo + hi) / 2.0

    def px(a, b):
        u = w / 2.0 + (a - mid[0]) * ppm
        v = top_pad + (h - top_pad - bot_pad) / 2.0 - (b - mid[1]) * ppm
        return int(round(u)), int(round(v))

    def at_px(p):
        return px(*flat(p))

    img = np.full((h, w, 3), 255, np.uint8)
    step = next((s for s in GRID_STEPS_MM if s * ppm >= 40), GRID_STEPS_MM[-1])
    for k in range(int(math.floor(lo[0] / step)), int(math.ceil(hi[0] / step)) + 1):
        x = px(k * step, 0)[0]
        cv2.line(img, (x, top_pad), (x, h - bot_pad), COLOR_AXIS if k == 0 else COLOR_GRID, 1)
    for k in range(int(math.floor(lo[1] / step)), int(math.ceil(hi[1] / step)) + 1):
        y = px(0, k * step)[1]
        cv2.line(img, (0, y), (w, y), COLOR_AXIS if k == 0 else COLOR_GRID, 1)
    if view == "side":
        y0 = px(0, 0)[1]
        cv2.line(img, (0, y0), (w, y0), (90, 90, 90), 2)
        cv2.putText(img, "mounting plane", (6, y0 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (90, 90, 90), 1, cv2.LINE_AA)
    if len(foot) >= 2:  # the base outline: a closed line from above, its extent on the mounting plane from the side
        ring = [at_px(q) for q in foot]
        cv2.polylines(img, [np.array(ring, np.int32)], view == "top", COLOR_FOOT, 2, cv2.LINE_AA)

    for i in range(len(L) - 1):  # join each link's end to the next link's start: one connected chain
        cv2.line(img, at_px(L[i][2]), at_px(L[i + 1][1]), COLOR_JOINT, 2, cv2.LINE_AA)
    chain_px = [at_px(p) for p in anchors]
    for a, b in zip(chain_px, chain_px[1:]):
        cv2.line(img, a, b, COLOR_JOINT, 2, cv2.LINE_AA)
    for _name, p0, p1, r in L:
        cv2.line(img, at_px(p0), at_px(p1), COLOR_LINK, int(np.clip(2 * r * ppm, 3, 24)), cv2.LINE_AA)
    for _name, p0, p1, _r in L:
        for p in (p0, p1):
            cv2.circle(img, at_px(p), 3, COLOR_JOINT, -1, cv2.LINE_AA)
    for a in chain_px:
        cv2.circle(img, a, 4, COLOR_JOINT, 1, cv2.LINE_AA)
    cv2.drawMarker(img, px(0, 0), (90, 90, 90), cv2.MARKER_TRIANGLE_UP, 14, 2)

    def label(s, at, color):
        cv2.putText(img, s, (at[0] + 7, at[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

    for e, p in C:
        at = at_px(p)
        col = COLOR_STALE if e.get("stale") else COLOR_CONTACT
        cv2.circle(img, at, 5, col, 2 if _unconfirmed(e) else -1, cv2.LINE_AA)
        label(_entry_label(e) + (" (stop)" if _unconfirmed(e) else ""), at, col)
    for e, p in O:
        at = at_px(p)
        col = COLOR_STALE if e.get("stale") else COLOR_OBJECT
        cv2.rectangle(img, (at[0] - 5, at[1] - 5), (at[0] + 5, at[1] + 5), col, 2)
        label(_entry_label(e), at, col)
    if com is not None:
        at = at_px(com)
        cv2.drawMarker(img, at, COLOR_FOOT, cv2.MARKER_DIAMOND, 12, 2)
        label("centre of mass", at, COLOR_FOOT)
    if tip is not None:
        at = at_px(tip)
        cv2.drawMarker(img, at, COLOR_TIP_S, cv2.MARKER_TILTED_CROSS, 14, 2)
        label("tip", at, COLOR_TIP_S)

    head = title or f"{view} view ({la}, {lb}), grid {step} mm"
    cv2.putText(img, head, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(img, f"+{la.split()[0]} ->", (w - 70, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
    Image.fromarray(img).save(str(out_path))
    return {
        "image": str(out_path),
        "view": view,
        "grid_mm": step,
        "px_per_mm": round(float(ppm), 4),
        "origin_px": list(px(0, 0)),
        "chain_px": [list(a) for a in chain_px],
        "drawn": {"links": len(L), "contacts": len(C), "objects": len(O), "tip": tip is not None,
                  "stops": sum(1 for e, _ in C if _unconfirmed(e)), "chain": len(anchors),
                  "footprint": len(foot) >= 2, "com": com is not None},
    }
