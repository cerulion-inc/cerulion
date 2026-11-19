# SPDX-License-Identifier: AGPL-3.0-only
"""Cue reading and view geometry for the device package. Pure functions on frames and the body model.

A cue is a small patch of the wrist image at a located point (or a pixel the operator names). It reads LIT when the
brightest tenth of the patch stands clearly above the median of a ring around it, and OFF otherwise. A series of
readings over time is OFF, STEADY or BLINKING. A reading is never taken from a point that does not project into the
frame, that the robot itself covers (a ray cast on the body model's shown geometry, plus the collision capsules of
any body with no shown geometry), or whose neighbourhood is too dark to tell OFF from unlit: those are UNOBSERVABLE
with a reason. The robot's own body in the patch or its ring (found by rays cast on the body model through
SECTORS directions around the cue) is left out of both, so a bright hand beside a lit cue does not make it read
OFF; a patch whose ring is mostly the robot is UNOBSERVABLE (occluded_by_robot). Nothing here reads simulator ground
truth or names a device kind.
"""

import math
from rcpx.r2 import device_bounds as derived
import weakref

import mujoco
import numpy as np

MM = 1000.0
LIT_RATIO = 1.6          # a lit cue's bright tenth is at least this many times the ring around it
LIT_MIN_DIFF = 6.0       # and at least this many luma counts above it (0 to 255)
DARK_LOCAL = 8.0         # cue and ring both under this luma: OFF cannot be told from unlit (too dark)
RING_IN, RING_OUT = 1.4, 2.2   # the ring around the cue, in cue radii
NEAR_SKIP_M = 0.01       # a sight line starts this far out from the camera
END_SKIP_M = 0.005       # and stops this short of the point (the point's own surface)
EDGE_PX = 2.0            # a point closer than this to the frame edge is out of view
SECTORS = 24             # directions around a cue in which the robot's own body is looked for (rays)
DISC_PROBE, RING_PROBE = 0.7, 1.8    # where in each direction, in cue radii
MIN_RING_CLEAR = 1.0 / 3.0           # at least this share of the ring's directions must be clear of the robot
MIN_DISC_CLEAR = 0.5                 # and this share of the patch's


def luma(img):
    f = np.asarray(img)[:, :, :3].astype(np.float64)
    return 0.299 * f[:, :, 0] + 0.587 * f[:, :, 1] + 0.114 * f[:, :, 2]


def cue_level(img, u, v, r, cover=None):
    """{peak, ring, lit, dark} of the patch of radius r px at (u, v), or None when the patch is off the frame.
    cover (patch_cover): the directions around the cue where the robot's own body is; its pixels are left out."""
    L = luma(img)
    h, w = L.shape
    r = max(1.5, float(r))
    lo_u, hi_u = int(max(0, math.floor(u - RING_OUT * r))), int(min(w, math.ceil(u + RING_OUT * r) + 1))
    lo_v, hi_v = int(max(0, math.floor(v - RING_OUT * r))), int(min(h, math.ceil(v + RING_OUT * r) + 1))
    if hi_u <= lo_u or hi_v <= lo_v:
        return None
    yy, xx = np.mgrid[lo_v:hi_v, lo_u:hi_u]
    d = np.hypot(xx + 0.5 - u, yy + 0.5 - v)
    patch = L[lo_v:hi_v, lo_u:hi_u]
    keep_disc = keep_ring = np.ones(d.shape, bool)
    if cover is not None:
        sector = (np.floor(np.arctan2(yy + 0.5 - v, xx + 0.5 - u) % (2 * math.pi) / (2 * math.pi) * SECTORS)
                  .astype(int) % SECTORS)
        keep_disc = ~np.asarray(cover["disc"], bool)[sector]
        keep_ring = ~np.asarray(cover["ring"], bool)[sector]
    disc = patch[(d <= r) & keep_disc]
    ring = patch[(d >= RING_IN * r) & (d <= RING_OUT * r) & keep_ring]
    if disc.size < 3 or ring.size < 5:
        return None
    peak, base = float(np.percentile(disc, 90)), float(np.median(ring))
    lit = peak >= LIT_RATIO * base and peak - base >= LIT_MIN_DIFF
    dark = (not lit) and peak < DARK_LOCAL and base < DARK_LOCAL
    return {"peak": round(peak, 1), "ring": round(base, 1), "lit": bool(lit), "dark": bool(dark)}


def classify(samples, rate_hz):
    """Verdict of a series [(t_s, level dict or None, frame can_see)]: off | steady | blinking | unobservable.
    Frames too dark to tell count only when some frame of the series shows the cue lit (a lit cue shows itself)."""
    usable = [(t, lv, cs) for t, lv, cs in samples if lv is not None]
    if not usable:
        return {"verdict": "unobservable", "reason": "out_of_view"}
    any_lit = any(lv["lit"] for _, lv, _ in usable)
    seq = [(t, lv["lit"]) for t, lv, cs in usable if any_lit or not (lv["dark"] or cs == "dark")]
    if not seq:
        return {"verdict": "unobservable", "reason": "dark"}
    flips = [(seq[i][0], seq[i][1]) for i in range(1, len(seq)) if seq[i][1] != seq[i - 1][1]]
    out = {"verdict": None, "period_s": None, "min_period_s": round(2.0 / float(rate_hz), 3)}
    if not flips:
        out["verdict"] = "steady" if seq[0][1] else "off"
    elif len(flips) == 1:
        out["verdict"] = "steady" if seq[-1][1] else "off"
        out["changed_at_s"] = round(flips[0][0], 3)
    else:
        out["verdict"] = "blinking"
        rises = [t for t, lit in flips if lit]
        falls = [t for t, lit in flips if not lit]
        edges = rises if len(rises) >= 2 else falls
        if len(edges) >= 2:
            out["period_s"] = round((edges[-1] - edges[0]) / (len(edges) - 1), 3)
        else:
            out["period_s"] = round(2.0 * (flips[-1][0] - flips[0][0]) / (len(flips) - 1), 3)
    return out


def cue_class(verdict):
    """lit | dark | None for a verdict or a single reading."""
    if verdict in ("steady", "blinking", "lit"):
        return "lit"
    if verdict == "off":
        return "dark"
    return None


# ---------------------------------------------------------------- geometry
def _seg_dist(a0, a1, b0, b1):
    d1, d2, r = a1 - a0, b1 - b0, a0 - b0
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    if a < 1e-12 and e < 1e-12:
        return float(np.linalg.norm(r))
    if a < 1e-12:
        s, t = 0.0, min(1.0, max(0.0, f / e))
    else:
        c = float(d1 @ r)
        if e < 1e-12:
            s, t = min(1.0, max(0.0, -c / a)), 0.0
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s = min(1.0, max(0.0, (b * f - c * e) / den)) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                s, t = min(1.0, max(0.0, -c / a)), 0.0
            elif t > 1.0:
                s, t = min(1.0, max(0.0, (b - c) / a)), 1.0
    return float(np.linalg.norm((a0 + d1 * s) - (b0 + d2 * t)))


def sight_blocked(capsules, origin, point):
    """Name of the first robot capsule that covers the sight line origin -> point, or None. A capsule the camera
    itself sits in is skipped: the camera's own mount cannot be told from an occluder by capsules."""
    o, p = np.asarray(origin, float), np.asarray(point, float)
    L = float(np.linalg.norm(p - o))
    if L < NEAR_SKIP_M + END_SKIP_M:
        return None
    v = (p - o) / L
    a0, a1 = o + v * NEAR_SKIP_M, p - v * END_SKIP_M
    for name, c0, c1, r in capsules:
        c0, c1 = np.asarray(c0, float), np.asarray(c1, float)
        if _seg_dist(o, o, c0, c1) <= r + 0.001:
            continue
        if _seg_dist(a0, a1, c0, c1) < r:
            return name
    return None


VISIBLE_GROUPS = np.array([1, 1, 1, 0, 0, 0], np.uint8)   # the geom groups a camera render shows
NEAR_OTHER_M = 0.006     # another body's geometry from this far in front of the lens covers the view (a frame clips
                         # only what is nearer than its near plane, about 5 mm); the camera's own body from NEAR_SKIP_M


def _ray_body(model, o, v, start, length, exclude):
    gid = np.zeros(1, np.int32)
    x = mujoco.mj_ray(model.mj, model.d, o + v * start, v, VISIBLE_GROUPS, 1, exclude, gid)
    if 0.0 <= x < length - start and gid[0] >= 0:
        return model.mj.body(int(model.mj.geom_bodyid[gid[0]])).name
    return None


def _model_ray(model, origin, point):
    """Name of the robot body whose own model geometry (the robot alone, at the pose last set on the model) the
    sight line origin -> point meets, or None. Two casts: every body but the camera's own from NEAR_OTHER_M (a link
    folded in front of the lens hides the view), and every body from NEAR_SKIP_M (the camera's own body,
    which the lens sits in, counts only beyond its mount: a finger in view)."""
    o, p = np.asarray(origin, float), np.asarray(point, float)
    L = float(np.linalg.norm(p - o))
    if L < NEAR_SKIP_M + END_SKIP_M:
        return None
    v = (p - o) / L
    cam_body = int(model.mj.cam_bodyid[model.mj.camera("wrist").id])
    return (_ray_body(model, o, v, NEAR_OTHER_M, L - END_SKIP_M, cam_body)
            or _ray_body(model, o, v, NEAR_SKIP_M, L - END_SKIP_M, -1))


_SHOWN = weakref.WeakKeyDictionary()


def _shown_bodies(model):
    """Names of the robot bodies that have geometry a camera shows (cached per loaded model, dropped with it)."""
    m = model.mj
    if m not in _SHOWN:
        _SHOWN[m] = {m.body(int(m.geom_bodyid[g])).name for g in range(m.ngeom)
                     if 0 <= int(m.geom_group[g]) < len(VISIBLE_GROUPS) and VISIBLE_GROUPS[int(m.geom_group[g])]}
    return _SHOWN[m]


def robot_cover(model, q, origin, point):
    """The robot part covering the sight line at pose q, from the body model (never the scene): the robot's own
    shown geometry decides; a collision capsule counts only for a body with no shown geometry. (Capsules alone
    are fat: on a small arm they cover a third of the free image.)"""
    caps = model.collision_capsules(q)                    # also sets the model to q for the ray below
    name = _model_ray(model, origin, point)
    if name is not None:
        return name
    shown = _shown_bodies(model)
    return sight_blocked([c for c in caps if c[0] not in shown], origin, point)


def point_view(model, q, p_m, radius_m=None):
    """Where a frame-B point (m) falls in the wrist image at q: {u, v, depth_m, radius_px, in_frame, blocked_by}."""
    u, v, depth = model.project(q, p_m)
    out = {"u": None if not math.isfinite(u) else round(u, 1), "v": None if not math.isfinite(v) else round(v, 1),
           "depth_mm": round(depth * MM, 1), "in_frame": False, "blocked_by": None, "radius_px": None}
    if depth <= 1e-6 or not math.isfinite(u):
        out["why"] = "the point is behind the camera"
        return out
    if not (EDGE_PX <= u <= model.cam_w - EDGE_PX and EDGE_PX <= v <= model.cam_h - EDGE_PX):
        out["why"] = f"the point projects at ({u:.0f}, {v:.0f}), outside the {model.cam_w} x {model.cam_h} frame"
        return out
    out["in_frame"] = True
    if radius_m is not None:
        out["radius_px"] = round(max(2.0, model.f_px * radius_m / depth), 1)
    pc = model.fk(q)["camera"][0]
    out["blocked_by"] = robot_cover(model, q, pc, p_m)
    return out


def pixel_blocked(model, q, u, v, reach_m):
    """Name of the robot part that covers pixel (u, v) along its ray out to reach_m, or None."""
    o, d = model.pixel_ray(q, u, v)
    return robot_cover(model, q, o, o + d * reach_m)


def patch_cover(model, q, u, v, r_px, reach_m):
    """Where the robot's own body lies around the cue at pixel (u, v), radius r_px, at pose q: for each of SECTORS
    directions, whether a ray cast on the body model through the patch (DISC_PROBE radii out) and through its ring
    (RING_PROBE radii out) meets the robot within reach_m. {disc, ring: [bool], disc_clear, ring_clear (shares of
    directions clear), by (a robot part met), covered (too little of the patch or ring is clear to read it)}."""
    disc, ring, by = [], [], None
    for k in range(SECTORS):
        th = (k + 0.5) * 2.0 * math.pi / SECTORS
        for rho, out in ((DISC_PROBE, disc), (RING_PROBE, ring)):
            o, d = model.pixel_ray(q, u + rho * r_px * math.cos(th), v + rho * r_px * math.sin(th))
            name = robot_cover(model, q, o, o + d * reach_m)
            out.append(name is not None)
            by = by or name
    dc, rc = 1.0 - sum(disc) / SECTORS, 1.0 - sum(ring) / SECTORS
    return {"disc": disc, "ring": ring, "disc_clear": round(dc, 3), "ring_clear": round(rc, 3), "by": by,
            "covered": dc < MIN_DISC_CLEAR or rc < MIN_RING_CLEAR}


def camera_delta(model, q, p_m, pix, depth_m):
    """move_tip offset in frame camera (mm: image right, image down, along the view) that puts the frame-B point
    p_m at pixel pix and depth depth_m by a pure translation of the hand, and the same offset in frame B (m)."""
    pc, Rc = model.fk(q)["camera"]
    f = model.f_px
    c_star = np.array([(pix[0] - model.cam_w / 2.0) / f * depth_m, -(pix[1] - model.cam_h / 2.0) / f * depth_m,
                       -depth_m])
    delta = (np.asarray(p_m, float) - Rc @ c_star) - pc
    cam = [float(delta @ Rc[:, 0]), -float(delta @ Rc[:, 1]), -float(delta @ Rc[:, 2])]
    return [round(x * MM, 1) for x in cam], delta


def _surround_covered(model, q, pv, p_m):
    if not pv.get("radius_px"):
        return False
    reach = float(np.linalg.norm(np.asarray(p_m, float) - model.fk(q)["camera"][0])) + model.length_scale
    return patch_cover(model, q, pv["u"], pv["v"], pv["radius_px"], reach)["covered"]


def view_hint(model, q, p_m, radius_m=None, valid=None, B=None):
    """A move_tip delta (frame camera) from pose q after which the point is in frame, not covered by the robot and
    its surroundings not mostly the robot, checked on the body model (ik with the hand's orientation held); the
    "checked" field says when no checked pose was found."""
    depth_now = model.project(q, p_m)[2]
    near = derived.near_focus_m(B, model)[0]
    wanted = radius_m * 2 * model.f_px / 24 if radius_m else depth_now
    depth = min(model.length_scale, max(near, wanted))
    tip, R = model.fk_tip(q)
    first = None
    for pix in ((0.5, 0.45), (0.5, 0.3), (0.3, 0.4), (0.7, 0.4)):
        for dep in (depth, (depth + near) / 2, (depth + model.length_scale) / 2):
            px = (pix[0] * model.cam_w, pix[1] * model.cam_h)
            cam_mm, delta = camera_delta(model, q, p_m, px, dep)
            hint = {"tool": "move_tip", "args": {"dx": cam_mm[0], "dy": cam_mm[1], "dz": cam_mm[2], "frame": "camera",
                                                 "keep_orientation": True},
                    "expect_px": [round(px[0], 1), round(px[1], 1)], "expect_depth_mm": round(dep * MM, 1)}
            first = first or hint
            kw = {"R_target": R} if model.n >= 6 else {"keep_axis": ("x", R[:, 0])}
            q2, ok, _ = model.ik(q, tip + delta, restarts=2, **kw)
            if not ok or (valid is not None and not valid(q2)):
                continue
            pv = point_view(model, q2, p_m, radius_m)
            if pv["in_frame"] and pv["blocked_by"] is None and not _surround_covered(model, q2, pv, p_m):
                hint["checked"] = "the body model reaches this pose and sees the point clear of the robot"
                return hint
    first = dict(first or {})
    first["checked"] = ("no pose the body model reaches shows the point clear of the robot: come at it from "
                        "another side, or reach_check the camera position first")
    return first
