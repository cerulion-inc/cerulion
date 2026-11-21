# SPDX-License-Identifier: AGPL-3.0-only
"""Geometry of the frames package: what move_tip arguments, base directions and image directions mean in one
look. Pure functions of RobotModel.fk, RobotModel.project and a World; no drawing, no executor state, no robot named.

Pixels: u right, v down, origin top-left, continuous (pixel i covers [i, i + 1)). Inside SI; results in mm, px, deg.

The camera may ride on the hand (it does on many loaded models): then the tip keeps its pixel in every look, and
an arrow means "where the tip goes relative to what THIS picture shows". That is the projection, through this look's
camera, of the tip's new position, which is what every number here is.
"""

import math

import numpy as np

from rcpx.context import refusal

MM = 1000.0
# (frame, axis index, move_tip argument): the five arrows drawn at the tip
AXES = (("base", 0, "dx"), ("base", 1, "dy"), ("base", 2, "dz"), ("tool", 0, "dx"), ("tool", 2, "dz"))
TICKS_MM = (20.0, 50.0, 100.0)    # ticks on the tool's forward line (tool +x)
ALONG_VIEW_DEG = 15.0             # an axis this close to the line of sight moves the tip mostly in depth
PLANE_MIN_SV_RATIO = 0.15         # base x, y moves reach every image direction only when the 2 x 2 image Jacobian
                                  # has its smaller singular value above this share of the larger one
STEP_MM = 1.0                     # finite step for image rates (a pinhole maps a line to a line: exact direction)
WORDS = ("right", "up-right", "up", "up-left", "left", "down-left", "down", "down-right")
FORMATS = ("px", "norm1000_yx", "norm_xy")


def _r(x, nd=1):
    return round(float(x), nd) + 0.0


def _vec(v, nd=3):
    return [_r(x, nd) for x in v]


def in_frame(u, v, w, h):
    return u is not None and v is not None and 0.0 <= u < w and 0.0 <= v < h


def project(model, q, p):
    """(u, v, depth_m) of a frame-B point through the look's camera, or None when it is behind the camera."""
    u, v, d = model.project(q, np.asarray(p, dtype=float))
    if not (d > 1e-6 and math.isfinite(u) and math.isfinite(v)):
        return None
    return float(u), float(v), float(d)


def image_word(du, dv):
    """One of eight compass words for an image direction (du right, dv down)."""
    ang = math.degrees(math.atan2(-dv, du)) % 360.0
    return WORDS[int(((ang + 22.5) % 360.0) // 45.0)]


def _rate(model, q, p, a, tip):
    """(unit image direction, px per mm) of moving the tip along unit vector a, or (None, None)."""
    s = project(model, q, p + a * (STEP_MM / MM))
    if s is None or tip is None:
        return None, None
    du, dv = s[0] - tip[0], s[1] - tip[1]
    n = math.hypot(du, dv)
    if n < 1e-9:
        return None, 0.0
    return (du / n, dv / n), n / STEP_MM


def axis_arrows(model, q, fk, tip):
    """axes_px: per arrow its move_tip label, base vector, image direction, px per mm and view angle."""
    p, Rt = fk["tip"]
    pc = fk["camera"][0]
    los = p - pc
    los = los / max(float(np.linalg.norm(los)), 1e-12)
    out = []
    for frame, i, arg in AXES:
        a = np.eye(3)[:, i] if frame == "base" else np.asarray(Rt[:, i], dtype=float)
        c = float(a @ los)
        ang = math.degrees(math.acos(min(1.0, abs(c))))
        e = {"label": f"{arg}+ {frame}", "move_tip": {arg: "+", "frame": frame}, "base_vec": _vec(a),
             "view_angle_deg": _r(ang), "along_view": ang < ALONG_VIEW_DEG,
             "depth": "away" if c > 0 else "toward"}
        d, rate = _rate(model, q, p, a, tip)
        if d is None:
            e.update(dir_px=None, image_dir=None, dir_deg=None, px_per_mm=None if rate is None else 0.0)
        else:
            e.update(dir_px=_vec(d, 4), image_dir=image_word(*d),
                     dir_deg=_r(math.degrees(math.atan2(-d[1], d[0])) % 360.0), px_per_mm=_r(rate, 3))
        out.append(e)
    return out


def _nearest(vec):
    i = int(np.argmax(np.abs(vec)))
    return f"{'xyz'[i]}", "+" if vec[i] > 0 else "-", abs(float(vec[i]))


def image_to_base(model, q, fk, tip):
    """The base unit vector that image right, left, up and down mean at the tip's depth (a move parallel to the
    image plane), the mm per px there, the base offset per 10 px, the nearest single move_tip argument, and the
    move on the mounting plane (dz = 0) that goes the same image way."""
    if tip is None:
        return {"available": False, "why": "the tip is behind the camera in this look"}
    p = fk["tip"][0]
    Rc = fk["camera"][1]
    right, up = np.asarray(Rc[:, 0], dtype=float), np.asarray(Rc[:, 1], dtype=float)
    _, rate = _rate(model, q, p, right, tip)
    if not rate:
        return {"available": False, "why": "the image scale at the tip cannot be measured in this look"}
    mmpp = 1.0 / rate
    J2 = np.zeros((2, 2))
    for k in range(2):
        s = project(model, q, p + np.eye(3)[:, k] * (STEP_MM / MM))
        if s is not None:
            J2[:, k] = [(s[0] - tip[0]) / STEP_MM, (s[1] - tip[1]) / STEP_MM]
    sv = np.linalg.svd(J2, compute_uv=False)
    plane_ok = sv[0] > 1e-9 and sv[1] > PLANE_MIN_SV_RATIO * sv[0]
    out = {"available": True, "depth_mm": _r(tip[2] * MM), "mm_per_px": _r(mmpp, 4)}
    per, plane, nearest = {}, {}, {}
    for word, vec, img in (("right", right, (1.0, 0.0)), ("left", -right, (-1.0, 0.0)),
                           ("up", up, (0.0, -1.0)), ("down", -up, (0.0, 1.0))):
        out[word] = _vec(vec)
        per[word] = {k: _r(x * 10.0 * mmpp, 2) for k, x in zip(("dx", "dy", "dz"), vec)}
        ax, sign, cos = _nearest(vec)
        nearest[word] = {"move_tip": f"d{ax}{sign} base", "cos": _r(cos, 3)}
        if plane_ok:
            ab = np.linalg.solve(J2, 10.0 * np.asarray(img))
            plane[word] = {"dx": _r(ab[0], 2), "dy": _r(ab[1], 2), "dz": 0.0}
        else:
            plane[word] = None
    out["per_10px_mm"] = per
    out["nearest"] = nearest
    out["on_plane_per_10px_mm"] = plane
    if not plane_ok:
        out["on_plane_why"] = ("the camera looks along the mounting plane here: some image directions need a "
                               "vertical move")
    return out


def forward_ticks(model, q, fk, w, h):
    """ticks_px: the tool's forward line (tool +x) at 20, 50 and 100 mm ahead of the tip."""
    p, Rt = fk["tip"]
    out = []
    for mm in TICKS_MM:
        s = project(model, q, p + np.asarray(Rt[:, 0], dtype=float) * (mm / MM))
        out.append({"mm": mm, "px": None if s is None else [_r(s[0]), _r(s[1])],
                    "in_view": s is not None and in_frame(s[0], s[1], w, h),
                    "depth_mm": None if s is None else _r(s[2] * MM)})
    return out


def world_points(model, q, world, w, h):
    """world_px: every world entry with a point, re-projected through this look's camera, with its age."""
    now = getattr(world, "now", None) or {"call": 0, "t_s": 0.0}
    out = []
    for e in world.query():
        pt = e.get("point_mm")
        if pt is None:
            continue
        s = project(model, q, np.asarray(pt, dtype=float) / MM)
        obs = e.get("observed_at") or {}
        out.append({"id": e.get("id"), "label": e.get("label"), "kind": e.get("kind"),
                    "px": None if s is None else [_r(s[0]), _r(s[1])],
                    "in_view": s is not None and in_frame(s[0], s[1], w, h), "behind": s is None,
                    "depth_mm": None if s is None else _r(s[2] * MM),
                    "age_calls": int(now.get("call", 0)) - int(obs.get("call", 0)),
                    "age_s": _r(float(now.get("t_s", 0.0)) - float(obs.get("t_s", 0.0)), 2),
                    "stale": bool(e.get("stale")), "confirmed": e.get("confirmed")})
    return out


def look_geometry(model, q, world):
    """Every number the look post hook adds: tip_px, axes_px, image_to_base, ticks_px, world_px."""
    q = np.asarray(q, dtype=float)
    fk = model.fk(q)
    w, h = int(model.cam_w), int(model.cam_h)
    tip = project(model, q, fk["tip"][0])
    return {"tip": tip, "tip_in_view": tip is not None and in_frame(tip[0], tip[1], w, h),
            "axes_px": axis_arrows(model, q, fk, tip), "image_to_base": image_to_base(model, q, fk, tip),
            "ticks_px": forward_ticks(model, q, fk, w, h), "world_px": world_points(model, q, world, w, h),
            "size": (w, h)}


RIGID_TOL = 1e-6                  # camera pose change in the tool frame (m, and rotation matrix entries) still rigid


def _camera_in_tool(model, q):
    f = model.fk(q)
    p, Rt = f["tip"]
    pc, Rc = f["camera"]
    return Rt.T @ (np.asarray(pc) - p), Rt.T @ np.asarray(Rc)


def rides_with_tip(model, step_rad=0.05):
    """True when no arm joint changes the camera's pose in the tool frame: the camera is rigid with the tool, so
    the tip keeps its pixel in every look (in front of the camera or not). Uses fk only, no projection."""
    q0 = np.asarray(model.q_home, dtype=float)
    p0, R0 = _camera_in_tool(model, q0)
    for i in range(model.n):
        q = q0.copy()
        q[i] = q[i] + step_rad if q[i] + step_rad <= model.q_hi[i] else q[i] - step_rad
        p, R = _camera_in_tool(model, q)
        if float(np.max(np.abs(p - p0))) > RIGID_TOL or float(np.max(np.abs(R - R0))) > RIGID_TOL:
            return False
    return True


# ---------------------------------------------------------------- point formats for mark
def pair(value, name):
    """([a, b] floats, None) or (None, why) for a two-number list."""
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None, f"{name} must be a list of two numbers, got {_show(value)}"
    out = []
    for x in value:
        if isinstance(x, bool) or not isinstance(x, (int, float, np.integer, np.floating)) \
                or not math.isfinite(float(x)):
            return None, f"{name} must be a list of two finite numbers, got {_show(value)}"
        out.append(float(x))
    return out, None


def _show(value):
    text = repr(value)
    return text if len(text) <= 60 else text[:57] + "..."


_HINT = ("give the point exactly one way: u and v in pixels, px [u, v] in pixels, norm1000_yx [y, x] from 0 to "
         "1000, or norm_xy [x, y] from 0 to 1 (u and x right, v and y down, from the top-left)")


def given_format(kwargs):
    """Which point formats a mark call names: a list from u_v, px, norm1000_yx, norm_xy (u or v alone counts)."""
    got = ["u_v"] if ("u" in kwargs or "v" in kwargs) else []
    return got + [k for k in FORMATS if k in kwargs]


def parse_point(kwargs, w, h):
    """The mark pre hook's decision, a pure function of the call's keyword arguments and the image size.

    Returns None (nothing to convert: u and v given, or a positional round one call), ("uv", u, v, fmt) for a
    converted point, or a refusal. Exactly one format must be given."""
    got = given_format(kwargs)
    if not got:
        if "look_id" in kwargs:   # u and v cannot be positional once look_id is a keyword
            return refusal("point_format", "mark got no point", _HINT)
        return None
    if len(got) > 1:
        return refusal("point_format", "mark got the point " + " and ".join(got) + "; exactly one is allowed",
                       _HINT)
    fmt = got[0]
    if fmt == "u_v":
        if "u" not in kwargs or "v" not in kwargs:
            return refusal("point_format", "mark got " + ("u without v" if "u" in kwargs else "v without u"),
                           _HINT)
        return None
    val, why = pair(kwargs[fmt], fmt)
    if why:
        return refusal("point_format", why, _HINT)
    if fmt == "px":
        return "uv", val[0], val[1], fmt
    lim = 1000.0 if fmt == "norm1000_yx" else 1.0
    if not all(0.0 <= x <= lim for x in val):
        hint = _HINT
        if fmt == "norm_xy" and all(0.0 <= x <= 1000.0 for x in val):
            hint = "norm_xy is [x, y] from 0 to 1; for values up to 1000 use norm1000_yx [y, x]; " + _HINT
        elif fmt == "norm1000_yx":
            hint = "norm1000_yx is [y, x] from 0 to 1000; for pixels use px [u, v]; " + _HINT
        return refusal("point_format", f"{fmt} values must be from 0 to {lim:g}, got {_show(kwargs[fmt])}", hint)
    if fmt == "norm1000_yx":
        x, y = val[1] / 1000.0, val[0] / 1000.0
    else:
        x, y = val
    # the far edge (1.0 or 1000) is the last pixel's far side: keep it inside the frame
    u = min(x * w, w - 1e-6)
    v = min(y * h, h - 1e-6)
    return "uv", u, v, fmt


def as_formats(u, v, w, h):
    """The same pixel in the other point formats, for the mark result."""
    return {"px": [_r(u, 2), _r(v, 2)], "norm1000_yx": [_r(v / h * 1000.0, 2), _r(u / w * 1000.0, 2)],
            "norm_xy": [_r(u / w, 5), _r(v / h, 5)]}
