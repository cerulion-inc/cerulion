# SPDX-License-Identifier: AGPL-3.0-only
"""Package scene_camera: every look-based tool works from a FIXED scene camera as well as from the
wrist camera. Loaded explicitly (it is not in rcpx.r2.core.PACKAGES): pass this module in the mods list.

Where the cameras come from: backend.scene_cameras() -> {name: info} (or ex.context["scene_cameras"], same shape),
info = {"width", "height", "fx", "fy", "cx", "cy", "fovy_deg", "installation": {"position_m", "R"}} with R the
camera's axes in frame B as columns (image right, image up, toward the viewer: the MuJoCo camera convention). The
installation pose is a fact of how the camera was mounted, like the robot's own base; nothing here reads the scene.

- look(camera=NAME): a picture from that camera (look_id C1, ...): frame and annotated (the tool tip, the base
  axes at the tip labelled with the move_tip argument along them, a 50 mm scale at the tip's depth, the stored
  points), camera_pose, tip_px, mm_per_px_at_tip, image_dirs (the base move_tip argument nearest image right, left,
  up and down at the tip, and how far the tip moves on the floor per 10 px), floor_view_mm (the patch of the surface
  the robot stands on that the view shows) and world_px. camera omitted or "wrist": the round one look.
- mark(look_id=C..., ...): a pixel of a scene look becomes a ray from that camera (u and v, px, norm1000_yx or
  norm_xy, as for a wrist look); triangulate(label) then works on any mix of scene and wrist marks, and stores the
  point as robot://world does for the round one triangulate. mark also says where the pointed ray lands: the depth
  point of that pixel where the camera gives depth (rcpx/r2/see_spot.py), else where the ray meets the support
  plane (labelled so: it is not the top of what was pointed at). triangulate gives the point with its error
  ("sigma_mm": along the view and across, see_spot.marks_error; stored with the label, so a spot named by the
  label states it); rays no further apart than the camera resolves fix no depth, and that is said first; when
  the depth it computes is known no better than the way from the hand to the point it says so too.
- every scene look is noted in the space book and keeps its camera's pose and, where the camera gives one, its
  depth picture: a scene look id is a spot for the whole-step tools,
  {"look": C1, "px": [u, v]} (rcpx/r2/place_point.py), read with no motion. Nothing is carved.
- crop(look_id=C..., center_px): an enlarged square of a scene look (resampled).
- robot://cameras: every camera, its size, lens, pose in frame B and the floor footprint of each fixed view.

Hook order: the mark pre hook goes FIRST (a scene look's point formats are read against its own size, before any
other mark hook reads them against the wrist frame); the others are added in package order. No robot is named.
"""

import math
import os

import numpy as np

from rcpx import perception, surfaces
from rcpx.bounds_ratios import BAND
from rcpx.context import refusal
from rcpx.control_base import _mm

ID = "N3"
MM = 1000.0
PREFIX = "C"
AXIS_MM = 50.0
EDGE_SAMPLES = 3
CROP_SIZE_PX = (16, 480)
CROP_SCALE = (1.0, 6.0)

TOOLS = [
    {"extends": "look",
     "args": {"camera": surfaces._str("which camera: wrist (the default, rides on the hand) or a fixed scene camera "
                                      "named in robot://cameras")},
     "note": "camera names a fixed scene camera instead (robot://cameras lists them): its look_id starts with C, it "
             "shows the arm from outside, and mark, triangulate and crop take it like a wrist look, so a scene mark "
             "and a wrist mark of the same point triangulate it."},
]

PRACTICES = ("look(camera=NAME) takes a picture from a fixed scene camera (robot://cameras; 0 control "
             "periods): it shows the tool tip and the base axes from outside; a point marked there and in a wrist "
             "look triangulates. image_dirs says which move_tip argument moves the tip toward image right, left, up "
             "and down.")


def _state(ex):
    return ex.state.setdefault("scene_camera", {"looks": {}, "n": 0, "crops": 0, "echoes": 0})


def cameras(ex):
    """{name: info} of the fixed cameras (module docstring), or {}."""
    fn = getattr(ex.backend, "scene_cameras", None)
    out = {}
    if callable(fn):
        try:
            out = dict(fn() or {})
        except Exception:
            out = {}
    if not out:
        out = dict((getattr(ex, "context", {}) or {}).get("scene_cameras") or {})
    return {str(k): v for k, v in out.items() if isinstance(v, dict) and v.get("installation")}


class Cam:
    """A fixed pinhole camera in frame B (u right, v down, origin top left)."""

    def __init__(self, name, info):
        self.name = name
        inst = info["installation"]
        self.p = np.asarray(inst["position_m"], dtype=float)
        self.R = np.asarray(inst["R"], dtype=float).reshape(3, 3)
        self.w, self.h = int(info["width"]), int(info["height"])
        f = (self.h / 2.0) / math.tan(math.radians(float(info.get("fovy_deg", info.get("fovy", 45.0)))) / 2.0)
        self.fx, self.fy = float(info.get("fx", f)), float(info.get("fy", f))
        self.cx, self.cy = float(info.get("cx", self.w / 2.0)), float(info.get("cy", self.h / 2.0))

    def project(self, P):
        x, y, z = self.R.T @ (np.asarray(P, dtype=float) - self.p)
        depth = -z
        if depth <= 1e-9:
            return float("nan"), float("nan"), float(depth)
        return self.cx + self.fx * x / depth, self.cy - self.fy * y / depth, float(depth)

    def ray(self, u, v):
        d = self.R @ np.array([(u - self.cx) / self.fx, -(v - self.cy) / self.fy, -1.0])
        return self.p.copy(), d / np.linalg.norm(d)

    def pose(self):
        return {"position_mm": _mm(self.p), "looks_along": [round(float(x), 4) for x in -self.R[:, 2]],
                "image_right": [round(float(x), 4) for x in self.R[:, 0]],
                "image_down": [round(float(x), 4) for x in -self.R[:, 1]]}

    def inside(self, u, v):
        return bool(np.isfinite(u) and np.isfinite(v) and 0.0 <= u < self.w and 0.0 <= v < self.h)


def _cam(ex, name):
    info = cameras(ex).get(str(name))
    return None if info is None else Cam(str(name), info)


def _floor_hit(ex, o, d):
    z = float(ex.cfg["floor_z_m"])
    if d[2] > -1e-6:
        return None
    t = (z - float(o[2])) / float(d[2])
    return None if t <= 0.0 else o + t * d


def footprint(ex, cam):
    """floor_view_mm of a fixed view: where the frame's edges meet the floor (clockwise from the top-left), the
    floor point at the centre, the nearest and farthest distance from the base axis, and whether part of the view
    is above the horizon. None when no part of it looks at the floor."""
    W, H, k = cam.w, cam.h, EDGE_SAMPLES
    rim = ([(W * i / k, 0.0) for i in range(k)] + [(W, H * i / k) for i in range(k)]
           + [(W * (k - i) / k, H) for i in range(k)] + [(0.0, H * (k - i) / k) for i in range(k)])
    pts, sky = [], False
    for u, v in rim:
        p = _floor_hit(ex, *cam.ray(u, v))
        if p is None:
            sky = True
        else:
            pts.append(p)
    if not pts:
        return None
    c = _floor_hit(ex, *cam.ray(W / 2.0, H / 2.0))
    r = [float(np.hypot(p[0], p[1])) for p in pts]
    return {"z_mm": round(float(ex.cfg["floor_z_m"]) * MM, 1),
            "outline": [[round(float(p[0]) * MM), round(float(p[1]) * MM)] for p in pts],
            "centre": None if c is None else [round(float(c[0]) * MM), round(float(c[1]) * MM)],
            "radius_mm": [round(min(r) * MM), round(max(r) * MM)], "above_horizon": sky}


_AXES = (("dx", np.array([1.0, 0, 0])), ("dy", np.array([0, 1.0, 0])), ("dz", np.array([0, 0, 1.0])))


def image_dirs(cam, tip):
    """At the tip: the base move_tip argument nearest image right, left, up and down (the axis whose projected
    motion points most that way), the floor move per 10 px (a 2 x 2 map from a pixel step to dx, dy in mm on the
    horizontal plane through the tip) and mm per px at the tip's depth."""
    u0, v0, depth = cam.project(tip)
    if not depth > 1e-6:
        return None
    eps = 0.01
    J = []
    for _, a in _AXES:
        u1, v1, _ = cam.project(tip + eps * a)
        J.append([(u1 - u0) / eps, (v1 - v0) / eps])   # px per m along each base axis
    J = np.array(J)
    out = {"frame": "base"}
    for word, want in (("right", (1, 0)), ("left", (-1, 0)), ("up", (0, -1)), ("down", (0, 1))):
        best = None
        for (name, _), row in zip(_AXES, J):
            n = float(np.linalg.norm(row))
            if n < 1e-9:
                continue
            for sign in (1.0, -1.0):
                c = sign * float(row @ np.array(want, dtype=float)) / n
                if best is None or c > best[0]:
                    best = (c, name + ("+" if sign > 0 else "-"))
        out[word] = None if best is None or best[0] < 0.3 else best[1]
    A = J[:2].T                                         # (du, dv) per (dx, dy) in m
    if abs(float(np.linalg.det(A))) > 1e-6:
        inv = np.linalg.inv(A) * MM * 10.0              # mm of (dx, dy) per 10 px of (du, dv)
        out["floor_mm_per_10px"] = {"right": [round(float(x), 1) for x in inv[:, 0]],
                                    "down": [round(float(x), 1) for x in inv[:, 1]]}
    out["mm_per_px_at_tip"] = round(depth * MM / cam.fy, 3)
    return out


def world_px(ex, cam):
    """The stored points that fall inside this view: id, label, px, stale."""
    out = []
    for e in ex.world.query(include_stale=True) if hasattr(ex.world, "query") else []:
        p = e.get("point_mm")
        if not p:
            continue
        u, v, depth = cam.project(np.asarray(p, dtype=float) / MM)
        if depth > 0 and cam.inside(u, v):
            out.append({"id": e.get("id"), "label": e.get("label"), "px": [round(u, 1), round(v, 1)],
                        "stale": bool(e.get("stale"))})
    return out


def _draw(img, cam, tip, points):
    """The annotated twin: the tip, the base axes at the tip, a 50 mm scale at its depth, the stored points."""
    from PIL import Image, ImageDraw

    im = Image.fromarray(np.ascontiguousarray(np.asarray(img)[:, :, :3]))
    dr = ImageDraw.Draw(im)
    u0, v0, depth = cam.project(tip)
    colors = {"dx": (230, 40, 40), "dy": (40, 180, 40), "dz": (60, 90, 240)}
    if depth > 0 and np.isfinite(u0):
        for name, a in _AXES:
            u1, v1, d1 = cam.project(tip + a * AXIS_MM / MM)
            if d1 > 0 and np.isfinite(u1):
                dr.line([(u0, v0), (u1, v1)], fill=colors[name], width=2)
                dr.text((u1 + 2, v1 - 6), name + "+", fill=colors[name])
        dr.ellipse([u0 - 4, v0 - 4, u0 + 4, v0 + 4], outline=(255, 255, 0), width=2)
        dr.text((u0 + 5, v0 + 3), "tip", fill=(255, 255, 0))
        bar = AXIS_MM / MM * cam.fy / depth
        y = cam.h - 8
        dr.line([(6, y), (6 + bar, y)], fill=(255, 255, 255), width=3)
        dr.text((8, y - 12), f"{AXIS_MM:g} mm at the tip", fill=(255, 255, 255))
    for pt in points:
        u, v = pt["px"]
        dr.line([(u - 4, v), (u + 4, v)], fill=(255, 160, 0), width=1)
        dr.line([(u, v - 4), (u, v + 4)], fill=(255, 160, 0), width=1)
        dr.text((u + 4, v + 2), str(pt.get("label"))[:20], fill=(255, 160, 0))
    return np.asarray(im)


# ---------------------------------------------------------------------------------------------- look
def scene_look(ex, camera, note=""):
    cam = _cam(ex, camera)
    names = sorted(cameras(ex))
    if cam is None:
        return refusal("camera", f"no camera {str(camera)[:40]!r}: the cameras are wrist" +
                       (" and " + ", ".join(names) if names else " only"), "read robot://cameras")
    s, eased = ex._ease_if_pressing()
    img = np.asarray(ex.backend.render(cam.name))
    light = perception.light(img)
    st = _state(ex)
    st["n"] += 1
    look_id = f"{PREFIX}{st['n']}"
    tip = ex.model.fk_tip(s.q)[0]
    pts = world_px(ex, cam)
    frame = ex._save(img, f"{look_id}_{cam.name}_raw")
    annotated = ex._save(_draw(img, cam, tip, pts), f"{look_id}_{cam.name}_annotated")
    b, cs = light["brightness"], light["can_see"]
    st["looks"][look_id] = {"camera": cam.name, "img": img, "frame": frame, "annotated": annotated, "can_see": cs,
                            "brightness": b, "q": s.q.copy(), "t": s.t, **ex._look_keeps(s, cam.name),
                            "cam": {"name": cam.name, "w": cam.w, "h": cam.h, "fx": cam.fx, "fy": cam.fy,
                                    "cx": cam.cx, "cy": cam.cy, "pos": cam.p.copy(), "R": cam.R.copy()}}
    ex._note_light(b, cs, "look", look_id)
    ex.world.note_look(look_id, cs, frame=frame if isinstance(frame, str) else None, t_s=s.t)
    u, v, depth = cam.project(tip)
    out = {"look_id": look_id, "camera": cam.name, "fixed_camera": True, "frame": frame, "annotated": annotated,
           "brightness": round(b, 1), "can_see": cs, "camera_pose": cam.pose(),
           "tip_px": [round(u, 1), round(v, 1)] if depth > 0 and cam.inside(u, v) else None,
           "tip_in_view": bool(depth > 0 and cam.inside(u, v)), "image_size": [cam.w, cam.h],
           "mm_per_px_at_tip": round(depth * MM / cam.fy, 3) if depth > 0 else None,
           "image_dirs": image_dirs(cam, tip), "floor_view_mm": footprint(ex, cam), "world_px": pts,
           "note": str(note or "")}
    if cs == "dark":
        out["warning"] = "too dark to see: a mark on this look locates nothing"
    _note(ex, look_id, cam, frame, s.t)
    out["depth"] = st["looks"][look_id].get("depth") is not None
    if eased:
        out["eased"] = "a joint was still pressing; eased before the picture"
    return {"said": f"Scene look {look_id}: point at a thing or a place in it with "
                    f"{{\"look\": \"{look_id}\", \"px\": [u, v]}}.", **out}


def _note(ex, look_id, cam, frame, t):
    """Note the look in the space book (the operator's word can name it): the camera as the book takes one, the
    camera frame's axes image right, image down, along the view. A failure costs the operator nothing."""
    try:
        from rcpx import bounds
        T = np.eye(len(cam.p) + 1)
        T[:len(cam.p), :len(cam.p)] = cam.R @ np.diag([1.0, -1.0, -1.0])
        T[:len(cam.p), len(cam.p)] = cam.p
        K = np.array([[cam.fx, 0.0, cam.cx], [0.0, cam.fy, cam.cy], [0.0, 0.0, 1.0]])
        bounds.of(ex).space.note_look(look_id, {"T_base_cam": T, "K": K, "size": [cam.w, cam.h]},
                                      frame=frame if isinstance(frame, str) else None, at=t, shown=True)
    except Exception as exc:
        ex._journal("bounds_problem", where="note_look", error=f"{type(exc).__name__}: {exc}")


def _pre_look(ex, name, kwargs):
    cam = kwargs.pop("camera", None)
    if cam is None or cam == "wrist":
        return None
    if not isinstance(cam, str):
        return refusal("camera", "camera is a name: wrist or a scene camera from robot://cameras",
                       "leave it out for the wrist camera")
    out = scene_look(ex, cam, kwargs.get("note", ""))
    return ("skip", out) if "refused" not in out else out


# ---------------------------------------------------------------------------------------------- mark
def _point(kwargs, w, h):
    """(u, v, format) of a mark's point against a w x h frame, or a refusal: exactly one of u and v, px [u, v],
    norm1000_yx [y, x] (0 to 1000) or norm_xy [x, y] (0 to 1)."""
    given = [k for k in ("px", "norm1000_yx", "norm_xy") if kwargs.get(k) is not None]
    if "u" in kwargs or "v" in kwargs:
        given.insert(0, "u_v")
    hint = "give exactly one: u and v, px [u, v], norm1000_yx [y, x] from 0 to 1000, or norm_xy [x, y] from 0 to 1"
    if len(given) != 1:
        return refusal("point_format", f"mark got {len(given)} point formats", hint)
    fmt = given[0]
    try:
        if fmt == "u_v":
            u, v = float(kwargs["u"]), float(kwargs["v"])
        else:
            a, b = (float(x) for x in kwargs[fmt])
            if fmt == "px":
                u, v = a, b
            elif fmt == "norm1000_yx":
                u, v = min(b / 1000.0 * w, w - 1e-6), min(a / 1000.0 * h, h - 1e-6)
            else:
                u, v = min(a * w, w - 1e-6), min(b * h, h - 1e-6)
    except (KeyError, TypeError, ValueError):
        return refusal("point_format", f"{fmt} needs two numbers", hint)
    if not (math.isfinite(u) and math.isfinite(v)):
        return refusal("point_format", "the point is not finite", hint)
    return u, v, fmt


def _echo(ex, rec, look_id, u, v, label):
    from PIL import Image, ImageDraw

    frame = rec.get("frame")
    if not isinstance(frame, str):
        return None
    st = _state(ex)
    st["echoes"] += 1
    im = Image.fromarray(np.ascontiguousarray(np.asarray(rec["img"])[:, :, :3]))
    dr = ImageDraw.Draw(im)
    dr.ellipse([u - 5, v - 5, u + 5, v + 5], outline=(255, 0, 255), width=2)
    dr.text((u + 6, v - 6), str(label)[:24], fill=(255, 0, 255))
    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in f"{look_id}_mark{st['echoes']}_{label}")[:80]
    path = os.path.join(os.path.dirname(frame), safe + ".png")
    im.save(path)
    return path


def scene_mark(ex, look_id, rec, kwargs):
    cam = _cam(ex, rec["camera"])
    if cam is None:
        return refusal("camera", f"the camera of look {look_id} is gone", "take a new look")
    if rec["can_see"] == "dark":
        return refusal("can_see", f"look {look_id} is dark: a mark on it locates nothing", "look again with light")
    pt = _point(kwargs, cam.w, cam.h)
    if isinstance(pt, dict):
        return pt
    u, v, fmt = pt
    if not cam.inside(u, v):
        return refusal("pixel", f"pixel ({u:g}, {v:g}) is outside the {cam.w} x {cam.h} frame of look {look_id}",
                       "u from 0 to width, v from 0 to height, from the top-left")
    label = str(kwargs.get("label", ""))
    if not label:
        return refusal("label", "mark needs a label", "name the point, e.g. corner")
    o, d = cam.ray(u, v)
    marks = ex._marks.setdefault(label, {})
    marks[look_id] = {"u": u, "v": v, "origin": o, "dir": d, "camera": cam.name, "f_px": cam.fy}
    hit = _floor_hit(ex, o, d)
    out = {"label": label, "look_id": look_id, "camera": cam.name, "u": u, "v": v, "marks": len(marks),
           "ray": {"origin_mm": _mm(o), "dir": [round(float(x), 4) for x in d]},
           "support_plane_hit": {"point_mm": None if hit is None else _mm(hit),
                                 "what": "where the ray meets the support plane (not the top of what you pointed "
                                         "at); point with {\"look\", \"px\"} and the tool finds the thing"},
           "px_used": [round(u, 2), round(v, 2)],
           "format_used": fmt, "echo_image": _echo(ex, rec, look_id, u, v, label)}
    try:      # where the pointed ray lands, by the look's own depth where the camera gives one (no motion)
        from rcpx.r2 import see_spot
        sp = see_spot.spot(ex, {"look": look_id, "px": [u, v]}, "place")
        if "refused" not in sp and sp["sigma_m"]["along_ray"] is not None:
            out["point"] = {"point_mm": _mm(np.asarray(sp["point_m"])), "what": sp["source"],
                            "sigma_mm": {k: float(f"{float(v_) * MM:.3g}") for k, v_ in sp["sigma_m"].items()}}
    except Exception as exc:
        ex._journal("bounds_problem", where="mark_point", error=f"{type(exc).__name__}: {exc}")
    if len(marks) < 2:
        out["next"] = "mark the same point in a wrist look (or another camera), then triangulate"
    return out


def _pre_mark(ex, name, kwargs):
    look_id = str(kwargs.get("look_id", ""))
    rec = _state(ex)["looks"].get(look_id)
    if rec is None:
        return None
    out = scene_mark(ex, look_id, rec, kwargs)
    return ("skip", out) if "refused" not in out else out


def _post_mark(ex, name, kwargs, result):
    """Another mark hook may have drawn its own echo for a scene look against the wrist frame: keep this one's."""
    if not isinstance(result, dict) or str(result.get("look_id", "")) not in _state(ex)["looks"]:
        return None
    mine = (_state(ex).get("last_echo") or {}).get(result.get("look_id"))
    if mine and result.get("echo_image") != mine:
        out = dict(result, echo_image=mine)
        out.pop("echo_why", None)
        return out
    return None


# ---------------------------------------------------------------------------------------------- triangulate
def scene_triangulate(ex, label):
    marks = ex._marks.get(str(label)) or {}
    if len(marks) < 2:
        return refusal("marks", f"{label!r} is marked in {len(marks)} look(s); two are needed",
                       "mark the same point in a second look (a wrist look, or another camera)")
    scene = _state(ex)["looks"]
    for key in marks:
        rec = scene.get(key) or ex._looks.get(key)
        if rec is None or rec.get("can_see") == "dark":
            return refusal("can_see", f"look {key} is dark or gone: its mark cannot be used", "look again with light")
    A, b, dirs = np.zeros((3, 3)), np.zeros(3), []
    for mk in marks.values():
        dv = np.asarray(mk["dir"], dtype=float)
        P = np.eye(3) - np.outer(dv, dv)
        A += P
        b += P @ np.asarray(mk["origin"], dtype=float)
        dirs.append(dv)
    widest = max(math.degrees(math.acos(min(1.0, abs(float(a @ c))))) for a in dirs for c in dirs)
    try:
        p = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        p = None
    f = min(float(mk.get("f_px", ex.model.f_px)) for mk in marks.values())
    q = None if p is None or not np.all(np.isfinite(p)) else perception.triangulation_quality(
        [(mk["origin"], mk["dir"]) for mk in marks.values()], p, f, floor_z_m=ex.cfg["floor_z_m"])
    unc = None if q is None else q["depth_uncertainty_mm"]
    if q is None or unc is None:                 # the marks are one ray: no point lies on both (no plan exists)
        return refusal("baseline", f"the rays differ by {widest:.1f} deg: they are one ray and fix no point",
                       "mark the point from a view further to the side")
    away = float(np.linalg.norm(p - ex.model.fk_tip(ex._sense().q)[0])) * MM
    loose = None
    if BAND * unc >= away:                       # the point's band already holds the hand: said first, not refused
        loose = (f"the rays differ by only {widest:.1f} deg: the depth is known to {unc:.0f} mm, and a band of "
                 f"{BAND} such errors reaches the hand ({away:.0f} mm away); a mark from further to the side fixes it")
    miss, rays = [], []
    for key, mk in marks.items():
        w = p - mk["origin"]
        if float(w @ mk["dir"]) <= 0:
            return refusal("behind", f"the point falls behind the camera of look {key}",
                           "check that the marks show the same point")
        miss.append(float(np.linalg.norm(w - (w @ mk["dir"]) * mk["dir"])))
        rays.append({"look_id": key, "camera": mk.get("camera", "wrist"), "origin_mm": _mm(mk["origin"]),
                     "dir": [round(float(x), 4) for x in mk["dir"]], "miss_mm": round(miss[-1] * MM, 2)})
    err = max(miss) * MM
    pm = _mm(p)
    # what the point is worth, kept WITH the label (error_mm is only how far the rays miss each other)
    from rcpx.r2 import see_spot
    worth = see_spot.marks_error([(mk["origin"], mk["dir"]) for mk in marks.values()], p, f)
    data = {"error_mm": round(err, 2), "looks": list(marks), **see_spot.marks_data(worth)}
    found = ex.world.query(kind="located", label=str(label))
    if found:
        e = ex.world.update(found[-1]["id"], point_mm=pm, source="triangulate", data=data)
    else:
        e = ex.world.add("located", str(label), point_mm=pm, source="triangulate", data=data)
    out = {"label": str(label), "point_mm": pm, "error_mm": round(err, 2), "rays": rays, "id": e.get("id"),
           "ray_angle_deg": round(widest, 2), "depth_uncertainty_mm": q["depth_uncertainty_mm"],
           "sigma_mm": data["sigma_mm"], "cameras": sorted({r["camera"] for r in rays})}
    warn = ([see_spot.no_depth_text(worth)] if worth["along_m"] is None else []) + ([loose] if loose else []) + \
        list(q["warnings"])
    if warn:
        out["warning"] = "; ".join(warn)
    return out


def _pre_triangulate(ex, name, kwargs):
    label = str(kwargs.get("label", ""))
    marks = ex._marks.get(label) or {}
    if not any(k in _state(ex)["looks"] for k in marks):
        return None
    return "skip", scene_triangulate(ex, label)


# ---------------------------------------------------------------------------------------------- crop
def scene_crop(ex, look_id, rec, center_px, size_px=160, scale=3):
    from PIL import Image

    try:
        cu, cv = (float(x) for x in center_px)
        size, k = int(size_px), float(scale)
    except (TypeError, ValueError):
        return refusal("center_px", "center_px is [u, v]; size_px a whole number; scale a number",
                       "e.g. center_px [128, 100]")
    if not (CROP_SIZE_PX[0] <= size <= CROP_SIZE_PX[1] and CROP_SCALE[0] <= k <= CROP_SCALE[1]):
        return refusal("size_px", f"size_px {CROP_SIZE_PX[0]} to {CROP_SIZE_PX[1]}, scale {CROP_SCALE[0]:g} to "
                       f"{CROP_SCALE[1]:g}", "e.g. size_px 120, scale 3")
    img = np.asarray(rec["img"])
    h, w = img.shape[:2]
    if not (0 <= cu < w and 0 <= cv < h):
        return refusal("pixel", f"pixel ({cu:g}, {cv:g}) is outside the {w} x {h} frame of look {look_id}",
                       "u from 0 to width, v from 0 to height, from the top-left")
    side = min(size, w, h)
    u0 = int(min(max(0, round(cu - side / 2)), w - side))
    v0 = int(min(max(0, round(cv - side / 2)), h - side))
    big = Image.fromarray(np.ascontiguousarray(img[v0:v0 + side, u0:u0 + side, :3])).resize(
        (int(round(side * k)), int(round(side * k))), Image.BICUBIC)
    st = _state(ex)
    st["crops"] += 1
    frame = rec.get("frame")
    if not isinstance(frame, str):
        return refusal("frame", "the look's frame was not saved", "run inside an episode")
    path = os.path.join(os.path.dirname(frame), f"{look_id}_crop{st['crops']}.png")
    big.save(path)
    f = big.size[0] / float(side)
    return {"look_id": look_id, "camera": rec["camera"], "crop": path, "center_px": [round(cu, 2), round(cv, 2)],
            "box_px": [u0, v0, u0 + side, v0 + side], "size_px": side, "scale": round(f, 4), "source": "resampled",
            "to_look_px": f"u = {u0} + x / {round(f, 4):g}, v = {v0} + y / {round(f, 4):g} (x, y in the crop)"}


def _pre_crop(ex, name, kwargs):
    look_id = str(kwargs.get("look_id", ""))
    rec = _state(ex)["looks"].get(look_id)
    if rec is None:
        return None
    out = scene_crop(ex, look_id, rec, kwargs.get("center_px"), kwargs.get("size_px", 160), kwargs.get("scale", 3))
    return ("skip", out) if "refused" not in out else out


# ---------------------------------------------------------------------------------------------- resource, setup
def cameras_doc(ex, session=None, **_kw):
    """robot://cameras."""
    wc = ex.model.spec["camera"]
    out = [{"name": "wrist", "fixed": False, "rides_with": wc["body"], "width": int(wc["width"]),
            "height": int(wc["height"]), "fovy_deg": float(wc["fovy"]),
            "note": "on the hand: its pose follows the arm (look's camera_pose)"}]
    for name, info in sorted(cameras(ex).items()):
        cam = Cam(name, info)
        out.append({"name": name, "fixed": True, "width": cam.w, "height": cam.h,
                    "fovy_deg": float(info.get("fovy_deg", 0.0)), "fx": cam.fx, "fy": cam.fy, "cx": cam.cx,
                    "cy": cam.cy, "pose": cam.pose(), "floor_view_mm": footprint(ex, cam),
                    "source": "the installation: where the camera is mounted, frame B"})
    return {"cameras": out, "frame": "B (base): mm; image u right, v down from the top-left"}


RESOURCES = {"robot://cameras": (cameras_doc, "Every camera: the wrist camera and each fixed scene camera with its "
                                              "size, lens, pose in the base frame and the floor patch it shows. "
                                              "Free.")}


def setup(ex):
    st = _state(ex)
    st["last_echo"] = {}

    def pre_mark(name, kw):
        r = _pre_mark(ex, name, kw)
        if isinstance(r, tuple) and isinstance(r[1], dict) and r[1].get("echo_image"):
            st["last_echo"][r[1]["look_id"]] = r[1]["echo_image"]
        return r
    pre = getattr(ex, "_pre", None)
    if isinstance(pre, list):    # first: a scene look's point is read against its own frame (module docstring)
        pre.insert(0, (pre_mark, {"mark"}))
    else:
        ex.add_pre(pre_mark, names=["mark"])
    ex.add_pre(lambda name, kw: _pre_look(ex, name, kw), names=["look"])
    ex.add_pre(lambda name, kw: _pre_triangulate(ex, name, kw), names=["triangulate"])
    ex.add_pre(lambda name, kw: _pre_crop(ex, name, kw), names=["crop"])
    ex.add_post(lambda name, kw, res: _post_mark(ex, name, kw, res), names=["mark"])
