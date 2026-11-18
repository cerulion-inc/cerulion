# SPDX-License-Identifier: AGPL-3.0-only
"""Package frames (I29, SPEC-R2.md section 3): frames any model and any camera can act on.

- Post hook on look and standard_look, BRIEF by default (FINDINGS S1: a look must stay small): image_dirs (the
  nearest base move_tip argument for image right, left, up and down, and mm per 10 px at the tip), world_px (the
  stored points inside the picture: id, label, px, stale), camera_rides_with_tip, detail (what full adds), and the
  arrows drawn on a NEW file beside the round one annotated frame (annotated_axes; the annotated frame is untouched
  and superseded in the frames list). detail="full" (an argument the pre hook pops when the primitive has no such
  parameter) adds axes_px (the base +x, +y, +z and tool +x, +z arrows at the tip, each labelled with the move_tip
  argument that moves along it), image_to_base (the base unit vector that image right, left, up and down mean at
  the tip's depth), ticks_px (the tool's forward line at 20, 50 and 100 mm) and world_px for every world entry with
  its age; the resource robot://look/{look_id} serves the same for any past look.
- Tool crop(look_id, center_px, size_px=160, scale=3): an enlarged square of a past look, rendered at twice the
  size when the backend can (render accepts scale and the arm is still at the look's pose), else resampled.
- mark accepts px [u, v], norm1000_yx [y, x] or norm_xy [x, y] besides u and v (exactly one): the pre hook turns
  the point into u and v, the post hook adds px_used, format_used, the point in every format and echo_image (the
  look with the mark drawn).

Everything comes from RobotModel.fk and RobotModel.project at the joint pose stored with the look; nothing reads
ground truth and no robot is named. It reads the round one executor's look records (ex._looks, ex._look_rec)
without changing them.

Surface note: the extends mechanism adds arguments but cannot drop u and v from mark's required list, so this
package serves a whole mark entry of the same name (the round one entry plus the new formats, u and v optional);
SPEC-CHANGES.md carries the request for an extends key that relaxes required.
"""

import copy
import inspect
import os

import numpy as np

from rcpx import surfaces
from rcpx.context import refusal
from rcpx.control_base import primitive
from rcpx.perception import brightness, can_see
from rcpx.r2 import frames_draw as draw
from rcpx.r2 import frames_geom as geom

ID = "I29"
CROP_SIZE_PX = (16, 480)
CROP_SCALE = (1.0, 6.0)
POSE_TOL_RAD = 1e-4          # the arm counts as still at the look's pose within this, for a fresh 2x render

_PAIR = {"type": "array", "items": {"type": "number", "minimum": -1e6, "maximum": 1e6}, "minItems": 2,
         "maxItems": 2}
_DETAIL = surfaces._str("brief (default) or full: full adds axes_px, image_to_base, ticks_px and every stored "
                        "point with its age", ["brief", "full"])


def _mark_entry():
    base = next(t for t in surfaces.VNEXT_TOOLS if t["name"] == "mark")
    e = copy.deepcopy(base)
    props = e["args"]["properties"]
    props["u"]["description"] += " (with v; or give px, norm1000_yx or norm_xy instead)"
    props["v"]["description"] += " (with u)"
    props["px"] = dict(_PAIR, description="the point as [u, v] in pixels, u right and v down from the top-left")
    props["norm1000_yx"] = dict(_PAIR, description="the point as [y, x], each from 0 to 1000 of the image height "
                                                   "and width, y down and x right from the top-left")
    props["norm_xy"] = dict(_PAIR, description="the point as [x, y], each from 0 to 1 of the image width and "
                                               "height, x right and y down from the top-left")
    e["args"]["required"] = ["look_id", "label"]
    e["description"] = (e["description"].rstrip() + " Give the point exactly one way: u and v, px [u, v], "
                        "norm1000_yx [y, x] from 0 to 1000, or norm_xy [x, y] from 0 to 1; the result has "
                        "px_used and echo_image (the look with your mark drawn): open it to check where the point "
                        "landed.")
    return e


TOOLS = [
    surfaces._tool(
        "crop",
        "Enlarge part of a past look so a small cue (a lamp, a lever edge, a printed mark) can be read: give the "
        "look_id and the pixel to centre on; returns the enlarged image path and the box it was cut from. Free: it "
        "does not use up a call.",
        surfaces._obj({"look_id": surfaces._ID,
                       "center_px": dict(_PAIR, description="[u, v] pixel of that look to centre on, u right and v "
                                                            "down from the top-left"),
                       "size_px": surfaces._int(CROP_SIZE_PX[0], CROP_SIZE_PX[1],
                                                "side of the square cut from the look, pixels (default 160)"),
                       "scale": surfaces._num(CROP_SCALE[0], CROP_SCALE[1], "how many times to enlarge (default 3)")},
                      ["look_id", "center_px"]),
        {"look_id": "L2", "center_px": [410, 188], "size_px": 120, "scale": 4},
        "cut the 120 px square around pixel (410, 188) of look L2 and show it four times larger"),
    _mark_entry(),
    {"extends": "look", "args": {"detail": _DETAIL},
     "note": "It also returns image_dirs (the base move_tip argument that sends the tip toward image right, left, "
             "up and down, and the mm per 10 px at the tip), world_px (stored points inside this picture) and "
             "annotated_axes, the frame with arrows labelled by move_tip argument (such as dx+ base) drawn on it. "
             "detail full adds axes_px, image_to_base, ticks_px (20, 50, 100 mm ahead along the tool) and every "
             "stored point with its age; robot://look/{look_id} gives the same for any past look."},
    {"extends": "standard_look", "args": {"detail": _DETAIL},
     "note": "Its look carries image_dirs, world_px and annotated_axes as look does (detail full: the rest)."},
]

PRACTICES = (
    "A look's image_dirs and annotated_axes label each arrow with the move_tip argument (such as dy+ base) that "
    "sends the tip that way in that picture, and give the mm per 10 px at the tip.\n"
    "mark takes u and v, px [u, v], norm1000_yx [y, x] or norm_xy [x, y]; its echo_image shows where the point "
    "landed. crop(look_id, center_px) enlarges a small cue (0 control periods)."
)


def _rec_image(rec):
    img = rec.get("img")
    if img is not None:
        return draw.load_rgb(np.asarray(img))
    return draw.load_rgb(rec.get("frame"))


def _beside(ex, rec, stem):
    """A path for a new image next to the look's own frame (or in the executor's frames dir), or None."""
    frame = rec.get("frame")
    d = os.path.dirname(frame) if isinstance(frame, str) else ex._frames_dir()
    if not d:
        return None
    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in stem)[:80]
    return os.path.join(d, safe + ".png")


def _state(ex):
    return ex.state.setdefault("frames", {"rides_with_tip": None, "looks_drawn": 0, "echoes": 0, "crops": 0,
                                          "marks_by_format": {}})


# ---------------------------------------------------------------- hooks
DETAILS = ("brief", "full")
BRIEF_NOTE = "brief; detail=full or robot://look/{look_id} adds axes_px, image_to_base, ticks_px, all points"


def image_dirs(itb):
    """The brief form of image_to_base: the nearest single base move_tip argument for image right, left, up and
    down at the tip, and the mm one 10 px step means there. None when the scale cannot be measured."""
    if not isinstance(itb, dict) or not itb.get("available"):
        return None
    out = {"frame": "base"}
    for word in ("right", "left", "up", "down"):
        n = (itb.get("nearest") or {}).get(word) or {}
        out[word] = str(n.get("move_tip", "")).replace(" base", "") or None
    out["mm_per_10px_at_tip"] = round(10.0 * float(itb["mm_per_px"]), 2)
    return out


def brief_world(points):
    """The stored points that fall inside this look, each as id, label, pixel and stale (the brief world_px)."""
    return [{"id": p.get("id"), "label": p.get("label"), "px": p.get("px"), "stale": p.get("stale")}
            for p in points if p.get("in_view")]


def full_geometry(ex, result, g=None):
    """The keys detail=full adds to a look result (and robot://look/{look_id} serves)."""
    if g is None:
        _, rec = ex._look_rec(result["look_id"])
        g = geom.look_geometry(ex.model, rec["q"], ex.world)
    return {"axes_px": g["axes_px"], "image_to_base": g["image_to_base"], "ticks_px": g["ticks_px"],
            "world_px": g["world_px"]}


def _takes(ex, name, arg):
    fn = getattr(type(ex), name, None)
    try:
        return arg in inspect.signature(getattr(fn, "__wrapped__", fn)).parameters
    except (TypeError, ValueError):
        return False


def _pre_look(ex, name, kwargs):
    d = kwargs.get("detail", "brief")
    if d not in DETAILS:
        return refusal("detail", f"detail is brief or full, not {str(d)[:20]!r}",
                       "leave it out for the brief look; full adds axes_px, image_to_base and ticks_px")
    if "detail" in kwargs and not _takes(ex, name, "detail"):
        kwargs.pop("detail")
    return None


def _post_look(ex, name, kwargs, result):
    if not isinstance(result, dict) or "refused" in result or result.get("look_id") is None:
        return None
    key, rec = ex._look_rec(result["look_id"])
    if rec is None:
        return None
    st = _state(ex)
    g = geom.look_geometry(ex.model, rec["q"], ex.world)
    out = dict(result)
    full = (kwargs or {}).get("detail") == "full"
    if full:
        out.update(full_geometry(ex, result, g), camera_rides_with_tip=st["rides_with_tip"], detail="full")
    else:
        out.update(image_dirs=image_dirs(g["image_to_base"]), world_px=brief_world(g["world_px"]),
                   camera_rides_with_tip=st["rides_with_tip"], detail=BRIEF_NOTE)
    src = result.get("annotated")
    img = draw.load_rgb(src) if isinstance(src, str) else None
    if img is None:
        out["annotated_axes"] = None
        out["annotated_axes_why"] = "the look's annotated frame is not a saved file here; the numbers still hold"
    else:
        path = os.path.splitext(src)[0] + "_axes.png"
        out["annotated_axes"] = draw.save_rgb(draw.draw_axes(img, g), path)
        if isinstance(out["annotated_axes"], str):
            out["supersedes"] = {"annotated_axes": "annotated"}   # the axes frame is the annotated one, drawn on
    st["looks_drawn"] += 1
    return out


def _look_resource(ex, session, look_id=None, **_kw):
    """robot://look/{look_id}: the full geometry of a past look (what detail=full adds), from its stored pose."""
    key, rec = ex._look_rec(look_id)
    if rec is None:
        return refusal("look_id", f"no look {str(look_id)[:20]!r} in this session", "a look_id a look returned")
    return {"look_id": key, "can_see": rec.get("can_see"), **full_geometry(ex, {"look_id": key})}


RESOURCES = {"robot://look/{look_id}": (_look_resource, "The full geometry of one past look: axes_px, "
                                                        "image_to_base, ticks_px and every stored point projected "
                                                        "into it (what look's detail=full adds). Free.")}


def _pre_mark(ex, name, kwargs):
    r = geom.parse_point(kwargs, int(ex.model.cam_w), int(ex.model.cam_h))
    if r is None or isinstance(r, dict):
        return r
    _, u, v, fmt = r
    kwargs.pop(fmt, None)
    kwargs["u"], kwargs["v"] = u, v
    return None


def _post_mark(ex, name, kwargs, result):
    if not isinstance(result, dict) or "refused" in result or result.get("u") is None:
        return None
    st = _state(ex)
    got = geom.given_format(kwargs)
    fmt = got[0] if len(got) == 1 else "u_v"
    st["marks_by_format"][fmt] = st["marks_by_format"].get(fmt, 0) + 1
    u, v = float(result["u"]), float(result["v"])
    w, h = int(ex.model.cam_w), int(ex.model.cam_h)
    out = dict(result)
    out.update(px_used=[round(u, 2), round(v, 2)], format_used=fmt, as_formats=geom.as_formats(u, v, w, h))
    if fmt == "px" and all(0.0 <= x <= 1.0 for x in (u, v)):
        out["format_note"] = "px values under 1 look normalised; norm_xy takes [x, y] from 0 to 1"
    key, rec = ex._look_rec(result.get("look_id"))
    img = None if rec is None else _rec_image(rec)
    path = None if img is None else _beside(ex, rec, f"{key}_mark{st['echoes'] + 1}_{result.get('label', '')}")
    if path is None:
        out["echo_image"] = None
        out["echo_why"] = "the look's frame is not a saved file here; px_used still holds"
    else:
        st["echoes"] += 1
        out["echo_image"] = draw.save_rgb(draw.draw_mark(img, u, v, str(result.get("label", ""))), path)
    return out


def setup(ex):
    st = _state(ex)
    st["rides_with_tip"] = geom.rides_with_tip(ex.model)
    # for other packages: a nested look (one made inside a hook or a primitive runs without hooks) gets the same
    # keys through ex.state["frames"]["annotate_look"](look_result) -> look_result
    st["annotate_look"] = lambda result, detail="brief": _post_look(ex, "look", {"detail": detail}, result) or result
    ex.add_pre(lambda name, kw: _pre_look(ex, name, kw), names=["look", "standard_look"])
    ex.add_post(lambda name, kw, res: _post_look(ex, name, kw, res), names=["look", "standard_look"])
    ex.add_pre(lambda name, kw: _pre_mark(ex, name, kw), names=["mark"])
    ex.add_post(lambda name, kw, res: _post_mark(ex, name, kw, res), names=["mark"])


# ---------------------------------------------------------------- crop
def _num_in(value, lo, hi, name, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)):
        return None, f"{name} must be a number from {lo:g} to {hi:g}, got {value!r}"
    x = float(value)
    if not np.isfinite(x) or not lo <= x <= hi or (integer and x != int(x)):
        kind = "a whole number" if integer else "a number"
        return None, f"{name} must be {kind} from {lo:g} to {hi:g}, got {value!r}"
    return (int(x) if integer else x), None


def _render_2x(ex, rec):
    """(image at twice the look's size, None) when the backend renders at a scale and the arm still sits at the
    look's pose; else (None, why)."""
    fn = getattr(ex.backend, "render", None)
    try:
        params = inspect.signature(fn).parameters
    except (TypeError, ValueError):
        params = {}
    if "scale" not in params:
        return None, "the backend renders at one size only, so the look's frame was resampled"
    q = ex._sense().q
    if float(np.max(np.abs(np.asarray(q) - np.asarray(rec["q"])))) > POSE_TOL_RAD:
        return None, "the arm moved since the look, so the look's own frame was resampled"
    try:
        img = draw.load_rgb(np.asarray(fn("wrist", scale=2)))
    except Exception as exc:  # a backend that advertises scale but fails keeps the crop working
        return None, f"the backend's scaled render failed ({type(exc).__name__}); the frame was resampled"
    want = (2 * int(ex.model.cam_h), 2 * int(ex.model.cam_w))
    if img is None or img.shape[:2] != want:
        return None, "the backend's scaled render had another size; the frame was resampled"
    return img, None


def crop_look(ex, look_id, center_px, size_px=160, scale=3):
    key, rec = ex._look_rec(look_id)
    if rec is None:
        return refusal("look_id", f"no look {look_id!r} in this session", "pass a look_id a look returned")
    if rec["can_see"] == "dark":
        return refusal("can_see", f"look {key} is dark (brightness {rec['brightness']:.1f} of 255): a crop of it "
                       "shows nothing", "get light on the scene or move the camera, look again, and crop that look")
    c, why = geom.pair(center_px, "center_px")
    if why:
        return refusal("center_px", why, "give [u, v] in pixels of that look, u right and v down, e.g. [320, 240]")
    size, why = _num_in(size_px, CROP_SIZE_PX[0], CROP_SIZE_PX[1], "size_px", integer=True)
    if why:
        return refusal("size_px", why, f"a square side from {CROP_SIZE_PX[0]} to {CROP_SIZE_PX[1]} px, e.g. 160")
    k, why = _num_in(scale, CROP_SCALE[0], CROP_SCALE[1], "scale")
    if why:
        return refusal("scale", why, f"enlarge from {CROP_SCALE[0]:g} to {CROP_SCALE[1]:g} times, e.g. 3")
    img = _rec_image(rec)
    if img is None:
        return refusal("frame", f"look {key}'s frame was not saved as an image", "take a new look and crop that")
    h, w = img.shape[:2]
    if not geom.in_frame(c[0], c[1], w, h):
        return refusal("pixel", f"pixel ({c[0]:g}, {c[1]:g}) is outside the {w} x {h} frame of look {key}",
                       "u from 0 to width, v from 0 to height, from the top-left")
    st = _state(ex)
    path = _beside(ex, rec, f"{key}_crop{st['crops'] + 1}")
    if path is None:
        return refusal("frame", "no frames dir to write the crop into", "run inside an episode")
    u0, v0, side = draw.crop_box(c[0], c[1], size, w, h)
    hi, why_resampled = _render_2x(ex, rec)
    if hi is not None:
        big = draw.enlarge(hi, 2 * u0, 2 * v0, 2 * side, k / 2.0)
        source = "rendered_2x"
    else:
        big = draw.enlarge(img, u0, v0, side, k)
        source = "resampled"
    st["crops"] += 1
    draw.save_rgb(big, path)
    f = big.shape[1] / float(side)
    mmpp = rec.get("mm_per_px")
    out = {"look_id": key, "crop": path, "center_px": [round(c[0], 2), round(c[1], 2)],
           "box_px": [u0, v0, u0 + side, v0 + side], "size_px": side, "scale": round(f, 4),
           "crop_size_px": [int(big.shape[1]), int(big.shape[0])], "source": source,
           "center_in_crop_px": [round((c[0] - u0) * f, 1), round((c[1] - v0) * f, 1)],
           "to_look_px": f"u = {u0} + x / {round(f, 4):g}, v = {v0} + y / {round(f, 4):g} (x, y in the crop)",
           "mm_per_crop_px_at_tip": None if mmpp is None else round(float(mmpp) / f, 4),
           "brightness": round(brightness(big), 1), "can_see": can_see(big)}
    notes = []
    if source == "rendered_2x":
        out["rendered_at_s"] = round(float(ex._sense().t), 3)
        notes.append("rendered again at twice the size at the look's pose: it shows the scene now")
    else:
        out["why_resampled"] = why_resampled
        notes.append("resampled: the look's own pixels enlarged, no detail finer than the look itself")
    if side < size:
        notes.append(f"size_px {size} is larger than the frame; cut {side} px")
    if notes:
        out["note"] = "; ".join(notes)
    return out


class Mixin:
    @primitive
    def crop(self, look_id, center_px, size_px=160, scale=3):
        return crop_look(self, look_id, center_px, size_px, scale)
