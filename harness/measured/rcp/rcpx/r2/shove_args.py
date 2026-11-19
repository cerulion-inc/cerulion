# SPDX-License-Identifier: AGPL-3.0-only
"""Meaningful argument checks for a horizontal push."""
import math
import numpy as np
from rcpx.r2.shove_bounds import refuse as refusal
from rcpx.r2 import shove_bounds as b

MM = 1000
CONTACTS = ("closed_jaws", "open_jaws")


def horizontal(v):
    v = np.asarray(v, float).copy()
    v[2] = 0
    n = float(np.linalg.norm(v))
    return v / n if n else v


def finite(x):
    try:
        return not isinstance(x, bool) and math.isfinite(float(x))
    except (TypeError, ValueError, OverflowError):
        return False


def resolve_point(ex, value, what):
    if isinstance(value, str):
        hits = [e for e in ex.world.query(label=value) if e.get("point_mm") is not None]
        fresh = [e for e in hits if not e.get("stale")]
        if not fresh:
            return None, None, refusal(what, f"robot://world holds no fresh point labelled {value!r}" + (" (it is only known from before "
                                             "something moved)" if hits else ""),
                                       "locate it again (look, mark it in two looks, triangulate) or give [x, y, z] in base frame mm")
        return np.asarray(fresh[-1]["point_mm"], float) / MM, value, None
    try:
        p = np.asarray(value, float)
    except (TypeError, ValueError):
        p = np.zeros(0)
    if p.shape != (3,) or not np.all(np.isfinite(p)):
        return None, None, refusal(what, f"{what} must be [x, y, z] in base frame mm or a located label, got {value!r}",
                                   "give the three coordinates of a point on the thing, or the label you located it under")
    return p / MM, None, None


def push_direction(ex, point, toward_mm, toward_label, direction_deg):
    given = [v for v in (toward_mm, toward_label, direction_deg) if v is not None]
    if len(given) != 1:
        return None, refusal("direction", "give the push direction as exactly one of toward_mm, toward_label or direction_deg",
                             "direction_deg=0 pushes straight out along the base x axis; it turns toward y as it grows")
    if direction_deg is not None:
        if not finite(direction_deg):
            return None, refusal("direction_deg", f"direction_deg must be a finite number of degrees, got {direction_deg!r}",
                                 "degrees from the base x axis toward y, for example direction_deg=90")
        a = math.radians(float(direction_deg))
        return np.array([math.cos(a), math.sin(a), 0]), None
    goal, _, bad = resolve_point(ex, toward_mm if toward_label is None else toward_label, "toward")
    if bad:
        return None, bad
    flat = goal - point
    flat[2] = 0
    d = horizontal(flat)
    B = b.of(ex)
    if np.linalg.norm(flat) <= b.same_spot(B, point, goal, b.sense(ex).q, d):
        return None, refusal("direction", "the point to push toward is the same spot as the point on the thing, within their "
                             "position errors: no direction follows",
                             "give direction_deg, or a point farther along the way")
    return d, None


def push_args(ex, point_mm, label, toward_mm, toward_label, direction_deg, distance_mm, contact, height_mm,
              max_force_n, standoff_mm, standoff_default_mm=None, clear=None, allow_derived_distance=False):
    if (ex.state.get("place") or {}).get("held"):
        return None, refusal("holding", "the jaws hold a thing: a push would carry it along",
                             "set it down first (the place tool), then push")
    if (point_mm is None) == (label is None):
        return None, refusal("point", "give the thing as exactly one of point_mm or label",
                             "a located point on its side nearest the hand: point_mm=[x, y, z], or the label it was located under")
    point, lab, bad = resolve_point(ex, point_mm if label is None else label, "point_mm" if label is None else "label")
    if bad:
        return None, bad
    d, bad = push_direction(ex, point, toward_mm, toward_label, direction_deg)
    if bad:
        return None, bad
    for name, value in (("distance_mm", distance_mm), ("height_mm", height_mm),
                        ("standoff_mm", standoff_mm), ("max_force_n", max_force_n)):
        if value is None and (name != "distance_mm" or allow_derived_distance):
            continue
        if not finite(value) or float(value) <= 0:
            return None, refusal(name, f"{name} must be a finite number above zero, got {value!r}",
                                 "leave it out and the robot derives it" if name != "distance_mm" else
                                 "state how far to push, in mm")
    if contact is not None and contact not in CONTACTS:
        return None, refusal("contact", f"unknown contact {contact!r}", "contact='closed_jaws' or contact='open_jaws'")
    if contact == "open_jaws" and not ex.model.spec.get("gripper"):
        return None, refusal("contact", "this hand has no jaws to open", "leave contact out")
    B, s = b.of(ex), b.sense(ex)
    E = B.thing(lab if lab else point)
    intent = {"clear": clear, "work_part": True, "move": True}
    if max_force_n is not None:
        intent["max_n"] = float(max_force_n)
    if distance_mm is not None:
        intent["travel"] = float(distance_mm) / MM
    fs = B.force("follow", E, s.q, d, intent)
    if max_force_n is not None and float(max_force_n) > fs.ceiling_n:
        it = fs.items[0]
        return None, refusal("max_force_n", f"{float(max_force_n):g} N is over the ceiling {fs.ceiling_n:g} N here "
                             f"({it['source']}: {it['evidence']})",
                             f"repeat the call with max_force_n={fs.ceiling_n:g}, or leave max_force_n out")
    standoff = B.standoff(B.v_free(s.q, d), E, s.q, d) if standoff_mm is None else float(standoff_mm) / MM
    try:
        support = b.support(B, point, ex.cfg.get("floor_z_m"))
    except ValueError as exc:
        return None, refusal("support", str(exc), "touch the surface the thing rests on, or state the support plane")
    distance = float(distance_mm) / MM if distance_mm is not None else max(0, point[2] - support.point[2] - b.band(B, E))
    keep = b.keep_base(B, E, ex.cfg.get("mount_half_m"), standoff)
    if b.reaches_base(point[:2], d[:2], distance, keep):
        at = b.base_distance(point[:2], d[:2], keep)
        return None, refusal("toward_base", f"the push would bring the thing within the base's own clearance "
                             f"after {at * MM:.0f} mm (the mount, the path clearance and the standoff)",
                             f"push across or away from the base, or at most distance_mm={max(0, at) * MM:.0f} this way")
    if clear is not None:
        word = B.word(intent)
        if word.get("refused"):
            return None, word["refused"]
    return dict(point=point, label=lab, d=d, distance=distance,
                distance_stated=distance_mm is not None, height=None if height_mm is None else float(height_mm) / MM,
                frac=1 if contact == "open_jaws" else 0, force=fs.ceiling_n, force_items=fs.items,
                contact="closed_jaws" if contact is None else contact, standoff=standoff, intent=intent), None
