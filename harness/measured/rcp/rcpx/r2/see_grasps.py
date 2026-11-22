# SPDX-License-Identifier: AGPL-3.0-only
"""Every way the hand can take the THING a pixel lies on.

A pixel names the thing it lies on, not a point: wherever on the thing the pixel lies (its top, its outer side, its
handle, its foot), the take goes to the thing. ways(ex, seen) reads the thing's own points in the look the spot was
pointed in (see_cloud.component: the points extent() took as the thing) and gives, nearest the pixel first:

- rim: where the thing is hollow (its top level is a ring: top points stand all around its seen middle and none at
  it), the point of its rim nearest the pixel, the level direction from there to its middle and the wall's thickness
  along it: the jaws take the wall there, one finger inside, WHEREVER the pixel lies on the thing (the pointed ray
  need not land in the hollow);
- chord: the narrowest chords of the thing's footprint that the look shows, read through points one pad apart all
  over the part of it the fingers can reach from above (within one finger depth of its top), each with its
  bearing, its width and its own top.

Each way says how far its middle lies from the pixel's own point. Which of them fits the hand, and which this arm
can take, is place_grasp.choose's to say. No size, name or pose of any thing is typed: the lengths are the hand's
own (see_spot.hand_of: pad, finger, opening), the look's own (one pixel's footprint, the depth's stated error) and
the named ratios. Clouds are read only through the frozen functions of see_spot and see_cloud. No motion, no
control period. Nothing here reads ground truth or names a robot. Lengths m, angles rad, the level axes of
see_cloud.axes_of(up) (the base x and y where up is the base z).
"""

import math

import numpy as np

from rcpx.bounds_ratios import BAND, QUANT
from rcpx.r2 import see_cloud as SC

SECTORS = int(QUANT * QUANT)        # the quarters of a turn a rim must stand in to stand all around a middle


def _cloud(ex, seen):
    """(view, points, valid, the look's key) of the look the spot was pointed in that gives depth, or None."""
    from rcpx.r2 import see_spot
    sp = seen.get("spot") if isinstance(seen.get("spot"), dict) else {}
    for key in sp.get("looks") or []:
        rec = see_spot.look_of(ex, key)[1]
        got = None if rec is None else see_spot.cloud_of(ex, rec)
        if got is not None:
            return got[0], got[1], got[2], key
    return None


def thing(ex, seen):
    """The thing the spot lies on as its own points: {"xy", "z", "top", "base", "foot", "at": the pixel's point on
    the level axes, "sigma", "hand", "look"}, or None (the spot came with no look that gives depth, or no thing is
    made out at it)."""
    from rcpx.r2 import see_spot
    sp = seen.get("spot") if isinstance(seen.get("spot"), dict) else {}
    e = sp.get("extent") if isinstance(sp.get("extent"), dict) else {}
    if e.get("kind") != "thing" or sp.get("point_m") is None:
        return None
    got = _cloud(ex, seen)
    if got is None:
        return None
    view, P, valid, key = got
    p = np.asarray(sp["point_m"], dtype=float)
    x, y, depth = view.project(p)
    if not (depth > 0.0 and view.inside(x, y)):
        return None
    up, _, _ = see_spot.support_of(ex)
    hand = see_spot.hand_of(ex)
    sigma = float(e.get("sigma_m") or 0.0)
    comp = SC.component(P, valid, view, int(math.floor(x)), int(math.floor(y)), up, hand, sigma)
    if comp is None or not len(comp["xy"]):
        return None
    e1, e2 = SC.axes_of(np.asarray(up, dtype=float))
    return {"xy": np.asarray(comp["xy"], dtype=float), "z": np.asarray(comp["z"], dtype=float),
            "top": float(comp["top"]), "base": float(comp["base"]), "foot": float(comp["foot"]),
            "at": np.array([float(p @ e1), float(p @ e2)]), "sigma": sigma, "hand": hand, "look": key}


def rim(T, centre, reach=None):
    """The rim of a hollow thing nearest the pixel: {"kind": "rim", "centre_xy": the wall's middle there,
    "inward": the level unit direction from it to the thing's middle, "bearing_rad", "width_m": the wall's
    thickness along it, "top_z", "inside_m": how far under its top the look shows its inside (None: not seen),
    "lean": how far the wall's seen face moves along inward per unit of depth under the rim (0 for an upright
    wall or where the look shows no face of it there), "from_pixel_m"}; None when the thing's top is no ring
    about its seen middle (centre, level axes). reach: how far under the rim the pads come (m): centre_xy is the
    wall's middle at HALF that depth (a wall that leans stands further in or out there than at its
    rim: jaws centred on the rim would meet it with one pad alone)."""
    xy, z, top, foot, hand = T["xy"], T["z"], T["top"], T["foot"], T["hand"]
    margin = max(BAND * T["sigma"], foot)
    high = z >= top - margin
    if int(high.sum()) < SECTORS:
        return None
    c0 = np.asarray(centre, dtype=float)
    rel = xy[high] - c0
    dist = np.linalg.norm(rel, axis=1)
    quarter = (np.floor((np.arctan2(rel[:, 1], rel[:, 0]) + math.pi) / (math.pi / 2)).astype(int)) % SECTORS
    if len(set(quarter.tolist())) < SECTORS or not float(dist.min()) > hand["pad_m"]:
        return None                       # its top does not stand all around its middle, or stands at it: no ring
    inside = np.linalg.norm(xy - c0, axis=1) <= float(dist.min()) - foot
    low = inside & (z < top - margin)
    p = xy[high][int(np.argmin(np.linalg.norm(xy[high] - T["at"], axis=1)))]
    inward = c0 - p
    n = float(np.linalg.norm(inward))
    if not n > 0.0:
        return None
    inward = inward / n
    a = math.atan2(inward[1], inward[0])
    got = SC.chord(xy[high], z[high], p, np.array([a]), SC.MID * hand["pad_m"], BAND * foot, foot, T["sigma"])
    if got is None:
        return None
    mid = np.asarray(got["centre_xy"], dtype=float)
    lean = 0.0
    if reach is not None and float(reach) > margin:
        # the face of the wall the look shows under the rim, in the pad's strip along inward: where it stands
        # against the depth under the rim (a straight line through its points; under SECTORS points: none)
        rel = xy - mid
        along, across = rel @ inward, rel @ np.array([-inward[1], inward[0]])
        face = (~high) & (z >= top - float(reach)) & (np.abs(across) <= SC.MID * hand["pad_m"]) \
            & (np.abs(along) <= hand["pad_m"])
        if int(face.sum()) >= SECTORS and float(np.ptp(top - z[face])) > margin:
            lean = float(np.polyfit(top - z[face], along[face], 1)[0])
            shift = lean * SC.MID * float(reach)
            if abs(shift) <= hand["pad_m"]:         # a shift over a pad's width is no lean of one wall
                mid = mid + inward * shift
            else:
                lean = 0.0
    return {"kind": "rim", "centre_xy": [float(v) for v in mid], "inward": [float(v) for v in inward],
            "bearing_rad": a, "width_m": float(got["narrow_m"]), "top_z": top, "lean": lean,
            "inside_m": float(top - np.median(z[low])) if low.any() else None,
            "from_pixel_m": float(np.linalg.norm(mid - T["at"]))}


def chords(T):
    """The narrowest chords of the thing's footprint within the fingers' reach of its top, read through points one
    pad apart all over it: [{"kind": "chord", "centre_xy", "bearing_rad", "width_m", "top_z", "from_pixel_m"}],
    nearest the pixel first, one per place (chords whose middles lie within half a pad of a nearer one are left
    out)."""
    xy, z, top, foot, hand = T["xy"], T["z"], T["top"], T["foot"], T["hand"]
    slab = z >= top - hand["finger_m"]
    if int(slab.sum()) < SECTORS:
        return []
    pts, zs = xy[slab], z[slab]
    th = SC.bearings(hand, foot)
    pad = float(hand["pad_m"])
    cells = {}
    for i, key in enumerate(map(tuple, np.floor(pts / pad).astype(int))):
        cells.setdefault(key, []).append(i)
    out = []
    for idx in cells.values():
        at = pts[idx].mean(axis=0)
        got = SC.chord(pts, zs, at, th, SC.MID * pad, BAND * foot, foot, T["sigma"])
        if got is None:
            continue
        mid = np.asarray(got["centre_xy"], dtype=float)
        out.append({"kind": "chord", "centre_xy": [float(v) for v in mid], "bearing_rad": float(got["bearing_rad"]),
                    "width_m": float(got["narrow_m"]), "top_z": float(got["top_z"]),
                    "from_pixel_m": float(np.linalg.norm(mid - T["at"]))})
    out.sort(key=lambda w: (w["from_pixel_m"], w["width_m"]))
    kept = []
    for w in out:
        if all(np.linalg.norm(np.asarray(w["centre_xy"]) - np.asarray(k["centre_xy"])) > SC.MID * pad for k in kept):
            kept.append(w)
    return kept


def ways(ex, seen, centre=None, reach=None):
    """Every way the hand can take the thing the spot lies on, nearest the pixel first (module text): [] when the
    spot came with no look that gives depth or no thing is made out there. centre: the thing's seen middle on the
    level axes (default: the spot's own extent's). reach: how far under a rim the pads come (rim())."""
    try:
        T = thing(ex, seen)
    except Exception as exc:        # a look that cannot be read costs the take nothing: no way is added
        journal = getattr(ex, "_journal", None)
        if callable(journal):
            journal("see_problem", where="see_grasps", error=f"{type(exc).__name__}: {exc}")
        return []
    if T is None:
        return []
    if centre is None:
        centre = ((seen.get("spot") or {}).get("extent") or {}).get("centre_xy")
    out = chords(T)
    r = None if centre is None else rim(T, centre, reach)
    if r is not None:
        out.append(r)
    out.sort(key=lambda w: w["from_pixel_m"])
    for w in out:
        w["look"] = T["look"]
    return out
