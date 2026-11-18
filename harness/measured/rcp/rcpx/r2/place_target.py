# SPDX-License-Identifier: AGPL-3.0-only
"""The place targets of package place (N4), round six (the design notes, package PLACE; contracts C11, C12): where a
held thing goes, from a spot.

A place target comes from a spot (rcpx/r2/place_point.py: a pixel of a look, a located label, a point):
- into: the pointed ray comes in over a rim (the spot's entry_m): the held thing's middle goes over the entry
  point; the floor inside is not seen; rim_in_z is the height of the rim the ray came in over (the wall on the
  lens's side of the pointed point): what stands that high or higher bounds the opening the thing goes into;
- onto: the spot's own point: the top of the thing or the bare surface the ray lands on.
Every target carries what the carry must clear at the place (the top of what stands near the point) and the doubt
of its spot. A label that is only known from before something moved is acted on and said. Nothing here
moves, reads ground truth or names a robot.
"""

import numpy as np

from rcpx.context import refusal
from rcpx.r2.place_hand import finite

MM = 1000.0
LOOK_AGAIN = {"tool": "look", "args": {}, "periods": 0}


# ---------------------------------------------------------------- targets
def resolve_point(ex, value, what):
    """A base frame point (m) from [x, y, z] mm or a label or id robot://world holds: (point, label, None, doubt) or
    (None, None, refusal, None). A label known only from before something moved is taken where it was last
    known, and the doubt says so."""
    if isinstance(value, str):
        named = ex.world.get(value)              # a world id names its entry; else every entry with that label
        hits = [e for e in ([named] if named is not None else ex.world.query(label=value))
                if e.get("point_mm") is not None]
        fresh = [e for e in hits if not e.get("stale")]
        if not hits:
            known = sorted({e["label"] for e in ex.world.query() if e.get("point_mm") is not None})
            return None, None, refusal(what, f"robot://world holds no point labelled {value!r}",
                                       "point at it with {look, px} in a look you were shown, or give [x, y, z] mm"
                                       + (f"; known labels: {', '.join(known[:12])}" if known else "")), None
        e = (fresh or hits)[-1]
        doubt = None
        if not fresh:
            doubt = {"what": f"{value!r} is only known from before something moved"
                             + (f" ({(e.get('data') or {}).get('stale_why')})"
                                if (e.get("data") or {}).get("stale_why") else ""),
                     "acted": "went to where it was last known", "would_remove": LOOK_AGAIN}
        return np.asarray(e["point_mm"], dtype=float) / MM, value, None, doubt
    try:
        v = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        v = np.zeros(0)
    if v.size != 3 or not np.all(np.isfinite(v)):
        return None, None, refusal(what, f"{what} must be {{look, px}}, a located label or [x, y, z] in base "
                                   f"frame mm, got {value!r}", "for example {\"look\": \"C1\", \"px\": [250, 301]}"), None
    return v / MM, None, None, None


def target_from_seen(seen, role):
    """A place target from a Seen (place_point.resolve, role place). Into: the spot has an entry point (the
    pointed ray comes in over a rim, or lands between two walls): the held thing's middle goes over it, the rim
    is the highest of what stands near the point and of what the ray came in over, the floor inside is not seen.
    Onto: the spot's own point, on a thing or on the bare surface."""
    sp = seen.get("spot") or {}
    e = sp.get("extent") or {}
    xy = seen.get("pointed_xy") or seen.get("centre_xy")
    point = sp.get("point_m")
    z = float(point[2]) if point is not None and seen.get("kind") == "thing" else float(seen["top_z"])
    if seen.get("kind") != "thing":
        z = float(seen["top_z"])          # a bare spot: the higher of what is seen and the stated support
    near = [float(v) for v in (e.get("top_near_z"), z) if finite(v)]
    out = {"centre": np.array([float(xy[0]), float(xy[1]), z]), "surface_z": z, "kind": "on", "size_m": None,
           "labels": [seen.get("label")], "seen": seen, "sigma_z": seen.get("sigma_z"),
           "sigma_xy": seen.get("sigma_xy"), "ref": seen.get("id"), "look": (seen.get("looks") or [None])[0],
           "top_near_z": max(near), "top_z": float(e["top_z"]) if finite(e.get("top_z")) else max(near),
           "doubt": [{"what": str(d), "acted": "went there with the contact stop armed",
                      "would_remove": sp.get("remove_by")} for d in (seen.get("doubt") or [])]}
    entry = sp.get("entry_m")
    if entry is not None or role == "into":
        slot = e.get("slot") or {}
        rim = [float(v) for v in (e.get("top_near_z"), e.get("way_in_z"), slot.get("rim_z")) if finite(v)]
        c = out["centre"] if entry is None else np.asarray(entry, dtype=float)
        came = e.get("way_in_z") if finite(e.get("way_in_z")) else slot.get("rim_z")
        out.update(kind="into", centre=np.array([float(c[0]), float(c[1]), z]), surface_z=None, seen_z=z,
                   rim_z=max(rim) if rim else z, top_near_z=max(rim + near),
                   rim_in_z=float(came) if finite(came) else None)
    return out


def resolve_target(ex, on=None, corners=None, rim=None, into=None, to=None, resolve=None):
    """{"centre": m, "surface_z": m or None, "kind": on | corners | rim | into, "top_near_z", "labels", "doubt",
    ...} or a refusal. resolve(value, role) turns one spot into (Seen or None, point m, label, refusal, doubt):
    place.Mixin._pl_spot."""
    given = [k for k, v in (("on", on), ("in_corners", corners), ("in_rim", rim), ("in", into), ("to", to))
             if v is not None]
    if len(given) != 1:
        return refusal("target", "say where to set it down: exactly one of to, on or in (the older on_point_mm, "
                       "on_label, in_corners and in_rim are still read)" +
                       (f" (got {', '.join(given)})" if given else ""),
                       "to {look, px}: a pixel on the spot, on the thing it goes on, or in the opening it goes into")
    single = on if on is not None else (to if to is not None else (into if not isinstance(into, (list, tuple))
                                                                  or _is_point(into) else None))
    if single is not None:
        role = "into" if into is not None else "place"
        seen, p, label, bad, doubt = resolve(single, role)
        if bad:
            return bad
        if seen is not None:
            t = target_from_seen(seen, "on" if on is not None else role)
            t["labels"] = [label]
        else:
            z = float(p[2])
            t = {"centre": np.asarray(p, dtype=float), "surface_z": z, "kind": "on", "size_m": None,
                 "labels": [label], "ref": label if label else [float(x) for x in p], "top_near_z": z, "top_z": z,
                 "doubt": []}
        if doubt:
            t["doubt"].append(doubt)
        return t
    items = corners if corners is not None else (rim if rim is not None else into)
    what = "in_corners" if corners is not None else ("in_rim" if rim is not None else "in")
    if not isinstance(items, (list, tuple)) or not items or (corners is not None and len(items) != 2):
        return refusal(what, f"{what} needs " + ("two opposite corners" if corners is not None else
                       "at least one spot on the rim of the opening") + f", got {items!r}",
                       "each a spot: {look, px}, a label robot://world holds or [x, y, z] mm")
    pts, labels, doubts = [], [], []
    for it in items:
        seen, p, label, bad, doubt = resolve(it, "place")
        if bad:
            return bad
        if seen is not None:
            xy = seen.get("pointed_xy") or seen.get("centre_xy")
            p = np.array([float(xy[0]), float(xy[1]), float(seen["top_z"])])
        pts.append(p)
        labels.append(label)
        if doubt:
            doubts.append(doubt)
    P = np.asarray(pts)
    c = (P.max(axis=0) + P.min(axis=0)) / 2 if len(P) > 1 else P.mean(axis=0)   # the middle of what the points span
    top = float(P[:, 2].max())
    if corners is not None:        # two opposite corners of a flat place: onto its middle, at the corners' height
        z = float(P[:, 2].mean())
        return {"centre": np.array([c[0], c[1], z]), "surface_z": z, "kind": "corners", "labels": labels,
                "size_m": (abs(float(P[0, 0] - P[1, 0])), abs(float(P[0, 1] - P[1, 1]))),
                "ref": [float(x) for x in c], "top_near_z": top, "top_z": top, "doubt": doubts}
    # spots on the rim of an opening: into the middle of what they span; the floor inside is not seen
    return {"centre": np.array([c[0], c[1], top]), "surface_z": None, "kind": "rim", "rim_z": top, "size_m": None,
            "opening": [[float(p[0]), float(p[1])] for p in P], "labels": labels,
            "ref": [float(c[0]), float(c[1]), top], "top_near_z": top, "top_z": top, "doubt": doubts}


def _is_point(v):
    return isinstance(v, (list, tuple)) and len(v) == 3 and all(finite(x) for x in v)
