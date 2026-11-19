# SPDX-License-Identifier: AGPL-3.0-only
"""Package place_roof: a cover is also what
the LOOKS show over a place. No tools of its own; rcpx/r2/place_side.py (the side entry) calls it. Nothing here moves.

A place stands under a roof when points of a look's cloud lie inside the held thing's column over the place,
between the top of the thing once it stands there and the height the carry would come in at: the lowest of them
is the roof (rcpx/r2/see_place.roof). Every look with depth this session holds is read (the robot's own body is
left out by the look itself, and so is what stood where the taken thing stood). No name of a fixture and no
height is typed: the column is the held thing's own half width and the spot's error, the heights are the place's
level, the thing's own height and the carry's.

The open side is looked for by rcpx/r2/place_side.py as for a cover the space book holds, with what the looks
show in place of the book: the hand can come down where no look point stands in its column (the mouth), and the
level way in is free where no look point lies inside what the hand and the thing sweep from the mouth to over
the place. What blocks each bearing tried is kept, so that a result can say it.

    points(ex)                                [(look id, n x 3 points)] of the looks with depth, newest first
    roof_over(ex, centre, half_m, low_z, high_z, floor_z)   the roof the looks show over a place, or None
    under_cover(looks, centre, half_m, low_z, high_z, floor_z)   the cells where the looks show the place's own
                                              level under what covers it (room under a roof, seen)
    column(ex, xy, radius_m, low_z, high_z)   what the looks show in a column (the mouth), or None
    swept(ex, p_mouth, p_in, offsets, grow_m) what the looks show inside the swept way in, or None
    arm_in_looks(ex, postures)                whether the arm's own body, at the way's postures, stands in a look's points
    hand_over_m(ex)                           how far the hand's own hull reaches behind the tip
The take is not changed for a roof (a take from the side so that the hand goes in level was tried and taken out:
a handle that arches over a container's opening is a roof by the same reading, and the side take then lost
takes that held).
Nothing here reads truth or names a robot or a thing.
"""

import numpy as np

from rcpx.bounds_ratios import BAND, QUANT
from rcpx.r2 import see_place, see_spot

MM = 1000.0          # unit


def points(ex):
    """[(look id, points n x 3 base frame, the depth's error at their middle range)] of every look with depth this
    session holds, the newest first; what stood where the taken thing stood is left out."""
    return [(key, pts, sigma) for key, pts, sigma, _ in _looks(ex)]


def _looks(ex):
    """points() with each look's footprint of one pixel at that middle range (m) beside it."""
    looks = [(k, r) for k, r in ((ex.state.get("scene_camera") or {}).get("looks") or {}).items()]
    looks += [(k, r) for k, r in (getattr(ex, "_looks", None) or {}).items()]
    skip = ex._pl_skip() if hasattr(ex, "_pl_skip") else None
    out = []
    for key, rec in sorted(looks, key=lambda x: -float(x[1].get("t") or 0.0)):
        try:
            got = see_spot.cloud_of(ex, rec)
        except Exception:
            got = None
        if got is None or not got[2].any():
            continue
        pts = got[1][got[2]]
        if skip is not None:
            pts = pts[np.linalg.norm(pts[:, :2] - np.asarray(skip[0], dtype=float), axis=1) > skip[1]]
        if not len(pts):
            continue
        rng = float(np.median(np.linalg.norm(pts - got[0].pos, axis=1)))
        foot = rng / float(got[0].f)
        out.append((str(key), pts, float(see_spot._sigma_at(rec, rng, foot)[0]), foot))
    return out


def under_cover(looks, centre, half_m, low_z, high_z, floor_z):
    """How many cells of the column the looks show the place's OWN level in, under a point they show over low_z,
    with nothing they show standing between (m: the room under a cover, seen free down to the place's level). A
    roof over a place has room under it, and a look that sees in under its edge sees the place's level there; the
    top of a thing that stands at the place hides the level under it from every look. The cell is QUANT of the
    coarsest look's footprints (a level surface seen at a slant has no cell without a point); each look's points
    within BAND of its depth error of the level are the level. Only cells whose eight neighbours are such cells
    too count: along the edge of a thing's top a cell holds that top and the level seen beside it (the level is
    beside the top there, not under it). looks: _looks(ex)."""
    c = np.asarray(centre, dtype=float)[:2]
    if not looks or not half_m > 0:
        return 0
    step = max(QUANT * foot for _, _, _, foot in looks)
    n = max(1, int(np.ceil((half_m + half_m) / step)))
    cover = np.zeros((n, n), dtype=bool)
    stand, level = cover.copy(), cover.copy()
    for _, pts, sigma, _ in looks:
        near = np.linalg.norm(pts[:, :2] - c, axis=1) <= half_m
        P = pts[near]
        if not len(P):
            continue
        ij = np.clip(np.floor((P[:, :2] - c + half_m) / step).astype(int), 0, n - 1)
        z, band = P[:, 2], BAND * sigma
        for m, sel in ((cover, (z > float(low_z) + band) & (z < high_z)),
                       (stand, (z > float(floor_z) + band) & (z <= float(low_z) + band)),
                       (level, np.abs(z - float(floor_z)) <= band)):
            m[ij[sel, 1], ij[sel, 0]] = True
    import cv2
    under = (cover & level & ~stand).astype(np.uint8)
    return int((cv2.erode(under, see_place.RING, borderType=cv2.BORDER_CONSTANT, borderValue=0) > 0).sum())


def roof_over(ex, centre, half_m, low_z, high_z, floor_z=None):
    """The roof the looks show over a place: {"z", "room_m": from low_z up to it, "look", "points", "covers": the
    share of the column's cells it stands over (a bar or a handle that crosses a place covers a strip of it, a
    roof covers it), "stands": how many points of the looks lie in the same column UNDER low_z, more than BAND of
    their depth error over floor_z (something stands at the place itself, from its level up), "under": how many
    cells of the column the looks show the place's own level in under what covers it (under_cover: room under a
    roof; the top of a thing standing at the place hides the level under it), "text"} or None. centre: the place
    (base frame m); half_m: the column's half width; low_z: the top of the thing once it stands there; high_z:
    the height the carry would come in at; floor_z: the level the thing stands on there. A look's points within
    BAND of its depth error over low_z are the place's own surroundings, not a roof."""
    best = None
    c = np.asarray(centre, dtype=float)[:2]
    stands = 0
    looks = _looks(ex)
    for key, pts, sigma, _ in looks:
        if floor_z is not None:
            z = pts[:, 2]
            stands += int(((np.linalg.norm(pts[:, :2] - c, axis=1) <= half_m) & (z > float(floor_z) + BAND * sigma)
                           & (z < float(low_z))).sum())
        got = see_place.roof(pts, c, half_m, float(low_z) + BAND * sigma, high_z)
        if got is not None and (best is None or got["z"] < best["z"]):
            over = pts[(np.linalg.norm(pts[:, :2] - c, axis=1) <= half_m) & (pts[:, 2] > float(low_z) + BAND * sigma)
                       & (pts[:, 2] < high_z)]
            n = QUANT + QUANT + 1             # the column as a grid of that many cells across: how much of it
            ij = np.clip(np.floor((over[:, :2] - c + half_m) / (half_m + half_m) * n).astype(int), 0, n - 1)
            g = (np.arange(n) + see_place.HALF) / n * (half_m + half_m) - half_m
            disc = np.hypot(*np.meshgrid(g, g)) <= half_m          # the roof covers
            filled = np.zeros((n, n), dtype=bool)
            filled[ij[:, 1], ij[:, 0]] = True
            best = dict(got, look=key, covers=float((filled & disc).sum()) / float(max(1, disc.sum())))
    if best is None:
        return None
    best["stands"] = stands
    best["under"] = 0 if floor_z is None else under_cover(looks, c, half_m, low_z, high_z, floor_z)
    best["room_m"] = best["z"] - float(low_z)
    best["text"] = (f"what look {best['look']} shows over the place ({best['points']} points of its depth picture in "
                    f"the held thing's column, the lowest {best['room_m'] * MM:.0f} mm over the thing's top once it "
                    "stands there)")
    return best


def column(ex, xy, radius_m, low_z, high_z):
    """What the looks show in the column of radius_m over level point xy between two heights: {"look", "z": the
    lowest, "points"} or None (nothing: the hand can come down there)."""
    for key, pts, sigma in points(ex):
        got = see_place.roof(pts, np.asarray(xy, dtype=float)[:2], radius_m, float(low_z) + BAND * sigma, high_z)
        if got is not None:
            return dict(got, look=key)
    return None


def swept(ex, p_mouth, p_in, offsets, grow_m=0.0):
    """What the looks show inside what the hand and the held thing sweep on the level line from p_mouth to p_in:
    {"look", "at_m": how far along the way in the first point lies, "over_m": how high over the swept bottom,
    "points"} or None. offsets: k x 3, every corner that moves with the hand relative to the tip (base frame).
    The swept box runs from the corners' own back to their front along the way, as wide as they are across it,
    between their lowest and highest point; a look's points within BAND of its depth error of its bottom
    or top are the surface the thing goes over and the roof it goes under."""
    a, b = np.asarray(p_mouth, dtype=float), np.asarray(p_in, dtype=float)
    length = float(np.linalg.norm((b - a)[:2]))
    if length <= 0.0:
        return None
    e = np.array([(b - a)[0], (b - a)[1], 0.0]) / length
    n = np.array([-e[1], e[0], 0.0])
    W = np.asarray(offsets, dtype=float).reshape(-1, 3)
    along, across, tall = W @ e, W @ n, W[:, 2]
    for key, pts, sigma in points(ex):
        rel = pts - a
        band = BAND * sigma
        s = rel @ e
        inside = (s >= float(along.min())) & (s <= length + float(along.max())) & \
            (rel @ n >= float(across.min()) - grow_m) & (rel @ n <= float(across.max()) + grow_m) & \
            (rel[:, 2] > float(tall.min()) + band) & (rel[:, 2] < float(tall.max()) - band)
        if inside.any():
            k = int(np.argmin(np.where(inside, s, np.inf)))
            return {"look": key, "at_m": float(s[k]), "over_m": float(rel[k, 2] - tall.min()),
                    "points": int(inside.sum())}
    return None


def arm_in_looks(ex, postures):
    """Whether the robot's own body, at one of the joint postures of a planned way, would stand in what a look
    shows: {"look", "points", "at": the index of the posture} or None. The body's own hulls at that posture
    (see_spot._bodies, the hulls a look leaves out as the robot's own) against every look's points, within BAND
    of the look's depth error: a 3D test, as see_cloud.inside_hulls makes it."""
    from rcpx.r2 import see_cloud
    for i, q in enumerate(postures):
        bodies = see_spot._bodies(ex, {"q": np.asarray(q, dtype=float), "jaws": None})
        for key, pts, sigma in points(ex):
            inside = see_cloud.inside_hulls(pts, bodies, BAND * sigma)
            if inside.any():
                return {"look": key, "points": int(inside.sum()), "at": i}
    return None


def hand_over_m(ex):
    """How far the hand's own hull reaches behind the tip along its pointing axis (m): what stands over a thing
    held from above."""
    from rcpx.r2.place_side import hand_points
    pts = hand_points(ex)
    return float(-pts[:, 0].min()) if len(pts) else 0.0
