# SPDX-License-Identifier: AGPL-3.0-only
"""Where the jaws go, of package place: from the spot a
pick was pointed at and the hand's own shape, never from a list of things, and never a refusal because something is
unsure. A measured thing makes the plan better; it is not the gate: every plan that a better look gives and this
arm cannot take falls back to the plan of the point itself.

choose(ex, seen, hints) returns one plan {strategy, yaw, open_m, tip_z, depth_m, centre, width_m, q_low, R, why,
doubt, ...} or a refusal of a class that stays (a malformed spot, a width stated or measured whole over the
opening with no wall and no part to take, no arm pose, a hand that cannot come down to the thing past what the
spot's own look shows standing higher in its way). The order of the rules:

1. top: the whole thing at its seen middle, jaws across its narrow side (or its long side when that fits and
   turns the hand less), only when the narrow side plus twice its error plus twice the hand's own band at rest is
   under the opening and the pointed spot lies within one finger depth of the thing's top.
2. rim: the thing is hollow at the pointed spot (higher points of it surround the point): its wall nearest the
   pointed ray, the jaws along the wall's inward direction and centred on it, one finger inside, the pads' own
   height under the rim and no deeper than the inside is seen where the pointed ray landed.
3. part: the narrowest chord of the thing through the pointed spot, jaws across it, the fingers' depth under the
   part's own top.
4. point: nothing is made out there (or no better plan has an arm pose): the open jaws at the point; with
   nothing seen of the thing the point is taken as its top and the fingers come half their length under it, no
   deeper than half its height over the stated support.
side (on the operator's word, or when no pose from above exists): level, around the narrow side at half height.

The jaws open to the chosen width plus twice its error plus twice what the hand can be off its path at the speed
it comes down with (the bounds' cross-track error), else as far as they go; with no error stated for where the
thing stands (a typed point) they open as far as they go whatever width is stated. The depth is the fingers' own,
never under the support less the tip's own resolution. What stands beside the thing bounds nothing here: of the
bearings a plan may take, one whose hand overlaps a MEASURED neighbour is taken last, and said. Where something
measured stands in the hand's way around the thing's middle, the jaws opened only to its width, its error and the
hand's band are tried, then the hand off its middle along the closing axis while the thing still stands between
the open jaws; blocked is refused only when none is free. A hand with one moving jaw and no width to go by takes
a thing at the point between its pads (place_pick_count.pads_mid_T); a width that was stated and not seen
does not say across which bearing the thing is that wide: its middle then goes to that same point, by the hand's
own rule (Jaws.hold_centre_T) with the fixed pad as far off its face as that takes (plan "follow": the hand
moves with its jaw as it closes).

Lengths are m, angles rad, the base frame (up is the base z). Every length comes from the hand's model
(place_hand.Jaws), the Seen and the bounds; the multiples are the ratios of rcpx/bounds_ratios.py.
"""

import math

import numpy as np

from rcpx.context import refusal
from rcpx.r2.place_hand import hand_rects, poly_gap, rect

MM = 1000.0      # unit
HALF = 0.5       # numeric: the middle of a width
EPS = 1e-9       # numeric
DOWN = np.array([0.0, 0.0, -1.0])
UP = -DOWN
LOOK = {"tool": "look", "args": {"camera": "wrist"}, "periods": 0}   # the one ready call that removes a doubt of seeing


def _ratios():
    from rcpx import bounds_ratios as R
    return R


def _bounds(ex):
    from rcpx import bounds
    return bounds.of(ex)


def num(x, default=None):
    """A bound's value: a bare number, (value, item), or {"value": ...}; default when there is none."""
    if isinstance(x, (tuple, list)) and x:
        x = x[0]
    if isinstance(x, dict):
        x = x.get("value")
    try:
        x = float(x)
    except (TypeError, ValueError):
        return default
    return x if math.isfinite(x) else default


def line(what, acted, would_remove=None):
    """One doubt line of a receipt: what was unsure with its number and source, what the tool did about it,
    and the ONE ready call that would remove it (or None)."""
    return {"what": str(what), "acted": str(acted), "would_remove": would_remove}


# ---------------------------------------------------------------------------------------------- reading a Seen
def _outline(o):
    """(centre xy, narrow, wide, bearing) of an outline given as a dict or a sequence, or None."""
    if o is None:
        return None
    if isinstance(o, dict):
        c = o.get("centre", o.get("centre_m", o.get("centre_xy")))
        n, w = o.get("narrow", o.get("narrow_m")), o.get("wide", o.get("wide_m"))
        b = o.get("bearing", o.get("bearing_rad"))
    else:
        c, n, w, b = (list(o) + [None] * 4)[:4]
    if n is None or w is None or b is None:
        return None
    c = None if c is None else np.asarray(c, dtype=float).reshape(-1)[:2]
    return c, float(n), float(w), float(b)


def read_seen(seen):
    """A Seen (place_point.resolve, with the spot of C11 under "spot") or a minimal one written by hand, as the
    fields this file uses. whole: the outline was seen whole (not cut by a picture's edge, not running on past
    what the hand spans); wall_at: the wall's own point and inward direction when the spot gives them."""
    o = _outline(seen.get("outline"))
    c = seen.get("centre_xy")
    c = (o[0] if o is not None and o[0] is not None else None) if c is None else np.asarray(c, dtype=float)[:2]
    pointed = seen.get("pointed_xy")
    pointed = None if pointed is None else np.asarray(pointed, dtype=float).reshape(-1)[:2]
    if c is None:
        c = pointed
    part = seen.get("part_chord")
    if isinstance(part, dict) and part.get("width") is not None and part.get("bearing") is not None:
        part = {"width": float(part["width"]), "bearing": float(part["bearing"]),
                "top_z": None if part.get("top_z") is None else float(part["top_z"]),
                "centre": None if part.get("centre") is None else np.asarray(part["centre"], dtype=float)[:2]}
    else:
        part = None
    sp = seen.get("spot") if isinstance(seen.get("spot"), dict) else {}
    e = sp.get("extent") if isinstance(sp.get("extent"), dict) and sp["extent"].get("kind") == "thing" else {}
    w = e.get("wall") or {}
    wall_at = None
    if w.get("point_m") is not None and w.get("inward") is not None:
        wall_at = {"point": np.asarray(w["point_m"], dtype=float)[:2],
                   "inward": np.asarray(w["inward"], dtype=float)[:2]}
    sz = seen.get("sigma_z")
    point = sp.get("point_m")
    return {"id": seen.get("id"), "kind": seen.get("kind", "thing"), "centre": c,
            "top_z": None if seen.get("top_z") is None else float(seen["top_z"]),
            "point_z": float(point[-1]) if point is not None and o is not None else (
                None if seen.get("top_z") is None else float(seen["top_z"])),
            "sigma_xy": float(seen.get("sigma_xy") or 0.0), "sigma_z": None if sz is None else float(sz),
            "placed": seen.get("sigma_xy") is not None,      # an error is stated for where it stands
            "narrow": None if o is None else o[1], "wide": None if o is None else o[2],
            "bearing": None if o is None else o[3],
            "height": None if seen.get("height") is None else float(seen["height"]),
            "hollow": seen.get("hollow") if seen.get("hollow") in (True, False) else None,
            "depth_inside": None if seen.get("depth_inside") is None else float(seen["depth_inside"]),
            "wall": None if seen.get("wall") is None else float(seen["wall"]), "wall_at": wall_at,
            "top_near": None if e.get("top_near_z") is None else float(e["top_near_z"]),
            "whole": not (e.get("cut") or e.get("border") or seen.get("cut")), "part": part, "pointed": pointed,
            "doubt": [str(d) for d in seen.get("doubt") or []], "remove_by": sp.get("remove_by"),
            "source": seen.get("source"),
            "neighbours": [n for n in (_neighbour(x) for x in seen.get("neighbours") or []) if n is not None]}


def _neighbour(nb):
    """A MEASURED neighbour as a rectangle on the support: {id, centre, half (along its bearing, across), bearing,
    top_z, sigma_xy}, or None when it has no outline."""
    if not isinstance(nb, dict):
        return None
    o = _outline(nb.get("outline"))
    if o is None and nb.get("lo") is not None and nb.get("hi") is not None:      # an axis-aligned box
        lo, hi = np.asarray(nb["lo"], dtype=float), np.asarray(nb["hi"], dtype=float)
        o = (HALF * (lo[:2] + hi[:2]), float(hi[0] - lo[0]), float(hi[1] - lo[1]), 0.0)
        nb = dict(nb, top_z=nb.get("top_z", float(hi[-1]) if hi.size > len(o[0]) else None))
    if o is None:
        return None
    c = o[0] if o[0] is not None else nb.get("centre_xy")
    if c is None:
        return None
    return {"id": nb.get("id"), "centre": np.asarray(c, dtype=float)[:2], "half": (HALF * o[1], HALF * o[2]),
            "bearing": o[3], "top_z": None if nb.get("top_z") is None else float(nb["top_z"]),
            "sigma_xy": float(nb.get("sigma_xy") or 0.0)}


def extent_along(narrow, wide, outline_bearing, bearing):
    """A rectangle's extent along a horizontal bearing."""
    d = float(bearing) - float(outline_bearing)
    return narrow * abs(math.cos(d)) + wide * abs(math.sin(d))


def centre_T(J, frac, obj_w=None):
    """Where a thing's middle stands in the tool frame, the jaws open at frac (place_pick_count.pads_mid_T)."""
    from rcpx.r2 import place_pick_count as one       # that file imports this one
    return one.pads_mid_T(J, frac) if J.one_jaw and obj_w is None else J.hold_centre_T(frac, obj_w)


def clearance(J, S, centre, yaw, open_m, tip_z, obj_w=None):
    """The least gap (m) between the hand's boxes and every measured neighbour that stands as high as that part
    of the hand comes: (gap, neighbour or None, the body's word)."""
    best = (math.inf, None, None)
    if J.one_jaw and obj_w is None and S["neighbours"]:     # hand_rects stands the hand by Jaws.hold_centre_T
        d = J.hold_centre_T(J.frac_for_width(open_m), obj_w) - centre_T(J, J.frac_for_width(open_m), obj_w)
        u, a, b = np.array([math.cos(yaw), math.sin(yaw)]), float(d @ J.axis_T), float(d @ J.cross_T)
        centre = np.asarray(centre, dtype=float)[:2] + a * u + b * np.array([-u[1], u[0]])
    for word, corners, x in hand_rects(J, centre, yaw, open_m, obj_w):
        low_z = tip_z - x[1]                              # the lowest point of that box (the approach is down)
        for nb in S["neighbours"]:
            if nb["top_z"] is not None and nb["top_z"] < low_z:
                continue
            g = poly_gap(corners, rect(nb["centre"], nb["half"][0], nb["half"][1], nb["bearing"]))
            if g < best[0]:
                best = (g, nb, word)
    return best


def seen_points(ex, seen, at, reach):
    """The points the spot's own look shows within `reach` (m, on the level) of the spot: N x 3 in the base
    frame, the robot's own body left out; None when the spot came with no depth picture. What they show is
    MEASURED: it decides which of the bearings and slides a plan takes, never whether the tool acts."""
    sp = seen.get("spot") if isinstance(seen.get("spot"), dict) else {}
    try:
        from rcpx.r2 import see_spot
        for key in sp.get("looks") or []:
            rec = see_spot.look_of(ex, key)[1]
            got = None if rec is None else see_spot.cloud_of(ex, rec)
            if got is not None:
                pts = got[1][got[2]]
                return pts[np.linalg.norm(pts[:, :2] - np.asarray(at, dtype=float)[:2], axis=1) <= reach]
    except Exception:           # a build with no seeing package, or a look that cannot be read: nothing is measured
        return None
    return None


def blocked(J, pts, centre, yaw, open_m, tip_z, obj_w, margin):
    """(how many seen points stand inside one of the hand's boxes and over the lowest point of that box by more than
    margin, how far the highest of them stands over it m, the body's word) for the jaws open open_m around
    centre with the closing axis along yaw and the tip at tip_z; (0, 0.0, None) with nothing seen."""
    worst = (0, 0.0, None)
    if pts is None or not len(pts):
        return worst
    frac = J.frac_for_width(open_m)
    c_T = centre_T(J, frac, obj_w)
    ca, cb = float(c_T @ J.axis_T), float(c_T @ J.cross_T)
    u = np.array([math.cos(yaw), math.sin(yaw)])
    d = pts[:, :2] - np.asarray(centre, dtype=float)[:2]
    da, db = d @ u, d @ np.array([-u[1], u[0]])
    for word, b in [("finger", x) for x in J.finger_boxes(frac)] + [("palm", J.palm_box())]:
        if b is None:
            continue
        inside = (da >= b["a"][0] - ca) & (da <= b["a"][1] - ca) & (db >= b["b"][0] - cb) & (db <= b["b"][1] - cb)
        over = pts[inside][:, -1] - (tip_z - b["x"][1]) - margin
        n = int((over > 0.0).sum())
        if n > worst[0]:
            worst = (n, float(over.max()) + margin, word)
    return worst


def support_at(B, xy):
    """(height of the support plane under xy, its sigma or None, its source) from the space book."""
    got = B.space.support()
    if not got or got[0] is None:
        return None, None, None
    plane, sigma, *rest = got
    n, off = plane
    source = rest[0] if rest else None
    n = np.asarray(n, dtype=float)
    z = (float(off) - float(n[0]) * float(xy[0]) - float(n[1]) * float(xy[1])) / (float(n[-1]) if abs(n[-1]) > EPS
                                                                                  else 1.0)
    return z, (None if sigma is None else float(sigma)), source


def _refuse(check, why, hint, **more):
    out = refusal(check, why, hint)
    out["refused"].update(more)
    return out


# ---------------------------------------------------------------------------------------------- the rules
def depth_rule(J, top_z, sup_z, frac, rigid, cap=None):
    """(tip height, depth under the top, what the rule was) for jaws that come down around something whose top
    is at top_z: the fingers' own depth (cap = (depth, its words): no deeper than that), and the fingertips'
    lowest point never under the stated support plus the tip's own resolution, WHEREVER the top is said to be (a
    top at or under that line gives depth 0: the fingers stop over the support and go around nothing)."""
    finger = J.finger_len_m or J.pad_height_m or 0.0
    d, why = finger, "the fingers' own depth under its top"
    if cap is not None and cap[0] < d:
        d, why = max(0.0, float(cap[0])), cap[1]
    tip = top_z - d
    if sup_z is not None and tip < sup_z + J.low_m(frac) + rigid:
        tip = sup_z + J.low_m(frac) + rigid
        why = "a low thing: the fingertips stop the tip's own resolution over the support"
    return tip, max(0.0, top_z - tip), why


def _wall_of(S, at):
    """(the wall's point xy, the level unit direction from it into the thing, its thickness or None): the spot's
    own wall; else, from an outline alone (a Seen written by hand, or the operator's word grasp rim), the edge of
    the outline nearest the pointed spot, toward the thing's middle. None when nothing gives a wall."""
    if S["wall_at"] is not None:
        n = float(np.linalg.norm(S["wall_at"]["inward"]))
        if n > EPS:
            return S["wall_at"]["point"], S["wall_at"]["inward"] / n, S["wall"]
    if S["narrow"] is None:
        return None
    c0 = np.asarray(S["centre"], dtype=float)
    d = np.asarray(at, dtype=float) - c0
    if float(np.linalg.norm(d)) <= EPS:
        d = -c0                                           # pointed at its middle: the wall on the robot's side
    if float(np.linalg.norm(d)) <= EPS:
        return None
    u = d / float(np.linalg.norm(d))
    ext = extent_along(S["narrow"], S["wide"], S["bearing"], math.atan2(u[1], u[0]))
    thick = S["wall"]
    return c0 + (HALF * ext - HALF * (thick or 0.0)) * u, -u, thick


def _pose(ex, J, centre, tip_z, yaw, t, open_m, obj_w, seeds):
    """Joints for the jaws open open_m around centre (xy) with the tip at tip_z and the closing axis along yaw:
    (q, R reached, yaw reached, squared) or (None, ...); one moving jaw, no width: place_pick_count.one_jaw_pose."""
    frac = J.frac_for_width(open_m)
    R0 = ex._pl_rot(t, yaw)
    c_T = centre_T(J, frac, obj_w)
    p = np.array([float(centre[0]), float(centre[1]), float(tip_z) + float((R0 @ c_T)[-1])])
    if J.one_jaw and obj_w is None:
        from rcpx.r2.place_pick_count import one_jaw_pose
        return one_jaw_pose(ex, p, c_T, t, R0, yaw, seeds)
    return ex._pl_best_pose(p, t, [yaw], frac, seeds, obj_w)


def _take(ex, J, S, F, cand):
    """The plan of one candidate (strategy, words, centre, [(yaw, width or None)], top, depth cap), or None when
    this arm has no pose for any of its bearings. The bearings (for a whole thing also slides along its other
    side, half a pad at a time while the pads' middle stays on it) are tried in this order: first those where nothing
    MEASURED stands in the hand's way (the points the spot's look shows, a neighbour a caller measured), then
    the least slide, then the least turn of the hand from where it stands. With F["tight"] a bearing whose hand
    is not free around the thing's middle is also tried with the jaws opened only to its width, its error and the
    hand's band, and (jaws that move together) off its middle along the closing axis by half and all of the room."""
    R = _ratios()
    strategy, words, centre, options, top, cap = cand
    pad = _finger_across(J)
    opts = []
    for yaw, w in options:
        # a point that came with no error has none to add to a width: nothing bounds where its far face stands,
        # so the jaws open as far as they go (the width still says where its middle stands between them)
        wide_open = w is None or not S["placed"]
        open_m = J.max_m if wide_open else J.open_for_reach(w + F["err"] + F["err"] + F["off"] + F["off"])  # shapes
        spots = [(0.0, 0.0, open_m)]                    # (slide along its other side, shift along the closing
        if strategy == "top" and S["narrow"] is not None and pad > 0.0:                    # axis, the opening)
            other = extent_along(S["narrow"], S["wide"], S["bearing"], yaw + HALF * math.pi)
            room = HALF * other - F["err"]              # the pads' middle stays on the thing
            k = 1
            while k * HALF * pad <= room:
                spots += [(k * HALF * pad, 0.0, open_m), (-k * HALF * pad, 0.0, open_m)]
                k += 1
        turn = abs((yaw - F["hand_yaw"] + math.pi) % (math.pi + math.pi) - math.pi)
        u = np.array([math.cos(yaw), math.sin(yaw)])
        k = 0
        while k < len(spots):
            slide, shift, o_m = spots[k]
            tip_z, depth, why = depth_rule(J, top, F["sup_z"], J.frac_for_width(o_m), F["rigid"],
                                           cap(o_m) if cap else None)
            c = np.asarray(centre, dtype=float)[:2] + slide * np.array([-u[1], u[0]]) + shift * u
            n, over, word = blocked(J, F["pts"], c, yaw, o_m, tip_z, w, F["margin"])
            gap, nb, nb_word = clearance(J, S, c, yaw, o_m, tip_z, w)
            opts.append(((n >= R.QUANT or gap <= 0.0, abs(slide) + abs(shift), turn), yaw, w, c, slide, shift, o_m,
                         tip_z, depth, why, (n, over, word), (gap, nb, nb_word)))
            if k == 0 and w is not None and F.get("tight") and opts[-1][0][0]:     # not free around its middle
                tight = min(J.max_m, w + F["err"] + F["err"] + F["band"] + F["band"])
                room = 0.0 if J.one_jaw else HALF * (J.max_m - w) - F["err"] - F["band"]
                spots += [(0.0, 0.0, tight)] if tight < o_m else []
                spots += [(0.0, s * f * room, J.max_m) for f in (HALF, 1.0) for s in (1.0, -1.0) if room > 0.0]
            k += 1
    opts.sort(key=lambda o: o[0])
    for key, yaw, w, c, slide, shift, open_m, tip_z, depth, why, seen_in, beside in opts:
        q, R_low, yaw_r, squared = _pose(ex, J, c, tip_z, yaw, F["t"], open_m, w, F["seeds"])
        if q is not None:
            break
    else:
        return None
    lines, follow = [], False
    if not squared and S["narrow"] is not None and strategy == "top":
        w = extent_along(S["narrow"], S["wide"], S["bearing"], yaw_r)    # an arm that cannot turn its jaws there:
        open_m = min(J.max_m, w + F["err"] + F["err"] + F["off"] + F["off"])     # the pose again, for the width
        again = _pose(ex, J, c, tip_z, yaw_r, F["t"], open_m, w, [q] + list(F["seeds"]))    # across that bearing
        q, R_low = (again[0], again[1]) if again[0] is not None else (q, R_low)
        lines.append(line(f"this arm cannot turn its jaws across the side chosen: they close along "
                          f"{math.degrees(yaw_r):.0f} deg, where it is seen {w * MM:.0f} mm across (the jaws open "
                          f"{J.max_m * MM:.0f} mm)", "came down with the jaws as the arm can turn them, the contact "
                          "stop armed", None))
    elif w is not None and S["narrow"] is None and J.one_jaw:
        # a width that was stated and not seen says nothing of the bearing across which the thing is that wide: the
        # fixed pad of a hand with one moving jaw, which would come down half that width off its middle (and on
        # the thing, where it is wider across the jaws), stands off its face by what puts the thing's middle at
        # the point between the pads with the jaws as far open as they go; the hand then moves with its jaw
        from rcpx.r2.place_pick_count import pads_mid_T      # that file imports this one
        frac, was = J.frac_for_width(J.max_m), J.clear_m
        tip_n, depth_n, why_n = depth_rule(J, top, F["sup_z"], frac, F["rigid"], cap(J.max_m) if cap else None)
        pts = J.points_tool(frac)
        toward = 1.0 if float((pts[0] - pts[1]) @ J.axis_T) >= 0.0 else -1.0
        J.clear_m = max(was, toward * float((pads_mid_T(J, frac) - J.hold_centre_T(frac, w, 0.0)) @ J.axis_T))
        again = _pose(ex, J, c, tip_n, yaw_r, F["t"], J.max_m, w, [q] + list(F["seeds"]))
        if again[0] is None:
            J.clear_m = was
        else:
            lines.append(line(f"its width was stated ({w * MM:.0f} mm) and not seen: nothing says across which "
                              f"bearing it is that wide, and the jaws close along {math.degrees(yaw_r):.0f} deg",
                              "the jaws as far open as they go, its middle at the point between the pads, the hand "
                              "moving with its jaw as it closes", LOOK))
            q, R_low, open_m, tip_z, depth, why, follow = again[0], again[1], J.max_m, tip_n, depth_n, why_n, True
    (n, over, word), (gap, nb, nb_word) = seen_in, beside
    if n >= R.QUANT:
        lines.append(line(f"the look shows something {over * MM:.0f} mm higher than the hand's {word} comes, where "
                          f"the {word} comes down ({n} points of its depth picture)", "no bearing or slide this arm "
                          "reaches is free of it: came down as planned, the contact stop armed", LOOK))
    if nb is not None and gap <= nb["sigma_xy"]:
        lines.append(line(f"the {nb_word} comes within {max(gap, 0.0) * MM:.0f} mm of what stands beside it (known to "
                          f"{nb['sigma_xy'] * MM:.1f} mm)", "took the freest bearing this arm reaches, the contact "
                          "stop armed", LOOK))
    if w is not None and (shift or open_m < min(J.max_m, w + F["err"] + F["err"] + F["off"] + F["off"])):
        lines.append(line("something the look shows stands in the hand's way around its middle: the "
                          + (f"hand comes down {abs(shift) * MM:.0f} mm off its middle along the closing axis (the "
                             "closing jaws bring it to theirs)" if shift else f"jaws open only {open_m * MM:.0f} mm")
                          + f", so a finger passes its face by {(HALF * (open_m - w) - abs(shift)) * MM:.0f} mm (known "
                          f"to {F['err'] * MM:.1f} mm; at this speed the hand can be {F['off'] * MM:.0f} mm off its "
                          "path)", "came down there, the contact stop armed", LOOK))
    slid = "" if not slide else f", {abs(slide) * MM:.0f} mm along its other side from its middle"
    slid += "" if not shift else f", {abs(shift) * MM:.0f} mm off its middle along the closing axis"
    # how far the jaws' middle goes from the point that was given (a pixel anywhere on a thing takes the thing:
    # its seen middle, its wall or its narrowest part there, not the pixel's own point)
    away = None if S.get("pointed") is None else float(np.linalg.norm(np.asarray(c, dtype=float)[:2] - S["pointed"]))
    if away is not None and strategy != "point":
        slid += f"; the jaws' middle goes {away * MM:.0f} mm from the point you gave"
    return {"strategy": strategy, "yaw": float(yaw_r), "slide_m": float(slide), "shift_m": float(shift),
            "from_point_m": away,
            "open_m": float(open_m), "tip_z": float(tip_z), "approach_axis": [float(x) for x in F["t"]],
            "v_down": F["v"], "v_free": F["v"], "depth_m": float(depth), "q_low": q, "R": R_low,
            "why": f"{words}, {depth * MM:.0f} mm under its top ({why}), jaws open {open_m * MM:.0f} mm{slid}",
            "centre": [float(c[0]), float(c[1])], "width_m": None if w is None else float(w), "squared": bool(squared),
            "neighbour_gap_m": None if nb is None else float(gap), "top_z": float(top), "support_z": F["sup_z"],
            "expect_m": None if w is None else float(w), "sigma_xy": F["err"], "off_m": F["off"], "band_m": F["band"],
            "in_the_way": n, "in_the_way_over_m": over, "doubt": lines, "follow": follow}


def choose(ex, seen, hints=None):
    """The grasp for a Seen. hints: {width_m, yaw_rad, grasp: auto | top | rim | side, t: approach axis, speed}.
    Returns the plan or a refusal (module docstring)."""
    from rcpx.r2 import place_grasp_ways       # that file imports this one
    R = _ratios()
    B = _bounds(ex)
    J = ex._pl_jaws
    hints = dict(hints or {})
    S = read_seen(seen)
    t = np.asarray(hints.get("t") if hints.get("t") is not None else DOWN, dtype=float)
    want = hints.get("grasp") or "auto"
    if S["centre"] is None or S["top_z"] is None:
        return _refuse("spot", "the spot gives no point to grasp at", "point at the thing in a look ({look, px}), "
                       "or give its top middle as a point")
    s0 = ex._sense()
    at = S["pointed"] if S["pointed"] is not None else S["centre"]
    sup_z, _, _ = support_at(B, at)
    if want == "side":
        return _side(ex, B, J, S, t, sup_z)
    v = num(B.v_free(s0.q, t), 0.0)
    v = min(v, float(hints["speed"])) if hints.get("speed") is not None else v
    rigid = float(hints["band_m"]) if hints.get("band_m") is not None else num(B.rigid(s0.q, t), 0.0)
    hand_yaw = ex._pl_jaw_bearing(s0.R)
    palm = J.palm_box() or {"a": (0.0, 0.0), "b": (0.0, 0.0)}
    reach = math.hypot(max(abs(x) for x in palm["a"]), max(abs(x) for x in palm["b"])) + J.max_m + (S["wide"] or 0.0)
    F = {"t": t, "sup_z": sup_z, "err": S["sigma_xy"], "rigid": rigid, "band": rigid, "v": v, "hand_yaw": hand_yaw,
         "off": num(B.e_cross(v), 0.0), "seeds": [s0.q, ex.model.q_home], "pts": seen_points(ex, seen, at, reach),
         "margin": R.BAND * max(S["sigma_xy"], S["sigma_z"] or 0.0, rigid)}
    both = F["err"] + F["err"] + F["band"] + F["band"]
    finger = J.finger_len_m or J.pad_height_m or 0.0
    fixed = hints.get("yaw_rad")
    narrow, wide, bearing, stated = S["narrow"], S["wide"], S["bearing"], False
    if narrow is None and hints.get("width_m") is not None:       # the operator states its width: that wide every way
        narrow = wide = float(hints["width_m"])
        bearing, stated = (hand_yaw if fixed is None else float(fixed)), True
    wall = _wall_of(S, at) if (S["hollow"] is True or want == "rim") else None
    part = S["part"] if S["pointed"] is not None and S["part"] is not None and \
        S["part"]["width"] - F["err"] - F["err"] <= J.max_m else None
    if narrow is not None and (stated or S["whole"]) and wall is None and part is None \
            and narrow - (0.0 if stated else F["err"] + F["err"]) > J.max_m:
        return _refuse("too_wide", f"it is {narrow * MM:.0f} mm across its narrow side ("
                       + ("as you stated" if stated else f"seen whole, known to {F['err'] * MM:.1f} mm")
                       + f") and the jaws open {J.max_m * MM:.0f} mm; no wall and no narrower part of it is seen at "
                       "your point", "push or slide it (push, shove), or point at a narrower part of it")
    lines = [line(d, "acted on what the look shows, the contact stop armed", S["remove_by"]) for d in S["doubt"]]
    cands = []
    point_z = S["point_z"]
    if sup_z is not None and point_z < sup_z:       # a point under the stated support: the hand meets the plane first
        if not any("under the stated support" in d for d in S["doubt"]):
            lines.append(line(f"your point lies {(sup_z - point_z) * MM:.0f} mm under the stated support",
                              "took the stated support as its height: the fingertips stop over the support", None))
        point_z = sup_z

    def unseen(open_m):
        """Nothing is seen of the thing under the point (no outline, no top of its own): the point is taken as
        its top, and the fingers come half their own length under it, no deeper than half its height over the
        stated support (the pads take it at its middle)."""
        half = None if sup_z is None else HALF * (point_z - sup_z)
        d = R.GIVE * finger if half is None or half <= 0.0 else min(R.GIVE * finger, half)
        return d, ("nothing is seen of what stands under the point: half the fingers' depth under it, no deeper "
                   "than half its height over the support")
    blind = unseen if S["narrow"] is None else None
    # a thing that fits the hand whole is taken at its seen middle wherever on it the pixel lies
    if place_grasp_ways.whole(narrow, both, J.max_m, want):
        if fixed is not None:
            opts = [(float(fixed), extent_along(narrow, wide, bearing, float(fixed)))]
        else:
            opts = [(bearing, narrow), (bearing + math.pi, narrow)]
            if wide + both < J.max_m:        # it fits across its long side too: the hand turns to the nearer
                opts += [(bearing + s * HALF * math.pi, wide) for s in (1, -1)]
        cands.append(("top", "a grasp of the whole thing from above, jaws across a side that fits them", S["centre"],
                      opts, S["top_z"], blind))
    elif narrow is not None and want == "auto":
        lines.append(line(f"it is seen {narrow * MM:.0f} mm across its narrow side (known to {F['err'] * MM:.1f} mm) "
                          f"and the jaws open {J.max_m * MM:.0f} mm", "took its wall or the part of it nearest your "
                          "point that fits the jaws, else the point itself (see grasp)", S["remove_by"]))
    if wall is not None and want in ("auto", "rim"):
        p, inward, thick = wall
        a = math.atan2(inward[1], inward[0])

        def inside(open_m, fall=S["depth_inside"], pad=J.pad_height_m):
            """How deep the jaws come around a wall: the pads' own height (the faces that press cover the wall
            from its rim down), no deeper than the inside is seen to be where the pointed ray landed."""
            known = [x for x in (pad, fall) if x is not None]
            return (min(known), "the pads' own height under its rim, no deeper than its inside is seen") if known \
                else None
        cands.append(("rim", "a grasp of its wall nearest your point, the jaws along the wall's inward direction, "
                      "one finger inside" + ("" if S["hollow"] is True else " (on your word that it is hollow)"),
                      p, [(a, thick), (a + math.pi, thick)] if fixed is None else [(float(fixed), thick)],
                      S["top_z"], inside))
    if part is not None and want == "auto":
        cands.append(("part", "a grasp of the part at the pointed spot, across its narrowest chord",
                      part["centre"] if part["centre"] is not None else S["pointed"],
                      [(part["bearing"] if fixed is None else float(fixed), part["width"])]
                      + ([(part["bearing"] + math.pi, part["width"])] if fixed is None else []),
                      part["top_z"] if part["top_z"] is not None else S["top_z"], None))
    turns = [0.0, math.pi, HALF * math.pi, -HALF * math.pi]
    last = ("point", "the open jaws at the point itself" + ("" if S["narrow"] is not None else
                                                            " (no thing is made out there)"), at,
            [(hand_yaw + k, None) for k in turns] if fixed is None else [(float(fixed), None)], point_z, blind)
    more = []

    def every():
        """The candidates in order: the whole thing; then, read once from the thing's own points when the whole
        thing gave no plan (place_grasp_ways), its rim nearest the pixel wherever on it the pixel lies
        (a hollow thing's wall, read from its top and centred where the pads meet it: ahead of the wall and the
        part read at the pixel itself); its wall at the pixel, its part through the pixel; every other part of
        it that fits the jaws, nearest the pixel first; the point itself last."""
        yield from (c for c in cands if c[0] == "top")
        rest = [c for c in cands if c[0] != "top"]
        if want == "auto" and not more:
            more.append(place_grasp_ways.cands(ex, seen, S, J, F, fixed))
        new = more[0] if more else []
        rims = [c for c in new if c[0] == "rim"]
        yield from rims
        yield from ([c for c in rest if c[0] != "rim"] if rims else rest)
        yield from (c for c in new if c[0] != "rim")
        yield last
    def pointless(plan):
        """What the look shows in the hand's way stands so high over the hand that the fingers cannot come down
        to the thing's top at all."""
        over = plan["in_the_way_over_m"]
        return plan["in_the_way"] >= R.QUANT * R.QUANT and over >= max(plan["depth_m"], F["margin"])
    for F["tight"] in (False, True):        # the tight openings and the shifts: only where every plan is blocked
        missed, plans = [], []
        for cand in every():
            plan = _take(ex, J, S, F, cand)
            if plan is None:
                missed.append(cand[0])
            else:
                plans.append((cand, plan))
                if not pointless(plan):
                    break
        if plans and not pointless(plans[-1][1]):
            break
    for cand, plan in plans[-1:] if plans and not pointless(plans[-1][1]) else plans[:1]:
        n, over = plan["in_the_way"], plan["in_the_way_over_m"]
        if pointless(plan):
            return _refuse("blocked", f"the look shows something {over * MM:.0f} mm higher than the hand comes, where "
                           f"the hand comes down for {plan['why'].split(',')[0]} ({n} points of its depth picture, "
                           f"known to {F['err'] * MM:.1f} mm): the fingers cannot reach {plan['depth_m'] * MM:.0f} mm "
                           "under its top past it, at any bearing, slide, opening or shift this arm reaches",
                           "move what stands beside it away first (push), or point at a part of it that stands free")
        missed += [c[0] + ": the fingers cannot come down past what the look shows" for c, _ in plans if c is not cand]
        if cand[0] == "point" and S["narrow"] is None and not S["doubt"]:
            lines.append(line("no thing is made out at your point (" + str(S["source"] or "the point as given") + ")",
                              "acted on the point: the open jaws around it, down to the fingers' depth under it",
                              S["remove_by"] or LOOK))
        if missed:
            lines.append(line(f"the better plan was not taken ({'; '.join(missed)}: no arm pose where not said)",
                              f"took {cand[1]}", None))
        plan["doubt"] = lines + plan["doubt"]
        plan["sigma_z"] = S["sigma_z"]
        if plan.get("width_m") is not None:       # where the width the close runs to came from (width.source)
            plan["width_source"] = "stated" if stated else "seen: the spot"
        return plan
    if abs(float(t[-1])) > HALF and want == "auto":
        side = _side(ex, B, J, S, t, sup_z)
        if "refused" not in side:
            return side
    return refusal("not_reachable", "no arm pose brings the open jaws to it from above",
                   "pick it from nearer the robot's base, or say grasp side")


def _finger_across(J):
    """A finger's own width across the closing axis (m): the spacing of the bearings a side grasp tries."""
    ws = [b["b"][1] - b["b"][0] for b in J.finger_boxes(1.0) if b is not None]
    return min(ws) if ws else 0.0


def _side(ex, B, J, S, t, sup_z):
    """The level grasp (rcpx/r2/place_grasp_side.py: this file is over its length)."""
    from rcpx.r2 import place_grasp_side
    return place_grasp_side.side(ex, B, J, S, t, sup_z)
