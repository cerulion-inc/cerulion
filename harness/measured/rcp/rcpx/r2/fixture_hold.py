# SPDX-License-Identifier: AGPL-3.0-only
"""The hold of package fixture: where the hand takes a fixed part, and with which posture.

A part is worked along a direction d (the way it should move). The hand's candidates:
- hook:    the closed fingers go in behind the part, thin side first, and draw it; nothing is squeezed (kept only
           where that side loads the part along the pull at least as much as across it);
- pinch:   the open jaws straddle the part from straight above the stroke: they close along d, one finger behind
           the part, so the pull does not rest on friction;
- across:  the open jaws straddle the part with the hand pointing at it against d, tilted up from d by a share of
           a right angle; the jaws close across the part's long side;
- fingers: for a push, the closed fingers lead with the tip along d (tilted like the grasps), or stand upright
           beside the part and push with their thin side;
- knob:    for a turn, the hand points along the axis into the part's face with the jaws across it.
Each candidate is solved by this arm's own IK in every distinct posture; how deep the tip goes past the part and
how wide the jaws come in are found by walking the WHOLE arm, on its hulls, against what the space book holds (the
support, every seen or touched thing); then the whole stroke the way implies is walked the same way. A candidate
whose links enter a band on the way in or at the hold is dropped; the fingers are not measured against the thing
that owns the part (that contact is the hold), the palm and every link are; along the stroke the owning thing is
not measured (it moves with the hand). Holds that do not rest on friction come first, then the stroke the arm
follows clear of every band, the least clearance, the least joint travel. A candidate whose links enter the band
of a MEASURED thing is not dropped: it is ranked after every clear one, by the room it leaves, and carries
which body passes which thing ("passes"); the tool takes the best ranked pose in reach and says so. Only a part
this arm's IK holds with none of the poses is refused (reach: no plan exists). The tip goes past the part no
deeper than puts the middle of the pads on it: what lies behind a part nobody measured is not known.

The jaws' readings are in rcpx/r2/fixture_probe.py (the close and the release are the core's, rcpx/control_hold.py);
jaw geometry is place_hand.Jaws.
Nothing here reads truth or names a robot. SI inside.
"""

import math

import numpy as np

from rcpx import bounds_laws as laws
from rcpx.bounds_book import cloud_clearance
from rcpx.bounds_ratios import BAND, QUANT, REL
from rcpx.context import refusal
from rcpx.model import _rotvec
from rcpx.r2 import fixture_look as L
from rcpx.r2.fixture_look import part_top, top_middle, up_of  # noqa: F401
from rcpx.r2.fixture_probe import HALF, TINY, bounds_of, laws_mm, rot, unit
from rcpx.r2.fixture_probe import jaw_read, jaw_step_m, squeeze_up  # noqa: F401


DIRECTIONS = {"up": (0.0, 0.0, 1.0), "down": (0.0, 0.0, -1.0), "forward": (1.0, 0.0, 0.0), "back": (-1.0, 0.0, 0.0),
              "left": (0.0, 1.0, 0.0), "right": (0.0, -1.0, 0.0)}      # unit vectors of the base frame, by name


def direction_of(direction):
    """(unit vector or None, refusal or None) of a stated direction: a name of the base frame or [x, y, z]."""
    if direction is None:
        return None, None
    v = DIRECTIONS.get(direction.strip().lower()) if isinstance(direction, str) else direction
    try:
        v = np.asarray(v, dtype=float).reshape(-1)
        ok = v.shape == (len(DIRECTIONS["up"]),) and bool(np.all(np.isfinite(v))) and float(np.linalg.norm(v)) > 0
    except (TypeError, ValueError):
        ok = False
    if not ok:
        return None, refusal("direction", f"direction must be one of {', '.join(DIRECTIONS)} or [x, y, z] in the base "
                             f"frame, got {direction!r}", "work_part(target=[x, y, z], way='pull', direction='up')")
    return unit(v), None


def lift_axis(ex, d, p):
    """The unit direction square to d from which the hand comes: away from the support; for a stroke along the
    support's own normal, back toward the robot's base."""
    up = up_of(ex)
    e = unit(up - float(up @ d) * d)
    if not np.any(e):
        back = -np.asarray(p, dtype=float)
        e = unit(back - float(back @ d) * d)
    return e


def face_normal(ex, p, reach_m):
    """The unit normal (pointing away from it) of the face of the nearest thing the space book holds within
    reach_m of point p, and that thing's id; (None, None) where nothing stands that near, or only the support."""
    B = bounds_of(ex)
    _, near, c, _ = B.space.kind(p, p, 0)
    if near is None or not c <= reach_m:
        return None, None
    E = B.thing(near)
    if E.id == "support":
        return None, None
    extent = E.extent or {}
    if "plane" in extent:
        return unit(extent["plane"][0]), E.id
    if "box" in extent:
        lo, hi = (np.asarray(v, dtype=float) for v in extent["box"])
        gap, k, sign = max(max((p[k] - hi[k], k, 1.0), (lo[k] - p[k], k, -1.0)) for k in range(len(p)))
        return sign * np.eye(len(p))[k], E.id
    if E.point is not None and np.any(unit(p - E.point)):
        return unit(p - E.point), E.id
    return None, None


def tilts():
    """Shares of a right angle the hand is tilted from the stroke axis, nearest the middle first."""
    return [s * math.pi * HALF for s in (HALF, HALF * HALF, 1 - HALF * HALF, 1, 0)]


def tool_rot(J, t, n):
    """The hand's rotation with its pointing axis along t and the jaws closing along n."""
    a = unit([0, J.axis_T[1], J.axis_T[QUANT]])
    x = np.eye(len(a))[0]
    tool = np.column_stack([x, a, np.cross(x, a)])
    return np.column_stack([t, n, np.cross(t, n)]) @ tool.T


def finger_box(ex, J, frac):
    """Half extents (m) of the fingers at opening frac in the tool frame: (across the jaws, the thin side), read
    from their own hulls; None where the model gives no hulls."""
    B, m = bounds_of(ex), ex.model
    q = m.q_home
    p, R = m.fk_tip(q)
    was = None if getattr(m, "_qgeom", None) is None else m._qgeom[m._jaw_adr].copy()
    m.set_jaws(J.q_of_frac(frac))
    try:
        pts = [B.robot.hull_points(q, name) for name in sorted(m.jaw_bodies())]
    finally:
        m.set_jaws(J.closed if was is None else was)
    pts = [h for h in pts if h is not None and len(h)]
    if not pts:
        return None
    T = (np.vstack(pts) - p) @ R
    return float(np.max(np.abs(T[:, 1]))), float(np.max(np.abs(T[:, QUANT])))


def part_of(ex, target, seen=None):
    """What is worked: {p (m), ref, thing, owner (the book thing whose band holds the point), width_m, across
    (the unit level direction across the part as a look shows it: along its narrowest chord through the point),
    bearing (its long side: square to across), long (the look shows the thing longer than it is narrow by more
    than BAND of its own error: only then does the long side say which way the part is worked), whole_across
    (where long: the unit level direction across the whole thing's outline, along its narrowest width), sigma_m}.
    target is a world id or label (a located point), [x, y, z] mm, or a resolved Seen."""
    B = bounds_of(ex)
    entry, width, bearing, across, long, stale, wide, whole = None, None, None, None, False, None, None, None
    if isinstance(seen, dict):
        xy = seen.get("pointed_xy") or seen.get("centre_xy")
        p = np.array([xy[0], xy[1], seen["top_z"]], dtype=float)
        chord = seen.get("part_chord") or {}
        width = chord.get("width")
        if chord.get("bearing") is not None:
            # the look gives the bearing of the NARROWEST chord through the point: the direction across the part.
            # The part's long side is square to it on the level
            up = up_of(ex)
            e1, e2 = L.level_axes(up)
            across = unit(math.cos(chord["bearing"]) * e1 + math.sin(chord["bearing"]) * e2)
            bearing = unit(np.cross(up, across))
        ref = seen.get("id") if seen.get("id") is not None else p
        sigma = max(float(seen.get("sigma_xy") or 0), float(seen.get("sigma_z") or 0))
        outline = seen.get("outline") or {}
        long = bool(across is not None and outline.get("wide") is not None and outline.get("narrow") is not None
                    and float(outline["wide"]) - float(outline["narrow"]) > BAND * sigma)
        wide = float(outline["wide"]) if long else None
        if long and outline.get("bearing") is not None:     # the whole thing's narrowest direction, as the look
            e1, e2 = L.level_axes(up_of(ex))                 # shows its outline (the chord is read at the point)
            whole = unit(math.cos(outline["bearing"]) * e1 + math.sin(outline["bearing"]) * e2)
    elif isinstance(target, str):
        got = ex.world.get(target)
        found = [got] if got is not None else ex.world.query(label=target)
        found = [e for e in found if e.get("point_mm") is not None]
        fresh = [e for e in found if not e.get("stale")]
        if not found:
            return None, refusal("unknown_target", f"robot://world holds no located point named {target!r}",
                                 "point at it in a look: work_part(target={'look': 'L1', 'px': [u, v]}, way='pull')")
        entry = (fresh or found)[-1]
        if not fresh:              # a stale target is acted on where it was last known, and said
            why = (entry.get("data") or {}).get("stale_why")
            stale = {"what": f"{target!r} is only known from before something moved" + (f" ({why})" if why else ""),
                     "acted": "went to where it was last known, the contact stop armed",
                     "would_remove": {"tool": "look", "args": {}, "periods": 0}}
        p = np.asarray(entry["point_mm"], dtype=float) / laws_mm()
        ref = entry["id"]
        sigma = None
    else:
        try:
            p = np.asarray(target, dtype=float).reshape(-1)
            ok = p.shape == (len(up_of(ex)),) and bool(np.all(np.isfinite(p)))
        except (TypeError, ValueError):
            ok = False
        if not ok:
            return None, refusal("target", "target must be a located label, a point [x, y, z] in mm, or a pixel of "
                                 "a look {look, px}", "work_part(target=[x, y, z], way='pull')")
        p = p / laws_mm()
        ref, sigma = p, None
    thing = B.thing(ref)
    if sigma is None:
        sigma = float(thing.sigma_p or 0)
    kind, near, _, _ = B.space.kind(p, p, 0)
    owner = None
    if kind == "expected" and near is not None and getattr(near, "id", None) != "support":
        owner = getattr(near, "id", None)
    return {"p": p, "ref": ref, "entry": entry, "thing": thing, "owner": owner, "width_m": width,
            "bearing": bearing, "across": across, "long": long, "wide_m": wide, "sigma_m": sigma, "stale": stale,
            "whole_across": whole}, None


# ---------------------------------------------------------------------------------------------- the candidates
def _cand(name, tilt, R, p_base, deep, a, f_open, closes, depth_min, depth_max, side_m=None):
    return {"name": name, "tilt_deg": None if tilt is None else round(math.degrees(tilt), 1), "R": R,
            "p_base": p_base, "deep": deep, "a": a, "open": f_open, "closes": closes, "depth_min": depth_min,
            "depth_max": depth_max, "side_m": side_m}


def _upright(J, t, d, flip):
    """The hand pointing along t with the fingers' thin side leading along d."""
    x, y = np.eye(len(d))[0], np.eye(len(d))[1]
    return np.column_stack([t, flip * np.cross(d, t), flip * d]) @ tool_rot(J, x, y).T


def candidates(ex, part, way, d, grasp):
    """Hand poses that take the part for a stroke along d (unit): [{name, tilt_deg, R, p_base (the tip with the
    part's point in the middle of the gap, or at the tip), deep (the unit direction the tip goes on past it),
    depth_min, depth_max, a (the unit direction of the last leg in), open (the jaws' opening share on the way in),
    closes}]. How deep the tip goes is found by the whole arm's walk (rank)."""
    J, B = ex._fx_jaws, bounds_of(ex)
    p, e = part["p"], lift_axis(ex, d, part["p"])
    sure = BAND * part["sigma_m"]
    if J is None:
        if way != "push":
            return []
        out = []
        for tilt in tilts():
            t = unit(math.cos(tilt) * d - math.sin(tilt) * e)
            for q, _, _ in ex.model.postures(ex._sense().q, p, keep_axis=t):
                out.append(_cand("tip", tilt, ex.model.fk_tip(q)[1], p.copy(), d, d, 0.0, False, 0.0, 0.0))
        return out
    bar = part["bearing"] if part["bearing"] is not None else unit(np.cross(d, e))
    box = finger_box(ex, J, 0)
    thin = box[1] if box is not None else J.max_m * HALF * HALF
    deepest = (J.finger_len_m or J.max_m) * HALF
    width = part["width_m"] or 0.0
    out = []
    if way == "pull":
        opens = [1.0, HALF, HALF * HALF]            # the widest the neighbours allow comes first
        if part["width_m"] is not None:
            room = B.clearance(0, part["thing"]) + QUANT * part["sigma_m"]
            opens.append(J.frac_for_width(min(J.max_m, width + room + room)))
        for tilt in tilts():
            t = unit(-math.cos(tilt) * d - math.sin(tilt) * e)
            n = unit(np.cross(bar, t))
            name = "pinch" if abs(float(t @ d)) <= TINY ** HALF else "across"
            if not np.any(n) or grasp not in ("auto", name):
                continue
            for f_open in opens:
                if J.width_m(J.q_of_frac(f_open)) <= width + sure:
                    continue
                for flip in (1, -1):
                    R = tool_rot(J, t, flip * n)
                    out.append(_cand(name, tilt, R, p - R @ J.gap_centre_T(f_open), t, t, f_open, True,
                                     width * HALF + sure, deepest,
                                     (J.width_m(J.q_of_frac(f_open)) - width) * HALF))
        if grasp in ("auto", "hook") and J.finger_len_m:
            for tilt in tilts():
                t = unit(-math.cos(tilt) * d - math.sin(tilt) * e)
                face = unit(d - float(d @ t) * t)          # the closed fingers' thin side, looking along the pull
                if float(face @ d) < abs(float(face @ e)):  # it would load the part across its path more than
                    continue                                # along it
                for side in (thin, thin * HALF, B.clearance(0, part["thing"])):   # room left between it and the part
                    for flip in (1, -1):
                        out.append(_cand("hook", tilt, _upright(J, t, face, flip),
                                         p - (width * HALF + thin + side) * face, t, t, 0.0, False,
                                         width * HALF + sure, deepest, side))
    elif way == "push" and grasp in ("auto", "fingers"):
        n = unit(np.cross(e, d))
        for tilt in tilts():
            t = unit(math.cos(tilt) * d - math.sin(tilt) * e)
            for flip in (1, -1):
                if abs(float(t @ d)) <= TINY ** HALF:      # upright beside the part: the thin side pushes
                    out.append(_cand("fingers", tilt, _upright(J, t, d, flip), p - thin * d, t, d, 0.0, False,
                                     sure, deepest))
                elif part.get("on_top"):                   # the look shows the part's TOP at the point: the face
                    #                                        that is pushed lies under it, and the tip goes down
                    #                                        past the point as an upright hand's does
                    out.append(_cand("fingers", tilt, tool_rot(J, t, flip * n), p.copy(), -e, d, 0.0, False,
                                     sure, deepest))
                else:
                    out.append(_cand("fingers", tilt, tool_rot(J, t, flip * n), p.copy(), d, d, 0.0, False, 0.0, 0.0))
    return out


def turn_candidates(ex, part, axis_in, grasp):
    """Hand poses for a turn about the tool axis: the hand points along axis_in (into the part's face), the jaws
    across the part at bearings a share of a half turn apart. First the bearing square to the narrowest chord a
    look shows through the pointed point (the order round four gave it; measured on the four development cells of
    the bridge's knob it holds the part in 4 of 4, the chord's own bearing first in 0 of 4: a part that is turned
    is gripped about its own middle, and the chord is read at the pointed point)."""
    J = ex._fx_jaws
    p = part["p"]
    e = lift_axis(ex, axis_in, p)
    side = unit(np.cross(axis_in, e))
    first = part.get("across")
    if first is not None:
        first = unit(first - float(first @ axis_in) * axis_in)
        first = unit(np.cross(axis_in, first)) if np.any(first) else None
    bearings = ([first] if first is not None else []) + [side, e, unit(side + e), unit(side - e)]
    width = part["width_m"] or 0.0
    sure = BAND * part["sigma_m"]
    opens = [1.0, HALF] if part["width_m"] is None else [J.frac_for_width(min(J.max_m, width + sure + sure
                                                                              + J.max_m * REL)), 1.0]
    out = []
    for n in bearings:
        for f_open in opens:
            if J.width_m(J.q_of_frac(f_open)) <= width + sure:
                continue
            for flip in (1, -1):
                R = tool_rot(J, axis_in, flip * n)
                out.append(_cand("knob", None, R, p - R @ J.gap_centre_T(f_open), axis_in, axis_in, f_open, True,
                                 sure, (J.finger_len_m or J.max_m) * HALF))
    return out


# ---------------------------------------------------------------------------------------------- the whole arm
def exempt_bodies(ex):
    return set(ex.model.jaw_bodies()) if hasattr(ex.model, "jaw_bodies") else set()


def blockers(ex, q, own=None, tool_free=False):
    """Every body of the arm at q that stands inside the band of a thing the space book holds: [(body, thing id,
    clearance m)], and the least clearance over its band (m). own: the id of the thing being worked: the fingers
    are not measured against it (that contact is the hold), the palm and every link are; with tool_free no body is
    (along the stroke the part moves with the hand: what was checked at the hold stays as it was)."""
    B = bounds_of(ex)
    free = exempt_bodies(ex)
    everyone = bool(tool_free)
    fits = B.robot.e_fit()
    hits, least = [], math.inf
    by_body = {}
    for pt in B.robot.points(q):
        by_body.setdefault(pt.body, []).append(pt)
    for body, ends in by_body.items():
        if body == "tip":
            continue
        kind, ref, c, _ = B.space.kind(ends[0].p, ends[-1].p, ends[0].radius)
        if ref is None:
            continue
        E = B.thing(ref)
        if E.id is not None and E.id == own and (everyone or body in free):
            continue
        band = laws.band(E.sigma_p or 0)
        margin = c - band
        if kind == "expected" or c <= fits.get(body, 0) + band:
            hull = B.robot.hull_points(q, body)
            if hull is not None and len(hull):
                c = float(np.min(cloud_clearance(hull, E)))
                margin = c - band
            if margin <= 0:
                hits.append((body, str(E.id), c))
        least = min(least, margin)
    return hits, least


def step_to(ex, q, p, R, rounds=None):
    """The joint vector near q whose tip stands at (p, R): a few damped least squares steps, joint limits kept.
    Returns (q, error m)."""
    m = ex.model
    q = np.asarray(q, dtype=float).copy()
    for _ in range(int(QUANT) if rounds is None else rounds):
        p_now, R_now = m.fk_tip(q)
        q = q + m.step_toward(q, p - p_now, _rotvec(R @ R_now.T))
    return q, float(np.linalg.norm(m.fk_tip(q)[0] - p))


def tip_at(ex, q, p, tol_m):
    """The joints near q whose tip stands at p with the rotation q has: steps that move the tip alone and hold the
    rotation, until the tip stands within tol_m of p or a step no longer brings it nearer by the accepted
    relative share. Returns (q, error m)."""
    m = ex.model
    q = np.asarray(q, dtype=float).copy()
    p = np.asarray(p, dtype=float)
    err = float(np.linalg.norm(m.fk_tip(q)[0] - p))
    while err > tol_m:
        q2 = q + m.step_toward(q, p - m.fk_tip(q)[0], np.zeros(len(p)))
        e2 = float(np.linalg.norm(m.fk_tip(q2)[0] - p))
        if not e2 < (1 - REL) * err:
            break
        q, err = q2, e2
    return q, err


def stand_at(ex, q, p, R, tol_m):
    """step_to, with the tip put where it was asked (tip_at). A turn commands a rotation that leads the hand (the
    lead is its torque); where the few pose steps do not reach both, the rotation gave the tip away: the
    commanded tip stood up to 67 mm from the pose asked for and the servo drove the hand along its axis with
    116 N (measured on a development cell, at a knob's stop). The place is what the hold needs; the lead is made
    up over the next periods. Returns (q, error m)."""
    return tip_at(ex, step_to(ex, q, p, R)[0], p, tol_m)


def walk(ex, q0, pose_at, length, own, spacing, tool_free=False, free=False, seen=False):
    """Walk the hand from posture q0 through pose_at(s) -> (p, R) for s from 0 to length (metres, or radians of a
    turn); the next sample stands no farther than half the least clearance over a band found at this one, and no
    nearer than spacing (a path is never thinned under what its margins allow). Returns {reach (how far the arm
    follows), clear (how far no body enters a band; -1: not even at the start), least_m (the least clearance over a
    band on the clear stretch), block (the first (body, thing, clearance) or None), q_clear (the posture at the end
    of the clear stretch)}. free: the walk does not end where a body enters a band (clear is then how far the arm
    follows, block the first band entered): what a pose that is not clear would pass. seen: a body that stands
    under what the newest look shows (fixture_look.under) blocks as a band does."""
    m, B = ex.model, bounds_of(ex)
    q, reach, clear, least, block, q_clear = np.asarray(q0, dtype=float), 0.0, -1.0, math.inf, None, None
    tol = max(B.rigid(q0, None), B.robot.w_tool() * REL)
    scale = 1.0
    if length > 0:
        probe, _ = pose_at(min(length, spacing))
        moved = float(np.linalg.norm(probe - pose_at(0.0)[0]))
        scale = min(length, spacing) / moved if moved > 0 else 0.0     # path coordinate per metre of travel
    s = 0.0
    while True:
        p, R = pose_at(s)
        if s > 0:
            q_next, err = step_to(ex, q, p, R)
            if err > tol or not m.within_limits(q_next):
                break
            q = q_next
        reach = s
        hits, margin = blockers(ex, q, own, tool_free)
        if seen:
            hits = hits + L.under(ex, q)
        if hits:
            block = block or hits[0]
            if not free:
                break
        if not hits or free:        # a free walk goes on past a band (the first block is kept): reach is asked
            least, clear, q_clear = min(least, margin), s, q.copy()
        if s >= length:
            break
        stride = max(spacing, (margin * HALF * scale) if scale > 0 and math.isfinite(margin) else spacing)
        s = min(length, s + stride)
    return {"reach": reach, "clear": clear, "least_m": least, "block": block, "q_clear": q_clear}


def _line(p0, R, d):
    return lambda s: (p0 + s * d, R)


def _spin(p0, R, axis):
    return lambda s: (p0, rot(axis, s) @ R)


def pose_score(name, passes, stroke_m, room_m, change, swept=None):
    """The rank key of a solved hand pose, best first in a reverse sort. A pose clear of every measured band and of
    what the look shows comes first, by the hold that does not rest on friction, then (a push: swept, push_hold)
    the hold whose arm clears what the look shows over the part along the stroke, the nearest the pointed point
    first, then the stroke followed clear (stroke_m), the room (room_m), the least joint travel (change). A pose
    that passes inside one (passes not None) comes after every clear one BY THE ROOM IT LEAVES first (how far under
    what a look shows, how near a measured thing), then as a clear one: the stroke's clear length never ranks it
    ahead of a pose that passes less deep (measured on four development cells of one part: a pose whose wrist
    stood 205 mm under what the look showed was taken first for a stroke 25 mm longer and its way in ended on a
    link at 241 N read; 0 of 4 took hold; with the room first 4 of 4 took hold)."""
    friction = name in ("hook", "pinch")
    stroke = round(max(0.0, float(stroke_m)), int(BAND))
    room = round(float(room_m), int(BAND))
    clears = bool(swept and swept.get("clears"))
    near = -round(float(swept.get("lowered_m") or 0.0), int(BAND)) if clears else 0.0
    if passes is None:
        return (True, friction, clears, near, stroke, room, -change)
    return (False, room, friction, clears, near, stroke, -change)


def push_hold(ex, part, cand, q_hold, p_hold, d, spacing):
    """Where the fingers of a push bear on the part's face, so that the arm, moved along the stroke d, clears what
    the newest look shows over the part's top (fixture_look above) until the part's face comes to the face of the
    thing it goes into (fixture_look face_ahead): the hold the pose was solved at first, else the nearest under it
    on the face, in steps of half a pad, down to what the part's thing stands on (its seen base, else the support).
    The arm is read at the hold as solved, moved down as one, then solved again at the hold taken. Where none
    clears, the hold stays where it was solved, and the reading says after how much of the stroke which body meets
    what the look shows. Returns (q, p, swept): swept None where the look shows nothing over the part or no face
    ahead (the pose is left as solved); else {"clears", "lowered_m", "look", "need_m", "met_m", "body", "top_m",
    "base_m"}. Nothing here reads truth."""
    e = (part.get("spot") or {}).get("extent") or {}
    up = up_of(ex)
    top = e.get("top_near_z") if e.get("kind") == "thing" else None
    top = float(part["p"] @ up) if top is None else max(float(top), float(part["p"] @ up))
    got = L.above(ex, top)
    if got[0] is None:
        return q_hold, p_hold, None
    m, B = ex.model, bounds_of(ex)
    hand = set(getattr(ex, "_tool_bodies", ()) or ()) | exempt_bodies(ex)
    caps = [c for c in L.capsules(ex, q_hold) if c[0] in hand] or L.capsules(ex, q_hold)
    # the face is read on the stroke's own line, as wide as the fingers that lead it (a face askew to the stroke
    # stands nearer at one end of the hand than on that line)
    fingers = [r for body, _, _, r in caps if body in exempt_bodies(ex)]
    need = L.face_ahead(ex, part["p"], d, max(fingers, default=got[2] * BAND), top, got)
    if need is None:
        return q_hold, p_hold, None
    support = B.space.support()
    base = e.get("base_z") if e.get("kind") == "thing" else None
    if base is None and support is not None:
        base = float(support[0][1]) / float(np.linalg.norm(support[0][0]))
    base = float(p_hold @ up) if base is None else float(base)
    J = ex._fx_jaws
    step = max(float(J.pad_height_m or J.max_m) * HALF if J is not None else 0.0, float(spacing), got[2])
    tell = got[2]
    rows, h = [], 0.0
    while not rows or float(p_hold @ up) - h >= base - TINY:
        shifted = [(body, a - h * up, b - h * up, r) for body, a, b, r in caps]
        met, body = L.first_meet(shifted, got[0], d, need, tell)
        rows.append((h, met, body))
        h += step
    out = {"look": got[1], "need_m": need, "top_m": top, "base_m": base}
    tol = max(B.rigid(q_hold, None), B.robot.w_tool() * REL)

    def solve(p):
        """The joints with the tip at p and the pose's rotation, where they hold it clear of every band; else None."""
        q, err = step_to(ex, q_hold, p, cand["R"], rounds=int(BAND * QUANT))
        ok = err <= tol and m.within_limits(q) and not blockers(ex, q, part["owner"])[0]
        return q if ok else None

    for h, met, _ in rows:
        if met < need:
            continue
        if h <= 0:
            return q_hold, p_hold, dict(out, clears=True, lowered_m=0.0, met_m=met, body=None)
        q = solve(p_hold - h * up)
        if q is not None:
            again, _ = L.first_meet([c for c in L.capsules(ex, q) if c[0] in hand] or L.capsules(ex, q), got[0], d,
                                    need, tell)
            if again >= need:
                return q, p_hold - h * up, dict(out, clears=True, lowered_m=h, met_m=again, body=None)
    # no hold on the face clears: the hold stays where it was solved (where the pointed point put it), and the
    # receipt says which body meets what the look shows, after how much of the stroke
    _, met, body = rows[0]
    return q_hold, p_hold, dict(out, clears=False, lowered_m=0.0, met_m=met, body=body)


def swept_text(swept):
    """The receipt's words for how a push's hold was chosen against what the look shows over the part (push_hold),
    or None where nothing was read or the hold stands where it was solved and clears."""
    if not swept or (swept["clears"] and not swept["lowered_m"]):
        return None
    mm = laws_mm()
    if swept["clears"]:
        return (f"the hold is taken {swept['lowered_m'] * mm:.0f} mm under where it was solved, where the hand, moved "
                f"along the stroke, clears what look {swept['look']} shows over the part for the "
                f"{swept['need_m'] * mm:.0f} mm until the face the part goes into")
    return (f"no hold on the part's face keeps the hand clear of what look {swept['look']} shows over the part for the "
            f"{swept['need_m'] * mm:.0f} mm until the face it goes into: {swept['body'] or 'the hand'} would meet it "
            f"after {swept['met_m'] * mm:.0f} mm of the stroke, from the hold as solved")


def rank(ex, part, cands, d, length, standoff_m, tool_free=False, turn=None):
    """Solve every candidate with this arm's IK (each distinct posture), find how deep the tip goes past the part
    (no deeper than puts the middle of the pads on it, and clear of every band where one is known), then walk the
    stroke (a line of length metres along d; for a turn the angle `turn` radians about the hand's axis, both
    senses). Best first: the poses clear of every measured band and of what the newest look shows (holds that do
    not rest on friction, the stroke the arm follows clear, the least clearance, the least joint travel), then
    the poses that pass inside a band or stand under what the look shows, by the room they leave; each of those
    carries "passes": (body, thing or look, clearance m). Returns (choices, why): choices [] only when the arm
    reaches none of the poses (why {reach, block})."""
    m, B, J = ex.model, bounds_of(ex), ex._fx_jaws
    s = ex._sense()
    support = B.space.support()
    spacing = B.spacing(laws.band(support[1] or 0)) if support is not None else 0.0
    if not spacing > 0:
        spacing = max(B.robot.w_tool(), (J.max_m if J is not None else 0) * REL * REL)
    pad = (float(J.pad_height_m or 0.0) * HALF) if J is not None else 0.0
    choices, block, solved, seen_pose, log = [], None, 0, set(), []
    for cand in cands:
        key = (cand["name"], cand["tilt_deg"], tuple(np.round(cand["R"].ravel(), int(BAND))))   # numeric: a key
        if key in seen_pose:
            continue                # a wider opening of the same pose is tried only when the first found no way
        for q0, _, change in m.postures(s.q, cand["p_base"], R_target=cand["R"], starts=int(BAND * QUANT)):
            solved += 1
            if J is not None:
                m.set_jaws(J.q_of_frac(cand["open"]))
            line = _line(cand["p_base"], cand["R"], cand["deep"])
            want = min(cand["depth_max"], max(cand["depth_min"], pad))
            deep = walk(ex, q0, line, want, part["owner"], spacing)
            passes = None
            if deep["clear"] < cand["depth_min"]:        # a band on the way to the hold: the pose stays, and says so
                passes = deep["block"]
                deep = walk(ex, q0, line, cand["depth_min"], part["owner"], spacing, free=True)
                if deep["reach"] < cand["depth_min"] or deep["q_clear"] is None:
                    log.append((cand["name"], cand["tilt_deg"], "reach", round(deep["reach"], 4)))
                    continue
            depth, q_hold = deep["clear"], deep["q_clear"]
            p_hold = cand["p_base"] + depth * cand["deep"]
            swept = None
            if tool_free and not cand["closes"] and turn is None:
                # a push: the hold on the face where the hand, moved along the stroke, clears what the look shows
                # over the part until the part comes to the face it goes into (a palm that stands over the
                # fingertips meets what stands over the part before the part is home)
                q_hold, p_hold, swept = push_hold(ex, part, cand, q_hold, p_hold, d, spacing)
            p_pre = p_hold - standoff_m * cand["a"]
            q_pre, err = step_to(ex, q_hold, p_pre, cand["R"], rounds=int(BAND * QUANT))
            if err > spacing or not m.within_limits(q_pre):
                log.append((cand["name"], cand["tilt_deg"], "pre", round(err, 4)))
                continue
            hits_pre, margin_pre = blockers(ex, q_pre)          # before the part nothing is exempt
            hits_pre = hits_pre or L.under(ex, q_pre)           # nor what the newest look shows standing there
            passes = passes or (hits_pre[0] if hits_pre else None)
            p_back, q_back = p_pre - standoff_m * cand["a"], None    # one more standoff back: where the way in
            if not cand["closes"]:                                   # of a hand that pushes runs in straight
                q_back, err = step_to(ex, q_pre, p_back, cand["R"], rounds=int(BAND * QUANT))
                if err > spacing or not m.within_limits(q_back) or blockers(ex, q_back)[0] or L.under(ex, q_back):
                    q_back = None
            if J is not None and not cand["closes"]:
                m.set_jaws(J.closed)
            if turn is not None:
                arcs = {sign: walk(ex, q_hold, _spin(p_hold, cand["R"], sign * cand["R"][:, 0]), turn,
                                   part["owner"], spacing / max(J.max_m, spacing)) for sign in (1, -1)}
                got = max(arcs.values(), key=lambda g: g["clear"])
                extra = {"turn_clear": {sign: max(0.0, g["clear"]) for sign, g in arcs.items()}}
            else:
                got = walk(ex, q_hold, _line(p_hold, cand["R"], d), length, part["owner"], spacing, True)
                extra = {}
            if got["clear"] <= 0 and got["block"] is not None and length > 0:
                passes = passes or got["block"]
            block = block or passes
            seen_pose.add(key)
            room = min(got["least_m"], margin_pre, deep["least_m"])
            if passes:       # the room a pose leaves that passes a band; one under what a look shows: how far under
                room = -passes[-1] if str(passes[1]).startswith("look ") else passes[-1]
            room = round(float(room), int(BAND))
            choices.append({**cand, **extra, "q_hold": q_hold, "q_pre": q_pre, "p_pre": p_pre, "p_hold": p_hold,
                            "q_back": q_back, "p_back": p_back,
                            "depth_m": depth, "stroke": got, "passes": passes, "room_m": room, "swept": swept,
                            "score": pose_score(cand["name"], passes, got["clear"], room, change, swept)})
    restore_jaws(ex)
    choices.sort(key=lambda c: c["score"], reverse=True)
    return choices, {"reach": solved > 0, "block": block, "log": log, "candidates": len(cands)}


def passes_text(passes):
    """What a pose that is not clear of a measured band says: which body passes which thing, and how near; one
    that stands under what a look shows says that."""
    if not passes:
        return None
    body, thing, c = passes
    if str(thing).startswith("look "):
        return L.under_text(passes)
    return f"{body} passes {max(0.0, float(c)) * laws_mm():.0f} mm from {thing}, inside its band"


def restore_jaws(ex):
    """Pose the body model's jaws where the newest reading has them (the checks above posed them)."""
    g = ex.backend.read().get("gripper")
    if isinstance(g, dict) and g.get("q") is not None and hasattr(ex.model, "set_jaws"):
        ex.model.set_jaws(g["q"])


def no_posture(why, way):
    """The one refusal left of taking hold: this arm's own kinematics holds none of the hand poses the way implies
    (no plan exists). A pose that passes inside a measured band is taken and said, never refused."""
    return refusal("reach", "this arm reaches the part with none of the hand poses the way implies",
                   "give another point on the part, or another way: work_part(target=[x, y, z], way='" + way + "')")
