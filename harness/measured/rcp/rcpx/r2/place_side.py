# SPDX-License-Identifier: AGPL-3.0-only
"""Side entry of package place, version two: setting a held thing down at a place that stands
under a cover the space book holds (a roof, a shelf above, the body over an opened drawer), through an open side.

When it applies: the way straight down onto the place, for the held thing and the hand above it, crosses the band
of a thing the space book holds above the place (not the place itself), or the LOOKS held show something over the
place inside the held thing's column, over its own top band (_pl_top_band: what they show under it stands where
the thing would stand, never a roof; rcpx/r2/place_roof.py; the looks then stand in for
the book at the mouth and along the way in; where they show no side free, the way from above would set the thing
down on what they show, so the place is refused no_side_reach with what blocks each side tried, the thing still
held). A cover the looks show over more than half of the column with the place's own level seen under it is a
roof however much stands beside the place (the walls of an opening): it is never entered from above. What rises
from the place's own level into the column with no level seen under what covers it is the top of a thing standing
at the place: the plain way, said. A place the free spot found taken (rcpx/r2/place_spot.py: something stands
within the footprint where the thing would go) is answered here before anything moves (_pl_answer_place). Nothing
is inferred from a name or a colour.

The corridor: level entry lines that end over the place with the thing's bottom at the edge of the place's band.
Candidate bearings, in order: the bearing of the ray of the look the operator pointed in (those rays reached the
spot), the bearing from where the hand stands (the least way), the bearing from the robot's base to the place
(the arm reaches in along it), then a fan around each whose step is the angle the carried half width subtends at
the corridor's length. A bearing is a corridor when (1) some
way out along it the hand can come down from the travel layer (no expected thing in that column), (2) every
corner of the hand's hull and of the held thing, moved level from there to over the place, stays out of every
expected thing's band (the place's own surface excepted), and (3) this arm's own kinematics holds the hand at
both ends of that line with one hand rotation. Space on the line that no measured thing stands in is entered
under the guard: the corridor needs no proof, only no known thing in it.

The entry: one stream to the corridor's mouth (rise, across, down), the level line in (kind carry), the set-down
along gravity (place_set: an expected contact), the release, the line back out the
same way (kind retreat: through the space the entry swept) and the rise. No corridor or no arm pose: the refusal
no_side_reach, with the look that would help.

Also here, because the set-down and the side entry both need them: the hand's own hull in the tool frame
(hand_points), the test of how deep a hand follows into an opening (hull_2d, inside_by, follow_depth), and where a
held thing fits into an opening a look shows (free_spot: pure geometry on a level grid, nothing moves). Nothing
here reads ground truth or names a robot or a thing.
"""

import math

import mujoco
import numpy as np

from rcpx import bounds
from rcpx import control_hold
from rcpx.bounds_ratios import BAND, QUANT
from rcpx.context import refusal
from rcpx.control_base import _mm
from rcpx.r2 import place_roof
from rcpx.r2.place_motion import FULL_HAND_JOINTS, horizontal, phase, pose
from rcpx.r2.place_side_words import reasons_by_count

MM = 1000.0
QUARTER = math.pi / 2          # numeric: a quarter turn
NEXT = (-1, 0, 1)              # index: a cell and its neighbours, along a row or a column
HALF = math.ldexp(1, -1)       # numeric: one half exactly (half a width, half a cell)


def hand_hulls(ex):
    """[(body name, corner points)] of the hand's own collision shapes (every body from the tool body outward, the
    jaws at their present reading) in the tool frame (m; x = the pointing axis, origin at the tip): the model's
    hulls when it has them, else the boxes of its shapes."""
    J = ex._pl_jaws
    m, d = J.m, J.d
    q, _ = J.reading(ex.backend.read().get("gripper"), ex._gripper_cmd)
    d.qpos[:] = J.q_full
    d.qpos[J.qadr] = J.closed if q is None else q
    mujoco.mj_kinematics(m, d)
    R = d.site_xmat[J.tip_site].reshape(3, 3)
    tip = d.site_xpos[J.tip_site]
    root = int(m.site_bodyid[J.tip_site])
    jaws = set(J.sides)
    hulls = None
    fn = getattr(ex.model, "hulls", None)
    if callable(fn):
        try:
            hulls = fn()
        except Exception:
            hulls = None
    out = []
    for b in range(m.nbody):
        k = b
        while k > 0 and k != root and k not in jaws:
            k = int(m.body_parentid[k])
        if k != root and k not in jaws:
            continue
        name = m.body(b).name
        Rb, pb = d.xmat[b].reshape(3, 3), d.xpos[b]
        pts = []
        if isinstance(hulls, dict) and name in hulls:
            for p in np.asarray(hulls[name]["points_m"], dtype=float).reshape(-1, 3):
                pts.append(R.T @ (pb + Rb @ p - tip))
        else:
            for g in range(m.ngeom):
                if int(m.geom_bodyid[g]) != b or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                    continue
                c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
                Rg, pg = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        for sz in (-1, 1):
                            pts.append(R.T @ (pg + Rg @ (c + h * np.array([sx, sy, sz])) - tip))
        if pts:
            out.append((name, np.asarray(pts, dtype=float).reshape(-1, 3)))
    return out


def hand_points(ex):
    """Corner points of the hand's own collision shapes in the tool frame (hand_hulls, all bodies together)."""
    hulls = hand_hulls(ex)
    return np.vstack([pts for _, pts in hulls]) if hulls else np.zeros((0, 3))


def inside_grown(poly, xy, grow_m):
    """Which points xy (n x 2) lie inside a convex polygon (counter-clockwise, plan) grown by grow_m; a polygon
    of fewer than three corners is its points grown by grow_m."""
    xy = np.asarray(xy, dtype=float)
    if len(poly) < 3:
        P = np.asarray(poly, dtype=float).reshape(-1, 2)
        return np.min(np.linalg.norm(xy[:, None, :] - P[None, :, :], axis=2), axis=1) <= grow_m
    ok = np.ones(len(xy), dtype=bool)
    for a, b in zip(poly, poly[1:] + poly[:1]):
        e = b - a
        n = float(np.linalg.norm(e))
        if n > 0:
            ok &= (e[0] * (xy[:, 1] - a[1]) - e[1] * (xy[:, 0] - a[0])) / n >= -grow_m
    return ok


def hull_2d(points):
    """The convex hull of points in plan, counter-clockwise (Andrew's chain); fewer than three points come back
    as they are."""
    pts = sorted({(float(p[0]), float(p[1])) for p in points})
    if len(pts) < 3:
        return [np.array(p) for p in pts]

    def turn(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) > 1 and turn(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) > 1 and turn(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return [np.array(p) for p in lower[:-1] + upper[:-1]]


def inside_by(poly, xy):
    """How far inside a convex polygon (counter-clockwise, plan) a point lies (m): the least distance to an
    edge, negative outside; minus infinity for a polygon of fewer than three corners (nothing fits a line)."""
    if len(poly) < 3:
        return -math.inf
    least = math.inf
    for a, b in zip(poly, poly[1:] + poly[:1]):
        e = b - a
        n = float(np.linalg.norm(e))
        if n <= 0:
            continue
        least = min(least, float(e[0] * (xy[1] - a[1]) - e[1] * (xy[0] - a[0])) / n)
    return least


def follow_depth(points_T, R, tip_xy, poly, margin_m):
    """How far below the plane of an opening's rim the tip can go before a point of the hand's hull would stand
    over the rim instead of inside the opening (m; inf when the whole hand fits; negative when a part of the
    hull that is outside reaches below the tip itself). poly: the opening in plan (convex, counter-clockwise);
    tip_xy: where the tip stands over it; margin_m: the room every hull point keeps to the rim."""
    W = (np.asarray(R, dtype=float) @ np.asarray(points_T, dtype=float).T).T    # hull offsets from the tip, base frame
    low = math.inf
    for w in W:
        if inside_by(poly, (tip_xy[0] + w[0], tip_xy[1] + w[1])) < margin_m:
            low = min(low, float(w[2]))
    return low


def look_ray_bearing(ex, look_id, point):
    """The horizontal unit vector along which the camera of look_id saw point (from the lens toward it), or
    None: a scene look's camera pose (the scene camera package's record), else the wrist camera at the look's
    joint reading."""
    if not look_id:
        return None
    o = None
    try:
        sc = ex.state.get("scene_camera") or {}
        rec = (sc.get("looks") or {}).get(look_id)
        if rec is not None:
            from rcpx.r2 import scene_camera as SC
            cam = SC._cam(ex, rec["camera"])
            o = np.asarray(cam.pose()["position_mm"], dtype=float) / MM
        elif look_id in getattr(ex, "_looks", {}):
            q = ex._looks[look_id]["q"]
            o, _ = ex.model.pixel_ray(q, 0.0, 0.0)
            o = np.asarray(o, dtype=float)
    except Exception:
        o = None
    if o is None:
        return None
    h = horizontal(np.asarray(point, dtype=float) - o)
    return h if float(np.linalg.norm(h)) > 0 else None


def _stamp(corners, step):
    """(the cells a footprint covers as a mask, rows along y and columns along x, its middle cell the footprint's
    own origin; the mask's half size in cells). corners: k x 2 (m, plan); a cell counts when its middle lies in
    the footprint's hull or within half a cell of it."""
    c = np.asarray(corners, dtype=float)
    a = int(math.ceil(float(np.max(np.abs(c))) / step)) + 1
    g = np.arange(-a, a + 1) * step
    X, Y = np.meshgrid(g, g)
    inside = inside_grown(hull_2d(c), np.column_stack([X.ravel(), Y.ravel()]), step * HALF)
    return inside.reshape(X.shape).astype(np.uint8), a


def free_spot(points, at_xy, half_m, level_z, bodies, grow_m, step_m, toward_xy=None, most=False, across=None):
    """Where a held thing fits into the opening a look shows (pure geometry on a level grid; nothing moves).

    points: n x 3 (base frame, m), what the look shows of the world; at_xy: [x, y], where the thing's middle is
    planned; half_m: the half width the thing surely covers about its middle; level_z: what the look shows
    higher than this is in the thing's way; bodies: [(name, corners k x 2 in plan relative to the thing's
    middle, level)]: what the look shows higher than a body's level is in that body's way; grow_m: what the
    plan may be off in plan (the room wanted); step_m: the least cell the look's own error allows; toward_xy: a
    point on the inner side of the opening (where the pointed ray lands), for a planned spot that stands on a
    rim; across: how many points the look shows across (the grid holds no more cells across than that: a finer
    grid holds nothing the look shows).

    The room of a spot is the least distance in plan from the thing's footprint and from each body's footprint
    to what is in its way (under 0 where a footprint stands over it: by how deep). The opening is the spots
    joined to the planned one on the grid without the thing's middle crossing what is in its way: a spot beyond
    a rim is another place. It is closed where no spot of it lies on the grid's own edge: the look shows
    something all around it. The search goes no further than gets a footprint clear, by the room wanted, of
    two walls that meet in a corner (the thing's half width and that room, along each; from the nearest free
    spot on the pointed ray's side when the planned spot stands on a rim); in a closed opening where nothing
    that near has the room wanted, the whole opening is searched.
    - the planned spot has the room wanted: None (nothing changes);
    - else the nearest spot of the opening in that search that has it, no further than the room that is
      missing at the planned spot and one cell: the least shift that can give it, straight away from one wall
      ("nearest");
    - else (the opening is tighter than the thing and its room; or most: a way down ended on something the plan
      did not expect) the spot with the most room in that search ("middle"), when it has more room than the
      planned spot by more than a cell. Of the spots within a cell of the most room:
      in a closed opening the one nearest the middle of the opening (a thing between two walls has the same
      room all along them, and the near end of that stretch is where a third wall stands as near: there it
      would hang on three things, in the middle on two);
      in an opening the look does not show closed the nearest to the planned spot, and only when the search's
      edge, further along the way there, has less room (room that stands as high at the edge says the look
      shows no far side: nothing says the opening goes on);
    - else None.
    Returns None or {"xy", "moved_m", "room_m", "room_at_m", "how", "by": what has the least room at the planned
    spot (a body's name, or "the held thing"), "step_m", "closed"}."""
    try:
        import cv2
    except ImportError:
        return None
    P = np.asarray(points, dtype=float)
    at = np.asarray(at_xy, dtype=float)
    far = max([float(half_m)] + [float(np.max(np.linalg.norm(np.asarray(c, dtype=float), axis=1)))
                                 for _, c, _ in bodies])
    span = float(half_m) + float(grow_m)            # what gets a footprint that touches one wall clear of it
    wide = math.hypot(span, span)                   # ... and clear of two walls that meet in a corner
    half = wide + span + far + float(grow_m)
    step = max(float(step_m), np.finfo(float).eps)
    if across:
        step = max(step, (half + half) / float(across))
    n = int(math.ceil(half / step))
    ij = np.rint((P[:, :-1] - at) / step).astype(int) + n
    inside = np.all((ij >= 0) & (ij <= n + n), axis=1)
    ring = np.ones((len(NEXT), len(NEXT)), np.uint8)

    def way(level):
        """(the cells that hold something higher than level, grown by one cell: a wall the look samples cell
        by cell has no holes; the distance of every cell from them, m: under 0 inside them, by how deep)."""
        m = np.zeros((n + n + 1, n + n + 1), np.uint8)
        sel = inside & (P[:, -1] > level)
        m[ij[sel, 1], ij[sel, 0]] = 1
        m = cv2.dilate(m, ring)
        if not m.any() or m.all():
            return m, np.full(m.shape, -np.inf if m.any() else np.inf, dtype=np.float32)
        out = cv2.distanceTransform(1 - m, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        return m, (out - cv2.distanceTransform(m, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)) * np.float32(step)

    mask, dist = way(level_z)
    room = dist - np.float32(half_m)
    by = np.zeros(room.shape, dtype=int)
    for k, (name, corners, level) in enumerate(bodies):
        m_b, d_b = way(level)
        if not m_b.any():
            continue
        r_b = cv2.erode(d_b, _stamp(np.asarray(corners, dtype=float), step)[0])
        lower = r_b < room
        room = np.where(lower, r_b, room)
        by[lower] = k + 1
    g = (np.arange(n + n + 1) - n) * step
    X, Y = np.meshgrid(g, g)
    off = np.hypot(X, Y)
    free = dist > step
    seed = (n, n)
    if not free[seed]:          # the thing's middle would stand on a rim: the opening is on the pointed ray's side
        cand = free & (off <= span)
        if not cand.any():
            return None
        ref = np.zeros_like(at) if toward_xy is None else np.asarray(toward_xy, dtype=float) - at
        d_ref = np.where(cand, np.hypot(X - ref[0], Y - ref[1]), np.inf)
        seed = np.unravel_index(int(np.argmin(d_ref)), d_ref.shape)
    labels = cv2.connectedComponents(free.astype(np.uint8))[1]
    opening = labels == labels[seed]
    closed = not (opening[0].any() or opening[-1].any() or opening[:, 0].any() or opening[:, -1].any())
    reach = wide + float(off[seed])
    near = opening & (off <= reach)
    here = float(room[n, n])
    if here >= float(grow_m) and not most:
        return None
    good = near & (room >= float(grow_m)) & (off <= float(grow_m) - here + step)
    if good.any() and not most:
        how, pick = "nearest", np.unravel_index(int(np.argmin(np.where(good, off, np.inf))), off.shape)
    else:
        if not near.any():
            return None
        if closed and float(room[near].max()) < float(grow_m):
            near = opening              # nothing that near has the room: the whole opening the look shows closed
        best = float(room[near].max())
        top = near & (room >= best - step)
        rank = off
        if closed:                      # ... and of the spots with the most room the one in the opening's middle
            rank = np.hypot(X - float(X[near].mean()), Y - float(Y[near].mean()))
        how, pick = "middle", np.unravel_index(int(np.argmin(np.where(top, rank, np.inf))), off.shape)
        if float(room[pick]) <= here + step:
            return None
        if not closed:
            if float(off[pick]) > reach - step:
                return None
            # a middle has less room beyond it: where the search's own edge, further along the same way, still
            # stands within a cell of the most room, the look shows no far side there
            along = np.array([g[pick[1]], g[pick[0]]]) / float(off[pick])
            edge = np.clip(np.rint(along * reach / step).astype(int) + n, 0, n + n)
            if float(room[edge[1], edge[0]]) >= best - step:
                return None
    names = ["the held thing"] + [str(name) for name, _, _ in bodies]
    xy = at + np.array([g[pick[1]], g[pick[0]]])
    return {"xy": [float(xy[0]), float(xy[1])], "moved_m": float(off[pick]), "room_m": float(room[pick]),
            "room_at_m": here, "how": how, "by": names[int(by[n, n])], "step_m": step, "closed": bool(closed)}


def fan(first, step_rad):
    """Bearings (rad) in the order they are tried: each first bearing, then alternately to both sides in steps
    of step_rad out to a quarter turn."""
    n = max(1, int(math.ceil(QUARTER / max(step_rad, np.finfo(float).eps))))
    out = []
    for k in range(n + 1):
        for base in first:
            for sign in ((1,) if k == 0 else (1, -1)):
                b = base + sign * k * step_rad
                if all(abs(math.remainder(b - x, math.tau)) > step_rad / 2 for x in out):
                    out.append(b)
    return out


class SideMixin:
    def _pl_is_place(self, thing, target):
        tid = getattr(thing, "id", thing)
        return tid is not None and (tid == target.get("ref") or tid in (target.get("labels") or [])
                                    or tid == "support")

    def _pl_cover(self, target, plan, B):
        """The id of a measured thing the space book holds in the column over the place, between the top of the
        held thing once it stands there and the layer the carry would cross at (place_carry._pl_layer), or None:
        the way straight down onto the place is not known blocked. The column is as wide as the held thing: a
        thing that stands beside the place is a neighbour, not a cover; and a cover has an extent (a box, a
        plane): a single point that was touched is none."""
        if ((target.get("stands") or {}).get("answer")) is not None:
            self.state["place"].update(roof=None, side_way=None)
            return "what the look shows standing at the place"   # _pl_side_place answers it: nothing moves
        facts = self._pl_hold_facts()
        told = self._told()
        c = np.asarray(target["centre"], dtype=float)
        low = float(plan["base_z"]) + float(facts["below_m"]) + float(facts["above_m"])
        high, _ = self._pl_layer(told.tip, np.array([c[0], c[1], low]), None, B)
        self.state["place"].update(roof=None, side_way=None)
        seen_low = self._pl_top_band(plan, facts)      # the looks: from over the held thing's own top band
        if high - low <= 0:
            return self._pl_cover_seen(target, plan, B, seen_low, np.inf, facts)
        kind, thing = self._pl_kind(B, [c[0], c[1], low], [c[0], c[1], high], float(facts["half_m"]))
        if kind != "expected" or self._pl_is_place(thing, target):
            return self._pl_cover_seen(target, plan, B, seen_low, np.inf, facts)
        ext = getattr(thing, "extent", None) or {}
        if "box" not in ext and "plane" not in ext:
            return None           # a point that was touched is no roof: the way straight down runs under the guard
        tid = getattr(thing, "id", None)
        return str(tid) if tid is not None else "a thing above the place"

    def _pl_top_band(self, plan, facts):
        """The upper edge of the held thing's own top band once it stands at the place (m): the place's level and
        the thing's top over its bottom (the larger of how far it hangs under the tip and its height as the take's
        look showed it: a thing held near its bottom stands that much over the tip; with its height not seen, the
        most it reaches: its hang and how far it reaches up the fingers), raised by BAND of the take's stated
        error of its bottom under the tip. What the looks show under it in the thing's column stands where
        the thing itself would stand (a rim, a neighbour), never a roof it goes in under."""
        took = self.state["place"].get("taken") or {}
        tall = took.get("tall_m")
        if tall is not None and np.isfinite(tall):
            top = max(float(facts["below_m"]), float(tall))
        else:                         # its height not seen: the most it reaches, its hang and up the fingers
            top = float(facts["below_m"]) + float(facts["above_m"])
        rec = control_hold.held(self)
        ext = dict.get(rec, "extent") if rec is not None else None
        sig = ext.get("below_sigma_m") if isinstance(ext, dict) else None
        err = BAND * float(sig) if sig is not None and np.isfinite(sig) else 0.0
        return float(plan["base_z"]) + top + err

    def _pl_cover_seen(self, target, plan, B, low, high, facts):
        """A cover the LOOKS show over the place (rcpx/r2/place_roof.py), or None: anything they show in the held
        thing's column from over that thing's own top band once it stands there (low: _pl_top_band) upward (the
        hand comes down from over all of it); what they show under that edge stands where the thing would stand.
        With a roof seen the way in is planned here (state place.side_way). Where no side is free, the way from
        above would set the thing down on what the looks show over the place: no plan exists, so side_way
        holds the refusal no_side_reach with what blocks each side tried, and _pl_side_place returns it before
        anything moves."""
        st = self.state["place"]
        st["roof"], st["side_way"] = None, None
        err = target.get("sigma_xy")
        roof = place_roof.roof_over(self, target["centre"], float(facts["half_m"]) + (
            float(err) if err is not None and np.isfinite(err) else 0.0), low, high,
            float(np.asarray(target["centre"], dtype=float)[-1]))
        if roof is None:
            return None
        if roof["covers"] <= HALF:       # a bar or a handle crosses the place and covers no more than half of the
            target.setdefault("doubt", []).append({      # thing's column: no roof; the plain way, under the guard
                "what": f"something crosses over the place ({roof['text']}; it covers "
                        f"{roof['covers'] * 100:.0f} percent of the held thing's column)",
                "acted": "went the plain way, from above, with the contact stop armed", "would_remove": None})
            return None
        if roof["stands"] >= QUANT * QUANT and int(roof.get("under") or 0) < QUANT * QUANT:
            # what the looks show there rises from the place's own level and no look shows that level under it:
            # the top of a thing standing at the place, not a roof (a look that sees in under a roof's edge sees
            # the level there). A cover with the place's level seen under it is a roof however much stands
            # beside the place (a wall of the opening): it is never entered from above
            target.setdefault("doubt", []).append({
                "what": f"something stands at the place itself and rises into the held thing's column ({roof['text']}; "
                        f"{roof['stands']} points of the looks stand under the thing's top there, and no look shows "
                        "the place's own level under what covers it)",
                "acted": "went the plain way, from above, with the contact stop armed: no level way in can pass under "
                         "a thing that stands on the place's own level", "would_remove": None})
            return None
        st["roof"] = roof
        way = self._pl_side_plan(target, plan, None, roof["text"])
        if "refused" in way:
            st["roof"] = None
            way["refused"]["roof"] = {"look": roof["look"], "room_mm": round(roof["room_m"] * MM, 1),
                                      "points": roof["points"], "covers": float(roof["covers"]),
                                      "level_seen_under_cells": int(roof.get("under") or 0)}
            way["refused"]["why"] = (way["refused"]["why"] + "; the way from above would set the thing down on "
                                     "what the looks show over the place")
            way["refused"]["hint"] = "a place that is open from above, or a look from another side that shows a way in"
        st["side_way"] = way
        return roof["text"]

    def _pl_side_corners(self, held):
        """Every corner that moves with the hand, in the tool frame (m): the hand's hull and the held thing."""
        pts = [np.asarray(p, dtype=float) for p in hand_points(self)]
        for c in (dict.get(held, "extent_m") if held is not None else None) or []:
            pts.append(np.asarray(c, dtype=float))
        return np.asarray(pts, dtype=float).reshape(-1, 3)

    def _pl_side_rot(self, R_hold, e):
        """The hand rotation for entering along e, the held rotation turned about the vertical: a hand that
        points level (a side grasp) gets its pointing axis along e; a hand that points down gets its jaws'
        closing axis square to e, so that the open fingers leave along the thing's sides on the way back out."""
        R_hold = np.asarray(R_hold, dtype=float)
        ax = R_hold[:, 0]
        level = abs(float(ax[2])) <= float(np.linalg.norm(ax[:2]))
        ref = horizontal(ax) if level else horizontal(R_hold @ self._pl_jaws.axis_T)
        if float(np.linalg.norm(ref)) <= 0:
            return R_hold
        want = math.atan2(float(e[1]), float(e[0])) + (0.0 if level else QUARTER)
        d = want - math.atan2(float(ref[1]), float(ref[0]))
        if not level:
            d = math.remainder(d, math.pi)                    # a closing axis has no sign: the smaller turn
        Rz = np.array([[math.cos(d), -math.sin(d), 0.0], [math.sin(d), math.cos(d), 0.0], [0.0, 0.0, 1.0]])
        return Rz @ R_hold

    def _pl_side_plan(self, target, plan, intent, cover):
        """The corridor: {"e" (unit, the way in), "T_mouth", "T_in" (tip poses), "length_m", "bearing_deg",
        "unseen" (bool: part of the line is not seen), "tried"} or the refusal no_side_reach."""
        B = bounds.of(self)
        s = self._sense()
        held = self._pl_held()
        facts = self._pl_hold_facts()
        half = self._pl_half_width(held)
        hang = float(facts["below_m"])
        c = np.asarray(target["centre"], dtype=float)
        z_edge = float(plan["base_z"])
        off_T = np.asarray(facts["offset_T"], dtype=float)
        corners = self._pl_side_corners(held)
        reach = float(np.linalg.norm((np.asarray(s.tip) - c)[:2])) + half
        layer, _ = self._pl_layer(s.tip, [c[0], c[1], z_edge + hang], held, B)
        look_e = look_ray_bearing(self, target.get("look"), c)
        radial = horizontal(c)
        here = horizontal(c - np.asarray(s.tip, dtype=float))       # from where the hand stands toward the place
        first = [math.atan2(float(v[1]), float(v[0])) for v in (look_e, here, radial) if v is not None
                 and float(np.linalg.norm(v)) > 0]
        if not first:
            first = [0.0]
        step = math.atan2(half, max(reach, half))
        tried, why, whys = 0, "no bearing was free of known things", []
        roof = self.state["place"].get("roof")      # a roof the looks show: the looks stand in for the book
        for b in fan(first, step):
            if tried:
                whys.append(f"bearing {math.degrees(fan_b):.0f} deg: {why}")
            fan_b = b
            tried += 1
            e = np.array([math.cos(b), math.sin(b), 0.0])
            R = self._pl_side_rot(s.R, e)
            # the level line runs over the surface the thing goes onto: its bottom keeps the band and, above
            # it, the clearance the bounds derive for a level move at the free speed (what the tip may be off
            # its line by); the set-down then lowers that much
            z_bottom = z_edge + float(B.clearance(float(B.v_free(s.q, e))))
            # where the thing's middle is relative to the tip at this hand rotation, and the tip over the place
            obj = R @ off_T
            down = float((R @ np.array([1.0, 0.0, 0.0]))[2]) < 0
            z_tip = z_bottom + hang if down else z_bottom + (hang + float(facts["above_m"])) / 2 - float(obj[2])
            p_in = np.array([c[0] - obj[0], c[1] - obj[1], z_tip])
            # (1) the mouth: the first place out along -e where the hand can come down from above
            mouth, d = None, half
            while d <= reach:
                p = p_in - e * d
                k, th = self._pl_kind(B, p, [p[0], p[1], max(float(s.tip[2]), p[2], layer)], half)
                free = k != "expected" or self._pl_is_place(th, target)
                if free and roof is not None:       # ... and no look shows anything in the hand's column there
                    W = (R @ corners.T).T
                    free = place_roof.column(self, p, float(np.max(np.linalg.norm(W[:, :2], axis=1))),
                                             p[2] + float(W[:, 2].min()), np.inf) is None
                if free:
                    mouth = p
                    break
                d += half
            if mouth is None:
                why = "no place outside the cover where the hand can come down was found along the way in"
                continue
            # (2) every corner of the hand and the thing, level from the mouth to over the place
            blocked, unseen = None, False
            for h in corners:
                w = R @ h
                k, th = self._pl_kind(B, mouth + w, p_in + w, 0.0)
                if k == "expected" and not self._pl_is_place(th, target):
                    blocked = getattr(th, "id", None) or "a known thing"
                    break
                unseen = unseen or k == "unseen"
            if blocked is not None:
                why = f"the hand or the thing would enter the band of {blocked} on the way in"
                continue
            hit = None if roof is None else place_roof.swept(self, mouth, p_in, (R @ corners.T).T)
            if hit is not None:
                why = (f"look {hit['look']} shows something {hit['at_m'] * MM:.0f} mm along the way in, "
                       f"{hit['over_m'] * MM:.0f} mm over the bottom of what the hand and the thing sweep")
                continue
            # (3) this arm holds the hand at both ends, and over the mouth at the height it travels at
            q_in, _ = self._pl_solve(p_in, R, R[:, 0], [s.q, self.model.q_home])
            q_out, _ = (None, None) if q_in is None else self._pl_solve(mouth, R, R[:, 0], [q_in, s.q])
            if q_in is None or q_out is None:
                why = "no arm pose holds the hand level at both ends of the way in"
                continue
            hit = None if roof is None else place_roof.arm_in_looks(self, [q_out, (q_out + q_in) / 2, q_in])
            if hit is not None:      # the arm above the hand goes in under the roof too
                why = (f"the arm's own body would stand in what look {hit['look']} shows ({hit['points']} points) at "
                       f"the {('mouth', 'middle', 'end')[hit['at']]} of the way in")
                continue
            high, _ = self._pl_layer(s.tip, mouth, held, B)
            q_top, _ = self._pl_solve([mouth[0], mouth[1], high], R, R[:, 0], [s.q, q_out])
            if q_top is None:
                why = "no arm pose holds the hand over the mouth of the way in at the height it travels at"
                continue
            return {"e": e, "T_mouth": pose(mouth, R), "T_in": pose(p_in, R), "q_mouth": q_out, "q_in": q_in,
                    "length_m": float(np.linalg.norm(p_in - mouth)), "bearing_deg": round(math.degrees(b), 1),
                    "unseen": unseen, "tried": tried, "from_look": look_e is not None}
        whys.append(f"bearing {math.degrees(fan_b):.0f} deg: {why}")
        # the reasons by how many bearings each ended, then by their text (never a set's order)
        told = reasons_by_count(whys)
        return refusal("no_side_reach", f"the place stands under {cover}, " + (
            "a thing this session measured" if roof is None else "seen, not measured") + ", and no level way in was "
            f"found over {tried} bearings: " + "; ".join(f"{n} of them: {w}" for n, w in told[:3])
            + f" (first tried: {whys[0]})", "a place that is open from above")

    def _pl_answer_place(self, target):
        """A place the look shows taken (place_spot.settle: no spot on the level the pixel lies on holds the held
        thing free of what stands there) is ANSWERED: stopped "answered", nothing moved by the place, the thing still
        held, "standing": what the look shows there with its pixel, and how a next call names free room."""
        rec = target["stands"]
        ans = rec["answer"]
        near = [dict(n, what="something standing at the place") for n in ans.get("standing") or []]
        said = (f"Did not set it down: {rec.get('said')}. It is still held; the place moved nothing (0 control "
                "periods). A pixel on free room names a place there.")
        return {"said": said, "stopped": "answered", "held": self._pl_held() is not None, "placed": False,
                "near": near, "answer": {k: ans.get(k) for k in ("why", "look", "footprint_mm", "level_mm", "free")},
                "stands": {k: v for k, v in rec.items() if k not in ("said", "answer")}}

    def _pl_side_place(self, target, plan, intent, cover, t0):
        """The whole side entry. Returns the set-down's result with side {bearing_deg, length_mm, cover}; a place
        the look shows taken is answered before anything moves (_pl_answer_place)."""
        if ((target.get("stands") or {}).get("answer")) is not None:
            return self._pl_answer_place(target)
        way = self.state["place"].get("side_way") or self._pl_side_plan(target, plan, intent, cover)
        if "refused" in way:
            return way
        held = self._pl_held()
        r = self._pl_travel(way["T_mouth"] if self.model.n >= FULL_HAND_JOINTS else way["q_mouth"], "carry", held=held,
                            target=target.get("ref"), intent=intent)
        if "refused" in r:
            return self._pl_refuse_motion(r, "carrying the thing to the open side")
        rec = phase(self, "carry", "carried it to the open side of the cover" if r["stopped"] == "goal" else
                    "the carry to the open side stopped", t0, [r])
        rec["strike"] = r.get("strike")            # every leg states its own figure (severe 3 (c))
        if r["stopped"] != "goal":
            return self._pl_stopped(r, "carrying it to the open side", {"bounds": r.get("bounds")})
        t1 = self.sim_time()
        s = self._told()
        r2 = self._pl_line(way["T_in"][:3, 3] - s.tip, "carry", target=target.get("ref"), intent=intent)
        if "refused" in r2:
            return self._pl_refuse_motion(r2, "entering under the cover")
        rec = phase(self, "enter", f"entered level under the cover, {way['length_m'] * MM:.0f} mm along bearing "
                                   f"{way['bearing_deg']:g} deg" if r2["stopped"] == "goal" else
                    "the way in under the cover stopped", t1, [r2])
        rec["strike"] = r2.get("strike")
        if r2["stopped"] != "goal":
            return self._pl_stopped(r2, "entering under the cover", {"bounds": r2.get("bounds")})
        t2 = self.sim_time()
        mouth = np.asarray(self._told().tip, dtype=float) - way["e"] * way["length_m"]
        out = self._pl_set_down(target, plan, intent, t2, rise=False)
        if "refused" in out or out.get("stopped") == "dropped":
            return out
        t3 = self.sim_time()
        s = self._told()
        back = mouth - np.asarray(s.tip, dtype=float)
        back[2] = 0.0
        r3 = self._pl_line(back, "retreat", away_from=[float(x) for x in target["centre"]], intent=intent)
        rec = phase(self, "withdraw", "withdrew the same way" if "refused" not in r3 and r3.get("stopped") == "goal"
                    else "the way back out stopped", t3, [r3])
        rec["strike"] = r3.get("strike") if isinstance(r3, dict) else None
        seen = self.state["place"].get("roof")
        if seen is not None:      # what showed the roof (stage C): the look, and how far over the thing it stands
            out["roof"] = {"look": seen["look"], "room_mm": round(seen["room_m"] * MM, 1), "points": seen["points"]}
        out["side"] = {"cover": str(cover), "bearing_deg": way["bearing_deg"],
                       "length_mm": round(way["length_m"] * MM, 1), "bearings_tried": way["tried"],
                       "way_unseen": way["unseen"], "from_look": way["from_look"],
                       "withdrew": "refused" not in r3 and r3.get("stopped") == "goal",
                       "tip_mm": _mm(self._sense().tip)}
        out["said"] = out["said"].replace("Set the thing down:", "Set the thing down under the cover, entering "
                                          "through its open side:")
        out["said"] = out["said"].rstrip() + (f" Entered level along bearing {way['bearing_deg']:g} deg under "
                                              f"{cover}.")
        return out
