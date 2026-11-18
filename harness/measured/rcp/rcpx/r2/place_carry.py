# SPDX-License-Identifier: AGPL-3.0-only
"""carry of package place (N4), round six (the design notes, package PLACE; contracts C8, C10, C12): the way a
held thing goes to its place (the place targets are rcpx/r2/place_target.py).

What is held is the core's one record (rcpx/control_hold.py): it knows the load, its guard is the hold's level, a
lost hold ends a move with its own words. This file keeps no watch of the jaws and sets no squeeze: both are the
core's. It writes no guard, no cap and no speed anywhere.

The way: the carry is ONE stream: up to the layer, across, down to the carry height. The way down of a set down
is a motion of its own from there, an expected contact (C9) with its own price. No leg is left out unless it is
shorter than the tip's own resolution (place_motion._pl_res: a figure that does not follow the session's contact
level); the way down is made whenever the plan has an end height under the carry height. The layer clears what
the looks of the session show near the way, what the world holds there and the place itself, each by what
reaches lowest under the tip (the held thing, or the hand's own lowest point) and the room the bounds derive;
where the arm does not reach that high over both ends it is the highest layer in reach, and the receipt says
so. An arm whose legs are joint legs (too few joints to hold a whole pose along a line) leaves the level line
between a leg's ends: the layer is raised by that dip. The carry height stands over the top of what is near the
place, and never under the stated support. A held thing that stands on something where the place begins (the
still arm reads more than the hold explains) is risen first, that contact behind the hand. A stop on the way is
returned as it is: nothing here falls back to another primitive. A stop on a contact with the thing in the hand
states after the stop ONE figure that bounds every pair of bodies the contact loaded (met_held: the landing's own
law, the arm's figure plus the pads' rise, never under the strike of a gap the jaws read closing while it
carried); before the motion the carry's figure is the same law on the plan's fastest sample (planned_held), so the
pads' rise is in it. A held stream that stops on a contact or on the cap reads the hold's load at rest inside its own
phase before any word (_pl_rest_read): the receipt says "held" only where the weight stands in the hand. Nothing here
reads ground truth or names a robot.
"""

import math

import numpy as np

from rcpx import bounds
from rcpx import bounds_laws as laws
from rcpx import bounds_ratios as ratios
from rcpx import control_hold_pads
from rcpx.context import refusal
from rcpx.control_base import _mm
from rcpx.r2.place_hand import finite
from rcpx.r2.place_land_watch import LandWatch
from rcpx.r2.place_motion import UP, horizontal, pose
from rcpx.r2.place_side import hand_hulls, hand_points, hull_2d, inside_grown

MM = 1000.0
DOWN = -UP
LOOK_AGAIN = {"tool": "look", "args": {}, "periods": 0}


def met_held(r, pads, read):
    """A carry stream that stopped on a contact with the thing in the hand:
    its figure after the stop bounds EVERY pair of bodies that contact loaded, the held thing on what it met and each
    pad on the thing over the squeeze that stood, by the landing's own law (rcpx/control_hold_pads.py): rise(), the
    arm's figure along the travel at the speed the contact was met at plus the pads' lever term, and caught(), a
    thing that moved between the pads and was caught again, from the fastest gap closing the jaws' own reading
    showed over the stream. The arm's figure is the largest of what the stop states (ran.peak_n), what the arm read
    at the stop and what it read over the stream (place_land_watch.LandWatch); the speed is the larger of the
    stop's own measure and the hand's speed in the period before a load first rose on the arm's reading. r:
    the stream's result (changed in place: strike.ran); pads: the hold's pads numbers (control_hold_pads
    .of_record); read: the watch's reading. Nothing changes where the stream ended at its goal, where the stop
    states no figure of the arm's, or where the jaws state no numbers."""
    if not isinstance(r, dict) or str(r.get("stopped")) not in ("contact", "cap") or pads is None:
        return
    st = r.get("strike")
    if not isinstance(st, dict) or not isinstance(st.get("ran"), dict):
        return
    ran, w = dict(st["ran"]), read if isinstance(read, dict) else {}
    arms = [float(x) for x in (ran.get("peak_n"), ran.get("read_n"), w.get("read_n")) if finite(x)]
    speeds = [float(x) for x in (ran.get("v_m_s"), (w.get("met") or {}).get("v_m_s")) if finite(x)]
    if not finite(ran.get("peak_n")) or not speeds:
        return
    arm, v = max(arms), max(speeds)
    one, lever = control_hold_pads.rise(pads, arm, v)
    again = control_hold_pads.caught(pads, float(w.get("gap_m_s") or 0.0))
    figure = max(float(one or 0.0), float(again))
    ran.update(arm_n=round(arm, 1), v_m_s=v, pads_rise_n=None if lever is None else round(float(lever), 1))
    if figure > float(ran["peak_n"]):
        ran["peak_n"] = round(figure, 1)
        ran["note"] = (str(ran.get("note") or "") + "; with the thing in the hand: one figure for every pair this "
                       "contact loaded (the thing on what it met, each pad on the thing over the squeeze that stood): "
                       "the arm's figure along the travel plus the pads' rise by the jaws' own stop slope"
                       + ("; the jaws' own reading showed the gap close (the thing moved between the pads): never "
                          "under the strike of that closing" if again > 0.0 else "")).lstrip("; ")
    if again > 0.0:
        ran["caught_n"] = round(again, 1)
    st["ran"] = ran
    r["strike"] = st


def planned_held(r, pads):
    """The carry's figure BEFORE the motion with the thing in the hand: every leg of the carry
    can bring the held thing onto something, and such a contact loads each pad on the thing too, so the figure the
    plan states for its fastest sample is the landing's one figure law (rcpx/control_hold_pads.py rise(): the arm's
    figure along the travel at that speed plus the pads' lever term), as strike.fast_n beside the arm's own peak_n.
    r: the stream's result (changed in place); pads: the hold's pads numbers. Nothing changes where the stream
    states no figure or speed of its fastest sample, or where the jaws state no numbers."""
    if not isinstance(r, dict) or pads is None or not isinstance(r.get("strike"), dict):
        return
    st = dict(r["strike"])
    for where in (r, st):           # a band block a stream states (C15 (5)): its two figures by the same law
        b = where.get("band")
        if isinstance(b, dict):
            b = dict(b)
            for key, v_key in (("fast_n", "fast_mm_s"), ("arrive_n", "arrive_mm_s")):
                if finite(b.get(key)) and finite(b.get(v_key)):
                    got, _ = control_hold_pads.rise(pads, float(b[key]), float(b[v_key]) / MM)
                    if got is not None:
                        b["arm_" + key], b[key] = b[key], round(float(got), 1)
            where["band"] = b
    if not finite(st.get("peak_n")) or not finite(st.get("v_m_s")):
        r["strike"] = st
        return
    one, lever = control_hold_pads.rise(pads, float(st["peak_n"]), float(st["v_m_s"]))
    if one is not None:
        st.update(fast_n=round(float(one), 1), arm_fast_n=round(float(st["peak_n"]), 1),
                  fast_pads_rise_n=None if lever is None else round(float(lever), 1),
                  fast_figure="before the motion, with the thing in the hand: one figure for every pair a contact "
                              "of this carry loads (the thing on what it meets, each pad on the thing over the "
                              "squeeze that stood): the arm's figure along the travel at the plan's fastest sample "
                              "plus the pads' rise, sqrt(arm figure x the jaws' stop slope x speed)")
    r["strike"] = st


def segment_distance(xy, a, b):
    """The distance in plan of points xy (n x 2) from the stretch a -> b."""
    ab = np.asarray(b, dtype=float) - np.asarray(a, dtype=float)
    L2 = float(ab @ ab)
    k = np.zeros(len(xy)) if L2 <= 0 else np.clip((xy - a) @ ab / L2, 0.0, 1.0)
    return np.linalg.norm(xy - (np.asarray(a, dtype=float) + k[:, None] * ab), axis=1)


# ---------------------------------------------------------------- carry
class CarryMixin:
    def _pl_object_centre(self, s, facts=None):
        """Where the held thing's middle is (between the jaws), from the tip pose (m)."""
        facts = facts or self._pl_hold_facts() or {}
        return s.tip + s.R @ np.asarray(facts.get("offset_T", np.zeros(3)), dtype=float)

    def _pl_room(self, B, q, u):
        """The room a moving hand keeps over what it crosses (m): the bounds' own clearance at the speed of the
        stretch (what the tip may be off its line by, and what it travels in one control period)."""
        return float(B.clearance(float(B.v_free(q, u))))

    def _pl_skip(self):
        """(xy, radius) of where the thing that was taken stood when the look was made, or None: what a look shows
        there is the taken thing itself. Its seen middle and half its seen width when this call's take saw it
        whole; else the pick point and the jaws' widest opening (nothing wider was taken)."""
        st = self.state["place"]
        took, picked = st.get("taken"), st.get("picked_at")
        if took:
            return np.asarray(took["centre_xy"], dtype=float), float(took["wide_m"]) / 2 + self._pl_jaws.max_m / 2
        if picked is not None:
            return np.asarray(picked[:2], dtype=float), float(self._pl_jaws.max_m)
        return None

    def _pl_seen_top(self, a_xy, b_xy, half_m):
        """(height, look id) of the highest surface the newest look with depth shows within half_m of the way
        a -> b in plan, or (None, None): the robot's own body is left out by the look itself, and so is what
        stood where the taken thing stood (_pl_skip)."""
        pts, look, sigma = self._pl_look_points()
        if pts is None:
            return None, None
        near = segment_distance(pts[:, :2], np.asarray(a_xy, dtype=float), np.asarray(b_xy, dtype=float)) <= half_m
        skip = self._pl_skip()
        if skip is not None:
            near &= np.linalg.norm(pts[:, :2] - skip[0], axis=1) > skip[1]
        if not near.any():
            return None, look
        from rcpx.r2 import see_cloud
        return float(see_cloud.top_of(pts[near][:, 2], max(sigma, np.finfo(float).eps))), look

    def _pl_look_points(self):
        """(points n x 3 of the newest look with depth, its id, the depth's error there) or (None, None, 0): what
        that look shows of the world; the robot's own body is left out by the look itself."""
        try:
            from rcpx.r2 import see_spot
        except ImportError:
            return None, None, 0.0
        scene = (self.state.get("scene_camera") or {}).get("looks", {})
        order = list(scene.items())[::-1] + list((getattr(self, "_looks", None) or {}).items())[::-1]
        for key, rec in order:
            try:
                got = see_spot.cloud_of(self, rec)
            except Exception:
                got = None
            if got is None or not got[2].any():
                continue
            pts = got[1][got[2]]
            rng = float(np.median(np.linalg.norm(pts - got[0].pos, axis=1)))
            return pts, str(key), float(see_spot._sigma_at(rec, rng, rng / got[0].f)[0])
        return None, None, 0.0

    def _pl_hand_stop(self, tip_xy, R, grow_m):
        """(the lowest tip height at which no body of the hand stands on what the newest look with depth shows
        under it, that body's name, the look's id), or (None, None, None) when no look shows anything under the
        hand. tip_xy: where the tip stands in plan; R: the hand's rotation; grow_m: what a body's footprint is
        widened by (what the hand may be off in plan). What stood where the taken thing stood is left out."""
        pts, look, sigma = self._pl_look_points()
        if pts is None:
            return None, None, None
        hulls = hand_hulls(self)
        if not hulls:
            return None, None, None
        from rcpx.r2 import see_cloud
        R = np.asarray(R, dtype=float)
        reach = max(float(np.max(np.linalg.norm((R @ h.T).T[:, :2], axis=1))) for _, h in hulls) + grow_m
        near = np.linalg.norm(pts[:, :2] - np.asarray(tip_xy, dtype=float), axis=1) <= reach
        skip = self._pl_skip()
        if skip is not None:
            near &= np.linalg.norm(pts[:, :2] - skip[0], axis=1) > skip[1]
        P = pts[near]
        best = (None, None)
        for name, h in hulls:
            w = (R @ h.T).T
            under = inside_grown(hull_2d(w[:, :2] + np.asarray(tip_xy, dtype=float)), P[:, :2], grow_m)
            if under.any():      # this body's lowest point on the top of what is seen under its footprint
                z = float(see_cloud.top_of(P[under][:, 2], max(sigma, np.finfo(float).eps))) - float(w[:, 2].min())
                if best[0] is None or z > best[0]:
                    best = (z, name)
        return best[0], best[1], look

    def _pl_leg(self, kind, p, R, q_seed, **more):
        """One leg to the tip pose (p, R): a straight tip line for an arm that can hold a whole pose along it; for
        an arm with fewer joints a joint leg to the same point, the tool pointing as R points (None when it
        cannot take that point)."""
        if self.model.n >= len(p) + len(p):
            return {"kind": kind, "to": pose(p, R), "line": True, **more}
        q, _ = self._pl_solve(p, R, R[:, 0], [q_seed])
        return None if q is None else {"kind": kind, "to": q, **more}

    # ---------------------------------------------------------------- what stands near a way
    def _pl_goal_pose(self, goal):
        g = np.asarray(goal, dtype=float)
        if g.ndim > 1:                    # a tip pose (a matrix); else a joint vector
            return g, None
        p, R = self.model.fk_tip(g)
        return pose(p, R), g

    def _pl_known_top(self, a_xy, b_xy, corridor_m, B):
        """(height, names): the highest thing the world holds within corridor_m of the way a -> b in plan,
        raised by its own band, or (None, [])."""
        top, names = None, []
        picked = self.state["place"].get("picked_at")
        facts = self._pl_hold_facts()
        half = 0.0 if facts is None else float(facts["half_m"])
        ab = np.asarray(b_xy, dtype=float) - np.asarray(a_xy, dtype=float)
        L2 = float(ab @ ab)
        for e in self.world.query():
            if e.get("stale") or e.get("point_mm") is None:
                continue
            p = np.asarray(e["point_mm"], dtype=float) / MM
            th = self._pl_thing(B, e.get("id"), p)
            sig = float(getattr(th, "sigma_p", None) or 0.0)
            if picked is not None and float(np.linalg.norm(p - np.asarray(picked))) <= half + sig:
                continue                            # the held thing itself, where it stood
            k = 0.0 if L2 <= 0 else min(1.0, max(0.0, float((p[:2] - a_xy) @ ab) / L2))
            z = float(p[2]) + self._pl_band(B, th)
            if float(np.linalg.norm(a_xy + k * ab - p[:2])) <= corridor_m + sig and (top is None or z > top):
                top = z
                names.append(f"{e.get('label')} ({e.get('kind')})")
        return top, names[-3:]

    def _pl_thing(self, B, ref, point=None):
        """The bounds' record of a thing (world id, label or point), or None when the bounds hold none."""
        for r in (ref, point):
            try:
                th = None if r is None else B.thing(r)
            except Exception:
                th = None
            if th is not None:
                return th
        return None

    def _pl_band(self, B, thing):
        """The band of a thing (m): BAND x its sigma by the bounds' law; 0 when its sigma is not known."""
        sig = getattr(thing, "sigma_p", None) if thing is not None else None
        return float(laws.band(float(sig))) if sig else 0.0

    def _pl_kind(self, B, p0, p1, radius):
        """(kind word, thing or None) of the space between two points; a book that cannot say answers unseen."""
        try:
            k = B.space.kind(np.asarray(p0, dtype=float), np.asarray(p1, dtype=float), float(radius))
        except Exception:
            return "unseen", None
        if not isinstance(k, (tuple, list)) or not k:
            return "unseen", None
        return str(k[0]), (k[1] if len(k) > 1 else None)

    @staticmethod
    def _pl_top_of(thing):
        """The height of a thing's top (m), from its box or its point; None for a plane or a thing with neither."""
        ext = getattr(thing, "extent", None) or {}
        if "box" in ext:
            return float(np.asarray(ext["box"][1], dtype=float)[2])
        pt = getattr(thing, "point", None)
        return None if pt is None else float(np.asarray(pt, dtype=float)[2])

    def _pl_layer(self, p0, p1, held, B, more=0.0, over_z=None):
        """(z, names): the tip height of the level stretch of a way p0 -> p1: no lower than both ends, over
        what the world holds near the way and over each thing the space book says the stretch would meet (its
        top + its band + the bounds' clearance at the free speed + what hangs below the tip + more)."""
        p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
        half, below = self._pl_half_width(held), self._pl_below_tip(held)
        q = self._sense().q
        way = horizontal(p1 - p0)
        clear = float(B.clearance(float(B.v_free(q, way if float(np.linalg.norm(way)) > 0 else UP)))) + float(more)
        z = max(float(p0[2]), float(p1[2]))
        level = float(np.linalg.norm((p1 - p0)[:2])) > self._pl_res(B, q)
        if over_z is not None and level:        # a way across comes in over the place it goes to, not into its edge
            z = max(z, float(over_z) + clear + below)
        top, names = self._pl_known_top(p0[:2], p1[:2], half + clear, B)
        if top is not None:
            z = max(z, top + clear + below)
        while True:
            hit = None
            for level in (z - below, z):
                k, th = self._pl_kind(B, [p0[0], p0[1], level], [p1[0], p1[1], level], half)
                if k == "expected" and self._pl_top_of(th) is not None:
                    hit = th
                    break
            if hit is None:
                return z, names
            z_new = self._pl_top_of(hit) + self._pl_band(B, hit) + clear + below
            if z_new <= z:
                return z, names
            z = z_new
            names.append(str(getattr(hit, "id", None) or "a seen thing"))

    def _pl_route(self, s, T_goal, held, B, intent=None):
        """The via points (tip, base frame m) of the way from the present tip to the goal's: rise to the layer of
        _pl_layer, across, down. Returns (points, words)."""
        p0, p1 = np.asarray(s.tip, dtype=float), np.asarray(T_goal[:3, 3], dtype=float)
        z, names = self._pl_layer(p0, p1, held, B, more=float((intent or {}).get("clearance_m") or 0.0),
                                  over_z=(intent or {}).get("over_z_m"))
        text = "over what is known near the way" + (f" ({', '.join(names[-3:])})" if names else "")
        return [np.array([p0[0], p0[1], z]), np.array([p1[0], p1[1], z])], text

    def _pl_legs(self, s, T_goal, q_goal, kind, held, target, intent, B):
        """The legs of one stream to the goal: rise (when the layer is higher), across (turning to the goal's
        hand rotation), down (to the goal itself, toward target)."""
        pts, text = self._pl_route(s, T_goal, held, B, intent)
        p_now, R_now = np.asarray(s.tip, dtype=float), np.asarray(s.R, dtype=float)
        R_goal, p_goal = T_goal[:3, :3], T_goal[:3, 3]
        least = self._pl_res(B, s.q)       # a via point nearer than the tip resolves is no leg of its own
        legs, names = [], []
        for p in pts:
            if float(np.linalg.norm(p - p_goal)) <= least:
                continue
            if float(np.linalg.norm(p - p_now)) <= least:
                continue
            level = abs(float(p[2] - p_now[2])) <= least
            R_leg = R_goal if level else R_now
            legs.append({"kind": kind, "to": pose(p, R_leg), "line": True, "intent": self._pl_core_intent(intent)})
            names.append("across" if level else "rise")
            p_now, R_now = p, R_leg
        last = {"kind": kind, "to": T_goal if q_goal is None else q_goal, "line": True, "target": target,
                "intent": self._pl_core_intent(intent)}
        level = abs(float(p_goal[2] - p_now[2])) <= least
        legs.append(last)
        names.append("across" if level else "down")
        return legs, names, text

    def _pl_travel(self, goal, kind, held=None, target=None, looks=None, intent=None):
        """One stream from where the hand stands to goal (a 4x4 tip pose in the base frame, or a joint vector):
        rise, across, down, each leg of the given kind. With nothing held, a contact stop on the rise or the
        way across stored a thing: the travel is planned again by the book ONCE (a higher layer goes over it);
        a second stop, or a plan that is no higher, is returned as it is. With something held every stop is
        returned (a push on a carried thing is never planned around). Returns the stream's result with route
        (words) or a refusal."""
        B = bounds.of(self)
        T_goal, q_goal = self._pl_goal_pose(goal)
        first = None
        while True:
            s = self._sense()
            legs, names, text = self._pl_legs(s, T_goal, q_goal, kind, held, target, intent, B)
            tops = [float(np.asarray(leg["to"])[2, 3]) for leg in legs if np.asarray(leg["to"]).ndim > 1]
            layer = max(tops + [float(T_goal[2, 3])])
            if first is not None and layer <= first[0]:
                return first[1]
            r = self._pl_stream(legs, names, held=held, looks=looks)
            if "refused" in r:
                return r
            r["route"] = text
            if held:
                self._pl_rest_read(r)     # a held stream that stopped on a contact: the load is read at rest
            if first is not None:
                r["replanned"] = "a stop on the way stored a thing: the way was planned again over it"
                return r
            if held or not (r["stopped"] == "contact" and r.get("leg_name") in ("rise", "across")):
                return r                  # with something held a contact is never planned around: it is returned
            first = (layer, r)

    def _pl_stands(self, told, facts):
        """The force (N) the still arm reads over what the hold explains where a place begins, when that says the
        held thing stands on something (more than the arm can hide at rest and more than GIVE of the held load
        is carried from outside); else None. Read once, before anything moves."""
        rec = self._pl_held()
        if rec is None:
            return None
        B, s = bounds.of(self), self._sense()
        f = float(s.Fmag)               # the estimate at rest: the held load is taken off it by the record's mass
        weight = dict.get(rec, "weight_n")
        if weight is None or not self._still(s, self._still_band(s)):
            return None                 # no load read yet, or a hand that still moves: the reading says nothing
        return f if f > max(float(B.f_thr(0, 0)), ratios.GIVE * float(weight)) else None

    def _pl_dip(self, q_a, q_b, room, below, thing_m=0.0, half_m=0.0):
        """How far, along the joint leg q_a -> q_b, the lowest point that moves with the hand (its own hull, or
        the held thing: thing_m below the tip along the tool, half_m to each side of it) sinks under where the
        plan puts it: the straight line between the leg's ends less `below` (m; 0 when it never does). Read once
        for every `room` of the way, the ends included: an arm with too few joints to hold a pose along a line
        turns its hand on the way, and its tip leaves the line."""
        q_a, q_b = np.asarray(q_a, dtype=float), np.asarray(q_b, dtype=float)
        pts = hand_points(self)
        p_a, p_b = self.model.fk_tip(q_a)[0], self.model.fk_tip(q_b)[0]
        n = max(int(ratios.QUANT), int(math.ceil(float(np.linalg.norm(p_b - p_a)) / max(room, np.finfo(float).eps))))
        worst = 0.0
        for k in range(n + 1):
            p, R = self.model.fk_tip(q_a + k / n * (q_b - q_a))
            thing = float(R[2, 0]) * float(thing_m) - float(half_m) * float(np.sum(np.abs(R[2, 1:])))
            under = [0.0, thing] + ([float(np.min((R @ pts.T)[2]))] if len(pts) else [])
            worst = max(worst, float(p_a[2] + k / n * (p_b[2] - p_a[2])) - below - (float(p[2]) + min(under)))
        return worst

    def _pl_way(self, target, facts, intent, B, z_end=None, expect=None, start=None, z_hand=None):
        """The way to a place, planned before anything moves. z_end: the tip height the way down ends at (None:
        no way down); expect: the expected contact of the way down (C9); start: the pose the way begins at (q,
        tip, R; default: where the hand was told to be).
        Returns {"legs", "names": the carry's one stream (up to the layer, across, down to the carry height),
        "set_leg": the way down from there, a motion of its own with its own price, or None, "q_over": the joints
        the carry ends at, "q_end": those the way down ends at, "layer", "clear_z": the height the level stretch
        must clear, "dip_m": what the layer stands over it for a joint leg, "z_carry", "said", "doubt": [...],
        "over": the tip point over the place at the carry height} or a refusal (no arm pose over the place at the
        carry height: no plan exists). No leg is left out unless it is shorter than the tip's own resolution."""
        told = self._told() if start is None else start
        p0, R = np.asarray(told.tip, dtype=float), np.asarray(told.R, dtype=float)
        obj = R @ np.asarray(facts["offset_T"], dtype=float)
        c = np.asarray(target["centre"], dtype=float)
        over_xy = np.array([c[0] - obj[0], c[1] - obj[1]])
        if self.model.n < len(p0) + len(p0):      # an arm that cannot keep its hand's rotation: where the held
            # thing's middle stands under the tip is read at the rotation the arm reaches over the place
            R_there = self._pl_solve(np.array([over_xy[0], over_xy[1], p0[2]]), R, R[:, 0],
                                     [told.q, self.model.q_home])[1]
            if R_there is not None:
                obj = np.asarray(R_there, dtype=float) @ np.asarray(facts["offset_T"], dtype=float)
                over_xy = np.array([c[0] - obj[0], c[1] - obj[1]])
        res = self._pl_res(B, told.q)
        half = self._pl_half_width()
        # what reaches lowest under the tip: the held thing, or the hand's own lowest point
        below = max(float(facts["below_m"]), self._pl_hand_low(R))
        way = horizontal(np.array([over_xy[0] - p0[0], over_xy[1] - p0[1], 0.0]))
        room = self._pl_room(B, told.q, way if float(np.linalg.norm(way)) > 0 else UP)
        room += float((intent or {}).get("clearance_m") or 0.0)
        doubt, first = [], {}
        floor = self._pl_support_z()
        top = float(target["top_near_z"])
        if floor is not None and top < floor - res:      # a point under the stated support: the carry stays over it
            doubt.append({"what": f"the place's point lies {(floor - top) * MM:.0f} mm under the stated support",
                          "acted": "carried it over the support and lowered it onto the support, the contact "
                                   "stop armed", "would_remove": LOOK_AGAIN})
            top = floor
        pressed = None if start is not None else self._pl_stands(told, facts)
        if pressed is not None:          # the held thing stands on something: it is risen first, that behind it
            under = [float(p0[0]), float(p0[1]), float(p0[2]) - below]
            first = {"kind": "lift", "away_from": "support" if floor is not None else under}
            if floor is not None:
                below = max(below, float(p0[2]) - floor)
            doubt.append({"what": f"the held thing stands on something where the place begins (the still arm reads "
                                  f"{pressed:.2f} N more than the hold explains)",
                          "acted": "rose first with that contact behind the hand" + (
                              f"; it reaches at most {below * MM:.0f} mm below the fingertips (they stand that far "
                              "over the stated support)" if floor is not None else ""),
                          "would_remove": LOOK_AGAIN})
        z_carry = top + below + room
        if start is None:
            self.state["place"]["way_z"] = None       # (set below: where this way's way down begins)
        if z_hand is None and start is None:          # no body of the hand comes down onto what the look shows
            z_hand = self._pl_hand_stop(over_xy, R, res)[0]
        if z_hand is not None:
            z_carry = max(z_carry, float(z_hand) + room)
        if z_end is not None:
            z_carry = max(z_carry, float(z_end))      # (a release height stated over the carry height)
        seeds = [told.q, self.model.q_home]
        deep = max(below, float(facts.get("below_max_m") or 0.0))
        if deep > below + room:
            # the take read the thing shallower than the fingertips stood over the stated support when the jaws
            # closed: the carry ends as high as the deeper figure asks (where the arm reaches that), and the way
            # down, an expected contact, covers the rest
            z_deep = max(z_carry, top + deep + room)
            if self._pl_solve(np.array([over_xy[0], over_xy[1], z_deep]), R, R[:, 0], seeds)[0] is not None:
                doubt.append({"what": f"the take read the held thing {float(facts['below_m']) * MM:.0f} mm deep under "
                                      f"the fingertips; they stood {deep * MM:.0f} mm over the stated support when "
                                      "the jaws closed",
                              "acted": "carried it as deep as that and lowered it from there, the contact stop armed",
                              "would_remove": None})
                z_carry = z_deep
        need = [(z_carry + room, "the place itself")]
        known, names = self._pl_known_top(p0[:2], over_xy, half + room, B)
        if known is not None:
            need.append((known + below + room, "what the world holds near the way" +
                         (f" ({', '.join(names)})" if names else "")))
        seen, look = self._pl_seen_top(p0[:2], over_xy, half + room)
        if seen is not None:
            need.append((seen + below + room, f"what look {look} shows near the way"))
        if floor is not None and deep > below:     # the level stretch clears the support by the most the thing can hang
            need.append((floor + deep + room + room, "the stated support, the held thing taken as deep as the "
                                                    "fingertips stood over that support when the jaws closed"))
        wanted, why = max(need, key=lambda t: t[0])
        while True:              # ... and over each measured thing the space book says the stretch would meet
            hit = None
            for level in (wanted - below, wanted):
                k, th = self._pl_kind(B, [p0[0], p0[1], level], [over_xy[0], over_xy[1], level], half)
                if k == "expected" and self._pl_top_of(th) is not None:
                    hit = th
                    break
            z_new = None if hit is None else self._pl_top_of(hit) + self._pl_band(B, hit) + room + below
            if z_new is None or z_new <= wanted:
                break
            wanted, why = z_new, f"{getattr(hit, 'id', None) or 'a measured thing'} (the space book)"
        layer = max(float(p0[2]), wanted)
        if float(p0[2]) >= wanted:
            why = "everything known near the way (the hand stood that high already)"

        def reach(z):
            a, _ = self._pl_solve(np.array([p0[0], p0[1], z]), R, R[:, 0], seeds)
            b, _ = self._pl_solve(np.array([over_xy[0], over_xy[1], z]), R, R[:, 0], seeds)
            return a is not None and b is not None

        low = max(float(p0[2]), z_carry)
        if self._pl_solve(np.array([over_xy[0], over_xy[1], z_carry]), R, R[:, 0], seeds)[0] is None:
            return refusal("not_reachable", "no arm pose holds the thing over the place with its bottom "
                           f"{room * MM:.0f} mm over what stands there (tip z {z_carry * MM:.0f} mm)",
                           "a place nearer the robot's base")
        if not reach(layer):
            lo, hi = low, layer
            while hi - lo > max(res, ratios.REL * room):       # the highest layer in reach over both ends
                mid = (lo + hi) / 2
                lo, hi = (mid, hi) if reach(mid) else (lo, mid)
            doubt.append({"what": f"the layer that clears {why} (tip z {layer * MM:.0f} mm) is out of reach over "
                                  "both ends of the way",
                          "acted": f"crossed at the highest layer in reach (tip z {lo * MM:.0f} mm), "
                                   f"{(layer - lo) * MM:.0f} mm lower, the contact stop armed",
                          "would_remove": None})
            layer = lo
        word = self._pl_core_intent(intent)
        over = np.array([over_xy[0], over_xy[1], z_carry])
        if start is None:
            self.state["place"]["way_z"] = float(z_carry)

        def build(z):
            """The carry's legs at layer z: (legs, names, the joints they end at, the dip of a joint leg across,
            the names of the points no arm pose takes)."""
            legs, names_out, q_prev, p_prev, dip, lost = [], [], told.q, p0, 0.0, []
            for name, p, more in (("rise", np.array([p0[0], p0[1], z]), first),
                                  ("across", np.array([over_xy[0], over_xy[1], z]), {}), ("down", over, {})):
                if float(np.linalg.norm(p - p_prev)) <= res:
                    continue            # under what the tip resolves: there is nothing to move
                leg = self._pl_leg(more.get("kind", "carry"), p, R, q_prev, intent=word,
                                   **{k: v for k, v in more.items() if k != "kind"})
                if leg is None:         # (an arm of joint legs) no pose takes this point: the next leg goes there
                    lost.append(name)
                    continue
                legs.append(leg)
                names_out.append(name)
                p_prev = p
                if np.asarray(leg["to"]).ndim == 1:
                    if name == "across":
                        dip = self._pl_dip(q_prev, leg["to"], room, below, deep, float(facts.get("half_m") or 0.0))
                    q_prev = leg["to"]
            return legs, names_out, q_prev, dip, lost

        legs, names_out, q_prev, dip, lost = build(layer)
        said, clear_z = f"crossed at tip z {layer * MM:.0f} mm, over {why}", layer
        if dip > res and reach(layer + dip):     # joint legs: the level stretch is raised by what its tip sinks
            layer += dip
            legs, names_out, q_prev, _, lost = build(layer)
            said = (f"crossed at tip z {layer * MM:.0f} mm, over {why} and {dip * MM:.0f} mm more (on a joint "
                    "leg this arm's hand turns and its lowest point sinks that far under a level line)")
        if lost:
            doubt.append({"what": f"no arm pose takes the hand to the end of the {' and the '.join(lost)} leg of the "
                                  f"way (tip z {layer * MM:.0f} mm)",
                          "acted": "went from the leg before straight to the next point of the way, the contact "
                                   "stop armed", "would_remove": None})
        set_leg, q_over, q_end = None, q_prev, None
        if z_end is not None and z_carry - float(z_end) > res:
            end = np.array([over_xy[0], over_xy[1], float(z_end)])
            if self.model.n >= len(over) + len(over):       # where the carry ends: the way down starts there
                q_over = self._pl_solve(over, R, R[:, 0], [q_prev, told.q])[0]
            label = next((x for x in (target.get("labels") or []) if x), None)
            set_leg = self._pl_leg("lower", end, R, q_prev if q_over is None else q_over, target=label,
                                   expect=expect, intent=word)
            if set_leg is not None:
                to = np.asarray(set_leg["to"], dtype=float)
                q_end = to if to.ndim == 1 else self._pl_solve(end, R, R[:, 0], [q_prev if q_over is None
                                                                                else q_over])[0]
        return {"legs": legs, "names": names_out, "layer": layer, "z_carry": z_carry, "doubt": doubt, "over": over,
                "set_leg": set_leg, "q_over": q_over, "q_end": q_end, "said": said, "stands_n": pressed,
                "clear_z": clear_z, "dip_m": layer - clear_z}

    def _pl_carry_impl(self, target, intent=None, way=None):
        """Carry the held thing over the place: ONE stream (rise, across, down to the carry height). Returns the
        stream's result in this package's words, with route and doubt, or a refusal."""
        B = bounds.of(self)
        facts = self._pl_hold_facts()
        way = way or self._pl_way(target, facts, intent, B)
        if "refused" in way:
            return way
        if not way["legs"]:
            return {"stopped": "goal", "leg_name": None, "route": "it stands over the place already", "bounds": None,
                    "doubt": way["doubt"], "ended": [], "contact": None}
        held = self._pl_held()
        pads = control_hold_pads.of_record(held)
        watch = LandWatch(self) if pads is not None else None      # the arm's and the jaws' own readings per period
        try:
            r = self._pl_stream(way["legs"], way["names"], held=held)
        finally:
            read = watch.done() if watch is not None else None
        if "refused" in r:
            return self._pl_refuse_motion(r, "carrying the thing to the place")
        r.update(route=way["said"], doubt=way["doubt"])
        planned_held(r, pads)
        met_held(r, pads, read)
        self._pl_rest_read(r)
        return r

    def _pl_rest_read(self, r):
        """A held stream that stopped on a contact or on the cap: the hold's load is read at rest NOW, inside the
        stream's own phase and before any word (control_hold.read_load with whole: a few control periods, said):
        r["load_at_rest"] = {said, in_hand, weight_n, periods}. Nothing is read where the stream ended otherwise or
        nothing is held."""
        from rcpx import control_hold
        if not isinstance(r, dict) or "refused" in r or str(r.get("stopped")) not in ("contact", "cap") \
                or self._pl_held() is None or "load_at_rest" in r:
            return
        load = control_hold.read_load(self, whole=True)
        r["load_at_rest"] = {"said": load.get("said"), "in_hand": load.get("in_hand"),
                             "weight_n": None if load.get("weight_n") is None else round(float(load["weight_n"]), 3),
                             "periods": int(load.get("periods") or 0)}

    def _pl_carried(self, r, target, facts):
        """The result of the carry tool for a stream that ended at its goal."""
        s = self._sense()
        obj = self._pl_object_centre(s, facts)
        off = float(np.linalg.norm(obj[:2] - np.asarray(target["centre"][:2], dtype=float)))
        over = float(s.tip[2]) - float(facts["below_m"]) - float(target["top_near_z"])
        return {"said": f"Carried the thing over the place: its bottom is about {over * MM:.0f} mm above what "
                        f"stands there, {off * MM:.0f} mm off it in plan; {r.get('route')}.", "held": True,
                "carried": True, "stopped": "goal", "object_mm": _mm(obj), "tip_mm": _mm(s.tip),
                "off_in_plan_mm": round(off * MM, 1), "bottom_over_mm": round(over * MM, 1),
                "route": r.get("route"), "bounds": r.get("bounds"), "strike": r.get("strike")}

    def _pl_stopped(self, r, doing, facts=None):
        """A way that ended before its goal, in the place's words: a lost hold says where. A stop on a contact or
        on the cap with the hold standing reads the hold's load at rest first (control_hold.read_load: a few
        control periods, said): "held" is said only where the thing's weight stands in the hand; where its weight
        rests on what it met the receipt says the jaws close on it and its weight rests there (held false); where
        that rest cannot tell (a thing whose weight this arm never read, or a hand held back) it says so and
        claims no hold. Any other stop is returned as it is, the thing still held."""
        from rcpx import control_hold
        stop = str(r.get("stopped"))
        facts = dict(facts or {})
        self._pl_rest_read(r)        # (read already where the stream read it inside its own phase)
        load = r.get("load_at_rest") if isinstance(r, dict) else None
        tip = _mm(self._sense().tip)
        if self._pl_held() is None:
            note = control_hold.ended(self) or {}
            why_lost = stop if stop.startswith("hold lost") else (note.get("said") or note.get("why"))
            return {"said": f"Lost the thing while {doing}: {why_lost}, with the tip at "
                            f"[{', '.join(f'{x:.0f}' for x in tip)}] mm; look there.",
                    "held": False, "carried": False, "placed": False, "stopped": "dropped", "dropped_at_mm": tip,
                    "why": stop, **facts}
        leg = f" on the {r['leg_name']} leg" if r.get("leg_name") else ""
        why = r.get("why") or ("something pushed on the thing or the arm" if stop in ("contact", "cap") else
                               "the move was told to stop")
        out = {"held": True, "carried": False, "placed": False, "stopped": stop, "leg": r.get("leg_name"),
               "contact": r.get("contact"), "why": r.get("why", ""), "tip_mm": tip, **facts}
        if load is None:
            out["said"] = f"Stopped while {doing}{leg} ({stop}): {why}; it is still held. Look before going on."
            return out
        out["load_at_rest"] = dict(load)
        read = f"read at rest over {int(load.get('periods') or 0)} control periods"
        if load.get("in_hand"):
            out["said"] = (f"Stopped while {doing}{leg} ({stop}): {why}; it is still held ({load.get('said')}, "
                           f"{read}). Look before going on.")
            return out
        out.update(held=False, in_jaws=True, rests_on_contact=load.get("in_hand") is False)
        out["said"] = (f"Stopped while {doing}{leg} ({stop}): {why}; "
                       + (f"{load.get('said')} ({read})" if load.get("in_hand") is False else
                          f"the jaws close on it; whether its weight still stands in the hand this rest does not "
                          f"tell ({load.get('said')}; {read})")
                       + ". Look before going on.")
        return out

    def _pl_relabel(self, top):
        """The picked label (if it was picked by label) now stands where it was set down."""
        label = self.state["place"].get("label")
        if not label:
            return
        hits = self.world.query(label=label)
        if hits:
            self.world.update(hits[-1]["id"], point_mm=_mm(top), source="place (set down here)")
