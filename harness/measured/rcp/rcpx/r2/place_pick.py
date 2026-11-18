# SPDX-License-Identifier: AGPL-3.0-only
"""pick of package place (round six: the design notes C8, C9, C11, C12, C14; the design rulings): take a thing, a
rim or a part at a spot, for any gripper arm, acting on what there is with the contact stop armed.

Before anything moves the take is counted (place_pick_count: the way as the bounds plan it, the close, the test
lift, the load read at rest): a stated limit these jaws cannot keep and a take that does not fit the control
periods left are refused there, the hand and the jaws where they were.

One take runs three phases, each a motion of the bounds (rcpx/bounds.py); the tool never states a speed:

- over and down: ONE stream (ex._stream): up to a height the hand stood free at (when it stands lower), across,
  down around the spot. The jaws open on the way (to the chosen width plus twice its error plus twice what the
  hand can be off its path, else as far as they go; never onto their own open end). The way down is an EXPECTED
  contact (C9): it brakes for nothing but a limit somebody stated (B.stated_limit), and a contact on it is no
  ending: the hand backs off by what its own give explains under the force it reads there while the jaws close
  where they stand, and that contact is the measured top. Reach is refused only after every route was tried: that one, one line to over
  the spot and down, and across at the hand's own height. The way down is measured as the hand ran it (how far
  it came down along the approach from where its last leg began), and the hold is told that length on every arm.
- close: the core's one close (control_hold.close), at the jaws' own rate unless a limit is stated. A hand with
  one moving jaw and no width seen moves with its jaw along its closing axis, the point between its pads where
  the thing's middle is (place_pick_count.follow). A stall wider than the thing was seen is said, not an ending.
  Under a stated limit the squeeze is eased by REL before the lift (the design rulings: kept by truth, the lift
  included; the close's squeeze is a figure of the jaws' own fitted numbers and the thing settles as it rises).
- test lift: ONE move straight up by twice the fingers' own reach, no longer than the way down (the hold's first
  leg, C8) unless that would not free the thing from where it stood (how deep the fingers took it, plus the
  hand's band): then it rises by what frees it, the hold kept, and the receipt says how much of the lift ran
  past the way down (the hold's own level judges that stretch). It ends with the load read at rest
  (control_hold.read_load). Held = the core's record stands after it AND the hand rose by the lift planned.
  A held thing is then read where it stands in the hand (place_hold.stands: the arm's own readings and one
  depth picture of the fixed camera the spot was pointed in; no motion, no control period): a thing taken off
  its middle turns between the pads as it comes up, and the record says by how much.

Nothing between the jaws: the hand rises the way it came down, then they open. A pick takes ONCE:
a take that did not hold comes back with what one fresh look shows at the spot (place_pick_again.shows) and the
one ready call that takes it again there; the operator decides. The receipt prints the take's planned count
beside the periods it ran, and lists the waits by motion the plan does not count (the end of the way down, the
close's arrive, the rest read) as owed by the motion core and the hold: they are no gate.

The thing comes as a Seen (place_point.resolve, with the spot of C11) or as a point; a point is a Seen with
nothing made out at it. On held the core's record carries "extent" (how far the thing reaches below the tip,
its half width) and the state of package place the keys the place reads (place_pick_out). Every doubt is a line
of C12 with one ready call. Nothing here reads ground truth or names a robot. Lengths m, forces N.
"""

import math

import numpy as np

from rcpx import control_hold
from rcpx.context import refusal
from rcpx.control_base import _mm
from rcpx.r2 import place_grasp, place_hold, place_land_watch, place_motion, place_pick_count
from rcpx.r2.place_grasp import line
from rcpx.r2.place_grasp_seat import SeatMixin
from rcpx.r2.place_motion import PlaceMotion, unit
from rcpx.r2.place_pick_out import PickOut

MM = 1000.0      # unit
HALF = 0.5       # numeric: the middle of a width; the share of its shorter leg a stream rounds a corner over
UP = np.array([0.0, 0.0, 1.0])
DOWN = -UP
OTHER_WAY = ("budget", "dark", "can_see", "selfknow_missing")     # refusals no other route to the spot changes


def _ratios():
    from rcpx import bounds_ratios as R
    return R


def _bounds(ex):
    from rcpx import bounds
    return bounds.of(ex)


def _pose(p, R):
    T = np.eye(len(p) + 1)
    T[:-1, :-1], T[:-1, -1] = R, p
    return T


def as_seen(seen):
    """A Seen as it is; a point as given (base frame, m) as a Seen with nothing made out at it."""
    if isinstance(seen, dict):
        return seen
    p = np.asarray(seen, dtype=float).reshape(-1)
    return {"id": None, "kind": "thing", "centre_xy": [float(p[0]), float(p[1])], "top_z": float(p[-1]),
            "pointed_xy": [float(p[0]), float(p[1])], "sigma_xy": 0.0, "sigma_z": None, "outline": None,
            "height": None, "hollow": None, "neighbours": [], "looks": [], "source": "the point as stated"}


class PickMixin(SeatMixin, PickOut, PlaceMotion):
    # ---------------------------------------------------------------- phases, legs, the stream
    def _pk_phase(self, name, did, t0, r=None, strikes=None):
        """The phase record for what ran since world time t0 (place_motion.phase), with the strike of its motion."""
        rec = place_motion.phase(self, name, did, t0, None if not isinstance(r, dict) else r.get("bounds"))
        if isinstance(r, dict) and r.get("strike") is not None:
            rec["strike"] = r["strike"]
            if strikes is not None:
                strikes.append((name, r["strike"]))
        return rec

    def _pk_leg(self, kind, p, R, axis, q_seed, **more):
        """One leg to the tip pose (p, R): a straight tip line for an arm that can hold a whole pose along it;
        for an arm with fewer joints than a pose has freedoms a joint leg to the same point, the tool pointing
        along axis (None when it cannot take that point)."""
        if self.model.n >= len(p) + len(p):
            return {"kind": kind, "to": _pose(p, R), "line": True, **more}
        q, _ = self._pl_solve(p, R, axis, [q_seed])
        return None if q is None else {"kind": kind, "to": q, **more}

    def _pk_stream(self, legs, intent):
        """ex._stream with the call's word; a refusal comes back as a refusal."""
        if getattr(self, "_stream", None) is None:      # a core without the stream: said, never replaced
            return refusal("core_motion", "this session's core has no stream (ex._stream): the pick cannot move "
                           "through it", "run it on a core that has the stream")
        place_land_watch.run_on(self)      # this session's free stops are read (how far the hand runs on)
        r = self._stream(legs, held=None, looks=None, need_m=None, intent=intent or None)
        return r if isinstance(r, dict) else {"stopped": "refused", "refused": {"check": "motion", "why": str(r)}}

    def _pk_expect(self, plan, intent, st):
        """(the expected contact of the way down (C9), the stated limit it rests on (C3)). The region is the
        stretch the fingers go down around the spot: its middle, and for its radius the spot's own error, the
        open hand's half width and half that stretch. With nothing stated bound_n is None: nothing lowers it."""
        B, J = _bounds(self), self._pl_jaws
        label = st.get("label") if isinstance(st.get("label"), str) else None
        try:
            lim = B.stated_limit(label, intent or None, expected=True)
        except Exception as exc:        # a label the bounds hold no entry for: what the call and the site state
            self._journal("bounds_problem", where="stated_limit", error=f"{type(exc).__name__}: {exc}")
            lim = B.stated_limit(None, intent or None, expected=True)
        bound = lim.get("limit_n")
        axis = np.asarray(plan["approach_axis"], dtype=float)
        p_low = np.asarray(self.model.fk_tip(plan["q_low"])[0], dtype=float)
        frac = J.frac_for_width(plan["open_m"])
        half = max([HALF * plan["open_m"]] + [abs(x) for b in J.finger_boxes(frac) if b is not None for x in b["a"]])
        return {"with": [float(x) for x in p_low - axis * HALF * plan["depth_m"]],
                "radius_m": plan["sigma_xy"] + half + HALF * plan["depth_m"], "along": [float(x) for x in axis],
                "bound_n": bound, "bound_source": lim.get("source") if bound is not None else None, "seek": True,
                "evidence": lim.get("evidence")}, lim

    def _pk_ways(self, plan, s, expect, st):
        """The ways in for a grasp plan, the better first: [(words, legs, the way down's length m, the point its
        last leg begins at, the joints there for an arm whose legs are joint legs)].
        (1) the plain route: up to the highest height the hand stood free at when a pick of this session began
        (when it stands lower now), across at that height, down (for a level approach: down to before the
        thing, then in). The hand is on its line long before the fingertips reach the thing, however far its
        servo trails the command round the corner.
        (2) one line to over the spot, then down: the way down twice what the fingers go down around the thing
        (their depth, their own reach past the tip, the hand's band), because a stream rounds a corner over half
        its shorter leg, and never shorter than the fingers are long (the line then crosses over the spot's top).
        (3) the plain route at the hand's own height, or the fingers just over the thing when that is higher."""
        J = self._pl_jaws
        axis = np.asarray(plan["approach_axis"], dtype=float)
        p_low, R_low = self.model.fk_tip(plan["q_low"])
        p_low, tip, band = np.asarray(p_low, dtype=float), np.asarray(s.tip, dtype=float), plan["band_m"]
        least = max(0.0, plan["depth_m"]) + J.low_m(J.frac_for_width(plan["open_m"])) + band
        down = {"kind": "approach", "to": _pose(p_low, R_low), "line": True, "target": None, "expect": expect}
        p_near = p_low - axis * least

        def plain(z_go, words):
            legs, start = [], tip
            if z_go - float(tip[-1]) > band:
                start = np.array([tip[0], tip[1], z_go])
                legs.append({"kind": "transit", "to": _pose(start, s.R), "line": True})
            p_go = np.array([p_near[0], p_near[1], z_go])
            if float(np.linalg.norm(p_go - start)) > band:
                legs.append({"kind": "transit", "to": _pose(p_go, R_low), "line": True})
            if abs(float(axis[-1])) < HALF and float(np.linalg.norm(p_go - p_near)) > band:
                legs.append({"kind": "transit", "to": _pose(p_near, R_low), "line": True})   # a level approach
                return words, legs + [down], least, p_near, None
            return words, legs + [down], float(np.linalg.norm(p_go - p_low)), p_go, None

        tall = max(least / HALF, J.finger_len_m or J.pad_height_m or 0.0)
        p_over = p_low - axis * tall
        # coming down from above, the corner of across and down stands as high over the low pose as (2)'s does:
        # a stream rounds a corner over half its shorter leg, so a corner at the thing's own top (the way down
        # only `least` long) brings a finger down onto that top before the hand stands on its line, and the
        # jaws then close on nothing over it
        z_lo = max(float(tip[-1]), float(p_near[-1]) if abs(float(axis[-1])) < HALF else float(p_over[-1]))
        z_hi = max(z_lo, float(st.get("free_z", tip[-1])))
        ways = [plain(z_hi, "up to where the hand stood free, across, down" if z_hi > z_lo else "across, then down")]
        if float(np.linalg.norm(p_over - tip)) > band:
            ways.append(("one line to over it, then down",
                         [{"kind": "transit", "to": _pose(p_over, R_low), "line": True}, down], tall, p_over, None))
        if z_hi - z_lo > band:
            ways.append(plain(z_lo, "across at the hand's own height, then down"))
        if self.model.n < len(p_low) + len(p_low):
            # an arm with fewer joints than a pose has freedoms cannot hold a whole pose along a line: its legs
            # are joint legs through the same points, the tool pointing along the approach; its way down begins
            # at the last of those points it can take, else where the hand stands
            joint = []
            for words, legs, length, _, _ in ways:
                q_prev, out, p_from = np.asarray(s.q, dtype=float), [], tip
                for leg in legs[:-1]:
                    T = leg["to"]
                    got = self._pk_leg(leg["kind"], T[:-1, -1], T[:-1, :-1], axis, q_prev)
                    if got is not None:               # a via point this arm cannot take is left out
                        out.append(got)
                        q_prev, p_from = got["to"], np.asarray(self.model.fk_tip(got["to"])[0], dtype=float)
                out.append({"kind": "approach", "to": np.asarray(plan["q_low"], dtype=float), "target": None,
                            "expect": expect})
                joint.append((words, out, length, p_from, q_prev))
            ways = joint
        return ways

    # ---------------------------------------------------------------- the pick
    def _pk_hints(self, st, t, width_m, yaw_rad, grasp, intent):
        """The operator's hints of a pick, with the hand's band at rest (the tip's resolution at the level of
        the arm at rest) as the session's first pick finds it: a level a contact of this session raised
        afterwards is no tracking error of the hand."""
        s = self._sense()
        band = st.setdefault("pick_band_m", place_grasp.num(_bounds(self).rigid(s.q, t), 0.0))
        return {"width_m": width_m, "yaw_rad": yaw_rad, "grasp": grasp, "t": t, "speed": intent.get("speed"),
                "band_m": band}

    def _pl_pick_impl(self, seen=None, t=None, width_hint_m=None, yaw_rad=None, grasp="auto", intent=None, **old):
        """The whole pick. seen: a Seen (place_point.resolve) or a point (base frame, m); t: unit approach
        direction. Returns a result (held true or false, takes, strategy, phases, doubt, strike, evidence) or a
        refusal; a refusal moved nothing unless it says so. old: the arguments of round two's callers (point,
        standoff_mm), taken as a point, the standoff ignored (it is derived)."""
        st = self.state["place"]
        intent = {k: v for k, v in dict(intent or {}).items() if v is not None}
        if seen is None:
            seen = old.get("point")
        t = DOWN if t is None else unit(np.asarray(t, dtype=float))
        self._pk_n = [0]
        ticks = getattr(self, "_ticks", None)

        def count(s):
            self._pk_n[0] += 1
            return None
        if isinstance(ticks, list):
            ticks.append(count)
        try:
            st["phases"] = []
            if seen is None:
                return refusal("spot", "no thing was given to pick", "point at it in a look ({look, px}), or give "
                               "its top middle as a point")
            return self._pk_run(st, as_seen(seen), self._pk_hints(st, t, width_hint_m, yaw_rad, grasp, intent), intent)
        finally:
            if isinstance(ticks, list) and count in ticks:
                ticks.remove(count)

    def _pk_run(self, st, seen, hints, intent):
        """ONE take (the design rulings: a pick never takes a second time by itself; a take that did not hold comes
        back with what the look shows, place_pick_out._pk_result)."""
        z0 = float(self._sense().tip[-1])
        st["free_z"] = max(float(st.get("free_z", z0)), z0)       # the hand stood free here when a pick began
        doubt, strikes, n0 = [], [], self._pk_n[0]
        take = self._pk_take(st, seen, hints, intent, doubt, strikes)
        if "refused" not in take:
            take["periods"] = self._pk_n[0] - n0
        return self._pk_result(st, seen, take, doubt, strikes)

    def _pk_prepare(self, st, seen, hints, intent):
        """What a take is planned from, before anything moves: (the Seen as read, the plan, ready), ready None
        when the plan is a refusal. ready: the jaws' opening command (never onto their own open end), the band
        of the close, the expected contact of the way down with the stated limit it rests on, the ways in."""
        R, B, J = _ratios(), _bounds(self), self._pl_jaws
        S = place_grasp.read_seen(seen)
        s = self._sense()
        v_way = place_grasp.num(B.v_free(s.q, hints["t"]), 0.0)
        J.clear_m = R.QUANT * S["sigma_xy"] + place_grasp.num(B.e_cross(v_way), 0.0)   # one moving jaw: the fixed
        plan = place_grasp.choose(self, seen, hints)                                    # jaw off the thing's face
        if "refused" in plan:
            return S, plan, None
        N = control_hold.numbers(self)
        open_frac = min(J.frac_for_width(plan["open_m"]), float(N["open_rest"]) - R.QUANT * float(N["still_band"]))
        band = S["sigma_xy"] + S["sigma_xy"] + R.QUANT * float(N["still_band"]) * float(N["gap_per_span_m"])
        expect, lim = self._pk_expect(plan, intent, st)
        return S, plan, {"open_frac": open_frac, "band": band, "expect": expect, "lim": lim, "N": N,
                         "ways": self._pk_ways(plan, s, expect, st)}

    def _pk_price(self, a, intent=None):
        """The control periods one take plans from where the hand stands (C12; the place's budget line asks it
        before a put starts): {"periods", "way", "close", "lift", "rest", "route"}, no second take. a: the
        pick's arguments (seen, spot, point, t, width_hint_m, yaw_rad, grasp). None when no take is planned (a
        spot that resolves to nothing, a plan that is a refusal): the pick itself then says why. Moves nothing."""
        st = self.state["place"]
        intent = {k: v for k, v in dict(intent or {}).items() if v is not None}
        seen = a.get("seen")
        if seen is None and a.get("spot") is not None:
            got = self._pl_spot(a["spot"], "pick")            # (the Seen, its point, its label, a refusal, ...)
            if got[3]:
                return None
            seen = got[0]
        seen = a.get("point") if seen is None else seen
        if seen is None:
            return None
        t = DOWN if a.get("t") is None else unit(np.asarray(a["t"], dtype=float))
        hints = self._pk_hints(st, t, a.get("width_hint_m"), a.get("yaw_rad"), a.get("grasp") or "auto", intent)
        z0 = float(self._sense().tip[-1])
        _, plan, ready = self._pk_prepare(dict(st, free_z=max(float(st.get("free_z", z0)), z0)), as_seen(seen),
                                          hints, intent)
        if ready is None:
            return None
        got = place_pick_count.count(self, plan, ready, {k: intent[k] for k in ("clear", "look", "speed", "accel")
                                                         if k in intent})
        return got if isinstance(got, dict) and "refused" not in got else None

    def _pk_take(self, st, seen, hints, intent, doubt, strikes):
        """One take: plan, count, over and down, close, test lift. Returns {held, said, plan, planned, ...} or a
        refusal; a refusal that came before the stream moved nothing."""
        R, B, J = _ratios(), _bounds(self), self._pl_jaws
        S, plan, ready = self._pk_prepare(st, seen, hints, intent)
        if ready is None:
            return plan
        doubt += [d for d in plan["doubt"] if d not in doubt]
        word = {k: intent[k] for k in ("clear", "look", "speed", "accel") if k in intent}
        planned = place_pick_count.count(self, plan, ready, word)       # counted before anything moves (C12)
        bad = place_pick_count.gate(self, plan, ready, word, planned)
        if bad is not None:
            return bad
        open_frac, band, lim, N = ready["open_frac"], ready["band"], ready["lim"], ready["N"]
        ev = {"seen_from": seen.get("source", "a Seen"),
              "jaws_go": {"centre_mm": _mm(np.asarray(plan["centre"])),
                          "jaw_bearing_deg": round(math.degrees(plan["yaw"]), 1),
                          "open_mm": round(plan["open_m"] * MM, 1), "tip_z_mm": round(plan["tip_z"] * MM, 1),
                          "depth_mm": round(plan["depth_m"] * MM, 1), "slide_mm": round(plan["slide_m"] * MM, 1),
                          "shift_mm": round(plan.get("shift_m", 0.0) * MM, 1),
                          "width_mm": None if plan["width_m"] is None else round(plan["width_m"] * MM, 1),
                          "seen_in_the_hands_way": plan.get("in_the_way")}}
        out = {"held": False, "plan": plan, "evidence": ev, "S": S, "seen": seen, "planned": planned}
        axis = np.asarray(plan["approach_axis"], dtype=float)
        # ---- the jaws open on the way (never onto their own open end); over and down in ONE stream
        self._gripper_cmd = open_frac
        t0, r, way, length, p_from, q_from, first, ran0 = self.sim_time(), None, "", 0.0, None, None, None, self._pk_n[0]
        for way, legs, length, p_from, q_from in ready["ways"]:
            r = self._pk_stream(legs, word)
            if "refused" not in r or r["refused"].get("check") in OTHER_WAY or self._pk_n[0] > ran0:
                break
            first = first or r                          # this route has no way: the next is tried before reach
        if "refused" in r:                              # is refused; the refusal said is the better route's
            return self._pl_refuse_motion(first or r, "coming to the thing (no route to it, the plain route up, "
                                          "across, down included)" if first else "coming to the thing")
        stop, on_down = r.get("stopped"), r.get("leg") == len(legs) - 1
        self._pk_phase("over and down", f"to over the spot and down around it in one stream ({way}), the jaws "
                       f"opening on the way ({stop})", t0, r, strikes)
        s_low = self._sense()
        # the way down as the hand ran it: how far it came down along the approach from where its last leg began
        ran = max(0.0, float((np.asarray(s_low.tip, dtype=float) - p_from) @ axis)) if on_down else 0.0
        ev["way"] = {"route": way, "stopped": stop, "ended": r.get("ended"), "periods": r.get("periods"),
                     "false_stops": r.get("false_stops"), "told_tip_z_mm": round(plan["tip_z"] * MM, 1),
                     "way_down_mm": round(ran * MM, 1), "way_down_planned_mm": round(length * MM, 1)}
        early = stop == "arrived" and on_down
        if stop != "goal" and not early:
            where = "on the way down" if on_down else "on the way to over the spot"
            out.update(said=f"The way to the thing stopped {where} ({stop}"
                       + (f": {r['why']}" if r.get("why") else "") + "): nothing was picked.", fingers="not_closed",
                       stopped=stop, contact=r.get("contact"), hint="retreat")
            return out
        # the hold is told the way down as it was run, on every arm (the leg the test lift may be as long as,
        # C8): the hold's own book takes the stretch after a stop the stream resumed from for the whole leg, and
        # the last straight stretch of a joint leg for its length
        self.__dict__.setdefault("_hold_legs", {}).update(leg_m=ran, by_leg=True)
        low = J.low_m(J.frac_for_width(plan["open_m"]))
        short = 0.0             # how far before the depth planned the way down ended (m, along the approach)
        if early:
            met = float(s_low.tip[-1]) - low
            ev["top_touched_z_mm"] = round(met * MM, 1)
            sz = plan.get("sigma_z")
            short = max(0.0, float((np.asarray(self.model.fk_tip(plan["q_low"])[0], dtype=float) - s_low.tip) @ axis))
            doubt.append(line(f"the way down met something at z {met * MM:.0f} mm, {short * MM:.0f} mm before the depth "
                              f"planned (its top was seen at {plan['top_z'] * MM:.0f} mm"
                              + ("" if sz is None else f", known to {sz * MM:.1f} mm") + ")",
                              "backed off by the arm's own give under the force it read there and closed where the "
                              "jaws stood: that contact is the measured top", None))
        # ---- close: the core's one close; a contact on the way down is backed off from while the jaws close
        t0, arm, q_back = self.sim_time(), None, None
        if early:
            # by what the arm's own give explains under the force it reads on that contact now (its resolution,
            # its jitter, that force over its servo's stiffness): a level a stop of this session raised is no
            # measure of how hard the hand rests on the thing
            back = place_grasp.num(B.rigid(s_low.q, axis, float(np.linalg.norm(s_low.F))), 0.0)
            q_back, _ = self._pl_solve(s_low.tip - axis * back, s_low.R, axis, [s_low.q])
            if q_back is not None:
                arm = lambda frac, q=np.asarray(q_back, dtype=float): q      # noqa: E731
                ev["backed_off_mm"] = round(back * MM, 2)
        # one moving jaw, no width seen: the hand moves with its jaw (from where it backed off to), the point
        with_jaw = place_pick_count.follow(self, q_back) if plan["width_m"] is None or plan.get("follow") else None
        arm = with_jaw or arm                                                                    # where it stands
        # a limit stated for the thing: the pads press it at once and it sees their sum, so each pad's strike and
        # squeeze is held to its share (18.3 N by truth on a thing whose limit was 14.67 N with the whole limit
        # given to each of two pads)
        how = dict(band_m=band, limit_n=lim.get("limit_n"), pads=len(J.sides),
                   limit_source=lim.get("source") if lim.get("limit_n") is not None else None,
                   depth_m=plan["depth_m"] if plan["depth_m"] > 0.0 else None, wide_m=place_pick_count.wide_of(plan))
        c = control_hold.close(self, expect_m=plan.get("expect_m"), arm=arm, width_source=plan.get("width_source"),
                               **how)
        if "refused" in c:
            return self._pl_refuse_motion(c, "closing on it (the hand stands at the spot, the jaws as they were)")
        shut = float(N["closed_gap"]) * (1.0 + R.REL) + R.QUANT * float(N["still_band"])
        if c["ended"] == "closed" and c.get("frac") is not None and c["frac"] > shut:
            # the jaws stand open of where they meet each other: something thin is between them that their
            # reading did not tell while it still gave way. They stand now: closed once more, it is told
            ev["closed_again"] = f"the jaws stood {c['gap_m'] * MM:.1f} mm apart with nothing told: closed once more"
            c2 = control_hold.close(self, expect_m=c.get("gap_m"), width_source="seen: the jaws' own reading (they "
                                    "stood that far apart)", **how)
            c = c2 if "refused" not in c2 else c
        gap = c.get("gap_m")
        ev.update(stall_width_mm=None if gap is None else round(gap * MM, 1), close=c["ended"],
                  squeeze_at_stall_n=c.get("squeeze_n"), arm_force_peak_n=round(float(c.get("arm_read_n") or 0.0), 2),
                  jaw_numbers=c.get("jaw_numbers"), hand_moved_with_jaw=with_jaw is not None)
        limited = lim.get("limit_n") is not None
        how_hard = "" if c.get("peak_n") is None else (
            f": the pads arrived at {float(c.get('arrive_m_s') or 0.0) * MM:.1f} mm/s, squeeze held under "
            f"{float(c['peak_n']):.1f} N on each pad (the arrival's strike and the squeeze it leaves, by the jaws' "
            "own stop slope)")
        self._pk_phase("close", f"closed on it ({c['ended']}){how_hard}", t0, {"bounds": self._pk_item(
            "force", c.get("squeeze_n"), c["price"], "nothing: the jaws' own rate (gentle: true states a limit)"
            if not limited else "leave gentle out" if lim.get("source") == "intent" else
            f"nothing on this call: the limit is stated by the {lim.get('source')}",
            lim.get("source") if limited else "model"),
            "strike": place_pick_count.close_strike(self, plan, ready, c)}, strikes)
        if c.get("held"):
            # where the pads stand on it, read before the lift: jaws that stalled wider than the thing stand on a
            # corner of it and are re-seated once inside this take (place_grasp_seat; the design rulings stands)
            c, s_now = self._pk_reseat(plan, ready, S, c, how, word, ev, doubt, strikes, ran)
            if ev.get("reseat"):
                gap, s_low = c.get("gap_m"), s_now
                ev.update(stall_width_mm=None if gap is None else round(gap * MM, 1), close=c.get("ended"),
                          squeeze_at_stall_n=c.get("squeeze_n"))
        out.update(close=c, s_low=s_low, way_down=ran, early=early)
        if not c.get("held"):
            # nothing between the jaws: the hand rises the way it came down before they open again (jaws that
            # open inside a hollow thing push on its walls); an arm of joint legs goes back to where its last
            # leg began
            t0, s = self.sim_time(), self._sense()
            up = {"kind": "retreat", "to": q_from, "away_from": -axis} if q_from is not None else \
                self._pk_leg("retreat", s.tip - axis * ran, s.R, axis, s.q, away_from=-axis)
            ru = {"stopped": "not reachable"} if up is None else self._pk_stream([up], word)
            if "refused" not in ru:
                self._pk_phase("rise", f"rose out of the spot with the jaws as they closed ({ru.get('stopped')})",
                               t0, ru, strikes)
            rose = float((np.asarray(s.tip, dtype=float) - self._sense().tip) @ axis)
            self._gripper_cmd = open_frac
            out.update(fingers="met_each_other" if c["ended"] == "closed" else "not_closed",
                       width_mm=None if gap is None else round(gap * MM, 1),
                       rose=bool(ru.get("stopped") == "goal" and rose >= (1.0 - R.REL) * ran - plan["band_m"]),
                       said=(f"the jaws closed to {(gap or 0.0) * MM:.1f} mm and met each other at z "
                             f"{s_low.tip[-1] * MM:.0f} mm: nothing was between them" if c["ended"] == "closed" else
                             f"the close ended ({c['ended']}) before the jaws' own reading told a thing between them"))
            return out
        if plan.get("expect_m") is not None and gap is not None and gap > plan["expect_m"] + band \
                and not ev.get("reseat"):
            doubt.append(line(f"the jaws stalled {gap * MM:.0f} mm apart where it was seen "
                              f"{plan['expect_m'] * MM:.0f} mm across (known to {band * MM:.1f} mm): they stand on "
                              "something wider, or on a corner of it", "kept the hold and ran the test lift",
                              {"tool": "look", "args": {"camera": "wrist"}, "periods": 0}))
        if limited and c.get("frac") is not None and not isinstance(self._gripper_cmd, str):
            # the design rulings (a limit served to the close is kept by truth, the lift included): the close leaves the
            # squeeze AT the limit by the jaws' own fitted numbers, and a held thing settles between the pads as
            # it comes up (20.7 N read in the lift on a 20 N thing): the command leads the reading REL less
            lead = max(0.0, float(c["frac"]) - float(self._gripper_cmd))
            self._gripper_cmd = float(self._gripper_cmd) + R.REL * lead
            control_hold.note_squeeze(self)
            ev["squeeze_eased"] = ("before the lift the jaws' command leads their reading REL less than the stated "
                                   "limit allows: the held thing settles between the pads as it comes up")
        return self._pk_lift(plan, S, s_low, c, ran, short, q_from, word, out, strikes, doubt)

    def _pk_lift(self, plan, S, s_low, c, ran, short, q_from, word, out, strikes, doubt):
        """The test lift: ONE move straight up, no longer than the way down (which is the leg in the hold's record
        was told, C8) unless that would not free the thing from where it stood (how deep it stands between the
        fingers: the depth planned less what an early contact stopped short of it, plus the hand's band): then
        the hold is kept and the hand rises by what frees it, on past where its last leg began, and the doubt
        says how far (the hold's record keeps the way down as run: the core judges the rest by the hold's own
        level). It ends with the load read at rest. Held = the record stands after it and the hand rose by the
        lift planned."""
        R, J, ev = _ratios(), self._pl_jaws, out["evidence"]
        corners, hang, half = self._pk_extent(plan, S, s_low, c.get("gap_m"))
        # the error of the thing's bottom under the tip, for the landing's band (place_land.band): the take's own
        # figure for the level it stood on (the look's height error; a point with none: the stated support's)
        sig_b = S["sigma_z"] if S["sigma_z"] is not None else place_grasp.support_at(_bounds(self), plan["centre"])[1]
        control_hold.stated(self, {"extent": {"below_tip_m": hang, "half_width_m": half, "below_sigma_m": sig_b},
                                   "extent_m": corners, "sigma_m": S["sigma_xy"]})
        rec, t0 = control_hold.held(self), self.sim_time()
        frees = place_pick_count.frees(plan, short)
        lift = place_pick_count.test_lift(J, plan, ran, short)
        s = self._sense()
        away = "support" if plan.get("support_z") is not None else UP
        leg = self._pk_leg("lift", s.tip + UP * lift, s.R, s.R[:, 0], s.q, away_from=away)
        if leg is None and q_from is not None:          # an arm of joint legs that cannot rise straight up from
            leg = {"kind": "lift", "to": q_from, "away_from": away}       # here: back to where its last leg began
            lift = max(0.0, float(self.model.fk_tip(q_from)[0][-1] - s.tip[-1]))
        if leg is None:
            return refusal("not_reachable", "the arm cannot rise straight up from here (the jaws are closed on it)",
                           "gripper open")
        if lift > ran:
            doubt.append(line(f"the way down ran {ran * MM:.0f} mm and {frees * MM:.0f} mm free a thing taken "
                              f"{(frees - plan['band_m']) * MM:.0f} mm deep from where it stood (the hand's band "
                              f"{plan['band_m'] * MM:.0f} mm)", f"kept the hold and rose {lift * MM:.0f} mm: "
                              f"{(lift - ran) * MM:.0f} mm of it past the way down, where the hold's own level "
                              "judges a contact", None))
        elif lift < frees:
            doubt.append(line(f"this arm rises {lift * MM:.0f} mm from here and {frees * MM:.0f} mm free a thing "
                              f"taken {(frees - plan['band_m']) * MM:.0f} mm deep: the lift does not show that it "
                              "came up", "kept the hold on the jaws' own record", place_grasp.LOOK))
        rl = self._pk_stream([leg], word)
        if "refused" in rl:
            return self._pl_refuse_motion(rl, "lifting it (the jaws are closed on it)")
        rec = control_hold.check(self)
        went = rl.get("stopped") == "goal"
        if rec is not None and went and dict.get(rec, "weight_n") is None and rec.get("read_call") != self._calls:
            control_hold.read_load(self)                # the lift ends with the load read at rest (C8)
            rec = control_hold.held(self)
        s1 = self._sense()
        rise = float(s1.tip[-1] - s_low.tip[-1])
        risen = went and rise >= (1.0 - R.REL) * lift - plan["band_m"]     # the hand rose by the lift planned
        self._pk_phase("lift", f"lifted {rise * MM:.0f} mm in one move, the load read at rest after it "
                       f"({rl.get('stopped')})", t0, rl, strikes)
        hev = place_hold.judge(self, s1.gripper, self._gripper_cmd)
        ev.update(held_evidence=hev, lift_mm=round(rise * MM, 1), lift_planned_mm=round(lift * MM, 1),
                  weight_felt_n=hev.get("weight_felt_n"))
        out.update(held=bool(rec is not None and risen), s1=s1, rise=rise, lift=rl, extent=(corners, hang, half))
        if out["held"]:
            self._pk_stands(out, s, s1, doubt)          # where it stands in the hand now: on the core's record
        else:
            gone = control_hold.ended(self) or {}
            out.update(fingers="slipped" if rec is None else "stalled_on_object", stopped=rl.get("stopped"),
                       said=(str(gone.get("said") or f"the hold ended in the lift ({gone.get('why')}): the jaws closed "
                                 f"{gone.get('closed_mm', 0.0)} mm past where they stood on it") if rec is None else
                             f"the test lift rose {rise * MM:.0f} mm of {lift * MM:.0f} mm ({rl.get('stopped')}"
                             + (f": {rl['why']}" if rl.get("why") else "") + "): it did not come up"))
        return out


def approach_vector(approach):
    """The approach as a unit vector (frame base): a name or [x, y, z]; None when it is not one."""
    names = {"down": [0.0, 0.0, -1.0], "up": [0.0, 0.0, 1.0], "forward": [1.0, 0.0, 0.0],
             "back": [-1.0, 0.0, 0.0], "left": [0.0, 1.0, 0.0], "right": [0.0, -1.0, 0.0]}
    if approach is None:
        return np.array([0.0, 0.0, -1.0])
    if isinstance(approach, str):
        v = names.get(approach.strip().lower())
        return None if v is None else np.asarray(v)
    try:
        v = np.asarray(approach, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    if v.size != len(UP) or not np.all(np.isfinite(v)) or not float(np.linalg.norm(v)) > 0.0:
        return None
    return unit(v)
