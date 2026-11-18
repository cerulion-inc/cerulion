# SPDX-License-Identifier: AGPL-3.0-only
"""The set down of package place (N4), round six (the design notes, package PLACE stage A; RULINGS-R6 the design rulings).

A set down is an expected contact (C9): the plan brakes for nothing but a stated limit (C3), the guard ends the way
down where the held thing lands ("arrived"), and where it meets nothing the way ends at its planned height and the
thing is let go there, with the drop said. Nothing is searched for and nothing is refused because a height is
not sure.

- onto (the spot's own point): the held thing's bottom is lowered to the point's height. It ends "arrived", or at
  that height with the drop said (also when the stated limit is one this arm cannot seek by force: C3, seek false).
- into (the spot has an entry point: the pointed ray comes in over a rim): the held thing's middle goes over the
  entry point and its bottom is lowered until "arrived" or until the hand's lowest hull point stands at the top of
  what is near the point plus the hand's tracking error; it is let go there. The receipt says how far its bottom
  stood under the rim, and how far over the floor the pointed ray showed inside. Into an opening whose floor no
  look shows (spots given on its rim) the thing is lowered toward the stated support, to the first contact the arm
  feels (the design rulings: the whole way under the rim is the arrival stretch), not let go over the rim; with no support
  stated either, the hand stops over the rim, said. (A shallow dish whose rim the hand clears is a place onto its
  floor: rcpx/r2/place_spot._dish.)
  An opening has a wall beside the thing where the hand stops over the rim before the thing's bottom reaches
  what the pointed ray shows inside (the thing is let go over the inside and has to fall in), or where the rim
  stands higher over the planned bottom than the hand's tracking error on the way down (a rim within that error
  is the surface the thing lands on: a shallow dish is no such opening). There the spot is held against the
  newest look first (_pl_fit, place_side.free_spot): where the look shows the rim under the thing's own
  footprint, or something under a body of the hand, nearer than the plan can be off, the thing goes to the
  nearest spot of the same opening that the look shows free by that much (the least shift that can give it);
  into an opening tighter than that, to its middle: the spot with the most room, and of spots with the same
  room the one nearest the middle of an opening the look shows closed all around (place_side.free_spot: a
  thing between two walls has the same room all along them; at the end of that stretch a third wall stands
  as near). The receipt says how far beside the pointed spot, why, and how far the footprint stands over what
  is around it when the opening is tighter than the thing.
  A way down into such an opening that ends on something that is no landing inside (a contact the core does
  not take for the expected one: a push on a link, a push across the travel; or a landing with the thing's
  bottom still at or over the rim, whatever height the plan ended at: a hand's stop over something beside
  the place can put the plan's own end over the rim) does not let go there: the hand rises to where the way
  down began, moves to the spot with the most room that the look shows, and comes down ONCE more
  (_pl_again), to the end the hand's stop gives at that spot; what that second way ends on is let go on.
  With no look, or no spot with more room, the thing is let go where the first way ended, as said. A way
  down that ended on the cap is never run again (the strike passed the robot's own rating).
- on the operator's word (release: drop, or a release height) it is let go above, the height stated.
The jaws open through the core's release (rcpx/control_hold.py: the load leaving the hand is expected); the hand
then rises clear of the thing in one stream. A thing that was let go is not written into the space book: where it
came to rest was not measured, and the next look shows it.

Arrival evidence, never a colour: the release facts (where the jaws opened, how high over what, whether the
landing was felt) and, where a camera can say, the same view before and after at the pick spot and the place.
Nothing here reads ground truth or names a robot or a thing.
"""

import math

import numpy as np

from rcpx import bounds
from rcpx import bounds_ratios as ratios
from rcpx import control_hold
from rcpx.control_base import _mm
from rcpx.r2.place_hand import finite
from rcpx.r2 import place_land
from rcpx.r2.place_land import LandMixin
from rcpx.r2.place_motion import UP, phase
from rcpx.r2.place_side import HALF, free_spot, hand_hulls, hand_points

MM = 1000.0
DOWN = -UP
LOOK_IN = {"tool": "look", "args": {"camera": "wrist"}, "periods": 0}


class SetMixin(LandMixin):
    def _pl_hand_half(self):
        """The largest distance of the hand's hull from its pointing axis (m), at the jaws' present command."""
        st = self.state["place"]
        key = self._gripper_cmd
        if st.get("hand_half") is None or st["hand_half"][0] != key:
            pts = hand_points(self)
            half = float(np.max(np.hypot(pts[:, 1], pts[:, 2]))) if len(pts) else self._pl_jaws.max_m / 2
            st["hand_half"] = (key, half)
        return st["hand_half"][1]

    def _pl_hand_low(self, R):
        """How far the hand's lowest hull point stands below the tip at hand rotation R (m, never negative)."""
        pts = hand_points(self)
        if not len(pts):
            return 0.0
        return max(0.0, -float(np.min((np.asarray(R, dtype=float) @ pts.T)[2])))

    def _pl_track(self, B, q, travel_m):
        """The hand's tracking error on a way down of about travel_m (m): the bounds' own clearance at the fastest
        speed that leg plans (the body's own speed down, or what its braking lets a leg that short reach): how
        far the tip trails its command there and half of what it travels in one control period; and the tip's
        own resolution (never a figure that grows with the session's contact level)."""
        v = min(float(B.v_free(q, DOWN)), math.sqrt(max(0.0, float(B.a_brake(q, DOWN)) * float(travel_m))))
        return float(B.clearance(v)) + self._pl_res(B, q, DOWN)

    def _pl_still(self, B, q, facts):
        """How near its end a way down is waited for before the jaws open (m): the tip's own resolution and what
        the held thing's own weight moves the tip by (a landing takes that weight off the arm). Neither follows
        the session's contact level: the core's own figure for the wait does, and after one stop the bounds
        call false it lets a hand that still trails its command by centimetres count as arrived."""
        self._pl_land_watch()         # the way down begins: it is read while it runs (place_land_watch.LandWatch)
        if self.state["place"].pop("land_fast", False):
            # the landing's arrival stretch follows: its end is the one waited for; the fast stretch's own end
            # only where the hand has to come to the arrival speed there (place_land: the hand-over)
            return self.state["place"].get("land_need")
        return self._pl_still_m(B, q, facts)

    def _pl_still_m(self, B, q, facts):
        """_pl_still's figure alone (the wait before the jaws open)."""
        w = facts.get("weight_n")
        return float(B.rigid(q, DOWN, float(w))) if finite(w) and float(w) > 0 else self._pl_res(B, q, DOWN)

    # ---------------------------------------------------------------- where it fits into an opening
    def _pl_thing_half(self, facts):
        """The half width the held thing surely covers about its middle (m): half its narrow width as the look
        showed it when this call's take saw it whole (its middle is then the thing's own); else half of what the
        jaws closed on (the thing is at least that wide where it is held)."""
        took = self.state["place"].get("taken")
        if took and str(facts.get("middle") or "").startswith("the middle of the thing"):
            return float(took["narrow_m"]) * HALF
        gap = facts.get("gap_m")
        return float(gap) * HALF if finite(gap) else float(facts["half_m"])

    def _pl_fit(self, target, facts, R, z_end, track, most=False):
        """Where the held thing fits into the opening the newest look with depth shows (place_side.free_spot), or
        None: no look shows anything, or the planned spot has the room, or no better spot is shown.
        In the thing's way: what the look shows higher than the lower of two levels: where the thing's bottom
        is planned, by more than the look's own band, and the rim the pointed ray came in over (the rim the
        spots were given on) less that band (a wall is counted from its side down to there, not only at its top
        edge). Where the pointed ray lands is not such a level: when it lands on the floor inside, the floor
        itself (seen within its band here, over it there) would stand in the way of a thing that never comes
        down to it. In a hand body's
        way: what the look shows higher than that body's lowest point at the planned end less the hand's
        tracking error (the same line the way down's end is held to), by more than that band. The room wanted:
        the spot's error in plan, what the tip may be off its line by across a way down at the body's own
        speed, and the tip's own resolution. What stood where the taken thing stood is left out. most: the spot with the most room (a way down ended on
        something the plan did not expect)."""
        pts, look, sigma = self._pl_look_points()
        if pts is None or not len(pts) or z_end is None:
            return None
        B = bounds.of(self)
        c = np.asarray(target["centre"], dtype=float)
        R = np.asarray(R, dtype=float)
        obj = R @ np.asarray(facts["offset_T"], dtype=float)
        band = ratios.BAND * float(sigma)
        skip = self._pl_skip()
        keep = np.ones(len(pts), dtype=bool) if skip is None else \
            np.linalg.norm(pts[:, :-1] - skip[0], axis=1) > skip[1]
        rim = target.get("rim_in_z") if finite(target.get("rim_in_z")) else target.get("rim_z")
        inside = [float(z_end) - float(facts["below_m"]) + band]
        inside += [float(v) - band for v in (rim,) if finite(v)]
        level = min(inside)
        bodies = []
        for name, h in hand_hulls(self):
            w = (R @ h.T).T
            bodies.append((name, w[:, :-1] - obj[:-1], float(z_end) + float(w[:, -1].min()) - float(track) + band))
        told = self._told()
        sig = target.get("sigma_xy")
        grow = (float(sig) if finite(sig) else 0.0) + float(B.e_cross(float(B.v_free(told.q, DOWN)))) \
            + self._pl_res(B, told.q)
        toward = (target.get("seen") or {}).get("pointed_xy")
        got = free_spot(pts[keep], c[:-1], self._pl_thing_half(facts), level, bodies, grow, band, toward_xy=toward,
                        most=most, across=math.sqrt(len(pts)))
        if got is not None:
            got.update(look=look, grow_m=grow)
        return got

    @staticmethod
    def _pl_fit_line(fit):
        """The doubt line of a spot that was moved to where the look shows room."""
        short = fit["room_at_m"]
        what = (f"look {fit['look']} shows something under {fit['by']} at the pointed spot" if short <= 0 else
                f"look {fit['look']} shows something {short * MM:.0f} mm beside {fit['by']} at the pointed spot")
        where = ("the nearest spot of the same opening" if fit["how"] == "nearest" else
                 "the middle of the opening, the spot with the most room,")
        room = (f"free by {fit['room_m'] * MM:.0f} mm" if fit["room_m"] >= 0 else
                f"{-fit['room_m'] * MM:.0f} mm over what stands around it (the opening is tighter than the thing "
                "and the room the plan can be off by)")
        return {"what": what + f" (the plan can be off by {fit['grow_m'] * MM:.0f} mm)",
                "acted": f"set it down {fit['moved_m'] * MM:.0f} mm beside the pointed spot, at {where} that the "
                         f"look shows {room}", "would_remove": LOOK_IN}

    # ---------------------------------------------------------------- what the set down will be
    def _pl_set_plan(self, target, intent, facts, release=None, release_m=None):
        """The plan of a set down, before anything moves: {"mode": onto | into | release_above, "z_end": the tip
        height the way down ends at (None: it ends at the carry height), "base_z", "expect" (C9), "limit", "why",
        "doubt": [...]}. Never a refusal: a place that cannot be reached is refused by the way."""
        B = bounds.of(self)
        told = self._told()
        below = float(facts["below_m"])
        into = target["kind"] in ("into", "rim")
        if target.get("pointed") is None:        # every plan starts from the spot as it was pointed
            target["pointed"] = np.asarray(target["centre"], dtype=float).copy()
        target["centre"] = np.asarray(target["pointed"], dtype=float).copy()
        surf, rim = target.get("surface_z"), target.get("rim_z")
        base = float(rim if into and rim is not None else (surf if surf is not None else target["centre"][2]))
        plan = {"mode": "into" if into else "onto", "base_z": base, "band_m": 0.0, "why": None, "doubt": [],
                "below_m": below}
        near = float(target["top_near_z"])
        track = self._pl_track(B, told.q, self._pl_room(B, told.q, DOWN) + (below if into else max(0.0, near - base)))
        plan["track_m"] = track
        falls, z_fit = into, None
        if into:
            # INTO an opening the hand's lowest hull point stops at the top of what is near the point plus its
            # tracking error, and the thing's bottom is not planned under the surface the pointed ray lands on
            # inside (what lies under a seen surface is not seen). Where no look shows the floor inside, the thing
            # is lowered to the first contact the arm feels (the design rulings: the whole way under the rim is the arrival
            # stretch), toward the stated support, never let go over the rim for want of a seen floor; the hand's
            # own stop over what the newest look shows under its bodies (_pl_hand_lift below) holds it higher
            # where a body would stand on a wall, and the opening's fit (_pl_fit) reads the hand over the rim as
            # before (z_fit). With neither a seen floor nor a stated support the hand stops over the rim, said.
            z_end = float(target["top_near_z"]) + track + self._pl_hand_low(told.R)
            floor = self._pl_support_z()
            if finite(target.get("seen_z")):
                falls = z_end >= float(target["seen_z"]) + below
                z_end = max(z_end, float(target["seen_z"]) + below)
                plan["why"] = "the hand stops over what stands near the place, with its tracking error"
            elif floor is not None:
                z_fit, z_end = z_end, floor + below      # (the opening's fit reads the hand over the rim, as before)
                plan["why"] = "the floor inside is not seen: lowered toward the stated support until it lands"
            else:
                plan["why"] = "the floor inside is not seen and no support is stated: the hand stops over the rim"
        else:
            z_end = base + below
        floor = self._pl_support_z()
        if floor is not None and floor + below > z_end:      # the thing's bottom never goes under the stated support
            z_end = floor + below
            plan["why"] = plan["why"] or "the spot lies under the stated support: lowered to the support"
        # an opening with a wall beside the thing: the hand stops over the rim before the thing's bottom reaches
        # what the pointed ray shows inside (it is let go over the inside and has to fall in), or the rim
        # stands higher over the planned bottom than the hand's tracking error on the way down. (A rim within
        # that error of the planned bottom is the surface the thing lands on: a shallow dish.)
        plan["walled"] = bool(into and (falls or float(target.get("rim_z", base)) - (z_end - below) > track))
        fit = self._pl_fit(target, facts, told.R, z_end if z_fit is None else z_fit, track) if plan["walled"] else None
        if fit is not None:      # into an opening: where the look shows room for the thing and for the hand
            target["centre"] = np.array([fit["xy"][0], fit["xy"][1], float(target["centre"][-1])])
            plan["fit"] = fit
            plan["doubt"].append(self._pl_fit_line(fit))
        # ... and no body of the hand is planned onto what the newest look shows under it (a neighbour, a rim)
        plan["z_low"], plan["z_fit"] = z_end, z_fit
        z_end, plan["hand_stop_z"], line, why = self._pl_hand_lift(target, facts, told.R, z_end, track)
        if line is not None:
            plan["doubt"].append(line)
            plan["why"] = plan["why"] or why
        above = release == "drop" or finite(release_m)
        if above:
            plan.update(mode="release_above", why="on your word (release)")
            z_end = None if not finite(release_m) else max(z_end, base + float(release_m) + below)
        plan["z_end"] = z_end
        # the landing (place_land: the design rulings): the force it may make, the band where a contact can begin, the way
        # down's expected contact and the two figures stated before anything moves
        plan.update(self._pl_land_plan(B, target, intent, facts, z_end, into, above, plan["walled"], plan["z_low"],
                                       plan["hand_stop_z"]))
        if facts.get("doubt"):
            plan["doubt"].append({"what": facts["doubt"], "acted": "lowered it with the contact stop armed",
                                  "would_remove": None})
        return plan

    def _pl_hand_lift(self, target, facts, R, z_end, track):
        """(the tip height the way down ends at, the hand's stop height or None, a doubt line or None, why or
        None) for the thing's middle over target's centre at hand rotation R: z_end, or higher where a body of
        the hand, its footprint grown by the spot's error and the hand's tracking error, would stand on what
        the newest look shows under it (place_carry._pl_hand_stop)."""
        sig = target.get("sigma_xy")
        c = np.asarray(target["centre"], dtype=float)
        obj = np.asarray(R, dtype=float) @ np.asarray(facts["offset_T"], dtype=float)
        stop, body, look = self._pl_hand_stop(c[:-1] - obj[:-1], R, (float(sig) if finite(sig) else 0.0) + track)
        if stop is None or stop + track <= z_end:
            return z_end, None if stop is None else stop + track, None, None
        line = {"what": f"look {look} shows something under the hand's {body} beside the place",
                "acted": f"the way down ends {(stop + track - z_end) * MM:.0f} mm higher, the hand over it; the "
                         "thing is let go from there", "would_remove": LOOK_IN}
        return stop + track, stop + track, line, f"the hand's {body} would stand on what look {look} shows beside " \
                                                 "the place"

    # ---------------------------------------------------------------- let go and rise
    def _pl_open_goal(self):
        """(the opening the release goes to, the open rest), shares of the span: from where the jaws stand, as far
        as their reading trails their command at their own rate plus QUANT still bands (the pads are then off the
        thing); the rest of the opening runs while the hand rises. The open rest when that is nearer."""
        N = control_hold.numbers(self)
        frac, _ = control_hold.read(self, self._sense())
        if N is None or frac is None:
            return None, None
        rest = float(N["open_rest"]) - ratios.QUANT * float(N["still_band"])
        rate = float(N.get("speed_open") or N["speed"])
        free = float(frac) + float(N["lag"][0]) + float(N["lag"][1]) * rate + ratios.QUANT * float(N["still_band"])
        return min(free, rest), rest

    def _pl_release(self):
        """Open the jaws through the core's release, as far as frees the thing (_pl_open_goal); the jaws' command
        then goes to their open rest, and they open on while the hand rises. Returns (its result, released):
        released is false when the jaws did not open (they met something at once, or the arm's cap ended the
        opening before they moved)."""
        frac0, _ = control_hold.read(self, self._sense())
        goal, rest = self._pl_open_goal()
        r = control_hold.release(self, goal)
        if "refused" in r:
            return r, False
        opened = float(r.get("frac") or 0.0) - float(frac0 or 0.0)
        r["from_frac"] = None if frac0 is None else round(float(frac0), 4)
        N = control_hold.numbers(self)
        ok = bool(r.get("ended") == "opened" or opened > float(N["still_band"]))
        if ok and r.get("ended") == "opened" and rest is not None:
            self._gripper_cmd = max(float(self._gripper_cmd), rest)
        return r, ok

    def _pl_release_periods(self):
        """The control periods that follow the way down the place's count holds (rcpx/r2/place_count.py asks this
        one figure): the landing's arrival stretch, which the set down runs itself from the band's upper edge
        (place_land: 0 where nothing slows the landing), then the release: the opening at the jaws' own rate and
        the tail it waits."""
        N = control_hold.numbers(self)
        frac, _ = control_hold.read(self, self._sense())
        goal, _ = self._pl_open_goal()
        land = int(self.state["place"].get("land_periods") or 0)    # the landing's arrival stretch runs before it
        if N is None or frac is None or goal is None:
            return land
        dt = float(self.dt)
        rate = float(N.get("speed_open") or N["speed"])
        n_lag = int(math.ceil(float(N["lag"][1]) / dt)) + int(ratios.QUANT)
        return land + int(math.ceil(max(0.0, goal - float(frac)) / (rate * dt))) + n_lag + n_lag

    def _pl_rise_leg(self, facts, B, at=None):
        """The retreat: (one leg straight up until the open fingers stand clear of the thing they let go: how far
        it reached up between them and the room a moving hand keeps; that rise in m)."""
        told = self._told()
        rise = float(facts["above_m"]) + self._pl_room(B, told.q, UP)
        p = np.asarray(told.tip, dtype=float) + UP * rise
        return self._pl_leg("lift", p, told.R, told.q, away_from=None if at is None else [float(x) for x in at]), rise

    def _pl_after_way(self, r, target, plan, facts, intent, t0, rise=True):
        """What follows the way down: let go where it ended and (rise true) rise clear. r: the result of the
        stream or line whose last leg was the way down. Before the jaws open, where the hand was told to be is
        held against the plan. A way that ended at its goal with the hand beside where the plan puts it over the
        place by more than its tracking error does not let go (a leg was not run: nothing is dropped where the
        hand happens to stand); one that ended higher than the plan's end lets go and the receipt says from how
        high. After a stop on a contact the hand stands where the contact left it: the receipt states how far
        that is from the plan. Returns the place's result."""
        st = self.state["place"]
        B = bounds.of(self)
        if not plan.get("landed_down"):       # the landing's arrival stretch, from the band's upper edge
            r = self._pl_land_down(plan, target, facts, intent, r)
            if "refused" in r:
                return self._pl_refuse_motion(r, "lowering the thing (it is held over the place)")
            plan = dict(plan, z_end=plan.get("z_true", plan.get("z_end")), landed_down=True)
        stop = str(r.get("stopped"))
        below = float(facts["below_m"])
        lost = self._pl_held() is None
        landed = stop == "arrived"
        met = stop in ("contact", "cap")            # something that is not the place's own landing ended the way
        above = plan["mode"] == "release_above"
        if not (landed or met or lost or stop in ("goal", "limit")):
            # a way down stopped for another reason (its control periods spent, a tick's stop): its periods are the
            # set down's own leg, with the landing's own figure over the pairs it loaded (final review 2, severe 3
            # (c): a period the robot ran is never left to a record of no leg)
            rec = phase(self, "set_down", f"lowering it: the way down stopped ({stop})", t0, [r])
            strike, band = self._pl_land_after(plan, r)
            rec["strike"] = strike if strike is not None else r.get("strike")
            if band is not None:
                rec["band"] = band
            return self._pl_stopped(r, "lowering it", {"bounds": r.get("bounds")})
        told = self._told()
        track = float(plan.get("track_m") or 0.0)
        # how far over the plan's end the hand stands: where it was told to be, and where it is (a hand that
        # still trails its command when the way ends stands higher than it was told)
        high = 0.0 if plan.get("z_end") is None else max(float(told.tip[2]), float(self._sense().tip[2])) \
            - float(plan["z_end"])
        beside = 0.0 if plan.get("over_xy") is None else float(np.linalg.norm(
            np.asarray(told.tip, dtype=float)[:2] - np.asarray(plan["over_xy"], dtype=float)))
        if not lost and stop in ("goal", "limit") and beside > track:
            return {"said": f"Did not let go: the hand stands {beside * MM:.0f} mm beside where the plan puts it "
                            f"over the place (it can be off by {track * MM:.0f} mm), {high * MM:.0f} mm over the "
                            "planned end; the thing is still held. Look before going on.", "held": True,
                    "placed": False, "stopped": "short", "beside_mm": round(beside * MM, 1),
                    "over_planned_mm": round(high * MM, 1), "bounds": r.get("bounds"),
                    "next": "place it again, or open the jaws (gripper open) to let it go here"}
        felt = (r.get("contact") or {}).get("force_n")
        if landed and high <= track and finite(felt, facts.get("weight_n")) and \
                float(felt) <= (1.0 + ratios.REL) * float(facts["weight_n"]):
            # the way reached its planned end and the reading that ended it is no larger than the held thing's
            # own load: that is the load, not a landing
            landed, stop = False, "goal"
            plan = dict(plan, doubt=list(plan["doubt"]) + [{
                "what": f"at the planned end the still arm read {float(felt):.2f} N, no more than the held thing's "
                        f"own {float(facts['weight_n']):.2f} N", "acted": "not taken for a landing: let go there",
                "would_remove": LOOK_IN}])
        if not lost and not above and plan.get("walled") and plan.get("again") is None:
            # into an opening: a way down that ended on something that is no landing inside is not let go on
            rim = float(target.get("rim_z", plan["base_z"]))
            sig_z = target.get("sigma_z")
            over = float(told.tip[2]) - below > rim - ratios.BAND * (float(sig_z) if finite(sig_z) else 0.0)
            if stop == "contact" or (landed and over):
                out = self._pl_again(r, target, plan, facts, intent, t0, rise, met, high,
                                     float(told.tip[2]) - below - rim)
                if out is not None:
                    return out
        short = high if (stop in ("goal", "limit") and high > track and not above) else None
        if short is not None:
            plan = dict(plan, why=f"the way down ended {short * MM:.0f} mm over its planned end"
                        + (f" ({plan['why']})" if plan.get("why") else ""))
        how = ("lowered until it landed" if landed else
               ("lowered and met something before the planned height" if met else
                ("lowered to the release height" if above else
                 ("lowered into the opening to the planned end; the arm read no load over what it tells" if
                  plan["mode"] == "into" else "lowered to the planned height; the arm read no load over what it "
                  "tells"))))
        rec = phase(self, "set_down", how, t0, [r])
        strike, band = self._pl_land_after(plan, r)       # ONE figure over every pair the landing loaded
        rec["strike"] = strike
        if band is not None:
            rec["band"] = band
        t1 = self.sim_time()
        s = self._sense()
        bottom = float(s.tip[2]) - below
        set_at = np.array([float(target["centre"][0]), float(target["centre"][1]), bottom])
        got, released = ({"ended": "lost", "periods": 0}, True) if lost else self._pl_release()
        if not released:
            phase(self, "release", "told the jaws to open: they did not", t1, [])
            if "refused" in got:
                return got
            return {"said": f"The jaws did not open ({got.get('ended')}): the thing is still held, its bottom at z "
                            f"{bottom * MM:.0f} mm over the place; nothing was set down.", "held": True,
                    "placed": False, "landed": landed, "dropped": False, "stopped": "release_failed",
                    "object_bottom_z_mm": round(bottom * MM, 1), "target_kind": target["kind"],
                    "next": "open the jaws (gripper open) to let it go here, or place it again",
                    "bounds": r.get("bounds")}
        rec = phase(self, "release", "opened the jaws" + (f" ({got.get('ended')})" if got.get("ended") != "opened"
                                                          else ""), t1, [])
        # what letting go can make (place_land.let_go): the thing's own weight on what it lands on and, where it
        # was let go over it, the stop of that fall (the height the place can be off by where none was measured)
        sig_z = target.get("sigma_z")
        fall = max(0.0, bottom - float(target["surface_z"])) if finite(target.get("surface_z")) else None
        if fall is None or (not landed and fall <= track):
            fall = max(fall or 0.0, float(ratios.BAND) * float(sig_z) if finite(sig_z) else track)
        rec["strike"] = place_land.let_go(self, (plan.get("land") or {}).get("pads"), facts, 0.0 if landed else fall,
                                          (strike.get("ran") or {}).get("peak_n"))
        t2 = self.sim_time()
        rr, rose = None, 0.0
        if rise:
            leg, _ = self._pl_rise_leg(facts, B, set_at)
            z0 = float(self._sense().tip[2])
            rr = {"stopped": "goal"} if leg is None else self._pl_stream([leg], ["rise"])
            rose = float(self._sense().tip[2]) - z0
            if "refused" in rr:
                rr = {"stopped": "refused", "why": rr["refused"].get("why"), "refused": rr["refused"]}
            rec = phase(self, "rise", f"rose {rose * MM:.0f} mm clear of it" if rr.get("stopped") == "goal" else
                        f"the rise ended after {rose * MM:.0f} mm ({rr.get('stopped')})", t2, [rr])
            rec["strike"] = rr.get("strike")
        self._pl_relabel(set_at)
        surf = target.get("surface_z")
        over = None if surf is None else bottom - float(surf)
        drop, doubt = None, list(plan["doubt"])
        under = float(target.get("rim_z", plan["base_z"])) - bottom       # (into) how far under the rim it stood
        rim_words = (f"{under * MM:.0f} mm under the rim" if under >= 0 else f"{-under * MM:.0f} mm over the rim")
        if landed and plan["mode"] == "into" and not lost:
            # a contact ended the way down into an opening: the thing landed, or the hand met the rim; which of
            # the two the arm cannot tell, and the floor inside is not seen
            drop = {"height_mm": None, "energy_mj": None, "from_z_mm": round(bottom * MM, 1),
                    "under_rim_mm": round(under * MM, 1),
                    "why": "a contact ended the way down: the thing landed inside, or the hand met the rim"}
            doubt.append({"what": "the floor inside is not seen: a contact ended the way down, by the thing landing "
                                  "or by the hand on the rim",
                          "acted": f"let it go there, its bottom {rim_words}", "would_remove": LOOK_IN})
        elif not landed and not lost:
            h = None if over is None else max(0.0, over)
            sig_z = target.get("sigma_z")
            drop = {"height_mm": None if h is None else round(h * MM, 1),
                    "energy_mj": None if h is None or not finite(facts.get("weight_n")) else
                    round(float(facts["weight_n"]) * h * MM, 2),
                    "from_z_mm": round(bottom * MM, 1), "why": plan.get("why") or (
                        "something was met before the planned height" if met else
                        "the arm read no load over what it tells down to the planned height")}
            if plan["mode"] == "into":
                drop["under_rim_mm"] = round(under * MM, 1)
                seen = target.get("seen_z")
                if finite(seen):        # the floor the pointed ray showed inside: how far over it it was let go
                    drop["height_mm"] = round(max(0.0, bottom - float(seen)) * MM, 1)
                    doubt.append({"what": f"no landing was felt: it was let go {drop['height_mm']:.0f} mm over the "
                                          "floor the pointed ray showed inside", "acted": f"let it go with its bottom "
                                          f"{rim_words}, where the way down ended ({drop['why']})",
                                  "would_remove": LOOK_IN})
                else:
                    doubt.append({"what": "the floor inside is not seen: how far it fell is not known",
                                  "acted": f"let it go with its bottom {rim_words}, where the way down ended "
                                           f"({drop['why']})", "would_remove": LOOK_IN})
            elif not above and not met and (h is None or h <= track):
                doubt.append({"what": "no landing was felt at the height the spot gives"
                                      + (f" (known to {float(sig_z) * MM:.1f} mm)" if finite(sig_z) and sig_z else ""),
                              "acted": "let it go there: it fell by at most what the spot's height is off",
                              "would_remove": LOOK_IN})
            elif not above and not met:
                doubt.append({"what": f"it was let go {h * MM:.0f} mm over the spot: {drop['why']}",
                              "acted": f"let it go from there: a drop of {h * MM:.0f} mm", "would_remove": LOOK_IN})
        rel = {"opened_at_mm": _mm(s.tip), "opened_t_s": round(t1, 4), "bottom_z_mm": round(bottom * MM, 1),
               "bottom_over_surface_mm": None if over is None else round(over * MM, 1), "touchdown_felt": landed,
               "touchdown_by": "the contact stop" if landed else None, "rise_dragged": False, "drop": drop,
               "jaws": got.get("ended"), "release_periods": got.get("periods"),
               "opened_from_frac": got.get("from_frac"),
               "opened_to_frac": None if got.get("frac") is None else round(float(got["frac"]), 4),
               "over_planned_mm": round(high * MM, 1), "beside_mm": round(beside * MM, 1),
               "track_mm": round(track * MM, 1), "rim": rim_words,
               # what the way down's own readings showed (the words say it, never "nothing" over it): the level the
               # guard felt nothing under on the stretch that ended the way, a load that rose, the jaws' gap
               "landing_read": {"f_guard_n": (r.get("strike") or {}).get("f_guard_n"),
                                "load_rose_at_z_mm": (strike.get("ran") or {}).get("load_rose_at_z_mm"),
                                "gap_closed_mm_s": (strike.get("ran") or {}).get("gap_closed_mm_s")}}
        # "placed": let go at the place's own height and not lost: True for now (the evidence of the
        # views decides after, place_spot.arrival); let go after something was met on the way down, over a rim,
        # above the place or higher over the spot than the hand tracks: None (the words say what happened)
        h_mm = None if drop is None else drop.get("height_mm")
        if lost:
            placed = False
        elif met or above:
            placed = None
        elif landed:
            placed = True if plan["mode"] != "into" else None
        else:
            placed = True if h_mm is not None and float(h_mm) <= float(rel["track_mm"]) else None
        out = {"said": self._pl_set_said(landed, lost, met, plan, rel, r), "held": False, "placed": placed,
               "landed": landed, "dropped": lost,
               "stopped": "dropped" if lost else (stop if landed or met else ("release_above" if above else
                                                                             "no_contact")),
               "object_bottom_z_mm": round(bottom * MM, 1), "set_down_mm": _mm(set_at),
               "target_kind": target["kind"], "rose_mm": round(rose * MM, 1), "release": rel,
               "route": r.get("route"), "bounds": r.get("bounds"), "strike": strike,
               "limit": {k: plan["limit"].get(k) for k in ("limit_n", "source", "evidence")},
               "doubt": doubt + list(r.get("doubt") or []) + list(target.get("doubt") or [])}
        if band is not None:
            out["band"] = band
            if band.get("cost") is not None:      # C12: what the stated limit cost the landing, and where
                out["limit"]["cost"] = band["cost"]
        if met:
            out["contact"] = r.get("contact")
        if plan.get("fit"):
            out["spot"] = {"beside_pointed_mm": round(float(np.linalg.norm(
                               np.asarray(target["centre"], dtype=float)[:-1]
                               - np.asarray(target.get("pointed", target["centre"]), dtype=float)[:-1])) * MM, 1),
                           "how": plan["fit"]["how"], "room_mm": round(plan["fit"]["room_m"] * MM, 1),
                           "look": plan["fit"]["look"]}
        if plan.get("again"):
            out["again"] = plan["again"]
        if rr is not None and rr.get("stopped") != "goal":
            out["rise"] = {"stopped": rr.get("stopped"), "why": rr.get("why", "")}
            out["said"] = out["said"].replace("; the jaws opened and the hand rose.", "; the jaws opened. The hand "
                                              f"did not rise clear ({rr.get('stopped')}: {rr.get('why') or 'stopped'}).")
        st.update(label=None, picked_at=None)
        self.note_event("place_place", landed=landed, set_down_mm=_mm(set_at))
        return out

    def _pl_again(self, r, target, plan, facts, intent, t0, rise, met, high, over_rim):
        """A way down into an opening ended on something that is no landing inside (r: its result; over_rim: how
        far over the rim the thing's bottom stands). Where the newest look shows a spot with more room (_pl_fit,
        most, at the end the plan had before the hand's stop lifted it), the hand rises to where the way down
        began, moves the thing over that spot and comes down once more, to the end the hand's stop gives at
        that spot; what follows that second way is the place's result (it is not tried again). None when no
        look shows such a spot, or the bounds refuse the way to it, or the control periods left do not hold it:
        the thing is then let go where the first way ended. A stop on the way to the spot is returned as it
        is, the thing still held."""
        st = self.state["place"]
        B = bounds.of(self)
        told = self._told()
        R = np.asarray(told.R, dtype=float)
        tip = np.asarray(told.tip, dtype=float)
        track = float(plan.get("track_m") or 0.0)
        low = plan.get("z_low", plan.get("z_end"))
        fit = self._pl_fit(target, facts, R, plan.get("z_fit") or low, track, most=True)
        if fit is None:
            return None
        obj = R @ np.asarray(facts["offset_T"], dtype=float)
        below = float(facts["below_m"])
        z_up = st.get("way_z")
        if not finite(z_up) or float(z_up) <= float(tip[2]):
            z_up = float(tip[2]) + float(facts["above_m"]) + self._pl_room(B, told.q, UP)
        word = self._pl_core_intent(intent)
        up = np.array([tip[0], tip[1], float(z_up)])
        over = np.array([fit["xy"][0] - obj[0], fit["xy"][1] - obj[1], float(z_up)])
        under = [float(tip[0]), float(tip[1]), float(tip[2]) - below]
        legs, names, q_prev = [], [], told.q
        for name, kind, p, more in (("rise", "lift", up, {"away_from": under}), ("across", "carry", over, {})):
            leg = self._pl_leg(kind, p, R, q_prev, intent=word, **more)
            if leg is None:
                return None
            legs.append(leg)
            names.append(name)
            if np.asarray(leg["to"]).ndim == 1:
                q_prev = leg["to"]
        held = self._pl_held()
        count, left = self._pl_count(legs, q0=told.q, held=held), self._pl_periods_left()
        if count is None or (left is not None and count > left):
            return None                  # the bounds refuse the way to the spot, or it does not fit: nothing moves
        ended = (f"met something that is no landing ({r.get('why') or r.get('stopped')})" if met else
                 f"ended on a contact {high * MM:.0f} mm over its planned end, the thing's bottom "
                 + (f"{over_rim * MM:.0f} mm over the rim" if over_rim > 0 else "not under the rim"))
        rec = phase(self, "set_down", "lowered: the way down " + ended, t0, [r])
        rec["strike"], band = self._pl_land_after(plan, r)
        if band is not None:
            rec["band"] = band
        t1 = self.sim_time()
        r1 = self._pl_stream(legs, names, held=held)
        if "refused" in r1:
            return self._pl_refuse_motion(r1, "moving the thing to the spot with more room")
        rec = phase(self, "again", f"rose and moved it {fit['moved_m'] * MM:.0f} mm to the spot with the most room"
                    if r1["stopped"] == "goal" else "the way to the spot with more room stopped", t1, [r1])
        rec["strike"] = r1.get("strike")
        if r1["stopped"] != "goal":
            return self._pl_stopped(r1, "moving it to the spot with more room", {"bounds": r1.get("bounds")})
        t2 = self.sim_time()
        told = self._told()
        line = {"what": f"the way down {ended}",
                "acted": f"not let go there: rose, moved it {fit['moved_m'] * MM:.0f} mm to the spot that look "
                         f"{fit['look']} shows with the most room ({max(0.0, fit['room_m']) * MM:.0f} mm) and came "
                         "down once more", "would_remove": LOOK_IN}
        target["centre"] = np.array([fit["xy"][0], fit["xy"][1], float(target["centre"][-1])])
        z_end, hand_z, lifted, _ = self._pl_hand_lift(target, facts, R, float(low), track)
        plan = dict(plan, over_xy=[float(over[0]), float(over[1])], fit=fit, z_end=z_end, hand_stop_z=hand_z,
                    doubt=[d for d in plan["doubt"] if "under the hand's" not in str(d.get("what"))] + [line]
                    + ([lifted] if lifted is not None else []),
                    again={"moved_mm": round(fit["moved_m"] * MM, 1), "why": ended, "contact": r.get("contact")})
        plan.update(self._pl_land_ends(plan, z_end))
        r2 = self._pl_land_down(plan, target, facts, intent)
        if "refused" in r2:
            return self._pl_refuse_motion(r2, "lowering the thing (it is held over the place)")
        r2.update(route=r.get("route"), doubt=list(r.get("doubt") or []))
        return self._pl_after_way(r2, target, dict(plan, z_end=plan["z_true"], landed_down=True), facts, intent, t2,
                                  rise)

    def _pl_set_down(self, target, plan, intent, t0, rise=True):
        """The way down alone, from where the hand stands over the place (the side entry comes in level and
        lowers from there), then let go and (rise true) rise clear."""
        facts = self._pl_hold_facts()
        told = self._told()
        self.state["place"]["way_z"] = float(told.tip[2])
        r = {"stopped": "goal", "bounds": None, "route": "level under the cover"}
        if plan.get("z_end") is not None:
            r = self._pl_land_down(plan, target, facts, intent)
            if "refused" in r:
                return self._pl_refuse_motion(r, "lowering the thing")
            r["route"] = "level under the cover"
            plan = dict(plan, z_end=plan.get("z_true", plan["z_end"]), landed_down=True)
        return self._pl_after_way(r, target, plan, facts, intent, t0, rise)
