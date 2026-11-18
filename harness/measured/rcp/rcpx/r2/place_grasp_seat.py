# SPDX-License-Identifier: AGPL-3.0-only
"""Where the pads stand on the thing after a close, and the re-seat.

A hand with ONE moving jaw pushes the thing against its fixed jaw. Where such a close stalls WIDER than the thing
is across the bearing the jaws were planned on (its width as seen or stated, its error and the jaws' own band), the
thing stands turned against the fixed jaw: two edges hold it, not two faces. It turns between the pads when the hand
moves, comes loose and is caught again by the closing jaw, or leaves the pads in the lift. The take reads it from the
jaws' own gap before the lift and re-seats ONCE inside the same take (it is no second take). A hand
with two moving jaws centres the thing between them and holds it on its corners; the gap alone shows no weak hold
there and nothing is re-seated (measured: such a hand held and lifted the thing as it stood, and a re-seat cost it
half again the take's control periods for nothing).

- read(): for a grasp of the whole thing by a hand with one moving jaw (a wall or a part taken between flat pads
  stalls wider than its seen thickness by its own curve, and no turn mends that): how far the jaws stand wider than
  the thing, and the turn that squares them by the thing's own sizes: a thing w across the planned bearing and l
  across the other (as the look showed it; a width that was stated and not seen is that wide every way, as
  place_grasp takes it) reads w cos(a) + l sin(a) between jaws that are turned a from its narrow side, so
  a = asin(gap / hypot(w, l)) - atan2(w, l). No size of any thing is typed.
- SeatMixin._pk_reseat(): the jaws open as far as the plan opened them, the hand goes back to where it stood open
  around the thing (a jaw that does not move stands on the thing after a close), rises until its fingertips stand
  the take's band over the thing's top, turns about the approach axis through the same point by that angle, comes
  down again, and the jaws close again as they closed (the core's one close), arriving
  from the widest the thing can stand between them by its own sizes (its diagonal). The gap alone does not say which way the
  turn has to go: where the turned jaws took no hold, or stall no narrower than before, the hand turns the other
  way by the same angle from where it began and closes once more: that is the same re-seat, its one correction.
  A re-seat never ends with less than it began with: where no turn stands narrower WITH a hold, the hand goes
  back to the bearing it began at and the jaws close there as the first close did. Every turn and gap is in the
  receipt (evidence "reseat" with its control periods, a doubt line, one phase per close).

A re-seat that the control periods left do not hold, a pose this arm cannot take, or a turn that stopped is not run:
the take goes on with the hold it has and says why. Nothing here reads ground truth or names a robot.
"""

import math

from rcpx import control_hold
from rcpx.r2 import place_grasp, place_pick_count
from rcpx.r2.place_grasp import HALF, MM, line


def read(plan, S, c, band, J=None):
    """{"gap_m", "width_m", "wider_m", "turn_rad", "long_m"} when the close c stalled wider than the thing is
    across the planned bearing by more than band (m), else None (also with no width to go by: nothing says how
    wide it should stand). J (the hand, place_hand.Jaws): how wide the thing stands between the jaws is the
    jaws' own SHAPES' gap at the opening they stalled at, at the depth the thing is taken at (J.reach_m), not
    the gap between their jaw points: a hinged jaw's face meets a thing nearer than its point (a hand with one
    hinged jaw read 50.8 mm by its points where its shapes stood 47.8 mm apart on a cube turned 29.9 deg, and the
    turn from the points' gap was 45 deg for 30)."""
    w, gap = plan.get("expect_m"), c.get("gap_m")
    if w is None or gap is None or not float(w) > 0.0 or plan.get("strategy") != "top" or not plan.get("one_jaw"):
        return None         # only a grasp of the WHOLE thing (a wall or a part between flat pads is no rectangle)
                            # by a hand with ONE moving jaw (module text)
    gap = stands(J, plan, c)
    wider = float(gap) - (float(w) + float(band))
    if not wider > 0.0:
        return None
    long = float(w)
    if S.get("narrow") is not None and S.get("wide") is not None:
        # the thing's extent across the planned bearing (the look's own outline): its other side
        long = place_grasp.extent_along(S["narrow"], S["wide"], S["bearing"], plan["yaw"] + math.pi / 2)
    turn = math.asin(min(1.0, float(gap) / math.hypot(float(w), long))) - math.atan2(float(w), long)
    if not turn > 0.0:
        return None
    return {"gap_m": float(gap), "width_m": float(w), "wider_m": wider, "turn_rad": turn, "long_m": long}


def stands(J, plan, c):
    """How wide the thing stands between the jaws after the close c (m): the jaws' own shapes' gap at the opening
    they stood at and the depth the thing is taken at (J.reach_m), never over the gap between their jaw points;
    the points' gap where the hand states no shapes. None: no reading."""
    gap = c.get("gap_m")
    if gap is None:
        return None
    if J is not None and c.get("frac") is not None and callable(getattr(J, "reach_m", None)):
        shapes = J.reach_m(float(c["frac"]), plan.get("depth_m") if (plan.get("depth_m") or 0.0) > 0.0 else None)
        if shapes is not None and math.isfinite(float(shapes)):
            return min(float(gap), float(shapes))
    return float(gap)


class SeatMixin:
    def _pk_reseat(self, plan, ready, S, c, how, word, ev, doubt, strikes, ran=None):
        """The re-seat of a hold that stands on a corner (module text). c: the close that stands; how: the
        arguments the take closed with; ran: the way down as the hand ran it (m), which every close of this take
        tells the hold as its leg in (the turn of the open jaws is no way in). Returns (the close that stands
        at the end, the hand as it stands)."""
        J = self._pl_jaws
        band = ready["band"]
        seat = read(dict(plan, one_jaw=bool(getattr(J, "one_jaw", False))), S, c, band, J)
        s_low = self._sense()
        if seat is None or not c.get("held"):
            return c, s_low
        yaw0, turn, t_begin = float(plan["yaw"]), float(seat["turn_rad"]), self.sim_time()
        how_known = "stated" if plan.get("width_source") == "stated" else "seen"
        note = {"why": f"the jaws stalled {seat['gap_m'] * MM:.0f} mm apart where it was {how_known} "
                       f"{seat['width_m'] * MM:.0f} mm across (known to {band * MM:.1f} mm): they stand on something "
                       "wider, or on a corner of it",
                "turn_deg": round(math.degrees(turn), 1), "tries": [], "kept": "the hold as it stood"}
        ev["reseat"] = note
        still = float(ready["N"]["still_band"]) * float(ready["N"]["gap_per_span_m"])
        t = plan["approach_axis"]
        first_gap = float(seat["gap_m"])
        first = {"q": plan["q_low"], "yaw": yaw0, "R": plan.get("R")}     # the bearing the take began at
        q_open = plan["q_low"]          # the open hand around the thing at the bearing it stands at
        # the thing was found turned against the bearing it was planned at: every close of the re-seat arrives
        # from the widest it can stand between the jaws, its diagonal by its own sizes, and never from under the
        # gap the jaws just stalled at (measured: the thing reaches at least that far, a width stated short of it)
        again = dict(how, wide_m=max(float(how.get("wide_m") or 0.0), math.hypot(seat["width_m"], seat["long_m"]),
                                     float(seat["gap_m"])))
        won = False
        yaw_open = yaw0
        # the open jaws turn OVER the thing, not about it where it stands: a thing that stands turned between them
        # reaches out to its diagonal, past the fixed jaw's face, and jaws turned through it turn it with them
        # (measured on a hand with one hinged jaw: every turn read the same gap). The hand rises until its fingertips stand
        # the take's own band over the thing's top (the depth the take went under it, the jaws' own low point at
        # that opening), turns there, and comes down at the new bearing.
        rise = (max(0.0, float(plan["depth_m"])) + float(J.low_m(J.frac_for_width(plan["open_m"])))
                + float(plan["band_m"]))

        def over(yaw_, seed):
            q_, _, _, _ = place_grasp._pose(self, J, plan["centre"], float(plan["tip_z"]) + rise, yaw_, t,
                                            plan["open_m"], plan["width_m"], [seed])
            return q_

        def close_at(q_to, did, yaw_to):
            """Open, go back to where the open hand stood, rise over the thing, turn, come down to the open hand's
            pose q_to, close as the take closed. Returns the close, or None with the reason noted when it did not
            run."""
            nonlocal q_open, yaw_open
            t0 = self.sim_time()
            got = control_hold.release(self, ready["open_frac"])
            if "refused" in got or got.get("ended") != "opened":
                note["tries"].append({"not_run": f"the jaws did not open ({got.get('ended', 'refused')})"})
                return None
            legs = [{"kind": "transit", "to": q_open}]
            if q_to is not q_open:
                up_from = over(yaw_open, q_open)
                up_to = None if up_from is None else over(yaw_to, up_from)
                if up_to is not None:
                    legs += [{"kind": "transit", "to": up_from}, {"kind": "transit", "to": up_to}]
                legs.append({"kind": "transit", "to": q_to})
            r = self._pk_stream(legs, word)
            ok = "refused" not in r and r.get("stopped") == "goal"
            self._pk_phase("reseat", (f"opened the jaws and {did}" if ok else "the turn of the open jaws about the "
                                      f"thing stopped ({r.get('stopped', 'refused')})"), t0, r, strikes)
            if not ok:
                note["tries"].append({"not_run": f"the turn stopped ({r.get('stopped', 'refused')})"})
            else:
                q_open, yaw_open = q_to, yaw_to
            t0 = self.sim_time()
            if ran is not None:
                self.__dict__.setdefault("_hold_legs", {}).update(leg_m=float(ran), by_leg=True)
            arm = place_pick_count.follow(self, None) if plan["width_m"] is None or plan.get("follow") else None
            c2 = self._pk_close_again(plan, again, arm, None)
            gap2 = c2.get("gap_m")
            self._pk_phase("close", f"closed on it again ({c2.get('ended')})"
                           + ("" if gap2 is None else f": the jaws stand {gap2 * MM:.1f} mm apart"), t0,
                           {"bounds": None, "strike": place_pick_count.close_strike(self, plan, ready, c2)}, strikes)
            return c2 if ok else {**c2, "_stopped": True}

        for sign in (1.0, -1.0):
            n_close, _ = place_pick_count.close_count(self, plan, ready)
            left = self._pl_periods_left()
            if left is not None and n_close is not None and n_close + n_close > left:
                # the turned close and the close that takes the first hold again must both fit
                note["tries"].append({"not_run": f"a close plans {n_close} control periods and {left} are left"})
                break
            s = self._sense()
            # a thing as wide one way as the other (a width stated and not seen is taken so) stands the same
            # between jaws turned a quarter turn further: those bearings are the same turn for it, tried where
            # the arm cannot take the first (an arm of five joints turns its jaws only as far as its wrist goes)
            square = abs(float(seat["long_m"]) - float(seat["width_m"])) <= float(band)
            q = None
            for k in ((0.0, 1.0, -1.0) if square else (0.0,)):
                yaw = yaw0 + sign * turn + k * HALF * math.pi
                q, R_low, yaw_r, _ = place_grasp._pose(self, J, plan["centre"], plan["tip_z"], yaw, t, plan["open_m"],
                                                       plan["width_m"], [s.q, first["q"], self.model.q_home])
                off = abs((float(yaw_r if yaw_r is not None else yaw) - yaw + HALF * math.pi) % math.pi
                          - HALF * math.pi)
                if q is not None and not off < HALF * turn:
                    # an arm of fewer joints than a pose has freedoms solved the pointing axis alone: its jaws
                    # stand nearer the bearing they were at than the one asked, so this bearing is not taken
                    q = None
                if q is not None:
                    break
            if q is None:
                note["tries"].append({"turn_deg": round(math.degrees(sign * turn), 1),
                                      "not_run": "no arm pose turns the jaws there"})
                continue
            c2 = close_at(q, f"turned them {math.degrees(sign * turn):.0f} deg over the thing", float(yaw_r))
            if c2 is None:
                break
            gap2 = stands(J, plan, c2)
            c = c2
            if c2.get("_stopped"):
                break
            note["tries"].append({"turn_deg": round(math.degrees(sign * turn), 1), "close": c2.get("ended"),
                                  "gap_mm": None if gap2 is None else round(gap2 * MM, 1)})
            plan.update(yaw=float(yaw_r), q_low=q, R=R_low)
            if c2.get("held") and gap2 is not None and (gap2 <= seat["width_m"] + band or gap2 < first_gap - still):
                note["kept"] = ("the jaws stand across it" if gap2 <= seat["width_m"] + band else
                                f"the jaws stand {(first_gap - gap2) * MM:.1f} mm narrower on it")
                won = True
                break
        ran_tries = [x for x in note["tries"] if x.get("gap_mm") is not None or x.get("close") is not None]
        if ran_tries and not won:
            # no turn stands narrower WITH a hold: back to the bearing the take began at, closed as it first closed
            c_back = close_at(first["q"], "turned them back to the bearing the take began at", yaw0)
            if c_back is not None:
                c = c_back
                plan.update(yaw=first["yaw"], q_low=first["q"])
                if first["R"] is not None:
                    plan["R"] = first["R"]
                note["kept"] = ("the hold at the bearing the take began at (no turn stood narrower on it with a "
                                "hold)" if c.get("held") else "no hold after the turns, nor back at the first bearing")
        note["periods"] = int(round((self.sim_time() - t_begin) / float(self.dt)))
        doubt.append(line(note["why"], ("re-seated inside the same take: the jaws opened, turned "
                                        + " and ".join(f"{x['turn_deg']:.0f} deg" for x in ran_tries
                                                       if x.get("turn_deg") is not None)
                                        + f" over it and closed again ({note['kept']}); the re-seat cost "
                                        f"{note['periods']} control periods") if ran_tries else
                          "not re-seated (" + "; ".join(str(x.get("not_run")) for x in note["tries"]) + "): kept the "
                          "hold and ran the test lift", place_grasp.LOOK))
        return c, self._sense()

    def _pk_close_again(self, plan, how, arm, fallback):
        """The core's one close again with the take's own arguments; a refusal leaves the close that stood."""
        c = control_hold.close(self, expect_m=plan.get("expect_m"), arm=arm, width_source=plan.get("width_source"),
                               look=False, **how)
        return c if "refused" not in c else (fallback if fallback is not None else {"ended": "refused", "held": False,
                                                                                    "gap_m": None, "price": ""})
