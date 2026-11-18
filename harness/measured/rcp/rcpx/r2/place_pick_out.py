# SPDX-License-Identifier: AGPL-3.0-only
"""The receipt of the pick of package place (round six: the design notes C12, C14): what a pick says of its takes,
and what it leaves for the place.

- _pk_extent: the held thing in the tool frame at the grasp (the core's record carries it: C8, C14).
- _pk_stands: after the test lift, where it stands in the hand (rcpx/r2/place_hold.py stands), on the record.
- _pk_result: said first; the take's control periods beside its plan's count and the waits it is owed; doubt
  (every line {what, acted, would_remove}); the largest strike of its motions; on held the state
  the place reads. With nothing held the jaws are open again when the tool returns, the receipt says what a
  fresh look shows at the spot (place_pick_again.shows) and the ONE ready call that takes it again there.
Nothing here reads ground truth or names a robot. Lengths m, forces N.
"""

import math

import numpy as np

from rcpx import control_hold
from rcpx.control_base import _mm
from rcpx.r2 import place_grasp, place_hold, place_pick_again
from rcpx.r2.place_grasp import HALF, MM, UP, line
from rcpx.r2.place_hand import rect


def _bounds(ex):
    from rcpx import bounds
    return bounds.of(ex)


def size(strike):
    """The size of a strike as stated: its peak, else the larger of the body's model figure and its floor."""
    keys = ("peak_n",) if strike.get("peak_n") is not None else ("model_peak_n", "at_least_n")
    vals = [strike.get(k) for k in keys]
    vals = [float(v) for v in vals if isinstance(v, (int, float))]
    return max(vals) if vals else -math.inf


class PickOut:
    def _pk_extent(self, plan, S, s_low, gap):
        """The held thing in the tool frame at the grasp: (its corners, how far it reaches below the tip m, its
        half width about the pointing axis m). From the seen outline, its top and what it stood on; with no
        outline a box as wide as the jaws stand, from the top to what it stood on."""
        J = self._pl_jaws
        tip, Rl = np.asarray(s_low.tip, dtype=float), np.asarray(s_low.R, dtype=float)
        top = max(float(plan["top_z"]), float(tip[-1]))
        sup_z = plan.get("support_z")
        bottom = float(tip[-1]) if sup_z is None else sup_z
        if S["height"] is not None:
            bottom = float(S["top_z"]) - S["height"]
        bottom = min(bottom, float(tip[-1]))
        if S["narrow"] is not None:         # a hand that came down off its middle: the jaws brought it to theirs
            mid = np.asarray(S["centre"], dtype=float)[:2] \
                + plan.get("shift_m", 0.0) * np.array([math.cos(plan["yaw"]), math.sin(plan["yaw"])])
            flat = rect(mid, HALF * S["narrow"], HALF * S["wide"], S["bearing"])
        else:
            w = HALF * (gap if gap else plan["open_m"])
            c = tip + Rl @ J.gap_centre_T(J.frac_for_width(gap if gap else plan["open_m"]))
            flat = rect(c[:2], w, w, self._pl_jaw_bearing(Rl))
        corners = [[float(v) for v in Rl.T @ (np.array([x, y, z]) - tip)] for x, y in flat for z in (top, bottom)]
        return corners, max(0.0, max(c[0] for c in corners)), max(math.hypot(c[1], c[2]) for c in corners)

    def _pk_stands(self, took, s_close, s_now, doubt):
        """After a held test lift: where the thing stands in the hand (place_hold.stands), stated on the core's
        record (the extent the take read stays as it was read). A thing that turned or moved between the pads
        by more than the reading's own band is a doubt line; so are a thing that hangs from the pads and swings,
        and a reading that could not be made of a thing the look made out (the place then holds the pose before
        the close, as it does where the reading lies within its band of that pose)."""
        corners, hang, half = took["extent"]
        tip, at = np.asarray(s_close.tip, dtype=float), took["S"]["centre"]
        mid = np.asarray(s_close.R, dtype=float).T @ (np.array([at[0], at[1], tip[-1]]) - tip)
        mid[0] = hang               # the middle of its bottom, in the tool frame as the jaws closed
        got = place_hold.stands(self, took.get("seen") or {}, s_close, s_now, mid)
        took["stands"] = got
        if not got.get("read"):
            if took["S"]["narrow"] is not None:
                doubt.append(line(f"where it stands in the hand after the lift was not read ({got['why']})",
                                  "the place holds it where the look before the take showed it", place_grasp.LOOK))
            return
        control_hold.stated(self, {"stands": got})
        if not got["moved"]:        # within the reading's own band of the pose before the close: that pose stands
            return
        if got["swings"]:           # it hangs from the pads: said, and no pose to plan a place with
            doubt.append(line(f"it hangs turned {math.degrees(got['turn_rad']):.0f} deg in the hand, its middle "
                              f"{got['moved_m'] * MM:.0f} mm from where it was taken (one depth picture of camera "
                              f"{got['camera']} after the lift): more than half its width "
                              f"({HALF * got['narrow_m'] * MM:.0f} mm), so it swings from the pads",
                              "the place holds it by the pose before the close: one look does not say where a "
                              "thing that swings will be at the place", place_grasp.LOOK))
            return
        doubt.append(line(f"it turned {math.degrees(got['turn_rad']):.0f} deg and its middle moved "
                          f"{got['moved_m'] * MM:.0f} mm between the pads as it came up (one depth picture of "
                          f"camera {got['camera']} after the lift, known to {got['sigma_m'] * MM:.1f} mm)",
                          "the place holds it where that picture shows it in the hand, not where it stood before "
                          "the close", None))

    def _pk_item(self, what, value, evidence, lift, source="model"):
        from rcpx import bounds_report as report
        return report.item(what, value, source, evidence, lift)

    def _pk_owed(self, took, planned):
        """The waits by motion a take ran that its plan does not count (the design rulings: owed by the motion core and
        the hold, listed, never gated): the way down's end, the close's arrive, the rest read past its settle
        window; and the run's periods less the plan's."""
        ph = {p["phase"]: p for p in self.state["place"].get("phases") or []}
        rest = ((took.get("evidence") or {}).get("held_evidence") or {}).get("rest_read_periods")
        return {"way_end_wait": (ph.get("over and down") or {}).get("waiting"),
                "close_arrive": (took.get("close") or {}).get("arrive_periods"),
                "rest_read": None if rest is None or planned is None else max(0, int(rest) - int(planned["rest"])),
                "run_less_plan": None if planned is None or took.get("periods") is None else
                int(took["periods"]) - int(planned["periods"])}

    def _pk_result(self, st, seen, took, doubt, strikes):
        """The pick's result from its ONE take (C12, C14; the design rulings): said first; the take's periods beside
        its plan's count, the waits it is owed; doubt, strike; on held the state the place reads and the core's
        record with its extent; with nothing held what a fresh look shows at the spot and the one ready call."""
        B, J = _bounds(self), self._pl_jaws
        if "refused" in took:
            return took
        plan, ev = took["plan"], took["evidence"]
        planned = took.get("planned") if isinstance(took.get("planned"), dict) else None
        ran = (f" The take ran {took.get('periods')} control periods" + ("." if planned is None else
               f"; its plan counted {planned['periods']} (the way {planned['way']}, the close {planned['close']}, "
               f"the lift {planned['lift']}, the rest read {planned['rest']})."))
        top = max(strikes, key=lambda ns: size(ns[1])) if strikes else None
        away = plan.get("from_point_m")
        base = {"strategy": plan["strategy"], "grasp": plan["why"], "approach_axis": plan["approach_axis"],
                "from_point_mm": None if away is None else round(float(away) * MM, 1),
                "phases": st["phases"], "evidence": ev, "takes": 1, "take_periods": [took.get("periods")],
                "planned_periods": planned, "periods_owed": self._pk_owed(took, planned), "doubt": doubt,
                "strike": None if top is None else dict(top[1], phase=top[0]),
                "predicted_peak_n": None if top is None else top[1].get("peak_n")}
        if not took.get("held"):
            if control_hold.held(self) is not None or took.get("close") is not None:
                t0 = self.sim_time()
                control_hold.release(self)              # the jaws are open again when the tool returns
                rec = self._pk_phase("open", "opened the jaws again", t0)
                # (cs, not st: st is the place's own state, whose "held" is set false just below)
                cs = (took.get("close") or {}).get("strike") if isinstance(took.get("close"), dict) else None
                if isinstance(cs, dict) and cs.get("pad_n") is not None:
                    # jaws that open on what they closed on: each pad's load on it only falls, and what it was
                    # pressed against gives back no more than the close itself made (the close's own figures)
                    rec["strike"] = {"peak_n": max(float(cs["pad_n"]), float(cs.get("other_n") or 0.0)),
                                     "pad_n": cs["pad_n"], "other_n": cs.get("other_n"), "at_least_n": None,
                                     "model_peak_n": None, "link": None, "gentle": None,
                                     "z_source": "the close's own figures: opening the jaws makes no more than "
                                                 "the close made"}
            st.update(held=False)
            shows = place_pick_again.shows(self, seen, took)       # the operator decides
            doubt.append(line("the take did not hold: " + str(took.get("said")).rstrip("."),
                              "not taken a second time (a pick takes once): " + shows["said"], shows["ready"]))
            blind = "" if took["S"]["narrow"] is not None else "; no thing was made out at the point it was given"
            return self._pk_out(False, "Not holding it: " + str(took.get("said")).rstrip(".") + ". It was not taken "
                                "a second time (a pick takes once): " + shows["said"] + blind + "." + ran,
                                fingers=took.get("fingers", "not_closed"), stopped=took.get("stopped"),
                                contact=took.get("contact"), width_mm=took.get("width_mm"), shows=shows,
                                hint=took.get("hint") or "look", **base)       # ONE ready call (C12)
        more = ran
        S, s_low, s1, c = took["S"], took["s_low"], took["s1"], took["close"]
        corners, hang, half = took["extent"]
        rec = control_hold.held(self) or {}
        q, target = J.reading(s1.gripper, self._gripper_cmd)
        w1 = J.width_m(q) if q is not None else 0.0
        squeeze = J.squeeze_n(q, target)
        weight = dict.get(rec, "weight_n")
        mass = float(dict.get(rec, "mass_kg") or 0.0)
        sup_z, height = plan.get("support_z"), S["height"]
        if height is None and sup_z is not None:
            height = plan["top_z"] - sup_z
        sig_sup = place_grasp.support_at(B, plan["centre"])[1]
        sigma_hang = math.sqrt((sig_sup or 0.0) ** 2 + place_grasp.num(B.rigid(s1.q, UP), 0.0) ** 2)
        picked = [plan["centre"][0], plan["centre"][1], plan["top_z"]]
        frac_now = J.frac_of_q(q) if q is not None else 0.0
        st.update(held=True, dropped=False, width_m=w1, lead_m=w1 - (J.width_m(target) if target is not None else w1),
                  squeeze_n=squeeze, hang_m=hang, height_m=height, support_z=sup_z, colour=None, picked_at=picked,
                  offset_T=J.gap_centre_T(frac_now).tolist(), open_w=plan["open_m"], weight_n=float(weight or 0.0),
                  since_call=self._calls, report=None, mass_kg=mass, extent_m=corners, sigma=S["sigma_xy"],
                  sigma_hang_m=sigma_hang, slide_room_m=max(0.0, (J.finger_len_m or 0.0) - plan["depth_m"]),
                  proofs=[], accel_ok=None, s_slip_n=None, s_slipped_n=None, strategy=plan["strategy"],
                  approach_axis=plan["approach_axis"])
        self.note_event("place_pick", held=True, width_mm=round(w1 * MM, 1), strategy=plan["strategy"],
                        support_z_mm=None if sup_z is None else round(sup_z * MM, 1))
        load = str(dict.get(rec, "said") or "the load is not read yet")
        if dict.get(rec, "kind") == "attached":
            doubt.append(line(load, "the hand rose with the jaws on it: taken as held", None))
        said = (f"Holding it by {plan['why'].split(',')[0]}: the jaws stand {w1 * MM:.0f} mm apart on it"
                + ("" if squeeze is None else f" and press with {squeeze:.1f} N")
                + f"; after a {took['rise'] * MM:.0f} mm test lift: {load}." + more)
        got = took.get("stands") or {}
        stands = {"read": False, "why": got.get("why")} if not got.get("read") else {
            "read": True, "camera": got["camera"], "moved": got["moved"], "swings": got["swings"],
            "turn_deg": round(math.degrees(got["turn_rad"]), 1), "moved_mm": round(got["moved_m"] * MM, 1),
            "middle_tool_mm": _mm(np.asarray(got["middle_T"])), "known_to_mm": round(got["sigma_m"] * MM, 1),
            "band_mm": round(got["band_m"] * MM, 1)}
        return self._pk_out(True, said, fingers="stalled_on_object", width_mm=round(w1 * MM, 1),
                            object_width_mm_est=None if c.get("gap_m") is None else round(c["gap_m"] * MM, 1),
                            object_height_mm_est=None if height is None else round(height * MM, 1),
                            support_z_mm=None if sup_z is None else round(sup_z * MM, 1),
                            hangs_below_tip_mm=round(hang * MM, 1), half_width_mm=round(half * MM, 1),
                            tip_mm=_mm(s1.tip),
                            jaw_bearing_deg=round(math.degrees(self._pl_jaw_bearing(s_low.R)), 1),
                            picked_at_mm=_mm(np.asarray(picked)), colour_seen=None, colour_px=None,
                            squeeze_n=None if squeeze is None else round(squeeze, 2), mass_kg=round(mass, 4),
                            weight_n=None if weight is None else round(float(weight), 3), s_slipped_n=None,
                            stands=stands, **base)

    def _pk_out(self, held, said, **rest):
        """A pick's result: it leads with its sentence and totals its control periods."""
        out = self._pl_out(held, said, **rest)
        out["periods_used"] = int(self._pk_n[0])
        out["periods"] = [[p["phase"], p["periods"]] for p in out.get("phases") or []]
        return out

    def _pl_out(self, held, said, **rest):
        """A result that leads with its sentence."""
        out = {"said": said, "held": bool(held)}
        out.update({k: v for k, v in rest.items() if k not in ("held", "said")})
        return out
