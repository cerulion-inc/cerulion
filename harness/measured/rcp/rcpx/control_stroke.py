# SPDX-License-Identifier: AGPL-3.0-only
"""What a stroke of control_contact.py counts, waits for and says (C9, C10, C12): its schedule, its budget count
before anything moves, the wait for the hand to stand, a touch the hand already made as it came to rest, whether a
stated limit is shown kept, and the words of a press result. No speed, force, travel or time here is typed; no
robot is named."""

from rcpx import bounds_laws as laws
from rcpx import bounds_limit as limit_of
from rcpx.bounds_ratios import GIVE
from rcpx import control_move_band, control_move_press, control_move_price, control_tail
from rcpx.control_base import _mm

MM = 1000.0
# What the end stop of a part that slides under a press may take, by the stroke's own speed (the force on the part
# plus the stop slope times the speed): always no more than the arm itself can push (the cap the press's guard
# runs with). Whether the call's own force_n bounds it as well is the switch below: True (force_n bounds it too) or
# False (only max_n, a sheet or the site bound a stroke's speed). One switch, so that the choice is one line.
STROKE_UNDER_FORCE_N = True


class StrokeMixin:
    def _schedule(self, plan, p0, d, speed, edge=0.0):
        """Where the command of a stroke stands at each control period of its free travel (m along d from p0): the
        line's plan as the bounds time it. From the travel `edge` on (where a contact can begin, C15; 0: the
        whole stroke) the arrival speed (speed, m/s) lowers the plan's, never raises it: the command follows the
        plan's samples by at most that speed's travel per period, and runs AT it wherever the plan is faster
        (the design rulings: the speed a stroke states is the speed that ran). Before the edge the plan's own speed stands."""
        step = None if speed is None else float(speed) * self.dt
        xs = [float((self.model.fk_tip(q)[0] - p0) @ d) for q in plan.path]
        sched, at = [0.0], 0.0
        for x in xs:
            if step is None or x <= edge:
                at = max(at, x)
            else:
                at = max(at, min(x, max(at, edge) + step))
            sched.append(at)
        end = max(xs, default=0.0)
        while step is not None and step > 0.0 and end - at > 0.0:   # what the arrival speed left of the travel
            at = min(end, at + step)
            sched.append(at)
        return sched

    @staticmethod
    def _ran_speed(sched, edge, dt):
        """The speed the command of a stroke ran at over its arrival stretch (m/s): the median of its steps from
        the travel `edge` on; None for a stroke with no such step."""
        steps = sorted(b - a for a, b in zip(sched, sched[1:]) if a >= edge and b > a)
        return steps[len(steps) // 2] / float(dt) if steps else None

    def _band_of(self, B, told, d, reach, thing):
        """The band of a stroke from where the hand was told to stand (control_move_band.of_stroke): how far all
        that the tool and the held thing sweep is known free, and the first known surface ahead."""
        return control_move_band.of_stroke(self, B, told.tip, d, reach, thing, told.q, self._held_for(None))

    def _band_keys(self, out, B, band, r, v_arr, q, d, detail=False):
        """The receipt's "band" block of a stroke (C15 (5)): carried when it changed what the stroke did (a fast
        part ran before the band), when it prices the lack of a source, or in the full result (C12)."""
        z = r.get("z_end")
        arrive_n = None if z is None or v_arr is None else float(B.f_thr(v_arr)) + float(z) * float(v_arr)
        keys = control_move_band.keys(band, r["sched"], self.dt, v_arr, out.get("ends_n"), out.get("ends_source"),
                                      r.get("plan"), arrive_n, r.get("n_plan"), float(B.v_free(q, d)))
        first, edge = r.get("first_resist_m"), float(band.get("from_m") or 0.0)
        if first is not None and r.get("v_hit"):               # the speed the command had when the contact was met
            out["met_mm_s"] = round(float(r["v_hit"]) * MM, 2)
            if edge > 0.0 and first < edge:
                out["met_before_band"] = True                  # in space that was known free: the bet was lost
        if detail or keys["from_mm"] > 0.0 or "price" in keys:
            out["band"] = keys
        return out

    def _stroke_room(self, told, d, travel, lead, stated):
        """(travel, refusal or None, hold, note) of a press's plan (its path check, ReachMixin._stroke_plan). A
        press whose travel nobody stated plans as far as the arm reaches along its line; where that plan would
        bring a link newly to the surface under the arm, the travel is halved until the plan stands (the design rulings: a
        travel nobody asked for refuses nothing) and the note says how far the stroke may go."""
        bad, hold = self._stroke_plan(told, d, travel + lead + lead)
        was, tell = travel, float(self.bounds.rigid(told.q, d))
        while (bad is not None and not stated and (bad.get("refused") or {}).get("check") == "path_plane"
               and travel > tell):
            travel *= GIVE
            bad, hold = self._stroke_plan(told, d, travel + lead + lead)
        note = None if travel == was else (f"no travel stated: the stroke may run {travel * MM:.0f} mm (further on a "
                                           "link of the arm would meet the surface under it)")
        return travel, bad, hold, note

    @staticmethod
    def _tell_speed(B, q, d):
        """The slowest speed at which this arm resolves a stop along d at q (m/s; C3): its still band travelled
        in one detection time under its own braking, by the bounds' own law. No bound of a stroke that the
        operator's own force states runs the part slower than this (the limit is then not shown kept)."""
        law = getattr(laws, "tell_speed", None) or (lambda band, t, a: laws.arrival_speed(band, 0.0, t, a))
        return float(law(float(B.rigid(q, d)), float(B.t_det()), float(B.a_brake(q, d))) or 0.0)

    def _stroke_hint(self, sched, n_plan):
        """The hint of a stroke's budget refusal: how much of its travel the periods left would cover."""
        left = self._steps_left()
        if left is None:
            return None
        if left <= 0:
            return "0 control periods are left: nothing can move"
        part = sched[min(len(sched), left) - 1]
        more = n_plan - len(sched)
        return (f"ask for a shorter stroke: the first {part * MM:.0f} mm of these {sched[-1] * MM:.0f} mm run in "
                f"the {left} control periods left" + (f" (its end, ramp and hold take {more} more)" if more > 0 else ""))

    def _wait_counted(self, told, d, travel, force_n, **kw):
        """(periods waited, refusal or None). The budget of a stroke, counted before anything runs (C12): the
        control periods of its plan as it will run from where the hand was told to stand, plus the wait for the
        hand to stand there (by its own lag); more than are left: refused budget, nothing ran. Else the wait
        (ex._arrive: guarded; a stop met in it is left in self._tail)."""
        n = self._push(told, d, travel, force_n, count=True, **kw)
        if n["stop"] == "refused":
            return 0, {"refused": n["refused"]}
        wait = control_tail.wait_count(self)
        if self._steps_left() is None or n["periods"] + wait <= self._steps_left():
            return self._arrive(), None
        B, thing, stated = self.bounds, kw.get("thing"), dict(kw.get("intent") or {})

        def count(it):   # the same stroke under another intent (another limit, or none): its plan's own count
            it = {k: v for k, v in it.items() if k != "_no_limit"}
            more = dict(kw, intent=it, speed=self._arrival(B, told.q, d, it, thing)["v"])
            if "stroke_v" in kw:
                more["stroke_v"] = self._arrival(B, told.q, d, it, thing, stroke=True)["v"]
            return int(self._push(told, d, travel, force_n, count=True, **more)["periods"]) + wait

        limit = limit_of.hold_of(B.stated_limit(thing, stated, expected=True))
        return 0, self._budget(n["periods"] + wait, n["hint"], control_move_price.price_of(limit, stated, count))

    @staticmethod
    def _stated(force_n=None, max_n=None, clear=None, hold_s=None, ends_n=None):
        """The call's intent (Q13) as the bounds read it: only what the operator stated. ends_n: the force_n of a
        touch, the force at which it ends (the design rulings: no limit on the way in)."""
        out = {"force_n": force_n, "max_n": max_n, "clear": clear, "hold": hold_s, "ends_n": ends_n}
        return {k: (v if k == "clear" else float(v)) for k, v in out.items() if v is not None}

    @staticmethod
    def _own_touch(out, arr, force_n):
        """the design rulings: the hand's own weight, which times the way in of a touch that states no force, is no limit and
        nobody's word: the receipt's limit keys carry the least limit stated beside it (none: None). That limit
        is said kept only where the slower way in for the hand's weight is itself shown kept and no reading of
        the call passed the limit."""
        if force_n is None and arr["expect"].get("floor") is not None:
            lim, src = arr["expect"]["floor"]
            peak = float(out.get("peak_force_total_n") or 0.0)
            out.update(limit_n=lim, limit_source=src,
                       limit_kept=None if lim is None else bool(arr["kept"]) and not peak > float(lim))
        return out

    def _tail_touch(self, waited, force_n, arr):
        """The result of a touch whose hand met something while it came to rest after the move before (the wait
        of _arrive ended on a guard stop): that contact is what the touch found; the stroke did not start."""
        tail = self._tail
        c = dict(tail.get("contact") or {})
        found = c.get("point_mm") is not None
        out = {"said": ("Touched something before the stroke began: " if found else "Stopped before the stroke "
                        "began: ") + tail["said"] + ".",
               "contact": found, "point_mm": c.get("point_mm"), "normal_est": c.get("normal_est") if found else None,
               "force_n": c.get("force_n"), "id": c.get("id"), "stopped": tail["stopped"], "in_tail": True,
               "surface": c.get("surface"), "at_tip": c.get("at_tip"), "source": c.get("source"),
               "travelled_mm": 0.0, "along_push_mm": 0.0, "asked_force_n": None if force_n is None else float(force_n),
               "limit_n": arr["expect"]["bound_n"], "limit_source": arr["expect"]["bound_source"],
               "limit_kept": self._kept(arr, float(c.get("force_n") or 0.0)),
               "peak_force_total_n": c.get("force_n"), "tip_mm_at_stop": tail.get("tip_mm") or _mm(self._sense().tip),
               "arrive_periods": waited, **self._move_keys(None, waited)}
        if not found:
            out.update(likely_links=c.get("likely_links"), why=tail.get("why") or "the stop was not made by the "
                       "tool at a place it can state: nothing was stored in robot://world")
        elif tail.get("why"):
            out["why"] = tail["why"]
        return out

    def _touch_none(self, out, stop, pstop, pwhy, max_mm, e, r):
        """The result of a touch that met nothing: why its stroke ended (its travel, the arm's reach, a slip, a
        stop request, its time)."""
        if stop == "travel" and pstop == "goal":
            stopped, why = "goal", f"felt nothing within {max_mm:g} mm"
        elif stop == "travel":
            stopped, why = "limit", pwhy or self._limit_why(e.q)
        elif stop in ("slipped", "off_axis"):
            stopped, why = "slipped", "the tip slid sideways off its line while something resisted"
        elif r["asked"]:
            stopped, why = stop, ""
        else:
            stopped, why = stop, "the touch ran out of time before its travel"
        return {"said": f"No contact: {why or stopped}.", "contact": False, "point_mm": None, "normal_est": None,
                "stopped": stopped, "why": why, **out}

    @staticmethod
    def _kept(arr, peak_n, strike_n=None):
        """Whether a stated limit is shown kept: what the bounds' arrival says of the speed, and never when a
        reading of this call passed the limit, nor when the strike an end stop could have taken (strike_n) does.
        None with no limit stated."""
        limit = arr["expect"]["bound_n"]
        if limit is None:
            return None
        return bool(arr["kept"]) and not max(float(peak_n), float(strike_n or 0.0)) > float(limit)

    def _press_words(self, out, seen, legs, r, stop, gave, moved_at, t_res, peak, felt):
        """The sentences of a press result: what happened first (said), why a part is said to have moved, a part
        that sprang back, where the contact was lost, a tip that left its line."""
        if moved_at is not None:
            out["moved_at_n"] = round(moved_at, 2)
        if t_res is None:
            said = f"Felt nothing over {out['travel_mm']:g} mm."
        elif stop == "moved":
            said = (f"The part moved at {out['moved_at_n']:g} N; the ramp stopped there: state force_n or "
                    "travel_mm to push it on.")
        else:   # the force it pressed to, from the reading at the stop where one reading ended it (a cap)
            out["peak_force_n"] = round(control_move_press.pressed_to(out["peak_force_n"], legs, stop), 2)
            said = f"Pressed to {out['peak_force_n']:g} N over {out['travel_mm']:g} mm; stopped: {stop}" + (
                f" ({r.get('cap_why')})." if stop == "cap" and r.get("cap_why") else ".")
        out = {"said": said, **out}
        if seen["moved_part"]:
            out["moved_part_why"] = seen["moved_part_why"]
        elif gave:
            out["moved_part_why"] = "the force along the push fell under half its peak while the tip went on"
        if out["sprang_back"]:
            out["sprang_back_why"] = (f"the part moved {seen['resisted_mm']:.1f} mm and came back "
                                      f"{out['sprang_back_mm']:.1f} mm as the push eased: it has not latched; push "
                                      "further from its pivot or a little harder")
        lost = next((lg["lost_m"] for lg in reversed(legs) if lg.get("lost_m") is not None), None)
        if lost is not None and t_res is not None:
            out["contact_lost_along_mm"] = round(lost * MM, 1)   # the part stopped pushing back here (FINDINGS P5)
        if gave or (seen["moved_part"] and not out["sprang_back"]):   # and voids the passage it can reach
            out.update({"void": v} if (v := self._stale_near(seen["first_resist_mm"], "a press moved the part there",
                                                             felt.resisted)) else {})
        if stop == "off_axis":
            out["why"] = (f"the tip left its line by {r['lateral_m'] * MM:.1f} mm, over the tool's half width, "
                          f"with {peak:.1f} N along the push: the part takes the push across its line (a hinge, a "
                          "stop or an edge of the tool); change the direction or the place before more force")
        return out
