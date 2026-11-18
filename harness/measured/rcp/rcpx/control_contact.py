# SPDX-License-Identifier: AGPL-3.0-only
"""Contact primitives of the Executor: touch, press and the stroke they share (_push). reach_check,
measure_base_give and what follows a stroke (ease, back-off, retrace) are in control_reach.py, the base check in
control_basecheck.py. No speed, force, travel or time here is typed: each comes from the bounds (rcpx/bounds.py).

A stroke runs along a line and means its contact (an expected contact, control_expect.py). It runs at the body's
speed only through space known free and at the speed solved for the force it ends at from its band on (C15 2 a;
a touch: force_n, else the hand's own weight; the approach of a press: force_n, else the ramp's first force,
control_move_press.py); a limit somebody stated (max_n, a sheet, the site, a rating: B.stated_limit) is kept at
the speed B.arrival gives it. A touch ends on the first reading over the contact threshold of the speed it then
has. Where the part first resists a press sets its command back onto the measured pose and from there the stroke
is LED BY FORCE: the target rises by one continuous ramp (B.force); the command closes the gap between target and
reading in one detection time and never leads the tip by more than the force asked over the servo's stiffness, so
a rigid part is loaded as the ramp says, and a part that slides (its stiffness read from a travel the arm's own
give does not explain) is followed under the bound of what its end stop could take. A part that slides can run into
its end stop, and a stop makes the force on the part plus the stop slope times the speed: the command never runs
faster than keeps that at or under the force asked and what the arm can push (its guard's cap), never
slower than the slowest speed at which this arm resolves a stop (C3: a limit under what the arm can tell is acted
on at that speed and not shown kept); max_n, a sheet or the site bound it further. That pace holds the whole stroke
once the part moved, and a dip of the readings is no new arrival (control_move_press). The result states the
fastest speed the part ran and the largest strike an end stop could have taken (limit_kept false when a reading or
that strike passed the limit). Before it starts a stroke counts its plan's control periods as it will run (C12:
the schedule at the arrival speed, ramp, hold, settle, ease, the wait for the hand to stand) and is refused budget
when more are needed than are left. The stops read the tip's own motion: the part MOVED once the tip went on past
what the arm's own give explains (B.rigid at that force); a GIVE is the force under GIVE of its peak while the tip
goes on that far; OFF AXIS is the tip leaving its line by the tool's own half width (slipped: the force gone as
well). After the stroke the tool is eased until the readings are quiet and backs off following the part. No robot
is named here; the joint split of rounds mode is the chain order.
"""

import collections
import math

import numpy as np

from rcpx import bounds
from rcpx import bounds_laws as laws
from rcpx import control_expect, control_move_band, control_move_press, control_tail
from rcpx.bounds_ratios import GIVE
from rcpx.context import refusal
from rcpx.control_base import Guard, _mm, primitive
from rcpx.control_basecheck import MODES, BaseCheckMixin
from rcpx.control_reach import LEADING, NAMES, ReachMixin, _dir_name, _refused, ramp_seconds  # noqa: F401
from rcpx.control_stroke import STROKE_UNDER_FORCE_N, StrokeMixin

MM = 1000.0


class ContactMixin(StrokeMixin, BaseCheckMixin, ReachMixin):
    def _push(self, s0, d, travel, force_n, speed=None, hold_s=None, mask=None, stop_on_force=False, stall=False,
              q_start=None, max_s=None, feel=None, hold=None, stop_on_give=False, *, ramp=None, thing=None,
              intent=None, until_moved=False, line=None, stroke_v=None, count=False, band=None):
        """One stroke along d (unit, base) from s0, at most travel. feel truthy: a touch (ends "contact" on the
        first reading over the threshold of its speed). Else a press to force_n (None: to the ramp's ceiling): see
        the module docstring. speed (m/s: the arrival speed of a stated limit, for the travel before anything
        resists), stroke_v (m/s: the same for a press's stroke once the part yields; None: the body's) and max_s
        only ever lower the derived ones. ramp: the ForceSpec to run (None: B.force for force_n). until_moved: end
        "moved" once the part moves
        (a ramp with no stated force found the force that moves it). line: the stroke's planned joint path, when
        the caller has it. Returns a dict: stop, s, q_des, "done" (the MEASURED joint path), "free" (its part
        before anything resisted: the path a retreat may retrace), "felt", peak_n (along the push), peak_total_n
        (the whole force), reached, gave, moved, moved_at_n, slipped, travel_m, lateral_m, across_m,
        first_resist_m, k_c, v_hit, impulse_ns, items, periods, v_part (the fastest the tip ran along d once the
        part moved), strike_n (the largest an end stop could have taken: the force on the part plus the
        stop slope times the tip's speed), plan. A stroke the bounds refuse, or one whose plan needs more control
        periods than are left, comes back with stop "refused" and the refusal under "refused". count: nothing
        moves; {"stop": "count", "periods": the plan's control periods, "hint"}. band (control_move_band.of_stroke):
        the stroke runs at the body's own speed up to the travel from which a contact can begin and at `speed`
        from there on (C15); none: the whole free travel at `speed`."""
        B = bounds.of(self)
        dt, t_det, t_set = B.dt(), B.t_det(), B.t_settle()
        speed = None if speed is None or not float(speed) > 0.0 else float(speed)
        hold = self.model.n < 6 if hold is None else hold   # see PlanMixin._rot_err
        q_des = np.asarray(s0.q if q_start is None else q_start, dtype=float).copy()
        p0, R0 = s0.tip.copy(), s0.R.copy()
        p_cmd0 = self.model.fk_tip(q_des)[0]
        intent = dict(intent or {})
        whole = True   # the line runs the whole travel asked (a caller that hands its line over has cut the travel)
        if line is None:
            line, verdict, _, _ = self._line(q_des, d, travel, hold)
            whole = verdict == "goal"
        self._travel_direction, self._period_kind = d, "moving"
        kind = "touch" if feel else "press"
        edge = float((band or {}).get("from_m") or 0.0)   # the travel from which a contact can begin (C15)
        line, expect = control_move_band.split(self, B, q_des, line, p_cmd0, d, edge, intent, kind, thing)
        plan = B.plan(q_des, list(line), kind, target=thing, intent={**intent, "expect": "contact"}, expect=expect)
        if plan.refused:
            return _refused(plan.refused, s0, q_des)
        sched = self._schedule(plan, p_cmd0, d, speed, edge)   # the free travel, period by period
        ticks = np.arange(len(sched), dtype=float)
        v_top = float(np.max(np.diff(sched))) / dt if len(sched) > 1 else 0.0
        v_free = B.v_free(q_des, d)
        v_top = v_top if v_top > 0.0 else v_free
        k_s = B.k_servo(q_des, d)
        z_end = B.slope(q_des, d, thing)[0]   # the stop slope here (None: unresolved)
        tell = B.rigid(q_des, d, 0.0)   # the least tip travel the readings can tell from none
        w_tool = B.standoff(0.0)        # no speed, no thing: the tool's own half width
        items, top, eps = list(plan.items), None, 0.0
        if not feel:
            if ramp is None:
                ramp = B.force("press", thing, q_des, d, {**intent, "force_n": float(force_n)})
            top = ramp.ceiling_n if force_n is None else min(float(force_n), ramp.ceiling_n)
            items = list(ramp.items) + items
            eps = ramp.rate_n_s(0.0) * t_det   # the least step of the ramp: what the estimate can tell apart
        # a press may lead the tip past its travel (the lead is its force); a touch's command ends with its line
        lead_cap = sched[-1] if feel else travel + top / k_s + tell + top / k_s
        n_set = max(1, int(math.ceil(t_set / dt)))
        n_det = max(1, int(math.ceil(t_det / dt)))
        # the stroke's own plan: its free travel as scheduled
        planned = len(sched) * dt + t_set
        if not feel:
            planned += ramp_seconds(ramp.rate_n_s, ramp.first_n, top, dt) + travel / v_top + (hold_s or t_set)
        n_plan = control_tail.stroke_count(planned, dt)   # the budget count: the plan as it will run (C12)
        if count:
            return {"stop": "count", "periods": n_plan, "hint": self._stroke_hint(sched, n_plan), "sched": sched}
        over = self._budget(n_plan, self._stroke_hint(sched, n_plan))
        if over is not None:
            return _refused(over, s0, q_des)
        limit = B.limit_s(planned)
        n_max = int(math.ceil((limit if max_s is None else min(limit, float(max_s))) / dt))
        # a touch stops on contact; a press means its contact: only the cap and a stop request end it. The cap is
        # the arm's own push (and a stated rating): the jaw motor's force range is none
        g = Guard(self, spec=plan.guard) if feel else Guard(self)
        g.cap_n = control_expect.arm_cap(self, q_des, d, g.cap_n)
        recent = collections.deque([(s0, q_des.copy())], control_expect.ring(self))
        c, load, F_t, k_c, v_cmd, v_part, strike, v_tip = 0.0, False, 0.0, None, 0.0, 0.0, 0.0, 0.0
        most = None if feel else (min(top, g.cap_n) if STROKE_UNDER_FORCE_N else g.cap_n)   # an end stop's most
        v_tell = self._tell_speed(B, q_des, d)       # never slower than the slowest speed a stop is resolved at
        peak, tip_at_peak, tr_at_peak, peak_total = 0.0, p0.copy(), 0.0, 0.0
        reached, gave, moved, moved_at, slipped, hold_left = False, False, False, None, False, hold_s
        first_resist, lat_max, hist, done = None, 0.0, [], [np.asarray(s0.q, dtype=float).copy()]
        n_free, t_resist, lost, quiet, at_end, held_n = None, None, None, 0, 0, 0
        lat_fix = np.zeros(3)   # integral hold of the tip on the push line: the servo's sideways give is undone
        stop, s, f_prev, tr, v_hit, impulse, v_was, v_met, rise = None, s0, None, 0.0, 0.0, 0.0, 0.0, None, 0.0
        if q_start is not None and not feel:
            # a stroke that goes on from a loaded command (the second stage of a round): the load the first
            # stage's joints carry stays on the part, the ramp goes on from the force that is there
            load, F_t, first_resist, n_free, t_resist = True, max(float(s0.F @ d), ramp.first_n), 0.0, 0, s0.t
        for _ in range(n_max):
            Fd = float(s.F @ d)   # the law reads the last reading; the stops below read it held over two
            if not load:
                # one period further along the plan from where the command stands (the plan's own speed there)
                nxt = float(np.interp(np.interp(c, sched, ticks) + 1.0, ticks, sched))
                if moved and not feel:   # the readings went quiet behind a part that moved: the stroke's own pace
                    v_q = control_move_press.quiet_pace(v_free, stroke_v, most, Fd, z_end, v_tell)   # where the
                    nxt = max(nxt, min(float(sched[-1]), c + v_q * dt))   # plan, timed for the approach, is slower
                v_cmd = max(0.0, nxt - c) / dt
                c = min(lead_cap, max(c, nxt))
            elif not (stop_on_force and reached):
                # the stroke is led by force (C9): the target gains what the ramp gives at its own force per
                # second from the force there (held over two readings), up to the ceiling; the command closes the
                # gap between that target and the READING in one detection time and never leads the measured tip
                # by more than the force asked over the servo's own stiffness. A part that slides is followed as
                # fast as that force moves it, never faster than keeps what its end stop would take (the force on
                # it plus the stop slope times the speed) at or under the force asked and the arm's own push. Once
                # the part moved under a ramp with no stated force the target stays. A stated limit (stroke_v:
                # max_n, a sheet, the site) bounds the speed of a part that runs (it slides, or it moved); while
                # it holds, the tip stands and the lead is force over stiffness, which the ramp keeps in bounds
                if not (moved and force_n is None):
                    F_t = max(F_t, min(top, hist[-1][1] if hist else F_t))
                    F_t = min(top, F_t + ramp.rate_n_s(F_t) * dt)
                v = laws.load_speed(F_t, Fd, laws.series(k_s, k_c), t_det)   # None: the part slides, follow it
                v_most = v_free if stroke_v is None or not (v is None or moved) else min(v_free, float(stroke_v))
                if (v_end := control_move_press.end_stop_speed(most, Fd, z_end, v_tell)) is not None:   # no faster
                    v_most = min(v_most, max(0.0, v_end - max(0.0, v_tip - v_end)))   # than its end stop takes: a
                v_cmd = (v_most if F_t > Fd else 0.0) if v is None else min(v, v_most)   # part may stand at once
                c = min(lead_cap, max(tr, min(tr + top / k_s + tell, c + v_cmd * dt)))
            f_res = B.f_thr(v_cmd)   # the contact threshold of the speed the command has now
            if feel:
                g.force_n, g.one_n = f_res, B.f_thr(v_cmd, 0.0, 1)
            p_cmd = p_cmd0 + d * c + lat_fix
            if mask is None:
                q_des, _, _ = self._track(q_des, p_cmd, R0, hold)
            else:
                q_des = self._track_mask(q_des, p_cmd, mask)
            s = self._period(q_des)
            done.append(s.q.copy())
            recent.append((s, q_des.copy()))
            delta = s.tip - p0
            tr = float(delta @ d)
            lat = delta - tr * d
            lat_max = max(lat_max, float(np.linalg.norm(lat)))
            f_now = float(s.F @ d)
            # a press reads the force held over two periods (as the guard counts contact); a touch the last one
            Fd = f_now if (f_prev is None or feel) else min(f_now, f_prev)
            f_prev = f_now
            rise = max(rise, s.Fmag - (float(recent[-2][0].Fmag) if len(recent) > 1 else 0.0))   # in one period
            peak_total = max(peak_total, s.Fmag)
            resists = s.Fmag > f_res
            v_now = max(0.0, float((self.model.jac(s.q)[:3] @ s.qd) @ d))   # the measured tip speed along d
            if resists:
                self._scene_touch(s)   # a load over the threshold: the scene may have changed (C10)
                impulse += s.Fmag * dt
                if n_free is None:   # the first resisting reading: where, when, how fast the command came in and
                    n_free, t_resist, v_hit, first_resist = len(done) - 1, s.t, v_cmd, tr   # the tip MET it
                    v_met = max(v_was, v_now)
                if not feel and not load and control_move_press.same_contact(lost, tr, tell, Fd, k_s):
                    c, load = min(c, tr + top / k_s + tell), True   # a dip of the readings: the command keeps its lead
                elif not feel and not load:   # stand on the measured pose: the servo's lead is taken off the part
                    q_des = s.q.copy()
                    c, load = tr, True
                    p_cmd0, lat_fix = s.tip - d * c, np.zeros(3)
                    F_t = min(top, max(F_t, f_now, ramp.first_n))   # from the force the contact already put there
                lost, quiet = None, 0
            elif n_free is not None:
                quiet += 1
                if lost is None:
                    lost = tr
                if load and quiet >= n_det:   # nothing pushes back any more: on along the free plan
                    load, k_c = False, None
            if load or resists:
                # a touch holds its line this way under load only: over its long free travel the integral makes
                # an arm with spare joints sway
                lat_fix -= self.cfg["lateral_gain_per_s"] * dt * lat
                v_tip = max(0.0, float((self.model.jac(s.q)[:3] @ s.qd) @ d))
                if moved and not feel:   # the part runs: how fast, and what its end stop could take at that
                    v_part, strike = max(v_part, v_tip), max(strike, max(f_now, 0.0) + (z_end or 0.0) * v_tip)
            # what the servo asks of the tip along d (it rises at once with the lead, while the force on a part
            # can lag behind a joint's dry friction): the stall rule reads this
            asks = float(np.linalg.lstsq(self.model.jac(s.q)[:3].T, s.tau - self.model.gravity_torque(s.q),
                                         rcond=None)[0] @ d) if stall else 0.0
            hist.append((tr, Fd, asks, c))
            if load and len(hist) > n_set:   # the part's stiffness, read live over one settle time
                dtr, dF = tr - hist[-1 - n_set][0], Fd - hist[-1 - n_set][1]
                # a travel the arm's own give explains for that change of force reads nothing of the part (a
                # stiffness read off the readings' noise sent the command to its lead cap at the body's speed)
                k_c = max(0.0, dF / dtr) if dtr > tell + abs(dF) / k_s else None
                if k_c == 0.0 and len(hist) - n_set < (n_free or 0):   # a window that reaches back into the free
                    k_c = None   # travel reads no slide (the arrival read as a slide: 392 N for a 167 N cap)
            if Fd > peak:
                peak, tip_at_peak, tr_at_peak = Fd, s.tip.copy(), tr
            rigid = tell + max(Fd, 0.0) / k_s   # the tip travel the arm's own give explains at this force
            if first_resist is not None and not moved and tr - first_resist > rigid:
                moved, moved_at = True, max(Fd, 0.0)
            if peak > f_res and Fd < GIVE * peak and tr - tr_at_peak > rigid:
                gave = True
            if not feel and Fd >= top - eps:
                reached = True
            v_was = v_now
            stop = g.check(s)   # cap, a touch's contact, or a tick hook's stop request
            if stop:
                g.recent = list(recent)
                break
            if stop_on_give and gave:
                stop = "gave"   # the part let go: the stroke ends here, before the tip runs on into free air
                break
            if until_moved and moved:
                stop = "moved"
                break
            if first_resist is not None and lat_max > w_tool:   # the tip left its line by the tool's half width
                slipped = Fd < GIVE * peak
                stop = "slipped" if slipped else "off_axis"
                break
            if load and tr >= travel - tell:
                stop = "travel"
                break
            if not load and c >= sched[-1] - tell:
                # the command stands at the end of the free plan: the tip closes up under the armed stops until
                # what is left of its motion over one detection time is under what it can tell, at the latest
                # after the arm's own settle time
                at_end += 1
                if self._quiet(s, tell) or at_end > n_set:
                    stop = "travel" if (feel or whole) else "lead"
                    break
            if not feel:
                if stop_on_force and reached:
                    stop = "force"
                    break
                if reached and Fd >= GIVE * top:
                    held_n += 1
                    # no hold stated: the force steady, at the latest a settle time after it was reached, while the
                    # part STANDS (one that still yields at the force asked is followed on: a knob that turns)
                    if hold_left is None and len(hist) > n_set and tr - hist[-1 - n_set][0] <= rigid and (
                            abs(Fd - hist[-1 - n_set][1]) <= eps or held_n > n_set):
                        stop = "force"
                        break
                    elif hold_left is not None:
                        hold_left -= dt
                        if hold_left <= 0.0:
                            stop = "force"
                            break
                if c >= lead_cap and not reached:
                    stop = "lead"
                    break
                if load and len(hist) > n_set and min(c - tr, hist[-1 - n_set][3] - hist[-1 - n_set][0]) >= top / k_s \
                        and tr - hist[-1 - n_set][0] <= tell:
                    # over one settle time the command led the tip by the whole force asked over the servo's own
                    # stiffness and the part stood: the servo gives that force whatever the estimate reads
                    stop = "force"
                    break
                if (stall and load and len(hist) > n_set and hist[-1][0] - hist[-1 - n_set][0] <= tell
                        and c - tr > tell and max(hist[-1][1] - hist[-1 - n_set][1],
                                                  hist[-1][2] - hist[-1 - n_set][2]) <= eps):
                    # the tip does not advance and neither the force on the part nor what the servo asks rises
                    # any more over one settle time: these joints are done
                    stop = "stall"
                    break
        if g.asked:
            self._stop_asked()
        delta = s.tip - p0
        return {"stop": stop or "time", "guard": g, "s": s, "q_des": q_des, "done": done, "peak_n": peak, "reached": reached,
                "gave": gave, "moved": moved, "moved_at_n": moved_at, "slipped": slipped,
                "travel_m": float(delta @ d), "lateral_m": lat_max,
                "across_m": float(np.linalg.norm(delta - float(delta @ d) * d)), "first_resist_m": first_resist,
                "tip_at_peak": tip_at_peak, "cap_why": g.cap_why, "felt": g.felt, "contact": g.contact,
                "asked": g.asked, "free": done[:len(done) if n_free is None else n_free], "peak_total_n": peak_total,
                "t_resist": t_resist, "lost_m": lost, "k_c": k_c, "v_hit": v_hit, "impulse_ns": impulse,
                "items": items, "periods": len(done) - 1, "top_n": top, "v_part": v_part, "plan": plan,
                "z_end": z_end, "strike_n": strike if z_end and not feel else None, "sched": sched, "n_plan": n_plan,
                "v_met": v_met, "read_stop_n": float(s.F @ d), "rise_n": rise}

    # ------------------------------------------------------------------ touch
    @primitive
    def touch(self, direction, max_mm, frame="base", force_n=None, detail=False, clear=None, max_n=None):
        """A touch for depth (C9): an expected contact with whatever lies ahead on the line. A limit (max_n on the
        call, a sheet of a named thing, a give, the site, a rating) is kept: the stroke arrives at the speed
        solved for it, also under the speed at which this arm can tell a contact. force_n is the force at
        which the touch ends, not a bound on the way in; with none stated it is the hand's own weight
. The stroke arrives at the speed solved for that force, never under the speed at which this arm
        can tell a contact and never over the body's own; the receipt states the ending force, its source, the
        peak the arrival made and, for a stated number, the speed the default would run. It ends on the first
        reading over the contact threshold. clear: the operator's word (recorded)."""
        B = bounds.of(self)
        no = {"contact": False, "stopped": "refused"}
        bad = self._dark_refusal_now() or self._selfknow_gate("touch")
        if bad is not None:
            return {**bad, **no}
        s, _ = self._ease_if_pressing()
        d, bad = self._unit_in_base(direction, frame, s)
        if bad is not None:
            return {**bad, **no}
        max_mm = float(max_mm)
        if not max_mm > 0.0 or any(v is not None and not float(v) > 0.0 for v in (force_n, max_n)):
            return {**refusal("touch", "max_mm (and force_n or max_n, when stated) must be over 0",
                              "touch(direction=[0, 0, -1], max_mm=40)"), **no}
        at_rest = B.f_thr()
        if s.Fmag > at_rest:
            return {**refusal("pressing", f"the tip already feels {s.Fmag:.2f} N, over the {at_rest:.2f} N contact "
                              "threshold at rest", "retreat or move away from the surface, then touch"), **no}
        hand = float(B.f_hand())                     # the design rulings: with no number a touch ends at the hand's own weight
        own = {**self._stated(ends_n=hand, max_n=max_n, clear=clear), "ends_default": True}
        intent = own if force_n is None else self._stated(ends_n=force_n, max_n=max_n, clear=clear)
        v_own = None if force_n is None else self._arrival(B, s.q, d, own)["v"]   # what the stated number cost
        arr = self._arrival(B, s.q, d, intent)
        if force_n is None and arr["note"]:
            arr["note"] = f"no force_n stated, the touch ends at the hand's own weight ({hand:.3g} N): " + arr["note"]
        if arr["refused"] is not None:
            return {**arr["refused"], **no}
        told = self._told()                          # the stroke starts where the hand was told to stand
        path, pstop, pwhy, hold = self._line(told.q, d, max_mm / MM)
        # a stroke stated longer than the way to a support whose height has a stated error: the line is cut at
        # the far edge of what is known of that height (never refused for it); with no error stated the stroke
        # the caller states stands
        path, cut = control_expect.cut_at_support(self, told.q, path, stroke=True)
        if cut:
            max_mm = (float((self.model.fk_tip(path[-1])[0] - told.tip) @ d) if path else 0.0) * MM
        bad = self._path_check([told.q] + path, kind="touch")
        if bad is not None:
            return {**bad, **no}
        reach = max_mm / MM
        if pstop != "goal":   # the line leaves the arm's reach: feel as far as the plan goes
            reach = float((self.model.fk_tip(path[-1])[0] - told.tip) @ d) if path else 0.0
        thing = self._ahead(B, told.tip, d, reach) if reach > 0.0 else None
        band = self._band_of(B, told, d, reach, thing)   # where a contact can begin (C15, the design rulings)
        kw = {"speed": arr["v"], "feel": True, "hold": hold, "thing": thing, "intent": intent, "band": band}
        waited, bad = self._wait_counted(told, d, reach, None, line=path, **kw)   # the budget, then the wait
        if bad is not None:
            return {**bad, **no}
        if self._tail is not None:                   # the hand met something as it came to rest: that is the touch
            return self._own_touch(self._tail_touch(waited, force_n, arr), arr, force_n)
        s = self._sense()
        false_stops, r, e, went, info, plan = 0, None, s, 0.0, None, None
        while True:   # a false stop (A1.1) refits the hidden level and the leg resumes from where it stopped
            r = self._push(e, d, reach - went, None, line=path if r is None else None,   # the first command
                           q_start=self._q_des.copy() if r is None else None, **kw)      # starts at the told pose
            if r["stop"] == "refused":
                return {"refused": r["refused"], **no}
            felt = r["felt"] if false_stops == 0 else felt.merge(r["felt"])
            plan = r["plan"] if false_stops == 0 else plan
            e = self._ease_impl()
            went = float((e.tip - s.tip) @ d)
            c = r["contact"]
            if r["stop"] not in ("contact", "cap") or c is None:
                break
            sigma = laws.contact_error(r["v_hit"], B.t_det(), B.quantum(r["s"].q, d))
            met = thing if self._same_spot(B, thing, c.get("point_mm"), sigma) else None
            # the bounds learn the stop (its measured arrival speed, its peak where the body reads one), or call
            # it false; it is the contact this stroke meant
            info = self._stop_note(r["guard"], "touch", met, None, e, expect=arr["expect"], travel=d)
            if r["stop"] != "contact" or not info["false"] or reach - went <= B.rigid(e.q, d):
                break
            false_stops += 1
            kw["band"] = {**band, "from_m": max(0.0, band["from_m"] - went)}
        stop = r["stop"]
        if e.Fmag > at_rest:   # a part still pushes back (joint friction held the tip): off it, by what it showed
            self._release(d, felt.resisted + B.travel("touch", thing, e.q, d, intent).backoff_m, hold, extra_m=0.0)
            e = self._sense()
        self._push_history(self._retrace(r["free"], e, d))
        peak_total = max(r["peak_total_n"], felt.peak, (info or {}).get("peak_n", 0.0))
        periods = int(round((e.t - s.t) / B.dt())) + waited
        v_ran = float((info or {}).get("speed_m_s") or r["v_hit"] or 0.0)
        keys = self._move_keys(plan, periods, (), {"_ran": info["ran"]} if info else None)
        keys.pop("bounds", None)
        if "predicted_peak_n" not in keys and r["z_end"] is not None:      # r6 shim: round four's own figure
            keys["predicted_peak_n"] = round(B.f_thr(v_ran) + r["z_end"] * v_ran, 2)
        out = {"travelled_mm": round(went * MM, 2), "asked_force_n": None if force_n is None else float(force_n),
               "ends_n": round(hand if force_n is None else float(force_n), 2),
               "ends_source": "the hand's own weight (no force_n stated)" if force_n is None else "your force_n",
               "speed_mm_s": control_tail.speed_said(self._ran_speed(r["sched"], kw["band"]["from_m"], B.dt()), arr["v"]),
               "limit_n": arr["expect"]["bound_n"], "limit_source": arr["expect"]["bound_source"],
               "limit_kept": self._kept(arr, peak_total), "peak_force_total_n": round(peak_total, 2),
               "tip_mm_at_stop": _mm(r["s"].tip), "along_push_mm": round(went * MM, 2),
               "across_mm": round(r["across_m"] * MM, 2), "eased": bool(e.Fmag <= at_rest), "arrive_periods": waited,
               **keys, **felt.report(None, detail=detail), **self._finger_keys(s.R, e.R, hold)}
        out.update({"notes": notes} if (notes := [n for n in (arr["note"], cut) if n]) else {})
        self._own_touch(out, arr, force_n)
        self._band_keys(out, B, band, r, arr["v"], told.q, d, detail)
        if v_own is not None:
            out["default"] = {"ends_n": round(hand, 2), "speed_mm_s": round(v_own * MM, 2),
                              "what": "the way in with no force_n stated: it ends at the hand's own weight"}
        out.update({"false_stops": false_stops} if false_stops else {})
        if not out["eased"]:
            out["eased_why"] = f"the tool still reads {e.Fmag:.1f} N: something holds against it"
        B.report(out, r["items"], "full" if detail else "brief")
        if stop in ("contact", "cap") and c is not None:
            self._record_contact(c, "touch", info=dict(info, arrived=False, meant=False) if info else None)
            if c.get("id") is not None:   # the new entry carries what the stop showed: k_c, what it bore, its error
                B.note_stop(B.thing(c["id"]), r["s"].q, d, v_ran, peak_total, r["impulse_ns"],
                            bool((info or {}).get("resolved")), r["felt"].resisted, rest_n=None, k_c=r["k_c"])
                self.world.note_sense("touch", c["id"], summary={"point_mm": c["point_mm"], "force_n": c["force_n"]})
            if info is not None:
                out["peak_resolved"] = bool(info["resolved"])
            if felt.moved:   # the part it moved voids the passage it can reach
                out.update({"void": v} if (v := self._stale_near(c.get("point_mm"), "a touch moved the part there",
                                                                 felt.resisted, d)) else {})
            out = {"said": f"Touched something after {out['along_push_mm']:g} mm: {peak_total:.1f} N at the stop.",
                   "contact": True, "point_mm": c["point_mm"], "normal_est": c["normal_est"], "force_n": c["force_n"],
                   "id": c.get("id"), "stopped": stop, "surface": c["surface"], "at_tip": c["at_tip"],
                   "source": c["source"], "fit_residual": c["fit_residual"], **out}
            if c["source"] != "tool":
                out.update(likely_links=c["likely_links"], tip_mm_at_stop=c["tip_mm_at_stop"], normal_est=None,
                           why="the stop was not made by the tool: a link of the arm met something, or the arm "
                               "met itself; point_mm is not measured and nothing was stored in robot://world")
            if stop == "cap":
                out["why"] = r["cap_why"]
            return out
        return self._touch_none(out, stop, pstop, pwhy, max_mm, e, r)

    # ------------------------------------------------------------------ press
    @primitive
    def press(self, direction, travel_mm=None, force_n=None, mode="nudge", max_rounds=None, frame="base",
              base_check="auto", detail=False, max_n=None, hold_s=None, clear=None):
        """Push along direction (C9). force_n: the force to reach, as the operator states it. max_n alone: the
        ramp runs until the part moves, at most to max_n. Neither: the ramp runs until the part moves, at most to
        what the arm can push there, and the result offers max_n. The approach to the part is an expected
        contact bounded by what is stated; the stroke after it is led by force. travel_mm: as stated; none: until
        the part gives or stops, inside the arm's reach. hold_s (firm, rounds): as stated; none: until the force
        is steady."""
        B = bounds.of(self)
        no = {"stopped": "refused"}
        bad = self._dark_refusal_now() or self._selfknow_gate("press")
        if bad is not None:
            return {**bad, **no}
        s, _ = self._ease_if_pressing()
        d, bad = self._unit_in_base(direction, frame, s)
        if bad is not None:
            return {**bad, **no}
        if mode not in ("nudge", "firm", "rounds"):
            return {**refusal("mode", f"unknown press mode {mode!r}", "nudge, firm or rounds"), **no}
        if base_check not in MODES:
            return {**refusal("base_check", f"unknown base_check {base_check!r}", "auto, defer or skip"), **no}
        intent = self._stated(force_n=force_n, max_n=max_n, clear=clear, hold_s=hold_s)
        notes = []
        if travel_mm is None:   # no travel stated: as far as the arm's own reach goes along the line
            reach = self._line(s.q, d, self.model.length_scale)[0]
            room = float((self.model.fk_tip(reach[-1])[0] - s.tip) @ d) if reach else 0.0
        else:
            room = float(travel_mm) / MM
        thing = self._ahead(B, s.tip, d, room)
        ramp = B.force("press", thing, s.q, d, intent)      # never refused: with no force stated or known
        way_in = control_move_press.approach(intent, force_n, ramp)   # the approach ends at the ramp's first force
        arr = self._arrival(B, s.q, d, way_in, thing)                  # the approach: an expected contact (C15 2 a)
        if arr["refused"] is not None:
            return {**arr["refused"], **no}
        strk = self._arrival(B, s.q, d, intent, thing, stroke=True)    # the stroke: max_n, a sheet, the site
        if all(i.get("source") in ("model", "rating") for i in ramp.items):   # the ceiling is the robot's own
            notes.append(f"no force stated: the ramp runs until the part moves, at most to what the robot can "
                         f"push here ({ramp.ceiling_n:.1f} N); state max_n to bound it")
        top = ramp.ceiling_n if force_n is None else float(force_n)
        if top > ramp.ceiling_n:
            why = "; ".join(str(i.get("evidence", "")) for i in ramp.items[:1])
            if "base give" not in why:
                return {**refusal("force", f"{top:g} N is over the {ramp.ceiling_n:.1f} N ceiling here ({why})",
                                  f"press with force_n {ramp.ceiling_n:.1f} or less, or come at it so the push "
                                  "runs along a stronger direction (reach_check reports the force at a point)"),
                        **no}
            notes.append(f"held to {ramp.ceiling_n:.1f} N of the {top:g} N asked ({why}): a base that gave bounds "
                         "the ramp and refuses nothing; measure_base_give lifts it")   # it cannot refuse (C9)
            top = ramp.ceiling_n
        told = self._told()                            # the stroke starts where the hand was told to stand
        lead = top / B.k_servo(told.q, d)
        travel = room if travel_mm is not None else max(0.0, room - lead - lead)
        if travel_mm is None:
            notes.append(f"no travel stated: until the part gives or stops, at most {travel * MM:.0f} mm")
        travel, bad, hold, short = self._stroke_room(told, d, travel, lead, travel_mm is not None)
        if bad is not None:
            return {**bad, **no}
        notes += [short] if short else []
        kw = {"ramp": ramp, "thing": thing, "intent": way_in, "until_moved": force_n is None, "speed": arr["v"],
              "stroke_v": strk["v"], "band": self._band_of(B, told, d, travel, thing)}
        waited, bad = self._wait_counted(told, d, travel, force_n, hold_s=hold_s if mode != "nudge" else None,
                                         hold=hold, **kw)   # the budget, then the wait for the hand to stand
        if bad is not None:
            return {**bad, **no}
        s = self._sense()
        q_pre, R_pre, s_pre = s.q.copy(), s.R.copy(), s
        slack, rounds, free, felt, asked, legs = 0.0, 0, None, None, None, []

        def ran(r):   # one push of this press: its path before anything resisted, what it met, a stop request
            nonlocal free, felt, asked
            if r["stop"] != "refused":
                free = r["free"] if free is None else free
                felt = r["felt"] if felt is None else felt.merge(r["felt"])
                asked = asked or r["asked"]
                legs.append(r)
            return r

        gave = slipped = False
        peak, cap_why, stop = 0.0, "", "time"
        if mode == "nudge":
            r = ran(self._push(s, d, travel, force_n, stop_on_force=True, hold=hold, stop_on_give=True, **kw))
        elif mode == "firm":
            r = ran(self._push(s, d, travel, force_n, hold_s=hold_s, hold=hold, **kw))
        else:   # rounds: the chain's base half takes up the load, its tool half finishes it
            r, rounds, slack, peak, s, p_start = control_move_press.rounds(self, s, d, travel, top, ramp, max_rounds,
                                                                           kw, ran, hold_s)
        if r is None or r["stop"] == "refused":
            if not legs:
                return {**({"refused": r["refused"]} if r else refusal("rounds", "max_rounds must be 1 or more",
                                                                       "omit it")), **no}
            r = legs[-1]
        gave, slipped, stop, cap_why = r["gave"], r["slipped"], r["stop"], r["cap_why"]
        peak = max(lg["peak_n"] for lg in legs)
        rounds = max(rounds, 1)
        if mode == "rounds":
            delta = s.tip - p_start
            along = float(delta @ d)
            r = {"s": s, "travel_m": along, "across_m": float(np.linalg.norm(delta - along * d)),
                 "lateral_m": float(np.linalg.norm(delta - along * d)), "done": [q for lg in legs for q in lg["done"]],
                 "peak_n": peak, "gave": gave, "felt": felt, "first_resist_m": legs[0]["first_resist_m"]}
        total = r["travel_m"]
        k_c = next((lg["k_c"] for lg in reversed(legs) if lg.get("k_c") is not None), None)
        moved_at = next((lg["moved_at_n"] for lg in legs if lg.get("moved_at_n") is not None), None)
        t_res = next((lg["t_resist"] for lg in legs if lg.get("t_resist") is not None), None)
        after, peak_after = self._after_stroke(r, d, s_pre, hold, give_lift=(mode == "nudge" and stop == "gave"),
                                               thing=thing, k_c=k_c, intent=intent)
        e = self._sense()
        if after.get("lifted") is None:
            self._push_history(self._retrace(free, e, d, peak))
        ref = f"P{self._calls}"
        peak_all = max(lg["peak_total_n"] for lg in legs)
        summary = {"peak_force_n": round(peak, 2), "gave": gave, "slipped": slipped, "travel_mm": round(total * MM, 2)}
        self.world.note_sense("press", ref, summary=summary)
        # due or not is judged against what the stored points were good to BEFORE this press told more of them
        hard = self._base_due(stop, peak_all, mode, felt.resisted, d)
        if thing is not None and thing.id is not None and t_res is not None:
            # what this part showed: the force it bore, or the force at which it moved under work meant to move it
            B.note_stop(thing, e.q, d, legs[0]["v_hit"], peak_all, sum(lg["impulse_ns"] for lg in legs), False,
                        0.0 if moved_at is not None else felt.resisted, rest_n=None, k_c=k_c)
            if moved_at is not None and thing.id != "support":
                self.world.update(thing.id, data={"f_moved_n": round(moved_at, 3), "f_worked_n": round(peak, 3)})
        fr = r.get("first_resist_m")
        pushed = _mm(s_pre.tip + d * max(0.0, fr if fr is not None else 0.0))
        base = self._base_check(hard and not asked, q_pre, back=-d * (max(total, 0.0) + self._lead_for(top, e.q, d)),
                                mode=base_check, pushed_mm=pushed, intent=intent,
                                part=thing if thing is not None and thing.id is not None else None)
        seen = felt.report(None, detail=detail)
        peak_total = max(peak_all, peak_after, base.get("peak_force_n") or 0.0)
        out = {"mode": mode, "travel_mm": round(total * MM, 2), "peak_force_n": round(peak, 2),
               "peak_force_total_n": round(peak_total, 2), "ceiling_n": round(ramp.ceiling_n, 2),
               "reached_force": bool(peak >= top - ramp.rate_n_s(0.0) * B.t_det()), "gave": bool(gave),
               "slipped": bool(slipped), "rounds": rounds, "slack_mm": round(slack * MM, 2), "base_moved": base,
               "stopped": stop, "ref": ref, "tip_mm": _mm(self._sense().tip), "resisted_mm": seen["resisted_mm"],
               "first_resist_mm": seen["first_resist_mm"], "moved_part": bool(seen["moved_part"] or gave), **after,
               "held_s": round(max(0.0, (e.t if t_res is not None else 0.0) - (t_res or 0.0)), 2),
               "periods": int(round((self._sense().t - s_pre.t) / B.dt())) + waited, "arrive_periods": waited,
               **self._stroke_keys(B, legs, top, arr, strk, peak_total),
               "ends_n": round(top if force_n is not None else ramp.first_n, 2),
               "ends_source": "your force_n" if force_n is not None else "the ramp's first force (no force_n stated)"}
        self._band_keys(out, B, kw["band"], legs[0], arr["v"], told.q, d, detail)   # C15 (5)
        hit = control_move_press.ran(B, legs, peak_total)   # the one figure after the stop (C2, the design rulings ii)
        if hit is not None:
            out["strike"] = {**(out.get("strike") or {}), "ran": hit}
        out = self._press_words(out, seen, legs, r, stop, gave, moved_at, t_res, peak, felt)
        if hold == "pitch":
            out.update(self._finger_keys(R_pre, e.R, hold))
        if cap_why:
            out["why"] = cap_why
        if notes:
            out["notes"] = notes
        return B.report(out, legs[0]["items"], "full" if detail else "brief")
