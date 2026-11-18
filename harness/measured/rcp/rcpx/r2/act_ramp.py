# SPDX-License-Identifier: AGPL-3.0-only
"""Continuous force legs of push and lift.

Each target follows the measured noise, torque error and detection time. A give
leg stops once it shows resolved travel beyond the earlier legs, even when force
still follows its target. A stop leg follows the part to its own end. The servo
lead is target / k_servo, the same compliance the core contact law uses. Force
and travel evidence uses the bounds object's keys on the target's world entry.
No scene or scoring truth enters this module.
"""
import numpy as np
from rcpx import r3_limits as L
from rcpx import bounds_laws
from rcpx.control_base import Guard
from rcpx.r2 import act_ladder
from rcpx.r2.act_engine import MM, Stop, _r, _unit, field, reach_mm


def drive_period(self, state, u, target, hold):
    """Drive one period with target / k_servo lead and the core lateral correction."""
    ex, B, s = self.ex, self.B, state["s"]
    k = B.k_servo(s.q, u)
    along = float((s.tip - state["p0"]) @ u)
    lat = s.tip - state["p0"] - along * u
    state["lat_fix"] -= ex.cfg["lateral_gain"] * lat
    p_cmd = state["p0"] + u * (along + target / k) + state["lat_fix"]
    # The servo lead is a force request, not permission to run into an end.
    # Advance the command at the load-speed law with the live series stiffness.
    k_c = state.get("k_c")
    k_w = bounds_laws.series(k, k_c) if k_c is not None and k_c > 0 else k
    speed = min(B.v_free(s.q, u), bounds_laws.load_speed(target, state["read_n"], k_w, B.t_det()))
    # toward an end not yet found the tip also goes no faster than the speed whose stop on that end peaks at the
    # ceiling, (C - F_now) / Z, wherever the bounds object resolves the stop slope Z
    z = B.slope(s.q, u, state.get("thing"))[0] if hasattr(B, "slope") else None
    if z:
        speed = min(speed, max(0.0, state["ceiling_n"] - state["read_n"]) / z)
    previous = state.get("p_cmd", state["p0"])
    advance = float((p_cmd - previous) @ u)
    p_cmd -= u * max(0.0, advance - speed * B.dt())
    state["q_des"], _, _ = ex._track(state["q_des"], p_cmd, state["R0"], hold)
    state["p_cmd"] = p_cmd
    state["s"] = ex._period(state["q_des"])
    self.moved = True
    return state["s"]


def run(self, u, thing, until, hold_s=0.0, intent=None, sheet_forces=None,
         sheet_travel_mm=None, moved_n=None, touched_n=None, shown_mm=0.0):
    """Run one continuous leg, stopping at motion beyond shown_mm, a give, the part end or the derived ceiling."""
    ex, B, u = self.ex, self.B, _unit(u)
    s = ex._sense()
    spec = L.force_spec(B, "press", thing, s.q, u, intent=intent, sheet_forces=sheet_forces, moved_n=moved_n, touched_n=touched_n, local=self.local)
    if spec.get("refused"):
        raise Stop("refused", stage="ramp", check="force_unknown", refused=spec["refused"])
    ceiling, target, travel = spec["ceiling_n"], min(spec["first_n"], spec["ceiling_n"]), L.stroke_mm(B, s.q, u, sheet_travel_mm=sheet_travel_mm, shown_mm=None)
    if travel is None:
        travel = reach_mm(ex, s.tip * MM, u)
        if u[2] < 0:
            travel = min(travel, max(0, (s.tip[2] * MM - float(ex.cfg.get("floor_z_m") or 0) * MM) / -u[2] - L.floor_keep_mm(B)))
    self.gate("ramp", "press", {"direction": _r(u), "travel_mm": travel,
                               "force_n": ceiling, "frame": "base"})
    bad, hold = ex._stroke_plan(s, u, travel / MM + ceiling / B.k_servo(s.q, u))
    if bad is not None:
        refused = bad.get("refused", bad)
        raise Stop("refused", stage="ramp", check=refused.get("check"), refused=refused)
    dt, t0, e = B.dt(), ex.sim_time(), B.e_tau()
    sigma = max(B.sigma_f(0, 0), self.local[1] if self.local else 0)
    planned = act_ladder.ramp_time(target, ceiling, sigma, e, B.t_det()) + hold_s + L.settle_s(B)
    bound = L.act_max_s(B, planned_s=planned)
    g = Guard(ex, None)
    state = {"s": s, "q_des": s.q.copy(), "p0": s.tip.copy(), "R0": s.R.copy(),
             "lat_fix": np.zeros_like(s.tip), "read_n": float(s.F @ u),
             "k_c": field(thing, "k_c"), "thing": thing, "ceiling_n": ceiling}
    peak, peak_total, peak_tr, f_prev, peak_along = 0, 0, 0, float(s.F @ u), 0
    tr, last_tr, follow_tr, no_advance, at_force = 0, 0, 0, 0, 0
    f_moved, f_bear, gave, trace, stopped = None, 0, False, [], None
    ref_tr, ref_f, k_shown = 0, f_prev, None
    periods, read, force_tr, interrupt = 0, f_prev, None, None
    try:
        while stopped is None:
            self.check_time("ramp")
            previous_target = target
            s = drive_period(self, state, u, target, hold)
            periods += 1
            now = float(s.F @ u)
            read, f_prev = min(now, f_prev), now
            tr = float((s.tip - state["p0"]) @ u) * MM
            # what counts as motion of the part: the arm's resolution and give plus the finding touch's own error,
            # never zero (L.moved_mm); small = QUANT x that is motion for certain
            rigid = L.moved_mm(B, s.q, u, v_mm_s=getattr(self, "touch_v_mm_s", None), force_n=0.0, thing=thing)
            small = L.ratio("QUANT") * rigid
            tol = act_ladder.follows_tol(target, sigma, e)
            if read > peak:
                peak, peak_tr = read, tr
            peak_total = max(peak_total, float(s.Fmag))
            peak_along = max(peak_along, read)
            moving = tr - follow_tr > rigid
            if read >= target - tol:
                follow_tr = tr
            if f_moved is None and tr > rigid:
                f_moved = read
                k_shown = read / tr if tr > 0 else None
            if f_moved is None:
                f_bear = max(f_bear, read)
            state["read_n"] = read
            if k_shown is not None:
                state["k_c"] = k_shown * MM
            no_advance = no_advance + dt if tr <= last_tr else 0
            if target >= ceiling and read >= ceiling - tol:
                if force_tr is None or tr - force_tr > rigid:
                    at_force, force_tr = 0, tr
                at_force += dt
            else:
                at_force, force_tr = 0, None
            last_tr = tr
            stopped = g.check(s)
            if stopped is None and float(np.linalg.norm(s.tip - state["p0"] - u * tr / MM)) * MM > L.step_look_back_mm(B):
                stopped = "off_axis"
            is_give = act_ladder.gave(peak, read, tr - peak_tr, rigid, L.f_touch(B, readings=2, local=self.local))
            if is_give:
                gave = True
                if until == "give" and stopped is None:
                    stopped = "gave"
                elif until == "stop":
                    peak, peak_tr = max(0, read), tr
            next_target = act_ladder.step(target, read, moving, spec["rate_n_s"](target), dt, ceiling, tol)
            if stopped is None and act_ladder.hard_stop(f_moved is not None, ref_f, read, tr - ref_tr, rigid, k_shown, tol):
                stopped = "stall"
            if tr - ref_tr >= rigid:
                ref_tr, ref_f = tr, read
            # until give the leg ends on motion: small on the first leg, then DOUBLE the travel the part already
            # showed (a doubling search for the travel at which it changes: few legs, each one further)
            if stopped is None and until == "give" and tr >= max(small, L.ratio("DOUBLE") * shown_mm):
                stopped = "moved"
            if stopped is None and at_force >= L.settle_s(B):
                stopped = "force"
            if stopped is None and tr >= travel:
                stopped = "travel"
            if stopped is None and no_advance >= bound:
                stopped = "time"
            trace.append([periods * dt, previous_target, read, tr])
            target = next_target if stopped is None else target
        if hold_s > 0 and stopped in ("gave", "moved", "stall", "force"):
            left = hold_s
            while left > 0:
                self.check_time("ramp_hold")
                s = ex._period(state["q_des"])
                periods += 1
                read = min(float(s.F @ u), f_prev)
                f_prev = float(s.F @ u)
                tr = float((s.tip - state["p0"]) @ u) * MM
                peak, peak_total = max(peak, read), max(peak_total, float(s.Fmag))
                peak_along = max(peak_along, read)
                trace.append([periods * dt, target, read, tr])
                guard_stop = g.check(s)
                if guard_stop:
                    stopped = guard_stop
                    break
                left -= dt
    except Stop as stop:
        stopped, interrupt = stop.stopped, stop
    finally:
        stop_tip = s.tip.copy()
        ease_start = periods
        period = ex._period
        def easing_period(q_des):
            """Read the force and travel of every easing period so the reported peak includes release."""
            nonlocal periods, peak_total, peak_along, f_prev
            eased = period(q_des)
            periods += 1
            force = float(eased.F @ u)
            held = min(force, f_prev)
            f_prev = force
            peak_total = max(peak_total, float(eased.Fmag))
            peak_along = max(peak_along, held)
            trace.append([ex.sim_time() - t0, 0, held,
                          float((eased.tip - state["p0"]) @ u) * MM])
            return eased
        owned = "_period" in vars(ex)        # the swap is undone exactly: no attribute is left on the executor
        ex._period = easing_period
        try:
            ex._ease_impl()
        finally:
            if owned:
                ex._period = period
            else:
                del ex._period
    if g.asked:
        ex._stop_asked()
    back = max(0, float((stop_tip - ex._sense().tip) @ u) * MM)
    spec_out = {k: v for k, v in spec.items() if k != "rate_n_s"}
    out = {**spec_out, "stopped": stopped, "gave": gave, "base_moved": False, "along_mm": tr,
           "total_mm": tr, "sprang_back": back >= L.moved_mm(B, s.q, u, v_mm_s=getattr(self, "touch_v_mm_s", None),
                                                               force_n=0.0, thing=thing), "sprang_back_mm": back,
           "force_n": read, "target_n": target, "peak_n": peak_along, "peak_total_n": peak_total,
           "f_moved_n": f_moved, "f_bear_n": f_bear, "tip_mm_at_stop": _r(stop_tip * MM),
           "start_mm": _r(state["p0"] * MM), "periods": periods, "seconds": ex.sim_time() - t0}
    if self.detail == "full":
        out["trace"] = trace
    self.ramps.append(out)
    self.items = spec["items"]
    self.peak_total = max(self.peak_total, peak_total)
    rec = {"by": "runtime", "tool": self.tool, "stage": "ramp", "primitive": "press",
           "t0": t0, "t1": ex.sim_time(), "periods": periods, "phase": "ramp", "did": "continuous press",
           "moving": ease_start, "waiting": periods - ease_start, "jaw": 0,
           "seconds": ex.sim_time() - t0, "free_share": None,
           "counts_source": "ramp and ease control periods counted directly",
           "stopped": stopped, "bounds": self.items[0]}
    self.legs.append(rec)
    ex.note_event("r3_leg", **rec)
    target_id = getattr(self, "target_id", None) or field(thing, "id")
    if target_id is not None:
        old = (ex.world.get(target_id) or {}).get("data") or {}
        data = {"f_bear_n": max(old.get("f_bear_n") or 0, f_bear), "touched": True}
        if f_moved is not None:
            data["f_moved_n"] = max(old.get("f_moved_n") or 0, f_moved)
        if stopped in ("gave", "stall"):
            data["f_worked_n"] = read
        if stopped == "stall" and f_moved is not None:
            data["end_mm"] = tr
        ex.world.update(target_id, data=data)
    if interrupt is not None:
        interrupt.fields["ramp"] = out
        raise interrupt
    return out
