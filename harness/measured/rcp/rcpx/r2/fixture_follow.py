# SPDX-License-Identifier: AGPL-3.0-only
"""The follow of package fixture (SPEC-R4 B5 phases 3 to 5, A4.5): one stream along the part's own path.

A held or pushed part is first loaded by ONE continuous force ramp along the candidate direction (no steps, no force
threshold): the command leads the tip by the force asked over the servo's own stiffness, from the first force
B.force gives up to its ceiling ("on your word to move it"). The part has moved when the tip's path passed rigid.
From there the follow runs by A4.5:
- the command leads the tip by no more than C_f / k_servo plus the servo's own lag at the speed commanded, C_f =
  F_moved x (1 + REL): against a stop the force that persists is C_f, never the servo's wind-up;
- across the path the command is the measured tip (compliance: no force builds across), and the direction is the
  fitted path's own tangent (a line, or an arc with the hand turned about its axis), refitted as the path grows;
- toward an end not yet found the speed is (C_end - F_now) / Z, Z with the part's reflected mass (read from the
  ramp); with a stated amount, or over a stretch the part was already moved through, it is the arm's free speed
  braked into the end by its own deceleration;
- it ends, with the reason: end (the path stalls with the command at its lead cap), amount, link_contact (the
  force is placed off the tool, or a body's hull would enter a band one stopping distance ahead), slipped (the jaws
  lost the part), refused (the part did not move at the ceiling), contact (a side force over the follow's guard),
  steps_spent, time (no progress over a settle time; a stroke that makes progress is bounded by the step budget).
The part has moved when the tip's path passed rigid under the force it then read (the ramp's own test) and went
on by more than the tip resolves: the peak a stop makes at the end of a stroke says nothing of the way before it.
A turn about the tool axis is the same law in N m about that axis; along that axis the hand keeps the height
its hold was taken at: the drift of a long turn is taken out inside the turn, not pressed into the part, and ends
the stroke only where the part would leave the pads. Motion between places is one planned stream (B.plan_legs)
through the core's ex._stream. Periods are the executor's own; strikes: rcpx/r2/fixture_strike.py (C12). SI inside.
"""

import math

import numpy as np

from rcpx import bounds_laws as laws
from rcpx import bounds_report as report
from rcpx.bounds_ratios import GIVE, QUANT, REL
from rcpx.r2 import fixture_cost as COST
from rcpx.r2 import fixture_hold as H
from rcpx.r2 import fixture_probe as P
from rcpx.r2 import fixture_strike as S
from rcpx.r2.fixture_probe import TINY, unit

ENDS = ("end", "amount", "link_contact", "slipped", "refused")


# ---------------------------------------------------------------------------------------------- periods and phases
def spent(ex):
    return ex.backend.read().get("steps_left") == 0


def tick(ex, q=None, kind="moving"):
    """One control period with the arm commanded to q (None: where it is commanded now), counted as kind."""
    if q is None:
        q = ex._q_des if getattr(ex, "_q_des", None) is not None else ex._sense().q
    was = getattr(ex, "_period_kind", None)
    if was is not None:
        ex._period_kind = kind
    try:
        return ex._period(np.asarray(q, dtype=float))
    finally:
        if was is not None:
            ex._period_kind = was


tick.spent = spent


def begin(ex, name):
    """A phase begins: the bounds re-see (A3), a phase record of A5 opens where the phase before it ended."""
    B = H.bounds_of(ex)
    changed = B.begin("phase")
    rec = report.phase_record(name, "", 0, 0, 0, 0.0, None, None)
    rec["_t0"] = float(ex._sense().t)
    S.open_phase(ex, rec)
    ex._fx["phases"].append(rec)
    return changed


def end(ex, did, item=None, free_share=None):
    """The phase ends: what it did, its seconds, its control periods by the executor's own counters, its binding
    bound and the worst strike of the motions it ran (C12)."""
    rec = ex._fx["phases"][-1]
    rec["did"] = did
    rec["seconds"] = round(float(ex._sense().t) - rec.pop("_t0"), 3)
    if item is not None:
        rec["bounds"] = item
    if free_share is not None:
        rec["free_share"] = free_share
    return S.close_phase(ex, rec)


def speed_item(B, v, evidence, source="model", lift="state an amount: until {mm: n}", over=None):
    return report.item("speed", v, source, evidence, lift, over)


# ---------------------------------------------------------------------------------------------- one stream
def stream(ex, legs, intent, need_m=None):
    """Run legs as ONE planned path through the core (ex._stream) with the operator's word passed on; the strike
    is kept for the phase that ran it."""
    legs, token = [dict(leg, intent=dict(intent or {})) for leg in legs], COST.start(ex)
    r = ex._stream(legs, need_m=need_m)
    COST.stream(ex, token, legs, r)          # what the operator's own word cost it: the legs planned again
    bound = r.get("bounds")
    bound = bound.get("binding") if isinstance(bound, dict) and "binding" in bound else bound
    return {"stopped": r.get("stopped", "refused"), "moving": r.get("moving", 0), "bounds": bound,
            "free_share": r.get("free_share"), "unseen": r.get("unseen"), "refused": r.get("refused"),
            "contact": r.get("contact"), "leg": r.get("leg"), "strike": S.note(ex, r.get("strike")),
            "why": r.get("why"), "left": r.get("left"), "pressed": r.get("pressed")}


# ---------------------------------------------------------------------------------------------- the follow
def side_guard(B, q, d, v, f_along):
    """The side force (N) over which a follow stops: the force the estimate can hide at this speed, what the
    servo's own cross-track error at this speed presses across the path, and what the contact the follow itself
    makes can turn sideways (no more than the force along the path)."""
    side = unit(np.cross(d, np.cross(np.eye(len(d))[int(np.argmin(np.abs(d)))], d)))
    return B.f_thr(v, 0) + B.k_servo(q, side) * (B.e_cross(v) + B.rigid(q, side)) + f_along


def follow(ex, part, cand, d0, until, intent, fs, known_m=None, hold_r=None, play_m=0.0):
    """Load the part along d0 by the ramp, then follow its own path (see the file's head). until: None (its end)
    or metres of path. known_m: path the part was already moved through from here (run at free speed, braked into
    its end). hold_r: the hand's rotation to keep (a push), None: it turns with a fitted arc. play_m: the free
    travel the hold itself leaves before the part resists (the jaws' room, the reach past a touch): crossed at the
    first-touch speed; nothing met over it ends the stroke. A pushed part that is no longer felt while it is
    followed (it ran ahead of the hand) is followed on over play_m before the stroke ends slipped. Returns the
    record."""
    B = H.bounds_of(ex)
    dt = B.dt()
    s = ex._sense()
    p0, R0, d = s.tip.copy(), s.R.copy(), unit(d0)
    C = fs.ceiling_n
    k = B.k_servo(s.q, d)
    lead = max(fs.first_n, B.f_thr(0, 0)) / k
    v_seek, seek_item = B.v_in(part["thing"], s.q, d, intent)
    pts, forces, expected, at_cap = [p0.copy()], [], [], []
    state, x_load, moved_at = "seek", None, None
    f_moved, f_peak, v, mass, found, travel, ahead = None, 0.0, 0.0, None, P.fit([], 0), 0.0, 0.0
    stop, why, link, side_n, lost, side_peak, v_goal, v_cmd, lost_at = None, None, 0, 0, 0, 0.0, 0.0, 0.0, 0.0
    n_win = P.window_periods(B)
    ramp_left = P.ramp_periods(B, lead * k, C)
    item = fs.items[0]
    held = cand["closes"]
    gap0, _, sq_peak, _, _ = H.jaw_read(ex) if held else (None, None, None, None, None)
    gap_min = (float(ex._fx_jaws.width_m(ex._fx_jaws.closed)) + QUANT * H.jaw_step_m(ex)) if held else None
    seat = B.rigid(s.q, d, C)
    spacing = max(B.spacing(seat), seat)
    free_n, moving_n, alongs, v_meas, jump, lag_s, still_n, slow_n, v_tell = 0, 0, [], 0.0, 0, 0.0, 0, 0, None
    v_fast, v_met, v_last = 0.0, 0.0, 0.0     # the fastest speed the stroke ran; the speed at its largest reading
    rest = np.zeros_like(np.asarray(s.F, dtype=float))   # a stroke begins IN contact (a touch, a hold): what the
    #                         estimate reads there is load, so nothing is taken off it (a stream does: see stream)
    while stop is None:
        if spent(ex):
            stop = "steps_spent"
            break
        k = B.k_servo(s.q, d)
        f_now = max(0.0, float((np.asarray(s.F) - rest) @ d))
        if f_now > f_peak:
            v_met = max(v_met, v_meas, v_last)
        f_peak = max(f_peak, f_now)
        thr = B.f_thr(v, 0) + REL * float(np.linalg.norm(rest))
        v_free = B.v_free(s.q, d)
        gone = float((s.tip - p0) @ d)
        if state == "seek" and f_now > thr:
            state, x_load = "ramp", s.tip.copy()
            lead = max(lead, f_now / k)
        if state == "seek":
            v_cmd = v_seek
            cap = lead + B.lag(v_cmd)
            ahead = min(cap, max(0.0, ahead) + v_cmd * dt)
            item = seek_item
        elif state == "ramp":
            lead = min(C / k, lead + P.ramp_step(B, lead * k, C) / k)
            ahead, v_cmd, cap = lead, 0.0, lead
            ramp_left -= 1
            item = fs.items[0]
        else:
            v_goal = 0.0
            c_f = min(C, f_moved * (1 + REL))
            z, z_src = B.slope(s.q, d, part["thing"], mass or 0.0)
            a_dec = B.a_brake(s.q, d)
            if not held:      # a part that is pushed is braked by what resists it alone: the hand may not outrun
                a_dec = min(a_dec, f_moved / mass) if mass else None     # that (no mass fitted yet: no free run)
            v_end = laws.follow_speed(C, f_now, z, v_free)[0] if z is not None else v_seek
            free_run = False
            if until is not None and a_dec is None:
                v_goal = v_end
                item = speed_item(B, v_goal, "toward the amount you stated at the speed of an end not yet found: "
                                  "the part is pushed and its mass is not fitted yet, so nothing says what would "
                                  "brake it", "model" if z is not None else "prior")
            elif until is not None:
                free_run = True
                v_goal = min(v_free, math.sqrt(laws.TWO * a_dec * max(0.0, until - travel - B.rigid(s.q, d, f_now))))
                item = speed_item(B, v_goal, "the arm's free speed, braked into the amount you stated by its own "
                                  f"deceleration ({a_dec:.3g} m/s^2)", B.src("qd_max")[0], "nothing: the robot's own rating")
            elif known_m is not None and travel < known_m and a_dec is not None:
                left = max(0.0, known_m - travel - laws.band(part["sigma_m"]) - B.d_stop(v_end, s.q, d))
                v_goal = min(v_free, math.sqrt(v_end ** 2 + laws.TWO * a_dec * left))
                free_run = v_goal > v_end
                item = speed_item(B, v_goal, "the arm's free speed over the stretch this part was already moved "
                                  "through, braked into its end", "contact", "nothing: the robot's own rating")
            else:
                v_goal = v_end
                item = speed_item(B, v_goal, f"toward an end not yet found: (the end's ceiling {C:.3g} N less the "
                                  f"force now {f_now:.3g} N) over the stop slope {0 if z is None else z:.4g} N per "
                                  f"m/s ({z_src}) with the part's mass {0 if mass is None else mass:.2g} kg in it",
                                  "model" if z is not None else "prior")
            if until is not None:       # and no faster than the arm stops inside what is left of the amount
                room = max(0.0, until - travel - B.rigid(s.q, d, f_now))
                v_stop = laws.speed_for_stop(room, B.t_det(), B.a_brake(s.q, d))
                v_goal = v_goal if v_stop is None else min(v_goal, v_stop)
            if (intent or {}).get("speed") is not None and intent["speed"] < v_goal:
                v_goal = intent["speed"]
                item = speed_item(B, v_goal, "on your word: the lower speed you stated", "intent",
                                  "repeat this call without the speed")
            # a follow whose own law leaves no speed is not run on: the part resists with what this call may push
            # with (the force stands at the ceiling within the accepted relative error of a reading), so that the
            # speed the ceiling leaves over the stop slope is under the slowest speed at which this arm resolves
            # a stop; counted over one settle window
            v_tell = laws.tell_speed(B.rigid(s.q, d, B.f_thr(0, 0)), B.t_det(), B.a_brake(s.q, d))
            slow_n = slow_n + 1 if (not free_run and f_now >= (1 - REL) * C and v_tell is not None
                                    and v_goal < v_tell) else 0
            # the force the lead may hold: toward an end not yet found C_f (what persists against a stop); over
            # a stated amount or a stretch already moved through, the ceiling (the part is sped up and braked)
            c_lead = C if free_run else c_f
            got = float((pts[-1] - pts[-min(len(pts), int(QUANT) + 1)]) @ d)
            suspect = moving_n >= QUANT and got < REL * sum(expected[-int(QUANT):])
            v_cmd = 0.0 if f_now > C else min(v_goal, v + B.a_brake(s.q, d) * dt)
            # the servo's own lag is what this follow shows of it (refit upward at once, Q7): the lead the tip
            # really ran with, less what the force explains, over its measured speed
            if not suspect and v_meas > 0 and moving_n >= QUANT:
                lag_s = max(lag_s, (max(0.0, ahead) - f_now / k) / v_meas)
            cap = c_lead / k + max(B.lag(v_cmd), lag_s * v_cmd)
            # a jump of stiffness: the tip falls far behind what was asked while the force climbs over C_f: the
            # part's hard stop (A4.5), told in QUANT readings instead of a whole settle time at the lead cap
            jump = jump + 1 if (suspect and f_now > c_f * (1 + REL)) else 0
            ahead = min(cap, max(0.0, ahead) + v_cmd * dt)
            free_n += int(v_cmd >= v_free * (1 - REL))
            moving_n += 1
        v = v_cmd
        expected.append(v_cmd * dt)
        at_cap.append(ahead >= cap * (1 - REL))
        if hold_r is None:
            p_cmd = s.tip + ahead * d                       # held: across the path the command is the measured tip
        else:
            p_cmd = p0 + (gone + ahead) * d                 # not held: the hand keeps its own line (nothing would
        R_cmd = R0                                          # bring it back: a hook or a fingertip slides off)
        if hold_r is None and found["shape"] == "arc" and moved_at is not None:
            R_cmd = P.rot(found["axis"], P.angle_between(pts[moved_at] - found["centre"], s.tip - found["centre"])) @ R0
        q_cmd, _ = H.step_to(ex, s.q, p_cmd, R_cmd)
        step = np.asarray(B.qd_max(), dtype=float) * dt
        base = ex._q_des if getattr(ex, "_q_des", None) is not None else s.q
        q_cmd = base + np.clip(q_cmd - base, -step, step)
        tip_before = s.tip
        s = tick(ex, q_cmd)
        v_last, v_meas = v_meas, max(0.0, float((s.tip - tip_before) @ d)) / dt
        v_fast = max(v_fast, v_cmd, float(np.linalg.norm(s.tip - tip_before)) / dt)
        ahead = float((p_cmd - s.tip) @ d)
        pts.append(s.tip.copy())
        felt = np.asarray(s.F, dtype=float) - rest
        forces.append(max(0.0, float(felt @ d)))
        f_side = felt - float(felt @ d) * d
        side_peak = max(side_peak, float(np.linalg.norm(f_side)))
        if state == "ramp" and float((s.tip - x_load) @ d) > B.rigid(s.q, d, forces[-1]) and forces[-1] > thr:
            state, moved_at = "follow", len(pts) - 1
            f_moved = max(forces)
        if state == "follow":
            own = pts[moved_at:]
            travel = float(P.walked(own)[-1])
            alongs.append(travel)
            new = P.fit(own, seat)
            if new["shape"] != "none":
                found = new
                if hold_r is None:
                    d = P.tangent(found, s.tip, d)
            if len(alongs) > n_win and (mass is None or v_cmd < v_goal):
                got_mass = P.part_mass(forces[moved_at - 1:], alongs, dt)
                mass = got_mass if got_mass is not None else mass
        # ---- the ends
        win_exp = sum(expected[-n_win:])
        win_got = float((pts[-1] - pts[-min(len(pts), n_win + 1)]) @ d)
        stalled = len(at_cap) >= n_win and all(at_cap[-n_win:]) and win_got <= P.still_m(B, s.q, d, win_exp)
        loc = getattr(s, "loc", None) or {}
        link = link + 1 if (loc.get("source", "tool") != "tool" and float(s.Fmag) > thr
                            + (f_moved or 0.0) * (1 + REL)) else 0     # over what the part itself resists with
        lost = lost + 1 if (state == "follow" and forces[-1] <= thr and v_cmd > 0) else 0
        lost_at = gone if lost <= 1 else lost_at       # how far the hand stood when the part was last felt
        side_n = side_n + 1 if (state == "follow" and float(np.linalg.norm(f_side)) > side_guard(B, s.q, d, v, max(f_peak, forces[-1]))) else 0
        idle = len(pts) > n_win and win_got <= P.still_m(B, s.q, d, win_exp)      # no progress (A4.8)
        if len(forces) >= QUANT and min(forces[-int(QUANT):]) > C:
            # the force along the way read over the ceiling of this call on QUANT readings in a row: it ends here
            stop, why = ("end" if state == "follow" else "refused"), (f"the force along the way read {forces[-1]:.3g} N, "
                                                                      f"over the ceiling of this call ({C:.3g} N)")
        elif state == "follow" and until is not None and travel >= until - B.rigid(s.q, d, forces[-1]):
            stop = "amount"
        elif state == "follow" and (stalled or jump >= QUANT):
            stop = "end"
        elif state == "follow" and slow_n >= n_win:
            stop, why = "end", (f"it does not move on this way at {forces[-1]:.3g} N: the ceiling of this call is "
                                f"{C:.3g} N, and the speed that leaves ({v_goal * report.MM:.2g} mm/s) is under the "
                                f"slowest at which this arm resolves a stop ({v_tell * report.MM:.2g} mm/s)")
        elif state == "ramp" and lead >= C / k * (1 - REL) and (stalled or ramp_left <= 0):
            stop, why = "refused", f"the part did not move under {lead * k:.3g} N, the ceiling of this call"
        elif state == "seek" and gone > play_m + B.rigid(s.q, d):
            stop, why = "refused", f"felt nothing over {gone * report.MM:.0f} mm along the way"
        elif link >= n_win:
            stop, why = "link_contact", "the force is placed off the tool: " + ", ".join(loc.get("links") or ["a link"])
        elif lost >= n_win and (held or float((s.tip - p0) @ d) - lost_at > play_m + B.rigid(s.q, d)):
            stop, why = "slipped", "the part is no longer felt: the force along the path fell to what free motion shows"
        elif side_n >= n_win:
            stop, why = "contact", f"a side force of {float(np.linalg.norm(f_side)):.3g} N across the path"
        elif state == "follow" and (idle or win_exp <= 0) and (at_cap[-1] or v_cmd <= 0):
            # no progress, and the command no longer gains on the tip (a lead that still grows is still raising
            # the force: such a stroke is not judged still)
            still_n += 1
            if still_n >= n_win and at_cap[-1]:      # it stands against the force that persists: its end
                stop = "end"
            elif still_n >= n_win:
                stop, why = "time", "no progress along the path over a settle time"
        else:
            still_n = 0
        if stop is None and held:
            gap, pressing, sq, _, _ = H.jaw_read(ex, s)
            sq_peak = max(float(sq_peak or 0.0), float(sq or 0.0))
            if gap is not None and (gap < gap_min or not pressing):
                if gap < gap_min or not H.squeeze_up(ex):
                    stop, why = "slipped", "the jaws lost the part"
        if stop is None and state == "follow" and moving_n and not moving_n % n_win:
            q_ahead, _ = H.step_to(ex, s.q, s.tip + (B.d_stop(v, s.q, d) + spacing) * d, R_cmd)
            hits, _ = H.blockers(ex, q_ahead, part["owner"], tool_free=True)
            if hits:
                body, thing, c = hits[0]
                stop, why = "link_contact", (f"{body} would come within {c * report.MM:.0f} mm of {thing}, inside "
                                             "its band, one stopping distance ahead")
    s = tick(ex, s.q, "waiting")
    ex._journal("fixture", stage="follow", stopped=stop, why=why, state=state, periods=len(pts) - 1,
                travel_mm=travel * report.MM, gone_mm=float((s.tip - p0) @ d) * report.MM, moved_from_n=f_moved,
                peak_n=f_peak, ceiling_n=C, mass_kg=mass, jump=jump, lead_mm=lead * report.MM,
                first_n=float(fs.first_n), level_n=float(B.f_thr(0, 0)), k_n_per_m=float(k))
    axis_item = None
    if found["shape"] != "none":
        axis_item = {"shape": found["shape"], "direction": [round(float(x), 4) for x in found["direction"]]}
        if found["shape"] == "arc":
            axis_item.update(centre_mm=[round(float(x) * report.MM, 1) for x in found["centre"]],
                             radius_mm=round(found["radius_m"] * report.MM, 1),
                             plane_normal=[round(float(x), 4) for x in found["axis"]],
                             angle_deg=round(math.degrees(found["angle_rad"]), 1))
    gap1 = H.jaw_read(ex)[0] if held else None
    S.stroke(ex, B, s.q, d, part["thing"], C, f_peak, side_peak, v_fast, max(v_met, v_meas), mass,
             pad_n=float(sq_peak or 0.0) if held else None)
    return {"stopped": stop, "why": why, "moved": bool(moved_at is not None and travel > B.rigid(s.q, d)),
            "travel_m": travel if moved_at is not None else 0.0, "f_moved_n": f_moved, "peak_n": f_peak,
            "side_peak_n": side_peak, "mass_kg": mass, "fit": found, "axis": axis_item, "d": d, "item": item,
            "ceiling_n": C, "free_share": (free_n / moving_n) if moving_n else None, "tip": s.tip.copy(),
            "gap_change_m": None if gap0 is None or gap1 is None else gap1 - gap0, "state": state,
            "start_tip": pts[moved_at].copy() if moved_at is not None else p0,
            "gone_m": float((s.tip - p0) @ d), "play_m": float(play_m), "v_fast_m_s": v_fast,
            "level_n": float(B.f_thr(0, 0)), "level_fast_n": float(B.f_thr(v_fast, 0))}


# ---------------------------------------------------------------------------------------------- the turn
def turn_weights(ex, q, axis):
    """Each joint's share of a turn about axis through the tip (rad of joint per rad of turn), the tip standing."""
    J = ex.model.jac(q)
    e = np.concatenate([np.zeros(len(axis)), unit(axis)])
    return np.linalg.pinv(J) @ e


def turn(ex, part, cand, sense_sign, until, intent, clear_rad=None):
    """Turn the held part about the tool axis under the torque ramp, then follow. sense_sign: +1 turns clockwise
    as the hand sees it (about its own pointing axis), -1 the other way. until: None (its end) or radians.
    clear_rad: how far the posture turns clear of every band (the stroke ends there with stopped 'posture').
    Returns the record."""
    B = H.bounds_of(ex)
    dt = B.dt()
    s = ex._sense()
    p0, R0 = s.tip.copy(), s.R.copy()
    axis = sense_sign * R0[:, 0]
    # along the hand's axis the turn keeps the height the HOLD was taken at, every stroke the same: the command
    # stands at that height and the commanded JOINTS put the tip there (H.stand_at, P.joint_step, H.tip_at: the
    # rotation leads the hand, the place does not give way to it). What the hand still stands off is the arm's
    # own give under the turn's load, taken out as the load goes: corrected inside the turn, not pressed into
    # the part (before, the commanded tip stood 23 to 67 mm off the height asked and the servo drove the hand
    # into the part: 10 mm and 58 N after a turn, measured on a development cell). The drift ends the stroke
    # ("posture") only where the hand stands off its hold by more than its pads are high: the part would leave
    J = ex._fx_jaws
    into, held_at = R0[:, 0], ex._fx.setdefault("hold_at", {})
    p_hold = np.asarray(held_at.setdefault("tip", p0.copy()), dtype=float)
    pad = float(J.pad_height_m or J.finger_len_m or J.max_m)
    still = B.rigid(s.q, R0[:, 0])
    w0 = turn_weights(ex, s.q, axis)
    k_rot = 1.0 / max(float(np.sum(w0 ** 2 / B.robot.kp)), TINY)
    c_nm = P.torque_ceiling(B, s.q, w0)
    max_nm = (intent or {}).get("max_nm")
    if max_nm is not None:
        c_nm = min(c_nm, float(max_nm))
    max_n = (intent or {}).get("max_n") or (((ex._fx.get("limit") or {}).get("limit_n")) if (intent or {}).get("gentle") else None)
    if max_n is not None:       # a force stated for the part (max_n, gentle): at the jaws' radius this torque about the axis
        c_nm = min(c_nm, float(max_n) * (H.jaw_read(ex)[0] or ex._fx_jaws.max_m) / laws.TWO)
    # under a stated force the turn HOLDS what one pad can put on the part to it (the design rulings: kept at the solved speed).
    # What a pad bears as the hand reads it is P.pad_load (its squeeze, its share of the couple about the hand's
    # axis, the whole force the hand reads); a stop of the turning pads adds the stop slope across the axis times
    # their speed. So the pads turn no faster than leaves that sum at the stated force (never under the speed
    # this arm tells a stop at: then said not kept); a reading over it raises no lead and turns no further, and
    # over one settle window it ends the turn. The largest of that sum is what the receipt holds against the
    # limit; the couple alone is no bound (2.75 N read as a couple where one pad put 26.2 N on the part by truth)
    m0, m_peak, over_n = P.wrench_torque(ex.model.jac(s.q), axis, s.ext), 0.0, 0
    pads, load_peak, bound_peak, floor_n, w_lim, v_tell, held_share = len(J.sides), 0.0, 0.0, 0, None, None, 0.0
    sigma_nm = float(np.abs(w0) @ np.asarray(B.tau_thr(0), dtype=float))
    tau0 = P.torque_about(ex.model.jac(s.q)[len(axis):], axis, s.ext)
    lead = max(sigma_nm, P.torque_step(B, 0.0, c_nm, sigma_nm)) / k_rot
    gap0 = H.jaw_read(ex)[0]
    gap_min = float(ex._fx_jaws.width_m(ex._fx_jaws.closed)) + QUANT * H.jaw_step_m(ex)
    resolved = P.turn_resolved(B, w0)
    n_win = P.window_periods(B)
    radius = (gap0 or ex._fx_jaws.max_m) / laws.TWO          # the pads stand half the gap from the hand's axis
    side = unit(np.cross(axis, np.eye(len(axis))[int(np.argmin(np.abs(axis)))]))
    angles, at_cap, expected = [0.0], [], []
    moved, tau_moved, tau_peak, w, stop, why, angle, loaded = False, None, 0.0, 0.0, None, None, 0.0, 0.0
    ramp_left = P.ramp_periods(B, lead * k_rot, c_nm)
    item = report.item("torque", c_nm, "rating", "the turning joints' own torque room over holding this pose; on "
                       "your word to move it", "nothing: the robot's own rating")
    free_n, moving_n = 0, 0
    w_fast, w_met, w_last, w_meas, f_all, sunk, gave, R_was = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, R0.copy()
    sq_top = 0.0          # the largest squeeze one pad read over the turn
    while stop is None:
        if spent(ex):
            stop = "steps_spent"
            break
        wts = turn_weights(ex, s.q, axis)
        tau_now = max(0.0, P.torque_about(ex.model.jac(s.q)[len(axis):], axis, s.ext) - tau0)
        if tau_now > tau_peak:
            w_met = max(w_met, w_meas, w_last)
        tau_peak, f_all = max(tau_peak, tau_now), max(f_all, float(s.Fmag))
        m_now = max(0.0, P.wrench_torque(ex.model.jac(s.q), axis, s.ext) - m0)
        # the hand's force as its guard reads it (the period's end reads a rising push harder), with what the
        # estimate can hide at the speed the pads run
        f_hand = max(float(s.Fmag), float(getattr(s, "Fend", None) or 0.0)) + B.f_thr(w * radius, 0)
        sq_now = H.jaw_read(ex, s)[2]
        sq_top = max(sq_top, float(sq_now or 0.0))
        load_now = P.pad_load(sq_now, m_now, radius, pads, f_hand)
        m_peak, load_peak = max(m_peak, m_now), max(load_peak, load_now)
        over = bool(max_n is not None and load_now > float(max_n))
        over_n = over_n + 1 if over else 0
        w_free = B.w_free(s.q, axis)
        if not moved:
            if not over:
                lead = min(c_nm / k_rot, lead + P.torque_step(B, lead * k_rot, c_nm, sigma_nm) / k_rot)
            ahead, w_cmd, cap = lead, 0.0, lead
            ramp_left -= 1
        else:
            c_f = min(c_nm, max(tau_moved, sigma_nm) * (1 + REL))
            alpha = float(np.min(np.asarray(B.qdd_max(s.q), dtype=float)[np.abs(wts) > TINY]
                                 / np.abs(wts)[np.abs(wts) > TINY])) if np.any(np.abs(wts) > TINY) else 0.0
            z, _ = B.slope(s.q, side, part["thing"])
            w_end = min(w_free, max(0.0, c_nm - tau_now) / (z * radius ** 2)) if z is not None and radius > 0 else w_free
            if until is not None:
                w_goal = min(w_free, math.sqrt(laws.TWO * alpha * max(0.0, until - angle)))
                text = "the wrist's free speed, braked into the angle you stated"
            else:
                w_goal = w_end
                text = "toward an end not yet found: the torque ceiling less the torque now, over the stop slope at the jaws' radius"
            if clear_rad is not None:
                w_goal = min(w_goal, math.sqrt(laws.TWO * alpha * max(0.0, clear_rad - angle)))
            if max_n is not None and z is not None and radius > 0:
                # the stated force, less what a pad bears now, over the stop slope: the speed whose stop keeps it
                w_lim = max(0.0, float(max_n) - load_now) / (z * radius)
                v_tell = laws.tell_speed(B.rigid(s.q, side, B.f_thr(0, 0)), B.t_det(), B.a_brake(s.q, side))
                if v_tell is not None and w_lim * radius < v_tell:
                    # no speed this arm can tell a stop at keeps it: your own word then runs at that slowest
                    # speed and is said not kept, never refused and never crept under it (the design rulings, C3)
                    w_lim, floor_n = v_tell / radius, floor_n + 1
                if w_lim < w_goal:
                    w_goal, held_share = w_lim, held_share + 1.0 - w_lim / w_goal     # what the word costs (fixture_cost)
                    text = (f"the force you stated ({float(max_n):g} N) less what one pad bears now "
                            f"({load_now:.3g} N), over the stop slope at the jaws' radius")
            w_cmd = 0.0 if over else min(w_goal, w + alpha * dt)
            item = report.item("speed", w_cmd * radius, B.src("qd_max")[0], text + f" ({math.degrees(w_goal):.3g} deg/s)",
                               "nothing: the robot's own rating")
            cap = c_f / k_rot + w_cmd * B.lag(1.0)
            ahead = min(cap, max(0.0, cmd_angle - angle) + w_cmd * dt)
            free_n += int(w_cmd >= w_free * (1 - REL))
            moving_n += 1
        w = w_cmd
        expected.append(w_cmd * dt if moved else 0.0)
        at_cap.append(ahead >= cap * (1 - REL))
        cmd_angle = angle + ahead
        # across the axis the command is the measured tip (the hand centres itself on the part's own axis as it
        # turns); along the axis it stands at the height of the hold
        drift = float((s.tip - p_hold) @ into)
        sunk = max(sunk, abs(drift))
        p_cmd = s.tip - drift * into
        q_cmd, _ = H.stand_at(ex, s.q, p_cmd, P.rot(axis, cmd_angle) @ R0, still)
        base = ex._q_des if getattr(ex, "_q_des", None) is not None else s.q
        q_cmd = base + P.joint_step(q_cmd, base, np.asarray(B.qd_max(), dtype=float) * dt)
        q_cmd, _ = H.tip_at(ex, q_cmd, p_cmd, still)     # the step is cut for the joints' speed: the tip stays put
        s = tick(ex, q_cmd)
        # the angle turned is summed period by period (the rotation since the stroke began, read at once, turns
        # its sign at half a turn: a stroke that passed 180 deg read -177 and ended there, measured on a cell)
        was_angle, angle, R_was = angle, angle + P.turned(s.R, R_was, axis), s.R.copy()
        w_last, w_meas = w_meas, max(0.0, angle - was_angle) / dt
        w_fast = max(w_fast, w_cmd, w_meas)
        if max_n is not None:      # what a stop of the pads at the speed they ran would add to what a pad bore
            z_now = B.slope(s.q, side, part["thing"])[0]
            bound_peak = max(bound_peak, load_now + (z_now or 0.0) * max(w_meas, w_last) * radius)
        if tau_now > sigma_nm:          # what the hand turned while the part resisted: the part came along
            loaded += max(0.0, angle - angles[-1])
        angles.append(angle)
        if not moved and angle > resolved + tau_now / k_rot:
            moved, tau_moved = True, tau_peak
        win_exp = sum(expected[-n_win:])
        win_got = angles[-1] - angles[-min(len(angles), n_win + 1)]
        stalled = len(at_cap) >= n_win and all(at_cap[-n_win:]) and win_got <= max(resolved, REL * win_exp)
        gap, pressing, _, _, _ = H.jaw_read(ex, s)
        if until is not None and moved and angle >= until:
            stop = "amount"
        elif clear_rad is not None and moved and angle >= clear_rad - resolved:
            stop, why = "posture", "the hand turned as far as this posture stays clear of every band"
        elif moved and stalled:
            stop = "end"
        elif not moved and lead >= c_nm / k_rot * (1 - REL) and (stalled or ramp_left <= 0):
            stop, why = "refused", f"the part did not turn under {lead * k_rot:.3g} N m, the ceiling of this call"
        elif gap is not None and gap < gap_min:
            stop, why = "slipped", "the jaws lost the part"
        elif over_n >= n_win:
            stop, why = ("end" if moved else "refused"), (
                f"one pad could put {load_now:.3g} N on the part by what the hand read (its squeeze, its share of "
                f"the couple about the hand's axis, the hand's own push), over the {float(max_n):.3g} N you stated")
        elif abs(drift) > pad:
            gave = True
            stop, why = "posture", (
                f"the hand stands {abs(drift) * report.MM:.1f} mm off the height of its hold, more than its pads "
                f"are high ({pad * report.MM:.0f} mm): the part would leave them")
        elif not ex.model.within_limits(q_cmd) or float(np.min(np.minimum(q_cmd - ex.model.q_lo, ex.model.q_hi - q_cmd))) <= 0:
            stop, why = "posture", "a joint reached the end of its range"
    s = tick(ex, s.q, "waiting")
    gap1 = H.jaw_read(ex)[0]
    S.turn(ex, B, s.q, side, part["thing"], c_nm, tau_peak, radius, f_all, w_fast, max(w_met, w_meas),
           weights=turn_weights(ex, s.q, axis), pad_n=sq_top)
    COST.held(ex, held_share, "probe_follow")
    # what one pad could put on the part by the hand's own readings is held against a limit somebody stated for
    # the part, every stroke of the turn (fixture_say._fx_seen): said PASSED where it stood over it
    if max_n is not None:
        said = (f"the turn: one pad could put {bound_peak:.1f} N on the part ({load_peak:.1f} N by what the hand "
                f"read: its squeeze, its share of the {m_peak:.2f} N m read as a couple about the hand's axis, the "
                f"hand's own push of up to {f_all:.1f} N; the rest a stop of the pads at the speed they ran)")
        if floor_n:
            said += (f"; over {floor_n} control periods the force you stated left no speed this arm can tell a "
                     f"stop at, and the pads ran at that slowest speed ({v_tell * report.MM:.2g} mm/s)")
        ex._fx_seen("the turn", bound_peak, said, shown=not floor_n)
    ex._journal("fixture", stage="turn", stopped=stop, why=why, sense=sense_sign, angle_deg=math.degrees(angle),
                peak_nm=tau_peak, ceiling_nm=c_nm, radius_mm=radius * report.MM, sunk_mm=sunk * report.MM,
                along_axis_mm=float((s.tip - p_hold) @ into) * report.MM, push_n=float(np.asarray(s.F) @ into),
                couple_nm=m_peak, pad_load_n=load_peak, pad_bound_n=bound_peak if max_n is not None else None,
                pad_mm=pad * report.MM,
                largest_force_n=f_all, still_mm=still * report.MM)
    return {"stopped": stop, "why": why, "moved": bool(moved and angle > resolved), "angle_rad": max(0.0, angle),
            "tau_moved_nm": tau_moved, "peak_nm": tau_peak, "ceiling_nm": c_nm, "item": item,
            "axis": [round(float(x), 4) for x in axis], "free_share": (free_n / moving_n) if moving_n else None,
            "gap_change_m": None if gap0 is None or gap1 is None else gap1 - gap0, "tip": s.tip.copy(),
            "loaded_rad": loaded, "resolved_rad": resolved, "couple_nm": m_peak, "radius_m": radius,
            "pad_load_n": load_peak, "along_axis_m": float((s.tip - p_hold) @ into), "still_m": float(still),
            "hold_gave": gave}


def turn_sense(ex, part, cand, intent, room, retake):
    """A turn to the part's end whose sense the caller did not state: the sense that GIVES is kept. Clockwise
    first. A part that stands at its stop turns the hand a little before it stalls (the jaws give, they slip
    round the part) and about half of that travel or less is under load, where a part that comes along resists
    nearly all the way (measured on a knob held about its own middle: 26 of 54 and 28 of 56 degrees at its stop,
    56 of 56 and 139 of 139 the way it gives): where the first sense ended on a stop with less than all but REL
    of its travel under load, the part did not come along. The hand then turns back by what it turned, still
    holding (the wind up against the stop goes). Where the part came along on that way back (more than GIVE of
    it under load) the hold is good and the turn goes on that way; else the jaws had slipped round the part: the
    hold is let go and taken again where it was first taken (retake() -> bool), and the part is turned the
    other way.
    Returns (record, sign, strokes, what was found: text or None)."""
    first = turn(ex, part, cand, 1.0, None, intent, clear_rad=(room or {}).get(1))
    stood = first["stopped"] in ("end", "refused", "posture", "slipped") and first["loaded_rad"] <= max(
        first["resolved_rad"], (1 - REL) * first["angle_rad"])
    if not stood or spent(ex):
        return first, 1.0, 1, None
    said = (f"no sense was stated: clockwise the hand turned {math.degrees(first['angle_rad']):.0f} deg and stood, "
            f"{math.degrees(first['loaded_rad']):.0f} deg of it under load: the part did not come along")
    strokes, back = [first], None
    if first["angle_rad"] > first["resolved_rad"]:
        back = turn(ex, part, cand, -1.0, first["angle_rad"], intent)                 # back to where it began
        strokes.append(back)
        came = back["loaded_rad"] > max(back["resolved_rad"], GIVE * back["angle_rad"])
        said += (f"; on the way back {math.degrees(back['loaded_rad']):.0f} of {math.degrees(back['angle_rad']):.0f} "
                 "deg were under load: " + ("the part comes along that way" if came else "the jaws had slipped "
                                            "round it, and the hold was taken again"))
        if not came and not retake():
            # what was measured: the hand turned, part of it under a torque, turned back, and the hold was not
            # found again. Whether the part turned is not shown by that (the design rulings rule 2: an absence is no
            # conclusion; on two development cells the part stood turned by truth where this said it did not turn)
            unsure = (f"clockwise the hand turned {math.degrees(first['angle_rad']):.0f} deg and stood, "
                      f"{math.degrees(first['loaded_rad']):.0f} deg of it under a torque about its axis (the largest "
                      f"{first['peak_nm']:.2f} N m); it turned back {math.degrees(back['angle_rad']):.0f} deg, "
                      f"{math.degrees(back['loaded_rad']):.0f} of them under load; the jaws then did not find the "
                      "part again")
            return dict(first, moved=None, stopped="slipped", why="the jaws did not find the part again",
                        unsure=unsure), 1.0, len(strokes), said.replace(", and the hold was taken again", "")
    strokes.append(turn(ex, part, cand, -1.0, None, intent, clear_rad=(room or {}).get(-1)))
    if back is not None and back["loaded_rad"] > 0:                              # what the way back already turned it
        strokes[-1] = dict(strokes[-1], angle_rad=strokes[-1]["angle_rad"] + back["loaded_rad"],
                           moved=bool(strokes[-1]["moved"] or came))
    return strokes[-1], -1.0, len(strokes), said + "; it was turned counterclockwise"
