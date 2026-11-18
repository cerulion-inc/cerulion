# SPDX-License-Identifier: AGPL-3.0-only
"""The approach of a press, its rounds and what it states after its stop.

A press that starts off the part has an arrival like a touch. Its approach ends at the force the press's ramp begins
with: force_n when the call states one (then it is also the bound of the push, the design rulings), else the ramp's own first
force (the least force this arm tells on two readings). The approach runs at the body's own speed only through space
known free (rcpx/bounds_free.py: a look, the arm's own passage, the site) and, from the first band where a contact
can begin (the stated support's band among them), at the speed solved for that force: the ramp's first force as an
ending force, never under the tell speed (B.arrival, C9); a stated force_n as the bound of the push, kept at its
solved speed also under the tell speed (the design rulings: an approach met faster strikes the part over force_n, and the stroke
then stops on its force before the part has moved). With no way known free the whole travel is the arrival
stretch, priced in the receipt's band block. max_n, gentle, a sheet, the site and a rating stay limits beside it.

Once the part MOVED under the stroke, the stroke has its own pace (end_stop_speed): a part that runs can run into its
end stop, which takes the force on the part plus the stop slope times the speed, so the command runs no faster than
keeps that at or under the force asked, and never under the tell speed. That pace holds the whole stroke
after the part moved: while the readings stand over the contact threshold, and in a stretch where they went quiet
behind a part whose force sits at the threshold (the stroke's free plan is timed for the approach and is slower). A
reading that comes back while the tip went on no further than the arm's own give explains is the same contact
(same_contact): the command keeps its lead.

After the stop the press states ONE figure for what it made (strike.ran): the larger of the strike its arrival made
at the speed the part was MET at (the guard's level there plus the stop slope times that speed), the largest push
it read with the level the estimate hides at rest and the rise the readings showed carried over one detection
time (the stop acts that late), taken up by the estimate's own error, and what an end stop could have taken while
the part ran. Where no stop slope stands the arrival's share is unresolved and no figure is stated. Nothing
is typed; no robot is named.

    approach(intent, force_n, ramp)                      -> the intent the approach is planned and timed with
    end_stop_speed(most_n, f_n, z, v_tell)               -> the stroke's pace once its part moved (m/s), or None
    quiet_pace(v_free, stroke_v, most_n, f_n, z, v_tell) -> that pace where the readings went quiet behind the part
    same_contact(lost_m, tr_m, tell_m, f_n, k_s)         -> True: a reading back after a quiet spell is the same contact
    limit_keys(arr, peak_n, strike_n)                    -> (limit_n, source, kept) of the least LIMIT stated
    ran(B, legs, peak_total_n)                           -> strike.ran of the press, or None (nothing resisted)
    pressed_to(peak_n, legs, stop)                       -> the force the press pressed to (the reading at a cap)
    rounds(ex, s, d, travel, top, ramp, max_rounds, kw, ran_cb, hold_s) -> (r, rounds, slack, peak, s, p_start)
"""

import math

from rcpx.bounds_ratios import REL


def approach(intent, force_n, ramp):
    """The approach's intent: with force_n stated, the call's own (force_n bounds the approach and the push);
    with none, the ramp's first force as the force the approach ends at (an ending force, never a limit)."""
    first = float(ramp.first_n or 0.0)
    if force_n is not None or not (math.isfinite(first) and first > 0.0):   # (no force the arm can tell: as built)
        return dict(intent)
    said = (f"no force_n stated: the approach of this press ends at the ramp's first force, {first:.3g} N (the "
            "least this arm tells on two readings); no bound on the way in")
    return {**intent, "ends_n": first, "ends_said": said}


def end_stop_speed(most_n, f_n, z, v_tell):
    """The fastest a part that runs under a press is followed (m/s): the speed at which the force on it (f_n) plus
    the stop slope z (N per m/s) times the speed stays at or under most_n (the force asked, else what the arm can
    push: the end stop it may run into takes that), never under the tell speed v_tell. None where no stop
    slope stands: nothing bounds the pace here."""
    if not z:
        return None
    return max(float(v_tell), max(0.0, float(most_n) - max(float(f_n), 0.0)) / float(z))


def quiet_pace(v_free, stroke_v, most_n, f_n, z, v_tell):
    """The stroke's own pace (m/s) in a stretch where the readings went quiet behind a part that moved (its force
    sits under the contact threshold): the body's speed v_free, a stated limit's stroke speed stroke_v (max_n, a
    sheet, the site; None: none), and the end stop's bound (end_stop_speed), whichever is least. The stroke's free
    plan is timed for the approach; behind a part that moved the command runs no slower than this."""
    v = float(v_free) if stroke_v is None else min(float(v_free), float(stroke_v))
    end = end_stop_speed(most_n, f_n, z, v_tell)
    return v if end is None else min(v, end)


def same_contact(lost_m, tr_m, tell_m, f_n, k_s):
    """True when a resisting reading that comes back after the readings went quiet is the contact the stroke
    already has: since the readings went quiet (the tip's travel lost_m; None: they never did, a first contact)
    the tip went on to tr_m no further than the arm's own give at that force explains (the least travel it tells
    plus the force over the servo's stiffness, B.rigid). Else the tip ran on and met something again: a new
    arrival."""
    return lost_m is not None and float(tr_m) - float(lost_m) <= float(tell_m) + max(float(f_n), 0.0) / float(k_s)


def limit_keys(arr, peak_n, strike_n=None):
    """(limit_n, source, kept): the least LIMIT stated on the approach (max_n, gentle, a sheet, the site, a
    rating, a give), never the force the approach ends at; kept as StrokeMixin._kept says (None with none)."""
    expect = arr["expect"]
    lim, src = expect["bound_n"], expect["bound_source"]
    if expect.get("floor") is not None:
        lim, src = expect["floor"]
    if lim is None:
        return None, None, None
    return lim, src, bool(arr["kept"]) and not max(float(peak_n), float(strike_n or 0.0)) > float(lim)


def ran(B, legs, peak_total_n):
    """strike.ran of a press (C2, the design rulings ii): {"v_m_s": the measured tip speed along the push where the part was
    met, "peak_n": the figure (None when the arrival's share is unresolved), "arrive_n", "read_n", "pushed_n",
    "end_stop_n", "note"}; None when nothing resisted."""
    first = next((lg for lg in legs if lg.get("t_resist") is not None), None)
    if first is None:
        return None
    v = first.get("v_met")
    z = next((lg["z_end"] for lg in legs if lg.get("z_end") is not None), None)
    e = float(B.e_tau() or 0.0)
    read = float(peak_total_n)
    hidden = float(B.f_thr(0.0, 0.0))
    # the push: the largest reading, the level the estimate hides at rest, and the force still rising over one
    # detection time at the fastest rise the readings showed (the stop acts that late), with the estimate's error
    n_det = math.ceil(float(B.t_det()) / float(B.dt()))
    climb = max((float(lg.get("rise_n") or 0.0) for lg in legs), default=0.0) * n_det
    pushed = (read + hidden + climb) / (1.0 - e) if e < 1.0 else None
    hit = None if z is None or v is None else float(B.f_thr(float(v))) + float(z) * float(v)
    end = max((float(lg.get("strike_n") or 0.0) for lg in legs), default=0.0)
    out = {"v_m_s": None if v is None else float(v), "read_n": round(read, 2),
           "pushed_n": None if pushed is None else round(pushed, 2),
           "arrive_n": None if hit is None else round(hit, 2), "end_stop_n": round(end, 2) if end > 0.0 else None}
    if hit is None:
        out.update(peak_n=None, note="the strike of the arrival is unresolved: no stop slope stands for this body "
                                     "here; the push it read is beside it")
        return out
    figs = [x for x in (hit, pushed, end if end > 0.0 else None) if x is not None]
    out.update(peak_n=round(max(figs), 2),
               note="the larger of the arrival's strike at the speed the part was met at, the push read with the "
                    "estimate's own error and the level it hides at rest, and what an end stop could have taken")
    return out


def pressed_to(peak_n, legs, stop):
    """The force a press pressed to (N): its largest push held over two readings; at a cap the reading that
    passed the cap (one reading ends a cap), when it is larger."""
    if stop != "cap":
        return float(peak_n)
    return max([float(peak_n)] + [float(lg.get("read_stop_n") or 0.0) for lg in legs])


def rounds(ex, s, d, travel, top, ramp, max_rounds, kw, ran_cb, hold_s=None):
    """A press in rounds: the chain's base half takes up the load to just under the force asked, its tool half
    finishes it, until a round gains neither travel the tip can tell nor force the estimate can tell.
    Returns (the last push or None, rounds, slack m, peak N, the reading, the tip where it began)."""
    B = ex.bounds
    prox, dist = ex._rounds_split(s.q, d)
    p_start = s.tip.copy()
    eps = ramp.rate_n_s(0.0) * B.t_det()
    r, n, slack, peak = None, 0, 0.0, 0.0
    while max_rounds is None or n < int(max_rounds):
        n += 1
        at, was = float((s.tip - p_start) @ d), peak
        r = ran_cb(ex._push(s, d, travel - at, top * (1.0 - REL), mask=prox, stall=True, stop_on_force=True, **kw))
        if r["stop"] == "refused":
            break
        fr = r["first_resist_m"]
        slack += max(0.0, fr if fr is not None else r["travel_m"])
        peak, s = max(peak, r["peak_n"]), r["s"]
        if r["stop"] in ("cap", "travel", "slipped", "off_axis", "moved") or r["gave"] or r["asked"]:
            break
        left = travel - float((s.tip - p_start) @ d)
        r = ran_cb(ex._push(s, d, max(left, B.rigid(s.q, d)), top, hold_s=hold_s, mask=dist, q_start=r["q_des"],
                            stall=True, **kw))
        if r["stop"] == "refused":
            break
        peak = max(peak, r["peak_n"])
        s = ex._ease_impl()
        if r["gave"] or r["slipped"] or r["stop"] in ("cap", "travel", "off_axis", "moved") or r["asked"]:
            break
        # a round that gained neither travel the tip can tell nor force the estimate can tell ends it
        if float((s.tip - p_start) @ d) - at <= B.rigid(s.q, d) and peak - was <= eps:
            break
    return r, n, slack, peak, s, p_start

