# SPDX-License-Identifier: AGPL-3.0-only
"""The close and the release of the hold (the record and the guard are in control_hold.py).

close(): no arm guard at the contact threshold judges it, only the cap. A close that makes a hold is an expected
contact whose purpose is a force that stays, the hold's squeeze: the free travel down to where the
pads can first meet the thing (its width as measured, stated or seen, with its error; nothing known: from where the
jaws stand) runs at the jaws' own rate, and from there the pads arrive at the rate solved for that squeeze over the
jaws' own stop slope, (squeeze - floor_n) / z_jaw, never under rate_tell and never over their own rate (rates()).
That is no limit: the receipt states the squeeze, the speed of the arrival and the peak. A stated limit L lowers the
rate further, to at most (L - floor_n) / z_jaw and never under rate_tell (z_jaw carries what the jaw servo winds up
before a touch is told), and in the hold squeeze the command never leads the reading by more than L allows through
the pad's shortest lever. A limit the operator's own word states under what these jaws leave standing once they
touch runs at rate_tell and is said not kept (never a refusal); a sheet, a rating, the site or a give
under it is refused limit_not_keepable. "Told" is the jaws' own
reading: the lag over the free lag of the running rate by REL of it plus TWO_READ spreads, on QUANT readings in a
row; or, once the command has stood at its end for the jaws' own lag, jaws that stand (no longer running) open of
where they meet each other and press (control_hold_jaws.presses: open of their command by more than a free jaw
at rest) for QUANT readings in a row: they stand on a thing (a squeezed thing creeps, and the told test takes a
creeping jaw for a running one); the closing command then already stands on it and nothing is wound (with no
limit and no squeeze asked). Then the command goes onto the reading, then to the hold squeeze. The arm's estimate is read through the
whole close and said when the squeeze went into the world. A close that the limit or the arm's own cap ended
keeps the command on the reading until the jaws stand; the record is made only where a touch was told.
release(): opening is an expected event (the load leaves); it stops short of the jaws' own open end. An opening
whose jaws never stood open of where they began (nor of the most closed they came to) did not open: nothing was let
go, and the hold stands.

SI inside; rates in spans per s, openings and lags in shares of the span. Nothing here names a robot.
"""

import math
from collections import deque

import numpy as np

from rcpx import bounds_ratios as R
from rcpx import control_hold as H
from rcpx import control_hold_pads as pads_of
from rcpx.context import refusal
from rcpx.control_hold_jaws import facts, numbers, presses, read, scene_touch, spread_at

MM = 1000.0


def _arrive(ex):
    """Before acting where the hand stands, wait by motion alone (ex._arrive, the motion core's). Returns
    (the periods it waited, how far the hand then stands from where it was last sent, m): the jaws act where the
    hand IS, and a caller sees how far that is from the end it commanded."""
    arrive = getattr(ex, "_arrive", None)      # compat shim: a core without it does not wait
    n = 0
    try:
        n = int(arrive()) if callable(arrive) else 0
    except Exception as exc:
        ex._journal("hold_problem", where="arrive", error=f"{type(exc).__name__}: {exc}")
    s, q_des = ex._sense(), getattr(ex, "_q_des", None)
    off = 0.0 if q_des is None else float(np.linalg.norm(
        ex.model.fk_tip(np.asarray(q_des, dtype=float))[0] - s.tip))
    return n, off


def _limit(ex, limit_n, limit_source):
    """The stated limit a close runs under ({limit_n, source, evidence}) or None: the caller's, else what the
    bounds hold as stated for an expected contact (the site, a rating, a give of this session)."""
    if limit_n is not None:
        return {"limit_n": float(limit_n), "source": limit_source or "intent", "evidence": "stated on the call"}
    ask = getattr(ex.bounds, "stated_limit", None)     # compat shim: bounds without it state none
    if ask is None:
        return None
    try:
        got = ask(None, None, expected=True)
    except Exception as exc:
        ex._journal("hold_problem", where="stated_limit", error=f"{type(exc).__name__}: {exc}")
        return None
    if not isinstance(got, dict) or got.get("limit_n") is None:
        return None
    return {"limit_n": float(got["limit_n"]), "source": got.get("source") or "site",
            "evidence": str(got.get("evidence") or "")}


def _steps_left(ex):
    left = getattr(ex.backend, "steps_left", None)
    left = left() if callable(left) else None
    return None if left is None else int(left)


def _cap_guard(ex):
    from rcpx.control_base import Guard        # here: control_base imports this module
    return Guard(ex, None, joint_guard=False, hold=False)   # no contact threshold judges a close or an opening


def at_depth(F, N, depth_m=None):
    """(the jaws' numbers, the pad's shortest lever) for a thing that reaches depth_m up the fingers from their
    tips: a hinged jaw presses harder the nearer its hinge the thing stands, so the floor and the stop slope grow
    by the levers' ratio. None: the depth the hand's own rule takes a thing at (the numbers as stored). The
    numbers carry the pads' friction cone (facts(): cone) for the figures of pad_peak()."""
    lever = F["lever_min"]
    N = dict(N, cone=float(F.get("cone") or 1.0))
    if depth_m is None:
        return N, lever
    lever = np.asarray(F["J"].pad_levers(depth_m)[0], dtype=float)
    deeper = float(np.max(F["lever_min"] / lever))
    return dict(N, floor_n=N["floor_n"] * deeper, z_jaw=None if not N.get("z_jaw") else N["z_jaw"] * deeper), lever


def squeeze_goal(F, lever_min, at_frac, to_frac, squeeze_n=None):
    """The squeeze a hold is closed for (N on the jaw that presses hardest, through the pad's shortest lever): the
    squeeze the caller asks (stated at the jaw tips), else what the jaw servo leaves standing when its command
    ends at to_frac and the jaws stand on a thing at the opening at_frac (its gain times that lead less the
    joint's friction, inside its own force range). None where the body states no servo gain."""
    if F["kp"] is None:
        return None
    if squeeze_n is not None:
        push = float(squeeze_n) * F["lever_tip"]
    else:
        push = F["kp"] * max(0.0, float(at_frac) - float(to_frac)) * F["span"] - F["friction"]
    if F.get("force_range") is not None:
        push = np.minimum(push, np.asarray(F["force_range"], dtype=float))
    return float(np.max(np.maximum(push, 0.0) / lever_min))


def rates(F, N, lever_min, frac, *, expect_m=None, band_m=None, depth_m=None, lim=None, squeeze_n=None,
          to_frac=0.0, hold=True, wide_m=None):
    """The rates of one close before the jaws move (pure). N, lever_min: at_depth(). Returns
    {own, tell, begin (the opening at which the pads can first meet a thing expect_m wide, None: not known),
     slow_from (the opening down to which the jaws run at their own rate), rate (the rate from there, spans per s),
     squeeze_goal_n, squeeze_rate (the rate solved for the hold's squeeze, None: no stop slope or no servo gain),
     solved (the rate solved for the stated limit), kept, under_floor (the limit lies under what the jaws leave
     standing once they touch), peak_n (what one pad can make on a rigid thing backed by the other jaw: the
     arrival's strike, floor_n + z_jaw x the arrival speed by the jaws' own stop slope, AND the squeeze the close
     then winds up on it: the strike has not died when the squeeze builds, so the two are added (an upper term
     each); times 1 + REL where that slope is the body's model; None: no stop slope), peak_fast_n (the same for
     a thing met while the jaws still run at their own rate: one that stands wider between them than was
     stated or seen), sat (the lead of the command over the reading at which the jaw servo's force stands on
     its own range, share of the span; None: it states none), stall (the opening the pads are expected to stand
     at, None: no width)}. wide_m: the widest the thing can stand between the jaws (a width that was stated
     with no bearing: the thing may be turned, and its diagonal is the gap the pads can first meet it at): the
     arrival stretch begins there."""
    own = float(N["speed"])
    tell = float(N["rate_tell"]) or float(min(N.get("rates") or [own]))      # the slowest rate they resolve
    tell = min(tell, own)
    z, floor, per = N.get("z_jaw"), float(N["floor_n"]), float(N["gap_per_span_m"])
    reach = None if expect_m is None else max(float(expect_m) + float(band_m or 0.0), float(wide_m or 0.0))
    begin = None if reach is None else F["J"].frac_for_reach(reach, depth_m)
    stall = None if expect_m is None else min(frac, F["J"].frac_for_reach(float(expect_m), depth_m))
    at = frac if begin is None else min(frac, begin)
    goal = squeeze_goal(F, lever_min, at, to_frac, squeeze_n) if hold else None
    rate, squeeze_rate, solved, kept, under = own, None, None, None, False
    if goal is not None and z:
        squeeze_rate = (goal - floor) / (z * per)
        rate = min(own, max(tell, squeeze_rate))
    if lim is not None:
        under = bool(lim["limit_n"] < floor)
        if z:
            solved = (lim["limit_n"] - floor) / (z * per)
        rate = min(rate, max(tell, solved if solved is not None else 0.0))
        kept = bool(not under and solved is not None and solved >= tell and N.get("z_jaw_source") != "model")
    rate = max(rate, np.finfo(float).tiny)
    sat = None
    if F["kp"] is not None and F.get("force_range") is not None:
        sat = float(np.max((np.asarray(F["force_range"], dtype=float) + F["friction"]) / (F["kp"] * F["span"])))
    return {"own": own, "tell": tell, "begin": begin, "slow_from": frac if rate >= own else at, "rate": rate,
            "squeeze_goal_n": goal, "squeeze_rate": squeeze_rate, "solved": solved, "kept": kept,
            "under_floor": under, "peak_n": pad_peak(N, rate, goal, lim is not None),
            "peak_fast_n": pad_peak(N, own, goal, lim is not None),
            "sat": sat, "stall": stall}


def pad_peak(N, rate, squeeze_n, limited=False):
    """What one pad can make on a rigid thing backed by the other jaw when the pads meet it at rate (spans per s)
    and the close then leaves squeeze_n standing on it (N per pad): the strike of that arrival by the jaws' own
    stop slope plus that squeeze (module text of rates()); a slope that is the body's model carries the
    accepted relative error of a fit on top (as the arm's does). limited: the close runs under a stated limit:
    the arrival is solved for the limit and the squeeze is wound up no faster and held to it, so the strike has
    died before the squeeze stands: the larger of the two, not their sum. None where the jaws state no stop
    slope."""
    z = N.get("z_jaw")
    if not z:
        return None
    hit, sq = float(N["floor_n"]) + float(z) * float(rate) * float(N["gap_per_span_m"]), float(squeeze_n or 0.0)
    peak = max(hit, sq) if limited else hit + sq
    peak = peak * (1.0 + R.REL) if N.get("z_jaw_source") == "model" else peak
    return peak * float(N.get("cone") or 1.0)       # the push along the closing axis inside the pads' friction cone


def met_of(rows, slow_from, own, rate):
    """Where and how fast the pads met the thing, from the close's own readings up to the period its reading first
    told the thing (pure). rows: [(reading, measured rate, command)] per period (openings as shares of the span,
    rates in spans per s). Returns (the rate the pads met it at, the largest lead of the command over the reading
    over the same periods, "fast" | "arrival"). The pads met it after the jaws' reading first came to slow_from
    (where the arrival stretch begins): the fastest the jaws ran from that period on, never under the arrival
    rate. Unless the thing was met in the stretch run at the jaws' own rate: the reading told it while still wider
    than slow_from (it stands wider than stated or seen), or the jaws fell under GIVE of the fastest they had run
    while their command still ran at their own rate (a stall on something in that stretch); then it is the
    fastest they ran over the whole close, never under their own rate."""
    if not rows:
        return rate, 0.0, "arrival"
    top, fast = 0.0, rows[-1][0] > slow_from
    for frac, v, cmd in rows:
        if cmd > slow_from and top >= R.GIVE * own and v < R.GIVE * top:
            fast = True
        top = max(top, v)
    if fast:
        return max(own, top), max(0.0, max(f - c for f, _, c in rows)), "fast"
    k = next((i for i, x in enumerate(rows) if x[0] <= slow_from), len(rows) - 1)
    part = rows[k:]
    return max(rate, max(v for _, v, _ in part)), max(0.0, max(f - c for f, _, c in part)), "arrival"


def planned_periods(P, frac, to_frac, dt):
    """The control periods a close with the rates P plans from the opening frac to to_frac: the free travel at
    the jaws' own rate, the arrival stretch and the wind up of the squeeze at the arrival rate; where the pads
    are expected to stand (a width is known) and the jaw servo's force stands on its own range from some lead on
    (sat), the rest of the command's way from that lead on at the jaws' own rate again (it adds no force)."""
    slow_to = to_frac
    if P.get("stall") is not None and P.get("sat") is not None:
        slow_to = max(to_frac, min(P["slow_from"], P["stall"] - P["sat"]))
    return int(math.ceil(max(0.0, frac - P["slow_from"]) / (P["own"] * dt))
               + math.ceil(max(0.0, P["slow_from"] - slow_to) / (P["rate"] * dt))
               + math.ceil(max(0.0, slow_to - to_frac) / (P["own"] * dt)))


def _price(N, P, lim):
    """The close's own line: the rate the pads arrived at, what set it, the peak that rate can make and what the
    figure rests on."""
    rate, own, per = P["rate"], P["own"], N["gap_per_span_m"]
    v = rate * per
    rests = (f"the jaws' own stop slope ({N['z_jaw'] / MM:.2f} N per mm/s, "
             + ("the body's model: these jaws give no force reading" if N.get("z_jaw_source") == "model" else
                "measured") + ")") if N.get("z_jaw") else None
    peak = "" if P["peak_n"] is None else (f"; one pad can make {P['peak_n']:.0f} N on a rigid thing backed by the "
                                           f"other jaw (the strike of that arrival plus the squeeze it leaves), by "
                                           f"{rests}"
                                           + ("" if rate >= own or P.get("peak_fast_n") is None else
                                              f"; a thing that stands wider between the jaws than that is met at "
                                              f"their own rate: {P['peak_fast_n']:.0f} N"))
    goal = P["squeeze_goal_n"]
    if lim is None:
        if P["squeeze_rate"] is None:
            return (f"the pads arrived at {v * MM:.0f} mm/s, the jaws' own rate (no rate is solved for the hold's "
                    "squeeze: " + ("this body states no contact model for its jaws" if not N.get("z_jaw") else
                                   "the jaw servo states no gain, or no hold was asked") + f"){peak}; gentle: "
                    "state a limit (max_n)")
        how = ("the jaws' own rate: the rate solved for it is faster" if rate >= own else
               "the slowest rate these jaws resolve: the rate solved for it is slower" if P["squeeze_rate"] < P["tell"]
               else "the rate solved for it over the jaws' stop slope")
        free = "" if rate >= own else (
            f"; down to where the pads can first meet the thing the jaws ran at their own {own * per * MM:.0f} mm/s"
            if P["begin"] is not None else
            "; the thing's width is not known, so the jaws ran at that rate from where they stood (a width, or a "
            "point in a look that shows the thing, leaves only its band slow)")
        return (f"the pads arrived at {v * MM:.1f} mm/s for the hold's squeeze of {goal:.1f} N ({how}){free}{peak}; "
                "gentle: state a limit (max_n)")
    each = "" if not lim.get("pads") else (f" of {lim['whole_n']:g} N on the thing, a share for each of the "
                                           f"{lim['pads']} pads that press it at once")
    head = f"its stated limit is {lim['limit_n']:g} N{each} ({lim['source']}): closed at {v * MM:.1f} mm/s at the touch"
    solved = P["solved"]
    if P["kept"]:
        return head + ", from the jaws' own measured stop slope: the limit is kept" + peak
    if P["under_floor"]:
        return head + (f", the slowest rate these jaws resolve: once they touch they cannot press under "
                       f"{N['floor_n']:.2f} N (their joint's own friction over the pad's shortest lever), so the "
                       f"limit is not kept; the smallest limit these jaws keep is {N['floor_n']:.2f} N") + peak
    if solved is None:
        return head + (", the slowest rate these jaws resolve: they give no force reading and state no contact "
                       "model, so the limit is not shown kept")
    if solved < P["tell"]:
        return head + (f", the slowest rate these jaws resolve (the limit asks {solved * per * MM:.2f}"
                       f" mm/s){peak}: the limit is not shown kept")
    return head + (f" = (limit - {N['floor_n']:.2f} N the jaw's own friction leaves) over the jaws' stop slope, "
                   f"which is the body's model ({N['z_jaw'] / MM:.2f} N per mm/s; these jaws give no force "
                   "reading): the limit is not shown kept")


def close(ex, *, expect_m=None, band_m=None, limit_n=None, limit_source=None, squeeze_n=None, hold=True,
          to_frac=0.0, arm=None, depth_m=None, kind=None, pads=1, width_source=None, look=True, wide_m=None):
    """The one close. expect_m, band_m: the gap between the jaws at which the thing is seen to begin, and its
    error (the jaws run at their own rate down to there even under a limit; a caller that knows the thing's width
    but not where it stands between the jaws states none, and a close under a limit is then slow from where the
    jaws stand). limit_n, limit_source: a stated limit on the thing; pads: how many pads press the thing at once
    (the thing sees their sum: each pad's strike and squeeze is held to its share of the limit, limit_n / pads;
    the hold's record keeps the whole limit for the legs that carry the thing). squeeze_n: the hold squeeze
    the caller asks (N at the jaw tips); none: the closing command stands (the jaw servo's own force, as the plain
    interface closes). hold False: the jaws rest on the thing, the command on the reading. to_frac: the opening
    the command ends at. depth_m: how far up the fingers from their tips the thing reaches (none: the depth the
    hand's own rule takes a thing at; a hinged jaw presses harder the nearer its hinge the thing stands, so the
    pad's shortest lever, the floor and the stop slope are those of that depth). arm(frac) -> joint target: the arm's own target for each period (a hand that moves with
    its jaw). kind "attached": what is taken is a part of something fixed (the record says so from the start). Returns {ended: on_thing | closed | limit | time (or a stop request's text), held, on_thing, gap_m,
    frac, squeeze_n, arm_read_n, rate_span_s (the rate the pads arrived at), arrive_m_s, squeeze_goal_n, peak_n
    (what one pad can make at that speed by the jaws' own stop slope), periods, arrive_periods, arrive_off_m,
    price, limit_n, limit_source, kept} or the refusal limit_not_keepable (a sheet, a rating, the site or a give
    under what the jaws leave standing: the jaws did not move) or budget. on_thing: the jaws' own reading
    told a thing between them. held: it did, the jaws stand on it (the stated limit or the arm's own cap may have
    ended the squeeze) and hold was asked: the record stands. A close that ended on the limit or the cap before
    any touch was told holds nothing.
    Width_source says where expect_m came from ("stated", "seen:
    ..."); with no width given and hold asked the robot looks for itself first (control_hold_look.before: a
    picture of its own camera that sees between the jaws, no control period; look False: the caller looked
    already, or wants none) and the close runs on that width with its error. The result gains "strike": {pad_n,
    other_n, arrive_m_s, source} (control_hold_pads.strike), "width": {m, sigma_m, source, and where no width was
    known "price": {periods, with_width_periods}}; the hold's record gains "squeeze_pad_n" and "pads".
    wide_m: the widest the thing can stand between the jaws (rates()).
    peak_n and strike.pad_n are ONE number, stated after the jaws stood: for the rate the jaws themselves
    met the thing at by their own readings (met_of: in the stretch run at their own rate where the reading told
    it wider than the arrival's begin, or they stalled there), plus the squeeze (the one read standing, never
    under the one the close was solved for nor the servo's push at the lead its command wound up), inside the
    pads' friction cone; met_how in the result says which stretch. The squeeze is
    wound up at the arrival rate only while the command's lead adds force; from the lead at which the jaw
    servo's force stands on its own range the command runs on at the jaws' own rate."""
    F = facts(ex)
    frac, _ = read(ex, ex._sense())
    if F is None or frac is None:
        return refusal("gripper", "this robot has no jaws, or its jaws give no reading",
                       "use touch and press with the tip")
    dt, J = float(ex.dt), F["J"]
    N, lever_min = at_depth(F, numbers(ex), depth_m)      # the numbers of the depth the thing stands at
    whole = _limit(ex, limit_n, limit_source)
    pads = max(1, int(pads or 1))
    lim = whole if whole is None or pads == 1 else dict(whole, limit_n=whole["limit_n"] / pads, pads=pads,
                                                        whole_n=whole["limit_n"])
    to_frac = min(1.0, max(0.0, float(to_frac)))
    width = {"m": None if expect_m is None else float(expect_m), "sigma_m": None if expect_m is None else
             float(band_m or 0.0), "source": width_source or ("stated" if expect_m is not None else None)}
    if expect_m is None and hold:
        from rcpx import control_hold_look
        width = control_hold_look.before(ex, look)        # the robot's own look before the close
        if width.get("m") is not None:
            expect_m, band_m = float(width["m"]), R.BAND * float(width["sigma_m"] or 0.0)
    frac0 = frac
    P = rates(F, N, lever_min, frac, expect_m=expect_m, band_m=band_m, depth_m=depth_m, lim=lim,
              squeeze_n=squeeze_n, to_frac=to_frac, hold=hold, wide_m=wide_m)
    if P["under_floor"] and lim["source"] != "intent":
        return refusal("limit_not_keepable", f"its stated limit is {lim['limit_n']:g} N ({lim['source']}); "
                       f"these jaws cannot press under {N['floor_n']:.2f} N once they touch (their joint's own "
                       f"friction over the pad's shortest lever, {N['source']}): not closed",
                       f"nothing on this call: the limit is stated by the {lim['source']}")
    own, rate, kept = P["own"], P["rate"], P["kept"]
    plan_n = planned_periods(P, frac, to_frac, dt)
    left = _steps_left(ex)
    if left is not None and plan_n > left:
        return refusal("budget", f"this close would take {plan_n} control periods and {left} are left"
                       + ("" if lim is None else f" (its stated limit of {lim['limit_n']:g} N sets the rate)"),
                       "state the thing's width so that only its band is closed slowly"
                       + ("" if lim is None else ", or a larger limit"))
    arrive_n, arrive_off = _arrive(ex)
    g = _cap_guard(ex)
    s = ex._sense()
    F0 = np.asarray(s.F, dtype=float).copy()
    a, b = (float(v) for v in N["lag"])
    still = float(N["still_band"])
    n_lag = int(math.ceil(b / dt)) + int(R.QUANT)
    recent, fracs = deque([0.0] * n_lag, maxlen=n_lag), deque([frac], maxlen=n_lag)
    n_max = plan_n + int(ex._wait_limit()) + n_lag
    cmd, k, n_over, end, arm_n, arm_now, tail, told, capped = frac, 0, 0, "time", 0.0, 0.0, 0, frac, []
    shut = N["closed_gap"] * (1.0 + R.REL) + R.QUANT * still       # at or under this the jaws met each other
    n_press, by_tail = 0, False      # readings in a row, the command at its end, that the jaws stand on something;
    # by_tail: the close was told so (not by the lag over the running free lag)
    rows, met_rate, lead_max, met_how = [], None, 0.0, None     # every period's (reading, the rate the jaws
    # themselves ran at, command): at the period their reading first told the thing met_of() reads from them the
    # rate it was MET at and the lead the command stood at over the reading

    def cap():
        """Under a limit: how far the command may lead the reading in the hold squeeze (share of the span) for
        the jaw that presses hardest to stay at the limit: the jaw servo's own push through the pad's shortest
        lever, plus what the arm reads at this moment (the part of the squeeze that goes into the world, a thing
        pressed against what it stands on or against one jaw, comes on top of the servo's push on one side).
        (In the closing run the lead at the touch is the free lag plus what the QUANT readings of "told" add:
        the stop slope carries that term, so the rate solved from the limit already keeps it.)"""
        if lim is None or F["kp"] is None:
            return None
        left = max(0.0, lim["limit_n"] - arm_now)
        return max(0.0, float(np.min((left * lever_min - F["friction"]) / F["kp"] / F["span"])))
    was, ex._period_kind = ex._period_kind, "jaw"

    def stand():
        """The jaws stand: over the lag's own settling they moved under half of what would call the hold lost."""
        return len(fracs) == n_lag and \
            max(fracs) - min(fracs) <= R.GIVE * (still + R.REL * max(0.0, frac - cmd))

    def step(new_cmd):
        """One period at the command new_cmd: (told this reading, the word that ends the close or None). Told:
        the lag stands over the free lag of the rate the jaw was told to run or runs by itself (whichever is
        faster: a jaw that starts, or that runs on after its command ended, is on its own free line)."""
        nonlocal cmd, s, frac, arm_n, arm_now, k
        recent.append((cmd - new_cmd) / dt)
        if new_cmd != cmd:
            scene_touch(ex)             # the jaws are commanded to move: the scene may change
        cmd = ex._gripper_cmd = new_cmd
        was_at = frac
        s = ex._period(ex._q_des if arm is None else arm(cmd))
        k += 1
        frac, lag = read(ex, s)
        fracs.append(frac)
        arm_now = float(np.linalg.norm(np.asarray(s.F, dtype=float) - F0))
        arm_n = max(arm_n, arm_now)
        stop = g.check(s)
        if stop:
            capped.append(g.cap_why)
            return False, ("limit" if stop == "cap" else str(stop))
        if whole is not None and arm_n > whole["limit_n"]:
            return False, "limit"
        moved = was_at - frac       # a jaw that moved under its own still band this period stands (a jaw that
        runs = max(max(recent), moved / dt if moved > still else 0.0)       # creeps on a thin thing is not running)
        rows.append((frac, max(0.0, moved / dt), cmd))
        return bool(lag is not None
                    and lag > (a + b * runs) * (1.0 + R.REL) + R.TWO_READ * spread_at(N, runs)), None
    try:
        while k < n_max:
            # the arrival stretch begins where the COMMAND reaches slow_from: the jaws trail their command by
            # their own lag at their own rate and coast that far on, so the JAWS, not their command, have come
            # to the arrival rate where the pads can first meet the thing (switched on the reading, the jaws
            # stood at that gap still at their own rate: 640 to 710 N a pad by truth on a rigid thing)
            r = own if cmd > P["slow_from"] else rate
            over, stop = step(max(to_frac, cmd - r * dt))
            if stop:
                end = stop
                break
            if over and n_over == 0:         # where and how fast the pads met it, from the jaws' own readings
                met_rate, lead_max, met_how = met_of(rows, P["slow_from"], own, rate)
            n_over = n_over + 1 if over else 0
            if n_over >= int(R.QUANT):
                end = "on_thing"
                break
            tail = tail + 1 if cmd <= to_frac else 0
            # the command has stood at its end for the jaws' own lag (n_lag periods): jaws that STAND (stand(): a
            # jaw still running down to where the jaws meet each other does not) open of that gap and open of
            # their command by more than a free jaw at rest stands from it (presses) stand ON something. The told
            # test above reads a jaw that creeps on a squeezed thing as a running one, with the scatter of a
            # commanded stroke, and told such a close "met each other" 9 to 12 mm open
            n_press = n_press + 1 if tail >= n_lag and frac > shut and stand() and presses(ex, s) else 0
            if tail >= n_lag + n_lag:
                end = "on_thing" if n_press >= int(R.QUANT) else "closed"   # "closed": nothing held them
                if end == "on_thing":
                    by_tail = True
                    met_rate, lead_max, met_how = met_of(rows, P["slow_from"], own, rate)
                break
        if end == "on_thing" and frac <= shut:
            end = "closed"                   # the jaws met each other
        on = end == "on_thing"               # the jaws' own reading told a thing between them
        told = frac
        # told at the command's end with no limit and no squeeze asked: the closing command already stands on
        # the thing, as the plain interface closes (its squeeze is the servo's own at that lead): nothing to wind
        stands_on = by_tail and lim is None and squeeze_n is None
        if end in ("on_thing", "limit") and not stands_on:
            step(frac)                       # the command onto the reading
        if on and hold and not stands_on:
            lead = None if squeeze_n is None or F["kp"] is None else float(np.min(
                (float(squeeze_n) * F["lever_tip"] + F["friction"]) / F["kp"] / F["span"]))
            reach = frac - to_frac if lead is None else min(frac - to_frac, lead)
            reach = reach if lim is None else min(reach, cap())      # how far the command has to lead the reading
            for _ in range(int(math.ceil(max(0.0, reach) / (rate * dt))) + int(ex._wait_limit()) + n_lag):
                goal = to_frac if lead is None else max(to_frac, frac - lead)
                goal = goal if lim is None else max(goal, frac - cap())
                # the squeeze winds up at the arrival rate while the lead adds force; from the lead at which the
                # jaw servo's force stands on its own range the command's way adds none: the jaws' own rate
                wind = rate if P["sat"] is None or frac - cmd < P["sat"] else own
                _, stop = step(max(goal, cmd - wind * dt))
                if stop:
                    end = stop
                    break
                if cmd <= goal and stand():
                    break
        if end == "limit":
            # the stated limit or the arm's own cap ended it: the command stays on the reading until the jaws
            # stand (no squeeze is left wound up, and what is recorded is where they came to rest)
            for _ in range(int(ex._wait_limit()) + n_lag):
                step(frac)
                if stand():
                    break
        if on and frac <= shut:
            end, on = "closed", False        # it gave way under the squeeze: the jaws met each other
    finally:
        ex._period_kind = was
    q, target = J.reading(s.gripper, ex._gripper_cmd)
    sq = None if q is None else J.squeeze_n(q, target)
    sides = None if q is None else pads_of.by_side(F, q, target, depth_m)     # per pad where it stands
    gap = None if q is None else float(J.width_m(q))
    # ONE figure after the jaws stood: the strike of the rate they met the thing at (met_of; the planned arrival
    # rate where nothing was told) plus the squeeze, per pad through the lever at the depth the thing stands at:
    # the squeeze they read standing, never under the one the close was solved for nor under the jaw servo's own
    # push at the largest lead its command stood at over the reading while they met it (a command that ran on
    # over jaws that stood on the thing winds that push up), all inside the pads' friction cone (pad_peak)
    met = rate if met_rate is None or end not in ("on_thing", "limit") else max(rate, float(met_rate))
    stands = None if sq is None else float(sq) * float(np.max(np.asarray(F["lever_tip"], dtype=float) / lever_min))
    wound = None if met_rate is None else squeeze_goal(F, lever_min, lead_max, 0.0)
    peak = pad_peak(N, met, max([float(v) for v in (stands, P["squeeze_goal_n"], wound) if v is not None] or [0.0]),
                    lim is not None)
    out = {"ended": end, "held": False, "on_thing": bool(on), "gap_m": gap, "frac": frac, "squeeze_n": sq,
           "pads_by_side": sides, "arm_read_n": arm_n, "rate_span_s": rate, "periods": k, "arrive_periods": arrive_n,
           "arrive_off_m": arrive_off,
           "price": _price(N, P, lim), "limit_n": None if lim is None else lim["limit_n"],
           "limit_whole_n": None if whole is None else whole["limit_n"],
           "arrive_m_s": rate * N["gap_per_span_m"], "met_m_s": met * N["gap_per_span_m"],
           "squeeze_goal_n": P["squeeze_goal_n"], "peak_n": peak, "peak_planned_n": P["peak_n"],
           "peak_fast_n": P["peak_fast_n"],
           "z_jaw": N.get("z_jaw"), "z_jaw_source": N.get("z_jaw_source"),
           "limit_source": None if lim is None else lim["source"], "kept": kept, "jaw_numbers": N["source"],
           "told_gap_m": float(J.width_m(J.q_of_frac(told))) if end in ("on_thing", "limit") else None}
    thr = H._num(ex.bounds.f_thr(0.0, 0.0)) or 0.0
    if arm_n > max(thr, float(N.get("arm_free_n") or 0.0)):
        out["pressed_world_n"] = round(arm_n, 2)
        if out["peak_n"] is not None:       # the arm's own push on the thing comes on top of one pad's figure
            out["peak_n"] += arm_n
        out["price"] += (f"; the arm read {arm_n:.2f} N while the jaws closed: the thing was pressed against what "
                         "it stands on, or against one jaw")
    out["met_how"] = met_how
    if met_how == "fast" and peak is not None:
        out["price"] += (f"; the jaws met the thing while they still ran at their own rate (up to "
                         f"{met * N['gap_per_span_m'] * MM:.0f} mm/s: it stands wider between them than was stated "
                         f"or seen): one pad can have made up to {out['peak_n']:.0f} N")
    elif met > rate * (1.0 + R.REL) and peak is not None:
        out["price"] += (f"; the jaws ran up to {met * N['gap_per_span_m'] * MM:.0f} mm/s from where the pads could "
                         f"first meet the thing: one pad can have made up to {out['peak_n']:.0f} N")
    if float(N.get("cone") or 1.0) > 1.0 and peak is not None:
        out["price"] += (f"; each figure of a pad is its push along the closing axis times {float(N['cone']):.2f}, the "
                         "friction cone of the jaws' own shapes (a thing that grips harder than they do is not "
                         "bounded by it)")
    if capped and capped[0]:
        out["price"] += f"; the close ended on the arm's own cap: {capped[0]}"
    if end == "limit" and not on:
        out["price"] += "; it ended before the jaws' own reading told a thing between them: nothing is held"
    if lim is None and on and hold and sq:
        out["price"] += f"; the closing command then stands on it: {sq:.1f} N of squeeze by the jaw servo's own lag"
    out["strike"] = pads_of.strike(F, N, P, out, frac0, arm_n)
    out["width"] = width
    if width.get("m") is not None and gap is not None:
        # how far the jaws stand wider than the width they ran to, over its own band (0: they stand on it as it
        # was stated, seen or looked): said, never an ending here; the caller judges what it took
        out["wider_m"] = max(0.0, float(gap) - float(width["m"]) - float(band_m or 0.0))
    if width.get("m") is None and hold and rate < own:
        # no width was stated, seen or looked: the arrival rate ran from where the jaws stood. Its price
        # beside what a width would have cost: the same close with the jaws' own rate down to where they stood on
        # the thing (its band as wide as the jaws' own still band says)
        # beside what the same close would plan with the width the jaws now stand at stated (its band the jaws'
        # own still band): the free travel at their own rate, the arrival, the wind up of the squeeze, the rest
        at = told if end in ("on_thing", "limit") else to_frac
        w_at = out.get("told_gap_m")
        if w_at is not None and w_at > 0.0:
            # the SAME close as rates() plans it with the width the jaws now stand at handed in as a caller hands
            # it, with the band this close was given: stated (no bearing: the thing may stand turned, so the
            # arrival begins at its diagonal, place_pick_count.wide_of) and from a look that shows the thing
            def plan_with(wide):
                Pw = rates(F, N, lever_min, frac0, expect_m=w_at, band_m=band_m, depth_m=depth_m, lim=lim,
                           squeeze_n=squeeze_n, to_frac=to_frac, hold=hold, wide_m=wide)
                return planned_periods(Pw, frac0, to_frac, dt) + n_lag
            with_w, with_look = plan_with(math.hypot(w_at, w_at)), plan_with(None)
        else:
            Pw = dict(P, slow_from=min(frac0, at + R.QUANT * still), stall=at)
            with_w = with_look = planned_periods(Pw, frac0, to_frac, dt) + n_lag
        width["price"] = {"periods": int(k), "with_width_periods": int(with_w), "with_look_periods": int(with_look),
                          "pad_n": None if out["peak_n"] is None else round(float(out["peak_n"]), 1)}
        out["price_periods"] = dict(width["price"])
        out["price"] += (f"; no width was known and no camera of this body measured between the jaws: the close "
                         f"cost {k} control periods; with its width stated about {with_w} (it may stand turned: "
                         f"the arrival begins at its diagonal), with a look that shows the thing and how it is "
                         f"turned about {with_look} (the same arrival and the same squeeze)")
    elif str(width.get("source") or "").startswith("looked"):
        out["price"] += f"; its width was read before the close by {width['source']}"
    if on and hold:                                # the jaws stand on a thing: what is between them is held
        rec = H.Hold(since_call=int(ex._calls), gap_m=gap, squeeze_n=sq,
                   squeeze_source="the jaw servo's lag at rest (its gain times the lag, less the joint's friction, "
                                  "at the jaw tip)",
                   rest_n=np.asarray(s.F, dtype=float).copy(),
                   rest_ext=None if getattr(s, "ext", None) is None else np.asarray(s.ext, dtype=float).copy(),
                   leg_in_m=H._leg_in(ex), weight_n=None, mass_kg=0.0, sigma_n=None, read_at=None, kind=None,
                   level_n=0.0, level_source="", limit_n=None if whole is None else whole["limit_n"],
                   limit_source=None if whole is None else whole["source"], extent=None,
                   frac=float(frac), cmd_frac=float(ex._gripper_cmd), still_frac=still,
                   judged_at=np.asarray(s.tip, dtype=float).copy())
        rec["pads"] = pads_of.stated(F, N, lever_min, out)       # what the pads bear and can make
        rec["squeeze_pad_n"] = rec["pads"]["squeeze_pad_n"]
        rec["squeeze_by_body"] = rec["pads"]["squeeze_by_body"]
        rec["depth_m"] = None if depth_m is None else float(depth_m)
        if kind == "attached":      # the caller takes a part of something fixed: no load is waited for
            rec.update(kind="attached", weight_n=0.0)
        H._relevel(ex, rec)
        ex.__dict__["_hold"] = rec
        ex.__dict__["_hold_ended"] = None
        out["held"] = True
        ex._journal("hold_began", gap_mm=None if gap is None else round(gap * MM, 2), squeeze_n=sq,
                    leg_in_mm=round(rec["leg_in_m"] * MM, 1), level_n=round(rec["level_n"], 3))
    return out


def release(ex, to_frac=None):
    """Open the jaws at their own rate and end the hold: the load leaving the hand is expected, so no arm guard at
    the contact threshold judges the opening, only the cap. It never leaves the jaws on their own open end (the
    opening where the free jaws rest less QUANT still bands). An opening jaw that meets something stands there
    (the command on the reading); a jaw whose reading closes past where it began has lost what held it open (the
    thing left the pads as the squeeze went) and opens on. Jaws whose reading, once the opening has run, stands
    open neither of where it began nor of the most closed it came to did not follow the command: ended
    did_not_open, nothing was let go, and a hold that stood stands again with its command back on its squeeze.
    Returns {ended: opened | did_not_open | on_thing | limit (the arm's cap, with why) | time, or a stop request's
    text; frac, was_holding, periods, arrive_periods, arrive_off_m}."""
    F = facts(ex)
    frac, _ = read(ex, ex._sense())
    if F is None or frac is None:
        return refusal("gripper", "this robot has no jaws, or its jaws give no reading",
                       "use touch and press with the tip")
    N, dt = numbers(ex), float(ex.dt)
    arrive_n, arrive_off = _arrive(ex)
    rec0, cmd0 = H.held(ex), ex._gripper_cmd
    was_holding = rec0 is not None
    H._end(ex, "released")
    still = float(N["still_band"])
    top = float(N["open_rest"]) - R.QUANT * still
    goal = top if to_frac is None else min(float(to_frac), top)
    rate = float(N.get("speed_open") or N["speed"])
    a, b = (float(v) for v in N["lag"])
    n_lag = int(math.ceil(b / dt)) + int(R.QUANT)
    recent, fracs = deque([0.0] * n_lag, maxlen=n_lag), deque([frac], maxlen=n_lag)
    g = _cap_guard(ex)
    cmd, k, n_over, end, tail, why, start = frac, 0, 0, "time", 0, None, frac
    low = frac                       # the most closed the jaws' reading stood over the opening
    n_max = int(math.ceil(max(0.0, goal - frac) / (rate * dt))) + int(ex._wait_limit()) + n_lag
    if goal <= frac:
        return {"ended": "opened", "frac": frac, "was_holding": was_holding, "periods": 0, "arrive_periods": arrive_n,
                "arrive_off_m": arrive_off}
    was, ex._period_kind = ex._period_kind, "jaw"
    try:
        while k < n_max:
            new = min(goal, cmd + rate * dt)
            recent.append((new - cmd) / dt)
            if new != cmd:
                scene_touch(ex)         # the jaws are commanded to move: the scene may change
            cmd = ex._gripper_cmd = new
            was_at = frac
            s = ex._period(ex._q_des)
            k += 1
            frac, lag = read(ex, s)
            fracs.append(frac)
            low = min(low, frac)
            stop = g.check(s)
            if stop:
                end, why = ("limit", g.cap_why) if stop == "cap" else (str(stop), None)
                break
            runs = max(max(recent), (frac - was_at) / dt)
            # an opening jaw that MEETS something stands while its command opens; one whose reading closes past
            # where it stood on the thing has lost what held it open (the thing left the pads as the squeeze
            # went): it trails its opening command by that closing, and it is no meeting (it opens on)
            over = lag is not None and frac >= start - still \
                and -lag > (a + b * runs) * (1.0 + R.REL) + R.TWO_READ * spread_at(N, runs)
            n_over = n_over + 1 if over else 0
            if n_over >= int(R.QUANT):
                end = "on_thing"
                break
            tail = tail + 1 if cmd >= goal else 0
            if tail >= n_lag + n_lag:
                end = "opened"
                break
        # the jaws' own reading after the whole opening: jaws that never stood open of where they began, nor of the
        # most closed they came to, did not follow the opening command (the thing left the pads as the squeeze went
        # closes them first and they then open on; jaws that do neither still stand on what they held)
        # (judged only where the command ran further open than a free jaw at rest trails it plus two still bands)
        followed = frac > start + still or frac > low + still or not goal - start > a + still + still
        if end == "opened" and not followed:
            end = "did_not_open"
        if end != "opened":
            ex._gripper_cmd = frac               # stand where it met it: no push past
            ex._period(ex._q_des)
            k += 1
        if end == "did_not_open" and rec0 is not None:
            # nothing was let go: the hold stands as it stood, its command back on the squeeze it had
            ex._gripper_cmd = cmd0
            ex.__dict__["_hold"], ex.__dict__["_hold_ended"] = rec0, None
            ex._journal("hold_kept", why="the jaws did not follow the opening command",
                        frac=round(float(frac), 4), started=round(float(start), 4))
    finally:
        ex._period_kind = was
    out = {"ended": end, "frac": frac, "was_holding": was_holding, "periods": k, "arrive_periods": arrive_n,
           "arrive_off_m": arrive_off}
    if end == "did_not_open":
        out["why"] = (f"the jaws were told to open and their own reading did not follow: it stood at "
                      f"{float(frac):.4f} of their span where it began at {float(start):.4f} (the most closed "
                      f"{float(low):.4f}), within their still band {still:.4f}: "
                      + ("the hold stands" if rec0 is not None else "nothing was held"))
    if why:
        out["why"] = why
    return out
