# SPDX-License-Identifier: AGPL-3.0-only
"""The tail of a move that did not wait at its end (C10), and what a stroke counts before it starts (C12).

A move with no tolerance stated returns while the hand is still travelling onto the end it was told to reach: its
lag tail. Nothing else runs in that tail but the next call, so the next call owns it:

- a call that MOVES starts its own guard with its first control period (the tail runs under it and a contact there
  is that call's stop);
- a call that acts where the hand stands waits first (arrive below, ex._arrive): it holds the told pose until the
  tip's motion over one detection time is under the still band.

While a hand brakes onto a command that stands still, its force estimate reads the braking itself (measured on the
bench: 20.3 N along the travel, located inside the table, 20 mm before the thing the next touch found at 1 N), and
no self-test measures that level. So in such a wait (arrive, and the settle of a move that waits at its end) only
the guard's CAP ends the wait at once; the contact threshold judges when the hand STANDS, at the level of an arm at
rest (C10: a wait judges the force at rest against the level): a hand that stands on something has met it
(standing()). Such a stop is eased, told to the bounds as found standing (its arrival speed was not measured, so
it never enters a slope fit), stored as a move's contact is, and, in arrive, handed on as ex._tail = {"call",
"stopped", "contact", "periods", "said", "ran"}; the receipts of the motion core carry it as tail_contact, every
other receipt of that call gets its sentence (the notes the call's end collects).

stroke_count() is the budget count of a stroke: the control periods of its plan as it will run (the schedule at the
arrival speed, the ramp, the hold, the settle, the ease), and wait_count() what the wait before it will take by the
hand's own lag.

A move that starts on a contact. Every guard writes where the tool last bore on something over what an arm
at rest can hide (ex._stand). start_of() tells the guard of a move that starts within the tool's own half width
of that place what it starts in: the load the still hand reads (the guard's baseline: no contact the motion
made; so the threshold at rest and the threshold of the move agree about a standing load); that the move LEAVES
the thing (what that thing puts on the tool as the motion starts ends no move, a push that then keeps growing
does: the motion makes it; the contact has ended when the tool read free over its own half width at the body's
own speed; a limit somebody stated for the thing stands until then: plan_leave() runs the hand's first stretch
at the speed that keeps it and the guard ends the move on a reading over it); or that the move goes INTO the
thing (a press, priced as one: press_price()). A move that means a contact keeps its own guard. started(),
off_base() and leaving() are the guard's part of it (rcpx/control_base.py Guard), left_keys() what the receipt
says. No robot is named here.
"""

import math

import numpy as np

from rcpx import bounds_free, control_expect
from rcpx.control_base import MM, Guard, _mm, _ratio, _val


def _speed(ex, s):
    return ex.model.jac(s.q)[:3] @ s.qd


def _behind(ex, s):
    """How far the hand stands from where it was told to be (m)."""
    q_des = getattr(ex, "_q_des", None)
    return 0.0 if q_des is None else float(np.linalg.norm(ex.model.fk_tip(q_des)[0] - s.tip))


def calm(ex, s):
    """The force at rest, less what the hand holds, is at or under the contact level at rest."""
    hold = control_expect._hold_module()
    if hold is not None:
        return hold.rest_force(ex, s) <= hold.level(ex)
    return s.Fmag <= _val(ex.bounds.f_thr(0.0, 0.0))


def rest_levels(ex, held=None):
    """(the contact threshold of an arm at rest on the guard's readings in a row, on one reading) as the bounds
    derive the guard of a hand that stands; (None, None) where that guard has no contact threshold (a hold whose
    load is not read yet)."""
    spec = ex._guard_spec("transit", v=0.0, a=0.0, held=held)
    two = getattr(spec, "force_n", None)
    one = getattr(spec, "one_reading_n", None)
    return (None, None) if two is None else (float(two), float(two if one is None else one))


def standing(ex, g, s, rest):
    """Set the guard g for a hand that waits on a standing command (see the module text): while the hand still
    moves only the cap judges; once it stands (its motion over one detection time under the still band) the
    contact threshold is the one of an arm at rest (rest: rest_levels()). Returns whether the hand stands; g
    .standing says that a stop it then makes was found standing."""
    still = ex._still(s, ex._still_band(s))
    if still and rest[0] is not None:
        g.force_n, g.one_n, g.standing = rest[0], rest[1], True
    else:
        g.force_n, g.n_force, g.first = None, 0, None
    return still


def wait_count(ex):
    """The control periods the wait of arrive() will take, by the hand's own lag: its speed falls by e in the
    time it is behind over its speed, and the wait ends when speed x detection time is under the still band."""
    s = ex._last if ex._last is not None else ex._sense()
    v = float(np.linalg.norm(_speed(ex, s)))
    band, t_det = ex._still_band(s), _val(ex.bounds.t_det())
    behind = _behind(ex, s)
    if v * t_det <= band or not behind > 0.0 or not band > 0.0:
        return 0
    return int(math.ceil((behind / v) * math.log(v * t_det / band) / ex.dt))


def _said(stop, c, v):
    head = (f"while the hand came to rest after the move before (it was still travelling at {v * MM:.0f} mm/s "
            "when this call began) ")
    if stop not in ("contact", "cap"):
        return head + f"a stop was asked ({stop}); the arm was eased there"
    where = "a link of the arm" if c.get("point_mm") is None else f"the tool at {c['point_mm']} mm"
    how = "it stood on it with" if stop == "contact" else "it pressed it with"
    return head + f"it met something: stopped {stop}, {how} {c.get('force_n')} N on {where}; the arm was eased there"


def arrive(ex):
    """SpeedMixin._arrive: wait where the hand was told to stand until it is still, under a guard (see the module
    text). Returns the control periods the call spent here (the caller charges them as arrive_periods); a stop
    is left in ex._tail."""
    B = ex.bounds
    s = ex._sense()
    ex._tail = None
    if ex._still(s, ex._still_band(s)):
        return 0                                           # it stands already: there is no tail to guard
    vel = _speed(ex, s)
    v0 = float(np.linalg.norm(vel))
    u = control_expect._unit(vel)
    ex._travel_direction = u
    held = ex._held_for(None)
    g = Guard(ex, spec=ex._guard_spec("transit", v=v0, held=held))   # its cap, for the travel the hand has
    rest = rest_levels(ex, held)
    window = max(int(_ratio("QUANT")), int(math.ceil(_val(B.t_settle()) / ex.dt)))
    t0 = sum(ex.periods.values())
    was, ex._period_kind = ex._period_kind, "waiting"
    seen, pend, stop = [], 0, None
    try:
        while True:
            if standing(ex, g, s, rest):
                if calm(ex, s) or pend >= g.need:
                    break                                  # it stands, and nothing holds against it
                pend += 1                                  # it stands ON something: the guard counts it out
            else:
                pend = 0
                seen.append(float(np.linalg.norm(_speed(ex, s))))
                if len(seen) >= window + window and max(seen[-window:]) >= (1.0 - _ratio("REL")) * max(
                        seen[-window - window:-window]):
                    break                                  # it gets no stiller: this is how it stands
            s = ex._period(ex._q_des)
            stop = g.check(s)
            if stop:
                break
        if stop:
            if stop == "cap" and not control_expect.own_cap(ex, g, s):
                stop, g.cap_why = "contact", ""
            s = ex._ease_impl()
            if g.asked:
                ex._stop_asked()
            info = ex._stop_note(g, "transit", None, held, s, travel=u) if stop in ("contact", "cap") else None
            if not (info or {}).get("false"):
                if info is not None:
                    ex._record_contact(g.contact, "the tail of the move before", info=info)
                ex._tail = {"call": int(ex._calls), "stopped": stop, "contact": g.contact,
                            "said": _said(stop, g.contact or {}, v0), "why": g.cap_why or None,
                            "ran": None if info is None else info["ran"], "tip_mm": _mm(s.tip)}
                ex.__dict__.setdefault("_guard_notes", []).append(ex._tail["said"])
                ex._journal("tail_contact", **{k: ex._tail[k] for k in ("stopped", "said", "why")})
    finally:
        ex._period_kind = was
    n = sum(ex.periods.values()) - t0
    if ex._tail is not None:
        ex._tail["periods"] = n
    return n


def tail_keys(ex):
    """{"tail_contact": ...} for a receipt of the call in which the tail of the move before met something."""
    tail = getattr(ex, "_tail", None)
    if not tail or tail.get("call") != getattr(ex, "_calls", None):
        return {}
    return {"tail_contact": {k: tail.get(k) for k in ("stopped", "contact", "periods", "said", "why", "ran")
                             if tail.get(k) is not None}}


def finish_count(ex, path, g, recent):
    """A reading over the threshold that the guard has not counted out when a path ends is not left behind: the
    last target is held for the readings its count still needs (none when the last reading was under the
    threshold). Returns (stop or None, the last reading or None)."""
    stop, s = None, None
    while stop is None and g.n_force > 0 and len(path):
        s = ex._period(path[-1])
        recent.append((s, path[-1]))
        stop = g.check(s)
    return stop, s


def stroke_count(planned_s, dt):
    """The budget count of a stroke (C12): the control periods of its plan as it will run, and the one period
    the ease that ends every stroke runs at the least (Executor._ease_impl)."""
    return int(math.ceil(float(planned_s) / float(dt))) + 1


# ---------------------------------------------------------------- a move that starts on a contact
def stand_zone(ex, s):
    """How far the tool is still within reach of what it last bore on (m): the tool's own half width (B.standoff
    with no speed and no thing) plus the still band where the hand stands."""
    return _val(ex.bounds.standoff(0.0)) + ex._still_band(s)


def _stand_thing(ex, rec, zone):
    """The id of the thing the hand stands on: the thing the stroke that made or worked the contact resolved
    (of_intent), else the world's nearest measured point within the zone of where the tool last bore; None."""
    meant = ex.__dict__.get("_meant") or {}
    if meant.get("call") == rec.get("call") and meant.get("thing") is not None:
        return meant["thing"]
    best, found = zone, None
    for kind in ("contact", "located", "device"):
        for e in ex.world.query(kind=kind, include_stale=True):
            if e.get("point_mm") is None:
                continue
            gap = float(np.linalg.norm(np.asarray(e["point_mm"], dtype=float) / MM - rec["tip"]))
            if gap <= best:
                best, found = gap, e.get("id")
    return found


def stand_limit(ex, rec, intent, zone):
    """The limit somebody stated for the thing the hand stands on (the design rulings's limits, never a force_n): max_n or
    gentle on this call or on the call that made or worked the contact, the sheet of that thing where the session
    serves one, a give, the site, a rating, people (B.stated_limit). {"limit_n": None, ...} with none."""
    words = {k: v for k, v in (intent or {}).items() if k in control_expect.LIMIT_WORDS and v}
    meant = ex.__dict__.get("_meant") or {}
    if meant.get("call") == rec.get("call"):
        for k, v in (meant.get("words") or {}).items():
            words[k] = v if k not in words or k != "max_n" else min(float(v), float(words[k]))
    thing = _stand_thing(ex, rec, zone)
    got = dict(ex.bounds.stated_limit(thing, words or None, expected=True))
    got["thing"] = thing
    return got


def start_of(ex, kind, q0, way, intent=None, held=None, meets=False):
    """What a move tells its guard of the contact it starts in, or None. A dict with

    "base": the force the still hand reads as the move begins (N, frame B): the guard's baseline. What the tool
        bears in that direction, up to that size, is the load the move started in and no contact the motion made;
        so the threshold of an arm at rest and the threshold of the move never disagree about a standing load.
    "leave": the hand stands where the tool last bore on something (ex._stand, written by every guard: within
        stand_zone of it) and the move does not go into that thing: {"from", "start": tips (m), "zone_m", "thing",
        "limit_n", "source", "evidence", "slow_m"}. Until the contact has ended the guard ends the move for no
        force on the tool but one over the thing's stated limit (and the cap, and a push on a link).
    "press": the move goes INTO the thing the hand stands on (the tool's last push lies along the travel by the
        give share or more): it is a press and is priced as one (press_price); its guard is the plain one.
    meets: the move means a contact (a kind that may meet a thing, or expect contact): it keeps its own guard and
    takes the baseline only. While something is held the hold's own guard stands (control_hold.Watch): None."""
    s = getattr(ex, "_last", None)
    if s is None or held or not len(way) or getattr(ex, "bounds", None) is None:
        return None
    out = {}
    tool = (getattr(s, "loc", None) or {}).get("source", "tool") == "tool"
    band = ex._still_band(s)
    if tool and s.Fmag > 0.0 and ex._still(s, band):
        out["base"] = np.asarray(s.F, dtype=float).copy()
    rec = ex.__dict__.get("_stand")
    if rec is None:
        return out or None
    zone = stand_zone(ex, s)
    p0 = ex.model.fk_tip(np.asarray(q0, dtype=float))[0]
    if float(np.linalg.norm(p0 - rec["tip"])) > zone:
        ex._stand = None                         # the hand no longer stands where the tool last bore on something
        return out or None
    if meets:
        return out or None
    u = None
    for q in way:                                # the travel as the move begins: the first step the tip can tell
        step = ex.model.fk_tip(np.asarray(q, dtype=float))[0] - p0
        if float(np.linalg.norm(step)) > band:
            u = control_expect._unit(step)
            break
    push = np.asarray(rec["F"], dtype=float)
    size = float(np.linalg.norm(push))
    if u is not None and size > 0.0 and float(push @ u) >= _ratio("GIVE") * size:
        out["press"] = press_price(ex, s, np.asarray(q0, dtype=float), u, rec)
        ex._pressed = (getattr(ex, "_calls", None), out["press"])
        return out
    limit = stand_limit(ex, rec, intent, zone)
    out["leave"] = {"from": np.asarray(rec["tip"], dtype=float), "start": p0, "zone_m": zone, "thing": limit["thing"],
                    "limit_n": limit.get("limit_n"), "source": limit.get("source") if limit.get("limit_n") is not None
                    else None, "evidence": limit.get("evidence"), "slow_m": 0.0}
    return out


def press_price(ex, s, q, u, rec):
    """What a move INTO the thing the hand stands on is priced as: a press. There is no arrival: the tool
    loads the thing by the servo's own stiffness along the travel, from the load it stands with, until the guard
    ends the move on the first reading over its threshold above that load (never over what the arm can push
    there)."""
    B = ex.bounds
    k, cap = _val(B.k_servo(q, u)), control_expect.arm_cap(ex, q, u)
    stand = float(s.Fmag)
    return {"start_n": round(stand, 1), "k_n_per_mm": round(k / MM, 1), "cap_n": round(cap, 1),
            "source": "the servo's stiffness along the travel; the arm's own push there",
            "text": (f"this move starts on the thing the hand stands on and goes into it: a press, not an arrival. "
                     f"It loads the thing by {k / MM:.2f} N per mm the command leads the tip, from the {stand:.2f} "
                     f"N it stands with, until the contact stop (at most {cap:.0f} N, what the arm can push here)")}


def plan_leave(ex, leave, q0, way, kind, target, intent, held):
    """The plan of a move that leaves a thing with a stated limit: over the hand's first zone of
    travel the tool runs no faster than the speed at which a stop stays under that limit (B.plan with an expected
    contact on those stretches: whatever the tool still bears on), from there at the body's own speed. The way is
    split where the zone ends; the plan's waypoint indices are the caller's own. leave gets slow_m and cost_s (the
    seconds the limit cost against the same way with no limit)."""
    B = ex.bounds
    nodes = [np.asarray(q0, dtype=float)] + [np.asarray(q, dtype=float) for q in way]
    tips = [ex.model.fk_tip(q)[0] for q in nodes]
    zone, run, n_slow, at = float(leave["zone_m"]), 0.0, len(way), None
    for k in range(len(way)):
        step = float(np.linalg.norm(tips[k + 1] - tips[k]))
        if run + step >= zone:
            share = (zone - run) / step if step > 0.0 else 1.0
            n_slow = k + 1
            if share < 1.0 - _ratio("REL"):                # the zone ends inside this stretch: a waypoint where it ends
                at = k
                way = list(way[:k]) + [nodes[k] + share * (nodes[k + 1] - nodes[k])] + list(way[k:])
            break
        run += step
    grow = lambda x: x if at is None or not isinstance(x, list) or len(x) != len(way) - 1 else x[:at] + [x[at]] + x[at:]
    said = {"with": None, "radius_m": None, "along": None, "bound_n": float(leave["limit_n"]),
            "bound_source": leave["source"], "seek": True,
            "evidence": f"the hand stands on a thing with a stated limit, kept until it has left it: {leave['evidence']}"}
    plan = B.plan(q0, list(way), grow(kind), target=grow(target), intent=intent or None, held=held,
                  expect=[said] * n_slow + [None] * (len(way) - n_slow))
    if control_expect._attr(plan, "refused") is not None:
        return plan
    idx = getattr(plan, "_segment_indices", None)
    if at is not None and idx is not None:
        plan._segment_indices = [int(i) if int(i) <= at else int(i) - 1 for i in idx]
    if getattr(plan, "guard", None) is not None:
        plan.guard.expect = None                 # the leave is no contact this move means
    leave["slow_m"] = min(zone, float(sum(np.linalg.norm(b - a) for a, b in zip(tips, tips[1:]))))
    try:
        free = B.plan_time(q0, [np.asarray(q, dtype=float) for q in nodes[1:]], kind, target=target,
                           intent=intent or None, held=held)
        leave["cost_s"] = max(0.0, float(plan.time_s) - float(free.time_s))
    except Exception as exc:
        ex._journal("bounds_problem", where="plan_leave cost", error=f"{type(exc).__name__}: {exc}")
    return plan


def left_keys(ex):
    """{"left": ...} for the receipt of a call whose move started on a thing the hand stood on and left it, and
    {"pressed": ...} for one that went into it: what the guard did with that contact, the largest force
    the tool read from it, whether the contact has ended, the limit kept over it with its source and its cost."""
    out, call = {}, getattr(ex, "_calls", None)
    left = ex.__dict__.get("_left")
    if left and left[0] == call:
        lv = left[1]
        went, peak = float(lv.get("went_m") or 0.0) * MM, float(lv.get("peak_n") or 0.0)
        block = {"thing": lv.get("thing"), "ended": bool(lv.get("ended")), "travel_mm": round(went, 1),
                 "peak_n": round(peak, 1), "zone_mm": round(float(lv["zone_m"]) * MM, 1)}
        said = (f"this move started where the tool last bore on something and left it: what that thing put on the "
                f"tool (up to {peak:.1f} N) ended no move")
        if lv.get("limit_n") is not None:
            block.update(limit_n=round(float(lv["limit_n"]), 1), limit_source=lv.get("source"),
                         limit_evidence=lv.get("evidence"), slow_mm=round(float(lv.get("slow_m") or 0.0) * MM, 1))
            if lv.get("cost_s") is not None:
                block["limit_cost_s"] = round(float(lv["cost_s"]), 1)
            said += (f"; its stated limit of {float(lv['limit_n']):g} N ({lv.get('source')}) stood until the contact "
                     f"ended: the first {block['slow_mm']:g} mm ran at the speed that keeps it")
            if lv.get("over_n") is not None:
                block["over_limit_n"] = round(float(lv["over_n"]), 1)
                said += f"; the tool read {float(lv['over_n']):.1f} N, over that limit: the move ended there"
        if lv.get("rose_n") is not None:
            block["rose_n"] = [round(float(x), 1) for x in lv["rose_n"]]
            said += (f"; its push on the tool then rose from {block['rose_n'][0]:g} to {block['rose_n'][1]:g} N as "
                     "the hand went on: the move ended there on a contact it made")
        said += ("; the contact has ended (the tool read free over its own half width)" if lv.get("ended") else
                 "; the contact has NOT ended: the next move leaves it still")
        out["left"] = dict(block, said=said)
    pressed = ex.__dict__.get("_pressed")
    if pressed and pressed[0] == call:
        out["pressed"] = pressed[1]
    return out


def started(g, spec, stated):
    """Guard.__init__: (base, leave) of a guard whose spec carries `start` (start_of): the load the still hand
    read as the move began, as (unit, N), and the record of the thing the move leaves; (None, None) for a guard
    a caller states, one with a direction of its own or one that holds something (the hold's Watch stands)."""
    start = getattr(spec, "start", None)
    if not start or stated or g.hold is not None or g.direction is not None:
        return None, None
    base, leave = start.get("base"), None
    size = 0.0 if base is None else float(np.linalg.norm(base))
    base = (np.asarray(base, dtype=float) / size, size) if size > 0.0 else None
    if start.get("leave") is not None:
        leave = dict(start["leave"], quiet_m=0.0, went_m=0.0, peak_n=0.0, at=None, ended=False, over_n=None,
                     rose_n=None)
        g.ex._left = (getattr(g.ex, "_calls", None), leave)   # what the call's receipt says of it (left_keys)
    return base, leave


def off_base(g, s, f):
    """(size, size on one reading) of a reading less the load the move started in: what the tool bears along
    that load, up to its size, is the contact the hand already stood in, not one the motion made."""
    b, size0 = g.base
    F = np.asarray(s.F, dtype=float)
    size = float(np.linalg.norm(F - min(max(float(F @ b), 0.0), size0) * b))
    return size, size + max(0.0, f - s.Fmag)


def leaving(g, s, size, hard):
    """One reading of a move that leaves the thing it started on: True when the move ends on it (the
    guard's contact is made, with why under its key leaving). It ends on a reading over that thing's stated limit,
    and on a push that grew past the most this leave has read by more than the move's own threshold: the first
    loaded reading is the contact the hand stood in, as the start of the motion shows it, and is the level from
    then on; a push that keeps growing while the hand goes on is a contact the motion MAKES. Any other reading
    ends nothing. The contact HAS ended once the tool read free (under the move's own threshold) over a travel
    of its zone at the body's own speed, past the stretch a stated limit slowed: from then on the guard is the
    plain one and the hand stands on nothing."""
    lv = g.leave
    last, lv["at"] = lv["at"], s.tip
    step = 0.0 if last is None else float(np.linalg.norm(s.tip - last))
    lv["went_m"] += step
    if not size > g.force_n:
        if float(np.linalg.norm(s.tip - lv["start"])) >= lv["slow_m"]:
            lv["quiet_m"] += step
            if lv["quiet_m"] >= lv["zone_m"]:
                lv["ended"], g.leave = True, None
                g.ex._stand = None
        return False
    level, lv["quiet_m"], lv["peak_n"] = lv["peak_n"], 0.0, max(lv["peak_n"], hard)
    if lv["limit_n"] is not None and hard > lv["limit_n"]:
        lv["over_n"] = hard
        why = (f"the tool read {hard:.1f} N, over its stated limit of {lv['limit_n']:g} N ({lv['source']})")
    elif level > 0.0 and hard - level > g.force_n:
        lv["rose_n"] = (level, hard)
        why = (f"its push on the tool rose from {level:.1f} N to {hard:.1f} N as the hand went on: a contact this "
               "motion made")
    else:
        return False
    g.first = g.first if g.first is not None else s
    g.n_force += 1                     # a stop a thing made: counted, with the impulse it put on the tool
    g.impulse += hard * float(getattr(g.ex, "dt", 0.0) or 0.0)
    g.contact = g.ex._contact_info(g.first, s)
    g.contact["leaving"] = "the thing the hand was leaving: " + why
    return True


def speed_said(ran_m_s, solved_m_s):
    """The speed a stroke's receipt states for its way in (mm/s): the speed its command RAN at over the arrival
    stretch; a stroke with no such stretch: the speed solved for it; None when neither is known."""
    v = ran_m_s if ran_m_s is not None else solved_m_s
    return None if v is None else round(float(v) * MM, 2)


def limit_block(lim, ran, link=None, at_s=None):
    """What a move says of the limit somebody stated on it (the design rulings: kept or passed is said,
    for the tool and for a link alike). lim: the plan's limit {limit_n, source, shown: every arrival of the plan
    under it is shown kept by this arm's own resolved stops, cost: what the limit added and where it held, zone:
    the plan's stretches (rcpx/bounds_free.py)}; ran: the stop's own record (strike.ran) or None when the move
    met nothing; link: the body the push was located on when it is no tool body; at_s: the path coordinate of
    the stop (m). kept is True only where the stop's own figure stands at or under the limit and the plan's
    speed is shown to keep it; False where a reading passed it; None where nothing was met or it is not shown.
    A contact on a stretch that was known free (the limit did not hold it, the design rulings) is said PASSED with its
    figure when that figure is over the limit or was not read; kept only when its figure stands at or under it."""
    limit_n, source = float(lim["limit_n"]), lim.get("source")
    head = f"the limit of {limit_n:g} N ({source})"
    base = {"limit_n": limit_n, "source": source}
    if lim.get("cost"):
        base["cost"] = lim["cost"]
    if not ran:
        return dict(base, kept=None, read_n=None, said=f"{head} set this motion's speed where a contact can begin, "
                    "for the tool and for the arm's links; it felt nothing")
    read, peak = ran.get("read_n"), ran.get("peak_n")
    bound = "at most" in str(ran.get("note") or "")        # the peak is an upper bound, not a measurement
    on = f" on {link}" if link else ""
    most = max([v for v in (read, None if bound else peak) if v is not None], default=None)
    out = dict(base, read_n=None if most is None else round(float(most), 2))
    if bounds_free.outside(lim.get("zone"), at_s):         # met in space that was known free
        top = max([v for v in (read, peak) if v is not None], default=None)
        where = (f"{on} outside the band where a contact can begin: that space was known free ("
                 + "; ".join(lim["zone"].get("by") and [bounds_free.WORDS[b] for b in lim["zone"]["by"]] or ["no source"])
                 + "), so the leg ran at the body's own speed there")
        if top is None or top > limit_n:                   # the figure, never "kept" (the bet was lost)
            said = (f"{head} was PASSED{where}; the contact it met made "
                    + ("a force that was not read" if top is None else f"{'at most ' if bound else ''}{top:.1f} N"))
            return dict(out, kept=False, outside_band=True, read_n=None if top is None else round(float(top), 2), said=said)
        said = (f"{head}: a contact was met{where}; it made {'at most ' if bound else ''}{top:.1f} N, at or under "
                "the limit")                               # every figure of the contact stands under the limit
        return dict(out, kept=True, outside_band=True, read_n=round(float(top), 2), said=said)
    if most is not None and most > limit_n:
        return dict(out, kept=False, said=f"{head} was PASSED{on}: {most:.1f} N read at the stop")
    top = peak if peak is not None else read
    if top is not None and top <= limit_n and lim.get("shown"):
        return dict(out, kept=True, said=f"{head} was kept{on}: {top:.1f} N at the stop, at a speed this arm's own "
                    "resolved stops show to keep it")
    why = ("the stop's upper bound is over it" if top is not None and top > limit_n else
           "the speed rests on the body's model of the stop slope, or lies under the slowest this arm resolves")
    return dict(out, kept=None, said=f"{head} is not shown kept{on}: "
                + ("no force was read at the stop" if most is None else f"{most:.1f} N read at the stop") + f"; {why}")
