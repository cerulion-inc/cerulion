# SPDX-License-Identifier: AGPL-3.0-only
"""The calibration's taps at more than one start.
Never inside a task or a scored cell: the self-test (rcpx/bounds_selftest.py run) calls run() where
it called the straight taps of rcpx/bounds_selftaps.py.

The straight taps climb from the tell speed by doubling and stop where the run to a steady speed no longer fits
the height the hand stands at. So the fastest stop (V1) sat at a third to a half of what the tools run (the
one bench arm: 664 mm/s for a free speed of 2162 mm/s), and every faster leg says "not measured above V1". Here:

- start(ex, B, know, q, u): the start a ladder needs for its top: the hand raised along the support's normal by
  what the run to the top speed needs over the height it stands at, as far as the arm keeps its line (each step
  of the command moves the tip along the normal within REL of the step, no joint leaves its range).
- across(ex, B, q, normal): a second pose: the hand moved along the support plane, across the line from the base
  to the hand, by GIVE of the hand's distance from the base's axis, as far as the arm keeps its line; None when
  no joint turns by more than REL rad on the way (no other pose: the fit counts poses by that step).
- run(ex, know, q0, at, log): a ladder of straight taps (bounds_selftaps.taps) from the raised start over where
  the hand stands, and one more from the raised start over the second pose; the hand comes back to q0 after each.
  A raised ladder that stores no tap (its way down met something over the support: the raise can drift the column
  by up to REL of its length) is run again from where the hand stood, as the straight taps always were.
  The entry "taps" holds every ladder ("ladders") and their rows together; "used" counts every tap stored. Each
  tap is measured and stored as the straight taps are (a tap's pose, travel and body; contract C5).
How long it takes is said in the entry ("took_s") and in the self-test's own time.

No tap is made with a LINK. A build that brought a link onto the support and read its push from the joint
torques (the force along the normal at the link's lowest point, or at its shortest lever there) read every one
of its taps UNDER the true peak, 0.36 to 0.83 of it on four arms (the design notes): a link's
push cannot be read from the joints well enough to bear a figure, so no stop is made with one and a link's
strike is said "not measured" with the body's model beside (rcpx/bounds_strike.py).
"""

import math

import numpy as np

from rcpx import bounds_laws as laws
from rcpx import bounds_stops as stops
from rcpx.bounds_ratios import GIVE, QUANT, REL

MM = stops.MM
STEPS = int(round(1 / REL ** 2))     # the line checks of a raise or a move across: steps of 1/25 of its length


def _support(B):
    """(unit normal, offset m) of the stated support plane, or None."""
    got = B.space.support()
    if got is None:
        return None
    (normal, offset), _, _ = got
    normal = np.asarray(normal, dtype=float)
    size = float(np.linalg.norm(normal))
    return normal / size, float(offset) / size


def _in_range(model, q):
    lo, hi = getattr(model, "q_lo", None), getattr(model, "q_hi", None)
    if lo is None or hi is None:
        return True
    lo, hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
    limited = hi > lo                                   # a joint the model states no range for has lo == hi
    return bool(np.all(~limited | ((q >= lo) & (q <= hi))))


def _keeps_line(ex, q, direction, dist):
    """(the pose after the tip went dist along direction by STEPS command steps, how far it went m): it stops at
    the last step whose tip moved along direction within REL of the step and whose joints stay in range."""
    from rcpx import bounds_selftaps as taps
    step, went = dist / STEPS, 0.0
    q = np.asarray(q, dtype=float).copy()
    for _ in range(STEPS):
        nxt = taps._step(ex, q, direction, step)
        moved = np.asarray(ex.model.fk_tip(nxt)[0], dtype=float) - np.asarray(ex.model.fk_tip(q)[0], dtype=float)
        along = float(moved @ direction)
        if not _in_range(ex.model, nxt) or abs(along - step) > REL * step or \
                float(np.linalg.norm(moved - along * direction)) > REL * step:
            break
        q, went = nxt, went + step
    return q, went


def _run_in(ex, B, know, q, u):
    """The run a tap at speed v needs before its contact (m) at pose q along u, as the straight taps take it."""
    a = GIVE * float(B.a_brake(q, u))
    t_run = float(know.get("t_settle")[0] or 0.0) + QUANT * QUANT * ex.dt
    return lambda v: v ** 2 / (laws.TWO * a) + v * t_run


def _z_down(ex, B, q, u, height):
    """The largest of the body's model of the stop slope on the way down to the support (as the straight taps take
    it), or None."""
    from rcpx import bounds_selftaps as taps, bounds_strike
    down = [np.asarray(q, dtype=float)]
    for _ in range(int(round(1 / REL))):
        down.append(taps._step(ex, down[-1], u, height * REL))
    slopes = [bounds_strike.normal_z_of(B, pose, u) for pose in down]
    return None if any(z is None for z in slopes) else max(slopes)


def start(ex, B, know, q, u):
    """(the raised start pose, how far it rose m, the top speed m/s it is raised for): see the module text."""
    from rcpx import bounds_selftaps as taps
    from rcpx.bounds_selftaps_parts import _top
    got = _support(B)
    if got is None:
        return q, 0.0, None
    normal, offset = got
    height = taps._low(ex, B, q, normal, offset)
    top, _, _ = _top(ex, B, q, u, ex.cfg.get("selftest_taps") == "free", _z_down(ex, B, q, u, height))
    if not top or not math.isfinite(top) or not height > 0:
        return q, 0.0, top
    need = (1 + REL) * _run_in(ex, B, know, q, u)(top) + height * REL - height
    if not need > 0:
        return q, 0.0, top
    raised, went = _keeps_line(ex, q, normal, need)
    return raised, went, top


def across(ex, B, q, normal):
    """The second pose (see the module text) or None."""
    p = np.asarray(ex.model.fk_tip(q)[0], dtype=float)
    flat = p - float(p @ normal) * normal
    size = float(np.linalg.norm(flat))
    if not size > 0:
        return None
    side = np.cross(normal, flat / size)
    best = None
    for sign in (1.0, -1.0):
        moved, went = _keeps_line(ex, q, sign * side, GIVE * size)
        turned = float(np.max(np.abs(moved - np.asarray(q, dtype=float))))
        if turned > REL and (best is None or went > best[1]):
            best = (moved, went, sign * side)
    return best


def _go(ex, know, q_from, direction, dist, q_to, v_top, a):
    """The hand from q_from dist along direction (a line that ramps and brakes), then held at q_to to rest. A
    guard of the transit kind is armed on the way: (the last reading, the stop word or None)."""
    from rcpx import bounds_selftaps as taps
    from rcpx.control_base import Guard
    g = Guard(ex, spec=ex._guard_spec("transit", v=v_top, a=a))
    _, s, word = taps._line(ex, np.asarray(q_from, dtype=float).copy(), direction, dist, v_top, a, watch=g.check)
    from rcpx import bounds
    B = bounds.of(ex)
    wait = max(QUANT, int(math.ceil(float(B.limit_s(float(B.t_settle()))) / ex.dt)))
    s, _ = taps._rest(ex, know, np.asarray(q_to, dtype=float), wait)
    return s, word


def run(ex, know, q0, at, log):
    """The taps of the calibration (see the module text). Stores the entry "taps"; returns it."""
    from rcpx import bounds, bounds_selftaps as taps
    B = bounds.of(ex)
    t0 = ex.sim_time()
    got = _support(B)
    if got is None:
        return taps.taps(ex, know, q0, at, log)
    normal, offset = got
    u = -normal
    q0 = np.asarray(q0, dtype=float).copy()
    a_way = REL * GIVE * float(B.a_brake(q0, u))
    v_way = lambda q: GIVE * float(B.v_free(q, normal))
    ladders, starts = [], [("where the hand stands", q0, None)]
    side = across(ex, B, q0, normal)
    if side is not None:
        starts.append((f"moved {side[1] * MM:.0f} mm across the line from the base", side[0], side))
    for name, q_at, how in starts:
        if how is not None:                                   # to the second pose, along the support plane
            s, word = _go(ex, know, q0, how[2], how[1], q_at, v_way(q0), a_way)
            if word:
                log(f"taps: the way to the second pose ended on {word}: no ladder there")
                _go(ex, know, s.q, -how[2], how[1], q0, v_way(q0), a_way)
                break
        raised, rose, top = start(ex, B, know, q_at, u)
        if rose > 0:
            s, word = _go(ex, know, q_at, normal, rose, raised, v_way(q_at), a_way)
            if word:
                raised, rose = q_at, 0.0
        entry = dict(taps.taps(ex, know, raised, at, log), start={"pose": name, "raised_mm": rose * MM,
                                                                     "top_mm_s": None if top is None else top * MM})
        ladders.append(entry)
        if rose > 0:
            _go(ex, know, raised, u, rose, q_at, v_way(q_at), a_way)
            if not entry.get("used"):     # the raised start met no support (the column it came down was another's):
                ladders.append(dict(taps.taps(ex, know, q_at, at, log),        # the ladder from where the hand stood
                                    start={"pose": name + ", again from where it stood", "raised_mm": 0.0,
                                           "top_mm_s": None if top is None else top * MM}))
        if how is not None:
            _go(ex, know, q_at, -how[2], how[1], q0, v_way(q_at), a_way)
    rows = [dict(r, ladder=i) for i, e in enumerate(ladders) for r in e.get("rows", [])]
    fit = stops.fit(know.stops())
    out = dict(ladders[0], rows=rows, used=sum(int(e.get("used") or 0) for e in ladders),
               ladders=[{k: e.get(k) for k in ("said", "used", "top", "start", "limit")} for e in ladders],
               took_s=ex.sim_time() - t0)
    out["top"] = dict(ladders[0].get("top") or {}, fastest_tap_mm_s=max(
        [float((e.get("top") or {}).get("fastest_tap_mm_s") or 0.0) for e in ladders]))
    out["said"] = "; ".join(f"from {e['start']['pose']} (raised {e['start']['raised_mm']:.0f} mm): {e.get('said')}"
                            for e in ladders)
    if fit.get("n"):
        out["said"] += (f". In all {fit['n']} stops at {fit['poses']} poses between {fit['v0'] * MM:.3g} and "
                        f"{fit['v1'] * MM:.3g} mm/s; {ex.sim_time() - t0:.1f} s of the robot's own time")
    know.set("taps", out, "self_test", at)
    log("taps: " + out["said"])
    return out
