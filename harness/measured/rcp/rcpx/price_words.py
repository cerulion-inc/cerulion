# SPDX-License-Identifier: AGPL-3.0-only
"""What a call costs on THIS arm, in control periods, made at session start from the body's own record. Every
price is digits followed by "control period(s)".

    prices(ex)          -> {"dt_s", "line", "touch", "jaws", "self_check", "survey"} (each part may be
                           None: it could not be planned here), made once per executor and world epoch
    tool_price(name, entry, P) -> the price words of the opening's line of one tool
    limit_words(P)      -> what a stated limit costs here, in figures (the cards of gentle and max_n)

Nothing moves and no world time passes: every figure is the robot's own planner run on a way it does not take
(bounds.plan_time, the same count a budget refusal reads) from where the hand stands. A figure is a pair (lo, hi):
the plan's periods and what the end of the move adds, the end being the first or the last of the two ends the
body's record gives for an arm closing on its standing command (its own lag closed to its still band at the plan's
top speed, B.lag over that speed; its settle time, B.t_settle). After a paced move of this session the end is that
move's own time constant (control_move_pace.allowance), and lo equals hi.

    line     a straight tip line from where the hand stands, PRICE_MM long (a unit the price is said in, no bound),
             along the first of WAYS the arm can run whole, with nothing stated, under gentle (the hand's own
             weight, B.f_hand) and under max_n at DOUBLE that weight (a named ratio of rcpx/bounds_ratios.py). The
             limited rows are planned with NOTHING known free (the design rulings: no look, no passage, no site), as the
             words say; where a look, the arm's own passage or the site shows a stretch free the limit holds only
             the rest (the result's limit.cost says how much).
    touch    a touch with no number: the force it ends at (B.f_hand, the design rulings) and the speed solved for it (kept for
             the record; its measured way in does not follow one law the opening can state at session start, so
             the opening says the force and what one second is here, and the touch's result says its count).
    jaws     the jaws' whole stroke at their own speed (control_hold_jaws.numbers: speed, spans per second): the
             most a close takes before it squeezes.
A turn (orient_tool) and the calls whose count rests on their own arguments or on the world (press, ease, watch,
hold, zero_check ...) are said with what one second is here; their result says their own count.
    self_check  its own planned legs (rcpx/r2/selfcheck_motion.planned_legs, on a scratch plan list), out and
             back, and at most every settle window it waits in (one after each leg, one before and after all).
    survey   its own view plan (rcpx/r2/survey_plan.plan), each leg planned as the look move it is, its end, and
             the ease after it (one control period at least, at most its settle window).

Nothing names a robot, a task or a benchmark.
"""
import contextlib
import math
import weakref

PRICE_MM = 100.0          # the length a price is said per (a unit of the statement, no bound)
_CACHE = weakref.WeakKeyDictionary()   # executor -> (epoch, prices): a later session's executor is its own key


def _B(ex):
    from rcpx import surface_session
    return surface_session.bounds_of(ex)


def _core(ex):
    from rcpx import surface_session
    return surface_session.core_executor(ex)


@contextlib.contextmanager
def _nothing_known_free():
    """While a limited row is priced: no space is known free (no look, no passage, no site), as its words say. The
    plan reads known space through bounds_free.known_free alone; it is put back on the way out."""
    import numpy as np
    from rcpx import bounds_free
    real = bounds_free.known_free

    def none(ex, points_m, radii_m):
        n = len(np.asarray(points_m, dtype=float).reshape(-1, 3))
        return np.zeros(n, dtype=bool), [None] * n, ["priced with nothing known free"] * n

    bounds_free.known_free = none
    try:
        yield
    finally:
        bounds_free.known_free = real


def _ends(ex, path, v):
    """(the first, the last) of the two ends the body's record gives after a path whose top speed is v (m/s), in
    control periods: the arm's own lag at v closed to its still band at the path's end, and its settle time."""
    B, dt = ex.bounds, float(ex.dt)
    ends = [int(math.ceil(float(B.t_settle()) / dt))] if float(B.t_settle()) > 0 else []
    lag, band = float(B.lag(v) or 0.0), float(ex._band_at(path[-1]))
    if v > 0 and lag > band > 0:
        ends.append(int(math.ceil(lag / v * math.log(lag / band) / dt)))
    return (min(ends), max(ends)) if ends else (0, 0)


def _count(ex, q, path, intent, kind="transit"):
    """(lo, hi) control periods of a path: its plan and its end (see the module docstring), or None."""
    import numpy as np
    from rcpx import control_move_pace
    plan = ex.bounds.plan_time(q, path, kind, intent=intent or None)
    if getattr(plan, "refused", None) is not None:
        return None
    n = int(plan.periods)
    tail = int(control_move_pace.allowance(ex, q, path))
    if tail or not len(path):
        return (n + tail, n + tail)
    tips = [np.asarray(ex.model.fk_tip(np.asarray(p, dtype=float))[0], dtype=float) for p in [q] + list(path)]
    span = sum(float(np.linalg.norm(b - a)) for a, b in zip(tips, tips[1:]))
    v = span / max(n, 1) / float(ex.dt) if n else 0.0          # the plan's mean tip speed
    lo, hi = _ends(ex, path, v)
    return (n + lo, n + hi)


WAYS = (("up", (0.0, 0.0, 1.0)), ("forward", (1.0, 0.0, 0.0)), ("back", (-1.0, 0.0, 0.0)), ("left", (0.0, 1.0, 0.0)),
        ("right", (0.0, -1.0, 0.0)), ("down", (0.0, 0.0, -1.0)))   # base frame; the first the arm can run whole
# (down last: a way toward the surface the robot stands on meets that surface's band, where a contact can begin)


def _line(ex):
    """The price of a PRICE_MM line from where the hand stands, along the first way of WAYS the arm can run whole
    (else the longest), under nothing stated, gentle and max_n (these with nothing known free); or None."""
    import numpy as np
    from rcpx.bounds_ratios import DOUBLE
    q = np.asarray(ex._sense().q, dtype=float)
    p0 = ex.model.fk_tip(q)[0]
    best = None
    for word, d in WAYS:
        path, _, _, _ = ex._line(q, np.array(d), PRICE_MM / 1000.0)
        if not path:
            continue
        mm = float(np.linalg.norm(ex.model.fk_tip(path[-1])[0] - p0)) * 1000.0
        if best is None or mm > best[2]:
            best = (word, path, mm)
        if mm >= PRICE_MM * (1.0 - 1e-3):
            break
    if best is None or best[2] <= 0:
        return None
    word, path, mm = best
    hand = float(ex.bounds.f_hand())
    bare = _count(ex, q, path, {})
    stated = DOUBLE * hand
    with _nothing_known_free():
        gentle = _count(ex, q, path, {"gentle": True})
        max_n = _count(ex, q, path, {"max_n": stated})
    if bare is None:
        return None
    return {"way": word, "mm": round(mm, 1), "periods": bare,
            "gentle": None if gentle is None else {"n": round(hand, 1), "periods": gentle},
            "max_n": None if max_n is None else {"n": round(stated, 1), "periods": max_n}}


def _touch(ex):
    """A touch with no number over PRICE_MM where a contact can begin: its way in at the speed solved for the hand's
    own weight (the touch's own default, the design rulings), up or down from where the hand stands, the slower; or None."""
    import numpy as np
    B, q = ex.bounds, np.asarray(ex._sense().q, dtype=float)
    hand = float(B.f_hand())
    speeds = []
    for d in ((0.0, 0.0, -1.0), (0.0, 0.0, 1.0)):
        v = ex._arrival(B, q, np.array(d), {"ends_n": hand, "ends_default": True}).get("v")
        if v and float(v) > 0.0:
            speeds.append(float(v))
    if not speeds:
        return None
    v = min(speeds)
    return {"n": round(hand, 1), "mm_s": round(v * 1000.0, 1),
            "periods": int(math.ceil(PRICE_MM / 1000.0 / v / float(ex.dt)))}


def _jaws(ex):
    """The jaws' whole stroke at their own speed, or None (no jaws, no speed)."""
    from rcpx import control_hold_jaws
    N = control_hold_jaws.numbers(ex)
    speed = float((N or {}).get("speed") or 0.0)
    return {"periods": int(math.ceil(1.0 / (speed * float(ex.dt))))} if speed > 0 else None


def _self_check(ex):
    """self_check's own planned legs from where the arm stands: each wiggle goes out and comes back; then at most
    every settle window it holds in (after each leg out and back, before the first and after the last)."""
    from rcpx.r2 import selfcheck_limits as limits
    from rcpx.r2 import selfcheck_motion as motion
    s = ex._sense()
    idx = [j for j in range(ex.model.n) if ex._sc_revolute(j)]
    amps, _ = limits.amplitudes(ex, s.q, idx, float(ex.cfg["floor_z_m"]), None)
    had = getattr(ex, "_sc_plans", None)      # the list a run of self_check keeps its plans in: a scratch one here
    ex._sc_plans = []
    try:
        legs = motion.planned_legs(ex, idx, amps, None)
    finally:
        if had is None:
            del ex._sc_plans
        else:
            ex._sc_plans = had
    if not legs or any(getattr(p, "refused", None) for p in legs):
        return None
    moving = int(2 * sum(len(getattr(p, "path", None) or []) for p in legs))
    window = int(math.ceil(float(ex.bounds.wait("settle", motion.picture_need(ex, s.q)).limit_s) / float(ex.dt)))
    return {"periods": (moving, moving + (2 * len(legs) + 2) * window), "legs": len(legs), "joints": len(idx)}


def _survey(ex):
    """survey's own view plan: each leg planned as the look move it is, its end, and the ease after it."""
    import numpy as np
    from rcpx import control_base
    from rcpx.r2 import survey_plan
    plan = survey_plan.plan(ex)
    if "refused" in plan:
        return None
    window = max(int(control_base._ratio("QUANT")), int(math.ceil(float(ex.bounds.t_settle()) / float(ex.dt))))
    lo = hi = 0
    q = np.asarray(ex._sense().q, dtype=float)
    for v in plan["views"]:
        target = np.asarray(v["q"], dtype=float)
        got = _count(ex, q, ex._plan_joint(q, target, timed=False), {}, kind="look")
        if got is None:
            return None
        lo, hi = lo + got[0] + 1, hi + got[1] + window
        q = target
    return {"periods": (lo, hi), "views": len(plan["views"])}


PARTS = (("line", _line), ("touch", _touch), ("jaws", _jaws), ("self_check", _self_check), ("survey", _survey))


def prices(ex):
    """The figures of the module docstring for this session, made once per executor and world epoch."""
    core = _core(ex)
    if core is None or _B(core) is None:
        return {}
    epoch = getattr(core, "epoch", 0)
    try:
        got = _CACHE.get(core)
    except TypeError:                      # an executor that takes no weak reference: priced each time
        got = None
    if got is not None and got[0] == epoch:
        return got[1]
    out = {}
    for name, fn in PARTS:
        try:
            out[name] = fn(core)
        except Exception as exc:          # a part that cannot be planned here is left out, and the journal says why
            out[name] = None
            try:
                core._journal("price_problem", part=name, error=f"{type(exc).__name__}: {exc}")
            except Exception:
                pass
    try:
        out["dt_s"] = float(core.bounds.dt())
    except Exception:
        out["dt_s"] = None
    try:
        _CACHE[core] = (epoch, out)
    except TypeError:
        pass
    return out


def figure(pair, short=False):
    """"N to M" for a (lo, hi) pair of control periods ("N" when they are one). short: one figure, the middle, where
    the two lie within the accepted relative error of a measured quantity (REL of rcpx/bounds_ratios.py) of each
    other (a long limited move, whose end is a small share of it)."""
    from rcpx.bounds_ratios import REL
    lo, hi = (pair, pair) if isinstance(pair, int) else (int(pair[0]), int(pair[1]))
    if lo == hi or short and hi - lo <= REL * lo:
        return f"{int(round((lo + hi) / 2.0))}"
    return f"{lo} to {hi}"


MOVES = ("move_tip", "move_joints", "retreat", "park", "home", "standard_look", "approach")
WHOLE = ("put", "pick", "place", "carry", "work_part")
JAWS = ("gripper", "grasp", "release", "pinch")
FREE = ("look", "crop", "mark", "state", "device_state", "remember", "verify", "triangulate", "picture",
        "reach_check")


def tool_price(name, entry, P):
    """The price words of one tool's line in the opening: 0 control periods for a call that moves nothing; for a
    move the free line's figure; for the jaws, self_check and survey their own figures; for a
    whole step the line's figure for each leg and the jaws' own time; else what one second is here (its result
    says its own count). With no price made (P empty) a moving tool's words carry no figure."""
    from rcpx import surface_session
    P = P or {}
    text = str((entry or {}).get("description", ""))
    if surface_session.is_free(name, entry) or " Free" in text[-400:] or name in FREE:
        return "0 control periods"
    line, jaws = P.get("line"), P.get("jaws")
    rate = (f"about {figure(line['periods'])} control periods for {line['mm']:g} mm of free way here" if line else
            "its plan's control periods, said in its result")
    if name == "self_check":
        got = P.get("self_check")
        return (f"about {figure(got['periods'])} control periods here ({got['legs']} legs, each out and back)" if got
                else "its control periods are its planned wiggles, said in its result")
    if name == "survey":
        got = P.get("survey")
        return (f"about {figure(got['periods'])} control periods here ({got['views']} views)" if got
                else "its control periods are its planned views, said in its result")
    dt = P.get("dt_s")
    second = f"1 s is {int(round(1.0 / dt))} control periods here" if dt else ""
    if name == "touch" and second:
        hand = (P.get("touch") or {}).get("n")
        return ("as a move through space known free, and where a contact can begin at the speed solved for the "
                + (f"{hand:g} N " if hand else "force ") + f"it ends at: its result says its control periods ({second})")
    if name in JAWS and jaws:
        return (f"a close at most about {jaws['periods']} control periods (the jaws' whole stroke at their own "
                "speed); an opening: its result says")
    if name in MOVES:
        return rate
    if name in WHOLE:
        return rate + " for each leg" + (f", and {jaws['periods']} for the jaws' whole stroke" if jaws else "")
    return f"its result says its control periods ({second})" if second else "its control periods are said in its result"


def priced(ex, name):
    """" (about N to M control periods here)" for a recommended self_check or survey in a hint (the design rulings: no hint
    recommends the body check or the survey without its price), or "" when no price is made."""
    try:
        got = (prices(ex) or {}).get(name)
    except Exception:
        got = None
    return f" (about {figure(got['periods'])} control periods here)" if got else ""


def limit_words(P):
    """What a stated limit costs on this arm, in figures (the cards of gentle and max_n), or "" with no figure. It
    starts with "about", as every rendered fact does (rcpx/surface_session.py render)."""
    line = (P or {}).get("line")
    if not line or not (line.get("gentle") or line.get("max_n")):
        return ""
    parts = []
    if line.get("gentle"):
        parts.append(f"{figure(line['gentle']['periods'], True)} (gentle, {line['gentle']['n']:g} N)")
    if line.get("max_n"):
        parts.append(f"{figure(line['max_n']['periods'], True)} (max_n {line['max_n']['n']:g} N)")
    return ("about " + " or ".join(parts) + f" control periods for {line['mm']:g} mm {line['way']} from the first "
            f"pose, nothing known free; {figure(line['periods'])} with none")


__all__ = ["JAWS", "MOVES", "PRICE_MM", "WHOLE", "figure", "limit_words", "priced", "prices", "tool_price"]
