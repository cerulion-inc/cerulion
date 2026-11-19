# SPDX-License-Identifier: AGPL-3.0-only
"""Which stop slope a speed is SOLVED with. Pure but for the bounds' own answers.

A speed is solved from a force in two cases that are not the same promise:
- a stated LIMIT (max_n, gentle, a sheet, the site, a rating, people, a give, the limit of a thing held): never
  over it, the strike of the way in included. It is solved with the slope a figure is stated with
  (rcpx.bounds_stops.slope: the body's model with its margin, raised where stops read over it), so that what
  the motion says it can strike and what it says it keeps are one number. That is the price of a limit.
- a force a motion ENDS at (a touch's force_n or the hand's own weight, a close's squeeze, a landing's force, a
  press's force_n, a push law's ceiling): no limit and no promise ("the way in runs at the speed solved
  for T"). It is solved with what this body's stops showed (rcpx.bounds_stops.estimate, also under the model),
  as the first pass did; the result states the strike the arrival could make by the first slope.
The hand-overs of rcpx/bounds.py see every stated number of a call (stated_limit answers it, plan, plan_time,
plan_legs and arrival are handed it) and set B._solving for the time of the plan or the arrival: "limit" or
"ends". B.slope reads it; outside a plan or an arrival (a push law that asks B.slope itself) it is None and the
measured slope answers. A link's point is always solved with the first slope (no stop was made with a link).

- noted(B, got): stated_limit's answer, remembered for this call: whether the number it gave is a LIMIT.
- keeps(B, intents, held, expects): "limit" or "ends" for a plan with those words, that hold and those expected
  contacts.
- planned(B, intents, held, expect, call): a plan run under keeps().
- run(B, mode, call): call() with B._solving = mode; what stood is put back (plans nest).
- new_call(B): a call begins; the cold session's line for the first result of any tool.
- above(B, got): an arrival solved faster than the fastest stop that bears the fit (V1) runs at the speed solved
: nothing holds it to V1. Its strike there is stated by the energy bound, an upper bound by
  physics (rcpx/bounds_strike.py rule 2), and the answer says so: it is not shown kept by a stop. The one case
  kept for a hold (no bound by physics can be stated) does not arise: a body whose description states
  no contact model has no stop that bears a fit, so no V1, and its arrival solves no speed from a slope.
- rest(B): the contact level of the still arm by this robot's own self-knowledge, never raised by this session's
  own false stops: the force rigid() reads the servo's give at when no force is named (bounds.Bounds.rigid).
- at_link(text): the slope text of a link's point when no stop was made with a link: never "fitted".
"""

from rcpx import bounds_limit as limit
from rcpx import bounds_stops as stops


def noted(B, got):
    """Remember, for this call, whether the number stated_limit answered is a LIMIT (its "hold" is at or under
    it) or a force a motion ends at. Inside a plan that was taken for "ends", a LIMIT turns it to "limit": the
    plan gathered it itself (an expected contact it asked for)."""
    number, hold = got.get("limit_n"), limit.hold_of(got)
    beside = got.get("floor")
    is_limit = hold is not None and (number is None or hold["limit_n"] <= number or (beside is not None and beside[0] is not None))
    if number is not None:
        book = B.__dict__.setdefault("_stated_numbers", {})
        book[float(number)] = bool(is_limit) or book.get(float(number), False)
    if is_limit and B._solving == "ends":
        B._solving = "limit"
    return got


def keeps(B, intents=(), held=None, expects=()):
    """"limit" when a LIMIT is stated on a plan: in the call's own words (any of intents), for the thing held, on
    an expected contact (a bound whose source is a sheet, the site, a rating or a give; a bound stated_limit
    answered as a limit in this call; an ending force with a limit beside it). Else "ends"."""
    for intent in list(intents) or [None]:      # the site, a rating and people: true are limits with no word at all
        if limit.hold_of(limit.gather_limit(B, None, intent or None, False)) is not None:
            return "limit"
    if limit.carried(held) is not None:
        return "limit"
    book = B.__dict__.get("_stated_numbers") or {}
    for want in expects:
        if not isinstance(want, dict) or want.get("bound_n") is None:
            continue
        beside = want.get("floor")
        if beside is not None:
            if beside[0] is not None:
                return "limit"
            continue
        if want.get("bound_source") in limit.NOT_SOUGHT or book.get(float(want["bound_n"])):
            return "limit"
    return "ends"


def planned(B, intents, held, expect, call):
    """call() (a plan) with B._solving set by what the plan's words, hold and expected contacts keep (keeps)."""
    return run(B, keeps(B, intents, held, expect if isinstance(expect, list) else [expect]), call)


def run(B, mode, call):
    """call() with B._solving = mode; what stood before is put back."""
    stood, B._solving = B._solving, mode
    try:
        return call()
    finally:
        B._solving = stood


def new_call(B):
    """A call begins: the numbers of the call before are forgotten. At the first call of a session (no motion) a
    cold session's line goes to the sentences the call's result takes (bounds_notes), whatever tool makes it
    (rcpx.bounds_selfstart.notice)."""
    B.__dict__.pop("_stated_numbers", None)
    if getattr(B, "_cold_asked", False):
        return
    B._cold_asked = True
    import importlib
    line = importlib.import_module("rcpx.bounds_selfstart").notice(B.ex, B)
    if line:
        B.ex.__dict__.setdefault("_guard_notes", []).append(line)


def above(B, got):
    """B.arrival's answer, said where it runs above the fastest stop that bears the fit (see the module text): the
    speed stays as solved; kept is false there (no stop shows it), shown_by "bound" (the energy bound states the
    figure), and "above_measured" carries both speeds."""
    v = got.get("v_m_s")
    link = getattr(B, "_point_model", None) is not None and getattr(B, "_point_body", None) is not None
    from rcpx.bounds_strike import arrive_most
    v1 = arrive_most(stops.of_kind(stops.fit(B._stops()), link))
    if v1 is None or v is None or not float(v) > v1:
        return got
    out = dict(got, kept=False, shown_by="bound", above_measured={"v_m_s": float(v), "v1_m_s": float(v1)})
    out["text"] = (str(got.get("text") or "") + f"; it runs above {v1 * stops.MM:.3g} mm/s, the fastest stop this arm "
                   "resolved: its strike there is stated by the energy bound the motion carries (an upper bound by "
                   "physics), not shown by a stop")
    return out


def rest(B):
    """The still arm's contact level (N) by the stored self-knowledge alone (see the module text): the level at
    rest with this session's false-stop tables set aside for the reading. A self-knowledge that keeps no such
    tables apart (the body's model alone, a test's stand-in) answers its level as it stands."""
    know = getattr(B.know, "_base", B.know)
    raised = getattr(know, "session", None)
    if not isinstance(raised, dict) or not any(raised.get(k) for k in ("f_hidden", "sigma_f")):
        return B.f_thr(0, 0)
    know.session = {k: v for k, v in raised.items() if k not in ("f_hidden", "sigma_f")}
    try:
        return B.f_thr(0, 0)
    finally:
        know.session = raised


def at_link(text):
    """The slope text of a link's point when no stop was made with a link: the body's model there, raised as the
    tool's stops read over it, and never "fitted" (no stop shows a link's figure kept)."""
    rest = str(text)[len("fitted: "):] if str(text).startswith("fitted: ") else str(text)
    return f"the body's model at a link: no stop was made with a link (the tool's stops: {rest})"
