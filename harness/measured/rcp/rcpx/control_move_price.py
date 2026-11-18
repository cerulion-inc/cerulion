# SPDX-License-Identifier: AGPL-3.0-only
"""What a budget refusal says of the limit that made the plan long.

A motion that needs more control periods than are left is refused before it starts. When a LIMIT somebody stated
(max_n or gentle on the call, a sheet, the site, a rating) is what makes the plan long, the refusal names that
word and its price, and offers what fits:

    refused = {"check": "budget",
               "why": "this move plans N control periods and M are left. Your word max_n (5 N) is K of them:
                       without it the move plans J.",
               "hint": text,
               "price": {"periods": N, "left": M, "word": "max_n" | "gentle" | "the sheet of X" | "the site" | ...,
                         "limit_n": L, "without_periods": J,
                         "fits_n": the smallest limit whose plan fits the periods left, or None},
               "ready": [the same call with max_n = fits_n (it keeps a limit: first),
                         the same call without the word (only where the word is the operator's own)],
               each with "periods"}

Nothing is typed: the counts are the plan's own (the same planner, the same path, another limit), the search for
fits_n doubles the stated limit until the plan fits and halves the last step until the limit is known to the
accepted relative error of a measured quantity (REL). With no look with depth of the scene as it stands held the
limit holds every leg: the refusal says what a look would make of it. No robot is named here.

    price_of(limit, intent, count)       -> what _budget needs, or None when no limit made the plan long
    refusal_of(ex, periods, left, hint, price) -> the refusal, or None (the plain budget refusal stands)
"""

import math
import sys

from rcpx.bounds_ratios import DOUBLE, REL
from rcpx.context import refusal

OWN = ("max_n", "gentle")          # the words of a call that are the operator's own limit


def price_of(limit, intent, count):
    """{"limit": the plan's limit record (limit_n, source, cost), "intent": the call's intent, "count": a function
    of an intent that returns the control periods the same motion plans under it} for _budget; None when no
    limit is stated."""
    if not isinstance(limit, dict) or limit.get("limit_n") is None:
        return None
    return {"limit": limit, "intent": dict(intent or {}), "count": count}


def of_move(ex, plan, intent, q0, way, kind, target, held, away_from):
    """price_of for a move the plan door timed: the same way under another intent, counted by the bounds' own
    plan_time (no path is laid out)."""
    def count(it):
        clean = {k: v for k, v in it.items() if k not in ("_limit_n", "_readings", "_share")}
        return int(ex.bounds.plan_time(q0, way, kind, target=target, intent=clean, held=held, away_from=away_from).periods)
    return price_of(getattr(plan, "limit", None), intent, count)


def _word(limit, intent):
    """(the word that states the limit as the operator reads it, whether it is the operator's own)."""
    source = str(limit.get("source") or "")
    if source == "intent":
        for word in OWN:
            if intent.get(word):
                return word, True
        return "your stated limit", False
    names = {"sheet": "the sheet of the thing this call names", "site": "the site", "rating": "the body's contact rating",
             "contact": "a give of this session"}
    return names.get(source, source or "a stated limit"), False


def _without(intent):
    return {k: v for k, v in intent.items() if k not in OWN}


def _fits(count, intent, limit_n, left, bare):
    """(fits_n, its periods): the smallest limit (N) whose plan fits `left` control periods, searched upward from
    the stated one by doubling and then halved in to REL; (None, None) when no limit fits (the motion without
    any does not fit either)."""
    if bare > left:
        return None, None
    lo, hi, n_hi = float(limit_n), float(limit_n), None
    for _ in range(sys.float_info.mant_dig):          # numeric: the most doublings a float's mantissa tells apart
        hi *= DOUBLE
        n_hi = count({**_without(intent), "max_n": hi})
        if n_hi <= left:
            break
        lo = hi
    else:
        return None, None
    while hi - lo > REL * lo:
        mid = lo + (hi - lo) / DOUBLE
        n = count({**_without(intent), "max_n": mid})
        if n <= left:
            hi, n_hi = mid, n
        else:
            lo = mid
    step = REL * float(limit_n)                       # said in steps of the stated limit's own accepted error
    up = math.ceil(hi / step) * step
    return up, int(count({**_without(intent), "max_n": up})) if up != hi else int(n_hi)


def refusal_of(ex, periods, left, hint, price):
    """The budget refusal that names the limit and its price, or None when the limit is not what makes
    the plan long (the same motion with no limit does not plan fewer periods)."""
    limit, intent, count = price["limit"], price["intent"], price["count"]
    try:
        bare = int(count({**_without(intent), "_no_limit": True}))
    except Exception as exc:
        ex._journal("bounds_problem", where="budget price", error=f"{type(exc).__name__}: {exc}")
        return None
    if bare >= int(periods):
        return None
    word, own = _word(limit, intent)
    limit_n = float(limit["limit_n"])
    fits_n = fit_periods = None
    if own:
        try:
            fits_n, fit_periods = _fits(count, intent, limit_n, int(left), bare)
        except Exception as exc:
            ex._journal("bounds_problem", where="budget fits", error=f"{type(exc).__name__}: {exc}")
    name, args = getattr(ex, "_call_now", None) or (None, {})
    ready = []
    if name is not None and own:
        base = {k: v for k, v in args.items() if k not in OWN}
        if fits_n is not None:
            ready.append({"tool": name, "args": {**base, "max_n": fits_n}, "periods": fit_periods,
                          "what": f"the same call with max_n {fits_n:g} N: the smallest limit whose plan fits"})
        if bare <= int(left):
            ready.append({"tool": name, "args": base, "periods": bare, "what": f"the same call without {word}"})
    cost = limit.get("cost") or {}
    why = (f"this move plans {int(periods)} control periods and {int(left)} are left. "
           + (f"Your word {word} ({limit_n:g} N)" if own else f"The limit of {limit_n:g} N ({word})")
           + f" is {int(periods) - bare} of them: without it the move plans {bare}."
           + (f" The least max_n whose plan fits is {fits_n:g} N ({fit_periods} control periods)." if fits_n is not None
              else ""))
    if cost.get("said") and "no look" in str(cost["said"]):
        why += (" No look with depth of the scene as it stands is held, so the limit holds every leg: after a look "
                "that shows the way free it holds only where a contact can begin.")
    out = refusal("budget", why, "one of the ready calls fits the periods left" if ready else
                  (hint or "ask for a shorter move"))
    out["refused"]["price"] = {"periods": int(periods), "left": int(left), "word": word, "limit_n": limit_n,
                               "without_periods": bare, "fits_n": fits_n}
    out["refused"]["ready"] = ready
    return out
