# SPDX-License-Identifier: AGPL-3.0-only
"""What a limit the operator stated on work_part cost in control periods (result["limit"]["cost"]): the price of
the word, said where the operator reads it.

- start(ex) before a stream, stream(ex, token, legs, got) after it (rcpx/r2/fixture_follow.py stream): where the
  legs carry the operator's own word (max_n or gentle), the same legs are planned again by the bounds from where
  the stream began, once with the word and once without it (the expected contact of the way in loses the bound
  that word gave it). The periods the stream ran are shared out by the two plans: cost = ran x (1 - without /
  with). A plan the bounds refuse prices nothing (said). No control period runs for this.
- held(ex, periods, where): a force led motion (the turn) notes the periods its own speed law spent under the
  word: per period the share of the speed the word took away (1 - the speed the word left / the speed without).
- of_call(ex, used): the block {"periods", "without_periods", "where", "how", "not_priced"} for
  result["limit"]["cost"], or None when the call stated no word of its own (a sheet's limit cannot be planned away:
  it is not the operator's). not_priced names what ran under the word and is not counted: the close of the jaws,
  whose arrival is solved for each pad's share of the limit inside the core (rcpx/control_hold_close.py), and a
  stream whose plan could not be made again.
Nothing here reads truth, names a robot or a device, or holds a typed number.
"""

from rcpx.r2.fixture_probe import bounds_of

OWN = ("max_n", "gentle")       # the words of a call that are the operator's own limit


def _word(legs):
    got = (legs[0].get("intent") if legs else None) or {}
    return {k: v for k, v in got.items() if k in OWN and v}


def start(ex):
    """(where the stream begins: the command, the executor's period count) for stream()."""
    q = getattr(ex, "_q_des", None)
    counts = getattr(ex, "periods", None)
    return (None if q is None else q.copy(), None if not isinstance(counts, dict) else sum(counts.values()))


def _periods(B, q0, legs, held):
    plan = B.plan_legs(q0, legs, held=held, looks=None, intent=None)
    if getattr(plan, "refused", None):
        return None
    return len(getattr(plan, "path", None) or [])


def stream(ex, token, legs, got):
    """Note what the word on these legs cost the stream that ran them (module text)."""
    q0, n0 = token
    if not _word(legs) or q0 is None or n0 is None or not isinstance(getattr(ex, "periods", None), dict):
        return None
    ran = sum(ex.periods.values()) - n0
    if ran <= 0 or not isinstance(got, dict) or got.get("stopped") == "refused":
        return None
    bare = []
    for leg in legs:
        leg = dict(leg, intent={k: v for k, v in (leg.get("intent") or {}).items() if k not in OWN})
        if isinstance(leg.get("expect"), dict) and leg["expect"].get("bound_source") == "intent":
            leg["expect"] = dict(leg["expect"], bound_n=None, bound_source=None)
        bare.append(leg)
    B = bounds_of(ex)
    held = ex._held_for(None) if hasattr(ex, "_held_for") else None
    try:
        with_word, without = _periods(B, q0, legs, held), _periods(B, q0, bare, held)
    except Exception as exc:      # the plan could not be made again: nothing is priced, and that is said
        with_word, without = None, None
        ex._journal("fixture", stage="cost", error=f"{type(exc).__name__}: {exc}")
    rec = {"where": _phase(ex), "ran": int(ran), "with_plan": with_word, "without_plan": without}
    if with_word and without is not None:
        rec["periods"] = float(ran) * max(0.0, 1.0 - float(without) / float(with_word))
    ex._fx.setdefault("cost", []).append(rec)
    return rec


def held(ex, periods, where):
    """Note the periods a force led motion's own speed law spent under the word (module text)."""
    if periods and periods > 0:
        ex._fx.setdefault("cost", []).append({"where": where or _phase(ex), "periods": float(periods), "ran": None})


def _phase(ex):
    phases = (getattr(ex, "_fx", None) or {}).get("phases") or []
    return phases[-1].get("phase") if phases else None


def of_call(ex, used, intent_words):
    """The cost block of the call's limit (module text), or None."""
    if not intent_words:
        return None
    rows = (getattr(ex, "_fx", None) or {}).get("cost") or []
    priced = [r for r in rows if r.get("periods") is not None]
    cost = int(round(sum(r["periods"] for r in priced)))
    where = sorted({str(r["where"]) for r in priced if r["periods"] >= 1})
    unpriced = sorted({str(r["where"]) for r in rows if r.get("periods") is None})
    out = {"periods": cost, "without_periods": max(0, int(used) - cost), "where": where,
           "how": ("each stream's own periods shared out by the same legs planned with your word and without it; a "
                   "turn by the share of its speed your word took, period by period")}
    if unpriced:
        out["not_priced"] = unpriced
    return out
