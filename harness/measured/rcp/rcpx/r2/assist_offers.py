# SPDX-License-Identifier: AGPL-3.0-only
"""The offer engine of the operator gauge, with its untried rule. Pure: plain data
in, plain data out; no session, no executor, no picture, no clock.

    choose_offer(snap, new_events, have, listed, history, prefs, ctx) -> the one offer for this result, or None
    call_text(tool, args, listed, n, needs=None)                      -> a call written so this client can emit it
    step_down(snap, history, prefs, ctx)                              -> D1 or D2, once each, or None
    untried(stage, snap, have, ctx)                                   -> at most 3 ready calls, a seeing rung first
    unsafe(args)                                                      -> True for an argument no written call may hold

What the caller owns: which events are new on this call, placing the offer in the result (first key, or after a
refusal's own keys), no offer on a list, a read, a claim answer or the result of a taken ready call, the pause
(ctx.paused), and every fact in ctx (target_ok per located entry, has_list meaning a FRESH list, in_reach from a dry
reach check). ctx may also carry three optional keys that only step_down reads: picks and acts, each
[{"call", "confirmed"}] newest last, and listed (the names this client was last sent).

Rules held here (5.3): silence without a new event or without an open event of the offer's skill; quiet and pause;
seeing before acting before checking; no offer within GAP_CALLS of the last unless the skill holds 2 or more open
events, and never two on one call; an id at most twice, the second after AGAIN_CALLS, a third only with 2 or more
open events; an offer names only a tool the session can run, that this client can be told how to call, with calls
left for it. Safety (4.7 rules 1 and 2, stricter where in doubt): no written call holds firm, further, a force or a
number in newtons; an act call names only the newest located entry the caller passed (target_ok None, fresh, in
reach) whose label is not a cue or forbidden part; feeling along a mark is never offered for a mark on a cue or a
forbidden part, nor for a mark the manual's colour contradicts; no contact call (a push, a touch, the routine,
feeling along a mark) is written while the manual forbids a part that has no fresh located entry, because the
guard and the push itself would refuse it; the stop condition of a routine is filled in only as the manual's cue,
else the call says that it needs one; the part a call names is only ever the operator's own label, and only when
that label is a part name of the manual.
"""

import json
import re
from rcpx.r2 import assist_words as W

_NEWTON = re.compile(r"(?<![A-Za-z0-9_.])\d+(?:\.\d+)?\s?N(?![A-Za-z0-9_])")


def _dict(value):
    return value if isinstance(value, dict) else {}


def _rows(value):
    return [r for r in value if isinstance(r, dict)] if isinstance(value, (list, tuple)) else []


def _names(value):
    """A set of names from any plain collection (a set, a list, a dict's keys); a str or None gives none."""
    if value is None or isinstance(value, (str, bytes)):
        return set()
    try:
        return {s for s in value if isinstance(s, str)}
    except TypeError:
        return set()


def _int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _num(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _copy(value):
    try:
        return json.loads(json.dumps(value))
    except (TypeError, ValueError, OverflowError, RecursionError):
        return None


def unsafe(args):
    """Scan nested plain data, including cycles, without changing it."""
    todo, seen = [args], set()
    while todo:
        value = todo.pop()
        if isinstance(value, str):
            if _NEWTON.search(value):
                return True
        elif isinstance(value, (dict, list, tuple)):
            if id(value) in seen:
                continue
            seen.add(id(value))
            if isinstance(value, dict):
                for key, item in value.items():
                    if isinstance(key, str):
                        key = key.lower()
                        if (key in W.UNSAFE_ARGS or any(p in key for p in W.UNSAFE_KEY_PARTS)
                                or key.endswith(W.UNSAFE_KEY_ENDS)):
                            return True
                    todo.append(item)
            else:
                todo.extend(value)
    return False


def call_text(tool, args, listed, n, needs=None):
    """Write a detached call using a name the client has received."""
    args = {} if args is None else args
    if not isinstance(tool, str) or not tool or not isinstance(args, dict) or unsafe(args):
        return None
    args = _copy(args)
    if args is None or not (needs is None or isinstance(needs, (str, list, tuple))):
        return None
    need = [] if needs is None else [needs] if isinstance(needs, str) else list(needs)
    listed = _names(listed)
    if tool in listed:
        out = {"tool": tool, "args": args}
    elif "assist" in listed:
        if (tool == "choose" and "px" not in args and all(k in ("n", "part") for k in args)
                and (_int(args.get("n")) or isinstance(args.get("n"), str))):
            inner = {"pick": args["n"]}
            if "part" in args:
                inner["part"] = args["part"]
            out = {"tool": "assist", "args": inner}
        elif "candidates" in need:
            if "until" in need:
                args["until"] = "WORDS"
            args["candidates"] = "NUMBERS"
            out = {"tool": "assist", "args": {"run": tool, "args": args}}
        else:
            if not _int(n):
                return None
            inner = {"take": n}
            if "until" in need:
                inner["until"] = "WORDS"
            out = {"tool": "assist", "args": inner, "runs": (tool + " " + json.dumps(args))[:W.RUNS_MAX]}
    elif "use" in listed:
        out = {"tool": "use", "args": {"name": tool, "args": args}}
    else:
        return None
    if needs:
        out["needs"] = list(needs) if isinstance(needs, (list, tuple)) else needs
    return out


def _parts(ctx):
    ctx = _dict(ctx)
    parts = _dict(ctx.get("parts"))
    return tuple([s for s in parts[k] if isinstance(s, str)] if isinstance(parts.get(k), (list, tuple)) else []
                 for k in ("control", "cue", "forbidden"))


def _placed(ctx):
    """True when every part the manual forbids has a fresh located entry (the keep-off rule can be tested)."""
    ctx = _dict(ctx)
    labels = [r.get("label") for r in _rows(ctx.get("located")) if not r.get("stale")]
    return all(name in labels for name in _parts(ctx)[2])


def _target(ctx):
    """The newest located entry a written act call may name (4.7 rule 2 is the caller's target_ok; the label
    test here is a second lock: never an entry labelled as a cue or a forbidden part)."""
    ctx = _dict(ctx)
    _, cue, forbidden = _parts(ctx)
    for row in reversed(_rows(ctx.get("located"))):
        if (isinstance(row.get("id"), str) and row.get("target_ok") is None
                and not row.get("stale") and row.get("in_reach") is not False
                and row.get("label") not in cue and row.get("label") not in forbidden):
            return row
    return None


def _budget(tool, ctx):
    """True when the session has calls left for this tool (no budget: always); a routine needs ROUTINE_CALLS."""
    ctx = _dict(ctx)
    left = ctx.get("calls_left")
    return left is None or (_num(left) and left >= (W.ROUTINE_CALLS if tool in ("try_each", "seek") else 1))


def _point(ctx):
    """(look, [u, v], label) of the operator's last mark when its label is a part name of the manual, else None:
    a label that names no part is never written into a call."""
    ctx = _dict(ctx)
    p = _dict(ctx.get("last_point"))
    px, label = p.get("px"), p.get("label")
    if (isinstance(p.get("look"), str) and isinstance(px, (list, tuple)) and len(px) == 2
            and all(_num(x) for x in px) and p.get("is_part") is True
            and isinstance(label, str) and label):
        # a whole pixel is written as an integer (357, not 357.0): shorter, and what an operator sends itself
        return p["look"], [int(x) if float(x).is_integer() else x for x in px], label
    return None


def _mislabelled(snap, label):
    snap = _dict(snap)
    return any(e.get("signal") == "mislabel" and _dict(e.get("data")).get("label") == label
               for e in _rows(snap.get("events")))


def _seek(have, ctx):
    """The seek call as RCP writes it: until "found" with every control part name at level words, else {}."""
    have, ctx = _names(have), _dict(ctx)
    if "seek" not in have or not _budget("seek", ctx):
        return None
    control = _parts(ctx)[0]
    args = {"until": "found " + ", ".join(control)} if ctx.get("eyes_level") == "words" and control else {}
    return "seek", args


def _list_look(ctx):
    ctx = _dict(ctx)
    look = _dict(ctx.get("last_point")).get("look")
    if isinstance(look, str):
        return look
    return ctx.get("newest_look") if isinstance(ctx.get("newest_look"), str) else None


def _routine(have, ctx):
    """The routine call as O7 writes it: (tool, args, needs) or None. Only with the routine loaded (the text of
    O7 promises it). With a fresh list and every forbidden part placed: try_each, its stop condition filled in
    only as the manual's cue (else the call needs "until"), and with a forbidden part in the manual the operator
    lists the numbers itself (needs "candidates"). Otherwise the list call, which comes first."""
    have, ctx = _names(have), _dict(ctx)
    if "try_each" not in have or not ("push" in have or "lift" in have):
        return None        # the routine runs push or lift: without one of them it answers not_built
    if ctx.get("has_list") and _placed(ctx):
        if not _budget("try_each", ctx):
            return None
        args, need = {}, []
        _, cue, forbidden = _parts(ctx)
        if cue:
            args["until"] = "cue changes"
            if ctx.get("watch") is not None:
                args["watch"] = ctx["watch"]
        else:
            need.append("until")
        if forbidden:
            need.append("candidates")
        return "try_each", args, None if not need else need[0] if len(need) == 1 else need
    if "candidates" in have and isinstance(ctx.get("newest_look"), str) and _budget("candidates", ctx):
        return "candidates", {}, None
    return None


def _act(have, ctx, touch=False):
    """The whole step (or a touch) on the target. None while a forbidden part is not placed: the push's own
    preflight and the guard refuse forbidden_unplaced there, so the call could not run."""
    have, ctx = _names(have), _dict(ctx)
    t = _target(ctx)
    if t is None or not _placed(ctx):
        return None
    if touch:
        return "act_on", {"on": t["id"], "verb": "touch"}
    if "push" in have:
        return "push", {"target": t["id"]}
    return None            # act_on runs push: with push not loaded there is no whole step to write


def _onto(have, ctx):
    """The operator's own first move onto the target where no whole step is loaded: approach {"target": ID}. The
    caller fills along from the touched surface (rcpx/r2/assist_ladder.py untried); a move to a standoff, no
    contact and no force. None with push loaded (the whole step is written instead) or with no target."""
    have, ctx = _names(have), _dict(ctx)
    t = _target(ctx)
    if t is None or "push" in have or "approach" not in have:
        return None
    return "approach", {"target": t["id"]}


def _took(history, oid):
    """True when an offer with this id was taken (5.3 rule 8: a taken rung that then fails moves to the next)."""
    return any(h.get("id") == oid and h.get("taken") not in (None, False) for h in _rows(history))


def _row(oid, signal, event, snap, have, ctx, history=None):
    """(tool, args, needs, can) for one row of the table of 5.2 when it is available now, else None."""
    if not isinstance(oid, str) or oid not in ("O1", "O2", "O3", "O4", "O5", "O7", "W1"):
        return None
    event, snap, have, ctx = _dict(event), _dict(snap), _names(have), _dict(ctx)
    can, needs = W.OFFERS[oid]["can"], None
    if oid == "O1":
        look = _list_look(ctx)
        if look is None:
            return None
        tool, args = "candidates", {"look": look}
        can = can["words" if ctx.get("eyes_level") == "words" and any(_parts(ctx)) else "plain"]
    elif oid == "O2":
        if not (ctx.get("list_taken") or _took(history, "O1") or (ctx.get("has_list") and ctx.get("unseen"))
                or (signal in ("samepx", "rays15") and ctx.get("has_list"))):
            return None
        call = _seek(have, ctx)
        if call is None:
            return None
        tool, args = call
    elif oid in ("O3", "O4"):
        p = _point(ctx)
        if p is None or _mislabelled(snap, p[2]):
            return None
        look, px, label = p
        tool = "locate" if oid == "O3" else "choose"
        args = {"look": look, "px": px, "part": label}
        if oid == "O4":
            # a contact the robot makes on its own: never along a mark on a cue or a forbidden part, and never
            # while the keep-off rule could not be tested (a forbidden part with no fresh located entry)
            _, cue, forbidden = _parts(ctx)
            if label in cue or label in forbidden or not _placed(ctx):
                return None
            rows = _rows(ctx.get("located"))
            matches = [r for r in rows if r.get("label") == label]
            entry = matches[-1] if matches else rows[-1] if rows else {}
            uncertain = entry.get("uncertain_mm")
            data = _dict(event.get("data"))
            to, ran = data.get("to_point_mm"), data.get("ran_mm")
            depth = _num(uncertain) and ((_num(to) and 0 < to <= uncertain) or (_num(ran) and uncertain > ran))
            if ctx.get("pixel_in_candidate") is not True and not depth:
                return None
            args["depth"] = "touch"
    elif oid == "O5":
        call = None if _took(history, "O5") else _act(have, ctx)
        if call is None:
            return None
        tool, args = call
    elif oid == "O7":
        call = _routine(have, ctx)
        if call is None:
            return None
        tool, args, needs = call
        can = can["cue" if _parts(ctx)[1] else "yours"]
    else:
        entries = ctx.get("untried")
        if not isinstance(entries, list) or not entries or not isinstance(entries[0], dict):
            return None
        entry = entries[0]
        tool, args, needs = entry.get("tool"), entry.get("args"), entry.get("needs")
        if not isinstance(tool, str) or not isinstance(args, dict):
            return None
    if tool not in have or not _budget(tool, ctx) or unsafe(args):
        return None
    args = _copy(args)
    return (tool, args, needs, can) if args is not None else None


def _fit(offer):
    """The offer within OFFER_MAX characters of JSON (5.1): can is dropped first, then noticed is cut keeping its
    call numbers; None when even that does not fit."""
    offer = _copy(offer)
    if not isinstance(offer, dict) or not all(k in offer for k in ("n", "noticed", "next")):
        return None
    full = W.cut(offer["noticed"], W.NOTICED_MAX)
    if not full:
        return None
    offer["noticed"] = full
    size = len(json.dumps(offer))
    if size <= W.OFFER_MAX:
        return offer
    offer.pop("can", None)
    limit = len(full)
    size = len(json.dumps(offer))
    while size > W.OFFER_MAX:
        limit -= size - W.OFFER_MAX
        text = W.cut(full, limit) if limit > 0 else ""
        if not text:
            return None
        offer["noticed"] = text
        size = len(json.dumps(offer))
    return offer


def _valid(prefs, ctx):
    prefs, ctx = _dict(prefs), _dict(ctx)
    return (prefs.get("offers") != "quiet" and not ctx.get("paused") and _int(ctx.get("next_n"))
            and ctx["next_n"] >= 1 and _int(ctx.get("call")) and _budget("assist", ctx))


def _last(history):
    history = _rows(history)
    calls = [h["call"] for h in history if _int(h.get("call"))]
    return max(calls) if calls else None


def _again(oid, history, call, count):
    """Rule 3: may this offer id be made (again) at this call, with count open events of its skill."""
    if not _int(call) or not _int(count):
        return False
    history = _rows(history)
    old = [h for h in history if h.get("id") == oid]
    if not old:
        return True
    if len(old) >= 3:
        return False
    if count >= 2:
        return True
    return len(old) == 1 and _int(old[0].get("call")) and call - old[0]["call"] >= W.AGAIN_CALLS


def _result(n, oid, skill, signal, tool, args, noticed, can, next_call):
    offer = _fit({"n": n, "noticed": noticed, "can": can, "next": next_call})
    if offer is None:
        return None
    return {"n": n, "id": oid, "skill": skill, "signal": signal, "tool": tool, "args": args, "offer": offer}


def choose_offer(snap, new_events, have, listed, history, prefs, ctx):
    """The one offer for the result of this call, or None (module text). new_events are the events that fired on
    this very call; the first of them, seeing before acting before checking, whose row passes every rule wins."""
    snap, prefs, ctx, history, have = _dict(snap), _dict(prefs), _dict(ctx), _rows(history), _names(have)
    if not _valid(prefs, ctx):
        return None
    events = [e for e in _rows(new_events) if isinstance(e.get("signal"), str) and e["signal"] in W.SIGNAL_SKILL]
    events.sort(key=lambda e: W.SKILLS.index(W.SIGNAL_SKILL[e["signal"]]))
    last = _last(history)
    for event in events:
        signal = event["signal"]
        skill = W.SIGNAL_SKILL[signal]
        count = _dict(snap.get("standing")).get(skill)
        count = count if _int(count) else 0
        if count < 1 or (last is not None and (last >= ctx["call"]
                                             or (ctx["call"] - last < W.GAP_CALLS and count < 2))):
            continue
        text = W.noticed(event)
        if not text:
            continue
        if signal == "air":
            ids = ("O4", "O2", "O1")
        elif signal == "rays15":
            ids = ("O2", "O3", "O1")
        elif skill == "seeing":
            ids = ("O2", "O1")
        elif skill == "acting":
            # where no whole step and no routine can run (neither push nor lift is loaded) an acting event is
            # answered with the list, while none is fresh: the robot numbers what it sees, the operator picks
            names = _names(have)
            bare = () if ("push" in names or "lift" in names or ctx.get("has_list")) else ("O1",)
            ids = (("O5", "O7") if signal == "refuse" else ("O7",)) + bare
        else:
            ids = ("W1",)
        for oid in ids:
            if not _again(oid, history, ctx["call"], count):
                continue
            row = _row(oid, signal, event, snap, have, ctx, history)
            if row is None:
                continue
            tool, args, needs, can = row
            next_call = call_text(tool, args, listed, ctx["next_n"], needs)
            if next_call is not None:
                result = _result(ctx["next_n"], oid, skill, signal, tool, args, text, can, next_call)
                if result is not None:
                    return result
    return None


def _run(value):
    if not isinstance(value, list) or len(value) < W.STEP_DOWN_RUN:
        return None
    last = value[-W.STEP_DOWN_RUN:]
    if all(isinstance(r, dict) and r.get("confirmed") is True and _int(r.get("call")) for r in last):
        return [r["call"] for r in last]
    return None


def step_down(snap, history, prefs, ctx):
    """D1 (prefs.seeing is numbers and the last 3 picks were each confirmed) or D2 (prefs.acting is steps or
    routine and the last 3 acts were each confirmed at the first try), once each per session, or None. The runs
    come from ctx.picks and ctx.acts ([{"call", "confirmed"}] newest last); without them: None."""
    prefs, ctx, history = _dict(prefs), _dict(ctx), _rows(history)
    last = _last(history)
    if not _valid(prefs, ctx) or (last is not None and ctx["call"] - last < W.GAP_CALLS):
        return None
    seen = [h.get("id") for h in history]
    picks, acts = _run(ctx.get("picks")), _run(ctx.get("acts"))
    if prefs.get("seeing") == "numbers" and "D1" not in seen and picks is not None:
        oid, calls = "D1", picks
    elif prefs.get("acting") in ("steps", "routine") and "D2" not in seen and acts is not None:
        oid, calls = "D2", acts
    else:
        return None
    row = W.OFFERS[oid]
    args = _copy(row["args"])
    # assist is the one tool every client is sent; a caller that knows better passes ctx.listed
    next_call = call_text("assist", args, ctx["listed"], ctx["next_n"]) if "listed" in ctx else \
        {"tool": "assist", "args": _copy(args)}
    if next_call is None:
        return None
    text = W.STEP_DOWN_NOTICED[oid].format(calls=", ".join(str(c) for c in calls))
    return _result(ctx["next_n"], oid, row["skill"], "step_down", "assist", args, text, row["can"], next_call)


def untried(stage, snap, have, ctx):
    """At most UNTRIED_MAX ready calls {"tool", "args"} (plus "needs") for a give-up or a wind-down (4.4): after
    a give the cheap check first; while nothing is confirmed and nothing gave, a seeing rung (the list, else the
    unseen sides, else page 2); then the stage's own call, an act only on the target. Never firm, further or a
    force; never the same part harder after it did not give."""
    snap, have, ctx = _dict(snap), _names(have), _dict(ctx)
    if not _budget("assist", ctx):
        return []
    out = []

    def add(call):
        if not isinstance(call, (list, tuple)) or len(call) < 2 or len(out) >= W.UNTRIED_MAX:
            return
        tool, args = call[:2]
        if not isinstance(tool, str) or not isinstance(args, dict):
            return
        if tool not in have or unsafe(args) or not _budget(tool, ctx):
            return
        if tool == "candidates" and any(r["tool"] == tool for r in out):
            return
        args = _copy(args)
        if args is None:
            return
        entry = {"tool": tool, "args": args}
        if len(call) > 2 and call[2]:
            entry["needs"] = _copy(call[2])
        out.append(entry)

    if stage == "gave":
        add(("changed", {}))
    chosen = bool(ctx.get("picks")) and _target(ctx) is not None    # the operator picked, and a target stands
    if not chosen and not any(s.get("sign") in ("confirmed", "gave") for s in _rows(snap.get("signs"))):
        look = ctx.get("newest_look")
        if ctx.get("has_list") and not ctx.get("picks") and "assist" in have:
            add(("assist", {"pick": "N"}, "N: the number you choose"))     # a fresh list nobody picked from yet
        elif ctx.get("has_list") and ctx.get("unseen"):
            add(_seek(have, ctx))
        if not out or (len(out) == 1 and out[0]["tool"] == "changed"):
            if isinstance(look, str):
                args = {"look": look, "page": 2} if ctx.get("has_list") else {"look": look}
                add(("candidates", args))
    if stage in ("located", "ran_free"):
        add(_act(have, ctx, touch=True))
    elif stage in ("met", "moved_back", "gave_unchanged"):
        add(_act(have, ctx) or _onto(have, ctx))
    if stage in ("no_give", "gave_unchanged"):
        add(_routine(have, ctx))
    return out
