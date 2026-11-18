# SPDX-License-Identifier: AGPL-3.0-only
"""Claim answers of the assist package (SPEC-R5 4.4): the ledger, the one ready call on every claim answer, and
assist's own gate.

THE LEDGER. The session edge records every claim response of the session in ex.state["assist"]["ledger"]:
{k (operator calls so far), success, answered, package, check, hard} (plus phase, recorded, seq). It is the one
bound across every package's gate:
  (a) a claim sent again with the same success value, with no operator call since an answer of ANY gate, is
      recorded (the one exception: assist's own hard answer, which comes a second time);
  (b) all gates together answer a give-up at most once before the first guarded act that met something and once
      after it;
  (c) assist answers at most MAX_ANSWERS claims in a session.

THE GATE. CLAIM_GATE(ex, args, session) -> None (record the claim) or {next, said, check, hard, why, not_recorded}.
It runs after every earlier package's gate (the session returns the first answer). Recorded at once: a claim whose
cannot cites a refused call of this session; a give-up in a session that holds, at ANY call, a refusal of a safety
class (SAFETY, plus what a loaded package lists in its export SAFETY_CHECKS); the newest light reading dark; every
untried call already refused; under a third of the calls left. Then the first check that holds (the words are
neutral: none grades the operator):
  no_contact (hard)         success true, no guarded act of the session met anything and nothing gave
  flip (hard)               success true after an answered give-up, with no confirmed sign since
  touch_contradicts (soft)  success true, a guarded act ran free and no feel has met anything since; once
  no_check (soft)           success true, no verify accepted and nothing felt to give; once
  budget_left (soft)        a give-up with a step not tried yet and, where the session has budgets, a third or
                            more of the calls and of the seconds left (no call budget: before any act met something)
A hard answer is not recorded until the robot's own check (the tool changed) has run once since the answer; it is
answered at most HARD_MAX times: a further send is recorded with the claim field assist_unchecked true, and after
the check the claim is recorded whatever it read, with the reading as the claim field assist_reading. In a session
with no changed tool a look of the operator since the answer stands in for it, and the reading says so.

THE READY CALL. Every claim answer that leaves the session gets ONE ready call as its first key next (with_next):
the answering gate's own first untried call when that is a seeing call the robot may write, else the first call
of the untried rule. Nothing else is added to another gate's answer.

THE ROUTE. A session that has the attribute claim_gates calls the gates itself and is never wrapped; only for a
session without it does route() wrap session._claim on the instance and call each loaded package's gate in
package order. The route is read from the attribute, never from source text.
No robot, device or part is named here; nothing reads ground truth.
"""
import json
import traceback

from rcpx.r2 import assist_ladder as ladder

SAFETY = ("forbidden", "forbidden_unplaced", "unknown_part", "cue_not_control", "cue_near", "no_grip", "constraint",
          "seen_obstacle", "path_box", "dark")
MAX_ANSWERS = 4            # claims assist answers in one session
HARD_MAX = 2               # times one hard check is answered before the claim is recorded unchecked
PLENTY = 3                 # "a third or more left": PLENTY x left >= total
SEEING = ("look", "candidates", "find", "seek", "survey", "view")
HARD = ("no_contact", "flip")
EXIT_SOFT = "or claim again to end"
EXIT_HARD = "then claim again"
SAID = {
    "no_contact": "Not recorded: no act of this session met anything, so the robot checks first; run next, ",
    "flip": "Not recorded: a give-up was answered at call {k} and no change has been checked since; run next, ",
    "touch_contradicts": "Not recorded yet: {evidence}, and nothing has touched the part since; next looks again, ",
    "no_check": "Not recorded yet: no check of the result was accepted and no give was felt; next looks "
                "again, ",
    "budget_left": "Not recorded yet: {why}; next is a step not tried yet, ",
}


def _real(ex):
    inner = getattr(ex, "_ex", None)
    return inner if inner is not None and hasattr(inner, "state") else ex


def _state(ex):
    st = getattr(ex, "state", None)
    return st.get("assist") if isinstance(st, dict) else None


def cstate(st):
    return st.setdefault("claim", {"answers": 0, "hard": {}, "soft": [], "await": None, "fields": {}, "own": None,
                                   "events": set(), "events_for": None, "refused": [], "safety": None})


def _journal(ex, kind, **fields):
    try:
        ex._journal(kind, **fields)
    except Exception:
        pass


def calls(st, session=None):
    """Operator calls answered so far: the session edge's own count, else the gauge's, else the session's."""
    if st.get("attached"):
        return int(st.get("calls") or 0)
    gauge = st.get("gauge")
    if gauge is not None and getattr(gauge, "calls", None) is not None:
        return int(gauge.calls)
    return int(getattr(session, "calls_used", 0) or 0)


def budget(session):
    """{calls_total, calls_left, s_total, s_left}; None where the session has no such budget."""
    b = getattr(session, "budget", None)
    b = b if isinstance(b, dict) else {}
    rem = {}
    if callable(getattr(session, "remaining", None)):
        try:
            rem = session.remaining() or {}
        except Exception:
            rem = {}
    return {"calls_total": b.get("calls") or None, "calls_left": rem.get("calls_left") if b.get("calls") else None,
            "s_total": b.get("sim_seconds") or None,
            "s_left": rem.get("sim_seconds_left") if b.get("sim_seconds") else None}


def _phase(st):
    return getattr(st.get("gauge"), "phase", None) or "before_met"


def _light(ex):
    last = (((ex.state.get("waiting") or {}).get("light") or {}).get("last")) or getattr(ex, "_light", None) or {}
    return last.get("can_see") if isinstance(last, dict) else None


def _safety_checks(ex):
    out = set(SAFETY)
    for mod in getattr(ex, "mods", ()) or ():
        out.update(str(c) for c in (getattr(mod, "SAFETY_CHECKS", None) or ()))
    return out


# ---------------------------------------------------------------- what the session edge tells this file
def note_call(ex, session, request, response):
    """After every answered call: keep the refusals (for the exemptions) and the robot's check a hard answer
    waits for."""
    st = _state(ex)
    if st is None:
        return
    c = cstate(st)
    res = response.get("result") if isinstance(response, dict) else None
    if not isinstance(res, dict):
        return
    name = str(request.get("name")) if isinstance(request, dict) else ""
    k = calls(st, session)
    refused = res.get("refused")
    if isinstance(refused, dict):
        check = refused.get("check")
        key = json.dumps([name, request.get("args") if isinstance(request, dict) else None], sort_keys=True,
                         default=str)
        c["refused"].append({"k": k, "seq": response.get("seq"), "check": check, "key": key})
        if check in _safety_checks(ex) and c["safety"] is None:
            c["safety"] = {"k": k, "check": check}
        return
    aw = c.get("await")
    if aw is None or aw.get("done"):
        return
    if name == "changed":
        seen = [{"at": r.get("at"), "changed": r.get("changed")} for r in (res.get("seen") or [])
                if isinstance(r, dict)][:4]
        aw.update(done=True, reading={"call": k, "by": "robot", "said": str(res.get("said") or "")[:200],
                                      "seen": seen})
    elif "changed" not in ladder._tools(session) and (res.get("look_id") is not None or name in SEEING):
        aw.update(done=True, reading={"call": k, "by": "operator", "look": res.get("look_id"),
                                      "said": "no robot check is loaded here: the operator looked again"})


def claim_events(ex, st, session, args):
    """The names of the gauge's events for this claim: read from the gauge's own claim_checks when it has one
    (nothing is fed); else the gauge is fed the claim here, once, and the session edge does not feed it again."""
    c = cstate(st)
    gauge = st.get("gauge")
    reader = getattr(gauge, "claim_checks", None)
    if callable(reader):                 # a gauge that can be asked without being fed: the edge feeds it after
        try:
            return {e["signal"] for e in reader(args.get("success")) or () if isinstance(e, dict)}
        except Exception:
            _journal(ex, "r2_problem", package="assist", where="gauge claim", trace=traceback.format_exc())
            return set()
    seq = getattr(session, "seq", None)
    if seq is not None and c.get("events_for") == seq:
        return c["events"]
    names = set()
    if gauge is not None and callable(getattr(gauge, "feed", None)):
        rem = session.remaining() if callable(getattr(session, "remaining", None)) else {}
        try:
            new = gauge.feed({"op": "claim", "args": args}, {"op": "claim", "result": {}, "budget": rem}, {}) or []
        except Exception:
            _journal(ex, "r2_problem", package="assist", where="gauge claim", trace=traceback.format_exc())
            new = []
        for e in new:
            if isinstance(e, dict) and e.get("signal"):
                names.add(e["signal"])
                _journal(ex, "assist_event", signal=e["signal"], call=e.get("call"), evidence=e.get("evidence"))
    c["events"], c["events_for"] = names, seq
    return names


def record(ex, session, request, response):
    """One ledger row for a claim response (the session edge calls it for every claim)."""
    st = _state(ex)
    if st is None or not isinstance(response, dict):
        return None
    c = cstate(st)
    args = request.get("args") if isinstance(request, dict) else None
    success = args.get("success") if isinstance(args, dict) and isinstance(args.get("success"), bool) else None
    res = response.get("result") if isinstance(response.get("result"), dict) else {}
    answered = response.get("episode") == "continues"
    seq = response.get("seq")
    package = None
    if answered:
        own = c.get("own")
        if own is not None and own.get("seq") == seq and own.get("check") == res.get("check"):
            package = "assist"
        else:
            package = _answering_package(ex, seq) or "other"
    row = {"k": calls(st, session), "success": success, "answered": answered, "package": package,
           "check": res.get("check") if answered else None, "hard": bool(res.get("hard")) if answered else False,
           "phase": _phase(st), "recorded": response.get("episode") == "claimed", "seq": seq}
    st["ledger"].append(row)
    _journal(ex, "assist_claim", k=row["k"], success=success, answered=answered, package=package,
             check=row["check"], hard=row["hard"])
    gauge = st.get("gauge")
    if answered and callable(getattr(gauge, "note", None)):
        try:
            gauge.note("claim_answered", success=success)
        except Exception:
            _journal(ex, "r2_problem", package="assist", where="gauge note", trace=traceback.format_exc())
    return row


def _answering_package(ex, seq):
    """The package whose own state says it answered the claim of this seq (finish keeps its answers)."""
    for name, state in (getattr(ex, "state", None) or {}).items():
        answers = state.get("answers") if isinstance(state, dict) and name != "assist" else None
        if isinstance(answers, list) and answers and isinstance(answers[-1], dict) and answers[-1].get("seq") == seq:
            return name
    return None


# ---------------------------------------------------------------- the gate
def _cites_refused(args, c):
    cannot = args.get("cannot")
    if not isinstance(cannot, dict):
        return False
    ev = cannot.get("evidence")
    cited = set()
    for item in ev if isinstance(ev, (list, tuple)) else [ev]:
        if isinstance(item, int) and not isinstance(item, bool):
            cited.add(item)
        elif isinstance(item, str) and item.strip().isdigit():
            cited.add(int(item.strip()))
    return bool(cited & {r["seq"] for r in c["refused"] if r.get("seq") is not None})


def _other_answered_give_up(ex, st, phase):
    """A gate of any package has answered a give-up in this phase (rule b): the ledger, and finish's own state."""
    if any(r["answered"] and r["success"] is False and r.get("phase") == phase for r in st["ledger"]):
        return True
    g1 = (ex.state.get("finish") or {}).get("g1") if isinstance(ex.state.get("finish"), dict) else None
    if isinstance(g1, dict):
        return bool(g1.get("before_act" if phase == "before_met" else "after_act"))
    return False


def _flip(st):
    """A give-up of this session was answered and no confirmed sign has come since (read from the ledger and the
    gauge's signs, so the rule holds whatever the gauge counts once)."""
    k0 = next((r["k"] for r in reversed(st["ledger"]) if r["answered"] and r["success"] is False), None)
    if k0 is None:
        return None
    signs = getattr(st.get("gauge"), "signs", None) or []
    if any(isinstance(s, dict) and s.get("sign") == "confirmed" and (s.get("call") or 0) > k0 for s in signs):
        return None
    return k0


def _air_evidence(st):
    for e in reversed(getattr(st.get("gauge"), "events", None) or []):
        if isinstance(e, dict) and e.get("signal") == "air" and e.get("evidence"):
            return str(e["evidence"])
    return "a guarded act ran free and felt nothing"


def _all_refused(items, c):
    keys = {r["key"] for r in c["refused"]}
    return bool(items) and all(json.dumps([it["tool"], it.get("args") or {}], sort_keys=True, default=str) in keys
                               for it in items)


def _next_for(ex, session, items, first=None):
    """The first of the given calls this client can be handed as a ready call: (next object, tool) or (None, None)."""
    for it in ([first] if first else []) + list(items):
        out = ladder.ready_call(ex, it["tool"], it.get("args") or {}, "claim", it.get("needs"))
        if out is not None:
            return out, it["tool"]
    return None, None


def CLAIM_GATE(ex, args, session):
    """None (record the claim) or the answer {next, said, check, hard, why, not_recorded} (module text)."""
    ex = _real(ex)
    st = _state(ex)
    if st is None or not isinstance(args, dict) or not isinstance(args.get("success"), bool):
        return None
    try:
        return _gate(ex, st, args, session)
    except Exception:
        _journal(ex, "r2_problem", package="assist", where="claim gate", trace=traceback.format_exc())
        return None


def _gate(ex, st, args, session):
    c = cstate(st)
    success = args["success"]
    k = calls(st, session)
    phase = _phase(st)
    names = claim_events(ex, st, session, args)
    last = st["ledger"][-1] if st["ledger"] else None
    aw = c.get("await")
    again = None
    if success and aw is not None:
        if aw.get("done"):                       # the robot's check ran since the hard answer: record with it
            c["fields"]["assist_reading"] = aw.get("reading")
            c["await"] = None
            return None
        if c["hard"].get(aw["check"], 0) >= HARD_MAX:
            c["fields"]["assist_unchecked"] = True
            c["await"] = None
            return None
        again = aw["check"]                      # the one answer that may come a second time
    if again is None and last is not None and last["answered"] and last["success"] == success and last["k"] == k:
        return None                              # (a) sent again right after an answer of any gate
    if not success and _other_answered_give_up(ex, st, phase):
        return None                              # (b) one give-up answer per phase for all packages
    if c["answers"] >= MAX_ANSWERS:
        return None                              # (c)
    if _cites_refused(args, c) or (not success and c["safety"] is not None) or _light(ex) == "dark":
        return None
    b = budget(session)
    if b["calls_total"] and b["calls_left"] is not None and PLENTY * b["calls_left"] < b["calls_total"]:
        return None                              # a claim forced by a spent budget always records
    items = ladder.untried(ex, session)
    if _all_refused(items, c):
        return None
    hard, first, why, fmt = False, None, "", {}
    if success:
        k0 = _flip(st)
        met_nothing = "no_contact" in names
        if again is not None:
            check = again
        elif met_nothing:
            check = "no_contact"
        elif "flip" in names or k0 is not None:
            check = "flip"
        elif "touch_contradicts" in names and "touch_contradicts" not in c["soft"]:
            check = "touch_contradicts"
        elif "no_check" in names and "no_check" not in c["soft"]:
            check = "no_check"
        else:
            return None
        hard = check in HARD
        if hard and "changed" in ladder._tools(session):
            first = {"tool": "changed", "args": {}}
        if check == "flip":
            fmt = {"k": k0 if k0 is not None else "an earlier"}
            why = f"a give-up was answered at call {fmt['k']}; no confirmed change since"
        elif check == "no_contact":
            why = "no guarded act of this session met anything and nothing gave"
        elif check == "touch_contradicts":
            fmt = {"evidence": _air_evidence(st)}
            why = fmt["evidence"] + "; no touch has met anything since"
        else:
            why = "no verify was accepted and no give was felt"
    else:
        if not items:
            return None
        if b["calls_total"]:
            if b["s_total"] and b["s_left"] is not None and PLENTY * b["s_left"] < b["s_total"]:
                return None
            used = int(b["calls_total"]) - int(b["calls_left"] or 0)
            why = f"{used} of {int(b['calls_total'])} calls used, {int(b['calls_left'] or 0)} left"
        elif phase == "before_met":
            why = "no act has met anything yet"
        else:
            return None
        check, fmt = "budget_left", {"why": why}
    nxt, tool = _next_for(ex, session, items, first)
    if nxt is None:
        return None                              # nothing this client can be handed: the claim is recorded
    c["answers"] += 1
    if hard:
        c["hard"][check] = c["hard"].get(check, 0) + 1
        c["await"] = {"check": check, "k": k, "done": False, "reading": None}
    else:
        c["soft"].append(check)
    c["own"] = {"seq": getattr(session, "seq", None), "check": check, "hard": hard, "k": k, "tool": tool}
    said = SAID[check].format(**fmt) + (EXIT_HARD if hard else EXIT_SOFT) + "."
    return {"next": nxt, "said": said, "check": check, "hard": hard, "why": why, "not_recorded": True}


def claim_fields(ex):
    """What assist adds to a recorded claim: assist_reading (the robot's check after a hard answer) or
    assist_unchecked true (a hard answer was sent again past its bound without the check)."""
    st = _state(_real(ex))
    return dict(cstate(st)["fields"]) if st is not None else {}


# ---------------------------------------------------------------- the ready call on every claim answer
def with_next(ex, session, response):
    """The claim response with ONE ready call as the first key next of its answer (a new dict), or the response
    itself when it is no answer, already leads with next, or no call can be written."""
    st = _state(ex)
    res = response.get("result") if isinstance(response, dict) else None
    if st is None or not isinstance(res, dict) or response.get("episode") != "continues":
        return response
    if next(iter(res), None) == "next":
        return response
    first = None
    own = res.get("untried")
    if isinstance(own, list) and own and isinstance(own[0], dict):
        it = own[0]
        if it.get("tool") in SEEING and not it.get("needs") and it["tool"] in ladder._tools(session) \
                and isinstance(it.get("args") or {}, dict):
            first = {"tool": it["tool"], "args": dict(it.get("args") or {})}
    nxt, tool = _next_for(ex, session, [] if first else ladder.untried(ex, session), first)
    if nxt is None and first is not None:
        nxt, tool = _next_for(ex, session, ladder.untried(ex, session))
    if nxt is None:
        return response
    _journal(ex, "assist_claim_next", seq=response.get("seq"), tool=tool)
    return dict(response, result={"next": nxt, **{k: v for k, v in res.items() if k != "next"}})


# ---------------------------------------------------------------- the route
def route(session, ex):
    """"session" when the session calls the claim gates itself (it has claim_gates: never wrapped), else "assist"
    after wrapping session._claim on the instance. Journaled once."""
    st = _state(ex)
    if st is None:
        return None
    if st.get("claim_route"):
        return st["claim_route"]
    if hasattr(session, "claim_gates"):
        by = "session"
    else:
        by = "assist"
        _wrap(session, ex)
    st["claim_route"] = by
    _journal(ex, "assist_claim_route", by=by)
    return by


def _wrap(session, ex):
    inner = getattr(session, "_claim", None)
    if not callable(inner) or getattr(inner, "assist", False):
        return
    mods = [m for m in getattr(ex, "mods", ()) or () if callable(getattr(m, "CLAIM_GATE", None))]

    def claim(seq, request):
        args = request.get("args") if isinstance(request, dict) else None
        if args is None and isinstance(request, dict) and isinstance(request.get("args_text"), str):
            try:
                args = json.loads(request["args_text"])
            except ValueError:
                args = None
        if isinstance(args, dict) and isinstance(args.get("success"), bool) and isinstance(args.get("statement"), str):
            for mod in mods:
                name = mod.__name__.rsplit(".", 1)[-1]
                try:
                    answer = mod.CLAIM_GATE(ex, args, session)
                except Exception:
                    _journal(ex, "r2_problem", package=name, where="claim gate", trace=traceback.format_exc())
                    continue
                if isinstance(answer, dict):
                    _journal(ex, "claim_answer", req=seq, check=answer.get("check"), hard=bool(answer.get("hard")),
                             package=name)
                    rem = session.remaining() if callable(getattr(session, "remaining", None)) else {}
                    return {"seq": seq, "op": "claim", "episode": "continues", "result": answer, "budget": rem}
        out = inner(seq, request)
        if isinstance(out, dict) and out.get("episode") == "claimed" and isinstance(out.get("result"), dict):
            extra = {}
            for mod in mods:
                fn = getattr(mod, "claim_fields", None)
                try:
                    extra.update(fn(ex) or {} if callable(fn) else {})
                except Exception:
                    pass
            if extra:
                out = dict(out, result={**out["result"], **extra})
        return out

    claim.assist = True
    session._claim = claim


__all__ = ["CLAIM_GATE", "HARD_MAX", "MAX_ANSWERS", "SAFETY", "budget", "calls", "claim_events", "claim_fields",
           "cstate", "note_call", "record", "route", "with_next"]
