# SPDX-License-Identifier: AGPL-3.0-only
"""Package assist (SPEC-R5): the robot gauges its operator from the session and offers to take work over.

It sits at the session edge (box.on_session, as the lean package does) and joins its sibling files:
- the dispatch wrap feeds the gauge with every answered request and with what only the robot knows (assist_check),
  keeps the claim ledger and puts ONE ready call on every claim answer (assist_claim), and sends an assist take,
  pick or run on as that tool's own direct call ("via": "assist"): budget, journal, gates and harm window are its own;
- the operator_answer wrap (after the brief step) asks the offer engine for at most one offer on the result of the
  call on which an event fired: the first key, or right after a refusal's own keys. With no event and no
  preference set the session's own answer object is returned untouched;
- the list hides the help tools and robot://assist (they run and read by name) and grows once, after the operator's
  first take, pick, run or direct call of a help tool; one declared tool, assist, and the resource robot://assist.

SENSING (SPEC-R5 0.1 G5). On the answer path: one ray test and one colour read per pointing act. Off it
(assist_sense): one request to the vision equipment per labelled pointing act, queued by the dispatch wrap, STARTED
at the end of the operator_answer wrap (the last step this package has before the answer leaves; for a caller of
Session.dispatch, first thing in the next dispatch) and JOINED first thing in the next request, at most 3 s.

A sibling file that is missing or broken leaves its part out and is named in robot://assist (not_loaded). No other
package is imported; nothing reads ground truth or names a robot.
"""
import importlib
import json
import time
import traceback

from rcpx import sheet_words, surfaces
from rcpx.context import refusal

ID = "assist"
URI = "robot://assist"
SIB, MISSING = {}, {}
for _name in ("assist_gauge", "assist_signals", "assist_offers", "assist_words", "assist_ladder", "assist_check",
              "assist_claim", "assist_sense", "assist_guard", "assist_routine", "assist_seek"):
    try:
        SIB[_name] = importlib.import_module("rcpx.r2." + _name)
    except ModuleNotFoundError as _exc:
        MISSING[_name] = "missing" if _exc.name == "rcpx.r2." + _name else f"broken: {_exc}"
    except Exception as _exc:                       # a broken sibling leaves its part out
        MISSING[_name] = f"broken: {type(_exc).__name__}: {_exc}"
ladder, check, claim, sense = (SIB.get("assist_" + n) for n in ("ladder", "check", "claim", "sense"))
OWN_OK = None not in (ladder, check, claim, sense)

DESCRIPTION = ("Help from the robot, when you want it. {} lists what the robot can take over for seeing, acting and "
               "checking, each with its call. {\"take\": N} runs ready call N from a result (add \"until\": WORDS "
               "when it asks for a stop condition). {\"pick\": N} chooses numbered candidate N. {\"run\": NAME, "
               "\"args\": {...}} runs a help tool. {\"set\": WORD} sets a standing preference ({} lists the words). "
               "Free.")
ARGS = ("take", "pick", "part", "until", "run", "args", "set")
# the schema carries no "required" and no "additionalProperties" key: with them the definition a chat loop sends
# passes the 760 character bound of SPEC-R5 0.1; an argument the tool does not have is refused by the handler
ENTRY = surfaces._tool("assist", DESCRIPTION,
                       {"type": "object",
                        "properties": {"take": {"type": "integer"}, "pick": {"type": "integer"},
                                       "part": {"type": "string"}, "until": {"type": "string"},
                                       "run": {"type": "string"}, "args": {"type": "object"},
                                       "set": {"type": "string"}}},
                       {}, "list the help", free=True,
                       brief="Help from the robot: {} lists it; take N, pick N, run NAME, set WORD. Free.")
PREFS, WORDS, SET_SAID = (getattr(check, n, {}) for n in ("PREFS", "WORDS", "SET_SAID"))   # the tables of 6.5
MAX_S = 3.0
INDEX_MAX = 900
PAUSE_OFFERS, PAUSE_CALLS, TAKE_WITHIN = 2, 15, 5
FEW_HINT = "assist {\"set\": \"few\"} shortens your tool list to the help tools"
CAVEAT = "this is the robot's guess from this session; it can be wrong; nothing changes unless you call"


def _merged(attr):
    """One export of the sibling files that bring tools (the routine, seek), merged: a list or a dict."""
    as_list = attr in ("TOOLS", "LADDER", "SAFETY_CHECKS")
    out = [] if as_list else {}
    for name in ("assist_routine", "assist_seek"):
        part = getattr(SIB.get(name), attr, None)
        if part:
            out.extend(part) if as_list else out.update(part)
    return out


def _state(ex):
    st = getattr(ex, "state", None)
    return st.get(ID) if isinstance(st, dict) else None


def _journal(ex, kind, **fields):
    try:
        ex._journal(kind, **fields)
    except Exception:
        pass


def _problem(ex, where):
    _journal(ex, "r2_problem", package=ID, where=where, trace=traceback.format_exc())


def _int(v):
    if isinstance(v, int) and not isinstance(v, bool):
        return v
    return int(v) if isinstance(v, str) and v.strip().isdigit() else None


# ---------------------------------------------------------------- setup
def _bind_guard(ex):
    st, mod = _state(ex), SIB.get("assist_guard")
    if st is None or mod is None:
        return
    try:
        st["guard"] = mod.bind(ex)
    except Exception:
        st["guard"] = None
        _problem(ex, "guard bind")


def setup(ex):
    st = ex.state[ID] = {
        "gauge": None, "history": [], "ready": {}, "prefs": dict(PREFS), "ladder": [], "claim_route": None,
        "ledger": [], "sense": {"checks": 0, "seconds_total": 0.0, "seconds_max": 0.0, "timeouts": 0, "findings": []},
        "by_robot": {"steps": 0, "moves": 0, "seconds": 0.0}, "guard": None, "offers": SIB.get("assist_offers"),
        "not_loaded": [f"{k}: {v}" for k, v in sorted(MISSING.items())], "listed": set(), "calls": 0,
        "attached": False, "hidden": {}, "list_taken": False, "grown": False, "paused": False, "pause_from": 0,
        "pending": None, "queued": None, "joined": [], "picks": [], "acts": [], "base": None, "approached": None,
        "sensor": sense.Sensor() if sense is not None else None}
    if ladder is not None:
        st["ready_call"] = lambda tool, args, source, needs=None: ladder.ready_call(ex, tool, args, source, needs)
    _bind_guard(ex)
    if callable(getattr(ex, "on_event", None)):
        ex.on_event("context", lambda **_: _bind_guard(ex))     # the manual arrives after the executor is built
    for name in ("assist_routine", "assist_seek"):
        fn = getattr(SIB.get(name), "setup", None)
        if callable(fn):
            try:
                fn(ex)
            except Exception as exc:             # a sibling whose setup fails leaves its part out, and is named
                st["not_loaded"].append(f"{name}: setup failed: {type(exc).__name__}: {exc}")
                _problem(ex, f"{name} setup")


def _gauge(ex, session):
    mod = SIB.get("assist_gauge")
    if mod is None:
        return None
    b = getattr(session, "budget", None)
    b = b if isinstance(b, dict) else {}
    classes = {}
    for m in getattr(ex, "mods", ()) or ():
        classes.update(getattr(m, "GAUGE", None) or {})
    reach = None
    try:
        body = (getattr(ex, "context", None) or {}).get("body")
        body = body() if callable(body) else body
        reach = float(body["reach"]["max_reach_mm"])
    except Exception:
        reach = None
    try:
        return mod.Gauge(budget_calls=b.get("calls"), budget_s=b.get("sim_seconds"), reach_mm=reach,
                         classes=classes or None)
    except Exception:
        _problem(ex, "gauge build")
        return None


# ---------------------------------------------------------------- the surface
def FILTER(surface, resources, handlers, ex=None, box=None):
    """The help tools and robot://assist leave the listing (they still run and read by name)."""
    st = _state(ex) if ex is not None else None
    if st is None or not OWN_OK:
        return surface, resources, handlers
    st["hidden"] = {t["name"]: t for t in surface["tools"] if t["name"] in ladder.HELP}
    out = dict(surface, tools=[t for t in surface["tools"] if t["name"] not in st["hidden"]])
    out["resources"] = {k: v for k, v in (surface.get("resources") or {}).items() if k != URI}
    if box is not None:
        box.on_session(lambda session: attach(ex, session))
    return out, resources, handlers


def attach(ex, session):
    """Once the Session exists: the hidden tools run by name, and dispatch, operator_answer and _list are wrapped."""
    st = _state(ex)
    inner = getattr(session, "dispatch", None)
    if st is None or not OWN_OK or st["attached"] or not callable(inner) or getattr(inner, "assist", False):
        return
    tools = getattr(session, "tools", None)
    if isinstance(tools, dict):
        for name, entry in st["hidden"].items():
            tools.setdefault(name, entry)
    st["base"] = getattr(session, "ep_dir", None) or getattr(getattr(session, "journal", None), "dir", None)
    st["gauge"] = _gauge(ex, session)
    _bind_guard(ex)
    st["ladder"] = ladder.build(ex, session)
    st["listed"] = {t["name"] for t in (getattr(session, "surface", None) or {}).get("tools", [])}

    def dispatch(request):
        try:
            _join(ex, st)
            t0 = time.perf_counter()
            sent, how = _direct(ex, session, st, request)
        except Exception:
            _problem(ex, "before dispatch")
            t0, sent, how = time.perf_counter(), request, None
        own = time.perf_counter() - t0
        response = inner(sent)
        t1 = time.perf_counter()
        try:
            return _after(ex, session, st, sent, how, response)
        except Exception:            # a bug here must not cost the operator its answer
            _problem(ex, "after dispatch")
            return response
        finally:                     # this package's own time on the answer path of this request (the join apart)
            st["path_ms"] = round(1000.0 * (own + time.perf_counter() - t1), 3)   # read by tests, never served

    dispatch.assist = True
    if getattr(inner, "lean", False):
        dispatch.lean = True
    session.dispatch = dispatch
    answer_inner = getattr(session, "operator_answer", None)
    if callable(answer_inner):
        def operator_answer(request, response):
            out = answer_inner(request, response)
            try:
                out = _decorate(ex, session, st, out)
            except Exception:
                _problem(ex, "operator answer")
            _start_queued(ex, st)        # sensing starts only now: the answer is built
            return out
        session.operator_answer = operator_answer
    list_inner = getattr(session, "_list", None)
    if callable(list_inner):
        def listing():
            out = list_inner()
            names = [t["name"] for t in out.get("tools", [])]
            tools_out = list(out.get("tools", []))
            if st["grown"] or st["prefs"]["tools"] == "few":
                tools_out += [ladder.listed_entry(st["hidden"][n]) for n in ladder.HELP
                              if n in st["hidden"] and n not in names]
            if st["prefs"]["tools"] == "few":
                tools_out = [t for t in tools_out if t["name"] in ladder.FEW]
            st["listed"] = {t["name"] for t in tools_out}
            return dict(out, tools=tools_out, resources=[r for r in out.get("resources", []) if r.get("uri") != URI])
        session._list = listing
    st["attached"] = True
    claim.route(session, ex)


# ---------------------------------------------------------------- sensing off the answer path
def _start_queued(ex, st):
    job, st["queued"] = st.get("queued"), None
    eyes = ex.state.get("eyes") if isinstance(getattr(ex, "state", None), dict) else None
    if job is None or st.get("sensor") is None or not isinstance(eyes, dict) or not callable(eyes.get("find_at")):
        return
    st["sensor"].start(job, check.run_job(eyes["find_at"]))


def _join(ex, st):
    """First thing in a request: the findings of the sensing started after the last answer (at most MAX_S)."""
    sensor = st.get("sensor")
    if sensor is None:
        return
    _start_queued(ex, st)                # a caller that never passed operator_answer: start it now
    if not sensor.pending():
        return
    found = sensor.join(MAX_S)
    for f in found:
        _journal(ex, "assist_sense", call=f.get("call"), what=f.get("what"), verdict=f.get("verdict"),
                 seconds=f.get("seconds"), **({"timeout": True} if f.get("timeout") else {}))
    s = st["sense"]
    stats = sensor.stats()
    s.update({k: stats[k] for k in ("checks", "seconds_total", "seconds_max", "timeouts")})
    s["findings"] = (s["findings"] + [{"call": f.get("call"), "what": f.get("what"), "verdict": f.get("verdict")}
                                      for f in found])[-8:]
    st["joined"] = st.get("joined", []) + found


# ---------------------------------------------------------------- assist take, pick, run: the direct call
def _direct(ex, session, st, request):
    """(the request to send on, how): an assist take, pick or run becomes the call it stands for."""
    if not (isinstance(request, dict) and request.get("op") == "call" and request.get("name") == "assist"):
        return request, None
    args = check._args(request)
    plan = ladder.plan(ex, session, args)
    if plan[0] != "direct":
        return request, None
    eyes = ex.state.get("eyes")
    if plan[1] == "candidates" and isinstance(eyes, dict) \
            and any(h["n"] == plan[4] and h.get("id") == "O7" for h in st["history"]):
        eyes["for_routine"] = True       # the list was reached through the routine offer: its next is the routine
    out = {k: v for k, v in request.items() if k not in ("name", "args", "args_text")}
    out.update(name=plan[1], args=plan[2], via="assist")
    return out, {"how": plan[3], "n": plan[4]}


# ---------------------------------------------------------------- after every answered request
def _after(ex, session, st, sent, how, response):
    op = sent.get("op") if isinstance(sent, dict) else None
    res = response.get("result") if isinstance(response, dict) else None
    res = res if isinstance(res, dict) else {}
    gauge = st["gauge"]
    extra = check.passive(ex, sent, response, st["base"])
    if op == "call" and st.get("joined"):        # findings joined at a list, read or claim wait for the next call
        joined, st["joined"] = st["joined"], []
        extra["sense"] = [{"call": f.get("call"), "what": f.get("what"), "verdict": f.get("verdict")}
                          for f in joined]
    if op == "claim":
        c = claim.cstate(st)
        if c.get("events_for") != response.get("seq"):       # the gate did not read the gauge for this claim
            _feed(ex, gauge, sent, response, extra)
        c["events_for"] = None
        claim.record(ex, session, sent, response)
        st["pending"] = None
        return claim.with_next(ex, session, response)
    if op != "call":
        _feed(ex, gauge, sent, response, extra)
        return response
    st["calls"] += 1
    k, name = st["calls"], str(sent.get("name"))
    claim.note_call(ex, session, sent, response)
    if res.get("by") == "robot":
        steps = [s for s in (res.get("steps") or []) if isinstance(s, dict)]
        rows = steps + [d for d in (res.get("did") or res.get("legs") or []) if isinstance(d, dict)]
        br = st["by_robot"]
        br["steps"] += len(steps) or 1
        br["moves"] += sum(len(s.get("did") or []) for s in steps) + len(res.get("did") or res.get("legs") or [])
        br["seconds"] = round(br["seconds"] + sum(float(r.get("seconds") or 0.0) for r in rows), 3)
    signs = len(getattr(gauge, "signs", None) or [])
    new = _feed(ex, gauge, sent, response, extra)
    confirmed = any(s.get("sign") == "confirmed" for s in (getattr(gauge, "signs", None) or [])[signs:])
    if "refused" not in res and "stopped" in res and any(x in res for x in ("contact", "outcome_class")):
        st["acts"].append({"call": k, "confirmed": confirmed})
    elif confirmed and st["picks"]:
        st["picks"][-1]["confirmed"] = True
    st["queued"] = check.sense_job(ex, sent, response, call=k, base=st["base"])
    pend = {"seq": response.get("seq"), "events": new, "name": name, "k": k, "took": False,
            "args": check._args(sent), "session": session}
    took = how or ({"how": "direct", "n": None} if name in ladder.HELP else None)
    n = took["n"] if took and took["how"] == "take" else None
    if n is None:                        # the named call itself: the newest open ready call of this tool
        n = next((m for m in reversed(list(st["ready"])) if st["ready"][m]["taken"] is None
                  and st["ready"][m]["tool"] == name and (took or k - st["ready"][m]["call"] <= TAKE_WITHIN)), None)
    if n is not None:
        st["ready"][n]["taken"] = k
        for h in st["history"]:
            if h["n"] == n and h["taken"] is None:
                h["taken"] = k
        _journal(ex, "assist_taken", n=n, call=k, how=(took or {"how": "direct"})["how"])
        _note(ex, gauge, "taken", n=n)
    if name == "approach" and "refused" not in res:      # where the operator's own approach stands (the chain next)
        t = check._args(sent).get("target")
        st["approached"] = t.strip().lower() if isinstance(t, str) and res.get("stopped") == "goal" else None
    if name == "choose" and "refused" not in res:
        st["picks"].append({"call": k, "confirmed": False})
        _note(ex, gauge, "picked", n=check._args(sent).get("n"), look=check._args(sent).get("look"))
    if name in ("candidates", "find", "seek") and "refused" not in res:
        st["list_taken"] = True
    if took or n is not None or name == "assist":
        st["pause_from"] = len(st["history"])                # the operator used the help: offers come back
    if took:
        pend["took"] = True
        if not st["grown"]:
            st["grown"] = pend["list_changed"] = True        # the list grows once, by the operator's own act
    met = res.get("contact") is True or res.get("stopped") in ("contact", "gave", "force", "cap", "part_hit")
    if st["prefs"]["checking"] == "robot_checks" and met and "refused" not in res and res.get("by") != "robot" \
            and name != "changed" and callable(getattr(session, "handlers", {}).get("changed")):
        pend["seen"] = _robot_check(ex, session, st, k)
    st["pending"] = pend
    return response


def _robot_check(ex, session, st, k):
    """The robot_checks preference: the changed tool's own handler (its gates, bounds and refusals) after an act of
    the operator that met something; no call is charged; the step is journaled and counted in by_robot."""
    clock = getattr(session, "sim_seconds_used", None)
    t0 = float(clock()) if callable(clock) else 0.0
    out = session.handlers["changed"]()
    out = out if isinstance(out, dict) else {}
    t1 = float(clock()) if callable(clock) else t0
    skipped = (out.get("refused") or {}).get("why") if isinstance(out.get("refused"), dict) else None
    st["by_robot"]["steps"] += 1
    st["by_robot"]["seconds"] = round(st["by_robot"]["seconds"] + max(0.0, t1 - t0), 3)
    _journal(ex, "assist_step", by="robot", routine="robot_checks", i=1, of=1, call=k,
             verdict="skipped" if skipped else "seen", seconds=round(max(0.0, t1 - t0), 3), travel_mm=None)
    if skipped:
        return {"by": "robot", "skipped": skipped}
    return {"by": "robot", "said": out.get("said"), "seen": out.get("seen"), "moved_whole": out.get("moved_whole")}


def _feed(ex, gauge, sent, response, extra):
    if gauge is None:
        return []
    try:
        new = gauge.feed(sent, response, extra) or []
    except Exception:
        _problem(ex, "gauge feed")
        return []
    for e in new:
        _journal(ex, "assist_event", signal=e.get("signal"), call=e.get("call"), evidence=e.get("evidence"))
    return new


def _note(ex, gauge, kind, **data):
    try:
        if gauge is not None:
            gauge.note(kind, **data)
    except Exception:
        _problem(ex, "gauge note")


# ---------------------------------------------------------------- what the operator's answer gains
def paused(st):
    h = st["history"][st.get("pause_from", 0):]
    return (len(h) >= PAUSE_OFFERS and not any(x["taken"] for x in st["history"])
            and st["calls"] - h[-1]["call"] >= PAUSE_CALLS)


def says(st):
    snap = st["gauge"].snap() if st["gauge"] is not None else {}
    text = snap.get("says") or "no gauge is loaded: nothing is counted"
    made = len(st["history"])
    return f"{text}; {made} offer{'s' if made != 1 else ''} made, {sum(1 for h in st['history'] if h['taken'])} taken"


def _place(result, key, value):
    """The result with `key` first, or right after a refusal's own keys."""
    if "refused" not in result:
        return {key: value, **result}
    out = {}
    for k, v in result.items():
        out[k] = v
        if k == "refused":
            out[key] = value
    return out


def _offer(ex, session, st, pend):
    engine, gauge = st.get("offers"), st["gauge"]
    if engine is None or gauge is None or st["prefs"]["offers"] != "on" or pend["took"]:
        return None
    stepped = st["prefs"]["seeing"] == "numbers" or st["prefs"]["acting"] in ("steps", "routine")
    if not pend["events"] and not stepped:
        return None                      # no event and no preference set: nothing is read, nothing is built
    st["paused"] = paused(st)
    ctx = ladder.context(ex, session)
    ctx.update(picks=list(st["picks"]), acts=list(st["acts"]))
    have = set(getattr(session, "tools", None) or ())
    history = [{k: h[k] for k in ("n", "id", "call", "taken")} for h in st["history"]]
    if pend["events"]:
        if st["paused"]:
            return None
        ctx["untried"] = ladder.untried(ex, session, ctx)
        off = engine.choose_offer(gauge.snap(), pend["events"], have, st["listed"], history, st["prefs"], ctx)
    else:
        off = engine.step_down(gauge.snap(), history, st["prefs"], ctx)
    if not isinstance(off, dict) or off.get("n") in st["ready"]:
        return None
    tool, args = off["tool"], dict(off.get("args") or {})
    if ladder.not_safe(ex, tool, args) or (ladder.is_contact(tool, args) and st["guard"] is None):
        return None
    nxt = (off.get("offer") or {}).get("next") or {}
    st["ready"][off["n"]] = {"tool": tool, "args": args, "from": "offer", "call": pend["k"],
                             "epoch": getattr(ex, "epoch", 0), "look": ladder._look_of(ex, tool, args),
                             "needs": nxt.get("needs"), "taken": None,
                             "key": json.dumps([tool, args, nxt.get("needs")], sort_keys=True, default=str)}
    st["history"].append({"n": off["n"], "id": off.get("id"), "call": pend["k"], "taken": None,
                          "signal": off.get("signal"), "noticed": off["offer"].get("noticed"), "next": nxt})
    _journal(ex, "assist_offer", n=off["n"], id=off.get("id"), call=pend["k"], offer=off["offer"])
    return off["offer"]


CHAIN = ("touch", "press")      # the operator's own contact calls that follow a pick and write no next themselves
PRESS_NEEDS = "travel_mm and force_n: yours (how far the part moves and how hard to press it)"


def _chain_next(ex, st, pend, result):
    """The one step after the operator's own contact, only for an operator that picked a number (it took the
    seeing help and is on the chain list, pick, approach, touch, press, check). After a touch that met something:
    press along the same line, the travel and the force left to the operator (RCP writes no force). After a press
    that met something: changed {}, the robot's own check. None otherwise; a tool's own next is never replaced."""
    met = result.get("contact") is True or result.get("stopped") in ("contact", "gave", "force", "moved_part")
    if not met or result.get("by") == "robot":
        return None
    args, session = pend.get("args") or {}, pend.get("session")
    if pend["name"] == "touch" and "press" in getattr(session, "tools", ()):
        # only where the touch met a surface at a located point (the gauge's own sign for this call) and an entry
        # stands that the guard passes as a target (4.7 rule 2): a press is never written toward anything else
        signs = getattr(st["gauge"], "signs", None) or []
        near = any(s.get("sign") == "surface_met" and s.get("call") == pend["k"] for s in signs[-4:])
        rows = [r for r in ladder.context(ex, session)["located"] if r.get("target_ok") is None]
        onto = st.get("approached")              # or the touch follows the operator's approach onto such an entry
        near = near or (onto is not None and any(onto in (str(r.get("id")).lower(), str(r.get("label")).lower())
                                                 for r in rows))
        call = {k: args[k] for k in ("direction", "frame") if k in args}
        if not (near and rows and "direction" in call):
            return None
        part = sheet_words.label_part(rows[-1].get("label"), check.manual(ex)["plist"])
        moves = " ".join(str((part or {}).get("moves") or "").split())[:90]      # the manual's own sentence
        return ladder.ready_call(ex, "press", call, "next", PRESS_NEEDS + (f"; the manual: {moves}" if moves else ""))
    if pend["name"] == "press" and "changed" in getattr(session, "tools", ()):
        return ladder.ready_call(ex, "changed", {}, "next")
    return None


def _decorate(ex, session, st, out):
    pend = st.get("pending")
    result = out.get("result") if isinstance(out, dict) else None
    if not (pend and isinstance(result, dict) and out.get("op") == "call" and out.get("seq") == pend["seq"]):
        return out
    st["pending"] = None
    new = result
    offer = _offer(ex, session, st, pend)
    if offer is not None:
        new = _place(new, "offer", offer)
        if "refused" not in result and isinstance(offer.get("next"), dict):
            # the offer's call is also the result's next, the key an operator acts on (the first live episode: an
            # offer whose call sat only inside the offer was not taken). The tool's own next, written for a call
            # the robot has just questioned, is kept beside it as tool_next. A refusal's keys are never touched.
            own = new.get("next")
            new = {**new, "next": offer["next"]}
            if own is not None:
                new["tool_next"] = own
    acting = st["prefs"]["acting"]
    if offer is not None:
        pass
    elif "refused" not in result and pend["name"] != "assist" and acting in ("steps", "routine"):
        nxt = None
        if acting == "routine":          # the robot's next step for the present stage, confirmed with one integer
            for it in ladder.untried(ex, session):
                nxt = ladder.ready_call(ex, it["tool"], it["args"], "next", it.get("needs"))
                if nxt is not None:
                    break
        else:
            eid = next((v for v in (result.get("located"), result.get("id"))
                        if isinstance(v, str) and v.startswith("located_")), None)
            guard = st["guard"]
            flags = dict((getattr(st["gauge"], "flags", None) or {}).get(eid) or {})
            if eid is not None and guard is not None and guard["target_ok"](eid, flags) is None:
                tool = "push" if "push" in session.tools else "act_on"
                nxt = ladder.ready_call(ex, tool, {"target": eid} if tool == "push" else {"on": eid}, "next")
        if nxt is not None:
            new = {**new, "next": nxt}
    elif "refused" not in result and st["picks"] and "next" not in result and pend["name"] in CHAIN:
        nxt = _chain_next(ex, st, pend, result)
        if nxt is not None:
            new = {**new, "next": nxt}
    if "refused" not in result and st["prefs"]["seeing"] == "numbers" and result.get("look_id") is not None:
        eyes = ex.state.get("eyes") if isinstance(ex.state.get("eyes"), dict) else {}
        if callable(eyes.get("list")):
            lst = eyes["list"](result["look_id"]) or {}
            rows = [{k: r.get(k) for k in ("n", "found_by", "where", "colour")} for r in lst.get("candidates") or []]
            new = {**new, "candidates": rows[:8], "numbered": (lst.get("pictures") or [None])[0],
                   "pick": "assist {\"pick\": N}"}
    if pend.get("seen") is not None:
        new = {**new, "seen": pend["seen"]}
    if pend["took"]:
        new = {**new, "state": says(st)}
        if pend.get("list_changed"):
            new["list_changed"] = True
            notes = (st["gauge"].snap().get("notes") or {}) if st["gauge"] is not None else {}
            if sum(v for k, v in notes.items() if k in ("malformed", "unknown_tool")) >= 2:
                new["few"] = FEW_HINT
    return out if new is result else dict(out, result=new)


# ---------------------------------------------------------------- the tool and the resource
def _index(ex, session, st):
    idx = ladder.index(ex, session, st["listed"])
    rows = {skill: [f"{r['tool']}: {r['call']}" if "call" in r else f"{r['tool']}: off ({r['off']})" for r in items]
            for skill, items in idx.items()}
    standing = st["gauge"].standing() if st["gauge"] is not None else {}
    out = {"said": "what the robot can take over, lowest rung first, each with its call; nothing changes unless "
                   "you call", "state": says(st), "standing": standing, "ladder": rows,
           "ready": ladder.open_ready(st, 3), "set": " | ".join(WORDS)}
    while len(json.dumps(out)) > INDEX_MAX:              # the brief form: shed the longest block first
        longest = max(out["ladder"], key=lambda s: len(json.dumps(out["ladder"][s])), default=None)
        if out["ready"]:
            out["ready"] = out["ready"][1:]
        elif longest is not None and len(out["ladder"][longest]) > 1:
            out["ladder"][longest] = out["ladder"][longest][:-1]
        else:
            out["state"] = out["state"][:max(0, len(out["state"]) - 60)]
            if not out["state"]:
                break
    return out


def _set(ex, session, st, word):
    if word not in WORDS:
        return refusal("unknown_word", f"no preference {str(word)[:40]!r}", "words: " + " | ".join(WORDS))
    key, value = WORDS[word]
    eyes = ex.state.get("eyes") if isinstance(ex.state.get("eyes"), dict) else {}
    if word == "numbers" and not callable(eyes.get("list")):
        return refusal("not_built", "no numbered list is loaded here", "the other words: " + " | ".join(WORDS))
    if word == "robot_checks" and "changed" not in session.tools:
        return refusal("not_built", "the robot's own compare (changed) is not loaded here",
                       "the other words: " + " | ".join(WORDS))
    before = st["prefs"][key]
    st["prefs"][key] = value
    _journal(ex, "assist_pref", word=word, call=st["calls"] + 1)
    _note(ex, st["gauge"], "pref", word=word)
    out = {"said": SET_SAID[word], "set": word, "prefs": dict(st["prefs"]), "state": says(st)}
    if key == "tools" and before != value:
        out["list_changed"] = True
    return out


def _assist(ex, session, **kw):
    st = _state(ex)
    if st is None or session is None or not st["attached"]:
        return refusal("unavailable", "assist needs a session", "call the tool you want by its own name")
    stray = sorted(k for k in kw if k not in ARGS)
    if stray:
        return refusal("args", f"assist has no argument {', '.join(stray)[:60]}", "its arguments: " + ", ".join(ARGS),
                       field=stray[0])
    plan = ladder.plan(ex, session, {k: v for k, v in kw.items() if v is not None})
    if plan[0] == "refuse":
        return plan[1]
    if plan[0] == "stale":
        _journal(ex, "assist_stale", n=plan[1], why=plan[2])
        return ladder.stale_refusal(ex, session, plan[1], plan[2])
    if plan[0] == "direct":              # only when the session's dispatch was replaced after this package attached
        return refusal("unavailable", f"assist could not send {plan[1]} on", f"call {plan[1]} by its own name")
    if plan[0] == "set":
        return _set(ex, session, st, plan[1])
    return _index(ex, session, st)


def resource(ex, session=None, **_):
    """robot://assist (SPEC-R5 2.6): what the robot thinks of this session, with the calls that are its evidence."""
    st = _state(ex)
    snap = st["gauge"].snap() if st["gauge"] is not None else {}
    eyes = ex.state.get("eyes") if isinstance(ex.state.get("eyes"), dict) else {}
    service = eyes.get("service") if isinstance(eyes.get("service"), dict) else {}
    stopped = paused(st)
    return {"says": says(st), "caveat": snap.get("caveat") or CAVEAT,
            "standing": snap.get("standing") or {"seeing": 0, "acting": 0, "checking": 0},
            "events": [{k: e.get(k) for k in ("signal", "skill", "call", "arm", "evidence")}
                       for e in snap.get("events") or []],
            "signs": snap.get("signs") or [], "notes": snap.get("notes") or {}, "pointing": snap.get("pointing") or {},
            "sensing": dict(st["sense"]), "stage": snap.get("stage"),
            "offers": [{k: h.get(k) for k in ("n", "id", "call", "signal", "noticed", "next", "taken")}
                       for h in st["history"]],
            "paused": stopped, **({"resume": "assist {} or any ready call"} if stopped else {}),
            "ready": ladder.open_ready(st), "prefs": dict(st["prefs"]),
            "ladder": ladder.index(ex, session, st["listed"]) if session is not None else {},
            "equipment": {"vision": eyes.get("level") or "absent", "describe": eyes.get("describe"),
                          "seconds_per_call": service.get("seconds_last"), "fallbacks": service.get("fallbacks")},
            "by_robot": dict(st["by_robot"]), "off": snap.get("off") or [], "not_loaded": list(st["not_loaded"]),
            "quiet": st["prefs"]["offers"] == "quiet", "guard": "bound" if st["guard"] is not None else "none: the "
            "robot writes no act call", "claim_route": st["claim_route"]}


TOOLS = [ENTRY] + _merged("TOOLS")
HANDLERS = {**_merged("HANDLERS"), "assist": _assist}
LADDER, GAUGE, UNTIL, SAFETY_CHECKS = (_merged(a) for a in ("LADDER", "GAUGE", "UNTIL", "SAFETY_CHECKS"))
RESOURCES = {URI: (resource, "What the robot has noticed about this session, with the calls that are its evidence, "
                             "and what it can take over. A guess that can be wrong. A read: free.")}
if claim is not None:
    CLAIM_GATE, claim_fields = claim.CLAIM_GATE, claim.claim_fields

__all__ = ["ARGS", "DESCRIPTION", "FILTER", "HANDLERS", "ID", "RESOURCES", "TOOLS", "URI", "attach", "paused",
           "resource", "says", "setup"]
