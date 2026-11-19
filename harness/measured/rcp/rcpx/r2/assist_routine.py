# SPDX-License-Identifier: AGPL-3.0-only
"""The routine the robot runs itself and its exports: try_each, act_on by a candidate
number, and the tool entries of act_on, try_each and changed. A sibling file of the assist package: it exports
TOOLS, HANDLERS and setup(ex), which rcpx/r2/assist.py merges.

try_each: the robot tries numbered candidates of the newest list one after another, each as ONE whole step
(rcpx/r2/assist_step.py do_step), until the stop condition THE MODEL STATES is met. until is required and has no
default; a call without it is refused, and the refusal's hint is a ready call with "cue changes" only when the
manual declares a cue part. What stops it, in this order after every step: a bound; a gate refusal that holds for
every candidate (STOP_CHECKS); harm_guard; the condition met; the thing moved as a whole; the scene changed (the
list is stale); two candidates in a row out of reach.

Before the first contact on a list the list's look is taken again from the same place and compared at the listed
rows: a scene that changed with no act of the robot (a device someone moved) is found there, and the list is
refused stale_list. That look is the before picture of the first step; each step's after picture is the next
step's before picture, so the watch and the acted thing are judged in the list's own pixels.

Rules of 4.7 as applied here: no call this file writes holds firm, a force or max_n (assist_step_read.ready);
skipped whatever the model listed: rows that carry keep_off, cue-coloured rows, rows on the robot's own body, the
watch, rows with no colour reading; with a forbidden part in the manual there are no default candidates; a step
is charged what its calls would cost the operator; every step is journaled as assist_step; the tool ends backed
off and never claims. Nothing here reads ground truth or names a robot, a device or a model.
"""
from rcpx import colour_words
from rcpx import r3_limits as L
from rcpx import sheet_words
from rcpx import surfaces as S
from rcpx.context import refusal
from rcpx.r2 import assist_step as A
from rcpx.r2 import assist_until as U
from rcpx.r2.assist_step_read import (OWN, back_off, guard_of, install_tick, loaded, look_rec, manual, ready, roles,
                                      same_view_pair, spec_of, state)

MAX_STEPS = 6
STATE_KINDS = ("lit", "went_dark")      # a change of light in place: a state, not a scene that moved


# ---------------------------------------------------------------- the list and its rows
def newest_list(ex):
    """(look id, the list record, None), or (None, None, a refusal) when no list can be read."""
    st = ex.state.get("eyes") if isinstance(ex.state.get("eyes"), dict) else None
    if st is None:
        return None, None, refusal("not_built", "nothing here numbers what the robot sees: the candidates tool is "
                                   "not loaded", "locate the part yourself, then act_on its id")
    key = st.get("newest")
    lst = (st.get("lists") or {}).get(key)
    if not isinstance(lst, dict) or not lst.get("items"):
        return None, None, refusal("no_list", "no numbered list was made yet", "candidates {}")
    return key, lst, None


def list_stale(ex, key, lst, g):
    """Rule 6: why the list may no longer be acted on, else None (the eyes package's own test, then the guard's
    with this file's record of acts that moved the scene after the list's look)."""
    fn = (ex.state.get("eyes") or {}).get("stale")
    why = fn(key) if callable(fn) else None
    if why:
        return str(why)
    rec = look_rec(ex, key)
    moved = [m for m in state(ex)["moved"] if rec is not None and m["t"] > float(rec.get("t") or 0.0)]
    why = g["list_stale"]({"look": key, "epoch": lst.get("epoch", 0), "call": lst.get("call", 0)},
                          int(getattr(ex, "epoch", 0)), [], bool(moved))
    return None if why is None else str(why) + (f" ({moved[0]['why']})" if moved else "")


def cue_colours(parts):
    return {c for p in parts if p["role"] == "cue" for c in (p.get("colours") or {}).values()}


def flagged(ex, key, row, parts):
    """A row with the three facts rule 5 needs beside its own keys: own_body, cue_colour, stale."""
    rec = look_rec(ex, key)
    first_hit = (ex.state.get("selfsee") or {}).get("first_hit")
    own = False
    if callable(first_hit) and rec is not None and row.get("point_px"):
        try:
            own = (first_hit(rec["q"], float(row["point_px"][0]), float(row["point_px"][1])) or {}).get("what") in OWN
        except Exception:
            own = False
    col = row.get("colour")
    return dict(row, own_body=bool(own or row.get("own_body")), stale=bool(row.get("stale")),
                cue_colour=bool(col is not None and any(colour_words.near(col, c) for c in cue_colours(parts))))


def default_watch(ex, rows, parts):
    """What the robot watches when the model names nothing: the cue's row (found by the cue part's word, else of
    one of its manual colours), else a fresh located entry of the cue part; None when the manual has no cue."""
    cues = [p for p in parts if p["role"] == "cue"]
    if not cues:
        return None
    names = {p["name"] for p in cues}
    row = next((r for r in rows if r.get("found_by") in names), None) \
        or next((r for r in rows if r.get("role") == "cue"), None) \
        or next((r for r in rows if r.get("colour") is not None
                 and any(colour_words.near(r["colour"], c) for c in cue_colours(parts))), None)
    if row is not None:
        return {"n": row["n"], "row": row}
    for e in reversed(ex.world.query(kind="located", include_stale=False)):
        p = sheet_words.label_part(e["label"], parts)
        if p is not None and p["role"] == "cue":
            return {"id": e["id"]}
    return None


def chosen_entry(ex, key, n, named=False):
    """The fresh located entry an earlier choose made for row n of this list, or None. named: the model states
    the part in this call, so an entry the robot stored as an unconfirmed candidate is not reused (it is chosen
    again under the model's own word)."""
    for e in reversed(ex.world.query(kind="located", include_stale=False)):
        d = e.get("data") or {}
        if d.get("candidate") == f"eyes:{key}:{n}" and not (named and d.get("unconfirmed")):
            return e["id"]
    return None


def scene_check(ex, run, key, rows):
    """The list's look taken again from the same place and compared at the listed rows (module text). Returns
    ("fresh", look id) | ("stale", why) | ("refused", body) | ("unchecked", why: no same view could be had)."""
    specs = [spec_of(ex, {"n": r["n"], "row": r}, key) for r in rows[:8] if r.get("box_px") is not None]
    pair = same_view_pair(ex, run.meter, key, specs)
    if isinstance(pair.get("refused"), dict):
        return "refused", pair["refused"]
    rec = pair.get("record")
    if rec is None:
        return "unchecked", pair.get("why") or "no same view"
    if rec.get("device_moved"):
        return "stale", "the thing moved as a whole since the list was made"
    moved = [it for it in rec.get("at") or [] if it.get("changed") == "yes" and it.get("kind") not in STATE_KINDS]
    boxes = [sp["box"] for sp in specs]
    near = [c for c in rec.get("changes") or [] if c.get("kind") not in STATE_KINDS and c.get("box_px") is not None
            and any(not (c["box_px"][2] < b[0] - L.RING_PX or c["box_px"][0] > b[2] + L.RING_PX
                         or c["box_px"][3] < b[1] - L.RING_PX or c["box_px"][1] > b[3] + L.RING_PX) for b in boxes)]
    if moved or near:           # a row that reads unclear because its surroundings changed too is caught by `near`
        return "stale", (f"{max(len(moved), len(near))} of the listed things look different from the same place "
                         "since the list was made")
    return "fresh", pair["look"]


def note_moved(ex, why, tool):
    state(ex)["moved"].append({"t": float(ex.sim_time()), "tool": tool, "why": str(why)})


def _post(ex, name, kwargs, result):
    """Result fields only: an act that moved the scene makes every list of an earlier look stale (rule 6)."""
    if not isinstance(result, dict) or "refused" in result:
        return None
    bm, dm = result.get("base_moved"), result.get("device_moved")
    if (isinstance(bm, dict) and bm.get("moved") is True) or bm is True:
        note_moved(ex, "the base moved", name)
    elif (isinstance(dm, dict) and dm.get("answer") == "yes") or dm is True:
        note_moved(ex, "the device moved", name)
    elif result.get("moved_whole") is True:
        note_moved(ex, "the thing moved as a whole", name)
    elif result.get("stopped") in A.SCENE_STOPS:
        note_moved(ex, f"an act stopped {result['stopped']}", name)
    return None


def setup(ex):
    state(ex)
    install_tick(ex)
    ex.add_post(lambda name, kwargs, result: _post(ex, name, kwargs, result))


def _skip_check(why):
    return {"the manual forbids it": "forbidden", "the robot's own body": "own_body",
            "it has the cue's colour": "cue_not_control", "no colour reading": "no_colour"}.get(why, "skipped")


def _number(v):
    if isinstance(v, int) and not isinstance(v, bool):
        return v
    return int(v) if isinstance(v, str) and v.strip().isdigit() else None


def _until_call(parts):
    """(until for a call this file writes, needs): the manual's cue or nothing."""
    return ("cue changes", None) if U.from_manual(parts) else (None, "until: your stop condition, two words")


def _until_from(ex, cond):
    """"the manual's cue" when the robot itself wrote "cue changes" into a call of this session, else "you"."""
    a = ex.state.get("assist") if isinstance(ex.state.get("assist"), dict) else {}
    wrote = state(ex).get("offered_until") or any(
        isinstance(r, dict) and r.get("tool") == "try_each" and U.parse((r.get("args") or {}).get("until")) == cond
        for r in (a.get("ready") or {}).values()) if isinstance(a.get("ready"), dict) else state(ex).get("offered_until")
    return "the manual's cue" if cond == {"cue": "changes"} and wrote else "you"


def has_forbidden(ex):
    """True when the manual forbids a part, or cannot be read (unknown is strict); read at call time."""
    doc = (getattr(ex, "context", None) or {}).get("devices")
    return not isinstance(doc, dict) or any(p["role"] == "forbidden" for p in manual(ex))


def _try_next(ex, numbers, parts, watch):
    args = {"candidates": list(numbers)}
    words, needs = _until_call(parts)
    if words:
        args["until"] = words
        state(ex)["offered_until"] = True
    if isinstance(watch, dict) and watch.get("n") is not None:
        args["watch"] = watch["n"]
    elif isinstance(watch, dict) and watch.get("id") is not None:
        args["watch"] = watch["id"]
    return ready(ex, "try_each", args, needs)


def _full(out, steps, detail):
    if detail == "full":
        out["full"] = {"steps": steps}
        return out
    return A.fit(out)


# ---------------------------------------------------------------- act_on
def act_on(ex, session, on=None, verb="push", part=None, until=None, detail="brief", **extra):
    """From a candidate number, a located or contact id or a part label to a checked result."""
    if extra:
        return A.bad_args("act_on", extra, 'act_on {"on": 3}')
    if verb not in A.VERBS or on is None or isinstance(on, bool) or detail not in ("brief", "full"):
        return refusal("args", "on is a candidate number, a located or contact id or a part name; verb is push, "
                       "lift or touch", 'act_on {"on": 3} or act_on {"on": "located_2", "verb": "touch"}')
    if not loaded(ex, verb):
        return refusal("not_built", f"{verb} is not loaded here, so the robot cannot run this whole step",
                       "the primitives do it by hand: approach, touch, press")
    parts, cond = manual(ex), None
    if until is not None:
        bad = U.valid(until, A.available(ex), parts)
        if bad is not None:
            return refusal(bad["check"], bad["why"], bad["hint"])
        cond = U.parse(until)
    g, key, rows, n = guard_of(ex), None, [], _number(on)
    if g is None:
        return A.start_refusal(ex, session, "act_on", 1)[1]
    if n is not None:
        key, lst, bad = newest_list(ex)
        if bad is not None:
            return bad
        why = list_stale(ex, key, lst, g)
        if why:
            return refusal("stale_list", why, "candidates {}")
        rows = lst["items"]
        row = next((r for r in rows if r.get("n") == n), None)
        if row is None:
            return refusal("unknown_candidate", f"no candidate {n} in the list of {key}",
                           "numbers: " + ", ".join(str(r.get("n")) for r in rows[:16]))
        skip = g["skip_reason"](flagged(ex, key, row, parts), True, None, roles(ex))
        if skip:
            return refusal(_skip_check(skip), f"number {n}: {skip}" + (f": {row['keep_off']}" if row.get("keep_off")
                           else ""), "choose another number; candidates {} lists them")
        target = {"id": chosen_entry(ex, key, n, part is not None), "n": n, "row": row, "look": key, "part": part,
                  "asked": part}
    else:
        e = ex.world.get(str(on).strip())
        if e is None or e.get("kind") not in ("located", "contact"):
            want = sheet_words.label_part(str(on), parts)
            hit = [x for x in ex.world.query(kind="located", include_stale=False)
                   if x["label"].strip().lower() == str(on).strip().lower()
                   or (want is not None and sheet_words.label_part(x["label"], parts) is want)]
            e = hit[-1] if hit else None
        if e is None:
            return refusal("unknown_target", f"no located or contact entry or part named {str(on)[:40]}",
                           "candidates {} numbers what the robot sees; act_on takes a number")
        target = {"id": e["id"]}
    run, bad = A.start_refusal(ex, session, "act_on", 1)
    if bad is not None:
        return bad
    try:
        watch = default_watch(ex, rows, parts)
        if n is not None:
            verdict, got = scene_check(ex, run, key, rows)
            if verdict == "stale":
                note_moved(ex, got, "act_on")
                return refusal("stale_list", f"{got}: the list of {key} is not acted on", "candidates {}")
            if verdict == "refused":
                return {"said": A.cut(f"a safety check stopped act_on before it moved: {got.get('check')}: "
                                      f"{got.get('why')}"), "stopped": "refused", "check": got.get("check"),
                        "end": back_off(ex, run.meter), "by": "robot", "next": ready(ex, "look", {})}
            run.ref, run.look = (got if verdict == "fresh" else None), key
        wspec = spec_of(ex, watch, run.ref or key) if watch else None
        step = A.do_step(ex, session, target, verb, cond, wspec, run)
        return _act_result(ex, session, run, step, verb, cond, watch, rows, parts, detail)
    finally:
        run.meter.close()


def _act_result(ex, session, run, step, verb, cond, watch, rows, parts, detail):
    n = step.get("n")
    e = ex.world.get(step["id"]) if step.get("id") else None
    label = f"number {n}" if n is not None else (f"the {e['label']}" if e else "the target")
    act, felt, seen = A.step_words(step, label, verb)
    tail = A.until_words(step, f"the thing at {label}" if n is not None else label)
    if step["end"].get("note"):
        act = step["end"]["note"] + "; " + act
    if step.get("moved_whole") and not tail:
        tail = "the thing moved; this does not show that the device changed"
    stopped = "harm_guard" if step["stopped"] in A.HARM_STOPS else step["stopped"]
    if step.get("scene"):
        note_moved(ex, step["scene"], "act_on")
    out = {"said": A.cut("; ".join(x for x in (act + ": " + felt, seen, tail) if x)), "stopped": stopped,
           "did": step["did"], "felt": step["felt"], "seen": step["seen"], "moved_whole": step["moved_whole"]}
    if "met" in step:
        out["met"] = step["met"]
    out.update(evidence=step["evidence"], shows_change=step["shows_change"], force=step["force"],
               located=step.get("id"), pictures=step["pictures"], end=step["end"], by="robot", used=run.meter.used())
    if step.get("refused"):
        out["check"] = step["refused"].get("check")
        out["hint"] = str(step["refused"].get("hint") or "")[:160]
    wn = (watch or {}).get("n") if isinstance(watch, dict) else None
    key = run.look
    others = [r["n"] for r in rows if r.get("n") != n and key is not None
              and run.guard["skip_reason"](flagged(ex, key, r, parts), True, wn, roles(ex)) is None]
    if stopped == "bound" and step.get("felt"):
        nxt = ready(ex, "changed", {})                    # the act ran, its check did not: the check is one call
    elif step.get("scene") or step.get("refused") or stopped in ("harm_guard", "bound", "not_found", "error"):
        nxt = A.fallback_next(ex, session)
    elif step.get("met") is True or step["evidence"] in ("seen", "felt"):
        w = (watch or {}).get("n") if isinstance(watch, dict) else None
        nxt = ready(ex, "changed", {"at": w} if (w is not None and step.get("met") is True
                                                 and step["evidence"] == "felt") else {})
    elif others:
        nxt = _try_next(ex, others[:MAX_STEPS], parts, watch)
    else:
        nxt = A.fallback_next(ex, session)
    out["next"] = nxt or ready(ex, "look", {})
    by = (ex.state.get("assist") or {}).get("by_robot") if isinstance(ex.state.get("assist"), dict) else None
    if isinstance(by, dict):
        by.update(steps=int(by.get("steps") or 0) + 1, moves=int(by.get("moves") or 0) + len(step["did"]),
                  seconds=round(float(by.get("seconds") or 0.0) + float(step["seconds"]), 1))
    ex._journal("assist_step", by="robot", routine="act_on", i=1, of=1, n=n, verdict=stopped,
                seconds=step["seconds"], travel_mm=out["used"]["travel_mm"])
    return _full(out, [step], detail)


# ---------------------------------------------------------------- try_each
def try_each(ex, session, candidates=None, verb="push", until=None, watch=None, part=None, max=3, detail="brief",
             **extra):
    """The robot tries each numbered candidate itself until the model's stop condition is met."""
    if extra:
        return A.bad_args("try_each", extra, 'try_each {"until": "cue changes"}')
    ok_list = candidates is None or (isinstance(candidates, (list, tuple)) and candidates
                                     and all(_number(c) is not None for c in candidates))
    if verb not in A.VERBS or not ok_list or isinstance(max, bool) or not isinstance(max, int) \
            or not 1 <= max <= MAX_STEPS or detail not in ("brief", "full"):
        return refusal("args", f"candidates is a list of numbers, verb push, lift or touch, max 1 to {MAX_STEPS}",
                       'try_each {"candidates": [2, 3], "until": "cue changes"}')
    g = guard_of(ex)
    if g is None or A.holding(ex):
        return A.start_refusal(ex, session, "try_each", max)[1]
    if not loaded(ex, verb):
        return refusal("not_built", f"{verb} is not loaded here, so the robot cannot run these steps",
                       "the primitives do it by hand: approach, touch, press")
    parts = manual(ex)
    bad = U.valid(until, A.available(ex), parts)
    if bad is not None:
        hint = bad["hint"]
        if bad["check"] == "needs_until" and U.from_manual(parts):
            call = dict({} if candidates is None else {"candidates": [_number(c) for c in candidates]},
                        until="cue changes")
            state(ex)["offered_until"] = True
            hint = (f"try_each {_dumps(call)} (from the manual: the {U.cue_part(parts)} shows the state); or state "
                    f"your own: {bad['hint']}")
        return refusal(bad["check"], bad["why"], hint)
    cond = U.parse(until)
    key, lst, bad = newest_list(ex)
    if bad is not None:
        return bad
    why = list_stale(ex, key, lst, g)
    if why:
        return refusal("stale_list", why, "candidates {}")
    rows = lst["items"]
    by_n = {r.get("n"): r for r in rows}
    role_of = roles(ex)
    if candidates is None:
        if has_forbidden(ex):
            may = [r["n"] for r in rows if g["skip_reason"](flagged(ex, key, r, parts), True, None, role_of) is None]
            return refusal("needs_candidates", "the manual forbids a part here, so the robot does not choose which "
                           "numbers to try: list them yourself", "numbers that may be listed: "
                           + (", ".join(str(x) for x in may[:16]) or "none of this list"))
        numbers = [r["n"] for r in rows if role_of.get(str(r.get("found_by") or "")) == "control"]
        numbers += [r["n"] for r in rows if r["n"] not in numbers]
    else:
        numbers = []
        for c in candidates:
            if _number(c) not in by_n:
                return refusal("unknown_candidate", f"no candidate {c} in the list of {key}",
                               "numbers: " + ", ".join(str(x) for x in list(by_n)[:16]))
            if _number(c) not in numbers:
                numbers.append(_number(c))
    wn = _number(watch)
    if watch is None:
        wt = default_watch(ex, rows, parts)
    elif wn is not None and wn in by_n:
        wt = {"n": wn, "row": by_n[wn]}
    elif isinstance(watch, str) and ex.world.get(watch.strip()) is not None:
        wt = {"id": watch.strip()}
    else:
        return refusal("unknown_watch", f"watch names no candidate of this list and no located entry: {str(watch)[:30]}",
                       "watch takes a number of the list or a located id; leave it out for the manual's cue")
    skipped, todo = [], []
    for x in numbers:
        skip = g["skip_reason"](flagged(ex, key, by_n[x], parts), candidates is not None,
                                (wt or {}).get("n"), role_of)
        (skipped.append({"n": x, "why": skip}) if skip else todo.append(x))
    run, bad = A.start_refusal(ex, session, "try_each", max)
    if bad is not None:
        return bad
    try:
        return _run(ex, session, run, key, rows, by_n, todo, skipped, wt, verb, cond, part, max, parts, detail)
    finally:
        run.meter.close()


def _dumps(obj):
    import json
    return json.dumps(obj)


def _run(ex, session, run, key, rows, by_n, todo, skipped, wt, verb, cond, part, max_steps, parts, detail):
    m = run.meter
    steps, tried, stopped, check, met_at, far, scene = [], [], "none", None, None, 0, None
    run.look, lst = key, (ex.state.get("eyes") or {}).get("lists", {}).get(key) or {}
    if todo:
        verdict, got = scene_check(ex, run, key, rows)
        if verdict == "stale":
            note_moved(ex, got, "try_each")
            return refusal("stale_list", f"{got}: the list of {key} is not acted on", "candidates {}")
        if verdict == "refused":
            stopped, check, todo = "refused", got.get("check"), []
            steps.append({"i": 0, "n": None, "stopped": "refused", "refused": got, "did": list(m.did), "seconds": 0.0})
        run.ref = got if verdict == "fresh" else None
    wspec = spec_of(ex, wt, run.ref or key) if wt else None
    for x in list(todo):
        w = m.why(new_step=True)
        if w is not None or len(tried) >= max_steps:
            stopped = "bound" if w is not None else stopped
            check = w
            break
        t0 = float(ex.sim_time())
        mm0 = m.used()["travel_mm"]
        target = {"id": chosen_entry(ex, key, x), "n": x, "row": by_n[x], "look": key, "part": None, "asked": part}
        step = A.do_step(ex, session, target, verb, cond, wspec, run)
        step["i"] = len(tried) + 1
        tried.append(x)
        steps.append(step)
        ref = step.get("refused") or {}
        chk = ref.get("check") or step.get("check")
        if step.get("scene"):
            note_moved(ex, step["scene"], "try_each")
        if step["stopped"] == "bound":
            verdict, check = "bound", step.get("bound")
        elif (ref or step["stopped"] in ("refused", "dark")) and chk in _stop_checks(ex):
            verdict, check = "refused", chk
        elif step["stopped"] in A.HARM_STOPS or step["stopped"] == "harm_guard":
            verdict, check = "harm_guard", step["stopped"]
        elif step.get("met") is True:
            verdict, met_at = "met", x
        elif step.get("moved_whole"):
            verdict = "moved_whole"
        elif step.get("scene") or list_stale(ex, key, lst, run.guard):
            verdict, scene = "scene_changed", step.get("scene") or "the list went stale"
        else:
            verdict = None
        far = far + 1 if step["stopped"] in ("out_of_reach", "cannot_reach") else 0
        ex._journal("assist_step", by="robot", routine="try_each", i=step["i"], of=min(len(todo), max_steps), n=x,
                    verdict=verdict or step["stopped"], seconds=step["seconds"],
                    travel_mm=round(m.used()["travel_mm"] - mm0, 0), t0_s=round(t0, 3),
                    t1_s=round(float(ex.sim_time()), 3))
        if verdict is not None:
            stopped = verdict
            break
        if far >= 2:
            check = "two candidates in a row out of reach"
            break
    untried = [x for x in todo if x not in tried]
    end = back_off(ex, m)
    return _result(ex, session, run, steps, tried, untried, skipped, stopped, check, met_at, scene, end, wt, cond,
                   verb, key, parts, detail)


def _stop_checks(ex):
    out = set(A.STOP_CHECKS)
    for mod in getattr(ex, "mods", None) or ():
        out.update(getattr(mod, "SAFETY_CHECKS", None) or ())
    return out


def _result(ex, session, run, steps, tried, untried, skipped, stopped, check, met_at, scene, end, wt, cond, verb, key,
            parts, detail):
    last = steps[-1] if steps else {}
    nums = ", ".join(str(x) for x in tried)
    head = (end.get("note") + "; " if end.get("note") else "") + (f"tried number{'s' if len(tried) != 1 else ''} "
                                                                    f"{nums}: " if tried else "tried nothing: ")
    lst = (ex.state.get("eyes") or {}).get("lists", {}).get(key) or {}
    unseen = [str(x) for x in lst.get("unseen") or []]
    if stopped == "met":
        rows = [r for r in last.get("seen") or [] if r["at"] == "watch" and r["changed"] == "yes"]
        colour = (f": what the robot watches changed from {rows[0]['colour_before']} to {rows[0]['colour_after']}"
                  if rows and rows[0].get("colour_before") and rows[0].get("colour_after") else "")
        tail = f"number {met_at} " + (A.until_words(last, f"the thing at number {met_at}") + colour
                                      if last.get("evidence") != "felt" else
                                      "moved under the push; this does not show that the device changed state")
    elif stopped == "moved_whole":
        tail = (f"number {tried[-1]} moved as a whole under the push: it is not a part of a fixed thing, and the "
                "list is stale now; leave it out next time")
    elif stopped == "scene_changed":
        tail = f"the scene changed ({scene}): the list is stale, nothing more was tried"
    elif stopped == "refused":
        why = str((last.get("refused") or {}).get("why") or "")
        tail = f"a safety check stopped it ({check}): {why}"
    elif stopped == "harm_guard":
        tail = f"stopped to prevent harm at number {tried[-1]} ({check}); the tool backed off"
    elif stopped == "bound":
        tail = f"stopped at its {check} bound" + (f"; untried: {', '.join(str(x) for x in untried)}" if untried
                                                   else "")
    else:
        tail = ("none met it" if tried else "every listed number is skipped") + (
            f" ({check})" if check else "") + (f"; this view cannot show the {unseen[0]}" if unseen else "")
    words = U.words(cond)
    out = {"said": A.cut(head + tail), "stopped": stopped}
    if check:
        out["check"] = check
    out.update(evidence=last.get("evidence", "none") if stopped == "met" else "none",
               shows_change=bool(stopped == "met" and last.get("shows_change")), until=words,
               until_from=_until_from(ex, cond),
               steps=[{k: v for k, v in (("i", s.get("i")), ("n", s.get("n")), ("stopped", s.get("stopped")),
                                         ("felt", (s.get("felt") or {}).get("word")),
                                         ("seen", "/".join(str(r["changed"]) for r in s.get("seen") or []) or None),
                                         ("met", s.get("met")), ("force", s.get("force"))) if v is not None}
                      for s in steps],
               met_at=met_at, untried=untried, skipped=skipped,
               pictures=list(last.get("pictures") or [])[:L.PICTURES_MAX], end=end, by="robot", used=run.meter.used())
    if A.budget(session)[1] is None:
        out["used"]["time_bound"] = "none: this session has no time budget"
    tools = getattr(session, "tools", None) or {}
    w = (wt or {}).get("n") if isinstance(wt, dict) else None
    if stopped == "met":
        nxt = ready(ex, "changed", {"at": w} if (w is not None and last.get("evidence") == "felt") else {})
    elif stopped in ("moved_whole", "scene_changed", "harm_guard", "refused"):
        nxt = A.fallback_next(ex, session)
    elif stopped == "bound" and last.get("felt") and not last.get("seen"):
        nxt = ready(ex, "changed", {})                    # the last act ran, its check did not
    elif stopped == "bound" and untried:
        nxt = _try_next(ex, untried[:MAX_STEPS], parts, wt)
    elif unseen and "seek" in tools:
        nxt = ready(ex, "seek", {})
    elif "candidates" in tools:
        nxt = ready(ex, "candidates", {"page": 2})
    elif untried:
        nxt = _try_next(ex, untried[:MAX_STEPS], parts, wt)
    else:
        nxt = A.fallback_next(ex, session)
    out["next"] = nxt or ready(ex, "look", {})
    by = (ex.state.get("assist") or {}).get("by_robot") if isinstance(ex.state.get("assist"), dict) else None
    if isinstance(by, dict):
        by.update(steps=int(by.get("steps") or 0) + len(tried), moves=int(by.get("moves") or 0) + len(run.meter.did),
                  seconds=round(float(by.get("seconds") or 0.0) + out["used"]["seconds"], 1))
    return _full(out, steps, detail)


# ---------------------------------------------------------------- the surface
_BOUNDS = ("a quarter of the time the session has left and tip travel of at most {k} the arm's reach, tested before "
           "every move it makes; it can pass a bound by one nested move, because a move is not cut short")
_DETAIL = S._str("brief (default) or full: full adds every step's working data", ["brief", "full"])
_NUM_OR_ID = {"anyOf": [{"type": "integer", "minimum": 1}, {"type": "string"}]}
_UNTIL = {"anyOf": [{"type": "string"}, {"type": "object"}],
          "description": "your stop condition, two words: cue changes | cue WORD (a state word of the manual) | "
                         "seen changes | part moves | part stays"}

TOOLS = [
    S._tool("act_on",
            "The robot does one whole step itself: from a candidate number, a located id or a part name to a checked "
            "result (it finds the point, acts, compares the same view before and after, backs off). On a thing not "
            "verified as the part it pushes once, at the first step of the push's own force ladder; it has no "
            "argument for more force. Bounds: one step, " + _BOUNDS.format(k="1 x") + ". Example: act_on "
            "{\"on\": 3}.",
            S._obj({"on": dict(_NUM_OR_ID, description="a candidate number of the newest list, a located or contact "
                                                       "id, or a part name"),
                    "verb": S._str("push (default), lift or touch", list(A.VERBS)),
                    "part": S._str("the part you say number N is (optional; without it the entry is candidate N)"),
                    "until": _UNTIL, "detail": _DETAIL}, ["on"]),
            {"on": 3}, "push candidate 3 once, compare the same view before and after, back off"),
    S._tool("try_each",
            "The robot tries numbered candidates itself, one whole step each, until YOUR stop condition is met; "
            "until is required (cue changes | cue WORD | seen changes | part moves | part stays). It never raises a "
            "force, pushes a thing not verified as the part once at the first step, skips what the manual forbids, "
            "cue-coloured things and its own body, reports each step, ends backed off and never claims. Bounds: at "
            f"most max steps (1 to {MAX_STEPS}, default 3), " + _BOUNDS.format(k="3 x") + " (one step 1 x). "
            "Example: try_each {\"until\": \"cue changes\"}.",
            S._obj({"candidates": {"type": "array", "items": {"type": "integer", "minimum": 1}, "minItems": 1,
                                   "maxItems": 16, "description": "numbers of the newest list, tried in this order "
                                   "(required when the manual forbids a part)"},
                    "verb": S._str("push (default), lift or touch", list(A.VERBS)),
                    "until": _UNTIL,
                    "watch": dict(_NUM_OR_ID, description="a number or a located id the robot watches and never "
                                                          "acts on (default: the manual's cue)"),
                    "part": S._str("the part you are looking for (kept as a note; entries stay candidate N)"),
                    "max": S._int(1, MAX_STEPS, "steps at most (default 3)"), "detail": _DETAIL}, []),
            {"until": "cue changes"}, "try the candidates until the part the manual says shows the state changes"),
    S._tool("changed",
            "The robot goes back to the view it had before the last act and says what changed there, part by part "
            "(a camera move only: it touches nothing; refused while the hand holds something). Bounds: one step, "
            + _BOUNDS.format(k="1 x") + ". Example: changed {}.",
            S._obj({"at": dict(_NUM_OR_ID, description="a located id, a part name or a candidate number (default: "
                                                       "the last acted part)"),
                    "since": {"anyOf": [{"type": "string"}, {"type": "integer", "minimum": 0}],
                              "description": "a look id to compare with (default: the look before the last act)"},
                    "detail": _DETAIL}, []),
            {}, "go back to the view before the last act and say what changed"),
]
HANDLERS = {"act_on": act_on, "try_each": try_each, "changed": A.changed}
GAUGE = {"act_on": "act", "try_each": "act", "changed": "look"}

__all__ = ["GAUGE", "HANDLERS", "MAX_STEPS", "TOOLS", "act_on", "setup", "try_each"]
