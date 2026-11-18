# SPDX-License-Identifier: AGPL-3.0-only
"""One whole step the robot runs itself, with the check inside (SPEC-R5 4.2, 4.3, 4.7): do_step, and the two tools
that are one step each: act_on (from a number, an id or a part label to a checked result) and changed (the robot
goes back to the view before the last act and says what changed).

A step is: a numbered candidate chosen (when it has no located entry yet), ONE act by a public tool (push, lift, or
approach and touch), then the robot's own check from the same view, then the tool off the part. Everything that
moves is a public primitive called as the operator's own call, so every gate, force cap and refusal of a model's
call applies (the hook core runs every pre hook, the journal and every post hook for it; ex.gates is NOT called
before a public primitive as well: that would run pre hooks with side effects twice). On top of that:
- a contact the robot makes outside push and lift (verb touch) is tested by the guard's keep-off rule first, and the
  felt point again after contact (rule 3 of 4.7);
- on an entry whose identity is not verified a push or lift is ONE step at the verb's own first force, sent as
  max_n, which can only lower a ceiling (rule 4). Nothing here sets firm or raises a force;
- bounds are tested before EVERY nested call (rule 7), and every exit leaves the tool backed off (rule 8);
- what the step touched is stored as "candidate N", unconfirmed, never under a part's name (rule 10).
With no guard (rcpx/r2/assist_guard.py neither served by the assist package nor importable) act_on refuses
not_guarded: a tool that makes a contact on its own does not run without the rules of 4.7.

Readers, the meter and the same-view pair are in rcpx/r2/assist_step_read.py. Nothing here reads ground truth or
names a robot, a device or a model.
"""
import math

from rcpx import r3_limits as L
from rcpx import sheet_words
from rcpx.context import refusal
from rcpx.r2 import assist_until as U
from rcpx.r2.assist_step_read import (MM, Bound, Meter, back_off, budget, cue_read, guard_of, holding, loaded,
                                      look_before, look_rec, manual, read_act, read_record, read_touch, ready,
                                      roles, same_view_pair, spec_of, state, tip_mm, witness_record)

VERBS = ("push", "lift", "touch")
# a refusal with one of these checks is the same for every candidate, or a reason to stop: the routine ends there
STOP_CHECKS = ("dark", "no_grip", "constraint", "seen_obstacle", "forbidden", "forbidden_unplaced", "unknown_part",
               "held", "holding", "no_move", "not_guarded")
HARM_STOPS = ("cap", "part_hit", "base_moved")          # the step itself did harm, or may have: harm_guard
NO_ACT_STOPS = ("not_found", "blocked", "cannot_reach", "not_square", "forbidden", "struck", "refused", "dark",
                "out_of_reach")
SCENE_STOPS = ("sliding", "unconfirmed")                # a verb's stop that leaves the scene unknown (rule 6)


# ---------------------------------------------------------------- the step
def choose_row(ex, meter, look, n, row, part, asked):
    """A numbered candidate becomes a located entry: the eyes package's choose when it is served, else locate at
    the row's own pixel. (entry id, None) or (None, the refusal body or a stop word). Without the model's own part
    the entry is "candidate N", unconfirmed (rule 10)."""
    eyes = ex.state.get("eyes") if isinstance(ex.state.get("eyes"), dict) else {}
    if callable(eyes.get("choose")):
        r = meter.call("choose", fn=eyes["choose"], n=n, part=part, look=look, depth="auto")
    elif loaded(ex, "choose"):
        r = meter.call("choose", n=n, part=part, look=look)
    elif loaded(ex, "locate") and row.get("point_px") is not None:
        r = meter.call("locate", look=look, px=[float(x) for x in row["point_px"][:2]], part=part or "other",
                       note=f"candidate {n}")
    else:
        return None, refusal("not_built", "no tool here turns a numbered candidate into a point (choose, locate)",
                             "mark it in two looks and triangulate, then act_on the id")["refused"]
    if isinstance(r.get("refused"), dict):
        return None, r["refused"]
    eid = r.get("located") or r.get("id")
    e = ex.world.get(eid) if eid is not None else None
    if e is None:
        return None, r.get("stopped") or "not_found"
    data = {"candidate": f"eyes:{look}:{n}"}
    if part is None:
        data.update(asked=asked, unconfirmed=True,
                    identity={"verified": False, "why": "a candidate the robot chose by number: nothing names it"})
        ex.world.update(eid, label=f"candidate {n}", data=data)
    else:
        ex.world.update(eid, data=data)
    return eid, None


def _touch(ex, run, e, step):
    """verb touch: the keep-off rule on the line, reach_check, approach, touch, the felt point through the guard
    again. Returns the reading (read_touch) or None when the step ended before a touch (step holds why)."""
    g, m, d = run.guard, run.meter, e.get("data") or {}
    if e.get("point_mm") is None:
        step.update(stopped="refused", refused=refusal("no_depth", f"{e['id']} is a line of sight with no depth: a "
                    "touch needs a point", "push feels along the line itself: that is your own call")["refused"])
        return None
    P = [float(x) for x in e["point_mm"]]
    ray = (d.get("ray") or {}).get("dir") if isinstance(d.get("ray"), dict) else None
    n = d.get("normal")
    u = [-float(x) for x in n] if n else ([float(x) for x in ray] if ray else [0.0, 0.0, -1.0])
    norm = math.sqrt(sum(x * x for x in u)) or 1.0
    u = [x / norm for x in u]
    unc = float(d.get("uncertain_mm") or 0.0)
    back, ahead = L.STANDOFF_MM + L.STANDOFF_UNC_MULT * unc, L.TOUCH_UNC_MULT * unc + L.TOUCH_EXTRA_MM
    a, b = [p - x * back for p, x in zip(P, u)], [p + x * ahead for p, x in zip(P, u)]
    bad = g["keep_off"](a, b, unc)
    if bad is not None:
        step.update(stopped="refused", refused=dict(bad))
        return None
    if loaded(ex, "reach_check"):
        m.check()
        rc = m.call("reach_check", point_mm=P)
        if rc.get("reachable") is False:
            step.update(stopped="out_of_reach", why=rc.get("why"))
            return None
    if loaded(ex, "approach"):
        m.check(next_mm=math.dist(tip_mm(ex) or a, a))
        ap = m.call("approach", target=e["id"], along=u)
        if isinstance(ap.get("refused"), dict):
            chk = ap["refused"].get("check")
            step.update(stopped="out_of_reach" if chk in ("reach", "reach_unknown") else "refused",
                        refused=ap["refused"])
            return None
        if not ap.get("reached"):
            step.update(stopped=ap.get("stopped") or "blocked", why="the approach stopped before its standoff")
            return None
    m.check(next_mm=back + ahead)
    t = read_touch(m.call("touch", direction=u, max_mm=round(back + ahead, 1)))
    step["touched"] = True
    if t["refused"]:
        step.update(stopped="refused", refused=t["refused"])
        return None
    if t["contact"] and t["point_mm"] is not None:
        hit = g["felt"](t["point_mm"])
        if hit is not None:
            step.update(stopped="harm_guard", refused=dict(hit), why="the felt point lies inside a keep-off")
            return None
    return t


def _felt_words(a, verb):
    if a.get("felt"):
        last = a["felt"][-1]
        return {"word": last.get("word"), "mm": round(float(a["along_push_mm"]), 1), "n": last.get("n")}
    if verb == "touch":
        return {"word": "met" if a.get("contact") else "felt nothing", "mm": round(float(a["along_push_mm"]), 1)}
    return None


def do_step(ex, session, target, verb, until, watch, run=None):
    """One whole step (module text). target {"id"} or {"n", "row", "look"} with optional "part" (the model's own)
    and "asked"; until: a canonical stop condition or None; watch: a spec (spec_of) or None. Returns the step
    record: n, id, chose, acted, did, stopped, refused, felt, seen, moved_whole, met, why, evidence, shows_change,
    force, verified, scene, record, pictures, end."""
    own = run is None
    run = run or Run(ex, session, guard_of(ex), 1, True)
    m = run.meter
    m.steps += 1
    step = {"n": target.get("n"), "id": target.get("id"), "chose": False, "acted": False, "stopped": None,
            "refused": None, "felt": None, "seen": [], "moved_whole": False, "force": None, "verified": False,
            "evidence": "none", "shows_change": False, "scene": None, "record": None, "pictures": []}
    did0, t0 = len(m.did), float(ex.sim_time())
    try:
        _step(ex, run, target, verb, until, watch, step)
    except Bound as b:
        step.update(act_stopped=step.get("stopped"), stopped="bound", bound=str(b))
    except Exception as exc:                          # rule 8: an exception too leaves the tool backed off
        step.update(stopped="error", why=f"{type(exc).__name__}: {exc}"[:160])
        ex._journal("r2_problem", package="assist_routine", where="do_step", trace=f"{type(exc).__name__}: {exc}")
    step["end"] = back_off(ex, m, touched=bool(step.pop("touched", False)))
    step["did"], step["seconds"] = m.did[did0:], round(float(ex.sim_time()) - t0, 1)
    state(ex)["last"] = {"id": step["id"], "n": step["n"], "t0": t0, "before": step.get("before"),
                         "watch": watch, "look": run.look}
    if own:
        m.close()
    return step


def _step(ex, run, target, verb, until, watch, step):
    m, g = run.meter, run.guard
    m.check()
    eid = target.get("id")
    if eid is None:                                   # 1. a candidate with no entry yet is chosen first
        m.charge()
        step["chose"] = True
        eid, bad = choose_row(ex, m, target.get("look"), target.get("n"), target.get("row") or {},
                              target.get("part"), target.get("asked"))
        if eid is None:
            step.update(stopped="refused" if isinstance(bad, dict) else str(bad),
                        refused=bad if isinstance(bad, dict) else None)
            return
        step["id"] = eid
    e = ex.world.get(eid)
    if e is None or e.get("kind") not in ("located", "contact"):
        step.update(stopped="refused", refused=refusal("unknown_target", f"no located or contact entry {eid}",
                                                       "robot://world lists the ids")["refused"])
        return
    if e.get("stale"):
        step.update(stopped="refused", refused=refusal("stale_target", f"{eid} is stale: the thing moved since",
                                                       "candidates {} numbers what is there now")["refused"])
        return
    ident = (e.get("data") or {}).get("identity")
    step["verified"] = bool(isinstance(ident, dict) and ident.get("verified") is True)
    part = sheet_words.label_part(e["label"], manual(ex))
    in_list = run.ref is not None and isinstance(target.get("row"), dict)
    step["before"] = run.ref
    m.check()                                         # 2. the act: one public tool, as the operator's own call
    m.charge()
    step["acted"] = True
    t_act = float(ex.sim_time())
    if verb == "touch":
        step["force"] = "touch"
        a = _touch(ex, run, e, step)
        if a is None:
            step["check"] = (step.get("refused") or {}).get("check")
            return
        step["stopped"] = a["stopped"] if a["stopped"] in HARM_STOPS else ("contact" if a["contact"]
                                                                          else "not_found")
    else:
        args = {"target": eid, "detail": "full"}
        if step["verified"]:
            step["force"] = "ladder"
        else:                                         # rule 4: one step at the verb's own first force, no climb
            args["max_n"] = g["first_force"](part["name"] if part else target.get("asked"))   # the asked part's
            step["force"], step["first_n"] = "first step", args["max_n"]
        a = read_act(m.call(verb, **args))
        step.update(stopped=a["stopped"], refused=a["refused"], check=a["check"])
        step["before"] = step["before"] or a["before"]
        if a["stopped"] in SCENE_STOPS:
            step["scene"] = f"the {verb} stopped {a['stopped']}"
    step["felt"] = _felt_words(a, verb)
    step["peak_n"] = a.get("peak_n")
    if a["refused"] or step["stopped"] in HARM_STOPS or (step["stopped"] in NO_ACT_STOPS and not in_list):
        return                 # in a list's view the check also runs after a stop that pushed nothing: a touch on
                               # the way may have moved a free thing, and the next step acts on the same pixels
    # 3. the check: the same view before and after (the list's view, the verb's own pair, or the look before the act)
    second = until == {"cue": "changes"} and watch is None
    if in_list:
        ref = run.ref
        acted = spec_of(ex, {"n": target.get("n"), "row": target["row"]}, ref)
        pair = same_view_pair(ex, m, ref, [acted, watch], second)
        if pair.get("record") is not None:
            run.ref = pair["look"]                    # the after picture is the next step's before picture
    elif a.get("record") and not second:
        ref = a.get("before")
        acted = spec_of(ex, {"id": eid}, ref)
        pair = {"record": witness_record(ex, a["record"]), "pictures": a.get("pictures") or []}
    else:
        ref = run.ref or a.get("before") or look_before(ex, t_act)
        acted = spec_of(ex, {"id": eid}, ref)
        pair = same_view_pair(ex, m, ref, [acted, watch], second) if ref else {"why": "no look before the act"}
        if run.ref is not None and pair.get("record") is not None:
            run.ref = pair["look"]
    step["before"] = step["before"] or ref
    looks = (ref, pair.get("look"))
    if isinstance(pair.get("refused"), dict):
        step["check_refused"] = pair["refused"]
    record = read_record(ex, pair.get("record"), acted, watch, pair.get("record2"), [k for k in looks if k])
    step["record"], step["pictures"] = record, list(pair.get("pictures") or [])[:L.PICTURES_MAX]
    if record is None:
        step["seen_why"] = pair.get("why") or (pair.get("refused") or {}).get("why") or "no same view"
    else:
        step["seen"] = [{"at": r["role"], "changed": r["changed"], "colour_before": r["colour_before"],
                         "colour_after": r["colour_after"]} for r in record["at"] if r["role"]]
        mine = next((r for r in record["at"] if r["role"] == "acted"), {})
        step["moved_whole"] = bool(record["device_moved"] or (mine.get("changed") == "yes"
                                                             and mine.get("kind") == "moved"))
    step["moved_whole"] = bool(step["moved_whole"] or a.get("moving"))
    if step["moved_whole"] or (record or {}).get("device_moved"):
        step["scene"] = "the thing moved as a whole"
    # 4. the stop condition, and which kind of evidence decided it
    data = {"gave": a["gave"], "moved_part": a["moved_part"], "sprang_back": a["sprang_back"],
            "along_push_mm": a["along_push_mm"], "small_mm": a.get("small_mm") or 0.0, "stopped": a["stopped"],
            "moved_whole": step["moved_whole"]}
    if until is not None:
        cue = {"state": None, "before": None, "watch": watch is not None}
        if U.form(until) == "cue WORD":
            cue["state"] = cue_read(ex, m, watch, next(iter(until.values())))
        verdict, why, evidence = U.met(until, data, record, cue, run.table)
        step.update(met=verdict if verdict is not None else "unknown", why=why, evidence=evidence,
                    shows_change=bool(verdict is True and U.shows_change(evidence, step["verified"])))
    else:
        mine = next((r for r in (record or {}).get("at", []) if r["role"] == "acted"), None)
        seen_yes = bool(mine and mine.get("changed") == "yes" and not step["moved_whole"])
        step["evidence"] = "seen" if mine and mine.get("changed") in ("yes", "no") else \
            ("felt" if step["felt"] else "none")
        step["shows_change"] = bool(seen_yes and U.shows_change("seen", step["verified"]))


class Run:
    """One robot-run tool call: the guard, the meter with its bounds, the list it reads and the look every
    same-view compare of the call is made against (ref; None: each act's own pair)."""

    def __init__(self, ex, session, guard, steps, one_step=False):
        self.ex, self.session, self.guard = ex, session, guard
        self.bounds = guard["bounds"](steps, session, L.ACT_MAX_S)
        self.meter = Meter(ex, session, self.bounds, one_step)
        self.ref = self.look = None
        table = dict(U.UNTIL)
        for mod in getattr(ex, "mods", None) or ():       # a package's own stop conditions (SPEC-R5 4.5)
            if isinstance(getattr(mod, "UNTIL", None), dict):
                table.update(mod.UNTIL)
        self.table = table


# ---------------------------------------------------------------- what the two one-step tools share
def bad_args(tool, extra, example):
    return refusal("args", f"{tool} takes no argument named {', '.join(sorted(str(k) for k in extra))[:60]}",
                   f"for example {example}")


def available(ex):
    """The stop conditions whose evidence source is loaded now (assist_until.FORMS entries)."""
    wit = ex.state.get("witness") if isinstance(ex.state.get("witness"), dict) else {}
    out = ["cue changes", "seen changes"] if callable(wit.get("compare")) and loaded(ex, "look") else []
    if loaded(ex, "watch"):
        out.append("cue WORD")
    return out + ["part moves", "part stays"]


def start_refusal(ex, session, tool, steps):
    """(run, None) or (None, the refusal that stops a robot-run contact tool before anything moves): no guard, a
    hold or grasp, too few calls or seconds for the steps asked (rule 7)."""
    g = guard_of(ex)
    if g is None:
        return None, refusal("not_guarded", f"the rules for what the robot may do on its own are not loaded, so "
                             f"{tool} makes no contact by itself", "make the move with your own call")
    h = holding(ex)
    if h is not None:
        return None, refusal("holding", f"the hand holds something ({h}): {tool} does not start while a hold or "
                             "grasp is active", "put it down or release it first")
    try:
        run = Run(ex, session, g, steps, one_step=tool != "try_each")
    except Exception as exc:
        return None, refusal("not_built", f"the bounds of {tool} could not be built: {type(exc).__name__}",
                             "robot://body must state the reach envelope")
    bad = run.bounds.start()
    if bad is not None:
        run.meter.close()
        return None, refusal(bad.get("check"), bad.get("why"), bad.get("hint"))
    return run, None


def step_words(step, label, verb):
    """The act, what was felt and what was seen, in that order (the first three parts of said)."""
    force = {"first step": f" once at the first step ({step.get('first_n')} N)", "ladder": " by its force ladder",
             "touch": ""}.get(step.get("force"), "")
    act = f"{'touched' if verb == 'touch' else verb + 'ed' if verb == 'lift' else 'pushed'} {label}{force}"
    f = step.get("felt") or {}
    ref = step.get("refused") or {}
    if ref:
        felt = f"refused ({ref.get('check')}): {str(ref.get('why') or '')[:90]}"
    elif step.get("stopped") == "bound" and not f:
        felt = f"stopped at its {step.get('bound')} bound before the act"
    elif step.get("stopped") == "error":
        felt = f"it failed inside the robot ({step.get('why')}); the tool backed off"
    elif not f:
        felt = f"it stopped {step.get('stopped')}"
    elif f.get("word") in ("met", "felt nothing"):
        felt = "it met a surface" if f["word"] == "met" else "it felt nothing"
    else:
        felt = {"rigid": "it did not move", "came_back": f"it moved {f.get('mm')} mm and came back",
                "bottomed": f"it moved {f.get('mm')} mm and came back", "small_move": "it barely moved"}.get(
            f.get("word"), f"it moved {f.get('mm')} mm ({f.get('word')})")
    rows = step.get("seen") or []
    if step.get("moved_whole"):
        seen = "the same view shows the thing moved as a whole"
    elif rows:
        seen = "same view: " + ", ".join(
            f"{'the thing' if r['at'] == 'acted' else 'what the robot watches'} "
            + ("CHANGED" + (f" from {r['colour_before']} to {r['colour_after']}" if r.get("colour_before")
                            and r.get("colour_after") and r["colour_before"] != r["colour_after"] else "")
               if r["changed"] == "yes" else "no change" if r["changed"] == "no" else "unclear") for r in rows[:2])
    elif step.get("stopped") == "bound" and f:
        seen = f"stopped at its {step.get('bound')} bound before the check: changed {{}} goes back and compares"
    elif step.get("acted") and not ref and step.get("stopped") not in NO_ACT_STOPS + HARM_STOPS + ("bound", "error"):
        seen = "no same view to compare with: " + str(step.get("seen_why") or "nothing judged")[:60]
    else:
        seen = ""
    return act, felt, seen


def until_words(step, n_words):
    """How the step answers the model's stop condition, naming the kind of evidence."""
    if "met" not in step:
        return ""
    if step["met"] is True and step["evidence"] == "cue":
        return "met your condition by the cue"
    if step["met"] is True and step["evidence"] == "felt":
        return f"{n_words} moved under the push; this does not show that the device changed state"
    if step["met"] is True:
        return f"met your condition by what was {step['evidence']}" + (
            "" if step["shows_change"] else "; this does not show that the device changed state")
    if step["met"] is False:
        return "your condition is not met"
    return "your condition could not be judged: " + str(step.get("why") or "")[:50]


def cut(text, n=L.SAID_MAX_WORDS):
    ws = str(text).split()
    return str(text) if len(ws) <= n else " ".join(ws[:n]).rstrip(",;:") + " ..."


def fit(out, drop=("did", "used", "seen", "skipped", "untried", "felt")):
    """A brief result at most BRIEF_MAX_BYTES besides pictures: the named keys are left out one by one, last the
    steps' own detail; said, stopped, end and next always stay."""
    import json

    def size(o):
        return len(json.dumps({k: v for k, v in o.items() if k != "pictures"}, default=str))
    for k in drop:
        if size(out) <= L.BRIEF_MAX_BYTES:
            break
        out.pop(k, None)
    if size(out) > L.BRIEF_MAX_BYTES and isinstance(out.get("steps"), list):
        out["steps"] = [{k: s[k] for k in ("i", "n", "stopped", "met") if k in s} for s in out["steps"]]
    return out


def fallback_next(ex, session):
    tools = getattr(session, "tools", None) or {}
    return ready(ex, "candidates", {}) if "candidates" in tools else ready(ex, "look", {})


# ---------------------------------------------------------------- changed
def changed(ex, session, at=None, since=None, detail="brief", **extra):
    """SPEC-R5 4.3: the robot goes back to the view it had before the last act and says what changed there."""
    if extra:
        return bad_args("changed", extra, "changed {}")
    h = holding(ex)
    if h is not None:
        return refusal("holding", f"the hand holds something ({h}): the camera does not go back while a hold or "
                       "grasp is active", "put it down or release it first, then changed {}")
    st = state(ex)
    last = st.get("last") or {}
    g = guard_of(ex)
    try:
        bounds = g["bounds"](1, session, L.ACT_MAX_S) if g else None
    except Exception:
        bounds = None
    m = Meter(ex, session, bounds, True)
    try:
        return _changed(ex, session, m, g, last, at, since, detail)
    except Bound as b:
        end = back_off(ex, m)
        return {"said": f"stopped at the {b} bound before the camera went back: nothing was compared",
                "stopped": "bound", "seen": [], "moved_whole": False, "end": end, "by": "robot",
                "used": m.used(), "next": ready(ex, "look", {})}
    finally:
        m.close()


def _changed(ex, session, m, g, last, at, since, detail):
    wit = ex.state.get("witness") if isinstance(ex.state.get("witness"), dict) else {}
    acts = wit.get("acts") or []
    lst = (ex.state.get("eyes") or {}).get("lists") if isinstance(ex.state.get("eyes"), dict) else None
    watch = last.get("watch")
    target = None
    if isinstance(at, int) and not isinstance(at, bool):       # a candidate number: judged in its list's own view
        look = last.get("look") or (ex.state.get("eyes") or {}).get("newest")
        row = next((r for r in ((lst or {}).get(look) or {}).get("items") or [] if r.get("n") == at), None)
        if row is None:
            return refusal("unknown_candidate", f"no candidate {at} in the newest list", "candidates {}")
        target, since = {"n": at, "row": row}, since or look
    elif isinstance(at, str) and at.strip():
        e = ex.world.get(at.strip())
        if e is None:
            part = sheet_words.label_part(at, manual(ex))
            hit = [x for x in ex.world.query(kind="located", include_stale=False)
                   if (part is not None and sheet_words.label_part(x["label"], manual(ex)) is part)
                   or x["label"].strip().lower() == at.strip().lower()]
            e = hit[-1] if hit else None
        if e is None:
            return refusal("unknown_target", f"no located or contact entry or part named {at[:40]}",
                           "robot://world lists the ids; or changed {} for the last acted part")
        target = {"id": e["id"]}
    elif last.get("id") is not None:
        target = {"id": last["id"]}
    elif acts and acts[-1].get("contact_id"):
        target = {"id": acts[-1]["contact_id"]}
    if since is None:
        since = last.get("before") if last.get("t0") is not None and (
            not acts or float(acts[-1]["t0"]) >= float(last["t0"]) - 1e-6) else None
        since = since or (look_before(ex, float(acts[-1]["t0"])) if acts else None)
    callable_pair = callable(wit.get("compare")) and loaded(ex, "look")
    if since is None and callable_pair:
        return refusal("no_act", "no act of this session has a look before it to go back to",
                       'act first (push {"target": ID}), or name the look: changed {"since": "L3"}')
    specs = [spec_of(ex, target, since) if target else None, watch]
    pair = same_view_pair(ex, m, since, specs) if callable_pair else {"why": "no same view to compare with"}
    end = back_off(ex, m)
    if isinstance(pair.get("refused"), dict):
        ref = pair["refused"]
        return {"said": cut(f"the camera could not go back to {since}: {ref.get('check')}: {ref.get('why')}"),
                "stopped": "refused", "check": ref.get("check"), "seen": [], "moved_whole": False, "end": end,
                "by": "robot", "used": m.used(), "next": ready(ex, "look", {})}
    rec = read_record(ex, pair.get("record"), specs[0], watch, None, [since, pair.get("look")])
    out = {"said": "", "stopped": "compared" if rec else "one_reading", "seen": [], "moved_whole": False,
           "reading": "unclear", "pictures": list(pair.get("pictures") or [])[:L.PICTURES_MAX], "end": end,
           "by": "robot", "used": m.used()}
    if rec is None:
        lk = m.call("look") if not pair.get("look") and loaded(ex, "look") else {}
        word = cue_read(ex, m, watch, "on") if watch else None
        out["look"] = pair.get("look") or lk.get("look_id")
        why = str(pair.get("why") or "")
        out["said"] = cut("no same view to compare with: this is one reading" + (
            f": the cue reads {word}" if word else
            (f" ({why})" if why and why != "no same view to compare with" else "")
            + (f"; {out['look']} shows the device as it is now: read its state there yourself" if out["look"]
               else "; nothing could be judged")))
        out["next"] = ready(ex, "look", {})
        return out
    rows = [r for r in rec["at"] if r["role"]] or rec["at"]
    out["seen"] = [{"at": r["role"] or r.get("label") or r.get("id"), "changed": r["changed"],
                    "colour_before": r["colour_before"], "colour_after": r["colour_after"]} for r in rows[:4]]
    mine = next((r for r in rec["at"] if r["role"] == "acted"), None)
    out["moved_whole"] = bool(rec["device_moved"] or (mine and mine["changed"] == "yes" and mine["kind"] == "moved"))
    marks = [r["changed"] for r in rows]
    out["reading"] = ("moved_whole" if out["moved_whole"] else "changed" if "yes" in marks else
                      "unchanged" if marks and all(x == "no" for x in marks) else "unclear")
    out["record"], out["look"] = rec["id"], pair.get("look")
    head = {"moved_whole": "the thing moved as a whole between the two pictures: its stored points are where it "
            "was; this is not a state change", "changed": "", "unchanged": "no change", "unclear": "unclear"}
    bits = [f"{'the acted part' if r['role'] == 'acted' else 'what the robot watches' if r['role'] == 'watch' else str(r.get('label'))}: "
            + ("CHANGED" + (f" from {r['colour_before']} to {r['colour_after']}" if r["colour_before"] and
                            r["colour_after"] and r["colour_before"] != r["colour_after"] else "")
               if r["changed"] == "yes" else "no change" if r["changed"] == "no" else "unclear") for r in rows[:3]]
    out["said"] = cut(f"went back to the view of {since} and compared: " + (
        head["moved_whole"] if out["moved_whole"] else "; ".join(bits) or "no located part is in that view"))
    tid = (target or {}).get("id")
    if out["reading"] == "moved_whole":
        nxt = fallback_next(ex, session)
    elif out["reading"] == "changed":
        nxt = ready(ex, "look", {})
    elif tid is not None and g is not None and loaded(ex, "push") and g["target_ok"](tid, {}) is None:
        nxt = ready(ex, "push", {"target": tid})          # the same whole step again: its own ladder decides
    else:
        nxt = fallback_next(ex, session)
    out["next"] = nxt or ready(ex, "look", {})
    if detail == "full":
        out["full"] = {"record": rec, "did": m.did, "since": since}
        return out
    return fit(out)
