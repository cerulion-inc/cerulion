# SPDX-License-Identifier: AGPL-3.0-only
"""Package finish (I36, SPEC-R3 4.2): the claim gate. A claim that the evidence does not carry is answered, not
recorded: a success claim with no act, no look after the act, a later step not reached, evidence against it, an
object never located after its push, or an act that moved nothing, gets a hard answer; a success claim whose
done test is not met yet, a give-up with most of the budget left and things not tried, and a give-up the evidence
contradicts get a soft answer. Every answer says why in one sentence and lists up to 3 untried calls.

Pieces:
- a post hook on every operator call records what the call did (finish_evidence.call_record): device acts, where
  they touched, how far they moved, and which looks and points each call produced, by position (see
  finish_evidence). Nothing moves; the hook only reads.
- CLAIM_GATE(ex, args, session): the gate (below). It reads finish's records, robot://task and robot://devices
  (ex.context), the World's located points, ex._looks, ex.state["witness"]["records"], ex.state["sight"]
  ["candidates"], the newest light reading and the session's budget and tools. Each other package is optional;
  without witness there are no same-view records, and an answer says so in its note.
- RESOURCES robot://progress (a read, free): per goal step its parts, located points, acts and evidence.
- FILTER (returns the tables unchanged): keeps the Box, so the post hook knows the session's seq and call index.

Rules the gate keeps: hard answers are capped at r3_limits.HARD_CAP
per episode; the next success claim is recorded with gate_overridden true. A claim sent again with the same success
value right after a soft answer (no operator call in between) skips the soft checks (the hard ones still run); a
success claim recorded that way
carries gate_overridden true too (would: the check), so a success is recorded plainly only when every goal step's
state was seen or sensed after its last act. unverified fires once per goal
step, seen_not_claimed once, give_up_early at most once before the first device act and once after; a give-up is
recorded at once in the dark, with nothing new to try, after a blocking refusal in the last 3 calls, or with a
third of the budget or less left.

PART B CALLS (core lane C2: the session's claim step, before the cannot check and before any claim is recorded):
    answer = rcpx.r2.finish.CLAIM_GATE(ex, args, session)
        ex       the hooked executor (a forwarding wrapper that holds it as ._ex is unwrapped)
        args     the claim's arguments as sent (a dict with success)
        session  the session (session.seq is the claim's seq while the claim step runs)
      None  -> record the claim, and merge rcpx.r2.finish.claim_fields(ex) -> {"gate_overridden": bool} into the
               claim record (the episode's gate_overridden);
      dict  -> answer {said, not_recorded: true, check, hard, why, untried, hint[, note]}: return {"seq", "op":
               "claim", "episode": "continues", "result": answer, "budget"}, journal claim_answer {seq, check,
               hard}, charge no call. finish also journals its own record (kind "finish_gate").
No robot, device or scene is named here, and no simulator ground truth is read.
"""

import json
import math
import inspect

from rcpx import r3_limits as L, sheet_words
from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx.r2 import finish_evidence as fe
from rcpx.r2 import finish_untried as fu

ID = "I36"
PKG = "finish"
NAME_MAP = {"verify_evidence": "verify"}         # a tool served by a handler reaches the executor under this name
BLOCKING = fe.BLOCKING
SOFT = ("unverified", "give_up_early", "seen_not_claimed")
OPTIONAL = ("witness", "sight", "act")
MM = 1000.0  # metres to millimetres


def state(ex):
    return ex.state.setdefault(PKG, {"calls": [], "look_pos": {}, "loc_pos": {}, "loc_key": {}, "records_seen": 0,
                                     "answers": [], "hard": 0, "last": None, "s1_steps": [],
                                     "g1": {"before_act": False, "after_act": False}, "g2": False,
                                     "recorded": None, "box": None})


def _real(ex):
    inner = getattr(ex, "_ex", None)
    return inner if inner is not None and hasattr(inner, "state") else ex


def _session(ex, session=None):
    if session is not None:
        return session
    box = state(ex).get("box")
    return getattr(box, "session", None) if box is not None else None


def _loaded(ex):
    return [m.__name__.rsplit(".", 1)[-1] for m in getattr(ex, "mods", ())]


# ---------------------------------------------------------------- recording (the post hook)
def _limits(ex, result):
    """(rigid_mm, small_mm) at the present pose: the largest measurement step along the base axes (or the R3 verb's
    own line). r3_limits gives it, asked with the session's bounds once it takes them: moved_mm(B, q, u), the least
    travel that shows a part moved (the arm's resolution and give, and never zero: on a robot whose readings never
    change, the error with which a touch knows where it began); small = QUANT x that. Until then from
    robot://body's stated resolution (r3_limits.inputs, rigid_mm, small_mm as they were)."""
    moved_mm = getattr(L, "moved_mm", None)
    takes_bounds = callable(moved_mm) and _first_param(moved_mm) in ("B", "bounds")
    try:
        q = getattr(getattr(ex, "_last", None), "q", None)
        if q is None:
            q = ex.backend.read()["q"]
        line = (result.get("line") or {}).get("u") if isinstance(result.get("line"), dict) else None
        axes = [line] if line else [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        if takes_bounds:
            B = bounds.of(ex)
            rigid = max(float(moved_mm(B, q, u)) for u in axes)
            return round(rigid, 3), round(R.QUANT * rigid, 3)
        facts = L.inputs(ex)
        quantum = max(L.quantum_mm(ex.model, q, u, facts["res_deg"]) for u in axes)
    except Exception:
        if takes_bounds:
            return 0.0, 0.0
        quantum = 0.0
    rigid = L.rigid_mm(quantum)
    return round(rigid, 3), round(L.small_mm(rigid), 3)


def note_call(ex, name, args, result, seq=None, call=None):
    """Record one operator call (the post hook's work; tests call it to build a state by hand)."""
    ex = _real(ex)
    st = state(ex)
    sess = _session(ex)
    name = NAME_MAP.get(name, name)
    met = isinstance(result, dict) and "refused" not in result
    rigid, small = _limits(ex, result) if met else (None, None)
    rec = fe.call_record(name, args, result, rigid_mm=rigid)
    k = len(st["calls"])
    rec["k"] = k
    rec["seq"] = seq if seq is not None else getattr(sess, "seq", None)
    rec["call"] = call if call is not None else (getattr(sess, "calls_used", None) or getattr(ex, "_calls", k + 1))
    if rigid is not None:
        rec["rigid_mm"], rec["small_mm"] = rigid, small
    if rec["r3"]:
        acts = (ex.state.get("act") or {}).get("acts") or []
        if acts and acts[-1].get("tool") == name:
            last = acts[-1]
            if fe._num(last.get("travel_mm")) is not None:       # a travel against the push reads as none
                rec["along"] = round(max(0.0, float(last["travel_mm"])), 2)
            rec["felt"] = rec["felt"] or [str(f.get("word")) for f in last.get("felt") or [] if isinstance(f, dict)]
    records = (ex.state.get("witness") or {}).get("records") or []
    new_recs = records[st["records_seen"]:]
    st["records_seen"] = len(records)
    before = {r.get("before") for r in new_recs if isinstance(r, dict)} | {rec["witness_before"]}
    for lid in list(getattr(ex, "_looks", {}) or {}):
        if lid not in st["look_pos"]:
            st["look_pos"][lid] = [k, 0 if lid in before else 2]
    for e in ex.world.entries.values():
        if e["kind"] in ("located", "contact"):
            key = [e["point_mm"], e["observed_at"]]
            if st["loc_key"].get(e["id"]) != key:
                st["loc_key"][e["id"]] = key
                st["loc_pos"][e["id"]] = [k, 2]
    st["calls"].append(rec)
    return rec


def _post(ex):
    def post(name, kwargs, result):
        note_call(ex, name, kwargs, result)
        return None
    return post


def setup(ex):
    state(ex)
    ex.add_post(_post(ex))


def FILTER(surface, resources, handlers, ex=None, box=None):
    """No table changes: finish only keeps the Box, so its records carry the session's seq and call index."""
    if ex is not None and box is not None:
        state(ex)["box"] = box
    return surface, resources, handlers


# ---------------------------------------------------------------- the facts
def _proj(ex, q, p_mm):
    try:
        u, v, d = ex.model.project(q, [float(x) / MM for x in p_mm])
    except Exception:
        return None
    if not (math.isfinite(u) and math.isfinite(v)) or d <= 0:
        return None
    return u, v, d


def _in_frame(ex, uvd):
    return uvd is not None and 0 <= uvd[0] < ex.model.cam_w and 0 <= uvd[1] < ex.model.cam_h


def _fresh(ex, e):
    if e["stale"]:
        return False
    stamp = ((ex.state.get("epochs") or {}).get("stamps") or {}).get(e["id"])
    return stamp is None or (stamp.get("epoch") == ex.epoch and not stamp.get("gone"))


def _candidates(ex, calls):
    cands = (ex.state.get("sight") or {}).get("candidates")
    if isinstance(cands, list) and cands:
        return [c for c in cands if isinstance(c, dict)]
    for c in reversed(calls):
        if c.get("candidates"):
            return c["candidates"]
    return []


def _region(ex, q, ctrl_points, cands, last_act):
    """The device region in one look: the survey box holding the control part's point, projected and grown; else a
    disc of REGION_DISC_MM round the last act's contact point."""
    for c in cands:
        floor = fe._num(c.get("floor_z_mm"))
        floor = float(ex.cfg.get("floor_z_m") or 0.0) * MM if floor is None else floor
        w, s, top = c.get("where_mm"), c.get("size_mm"), c.get("top_mm")
        if not (w and s and top is not None):
            continue
        near = _limit(ex, "DEVICE_BOX_NEAR_MM", max(c["err_mm"]) if c.get("err_mm") else None)
        if not any(abs(p[0] - w[0]) <= s[0] / 2 + near and abs(p[1] - w[1]) <= s[1] / 2 + near for p in ctrl_points):
            continue
        uv = [_proj(ex, q, [w[0] + dx * s[0] / 2, w[1] + dy * s[1] / 2, z]) for dx in (-1, 1) for dy in (-1, 1)
              for z in (floor, float(top))]
        uv = [x for x in uv if x is not None]
        if uv:
            g = L.REGION_GROW_PX
            return {"box": [min(x[0] for x in uv) - g, min(x[1] for x in uv) - g, max(x[0] for x in uv) + g,
                            max(x[1] for x in uv) + g]}
    if last_act is not None and last_act["points"]:
        c = _proj(ex, q, last_act["points"][0])
        if c is not None:
            return {"disc": [c[0], c[1], ex.model.f_px * L.REGION_DISC_MM / (c[2] * MM)]}
    return None


def _tools(ex, sess):
    if sess is not None and isinstance(getattr(sess, "tools", None), dict):
        tools = set(sess.tools)
        look = sess.tools.get("look") or {}
        like = "like" in (((look.get("args") or {}).get("properties")) or {})
    else:
        tools = set(ex.primitive_names()) if hasattr(ex, "primitive_names") else set()
        like = "witness" in _loaded(ex)
    return tools, like


def _lift_declared(ex):
    cfg = getattr(ex, "cfg", None) or {}
    if "r3_lift_declared" in cfg:
        return cfg.get("r3_lift_declared") is True
    return (getattr(ex, "context", None) or {}).get("r3_lift_declared") is True


def _light(ex):
    last = (((ex.state.get("waiting") or {}).get("light") or {}).get("last")) or getattr(ex, "_light", None) or {}
    return last.get("can_see")


def _budget(sess):
    if sess is None:
        return None
    rem = sess.remaining() if callable(getattr(sess, "remaining", None)) else {}
    b = getattr(sess, "budget", None) or {}
    return {"calls_total": int(b.get("calls") or 0), "calls_left": int(rem.get("calls_left") or 0),
            "s_total": float(b.get("sim_seconds") or 0.0), "s_left": float(rem.get("sim_seconds_left") or 0.0),
            "s_used": float(rem.get("sim_seconds_used") or 0.0), "calls_used": int(rem.get("calls_used") or 0)}


def build_facts(ex, session=None):
    """Everything the pure modules read, at this moment (finish_evidence module text)."""
    ex = _real(ex)
    st, sess = state(ex), _session(ex, session)
    doc = (ex.context or {}).get("devices") or {}
    task = (ex.context or {}).get("task") or {}
    sentence = task.get("instruction") or task.get("sentence") or task.get("task") or ""
    plist = sheet_words.parts(doc)
    steps = (sheet_words.steps(sentence, doc) if plist else None) or [sheet_words.step(None, doc)]
    parts = {}
    for p in plist:
        parts.setdefault(p["name"], {"role": p["role"], "sliding": p["sliding"], "kind": p["kind"],
                                     "size_max": max(p["size_mm"].values()) if p["size_mm"] else None,
                                     "colours": p["colours"], "states": p["states"], "how_to_tell": p["how_to_tell"]})
    pos = lambda v: tuple(v) if v is not None else fe.BEGIN   # noqa: E731
    calls = []
    for c in st["calls"]:
        c = dict(c, pos=(c["k"], 1))
        t = (c.get("r3_target") or c["args"].get("target")) if c["r3"] else None
        if t is not None:
            e = ex.world.entries.get(str(t))
            hit = sheet_words.label_part(c.get("r3_part") or (e["label"] if e else t), plist)
            c["target_part"] = hit["name"] if hit else None
            if e is not None and e.get("point_mm") and e["point_mm"] not in c["points"]:
                c["points"] = c["points"] + [e["point_mm"]]
        calls.append(c)
    located, points, fresh_contacts = [], {}, []
    for e in ex.world.entries.values():
        if e["kind"] not in ("located", "contact"):
            continue
        if e["point_mm"] is not None:
            points[e["id"]] = e["point_mm"]
        if e["kind"] == "contact":
            if _fresh(ex, e):
                fresh_contacts.append(e["id"])
            continue
        hit = sheet_words.label_part(e["label"], plist)
        located.append({"id": e["id"], "label": e["label"], "part": hit["name"] if hit else None,
                        "point": e["point_mm"], "pos": pos(st["loc_pos"].get(e["id"])), "fresh": _fresh(ex, e),
                        "part_check": (e.get("data") or {}).get("part_check") or None,
                        "unc": fe._num((e.get("data") or {}).get("uncertain_mm"))})
    acts = [c for c in calls if c["act"]]
    cands = _candidates(ex, calls)
    ctrl = [x["point"] for x in located if x["point"] is not None and x["part"] == (steps[0] or {}).get("control")]
    records = []
    for r in (ex.state.get("witness") or {}).get("records") or []:
        if isinstance(r, dict) and r.get("after"):
            records.append(dict(r, before_pos=pos(st["look_pos"].get(r.get("before"))),
                                after_pos=pos(st["look_pos"].get(r.get("after")))))
    after_ids = {r["after"] for r in records}
    looks = {}
    for lid, rec in (getattr(ex, "_looks", {}) or {}).items():
        q = rec.get("q")
        lk = {"pos": pos(st["look_pos"].get(lid)), "can_see": rec.get("can_see"), "shows": [], "region": None}
        if q is not None:
            for eid, p in points.items():
                if _in_frame(ex, _proj(ex, q, p)):
                    lk["shows"].append(eid)
            for a in acts:
                if a["points"] and _in_frame(ex, _proj(ex, q, a["points"][0])):
                    lk["shows"].append(f"@{a['k']}")
            if lid in after_ids:
                prior = [a for a in acts if a["pos"] < lk["pos"]]
                lk["region"] = _region(ex, q, ctrl, cands, prior[-1] if prior else (acts[-1] if acts else None))
        looks[lid] = lk
    verifies = []
    for c in calls:
        v = c.get("verify")
        if v and not c["refused"]:
            lp = st["look_pos"].get(v.get("look"))
            verifies.append(dict(v, pos=tuple(lp) if lp else (c["k"], 2)))
    tools, like = _tools(ex, sess)
    views = [{"target": c["args"].get("target"), "side": c["args"].get("side") or "robot_side"}
             for c in calls if c["name"] == "view" and not c["refused"]]
    return {"steps": steps, "parts": parts, "plist": plist, "doc": doc, "calls": calls, "located": located,
            "points": points, "fresh_contacts": fresh_contacts, "looks": looks, "records": records,
            "verifies": verifies, "budget": _budget(sess), "light": _light(ex), "tools": tools, "like": like,
            "lift_listed": "lift" in tools and _lift_declared(ex), "candidates": cands,
            "surveyed": any(c["name"] == "survey" and not c["refused"] for c in calls), "views": views,
            "now_call": int(getattr(sess, "calls_used", 0) or (calls[-1]["call"] if calls else 0)),
            "loaded": _loaded(ex)}


# ---------------------------------------------------------------- the gate
def _words(text, n):
    w = str(text).split()
    return " ".join(w if len(w) <= n else w[:n])


def _note(facts):
    missing = [p for p in OPTIONAL if p not in facts["loaded"]]
    if not missing:
        return None
    return (f"{', '.join(missing)} not loaded here: no " + ("same-view records" if "witness" in missing else
                                                            "R3 verbs or points") + " can carry a done")


def _give_up(facts, untried):
    """give_up_early's why, or None when it does not fire (budget, exemptions, nothing of rules 1 to 7)."""
    b = facts["budget"]
    if b is None or not b["calls_total"] or not b["s_total"]:
        return None
    if not (b["calls_left"] > L.GIVE_UP_LEFT_FRAC * b["calls_total"] and
            b["s_left"] > L.GIVE_UP_LEFT_FRAC * b["s_total"]):
        return None
    if facts["light"] == "dark":
        return None
    recent = [c for c in facts["calls"] if c["call"] is not None and c["call"] > facts["now_call"] - L.EXEMPT_RECENT_CALLS]
    if any(c["refused"] in BLOCKING for c in recent):
        return None
    real = [it for it in untried if it["rule"] <= 7]
    if not real or fu.already_refused(facts, real):
        return None
    used = b["calls_total"] - b["calls_left"]
    return (f"{used} of {b['calls_total']} calls used ({b['calls_left']} left), {b['s_used']:.0f} of "
            f"{b['s_total']:g} s used; not tried yet: {fu.describe(real[0], facts)}")


def _answer(ex, facts, cand, untried, seq, success):
    st = state(ex)
    hard, check = cand["hard"], cand["check"]
    why = cand["why"]
    lead = "Not recorded" if hard else "Not recorded yet"
    said = _words(f"{lead}: {why}.", L.SAID_MAX_WORDS)
    if not said.endswith("."):
        said += "."
    if check == "seen_not_claimed":
        hint = "if the picture agrees, claim success true; or send the same claim again to end"
    elif hard:
        hint = "run untried[0], then claim again" if untried else "claim again once a look after the act shows it"
    elif success:
        hint = ("run untried[0], or send the same claim again: it is then recorded as not shown by the evidence"
                if untried else "send the same claim again: it is then recorded as not shown by the evidence")
    else:
        hint = "run untried[0], or send the same claim again to end" if untried else "send the same claim again to end"
    out = {"said": said, "not_recorded": True, "check": check, "hard": hard, "why": why,
           "untried": fu.public(untried), "hint": hint}
    note = _note(facts)
    if note:
        out["note"] = note
    while len(_json(out)) > L.BRIEF_MAX_BYTES and out["untried"]:
        out["untried"] = out["untried"][:-1]
    stage = (("after_act" if fe.acts(facts) else "before_act") if check == "give_up_early"
             else f"step {cand['step']}" if cand.get("step") else "episode")
    st["answers"].append({"seq": seq, "check": check, "hard": hard, "stage": stage})
    st["last"] = {"check": check, "hard": hard, "success": success, "k": len(st["calls"])}
    if hard:
        st["hard"] += 1
    if check == "unverified":
        st["s1_steps"].append(cand["step"])
    if check == "give_up_early":
        st["g1"][stage] = True
    if check == "seen_not_claimed":
        st["g2"] = True
    ex._journal("finish_gate", seq=seq, check=check, hard=hard, stage=stage, recorded=False,
                untried=[fu.describe(it, facts) for it in untried], rules=[it["rule"] for it in untried])
    return out


def _json(obj):
    return json.dumps(obj, separators=(",", ":"))


def _allowed(st, cand, repeat_soft, facts):
    if cand["hard"]:
        return True
    if repeat_soft:
        return False
    if cand["check"] == "unverified":
        return cand["step"] not in st["s1_steps"]
    if cand["check"] == "seen_not_claimed":
        return not st["g2"]
    if cand["check"] == "give_up_early":
        return not st["g1"]["after_act" if fe.acts(facts) else "before_act"]
    return True


def CLAIM_GATE(ex, args, session):
    """None (record the claim) or the answer (module text). Free: it moves nothing and charges nothing."""
    ex = _real(ex)
    st = state(ex)
    if not isinstance(args, dict) or not isinstance(args.get("success"), bool):
        return None                      # the session's own checks answer a claim without a success value
    success = args["success"]
    sess = _session(ex, session)
    seq = getattr(sess, "seq", None)
    facts = build_facts(ex, sess)
    ev = fe.evaluate(facts)
    cands = fe.checks(facts, success, ev)
    last = st["last"]
    # "sent again" = the same success value with no operator call since the soft answer; after a new call the
    # once-rules decide (so a give-up after the first device act is answered once more)
    repeat_soft = (last is not None and not last["hard"] and last["success"] == success
                   and last.get("k") == len(st["calls"]))
    if success and st["hard"] >= L.HARD_CAP:
        would = cands[0] if cands else None
        return _record(ex, facts, seq, success, overridden=would is not None, would=would)
    for cand in cands:
        if not _allowed(st, cand, repeat_soft, facts):
            continue
        untried = fu.rank(facts, cand["check"], cand.get("ev"))
        if cand["check"] == "give_up_early":
            why = _give_up(facts, untried)
            if why is None:
                continue
            cand = dict(cand, why=why)
        return _answer(ex, facts, cand, untried, seq, success)
    # a success claim is recorded plainly only when no check fires (every step's goal state was seen or sensed
    # after its last act); one recorded past a soft answer says so: gate_overridden, with the check it passed
    would = cands[0] if (success and cands) else None
    return _record(ex, facts, seq, success, overridden=would is not None, would=would)


def _record(ex, facts, seq, success, overridden, would):
    st = state(ex)
    st["recorded"] = {"seq": seq, "success": success, "gate_overridden": bool(overridden),
                      "would": None if would is None else would["check"], "hard_answers": st["hard"]}
    ex._journal("finish_gate", seq=seq, recorded=True, success=success, gate_overridden=bool(overridden),
                would=st["recorded"]["would"], hard_answers=st["hard"])
    return None


def claim_fields(ex):
    """What C2 adds to a recorded claim's record: {"gate_overridden": bool}."""
    rec = state(_real(ex)).get("recorded") or {}
    return {"gate_overridden": bool(rec.get("gate_overridden"))}


# ---------------------------------------------------------------- robot://progress
def progress(ex, session=None, **_kw):
    ex = _real(ex)
    facts = build_facts(ex, session)
    rows = []
    ev = fe.evaluate(facts)
    fires = {}                               # a step reads done only when no check of the gate fires for it
    for c in fe.checks(facts, True, ev):
        fires.setdefault(c["step"], c)
    for e in ev:
        s, d = e["step"], e["done"]
        c = fires.get(e["index"])
        evid = "contradicted" if e["contradicted"] else ("done" if d["done"] and c is None else "missing")
        rows.append({"step": e["index"], "state": s["state"], "deciding_part": s["deciding"],
                     "control_part": s["control"], "class": s["done_class"],
                     "located": [x["id"] for x in facts["located"] if x["part"] in (s["deciding"], s["control"],
                                                                                    s.get("mark")) and x["part"]],
                     "acts": [a["seq"] if a["seq"] is not None else a["call"] for a in e["on"]],
                     "evidence": evid,
                     "missing": e["why"] if e["contradicted"] else ("" if evid == "done" else
                                                                    (c["why"] if c else d["missing"]))})
    st = state(ex)
    done = sum(1 for r in rows if r["evidence"] == "done")
    out = {"said": f"{done} of {len(rows)} goal steps show done evidence; {len(st['answers'])} claim answers so far.",
           "steps": rows, "claim_answers": len(st["answers"]), "hard_answers": st["hard"]}
    note = _note(facts)
    if note:
        out["note"] = note
    return out


RESOURCES = {"robot://progress": (progress, "Each goal step of the task: its deciding and control parts, the "
                                            "located points and acts on it, and whether the evidence shows it "
                                            "done, contradicted or missing (and what is missing). A read: free.")}

__all__ = ["CLAIM_GATE", "FILTER", "ID", "RESOURCES", "build_facts", "claim_fields", "note_call", "progress",
           "setup", "state"]


def _first_param(fn):
    try:
        return next(iter(inspect.signature(fn).parameters), None)
    except (TypeError, ValueError):
        return None


def _limit(ex, name, *args):
    """The round three limit NAME as this session derives it: r3_limits' lower case function of that name called
    with the session's bounds and args once it exists (device_box_near_mm(B, sigma_mm)); until then the constant
    (the Review stage removes this branch)."""
    fn = getattr(L, name.lower(), None)
    if callable(fn) and _first_param(fn) in ("B", "bounds"):
        return float(fn(bounds.of(ex), *args))
    return float(getattr(L, name))
