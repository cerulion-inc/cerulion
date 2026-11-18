# SPDX-License-Identifier: AGPL-3.0-only
"""I22 THE PERSON IS A GATED, COUNTED RESOURCE: constraints in, asks out (SPEC-R2.md section 3, NEXT-IMPROVEMENTS.md).

What this package serves:
- robot://task gains "constraints" (from ex.context["task"]["constraints"]): {kind: no_move, object, tolerance_mm},
  {kind: no_grip, part}, {kind: no_markers}; other kinds are served as written and not checked.
- ask_person(kind, question, target, evidence), gated before the ask leaves:
    device_info     always allowed;
    change_world    needs, in this epoch, a failing reach_check of the target from every allowed side with every
                    tool surface (a failing reach_check with no approach covers every side for the surface it
                    checked; one that names no surface checks the tip when reach_check takes a surface, and stands
                    for every surface only when it cannot name one), of the target's own point (within its uncertainty, or
                    within the distance that check was out of reach by), and is refused outright when a no_move
                    constraint names the target; a reachable check from an allowed side refuses it;
    hardware_check  needs a self_check in this epoch (refused, saying so, when no self_check exists here) and is
                    refused for a part already asked about.
  A refusal lists the missing checks. An allowed ask calls ex.context["person"](kind, question, evidence) and
  returns its answer; without one it answers {"answered": false, "why": "no person is available"}. Asks, refused
  asks and kinds are counted in ex.state["person"] and served at robot://asks.
- Pre hook on gripper (closing), grasp, pinch and press(mode="pinch"): refused when a no_grip part's located point
  lies between the jaws (the swept region from the body model; for pinch, the region where its placing move puts
  the jaws, and the pinch point itself); a robot without a gripper is never refused.
- Post hook on press, move_tip, touch (and impulse and approach when a package serves them): when the stroke or its
  contact came within its extent plus stroke error and uncertainty of a no_move object's located point, that object's region is compared between the
  newest lit standard look before the work and the newest lit standard look after it (the push's own, if it took
  one; else the next one taken; a dark one is waited past) and constraint_breached {object, moved_mm_est} is added
  past tolerance, or with moved_mm_est null and suspected true when the object's face is found nowhere near (moved
  far, turned over or hidden). A look where the object's edge cannot be told from its surroundings (dim, or a plain
  run of colour) gives checked false with why, never a number.
- check_cannot(ex, claim) -> {"ok", "missing"}: a "cannot" claim must give a reason and cite call ids that exist in
  the journal (the core lane wires it into claim).
No robot is named here and nothing reads simulator ground truth.
"""

import traceback
import weakref

import numpy as np

from rcpx import surfaces
from rcpx.context import jsonable, refusal
from rcpx.r2 import person_gates as gates
from rcpx.r2 import person_bounds as derived
from rcpx.r2.person_look import Jaws, inside, region_motion, extent_px, comparison_error_px

ID = "I22"
MM = 1000.0
GRIP_NAMES = ("gripper", "grasp", "pinch", "press")
PUSH_NAMES = ("press", "move_tip", "touch", "impulse", "approach", "push", "lift")   # the last four: when served
VERB_NAMES = ("push", "lift")  # whole-step verbs: their own moves pass these hooks as runtime gates (core gates)
KINDS = ("device_info", "change_world", "hardware_check")
QUESTION_MAX = 2000
LOG_SHOWN = 50
ENFORCED = {
    "no_move": f"checked: after {', '.join(PUSH_NAMES)} within its extent plus lateral error and uncertainty, its region is "
               "compared between standard looks (a look where its edge cannot be told apart is not measured)",
    "no_grip": "checked: closing the gripper, grasp or a pinch is refused while its located point is between the "
               "jaws",
    "no_markers": "a change_world or hardware_check ask about a marker or tag is refused; nothing in this runtime "
                  "places one",
}
_PRIV = weakref.WeakKeyDictionary()


# ------------------------------------------------------------------------------------------------ state
def _fresh():
    return {"asks": 0, "refused": 0, "unanswered": 0, "attempts": 0,
            "kinds": {k: {"asked": 0, "refused": 0} for k in KINDS + ("other",)},
            "log": [], "asked_parts": [], "breaches": [], "checks": [], "pending": {}, "grip_refusals": 0,
            "unlocated_noted": []}


def _state(ex):
    st = ex.state.get("person")
    if not isinstance(st, dict) or "asks" not in st:
        st = ex.state["person"] = _fresh()
    return st


def _priv(ex):
    p = _PRIV.get(ex)
    if p is None:
        p = _PRIV[ex] = {"jaws": None, "push": None, "epoch_seen": getattr(ex, "epoch", 0)}
    return p


def _mark_epoch(ex):
    """Journal the epoch when it changed, so later records are read as being in the new epoch."""
    p = _priv(ex)
    if getattr(ex, "epoch", 0) != p["epoch_seen"]:
        p["epoch_seen"] = ex.epoch
        ex._journal("person_epoch", epoch=ex.epoch)


def constraints(ex, content=None):
    """The task's constraints, normalised: kind lower case; no_move gets object and tolerance_mm; no_grip part."""
    task = (getattr(ex, "context", None) or {}).get("task")
    raw = task.get("constraints") if isinstance(task, dict) else None
    if raw is None and isinstance(content, dict):
        raw = content.get("constraints")
    out = []
    for c in raw if isinstance(raw, (list, tuple)) else []:
        if not isinstance(c, dict) or not str(c.get("kind") or "").strip():
            continue
        e = jsonable(dict(c))
        kind = str(e["kind"]).strip().lower()
        e["kind"] = kind
        if kind == "no_move":
            e["object"] = str(e.get("object") or "").strip()
            tol = e.get("tolerance_mm")
            if isinstance(tol, (int, float)) and not isinstance(tol, bool) and tol > 0:
                e["tolerance_mm"] = float(tol)
            else:
                pose = ex._sense()
                depth = abs(float(ex.model.fk(pose.q)["camera"][0][2]))
                value, source, evidence = derived.tolerance_m(derived.of(ex), depth, ex.model.f_px, comparison_error_px(ex))
                e["tolerance_mm"], e["tolerance_source"] = value * MM, source
                e["tolerance_evidence"] = evidence
        elif kind == "no_grip":
            e["part"] = str(e.get("part") or e.get("object") or "").strip()
        e["enforced"] = ENFORCED.get(kind, "served as written; this runtime does not check it")
        out.append(e)
    return out


# ------------------------------------------------------------------------------------------------ hooks
def _std_ids(ex):
    looks = getattr(ex, "_looks", None) or {}
    return [k for k, r in looks.items() if isinstance(r, dict) and r.get("img") is not None]


def _seg_dist(p, a, b):
    ab = b - a
    t = 0.0 if float(ab @ ab) < 1e-12 else float(np.clip((p - a) @ ab / float(ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _contact_point(ex, name, result):
    c = result.get("contact")
    if isinstance(c, dict) and c.get("point_mm") is not None:
        return np.asarray(c["point_mm"], dtype=float)
    if name == "touch" and c is True and result.get("point_mm") is not None:
        return np.asarray(result["point_mm"], dtype=float)
    pk = result.get("peak_force_n")
    if name == "press" and isinstance(pk, (int, float)) and pk >= derived.of(ex).f_thr() and result.get("tip_mm"):
        return np.asarray(result["tip_mm"], dtype=float)
    return None


def _record_breach(ex, chk):
    st = _state(ex)
    b = {"object": chk["object"], "moved_mm_est": chk.get("moved_mm_est"), "tolerance_mm": chk["tolerance_mm"],
         "looks": chk.get("looks"), "suspected": chk.get("moved_mm_est") is None, "call": getattr(ex, "_calls", None)}
    st["breaches"].append(b)
    ex.note_event("constraint_breached", **b)
    try:
        ex.world.event("constraint_breached", data=b, source="person")
        if chk.get("entry"):
            ex.world.update(chk["entry"], data={"possibly_moved": True, "moved_mm_est": b["moved_mm_est"]})
    except Exception:  # the world bookkeeping must not lose the breach itself; the failure is journaled
        ex._journal("r2_problem", package="person", where="breach world update", trace=traceback.format_exc())


def _resolve(ex):
    """Compare every pending no_move object whose before-look has a later standard look. Returns the checks."""
    st, ids, looks, out = _state(ex), _std_ids(ex), getattr(ex, "_looks", None) or {}, []
    for key, pend in list(st["pending"].items()):
        head = {"object": pend["object"], "tolerance_mm": pend["tolerance_mm"], "near_mm": pend["near_mm"],
                "entry": pend["entry"]}
        before = pend["before"]
        if before is None or before not in ids:
            out.append({**head, "checked": False, "why": f"no standard look was taken before the work near "
                        f"{pend['object']}, so there is nothing to compare with",
                        "hint": "take standard_look(set_reference=true) before working near it"})
            del st["pending"][key]
            continue
        # a dark later look shows nothing to compare: the check waits for the next lit standard look
        later = [k for k in ids[ids.index(before) + 1:] if (looks.get(k) or {}).get("can_see") != "dark"]
        if not later:
            continue
        r = region_motion(ex.model, looks[before], looks[later[-1]], pend["point_mm"], before, later[-1])
        if pend.get("tolerance_source") == "measurable" and r.get("match_error_px") is not None:
            depth = ex.model.project(looks[before]["q"], np.asarray(pend["point_mm"]) / MM)[2]
            value = derived.tolerance_m(derived.of(ex), depth, ex.model.f_px, r["match_error_px"])[0]
            pend["tolerance_mm"] = head["tolerance_mm"] = value * MM
        chk = {**head, **r}
        est = r.get("moved_mm_est")
        chk["breached"] = bool((est is not None and est > pend["tolerance_mm"]) or r.get("suspect"))
        if chk["breached"]:
            _record_breach(ex, chk)
        st["checks"].append({k: chk.get(k) for k in ("object", "checked", "moved_mm_est", "breached", "looks")})
        out.append(chk)
        del st["pending"][key]
    return out


def _attach(result, checks):
    out = dict(result)
    out["constraint_checks"] = list(result.get("constraint_checks") or []) + checks
    hits = [c for c in checks if c.get("breached")]
    if hits:
        worst = max(hits, key=lambda c: float("inf") if c.get("moved_mm_est") is None else c["moved_mm_est"])
        out["constraint_breached"] = {"object": worst["object"], "moved_mm_est": worst.get("moved_mm_est"),
                                      "tolerance_mm": worst["tolerance_mm"], "looks": worst.get("looks"),
                                      "suspected": worst.get("moved_mm_est") is None}
    return out


def setup(ex):
    _state(ex)
    p = _priv(ex)
    try:
        p["jaws"] = Jaws(ex.model)
    except Exception:   # no_grip cannot be checked; every other hook below still registers. Journaled, and
        p["jaws"] = None    # robot://asks says so (grip_check)
        _state(ex)["grip_check"] = "unavailable: the gripper geometry could not be read from the body model"
        ex._journal("r2_problem", package="person", where="setup jaws", trace=traceback.format_exc())

    def pre_epoch(name, kwargs):
        _mark_epoch(ex)

    def pre_push(name, kwargs):
        held = p["push"]
        if held is not None and held.get("by") in VERB_NAMES and name not in VERB_NAMES:
            return None   # a move inside a whole-step verb (a runtime gate): the verb's own start stands
        looks = getattr(ex, "_looks", None) or {}
        lit = [k for k in _std_ids(ex) if (looks.get(k) or {}).get("can_see") != "dark"]   # a dark one shows nothing
        try:
            tip = ex.model.fk_tip(np.asarray(ex.backend.read()["q"], dtype=float))[0] * MM
        except Exception:   # the stroke's start is unknown: only its end and contact are compared
            tip = None
            ex._journal("r2_problem", package="person", where="push start tip", trace=traceback.format_exc())
        p["push"] = {"before": lit[-1] if lit else None, "tip_start": tip, "by": name, "lateral_error_m": 0}

    def pre_grip(name, kwargs):
        if name == "press" and kwargs.get("mode") != "pinch":
            return None
        jaws = p["jaws"]
        parts = [c["part"] for c in constraints(ex) if c["kind"] == "no_grip" and c.get("part")]
        if jaws is None or not jaws.ok or not parts:
            return None
        rd = ex.backend.read()
        g_now = jaws.present_q(rd.get("gripper"))
        place = None
        try:
            if name == "gripper":
                wf = kwargs.get("width_frac")
                if wf == "open":
                    return None
                frac = 0.0 if wf in (None, "close") else min(1.0, max(0.0, float(wf)))
            elif name == "pinch":   # the jaw goes onto part_point_mm first, then close_frac of the opening shuts
                cf = kwargs.get("close_frac", 0.5)
                frac = jaws.opening(g_now) * (1.0 - min(1.0, max(0.0, 0.5 if cf is None else float(cf))))
                place = np.asarray(kwargs.get("part_point_mm"), dtype=float).reshape(3) / MM
            else:                   # grasp and press(mode=pinch) close on whatever is between the jaws
                frac = 0.0
        except (TypeError, ValueError):
            return None                                   # the primitive refuses its own bad argument
        g_to = jaws.frac_to_q(frac)
        if jaws.opening(g_to) >= max(0.0, jaws.opening(g_now) - derived.jaw_step_frac(derived.of(ex), jaws)):
            return None                                   # opening or holding: nothing is closed on
        eqs, pts = jaws.sweep(rd["q"], g_now, g_to)
        shift = np.zeros(3)
        if place is not None and np.all(np.isfinite(place)):
            near = jaws.points(rd["q"], g_now)
            shift = place - near[int(np.argmin(np.linalg.norm(near - place, axis=1)))]
        for part in parts:
            for e in gates.located_points(ex.world, part):
                err = (e.get("data") or {}).get("error_mm")
                err = derived.sigma_m(derived.of(ex), e)
                pt = np.asarray(e["point_mm"], dtype=float) / MM
                margin = derived.grip_margin_m(derived.of(ex), err, derived.jaw_error_m(derived.of(ex), jaws, pts))
                on_it = place is not None and float(np.linalg.norm(pt - place)) <= margin
                if on_it or inside(eqs, pts, pt - shift, margin):
                    _state(ex)["grip_refusals"] += 1
                    where = "is where the pinch puts the jaw" if on_it else "lies between the jaws"
                    out = refusal("no_grip", f"the task forbids gripping {part}, and its located point "
                                  f"{gates._fmt(e['point_mm'])} mm ({e['id']}) {where}: closing would grip it",
                                  "open, move the hand so the part is outside the jaws, or grip something else; "
                                  "robot://task lists the constraints")
                    out["refused"].update(part=part, point_mm=e["point_mm"])
                    return out
        return None

    def post_grip(name, kwargs, result):
        if name == "press" or not isinstance(result, dict) or "refused" in result:
            return None
        lost = [c["part"] for c in constraints(ex) if c["kind"] == "no_grip" and c.get("part")
                and not gates.located_points(ex.world, c["part"])]
        if not lost or not (p["jaws"] and p["jaws"].ok):
            return None
        return {**result, "constraint_note": f"no located point for {', '.join(lost)} (no_grip): whether it was "
                "between the jaws could not be checked; triangulate it with that label"}

    def post_push(name, kwargs, result):
        push, p["push"] = p["push"], None
        if not isinstance(result, dict) or "refused" in result or push is None:
            return None
        nm = [c for c in constraints(ex) if c["kind"] == "no_move" and c.get("object")]
        if not nm:
            return None
        st, notes = _state(ex), []
        contact = _contact_point(ex, name, result)
        start = push["tip_start"]
        end = np.asarray(result["tip_mm"], dtype=float) if result.get("tip_mm") is not None else start
        if name in VERB_NAMES and result.get("tip_mm") is None:   # a verb's result states no tip: read where it is
            try:
                end = ex.model.fk_tip(np.asarray(ex.backend.read()["q"], dtype=float))[0] * MM
            except Exception:
                ex._journal("r2_problem", package="person", where="push end tip", trace=traceback.format_exc())
        for c in nm:
            pts = gates.located_points(ex.world, c["object"])
            if not pts:
                if gates.norm(c["object"]) not in st["unlocated_noted"]:
                    st["unlocated_noted"].append(gates.norm(c["object"]))
                    notes.append({"object": c["object"], "checked": False, "why": f"no located point for "
                                  f"{c['object']} (no_move): work near it cannot be checked",
                                  "hint": "triangulate it with that label"})
                continue
            pt = np.asarray(pts[-1]["point_mm"], dtype=float)
            dists = [float(np.linalg.norm(pt - contact))] if contact is not None else []
            if start is not None and end is not None:
                dists.append(_seg_dist(pt, np.asarray(start, dtype=float), np.asarray(end, dtype=float)))
            B = derived.of(ex)
            pose = ex._sense()
            reported = result.get("tip_speed_mm_s")
            if reported is None and isinstance(result.get("contact"), dict):
                reported = result["contact"].get("tip_speed_mm_s")
            speed = float(reported) / MM if reported is not None else 0.0
            thing = B.thing(pts[-1]["id"])
            before = (getattr(ex, "_looks", None) or {}).get(push["before"])
            seen_extent = 0
            if before is not None:
                u, v, depth = ex.model.project(before["q"], pt / MM)
                img = before["img"]
                if depth > 0 and 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
                    pixels = extent_px(img, u, v, 0, max(img.shape[:2]))
                    seen_extent = 2.0 * pixels * depth / ex.model.f_px  # full extent from measured half extent
            target = {"extent": max(seen_extent, derived.extent_m(B, thing)),
                      "sigma_p": derived.sigma_m(B, pts[-1])}
            near_m = derived.near_m(B, target, max(push["lateral_error_m"], B.e_cross(speed)))
            if not dists or min(dists) / MM > near_m:
                continue
            key = gates.norm(c["object"])
            if key not in st["pending"]:
                st["pending"][key] = {"object": c["object"], "before": push["before"], "point_mm": pt.tolist(),
                                      "entry": pts[-1]["id"], "tolerance_mm": c["tolerance_mm"],
                                      "tolerance_source": c.get("tolerance_source"),
                                      "near_mm": round(min(dists), 1), "by": name, "contact": contact is not None}
        checks = notes + _resolve(ex)
        for pend in st["pending"].values():
            checks.append({"object": pend["object"], "checked": False, "pending": True,
                           "why": f"the arm worked within {pend['near_mm']:g} mm of {pend['object']}; whether it "
                                  "moved is checked at the next standard look",
                           "hint": "take standard_look() to check it"})
        return _attach(result, checks) if checks else None

    def post_resolve(name, kwargs, result):
        if name in PUSH_NAMES or not isinstance(result, dict) or "refused" in result:
            return None
        if not _state(ex)["pending"]:
            return None
        checks = _resolve(ex)
        return _attach(result, checks) if checks else None

    def push_tick(s):
        active = p["push"]
        if active is not None:
            v = float(np.linalg.norm(ex.model.jac(s.q)[:3] @ s.qd))
            active["lateral_error_m"] = max(active["lateral_error_m"], derived.of(ex).e_cross(v))
        return None

    ex.add_tick(push_tick)
    ex.add_pre(pre_epoch)
    ex.add_pre(pre_grip, names=list(GRIP_NAMES))
    ex.add_pre(pre_push, names=list(PUSH_NAMES))
    ex.add_post(post_grip, names=["gripper", "grasp", "pinch"])
    ex.add_post(post_push, names=list(PUSH_NAMES))
    ex.add_post(post_resolve)


# ------------------------------------------------------------------------------------------------ ask_person
def _count(ex, kind, refused, check=None, target=None, question="", ask_id=None, answer=None, answered=None):
    st = _state(ex)
    k = kind if kind in KINDS else "other"
    if refused:
        st["refused"] += 1
        st["kinds"][k]["refused"] += 1
    else:
        st["asks"] += 1
        st["kinds"][k]["asked"] += 1
        if not answered:
            st["unanswered"] += 1
    st["log"].append(jsonable({"ask_id": ask_id, "kind": kind, "target": target, "question": question[:200],
                               "refused": check if refused else None, "answered": answered, "answer": answer,
                               "epoch": getattr(ex, "epoch", 0)}))
    ex.note_event("ask_person", ask_kind=kind, allowed=not refused, check=check, target=target, ask_id=ask_id)


def ask_person(ex, session, kind=None, question=None, evidence=None, target=None, **extra):
    """The ask_person tool (served by the session)."""
    st = _state(ex)
    st["attempts"] += 1
    ask_id = f"A{st['attempts']}"
    kind_s = str(kind or "").strip().lower()
    q = str(question or "").strip()[:QUESTION_MAX]
    tgt = None if target is None else str(target).strip()[:200]

    def no(r):
        _count(ex, kind_s, True, r["refused"]["check"], tgt, q, ask_id)
        r["refused"]["ask_id"] = ask_id
        return r

    if extra:
        return no(refusal("args", f"unknown argument {sorted(extra)[0]!r}",
                          "ask_person takes kind, question, target and evidence"))
    if kind_s not in KINDS:
        return no(refusal("kind", f"unknown ask kind {str(kind)[:40]!r}", "device_info, change_world or "
                          "hardware_check; confirming a device state is the robot's job: look, then verify"))
    if not q:
        return no(refusal("question", "the ask has no question", "say what you want to know or have done"))
    ev = gates.as_list(evidence)[:40]
    _mark_epoch(ex)
    cons = constraints(ex)
    calls = gates.journal_calls(ex, session)
    if calls is None and kind_s != "device_info":
        return no(refusal("journal", "this session keeps no journal, so the checks behind the ask cannot be read",
                          "run inside an episode"))
    found, unknown = gates.resolve_evidence(ev, calls or [])
    if unknown and kind_s != "device_info":   # device_info is always allowed: its unknown ids are only reported
        out = refusal("evidence", f"no call in this session's journal for {', '.join(repr(u) for u in unknown)}",
                      "cite the seq of a response, or an id a call returned (look_id, contact id, ref)")
        out["refused"]["missing"] = [{"check": "evidence", "id": u} for u in unknown]
        return no(out)
    gate = {"ok": True}
    if kind_s != "device_info":
        bad = gates.marker_gate(cons, q)
        if bad is not None:
            return no(bad)
    if kind_s == "change_world":
        gate = gates.change_world_gate(ex, calls, tgt, found, cons, gates.takes_surface(session))
    elif kind_s == "hardware_check":
        gate = gates.hardware_gate(ex, calls, tgt, st["asked_parts"])
    if "refused" in gate:
        return no(gate)
    person = (getattr(ex, "context", None) or {}).get("person")
    ids = jsonable([e for e, _ in found])
    out = {"asked": True, "ask_id": ask_id, "kind": kind_s, "target": tgt,
           "evidence": [{"id": jsonable(e), "seq": c.get("req"), "name": c.get("name")} for e, c in found],
           "checks": {k: v for k, v in gate.items() if k != "ok"}}
    if unknown:
        out["evidence_unknown"] = jsonable(unknown)
    if callable(person):
        try:
            ans = person(kind_s, q, ids)
            out.update(answered=True, answer=ans if isinstance(ans, str) else jsonable(ans))
        except Exception as exc:
            out.update(answered=False, why=f"the person could not answer: {type(exc).__name__}: {exc}")
    else:
        out.update(answered=False, why="no person is available")
    if kind_s == "hardware_check":
        st["asked_parts"].append(gates.norm(tgt))
    _count(ex, kind_s, False, None, tgt, q, ask_id, out.get("answer"), out["answered"])
    out["asks_so_far"] = st["asks"]
    return out


# ------------------------------------------------------------------------------------------------ claim path
def check_cannot(ex, claim, session=None):
    """{"ok", "missing", "found"} for a claim's cannot {reason, evidence}: a reason, and evidence that cites
    call ids found in the journal. A claim without cannot is ok."""
    c = claim.get("cannot") if isinstance(claim, dict) and "cannot" in claim else claim
    if c is None or (isinstance(claim, dict) and "cannot" not in claim and not {"reason", "evidence"} & set(claim)):
        return {"ok": True, "missing": [], "found": []}
    if not isinstance(c, dict):
        return {"ok": False, "missing": ["cannot must be an object {reason, evidence}"], "found": []}
    missing = []
    if not str(c.get("reason") or "").strip():
        missing.append("reason: say why the task cannot be done")
    ev = gates.as_list(c.get("evidence"))
    if not ev:
        missing.append("evidence: cite the call ids (each response's seq) that show it cannot be done")
    calls = gates.journal_calls(ex, session)
    found = []
    if calls is None:
        missing.append("journal: this session keeps no journal to check the evidence against")
    elif ev:
        hits, unknown = gates.resolve_evidence(ev, calls)
        found = [{"id": jsonable(e), "seq": h.get("req"), "name": h.get("name"),
                  "refused": (h.get("result") or {}).get("refused", {}).get("check")} for e, h in hits]
        missing += [f"evidence {u!r}: no such call in the journal" for u in unknown]
    return {"ok": not missing, "missing": missing, "found": found}


# ------------------------------------------------------------------------------------------------ resources
def _asks_resource(ex, session):
    st = _state(ex)
    return {"asks": st["asks"], "refused": st["refused"], "unanswered": st["unanswered"],
            "by_kind": st["kinds"], "log": st["log"][-LOG_SHOWN:], "shown": min(len(st["log"]), LOG_SHOWN),
            "constraint_breaches": st["breaches"], "constraint_checks": st["checks"][-LOG_SHOWN:],
            "pending_checks": list(st["pending"].values()),
            "constraints": constraints(ex), "grip_check": st.get("grip_check", "available"),
            "note": "every ask is counted; refused asks name the check that refused them"}


def _task_patch(ex, session, content):
    cons = constraints(ex, content)
    base = dict(content) if isinstance(content, dict) else {"task": content}
    base["constraints"] = cons
    return base


TOOLS = [
    surfaces._tool(
        "ask_person",
        "Ask the person at the robot one question (counted, see robot://asks): device_info (what the device's "
        "manual says) any time; change_world (move or change something in the scene) only after reach_check of "
        "the target's own point failed from every allowed side with every tool surface in this epoch; "
        "hardware_check (inspect a robot part) only after self_check, once per part; a refused ask lists the "
        "checks still missing.",
        surfaces._obj({
            "kind": surfaces._str("what the ask is for", list(KINDS)),
            "question": surfaces._str("the question in plain words"),
            "target": surfaces._str("what the ask is about (needed for change_world and hardware_check): a label "
                                    "you located, a device or part id from robot://devices, or a robot part such "
                                    "as a joint name"),
            "evidence": {"type": "array", "maxItems": 40,
                         "items": {"anyOf": [{"type": "string"}, {"type": "integer", "minimum": 0}]},
                         "description": "the calls that back the ask: the seq of each response, or an id a call "
                                        "returned (look_id, contact id, ref)"}},
            ["kind", "question"]),
        {"kind": "change_world", "question": "The machine is out of reach from every side; could it be moved "
         "closer?", "target": "machine", "evidence": []},
        "ask the person to move the machine, citing the failed reach_check from this session"),
]
RESOURCES = {"robot://asks": (_asks_resource, "Questions asked of the person so far: counts by kind, refused asks "
                              "with the check that refused them, answers, and no_move checks and breaches.")}
RESOURCE_PATCH = {"robot://task": _task_patch}
HANDLERS = {"ask_person": ask_person}
PRACTICES = ("robot://task states the constraints: a no_move object is never moved, a no_grip part never "
             "gripped. ask_person is a counted call. A claim of cannot cites the response seq of each call that "
             "shows it.")
