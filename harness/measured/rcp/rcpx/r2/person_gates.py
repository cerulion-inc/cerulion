# SPDX-License-Identifier: AGPL-3.0-only
"""Person package, part one: the session's journal as evidence, and the gates an ask must pass.

Read by rcpx/r2/person.py. Nothing here moves the arm or reads simulator ground truth: every fact comes from the
journal (the executor's "primitive" records and the session's "call" records, which share one file in an episode),
the World, the task constraints and the device knowledge the runner put in ex.context.

Call ids. A call id is the seq of the response the operator got (the session journals it as "req"); an id a call
returned (look_id, a press ref, a contact or world entry id, an ask_id) names the call that returned it.

Epochs. A record belongs to the epoch named by the latest "epoch" field before it in the journal (every event the
executor journals carries the epoch, and person.py journals a marker when it sees the epoch change), so a check
counts "in this epoch" only when no epoch change was journaled between it and now.
"""

import math
from rcpx.r2 import person_bounds as derived
import re

import numpy as np

from rcpx.context import read_journal, refusal
from rcpx.surfaces import DIRECTION_NAMES

                                # one further off counts only when the point it checked was out of reach by more than
                                # its distance from the target (then no point that close to it is reachable)
SIDE_COS = math.cos(math.radians(45.0))  # geometry: six named approach sides
TIP = "tip"                     # the surface a reach_check checks when it names none and the tool takes a surface
SIDES = ("above", "front", "left", "right", "behind", "below")
DEFAULT_SIDES = ("above", "front", "left", "right", "behind")   # "below" only when the device sheet names it
ID_KEYS = ("look_id", "ref", "id", "claim_id", "ask_id")
_SIDE_RX = {
    "above": r"from (?:the )?(?:above|top|over)",
    "below": r"from (?:the )?(?:below|under|underneath|bottom)",
    "front": r"from (?:the )?front",
    "behind": r"from (?:the )?(?:behind|back|rear)",
    "left": r"from (?:the )?left",
    "right": r"from (?:the )?right",
    "sides": r"from (?:the |either )?sides?\b",
}
MARKER_WORDS = ("marker", "tag", "sticker", "fiducial", "aruco", "apriltag", "qr code")
# whole words only (plurals and "tagged" too): "voltage", "stage" or "cottage" are not an ask for a tag
_MARKER_RX = re.compile(r"\b(?:" + "|".join(re.escape(w) for w in MARKER_WORDS) + r")(?:s|es|ged|ging)?\b")


# ------------------------------------------------------------------------------------------------ the journal
def journal_paths(ex, session):
    out = []
    for owner in (session, ex):
        j = getattr(owner, "journal", None) if owner is not None else None
        p = getattr(j, "path", None)
        if p is not None and str(p) not in [str(x) for x in out]:
            out.append(p)
    return out


def journal_calls(ex, session):
    """Every operator call and every journaled primitive in order: {req, name, args, sent, result, epoch}.
    None when the session keeps no journal. A primitive is joined to the session's call record that follows it,
    so a call carries the operator's arguments as sent (sent) and its final result (after post hooks)."""
    paths = journal_paths(ex, session)
    if not paths:
        return None
    records = []
    for p in paths:                                   # two files only when session and executor log apart
        records.extend(read_journal(p))
    epoch, out, pending = 0, [], []
    for r in records:
        if isinstance(r.get("epoch"), int) and not isinstance(r.get("epoch"), bool):
            epoch = r["epoch"]
        kind = r.get("kind")
        if kind == "primitive":
            e = {"req": None, "name": r.get("name"), "args": r.get("args") or {}, "sent": None,
                 "result": r.get("result") if isinstance(r.get("result"), dict) else {}, "epoch": epoch,
                 "by": r.get("by")}
            out.append(e)
            pending.append(e)
        elif kind == "call":
            name = r.get("name")
            match = next((e for e in reversed(pending) if e["name"] == name), None)
            result = r.get("result") if isinstance(r.get("result"), dict) else {}
            if match is None:
                out.append({"req": r.get("req"), "name": name, "args": r.get("args") or {}, "sent": r.get("args"),
                            "result": result, "epoch": epoch, "by": "session"})
            else:
                match.update(req=r.get("req"), sent=r.get("args") if isinstance(r.get("args"), dict) else {},
                             result=result or match["result"])
            pending = []
    return out


def _ids_in(result):
    found = set()
    if not isinstance(result, dict):
        return found
    for k in ID_KEYS:
        v = result.get(k)
        if isinstance(v, (str, int)) and not isinstance(v, bool):
            found.add(str(v))
    c = result.get("contact")
    if isinstance(c, dict) and isinstance(c.get("id"), str):
        found.add(c["id"])
    # a look taken inside another call (look_after, wait_for_light, survey) is cited by its look id too
    inner = [result.get("look")] + list(result.get("looks") or [] if isinstance(result.get("looks"), list) else [])
    for lk in inner:
        if isinstance(lk, dict) and isinstance(lk.get("look_id"), str):
            found.add(lk["look_id"])
    return found


def _as_req(ev):
    if isinstance(ev, bool):
        return None
    if isinstance(ev, int):
        return ev
    if isinstance(ev, float) and ev.is_integer():
        return int(ev)
    if isinstance(ev, str):
        m = re.fullmatch(r"\s*(?:#|seq\s*|req\s*|call\s*)?(\d{1,9})\s*", ev, re.IGNORECASE)
        if m:
            return int(m.group(1))
    return None


def resolve_evidence(evidence, calls):
    """(found [(id, call)], unknown [id]) for a list of call ids."""
    found, unknown = [], []
    for ev in evidence:
        req = _as_req(ev)
        hit = None
        if req is not None:
            hit = next((c for c in calls if c["req"] == req), None)
        if hit is None and isinstance(ev, str) and ev.strip():
            key = ev.strip()
            hit = next((c for c in reversed(calls) if key in _ids_in(c["result"])), None)
        if hit is None:
            unknown.append(ev)
        else:
            found.append((ev, hit))
    return found, unknown


def as_list(evidence):
    if evidence is None:
        return []
    if isinstance(evidence, (list, tuple)):
        return list(evidence)
    return [evidence]


def call_name(c):
    return f"#{c['req']}" if c.get("req") is not None else f"a {c['name']} run by the runtime"


# ------------------------------------------------------------------------------------------------ the target
def norm(text):
    return str(text or "").strip().lower()


def located_points(world, label, include_stale=True):
    """World entries naming `label` (by label, or data part, device or object), newest last."""
    want = norm(label)
    out = []
    if not want:
        return out
    for e in world.query(include_stale=include_stale):
        if e.get("point_mm") is None or e.get("kind") not in ("located", "device", "contact"):
            continue
        data = e.get("data") or {}
        names = {norm(e.get("label"))} | {norm(data.get(k)) for k in ("part", "device", "object")}
        if want in names:
            out.append(e)
    return out


def knowledge_parts(ex, target):
    """Parts of the device knowledge (ex.context["devices"]) that the target names: a device id (all its parts)
    or a part name."""
    doc = (ex.context or {}).get("devices")
    if callable(doc):
        doc = doc()
    want = norm(target)
    parts = []
    for dev in (doc or {}).get("devices", []) if isinstance(doc, dict) else []:
        if not isinstance(dev, dict):
            continue
        dev_parts = [p for p in dev.get("parts", []) if isinstance(p, dict)]
        if want and want in (norm(dev.get("id")), norm(dev.get("name"))):
            parts.extend(dev_parts)
            continue
        parts.extend(p for p in dev_parts if want and want in (norm(p.get("part")), norm(p.get("name")),
                                                               norm(p.get("label"))))
    return parts


def _texts(value):
    if isinstance(value, str):
        return [value]
    if isinstance(value, (list, tuple)):
        return [str(v) for v in value]
    return []


def allowed_sides(parts):
    """(sides, source): the sides the device sheet allows ("from above", "from the front", ...), minus those it
    forbids by the same words; the default five when it names none."""
    allowed, forbidden = set(), set()
    for p in parts:
        for bag, key in ((allowed, "approach_allowed"), (forbidden, "approach_forbidden")):
            for text in _texts(p.get(key)):
                low = text.lower()
                for side, rx in _SIDE_RX.items():
                    if re.search(rx, low):
                        bag.update(("left", "right") if side == "sides" else (side,))
    sides = [s for s in SIDES if s in allowed] if allowed else list(DEFAULT_SIDES)
    sides = [s for s in sides if s not in forbidden]
    source = "device sheet" if allowed or forbidden else "default sides (the device sheet names none)"
    return sides, source


def needed_force(parts):
    lo = None
    for p in parts:
        f = p.get("force_n")
        vals = [f] if isinstance(f, (int, float)) and not isinstance(f, bool) else (
            [v for v in f if isinstance(v, (int, float))] if isinstance(f, (list, tuple)) else [])
        for v in vals:
            lo = float(v) if lo is None else min(lo, float(v))
    return lo


def side_approach(side, p_mm):
    """The approach (travel direction, base frame) of a stroke that arrives at p from the named side. front is
    the side facing the robot, left the side toward the robot's left."""
    r = np.array([float(p_mm[0]), float(p_mm[1]), 0.0])
    n = float(np.linalg.norm(r))
    r = r / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
    t = np.array([-r[1], r[0], 0.0])
    return {"above": np.array([0.0, 0.0, -1.0]), "below": np.array([0.0, 0.0, 1.0]), "front": r, "behind": -r,
            "left": -t, "right": t}[side]


def _vec(v):
    if isinstance(v, str):
        v = DIRECTION_NAMES.get(v.strip().lower())
    if v is None:
        return None
    try:
        a = np.asarray(v, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    if a.size != 3 or not np.all(np.isfinite(a)) or np.linalg.norm(a) < 1e-9:
        return None
    return a / np.linalg.norm(a)


def takes_surface(session):
    """True when this session's reach_check takes a surface argument (a package extends it). An omitted surface is
    then the tip's check only; without the argument round one's reach_check tries every leading face at the tip and
    no surface can be named, so an omitted surface stands for every one."""
    tools = getattr(session, "tools", None)
    entry = tools.get("reach_check") if isinstance(tools, dict) else None
    props = ((entry or {}).get("args") or {}).get("properties") if isinstance(entry, dict) else None
    return isinstance(props, dict) and "surface" in props


def reach_entries(calls, epoch, named=False):
    """reach_check calls of this epoch that ran: {call, point, approach, surface, reachable, force_n, short_by}.
    surface None = every tool surface (only when the tool cannot name one: named false); else its name."""
    out = []
    for c in calls:
        if c["name"] != "reach_check" or c["epoch"] != epoch:
            continue
        res = c["result"] or {}
        if "refused" in res or not isinstance(res.get("reachable"), bool):
            continue
        args, sent = c["args"] or {}, c["sent"] or {}
        pt = _first_point(args.get("point_mm"), sent.get("point_mm"))
        if pt is None:
            continue
        appr = args.get("approach") if args.get("approach") is not None else sent.get("approach")
        surface = sent.get("surface", args.get("surface"))
        surface = str(surface) if surface else (TIP if named else None)
        f, sb = res.get("force_n"), res.get("short_by_mm")
        num = (int, float)
        out.append({"call": c, "point": pt, "approach": _vec(appr), "surface": surface,
                    "reachable": res["reachable"], "force_n": f if isinstance(f, num) and not isinstance(f, bool)
                    else None, "short_by": float(sb) if isinstance(sb, num) and not isinstance(sb, bool) else None})
    return out


def _point(v):
    try:
        a = np.asarray(v, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    return a if a.size == 3 and np.all(np.isfinite(a)) else None


def _first_point(*values):
    for v in values:
        p = _point(v)
        if p is not None:
            return p
    return None


def _fmt(p):
    return "[" + ", ".join(f"{float(x):.0f}" for x in p) + "]"


# ------------------------------------------------------------------------------------------------ the gates
def change_world_gate(ex, calls, target, cited, constraints, named=False):
    """{"ok": True, ...} when change_world may be asked, else a refusal listing the missing checks. named: the
    session's reach_check takes a surface argument (see takes_surface)."""
    want = norm(target)
    if not want:
        return refusal("target", "change_world needs the target: the object to be moved or changed",
                       "give target, the label or device id of what you cannot reach")
    for c in constraints:
        if c.get("kind") == "no_move" and norm(c.get("object")) == want:
            return refusal("no_move", f"the task says {c.get('object')} must not be moved, so asking to move it "
                           "is ruled out", "work with the positions you have; claim cannot with your evidence "
                           "if the task cannot be done from here")
    pts = located_points(ex.world, target, include_stale=False) or located_points(ex.world, target)
    cited_reach = [c for _, c in cited if c["name"] == "reach_check"]
    p_tgt, p_src = None, ""
    if pts:
        p_tgt, p_src = np.asarray(pts[-1]["point_mm"], dtype=float), f"world entry {pts[-1]['id']}"
    else:
        for c in cited_reach:
            p = _first_point((c["args"] or {}).get("point_mm"), (c["sent"] or {}).get("point_mm"))
            if p is not None:
                p_tgt, p_src = p, f"the point of reach_check {call_name(c)}"
                break
    if p_tgt is None:
        return refusal("target_point", f"no located point for {target} and no reach_check of it is cited",
                       f"locate {target} (triangulate with that label) or cite the reach_check calls of it")
    parts = knowledge_parts(ex, target)
    sides, side_src = allowed_sides(parts)
    need_n = needed_force(parts)
    tool_surfaces = [s["name"] for s in ex.model.spec.get("tool_surfaces", [])]
    banned = {norm(c.get("surface")) for c in constraints if c.get("kind") == "no_surface"}
    surfaces = [s for s in tool_surfaces if norm(s) not in banned]
    B = derived.of(ex)
    pose = ex._sense()
    thing = B.thing({"id": pts[-1]["id"] if pts else target,
                     "sigma_p": derived.sigma_m(B, pts[-1]) if pts else 0})
    match_mm = derived.target_match_m(B, thing, pose.q, [0, 0, 1]) * 1000.0
    in_reach, fails = [], []
    for e in reach_entries(calls, ex.epoch, named):
        # World._point rounds millimetres to three decimal places. Compare
        # the same representation, without widening the target's uncertainty.
        represented = np.round(e["point"], 3) if pts else e["point"]
        d = float(np.linalg.norm(represented - p_tgt))
        weak = e["reachable"] and need_n is not None and e["force_n"] is not None and e["force_n"] < need_n
        if e["reachable"] and not weak:
            if d <= match_mm:
                in_reach.append(e)
        elif d <= match_mm or (not e["reachable"] and e["short_by"] is not None and d < e["short_by"]):
            fails.append(e)                # a check of a point further off proves nothing about the target
    used = in_reach + fails
    far = [c for c in cited_reach if c["epoch"] == ex.epoch and all(c is not e["call"] for e in used)]
    if not named or not surfaces:
        surfaces = []                      # round one cannot name a surface: one check stands for every one
    covered, missing, reachable = {}, [], []
    for side in sides:
        a = side_approach(side, p_tgt)
        reachable += [(side, e) for e in in_reach
                      if e["approach"] is not None and float(e["approach"] @ a) >= SIDE_COS]
        # a failing reach_check with no approach fails from every side, for the surface it checked
        failing = [e for e in fails if e["approach"] is None or float(e["approach"] @ a) >= SIDE_COS]
        if not surfaces:
            if failing:
                covered[side] = [call_name(e["call"]) for e in failing][:1]
            else:
                missing.append({"side": side, "approach": [round(float(x), 3) for x in a],
                                "surfaces": "every tool surface (leave surface out)"})
            continue
        checked = {norm(e["surface"]) for e in failing if e["surface"] is not None}
        left = [s for s in surfaces if norm(s) not in checked]
        if left:
            missing.append({"side": side, "approach": [round(float(x), 3) for x in a], "surfaces": left})
        else:
            covered[side] = [call_name(e["call"]) for e in failing]
    if reachable:
        side, e = reachable[0]
        return refusal("reachable", f"reach_check {call_name(e['call'])} found {target} reachable from the {side} "
                       f"in this epoch (force there {e['force_n']} N)", "make the stroke from that side; ask to "
                       "change the world only when every allowed side fails")
    if missing:
        text = "; ".join(f"from the {m['side']}: reach_check(point_mm={_fmt(p_tgt)}, approach="
                         f"{_fmt_dir(m['approach'])})" + ("" if isinstance(m["surfaces"], str) else
                                                          f" with surface {', '.join(m['surfaces'])}")
                         for m in missing)
        why = (f"change_world needs a failing reach_check of {target} (at {_fmt(p_tgt)} mm, from {p_src}) from "
               f"every allowed side in this epoch ({side_src}: {', '.join(sides)})"
               + (f" with every tool surface ({', '.join(surfaces)}; one that names none checks {TIP})"
                  if surfaces else "") + "; missing: " + ", ".join(m["side"] for m in missing))
        if far:
            why += (f"; cited reach_check {call_name(far[0])} checked a point too far from the target to count (a "
                    f"failing check counts within {match_mm:g} mm of it, or within the distance it was out of reach "
                    "by)")
        out = refusal("reach_check", why, "run " + text + (", or per surface one reach_check with no approach that "
                                                           "fails" if surfaces else ", or one reach_check with no "
                                                           "approach that fails"))
        out["refused"]["missing"] = [{"check": "reach_check", **m} for m in missing]
        return out
    return {"ok": True, "target_mm": [round(float(x), 1) for x in p_tgt], "sides": covered,
            "side_source": side_src, "surfaces": surfaces or "every tool surface"}


def _fmt_dir(a):
    return "[" + ", ".join(f"{x:g}" for x in a) + "]"


def hardware_gate(ex, calls, target, asked_parts):
    want = norm(target)
    if not want:
        return refusal("target", "hardware_check needs the part to inspect",
                       "give target, a joint name from robot://body or a part such as camera or gripper")
    if want in asked_parts:
        return refusal("already_asked", f"{target} was already asked about in this session",
                       "use the answer you got; work around the part or claim cannot with your evidence")
    runs = [c for c in calls if c["name"] == "self_check" and "refused" not in (c["result"] or {})]
    has_tool = "self_check" in getattr(ex, "primitive_names", lambda: [])()
    if not runs and not has_tool:
        out = refusal("self_check", "a hardware ask needs a self_check of the body in this epoch, and this runtime "
                      "has no self_check (the selfcheck package is not loaded), so no hardware ask can be backed "
                      "here", "work around the part, or claim cannot with the calls that show the fault")
        out["refused"]["missing"] = [{"check": "self_check", "available": False}]
        return out
    now = [c for c in runs if c["epoch"] == ex.epoch]
    if not now:
        why = "no self_check ran in this epoch" + (f" (world epoch {ex.epoch}; the last one was in an earlier "
                                                   "epoch)" if runs else "")
        out = refusal("self_check", f"a hardware ask needs a self_check first: {why}",
                      "call self_check, read its verdict, then ask only about what it could not settle")
        out["refused"]["missing"] = [{"check": "self_check", "available": True}]
        return out
    return {"ok": True, "self_check": call_name(now[-1])}


def marker_gate(constraints, question):
    if not any(c.get("kind") == "no_markers" for c in constraints):
        return None
    low = str(question or "").lower()
    if _MARKER_RX.search(low):
        return refusal("no_markers", "the task rules out markers and tags, and this ask is about one",
                       "work with the scene as it is: looks, touch and the device sheet")
    return None
