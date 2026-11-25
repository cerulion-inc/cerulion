# SPDX-License-Identifier: AGPL-3.0-only
"""What is left to try, ranked, pure: finish_evidence's facts in, at most UNTRIED_MAX calls
out. Only tools the session lists or reaches through use (facts["tools"]); never a stale target; an item whose
tool is not loaded is dropped. Each item is a ready call {tool, args, why} or a point call {tool: "locate",
args: {look, part}, needs: "px", why} (the operator supplies the pixel); "rule" is its rank (1 to 8), kept for the
gate (give_up_early needs an item of rules 1 to 7) and the journal, and taken off before an operator sees it.

Rules, in order:
1. for an unverified, unmoved, no_after or mark_unlocated answer, by the step's class: cue, generic, sliding,
   self_moving: look {like: the newest look before the last act from which the deciding part was in view} (its
   post hook compares); mark: a point call locate {look: the newest look after the act, part: the object}, then one
   for the mark when it has no located point;
2. a located point whose last act was a press or touch that sprang back (not an R3 verb), or a sliding part whose
   last act stopped while it was still moving: push {target: its id} when the point may be pushed (below); a point
   that names no part of the sheet gets a point call locate {look: the newest look, part: the step's control part}
   instead (the operator points at the part the sheet says to act on);
3. a fresh located control part never acted on: push {target} (lift when its do line's verbs hold lift and lift
   is listed), when the point may be pushed;
4. a push that stopped not_found, blocked, not_square or cannot_reach: view {target, side: "around"};
5. a control part with no located point while the device has a located point or candidate: view {target, side:
   the first unseen of robot_side, left, right, far_side}; a control part whose located points are all stale: a
   point call locate {look: the newest look, part} (a stale point is never a target);
6. a candidate whose fits is true and was never viewed: view {target: cN, side: "robot_side"};
7. no survey yet: survey {};
8. otherwise look {}.

A point may be pushed (SAFETY: untried never hands back a ready push or lift on a forbidden or unverified part):
its label names a part of robot://devices whose role is control (never an unknown label, a cue, a fixed or a
forbidden part); its locate carried no part_check warning (the spot's colour is another part's); and no located
point of a forbidden part is the same spot (nearer than the two points' own stated uncertainties together, and
never less than the act-near distance). A view or a locate only looks, so those keep any target.
"""

import json

from rcpx import r3_limits as L, sheet_words
from rcpx.r2 import finish_evidence as fe

RULE1_CHECKS = ("unverified", "unmoved", "no_after", "mark_unlocated")
PUSH_TROUBLE = ("not_found", "blocked", "not_square", "cannot_reach")
SIDES = ("robot_side", "left", "right", "far_side")
POINT_TOOLS = ("press", "touch")
STILL_MOVING = ("travel", "lead", "moving")


def pushable(facts, x):
    """True when untried may hand back a ready push or lift on located point x (module text, SAFETY)."""
    part = facts["parts"].get(x.get("part") or "")
    if not part or part.get("role") != "control" or x.get("point") is None or x.get("part_check"):
        return False
    for y in facts["located"]:
        other = facts["parts"].get(y.get("part") or "")
        if other and other.get("role") == "forbidden" and y.get("point") is not None:
            same = max(L.ACT_NEAR_MIN_MM, float(x.get("unc") or 0.0) + float(y.get("unc") or 0.0))
            if fe._dist(x["point"], y["point"]) <= same:
                return False
    return True


def _item(rule, tool, args, why, needs=None):
    out = {"tool": tool, "args": args}
    if needs:
        out["needs"] = needs
    out["why"] = why
    out["rule"] = rule
    return out


def _can_see_looks(facts):
    return sorted(((lid, lk) for lid, lk in facts["looks"].items() if lk["can_see"] in fe.CAN_SEE),
                  key=lambda t: t[1]["pos"])


def _fresh(facts, target):
    """A target an item may name: a fresh located or contact id, a known candidate, or a part label."""
    t = str(target)
    if t.startswith(("located_", "contact_")):
        x = next((x for x in facts["located"] if x["id"] == t), None)
        if x is not None:
            return x["fresh"]
        return t in facts.get("fresh_contacts", ())
    if t.startswith("c") and t[1:].isdigit():
        return any(str(c.get("id")) == t for c in facts["candidates"])
    return True


def _near_acts(facts, x, pool):
    lim = fe.near_mm(facts, x["part"]) if x["part"] else L.ACT_NEAR_MIN_MM
    out = []
    for a in pool:
        if a["r3"] and (a["args"].get("target") == x["id"] or (x["part"] and a.get("target_part") == x["part"])):
            out.append(a)
        elif any(fe._dist(p, x["point"]) <= lim for p in a["points"]):
            out.append(a)
    return out


def _rule1(facts, check, e, last):
    if check not in RULE1_CHECKS or e is None:
        return []
    step = e["step"]
    out = []
    if step["done_class"] == "mark":
        after = [(lid, lk) for lid, lk in _can_see_looks(facts) if last is None or lk["pos"] > last["pos"]]
        if after and "locate" in facts["tools"]:
            look = after[-1][0]
            out.append(_item(1, "locate", {"look": look, "part": step["control"]},
                             f"point at the {step['control']} here: where it is now", "px"))
            if not fe.located_of(facts, step["mark"]):
                out.append(_item(1, "locate", {"look": look, "part": step["mark"]},
                                 f"point at the {step['mark']} too: the distance decides", "px"))
        return out
    if last is None or not facts["like"]:
        return out
    want = [x["id"] for x in fe.located_of(facts, step["deciding"])] or \
        [x["id"] for x in fe.located_of(facts, step["control"])] or [f"@{last['k']}"]
    before = [lid for lid, lk in _can_see_looks(facts) if lk["pos"] < last["pos"]
              and any(w in (lk.get("shows") or ()) for w in want)]
    if before:
        part = step["deciding"] or step["control"] or "device"
        out.append(_item(1, "look", {"like": before[-1]}, f"the same view again shows whether the {part} changed"))
    return out


def _push_tool(facts, x):
    if facts["lift_listed"] and x["part"]:
        line = sheet_words.do_line(x["part"], facts["doc"], facts["tools"]) if facts.get("doc") else None
        if line and "lift" in (line.get("verbs") or []):
            return "lift"
    return "push"


def rank(facts, check, e=None):
    """The untried calls for an answer of `check` about step evaluation e (finish_evidence.evaluate), ranked."""
    pool = fe.acts(facts)
    last = (e["on"][-1] if e and e.get("on") else None) or (pool[-1] if pool else None)
    if check == "no_after" and pool:
        last = pool[-1]
    items = list(_rule1(facts, check, e, last))
    fresh = sorted((x for x in facts["located"] if x["fresh"] and x["point"] is not None),
                   key=lambda x: x["pos"], reverse=True)
    step = (e or {}).get("step") or (facts["steps"][0] if facts["steps"] else {})
    ctrl = step.get("control")
    looks = _can_see_looks(facts)
    for x in fresh:                                                        # rule 2
        near = _near_acts(facts, x, pool)
        if not near:
            continue
        back = near[-1]["name"] in POINT_TOOLS and near[-1]["sprang_back"] and not near[-1]["r3"]
        going = (facts["parts"].get(x["part"]) or {}).get("sliding") and near[-1]["stopped"] in STILL_MOVING
        if not (back or going):
            continue
        if pushable(facts, x):
            items.append(_item(2, "push", {"target": x["id"]}, "it moved and came back: push finds the force"
                               if back else "it was still moving when the push stopped"))
        elif x["part"] is None and ctrl and looks and not any(y["part"] == ctrl for y in fresh):
            items.append(_item(2, "locate", {"look": looks[-1][0], "part": ctrl},
                               f"that point names no part: point at the {ctrl}", "px"))
    for x in fresh:                                                        # rule 3
        if pushable(facts, x) and not _near_acts(facts, x, pool):
            tool = _push_tool(facts, x)
            items.append(_item(3, tool, {"target": x["id"]}, f"a located {x['part']} that nothing has pushed yet"))
    for c in reversed(facts["calls"]):                                     # rule 4
        if c["name"] == "push" and c["stopped"] in PUSH_TROUBLE and c["args"].get("target") is not None:
            items.append(_item(4, "view", {"target": c["args"]["target"], "side": "around"},
                               f"the push there stopped {c['stopped'].replace('_', ' ')}: see every side"))
    if ctrl and looks and fe.located_of(facts, ctrl) and not any(y["part"] == ctrl for y in fresh):
        items.append(_item(5, "locate", {"look": looks[-1][0], "part": ctrl},      # rule 5, stale points
                           f"the {ctrl}'s points are stale: point at it again", "px"))
    if ctrl and not fe.located_of(facts, ctrl):                            # rule 5
        target = next((x["id"] for x in fresh if x["part"]), None) or next((x["id"] for x in fresh), None)
        if target is None and facts["candidates"]:
            fits = [c for c in facts["candidates"] if (c.get("fits") or {}).get("value") is True]
            best = sorted(fits, key=lambda c: (c.get("fits") or {}).get("score") or 0.0) or facts["candidates"]
            target = best[0].get("id")
        if target is not None:
            used = {v["side"] for v in facts["views"] if v["target"] == target}
            side = next((s for s in SIDES if s not in used), None)
            if side is not None:
                items.append(_item(5, "view", {"target": target, "side": side},
                                   f"the {ctrl} is in no look yet: see another side"))
    viewed = {v["target"] for v in facts["views"]}
    fits = [c for c in facts["candidates"] if (c.get("fits") or {}).get("value") is True and c.get("id") not in viewed]
    for c in sorted(fits, key=lambda c: (c.get("fits") or {}).get("score") or 0.0):   # rule 6
        items.append(_item(6, "view", {"target": c["id"], "side": "robot_side"},
                           f"{c['id']} fits the device and has not been viewed"))
    if not facts["surveyed"]:                                              # rule 7
        items.append(_item(7, "survey", {}, "no survey yet: it lists the things on the table"))
    out = _keep(facts, items)
    if not out:                                                            # rule 8: otherwise
        out = _keep(facts, [_item(8, "look", {}, "a new look shows the state now")])
    return out


def _keep(facts, items):
    """Reachable tools only, fresh targets only, no repeats, at most UNTRIED_MAX."""
    out, seen = [], set()
    for it in items:
        key = (it["tool"], json.dumps(it["args"], sort_keys=True))
        if key in seen or it["tool"] not in facts["tools"]:
            continue
        t = it["args"].get("target")
        if t is not None and not _fresh(facts, t):
            continue
        seen.add(key)
        out.append(it)
        if len(out) >= L.UNTRIED_MAX:
            break
    return out


def describe(item, facts):
    """An item in a few words, for a why: "push green_mark", "view c2 from robot_side", "survey"."""
    a = item.get("args") or {}
    label = {x["id"]: x["label"] for x in facts["located"]}
    if item["tool"] in ("push", "lift"):
        return f"{item['tool']} {label.get(a.get('target'), a.get('target'))}"
    if item["tool"] == "view":
        return f"view {label.get(a.get('target'), a.get('target'))} from {a.get('side')}"
    if item["tool"] == "locate":
        return f"locate the {a.get('part')} in {a.get('look')}"
    if item["tool"] == "look" and a.get("like"):
        return f"look like {a['like']}"
    return item["tool"]


def already_refused(facts, items):
    """True when every item was sent before and was refused or stopped cannot_reach (nothing new to try)."""
    if not items:
        return False
    for it in items:
        keys = {k: v for k, v in (it.get("args") or {}).items() if k in ("target", "like", "look", "part", "side")}
        hit = any(c["name"] == it["tool"] and (c["refused"] or c["stopped"] == "cannot_reach")
                  and all(c["args"].get(k) == v for k, v in keys.items()) for c in facts["calls"])
        if not hit:
            return False
    return True


def public(items):
    """The items as an operator reads them: without the rank."""
    return [{k: v for k, v in it.items() if k != "rule"} for it in items]


__all__ = ["already_refused", "describe", "public", "pushable", "rank"]
