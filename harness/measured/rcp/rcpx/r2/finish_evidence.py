# SPDX-License-Identifier: AGPL-3.0-only
"""The claim gate's evidence, pure: what the operator did and saw, read from the finish package's
own call records and the shared ex.state records, into done, contradicted and the gate's checks. No executor, no
model, no motion: finish.py builds the facts (projections included) and this module only reads them.

A position orders everything: (k, sub), k the index of the operator call in finish's records, sub 0 for a look
taken inside call k before its act (the before picture of a push), 1 for the act itself, 2 for whatever came
after it inside that call (a look_after, a push's after picture, a point stored). So "after the act" is a tuple
comparison, the same on every robot.

Facts (finish.build_facts):
    steps       sheet_words.steps of the task sentence: {state, device, deciding, control, done_class, colour, mark}
    parts       {name: {role, sliding, size_max, colours, states, how_to_tell}}
    plist       sheet_words.parts of robot://devices (for label_part)
    calls       the call records (call_record + k, seq, call, pos, small_mm, rigid_mm, target_part, ...)
    located     [{id, label, part, point, pos, fresh}]  (World kind located, stale ones included)
    points      {entry id: point}  (located and contact entries)
    looks       {look id: {pos, can_see, shows: [entry ids in frame], region}}
    records     witness records {id, before, after, before_pos, after_pos, at, changes, light_changed, device_moved}
    verifies    [{pos, tested, accepted, state, verdict}]
    budget, light, tools, like, lift_listed, candidates, surveyed, views, now_call, doc

Travel thresholds: no millimetre is typed here. Each act record carries rigid_mm, the arm's
own measurement step at the act's pose and force (finish writes it from the session's bounds). A sliding part's
last act ended at an end, and a press was still moving, when it went at least QUANT x that act's step along; its
acts moved it in all when they add to QUANT x the largest step among them. An act with no recorded step counts a
step of 0: then any travel over 0 reads as travel, and no travel never does.
"""

import math

from rcpx import bounds_ratios as R
from rcpx import colour_words, r3_limits as L, sheet_words
from rcpx.r2.finish_steps import small_mm as _small_mm, step_mm as _step_mm, under as _under

R3_VERBS = ("push", "lift")
MET_STOPS = ("force", "contact", "part_hit")       # the move ended ON something: the runtime's own stop
FLAG_STOPS = ("gave", "moved_part")                # a flag only: counts with measured travel against resistance
LEAVING = ("retreat",)                             # a call whose stated intent is to leave what it rests on
RESIST_KEYS = ("resisted_mm", "moved_under_load_mm")
BLOCKING = ("no_grip", "constraint", "seen_obstacle", "path_box", "dark")
END_STOPS = ("force", "stall", "contact")          # a press, move_tip or touch that ended on something
SLIDING_BAD = ("ceiling", "time", "not_found", "blocked", "cannot_reach")
STILL_MOVING_STOPS = ("travel", "lead")
LIT_STATES, OFF_STATES = ("on", "lit"), ("off", "unlit", "dark")
CAN_SEE = ("ok", "dim")
POINT_KEYS = ("point_mm", "first_resist_mm", "tip_mm_at_stop", "tip_mm")
STEP_NOUN = {"open": "the open", "shut": "the close", "closed": "the close", "on": "the switch on",
             "off": "the switch off"}
BEGIN = (-1, 2)


# ---------------------------------------------------------------- one call
def _num(v):
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else None


def _pt(v):
    if isinstance(v, (list, tuple)) and len(v) == 3 and all(_num(x) is not None for x in v):
        return [float(x) for x in v]
    return None


def resisted_mm(result):
    """The tip's measured travel against resistance in this call (0 when the result states none)."""
    for key in RESIST_KEYS:
        v = _num(result.get(key))
        if v is not None:
            return max(0.0, v)
    return 0.0


def is_device_act(name, result, rigid_mm=None):
    """A call that MET something: an R3 verb that felt something; a contact the runtime reports (contact not null);
    a stop on something (force, contact, part_hit); or a gave or moved_part reading (the flag or the stop) backed by
    travel against resistance of at least rigid_mm, the robot's own measurement step at that pose (finish._limits,
    from robot://body's stated resolution). A flag alone is not a meeting: on a robot with lag and sensor noise a
    press in free air reads gave and moved_part with 0 to 0.2 mm resisted (measured). A call that leaves
    (retreat) counts only when it stopped on something: what it reads on the way out is the part coming back behind
    it. Any other tool counts (a move_tip push that slid a part 53 mm with contact null too)."""
    if not isinstance(result, dict) or "refused" in result:
        return False
    if name in R3_VERBS and result.get("felt"):
        return True
    c = result.get("contact")
    if c is not None and c is not False:
        return True
    if result.get("stopped") in MET_STOPS:
        return True
    if name in LEAVING:
        return False
    flag = result.get("moved_part") is True or result.get("gave") is True or result.get("stopped") in FLAG_STOPS
    return flag and resisted_mm(result) > 0 and resisted_mm(result) >= (0.0 if rigid_mm is None else rigid_mm)


def inner_refusal(name, result):
    """The check of a refusal inside a call that still returned a result: an R3 verb a nested gate stopped reports
    stopped "refused" with the check in a key or only in its said text."""
    if result.get("stopped") != "refused":
        return None
    for key in ("check", "stopped_check"):
        if isinstance(result.get(key), str):
            return result[key]
    said = str(result.get("said") or "")
    return next((b for b in BLOCKING if b in said), "refused")


def felt_words(result):
    return [str(f.get("word")) for f in (result.get("felt") or []) if isinstance(f, dict) and f.get("word")]


def along_mm(name, result):
    """How far the act moved the part along its push: an R3 verb's travel; else resisted_mm (the tip's travel
    against resistance, what the part gave), then a press's along_push_mm or travel_mm, then moved_under_load_mm.
    A touch's along_push_mm is never read: it is the touch move's whole travel, free air included (the live touch
    result, measured on the r1b replays: 30 to 100 mm of travel against 0.01 to 0.13 mm resisted)."""
    if name in R3_VERBS:
        v = _num(result.get("travel_mm"))
        if v is not None:
            return max(0.0, v)
        felt = [f for f in (result.get("felt") or []) if isinstance(f, dict) and _num(f.get("mm")) is not None]
        if not felt:
            return 0.0
        legs = [float(f["mm"]) for f in felt if f.get("word") in ("moving", "reached_end")]
        return sum(legs) if legs else max(float(f["mm"]) for f in felt)
    keys = ("resisted_mm", "along_push_mm", "travel_mm", "moved_under_load_mm") if name == "press" else \
        ("resisted_mm", "moved_under_load_mm")
    for key in keys:
        v = _num(result.get(key))
        if v is not None:
            return max(0.0, v)
    return 0.0


def call_record(name, args, result, rigid_mm=None):
    """The compact record of one operator call: what the gate reads later. rigid_mm: the robot's measurement step
    at the call's pose (is_device_act)."""
    res = result if isinstance(result, dict) else {}
    ref = res.get("refused")
    contact = res.get("contact") if isinstance(res.get("contact"), dict) else None
    pts = []
    if contact is not None and _pt(contact.get("point_mm")):
        pts.append(_pt(contact["point_mm"]))
    for key in POINT_KEYS:
        p = _pt(res.get(key))
        if p is not None and p not in pts:
            pts.append(p)
    wit = res.get("witness") if isinstance(res.get("witness"), dict) else {}
    look = res.get("look") if isinstance(res.get("look"), dict) else {}
    out = {"name": name, "args": dict(args) if isinstance(args, dict) else {},
           "refused": ref.get("check") if isinstance(ref, dict) else inner_refusal(name, res),
           "stopped": res.get("stopped"), "resisted": round(resisted_mm(res), 2),
           "act": is_device_act(name, res, rigid_mm), "r3": name in R3_VERBS, "points": pts,
           "contact_id": (contact or {}).get("id") or (res.get("id") if res.get("contact") is True else None),
           "felt": felt_words(res), "along": round(along_mm(name, res), 2),
           "sprang_back": res.get("sprang_back") is True, "gave": res.get("gave") is True,
           "moved_part": res.get("moved_part") is True, "peak_n": _num(res.get("peak_n", res.get("peak_force_n"))),
           "ceiling_n": _num(res.get("ceiling_n")), "witness_before": wit.get("before"),
           "witness_after": wit.get("after"), "look_id": res.get("look_id") or look.get("look_id")}
    if name in R3_VERBS:
        out["r3_part"], out["r3_target"] = res.get("part"), res.get("target")
    if name == "verify":
        ec = res.get("evidence_checked") if isinstance(res.get("evidence_checked"), dict) else {}
        out["verify"] = {"tested": ec.get("tested") or res.get("tested"), "accepted": res.get("accepted") is True,
                         "verdict": res.get("verdict"), "state": res.get("state") or out["args"].get("state"),
                         "look": out["args"].get("look_id")}
    if name == "survey" and isinstance(res.get("candidates"), list):
        out["candidates"] = [{k: c.get(k) for k in ("id", "where_mm", "size_mm", "top_mm", "fits")}
                             for c in res["candidates"] if isinstance(c, dict)]
    return out


# ---------------------------------------------------------------- reading the facts
def _dist(a, b, xy=False):
    n = 2 if xy else 3
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(n)))


def acts(facts):
    return [c for c in facts["calls"] if c["act"]]


def act_word(a):
    """came_back, rigid, gave, ... of one act: its own felt words for an R3 verb, else read off the result."""
    if a["r3"]:
        return a["felt"][-1] if a["felt"] else (a["stopped"] or "")
    if a["sprang_back"]:
        return "came_back"
    if a["gave"]:
        return "gave"
    if _under(a["along"], _step_mm(a)) and not a["moved_part"]:
        return "rigid"
    if a["along"] < _small_mm(a):
        return "small_move"
    return "stayed_in"


def moved(a):
    """moved(L): it did not spring back and it went small_mm along, or it gave or moved a part by at least rigid_mm
    along (a flag with no measured travel behind it is not a move); an R3 verb that stopped changed or
    reached_end."""
    if a["r3"]:
        return a["stopped"] in ("changed", "reached_end")
    if a["sprang_back"] or a["along"] <= 0:
        return False
    if a["along"] >= _small_mm(a):
        return True
    return (a["gave"] or a["moved_part"]) and a["along"] >= _step_mm(a)


def felt_rigid_or_back(a):
    if a["r3"]:
        return any(w in ("rigid", "came_back") for w in a["felt"]) or a["stopped"] in ("rigid", "ceiling")
    return a["sprang_back"] or (not a["gave"] and _under(a["along"], _step_mm(a)))


def near_mm(facts, part):
    size = (facts["parts"].get(part) or {}).get("size_max")
    return max(L.ACT_NEAR_MIN_MM, L.ACT_NEAR_SIZE_FRAC * size) if size else L.ACT_NEAR_MIN_MM


def located_of(facts, part):
    if part is None:
        return []
    return [x for x in facts["located"] if x["part"] == part and x["point"] is not None]


def acts_on(facts, part, pool):
    """The acts of pool on part: an R3 verb whose target names it, or an act whose contact point or target lies
    near one of its located points. With no located point, or none of the acts near one, every act of pool."""
    if part is None:
        return list(pool)
    locs, lim = located_of(facts, part), near_mm(facts, part)
    on = []
    for a in pool:
        if a["r3"] and a.get("target_part") is not None:
            if a["target_part"] == part:
                on.append(a)
            continue
        if any(_dist(p, x["point"]) <= lim for p in a["points"] for x in locs):
            on.append(a)
    return on if (locs and on) else list(pool)


def on_by_fallback(facts, part, pool):
    """Why acts_on(part) is every act (the gate then knows that something was met, not that it was part): no
    point of part is located, or no act met one; None when some act is known to be on part."""
    if part is None or not pool:
        return None
    locs, lim = located_of(facts, part), near_mm(facts, part)
    for a in pool:
        if a["r3"] and a.get("target_part") is not None:
            if a["target_part"] == part:
                return None
            continue
        if any(_dist(p, x["point"]) <= lim for p in a["points"] for x in locs):
            return None
    return f"no act met a located point of the {part}" if locs else f"no point of the {part} is located"


def at_part(facts, at):
    part = at.get("part")
    if part:
        return str(part).lower()
    p = sheet_words.label_part(at.get("label"), facts["plist"])
    if p is not None:
        return p["name"]
    for x in facts["located"]:
        if x["id"] == at.get("id"):
            return x["part"]
    return None


def _at_point(facts, at):
    return facts["points"].get(at.get("id"))


def records_after(facts, pos, upper=None):
    """Witness records whose after-look came after pos (and before upper); a whole-device move never counts."""
    return [r for r in facts["records"] if r["after_pos"] > pos and (upper is None or r["after_pos"] < upper)
            and not r.get("device_moved")]


def changed_at(facts, rec, part, near=None, radius=None):
    """The at item of rec that says changed yes at part (or, with near and radius, at a point that close)."""
    for at in rec.get("at") or []:
        if at.get("changed") != "yes":
            continue
        if part is None or at_part(facts, at) == part:
            return at
        p = _at_point(facts, at)
        if near and radius and p is not None and any(_dist(p, c) <= radius for c in near):
            return at
    return None


def solid_no_at(facts, rec, parts):
    for at in rec.get("at") or []:
        if at.get("changed") == "no" and at.get("solid") and at_part(facts, at) in parts:
            return at
    return None


def lit_state(colours, state):
    off = [s for s in colours if s in OFF_STATES]
    if off:
        return colours.get(state) is not None and colours.get(state) != colours[off[0]]
    return state in LIT_STATES


def _in_region(region, box):
    if not region or not box or len(box) != 4:
        return False
    u, v = 0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3])
    if "box" in region:
        b = region["box"]
        return b[0] <= u <= b[2] and b[1] <= v <= b[3]
    cu, cv, r = region["disc"]
    return math.hypot(u - cu, v - cv) <= r


def _act_text(a):
    return f"the {a['name']} at call {a['call']}"


# ---------------------------------------------------------------- done, by class
def _res(done, missing="", rec=None, evidence="", pos=None, L_act=None, seen=True):
    """seen: a done that rests on a same-view record or a cue reading taken after the act (the device was seen in
    its goal state); False for a done that rests only on where labelled points lie (the mark test)."""
    return {"done": done, "missing": missing, "record": rec, "evidence": evidence, "pos": pos, "last": L_act,
            "seen": bool(done and seen)}


def _cue(facts, step, on, Lact, recs, prev_pos):
    dec, colour, state = step["deciding"], step["colour"], step["state"]
    cols = (facts["parts"].get(dec) or {}).get("colours") or {}
    lit = lit_state(cols, state)
    for r in recs:
        at = changed_at(facts, r, dec)
        if at is not None and colour_words.matches(at.get("colour_after"), bool(at.get("glowing")), colour, lit):
            return _res(True, rec=r["id"], pos=r["after_pos"], L_act=Lact,
                        evidence=f"the {dec} went from {at.get('colour_before')} to {at.get('colour_after')}")
    if not located_of(facts, dec):
        for r in recs:
            region = (facts["looks"].get(r["after"]) or {}).get("region")
            for ch in r.get("changes") or []:
                if ch.get("kind") in ("lit", "colour_changed") and \
                        _in_region(region, ch.get("box") or ch.get("box_px")) and \
                        colour_words.matches(ch.get("colour_after"), bool(ch.get("glowing")), colour, lit):
                    return _res(True, rec=r["id"], pos=r["after_pos"], L_act=Lact,
                                evidence=f"a patch on the device went {ch.get('colour_after')}")
    for v in facts["verifies"]:
        if v["pos"] > Lact["pos"] and v["tested"] == "cue_read" and v["accepted"] and v["state"] == state:
            return _res(True, rec=None, pos=v["pos"], L_act=Lact, evidence=f"verify read the {dec} as {state}")
    return _res(False, f"no same view before and after {_act_text(Lact)} shows the {dec} {colour}", L_act=Lact)


def _generic(facts, step, on, Lact, recs, prev_pos):
    part = step["deciding"] or step["control"]
    for r in recs:
        if changed_at(facts, r, part) is not None:
            return _res(True, rec=r["id"], pos=r["after_pos"], L_act=Lact,
                        evidence=f"the {part or 'device'} changed")
    return _res(False, f"no same view before and after {_act_text(Lact)} at the {part or 'device'}", L_act=Lact)


def _sliding(facts, step, on, Lact, recs, prev_pos):
    part, first = step["control"], on[0]
    end = (Lact["r3"] and Lact["stopped"] == "reached_end") or (
        not Lact["r3"] and Lact["name"] in ("press", "move_tip", "touch") and Lact["stopped"] in END_STOPS
        and not _under(Lact["along"], R.QUANT * _step_mm(Lact)))
    total = sum(a["along"] for a in on)          # compared before any rounding for display
    rec = next((r for r in recs if r["after_pos"] > first["pos"] and changed_at(facts, r, part) is not None), None)
    if not end:
        return _res(False, f"{_act_text(Lact)} did not end at a stop", L_act=Lact)
    need = R.QUANT * _step_mm(on)
    if _under(total, need):
        return _res(False, f"the acts on the {part} moved it {round(total, 2):g} mm in all (under {need:g} mm, twice "
                    "the arm's measurement step there, reads as none)", L_act=Lact)
    if rec is None:
        return _res(False, f"no same view before and after {_act_text(first)} shows the {part} move", L_act=Lact)
    return _res(True, rec=rec["id"], pos=rec["after_pos"], L_act=Lact,
                evidence=f"the {part} moved {total:g} mm, stopped at its end and changed in the same view")


def _mark_info(facts, step, Lact):
    obj, mark = step["control"], step["mark"]
    after = sorted((x for x in located_of(facts, obj) if Lact is None or x["pos"] > Lact["pos"]),
                   key=lambda x: x["pos"])
    marks = located_of(facts, mark)
    size = (facts["parts"].get(mark) or {}).get("size_max") or 0.0
    return obj, mark, after, marks, size


def _mark(facts, step, on, Lact, recs, prev_pos):
    obj, mark, after, marks, size = _mark_info(facts, step, Lact)
    if not after:
        return _res(False, f"no point of the {obj} since the last push", L_act=Lact)
    if not marks:
        return _res(False, f"no point of the {mark}: locate it", L_act=Lact)
    doubt = [x for x in after[-1:] + marks if x.get("part_check")]     # sight's own warning on that point
    if doubt:
        return _res(False, f"the point named {doubt[0]['label']} ({doubt[0]['id']}) may be another part by its "
                           "colour: locate it again", L_act=Lact)
    lim = L.MARK_IN_FRAC * size
    for x in after:
        d = min(_dist(x["point"], m["point"], xy=True) for m in marks)
        if d <= lim:
            return _res(True, pos=x["pos"], L_act=Lact, seen=False,
                        evidence=f"the {obj} ({x['id']}) lies {d:.0f} mm from the {mark}'s point")
    d = min(_dist(after[-1]["point"], m["point"], xy=True) for m in marks)
    return _res(False, f"the {obj}'s newest point ({after[-1]['id']}) lies {d:.0f} mm from the {mark} "
                       f"({lim:.0f} counts as on it)", L_act=Lact)


def _self_moving(facts, step, on, Lact, recs, prev_pos):
    dec = step["deciding"]
    radius = (facts["parts"].get(dec) or {}).get("size_max")
    near = [x["point"] for x in located_of(facts, dec) + located_of(facts, step["control"])] + Lact["points"][:1]
    for r in recs:
        if prev_pos is not None and r["before_pos"] <= prev_pos:
            continue
        if changed_at(facts, r, dec, near=near, radius=radius) is not None:
            return _res(True, rec=r["id"], pos=r["after_pos"], L_act=Lact, evidence=f"the {dec} moved ({r['id']})")
    return _res(False, f"no same view before and after {_act_text(Lact)} shows the {dec or 'device'} move",
                L_act=Lact)


CLASS_TEST = {"cue": _cue, "generic": _generic, "sliding": _sliding, "mark": _mark, "self_moving": _self_moving}


def done_step(facts, step, pool, start, final):
    """done for one step over the acts of pool (those after start). A step before the last is done at the first
    act after which its test holds (records up to the next act); the last step is judged after its last act."""
    on = acts_on(facts, step["control"], pool)
    if not on:
        return _res(False, f"nothing has met the {step['control'] or 'device'} yet")
    test = CLASS_TEST.get(step["done_class"], _generic)
    prev = None if start == BEGIN else start
    idx = [len(on) - 1] if final else range(len(on))
    res = None
    for i in idx:
        Lact = on[i]
        upper = on[i + 1]["pos"] if (not final and i + 1 < len(on)) else None
        res = test(facts, step, on[:i + 1], Lact, records_after(facts, Lact["pos"], upper), prev)
        if res["done"]:
            return res
    return res


def contradicted(facts, step, pool):
    """(True, why) when the newest evidence after the step's last act says it did not happen."""
    on = acts_on(facts, step["control"], pool)
    if not on:
        return False, ""
    Lact, cls, part = on[-1], step["done_class"], step["control"] or "device"
    if cls == "sliding":
        if Lact["stopped"] == "ceiling":
            c = Lact["ceiling_n"] or Lact["peak_n"]
            return True, (f"{_act_text(Lact)} stopped at its ceiling{f' {c:g} N' if c else ''} after "
                          f"{Lact['along']:g} mm: the {part} was still moving")
        if Lact["stopped"] in SLIDING_BAD:
            return True, f"{_act_text(Lact)} stopped {Lact['stopped']}: the {part} did not reach its end"
        if Lact["name"] == "press" and Lact["stopped"] in STILL_MOVING_STOPS \
                and not _under(Lact["along"], R.QUANT * _step_mm(Lact)):
            return True, f"{_act_text(Lact)} stopped on its {Lact['stopped']} after {Lact['along']:g} mm: the " \
                         f"{part} was still moving"
    if cls == "mark":
        obj, mark, after, marks, size = _mark_info(facts, step, Lact)
        if after and marks:
            d = min(_dist(after[-1]["point"], m["point"], xy=True) for m in marks)
            if d > L.MARK_OUT_FRAC * size:
                return True, (f"the {obj}'s newest point ({after[-1]['id']}) lies {d:.0f} mm from the {mark}: "
                              f"more than {L.MARK_OUT_FRAC * size:.0f} mm is off it")
    recs = records_after(facts, Lact["pos"])
    newest = recs[-1] if recs else None
    dec = step["deciding"]
    if cls == "cue" and newest is not None:
        cols = (facts["parts"].get(dec) or {}).get("colours") or {}
        for at in newest.get("at") or []:
            if at_part(facts, at) == dec and not at.get("glowing"):
                other = [s for s, c in cols.items() if s != step["state"] and c == at.get("colour_after")]
                if other:
                    return True, (f"{newest['id']} shows the {dec} {at.get('colour_after')} after {_act_text(Lact)}:"
                                  f" robot://devices says {at.get('colour_after')} means {other[0]}")
    if cls == "cue":
        for v in facts["verifies"]:
            if v["pos"] > Lact["pos"] and v["verdict"] == "contradicted":     # only a cue reading contradicts
                return True, f"verify read the {dec} as not {step['state']} after {_act_text(Lact)}"
    if newest is not None and not newest.get("light_changed") and felt_rigid_or_back(Lact):
        at = solid_no_at(facts, newest, [p for p in (dec, step["control"]) if p])
        if at is not None:
            return True, (f"{newest['id']} shows no change at the {at_part(facts, at)} and {_act_text(Lact)} felt it "
                          f"{'come back' if act_word(Lact) == 'came_back' else act_word(Lact).replace('_', ' ')}")
    return False, ""


# ---------------------------------------------------------------- every step
def evaluate(facts):
    """Per goal step, in order: {index, step, start, pool, on, done (the done_step result), contradicted, why,
    missing_prev}. A step after one that is not done is judged with no window (missing_prev true)."""
    out, start, all_acts = [], BEGIN, acts(facts)
    n = len(facts["steps"])
    for i, step in enumerate(facts["steps"]):
        final = i == n - 1
        if start is None:
            out.append({"index": i + 1, "step": step, "start": None, "pool": [], "on": [], "missing_prev": True,
                        "done": _res(False, "the step before it is not shown yet"), "contradicted": False, "why": ""})
            continue
        pool = [a for a in all_acts if a["pos"] > start]
        d = done_step(facts, step, pool, start, final)
        c, why = (False, "")
        if final or not d["done"]:
            c, why = contradicted(facts, step, pool)
        out.append({"index": i + 1, "step": step, "start": start, "pool": pool,
                    "on": acts_on(facts, step["control"], pool), "missing_prev": False, "done": d,
                    "fallback": on_by_fallback(facts, step["control"], pool), "contradicted": c, "why": why})
        start = d["pos"] if d["done"] else None
    return out


# ---------------------------------------------------------------- the checks, in the spec's order
def _ans(check, hard, why, ev=None):
    return {"check": check, "hard": hard, "why": why, "step": None if ev is None else ev["index"], "ev": ev}


def checks(facts, success, ev=None):
    """Every check that fires for this claim, in order (the gate takes the first it may give). Success: no_act,
    no_after, step_missing, contradicted, mark_unlocated, unmoved, unverified. Failure: seen_not_claimed,
    give_up_early (finish_untried decides whether give_up_early has an item to offer; the caller filters)."""
    ev = evaluate(facts) if ev is None else ev
    all_acts = acts(facts)
    out = []
    if not success:
        if ev and all(e["done"]["done"] and not e["contradicted"] for e in ev):
            shown = []
            for e in ev:
                d = e["done"]
                shown.append(d["evidence"] + (f" ({d['record']})" if d["record"] else ""))
            how = (facts["parts"].get(ev[-1]["step"]["deciding"]) or {}).get("how_to_tell")
            why = "the evidence shows " + "; then ".join(shown) + (f'; the sheet says "{how}"' if how else "")
            out.append(_ans("seen_not_claimed", False, why, ev[-1]))
        out.append(_ans("give_up_early", False, "", ev[0] if ev else None))
        return out
    if not all_acts:
        why = "nothing has met the device yet: no state can have changed"
        flags = [c for c in facts["calls"] if c["name"] not in LEAVING and not c["refused"] and
                 (c["gave"] or c["moved_part"] or c["stopped"] in FLAG_STOPS)]
        if flags:        # say which bound dismissed them: the arm's own measurement step, not a typed number
            step_mm = max(c.get("rigid_mm") or 0.0 for c in flags)
            why += (f" ({len(flags)} calls read gave or moved with under {step_mm:g} mm against resistance, the "
                    "arm's measurement step there)")
        return [_ans("no_act", True, why, ev[0] if ev else None)]
    last = all_acts[-1]
    if not any(lk["pos"] > last["pos"] and lk["can_see"] in CAN_SEE for lk in facts["looks"].values()):
        out.append(_ans("no_after", True, f"no look that could see since {_act_text(last)}", ev[-1] if ev else None))
    for i, e in enumerate(ev[1:], start=1):
        prev = ev[i - 1]
        st = e["step"]["state"]
        noun = STEP_NOUN.get(st, f"the step to {st}")
        if e["missing_prev"] or not prev["done"]["done"]:
            out.append(_ans("step_missing", True, f"no same view before and after has shown the "
                                                  f"{prev['step']['deciding'] or 'device'} {prev['step']['state']} "
                                                  f"yet, so {noun} cannot count", e))
            break
        if not e["pool"]:
            out.append(_ans("step_missing", True, f"the {prev['step']['deciding'] or 'device'} was seen "
                                                  f"{prev['step']['state']} ({prev['done']['record'] or 'a look'}) "
                                                  f"and nothing has pushed since: {noun} has not happened", e))
            break
    for e in ev:
        if e["contradicted"]:
            out.append(_ans("contradicted", True, e["why"], e))
            break
    for e in ev:
        if e["step"]["done_class"] == "mark" and not e["missing_prev"]:
            on = e["on"]
            obj, mark, after, marks, size = _mark_info(facts, e["step"], on[-1] if on else None)
            if not after:
                out.append(_ans("mark_unlocated", True, f"no point of the {obj} since the last push: locate the "
                                                        f"{obj} and the {mark}", e))
                break
    for e in ev:
        # H6 skips a step whose done was SEEN (a same-view record or a cue reading after the act); a done that
        # rests on labelled points alone (the mark test) does not excuse an act that moved nothing
        if e["missing_prev"] or e["done"]["seen"] or not e["on"]:
            continue
        Lact = e["on"][-1]
        dec = e["step"]["deciding"] or e["step"]["control"]
        seen = any(changed_at(facts, r, dec) is not None for r in records_after(facts, Lact["pos"]))
        if not moved(Lact) and not seen:
            part = e["step"]["control"] or "device"
            if e.get("fallback"):
                why = (f"{_act_text(Lact)} moved what it met {Lact['along']:g} mm and no same-view pair shows a "
                       f"change; {e['fallback']}")
            else:
                why = f"{_act_text(Lact)} moved the {part} {Lact['along']:g} mm and no same-view pair shows a change"
            out.append(_ans("unmoved", True, why, e))
            break
    for e in ev:
        if not e["done"]["done"] and not e["missing_prev"]:
            why = e["done"]["missing"]
            if e.get("fallback") and e["on"]:
                why += f"; {e['fallback']}"
            out.append(_ans("unverified", False, why, e))
    return out


__all__ = ["acts", "acts_on", "act_word", "along_mm", "call_record", "changed_at", "checks", "contradicted",
           "done_step", "evaluate", "felt_rigid_or_back", "inner_refusal", "is_device_act", "lit_state", "moved",
           "on_by_fallback", "records_after", "resisted_mm"]
