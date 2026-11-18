# SPDX-License-Identifier: AGPL-3.0-only
"""What a same-view reading means (the witness package, SPEC-R3.md 4.4): the device's own move as a first-class
answer, one reading every consumer can act on, and the fork a compare offers after a change.

A part of a device reads one of four ways, and no consumer has to guess between them:
    changed        the part's region changed and its surroundings did not (an at item "changed": "yes")
    unchanged      a solid no: a region SOLID_MIN_PX or more across, frames not dim, the light the same
    unclear        the frames could not decide it (dim, small, covered, the light changed, not lined up)
    device_moved   the device itself moved between the looks, so every stored point on it is where the part WAS:
                   nothing on it can read changed or unchanged until the part is located again
and a part with no reading at all (no record, a refused compare, the part out of view) is not_judged, never
unchanged.

The device's own move (moved): the device's outline in the before frame is looked for again in the after frame
(witness_diff.device_offset: the best normalised correlation over offsets and small turns, and the correlation
where it was). The offset must exceed what the camera's own displacement between the two looks explains (the
camera stood cam_mm away from its first place by the model's FK at the measured joints; a thing at depth d then
shifts up to f x cam_mm / d pixels by parallax alone, which the frames' alignment on the background does not take
out). Then the answer has two strengths with ONE consequence, so no decision sits on the correlation's exact
value: "yes" at DEVICE_MOVED_NCC or more, "possible" from DEVICE_POSSIBLE_NCC; either way every target on the
device reads unclear ("device_moved"), never yes, never no, and no claim is offered. Measured on two renderers
(half-size frames): a device moved 38 mm and turned 7.6 deg reads 0.795 and 0.812; a part sliding 40 mm inside a
device that stayed reads 0.55 to 0.65; a rigid thing that stayed reads its best where it was.

Nothing here reads ground truth or names a robot or a device.
"""

import math

from rcpx import colour_words, sheet_words
from rcpx import r3_limits as L
from rcpx.r2 import witness_diff as diff
from rcpx.r2 import witness_pose as pose

DEVICE_POSSIBLE_NCC = 0.7   # the device's outline found again elsewhere at this correlation or more MAY have moved:
                            # between a part's own slide inside a still device (0.65 at most, measured) and a moved
                            # device (0.79 at least); below DEVICE_MOVED_NCC it is "possible", with the same result
LIT_WORDS = ("on", "lit")   # goal states a light shows by glowing (the device package's own words)
ANSWERS = ("changed", "unchanged", "unclear", "device_moved", "not_judged")


def moved(off, box, depth_mm, f_px, cam_mm):
    """The device's own move from device_offset's (dx, dy, ncc, turn_deg, stayed_ncc): None (it stayed, or cannot
    be read) or {"answer": "yes" | "possible", "px", "mm", "turn_deg", "ncc", "stayed_ncc", "box", "why"}."""
    if off is None:
        return None
    px = math.hypot(off[0], off[1])
    parallax = float(f_px) * float(cam_mm) / max(float(depth_mm), 1e-6)
    shifted = px >= L.DEVICE_MOVED_PX + parallax or abs(off[3]) >= diff.DEVICE_TURN_MIN_DEG
    if not shifted or off[2] < DEVICE_POSSIBLE_NCC or off[2] <= off[4]:
        return None
    sure = off[2] >= L.DEVICE_MOVED_NCC
    return {"answer": "yes" if sure else "possible", "px": [round(off[0], 1), round(off[1], 1)],
            "mm": round(px * float(depth_mm) / float(f_px), 1), "turn_deg": off[3], "ncc": round(off[2], 3),
            "stayed_ncc": round(off[4], 3), "box": [int(x) for x in box],
            "why": ("the whole device moved" if sure else "the whole device may have moved")
            + ": its stored points are where its parts were, locate the part again"}


def record_reading(rec):
    """One word for a whole record: device_moved, changed (some part yes), unchanged (every judged part a solid
    no), unclear (anything else, also nothing judged)."""
    if rec.get("device_moved"):
        return "device_moved"
    marks = [i.get("changed") for i in rec.get("at") or []]
    if "yes" in marks:
        return "changed"
    if marks and all(m == "no" for m in marks) and not rec.get("light_changed"):
        return "unchanged"
    return "unclear"


def _dist(a, b):
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def item_for(rec, part=None, points=(), reach_mm=None):
    """(item, how): the record's at item for a part: by the part's name ("part"), else the item whose own point
    lies nearest one of points within reach_mm and whose label names no sheet part ("near": the operator's own
    label, or the act's own point, where the act or the evidence was); (None, None) when the record judged
    neither."""
    items = rec.get("at") or []
    if part is not None:
        it = next((i for i in items if i.get("part") == part), None)
        if it is not None:
            return it, "part"
    best = None
    for i in items:
        if i.get("point_mm") is None or reach_mm is None or i.get("part") is not None:
            continue                       # another sheet part's reading is never borrowed for this one
        for p in points:
            if p is None:
                continue
            d = _dist(i["point_mm"], p)
            if d <= reach_mm and (best is None or d < best[0]):
                best = (d, i)
    return (best[1], "near") if best is not None else (None, None)


def reading(rec, part=None, points=(), reach_mm=None):
    """{"answer": one of ANSWERS, "record", "at", "how", "solid", "elsewhere", "why"} for a part (or, with no part
    and no points, for the whole record). rec None: not_judged. A consumer reads answer alone: "unchanged" is said
    only for a solid no with the light the same; "elsewhere" counts the changes the record saw on the device away
    from every judged point (a no with changes elsewhere may be a point that is not on the part)."""
    if not isinstance(rec, dict):
        return {"answer": "not_judged", "record": None, "why": "no same-view record"}
    base = {"record": rec.get("id"), "elsewhere": int(rec.get("elsewhere") or 0)}
    if part is None and not points:
        return {"answer": record_reading(rec), **base, "why": (rec.get("device_moved") or {}).get("why")}
    it, how = item_for(rec, part, points, reach_mm)
    if it is None:
        return {"answer": "not_judged", **base, "why": "the record judged no point of that part"}
    out = {**base, "at": it.get("label"), "how": how, "solid": bool(it.get("solid"))}
    if rec.get("device_moved") and it.get("changed") == "unclear":
        return {"answer": "device_moved", **out, "why": rec["device_moved"].get("why")}
    if it.get("changed") == "yes":
        return {"answer": "changed", **out, "why": None}
    if it.get("changed") == "no" and it.get("solid") and not rec.get("light_changed"):
        return {"answer": "unchanged", **out, "why": None}
    return {"answer": "unclear", **out, "why": it.get("why") or "not decided"}


def elsewhere(described, dbox, specs, overlap):
    """How many described changes lie on the device (its box) and at no judged target."""
    if dbox is None:
        return 0
    dev = {"box": [float(x) for x in dbox]}
    return sum(1 for d in described if overlap(d["box_px"], dev) and not any(overlap(d["box_px"], s) for s in specs))


def fork(ex, items, rid, dev):
    """What a compare offers after changed yes at a deciding part (text only, naming loaded tools). A claim is
    offered only for a goal of ONE step, with no device move, and, for a part that shows its state by colour, only
    when the colour seen after is the goal's colour: a change is not the goal state (a light that went dark, a part
    opened in a task that goes on to close it). Otherwise the next thing to do, never claim."""
    doc, task = pose.doc(ex), (getattr(ex, "context", None) or {}).get("task") or {}
    if doc is None or dev is not None:
        return None
    try:
        steps = [s for s in sheet_words.steps(task.get("instruction") or "", doc) if s.get("deciding")]
    except Exception:          # a sheet or sentence this reader cannot parse offers nothing
        return None
    deciding = {s["deciding"] for s in steps}
    hit = next((i for i in items if i["changed"] == "yes" and i.get("part") in deciding), None)
    if hit is None:
        return None
    part = next((p for p in pose.parts_of(ex) if p["name"] == hit["part"]), None)
    names = set(ex.primitive_names())
    again = "push again" if "push" in names else "press again"
    side = "view another side" if "view" in names else "look from another side"
    tell = (part or {}).get("how_to_tell") or f"the {hit['part']} in its goal state"
    if len(steps) > 1:
        return {"if_seen_shows": tell, "then": "do the task's next step (claim only when its last step shows too)",
                "else": f"{again} or {side}"}
    goal = steps[0]
    if goal.get("done_class") == "cue" and goal.get("colour"):
        lit = str(goal.get("state") or "").lower() in LIT_WORDS
        if not colour_words.matches(hit.get("colour_after"), bool(hit.get("glowing")), goal["colour"], lit):
            return {"seen_shows": f"the {hit['part']} now reads {hit.get('colour_after')}, and {goal['state']} "
                                  f"shows as {goal['colour']}: changed, but not to the goal state",
                    "then": again, "else": side}
    return {"if_seen_shows": tell, "then": f"claim, citing {rid}", "else": f"{again} or {side}"}
