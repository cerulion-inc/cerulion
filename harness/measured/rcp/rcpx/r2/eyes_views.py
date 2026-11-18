# SPDX-License-Identifier: AGPL-3.0-only
"""eyes_views: the depth of a chosen candidate from a second look the robot takes itself (integrator, 2026-11-12).

Why. choose with depth auto measured a candidate's point through the locate tool, and with no locate loaded it
felt down the candidate's line of sight. The felt point is where the TOOL first met the thing coming along the
camera's line: on a part that sits on a side face, seen from above, that is the part's upper rim, not the part
(the design notes, the integrator's lines; measured 16 mm off on a 20 mm part). This file gives choose
a point the way an operator makes one with the tools every surface has: the same thing in two looks from two places,
two marks, triangulate. The robot takes the second look itself.

What it does (run): 1. the first line of sight is the candidate's pixel in its own look; 2. when the camera has
not moved far enough since that look, ONE guarded move_tip sideways (across the line of sight, level), the other
way when the first is refused or stopped short; 3. a look, and the words-first list of that look (the same vision
equipment, the same merge: rcpx/r2/eyes.py list_); 4. the row of the new list that is the same thing: found by the
same part's word (else of the same colour word), and the one whose line of sight passes nearest the first line;
it is accepted only when the two lines pass within MISS_MAX_MM of each other and cross in front of both cameras,
above the standing surface; 5. two marks under a label of its own and triangulate, the session's own primitives, so
the entry is the entry every other tool knows. Nothing is touched. The camera stays where the second look was taken.

Safety and bounds: the one move is a public primitive called after ex.gates (the dark gate, the path gates and the
person gates of an operator's own move); it is at most BASE_MAX_SHARE of the arm's reach; no contact is sought and a
move that meets something stops as a guarded move does. No force is written anywhere. The marks are the robot's own
(a label no operator uses), so they are not the operator's pointing and the gauge does not read them.

Returns {"point_mm", "uncertain_mm", "error_mm", "id", "look2", "px2", "moved_mm", "legs", "notes"} when a point
was made, else {"none": why, "legs", "notes"} (the caller then feels along the line, or says so), or {"refused": body}
when a gate refused the move and no second view could be had another way.
"""
import math

import numpy as np

MM = 1000.0
BASE_SHARE = 0.25          # the sideways move, as a share of the distance from the camera to where the line ends
BASE_MAX_SHARE = 1.0 / 6   # and never more than this share of the arm's reach
MOVED_ENOUGH = 0.5         # a camera that already stands this share of that move away needs no move
MISS_MAX_MM = 10.0         # two lines of sight that pass farther apart than this are not the same thing
MISS_KEPT_MULT = 4.0       # a part the robot only keeps off may be placed more loosely: the miss widens its keep-off
ANGLE_MIN_DEG = 3.0        # two lines closer to parallel than this give no depth
FLOOR_TOL_MM = 15.0        # the crossing may lie this far under the standing surface (the lines' own error)


def _unit(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else None


def _cross_point(o1, d1, o2, d2):
    """(miss in m, midpoint, t1, t2, angle in deg) of two lines o + t d (unit d), or None when near parallel."""
    w = o1 - o2
    b = float(d1 @ d2)
    den = 1.0 - b * b
    ang = math.degrees(math.acos(max(-1.0, min(1.0, b))))
    if den < 1e-9:
        return None
    t1 = (b * float(d2 @ w) - float(d1 @ w)) / den
    t2 = (float(d2 @ w) - b * float(d1 @ w)) / den
    p1, p2 = o1 + t1 * d1, o2 + t2 * d2
    return float(np.linalg.norm(p1 - p2)), 0.5 * (p1 + p2), t1, t2, ang


def _same_thing(row, other):
    """Could `other` (a row of the second list) be the thing `row` is: the same part word found it, else the same
    colour word on a row no word found."""
    if row.get("found_by"):
        return other.get("found_by") == row.get("found_by")
    return not other.get("found_by") and other.get("colour") is not None and other.get("colour") == row.get("colour")


def _call(ex, legs, name, **kw):
    """One public primitive after its gates; (result, refusal body or None). A leg is recorded."""
    g = ex.gates(name, kw)
    if g is not None:
        body = g.get("refused") if isinstance(g, dict) and isinstance(g.get("refused"), dict) else {"check": "gate"}
        legs.append({"leg": name, "stopped": "refused", "check": body.get("check")})
        return None, body
    t0 = float(ex.sim_time())
    r = getattr(ex, name)(**kw)
    r = r if isinstance(r, dict) else {}
    body = r.get("refused") if isinstance(r.get("refused"), dict) else None
    legs.append({"leg": name, "stopped": "refused" if body else r.get("stopped") or "done",
                 "seconds": round(float(ex.sim_time()) - t0, 1)})
    return r, body


def run(ex, st, list_fn, key, rec, u, v, row, n, label, kept=False):
    """See the module text. list_fn(ex, look_id) makes the list of a look and returns its result (eyes.list_).
    kept: the row is one the manual keeps the robot off; it is matched among the kept-off rows of the second list
    (a part to act on is never matched to one of those) and is only placed, never touched."""
    legs, notes = [], []
    q1 = np.asarray(rec["q"], dtype=float)
    o1, d1 = ex.model.pixel_ray(q1, u, v)
    o1, d1 = np.asarray(o1, dtype=float), _unit(d1)
    if d1 is None:
        return {"none": "that pixel has no line of sight", "legs": legs, "notes": notes}
    hit = ex._floor_hit(q1, u, v)
    reach = float((ex.model.reach() or {}).get("max_reach_mm") or 0.0) / MM
    far = float(np.linalg.norm(np.asarray(hit, dtype=float) - o1)) if hit is not None else reach
    want = min(BASE_SHARE * far, BASE_MAX_SHARE * reach) if reach > 0 else BASE_SHARE * far
    if not want > 0.0:
        return {"none": "the line of sight ends nowhere the robot knows", "legs": legs, "notes": notes}
    try:
        cam_now = np.asarray(ex.model.pixel_ray(np.asarray(ex._sense().q, dtype=float), u, v)[0], dtype=float)
    except Exception:
        cam_now = o1
    moved, refused = float(np.linalg.norm(cam_now - o1)), None
    if moved < MOVED_ENOUGH * want:
        side = _unit(np.cross(d1, [0.0, 0.0, 1.0]))
        side = side if side is not None else np.array([0.0, 1.0, 0.0])
        for sign in (1.0, -1.0):                       # across the line of sight, level; the other way when refused
            step = sign * side * want * MM
            r, body = _call(ex, legs, "move_tip", dx=round(float(step[0]), 1), dy=round(float(step[1]), 1), dz=0.0,
                            frame="base")
            if body is not None:
                refused = body
                continue
            try:
                cam_now = np.asarray(ex.model.pixel_ray(np.asarray(ex._sense().q, dtype=float), u, v)[0],
                                     dtype=float)
            except Exception:
                break
            moved = float(np.linalg.norm(cam_now - o1))
            if moved >= MOVED_ENOUGH * want:
                break
        if moved < MOVED_ENOUGH * want:
            if refused is not None and not any(g.get("stopped") != "refused" for g in legs):
                return {"refused": refused, "legs": legs, "notes": notes}
            return {"none": "the camera could not be moved far enough to the side for a second view", "legs": legs,
                    "notes": notes}
    lk, body = _call(ex, legs, "look")
    if body is not None:
        return {"refused": body, "legs": legs, "notes": notes}
    key2 = lk.get("look_id")
    rec2 = (getattr(ex, "_looks", None) or {}).get(key2)
    if rec2 is None or rec2.get("can_see") == "dark":
        return {"none": "the second look could not see", "legs": legs, "notes": notes}
    res = list_fn(ex, key2)
    if not isinstance(res, dict) or "refused" in res:
        return {"none": "the second look could not be listed", "legs": legs, "notes": notes, "look2": key2}
    lst2 = (st.get("lists") or {}).get(key2) or {}
    q2 = np.asarray(rec2["q"], dtype=float)
    best = None
    for other in lst2.get("items") or ():
        if (bool(other.get("keep_off")) != bool(kept)) or not _same_thing(row, other) or not other.get("point_px"):
            continue
        u2, v2 = (float(x) for x in other["point_px"][:2])
        o2, d2 = ex.model.pixel_ray(q2, u2, v2)
        got = _cross_point(o1, d1, np.asarray(o2, dtype=float), _unit(d2))
        if got is None:
            continue
        miss, mid, t1, t2, ang = got
        if t1 <= 0 or t2 <= 0 or ang < ANGLE_MIN_DEG:
            continue
        if best is None or miss < best[0]:
            best = (miss, mid, u2, v2, other.get("n"), ang)
    if best is None:
        return {"none": f"nothing in {key2} is the same thing by its word or colour", "legs": legs, "notes": notes,
                "look2": key2}
    miss, mid, u2, v2, n2, ang = best
    floor = float(ex._floor_at(mid))
    if miss * MM > MISS_MAX_MM * (MISS_KEPT_MULT if kept else 1.0) or float(mid[2]) < floor - FLOOR_TOL_MM / MM:
        return {"none": f"the nearest match in {key2} (number {n2}) passes {miss * MM:.0f} mm from the first line of "
                "sight: not surely the same thing", "legs": legs, "notes": notes, "look2": key2}
    tmp = f"robot view of {key} {n if n is not None else 'px'}"      # a label no operator uses
    for look_id, uu, vv in ((key, u, v), (key2, u2, v2)):
        m = ex.mark(look_id=look_id, u=float(uu), v=float(vv), label=tmp)
        if isinstance(m, dict) and "refused" in m:
            return {"none": "a mark was refused: " + str((m["refused"] or {}).get("why"))[:100], "legs": legs,
                    "notes": notes, "look2": key2}
    t = ex.triangulate(label=tmp)
    if not isinstance(t, dict) or "refused" in t or not t.get("id") or t.get("point_mm") is None:
        why = ((t or {}).get("refused") or {}).get("why") if isinstance(t, dict) else None
        return {"none": "the two views gave no point" + (f": {str(why)[:100]}" if why else ""), "legs": legs,
                "notes": notes, "look2": key2}
    unc = t.get("depth_uncertainty_mm")
    unc = float(unc) if isinstance(unc, (int, float)) else max(MISS_MAX_MM, float(t.get("error_mm") or 0.0))
    if kept:
        unc = max(unc, miss * MM)          # a looser placement keeps the robot farther off, never nearer
    ex.world.update(t["id"], label=label)
    return {"id": t["id"], "point_mm": t["point_mm"], "uncertain_mm": round(unc, 1),
            "error_mm": t.get("error_mm"), "angle_deg": round(ang, 1), "look2": key2,
            "px2": [round(u2, 1), round(v2, 1)], "n2": n2, "moved_mm": round(moved * MM, 0), "legs": legs,
            "notes": notes}


__all__ = ["run"]
