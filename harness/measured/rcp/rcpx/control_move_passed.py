# SPDX-License-Identifier: AGPL-3.0-only
"""Space the session knows free without a look.

(2) PASSED. Where the arm's own bodies or the held thing have been in this session no other thing stands, while
nothing was met and no jaw was told to move since (ex._scene_t, the last act that changed the scene: a contact
may have pushed a thing there, a release may have left one there) and no look taken since shows a surface there
(that veto is the caller's: rcpx/bounds_free.py asks rcpx/r2/see_free.py). The record is the arm's moving
capsules, each grown by its own fit error (the farthest a point of the body's shape stands out of it), at the
poses the arm was READ at, the tip with the tool's own half width, and the ball around what the hand held then. A pose is kept when the arm has moved
more than half the tool's own half width since the last kept one (its joints' travel times the arm's length: an
upper bound of any point's travel), and the poses between two kept ones are filled in at that spacing (the arm
moved from one to the other), so no thing as wide as the tool's half width fits between two kept poses; a ball
counts as passed when it lies inside a kept capsule to within that spacing. Where the arm stands now is always
part of it. A pose read at or before ex._scene_t never counts again and is dropped; a pose the arm cannot have
reached by its own joints since the reading before (a body set anew) starts the record again.

THE PASSAGE FREES SPACE ONLY INSIDE THE CALL THAT MADE IT: the world can change between two calls (a
thing set there, a person, another agent) and no reading of the robot's own sees that. When an operator's call
begins (open_call, from the call's own entry in rcpx/control_base.py and rcpx/r2/core.py) every pose kept before
it is dropped; the pose the arm stands at as the call begins is its first (the arm's own bodies stand there now).
From a call's end, space is free only by a look taken after the arm's last motion (ex._look_since, the time of the
last moving control period before the call began; rcpx/r2/see_free.py reads no look taken before it).

A part the session's own act set moving (a press or a touch that ended in a give or moved the part, a push that
moved what was there) VOIDS the passage it can reach (the rule the real arm's adapter has too): the
side of the contact the part went, ahead of it along the act, as far as the part can have gone (all of that side:
no extent of the part is known here); the space behind the contact, where the arm's own bodies came from, stands.
A ball that reaches into a void counts as passed by no pose, kept before or after the act: the part may still be
moving there by its own motion (a lid that opens on its motor). Only a look of the scene as it stands frees it
(the look source of rcpx/bounds_free.py). The receipt of the act names the part and the void.

(3) STATED. A zone of the owner's site statement that says "free": true ({"box_mm": [lo, hi], "free": true},
frame B) is free for a ball that lies wholly inside it.

Nothing is typed, no simulator truth is read and no control period runs here.

    open_call(ex)                    an operator's call begins: the passage of earlier calls is dropped
    note(ex, s)                      one reading: keep its pose when the arm moved
    passed(ex, points_m, radii_m)    -> per ball the time (s) of the newest kept pose that held it; nan: none did
    void(ex, point_m, along, why, part) -> the receipt's "void" block: a part set moving voids what it can reach
    voided(ex, points_m, radii_m)    -> bool per ball: it reaches into a void
    stated(ex, points_m, radii_m)    -> bool per ball: it lies inside a zone the site states free
"""

import collections

import numpy as np

from rcpx.bounds_ratios import GIVE

MM = 1000.0


def _record(ex):
    rec = ex.__dict__.get("_passed")
    if rec is None:
        rec = ex.__dict__["_passed"] = {"rows": collections.deque(), "q": None, "spacing": None, "fit": None}
    return rec


def spacing(ex):
    """The spacing of the kept poses (m): half the tool's own half width; a tool with none stated: half the
    radius of the arm's thinnest capsule."""
    rec = _record(ex)
    if rec["spacing"] is None:
        w = 0.0
        try:
            w = float(ex.bounds.robot.w_tool())
        except Exception:
            w = 0.0
        if not w > 0.0:
            w = min((float(c[3]) for c in ex.model.collision_capsules(ex.model.q_home)[1:]), default=0.0)
        rec["spacing"] = GIVE * w
    return rec["spacing"]


def _row(ex, q, t):
    """One kept pose: its time, the capsules (ends and radii grown by the fit error) and their box."""
    rec = _record(ex)
    if rec["fit"] is None:
        try:
            rec["fit"] = dict(ex.model.e_fit())
        except Exception:
            rec["fit"] = {}
    q = np.asarray(q, dtype=float)
    caps = ex.model.collision_capsules(q)[1:]
    p0 = [np.asarray(c[1], dtype=float) for c in caps]
    p1 = [np.asarray(c[2], dtype=float) for c in caps]
    r = [float(c[3]) + float(rec["fit"].get(c[0]) or 0.0) for c in caps]
    tip, R = ex.model.fk_tip(q)
    w = 0.0
    try:
        w = float(ex.bounds.robot.w_tool())
    except Exception:
        w = 0.0
    if w > 0.0:                                  # the tip with the tool's own half width has been there too
        p0.append(np.asarray(tip, dtype=float))
        p1.append(np.asarray(tip, dtype=float))
        r.append(w)
    held = None
    try:
        held = ex._held
    except Exception:
        held = None
    extent = None if not isinstance(held, dict) else held.get("extent_m")
    if extent is not None and len(extent):
        pts = np.asarray(tip, dtype=float) + np.asarray(extent, dtype=float) @ np.asarray(R, dtype=float).T
        mid = pts.mean(axis=0)
        p0.append(mid)
        p1.append(mid)
        r.append(float(np.max(np.linalg.norm(pts - mid, axis=1))))
    p0, p1, r = np.asarray(p0, dtype=float).reshape(-1, 3), np.asarray(p1, dtype=float).reshape(-1, 3), np.asarray(r)
    lo = np.minimum(p0, p1).min(axis=0) - r.max() if len(r) else np.zeros(3)
    hi = np.maximum(p0, p1).max(axis=0) + r.max() if len(r) else np.zeros(3)
    return {"t": float(t), "p0": p0, "p1": p1, "r": r, "lo": lo, "hi": hi}


def _drop(ex, rows):
    since = getattr(ex, "_scene_t", None)
    while rows and since is not None and rows[0]["t"] <= float(since):
        rows.popleft()


def _continuous(ex, seen, q, t):
    """True when the arm can have moved from its last reading (seen: time, pose) to this one by its own joints
    (no joint travelled farther than its own speed allows in the time between): a pose that jumped (a body set
    anew) passed nothing."""
    if seen is None or seen[1].shape != q.shape:
        return False
    try:
        qd = np.asarray(ex.bounds.qd_max(), dtype=float).reshape(-1)
    except Exception:
        return True
    if qd.shape != q.shape:
        return True
    return bool(np.all(np.abs(q - seen[1]) <= qd * max(float(t) - seen[0], float(ex.dt)) * (1.0 + GIVE)))


def open_call(ex):
    """An operator's call begins. Every pose kept in an earlier call is dropped (the arm's own passage
    frees space only inside the call that made it); the pose the arm stands at now is the record's first. The
    looks that free space in this call are those taken after the arm's last motion before it (ex._look_since =
    ex._moved_t, the time of the last moving control period; None: the arm has not moved this session). Moves
    nothing and runs no control period."""
    rec = _record(ex)
    rec["rows"].clear()
    rec["here"] = None
    ex._look_since = getattr(ex, "_moved_t", None)
    last = getattr(ex, "_last", None)
    if last is None or getattr(last, "q", None) is None:
        rec["q"] = None
        return
    q = np.asarray(last.q, dtype=float)
    rec["q"] = q.copy()
    rec["rows"].append(_row(ex, q, float(last.t)))


def note(ex, s):
    """One reading of the arm: keep its pose when the arm has moved more than the spacing since the last kept
    one, with the poses between the two (the arm moved from one to the other: its capsules' ends carried
    straight across, one every spacing), so that no gap wider than the spacing stays in what it swept. Called
    for every new reading (control_sense._sense); it moves nothing."""
    rec = _record(ex)
    rows = rec["rows"]
    _drop(ex, rows)
    q = np.asarray(s.q, dtype=float)
    seen, rec["seen"] = rec.get("seen"), (float(s.t), q.copy())
    step = spacing(ex)
    if rows and rec["q"] is not None and rec["q"].shape == q.shape:
        if float(np.sum(np.abs(q - rec["q"]))) * float(ex.model.length_scale) <= step:
            return
    if not _continuous(ex, seen, q, s.t):
        rows.clear()                             # the body was set anew: what it swept before is another scene's
    rec["q"] = q.copy()
    row = _row(ex, q, s.t)
    last = rows[-1] if rows else None
    if last is not None and step > 0.0 and last["p0"].shape == row["p0"].shape:
        gap = max(float(np.max(np.linalg.norm(row[k] - last[k], axis=1))) for k in ("p0", "p1"))
        n = int(np.ceil(gap / step))
        for j in range(1, n):                    # between two kept poses: the time of the earlier one
            f = j / n
            p0, p1 = last["p0"] + f * (row["p0"] - last["p0"]), last["p1"] + f * (row["p1"] - last["p1"])
            r = np.minimum(last["r"], row["r"])
            rows.append({"t": last["t"], "p0": p0, "p1": p1, "r": r,
                         "lo": np.minimum(p0, p1).min(axis=0) - r.max(), "hi": np.maximum(p0, p1).max(axis=0) + r.max()})
    rows.append(row)


def _dist(x, p0, p1):
    """N x M distances of the points x to the segments p0-p1."""
    ab = p1 - p0
    n = np.einsum("mk,mk->m", ab, ab)
    ax = x[:, None, :] - p0[None, :, :]
    t = np.einsum("nmk,mk->nm", ax, ab) / np.where(n > 0.0, n, 1.0)[None, :]
    t = np.clip(np.where(n[None, :] > 0.0, t, 0.0), 0.0, 1.0)
    return np.linalg.norm(ax - t[:, :, None] * ab[None, :, :], axis=2)


def passed(ex, points_m, radii_m):
    """Per ball (centre, radius): the time (s) of the newest kept pose at which the ball lay inside one of the
    arm's capsules or the held thing's ball, to within the spacing of the kept poses; nan where none held it."""
    x = np.asarray(points_m, dtype=float).reshape(-1, 3)
    r = np.broadcast_to(np.asarray(radii_m, dtype=float), (len(x),)).astype(float)
    out = np.full(len(x), np.nan)
    if not len(x):
        return out
    rec = _record(ex)
    rows = rec["rows"]
    _drop(ex, rows)
    last = getattr(ex, "_last", None)
    here = []
    if last is not None:                                        # where the arm stands now
        if rec.get("here") is None or rec["here"]["t"] != float(last.t):
            rec["here"] = _row(ex, last.q, last.t)
        here = [rec["here"]]
    tol = spacing(ex)
    lo, hi = (x - r[:, None]).min(axis=0), (x + r[:, None]).max(axis=0)
    for row in here + list(reversed(rows)):                     # the newest first: a ball keeps its newest pose
        if np.any(row["hi"] < lo - tol) or np.any(row["lo"] > hi + tol) or not len(row["r"]):
            continue
        todo = np.flatnonzero(np.isnan(out))
        if not len(todo):
            break
        d = _dist(x[todo], row["p0"], row["p1"])
        inside = np.any(d + r[todo, None] <= row["r"][None, :] + tol, axis=1)
        out[todo[inside]] = row["t"]
    out[voided(ex, x, r)] = np.nan                              # a part set moving may stand there now
    return out


def void(ex, point_m, along, why, part=None):
    """The act at point_m (m) along `along` (the act's own travel; None: unknown) set a part moving. Every
    ball on the side ahead of the point along it (every side, with no direction) is voided for the passage from
    now on. Returns the receipt's block {"part", "at_mm", "along", "side", "why", "until"}."""
    rec = _record(ex)
    p = np.asarray(point_m, dtype=float).reshape(-1)[:3]
    n = None
    if along is not None:
        a = np.asarray(along, dtype=float).reshape(-1)[:3]
        size = float(np.linalg.norm(a))
        n = a / size if size > 0.0 else None
    rec.setdefault("voids", []).append({"p": p, "n": n, "why": why, "part": part})
    at = [round(float(x) * MM, 1) for x in p]
    side = ("every side of it: the act's direction is not known" if n is None else
            "the side ahead of it along the act, as far as the part can go (its extent is not known)")
    return {"part": part or "the part met there", "at_mm": at, "along": None if n is None else
            [round(float(x), 3) for x in n], "side": side, "why": why,
            "until": "a look of the scene as it stands shows that space free; the arm's passage frees none of it"}


def voided(ex, points_m, radii_m):
    """bool per ball (centre m, radius m): some part of it lies on the voided side of a part the session set
    moving."""
    x = np.asarray(points_m, dtype=float).reshape(-1, 3)
    r = np.broadcast_to(np.asarray(radii_m, dtype=float), (len(x),)).astype(float)
    out = np.zeros(len(x), dtype=bool)
    for v in _record(ex).get("voids", ()):
        out |= np.ones(len(x), dtype=bool) if v["n"] is None else (x - v["p"]) @ v["n"] > -r
    return out


def stated(ex, points_m, radii_m):
    """bool per ball: it lies wholly inside a zone the owner's site statement calls free."""
    x = np.asarray(points_m, dtype=float).reshape(-1, 3)
    r = np.broadcast_to(np.asarray(radii_m, dtype=float), (len(x),)).astype(float)
    out = np.zeros(len(x), dtype=bool)
    site = (getattr(ex, "cfg", None) or {}).get("site") or {}
    for zone in site.get("zones", []) if isinstance(site, dict) else []:
        if not zone.get("free") or zone.get("box_mm") is None:
            continue
        lo, hi = np.asarray(zone["box_mm"], dtype=float) / MM
        out |= np.all(x - r[:, None] >= lo, axis=1) & np.all(x + r[:, None] <= hi, axis=1)
    return out
