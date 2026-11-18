# SPDX-License-Identifier: AGPL-3.0-only
"""Which space the looks show free.

A point is SHOWN FREE when a look of the scene AS IT STANDS (taken after the last act that changed the scene,
ex._scene_t: the last control period in which a guard read a load over its threshold or the jaws were told to
move; and taken after the arm's last motion before the present call began, ex._look_since: the design rulings) has it in
view and sees past it: along the rays through a ball of the stated radius around the point, the
surface the camera's depth picture shows lies farther than the point by more than that radius and BAND times the
look's own stated depth error at that range. Everything else is not shown free: space out of every view, space
hidden behind a surface (the robot's own body at the time of the look is such a surface), space within the look's
error of a surface, space a look showed only before the scene changed, and every space when no look with depth is
held. There a contact can begin.

Read only: the looks are reached through see_spot.look_of and their clouds through see_spot.cloud_of (frozen for
this pass); no control period runs here, nothing is stored on a look but a small cache of its own, and nothing is
typed: the radius is the caller's (a body's own size), the error is the look's stated one (none stated: one pixel
footprint at that range), the band is BAND of rcpx/bounds_ratios.py.

    looks_now(ex)                                  -> [(look id, record, view, depth picture, own, body)], newest first
    shown_free(ex, points_m, radius_m)             -> (free: bool per point, why: text per point)
    shows_in(ex, points_m, radii_m, since_t)       -> bool per ball: a look taken after since_t shows a surface in it
    first_contact(ex, p0_m, d, reach_m, half_m)    -> {"at_m", "free_to_m", "sigma_m", "look", "why", "surface"} or None
    saved_by_a_look(ex)                            -> True when no look with depth of the scene as it stands is held
"""

import math

import numpy as np

from rcpx.bounds_ratios import BAND
from rcpx.r2 import see_spot

MM = 1000.0


def _records(ex):
    """Every look record this session holds, (key, record), read only."""
    scene = ((getattr(ex, "state", None) or {}).get("scene_camera") or {}).get("looks", {})
    own = getattr(ex, "_looks", None) or {}
    return [(str(k), scene[k]) for k in scene] + [(str(k), own[k]) for k in own if str(k) not in scene]


def looks_now(ex):
    """The looks with depth of the scene as it stands, newest first: [(id, record, view, depth, own, body)]; own:
    the pixels that show the robot's own body as it stood then; body: its capsules at that pose. A look counts
    when it was taken after the last act that changed the scene (ex._scene_t) and, across calls, after the arm's
    last motion before this call began (ex._look_since, the design rulings: the frame the call is planned on)."""
    since = [float(t) for t in (getattr(ex, "_scene_t", None), getattr(ex, "_look_since", None)) if t is not None]
    since = max(since) if since else None
    out = []
    for key, _ in _records(ex):
        key, rec, _ = see_spot.look_of(ex, key)
        if rec is None or rec.get("t") is None:
            continue
        if since is not None and float(rec["t"]) < float(since):
            continue                               # seen only before the last act that changed the scene
        got = see_spot.cloud_of(ex, rec)
        if got is None:
            continue
        out.append((float(rec["t"]), key, rec, got[0], np.asarray(rec["depth"], dtype=float), got[3], _body(ex, rec)))
    out.sort(key=lambda row: -row[0])
    return [row[1:] for row in out]


def _body(ex, rec):
    """The robot's own capsules [(p0, p1, radius)] at the joint pose of the look (kept with the look): where its
    body stood then no other thing stood."""
    if rec.get("_free_body") is None:
        try:
            rec["_free_body"] = [(np.asarray(p0, dtype=float), np.asarray(p1, dtype=float), float(r))
                                 for _, p0, p1, r in ex.model.collision_capsules(np.asarray(rec["q"], dtype=float))]
        except Exception:
            rec["_free_body"] = []
    return rec["_free_body"]


def _inside(body, p, radius):
    """True when a ball of that radius around p lies inside one of the capsules."""
    for p0, p1, r in body:
        axis = p1 - p0
        n = float(axis @ axis)
        t = 0.0 if n <= 0.0 else min(1.0, max(0.0, float((p - p0) @ axis) / n))
        if float(np.linalg.norm(p - (p0 + t * axis))) + radius <= r:
            return True
    return False


def saved_by_a_look(ex):
    """True when the session holds no look with depth of the scene as it stands: a look would show free space."""
    return not looks_now(ex)


def _sigma(rec, view, rng):
    """The look's stated depth error at that range (m); none stated: one pixel footprint there."""
    stated = rec.get("depth_stated")
    if not isinstance(stated, dict) or stated.get("sigma_m_at_1m") is None:
        return float(rng) / view.f
    s = float(stated["sigma_m_at_1m"])
    return s * float(rng) * float(rng) if stated.get("grows") == "square" else s


def _window(view, z, own, p, radius):
    """(why not, the depth window, the own-body window, the point's depth, its range) of a ball's picture in one
    look; why not is a text when the look cannot judge the ball (out of view, at the edge, no depth reading)."""
    x, y, zp = view.project(p)
    x, y, zp = float(x), float(y), float(zp)
    if not zp > 0.0 or not view.inside(x, y):
        return "out of its view", None, None, zp, None
    rng = float(np.linalg.norm(np.asarray(p, dtype=float) - view.pos))
    half = int(math.ceil(radius * view.f / zp)) + 1            # the ball's own picture, and one pixel around it
    u, v = int(math.floor(x)), int(math.floor(y))
    if u - half < 0 or v - half < 0 or u + half >= view.w or v + half >= view.h:
        return "at the edge of its view", None, None, zp, rng
    win = z[v - half:v + half + 1, u - half:u + half + 1]
    if not np.all(np.isfinite(win) & (win > 0.0)):
        return "its depth picture has no reading there", None, None, zp, rng
    return None, win, own[v - half:v + half + 1, u - half:u + half + 1], zp, rng


def _one(view, rec, z, own, body, p, radius):
    """(free, why, sigma) of one point in one look. Where the robot's own body stood when the look was taken no
    other thing stood. Where the look shows the robot's own body, that body is no thing in the way: the point
    is free there unless the body stands in front of it by more than the margin (then the point is hidden
    behind the robot). The words tell a surface AT the point from one in front of it (the space behind a
    surface is hidden, not shown full)."""
    if _inside(body, np.asarray(p, dtype=float), radius):
        return True, "", None
    bad, win, mine, zp, rng = _window(view, z, own, p, radius)
    sigma = None if rng is None else _sigma(rec, view, rng)
    if bad is not None:
        return False, bad, sigma
    margin = radius + BAND * sigma
    past = win - zp
    if np.all(np.where(mine, past > -margin, past > margin)):
        return True, "", sigma
    if np.any(~mine & (np.abs(past) <= margin)):
        return False, "it shows a surface there", sigma
    if np.any(mine & ~(past > -margin)) and np.all(mine | (past > margin)):
        return False, "the robot's own body hid it", sigma
    return False, "a surface in front hides it", sigma


def shows_in(ex, points_m, radii_m, since_t):
    """bool per ball: a look of the scene as it stands, taken after since_t (s, per ball), has the ball in view
    and shows a surface that is not the robot's own INSIDE it (nearer than its far side, farther than its near
    side). What the arm passed through and such a look shows full is no longer free (the design rulings (2))."""
    pts = np.asarray(points_m, dtype=float).reshape(-1, 3)
    r = np.broadcast_to(np.asarray(radii_m, dtype=float), (len(pts),))
    t = np.broadcast_to(np.asarray(since_t, dtype=float), (len(pts),))
    out = np.zeros(len(pts), dtype=bool)
    looks = looks_now(ex)
    for i, p in enumerate(pts):
        for key, rec, view, z, own, body in looks:
            if not float(rec["t"]) > float(t[i]):
                break                                          # newest first: the rest are older still
            bad, win, mine, zp, _ = _window(view, z, own, p, float(r[i]))
            if bad is None and np.any(~mine & (np.abs(win - zp) <= float(r[i]))):
                out[i] = True
                break
    return out


def shown_free(ex, points_m, radius_m):
    """(free: bool per point, why: text per point). free: some look of the scene as it stands shows a ball of
    radius_m (one radius, or one per point) around the point free by more than its own stated error. why names
    the look that shows it, or what the newest look that has the point in view says of it."""
    pts = np.asarray(points_m, dtype=float).reshape(-1, 3)
    radii = np.broadcast_to(np.asarray(radius_m, dtype=float), (len(pts),))
    free = np.zeros(len(pts), dtype=bool)
    looks = looks_now(ex)
    why = ["no look with depth of the scene as it stands is held" if not looks else "no look has it in view"] * len(pts)
    for i, p in enumerate(pts):
        for key, rec, view, z, own, body in looks:
            ok, text, _ = _one(view, rec, z, own, body, p, float(radii[i]))
            if ok:
                free[i], why[i] = True, f"look {key}"
                break
            if why[i].startswith("no look has"):
                why[i] = f"look {key}: {text}"
    return free, why


def first_contact(ex, p0_m, d, reach_m, half_m):
    """Along the column of half width half_m from p0_m along d (unit) to reach_m: where a contact can first begin
    by what the looks show, or None when no look with depth of the scene as it stands is held. at_m: the first
    station of the column that is not shown free (reach_m when all of it is); free_to_m: the last station before
    it that is (the column is shown free up to there); sigma_m: the stated depth error of the look that judged
    at_m; look: its id; surface: True when that look shows a surface there (else the space is out of view or
    hidden); why: the words. Stations stand one half width apart: no thing as wide as the column slips between."""
    looks = looks_now(ex)
    if not looks:
        return None
    p0, d = np.asarray(p0_m, dtype=float), np.asarray(d, dtype=float)
    half, reach = float(half_m), float(reach_m)
    n = max(1, int(math.ceil(reach / half))) if half > 0.0 else 1
    stations = [reach * k / n for k in range(1, n + 1)]
    free_to, first = 0.0, None
    for t in stations:
        p = p0 + d * t
        shown, said = False, None
        for key, rec, view, z, own, body in looks:
            ok, text, sigma = _one(view, rec, z, own, body, p, half)
            if ok:
                shown = True
                break
            if said is None or (said[2] is None and sigma is not None):
                said = (key, text, sigma)
        if shown:
            free_to = t
            continue
        first = (t, said)
        break
    if first is None:
        key, rec, view = looks[0][:3]
        sigma = _sigma(rec, view, float(np.linalg.norm(p0 + d * reach - view.pos)))
        return {"at_m": reach, "free_to_m": reach, "sigma_m": sigma, "look": key, "surface": False,
                "why": f"look {key} shows the whole column free"}
    t, (key, text, sigma) = first
    _, rec, _ = see_spot.look_of(ex, key)
    if sigma is None:
        view = see_spot.view_of(rec)
        sigma = _sigma(rec, view, float(np.linalg.norm(p0 + d * t - view.pos)))
    return {"at_m": t, "free_to_m": free_to, "sigma_m": sigma, "look": key, "surface": text.startswith("it shows"),
            "why": f"look {key}: {text} {t * MM:.0f} mm along the way (the column is shown free for {free_to * MM:.0f} mm)"}
