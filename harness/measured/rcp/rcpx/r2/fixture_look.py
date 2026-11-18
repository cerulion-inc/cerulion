# SPDX-License-Identifier: AGPL-3.0-only
"""What the newest look shows beside a part (package fixture, round six): three readings of a depth picture that
order what work_part does. None of them moves the arm, costs a control period or refuses anything: a look is
MEASURED evidence, it decides which side a hand comes from and which way it takes, never whether the tool acts.

- points(ex): the world points of the newest look with depth (the robot's own body left out by the look itself),
  the look's id, the depth's error there and one pixel's footprint. Kept for the call.
- side(ex, p, across, half_m, reach_m): on which side of a part the thing it belongs to stands. The part's own
  face is not what a camera over it shows: it shows the part's top, and right beside it, on one side, something
  that stands higher or as high (the thing the part is on: a front, a body) and on the other side something lower
  (the room the fingers have). A push goes from the lower side into the higher one, a pull draws the part out to
  the lower one. Right beside: within the room the fingers themselves stand in (reach_m), so that a thing which
  stands a little off the part on its free side is not taken for the thing the part is on. The sides are told
  apart only where the lower one stands under the part itself and the two differ by more than the look can tell
  (BAND sigma of its depth, or one footprint of a pixel).
- under(ex, q): the bodies of the arm at pose q that stand under what the look shows. The look is read as a height
  field: what lies under a seen point is not free (a camera sees a thing's top, not the room below it). Each
  body is read at the ends and the middle of its own capsule.
- top_middle(ex, p, reach_m, sigma_m), part_top(...): the middle and the top of what stands highest about a
  pointed point (a part that is turned from above is gripped about its own middle, at its own top), and whether
  the point lies under it in plan (a pixel on the part's edge or side is on the part).
- above(ex, top_h), face_ahead(...), capsules(ex, q), first_meet(...): what a push's hold is chosen by (the hand
  that pushes a part into the thing it belongs to must not meet what stands over the part on the way). above:
  the look's points over the part's top by more than the look can tell. face_ahead: how far along the push the
  face of the thing the part goes into stands: the nearest place along the stroke where what the look shows over
  the part's top reaches down to that top (a face; a handle seen only by its top does not reach down and is
  passed under). first_meet: how far the arm's capsules, moved along the stroke, go before one comes within its
  radius and what the look can tell of one of those points. These read the points themselves, never a height
  field: what stands over a part (a handle) leaves the room under it as the look shows it.
Nothing here reads truth or names a robot. SI inside.
"""

import numpy as np

from rcpx.bounds_ratios import BAND
from rcpx.r2.fixture_probe import HALF, TINY, bounds_of, laws_mm, unit


def up_of(ex):
    """The unit direction away from the support (its stated or touched plane's normal; else against gravity)."""
    support = bounds_of(ex).space.support()
    if support is not None:
        return unit(support[0][0])
    return -unit(ex.model.mj.opt.gravity)


def level_axes(up):
    """(e1, e2): the level axes every bearing of a look is given in (rcpx/r2/see_cloud.py axes_of)."""
    try:
        from rcpx.r2.see_cloud import axes_of
        return axes_of(up)
    except ImportError:
        e1 = unit(np.eye(len(up))[0] - float(np.eye(len(up))[0] @ up) * up)
        return e1, np.cross(up, e1)


def points(ex):
    """(points N x 3, the look's id, the depth's error at the look's middle range m, one pixel's footprint there
    m) of the newest look with depth, or (None, None, 0.0, 0.0). Kept for the call (ex._fx)."""
    keep = getattr(ex, "_fx", None)
    if isinstance(keep, dict) and "look_points" in keep:
        return keep["look_points"]
    out = (None, None, 0.0, 0.0)
    try:
        from rcpx.r2 import see_spot
    except ImportError:
        return out
    scene = (ex.state.get("scene_camera") or {}).get("looks", {})
    order = list(scene.items())[::-1] + list((getattr(ex, "_looks", None) or {}).items())[::-1]
    for key, rec in order:
        try:
            got = see_spot.cloud_of(ex, rec)
        except Exception:      # a look that cannot be read shows nothing
            got = None
        if got is None:
            continue
        view, cloud, valid, _ = got
        if not valid.any():
            continue
        pts = cloud[valid]
        rng = float(np.median(np.linalg.norm(pts - view.pos, axis=1)))
        foot = rng / view.f
        out = (pts, str(key), max(float(see_spot._sigma_at(rec, rng, foot)[0]), TINY), foot)
        break
    if isinstance(keep, dict):
        keep["look_points"] = out
    return out


def _top(heights, sigma):
    try:
        from rcpx.r2.see_cloud import top_of
        return float(top_of(heights, sigma))
    except ImportError:
        return float(np.max(heights))


def side(ex, p, across, half_m, reach_m):
    """(+1, -1 or 0, the look's id): +1 when the newest look with depth shows the thing a part at p belongs to on
    the side the unit level vector `across` points to (what stands there within reach_m of p, past half_m from
    the part's own middle line, is the higher), -1 when on the other side, 0 when the look tells no side: no look
    with depth, nothing seen on one side, the lower side not under the part itself, or the two sides within what
    the look can tell (BAND sigma of its depth or one footprint)."""
    pts, look, sigma, foot = points(ex)
    if pts is None:
        return 0, None
    up = up_of(ex)
    rel = pts - np.asarray(p, dtype=float)
    h = rel @ up
    flat = rel - np.outer(h, up)
    t = flat @ unit(across)
    near = np.linalg.norm(flat, axis=1) <= reach_m
    tops = []
    for sign in (1, -1):
        on = near & (sign * t > half_m)
        if not on.any():
            return 0, look
        tops.append(_top(h[on], sigma))
    tell = max(BAND * sigma, foot)
    if abs(tops[0] - tops[1]) <= tell or min(tops) >= -tell:
        return 0, look
    return (1 if tops[0] > tops[1] else -1), look


def under(ex, q, skip=None):
    """The bodies of the arm at q that stand under what the newest look with depth shows: [(body, "look <id>",
    how far its lowest side stands under what is seen there, m)], the worst first; [] with no such look. A body
    is read at the two ends and the middle of its capsule: the look's points within the capsule's radius and one
    footprint of that point in plan, and their top against the point's own lowest side. What the look shows no
    higher than the support plus what it can tell is the support itself and is left out. skip: (point, radius
    m): what the look shows within that radius of the point in plan is left out (the part the hand is meant to
    meet)."""
    pts, look, sigma, foot = points(ex)
    if pts is None:
        return []
    B = bounds_of(ex)
    up = up_of(ex)
    tell = max(BAND * sigma, foot)
    h = pts @ up
    keep = np.ones(len(pts), dtype=bool)
    support = B.space.support()
    if support is not None:
        n = np.asarray(support[0][0], dtype=float)
        keep &= h > float(support[0][1]) / float(np.linalg.norm(n)) + tell
    flat = pts - np.outer(h, up)
    if skip is not None:
        c = np.asarray(skip[0], dtype=float)
        keep &= np.linalg.norm(flat - (c - float(c @ up) * up), axis=1) > skip[1]
    if not keep.any():
        return []
    h, flat = h[keep], flat[keep]
    worst = {}
    by_body = {}
    for pt in B.robot.points(q):
        by_body.setdefault(pt.body, []).append(pt)
    for body, ends in by_body.items():
        if body == "tip":
            continue
        a, b = ends[0].p, ends[-1].p
        for c in (a, (a + b) * HALF, b):
            near = np.linalg.norm(flat - (c - float(c @ up) * up), axis=1) <= ends[0].radius + foot
            if not near.any():
                continue
            deep = _top(h[near], sigma) - (float(c @ up) - ends[0].radius)
            if deep > tell and deep > worst.get(body, 0.0):
                worst[body] = deep
    return sorted(((body, "look " + look, deep) for body, deep in worst.items()), key=lambda hit: -hit[-1])


def top_middle(ex, p, reach_m, sigma_m):
    """(the middle of what the newest look with depth shows standing highest within reach_m, in plan, of the point p,
    at p's own height; how wide that is; the look's id), or (None, None, None): no look shows anything there, or
    what stands highest is as wide as the reach (a face, not a part). Highest: at or over the half height between
    the point and the top of what stands there, and never less than the depth's own band under that top."""
    got = part_top(ex, p, reach_m, sigma_m)
    return (None, None, None) if got is None else (got["middle"], got["wide_m"], got["look"])


def part_top(ex, p, reach_m, sigma_m):
    """What the newest look with depth shows standing highest about a pointed point p, read as a part seen from
    above: {"middle": its middle in plan at p's own height, "top": that middle at the height of its top, "top_h":
    that height along up, "wide_m", "look", "covers": the point lies under it in plan (within the look's own
    error there: BAND sigma of its depth or one pixel's footprint), "off_m": the middle's distance from p in
    plan}, or None (see top_middle). The middle is found again about itself until it no longer moves by more
    than the look can tell: a pixel anywhere on a part, also on its edge or its side, gives the part's own
    middle, not the middle of the piece of it that lies within reach of the pixel."""
    try:
        from rcpx.r2 import see_cloud
    except ImportError:
        return None
    pts, key, sigma, foot = points(ex)
    if pts is None:
        return None
    up = up_of(ex)
    sigma = max(float(sigma_m or 0.0), sigma)
    tell = max(BAND * sigma, float(foot))
    h = pts @ up
    flat = pts - np.outer(h, up)
    p = np.asarray(p, dtype=float)
    p_h, p_flat = float(p @ up), p - float(p @ up) * up
    at, out = p_flat, None
    for _ in range(int(BAND * BAND)):          # numeric: a fixed point search ends when it no longer moves
        near = np.linalg.norm(flat - at, axis=1) <= reach_m
        if not near.any():
            break
        top = float(see_cloud.top_of(h[near], sigma))
        cut = top - max(BAND * sigma, (top - p_h) * HALF)
        cap = flat[near][h[near] >= cut]
        if not len(cap):
            break
        lo, hi = cap.min(axis=0), cap.max(axis=0)
        wide = float(np.linalg.norm(hi - lo))
        if wide >= reach_m + reach_m - BAND * sigma:
            break                                 # it fills the reach: a face, not a part
        mid = (lo + hi) * HALF
        moved = float(np.linalg.norm(mid - at))
        out = {"middle": mid + p_h * up, "top": mid + top * up, "top_h": top, "wide_m": wide, "look": key,
               "covers": bool(float(np.linalg.norm(cap - p_flat, axis=1).min()) <= tell),
               "off_m": float(np.linalg.norm(mid - p_flat))}
        if moved <= tell:
            break
        at = mid
    return out


def above(ex, top_h):
    """(points N x 3, the look's id, what the look can tell m) of the newest look with depth that stand higher than
    top_h (along up) by more than the look can tell: what the look shows over a part's top. (None, id or None, what
    it can tell) where the look shows none, or there is no look with depth."""
    pts, look, sigma, foot = points(ex)
    tell = max(BAND * sigma, foot)
    if pts is None:
        return None, None, tell
    keep = pts @ up_of(ex) > float(top_h) + tell
    return (pts[keep] if keep.any() else None), look, tell


def face_ahead(ex, p, d, half_m, top_h, got=None):
    """How far (m) along the unit level direction d from the point p the face of the thing a pushed part goes into
    stands, as the newest look shows it: the nearest offset along d past what the look can tell, within half_m of
    the stroke's line across d in plan, at which what the look shows over the part's top (above(), or got: its
    answer) reaches down to that top within BAND times what the look can tell (a face; a protrusion the look
    shows only by its top stands higher and is passed under). The offsets are read in steps of what the look can
    tell. None where the look shows no such face."""
    pts, _, tell = above(ex, top_h) if got is None else got
    if pts is None or not tell > 0:
        return None
    up = up_of(ex)
    d = unit(np.asarray(d, dtype=float) - float(np.asarray(d, dtype=float) @ up) * up)
    rel = pts - np.asarray(p, dtype=float)
    along, across = rel @ d, np.abs(rel @ np.cross(up, d))
    low = pts @ up <= float(top_h) + tell + BAND * tell
    on = (along > tell) & (across <= float(half_m)) & low
    if not on.any():
        return None
    return float(np.floor(float(along[on].min()) / tell) * tell)


def capsules(ex, q):
    """[(body, end a, end b, radius)] of the arm's collision capsules at the joints q (the base left out: it rests
    on its mount)."""
    return [(str(body), np.asarray(a, dtype=float), np.asarray(b, dtype=float), float(r))
            for body, a, b, r in ex.model.collision_capsules(np.asarray(q, dtype=float))[1:]]


def _enter_ball(w, d, r):
    """Per row of w (a point less a ball's centre, N x 3): the least travel s >= 0 of the ball along unit d at which
    the point lies in it (0 where it lies in it already); inf where it never does."""
    b = w @ d
    c = np.einsum("ij,ij->i", w, w) - r * r
    disc = b * b - c
    s = b - np.sqrt(np.maximum(disc, 0.0))
    return np.where(c <= 0, 0.0, np.where((disc >= 0) & (s >= 0), s, np.inf))


def _enter_side(w, d, e, length, r):
    """The same for the round side of a capsule whose axis runs from its end a along unit e for length m (w: the
    points less a): the travel at which the point first lies within r of the axis, where the point's place along
    the axis then lies between the two ends; inf elsewhere (the ends are balls of their own)."""
    w_perp = w - np.outer(w @ e, e)
    d_perp = d - float(d @ e) * e
    A = float(d_perp @ d_perp)
    B = w_perp @ d_perp
    C = np.einsum("ij,ij->i", w_perp, w_perp) - r * r
    if not A > TINY:
        s = np.where(C <= 0, 0.0, np.inf)
    else:
        disc = B * B - A * C
        s = np.where(C <= 0, 0.0, np.where(disc >= 0, (B - np.sqrt(np.maximum(disc, 0.0))) / A, np.inf))
        s = np.where(s >= 0, s, np.inf)
    t = w @ e - np.where(np.isfinite(s), s, 0.0) * float(d @ e)
    return np.where((t >= 0) & (t <= length), s, np.inf)


def first_meet(caps, pts, d, length_m, pad_m):
    """(travel m, body): how far the capsules caps (capsules()) go, moved together along the unit direction d, before
    one comes within its radius plus pad_m of one of the points pts; (length_m, None) where none does within
    length_m. The points are what a look shows: nothing is assumed of what lies behind them."""
    d = unit(np.asarray(d, dtype=float))
    best, who = float(length_m), None
    if pts is None or not len(pts):
        return best, who
    for body, a, b, r in caps:
        R = r + float(pad_m)
        span = b - a
        length = float(np.linalg.norm(span))
        lo, hi = np.minimum(a, b) - R, np.maximum(a, b) + R + np.abs(d) * best
        near = np.all((pts >= lo - np.abs(d) * best) & (pts <= hi), axis=1)
        if not near.any():
            continue
        P = pts[near]
        s = np.minimum(_enter_ball(P - a, d, R), _enter_ball(P - b, d, R))
        if length > TINY:
            s = np.minimum(s, _enter_side(P - a, d, span / length, length, R))
        top = float(s.min())
        if top < best:
            best, who = top, body
    return best, who


def under_text(hit):
    """What a pose that stands under something a look shows says."""
    if not hit:
        return None
    body, look, deep = hit
    return f"{body} would stand {deep * laws_mm():.0f} mm under what {look} shows there"
