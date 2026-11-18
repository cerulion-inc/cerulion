# SPDX-License-Identifier: AGPL-3.0-only
"""Which space is KNOWN free, and where a stated limit holds a leg.

Space is known free by one of three sources and by nothing else; a word of the caller buys nothing:

  1. a look of the scene as it stands shows it free by more than the look's own stated error (rcpx/r2/see_free.py;
     across calls only a look taken after the arm's last motion, the design rulings);
  2. the arm's own bodies or the held thing passed through it IN THIS CALL, with nothing met and no jaw told
     to move since, no look taken since that shows a surface in it, and no part the session set moving able to
     reach it (rcpx/control_move_passed.py; the design rulings: the passage of an earlier call frees nothing, the
     world can change between calls);
  3. the owner's site statement calls it free (a zone with "free": true).

Everywhere else a contact CAN begin. A stated support, a measured thing or a touched point say where a surface
IS; they never say that the way to it is empty.

A LIMIT stated on a motion (max_n, gentle, the site, a rating, people, a give, the limit of a thing held) holds a
free stretch only where a contact can begin, for any moving part of the arm and for the held thing: there the
stretch runs at the speed solved for the limit (rcpx/bounds_limit.py), the arm's links too. Where all that the
arm and the held thing sweep is known free it runs at the body's own speed, and the plan's strike says what a
contact there would make. A motion with an ENDING force (a touch, the approach of a press) runs at the body's
speed only while all that the tool's own bodies and the held thing sweep is known free (stroke()).

Nothing is typed: the swept extent is the arm's own moving points (both ends of every capsule with its radius,
the hull points near a measured thing, the tip, the held thing's points) at both ends of a stretch and between
them, a station every radius; for a stroke the tool's capsules, each grown by its own fit error, the tip with
the tool's half width and the ball around the held thing, a station every half width of the tool.

    known_free(ex, points_m, radii_m)   -> (free: bool per ball, by: source per ball, why: text per ball)
    stroke(ex, B, q, p0, d, reach, held)-> {"free_to_m", "by", "why", "lead_m"}: how far a stroke's way is known free
    zone(B, free_limit)                 -> None (no LIMIT stated) or the record of this plan's stretches
    holds(B, zone, ps, pe, s0, L)       -> True when the limit holds the stretch (a contact can begin there)
    cost(zone, periods, without)        -> the receipt's limit.cost: {"periods", "without_periods", "where", ...}
    outside(zone, s_m)                  -> True when the path coordinate s_m lies on a stretch the limit did not hold
"""

import math

import numpy as np

from rcpx import bounds_limit as limit
from rcpx.bounds_ratios import BAND
from rcpx import control_move_passed as passage

MM = 1000.0
WORDS = {"look": "a look of the scene as it stands shows it free", "passed": "the arm passed through it in this call "
         "and felt nothing since", "site": "the site states it free"}


def _see(ex):
    """The reader of the looks, or None in a build with no seeing package."""
    try:
        from rcpx.r2 import see_free
        return see_free
    except Exception as exc:
        ex._journal("bounds_problem", where="bounds_free: see_free", error=f"{type(exc).__name__}: {exc}")
        return None


def known_free(ex, points_m, radii_m):
    """(free, by, why) per ball (centre m, radius m; the design rulings). by: "site", "passed", "look" or None; why: for a
    ball that is not known free, what each source says of it."""
    pts = np.asarray(points_m, dtype=float).reshape(-1, 3)
    radii = np.broadcast_to(np.asarray(radii_m, dtype=float), (len(pts),)).astype(float)
    free, by, why = np.zeros(len(pts), dtype=bool), [None] * len(pts), [""] * len(pts)
    if not len(pts):
        return free, by, why
    said = passage.stated(ex, pts, radii)
    when = passage.passed(ex, pts, radii)
    met = _measured_in(ex, pts, radii)       # a touched or located point says a surface IS there
    gone = passage.voided(ex, pts, radii)    # a part the session set moving can reach it
    was = ~np.isnan(when) & ~met
    see = _see(ex)
    full = np.zeros(len(pts), dtype=bool)
    idx = np.flatnonzero(was & ~said)
    if see is not None and len(idx):
        try:
            full[idx] = see.shows_in(ex, pts[idx], radii[idx], when[idx])
        except Exception as exc:                 # a look that cannot be read vetoes nothing and shows nothing
            ex._journal("bounds_problem", where="see_free.shows_in", error=f"{type(exc).__name__}: {exc}")
    rest = np.flatnonzero(~said & ~(was & ~full))
    shown, words = np.zeros(len(rest), dtype=bool), ["no look with depth of the scene as it stands is held"] * len(rest)
    if see is not None and len(rest):
        try:
            shown, words = see.shown_free(ex, pts[rest], radii[rest])
        except Exception as exc:
            ex._journal("bounds_problem", where="see_free.shown_free", error=f"{type(exc).__name__}: {exc}")
    looked = dict(zip(rest.tolist(), zip(np.asarray(shown, dtype=bool).tolist(), words)))
    for i in range(len(pts)):
        if said[i]:
            free[i], by[i] = True, "site"
        elif was[i] and not full[i]:
            free[i], by[i] = True, "passed"
        elif looked[i][0]:
            free[i], by[i] = True, "look"
        else:
            why[i] = looked[i][1] + ("; the arm passed there, and a look taken since shows a surface in it" if full[i]
                                     else "; a measured surface (a touched or located point) lies in it" if met[i]
                                     else "; a part this session's own act set moving can reach it" if gone[i]
                                     else "; the arm has not passed there in this call since the scene last changed")
    return free, by, why


def _measured_in(ex, pts, radii):
    """bool per ball: a point of a measured thing of the space book (touched, located, a device's) lies inside the
    ball grown by BAND times that point's own error. The arm's passage frees no ball that holds a surface the
    session measured: a pose the arm stood at while it touched holds that surface (the design rulings, b (2))."""
    out = np.zeros(len(pts), dtype=bool)
    try:
        things = ex.bounds.space._all()
    except Exception:                    # no book: nothing measured
        return out
    for thing, _ in things:
        point = getattr(thing, "point", None)
        if point is None:
            continue
        p = np.asarray(point, dtype=float).reshape(-1)[:3]
        grow = BAND * float(getattr(thing, "sigma_p", None) or 0.0)
        out |= np.linalg.norm(pts - p, axis=1) <= radii + grow
    return out


def _tool_balls(ex, B, q, held):
    """(centres N x 3, radii N) of the balls that cover what a stroke of the tool sweeps first: the tool's own
    bodies (their capsules, each grown by its fit error, a ball every radius along the axis, the jaws where they
    stand), the tip with the tool's own half width, and the ball around the held thing."""
    q = np.asarray(q, dtype=float)
    fit = {}
    try:
        fit = dict(ex.model.e_fit())
    except Exception:
        fit = {}
    pts, radii = [], []
    tool = getattr(ex, "_tool_bodies", ())
    for name, a, b, r in ex.model.collision_capsules(q)[1:]:
        if name not in tool:
            continue
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        rr = float(r) + float(fit.get(name) or 0.0)
        n = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / rr))) if rr > 0.0 else 1
        for i in range(n + 1):
            pts.append(a + (b - a) * (i / n))
            radii.append(rr)
    w = float(B.robot.w_tool())
    if w > 0.0 or not pts:
        pts.append(np.asarray(ex.model.fk_tip(q)[0], dtype=float))
        radii.append(w)
    grip = B.robot.held_points(q, held)
    if grip:
        cloud = np.asarray([v.p for v in grip], dtype=float)
        mid = cloud.mean(axis=0)
        pts.append(mid)
        radii.append(float(np.max(np.linalg.norm(cloud - mid, axis=1))))
    return np.asarray(pts, dtype=float).reshape(-1, 3), np.asarray(radii, dtype=float)


def lead(ex, B, q, d, held=None):
    """How far the point of the tool's own shape (and of the held thing) that leads along d stands ahead of the
    tip (m, never under 0): where that point meets a surface, the tip is still this far from it."""
    q, d = np.asarray(q, dtype=float), np.asarray(d, dtype=float)
    tip = np.asarray(ex.model.fk_tip(q)[0], dtype=float)
    most = 0.0
    for name, a, b, r in ex.model.collision_capsules(q)[1:]:
        if name not in getattr(ex, "_tool_bodies", ()):
            continue
        hull = None
        try:
            hull = B.robot.hull_points(q, name)
        except Exception:
            hull = None
        if hull is not None and len(hull):
            most = max(most, float(np.max((np.asarray(hull, dtype=float) - tip) @ d)))
        else:                                    # no shape of its own: the capsule's far side
            most = max(most, max(float((np.asarray(p, dtype=float) - tip) @ d) for p in (a, b)) + float(r))
    for v in B.robot.held_points(q, held):
        most = max(most, float((np.asarray(v.p, dtype=float) - tip) @ d))
    return most


def stroke(ex, B, q, p0, d, reach, held=None):
    """How far along a stroke from the pose q (tip p0) along d (unit, base), at most reach (m), ALL that the
    tool's own bodies and the held thing sweep is known free (the design rulings, c; C15 (1)). The tool keeps its turn on a
    stroke, so its balls move with the tip. A station every half width of the tool (no thing as wide slips
    between). free_to_m: the travel of the last station known free (0: not even the first); by: the sources
    that carried it; why: what the first station that is not known free lacks (None: the whole reach is free)."""
    p0, d, reach = np.asarray(p0, dtype=float), np.asarray(d, dtype=float), float(reach)
    out = {"free_to_m": 0.0, "by": [], "why": None, "lead_m": lead(ex, B, q, d, held)}
    if not reach > 0.0:
        return out
    centres, radii = _tool_balls(ex, B, q, held)
    centres = centres - np.asarray(ex.model.fk_tip(np.asarray(q, dtype=float))[0], dtype=float) + p0
    step = float(B.robot.w_tool())
    step = step if step > 0.0 else float(np.min(radii[radii > 0.0])) if np.any(radii > 0.0) else reach
    n = max(1, int(math.ceil(reach / step)))
    used = []
    for k in range(1, n + 1):
        t = reach * k / n
        free, by, why = known_free(ex, centres + d * t, radii)
        if not np.all(free):
            out["why"] = why[int(np.argmin(free))]
            break
        out["free_to_m"] = t
        used = list(dict.fromkeys(used + list(by)))
    out["by"] = used
    return out


def zone(B, free_limit):
    """The record of one plan under a limit stated on free space, or None when none is stated. "limit": what
    binds the tool there: the least stated limit (a force_n stated on a move counts, C3), but never the force a
    touch ends at (the design rulings: that record carries "floor"; only the LIMIT stated beside it binds a free stretch)."""
    plain = free_limit.get("limit_n") is not None and "floor" not in free_limit
    lim = free_limit if plain else limit.hold_of(free_limit)
    if lim is None:
        return None
    see, looks = _see(B.ex), False
    try:
        looks = bool(see.looks_now(B.ex)) if see is not None else False
    except Exception:                    # no look that can be read: no look
        looks = False
    return {"looks": looks, "held": [], "free": [], "by": [], "w": float(B.robot.w_tool()), "limit": lim}


def _stations(a0, a1, b0, b1, radius):
    """Points of the surface swept by the segment a0-a1 (a body at the start of a stretch) moving to b0-b1 (at its
    end), one every radius along both."""
    span = max(float(np.linalg.norm(a1 - a0)), float(np.linalg.norm(b1 - b0)))
    travel = max(float(np.linalg.norm(b0 - a0)), float(np.linalg.norm(b1 - a1)))
    n, m = max(1, int(math.ceil(span / radius))), max(1, int(math.ceil(travel / radius)))
    out = []
    for i in range(n + 1):
        p, q = a0 + (a1 - a0) * (i / n), b0 + (b1 - b0) * (i / n)
        out += [p + (q - p) * (j / m) for j in range(m + 1)]
    return out


def holds(B, zone, ps, pe, s0, L):
    """True when the stated limit holds the stretch whose moving points go from ps to pe (path coordinate s0, length
    L): some part of what the arm or the held thing sweeps there is not known free. False: all of it is known
    free (by a look, by the arm's own passage, by the site), and the stretch runs at the body's own speed."""
    if zone is None:
        return False
    bodies = {}
    for a, b in zip(ps, pe):             # the two ends of a capsule share its body: the swept surface between them
        bodies.setdefault(a.body, []).append((np.asarray(a.p, dtype=float), np.asarray(b.p, dtype=float),
                                              float(a.radius) if a.radius > 0.0 else zone["w"]))
    pts, radii, names = [], [], []
    for body, ends in bodies.items():
        for k in range(max(1, len(ends) - 1)):
            a0, b0, r = ends[k]
            a1, b1, _ = ends[min(k + 1, len(ends) - 1)]
            if not r > 0.0:
                r = float(np.linalg.norm(b0 - a0)) or L or 1.0   # a point with no size at all: one station per end
            got = _stations(a0, a1, b0, b1, r)
            pts += got
            radii += [r] * len(got)
            names += [body] * len(got)
    free, by, why = known_free(B.ex, pts, radii)
    if not np.all(free):
        first = int(np.argmin(free))
        zone["held"].append((s0, s0 + L, f"{names[first]}: {why[first]}"))
        return True
    zone["free"].append((s0, s0 + L))
    zone["by"] = list(dict.fromkeys(zone["by"] + list(by)))
    return False


def outside(zone, s_m):
    """True when the path coordinate s_m (m) lies on a stretch the limit did not hold: it was known free."""
    if zone is None or s_m is None:
        return False
    return any(lo <= float(s_m) <= hi for lo, hi in zone["free"]) and not any(
        lo <= float(s_m) <= hi for lo, hi, _ in zone["held"])


def _merge(rows):
    out = []
    for lo, hi, why in rows:
        if out and out[-1]["to_mm"] == round(lo * MM, 1) and out[-1]["why"] == why:
            out[-1]["to_mm"] = round(hi * MM, 1)
        else:
            out.append({"from_mm": round(lo * MM, 1), "to_mm": round(hi * MM, 1), "why": why})
    return out


def cost(zone, periods, without):
    """The receipt's limit.cost (C12): the control periods the limit added, the same motion's count with no
    limit, where the limit held (path coordinate, mm, with what each source says of the stretch) and which
    sources carried the stretches it did not hold. With no look held: what a look that shows the way free would
    have made of it."""
    if zone is None:
        return None
    out = {"periods": max(0, int(periods) - int(without)), "without_periods": int(without),
           "where": _merge(zone["held"]), "free_mm": round(sum(hi - lo for lo, hi in zone["free"]) * MM, 1),
           "free_by": [WORDS[b] for b in zone["by"]]}
    if not out["periods"]:
        return out
    if not zone["looks"]:
        out["said"] = (f"the limit added {out['periods']} control periods: no look with depth of the scene as it "
                       f"stands is held, so the limit holds every leg the arm has not passed since the scene last "
                       f"changed ({out['free_mm']:g} mm ran at the body's own speed): with a look that shows the way "
                       f"free {int(without)} control periods instead of {int(periods)} (the limit then holds only "
                       "where a contact can begin)")
    else:
        out["said"] = (f"the limit added {out['periods']} control periods: it held the legs where a contact can "
                       f"begin ({len(out['where'])} stretch(es)); {out['free_mm']:g} mm known free ran at the body's "
                       "own speed")
    return out
