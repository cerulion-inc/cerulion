# SPDX-License-Identifier: AGPL-3.0-only
"""The band where a contact can begin on a stroke.

A motion with an ending force (a touch, the approach of a press) runs at the body's own speed only through space
that is KNOWN free, and from the first place where a contact can begin at the speed solved for the ending force
(what lies past the first surface is hidden). Two things place that edge, and the nearer one decides:

  the WAY: how far ALL that the tool's own bodies (the hand, each jaw where it stands) and the held thing sweep
  is known free (rcpx/bounds_free.py stroke): by a look of the scene as it stands, by the arm's own
  passage this session with nothing met since, by the site's word. A word of the caller buys nothing.

  the END: the first surface the session knows of along the stroke, each less BAND times its own stated or
  measured error, less the tool's own half width, less how far the tool's (or the held thing's) leading point
  stands ahead of the tip: the space book's measured thing ahead (touched, located, the extent of a spot) and
  the stated support. A stated support says where a surface IS; it never says the way to it is empty.

With no way known free the whole travel is the arrival stretch and the receipt prices it. Nothing here is typed
and nothing reads the scene's truth.

The bet is priced, not hidden: before the motion the receipt carries fast_n, the strike a contact
outside the band would make at the stroke's fastest sample (the plan's own figure), and arrive_n, the band's
figure at the arrival speed; after a stop the one figure for the speed the contact was met at (strike.ran).

    of_stroke(ex, B, p0, d, reach, thing, q0, held)  -> the band {"from_m", "at_m", "sigma_m", "half_m", "source", "by"}
    split(ex, B, q0, line, p0, d, from_m, intent, kind, thing) -> (line with a waypoint at the band's edge, expect list)
    keys(band, sched, dt, v_arr, ends_n, ends_source, plan, arrive_n) -> the receipt's "band" block
"""

import math

import numpy as np

from rcpx import bounds_laws as laws
from rcpx import bounds_limit as limit
from rcpx.control_base import MM, _val


def _travel_to(thing, p0, d):
    """How far along d from p0 the surface of a measured thing lies (m): the crossing of its plane, else the foot
    of its point on the line. None for a thing with neither, or a plane the line does not run into."""
    extent = thing.extent if isinstance(getattr(thing, "extent", None), dict) else {}
    plane = extent.get("plane")
    if plane is not None:
        n = np.asarray(plane[0], dtype=float)
        size = float(np.linalg.norm(n))
        closing = -float(n @ d) / size
        if not closing > 0.0:
            return None
        return (float(n @ p0) - float(plane[1])) / size / closing
    point = getattr(thing, "point", None)
    return None if point is None else float((np.asarray(point, dtype=float) - p0) @ d)


def _end(ex, B, p0, d, reach, thing, lead):
    """The first surface the session knows of along the stroke, or None: {"from_m": the tip travel from which
    the tool can meet it, "at_m", "sigma_m", "half_m", "what"}. Only one the stroke reaches."""
    w = float(_val(B.standoff(0.0)))
    rows = []
    if thing is not None:
        t = _travel_to(thing, p0, d)
        if t is not None:
            sigma = getattr(thing, "sigma_p", None)
            name = getattr(thing, "id", None) or "a measured thing"
            what = "the stated support" if name == "support" else f"the measured thing ahead ({name})"
            rows.append((t, sigma, what))
    n, offset, sigma = ex._support()
    closing = -float(np.asarray(n, dtype=float) @ d)
    if closing > 0.0:
        rows.append(((float(np.asarray(n, dtype=float) @ p0) - float(offset)) / closing, sigma, "the stated support"))
    best = None
    for t, sigma, what in rows:
        half = laws.band(float(sigma or 0.0)) + w
        edge = max(0.0, t - lead - half)
        if edge <= reach and (best is None or edge < best["from_m"]):
            best = {"from_m": edge, "at_m": t, "sigma_m": sigma, "half_m": half,
                    "what": f"{what}, {t * MM:.0f} mm along the way, "
                            + ("stated with no error" if sigma is None else f"known to {float(sigma) * MM:.1f} mm")}
    return best


def of_stroke(ex, B, p0, d, reach, thing=None, q0=None, held=None):
    """C15 for a stroke from p0 along d (unit, base) of at most reach (m), the arm at q0 (None: where it was
    last read), holding `held`. from_m: the tip travel from which a contact can begin (0: the whole stroke);
    at_m: where the surface that ends the fast part is expected (None: the edge of what is known free);
    sigma_m: its stated or measured error; half_m: the band's half depth; source: the words, naming what carried
    the fast part and what ends it; by: "look", "passed", "site" or None when no way is known free."""
    from rcpx import bounds_free
    p0, d, reach = np.asarray(p0, dtype=float), np.asarray(d, dtype=float), float(reach)
    w = float(_val(B.standoff(0.0)))                      # the tool's own half width
    none = {"from_m": 0.0, "at_m": None, "sigma_m": None, "half_m": w, "by": None, "source": "none"}
    if not reach > 0.0:
        return none
    q0 = np.asarray(ex._last.q if q0 is None else q0, dtype=float)
    ahead = bounds_free.lead(ex, B, q0, d, held)
    end = _end(ex, B, p0, d, reach, thing, ahead)
    far = reach if end is None else min(reach, end["from_m"])
    way = bounds_free.stroke(ex, B, q0, p0, d, far, held) if far > 0.0 else {"free_to_m": 0.0, "by": [], "why": None}
    free_to = float(way["free_to_m"])
    ends = "" if end is None else f"; the stroke's band is placed by {end['what']}"
    if not free_to > 0.0:
        lacks = way["why"] or "the band begins where the stroke starts"
        return {**none, **({} if end is None else {k: end[k] for k in ("at_m", "sigma_m", "half_m")}),
                "source": f"none: the way is not known free ({lacks}){ends}"}
    carried = " and ".join(bounds_free.WORDS[b] for b in way["by"])
    out = {"from_m": free_to, "by": way["by"][0], "sources": list(way["by"])}
    if way["why"] is None and end is not None:            # the way is known free up to the band of a known surface
        return {**out, **{k: end[k] for k in ("at_m", "sigma_m", "half_m")},
                "source": f"{carried} for {free_to * MM:.0f} mm, up to the band of {end['what']}"}
    if way["why"] is None:                                # known free to the stroke's own end: no surface is known
        return {**out, "at_m": None, "sigma_m": None, "half_m": w,
                "source": f"{carried} for the whole {free_to * MM:.0f} mm of the stroke; no surface is known ahead"}
    return {**out, "at_m": None, "sigma_m": None, "half_m": w,
            "source": f"{carried} for {free_to * MM:.0f} mm; from there the way is not known free ({way['why']}){ends}"}


def split(ex, B, q0, line, p0, d, from_m, intent, kind, thing):
    """(line, expect): the stroke's joint waypoints with one more where the tip reaches the band's edge, and the
    expected contact of each stretch for B.plan: None before the band (a free line at the body's own speed),
    the stroke's own expected contact with whatever lies ahead from the edge on. expect None: no band was given
    (from_m 0): the plan marks the whole stroke itself, as before."""
    if not from_m > 0.0 or not len(line):
        return list(line), None
    one = limit.expects(B, None, {**(intent or {}), "expect": "contact"}, [kind], [thing])[0]
    one = None if one is None else {**one, "with": None}   # whatever lies ahead from the band's edge on
    fk = ex.model.fk_tip
    tell = float(_val(B.rigid(q0, d, 0.0)))               # the least tip travel the readings can tell from none
    out, edge = [], None
    was_q, was_x = np.asarray(q0, dtype=float), 0.0
    for q in line:
        q = np.asarray(q, dtype=float)
        x = float((fk(q)[0] - p0) @ d)
        if edge is None and x > from_m + tell:
            if from_m - was_x > tell:
                out.append(was_q + (from_m - was_x) / (x - was_x) * (q - was_q))
            edge = len(out)
        out.append(q)
        was_q, was_x = q, x
    edge = len(out) if edge is None else edge
    return out, [None] * edge + [one] * (len(out) - edge)


def keys(band, sched, dt, v_arr, ends_n, ends_source, plan, arrive_n, n_plan=None, v_free=None):
    """The receipt's "band" block: where the arrival stretch began and by what source, the speed of the
    fast part, the ONE speed the arrival ran at (arrive_mm_s) beside the speed solved for it (solved_mm_s), the
    ending force and the two figures stated before the motion. With no source: the price of that, the
    control periods the stroke plans beside what a look or a stated distance would make of it."""
    from_m = float(band.get("from_m") or 0.0)
    steps = np.diff(np.asarray(sched, dtype=float)) / float(dt) if len(sched) > 1 else np.zeros(0)
    fast = [float(v) for v, at in zip(steps, sched[1:]) if at <= from_m] if from_m > 0.0 else []
    slow = sorted(float(v) for v, was in zip(steps, sched[:-1]) if was >= from_m and v > 0.0)
    ran = slow[len(slow) // 2] if slow else None          # the speed the arrival stretch RAN at (its median step)
    strike = getattr(plan, "strike", None) or {}
    out = {"from_mm": round(from_m * MM, 1), "source": band.get("source"),
           "sigma_mm": None if band.get("sigma_m") is None else round(float(band["sigma_m"]) * MM, 2),
           "half_mm": round(float(band.get("half_m") or 0.0) * MM, 2),
           "fast_mm_s": round(max(fast) * MM, 1) if fast else None,
           "arrive_mm_s": None if (ran if ran is not None else v_arr) is None else
           round(float(ran if ran is not None else v_arr) * MM, 2),
           "solved_mm_s": None if v_arr is None else round(float(v_arr) * MM, 2),
           "ends_n": None if ends_n is None else round(float(ends_n), 2), "ends_source": ends_source,
           "fast_n": (None if strike.get("peak_n") is None else round(float(strike["peak_n"]), 1)) if fast else None,
           "arrive_n": None if arrive_n is None else round(float(arrive_n), 2)}
    if fast and strike.get("peak_n") is None:
        out["fast_note"] = strike.get("z_source") or "the strike of a contact before the band is not measured at that speed"
    if band.get("by") is None and v_arr and v_free and n_plan and len(sched) > 1:
        # the price of no way known free: the same stroke with its band at the tool's own width from its end
        half = float(band.get("half_m") or 0.0)
        depth = min(float(sched[-1]), half + half)
        about = int(math.ceil((float(sched[-1]) - depth) / (float(v_free) * dt))) + int(math.ceil(depth / (float(v_arr) * dt)))
        slow = int(len(sched) - 1)
        if about < slow:
            out["price"] = {"periods": int(n_plan), "with_source_periods": int(n_plan) - slow + about,
                            "said": f"{int(n_plan)} control periods: the way is not known free, so the whole travel "
                                    f"is the arrival stretch; with the way known free (a look with depth that shows "
                                    f"it, or the arm having passed it with nothing felt since) about "
                                    f"{int(n_plan) - slow + about}"}
    return out
