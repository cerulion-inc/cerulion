# SPDX-License-Identifier: AGPL-3.0-only
"""The dry check of a camera path (package sight, I37; SPEC-R4 A3, A4.3): no clearance, speed or pose count is
typed here.

- The path is the straight joint line the runtime's own camera move takes. It is checked at poses no further apart,
  at any capsule end and at the tip, than the spacing the session's bounds give for the smallest clearance in force
  (half of it): a long path is SPLIT into more poses, never thinned (samples).
- Every body of the arm keeps the bounds' clearance for a path from every stored point, every survey box and (the
  tool) the line of sight being measured. That clearance grows with the body's speed, linearly, so the speed is
  SOLVED, not assumed: path_check finds the largest share of full joint speed at which every body keeps its
  clearance, and the move then runs at that share. A path is refused only when a body would come nearer a thing
  than the clearance it needs standing still, or when it would have to run under REL x REL of full speed.
- A body that starts nearer one thing than its standstill clearance may not come nearer to THAT thing than it
  started (it may leave), and that gives it no leave to come near any other thing.
- A thing's own position error counts: known_refs hands the bounds each stored point with the error its entry
  carries and each survey box with its two-view error.
Nothing here moves the arm, names a robot or a device, or reads ground truth.
"""

import math

import numpy as np

from rcpx import bounds_ratios as R
from rcpx.r2 import sight_bounds as SB

MM = 1000.0  # metres to millimetres
SAMPLES_GUARD = 131072  # solver numeric: refuse a spacing too fine to represent and check
FLOAT_EPS = 1e-12  # arithmetic roundoff in segment and endpoint computations


def _parts(ex, q):
    """The moving robot at q: [(name, p0, p1, r, is_tool)], the tip last (a tool point of radius 0)."""
    out = []
    for name, p0, p1, r in ex.model.collision_capsules(q)[1:]:
        out.append((name, np.asarray(p0, dtype=float), np.asarray(p1, dtype=float), float(r),
                    name in ex._tool_bodies))
    tip = np.asarray(ex.model.fk_tip(q)[0], dtype=float)
    out.append(("tip", tip, tip, 0.0, True))
    return out


def _seg_point(p0, p1, x):
    ab = p1 - p0
    L2 = float(ab @ ab)
    t = 0.0 if L2 < FLOAT_EPS else min(1.0, max(0.0, float((x - p0) @ ab) / L2))
    return float(np.linalg.norm(p0 + t * ab - x))


def _seg_box(p0, p1, lo, hi, n=7):
    best = float("inf")
    for t in np.linspace(0.0, 1.0, n):
        p = p0 + t * (p1 - p0)
        d = np.maximum(np.maximum(lo - p, 0.0), p - hi)
        best = min(best, float(np.linalg.norm(d)))
    return best


def _seg_seg(a0, a1, b0, b1, n=9):
    return min(_seg_point(b0, b1, a0 + t * (a1 - a0)) for t in np.linspace(0.0, 1.0, n))


def clearances(ex, q, pts, boxes, line=None):
    """{link, tool, line}: (clearance m, what) of the non-tool links and of the tool from the points and boxes, and
    of the tool from the segment `line` ((a, b) m) when given."""
    out = {"link": (float("inf"), ""), "tool": (float("inf"), ""), "line": (float("inf"), "")}
    for name, p0, p1, r, tool in _parts(ex, q):
        key = "tool" if tool else "link"
        for i, x in enumerate(pts):
            d = _seg_point(p0, p1, x) - r
            if d < out[key][0]:
                out[key] = (d, f"the {name} and the stored point at [{', '.join(f'{v * MM:.0f}' for v in x)}] mm")
        for lo, hi, bid in boxes:
            d = _seg_box(p0, p1, lo, hi) - r
            if d < out[key][0]:
                out[key] = (d, f"the {name} and survey box {bid}")
        if tool and line is not None:
            d = _seg_seg(p0, p1, line[0], line[1]) - r
            if d < out["line"][0]:
                out["line"] = (d, f"the {name} and the line of sight being measured")
    return out


def each_clearance(ex, q, pts, boxes, line=None):
    """{(kind, body name, thing): (clearance m, what)} for every body of the arm against every stored point, every
    survey box and (the tool only) the line of sight: kind is link, tool or line."""
    out = {}
    for name, p0, p1, r, tool in _parts(ex, q):
        key = "tool" if tool else "link"
        for i, x in enumerate(pts):
            out[(key, name, ("pt", i))] = (_seg_point(p0, p1, x) - r, f"the {name} and the stored point at "
                                           f"[{', '.join(f'{v * MM:.0f}' for v in x)}] mm")
        for j, (lo, hi, bid) in enumerate(boxes):
            out[(key, name, ("box", j))] = (_seg_box(p0, p1, lo, hi) - r, f"the {name} and survey box {bid}")
        if tool and line is not None:
            out[("line", name, ("line", 0))] = (_seg_seg(p0, p1, line[0], line[1]) - r,
                                                f"the {name} and the line of sight being measured")
    return out


def _points(ex, q):
    return np.asarray([p for _, a, b, _, _ in _parts(ex, q) for p in (a, b)])


def _radius(ex, q):
    return min((r for _, _, _, r, _ in _parts(ex, q) if r > 0), default=0.0)


def _length(ex, q0, q1):
    return float(np.max(np.linalg.norm(_points(ex, q1) - _points(ex, q0), axis=1)))


def samples(ex, q_from, q_to, spacing_m):
    """Straight joint-line poses, excluding the start and including the goal. Start with ceil(endpoint travel /
    spacing), then double until every neighbouring endpoint travel fits. There is no thinning or count cap on a
    valid path; the solver guard refuses an unrepresentably fine spacing.
    """
    q0, q1 = np.asarray(q_from, dtype=float), np.asarray(q_to, dtype=float)
    if not math.isfinite(spacing_m) or spacing_m <= 0:
        return None
    ratio = _length(ex, q0, q1) / spacing_m
    if not math.isfinite(ratio) or ratio > SAMPLES_GUARD:
        return None
    n = max(1, math.ceil(ratio))
    while n <= SAMPLES_GUARD:
        path = [q0 + (q1 - q0) * (k / n) for k in range(1, n + 1)]
        previous = _points(ex, q0)
        for q in path:
            present = _points(ex, q)
            if np.max(np.linalg.norm(present - previous, axis=1)) > spacing_m:
                break
            previous = present
        else:
            return path
        n *= R.DOUBLE
    return None


def body_speeds(ex, q_from, q_to, path):
    """{body name: m/s}: the largest speed each body's end points reach when the move runs at full speed (the
    core's line rate, sight_bounds.line_rate: every joint at most at its own speed bound). The plan may run slower.
    End point travel between neighbouring poses x the count of poses x the share of the line per second."""
    rate = SB.line_rate(ex, q_from, q_to)
    speeds = {name: 0.0 for name, *_ in _parts(ex, q_from)}
    previous = _parts(ex, q_from)
    for q in path:
        present = _parts(ex, q)
        for (name, a, b, _, _), (_, pa, pb, _, _) in zip(present, previous):
            step = float(max(np.linalg.norm(a - pa), np.linalg.norm(b - pb)))
            speeds[name] = max(speeds[name], step * len(path) * rate)
        previous = present
    return speeds


def known_refs(ex):
    """References in known()'s order, carrying the position error of each stored point or surveyed box."""
    from rcpx.r2.sight_plan import box_of
    refs, i, j = {}, 0, 0
    for kind in ("contact", "located", "device"):
        for entry in ex.world.query(kind=kind, include_stale=True):
            if entry.get("point_mm") is None:
                continue
            data = entry.get("data") or {}
            error = next((data[k] for k in ("sigma_p_mm", "uncertain_mm", "error_mm")
                          if data.get(k) is not None), None)
            refs[("pt", i)] = {"id": entry.get("id"), "point": np.asarray(entry["point_mm"]) / MM,
                                 "sigma_p": None if error is None else float(error) / MM}
            i += 1
    for candidate in (ex.state.get("sight") or {}).get("candidates") or []:
        box = box_of(candidate)
        if box is None:
            continue
        lo, hi, bid = box
        error = candidate.get("err_mm")      # the survey's own two-view error of this box [xy, z]
        refs[("box", j)] = {"id": bid, "extent": {"box": [lo, hi]},
                            "sigma_p": max(float(e) for e in error) / MM if error else None}
        j += 1
    return refs


def _ref(item, pts, boxes, refs):
    thing = item[2]
    if refs is not None and thing in refs:
        return refs[thing]
    return pts[thing[1]] if thing[0] == "pt" else (boxes[thing[1]][2] if thing[0] == "box" else None)


def margins(ex, q_from, q_to, pts, boxes, line=None, refs=None):
    """({item: clearance at full speed}, spacing m, {path, still: {item: clearance at standstill}, length_mm}). The
    path is sampled at the bounds' spacing for the smallest standstill clearance; where that is not positive (every
    error zero on a noise-free robot) at REL x the smallest moving capsule radius (the relative form, SPEC-R4
    A1.1); with nothing to keep clear of, at that radius itself."""
    items = each_clearance(ex, q_from, pts, boxes, line)
    still = {item: SB.clearance_m(ex, 0.0, _ref(item, pts, boxes, refs), item[1]) for item in items}
    minimum = min(still.values(), default=0.0)
    spacing = SB.spacing_m(ex, minimum) if items else _radius(ex, q_from)
    if spacing <= 0:
        spacing = R.REL * _radius(ex, q_from)
    path = samples(ex, q_from, q_to, spacing)
    info = {"path": path, "still": still, "length_mm": _length(ex, q_from, q_to) * MM}
    if path is None:
        return {}, spacing, info
    speeds = body_speeds(ex, q_from, q_to, path)
    full = {item: SB.clearance_m(ex, speeds[item[1]], _ref(item, pts, boxes, refs), item[1]) for item in items}
    return full, spacing, info


def path_check(ex, q_from, q_to, pts, boxes, line=None, info=None, refs=None):
    """None or a refusal: joint limits, the executor's dry check, and every body's derived clearance.
    The law is linear in speed, so solve a share from standstill and full-speed clearance. Refuse only when no
    speed clears, or the solved share is under REL squared. A body starting inside its standstill clearance is
    held to its own start distance and imposes no speed bound.
    """
    q0, q1 = np.asarray(q_from, dtype=float), np.asarray(q_to, dtype=float)
    if not ex.model.within_limits(q1):
        return {"check": "joint_limit", "why": "the pose is outside the joint limits"}
    full, spacing, detail = margins(ex, q0, q1, pts, boxes, line, refs)
    path, still = detail["path"], detail["still"]
    if isinstance(info, dict):
        info.update(poses=0 if path is None else len(path), spacing_mm=spacing * MM,
                    margin_mm=min(still.values(), default=0.0) * MM, length_mm=detail["length_mm"], share=1.0)
    if path is None:
        return {"check": "clearance", "why": "the path cannot be checked finely enough for its margins"}
    bad = ex._path_check([q0] + path, True)
    if bad is not None:
        r = bad.get("refused", bad)
        return {"check": r.get("check", "path"), "why": r.get("why", "")}
    start = each_clearance(ex, q0, pts, boxes, line)
    smallest = dict(start)
    for q in path:
        for item, value in each_clearance(ex, q, pts, boxes, line).items():
            if value[0] < smallest[item][0]:
                smallest[item] = value
    share = 1.0
    for item, (distance, what) in smallest.items():
        c0, c1 = still[item], full[item]
        if start[item][0] < c0:
            if distance < start[item][0] - FLOAT_EPS:
                return {"check": "clearance", "why": f"{what} would come within {max(distance, 0) * MM:.0f} mm; "
                        f"it started {max(start[item][0], 0) * MM:.0f} mm away and may not come nearer "
                        f"(the bounds' standstill clearance is {c0 * MM:.0f} mm)"}
            continue
        if distance < c0 - FLOAT_EPS:
            return {"check": "clearance", "why": f"{what} would come within {max(distance, 0) * MM:.0f} mm; "
                    f"the bounds' standstill clearance is {c0 * MM:.0f} mm and no speed clears it"}
        if distance < c1 and c1 > c0:
            share = min(share, max(0.0, (distance - c0) / (c1 - c0)))
    if isinstance(info, dict):
        info["share"] = share
    if share < R.REL * R.REL:
        return {"check": "clearance", "why": "the path would have to run under a twenty-fifth of full speed to keep "
                "its clearances"}
    return None


def floor_clear_m(ex, q=None):
    """How far over the support a camera position must stand (m): the camera body's radius + BAND x the support's
    height error. The camera rides on the hand and the model holds no housing of its own, so the radius is the
    largest capsule radius of the tool's bodies (at q, default the present pose)."""
    q = ex._sense().q if q is None else q
    radius = max((r for _, _, _, r, tool in _parts(ex, q) if tool), default=0.0)
    return radius + R.BAND * SB.support_sigma_m(ex)


def tool_span_m(ex, q=None):
    """How far the tool reaches from the camera (m): the largest distance from the lens to any point of the tool's
    capsules (their end points and radius) and to the tip, at q (default the present pose). The camera rides on the
    hand: from nearer than this the tool could reach the thing it looks at whichever way the hand is turned."""
    q = ex._sense().q if q is None else q
    cam = np.asarray(ex.model.fk(np.asarray(q, dtype=float))["camera"][0], dtype=float)
    return max((float(np.linalg.norm(p - cam)) + r for _, a, b, r, tool in _parts(ex, q) if tool for p in (a, b)),
               default=0.0)


__all__ = ["body_speeds", "clearances", "each_clearance", "floor_clear_m", "known_refs", "margins", "path_check",
           "samples", "tool_span_m"]
