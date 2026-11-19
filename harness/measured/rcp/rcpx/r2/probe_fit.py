# SPDX-License-Identifier: AGPL-3.0-only
"""Package probe, the fit: a line and a circle through the tip path of a probe, and which one it was.

Pure geometry on points in metres (frame B); no robot, no device and no simulator is named or read here.

A straight path is a part on a slide (prismatic); an arc is a part on a hinge (revolute). A circle always fits a
path at least as well as a line does (a line is a circle of endless radius), so the circle is taken only when the
line is clearly worse AND the path bends by a measurable angle about a centre that is not far away AND the circle
itself fits within the path noise. The line is taken only when it fits within that noise over a path long enough
to have shown a hinge. Anything else is no answer (kind None, with why): a path too short to tell the two apart,
or one that neither fits. A line says how near a hinge it rules out (hinge_ruled_out, m), never that there is none.

All of this reads the TIP's path. It is the part's path only while friction carries the tip with the part; a tip
that slides on the part draws the push's own line, or a bend the turning push direction made (probe_push.follow).
"""

import math
from rcpx.r2 import probe_bounds as derived
MIN_POINTS = 4
SETTLE_FULL = 7
CIRCLE_GAIN = 3.0

import numpy as np

ARC_LINE_RMS = 0.0373     # a line through an arc of length L and radius R misses it by this x L^2 / R (rms; the
                          # sagitta L^2 / 8R times 2 / (3 sqrt 5), for evenly spaced samples)


def unit(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return None if n < 1e-12 else v / n


def path_length(points):
    p = np.asarray(points, dtype=float)
    return 0.0 if len(p) < 2 else float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def fit_line(points):
    """Least squares line through 3D points: {point, dir (unit, along the travel), rms, max} or None."""
    p = np.asarray(points, dtype=float)
    if len(p) < 2:
        return None
    c = p.mean(axis=0)
    _, sv, vt = np.linalg.svd(p - c)
    if sv[0] < 1e-9:
        return None
    u = vt[0]
    if float((p[-1] - p[0]) @ u) < 0:
        u = -u
    off = (p - c) - np.outer((p - c) @ u, u)
    dist = np.linalg.norm(off, axis=1)
    return {"point": c, "dir": u, "rms": float(np.sqrt(np.mean(dist ** 2))), "max": float(dist.max())}


def _circle_2d(xy):
    """Algebraic circle through 2D points, then a few Gauss-Newton steps on the geometric distance."""
    x, y = xy[:, 0], xy[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    sol, *_ = np.linalg.lstsq(A, x * x + y * y, rcond=None)
    cx, cy = float(sol[0]), float(sol[1])
    r2 = float(sol[2]) + cx * cx + cy * cy
    if not math.isfinite(r2) or r2 <= 0:
        return None
    r = math.sqrt(r2)
    for _ in range(12):
        dx, dy = x - cx, y - cy
        rho = np.hypot(dx, dy)
        if np.any(rho < 1e-12):
            break
        res = rho - r
        J = np.column_stack([-dx / rho, -dy / rho, -np.ones(len(x))])
        step, *_ = np.linalg.lstsq(J, -res, rcond=None)
        if not np.all(np.isfinite(step)):
            break
        cx, cy, r = cx + float(step[0]), cy + float(step[1]), r + float(step[2])
        if float(np.linalg.norm(step)) < 1e-10:
            break
    if not (math.isfinite(r) and r > 0):
        return None
    return cx, cy, r


def fit_circle(points):
    """Least squares circle through 3D points: the plane of the path first, then a circle in it.
    {center, axis (unit, right-handed with the travel), radius, rms, arc_rad, plane_rms} or None."""
    p = np.asarray(points, dtype=float)
    if len(p) < MIN_POINTS:
        return None
    c0 = p.mean(axis=0)
    _, sv, vt = np.linalg.svd(p - c0)
    if sv[0] < 1e-9 or sv[1] < 1e-12:
        return None                                   # the points lie on one line: no plane, no circle
    e1, e2, n = vt[0], vt[1], vt[2]
    xy = np.column_stack([(p - c0) @ e1, (p - c0) @ e2])
    out_of_plane = (p - c0) @ n
    got = _circle_2d(xy)
    if got is None:
        return None
    cx, cy, r = got
    center = c0 + cx * e1 + cy * e2
    radial = np.hypot(xy[:, 0] - cx, xy[:, 1] - cy) - r
    rms = float(np.sqrt(np.mean(radial ** 2 + out_of_plane ** 2)))
    a = np.unwrap(np.arctan2(xy[:, 1] - cy, xy[:, 0] - cx))
    arc = float(a[-1] - a[0])
    axis = n if arc >= 0 else -n                      # e1 x e2 = n: a positive angle turns about +n
    if float(np.cross(e1, e2) @ n) < 0:
        axis = -axis
    return {"center": center, "axis": axis, "radius": float(r), "rms": rms, "arc_rad": abs(arc),
            "plane_rms": float(np.sqrt(np.mean(out_of_plane ** 2)))}


def tangent_at(circle, point):
    """The way a point on the fitted hinge moves when the part swings on (unit, frame B)."""
    return unit(np.cross(circle["axis"], np.asarray(point, dtype=float) - circle["center"]))


def settled(points):
    """The path without its first samples when enough remain: over the first one or two steps the contact beds in
    (the tip is not yet carried by the part, the push direction has not yet turned to the part's own), and a kink
    there reads as a bend."""
    p = np.asarray(points, dtype=float)
    drop = 2 if len(p) >= SETTLE_FULL else 0
    return p[drop:], drop


def noise_bound(B, q, u, length=0, jitter_m=0):
    """The path misfit (m, rms) allowed by encoder quanta and measured still-hold jitter."""
    return derived.path_noise_m(B, q, u, jitter_m)


def hinge_ruled_out(B, q, u, length, reach_m, jitter_m=0):
    """The radius within which a hinge would bend this path more than its noise bound:
    a straight path rules out a nearer hinge, and no farther than the model's reach.

    Multiplication and bisection avoid dividing by noise, including exactly zero noise.
    """
    noise = noise_bound(B, q, u, length, jitter_m)
    signal = ARC_LINE_RMS * length**2
    if noise * reach_m <= signal:
        return reach_m
    lo, hi = 0, reach_m
    while True:
        mid = (lo + hi) / 2.0
        if mid == lo or mid == hi:
            return lo
        if noise * mid <= signal:
            lo = mid
        else:
            hi = mid


def classify(points, B, q=None, u=None, reach_m=None, jitter_m=0):
    """The motion a tip path shows: {"kind": prismatic | revolute | None, "why", "line", "circle", "length",
    "used", "dropped", "short", "hinge_ruled_out"}. kind None is no answer: the path is too short to fit, too short
    to tell a slide from a hinge (short True, line set), or neither a line nor an arc fits it (line set); "why"
    says which. hinge_ruled_out (m) comes with prismatic. The fit runs on the settled part of the path (see
    settled); length is the whole path's."""
    reach_m = float(reach_m) if reach_m is not None else float(getattr(B, "reach_m"))
    whole = np.asarray(points, dtype=float).reshape(-1, 3)
    floor = derived.path_noise_m(B, q, u, jitter_m)
    # the shortest path that can tell a slide from any hinge: an arc whose radius is the path's own length misses
    # its chord by ARC_LINE_RMS x length, so under noise / ARC_LINE_RMS of path even that arc hides in the noise
    min_arc = floor / ARC_LINE_RMS
    p, drop = settled(whole)
    length = path_length(whole)
    out = {"kind": None, "line": None, "circle": None, "length": length, "why": "", "used": len(p), "dropped": drop,
           "short": False, "hinge_ruled_out": None}
    length = path_length(p)
    if len(p) < MIN_POINTS or length <= floor:
        out.update(short=True, why=f"the part moved {length * 1000:.1f} mm over {len(p)} samples: too little to "
                   f"fit (at least {p.shape[1] + 1} samples over {floor * 1000:g} mm)")
        return out
    line = fit_line(p)
    if line is None:
        out["why"] = "the path has no direction"
        return out
    out["line"] = line
    if length < min_arc:
        out.update(short=True, why=f"{length * 1000:.2f} mm of path is too little to tell a slide from a hinge "
                   f"(an arc needs {min_arc * 1000:g} mm)")
        return out
    bound = noise_bound(B, q, u, length, jitter_m)
    bound = max(bound, np.finfo(float).eps * max(length, reach_m))
    circle = fit_circle(p)
    out["circle"] = circle
    arc_deg = None if circle is None else math.degrees(circle["arc_rad"])
    bends = (circle is not None and line["rms"] > bound and line["rms"] >= CIRCLE_GAIN * circle["rms"]
             and circle["arc_rad"] * circle["radius"] > floor and circle["radius"] <= reach_m)
    if bends and circle["rms"] <= bound:
        out.update(kind="revolute", why=f"the path bends {arc_deg:.1f} deg about a centre "
                   f"{circle['radius'] * 1000:.0f} mm away; a line misses it by {line['rms'] * 1000:.2f} mm, the "
                   f"circle by {circle['rms'] * 1000:.2f} mm")
    elif line["rms"] <= bound or (circle is not None and circle["rms"] <= bound and circle["radius"] > reach_m):
        near = hinge_ruled_out(B, q, u, length, reach_m, jitter_m)
        out.update(kind="prismatic", hinge_ruled_out=near,
                   why=f"the path is a line at the model reach (line residual {line['rms'] * 1000:.3f} mm over {length * 1000:.1f} mm); that rules out "
                       f"a hinge nearer than about {near * 1000:.0f} mm, not one farther away")
    else:
        out["why"] = (f"a line misses the path by {line['rms'] * 1000:.3f} mm"
                      + ("" if circle is None else f" and an arc by {circle['rms'] * 1000:.3f} mm"
                         + (f" ({arc_deg:.1f} deg seen)" if arc_deg is not None else ""))
                      + f", where a clean path of this length fits within {bound * 1000:.3f} mm: the tip did not "
                        "follow one clean slide or swing")
    return out
