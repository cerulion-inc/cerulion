# SPDX-License-Identifier: AGPL-3.0-only
"""Measured-path math in SI. Thresholds and windows are supplied by Bounds."""

import numpy as np

from rcpx.r2.articulate_bounds import ratios


def unit(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else np.zeros_like(v)


def rot(axis, angle):
    a = unit(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def angle_between(a, b):
    return float(np.arccos(np.clip(unit(a) @ unit(b), -1, 1)))


def between(a, b):
    a, b = unit(a), unit(b)
    axis = np.cross(a, b)
    if np.linalg.norm(axis) < 1e-10:
        if a @ b >= 0:
            return np.eye(3)
        basis = np.eye(3)[int(np.argmin(np.abs(a)))]
        axis = np.cross(a, basis)
    return rot(axis, angle_between(a, b))


def level_across(t, angle_deg):
    """The jaws' starting closing bearing for a twist of angle_deg about u = -t: the level bearing cross(up, t)
    turned by -angle/2 about u, so the turn (+angle about u) carries it through level at its middle and the hand's
    long side sweeps least below the part."""
    t = unit(t)
    level = unit(np.cross([0, 0, 1], t))
    return rot(-t, np.radians(-angle_deg / 2)) @ level


def trim_path(points_mm, skip_mm):
    """The points from the first one at least skip_mm of path from the start (the part seats in the jaws over
    the first mm of a pull; its own path is read from there). An empty list until the path is that long."""
    p = np.asarray(points_mm, dtype=float).reshape(-1, 3)
    walked = 0.0
    for i in range(1, len(p)):
        walked += float(np.linalg.norm(p[i] - p[i - 1]))
        if walked >= skip_mm:
            return p[i:].tolist()
    return []


def follow_step(d, motion, side_force, lead_m, k_side, dead_n, min_motion_m, keep_off=None):
    """The motion term discounts unresolved travel. The force term relieves sideways servo give."""
    d = unit(d)
    side = np.asarray(side_force, dtype=float)
    f = float(np.linalg.norm(side))
    soft = side * max(0, 1 - dead_n / f) if f > 0 else side
    correction = np.zeros(3)
    size = float(np.linalg.norm(motion)) if motion is not None else 0
    if size > min_motion_m:
        correction = max(0, 1 - min_motion_m / size) * (unit(motion) - d)
        if keep_off is not None:
            k = unit(np.asarray(keep_off) - (np.asarray(keep_off) @ d) * d)
            correction -= (correction @ k) * k
    side_correction = soft / (k_side * lead_m) if lead_m > 0 and k_side > 0 else np.zeros_like(soft)
    new = unit(d + correction - side_correction)
    if not np.any(new):
        return d
    turn = angle_between(d, new)
    # With exact readings there is no angular cap based on their resolution.
    cap = float(np.arctan2(min_motion_m, lead_m)) if min_motion_m > 0 else None
    if cap is not None and turn > cap:
        return rot(np.cross(d, new), cap) @ d
    return new


def window_motion(points_mm, distance_mm, first=False):
    p = np.asarray(points_mm, dtype=float)
    if len(p) < 2:
        return None
    if first:
        walked = 0.0
        for i in range(1, len(p)):
            step = float(np.linalg.norm(p[i] - p[i - 1]))
            if step and walked + step >= distance_mm:
                end = p[i - 1] + (p[i] - p[i - 1]) * ((distance_mm - walked) / step)
                return end - p[0]
            walked += step
    else:
        walked = 0.0
        for i in range(len(p) - 1, 0, -1):
            step = float(np.linalg.norm(p[i] - p[i - 1]))
            if step and walked + step >= distance_mm:
                start = p[i] + (p[i - 1] - p[i]) * ((distance_mm - walked) / step)
                return p[-1] - start
            walked += step
    return None


def fit_path(points, sigma_m, window_m):
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    out = {"shape": "straight", "radius_mm": None, "axis_b": None,
           "turned_deg": 0.0, "heading_b": [0.0, 0.0, 0.0], "rms_mm": 0.0}
    if len(p) < 2:
        return out
    chord = p[-1] - p[0]
    newest = window_motion(p, window_m) if len(p) > 1 else None
    oldest = window_motion(p, window_m, first=True)
    out["heading_b"] = unit(chord if newest is None else newest).tolist()
    if newest is not None and oldest is not None:
        out["turned_deg"] = float(np.degrees(angle_between(oldest, newest)))
    centre = p.mean(axis=0)
    z = p - centre
    _, _, V = np.linalg.svd(z, full_matrices=True)
    line = z - np.outer(z @ V[0], V[0])
    line_rms = float(np.sqrt(np.mean(np.sum(line * line, axis=1))))
    out["rms_mm"] = line_rms * 1000
    if len(p) <= 3 + ratios().QUANT:
        return out
    off = p - p[0]
    sag = np.linalg.norm(off - np.outer(off @ unit(chord), unit(chord)), axis=1).max()
    xy = z @ V[:2].T
    A = np.column_stack([2 * xy, np.ones(len(xy))])
    sol = np.linalg.lstsq(A, np.sum(xy * xy, axis=1), rcond=None)[0]
    c = sol[:2]
    radius_sq = sol[2] + c @ c
    if radius_sq <= 0:
        return out
    radius = float(np.sqrt(radius_sq))
    radial = xy - c
    angles = np.unwrap(np.arctan2(radial[:, 1], radial[:, 0]))
    sweep = float(angles[-1] - angles[0])
    rms = float(np.sqrt(np.mean((np.linalg.norm(radial, axis=1) - radius) ** 2 + (z @ V[2]) ** 2)))
    if sag >= ratios().BAND * sigma_m and rms <= ratios().GIVE * line_rms + sigma_m:
        axis = unit(np.cross(V[0], V[1])) * (1 if sweep >= 0 else -1)
        tangent = unit(np.cross(axis, V[:2].T @ radial[-1]))
        out.update(shape="arc", radius_mm=radius * 1000, axis_b=axis.tolist(),
                   turned_deg=float(abs(np.degrees(sweep))), heading_b=tangent.tolist(), rms_mm=rms * 1000)
    return out


def end_index(forces, motions, reached_n, periods, still_m):
    count = 0
    for i, (f, m) in enumerate(zip(forces, motions)):
        count = count + 1 if f >= reached_n and abs(m) <= still_m else 0
        if count >= periods:
            return i
    return None


def slip_index(gaps, min_gap_m, pressing, lost_periods):
    lost = 0
    for i, gap in enumerate(gaps):
        if gap < min_gap_m:
            return i
        lost = lost + 1 if pressing is not None and not pressing[i] else 0
        if lost >= lost_periods:
            return i
    return None


def joint_turn(q_now, angle_rad, axis_dot, lo, hi):
    step = float(angle_rad) * float(np.sign(axis_dot))
    goal = float(q_now) + step
    if lo <= goal <= hi:
        return goal, None
    return None, max(0.0, float(hi - q_now if step > 0 else q_now - lo))


def torque_about(J_rot, u, ext):
    w = np.asarray(J_rot, dtype=float).T @ unit(u)
    den = float(w @ w)
    return float(w @ np.asarray(ext, dtype=float) / den) if den > 1e-12 else 0.0


def guard_step(contact_count, cap_count, magnitude, residual, external, room,
               force_n, one_reading_n, tau, cap_n, readings, total_force=None):
    contact = magnitude > force_n or bool(np.any(np.abs(residual) > tau))
    contact_count = contact_count + 1 if contact else 0
    cap = (magnitude if total_force is None else total_force) > cap_n or bool(np.any(np.abs(external) > room))
    cap_count = cap_count + 1 if cap else 0
    stop = "cap" if cap_count >= readings else ("contact" if contact_count >= readings or magnitude > one_reading_n else None)
    return contact_count, cap_count, stop


def window_stalled(motions, periods, still_m, at_cap):
    if len(motions) < periods or not all(at_cap[-periods:]):
        return False
    return sum(max(0, step) for step in motions[-periods:]) <= still_m
