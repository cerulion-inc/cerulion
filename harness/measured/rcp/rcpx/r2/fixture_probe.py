# SPDX-License-Identifier: AGPL-3.0-only
"""The probe of package fixture (SPEC-R4 B5 phase 3) and the jaws' own readings: fits, ramp laws, the jaw strokes.

A part that is held or pushed shows its own path from the first travel the robot can tell from none (rigid). The
points past that travel fit a LINE (one direction) or an ARC (a centre, a radius and the normal of its plane); the
part's reflected mass is read from the ramp's own force against the tip's acceleration. A part that turns about the
tool axis is probed by turning the hand under a torque ramp (laws in the middle of this file).

There is no force threshold: a part has moved when its tip path passed rigid under load, whatever force that took
(parts move at a tenth of a newton, and a simulator's estimate is noise-free: nothing here divides by a noise or
waits for a reading to cross one).

The jaws (A4.7, at the end): their gap, pressing and squeeze from place_hand.Jaws; a stroke at their own speed until
they stand still; a close and an opening at the first-touch speed beside a thing not yet touched; the least squeeze
that presses; the hold judged by place_hold.judge where the free jaws' numbers are known, else by the jaws' own
geometry. SI inside. The numbers named here are fit numerics, not limits.
"""

import math

import numpy as np

from rcpx import bounds_laws as laws
from rcpx.bounds_ratios import BAND, GIVE, QUANT, REL

TINY = np.finfo(float).eps        # numeric: the smallest length or angle told from zero
FIT_MIN = 3                       # numeric: points a circle fit needs (three points fix a circle)
HALF = 1 / laws.TWO               # numeric: geometry, the middle of two values and half a right angle


def bounds_of(ex):
    from rcpx import bounds
    return bounds.of(ex)


def laws_mm():
    from rcpx.bounds_report import MM
    return MM


def unit(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > TINY else np.zeros_like(v)


def rot(axis, angle):
    """The rotation by angle (rad) about axis (Rodrigues)."""
    a = unit(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(len(a)) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def between(a, b):
    """The least rotation that takes direction a onto direction b."""
    a, b = unit(a), unit(b)
    axis = np.cross(a, b)
    s, c = float(np.linalg.norm(axis)), float(a @ b)
    if s <= TINY:
        return np.eye(len(a)) if c >= 0 else rot(np.cross(a, np.eye(len(a))[int(np.argmin(np.abs(a)))]), math.pi)
    return rot(axis, math.atan2(s, c))


def angle_between(a, b):
    return float(math.acos(max(-1.0, min(1.0, float(unit(a) @ unit(b))))))


def walked(points):
    """Path length up to each point (m)."""
    p = np.asarray(points, dtype=float).reshape(-1, len(points[0])) if len(points) else np.zeros((0, 1))
    if len(p) < QUANT:
        return np.zeros(len(p))
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])


def resolved(points, rigid_m):
    """The points from the first one that stands rigid_m of path from the start: the part's own path (before
    that the hand seats in its hold and the servo gives). Empty until the path is that long."""
    w = walked(points)
    hit = np.nonzero(w >= rigid_m)[0]
    return [] if not len(hit) or rigid_m <= 0 and len(points) < QUANT else list(points[int(hit[0]):])


def window(points, length_m):
    """The newest displacement over the last length_m of path, or None while the path is shorter."""
    p = np.asarray(points, dtype=float)
    if len(p) < QUANT:
        return None
    w = walked(p)
    need = w[-1] - length_m
    if need < 0:
        return None
    i = int(np.nonzero(w <= need)[0][-1])
    span = w[i + 1] - w[i] if i + 1 < len(w) else 0.0
    start = p[i] + (p[i + 1] - p[i]) * ((need - w[i]) / span) if span > 0 else p[i]
    return p[-1] - start


def fit(points, resolution_m):
    """Line or arc through the points (m). resolution_m: what the robot can tell from no travel across the path
    (rigid): an arc is claimed only when the path sags from its chord by BAND times that and the circle explains
    the points better than GIVE of the line's error plus that resolution. Returns {shape: none | line | arc,
    direction (unit, the newest tangent), travel_m, rms_m, and for an arc centre, radius_m, axis (the plane's unit
    normal, right-handed with the travel), angle_rad}."""
    p = np.asarray(points, dtype=float)
    out = {"shape": "none", "direction": None, "travel_m": 0.0, "rms_m": 0.0, "centre": None, "radius_m": None,
           "axis": None, "angle_rad": 0.0, "points": int(len(p))}
    if len(p) < QUANT or float(np.linalg.norm(p[-1] - p[0])) <= max(resolution_m, TINY):
        return out
    chord = p[-1] - p[0]
    centre = p.mean(axis=0)
    z = p - centre
    _, _, V = np.linalg.svd(z, full_matrices=True)
    line = V[0] * (1 if V[0] @ chord >= 0 else -1)
    off = z - np.outer(z @ line, line)
    line_rms = float(np.sqrt(np.mean(np.sum(off * off, axis=1))))
    out.update(shape="line", direction=line, travel_m=float(walked(p)[-1]), rms_m=line_rms)
    if len(p) <= FIT_MIN:
        return out
    rel = p - p[0]
    sag = float(np.linalg.norm(rel - np.outer(rel @ unit(chord), unit(chord)), axis=1).max())
    if sag < BAND * resolution_m or sag <= TINY:
        return out
    xy = z @ V[:QUANT].T                                   # the points in their best plane
    A = np.column_stack([laws.TWO * xy, np.ones(len(xy))])
    sol = np.linalg.lstsq(A, np.sum(xy * xy, axis=1), rcond=None)[0]
    c2 = sol[:QUANT]
    r2 = float(sol[QUANT] + c2 @ c2)
    if r2 <= 0:
        return out
    radius = math.sqrt(r2)
    radial = xy - c2
    ang = np.unwrap(np.arctan2(radial[:, 1], radial[:, 0]))
    sweep = float(ang[-1] - ang[0])
    arc_rms = float(np.sqrt(np.mean((np.linalg.norm(radial, axis=1) - radius) ** 2 + (z @ V[QUANT]) ** 2)))
    if arc_rms > GIVE * line_rms + resolution_m or abs(sweep) <= TINY:
        return out
    axis = unit(np.cross(V[0], V[1])) * (1 if sweep >= 0 else -1)
    c3 = centre + V[:QUANT].T @ c2
    tangent = unit(np.cross(axis, p[-1] - c3))
    out.update(shape="arc", direction=tangent, centre=c3, radius_m=radius, axis=axis, angle_rad=abs(sweep),
               rms_m=arc_rms, travel_m=radius * abs(sweep))
    return out


def tangent(found, p, fallback):
    """The path's direction at point p: the line's own, or the arc's tangent there."""
    if found["shape"] == "arc":
        t = unit(np.cross(found["axis"], np.asarray(p, dtype=float) - found["centre"]))
        return t if np.any(t) else fallback
    return found["direction"] if found["shape"] == "line" else fallback


def axis_error_deg(found, true_axis):
    """Degrees between the fitted axis (a line's direction, an arc's normal) and a true one (sign left out)."""
    a = found["axis"] if found["shape"] == "arc" else found["direction"]
    if a is None:
        return None
    c = abs(float(unit(a) @ unit(true_axis)))
    return math.degrees(math.acos(min(1.0, c)))


def part_mass(forces, along, dt):
    """The part's reflected mass (kg) from the ramp's own force against acceleration: over the periods since it
    began to move, the line force = what resists it + mass x acceleration through the pairs (the second difference
    of its travel along the path, the force along it). None when the acceleration is not resolved: fewer than
    FIT_MIN pairs, no spread in the acceleration, or a slope that is not above zero."""
    f = np.asarray(forces, dtype=float)
    x = np.asarray(along, dtype=float)
    n = min(len(f), len(x))
    if n < FIT_MIN + QUANT or dt <= 0:
        return None
    f, x = f[:n], x[:n]
    a = np.diff(x, n=int(QUANT)) / dt ** 2
    y = f[1:-1]
    spread = float(np.sum((a - a.mean()) ** 2))
    if not spread > 0:
        return None
    slope = float(np.sum((a - a.mean()) * (y - y.mean()))) / spread
    return slope if slope > 0 else None


# ---------------------------------------------------------------------------------------------- the ramp (A4.5)
def ramp_step(B, f_now, ceiling_n):
    """One control period of the force ramp: newtons the commanded force may rise now (B.force's own rate)."""
    rate = laws.ramp_rate(f_now, B.sigma_f(), B.e_tau(), B.t_det(), ceiling_n)
    return 0.0 if rate is None else rate * B.dt()


def ramp_periods(B, first_n, ceiling_n):
    """Control periods the whole ramp may take from first_n to its ceiling (its own time with the call margin)."""
    t = laws.ramp_time(first_n, ceiling_n, B.sigma_f(), B.e_tau(), B.t_det(), ceiling_n)
    t = B.t_settle() if t is None else t
    return int(math.ceil(B.limit_s(t + B.t_settle()) / B.dt()))


def still_m(B, q, d, expected_m):
    """Travel that counts as none over a window in which expected_m was commanded (A4.8 no progress)."""
    return max(QUANT * B.quantum(q, d), REL * expected_m)


def window_periods(B):
    """Control periods of one settle time, at least what a decision on two readings takes."""
    return max(int(QUANT), int(math.ceil(B.t_settle() / B.dt())), int(math.ceil(B.t_det() / B.dt())))


# ---------------------------------------------------------------------------------------------- turning (A4.5)
def torque_about(J_rot, axis, ext):
    """The external torque about axis that the joint torques ext explain (N m): their projection on the joints'
    answer to a turn about that axis."""
    w = np.asarray(J_rot, dtype=float).T @ unit(axis)
    den = float(w @ w)
    return float(w @ np.asarray(ext, dtype=float)) / den if den > TINY else 0.0


def wrench_torque(J, axis, ext):
    """The external torque about axis at the tip (N m) when the joint torques ext are read as ONE wrench at the
    tip, a force and a couple (least squares over the whole Jacobian J, linear rows first): a push on the tip
    then takes no part of it. torque_about projects the joint torques on the turn alone and reads a push along
    the hand as torque (measured on the own bench: 2.04 N m read where the turn's own law held 0.27 N m); this
    reading is the one a limit stated for the part is held and shown against."""
    J = np.asarray(J, dtype=float)
    n = len(axis)
    w = np.linalg.lstsq(J.T, np.asarray(ext, dtype=float), rcond=None)[0]
    return float(w[n:] @ unit(axis))


def pad_load(squeeze_n, couple_nm, radius_m, pads, force_n):
    """An upper bound (N) of what ONE pad puts on a part the jaws hold while the hand turns it about its own axis,
    from the hand's own readings: the pad's squeeze, its share of the couple about the hand's axis at the jaws'
    radius, and the whole force the hand reads (one pad can carry all of it), added up (no direction of the
    three is assumed: the plain sum bounds every sum of them as vectors). squeeze_n None: not read."""
    turn = abs(float(couple_nm)) / (max(float(radius_m), TINY) * max(int(pads), 1))
    return float(squeeze_n or 0.0) + turn + float(force_n)


def joint_step(q_to, q_from, most):
    """The step from the joints q_from toward q_to that no joint takes faster than most (rad per period, per
    joint), ALL joints scaled by one share: the commanded pose stays on the straight joint way between the two,
    so a hand that only turns stays where it stands. Cutting each joint by itself bends that way: the turn then
    commanded a tip up to 23 mm off the pose it asked for and the servo drove the hand into the part with 73 N
    (measured on a development cell)."""
    dq = np.asarray(q_to, dtype=float) - np.asarray(q_from, dtype=float)
    most = np.asarray(most, dtype=float)
    over = np.abs(dq) > most
    share = float(np.min(most[over] / np.abs(dq[over]))) if np.any(over) else 1.0
    return dq * share


def turned(R_now, R_was, axis):
    """The angle (rad, signed) the hand turned about axis between two readings of its rotation. A turn sums it
    period by period: read at once against where the stroke began it turns its sign at half a turn."""
    from rcpx.model import _rotvec
    return float(_rotvec(np.asarray(R_now, dtype=float) @ np.asarray(R_was, dtype=float).T) @ unit(axis))


def torque_ceiling(B, q, weights):
    """What the joints that turn may add over what holding the pose takes (N m about the axis): the least room of
    a joint the turn uses, over its share of the turn."""
    room = np.maximum(0.0, np.asarray(B.tau_max(), dtype=float) - np.abs(B.robot.gravity_torque(q)))
    w = np.abs(np.asarray(weights, dtype=float))
    use = w > TINY
    return float(np.min(room[use] / w[use])) if np.any(use) else 0.0


def torque_step(B, tau_now, ceiling_nm, sigma_nm):
    """One control period of the torque ramp (the force ramp's law in N m)."""
    step = max(BAND * sigma_nm, REL * abs(tau_now), REL * REL * ceiling_nm)
    return step * B.dt() / B.t_det() if B.t_det() > 0 else step


def turn_resolved(B, weights):
    """The angle (rad) the robot can tell from none about the axis: QUANT quanta of the joints that turn."""
    w = np.abs(np.asarray(weights, dtype=float))
    quantum = np.asarray(B.joint_quantum(), dtype=float)
    return float(QUANT * np.max(quantum * (w > TINY))) if len(w) else 0.0


# ---------------------------------------------------------------------------------------------- the jaws (A4.7)
def jaw_read(ex, s=None):
    """(gap m, pressing, squeeze N or None, q, target) of the jaws now."""
    J = ex._fx_jaws
    g = s.gripper if s is not None else ex.backend.read().get("gripper")
    q, target = J.reading(g, ex._gripper_cmd)
    if q is None:
        return None, False, None, None, None
    return float(J.width_m(q)), bool(J.pressing(q, target)), J.squeeze_n(q, target), q, target


def jaw_step_m(ex):
    """The gap's own resolution (m): one joint quantum of the jaws carried to the gap."""
    J, B = ex._fx_jaws, bounds_of(ex)
    quantum = float(np.max(np.asarray(B.joint_quantum(), dtype=float))) if np.size(B.joint_quantum()) else 0.0
    return abs(float(J.width_m(J.closed + quantum * np.sign(J.span)) - J.width_m(J.closed)))


def squeeze_up(ex):
    """One ramp step more squeeze on what the jaws hold (after a slip): the command leads the reading by more.
    Returns False when the jaw servo's range is spent, or when a limit stated for the part is reached."""
    J = ex._fx_jaws
    gap, _, squeeze, q, target = jaw_read(ex)
    if q is None or J.kp is None or J.force_range is None or squeeze is None:
        return False
    top = float(np.min(J.force_range / J.lever))
    stated = (getattr(ex, "_fx", None) or {}).get("limit")
    if stated:                    # a limit stated for the part: each pad presses no more than its share of it
        top = min(top, float(stated["limit_n"]) / len(J.sides))
    want = min(top, squeeze + max(REL * squeeze, REL * REL * top))
    if want <= squeeze:
        return False
    lead = (want * J.lever + J.friction) / J.kp
    frac = J.frac_of_q(q - np.sign(J.span) * lead)
    if frac >= ex._gripper_cmd and ex._gripper_cmd <= 0:
        return False
    ex._gripper_cmd = min(1.0, max(0.0, frac))
    return True
