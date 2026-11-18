# SPDX-License-Identifier: AGPL-3.0-only
"""Side words (SPEC-R3.md 3.4): robot-relative directions from the base axis and gravity, and one device-relative
direction (front). Pure: numbers in, words and unit vectors out; nothing here reads a device, a robot name or a scene.

Frame B (SPEC.md section 0): x forward, y left, z up, in mm. A "thing" is a point in B (a located point, a contact).

    radial(p_mm)                 the horizontal unit vector from the base axis to p; +x within RADIAL_NEAR_MM
    side_vector(side, p_mm)      robot_side = -r, far_side = +r, left = z x r, right = -(z x r), above = +z
    front(centre_mm, p_mm)       the horizontal unit vector from a device centre to p (None nearer than 15 mm)
    words(h, p_mm)               a direction in words, by its angle to r at p ("away from the robot", "up", ...)
    side_of(h, p_mm)             the side name (robot_side, far_side, left, right, above, below) h points to
    angle_deg(a, b)              the angle between two vectors (0 to 180)

Every number here is in rcpx.r3_limits with its reason.
"""

import math

import numpy as np

from rcpx import r3_limits as L

Z = np.array([0.0, 0.0, 1.0])
SIDES = ("robot_side", "far_side", "left", "right", "above")
WORDS = ("up", "down", "away from the robot", "toward the robot", "to the robot's left", "to the robot's right")


def _vec(v, n=3):
    a = np.asarray(v, dtype=float).reshape(-1)
    if a.shape[0] < n:
        a = np.concatenate([a, np.zeros(n - a.shape[0])])
    return a[:n]


def _unit(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return None if n < 1e-12 else v / n


def radial(p_mm):
    """r: the horizontal unit vector from the base axis (the z axis of B) to p. A point within RADIAL_NEAR_MM of the
    axis has no direction of its own there: r is +x (forward) by convention."""
    p = _vec(p_mm)
    h = np.array([p[0], p[1], 0.0])
    if float(np.linalg.norm(h)) < L.RADIAL_NEAR_MM:
        return np.array([1.0, 0.0, 0.0])
    return _unit(h)


def side_vector(side, p_mm):
    """The unit vector of a robot-relative side at p: robot_side = -r, far_side = +r, left = z x r,
    right = -(z x r), above = +z. Raises ValueError for any other name (front is device-relative: see front)."""
    r = radial(p_mm)
    left = np.cross(Z, r)
    table = {"robot_side": -r, "far_side": r, "left": left, "right": -left, "above": Z.copy()}
    if side not in table:
        raise ValueError(f"unknown side {side!r}; robot-relative sides: {', '.join(SIDES)}")
    return table[side]


def front(centre_mm, p_mm):
    """The device-relative front at p: the horizontal unit vector from the device centre (x, y) to p. None when the
    centre is unknown or p is nearer than FRONT_MIN_MM to it horizontally (no side is defined there)."""
    if centre_mm is None or p_mm is None:
        return None
    c, p = _vec(centre_mm, 2), _vec(p_mm)
    d = np.array([p[0] - c[0], p[1] - c[1], 0.0])
    if float(np.linalg.norm(d)) < L.FRONT_MIN_MM:
        return None
    return _unit(d)


def angle_deg(a, b):
    """The angle between a and b in degrees (0 to 180); 0 when either is a zero vector."""
    ua, ub = _unit(_vec(a)), _unit(_vec(b))
    if ua is None or ub is None:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, float(ua @ ub)))))


def words(h, p_mm):
    """A direction h at the thing p in words, by its angle to r (SPEC-R3 3.4): a mostly vertical h (|h_z| over
    VERTICAL_Z of its length) is "up" or "down"; else the horizontal part's angle to r: within AWAY_DEG "away from
    the robot", TOWARD_DEG or more "toward the robot", else "to the robot's left" or "to the robot's right" by the
    sign of z . (r x h)."""
    u = _unit(_vec(h))
    if u is None:
        return "nowhere (a zero direction)"
    if abs(float(u[2])) > L.VERTICAL_Z:
        return "up" if u[2] > 0 else "down"
    r = radial(p_mm)
    a = angle_deg(np.array([u[0], u[1], 0.0]), r)
    if a <= L.AWAY_DEG:
        return "away from the robot"
    if a >= L.TOWARD_DEG:
        return "toward the robot"
    return "to the robot's left" if float(Z @ np.cross(r, u)) > 0 else "to the robot's right"


SIDE_OF_WORDS = {"up": "above", "down": "below", "away from the robot": "far_side",
                 "toward the robot": "robot_side", "to the robot's left": "left", "to the robot's right": "right"}


def side_of(h, p_mm):
    """The side name h points to at p (the same rule as words): above, below, far_side, robot_side, left, right."""
    return SIDE_OF_WORDS.get(words(h, p_mm), "none")


__all__ = ["SIDES", "WORDS", "angle_deg", "front", "radial", "side_of", "side_vector", "words"]
