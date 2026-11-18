# SPDX-License-Identifier: AGPL-3.0-only
"""What the arm READ at a stop (an earlier round, p4-read): the touching point's speed from the joint speeds the backend
reported, and what the drives can push at that point. No robot is named; nothing is typed.

    note_reading(ex, s)                  -> keeps (t, q, qd) of each reading of the present call
    read_speed_m_s(ex, point_mm, body, t_s) -> (speed m/s, words): the touching point's speed the arm read
    drives_push_n(ex, q, u, point_mm, body) -> (N, words): the largest push the drives give at that point along u
    over_drives(ex, force_n, q, u, point_mm, body) -> (True when the force is over it, the push, words)

The speed at a stop is the larger of the speeds at the reading the stop was read at (t_s) and at the reading before
it (the contact may already have braked the point by then), each the point's speed from that reading's joint speeds
(J at the point times qd). With no reading of the call kept, the newest reading the executor holds is used and the
words say so. A point on a named body of the body model is taken fixed to that body; otherwise it is taken fixed to
the hand (the tip's motion and the hand's turn about the tip).
"""

import collections
import math

import numpy as np

MM = 1000.0
KEEP = 1500          # memory only: the readings of one call (a call longer than that keeps its newest)


def _ring(ex):
    """The readings of the present call (a new call starts an empty one: the last call's readings are no reading
    of this one)."""
    call = getattr(ex, "_calls", None)
    kept = ex.__dict__.get("_read_ring")
    if kept is None or kept[0] != call:
        kept = (call, collections.deque(maxlen=KEEP))
        ex.__dict__["_read_ring"] = kept
    return kept[1]


def note_reading(ex, s):
    """Keep (t, q, qd) of a reading (control_sense._sense calls it for every new reading)."""
    if s is None or getattr(s, "qd", None) is None:
        return
    _ring(ex).append((float(s.t), np.asarray(s.q, dtype=float).copy(), np.asarray(s.qd, dtype=float).copy()))


def _vec(v, n=3):
    try:
        a = np.asarray(v, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    return a if a.size == n and np.all(np.isfinite(a)) else None


def _body_id(model, body):
    if body is None or not hasattr(model, "mj"):
        return None
    try:
        return int(model.mj.body(str(body)).id)
    except (KeyError, TypeError, ValueError):
        return None


def point_jac(model, q, point_mm=None, body=None):
    """3 x n translational Jacobian (arm joints) of a point fixed to `body` (frame B, mm), or to the hand when the
    body is not one of the model's: the tip's Jacobian and the hand's turn about the tip. No point: the tip."""
    q = np.asarray(q, dtype=float)
    p = _vec(point_mm)
    bid = _body_id(model, body)
    if bid is not None and p is not None:
        import mujoco
        model._set(q)
        jp = np.zeros((3, model.mj.nv))
        jr = np.zeros((3, model.mj.nv))
        mujoco.mj_jac(model.mj, model.d, jp, jr, p / MM, bid)
        return jp[:, model._dadr], "the touching point on " + str(body)
    J = np.asarray(model.jac(q), dtype=float)
    if p is None:
        return J[:3], "the tip"
    tip = np.asarray(model.fk_tip(q)[0], dtype=float)
    r = p / MM - tip
    # v = v_tip + w x r = (Jv - [r]x Jw) qd
    skew = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])
    return J[:3] - skew @ J[3:], "the touching point, taken fixed to the hand"


def _readings_at(ex, t_s):
    """(reading at or before t_s, the one before it) from this call's kept readings; else the newest the executor
    holds, alone. Each a (t, q, qd)."""
    ring = list(_ring(ex))
    if ring:
        if t_s is None:
            idx = len(ring) - 1
        else:
            idx = max([i for i, r in enumerate(ring) if r[0] <= float(t_s) + 1e-9] or [0])
        return [ring[idx]] + ([ring[idx - 1]] if idx > 0 else []), "the readings of this call"
    s = getattr(ex, "_last", None)
    if s is None:
        s = ex._sense()
    qd = getattr(s, "qd", None)
    qd = np.zeros_like(np.asarray(s.q, dtype=float)) if qd is None else np.asarray(qd, dtype=float)
    return [(float(s.t), np.asarray(s.q, dtype=float), qd)], "the newest reading held"


def read_speed_m_s(ex, point_mm=None, body=None, t_s=None):
    """(speed m/s, words) of the touching point the arm READ at the stop: the larger of the speeds at the reading
    at t_s and the one before it (a reading with no joint speeds reads none)."""
    rows, where = _readings_at(ex, t_s)
    best, said = 0.0, "the tip"
    for _, q, qd in rows:
        J, said = point_jac(ex.model, q, point_mm, body)
        v = float(np.linalg.norm(J @ np.asarray(qd, dtype=float)))
        if math.isfinite(v):
            best = max(best, v)
    return best, f"the speed the arm read at the stop at {said} (its joint speeds; {where})"


def _tau_max(ex):
    B = getattr(ex, "bounds", None)
    try:
        tau = B.tau_max() if B is not None else None
        tau = tau.get("value") if isinstance(tau, dict) else (tau[0] if isinstance(tau, tuple) else tau)
        tau = np.asarray(tau, dtype=float).reshape(-1)
        if tau.size == ex.model.n and np.all(np.isfinite(tau)):
            return tau, "the bounds' joint torque limits"
    except Exception:          # bounds that cannot say: the body's own stated limits
        pass
    return np.asarray(ex.torque_limit, dtype=float).reshape(-1), "the body's stated joint torque limits"


def drives_push_n(ex, q, u, point_mm=None, body=None):
    """(N, words): the largest push the drives give at the point along unit u at pose q: a force f u there asks
    J_p^T u f of the joints, and no joint gives more than its limit, so f is at most min_j tau_j / |(J_p^T u)_j|
    over the joints the push loads. inf when the point's Jacobian loads no joint along u."""
    u = _vec(u)
    if u is None or float(np.linalg.norm(u)) <= 0.0:
        return math.inf, "no direction"
    u = u / float(np.linalg.norm(u))
    J, said = point_jac(ex.model, q, point_mm, body)
    lever = np.abs(J.T @ u)
    tau, src = _tau_max(ex)
    ok = lever > 1e-9 * max(1.0, float(np.max(lever)) if lever.size else 1.0)
    if not np.any(ok):
        return math.inf, f"a push along it at {said} loads no joint"
    return float(np.min(tau[ok] / lever[ok])), f"{src} over the levers at {said} along the push"


def over_drives(ex, force_n, q, u, point_mm=None, body=None, margin=0.0):
    """(True when force_n is over what the drives push at the point along u by more than `margin` of it, the push N,
    words)."""
    if force_n is None or not math.isfinite(float(force_n)):
        return False, None, "no force"
    cap, words = drives_push_n(ex, q, u, point_mm, body)
    return bool(float(force_n) > cap * (1.0 + float(margin))), cap, words
