# SPDX-License-Identifier: AGPL-3.0-only
"""Parts of the self-test's taps (rcpx/bounds_selftaps.py taps), kept apart for that file's length: the arm's own
force at each fast sample (_model_force, for a body with no force sensor: its fast joint speeds through its own
model and servo law), what a tap may reach and how fast the ladder climbs (_cap, _top), and the price of the way
down to the first tap (_way_price). See that module's text."""

import numpy as np

from rcpx import bounds_laws as laws
from rcpx import bounds_stops as stops
from rcpx.bounds_ratios import QUANT, REL


def _step(ex, q_cmd, direction, dist):
    from rcpx import bounds_selftaps
    return bounds_selftaps._step(ex, q_cmd, direction, dist)


def _servo(ex):
    """The joint servo law the backend states ({"kp", "kd", "torque_limit"}), or None where it states none."""
    law = ex.backend.servo_law() if hasattr(ex.backend, "servo_law") else None
    if not isinstance(law, dict) or law.get("kind") != "joint_pd" or law.get("kp") is None or law.get("kd") is None:
        return None
    return law


def _model_force(ex, before, s, target, frac, u, free=None):
    """(the arm's own force N at each fast sample of the period that ended at reading s, what the joints' dry
    friction can hide at a stop there N, the joint torques from outside that gave them: k x n), or None where the body
    gives no fast joint speeds or states no joint servo law. before: the reading the period began at; target:
    the servo target that stood over the period; frac: its torque share; u: the travel's direction (its length
    says how many rows of the tip's Jacobian are the position's); free: the joint torques from outside a free
    period of the same motion showed at the same samples (what the body's model leaves there is not a push:
    taken off). See the module text."""
    fast = s.fast if isinstance(getattr(s, "fast", None), dict) else None
    law = _servo(ex)
    rows = None if fast is None else fast.get("qd")
    if law is None or rows is None or len(rows) < QUANT or not fast.get("dt_s") or target is None:
        return None
    rows, h = np.asarray(rows, dtype=float), float(fast["dt_s"])
    kp, kd = np.asarray(law["kp"], dtype=float), np.asarray(law["kd"], dtype=float)
    top = np.asarray(law.get("torque_limit", ex.torque_limit), dtype=float) * float(frac)
    dry = np.asarray(getattr(ex, "_frictionloss", np.zeros(ex.model.n)), dtype=float)
    q, qd = np.asarray(before.q, dtype=float).copy(), np.asarray(before.qd, dtype=float).copy()
    force, hides, outside = [], [], []
    for k, row in enumerate(rows):
        tau = np.clip(kp * (np.asarray(target, dtype=float) - q) - kd * qd, -top, top)
        outside.append(tau - ex._model_torque(q, qd, (row - qd) / h))     # the arm on the world, per joint
        mine = outside[-1] if free is None or len(free) != len(rows) else outside[-1] - np.asarray(free[k], dtype=float)
        to_tip = np.linalg.pinv(np.asarray(ex.model.jac(q), dtype=float)[:len(u)].T)   # joint torque to tip force
        at_tip = to_tip @ mine
        size = float(np.linalg.norm(at_tip))
        along = np.abs(at_tip @ to_tip) / size if size > 0 else np.linalg.norm(to_tip, axis=0)
        force.append(size)
        # a joint's dry friction stands against the travel while the hand moves and against the way back once it
        # is stopped: at a stop it can change by twice its size, and the readings cannot tell that from a push
        hides.append(float(along @ (laws.TWO * dry)))
        q, qd = q + h * row, row
    return force, hides, outside


def _cap(ex, B, q, u):
    """(the least force a tap may reach N, its name, the least stated limit {"n", "what"} or None): what the arm
    can push there, or a stated tool rating or site force under it."""
    stated = []
    if B.f_tool() is not None:
        stated.append((float(B.f_tool()), "the tool's stated rating"))
    if B.site and B.site.get("force_n") is not None:
        stated.append((float(B.site["force_n"]), "the site's stated force"))
    said = None if not stated else {"n": min(stated)[0], "what": min(stated)[1]}
    push = (float(B.f_cap(q, u)) * (1.0 - float(B.e_tau())), "what the arm can push there (its own joint torques)")
    limit, why = min([push] + stated)
    return limit, why, said


def _top(ex, B, q, u, free, model_z, got=None):
    """(the ladder's top speed m/s, why, the least stated limit {"n", "what"} or None): see the module text.
    model_z: the largest of the body's model of the stop slope on the way down to the support. got: the fit of
    the taps made so far (none: the body's model alone)."""
    if free:
        return float(B.v_free(q, u)), "the free speed (cfg selftest_taps free: the owner's statement)", None
    limit, why, said = _cap(ex, B, q, u)
    z, source = stops.slope(model_z, got)
    by = "the taps' own slope" if str(source).startswith("fitted") else "the body's model"
    room = limit - float(B.f_thr(0.0, 0.0))
    if z is None:
        return 0.0, "the body's model gives no stop slope here", said
    if not room > 0:
        return 0.0, f"{why}, {limit:.3g} N, is at or under the level the guard runs with at rest", said
    return room / z, f"where the peak by {by} reaches {why}, {limit:.3g} N", said


def _way_price(ex, B, q0, u, dist, v, f_guard):
    """What the way down could strike (rcpx.bounds_strike.price of that line): the line's poses by the same steps
    the drive makes, the tool's fastest point at the line's top speed, the guard's own threshold."""
    from rcpx import bounds_strike
    n = int(1 / REL)
    poses = [np.asarray(q0, dtype=float)]
    for _ in range(n):
        poses.append(_step(ex, poses[-1], u, dist / n))
    speeds = [np.linalg.pinv(np.asarray(ex.model.jac(q), dtype=float)[:len(u)]) @ (np.asarray(u, dtype=float) * v) for q in poses]
    got = bounds_strike.price([bounds_strike.stretch_of(B, poses, speeds, None, f_guard)],
                              stops.fit(B.know.stops()), bounds_strike.detection(B)[0])
    return {k: got[k] for k in ("peak_n", "at_least_n", "model_peak_n", "v_m_s", "f_guard_n", "z_source", "text")}
