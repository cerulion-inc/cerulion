# SPDX-License-Identifier: AGPL-3.0-only
"""Derived quantities of the selfcheck package, in SI: how far a wiggle turns, how fast and how far a stop
search runs, what a stop reading is worth. Nothing here is a typed limit: every value comes from the body model
(Jacobian, gravity torque, joint ranges, the camera), from the fit's own numerics (selfcheck_fit.py), from
rcpx.bounds (joint speed and acceleration, detection time, joint torque threshold, the hand's weight, the joint
quantum) and from what this session's checks have measured. Pure helpers: no state, no motion.

- stop_angle: the angle a joint needs to stop from a joint speed (detection time and braking). It is the margin a
  wiggle and a search keep inside a joint limit, and part of the slow zone.
- amplitudes: per joint the least turn that moves the picture by the pixels the fit needs (fit.need_px) and is at
  least 1 / REL encoder steps; an asked amplitude is the operator's word.
- zero_slow: the joint speed whose stop on the joint's own end stop stays under what that stop may take: no more
  than the hand's own weight brought to the joint for a stop never found, or, on the operator's word firm, the
  joint's own rating less what holds the arm; against the servo's wind-up over the detection time and, where a
  stop slope exists, the strike.
- zero_zone, zero_known, offset_range: where the search must be slow: the widest zero offset this session's
  checks have shown plus the stopping angle from full speed; the joint's whole range when nothing was measured.
- stop_spread: what the torque the joint cannot tell from none leaves in the servo and in the stop after the ease,
  plus the encoder step.
- contact_step: the least joint lead the robot can tell from none (the joint lead guard of selfcheck_guard.py).
"""
import math
import numpy as np
from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx import bounds_laws as laws
from rcpx.r2 import selfcheck_fit as fit
DEG = math.pi / 180.0


def lever(ex, q, j):
    return float(np.linalg.norm(ex.model.jac(q)[:3, j]))


def stop_angle(ex, q, j, w):
    B = bounds.of(ex)
    a = B.qdd_max(q)[j]
    if w == 0:
        return 0.0
    return w * B.t_det() + (w ** 2 / (laws.TWO * a) if a > 0 else math.inf)


def amplitudes(ex, q, joints, floor_z, asked_rad=None):
    if asked_rad is not None:
        return {j: asked_rad for j in joints}, "the operator's stated angle"
    gains = fit.picture_gain(ex.model, q, floor_z)
    quantum = bounds.of(ex).joint_quantum()
    seen = {j: max(fit.need_px() / gains[j], quantum[j] / R.REL) for j in joints
            if gains[j] * DEG > fit.BLIND_PX}
    blind = min(seen.values()) if seen else None
    return ({j: seen[j] if j in seen else (blind if blind is not None else quantum[j] / R.REL)
             for j in joints}, "the least picture motion the gain fit can read, above the encoder steps it needs")


def zero_known(ex, st, name):
    values = [(abs(r["deg"]) + R.BAND * r["sigma_deg"]) * DEG
              for r in st.get("zero_history", {}).get(name, [])]
    records = st.get("measured", {}).get(name)
    if records is not None:
        for rec in ([records] if isinstance(records, dict) else records):
            values.append((abs(rec["deg"]) + R.BAND * rec["sigma_deg"]) * DEG)
    records = st.get("zero", {}).get(name)
    if records is not None:
        for rec in ([records] if isinstance(records, dict) else records):
            values.append(abs(rec["offset_deg"]) * DEG)
    return (max(values), "the widest zero offset this session's checks showed") if values else (None, "no zero offset measured yet")


def zero_allow(ex, q_end, j, firm):
    B = bounds.of(ex)
    rated = B.tau_max()[j] * (1 - B.e_tau()) - abs(ex.model.gravity_torque(q_end)[j])
    value = rated if firm else min(rated, B.f_hand() * lever(ex, q_end, j))
    return value, ("on your word: firm, the joint's own rating" if firm else "the hand's own weight at this joint")


def zero_slow(ex, q_end, j, firm):
    B = bounds.of(ex)
    tau, words = zero_allow(ex, q_end, j, firm)
    ell = lever(ex, q_end, j)
    u = np.asarray(ex.model.jac(q_end)[:3, j], float)
    u = u / ell if ell > 0 else np.array([0.0, 0.0, 1.0])
    z, source = B.slope(q_end, u)
    den = ex.kp[j] * B.t_det() + (z * ell ** 2 if z is not None else 0)
    margin = tau - B.tau_thr(0.0)[j]
    w = max(0, margin / den) if den > 0 else 0.0
    if z is None:
        words += "; peak unresolved: only the servo's wind-up is bounded"
    else:
        words += "; stop slope " + source
    return min(B.qd_max()[j], w), words


def zero_zone(ex, st, q, j, name):
    known, words = zero_known(ex, st, name)
    if known is None:
        return offset_range(ex, j), "no zero measured: slow search over the whole joint range"
    return known + stop_angle(ex, q, j, bounds.of(ex).qd_max()[j]), words + " plus stopping angle at full joint speed"


def stop_spread(ex, j, give_rad, load_nm):
    """Spread after easing: encoder step and give under unfelt torque only.

    The search measures stop stiffness as load divided by the eased give.
    With no measured load the stop's stiffness term contributes nothing.
    """
    B = bounds.of(ex)
    threshold = B.tau_thr(0.0)[j]
    return (B.joint_quantum()[j] + threshold / ex.kp[j] +
            (threshold * give_rad / load_nm if load_nm > 0 else 0))


def contact_step(ex, v=0.0):
    """Least joint lead distinguishable from encoder rounding or unfelt torque."""
    B = bounds.of(ex)
    return np.maximum(R.QUANT * B.joint_quantum(), B.tau_thr(v) / ex.kp)


def offset_range(ex, j):
    return float(ex.model.q_hi[j] - ex.model.q_lo[j])


def camera_facts(ex):
    """The camera's own measured levels (its hidden level and noise) as the robot's self-knowledge holds them,
    for rcpx.perception's dark and dim levels; None when they were not measured."""
    try:
        value = bounds.of(ex).know.get("camera")[0]
    except Exception:      # self-knowledge that cannot answer is no camera fact: the light is still read
        return None
    return value if isinstance(value, dict) and ("hidden" in value or "noise" in value) else None
