# SPDX-License-Identifier: AGPL-3.0-only
"""What a rigid thing can push back with, per unit of speed, at ONE moving point of the body (round six, second
pass; the design rulings; contract C5 changed). Pure but for the body's own description.

A strike has two parts, and each is taken here from the body's own description alone:
- the impact: the contact model's rate the backend states (b, 1/s: force per unit of moving mass and arrival
  speed) times the mass that moves along the travel at that point when the thing stops the point DEAD:
  held_fast(A, u) = u' A^+ u with A the point's mobility (J M^-1 J', 1/kg). It is never under the plain
  directional mass 1 / (u' A u) of a point that may slide aside (the first pass's model, which read under the
  truth where a thing held the point: 2.08 times on one row).
- the servo: while the stop is not yet told the command runs on and the servo winds up against the thing: the
  servo's stiffness at the point held the same way (the same form over the point's compliance J Kp^-1 J', m/N)
  times the time the command leads by (the measured detection time plus the lag).
The two are added though they peak at different times. Both are taken as held_fast(): the point held fast,
at the tool and at a link's point alike, with no typed share: where the held-fast figure runs
away with the lever (a link's point near the base while the hand moves fast) the energy bound of moving() is what
bounds it, and a SOLVE at a link's point reads moving() at the leg's own joint step (terms(..., qd=...)).

It is a MODEL, not a bound, and nothing here calls it one. It is the stop of one point by a thing that holds
that point. A thing can do more: stand askew to the travel where the point can hardly move across (a wedge
against the body's own bearings: the force grows as one over the angle), hold a whole body by several points
(its turning too), or pin a link near the base, where the whole arm's motion is taken out through a lever that
goes to nothing. On rows nobody fitted to (252 contacts of five arms, any pose, travel and speed, a box that
stands askew) the truth read up to 2.3 times this model at the point that met, on a link and, once, on the
tool at three times the speed of its fastest tap. So a figure is stated from it only inside what this body's
own stops cover (rcpx/bounds_strike.py), and a body that states no contact model has no such model at all.

- held_fast(A, u): see above; None where the point cannot move along u at all.
- tap_parts(B, q, u): the facts at the tip along u, for a tap and for an arrival solved along the travel.
- terms(B, q, J, u, body, held_kg, qd): {"m_kg", "k_n_m", "impact", "servo", "b_per_s", "lead_s"}: the two terms
  at a point with position Jacobian J moving along u; None values where the body states no contact model or no
  servo. With qd (joint speeds, any scale: the masses do not change with it) moving()'s energy bound gives the
  terms, along the point's own travel there.
- slope_of(terms): the two terms together; None with no contact model.
- moving(B, q, qd, J, body, held_kg): THE ENERGY BOUND at a point moving with the joint speeds
  qd: terms() along the point's own travel with no typed share, the mass taken as the smaller of the mass held
  fast and the mass the moving body's kinetic energy can put behind that point (qd' M qd over the point's speed
  squared: Cauchy-Schwarz in the joint inertia metric), and the servo's stiffness the same way in its own metric
  (qd' Kp qd over the point's speed squared: the spring energy the command stores while it leads). Each term says
  which of the two bounds set it. No tap and no fitted share enters it; it needs the body's own description only.
- push(B, q, J, qd, soft): the most force the joints' stated torque limits let the servo put on a held point
  along the way its own force builds up there: the bound of a servo that runs on against a contact the guard is
  not shown to see (a link's: no stop was made with a link).
"""

import math

import numpy as np

CUT = float(np.sqrt(np.finfo(float).eps))  # numeric: an eigenvalue under this share of the largest is the
                                           # arithmetic's own error: a direction the point cannot move in


def held_fast(A, u):
    """u' A^+ u: what the point shows along unit u when the thing holds it FAST there (all three directions are
    held, and the part of the answer along u is read): the mass along u for A the point's mobility, the
    stiffness for its compliance. Never under the plain 1 / (u' A u) of a point that may slide aside. None when
    u has no part in A's range."""
    A, u = np.asarray(A, dtype=float), np.asarray(u, dtype=float)
    lam, vec = np.linalg.eigh(A)
    keep = lam > CUT * max(float(lam.max()), 0.0)
    a = (vec.T @ u)[keep]
    if not keep.any() or not float(a @ a) > 0:
        return None
    return float(np.sum(a * a / lam[keep]))


def tap_parts(B, q, u):
    """The model's facts at the tip along u ({"m_eff", "b_per_s", "k_servo", "lead_s"}): what a tap onto the
    stated support is measured against."""
    q, u = np.asarray(q, dtype=float), np.asarray(u, dtype=float)
    J = np.asarray(B.robot.jac_tip(q), dtype=float)[:3]
    m_inv, soft, lead = _facts(B, q)
    model = B.robot.contact_model() or {}
    b = model.get("b_per_s")
    return {"m_eff": held_fast(J @ m_inv @ J.T, u), "b_per_s": max(b.values(), default=None) if isinstance(b, dict) else b,
            "k_servo": None if soft is None else held_fast(J @ soft @ J.T, u), "lead_s": lead}


def _facts(B, q, held_kg=0.0):
    """(M^-1, the joints' compliance 1 / kp as a diagonal or None, the time the command leads by) at pose q: read
    once per pose (the last one asked is kept)."""
    key = (np.asarray(q, dtype=float).tobytes(), float(held_kg))
    kept = getattr(B, "_point_facts", None)
    if kept is not None and kept[0] == key:
        return kept[1]
    M = B.robot.mass_matrix(q)
    if held_kg:
        tip = B.robot.jac_tip(q)[:3]
        M = M + held_kg * (tip.T @ tip)
    servo = B.robot.servo_law()
    kp = np.asarray(servo.get("kp", B.robot.kp) if servo and servo.get("kind") == "joint_pd" else B.robot.kp, dtype=float)
    soft = np.diag(np.where(kp > 0, 1.0 / np.where(kp > 0, kp, 1.0), 0.0)) if np.any(kp > 0) else None
    v = _v_free(B, q)
    lead = float(B.t_det()) + (float(B.lag(v)) / v if math.isfinite(v) and v > 0 else 0.0)
    out = (np.linalg.pinv(M), soft, lead)
    B._point_facts = (key, out)
    return out


def _v_free(B, q):
    """The tip's own free speed at q straight down (m/s): the speed the lag is read at (the lag over the speed is
    the servo's own time, the same along any travel)."""
    from rcpx import bounds_laws as laws
    from rcpx.bounds_robot import pinv
    return laws.v_free(B.qd_max(), pinv(B.robot.jac_tip(q)[:3]) @ np.asarray((0.0, 0.0, -1.0)))


def terms(B, q, J, u, body=None, held_kg=0.0, qd=None):
    """The two terms of the stop slope at a point whose position Jacobian is J (3 x n), moving along unit u, with
    their facts: {"m_kg": the mass that moves along u when the point is held fast, "k_n_m": the servo's
    stiffness held the same way, "b_per_s", "lead_s": the detection time plus the lag, "impact": b x m, "servo":
    k x lead}. impact is None where the backend states no contact model, servo None where it states no stiffness;
    m_kg or k_n_m None where the point cannot move along u. qd: the joint speeds the point moves with (any scale):
    the terms are then moving()'s, the energy bound along the point's own travel ("m_from" says which mass set it)."""
    if qd is not None:
        got = moving(B, q, qd, J, body, held_kg)
        if got is not None:
            return {k: got[k] for k in ("m_kg", "k_n_m", "b_per_s", "lead_s", "impact", "servo", "m_from")}
    q, u, J = np.asarray(q, dtype=float), np.asarray(u, dtype=float), np.asarray(J, dtype=float)
    m_inv, soft, lead = _facts(B, q, held_kg)
    m = held_fast(J @ m_inv @ J.T, u)
    k = held_fast(J @ soft @ J.T, u) if soft is not None else None
    model = B.robot.contact_model() or {}
    by = (model.get("by_body") or {}).get(body) if body is not None else None
    b = model.get("b_per_s") if by is None else by
    b = max(b.values(), default=None) if isinstance(b, dict) else b
    return {"m_kg": m, "k_n_m": k, "b_per_s": b, "lead_s": lead,
            "impact": None if b is None or m is None else float(b) * m,
            "servo": None if k is None else k * lead}


def slope_of(got):
    """The body's model of the stop slope (N per m/s) from terms(): the two terms together; None where the body
    states no contact model (a servo term alone is no bound of a strike)."""
    if got is None or got.get("impact") is None:
        return None
    return float(got["impact"]) + float(got.get("servo") or 0.0)


HELD, ENERGY = "the mass held fast there", "the moving body's kinetic energy"
SPRING, STORED = "the servo's stiffness held fast there", "the spring energy the servo's command stores while it leads"


def _smaller(fast, metric, qd, v):
    """(the smaller of the held fast figure and qd' metric qd / v^2, which one set it): the energy cap of moving()."""
    cap = float(qd @ metric @ qd) / (v * v) if v > 0 else None
    if fast is None:
        return cap, cap is not None
    if cap is None or fast <= cap:
        return fast, False
    return cap, True


def moving(B, q, qd, J, body=None, held_kg=0.0, M=None):
    """The energy bound at a point with position Jacobian J (3 x n) at pose q, moving with joint speeds qd (see the
    module text). {"v": the point's speed (m/s), "u": its travel, "m_kg", "m_from", "k_n_m", "k_from", "b_per_s",
    "lead_s", "impact", "servo", "z": the slope (N per m/s; None with no contact model)}; None where the point
    does not move. A strike there is at most the guard's level plus z times v: the impulse a rigid contact needs
    to stop that point is the contact rate times the mass behind it, and no more mass can be behind the point
    than the energy the joints carry gives at that speed. M: the joint mass matrix at q where the caller has it."""
    q, qd, J = np.asarray(q, dtype=float), np.asarray(qd, dtype=float), np.asarray(J, dtype=float)
    w = J @ qd
    v = float(np.linalg.norm(w))
    if not v > 0 or not math.isfinite(v):
        return None
    u = w / v
    m_inv, soft, lead = _facts(B, q, held_kg)
    M = np.asarray(B.robot.mass_matrix(q) if M is None else M, dtype=float)
    if held_kg:
        tip = np.asarray(B.robot.jac_tip(q), dtype=float)[:3]
        M = M + held_kg * (tip.T @ tip)
    m, m_cap = _smaller(held_fast(J @ m_inv @ J.T, u), M, qd, v)
    k, k_cap = (None, False)
    if soft is not None:
        kp = np.diag(np.where(np.diag(soft) > 0, 1.0 / np.where(np.diag(soft) > 0, np.diag(soft), 1.0), 0.0))
        k, k_cap = _smaller(held_fast(J @ soft @ J.T, u), kp, qd, v)
    model = B.robot.contact_model() or {}
    by = (model.get("by_body") or {}).get(body) if body is not None else None
    b = model.get("b_per_s") if by is None else by
    b = max(b.values(), default=None) if isinstance(b, dict) else b
    impact = None if b is None or m is None else float(b) * m
    servo = None if k is None else k * lead
    return {"v": v, "u": [float(x) for x in u], "m_kg": m, "m_from": ENERGY if m_cap else HELD, "k_n_m": k,
            "k_from": STORED if k_cap else SPRING, "b_per_s": None if b is None else float(b), "lead_s": float(lead),
            "impact": impact, "servo": servo, "push_n": push(B, q, J, qd, soft),
            "z": None if impact is None else float(impact) + float(servo or 0.0)}


def push(B, q, J, qd, soft):
    """The most force the joints can put on a point with position Jacobian J at pose q while the point is held and
    the servo runs on along the joint speeds qd (N). Held there, the servo's own force on the point builds up along
    d = K w, w = J qd the point's commanded travel and K = (J Kp^-1 J')^+ its stiffness held fast (soft: Kp^-1), and
    it stops growing where a joint reaches its stated torque limit (plus the gravity it holds there): along d at
    most the least of (limit_j + |gravity_j|) / |(J' d)_j| over the joints that move the point; and where the
    joints that move it stand still because it is held, the force their torques at the limit make there, the way
    the command turns each (the larger of the two is stated). It holds whatever
    the time a contact there goes unseen: no command makes a joint push harder than its own limit. None where the
    body states no servo or no joint moves the point."""
    J, qd = np.asarray(J, dtype=float), np.asarray(qd, dtype=float)
    if soft is None:
        return None
    tau = np.abs(np.asarray(B.tau_max(), dtype=float).reshape(-1))
    g = np.abs(np.asarray(B.robot.gravity_torque(q), dtype=float).reshape(-1))
    if J.ndim != 2 or tau.shape != (J.shape[1],) or g.shape != tau.shape:
        return None
    C = J @ np.asarray(soft, dtype=float) @ J.T
    lam, vec = np.linalg.eigh(C)
    keep = lam > CUT * max(float(lam.max()), 0.0)
    if not keep.any():
        return None
    d = vec[:, keep] @ ((vec[:, keep].T @ (J @ qd)) / lam[keep])
    if not float(np.linalg.norm(d)) > 0:
        return None
    lever = np.abs(J.T @ (d / float(np.linalg.norm(d))))
    moves = (lever > CUT * float(lever.max())) & ((tau + g) > 0)
    if not moves.any():
        return None
    along = float(np.min((tau + g)[moves] / lever[moves]))
    # held fast, the joints that move the point cannot turn: each one's servo error grows the way the command
    # turns it until its torque stands at its limit, and the point then bears the force those torques make there
    # (J' F = the torques over what holds gravity; least squares where more joints move it than it has directions)
    turns = np.abs(qd) > CUT * float(np.abs(qd).max()) if np.any(qd) else np.zeros_like(qd, dtype=bool)
    use = moves & turns
    if not use.any():
        return along
    sat = np.sign(qd[use]) * (tau + g)[use]
    from rcpx.bounds_robot import pinv
    pinned = float(np.linalg.norm(pinv(J[:, use].T) @ sat))
    return max(along, pinned) if math.isfinite(pinned) else along
