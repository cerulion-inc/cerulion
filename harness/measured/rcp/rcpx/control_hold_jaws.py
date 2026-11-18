# SPDX-License-Identifier: AGPL-3.0-only
"""The jaws' own numbers (round six, contract C7): what the free jaws do, measured at STEADY closing rates.

    numbers(ex)       the numbers a close and a release run on: the robot's stored self-knowledge (entry jaw) when it
                      holds them, else the body's own description (source model, said in every receipt)
    measure_jaws(ex)  the measurement, called by the self-test in its calibration session, never inside a task
    facts(ex)         the jaw servo as the body's description gives it (gain, force range, joint friction, the pad's
                      levers, the inertia each jaw joint moves, the contact rate of the jaw's shapes)

Every rate is a share of the jaws' span per second, every opening and lag a share of the span, forces N, z_jaw N per
(m/s) of the gap's closing speed at the jaw tips. A free number comes from a jaw closing at a steady rate and stopped
before the closed end: never from a step command, never from an end stop, never from a stop on a thing. The closed
gap (where the jaws meet) and the rest opening are their own numbers, not hidden levels. Nothing here names a robot.
"""

import numpy as np

from rcpx import bounds_ratios as R

TINY = np.finfo(float).tiny


def _val(x):
    """A stored entry's value: (value, source, at), {value, ...} or the bare value."""
    if isinstance(x, dict) and "value" in x:
        return x["value"]
    if isinstance(x, tuple):
        return x[0] if x else None
    return x


def facts(ex):
    """The jaw servo from the body's own description, read once; None for a robot with no jaws. Per moving jaw
    joint: span (joint units), sign of opening, kp, force range, dry friction, the servo's own damping over its
    gain (s), the tip's lever and the pad's shortest lever (m; 1 for a slide, whose force is the joint's own), the
    inertia the joint moves (with the armature its file states). b_per_s: the contact rate of the jaws' own shapes
    as the backend states it (None: no contact model). gap_per_span_m: what one span closes at the tips."""
    got = ex.__dict__.get("_jaw_facts")
    if got is not None:
        return got or None
    if not ex.model.spec.get("gripper"):
        ex.__dict__["_jaw_facts"] = {}
        return None
    from rcpx.r2.place_hand import Jaws      # the hand's own geometry from the body's model; no package state
    J = getattr(ex, "_pl_jaws", None) or Jaws(ex.model)
    m = ex.model.mj
    jids = [m.joint(j).id for j in J.joints]
    dofs = [int(m.jnt_dofadr[j]) for j in jids]
    span = np.maximum(np.abs(J.span), TINY)
    kp = None if J.kp is None else np.maximum(np.asarray(J.kp, dtype=float), TINY)
    damp = np.asarray([float(m.dof_damping[d]) for d in dofs])
    for a in range(m.nu):                    # a position actuator's own speed term adds to the joint's damping
        j = int(m.actuator_trnid[a][0])
        if j in jids:
            damp[jids.index(j)] += abs(float(m.actuator_biasprm[a][-1]))
    lever_min, _ = J.pad_levers()
    cm = getattr(ex.backend, "contact_model", None)
    cm = cm() if callable(cm) else cm
    b = None
    if isinstance(cm, dict):
        by = cm.get("by_body") or {}
        mine = [float(by[n]) for n in ex.model.jaw_bodies() if isinstance(by.get(n), (int, float))]
        b = max(mine) if mine else cm.get("b_per_s")
    mu = _pad_friction(ex, m)
    out = {"J": J, "joints": list(J.joints), "span": span, "kp": kp, "force_range": J.force_range,
           "friction": np.asarray(J.friction, dtype=float), "tau_s": None if kp is None else damp / kp,
           "lever_tip": np.asarray(J.lever, dtype=float), "lever_min": np.asarray(lever_min, dtype=float),
           "inertia": np.asarray([float(m.dof_M0[d]) for d in dofs]), "b_per_s": None if b is None else float(b),
           "gap_per_span_m": max(float(J.table[-1] - J.table[0]), TINY), "one_jaw": bool(J.one_jaw),
           "mu": mu, "cone": 1.0 if mu is None else float(np.hypot(1.0, mu))}
    ex.__dict__["_jaw_facts"] = out
    return out


def _pad_friction(ex, m):
    """The largest sliding friction coefficient of the tool's own collision shapes (the last arm joint's body and
    everything under it: both jaws, a fixed one too), from the body's description; None where it states none. A
    pad's force on a thing lies inside that contact's friction cone about the pad's face: its push along the
    closing axis times hypot(1, mu) bounds the whole force of the pair (the "cone" of facts()) for a thing whose
    own friction is no larger than the pads' (a contact's friction taken as the larger of its two shapes'; a
    thing that grips harder than the pads is not bounded by it, and the figure's source says so)."""
    names = getattr(ex, "_tool_bodies", None) or set(ex.model.jaw_bodies())
    got = [float(m.geom_friction[g][0]) for g in range(m.ngeom)
           if m.body(int(m.geom_bodyid[g])).name in names and (m.geom_contype[g] or m.geom_conaffinity[g])]
    return max(got) if got else None


def z_model(F, lag_s=None, dt=None):
    """The jaws' stop slope by the body's model, N per (m/s) of the gap's closing speed, over the pad's shortest
    lever: the inertia each jaw joint moves times the contact model's rate (a jaw of inertia I turning at w meets
    a backed thing at the lever l with I x b x w / l), plus what the jaw servo itself winds up before the touch is
    told (its gain times the command's lead when the touch is told: the free lag's slope lag_s with the REL a
    touch must stand over it, plus the QUANT periods it is told on, per unit of rate). None when the body states
    no contact model."""
    if F["b_per_s"] is None:
        return None
    per_span = F["inertia"] * F["b_per_s"] * F["span"] / F["lever_min"]      # N per (span / s), each joint
    if F["kp"] is not None and lag_s is not None and dt is not None:
        lead_s = float(lag_s) * (1.0 + R.REL) + R.QUANT * float(dt)          # the lead at "told", s per unit rate
        per_span = per_span + F["kp"] * lead_s * F["span"] / F["lever_min"]
    return float(np.max(per_span)) / F["gap_per_span_m"]


def floor_of(F, a_share=None):
    """The squeeze the jaw joint's own friction leaves standing once the jaws touch (N): kp times the free lag at
    rate zero over the pad's shortest lever; with no measured lag the joint's dry friction over that lever."""
    if a_share is not None and F["kp"] is not None:
        return float(np.max(F["kp"] * float(a_share) * F["span"] / F["lever_min"]))
    return float(np.max(F["friction"] / F["lever_min"]))


def model_numbers(ex, old=None):
    """C7's numbers from the body's own description, for a robot that has not measured its jaws at steady rates
    (source model: a receipt that rests on them says so). old: round four's stored entry (its speed and closed gap
    in the jaw joints' own unit), used for those two where it has them."""
    F = facts(ex)
    span = float(np.max(F["span"]))
    old = old if isinstance(old, dict) else {}
    speed = float(old["speed"]) / span if old.get("speed") else float(ex._jaw_rate())
    a = 0.0 if F["kp"] is None else float(np.max(F["friction"] / F["kp"] / F["span"]))
    b = float(ex.dt) + (0.0 if F["tau_s"] is None else float(np.max(F["tau_s"])))
    spread = R.REL * a
    band = max(a, R.REL * R.REL * R.REL)     # until measured: the stiction band, never under REL cubed of the span
    closed = float(old["closed_gap"]) / span if old.get("closed_gap") is not None else a
    return {"speed": speed, "speed_open": speed, "rates": [], "rate_tell": R.TWO_READ * spread / b,
            "lag": [a, b], "lag_spread": spread, "lag_rest": a, "closed_gap": closed, "open_rest": 1.0 - a,
            "still_band": band, "floor_n": floor_of(F), "z_jaw": z_model(F, b, ex.dt), "z_jaw_source": "model",
            "gap_per_span_m": F["gap_per_span_m"], "source": "model"}


def numbers(ex):
    """The jaws' numbers in force (C7 keys, plus source: self_test or model); None for a robot with no jaws. A
    stop slope that is the body's model is worked here from the body's description and the lag in force (the
    stored entry keeps what was measured; the model is not a measurement)."""
    if facts(ex) is None:
        return None
    know = getattr(getattr(ex, "bounds", None), "know", None)
    try:
        entry = _val(know.get("jaw")) if know is not None else None
    except Exception:        # a self-knowledge with no such entry
        entry = None
    key = (id(know), id(entry))
    cache = ex.__dict__.get("_jaw_numbers")
    if cache is not None and cache[0] == key:
        return cache[1]
    if isinstance(entry, dict) and entry.get("rate_tell") is not None and entry.get("lag") is not None:
        N = dict(entry)
        N["source"] = "self_test"
        N.setdefault("gap_per_span_m", facts(ex)["gap_per_span_m"])
        if N.get("z_jaw_source", "model") == "model":
            N["z_jaw"], N["z_jaw_source"] = z_model(facts(ex), N["lag"][1], ex.dt), "model"
    else:
        N = model_numbers(ex, entry)
    ex.__dict__["_jaw_numbers"] = (key, N)
    return N


def read(ex, s=None):
    """(opening, lag) of the jaws at reading s (default the last one): the opening as a share of the span (the mean
    over the jaws) and how far the reading stands OPEN of the servo's own target (share of the span, the largest
    jaw; negative: it stands closed of it). (None, None) without a reading."""
    F = facts(ex)
    s = ex._last if s is None else s
    g = getattr(s, "gripper", None) if s is not None else None
    if F is None or not isinstance(g, dict):
        return None, None
    J = F["J"]
    q, target = J.reading(g, ex._gripper_cmd)
    if q is None:
        return None, None
    lag = None if target is None else float(np.max(J.lag(q, target) / F["span"]))
    return float(J.frac_of_q(q)), lag


def presses(ex, s=None):
    """The jaws' own reading at s (default the last one) shows them pressing on something: it stands open of their
    command by more than a free jaw at rest stands from it (its lag at rest with REL, plus TWO_READ spreads of its
    scatter there)."""
    _, lag = read(ex, s)
    N = numbers(ex)
    if lag is None or N is None:
        return False
    rest = max(float(N["lag"][0]), float(N.get("lag_rest") or 0.0))
    return lag > rest * (1.0 + R.REL) + R.TWO_READ * spread_at(N, 0.0)


def scene_touch(ex):
    """Contract C10: tell the motion core that the jaws are commanded to move in this control period (the last act
    that changed the scene, the design rulings: a look taken before it no longer shows the scene as it stands). The core keeps
    the time (ex._scene_t); a core that has no such hook is told nothing."""
    touch = getattr(ex, "_scene_touch", None)
    if callable(touch):
        touch()


def _run(ex, q_hold, cmd):
    ex._gripper_cmd = float(min(1.0, max(0.0, cmd)))
    s = ex._period(q_hold)
    frac, lag = read(ex, s)
    return s, frac, (0.0 if lag is None else lag)


def _rest(ex, q_hold, cmd, n):
    """Hold the command until the reading stands: its range over n readings is under REL cubed of the largest
    range it showed over n readings since the command was set (or under what the arithmetic resolves). Returns the
    last n readings as (opening, lag)."""
    rows, most, fine = [], 0.0, float(np.sqrt(np.finfo(float).eps))
    for _ in range(n * int(1 / R.REL) ** int(R.QUANT)):
        _, frac, lag = _run(ex, q_hold, cmd)
        rows.append((frac, lag))
        if len(rows) >= n:
            now = float(np.ptp([r[0] for r in rows[-n:]]))
            if len(rows) >= n + n and now <= max(R.REL * R.REL * R.REL * most, fine):
                break
            most = max(most, now)
    return rows[-n:]


def _steady(ex, q_hold, f0, f1, rate, n=None):
    """Command the opening from f0 toward f1 at `rate` (spans per s), one step a period. With n: stop once the lag
    behind the command has stood steady for n readings (its change over QUANT readings under REL of itself);
    without: run to f1. Steady: the lag's change over QUANT readings is under REL x REL of itself. Returns (the
    reading's own rate over the steady part, that part's lags, the arm's largest force change over the stroke)."""
    dt, sign, q = float(ex.dt), (1.0 if f1 >= f0 else -1.0), int(R.QUANT)
    F0 = np.asarray(ex._sense().F, dtype=float).copy()
    cmd, rows, arm, since = float(f0), [], 0.0, None
    while sign * (f1 - cmd) > 0.0 and rate > 0.0:
        cmd = cmd + sign * min(rate * dt, sign * (f1 - cmd))
        s, frac, lag = _run(ex, q_hold, cmd)
        rows.append((frac, -sign * lag))          # how far the reading stands behind the command, either way
        arm = max(arm, float(np.linalg.norm(np.asarray(s.F, dtype=float) - F0)))
        if n is None:
            continue
        if since is None and len(rows) > q and \
                abs(rows[-1][1] - rows[-1 - q][1]) <= R.REL * R.REL * abs(rows[-1][1]):
            since = len(rows) - 1
        if since is not None and len(rows) - since >= n:
            break
    part = rows[since:] if since is not None else rows[len(rows) // q:]
    got = abs(part[-1][0] - part[0][0]) / ((len(part) - 1) * dt) if len(part) > 1 else 0.0
    return got, [r[1] for r in part], arm


def measure_jaws(ex):
    """C7: the jaws' own numbers from steady rates in free air (the hand stands where it is; nothing between the
    jaws). The jaws' own rate is the fastest the reading follows a command (the command's rate doubled from a slow
    stroke until the reading falls behind it, or a stroke gets too short to tell); the lag line a + b x rate is
    fitted over that rate and its halves; rate_tell is the slowest of those rates at which the moving jaw's lag
    still stood TWO_READ spreads over its lag at rest (the halving ends at the first that does not). Every end (open, closed) is come to at a slow steady rate, never by a
    step. Leaves the jaws at their rest opening less two still bands. None with no jaws or no jaw reading."""
    F = facts(ex)
    s = ex._sense()
    if F is None or read(ex, s)[0] is None:
        return None
    q_hold, dt, t0 = s.q.copy(), float(ex.dt), sum(ex.periods.values())
    n = max(int(R.QUANT), int(round(R.DOUBLE / R.REL)))      # readings a steady stretch or a rest is judged on
    was, ex._period_kind = ex._period_kind, "jaw"
    try:
        slow = R.REL / (n * dt)                              # a whole span in n / REL readings: every end is
        _steady(ex, q_hold, read(ex, s)[0], 1.0, slow)       # come to at this rate, never by a step
        rest = _rest(ex, q_hold, 1.0, n)                     # told to open: where the free jaws rest
        open_rest = float(np.mean([r[0] for r in rest]))
        _steady(ex, q_hold, open_rest, 0.0, slow)
        shut = _rest(ex, q_hold, 0.0, n)                     # where the jaws meet: its own number
        closed = float(np.mean([r[0] for r in shut]))
        _steady(ex, q_hold, closed, 1.0, slow)
        again = _rest(ex, q_hold, 1.0, n)
        lag_rest, spread_rest = float(np.mean([abs(r[1]) for r in again])), float(np.std([r[1] for r in again]))
        # the still band: what a free jaw at rest stands within (its reading's own range at rest, how far two
        # rests at the same command differ, and how far from its command its own friction leaves it)
        still = max(float(np.ptp([r[0] for r in rest])), float(np.ptp([r[0] for r in shut])),
                    abs(open_rest - float(np.mean([r[0] for r in again]))), lag_rest, TINY)
        hi = open_rest - R.QUANT * still
        lo = closed + R.REL * (hi - closed)                  # every stroke stops this far before the closed end

        def stroke(rate):
            got, lags, arm = _steady(ex, q_hold, hi, lo, rate, n)
            back = _steady(ex, q_hold, read(ex)[0], hi, max(rate, own), None)
            _rest(ex, q_hold, hi, n)
            return got, lags, arm, back[0]
        own, arm_free, rows, opened = 0.0, 0.0, [], 0.0
        rate = (hi - lo) * R.REL / (n * dt)                  # a stroke of n / REL readings; doubled from there
        while True:                                          # the jaws' own rate
            got, lags, arm, back = stroke(rate)
            arm_free, opened = max(arm_free, arm), max(opened, back)
            own = max(own, got)
            if got < (1.0 - R.REL) * rate:
                break                                        # it fell behind its command: what it ran is its own
            if R.DOUBLE * rate * dt * R.QUANT > hi - lo:
                break                                        # a faster stroke has under QUANT readings: this far
            rate *= R.DOUBLE                                 # it follows
        if not own > 0.0:
            return None                                      # the jaws did not move: nothing was measured
        rate, tell = own, own
        for _ in range(int(R.DOUBLE / R.REL) + 1):           # the lag at that rate and at its halves, down to the
            got, lags, arm, back = stroke(rate)              # rate whose lag no longer stands over the lag at rest
            arm_free, opened = max(arm_free, arm), max(opened, back)
            if lags and got > 0.0:
                mean, std = float(np.mean(lags)), float(np.std(lags))
                if mean < lag_rest + R.TWO_READ * max(std, spread_rest):
                    break
                rows.append((got, mean, std))
                tell = min(tell, got)
            rate *= R.GIVE
    finally:
        ex._period_kind = was
    x, y = np.asarray([r[0] for r in rows]), np.asarray([r[1] for r in rows])
    if len(rows) >= int(R.QUANT) and float(np.ptp(x)) > 0.0:
        b, a = (float(v) for v in np.polyfit(x, y, 1))
    else:
        a, b = lag_rest, (float(y[0] - lag_rest) / float(x[0]) if len(rows) else dt)
    a, b = max(a, 0.0), max(b, TINY)
    spread = max([r[-1] for r in rows] + [spread_rest])      # the lag's own scatter at a steady rate and at rest
    return {"speed": own, "speed_open": opened or own, "rates": [float(v) for v in sorted(x)],
            "rate_tell": tell, "lag": [a, b], "lag_spread": spread,
            "lag_spreads": [[float(r[0]), float(r[-1])] for r in sorted(rows)],
            "lag_rest": lag_rest, "lag_rest_spread": spread_rest, "closed_gap": closed, "open_rest": open_rest,
            "still_band": still,
            "floor_n": floor_of(F, a), "z_jaw": z_model(F, b, dt), "z_jaw_source": "model", "arm_free_n": arm_free,
            "gap_per_span_m": F["gap_per_span_m"], "span": [float(v) for v in F["span"]],
            "periods": sum(ex.periods.values()) - t0,
            "unit": "openings, lags and bands: shares of the jaw span; rates: spans per s; z_jaw: N per (m/s) of "
                    "the gap, from the body's model (these jaws read no force): the jaw's inertia on the contact "
                    "model's rate plus the servo's gain on the lead at the touch; rate_tell: the slowest measured "
                    "rate whose lag stood TWO_READ spreads over the lag at rest"}


def spread_at(N, rate):
    """The scatter of the free jaw's lag at the measured steady rate nearest `rate`; at rest its scatter at rest;
    the one stored spread where no rate was measured."""
    rows = N.get("lag_spreads") or []
    if not rate > 0.0:
        return float(N.get("lag_rest_spread", N["lag_spread"]))
    if not rows:
        return float(N["lag_spread"])
    return float(min(rows, key=lambda r: abs(np.log(float(r[0]) / rate)))[1])
