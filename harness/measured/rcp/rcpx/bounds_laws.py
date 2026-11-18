# SPDX-License-Identifier: AGPL-3.0-only
"""Pure SI laws for derived motion bounds, A1 and A4.

Round six: no law here lowers a speed or a force for doubt. What a contact may see is a stated limit
(rcpx/bounds_limit.py); what a motion could strike is priced (rcpx/bounds_strike.py).
"""

import math
import numpy as np
from rcpx.bounds_ratios import TWO_READ, ONE_READ, JOINT, BAND, QUANT, REL, DOUBLE

TWO = 2.0  # numeric: kinematics, v^2 = 2 a d and the mean of two values
ROUND = math.sqrt(np.finfo(float).eps)


def f_thr(f_hidden, sigma_f, readings=2):
    """Hidden level plus the reading count's noise multiple, A4.1."""
    return f_hidden + (ONE_READ if readings <= 1 else TWO_READ) * sigma_f


def tau_thr(tau_hidden, sigma_tau):
    """Hidden torque plus JOINT times its noise, A4.1."""
    return np.asarray(tau_hidden) + JOINT * np.asarray(sigma_tau)


def samples_for(rel=REL):
    """Ceil of reciprocal twice squared relative error, A1.1."""
    return int(math.ceil(1 / (TWO * rel ** 2)))


def v_free(qd_max, jpinv_u):
    """Least rated joint speed over its directional motion, A4.2."""
    return min((float(v) / abs(a) for v, a in zip(qd_max, jpinv_u) if a != 0), default=math.inf)


def d_stop(v, t_det, a_brake):
    """Detection travel plus constant-deceleration travel, A4.8."""
    if v <= 0:
        return 0.0
    return math.inf if a_brake is None or not a_brake > 0 else v * t_det + v ** 2 / (TWO * a_brake)


def speed_for_stop(d, t_det, a_brake):
    """Nonnegative inverse of detection and braking travel, A4.3."""
    if d <= 0:
        return 0.0
    if a_brake is None or not a_brake > 0:
        return None
    # Rationalized form avoids cancellation at a short distance.
    return TWO * d / (t_det + math.sqrt(t_det ** 2 + TWO * d / a_brake))


def band(sigma, e_fit=0.0):
    """Position noise band plus the fit error in force, A3."""
    return BAND * sigma + e_fit


def v_toward(c, band_m, v_in, a_dec, v_free):
    """Arrival speed with braking room outside the band, A4.2."""
    return min(v_free, math.sqrt(v_in ** 2 + TWO * a_dec * max(0, c - band_m)))


def _bisect(lo, hi, fits):
    """Solve a monotone bound until floating-point endpoints coincide, A4.2."""
    while True:
        mid = lo + (hi - lo) / TWO
        if mid <= lo or mid >= hi:
            return lo
        if fits(mid):
            lo = mid
        else:
            hi = mid


def v_along(e_cross, c, sigma_e, v_free):
    """Largest speed whose cross-track error fits the gap c to a measured thing, A4.2. The thing's position
    error (sigma_e) takes nothing off the gap: doubt lowers no speed."""
    room = c
    if room < 0 or e_cross(0) > room:
        return None
    hi = v_free
    if math.isinf(hi):
        at_one = e_cross(1.0)
        hi = room / at_one if at_one > 0 and room > 0 else 1.0
        while e_cross(hi) <= room:
            larger = hi * DOUBLE
            if not math.isfinite(larger):
                return math.inf
            hi = larger
    elif e_cross(hi) <= room:
        return hi
    return _bisect(0.0, hi, lambda v: e_cross(v) <= room)


def a_dec(f_thr_of_a, f_allow, a_brake):
    """Largest braking acceleration below the force threshold, A4.2."""
    if a_brake is None or f_thr_of_a(0) >= f_allow:
        return None
    if f_thr_of_a(a_brake) < f_allow:
        return a_brake
    return _bisect(0.0, a_brake, lambda a: f_thr_of_a(a) < f_allow)


def series(k1, k2):
    """Series stiffness with an unknown stiffness treated as rigid, Q9."""
    if k1 is None or math.isinf(k1):
        return math.inf if k2 is None else k2
    if k2 is None or math.isinf(k2):
        return k1
    return k1 * k2 / (k1 + k2) if k1 + k2 > 0 else 0.0


def z_impact(m_eff, b_per_s=None, c=1.0, c2=None):
    """Contact-model or fitted impact slope times effective mass, Q9."""
    if b_per_s is not None:
        return c * b_per_s * m_eff
    return c2 * m_eff if c2 is not None else None


def stop_slope(z_i, k_w, t_det, lag_s):
    """Impact plus servo run-on slope with its relative margin, Q9."""
    if z_i is None:
        return None
    if not math.isfinite(k_w):
        raise ValueError("stop slope needs finite servo stiffness")
    return (z_i + k_w * (t_det + lag_s)) * (1 + REL)


def with_margin(z):
    """Apply the relative margin to a fitted stop slope, Q9."""
    return None if z is None else z * (1 + REL)


def v_in(f_allow, f_thr, z, fastest=None):
    """Solve allowed force minus hidden threshold over stop slope, A4.3."""
    f = f_thr if callable(f_thr) else lambda v: f_thr
    if z is None or not z > 0 or not math.isfinite(z) or f_allow <= f(0):
        return None
    hi = (f_allow - f(0)) / z
    if not math.isfinite(hi):
        return None
    v = _bisect(0.0, hi, lambda v: f(v) + z * v <= f_allow) if callable(f_thr) else hi
    if fastest is not None:
        v = min(v, DOUBLE * fastest)
    return v, f(v) + z * v


def tell_speed(rigid_m, t_det, a_brake):
    """The tell speed: the speed whose run-on (detection plus braking) is the least travel the body can tell,
    A4.3. No motion under a stated limit runs slower: under it a stop is not resolved."""
    return speed_for_stop(rigid_m, t_det, a_brake) if rigid_m > 0 and math.isfinite(rigid_m) else None


def arrival_speed(rigid_m, band_m, t_det, a_brake):   # r6 shim: the older name and signature of tell_speed
    return tell_speed(rigid_m, t_det, a_brake)


def strike_energy(m_eff, v):
    """Kinetic energy of a directional strike in joules, A4.3."""
    return m_eff * v ** 2 / TWO


def peak_floor(m_eff, v, t_sample):
    """Impulse divided by one fast sample is a peak floor, A4.3."""
    return m_eff * v / t_sample if t_sample > 0 else None


def f_hand(hand_kg, g, held_n=0.0):
    """Weight of the hand and any carried load, Q2 (a load not read yet counts as none)."""
    return g * hand_kg + (held_n or 0.0)


def _ramp(sigma_f, e_tau, ceiling):
    """Choose measured or relative force-ramp resolution, A4.5."""
    e = e_tau if e_tau > 0 else REL
    least = TWO_READ * sigma_f if sigma_f > 0 else REL * REL * ceiling
    return e, least


def ramp_rate(f, sigma_f, e_tau, t_det, ceiling):
    """Continuous relative force ramp with a resolvable least step, A4.5."""
    e, least = _ramp(sigma_f, e_tau, ceiling)
    return max(least, e * f) / t_det if t_det > 0 else None


def ramp_time(f0, f1, sigma_f, e_tau, t_det, ceiling):
    """Linear ramp to its knee then the exponential integral, A4.5."""
    if f1 <= f0:
        return 0.0
    e, least = _ramp(sigma_f, e_tau, ceiling)
    knee = least / e
    linear = max(0.0, min(f1, knee) - f0)
    if linear > 0 and not least > 0:
        return None
    t = linear * t_det / least if linear > 0 else 0.0
    lower = max(f0, knee)
    if f1 > lower:
        if not lower > 0:
            return None
        t += t_det / e * math.log(f1 / lower)
    return t


def standoff(d_stop_m, sigma_p, w_tool):
    """Stopping travel plus contact uncertainty and tool width, A4.3."""
    return d_stop_m + QUANT * sigma_p + w_tool


def clearance(e_fit, e_cross_m, sigma_p, spacing_m):
    """Fit, cross-track error, position band and half-spacing, A4.3."""
    return e_fit + e_cross_m + BAND * sigma_p + spacing_m / TWO


def spacing(margin_m):
    """Check a path at most half the smallest active margin, A3."""
    return margin_m / TWO


def reach_past(sigma_p, rigid_m):
    """Position band plus distinguishable rigid travel, A4.3."""
    return BAND * sigma_p + rigid_m


def same_spot(sigma_a, sigma_b, w_tool):
    """Root summed contact variances plus tool half width, A4.3."""
    return math.sqrt(sigma_a ** 2 + sigma_b ** 2) + w_tool


def rigid(quantum, jitter, force_n, k_servo):
    """Resolvable travel plus elastic servo give, Q3."""
    give = force_n / k_servo if force_n and k_servo is not None and math.isfinite(k_servo) and k_servo > 0 else 0.0
    return QUANT * quantum + jitter + give


def contact_error(v, t_det, quantum):
    """Contact position error from detection travel and quantum, A4.3."""
    return v * t_det + quantum


def limit_s(planned_s):
    """Planned duration enlarged by the relative margin, A4.8."""
    return planned_s * (1 + REL)


def quiet(force_n, f_thr0, left_m, need_m):
    """Quiet means force and motion left fit the next phase, A4.8."""
    return need_m is None or (force_n <= f_thr0 and left_m <= need_m)


def no_progress(travel_m, expected_m, quantum):
    """Progress below quanta or expected relative travel is absent, A4.8."""
    return travel_m < max(QUANT * quantum, REL * expected_m)


def load_n(mass_kg, g, accel, cos_up=1.0):
    """Magnitude of gravity plus directional acceleration load, A4.7."""
    return mass_kg * math.sqrt(max(0, g ** 2 + accel ** 2 + TWO * g * accel * cos_up))


def grip_capacity(proofs, squeeze_now):
    """Largest demonstrated load scaled by current squeeze, A4.7."""
    return max((load * squeeze_now / squeeze for load, squeeze in proofs if squeeze > 0), default=None)


def accel_ok(capacity_n, mass_kg, g, cos_up=0.0):
    """Largest directional acceleration carried by the proved grip, A4.7."""
    if capacity_n is None:
        return None
    if capacity_n < mass_kg * g:
        return 0.0
    if not mass_kg > 0:
        return None
    return max(0, -g * cos_up + math.sqrt(max(0, (capacity_n / mass_kg) ** 2 - g ** 2 * (1 - cos_up ** 2))))


def g_max(mass_kg, g, accel, squeeze_now, squeeze_slipped, cos_up=1.0):
    """Slip load scaled by squeeze is the set-down fuse, A4.6."""
    return None if squeeze_slipped is None or not squeeze_slipped > 0 else load_n(mass_kg, g, accel, cos_up) * squeeze_now / squeeze_slipped


def setdown_speed(slide_room_m, t_det, a_brake, f_allow, w_obj_n, g_max_n, f_thr, z):
    """Slide stopping travel and either fuse or surface bound, A4.6."""
    v1 = speed_for_stop(slide_room_m, t_det, a_brake)
    if v1 is None:
        return None, "unresolved"
    if g_max_n is not None and w_obj_n + g_max_n <= f_allow:
        return v1, "fuse"
    r = v_in(f_allow, f_thr, z)
    return (min(v1, r[0]), "surface") if r is not None else (None, "unresolved")


def lead_cap(f_moved, k_servo):
    """Maximum command lead from the force a part needed, A4.5."""
    return f_moved * (1 + REL) / k_servo if k_servo is not None and k_servo > 0 else None


def follow_speed(c_end, f_now, z, v_free, remaining_m=None, a_dec=None):
    """Free speed braking into stated travel or an unknown end, A4.5."""
    if remaining_m is not None:
        return (min(v_free, math.sqrt(TWO * a_dec * max(0, remaining_m))), "stated") if a_dec is not None else (None, "unresolved")
    return (min(v_free, max(0, (c_end - f_now) / z)), "end") if z is not None and z > 0 else (None, "unresolved")


def load_speed(f_target, f_now, k_w, t_det):
    """Force increment over stiffness and detection time, A4.5."""
    return (f_target - f_now) / (k_w * t_det) if k_w > 0 and t_det > 0 else None


def strike_speed(f_allow, aim_n, z):
    """Allowed force less aim divided by stop slope, A4.5."""
    return (f_allow - aim_n) / z if z is not None and z > 0 else None


def false_stop(impulse_ns, sigma_f, t_det, rest_n, f_thr0, stop_impulse_ns=None):
    """Small impulse gone at rest is false, A1.1. Small is inside the noise or, in the relative form a noise-free
    reading needs, under REL of the impulse that stopping the moving mass would give (m_eff x v, Q8)."""
    small = impulse_ns < TWO_READ * sigma_f * t_det
    if stop_impulse_ns is not None and math.isfinite(stop_impulse_ns):
        small = small or impulse_ns < REL * stop_impulse_ns
    return bool(small and rest_n is not None and rest_n <= f_thr0)


def m_eff(a, m_inv):
    """Reciprocal directional joint mobility is effective mass, Q8."""
    den = float(np.asarray(a) @ m_inv @ np.asarray(a))
    return 1 / den if den > 0 else math.inf


def k_servo(a, kp):
    """Reciprocal directional joint compliance is servo stiffness, Q8."""
    den = sum(aj ** 2 / kj for aj, kj in zip(a, kp) if kj > 0)
    return 1 / den if den > 0 else math.inf


def a_brake_model(tau_max, g_tau, m_jpinv_u):
    """Torque left after gravity over directional joint inertia, Q7."""
    return min((max(0, t - abs(g)) / abs(a) for t, g, a in zip(tau_max, g_tau, m_jpinv_u) if a != 0), default=math.inf)


def qdd_model(tau_max, g_tau, m_diag):
    """Per-joint gravity-reserved torque over diagonal inertia, Q1."""
    margin = np.maximum(0, np.asarray(tau_max) - np.abs(g_tau))
    return np.divide(margin, m_diag, out=np.zeros_like(margin, dtype=float), where=np.asarray(m_diag) > 0)


def qd_until_known(qdd, joint_range):
    """Speed reached over the joint range at model acceleration, Q1."""
    return np.sqrt(TWO * np.asarray(qdd) * np.asarray(joint_range))


def f_allow(thing_side, f_cap, e_tau, f_tool, site=None):
    """Minimum thing, derated capability, tool and site force, A4.3."""
    terms = ((thing_side, "thing"), (None if f_cap is None else f_cap * (1 - e_tau), "cap"), (f_tool, "tool"), (site, "site"))
    return min((p for p in terms if p[0] is not None), key=lambda p: p[0], default=(math.inf, "none"))


def gentle(f_hand_n):
    """The meaning of the word gentle: no more than the hand's own weight, A4.3."""
    return f_hand_n, f"your word gentle: the hand's own weight ({f_hand_n:g} N)"
