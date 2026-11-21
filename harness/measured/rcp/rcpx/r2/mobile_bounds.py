# SPDX-License-Identifier: AGPL-3.0-only
"""Derived base bounds. Every input and returned value is SI."""
import math


def of(ex):
    from rcpx import bounds
    return bounds.of(ex)


def ratios():
    from rcpx import bounds_ratios
    return bounds_ratios


def item(what, value, unit, source, evidence, lift=None):
    return dict(what=what, value=float(value), unit=unit, over_mm=None, source=source,
                evidence=evidence, lift=lift)


def empty_know():
    # settled_s: the longest time the base took from its last waypoint to still (a measured row, not a setting)
    return dict.fromkeys(('quantum_m', 'quantum_rad', 'run_on_s', 'lag_s', 'settled_s',
                          'f_free', 'sigma_free', 'drift'))


def base_limits(info):
    torso = info.get('torso')
    return float(info['max_mps']), float(info['max_turn_rps']), float(torso['max_mps']) if torso else 0


def swept_speed(info, distance_m, turn_rad, corner_radius_m):
    v,w,unused = base_limits(info)
    duration = max(distance_m/v if distance_m else 0,abs(turn_rad)/w if turn_rad else 0)
    return (distance_m+abs(turn_rad)*corner_radius_m)/duration if duration else 0


def run_on(B, know):
    value = know.get('run_on_s')
    return B.t_settle() if value is None else value


def settle_time(B, know):
    value = know.get('settled_s')
    return B.t_settle() if value is None else value


def stop_time(B, know):
    # A pair of readings is a detection fact, not a period budget.
    return B.t_det(2) + run_on(B, know)


def stop_distance(B, know, v):
    return abs(v) * stop_time(B, know)


def clearance(B, know, v, sigma_p, est_err, stated=None):
    value = stop_distance(B, know, v) + sigma_p + est_err
    return value if stated is None else max(value, stated)


def speed_at(dist, sigma_p, est_err, t_stop_s, v_max):
    if t_stop_s <= 0:
        return v_max if dist > sigma_p + est_err else 0
    return min(v_max, max(0, dist - sigma_p - est_err) / t_stop_s)


def est_error(know, moved_m, turn_terms_m):
    drift = know.get('drift')
    value = (drift or 0) * moved_m + turn_terms_m + ratios().QUANT * (know.get('quantum_m') or 0)
    text = 'odometry drift not measured yet' if drift is None else 'the largest measured odometry drift'
    return value, text


def stall_index(planned_step_m, measured_steps_m, window_n, quantum_m):
    if window_n <= 0:
        raise ValueError('stall window must contain a reading')
    if isinstance(planned_step_m, (int, float)):
        plans = [abs(planned_step_m)] * len(measured_steps_m)
    else:
        plans = [abs(v) for v in planned_step_m]
    for i in range(window_n - 1, len(measured_steps_m)):
        start = i + 1 - window_n
        planned = sum(plans[start:i+1])
        measured = sum(abs(v) for v in measured_steps_m[start:i+1])
        threshold = max(ratios().QUANT * quantum_m, ratios().REL * planned)
        if planned > 0 and measured < threshold:
            return i
    return None


def stall_window(B, know):
    return max(1, math.ceil(settle_time(B, know) / B.dt()))


def settled(error_m, left_m, quantum_m, planned_step_m):
    if quantum_m > 0:
        return abs(error_m) <= ratios().QUANT * quantum_m
    return abs(left_m) < ratios().REL * abs(planned_step_m) or error_m == left_m == 0


def contact_rise(B, know, v, a_meas, g):
    hidden = max(know.get('f_free') or 0, B.f_hidden(v, a_meas))
    sigma = max(know.get('sigma_free') or 0, B.sigma_f(v, a_meas))
    inertial = (B.f_hand() / g) * abs(a_meas) if g > 0 else 0
    return hidden + ratios().TWO_READ * sigma + inertial


def sweep_step(B, margin_m, corner_radius_m):
    step = B.spacing(margin_m)
    turn = step / corner_radius_m if corner_radius_m > 0 else math.tau
    return step, turn


def top_margin(sigma_p, est_err):
    return ratios().BAND * sigma_p + est_err


def comparison_slack(know):
    return ratios().QUANT * (know.get('quantum_m') or 0)


def quantum(B, know, angular=False):
    return know.get('quantum_rad' if angular else 'quantum_m') or 0


def speed_item(B, know, v, rated, point=None, intent=None):
    if point is None:
        source, text = 'rating', 'the base controller\'s own rated speed'
    else:
        source = 'contact' if point.get('kind') == 'contact' else 'seen'
        fallback = 'the arm settling time until base run-on is measured' if know.get('run_on_s') is None else 'measured base run-on'
        text = f"stopping distance reaches stored point {point['id']} ({point['label']}); {fallback}"
    clear = (intent or {}).get('clear')
    if clear is not None:
        look = clear.get('look') if isinstance(clear,dict) else (intent or {}).get('look') or 'the newest look'
        text += f'; the operator judged the way clear after look {look}; stored point clearance remains in force'
    return item('speed', v, 'm/s', source, text, 'drive around the stored point' if point else None)


def time_item(B, planned):
    return item('time', B.limit_s(planned), 's', 'model', 'planned travel time and derived settling time')


def available(B, know, v, g):
    try:
        values = [B.dt(),stop_time(B,know),settle_time(B,know),contact_rise(B,know,v,0,g)]
        if not all(isinstance(value,(int,float)) and math.isfinite(value) for value in values):
            raise ValueError('a scalar base bound is unavailable')
    except Exception as exc:
        return {'said':'Refused: base stopping bounds are unavailable.',
                'refused':{'check':'bounds_unavailable','why':f'base motion needs scalar bounds: {exc}','hint':'base_pose()'}}
    return None


def end_word(B):
    fn = getattr(B.space,'end_call',None)
    if callable(fn):
        fn()


def journal(ex, kind, **fields):
    return ex._journal(kind, **fields)
