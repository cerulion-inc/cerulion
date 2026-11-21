# SPDX-License-Identifier: AGPL-3.0-only
"""Derived pull and turn bounds, in SI, and the motion compatibility adapter."""

import inspect
import math
from functools import lru_cache
import copy
import dataclasses

import numpy as np


def of(ex):
    from rcpx import bounds
    return bounds.of(ex)


def ratios():
    from rcpx import bounds_ratios
    return bounds_ratios


def item(what, value, unit, evidence, source="model", lift=None):
    return dict(what=what, value=float(value), unit=unit, over_mm=None,
                source=source, evidence=evidence, lift=lift)


def grip(B, held):
    answer = B.grip(held)
    if isinstance(answer, dict):
        return answer.get("capacity_n"), answer.get("accel_ok"), answer.get("items", [])
    return answer


def band(B, part):
    return ratios().BAND * float(part.sigma_p or 0)


def depth_m(B, part, finger_length):
    depth = band(B, part)
    return depth if finger_length is None else min(depth, finger_length)


def opening_m(B, part, jaws, width_m):
    if width_m is None:
        return jaws.max_m
    clear = B.clearance(0, part)
    return min(jaws.max_m, width_m + clear + clear)


def jaw_step_m(B, jaws):
    quantum = np.asarray(B.joint_quantum(), float)
    step = float(np.max(quantum)) if quantum.size else 0
    return abs(float(jaws.width_m(jaws.closed + step) - jaws.width_m(jaws.closed)))


def min_gap_m(B, jaws):
    return float(jaws.width_m(jaws.closed)) + ratios().QUANT * jaw_step_m(B, jaws)


def same_spot_m(B, part, other, jaws):
    return float(part.sigma_p or 0) + float(other.sigma_p or 0) + jaws.max_m


def support(B, floor_z=None):
    plane = B.space.support()
    if plane is None and floor_z is not None:
        plane = (((0, 0, 1), floor_z), 0, "stated")
    if plane is None:
        return None, None
    (normal, offset), sigma, source = plane
    thing = B.thing(None)
    changes = dict(sigma_p=float(sigma or 0), point=np.asarray(normal) * offset)
    if dataclasses.is_dataclass(thing):
        thing = dataclasses.replace(thing, **changes)
    elif hasattr(thing, "_replace"):
        thing = thing._replace(**changes)
    else:
        thing = copy.copy(thing)
        for name, value in changes.items():
            setattr(thing, name, value)
    return plane, thing


def margins(B, support_thing):
    return B.clearance(0, support_thing, body=None), B.clearance(0, None, body=None)


def via_points(B, p_now, p_pre, radius, part, q, down, support_plane):
    route = B.space.route(p_now, p_pre, radius)
    if route is not None:
        return [np.asarray(p, float) for p in route], "the space map gave a route"
    ceiling = B.space.ceiling(p_now[:2], p_pre[:2], radius)
    if ceiling is None:
        height = float(part.point[2]) if part.point is not None else float(p_pre[2])
        if support_plane is not None:
            (normal, offset), _, _ = support_plane
            if normal[2] != 0:
                height = max(height, (offset - np.asarray(normal[:2]) @ p_pre[:2]) / normal[2])
        ceiling = height + band(B, part) + B.clearance(B.v_free(q, down), part)
        text = "the space map gave no route; height from the part, its uncertainty and path clearance"
    else:
        text = "the space map gave a free ceiling"
    via = np.asarray(p_pre, float).copy()
    via[2] = max(float(p_pre[2]), float(ceiling))
    return [via, np.asarray(p_pre, float)], text


def seat_m(B, part, q, d):
    return max(band(B, part), ratios().QUANT * B.rigid(q, d))


def follow_speed(B, part, q, d, f_now, c_end, remaining_m, intent=None):
    free = B.v_free(q, d)
    if remaining_m is not None:
        arrival = B.v_in(part, q, d, intent)[0]
        # v^2 = v_in^2 + 2 a s: the 2 is the kinematics of a constant deceleration (a fact)
        speed = min(free, math.sqrt(arrival ** 2 + 2 * B.a_brake(q, d) * max(0, remaining_m - band(B, part))))
    else:
        slope, _ = B.slope(q, d, part)
        speed = min(free, max(0, c_end - f_now) / slope) if slope is not None and slope > 0 else B.v_in(part, q, d, intent)[0]
    if (intent or {}).get("speed") is not None:
        speed = min(speed, intent["speed"])
    return max(0, speed)


def follow_item(B, speed, remaining_m):
    evidence = ("free travel braked into the stated travel or stored end" if remaining_m is not None
                else "end ceiling less present force over the resolved stop slope; unresolved uses arrival speed")
    return item("speed", speed, "m/s", evidence)


def lead_cap(c_f, k):
    return max(0, c_f) / k


def worked_cap(B, f_moved, c_end):
    return min(c_end, max(0, f_moved) * (1 + ratios().REL))


def reached_n(B, c_end, v=0):
    sigma = B.sigma_f(v)
    step = ratios().TWO_READ * sigma if sigma > 0 else ratios().REL ** 2 * c_end
    return max(0, c_end - step)


def still_m(B, q, d, expected_m):
    return max(ratios().QUANT * B.quantum(q, d), ratios().REL * expected_m)


def follower(B, q, d, v, side):
    return B.k_servo(q, side), B.f_thr(v, 0), ratios().QUANT * B.rigid(q, d)


def periods(B):
    return math.ceil(B.t_det(readings=B.guard("follow").readings) / B.dt())


def settle_periods(B):
    return math.ceil(B.t_settle() / B.dt())


def tracking_m(B, q, d, v, cap_m):
    return cap_m + B.lag(v) + B.rigid(q, d)


def turned(first, newest, rel):
    a, b = np.asarray(first, float), np.asarray(newest, float)
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na == 0 or nb == 0:
        return False
    return bool(np.linalg.norm(b - (b @ (a / na)) * (a / na)) > rel * nb)


def joint_step(B):
    return np.asarray(B.qd_max()) * B.dt()


def follow_limit_s(B, planned_m, free_speed, fs):
    rate = fs.rate_n_s(fs.first_n)
    ramp = max(0, fs.ceiling_n - fs.first_n) / rate if rate > 0 else 0
    return B.limit_s(planned_m / free_speed + B.t_settle() + ramp)


def reach_left_m(B, p, d, reach_m):
    p, d = np.asarray(p), np.asarray(d)
    along = float(p @ d)
    return max(0, -along + math.sqrt(max(0, along ** 2 + reach_m ** 2 - float(p @ p))))


def turn_window_s(B, window_m, speed):
    return (1 + ratios().REL) / ratios().REL * window_m / speed if speed > 0 else B.t_settle()


def last_joint_drift(off_axis_m, axis_dot, angle_rad):
    # The tip traces a chord about the last joint. The sign fixes the turn sense.
    signed = angle_rad * (1 if axis_dot >= 0 else -1)
    # the chord of a circle of radius r over an angle a is r x sqrt(2 (1 - cos a)): the 2 is geometry (a fact)
    return abs(off_axis_m) * math.sqrt(max(0, 2 * (1 - math.cos(signed))))


def turn_sample_rad(B, radius_m, floor_margin_m, asked_rad):
    spacing = B.spacing(floor_margin_m)
    if radius_m <= 0:
        return abs(asked_rad)
    if spacing <= 0:
        # Exact hull geometry has no margin to lose; use its relative chord resolution.
        spacing = ratios().REL * radius_m
    # the angle whose chord on a circle of that radius is the spacing: 2 asin(s / 2r) (geometry, a fact)
    return 2 * math.asin(min(1, spacing / (2 * radius_m)))


def rotational_k(B, weights, kp):
    compliance = float(np.sum(np.asarray(weights) ** 2 / np.asarray(kp)))
    return 1 / max(compliance, np.finfo(float).eps)


def angle_gain(B, weights):
    return float(np.sum(np.abs(weights)))


def torque_ceiling(B, tau_plan, weights, last_joint):
    room = np.maximum(0, np.asarray(B.tau_max()) - np.abs(tau_plan))
    if last_joint:
        return float(room[-1])
    w = np.abs(weights)
    active = w > 0
    return float(np.min(room[active] / w[active])) if np.any(active) else 0


def torque_threshold(B, weights, last_joint, speed=0):
    thr = np.asarray(B.tau_thr(speed))
    if last_joint:
        return float(thr[-1])
    w = np.asarray(weights)
    den = float(w @ w)
    return float(np.abs(w) @ thr / den) if den > 0 else 0


def torque_rate(B, torque, ceiling, sigma):
    step = max(ratios().JOINT * sigma, ratios().REL * torque, ratios().REL ** 2 * ceiling)
    return step / B.t_det(readings=B.guard("follow").readings)


def rotation_resolution(B, weights, expected_rad):
    return max(ratios().QUANT * float(np.max(B.joint_quantum())) * angle_gain(B, weights),
               ratios().REL * abs(expected_rad))


def angle_tolerance(B, asked_rad, gain=1):
    return max(ratios().QUANT * float(np.max(B.joint_quantum())) * gain,
               ratios().REL ** 2 * abs(asked_rad))


def joint_still(B, commanded_speed):
    return np.maximum(ratios().QUANT * np.asarray(B.joint_quantum()) / B.dt(),
                      ratios().REL * np.abs(commanded_speed))


@lru_cache(None)
def _modern(cls):
    return "kind" in inspect.signature(cls._cart_move).parameters


def _count(result):
    if "periods" not in result:
        result["periods"] = max(0, len(result.get("done", [])) - 1)
    return result


def _speed(B, q, u, kind, target, intent, held, inside):
    if kind in ("approach", "touch", "press", "lower", "close") and inside:
        speed, bound = B.v_in(B.thing(target), q, u, intent, (held or {}).get("weight_n", 0))
    else:
        speed = B.v_free(q, u)
        bound = item("speed", speed, "m/s", "the joints' own free speed")
    if (intent or {}).get("speed") is not None:
        speed = min(speed, intent["speed"])
        bound = item("speed", speed, "m/s", "the speed this call states", "intent")
    return speed, bound


def line(ex, dp, drot=None, *, kind="transit", target=None, intent=None, away_from=None, held=None):
    if _modern(type(ex)):
        return _count(ex._cart_move(ex._sense(), dp, drot, None, True, True, kind=kind, target=target,
                                   intent=intent, away_from=away_from, held=held))
    B, s = of(ex), ex._sense()
    dp = np.asarray(dp, float)
    length = float(np.linalg.norm(dp))
    u = dp / length if length else s.R[:, 0]
    part = B.thing(target) if target is not None else None
    inside, split = False, None
    if part is not None and part.point is not None and kind in ("approach", "touch", "press", "lower", "close"):
        r = s.tip - np.asarray(part.point)
        width = band(B, part)
        inside = np.linalg.norm(r) <= width
        # Entry into the target's uncertainty sphere, solved without sampling a path.
        projection = float(r @ u)
        disc = projection ** 2 - float(r @ r) + width ** 2
        if not inside and disc >= 0:
            entry = -projection - math.sqrt(disc)
            if 0 < entry < length:
                split = entry / length
    accumulated = 0
    if split is not None:
        first = line(ex, dp * split, None if drot is None else np.asarray(drot) * split,
                     kind="transit", target=target, intent=intent, away_from=away_from, held=held)
        if first.get("stop", first.get("stopped")) != "goal" or "refused" in first:
            return first
        accumulated = first["periods"]
        dp = dp * (1 - split)
        drot = None if drot is None else np.asarray(drot) * (1 - split)
        s, inside = ex._sense(), True
    speed, bound = _speed(B, s.q, u, kind, target, intent, held, inside)
    spec = B.guard(kind, speed, held=held, away_from=away_from)
    result = _count(ex._cart_move(s, dp, drot, speed, True, True, spec.force_n, limit=None))
    result["periods"] += accumulated
    limited = result.get("speed", {}).get("speed_limited") or result.get("speed_limited")
    if isinstance(limited, dict) and limited.get("factor", 1) > 1:
        bound = dict(bound, value=bound["value"] / limited["factor"],
                     evidence=bound["evidence"] + "; legacy core retiming: " + limited.get("why", "reported slower path"))
    return B.report(result, [bound])


def joints(ex, q, *, kind="transit", target=None, intent=None, away_from=None, held=None,
           dark_ok=False, record_contact=True):
    if _modern(type(ex)):
        return _count(ex._joint_move(q, None, kind=kind, target=target, intent=intent, away_from=away_from,
                                    held=held, dark_ok=dark_ok, record_contact=record_contact))
    B, s = of(ex), ex._sense()
    u = ex.model.fk_tip(q)[0] - s.tip if hasattr(ex, "model") else s.R[:, 0]
    size = float(np.linalg.norm(u))
    u = u / size if size > 0 else s.R[:, 0]
    result = _count(ex._joint_move(q, 1, dark_ok=dark_ok, record_contact=record_contact, limit=None))
    return B.report(result, [item("speed", B.v_free(s.q, u), "m/s", "the joints' own free speed; legacy joint helper uses its full share")])


def stream(ex, legs, *, held=None, looks=None, need_m=None):
    if _modern(type(ex)):
        return ex._stream(legs, held=held, looks=looks, need_m=need_m)
    counts, moving, waiting, bound = [], 0, 0, None
    stopped, last = "goal", -1
    for last, leg in enumerate(legs):
        goal = np.asarray(leg["to"], float)
        kw = {k: leg.get(k) for k in ("kind", "target", "intent", "away_from")}
        kw["kind"] = kw["kind"] or "transit"
        if goal.ndim == 1:
            result = joints(ex, goal, held=held, **kw)
        else:
            from rcpx.model import _rotvec
            s = ex._sense()
            result = line(ex, goal[:3, 3] - s.tip, _rotvec(goal[:3, :3] @ s.R.T), held=held, **kw)
        count = result.get("periods", 0)
        counts.append(count)
        moving += result.get("moving", count)
        waiting += result.get("waiting", 0)
        bound = result.get("bounds", bound)
        stopped = result.get("stop", result.get("stopped", "refused"))
        if stopped != "goal" or "refused" in result:
            break
    out = dict(stopped=stopped, leg=last, periods=counts, moving=moving, waiting=waiting,
               looks=[], bounds=bound)
    if legs and "refused" in result:
        out["refused"] = result["refused"]
    return out
