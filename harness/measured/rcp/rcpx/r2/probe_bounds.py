# SPDX-License-Identifier: AGPL-3.0-only
"""Probe force, travel and observation bounds, in SI units."""
import math
from rcpx.bounds_ratios import QUANT, TWO_READ


def of(ex):
    from rcpx import bounds
    return bounds.of(ex)


def field(thing, name, default=0):
    value = thing.get(name, default) if isinstance(thing, dict) else getattr(thing, name, default)
    return default if value is None else value


def path_noise_m(B, q, u, jitter_m=0):
    return max(B.rigid(q, u, 0), jitter_m)


def step_m(B, q, u, planned_m=0):
    measured = QUANT * B.quantum(q, u)
    from rcpx.r2.probe_fit import SETTLE_FULL
    return max(measured, abs(planned_m) / SETTLE_FULL)


def measurable_travel_m(B, q, u, reach_m):
    """The least travel that tells a slide from a hinge on this robot (m): the chord over which an arc of the
    largest radius that still counts as a hinge (the reach) departs from a line by the least travel the robot can
    tell from none, B.rigid at zero force (QUANT x quantum + the still hold's jitter). Zero when neither a
    resolution nor a jitter is known: the caller then asks the operator for the travel."""
    sagitta = B.rigid(q, u, 0)
    # Exact chord travel for a circle of the largest radius that counts as a hinge.
    return 2.0 * math.sqrt(max(0, 2.0 * reach_m * sagitta - sagitta**2))


def travel_item(B, q, u, reach_m):
    return {"what": "travel", "value": measurable_travel_m(B, q, u, reach_m), "unit": "m",
            "over_mm": None, "source": "measurable",
            "evidence": "an arc at the model reach departs from a line by the least travel this robot can tell from none",
            "lift": "state max_mm"}


def default_force_n(B, thing, entry=None):
    data = (entry or {}).get("data") or {}
    value = data.get("force_n", data.get("contact_force_n"))
    if value is not None:
        return float(value), "contact", "force of the finding touch"
    value = field(thing, "f_bear", None)
    return (value, "contact", "force at which the finding touch saw the part resist")


def seek_m(B, tip, direction, target, thing, q=None):
    """Free travel before the part must have been met (m): to the target along the push, past it by the reach past
    of a touch target (BAND x its sigma_p + rigid: B.travel, the one function for 'reached this point'), plus the
    back-off the finding touch made when it let go (B.travel's backoff: the tool's own width and the travel the part
    showed), because the tip stands that far off the part when the probe starts. No typed distance."""
    import numpy as np
    ahead = float((np.asarray(target) - np.asarray(tip)) @ np.asarray(direction))
    reach = B.travel("touch", thing, q, np.asarray(direction, dtype=float))
    return max(0, ahead) + float(reach.past_m) + float(getattr(reach, "backoff_m", 0) or 0)


def near_m(B, thing, seek_allowed_m):
    return field(thing, "sigma_p") + seek_allowed_m


def under_load_speed(B, q, u, thing, force_target, force_now):
    kc = field(thing, "k_c", None)
    servo = B.k_servo(q, u)
    stiffness = servo if kc is None or not math.isfinite(kc) else (servo * kc / (servo + kc) if servo + kc else servo)
    if stiffness <= 0:
        return B.v_free(q, u)
    return min(B.v_free(q, u), max(0, force_target - force_now) / (stiffness * B.t_det()))


def lag_window_s(B, speed):
    return max(B.t_det(), B.lag(speed) / speed) if speed > 0 else max(B.t_det(), B.t_settle())


def slip_m(B, thing, advance_m, q=None, u=None):
    """How far across the asked direction the tip may be carried before the push counts as slipped off (m): the
    part's own extent where the entry has one, QUANT x the advance made, and never less than the tool's own width
    (A4.5: off axis, a stroke ends when the tip leaves its line by w_tool; B.travel's back-off carries that width)."""
    import numpy as np
    extent = field(thing, "extent")
    if isinstance(extent, dict):
        extent = extent.get("size_m", extent.get("points_m", 0))
    if np.ndim(extent):
        extent = float(np.max(np.linalg.norm(np.asarray(extent), axis=-1)))
    tool = B.travel("touch", thing, q, None if u is None else np.asarray(u, dtype=float))
    return max(float(extent or 0), QUANT * abs(advance_m), float(getattr(tool, "backoff_m", 0) or 0))


def memory_tolerance_n(B, hidden_n, sigma_n):
    return hidden_n + TWO_READ * sigma_n


def base_due(B, peak_n, give, stored_errors_m, base_kind=None, cap_stop=False):
    if base_kind in ("fixed", "rigid") or (give or {}).get("verdict") == "rigid":
        return False
    per_n = (give or {}).get("mm_per_n")
    if per_n is None:
        return bool(cap_stop)
    if not stored_errors_m:
        return False
    return peak_n * per_n / 1000.0 > min(stored_errors_m)


def target_thing(B, thing, entry, q, u):
    """Retain the contact's measured uncertainty on cores that lack note_stop."""
    if entry is None:
        return thing
    data = entry.get("data") or {}
    sigma = max(field(thing, "sigma_p"), (data.get("error_mm") or 0) / 1000,
                (data.get("sigma_p_mm") or 0) / 1000)
    if entry.get("kind") == "contact":
        sigma = max(sigma, B.quantum(q, u))
    if hasattr(thing, "_replace"):
        return thing._replace(sigma_p=sigma, f_bear=field(thing, "f_bear", data.get("force_n")), touched=entry.get("kind") == "contact" or field(thing, "touched"))
    if isinstance(thing, dict):
        return dict(thing, sigma_p=sigma, f_bear=field(thing, "f_bear", data.get("force_n")), touched=entry.get("kind") == "contact" or field(thing, "touched"))
    from dataclasses import replace
    return replace(thing, sigma_p=sigma, f_bear=field(thing, "f_bear", data.get("force_n")), touched=entry.get("kind") == "contact" or field(thing, "touched"))


def contact_error_m(B, speed_m_s, q, u):
    return abs(speed_m_s) * B.t_det() + B.quantum(q, u)


def observed_stiffness(B, points, forces, q, u):
    """Positive slope of this push's force against measured travel, N/m.

    A regression is a measurement, not a force threshold. A sample span below
    the pose's resolution cannot give a stiffness; a rigid contact then keeps
    the servo-stiffness model. No noise or quantum is a denominator.
    """
    import numpy as np
    if len(points) < 2:
        return None
    x = np.asarray(points, dtype=float) @ np.asarray(u, dtype=float)
    if float(np.ptp(x)) <= path_noise_m(B, q, u):
        return None
    x = x - x.mean()
    f = np.asarray(forces, dtype=float)
    slope = float(x @ (f - f.mean()) / (x @ x))
    return slope if math.isfinite(slope) and slope > 0 else None
