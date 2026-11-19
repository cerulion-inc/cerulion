# SPDX-License-Identifier: AGPL-3.0-only
"""Device timing and geometry from the sheet, the camera and measured contacts."""
import math


def of(ex):
    from rcpx import bounds
    return bounds.of(ex)


def field(thing, name, default=0):
    value = thing.get(name, default) if isinstance(thing, dict) else getattr(thing, name, default)
    return default if value is None else value


def frame_time_s(B, backend=None, model=None):
    camera = (getattr(model, "spec", {}) or {}).get("camera") or {}
    period = getattr(backend, "camera_frame_s", None) or camera.get("frame_s")
    rate = getattr(backend, "camera_rate_hz", None) or camera.get("rate_hz")
    if period is not None:
        return float(period), "model", "camera frame period"
    if rate:
        return 1 / float(rate), "model", "camera frame rate"
    return B.dt(), "model", "the backend supplies one camera frame per control period"


def near_focus_m(B, model):
    camera = model.spec.get("camera") or {}
    value = camera.get("near_focus_m")
    if value is not None:
        return float(value), "model", "camera near focus from the model"
    # The renderer clipping plane is the only optical limit a model camera states.
    value = float(model.mj.vis.map.znear * model.mj.stat.extent)
    return value, "model", "camera clipping plane; near focus is not separately stated"


def blink_period_s(entry, measured=None):
    if measured is not None and measured > 0:
        return measured, "seen", "blink period measured by this session's watch"
    entry = entry or {}
    values = []
    for item in [entry] + list(entry.get("parts") or []):
        for key in ("blink_period_s", "period_s"):
            if isinstance(item.get(key), (int, float)) and item[key] > 0:
                values.append(item[key])
    return (min(values), "sheet", "blink period on the device sheet") if values else (None, "model", "no blinking state is known")


def wait_s(B, entry, frame_s, measured_period=None):
    period, source, evidence = blink_period_s(entry, measured_period)
    values = []
    for transition in (entry or {}).get("transitions") or []:
        within = transition.get("within_s")
        if isinstance(within, (list, tuple)):
            within = min(within) if within else None
        if isinstance(within, (int, float)) and within >= 0:
            values.append(within)
    if period is not None:
        values.append(period)
    if values:
        return min(values), "sheet" if (entry or {}).get("transitions") else source, "least transition within_s or blink period"
    return frame_s, "model", "one camera frame once the tip is clear of the cue's sight line"


def burst(B, entry, frame_s, measured_period=None):
    period, source, evidence = blink_period_s(entry, measured_period)
    return (1 if period is None else math.ceil(period / frame_s) + 1), frame_s, source, evidence


def extent_m(B, entry, points_mm=()):
    import numpy as np
    sizes = []
    for item in [entry or {}] + list((entry or {}).get("parts") or []):
        size = item.get("size_mm")
        if size is None:
            dimensions = item.get("where_mm") or {}
            if isinstance(dimensions, dict):
                size = [dimensions[key] for key in ("diameter", "width", "height") if key in dimensions] or None
        if isinstance(size, dict):
            size = [value for key, value in size.items() if key in
                        ("width", "depth", "height", "length", "diameter", "size", "thickness")]
            if not size:
                size = None
        if size is not None:
            sizes.append(float(np.max(np.asarray(size))) / 1000.0)
    if points_mm:
        sizes.append(float(np.linalg.norm(np.ptp(np.asarray(points_mm), axis=0))) / 1000.0)
    thing = B.thing((entry or {}).get("id"))
    extent = field(thing, "extent")
    if isinstance(extent, dict):
        extent = extent.get("points_m", extent.get("size_m", 0))
    if np.ndim(extent):
        extent = float(np.max(np.linalg.norm(np.asarray(extent), axis=-1)))
    return max(sizes + [extent]) + field(thing, "sigma_p")


def same_m(B, sigma_a, sigma_b):
    return sigma_a + sigma_b


def moved_m(B, q, u, force_n):
    return B.rigid(q, u, force_n)


def timing_error_s(B, history):
    differences = [abs(row["contact_s"] - row["predicted_s"]) for row in history
                   if row.get("contact_s") is not None and row.get("predicted_s") is not None]
    spread = max(differences) - min(differences) if differences else 0
    return B.t_det() + spread


def press_contact_s(B, kwargs, thing, q, u, history=()):
    spec = B.force("press", thing, q, u, intent=kwargs)
    asked = float(kwargs.get("force_n", kwargs.get("max_n", spec.first_n)))
    asked = min(asked, spec.ceiling_n)
    ramp = 0
    if asked > spec.first_n:
        force = spec.first_n
        while force < asked:
            rate = spec.rate_n_s(force)
            if rate is None or rate <= 0:
                raise ValueError("force ramp has no positive rate")
            seconds = min(B.t_det(), (asked - force) / rate)
            ramp += seconds
            next_force = min(asked, force + rate * seconds)
            if next_force <= force:
                break
            force = next_force
    predicted = ramp + float(kwargs.get("hold", kwargs.get("hold_s", 0))) + B.t_settle()
    ratios = [row["contact_s"] / row["predicted_s"] for row in history
              if row.get("contact_s") is not None and (row.get("predicted_s") or 0) > 0]
    return predicted * max([1] + ratios), spec.items


def long_limit_s(B, long_press_s, history):
    return max(0, long_press_s - timing_error_s(B, history))


def long_stop_s(B, long_press_s, ease_s=None):
    """Contact time at which the tick hook asks a press to stop: long_press_s less the detection delay and the
    longest the tip stays on after a stop is asked (ease_s; the settling time when none is given)."""
    return max(0, long_press_s - (B.t_det() + (B.t_settle() if ease_s is None else ease_s)))


def contact_sigma_m(B, entry, result, q, u):
    data = (entry or {}).get("data") or {}
    measured = field(B.thing((entry or {}).get("id")), "sigma_p", None)
    values = [value / 1000 for value in (data.get("error_mm"), data.get("sigma_p_mm")) if value is not None]
    if measured is not None:
        values.append(measured)
    if values:
        return max(values), "contact", "the stored contact position uncertainty"
    contact = result.get("contact") if isinstance(result.get("contact"), dict) else {}
    speed = contact.get("tip_speed_mm_s")
    v = B.v_free(q, u) if speed is None else speed / 1000
    return v * B.t_det() + B.quantum(q, u), "model" if speed is None else "contact", "speed not reported: the free speed bound" if speed is None else "reported contact speed times detection delay plus quantum"


def prediction_source(history):
    count = sum(row.get("contact_s") is not None and (row.get("predicted_s") or 0) > 0 for row in history)
    return f"measured on {count} presses" if count else "model, not yet checked by a press of this session"
