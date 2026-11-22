# SPDX-License-Identifier: AGPL-3.0-only
"""Measured obstacle bounds. All distances and speeds use SI units."""
import math
from rcpx.bounds_ratios import BAND, QUANT, REL, TWO_READ, ONE_READ


def of(ex):
    from rcpx import bounds
    return bounds.of(ex)


def margin(B, v, thing, body):
    return B.clearance(v, thing=thing, body=body)


def margin_item(B, v, thing, body):
    value = margin(B, v, thing, body)
    source, evidence = B.src("clearance")
    cross = getattr(B, "e_cross", B.lag)(v)
    position = BAND * field(thing, "sigma_p")
    rest = value - cross - position
    evidence = (f"{evidence}; cross-track {cross:g} m (lag until measured), "
                f"position band {position:g} m, fit and sampling {rest:g} m")
    return {"what": "travel", "value": value, "unit": "m", "over_mm": None,
            "source": source, "evidence": evidence,
            "lift": "state a speed whose clearance margin fits this path"}


def clearing_speed(B, clearance_m, thing, body, free_speed):
    if margin(B, 0, thing, body) > clearance_m:
        return None
    if margin(B, free_speed, thing, body) <= clearance_m:
        return free_speed
    lo, hi = 0, free_speed
    # Bisection ends at floating point resolution, without a travel or speed tolerance.
    while True:
        mid = (lo + hi) / 2.0
        if mid == lo or mid == hi:
            return lo
        if margin(B, mid, thing, body) <= clearance_m:
            lo = mid
        else:
            hi = mid


def field(thing, name, default=0):
    value = thing.get(name, default) if isinstance(thing, dict) else getattr(thing, name, default)
    return default if value is None else value


def uncertainty_m(B, entry):
    data = entry.get("data") or {}
    sigma = field(B.thing(entry.get("id")), "sigma_p")
    return max(sigma, (data.get("error_mm") or 0) / 1000.0,
               (data.get("depth_uncertainty_mm") or 0) / 1000.0)


def contact_radius_m(B, entry, surface_size_m):
    return uncertainty_m(B, entry) + surface_size_m / 2.0 + (entry.get("data") or {}).get("part_spread_mm", 0) / (2.0 * 1000.0)


def trigger_pad_m(B, hinge_entry, act_entry):
    return uncertainty_m(B, hinge_entry) + uncertainty_m(B, act_entry)


def housing(B, model):
    import numpy as np
    known = getattr(model, "camera_body_geometry", None)
    if callable(known):
        known = known()
    if known:
        return dict(known, source="model", evidence="camera body geometry from the loaded model")
    stated = (model.spec.get("camera") or {}).get("housing_mm")
    if stated is not None:
        # the robot's own spec states the camera body: width, height (image right, image up) and depth behind the
        # front face, with how far the lens sits behind that face (housing_lens_back_mm, zero when not stated)
        size = np.asarray(stated, dtype=float) / 1000.0
        back = float((model.spec.get("camera") or {}).get("housing_lens_back_mm") or 0.0) / 1000.0
        return {"shape": "box", "c": np.array([0.0, 0.0, size[2] / 2.0 - back]), "half": size / 2.0, "size_m": size,
                "source": "model", "evidence": "the camera body the robot's spec states (camera housing_mm)"}
    carrier = model.mj.body(int(model.mj.cam_bodyid[model._cam])).name
    caps = [c for c in model.collision_capsules(model.q_home) if c[0] == carrier]
    radius = max((float(np.linalg.norm(c[2] - c[1])) + 2.0 * c[3] for c in caps), default=model.length_scale)
    return {"shape": "sphere", "c": np.zeros(3), "half": np.full(3, radius),
            "radius": radius, "size_m": np.full(3, 2.0 * radius), "source": "model",
            "evidence": "assumed until measured: the carrying link's largest capsule dimension"}


def known(B, name):
    """A scalar the robot's self-knowledge holds under this name, or None: only a finite number counts (a
    self-knowledge that answers an unknown name with an empty record, a tuple or a table answers nothing here)."""
    book = getattr(B, "know", None)
    value = getattr(book, name, None)
    getter = getattr(book, "get", None)
    if (value is None or callable(value)) and callable(getter):
        try:
            value = getter(name)
        except Exception:
            return None
    if isinstance(value, tuple) and value:
        value = value[0]
    if isinstance(value, dict):
        value = value.get("value")
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value != value:
        return None
    return float(value)


def friction(B, n):
    """Dry joint friction the model may lack, per joint, as a share of each torque limit, with its source: the
    hidden joint torque of the robot's self-knowledge (tau_hidden holds what dry friction and the model's own
    error hide; B.tau_thr at rest adds JOINT x its noise)."""
    import numpy as np
    shares = np.asarray(B.tau_thr(0), dtype=float)[:n] / np.asarray(B.tau_max(), dtype=float)[:n]
    return shares, B.src("tau_thr")[0]


def relative_bands(B, hidden, quiet):
    return {"ambiguous": hidden + max(hidden, TWO_READ * B.e_tau()),
            "tie": hidden + REL * B.e_tau(),
            "free": hidden + ONE_READ * B.e_tau(),
            "poor": quiet + ONE_READ * B.e_tau()}


def watch_gap(B, hidden=0):
    """How much worse (in relative residual) the tool must fit a loud reading than the best part before the watch
    counts it: the hidden level (the best part's own residual on that reading, what the fit itself leaves) plus
    TWO_READ x the torque estimate's relative error (the watch needs two readings in a row)."""
    return hidden + TWO_READ * B.e_tau()


def jaw_lag_frac(B, model):
    import numpy as np
    jaws = model.spec.get("gripper") or {}
    measured = known(B, "jaw_free_lag_frac")
    if measured is not None:
        return float(measured), "self_test"
    # A model stiction torque divided by servo stiffness and travel is a gap share.
    m = model.mj
    shares = []
    for name in jaws.get("joints", []):
        j = m.joint(name).id
        span = float(m.jnt_range[j, 1] - m.jnt_range[j, 0])
        dof = int(m.jnt_dofadr[j])
        kp = jaws.get("kp", model.spec.get("gripper_kp", 0))
        if isinstance(kp, (list, tuple)):
            kp = max(kp, default=0)
        shares.append(float(m.dof_frictionloss[dof]) / (kp * span) if kp and span else 0)
    step = known(B, "jaw_step")
    step = jaws.get("resolution", 0) if step is None else step
    spans = np.abs(np.asarray(jaws.get("open", [])) - np.asarray(jaws.get("closed", [])))
    mask = spans > 0
    rounding = float(np.max(np.broadcast_to(step, spans.shape)[mask] / spans[mask])) if mask.any() else 0
    return max(shares, default=0) + QUANT * rounding, "model"


def position_bands(things):
    import numpy as np
    return BAND * np.asarray([field(t, "sigma_p") for t in things])


def part_speeds(B, q0, q1, points0, points1, starts, stated_speed=None):
    """Every joint is at most its rated speed; each body's fastest sample sets its speed."""
    import numpy as np
    dq = np.abs(np.asarray(q1) - q0)
    rated = np.asarray(B.qd_max())
    times = np.divide(dq, rated, out=np.where(dq > 0, np.inf, 0.0), where=rated > 0)
    seconds = float(np.max(times))
    p0, p1 = np.asarray(points0), np.asarray(points1)
    delta = p1 - p0
    # FK cancellation inherits the chain's coordinate rounding, even at a zero coordinate.
    # Four coordinates per homogeneous matrix, accumulated across the joints.
    rounding = np.spacing(max(float(np.max(np.abs(p0))), float(np.max(np.abs(p1))))) * (4 * len(dq))
    delta = np.where(np.abs(delta) <= np.abs(rounding), 0.0, delta)
    travel = np.linalg.norm(delta, axis=1)
    speeds = np.maximum.reduceat(travel, starts) / seconds if seconds > 0 else np.zeros(len(starts))
    return speeds if stated_speed is None else np.minimum(speeds, stated_speed)
