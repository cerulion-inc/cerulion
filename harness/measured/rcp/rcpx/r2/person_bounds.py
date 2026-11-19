# SPDX-License-Identifier: AGPL-3.0-only
"""Constraint distances derived from located uncertainty and viewing resolution."""
from rcpx.bounds_ratios import QUANT


def of(ex):
    from rcpx import bounds
    return bounds.of(ex)


def field(thing, name, default=0):
    value = thing.get(name, default) if isinstance(thing, dict) else getattr(thing, name, default)
    return default if value is None else value


def sigma_m(B, entry):
    return max(field(B.thing(entry.get("id")), "sigma_p"),
               ((entry.get("data") or {}).get("error_mm") or (entry.get("data") or {}).get("sigma_p_mm") or 0) / 1000.0,
               ((entry.get("data") or {}).get("depth_uncertainty_mm") or 0) / 1000.0)


def extent_m(B, thing):
    import numpy as np
    extent = field(thing, "extent")
    if isinstance(extent, dict):
        extent = extent.get("points_m", extent.get("size_m", 0))
    if np.ndim(extent):
        extent = float(np.max(np.linalg.norm(np.asarray(extent), axis=-1)))
    return extent


def near_m(B, thing, lateral_error_m):
    return extent_m(B, thing) + lateral_error_m + field(thing, "sigma_p")


def tolerance_m(B, depth_m, f_px, match_error_px):
    value = abs(depth_m) * match_error_px / f_px
    return value, "measurable", "pixel size at this depth times the look comparison's match error"


def grip_margin_m(B, located_error_m, jaw_sampling_error_m):
    return located_error_m + jaw_sampling_error_m


def target_match_m(B, target, q=None, u=None):
    return B.travel("touch", target, q, u).past_m


def side_matches(B, a, expected, lever_m, error_m):
    import numpy as np
    miss = float(np.linalg.norm(np.asarray(a) - np.asarray(expected))) * lever_m
    return miss <= error_m + np.finfo(float).eps * abs(lever_m)


def jaw_error_m(B, jaws, points):
    import numpy as np
    if len(points) < 2:
        return 0
    # A convex hull fills between these measured vertices; the sampling error is the jaw travel between sweep poses.
    vertices = sum(len(local) for _, local in jaws.local)
    poses = np.asarray(points).reshape(-1, vertices, np.asarray(points).shape[-1])
    return float(np.max(np.linalg.norm(np.diff(poses, axis=0), axis=-1))) / 2.0 if len(poses) > 1 else 0


def jaw_step_frac(B, jaws):
    """A measured jaw step expressed in its own commanded opening fraction."""
    import numpy as np
    book = getattr(B, "know", None)
    step = getattr(book, "jaw_step", None)
    getter = getattr(book, "get", None)
    if step is None and callable(getter):
        step = getter("jaw_step")
        if isinstance(step, tuple):
            step = step[0]
    if step is None:
        # Until jaw-specific readings exist, the model's resolution is the only evidence.
        step = (jaws.model.spec.get("gripper") or {}).get("resolution", 0)
    span = np.abs(jaws.open - jaws.closed)
    mask = span > 0
    return QUANT * float(np.max(np.broadcast_to(step, span.shape)[mask] / span[mask])) if mask.any() else 0
