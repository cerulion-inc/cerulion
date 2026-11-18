# SPDX-License-Identifier: AGPL-3.0-only
"""The look before a close.

When no width is stated or seen for what the jaws close on, the robot looks for itself before the close if a camera
of its own can see between the jaws: one picture, no motion, no control period, said in the receipt. The width then
comes from that look with its error and the close runs as the design rulings built it (the jaws' own rate down to where
the pads can first meet the thing, the rate solved for the hold's squeeze from there). Where no camera of this body
measures between the jaws the close runs at the arrival rate from where the jaws stand and prices that beside what a
width would have cost (control_hold_close.close).

before(ex, look) is the core's door; the seeing itself is rcpx/r2/see_jaws.py (contract C11), read lazily so that a
core with no seeing package closes as before. Nothing here names a robot.
"""

NO_CAMERA = "none: no camera of this body measures between the jaws"


def before(ex, look=True):
    """The width dict of a close that was given no width: {"m", "sigma_m", "source", ...}. source "looked: look ID,
    the robot's own look before the close" with the width (m) and its error where see_jaws.width_between read one;
    else "none: ..." with why (m None). look False: the caller looked already or asked for none."""
    if not look:
        return {"m": None, "sigma_m": None, "source": NO_CAMERA, "why": "no look was asked of this close"}
    try:
        from rcpx.r2 import see_jaws
    except ImportError:
        return {"m": None, "sigma_m": None, "source": NO_CAMERA, "why": "this build has no seeing package"}
    try:
        got = see_jaws.width_between(ex)
    except Exception as exc:        # a look that cannot be read costs the close nothing: it runs as with none
        ex._journal("hold_problem", where="look before the close", error=f"{type(exc).__name__}: {exc}")
        got = None
    if not isinstance(got, dict) or got.get("m") is None:
        why = (got or {}).get("why") if isinstance(got, dict) else None
        return {"m": None, "sigma_m": None, "source": NO_CAMERA, "why": why or "the look gave no width"}
    return {"m": float(got["m"]), "sigma_m": float(got["sigma_m"]),
            "source": f"looked: look {got['look']}, the robot's own look before the close",
            "look": got["look"], "how": got.get("source"), "periods": 0, "upper": bool(got.get("upper"))}
