# SPDX-License-Identifier: AGPL-3.0-only
"""Hooks of package grasp (see rcpx/r2/grasp.py): the held-state watch on every result, the gripper result
keys, reach_check's force_n and base_limited, and the robot://body patch. They read the gripper only through
Backend.read()["gripper"] (via the Mixin's _grasp_read) and the executor's own base_give; no ground truth.
"""

import numpy as np

from rcpx import control_hold
from rcpx.context import refusal
from rcpx.r2.grasp_hand import finite as _finite


MM = 1000.0


def watch(ex):
    """Post hook on every result while a part is held: the watched state follows the core's one record."""

    def post(name, kwargs, result):
        """Read the result and the core's record without stepping the robot."""
        st = ex.state.get("grasp") or {}
        if not st.get("held") or not isinstance(result, dict) or name in ("grasp", "release"):
            return None
        q, target, w = ex._grasp_read()
        if q is None:
            return None
        w_mm = w * MM
        change = w_mm - float(st["width_mm"])
        block = {"held": True, "width_mm": round(w_mm, 2), "width_change_mm": round(change, 2)}
        if control_hold.check(ex) is None:      # the core's record ended: lost, or the jaws were opened
            why = (control_hold.ended(ex) or {}).get("why")
            if why == "slipped":
                block.update(slipped_out=True, why=f"the jaws closed a further {-change:.2f} mm since the grasp: "
                             "the part is no longer between them")
                st.update(slipped_out=True)
                if st.get("squeeze_n") is not None:
                    st["s_slipped_n"] = st["squeeze_n"]
                lost = "slipped_out"
            else:
                block.update(opened=True, why=f"the jaws opened {change:.2f} mm since the grasp: the part is let go")
                lost = "opened"
            block["held"] = False
            st.update(held=False)
            ex.note_event("grasp_lost", reason=lost, primitive=name, width_mm=round(w_mm, 2),
                width_change_mm=round(change, 2))
        return {**result, "grasp": block}

    return post


def gripper_post(ex):
    """Post hook on the round one gripper tool: its result also says width_mm, closed_on_object (the close ended
    on a thing), holding (the core's record stands), pressing and still. Only grasp starts a watched hold; this
    hook never steps the robot."""

    def post(name, kwargs, result):
        """Read the result and the core's record without stepping the robot."""
        if not isinstance(result, dict) or "refused" in result or ex._grasp_hand is None:
            return None
        hand = ex._grasp_hand
        q, target, w = ex._grasp_read()
        if q is None or target is None:
            return None
        holding = control_hold.held(ex) is not None
        return {"said": result.get("said") or ("the jaws hold; lift and watch the grasp" if holding else
                "the jaws finished without a hold; inspect them before lifting"), **result,
                    "width_mm": round(w * MM, 2), "closed_on_object": result.get("ended") == "on_thing",
                "holding": holding, "pressing": bool(hand.squeeze_n(q, target)), "still": True}


    return post


def _asked_frac(width_frac):
    """The opening fraction a gripper call asked for, as the round one gripper reads it; None if unreadable."""
    if width_frac in ("open", "close"):
        return 1.0 if width_frac == "open" else 0.0
    if isinstance(width_frac, str) or not _finite(width_frac):
        return None
    return min(1.0, max(0.0, float(width_frac)))


def reach_pre(name, kwargs):
    """Remove and validate the callers stated push force before the base reach primitive runs."""
    if "force_n" not in kwargs:
        return None
    f = kwargs.pop("force_n")
    if f is not None and (not _finite(f) or float(f) <= 0):
        return refusal("force_n", f"force_n must be a positive number of newtons, got {f!r}",
                       "the push force you plan to use there, in newtons")
    return None


def _impulse_fits(ex, kwargs, result):
    """Would impulse run fast enough at the pose reach_check solved (else the present pose): along the approach
    when one was given, else along any of the six base directions."""
    q = ex._sense().q
    sol = result.get("q_deg")
    if isinstance(sol, dict) and len(sol) == ex.model.n:
        q = np.radians([float(sol.get(n, 0.0)) for n in ex.model.joint_names])
    a = kwargs.get("approach")
    dirs = [a] if isinstance(a, (list, tuple)) and len(a) == 3 and _finite(*a) and any(a) else \
        [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
    return any(ex._grasp_impulse_ok(q, np.asarray(v, dtype=float) / np.linalg.norm(np.asarray(v, dtype=float)))
               for v in dirs)


def reach_post(ex):
    """Compare the reach verdict with the measured base give; no force is taken from a cfg cap."""
    def post(name, kwargs, result):
        """Read the result and session measurements without stepping the robot."""
        if not isinstance(result, dict) or "refused" in result:
            return None
        asked = kwargs.get("force_n")
        avail = result.get("force_n")
        if isinstance(avail, dict):
            avail = max((float(v) for v in avail.values() if _finite(v)), default=None)
        # without force_n: the most the arm may push there (the force reach_check reports there)
        need = float(asked) if _finite(asked) else (float(avail) if _finite(avail) else None)
        give = ex.base_give or {}
        bound = give.get("bound_n")
        if not give:
            return {**result, "base_limited": None,
                    "base_note": "the base give is not measured: touch a firm surface, then measure_base_give"}
        if bound is None:
            return {**result, "base_limited": False, "base_note": "the measured give puts no bound on pushes"}
        if need is None:
            return {**result, "base_limited": None, "base_bound_n": round(bound, 2),
                    "base_note": f"steady pushes over {bound:.1f} N walk the base; the force there is not known: "
                                 "give force_n (the push you plan) to compare"}
        if bound >= need:
            return {**result, "base_limited": False, "base_bound_n": round(bound, 2)}
        modes = (["impulse"] if _impulse_fits(ex, kwargs, result) else []) + \
            (["pinch"] if ex._grasp_hand is not None else [])
        tail = (f"{' or '.join(modes)} does not hold that load on the base" if modes else
                "no mode here avoids loading the base: lower the force or brace the base")
        return {**result, "base_limited": True, "base_bound_n": round(bound, 2), "force_needed_n": round(need, 2),
                "modes_without_base_load": modes,
                "base_note": f"the base gives {give.get('mm_per_n', 0.0):.2f} mm per N: a steady push over "
                             f"{bound:.1f} N walks it before {need:.1f} N"
                             + ("" if _finite(asked) else " (no force_n given: the most the arm may push there)")
                             + f"; {tail}"}

    return post


def body_patch(ex, session, content):
    """Expose the hands model geometry and force range in the body document."""
    if not isinstance(content, dict):
        return content
    hand = ex._grasp_hand
    return {**content, "grasp": None if hand is None else hand.describe()}
