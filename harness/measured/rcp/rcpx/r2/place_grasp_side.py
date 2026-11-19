# SPDX-License-Identifier: AGPL-3.0-only
"""The level grasp of package place (moved out of rcpx/r2/place_grasp.py, which is over its length; the rule is
unchanged): around the narrow side at half height, along a bearing this arm's IK reaches."""

import math

import numpy as np

from rcpx.r2.place_grasp import EPS, HALF, LOOK, MM, _finger_across, _refuse, extent_along, line, num


def side(ex, B, J, S, t, sup_z):
    """A level grasp around the narrow side at half height, along a bearing this arm's IK reaches: the stated
    approach first when it is level, else the bearings from the robot outward. With no outline seen: the open
    jaws level at the point as given, at the point's own height. A bearing across which the thing was MEASURED
    whole and wider than the jaws is left out."""
    s0 = ex._sense()
    err = S["sigma_xy"]
    narrow, wide, bearing = S["narrow"], S["wide"], S["bearing"]
    base = S["top_z"] - S["height"] if S["height"] is not None else sup_z
    z = S["top_z"] if narrow is None or base is None else HALF * (S["top_z"] + base)
    c0 = np.asarray(S["centre"], dtype=float)
    lines = [line(d, "acted on what the look shows, the contact stop armed", S["remove_by"]) for d in S["doubt"]]
    if narrow is None:
        lines.append(line("no outline of it is seen", "the open jaws came level to the point as given",
                          S["remove_by"] or LOOK))
    level = abs(float(t[-1])) < HALF
    first = math.atan2(t[1], t[0]) if level else math.atan2(c0[1], c0[0])
    step = max(_finger_across(J), EPS) / max(HALF * (wide or J.max_m), EPS)
    k, tried = 0, 0
    while k * step <= HALF * math.pi:
        for a in ([first + k * step] + ([first - k * step] if k else [])):
            tried += 1
            axis = np.array([math.cos(a), math.sin(a), 0.0])
            across = None if narrow is None else extent_along(narrow, wide, bearing, a + HALF * math.pi)
            if across is not None and S["whole"] and across - err - err > J.max_m:
                continue
            along = 0.0 if narrow is None else extent_along(narrow, wide, bearing, a)
            depth = 0.0 if narrow is None else min(x for x in (HALF * along, J.finger_len_m) if x is not None)
            p = np.array([c0[0], c0[1], z]) - axis * (HALF * along - depth)
            q, R_low, yaw_r, squared = ex._pl_best_pose(p, axis, [a + HALF * math.pi], J.frac_for_width(J.max_m),
                                                        [s0.q, ex.model.q_home], across)
            if q is None:
                continue
            v_free = num(B.v_free(q, axis), math.inf)
            return {"strategy": "side", "yaw": float(yaw_r), "slide_m": 0.0, "open_m": float(J.max_m),
                    "tip_z": float(p[-1]), "approach_axis": [float(x) for x in axis], "v_down": v_free,
                    "why": f"a side grasp: level along {math.degrees(a):.0f} deg, the jaws around it at "
                           + ("the point's own height" if narrow is None else "half height")
                           + f", {depth * MM:.0f} mm past its near face",
                    "centre": [float(c0[0]), float(c0[1])], "width_m": None if across is None else float(across),
                    "depth_m": float(depth), "q_low": q, "R": R_low, "squared": bool(squared), "v_free": v_free,
                    "neighbour_gap_m": None, "top_z": float(S["top_z"]), "support_z": sup_z,
                    "expect_m": None if across is None else float(across), "sigma_xy": err,
                    "off_m": num(B.e_cross(v_free if math.isfinite(v_free) else 0.0), 0.0),
                    "band_m": num(B.rigid(q, axis), 0.0), "doubt": lines, "sigma_z": S["sigma_z"]}
        k += 1
    return _refuse("not_reachable", f"no level bearing of {tried} tried lets this arm bring the jaws around it",
                   "pick it from above (grasp top), or bring it nearer the robot's base")
