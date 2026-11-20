# SPDX-License-Identifier: AGPL-3.0-only
"""Interface validation, settings and free documents for mobile.

Backend settings are noted and ignored. Ratings come from mobile_info.
For a valid mobile interface the initial floor always comes from mobile_info.
"""

import copy
import numbers

import numpy as np

from rcpx.r2 import mobile_bounds as D, mobile_geom as G, mobile_world as W


def finite(v):
    return isinstance(v, numbers.Real) and not isinstance(v, (bool, np.bool_)) and bool(np.isfinite(v))


def vector(v, n):
    return isinstance(v, (list, tuple, np.ndarray)) and len(v) == n and all(finite(x) for x in v)


def valid_info(info, backend):
    if not isinstance(info, dict) or info.get("drive") not in ("holonomic", "differential"):
        return False
    if any(not finite(info.get(k)) or info[k] <= 0 for k in ("max_mps", "max_turn_rps")):
        return False
    fp, inst, torso = info.get("footprint_b"), info.get("b_in_base"), info.get("torso")
    if not isinstance(fp, dict) or not isinstance(inst, dict) or "torso" not in info:
        return False
    if any(not vector(fp.get(k), 2) or fp[k][0] >= fp[k][1] for k in ("x", "y")):
        return False
    if not finite(fp.get("z_top")) or not finite(info.get("floor_z_b_at_torso0")):
        return False
    if not vector(inst.get("pos"), 3) or not vector(inst.get("quat_wxyz"), 4):
        return False
    if np.linalg.norm(inst["quat_wxyz"]) < 1e-12 or info.get("cameras_ride") != "frame_b":
        return False
    if torso is not None:
        if not isinstance(torso, dict) or not vector(torso.get("range_m"), 2):
            return False
        if torso["range_m"][0] > torso["range_m"][1] or not finite(torso.get("max_mps")):
            return False
        if torso["max_mps"] <= 0 or not vector(torso.get("axis_b"), 3):
            return False
        if not np.isclose(np.linalg.norm(torso["axis_b"]), 1.0):
            return False
    methods = ["odometry", "command_base", "hold_base"] + (["command_torso"] if torso else [])
    return all(callable(getattr(backend, k, None)) for k in methods)


def setup(ex):
    st = ex.state["mobile"] = {"info": None}
    problems = []
    try:
        fn = getattr(ex.backend, "mobile_info", None)
        if fn is None:
            return
        info = fn()
        if info is None:
            return
        if not valid_info(info, ex.backend):
            raise ValueError("malformed mobile interface")
        odo = ex.backend.odometry()
        if not isinstance(odo, dict) or not all(finite(odo.get(k)) for k in
                                               ("x_m", "y_m", "yaw_rad", "torso_m")):
            raise ValueError("malformed mobile odometry")
        info = copy.deepcopy(info)
    except Exception as exc:
        problems.append(f"mobile interface unavailable: {type(exc).__name__}: {exc}")
        info = None
    if info is not None:
        cfg = getattr(ex.backend, 'mobile_cfg', None)
        for key in sorted(cfg if isinstance(cfg,dict) else {}):
            D.journal(ex,'mobile_setting_ignored', key=str(key), why='base limits now come from its controller and measurements')
        st.update(info=info, entries={}, old_looks=set(), seen_looks=set(), old_marks={}, look_tools=[],
                  driven={"distance_mm": 0.0, "turn_deg": 0.0, "torso_mm": 0.0}, know=D.empty_know())
        W.floor(ex, odo)
        st["look_pre"] = lambda name, kwargs: W.look_pre(ex, name, kwargs)
        st["post"] = lambda name, kwargs, result: W.post(ex, name, kwargs, result)
        ex.add_pre(st["look_pre"])
        ex.add_post(st["post"])
        W.post(ex, "setup", {}, {})
        W.survey(ex)
        W.first_hook(ex)
        join_dark_gate(ex)
    if problems:
        message = "; ".join(problems)
        ex.r2_problems["mobile"] = message
        D.journal(ex,"r2_problem", package="mobile", why=message)


def pose(odo, height=False):
    out = {"x_mm": round(odo["x_m"] * 1000.0, 1), "y_mm": round(odo["y_m"] * 1000.0, 1),
           "yaw_deg": round(float(np.rad2deg(odo["yaw_rad"])), 2)}
    if height:
        out["height_mm"] = round(odo["torso_m"] * 1000.0, 1)
    return out


def speeds(ex):
    return D.base_limits(ex.state['mobile']['info'])


def base_document(ex, session=None, **kw):
    st = ex.state.get("mobile", {})
    if st.get("info") is None:
        return {"mobile": False, "note": "This backend offers no usable mobile interface."}
    info, odo = st["info"], ex.backend.odometry()
    v, w, u = speeds(ex)
    fp = G.footprint_now(info, odo["torso_m"])
    return {"pose": pose(odo), "height_mm": round(odo["torso_m"] * 1000.0, 1),
            "driven": {k: round(float(x), 2 if k == "turn_deg" else 1) for k, x in st["driven"].items()},
            "limits": {"drive": info["drive"], "max_mm_s": round(v * 1000.0, 1),
                       "max_turn_deg_s": round(float(np.rad2deg(w)), 2), "torso_mm_s": round(u * 1000.0, 1),
                       "torso_range_mm": None if info["torso"] is None else
                       [round(x * 1000.0, 1) for x in info["torso"]["range_m"]]},
            "footprint_mm": {k: [round(x * 1000.0, 1) for x in val] if isinstance(val, list) else
                             round(val * 1000.0, 1) for k, val in fp.items()},
            "floor_z_mm": round(st["floor_z_m"] * 1000.0, 1),
            "frame": "The pose is the base relative to where it stood at the start, x forward, y left, yaw "
                     "anticlockwise seen from above; points are in the robot frame, which rides the base and the torso."}


def body_patch(ex, session, content):
    if ex.state.get("mobile", {}).get("info") is None or not isinstance(content, dict):
        return content
    if not isinstance(content.get("base"), dict):
        return content
    out = copy.deepcopy(content)
    doc = base_document(ex)
    out["base"]["mobile"] = {**doc["limits"], "footprint_mm": doc["footprint_mm"],
                             "pose": {**doc["pose"], "height_mm": doc["height_mm"]},
                             "cameras_ride": ex.state["mobile"]["info"]["cameras_ride"],
                             "how": "the base drives on the floor and the torso raises and lowers the arm "
                                    "and the scene cameras together"}
    if isinstance(out["base"].get("standing_surface"), dict):
        out["base"]["standing_surface"]["floor_z_mm"] = doc["floor_z_mm"]
    return out


def join_dark_gate(ex):
    """drive and torso move the robot toward the world: the waiting package's dark gate refuses them while the
    newest light reading is dark (waiting.py documents ex.state["waiting"]["toward_world"] as the list to join)."""
    w = ex.state.get("waiting")
    if ex.state.get("mobile", {}).get("info") is not None and isinstance(w, dict) \
            and isinstance(w.get("toward_world"), set):
        w["toward_world"].update(("drive", "torso"))


def FILTER(surface, resources, handlers, ex=None, box=None):
    if ex is not None:
        W.first_hook(ex)
        W.survey(ex, surface)
        join_dark_gate(ex)
    return surface, resources, handlers
