# SPDX-License-Identifier: AGPL-3.0-only
"""Mobile record maintenance and look gates.

ex.state['mobile'] keeps info, per-entry odometry accumulators and last written points,
old_looks (cumulative ids), seen_looks (ids returned by primitives), old_marks (ids only),
look_tools, driven (absolute travel totals), and the look pre hook. World geometry is
re-expressed; other packages receive base_driven with a full relative transform.
The acts list in witness documents point_mm in its own module docstring, and the probe
package documents its fit (axis_dir, motion_dir, center_mm, contact_mm) in its own; those
fields are re-expressed too (in World data and in ex.state["probe"]["fits"]). Other
packages' undocumented records and candidate shapes are left alone.
"""

import inspect

import numpy as np

from rcpx.context import POSITIONAL, refusal
from rcpx.r2 import mobile_bounds as D, mobile_geom as G

try:
    from rcpx.r2.epochs_record import DIRECTION_KEYS, POINT_DATA_KEYS
except ImportError:
    DIRECTION_KEYS = ("normal_est", "normal", "approach_normal", "fit_normal", "part_normal")
    POINT_DATA_KEYS = ("part_point_mm",)

# Arguments that may name a look id (look, before and after are other packages' look arguments). A value is
# refused only when it IS an old look id, so a non-look argument of the same name is never affected.
LOOK_ARGS = ("look_id", "looks", "ref", "like", "look", "before", "after")
# Frame B geometry the probe package documents in its own module docstring for its fit (the World device entry's
# data and ex.state["probe"]["fits"][label]): directions turn with the robot, points move with it.
FIT_DIRECTION_KEYS = ("axis_dir", "motion_dir", "pushed_along")
FIT_POINT_KEYS = ("center_mm", "contact_mm")
HOW = "re-expressed by the base's odometry: the robot moved, the world did not"


def first_hook(ex):
    st = ex.state.get("mobile", {})
    fn = st.get("look_pre")
    if fn is not None:
        ex._pre[:] = [(f, n) for f, n in ex._pre if f is not fn]
        ex._pre.insert(0, (fn, None))
    last_post(ex)


def last_post(ex):
    fn = ex.state.get("mobile", {}).get("post")
    if fn is not None:
        ex._post[:] = [(f, n) for f, n in ex._post if f is not fn]
        ex._post.append((fn, None))


def look_ids(value):
    if isinstance(value, (str, int)) and not isinstance(value, bool):
        key = str(value)
        return ["L" + key if key.isdigit() else key]
    if isinstance(value, list):
        return [key for v in value for key in look_ids(v)]
    return []


def old_refusal(ids):
    which = ", ".join(ids)
    return {"said": f"Refused: look {which} was taken before the base or torso last moved, so its view no longer "
                    "matches the robot frame.",
            **refusal("look_before_drive", "look " + which +
                      " was taken before the base or torso last moved",
                      "take a new look; points located before the move are already carried over in robot://world")}


def look_pre(ex, name, kwargs):
    # Later packages may measure points or add look ids in their post hooks.
    # Move our maintenance last before post iteration starts.
    last_post(ex)
    st = ex.state["mobile"]
    for arg in LOOK_ARGS:
        old = [key for key in look_ids(kwargs.get(arg)) if key in st["old_looks"]]
        if old:
            return old_refusal(old)
    if name == "triangulate":
        label = str(kwargs.get("label"))
        if label in st["old_marks"] and len(ex._marks.get(label, {})) < 2:
            return old_refusal(st["old_marks"][label])
    return None


def collect_result(st, value):
    if isinstance(value, dict):
        st["seen_looks"].update(look_ids(value.get("look_id")))
        for v in value.values():
            if isinstance(v, (dict, list)):
                collect_result(st, v)
    elif isinstance(value, list):
        for v in value:
            collect_result(st, v)


def fresh_record(p):
    return {"point": list(p), "moved": 0.0, "turned": 0.0, "terms": 0.0}


def post(ex, name, kwargs, result):
    st = ex.state["mobile"]
    collect_result(st, result)
    for eid, e in ex.world.entries.items():
        p = e.get("point_mm")
        if e["kind"] not in POSITIONAL or e.get("stale") or p is None:
            continue
        old = st["entries"].get(eid)
        if old is not None and not np.array_equal(p, old["point"]) and (e.get("data") or {}).get("odometry"):
            moved_m = old["moved"] / 1000
            if moved_m > 0:
                drift = float(np.linalg.norm(np.asarray(p)-np.asarray(old["point"])) / 1000 / moved_m)
                st["know"]["drift"] = max(st["know"].get("drift") or 0, drift)
            ex.world.update(eid, data={"odometry": None})
        if old is None or not np.array_equal(p, old["point"]):
            st["entries"][eid] = fresh_record(p)
    if name in ("drive", "torso", "base_pose") and isinstance(result, dict) and "said" not in result:
        why = (result.get("refused") or {}).get("why") or "another rule of this robot refused it"
        return {"said": f"Refused, nothing moved: {why}.", **result}
    return None


def floor(ex, odo):
    info = ex.state["mobile"]["info"]
    torso = info["torso"]
    drop = 0.0 if torso is None else (odo["torso_m"] - torso["range_m"][0]) * torso["axis_b"][2]
    z = float(info["floor_z_b_at_torso0"] - drop)
    ex.state["mobile"]["floor_z_m"] = z
    D.of(ex).space.support(plane=((0,0,1),z), sigma=0, source="model")


def survey(ex, surface=None):
    st = ex.state.get("mobile", {})
    if st.get("info") is None:
        return
    names = set(st["look_tools"])
    for name in ex.primitive_names():
        sig = inspect.signature(getattr(ex, name))
        if any(k in sig.parameters for k in LOOK_ARGS) or any(
                p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values()):
            names.add(name)
    for tool in (surface or {}).get("tools", []):
        if any(k in tool.get("args", {}).get("properties", {}) for k in LOOK_ARGS):
            names.add(tool["name"])
    st["look_tools"] = sorted(names)


def moved_vectors(T, doc, direction_keys, point_keys):
    """{key: new value} for every numeric 3-vector of doc under those keys (directions rotated, points in mm
    moved); anything else (a direction name, a missing key) is left out."""
    out = {}
    for key in direction_keys + point_keys:
        try:
            a = np.asarray(doc.get(key), dtype=float)
        except (TypeError, ValueError):
            continue
        if a.shape == (3,) and np.all(np.isfinite(a)):
            out[key] = (G.rotate(T, a) if key in direction_keys else G.apply_mm(T, a)).tolist()
    return out


def documented_private(ex, T):
    # witness's own module docstring specifies acts [{..., point_mm, ...}].
    w = ex.state.get("witness")
    acts = w.get("acts") if isinstance(w, dict) else None
    for act in acts if isinstance(acts, list) else []:
        if isinstance(act, dict):
            act.update(moved_vectors(T, act, (), ("point_mm",)))
    # probe's own module docstring specifies its fit (axis_dir, center_mm, motion_dir, ...) in fits[label].
    fits = (ex.state.get("probe") or {}).get("fits") if isinstance(ex.state.get("probe"), dict) else None
    for fit in fits.values() if isinstance(fits, dict) else []:
        if isinstance(fit, dict):
            fit.update(moved_vectors(T, fit, FIT_DIRECTION_KEYS, FIT_POINT_KEYS))


def moved(ex, name, odo0, odo1, travel, moved_doc, pose_doc):
    st = ex.state["mobile"]
    info = st["info"]
    T = G.T_new_old(odo0, odo1, info)
    Tb = G.T_base_B(info, odo0["torso_m"])
    ids = []
    for eid, e in list(ex.world.entries.items()):
        p = e.get("point_mm")
        if e["kind"] not in POSITIONAL or e.get("stale") or p is None:
            continue
        record = st["entries"].get(eid)
        if record is None or not np.array_equal(p, record["point"]):
            record = fresh_record(p)
        radius_mm = float(np.linalg.norm(G.apply_mm(Tb, p)[:2]))
        record["moved"] += travel["distance_mm"]
        record["turned"] += np.rad2deg(travel["turn_rad"])
        record["terms"] += travel["turn_rad"] * radius_mm
        data = moved_vectors(T, e.get("data") or {}, tuple(DIRECTION_KEYS) + FIT_DIRECTION_KEYS,
                             tuple(POINT_DATA_KEYS) + FIT_POINT_KEYS)
        data["odometry"] = {"moved_mm": round(record["moved"], 1),
                            "turned_deg": round(float(record["turned"]), 2),
                            "est_error_mm": D.est_error(st["know"],record["moved"]/1000,record["terms"]/1000)[0]*1000,
                            "how": HOW + "; " + D.est_error(st["know"],record["moved"]/1000,record["terms"]/1000)[1]}
        pnew = G.apply_mm(T, p).tolist()
        written = ex.world.update(eid, point_mm=pnew, data=data)
        if "refused" in written:          # World refused the point (not finite): the entry keeps its old one
            continue
        # Remember World's normalized point, including its storage rounding.
        record["point"] = list(written["point_mm"])
        st["entries"][eid] = record
        ids.append(eid)
    if any(travel.values()):
        D.of(ex).space.drop('base_driven')
    floor(ex, odo1)
    st["old_looks"].update(str(k) for k in ex._looks)
    st["old_looks"].update(st["seen_looks"])
    for value in ex.state.values():
        if isinstance(value, dict) and isinstance(value.get("looks"), dict):
            st["old_looks"].update(str(k) for k in value["looks"])
    for label, marks in list(ex._marks.items()):
        old = [k for k in marks if str(k) in st["old_looks"]]
        if old:
            st["old_marks"][label] = sorted(set(st["old_marks"].get(label, [])) | set(old))
            for key in old:
                del marks[key]
        if not marks:
            del ex._marks[label]
    ex._history = []
    ex._std_ref = ex._std_last = None
    documented_private(ex, T)
    st["driven"]["distance_mm"] += travel["distance_mm"]
    st["driven"]["turn_deg"] += float(np.rad2deg(travel["turn_rad"]))
    st["driven"]["torso_mm"] += travel["torso_mm"]
    Tmm = T.copy()
    Tmm[:3, 3] *= 1000.0
    fields = {"primitive": name, "moved": moved_doc, "pose": pose_doc,
              "T_new_old_mm": Tmm.tolist(), "records_moved": len(ids), "ids": ids}
    ex.world.event("base_driven", data=fields, source="mobile")
    ex.note_event("base_driven", **fields)
    first_hook(ex)
    return len(ids)
