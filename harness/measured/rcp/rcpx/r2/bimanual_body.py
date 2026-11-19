# SPDX-License-Identifier: AGPL-3.0-only
"""The body of a robot with several arms (rcpx/r2/bimanual.py): robot://body and robot://cameras.

robot://body = {"arms": [{"name", "base": the arm's frame B in frame R (position_mm, R, x_axis, z_axis),
"wrist_camera": the name of the camera that rides on it, "body": the arm's own body document (the round one
describe() with the installation, and what packages add to it)}], "frame_R", "fixed_cameras": [each fixed camera's
size, lens and pose in frame R], "between_arms": the clearance margins and the distance between every two bases,
"how": how the tools address the arms}. Every number comes from the robot (its model and installation) or from
the session's own settings; nothing here names a robot.

robot://cameras = {"frame_R", "cameras": every camera once: a wrist camera with the arm it rides on and that arm's
own wrist entry; a fixed camera with its pose in frame R and in each arm's frame B (what look(camera=NAME) through
that arm uses)}.
"""

import numpy as np
from rcpx.r2 import bimanual_bounds as D

MM = 1000.0
FRAME_R = ("R: the robot's own frame, the one every arm base is installed in (mm, z up). Each arm's tools take and "
           "give positions in that arm's own frame B; robot://world shows stored points in R and in each arm's B")
HOW = ("Every tool that moves or looks through an arm takes arm (one of the names below). move_arms moves several "
       "arms at once, home_arms takes every arm home, handover passes a held object between two arms.")


def _r(v, nd=3):
    return [round(float(x), nd) + 0.0 for x in np.asarray(v, dtype=float).reshape(-1)]


def _pose_R(T):
    R = np.asarray(T, dtype=float)[:3, :3]
    return {"position_mm": _r(np.asarray(T, dtype=float)[:3, 3] * MM, 1), "looks_along": _r(-R[:, 2], 4),
            "image_right": _r(R[:, 0], 4), "image_down": _r(-R[:, 1], 4)}


def fixed_cameras(bm):
    out = []
    for name in bm.fixed_cameras:
        info = dict(bm.multi.camera_info(name))
        row = {k: info.get(k) for k in ("width", "height", "fovy_deg", "fx", "fy", "cx", "cy")}
        row.update(name=name, fixed=True, pose_R=_pose_R(bm.multi.T_R_cam(name)),
                   model=info.get("model", "pinhole, no distortion; u right, v down, origin top left"))
        out.append(row)
    return out


def body_doc(bm, arm_docs):
    """robot://body (module docstring). arm_docs {arm: that arm's robot://body content}."""
    arms = []
    for a in bm.arms:
        arms.append({"name": a, "base": dict(bm.frames.base_doc(a), frame="R"),
                     "wrist_camera": bm.views[a].wrist_name, "body": arm_docs.get(a)})
    dist = {}
    for i, a in enumerate(bm.arms):
        for b in bm.arms[i + 1:]:
            pa, pb = bm.frames.T_R_B(a)[:3, 3], bm.frames.T_R_B(b)[:3, 3]
            dist[f"{a} to {b}"] = round(float(np.linalg.norm(pa - pb)) * MM, 1)
    margins = {}
    bm.clear.refresh()
    for a in bm.arms:
        d, na, b, nb = bm.clear.now(a)
        if na is not None:
            va = bm.clear.live[a]["speeds"].get(na, 0)
            vb = bm.clear.live[b]["speeds"].get(nb, 0)
            margins[a] = {"stop_mm": bm.clear.margin(a,na,b,nb,va,vb)[0]*MM,
                          "clear_mm": bm.clear.margin(a,na,b,nb,va,vb,True)[0]*MM}
    return {"arms": arms, "frame_R": FRAME_R, "fixed_cameras": fixed_cameras(bm),
            "between_arms": {"by_arm": margins,
                             "base_distance_mm": dist,
                             "note": "a planned move that brings two arms nearer than clear_mm is refused (check "
                                     "arm_clearance); a move under way stops (stopped arm_clearance) when they come "
                                     "nearer than stop_mm"},
            "how": HOW}


def cameras_doc(bm, arm_docs):
    """robot://cameras (module docstring). arm_docs {arm: that arm's robot://cameras content}."""
    out = []
    for a in bm.arms:
        doc = arm_docs.get(a) if isinstance(arm_docs.get(a), dict) else {}
        wrist = next((c for c in doc.get("cameras", []) if not c.get("fixed")), None)
        if bm.views[a].wrist_name is not None:
            row = dict(wrist or {}, name=bm.views[a].wrist_name, arm=a, fixed=False)
            row["how"] = f"look(arm={a}) takes it (camera wrist, or this name)"
            out.append(row)
    for cam in fixed_cameras(bm):
        cam = dict(cam)
        cam["pose_in"] = {}
        for a in bm.arms:
            T = bm.frames.T_B_R(a) @ np.asarray(bm.multi.T_R_cam(cam["name"]), dtype=float)
            cam["pose_in"][a] = _pose_R(T)
        cam["how"] = f"look(arm=ARM, camera={cam['name']}) takes it and marks that arm's tip in the picture"
        out.append(cam)
    return {"frame_R": FRAME_R, "cameras": out,
            "note": "pose_R is in frame R; pose_in gives a fixed camera in each arm's frame B"}
