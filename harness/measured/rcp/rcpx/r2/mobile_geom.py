# SPDX-License-Identifier: AGPL-3.0-only
"""Pure frame, footprint, path, timing and stall arithmetic for mobile."""

import math
import numpy as np


def wrap(a):
    out = (float(a) + np.pi) % math.tau - np.pi
    return float(np.pi if out <= -np.pi else out)


def pose_compose(pose, forward_m, left_m, turn_rad):
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    return (float(x + c * forward_m - s * left_m),
            float(y + s * forward_m + c * left_m), float(yaw + turn_rad))


def quat_wxyz_to_R(q):
    q = np.asarray(q, dtype=float)
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def T_from(R, p):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def T_S_base(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return T_from(np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), [x, y, 0])


def T_base_B(info, torso_m):
    inst = info["b_in_base"]
    T = T_from(quat_wxyz_to_R(inst["quat_wxyz"]), inst["pos"])
    torso = info.get("torso")
    if torso is not None:
        rise = torso_m - torso["range_m"][0]
        T = T @ T_from(np.eye(3), np.asarray(torso["axis_b"]) * rise)
    return T


def T_S_B(odo, info):
    return T_S_base(odo["x_m"], odo["y_m"], odo["yaw_rad"]) @ T_base_B(info, odo["torso_m"])


def T_new_old(odo_old, odo_new, info):
    return np.linalg.inv(T_S_B(odo_new, info)) @ T_S_B(odo_old, info)


def apply_mm(T, p_mm):
    return T[:3, :3] @ np.asarray(p_mm, dtype=float) + T[:3, 3] * 1000.0


def rotate(T, v):
    return T[:3, :3] @ np.asarray(v, dtype=float)


def footprint_now(info, torso_m):
    fp = info["footprint_b"]
    torso = info.get("torso")
    shift = np.zeros(3) if torso is None else (
        np.asarray(torso["axis_b"]) * (torso_m - torso["range_m"][0]))
    return {"x": [float(v - shift[0]) for v in fp["x"]],
            "y": [float(v - shift[1]) for v in fp["y"]],
            "z_top": float(fp["z_top"] - shift[2])}


def rect_dist(px, py, x_rng, y_rng):
    dx = max(x_rng[0] - px, 0, px - x_rng[1])
    dy = max(y_rng[0] - py, 0, py - y_rng[1])
    return float(np.hypot(dx, dy))


def sweep_poses(forward_mm, left_mm, turn_deg, step_mm, step_deg):
    n = max(int(np.ceil(np.hypot(forward_mm, left_mm) / step_mm)),
            int(np.ceil(abs(turn_deg) / step_deg)), 1)
    return [(forward_mm * k / n / 1000.0, left_mm * k / n / 1000.0,
             float(np.deg2rad(turn_deg) * k / n)) for k in range(n + 1)]


def sweep_dist_mm(p_mm, poses, info, torso_m):
    Tb = T_base_B(info, torso_m)
    inv = np.linalg.inv(Tb)
    q = Tb @ np.r_[np.asarray(p_mm) / 1000.0, 1.0]
    fp = footprint_now(info, torso_m)
    distances = []
    for x, y, yaw in poses:
        r = inv @ np.linalg.inv(T_S_base(x, y, yaw)) @ q
        distances.append(rect_dist(r[0], r[1], fp["x"], fp["y"]) * 1000.0)
    return float(min(distances)), float(distances[0])


def fresh_points(entries):
    return [e for e in entries if e.get('kind') in ('contact','located','device')
            and not e.get('stale') and e.get('point_mm') is not None]


def path_check(entries, poses, info, torso_m, margin_fn, top_fn, slack_m):
    top = footprint_now(info,torso_m)['z_top']
    nearest = None
    for e in fresh_points(entries):
        p = e['point_mm']
        if p[2] / 1000 > top + top_fn(e):
            continue
        near,start = sweep_dist_mm(p,poses,info,torso_m)
        if nearest is None or near < nearest['distance_mm']:
            nearest = dict(e,distance_mm=near)
        if near / 1000 <= margin_fn(e) and near < start - slack_m * 1000:
            return dict(e,distance_mm=near), nearest
    return None, nearest


def point_segment_dist(p,a,b):
    d = b-a
    norm = float(d @ d)
    share = min(1,max(0,float((p-a) @ d)/norm)) if norm else 0
    return float(np.linalg.norm(p-a-d*share))


def arm_sweep(entries,capsules,poses,info,torso_m,margin_fn,slack_m):
    """The full arm follows every base pose. Heights are not filtered."""
    Tb = T_base_B(info,torso_m)
    nearest = None
    for e in fresh_points(entries):
        p = Tb[:3,:3] @ (np.asarray(e['point_mm']) / 1000) + Tb[:3,3]
        for name,a,b,r in capsules:
            a = Tb[:3,:3] @ a + Tb[:3,3]
            b = Tb[:3,:3] @ b + Tb[:3,3]
            start = point_segment_dist(p,a,b)-r
            for x,y,yaw in poses:
                T = T_S_base(x,y,yaw)
                d = point_segment_dist(p,T[:3,:3] @ a + T[:3,3],T[:3,:3] @ b + T[:3,3])-r
                rec = {'id':e['id'],'label':e['label'],'link':name,'distance_mm':d*1000}
                if nearest is None or d*1000 < nearest['distance_mm']:
                    nearest = rec
                if d < margin_fn(e,(x,y,yaw)) and d < start-slack_m:
                    return rec,nearest
    return None,nearest


def current_distance(e,pose,info,torso_m):
    return sweep_dist_mm(e['point_mm'],[pose],info,torso_m)[0] / 1000


def tip_out_mm(tip_mm, info, torso_m, margin_mm):
    fp = footprint_now(info, torso_m)
    x = [fp["x"][0] * 1000 - margin_mm, fp["x"][1] * 1000 + margin_mm]
    y = [fp["y"][0] * 1000 - margin_mm, fp["y"][1] * 1000 + margin_mm]
    return rect_dist(tip_mm[0], tip_mm[1], x, y)


def plan_periods(dist_m, turn_rad, v_mps, w_rps, dt):
    return max(int(np.ceil(dist_m / (v_mps * dt) - 1e-9)),
               int(np.ceil(abs(turn_rad) / (w_rps * dt) - 1e-9)), 1)

