# SPDX-License-Identifier: AGPL-3.0-only
"""Where locate and view put the camera (package sight): camera poses solved on the body model, and
the dry checks every pose and joint path passes before anything moves. Nothing here moves the arm.

- The camera rides on the hand, so a wanted camera pose is a tip pose through the fixed camera-to-tip transform read
  off the body model (cam_offset). solve_view solves it on ex.model: an arm of six joints or more tries ROLLS_TRIED
  rolls about the optical axis, the most upright first, and keeps the first that solves; an arm of five joints (or
  one whose roll does not solve) solves the position and the optical axis only, so it keeps the roll its joints give
  (the arm refuses roll and yaw).
- Upright: image up as near world up as the axis allows; for an axis within UPRIGHT_VERTICAL_DEG of vertical (a top
  view) world up has no image direction, and "away from the robot" (side_words.radial) stands in for it.
- The dry checks (checks): the joint path passes the executor's own path check (path_plane, path_self), every
  link and the tool keep the bounds' clearance for a path, at the speed the check solves (rcpx/r2/sight_path.py),
  from every point robot://world holds and every survey box
  sight keeps (a part that starts nearer may not come nearer), the camera's line to the target is not crossed by the
  arm's own links, and the target is not inside the frame the hand hides.
- locate_views: the extra views of locate step 6 (offsets on a sphere about the bracket midpoint, ordered so the
  epipolar line crosses the patch's edge), and the first pair whose rays span enough to place the pointed patch
  (span_min_deg).
- side_pose: view's pose search (and plan_camera's, for act's witness views).

No robot or device is named. The numbers named below are solver and image numerics; every clearance, speed and
span is the session's (rcpx.bounds through rcpx/r2/sight_bounds.py and sight_path.py).
"""

import math

import mujoco
import numpy as np

from rcpx import bounds_ratios as R
from rcpx import r3_limits as L
from rcpx import side_words
from rcpx.r2 import sight_bounds as SB
from rcpx.r2.sight_path import (_parts, _seg_box, _seg_point, _seg_seg, clearances, each_clearance,  # noqa: F401
                                floor_clear_m, known_refs, path_check, tool_span_m)

MM = 1000.0
ROT_W = 0.15                 # m per rad: an orientation error against a position error in a solve (as RobotModel)
SOLVE_ITERS = 90             # damped least squares iterations per seed
SOLVE_POS_MM = 1.0           # a camera pose solves when the camera is within this ...
SOLVE_DEG = 0.6              # ... and its axis (and, for a six joint solve, its whole turn) within this
STEP_MAX_RAD = 0.25          # one solve step moves no joint further (keeps the linearisation valid)
UPRIGHT_VERTICAL_DEG = 15.0  # an axis this near vertical is a top view: "upright" is away from the robot
BRACKET_SAMPLES = 9          # points of the bracket projected for the frame check
COVER_FROM = 0.25            # the frame keeps the bracket from this share of its length on (see locate_views)
B_COVER_M = 0.05             # view B keeps the naming ray this far each side of view A's point in its frame
CORNER_COHERENCE = 0.5       # a patch whose structure tensor coherence is under this has edges both ways (a corner)


def _unit(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def _skew(r):
    return np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])


def _rotvec(R):
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = math.sin(a)
    if abs(s) < 1e-6:            # a half turn: the axis from the symmetric part
        M = (R + np.eye(3)) / 2.0
        k = int(np.argmax(np.diag(M)))
        ax = _unit(M[:, k])
        return ax * a
    return w * (a / (2.0 * s))


def cam_pose(model, q):
    pc, Rc = model.fk(np.asarray(q, dtype=float))["camera"]
    return np.asarray(pc, dtype=float), np.asarray(Rc, dtype=float)


def look_rot(axis, up):
    """The camera rotation (columns: image right, image up, back along the view) that looks along axis with image up
    nearest up (a unit vector not along the axis)."""
    a = _unit(axis)
    u = np.asarray(up, dtype=float) - float(np.dot(up, a)) * a
    if float(np.linalg.norm(u)) < 1e-6:
        u = np.array([1.0, 0.0, 0.0]) - a[0] * a
    u = _unit(u)
    return np.column_stack([np.cross(a, u), u, -a])


def upright(axis, target_m):
    """The image up an upright view of target_m along axis has: world up, or away from the robot for a top view."""
    a = _unit(axis)
    if abs(float(a[2])) >= math.cos(math.radians(UPRIGHT_VERTICAL_DEG)):
        return side_words.radial(np.asarray(target_m, dtype=float) * MM)
    return np.array([0.0, 0.0, 1.0])


def roll_deg(Rc, up):
    """Signed angle (deg) from the image up of camera rotation Rc to `up` projected on the image plane."""
    a = -Rc[:, 2]
    u = np.asarray(up, dtype=float) - float(np.dot(up, a)) * a
    if float(np.linalg.norm(u)) < 1e-9:
        return 0.0
    u = _unit(u)
    y = Rc[:, 1]
    return math.degrees(math.atan2(float(np.dot(np.cross(y, u), a)), float(np.dot(y, u))))


def _jac_cam(model, q):
    """(J (6 x n) of the camera point and the hand's turn, camera position, camera rotation) at q."""
    J = np.asarray(model.jac(q), dtype=float)
    fk = model.fk(q)
    pt = np.asarray(fk["tip"][0], dtype=float)
    pc, Rc = np.asarray(fk["camera"][0], dtype=float), np.asarray(fk["camera"][1], dtype=float)
    Jp = J[:3] - _skew(pc - pt) @ J[3:]
    return np.vstack([Jp, J[3:]]), pc, Rc


def _solve(model, q0, pc_want, Rc_want=None, axis_want=None):
    """Damped least squares from q0 toward the camera at pc_want with the whole turn Rc_want (six rows) or only its
    optical axis axis_want (five: the roll stays free). Returns (q, pos_err_mm, ang_err_deg)."""
    q = model.clip(np.asarray(q0, dtype=float))
    pe, ae = float("inf"), float("inf")
    for _ in range(SOLVE_ITERS):
        J, pc, Rc = _jac_cam(model, q)
        ep = np.asarray(pc_want, dtype=float) - pc
        if Rc_want is not None:
            er = _rotvec(np.asarray(Rc_want) @ Rc.T)
            rows = J[3:]
        else:
            ac = -Rc[:, 2]
            er = np.cross(ac, _unit(axis_want))
            P = np.eye(3) - np.outer(ac, ac)
            rows = P @ J[3:]
        pe, ae = float(np.linalg.norm(ep)) * MM, math.degrees(float(np.linalg.norm(er)))
        if pe < SOLVE_POS_MM and ae < SOLVE_DEG:
            break
        e = np.concatenate([ep, ROT_W * er])
        Js = np.vstack([J[:3], ROT_W * rows])
        dq = model._dls(Js, e, q)
        big = float(np.max(np.abs(dq))) if dq.size else 0.0
        if big > STEP_MAX_RAD:
            dq *= STEP_MAX_RAD / big
        q = model.clip(q + dq)
    return q, pe, ae


def solve_view(model, seeds, pc_want, axis, target_m):
    """The joint pose that puts the camera at pc_want looking along axis, the most upright roll that solves: {q,
    roll_deg, how} or None. seeds: start poses in order (the present pose first). An arm that cannot hold the asked
    pose (five joints) keeps the nearest pose it holds when that is the asked view to within the relative error the
    laws accept (REL): the camera within REL of its distance from the target, its axis within REL rad, and the
    target still inside the central FRAME_KEEP_FRAC of the frame; the checks that follow read the pose held."""
    up = upright(axis, target_m)
    q5 = None
    for q0 in seeds:                       # the position and the axis first: a near start for every roll below
        q, pe, ae = _solve(model, q0, pc_want, axis_want=axis)
        if model.within_limits(q) and pe < SOLVE_POS_MM and ae < SOLVE_DEG:
            q5 = q
            break
    if model.n >= 6:
        R0 = look_rot(axis, up)
        a = _unit(axis)
        rolls = [0.0]
        step = 360.0 / L.ROLLS_TRIED
        for k in range(1, L.ROLLS_TRIED // 2 + 1):
            rolls += [k * step, -k * step] if k * step < 180.0 else [180.0]
        for r in rolls:
            c, s = math.cos(math.radians(r)), math.sin(math.radians(r))
            K = _skew(a)
            Rr = np.eye(3) + s * K + (1.0 - c) * (K @ K)       # turn by r about the axis
            Rw = Rr @ R0
            for q0 in ([q5] if q5 is not None else []) + list(seeds):
                q, pe, ae = _solve(model, q0, pc_want, Rc_want=Rw)
                if pe < SOLVE_POS_MM and ae < SOLVE_DEG and model.within_limits(q):
                    return {"q": q, "roll_deg": round(roll_deg(cam_pose(model, q)[1], up), 1),
                            "how": "camera pose solved with the most upright roll that the arm reaches"}
    best = None
    for q0 in seeds:
        q, pe, ae = _solve(model, q0, pc_want, axis_want=axis)
        if not model.within_limits(q):
            continue
        if pe < SOLVE_POS_MM and ae < SOLVE_DEG:
            return {"q": q, "roll_deg": round(roll_deg(cam_pose(model, q)[1], up), 1),
                    "how": "camera position and axis solved; the roll is the one the joints give"}
        if best is None or pe + ae < best[1] + best[2]:
            best = (q, pe, ae)
    if best is not None:            # an arm that cannot hold every camera pose (five joints): the nearest it holds
        q, pe, ae = best
        away = float(np.linalg.norm(np.asarray(pc_want, dtype=float) - np.asarray(target_m, dtype=float)))
        if pe / MM <= R.REL * away and math.radians(ae) <= R.REL and in_frame(model, q, [target_m]):
            return {"q": q, "roll_deg": round(roll_deg(cam_pose(model, q)[1], up), 1),
                    "how": f"the nearest camera pose the joints hold ({pe:.0f} mm and {ae:.0f} deg from the asked "
                           "one)"}
    return None


def rot_between(a, b):
    """The smallest turn (3 x 3) that takes unit direction a onto unit direction b."""
    a, b = _unit(a), _unit(b)
    v = np.cross(a, b)
    s, c = float(np.linalg.norm(v)), float(a @ b)
    if s < 1e-9:
        return np.eye(3)            # the same direction (opposite ones do not occur between views a few deg apart)
    K = _skew(v / s)
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def solve_like(model, seeds, pc_want, Rc_want):
    """The joint pose that puts the camera at pc_want turned as Rc_want (an arm of six joints or more: the whole
    turn; five joints: the position and the optical axis, the roll what the joints give): {q, how} or None."""
    for q0 in seeds:
        if model.n >= 6:
            q, pe, ae = _solve(model, q0, pc_want, Rc_want=Rc_want)
        else:
            q, pe, ae = _solve(model, q0, pc_want, axis_want=-np.asarray(Rc_want)[:, 2])
        if pe < SOLVE_POS_MM and ae < SOLVE_DEG and model.within_limits(q):
            return {"q": q, "how": "the naming camera carried round the point without turning about its own axis"}
    return None


# ---------------------------------------------------------------- what the world holds (for the clearances)
def known(ex):
    """(points (k, 3) m: stored contacts, located points and device entries with a point, stale ones too; boxes:
    [(lo (3,), hi (3,), id)] m from the survey candidates sight keeps)."""
    pts = []
    for kind in ("contact", "located", "device"):
        for e in ex.world.query(kind=kind, include_stale=True):
            if e.get("point_mm") is not None:
                pts.append(np.asarray(e["point_mm"], dtype=float) / MM)
    boxes = []
    st = ex.state.get("sight") or {}
    for c in st.get("candidates") or []:
        b = box_of(c)
        if b is not None:
            boxes.append(b)
    return np.asarray(pts, dtype=float).reshape(-1, 3), boxes


def box_of(c):
    """(lo, hi, id) in m of a survey candidate {where_mm, size_mm, top_mm, floor_z_mm}; None without its numbers."""
    try:
        x, y = (float(v) for v in c["where_mm"][:2])
        sx, sy = (float(v) for v in c["size_mm"][:2])
        top, floor = float(c["top_mm"]), float(c.get("floor_z_mm") or 0.0)
    except (KeyError, TypeError, ValueError):
        return None
    lo = np.array([x - sx / 2.0, y - sy / 2.0, min(floor, top)]) / MM
    hi = np.array([x + sx / 2.0, y + sy / 2.0, max(floor, top)]) / MM
    return lo, hi, c.get("id")



def in_frame(model, q, pts_m, frac=L.FRAME_KEEP_FRAC):
    """True when every point projects in front of the camera and inside the central frac of the frame."""
    W, H = float(model.cam_w), float(model.cam_h)
    mu, mv = 0.5 * (1.0 - frac) * W, 0.5 * (1.0 - frac) * H
    for p in pts_m:
        u, v, dep = model.project(q, p)
        if not (dep > 1e-3 and mu <= u <= W - mu and mv <= v <= H - mv):
            return False
    return True


_RAYD = {}


def robot_hides(model, q, p_m):
    """The name of the robot's own body that the camera's line to p_m meets first at pose q (the robot-only model's
    own geometry, every link and the hand), or None when the line is clear."""
    m = model.mj
    key = id(m)
    if key not in _RAYD:
        qadr = np.array([m.jnt_qposadr[m.joint(n).id] for n in model.joint_names])
        _RAYD[key] = (mujoco.MjData(m), qadr)
    d, qadr = _RAYD[key]
    d.qpos[:] = m.qpos0
    d.qpos[qadr] = np.asarray(q, dtype=float)
    mujoco.mj_kinematics(m, d)
    pc = cam_pose(model, q)[0]
    v = np.asarray(p_m, dtype=float) - pc
    dist = float(np.linalg.norm(v))
    if dist < 1e-6:
        return None
    gid = np.zeros(1, np.int32)
    hit = mujoco.mj_ray(m, d, pc, v / dist, None, 1, -1, gid)
    if gid[0] < 0 or hit < 0 or hit >= dist:
        return None
    return m.body(int(m.geom_bodyid[int(gid[0])])).name


def hand_hides(ex, q, p_m):
    """True when the point is behind the camera, or the robot's own geometry stands on the camera's line to it."""
    u, v, dep = ex.model.project(q, p_m)
    if not dep > 1e-3:
        return True
    return robot_hides(ex.model, q, p_m) is not None


def line_blocked(ex, q, p_m):
    """The name of the robot's own body on the camera's line to p_m, else None (robot_hides)."""
    return robot_hides(ex.model, q, p_m)


def _sph(az, el):
    return np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])


def epi_dir(model, q_look, cam_k, px):
    """The direction (unit, pixels) of the epipolar line through px in the naming image for a camera at cam_k."""
    pc, Rc = cam_pose(model, q_look)
    b = Rc.T @ (np.asarray(cam_k, dtype=float) - pc)
    f, cx, cy = model.f_px, model.cam_w / 2.0, model.cam_h / 2.0
    u, v = float(px[0]), float(px[1])
    d = np.array([f * b[0] + (cx - u) * (-b[2]), -f * b[1] + (cy - v) * (-b[2])])
    return _unit(d) if float(np.linalg.norm(d)) > 1e-9 else np.array([1.0, 0.0])


def ring(ex, q_look, q_now, centre, cover, grad_dir, px, notes, rigid=False):
    """The feasible extra views about `centre` (m) at the naming camera's distance: the offsets of AZ_OFFSETS_DEG
    and EL_OFFSETS_DEG (true angles at the centre; the azimuth turns about the vertical, or about image up for a top
    view), at most VIEW_FROM_OP_MAX_DEG from the naming view, ordered by |cos| of the angle between the epipolar line
    through px and the patch's dominant gradient (an edge is crossed, not slid along), then by the smaller offset.
    Each kept view solves, stands the camera body's radius and the support's error band over the support
    (sight_path.floor_clear_m), keeps the points `cover` in the central FRAME_KEEP_FRAC and sees the centre past the
    arm. rigid (the robot's body sheet states a camera pose uncertainty): the naming camera is carried round the
    centre without turning about its own axis where the arm can, so a camera that sits off its modelled place errs
    the same way in every view and the error drops out of the depth (measured: a 1.5 deg camera error turned with
    the hand between upright views and put the depth 12 mm off; carried, 1.5 mm); without it the views are
    upright (solve_view). [view {q, cam_m, axis, w, az_deg, el_deg, off_deg, score, roll_deg, how}]."""
    m = ex.model
    floor_min = float(ex.cfg.get("floor_z_m") or 0.0) + floor_clear_m(ex, q_now)
    C0 = cam_pose(m, q_look)[0]
    away = float(np.linalg.norm(C0 - centre))
    w0 = _unit(C0 - centre)
    e_az = np.cross([0.0, 0.0, 1.0], w0)
    if float(np.linalg.norm(e_az)) < 0.2:          # a top view: the turn "about the vertical" is about image up
        x = cam_pose(m, q_look)[1][:, 0]
        e_az = x - float(x @ w0) * w0
    e_az = _unit(e_az)
    e_el = _unit(np.cross(w0, e_az))
    if float(e_el[2]) < 0.0:
        e_el = -e_el
    offs = [(s * a, 0.0) for a in L.AZ_OFFSETS_DEG for s in (1, -1)] + \
           [(0.0, s * e) for e in L.EL_OFFSETS_DEG for s in (1, -1)]
    cands = []
    g = np.asarray(grad_dir, dtype=float)
    for daz, dele in offs:
        ang = math.radians(daz if daz else dele)
        w = _unit(math.cos(ang) * w0 + math.sin(ang) * (e_az if daz else e_el))
        off = _ang(w, w0)
        if off > L.VIEW_FROM_OP_MAX_DEG + 1e-6:
            continue
        cam = centre + away * w
        if cam[2] < floor_min:
            continue
        score = abs(float(epi_dir(m, q_look, cam, px) @ g)) if float(np.linalg.norm(g)) > 0 else 0.0
        cands.append({"cam_m": cam, "axis": -w, "w": w, "az_deg": daz, "el_deg": dele, "off_deg": round(off, 1),
                      "score": round(score, 3)})
    cands.sort(key=lambda c: (-c["score"], abs(c["az_deg"]) + abs(c["el_deg"])))
    ok = []
    seeds = [np.asarray(q_now, dtype=float), np.asarray(q_look, dtype=float), m.q_home.copy()]
    R0 = cam_pose(m, q_look)[1]
    up = upright(-w0, centre)
    for c in cands:
        tag = f"az {c['az_deg']:+g} el {c['el_deg']:+g}"
        sd = seeds + [x["q"] for x in ok[-2:]]
        # rigid: 1 the naming camera carried round the centre as one rigid body (the part keeps its pixel and its
        #   turn in the picture); 2 the same turn with the centre brought to the middle of the frame; then, and
        #   without rigid at once, 3 an upright view (solve_view). The first that passes the checks is kept.
        tries = ((lambda: solve_like(m, sd, c["cam_m"], rot_between(w0, c["w"]) @ R0),
                  lambda: solve_like(m, sd, c["cam_m"], rot_between(-R0[:, 2], -c["w"]) @ R0)) if rigid else ()) + \
            (lambda: solve_view(m, sd, c["cam_m"], c["axis"], centre),)
        why = "no joint pose"
        for k, solve in enumerate(tries):
            sol = solve()
            if sol is None:
                continue
            q = sol["q"]
            held = cam_pose(m, q)[0]             # the camera position the pose really holds
            if held[2] < floor_min:
                why = "the camera's held pose is too near the support"
                continue
            if not in_frame(m, q, cover):
                why = "the line leaves the frame"
                continue
            if hand_hides(ex, q, centre) or line_blocked(ex, q, centre):
                why = "the arm hides the line"
                continue
            c.update(q=q, cam_m=held, roll_deg=sol.get("roll_deg", round(roll_deg(cam_pose(m, q)[1], up), 1)),
                     how=sol["how"], carried=bool(rigid) and k < 2)
            ok.append(c)
            break
        else:
            notes.append(f"{tag}: {why}")
    return ok


def locate_views(ex, q_look, q_now, o, d, t_near, t_far, grad_dir, px, patch_px=None):
    """The two extra views of locate. o, d: the naming ray (m, unit); [t_near, t_far] its
    bracket (m along d). Returns {"pair": [view, view] or None, "why", "notes", "ranked", "ctx"}: the ring about the
    bracket midpoint (ring), and the first pair whose rays span span_min_deg there with every path passing (each
    path at the speed its check solves). patch_px: the side of the patch the operator pointed at. The frame
    check keeps the bracket from COVER_FROM of its length on in the central FRAME_KEEP_FRAC: the bracket's near end
    (60 mm from the camera) leaves every turned view, and a part that near would stand in front of the tool.
    Nothing moves."""
    m = ex.model
    o, d = np.asarray(o, dtype=float), _unit(d)
    M = o + d * 0.5 * (t_near + t_far)
    C0 = cam_pose(m, q_look)[0]
    t_cov = t_near + COVER_FROM * (t_far - t_near)
    cover = [o + d * t for t in np.linspace(t_cov, t_far, BRACKET_SAMPLES)]
    pts, boxes = known(ex)
    line = (o + d * t_cov, o + d * t_far)
    notes = []
    facts = SB.inputs(ex)
    rigid = float(facts.get("pu_deg") or 0.0) > 0.0 or float(facts.get("pu_mm") or 0.0) > 0.0
    ok = ring(ex, q_look, q_now, M, cover, grad_dir, px, notes, rigid=rigid)
    ctx = {"pts": pts, "boxes": boxes, "line": line, "q_now": np.asarray(q_now, dtype=float), "C0": C0, "rigid": rigid,
           "q_look": np.asarray(q_look, dtype=float), "grad_dir": grad_dir, "px": px, "o": o, "d": d,
           "t_near": t_near, "t_far": t_far, "patch_px": patch_px, "refs": known_refs(ex)}
    pair, why = None, ""
    for a in ok:
        info = {}
        bad = path_check(ex, q_now, a["q"], pts, boxes, line, info, ctx["refs"])
        if bad is None:                 # the arm stands at q_now: the gates can be asked of this leg before it runs
            gate = SB.gate(ex, a["q"], info.get("share"))[0]
            bad = None if gate is None else gate["refused"]
        if bad is not None:
            notes.append(f"az {a['az_deg']:+g} el {a['el_deg']:+g}: {bad.get('check')}: {bad.get('why', '')}")
            continue
        b = pick_b(ex, ok, a, M, ctx, notes)
        if b is not None:
            pair = [a, b]
            break
    if pair is None:
        why = "no second view reachable here" + (f" ({notes[0]})" if notes else "")
    return {"pair": pair, "why": why, "notes": notes[:6], "centre_m": M, "ranked": ok, "ctx": ctx}


def b_about(ex, a, P_est, ctx, exclude=(), same_side=False):
    """View B re-centred on the point view A's match gives (P_est on the naming ray): the ring about it (the same
    offsets, now true angles at the point), the first that spans span_min_deg with the naming view and view A at the
    point and whose paths pass; None when none does."""
    o, d = ctx["o"], ctx["d"]
    t = float((np.asarray(P_est) - o) @ d)
    near = [o + d * x for x in np.linspace(max(ctx["t_near"], t - B_COVER_M), min(ctx["t_far"], t + B_COVER_M), 5)]
    ranked = ring(ex, ctx["q_look"], a["q"], np.asarray(P_est, dtype=float), near, ctx["grad_dir"], ctx["px"], [],
                  rigid=bool(ctx.get("rigid")))
    C0, P_ = ctx["C0"], np.asarray(P_est, dtype=float)
    w0, wa = _unit(C0 - P_), _unit(a["cam_m"] - P_)

    corner = float(ctx.get("coherence", 1.0)) < CORNER_COHERENCE
    da = _unit(wa - w0)

    def key(b):
        wb = _unit(b["cam_m"] - P_)
        if corner:              # a corner-like patch: the turn across view A's checks the depth in the other axis
            return (round(abs(float(_unit(wb - w0) @ da)), 1), _ang(wb, w0))
        if same_side:           # further along view A's own turn first
            return (round(-float(_unit(wb - w0) @ da), 1), _ang(wb, w0))
        same = float((wb - w0) @ (wa - w0)) > 0.0
        return (same, _ang(wb, w0))
    # an edge-like patch: the other side of the naming view from view A first, with the smallest turn that spans
    # enough: the two extra views bracket the naming view, so each sees the part much as the operator's look did (a
    # view further out on A's side loses a part on a face as the view turns away from it, measured on
    # front-face buttons), and both turns cross the edge. A corner-like patch: the turn across view A's first, so a
    # junction of two edges at different depths (an occluding corner) fails the three-ray miss instead of agreeing
    # same_side (the replacement after a view on the other side lost the part): further out on view A's own side,
    # where the part was found, the smallest turn that still spans enough with the naming view
    ranked.sort(key=key)
    return pick_b(ex, ranked, a, P_est, ctx, exclude=exclude)


def span_at(P, cams):
    """The largest angle (deg) between the rays from the cameras to the point P."""
    dirs = [_unit(np.asarray(P, dtype=float) - np.asarray(c, dtype=float)) for c in cams]
    return max(_ang(a, b) for a in dirs for b in dirs)


def pick_b(ex, ranked, a, P_est, ctx, notes=None, exclude=()):
    """The second view: the first of the ranked views (not a) whose rays with the naming camera and view a span
    span_min_deg at the point P_est (the bracket midpoint before view a's match, the two-ray point after it), and
    whose paths (a to it, it back to the start) pass."""
    for b in ranked:
        if b is a or any(float(np.linalg.norm(b["cam_m"] - x["cam_m"])) < 0.01 for x in exclude):
            continue
        away = float(np.linalg.norm(np.asarray(P_est, dtype=float) - ctx["C0"]))
        if span_at(P_est, [ctx["C0"], a["cam_m"], b["cam_m"]]) < span_min_deg(ex, ctx.get("patch_px"), away) - 1e-6:
            continue
        bad = path_check(ex, a["q"], b["q"], ctx["pts"], ctx["boxes"], ctx["line"], refs=ctx.get("refs")) or \
            path_check(ex, b["q"], ctx["q_now"], ctx["pts"], ctx["boxes"], ctx["line"], refs=ctx.get("refs"))
        if bad is not None:
            if notes is not None:
                notes.append(f"az {b['az_deg']:+g} el {b['el_deg']:+g}: {bad['check']}: {bad['why']}")
            continue
        return b
    return None


def _ang(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(_unit(a), _unit(b)))))))


__all__ = ["b_about", "cam_pose", "clearances", "each_clearance", "epi_dir", "floor_clear_m", "hand_hides",
           "in_frame", "known", "known_refs", "line_blocked", "locate_views", "look_rot", "path_check", "pick_b",
           "ring", "robot_hides", "roll_deg", "rot_between", "solve_like", "solve_view", "span_at", "span_min_deg",
           "tool_span_m", "upright"]


def span_min_deg(ex, patch_px=None, range_m=None):
    """The least angle (deg) the rays of a pair of views must span: the round three limit SPAN_MIN_DEG as the
    session derives it. r3_limits.span_min_deg(B, patch_px=...) computes it once it exists: the depth error of two
    rays one match error apart must stay under half the footprint of the patch the operator pointed at (what the
    approach that follows needs: the point must lie on the patch), so sin(span) >= QUANT x QUANT x MATCH_ERR_PX /
    patch_px, whatever the range. Until that function exists the constant stands (sight_bounds.limit). patch_px
    None: the smallest patch locate uses."""
    return SB.limit(ex, "SPAN_MIN_DEG", patch_px=min(L.PATCH_SIZES_PX) if patch_px is None else patch_px)
