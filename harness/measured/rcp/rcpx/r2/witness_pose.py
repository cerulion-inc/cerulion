# SPDX-License-Identifier: AGPL-3.0-only
"""Camera poses of the witness package (SPEC-R3.md 4.4): the same view, the robot's own body in a frame, where a
point or a survey candidate falls in a look, and the move that puts the camera back at a look's pose.

    cam_pose(model, q)                       (position m, rotation): the wrist camera at joint pose q (frame B)
    project(model, q, points_m)              (u, v, depth_m) arrays for frame-B points (depth <= 0: behind)
    same_view(model, q_a, q_b, tol)          {same, mm, deg, roll_deg, tol}: by FK at the stored (measured) q
    tolerance(ex, q)                         r3_limits.view_tol from robot://body's stated resolution at q
    own_mask(model, q, margin_px)            the pixels the robot's own capsules cover (grown by margin_px)
    candidate_box(model, q, cand, floor_z)   a survey candidate's box in pixels, or None
    candidate_mask(model, q, cand, floor_z)  the pixels inside the projected outline of that box
    targets(ex, at, q)                       compare's at as regions in a look (located and contact entries, part
                                             names, survey candidates, px or box), or a refusal unknown_target
    device_box(ex, q, t_after, points)       the acted device's region in a look, for device_moved
    go_to(ex, q_goal, tool)                  current dark frame, stored-point goal check, gate with one retreat
                                             and one retry, then guarded move and correctives (look like)
    settle(ex)                               two frames a frame interval apart agree, or the picture stops calming
                                             after the arm's own settle time
    go_and_look(ex, q, note)                 go_to, settle, then a look (for act's witness looks)

Correctives (look like on a real arm): when the camera did not come back to the same view, up to CORRECTIVES_MAX
more moves aim past the goal by the shortfall each joint showed (the lead accumulates over the moves and is clipped
to CORRECTIVE_LEAD_MULT times the largest shortfall that joint showed, so an ask never runs far past its goal); a
joint whose shortfall is one encoder step or less gets no lead. Every move is a guarded joint move (path checked,
at the session's derived speed, the contact stop armed) and is gated first (ex.gates("move_joints")), so a dark
reading or a path gate stops it with
that refusal. Before the first gate, a current wrist frame is read with the look's own light rule: dark refuses
without movement. Next, the modeled goal tip within ACT_NEAR_MIN_MM of any stored contact or located point,
stale included, refuses like_unreachable without movement, using the existing act-near limit and stored evidence.
A first non-dark gate refusal gets one retreat step if history exists, then one gate retry; another refusal (or
no history) is like_unreachable. A dark gate refusal makes no retreat. Nothing here reads ground truth or names a robot.
"""

import json
import math
import inspect

import cv2
import numpy as np

from rcpx import r3_limits as L
from rcpx import sheet_words
from rcpx import bounds_ratios as R
from rcpx.r2 import witness_bounds as WB
from rcpx.context import refusal
from rcpx.r2 import witness_diff as diff

MM = 1000.0
CAPSULE_SAMPLES = 9          # points along each robot capsule drawn into the own-body mask
OWN_MARGIN_PX = 2.0          # the own-body mask is grown this much past the capsules' outline ...


def cam_pose(model, q):
    pc, Rc = model.fk(np.asarray(q, dtype=float))["camera"]
    return np.asarray(pc, dtype=float).copy(), np.asarray(Rc, dtype=float).copy()


def project_pose(pc, Rc, f, w, h, points_m):
    """(u, v, depth) arrays for frame-B points through a pinhole camera at (pc, Rc) (OpenGL axes: it looks along
    -z, y up), as RobotModel.project does."""
    P = np.atleast_2d(np.asarray(points_m, dtype=float))
    c = (P - pc) @ Rc
    depth = -c[:, 2]
    ok = depth > 1e-9
    u = np.full(len(P), np.nan)
    v = np.full(len(P), np.nan)
    u[ok] = w / 2.0 + f * c[ok, 0] / depth[ok]
    v[ok] = h / 2.0 - f * c[ok, 1] / depth[ok]
    return u, v, depth


def project(model, q, points_m):
    pc, Rc = cam_pose(model, q)
    return project_pose(pc, Rc, float(model.f_px), int(model.cam_w), int(model.cam_h), points_m)


def _roll_deg(Ra, Rb):
    """The turn about the optical axis between two cameras: b's image right, carried onto a's axis by the
    smallest rotation, measured against a's image right (signed, degrees)."""
    za, zb = -Ra[:, 2], -Rb[:, 2]
    xb = Rb[:, 0]
    axis = np.cross(zb, za)
    s, c = float(np.linalg.norm(axis)), float(np.dot(zb, za))
    if s > 1e-12:
        k = axis / s
        ang = math.atan2(s, c)
        xb = xb * math.cos(ang) + np.cross(k, xb) * math.sin(ang) + k * np.dot(k, xb) * (1 - math.cos(ang))
    xa = Ra[:, 0]
    return math.degrees(math.atan2(float(np.dot(np.cross(xa, xb), za)), float(np.dot(xa, xb))))


def view_numbers(model, q_a, q_b):
    """{mm, deg, roll_deg}: camera distance, optical axis angle and roll between two joint poses (by FK)."""
    return pose_numbers(cam_pose(model, q_a), cam_pose(model, q_b))


def pose_numbers(cam_a, cam_b):
    """view_numbers for two camera poses (position m, rotation) already known."""
    (pa, Ra), (pb, Rb) = cam_a, cam_b
    dot = float(np.clip(np.dot(-Ra[:, 2], -Rb[:, 2]), -1.0, 1.0))
    return {"mm": round(float(np.linalg.norm(pa - pb)) * MM, 2), "deg": round(math.degrees(math.acos(dot)), 3),
            "roll_deg": round(_roll_deg(Ra, Rb), 3)}


def tolerance(ex, q):
    """The bounds' same-view tolerance at the requested joint pose."""
    return WB.view_tol(ex, q)


def same_view(model, q_a, q_b, tol):
    n = view_numbers(model, q_a, q_b)
    same = n["mm"] <= tol["mm"] and n["deg"] <= tol["deg"] and abs(n["roll_deg"]) <= tol["roll_deg"]
    return {"same": bool(same), **n, "tol": dict(tol)}


# ---------------------------------------------------------------- what a frame shows
def own_mask(model, q, margin_px=OWN_MARGIN_PX):
    """bool H x W: the pixels the robot's own collision capsules cover at q (each sample drawn as a disc of its
    projected radius plus margin_px, joined along the capsule). The camera's pose comes from the model alone."""
    w, h, f = int(model.cam_w), int(model.cam_h), float(model.f_px)
    pc, Rc = cam_pose(model, q)
    mask = np.zeros((h, w), dtype=np.uint8)
    for _, p0, p1, r in model.collision_capsules(np.asarray(q, dtype=float)):
        ts = np.linspace(0.0, 1.0, CAPSULE_SAMPLES)
        P = np.asarray(p0)[None, :] + ts[:, None] * (np.asarray(p1) - np.asarray(p0))[None, :]
        u, v, d = project_pose(pc, Rc, f, w, h, P)
        prev = None
        for i in range(len(P)):
            if not d[i] > 1e-3:
                prev = None
                continue
            rad = f * float(r) / float(d[i]) + float(margin_px)
            if not (-rad - w < u[i] < 2 * w + rad and -rad - h < v[i] < 2 * h + rad):
                prev = None
                continue
            c = (int(round(u[i])), int(round(v[i])))
            rr = int(min(4 * max(w, h), math.ceil(rad)))
            cv2.circle(mask, c, rr, 1, -1)
            if prev is not None:
                cv2.line(mask, prev[0], c, 1, max(1, 2 * min(prev[1], rr)))
            prev = (c, rr)
    return mask.astype(bool)


def point_px(model, q, p_mm):
    """(u, v, depth_mm) of a frame-B point in mm, or None when it is behind the camera."""
    u, v, d = project(model, q, np.asarray(p_mm, dtype=float) / MM)
    if not d[0] > 1e-6:
        return None
    return float(u[0]), float(v[0]), float(d[0]) * MM


def in_frame(model, u, v, pad=0.0):
    return -pad <= u < model.cam_w + pad and -pad <= v < model.cam_h + pad


def _candidate_px(model, q, cand, floor_z_m):
    try:
        cx, cy = (float(x) for x in cand["where_mm"][:2])
        sx, sy = (float(x) for x in cand["size_mm"][:2])
        top = float(cand.get("top_mm") if cand.get("top_mm") is not None else floor_z_m * MM)
    except (KeyError, TypeError, ValueError):
        return None
    z0 = floor_z_m * MM
    pts = [[cx + a * sx / 2, cy + b * sy / 2, z] for a in (-1, 1) for b in (-1, 1) for z in (z0, max(top, z0))]
    u, v, d = project(model, q, np.asarray(pts) / MM)
    ok = d > 1e-6
    if not ok.all():
        return None                   # a corner behind the camera: no outline to draw
    return np.stack([u, v], axis=1)


def candidate_box(model, q, cand, floor_z_m=0.0):
    """A survey candidate's box (where_mm [x, y] centre, size_mm [sx, sy], top_mm) projected at q, as
    [u0, v0, u1, v1] clipped to the frame; None when it does not fall in the frame."""
    uv = _candidate_px(model, q, cand, floor_z_m)
    if uv is None:
        return None
    u0, v0 = max(0.0, float(uv[:, 0].min())), max(0.0, float(uv[:, 1].min()))
    u1, v1 = min(float(model.cam_w), float(uv[:, 0].max())), min(float(model.cam_h), float(uv[:, 1].max()))
    if u1 - u0 < 2 or v1 - v0 < 2:
        return None
    return [int(round(u0)), int(round(v0)), int(round(u1)), int(round(v1))]


def candidate_mask(model, q, cand, floor_z_m=0.0):
    """bool H x W: the pixels inside the outline of a survey candidate's box projected at q (the convex hull of
    its eight corners), or None."""
    uv = _candidate_px(model, q, cand, floor_z_m)
    if uv is None:
        return None
    mask = np.zeros((int(model.cam_h), int(model.cam_w)), dtype=np.uint8)
    hull = cv2.convexHull(np.round(uv).astype(np.int32))
    cv2.fillConvexPoly(mask, hull, 1)
    return mask.astype(bool)


# ---------------------------------------------------------------- the device points a compare judges
def doc(ex):
    d = (getattr(ex, "context", None) or {}).get("devices")
    return d if isinstance(d, dict) else None


def parts_of(ex):
    try:
        return sheet_words.parts(doc(ex))
    except Exception:          # a document this reader cannot parse names no parts
        return []


def candidates(ex):
    sight = ex.state.get("sight") or {}
    c = sight.get("candidates")
    return list(c) if isinstance(c, list) and c else list((ex.state.get("witness") or {}).get("candidates") or [])


def part_size_mm(part):
    sz = (part or {}).get("size_mm") or {}
    vals = [float(v) for v in sz.values() if isinstance(v, (int, float))]
    return max(vals) if vals else None


def floor_z(ex):
    return float(ex.cfg.get("floor_z_m") or 0.0)


# ---------------------------------------------------------------- targets
def _entry_target(ex, e, plist, q):
    lab = e.get("label")
    part = sheet_words.label_part(lab, plist)
    tgt = {"id": e["id"], "label": lab, "part": part["name"] if part else None, "point_mm": e.get("point_mm")}
    if e.get("point_mm") is None:
        return {**tgt, "why": "no point"}
    pp = point_px(ex.model, q, e["point_mm"])
    if pp is None or not in_frame(ex.model, pp[0], pp[1]):
        return {**tgt, "out": True}
    size = part_size_mm(part)
    part_px = None if size is None else float(ex.model.f_px) * size / max(pp[2], 1e-6)
    r = max(L.REGION_MIN_PX, 0.5 * part_px) if part_px else L.REGION_MIN_PX
    return {**tgt, "spec": {"disc": [pp[0], pp[1], r]}, "depth_mm": pp[2], "part_px": part_px}


def _located_in_view(ex, q, plist):
    out = []
    for e in ex.world.query(kind="located", include_stale=False):
        if e.get("point_mm") is not None:
            out.append(_entry_target(ex, e, plist, q))
    return out


def _named(ex, name, plist, q):
    w = ex.world
    e = w.entries.get(str(name))
    if e is not None and e.get("kind") in ("located", "contact"):
        return _entry_target(ex, e, plist, q)
    for c in candidates(ex):
        if str(c.get("id")) == str(name):
            box = candidate_box(ex.model, q, c, floor_z(ex))
            if box is None:
                return {"id": c["id"], "label": c["id"], "part": None, "out": True}
            return {"id": c["id"], "label": c["id"], "part": None, "spec": {"box": box}, "part_px": None,
                    "depth_mm": None}
    part = sheet_words.label_part(name, plist)
    fresh = [e for e in w.query(kind="located", include_stale=False) if e.get("point_mm") is not None]
    hit = [e for e in fresh if (part is not None and (sheet_words.label_part(e["label"], plist) or {}).get("name")
                                == part["name"]) or str(e["label"]).strip().lower() == str(name).strip().lower()]
    if hit:
        return _entry_target(ex, hit[-1], plist, q)
    return None


def targets(ex, at, q):
    """[target] for compare's at (see the module text) at joint pose q, or a refusal for a name nothing has."""
    plist = parts_of(ex)
    if at is None:
        return _located_in_view(ex, q, plist)
    items = at if isinstance(at, list) else [at]
    out, seen = [], set()
    for it in items:
        if isinstance(it, dict) and "px" in it:
            u, v = (float(x) for x in it["px"][:2])
            r = float(it.get("radius_px") or L.REGION_MIN_PX)
            t = {"id": None, "label": f"px ({u:.0f}, {v:.0f})", "part": None, "spec": {"disc": [u, v, r]},
                 "part_px": None, "depth_mm": None}
        elif isinstance(it, dict) and "box_px" in it:
            b = [float(x) for x in it["box_px"][:4]]
            t = {"id": None, "label": "box ({:.0f}, {:.0f}, {:.0f}, {:.0f})".format(*b), "part": None,
                 "spec": {"box": b}, "part_px": None, "depth_mm": None}
        elif isinstance(it, dict) and "point_mm" in it:    # an act's point with no entry (the hooks' use)
            t = _entry_target(ex, {"id": it.get("id"), "label": it.get("label", "the act's point"),
                                   "point_mm": it["point_mm"]}, plist, q)
        else:
            t = _named(ex, it, plist, q)
            if t is None:
                return refusal("unknown_target", f"no located or contact entry, candidate or part named "
                               f"{str(it)[:40]!r}", "robot://world lists the ids; or give {\"px\": [u, v]}")
        key = t.get("id") or (json.dumps(t["spec"], sort_keys=True) if "spec" in t else t.get("label"))
        if key not in seen:
            seen.add(key)
            out.append(t)
    return out


def device_box(ex, q, t_after, points=()):
    """The acted device's region in this look: the survey candidate box holding the newest act's point (grown
    DEVICE_BOX_NEAR_MM), else a disc of REGION_DISC_MM around it; with the point's depth and the outline's mask
    (the candidate's projected box, or the disc). With no act, the first of points (a located device point in view)
    stands for it: a device moved by someone else moves as one too. None with neither."""
    acts = [a for a in (ex.state.get("witness") or {}).get("acts") or []
            if a.get("point_mm") is not None and a["t0"] <= t_after]
    if acts:
        p = acts[-1]["point_mm"]
    elif points:
        p = points[0]
    else:
        return None
    pp = point_px(ex.model, q, p)
    if pp is None:
        return None
    for c in candidates(ex):
        try:
            (cx, cy), (sx, sy) = c["where_mm"][:2], c["size_mm"][:2]
        except (KeyError, TypeError, ValueError):
            continue
        g = WB.limit(ex, "DEVICE_BOX_NEAR_MM")
        if abs(p[0] - cx) <= sx / 2 + g and abs(p[1] - cy) <= sy / 2 + g:
            box = candidate_box(ex.model, q, c, floor_z(ex))
            if box is not None:
                return box, pp[2], candidate_mask(ex.model, q, c, floor_z(ex))
    r = float(ex.model.f_px) * L.REGION_DISC_MM / max(pp[2], 1e-6)
    disc, _ = diff.region((int(ex.model.cam_h), int(ex.model.cam_w)), {"disc": [pp[0], pp[1], r]}, grow=0)
    return [pp[0] - r, pp[1] - r, pp[0] + r, pp[1] + r], pp[2], disc


# ---------------------------------------------------------------- moves (look like, act's witness looks)
def _deg_list(q):
    return [round(math.degrees(float(x)), 4) for x in q]


def _gate_share(ex, q):
    """(refusal or None, share): the gates of the camera's own move to q, asked at the speed it will run; a path
    gate that judges a speed is asked again at half of it (witness_bounds.gate). share: the share of full speed
    that passed."""
    return WB.gate(ex, q)


def _gate(ex, q):
    return _gate_share(ex, q)[0]


def _retreat_one(ex, intent, moves):
    """One step back along the arm's own history, with the call's intent where the core takes one."""
    retreat = ex._retreat_impl
    kw = {"intent": intent} if "intent" in inspect.signature(retreat).parameters else {}
    moves.append(retreat(steps=1, **kw))


def _leg(ex, tool, stage, t0):
    ex.note_event("r3_leg", by="runtime", tool=tool, stage=stage, t0=round(float(t0), 3),
                  t1=round(float(ex.sim_time()), 3))


def _res_rad(ex):
    return WB.res_rad(ex)


def _return_refused(ex, out, q_goal, tol, stopped, retreated=False, q_before=None):
    """A refusal carries the measured view, never a claim that the goal was reached."""
    q_now = np.asarray(ex._sense().q, dtype=float)
    return {**out, "stopped": stopped, "retreated": retreated,
            "view": same_view(ex.model, q_now, q_goal, tol), "correctives": 0,
            "moved": bool(q_before is not None and not np.array_equal(q_now, q_before))}


def _unreachable(out, retreated):
    why = (out.get("refused") or {}).get("why", "the path was refused")
    return refusal("like_unreachable", f"the camera cannot go back to that pose: {why}"
                   + (" (one retreat step was made first)" if retreated else ""),
                   "look from where the arm is, or look like an earlier look taken before the touch")


def _goal_at_point(ex, q_goal):
    """Use the modeled tool tip and all stored points, including stale contact evidence."""
    tip_mm = np.asarray(ex.model.fk_tip(q_goal)[0], dtype=float) * MM
    for kind in ("contact", "located"):
        for entry in ex.world.query(kind=kind, include_stale=True):
            if entry.get("point_mm") is None:
                continue
            d = float(np.linalg.norm(tip_mm - np.asarray(entry["point_mm"], dtype=float)))
            if d <= L.ACT_NEAR_MIN_MM:
                label = entry.get("label") or entry["id"]
                return refusal("like_unreachable",
                               f"that look was taken with the tool {d:.1f} mm from {label} "
                               "(a point touched or located before): going back would touch it again",
                               "look from where the arm is, or look like a look taken before the touch")
    return None


def _dark_frame(ex):
    """Read one current wrist frame by the same light rule as a look."""
    from rcpx import perception
    light = perception.light(_frame(ex), cam=WB.camera(ex))
    if light["can_see"] == "dark":
        return refusal("dark", f"the camera's frame is dark now (brightness {light['brightness']:.1f} of 255): "
                       "a move toward the world is not made blind",
                       "get light on the scene or wait for light, then send the look again")
    return None


def _go_to(ex, q_goal, tool="look", intent=None, moves=None):
    """Read a current wrist frame first: dark refuses without movement. Then refuse a goal tool tip within
    ACT_NEAR_MIN_MM of any stored contact or located point, stale included. Gate the guarded joint move; a
    non-dark refusal gets one retreat step if history exists and one retry, else like_unreachable. A dark gate
    refuses without retreat. Then move and apply up to CORRECTIVES_MAX correctives as before. Returns refused
    when blocked, stopped, retreated, the measured view, correctives and moved; no result claims an unreached view.
    Every move is the runtime's own look move at the session's derived speed with the call's intent (the operator's
    word); a move that stopped on the way is not tried again by another primitive.
    """
    m = ex.model
    q_goal = m.clip(np.asarray(q_goal, dtype=float))
    t0 = ex.sim_time()
    tol = tolerance(ex, q_goal)
    q_start = np.asarray(ex._sense().q, dtype=float).copy()
    out = _dark_frame(ex)
    if out is not None:
        return _return_refused(ex, out, q_goal, tol, "dark")
    out = _goal_at_point(ex, q_goal)
    if out is not None:
        return _return_refused(ex, out, q_goal, tol, "refused")
    retreated = False
    gate, share = _gate_share(ex, q_goal)
    if gate is not None:
        chk = (gate.get("refused") or {}).get("check")
        if chk == "dark":
            return _return_refused(ex, gate, q_goal, tol, "dark")
        if ex._history:
            _retreat_one(ex, intent, moves)
            retreated = True
            _leg(ex, tool, "retreat", t0)
            gate, share = _gate_share(ex, q_goal)
        if gate is not None:
            return _return_refused(ex, _unreachable(gate, retreated), q_goal, tol, "refused", retreated, q_start)
    r = WB.look_move(ex, q_goal, intent, share)
    moves.append(r)
    if "refused" in r and ex._history and not retreated:
        chk = (r.get("refused") or {}).get("check")
        if chk == "dark":
            return _return_refused(ex, r, q_goal, tol, "dark")
        _retreat_one(ex, intent, moves)      # the move was refused before it ran: one retreat step out of a tight
        retreated = True                     # place, then one retry
        gate, share = _gate_share(ex, q_goal)
        if gate is not None:
            _leg(ex, tool, "retreat", t0)
            return _return_refused(ex, _unreachable(gate, retreated), q_goal, tol, "refused", retreated, q_start)
        r = WB.look_move(ex, q_goal, intent, share)
        moves.append(r)
    if "refused" in r:
        _leg(ex, tool, "move", t0)
        return _return_refused(ex, _unreachable(r, retreated), q_goal, tol, "refused", retreated, q_start)
    _leg(ex, tool, "move", t0)
    n_corr, stopped = 0, r["stop"]
    if stopped == "goal":
        res = _res_rad(ex)
        lead = np.zeros(m.n)
        worst = np.zeros(m.n)
        for _ in range(L.CORRECTIVES_MAX):
            q_now = np.asarray(ex._sense().q, dtype=float)
            if same_view(m, q_now, q_goal, tol)["same"]:
                break
            short = q_goal - q_now
            short = np.where(np.abs(short) > res, short, 0.0)   # one encoder step is rounding, not a shortfall
            worst = np.maximum(worst, np.abs(short))
            cap = L.CORRECTIVE_LEAD_MULT * worst
            lead = np.clip(lead + short, -cap, cap)
            target = m.clip(q_goal + lead)
            gate, share = _gate_share(ex, target)
            if gate is not None:
                break
            t1 = ex.sim_time()
            r = WB.look_move(ex, target, intent, share)
            moves.append(r)
            n_corr += 1
            _leg(ex, tool, "corrective", t1)
            if "refused" in r or r["stop"] != "goal":
                stopped = "refused" if "refused" in r else r["stop"]
                break
    q_now = np.asarray(ex._sense().q, dtype=float)
    return {"stopped": stopped, "view": same_view(m, q_now, q_goal, tol), "correctives": n_corr,
            "retreated": retreated, "moved": True}


def _frame(ex):
    return np.asarray(ex.backend.render("wrist"))[:, :, :3].astype(np.int16)


def settle(ex, tool="look"):
    """Hold until two frames one frame interval apart agree (their mean absolute difference under the signal the
    camera's own noise can show: rcpx.perception.noise_level): a part still swinging, a camera still adjusting its
    exposure. At least one pair is compared. The wait goes on while the time waited is under the arm's own settle
    time with its margin (witness_bounds.settle_bound_s) or the picture is still calming (the difference fell by REL
    of the last one; the first pair that differs has no last one and is given a second); it ends at the first pair
    that did not get calmer after that time (no progress, SPEC-R4 A4.8). No number of seconds is typed.
    {settled, waited_s, ended: agree | no_progress}."""
    from rcpx import perception
    t0, waited, previous = ex.sim_time(), 0.0, None
    step, bound = WB.settle_step_s(ex), WB.settle_bound_s(ex)
    agree = perception.noise_level(WB.camera(ex))
    f0 = _frame(ex)
    while True:
        ex.idle(step)
        waited += step
        f1 = _frame(ex)
        difference = float(np.mean(np.abs(f1 - f0)))
        if difference <= agree:
            ended = "agree"
            break
        calming = previous is None or difference <= (1 - R.REL) * previous      # one pair alone shows no trend
        if waited >= bound and not calming:
            ended = "no_progress"
            break
        f0, previous = f1, difference
    _leg(ex, tool, "settle", t0)
    return {"settled": ended == "agree", "waited_s": round(waited, 2), "ended": ended}


def look_hooks(ex, lk, note):
    """The post hooks registered for "look" by name, and core's look event, on a look the runtime took itself
    (as the approach package does for look_after)."""
    kwargs = {"note": note}
    core_events = getattr(ex, "_core_events", None)
    for fn, names in list(getattr(ex, "_post", [])):
        if not ((names is not None and "look" in names) or (core_events is not None and fn == core_events)):
            continue
        r = ex._guarded(fn, "post look (witness)", "look", kwargs, lk)
        if isinstance(r, dict):
            lk = r
    return lk


def go_and_look(ex, q, note="", intent=None):
    """The camera to joint pose q (go_to), settle, then a look with the look hooks: the look result (it gains
    "went": the move's view numbers and correctives) or a refusal (nothing more moves)."""
    g = go_to(ex, q, tool=str(note or "witness"), intent=intent)
    if "refused" in g:
        return g
    s = settle(ex, tool=str(note or "witness"))
    lk = ex._look_impl(str(note or "witness look"))
    lk = look_hooks(ex, lk, str(note or "witness look"))
    lk = dict(lk)
    lk["went"] = {"stopped": g["stopped"], "same_view": g["view"]["same"], "mm": g["view"]["mm"],
                  "deg": g["view"]["deg"], "correctives": g["correctives"], "settled": s["settled"]}
    for key in ("periods", "bounds"):
        if key in g:
            lk["went"][key] = g[key]
    return lk


def go_to(ex, q_goal, tool="look", intent=None):
    """Move at the bounds' offered speed, carrying intent and reporting control periods and binding bounds."""
    t0, moves = ex.sim_time(), []
    out = _go_to(ex, q_goal, tool, intent, moves)
    return WB.report(ex, out, moves, t0) if ex.sim_time() > t0 else out
