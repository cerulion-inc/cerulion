# SPDX-License-Identifier: AGPL-3.0-only
"""Where a survey looks from (package survey, I32): a ring of views around the robot, from the body model.

The views are the standard look turned about the base's vertical axis. When the first joint of the chain turns
about a vertical axis through the base origin (read off RobotModel.joint_frames at the standard pose), a view is
the standard pose with that joint turned: the arm keeps its shape, so the floor clearance and the self clearance of
every view are the standard pose's. Otherwise each view is solved (RobotModel.ik) for the standard camera pose
turned about the base z axis, tip target from the camera's fixed place on the tool, and a view that does not solve
is left out and named.

The step between views is a third of the angular width of the standard view's floor patch (measured at the
patch's far part, where it is narrowest), so every floor point of the ring is in about three views: the
raised-object test needs two, and the hand hides the lower middle of each frame. Without span_deg the span is
the first joint's whole turn inside a half turn each side of the standard view, cut to what the arm's own path
check finds free (free_views); span_deg widens or narrows
it up to a full circle. More views than max_views widen the step, and the result says the overlap dropped.

Nothing here names a robot or reads a scene: joint frames, limits, the camera model and the floor height from cfg.
"""

import inspect
import math

import numpy as np

from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx.r2 import survey_see as see

VERTICAL_COS = 0.995        # the first joint's axis within about 6 deg of base z counts as vertical
ON_AXIS_M = 0.01            # and passing this close to the base origin
STEP_MIN_DEG, STEP_MAX_DEG = 10.0, 90.0
STEP_FRAC = 1.0 / 3.0       # the step is this share of a view's angular width: a floor point is in three views
MAX_VIEWS = 16
FAR_PART = 0.75             # the angular width is read at this quantile of the patch's radii (its far part)


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def turn_axis(model, q_std):
    """+1 or -1 when the first joint turns the arm about base z through the origin (its sign: a positive joint
    angle turns the view toward +bearing, i.e. left), else None."""
    anchor, axis = model.joint_frames(q_std)[0]
    axis = np.asarray(axis, dtype=float) / max(float(np.linalg.norm(axis)), 1e-12)
    if abs(float(axis[2])) < VERTICAL_COS or math.hypot(float(anchor[0]), float(anchor[1])) > ON_AXIS_M:
        return None
    return 1.0 if axis[2] > 0 else -1.0


def view_width_deg(model, q, z):
    """Angular width (deg) of the floor patch seen at q, read at its far part, and its bearing (deg); (None, None)
    when the camera does not see the floor."""
    pts = see.footprint_bearings(model, q, z)
    if len(pts) < 8:
        return None, None
    r = np.array([p[0] for p in pts])
    b = np.degrees(np.array([p[1] for p in pts]))
    centre = see.floor_view(model, q, z)["centre"]
    c = math.degrees(math.atan2(centre[1], centre[0])) if centre else float(np.median(b))
    rel = _wrap(b - c)
    if np.min(r) < 0.02:                     # the patch holds the base axis: it sees every bearing
        return 360.0, c
    far = r >= np.quantile(r, FAR_PART) - 1e-9
    near_far = rel[far] if far.sum() >= 3 else rel
    return float(np.max(near_far) - np.min(near_far)), c


def plan(ex, span_deg=None, max_views=MAX_VIEWS, keep_overlap=False):
    """{"views": [{"turn_deg", "bearing_deg", "q"}], "step_deg", "span_deg", "method", "notes"} or {"refused"}.
    keep_overlap (the default sweep: no span and no view count asked): when the whole turn needs more views than
    max_views at the step that puts every floor point in three views, the sweep keeps that step and covers the part
    of the turn nearest the standard view, and a note names the bearings left for a second call."""
    m = ex.model
    z = float(ex.cfg.get("floor_z_m") or 0.0)
    q_std = np.asarray(ex._standard_q(), dtype=float)
    width, b_std = view_width_deg(m, q_std, z)
    if width is None:
        return {"why": "the standard look does not see the surface the robot stands on, so there is no floor "
                       "patch to sweep"}
    sign = turn_axis(m, q_std)
    notes = []
    step = min(STEP_MAX_DEG, max(STEP_MIN_DEG, STEP_FRAC * width))
    if sign is not None:
        lo, hi = (math.degrees(m.q_lo[0] - q_std[0]), math.degrees(m.q_hi[0] - q_std[0]))
        t_lo, t_hi = (lo, hi) if sign > 0 else (-hi, -lo)       # in bearing units relative to the standard view
        method = f"the first joint ({m.joint_names[0]}) turned from the standard pose"
    else:
        t_lo, t_hi = -180.0, 180.0
        method = "the standard camera pose turned about the base z axis, each view solved by inverse kinematics"
    full = t_hi - t_lo >= 360.0 - 1e-6
    t_lo, t_hi = max(t_lo, -180.0), min(t_hi, 180.0)
    if span_deg is not None and abs(float(span_deg[1]) - float(span_deg[0])) < 360.0 - 1e-6:
        lo_b, hi_b = sorted(float(x) for x in span_deg)
        a, b = lo_b - b_std, hi_b - b_std
        shift = 360.0 * round(0.5 * (a + b) / 360.0)      # the same bearings, centred on the standard view
        t_lo, t_hi, full = max(t_lo, a - shift), min(t_hi, b - shift), False
        if t_hi < t_lo:
            return {"why": f"the bearings {span_deg[0]:g} to {span_deg[1]:g} deg are outside the arm's turn "
                           f"({b_std + t_lo:.0f} to {b_std + t_hi:.0f} deg)"}
    full = full and t_hi - t_lo >= 360.0 - 1e-6
    span = t_hi - t_lo
    if full:
        n = max(1, int(math.ceil(360.0 / step - 1e-9)))
    else:
        n = max(1, int(math.ceil(span / step - 1e-9)) + 1)
    if n > max_views and keep_overlap and max_views > 1:
        cover = (max_views - 1) * step                       # what max_views views cover at the full overlap
        lo = max(t_lo, min(-cover / 2.0, t_hi - cover))      # centred on the standard view where the turn allows
        hi = min(t_hi, lo + cover)
        notes.append(f"the arm's whole turn ({b_std + t_lo:.0f} to {b_std + t_hi:.0f} deg) needs {n} views to put "
                     f"each floor point in three; this sweep takes {max_views} and covers {b_std + lo:.0f} to "
                     f"{b_std + hi:.0f} deg: survey again with span_deg for the rest")
        t_lo, t_hi, full, n = lo, hi, False, max_views
    elif n > max_views:
        notes.append(f"{n} views would put each floor point in three; {max_views} were taken, so neighbouring views "
                     "overlap less and the raised-object test sees less of the ring")
        n = max_views
    if full:
        turns = [t_lo + k * 360.0 / n for k in range(n)]
    else:
        turns = [0.5 * (t_lo + t_hi)] if n == 1 else list(np.linspace(t_lo, t_hi, n))
    views = []
    for t in turns:
        if sign is not None:
            q = q_std.copy()
            q[0] += sign * math.radians(t)
            q = m.clip(q)
        else:
            q = _solve_turned(m, q_std, math.radians(t))
            if q is None:
                notes.append(f"no arm pose holds the camera at bearing {b_std + t:.0f} deg; that view is left out")
                continue
        views.append({"turn_deg": float(t), "bearing_deg": round(_wrap(b_std + t), 1), "q": q})
    return {"views": views, "step_deg": round((turns[1] - turns[0]) if len(turns) > 1 else 0.0, 1),
            "width_deg": round(width, 1), "span_deg": [round(b_std + t_lo, 1), round(b_std + t_hi, 1)],
            "full_turn": bool(full), "method": method, "notes": notes, "standard_bearing_deg": round(b_std, 1)}


def _solve_turned(m, q_std, turn):
    """The joint pose that holds the standard camera pose turned by `turn` about base z, or None."""
    fk = m.fk(q_std)
    p_t, R_t = fk["tip"]
    p_c, R_c = fk["camera"]
    c, s = math.cos(turn), math.sin(turn)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    # the camera rides on the tool: turning the camera pose turns the tip pose the same way about base z
    q, ok, _ = m.ik(q_std, Rz @ p_t, Rz @ R_t)
    if not ok:
        return None
    pc2 = np.asarray(m.fk(q)["camera"][0], dtype=float)
    if float(np.linalg.norm(pc2 - Rz @ p_c)) > 0.01:
        return None
    return q


def order(views, q_now, index=0):
    """The views in sweep order: first the view nearest the arm's present first joint angle, then on toward the
    nearer end of the span, then back across to the far end (the views near where the arm stands come first)."""
    if len(views) < 2:
        return list(views)
    k0 = int(np.argmin([abs(float(v["q"][index] - q_now[index])) for v in views]))
    up, down = list(views[k0:]), list(reversed(views[:k0]))
    if not down:
        return up
    if len(up) - 1 <= len(down):          # the shorter side first, so the way back across is the shorter one
        return up + down
    return [up[0]] + down + up[1:]


def leg_seconds(ex, q0, q1, intent=None):
    """World seconds the look move from q0 to q1 takes at the least: the largest joint travel over that joint's own
    speed bound (B.qd_max: every joint at its bound). The move can only run slower (through space no look has shown
    it runs at the gentle bound), so a sweep is cut here only when it cannot fit even at full speed; the session's
    budget bounds the rest. The session's full plan (B.plan) is not asked for a time: measured on the live tree it
    builds one pose per control period, and a slow plan of one survey test grew to 37 GB before it was stopped.
    TRANSITION: on a core that takes no kind, the time of that core's own plan at the share the move is called
    with. Nothing moves."""
    q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
    move = getattr(ex, "_joint_move", None)
    if move is not None and "kind" not in inspect.signature(move).parameters:
        return len(ex._plan_joint(q0, q1, OLD_CORE_SHARE)) * float(ex.dt)
    return max((abs(d) / v if v > 0 else (float("inf") if d else 0.0)
                for d, v in zip(q1 - q0, bounds.of(ex).qd_max())), default=0.0)


def fit_time(ex, views, q_now, max_s=None, intent=None):
    """(the views, in sweep order, whose legs fit in max_s of world time, the planned seconds, the seconds all of
    them would take). The sweep keeps its order and ends at the first leg that would pass max_s; the first view is
    always kept (a survey that moves nowhere would see nothing). None means no operator or budget bound."""
    total, kept, q = 0.0, [], np.asarray(q_now, dtype=float)
    every, ended = 0.0, False
    for v in views:
        dt = leg_seconds(ex, q, v["q"], intent) + SETTLE_S
        every += dt
        if not ended and (not kept or max_s is None or total + dt <= max_s):
            kept.append(v)
            total += dt
        else:
            ended = True
        q = v["q"]
    return kept, total, every


SETTLE_S = 0.2              # the ease and look at each view


CLEAR_ARG = {"anyOf": [{"type": "boolean"}, {"type": "object", "properties": {"look": {"anyOf": [
    {"type": "string"}, {"type": "integer"}]}}, "required": ["look"]}],
    "description": "true when you looked and judge the camera's way clear (or {\"look\": id} for the look you "
                   "judged it in): recorded in the result; it changes no speed"}


def clear_word(clear):
    """(intent, why): the operator's word of a tool call as the intent its moves take (SPEC-R4 A6: clear is true, or
    {"look": id}); why is the text of a refusal when clear is anything else."""
    if clear is None or clear is False:
        return None, None
    if clear is True:
        return {"clear": True}, None
    if isinstance(clear, dict) and set(clear) == {"look"} and isinstance(clear["look"], (str, int)) \
            and not isinstance(clear["look"], bool):
        return {"clear": {"look": clear["look"]}}, None
    return None, ('clear is true (you looked and judge the camera\'s way clear) or {"look": id} for the look you '
                  'judged it in')


OLD_CORE_SHARE = 0.5     # TRANSITION only (see look_move): never read on a core that takes kind


def look_move(ex, q, intent=None, share=None):
    """The camera's own joint move to q: ex._joint_move(q, None, kind="look", intent=...) (SPEC-R4 A8.2: speed
    None is the derived full speed, the contact stop armed; a share under 1 is the caller's own lower speed).
    TRANSITION: a core that does not take kind yet is called the way it was called before round four (the share
    OLD_CORE_SHARE of its own speed setting, guarded), so nothing changes on that core; the Review stage removes
    that branch and the constant. Returns the core's stop dict or a refusal."""
    move = ex._joint_move
    q = np.asarray(q, dtype=float)
    slower = share is not None and share < 1
    if "kind" in inspect.signature(move).parameters:
        return move(q, float(share) if slower else None, kind="look", intent=intent)
    return move(q, OLD_CORE_SHARE * (float(share) if slower else 1.0), guarded=True)


def periods(ex, t0):
    """Control periods since world time t0 (the session's own clock over the measured loop period)."""
    return int(round((float(ex.sim_time()) - float(t0)) / float(bounds.of(ex).dt())))


def binding(move):
    """The binding bound of one core motion result ({what, value, unit, source, evidence, lift}), or None."""
    b = move.get("bounds") if isinstance(move, dict) else None
    if isinstance(b, dict):
        return b.get("binding") if isinstance(b.get("binding"), dict) else (b if "what" in b else None)
    if isinstance(b, list) and b and isinstance(b[0], dict):
        return b[0]
    return None


def report(ex, out, moves, t0, detail="brief"):
    """out with periods (control periods since t0) and, when a move reported one, bounds (SPEC-R4 A5): the binding
    bounds of the tool's own moves through the session's report."""
    out["periods"] = periods(ex, t0)
    items = [i for i in (binding(m) for m in moves) if i is not None]
    return bounds.of(ex).report(out, items, detail) if items else out


SAMPLES_GUARD = 131072  # solver numeric: reject an unrepresentably fine geometric sampling


def _ends(ex, q):
    return np.asarray([p for _, a, b, _ in ex.model.collision_capsules(q)[1:] for p in (a, b)])


def _line(ex, qa, qb):
    """Straight joint poses, doubled until every capsule endpoint travels at most the smallest moving radius."""
    radius = min((float(r) for _, _, _, r in ex.model.collision_capsules(qa)[1:] if r > 0), default=0.0)
    if radius <= 0:
        return None
    n = max(1, math.ceil(float(np.max(np.linalg.norm(_ends(ex, qb) - _ends(ex, qa), axis=1))) / radius))
    while n <= SAMPLES_GUARD:
        line = [np.asarray(qa) + (np.asarray(qb) - qa) * (k / n) for k in range(1, n + 1)]
        previous = _ends(ex, qa)
        for q in line:
            present = _ends(ex, q)
            if np.max(np.linalg.norm(present - previous, axis=1)) > radius:
                break
            previous = present
        else:
            return line
        n *= R.DOUBLE
    return None


def free_views(ex, views, q_now):
    """(the views the sweep keeps, notes): the planned ring cut to the free space the arm's own path check finds.
    The views are walked in sweep order (order): each leg, the straight joint line from the last kept view, is
    checked dry (ex._path_check); the sweep keeps the views up to the first refused leg on each side of where it
    starts, and a note names the bearing where the free turn ended and why. The first view is always kept: its
    leg starts where the arm stands now and the move itself answers for it (it may retreat once first)."""
    ordered = order(views, q_now)
    if not ordered:
        return [], []
    start = float(ordered[0]["turn_deg"])
    kept, notes, blocked = [ordered[0]], [], set()
    qa = np.asarray(ordered[0]["q"], dtype=float)
    for v in ordered[1:]:
        side = 1 if float(v["turn_deg"]) > start else -1
        if side in blocked:
            continue
        line = _line(ex, qa, v["q"])
        bad = {"refused": {"why": "the path cannot be sampled finely enough"}} if line is None else \
            ex._path_check([qa] + line, True)
        if bad is not None:
            blocked.add(side)
            reason = bad.get("refused", bad).get("why", "the arm's path check refused the leg")
            notes.append(f"the free turn ended before bearing {v['bearing_deg']:g} deg: {reason}")
            continue
        kept.append(v)
        qa = np.asarray(v["q"], dtype=float)
    return kept, notes
