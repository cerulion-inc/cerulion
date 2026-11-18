# SPDX-License-Identifier: AGPL-3.0-only
"""The camera moves of seek (package assist) and how they keep clear.

camera_to(run, target, side) puts the camera on one side of the thing seek looks round and takes a look there, one
of three ways, each by PUBLIC primitives only, so every gate, cap, journal record and post hook of an operator's
own call applies to it:
  (a) view {target, side}, when the view tool is loaded and the target is something it takes (an id);
  (b) else a pose from the view planner (ex.state["sight"]["plan_camera"]) reached by move_joints, then look;
  (c) with no planner, one guarded move_tip of at most a third of the reach round the thing, the camera's own
      distance kept, a turn of the tool that puts the camera back on the thing, then look.
It answers {stopped: looked, look, can_see, side, pic} or {stopped: out_of_reach | refused | bound, why, check}.
A side that cannot be reached is found dry (the planner, reach_check, the clearance below) and costs no call.

Keeping clear, for (b) and (c). The planner keeps the arm off what robot://world holds, and a thing that was only
numbered in a picture is not there (measured: a planned move round a bare table point met the thing there).
So the thing's extent is read off its outline in that picture (keep_of): its near base is where the outline's
point nearest the camera meets the table, its width is the angle the outline fills, it is taken to be as deep as
it is wide, and its height is bounded by the line of sight over its furthest point. No body of the arm may come
nearer that upright cylinder than the view planner's tool clearance (nor nearer than it already stands). The
camera stands off by the thing's radius, the tool's lead before the lens and that clearance; a straight joint move
that would pass nearer goes over the top, else round by a side next to the one asked, and a side with no clear way
is out of reach. An outline cut by the picture's edge gives no size, and seek does not start on it.
Known limit: a part that stands on something that was not numbered is placed where its line of sight meets the
table, behind the thing that carries it; the guard of every move (a stop on contact) is what is left there.

Run counts one routine: every nested public call goes through Run.call, which tests the guard's bound first (a
nested call cannot be cut short: the worst case is a bound plus one call) and adds the tip's travel.
Nothing here reads simulator ground truth or names a robot, a device, a task or a model.
"""

import math

import numpy as np

from rcpx import r3_limits as L
from rcpx import side_words

MM = 1000.0
SAY = {"robot_side": "robot side", "left": "left side", "right": "right side", "far_side": "far side", "above": "top"}
NEXT_TO = {"robot_side": ("left", "right"), "far_side": ("left", "right"), "left": ("robot_side", "far_side"),
           "right": ("robot_side", "far_side")}
STOP_CHECKS = ("dark", "can_see", "constraint", "seen_obstacle", "no_grip", "stale_target")   # and every path check
CLEAR_M = L.VIEW_TOOL_CLEAR_MM / MM
PATH_STEP_DEG = 3.0          # the straight joint move is tested at poses no further apart than this on any joint
TURN_MIN_DEG = 2.0           # a camera this near its aim is not turned again


def q_now(ex):
    return np.asarray(ex._sense().q, dtype=float)


def tip(ex, q=None):
    return np.asarray(ex.model.fk_tip(q_now(ex) if q is None else q)[0], dtype=float)


def floor(ex):
    return float(ex.cfg.get("floor_z_m") or 0.0)


def hit(ex, q, u, v):
    """Where the line of sight through a pixel meets the table (m, frame B), or None when it does not."""
    o, d = ex.model.pixel_ray(q, float(u), float(v))
    if d[2] > -1e-6:
        return None
    t = (floor(ex) - float(o[2])) / float(d[2])
    return None if t <= 0 else np.asarray(o, dtype=float) + t * np.asarray(d, dtype=float)


def keep_of(ex, q, row):
    """(centre xy, radius, height) in m of the upright cylinder taken to hold the thing a row outlines (module
    text), or None when its size cannot be read. The outline is the row's polygon, else the corners of its box. Its
    point nearest the camera on the table is the thing's near base; the spread of the outline's lines of sight
    about the vertical is its width; a round thing with that base and that width gives the centre, and the radius
    is that thing's times root two (the corners of a square one); the height is bounded by the line of sight over
    the outline's furthest point. The camera may be rolled: nothing here reads image rows. None when a point that
    decides the base or the width lies on the picture's edge (the outline is cut there), when no point meets the
    table, or when the thing fills too wide an angle to be bounded."""
    poly, box = row.get("polygon_px"), row.get("box_px")
    if isinstance(poly, (list, tuple)) and len(poly) >= 3:
        pts = [(float(p[0]), float(p[1])) for p in poly]
    elif isinstance(box, (list, tuple)) and len(box) == 4:
        x1, y1, x2, y2 = (float(v) for v in box)
        pts = [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]
    else:
        return None
    w_px, h_px = float(ex.model.cam_w), float(ex.model.cam_h)
    pc, Rc = (np.asarray(x, dtype=float) for x in ex.model.fk(q)["camera"])
    ahead = -Rc[:, 2]
    a0 = math.atan2(ahead[1], ahead[0])            # the camera's own heading: the angles below are taken about it
    rows = []
    for u, v in pts:
        _, d = ex.model.pixel_ray(q, u, v)
        p = hit(ex, q, u, v)
        rows.append({"az": (math.atan2(d[1], d[0]) - a0 + math.pi) % (2.0 * math.pi) - math.pi,
                     "far": math.inf if p is None else float(np.linalg.norm(p[:2] - pc[:2])),
                     "cut": u <= 1.0 or v <= 1.0 or u >= w_px - 1.0 or v >= h_px - 1.0})
    base, lo, hi = min(rows, key=lambda r: r["far"]), min(rows, key=lambda r: r["az"]), max(rows, key=lambda r: r["az"])
    if not math.isfinite(base["far"]) or base["cut"] or lo["cut"] or hi["cut"]:
        return None
    s = math.sin(0.5 * (hi["az"] - lo["az"]))
    if not 0.0 < s < 0.9:
        return None
    r = base["far"] * s / (1.0 - s)
    mid = a0 + 0.5 * (hi["az"] + lo["az"])
    high = pc[2] - floor(ex)                       # no taller than the camera stands; lower by the furthest sight
    furthest = max(rows, key=lambda r: r["far"])
    if math.isfinite(furthest["far"]) and not any(r["cut"] for r in rows):
        high *= max(0.0, 1.0 - base["far"] / max(furthest["far"], base["far"]))
    centre = pc[:2] + (base["far"] + r) * np.array([math.cos(mid), math.sin(mid)])
    return centre, r * math.sqrt(2.0), high


def _gap(ex, q, keep):
    """How near (m) a body of the arm at pose q stands to the keep-clear cylinder (the base left out)."""
    c, r, h = keep
    pts = [(p0 + s * (p1 - p0), rad) for _, p0, p1, rad in ex.model.collision_capsules(q)[1:]
           for s in (0.0, 0.25, 0.5, 0.75, 1.0)] + [(tip(ex, q), 0.0)]
    best = math.inf
    for p, rad in pts:
        side = float(np.hypot(p[0] - c[0], p[1] - c[1])) - rad - r
        over = float(p[2]) - rad - (floor(ex) + h)
        best = min(best, side if over <= 0 else (over if side <= 0 else float(np.hypot(side, over))))
    return best


def blocked(ex, q0, q1, keep):
    """None, or how near (mm) the straight joint move q0 to q1 would bring the arm to the thing: nearer than the
    view planner's tool clearance and nearer than the arm stands now."""
    least = min(CLEAR_M, _gap(ex, q0, keep)) - 0.002
    n = max(2, int(float(np.max(np.abs(q1 - q0))) / math.radians(PATH_STEP_DEG)) + 1)
    worst = min(_gap(ex, q0 + (q1 - q0) * (k / n), keep) for k in range(1, n + 1))
    return None if worst >= least else round(worst * MM)


class Run:
    """One routine's counters: looks taken, tip travel, the start time and the guard's Bounds."""

    def __init__(self, ex, session, bounds, reach_mm):
        self.ex, self.session, self.b, self.reach_mm = ex, session, bounds, reach_mm
        self.t0, self.taken, self.travel = ex.sim_time(), 0, 0.0

    def call(self, name, next_mm=0.0, **kw):
        """(result, None) of a public primitive, or (None, the bound that forbids it): tested before EVERY call."""
        try:
            why = self.b.before(self.taken, self.ex.sim_time() - self.t0, self.travel, next_mm)
        except Exception:              # noqa: BLE001 (a guard that fails counts as a bound: nothing more runs)
            why = "guard"
        if why:
            return None, {"stopped": "bound", "why": str(why)}
        tip0 = tip(self.ex)
        out = getattr(self.ex, name)(**kw)
        self.travel += float(np.linalg.norm(tip(self.ex) - tip0)) * MM
        return (out if isinstance(out, dict) else {}), None


def read(r, side):
    """What a nested call that should end in a look gave: a look, a side out of reach, or a stop."""
    ref = r.get("refused")
    if isinstance(ref, dict):
        chk, why = str(ref.get("check")), str(ref.get("why") or "")[:160]
        if chk in STOP_CHECKS or chk.startswith("path"):
            return {"stopped": "refused", "check": chk, "why": why}
        return {"stopped": "out_of_reach", "why": f"{chk}: {why}"}
    if r.get("look_id") is None:       # a guarded camera move that did not end at its goal
        return {"stopped": "refused", "check": str(r.get("stopped") or "stopped"),
                "why": str(r.get("why") or r.get("said") or "the camera's move stopped on the way")[:160]}
    pics = r.get("pictures") or [r.get("annotated")]
    return {"stopped": "looked", "look": r["look_id"], "can_see": r.get("can_see") or "ok",
            "side": r.get("got_side") or side, "pic": pics[0] if pics and isinstance(pics[0], str) else None}


def _move(run, q):
    r, bad = run.call("move_joints", float(np.linalg.norm(tip(run.ex, q) - tip(run.ex))) * MM,
                      q_deg=[round(math.degrees(float(x)), 3) for x in q], guarded=True)
    if bad is None and (isinstance(r.get("refused"), dict) or r.get("stopped") != "goal"):
        bad = read(r, None)
    return bad


def _look(run, side, say):
    r, bad = run.call("look", note=f"seek: {say} from the {SAY[side]}")
    return bad or read(r, side)


def _vantage(ex, tg, side, reach_mm):
    """(q, why, blocked): a camera pose on that side of the thing that the view planner solves and that keeps the
    arm clear of it on the way there, the nearest such distance first. Dry: nothing moves."""
    plan, keep, q0 = ex.state["sight"]["plan_camera"], tg["keep"], q_now(ex)
    far = min(L.VIEW_D_MAX_MM, reach_mm)
    if keep is None:
        dists = [None]                 # an id the world holds: the planner's own distances and clearance
    else:
        pc, Rc = ex.model.fk(q0)["camera"]
        lead = max(0.0, float((tip(ex, q0) - np.asarray(pc)) @ (-np.asarray(Rc)[:, 2])))   # the tool before the lens
        near = min(far, max(L.VIEW_D_MIN_MM, (keep[1] + lead) * MM + L.VIEW_TOOL_CLEAR_MM))
        dists = list(np.arange(near, far + 1e-6, L.VIEW_D_STEP_MM))
    why, stood = None, False           # the reason at the nearest distance is the one reported
    for d in dists:
        p = plan([float(x) * MM for x in tg["point_m"]], side, None if d is None else float(d))
        if p.get("q") is None:
            why = why or str(p.get("why") or "no camera pose there")
            continue
        q = np.asarray(p["q"], dtype=float)
        near = None if keep is None else blocked(ex, q0, q, keep)
        if near is None:
            return q, "", False
        why, stood = why or f"the way there would pass {near} mm from {tg['say']}", True
    return None, (why or "no camera pose there")[:120], stood


def _by_plan(run, tg, side):
    ex = run.ex
    q, why, stood = _vantage(ex, tg, side, run.reach_mm)
    if q is None and stood:            # the straight way passes the thing: over its top, else round by a next side
        for via in ("above",) + NEXT_TO.get(side, ()):
            qv, _, _ = _vantage(ex, tg, via, run.reach_mm)
            if qv is None:
                continue
            bad = _move(run, qv)
            if bad:
                return bad
            q, _, _ = _vantage(ex, tg, side, run.reach_mm)
            if q is not None:
                break
    if q is None:
        return {"stopped": "out_of_reach", "why": why}
    return _move(run, q) or _look(run, side, tg["say"])


def _by_view(run, tg, side):
    ex, nxt = run.ex, 0.0
    plan = (ex.state.get("sight") or {}).get("plan_camera")
    if callable(plan):                 # dry first: a side the planner cannot reach costs no call
        p = plan([float(x) * MM for x in tg["point_m"]], side)
        if p.get("q") is None:
            return {"stopped": "out_of_reach", "why": str(p.get("why") or "no camera pose there")[:120]}
        nxt = float(np.linalg.norm(tip(ex, np.asarray(p["q"], dtype=float)) - tip(ex))) * MM
    r, bad = run.call("view", nxt, target=tg["id"], side=side)
    return bad or read(r, side)


def _by_tip(run, tg, side):
    """(c) of the module text."""
    ex, T = run.ex, tg["point_m"]
    q = q_now(ex)
    pc, tp = np.asarray(ex.model.fk(q)["camera"][0], dtype=float), tip(ex, q)
    off, h = pc[:2] - T[:2], side_words.side_vector(side, T * MM)[:2]
    rho = float(np.linalg.norm(off))
    if rho < L.RADIAL_NEAR_MM / MM:    # the camera stands over the thing: out toward that side
        off, rho = h * 1e-3, max(pc[2] - T[2], L.VIEW_D_MIN_MM / MM)
    a0 = math.atan2(off[1], off[0])
    turn = (math.atan2(h[1], h[0]) - a0 + math.pi) % (2.0 * math.pi) - math.pi
    if abs(turn) < math.radians(L.VIEW_AZ_STEP_DEG):
        return {"stopped": "out_of_reach", "why": "the camera already stands on that side"}
    step = math.copysign(min(abs(turn), 2.0 * math.asin(min(1.0, run.reach_mm / MM / 3.0 / (2.0 * rho)))), turn)
    dp = np.array([T[0] + rho * math.cos(a0 + step) - pc[0], T[1] + rho * math.sin(a0 + step) - pc[1], 0.0])
    if tg["keep"] is not None:         # the hand is carried without turning: its two ends against the cylinder
        c, r, hgt = tg["keep"]
        for s in np.linspace(0.0, 1.0, 9):
            for p in (pc + s * dp, tp + s * dp):
                if p[2] <= floor(ex) + hgt + CLEAR_M and float(np.hypot(p[0] - c[0], p[1] - c[1])) - r < CLEAR_M:
                    return {"stopped": "out_of_reach", "why": f"the way there would pass too near {tg['say']}"}
    if callable(getattr(ex, "reach_check", None)):
        rc, bad = run.call("reach_check", point_mm=[round(float(x) * MM, 1) for x in tp + dp])
        if bad:
            return bad
        if not rc.get("reachable"):
            return {"stopped": "out_of_reach", "why": str(rc.get("why") or (rc.get("refused") or {}).get("why"))[:120]}
    r, bad = run.call("move_tip", float(np.linalg.norm(dp)) * MM, dx=round(float(dp[0]) * MM, 1),
                      dy=round(float(dp[1]) * MM, 1), dz=0.0, frame="base", guarded=True)
    if bad is None and (isinstance(r.get("refused"), dict) or r.get("stopped") not in ("goal", "limit")):
        bad = read(r, None)
    if bad:
        return bad
    q = q_now(ex)
    pc, Rc = (np.asarray(x, dtype=float) for x in ex.model.fk(q)["camera"])
    a, b = -Rc[:, 2], (T - pc) / max(float(np.linalg.norm(T - pc)), 1e-9)
    axis = np.cross(a, b)
    ang = math.atan2(float(np.linalg.norm(axis)), float(a @ b))
    if ang > math.radians(TURN_MIN_DEG) and callable(getattr(ex, "orient_tool", None)):
        k = axis / float(np.linalg.norm(axis))
        K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
        Rt = np.asarray(ex.model.fk_tip(q)[1], dtype=float)
        E = Rt.T @ (np.eye(3) + math.sin(ang) * K + (1.0 - math.cos(ang)) * (K @ K)) @ Rt   # the turn, tool frame
        r, bad = run.call("orient_tool", yaw_deg=round(math.degrees(math.atan2(E[1, 0], E[0, 0])), 2),
                          pitch_deg=round(math.degrees(-math.asin(max(-1.0, min(1.0, E[2, 0])))), 2),
                          roll_deg=round(math.degrees(math.atan2(E[2, 1], E[2, 2])), 2), relative=True)
        if bad:
            return bad
        got = read(r, None)            # a gate's refusal or a contact ends it; a turn the arm cannot make does not
        if got["stopped"] == "refused" and (isinstance(r.get("refused"), dict)
                                            or r.get("stopped") in ("contact", "cap")):
            return got
    return _look(run, side, tg["say"])


def camera_to(run, tg, side):
    """The camera on that side of the target and a look there (module text). tg: {id, point_m, keep, say}."""
    if tg["id"] is not None and callable(getattr(run.ex, "view", None)):
        return _by_view(run, tg, side)
    if callable((run.ex.state.get("sight") or {}).get("plan_camera")):
        return _by_plan(run, tg, side)
    return _by_tip(run, tg, side)


__all__ = ["CLEAR_M", "MM", "Run", "SAY", "STOP_CHECKS", "blocked", "camera_to", "floor", "hit", "keep_of", "q_now",
           "read", "tip"]
