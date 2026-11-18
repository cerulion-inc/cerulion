# SPDX-License-Identifier: AGPL-3.0-only
"""What package witness takes from the session's derived bounds (SPEC-R4 A4, A8): the camera's own joint move, how
long a settle may last, the camera's own noise, the round three limits as the session derives them, and the bounds
block of a result. No limit is typed here: every value is the session's (rcpx.bounds.of(ex)), a ratio of
rcpx.bounds_ratios, or a round three limit read through r3_limits' lower case function of the same name (the one
typed number is OLD_CORE_SHARE, read only on the core before round four). A package never imports another package
(SPEC-R2 1.2), so the few lines package sight has of the same kind are repeated here.

TRANSITION (the core and r3_limits are rebuilt beside this package; the Review stage removes these three branches):
- look_move calls ex._joint_move(q, None, kind="look", intent=...) (SPEC-R4 A8.2: speed None is the derived full
  speed, the contact stop armed; a share under 1 is the caller's own lower speed). A core that does not take kind
  yet is called exactly the way it was called before round four (the share OLD_CORE_SHARE of its own speed setting,
  guarded), so nothing changes on that core until the core of A8.2 lands.
- limit(ex, "NAME", ...) calls r3_limits.name(B, ...) when that function exists, else reads the constant NAME.
- r3(ex, "fn", ...) calls an r3_limits formula with the session's bounds when its first parameter is B, else with the
  arguments it takes today.
Nothing here names a robot or a device or reads ground truth.
"""

import inspect

import numpy as np

from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx import r3_limits as L

MM = 1000.0
BOUNDS_NAMES = ("B", "bounds")
OLD_CORE_SHARE = 0.5     # TRANSITION only (module text): the share of its speed setting the old core ran the
                         # runtime's camera moves at; never read on a core that takes kind; goes with that branch
CLEAR_ARG = {"anyOf": [{"type": "boolean"}, {"type": "object", "properties": {"look": {"anyOf": [
    {"type": "string"}, {"type": "integer"}]}}, "required": ["look"]}],
    "description": "true when you looked and judge the camera's way clear (or {\"look\": id} for the look you "
                   "judged it in): recorded in the result; it changes no speed"}


def of(ex):
    return bounds.of(ex)


def _first_param(fn):
    try:
        return next(iter(inspect.signature(fn).parameters), None)
    except (TypeError, ValueError):
        return None


def limit(ex, name, *args, **kw):
    """The round three limit NAME in the unit its constant has, as this session derives it: r3_limits' lower case
    function of that name called with the session's bounds and the limit's own arguments (bracket_near_mm(B, model),
    bracket_reach_extra_mm(B, model), under_table_mm(B), device_box_near_mm(B, sigma_mm), span_min_deg(B,
    patch_px=...)); the constant NAME while that function does not exist (module text)."""
    fn = getattr(L, name.lower(), None)
    if callable(fn) and _first_param(fn) in BOUNDS_NAMES:
        return float(fn(bounds.of(ex), *args, **kw))
    return float(getattr(L, name))


def r3(ex, name, old, new=()):
    """r3_limits.<name>: called with the session's bounds and the arguments `new` when its first parameter is the
    bounds, else with the arguments `old` it takes today (module text)."""
    fn = getattr(L, name)
    if _first_param(fn) in BOUNDS_NAMES:
        return fn(bounds.of(ex), *new)
    return fn(*old)


def inputs(ex):
    """The resolution facts robot://body serves ({res_deg, pu_mm, pu_deg, source})."""
    return r3(ex, "inputs", (ex,))


def view_tol(ex, q):
    """The same-view tolerance {mm, deg, roll_deg} at joint pose q."""
    if _first_param(L.view_tol) in BOUNDS_NAMES:
        return L.view_tol(bounds.of(ex), q)
    cm, cd = r3(ex, "camera_step", (ex.model, q, inputs(ex)["res_deg"]), (q,))
    return L.view_tol(cm, cd)



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


def look_move(ex, q, intent=None, share=None):
    """The camera's own joint move to q: kind look, the derived full speed, the contact stop armed (module text).
    share (0 to 1): the share of the full joint speed a path check found the path may run at; None or 1 is full
    speed. Returns the core's stop dict or a refusal."""
    move = ex._joint_move
    q = np.asarray(q, dtype=float)
    slower = share is not None and share < 1
    if "kind" in inspect.signature(move).parameters:
        return move(q, float(share) if slower else None, kind="look", intent=intent)
    return move(q, OLD_CORE_SHARE * (float(share) if slower else 1.0), guarded=True)

def line_rate(ex, q_from, q_to):
    """The share of the straight joint line from q_from to q_to the core runs per second at full speed (1/s). The
    core of SPEC-R4 A8.2 runs every joint at most at its own rated or measured speed: the least qd_max[j] /
    abs(dq[j]) over the joints that move. A core that does not take kind yet runs its own plan: the rate is the
    largest step of that plan (TRANSITION, module text)."""
    q_from, q_to = np.asarray(q_from, dtype=float), np.asarray(q_to, dtype=float)
    dq = q_to - q_from
    big = float(np.max(np.abs(dq))) if dq.size else 0.0
    if not big > 0:
        return 0.0
    move = getattr(ex, "_joint_move", None)
    if move is None or "kind" in inspect.signature(move).parameters:
        qd = np.asarray(bounds.of(ex).qd_max(), dtype=float)
        moving = np.abs(dq) > 0
        return float(np.min(qd[moving] / np.abs(dq[moving])))
    path = [q_from] + [np.asarray(q, dtype=float) for q in ex._plan_joint(q_from, q_to, OLD_CORE_SHARE)]
    step = max(float(np.max(np.abs(b - a))) for a, b in zip(path[:-1], path[1:]))
    return step / big / float(ex.dt)


def tip_speed(ex, q_from, q_to, share=None):
    """The tip's speed (m/s) on the joint move q_from to q_to at that share of full speed: its straight travel
    times the line rate."""
    travel = float(np.linalg.norm(np.asarray(ex.model.fk_tip(np.asarray(q_to, dtype=float))[0], dtype=float)
                                  - np.asarray(ex.model.fk_tip(np.asarray(q_from, dtype=float))[0], dtype=float)))
    return travel * line_rate(ex, q_from, q_to) * (1.0 if share is None else float(share))


def gate(ex, q, share=None):
    """(refusal or None, share): the gates of the runtime's own joint move to q, asked the way an operator's
    move_joints is (SPEC-R3 3.8), with the tip speed the move will run at stated in the call (speed_mm_s), so a path
    gate that derives its clearance from the speed judges this move's own. A gate whose refusal states the speed it
    judged is asked again at half that speed, down to REL x REL of full speed; the move then runs at the share that
    passed. Any other refusal is returned at once."""
    q = np.asarray(q, dtype=float)
    q_now = np.asarray(ex._sense().q, dtype=float)
    share = 1.0 if share is None else float(share)
    q_deg = [round(float(np.degrees(x)), 4) for x in q]
    while True:
        args = {"q_deg": q_deg, "guarded": True, "speed_mm_s": round(tip_speed(ex, q_now, q, share) * MM, 3)}
        r = ex.gates("move_joints", args)
        if r is None:
            return None, share
        judged = any(isinstance(v, dict) and "speed_mm_s" in v for v in r.values())
        if not judged or share / R.DOUBLE < R.REL * R.REL:
            return r, share
        share = share / R.DOUBLE


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


def look_takes_clear(ex):
    """True when the core's own look primitive takes the operator's word (clear) itself."""
    fn = getattr(type(ex), "look", None)
    try:
        return "clear" in inspect.signature(getattr(fn, "__wrapped__", fn)).parameters
    except (TypeError, ValueError):
        return False


def res_rad(ex):
    """Each joint's measurement step (rad): robot://body's stated resolution, 0 where none is stated."""
    return r3(ex, "res_rad", (inputs(ex)["res_deg"], ex.model.n))


def camera(ex):
    """{"hidden", "noise"}: the hidden level and the noise (grey levels of 255) the camera showed in the robot's own
    still hold, for rcpx.perception's light levels; an entry with no measurement is left out."""
    try:
        answer = bounds.of(ex).know.get("camera")
    except (AttributeError, KeyError, TypeError):
        answer = None
    value = answer[0] if isinstance(answer, tuple) and answer else answer
    out = {}
    for key in ("hidden", "noise"):
        v = value.get(key) if isinstance(value, dict) else None
        v = v.get("value") if isinstance(v, dict) else v
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            out[key] = float(v)
    return out


def settle_step_s(ex):
    """The world seconds between the two frames a settle compares: r3_limits.settle_step_s(B) when it exists, else
    the constant FRAME_SETTLE_DT_S (the camera's frame interval)."""
    fn = getattr(L, "settle_step_s", None)
    if callable(fn) and _first_param(fn) in BOUNDS_NAMES:
        return float(fn(bounds.of(ex)))
    return float(L.FRAME_SETTLE_DT_S)


def settle_bound_s(ex):
    """The world seconds a settle is planned to take, with the margin every call's time carries (SPEC-R4 A4.8):
    r3_limits.settle_max_s(B) when it exists, else the arm's own measured settle time through limit_s; 0 when the
    robot has not measured one."""
    B = bounds.of(ex)
    fn = getattr(L, "settle_max_s", None)
    if callable(fn) and _first_param(fn) in BOUNDS_NAMES:
        return float(fn(B))
    try:
        t = B.t_settle()
    except (AttributeError, TypeError):
        t = None
    if not isinstance(t, (int, float)) or isinstance(t, bool) or not t > 0:
        return 0.0
    return float(B.limit_s(float(t)))


__all__ = ["binding", "camera", "clear_word", "gate", "inputs", "limit", "line_rate", "look_move",
           "look_takes_clear", "of", "periods", "r3", "report", "res_rad", "settle_bound_s", "settle_step_s",
           "tip_speed", "view_tol"]
