# SPDX-License-Identifier: AGPL-3.0-only
"""Park from any pose: lift the tool straight up, guarded, when the plain park cannot run.

The round one park streams a straight line in joint space. From a pose resting on the surface the arm stands on
that line can start by dipping a link (the path check then refuses the park before it moves) or by dragging the
tool along the surface (the guard then stops it part way). Two cases, and only these, get a lift:
- BEFORE the park, when its own path check would refuse it for the surface: lift_clear() raises the tool tip
  straight up, guarded, until the tool is clear and the park's path check passes;
- AFTER a park that started from the surface (the tool low over it or resting on it), stopped short on a
  contact and is still on the surface: the same lift, then the park again; the result carries first_try. (A park
  that left the surface and met something on its way is not tried again: it would touch that thing twice.)
A park whose plain path runs is left alone: the lift knows the surface the arm stands on and nothing else, and an
arm lying across a thing on that surface can swing a link into it while the tip rises (measured: a forearm
struck the object it lay on with 87 N where the plain park left it with 1 N).

The lift is ONE rise of kind lift (the core's Cartesian move: its path check, its guard, made directional away
from the support, and its history all apply; no typed speed: the derived full speed) to the height at which the
tool is no longer low: margin() above the surface. A further rise is made only when one stopped short or the
park's path is still refused, then by DOUBLE the rise so far (the search step of rcpx.bounds_ratios); there is no
typed top: the lift ends on the move's own limit stop or refusal, or after MAX_MOVES. When the tool cannot rise
straight up (the arm lies folded against its own links, or something holds it), joint_by_joint() goes to the rest
pose one joint at a time in the order the body model says dips the arm least.

"Low" is derived: the tool is within one standoff of the surface (rcpx.bounds standoff: the stop distance at the
speed at which the tip may arrive at the support, the support's position band and the tool's half width).
"Resting" is a downward force over what the robot can tell from none. "Moved" and "stopped short" are judged
against the least travel the robot can tell from none and the arm's rest angle (waiting_limits.py). Nothing here
names a robot: the rest pose, the capsules, the surface and every margin come from the executor, its body model
and rcpx.bounds.

The lift and the joint route are legs of park, so they run after a dark look as park itself does (as_park): the
round one rule that refuses moves toward the world while the last look was dark would otherwise refuse every one
of them, and a park in the dark from the table would be the plain park again. The operator's word (clear) given
to park, or to wait_for_light, is passed into every move made here.
"""

import contextlib
import math
import traceback

import numpy as np

from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx import bounds_laws as laws
from rcpx.r2 import waiting_limits as limits

MM = 1000.0
MAX_MOVES = 16           # lift moves in one park at most (a loop bound)
UP = np.array([0.0, 0.0, 1.0])
DOWN = -UP


@contextlib.contextmanager
def as_park(ex):
    """Run the moves inside as legs of park. Round one refuses moves toward the world while the last look was
    dark and lets park, home, retreat and ease run; the lift (straight up, guarded) and the joint route (to the
    rest pose, guarded) are park's own legs, so the dark mark is set aside for them and put back after."""
    mark = getattr(ex, "_dark_look", None)
    ex._dark_look = None
    try:
        yield
    finally:
        ex._dark_look = mark


def tool_clearance(ex, q):
    """Lowest point of the tool (its capsules and the tip) above the surface under it (m)."""
    tip = ex.model.fk_tip(q)[0]
    low = float(tip[2] - ex._floor_at(tip))
    for name, p0, p1, r in ex.model.collision_capsules(q)[1:]:
        if name in ex._tool_bodies:
            low = min(low, ex._cap_clearance(p0, p1, r))
    return low


def margin(ex, q):
    """m: how far over the surface the tool must be not to be low: one standoff at the speed at which the tip may
    arrive at the support there (its stop distance, the support's position band and the tool's half width)."""
    B = bounds.of(ex)
    return float(B.standoff(B.v_in("support", q, DOWN)[0], thing="support", q=q, u=DOWN))


def joint_line(ex, q, q1):
    """The straight joint line from q to q1 as a list of poses (without q), sampled so that the tip moves no more
    than the spacing a path check needs for that margin between two samples."""
    B = bounds.of(ex)
    spacing = B.spacing(margin(ex, q))
    distance = float(np.linalg.norm(ex.model.fk_tip(q)[0]))
    turn = float(np.max(np.abs(q1 - q)))
    n = max(1, int(math.ceil(turn * distance / spacing))) if spacing > 0 else 1
    return [q + (q1 - q) * (k / n) for k in range(1, n + 1)]


def park_check(ex, q):
    """The path check of a park from q, on a straight joint line built here: None when it passes, else its
    refusal."""
    q = np.asarray(q, float)
    return ex._path_check([q] + joint_line(ex, q, np.asarray(ex._named_poses()["park"], float)), True)


def need(ex, s):
    """m: the height over the surface the lift takes the tool to: where it is no longer low."""
    return margin(ex, s.q)


def read_move(ex, r):
    """(refused, stop, last sense, contact, why) of a core move's answer, whatever shape it has."""
    refused = "refused" in r
    stop = r.get("stopped", r.get("stop"))
    s = r.get("s") or ex._sense()
    contact = r.get("contact")
    why = r.get("why") or (r.get("refused") or {}).get("why") or ""
    return refused, stop, s, contact, why


def assess(ex, s):
    """Where the park starts from: {needs, why, tool_clear_m, park_refusal}. park_refusal: the refusal of the
    park's own path check for the surface (the lift runs before the park); needs: that, or the tool low over the
    surface or resting on it (the lift goes on until none holds; a park from there that stops short is lifted
    and tried again)."""
    bad = park_check(ex, s.q)
    plane = bad is not None and bad["refused"].get("check") == "path_plane"
    clear = tool_clearance(ex, s.q)
    B = bounds.of(ex)
    room = margin(ex, s.q)
    low = clear < room
    F0 = B.f_thr(0.0, 0.0)
    resting = s.Fmag > F0 and float(s.F[2]) < -R.GIVE * F0
    why = []
    if plane:
        why.append("the straight joint path to the rest pose is refused: " + bad["refused"]["why"])
    if low:
        why.append(f"the tool is {max(clear, 0.0) * MM:.0f} mm above the surface under the arm (inside the "
                   f"{room * MM:.0f} mm one standoff needs)")
    if resting:
        why.append(f"the tip rests on something ({s.Fmag:.1f} N down)")
    return {"needs": bool(plane or low or resting), "why": "; ".join(why), "tool_clear_m": clear,
            "park_refusal": bad if plane else None}


def lift_clear(ex, forced=None, intent=None):
    raw = assess(ex, ex._sense())
    s, _ = ex._ease_if_pressing()
    a = assess(ex, s)
    if raw["needs"] and not a["needs"]:
        a = {**a, "needs": True, "why": raw["why"] + " (read before easing)"}
    if a["park_refusal"] is None and forced is None:
        return None, a
    s = ex._ease_impl()
    z0 = float(s.tip[2])
    B = bounds.of(ex)
    required = need(ex, s)
    rec = {"why": "; ".join(x for x in (forced, a["why"]) if x), "lifted_mm": 0.0,
           "moves": 0, "ended": "max_moves", "contact": None, "start": (s.q.copy(), s.tip.copy()),
           "need_mm": round(required * MM, 1),
           "limits": {"need_mm": {"value": round(required * MM, 1),
                                  "source": "one standoff over the support: the stop distance at the speed the "
                                            "tip may arrive there, its position band and the tool's half width"}}}
    first, keep, stuck = a, True, 0
    # the rise ends the least travel the robot can tell from none over the height needed, so that the next
    # reading says clear and not low by rounding
    h = max(0.0, required - tool_clearance(ex, s.q)) + B.rigid(s.q, UP)
    for _ in range(MAX_MOVES):
        before_z = float(s.tip[2])
        r = ex._cart_move(s, np.array([0.0, 0.0, h]), None, None, True, keep,
                          kind="lift", away_from="support", intent=intent)
        refused, stop, s, contact, why = read_move(ex, r)
        rec["moves"] += 1
        if contact is not None:
            ex._record_contact(contact, "park lift guard")
        rise = float(s.tip[2]) - before_z
        no_progress = laws.no_progress(rise, h, B.quantum(s.q, UP)) or (rise == 0 and h == 0)
        if no_progress:
            if (refused or stop == "limit") and keep:
                keep = False
                continue
            stuck += 1
            if refused or stop == "limit" or stuck >= 2:
                rec["ended"], rec["contact"], rec["lift_why"] = "refused" if refused else stop, contact, why
                break
            continue
        stuck = 0
        a = assess(ex, s)
        required = need(ex, s)
        if not a["needs"] and a["tool_clear_m"] >= required:
            rec["ended"] = "clear"
            break
        if a["park_refusal"] is not None and a["tool_clear_m"] >= required and stop == "goal":
            rec["ended"] = "path_still_refused"
            break
        total = float(s.tip[2]) - z0
        h = max(R.DOUBLE * total, max(0.0, required - a["tool_clear_m"]) + B.rigid(s.q, UP))
    s = ex._sense()
    bad = park_check(ex, s.q)
    rec["lifted_mm"] = round((float(s.tip[2]) - z0) * MM, 1)
    rec["tool_clear_mm"] = round(tool_clearance(ex, s.q) * MM, 1)
    rec["park_path"] = "clear" if bad is None else bad["refused"]["why"]
    rec["park_refusal"] = bad
    return rec, first


def joint_by_joint(ex, intent=None):
    """The fallback when the tool cannot rise straight up (its own links in the way, or held down): go to the rest
    pose one joint at a time, each time the joint whose move alone passes the path check and dips the arm least
    toward the surface under it (the body model decides the order; every move is guarded). Returns {order, ended}:
    order names the joints whose move ran (a refused move is not in it); ended is "done", "no_safe_joint",
    "refused" (with why) or the stop of the move that ended it."""
    m = ex.model
    q_park = np.asarray(ex._named_poses()["park"], dtype=float)
    q = ex._sense().q.copy()
    left = [j for j in range(m.n) if abs(q_park[j] - q[j]) > limits.rest_angle(ex, q)[0]]
    out = {"order": [], "ended": "done"}
    while left:
        c0, best = ex.floor_clearance(q), None
        for j in left:
            q1 = q.copy()
            q1[j] = q_park[j]
            path = joint_line(ex, q, q1)
            if not path or ex._path_check([q] + path, True) is not None:
                continue
            dip = max(0.0, c0 - min(ex.floor_clearance(x) for x in path))
            t0 = tool_clearance(ex, q)
            tool_dip = max(0.0, t0 - min(tool_clearance(ex, x) for x in path))
            room = margin(ex, q)
            # a move that lowers the arm, or the tool, to within one standoff of the surface (or under it) is
            # not safe: the dip must end above the surface with that room to spare
            if (dip > 0 and c0 - dip < room) or (tool_dip > 0 and t0 - tool_dip < room):
                continue
            key = (max(dip, tool_dip), -ex.floor_clearance(q1))
            if best is None or key < best[0]:
                best = (key, j, q1, dip)
        if best is None:
            out["ended"] = "no_safe_joint"
            break
        _, j, q1, _ = best
        r = ex._joint_move(q1, None, dark_ok=True, kind="park", intent=intent)
        refused, stop, _, _, why = read_move(ex, r)
        if refused:
            out["ended"], out["why"] = "refused", why
            break
        out["order"].append(m.joint_names[j])
        if stop != "goal":
            out["ended"] = stop
            break
        q = ex._sense().q.copy()
        left.remove(j)
    return out


def prepare(ex, forced=None, intent=None):
    """The lift and, when that could not clear the tool, the joint by joint route. Returns (rec, a): rec is None
    when nothing was needed (the park's own path check passes and nothing is forced), else the record a result
    carries as lifted_first; a is the assessment before anything moved. moved is measured: the tip or a joint is
    not where it stood when the lift began."""
    with as_park(ex):
        rec, a = lift_clear(ex, forced, intent)
        if rec is not None and rec["ended"] != "clear":
            rec["joint_by_joint"] = joint_by_joint(ex, intent)
            s = ex._sense()
            bad = park_check(ex, s.q)
            rec["tool_clear_mm"] = round(tool_clearance(ex, s.q) * MM, 1)
            rec["park_path"] = "clear" if bad is None else bad["refused"]["why"]
            rec["park_refusal"] = bad
    if rec is not None:
        q0, tip0 = rec["start"]
        s = ex._sense()
        rec["moved"] = bool(float(np.linalg.norm(s.tip - tip0)) > bounds.of(ex).rigid(q0, UP) or
                            float(np.max(np.abs(s.q - q0))) > limits.rest_angle(ex, q0)[0])
    return rec, a


def public(rec):
    """The lift record as a result carries it (the refusal object and the start reading stay inside)."""
    return None if rec is None else {k: v for k, v in rec.items() if k not in ("park_refusal", "moved", "start")}


def lifted_only(ex, rec, bad):
    """The answer when the park's path is still refused after the arm moved: a result, never a refusal (a refusal
    says nothing changed). bad: the body of the refusal."""
    s = ex._sense()
    return {"pose": "park", "stopped": "lifted_only", "why": bad["why"], "hint": bad["hint"],
            "lifted_first": public(rec), "tip_mm": [round(float(x) * MM, 2) for x in s.tip]}


def before(ex, intent=None):
    """What runs before the round one park. Returns (rec, low): rec is the lift record when the park's own path
    check refused and the tool was lifted first, else None; low is why the park starts from the surface (the
    tool low over it or resting on it) when it does and nothing was lifted, else None."""
    rec, a = prepare(ex, intent=intent)
    return rec, (a["why"] if rec is None and a["needs"] else None)


def stopped_short(result, ex):
    return (isinstance(result, dict) and result.get("stopped") in ("contact", "cap")
            and float(result.get("error_deg") or 0.0) > math.degrees(limits.rest_angle(ex, ex._sense().q)[0]))


def after(ex, result, rec, low, intent=None):
    """What runs after the round one park. A park that started from the surface (low), stopped short on a
    contact and still has its tool low over the surface or resting on it is lifted and tried again; the result
    of the second park carries first_try. A lift record is attached as lifted_first."""
    if not isinstance(result, dict):
        return result
    # a park that stopped on a link of its own arm, or on a part of the tool away from its tip, dragging on the
    # surface is lifted too, even when it started clear of the surface (with firm friction a link no longer
    # slides off it)
    contact = result.get("contact") or {}
    dragged = contact.get("source") == "arm or self" or contact.get("at_tip") is False
    normal = (result.get("contact") or {}).get("normal_est")
    above = normal is not None and float(normal[-1]) < 0
    if rec is None and not above and (low or dragged) and stopped_short(result, ex) and assess(ex, ex._sense())["needs"]:
        first = {k: result[k] for k in ("stopped", "error_deg") if k in result}
        rec, _ = prepare(ex, forced=f"the park from the surface stopped on {result['stopped']} "
                                    f"{float(result['error_deg']):g} deg from the rest pose", intent=intent)
        again = _park(ex, intent)
        if "refused" not in again:
            result = {**again, "first_try": first}
        elif rec["moved"]:                 # the arm moved: never a refusal
            result = {**lifted_only(ex, rec, again["refused"]), "first_try": first}
        else:                              # nothing moved since the first try: its stop is still the answer
            result = {**result, "second_try": {"refused": again["refused"]}}
    elif rec is not None and "refused" in result and rec["moved"]:
        result = lifted_only(ex, rec, result["refused"])       # the arm moved: never a refusal
    if rec is not None and "lifted_first" not in result:
        result = {**result, "lifted_first": public(rec)}
    return result


def park_from_anywhere(ex, intent=None):
    """before, the round one park, after (for callers inside a primitive, where hooks do not run). A failure
    inside the lift is journaled and the plain park runs, as the hook core does for a hook that raises: the
    caller's tool (a wait for light) must not fail because the lift could not be made."""
    try:
        rec, low = before(ex, intent)
    except Exception:
        ex._journal("r2_problem", package="waiting", where="park lift", trace=traceback.format_exc())
        rec, low = None, None
    return after(ex, _park(ex, intent), rec, low, intent)


def _park(ex, intent):
    """The round one park, with the operator's word when the call carried one."""
    clear = (intent or {}).get("clear")
    return ex.park() if clear is None else ex.park(clear=clear)


def setup(ex, st):
    def pre(name, kwargs):
        intent = {"clear": kwargs["clear"]} if kwargs.get("clear") is not None else None
        rec, low = before(ex, intent)
        st["park_lift"] = (rec, low)
        if rec is None or rec["park_refusal"] is None or not rec["moved"]:
            return None                    # park runs (and answers for itself when nothing was lifted)
        st["park_lift"] = None
        return ("skip", lifted_only(ex, rec, rec["park_refusal"]["refused"]))

    def post(name, kwargs, result):
        rec, low = st.get("park_lift") or (None, None)
        st["park_lift"] = None
        out = after(ex, result, rec, low, {"clear": kwargs["clear"]} if kwargs.get("clear") is not None else None)
        return None if out is result else out

    ex.add_pre(pre, names=["park"])
    ex.add_post(post, names=["park"])
