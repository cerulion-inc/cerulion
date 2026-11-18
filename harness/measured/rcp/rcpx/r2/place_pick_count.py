# SPDX-License-Identifier: AGPL-3.0-only
"""The count of a take of package place: what a take will cost, made before
anything moves.

The count is the control periods of the plan AS IT WILL RUN at its planned speeds, with no margin and no branch
that is not planned (no end wait, no second take):

- the way: the first of the take's ways in that the bounds plan (B.plan_legs: the call the stream itself makes);
- the close: as the core's close counts itself (rcpx/control_hold_close.py rates: the jaws' own rate down to
  where the pads can first meet the thing, then the rate solved for the hold's squeeze, lower under a stated
  limit);
- the test lift: the bounds' plan of its one leg from the grasp pose, under the limit the hold's record will
  carry (a limit stated for the thing bounds every leg that carries it);
- the load read at rest: the settle window (control_hold.read_load).

gate(): what a take earns before the hand or the jaws move: limit_not_keepable when a stated limit lies under
what these jaws leave standing once they touch (the close's own refusal, asked first), budget when the count is
over the control periods left (ex._pl_periods_left; none stated: nothing is refused). count() is also the
pick's price for the place's budget line (ex._pk_price), and the pick's receipt prints it beside the periods the
take ran (the waits by motion the plan does not count are listed there as owed, never gated).

The take of a hand with one moving jaw (a fixed jaw that carries the tool tip, a jaw that swings against it) is
here too. With no width to go by such a hand takes a thing at the point between its PADS, not at the fixed
jaw's tip and not at the middle between the jaw tips (which stands over the fixed tip while the jaw is open):
- pads_mid_T: that point in the tool frame at an opening, from the hand's own model;
- one_jaw_pose: the arm pose that puts it on the thing's middle, the tip at its height;
- follow: while the jaws close the hand moves with its jaw ALONG THE CLOSING AXIS ONLY, by the opening the jaws
  are told, so that the point stays where it is: both pads come to the thing together and the tip keeps its
  height.
Nothing here reads ground truth or names a robot.
"""

import math

import numpy as np

from rcpx import control_hold
from rcpx.context import refusal
from rcpx.r2.place_grasp import HALF, UP, num


def _ratios():
    from rcpx import bounds_ratios as R
    return R


def _bounds(ex):
    from rcpx import bounds
    return bounds.of(ex)


def lift_len(J, plan, way_down):
    """The test lift (m): twice what the fingers reach around the thing (their length, their reach past the tip,
    the hand's band), never longer than the way down."""
    low = J.low_m(J.frac_for_width(plan["open_m"]))
    return min(way_down, ((J.finger_len_m or J.pad_height_m or 0.0) + low + plan["band_m"]) / HALF)


def test_lift(J, plan, way_down, short_m=0.0):
    """The test lift as it is run (m): lift_len, and never less than what frees the thing (frees): a lift under
    that would test nothing, so where the way down was shorter the hand rises on past where its last leg began."""
    return max(lift_len(J, plan, way_down), frees(plan, short_m))


def frees(plan, short_m=0.0):
    """How far a lift must rise to free the thing from where it stood (m): how deep it stands between the
    fingers (the depth planned, less what the way down stopped short of it on an early contact) plus the hand's
    band."""
    return max(0.0, plan["depth_m"] - max(0.0, short_m)) + plan["band_m"]


def wide_of(plan):
    """The widest the thing can stand between the jaws (m), or None where its bearing is known: a width that was
    STATED and not seen says nothing of how the thing is turned, so the pads can first meet its diagonal (it is
    taken as that wide every way: place_grasp); a width the look showed, or the robot's own look between the
    jaws, is the extent across the bearing the jaws are planned on."""
    w = plan.get("expect_m")
    return None if w is None or plan.get("width_source") != "stated" else math.hypot(float(w), float(w))


def close_count(ex, plan, ready):
    """(the control periods the close plans, None), or (None, the refusal limit_not_keepable): the core's close
    as it counts and refuses itself (rcpx/control_hold_close.py rates), asked before the hand
    moves. The floor the jaw joint's own friction leaves and the jaws' stop slope are those of the depth the
    thing is taken at (a hinged jaw presses harder the nearer its hinge the thing stands). The operator's own
    word under that floor is never refused: the close runs at the slowest rate the jaws resolve and
    says the limit is not kept."""
    from rcpx import control_hold_close as C
    F, lim = control_hold.facts(ex), ready["lim"]
    depth = plan["depth_m"] if plan["depth_m"] > 0.0 else None
    N, lever = C.at_depth(F, ready["N"], depth)
    limit = lim.get("limit_n")
    frac = float(ready["open_frac"])
    pads = max(1, len(F["J"].sides))        # the thing sees every pad's push at once: each pad has its share
    P = C.rates(F, N, lever, frac, expect_m=plan.get("expect_m"), band_m=ready["band"], depth_m=depth,
                lim=None if limit is None else {"limit_n": float(limit) / pads, "source": lim.get("source")},
                wide_m=wide_of(plan))
    if P["under_floor"] and lim.get("source") != "intent":
        return None, refusal(
            "limit_not_keepable", f"its stated limit is {limit:g} N ({lim.get('source')}); these jaws cannot "
            f"press under {N['floor_n']:.2f} N once they touch (their joint's own friction over the pad's shortest "
            f"lever, {N['source']}): not started, nothing moved",
            f"nothing on this call: the limit is stated by the {lim.get('source')}")
    return C.planned_periods(P, frac, 0.0, float(ex.dt)), None


def close_strike(ex, plan, ready, c):
    """The close's strike as a strike block for the receipt's largest strike, or None where the jaws' stop
    slope is not known. The close states its peak: what one pad can make on a rigid thing backed by
    the other jaw at the rate the jaws ran at when their reading told the thing (the jaws' stop slope at the depth
    the thing is taken at; the slope's source is said: these jaws give no force reading, so it is the body's
    model) with the squeeze they then read standing: the close's own ONE number (control_hold_close.close)."""
    z, v = c.get("z_jaw"), c.get("met_m_s", c.get("arrive_m_s"))
    if not z or v is None or c.get("peak_n") is None:
        return None
    peak = float(c["peak_n"])       # ONE number: the close's own figure after the jaws stood
    st = c.get("strike") if isinstance(c.get("strike"), dict) else {}
    return {"peak_n": peak, "at_least_n": None, "model_peak_n": float(z) * float(v), "v_m_s": float(v),
            "f_guard_n": None, "z_n_s_m": float(z),
            "z_source": f"the jaws' own stop slope ({c.get('z_jaw_source') or ready['N']['source']}), per pad",
            "stops": 0, "v_max_m_s": None, "link": None, "gentle": None, "text": c["price"],
            # per pair: each pad on the thing against pad_n, a jaw body on anything else it
            # met while closing against other_n
            "pad_n": peak, "other_n": st.get("other_n"), "other_source": st.get("source"), "width": c.get("width")}


def _planned(ex, q0, legs, held, word):
    """The control periods the bounds plan for these legs from q0, or None when they plan none."""
    try:
        got = _bounds(ex).plan_legs(np.asarray(q0, dtype=float), [dict(leg) for leg in legs], held=held,
                                    intent=word or None)
    except Exception as exc:        # a plan the bounds cannot make: the stream then says why
        ex._journal("bounds_problem", where="pick count", error=f"{type(exc).__name__}: {exc}")
        return None
    return None if getattr(got, "refused", None) else len(getattr(got, "path", None) or [])


def count(ex, plan, ready, word=None):
    """{"periods", "way", "close", "lift", "rest", "route"}: the take as it will run (module docstring); the
    refusal limit_not_keepable; or None when the bounds plan no way in (the stream then says why)."""
    R, B, J = _ratios(), _bounds(ex), ex._pl_jaws
    close_n, bad = close_count(ex, plan, ready)
    if bad is not None:
        return bad
    way_n, route, length, q_from = None, None, 0.0, None
    q0 = ex._sense().q if ex._q_des is None else ex._q_des      # where the hand was told to be, as the stream plans
    for route, legs, length, _, q_from in ready["ways"]:
        way_n = _planned(ex, q0, legs, None, word)
        if way_n is not None:
            break
    if way_n is None:
        return None
    lim = ready["lim"]
    q_low = np.asarray(plan["q_low"], dtype=float)
    p_low, R_low = ex.model.fk_tip(q_low)
    away = "support" if plan.get("support_z") is not None else UP
    leg = ex._pk_leg("lift", np.asarray(p_low, dtype=float) + UP * test_lift(J, plan, length), R_low, R_low[:, 0],
                     q_low, away_from=away)
    if leg is None and q_from is not None:      # an arm of joint legs: back to where its last leg began
        leg = {"kind": "lift", "to": q_from, "away_from": away}
    first = {"weight_n": None, "mass_kg": 0.0, "extent_m": None, "proofs": None, "limit_n": lim.get("limit_n"),
             "limit_source": lim.get("source") if lim.get("limit_n") is not None else None}
    lift_n = 0 if leg is None else (_planned(ex, q_low, [leg], first, word) or 0)
    rest_n = max(int(R.QUANT), int(math.ceil(num(B.t_settle(), 0.0) / float(ex.dt))))
    return {"periods": int(way_n + close_n + lift_n + rest_n), "way": int(way_n), "close": int(close_n),
            "lift": int(lift_n), "rest": int(rest_n), "route": route}


def gate(ex, plan, ready, word=None, got=None):
    """None, or the refusal a take earns before anything moves (module docstring). got: the take's count (count();
    None: no way in is planned, and the stream then says why). The hint of a budget refusal is ONE ready call
    where one helps (the same pick with no limit stated, when that fits), else nothing."""
    left = ex._pl_periods_left()
    if got is None:                 # no count: the close's own refusal is still asked before the hand moves
        got = close_count(ex, plan, ready)[1]
    if isinstance(got, dict) and "refused" in got:
        return got
    if left is None or got is None or got["periods"] <= left:
        return None
    hint = "nothing: this is the take's own plan at the body's rate"
    lim = ready["lim"]
    if lim.get("limit_n") is not None and lim.get("source") == "intent":
        bare = [(w, [dict(leg, expect=dict(leg["expect"], bound_n=None, bound_source=None)) if leg.get("expect")
                     else leg for leg in legs], n, p, q) for w, legs, n, p, q in ready["ways"]]
        free = count(ex, plan, dict(ready, lim={"limit_n": None, "source": None}, ways=bare), word)
        if isinstance(free, dict) and "refused" not in free and free["periods"] <= left:
            hint = f"leave gentle out: with no limit stated the take plans {free['periods']} control periods"
    return refusal("budget", f"this take plans {got['periods']} control periods (the way to the thing "
                   f"{got['way']}, the close {got['close']}, the test lift {got['lift']}, the load read at rest "
                   f"{got['rest']}) and {left} are left: not started, nothing moved", hint)


# ---------------------------------------------------------------------------------------------- one moving jaw
def pads_mid_T(J, frac):
    """One moving jaw: the point between the pads at opening frac, in the tool frame (m): on the closing axis
    through the tip, midway along that axis between the two jaw points. It moves along the closing axis alone as
    the jaw closes (the middle of the jaw points themselves stands over the fixed tip while a hinged jaw is open:
    a hand that keeps that point where it is rises as its jaw closes, and closes over the thing's top)."""
    pts = J.points_tool(frac)
    return J.axis_T * (HALF * float((pts[0] + pts[-1]) @ J.axis_T))


def one_jaw_pose(ex, p, c_T, t, R0, yaw, seeds):
    """Joints that put the hand-fixed point c_T (tool frame) on p with the tool pointing along t: (q, R reached,
    the jaws' bearing reached, squared). The whole hand rotation R0 first (squared: the jaws close along yaw); an
    arm of fewer joints than a pose has freedoms: the pointing axis alone, its jaws turned as its pose allows."""
    for R in (R0, None) if ex.model.n < len(p) + len(p) else (R0,):
        for q0 in seeds:
            q = None if q0 is None else ex._pl_ik_held(np.asarray(q0, dtype=float), p, c_T, t, R)
            if q is not None:
                Rq = ex.model.fk_tip(q)[1]
                return q, Rq, (yaw if R is not None else ex._pl_jaw_bearing(Rq)), R is not None
    return None, None, None, False


def follow(ex, q_from=None):
    """One moving jaw: the arm's own target for each control period of a close (control_hold.close, arm=), so that
    the point between the pads (pads_mid_T at the opening the jaws are TOLD: the hand trails its own command as
    the jaws trail theirs, and where the hand leads, the fixed pad, which stands square, meets the thing first)
    stays where it stands in the world, the tool pointing as it points: the hand moves along its closing axis
    only, the tip at its height. Once the jaws have run and then stand (a period's travel under their own still
    band: they are on something, or shut) the hand stands where it is read, whatever the command does after. It
    starts from the joints q_from (a hand that backs off a contact first), else from where the hand was told to
    be. Where the arm cannot take a pose it stands where it last could. None for jaws that move together, or
    jaws that give no reading."""
    J = ex._pl_jaws
    s = ex._sense()
    q0, _ = J.reading(s.gripper, ex._gripper_cmd)
    if not J.one_jaw or q0 is None:
        return None
    full = ex.model.n >= len(s.tip) + len(s.tip)       # an arm that can hold a whole pose keeps its rotation
    last = [np.asarray(q_from if q_from is not None else s.q if ex._q_des is None else ex._q_des, dtype=float).copy()]
    tip, R0 = (s.tip, s.R) if q_from is None else ex.model.fk_tip(last[0])
    R0 = np.asarray(R0, dtype=float).copy()
    centre = np.asarray(tip, dtype=float) + R0 @ pads_mid_T(J, J.frac_of_q(q0))

    still, seen = float(control_hold.numbers(ex)["still_band"]), {"frac": None, "ran": False, "stands": False}

    def arm(cmd):
        frac = control_hold.read(ex)[0]
        if not seen["stands"] and frac is not None and seen["frac"] is not None:
            moved = seen["frac"] - frac > still
            if seen["ran"] and not moved:               # the jaws ran and now stand: the hand stands where it is
                seen["stands"], last[0] = True, np.asarray(ex._last.q, dtype=float).copy()
            seen["ran"] = seen["ran"] or moved
        seen["frac"] = frac
        if seen["stands"]:
            return last[0]
        q = ex._pl_ik_held(last[0], centre, pads_mid_T(J, cmd), R0[:, 0], R0 if full else None, restarts=0)
        if q is not None:
            last[0] = np.asarray(q, dtype=float)
        return last[0]
    return arm
