# SPDX-License-Identifier: AGPL-3.0-only
"""Round three limits (SPEC-R3.md 3.7 and the rules of sections 3 and 4, as SPEC-R4 derives them). No number that
bounds speed, force, travel or time is typed here: each is a function of the bounds object B (rcpx.bounds.of(ex),
SPEC-R4 A8.1) and of the eight ratios of rcpx/bounds_ratios.py, so it follows the robot's own ratings, what it
measured on itself, what contact showed and what the operating model stated. Pure: no motion, no scene, no robot
named. This module is the one place where round three's packages reach the bounds object.

Every limit is name(B, q=None, u=None, **kw): B the bounds object, q the joint pose, u the unit direction (base
frame) where the law depends on them, the rest by keyword; it returns the value in the unit its old constant had
(mm, N, s, deg). The numbers kept below are word rules, image numerics and counts, not limits. The names and the
old call forms that files outside work package 06 of round four still read stand in one block at the end.

    bounds_of(ex), ratio(name), thing_of(B, ref, sigma_mm)
    rigid_mm, small_mm, quantum_mm, noise_n, f_touch, thresholds        the 3.7 thresholds
    force_spec, f0                                                       the ramp and its ceiling (A4.5)
    standoff_mm, touch_reach_mm, hand_lands_mm, blocked_mm, same_spot_mm, cue_near_mm ...   margins (A4.3)
    confirm_*, fit_*, stroke_mm, retract_*, lift_*, settle_*, noise_*    the stages of push and lift
    view_tol, miss_mm, span_min_deg, view_distance_mm, bracket_near_mm   the camera
    camera_step(model, q, res_deg), inputs(ex, body, B), res_rad         formulas over robot://body
"""
import functools
import math

import numpy as np

MM = 1000.0

from rcpx.r3_numerics import *  # noqa: F401,F403  the kept word rules, image numerics and counts (not limits)

# ---------------------------------------------------------------- the bounds object and the ratios
def bounds_of(ex):
    """The executor's bounds: ex.bounds when it is set, else rcpx.bounds.of(ex) (SPEC-R4 A8.1)."""
    B = getattr(ex, "bounds", None)
    if B is not None:
        return B
    from rcpx import bounds
    return bounds.of(ex)

def ratio(name):
    """One of the eight ratios of SPEC-R4 A7 by name."""
    from rcpx import bounds_ratios
    return getattr(bounds_ratios, name)

def _laws():
    from rcpx import bounds_laws
    return bounds_laws

def _either(*names):
    """A limit answers both call forms: name(B, q, u, **kw) and, for a caller that passes the limit's own
    arguments by position, name(B, a, b, ...) with a, b the arguments named here. The first form is read when the
    first two positional arguments are each None or an array (a pose and a direction)."""
    def wrap(fn):
        @functools.wraps(fn)
        def call(B=None, *args, **kw):
            pose = len(args) >= len(("q", "u")) and all(a is None or isinstance(a, (list, tuple, np.ndarray))
                                                        for a in args[:len(("q", "u"))])
            if pose or not args:
                return fn(B, *args, **kw)
            kw.update(zip(names, args))
            return fn(B, None, None, **kw)
        return call
    return wrap

def half(x):
    """Half of a length (a radius from a size); the 2 is rcpx.bounds_laws' numeric, not a ratio."""
    return float(x) / _laws().TWO

def _get(value, name, default=None):
    return value.get(name, default) if isinstance(value, dict) else getattr(value, name, default)

def _model(B):
    model = getattr(B, "model", None)
    if model is None:
        model = _get(getattr(B, "robot", None), "model")
    return model if model is not None else _get(getattr(B, "ex", None), "model")

def thing_of(B, ref=None, sigma_mm=None):
    """What B.thing(ref) knows of a world entry, a point or None, with the caller's own position error (mm) where
    the entry carries none: a located entry states error_mm, which the bounds object does not read."""
    E = None if ref is None else B.thing(ref)
    if sigma_mm is None or (E is not None and _get(E, "sigma_p") is not None):
        return E
    fields = dict(getattr(E, "__dict__", None) or {}) if E is not None else {}
    fields["sigma_p"] = float(sigma_mm or 0) / MM
    return B.thing(fields)

def _sigma_mm(thing, sigma_mm=None):
    return float(sigma_mm or 0) if sigma_mm is not None else float(_get(thing, "sigma_p") or 0) * MM
def _w_tool_mm(B):
    return B.standoff(0.0) * MM       # at speed 0 with no thing the standoff law leaves the tool's half width
def _err_mm(B, q, u, v_mm_s):
    return _laws().contact_error(float(v_mm_s or 0) / MM, B.t_det(1), B.quantum(q, u)) * MM

def _src(B, name):
    src = getattr(B, "src", None)
    return src(name)[0] if callable(src) else "model"

# ---------------------------------------------------------------- 3.7 resolution and thresholds
def rigid_mm(B, q=None, u=None, force_n=None, **kw):
    """QUANT x quantum + jitter + F / k_servo (no floor); force None = the touch threshold's own give."""
    return B.rigid(q, u, force_n) * MM

def moved_mm(B, q=None, u=None, v_mm_s=None, force_n=None, thing=None, **kw):
    """The least travel that shows a part moved under a push, never zero: the arm's own resolution and give
    (rigid); on a robot whose readings never change (rigid 0), the error with which the touch that found the part
    knows where it began (v x t_det + quantum, v the touch's own speed, else the arrival speed the bounds give);
    with no speed either, REL of the tool's half width (the relative form of SPEC-R4 A1.1; closes CK-08)."""
    rigid = rigid_mm(B, q, u, force_n)
    if rigid > 0:
        return rigid
    err = _err_mm(B, q, u, B.v_in(thing, q, u)[0] * MM if v_mm_s is None else v_mm_s)
    return err if err > 0 else ratio("REL") * _w_tool_mm(B)

def small_mm(B, q=None, u=None, force_n=None, **kw):
    return ratio("QUANT") * rigid_mm(B, q, u, force_n)

def quantum_mm(B, q=None, u=None, **kw):
    """The tip's measurement step along u: the stated resolution, else what the robot measured (0 is allowed)."""
    return B.quantum(q, u) * MM

def noise_n(B, q=None, u=None, v=0.0, a=0.0, **kw):
    return B.sigma_f(v, a)

def f_touch(B, q=None, u=None, v=0.0, a=0.0, readings=1, local=None, **kw):
    """F_thr = hidden level + (ONE_READ on one reading, TWO_READ on two) x sigma; local = (hidden, sigma) of the
    caller's own still hold: the larger of the two stands."""
    thr = B.f_thr(v, a, readings)
    if local is not None:
        thr = max(thr, local[0] + ratio("ONE_READ" if readings == 1 else "TWO_READ") * local[1])
    return thr

def ladder_round_n(B, q=None, u=None, **kw):
    return B.sigma_f(0, 0)

def force_spec(B, kind="press", thing=None, q=None, u=None, intent=None, sheet_forces=None, moved_n=None,
               touched_n=None, local=None):
    """The ramp and its ceiling (SPEC-R4 A4.5): {first_n, rate_n_s(F), ceiling_n, source, items, refused}. B.force is
    asked first; the facts the caller holds (the sheet's forces, the largest force that moved this part or another
    part of its device, the still hold's noise) complete it. No sheet force, no stated force, nothing that moved
    and no firm word: NOT refused (the design rulings, C9): the ceiling is what the arm can push and the ramp runs until the
    part moves (round four refused force_unknown here; the core's press says so in its notes and offers max_n)."""
    from rcpx import bounds_report
    intent = dict(intent or {})
    forces = [float(f) for f in (sheet_forces or []) if isinstance(f, (int, float)) and not isinstance(f, bool)
              and f > 0]
    if forces and "sheet" not in intent:
        intent["sheet"] = {"force_high_n": max(forces)}
    ex = getattr(B, "ex", None)
    if ex is not None and getattr(ex, "world", None) is None:
        thing, spec = None, None      # a text built outside a session: no world, so no stored thing and no sheet
    else:
        spec = B.force(kind, thing, q, u, intent)
    sigma = max(B.sigma_f(0, 0), local[1] if local is not None else 0)
    e_tau = B.e_tau()
    robot = [(B.f_cap(q, u) * (1 - e_tau), "model", "what the arm can push along this line here, less the torque "
              "estimate's error", "nothing: the arm's own rating")]
    if B.f_tool() is not None:
        robot.append((B.f_tool(), _src(B, "f_tool"), "the tool's own rating", "nothing: the tool's own rating"))
    terms = list(robot)
    if forces:
        terms.append((max(forces) + ratio("TWO_READ") * sigma, "sheet", "the sheet's highest force for this part "
                      "plus what the force estimate cannot tell apart", "state max_n"))
    stated = [k for k in ("max_n", "force_n") if intent.get(k) is not None]
    for k in stated:
        terms.append((float(intent[k]), "intent", f"{k} as you stated it", f"state a larger {k}"))
    moved = moved_n if moved_n is not None else _get(thing, "f_moved")
    if not forces and not stated and moved is not None:
        terms.append((float(moved), "contact", "the largest force at which this part or another part of its device "
                      "moved", "state max_n, or firm true"))
    least = min(t[0] for t in terms)
    own = _get(spec, "ceiling_n")
    if own is not None and own < least and not math.isclose(own, least):
        it = (_get(spec, "items") or [{}])[0]       # a term only the bounds object holds (base give, a hold's fuse)
        terms.append((own, _get(it, "source", "model"), _get(it, "evidence", "the bounds object's own term"),
                      _get(it, "lift", "state max_n")))
    firm = intent.get("firm") is True
    terms.sort(key=lambda t: t[0])
    if firm and not forces and not stated and moved is None:
        terms[0] = (terms[0][0], "intent", "firm, on your word: the arm's own rating alone", terms[0][3])
    items = [bounds_report.item("force", t[0], t[1], t[2], t[3]) for t in terms]
    # the design rulings, C9: nothing known of the force this part bears is no reason to refuse (a tool never refuses because
    # it is unsure): the ceiling stands with its source, and the ramp runs until the part moves
    refused, ceiling, source = None, terms[0][0], terms[0][1]
    first = max(touched_n or 0, _get(spec, "first_n") or 0, f_touch(B, readings=2, local=local))
    if ceiling is not None:
        first = min(first, ceiling)
    t_det = B.t_det()
    return {"first_n": first, "rate_n_s": lambda F: _laws().ramp_rate(F, sigma, e_tau, t_det, ceiling or 0),
            "ceiling_n": ceiling, "source": source, "items": items, "refused": refused, "sigma_n": sigma,
            "e_tau": e_tau, "t_det_s": t_det}

def f0(B, q=None, u=None, thing=None, intent=None, touched_n=None, local=None, forces=None, noise_n=None, **kw):
    """The ramp's first force: what the finding touch already put on the part, never under F_thr. forces (the
    sheet's) and noise_n (a caller's own noise reading) are accepted for callers that hold them."""
    if local is None and noise_n is not None:
        local = (0.0, float(noise_n))
    return force_spec(B, "press", thing, q, u, intent, forces, touched_n=touched_n, local=local)["first_n"]

def _view_depth_mm(B, q):
    """How far the scene lies from the camera at q: along its view axis to the support plane; where the axis does
    not meet the plane, the reach the model states."""
    model = _model(B)
    p, R = model.fk(np.asarray(model.q_home if q is None else q, dtype=float))["camera"]
    axis = -np.asarray(R, dtype=float)[:, -1]
    support = B.space.support()
    if support and support[0] is not None:
        gap = float(support[0][1]) - float(np.asarray(support[0][0], dtype=float) @ np.asarray(p, dtype=float))
        closing = float(np.asarray(support[0][0], dtype=float) @ axis)
        if gap < 0 and closing < 0:
            return gap / closing * MM
    return float((model.reach() or {}).get("max_reach_mm") or 0.0)

def view_tol(B, q=None, u=None, depth_mm=None, **kw):
    """Same view {mm, deg, roll_deg}: what the robot cannot tell from none (QUANT x the camera's own motion for one
    quantum on every joint) plus what the frame comparison itself lines up (ALIGN_SHIFT_PX pixels at the scene's
    depth, ALIGN_TURN_DEG of roll). It follows the camera and its readings and does not fall to zero on a robot
    whose readings never change (CK-08)."""
    model = _model(B)
    cm, cd = camera_step(model, q, [math.degrees(v) for v in B.joint_quantum()])
    k, f_px = ratio("QUANT"), float(model.f_px)
    depth = _view_depth_mm(B, q) if depth_mm is None else float(depth_mm)
    return {"mm": k * cm + ALIGN_SHIFT_PX * depth / f_px, "deg": k * cd + math.degrees(ALIGN_SHIFT_PX / f_px),
            "roll_deg": k * cd + ALIGN_TURN_DEG}

def miss_mm(B, q=None, u=None, depth_mm=0.0, f_px=None, pu_mm=0.0, pu_deg=0.0, **kw):
    """The most a ray may miss a fit at that depth: QUANT pixels there and the camera's pose uncertainty there."""
    depth = max(0.0, float(depth_mm or 0))
    px = ratio("QUANT") * depth / float(f_px) if f_px else 0.0
    return math.hypot(px, float(pu_mm or 0) + depth * math.radians(float(pu_deg or 0)))

def thresholds(B, q=None, u=None, **kw):
    """Every threshold at q along u at once, with the source words the bounds object gives."""
    cm, cd = camera_step(_model(B), q, [math.degrees(x) for x in B.joint_quantum()])
    out = {"quantum_mm": quantum_mm(B, q, u), "rigid_mm": rigid_mm(B, q, u, kw.get("force_n")),
           "small_mm": small_mm(B, q, u, kw.get("force_n")),
           "f_touch_n": f_touch(B, q, u, kw.get("v", 0.0), kw.get("a", 0.0), kw.get("readings", 1), kw.get("local")),
           "noise_n": noise_n(B, q, u, kw.get("v", 0.0), kw.get("a", 0.0)), "view_tol": view_tol(B, q),
           "camera_step": {"mm": cm, "deg": cd}}
    out["source"] = {name: _src(B, name) for name in ("quantum", "sigma_f")}
    return out

# ---------------------------------------------------------------- margins (SPEC-R4 A4.3, A4.8)
def standoff_mm(B, q=None, u=None, thing=None, v_mm_s=None, **kw):
    """d_stop(v) + QUANT x sigma_p + w_tool before a point; v None = the speed the tool arrives with (v_in)."""
    v = B.v_in(thing, q, u)[0] if v_mm_s is None else float(v_mm_s or 0) / MM
    return B.standoff(v, thing, q, u) * MM

def touch_reach_mm(B, q=None, u=None, thing=None, sigma_mm=None, force_n=None, **kw):
    """Reach past a touch target: BAND x sigma_p + rigid (one function for the check and the touch)."""
    return _laws().reach_past(_sigma_mm(thing, sigma_mm), rigid_mm(B, q, u, force_n))

def hand_lands_mm(B, q=None, u=None, thing=None, body=None, sigma_mm=None, **kw):
    return B.clearance(0.0, thing_of(B, thing, sigma_mm), body) * MM
def blocked_mm(B, q=None, u=None, sigma_mm=0.0, v_mm_s=0.0, **kw):
    return ratio("BAND") * float(sigma_mm or 0) + B.d_stop(float(v_mm_s or 0) / MM, q, u) * MM

def same_spot_mm(B, q=None, u=None, sigma_a_mm=0.0, sigma_b_mm=0.0, **kw):
    """The two contacts' combined position error plus the tool's half width."""
    return _laws().same_spot(float(sigma_a_mm or 0), float(sigma_b_mm or 0), _w_tool_mm(B))

def cue_near_mm(B, q=None, u=None, sigma_mm=0.0, size_mm=None, **kw):
    """The cue point's own error, half its size on the sheet and the tool's half width."""
    return float(sigma_mm or 0) + half(size_mm or 0) + _w_tool_mm(B)

def contact_keep_mm(B, q=None, u=None, sigma_mm=0.0, **kw):
    return float(sigma_mm or 0) + _w_tool_mm(B)
def device_box_near_mm(B, q=None, u=None, sigma_mm=0.0, **kw):
    return ratio("BAND") * float(sigma_mm or 0) + _w_tool_mm(B)
def line_centre_min_mm(B, q=None, u=None, sigma_mm=0.0, **kw):
    return ratio("BAND") * float(sigma_mm or 0)
def line_top_mm(B, q=None, u=None, sigma_mm=0.0, **kw):
    return ratio("BAND") * float(sigma_mm or 0)
def capsule_grow_mm(B, q=None, u=None, body=None, **kw):
    return B.clearance(0.0, None, body) * MM
def capsule_near_mm(B, q=None, u=None, thing=None, body=None, **kw):
    return B.clearance(0.0, thing, body) * MM

def link_clear_mm(B, q=None, u=None, v_mm_s=0.0, thing=None, body=None, **kw):
    """A link's clearance for a camera move: the path margin at that speed plus its stopping distance."""
    v = float(v_mm_s or 0) / MM
    return (B.clearance(v, thing, body) + B.d_stop(v, q, u)) * MM

def floor_keep_mm(B, q=None, u=None, **kw):
    """BAND x sigma_z of the support plane (0 where no plane or no error is known)."""
    support = B.space.support()
    return ratio("BAND") * float((support[1] if support else 0) or 0) * MM

def step_look_back_mm(B, q=None, u=None, **kw):
    return _w_tool_mm(B)
def stay_reach_mm(B, q=None, u=None, v_mm_s=0.0, **kw):
    return small_mm(B, q, u) + B.d_stop(float(v_mm_s or 0) / MM, q, u) * MM
def seat_gap_mm(B, q=None, u=None, v_mm_s=0.0, **kw):
    return _err_mm(B, q, u, v_mm_s)

# ---------------------------------------------------------------- the noise hold, times
def noise_samples(B=None, q=None, u=None, **kw):
    return _laws().samples_for()
def noise_hold_s(B, q=None, u=None, **kw):
    return noise_samples(B) * B.dt()
def noise_clear_mm(B, q=None, u=None, sigma_mm=0.0, **kw):
    return ratio("BAND") * float(sigma_mm or 0) + _w_tool_mm(B)
def act_max_s(B, q=None, u=None, planned_s=0.0, **kw):
    return B.limit_s(planned_s or 0.0)
def touch_piece_s(B, q=None, u=None, travel_mm=0.0, speed_mm_s=None, **kw):
    return B.limit_s(float(travel_mm or 0) / float(speed_mm_s)) if speed_mm_s else None
def hold_max_s(B, q=None, u=None, long_press_s=None, **kw):
    return None if long_press_s is None else float(long_press_s) - B.t_det()
def settle_force_n(B, q=None, u=None, **kw):
    return B.f_thr(0.0, 0.0)
def settle_s(B, q=None, u=None, **kw):
    return B.t_settle()
def settle_step_s(B, q=None, u=None, **kw):
    return B.t_det()
def settle_max_s(B, q=None, u=None, **kw):
    return B.limit_s(B.t_settle())

# ---------------------------------------------------------------- 4.3 act: confirm, fit, travel, release, lift
def confirm_back_mm(B, q=None, u=None, travel_mm=0.0, **kw):
    """Back off by the travel the part showed and the tool's half width (SPEC-R4 A4.8)."""
    return float(travel_mm or 0) + _w_tool_mm(B)

def confirm_tol_mm(B, q=None, u=None, v_mm_s=0.0, force_n=None, **kw):
    """The two contacts' own errors (v x t_det + quantum each) and the arm's give."""
    err = _err_mm(B, q, u, v_mm_s)
    return err + err + rigid_mm(B, q, u, force_n)

def confirm_max_mm(B, q=None, u=None, v_mm_s=0.0, travel_mm=0.0, force_n=None, **kw):
    return confirm_back_mm(B, q, u, travel_mm) + confirm_tol_mm(B, q, u, v_mm_s, force_n)

def need_confirm(B, q=None, u=None, contact=None, **kw):
    """False only when the stored contact shows a stiffness of at least the arm's own (the force rose as fast as
    k_servo predicts: a rigid surface); an unknown stiffness is confirmed by a second touch."""
    k = _get(contact, "k_c")
    return k is None or k < B.k_servo(q, u)

def fit_offset_mm(B, q=None, u=None, thing=None, sigma_mm=None, **kw):
    return ratio("QUANT") * _sigma_mm(thing, sigma_mm) + _w_tool_mm(B)

def fit_skip_deg(B, q=None, u=None, force_n=0.0, v=0.0, **kw):
    """The felt normal's own angle error: atan(max(F_thr on one reading / the contact force, REL))."""
    thr = B.f_thr(v, 0, 1)
    share = thr / force_n if force_n and force_n > 0 else (math.inf if thr > 0 else 0.0)
    return math.degrees(math.atan(max(share, ratio("REL"))))

def fit_done_deg(B, q=None, u=None, v_mm_s=0.0, offset_mm=0.0, **kw):
    """The fitted plane's own angle error: atan(contact error / offset)."""
    return math.degrees(math.atan2(_err_mm(B, q, u, v_mm_s), float(offset_mm or 0)))

def fit_resid_mm(B, q=None, u=None, v_mm_s=0.0, force_n=None, **kw):
    return _err_mm(B, q, u, v_mm_s) + rigid_mm(B, q, u, force_n)

def reached_end_mm(B, q=None, u=None, force_n=None, **kw):
    """QUANT x rigid: the travel a push must show before the robot says the part moved to an end."""
    return ratio("QUANT") * rigid_mm(B, q, u, force_n)

moving_mm = end_along_mm = sliding_total_mm = still_moving_mm = reached_end_mm

def stroke_mm(B, q=None, u=None, sheet_travel_mm=None, shown_mm=None, **kw):
    """The sheet's travel plus rigid, else the travel a like part showed, else None (until the give or the hard
    stop: the caller bounds it by the reach along the line)."""
    return float(sheet_travel_mm) + rigid_mm(B, q, u) if sheet_travel_mm is not None else shown_mm

def retract_mm(B, q=None, u=None, travel_mm=0.0, size_mm=None, **kw):
    """Off the part: the travel it showed, the tool's half width, and its size when it moves by itself."""
    return float(travel_mm or 0) + _w_tool_mm(B) + float(size_mm or 0)

def retract_up_mm(B, q=None, u=None, thing=None, **kw):
    """Back up to the standoff the tool came in by."""
    return standoff_mm(B, q, (0, 0, 1), thing)

def lift_drop_mm(B, q=None, u=None, sigma_mm=0.0, **kw):
    """Under the point by the finger's thickness (two half widths) and the point's own band."""
    width = _w_tool_mm(B)
    return width + width + ratio("BAND") * float(sigma_mm or 0)

def lift_lower_mm(B, q=None, u=None, **kw):
    width = _w_tool_mm(B)
    return width + width

def lift_under_mm(B, q=None, u=None, sigma_mm=0.0, **kw):
    return ratio("BAND") * float(sigma_mm or 0) + rigid_mm(B, q, u)
def lift_profile_mm(B, q=None, u=None, thing=None, sigma_mm=None, size_mm=None, **kw):
    return standoff_mm(B, q, u, thing) + touch_reach_mm(B, q, u, thing, sigma_mm) + float(size_mm or 0)

# ---------------------------------------------------------------- 4.4, 4.5 the camera
def camera_near_mm(model):
    """The camera's near focus: the spec's when stated, else the model file's own near clip."""
    near = (model.spec.get("camera") or {}).get("near_m")
    return float(near if near is not None else model.mj.vis.map.znear * model.mj.stat.extent) * MM

def bracket_near_mm(B, q=None, u=None, model=None, **kw):
    return camera_near_mm(model if model is not None else _model(B))

def bracket_reach_extra_mm(B, q=None, u=None, **kw):
    return 0.0      # the bracket ends at the support or at the reach the model states

def view_distance_mm(B, q=None, u=None, f_px=None, size_mm=None, need_px=None, **kw):
    """The distance at which a part of that size spans the pixels asked; None with no size."""
    return None if not size_mm or not f_px or not need_px else float(size_mm) * float(f_px) / float(need_px)

def witness_plan_mm(B, q=None, u=None, f_px=None, size_mm=None, **kw):
    return view_distance_mm(B, q, u, f_px, size_mm, WITNESS_MIN_PX)

def span_min_deg(B, q=None, u=None, px_err=None, range_mm=None, f_px=None, need_mm=None, patch_px=None, **kw):
    """The least angle two rays must span for their depth error to fit what the next approach needs of the point:
    sin(span) at least QUANT x px_err x range / (f_px x need_mm). With only the pointed patch given (patch_px) the
    need is half its footprint and range and focal length cancel: QUANT x QUANT x px_err / patch_px."""
    err = MATCH_ERR_PX if px_err is None else float(px_err)
    if range_mm and f_px and need_mm:
        share = ratio("QUANT") * err * float(range_mm) / (float(f_px) * float(need_mm))
    else:
        share = ratio("QUANT") * ratio("QUANT") * err / float(min(PATCH_SIZES_PX) if patch_px is None else patch_px)
    return math.degrees(math.asin(min(1.0, share)))

# ---------------------------------------------------------------- formulas over robot://body (not limits)
def res_rad(res_deg, n):
    """Per-joint resolution in rad from robot://body joints[].resolution_deg (None or a missing entry counts 0)."""
    vals = list(res_deg or [])
    out = np.zeros(int(n))
    for j in range(int(n)):
        v = vals[j] if j < len(vals) else None
        out[j] = 0.0 if v is None else math.radians(abs(float(v)))
    return out

def _turn_deg(Ra, Rb):
    c = (float(np.trace(np.asarray(Ra) @ np.asarray(Rb).T)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))

def camera_step(model, q, res_deg):
    """(cam_mm, cam_deg): the root-sum-square camera motion for one resolution step on every joint at q."""
    q = np.asarray(model.q_home if q is None else q, dtype=float)
    res = res_rad(res_deg, len(q))
    p0, R0 = model.fk(q)["camera"]
    mm2 = deg2 = 0.0
    for j, step in enumerate(res):
        if step <= 0.0:
            continue
        q1 = q.copy()
        q1[j] += step
        p1, R1 = model.fk(q1)["camera"]
        mm2 += (float(np.linalg.norm(np.asarray(p1) - np.asarray(p0))) * MM) ** 2
        deg2 += _turn_deg(R1, R0) ** 2
    return math.sqrt(mm2), math.sqrt(deg2)

def _from_body(body):
    joints = body.get("joints") or []
    res = [j.get("resolution_deg") if isinstance(j, dict) else None for j in joints]
    pu = (body.get("camera") or {}).get("pose_uncertainty") or {}
    return {"res_deg": res, "pu_mm": float(pu.get("mm") or 0.0), "pu_deg": float(pu.get("deg") or 0.0),
            "source": "robot://body joints[].resolution_deg and camera.pose_uncertainty"}

def inputs(ex=None, body=None, B=None):
    """The resolution facts as served: {res_deg (per joint, None where none is stated), pu_mm, pu_deg, source}.
    Read from, in order: body (the robot://body document); ex.context["body"] (the document or a callable);
    ex.model.spec["encoder_counts"] with ex.context["camera_uncertainty"]. With B, quantum_source says whether the
    quantum is stated or measured by the robot on itself (a robot that states none measures it: CK-08)."""
    if isinstance(body, dict):
        facts = _from_body(body)
    else:
        ctx = getattr(ex, "context", None) or {}
        doc = ctx.get("body")
        if callable(doc):
            try:
                doc = doc()
            except Exception:      # a body that cannot be read now: the spec below
                doc = None
        if isinstance(doc, dict) and doc.get("joints"):
            facts = _from_body(doc)
        else:
            model = getattr(ex, "model", None)
            n = int(getattr(model, "n", 0) or 0)
            counts = ((getattr(model, "spec", None) or {}).get("encoder_counts")) if model is not None else None
            res = [None] * n
            if counts:
                per = counts if isinstance(counts, (list, tuple)) else [counts] * n
                turn = math.degrees(math.tau)
                res = [None if not c else turn / float(c) for c in list(per)[:n]] + [None] * max(0, n - len(per))
            pu = ctx.get("camera_uncertainty") or {}
            facts = {"res_deg": res, "pu_mm": float(pu.get("mm") or 0.0), "pu_deg": float(pu.get("deg") or 0.0),
                     "source": ("the robot spec's encoder_counts" if counts else "none stated")
                     + ("; the stated camera uncertainty" if pu else "")}
    if B is not None:
        facts["quantum_source"] = ("stated" if any(v is not None for v in facts["res_deg"]) else
                                   "measured by the robot on itself (the smallest change its readings showed)")
    return facts

# Still read outside this work package (each goes when its last reader has switched):
NOISE_DEFAULT_N = 0.1  # rcpx/r2/assist_guard.py, rcpx/r2/assist_step_read.py (round five)
ACT_MAX_S = 60.0  # rcpx/r2/assist_step.py (round five)
LADDER_ROUND_N = 0.1  # rcpx/r2/assist_guard.py (round five)
STANDOFF_MM, STANDOFF_UNC_MULT = 30.0, 2.0  # rcpx/r2/assist_step.py (round five)
TOUCH_EXTRA_MM, TOUCH_UNC_MULT = 10.0, 3.0  # rcpx/r2/assist_step.py (round five)
TOUCH_PIECE_S = 40.0  # rcpx/r2/eyes_choose.py (round five)
VIEW_D_MIN_MM, VIEW_D_MAX_MM, VIEW_D_STEP_MM = 150.0, 400.0, 50.0  # rcpx/r2/assist_seek_move.py (round five)
VIEW_TOOL_CLEAR_MM = 40.0  # rcpx/r2/assist_seek_move.py (round five)
DEVICE_BOX_NEAR_MM = 20.0  # rcpx/r2/assist_seek.py (round five)
BRACKET_NEAR_MM = 60.0  # rcpx/r2/eyes_choose.py (round five), tests/test_r2_sight_path.py
MISS_PU_FRAC = 0.5  # rcpx/r2/sight_locate.py
CAPSULE_GROW_MM, CAPSULE_NEAR_MM, CONTACT_KEEP_MM = 5.0, 15.0, 10.0  # rcpx/r2/selfsee_body.py (its pure callers)
FRAME_SETTLE_DT_S, FRAME_SETTLE_FRAC = 0.2, 0.02  # rcpx/r2/witness_pose.py
AZ_OFFSETS_DEG, EL_OFFSETS_DEG = (12.0, 20.0, 25.0), (10.0, 15.0)  # rcpx/r2/sight_plan.py
VIEW_FROM_OP_MAX_DEG = 25.0  # rcpx/r2/sight_plan.py
# The old call forms of four functions, for the same readers (tests/test_r3_real_like.py: quantum_mm(model, q, u,
# res_deg) and rigid_mm(quantum); rcpx/r2/assist_guard.py: f0(noise_n, sheet_forces); tests/test_r2_witness.py and
# the transition branches of sight_bounds.py and witness_bounds.py: view_tol(cam_mm, cam_deg)). Each answers
# exactly as before when its first argument is not a bounds object; their numbers live only here.
_OLD = {"rigid_min_mm": 0.5, "quanta": 2.0, "f0_noise": 3.0, "f0_no_sheet_n": 1.5, "f0_min_n": 1.0,
        "f0_sheet_frac": 0.5, "view_mm": 3.0, "view_deg": 1.5, "view_roll_deg": 2.0, "view_step": 2.0}

def _old_quantum_mm(model, q, u, res_deg):
    J = np.asarray(model.jac(np.asarray(q, dtype=float)), dtype=float)[:len(np.asarray(u).reshape(-1))]
    uu = np.asarray(u, dtype=float).reshape(-1)
    nrm = float(np.linalg.norm(uu))
    if not nrm > 0:
        return 0.0
    along = (J.T @ (uu / nrm)) * res_rad(res_deg, J.shape[1])
    return float(np.linalg.norm(along)) * MM

def _old_rigid_mm(quantum):
    return max(_OLD["rigid_min_mm"], _OLD["quanta"] * float(quantum))

def _old_f0(noise_n=NOISE_DEFAULT_N, sheet_forces=None):
    forces = [float(f) for f in (sheet_forces or []) if isinstance(f, (int, float)) and not isinstance(f, bool)
              and f > 0]
    base = _OLD["f0_no_sheet_n"] if not forces else max(_OLD["f0_min_n"], _OLD["f0_sheet_frac"] * min(forces))
    return max(_OLD["f0_noise"] * float(noise_n), base)

def _old_view_tol(cam_mm=0.0, cam_deg=0.0):
    k = _OLD["view_step"]
    return {"mm": round(max(_OLD["view_mm"], k * float(cam_mm)), 2),
            "deg": round(max(_OLD["view_deg"], k * float(cam_deg)), 2),
            "roll_deg": round(max(_OLD["view_roll_deg"], k * float(cam_deg)), 2)}

def _with_old_form(new, old):
    @functools.wraps(new)
    def call(*args, **kwargs):
        first = args[0] if args else kwargs.get("B")
        return new(*args, **kwargs) if hasattr(first, "f_thr") else old(*args, **kwargs)
    return call

quantum_mm = _with_old_form(quantum_mm, _old_quantum_mm)
rigid_mm = _with_old_form(rigid_mm, _old_rigid_mm)
f0 = _with_old_form(f0, _old_f0)
view_tol = _with_old_form(view_tol, _old_view_tol)

# The limits whose own arguments do not begin with a pose answer both call forms (see _either).
_OWN = {"noise_n": ("v", "a"), "f_touch": ("v", "a", "readings", "local"), "ladder_round_n": (),
        "miss_mm": ("depth_mm", "f_px", "pu_mm", "pu_deg"), "hand_lands_mm": ("thing", "body", "sigma_mm"),
        "same_spot_mm": ("sigma_a_mm", "sigma_b_mm"), "cue_near_mm": ("sigma_mm", "size_mm"),
        "contact_keep_mm": ("sigma_mm",), "device_box_near_mm": ("sigma_mm",), "line_centre_min_mm": ("sigma_mm",),
        "line_top_mm": ("sigma_mm",), "capsule_grow_mm": ("body",), "capsule_near_mm": ("thing", "body"),
        "link_clear_mm": ("v_mm_s", "thing", "body"), "floor_keep_mm": (), "step_look_back_mm": (),
        "noise_samples": (), "noise_hold_s": (), "noise_clear_mm": ("sigma_mm",), "act_max_s": ("planned_s",),
        "touch_piece_s": ("travel_mm", "speed_mm_s"), "hold_max_s": ("long_press_s",), "settle_force_n": (),
        "settle_s": (), "settle_step_s": (), "settle_max_s": (), "fit_skip_deg": ("force_n", "v"),
        "lift_drop_mm": ("sigma_mm",), "lift_lower_mm": (), "bracket_near_mm": ("model",),
        "bracket_reach_extra_mm": ("model",), "view_distance_mm": ("f_px", "size_mm", "need_px"),
        "witness_plan_mm": ("f_px", "size_mm"), "span_min_deg": ("px_err", "range_mm", "f_px", "need_mm")}
for _name, _names in _OWN.items():
    globals()[_name] = _either(*_names)(globals()[_name])
tool_clear_mm = view_link_clear_mm = view_tool_clear_mm = link_clear_mm
under_table_mm = floor_keep_mm
ray_only_ahead_mm = bracket_near_mm
__all__ = sorted(n for n, v in list(globals().items()) if callable(v) and not n.startswith("_")
                 and getattr(v, "__module__", None) == __name__)
