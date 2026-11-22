# SPDX-License-Identifier: AGPL-3.0-only
"""Motion of the selfcheck package: camera wiggles and joint stop searches.

Bounds plans straight joint legs and the path check checks them before they
stream through Executor._period, so tick hooks run. The cap uses applied
torque and failure to gain commanded speed, because a body being checked can
give a wrong external torque estimate. A wiggle compares the tip force rise
with the leg's noise threshold and planned inertia. It holds the last target
at a stop: error gone at rest slows this and later legs, error proportional
to joint angle supplies a bias line, and another rise ends as contact. Each
resume must make measurable progress; a sufficient turn supplies a look. A
classified force stop with no progress and no unexplained joint lead makes
the force estimate untrustworthy. This wiggle then resumes at full planned
speed under the joint lead rule, retaining the cap rules.

The camera's fit and encoder steps derive each wiggle amplitude. A look waits
for motion left to fit its pixel resolution, and records the joint reading.
Stop search retimes the Bounds path into a slow zone derived from known zero
error and stopping travel, or the whole range when no zero has been measured.
Its joint lead, speed and applied torque rise over learned free motion find
the stall; the model's gravity change is allowed for, while external torque
and tip force estimates are never used. The allowance derives from hand
weight, or the joint rating on the operator's firm word. Easing measures the
stop's give, and the arm retreats along the path searched. No ground truth
is read here.
"""
import math
import numpy as np
from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx import bounds_laws as laws
from rcpx import bounds_report as report
from rcpx.context import refusal as context_refusal
from rcpx.r2 import selfcheck_limits as limits
from rcpx.r2.selfcheck_guard import LegGuard, classify
DEG = math.pi / 180.0
MM = 1000.0


def refusal(check, why, hint):
    return {"said": str(why).rstrip(".") + ".", **context_refusal(check, why, hint)}


def plan_leg(ex, q_from, q_to, kind, intent):
    plan = bounds.of(ex).plan(q_from, [np.asarray(q_to, float)], kind,
                              intent=leg_intent(ex, q_from, q_to, intent))
    if not plan.refused:
        plan.refused = ex._path_check([q_from] + plan.path, False)
    plan._sc_q0 = np.asarray(q_from, float).copy()
    ex._sc_plans.append(plan)
    return plan


def leg_intent(ex, q_from, q_to, intent, joint_reserve=False, lead=False):
    """The first selfcheck pass reserves half its available modelled braking."""
    B = bounds.of(ex)
    dp = ex.model.fk_tip(q_to)[0] - ex.model.fk_tip(q_from)[0]
    length = float(np.linalg.norm(dp))
    u = dp / length if length > 0 else np.array([0.0, 0.0, 1.0])
    accel = R.GIVE * B.a_brake(q_from, u) * ex.cfg['torque_frac']
    # A long joint arc can have a short endpoint chord. Carry the joint's
    # available acceleration into that same chord coordinate as Bounds.plan.
    if joint_reserve and length > 0:
        dq = np.asarray(q_to) - q_from
        a_joint = laws.qdd_model(B.tau_max() * ex.cfg['torque_frac'],
                                 ex.model.gravity_torque(q_from), np.diag(B.robot.mass_matrix(q_from)))
        accel = min(accel, R.GIVE * min((a * length / abs(d) for a, d in zip(a_joint, dq) if d != 0), default=math.inf))
    if lead and length > 0:
        # Free lag left after a ramp ends must fit the lead guard's step.
        # The two servo stages contribute lag squared and inertia over kp
        # to that residual per unit of commanded joint acceleration.
        dq = np.asarray(q_to) - q_from
        stages = B.lag(1.0) ** 2 + np.diag(B.robot.mass_matrix(q_from)) / ex.kp
        a_joint = np.divide(limits.contact_step(ex), stages,
                            out=np.full(ex.model.n, math.inf), where=stages > 0)
        accel = min(accel, R.GIVE * min((a * length / abs(d) for a, d in zip(a_joint, dq) if d != 0), default=math.inf))
    return {**(intent or {}), 'accel': min((intent or {}).get('accel', math.inf), accel)}


def turn_lines(ex, joints, amps):
    """The straight joint turns a wiggle of `joints` would make from the commanded pose, before any plan exists:
    one object with a .path per joint and side (out to the full amplitude inside the joint's range), sampled at
    every REL of the turn (for free_space, before anything is planned)."""
    import types
    q_c = ex._q_des.copy()
    n = max(1, int(math.ceil(1 / R.REL - laws.ROUND)))
    lines = []
    for j in joints:
        for sign in (1, -1):
            end = float(np.clip(q_c[j] + sign * amps[j], ex.model.q_lo[j], ex.model.q_hi[j]))
            if end == q_c[j]:
                continue
            path = []
            for k in range(n + 1):
                q = q_c.copy()
                q[j] = q_c[j] + (end - q_c[j]) * (k / n)
                path.append(q)
            lines.append(types.SimpleNamespace(path=path))
    return lines


def free_space(ex, legs, clear_m, touching=False):
    """None, or (why, hint) when the wiggle's legs (joint paths) would bring the robot within clear_m of a point
    robot://world holds (a stored contact, a located point, a device entry with a point): a wiggle runs in free
    space only, and a part beside the tool is the thing it would drive into. touching: a point exactly at
    clear_m counts too (the check made before a plan exists, where clear_m may be zero)."""
    known = ex._known_points()
    if not len(known):
        return None
    for plan in legs:
        last_tip = None
        for k, q in enumerate(plan.path):
            tip = ex.model.fk_tip(q)[0]
            if last_tip is not None and float(np.linalg.norm(tip - last_tip)) <= bounds.of(ex).spacing(clear_m) and k != len(plan.path) - 1:
                continue
            last_tip = tip
            c, i, name = ex._nearest_known(np.asarray(q, dtype=float), known)
            if c < clear_m or (touching and c <= clear_m):
                p = ", ".join(f"{x * MM:.0f}" for x in known[i])
                return (f"the wiggle would bring the {name} within {max(c, 0.0) * MM:.0f} mm of a stored point at "
                        f"[{p}] mm: self_check turns each joint in free space only, and a part beside the tool is "
                        "what it would drive into",
                        f"move the arm so nothing it has touched or located is within {clear_m * MM:.0f} mm of "
                        "it plus the wiggle (retreat, then home or a pose above the surface), then run self_check")
    return None


def picture_need(ex, q):
    height = abs(ex.model.fk(q)["camera"][0][2] - ex.cfg["floor_z_m"])
    return R.QUANT * height / ex.model.f_px


def force_need(ex, q):
    """Quiet enough to compare held force, within its noise-scale servo give."""
    B = bounds.of(ex)
    return min(picture_need(ex, q), min(B.rigid(q, u, B.f_thr(0, 0)) for u in np.eye(3)))


def hold_quiet(ex, q, need_m, guard):
    spec = bounds.of(ex).wait("settle", need_m)
    s = ex._sense()
    for _ in range(max(1, int(math.ceil(spec.limit_s / ex.dt)))):
        s = ex._period(q)
        stop = guard.check(s)
        if stop:
            return stop, s
        left = float(np.linalg.norm(ex.model.jac(s.q)[:3] @ s.qd)) * spec.t_det
        # A camera can still turn while the tip position is stationary.
        height = abs(ex.model.fk(s.q)["camera"][0][2] - ex.cfg["floor_z_m"])
        turning = float(np.linalg.norm(ex.model.jac(s.q)[3:] @ s.qd)) * height * spec.t_det
        if left <= need_m and turning <= need_m:
            break
    return None, s


def hold_force_quiet(ex, q, guard):
    """Repeat derived settle windows while measured residual motion improves.

    A single servo time constant need not reach the force comparison's
    resolution, especially while a wrong body is finding its held pose.
    Each window keeps its cap and tick guard; lack of progress ends the wait.
    """
    need = force_need(ex, ex._sense().q)
    previous = None
    while True:
        stop, s = hold_quiet(ex, q, need, guard)
        detection = bounds.of(ex).t_det()
        jac = ex.model.jac(s.q)
        height = abs(ex.model.fk(s.q)['camera'][0][2] - ex.cfg['floor_z_m'])
        left = max(float(np.linalg.norm(jac[:3] @ s.qd)),
                   float(np.linalg.norm(jac[3:] @ s.qd)) * height) * detection
        if stop or left <= need:
            return stop, s
        if previous is not None and previous - left <= R.REL * previous:
            return None, s
        previous = left


def stream(ex, plan, guard, capture=None):
    s = ex._sense()
    for k, q in enumerate(plan.path):
        plan._sc_ran = True
        before = s
        s = ex._period(q)
        if capture is not None:
            capture(k, before, s)
        stop = guard.check(s)
        if stop:
            return stop, s
    return "goal", s


def go(ex, q_to, kind="transit", intent=None, j=None, contact=True, force_cap=True, picture=False):
    plan = plan_leg(ex, ex._q_des.copy(), np.asarray(q_to, float), kind, intent)
    if plan.refused:
        return "refused", plan.refused
    guard = LegGuard(ex, plan, j, contact=contact, force_cap=force_cap and kind != 'retreat')
    stop, s = stream(ex, plan, guard)
    if stop == "goal" and picture:
        ended, s = hold_quiet(ex, q_to, picture_need(ex, s.q), guard)
        stop = ended or stop
    return stop, s


def sides(model, q_c, j, amp, margin):
    out = []
    if amp <= 0:
        return out
    for sign in (1, -1):
        end = model.q_hi[j] - margin if sign > 0 else model.q_lo[j] + margin
        room = sign * (end - q_c[j])
        if room >= R.GIVE * amp:
            out.append((sign, q_c[j] + sign * min(amp, room)))
    return out


def planned_legs(ex, joints, amps, intent=None):
    q_c = ex._q_des.copy()
    legs = []
    for j in joints:
        if amps[j] <= 0:
            continue
        for sign in (1, -1):
            q_t = q_c.copy()
            q_t[j] = np.clip(q_c[j] + sign * amps[j], ex.model.q_lo[j], ex.model.q_hi[j])
            preview = plan_leg(ex, q_c, q_t, "transit", intent)
            if preview.refused:
                legs.append(preview)
                return legs
            points = [q_c] + preview.path
            peak = max((abs(y[j] - x[j]) / ex.dt for x, y in zip(points, points[1:])), default=0.0)
            margin = limits.stop_angle(ex, q_c, j, peak)
            todo = dict(sides(ex.model, q_c, j, amps[j], margin))
            if sign not in todo:
                continue
            q_t[j] = todo[sign]
            plan = plan_leg(ex, q_c, q_t, "transit", intent)
            plan._sc_j, plan._sc_side, plan._sc_target = j, sign, q_t
            legs.append(plan)
    return legs


def wiggle(ex, joints, amps, intent=None, legs=None, capture=None, *, guard_mode='force'):
    """Run one wiggle; guard_mode is a test hook, not a tool argument."""
    ex._sc_intent, ex._sc_bias_lines = intent, {}
    ex._sc_guard_mode = guard_mode
    ex._sc_wiggle_state = {"speed_factor": 1.0}
    legs = planned_legs(ex, joints, amps, intent) if legs is None else legs
    for plan in legs:
        if plan.refused:
            return plan.refused
    q_c = ex._q_des.copy()
    ref_plan = bounds.of(ex).plan(q_c, [], "transit", intent=intent)
    if ref_plan.refused:
        return ref_plan.refused
    ref_guard = LegGuard(ex, ref_plan, contact=False, force_cap=False)
    stop, s = hold_force_quiet(ex, q_c, ref_guard)
    out = {"q_c": q_c, "q_ref": s.q.copy(), "ref_img": np.asarray(ex.backend.render("wrist")),
           "looks": [], "planned_looks": len(legs), "skipped": [], "stopped": None, "returned": True, "false_stops": [],
           "contact_lead_rad": float(np.max(limits.contact_step(ex)))}
    if stop:
        out["stopped"] = {"joint": joints[0], "stopped": stop,
                          "peak_n": ref_guard.peak, "threshold_n": ref_guard.threshold}
    for j in joints:
        if not any(p._sc_j == j for p in legs):
            out["skipped"].append({"joint": j, "side": 0,
                                   "why": "the camera cannot follow this joint and no encoder step is stated" if amps[j] == 0 else "no room to either side inside the joint limits"})
    for planned in legs:
        if out["stopped"]:
            break
        j, sign, q_t = planned._sc_j, planned._sc_side, planned._sc_target
        look_ready = False
        q_start = ex._sense().q.copy()
        previous_stop = float(q_start[j])
        remaining = abs(q_t[j] - previous_stop)
        points = [planned._sc_q0] + planned.path
        peak = max((abs(y[j] - x[j]) / ex.dt for x, y in zip(points, points[1:])), default=0.0)
        while True:
            factor = ex._sc_wiggle_state['speed_factor']
            slowed = leg_intent(ex, ex._q_des, q_t, intent, lead=ex._sc_guard_mode == 'lead')
            slowed['accel'] *= factor ** 2
            plan = plan_leg(ex, ex._q_des.copy(), q_t, "transit", slowed)
            if plan.refused:
                out["skipped"].append({"joint": j, "side": sign, "why": plan.refused["refused"]["why"]})
                stop, s = "refused", ex._sense()
                break
            plan.path = retime_joint(plan.path, plan._sc_q0, j, lambda x: factor * peak, ex.dt)
            plan.time_s = len(plan.path) * ex.dt
            guard = LegGuard(ex, plan, j)
            watch = None if capture is None else lambda k, before, after: capture(j, sign, plan, k, before, after, out)
            stop, s = stream(ex, plan, guard, watch)
            if stop == "goal":
                ended, s = hold_quiet(ex, q_t, picture_need(ex, s.q), guard)
                stop = ended or stop
            out['contact_lead_rad'] = max(out['contact_lead_rad'], guard.lead_step_peak)
            if stop != "contact":
                break
            if guard.mode == 'lead':
                break
            label = classify(ex, j, guard.q_start, guard.F_ref, guard)
            s = ex._sense()
            if label == 'contact':
                stop = getattr(guard, 'classified_stop', 'contact')
                break
            progress = abs(guard.q_hold[j] - previous_stop)
            if label == 'dynamic':
                ex._sc_wiggle_state['speed_factor'] *= R.GIVE
            if sign * (guard.q_hold[j] - q_start[j]) >= R.GIVE * abs(q_t[j] - q_start[j]):
                look_ready = True
                stop = 'goal'
                break
            if progress <= max(R.QUANT * bounds.of(ex).joint_quantum()[j], R.REL * remaining):
                if guard.extra_peak <= 0:
                    ex._sc_guard_mode = 'lead'
                    ex._sc_wiggle_state['speed_factor'] = 1.0
                    out['false_stops'].append({'joint': j, 'side': sign,
                                               'kind': 'distrusted', 'rise_n': guard.peak})
                    continue
                stop = 'contact'
                break
            previous_stop = float(guard.q_hold[j])
            remaining = abs(q_t[j] - previous_stop)
            record = {"joint": j, "side": sign, "kind": label}
            if label == 'dynamic':
                record['speed_factor'] = ex._sc_wiggle_state['speed_factor']
            else:
                record['slope_n_per_deg'] = (ex._sc_bias_lines[j] * DEG).tolist()
            out["false_stops"].append(record)
        if stop not in ("goal", "refused"):
            out["stopped"] = {"joint": j, "stopped": stop, "peak_n": guard.peak,
                              "threshold_n": guard.threshold}
            if stop == 'contact' and guard.mode == 'lead':
                k = int(np.argmax(guard.lead_extra - guard.lead_step))
                out['stopped'].update({'lead_joint': k, 'extra_lead_deg': math.degrees(guard.lead_extra[k]),
                                       'step_deg': math.degrees(guard.lead_step[k])})
            ex._ease_impl()
            break
        if stop == "goal":
            if not look_ready and sign * (s.q[j] - out["q_ref"][j]) < R.GIVE * abs(q_t[j] - q_c[j]):
                out["skipped"].append({"joint": j, "side": sign, "why": "the joint did not follow its target"})
            else:
                out["looks"].append({"joint": j, "side": sign, "q": s.q.copy(),
                                     "img": np.asarray(ex.backend.render("wrist"))})
        factor = ex._sc_wiggle_state['speed_factor']
        slowed = leg_intent(ex, ex._q_des, q_c, intent, lead=ex._sc_guard_mode == 'lead')
        slowed['accel'] *= factor ** 2
        back_plan = plan_leg(ex, ex._q_des.copy(), q_c, "retreat", slowed)
        if back_plan.refused:
            back = "refused"
        else:
            back_plan.path = retime_joint(back_plan.path, back_plan._sc_q0, j, lambda x: factor * peak, ex.dt)
            back_plan.time_s = len(back_plan.path) * ex.dt
            back_guard = LegGuard(ex, back_plan, j, contact=False, force_cap=False)
            back, _ = stream(ex, back_plan, back_guard)
            if back == 'goal':
                ended, _ = hold_force_quiet(ex, q_c, back_guard)
                back = ended or back
        if back != "goal":
            out["returned"] = False
            out["stopped"] = {"joint": j, "stopped": back}
            ex._ease_impl()
            break
    if out["stopped"]:
        out["returned"] = False
    else:
        final = bounds.of(ex).plan(q_c, [], "retreat", intent=intent)
        guard = LegGuard(ex, final, contact=False, force_cap=False)
        stop, _ = hold_quiet(ex, q_c, picture_need(ex, ex._sense().q), guard)
        out["returned"] = stop is None
    ex._sc_bias_lines = {}
    out['contact_guard'] = 'joint lead' if ex._sc_guard_mode == 'lead' else 'tip force'
    return out


def retime_joint(path, q0, j, w_limit, dt):
    out, previous = [], np.asarray(q0, float)
    for q in path:
        q = np.asarray(q, float)
        w = w_limit(q[j])
        turn = abs(q[j] - previous[j])
        if turn > 0 and w <= 0:
            raise ValueError("zero search speed cannot carry this step")
        n = max(1, int(math.ceil(turn / (w * dt) - laws.ROUND))) if turn > 0 else 1
        out.extend(previous + (q - previous) * (k / n) for k in range(1, n + 1))
        previous = q
    return out


def zero_plan(ex, j, direction, st, name, firm, intent):
    B, q0 = bounds.of(ex), ex._q_des.copy()
    end = ex.model.q_hi[j] if direction > 0 else ex.model.q_lo[j]
    q_end = q0.copy(); q_end[j] = end
    slow, slow_source = limits.zero_slow(ex, q_end, j, firm)
    if slow <= 0:
        tau, _ = limits.zero_allow(ex, q_end, j, firm)
        threshold = B.tau_thr(0.0)[j]
        why = (f"at this joint the hand's own weight makes {tau:g} N m, under the {threshold:g} N m "
               "the joint can tell from nothing: a gentle search could not feel the stop") if not firm else (
               f"at this joint the remaining rating is {tau:g} N m, under the {threshold:g} N m "
               "the joint can tell from nothing: the search could not feel the stop")
        return refusal("zero_blind", why,
                       f"zero_check(joint={name!r}, firm=true)")
    known, known_source = limits.zero_known(ex, st, name)
    past = short = limits.offset_range(ex, j) if known is None else known
    past_source = known_source if known is not None else "the whole joint range: no zero measured"
    if known is not None:
        # The offset bounds the reading of the stop. The servo target must
        # also have room to take up free lag and detect and brake at it.
        past += limits.stop_angle(ex, q_end, j, slow)
        past_source += " plus detection and braking travel at the slow search speed"
    zone, zone_source = limits.zero_zone(ex, st, q_end, j, name)
    target = q0.copy(); target[j] = end + direction * past
    plan = B.plan(q0, [target], "transit", intent=leg_intent(ex, q0, target, intent, joint_reserve=True))
    ex._sc_plans.append(plan)
    if plan.refused:
        return plan.refused
    a = B.qdd_max(q_end)[j]
    def w_limit(x):
        distance = max(0, direction * (end - x) - zone)
        return min(B.qd_max()[j], math.sqrt(slow ** 2 + laws.TWO * a * distance))
    original = [q0] + plan.path
    speeds = [abs(y[j] - x[j]) / ex.dt for x, y in zip(original, original[1:])]
    peak = max(speeds, default=0.0)
    ramp_end = next((k for k, (q, w) in enumerate(zip(plan.path, speeds))
                     if w >= min(peak, w_limit(q[j])) * (1 - laws.ROUND)), 0)
    plan._sc_ramp_periods = sum(max(1, int(math.ceil(w / w_limit(q[j]) - laws.ROUND)))
                                  for q, w in zip(plan.path[:ramp_end + 1], speeds))
    plan.path = retime_joint(plan.path, q0, j, w_limit, ex.dt)
    plan.time_s = len(plan.path) * ex.dt
    inside = [q for q in plan.path if ex.model.q_lo[j] <= q[j] <= ex.model.q_hi[j]]
    bad = ex._path_check([q0] + inside, False)
    if bad is not None:
        return bad
    tau, tau_source = limits.zero_allow(ex, q_end, j, firm)
    plan.items = [report.item("torque", tau, "intent" if firm else "prior", slow_source,
                              "state firm: true" if not firm else "the joint rating decides")] + [
        item for item in plan.items if item["what"] != "time"] + [
        report.item("time", B.limit_s(plan.time_s), B.src("dt")[0],
                    "the retimed search plus the relative margin", "change the search path")]
    search = {"fast_deg_s": math.degrees(B.qd_max()[j]), "slow_deg_s": math.degrees(slow),
              "slow_zone_deg": math.degrees(zone), "past_deg": math.degrees(past),
              "short_deg": math.degrees(short), "tau_allow_nm": tau,
              "sources": {"fast_deg_s": "the joint's measured or rated full speed",
                          "slow_deg_s": slow_source, "slow_zone_deg": zone_source,
                          "past_deg": past_source,
                          "short_deg": known_source if known is not None else "the whole joint range: no zero measured",
                          "tau_allow_nm": tau_source}}
    return plan, end, search


def zero_search(ex, j, direction, st, name, firm=False, intent=None):
    planned = zero_plan(ex, j, direction, st, name, firm, intent)
    if isinstance(planned, dict):
        return planned
    plan, end, search = planned
    q0 = ex._q_des.copy()
    B = bounds.of(ex)
    guard = LegGuard(ex, plan, j, contact=False, force_cap=False)
    start = s = ex._sense()
    points = [q0] + plan.path
    speeds = [abs(y[j] - x[j]) / ex.dt for x, y in zip(points, points[1:])]
    warm = plan._sc_ramp_periods + int(math.ceil(B.t_settle() / ex.dt))
    # A fast transit's ramp is not the free drive of the slow approach.
    # Relearn there only if its settling stretch fits before the earliest
    # possible stop established by the session's zero measurements.
    slow = search['slow_deg_s'] * DEG
    slow_room = (search['slow_zone_deg'] - search['short_deg']) * DEG
    relearn = slow_room > slow * B.t_settle()
    leads, loads, signs = [], [], 0
    g_ref = float(ex.model.gravity_torque(start.q)[j])
    rise = 0.0
    found, stopped, why = False, None, ""
    for k, (q, w) in enumerate(zip(plan.path, speeds)):
        plan._sc_ran = True
        s = ex._period(q)
        stopped = guard.check(s)
        lead = direction * (q[j] - s.q[j])
        speed = direction * s.qd[j]
        drive = abs(float(s.tau[j]))
        lead_free = w * B.lag(1.0) + R.QUANT * B.joint_quantum()[j]
        distance = direction * (end - q[j])
        if (relearn and w <= slow and distance <= search['slow_zone_deg'] * DEG and
                distance - search['short_deg'] * DEG > slow * B.t_settle()):
            leads, loads = [], []
            warm = k + int(math.ceil(B.t_settle() / ex.dt))
            relearn = False
        if k < warm:
            stall = speed < R.GIVE * w and lead > R.DOUBLE * lead_free
            if not stall:
                leads.append(lead - lead_free); loads.append(drive)
                g_ref = float(ex.model.gravity_torque(s.q)[j])
        else:
            hidden_lead, hidden_load = max(leads, default=0), max(loads, default=0)
            spread_lead = float(np.std(leads)) if leads else 0.0
            spread_load = float(np.std(loads)) if loads else 0.0
            rise = drive - hidden_load - abs(float(ex.model.gravity_torque(s.q)[j]) - g_ref)
            stall = speed < R.GIVE * w and (
                lead - lead_free > hidden_lead + R.TWO_READ * spread_lead or
                rise > R.TWO_READ * spread_load + B.tau_thr(w * limits.lever(ex, s.q, j))[j] + B.e_tau() * B.tau_max()[j])
        rise = drive - max(loads, default=0) - abs(float(ex.model.gravity_torque(s.q)[j]) - g_ref)
        # The intended joint stop ends at its allowance before its own cap reading.
        if rise >= search['tau_allow_nm'] and stopped in (None, 'cap'):
            found, stopped = True, None
            break
        if stopped:
            break
        signs = signs + 1 if stall else 0
        if signs >= plan.guard.readings:
            found = True
            break
    load_nm, q_stall = max(0, rise), float(s.q[j])
    s = ex._ease_impl()
    reading = float(s.q[j])
    give = abs(q_stall - reading)
    short = direction * (end - reading)
    if found and short > search["short_deg"] * DEG:
        found, why = False, f"the joint stalled {math.degrees(short):g} deg before its declared end: not taken as the stop"
    elif not found and not stopped:
        why = f"no stop met up to {search['past_deg']:g} deg past the declared end"
    elif stopped:
        found, why = False, f"the search ended on {stopped} before a stop was met ({getattr(guard, 'cap_reason', 'tick stop')})"
    back, _ = go(ex, q0, "retreat", intent, j, contact=False, force_cap=False, picture=True)
    if back != "goal":
        ex._ease_impl()
    return {"found": found, "reading": reading if found else None, "why": why, "stopped": stopped,
            "travel": abs(reading - start.q[j]), "load": round(load_nm / B.tau_max()[j], 3),
            "returned": back == "goal", "moved": bool(plan.path), "give": give, "load_nm": load_nm,
            "planned_s": len(plan.path) * ex.dt, "search": search}
