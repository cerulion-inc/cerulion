# SPDX-License-Identifier: AGPL-3.0-only
"""Package probe, the push: short force-limited steps in which the part, not the command, decides the path.

The round one push (control_contact._push) holds the tip on its commanded line: whatever the part does, the tip
path is the line that was asked for, so it says nothing about the part. Here the command is reset to the MEASURED
tip across the push line while the push goes on, and at every step along it:

    p_cmd = tip_measured + C d f,     f = (advance asked - advance made) / (d' C d),   0 <= f <= force_n

C is the tip's compliance under the joint servos (J K^-1 J'), so the servo's static force on the tip is f along d
and nothing across it: the tip has no stiffness sideways, and friction with the part carries it wherever the part
goes (an arc about a hinge, a slide's own line). The lead along d is capped at force_n, so the static push can
never pass the asked force; the servos' own damping is fed forward only for the motion the tip is seen to make.

How often the sideways reset runs depends on the arm (free_across): nearly every control period on an arm whose
joints turn freely, once per step on an arm whose joints have dry friction of their own (a tip with no sideways
hold wanders on such an arm). Before the part is met the tip is held on its line as in round one. Between steps
the along-line lead (the push force) is kept, so a sprung part does not spring back; the full ease to the measured
pose runs once, after the last step, in the caller.

After each step the tip stands still for a moment. A drift across the push that is steady over that hold is the
tip sliding on the part, not the part moving: its rate is measured and taken out of the path (slid_m reports it).
The push direction then turns toward the way the tip was carried (never more than the stated tolerance from the asked
direction), so the next step asks little of the friction.

Over the last hold_dir_steps steps the push direction is held fixed. A tip that slides on the part goes where it
is pushed, so a push direction that follows the path can bend the path by itself, and that bend reads as a hinge.
With the direction held, a part on a hinge goes on bending the path at the same rate and a sliding tip draws a
straight line: tail_turn (below) is the bend of that held stretch, for the report to compare with the fitted
circle. No robot or device is named here.
"""

import math

import numpy as np

from rcpx.control_base import Guard
from rcpx.r2 import probe_bounds as derived

FF_SHARE = 0.8            # share of the servos' own damping the command feeds forward while it moves
RESET_SHARE = 0.5         # the command is reset to the measured tip across the push line every (sideways give,
RESET_FRICTION = 2.0      # m per N) x max(RESET_SHARE x push force, RESET_FRICTION x the joints' own dry friction
                          # seen at the tip) of travel: nearly every period on an arm whose joints turn freely, once
                          # per step on one whose joint friction would let a tip with no sideways hold wander
STEP_MAX_S = 2.5
SLIDE_STEADY = 0.6        # control numeric: agreement of consecutive drift windows
TURN_BLEND = 0.6          # control numeric: share of the observed direction
TRACK_PASSES = 3         # solver numeric: passes on a known reachable calibration pose
TOUCH_FRAC = 0.15         # kept numeric: share of the intended push force, with the detection floor
HOLD_DIR_STEPS = 4        # count: final steps with fixed direction


def touch_force(B, force_n, speed=0):
    return max(B.f_thr(speed), TOUCH_FRAC * force_n)


def compliance(ex, q):
    """Tip compliance under the joint servos, m per N (3 x 3, frame B)."""
    J = ex.model.jac(q)[:3]
    return J @ np.diag(1.0 / ex.kp) @ J.T


def joint_friction_n(ex, q):
    """The tip force the joints' own dry friction can hold back (N): the loaded model's friction torques seen
    through the Jacobian. Zero for joints that turn freely."""
    loss = getattr(ex, "_frictionloss", None)
    if loss is None or not np.any(np.asarray(loss) > 0):
        return 0.0
    J = ex.model.jac(q)[:3]
    return float(np.linalg.norm(np.linalg.lstsq(J.T, np.asarray(loss, dtype=float), rcond=1e-3)[0]))


def solve_pose(ex, q, point, rotation, axis_only):
    previous = float("inf")
    rounding = math.sqrt(np.finfo(float).eps) * ex.model.length_scale
    while True:
        q_next, error, turn = ex._track(q, point, rotation, axis_only)
        if error <= rounding or error >= previous or np.array_equal(q_next, q):
            return q_next, error, turn
        previous, q = error, q_next


def reach_plan(ex, s, d, length):
    """Joint path of a straight stroke of `length` along d from the present pose (one target per measured step), and how far
    it got before the arm could not follow (m). Used for the path check and the reach refusal before any motion."""
    ax = ex.model.n < 6
    q, path, ok_len = s.q.copy(), [], 0.0
    B = derived.of(ex)
    step = derived.step_m(B, s.q, d, length)
    if length == 0:
        return [], 0
    n = max(1, int(math.ceil(length / step)))
    # FK makes this calibration target reachable by construction. Its residual is
    # numerical solver error, independent of the proposed straight stroke's reach.
    dq = np.linalg.lstsq(ex.model.jac(s.q)[:3], d * step, rcond=None)[0]
    reachable = ex.model.clip(s.q + dq)
    point, rotation = ex.model.fk_tip(reachable)
    calibration = s.q.copy()
    for _ in range(TRACK_PASSES):
        calibration, residual_m, residual_rad = ex._track(calibration, point, rotation, ax)
    solver_error = max(2.0 * residual_m, math.sqrt(np.finfo(float).eps) * ex.model.length_scale)
    solver_turn = max(2.0 * residual_rad, math.acos(1 - 2.0 * np.finfo(float).eps))
    for k in range(1, n + 1):
        p_des = s.tip + d * min(length, k * step)
        q, e, er = solve_pose(ex, q, p_des, s.R, ax)
        if e > max(B.rigid(q, d, 0), solver_error) or er > max(float(np.max(np.asarray(B.joint_quantum()))), solver_turn):
            break
        path.append(q.copy())
        ok_len = min(length, k * step)
    return path, ok_len


def seek_length(B, s, d, target, thing):
    """How far the seek may go before the part must have been met (m): to the target along the push and a little
    past it, never under the measurable seek nor over the target uncertainty."""
    ahead = float((np.asarray(target, dtype=float) - s.tip) @ d)
    return derived.seek_m(B, s.tip, d, target, thing, s.q)


def run(ex, s0, d0, max_m, force_n, seek_m=None, thing=None, intent=None, tolerance_deg=None):
    """Seek the part along d0 (for seek_m at most), then push it in measured steps. Returns a dict: points (the tip
    where the part was met and after each step, m, with the tip's own slide, as measured in the still holds, taken
    out), raw (the same, as measured), forces (N along the push at each sample), dirs (the push direction of each
    step), stop, peak_n, free_m, steps, across_m, slid_m, contact, done (joint targets streamed), d (the last push
    direction), tick (a tick hook's stop request, if any), cap_why, s (the last reading), free (the MEASURED joint
    poses from before anything resisted: the only stretch a retreat may retrace, since the streamed targets lead
    into the part), felt (control_base.Felt: what the whole push met, below the contact threshold too) and
    tail_turn (see tail_turn; None unless every step was made)."""
    B = derived.of(ex)
    dt = B.dt()
    step = derived.step_m(B, s0.q, d0, max_m)
    release_periods = max(1, math.ceil(B.t_settle() / dt))
    slide_periods = max(1, math.ceil(B.t_settle() / dt))
    spec = B.force("probe", thing, s0.q, d0, intent=intent)
    f_target = spec.first_n
    seek_speed = B.v_in(thing, s0.q, d0, intent=intent)[0]
    if seek_m is not None and seek_m == 0 and derived.field(thing, "touched", False):
        seek_speed = derived.under_load_speed(B, s0.q, d0, thing, spec.first_n, 0)
    stall_periods = max(1, math.ceil(derived.lag_window_s(B, seek_speed) / dt))
    lost_periods = stall_periods
    settled_periods = max(1, math.ceil(B.t_det() / dt))
    hold_dir_steps = HOLD_DIR_STEPS
    ax = ex.model.n < 6
    R0 = s0.R.copy()
    f_touch = touch_force(B, force_n, seek_speed)
    g = Guard(ex, None)
    ff_gain = FF_SHARE * ex.kd / ex.kp / dt        # joint target lead per joint target change in one period
    st = {"q_des": s0.q.copy(), "q_prev": s0.q.copy(), "s": s0, "f_prev": None, "p_cmd": s0.tip.copy(),
          "share": 0.0, "anchor": None, "resisted": False}
    done, free = [], [np.asarray(s0.q, dtype=float).copy()]
    out = {"points": [], "raw": [], "forces": [], "dirs": [], "stop": None, "peak_n": 0.0, "free_m": 0.0,
           "tick": None, "cap_why": "", "contact": None, "steps": 0, "across_m": 0.0, "slid_m": 0.0,
           "free": free, "felt": g.felt, "tail_turn": None, "touch_force_n": f_touch, "seek_speed": seek_speed}

    def period(d):
        """One control period toward st["p_cmd"]; returns the force along d held over two periods, and sets
        out["stop"] on a cap, a tick hook's stop request or an arm that cannot follow."""
        s_old = st["s"]
        st["q_prev"] = st["q_des"]
        st["q_des"], e_track, _ = ex._track(st["q_des"], st["p_cmd"], R0, ax)
        # the servos' own damping, fed forward for the share of the commanded motion the tip is seen to make: a
        # tip that stands still gets none of it, so the push on a part that does not move stays the static lead
        lead = ff_gain * st["share"] * (st["q_des"] - st["q_prev"])
        q_cmd = st["q_des"] + lead
        # Include damping feed-forward in the same lead budget as the static
        # target. Feed-forward must not wind the servo past the intended force.
        J = ex.model.jac(s_old.q)[:3]
        Cd = compliance(ex, s_old.q) @ d
        ahead = float(d @ J @ (q_cmd - s_old.q))
        cap = force_n * float(d @ Cd)
        if ahead > cap:
            q_cmd -= (J.T @ d) / ex.kp * ((ahead - cap) / float(d @ Cd))
        s = st["s"] = ex._period(q_cmd)
        asked = float(np.linalg.norm(ex.model.fk_tip(st["q_des"])[0] - ex.model.fk_tip(st["q_prev"])[0]))
        made = float((s.tip - s_old.tip) @ d)
        st["share"] = 0.5 * st["share"] + 0.5 * (min(1.0, max(0.0, made / asked)) if asked > 1e-7 else 0.0)
        done.append(st["q_des"].copy())
        st["resisted"] = st["resisted"] or s.Fmag > B.f_thr()
        if not st["resisted"]:
            free.append(s.q.copy())
        f_now = float(s.F @ d)
        fd = f_now if st["f_prev"] is None else min(f_now, st["f_prev"])
        st["f_prev"] = f_now
        out["peak_n"] = max(out["peak_n"], fd)
        if g.check(s) == "cap":
            out["cap_why"], out["stop"] = g.cap_why, "cap"
        elif getattr(ex, "_tick_stop", None):
            out["tick"] = out["stop"] = str(ex._tick_stop)
        elif e_track > B.rigid(s.q, d, fd):
            out["stop"] = "limit"
        return fd

    def free_across(d, c, p_ref, reset=True):
        """The command for one period of the push: on the push line at advance c along d; across the line it is
        reset to the MEASURED tip (so the servo's static force is along d only) when `reset`, or when the tip has
        advanced by as much as the arm's own sideways give can take up since the last reset. Returns the force the
        command asks."""
        s = st["s"]
        C = compliance(ex, s.q)
        Cd = C @ d
        tr = float((s.tip - p_ref) @ d)
        f = min(force_n, max(0.0, (c - tr) / float(d @ Cd)))
        give = float(np.trace(C) - float(d @ Cd)) / 2.0             # the tip's sideways give, m per N
        every = give * max(RESET_SHARE * f, RESET_FRICTION * joint_friction_n(ex, s.q))
        off = s.tip - p_ref
        if reset or st["anchor"] is None or abs(tr - st["anchor"][1]) >= every:
            st["anchor"] = (off - float(off @ d) * d, tr)
        lead = Cd * f                               # pushes with f along d and nothing across it
        st["p_cmd"] = p_ref + d * (tr + float(lead @ d)) + st["anchor"][0] + (lead - float(lead @ d) * d)
        return f

    # ---- seek: the tip on its line, as a guarded move, until the part pushes back
    d = np.asarray(d0, dtype=float)
    p0, fd, met = s0.tip.copy(), 0.0, False
    seeking_points, seeking_forces = [], []
    seek_m = B.rigid(s0.q, d0) if seek_m is None else float(seek_m)
    seek_s = B.limit_s(seek_m / seek_speed + B.t_settle()) if seek_speed > 0 else B.t_settle()
    if seek_m == 0 and not derived.field(thing, "touched", False):
        seek_s = 0  # no measured target band permits a blind advance here
    for k in range(math.ceil(seek_s / dt)):
        s = st["s"]
        lead_max = force_n * float(d @ compliance(ex, s.q) @ d)
        room = lead_max - float((st["p_cmd"] - s.tip) @ d)
        speed = seek_speed
        if seek_m == 0 and derived.field(thing, "touched", False):
            f_target = min(force_n, spec.ceiling_n, f_target + spec.rate_n_s(f_target) * dt)
            speed = derived.under_load_speed(B, s.q, d, thing, f_target, fd)
        st["p_cmd"] = st["p_cmd"] + d * min(speed * dt, room)
        fd = period(d)
        if out["stop"]:
            break
        if fd >= B.f_thr(0):
            seeking_points.append(st["s"].tip.copy())
            seeking_forces.append(fd)
        if fd >= f_touch and fd > 0:
            met = True
            break
        if seek_m > 0 and float((st["s"].tip - p0) @ d) >= seek_m:
            break
    out["free_m"] = max(0.0, float((st["s"].tip - p0) @ d))
    if out["stop"] or not met:
        out["stop"] = out["stop"] or "no_contact"
        out.update(s=st["s"], q_des=st["q_des"], done=done, d=d)
        return out

    # ---- push: measured steps, the command reset to the measured tip across the push line (see free_across)
    s = st["s"]
    p_ref = s.tip.copy()
    c = min(force_n, max(fd, f_touch)) * float(d @ compliance(ex, s.q) @ d)   # the lead that keeps the force
    slid = np.zeros(3)
    observed_thing = thing
    measured_kc = derived.observed_stiffness(B, seeking_points, seeking_forces, s.q, d)
    if measured_kc is not None:
        observed_thing = {"k_c": measured_kc}

    def hold(d, c, p_ref):
        """Stand still with the push force kept. The first periods let go what the step left; any drift across
        the push after that is the tip sliding on the part, not the part moving. Returns the force and the rate
        of that slide (m per s, across d)."""
        fd, tips = 0.0, []
        for k in range(release_periods + 2 * slide_periods):
            free_across(d, c, p_ref)
            fd = period(d)
            if out["stop"]:
                return fd, np.zeros(3)
            if k >= release_periods - 1:
                tips.append(st["s"].tip.copy())
        # two windows of the hold: a steady slide moves the tip as far in the second as in the first; a tip that
        # is only settling (the arm coming to rest) has all but stopped by the second and is not a slide
        a = tips[slide_periods] - tips[0]
        b = tips[2 * slide_periods] - tips[slide_periods]
        a, b = a - float(a @ d) * d, b - float(b @ d) * d
        na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
        # Drift is transverse to the push. Only force along that drift
        # can account for elastic servo give in the transverse measurement.
        drift_u = b / nb if nb > 0 else d
        drift_force = abs(float(st["s"].F @ drift_u))
        if nb < B.rigid(st["s"].q, drift_u, drift_force) or float(a @ b) < SLIDE_STEADY * na * nb or nb < SLIDE_STEADY * na:
            return fd, np.zeros(3)
        return fd, b / (slide_periods * dt)

    fd, _ = hold(d, c, p_ref)
    s = st["s"]
    out["contact"] = {"point": s.tip.copy(), "force_n": fd, "normal": -s.F / max(s.Fmag, 1e-9)}
    out["points"].append(s.tip.copy())
    out["raw"].append(s.tip.copy())
    out["forces"].append(fd)
    lost = 0
    n_steps = math.ceil(max_m / step) if step > 0 else 0
    for k in range(1, n_steps + 1):
        if out["stop"]:
            break
        s = st["s"]
        p_ref = s.tip.copy()
        st["anchor"] = None
        Cdd = float(d @ compliance(ex, s.q) @ d)
        c = min(c, force_n * Cdd)        # carried over: the lead that keeps the present force, nothing jumps
        goal, hist, stalled, n_cap = c + min(step, max_m - (k - 1) * step), [], False, 0
        for _ in range(int(STEP_MAX_S / dt)):
            s = st["s"]
            tr = float((s.tip - p_ref) @ d)
            lead_max = force_n * float(d @ compliance(ex, s.q) @ d)
            f_target = min(force_n, spec.ceiling_n, f_target + spec.rate_n_s(f_target) * dt)
            speed = derived.under_load_speed(B, s.q, d, observed_thing, f_target, fd)
            stall_periods = max(1, math.ceil(derived.lag_window_s(B, speed) / dt))
            lost_periods = stall_periods
            c = min(goal, c + speed * dt, tr + lead_max)
            free_across(d, c, p_ref, reset=False)
            fd = period(d)
            if out["stop"]:
                break
            tr = float((st["s"].tip - p_ref) @ d)
            hist.append(tr)
            lost = lost + 1 if fd < B.f_thr(0) else 0
            if lost >= lost_periods:
                out["stop"] = "released"
                break
            n_cap = n_cap + 1 if (c - tr >= (1 - B.e_tau()) * lead_max or fd >= (1 - B.e_tau()) * force_n) else 0
            if n_cap > stall_periods and hist[-1] - hist[-1 - stall_periods] < B.rigid(s.q, d, fd):
                stalled = True
                break
            if tr >= min(step, max_m - (k - 1) * step) - derived.path_noise_m(B, s.q, d) or (c >= goal - 1e-9 and len(hist) > settled_periods and hist[-1] - hist[-1 - settled_periods] < derived.path_noise_m(B, s.q, d)):
                break                  # the step is made, or the command is at its goal and the tip has settled
        rate = np.zeros(3)
        if out["stop"] is None:
            c = c - float((st["s"].tip - p_ref) @ d)      # re-base along the line: the lead left, from here
            p_ref = st["s"].tip.copy()
            fd, rate = hold(d, c, p_ref)
            c = c - float((st["s"].tip - p_ref) @ d)
        s = st["s"]
        # the slide measured in the hold went on through the whole step (same load, same contact): take it out
        slid = slid + rate * (len(hist) + release_periods + 2 * slide_periods) * dt
        out["slid_m"] = float(np.linalg.norm(slid))
        if float(np.linalg.norm(s.tip - out["raw"][-1])) >= 0.25 * step or not stalled:
            out["raw"].append(s.tip.copy())
            out["points"].append(s.tip - slid)
            out["forces"].append(fd)
            out["dirs"].append(d.copy())
            out["steps"] += 1
        off = s.tip - out["raw"][0]
        out["across_m"] = float(np.linalg.norm(off - (off @ d0) * d0))
        if out["stop"]:
            break
        if stalled:
            out["stop"] = "stall"
            break
        if out["across_m"] > derived.slip_m(B, thing, float(off @ d0), s.q, d0) and float(off @ d0) < 0.5 * out["across_m"]:
            out["stop"] = "slipped"
            break
        # The load stiffness is observed on this push, not assumed rigid once
        # the tip has followed a soft part. Compare settled step samples so the
        # servo's own bending is not mistaken for the part's displacement.
        if len(out["raw"]) >= 2:
            advance = float((out["raw"][-1] - out["raw"][-2]) @ d)
            rise = out["forces"][-1] - out["forces"][-2]
            if advance > derived.path_noise_m(B, s.q, d) and rise > 0:
                observed_thing = {"k_c": rise / advance}
        d_new = follow(out["points"], d, d0, min_m=derived.path_noise_m(B, s.q, d), tolerance_deg=tolerance_deg) if k <= n_steps - hold_dir_steps else d
        if d_new is not d:
            c = c * float(d_new @ compliance(ex, s.q) @ d_new) / max(float(d @ compliance(ex, s.q) @ d), 1e-12)
            d = d_new
    out["stop"] = out["stop"] or "goal"
    out["tail_turn"] = tail_turn(out["points"], hold_dir_steps) if out["steps"] == n_steps else None
    out.update(s=st["s"], q_des=st["q_des"], done=done, d=d)
    return out


def tail_turn(points, hold_dir_steps=None):
    """The bend of the path over the last hold_dir_steps steps, pushed along one fixed direction: (the angle
    between the chord over the first half of that stretch and the chord over the second half, rad; the path length
    between the two chords' middles, m), or None when the path is shorter than that stretch. A hinge of radius R
    turns the path by that length / R; a tip sliding where it is pushed turns it by nothing."""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    half = (len(points) - 1) // 2 if hold_dir_steps is None else max(1, hold_dir_steps // 2)
    if len(p) < 2 * half + 1:
        return None
    a, b = p[-1 - half] - p[-1 - 2 * half], p[-1] - p[-1 - half]
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-9 or nb < 1e-9:
        return None
    return math.acos(max(-1.0, min(1.0, float(a @ b) / (na * nb)))), 0.5 * (na + nb)


def follow(points, d, d0, min_m=0, tolerance_deg=None):
    """The next push direction: toward the way the tip was carried over the last steps, within the stated tolerance of the
    asked direction."""
    p = np.asarray(points, dtype=float)
    if len(p) < 3:
        return d
    back = p[-1] - p[max(0, len(p) - 4)]
    n = float(np.linalg.norm(back))
    if n <= min_m:
        return d
    t = back / n
    if float(t @ d0) <= 0.2:
        return d
    new = (1.0 - TURN_BLEND) * d + TURN_BLEND * t
    new = new / np.linalg.norm(new)
    ang = math.acos(max(-1.0, min(1.0, float(new @ d0))))
    cap = math.radians(tolerance_deg) if tolerance_deg is not None else math.pi / 4
    if ang > cap:                                   # slerp back onto the cone around the asked direction
        side = new - float(new @ d0) * d0
        side = side / max(float(np.linalg.norm(side)), 1e-12)
        new = math.cos(cap) * d0 + math.sin(cap) * side
    return new
