# SPDX-License-Identifier: AGPL-3.0-only
"""Kinematic torque cap, joint lead and held-pose force classification.

The body being checked may give a wrong external torque estimate. A cap uses
applied torque and failure to gain commanded speed. Wiggle force rises are
checked at the last commanded target before any easing: a rise gone at rest
is dynamic, a rise proportional to angle is a body bias, and another rise is
contact. Every reading also checks joint lead against free lag, inertia and
the starting lead. A wiggle can distrust a force rise on a freely following
arm and continue with joint lead as its contact rule. Both modes retain the
cap rules. Searches and retreats do not use the tip force cap.
"""
import math
import numpy as np
from rcpx import bounds
from rcpx import bounds_ratios as R
from rcpx.r2 import selfcheck_limits as limits
DEG = math.pi / 180.0


class LegGuard:
    def __init__(self, ex, plan, j=None, *, contact=True, force_cap=True):
        self.ex, self.plan, self.j = ex, plan, j
        s = ex._sense()
        self.q_start = s.q.copy()
        self.F_ref = s.F.copy() if contact or force_cap else None
        self.qd_previous = s.qd.copy()
        self.q_plan = ex._q_des.copy()
        self.slope = np.asarray(getattr(ex, '_sc_bias_lines', {}).get(j, np.zeros(3)))
        self.k = self.n_force = 0
        self.n_cap = np.zeros(ex.model.n, int)
        self.contact, self.force_cap = contact, force_cap
        self.peak = 0.0
        self.threshold = plan.guard.force_n
        self.last = None
        self.last_target = self.q_plan.copy()
        self.mode = getattr(ex, '_sc_guard_mode', 'force') if contact else 'force'
        self.inertia = np.diag(bounds.of(ex).robot.mass_matrix(s.q)).copy()
        self.lead_start = np.abs(self.q_plan - s.q)
        self.extra_peak = 0.0
        self.lead_hits = 0
        self.lead_step = limits.contact_step(ex)
        self.lead_step_peak = float(np.max(self.lead_step))
        self.lead_extra = np.zeros(ex.model.n)

    def check(self, s):
        self.last = s
        ex, B, path = self.ex, bounds.of(self.ex), self.plan.path
        if self.k < len(path):
            q = np.asarray(path[self.k])
            prev = np.asarray(path[self.k - 1]) if self.k > 0 else self.q_plan
            older = np.asarray(path[self.k - 2]) if self.k > 1 else self.q_plan
            joint_velocity = (q - prev) / ex.dt
            joint_accel = (q - prev - (prev - older)) / ex.dt ** 2
            velocity = ex.model.jac(s.q)[:3] @ joint_velocity
            accel = ex.model.jac(s.q)[:3] @ joint_accel
            step = q - prev
        else:
            velocity, accel, step = np.zeros(3), np.zeros(3), np.zeros(ex.model.n)
            joint_velocity = joint_accel = np.zeros(ex.model.n)
            # Repeating the final target commands zero speed, with one last
            # deceleration sample. Its inertial lead is still free motion.
            if self.k == len(path) and path:
                prev = np.asarray(path[-1])
                older = np.asarray(path[-2]) if len(path) > 1 else self.q_plan
                joint_accel = -(prev - older) / ex.dt ** 2
        self.last_target = ex._q_des.copy()
        self.k += 1
        v = float(np.linalg.norm(velocity))
        lead_free = (np.abs(joint_velocity) * B.lag(1.0) +
                     np.abs(self.inertia * joint_accel) / ex.kp + self.lead_start)
        self.lead_extra = np.abs(self.last_target - s.q) - lead_free
        self.lead_step = limits.contact_step(ex, v)
        self.lead_step_peak = max(self.lead_step_peak, float(np.max(self.lead_step)))
        excess = float(np.max(self.lead_extra - self.lead_step))
        self.extra_peak = max(self.extra_peak, excess)
        self.lead_hits = self.lead_hits + 1 if excess > 0 else 0
        tick = getattr(ex, '_tick_stop', None)
        if tick:
            return str(tick)
        lim = B.tau_max() * ex.cfg['torque_frac']
        gaining = np.sign(step) * (s.qd - self.qd_previous) > 0
        at_cap = (np.abs(s.tau) >= lim * (1 - B.e_tau())) & ~gaining
        self.qd_previous = s.qd.copy()
        self.n_cap = np.where(at_cap, self.n_cap + 1, 0)
        if np.any(self.n_cap >= self.plan.guard.readings):
            k = int(np.argmax(self.n_cap))
            self.cap_reason = f"applied torque of joint {ex.model.joint_names[k]} reached its rating without gaining commanded speed"
            return 'cap'
        if not self.contact and not self.force_cap:
            return None
        rise_vec = s.F - self.F_ref
        if self.j is not None:
            rise_vec = rise_vec - self.slope * (s.q[self.j] - self.q_start[self.j])
        rise = float(np.linalg.norm(rise_vec))
        self.peak = max(self.peak, rise)
        if self.force_cap and rise > self.plan.guard.cap_n:
            self.cap_reason = f"tip force rise {rise:g} N passed the plan cap {self.plan.guard.cap_n:g} N"
            return 'cap'
        if not self.contact:
            return None
        if self.mode == 'lead':
            return 'contact' if self.lead_hits >= self.plan.guard.readings else None
        a = float(np.linalg.norm(accel))
        u = velocity / v if v > 0 else (accel / a if a > 0 else np.array([0.0, 0.0, 1.0]))
        inertia = B.m_eff(s.q, u) * a if a > 0 else 0.0
        self.threshold = self.plan.guard.force_n + inertia
        self.n_force = self.n_force + 1 if rise > self.threshold else 0
        if rise > self.plan.guard.one_reading_n + inertia or self.n_force >= self.plan.guard.readings:
            return 'contact'
        return None


def classify(ex, j, q_start, F_ref, guard):
    """Hold the last target, then test dynamic error or a force line over angle."""
    from rcpx.r2 import selfcheck_motion as motion
    B = bounds.of(ex)
    held = bounds.of(ex).plan(guard.last_target, [], 'retreat', intent=getattr(ex, '_sc_intent', None))
    cap = LegGuard(ex, held, j, contact=False, force_cap=False)
    stop, s = motion.hold_force_quiet(ex, guard.last_target, cap)
    if stop:
        guard.classified_stop = stop
        return 'contact'
    F_stop, q_stop = s.F.copy(), s.q.copy()
    guard.q_hold = q_stop.copy()
    threshold = B.f_thr(0.0, 0.0)
    if float(np.linalg.norm(F_stop - F_ref - guard.slope * (q_stop[j] - q_start[j]))) <= threshold:
        return 'dynamic'
    delta = float(q_stop[j] - q_start[j])
    if delta == 0:
        return 'contact'
    q_mid = guard.last_target.copy()
    # Keep the other goals, and subtract the observed held tracking error so
    # this joint's reading, rather than its servo goal, reaches the midpoint.
    q_mid[j] = q_start[j] + R.GIVE * delta - (q_stop[j] - guard.last_target[j])
    mid_read = q_start[j] + R.GIVE * delta
    load = float(np.linalg.norm(F_stop - F_ref))
    quantum = R.QUANT * B.joint_quantum()[j]
    need = max(quantum, min(R.REL * abs(delta), threshold * abs(delta) / load)) if load > 0 else quantum
    previous_error = None
    while True:
        plan = motion.plan_leg(ex, ex._q_des, q_mid, 'retreat', getattr(ex, '_sc_intent', None))
        if plan.refused:
            return 'contact'
        cap = LegGuard(ex, plan, j, contact=False, force_cap=False)
        stop, s = motion.stream(ex, plan, cap)
        if stop != 'goal':
            guard.classified_stop = stop
            return 'contact'
        stop, s = motion.hold_force_quiet(ex, q_mid, cap)
        if stop:
            guard.classified_stop = stop
            return 'contact'
        error = mid_read - s.q[j]
        if abs(error) <= need:
            break
        if previous_error is not None and abs(previous_error) - abs(error) <= max(quantum, R.REL * abs(previous_error)):
            return 'contact'
        previous_error = error
        q_mid[j] += error
    if float(np.linalg.norm((s.F - F_ref) - R.GIVE * (F_stop - F_ref))) <= threshold:
        ex._sc_bias_lines[j] = (F_stop - F_ref) / delta
        return 'bias'
    return 'contact'
