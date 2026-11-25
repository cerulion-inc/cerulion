# SPDX-License-Identifier: AGPL-3.0-only
"""Derived quantities of the waiting package: what "at rest", "moved", "dropping" and "pushing" mean on THIS
robot at THIS pose. Nothing here is a typed limit: every value comes from the body model (gravity torque, joint
inertia, dry friction, the servo gains the executor holds) and from rcpx.bounds (the joint quantum, the joint
torque threshold, the detection and settling times). Pure helpers: no state, no motion.

The one angle everything else hangs on is the arm's REST ANGLE at a pose: the largest over the joints of
- two encoder steps (QUANT joint quanta),
- the servo's own sag under the joint's gravity load, abs(g_j) / kp_j (the executor's gravity hold is exactly
  that goal offset: rcpx/control.py _command), and
- the joint's give under a torque the robot cannot tell from none, tau_thr_j / kp_j.
A joint that is no further than this from where it should be has not moved in any way the robot can vouch for;
one that is further has. It replaces the typed half degree of the hold's drift test, of the re-hold of
release_check and of the park's "moved" and "stopped short" tests (on the bench arms it comes out at 0.38 to 0.51
degrees at their home poses, by the gravity term).

Every comparison uses <= for "inside" and > for "past", so an exactly still arm is inside even when every term is
zero (a noise-free simulator with no stated encoder step and no gravity load).
"""

import math

import mujoco
import numpy as np

from rcpx import bounds
from rcpx import bounds_laws as laws
from rcpx import bounds_ratios as R


def _scratch(ex):
    data = getattr(ex, "_wt_release_data", None)
    if data is None:
        data = mujoco.MjData(ex.model.mj)
        ex._wt_release_data = data
    return data


def mass_matrix(ex, q):
    """The arm joints' mass matrix at q (kg m^2, rotor armature included) and the body model's dry friction per
    joint (N m)."""
    m, d = ex.model.mj, _scratch(ex)
    dadr = ex._dadr
    d.qpos[:] = ex._qfull
    d.qpos[ex._qadr] = q
    mujoco.mj_forward(m, d)
    out, e, res = np.zeros((len(dadr), len(dadr))), np.zeros(m.nv), np.zeros(m.nv)
    for k, a in enumerate(dadr):
        e[:] = 0.0
        e[a] = 1.0
        mujoco.mj_mulM(m, d, res, e)
        out[:, k] = res[dadr]
    return out, np.asarray(m.dof_frictionloss[dadr], dtype=float)


def joint_inertia(ex, q):
    """Each joint's own inertia at q with the others held (kg m^2: the diagonal of the mass matrix) and the body
    model's dry friction per joint."""
    M, friction = mass_matrix(ex, q)
    return np.diag(M).copy(), friction


def servo_sag(ex, q):
    """Per joint (rad): how far the servo sags under the joint's own gravity load when its gravity hold is taken
    away, abs(g_j(q)) / kp_j."""
    return np.abs(np.asarray(ex.model.gravity_torque(q), dtype=float)) / ex.kp


def joint_step(ex):
    """Per joint (rad): the least turn of a joint the robot can tell from none: two encoder steps, or its give
    under the joint torque threshold, whichever is larger."""
    B = bounds.of(ex)
    return np.maximum(R.QUANT * B.joint_quantum(), B.tau_thr(0.0) / ex.kp)


def rest_angle(ex, q):
    """(angle in rad, words): the arm's rest angle at pose q (module docstring), with the term and the joint that
    decided it."""
    B = bounds.of(ex)
    terms = (R.QUANT * B.joint_quantum(), servo_sag(ex, q), B.tau_thr(0.0) / ex.kp)
    term, j = max(((i, k) for i, values in enumerate(terms) for k in range(len(values))),
                  key=lambda ik: terms[ik[0]][ik[1]])
    name = ex.model.joint_names[j]
    words = (f"two encoder steps of joint {name}",
             f"the servo's own sag under joint {name}'s gravity load",
             f"the give of joint {name} under a torque the robot cannot tell from none")
    return float(terms[term][j]), words[term]


def free_accel(ex, q):
    """Per joint (rad/s^2): how fast gravity would turn the joint with no motor torque, the others held:
    max(0, abs(g_j) - dry friction_j) / M_jj."""
    inertia, friction = joint_inertia(ex, q)
    net = np.maximum(0.0, np.abs(np.asarray(ex.model.gravity_torque(q), dtype=float)) - friction)
    return np.divide(net, inertia, out=np.zeros_like(net), where=inertia > 0)


def sink_speed(ex, q):
    """Per joint (rad/s): how fast a joint only its motor carries sinks in the follow-down, where the goal follows
    the joint and the servo's damping is all that slows it: max(0, abs(g_j) - dry friction_j) / kd_j."""
    _, friction = joint_inertia(ex, q)
    net = np.maximum(0.0, np.abs(np.asarray(ex.model.gravity_torque(q), dtype=float)) - friction)
    return np.divide(net, ex.kd, out=np.zeros_like(net), where=ex.kd > 0)


def rehold(ex, q):
    """(array in rad per joint, words per joint): the re-hold angle: a released joint that has turned further than
    this is re-held at once. It is the arm's rest angle for every joint: the motion the arm makes anyway when a
    gravity hold is taken away. (It is not cut down by the clearance of the arm's lowest point: measured on an arm
    lying on a table, such a cut read a joint the table carries as dropping, because its settling passed the
    clearance angle; a sag of the rest angle onto a surface that near is the arm coming to rest on it.)"""
    A, words = rest_angle(ex, q)
    return np.full(ex.model.n, A), [f"the arm's rest angle ({words})"] * ex.model.n


def quiet_need(ex, q):
    """rad: what "still" and "too small to measure" mean around the test: two encoder steps, or REL of the rest
    angle, whichever is larger."""
    return max(float(np.max(R.QUANT * bounds.of(ex).joint_quantum())), R.REL * rest_angle(ex, q)[0])


def moving(ex, s):
    """The arm is still moving: over one detection time some joint would turn more than quiet_need."""
    return float(np.max(np.abs(s.qd))) * bounds.of(ex).t_det() > quiet_need(ex, s.q)


def pushing(ex, s):
    """(bool, joint name, lead in rad): the servo is pushing on something: the commanded goal of some joint leads
    its reading by more than the joint can resolve (joint_step) and more than REL of the rest angle. An arm that
    was eased, or that only rests on a surface under its own weight, has no such lead."""
    lead = np.abs(np.asarray(ex._q_des, dtype=float) - s.q)
    over = lead - np.maximum(joint_step(ex), R.REL * rest_angle(ex, s.q)[0])
    j = int(np.argmax(over))
    return bool(over[j] > 0), ex.model.joint_names[j], float(lead[j])


def sag_window(ex, limit, accel, stay):
    """Control periods of the sag step of one joint: the time a free joint needs to turn the re-hold angle under
    its own gravity torque, sqrt(2 x limit / accel), never under the detection time on two readings, never over
    the stay time."""
    window = math.sqrt(laws.TWO * limit / accel) if accel > 0 else 0.0
    window = min(stay, max(bounds.of(ex).t_det(), window))
    return max(1, int(math.ceil(window / ex.dt - laws.ROUND)))


def stay_offer(ex, q, measurable, lim):
    """(seconds, words): the stay time RCP offers when the caller states none: long enough for the test to reach
    a verdict on every joint it can measure. The largest of the servo's settling time, the time two encoder steps
    of sag take to show on the least loaded measurable joint, and the time the slowest measurable joint needs to
    sink its re-hold angle in the follow-down."""
    B = bounds.of(ex)
    best, words = B.t_settle(), "the servo's settling time"
    accel, sink, quantum = free_accel(ex, q), sink_speed(ex, q), B.joint_quantum()
    for j in measurable:
        name = ex.model.joint_names[j]
        if accel[j] > 0 and quantum[j] > 0:
            t = math.sqrt(laws.TWO * R.QUANT * quantum[j] / accel[j])
            if t > best:
                best, words = t, f"the time two encoder steps of sag take to show on joint {name}"
        if sink[j] > 0:
            t = lim[j] / sink[j]
            if t > best:
                best, words = t, f"the time joint {name} needs to sink its re-hold angle against its servo's damping"
    return max(ex.dt, best), words


def rest_rule(ex, rehold_j, stay_s, j):
    """(periods, threshold in rad): "at rest" for joint j with its torque truly off (the whole-arm cut). The
    slowest sink that matters is w = rehold_j / stay_s: a slower one does not reach the re-hold angle within the
    stay time. The window is the time that sink takes to show two encoder steps (the detection time where no
    step is stated); the joint is at rest when its reading changed by no more than max(two steps, w x window)
    over the window. So a judgement is never made under the encoder's own step."""
    B = bounds.of(ex)
    step = R.QUANT * B.joint_quantum()[j]
    w = rehold_j / stay_s
    T = step / w if step > 0 and w > 0 else B.t_det()
    periods = max(1, int(math.ceil(T / ex.dt - laws.ROUND)))
    return periods, max(step, w * periods * ex.dt)


def follow_rule(ex, q, j, rehold_j, stay_s):
    """(periods, threshold in rad): "came to rest" for joint j in the follow-down, where a joint only its motor
    carries sinks at sink_speed (its weight against the servo's damping). It has come to rest (something else
    took its weight) when it sinks slower than REL of that speed AND slower than the slowest sink that matters
    (rehold_j / stay_s, as rest_rule): its reading changed by no more than max(two encoder steps, w x window)
    over the window in which that speed w shows two steps (the detection time where no step is stated). A joint
    that keeps its sink speed is carried by nothing, however slow that speed is (with its torque truly off it
    would not be damped at all), and one that creeps on fast enough to pass the re-hold angle within the stay
    time has not come to rest either."""
    B = bounds.of(ex)
    step = R.QUANT * B.joint_quantum()[j]
    w = min(R.REL * sink_speed(ex, q)[j], rehold_j / stay_s)
    T = step / w if step > 0 and w > 0 else B.t_det()
    periods = max(1, int(math.ceil(T / ex.dt - laws.ROUND)))
    return periods, max(step, w * periods * ex.dt)
