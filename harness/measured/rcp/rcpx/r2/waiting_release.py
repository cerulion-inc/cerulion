# SPDX-License-Identifier: AGPL-3.0-only
"""release_check of the waiting package: which joints stay put without motor torque at this pose.

The Backend interface cuts torque for the whole arm only (command(q_target, torque_frac) takes one share), so a
real cut on one joint cannot be had through it. Each joint is instead released in two steps while the other joints
keep their goals:
1. SAG: its servo goal is set to its present position, so the servo gives it no holding torque where it stands,
   only a spring back to that place. A joint the motor alone carries sags by its gravity torque over the servo
   stiffness; a joint something else carries (a surface under the arm, joint friction, a joint stop) hardly
   moves. The step lasts the time a free joint would need to turn the re-hold angle under its own gravity torque
   (never under the detection time). The measured sag is compared with the sag the body model predicts for an arm
   in free space (RobotModel.gravity_torque, the joint's inertia, the servo gains and the model's own joint
   friction) inside the relative error accepted of that prediction (REL of rcpx.bounds_ratios).
2. FOLLOW DOWN (only a joint that sagged as a carried one): the goal follows the joint every control period, so
   it sinks under its load against the servo's damping. It either comes to rest (it sinks slower than REL of the
   speed its own weight gives it: a support just under it took the weight, it holds from there), or passes the
   re-hold angle (re-held at once: it would drop), or is still sinking when the stay time is up (it would drop).
A joint that moves more than the re-hold angle at any point is re-held at once.

The numbers are the session's (rcpx/r2/waiting_limits.py): the re-hold angle is the arm's rest angle at the pose;
the stay time is the caller's (stay_s), or RCP's offer; a sag under what "still" means there is too small to
measure and the body model alone answers (predicted).

Measured: moved_deg, reheld, came_to_rest. Predicted from the body model (marked so in the result):
expected_sag_deg, gravity_load, free_drop_deg (what a real torque cut of the stay time would do to a free joint).

The whole arm is another matter: torque_frac 0 is a real cut. When no joint reads as dropping, cut_whole_arm()
cuts the torque of the whole arm; it is re-held at once when a joint passes its re-hold angle, lasts at least the
longest sag window, and ends at rest when no joint's reading changed by more than two encoder steps (or what the
slowest sink that matters makes) over the window that sink needs to show them. That measurement, not the per
joint emulation, decides arm_holds. A sink too slow to reach the re-hold angle within the stay time is not told
from rest, and the result says how slow that is.

One thing no short cut can show: an arm that stands BALANCED over its own weight (links straight up over a joint)
lies as still as one that rests on a table until something tips it. unstable_balance() asks the body model: of
the joints with no gravity load to measure, which stand over their load so that gravity already turns them past
the model's dry friction, or would after a turn the robot cannot tell from none, and would then fall within the
stay time. Those joints read cannot tell (balanced), and the whole-arm cut is not made for them: arm_holds is
then false. Joints that stand over their load but that friction keeps there are named in held_by_friction: the
cut is made, and the result says that only friction keeps them.

Each joint row says what its verdict rests on (basis): "measured" (the sag test, the follow-down, a re-hold) or
"predicted" (no load to measure: the body model alone). Nothing here reads simulator ground truth.
"""

import math

import numpy as np

from rcpx import bounds
from rcpx import bounds_laws as laws
from rcpx import bounds_ratios as R
from rcpx.r2 import waiting_limits as limits
from rcpx.r2.waiting_limits import joint_inertia, mass_matrix

BALANCE_SHARE = 0.1      # a joint takes part in a fall when gravity drives it over this share of the most driven
GRAVITY_STEP_RAD = 1e-3  # the finite difference step that reads how gravity torque changes with angle
PREDICTED = ("expected_sag_deg", "gravity_load", "free_drop_deg", "balanced")


def unstable_balance(ex, q, joints, stay):
    """Of `joints` (indices: the joints with no gravity load to measure at q), those that stand balanced over
    their load instead of hanging under it or lying neutral. With the other joints held, the body model gives the
    change of gravity torque with angle (the curvature of the arm's height) and the mass matrix. A direction in
    which a turn pulls the joints further the same way, and in which the fall grows e-fold within the stay time,
    is a fall waiting to happen when the pull beats the model's dry friction: the pull gravity already has where
    the arm stands (the pose is never exactly on balance) plus the pull a turn of one joint step adds (the least
    turn the robot can tell from none: it cannot know it stands nearer than that). A torque cut cannot show it
    (the arm stands still until tipped), so this is predicted. Returns (tipped, held): the joints gravity drives
    in such a fall (a joint that only turns along through the links' inertia, like one about a vertical axis, is
    not one), and the joints of unstable directions that friction keeps where they stand; both sorted."""
    S = [int(j) for j in joints]
    if not S:
        return [], []
    q = np.asarray(q, dtype=float)
    H = np.zeros((len(S), len(S)))
    for k, j in enumerate(S):
        e = np.zeros(ex.model.n)
        e[j] = GRAVITY_STEP_RAD
        H[:, k] = (np.asarray(ex.model.gravity_torque(q + e), dtype=float) -
                   np.asarray(ex.model.gravity_torque(q - e), dtype=float))[S] / (laws.TWO * GRAVITY_STEP_RAD)
    M, friction = mass_matrix(ex, q)
    M, friction = M[np.ix_(S, S)], friction[S]
    g = np.asarray(ex.model.gravity_torque(q), dtype=float)[S]
    step = float(np.max(limits.joint_step(ex)[S]))
    Li = np.linalg.inv(np.linalg.cholesky(M))
    H = (H + H.T) / laws.TWO
    rate2, modes = np.linalg.eigh(Li @ -H @ Li.T)           # M v'' = -H v: a direction with rate2 > 0 grows
    out, kept = set(), set()
    for mu, w in zip(rate2, modes.T):
        if mu <= 0.0 or mu * stay ** 2 < 1.0:              # stable, or too slow to grow within the stay time
            continue
        v = Li.T @ w
        v = v / float(np.max(np.abs(v)))                   # the joint that turns most turns by one unit
        held = float(np.abs(v) @ friction)                 # what dry friction holds along this direction
        pull = abs(float(v @ g)) + mu * float(v @ M @ v) * step
        drive = (-H @ v) * v                               # per joint: gravity pushing it further the way it turned
        driven = {S[k] for k in range(len(S)) if drive[k] >= BALANCE_SHARE * float(np.max(drive))}
        if pull > held:
            out |= driven
        else:
            kept |= driven
    return sorted(out), sorted(kept - out)


def sag_after(pull, friction, kp, kd, inertia, seconds, h=0.0005):
    """How far (rad) one joint turns in `seconds` under a constant torque `pull` with a servo spring kp back to
    where it started, servo damping kd and dry friction, from rest (the other joints held)."""
    x = v = 0.0
    for _ in range(int(round(seconds / h))):
        drive = pull - kp * x - kd * v
        if abs(v) < 1e-9 and abs(drive) <= friction:
            v = 0.0
            continue
        a = (drive - friction * (1.0 if (v > 0 or (v == 0 and drive > 0)) else -1.0)) / inertia
        v2 = v + a * h
        if v != 0.0 and v * v2 < 0.0:      # friction stops the joint inside the step
            v2 = 0.0
        v = v2
        x += v * h
    return max(0.0, x)


def predict(ex, q, lim, stay):
    """Per joint, from the body model at q: the gravity torque the motor holds, the sag the release should show
    over the joint's own sag window if only the motor carries the joint, whether that sag is large enough to
    measure, and what a real torque cut of the stay time would do."""
    g = np.asarray(ex.model.gravity_torque(q), dtype=float)
    inertia, friction = joint_inertia(ex, q)
    acc, need = limits.free_accel(ex, q), limits.joint_step(ex)
    rows = []
    for j in range(ex.model.n):
        n = limits.sag_window(ex, lim[j], acc[j], stay)
        sag = sag_after(abs(float(g[j])), float(friction[j]), float(ex.kp[j]), float(ex.kd[j]), float(inertia[j]),
                        n * ex.dt)
        if not sag > need[j]:              # too small to measure: released for the detection time only, so that
            n = limits.sag_window(ex, 0.0, 0.0, stay)      # a surprise is still caught and nothing is waited for
            sag = sag_after(abs(float(g[j])), float(friction[j]), float(ex.kp[j]), float(ex.kd[j]),
                            float(inertia[j]), n * ex.dt)
        drop = acc[j] * stay ** 2 / laws.TWO
        rows.append({"pull_sign": -1.0 if g[j] > 0 else 1.0,      # the way the joint turns when it sags
                     "expected_sag_deg": math.degrees(sag),
                     "gravity_load": float(g[j]) / float(ex.torque_limit[j]),
                     "free_drop_deg": min(math.degrees(math.pi), math.degrees(drop)),
                     "stays": bool(drop < lim[j]), "measurable": bool(sag > need[j]), "accel": float(acc[j]),
                     "periods": n, "window_s": round(n * ex.dt, 6), "rehold_deg": math.degrees(lim[j]),
                     "stay_s": stay, "least_deg": math.degrees(need[j])})
    return rows


def verdict(moved_deg, along_deg, reheld, at_s, pred):
    """(holds, why, basis) after the sag step. holds: True, False, None (cannot tell), "carried" or "partly" (the
    follow-down goes on); basis: "measured", or "predicted" when the load is too small to measure. A measurable
    sag under REL of the prediction: something else carries the joint. Within REL of the whole prediction: only
    the motor carries it ("carried": the follow-down decides between a support just under it and a drop). Between
    the two the joint is partly carried ("partly"): the follow-down can still show that it drops, but a rest
    there proves nothing, because the servo's damping slows a creep that a real cut would not slow."""
    exp, e = pred["expected_sag_deg"], R.REL
    if reheld:
        return False, (f"it moved {moved_deg:.2f} deg within {at_s:.2f} s of losing its holding torque and was "
                       "re-held: the motor carries this joint"), "measured"
    if pred["accel"] == 0:
        return True, ("no gravity load on this joint at this pose (predicted from the body model): nothing would "
                      "turn it"), "predicted"
    if pred["measurable"]:
        if moved_deg <= e * exp:
            why = (f"it sagged {moved_deg:.3f} deg where a free arm would sag {exp:.3f} deg: something other "
                   "than the motor carries this joint (a surface under the arm, friction or a joint stop)")
            return True, why, "measured"
        if along_deg >= (1 - e) * exp:
            return "carried", "", "measured"
        if along_deg > e * exp:
            return "partly", "", "measured"
        return None, (f"it moved {moved_deg:.3f} deg, but not the way gravity pulls it (predicted sag "
                      f"{exp:.3f} deg): something else moved it, the test cannot tell whether it would stay"), \
            "measured"
    if pred["stays"]:
        return True, ("no gravity load to speak of on this joint at this pose (predicted from the body model; a "
                      f"sag of {exp:.4f} deg is under the {pred['least_deg']:.3f} deg the test can measure here)"), \
            "predicted"
    return None, (f"the gravity load on this joint is too small for the test to measure (predicted sag "
                  f"{exp:.4f} deg, under {pred['least_deg']:.3f} deg), yet a free joint would turn about "
                  f"{pred['free_drop_deg']:.2f} deg in {pred['stay_s']:g} s (predicted): cannot tell"), "predicted"


def mark_balanced(row):
    """A joint the body model says stands balanced over its load: its verdict becomes cannot tell, and says why."""
    row.update(holds=None, balanced=True, basis="predicted",
               why=("no gravity load on this joint here, but it stands balanced over its load (predicted from the "
                    "body model): a push of no more than the hand's own weight tips it and it would keep falling, "
                    "so with torque off the first push tips it; a short release cannot show that"))
    return row


def _past(s, q0, j, limit, A):
    """A released joint is past its re-hold angle, or (per joint release) another joint is past the arm's rest
    angle: the others are servo held, so one that moves that far is being pushed."""
    change = np.abs(s.q - q0)
    if j is None:
        return bool(np.any(change > limit))
    other = change.copy()
    other[j] = 0.0
    return bool(change[j] > limit or np.any(other > A))


def _follow_down(ex, j, q0, cmd, limit, A, stay):
    """The goal of joint j follows the joint every control period: ("moved" | "rest" | "sinking", periods)."""
    n_rest, threshold = limits.follow_rule(ex, q0, j, limit, stay)
    n = max(n_rest + 1, int(math.ceil(stay / ex.dt - laws.ROUND)))
    trail = [float(ex._sense().q[j])]
    for k in range(n):
        s = ex._sense()
        cmd = cmd.copy()
        cmd[j] = s.q[j]
        ex.backend.command(cmd, torque_frac=ex._frac_cmd, gripper=ex._gripper_cmd)
        ex._q_cmd = cmd
        ex.backend.step(1)
        s = ex._sense()
        trail.append(float(s.q[j]))
        if _past(s, q0, j, limit, A):
            return "moved", k + 1
        if len(trail) > n_rest and abs(trail[-1] - trail[-1 - n_rest]) <= threshold:
            return "rest", k + 1
    return "sinking", n


def _back(ex, goal, q0):
    """The commanded goal again, with its gravity hold, until the arm is back and still (at most the settling
    time with its margin). Returns how far it then is from where it stood (deg)."""
    ex._command(goal)
    B = bounds.of(ex)
    back = float(np.max(np.abs(ex._sense().q - q0)))
    for _ in range(max(1, int(math.ceil(B.limit_s(B.t_settle()) / ex.dt - laws.ROUND)))):
        s = ex._period(goal)
        back = float(np.max(np.abs(s.q - q0)))
        if back <= limits.quiet_need(ex, q0) and not limits.moving(ex, s):
            break
    return math.degrees(back)


def release_one(ex, j, goal, pred, lim, stay):
    """Release joint j (the sag step, then the follow-down when it sagged as a carried joint), re-hold, let the
    arm come back. Returns the measured row. A joint gravity cannot turn at all (pred accel 0) is not released."""
    s = ex._sense()
    q0 = s.q.copy()
    A = limits.rest_angle(ex, q0)[0]
    cmd = np.asarray(ex._q_cmd, dtype=float).copy()
    cmd[j] = q0[j]
    n = pred["periods"] if pred["accel"] > 0 else 0
    reheld, periods, rest = False, n, None
    limit_deg = math.degrees(lim[j])
    try:
        if n:
            ex.backend.command(cmd, torque_frac=ex._frac_cmd, gripper=ex._gripper_cmd)
            ex._q_cmd = cmd
            for k in range(n):
                ex.backend.step(1)
                s = ex._sense()
                if _past(s, q0, j, lim[j], A):
                    reheld, periods = True, k + 1
                    break
        sag = float(s.q[j] - q0[j])
        sag_deg = abs(math.degrees(sag))
        holds, why, basis = verdict(sag_deg, math.degrees(sag) * pred["pull_sign"], reheld, periods * ex.dt, pred)
        if holds in ("carried", "partly"):
            partly = holds == "partly"
            end, more = _follow_down(ex, j, q0, cmd, lim[j], A, stay)
            periods += more
            s = ex._sense()
            sank = abs(math.degrees(float(s.q[j] - q0[j])))
            if end == "rest" and partly:
                holds, rest = None, True
                why = (f"it sagged {sag_deg:.3f} deg of a predicted {pred['expected_sag_deg']:.3f} (partly carried "
                       f"by something else), then sank {sank:.3f} deg and came to rest while its servo still "
                       "damped it; a real cut has no such damping, so it may creep on: the test cannot tell "
                       "whether it would stay")
            elif end == "rest":
                holds, rest = True, True
                why = (f"it sagged as a joint its motor carries ({sag_deg:.3f} deg), then sank {sank:.3f} deg and "
                       "came to rest: a support just under the arm carries it from there")
            else:
                holds, rest, reheld = False, False, end == "moved"
                why = (f"it sagged {sag_deg:.3f} deg where the body model predicts {pred['expected_sag_deg']:.3f} "
                       "deg for a joint only the motor carries and " +
                       (f"kept sinking past {limit_deg:.3g} deg in {periods * ex.dt:.2f} s (re-held)"
                        if end == "moved" else f"was still sinking after {periods * ex.dt:.2f} s") +
                       f": with torque off it would drop (about {pred['free_drop_deg']:.1f} deg in "
                       f"{stay:g} s, predicted)")
        moved = float(s.q[j] - q0[j])
    finally:
        back = _back(ex, goal, q0) if n else math.degrees(float(np.max(np.abs(ex._sense().q - q0))))
    return {"joint": ex.model.joint_names[j], "holds": holds, "basis": basis, "why": why, "balanced": False,
            "moved_deg": round(abs(math.degrees(moved)), 4), "sag_deg": round(sag_deg, 4),
            "expected_sag_deg": round(pred["expected_sag_deg"], 4), "gravity_load": round(pred["gravity_load"], 4),
            "free_drop_deg": round(pred["free_drop_deg"], 2), "reheld": reheld, "came_to_rest": rest,
            "released_s": round(periods * ex.dt, 3), "back_within_deg": round(back, 4),
            "rehold_deg": round(limit_deg, 4), "window_s": pred["window_s"] if n else 0.0,
            "measurable": pred["measurable"], "prediction_error": R.REL}


def cut_whole_arm(ex, goal, lim, stay):
    """The one real torque cut the Backend interface offers: the torque share of the whole arm at zero, re-held at
    once when any joint moves more than its re-hold angle. The cut lasts at least the longest sag window (the time
    a free joint would need to turn its re-hold angle) and ends at rest when every joint is at rest by
    waiting_limits.rest_rule; it goes on while the arm still creeps, up to the settling time or the rest window,
    whichever is longer: an arm that is sliding has not shown that it holds. Measured, not emulated. holds: it
    came to rest inside the re-hold angle."""
    q0 = ex._sense().q.copy()
    B = bounds.of(ex)
    accel = limits.free_accel(ex, q0)
    n_min = max((limits.sag_window(ex, lim[j], a, stay) for j, a in enumerate(accel) if a > 0), default=1)
    rules = [limits.rest_rule(ex, lim[j], stay, j) for j in range(ex.model.n)]
    n_rest = max(n for n, _ in rules)
    n_max = max(n_min, n_rest, int(math.ceil(B.t_settle() / ex.dt - laws.ROUND))) + n_rest
    ended, k_end, moved, trail = "creeping", n_max, 0.0, [q0.copy()]
    try:
        ex.backend.command(ex._q_cmd, torque_frac=0.0, gripper=ex._gripper_cmd)
        ex._frac_cmd = 0.0
        for k in range(n_max):
            ex.backend.step(1)
            s = ex._sense()
            trail.append(s.q.copy())
            moved = max(moved, float(np.max(np.abs(s.q - q0))))
            if _past(s, q0, None, lim, None):
                ended, k_end = "moved", k + 1
                break
            if k + 1 >= max(n_min, n_rest) and all(abs(trail[-1][j] - trail[-1 - n][j]) <= t
                                                  for j, (n, t) in enumerate(rules)):
                ended, k_end = "rest", k + 1
                break
    finally:
        back = _back(ex, goal, q0)         # torque back on, the commanded goal with its gravity hold
    return {"done": True, "holds": ended == "rest", "ended": ended, "moved_deg": round(math.degrees(moved), 4),
            "reheld": ended == "moved", "cut_s": round(k_end * ex.dt, 3), "back_within_deg": round(back, 4)}
