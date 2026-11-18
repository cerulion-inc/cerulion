# SPDX-License-Identifier: AGPL-3.0-only
"""The self-test (SPEC-R4 A2.1, round six the design rulings, contract C17): the robot measures its own numbers once per
robot and controller and keeps them (rcpx/bounds_self.py SelfKnow). Never a task's or a scene's numbers; never
inside a task or a scored cell.

The gate a session calls before its first move, the stored record's identity, the cold start and the preflight
are in rcpx/bounds_selfstart.py (their names are also reachable from here).

run(ex, intent=None) measures, each step using only what earlier steps gave:
- S1, a still hold of n = ceil(1 / (2 REL^2)) periods: the hidden level and noise of the tip force estimate, of each
  joint's external torque and speed reading, the jitter of the tip, the quantum of each joint reading, two camera
  frames.
- The jaws (rcpx/bounds_selftaps.py jaws): their own numbers from control_hold.measure_jaws where that module is.
  After it the self-test sets the jaws itself: closed for the sweep and the taps, at their open rest for the ramps.
- S2, a sweep, one joint at a time, the others holding, the jaws closed. Range and speed grow together: the first
  pass covers the arc over which the farthest point the joint carries travels the hand hull's smallest width, at the
  speed the model's own braking stops inside it; each later pass DOUBLES both and leaves room for its stop from the
  run-on MEASURED in the pass before. The owner's word (intent {"clear": look id}) is recorded and gates no speed
: the contact stop is armed either way. A pass that does not fit the joint's range or the checked arc is
  skipped; a contact ends the range there and the thing is stored. The passes end at the maker's rating; a joint
  with none ends when its plateau falls short of the command or stops gaining. Every pass ends in a commanded stop
  (the goal snaps to the present reading): its run-on, stop time and settling time are read there; the plateau
  gives the tracked speed, the lag and the cross-track error; ramps and plateaus give the hidden levels by speed and
  acceleration; the rest poses give e_tau. A hidden level is kept only from a pass that ended at its goal.
- Braking and acceleration (bounds_selftaps.ramps, contract C6), the jaws at their open rest: per joint, rest to
  rest passes that ramp and brake at GIVE times the model's acceleration, then at the model's whole; the largest
  that ended inside the still band is stored as brake_j and accel_j. No single braking number is stored.
- S3, taps (bounds_selftaps.taps, contract C5): straight descents onto the stated support at speeds that double
  from the tell speed, each measured by rcpx.bounds_stops.measure. They give the detection time, and, on a body
  with a force signal at the physics rate, the stop slope. The entry "taps" also prices the way down to them.

A contact in a pass is told from the hidden level rising with speed by its impulse: at a speed not yet run a stop
needs REL of the impulse that stopping the moving mass at that speed would give, inside the detection window; at a
speed already run it is the stored hidden level plus TWO_READ sigma on two readings. SI inside. Nothing here names a
robot; every number comes from the model, the backend or the readings.
"""

import importlib
import math

import numpy as np

from rcpx import backend as backend_facts
from rcpx.bounds_ratios import DOUBLE, GIVE, JOINT, QUANT, REL, TWO_READ
from rcpx.bounds_self import VERSION, SelfKnow
from rcpx.bounds_selfstart import (BODY, HINT, LACKS, _install, apply_to_model, before_move, body_bytes, cold,  # noqa: F401
                                   e_tau_of, expected_s, identity_of, know_of, preflight)


# ------------------------------------------------------------------ readings
class _Rec:
    """The readings of a stretch of control periods."""

    def __init__(self):
        self.t, self.q, self.qd, self.ext, self.f, self.tip, self.cmd, self.fast = [], [], [], [], [], [], [], []
        self.tau, self.levels = [], []   # levels: hidden levels this stretch showed, kept only if it ends at its goal

    def take(self, ex, q_cmd):
        s = ex._period(q_cmd)
        self.t.append(s.t)
        self.q.append(s.q.copy())
        self.qd.append(s.qd.copy())
        self.ext.append(np.abs(np.asarray(s.ext, dtype=float)))
        self.tau.append(np.abs(np.asarray(s.tau, dtype=float)))
        self.f.append(max(float(s.Fmag), float(getattr(s, "Fend", None) or 0.0)))
        self.tip.append(s.tip.copy())
        self.cmd.append(np.asarray(q_cmd, dtype=float).copy())
        fast = backend_facts.fast_of(ex.backend.read())
        self.fast.append(None if fast is None else np.asarray(fast["qd"], dtype=float))
        return s

    def speed(self, j, lo, hi):
        """Mean absolute speed of joint j over periods lo to hi: the fast samples where the backend gives them."""
        rows = [self.fast[k][:, j] if self.fast[k] is not None else np.array([self.qd[k][j]]) for k in range(lo, hi)]
        return float(np.mean(np.abs(np.concatenate(rows)))) if rows else 0.0


def _spread(x):
    x = np.asarray(x, dtype=float)
    return (float(np.max(np.abs(x))), float(np.std(x))) if x.size else (0.0, 0.0)


def _still_hold(ex, know, at):
    """S1: hold the present pose for n periods; hidden levels, noises, jitter and quantum of what the robot reads."""
    n = int(math.ceil(1 / (2 * REL ** 2)))
    q_hold = ex._sense().q.copy()
    rec = _Rec()
    for _ in range(n):
        rec.take(ex, q_hold)
    f_hid, f_sig = _spread(rec.f)
    ext, qd, q = np.array(rec.ext), np.array(rec.qd), np.array(rec.q)
    know.add_row("f_hidden", [0.0, 0.0], f_hid, at=at)
    know.add_row("sigma_f", [0.0, 0.0], f_sig, at=at)
    know.add_row("tau_hidden", [0.0], ext.max(axis=0), at=at)
    know.add_row("sigma_tau", [0.0], ext.std(axis=0), at=at)
    know.set("qd_hidden", np.abs(qd).max(axis=0), "self_test", at)
    know.set("sigma_qd", qd.std(axis=0), "self_test", at)
    tip = np.array(rec.tip)
    know.set("jitter", float(np.max(np.linalg.norm(tip - tip.mean(axis=0), axis=1))), "self_test", at)
    know.set("q_spread", np.abs(q - q.mean(axis=0)).max(axis=0), "self_test", at)
    steps = np.abs(np.diff(q, axis=0))
    know.set("quantum", [float(c[c > 0].min()) if (c > 0).any() else 0.0 for c in steps.T], "self_test", at)
    try:   # camera: the spread of two frames of a still scene
        a, b = (np.asarray(ex.backend.render("wrist"), dtype=float) for _ in range(QUANT))
        know.set("camera", {"noise": float(np.mean(np.abs(a - b))), "latency_s": None}, "self_test", at)
    except Exception as exc:
        know.set("camera", {"noise": None, "latency_s": None, "why": f"{type(exc).__name__}"}, "self_test", at)
    know.set("s1", True, "self_test", at)
    s = ex._sense()   # what the torque model's own error hides here: the first reading of e_tau (the passes add theirs)
    err = (np.asarray(s.tau, dtype=float) - ex.model.gravity_torque(s.q)) / ex.model.torque_limit
    ex._st_e_tau = float(np.max(np.abs(err)))
    return rec, n


# ------------------------------------------------------------------ S2: the sweep
def _joint_facts(ex, q0, j):
    """(radius m, reflected mass kg, servo stiffness N/m) of the farthest point joint j carries at q0: its distance
    from the joint's axis, and the joint's own inertia and gain carried to that point."""
    model = ex.model
    anchor, axis = model.joint_frames(q0)[j]
    caps = model.collision_capsules(q0)[1:]
    pts = [p for name, p0, p1, _ in caps if model._arm_key(model.mj.body(name).id) >= j for p in (p0, p1)]
    pts.append(model.fk_tip(q0)[0])
    radius = max(float(np.linalg.norm((p - anchor) - axis * float((p - anchor) @ axis))) for p in pts)
    from rcpx.model_solve import joint_inertia

    radius = radius if radius > 0 else float(model.rot_weight_m)
    return radius, float(joint_inertia(model, q0)[j]) / radius ** 2, float(ex.kp[j]) / radius ** 2


def _arc_ok(ex, q0, j, sign, theta, omega=0.0):
    """Whether the arc of joint j from q0 by sign theta stays inside the joint's range less its stopping angle at
    the pass's speed omega, and passes the executor's own path check (the floor, the mount, the arm itself),
    sampled no coarser than the hand."""
    model = ex.model
    end = q0[j] + sign * theta
    need = model.limit_stop_rad(np.eye(model.n)[j] * omega)[j]
    if not (model.q_lo[j] + need <= end <= model.q_hi[j] - need):
        return False
    n = max(QUANT, int(math.ceil(theta / max(ex._st_first[j] / QUANT, np.finfo(float).eps))))
    path = [q0 + sign * theta * k / n * np.eye(model.n)[j] for k in range(n + 1)]
    try:
        return ex._path_check(path, False) is None
    except TypeError:
        return all(model.plane_clearance(q) > 0 and not model.self_collisions(q) for q in path)


def side_of(ex, q0, j, theta):
    """The way joint j sweeps from q0: the side with more of its range, of those whose first arc is clear; None
    when neither is."""
    model = ex.model
    side = [s for s in (1.0, -1.0) if _arc_ok(ex, q0, j, s, theta)]
    if not side:
        return None
    return max(side, key=lambda s: (model.q_hi[j] - q0[j]) if s > 0 else (q0[j] - model.q_lo[j]))


def _stopped(ex, rec, know, v, mass, lo):
    """The explore guard on the newest readings: "cap" when a joint sits at its torque limit through the detection
    window; "contact" when the force estimate (or a joint's external torque) passes its threshold through the window
    at a speed already run, or, at a speed not yet run (v: the speed the pass may have here, m/s), when the window
    holds REL of the impulse that stopping the moving mass at that speed would give."""
    n = int(know.get("n_det")[0])
    if len(rec.f) - lo < n:
        return None
    if np.all(np.asarray(rec.tau[-n:]) >= ex.torque_limit, axis=0).any():
        return "cap"
    last = np.asarray(rec.f[-n:])
    # a contact carries momentum: the window holds REL of the impulse that stopping the moving mass would give.
    # A reading over its threshold with less than that is the hidden level of this speed showing itself (A1.1,
    # the relative form on a reading with no noise): it is stored, upward, and the pass goes on.
    carries = float(np.sum(last)) * ex.dt >= REL * mass * v
    hides = float(getattr(ex, "_st_e_tau", 0.0)) * np.asarray(ex.torque_limit, dtype=float)   # the model's own error
    if any(r["x"][0] >= v for r in know.rows("f_hidden")):
        shown = [y for t, x, y in rec.levels if t == "f_hidden"]          # what this pass itself showed so far
        thr = max([know.f_hidden(v)[0]] + shown) + TWO_READ * (know.sigma_f(v)[0] or 0.0)
        hidden = np.maximum(np.asarray(know.tau_hidden(v)[0]), hides)
        for t, x, y in rec.levels:
            hidden = np.maximum(hidden, np.asarray(y)) if t == "tau_hidden" else hidden
        tau = hidden + JOINT * np.asarray(know.sigma_tau(v)[0])
        ext = np.asarray(rec.ext[-n:])
        over_f, over_t = bool(np.all(last > thr)), bool(np.all(ext > tau, axis=0).any())
        if not (over_f or over_t):
            return None
        if carries:
            return "contact"
        if over_f:
            rec.levels.append(("f_hidden", [v, 0.0], float(last.max())))
        if over_t:
            rec.levels.append(("tau_hidden", [v], np.maximum(np.asarray(know.tau_hidden(v)[0]), ext.max(axis=0)).tolist()))
        return None
    if np.all(last > know.f_hidden()[0]) and carries:
        return "contact"
    return None


def _run_pass(ex, know, q0, j, sign, theta, omega, alpha, reserve, facts, at):
    """One pass of joint j: ramp at alpha to omega, hold it, command a stop where theta less reserve is reached (the
    goal snaps to the present reading), wait until still. Returns the pass record."""
    radius, mass, _ = facts
    dt, n_j = ex.dt, ex.model.n
    unit = np.eye(n_j)[j] * sign
    rec, phi, w, k_plateau = _Rec(), 0.0, 0.0, None
    stop, lo = None, 0
    while phi < theta - reserve:
        top = omega
        w = min(w + alpha * dt, top)
        if k_plateau is None and w >= omega:
            k_plateau = len(rec.t)
        phi = min(phi + w * dt, theta - reserve)
        rec.take(ex, q0 + unit * phi)
        stop = _stopped(ex, rec, know, radius * top, mass, lo)
        if stop:
            break
    k_stop = len(rec.t)   # the commanded stop, as the executor's own stop does it: the goal follows the reading
    s = ex._sense()
    q_snap = s.q.copy()
    w_at = abs(float(rec.qd[-1][j])) if rec.qd else 0.0
    still_under, run_on, quiet, floor = None, 0.0, 0, know.get("q_spread")[0][j]
    for _ in range(max(k_stop, QUANT) * int(1 / REL)):   # the wait returns: no longer than the pass by 1 / REL
        before = s.q[j]
        s = rec.take(ex, s.q)
        run_on = max(run_on, sign * float(s.q[j] - q_snap[j]))
        still_under = max(floor, QUANT * know.get("quantum")[0][j], REL ** 3 * run_on)
        quiet = quiet + 1 if abs(float(s.q[j] - before)) <= still_under else 0
        if quiet >= QUANT:
            break
    if stop is None and float(np.sum(rec.f[k_stop:])) * dt >= REL * mass * radius * w_at > 0.0:
        stop = "contact"   # the run-on of the commanded stop met something: its force carried the arm's momentum
    out = {"joint": j, "sign": sign, "theta": theta, "omega": omega, "alpha": alpha, "reserve": reserve,
           "stopped": stop or "goal", "periods": len(rec.t), "reached": float(abs(s.q[j] - q0[j])),
           "radius": radius, "run_on": run_on, "settle_s": (len(rec.t) - k_stop) * dt, "w_at_stop": w_at,
           "f_stop": float(max(rec.f[k_stop:] or [0.0])), "rest_q": s.q.copy(), "rest_tau": np.asarray(s.tau).copy(),
           "plateau": None, "rec": rec, "k_stop": k_stop, "f_peak": float(max(rec.f or [0.0]))}
    lags = [abs(float(c[j] - q[j])) for c, q in zip(rec.cmd[:k_stop], rec.q[:k_stop])]
    settled = (k_plateau is not None and k_stop - k_plateau >= QUANT * QUANT
               and abs(lags[-1] - lags[-QUANT]) <= REL * lags[-1])
    if stop is None and settled:
        half = k_plateau + int((k_stop - k_plateau) * GIVE)   # the plateau's second half: the lag has settled
        ideal = [ex.model.fk_tip(q0 + (q[j] - q0[j]) * np.eye(n_j)[j])[0] for q in rec.q[half:k_stop]]
        out["plateau"] = {
            "speed": rec.speed(j, half, k_stop),
            "lag": float(abs(rec.cmd[k_stop - 1][j] - rec.q[k_stop - 1][j])),
            "cross": float(max(np.linalg.norm(a - b) for a, b in zip(rec.tip[half:k_stop], ideal))),
            "f": _spread(rec.f[half:k_stop]), "f_ramp": _spread(rec.f[:k_plateau]),
            "ext": (np.array(rec.ext[:k_stop]).max(axis=0), np.array(rec.ext[half:k_stop]).std(axis=0))}
    return out


def _go_back(ex, q_start, j, omega, alpha):
    """Back to q_start through the arc just swept: ramp, hold, ramp down to rest there. Returns (readings, the
    plateau's tracked speed and lag angle, or None when the way back held no plateau)."""
    q = ex._sense().q.copy()
    rec, dt = _Rec(), ex.dt
    dist = abs(float(q_start[j] - q[j]))
    sign = math.copysign(1.0, float(q_start[j] - q[j])) if dist > 0 else 0.0
    phi, w, flat = 0.0, 0.0, []
    while phi < dist:
        w = min(w + alpha * dt, omega, math.sqrt(2 * alpha * (dist - phi)))
        phi = min(phi + max(w, alpha * dt) * dt, dist)
        cmd = q.copy()
        cmd[j] = q[j] + sign * phi
        rec.take(ex, cmd)
        if w >= omega:
            flat.append(len(rec.t) - 1)
    for _ in range(int(math.ceil(1 / (2 * REL ** 2)))):
        rec.take(ex, q_start if dist > 0 else q)
    plateau = None
    if len(flat) >= QUANT * QUANT:
        half = flat[int(len(flat) * GIVE):]
        plateau = {"speed": rec.speed(j, half[0], half[-1] + 1),
                   "lag": float(abs(rec.cmd[half[-1]][j] - rec.q[half[-1]][j]))}
    return rec, plateau


def _note_pass(know, model, p, at):
    """Store what one clean pass showed: the plateau's tracked speed, lag and cross-track error, the hidden levels
    by speed and acceleration, the stop's run-on, stop time, deceleration and settling time, and e_tau at rest."""
    pl, r, j = p["plateau"], p["radius"], p["joint"]
    row = {"joint": j, "omega": p["omega"], "periods": p["periods"], "stopped": p["stopped"]}
    if p["stopped"] == "goal":   # no contact is proven in this pass: what it showed of the hidden levels is kept
        for table, x, y in p["rec"].levels:
            know.add_row(table, x, y, at=at)
    if p["stopped"] == "goal" and p["run_on"] > 0 and p["w_at_stop"] > 0:   # the commanded stop
        brake = p["w_at_stop"] ** 2 / (2 * p["run_on"])
        know.add_row("d_stop", [r * p["w_at_stop"]], r * p["run_on"], at=at)
        know.add_row("f_hidden", [0.0, r * brake], p["f_stop"], at=at)   # the settling stretch after a stop
        row.update(d_stop_m=r * p["run_on"], stop_s=p["run_on"] / p["w_at_stop"], brake=brake, settle_s=p["settle_s"])
        err = (p["rest_tau"] - model.gravity_torque(p["rest_q"])) / model.torque_limit
        row["e_tau"] = float(np.max(np.abs(err)))
    if pl is None:
        return row
    v = r * pl["speed"]
    know.add_row("f_hidden", [v, 0.0], pl["f"][0], at=at)
    know.add_row("sigma_f", [v, 0.0], pl["f"][1], at=at)
    know.add_row("f_hidden", [v, r * p["alpha"]], max(pl["f_ramp"][0], pl["f"][0]), at=at)
    know.add_row("sigma_f", [v, r * p["alpha"]], pl["f_ramp"][1], at=at)
    know.add_row("tau_hidden", [v], pl["ext"][0], at=at)
    know.add_row("sigma_tau", [v], pl["ext"][1], at=at)
    know.add_row("lag", [v], r * pl["lag"], at=at)
    know.add_row("e_cross", [v], pl["cross"], at=at)
    row.update(v=v, plateau=pl["speed"], lag_s=pl["lag"] / pl["speed"], cross_m=pl["cross"])
    return row


def _sweep_joint(ex, know, q0, j, word, at, log):
    """S2 for one joint (the others holding at q0). Returns its summary."""
    model = ex.model
    facts = _joint_facts(ex, q0, j)
    radius, mass, stiff = facts
    rated = model.speed_limit[j]
    acc = GIVE * float(model.motion["acc"][j])   # the sweep's passes ramp and brake with half the model's own
    why = "the pass's own speed, the contact stop armed" + (" (the owner's word is recorded)" if word else "")
    theta = ex._st_first[j]
    sign = side_of(ex, q0, j, theta)
    while sign is None and theta > ex._st_first[j] * REL ** 2:   # a first arc that does not fit: a shorter one
        theta = theta * GIVE
        sign = side_of(ex, q0, j, theta)
    out = {"joint": model.joint_names[j], "passes": [], "space": why, "ended": None, "qd_track": None,
           "rated": rated, "thing": None, "sign": sign, "arc": 0.0}
    if sign is None or not math.isfinite(mass):
        out["ended"] = "skipped: the first arc does not fit the joint's range or the checked space"
        return out
    top = rated if rated is not None else float(model.motion["qd"][j])
    reserve = None   # the room a pass leaves for its stop: measured in the pass before
    omega, best, last_run_on, fell_short = None, [], None, False
    hold = float(model.motion["lag_s"][j]) / REL   # a plateau long enough for the lag to settle to REL of REL

    def fitting(arc, room=None):
        """The largest speed whose ramp at acc, plateau of `hold` and room for its stop fit inside the arc; room
        None: the model's own braking at acc."""
        if room is None:
            return (acc / 2) * (math.sqrt(hold ** 2 + 4 * arc / acc) - hold)
        return acc * (math.sqrt(hold ** 2 + 2 * max(arc - room, 0.0) / acc) - hold)

    while True:
        if omega is None:   # the first pass: the speed the model's own braking stops inside the arc
            omega = min(fitting(theta), top)
            reserve = omega ** 2 / (2 * acc)
        else:
            want = min(DOUBLE * omega, top)
            theta_next = DOUBLE * theta
            while theta_next > theta and not _arc_ok(ex, q0, j, sign, theta_next, want):
                theta_next = theta + (theta_next - theta) * GIVE
                if theta_next - theta < ex._st_first[j] * REL:
                    theta_next = theta
            reserve = DOUBLE ** 2 * last_run_on if last_run_on else want ** 2 / (2 * acc)
            fits = fitting(theta_next, reserve)
            if (want <= omega and theta_next <= theta) or fits <= omega * (1 + REL) and theta_next <= theta:
                out["ended"] = out["ended"] or ("the rating" if rated is not None and omega >= rated * (1 - REL)
                                                 else "the joint's own range or the checked space: no wider arc fits")
                break
            omega, theta = min(want, max(fits, omega)), theta_next
        speed = min(omega, top)
        p = _run_pass(ex, know, q0, j, sign, theta, speed, acc, min(reserve, theta * GIVE), facts, at)
        row = _note_pass(know, model, p, at)
        ex._st_e_tau = max(float(getattr(ex, "_st_e_tau", 0.0)), float(row.get("e_tau") or 0.0))
        out["passes"].append(row)
        log(f"joint {model.joint_names[j]} pass {len(out['passes'])}: {row}")
        back, flat = _go_back(ex, q0, j, speed, acc)
        if p["stopped"] == "contact":   # a contact ends the range there; the thing is stored
            out["ended"] = f"a contact at {math.degrees(p['reached']):.1f} deg of the arc"
            out["thing"] = _store_thing(ex, p)
            break
        if p["stopped"] != "goal":      # the joint sat at its torque limit: no faster pass, and nothing was met
            out["ended"] = f"the joint's torque limit, at {math.degrees(p['reached']):.1f} deg of the arc"
            break
        know.note_free_leg(radius * speed, 0.0, _spread(back.f)[0], _spread(back.f)[1], swept=True, ended="goal", at=at)
        out["arc"], last_run_on = float(p["reached"]), (p["run_on"] or None)
        seen = p["plateau"] or flat
        if p["plateau"] is None and flat is not None:
            know.add_row("lag", [radius * flat["speed"]], radius * flat["lag"], at=at)
            row.update(v=radius * flat["speed"], plateau=flat["speed"], lag_s=flat["lag"] / flat["speed"])
        if seen is not None:
            best.append(seen["speed"])
            short = speed - seen["speed"]
            limit = max(TWO_READ * know.get("sigma_qd")[0][j], REL * speed)
            fell_short = fell_short or short > limit
            if rated is None and (short > limit or (len(best) > 1 and best[-1] < best[-QUANT] * (1 + REL))):
                out["ended"] = "the plateau fell short of the command or stopped gaining"
                break
        if rated is not None and speed >= rated:
            out["ended"] = "the rating"
            break
    # the tracked speed is stored only where the sweep says something of tracking: the joint has no rating, or a
    # plateau fell short of its command by more than the accepted error; a rated joint whose passes reached the
    # rating, or ended for want of room or on a contact, keeps its rating (fastest_rad_s says how far it was checked)
    if best and (rated is None or fell_short):
        out["qd_track"] = max(best)
    out["fastest_rad_s"] = max(best) if best else None
    return out


def _store_thing(ex, p):
    """A contact that ended a pass is a thing in the way: stored in the world where the executor can."""
    try:
        rec = p["rec"]
        tip = rec.tip[p["k_stop"] - 1]
        e = ex.world.add("contact", "a thing met in the robot's own sweep",
                         point_mm=[round(float(x) * 1000.0, 1) for x in tip], source="self-test sweep",
                         data={"force_n": round(p["f_peak"], 2), "joint": ex.model.joint_names[p["joint"]]})
        return {"tip_m": [float(x) for x in tip], "force_n": p["f_peak"], "stored": True, "id": e.get("id")}
    except Exception as exc:
        return {"tip_m": [float(x) for x in p["rec"].tip[-1]], "force_n": p["f_peak"], "stored": False,
                "why": f"{type(exc).__name__}: {exc}"[:120]}


# ------------------------------------------------------------------ the whole test
def run(ex, intent=None, directory=None, joints=None, save=True, log=None):
    """Measure the robot's own numbers (S1, the jaws, S2, braking and acceleration, S3 the taps) and keep them.
    intent {"clear": look id}: the owner's word after a look; it is recorded and gates no speed. joints: a subset
    of joint indices (a test: the sweep alone, no ramps and no taps). Returns {said, took_s, expected_s, key, path,
    joints, jaws, ramps, taps, ...}."""
    from rcpx import bounds_selftaps as more

    log = log or (lambda text: None)
    model, be = ex.model, ex.backend
    know, _ = know_of(ex, directory)
    know = SelfKnow(know.key, know.directory, know.ident)   # measured afresh
    t0, at = ex.sim_time(), f"self-test at t {ex.sim_time():.2f} s"
    dt = backend_facts.period_of(be)
    look = (intent or {}).get("clear")
    word = bool(look) and (look is True or look in getattr(ex, "_looks", {}))
    fast = backend_facts.fast_of(be.read())
    f_tool = model.f_tool()
    know.set("dt", dt, "model", at)
    know.set("n_det", QUANT, "model", at)
    know.set("t_sample", dt if fast is None else float(fast["dt_s"]), "model", at)
    know.set("fast", None if fast is None else {"dt_s": float(fast["dt_s"]), "signal": "joint speeds" + (
        " and the wrist force" if fast.get("wrist_n") is not None else "")}, "model", at)
    know.set("tau_max", model.torque_limit, "rating", at)
    know.set("qd_rated", model.speed_limit, "rating", at)
    know.set("f_tool", f_tool[0], "rating" if f_tool[0] is not None else "model", at)   # None: no rating is stated
    know.set("f_hand", model.f_hand()[0], "model", at)
    _, n_hold = _still_hold(ex, know, at)
    q0 = ex._sense().q.copy()
    hull = model.hulls().get(model.spec["tip"]["body"]) or next(iter(model.hulls().values()))
    thin = np.linalg.eigh(np.cov(hull["points_m"].T))[1][:, 0]   # the hull's thinnest direction
    width = float(np.ptp(hull["points_m"] @ thin))
    ex._st_first = [width / _joint_facts(ex, q0, j)[0] for j in range(model.n)]
    which = list(range(model.n)) if joints is None else [int(j) for j in joints]
    expected = expected_s(ex, which)
    said_word = f"the owner's word after look {look} is recorded" if word else "no word was given: none is needed"
    ex._journal("self_test", expected_s=round(float(expected), 1), word=said_word, key=know.key)
    log(f"self-test: expected about {expected:.0f} s of robot time for the sweep; {said_word}")
    whole = joints is None
    grip = ex._sense().gripper
    opening, jaw_cmd = (None if grip is None else float(grip["open_frac"])), ex._gripper_cmd   # as the session stood
    jaws = more.jaws(ex, know, at) if whole else None
    if whole:
        more._jaws_to(ex, q0, "close")   # the sweep runs with the jaws closed, whatever measuring them left
    rows = [_sweep_joint(ex, know, q0, j, word, at, log) for j in which]
    track = [None] * model.n
    passes = [p for r in rows for p in r["passes"]]
    for r, j in zip(rows, which):
        track[j] = r["qd_track"]
    know.set("qd_track", track, "self_test", at)
    know.set("qd_checked", [r.get("fastest_rad_s") for r in rows] if whole else None, "self_test", at)
    per = {name: [None] * model.n for name in ("lag_s", "stop_s")}
    for p in passes:
        for name in per:
            if p.get(name) is not None:
                per[name][p["joint"]] = max(per[name][p["joint"]] or 0.0, p[name])
    for name, values in per.items():
        know.set(name, values, "self_test", at)
    settle = [p["settle_s"] for p in passes if p.get("settle_s") is not None]
    know.set("t_settle", max(settle) if settle else None, "self_test", at)
    e_tau = [p["e_tau"] for p in passes if p.get("e_tau") is not None] + [float(getattr(ex, "_st_e_tau", 0.0))]
    know.set("e_tau", e_tau_of(*e_tau), "self_test", at)
    _install(ex, know)   # the bounds read the fresh numbers from here on: the ramps and the taps are planned on them
    ramps = more.ramps(ex, know, q0, rows, which, at, log) if whole else None
    taps = importlib.import_module("rcpx.bounds_selftaps_more").run(ex, know, q0, at, log) if whole else None
    if whole and opening is not None:   # the jaws back where the session had them, and the command it had
        more._jaws_to(ex, q0, opening)
        ex._gripper_cmd = jaw_cmd
    took = ex.sim_time() - t0
    know.set("took_s", took, "self_test", at)
    free = ex.cfg.get("selftest_taps") == "free"
    climb = "to the free speed on the owner's statement" if free else "to what the arm can push"
    know.measured_by = (f"rcpx self-test, record version {VERSION}: a still hold, the jaws, the sweep of {len(which)} "
                        f"joints, ramps, taps onto the stated support ({climb}); "
                        f"{took:.1f} s of the robot's own time, {int(round(took / dt))} control periods")
    beside = None
    if save:
        know.save()
        beside = know.beside
    _install(ex, know)
    s3 = "not run (a test of some joints)" if taps is None else taps["said"]
    said = (f"The robot measured itself in {took:.1f} s of its own time ({said_word}); the numbers are kept at "
            f"{know.path if save else 'no file (save off)'}"
            + (f" (a NEW record: the one that stood, {beside}, is kept beside it and is no longer the one read)" if beside else "")
            + f". {s3}.")
    out = {"said": said, "took_s": took, "expected_s": float(expected), "key": know.key,
           "path": know.path if save else None, "beside": beside, "version": VERSION, "measured_by": know.measured_by,
           "word": bool(word), "joints": rows, "jaws": jaws, "ramps": ramps,
           "taps": taps, "s3": s3, "f_tool_n": f_tool[0], "f_tool_how": f_tool[1], "periods": int(round(took / dt))}
    ex._journal("self_test", took_s=round(took, 2), key=know.key, word=bool(word), stored=bool(save),
                ended=[r["ended"] for r in rows])
    return out


def d_stop_at(know, model, q, u, v):
    """The tip's run-on (m) after a commanded stop at tip speed v (m/s) along unit direction u at pose q: each joint
    runs on for its own measured stop time at the speed the line asks of it. None before the sweep."""
    stop_s = know.get("stop_s")[0]
    if stop_s is None:
        return None
    # a joint whose sweep gave no stop keeps the model's own time constant (kd / kp)
    stop_s = [model.motion["lag_s"][i] if x is None else x for i, x in enumerate(stop_s)]
    J = model.jac(q)[:3]
    qd = np.linalg.pinv(J) @ (np.asarray(u, dtype=float) / np.linalg.norm(u) * v)
    return float(np.linalg.norm(J @ (np.asarray(stop_s) * qd)))
