# SPDX-License-Identifier: AGPL-3.0-only
"""Three parts of the self-test (rcpx/bounds_selftest.py run), round six: the jaws' own numbers, braking and
acceleration per joint (contract C6), and the taps (contract C5). Never inside a task or a scored cell.

- jaws(ex, know, at): the entry "jaw" from control_hold.measure_jaws (contract C7) where that module is; else one
  timed REAL opening and closing (the jaws are closed first, so the opening moves), and the entry says that these
  are end stop numbers. Whatever measuring leaves the jaws at, the self-test sets them itself after it: closed for
  the sweep and for the taps (the hand as the body's hulls hold it), at their open rest for the ramp passes.
- ramps(ex, know, q0, rows, which, at, log): per joint, inside the arc its sweep ran, rest to rest passes out and
  back that ramp up and brake at one acceleration: GIVE times the model's, then the model's whole (DOUBLE); a joint
  that fails at GIVE times the model's is tried lower, so that a number it showed stands. A pass ended inside the
  still band when the joint never passed its rest place by more than the still band (its reading's spread at rest,
  QUANT quanta) plus the tracking lag its sweep stored at that speed (with its accepted error REL). brake_j is the
  largest acceleration whose passes ended so, under the least that did not. accel_j is the largest whose passes
  reached their commanded end at all: a ramp up leaves nothing at the end of a pass (the lag settles in the
  cruise), and where the joint's torque sat at its limit while ramping the row says so (sat_up). The passes run
  with the jaws at their open rest (the sweep ran them closed); a joint whose open hand met something runs them
  closed, on the arc the sweep checked with that hand, then on half of it. A pass that ended at its goal gives the
  hidden levels at its speed and acceleration; a pass that met something gives none. The rows are kept (know
  "ramps"): what ran, at which acceleration and speed, how far past its rest place, with which jaws.
- taps(ex, know, q0, at, log): straight descents onto the stated support from where the hand stands, the jaws
  closed, driven here, stopped by the executor's own guard and measured by rcpx.bounds_stops.measure, at speeds
  that double from the tell speed. Every body that resolves a peak climbs: one with a force sensor read at the
  fast rate by that signal, one without by the arm's own force at each fast sample (_model_force: its fast joint
  speeds through its own model and servo law). The climb ends where the peak reaches what the arm can push there:
  by the body's model, and by the slope the taps made so far where they read over the model (such a body stops
  short of the model's top); with cfg selftest_taps "free" the free speed. A body that resolves no peak (no force
  signal and no fast joint speeds, or no stated servo law) gets three taps (the detection time only). No tap is
  made when fewer than QUANT + 1 speeds fit by the body's model (fewer cannot show that the rest height repeats);
  that is every arm whose tell speed is at or over the top (it cannot tell the support from a thing its own
  resolution over it at a speed it may tap at). A tap is stored only when the tool's lowest point came to rest
  at the stated support within the support's error and the taps repeat within the tip's resolution (both with
  the elastic give of the force the hand rests with); else the entry "taps" says what height was met and no tap
  is used. A body whose servo ripples inside each period (it follows a stepped command closely) is not resolved:
  the stop's own check of the speed of arrival calls its speed changing.
  Under a stated limit (a tool rating, a site force; the design rulings, b) the body's model may not stand as the bound: it
  is no upper bound of a true peak. A body that resolves no peak taps at the tell speed only, and only when the
  model puts that tap under the limit; the entry says the limit is not shown kept. A body that resolves a peak
  climbs only once a tap is resolved and only while the taps it has made put the next one under the limit, and
  the entry says what the taps' own peaks were against it.
  The way down to the first tap is a transit toward a height known by statement only: the entry carries its price
  (rcpx.bounds_strike.price of that line) and what ran (its speed, the largest reading, how it ended).
- _model_force(ex, before, s, target, frac, u, free): the arm's own force at each fast sample of one period, for
  a body with no force sensor: what the servo's law gave at that sample's own pose and speed, less what the arm
  alone needs for the acceleration its fast joint speeds show, less what the same sum left in a free period of
  the same motion, read as a force at the tip; and beside it what the joints' dry friction can hide at a stop
  (it turns from against the travel to against the way back: twice its size, added to the peak: an upper bound
  of the push). No simulator truth: the joint speeds, the servo
  law the backend states and the body's own description.
"""

import math

import numpy as np

from rcpx import bounds_laws as laws
from rcpx import bounds_stops as stops
from rcpx.bounds_ratios import BAND, DOUBLE, GIVE, QUANT, REL, TWO_READ
from rcpx.bounds_selftest import _Rec, _joint_facts, _stopped
from rcpx.bounds_selftaps_parts import _cap, _model_force, _servo, _top, _way_price  # noqa: F401

MM = stops.MM
KEEP = QUANT * QUANT + QUANT   # readings kept before a tap's contact: enough for the reading before and its trend


# ------------------------------------------------------------------ the jaws
def jaws(ex, know, at):
    """Measure and store the jaws' own numbers (know "jaw"); None for a hand with none."""
    g = ex.model.spec.get("gripper")
    s = ex._sense()
    if not g or s.gripper is None:
        return None
    try:   # r6 shim: package HOLD's module; absent until it is merged
        from rcpx import control_hold
        measure = getattr(control_hold, "measure_jaws", None)
    except ImportError:
        measure = None
    entry = dict(measure(ex)) if measure is not None else _timed(ex, s)
    know.set("jaw", entry, "self_test", at)
    return entry


def _timed(ex, s):
    """One timed real opening and closing in free air, in shares of the span: an end stop measurement."""
    q_hold, dt, out, before = s.q.copy(), ex.dt, {}, ex._gripper_cmd
    for name, cmd in (("first", "close"), ("open", "open"), ("close", "close")):
        ex._gripper_cmd = cmd
        fracs, quiet = [float(s.gripper["open_frac"])], 0
        while quiet < QUANT and len(fracs) < int(1 / REL) ** 3:
            s = ex._period(q_hold)
            fracs.append(float(s.gripper["open_frac"]))
            quiet = quiet + 1 if fracs[-1] == fracs[-2] or abs(fracs[-1] - fracs[-2]) < REL ** 3 * abs(
                fracs[-1] - fracs[0]) else 0
        steps = np.abs(np.diff(fracs))
        out[name] = {"time_s": (len(fracs) - 1 - QUANT) * dt, "speed": float(steps.max() / dt), "rest": fracs[-1],
                     "still": float(steps[-QUANT:].max())}
    ex._gripper_cmd = before   # None keeps the last goal: closed, as a session starts
    g = ex.model.spec["gripper"]
    span = float(np.max(np.abs(np.asarray(g["open"], dtype=float) - np.asarray(g["closed"], dtype=float))))
    return {"speed": out["close"]["speed"] * span, "closed_gap": out["close"]["rest"] * span,
            "unit": "the gripper joints' own unit per s",       # speed and closed_gap, as round four's readers take them
            "speed_open": out["open"]["speed"], "open_s": out["open"]["time_s"], "close_s": out["close"]["time_s"],
            "open_rest": out["open"]["rest"], "still_band": max(out["open"]["still"], out["close"]["still"]),
            "shares": "speed_open (per s), open_rest and still_band are shares of the jaws' span",
            "how": "one timed real opening and closing onto the jaws' own end stops (control_hold.measure_jaws is "
                   "absent): not steady rates"}


def _jaws_to(ex, q_hold, cmd):
    """Send the jaws to cmd and hold the arm until they stand still."""
    s = ex._sense()
    if s.gripper is None:
        return
    ex._gripper_cmd = cmd
    last, quiet = float(s.gripper["open_frac"]), 0
    for _ in range(int(1 / REL) ** 3):
        s = ex._period(q_hold)
        now = float(s.gripper["open_frac"])
        quiet = quiet + 1 if abs(now - last) <= REL ** 3 * max(abs(now), REL ** 3) else 0
        last = now
        if quiet >= QUANT:
            return


# ------------------------------------------------------------------ braking and acceleration per joint (C6)
def _ramp_pass(ex, know, q_from, j, goal, w_top, a, facts):
    """One rest to rest pass of joint j from its present command to angle goal: ramp up at a, at most w_top, brake
    at a, then hold the commanded end until still. Returns what the joint's own readings showed."""
    radius, mass, _ = facts
    dt, lim = ex.dt, float(np.asarray(ex.torque_limit, dtype=float)[j])
    dist = abs(float(goal - q_from[j]))
    sign = math.copysign(1.0, float(goal - q_from[j]))
    rec, phi, w, peak, up, k_peak, stop = _Rec(), 0.0, 0.0, 0.0, [], 0, None
    while phi < dist:
        rise = w + a * dt
        w = min(rise, w_top, math.sqrt(2 * a * (dist - phi)))
        up.append(w >= rise or w >= w_top)           # this period ramped up or cruised; else it braked
        if w > peak:
            peak, k_peak = w, len(rec.t)
        phi = min(phi + max(w, a * dt) * dt, dist)
        cmd = q_from.copy()
        cmd[j] = q_from[j] + sign * phi
        rec.take(ex, cmd)
        stop = _stopped(ex, rec, know, radius * w_top, mass, 0)
        if stop == "contact":
            break
        stop = None            # a joint at its torque limit is no contact here: it is what the pass measures
    k_end = len(rec.t)
    end = rec.cmd[-1].copy() if stop is None else ex._sense().q.copy()   # a contact: hold where the joint is
    band = max(float(know.get("q_spread")[0][j]), QUANT * float(know.get("quantum")[0][j]))
    still = max(band, REL ** 3 * peak ** 2 / (laws.TWO * a))   # still: a step under REL^3 of the pass's braking angle
    quiet, s = 0, None
    for _ in range(max(k_end, QUANT) * int(1 / REL)):
        before = rec.q[-1][j]
        s = rec.take(ex, end)
        quiet = quiet + 1 if abs(float(s.q[j] - before)) <= still else 0
        if quiet >= QUANT:
            break
    if stop is None and k_end > 0:
        # the end of the pass met something (with the jaws open the hand is another shape than in the sweep) when
        # the hand rests over the level a guard at rest would run with (the largest free level read so far, or what
        # the torque model's own error hides, plus the noise), or the settling stretch held, over its own rest
        # reading, REL of the momentum the pass ran with
        rest = min(rec.f[-QUANT:])
        hides = float(getattr(ex, "_st_e_tau", 0.0)) * float(ex.model.tip_force_capability(rec.q[-1], np.array([0.0, 0.0, -1.0])))
        level = max(float(know.f_hidden()[0] or 0.0), hides) + TWO_READ * float(know.sigma_f()[0] or 0.0)
        carried = float(np.sum(np.maximum(np.asarray(rec.f[k_end:]) - rest, 0.0))) * dt >= REL * mass * radius * peak > 0.0
        if rest > level or carried:
            stop = "contact"
    tau = np.abs(np.asarray(rec.tau)[:, j])
    at_limit = tau >= lim
    runs = lambda mask: any(all(mask[k:k + QUANT]) for k in range(max(0, len(mask) - QUANT + 1))) if len(mask) >= QUANT else False
    moving = np.asarray(up + [False] * (len(rec.t) - k_end), dtype=bool)
    rest = float(rec.q[-1][j])
    past = max(0.0, max(sign * (float(q[j]) - rest) for q in rec.q[k_end:])) if stop is None else None
    behind = max((abs(float(c[j] - q[j])) for c, q, m in zip(rec.cmd[:k_end], rec.q[:k_end], up) if m), default=0.0)
    return {"a": a, "peak": peak, "stopped": stop or "goal", "past": past, "band": band, "behind": behind,
            "sat_up": bool(runs(list(at_limit & moving))), "sat_down": bool(runs(list(at_limit & ~moving))),
            "periods": len(rec.t), "rec": rec, "k_end": k_end, "end": end}


def ramps(ex, know, q0, rows, which, at, log):
    """Braking and acceleration per joint (see the module text): stores brake_j, accel_j and the rows (know
    "ramps"), returns the rows. It sets the jaws itself and leaves them closed."""
    model = ex.model
    jaw = know.get("jaw")[0]
    opened = None                                        # the jaws at their open rest, never on the stop
    if isinstance(jaw, dict) and jaw.get("open_rest") is not None:
        opened = max(0.0, float(jaw["open_rest"]) - QUANT * float(jaw.get("still_band") or 0.0))
    hand = {"now": None}

    def jaws_to(state):
        if hand["now"] != state:
            _jaws_to(ex, q0, opened if state == "open" else "close")
            hand["now"] = state

    brake, accel, out = [None] * model.n, [None] * model.n, []
    lag_s = know.get("lag_s")[0] or [None] * model.n
    for r, j in zip(rows, which):
        arc, sign = float(r.get("arc") or 0.0), r.get("sign")
        row = {"joint": model.joint_names[j], "passes": [], "brake": None, "accel": None, "why": None}
        out.append(row)
        if not arc > 0 or sign is None or r.get("thing") is not None:
            row["why"] = "no free arc was swept for this joint: the model's values stand"
            continue
        facts = _joint_facts(ex, q0, j)
        w_top = float(r.get("fastest_rad_s") or model.motion["qd"][j])
        lag = float(lag_s[j]) if lag_s[j] is not None else float(model.motion["lag_s"][j])
        whole = float(model.motion["acc"][j])
        # the hands a joint's passes are run with, in turn, each only when the one before met something: the open
        # hand is another shape than the one that swept the arc, so then the closed one on that arc, then on half
        hands = ([("open", arc)] if opened is not None else []) + [("closed", arc), ("closed", arc * GIVE)]
        # up: GIVE times the model's, then the model's whole; a joint that does not end inside its band at GIVE
        # times the model's is tried at GIVE of that, and again, so that a number it showed stands, not the model's
        tries, failed = [GIVE * whole, whole], math.inf     # failed: the least acceleration whose braking did not end inside
        while tries:
            a = tries.pop(0)
            state, arc_now = hands[0]
            jaws_to(state)
            ok_brake = ok_accel = True
            met = False
            reach = min(arc_now, w_top ** 2 / a + w_top * lag / REL)     # room to ramp up, let the lag settle, and brake
            for goal in (q0[j] + sign * reach, q0[j]):                  # out, then back
                q_now = q0.copy()
                q_now[j] = ex._sense().q[j]
                p = _ramp_pass(ex, know, q_now, j, goal, w_top, a, facts)
                allowed = p["band"] + (1 + REL) * lag * p["peak"]     # the still band and the stored tracking lag
                goal_ok = p["stopped"] == "goal"
                inside = goal_ok and p["past"] <= allowed
                row["passes"].append({"a": a, "peak": p["peak"], "stopped": p["stopped"], "past": p["past"],
                                      "behind": p["behind"], "allowed": allowed, "sat_up": p["sat_up"],
                                      "sat_down": p["sat_down"], "periods": p["periods"], "jaws": state})
                if not goal_ok:                                  # it met something: no level, and back to the start
                    q_back = q0.copy()
                    q_back[j] = ex._sense().q[j]
                    _ramp_pass(ex, know, q_back, j, q0[j], w_top * GIVE, GIVE * a, facts)
                    ok_brake = ok_accel = False
                    met = True
                    break
                _levels(know, p, facts, j, at)
                ok_brake, ok_accel = ok_brake and inside, ok_accel and goal_ok
            if met and len(hands) > 1:
                hands, tries = hands[1:], [a] + tries            # the same acceleration again with the next hand
                continue
            if met:
                row["why"], tries = f"a pass ended on {p['stopped']} with the jaws {state}: no faster pass", []
            failed = failed if ok_brake else min(failed, a)
            brake[j] = a if ok_brake and failed > a > (brake[j] or 0.0) else brake[j]
            accel[j] = a if ok_accel and a > (accel[j] or 0.0) else accel[j]
            if a < whole and not (ok_brake and ok_accel) and row["why"] is None:
                lower = a * GIVE >= whole * GIVE ** 3 and (brake[j] is None or accel[j] is None)
                tries = ([a * GIVE] if lower else []) + [t for t in tries if ok_brake or ok_accel]
        row["brake"], row["accel"] = brake[j], accel[j]
        if brake[j] is None and row["why"] is None:
            row["why"] = "no pass ended inside the still band at half the model's acceleration: the model's values stand"
        log(f"ramps {row['joint']}: brake {brake[j]}, accel {accel[j]}; {row['passes']}")
    know.set("brake_j", brake, "self_test", at)
    know.set("accel_j", accel, "self_test", at)
    know.set("ramps", out, "self_test", at)
    hand["now"] = None
    jaws_to("closed")                                   # the sweep's hand again: the taps run with it
    return out


def _levels(know, p, facts, j, at):
    """The hidden levels a ramp pass showed (it ended at its goal: no contact is proven in it)."""
    radius = facts[0]
    rec, k = p["rec"], p["k_end"]
    for table, x, y in rec.levels:
        know.add_row(table, x, y, at=at)
    know.note_free_leg(radius * p["peak"], radius * p["a"], float(max(rec.f[:k] or [0.0])),
                       tau_abs_max=np.asarray(rec.ext[:k]).max(axis=0), swept=True, ended="goal", at=at)
    know.add_row("f_hidden", [0.0, radius * p["a"]], float(max(rec.f[k:] or [0.0])), at=at)   # the settling stretch


# ------------------------------------------------------------------ the taps (C5)
def _low(ex, B, q, normal, offset):
    """How far (m) the lowest point of the tool stands over the stated support plane at pose q: the tool bodies'
    own hull points and the tip."""
    points = [np.asarray(ex.model.fk_tip(q)[0], dtype=float).reshape(1, -1)]
    for body in getattr(ex, "_tool_bodies", ()):
        hull = B.robot.hull_points(q, body)
        if hull is not None and len(hull):
            points.append(np.asarray(hull, dtype=float))
    return float(np.min(np.vstack(points) @ normal) - offset)


def _step(ex, q_cmd, direction, dist):
    """The command moved so that the tip goes dist along direction (the hand's turn held where the arm can)."""
    J = np.asarray(ex.model.jac(q_cmd), dtype=float)
    J = J if ex.model.n >= J.shape[0] else J[:len(direction)]
    twist = np.concatenate([np.asarray(direction, dtype=float) * dist, np.zeros(J.shape[0] - len(direction))])
    return q_cmd + np.linalg.pinv(J) @ twist


def _line(ex, q_cmd, direction, dist, w_top, a, watch=None, keep=None, brake=None):
    """Drive the tip dist along direction: ramp at a to w_top, then brake at a to rest at the end (brake False, or
    none given with a keep list: hold w_top to the end). keep: a list that takes the readings. Returns (the last
    command, the last reading, the stop word of watch or None)."""
    dt, phi, w, s = ex.dt, 0.0, 0.0, None
    brake = keep is None if brake is None else brake
    while phi < dist:
        w = min(w + a * dt, w_top, math.sqrt(2 * a * (dist - phi))) if brake else min(w + a * dt, w_top)
        step = min(max(w, min(a * dt, w_top)) * dt, dist - phi)     # never over w_top, never so small that it stalls
        q_cmd = _step(ex, q_cmd, direction, step)
        phi += step
        s = ex._period(q_cmd)
        if keep is not None:
            keep.append(s)
        word = watch(s) if watch is not None else None
        if word:
            return q_cmd, s, word
    return q_cmd, s, None


def _rest(ex, know, q_hold, limit, watch=None):
    """Hold q_hold until every joint stands still over QUANT readings (or limit periods). Returns (the last
    reading, the stop word of watch or None)."""
    band = np.maximum(np.asarray(know.get("q_spread")[0], dtype=float), QUANT * np.asarray(know.get("quantum")[0], dtype=float))
    s, quiet = ex._period(q_hold), 0
    for _ in range(limit):
        before = s.q
        s = ex._period(q_hold)
        word = watch(s) if watch is not None else None
        if word:
            return s, word
        quiet = quiet + 1 if np.all(np.abs(s.q - before) <= band) else 0
        if quiet >= QUANT:
            break
    return s, None


def taps(ex, know, q0, at, log):
    """S3 (see the module text). Stores the entry "taps" and the used taps as stops; returns the entry."""
    from rcpx import bounds, bounds_strike
    from rcpx.control_base import Guard

    B = bounds.of(ex)
    out = {"said": "", "used": 0, "rows": [], "top": None, "support": None, "limit": None, "way_down": None}

    def done(text):
        out["said"] = text
        for r in out["rows"]:
            r.pop("stop", None)
            r.pop("pose", None)
        know.set("taps", out, "self_test", at)
        log("taps: " + text)
        return out

    support = B.space.support()
    if support is None:
        return done("no tap: no support is stated")
    _jaws_to(ex, q0, "close")       # the hand as the body's hulls hold it, whatever measuring the jaws left
    (normal, offset), sigma, source = support
    normal = np.asarray(normal, dtype=float)
    offset, normal = float(offset) / float(np.linalg.norm(normal)), normal / float(np.linalg.norm(normal))
    u, dt = -normal, ex.dt
    q = ex._sense().q.copy()
    rigid = float(B.rigid(q, u))
    hulls = ex.model.hulls() if hasattr(ex.model, "hulls") else {}
    e_tool = max((float(h.get("tol_m") or 0.0) for name, h in hulls.items() if name in getattr(ex, "_tool_bodies", ())), default=0.0)
    error = BAND * float(sigma or 0.0) + e_tool + rigid          # how well the support's height and the tool's are known
    out["support"] = {"source": source, "sigma_mm": None if sigma is None else float(sigma) * MM, "error_mm": error * MM}
    a = GIVE * float(B.a_brake(q, u))
    v_tell = laws.arrival_speed(rigid, 0.0, bounds_strike.detection(B)[0], float(B.a_brake(q, u)))
    height = _low(ex, B, q, normal, offset)
    fast = ex._sense().fast if isinstance(ex._sense().fast, dict) else {}
    sensor = fast.get("wrist_n") is not None                 # a force sensor read at the fast rate
    own = not sensor and fast.get("qd") is not None and _servo(ex) is not None    # else the arm's own force there
    fast_force = sensor or own                               # this body resolves a peak
    out["peak_by"] = ("a force sensor read at the fast rate" if sensor else
                      "the arm's own force at each fast sample: its fast joint speeds through its own model and "
                      "servo law" if own else "nothing: no force signal and no fast joint speeds with a stated servo law")
    # the body's model of the stop slope where a tap is met: the largest over the way down to the support (it
    # changes with the pose, and the contact is at the END of the way)
    down = [q]
    for _ in range(int(1 / REL)):
        down.append(_step(ex, down[-1], u, height * REL))
    slopes = [bounds_strike.normal_z_of(B, pose, u) for pose in down]     # a tap's normal is its travel
    z_model = None if any(z is None for z in slopes) else max(slopes)
    top, top_why, limit = _top(ex, B, q, u, ex.cfg.get("selftest_taps") == "free", z_model)
    t_run = float(know.get("t_settle")[0] or 0.0) + QUANT * QUANT * dt
    run_in = lambda v: v ** 2 / (2 * a) + v * t_run
    out["top"] = {"top_mm_s": top * MM, "why": top_why, "tell_mm_s": None if not v_tell else v_tell * MM, "fastest_tap_mm_s": 0.0}
    if not v_tell or not math.isfinite(v_tell) or not top > 0 or not height > error:
        return done(f"no tap: the tell speed, the ladder's top or the height over the support is not resolved "
                    f"(tell {v_tell}, top {top}, height {height * MM:.1f} mm; {top_why})")

    def reach_n(v, got=None):
        """What a tap at v is modelled to reach (N), with the margin: the guard's level plus the slope times v."""
        z, _ = stops.slope(z_model, got)
        level = ex._guard_spec("touch", v=v).force_n
        return None if z is None or level is None else float(level) + z * v

    if limit is not None:
        first = reach_n(v_tell)
        out["limit"] = dict(limit, kept=None, tell_model_n=first)
        if first is None or first > limit["n"]:
            out["limit"]["said"] = (f"a tap at the tell speed ({v_tell * MM:.3g} mm/s) is modelled to reach "
                                    f"{'no number' if first is None else format(first, '.3g') + ' N'}, over "
                                    f"{limit['what']} of {limit['n']:.3g} N")
            return done("no tap: " + out["limit"]["said"])
    ladder, v = [], float(v_tell)
    if limit is not None and not fast_force:
        ladder = [v] * (QUANT + 1) if run_in(v) < height - error else []     # the tell speed only (the module text)
    else:
        while v < top * (1 - REL ** 2) and run_in(v) < height - error and (fast_force or len(ladder) < QUANT + 1):
            ladder.append(v)
            v *= DOUBLE
        if (fast_force or len(ladder) < QUANT + 1) and run_in(top) < height - error and (not ladder or top > ladder[-1]):
            ladder.append(top)                  # the top itself: the last tap, or the third where doubling gives two
        elif fast_force and ladder:
            top_why += "; cut where the run to a steady speed no longer fits the height the hand stood at"
        if limit is not None and len(ladder) < QUANT + 1 and run_in(float(v_tell)) < height - error:
            ladder = [float(v_tell)] * (QUANT + 1)      # too few speeds fit under the stated limit: the tell speed only
    out["top"].update(why=top_why, fastest_tap_mm_s=(max(ladder) if ladder else 0.0) * MM)
    if len(ladder) < QUANT + 1:
        if not v_tell < top:
            return done(f"no tap: the tell speed ({v_tell * MM:.3g} mm/s) is at or over the ladder's top ({top * MM:.3g} "
                        f"mm/s, {top_why}): the arm cannot tell the support from a thing {error * MM:.2f} mm over it "
                        "at a speed it may tap at")
        return done(f"no tap: {len(ladder)} speeds fit between the tell speed ({v_tell * MM:.3g} mm/s) and the "
                    f"ladder's top ({top * MM:.3g} mm/s) from the height the hand stands at; fewer than {QUANT + 1} "
                    "taps cannot show that the rest height repeats")
    ex._travel_direction = u
    wait = max(QUANT, int(math.ceil(float(B.limit_s(float(B.t_settle()))) / dt)))
    home = lambda: _rest(ex, know, q0, wait)
    # the way down to the first tap's start: one line that ramps and brakes at REL of the pass acceleration, so that
    # the servo follows its end, to rest above the support's band; the guard is armed on the way and at the rest
    start = run_in(ladder[0]) + error + REL ** 2 * height
    a_way = REL * a
    v_way = min(GIVE * float(B.v_free(q, u)), math.sqrt(a_way * max(height - start, 0.0)))
    g = Guard(ex, spec=ex._guard_spec("transit", v=v_way, a=a_way))
    out["way_down"] = {"strike": _way_price(ex, B, q0, u, height - start, v_way, g.force_n), "ran": None}
    seen = []
    q_cmd, s, word = _line(ex, q0.copy(), u, height - start, v_way, a_way, watch=g.check, keep=seen, brake=True)
    s, word = (s, word) if word else _rest(ex, know, q_cmd, wait, watch=g.check)
    out["way_down"]["ran"] = {"v_m_s": v_way, "read_n": max((float(r.Fmag) for r in seen), default=0.0),
                              "stopped": word or "goal", "note": bounds_strike.READ_NOTE}
    if word:
        q_cmd = ex._sense().q.copy()
        s, _ = _rest(ex, know, q_cmd, wait)
        met = _low(ex, B, s.q, normal, offset)
        _line(ex, q_cmd, normal, max(0.0, height - met), v_way, a_way)
        home()
        return done(f"no tap: the way down ended on {word} {met * MM:.1f} mm over the stated support; "
                    f"{out['way_down']['strike']['text']}")
    k, last = 0, False
    while k < len(ladder):
        v = ladder[k]
        made = [r["stop"] for r in out["rows"]]
        got = stops.fit(made)
        if limit is not None and out["rows"] and not got["n"]:
            # under a stated limit the body's model may not stand as the bound: while no tap is resolved, none
            # is made faster than the ones made, and no more of them than show that the rest height repeats
            if len(out["rows"]) >= QUANT + 1:
                break
            v = max(r["v_cmd_mm_s"] for r in out["rows"]) / MM
        if fast_force and got["n"] and got["c"] > 1:
            # the taps made so far read over the body's model: they say where the peak reaches what the arm can
            # push (or a stated limit), short of the model's top: no tap is made over that speed, and the one at
            # it is the last
            v_most, why_most, _ = _top(ex, B, q, u, ex.cfg.get("selftest_taps") == "free", z_model, got)
            if v >= v_most * (1 - REL ** 2):
                fastest = max(r["v_cmd_mm_s"] for r in out["rows"]) / MM
                v, last = min(v, v_most), True
                out["top"].update(top_mm_s=v_most * MM, why=why_most)
                if not v > (1 + REL ** 2) * fastest:
                    if len(out["rows"]) >= QUANT + 1:
                        break                                     # nothing faster than the last tap is left
                    v, last = fastest, False                      # too few taps to show the rest height repeats:
                                                                  # once more at the fastest speed tapped
        g = Guard(ex, spec=ex._guard_spec("touch", v=v))
        kept, drive, w, word, start_at = [], [], 0.0, None, s
        # the command's own travel, at most: the measured height, the support's error, the run and the tracking lag;
        # and where the command stands OVER the hand at the start (the joints' own friction band, left by the way
        # back up), that offset twice: once to take it up, once more as the trail the hand then follows with
        low = _low(ex, B, s.q, normal, offset)
        over = max(0.0, _low(ex, B, q_cmd, normal, offset) - low)
        reach = low + laws.TWO * over + error + v * t_run + float(B.lag(v))
        while word is None and reach > 0 and _low(ex, B, s.q, normal, offset) > -(error + QUANT * v * dt):
            w = min(w + a * dt, v)
            q_cmd = _step(ex, q_cmd, u, w * dt)
            reach -= w * dt
            s = ex._period(q_cmd)
            kept.append(s)
            drive.append((np.asarray(ex._q_cmd, dtype=float).copy(), float(ex._frac_cmd)))   # the servo's own target
            word = g.check(s)
        row = {"v_cmd_mm_s": v * MM, "stopped": word or "nothing met", "f_guard_n": g.force_n, "periods": len(kept),
               "t_s": float(ex.sim_time())}
        if not word:        # what the descent that met nothing read: where the tool ended and what force it showed
            row.update(low_mm=_low(ex, B, s.q, normal, offset) * MM, reach_left_mm=reach * MM,
                       read_n=max((float(r.Fmag) for r in kept), default=0.0))
        if word:
            q_cmd = ex._sense().q.copy()                           # the goal snaps to the reading: the hand rests on it
            for _ in range(QUANT):                                 # the stop's own readings after the one that ended it
                kept.append(ex._period(q_cmd))
                drive.append((np.asarray(ex._q_cmd, dtype=float).copy(), float(ex._frac_cmd)))
            n_used = min(len(kept), KEEP + QUANT + QUANT)          # the readings before the contact, the run that
            used = kept[-n_used:]                                  # tripped and the ones after it
            ended = used[-1 - QUANT]
            samples, free = [], None
            for i in range(len(kept) - n_used, len(kept)):
                r = kept[i]
                block = r.fast
                if own and isinstance(block, dict):
                    # the first reading kept stands for the free motion: what the model leaves there is taken
                    # off the readings up to the one that ended the move (after it the command stands)
                    mine = _model_force(ex, kept[i - 1] if i > 0 else start_at, r, *drive[i], u,
                                        free if i <= len(kept) - 1 - QUANT else None)
                    if mine is not None and free is None and n_used == KEEP + QUANT + QUANT:
                        free = mine[2]                             # a whole window: its first reading is free
                        mine = _model_force(ex, kept[i - 1] if i > 0 else start_at, r, *drive[i], u, free)
                    if mine is not None:
                        block = dict(block, model_n=mine[0], hides_n=mine[1])
                samples.append({"qd": r.qd, "J": np.asarray(ex.model.jac(r.q), dtype=float)[:len(u)],
                                "f": float(r.Fmag), "fast": block, "ended": r is ended})
            pose = ended.q
            stop = stops.measure(samples, g.force_n, bounds_strike.normal_z_of(B, pose, u), dt, plan_v=v)
            stop.update(q=[float(x) for x in pose], u=[float(x) for x in u], body="tool")     # C5: its pose and body
            s, _ = _rest(ex, know, q_cmd, wait)
            stop["height_m"], stop["rest_n"], stop["where"] = _low(ex, B, s.q, normal, offset), float(s.Fmag), "tap"
            give = float(B.rigid(s.q, u, max(float(g.force_n or 0.0), stop["rest_n"])))   # with the force it rests with
            qd = np.linalg.pinv(np.asarray(ex.model.jac(pose), dtype=float)[:len(u)]) @ (u * v)
            said = bounds_strike.price([bounds_strike.sample_of(B, pose, qd, None, g.force_n)], None, bounds_strike.detection(B)[0])
            row.update(stop=stop, pose=pose, give=give, v_mm_s=None if stop["v"] is None else stop["v"] * MM,
                       height_mm=stop["height_m"] * MM, rest_n=stop["rest_n"], det_periods=stop["det_periods"],
                       resolved=stop["resolved"], peak_n=stop["peak_n"], read_n=stop["read_n"], why=stop["why"],
                       model_peak_n=said["model_peak_n"], at_least_n=said["at_least_n"])
        if not word:
            out["rows"].append(row)
            log(f"tap at {v * MM:.2f} mm/s: {row}")
            break
        if abs(row["height_mm"] / MM) > error - rigid + row["give"]:
            # a descent that ended away from the stated support (a joint at its torque limit on the way, a thing
            # in the way) is no tap of the support: the climb ends there and the taps made before it stand
            row["void"] = (f"ended on {word} {row['height_mm']:.1f} mm over the stated support, outside its error: "
                           "no tap of the support; the climb ends here")
            out["rows"].append(row)
            log(f"tap at {v * MM:.2f} mm/s: {row['void']}")
            break
        out["rows"].append(row)
        log(f"tap at {v * MM:.2f} mm/s: {({k_: val for k_, val in row.items() if k_ not in ('stop', 'pose')})}")
        k += 1
        if last:
            break
        back = run_in(ladder[min(len(ladder) - 1, k)]) + error
        q_cmd, s, _ = _line(ex, q_cmd, normal, back, max(v, v_way * GIVE), a)
        s, _ = _rest(ex, know, q_cmd, wait)
    out["top"]["fastest_tap_mm_s"] = max((r["v_cmd_mm_s"] for r in out["rows"]), default=0.0)
    up = height - _low(ex, B, s.q, normal, offset)
    if up > 0:
        _line(ex, q_cmd, normal, up, v_way, a_way)
    home()
    met = [r for r in out["rows"] if r.get("stop") is not None and not r.get("void")]
    heights = [r["height_mm"] / MM for r in met]
    give = max([r["give"] for r in met], default=rigid)
    error, rigid = error - rigid + give, give
    out["support"]["error_mm"], out["support"]["resolution_mm"] = error * MM, rigid * MM
    ok, text = stops.tapped(heights, len([r for r in out["rows"] if not r.get("void")]) if fast_force else len(ladder), error, rigid)
    if ok:
        # each tap was measured against the body's model with the detection time in force while it ran (the guard's
        # own readings: no stop was stored yet). From here on the model stands with the detection time the taps
        # showed, so each tap's model slope is taken again with it: the fitted slope then gives back the taps'
        # own peaks at their own speeds
        seconds, _, _ = stops.t_det([r["stop"] for r in met], dt)
        for r in met:
            again = stops.model_z(**dict(bounds_strike.model_parts(B, r["pose"], u), t_det_s=seconds))
            if again is not None and r["stop"].get("model_z") is not None:
                r["stop"]["model_z"] = float(again)
            know.add_stop(r["stop"], where="tap", at=at)
        out["used"] = len(met)
        seconds, _, _ = stops.t_det(know.stops(), dt)
        got = stops.fit(know.stops())
        text = (f"{len(met)} taps onto the stated support between {min(r['v_cmd_mm_s'] for r in met):.3g} and "
                f"{max(r['v_cmd_mm_s'] for r in met):.3g} mm/s: the detection time is {seconds / dt:.0f} control "
                f"periods ({seconds:.3g} s)")
        text += (f"; {got['n']} stops resolved by {out['peak_by']}: they read {got['c']:.3g} times the body's model"
                 + ("" if got["c"] > 1 else " (the model stands)") if got["n"] else f"; {stops.NO_PEAK}")
    if limit is not None:
        peaks = [r["peak_n"] for r in met if r.get("resolved")]
        if not fast_force or not peaks:
            out["limit"].update(kept=None, said=(f"{limit['what']} of {limit['n']:.3g} N is not shown kept: this body "
                                                 f"resolved no peak of these taps; they ran at the tell speed only, where "
                                                 f"the body's model says {out['limit']['tell_model_n']:.3g} N"))
        else:
            out["limit"].update(kept=max(peaks) <= limit["n"], largest_peak_n=max(peaks), said=(
                f"{limit['what']} of {limit['n']:.3g} N: the taps' own largest peak was {max(peaks):.3g} N"
                + ("" if max(peaks) <= limit["n"] else ": the limit was passed")))
        text += "; " + out["limit"]["said"]
    return done(text)
