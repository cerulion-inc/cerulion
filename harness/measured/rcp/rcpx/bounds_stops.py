# SPDX-License-Identifier: AGPL-3.0-only
"""A stop: what the robot's own readings say of it, and what the robot learns from it (round six, the design rulings).

Pure: no robot, no backend, no session. The same code serves a task's stop and a self-test tap.

- measure(samples, f_thr, model_z, dt, plan_v=None, rel=REL): one stop from the drive's readings around it.
  samples, one per control period in time order, each {"qd": joint speeds at the reading, "J": the tip's 3 x n
  position Jacobian at the reading's pose (or "v_tip": the tip's velocity, 3), "f": the period's force reading in
  N, "fast": the backend's fast block of that period or None ({"dt_s", "qd": k x n and a force signal at the fast
  rate where there is one: "wrist_n": k x 3 of a force sensor, or "model_n": k values of the arm's own force, which
  the self-test makes from the fast joint speeds through the body's own model and servo law, with "hides_n": what
  the joints' dry friction can hide at each of them, added to the peak alone), "ended": True on the reading that
  ended the move (none marked: the last one)}.
  v is the MEASURED tip speed of the reading before the first LOADED one, u the travel's direction: the reading
  before the first one over the threshold, or, where the load rose over two readings or more before it (a crawl
  under a stated limit, a press), the last reading kept before that rise and no less than the hand was told.
  Where the speed was changing there (a leg that brakes, the lag tail after a plan ended, or a contact that began
  a period before the threshold was passed) the fast joint speeds of the last two periods give the speed just
  before the contact, when they follow the trend of the readings before up to one sharp drop; else the stop says
  so and is not resolved. A contact that began a period early counts that period in det_periods.
  The peak, three ways, each named in the result:
  peak_n: from a force signal at the fast rate, resolved when at least QUANT of its samples stand over the
  threshold. Where a force sensor's signal itself stands over the threshold in the period before the first
  reading over it, the contact began there (with or without fast joint speeds). The arm's own force moves no
  onset (what its model leaves in a free period is no contact); the period before the contact's is searched for
  its peak too.
  bound_n: where the body has no such signal but a fast block: an UPPER BOUND of the peak (see _gathered: a
  period's reading is its mean force, so its impulse is known, and the most the force can have been is that
  impulse gathered in one fast sample). It is what a result states for that stop; it is no measurement of the
  peak, so the stop is not resolved and enters no fit by it.
  read_n: the largest reading, once per period: a floor of the peak, always given. A deceleration times a mass
  is not a peak.
  The peak and read_n are the stop's own: readings after the one that ended the move count only while the force
  has not yet fallen REL under its largest so far (a rise after that fall is another event, the tool's own later
  press). det_periods (the control periods from the reading before the contact to the reading that ended the
  move, at least 1) is given either way; a stop with no reading before its contact has det_periods None and no
  speed: nothing reads a detection time off it.
- fit(stops): over the resolved stops, c = the largest (peak_n - hid_n - f_thr) / (v x model_z of that stop) over
  EVERY pose and body tapped (contract C5 changed): how the body's own model stood against what a stop showed,
  each stop at its own pose, over it or under it. hid_n: what the joints' dry friction adds to a stop's peak
  whatever its speed. e0: the most a stop's own peak stood over the line the stated slope draws through the
  guard's level: stated beside the slope, never inside it, so every figure gives back the stops' own peaks.
  The range a number is stated in (v0, v1) and n come only from stops that bear it: a self-test tap ("where":
  "tap"), a stop on a thing known rigid (the caller's "rigid": True: the stated support, a fixed thing), or a
  stop whose own ratio is within REL of c; each kind of body on its own: the tool's at the top, and "link" the
  same fit of the stops made with a link (a link's figure is stated only inside them, on their own ratio;
  of_kind(fit, link) gives the one a figure rests on; the calibration makes none: rcpx/bounds_selftaps_more.py
  says why). A fast stop on a thing that gave way widens nothing. f0: the peak of the slowest stop that bears the
  range. poses and bodies: how many different poses (a joint apart by more than REL rad) and bodies (the tool,
  or the link that met) the bearing stops stand at: said in the texts. A stop that is only bounded or only
  read bears on nothing.
- slope(model_z_here, fit): the stop slope a FIGURE is stated with and a stated LIMIT is solved with, at a pose:
  (1 + rel) x the body's model there (rcpx/bounds_know_point.py: the point held fast), times c where the stops
  read over the model (c over 1). A ratio under 1 is never carried into it: it was read at the taps' own pose
  and travel, and on rows nobody fitted to one arm's true ratio ran 1.45 times its taps' (the first pass let a
  stated limit be passed by a slope scaled down so). The text starts "fitted:" whenever a stop bears it and
  names the ONE term that set the number.
- estimate(model_z_here, fit): the stop slope for SOLVING a speed from a force that is no stated limit (the
  force a touch, a close or a landing ends at: the design rulings; a push law's own ceiling): what this
  body's stops showed, (1 + rel) x c x the body's model here, c also under 1. It promises nothing and states
  no figure: the receipt's strike is slope()'s. Without it every such arrival ran at half its speed or less.
- said_of(z, text, fit, q, u): what the bounds hand over with a model slope (the range, the travels and the
  pose spread of the stops that bear it); covers(said, u, q): whether those stops cover a stop's travel and pose.
- t_det(stops, dt, readings): the largest measured detection time; with no stop the guard's own readings.
- model_z(...): the body's model of the stop slope: the contact model's impact term plus the servo's stiffness
  times (the detection time plus the lag). No rise term.
- tapped(heights, n, error, rigid): whether a ladder of self-test taps met the stated support and nothing else
  (at least QUANT + 1 taps: fewer cannot show that the rest height repeats).
"""

import math

import numpy as np

from rcpx.bounds_ratios import GIVE, QUANT, REL

MM = 1000.0  # unit: metres to millimetres, for the texts
NO_FAST = "no force signal at the physics rate: the arm's reading once per period is not the peak"
NO_PEAK = "no stop's peak resolved on this body"
BOUND = ("no force signal at the physics rate: the peak is bounded from above by two readings' impulse gathered in "
         "one fast sample; it is not measured")


class Slope(float):
    """A model slope (N per m/s) that carries what its maker knows beside the number (plain attributes; it adds and
    multiplies as the float it is): "said": {"z", "source", "e0", "v0", "f0", "v1", "n"}: the stop slope the
    bounds state at that pose and the range this body's stops bear it in, for the figure a stop states at the
    speed it was met at (rcpx.bounds_strike.ran)."""

    def __new__(cls, value, **more):
        obj = float.__new__(cls, value)
        obj.__dict__.update(more)
        return obj


def _reading_speed(sample):
    """The tip's velocity (3) at a reading: its Jacobian times the joint speeds (the last fast ones of its period
    where the body gives them), else the tip velocity the sample states; None when it has neither."""
    if sample.get("J") is None or sample.get("qd") is None:
        return None if sample.get("v_tip") is None else np.asarray(sample["v_tip"], dtype=float)
    fast = sample.get("fast") if isinstance(sample.get("fast"), dict) else None
    rows = None if fast is None else fast.get("qd")
    qd = np.asarray(rows, dtype=float)[-1] if rows is not None and len(rows) else np.asarray(sample["qd"], dtype=float)
    return np.asarray(sample["J"], dtype=float) @ qd


def _period_speed(sample):
    """The tip's mean velocity (3) over the control period that ended at a reading: its Jacobian times the mean of
    the period's fast joint speeds; None where the body gives none."""
    fast = sample.get("fast") if isinstance(sample.get("fast"), dict) else None
    rows = None if fast is None else fast.get("qd")
    if rows is None or not len(rows) or sample.get("J") is None:
        return None
    return np.asarray(sample["J"], dtype=float) @ np.mean(np.asarray(rows, dtype=float), axis=0)


def _fast_along(sample, J, u):
    """The tip's speed along u at every fast sample of a reading's period, or None where the body gives none."""
    fast = sample.get("fast") if isinstance(sample.get("fast"), dict) else None
    rows = None if fast is None else fast.get("qd")
    if rows is None or len(rows) < QUANT or not fast.get("dt_s") or J is None:
        return None, None
    return [float(x) for x in np.asarray(rows, dtype=float) @ np.asarray(J, dtype=float).T @ u], float(fast["dt_s"])


def _arrival(speeds, samples, first, u, dt, tol):
    """(arrival speed, why not, early, least): the tip's speed along u just before the contact began. speeds[i]:
    the tip's speed along u at reading i (None where not given); first: the first reading over the threshold.
    early: the contact began in the period BEFORE that reading (its fast joint speeds show the drop there).
    least: where the speed was changing before the contact, the least fast speed before the drop (else None)."""
    v, before = speeds[first - 1], speeds[first - QUANT] if first >= QUANT else None
    steady = before is not None and abs(v - before) <= tol * max(v, before)
    J = samples[first - 1].get("J")
    tripped, h = _fast_along(samples[first], J, u)
    changing = ("the hand's speed was changing at the stop (a leg that brakes, or the lag tail) and the fast joint "
                "speeds do not show the speed of arrival")
    if tripped is not None:
        last, _ = _fast_along(samples[first - 1], J, u)
        if last is not None and first > QUANT and speeds[first - QUANT - 1] is not None and before is not None:
            # two periods of fast speeds from the reading two before: the trend is the one the readings before had
            base, trend, series = before, (speeds[first - QUANT - 1] - before) * h / dt, [before] + last + tripped
        else:
            base, trend, series = v, (0.0 if before is None else (before - v) * h / dt), [v] + tripped
            last = []
        drops = [x - y for x, y in zip(series, series[1:])]
        k = int(np.argmax(drops))
        follows = all(abs(series[i] - (base - trend * i)) <= tol * base for i in range(k + 1))
        if drops[k] > QUANT * abs(trend) and follows:
            early = k < len(last)
            return (v if steady and not early else series[k]), "", early, None
        if drops[k] > 0 and series[k] > 0 and min(series[1:k + 1], default=series[k]) > 0:
            # the speed was changing before the contact. The contact begins at the first drop of at least GIVE of
            # the largest one. A hand that only brakes or only gains speed up to there (the lag tail, a ramp) met
            # the thing at its fast speed just before that drop; one whose speed goes up and down (a servo that
            # ripples inside each period, joints that stick and slip) is given no speed here: the caller takes
            # its mean over the period. The LEAST fast speed before the drop stands in a slope's ratio (the
            # slower the arrival, the larger the ratio).
            k = next(i for i, d in enumerate(drops) if d >= GIVE * drops[k])
            steps = [b - a for a, b in zip(series[1:k + 1], series[2:k + 1])]
            one_way = all(d <= 0 for d in steps) or all(d >= 0 for d in steps)
            return (series[k] if one_way else None), "", k < len(last), min(min(series[1:k + 1], default=series[k]), v)
        if drops[k] > 0:
            return None, changing, False, None
    if steady:
        return v, "", False, None
    if before is None:
        return None, "one reading before the contact: the hand's speed there is not known to be steady", False, None
    return None, ("the hand's speed was changing at the stop (a leg that brakes, or the lag tail): the speed of "
                  "arrival is not known to " + format(tol, ".0%")), False, None


def _wrist(sample, hidden=False, sensor=False):
    """The magnitudes of the force signal at the fast rate over a reading's period ([] where the body has none):
    a force sensor's ("wrist_n": k x 3), else the arm's own force at each fast sample ("model_n": k values).
    hidden: with what the joints' dry friction can hide at that sample on top ("hides_n": k values, where the
    caller states it): the most the push can have been. sensor: a force sensor's signal only."""
    fast = sample.get("fast") if isinstance(sample.get("fast"), dict) else None
    rows = None if fast is None else fast.get("wrist_n")
    if rows is None or not len(rows):
        own = None if fast is None or sensor else fast.get("model_n")
        if own is None or not len(own):
            return []
        more = fast.get("hides_n") if hidden else None
        more = [0.0] * len(own) if more is None or len(more) != len(own) else more
        return [float(x) + float(y) for x, y in zip(own, more)]
    return [float(x) for x in np.linalg.norm(np.asarray(rows, dtype=float).reshape(len(rows), -1), axis=1)]


def _gathered(samples, onset, end, rel):
    """An upper bound (N) of a stop's peak from the arm's own readings where it has no force signal at the fast
    rate; None where a period it needs gives no fast block. A period's reading is the period's mean force, so the
    period's impulse is known, and no signal of the body resolves a time shorter than one fast sample: the most
    the force can have been is that impulse gathered in ONE fast sample, the reading times the samples of the
    period. A burst in a period's last sample is read in part by the reading after it, so two readings in a row
    count together. The periods: from the one before the first reading over the threshold (the contact may begin
    there) to QUANT after the one that ended the move. The largest, with the accepted relative error of a
    reading on top. It is a worst case, several times the peak of a stop that took its time; it is under no
    true peak."""
    best = None
    for i in range(max(0, onset - 1), min(len(samples), end + QUANT + 1)):
        fast = samples[i].get("fast") if isinstance(samples[i].get("fast"), dict) else None
        rows = None if fast is None else fast.get("qd")
        if rows is None or not len(rows):
            return None
        after = float(samples[i + 1].get("f") or 0.0) if i + 1 < len(samples) else 0.0
        top = (float(samples[i].get("f") or 0.0) + after) * len(rows)
        best = top if best is None else max(best, top)
    return None if best is None else (1 + rel) * best


def _own(inside, after, rel):
    """The values of a stop's own force series: those up to the reading that ended the move, then the later ones
    only while the force has not yet fallen rel under its largest so far (a rise after that fall is another event)."""
    out = list(inside)
    top = max(out, default=None)
    for x in after:
        if top is None or not out or out[-1] < (1 - rel) * top or x < (1 - rel) * top:
            break
        out.append(x)
        top = max(top, x)
    return out


def measure(samples, f_thr, model_z, dt, plan_v=None, rel=REL):
    """One stop from the drive's readings around it (see the module text). Returns {"v", "u", "peak_n" or None,
    "bound_n" or None, "read_n", "resolved", "det_periods" (None with no reading before the contact), "model_z",
    "f_thr", "why"}; v and u are None when no reading before the contact gives a speed."""
    samples = list(samples)
    out = {"v": None, "u": None, "peak_n": None, "bound_n": None, "read_n": None, "resolved": False,
           "det_periods": None, "model_z": None if model_z is None else float(model_z),
           "f_thr": None if f_thr is None else float(f_thr), "why": ""}
    if not samples:
        out["why"] = "no reading of the stop"
        return out
    end = next((i for i, s in enumerate(samples) if s.get("ended")), len(samples) - 1)
    over = lambda s: f_thr is not None and float(s.get("f") or 0.0) > f_thr
    first = end
    while first > 0 and over(samples[first]) and over(samples[first - 1]):
        first -= 1                                    # the first reading of the run that tripped
    onset = first                                     # the first reading in whose period the contact stood
    while onset > 0 and f_thr is not None and max(_wrist(samples[onset - 1], sensor=True), default=0.0) > f_thr:
        onset -= 1                                    # a force sensor's signal itself shows it a period before
    reads = lambda rows: [float(s.get("f") or 0.0) for s in rows]
    out["read_n"] = max(_own(reads(samples[first:end + 1]), reads(samples[end + 1:]), rel))
    why, early = [], False
    if onset == 0:
        why.append("no reading before the contact")
    else:
        out["det_periods"] = max(1, end - (onset - 1))
        vels = [_reading_speed(r) for r in samples[:onset]]
        vel = vels[onset - 1]
        v = None if vel is None else float(np.linalg.norm(vel))
        base = vels[onset - QUANT] if onset >= QUANT and vels[onset - QUANT] is not None else vel
        if not v or not float(np.linalg.norm(base)) > 0:
            why.append("the reading before the contact gives no tip speed" if v is None else
                       "the hand stood still before the contact")
        else:
            u = base / float(np.linalg.norm(base))            # the travel's direction, from before any contact
            speeds = [None if x is None else float(x @ u) for x in vels]
            arrive, why_not, early, least = _arrival(speeds, samples, onset, u, dt, rel / QUANT)
            if arrive is None:
                # no steady, sharply ended or one-way arrival: the speed met is the hand's MEAN speed over the
                # control period before the first loaded reading (C2), from its fast joint speeds where the body
                # gives them: a hand that sticks and slips reads anything at one instant
                mean = _period_speed(samples[onset - 1])
                arrive = speeds[onset - 1] if mean is None else float(mean @ u)
            out["v"], out["u"] = float(arrive), [float(x) for x in u]
            if onset >= QUANT and speeds[onset - 1] is not None and speeds[onset - QUANT] is not None \
                    and speeds[onset - 1] > speeds[onset - QUANT]:
                out["v_gain"] = float(speeds[onset - 1] - speeds[onset - QUANT])   # still speeding up: one more
                # period of the same gain may have passed before the contact (rcpx.bounds_strike.ran)
            if least is not None and least < out["v"]:
                out["v_least"] = float(least)
            if early:   # the fast joint speeds show the contact a period before: detection took that period too
                out["det_periods"] += 1
            if early or onset < first:
                out["began"] = "the contact began one control period before the first reading over the threshold"
            if why_not:
                why.append(why_not)
            if plan_v is not None and abs(float(plan_v) - out["v"]) > rel * max(float(plan_v), out["v"]):
                why.append("the plan's speed and the hand's differ")
            # a load that rose reading by reading up to the threshold (a slow arrival under a stated limit, a
            # press) began before the first reading over it. Where it rose over two readings or more, the reading
            # before the threshold is no reading before the contact: the hand already leans on the thing and
            # has lost speed. The speed met is then the largest of the last reading kept before that rise (its
            # period's mean), the speed the hand was told there (the plan's) and what was read; where no reading
            # kept stands before the rise, the speed the hand was told. Such a stop is no arrival for a fit.
            loaded = onset
            level = lambda i: float(samples[i].get("f") or 0.0)
            while loaded > 0 and rel * level(first) < level(loaded - 1) < level(loaded):
                loaded -= 1
            if loaded <= 1 and onset > 1 and plan_v is not None:      # the rise fills the readings kept
                out["v"] = float(plan_v)
                out["began"] = ("the load had begun before the first reading kept: the speed met is the speed the "
                                "hand was told")
                why.append("the load rose over the readings before the threshold: no arrival is read")
            elif 1 <= loaded <= onset - QUANT:
                free = _period_speed(samples[loaded - 1])
                free = speeds[loaded - 1] if free is None else float(free @ u)
                out["v"] = max(out["v"], float(free or 0.0), float(plan_v or 0.0))
                out["began"] = (f"the load rose over {onset - loaded} readings before the threshold: the speed met "
                                "is that of the last reading before the rise, no less than the hand was told")
                why.append("the load rose over the readings before the threshold: no arrival is read")
    lo = max(0, onset - (1 if early else 0))
    flat = lambda rows, hidden=False: [x for s in rows for x in _wrist(s, hidden)]
    mags = _own(flat(samples[lo:end + 1]), flat(samples[end + 1:]), rel)
    if mags:
        # the same samples with what dry friction can hide on top (nothing on a body that states none). The arm's
        # own force is no sensor: what its model leaves in a free period can stand over the threshold, so it moves
        # no onset; the period before the contact's is searched for the peak instead (a burst in its last sample)
        back = 0 if _wrist(samples[lo], sensor=True) else int(lo > 0)
        own = flat(samples[lo - back:lo], True) + flat(samples[lo:end + 1], True) + flat(samples[end + 1:], True)
        out["peak_n"] = float(max(own[:len(flat(samples[lo - back:lo])) + len(mags)]))
        # what the joints' dry friction adds to that peak (0 on a body that states none): a force that stands at
        # the stop whatever the speed of arrival, so it is no part of a slope (fit) and is stated beside it
        out["hid_n"] = max(0.0, out["peak_n"] - float(max(flat(samples[lo - back:lo]) + mags)))
        # the peak is the stop's own; that the signal resolves it shows in how many of its samples stand over the
        # threshold from the contact on (the fall after the peak is the same contact)
        if sum(1 for x in flat(samples[lo:]) if x > (f_thr or 0.0)) < QUANT:
            why.append(f"the peak stands on fewer than {QUANT} samples of the force signal")
    else:
        bound = _gathered(samples, onset, end, rel)
        if bound is not None:
            out["bound_n"] = float(max(bound, out["read_n"]))
        why.append(NO_FAST if bound is None else BOUND)
    if model_z is None or not model_z > 0:
        why.append("the body's model gives no stop slope at this pose")
    if getattr(model_z, "said", None):        # the stop slope the bounds state here (Slope): for ran's figure
        out["said"] = dict(model_z.said)
    energy = getattr(model_z, "moving", None)
    if callable(energy) and out.get("v") and out.get("u") is not None and onset > 0:
        out.update(_moving(samples[onset - 1], out, energy, dt))
    out["resolved"], out["why"] = not why, "; ".join(why)
    return out


def _moving(sample, out, energy, dt):
    """{"moving": {body: {"v", "z", "point", "m_from"}}} or {"moving_why": text}: the energy bound of every moving
    body (the bounds' own Slope.moving, rcpx.bounds_strike.points_of) at the joint speeds of the reading before
    the contact, raised to the speed the stop says it was met at where that is faster. A stop of a body that
    states no such model has none, and says why."""
    fast = sample.get("fast") if isinstance(sample.get("fast"), dict) else None
    rows = None if fast is None else fast.get("qd")
    qd = np.asarray(rows, dtype=float)[-1] if rows is not None and len(rows) else sample.get("qd")
    if qd is None:
        return {"moving_why": "the reading before the contact gives no joint speeds"}
    qd = np.asarray(qd, dtype=float)
    vel = _reading_speed(sample)
    read = 0.0 if vel is None else float(vel @ np.asarray(out["u"], dtype=float))
    want = float(out["v"]) + float(out.get("v_gain") or 0.0)
    if read > 0 and want > read:
        qd = qd * (want / read)
    try:
        got = energy(qd)
    except (ValueError, ArithmeticError, np.linalg.LinAlgError) as error:
        return {"moving_why": f"the body's model gave no energy bound ({type(error).__name__}: {error})"}
    keep = ("point", "m_from", "k_from", "impact", "k_n_m", "lead_s", "push_n")
    return {"moving": {name: dict({k: e.get(k) for k in keep}, v=float(e["v"]), z=float(e["z"]))
                       for name, e in got.items() if e.get("z") is not None},
            "det_s": None if out.get("det_periods") is None else float(out["det_periods"]) * float(dt)}


def _resolved(stops):
    return [s for s in stops or () if s.get("resolved") and s.get("peak_n") is not None
            and (s.get("v") or 0) > 0 and (s.get("model_z") or 0) > 0]


def _poses(stops, rel):
    """How many different poses the stops stand at: two stand at one pose when no joint differs by more than rel
    rad; a stop that keeps no pose counts with the first."""
    seen = []
    for stop in stops:
        q = None if stop.get("q") is None else np.asarray(stop["q"], dtype=float)
        if not any((q is None and p is None) or (q is not None and p is not None and q.shape == p.shape
                                                 and float(np.max(np.abs(q - p))) <= rel) for p in seen):
            seen.append(q)
    return len(seen)


def fit(stops, rel=REL):
    """{"c", "n", "v0", "v1", "seen", "f0", "t0", "e0", "poses", "bodies", "link"}: the fit of the stops made with
    the TOOL, and under "link" the same fit of the stops made with a LINK ("body" names one): each kind's figures
    rest on that kind's own stops (a link's point meets a thing through other masses and levers than the tool's).
    In each: c = the largest (peak_n - hid_n - f_thr) / (v x model_z of that stop) over the resolved stops of
    EVERY pose and body of that kind (seen: how many they are; hid_n: what the joints' dry friction adds to a
    stop's peak whatever its speed). e0 (N, never under 0): the most a stop's own peak stood over the line the
    stated slope draws through the guard's level (f_thr + (1 + rel) x the larger of 1 and c x v x model_z):
    what a stop adds whatever its speed (dry friction, a force that grows between two readings), stated beside
    the slope, never inside it, so that the stated figure gives back at least every stop's own peak at its own
    speed. n, v0 and v1: how many stops bear the range a number is stated in, and the slowest and the fastest
    of them (m/s): a self-test tap, a stop on a thing known rigid ("rigid": True), or a stop whose own ratio is
    within rel of c. f0, t0: the peak (N) of the slowest of them and the threshold it ran with. poses, bodies
    (at the top): the different poses and bodies the bearing stops of both kinds stand at. No resolved stop: c
    None, n 0. A stop that is only bounded (bound_n) or only read (read_n) bears on nothing: its reading may be
    the tool's own press, which no speed of arrival made."""
    used = _resolved(stops)
    is_tool = lambda s: (s.get("body") or "tool") == "tool"
    out, bear = _one([s for s in used if is_tool(s)], rel)
    link, bear_link = _one([s for s in used if not is_tool(s)], rel)
    every = bear + bear_link
    out.update(seen=len(used), link=link, poses=_poses(every, rel) if every else 0,
               bodies=len({s.get("body") or "tool" for s in every}))
    return out


def _one(used, rel):
    """(the fit of fit() over the resolved stops of one kind, the stops that bear it)."""
    none = {"c": None, "n": 0, "v0": None, "v1": None, "seen": len(used), "f0": None, "e0": 0.0}
    if not used:
        return none, []
    ratio = lambda s: ((s["peak_n"] - (s.get("hid_n") or 0.0) - (s.get("f_thr") or 0.0))
                       / ((s.get("v_least") or s["v"]) * s["model_z"]))
    c = max(ratio(s) for s in used)
    bear = [s for s in used if s.get("where") == "tap" or s.get("rigid") is True or ratio(s) >= (1 - rel) * c]
    if not bear or not c > 0:
        return none, []
    stated = max(1.0, c)                                        # the ratio slope() states the model with
    over = lambda s: s["peak_n"] - (s.get("f_thr") or 0.0) - (1 + rel) * stated * s["v"] * s["model_z"]
    slowest = min(bear, key=lambda s: s["v"])
    poses = [np.asarray(s["q"], dtype=float) for s in bear if s.get("q") is not None]
    poses = [p for p in poses if p.shape == poses[0].shape] if poses else []
    return dict(none, c=float(c), e0=float(max(0.0, max(over(s) for s in used))), n=len(bear),
                v0=float(slowest["v"]), v1=float(max(s["v"] for s in bear)), f0=float(slowest["peak_n"]),
                t0=float(slowest.get("f_thr") or 0.0),
                dirs=[[float(x) for x in s["u"]] for s in bear if s.get("u") is not None],
                q_lo=[float(x) for x in np.min(poses, axis=0)] if poses else None,
                q_hi=[float(x) for x in np.max(poses, axis=0)] if poses else None), bear


def said_of(z, text, fit, q=None, u=None):
    """What the bounds hand over with a model slope at a pose (Slope.said, for rcpx.bounds_strike.ran): the slope
    a figure is stated with there and its text, the range the stops bear it in, the travels and the pose spread
    of the stops that bear it (covers), and the pose q and travel u the model was read at."""
    fit = fit or {}
    keep = ("v0", "f0", "v1", "dirs", "q_lo", "q_hi")
    return dict({k: fit.get(k) for k in keep}, z=z, source=text, e0=fit.get("e0") or 0.0, n=fit.get("n") or 0,
                q=None if q is None else [float(x) for x in np.asarray(q, dtype=float)],
                u=None if u is None else [float(x) for x in np.asarray(u, dtype=float)])


def covers(said, u, q, rel=REL):
    """(True, "") where the stops that bear the fit cover a stop of travel u at pose q: one of them ran within rel
    of that travel (the distance of the two unit vectors) and the pose lies inside their pose spread grown by rel
    rad per joint. Else (False, why). The fit's largest ratio was read along those travels at those poses alone:
    a travel across the support's normal, or a pose nobody tapped, is no part of what it bounds."""
    said = said or {}
    dirs, lo, hi = said.get("dirs") or [], said.get("q_lo"), said.get("q_hi")
    if u is None or not dirs:
        return False, "no stop that bears the fit gives its travel"
    u = np.asarray(u, dtype=float)
    u = u / float(np.linalg.norm(u)) if float(np.linalg.norm(u)) > 0 else u
    if not any(float(np.linalg.norm(u - np.asarray(d, dtype=float))) <= rel for d in dirs):
        return False, "the stop's travel is none the stops that bear the fit ran along"
    if q is None or lo is None or hi is None or len(q) != len(lo):
        return False, "the stop's pose is not known against the pose spread of the stops that bear the fit"
    q = np.asarray(q, dtype=float)
    if np.any(q < np.asarray(lo) - rel) or np.any(q > np.asarray(hi) + rel):
        return False, "the stop's pose lies outside the pose spread of the stops that bear the fit"
    return True, ""


def of_kind(fit, link):
    """The fit a figure of that kind rests on: the link's (fit "link") for a link's point, else the tool's."""
    fit = fit or {}
    return (fit.get("link") or {"c": None, "n": 0, "v0": None, "v1": None, "f0": None, "e0": 0.0}) if link else fit


def _where(fit):
    return (f"{fit['n']} stops at {fit.get('poses') or 1} poses on {fit.get('bodies') or 1} bodies between "
            f"{fit['v0'] * MM:.3g} and {fit['v1'] * MM:.3g} mm/s")


def slope(model_z_here, fit=None, rel=REL):
    """(z in N per m/s, the one term that set it): the slope a figure is stated with and a stated limit is solved
    with (see the module text): (1 + rel) x the body's model here, times c where the stops read over the model.
    A ratio under 1 is never carried: the model stands."""
    if model_z_here is None:
        return None, "unresolved: the body's model gives no stop slope here"
    fit = fit or {}
    c, n = fit.get("c"), fit.get("n") or 0
    if not n or c is None:
        return (1 + rel) * model_z_here, "the body's model (no stop resolved on this body)"
    if c > 1:
        return (1 + rel) * c * model_z_here, f"fitted: {_where(fit)}, {c:.3g} times the body's model"
    return (1 + rel) * model_z_here, (f"fitted: {_where(fit)} read at or under the body's model ({c:.3g} times it): "
                                      "the model stands (a ratio under 1 was read at the taps' own pose and travel: "
                                      "it is no bound at another)")


def estimate(model_z_here, fit=None, rel=REL):
    """(z in N per m/s, what it rests on) for SOLVING a speed from a force that is no stated limit (see the module
    text): (1 + rel) x c x the body's model here, c also under 1: what this body's stops showed, carried from
    their own pose and travel. Where the stops read over the model, or none is resolved, it is slope()."""
    z, text = slope(model_z_here, fit, rel)
    c = (fit or {}).get("c")
    if z is None or not (fit or {}).get("n") or c is None or c > 1:
        return z, text
    return (1 + rel) * c * model_z_here, (f"fitted: {_where(fit)}, {c:.3g} times the body's model, carried from the "
                                          "taps' own pose and travel: for a speed that keeps no stated limit; the "
                                          "figure a motion states never goes under the model")


def solving(z, text, fit, v_tell):
    """(z, text) for SOLVING a speed from a force (an arrival under a limit or an ending force), where the caller
    solves level + z x v and knows no additive term and no floor. Two things the stops showed are carried in the
    slope: what they stood over the line whatever their speed (fit e0), at the tell speed, the slowest an arrival
    runs; and the slowest stop's own slope, (f0 - its threshold) / v0, so that a speed solved under that stop's
    speed is not faster than that stop itself allows. Neither applies: unchanged."""
    fit = fit or {}
    if z is None:
        return z, text
    e0, v0, f0 = float(fit.get("e0") or 0.0), fit.get("v0"), fit.get("f0")
    carried = z + e0 / v_tell if e0 > 0 and v_tell and math.isfinite(v_tell) else z
    own = (f0 - float(fit.get("t0") or 0.0)) / v0 if v0 and f0 is not None else 0.0
    if max(carried, own) <= z:
        return z, text
    return max(carried, own), text + "; for solving a speed: not under the slowest stop's own slope and what the stops showed over that line"


def t_det(stops, dt, readings=QUANT):
    """(seconds, text, n): the largest detection time the stops showed; with no stop, the guard's own readings.
    A stop with no reading before its contact (det_periods None) shows none."""
    seen = [int(s["det_periods"]) for s in stops or () if s.get("det_periods")]
    if not seen:
        return readings * dt, "model: the guard's own readings", 0
    return max(seen) * dt, f"measured: the largest of {len(seen)} stops, {max(seen)} control periods", len(seen)


def model_terms(m_eff, b_per_s, k_servo, t_det_s, lag_s):
    """{"impact", "servo"} in N per m/s: the contact model's impact term (b x m_eff; 0 with no contact model) and
    the servo's term (its stiffness times the detection time plus the lag)."""
    impact = float(b_per_s) * float(m_eff) if b_per_s is not None and m_eff is not None and math.isfinite(m_eff) else 0.0
    servo = float(k_servo) * (float(t_det_s) + float(lag_s or 0.0)) if k_servo is not None and math.isfinite(k_servo) else 0.0
    return {"impact": impact, "servo": servo}


def model_z(m_eff, b_per_s, k_servo, t_det_s, lag_s):
    """The body's model of the stop slope (N per m/s, no margin). None when neither term is known."""
    terms = model_terms(m_eff, b_per_s, k_servo, t_det_s, lag_s)
    z = terms["impact"] + terms["servo"]
    return z if z > 0 else None


def tapped(heights_m, n_ladder, error_m, rigid_m):
    """(used, why not): a ladder of taps is used only when it holds at least QUANT + 1 taps, every descent met
    something, the tool came to rest at
    the stated support within the support's error (heights_m: the tool's lowest point over the stated plane at
    rest after each tap) and the rest heights repeat within the tip's resolution over the ladder. Else the text
    says what height was met."""
    heights = [float(h) for h in heights_m]
    if n_ladder < QUANT + 1:
        return False, (f"no tap is used: a ladder of {n_ladder} cannot show that the rest height repeats "
                       f"({QUANT + 1} taps are the least)")
    if not heights or len(heights) < n_ladder:
        return False, (f"no tap is used: {len(heights)} of {n_ladder} descents met something; the others ran "
                       f"{error_m * MM:.2f} mm past the stated support and felt nothing")
    if max(abs(h) for h in heights) > error_m:
        return False, (f"no tap is used: the tool came to rest {min(heights) * MM:.2f} to {max(heights) * MM:.2f} mm "
                       f"over the stated support, outside its error of {error_m * MM:.2f} mm: what was met is not the "
                       "support")
    if max(heights) - min(heights) > rigid_m:
        return False, (f"no tap is used: the rest heights spread over {(max(heights) - min(heights)) * MM:.3f} mm "
                       f"across the ladder, more than the tip's resolution of {rigid_m * MM:.3f} mm: what was met "
                       "gives way")
    return True, ""
