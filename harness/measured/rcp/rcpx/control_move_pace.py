# SPDX-License-Identifier: AGPL-3.0-only
"""Pacing and the end of a move.

A move ends where it says. The command of a free leg (and of the fast part of a contact motion) never leads the
measured arm by more than the body's own lag at the speed the command has there: the stored record's lag (B.lag,
measured outside a task) or, where this move's own first periods show a longer one, that (a cold arm paces by
what it shows itself). While the next sample would lead by more and the arm is still closing on the command, the
command holds. The arm is read at all its moving points (both ends of every capsule and the tip), so a turn of
the wrist about a standing tip counts as travel. Nothing is typed: the band is the body's still band (the tip
travel its own give explains under a force it cannot tell from none, B.rigid, at the hold's own level while it
holds something), the lag is measured, the window is the detection time.

So a plan that brakes its command to a stand has its last samples wait for the arm, and no wait is added after
them (owed waits stay owed to nobody). When the command stands at its end the move ends

- "goal" as soon as the arm will be inside its still band of that end one detection time on (the test a wait
  uses); or once its farthest point has PASSED its end with no reading over the contact threshold of its
  speed (a push met while it closes is judged when it stands) and stands no further from it than its rest band
  (the same give along its softest direction, with what its joints' dry friction can hide: the body's own rest
  error) plus its own lag at the speed it still has (the lag this move's own first periods showed; a move that
  showed none: the stored record's): what is left is its own ringing about the end; or once it STANDS (its
  motion over one detection time is under the still band) inside that rest band. No share of the lead the move
  ran with counts: an arm that was held back says so, however far its command led it on the way;
- "short" when it has stood for the detection time (a turning point of its own ringing does not last that
  long) further off than that, or the call's own time is spent there: told_mm becomes the MEASURED pose (the
  command is set onto it, no control period), behind_mm and why are said, and the next line starts from where
  the arm is.

While the arm brakes onto the standing command only the guard's cap judges (the force estimate reads the braking
itself: tests/test_r6_move_fix.py test_braking_is_no_contact), and once it stands the contact threshold of an arm
at rest (control_tail.standing, as every wait of C10): a contact that still stands then ends the move as any
other. After the first loaded reading the command is never held here: the lead is then the force law's.
The record of the move (ex._pace) says how many control periods the command held and where the arm stood; the
receipt carries it only when it changed what the move did. No robot is named here.
"""

import math

import numpy as np

from rcpx import control_tail
from rcpx.control_base import MM, _ratio, _val


class Pace:
    """One run of a path under pacing. holds(k, s, g): whether sample k must wait for the arm (the caller then
    sends the standing command again and counts n_held); close(s, g, recent): the end of the run, (stop or None,
    last reading), with verdict "goal" or "short" when no guard stop ended it."""

    def __init__(self, ex, q_from, path, close=True):
        self.ex, self.path = ex, path
        self.closes = bool(close) and bool(len(path))
        self.n_held, self.tau, self.verdict, self.why = 0, 0.0, None, ""
        self.band = self.left = self.v_end = self.lag_end = self._band = None
        self.lead = 0.0                        # the largest lead the command had over the arm in this run (m)
        self._pts = self._speeds = None
        self._rising = True
        self._best, self._since = math.inf, 0
        self._q_from = np.asarray(q_from, dtype=float)

    # ------------------------------------------------------------------ what the plan and the body say
    def _points(self, q):
        return np.asarray(self.ex._robot_points(np.asarray(q, dtype=float))[0], dtype=float)

    def _plan(self):
        """The command's moving points at every sample and the speed the fastest of them has there (m, m/s)."""
        if self._pts is None:
            pts = [self._points(q) for q in [self._q_from] + list(self.path)]
            self._pts = pts[1:]
            self._speeds = [float(np.max(np.linalg.norm(b - a, axis=1))) / float(self.ex.dt)
                            for a, b in zip(pts, pts[1:])]
        return self._pts, self._speeds

    def _bands(self, q, s=None):
        """(the still band, the rest band) of this run (m), read once at the pose q. The still band is C10's own:
        the tip travel the arm's give explains along the tool under a force it cannot tell from none (the level
        the hand rests with: the hold's own while something is held); an arm whose motion over one detection time
        is under it STANDS. The rest band is how far a standing arm may rest off its command by its own doing:
        the same give along the direction in which its servo is softest (the base's and the tool's own axes),
        plus the dead band its joints' dry friction leaves a joint servo (each joint stands still anywhere its
        servo pushes less than its friction: friction over the joint's own gain, carried to the tip by that
        joint's lever; the body's own description)."""
        if self._band is None:
            ex, B = self.ex, self.ex.bounds
            hold = control_tail.control_expect._hold_module()
            level = float(hold.level(ex)) if hold is not None and hold.held(ex) is not None else _val(B.f_thr(0.0, 0.0))
            R = np.asarray(ex.model.fk_tip(q)[1], dtype=float)
            self._band = (_val(B.rigid(q, R[:, 0], level)),
                          max(_val(B.rigid(q, u, level)) for u in np.vstack([np.eye(3), R.T])) + self._dry(q))
        return self._band

    def _dry(self, q):
        """How far the tip may stand off its command by the joints' dry friction alone (m): per joint its friction
        over its servo gain, times the length of its own column of the tip's Jacobian, summed (an upper bound
        over every direction). 0 for a body whose description states no friction or no joint gain."""
        ex = self.ex
        fric = np.asarray(getattr(ex, "_frictionloss", ()), dtype=float)
        try:
            servo = ex.bounds.robot.servo_law()
            kp = servo.get("kp") if isinstance(servo, dict) and servo.get("kind") == "joint_pd" else None
            kp = np.asarray(ex.bounds.robot.kp if kp is None else kp, dtype=float)
        except Exception:                        # no joint gain stated: no dead band can be derived
            return 0.0
        J = np.asarray(ex.model.jac(np.asarray(q, dtype=float))[:3], dtype=float)
        if fric.shape != (J.shape[1],) or kp.shape != fric.shape or not np.all(kp > 0.0):
            return 0.0
        return float(np.sum(np.linalg.norm(J, axis=0) * fric / kp))

    def _lag(self, v, own=False):
        """The body's lag at the speed v (m), with the accepted relative error of a measured quantity. In motion:
        the stored record's or what this move showed, the larger (pacing never slows an arm that runs as it was
        measured to). own: at the end, what THIS move's own first periods showed where they showed one (the arm
        as it ran here), else the stored record's."""
        stored = float(_val(self.ex.bounds.lag(float(v))) or 0.0)
        seen = self.tau * float(v)
        return (1.0 + _ratio("REL")) * ((seen if seen > 0.0 else stored) if own else max(stored, seen))

    def _read(self, s):
        """(the arm's moving points now, their travel in one control period at the joint speeds it reads)."""
        now = self._points(s.q)
        return now, self._points(np.asarray(s.q, dtype=float) + np.asarray(s.qd, dtype=float) * float(self.ex.dt)) - now

    def _at_rating(self, s):
        """True while a joint runs at its own torque rating (within the accepted relative error): the plan's own
        torque limited stretch. The command does not stand for an arm that already gives all it has (the guard's
        estimate would read the standing command as a push from outside); the end of the move waits for it."""
        lim = getattr(self.ex, "torque_limit", None)
        if lim is None or getattr(s, "tau", None) is None:
            return False
        lim = np.asarray(lim, dtype=float) * float(self.ex.cfg["torque_frac"])
        tau = np.abs(np.asarray(s.tau, dtype=float))
        return tau.shape == lim.shape and bool(np.any(tau >= (1.0 - _ratio("REL")) * lim))

    # ------------------------------------------------------------------ in motion
    def holds(self, k, s, g):
        """True when sample k of the path would lead the measured arm by more than the body's lag at the speed
        the command has there (and the sample's own travel in one control period: the reading is one period
        old), and the arm is still closing on the command that stands. Never before the first
        reading, never once a reading is over the guard's threshold (the lead is then the force law's), never
        while the arm stands or makes no progress (a command that waits for an arm that does not come would
        never end: the plan goes on and the guard judges what holds the arm)."""
        if s is None or not self.closes:
            return False
        now, step = self._read(s)
        if getattr(g, "n_force", 0) > 0 or getattr(g, "n_cap", 0) > 0 or self._at_rating(s):
            return False
        pts, speeds = self._plan()
        speed = float(np.max(np.linalg.norm(step, axis=1))) / float(self.ex.dt)
        was = self._points(self._q_from) if k == 0 else pts[k - 1]
        behind = float(np.max(np.linalg.norm(was - now, axis=1)))
        if self._rising and k > 0:                # this move's own first periods: while its command speeds up
            if speed * _val(self.ex.bounds.t_det()) > self._bands(self.path[-1], s)[0]:
                self.tau = max(self.tau, behind / speed)
            self._rising = speeds[k] > speeds[k - 1]
        lead = float(np.max(np.linalg.norm(pts[k] - now, axis=1)))
        band, self.lead = self._bands(self.path[-1], s)[0], max(self.lead, behind)
        step = speeds[k] * float(self.ex.dt)          # the sample's own travel: the arm answers one period on
        if lead <= band + self._lag(speeds[k]) + step or not speed * _val(self.ex.bounds.t_det()) > band:
            self._best, self._since = math.inf, 0     # inside its lag, or the arm stands: the command goes on
            return False
        return self._closing(behind)

    def _closing(self, left):
        """Whether the arm still closes on the command that stands: over the detection time (at least the
        distinct readings a resolution needs) the distance left fell."""
        if left < self._best:
            self._best, self._since = left, 0
            return True
        self._since += 1
        ex = self.ex
        return self._since < max(int(_ratio("QUANT")), int(math.ceil(_val(ex.bounds.t_det()) / float(ex.dt))))

    # ------------------------------------------------------------------ the end
    def close(self, s, g, recent):
        """The command stands at its end: hold it while the arm closes. Returns (stop, reading): stop None when
        the move ended by pacing (self.verdict is then "goal" or "short"), else the guard's word."""
        ex = self.ex
        if not self.closes or s is None:
            return None, s
        q_end = np.asarray(self.path[-1], dtype=float)
        end = self._plan()[0][-1]
        t_det, dt = _val(ex.bounds.t_det()), float(ex.dt)
        rest = control_tail.rest_levels(ex, ex._held_for(None))
        most = len(self.path) + ex._wait_limit()          # a call always returns: the plan's own length and a wait's
        was, ex._period_kind = ex._period_kind, "waiting"
        stop, n, first, tau, cap = None, 0, None, g.tau, g.cap_n
        still, need, lead_at, gap0 = 0, max(int(_ratio("QUANT")), int(math.ceil(t_det / dt))), None, None
        try:
            while True:
                now, step = self._read(s)
                gap = end - now
                left = float(np.max(np.linalg.norm(gap, axis=1)))
                if first is None:                             # the lag it had when its command came to stand
                    first, lead_at, gap0 = left, int(np.argmax(np.linalg.norm(gap, axis=1))), gap.copy()
                ahead = float(np.max(np.linalg.norm(gap - step * (t_det / dt), axis=1)))
                speed = float(np.max(np.linalg.norm(step, axis=1))) / dt
                self.band, rest_band = self._bands(q_end, s)
                self.left, self.v_end = left, speed
                self.lag_end = (rest_band - self.band) + self._lag(speed, True)   # no share of the lead it ran with
                allowed = self.band + self.lag_end
                # its farthest point has passed its end, and nothing reads over the contact threshold of its speed
                reached = (float(gap[lead_at] @ gap0[lead_at]) <= 0.0
                           and float(getattr(s, "Fmag", 0.0) or 0.0) <= _val(ex.bounds.f_thr(speed)))
                if ahead <= self.band or (reached and left <= allowed):
                    self.verdict = "goal"                     # there, or past its end and inside its own lag: what
                    break                                     # is left is its own ringing about the end
                stands = speed * t_det <= self.band
                still = still + 1 if stands else 0
                if (stands and left <= allowed) or still >= need or n >= most:
                    # it stands inside its rest band, or it has stood for the detection time (a turning point of
                    # its own ringing does not last that long), or the call's own time is spent
                    self.verdict = "goal" if left <= allowed else "short"
                    if self.verdict == "short":
                        self.why = (f"the arm stands {left * MM:.1f} mm short of the end it was told to reach: it "
                                    f"did not make up the {first * MM:.1f} mm it was behind when its command came "
                                    f"to stand (its rest band and its own lag explain {allowed * MM:.1f} mm); "
                                    "nothing reads over the contact threshold")
                    break
                s = ex._period(q_end)
                n += 1
                self.n_held += 1
                if recent is not None:
                    recent.append((s, q_end))
                # the cap alone while it brakes, the rest level once it stands; while a joint brakes at its own
                # rating onto the standing command the force estimate reads that braking: no cap from outside
                g.tau = tau if control_tail.standing(ex, g, s, rest) else None
                g.cap_n = math.inf if self._at_rating(s) else cap   # its own braking at its rating is no push
                stop = g.check(s)
                if stop:
                    break
        finally:
            ex._period_kind, g.tau, g.cap_n = was, tau, cap
        if stop is None and self.verdict == "short":
            ex._q_des = np.asarray(s.q, dtype=float).copy()   # told is where the arm is: the next line starts there
        return stop, s

    def record(self, s):
        """What the move's receipt may carry (ex._pace): only a move whose command held says so."""
        ex = self.ex
        mm = lambda x: None if x is None else round(float(x) * MM, 2)
        ex._pace_last = {"held_periods": int(self.n_held), "ended": self.verdict, "band_mm": mm(self.band),
                         "left_mm": mm(self.left), "v_end_mm_s": mm(self.v_end), "lag_mm": mm(self.lag_end),
                         "tau_s": round(self.tau, 4)}       # every run's, for a reader of the session (a test)
        if not self.n_held and self.verdict != "short":
            ex._pace = None
            return
        ex._pace = {k: ex._pace_last[k] for k in ("held_periods", "ended", "band_mm", "left_mm", "v_end_mm_s")}
        ex._pace["source"] = ("the body's still band, its lag and its detection time; the command held while the "
                              "arm closed on it")
        if self.verdict == "short":
            ex._pace["why"] = self.why


def allowance(ex, q0, path):
    """The control periods pacing may add to a path's own count (the budget counts them): the arm closes on
    its standing command by e in its own time constant, and the move ends once what is left is inside its still
    band. The time constant is the one this session's own last paced move showed (tau_s: its lag over its
    speed); a session that has shown none yet counts none. 0 for an empty path."""
    tau = float((getattr(ex, "_pace_last", None) or {}).get("tau_s") or 0.0)
    if not len(path) or not tau > 0.0:
        return 0
    tips = [np.asarray(ex.model.fk_tip(np.asarray(q, dtype=float))[0], dtype=float) for q in [q0] + list(path)]
    v = max(float(np.linalg.norm(b - a)) for a, b in zip(tips, tips[1:])) / float(ex.dt)
    band = float(ex._band_at(path[-1]))
    lag = tau * v
    if not lag > band or not band > 0.0:
        return 0
    return int(math.ceil(tau * math.log(lag / band) / float(ex.dt)))


def keys(ex):
    """{"pace": ...} for the receipt of a move whose command held (a block enters a result only when it
    changed what the tool did), else {}. Read once: the next move writes its own."""
    pace = getattr(ex, "_pace", None)
    ex._pace = None
    return {"pace": pace} if pace else {}
