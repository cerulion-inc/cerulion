# SPDX-License-Identifier: AGPL-3.0-only
"""Planning of the Executor: rotation helpers, the resolved-rate step, Cartesian and joint plans, the
support under the robot and the path check.

A Cartesian move is planned BEFORE motion by resolved-rate steps through model.step_toward (every joint in the solve)
along a straight line. A plan is the robot's own fastest: each joint within its speed limit and its acceleration at
the pose (both asked of the session's bounds), so a ramp is v / a and a stated speed only lowers it; the bounds then
slow it where they have reason to (control_speed.py). The whole plan is path checked (the links' capsules against
the surface the robot stands on and against each other, each with the cross track error measured at rest, never a
term that grows with the planned speed or with a thing's position doubt; sampled no
coarser than half the smallest margin) and a failed check is a refusal that changes nothing. The TIP is never
refused for the surface: a line planned through it is cut there and run to contact (control_expect
.cut_at_support). Holding orientation along a line means, by
the arm: the whole hand orientation with six or more joints; the finger's pointing direction with fewer ("axis");
and, when such an arm cannot keep that direction on the line (its finger turns with the tip's bearing), the finger's
pitch only ("pitch"), with the turn reported. No robot is named here.
"""

import math

import numpy as np

from rcpx.bounds_ratios import GIVE
from rcpx.context import refusal
from rcpx.model import ROT_WEIGHT_M

MM = 1000.0   # unit: mm per m
UP = np.array([0.0, 0.0, 1.0])
TINY = np.finfo(float).tiny
LINE_TOL_SHARE = 0.002    # numeric: a planned line may leave its line by this share of the arm's length
LINE_TOL_RAD = math.radians(2.0)    # numeric: ... and its orientation by this
END_TOL_M = 0.001         # numeric: m; the end of a plan lies this near its goal
END_TOL_RAD = math.radians(1.0)     # numeric
POLISH_STEPS = 30         # numeric: extra waypoints that bring the end of a plan onto its goal
POLISH_DONE_M, POLISH_DONE_RAD = 1e-4, 1e-3   # numeric: the polish ends this near
SHRINK = 0.9              # numeric: a step that would pass a joint's speed limit is shrunk to this share of it
GROW = 1.25               # numeric: the next step's first guess, as a share of the last one
SHRINK_TRIES = 4          # numeric
SUB_PER_PERIOD = 2        # numeric: pieces a full-speed control period is cut into when a path is timed
NEWLY = 1e-4              # numeric: m; "nearer than it started" means nearer by more than this
MEETS = ("touch", "press", "lower", "approach", "close", "follow", "explore")   # kinds whose end may meet a thing


def rot_from_vec(v):
    """Rotation matrix of a rotation vector (axis * angle)."""
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def vec_from_rot(R):
    """Rotation vector of a rotation matrix."""
    R = np.asarray(R, dtype=float)
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    a = math.acos(c)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if a < 1e-9:
        return 0.5 * w
    if math.pi - a < 1e-6:  # half turn: axis from the symmetric part
        M = (R + np.eye(3)) / 2.0
        k = M[:, int(np.argmax(np.diag(M)))]
        return a * k / np.linalg.norm(k)
    return a * w / (2.0 * math.sin(a))


def profile(t, T, ramp):
    """Trapezoid progress 0..1 at time t for cruise time T and ramps of `ramp` at both ends (total T + ramp)."""
    if ramp <= 0:
        return min(1.0, max(0.0, t / T))
    if t <= 0:
        return 0.0
    if t < ramp:
        return 0.5 * t * t / (ramp * T)
    if t < T:
        return (t - ramp / 2.0) / T
    if t < T + ramp:
        return 1.0 - 0.5 * (T + ramp - t) ** 2 / (ramp * T)
    return 1.0


def _axes(R):
    return {"x": R[:, 0], "y": R[:, 1], "z": R[:, 2]}


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class PlanMixin:
    # ------------------------------------------------------------------ planning
    def _axis_only(self, drot):
        """An arm with fewer than six joints cannot hold a full orientation while it moves: holding orientation
        then means holding the finger's pointing axis (tool x), with the roll about it left free."""
        return self.model.n < 6 and drot is None

    @staticmethod
    def _pitch_axis(x):
        """Unit axis a turn about which raises the finger x (None for a finger that stands vertical)."""
        h = np.cross(x, UP)
        n = float(np.linalg.norm(h))
        return None if n < 1e-3 else h / n

    def _can_turn(self, q):
        """Projector (3 x 3, frame B) onto the turns the tool can make at q with the tip held still: all of them
        with six or more joints, a plane (or a line) with fewer."""
        J = self.model.jac(q)
        _, sv, vt = np.linalg.svd(J[:3])
        free = vt[int(np.sum(sv > 1e-6 * sv[0])):].T          # joint motions that leave the tip where it is
        if free.size == 0:
            return np.zeros((3, 3))
        u, sa, _ = np.linalg.svd(J[3:] @ free, full_matrices=False)
        u = u[:, sa > 1e-6 * max(float(sa[0]), 1e-12)]
        return u @ u.T

    def _turn_words(self, q):
        """(can, cannot): the tool-frame turns (roll about tool x, pitch about tool y, yaw about tool z) the arm
        can and cannot make at q with the tip held."""
        P, R = self._can_turn(q), self.model.fk_tip(q)[1]
        share = {name: float(np.linalg.norm(P @ R[:, i])) for i, name in enumerate(("roll", "pitch", "yaw"))}
        return [k for k, v in share.items() if v > 0.9], [k for k, v in share.items() if v < 0.3]

    def _rot_err(self, R_des, R, hold):
        """Rotation still to make (rotation vector, frame B) under a hold: falsy = the whole orientation, "pitch" =
        only the finger's angle to the vertical, "can" = the whole orientation, of which _step and _track then
        take the part the arm can make with the tip held, any other truthy value = the finger's direction."""
        if R_des is None:
            return None
        if hold == "can":
            return vec_from_rot(R_des @ R.T)
        if hold == "pitch":
            h = self._pitch_axis(R[:, 0])
            if h is None:
                return np.zeros(3)
            return (math.asin(max(-1.0, min(1.0, R_des[2, 0]))) - math.asin(max(-1.0, min(1.0, R[2, 0])))) * h
        if hold:
            return np.cross(R[:, 0], R_des[:, 0])
        return vec_from_rot(R_des @ R.T)

    def _step(self, q, e_p, e_r, hold):
        if e_r is None or not hold:
            return self.model.step_toward(q, e_p, e_r)
        J = self.model.jac(q)
        x = self.model.fk_tip(q)[1][:, 0]
        if hold == "pitch":   # only the turn that changes the finger's pitch is asked for
            h = self._pitch_axis(x)
            P = np.zeros((3, 3)) if h is None else np.outer(h, h)
        elif hold == "can":
            P = self._can_turn(q)
        else:
            P = np.eye(3) - np.outer(x, x)  # rotation about the finger axis is not asked for
        Js = np.vstack([J[:3], ROT_WEIGHT_M * (P @ J[3:])])
        e = np.concatenate([e_p, ROT_WEIGHT_M * (P @ e_r)])
        q = self.model.clip(q)
        return self.model.clip(q + self.model._dls(Js, e, q)) - q

    def _joint_step(self):
        """The largest change of each joint in one control period: its speed limit x the period (rad)."""
        return np.asarray(self.bounds.qd_max(), dtype=float) * self.dt

    def _track(self, q, p_des, R_des, hold=False):
        p, R = self.model.fk_tip(q)
        dq = self._step(q, p_des - p, self._rot_err(R_des, R, hold), hold)
        big = float(np.max(np.abs(dq) / self._joint_step())) if dq.size else 0.0
        if big > 1.0:
            dq = dq / big
        q2 = self.model.clip(q + dq)
        p2, R2 = self.model.fk_tip(q2)
        er = self._rot_err(R_des, R2, hold)
        if er is not None and hold == "can":
            er = self._can_turn(q2) @ er
        return q2, float(np.linalg.norm(p_des - p2)), 0.0 if er is None else float(np.linalg.norm(er))

    def _plan_hold(self, q0, dp, drot, speed, keep, timed=True):
        """_plan_cart under the hold this arm can keep on this line: (path, planned stop, why, hold). An arm with
        fewer than six joints holds its finger's direction; when that plan cannot run the line (the finger of
        such an arm turns with the tip's bearing) the finger's pitch alone is held, if that goes further. A turn
        asked of such an arm (drot) is planned for the part it can make with the tip held ("can")."""
        if drot is not None and self.model.n < 6:
            return (*self._plan_cart(q0, dp, drot, speed, keep, "can", timed=timed), "can")
        hold = self._axis_only(drot)
        path, pstop, pwhy = self._plan_cart(q0, dp, drot, speed, keep, hold, timed=timed)
        if keep and hold and pstop == "limit" and not isinstance(path, dict):
            alt = self._plan_cart(q0, dp, drot, speed, keep, "pitch", timed=timed)
            if not isinstance(alt[0], dict) and (alt[1] == "goal" or len(alt[0]) > len(path)):
                return alt[0], alt[1], alt[2], "pitch"
        return path, pstop, pwhy, hold

    def _finger_keys(self, R_from, R_to, hold):
        """Result keys on where the finger points after a move: finger_dir always; the turn and a note when the
        arm could only keep the finger's pitch."""
        out = {"finger_dir": [round(float(v), 3) for v in R_to[:, 0]]}
        if hold == "pitch":
            turn = math.degrees(math.acos(max(-1.0, min(1.0, float(R_from[:, 0] @ R_to[:, 0])))))
            out["finger_turned_deg"] = round(turn, 1)
            out["finger_note"] = ("this arm cannot keep its finger's direction along this line (its finger turns "
                                  f"with the tip's bearing): the finger's pitch was held and its direction turned "
                                  f"{turn:.1f} deg; finger_dir is where it points now")
        return out

    def _joint_keys(self, q):
        """q_deg (the joints after the move, by name) and limit_margin_deg (the joint nearest a limit, how far, and
        which way, plus or minus, turns it away from that limit) for motion results (a round one surface
        has no read_state)."""
        m = self.model
        lo = np.degrees(np.asarray(q) - m.q_lo)
        hi = np.degrees(m.q_hi - np.asarray(q))
        i = int(np.argmin(np.minimum(lo, hi)))
        free = "minus" if hi[i] < lo[i] else "plus"
        return {"q_deg": {n: round(math.degrees(float(v)), 2) for n, v in zip(m.joint_names, q)},
                "limit_margin_deg": {"joint": m.joint_names[i], "deg": round(float(min(lo[i], hi[i])), 2),
                                     "away_from_limit": free}}

    def _limit_why(self, q):
        step = self._joint_step()   # a joint that would reach its limit inside one control period is at it
        near = [self.model.joint_names[i] for i in range(self.model.n)
                if min(q[i] - self.model.q_lo[i], self.model.q_hi[i] - q[i]) < step[i]]
        if near:
            return "joint " + ", ".join(near) + " at its limit"
        return "the line leaves the arm's reach or passes a singularity here"

    def _plan_cart(self, q0, dp, drot, speed, keep, hold=None, timed=True):
        """(path, planned stop, why) or a refusal. timed (a caller that streams the path itself): one joint target
        per control period, from rest to rest, at the robot's own full speed along the line (speed, m/s: a lower
        tip speed the caller states; None: none). Not timed (the core's own moves, which the bounds time): the
        line's joint waypoints, one per control period of full speed. hold: see _rot_err (None: what _axis_only
        says)."""
        p0, R0 = self.model.fk_tip(q0)
        L = float(np.linalg.norm(dp))
        A = 0.0 if drot is None else float(np.linalg.norm(drot))
        if L <= 0.0 and A <= 0.0:
            return [], "goal", ""
        R_goal = R0 if drot is None else rot_from_vec(drot) @ R0
        ax = self._axis_only(drot) if hold is None else hold
        way, q = [], np.asarray(q0, dtype=float).copy()
        scale = float(self.model.length_scale)
        cap = self._joint_step()
        sk, inc = 0.0, 1.0
        stop, why = "goal", ""
        while sk < 1.0:
            inc = min(1.0 - sk, inc * GROW)
            for _ in range(SHRINK_TRIES):   # a step that would turn a joint faster than its limit advances less
                s1 = min(1.0, sk + inc)
                p_des = p0 + s1 * dp
                R_des = None if not keep else (R0 if drot is None else rot_from_vec(s1 * drot) @ R0)
                p, R = self.model.fk_tip(q)
                dq = self._step(q, p_des - p, self._rot_err(R_des, R, ax), ax)
                big = float(np.max(np.abs(dq) / cap)) if dq.size else 0.0
                if big <= 1.0:
                    break
                inc *= SHRINK / big
            if inc <= np.finfo(float).eps:
                stop, why = "limit", self._limit_why(q)
                break
            q1, e, er = self._track(q, p_des, R_des, ax)
            if e > LINE_TOL_SHARE * scale or er > LINE_TOL_RAD:
                # the solve does not reach a waypoint one full-speed period on (a fast arm near a joint limit
                # or a turn of its wrist): the same line at half the step (GIVE); the line is called out of
                # reach only where no step a float can tell from none is reached (the check above)
                inc *= GIVE / GROW
                continue
            sk, q = min(1.0, sk + inc), q1
            way.append(q.copy())
        if stop == "goal":
            p_goal = p0 + dp
            for _ in range(POLISH_STEPS):  # polish the end of the plan onto the goal
                q, e, er = self._track(q, p_goal, R_goal if keep else None, ax)
                way.append(q.copy())
                if e < POLISH_DONE_M and er < POLISH_DONE_RAD:
                    break
            if e > END_TOL_M or er > END_TOL_RAD:
                stop, why = "limit", self._limit_why(q)
        return (self._time_path(q0, way, tip_speed=speed) if timed else way), stop, why

    def _thin(self, q0, path):
        """The waypoints of a path that was streamed (one pose per control period, crowded where it ran slowly) as
        the bounds want a path's shape: no two nearer than one control period of full speed."""
        step = self._joint_step()
        out, last = [], np.asarray(q0, dtype=float)
        for q in path[:-1]:
            q = np.asarray(q, dtype=float)
            if float(np.max(np.abs(q - last) / step)) >= 1.0:
                out.append(q)
                last = q
        if len(path) and np.any(np.asarray(path[-1], dtype=float) != last):
            out.append(np.asarray(path[-1], dtype=float))
        return out

    def _slow(self, q0, path, share):
        """A timed path run at a lower share of its speed: the same poses in order, 1 / share as many periods."""
        if share is None or share >= 1.0 or not path:
            return path
        pts = np.asarray([np.asarray(q0, dtype=float)] + [np.asarray(q, dtype=float) for q in path])
        n = int(math.ceil(len(path) / max(float(share), np.finfo(float).eps)))
        x = np.linspace(0.0, len(path), n + 1)[1:]
        k = np.minimum(len(path) - 1, np.floor(x).astype(int))
        w = (x - k)[:, None]
        return list(pts[k] * (1.0 - w) + pts[k + 1] * w)

    def _plan_joint(self, q0, q1, speed_frac=None, timed=True):
        """The straight joint line from q0 to q1. timed: one joint target per control period, from rest to rest,
        at the robot's own full speed (speed_frac: a lower share of it the caller states; None: none); not timed:
        its waypoints, one per control period of full speed (the bounds time them)."""
        q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
        dq = q1 - q0
        full = float(np.max(np.abs(dq) / self._joint_step())) if dq.size else 0.0   # periods at full speed
        if full <= 0.0:
            return []
        n = int(math.ceil(full))
        way = [q0 + dq * (k / n) for k in range(1, n + 1)]
        return self._time_path(q0, way, share=speed_frac) if timed else way

    def _time_path(self, q0, way, tip_speed=None, share=None):
        """One joint target per control period along the joint waypoints `way`, from rest at q0 to rest at the last
        one, as fast as the robot's own joints allow: each within its speed limit and its acceleration at the pose
        (asked of the bounds), so a ramp is v / a and a corner is taken at the speed whose change the joints can
        make in one period. tip_speed (m/s) and share (of full speed) only lower it. The path is never re-shaped."""
        B, dt = self.bounds, self.dt
        pts = [np.asarray(q0, dtype=float)]
        for q in way:
            q = np.asarray(q, dtype=float)
            if np.any(q != pts[-1]):
                pts.append(q)
        if len(pts) < 2:
            return []
        qd = np.asarray(B.qd_max(), dtype=float)
        if share is not None:
            qd = qd * min(1.0, max(float(share), np.finfo(float).eps))
        lo = np.minimum.reduce([np.asarray(B.qdd_max(pts[i]), dtype=float) for i in (0, len(pts) // 2, -1)])
        # a joint the model gives no acceleration to spare at a pose still gets what takes it from rest to its
        # speed limit over its whole range (the slowest ramp its own geometry defines)
        span = np.maximum(np.asarray(self.model.q_hi, dtype=float) - np.asarray(self.model.q_lo, dtype=float), TINY)
        lo = np.maximum(lo, qd * qd / (2.0 * span))              # numeric: v^2 = 2 a d
        fine = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):   # pieces short against a control period, so the ramps resolve
            n = max(SUB_PER_PERIOD, int(math.ceil(SUB_PER_PERIOD * float(np.max(np.abs(b - a) / qd)) / dt)))
            fine.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
        P = np.asarray(fine)
        D = np.diff(P, axis=0)
        T = np.max(np.abs(D) / qd, axis=1)                       # s each piece takes at full speed
        if tip_speed is not None and tip_speed > 0.0:
            tips = np.asarray([self.model.fk_tip(q)[0] for q in P])
            T = np.maximum(T, np.linalg.norm(np.diff(tips, axis=0), axis=1) / float(tip_speed))
        T = np.maximum(T, TINY)
        V = D / T[:, None]                                       # joint speeds on each piece at full speed
        with np.errstate(divide="ignore", over="ignore"):       # a joint that stands still bounds nothing (inf)
            rate = np.min(lo / np.abs(V), axis=1)                # 1/s: how fast the share of full speed may change
        N = len(T)
        f = np.ones(N + 1)
        f[0] = f[N] = 0.0
        jump = np.abs(np.diff(V, axis=0))                        # the change of joint speed a corner asks at once
        with np.errstate(divide="ignore", over="ignore"):
            f[1:N] = np.minimum(1.0, np.min(lo * dt / jump, axis=1))
        for k in range(1, N + 1):
            f[k] = min(f[k], math.sqrt(f[k - 1] ** 2 + 2.0 * rate[k - 1] * T[k - 1]))   # numeric: v^2 = 2 a d
        for k in range(N - 1, -1, -1):
            f[k] = min(f[k], math.sqrt(f[k + 1] ** 2 + 2.0 * rate[k] * T[k]))           # numeric: v^2 = 2 a d
        mean = np.maximum((f[:-1] + f[1:]) / 2.0, TINY)                                  # numeric: a mean of two
        t = np.concatenate([[0.0], np.cumsum(T / mean)])
        n_out = max(1, int(math.ceil(t[-1] / dt - NEWLY)))
        out = []
        for m in range(1, n_out + 1):
            tm = min(m * dt, t[-1])
            k = min(N - 1, int(np.searchsorted(t, tm, side="right")) - 1)
            w = (tm - t[k]) / max(t[k + 1] - t[k], TINY)
            out.append(P[k] + (P[k + 1] - P[k]) * min(1.0, max(0.0, w)))
        out[-1] = P[-1].copy()
        return out

    # ------------------------------------------------------------------ the support (the surface the robot stands on)
    def _support(self):
        """(unit normal, offset m, sigma m or None) of the support plane in frame B: what the session's space book
        holds (a touch, a look or a statement set it), else cfg support when the owner states one, else the level
        plane at cfg floor_z_m with no stated error."""
        book = getattr(getattr(self, "bounds", None), "space", None)
        try:
            held = book.support() if book is not None else None
        except Exception:
            held = None
        if held is not None and held[0] is not None:
            (normal, offset), sigma = held[0], held[1]
            n = np.asarray(normal, dtype=float)
            size = float(np.linalg.norm(n))
            if size > 0.0:
                return n / size, float(offset) / size, sigma
        sup = self.cfg["support"]
        if isinstance(sup, dict) and sup.get("normal") is not None:
            n = np.asarray(sup["normal"], dtype=float)
            n = n / np.linalg.norm(n)
            return n, float(sup.get("offset_m", 0.0)), sup.get("sigma_m")
        return UP.copy(), float(self.cfg["floor_z_m"]), None

    def _floor_at(self, p, r=0.0):
        """Height (m, frame B) of the surface under point p: the mount top (z = 0) where p (grown by r) is over the
        mount, else the stated support under p."""
        h = self.cfg["mount_half_m"]
        if h is not None and abs(float(p[0])) <= h + r and abs(float(p[1])) <= h + r:
            return 0.0
        n, d, _ = self._support()
        if n[2] <= 0.0:
            return float(d)
        return float((d - n[0] * float(p[0]) - n[1] * float(p[1])) / n[2])

    def _cap_clearance(self, p0, p1, r):
        """Lowest point of a capsule above the surface under it (m)."""
        lo = p0 if p0[2] <= p1[2] else p1
        return float(lo[2]) - r - self._floor_at(lo, r)

    def floor_clearance(self, q):
        """Lowest moving capsule or the tip above the surface under it (m): the plane_clearance of the floor."""
        caps = self.model.collision_capsules(q)[1:]
        low = min((self._cap_clearance(p0, p1, r) for _, p0, p1, r in caps), default=float("inf"))
        tip = self.model.fk_tip(q)[0]
        return float(min(low, tip[2] - self._floor_at(tip)))

    def _margins(self, v=0.0, kind=None):
        """(to the support, between the robot's own links, how far below the support a tool that is meant to meet
        it may plan or None for no end): what a plan keeps against a measured thing is the cross track error
        measured at rest (a link's capsule stands for its body). Nothing here grows
        with the planned speed or with a thing's position doubt: what speed adds is priced, not refused. v stays
        in the signature for older callers and is not read."""
        B = self.bounds
        own = max(0.0, float(B.clearance(0.0)))
        below = None
        if kind in MEETS:
            support = self._thing("support") if hasattr(self, "_thing") else None
            try:
                s = self._last
                q = self.model.q_home if s is None else s.q
                spec = B.travel(kind, support, q, -UP)
                below = getattr(spec, "search_m", None) if not isinstance(spec, dict) else spec.get("search_m")
            except Exception:
                below = None
            _, _, sigma = self._support()
            if sigma is None and not below:
                below = None      # a stated height with no stated error: the stroke the caller states stands
        return own, own, (None if below is None else float(below))

    def standing_surface(self):
        """The floor as robot://body may serve it (mm, frame B)."""
        h = self.cfg["mount_half_m"]
        n, d, sigma = self._support()
        plane, _, _ = self._margins()
        return {"floor_z_mm": round(float(d) * MM, 1), "floor_sigma_mm": None if sigma is None else
                round(float(sigma) * MM, 2),
                "mount_top_z_mm": 0.0, "mount_half_side_mm": None if h is None else round(float(h) * MM, 1),
                "note": "the surface the robot stands on, as stated; a plan keeps the robot's links "
                        f"{plane * MM:.1f} mm above it (its tracking error at rest); a tip line planned through it "
                        "is cut there and run to contact, the contact stop armed"}

    # ------------------------------------------------------------------ sampling a path for a check (CC-018)
    def _check_samples(self, path, margin_m):
        """[(index into path of the waypoint the sample follows, joint pose, tip speed there m/s)] for a check with
        margin_m, the smallest margin in force: no robot point travels further between two samples than the
        spacing the bounds give that margin (half of it), so a long path is split, never thinned; a path whose
        own samples lie closer than that (a slow stretch) is read at that spacing. With no margin in force the
        spacing is that of the capsules' own fit error (the path's own largest step on a body that states none)."""
        qs = [np.asarray(q, dtype=float) for q in path]
        P = [self._robot_points(q)[0] for q in qs]
        steps = [float(np.max(np.linalg.norm(b - a, axis=1))) for a, b in zip(P[:-1], P[1:])]
        tips = [float(np.linalg.norm(b[-1] - a[-1])) / self.dt for a, b in zip(P[:-1], P[1:])]
        spacing = float(self.bounds.spacing(margin_m)) if margin_m and margin_m > 0.0 else 0.0
        # never finer than the body model itself is true: a margin under the capsules' own fit error (a reading's
        # float noise taken for a quantum on a noise-free robot, A1.1) would split a path into millions of poses.
        # With no margin in force the fit error alone sets the spacing, so the same path is read at the same
        # places whatever speed it is timed at
        try:
            fits = [float(v) for v in self.model.e_fit().values() if v and v > 0.0]
        except Exception:
            fits = []
        if fits:
            spacing = max(spacing, float(self.bounds.spacing(min(fits))))
        if not spacing > 0.0:
            spacing = max(steps, default=0.0)
        out, since = [(0, qs[0], tips[0] if tips else 0.0)], 0.0
        for k in range(1, len(qs)):
            step = steps[k - 1]
            if spacing > 0.0 and step > spacing:
                n = int(math.ceil(step / spacing))
                for j in range(1, n):
                    out.append((k, qs[k - 1] + (qs[k] - qs[k - 1]) * (j / n), tips[k - 1]))
                since = spacing
            since += step
            if since >= spacing or k == len(qs) - 1:
                out.append((k, qs[k], tips[k - 1]))
                since = 0.0
        return out

    def _path_speed(self, path):
        """The largest tip speed of a timed path (m/s)."""
        tips = np.asarray([self.model.fk_tip(np.asarray(q, dtype=float))[0] for q in path])
        return float(np.max(np.linalg.norm(np.diff(tips, axis=0), axis=1))) / self.dt if len(tips) > 1 else 0.0

    # ------------------------------------------------------------------ clearance from what the world holds (R1)
    def _nearest_known(self, q, known):
        """(clearance m, index of the known point, link name) of the robot at q from the nearest known point: the
        distance from each moving capsule's axis (and the tip) less its radius."""
        best = (float("inf"), None, None)
        segs = [(n, p0, p1, r) for n, p0, p1, r in self.model.collision_capsules(q)[1:]]
        tip = self.model.fk_tip(q)[0]
        segs.append(("tip", tip, tip, 0.0))
        for name, p0, p1, r in segs:
            ab = p1 - p0
            L2 = float(ab @ ab)
            for i, k in enumerate(known):
                t = 0.0 if L2 <= 0.0 else min(1.0, max(0.0, float((k - p0) @ ab) / L2))
                d = float(np.linalg.norm(p0 + t * ab - k)) - r
                if d < best[0]:
                    best = (d, i, name)
        return best

    def _contact_clearance(self, path, clear_m=None, exempt=None):
        """None, or a refusal when the path brings the robot newly within its margin of a point robot://world
        holds (a stored contact or a located point: measured things): a part of the arm that starts nearer than
        that may not come nearer still. clear_m None: the cross track error measured at rest (nothing that
        grows with the planned speed; the capsule itself stands for the body). exempt: the id or label of the
        thing this call takes, works or holds; it is not kept clear of. Sampled by _check_samples."""
        known = self._known_points(exempt)
        if len(path) < 2 or not len(known):
            return None
        need = max(0.0, float(self.bounds.clearance(0.0))) if clear_m is None else float(clear_m)
        samples = self._check_samples(path, need)
        c0 = self._nearest_known(samples[0][1], known)[0]
        for k, q, _ in samples[1:]:
            c, i, name = self._nearest_known(q, known)
            if c < need and c < c0 - NEWLY:
                pct = int(round(100.0 * k / (len(path) - 1)))
                return refusal("path_contact", f"the {name} would come within {max(c, 0.0) * MM:.0f} mm of a stored "
                               f"point at [{', '.join(f'{x * MM:.0f}' for x in known[i])}] mm at {pct} % of the path "
                               f"(the runtime keeps {need * MM:.1f} mm from what it has touched or located when "
                               "it moves by itself: its tracking error at rest)",
                               "move the arm clear of the part first (retreat, or move_tip away from it)")
        return None

    # ------------------------------------------------------------------ path check
    def _path_check(self, path, guarded=True, *, kind=None):
        """None, or a refusal when the path (one pose per control period) brings a LINK newly near the support
        under it or another link. The margin is the cross track error measured at rest (the link's capsule
        stands for its body): what speed adds across a path is priced by the plan, never refused, and two links
        of one arm lag together. The tool is never refused for the surface, on a move of any kind: the guard
        feels the tool, a tip line planned through the surface is cut there before it is timed and ends under
        its contact stop (control_expect.cut_at_support, called by _drive), and the tool's own bodies stand
        around the tip by their capsules' fit, which is no measured collision (a line that ends just over the
        surface is judged as one that ends in it). On a touch or a press that brings the tip nearer to the
        surface the link next to the tool is exempt there too (it stands on the tool, its capsule reaches
        the surface when the tool does, and the contact the stroke means ends the stroke); other links are not,
        but their capsules count as at the surface only past their own fit error."""
        if len(path) < 2:
            return None
        m = self.model
        spared, fit = set(self._tool_bodies), {}
        tip0, tip1 = (m.fk_tip(np.asarray(q, dtype=float))[0] for q in (path[0], path[-1]))
        if kind in ("touch", "press") and tip1[2] - self._floor_at(tip1) < tip0[2] - self._floor_at(tip0):
            links = [c[0] for c in m.collision_capsules(np.asarray(path[0], dtype=float))[1:] if c[0] not in spared]
            spared |= set(links[-1:])            # the last link of the chain before the tool's own bodies
            fit = dict(m.e_fit())                # a capsule under the surface by less than its own fit error is no
            # body under it: each link's clearance is read with its own fit error added (the plan refuses
            # only a collision with what is measured)
        plane, own, _ = self._margins(0.0, None)
        least = [x for x in (plane, own) if x > 0.0]
        samples = self._check_samples(path, min(least) if least else 0.0)
        q_first = samples[0][1]
        low0 = {c[0]: self._cap_clearance(c[1], c[2], c[3]) for c in m.collision_capsules(q_first)[1:]}
        hits0 = set(m.self_collisions(q_first, own))
        for k, q, v in samples[1:]:
            pct = int(round(100.0 * k / (len(path) - 1)))
            for name, p0, p1, r in m.collision_capsules(q)[1:]:
                if name in spared:
                    continue  # the guard feels the tool
                low = self._cap_clearance(p0, p1, r) + float(fit.get(name) or 0.0)
                if low < plane and low < low0.get(name, low) - NEWLY:
                    lo = p0 if p0[2] <= p1[2] else p1
                    return refusal("path_plane", f"the {name} link would come within {max(low, 0) * MM:.0f} mm of "
                                   f"the surface under the arm at {pct} % of the path (the clearance kept is "
                                   f"{plane * MM:.1f} mm: the arm's tracking error at rest; the surface is at z "
                                   f"{self._floor_at(lo, r) * MM:.0f} mm, the link's lowest point at z "
                                   f"{(float(lo[2]) - r) * MM:.0f} mm; the tool may be brought to the surface, "
                                   "not the links)",
                                   "approach from higher, or move the joints so the arm stays up; picture(view="
                                   "side) shows the arm")
            new = set(m.self_collisions(q, own)) - hits0
            if new:
                a, b = sorted(new)[0]
                return refusal("path_self", f"the {a} and {b} links would come within {own * MM:.1f}"
                               f" mm of each other at {pct} % of the path",
                               "split the move or go through another joint pose (move_joints)")
        return None
