# SPDX-License-Identifier: AGPL-3.0-only
"""Sensing of the Executor: the external joint torque from two readings one period apart, and where on the robot
the force that explains it acts.

Estimate (prediction-error observer, closed loop). The arm alone (the loaded model, no contacts) is stepped from the
previous reading over the elapsed time under the servo law with the commanded target held, exactly as the backend
runs it. The gap between the measured end state and that free prediction is read as an external joint torque: when
it is smaller than the solve itself resolves, by the gap's own first-order torque (never set to zero: what free
motion reads is the estimate's hidden level, which the bounds keep by speed and acceleration); otherwise it is
solved for: a torque in the first half of the period and another in the second (so a push that begins or ends
inside the period is matched in both end pose and end speed), with the servo's own answer to the push inside the
period in the loop, by Newton steps on the observer's arm started from the torque balance of the period's mean
servo torque. The estimate is the mean of the two halves. A load the hand holds is taken off the estimate with the
MEASURED acceleration of the tip (the fast joint speeds when the backend has them, else two readings), once its
mass is read at rest (rcpx/control_hold.py); nothing is guessed before that.

Location. The torque is mapped to a force on the tool: the tip when a force there explains it, else the tool point
or the part of the robot's own geometry on the force's line of action; a pattern no single push on the tool explains
is reported as source "arm or self". No robot is named here.
"""

import mujoco
import numpy as np

from rcpx import control_move_passed
from rcpx.control_base import Sense, _mm

FRICTION_V_RAD_S = 0.01  # dry joint friction is smoothed over this speed
FIT_ITERS = 6            # Newton steps of the torque solve at most
FIT_REG = 1e-4           # Tikhonov weight of a step, relative to the sensitivity's largest singular value
FIT_DONE = 0.001         # the solve ends once this share (or less) of the free deviation is left unexplained
FIT_STEP_DONE = 1e-4     # or, on an arm with dry joint friction, once a step is under this share of the limits
COUPLE_SHARE = 0.1       # numeric: a joint is loaded when its share of its limit is over this share of the largest
FACING = 0.3             # numeric: a tool surface faces a push when its normal has this much of it
_U = (np.arange(10) + 0.5) / 10.0   # Hermite samples across a period
_H = np.stack([2 * _U ** 3 - 3 * _U ** 2 + 1, _U ** 3 - 2 * _U ** 2 + _U, -2 * _U ** 3 + 3 * _U ** 2,
               _U ** 3 - _U ** 2], axis=1)
_DH = np.stack([6 * _U ** 2 - 6 * _U, 3 * _U ** 2 - 4 * _U + 1, -6 * _U ** 2 + 6 * _U, 3 * _U ** 2 - 2 * _U], axis=1)


class SenseMixin:
    # ---------------------------------------------------------------- the tool's own sizes (CC-090, CC-091)
    def _tool_sizes(self):
        """{at, near, surface, push} (m), from the tool's own geometry in the body model: a contact point within
        the tip capsule's radius of the tip is the tip (at); a line of action within that capsule's whole length
        of the tip that misses the hand's geometry is still the tool (near); a contact point within the mean of
        the two of a named tool surface carries its name (surface); a fitted wrench whose couple along the force
        is over force x the tool's length is not one push (push)."""
        if getattr(self, "_tool_sz", None) is None:
            m = self.model
            caps = [c for c in m.collision_capsules(m.q_home)[1:] if c[0] in self._tool_bodies]
            tip = m.fk_tip(m.q_home)[0]
            if caps:
                name, p0, p1, r = min(caps, key=lambda c: min(float(np.linalg.norm(c[1] - tip)),
                                                              float(np.linalg.norm(c[2] - tip))))
                at, near = float(r), float(np.linalg.norm(p1 - p0)) + float(r)
            else:
                at = near = 0.0
            self._set_kin(m.q_home)
            base = self._dd.xpos[m._jbody[-1]].copy()            # the body the last arm joint carries
            push = max(near, float(np.linalg.norm(tip - base)))  # the tool's length
            self._tool_sz = {"at": at, "near": max(at, near), "surface": (at + max(at, near)) / 2.0,
                             "push": push}   # numeric: a mean of two
        return self._tool_sz

    def _locate_min(self):
        """N m: a moment about the tip under this says nothing of where the force acts: the largest joint torque
        threshold the bounds derive at rest (the torque the estimate can hide plus its noise). Read once per
        call; 0 while the bounds cannot say (nothing measured yet)."""
        B = getattr(self, "bounds", None)
        if B is None:
            return 0.0
        cache = getattr(self, "_locate_cache", None)
        if cache is not None and cache[0] == self._calls:
            return cache[1]
        try:
            value = float(np.max(np.asarray(B.tau_thr(0.0), dtype=float)))
        except Exception:
            value = 0.0
        self._locate_cache = (self._calls, value)
        return value

    def _gravity(self):
        return np.asarray(self.model.mj.opt.gravity, dtype=float)

    # ---------------------------------------------------------------- dynamics
    def _model_torque(self, q, qd, qdd):
        """Joint torques the arm alone needs for (q, qd, qdd): M qdd + bias - passive, from the loaded model
        (rotor armature and dry joint friction included)."""
        m, d = self.model.mj, self._dd
        d.qpos[:] = self._qfull
        d.qpos[self._qadr] = q
        d.qvel[:] = 0
        d.qvel[self._dadr] = qd
        d.qacc[:] = 0
        d.qacc[self._dadr] = qdd
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        mujoco.mj_comVel(m, d)
        mujoco.mj_passive(m, d)
        mujoco.mj_rne(m, d, 1, self._rne)
        friction = self._frictionloss * np.tanh(np.asarray(qd, dtype=float) / FRICTION_V_RAD_S)
        return self._rne[self._dadr] - d.qfrc_passive[self._dadr] + self._armature * qdd + friction

    def _predict(self, prev, q_target, frac, h, first=None, second=None):
        """The arm alone over h from the previous reading under the servo law with q_target held, plus a joint
        torque from outside (N m, the world on the arm): `first` over the first half of h and `second` over the
        rest (None: none). Returns (q, qd)."""
        m, d = self._pm, self._pd
        qa, da, mot = self._pqa, self._pda, self._pmot
        d.qpos[:] = self._pfull
        d.qpos[qa] = prev.q
        d.qvel[:] = 0
        d.qvel[da] = prev.qd
        d.qacc_warmstart[:] = 0
        d.ctrl[:] = self._pctrl
        d.qfrc_applied[:] = 0
        lim = self.torque_limit * frac
        kp, kd = self.kp, self.kd
        steps = max(1, int(round(h / self.cfg["physics_dt_s"])))
        half = steps // 2
        for k in range(steps):
            if first is not None and (k == 0 or k == half):
                d.qfrc_applied[da] = first if k < half else second
            tau = kp * (q_target - d.qpos[qa]) - kd * d.qvel[da]
            d.ctrl[mot] = np.minimum(np.maximum(tau, -lim), lim)
            mujoco.mj_step(m, d)
        d.qfrc_applied[:] = 0
        return d.qpos[qa].copy(), d.qvel[da].copy()

    def _deadband(self, e, v):
        """A joint in stiction holds any torque inside its dry friction: that much of a residual is not a push."""
        band = self._frictionloss * (1.0 - np.abs(np.tanh(v / FRICTION_V_RAD_S)))
        return np.sign(e) * np.maximum(0.0, np.abs(e) - band)

    def _static_estimate(self, prev, s, q_target, frac, h):
        """Torque balance over the period (the arm on the world): the servo's mean torque along the cubic through
        the two readings, minus the model's torque at the mean pose, speed and acceleration. Exact while the joints
        turn slowly; the start of the solve."""
        lim = self.torque_limit * frac
        q = np.outer(_H[:, 0], prev.q) + np.outer(_H[:, 1], h * prev.qd) + np.outer(_H[:, 2], s.q) \
            + np.outer(_H[:, 3], h * s.qd)
        qd = (np.outer(_DH[:, 0], prev.q) + np.outer(_DH[:, 2], s.q)) / h + np.outer(_DH[:, 1], prev.qd) \
            + np.outer(_DH[:, 3], s.qd)
        tau = np.clip(self.kp * (q_target - q) - self.kd * qd, -lim, lim).mean(axis=0)
        v = (s.q - prev.q) / h
        # no dead band here: a joint held by dry friction is read at the middle of what it can hide (free motion
        # never comes this far: its estimate is zero by the free prediction, friction and all)
        return tau - self._model_torque(0.5 * (prev.q + s.q), v, (s.qd - prev.qd) / h)

    def _gap_torque(self, prev, s, q0, qd0, h):
        """The gap between the measured end state and the free prediction (q0, qd0), read as the joint torque from
        outside that makes it (N m, the world on the arm), to first order through the arm's inertia and the
        servo's stiffness and damping."""
        dq, dqd = s.q - q0, s.qd - qd0
        if not np.any(dq) and not np.any(dqd):
            return np.zeros(self.model.n)
        qm = 0.5 * (prev.q + s.q)
        zero = np.zeros(self.model.n)
        inertial = self._model_torque(qm, zero, dqd / h) - self._model_torque(qm, zero, zero)
        return inertial + (self._damping + self.kd) * dq / h + self.kp * (dq / 2.0 - h * dqd / 12.0)

    def _fit_estimate(self, prev, s, q_target, frac, h, free_end, start):
        """The external joint torque (the arm on the world, mean over the period) that makes the observer's arm end
        where the real one did. Unknowns in torque-limit units: the world's torque on the arm over the first and
        the second half of the period. Each Newton step linearises the observer around the present guess (one
        prediction per unknown), so a joint held by dry friction is linearised in the regime it is in."""
        n, lim = self.model.n, self.torque_limit
        w = self.cfg["observer_speed_weight"]
        probe = self.cfg["observer_probe_frac"]
        meas = np.concatenate([s.q / h, w * s.qd])

        def end(x):
            q, qd = self._predict(prev, q_target, frac, h, x[:n] * lim, x[n:] * lim)
            return np.concatenate([q / h, w * qd])

        r_free = float(np.linalg.norm(meas - free_end))
        x = np.concatenate([-start / lim, -start / lim])
        base = end(x) if np.any(x) else free_end
        best = (float(np.linalg.norm(meas - base)), x)
        # a joint with dry friction can hide torque behind a very small gap in pose and speed: for such an arm
        # the solve frees both halves from the first step and runs until its step is small, not until the gap is
        sticky = bool(np.any(self._frictionloss > 0.0))
        for k in range(FIT_ITERS):
            if not sticky and best[0] <= FIT_DONE * r_free:
                break
            together = k == 0 and not sticky   # the first step moves both halves together (a steady push)
            cols = n if together else 2 * n
            S = np.zeros((2 * n, cols))
            for j in range(cols):
                xj = x.copy()
                xj[j] += probe
                if together:
                    xj[n + j] += probe
                S[:, j] = (end(xj) - base) / probe
            mu = FIT_REG * float(np.linalg.norm(S, 2))
            A = np.vstack([S, mu * np.eye(cols)])
            dx = np.linalg.lstsq(A, np.concatenate([meas - base, np.zeros(cols)]), rcond=None)[0]
            if not np.all(np.isfinite(dx)):
                break
            x = np.clip(x + (np.concatenate([dx, dx]) if together else dx), -3.0, 3.0)
            base = end(x)
            res = float(np.linalg.norm(meas - base))
            if res < best[0]:
                best = (res, x)
            if sticky and float(np.max(np.abs(dx))) < FIT_STEP_DONE:
                break
        x = best[1]
        self._ext_end = -x[n:] * lim     # the second half alone: nearer the force at the reading (a rising push)
        return -0.5 * (x[:n] + x[n:]) * lim

    def _pick_estimate(self, prev, s, q_target, frac, h):
        """External joint torque (the arm on the world, N m) over the period that ended at reading s. A gap to the
        free prediction that is under what the solve resolves (FIT_STEP_DONE of each joint's limit) is read by its
        own first-order torque; it is never set to zero."""
        q0, qd0 = self._predict(prev, q_target, frac, h)
        e = self._gap_torque(prev, s, q0, qd0, h)
        if np.all(np.abs(e) <= FIT_STEP_DONE * self.torque_limit):
            self._ext_end = None
            return -e
        w = self.cfg["observer_speed_weight"]
        start = self._static_estimate(prev, s, q_target, frac, h)
        return self._fit_estimate(prev, s, q_target, frac, h, np.concatenate([q0 / h, w * qd0]), start)

    def _held_torque(self, prev, s, h, fast):
        """(joint torque N m the held load asks of the arm on the world, force N at the tip) for what the hand
        holds, with the MEASURED acceleration of the tip: m (a - g). The mass is the hold record's (rcpx/
        control_hold.py: read from the RAW estimate at rest after the first move that followed the close); None
        while nothing is held or its load is not read yet."""
        held = getattr(self, "_held", None)
        if not held or not held.get("mass_kg"):
            return None
        J = self.model.jac(s.q)[:3]
        a = np.zeros(3)
        qd = fast.get("qd") if isinstance(fast, dict) else None
        dt_f = float(fast.get("dt_s") or 0.0) if isinstance(fast, dict) else 0.0
        if qd is not None and len(qd) > 1 and dt_f > 0.0:
            v = np.asarray(qd, dtype=float) @ J.T
            a = (v[-1] - v[0]) / (dt_f * (len(v) - 1))
        elif prev is not None and h:
            a = (J @ s.qd - self.model.jac(prev.q)[:3] @ prev.qd) / h
        F = float(held["mass_kg"]) * (a - self._gravity())
        return J.T @ F, F

    def _load_off(self, s, prev, h):
        """Take the held load off reading s (its weight and inertia at the measured acceleration) and say where
        what is left acts. Returns the point it acts at (None: the tip)."""
        load = self._held_torque(prev, s, h, s.fast)
        s.ext = s.ext0 if load is None else s.ext0 - load[0]
        s.F, s.where, at, s.loc = self._locate(s.q, s.ext, s.hidden)
        s.Fmag = float(np.linalg.norm(s.F))
        # Fend: the hardest the period read. With a fast wrist signal: its largest reading over the period, a
        # held load taken off. With none the mean is all there is: the peak of a strike is then unresolved, and a
        # result that reports one says so (control_speed._stop_peak).
        s.Fend, s.peak_note = s.Fmag, None
        wrist = s.fast.get("wrist_n") if s.fast is not None else None
        if wrist is not None and len(wrist):
            w = np.asarray(wrist, dtype=float).reshape(len(wrist), -1)
            if load is not None:
                w = w - load[1]
            s.Fend = max(s.Fmag, float(np.max(np.linalg.norm(w, axis=1))))
        else:
            s.peak_note = "peak unresolved: this backend gives no signal faster than the control period"
        return at

    def _resense(self):
        """The last reading again with the hold as it stands now (control_hold.read_load calls it once the load
        is read: from then on the reading has the load off)."""
        s = self._last
        if s is not None and getattr(s, "ext0", None) is not None:
            at = self._load_off(s, None, None)
            s.at = s.tip.copy() if at is None else at
        return s

    def _sense(self):
        """One reading with the external tip force estimate (frame B, N, the force the robot puts on the world) and
        where it acts (see the module docstring). The first reading, and one that follows a gap longer than
        observer_max_s, compares the measured torque with the model at rest. A held load whose mass is read is
        taken off (its weight and inertia at the measured acceleration); raw keeps the whole estimate."""
        r = self.backend.read()
        s = Sense()
        s.t = float(r["t"])
        s.q = np.asarray(r["q"], dtype=float).copy()
        s.qd = np.asarray(r.get("qd", np.zeros_like(s.q)), dtype=float).copy()
        s.tau = np.asarray(r["tau"], dtype=float).copy()
        s.gripper = r.get("gripper")
        s.fast = r.get("fast") if isinstance(r.get("fast"), dict) else None
        s.overload = bool(r.get("overload", False))
        prev = self._last
        if prev is not None and abs(s.t - prev.t) < 1e-9 and np.array_equal(s.q, prev.q):
            return prev
        h = None if prev is None else s.t - prev.t
        self._ext_end = None
        if h is not None and 1e-6 < h <= self.cfg["observer_max_s"] and self._q_cmd is not None:
            q_target = np.asarray(r.get("q_target", self._q_cmd), dtype=float)
            frac = float(r.get("torque_frac", self._frac_cmd))
            s.ext = self._pick_estimate(prev, s, q_target, frac, h)
        else:
            s.ext = self._deadband(s.tau - self._model_torque(s.q, s.qd, np.zeros_like(s.q)), s.qd)
        # what joints near standstill can hide behind their dry friction: no place off the tip is read from less
        s.hidden = float(np.linalg.norm(self._frictionloss * (1.0 - np.abs(np.tanh(s.qd / FRICTION_V_RAD_S)))))
        s.ext0 = s.ext
        s.raw = self._locate_where(s.q, s.ext, s.hidden)[0]
        at = self._load_off(s, prev, h)
        s.tip, s.R = self.model.fk_tip(s.q)
        s.at = s.tip.copy() if at is None else at
        self._last = s
        control_move_passed.note(self, s)   # where the arm's own bodies have been is free space
        return s

    def force_now(self):
        """The present external tip force estimate (N, frame B)."""
        return self._sense().F

    # ---------------------------------------------------------------- where the force acts
    def _tool_points(self):
        """(name, body id, point in the body, outward normal in the body or None) of the tip and every tool surface
        of the body model."""
        if getattr(self, "_tool_pts", None) is None:
            m, spec = self.model.mj, self.model.spec
            pts = [("tip", m.site("tip").id, None)]
            for surf in spec.get("tool_surfaces", []):
                try:
                    sid = m.site("surface_" + surf["name"]).id
                except KeyError:
                    continue
                nrm = surf.get("normal")
                pts.append((surf["name"], sid, None if nrm is None else np.asarray(nrm, dtype=float)))
            self._tool_pts = [(n, int(m.site_bodyid[i]), np.asarray(m.site_pos[i], dtype=float).copy(), nrm)
                              for n, i, nrm in pts]
        return self._tool_pts

    def _set_kin(self, q):
        m, d = self.model.mj, self._dd
        d.qpos[:] = self._qfull
        d.qpos[self._qadr] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        return m, d

    def _hand_body(self, b):
        """True for a body the last arm joint carries (the hand: the last link, the tool and its parts)."""
        return self.model._arm_key(int(b)) == self.model.n - 1

    def _links_loaded(self, ext):
        """Names of the links a push that makes this torque pattern can sit on: the body the last loaded joint
        carries and the arm's links after it (a push loads no joint beyond its own link, but a push along a
        joint's axis, or through it, leaves that joint unloaded too, so the links further out stay possible)."""
        share = np.abs(ext) / self.torque_limit
        loaded = np.nonzero(share > COUPLE_SHARE * float(share.max()))[0]
        if loaded.size == 0:
            return []
        m = self.model.mj
        return [m.body(b).name for b in self.model._jbody[int(loaded[-1]):]]

    def _ray_hits(self, q, p_line, u):
        """Where a ray that comes from outside along -u through p_line enters each piece of the robot's own
        collision geometry: [(body id, point)], nearest the outside first. Those are the outer faces a push along
        -u on the robot can land on."""
        m, d = self._set_kin(q)
        start = p_line + u * 2.0 * self.model.length_scale
        back = -u
        hits = []
        for g in range(m.ngeom):
            if not (m.geom_contype[g] or m.geom_conaffinity[g]):
                continue
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                dist = mujoco.mj_rayMesh(m, d, g, start, back)
            else:
                dist = mujoco.mju_rayGeom(d.geom_xpos[g], d.geom_xmat[g], m.geom_size[g], start, back,
                                          int(m.geom_type[g]))
            if dist >= 0:
                hits.append((float(dist), int(m.geom_bodyid[g])))
        hits.sort()
        return [(b, start + back * dist) for dist, b in hits]

    def _name_point(self, body, p, u):
        """The named tool surface nearest a point on the tool (within the tool's own surface size, facing the push
        u), else the body's own name. Kinematics must be set."""
        m, d = self.model.mj, self._dd
        best = (self._tool_sizes()["surface"], m.body(body).name)
        for name, b, local, nrm in self._tool_points():
            R = d.xmat[b].reshape(3, 3)
            if nrm is not None and float((R @ nrm) @ u) < FACING:
                continue
            dist = float(np.linalg.norm(d.xpos[b] + R @ local - p))
            if dist < best[0]:
                best = (dist, name)
        return best[1]

    def _beyond(self, q, ext):
        """The part of the external joint torques that no wrench at the tip explains (N m per joint). A force and
        a moment at the tip are what the tool and a thing gripped in it can pass on (a held part that is turned
        loads the arm with a couple); what is left over was put on a link. None for an arm of six joints or
        fewer (every torque pattern is then some wrench at the tip) or where the tip's wrench is not determined."""
        if self.model.n <= 6:
            return None
        J6 = self.model.jac(q)
        if np.linalg.cond(J6) >= 1e4:
            return None
        return ext - J6.T @ np.linalg.lstsq(J6.T, ext, rcond=1e-6)[0]

    def _locate(self, q, ext, hidden=0.0):
        """_locate_where's answer with loc["beyond"] (see _beyond) added: what a guard judges while something is
        held."""
        F, where, at, loc = self._locate_where(q, ext, hidden)
        return F, where, at, dict(loc, beyond=self._beyond(q, ext))

    def _locate_where(self, q, ext, hidden=0.0):
        """(F, where, at, loc): the force on the world (N, frame B), the name of what touched, the point (None: the
        tip) and loc = {at_tip, residual, source, links}. residual is the share of the joint torque the chosen
        explanation leaves; source is "tool", or "arm or self" when no single push on the hand explains the
        torque (then where is None and links names the likely link).

        A force at a point off the tip loads the joints with its moment about the tip too. With six or more joints
        the full wrench at the tip is fitted: force and moment fix the force's line of action, and the robot's own
        geometry met first on that line from outside is the contact. (A push may slide along its line of action
        without changing any joint torque, so the line is all the joints can say; the geometry says the rest.)
        When that line passes through the hand the contact is on the hand (its first face from outside); when it
        misses the hand, or the wrench is a couple, no push on the hand made the torque and the links named are
        those from the last loaded joint outward. With fewer joints the wrench is not determined: the tip and
        each named tool surface are tried as points, and an arm contact is told only from torque no tool point
        explains beyond the joints' dry friction. An offset from the tip is only read from a moment (or, with
        fewer than six joints, a torque the tip leaves unexplained) over the joints' own torque threshold
        (_locate_min) plus `hidden`, the torque (N m) that joints near standstill can hide behind their dry
        friction at this reading. The sizes that say "at the tip" and "on the tool" are the tool's own
        (_tool_sizes)."""
        n = self.model.n
        sz, least = self._tool_sizes(), self._locate_min()
        J6 = self.model.jac(q)
        J = J6[:3]
        e_norm = float(np.linalg.norm(ext))
        at_tip = {"at_tip": True, "residual": 0.0, "source": "tool", "links": []}
        if e_norm < 1e-9:
            return np.zeros(3), "tip", None, at_tip
        F0 = np.linalg.lstsq(J.T, ext, rcond=1e-3)[0]
        r0 = float(np.linalg.norm(J.T @ F0 - ext)) / e_norm
        m, d = self._set_kin(q)
        tip = d.site_xpos[m.site("tip").id].copy()
        arm = {"at_tip": False, "residual": r0, "source": "arm or self", "links": self._links_loaded(ext)}
        if n >= 6 and np.linalg.cond(J6) < 1e4:
            w = np.linalg.lstsq(J6.T, ext, rcond=1e-6)[0]
            F, M = w[:3], w[3:]
            f = float(np.linalg.norm(F))
            res = float(np.linalg.norm(J6.T @ w - ext)) / e_norm
            if float(np.linalg.norm(M)) < least + hidden:
                return F, "tip", None, dict(at_tip, residual=res)
            if f < 1e-6 or abs(float(M @ F)) > sz["push"] * f * f:
                return F0, None, None, arm      # a couple, or a force pair: not one push on the tool
            offset = np.cross(F, M) / (f * f)   # from the tip to the line of action, across it
            if float(np.linalg.norm(offset)) <= sz["at"]:
                return F, "tip", None, dict(at_tip, residual=res)
            hits = self._ray_hits(q, tip + offset, F / f)
            hand = [h for h in hits if self._hand_body(h[0])]
            if not hand and float(np.linalg.norm(offset)) <= sz["near"]:
                # the line passes just beside the hand's geometry near the tip (an edge of the finger): a tool
                # contact beside the tip, placed on the line where it passes nearest the tip
                p = tip + offset
                body = int(m.site_bodyid[m.site("tip").id])
                return F, self._name_point(body, p, F / f), p, {"at_tip": False, "residual": res,
                                                                "source": "tool", "links": []}
            if not hand:   # no push on the hand makes this torque: a link of the arm, or the arm on itself
                return F0, None, None, dict(arm, residual=res)
            body, p = hand[0]
            if float(np.linalg.norm(p - tip)) <= sz["at"]:
                return F, "tip", None, dict(at_tip, residual=res)
            return F, self._name_point(body, p, F / f), p, {"at_tip": False, "residual": res, "source": "tool",
                                                            "links": []}
        if r0 < 0.004 or r0 * e_norm < least + hidden:
            return F0, "tip", None, dict(at_tip, residual=r0)
        jp = np.zeros((3, m.nv))
        best = (r0, F0, "tip", None)
        for name, b, local, nrm in self._tool_points()[1:]:
            R = d.xmat[b].reshape(3, 3)
            pw = d.xpos[b] + R @ local
            mujoco.mj_jac(m, d, jp, None, pw, b)
            Jc = jp[:, self._dadr]
            Fc = np.linalg.lstsq(Jc.T, ext, rcond=1e-3)[0]
            if nrm is not None and float(Fc @ (R @ nrm)) <= 0.0:
                continue   # a surface is pushed on from outside: the robot's push points out of it
            rc = float(np.linalg.norm(Jc.T @ Fc - ext)) / e_norm
            if rc < 0.5 * best[0]:
                best = (rc, Fc, name, pw.copy())
        # an arm contact is only read from torque no tool point explains beyond the joints' dry friction
        left = best[0] * e_norm
        if best[0] > 0.08 and left > 2.0 * float(np.linalg.norm(self._frictionloss)) + 2.0 * least:
            return F0, None, None, dict(arm, residual=best[0])
        return best[1], best[2], best[3], {"at_tip": best[2] == "tip", "residual": best[0], "source": "tool",
                                           "links": []}

    def _contact_info(self, first, last):
        """The contact as results carry it. The place is the tip (or the tool point that touched) at the first
        reading over the threshold; the force and what touched are read at the last one; tip_speed_mm_s is the tip's
        speed at the first reading (a rigid surface met faster reads harder at the stop)."""
        F = last.F
        mag = float(np.linalg.norm(F))
        normal = (-F / mag) if mag > 1e-9 else np.zeros(3)
        mag = max(mag, float(getattr(last, "Fend", None) or 0.0))   # the reading's end: a rising push reads harder
        loc = getattr(last, "loc", None) or {}
        tool = loc.get("source", "tool") == "tool"
        where = (getattr(last, "where", None) or "tip") if tool else None
        at_tip = bool(tool and loc.get("at_tip", where == "tip"))
        if not tool:
            point = None
        elif at_tip:
            point = first.tip
        else:
            point = last.at + (first.tip - last.tip)   # the tool point, back where the tool stood at first touch
        qd = getattr(first, "qd", None)
        speed = None if qd is None else round(float(np.linalg.norm(self.model.jac(first.q)[:3] @ qd)) * 1000.0, 1)
        note = getattr(last, "peak_note", None)
        out = {"point_mm": None if point is None else _mm(point), "force_n": round(mag, 2), "tip_speed_mm_s": speed,
               "force_vec_n": [round(float(x), 2) for x in F], "normal_est": [round(float(x), 3) for x in normal],
               "t_s": round(last.t, 3), "surface": where, "at_tip": at_tip,
               "fit_residual": round(float(loc.get("residual", 0.0)), 3), "source": "tool" if tool else "arm or self"}
        if note:
            out["peak_note"] = note
        if not tool:
            out["likely_links"] = list(loc.get("links") or [])
            out["tip_mm_at_stop"] = _mm(last.tip)
        return out
