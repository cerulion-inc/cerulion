# SPDX-License-Identifier: AGPL-3.0-only
"""External joint torque over one control period, closed loop (for touched_by).

The executor's own estimate (Sense.ext) predicts the arm alone from the previous reading with the servo law and the
commanded target held, and reads the external torque as M (qd_measured - qd_predicted) / elapsed. That prediction
runs the servo on the PREDICTED joint path, so the servo's own reaction to the push inside the period is missing:
on a stiff servo the estimate shrinks by a different factor on every joint, which tilts the torque pattern a point
fit depends on. Here the same arm-alone prediction is run with an unknown external joint torque in it, modelled as
a constant plus a ramp across the period, and solved so that the predicted end position and end velocity match the
measured ones (2n equations, 2n unknowns, linear: the sensitivities come from one prediction per unknown). The
constant part is the period's mean external torque, servo reaction included.

Sign as Sense.ext: the torque the robot puts on the world (ext = J^T F for a force F the robot pushes the world
with). Only readings the executor already takes are used: joint angles and speeds, time, and the commanded target
and torque share. Runs on the runtime's own copy of the arm model; nothing here reads the scene.
"""

import mujoco
import numpy as np

from bench.robots import home_qpos, load_robot_mjcf

PERTURB_FRAC = 0.01   # sensitivity step: this share of each joint's torque limit
REG = 1e-4            # Tikhonov weight, relative to the sensitivity's largest singular value
NEWTON_ITERS = 3


class TorqueEstimator:
    def __init__(self, model, physics_dt):
        spec = model.spec
        s = load_robot_mjcf(spec)
        s.option.timestep = float(physics_dt)
        s.option.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.m = s.compile()
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.qa = np.array([m.jnt_qposadr[m.joint(j).id] for j in model.joint_names])
        self.da = np.array([m.jnt_dofadr[m.joint(j).id] for j in model.joint_names])
        self.mot = np.array([m.actuator("motor_" + j).id for j in model.joint_names])
        self.full = home_qpos(m, spec)
        self.ctrl0 = np.zeros(m.nu)
        g = spec.get("gripper")
        for a, v in zip(g["actuators"] if g else [], g["closed"] if g else []):
            self.ctrl0[m.actuator(a).id] = v
        self.kp = np.asarray(spec["kp"], dtype=float)
        self.kd = np.asarray(spec["kd"], dtype=float)
        self.lim = np.asarray(spec["torque_limit"], dtype=float)
        self.n = len(model.joint_names)
        self.dt = float(physics_dt)

    def _predict(self, q0, qd0, q_target, frac, h, t_const, t_ramp):
        m, d = self.m, self.d
        d.qpos[:] = self.full
        d.qpos[self.qa] = q0
        d.qvel[:] = 0.0
        d.qvel[self.da] = qd0
        d.qacc_warmstart[:] = 0.0
        d.ctrl[:] = self.ctrl0
        lim = self.lim * frac
        steps = max(1, int(round(h / self.dt)))
        for k in range(steps):
            d.qfrc_applied[:] = 0.0
            d.qfrc_applied[self.da] = t_const + t_ramp * ((k + 0.5) / steps - 0.5)
            d.ctrl[self.mot] = np.clip(self.kp * (q_target - d.qpos[self.qa]) - self.kd * d.qvel[self.da], -lim, lim)
            mujoco.mj_step(m, d)
        return d.qpos[self.qa].copy(), d.qvel[self.da].copy()

    def _state(self, x, n):
        return x[:n] * self.lim, x[n:] * self.lim

    def estimate(self, prev, cur, q_target, frac, start=None, iters=NEWTON_ITERS):
        """Mean external torque (the robot on the world) over the period from reading prev (t, q, qd) to cur, or
        None when the two readings do not make one period. start: a first guess in the same sign (the executor's
        own estimate); the solve is linearised around the guess and repeated, so a joint held by dry friction is
        linearised in the regime it was really in."""
        t0, q0, qd0 = prev
        t1, q1, qd1 = cur
        h = float(t1) - float(t0)
        if not (1e-6 < h <= 0.5) or q_target is None:
            return None
        n = self.n
        q_target = np.asarray(q_target, dtype=float)
        meas = np.concatenate([np.asarray(q1, dtype=float) / h, np.asarray(qd1, dtype=float)])
        x = np.zeros(2 * n)                       # unknowns in torque-limit units: constant part, then ramp part
        if start is not None and np.all(np.isfinite(start)):
            x[:n] = -np.asarray(start, dtype=float) / self.lim
        for _ in range(max(1, int(iters))):
            tc, tr = self._state(x, n)
            p0, v0 = self._predict(q0, qd0, q_target, frac, h, tc, tr)
            base = np.concatenate([p0 / h, v0])
            S = np.zeros((2 * n, 2 * n))
            for j in range(2 * n):
                xj = x.copy()
                xj[j] += PERTURB_FRAC
                pj, vj = self._predict(q0, qd0, q_target, frac, h, *self._state(xj, n))
                S[:, j] = (np.concatenate([pj / h, vj]) - base) / PERTURB_FRAC
            mu = REG * float(np.linalg.norm(S, 2))
            A = np.vstack([S, mu * np.eye(2 * n)])
            rhs = np.concatenate([meas - base, np.zeros(2 * n)])
            dx = np.linalg.lstsq(A, rhs, rcond=None)[0]
            if not np.all(np.isfinite(dx)):
                return None
            x = x + dx
            if float(np.max(np.abs(dx))) < 1e-6:
                break
        return -x[:n] * self.lim   # the world on the robot is x; the robot on the world is its negative
