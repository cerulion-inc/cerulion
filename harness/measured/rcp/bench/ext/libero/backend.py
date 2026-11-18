# SPDX-License-Identifier: AGPL-3.0-only
"""The rcpx arm on a LIBERO episode (build plan S6): an rcpx Backend over the env host, the servo law the arm really
has, and the vnext session episode_ext serves.

    session = build_session(env, ep_dir, facts, {"requests": 80, "steps": 900}, "all")   # episode_ext's hook

LiberoBackend (the interface of rcpx/backend.py, as rcpx/sim_backend.py offers it) proxies to the env host over
bench/ext/envproto.py (a connection of its own to the host's public socket):
- command(q_target, torque_frac, gripper) and step(n): the joint goal is driven closed loop through the env's own
  JOINT_POSITION controller (envhost servo_joints: each control step the goal is the present pose plus the error
  clipped to L.JOINT_SERVO step_rad; its kp too), so the physics is the plain arm's; the torque cap clips the
  controller's output. Each control
  period is one env step and counts against the cell's shared step budget; once the host runs no more steps (budget
  spent, or the episode over) the world is frozen and the clock alone runs, so no motion loop waits forever.
- the gravity lead: rcpx commands q_des + g(q_des) / kp (the lead a plain joint PD needs to rest at q_des,
  rcpx/control.py _command). This arm's controller compensates gravity itself, so the backend drives the goal at
  which a plain PD with that command would rest: the q with q + g(q) / kp = q_target (a fixed point, solved on the
  rcpx body model). read() reports q_target as commanded and servo_goal as driven.
- read(): joint angles, speeds, applied torques and their share of the limits, the finger joints and their
  targets, and the wrist force and torque the hand's own sensor reads, tared by the model (the weight of what hangs
  below the sensor, at the present pose) and stated as what the robot puts on the world, in frame B. Never a
  contact name, an object, a pose of anything but the arm, or a success signal.
- render(camera) and camera_info(camera): the wrist camera at the spec's size, and the fixed scene camera(s) with
  the installation pose in frame B (rcpx.r2.scene_camera works the looks from them).

ServoLawMixin (exported as this module's Mixin, so rcpx.r2.core.executor_class takes this module like a package):
rcpx's force observer predicts each control period with the servo law of the arm it drives. This arm's law is the
env controller's: tau = M(q) (kp (goal - q) - kd qd) + bias(q, qd), goal = q0 + clip(target - q0, +-step_rad), clipped
to the torque cap, with the scene's air drag on every link (the arm model carries it). The mixin predicts with that
law (OVERRIDES: _predict, _static_estimate, and _pick_estimate, which leaves unjudged the one period in which the host
hands the arm to its joint controller), and setup() stops a motion with "steps_spent" once the host runs no more
control steps. Replayed on recorded free motion this law matches the env to about 1e-6 rad per period.
"""

import json
import math
import os

import mujoco
import numpy as np

from bench.ext import envproto as P
from bench.ext import libero as L
from bench.ext.libero import robot_spec as RS
from bench.ext.libero.sense import SenseMixin
from rcpx.backend import Backend

ID = "B3"
OVERRIDES = ("_predict", "_static_estimate", "_pick_estimate")
CONTROL_HZ = 20.0   # the env's own control rate (the benchmark's 20 Hz); the backend states its period from it
# The arm's rated joint speeds, rad/s (SPEC-R4 Q1, maintainer ruling 5: a maker's figure with its sheet cited). The
# benchmark's robot file carries no velocity limit; the maker's own robot description does:
RATED_QD = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]
RATED_QD_SHEET = ("the maker's robot description, franka_ros franka_description robots/panda/joint_limits.yaml "
                  "(max_velocity per joint; the same figures as the maker's control parameter table)")
# Self-knowledge: measured in ONE calibration episode outside every scored cell, kept in
# this folder, and REQUIRED in every scored cell (an absent file is a refusal there, never a self-test in a cell).
# Where (the design rulings, C17): env RCPX_SELFKNOW_DIR (the older RCPX_EXT_SELFKNOW_DIR wins), else ~/.rcpx/selfknow.
SELFKNOW_DIR = os.environ.get("RCPX_EXT_SELFKNOW_DIR") or os.environ.get("RCPX_SELFKNOW_DIR") or os.path.join(
    os.path.expanduser("~"), ".rcpx", "selfknow")
# Which mode (cfg selfknow, rcpx/bounds_selfstart.py): build_session's argument when a caller states one, else env
# RCPX_SELFKNOW_MODE, else "require" (a stored record or a refusal, as every scored LIBERO cell ran). The words:
# "require", "measure" (the robot calibrates itself now and writes the record), "cold" (cfg selfknow None: no
# self-test, the body's own description). Any other word raises before a session is built.
SELFKNOW_MODES = {"require": "require", "measure": "measure", "cold": None}
SELFKNOW_DEFAULT = "require"
STATED = object()   # build_session's selfknow when the caller states none


def selfknow_mode(stated=STATED, environ=None):
    """cfg selfknow of a session: stated (None is cold) when given, else the word of RCPX_SELFKNOW_MODE, else
    SELFKNOW_DEFAULT. Raises ValueError on a word not in SELFKNOW_MODES."""
    if stated is not STATED:
        if stated not in SELFKNOW_MODES.values():
            raise ValueError(f"cfg selfknow {stated!r} is none of {sorted(SELFKNOW_MODES)}")
        return stated
    word = str((os.environ if environ is None else environ).get("RCPX_SELFKNOW_MODE") or "").strip() \
        or SELFKNOW_DEFAULT
    if word not in SELFKNOW_MODES:
        raise ValueError(f"RCPX_SELFKNOW_MODE {word!r} is none of {sorted(SELFKNOW_MODES)}")
    return SELFKNOW_MODES[word]
_JS = L.JOINT_SERVO   # the env host's joint servo gains (bench/ext/libero/__init__.py)
LAW = {"kind": "inertia weighted joint PD with gravity and speed compensation (the env's JOINT_POSITION controller)",
       "kp": [_JS["kp"]] * 7, "kd": [2.0 * _JS["damping_ratio"] * math.sqrt(_JS["kp"])] * 7,
       "goal_step_rad": _JS["step_rad"]}
SCENE_CAMERAS = ("agentview",)    # the benchmark's fixed scene camera: the one the plain arm sees too
SCENE_PX = (256, 256)
FRAC_MIN = 1e-6                   # the host takes a torque share above 0; "torque off" is this
GRIP_DEADBAND = 0.125             # half the gripper ramp of one control step (0.01 per physics step, 25 steps)
GRIP_RAMP_PER_STEP = 0.25
LEAD_ITERS = 12
ENV_CONTROL = "joint"             # episode_ext's envhost --control (the host switches controllers per request)


def _vec(x, n=None):
    a = np.asarray(x, dtype=float).reshape(-1)
    if n is not None and a.shape[0] != n:
        raise ValueError(f"expected {n} values, got {a.shape[0]}")
    return a


_full_m = RS.full_inertia


class HostLink:
    """call(op, **args) on the env host's public socket: an envproto.Client, or a fresh one on the socket path of
    any object that names it (episode_ext's EnvClient has sock_path)."""

    def __init__(self, target, timeout_s=600.0):
        if callable(getattr(target, "call", None)):
            self.client, self.own = target, False
        else:
            path = getattr(target, "sock_path", target)
            self.client, self.own = P.Client(str(path), timeout_s), True

    def call(self, op, **args):
        return self.client.call(op, **args)

    def close(self):
        if self.own:
            self.client.close()


class LiberoBackend(SenseMixin, Backend):
    """The arm through the env host (module docstring; depth and the fast wrist force: bench/ext/libero/sense.py). link: HostLink; model: rcpx.model.RobotModel of the spec
    robot_spec.build_spec made; facts: build_spec's facts (T_world_B, finger_ctrlrange)."""

    def __init__(self, link, model, facts, law=None, state=None, scene_px=SCENE_PX):
        self.link, self.model, self.facts = link, model, dict(facts)
        spec = model.spec
        self.law = dict(LAW if law is None else law)
        # the law as the bridge's own observer reads it (controller gains, not joint stiffness: the kind is the
        # backend's own word, so rcpx's bounds take the joint stiffness from the body model's kp, A8.4)
        self.servo_law = {"kind": "inertia_weighted_joint_pd", "kp": np.asarray(self.law["kp"], float),
                          "kd": np.asarray(self.law["kd"], float),
                          "goal_step_rad": float(self.law["goal_step_rad"])}
        self.dt_s = 1.0 / CONTROL_HZ
        self.is_hardware = False
        self.kp = np.asarray(spec["kp"], float)
        self.limit = np.asarray(spec["torque_limit"], float)
        self.n = model.n
        self.T_WB = np.asarray(self.facts["T_world_B"], float)
        self.g_B = self.T_WB[:3, :3].T @ np.array([0.0, 0.0, -9.81])
        g = spec.get("gripper") or {}
        self._g_open = np.asarray(g.get("open", [0.04, -0.04]), float)
        self._g_closed = np.asarray(g.get("closed", [0.0, 0.0]), float)
        rng = self.facts.get("finger_ctrlrange") or {}
        self._g_ctrl = np.array([rng.get(j, [0.0, 0.04]) for j in g.get("joints", [])], float).reshape(-1, 2)
        self.scene = dict(spec.get("scene_cameras") or {})
        self.scene_px = scene_px
        self._d = mujoco.MjData(model.mj)          # the backend's own kinematics (the wrist sensor tare)
        m = model.mj
        self._ft_site = m.site(RS.FT_SITE).id
        self._ft_body = int(m.site_bodyid[self._ft_site])
        self._ft_mass = float(m.body_subtreemass[self._ft_body])
        self._qa = np.array([m.jnt_qposadr[m.joint(j).id] for j in model.joint_names])
        self._qfull = self._d.qpos.copy()
        self.steps_run = 0
        self.spent = False          # the host runs no more control steps (budget spent or the episode over)
        self.lost = False           # the host stopped answering
        self._t_frozen = 0.0        # clock seconds run while the world was frozen
        self.render_failures, self.render_retried = [], False
        self._st = state if state is not None else link.call("read")
        q = _vec(self._st["q"], self.n)
        self.q_goal = q.copy()
        self.q_cmd = q + model.gravity_torque(q) / self.kp
        self.torque_frac = 1.0
        self.grip = "open"          # what rcpx asked: "open", "close" or an opening fraction
        self._g_sent = -1.0
        self.unjudged_t = None      # end of a period in which the env's controller was handed over (not judged)

    def contact_model(self):
        """{"kind": "damped", "b_per_s", "by_body"} (rcpx/backend.py, SPEC-R4 A8.4): the contact rate of the arm's
        own collision geoms, read from the robot file the env host serves (a geom's contact is a damped spring
        given as (time constant, damping ratio): b = 2 / (d timeconst), d the top of its impedance, the time
        constant kept at or over two physics steps as the simulator does; or as (-stiffness, -damping): b =
        damping / d). b_per_s is the stiffest of the tool's geoms (the last arm joint's body and all under it).
        the design rulings: each body's rate is the stiffer of its own geoms' and the stiffest its geoms can meet the scene
        with by the simulator's own mixing rule (the env host's contact_rates: robot body names only), so a scene
        part stiffer than the arm's own geoms (a button of 0.001 s) is read with the rate its contact has."""
        if getattr(self, "_contact", None) is None:
            m = self.model.mj
            safe = not (int(m.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_REFSAFE))
            by_body = {}
            for g in range(m.ngeom):
                if not (m.geom_contype[g] or m.geom_conaffinity[g]):
                    continue
                d = min(float(m.geom_solimp[g][1]), 1.0)
                ref = m.geom_solref[g]
                if d <= 0:
                    continue
                if ref[0] > 0:
                    tc = max(float(ref[0]), 2 * float(m.opt.timestep)) if safe else float(ref[0])
                    rate = 2 / (d * tc)
                else:
                    rate = -float(ref[1]) / d
                name = m.body(int(m.geom_bodyid[g])).name
                by_body[name] = max(by_body.get(name, 0.0), rate)
            last = int(m.jnt_bodyid[m.joint(self.model.joint_names[-1]).id])

            def under(b):
                while b > 0 and b != last:
                    b = int(m.body_parentid[b])
                return b == last

            src = "the robot file the env host serves (its own geoms alone: the host stated no mixed rate)"
            try:     # the design rulings: the stiffest contact each body can meet the scene with, mixed by the simulator's rule
                mixed = self.link.call("contact_rates")
            except Exception:
                mixed = None
            got = (mixed or {}).get("by_body") if isinstance(mixed, dict) else None
            if isinstance(got, dict) and got:
                for name, rate in got.items():
                    if isinstance(rate, (int, float)) and math.isfinite(float(rate)):
                        by_body[name] = max(by_body.get(name, 0.0), float(rate))
                src = ("the robot file the env host serves, and each of its geoms mixed with each scene geom it can "
                       "collide with by the simulator's own rule (solref and solimp by priority and solmix): per "
                       "body the stiffer of its own rate and the stiffest mixed one")
            tool = [by_body[m.body(b).name] for b in range(m.nbody) if m.body(b).name in by_body and under(b)]
            self._contact = ({"kind": "damped", "b_per_s": max(tool), "by_body": by_body,
                              "source": src} if tool else False)
        return dict(self._contact) if self._contact else None

    # ------------------------------------------------------------------ command and stepping
    def servo_goal(self, q_target):
        """The goal this arm's controller is driven to for an rcpx servo target (module docstring)."""
        q_target = _vec(q_target, self.n)
        if np.array_equal(q_target, self.q_cmd):
            return self.q_goal.copy()
        x = self.model.clip(q_target)
        for _ in range(LEAD_ITERS):
            nx = self.model.clip(q_target - self.model.gravity_torque(x) / self.kp)
            if float(np.max(np.abs(nx - x))) < 1e-10:
                x = nx
                break
            x = nx
        return x

    def command(self, q_target, torque_frac=1.0, gripper=None):
        q = _vec(q_target)
        if q.shape[0] != self.n or not np.all(np.isfinite(q)):
            raise ValueError(f"q_target needs {self.n} finite values")
        q = self.model.clip(q)
        goal = self.servo_goal(q)
        self.q_cmd, self.q_goal = q, goal
        self.torque_frac = float(np.clip(torque_frac, 0.0, 1.0))
        if gripper is not None:
            if gripper in ("open", "close"):
                self.grip = gripper
            else:
                self.grip = float(np.clip(float(gripper), 0.0, 1.0))

    def _grip_value(self):
        """The env's gripper action for the next control step: -1 opens, +1 closes, 0 holds the ramp where it is.
        A fraction moves the ramp toward 2 f - 1 (the env gripper only takes a direction)."""
        if self.grip == "open":
            return -1.0
        if self.grip == "close":
            return 1.0
        ramp = np.asarray(self._st.get("gripper_ramp") or [0.0], float).reshape(-1)
        want = 2.0 * float(self.grip) - 1.0
        gap = want - float(ramp[0])
        if abs(gap) <= GRIP_DEADBAND:
            return 0.0
        return -1.0 if gap > 0 else 1.0

    def step(self, n=1):
        n = int(n)
        while n > 0:
            if self.spent or self.lost:
                self._t_frozen += n * self.dt
                return
            g = self._grip_value()
            handover = self._st.get("mode") not in (None, "joint")
            k = 1 if (not isinstance(self.grip, str) and g != 0.0) else min(n, P.MAX_STEPS_PER_REQUEST)
            try:
                res = self.link.call("servo_joints", q=[float(v) for v in self.q_goal], n=k, gripper=g,
                                     torque_frac=max(self.torque_frac, FRAC_MIN))
            except P.EnvHostError as exc:
                if exc.kind == "transport":
                    self.lost = True
                raise
            self._g_sent = g
            ran = int(res.get("steps_run", 0))
            self.steps_run += ran
            self._st = res.get("state") or self._st
            if handover and ran:
                self.unjudged_t = float(self._st["t"])
            if ran < k or int(res.get("steps_left", 1)) <= 0:
                self.spent = True
                self._t_frozen += (k - ran) * self.dt
            n -= k

    def steps_left(self):
        return 0 if self.spent else self._st.get("steps_left")

    def refresh(self):
        """Read the host again (after anything but this backend moved the arm: a test driving the host itself)."""
        self._st = self.link.call("read")
        return self.read()

    # ------------------------------------------------------------------ reading
    def _set_kin(self, q):
        d = self._d
        d.qpos[:] = self._qfull
        d.qpos[self._qa] = q
        mujoco.mj_kinematics(self.model.mj, d)
        mujoco.mj_comPos(self.model.mj, d)

    def wrist(self, st=None):
        """(force N, torque N m) the robot puts on the world at the wrist sensor, frame B: the sensor's reading
        (the support of what hangs below it) plus that part's weight at the present pose."""
        st = self._st if st is None else st
        if st.get("wrist_force") is None:
            return None, None
        raw_f = _vec(st["wrist_force"], 3) + _vec(st.get("wrist_tare_force") or [0, 0, 0], 3)
        raw_t = _vec(st.get("wrist_torque") or [0, 0, 0], 3) + _vec(st.get("wrist_tare_torque") or [0, 0, 0], 3)
        self._set_kin(_vec(st["q"], self.n))
        R = self._d.site_xmat[self._ft_site].reshape(3, 3)
        p = self._d.site_xpos[self._ft_site]
        c = self._d.subtree_com[self._ft_body]
        w = self._ft_mass * self.g_B
        return R @ raw_f + w, R @ raw_t + np.cross(c - p, w)

    def read(self):
        st = self._st
        q, qd = _vec(st["q"], self.n), _vec(st["qd"], self.n)
        tau = _vec(st["tau"], self.n)
        out = {"t": float(st["t"]) + self._t_frozen, "q": q, "qd": qd, "tau": tau, "load": tau / self.limit,
               "torque_frac": self.torque_frac, "q_target": self.q_cmd.copy(), "servo_goal": self.q_goal.copy(),
               "gripper": None, "steps_left": self.steps_left()}
        if st.get("fingers") is not None:
            gq = _vec(st["fingers"])
            span = self._g_open - self._g_closed
            frac = float(np.mean((gq - self._g_closed) / np.where(np.abs(span) > 1e-9, span, 1.0)))
            ramp = _vec(st.get("gripper_ramp") or [0.0] * len(gq))
            if ramp.size == 1:
                ramp = np.array([ramp[0], -ramp[0]])
            lo, hi = self._g_ctrl[:, 0], self._g_ctrl[:, 1]
            target = lo + (hi - lo) * (np.clip(ramp[:len(gq)], -1, 1) + 1.0) / 2.0
            out["gripper"] = {"q": gq, "qd": _vec(st.get("fingers_vel") or [0.0] * len(gq)), "target": target,
                              "open_frac": frac}
        f, t = self.wrist(st)
        if f is not None:
            out["wrist_force_n"], out["wrist_torque_nm"] = f, t
        if st.get("wrist_force_step_mean") is not None:   # the sensor averaged over the step, when the host has it
            self._set_kin(q)
            R = self._d.site_xmat[self._ft_site].reshape(3, 3)
            out["wrist_force_mean_n"] = R @ _vec(st["wrist_force_step_mean"], 3) + self._ft_mass * self.g_B
        fast = self.fast_block(st)
        if fast is not None:
            out["fast"] = fast
        return out

    # cameras, depth and the wrist force at the physics rate: SenseMixin (bench/ext/libero/sense.py)
    def close(self):
        self.link.close()


# ---------------------------------------------------------------------- the observer with the arm's own servo law
class ServoLawMixin:
    """rcpx's force observer (rcpx/control_sense.py) with the servo law of this backend (module docstring). A
    backend without servo_law gets the round one law."""

    def _sl_fingers(self):
        cache = getattr(self, "_sl_cache", None)
        if cache is None:
            pm, g = self._pm, self.model.spec.get("gripper") or {}
            qa = [pm.jnt_qposadr[pm.joint(j).id] for j in g.get("joints", [])]
            da = [pm.jnt_dofadr[pm.joint(j).id] for j in g.get("joints", [])]
            act = [pm.actuator(a).id for a in g.get("actuators", [])]
            cache = self._sl_cache = (np.array(qa, int), np.array(da, int), np.array(act, int),
                                      np.zeros((pm.nv, pm.nv)), np.zeros((self.model.mj.nv, self.model.mj.nv)))
        return cache

    def _sl_goal(self, prev, q_target, law):
        goal = self.backend.servo_goal(q_target)
        step = law["goal_step_rad"]
        return prev.q + np.clip(goal - prev.q, -step, step)

    def _predict(self, prev, q_target, frac, h, first=None, second=None):
        law = getattr(self.backend, "servo_law", None)
        if law is None:
            return super()._predict(prev, q_target, frac, h, first, second)
        m, d = self._pm, self._pd
        qa, da, mot = self._pqa, self._pda, self._pmot
        gqa, gda, gact, M, _ = self._sl_fingers()
        d.qpos[:] = self._pfull
        d.qpos[qa] = prev.q
        d.qvel[:] = 0
        d.qvel[da] = prev.qd
        d.qacc_warmstart[:] = 0
        d.ctrl[:] = self._pctrl
        g = prev.gripper if isinstance(getattr(prev, "gripper", None), dict) else None
        if g is not None and len(gqa):
            d.qpos[gqa] = np.asarray(g["q"], float)
            d.qvel[gda] = np.asarray(g.get("qd", np.zeros(len(gda))), float)
            d.ctrl[gact] = np.asarray(g.get("target", g["q"]), float)
        d.qfrc_applied[:] = 0
        goal = self._sl_goal(prev, q_target, law)
        lim = self.torque_limit * frac
        kp, kd = law["kp"], law["kd"]
        steps = max(1, int(round(h / self.cfg["physics_dt_s"])))
        half = steps // 2
        for k in range(steps):
            if first is not None and (k == 0 or k == half):
                d.qfrc_applied[da] = first if k < half else second
            mujoco.mj_step1(m, d)
            _full_m(m, d, M)
            tau = M[np.ix_(da, da)] @ (kp * (goal - d.qpos[qa]) - kd * d.qvel[da]) + d.qfrc_bias[da]
            d.ctrl[mot] = np.minimum(np.maximum(tau, -lim), lim)
            mujoco.mj_step2(m, d)
        d.qfrc_applied[:] = 0
        return d.qpos[qa].copy(), d.qvel[da].copy()

    def _pick_estimate(self, prev, s, q_target, frac, h):
        """A period in which the env handed the arm to its joint controller is not judged: that controller's first
        step may start from the pose it last read (the host's handover), which no servo law predicts."""
        t = getattr(self.backend, "unjudged_t", None)
        if getattr(self.backend, "servo_law", None) is not None and t is not None and abs(s.t - t) < 1e-9:
            return np.zeros(self.model.n)
        return super()._pick_estimate(prev, s, q_target, frac, h)

    def _static_estimate(self, prev, s, q_target, frac, h):
        law = getattr(self.backend, "servo_law", None)
        if law is None:
            return super()._static_estimate(prev, s, q_target, frac, h)
        from rcpx.control_sense import _DH, _H

        m, d = self.model.mj, self._dd
        _, _, _, _, M = self._sl_fingers()
        goal = self._sl_goal(prev, q_target, law)
        lim = self.torque_limit * frac
        q = np.outer(_H[:, 0], prev.q) + np.outer(_H[:, 1], h * prev.qd) + np.outer(_H[:, 2], s.q) \
            + np.outer(_H[:, 3], h * s.qd)
        qd = (np.outer(_DH[:, 0], prev.q) + np.outer(_DH[:, 2], s.q)) / h + np.outer(_DH[:, 1], prev.qd) \
            + np.outer(_DH[:, 3], s.qd)
        taus = []
        for qi, qdi in zip(q, qd):
            d.qpos[:] = self._qfull
            d.qpos[self._qadr] = qi
            d.qvel[:] = 0
            d.qvel[self._dadr] = qdi
            mujoco.mj_forward(m, d)
            _full_m(m, d, M)
            t = M[np.ix_(self._dadr, self._dadr)] @ (law["kp"] * (goal - qi) - law["kd"] * qdi) \
                + d.qfrc_bias[self._dadr]
            taus.append(np.clip(t, -lim, lim))
        v = (s.q - prev.q) / h
        return np.mean(taus, axis=0) - self._model_torque(0.5 * (prev.q + s.q), v, (s.qd - prev.qd) / h)


Mixin = ServoLawMixin


def setup(ex):
    """Stop a motion once the host runs no more control steps (the budget is spent or the episode is over)."""
    be = ex.backend

    def tick(sense):
        if getattr(be, "spent", False) or getattr(be, "lost", False):
            return "steps_spent"
        return None
    ex.add_tick(tick)


# ---------------------------------------------------------------------- the vnext session for episode_ext
DEVICES_DOC = {"level": "none", "devices": [],
               "note": "no device knowledge for this task: find what the task names by looking and name it yourself"}
INSTALL_HOW = "how the robot is installed, as the person who set it up gave it"
JOURNAL_TAIL = 200


def task_sentence(facts):
    for key in ("language", "instruction", "task_sentence"):
        v = (facts or {}).get(key)
        if isinstance(v, str) and v.strip():
            return v.strip()
    raise ValueError("the episode facts carry no task sentence (language)")


def scene_camera_world(link, name, px=SCENE_PX):
    """The host's cam answer for a fixed camera, as build_spec takes it (robot_spec.camera_from_host)."""
    return RS.camera_from_host(link.call("cam", camera=name, width=int(px[0]), height=int(px[1])))


class _ExtSessionMixin:
    """Ends the session when the host runs no more control steps: that call's answer says the episode ended."""

    def dispatch(self, request):
        resp = super().dispatch(request)
        be = getattr(self, "ext_backend", None)
        if be is not None and not self.ended and (be.spent or be.lost):
            self._end("env_lost" if be.lost else "over", steps_run=be.steps_run)
            if isinstance(resp, dict):
                resp = dict(resp, episode="ended", message="the episode is over: no control steps are left")
        return resp


def _packages(packages, problems):
    """(mods, absent): the round two packages PACKAGES names (rcpx.r2.core.load: all, none or a comma list), then
    every named package outside core.PACKAGES (core.load keeps only its own list), imported by name; the scene
    camera package is added by build_session itself. A package that fails to import is reported and skipped."""
    import importlib
    import re

    from rcpx.r2 import core

    mods = core.load(packages)
    problems.update({k: str(v).strip().splitlines()[-1][:300] for k, v in core.load.problems.items()})
    absent = list(core.load.absent)
    names = [n.strip().lower() for n in str(packages or "").split(",") if n.strip()]
    for n in names:
        if n in core.PACKAGES or n in ("all", "none", "scene_camera") or not re.fullmatch(r"[a-z_][a-z0-9_]*", n):
            continue
        if any(m.__name__ == "rcpx.r2." + n for m in mods):
            continue
        try:
            mods.append(importlib.import_module("rcpx.r2." + n))
        except ModuleNotFoundError as exc:
            if exc.name == "rcpx.r2." + n:
                absent.append(n)
            else:
                problems[n] = f"{type(exc).__name__}: {exc}"[:300]
        except Exception as exc:
            problems[n] = f"{type(exc).__name__}: {exc}"[:300]
    return mods, absent


def build_session(env, ep_dir, facts, budget, packages="all", wrist_px=RS.WRIST_PX, selfknow=STATED,
                  selfknow_dir=None):
    """The vnext session for one LIBERO episode, served on EPDIR/rcp.sock by episode_ext (its hook). env: the env
    host client (or its socket path); facts: the episode facts (language, workspace_offset); budget: {"requests",
    "steps"}; packages: the cell's PACKAGES (all, none or a comma list) for the round two packages; the scene camera
    package and this arm's servo law are always on. selfknow: cfg selfknow (selfknow_mode: stated, else
    RCPX_SELFKNOW_MODE, else "require"); selfknow_dir: the record folder, else SELFKNOW_DIR."""
    import importlib
    import sys

    from bench.episode_r2_session import CLAIM_R2, SessionR2
    from rcpx import surfaces
    from rcpx.context import Journal, World, practices_text, read_journal, refusal
    from rcpx.model import RobotModel
    from rcpx.model_body import with_installation
    from rcpx.r2 import core

    selfknow = selfknow_mode(selfknow)   # before anything is built: a bad word builds nothing
    scene_mod = importlib.import_module("rcpx.r2.scene_camera")
    link = HostLink(env)
    sentence = task_sentence(facts)
    info = link.call("robot_xml")
    with open(info["path"]) as f:
        xml_text = f.read()
    st = link.call("read")
    cams = {name: scene_camera_world(link, name) for name in SCENE_CAMERAS}
    home, home_how = RS.home_pose(info, st)
    spec, sfacts = RS.build_spec(xml_text, home, LAW, wrist_px, cams, base_dir=os.path.dirname(info["path"]))
    if all(v is None for v in (spec.get("speed_limit") or [None])) and \
            len(spec.get("arm_joints", RATED_QD)) == len(RATED_QD):
        spec["speed_limit"] = list(RATED_QD)
        spec.setdefault("source", {})["speed_limit"] = RATED_QD_SHEET
    model = RobotModel(spec)
    backend = LiberoBackend(link, model, sfacts, LAW, state=st)
    ep_dir = os.path.abspath(ep_dir)
    journal = Journal(ep_dir)
    world = World(journal)
    problems = {}
    mods, absent = _packages(packages, problems)
    mods = mods + [scene_mod, sys.modules[__name__]]
    while True:
        try:
            # no site limit and no people fact are stated: the robot's own ratings and what
            # it measured on itself bound it; the support is the plane stated below with no error
            cfg = dict(RS.executor_cfg(facts, sfacts), selfknow=selfknow, selfknow_dir=selfknow_dir or SELFKNOW_DIR)
            ex = core.make_executor(model, backend, journal=journal, world=world, cfg=cfg, mods=mods)
            break
        except TypeError:   # a package whose Mixin clashes: dropped and reported, never fatal
            bad = next((mm for mm in mods[:-2] if not _class_ok(mm)), None)
            if bad is None:
                raise
            problems[bad.__name__.rsplit(".", 1)[-1]] = "its Mixin does not join the executor class"
            mods = [mm for mm in mods if mm is not bad]
    task_doc = {"instruction": sentence, "surface": "vnext", "knowledge_level": "none",
                "note": "the task is this sentence; nothing else about the scene is given"}
    ex.set_context(devices=DEVICES_DOC, task=task_doc, experience_path=None, think_s=0.0)

    def body():
        doc = with_installation(model.describe(), kind="fixed", standing_surface=ex.standing_surface(),
                                kind_how=INSTALL_HOW)
        doc["base"]["measured_give"] = ex.base_give
        return doc

    def device(id):
        return refusal("unknown_device", f"no device {id!r}: this task has no device knowledge",
                       "find what the task names by looking")

    def journal_doc():
        recs = read_journal(journal.path)
        return {"count": len(recs), "shown": min(len(recs), JOURNAL_TAIL), "records": recs[-JOURNAL_TAIL:]}

    base_res = {"robot://practices": practices_text, "robot://task": lambda: dict(task_doc),
                "robot://body": body, "robot://world": world.to_json, "robot://devices": lambda: dict(DEVICES_DOC),
                "robot://devices/{id}": device, "robot://journal": journal_doc}
    surface_t, res, handlers, box = core.assemble(ex, mods, surfaces.get("vnext"), base_res, {})
    holder = {}
    cls = type("SessionExt", (_ExtSessionMixin, SessionR2), {})
    session = cls(ex, surface_t, res, {"calls": int(budget["requests"]),
                                       "sim_seconds": int(budget["steps"]) / CONTROL_HZ},
                  journal=journal, ep_dir=ep_dir, handlers=handlers, claim=CLAIM_R2,
                  claim_check=_claim_check(mods, ex, holder, refusal))
    holder["session"] = session
    session.ext_backend = backend
    box.session = session
    rec = {"surface": "vnext", "requested": packages, "loaded": [mm.__name__.rsplit(".", 1)[-1] for mm in mods],
           "absent": absent, "problems": {**problems, **{k: str(v)[-300:] for k, v in ex.r2_problems.items()}},
           "spec": {"name": spec["name"], "q_home": spec["q_home"], "q_home_how": home_how, "kp": spec["kp"],
                    "kd": spec["kd"]},
           "selfknow_mode": selfknow or "cold", "floor_z_m": ex.cfg["floor_z_m"]}
    with open(os.path.join(ep_dir, "r2_problems.json"), "w") as f:
        json.dump(rec, f, indent=2, sort_keys=True, default=str)
    return session


def _claim_check(mods, ex, holder, refusal):
    """The claim's cannot check of the last package that has one (the person package), or None."""
    fns = [getattr(mm, "check_cannot") for mm in mods if callable(getattr(mm, "check_cannot", None))]
    if not fns:
        return None
    fn = fns[-1]

    def check(args):
        out = fn(ex, args, holder.get("session"))
        if isinstance(out, dict) and out.get("ok") is False:
            return refusal("cannot_evidence", "the cannot claim is missing: " + "; ".join(out.get("missing", [])),
                           "give a reason and cite the response seq of each call that shows it")
        return None
    return check


def _class_ok(mod):
    from rcpx.r2 import core

    try:
        core.executor_class([mod])
        return True
    except TypeError:
        return False
