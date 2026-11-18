# SPDX-License-Identifier: AGPL-3.0-only
"""MujocoBackend: the bench backend over a bench.scenes.Scene (SPEC section 3).

Servo law, evaluated every physics step of the scene's own model with the goal held over one control period
(SUBSTEPS physics steps: dt_s is that product, read from the scene, never typed):
    tau = kp (goal - q) - kd qd, clipped to +-torque_limit * torque_frac, through the torque motors;
    goal, held over a period, follows the commanded q_target, but from one period to the next it moves AWAY from the
    joint's present position by no more than the spec's RATED joint speed times the period (the commanded joint
    speed is clipped at the rating beside the torque clip; a joint with no rating has no speed clip). A goal that
    comes back toward the joint (a stop) is taken at once, and a path that runs at or under the rating gets exactly
    the unclipped law: goal = q_target. read() reports the goal in force as q_target.
A push is only as strong as the goal leads the position, and the cap ends it there. Every device's on_step runs
after every physics step. read()["fast"] is the joint speeds at every physics step of the last period;
contact_model() reads the contact rate of the robot's own geoms from the scene's model; servo_law() states the law
above (SPEC-R4 A8.4). A truth log (every device truth at 10 Hz, base pose, and events: device state changes,
new violations, interventions) is kept for scoring; truth() and truth_log() are never reachable from a tool.

At construction the scene settles for settle_s of simulated time with the servo holding the start pose (home unless
the setting starts the arm elsewhere: scene.start_q) (gravity sag, the pad taking the base's weight); t = 0 is the
end of that settle.

Body errors (the setting's body_error zero or sign, scene.body_error): the robot's own joint readings and commands
are in ITS coordinates, the model's joints are the truth, and q_true = home + sign * (q_reported - home) + offset.
read() reports q, qd and tau in the robot's coordinates and command() takes them, so a runtime that loads the
nominal spec sees a body that does not match its model. The lights follow the scene's light schedule
(scene.update_lights) every control period.
"""

import json

import mujoco
import numpy as np

from rcpx.backend import Backend

SUBSTEPS = 10
LOG_EVERY = 5  # control periods: 10 Hz


class MujocoBackend(Backend):
    def __init__(self, scene, settle_s=0.5):
        self.scene = scene
        self.m, self.d = scene.model, scene.data
        spec = scene.spec
        self.physics_dt = float(self.m.opt.timestep)
        self.substeps = SUBSTEPS
        self.dt_s = self.physics_dt * SUBSTEPS   # the loop period: the scene's own step times the steps per period
        ids = scene.ids
        self._qa, self._da, self._mot = ids["qadr"], ids["dadr"], ids["motors"]
        self.q_lo, self.q_hi = ids["q_lo"].copy(), ids["q_hi"].copy()
        self.kp = np.asarray(spec["kp"], float)
        self.kd = np.asarray(spec["kd"], float)
        self.limit = np.asarray(spec["torque_limit"], float)
        rated = spec.get("speed_limit") or [None] * len(self.kp)
        self.speed_limit = np.array([np.inf if v is None else float(v) for v in rated])
        self._fast = np.zeros((SUBSTEPS, len(self.kp)))
        self._contact_model = None
        g = spec.get("gripper")
        self._g = g
        self._g_open = np.asarray(g["open"], float) if g else None
        self._g_closed = np.asarray(g["closed"], float) if g else None
        err = getattr(scene, "body_error", None)
        n = len(self._qa)
        self._home = np.asarray(spec["q_home"], float).copy() if err is None else np.asarray(err["home"], float)
        self._sgn = np.ones(n) if err is None else np.asarray(err["sign"], float)
        self._off = np.zeros(n) if err is None else np.asarray(err["offset"], float)
        start = getattr(scene, "start_q", None)
        self.q_target = np.asarray(spec["q_home"] if start is None else start, float).copy()
        self._goal = self.q_target.copy()   # the goal in force: the commanded q_target, its speed clipped
        self.torque_frac = 1.0
        self._lights = hasattr(scene, "update_lights") and bool(getattr(scene, "perturbation", {}).get(
            "light_schedule"))
        self.g_target = self._g_closed.copy() if g else None
        self.tau = np.zeros(len(self._qa))
        self._renderers = {}
        self.render_failures = []     # failed offscreen renders (each is retried once; the second failure raises)
        self.render_retried = False   # whether the last frame handed out needed the retry
        self._periods = 0
        self.d.qpos[:] = scene.initial_qpos
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)
        for _ in range(int(round(settle_s / self.dt))):
            self._period()
        self.t0 = float(self.d.time)
        self._periods = 0
        self.base_start = scene.base_truth(self.d)
        self._last_state = {}
        self._last_viol = {}
        self.samples = []
        self.events = []
        self.substep_hooks = []   # scoring only: callables run after every physics step (bench harm monitor)
        self._watch(log_sample=True)

    # ------------------------------------------------------------------ servo and stepping
    def _true(self, q_reported):
        """True joint angles for the robot's own (reported) ones."""
        return self._home + self._sgn * (np.asarray(q_reported, float) - self._home) + self._off

    def _reported(self, q_true):
        """What the robot's encoders say at true joint angles q_true."""
        return self._home + self._sgn * (np.asarray(q_true, float) - self._home - self._off)

    def _period(self):
        m, d = self.m, self.d
        cap = self.limit * self.torque_frac
        if self._lights:
            self.scene.update_lights(float(d.time) - getattr(self, "t0", float(d.time)))
        # the commanded joint speed is clipped at the rating: the goal in force moves toward the commanded one, freely
        # as far as the joint's present position (a stop), beyond it by at most the rated speed times the period
        step, back = self.q_target - self._goal, self._reported(d.qpos[self._qa]) - self._goal
        free = np.where(step * back > 0, np.sign(step) * np.minimum(np.abs(step), np.abs(back)), 0.0)
        reach = self.speed_limit * self.dt_s
        self._goal = self._goal + free + np.clip(step - free, -reach, reach)
        for k in range(SUBSTEPS):
            # the servo runs on the robot's own readings; its torque acts on the true joint (sign: the other way)
            tau = self.kp * (self._goal - self._reported(d.qpos[self._qa])) - self.kd * self._sgn * d.qvel[self._da]
            tau = np.clip(tau, -cap, cap)
            d.ctrl[self._mot] = self._sgn * tau
            if self._g is not None:
                d.ctrl[self.scene.ids["grip_act"]] = self.g_target
            mujoco.mj_step(m, d)
            for dev in self.scene.devices:
                dev.on_step(m, d)
            for hook in getattr(self, "substep_hooks", ()):
                hook(m, d)
            self._fast[k] = self._sgn * d.qvel[self._da]
        self.tau = tau

    def command(self, q_target, torque_frac=1.0, gripper=None):
        q = np.asarray(q_target, float).reshape(-1)
        if q.shape[0] != len(self._qa) or not np.all(np.isfinite(q)):
            raise ValueError(f"q_target needs {len(self._qa)} finite values")
        self.q_target = np.clip(q, self.q_lo, self.q_hi)
        self.torque_frac = float(np.clip(torque_frac, 0.0, 1.0))
        if gripper is not None and self._g is not None:
            if gripper == "open":
                f = 1.0
            elif gripper == "close":
                f = 0.0
            else:
                f = float(np.clip(float(gripper), 0.0, 1.0))
            self.g_target = self._g_closed + f * (self._g_open - self._g_closed)

    def step(self, n=1):
        for _ in range(int(n)):
            self._period()
            self._periods += 1
            self._watch(log_sample=self._periods % LOG_EVERY == 0)

    def read(self):
        d = self.d
        out = {"t": float(d.time) - self.t0, "q": self._reported(d.qpos[self._qa]),
               "qd": self._sgn * d.qvel[self._da],
               "tau": self.tau.copy(), "load": self.tau / self.limit, "torque_frac": self.torque_frac,
               "q_target": self._goal.copy(), "gripper": None,
               "fast": {"dt_s": self.physics_dt, "qd": self._fast.copy()}}
        if self._g is not None:
            gq = d.qpos[self.scene.ids["grip_qadr"]].copy()
            span = self._g_open - self._g_closed
            frac = float(np.mean((gq - self._g_closed) / np.where(np.abs(span) > 1e-9, span, 1.0)))
            out["gripper"] = {"q": gq, "target": self.g_target.copy(), "open_frac": frac}
        return out

    # ------------------------------------------------------------------ what the backend states of itself
    def servo_law(self):
        """The law of _period: a joint PD with a torque clip, its goal's speed clipped at the rated joint speed."""
        return {"kind": "joint_pd", "kp": self.kp.copy(), "kd": self.kd.copy(), "torque_limit": self.limit.copy(),
                "speed_limit": [None if not np.isfinite(v) else float(v) for v in self.speed_limit],
                "goal": "the command, held over a period; from period to period it moves away from the joint no "
                        "faster than speed_limit",
                "step_s": self.physics_dt, "dt_s": self.dt_s}

    def contact_model(self):
        """{"kind": "damped", "b_per_s", "by_body"}: the contact rate (1/s: force per unit of moving mass and
        arrival speed) of the robot's own collision geoms, read from the scene's model. A geom's contact is a
        damped spring given either as (time constant, damping ratio), then b = 2 / (d timeconst), or directly as
        (-stiffness, -damping), then b = damping / d; d is the top of its impedance, and the simulator keeps a time
        constant at or over two of its steps unless that guard is switched off. by_body gives each robot body the
        stiffest of its geoms; b_per_s is the stiffest of the tool's (the last arm joint's body and all under it).
        A body with no collision geom has no entry; a tool with none gives None (no contact model)."""
        if self._contact_model is None:
            m = self.m
            safe = not (int(m.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_REFSAFE))
            top = float(getattr(mujoco, "mjMAXIMP", 1))
            by_body = {}
            robot = self.scene.ids.get("robot_bodies")
            if robot is None:   # a scene that lists none: every body under the first arm joint's root
                first = int(m.jnt_bodyid[m.joint(self.scene.spec["arm_joints"][0]).id])
                robot = [b for b in range(m.nbody) if _under(m, b, int(m.body_rootid[first]))]
            for g in range(m.ngeom):
                b = int(m.geom_bodyid[g])
                if b not in robot or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                    continue
                d = min(float(m.geom_solimp[g][1]), top)
                ref = m.geom_solref[g]
                if d <= 0:
                    continue
                if ref[0] > 0:
                    tc = max(float(ref[0]), 2 * float(m.opt.timestep)) if safe else float(ref[0])
                    rate = 2 / (d * tc)
                else:
                    rate = -float(ref[1]) / d
                name = m.body(b).name
                by_body[name] = max(by_body.get(name, 0.0), rate)
            last = int(m.jnt_bodyid[m.joint(self.scene.spec["arm_joints"][-1]).id])
            tool = [by_body[m.body(b).name] for b in range(m.nbody)
                    if m.body(b).name in by_body and _under(m, b, last)]
            self._contact_model = ({"kind": "damped", "b_per_s": max(tool), "by_body": by_body, "source": "model"}
                                   if tool else False)
        return dict(self._contact_model) if self._contact_model else None

    # ------------------------------------------------------------------ camera
    def render(self, camera="wrist"):
        if camera not in ("wrist", "overview"):
            raise ValueError("camera is wrist (overview is for reports only)")
        cid = self.m.camera(camera).id
        w, h = (int(x) for x in self.m.cam_resolution[cid]) if camera == "wrist" else (640, 480)
        key = (camera, w, h)
        for attempt in (0, 1):   # a failed offscreen render is the harness's fault, not the robot's: one retry
            try:
                if key not in self._renderers:
                    self._renderers[key] = mujoco.Renderer(self.m, height=h, width=w)
                r = self._renderers[key]
                r.update_scene(self.d, camera=camera)
                img = np.array(r.render(), dtype=np.uint8)
                self.render_retried = attempt > 0   # read by the look that asked for this frame
                return img
            except Exception as exc:
                bad = self._renderers.pop(key, None)   # a fresh renderer for the retry
                try:
                    if bad is not None:
                        bad.close()
                except Exception:
                    pass
                self.render_failures.append({"t": float(self.d.time), "camera": camera, "attempt": attempt,
                                             "error": f"{type(exc).__name__}: {exc}"[:300]})
                if attempt:
                    raise

    def camera_info(self, camera="wrist"):
        if camera != "wrist":
            raise ValueError("only the wrist camera is a robot sensor")
        cid = self.m.camera("wrist").id
        w, h = (int(x) for x in self.m.cam_resolution[cid])
        fovy = float(self.m.cam_fovy[cid])
        f = (h / 2.0) / np.tan(np.radians(fovy) / 2.0)
        return {"name": "wrist", "width": w, "height": h, "fovy_deg": fovy, "fx": float(f), "fy": float(f),
                "cx": w / 2.0, "cy": h / 2.0, "model": "pinhole, no distortion; u right, v down, origin top left"}

    def close(self):
        for r in self._renderers.values():
            r.close()
        self._renderers = {}

    # ------------------------------------------------------------------ interventions
    def apply_intervention(self, kind):
        """Move the base or the device a few cm and degrees between control steps (bench only)."""
        rec = self.scene.apply_intervention(kind, self.d)
        rec = dict(rec, t=round(float(self.d.time) - self.t0, 4))
        self.events.append(dict(rec, event="intervention"))
        return rec

    def maybe_intervene(self, call_index):
        """Apply the setting's scheduled intervention once, when the operator's call count reaches at_call."""
        iv = self.scene.intervention
        if not iv or getattr(self, "_intervened", False) or int(call_index) < int(iv["at_call"]):
            return None
        self._intervened = True
        return self.apply_intervention(iv["kind"])

    # ------------------------------------------------------------------ scoring only
    def truth(self):
        """Scoring only; never reachable from a tool."""
        tr = self.scene.truth(self.d)
        tr["t"] = float(self.d.time) - self.t0
        tr["base_start"] = self.base_start
        return tr

    def _watch(self, log_sample):
        t = round(float(self.d.time) - self.t0, 4)
        devs = {}
        for dev in self.scene.devices:
            tr = dev.truth(self.m, self.d)
            devs[dev.name] = tr
            st = tr.get("state")
            if dev.name in self._last_state and st != self._last_state[dev.name]:
                self.events.append({"t": t, "event": "state", "device": dev.name, "from": self._last_state[dev.name],
                                    "to": st})
            self._last_state[dev.name] = st
            viol = list(tr.get("violations", []))
            for v in viol:
                if v not in self._last_viol.get(dev.name, []):
                    self.events.append({"t": t, "event": "violation", "device": dev.name, "what": v})
            self._last_viol[dev.name] = viol
        if log_sample:
            self.samples.append({"t": t, "devices": _jsonable(devs), "base": self.scene.base_truth(self.d)})

    def truth_log(self):
        """{"samples": 10 Hz device truths and base pose, "events": [...], scene facts}. Scoring only."""
        sc = self.scene
        return {"robot": sc.robot, "task": sc.task, "device": sc.device_kind, "setting": sc.setting.get("name"),
                "seed": sc.seed, "base_kind": sc.base_kind, "base_start": self.base_start,
                "samples": list(self.samples), "events": list(self.events),
                "render_failures": list(self.render_failures)}

    def save_truth_log(self, path):
        with open(path, "w") as f:
            json.dump(_jsonable(self.truth_log()), f)


def _under(m, b, root):
    """True when body b is root or hangs under it."""
    while b > 0:
        if b == root:
            return True
        b = int(m.body_parentid[b])
    return False


def _jsonable(x):
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return [_jsonable(v) for v in x.tolist()]
    if isinstance(x, (np.floating,)):
        return float(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.bool_,)):
        return bool(x)
    return x
