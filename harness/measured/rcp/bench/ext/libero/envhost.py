# SPDX-License-Identifier: AGPL-3.0-only
"""LIBERO env host: one LIBERO episode per reset, served over two unix sockets (bench/ext/envproto.py).

Runs in the benchmark's own Python (~/bx/libero-env, Python 3.10, mujoco 3.2.3, robosuite 1.4.0); start it with
bench.ext.libero.launch, which sets LIBERO_CONFIG_PATH, the renderer and a clean environment:
    PY -s -m bench.ext.libero.envhost --runtime base|pro --sock PATH --scoring-dir DIR [--files-dir DIR]
        [--gl egl|osmesa] [--idle-s S]          (cwd = ROOT)

reset(cell) builds the env exactly as the reference evaluations do: OffScreenRenderEnv with 256 px cameras and
ignore_done (the horizon never raises), env.seed(142 + init), env.reset(), the stored init state, 10 settle steps of
[0, 0, 0, 0, 0, 0, -1] (gripper open), then the step counter starts at 0. The two observation cameras are switched
off after the build (renders happen on request), which leaves the physics bit for bit unchanged (the test proves it).

Control. act sends native 7-D actions (OSC_POSE delta). servo_pose sends absolute OSC_POSE targets (the controller's
use_delta off). servo_joints swaps in robosuite's JOINT_POSITION controller and drives it closed loop on the joint
goal (each step: clip((q_goal - q) / J, -s, s), s = max_step / J, J = L.JOINT_SERVO step_rad; kp and damping from
L.JOINT_SERVO, not robosuite's default kp 50 and 0.05 rad). gripper holds the arm on its last servo goal
(or the pose it is in) while the gripper runs. A switch back to OSC re-bases its goal on the present pose.
torque_frac below 1 clips each controller's torque output to that fraction of the joint limits for that request.

Every control step counts against the cell's budget (envproto.StepLedger) and the task's goal predicates are
evaluated after every step and written with the step index to SCORING_DIR/truth_log.jsonl. The public socket never
answers object poses, object state, object or region names, contact names or the success flag; any answer whose
text would name a scene item is withheld (kind "withheld") and the reason goes to SCORING_DIR/host.log. The scoring
socket (SCORING_DIR/truth.sock) answers status, truth, contacts and audit renders.

render takes depth: true: the same render call with its depth flag, turned to
metres along the optical axis with the model's near and far planes and written beside the picture as a float32
.npy; a render is no control step, so the step counter does not move. Every state carries "fast": the raw wrist
force sensor and the arm's joint speeds after every physics step of the last control step (the body's own signals
at its own rate; "qd0" is the joint speed before that step's first physics step).
"""

import argparse
import hashlib
import json
import math
import os
import sys
import time
import traceback

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from bench.ext import envproto as P  # noqa: E402
from bench.ext import libero as L  # noqa: E402
from bench.ext.libero import hostsense as HS  # noqa: E402
from bench.ext.libero import hosttruth as HT  # noqa: E402

PUBLIC_CAMERAS = {"agentview": "agentview", "wrist": "robot0_eye_in_hand", "robot0_eye_in_hand": "robot0_eye_in_hand"}
SETTLE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
ROBOT_PREFIXES = L.ROBOT_PREFIXES
JP_KEYS = ("robot_name", "sim", "eef_name", "eef_rot_offset", "joint_indexes", "actuator_range", "policy_freq", "ndim")
IMAGE_OBS = ("agentview_image", "robot0_eye_in_hand_image")


BadArgs, NoEpisode, WrongRuntime, Withheld, KINDS = L.BadArgs, L.NoEpisode, L.WrongRuntime, L.Withheld, L.KINDS
_floats, _count, _gripper, _frac = L.floats_arg, L.count_arg, L.gripper_arg, L.frac_arg


class Host(L.ScoringMixin, HT.TruthMixin):
    """One env host: the public ops (op_*) here, the scoring ops (sc_*) in bench.ext.libero.ScoringMixin and, for
    the gates' instruments (segment, ray, things, shapes, peaks), in bench.ext.libero.hosttruth.TruthMixin."""
    cameras, robot_prefixes, BadArgs, plain = PUBLIC_CAMERAS, ROBOT_PREFIXES, BadArgs, staticmethod(P.plain)

    def __init__(self, runtime, scoring_dir, files_dir, gl):
        import imageio
        import mujoco
        import numpy as np
        import robosuite.utils.transform_utils as T
        from libero.libero import benchmark, get_libero_path
        from libero.libero.envs import OffScreenRenderEnv
        from robosuite.controllers import controller_factory, load_controller_config
        import libero.libero as libero_pkg
        self.np, self.mj, self.T, self.imageio = np, mujoco, T, imageio
        self.benchmark, self.get_libero_path, self.Env = benchmark, get_libero_path, OffScreenRenderEnv
        self.controller_factory, self.load_controller_config = controller_factory, load_controller_config
        self.libero_root = os.path.dirname(os.path.abspath(libero_pkg.__file__))
        want = L.libero_package_root(runtime)
        if not self.libero_root.startswith(want):
            raise RuntimeError(f"runtime {runtime} loaded libero from {self.libero_root}, expected under {want}")
        self.runtime, self.gl = runtime, gl
        self.scoring_dir, self.files_dir = scoring_dir, files_dir
        os.makedirs(files_dir, exist_ok=True)
        self.env = None
        self.ep = -1
        self.req = None
        self.renders = 0
        self.stop = False
        self._log = open(os.path.join(scoring_dir, L.TRUTH_LOG), "a", encoding="utf-8")
        self._hostlog = open(os.path.join(scoring_dir, L.HOST_LOG), "a", encoding="utf-8")

    # -- logs --------------------------------------------------------------------------------------------------------
    def note(self, text):
        self._hostlog.write(text.rstrip("\n") + "\n")
        self._hostlog.flush()

    def truth_line(self, rec):
        self._log.write(json.dumps(P.plain(dict(rec, ep=self.ep)), separators=(",", ":")) + "\n")

    # -- episode build -----------------------------------------------------------------------------------------------
    def op_reset(self, args):
        cell = args.get("cell")
        if not isinstance(cell, dict):
            raise BadArgs("reset needs cell: {suite, task, init, runtime, budget_steps}")
        if cell.get("runtime", "base") != self.runtime:
            raise WrongRuntime(f"this host runs the {self.runtime} runtime; start a {cell.get('runtime')} host")
        task_i, init_i = cell.get("task"), cell.get("init")
        if not isinstance(task_i, int) or not isinstance(init_i, int) or task_i < 0 or init_i < 0:
            raise BadArgs("cell task and init must be integers of 0 or more")
        budget = cell.get("budget_steps", 900)
        seed = cell.get("env_seed", 142 + init_i)
        px = cell.get("camera_px", 256)
        if not all(isinstance(v, int) and not isinstance(v, bool) for v in (budget, seed, px)) or budget < 1:
            raise BadArgs("budget_steps, env_seed and camera_px must be integers (budget at least 1)")
        bd = self.benchmark.get_benchmark_dict()
        if cell.get("suite") not in bd:
            raise BadArgs(f"suite {cell.get('suite')!r} is not in this runtime's benchmark")
        if self.env is not None:
            self._end_episode("reset")
        b = bd[cell["suite"]]()
        if task_i >= b.get_num_tasks():
            raise BadArgs(f"task {task_i} is not in suite {cell['suite']} ({b.get_num_tasks()} tasks)")
        task = b.get_task(task_i)
        init_path = os.path.join(self.get_libero_path("init_states"), task.problem_folder, task.init_states_file)
        inits = L.load_init_states(init_path)
        if init_i >= len(inits):
            raise BadArgs(f"init {init_i} is not stored for this task ({len(inits)} init states)")
        self.bddl = b.get_task_bddl_file_path(task_i)
        env, retries = self._build(px, seed, inits[init_i])
        self.obs_images = bool(args.get("obs_images", False))
        if not self.obs_images:
            for name in IMAGE_OBS:
                env.env.modify_observable(name, "enabled", False)
        self.env, self.inner = env, env.env
        self.robot = r = self.inner.robots[0]
        self.m, self.d = self.inner.sim.model._model, self.inner.sim.data._data
        self.osc = r.controller
        jp_cfg = self.load_controller_config(default_controller="JOINT_POSITION")
        jp_cfg.update({k: r.controller_config[k] for k in JP_KEYS})
        js = L.JOINT_SERVO
        jp_cfg.update(kp=js["kp"], damping_ratio=js["damping_ratio"], output_max=js["step_rad"],
                      output_min=-js["step_rad"])
        self.jp = self.controller_factory(jp_cfg["type"], jp_cfg)
        self.jp_scale = float(jp_cfg["output_max"])
        self.torque_frac = 1.0
        for ctrl in (self.osc, self.jp):
            self._cap(ctrl)
        self.mode, self.hold, self.grip_cmd = "osc_delta", None, SETTLE[6]
        self.ep += 1
        self.cell, self.init_len, self.seed = dict(cell), len(inits), seed
        self.goal_state = [list(s) for s in self.inner.parsed_problem["goal_state"]]
        self.secrets = self._secret_names()
        self.ledger = None
        self._watch_force()
        for _ in range(10):
            self._step(SETTLE)
        self.tare_f = self.np.array(r.ee_force, dtype=float)
        self.tare_t = self.np.array(r.ee_torque, dtype=float)
        self.ledger = P.StepLedger(budget, end_on_success=bool(args.get("end_on_success", True)))
        goals = self._goals()
        self.truth_line({"event": "reset", "cell": self.cell, "language": env.language_instruction,
                         "bddl": self.bddl, "env_seed": seed, "settle_steps": 10, "goal_state": self.goal_state,
                         "goals_after_settle": goals, "success_after_settle": all(goals),
                         "objects": self._poses(), "state_sha256": self._state_hash(), "libero": self.libero_root,
                         "obs_images": self.obs_images, "end_on_success": self.ledger.end_on_success,
                         "renderer_retries": retries})
        self._log.flush()
        return {"step": 0, "steps_left": budget, "dt": float(self.inner.control_timestep), "state": self._state()}

    def _build(self, px, seed, init_state, attempts=4):
        """The reference build. A renderer that cannot make its framebuffer (EGL under GPU memory pressure) fails
        the whole build; the build is then retried from scratch after a pause, up to attempts times."""
        retries = []
        for k in range(attempts):
            try:
                env = self.Env(bddl_file_name=self.bddl, camera_heights=px, camera_widths=px, ignore_done=True)
                env.seed(seed)
                env.reset()
                env.set_init_state(init_state)
                return env, retries
            except self.mj.FatalError as exc:
                if "framebuffer" not in str(exc).lower() or k == attempts - 1:
                    raise
                retries.append(str(exc)[:200])
                self.note(f"reset: renderer failed ({exc}); build retried in {2 * (k + 1)} s")
                time.sleep(2 * (k + 1))

    def _cap(self, ctrl):
        run = ctrl.run_controller

        def capped():
            tau = run()
            if self.torque_frac < 1.0:
                lo, hi = self.robot.torque_limits
                tau = self.np.clip(tau, lo * self.torque_frac, hi * self.torque_frac)
            return tau
        ctrl.run_controller = capped

    def _secret_names(self):
        names = set(self.inner.objects_dict) | set(self.inner.fixtures_dict) | set(self.inner.object_sites_dict)
        for s in self.goal_state:
            names.update(str(x) for x in s[1:])
        for kind, count in ((self.mj.mjtObj.mjOBJ_BODY, self.m.nbody), (self.mj.mjtObj.mjOBJ_GEOM, self.m.ngeom),
                            (self.mj.mjtObj.mjOBJ_SITE, self.m.nsite), (self.mj.mjtObj.mjOBJ_JOINT, self.m.njnt)):
            for i in range(count):
                n = self.mj.mj_id2name(self.m, kind, i)
                if n and n != "world" and not n.startswith(ROBOT_PREFIXES):
                    names.add(n)
        return sorted(n for n in names if len(n) >= 4)

    def _end_episode(self, why):
        if self.ledger is not None:
            self.truth_line({"event": "end", "why": why, "step": self.ledger.step,
                             "first_success_step": self.ledger.first_success, "objects": self._poses(),
                             "state_sha256": self._state_hash()})
            self._log.flush()
        self.env.close()
        self.env = None

    def _need_env(self):
        if self.env is None:
            raise NoEpisode("no episode: send reset first")

    # -- stepping ----------------------------------------------------------------------------------------------------
    def _goals(self):
        return [bool(self.inner._eval_predicate(s)) for s in self.goal_state]

    def _watch_force(self):
        """Keep the raw wrist force sensor and the arm's joint speeds after every physics step (hostsense)."""
        adr = self.m.sensor_adr[self.mj.mj_name2id(self.m, self.mj.mjtObj.mjOBJ_SENSOR,
                                                   self.robot.gripper.important_sensors["force_ee"])]
        self.watch = HS.FastWatch(self.inner.sim, self.d, self.np, adr, self.robot._ref_joint_vel_indexes)
        names = [self.mj.mj_id2name(self.m, self.mj.mjtObj.mjOBJ_BODY, b) or "" for b in range(self.m.nbody)]
        robot = [b for b in range(self.m.nbody) if names[b].startswith(ROBOT_PREFIXES)]   # scoring only (peaks)
        self.pairs = HT.PairWatch(self.inner.sim, self.m, self.d, self.np, self.mj, robot, self.robot.eef_site_id)

    def _step(self, action):
        """One control step; the watch keeps its physics steps' force and joint speeds. Returns env.step's done."""
        self.watch.begin()
        self.pairs.begin(self.ledger.step + 1 if self.ledger is not None else None)
        _, _, done, _ = self.env.step(action)
        self.watch.end()
        self.pairs.end()
        return done

    def _use(self, mode):
        """Put the controller the next steps need in place; OSC re-bases its goal after joint control."""
        if mode == self.mode:
            return
        if mode == "joint":   # robosuite skips update() until the controller has run: force it, never a stale pose
            self.robot.controller = self.jp
            self.jp.update(force=True)
        else:
            if self.mode == "joint":
                self.robot.controller = self.osc
                self.osc.update(force=True)
                self.osc.reset_goal()
            self.osc.use_delta = mode == "osc_delta"
        self.inner._action_dim = self.robot.action_dim
        self.mode = mode

    def _run(self, n, mode, action_fn, frac=1.0):
        self._use(mode)
        k = self.ledger.allowed(n)
        ran = 0
        self.torque_frac = frac
        try:
            for _ in range(k):
                a = action_fn()
                done = self._step(a)
                goals = self._goals()
                ran += 1
                more = self.ledger.record(bool(done))
                rec = {"step": self.ledger.step, "req": self.req, "success": bool(done), "goals": goals}
                if bool(done) != all(goals):
                    rec["mismatch"] = True
                self.truth_line(rec)
                if self.ledger.first_success == self.ledger.step:
                    self.truth_line({"event": "first_success", "step": self.ledger.step})
                if not more:
                    break
        finally:
            self.torque_frac = 1.0
            if ran:
                self.grip_cmd = float(a[-1])
            self._log.flush()
        out = {"steps_run": ran, "step": self.ledger.step, "steps_left": self.ledger.steps_left}
        if ran < n:
            out["note"] = f"{ran} of {n} steps ran; the episode has {self.ledger.steps_left} steps left"
        return dict(out, state=self._state())

    def op_act(self, args):
        self._need_env()
        if "actions" in args:
            if "action" in args or "n" in args:
                raise BadArgs("send actions (a list, one per step) or action with n, not both")
            acts = args["actions"]
            if not isinstance(acts, list) or not 1 <= len(acts) <= P.MAX_STEPS_PER_REQUEST:
                raise BadArgs(f"actions is a list of 1 to {P.MAX_STEPS_PER_REQUEST} actions")
            acts = [_floats(a, 7, "each action") for a in acts]
        else:
            acts = [_floats(args.get("action"), 7, "action")] * _count(args)
        it = iter(acts)
        self.hold = None
        return self._run(len(acts), "osc_delta", lambda: self.np.array(next(it)))

    def _axisangle(self, args):
        given = [k for k in ("axisangle", "mat", "quat_xyzw") if args.get(k) is not None]
        if len(given) > 1:
            raise BadArgs("give at most one of axisangle, mat, quat_xyzw")
        T, np = self.T, self.np
        if not given:   # hold the orientation goal the controller has now (re-based after joint control)
            if self.mode == "joint":
                self._use("osc_abs")
            return T.quat2axisangle(T.mat2quat(np.array(self.osc.goal_ori))).tolist()
        k = given[0]
        if k == "axisangle":
            return _floats(args[k], 3, "axisangle")
        if k == "quat_xyzw":
            q = np.array(_floats(args[k], 4, "quat_xyzw"))
            if not 0.5 < np.linalg.norm(q) < 1.5:
                raise BadArgs("quat_xyzw must be close to unit length")
            return T.quat2axisangle(q / np.linalg.norm(q)).tolist()
        flat = args[k] if not (isinstance(args[k], list) and args[k] and isinstance(args[k][0], list)) else \
            [x for row in args[k] for x in row]
        mat = np.array(_floats(flat, 9, "mat")).reshape(3, 3)
        if np.abs(mat @ mat.T - np.eye(3)).max() > 1e-3:
            raise BadArgs("mat must be a rotation matrix (rows orthonormal)")
        return T.quat2axisangle(T.mat2quat(mat)).tolist()

    def op_servo_pose(self, args):
        self._need_env()
        pos = _floats(args.get("pos"), 3, "pos")
        n, frac = _count(args), _frac(args)
        aa = self._axisangle(args)
        g = _gripper(args.get("gripper"), self.grip_cmd)
        self.hold = ("pose", pos, aa)
        action = self.np.array(pos + aa + [g])
        return self._run(n, "osc_abs", lambda: action, frac)

    def _joint_action(self, q_goal, step_max, g):
        np = self.np
        idx = self.robot._ref_joint_pos_indexes
        s = step_max / self.jp_scale

        def make():
            q = np.array(self.d.qpos[idx])
            return np.concatenate([np.clip((q_goal - q) / self.jp_scale, -s, s), [g]])
        return make

    def op_servo_joints(self, args):
        self._need_env()
        q = self.np.array(_floats(args.get("q"), 7, "q"))
        lo, hi = self.m.jnt_range[self.robot._ref_joint_indexes].T
        q_goal = self.np.clip(q, lo, hi)
        step_max = args.get("max_step", self.jp_scale)
        if isinstance(step_max, bool) or not isinstance(step_max, (int, float)) or \
                not 0.0 < float(step_max) <= self.jp_scale:
            raise BadArgs(f"max_step (rad per control step) must be above 0 and at most {self.jp_scale}")
        n, frac = _count(args), _frac(args)
        g = _gripper(args.get("gripper"), self.grip_cmd)
        self.hold = ("joint", q_goal)
        out = self._run(n, "joint", self._joint_action(q_goal, float(step_max), g), frac)
        if self.np.any(q_goal != q):
            out["q_clipped"] = q_goal.tolist()
        return out

    def op_gripper(self, args):
        self._need_env()
        if args.get("command") is None:
            raise BadArgs('gripper needs command: "open", "close" or a number from -1 to 1')
        g = _gripper(args.get("command"), self.grip_cmd)
        n, frac = _count(args), _frac(args)
        if self.hold is None:   # after native actions: hold the pose the hand is in
            T = self.T
            pos = self.d.site_xpos[self.robot.eef_site_id].tolist()
            mat = self.d.site_xmat[self.robot.eef_site_id].reshape(3, 3)
            self.hold = ("pose", pos, T.quat2axisangle(T.mat2quat(self.np.array(mat))).tolist())
        if self.hold[0] == "joint":
            return self._run(n, "joint", self._joint_action(self.hold[1], self.jp_scale, g), frac)
        action = self.np.array(list(self.hold[1]) + list(self.hold[2]) + [g])
        return self._run(n, "osc_abs", lambda: action, frac)

    # -- reading -----------------------------------------------------------------------------------------------------
    def _state(self):
        np, r, m, d = self.np, self.robot, self.m, self.d
        sid = r.eef_site_id
        mat = np.array(d.site_xmat[sid]).reshape(3, 3)
        vel = np.zeros(6)
        self.mj.mj_objectVelocity(m, d, self.mj.mjtObj.mjOBJ_SITE, sid, vel, 0)
        led = self.ledger
        step = led.step if led else 0
        out = {"step": step, "steps_left": led.steps_left if led else None,
               "t": round(step * self.inner.control_timestep, 6),
               "q": d.qpos[r._ref_joint_pos_indexes], "qd": d.qvel[r._ref_joint_vel_indexes],
               "tau": np.array(r.torques), "tau_limit": r.torque_limits[1],
               "fingers": d.qpos[r._ref_gripper_joint_pos_indexes],
               "fingers_vel": d.qvel[r._ref_gripper_joint_vel_indexes],
               "gripper_cmd": self.grip_cmd, "gripper_ramp": np.array(r.gripper.current_action),
               "eef_pos": d.site_xpos[sid], "eef_mat": mat,
               "eef_quat_xyzw": self.T.mat2quat(mat), "eef_axisangle": self.T.quat2axisangle(self.T.mat2quat(mat)),
               "eef_lin_vel": vel[3:], "eef_ang_vel": vel[:3],
               "wrist_force": np.array(r.ee_force) - self.tare_f, "wrist_torque": np.array(r.ee_torque) - self.tare_t,
               "wrist_tare_force": self.tare_f, "wrist_tare_torque": self.tare_t,
               "wrist_force_step_mean": self.watch.mean, "mode": self.mode,
               "fast": self.watch.block(m.opt.timestep)}
        if self.hold is not None and self.hold[0] == "joint":
            out["q_target"] = self.hold[1]
        return P.plain(out)

    def op_read(self, args):
        self._need_env()
        return self._state()

    def op_steps_left(self, args):
        self._need_env()
        return {"steps_left": self.ledger.steps_left, "step": self.ledger.step, "budget": self.ledger.budget}

    def _size(self, args):
        w, h = args.get("width", 256), args.get("height", 256)
        if not all(isinstance(v, int) and not isinstance(v, bool) and 16 <= v <= 2048 for v in (w, h)):
            raise BadArgs("width and height are integers from 16 to 2048")
        return w, h

    def _render(self, cam, label, args, default_dir):
        w, h = self._size(args)
        depth = args.get("depth", False)
        if not isinstance(depth, bool):
            raise BadArgs("depth is true or false")
        kw = {"depth": True} if depth else {}
        try:
            raw = self.inner.sim.render(camera_name=cam, width=w, height=h, **kw)
        except self.mj.FatalError as exc:   # a framebuffer lost to GPU memory pressure: one retry after a pause
            self.note(f"render: renderer failed ({exc}); retried once")
            time.sleep(1.0)
            raw = self.inner.sim.render(camera_name=cam, width=w, height=h, **kw)
        raw, buf = raw if depth else (raw, None)
        img = self.np.ascontiguousarray(raw[::-1])
        path = args.get("path")
        if path is None:
            self.renders += 1
            path = os.path.join(default_dir, f"e{self.ep:02d}_s{self.ledger.step:04d}_{label}_{self.renders:04d}.png")
        elif not (isinstance(path, str) and os.path.isabs(path) and path.endswith(".png")
                  and os.path.isdir(os.path.dirname(path))):
            raise BadArgs("path must be an absolute .png path in an existing folder")
        self.imageio.imwrite(path, img)
        out = {"path": path, "camera": label, "width": w, "height": h, "step": self.ledger.step,
               "pixels_sha256": hashlib.sha256(img.tobytes()).hexdigest()}
        if depth:   # the buffer's values as metres along the optical axis, beside the picture
            facts = HS.depth_facts(self.m, self.np, os.environ.get("MUJOCO_GL"))
            dpath = path[:-len(".png")] + "_depth.npy"
            self.np.save(dpath, HS.depth_metres(buf, facts, self.np))
            out["depth"] = dict(facts, path=dpath)
        return out

    def op_render(self, args):
        self._need_env()
        cam = args.get("camera", "agentview")
        if cam not in PUBLIC_CAMERAS:
            raise BadArgs("camera is agentview or wrist")
        return self._render(PUBLIC_CAMERAS[cam], "wrist" if cam != "agentview" else "agentview", args,
                            self.files_dir)

    def op_cam(self, args):
        self._need_env()
        cam = args.get("camera", "agentview")
        if cam not in PUBLIC_CAMERAS:
            raise BadArgs("camera is agentview or wrist")
        np, m, d = self.np, self.m, self.d
        cid = self.mj.mj_name2id(m, self.mj.mjtObj.mjOBJ_CAMERA, PUBLIC_CAMERAS[cam])
        w, h = self._size(args)
        fovy = float(m.cam_fovy[cid])
        f = 0.5 * h / math.tan(math.radians(fovy) / 2.0)
        r_cv = np.array(d.cam_xmat[cid]).reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        return P.plain({"camera": "wrist" if cam != "agentview" else "agentview", "width": w, "height": h,
                        "fovy_deg": fovy, "fx": f, "fy": f, "cx": w / 2.0, "cy": h / 2.0,
                        "pos": d.cam_xpos[cid], "mat_world_from_cam": r_cv, "step": self.ledger.step,
                        "moves_with_arm": bool(m.cam_bodyid[cid] != 0),
                        "depth": HS.depth_facts(m, np, os.environ.get("MUJOCO_GL")),
                        "axes": "OpenCV: x right, y down, z forward; pixel u = column, v = row of the true-orientation "
                                "image (row 0 at the top), no distortion"})

    def op_robot_xml(self, args):
        self._need_env()
        out_dir = args.get("dir") or os.path.join(self.files_dir, f"robot_e{self.ep:02d}")
        if not os.path.isabs(out_dir):
            raise BadArgs("dir must be an absolute path")
        info = L.robot_mjcf(self.inner.sim.model.get_xml(), out_dir)
        r, m, d = self.robot, self.m, self.d
        base = self.mj.mj_name2id(m, self.mj.mjtObj.mjOBJ_BODY, "robot0_base")
        jids = r._ref_joint_indexes
        return P.plain(dict(info, joints=list(r.robot_joints), fingers=list(r.gripper.joints),
                            eef_point=r.gripper.important_sites["grip_site"], wrist_camera="robot0_eye_in_hand",
                            base_pos=d.xpos[base], base_quat_wxyz=d.xquat[base], joint_ranges=m.jnt_range[jids],
                            tau_limit=r.torque_limits[1], robot_class=type(r.robot_model).__name__,
                            init_qpos=r.init_qpos))

    def op_contact_rates(self, args):
        """The stiffest contact rate each of the robot's own bodies can meet the scene with, by the
        simulator's own mixing rule (bench/ext/libero/contact_mix.py), keyed by the robot's own body names only (no
        scene item is named), with its own geoms' rate beside."""
        self._need_env()
        from bench.ext.libero import contact_mix as CM
        m, mj = self.m, self.mj

        def robot(b):
            n = mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, int(b)) or ""
            return n.startswith(ROBOT_PREFIXES)
        return P.plain(dict(CM.rates(m, robot), source="the scene's compiled model: each robot geom mixed with each "
                                                       "scene geom it can collide with, by the simulator's own rule"))

    def op_hello(self, args):
        return {"protocol": P.PROTOCOL, "pid": os.getpid(), "runtime": self.runtime, "gl": self.gl,
                "ops": sorted(PUBLIC_HANDLERS)}

    def op_close(self, args):
        if self.env is not None:
            self._end_episode("close")
        self.stop = True
        return {"closed": True}

    # -- dispatch ----------------------------------------------------------------------------------------------------
    def _guard(self, result):
        """Fail closed: a public answer that would name a scene item is not sent."""
        named = P.names_in(result, self.secrets) if self.env is not None else []
        if named:
            self.note(f"withheld a public answer (request {self.req}) naming {named}")
            raise Withheld("the answer would have named a scene item and was withheld")
        return result

    def handle(self, line, scoring):
        try:
            req = P.decode(line)
        except ValueError as exc:
            return P.err(None, "request", f"not one JSON object: {exc}")
        rid, op, args = req.get("id"), req.get("op"), req.get("args")
        args = {} if args is None else args
        table = SCORING_HANDLERS if scoring else PUBLIC_HANDLERS
        if op not in table:
            return P.err(rid, "unknown_op", f"no op {op!r}; ops: {', '.join(sorted(table))}")
        if not isinstance(args, dict):
            return P.err(rid, "bad_args", "args must be a JSON object")
        self.req = rid
        try:
            result = table[op](self, args)
            return P.ok(rid, result if scoring else self._guard(result))
        except Withheld as exc:
            return P.err(rid, "withheld", str(exc))
        except (BadArgs, NoEpisode, WrongRuntime) as exc:
            kind, text, trace = KINDS[type(exc)], str(exc), traceback.format_exc()
        except Exception as exc:
            kind, text, trace = "internal", f"{type(exc).__name__}: {exc}", traceback.format_exc()
        self.note(f"request {rid} op {op} failed ({kind}):\n{trace}")
        if not scoring and self.env is not None and P.names_in(text, self.secrets):
            text = "the host failed on this request; the reason is in the scoring folder's host log"
        return P.err(rid, kind, text)


PUBLIC_HANDLERS = {op: getattr(Host, "op_" + op) for op in P.PUBLIC_OPS}
SCORING_HANDLERS = {op: getattr(Host, "sc_" + op) for op in P.SCORING_OPS}


def main(argv=None):
    ap = argparse.ArgumentParser(prog="bench.ext.libero.envhost")
    ap.add_argument("--runtime", choices=("base", "pro"), default="base")
    ap.add_argument("--sock", required=True, help="the public socket path (keep it outside the episode folder)")
    ap.add_argument("--scoring-dir", required=True, help="truth.sock, truth_log.jsonl, host.log, ready.json")
    ap.add_argument("--files-dir", default=None, help="default folder for renders and robot_xml (default "
                                                      "SCORING_DIR/public)")
    env_gl = os.environ.get("MUJOCO_GL", "").lower()
    ap.add_argument("--gl", choices=L.GLS, default=env_gl if env_gl in ("egl", "osmesa") else "egl",
                    help="egl (waits for one of the box's EGL slots), osmesa, or auto (an EGL slot if one is free, "
                         "else osmesa); default: MUJOCO_GL of the environment when it is egl or osmesa, else egl")
    ap.add_argument("--egl-wait-s", type=float, default=120.0)
    ap.add_argument("--idle-s", type=float, default=3 * 3600.0)
    ns = ap.parse_args(argv)
    gl, slot = ns.gl, None
    if gl in ("egl", "auto"):   # kept open (the slot is held) until this process ends
        slot = L.take_egl_slot(ns.egl_wait_s if gl == "egl" else 0.0)
        if slot is None and gl == "egl":
            print(f"no EGL slot came free in {ns.egl_wait_s:g} s ({L.EGL_SLOTS} slots in {L.EGL_SLOT_DIR}); start "
                  "with --gl osmesa or auto", file=sys.stderr)
            return 3
        gl = "egl" if slot is not None else "osmesa"
    os.environ["MUJOCO_GL"] = gl
    os.environ["PYOPENGL_PLATFORM"] = gl
    if ns.runtime == "pro":   # LIBERO-PRO's own libero package, in place of the installed base one
        sys.path[:] = [L.PRO_ROOT] + [p for p in sys.path if os.path.abspath(p) != os.path.abspath(L.BASE_ROOT)]
    scoring = os.path.abspath(ns.scoring_dir)
    os.makedirs(scoring, exist_ok=True)
    files = os.path.abspath(ns.files_dir or os.path.join(scoring, "public"))
    host = Host(ns.runtime, scoring, files, gl)
    info = {"runtime": host.runtime, "gl": host.gl, "egl_slot": slot[0] if slot else None}
    why = "host error"   # replaced by serve's own reason when it returns
    try:
        why = P.serve(host.handle, os.path.abspath(ns.sock), os.path.join(scoring, L.TRUTH_SOCK),
                      os.path.join(scoring, L.READY), info, ns.idle_s, lambda: host.stop)
    finally:
        if host.env is not None:
            host._end_episode(why)
    return 0


if __name__ == "__main__":
    sys.exit(main())
