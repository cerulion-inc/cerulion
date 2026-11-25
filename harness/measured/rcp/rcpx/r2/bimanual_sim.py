# SPDX-License-Identifier: AGPL-3.0-only
"""Generic N-arm MuJoCo world with free boxes and fixed or arm-mounted cameras, for tests of the bimanual
package (rcpx/r2/bimanual.py).

TwinSim(spec, bases, boxes=..., cameras=...) implements rcpx.r2.bimanual_arms.MultiArmBackend: N copies of one bench
robot spec attached by MjSpec at the stated T_R_B (frame R = the MuJoCo world, z up, floor at z = 0), arm names as
given. Wrist cameras are "<arm>_wrist"; every renderer lives in one GL thread for the sim's life (render).
truth() and set_box() are for tests only and must never be exposed through a tool.
The servo law is the round one law of rcpx/sim_backend.py, without body errors.
"""

import queue
import threading
import types

import mujoco
import numpy as np

from bench.robots import load_robot_mjcf
from rcpx.r2.bimanual_arms import MultiArmBackend


def pose(pos_m=(0, 0, 0), yaw_deg=0.0) -> np.ndarray:
    """Rigid pose with a rotation about +z and a translation in the parent frame."""
    a = np.radians(float(yaw_deg))
    c, s = np.cos(a), np.sin(a)
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = _vector(pos_m, 3, "pos_m")
    if not np.isfinite(a):
        raise ValueError("yaw_deg must be finite")
    return T


def _vector(value, n, name):
    v = np.asarray(value, dtype=float).reshape(-1)
    if v.size != n or not np.all(np.isfinite(v)):
        raise ValueError(f"{name} needs {n} finite values")
    return v.copy()


def _quat(R):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).reshape(9))
    return q


def _unit_quat(value):
    q = _vector(value, 4, "quat")
    norm = np.linalg.norm(q)
    if norm == 0:
        raise ValueError("quat must be nonzero")
    return q / norm


def _camera_rotation(pos, lookat):
    z = pos - lookat
    norm = np.linalg.norm(z)
    if norm == 0:
        raise ValueError("camera position and lookat must differ")
    z /= norm
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-12:
        x = np.cross([0.0, 1.0, 0.0], z)
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


class TwinSim(MultiArmBackend):
    def __init__(self, spec, bases, *, boxes=(), cameras=(), start_q=None, grippers=None,
                 settle_s=None, physics_dt=None, substeps=None):
        self.arms = tuple(bases)
        if not self.arms or any(not isinstance(a, str) or not a for a in self.arms):
            raise ValueError("bases needs at least one named arm")
        # These are facts of this controller when no spec option states them.
        physics_dt = spec.get("physics_dt_s", 0.002) if physics_dt is None else physics_dt
        substeps = spec.get("substeps", 10) if substeps is None else substeps
        settle_s = 0 if settle_s is None else settle_s
        self.physics_dt = float(physics_dt)
        self.substeps = int(substeps)
        if not np.isfinite(self.physics_dt) or self.physics_dt <= 0:
            raise ValueError("physics_dt must be positive and finite")
        if self.substeps < 1 or self.substeps != substeps:
            raise ValueError("substeps must be a positive integer")
        if not np.isfinite(settle_s) or settle_s < 0:
            raise ValueError("settle_s must be finite and nonnegative")
        self.dt = self.physics_dt * self.substeps
        self.spent, self.lost, self.steps_run = False, False, 0
        self._bases, self._arm, self._boxes, self._cameras = {}, {}, {}, {}
        self._gl = None                  # the GL thread's job queue, made at the first render
        self.kp = np.asarray(spec["kp"], dtype=float).copy()
        self.kd = np.asarray(spec["kd"], dtype=float).copy()
        self.limit = np.asarray(spec["torque_limit"], dtype=float).copy()
        self._g = spec.get("gripper")
        self._g_open = np.asarray(self._g["open"], float).copy() if self._g else None
        self._g_closed = np.asarray(self._g["closed"], float).copy() if self._g else None

        w = mujoco.MjSpec()
        w.option.timestep = self.physics_dt
        w.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.05])
        w.worldbody.add_light(pos=[0, 0, 3], dir=[0, 0, -1])
        for i, arm in enumerate(self.arms):
            T = np.asarray(bases[arm], dtype=float).copy()
            if (T.shape != (4, 4) or not np.all(np.isfinite(T))
                    or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
                    or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-10, rtol=0)
                    or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-10, rtol=0)):
                raise ValueError(f"base for {arm} must be a rigid 4x4 pose")
            self._bases[arm] = T
            c = load_robot_mjcf(spec)
            if i == 0:
                w.option.integrator = c.option.integrator
            f = w.worldbody.add_frame(pos=T[:3, 3], quat=_quat(T[:3, :3]))
            w.attach(c, prefix=arm + "/", frame=f)
            cam = spec["camera"]
            self._cameras[arm + "_wrist"] = {
                "mount": arm, "width": int(cam["width"]), "height": int(cam["height"]),
                "mj_name": arm + "/wrist"}

        for box in boxes:
            name = box["name"]
            if name in self._boxes:
                raise ValueError(f"duplicate box {name}")
            size = _vector(box["size_m"], 3, "size_m")
            mass = float(box["mass_kg"])
            if np.any(size <= 0) or not np.isfinite(mass) or mass <= 0:
                raise ValueError("box size and mass must be positive and finite")
            body = w.worldbody.add_body(name=name, pos=_vector(box["pos_m"], 3, "pos_m"),
                                       quat=_unit_quat(box.get("quat", [1, 0, 0, 0])))
            joint = name + "/free"
            body.add_freejoint(name=joint)
            body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=size / 2, mass=mass,
                          friction=[1.0, 0.005, 0.0001], condim=4,
                          rgba=box.get("rgba", [0.65, 0.45, 0.25, 1.0]))
            self._boxes[name] = joint

        for cam in cameras:
            name = cam["name"]
            if name in self._cameras:
                raise ValueError(f"duplicate camera {name}")
            pos = _vector(cam["pos_m"], 3, "pos_m")
            lookat = _vector(cam["lookat_m"], 3, "lookat_m")
            width, height = int(cam["width"]), int(cam["height"])
            fovy = float(cam["fovy_deg"])
            if width < 1 or height < 1 or not 0 < fovy < 180:
                raise ValueError("camera needs positive dimensions and fovy_deg between 0 and 180")
            c = w.worldbody.add_camera(name=name, pos=pos, quat=_quat(_camera_rotation(pos, lookat)))
            c.fovy, c.resolution = fovy, [width, height]
            self._cameras[name] = {"mount": "fixed", "width": width, "height": height, "mj_name": name}
        w.visual.global_.offwidth = max(w.visual.global_.offwidth, *(c["width"] for c in self._cameras.values()))
        w.visual.global_.offheight = max(w.visual.global_.offheight, *(c["height"] for c in self._cameras.values()))
        self.m = w.compile()
        self.d = mujoco.MjData(self.m)

        start_q, grippers = start_q or {}, grippers or {}
        for arm in self.arms:
            jids = [self.m.joint(arm + "/" + j).id for j in spec["arm_joints"]]
            limited = self.m.jnt_limited[jids].astype(bool)   # a joint without limits keeps every angle
            state = types.SimpleNamespace(
                qa=self.m.jnt_qposadr[jids].copy(), da=self.m.jnt_dofadr[jids].copy(),
                motors=np.array([self.m.actuator(arm + "/motor_" + j).id for j in spec["arm_joints"]]),
                q_lo=np.where(limited, self.m.jnt_range[jids, 0], -np.inf),
                q_hi=np.where(limited, self.m.jnt_range[jids, 1], np.inf),
                q_target=None, torque_frac=1.0, tau=np.zeros(len(jids)),
                g_target=self._g_closed.copy() if self._g else None, extra=types.SimpleNamespace())
            if self._g:
                gids = [self.m.joint(arm + "/" + j).id for j in self._g["joints"]]
                state.gqa, state.gda = self.m.jnt_qposadr[gids].copy(), self.m.jnt_dofadr[gids].copy()
                state.gact = np.array([self.m.actuator(arm + "/" + a).id for a in self._g["actuators"]])
            self._arm[arm] = state
            self.command(arm, start_q.get(arm, spec["q_home"]), gripper=grippers.get(arm, "close"))
            self.d.qpos[state.qa] = state.q_target
            if self._g:
                self.d.qpos[state.gqa] = state.g_target
        self._boxes = {name: (int(self.m.jnt_qposadr[self.m.joint(j).id]),
                              int(self.m.jnt_dofadr[self.m.joint(j).id])) for name, j in self._boxes.items()}
        mujoco.mj_forward(self.m, self.d)
        for _ in range(int(round(settle_s / self.dt))):
            self._period()
        self.t0 = float(self.d.time)

    def _state(self, arm):
        if arm not in self._arm:
            raise ValueError(f"unknown arm {arm}")
        return self._arm[arm]

    def command(self, arm, q_target, torque_frac=1.0, gripper=None):
        s = self._state(arm)
        q = _vector(q_target, len(s.qa), "q_target")
        fraction = float(torque_frac)
        if not np.isfinite(fraction):
            raise ValueError("torque_frac must be finite")
        g_target = s.g_target
        if gripper is not None and self._g:
            f = 1.0 if gripper == "open" else 0.0 if gripper == "close" else float(gripper)
            if not np.isfinite(f):
                raise ValueError("gripper fraction must be finite")
            g_target = self._g_closed + np.clip(f, 0, 1) * (self._g_open - self._g_closed)
        s.q_target = np.clip(q, s.q_lo, s.q_hi)
        s.torque_frac, s.g_target = float(np.clip(fraction, 0, 1)), g_target

    def _period(self):
        for _ in range(self.substeps):
            for s in self._arm.values():
                cap = self.limit * s.torque_frac
                s.tau = np.clip(self.kp * (s.q_target - self.d.qpos[s.qa])
                                - self.kd * self.d.qvel[s.da], -cap, cap)
                self.d.ctrl[s.motors] = s.tau
                if self._g:
                    self.d.ctrl[s.gact] = s.g_target
            mujoco.mj_step(self.m, self.d)
        # Derived poses and contacts must describe the final integrated joint positions.
        mujoco.mj_forward(self.m, self.d)

    def step(self, n=1):
        for _ in range(int(n)):
            self._period()
            self.steps_run += 1

    def read(self, arm):
        s, d = self._state(arm), self.d
        out = {"t": float(d.time) - self.t0, "q": d.qpos[s.qa].copy(), "qd": d.qvel[s.da].copy(),
               "tau": s.tau.copy(), "load": s.tau / self.limit, "torque_frac": s.torque_frac,
               "q_target": s.q_target.copy(), "gripper": None, "steps_left": None}
        if self._g:
            q = d.qpos[s.gqa].copy()
            span = self._g_open - self._g_closed
            frac = float(np.mean((q - self._g_closed) / np.where(np.abs(span) > 1e-9, span, 1.0)))
            out["gripper"] = {"q": q, "qd": d.qvel[s.gda].copy(), "target": s.g_target.copy(),
                              "open_frac": frac}
        return out

    def _camera(self, name):
        if name not in self._cameras:
            raise ValueError(f"unknown camera {name}")
        return self._cameras[name]

    def cameras(self):
        return {n: {k: c[k] for k in ("mount", "width", "height")} for n, c in self._cameras.items()}

    def render(self, camera):
        """The camera's frame. Every renderer lives in one GL thread for the sim's life (_gl_loop): on macOS a
        MuJoCo renderer made in one thread hangs when another thread uses or frees it, and the arms of a time
        aligned move call from their own threads."""
        c = self._camera(camera)
        if self._gl is None:
            self._gl = queue.Queue()
            threading.Thread(target=_gl_loop, args=(self.m, self.d, self._gl), name="twinsim-gl", daemon=True).start()
        box, done = {}, threading.Event()
        self._gl.put(((c["mj_name"], c["width"], c["height"]), box, done))
        done.wait()
        if "err" in box:
            raise box["err"]
        return box["img"]

    def camera_info(self, camera):
        c = self._camera(camera)
        w, h = c["width"], c["height"]
        fovy = float(self.m.cam_fovy[self.m.camera(c["mj_name"]).id])
        f = float((h / 2) / np.tan(np.radians(fovy) / 2))
        return {"name": camera, "width": w, "height": h, "fovy_deg": fovy, "fx": f, "fy": f,
                "cx": w / 2, "cy": h / 2, "model": "pinhole, no distortion; u right, v down, origin top left"}

    def T_R_B(self, arm):
        self._state(arm)
        return self._bases[arm].copy()

    def T_R_cam(self, name):
        c = self._camera(name)
        if c["mount"] != "fixed":
            raise ValueError("T_R_cam requires a fixed camera")
        cid = self.m.camera(c["mj_name"]).id
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.d.cam_xmat[cid].reshape(3, 3), self.d.cam_xpos[cid]
        return T

    def arm_extra(self, arm):
        return self._state(arm).extra

    def _geom_name(self, gid):
        return self.m.geom(gid).name or f"{self.m.body(int(self.m.geom_bodyid[gid])).name}#{gid}"

    def truth(self):
        """Test only: free body poses and contacts, never an operator sensor."""
        boxes = {name: {"pos_m": self.d.qpos[qa:qa + 3].tolist(), "quat": self.d.qpos[qa + 3:qa + 7].tolist()}
                 for name, (qa, _) in self._boxes.items()}
        contacts = {tuple(sorted((self._geom_name(int(c.geom1)), self._geom_name(int(c.geom2)))))
                    for c in self.d.contact[:self.d.ncon]}
        return {"t": float(self.d.time) - self.t0, "boxes": boxes, "contacts": [list(p) for p in sorted(contacts)]}

    def set_box(self, name, pos_m, quat=(1, 0, 0, 0)):
        """Test only: reset a free body at rest and refresh the world."""
        if name not in self._boxes:
            raise ValueError(f"unknown box {name}")
        pos, q = _vector(pos_m, 3, "pos_m"), _unit_quat(quat)
        qa, da = self._boxes[name]
        self.d.qpos[qa:qa + 7] = np.concatenate((pos, q))
        self.d.qvel[da:da + 6] = 0
        self.d.qacc_warmstart[da:da + 6] = 0
        mujoco.mj_forward(self.m, self.d)

    def geom_names(self, arm, body):
        self._state(arm)
        root = self.m.body(arm + "/" + body).id
        descendants = {root}
        for b in range(root + 1, self.m.nbody):
            if int(self.m.body_parentid[b]) in descendants:
                descendants.add(b)
        return sorted(self._geom_name(g) for g in range(self.m.ngeom) if int(self.m.geom_bodyid[g]) in descendants)

    def close(self):
        gl, self._gl = self._gl, None
        if gl is not None:
            gl.put(None)


def _gl_loop(m, d, jobs):
    """The GL thread: makes, uses and frees every renderer of one sim (TwinSim.render). One retry with a fresh
    renderer after a failure, as rcpx/sim_backend.py does."""
    renderers = {}
    while True:
        job = jobs.get()
        if job is None:
            for r in renderers.values():
                try:
                    r.close()
                except Exception:
                    pass
            return
        (name, w, h), box, done = job
        for attempt in (0, 1):
            try:
                if (name, w, h) not in renderers:
                    renderers[(name, w, h)] = mujoco.Renderer(m, height=h, width=w)
                r = renderers[(name, w, h)]
                r.update_scene(d, camera=name)
                box["img"] = np.array(r.render(), dtype=np.uint8, copy=True)
                break
            except Exception as exc:
                bad = renderers.pop((name, w, h), None)
                try:
                    if bad is not None:
                        bad.close()
                except Exception:
                    pass
                if attempt:
                    box["err"] = exc
        done.set()
