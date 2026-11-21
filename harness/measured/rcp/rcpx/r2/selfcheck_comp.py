# SPDX-License-Identifier: AGPL-3.0-only
"""Compensation for the selfcheck package: two forwarding proxies and the bookkeeping to switch them.

JointComp wraps ex.backend. It adds each joint's measured zero offset to what the backend reads and takes it off
what the executor commands, so every primitive works in the body model's own joint angles (true angle = reading +
offset). Every attribute it does not change forwards to the wrapped backend.

CameraComp wraps ex.model. It moves and turns the body model's camera by the measured mount error in the camera's
own frame, so looks, pixel rays, the camera frame of move_tip and triangulation use the camera where it really is.
Every other attribute (kinematics of the arm, limits, capsules, the model data) forwards untouched.

Both start as the identity and are installed once, in the package's setup, so a package that keeps a reference to
ex.backend or ex.model from its own setup sees the same compensation. No ground truth is read here.
"""

import math

import numpy as np

DEG = math.pi / 180.0


def rot(v):
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def rotvec(R):
    """Rotation vector (axis x angle, rad) of a rotation matrix."""
    R = np.asarray(R, dtype=float)
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    a = math.acos(c)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if a < 1e-9:
        return 0.5 * w
    return a * w / (2.0 * math.sin(a))


class JointComp:
    """Backend proxy: readings plus offset in, commands minus offset out (rad, joint order)."""

    def __init__(self, inner, n):
        object.__setattr__(self, "_inner", inner)
        object.__setattr__(self, "offset", np.zeros(int(n)))

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, "_inner"), name)

    def __setattr__(self, name, value):
        if name in ("offset",):
            object.__setattr__(self, name, value)
        else:
            setattr(self._inner, name, value)

    @property
    def selfcheck_inner(self):
        return self._inner

    def read(self):
        r = self._inner.read()
        if not np.any(self.offset):
            return r
        r = dict(r)
        r["q"] = np.asarray(r["q"], dtype=float) + self.offset
        if r.get("q_target") is not None:
            r["q_target"] = np.asarray(r["q_target"], dtype=float) + self.offset
        return r

    def command(self, q_target, torque_frac=1.0, gripper=None):
        q = np.asarray(q_target, dtype=float)
        if np.any(self.offset):
            q = q - self.offset
        return self._inner.command(q, torque_frac=torque_frac, gripper=gripper)

    def step(self, n=1):
        return self._inner.step(n)

    def render(self, camera="wrist"):
        return self._inner.render(camera)

    def camera_info(self, camera="wrist"):
        return self._inner.camera_info(camera)

    def sim_time(self):
        return float(self._inner.read()["t"])


class CameraComp:
    """Model proxy: the body model with its camera moved by `shift` (m) and turned by `turn` (rotation matrix), both
    in the camera's own frame (x image right, y image up, z toward the viewer)."""

    _OWN = ("_inner", "turn", "shift", "active")

    def __init__(self, inner):
        object.__setattr__(self, "_inner", inner)
        object.__setattr__(self, "turn", np.eye(3))
        object.__setattr__(self, "shift", np.zeros(3))
        object.__setattr__(self, "active", False)

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, "_inner"), name)

    def __setattr__(self, name, value):
        if name in CameraComp._OWN:
            object.__setattr__(self, name, value)
        else:
            setattr(self._inner, name, value)

    @property
    def selfcheck_inner(self):
        return self._inner

    def set_mount(self, turn, shift):
        self.turn = np.asarray(turn, dtype=float).copy()
        self.shift = np.asarray(shift, dtype=float).copy()
        self.active = bool(np.any(np.abs(self.turn - np.eye(3)) > 1e-12) or np.any(np.abs(self.shift) > 1e-12))

    def add_mount(self, r, s):
        """Compose a further error (rotation vector r, shift s) measured against the present, corrected camera."""
        self.set_mount(self.turn @ rot(r), self.shift + self.turn @ np.asarray(s, dtype=float))

    def mount_deg_mm(self):
        return [float(x) / DEG for x in rotvec(self.turn)], [float(x) * 1000.0 for x in self.shift]

    def _camera(self, q):
        m = self._inner
        m._set(q)
        pc = m.d.cam_xpos[m._cam].copy()
        Rc = m.d.cam_xmat[m._cam].reshape(3, 3).copy()
        if self.active:
            pc, Rc = pc + Rc @ self.shift, Rc @ self.turn
        return pc, Rc

    def fk(self, q):
        out = self._inner.fk(q)
        if self.active:
            pc, Rc = out["camera"]
            out = dict(out)
            out["camera"] = (pc + Rc @ self.shift, Rc @ self.turn)
        return out

    def project(self, q, p_world):
        if not self.active:
            return self._inner.project(q, p_world)
        pc, Rc = self._camera(q)
        m = self._inner
        x, y, z = Rc.T @ (np.asarray(p_world, dtype=float) - pc)
        depth = -z
        if depth <= 1e-9:
            return float("nan"), float("nan"), float(depth)
        return (float(m.cam_w / 2.0 + m.f_px * x / depth), float(m.cam_h / 2.0 - m.f_px * y / depth), float(depth))

    def pixel_ray(self, q, u, v):
        if not self.active:
            return self._inner.pixel_ray(q, u, v)
        pc, Rc = self._camera(q)
        m = self._inner
        dc = np.array([(u - m.cam_w / 2.0) / m.f_px, -(v - m.cam_h / 2.0) / m.f_px, -1.0])
        dw = Rc @ dc
        return pc, dw / np.linalg.norm(dw)


def install(ex):
    """Wrap ex.backend and ex.model once. Returns (JointComp, CameraComp)."""
    jc = ex.backend if isinstance(ex.backend, JointComp) else JointComp(ex.backend, ex.model.n)
    cc = ex.model if isinstance(ex.model, CameraComp) else CameraComp(ex.model)
    ex.backend, ex.model = jc, cc
    return jc, cc


def resync(ex, delta=None):
    """After a compensation changed: shift everything the executor remembers in joint angles by `delta` (rad, the
    change of the offsets) so the arm holds still and old looks keep their true poses; recompute every mark's ray
    (the joint angles or the camera changed); restart the force observer."""
    if delta is not None and np.any(delta):
        if ex._q_des is not None:
            ex._q_des = ex._q_des + delta
        for path in ex._history:
            for k in range(len(path)):
                path[k] = np.asarray(path[k], dtype=float) + delta
        seen = set()
        for rec in list(ex._looks.values()) + [ex._std_ref, ex._std_last]:
            if rec is None or id(rec) in seen:
                continue
            seen.add(id(rec))
            rec["q"] = np.asarray(rec["q"], dtype=float) + delta
    for marks in ex._marks.values():
        for key, mk in marks.items():
            rec = ex._looks.get(key)
            if rec is None:
                continue
            mk["origin"], mk["dir"] = ex.model.pixel_ray(rec["q"], mk["u"], mk["v"])
    ex._last = None
    if ex._q_des is not None:
        ex._command(ex._q_des)
    return ex._sense()
