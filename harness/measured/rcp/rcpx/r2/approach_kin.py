# SPDX-License-Identifier: AGPL-3.0-only
"""Named tool surface kinematics for the approach package.

A tool surface is a site the robot loader adds to the tip body for every entry of the robot spec's tool_surfaces
("surface_<name>", site z = the surface's outward normal); "tip" without such an entry is the tip site with tool x
as its facing. SurfaceKin gives that surface's pose and Jacobian on the robot's own kinematic model
(RobotModel.mj, the robot alone), a prioritised solve (position first, then the surface facing a direction, then
the target kept in the wrist view through RobotModel.project, each in the null space of the ones before), the push
force the surface can make, and what the wrist camera would see of a point (in frame, in front, not hidden by the
robot's own geometry). Nothing here reads a scene or names a robot. SI inside.
"""

import math

import mujoco
import numpy as np

from bench.robots import home_qpos

LIMIT_SLOW_RAD = 0.1      # a joint closer than this to a limit is slowed in the solve (as RobotModel does)
MAX_STEP_RAD = 0.2        # largest joint change per solve iteration
DRIFT_M = 0.003           # a lower task's step may drift the met position this far (the next step takes it back)
POS_TOL_M = 2e-4
ANG_TOL_RAD = math.radians(0.3)
STILL_RAD = 1e-4          # a solve whose step is under this with the position met has converged
VIEW_GAIN = 0.5
VIEW_EDGE_FRAC = 0.04     # a point closer than this share of the frame to an edge counts as out of view
SIGHT_SHORT_M = 0.004     # the sight line stops this short of the point (the surface itself is not a blocker)
GIMBAL_COS = 1e-4         # rpy_deg: cos(pitch) under this is pitch +-90, where roll and yaw are not apart


def unit(v):
    v = np.asarray(v, dtype=float).reshape(-1)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else None


def angle_deg(a, b):
    c = float(np.clip(np.dot(unit(a), unit(b)), -1.0, 1.0))
    return math.degrees(math.acos(c))


def rpy_deg(R):
    """[roll, pitch, yaw] in degrees with R = Rz(yaw) Ry(pitch) Rx(roll) (frame axes in base coordinates). With
    the x axis within GIMBAL_COS of straight up or down (pitch +-90) roll and yaw turn about the same axis and the
    four entries that would set them apart are rounding noise, so roll is 0, pitch exactly +-90 and yaw carries the
    turn (rebuilt within asin(GIMBAL_COS), under 0.006 deg)."""
    R = np.asarray(R, dtype=float)
    if math.hypot(float(R[0, 0]), float(R[1, 0])) < GIMBAL_COS:
        pitch = math.copysign(math.pi / 2, -float(R[2, 0]))
        return [0.0, round(math.degrees(pitch), 3), round(math.degrees(math.atan2(-float(R[0, 1]), float(R[1, 1]))), 3)]
    pitch = math.asin(float(np.clip(-R[2, 0], -1.0, 1.0)))
    roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
    yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    return [round(math.degrees(x), 3) for x in (roll, pitch, yaw)]


def rot_from_rpy_deg(rpy):
    r, p, y = (math.radians(float(x)) for x in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    Ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    Rx = np.array([[1.0, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _pinv(J, lam):
    """Damped least squares pseudo-inverse J^T (J J^T + lam^2 I)^-1."""
    return J.T @ np.linalg.solve(J @ J.T + (lam * lam) * np.eye(J.shape[0]), np.eye(J.shape[0]))


def surface_names(spec):
    names = [s["name"] for s in spec.get("tool_surfaces") or []]
    if "tip" not in names:
        names.insert(0, "tip")
    return names


class SurfaceKin:
    """One named tool surface on the robot's kinematic model. Raises KeyError for an unknown name."""

    def __init__(self, model, name):
        spec, m = model.spec, model.mj
        self.model, self.name, self.m = model, str(name), m
        entry = next((s for s in spec.get("tool_surfaces") or [] if s["name"] == name), None)
        if entry is not None:
            self.site, self.axis = m.site("surface_" + entry["name"]).id, 2
            self.size_mm = [float(x) for x in entry.get("size_mm", [])]
        elif name == "tip":
            self.site, self.axis, self.size_mm = m.site("tip").id, 0, []
        else:
            raise KeyError(name)
        self.d = mujoco.MjData(m)
        self.qfull = home_qpos(m, spec)
        jids = [m.joint(j).id for j in model.joint_names]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jids])
        self.dadr = np.array([m.jnt_dofadr[j] for j in jids])
        self.jp = np.zeros((3, m.nv))
        self.jr = np.zeros((3, m.nv))
        self.n = len(jids)

    # ---------------------------------------------------------------- pose
    def _set(self, q):
        d = self.d
        d.qpos[:] = self.qfull
        d.qpos[self.qadr] = np.asarray(q, dtype=float)
        mujoco.mj_kinematics(self.m, d)
        mujoco.mj_comPos(self.m, d)

    def pose(self, q):
        """(point, outward normal, site rotation) in frame B."""
        self._set(q)
        R = self.d.site_xmat[self.site].reshape(3, 3).copy()
        return self.d.site_xpos[self.site].copy(), R[:, self.axis].copy(), R

    def jac(self, q):
        self._set(q)
        mujoco.mj_jacSite(self.m, self.d, self.jp, self.jr, self.site)
        return self.jp[:, self.dadr].copy(), self.jr[:, self.dadr].copy()

    # ---------------------------------------------------------------- solve
    def _view_task(self, q, point):
        """(pixel error to the image centre in focal units, its joint Jacobian) or None when behind the camera."""
        model = self.model
        u, v, depth = model.project(q, point)
        if not (depth > 1e-3 and math.isfinite(u)):
            return None
        e = np.array([model.cam_w / 2.0 - u, model.cam_h / 2.0 - v]) / model.f_px
        Jv = np.zeros((2, self.n))
        h = 1e-5
        for i in range(self.n):
            dq = np.array(q, dtype=float)
            dq[i] += h
            u2, v2, _ = model.project(dq, point)
            Jv[:, i] = [(u2 - u) / h / model.f_px, (v2 - v) / h / model.f_px]
        return e, Jv

    def _step(self, q, p_goal, ng, view_point, w, up=None):
        """One prioritised step with joint weights w (0 = the joint is held). up = (axis index, direction): the
        roll about the facing that turns that site axis toward the direction, below the facing and above the view.
        Returns (dq, position error m, facing error rad, roll error rad)."""
        p, nrm, R = self.pose(q)
        Jp, Jr = self.jac(q)
        e_p = p_goal - p
        L = float(np.linalg.norm(e_p))
        if L > 0.05:
            e_p = e_p * (0.05 / L)
        Jpw = Jp * w
        A1 = _pinv(Jpw, 1e-3)
        x = A1 @ e_p
        N = np.eye(self.n) - A1 @ Jpw
        ang = 0.0
        if ng is not None:
            c = float(np.clip(nrm @ ng, -1.0, 1.0))
            ang = math.acos(c)
            ax = np.cross(nrm, ng)
            if np.linalg.norm(ax) < 1e-9:
                ax = np.zeros(3) if c > 0 else np.cross(nrm, [1.0, 0, 0] if abs(nrm[0]) < 0.9 else [0, 1.0, 0])
            ax = ax / max(float(np.linalg.norm(ax)), 1e-12)
            e_a = ax * min(ang, 0.2)
            P = np.eye(3) - np.outer(nrm, nrm)  # turning about the normal itself is not asked for
            Ja = (P @ Jr) * w
            JaN = Ja @ N
            A2 = _pinv(JaN, 1e-2)
            x = x + A2 @ (e_a - Ja @ x)
            N = N - A2 @ JaN
        roll = 0.0
        if up is not None and ng is not None:
            j, t = up
            a_j = R[:, j]
            ap, tp = a_j - (a_j @ nrm) * nrm, t - (t @ nrm) * nrm
            if np.linalg.norm(ap) > 1e-6 and np.linalg.norm(tp) > 1e-6:
                roll = math.atan2(float(nrm @ np.cross(ap, tp)), float(ap @ tp))
                Jl = (nrm @ Jr)[None, :] * w
                JlN = Jl @ N
                A4 = _pinv(JlN, 1e-2)
                x = x + A4 @ (np.array([max(-0.2, min(0.2, roll))]) - Jl @ x)
                N = N - A4 @ JlN
        if view_point is not None:
            vt = self._view_task(q, view_point)
            if vt is not None:
                e_v, Jv = vt
                Jvw = Jv * w
                JvN = Jvw @ N
                A3 = _pinv(JvN, 0.05)
                x = x + A3 @ (VIEW_GAIN * e_v - Jvw @ x)
        return w * x, float(np.linalg.norm(p_goal - p)), ang, abs(roll)

    def _run(self, q, p_goal, ng, view_point, iters, up=None):
        """Iterate _step. Once the position is met, a step that would drift it more than DRIFT_M (which the next
        step's position task takes back) is halved up to four times, and the run stops when it still would: a lower
        task never walks the arm away from the position."""
        model = self.model
        lo, hi = model.q_lo, model.q_hi
        for _ in range(int(iters)):
            w = np.clip(np.minimum(q - lo, hi - q) / LIMIT_SLOW_RAD, 0.05, 1.0)
            dq, L, ang, roll = self._step(q, p_goal, ng, view_point, w, up)
            # a joint at a limit that the step pushes further is held, and the step is solved again without it
            held = ((q <= lo + 1e-9) & (dq < 0)) | ((q >= hi - 1e-9) & (dq > 0))
            if held.any():
                dq, L, ang, roll = self._step(q, p_goal, ng, view_point, np.where(held, 0.0, w), up)
            big = float(np.max(np.abs(dq))) if dq.size else 0.0
            if big > MAX_STEP_RAD:
                dq = dq * (MAX_STEP_RAD / big)
                big = MAX_STEP_RAD
            q_new = model.clip(q + dq)
            if L < 1e-3:
                for _ in range(4):
                    if float(np.linalg.norm(p_goal - self.pose(q_new)[0])) <= DRIFT_M:
                        break
                    dq = 0.5 * dq
                    q_new = model.clip(q + dq)
                else:
                    break                  # the lower tasks cannot move without losing the position: settled
            q = q_new
            if L < POS_TOL_M and big < STILL_RAD:
                break                      # every task has settled (a facing that cannot be met included)
            if view_point is None and L < POS_TOL_M and (ng is None or ang < ANG_TOL_RAD) and (
                    up is None or roll < ANG_TOL_RAD):
                break
        return q

    def solve(self, q0, p_goal, n_goal=None, view_point=None, iters=150, up=None):
        """Joint pose putting the surface point at p_goal (first), its outward normal along n_goal (second), the roll
        about the normal toward up = (site axis index, direction) when given (third; else the roll is free) and
        view_point toward the wrist image centre (last). Every joint is in the solve and limits hold. The position
        is solved alone first, then the facing (and roll) is added, then the view, then a polish without the view,
        so no lower task costs the position. Returns (q, position error m, normal error rad or None)."""
        q = self.model.clip(np.asarray(q0, dtype=float))
        p_goal = np.asarray(p_goal, dtype=float)
        ng = None if n_goal is None else unit(n_goal)
        up = None if up is None or ng is None else (int(up[0]), unit(up[1]))
        q = self._run(q, p_goal, None, None, iters)
        if ng is not None:
            q = self._run(q, p_goal, ng, None, iters, up)
        if view_point is not None:
            q = self._run(q, p_goal, ng, view_point, iters, up)
            q = self._run(q, p_goal, ng, None, 60, up)
        q = self._run(q, p_goal, None, None, 60)   # the position polish: what a held facing step left over
        p, nrm, _ = self.pose(q)
        pos_err = float(np.linalg.norm(p_goal - p))
        ang_err = None if ng is None else math.acos(float(np.clip(nrm @ ng, -1.0, 1.0)))
        return q, pos_err, ang_err

    # ---------------------------------------------------------------- statics and view
    def force_capability(self, q, direction):
        """Largest steady force (N) the surface point can push on the world along direction (frame B) at q, within
        the joint torque limits after holding gravity (RobotModel.tip_force_capability at this point)."""
        model = self.model
        dvec = unit(direction)
        g = model.gravity_torque(q)
        Jp, _ = self.jac(q)
        a = Jp.T @ dvec
        F = float("inf")
        for ai, gi, li in zip(a, g, model.torque_limit):
            if abs(gi) > li:
                return 0.0
            if ai > 1e-9:
                F = min(F, (li - gi) / ai)
            elif ai < -1e-9:
                F = min(F, (li + gi) / -ai)
        return float(F) if math.isfinite(F) else 1e6

    def view_of(self, q, point):
        """What the wrist camera would see of point at q: pixel, depth, in frame, and the robot geom (by body) the
        sight line meets first, from the robot's own geometry."""
        model, m = self.model, self.m
        u, v, depth = model.project(q, point)
        out = {"target_px": None, "depth_mm": round(depth * 1000.0, 1), "in_frame": False, "in_view": False,
               "hidden_by": None}
        if not (depth > 1e-3 and math.isfinite(u)):
            out["why"] = "the point is behind the camera"
            return out
        out["target_px"] = [round(u, 1), round(v, 1)]
        mu, mv = VIEW_EDGE_FRAC * model.cam_w, VIEW_EDGE_FRAC * model.cam_h
        out["in_frame"] = bool(mu <= u <= model.cam_w - mu and mv <= v <= model.cam_h - mv)
        self._set(q)
        mujoco.mj_camlight(m, self.d)
        cid = m.camera("wrist").id
        pc = self.d.cam_xpos[cid].copy()
        ray = np.asarray(point, dtype=float) - pc
        dist = float(np.linalg.norm(ray))
        gid = np.zeros(1, dtype=np.int32)
        hit = mujoco.mj_ray(m, self.d, pc, ray / dist, None, 1, -1, gid)
        if 0.0 <= hit < dist - SIGHT_SHORT_M and gid[0] >= 0:
            out["hidden_by"] = m.body(int(m.geom_bodyid[gid[0]])).name
        out["in_view"] = bool(out["in_frame"] and out["hidden_by"] is None)
        if not out["in_frame"]:
            out["why"] = "the point falls outside the wrist frame"
        elif out["hidden_by"] is not None:
            out["why"] = f"the robot's own {out['hidden_by']} is in the line of sight"
        return out


def _axis_angle(axis, angle):
    a = unit(axis)
    K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def nearest_facing(R_now, axis, f):
    """The rotation nearest R_now (largest trace of R_now' R) among those whose column `axis` points along f: the
    present hand turned the short way onto the facing, then rolled about it to the nearest roll (closed form)."""
    R_now = np.asarray(R_now, dtype=float)
    f = unit(f)
    a = R_now[:, axis]
    v = np.cross(a, f)
    s, c = float(np.linalg.norm(v)), float(np.clip(a @ f, -1.0, 1.0))
    if s < 1e-9:
        side = R_now[:, (axis + 1) % 3]
        R0 = R_now if c > 0 else _axis_angle(side, math.pi) @ R_now
    else:
        R0 = _axis_angle(v / s, math.atan2(s, c)) @ R_now
    M = R0 @ R_now.T
    K = np.array([[0.0, -f[2], f[1]], [f[2], 0.0, -f[0]], [-f[1], f[0], 0.0]])
    A = float(np.trace(M) - f @ M @ f)
    B = float(np.trace(K @ M))
    return _axis_angle(f, math.atan2(B, A)) @ R0


def turn_deg(Ra, Rb):
    """The angle (deg) of the rotation from Ra to Rb."""
    c = (float(np.trace(np.asarray(Ra).T @ np.asarray(Rb))) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))
