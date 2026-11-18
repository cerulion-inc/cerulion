# SPDX-License-Identifier: AGPL-3.0-only
"""RobotModel: kinematics, statics and the robot://body document, all derived from the loaded model.

Built on a kinematics-only MjModel of the robot alone (bench.robots.load_robot_mjcf). SI inside; describe() speaks
mm, degrees, newtons. No robot is named here; everything comes from the robot_spec and the compiled model.

Self collision is measured on the model file's own collision geometry, with the pair rules of the physics (see
_self_setup): the capsules stay as the coarse shape for the floor check and the picture. The body document is in
rcpx/model_body.py, the tool's shape in rcpx/model_tool.py.
"""

import math

import mujoco
import numpy as np

from rcpx import model_solve, model_tool
from rcpx.model_solve import OfModel

# metres per radian an orientation error counts against a position error in a solve: the tool's own lever arm
ROT_WEIGHT_M = OfModel(lambda model: model.rot_weight_m)
SELF_HOME_SLACK_M = 0.001  # a link pair nearer than the asked margin at home may come this much nearer than it is there
SELF_TOUCH_M = 1e-5  # a pair this near (or overlapping) at home counts as touching there


def _rotvec(R):
    """Rotation vector (axis * angle) of a 3x3 rotation."""
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).reshape(9))
    v = np.zeros(3)
    mujoco.mju_quat2Vel(v, q, 1.0)
    return v


class RobotModel:
    def __init__(self, spec):
        from bench.robots import home_qpos, load_robot_mjcf  # here, not at the top: bench.robots builds models too

        self.spec = spec
        self.name = spec["name"]
        self.mj = load_robot_mjcf(spec).compile()
        self.d = mujoco.MjData(self.mj)
        m = self.mj
        self.joint_names = list(spec["arm_joints"])
        self.n = len(self.joint_names)
        jids = [m.joint(j).id for j in self.joint_names]
        self._qadr = np.array([m.jnt_qposadr[j] for j in jids])
        self._dadr = np.array([m.jnt_dofadr[j] for j in jids])
        self._jbody = [int(m.jnt_bodyid[j]) for j in jids]
        self._jids = jids
        self.q_lo = np.array([m.jnt_range[j][0] for j in jids])
        self.q_hi = np.array([m.jnt_range[j][1] for j in jids])
        self.q_home = np.asarray(spec["q_home"], dtype=float)
        self.torque_limit = np.asarray(spec["torque_limit"], dtype=float)
        rated = spec.get("speed_limit") or [None] * self.n   # the maker's rated joint speeds; None where none is named
        self.speed_limit = [None if v is None else float(v) for v in rated]
        self._qfull = home_qpos(m, spec)
        self._tip = m.site("tip").id
        self._cam = m.camera("wrist").id
        self._tip_body = m.site_bodyid[self._tip]
        cam = spec["camera"]
        self.cam_w, self.cam_h = int(cam["width"]), int(cam["height"])
        self.fovy = math.radians(float(cam["fovy"]))
        self.f_px = (self.cam_h / 2.0) / math.tan(self.fovy / 2.0)
        self._root = self._root_body()
        anchors = [a for a, _ in self.joint_frames(self.q_home)] + [self.fk_tip(self.q_home)[0]]
        self.length_scale = float(sum(np.linalg.norm(anchors[i + 1] - anchors[i]) for i in range(self.n)))
        self._capsule_defs = self._fit_capsules()
        self._self_setup()
        self._reach = None
        self.rot_weight_m = model_tool.tool_lever_m(self)
        self.motion = model_solve.motion_of(self)
        self._limit_slow = self.limit_stop_rad(self.motion["qd_known"])

    # ---------- basics ----------
    def _root_body(self):
        b = self._jbody[0]
        while self.mj.body_parentid[b] != 0:
            b = self.mj.body_parentid[b]
        return int(b)

    def set_jaws(self, values):
        """Pose the gripper joints at `values` (the newest reading, the spec's gripper joint order) for the collision
        capsules only: the bodies the jaws move follow them, so the path check and the speed law see an open jaw
        where it is (the capsules were fitted closed). Kinematics, gravity and the camera keep the jaws as loaded.
        Returns True when anything changed."""
        g = self.spec.get("gripper")
        if not g or values is None:
            return False
        adr = getattr(self, "_jaw_adr", None)
        if adr is None:
            adr = self._jaw_adr = np.array([self.mj.jnt_qposadr[self.mj.joint(j).id] for j in g["joints"]], dtype=int)
        v = np.asarray(values, dtype=float).reshape(-1)
        if v.size != adr.size or not np.all(np.isfinite(v)):
            return False
        base = getattr(self, "_qgeom", None)
        base = self._qfull if base is None else base
        if np.allclose(base[adr], v, atol=1e-5):
            return False
        q = self._qfull.copy()
        q[adr] = v
        self._qgeom = None if np.allclose(q, self._qfull) else q
        return True

    def jaw_bodies(self):
        """Names of the bodies the gripper joints move (each gripper joint's body and everything under it)."""
        if getattr(self, "_jaw_names", None) is None:
            m, g = self.mj, self.spec.get("gripper")
            roots = {int(m.jnt_bodyid[m.joint(j).id]) for j in (g or {}).get("joints", [])}
            names = set()
            for b in range(m.nbody):
                c = b
                while c > 0:
                    if c in roots:
                        names.add(m.body(b).name)
                        break
                    c = int(m.body_parentid[c])
            self._jaw_names = names
        return self._jaw_names

    def _set(self, q):
        q = np.asarray(q, dtype=float)
        self.d.qpos[:] = self._qfull
        self.d.qpos[self._qadr] = q
        self.d.qvel[:] = 0
        mujoco.mj_kinematics(self.mj, self.d)
        mujoco.mj_comPos(self.mj, self.d)
        mujoco.mj_camlight(self.mj, self.d)
        model_solve.IN_USE[0] = self

    def clip(self, q):
        return np.minimum(np.maximum(np.asarray(q, dtype=float), self.q_lo), self.q_hi)

    def within_limits(self, q, tol=1e-9):
        q = np.asarray(q, dtype=float)
        return bool(np.all(q >= self.q_lo - tol) and np.all(q <= self.q_hi + tol))

    def _tip_pose(self):
        return self.d.site_xpos[self._tip].copy(), self.d.site_xmat[self._tip].reshape(3, 3).copy()

    def fk(self, q):
        self._set(q)
        tip = self._tip_pose()
        cam = (self.d.cam_xpos[self._cam].copy(), self.d.cam_xmat[self._cam].reshape(3, 3).copy())
        links = {name: (p0, p1, r) for name, p0, p1, r in self._capsules_now()}
        return {"tip": tip, "camera": cam, "links": links}

    def joint_frames(self, q):
        """Per arm joint: (anchor point, unit axis) in frame B."""
        self._set(q)
        return [(self.d.xanchor[j].copy(), self.d.xaxis[j].copy()) for j in self._jids]

    def jac(self, q):
        self._set(q)
        jp = np.zeros((3, self.mj.nv))
        jr = np.zeros((3, self.mj.nv))
        mujoco.mj_jacSite(self.mj, self.d, jp, jr, self._tip)
        return np.vstack([jp[:, self._dadr], jr[:, self._dadr]])

    # ---------- solves ----------
    def _task_error(self, q, p_target, R_target, keep_axis):
        p, R = self.fk_tip(q)
        e = [np.asarray(p_target, dtype=float) - p]
        J = self.jac(q)
        rows = [J[:3]]
        if R_target is not None:
            e.append(self.rot_weight_m * _rotvec(np.asarray(R_target) @ R.T))
            rows.append(self.rot_weight_m * J[3:])
        elif keep_axis is not None:
            idx, want = self._axis_spec(keep_axis)
            # rotation that would bring the present axis onto the wanted one
            e.append(self.rot_weight_m * _axis_error(R[:, idx], want, R[:, (idx + 1) % 3]))
            rows.append(self.rot_weight_m * J[3:])
        return np.concatenate(e), np.vstack(rows)

    @staticmethod
    def _axis_spec(keep_axis):
        if isinstance(keep_axis, (tuple, list)) and len(keep_axis) == 2 and isinstance(keep_axis[0], str):
            idx = "xyz".index(keep_axis[0])
            v = np.asarray(keep_axis[1], dtype=float)
        else:
            idx, v = 0, np.asarray(keep_axis, dtype=float)
        return idx, v / np.linalg.norm(v)

    def fk_tip(self, q):
        self._set(q)
        return self._tip_pose()

    def _dls(self, J, e, q, lam=None):
        """Damped least squares with joint-limit weighting: a joint nearer a limit than its own stopping angle at
        its full speed (limit_stop_rad; a joint whose speed is neither rated nor measured yet: at rest) is slowed. The damping grows only near a singularity (smallest singular
        value under 5 % of the arm's length scale) unless lam is given."""
        margin = np.minimum(q - self.q_lo, self.q_hi - q)  # rad to the nearer limit
        w = np.clip(margin / self._limit_slow, 0.05, 1.0)
        Jw = J * w
        if lam is None:
            eps = 0.05 * self.length_scale
            smin = float(np.linalg.svd(Jw, compute_uv=False)[-1]) if J.shape[0] <= self.n else 0.0
            lam2 = 0.0 if smin >= eps else (1.0 - (smin / eps) ** 2) * (0.5 * eps) ** 2
            lam2 = max(lam2, 1e-10)
        else:
            lam2 = lam ** 2
        A = Jw @ Jw.T + lam2 * np.eye(J.shape[0])
        dq = w * (Jw.T @ np.linalg.solve(A, e))
        return dq

    def ik(self, q0, p_target, R_target=None, keep_axis=None, iters=200, tol=1e-3, restarts=8, seed=0, rot_tol=None):
        """Every arm joint in the solve, limits respected. Returns (q, ok, position error in m). ok: the tip within
        tol (m) and the orientation within rot_tol (rad; default 10 x tol, about 0.57 deg): an arm that cannot hold
        an axis exactly (a five joint arm off its own plane) can be asked for its nearest pose within a few degrees."""
        p_target = np.asarray(p_target, dtype=float)
        rtol = 10 * tol if rot_tol is None else float(rot_tol)
        starts = [self.clip(q0), self.clip(self.q_home)]
        rng = np.random.default_rng(seed)
        for _ in range(restarts):
            starts.append(rng.uniform(self.q_lo, self.q_hi))
        best = (self.clip(q0), False, float("inf"), float("inf"))
        for qs in starts:
            q = qs.copy()
            for _ in range(iters):
                e, J = self._task_error(q, p_target, R_target, keep_axis)
                pos_err = float(np.linalg.norm(e[:3]))
                rot_err = float(np.linalg.norm(e[3:])) / self.rot_weight_m if e.size > 3 else 0.0
                if pos_err < tol and rot_err < rtol:
                    return q, True, pos_err
                dq = self._dls(J, e, q)
                step = np.max(np.abs(dq))
                if step > 0.3:
                    dq *= 0.3 / step
                q = self.clip(q + dq)
            e, _ = self._task_error(q, p_target, R_target, keep_axis)
            pos_err = float(np.linalg.norm(e[:3]))
            rot_err = float(np.linalg.norm(e[3:])) / self.rot_weight_m if e.size > 3 else 0.0
            if (pos_err + rot_err * self.rot_weight_m) < best[2] + best[3] * self.rot_weight_m:
                best = (q.copy(), False, pos_err, rot_err)
        return best[0], False, best[2]

    def step_toward(self, q, dp, drot=None, lam=None):
        """dq moving the tip by dp (m, frame B) and, when given, rotating it by drot (rad vector, frame B)."""
        q = self.clip(q)
        J = self.jac(q)
        if drot is None:
            e, Js = np.asarray(dp, dtype=float), J[:3]
        else:
            e = np.concatenate([np.asarray(dp, dtype=float), self.rot_weight_m * np.asarray(drot, dtype=float)])
            Js = np.vstack([J[:3], self.rot_weight_m * J[3:]])
        dq = self._dls(Js, e, q, lam)
        return self.clip(q + dq) - q

    # ---------- statics ----------
    def gravity_torque(self, q):
        """Joint torques (N m) that hold the arm still at q against gravity."""
        self._set(q)
        mujoco.mj_forward(self.mj, self.d)
        return self.d.qfrc_bias[self._dadr].copy()

    def tip_force_capability(self, q, direction):
        """Largest steady force (N) the tip can push on the world along direction (frame B) at q: the smaller of
        what the joint torque limits allow after holding gravity, and what the position servo holds without
        folding (push_stability). None when neither bounds it (no joint carries a push along that direction and
        the servo does not fold): the robot's ratings then say nothing about that push."""
        dvec = np.asarray(direction, dtype=float)
        dvec = dvec / np.linalg.norm(dvec)
        g = self.gravity_torque(q)
        a = self.jac(q)[:3].T @ dvec  # tau = g + F a holds a push of F along d
        F = float("inf")
        for ai, gi, li in zip(a, g, self.torque_limit):
            if abs(gi) > li:
                return 0.0
            if ai > 1e-9:
                F = min(F, (li - gi) / ai)
            elif ai < -1e-9:
                F = min(F, (li + gi) / -ai)
        F = min(F, self.push_stability(q, dvec))
        return float(F) if math.isfinite(F) else None

    def push_stability(self, q, direction):
        """Largest push (N) along direction the position servo holds without folding. The push's reaction loads
        the joints through the tip Jacobian, and as the arm gives the Jacobian changes: past this load the joint
        loads grow faster than the servo stiffness kp pulls back, and the arm buckles like a column however much
        torque is left (a soft servo on a strong arm, or any arm pushed along its own stretched length). It is
        the F at which kp + F d(J^T d)/dq stops being positive definite; inf when the load only steadies the
        arm. The arm's own weight is not counted."""
        dvec = np.asarray(direction, dtype=float)
        dvec = dvec / np.linalg.norm(dvec)
        q = np.asarray(q, dtype=float)
        eps = 1e-5
        H = np.zeros((self.n, self.n))
        for k in range(self.n):
            dq = np.zeros(self.n)
            dq[k] = eps
            H[:, k] = (self.jac(q + dq)[:3].T @ dvec - self.jac(q - dq)[:3].T @ dvec) / (2 * eps)
        s = 1.0 / np.sqrt(np.asarray(self.spec["kp"], dtype=float))
        lam = float(np.linalg.eigvalsh(0.5 * (H + H.T) * s[:, None] * s[None, :])[0])
        return float("inf") if lam > -1e-12 else -1.0 / lam

    # ---------- collision capsules ----------
    def _body_points(self, bid):
        """Collision geometry of one body, as points in that body's frame (at the current pose)."""
        m, d = self.mj, self.d
        Rb, pb = d.xmat[bid].reshape(3, 3), d.xpos[bid]
        pts = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != bid:
                continue
            Rg, pg = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mid = m.geom_dataid[g]
                v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            else:
                s = m.geom_size[g]
                t = m.geom_type[g]
                if t == mujoco.mjtGeom.mjGEOM_BOX:
                    hx, hy, hz = s
                elif t in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
                    hx, hy, hz = s[0], s[0], s[1] + (s[0] if t == mujoco.mjtGeom.mjGEOM_CAPSULE else 0)
                else:
                    hx = hy = hz = m.geom_rbound[g]
                v = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            pts.append(((v @ Rg.T + pg) - pb) @ Rb)
        return np.vstack(pts) if pts else np.zeros((0, 3))

    def _fit_capsules(self):
        """One capsule per moving body (and the base), fitted to its geometry at home: axis = principal direction."""
        self._set(self.q_home)
        m = self.mj
        bodies = [self._root] + sorted({b for b in range(1, m.nbody) if self._moves_with_arm(b)})
        defs = []
        for b in bodies:
            P = self._body_points(b)
            if len(P) < 2:
                continue
            c = P.mean(axis=0)
            _, _, vt = np.linalg.svd(P - c, full_matrices=False)
            ax = vt[0]
            t = (P - c) @ ax
            perp = np.linalg.norm((P - c) - np.outer(t, ax), axis=1)
            r = float(np.percentile(perp, 90))
            lo, hi = float(np.percentile(t, 2)), float(np.percentile(t, 98))
            p0 = c + ax * min(lo + r, (lo + hi) / 2)
            p1 = c + ax * max(hi - r, (lo + hi) / 2)
            defs.append((m.body(b).name, b, p0, p1, r))
        return defs

    def _moves_with_arm(self, b):
        m = self.mj
        while b != 0:
            if b in self._jbody:
                return True
            b = m.body_parentid[b]
        return False

    def _capsules_now(self):
        out = []
        for name, b, p0, p1, r in self._capsule_defs:
            R, p = self.d.xmat[b].reshape(3, 3), self.d.xpos[b]
            out.append((name, R @ p0 + p, R @ p1 + p, r))
        return out

    def collision_capsules(self, q):
        """(name, p0, p1, radius) in frame B; the first entry is the base, which rests on the mounting plane. The
        jaws stand where set_jaws last posed them."""
        geom = getattr(self, "_qgeom", None)
        if geom is None:
            self._set(q)
        else:
            full, self._qfull = self._qfull, geom
            try:
                self._set(q)
            finally:
                self._qfull = full
        return self._capsules_now()

    # ---------- self collision: the model file's own collision geometry ----------
    def _arm_key(self, b):
        """Index of the deepest arm joint that carries body b (-1: none, the base)."""
        m = self.mj
        while b != 0:
            if b in self._jbody:
                return max(i for i, jb in enumerate(self._jbody) if jb == b)
            b = int(m.body_parentid[b])
        return -1

    def _self_setup(self):
        """The link pairs the arm's own motion can bring together, each with the geom pairs the physics would test
        and its clearance at home. Left out, as the physics leaves them out: adjacent links (one is the other's
        parent, through bodies fixed to either; their joint limits are their collision model), pairs the model file
        excludes, geoms whose contact bits do not meet. Also left out: two bodies no arm joint lies between (jaws
        of one gripper, a hand and its paddle), which no arm motion moves against each other."""
        m, d = self.mj, self.d
        col = {g for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]}
        every = not col  # a file with no collision geometry at all: every geom stands for its body
        excluded = {(int(x) >> 16, int(x) & 0xFFFF) for x in m.exclude_signature}
        bodies = [self._root] + sorted(b for b in range(1, m.nbody) if self._moves_with_arm(b) and b != self._root)
        geoms = {b: [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and (every or g in col)] for b in bodies}
        weld = m.body_weldid
        self._set(self.q_home)
        names, home, ga_all, gb_all, pair_of = [], [], [], [], []
        for i, a in enumerate(bodies):
            for b in bodies[i + 1:]:
                wa, wb = int(weld[a]), int(weld[b])
                if (self._arm_key(a) == self._arm_key(b) or int(weld[m.body_parentid[wa]]) == wb
                        or int(weld[m.body_parentid[wb]]) == wa or (a, b) in excluded or (b, a) in excluded):
                    continue
                gp = [(ga, gb) for ga in geoms[a] for gb in geoms[b]
                      if every or (m.geom_contype[ga] & m.geom_conaffinity[gb])
                      or (m.geom_contype[gb] & m.geom_conaffinity[ga])]
                if not gp:
                    continue
                home.append(min(float(mujoco.mj_geomDistance(m, d, ga, gb, 10.0, None)) for ga, gb in gp))
                for ga, gb in gp:
                    ga_all.append(ga)
                    gb_all.append(gb)
                    pair_of.append(len(names))
                names.append((m.body(a).name, m.body(b).name))
        self._sp_names = names
        self._sp_home = np.asarray(home, dtype=float)
        # the nearest each pair may come whatever margin is asked: a pair apart at home keeps part of its home
        # clearance (the slack, at most half of it), so the check still fires before the pair meets; a pair that
        # touches at home may go the slack deeper
        h = self._sp_home
        self._sp_allow = np.where(h > SELF_TOUCH_M, h - np.minimum(SELF_HOME_SLACK_M, np.maximum(h, 0.0) / 2.0),
                                  h - SELF_HOME_SLACK_M)
        self._sp_ga = np.asarray(ga_all, dtype=int)
        self._sp_gb = np.asarray(gb_all, dtype=int)
        self._sp_pair = np.asarray(pair_of, dtype=int)
        self._sp_rsum = (m.geom_rbound[self._sp_ga] + m.geom_rbound[self._sp_gb]) if ga_all else np.zeros(0)

    def _self_near(self, thr):
        """{pair index: distance (m)} of the pairs nearer than thr[pair] at the pose now set."""
        if not len(self._sp_ga):
            return {}
        m, d = self.mj, self.d
        c = d.geom_xpos
        gap = np.linalg.norm(c[self._sp_ga] - c[self._sp_gb], axis=1) - self._sp_rsum  # bounding spheres first
        out = {}
        for k in np.nonzero(gap < thr[self._sp_pair])[0]:
            p = int(self._sp_pair[k])
            t = float(thr[p])
            # distances come back capped at the third-last argument; a threshold under zero asks for a depth
            dist = float(mujoco.mj_geomDistance(m, d, int(self._sp_ga[k]), int(self._sp_gb[k]), max(t, 0.0) + 1e-9,
                                                None))
            if dist < t and dist < out.get(p, float("inf")):
                out[p] = dist
        return out

    def self_collisions(self, q, margin=0.0):
        """Link pairs (name_a, name_b) whose collision geometry is nearer than margin (m) at q; margin 0 is true
        contact. A pair that sits nearer than the margin at home keeps its home clearance as its allowance: it
        counts only once it is nearer than at home by SELF_HOME_SLACK_M (by half its home clearance when that is
        less), so home itself never counts and a pair apart at home still counts before it meets. Adjacent links
        never count."""
        self._set(q)
        thr = np.minimum(float(margin), self._sp_allow)
        return [self._sp_names[p] for p in sorted(self._self_near(thr))]

    def self_clearances(self, q, within=0.02):
        """[(name_a, name_b, distance m, distance at home m)] of the link pairs nearer than `within` at q, nearest
        first; a negative distance is a depth of overlap."""
        self._set(q)
        near = self._self_near(np.full(len(self._sp_names), float(within)))
        rows = [(self._sp_names[p][0], self._sp_names[p][1], dist, float(self._sp_home[p])) for p, dist in near.items()]
        return sorted(rows, key=lambda r: r[2])

    def plane_clearance(self, q):
        """Lowest point (m) of any moving capsule and the tip above the mounting plane z = 0."""
        caps = self.collision_capsules(q)[1:]
        low = min(min(p0[2], p1[2]) - r for _, p0, p1, r in caps) if caps else float("inf")
        return float(min(low, self.fk_tip(q)[0][2]))

    # ---------- camera ----------
    def project(self, q, p_world):
        """(u, v, depth) of a frame-B point in the wrist image; depth in m along the optical axis (<= 0: behind)."""
        self._set(q)
        pc = self.d.cam_xpos[self._cam]
        Rc = self.d.cam_xmat[self._cam].reshape(3, 3)
        x, y, z = Rc.T @ (np.asarray(p_world, dtype=float) - pc)
        depth = -z
        if depth <= 1e-9:
            return float("nan"), float("nan"), float(depth)
        u = self.cam_w / 2.0 + self.f_px * x / depth
        v = self.cam_h / 2.0 - self.f_px * y / depth
        return float(u), float(v), float(depth)

    def pixel_ray(self, q, u, v):
        self._set(q)
        pc = self.d.cam_xpos[self._cam].copy()
        Rc = self.d.cam_xmat[self._cam].reshape(3, 3)
        dc = np.array([(u - self.cam_w / 2.0) / self.f_px, -(v - self.cam_h / 2.0) / self.f_px, -1.0])
        dw = Rc @ dc
        return pc, dw / np.linalg.norm(dw)

    # ---------- reach and the body document ----------
    def reach(self, n_samples=4000, seed=0):
        if self._reach is not None and self._reach[0] == (n_samples, seed):
            return self._reach[1]
        from rcpx.model_body import reach_summary

        out = reach_summary(self, n_samples, seed)
        self._reach = ((n_samples, seed), out)
        return out

    def describe(self):
        from rcpx.model_body import describe

        return describe(self)

    # ---------- the robot's own shape, ratings and stopping (SPEC-R4 A8.4, Q1, Q2, Q4) ----------
    def hulls(self):
        """{body: {"points_m": N x 3 in the body frame, "tol_m"}}: the convex shape each body collides with."""
        return model_tool.body_hulls(self)

    def e_fit(self):
        """{body: m}: the farthest a hull point stands out of that body's capsule."""
        return model_tool.fit_errors(self)

    def f_tool(self):
        """(N, how): what the tool itself is built to bear (stated, the jaw's own range at the pad, or the wrist's)."""
        return model_tool.tool_force(self)

    def f_hand(self):
        """(N, kg): the weight of everything the last arm joint moves."""
        return model_tool.hand_weight(self)

    def set_motion(self, **known):
        """What a session has learned of the robot's own motion (qd: joint speeds it may run, rad/s; acc: joint
        accelerations, rad/s^2; lag_s: tracking lag per joint, s) replaces the model's own until-known values."""
        self.motion = model_solve.motion_of(self, **known)
        self._limit_slow = self.limit_stop_rad(self.motion["qd_known"])
        return self.motion

    def limit_stop_rad(self, qd=None):
        """Per joint (rad): how near a limit the joint can no longer be kept off it (model_solve.limit_stop)."""
        return model_solve.limit_stop(self, qd)

    def self_margin(self, qd=None):
        """How far apart (m) two links must stay: the fit error of the geometry the distance is measured on, plus
        what the joints' tracking lag can move a link at joint speeds qd (model_solve.self_margin)."""
        return model_solve.self_margin(self, qd)

    def postures(self, q0, p, R_target=None, keep_axis=None, **kw):
        """Distinct postures that reach p with the hand at R_target (or keep_axis): rcpx.model_solve.postures."""
        from rcpx.model_solve import postures

        return postures(self, q0, p, R_target=R_target, keep_axis=keep_axis, **kw)

    def axis_options(self, q0, p, want, **kw):
        """The nearest tool-axis direction the arm holds at p: rcpx.model_solve.axis_options."""
        from rcpx.model_solve import axis_options

        return axis_options(self, q0, p, want, **kw)


def _axis_error(have, want, side):
    """Rotation vector (rad) turning unit axis `have` onto `want` the short way. Its length is the angle between
    them, so an axis pointing exactly the wrong way is an error of pi (the cross product alone is zero there);
    `side` is any direction square to `have`, the axis to turn about in that case."""
    c = np.cross(have, want)
    s, dot = float(np.linalg.norm(c)), float(have @ want)
    if s < 1e-9:
        return np.zeros(3) if dot > 0 else math.pi * np.asarray(side, dtype=float)
    return c * (math.atan2(s, dot) / s)


def _seg_dist(a0, a1, b0, b1):
    """Shortest distance between segments a0-a1 and b0-b1."""
    d1, d2, r = a1 - a0, b1 - b0, a0 - b0
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a < 1e-12 and e < 1e-12:
        return float(np.linalg.norm(r))
    if a < 1e-12:
        s, t = 0.0, np.clip(f / e, 0, 1)
    else:
        c = d1 @ r
        if e < 1e-12:
            t, s = 0.0, np.clip(-c / a, 0, 1)
        else:
            b = d1 @ d2
            den = a * e - b * b
            s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0:
                t, s = 0.0, np.clip(-c / a, 0, 1)
            elif t > 1:
                t, s = 1.0, np.clip((b - c) / a, 0, 1)
    return float(np.linalg.norm(a0 + d1 * s - (b0 + d2 * t)))
