# SPDX-License-Identifier: AGPL-3.0-only
"""Which part touched (I20 touched_by): fit the measured external joint torque to J_p(q)^T f for every candidate
point on the robot and rank the parts by how well a single point force explains the torque.

The torque is the closed-loop estimate of obstacles_torque.py (sign as Sense.ext: ext = J^T F for a force F the
robot puts on the world). The point Jacobian J_p of a point p fixed to body b comes from
mujoco.mj_jac at the body's origin, shifted to p (J_p = J_o - [p - o]x J_w), on the runtime's own copy of the
robot model. Candidates: every tool surface centre (the spec's tool surfaces), points along every moving collision
capsule's axis (a link; the bodies that carry the tool count as the tool), points on every face of each mounted
part, and, for each body, the point of each of its parts nearest the line of action a single force on that body
would need (from a free wrench fit on the body: a contact with friction, or off a capsule's axis, does not push
through the axis, so axis points alone miss it and the off-axis face points of a mounted box fit it by accident). Rows
are weighted by 1 / torque limit, so a wrist joint's few newton metres count as much as a shoulder's
many. A part's score is the relative residual of its best candidate; with refine (the stop's own fit) each part's
volume is also searched for its best point, so no part loses for want of a sampled point. A candidate whose fitted
push points against the motion that brought it there (joint speeds before the push rose) is ranked down: a point
moving away cannot be pressing. On an exact tie the line of action runs through several parts (one rigid piece, or
a line that also crosses the axes of the joints an outer part adds, as a push along the last joint's axis does);
the world is on the side the robot pushes toward, so the part whose surface the line leaves last that way is named.
touched_by is ambiguous when the runner-up part fits nearly as well, when the best fit is poor, when the force
is too small to read, or when dry joint friction the model lacks (up to the measured friction share of each joint's limit, along
each turning joint's motion) explains the torque about as well as the best part; tool_excluded says the tool itself
fits clearly worse and friction does not explain the torque. On a five joint arm a point chosen on
its last body fits any reading, so such a best fit is ambiguous unless the runner-up is worse by the torque error band.
While the hand holds or squeezes something (hand_wrench), the tool may carry a force AND a moment (a held part
pressed on its support, two jaws squeezing): the tool's residual is then the best wrench on the hand's body, which on
an arm of six joints or fewer fits any reading, so the tool is never ruled out there and a part is only possible.

Limits, said plainly: a force whose line of action passes through a joint axis gives that joint no torque, so a
contact on a link close to the base can be indistinguishable from one on the next link out; a point anywhere
along the line of action fits equally, so the point along a part is uncertain (point_spread_mm says how much). The
parts are RobotModel's collision capsules: where a capsule is a poor fit to a link's real shape, a contact on the
part of the link it leaves out is not explained by that link (ambiguous or another part's name).
"""

import numpy as np
import mujoco
from scipy.optimize import least_squares

from bench.robots import home_qpos
from rcpx.r2 import obstacles_bounds as derived
from rcpx.bounds_ratios import TWO_READ
# The free-fit floor follows the largest relative error of the torque estimate.
# Friction evidence carries the measured torque estimate error, added below.

MM = 1000.0
AXIS_POINTS = 7          # candidate points along each capsule axis
REFINE_EVALS = 30        # refine: evaluations of the bounded search per part
FACE_GRID = (-0.5, 0.0, 0.5)   # face points per mounted box face, as fractions of the half extents
OPPOSED_COS = -0.3       # a candidate whose fitted push points this far against its own motion is implausible
MOVING_M_S = 0.005       # ... when the point was moving at least this fast just before the reading
TURNING_RAD_S = 0.01     # a joint at least this fast at both ends of the period, one way, is turning


def _skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def _seg_line(c0, c1, p, u):
    """Closest points of the segment c0-c1 and the line p + s u (u unit): (point on the segment, point on the
    line)."""
    d = c1 - c0
    dd = float(d @ d)
    w = c0 - p
    if dd < 1e-12:
        t = 0.0
    else:
        b = float(d @ u)
        den = dd - b * b
        t = 0.0 if den < 1e-12 else float(np.clip((b * float(u @ w) - float(d @ w)) / den, 0.0, 1.0))
    ps = c0 + t * d
    return ps, p + float((ps - p) @ u) * u


def _exit_point(vol, p, u):
    """Where the line p + s u leaves the part's volume going along u (the side the robot pushes toward), or the
    volume's surface point nearest the line when the line misses it. For a capsule the exit is taken on the ball
    around the axis point nearest the line, so a line oblique to the axis gets a point a little inside the surface
    (still on the line, so its fit is the same)."""
    if vol[0] == "cap":
        _, c0, c1, r = vol
        ps, pL = _seg_line(c0, c1, p, u)
        gap = float(np.linalg.norm(pL - ps))
        if gap <= r:
            return pL + u * float(np.sqrt(max(r * r - gap * gap, 0.0)))
        return ps + r * (pL - ps) / gap
    _, c, R, half = vol
    x = R.T @ (p + float((c - p) @ u) * u - c)
    if np.all(np.abs(x) <= half):
        ul = R.T @ u
        s = min(((np.sign(ul[i]) * half[i] - x[i]) / ul[i] for i in range(3) if abs(ul[i]) > 1e-12), default=0.0)
        x = x + s * ul
    return c + R @ np.clip(x, -half, half)


def _search_box(vol):
    """(origin, axes as columns, lower, upper) of a box that holds the part's volume, for the bounded search."""
    if vol[0] == "box":
        _, c, R, half = vol
        h = np.maximum(np.asarray(half, dtype=float), 1e-6)
        return c, R, -h, h
    _, c0, c1, r = vol
    d = c1 - c0
    L = float(np.linalg.norm(d))
    a = d / L if L > 1e-9 else np.array([1.0, 0.0, 0.0])
    e1 = np.cross(a, [1.0, 0.0, 0.0] if abs(a[0]) < 0.9 else [0.0, 1.0, 0.0])
    e1 = e1 / np.linalg.norm(e1)
    R = np.column_stack([a, e1, np.cross(a, e1)])
    r = max(float(r), 1e-6)
    return c0, R, np.array([-r, -r, -r]), np.array([L + r, r, r])


def _inside(vol, p):
    """p moved onto the part's volume (a capsule: the nearest point of it; a box: clamped)."""
    if vol[0] == "box":
        _, c, R, half = vol
        return c + R @ np.clip(R.T @ (p - c), -half, half)
    _, c0, c1, r = vol
    d = c1 - c0
    dd = float(d @ d)
    t = 0.0 if dd < 1e-12 else float(np.clip((p - c0) @ d / dd, 0.0, 1.0))
    q = c0 + t * d
    gap = float(np.linalg.norm(p - q))
    return p if gap <= r else q + r * (p - q) / gap


class PartFit:
    def __init__(self, model, tool_bodies, mounted, B=None):
        self.B = B
        self.quiet_residual = 0
        self.model = model
        self.m = model.mj
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in model.joint_names])
        self.dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in model.joint_names])
        self.qfull = home_qpos(m, model.spec)
        self.w = 1.0 / np.maximum(np.asarray(model.torque_limit, dtype=float), 1e-6)
        self.tool_bodies = set(tool_bodies)
        self.root = m.body(model._root).name
        self.surfaces = [(s["name"], m.site("surface_" + s["name"]).id) for s in model.spec.get("tool_surfaces", [])]
        self.cam_body = int(m.cam_bodyid[model._cam])
        self.hand_body = int(m.site_bodyid[m.site("tip").id])
        self.mounted = list(mounted)
        self.jbodies = {int(m.jnt_bodyid[m.joint(j).id]) for j in model.joint_names}
        self._jp = np.zeros((3, m.nv))
        self._jr = np.zeros((3, m.nv))

    def _depth(self, b, jbodies):
        """How many arm joints move body b (the joints on its path to the root)."""
        n = 0
        while b > 0:
            n += b in jbodies
            b = int(self.m.body_parentid[b])
        return n

    # ------------------------------------------------------------------------------------------ jacobians
    def _pose(self, q):
        d = self.d
        d.qpos[:] = self.qfull
        d.qpos[self.qadr] = np.asarray(q, dtype=float)
        d.qvel[:] = 0.0
        mujoco.mj_kinematics(self.m, d)
        mujoco.mj_comPos(self.m, d)
        self._cache = {}

    def _body_jac(self, bid):
        if bid not in self._cache:
            o = self.d.xpos[bid].copy()
            mujoco.mj_jac(self.m, self.d, self._jp, self._jr, o, bid)
            self._cache[bid] = (o, self._jp[:, self.dadr].copy(), self._jr[:, self.dadr].copy())
        return self._cache[bid]

    def point_jac(self, bid, p):
        """3 x n translational Jacobian (arm joints) of the point p (frame B) fixed to body bid."""
        o, Jo, Jr = self._body_jac(bid)
        return Jo - _skew(np.asarray(p, dtype=float) - o) @ Jr

    def lines(self, bid, b):
        """Lines of action (point, unit direction, frame B) along which one force on body bid gives the weighted
        joint torque b as nearly as any force and moment on that body can: from the minimum-norm wrench, and, when
        the body leaves the wrench one free direction (a body moved by one joint fewer than six), also from the
        members of that family with no moment along the force (a single force has none). The pose must be set."""
        o, Jo, Jr = self._body_jac(bid)
        A = self.w[:, None] * np.vstack([Jo, Jr]).T           # n x 6: torque from (force, moment about o)
        _, S, Vt = np.linalg.svd(A)
        if S.size == 0 or not np.isfinite(S[0]) or S[0] < 1e-12:
            return []
        k = int(np.sum(S > 1e-9 * S[0]))
        w0 = np.linalg.lstsq(A, b, rcond=1e-9)[0]
        ws = [w0]
        if 6 - k == 1:
            v = Vt[-1]
            a2 = float(v[:3] @ v[3:])
            a1 = float(w0[:3] @ v[3:] + v[:3] @ w0[3:])
            a0 = float(w0[:3] @ w0[3:])
            roots = []
            if abs(a2) > 1e-12:
                disc = a1 * a1 - 4.0 * a2 * a0
                if disc >= 0.0:
                    roots = [(-a1 + sg * np.sqrt(disc)) / (2.0 * a2) for sg in (1.0, -1.0)]
            elif abs(a1) > 1e-12:
                roots = [-a0 / a1]
            ws += [w0 + x * v for x in roots if np.isfinite(x)]
        out = []
        for w in ws:
            f, mo = w[:3], w[3:]
            nf = float(np.linalg.norm(f))
            if nf < 1e-9:
                continue
            out.append((o + np.cross(f, mo) / (nf * nf), f / nf))
        return out

    def hand_residual(self, b, nb):
        """Relative residual of the best force and moment on the hand's body (the body that carries the tip) for the
        weighted torque b: a held part or a squeeze loads the hand with a wrench, not one point force. On an arm of
        six joints or fewer it fits any reading. The pose must be set."""
        _, Jo, Jr = self._body_jac(self.hand_body)
        A = self.w[:, None] * np.vstack([Jo, Jr]).T
        wv = np.linalg.lstsq(A, b, rcond=1e-9)[0]
        return float(np.linalg.norm(b - A @ wv)) / nb

    @staticmethod
    def turning(qd_period):
        """(turning, sign) per joint for the period's joint speeds (start, end): turning one way through it."""
        v0, v1 = (np.asarray(v, dtype=float) for v in qd_period)
        on = (np.abs(v0) >= TURNING_RAD_S) & (np.abs(v1) >= TURNING_RAD_S) & (np.sign(v0) == np.sign(v1))
        return on, np.sign(v1)

    def friction_residual(self, b, nb, qd_period, seen=None):
        """Relative residual of the best explanation of the weighted torque b (ext / torque limit) by dry friction
        in the joints alone, which the robot model may lack: a joint turning one way through the period reads up to
        the measured friction share of its limit along its motion (the arm pushes through the friction, so ext has the motion's
        sign), or less when seen (2 x n: the least friction-signed reading of a quiet period of the same call with
        the joint turning the positive, then the negative way; inf: none) bounds it; a joint at rest or turning
        round may hold up to the measured friction share either way. None without the period's joint speeds (start, end)."""
        if qd_period is None:
            return None
        if any(np.shape(v) != b.shape for v in qd_period):
            return None
        on, sg = self.turning(qd_period)
        base, source = derived.friction(self.B, len(b))
        base = np.max(base, axis=0) if base.ndim > 1 else base
        cap = base.copy()
        if seen is not None:
            up, down = (np.minimum(base, np.asarray(x, dtype=float) + self.B.e_tau()) for x in seen)   # the reading plus the estimate's own error
            cap = np.where(sg > 0, up, down)
        lo = np.where(on, np.where(sg > 0, 0.0, -cap), -base)
        hi = np.where(on, np.where(sg > 0, cap, 0.0), base)
        return float(np.linalg.norm(b - np.clip(b, lo, hi))) / nb

    def tool_residual_quick(self, q, ext):
        """Least relative residual of a single force at a tool surface centre for the joint torque ext (the cheap
        part of the tool's score: the full fit tries these points and more tool points, so its tool residual is
        never larger for the same torque). None when there is nothing to read or no tool surface."""
        b = self.w * np.asarray(ext, dtype=float)
        nb = float(np.linalg.norm(b))
        if not np.all(np.isfinite(b)) or nb < 1e-9:
            return None
        self._pose(q)
        best = None
        for _, sid in self.surfaces:
            A = self.w[:, None] * self.point_jac(int(self.m.site_bodyid[sid]), self.d.site_xpos[sid]).T
            f = np.linalg.lstsq(A, b, rcond=1e-9)[0]
            rel = float(np.linalg.norm(b - A @ f)) / nb
            best = rel if best is None else min(best, rel)
        return best

    # ------------------------------------------------------------------------------------------ candidates
    def candidates(self, q, b=None):
        """[(part, kind, detail, body id, point in frame B, extra, volume)] at joint pose q (the pose is set here).
        With the weighted torque b, each part also gets its point nearest each line of action of its body (lines).
        volume: ("cap", p0, p1, r) or ("box", centre, axes, half) of the part the point lies on (None: a site)."""
        self._pose(q)
        m, d = self.m, self.d
        out = []
        line_of = {}

        def body_lines(bid):
            if bid not in line_of:
                line_of[bid] = [] if b is None else self.lines(bid, b)
            return line_of[bid]

        for name, sid in self.surfaces:
            out.append(("tool", "tool", name, int(m.site_bodyid[sid]), d.site_xpos[sid].copy(), None, None))
        for name, p0, p1, r in self.model.collision_capsules(q):
            if name == self.root:
                continue
            bid = m.body(name).id
            tool = name in self.tool_bodies
            part, kind = ("tool", "tool") if tool else (name, "link")
            vol = ("cap", p0, p1, r)
            for t in np.linspace(0.0, 1.0, AXIS_POINTS):
                out.append((part, kind, f"{name} axis {t:.2f}", bid, p0 + t * (p1 - p0), {"axis": (p1 - p0), "r": r},
                            vol))
            for k, (pl, u) in enumerate(body_lines(bid)):
                out.append((part, kind, f"{name} line {k}", bid, _exit_point(vol, pl, u), None, vol))
        if self.mounted:
            pc, Rc = self.model.fk(q)["camera"]
            for mp in self.mounted:
                cw = pc + Rc @ mp["c"]
                vol = (("cap", cw, cw, mp["radius"]) if mp.get("shape") == "sphere"
                       else ("box", cw, Rc, mp["half"]))
                for k, (pl, u) in enumerate(body_lines(self.cam_body)):
                    out.append((mp["name"], "mounted", f"line {k}", self.cam_body, _exit_point(vol, pl, u), None, vol))
                c, half = mp["c"], mp["half"]
                if mp.get("shape") == "sphere":
                    for ax in np.eye(len(c)):
                        for sgn in (-1, 1):
                            point = cw + sgn * mp["radius"] * (Rc @ ax)
                            out.append((mp["name"], "mounted", "sphere", self.cam_body, point, None, vol))
                    continue
                for ax in range(3):
                    o1, o2 = [i for i in range(3) if i != ax]
                    for sgn in (-1.0, 1.0):
                        for fa in FACE_GRID:
                            for fb in FACE_GRID:
                                local = c.copy()
                                local[ax] += sgn * half[ax]
                                local[o1] += fa * half[o1]
                                local[o2] += fb * half[o2]
                                face = f"{'+' if sgn > 0 else '-'}{'xyz'[ax]} face"
                                out.append((mp["name"], "mounted", face, self.cam_body, pc + Rc @ local, None, vol))
        return out

    # ------------------------------------------------------------------------------------------ one candidate
    def _row(self, part, kind, detail, bid, p, extra, vol, b, nb, qd):
        J = self.point_jac(bid, p)
        A = self.w[:, None] * J.T
        f, *_ = np.linalg.lstsq(A, b, rcond=1e-9)
        rel = float(np.linalg.norm(b - A @ f)) / nb
        motion, rank, v = None, rel, None
        if qd is not None:
            v = J @ np.asarray(qd, dtype=float)
            nv, nf = float(np.linalg.norm(v)), float(np.linalg.norm(f))
            motion = float(v @ f) / (nv * nf) if nv > 1e-6 and nf > 1e-9 else None
            if motion is not None and motion < OPPOSED_COS and nv > MOVING_M_S:
                rank = rel + 1.0   # the point was moving away from where the force says it pressed
        free = "line" in detail.split() or detail.endswith(" refined")   # its point was chosen to fit this torque
        return {"part": part, "kind": kind, "detail": detail, "bid": bid, "p": np.asarray(p, dtype=float), "f": f,
                "rel": rel, "rank": rank, "motion": motion, "extra": extra, "vol": vol, "v": v, "free": free}

    def free_dof(self, bid):
        """Checks left over when the point on body bid is chosen to fit the torque: the joints that do not move the
        body (their torque must be zero) plus the joints beyond the five a single force on a free line can match.
        Zero (a five joint arm's last body): a chosen point fits any reading, so its residual is no evidence."""
        n = len(self.dadr)
        k = self._depth(int(bid), self.jbodies)
        return (n - k) + max(0, k - 5)

    def _refine(self, row, b, nb, qd):
        """The part of row searched from row's point for the point inside its volume whose single force explains
        the torque best (bounded least squares over the point; the force is solved at each trial point), moved
        along its line of action to where that line leaves the surface. None when nothing better is found."""
        o, R, lo, hi = _search_box(row["vol"])
        bid = row["bid"]

        def resid(x):
            A = self.w[:, None] * self.point_jac(bid, o + R @ x).T
            return (b - A @ np.linalg.lstsq(A, b, rcond=1e-9)[0]) / nb

        x0 = np.clip(R.T @ (row["p"] - o), lo + 1e-7, hi - 1e-7)
        try:
            sol = least_squares(resid, x0, bounds=(lo, hi), method="trf", x_scale=(hi - lo) / 2.0,
                                max_nfev=REFINE_EVALS)
        except (ValueError, np.linalg.LinAlgError):
            return None
        p = _inside(row["vol"], o + R @ sol.x)
        A = self.w[:, None] * self.point_jac(bid, p).T
        f = np.linalg.lstsq(A, b, rcond=1e-9)[0]
        nf = float(np.linalg.norm(f))
        if nf > 1e-9:
            p = _exit_point(row["vol"], p, f / nf)
        new = self._row(row["part"], row["kind"], row["detail"].split(" line ")[0].split(" axis ")[0] + " refined",
                        bid, p, None, row["vol"], b, nb, qd)
        return new if new["rel"] < row["rel"] - 1e-9 else None

    # ------------------------------------------------------------------------------------------ the fit
    def fit(self, q, ext, qd=None, refine=False, hand_wrench=False, qd_period=None, friction_seen=None):
        """touched_by for one torque reading, or None when there is nothing to read. refine: search each part's
        volume for its best point too (_refine; a few ms per part), so every part is scored by the best point it
        has, not by the nearest sampled one. hand_wrench: the hand holds or squeezes something, so the tool's
        residual is the best wrench on the hand (hand_residual), not the best single point force. qd_period: the
        joint speeds at the start and the end of the reading's period; when dry joint friction the model lacks
        (friction_residual; friction_seen: what quiet readings of the call showed of it) explains the torque about as
        well as the best part, the tool is not ruled out and the answer is ambiguous (an arm moving through free
        space with stiff joints reads like a push on a link)."""
        ext = np.asarray(ext, dtype=float)
        b = self.w * ext
        nb = float(np.linalg.norm(b))
        if not np.all(np.isfinite(b)) or nb < 1e-9:
            return None
        rows = [self._row(*c, b, nb, qd) for c in self.candidates(q, b)]

        def best_by_part(pool):
            out = {}
            for r in pool:  # rows keep a fixed order, so ties resolve the same way every run
                if r["part"] not in out or r["rank"] < out[r["part"]]["rank"] - 1e-12:
                    out[r["part"]] = r
            return out

        if refine:
            for r in best_by_part([r for r in rows if r["vol"] is not None]).values():
                new = self._refine(r, b, nb, qd)
                if new is not None:
                    rows.append(new)
        best_of = best_by_part(rows)
        ranked = sorted(best_of.values(), key=lambda r: r["rank"])
        # an exact tie: the torques fix only the line of action, and it runs through several parts (one rigid piece,
        # or a line that also crosses the axes of the joints an outer part adds, as a push along the last joint's
        # axis does). The world is on the side the robot pushes toward, so the part touched is the one whose
        # surface the line leaves last going that way.
        bands = derived.relative_bands(self.B, ranked[0]["rel"], self.quiet_residual)
        tie = bands["tie"]
        ties = [r for r in ranked if r["rank"] <= ranked[0]["rank"] + tie]
        push = ranked[0]["f"]
        npush = float(np.linalg.norm(push))
        if len(ties) > 1 and npush > 1e-9:
            u = push / npush
            names = {r["part"] for r in ties}

            def reach(r):
                return float(r["p"] @ u) + (r["extra"]["r"] if r["extra"] else 0.0)

            far = {}
            for r in rows:     # each tied part's furthest point that still fits as well as the tie
                if r["part"] in names and r["rank"] <= ranked[0]["rank"] + tie:
                    far[r["part"]] = max(far.get(r["part"], -np.inf), reach(r))
            top = max(ties, key=lambda r: (far.get(r["part"], reach(r)), -r["rank"]))
        else:
            top = min(ties, key=lambda r: r["rank"])
        others = [r for r in ranked if r["part"] != top["part"]]
        runner = others[0] if others else None
        margin = bands["ambiguous"]
        if top["free"] and self.free_dof(top["bid"]) == 0:
            margin = max(margin, bands["free"])   # its near zero residual is no evidence: only torque noise is
        tools = [r for r in ranked if r["kind"] == "tool"]
        tool_rank, tool_rel = (tools[0]["rank"], tools[0]["rel"]) if tools else (None, None)
        hand = self.hand_residual(b, nb) if hand_wrench else None
        if hand is not None and (tool_rank is None or hand < tool_rank):
            tool_rank, tool_rel = hand, hand
        fric = self.friction_residual(b, nb, qd_period, friction_seen)
        friction_doubt = fric is not None and top["kind"] != "tool" and fric - top["rank"] < margin
        tool_excluded = (top["kind"] != "tool" and not friction_doubt
                         and (tool_rank is None or tool_rank - top["rank"] >= margin))
        best = top
        same = [r for r in rows if r["part"] == top["part"] and r["rank"] <= top["rank"] + bands["ambiguous"]]
        moving = [r for r in same if r["motion"] is not None]
        if moving:  # along a part the torque cannot tell points on one line of action apart: prefer the point
            best = max(moving, key=lambda r: (r["motion"], -r["rel"]))  # whose motion the force opposes
        pts = np.array([r["p"] for r in same])
        spread = float(np.max(np.linalg.norm(pts - pts.mean(axis=0), axis=1))) * 2.0 if len(pts) > 1 else 0.0
        f = best["f"]
        fn = float(np.linalg.norm(f))
        point = best["p"].copy()
        if best["extra"] is not None and fn > 1e-9:  # a capsule: the contact is on its surface, facing the push
            ax = best["extra"]["axis"]
            la = float(np.linalg.norm(ax))
            perp = f - (f @ ax) * ax / (la * la) if la > 1e-9 else f
            if float(np.linalg.norm(perp)) > 1e-9:
                point = point + best["extra"]["r"] * perp / float(np.linalg.norm(perp))
        v = best["v"]
        approach = None if v is None or float(np.linalg.norm(v)) <= MOVING_M_S else v / float(np.linalg.norm(v))
        why = []
        if runner is not None and runner["rank"] - top["rank"] < margin:
            why.append(f"{runner['part']} fits nearly as well")
        if hand is not None and top["kind"] != "tool" and hand - top["rank"] < margin:
            why.append("the hand is holding or squeezing: a load through the hand explains the joint torques as well")
        if friction_doubt:
            why.append("dry friction in the joints from the measured or model torque threshold explains the reading as well")
        if top["rel"] > bands["poor"]:
            why.append("no single point force explains the joint torques well")
        if fn < self.B.f_thr(float(np.linalg.norm(v)) if v is not None else 0):
            why.append(f"the force is under {self.B.f_thr(float(np.linalg.norm(v)) if v is not None else 0):g} N")
        out = {"part": best["part"], "kind": best["kind"], "detail": best["detail"],
               "residual": round(top["rel"], 4),
               "runner_up": None if runner is None else {"part": runner["part"], "kind": runner["kind"],
                                                         "residual": round(runner["rel"], 4),
                                                         "against_motion": runner["rank"] > runner["rel"]},
               "ambiguous": bool(why), "tool_excluded": bool(tool_excluded and top["rel"] <= bands["poor"]
                                                            and fn >= self.B.f_thr(float(np.linalg.norm(v)) if v is not None else 0)),
               "tool_residual": None if tool_rel is None else round(tool_rel, 4),
               "force_n": round(fn, 2),
               "force_vec_n": [round(float(x), 2) for x in f],
               "normal_est": [round(float(x), 3) for x in (-f / fn if fn > 1e-9 else np.zeros(3))],
               "point_mm": [round(float(x) * MM, 1) for x in point],
               "point_spread_mm": round(spread * MM, 1),
               "approach_dir": None if approach is None else [round(float(x), 3) for x in approach]}
        if hand is not None:
            out["hand_residual"] = round(hand, 4)
        if fric is not None:
            out["friction_residual"] = round(fric, 4)
        if why:
            out["why_ambiguous"] = "; ".join(why)
        return out
