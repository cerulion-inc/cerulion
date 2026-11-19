# SPDX-License-Identifier: AGPL-3.0-only
"""The jaws of package place: opening, closing axis, the gap between the jaws and the hand's own shape, from the
robot's own model.

Everything comes from the robot spec's gripper block (joints, open and closed values, the servo gain and force range
when the spec gives them) and the loaded kinematic model; nothing is ground truth about the world:

- jaw sides: each gripper joint's body is a moving jaw; a gripper with one moving jaw closes against the body that
  carries that joint (the fixed jaw);
- jaw points: the tool tip site expressed in each side's body frame with the jaws closed; with the jaws open those
  points sit on each jaw's tip, so the opening is the distance between them and the gap's centre is their midpoint;
- the tool frame (the tip site's axes) carries the hand, so the closing axis and the gap centre in it do not depend
  on the arm's pose: they are read once, at the home pose;
- hulls: every collision shape of the hand as points (a mesh's own vertices, the corners of a box; the model's
  hulls give their stated tolerance), kept per jaw side and for the palm (the hand's shapes that reach across
  the gap). From them: the finger length (tip to the palm plane), the pad (the points of each jaw's face nearest
  the other jaw: how far its upper edge stands above the fingertip), each finger's box and the palm's box;
- squeeze: a position servo presses kp times its lag (command minus reading) up to its force range, less the jaw
  joint's dry friction; on a hinge that torque acts at the jaw point's distance from the axis. "Pressing" is a
  squeeze over the level the free jaws showed (Jaws.know, set by place_hold from the robot's self-knowledge or its
  one timed free close); before that level is known, a squeeze over what the model's own friction leaves.

SI inside (m, rad, N).
"""

import math

import mujoco
import numpy as np

N_TABLE = 81             # numeric: samples of opening fraction -> width (to turn a width into a command)
EPS = 1e-9               # numeric: a length or a ratio under this is zero
HALF = 0.5               # numeric: the middle of a width


def finite(*xs):
    try:
        return all(math.isfinite(float(x)) for x in xs)
    except (TypeError, ValueError):
        return False


class Jaws:
    """Jaw geometry and readings of one robot model (rcpx.model.RobotModel); every reader is None-safe."""

    def __init__(self, model):
        spec = model.spec
        g = spec["gripper"]
        m = model.mj
        self.m, self.d = m, mujoco.MjData(m)
        self.joints = list(g["joints"])
        jids = [m.joint(j).id for j in self.joints]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jids])
        self.closed = np.asarray(g["closed"], dtype=float)
        self.open = np.asarray(g["open"], dtype=float)
        self.span = self.open - self.closed
        self.kp = np.asarray(g["kp"], dtype=float) if g.get("kp") is not None else None
        fr = g.get("forcerange")
        self.force_range = np.asarray(fr, dtype=float) if fr is not None else None
        self.friction = np.asarray([float(m.dof_frictionloss[m.jnt_dofadr[j]]) for j in jids])
        bodies = [int(m.jnt_bodyid[j]) for j in jids]
        self.hinge = [int(m.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_HINGE) for j in jids]
        self.sides = list(bodies) if len(bodies) > 1 else [bodies[0], int(m.body_parentid[bodies[0]])]
        self.q_full = np.asarray(model._qfull, dtype=float).copy()   # home pose, jaws closed
        self.tip_site = m.site("tip").id
        d = self.d
        d.qpos[:] = self.q_full
        d.qpos[self.qadr] = self.closed
        mujoco.mj_kinematics(m, d)
        tip = d.site_xpos[self.tip_site].copy()
        self.local = [d.xmat[b].reshape(3, 3).T @ (tip - d.xpos[b]) for b in self.sides]
        lever = []
        for j, is_hinge in zip(jids, self.hinge):
            if not is_hinge:
                lever.append(1.0)
                continue
            a, ax = d.xanchor[j].copy(), d.xaxis[j].copy()
            r = tip - a
            lever.append(max(float(np.linalg.norm(r - (r @ ax) * ax)), EPS))
        self.lever = np.asarray(lever)
        self.fracs = np.linspace(0.0, 1.0, N_TABLE)
        self.table = np.array([self.width_m(self.q_of_frac(f)) for f in self.fracs])
        self.max_m = float(self.table.max())
        self.monotone = bool(np.all(np.diff(self.table) >= -EPS))
        a = self.points_tool(1.0)
        ax = a[0] - a[1]
        ax[0] = 0.0                                     # across the finger, not along it
        n = float(np.linalg.norm(ax))
        self.axis_T = ax / n if n > EPS else np.array([0.0, 1.0, 0.0])
        self.cross_T = np.cross(np.array([1.0, 0.0, 0.0]), self.axis_T)
        self.know = None        # {"press_n", "lag"}: the free jaws' own levels (place_hold.profile sets it)
        self.clear_m = 0.0      # one moving jaw: the fixed jaw comes down this far off the object's face (the pick
        #                         sets it from the pick point's own error before it plans)
        self._shape(model)

    # ---------------------------------------------------------------- the hand's own shape
    def _shape(self, model):
        """Hull points of the hand, split into each jaw side, the palm (the hand's shapes that reach across the gap)
        and the fixed finger of a one-jaw gripper (the tool body's shapes that do not). Body frames, read once."""
        m, d = self.m, self.d
        tip_body = int(m.site_bodyid[self.tip_site])
        group = {tip_body}                               # bodies welded to the tool body (no joint between)
        k = tip_body
        while k > 0 and int(m.body_jntnum[k]) == 0:
            k = int(m.body_parentid[k])
            if k > 0:
                group.add(k)
        moving = self.sides[:len(self.joints)]
        owner = {}
        for b in range(1, m.nbody):
            c = b
            while c > 0:
                if c in moving:
                    owner[b] = ("jaw", moving.index(c))
                    break
                if c in group:
                    owner[b] = ("hand", None)
                    break
                c = int(m.body_parentid[c])
        hulls = None
        try:
            hulls = model.hulls() if hasattr(model, "hulls") else None
        except Exception:    # a model whose hulls cannot be read: the shapes' own points stand in
            hulls = None
        self.hull_tol_m = 0.0
        shapes = []                                      # (kind, side or None, body id, points in the body frame)
        d.qpos[:] = self.q_full
        d.qpos[self.qadr] = self.q_of_frac(1.0)
        mujoco.mj_kinematics(m, d)
        for b, (kind, side) in owner.items():
            entry = (hulls or {}).get(m.body(b).name)
            if isinstance(entry, dict):
                self.hull_tol_m = max(self.hull_tol_m, float(entry.get("tol_m") or 0.0))
            # each collision shape by itself: a whole body's hull cannot tell a fixed jaw from the palm it is
            # part of, nor a pad from the finger behind it; the model's hulls give their stated tolerance
            Rb, pb = d.xmat[b].reshape(3, 3), d.xpos[b]
            for g in range(m.ngeom):
                if int(m.geom_bodyid[g]) != b or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                    continue
                shapes.append((kind, side, b, (_geom_points(m, d, g) - pb) @ Rb))
        self._shapes = shapes
        R, tip = self._tool_frame(0.0)
        d = self.d

        def tool(b, pts):
            return (pts @ d.xmat[b].reshape(3, 3).T + d.xpos[b] - tip) @ R
        mids = [float(np.mean(tool(b, pts) @ self.axis_T)) for kind, _, b, pts in shapes if kind == "jaw"]
        palm, fixed = [], []
        for kind, side, b, pts in shapes:
            if kind != "hand":
                continue
            a = tool(b, pts) @ self.axis_T
            across = bool(mids) and float(a.min()) < min(mids) and float(a.max()) > max(mids)
            (palm if across else fixed).append((b, pts))
        self._palm, self._fixed = palm, fixed
        x_palm = [float(tool(b, pts)[:, 0].max()) for b, pts in palm]
        self.finger_len_m = -max(x_palm) if x_palm and max(x_palm) < -EPS else None
        self.pad_span_m, self.pad_height_m = self._pad()

    def _tool_frame(self, frac):
        d = self.d
        d.qpos[:] = self.q_full
        d.qpos[self.qadr] = self.q_of_frac(frac)
        mujoco.mj_kinematics(self.m, d)
        return d.site_xmat[self.tip_site].reshape(3, 3).copy(), d.site_xpos[self.tip_site].copy()

    def _side_points(self, frac):
        """Per jaw side: its hull points in the tool frame at opening frac (the fixed jaw of a one-jaw gripper: the
        tool body's shapes that do not reach across the gap)."""
        R, tip = self._tool_frame(frac)
        d = self.d
        out = [[] for _ in self.sides]
        for kind, side, b, pts in self._shapes:
            if kind == "jaw":
                out[side].append((pts @ d.xmat[b].reshape(3, 3).T + d.xpos[b] - tip) @ R)
        if self.one_jaw:
            for b, pts in self._fixed:
                out[1].append((pts @ d.xmat[b].reshape(3, 3).T + d.xpos[b] - tip) @ R)
        return [np.vstack(p) if p else np.zeros((0, 3)) for p in out]

    def _faces(self, frac=0.0):
        """Per jaw side at opening frac: (its pad face's points in the tool frame, the sign that points toward the
        other jaw along the closing axis). The pad face is the part of the side's hull nearest the other jaw
        (within the hull's own stated tolerance); None for a side with no collision shape."""
        sides = self._side_points(frac)
        mid = float(np.mean([p @ self.axis_T for p in self.points_tool(frac)]))
        out = []
        for P in sides:
            if not len(P):
                out.append(None)
                continue
            a = P @ self.axis_T
            sign = 1.0 if float(np.mean(a)) < mid else -1.0     # toward the other jaw
            face = float(np.max(sign * a))
            out.append((P[sign * a >= face - self.hull_tol_m - EPS], sign))
        return out

    def _pad(self):
        """((upper edge, lower edge) of the pad along the pointing axis, m, tool frame; how far the pad's upper
        edge stands above the fingertip, m), jaws closed; the shorter pad of the sides counts."""
        best = None
        for got in self._faces(0.0):
            if got is None:
                continue
            x = got[0][:, 0]
            span = (float(x.min()), float(x.max()))
            if best is None or span[0] > best[0]:
                best = span
        if best is None:
            return None, None
        return best, max(0.0, -best[0])

    def _depth(self, depth_m=None):
        """How far up the fingers from their tips a thing between the jaws reaches (m): what the caller states,
        never past the palm; with nothing stated, the depth the hand's own rule takes a thing at when its top's
        error is not known (GIVE of the finger length: place_grasp). None: the palm is not known."""
        if self.finger_len_m is None:
            return None if depth_m is None else float(depth_m)
        from rcpx import bounds_ratios as R
        return R.GIVE * self.finger_len_m if depth_m is None else min(float(depth_m), self.finger_len_m)

    def _reach_points(self, frac, depth_m=None):
        """Per jaw side at opening frac: the points of its own shapes that stand where a thing between the jaws
        can be (tool frame): no further up the finger than the thing reaches (_depth). A side with none there at
        this opening cannot touch such a thing yet (a hinged jaw swung wide): it gets no points. Every point
        when the depth is not known."""
        depth, out = self._depth(depth_m), []
        for P in self._side_points(frac):
            out.append(P if depth is None or not len(P) else P[P[:, 0] >= -depth - EPS])
        return out

    def pad_levers(self, depth_m=None):
        """(shortest, longest) lever per moving jaw joint: the distance (m) from a hinge's axis to where its jaw
        first meets a flat thing standing at the other jaw, over every opening from closed to open (at each
        opening: the point of the jaw's own shapes nearest the other jaw along the closing axis, among those a
        thing that reaches depth_m up the fingers can meet); the longest is never under the jaw point's own
        lever. 1 for a slide (its force is the joint's own); a jaw with no collision shape: the jaw point's
        lever both ways."""
        cache = self.__dict__.setdefault("_levers", {})
        key = self._depth(depth_m)
        if key in cache:
            return cache[key]
        d, m = self.d, self.m
        lo, hi = [float(v) for v in self.lever], [float(v) for v in self.lever]
        for k, joint in enumerate(self.joints):
            if not self.hinge[k]:
                continue
            j, seen = m.joint(joint).id, []
            for f in self.fracs:
                P = self._reach_points(f, depth_m)[k]        # leaves the kinematics at opening f
                if not len(P):
                    continue                                 # swung clear of where the thing can be
                R, tip = d.site_xmat[self.tip_site].reshape(3, 3), d.site_xpos[self.tip_site]
                a = P @ self.axis_T
                r = P[int(np.argmin(a) if k == 0 else np.argmax(a))] - (d.xanchor[j] - tip) @ R
                axis = d.xaxis[j] @ R
                seen.append(float(np.linalg.norm(r - (r @ axis) * axis)))
            if seen:
                lo[k], hi[k] = max(min(seen), EPS), max(max(seen), hi[k])
        cache[key] = (np.asarray(lo), np.asarray(hi))
        return cache[key]

    def pad_lever_at(self, frac, depth_m=None):
        """Per moving jaw joint: the lever (m) its squeeze acts through where a thing between the jaws stands NOW,
        at opening frac: the distance from a hinge's axis to the point of its jaw's own shapes nearest the other
        jaw at that opening, among those a thing that reaches depth_m up the fingers can meet (pad_levers reads
        the same point over every opening and keeps the shortest). 1 for a slide; the jaw point's lever for a
        jaw with no shape there."""
        d, m = self.d, self.m
        out, P_all = [float(v) for v in self.lever], None
        for k, joint in enumerate(self.joints):
            if not self.hinge[k]:
                continue
            if P_all is None:
                P_all = self._reach_points(float(frac), depth_m)     # leaves the kinematics at opening frac
            P = P_all[k]
            if not len(P):
                continue
            j = m.joint(joint).id
            R, tip = d.site_xmat[self.tip_site].reshape(3, 3), d.site_xpos[self.tip_site]
            a = P @ self.axis_T
            r = P[int(np.argmin(a) if k == 0 else np.argmax(a))] - (d.xanchor[j] - tip) @ R
            axis = d.xaxis[j] @ R
            out[k] = max(float(np.linalg.norm(r - (r @ axis) * axis)), EPS)
        return np.asarray(out)

    def side_names(self):
        """The body name of each jaw side (the moving jaws first; a one-jaw gripper's fixed side is the body
        the moving jaw hangs from)."""
        return [self.m.body(int(b)).name for b in self.sides]

    def reach_m(self, frac, depth_m=None):
        """The least gap along the closing axis between the two jaw sides' own shapes at opening frac (m), over
        the points a thing that reaches depth_m up the fingers can meet: the width of the widest such thing the
        jaws already touch somewhere. Jaws whose faces stay parallel: the gap at the tips; a hinged jaw: less
        than its tip gap."""
        sides = self._reach_points(frac, depth_m)
        if len(sides) != len(self.sides) or len(sides) < 2:
            return self.width_m(self.q_of_frac(frac))
        if not len(sides[0]) or not len(sides[1]):
            # a side with no shape at all: the gap at the tips; a side swung clear of the thing: no touch yet
            return self.width_m(self.q_of_frac(frac)) if not all(len(P) for P in self._side_points(frac)) \
                else math.inf
        return max(0.0, float((sides[0] @ self.axis_T).min() - (sides[1] @ self.axis_T).max()))

    def frac_for_reach(self, w_m, depth_m=None):
        """The largest opening at which the jaws' own shapes can already touch a thing w_m wide between them."""
        cache = self.__dict__.setdefault("_reach", {})
        key = self._depth(depth_m)
        if key not in cache:
            cache[key] = np.maximum.accumulate(np.array([self.reach_m(f, depth_m) for f in self.fracs]))
        table = cache[key]
        k = int(np.searchsorted(table, max(float(w_m), float(table[0])), side="right")) - 1
        return float(self.fracs[min(max(k, 0), len(self.fracs) - 1)])

    def open_for_reach(self, w_m, depth_m=None):
        """The opening (the gap between the jaw points, m, as frac_for_width reads it) at which the jaws' own
        SHAPES stand at least w_m apart where a thing reaches depth_m up the fingers: w_m itself for jaws that move
        together (their faces stay parallel); wider for a hand with one hinged jaw, whose shapes come nearer the other jaw than its point does
        (a hand opened to a thing's width by its points' gap meets the thing's edge with its shapes on the way
        down). Never over the hand's own full opening."""
        w = float(w_m)
        if not w < float(self.max_m):
            return float(self.max_m)
        if not self.one_jaw:
            return w                                    # jaws that move together keep their faces parallel
        if self.reach_m(self.frac_for_width(w), depth_m) >= w:
            return w                                    # the shapes stand at least that far apart there already
        self.frac_for_reach(w, depth_m)                 # the reach table of that depth, made once
        table = self.__dict__["_reach"][self._depth(depth_m)]
        k = int(np.searchsorted(table, w, side="left"))
        if k >= len(self.fracs):
            return float(self.max_m)
        return float(min(self.max_m, max(w, self.width_m(self.q_of_frac(float(self.fracs[k]))))))

    def fixed_face_T(self):
        """One moving jaw: where the fixed jaw's pad face stands along the closing axis, in the tool frame (m, the
        coordinate along axis_T); None for jaws that move together or a fixed jaw with no collision shape."""
        if not self.one_jaw:
            return None
        got = self._faces(1.0)[1]
        if got is None:
            return None
        pts, sign = got
        return float(sign * np.max(sign * (pts @ self.axis_T)))

    def finger_boxes(self, frac):
        """Per jaw side at opening frac: its hull's box in the tool frame, {"x": (lo, hi) along the pointing axis,
        "a": (lo, hi) along the closing axis, "b": (lo, hi) across}, m; None for a side with no collision shape."""
        out = []
        for P in self._side_points(frac):
            out.append(None if not len(P) else _box(P, self.axis_T, self.cross_T))
        return out

    def palm_box(self):
        """The palm's box in the tool frame (the same keys), or None when the hand has no shape across the gap."""
        R, tip = self._tool_frame(1.0)
        d = self.d
        P = [(pts @ d.xmat[b].reshape(3, 3).T + d.xpos[b] - tip) @ R for b, pts in self._palm]
        return _box(np.vstack(P), self.axis_T, self.cross_T) if P else None

    def low_m(self, frac):
        """How far the lowest point of the jaws' hulls reaches past the tip along the pointing axis (m, >= 0)."""
        xs = [float(P[:, 0].max()) for P in self._side_points(frac) if len(P)]
        return max([0.0] + xs)

    def palm_clear(self, tip, axis, top=None):
        """The palm plane for the tip at `tip` (base frame, m) pointing along the unit vector `axis`: the point of
        the plane behind the tip; with `top` (a point on a surface the palm comes toward): the palm's clearance
        from it along the axis (m, negative = the palm would be past it). None when the palm is not known."""
        if self.finger_len_m is None:
            return None
        axis = np.asarray(axis, dtype=float)
        p = np.asarray(tip, dtype=float) - axis * self.finger_len_m
        if top is None:
            return p
        return float((np.asarray(top, dtype=float) - p) @ axis)

    # ---------------------------------------------------------------- geometry
    def q_of_frac(self, f):
        return self.closed + float(f) * self.span

    def frac_of_q(self, q):
        q = np.asarray(q, dtype=float)
        span = np.where(np.abs(self.span) > EPS, self.span, 1.0)
        return float(np.mean((q - self.closed) / span))

    def _points(self, q_grip):
        d = self.d
        d.qpos[:] = self.q_full
        d.qpos[self.qadr] = np.asarray(q_grip, dtype=float)
        mujoco.mj_kinematics(self.m, d)
        return [d.xpos[b] + d.xmat[b].reshape(3, 3) @ loc for b, loc in zip(self.sides, self.local)]

    def width_m(self, q_grip):
        """Opening at the jaw tips (m); more than two jaws: the mean pair gap."""
        pts = self._points(q_grip)
        gaps = [float(np.linalg.norm(pts[i] - pts[k])) for i in range(len(pts)) for k in range(i + 1, len(pts))]
        return float(np.mean(gaps))

    def points_tool(self, frac):
        """The jaw points at opening fraction frac in the tool frame (tip site axes, origin at the tip)."""
        pts = self._points(self.q_of_frac(frac))
        d = self.d
        R = d.site_xmat[self.tip_site].reshape(3, 3)
        tip = d.site_xpos[self.tip_site]
        return [R.T @ (p - tip) for p in pts]

    def gap_centre_T(self, frac):
        """Where the middle of the gap between the jaws is at opening frac, in the tool frame (m)."""
        pts = self.points_tool(frac)
        return np.mean(pts, axis=0)

    @property
    def one_jaw(self):
        """One moving jaw closing against a fixed one: the object ends against the fixed jaw."""
        return len(self.joints) == 1

    def hold_centre_T(self, frac, obj_w_m=None, clear_m=None):
        """Where the middle of an object of width obj_w_m should stand, in the tool frame, with the jaws open at
        frac: the middle of the gap for jaws that move together, and for one moving jaw when no width is seen
        (the hand then moves with the jaw as it closes, so that the middle stays where it is: centre_follow_T);
        for one moving jaw with a width, half that width plus clear_m (default: what the pick set from its
        point's error) off the fixed jaw's PAD FACE, measured from the hand's own shapes (the fixed jaw's point
        is the tool tip, and the pad is set back from it), so the moving jaw closes and does not shove."""
        if not self.one_jaw or obj_w_m is None:
            return self.gap_centre_T(frac)
        pts = self.points_tool(frac)
        clear = self.clear_m if clear_m is None else float(clear_m)
        face = self.fixed_face_T()
        base = float(pts[1] @ self.axis_T) if face is None else face
        toward = 1.0 if float((pts[0] - pts[1]) @ self.axis_T) >= 0.0 else -1.0
        return pts[1] + self.axis_T * (base - float(pts[1] @ self.axis_T)
                                       + toward * (HALF * float(obj_w_m) + clear))

    def centre_follow_T(self, frac0, frac):
        """How far the tool must move, in its own frame (m), while the jaws go from opening frac0 to frac, for the
        gap's middle to stay where it is in the world: a one jaw hand closing on a thing of no seen width moves
        by this, so both faces arrive together."""
        return self.gap_centre_T(frac0) - self.gap_centre_T(frac)

    def frac_for_width(self, w_m):
        w = min(max(float(w_m), float(self.table[0])), float(self.table[-1]))
        if self.monotone:
            return float(np.interp(w, self.table, self.fracs))
        return float(self.fracs[int(np.argmin(np.abs(self.table - w)))])

    # ---------------------------------------------------------------- readings (a Backend.read()["gripper"] block)
    def reading(self, g, cmd_frac=None):
        """(q, target) of a gripper block; target falls back to the commanded fraction."""
        if not isinstance(g, dict):
            return None, None
        q = g.get("q")
        if q is None and g.get("open_frac") is not None:
            q = self.q_of_frac(g["open_frac"])
        if q is None:
            return None, None
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape[0] != len(self.joints):
            return None, None
        target = g.get("target")
        if target is None and cmd_frac is not None and not isinstance(cmd_frac, str):
            target = self.q_of_frac(cmd_frac)
        if target is not None:
            target = np.asarray(target, dtype=float).reshape(-1)
            if target.shape[0] != len(self.joints):
                target = None
        return q, target

    def lag(self, q, target):
        """Per joint: how far the jaw stands open of its command, in joint units (positive = pressing)."""
        if q is None or target is None:
            return None
        return (q - target) * np.sign(np.where(np.abs(self.span) > EPS, self.span, 1.0))

    def squeeze_n(self, q, target):
        """Estimated squeeze on what the jaws hold (N), or None without a servo gain and force range."""
        lag = self.lag(q, target)
        if lag is None or self.kp is None or self.force_range is None:
            return None
        f = np.minimum(np.maximum(self.kp * lag - self.friction, 0.0), self.force_range)
        return float(np.mean(f / self.lever))

    def squeeze_top_n(self):
        """The largest squeeze the jaw servo's own force range gives at the jaw points (N), or None."""
        if self.force_range is None:
            return None
        return float(np.mean(np.maximum(self.force_range - self.friction, 0.0) / self.lever))

    def lead_for_squeeze(self, squeeze_n):
        """Per joint: how far the command must lead the reading (joint units) to press squeeze_n at the jaw
        points, or None without a servo gain."""
        if self.kp is None:
            return None
        return (float(squeeze_n) * self.lever + self.friction) / np.where(self.kp > EPS, self.kp, 1.0)

    def pressing(self, q, target, level=None):
        """Do the jaws press on something: a squeeze over `level` (N; default the free jaws' own level in
        Jaws.know, else what the model's friction leaves: any squeeze at all), or, with no servo gain, a lag on
        every jaw over the free jaws' own lag (else any lag)."""
        know = self.know or {}
        s = self.squeeze_n(q, target)
        if s is not None:
            return bool(s > float(know.get("press_n", 0.0) if level is None else level))
        lag = self.lag(q, target)
        return lag is not None and bool(np.all(lag > float(know.get("lag", 0.0))))

    def share_for_squeeze(self, squeeze_n, default=None):
        """The share of the jaw servo's force that squeezes about squeeze_n at the jaw points (what it needs at
        the joint over the servo's range, never over the whole range); `default` when the spec gives no range."""
        if self.force_range is None:
            return default
        need = float(squeeze_n) * self.lever + self.friction
        return float(min(1.0, float(np.mean(need / np.maximum(self.force_range, EPS)))))

    def describe(self):
        mm = 1000.0     # unit
        return {"max_opening_mm": round(self.max_m * mm, 1), "jaws": len(self.sides),
                "finger_length_mm": None if self.finger_len_m is None else round(self.finger_len_m * mm, 1),
                "pad_height_mm": None if self.pad_height_m is None else round(self.pad_height_m * mm, 1),
                "moving_jaws": len(self.joints),
                "squeeze_known": self.kp is not None and self.force_range is not None}


def _box(P, a_T, b_T):
    a, b = P @ a_T, P @ b_T
    return {"x": (float(P[:, 0].min()), float(P[:, 0].max())), "a": (float(a.min()), float(a.max())),
            "b": (float(b.min()), float(b.max()))}


def _geom_points(m, d, g):
    """One collision shape as points in the world frame of d: a mesh's own vertices, else the corners of its box."""
    Rg, pg = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
    if int(m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_MESH):
        mid = int(m.geom_dataid[g])
        v = np.asarray(m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]], dtype=float)
    else:
        c, h = np.asarray(m.geom_aabb[g][:3], dtype=float), np.asarray(m.geom_aabb[g][3:], dtype=float)
        v = np.array([c + h * np.array([sx, sy, sz]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return v @ Rg.T + pg


# ---------------------------------------------------------------- plane geometry (the hand over a spot)
def rect(centre, half_a, half_b, bearing):
    """Corners (4 x 2) of a rectangle: half_a along the bearing, half_b across."""
    u = np.array([math.cos(bearing), math.sin(bearing)])
    w = np.array([-u[1], u[0]])
    c = np.asarray(centre, dtype=float)
    return np.array([c + sa * half_a * u + sb * half_b * w for sa, sb in ((1, 1), (1, -1), (-1, -1), (-1, 1))])


def _seg_point(p, a, b):
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(float(ab @ ab), EPS), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def poly_gap(A, B):
    """Distance between two convex polygons (corner arrays); 0 when they overlap."""
    for P, Q in ((A, B), (B, A)):
        n = len(P)
        for i in range(n):
            e = P[(i + 1) % n] - P[i]
            ax = np.array([-e[1], e[0]])
            if float(np.min(Q @ ax)) > float(np.max(P @ ax)) or float(np.max(Q @ ax)) < float(np.min(P @ ax)):
                break
        else:
            continue
        break
    else:
        return 0.0
    best = math.inf
    for P, Q in ((A, B), (B, A)):
        n = len(Q)
        for p in P:
            for i in range(n):
                best = min(best, _seg_point(p, Q[i], Q[(i + 1) % n]))
    return best


def hand_rects(J, centre, yaw, open_m, obj_w=None):
    """The fingers' and the palm's boxes on the support plane for the jaws open open_m with the thing's middle at
    centre and the closing axis along yaw: [(body word, corners, lowest reach past the tip m or None)]."""
    frac = J.frac_for_width(open_m)
    c_T = J.hold_centre_T(frac, obj_w)
    ca, cb = float(c_T @ J.axis_T), float(c_T @ J.cross_T)
    u = np.array([math.cos(yaw), math.sin(yaw)])
    w = np.array([-u[1], u[0]])
    out = []
    boxes = [("finger", b) for b in J.finger_boxes(frac)] + [("palm", J.palm_box())]
    for word, b in boxes:
        if b is None:
            continue
        a0, a1 = b["a"][0] - ca, b["a"][1] - ca
        hb = max(abs(b["b"][0] - cb), abs(b["b"][1] - cb))
        mid = np.asarray(centre, dtype=float) + HALF * (a0 + a1) * u
        out.append((word, rect(mid, HALF * (a1 - a0), hb, yaw), b["x"]))
    return out
