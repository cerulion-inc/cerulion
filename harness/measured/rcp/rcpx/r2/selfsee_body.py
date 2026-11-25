# SPDX-License-Identifier: AGPL-3.0-only
"""The robot's own body in the picture and in space (package selfsee).

Everything here comes from the body model (RobotModel.mj at a joint pose: ray casts against the model's own visible
geometry, its collision capsules, its kinematic tree) and from robot://body's standing surface (Executor.
standing_surface(): floor_z_mm, mount_top_z_mm, mount_half_side_mm). Nothing reads the picture, the simulator or a
device; no robot is named.

Groups, by the kinematic tree, never by name (an "arm joint" is a joint of robot://body joint_order):
    own_base   bodies with no arm joint between them and the world (the base and what is fastened to it)
    own_hand   bodies with no arm joint between them and the camera's body, and the bodies the last arm joint
               carries (the hand and its tool: gripper jaws ride their own joints but stay with the hand)
    own_arm    every other body that moves with the arm
Region B (the robot's own mount, 4.1 a): |x| and |y| up to mount_half_side_mm + MOUNT_MARGIN_MM and z up to
mount_top_z_mm + MOUNT_TOP_MARGIN_MM (no lower bound: a point marked on the mount can triangulate a little under
the floor). With mount_half_side_mm null, B is every geom of the own_base bodies, each box grown by MOUNT_MARGIN_MM.

    Body(ex)                         per executor; reads the model and the standing surface lazily
    body.first_hit(q, u, v)          {what, dist_mm, floor_mm, source}: what is own_base | own_arm | own_hand |
                                     world | floor | none, the first thing the pixel's ray meets
    body.first_hits(q, pixels)       the same for many pixels at once (one multi-ray cast)
    body.grid(q, step_px)            (us, vs, what codes) over the whole frame
    body.mask(q, step_px=16)         [(group, box_px, share)], largest share first
    body.on_body(p_mm, qs, contacts) None or a group name; body.why_on_body(...) gives (group, how) as well
"""

import math
import inspect
from rcpx import bounds

import mujoco
import numpy as np

from rcpx import r3_limits as L

MM = 1000.0
GROUPS = ("own_base", "own_arm", "own_hand")
CODES = {"none": -1, "floor": 0, "world": 1, "own_base": 2, "own_arm": 3, "own_hand": 4}
NAMES = {v: k for k, v in CODES.items()}
OWN = frozenset(("own_base", "own_arm", "own_hand"))
VISIBLE_GROUPS = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)   # the geom groups a render shows by default
RAY_CUTOFF_M = 10.0          # a robot geom farther along a ray than this cannot be the robot's own (the arm's reach
#                              is under 2 m on every arm a wrist camera rides)
INSIDE_SLACK_MM = 1e-6       # a point this close to a region's face counts as inside
SOURCES = ("none", "model", "region_b", "world", "floor")


def _unit(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def _seg_dist(p, a, b):
    ab = b - a
    t = 0.0 if float(ab @ ab) < 1e-18 else float(np.clip((p - a) @ ab / float(ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _nearer(t, code_new, source_new, code, dist, source):
    """Where t is nearer than the present hit, the new hit wins."""
    better = t < dist
    return (np.where(better, code_new, code), np.where(better, t, dist),
            np.where(better, SOURCES.index(source_new), source))


def slab(o, d, lo, hi):
    """Entry distance (o and lo/hi in the same unit) of rays o + t d (d: N x 3) into the axis-aligned box [lo, hi]; inf where a
    ray misses or that box lies behind. A ray that starts inside enters at t = 0."""
    d = np.atleast_2d(np.asarray(d, dtype=float))
    o = np.asarray(o, dtype=float).reshape(1, 3)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / d
        t1 = (np.asarray(lo, dtype=float).reshape(1, 3) - o) * inv
        t2 = (np.asarray(hi, dtype=float).reshape(1, 3) - o) * inv
    tmin = np.where(np.isnan(t1), -np.inf, np.minimum(t1, t2))
    tmax = np.where(np.isnan(t2), np.inf, np.maximum(t1, t2))
    # an axis the ray runs parallel to: inside the slab it never limits, outside it rules the ray out
    par = np.abs(d) < 1e-12
    inside = (o >= np.asarray(lo).reshape(1, 3) - INSIDE_SLACK_MM) & (o <= np.asarray(hi).reshape(1, 3)
                                                                       + INSIDE_SLACK_MM)
    tmin = np.where(par, np.where(inside, -np.inf, np.inf), tmin)
    tmax = np.where(par, np.where(inside, np.inf, -np.inf), tmax)
    t_in = np.max(tmin, axis=1)
    t_out = np.min(tmax, axis=1)
    hit = (t_out >= np.maximum(t_in, 0.0))
    return np.where(hit, np.maximum(t_in, 0.0), np.inf)


class Body:
    """The robot's own body as the camera and the space around it see it (module text)."""

    def __init__(self, ex):
        self.ex = ex
        self.model = ex.model
        m = self.model.mj
        self._d = mujoco.MjData(m)
        names = list(self.model.joint_names)
        jbody = [int(m.jnt_bodyid[m.joint(n).id]) for n in names]
        self.n = len(names)
        key = np.full(m.nbody, -1, dtype=int)
        for b in range(1, m.nbody):
            c = b
            while c != 0:
                if c in jbody:
                    key[b] = max(i for i, jb in enumerate(jbody) if jb == c)
                    break
                c = int(m.body_parentid[c])
        self.arm_key = key
        cam_body = int(m.cam_bodyid[m.camera("wrist").id])
        tip_name = (self.model.spec.get("tip") or {}).get("body")
        tip_body = m.body(tip_name).id if tip_name else -1
        grp = np.full(m.nbody, -1, dtype=int)
        for b in range(1, m.nbody):
            if key[b] < 0:
                grp[b] = CODES["own_base"]
            elif key[b] == key[cam_body] or key[b] == self.n - 1:
                grp[b] = CODES["own_hand"]
            else:
                grp[b] = CODES["own_arm"]
        self.body_group = grp
        self.tool = set()
        for b in range(1, m.nbody):
            c = b
            while c > 0:
                if c == tip_body:
                    self.tool.add(m.body(b).name)
                    break
                c = int(m.body_parentid[c])
        self._adr = np.array([m.jnt_qposadr[m.joint(n).id] for n in names], dtype=int)
        self.geom_code = np.array([int(grp[m.geom_bodyid[g]]) if m.geom_bodyid[g] > 0 else CODES["none"]
                                   for g in range(m.ngeom)], dtype=int)
        vis = [g for g in range(m.ngeom) if int(m.geom_group[g]) <= 2]
        self.groups = VISIBLE_GROUPS if vis else None   # a model with no visible group: every geom counts
        self._fallback_boxes = None
        self._contain_pose = None
        self._contain_geoms = set()

    # ------------------------------------------------------------------ the standing surface and region B
    def surface(self):
        """(floor_z_mm, mount_top_z_mm, mount_half_side_mm) as robot://body serves them."""
        fn = getattr(self.ex, "standing_surface", None)
        s = fn() if callable(fn) else {}
        floor = s.get("floor_z_mm")
        top = s.get("mount_top_z_mm")
        return (0.0 if floor is None else float(floor), 0.0 if top is None else float(top),
                None if s.get("mount_half_side_mm") is None else float(s["mount_half_side_mm"]))

    def region_b(self):
        """[(lo_mm, hi_mm)]: the boxes of region B (module text)."""
        floor, top, h = self.surface()
        if h is not None:
            a = h + L.MOUNT_MARGIN_MM
            return [(np.array([-a, -a, -1e9]), np.array([a, a, top + L.MOUNT_TOP_MARGIN_MM]))]
        if self._fallback_boxes is None:
            self._fallback_boxes = self._base_geom_boxes()
        return self._fallback_boxes

    def _base_geom_boxes(self):
        """The own_base geoms' boxes (frame B, mm), each grown by MOUNT_MARGIN_MM: region B without a mount size.
        The base does not move with the arm, so any pose gives them; the home pose is used."""
        m, d = self.model.mj, self._d
        self._set(self.model.q_home)
        out = []
        for g in range(m.ngeom):
            if self.geom_code[g] != CODES["own_base"]:
                continue
            half = np.asarray(m.geom_aabb[g][3:], dtype=float)
            c = np.asarray(m.geom_aabb[g][:3], dtype=float)
            R = d.geom_xmat[g].reshape(3, 3)
            centre = d.geom_xpos[g] + R @ c
            ext = np.abs(R) @ half
            grow = L.MOUNT_MARGIN_MM / MM
            out.append(((centre - ext - grow) * MM, (centre + ext + grow) * MM))
        return out

    def in_b(self, p_mm):
        p = np.asarray(p_mm, dtype=float).reshape(3)
        return any(bool(np.all(p >= lo - INSIDE_SLACK_MM) and np.all(p <= hi + INSIDE_SLACK_MM))
                   for lo, hi in self.region_b())

    # ------------------------------------------------------------------ rays
    def _set(self, q):
        m, d = self.model.mj, self._d
        full = getattr(self.model, "_qfull", None)
        d.qpos[:] = m.qpos0 if full is None else full
        d.qpos[self._adr] = np.asarray(q, dtype=float)
        mujoco.mj_kinematics(m, d)
        mujoco.mj_camlight(m, d)
        return m, d

    def _camera(self, q):
        m, d = self._set(q)
        cid = m.camera("wrist").id
        return d.cam_xpos[cid].copy(), d.cam_xmat[cid].reshape(3, 3).copy()

    def rays(self, q, us, vs):
        """(origin m, unit directions N x 3) of the pixels (us, vs) at q (the pinhole RobotModel.pixel_ray uses)."""
        pc, Rc = self._camera(q)
        W, H, f = float(self.model.cam_w), float(self.model.cam_h), float(self.model.f_px)
        us, vs = np.asarray(us, dtype=float).reshape(-1), np.asarray(vs, dtype=float).reshape(-1)
        dc = np.stack([(us - W / 2.0) / f, -(vs - H / 2.0) / f, -np.ones_like(us)], -1)
        dw = dc @ Rc.T
        return pc, dw / np.linalg.norm(dw, axis=1, keepdims=True)

    def _robot_hits(self, pc, dw):
        """(geom id or -1, distance m or inf) of every ray against the robot model's own visible geometry. The
        kinematics must be set at the rays' pose (rays() does)."""
        m, d = self.model.mj, self._d
        n = len(dw)
        gid = np.full(n, -1, dtype=np.int32)
        dist = np.full(n, -1.0)
        mujoco.mj_multiRay(m, d, np.asarray(pc, dtype=float), np.ascontiguousarray(dw.reshape(-1)), self.groups, 1,
                           -1, gid, dist, None, n, RAY_CUTOFF_M)
        containing = self._containing(pc, gid)
        if containing:
            for k in np.flatnonzero(np.isin(gid, list(containing))):
                inside = set(containing)
                travelled = 0.0
                while int(gid[k]) in inside and dist[k] >= 0.0:
                    inside.remove(int(gid[k]))
                    travelled += float(dist[k]) + 1e-6
                    hit = np.full(1, -1, dtype=np.int32)
                    dist[k] = mujoco.mj_ray(m, d, pc + travelled * dw[k], dw[k], self.groups, 1, -1, hit)
                    gid[k] = hit[0]
                if dist[k] >= 0.0:
                    dist[k] += travelled
                if dist[k] > RAY_CUTOFF_M:
                    gid[k] = -1
        keep = (gid >= 0) & (dist >= 0.0)
        codes = np.where(keep, self.geom_code[np.clip(gid, 0, None)], CODES["none"])
        own = codes >= CODES["own_base"]
        return np.where(own, codes, CODES["none"]), np.where(own, dist, np.inf)

    def _geom_crossings(self, g, pc, direction):
        """Count crossings of one geom, stepping past each face by the ray numerical tolerance (1 um).
        The model's own geometry determines where the march ends (its face count: see below)."""
        m, d = self.model.mj, self._d
        p = np.asarray(pc, dtype=float).copy()
        count = 0
        mesh = int(m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_MESH)
        # a straight line crosses each face of a mesh at most once, and a primitive (convex) at most twice: the
        # march ends there at the latest (the bound is the geometry's own, so a grazing ray cannot run on)
        most = int(m.mesh_facenum[int(m.geom_dataid[g])]) if mesh else 2
        while count < most:
            if mesh:
                t = mujoco.mj_rayMesh(m, d, g, p, direction)
            else:
                t = mujoco.mju_rayGeom(d.geom_xpos[g], d.geom_xmat[g], m.geom_size[g], p, direction,
                                       int(m.geom_type[g]))
            if t < 0.0:
                return count
            count += 1
            p += (float(t) + 1e-6) * direction
        return count

    def _containing(self, pc, first=None):
        """Own visible geoms containing the optical centre, cached at the full model pose for this cast.
        A bounding box rejects impossible candidates without another ray cast in the common case. Candidates
        met by the picture rays or the six camera axes need odd crossing parity on at least five axes."""
        m, d = self.model.mj, self._d
        pose = (d.qpos.tobytes(), np.asarray(pc, dtype=float).tobytes())
        if pose == self._contain_pose:
            return self._contain_geoms
        possible = set()
        for g in np.flatnonzero(self.geom_code >= CODES["own_base"]):
            if self.groups is not None and not self.groups[int(m.geom_group[g])]:
                continue
            local = d.geom_xmat[g].reshape(3, 3).T @ (pc - d.geom_xpos[g]) - m.geom_aabb[g, :3]
            if np.all(np.abs(local) <= m.geom_aabb[g, 3:]):
                possible.add(int(g))
        containing = set()
        if possible:
            Rc = d.cam_xmat[m.camera("wrist").id].reshape(3, 3)
            axes = np.ascontiguousarray(np.concatenate((Rc.T, -Rc.T)))
            hits = np.full(6, -1, dtype=np.int32)
            distances = np.full(6, -1.0)
            mujoco.mj_multiRay(m, d, pc, axes.reshape(-1), self.groups, 1, -1, hits, distances, None, 6,
                              RAY_CUTOFF_M)
            candidates = possible.intersection(set(map(int, hits)) | set(map(int, () if first is None else first)))
            for g in candidates:
                if sum(self._geom_crossings(g, pc, axis) % 2 for axis in axes) >= 5:
                    containing.add(g)
        self._contain_pose, self._contain_geoms = pose, containing
        return containing

    def _world_boxes(self):
        """[(lo_mm, hi_mm)] of the known things the World holds as boxes (kind box: centre_mm, size_mm, top_mm),
        standing on the floor; none until a package stores them."""
        floor, _, _ = self.surface()
        out = []
        world = getattr(self.ex, "world", None)
        for e in (world.query(kind="box", include_stale=False) if world is not None else []):
            data = e.get("data") or {}
            c, s, top = data.get("centre_mm"), data.get("size_mm"), data.get("top_mm")
            try:
                c = [float(c[0]), float(c[1])]
                s = [abs(float(s[0])), abs(float(s[1]))]
                top = float(top)
            except (TypeError, ValueError, IndexError):
                continue
            out.append((np.array([c[0] - s[0] / 2, c[1] - s[1] / 2, floor]),
                        np.array([c[0] + s[0] / 2, c[1] + s[1] / 2, top])))
        return out

    def cast(self, q, us, vs):
        """Per pixel: (code, dist_mm, floor point mm N x 3 with NaN where the ray never meets the floor, source
        code) with source one of SOURCES: model, region_b, world, floor or none (module text)."""
        pc, dw = self.rays(q, us, vs)
        code, dist = self._robot_hits(pc, dw)
        dist = dist * MM
        source = np.where(code >= CODES["own_base"], SOURCES.index("model"), SOURCES.index("none"))
        o = pc * MM
        floor, top, _ = self.surface()
        for lo, hi in self.region_b():
            if bool(np.all(o >= lo) and np.all(o <= hi)):
                # the camera sits inside B's margin above the mount: B there is the mount itself, not its margin
                hi = np.array([hi[0], hi[1], min(float(hi[2]), top)])
            t = slab(o, dw, lo, hi)
            code, dist, source = _nearer(t, CODES["own_base"], "region_b", code, dist, source)
        for lo, hi in self._world_boxes():
            code, dist, source = _nearer(slab(o, dw, lo, hi), CODES["world"], "world", code, dist, source)
        with np.errstate(divide="ignore", invalid="ignore"):
            tf = np.where(dw[:, 2] < -1e-9, (floor - o[2]) / dw[:, 2], np.inf)
        tf = np.where(tf > 0.0, tf, np.inf)
        fp = np.where(np.isfinite(tf)[:, None], o.reshape(1, 3) + np.where(np.isfinite(tf), tf, 0.0)[:, None] * dw,
                      np.nan)
        code, dist, source = _nearer(tf, CODES["floor"], "floor", code, dist, source)
        return code, dist, fp, source

    def first_hit(self, q, u, v):
        """{what, dist_mm, floor_mm, source}: the first thing the ray through pixel (u, v) meets at joint pose q."""
        return self.first_hits(q, [(u, v)])[0]

    def first_hits(self, q, pixels):
        pix = np.asarray(pixels, dtype=float).reshape(-1, 2)
        code, dist, fp, source = self.cast(q, pix[:, 0], pix[:, 1])
        out = []
        for k in range(len(pix)):
            f = fp[k]
            out.append({"what": NAMES[int(code[k])],
                        "dist_mm": None if not math.isfinite(float(dist[k])) else round(float(dist[k]), 1),
                        "floor_mm": None if not np.all(np.isfinite(f)) else [round(float(x), 1) for x in f],
                        "source": SOURCES[int(source[k])]})
        return out

    def grid(self, q, step_px=L.SELF_GRID_PX):
        """(us, vs, codes): the first thing met at the centre of every step_px cell of the frame."""
        W, H = int(self.model.cam_w), int(self.model.cam_h)
        us, vs = np.meshgrid(np.arange(step_px / 2.0, W, step_px), np.arange(step_px / 2.0, H, step_px))
        code, _, _, _ = self.cast(q, us.reshape(-1), vs.reshape(-1))
        return us, vs, code.reshape(us.shape)

    def mask(self, q, step_px=L.SELF_GRID_PX):
        """[(group, box_px [u0, v0, u1, v1], share)] of the robot's own body in the frame at q, largest first."""
        us, vs, codes = self.grid(q, step_px)
        return mask_from_grid(us, vs, codes, step_px, int(self.model.cam_w), int(self.model.cam_h))[0]

    # ------------------------------------------------------------------ points in space
    def why_on_body(self, p_mm, qs, contacts=()):
        """(group or None, how): how is "mount" (inside region B), "inside" (inside a non-tool capsule grown
        CAPSULE_GROW_MM at one of the poses qs) or "beside" (within CAPSULE_NEAR_MM of one and not within
        CONTACT_KEEP_MM of a stored contact). The three limits are the session's (r3_limits' lower case functions
        once they exist). Where the derived grow reaches as far as the near band (the capsule's own fit error is
        both), the grown band IS the beside band: a point there is the robot's own unless a stored contact lies
        beside it, and only a point inside the capsule itself is sure. The tool's own capsules never count
        (4.1 e)."""
        p = np.asarray(p_mm, dtype=float).reshape(3)
        if self.in_b(p):
            return "own_base", "mount"
        near = None
        m = self.model.mj
        for q in qs or ():
            for name, p0, p1, r in self.model.collision_capsules(np.asarray(q, dtype=float)):
                if name in self.tool:
                    continue
                gap = _seg_dist(p / MM, np.asarray(p0), np.asarray(p1)) * MM - float(r) * MM
                group = NAMES[int(self.body_group[m.body(name).id])]
                grow, beside = _limit(self.ex, "CAPSULE_GROW_MM", name), _limit(self.ex, "CAPSULE_NEAR_MM", None, name)
                if gap <= 0.0 or (gap <= grow and grow < beside):
                    return group, "inside"
                if gap <= max(grow, beside) and (near is None or gap < near[0]):
                    near = (gap, group)
        if near is None:
            return None, None
        for c in contacts or ():
            cp = c.get("point_mm") if isinstance(c, dict) else c
            keep = _limit(self.ex, "CONTACT_KEEP_MM", c.get("error_mm") if isinstance(c, dict) else None)
            if cp is not None and float(np.linalg.norm(np.asarray(cp, dtype=float) - p)) <= keep:
                return None, "beside a stored contact"
        return near[1], "beside"

    def on_body(self, p_mm, qs, contacts=()):
        return self.why_on_body(p_mm, qs, contacts)[0]


def mask_from_grid(us, vs, codes, step_px, W, H):
    """([(group, box_px, share)], {group: centroid (u, v)}) from a grid of codes (see Body.grid), largest share
    first, ties in GROUPS order."""
    total = float(codes.size)
    out = []
    half = step_px / 2.0
    for group in GROUPS:
        sel = codes == CODES[group]
        k = int(np.count_nonzero(sel))
        if k == 0:
            continue
        u, v = us[sel], vs[sel]
        box = [int(max(0, math.floor(float(u.min()) - half))), int(max(0, math.floor(float(v.min()) - half))),
               int(min(W, math.ceil(float(u.max()) + half))), int(min(H, math.ceil(float(v.max()) + half)))]
        out.append((group, box, round(k / total, 3), (float(u.mean()), float(v.mean()))))
    out.sort(key=lambda g: (-g[2], GROUPS.index(g[0])))
    return [(g, b, s) for g, b, s, _ in out], {g: c for g, _, _, c in out}


__all__ = ["Body", "CODES", "GROUPS", "NAMES", "OWN", "SOURCES", "mask_from_grid", "slab"]


def _limit(ex, name, *args):
    """The round three limit NAME (mm) as the session derives it: r3_limits' lower case function of that name with
    the session's bounds and args once it exists (capsule_grow_mm(B, body), capsule_near_mm(B, thing, body),
    contact_keep_mm(B, sigma_mm)) and an executor is at hand; else the constant (a pure use in a test)."""
    fn = getattr(L, name.lower(), None)
    if ex is not None and callable(fn) and next(iter(inspect.signature(fn).parameters), None) in ("B", "bounds"):
        return float(fn(bounds.of(ex), *args))
    return float(getattr(L, name))
