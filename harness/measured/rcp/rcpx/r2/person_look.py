# SPDX-License-Identifier: AGPL-3.0-only
"""I22 person package, part two: geometry and looks for the two constraint behaviours.

Jaws. The region "between the jaws" is the convex hull of every jaw body's geometry (the bodies the spec's gripper
joints move, with their children) sampled at openings from the present one to the commanded one, at the present arm
pose, from the loaded body model. A closing sweeps that region, so a part inside it would be gripped or hit.

Object region. A no_move object's located point is projected into two standard looks (the camera pose of each comes
from the body model at the joint pose stored with the look). Checks that cannot be made say why: a dark look, the
point outside the frame, the robot's own body in front of it (a ray cast on the robot's model).
1. Camera motion over the table: a homography fitted to ORB matches over the whole frame (the table dominates them),
   then the planar motion of the table in frame B (a turn about z and a shift) that reproduces it through the body
   model; the object's own 3D point carried by that motion is where it would appear had it stayed. This keeps the
   parallax of a tall object out of the estimate when the base itself moved.
2. The object's face: the colour region the located pixel sits in (a flood fill), minus the robot's own pixels (the
   same in both looks from one pose), warped by the homography so it is compared the same way up, is searched for
   in the later look by masked squared difference. In place within 2 grey levels of the best match: it stayed.
   A region that runs on past 60 mm has no edge to find it by: the object's motion is then not measured at all
   (checked false, with why), since patches sized by that clamp match the still surroundings, not the object.
3. Too small a face (under 60 px), or a face whose best match is worse than 18 grey levels (a still shadow of the
   arm now falls across it elsewhere, or it moved far or turned): patches 1.2, 1.5 and 1.9 times the face's size are
   matched by normalised colour cross-correlation instead, each voting still (in-place score 0.9 or better and
   within 0.03 of its best peak) or moved (peak 0.75 or better); the median vote counts only when two patch sizes
   agree. After a face that was not found, only strong peaks count and the patches must place the object away from
   where it was (a patch that says "still" there matches the still table or shadow, not the object); otherwise
   the object is "suspect" (moved far, turned over or hidden) and its distance is not guessed. A small face (9 px
   or more) still vetoes a "still" from the patches when its own pixels are no longer in place (a busy table can
   dominate a patch around a small object): it is then placed where its face fits again, or is suspect.
The found pixel and the expected one are taken back to the object's height through each pixel's ray: the difference
is the object's own motion over the table. perception.relocalize on a 50 mm crop around the point is reported as a
second opinion only: on plain objects it found too few features or inliers in every moved case tried.
Nothing here reads simulator ground truth.
"""

import math

import cv2
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull, QhullError

from rcpx import perception

MM = 1000.0
JAW_STEPS = 6              # openings sampled between the present and the commanded opening
MESH_POINTS = 400          # at most this many vertices per mesh geom (every k-th, deterministic)
PATCH_GROW = (1.2, 1.5, 1.9)   # patch half sizes: the object's half extent times these, plus PATCH_PAD_PX
PATCH_PAD_PX = 3
EXTENT_MM = (8.0, 60.0)    # the object's half extent is taken within these bounds
FILL_DIFF = 14             # grey levels per channel a pixel may differ from the located pixel and be the object
SEARCH_MM = 90.0           # how far from the expected place the patch is searched for
MIN_PATCH_STD = 4.0        # a patch flatter than this (grey levels) cannot be found again
MIN_PEAK = 0.75            # a correlation peak under this is not a find on its own
WEAK_PEAK = 0.55           # ... but two patch sizes at least this good that agree within AGREE_MM are one
AGREE_MM = 3.0
ROBOT_GRID_PX = 8          # the robot's own pixels are found on this grid
MAX_FIT_PX = 4.0           # a table motion fit worse than this (median px) is not used; the homography is
STILL_MARGIN = 0.03        # an in-place score this close to the patch's best peak is no evidence of motion ...
STILL_PEAK = 0.9           # ... when that in-place score is itself this good
MIN_REGION_PX = 60         # an object face smaller than this in the look is matched by patches instead
MIN_SMALL_PX = 9           # ... but a face of at least this many pixels still vetoes a "still" the patches give
MATCH_RMS = 18.0           # grey levels: a region that matches no better than this was not found again
STILL_RMS = 2.0            # grey levels: an in-place match this close to the best one means the object stayed
LOST_MIN_MM = 0.5          # after a face that was not found again, patches must place it at least this far away


# ------------------------------------------------------------------------------------------------ jaws
class Jaws:
    """The gripper's jaw bodies on a private copy of the body model's data (the model's own data is untouched)."""

    def __init__(self, model):
        g = model.spec.get("gripper")
        m = model.mj
        self.model, self.m, self.d = model, m, mujoco.MjData(m)
        self.ok = bool(g)
        if not g:
            return
        jids = [m.joint(j).id for j in g["joints"]]
        self.gadr = np.array([m.jnt_qposadr[j] for j in jids])
        roots = {int(m.jnt_bodyid[j]) for j in jids}
        bodies = set()
        for b in range(1, m.nbody):
            a = b
            while a > 0:
                if a in roots:
                    bodies.add(b)
                    break
                a = int(m.body_parentid[a])
        self.open = np.asarray(g["open"], dtype=float)
        self.closed = np.asarray(g["closed"], dtype=float)
        self.local = []
        for gid in range(m.ngeom):
            if int(m.geom_bodyid[gid]) not in bodies:
                continue
            if m.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH:
                mid = m.geom_dataid[gid]
                v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
                v = v[:: max(1, len(v) // MESH_POINTS)]
            else:
                s, t = m.geom_size[gid], m.geom_type[gid]
                if t == mujoco.mjtGeom.mjGEOM_BOX:
                    h = s[:3]
                elif t in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
                    h = [s[0], s[0], s[1] + (s[0] if t == mujoco.mjtGeom.mjGEOM_CAPSULE else 0.0)]
                else:
                    h = [m.geom_rbound[gid]] * 3
                v = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1)
                              for sz in (-1, 1)])
            self.local.append((gid, np.asarray(v, dtype=float)))
        self.qfull = np.asarray(getattr(model, "_qfull", m.qpos0), dtype=float).copy()

    def frac_to_q(self, frac):
        return self.closed + float(frac) * (self.open - self.closed)

    def present_q(self, grip):
        """Gripper joint values from a backend reading's gripper entry (fully open when unknown)."""
        if isinstance(grip, dict):
            q = grip.get("q")
            try:
                a = np.asarray(q, dtype=float).reshape(-1)
                if a.size == len(self.open) and np.all(np.isfinite(a)):
                    return a
            except (TypeError, ValueError):
                pass
            f = grip.get("open_frac")
            if isinstance(f, (int, float)) and math.isfinite(f):
                return self.frac_to_q(min(1.0, max(0.0, f)))
        return self.open.copy()

    def opening(self, gq):
        span = np.where(np.abs(self.open - self.closed) > 1e-9, self.open - self.closed, 1.0)
        return float(np.mean((np.asarray(gq, dtype=float) - self.closed) / span))

    def points(self, q_arm, gq):
        d = self.d
        d.qpos[:] = self.qfull
        d.qpos[self.model._qadr] = np.asarray(q_arm, dtype=float)
        d.qpos[self.gadr] = np.asarray(gq, dtype=float)
        mujoco.mj_kinematics(self.m, d)
        out = [v @ d.geom_xmat[gid].reshape(3, 3).T + d.geom_xpos[gid] for gid, v in self.local]
        return np.vstack(out) if out else np.zeros((0, 3))

    def sweep(self, q_arm, g_from, g_to):
        """(hull equations or None, points m) of the region the jaws sweep from g_from to g_to."""
        pts = np.vstack([self.points(q_arm, g_from + (g_to - g_from) * k / (JAW_STEPS - 1))
                         for k in range(JAW_STEPS)])
        try:
            return ConvexHull(pts, qhull_options="QJ").equations, pts
        except (QhullError, ValueError):
            return None, pts


def inside(eqs, pts, p_m, margin_m):
    """True when p is inside the hull (or, without one, the bounding box of the points) grown by margin."""
    p = np.asarray(p_m, dtype=float)
    if eqs is not None:
        return bool(np.max(eqs[:, :3] @ p + eqs[:, 3]) <= margin_m)
    if len(pts) == 0:
        return False
    return bool(np.all(p >= pts.min(axis=0) - margin_m) and np.all(p <= pts.max(axis=0) + margin_m))


# ------------------------------------------------------------------------------------------------ object region
def homography(ref, cur, residual=False):
    """(3x3 ref -> cur or None, inliers) from ORB matches over the whole frame."""
    g0, g1 = perception._prep(ref), perception._prep(cur)
    orb = cv2.ORB_create(nfeatures=perception.ORB_FEATURES, fastThreshold=10)
    k0, d0 = orb.detectAndCompute(g0, None)
    k1, d1 = orb.detectAndCompute(g1, None)
    if d0 is None or d1 is None or len(k0) < perception.MIN_MATCHES or len(k1) < perception.MIN_MATCHES:
        return None, 0
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d0, d1, k=2)
    good = [p[0] for p in pairs if len(p) == 2 and p[0].distance < perception.RATIO * p[1].distance]
    if len(good) < perception.MIN_MATCHES:
        return None, 0
    src = np.float32([k0[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([k1[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    cv2.setRNGSeed(0)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, perception.RANSAC_PX, maxIters=4000, confidence=0.999)
    if H is None or mask is None or int(mask.sum()) < perception.MIN_INLIERS:
        return None, 0
    if residual:
        use = mask.ravel().astype(bool)
        predicted = cv2.perspectiveTransform(src[use], H)
        error = float(np.sqrt(np.mean(np.sum((predicted - dst[use])**2, axis=-1))))
        return H, max(0.5, error)  # numeric: half a pixel of image coordinate rounding
    return H, int(mask.sum())


def _apply(H, u, v):
    x = H @ np.array([u, v, 1.0])
    return float(x[0] / x[2]), float(x[1] / x[2])


def _ray_plane(model, q, u, v, z):
    o, d = model.pixel_ray(q, u, v)
    if abs(float(d[2])) < 1e-3:
        return None
    t = (z - float(o[2])) / float(d[2])
    return None if t <= 0 else o + t * d


def extent_px(img, u, v, lo_px, hi_px):
    """Half extent (px) of the colour region the located pixel sits in (a flood fill of the colour within
    FILL_DIFF of it, blurred first), clamped to [lo_px, hi_px]: the object's own size in the look."""
    a = cv2.GaussianBlur(np.ascontiguousarray(img[:, :, :3]), (3, 3), 0)
    H, W = a.shape[:2]
    mask = np.zeros((H + 2, W + 2), np.uint8)
    flags = 4 | cv2.FLOODFILL_FIXED_RANGE | cv2.FLOODFILL_MASK_ONLY | (255 << 8)
    cv2.floodFill(a, mask, (int(round(u)), int(round(v))), (0, 0, 0), (FILL_DIFF,) * 3, (FILL_DIFF,) * 3, flags)
    ys, xs = np.nonzero(mask[1:-1, 1:-1])
    if len(xs) == 0:
        return float(lo_px)
    return float(min(hi_px, max(lo_px, max(np.abs(xs - u).max(), np.abs(ys - v).max()))))


def region_mask(img, u, v, max_px):
    """The colour region the located pixel sits in (flood fill within FILL_DIFF of it on a lightly blurred copy),
    grown by one pixel: the visible face of the object there. None when the region runs on past max_px from the
    pixel (a face larger than any object here, or a plain look): it has no edge to find it by."""
    a = cv2.GaussianBlur(np.ascontiguousarray(img[:, :, :3]), (3, 3), 0)
    H, W = a.shape[:2]
    mask = np.zeros((H + 2, W + 2), np.uint8)
    flags = 4 | cv2.FLOODFILL_FIXED_RANGE | cv2.FLOODFILL_MASK_ONLY | (255 << 8)
    cv2.floodFill(a, mask, (int(round(u)), int(round(v))), (0, 0, 0), (FILL_DIFF,) * 3, (FILL_DIFF,) * 3, flags)
    m = mask[1:-1, 1:-1].copy()
    r = int(round(max_px))
    u0, v0 = int(round(u)), int(round(v))
    total = int(np.count_nonzero(m))
    m[:max(0, v0 - r)] = 0
    m[v0 + r + 1:] = 0
    m[:, :max(0, u0 - r)] = 0
    m[:, u0 + r + 1:] = 0
    if int(np.count_nonzero(m)) < total:
        return None                                       # the colour runs on past any object's size: no face
    return cv2.dilate(m, np.ones((3, 3), np.uint8)) > 0


def shape_find(ref, cur, region, u, v, ue, ve, search_px):
    """Where the region's pixels of ref reappear in cur, searched around (ue, ve): masked squared difference.
    {u, v, rms, in_place_rms, pixels} (grey levels per channel), or None when it cannot be searched."""
    ys, xs = np.nonzero(region)
    if len(xs) == 0:
        return None
    x0, x1, y0, y1 = int(xs.min()), int(xs.max()) + 1, int(ys.min()), int(ys.max()) + 1
    T = ref[y0:y1, x0:x1].astype(np.float32)
    M = region[y0:y1, x0:x1].astype(np.float32)
    H, W = cur.shape[:2]
    ax, ay = int(round(u)) - x0, int(round(v)) - y0
    ue0, ve0 = int(round(ue)), int(round(ve))
    sx0, sy0 = max(0, ue0 - ax - search_px), max(0, ve0 - ay - search_px)
    sx1, sy1 = min(W, ue0 - ax + T.shape[1] + search_px), min(H, ve0 - ay + T.shape[0] + search_px)
    if sx1 - sx0 < T.shape[1] or sy1 - sy0 < T.shape[0]:
        return None
    res = cv2.matchTemplate(cur[sy0:sy1, sx0:sx1].astype(np.float32), T, cv2.TM_SQDIFF, mask=np.dstack([M] * 3))
    res = np.sqrt(np.maximum(res, 0.0) / (3.0 * float(M.sum())))
    lo, _, loc, _ = cv2.minMaxLoc(res)
    zx, zy = ue0 - ax - sx0, ve0 - ay - sy0
    here = float(res[zy, zx]) if 0 <= zy < res.shape[0] and 0 <= zx < res.shape[1] else None
    return {"u": loc[0] + sx0 + ax, "v": loc[1] + sy0 + ay, "rms": float(lo), "in_place_rms": here,
            "pixels": int(M.sum())}


def _finds(ref, cur, u, v, ue, ve, halves_px, search_px, usable=None):
    """Per patch half size: the patch of ref around (u, v) (clipped to the frame) searched for in cur around
    (ue, ve). {half_px, u, v (where its anchor landed), peak, in_place}."""
    a, b = ref.astype(np.float32), cur.astype(np.float32)
    H, W = a.shape[:2]
    u0, v0, ue0, ve0 = int(round(u)), int(round(v)), int(round(ue)), int(round(ve))
    out = []
    for h in halves_px:
        h = int(round(h))
        tx0, tx1, ty0, ty1 = max(0, u0 - h), min(W, u0 + h + 1), max(0, v0 - h), min(H, v0 + h + 1)
        if tx1 - tx0 < 9 or ty1 - ty0 < 9 or not (0 <= u0 < W and 0 <= v0 < H):
            continue
        T = a[ty0:ty1, tx0:tx1]
        M = None if usable is None else usable[ty0:ty1, tx0:tx1].astype(np.float32)
        if M is not None and float(M.mean()) < 0.5:
            continue                                     # the robot's own body fills most of this patch
        if float(T[M > 0].std() if M is not None else T.std()) < MIN_PATCH_STD:
            continue
        ax, ay = u0 - tx0, v0 - ty0                      # the located pixel inside the patch
        x0, y0 = max(0, ue0 - ax - search_px), max(0, ve0 - ay - search_px)
        x1 = min(W, ue0 - ax + T.shape[1] + search_px)
        y1 = min(H, ve0 - ay + T.shape[0] + search_px)
        if x1 - x0 < T.shape[1] or y1 - y0 < T.shape[0]:
            continue
        res = cv2.matchTemplate(b[y0:y1, x0:x1], T, cv2.TM_CCOEFF_NORMED,
                                mask=None if M is None else np.dstack([M] * 3))
        res = np.nan_to_num(res, nan=-1.0, posinf=-1.0, neginf=-1.0)
        _, peak, _, loc = cv2.minMaxLoc(res)
        zx, zy = ue0 - ax - x0, ve0 - ay - y0
        same = float(res[zy, zx]) if 0 <= zy < res.shape[0] and 0 <= zx < res.shape[1] else None
        out.append({"half_px": h, "u": loc[0] + x0 + ax, "v": loc[1] + y0 + ay, "peak": float(peak),
                    "in_place": same})
    return out


def table_motion(model, q_ref, q_cur, H, width, height):
    """The planar motion (yaw about B z, then a shift) that carries table points (z = 0 in frame B) from the
    earlier look's frame B to the later one's, fitted so that projecting them through the body model reproduces
    the homography H. (fn mapping a frame B point, residual px) or (None, None) when it cannot be fitted."""
    if H is None:
        return None, None
    src, P = [], []
    for u in np.linspace(0.08, 0.92, 9) * width:
        for v in np.linspace(0.08, 0.92, 7) * height:
            o, d = model.pixel_ray(q_ref, u, v)
            if float(d[2]) > -1e-3:
                continue
            P.append(o - d * (float(o[2]) / float(d[2])))
            src.append((u, v))
    if len(P) < 8:
        return None, None
    P = np.asarray(P)
    dst = np.asarray([_apply(H, u, v) for u, v in src])

    def move(x, pts):
        c, s_ = math.cos(x[0]), math.sin(x[0])
        R = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
        return pts @ R.T + np.array([x[1], x[2], 0.0])

    def resid(x):
        out = []
        for q, (du, dv) in zip(move(x, P), dst):
            u, v, depth = model.project(q_cur, q)
            out += [u - du, v - dv] if depth > 0 and math.isfinite(u) else [50.0, 50.0]
        return np.asarray(out)

    fit = least_squares(resid, np.zeros(3), x_scale=[0.05, 0.02, 0.02], max_nfev=200)
    err = float(np.median(np.abs(fit.fun)))
    return (lambda pt: move(fit.x, np.asarray(pt, dtype=float).reshape(1, 3))[0]), err


def hidden_by_body(model, q, p_m, margin_m=0.005):
    """The robot body whose geometry the wrist camera's sight line to p crosses first (a ray cast on the robot's
    own model at joint pose q), or None when nothing of the robot is in the way."""
    m = model.mj
    d = mujoco.MjData(m)
    d.qpos[:] = np.asarray(getattr(model, "_qfull", m.qpos0), dtype=float)
    d.qpos[model._qadr] = np.asarray(q, dtype=float)
    mujoco.mj_kinematics(m, d)
    mujoco.mj_camlight(m, d)
    pc = d.cam_xpos[model._cam].copy()
    ray = np.asarray(p_m, dtype=float) - pc
    n = float(np.linalg.norm(ray))
    if n <= 2 * margin_m:
        return None
    gid = np.array([-1], dtype=np.int32)
    dist = mujoco.mj_ray(m, d, pc, ray / n, None, 1, -1, gid)
    if dist >= 0 and dist < n - margin_m and gid[0] >= 0:
        return m.body(int(m.geom_bodyid[gid[0]])).name
    return None


def _agreeing(finds, tol_px):
    """The largest group of finds whose places agree within tol_px, when it has two or more."""
    best = []
    for f in finds:
        group = [g for g in finds if abs(g["u"] - f["u"]) <= tol_px and abs(g["v"] - f["v"]) <= tol_px]
        if len(group) > len(best):
            best = group
    return best if len(best) >= 2 else []


def robot_pixels(model, q, width, height, step=ROBOT_GRID_PX):
    """Pixels of the wrist look that show the robot's own body at joint pose q (camera rays cast on the robot's
    model every `step` pixels, each hit grown to its cell): the same in two looks from one pose."""
    m = model.mj
    d = mujoco.MjData(m)
    d.qpos[:] = np.asarray(getattr(model, "_qfull", m.qpos0), dtype=float)
    d.qpos[model._qadr] = np.asarray(q, dtype=float)
    mujoco.mj_kinematics(m, d)
    mujoco.mj_camlight(m, d)
    pc = d.cam_xpos[model._cam].copy()
    gid = np.array([-1], dtype=np.int32)
    out = np.zeros((height, width), bool)
    for v in range(0, height, step):
        for u in range(0, width, step):
            _, ray = model.pixel_ray(q, u + step / 2.0, v + step / 2.0)
            if mujoco.mj_ray(m, d, pc, ray, None, 1, -1, gid) >= 0:
                out[v:v + step, u:u + step] = True
    return cv2.dilate(out.astype(np.uint8), np.ones((step + 1, step + 1), np.uint8)) > 0


def region_motion(model, ref_rec, cur_rec, point_mm, ref_id, cur_id):
    """How far the object at point_mm moved over the table between two standard looks.
    {checked, moved_mm_est, moved_mm_vec, confidence, method, looks, why}."""
    base = {"looks": [ref_id, cur_id], "checked": False, "moved_mm_est": None}
    for rec, name in ((ref_rec, ref_id), (cur_rec, cur_id)):
        if rec is None or rec.get("img") is None:
            return {**base, "why": f"look {name} keeps no image to compare"}
        if rec.get("can_see") == "dark":
            return {**base, "why": f"look {name} is dark"}
    ref, cur = np.asarray(ref_rec["img"])[:, :, :3], np.asarray(cur_rec["img"])[:, :, :3]
    p = np.asarray(point_mm, dtype=float) / MM
    u, v, depth = model.project(ref_rec["q"], p)
    H_, W_ = ref.shape[:2]
    if not (math.isfinite(u) and math.isfinite(v)) or depth <= 0 or not (0 <= u < W_ and 0 <= v < H_):
        return {**base, "why": "the object's located point is outside the standard look"}
    mmpp = depth * MM / model.f_px
    hider = hidden_by_body(model, ref_rec["q"], p)
    if hider is not None:
        return {**base, "why": f"the robot's {hider} hides the object's located point in the standard look"}
    region0 = region_mask(ref, u, v, EXTENT_MM[1] / mmpp)
    if region0 is None:
        # no edge: patches sized by the clamp would match the still surroundings, not the object, and read a moved
        # object as still (a dim look, where every colour is within FILL_DIFF of the next, or a plain run of colour)
        return {**base, "why": f"the object's edge cannot be told from its surroundings in this look (its colour "
                               f"runs on past {EXTENT_MM[1]:g} mm: a dim look or a plain run of colour), so its motion "
                               "is not measured", "hint": "get light on the scene, then take standard_look again"}
    _, match_error = homography(ref, cur, residual=True)
    base["match_error_px"] = match_error
    Hm, inl = homography(ref, cur)
    ue, ve = (u, v) if Hm is None else _apply(Hm, u, v)
    p_cur, how = p, "none found (assumed still)" if Hm is None else "homography of the table"
    carry, err = table_motion(model, ref_rec["q"], cur_rec["q"], Hm, W_, H_)
    if carry is not None and err is not None and err <= MAX_FIT_PX:
        p_cur = carry(p)                                  # where the object's own point would be had it stayed
        uu, vv, dd = model.project(cur_rec["q"], p_cur)
        if dd > 0 and math.isfinite(uu) and math.isfinite(vv):
            ue, ve, how = uu, vv, f"table motion in frame B (fit {err:.1f} px)"
    search = int(round(SEARCH_MM / mmpp))
    b_pt = _ray_plane(model, cur_rec["q"], ue, ve, p_cur[2])

    def to_mm(fu, fv):
        a_pt = _ray_plane(model, cur_rec["q"], fu, fv, p_cur[2])
        vec = (a_pt - b_pt) * MM if a_pt is not None and b_pt is not None else np.array([fu - ue, fv - ve, 0.0]) * mmpp
        vec[2] = 0.0
        return float(np.linalg.norm(vec)), vec

    body = robot_pixels(model, ref_rec["q"], W_, H_)
    if not np.array_equal(np.asarray(ref_rec["q"]), np.asarray(cur_rec["q"])):
        body |= robot_pixels(model, cur_rec["q"], W_, H_)
    region = region0 & ~body
    ext = extent_px(ref, u, v, EXTENT_MM[0] / mmpp, EXTENT_MM[1] / mmpp)
    ua, va, ref_w = u, v, ref
    if Hm is not None:   # undo the camera's turn over the table first, so the face is compared the same way up
        ref_w = cv2.warpPerspective(ref, Hm, (W_, H_))
        region = cv2.warpPerspective(region.astype(np.uint8), Hm, (W_, H_), flags=cv2.INTER_NEAREST) > 0
        ua, va = _apply(Hm, u, v)
    region &= ~body
    n_region = int(region.sum())
    shape = shape_find(ref_w, cur, region, ua, va, ue, ve, search) if n_region >= MIN_REGION_PX else None
    # a small face is matched by patches, but its own pixels still say whether it is where it was (a textured table
    # can dominate a patch around a small object, so the patches alone read a moved one as still)
    small = shape_find(ref_w, cur, region, ua, va, ue, ve, search) if MIN_SMALL_PX <= n_region < MIN_REGION_PX \
        else None
    lost = None
    if shape is not None and shape["rms"] > MATCH_RMS:
        # the face is not where any shift of it fits: it moved far or turned, or a still shadow (the arm's) now
        # falls across it at another place. The patches (normalised correlation) decide; suspect only if they fail
        lost = {**base, "why": "the object's face was not found again in the later look, nor its colour patch: it "
                               "may have moved far, turned over or be hidden", "suspect": True,
                "method": f"object face match (rms {shape['rms']:.1f}), then colour patch correlation"}
        shape = None
    if shape is not None:
        stayed = shape["in_place_rms"] is not None and shape["in_place_rms"] <= shape["rms"] + STILL_RMS
        mm, vec = (0.0, np.zeros(3)) if stayed else to_mm(shape["u"], shape["v"])
        method, conf = f"object face match ({shape['pixels']} px, rms {shape['rms']:.1f})", 1.0 - shape["rms"] / 255
        votes = {"still": int(stayed), "moved": int(not stayed)}
    else:
        halves = [max(8.0, g * ext + PATCH_PAD_PX) for g in PATCH_GROW]
        every = _finds(ref_w, cur, ua, va, ue, ve, halves, search, ~body)
        if not every and lost is not None:
            return lost
        if not every:
            h = int(round(halves[1]))
            near = body[max(0, int(v) - h):int(v) + h + 1, max(0, int(u) - h):int(u) + h + 1]
            if near.size and float(near.mean()) >= 0.3:
                return {**base, "why": "the robot's own body covers much of the object in the standard look"}
            return {**base, "why": "the look is too plain around the object's located point to compare a patch"}
        still = [f for f in every if f["in_place"] is not None and f["in_place"] >= STILL_PEAK
                 and f["in_place"] >= f["peak"] - STILL_MARGIN]
        finds = [f for f in every if f["peak"] >= MIN_PEAK and all(f is not g for g in still)]
        if not finds and not still and lost is None:   # after a lost face only a strong peak counts as a find
            finds = _agreeing([f for f in every if f["peak"] >= WEAK_PEAK], max(2.0, AGREE_MM / mmpp))
        if not finds and not still:
            return lost or {**base, "why": "the object's patch was not found again in the later look: it may have "
                                           "moved far, turned over or be hidden", "suspect": True}
        if not still and len(_agreeing(finds, max(2.0, AGREE_MM / mmpp))) < min(2, len(finds)):
            return lost or {**base, "why": "the object's face is too small in the look and the patch sizes disagree "
                                           "on where it went; look at it from closer"}
        moves = [(0.0, np.zeros(3), dict(f, peak=f["in_place"])) for f in still]   # a still vote: no motion
        moves += [(*to_mm(f["u"], f["v"]), f) for f in finds]
        moves.sort(key=lambda x: (x[0], x[2]["half_px"]))
        mm, vec, f = moves[(len(moves) - 1) // 2]
        if lost is not None and (still or mm < LOST_MIN_MM):
            # the face changed in place, so a patch that says "still" is matching something that did not move with
            # the object (a still shadow edge, the table): the face match's "not found" stands
            return lost
        method = f"colour patch correlation, median of {len(moves)} of {len(PATCH_GROW)} patch sizes"
        if lost is not None:
            method += f" (the face match found no fit: {lost['method'].split(', then')[0]})"
        conf, votes = f["peak"], {"still": len(still), "moved": len(moves) - len(still)}
        if (small is not None and mm < LOST_MIN_MM and small["in_place_rms"] is not None
                and small["in_place_rms"] > MATCH_RMS):
            # the patches say still, but the small face's own pixels are no longer there: the patches matched the
            # still table around it. Where the face fits again is the answer; nowhere: suspect
            if small["rms"] > MATCH_RMS:
                return {**base, "why": "the object's small face is not where it was and was not found again in the "
                                       "later look: it may have moved far, turned over or be hidden", "suspect": True,
                        "method": f"small face match (rms {small['rms']:.1f} in place {small['in_place_rms']:.1f})"}
            mm, vec = to_mm(small["u"], small["v"])
            method = (f"object face match ({small['pixels']} px, rms {small['rms']:.1f}; the patches around this "
                      "small face matched the still table)")
            conf, votes = 1.0 - small["rms"] / 255, {"still": 0, "moved": 1}
    crop = _crop_relocalize(ref, cur, u, v, mmpp)
    return {"looks": [ref_id, cur_id], "checked": True, "moved_mm_est": round(mm, 1),
            "moved_mm_vec": [round(float(x), 1) for x in vec], "confidence": round(float(conf), 3),
            "in_place": mm == 0.0, "votes": votes, "extent_mm": round(ext * mmpp, 1), "method": method,
            "camera_motion": how,
            "crop_relocalize": crop, "why": ""}


def _crop_relocalize(ref, cur, u, v, mmpp, half_mm=50.0, side_px=360):
    h = max(8, int(round(half_mm / mmpp)))
    H, W = ref.shape[:2]
    u0, v0 = int(round(u)), int(round(v))
    sl = (slice(max(0, v0 - h), min(H, v0 + h)), slice(max(0, u0 - h), min(W, u0 + h)))
    a, b = ref[sl], cur[sl]
    if a.size == 0 or min(a.shape[:2]) < 16:
        return {"confidence": 0.0, "why": "crop outside the frame"}
    f = max(1.0, side_px / max(a.shape[:2]))
    cv2.setRNGSeed(0)
    r = perception.relocalize(cv2.resize(a, None, fx=f, fy=f, interpolation=cv2.INTER_CUBIC),
                              cv2.resize(b, None, fx=f, fy=f, interpolation=cv2.INTER_CUBIC))
    sh = np.asarray(r["shift_px"], dtype=float) / f
    return {"shift_mm_est": round(float(np.linalg.norm(sh)) * mmpp, 1), "confidence": r["confidence"],
            "inliers": r["inliers"], "why": r.get("why", "")}


def comparison_error_px(ex):
    looks = [v for v in (getattr(ex, "_looks", None) or {}).values() if v.get("img") is not None]
    if not looks:
        return 0.5  # numeric: half a pixel of image coordinate rounding
    ref = looks[-2] if len(looks) > 1 else looks[-1]
    _, error = homography(ref["img"], looks[-1]["img"], residual=True)
    return max(0.5, error)
