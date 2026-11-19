# SPDX-License-Identifier: AGPL-3.0-only
"""Ground-plane base motion from two standard looks, for the epochs package.

Features matched between the reference standard look and a later one are carried out along their camera rays (camera
poses from the body model at each look's own joint pose) to the mounting plane z = 0 of frame B, and a planar rigid
motion (turn about z plus a shift) is fitted to the pairs by seeded RANSAC and a least squares refit. The fit D maps
the reference look's frame B coordinates of a fixed point to its present frame B coordinates.

Two hypotheses are fitted: one over every pair, and one over the pairs that moved in the image. Features that do not
move in the image at all ride with the robot (the finger, a base plate in view, the arm's own shadow), so once the
surface moved they would vote for "nothing moved". The moving hypothesis wins when it has at least half as many
inliers as there are still features and its inliers spread over at least half the image extent the still ones cover
(a part that moved alone, such as a pushed device, covers a patch, not the view).

Error model, per point p in frame B now, in mm:
- the surface under the camera may sit up to the support book's height uncertainty off the mounting plane; that scales every plane
  point about the point straight under the camera (both looks share the camera pose in frame B), which leaves the
  fitted turn exact and puts the fitted shift off by the support book's height uncertainty / camera height times the fitted
  displacement of that point under the camera, the same at every point;
- the fit's own scatter: inlier RMS, growing with the distance from the inliers'
  centre (a turn error has a lever arm);
- the turn's own error: the fitted angular covariance times the distance from the inliers' centre. The fitted turn can be off by
  more than its scatter shows because of pixel rounding; a turn under twice the fit error at its own point is not called a base move (epochs_book);
- the joint quantum and pixel comparison error for the joint and camera model.
Nothing here reads simulator ground truth: the inputs are two frames, two joint poses and the body model.
"""

import math

import cv2
import numpy as np

ORB_FEATURES = 6000
ORB_FAST = 5                 # a low corner threshold: a table's texture has soft corners
RATIO = 0.75
MIN_PAIRS = 20               # fewer matched pairs on the plane than this: no estimate
MIN_INLIERS = 20             # fewer RANSAC inliers than this: no estimate
RANSAC_ITERS = 400
RANSAC_SEED = 0
INLIER_MM = 3.0              # a pair within this of the fitted motion is an inlier
MIN_SPAN_MM = 20.0           # a RANSAC sample's two points must be this far apart
STUCK_PX = 1.5               # a feature that moved less than this in the image is still
MM = 1000.0


# ------------------------------------------------------------------ planar poses (mm, rad)
def pose(x=0.0, y=0.0, yaw=0.0):
    return {"x": float(x), "y": float(y), "yaw": float(yaw)}


def _mat(p):
    c, s = math.cos(p["yaw"]), math.sin(p["yaw"])
    return np.array([[c, -s, p["x"]], [s, c, p["y"]], [0.0, 0.0, 1.0]])


def _pose_of(M):
    return pose(M[0, 2], M[1, 2], math.atan2(M[1, 0], M[0, 0]))


def compose(a, b):
    """a then b as maps: compose(a, b)(p) = a(b(p))."""
    return _pose_of(_mat(a) @ _mat(b))


def invert(a):
    return _pose_of(np.linalg.inv(_mat(a)))


def apply(a, p_mm):
    """A planar pose applied to a 3D point in mm (z unchanged)."""
    q = _mat(a) @ np.array([p_mm[0], p_mm[1], 1.0])
    return [float(q[0]), float(q[1]), float(p_mm[2])]


def motion_size(a, at_mm):
    """(turn in degrees, displacement in mm of the point at_mm) of a planar map a."""
    p = apply(a, [at_mm[0], at_mm[1], 0.0])
    return math.degrees(a["yaw"]), float(math.hypot(p[0] - at_mm[0], p[1] - at_mm[1]))


# ------------------------------------------------------------------ features
def _gray(img):
    a = np.asarray(img)[:, :, :3].astype(np.uint8)
    g = cv2.cvtColor(np.ascontiguousarray(a), cv2.COLOR_RGB2GRAY)
    lo, hi = np.percentile(g, (1.0, 99.0))
    if hi - lo >= 2.0:
        g = np.clip((g.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)
    return cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(g)


def match(ref_img, cur_img):
    """Matched pixels (N x 2 each, u right, v down) between two frames, ORB with a ratio test, in a fixed order."""
    orb = cv2.ORB_create(nfeatures=ORB_FEATURES, fastThreshold=ORB_FAST)
    k0, d0 = orb.detectAndCompute(_gray(ref_img), None)
    k1, d1 = orb.detectAndCompute(_gray(cur_img), None)
    if d0 is None or d1 is None or len(k0) < 2 or len(k1) < 2:
        return np.zeros((0, 2)), np.zeros((0, 2))
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d0, d1, k=2)
    good = [p[0] for p in pairs if len(p) == 2 and p[0].distance < RATIO * p[1].distance]
    uv0 = np.array([k0[m.queryIdx].pt for m in good], dtype=float).reshape(-1, 2)
    uv1 = np.array([k1[m.trainIdx].pt for m in good], dtype=float).reshape(-1, 2)
    return uv0, uv1


# ------------------------------------------------------------------ camera geometry (the model's pinhole)
def camera(model, q):
    pc, Rc = model.fk(q)["camera"]
    return np.asarray(pc, dtype=float), np.asarray(Rc, dtype=float)


def rays(model, cam, uv):
    """Unit ray directions in frame B for pixels uv, the same pinhole as RobotModel.pixel_ray."""
    _, Rc = cam
    dc = np.column_stack([(uv[:, 0] - model.cam_w / 2.0) / model.f_px, -(uv[:, 1] - model.cam_h / 2.0) / model.f_px,
                          -np.ones(len(uv))])
    dw = dc @ Rc.T
    return dw / np.linalg.norm(dw, axis=1, keepdims=True)


def on_plane(model, cam, uv, z_m=0.0):
    """(points N x 3 in m, valid mask) where the pixels' rays meet the plane z = z_m."""
    pc, _ = cam
    d = rays(model, cam, uv)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (z_m - pc[2]) / d[:, 2]
    ok = (d[:, 2] < -1e-3) & np.isfinite(t) & (t > 0) & (t < 5.0)
    P = pc[None, :] + np.where(ok, t, 0.0)[:, None] * d
    return P, ok


def foot_point(cam):
    """Where the camera's optical axis meets the mounting plane (m), else the point under the camera."""
    pc, Rc = cam
    view = -Rc[:, 2]
    if view[2] < -0.1 and pc[2] > 0:
        return pc + view * (pc[2] / -view[2])
    return np.array([pc[0], pc[1], 0.0])


# ------------------------------------------------------------------ the rigid fit
def _fit2(A, B):
    """Least squares planar rigid map (R, t) with B ~ R A + t (2D arrays)."""
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (A - ca).T @ (B - cb)
    ang = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    c, s = math.cos(ang), math.sin(ang)
    R = np.array([[c, -s], [s, c]])
    return R, cb - R @ ca, ang


def _ransac(A, B, thr, min_span=MIN_SPAN_MM):
    n = len(A)
    rng = np.random.default_rng(RANSAC_SEED)
    best, best_key = None, None
    for _ in range(RANSAC_ITERS):
        i, j = (int(x) for x in rng.choice(n, size=2, replace=False))
        if np.linalg.norm(A[i] - A[j]) < min_span:
            continue
        R, t, _ = _fit2(A[[i, j]], B[[i, j]])
        res = np.linalg.norm(A @ R.T + t - B, axis=1)
        inl = res < thr
        key = (int(inl.sum()), -float(res[inl].sum()))
        if best_key is None or key > best_key:
            best, best_key = inl, key
    if best is None or best.sum() < 2:
        return None
    inl = best
    for _ in range(3):
        R, t, ang = _fit2(A[inl], B[inl])
        res = np.linalg.norm(A @ R.T + t - B, axis=1)
        new = res < thr
        if new.sum() < 2 or np.array_equal(new, inl):
            break
        inl = new
    R, t, ang = _fit2(A[inl], B[inl])
    res = np.linalg.norm(A @ R.T + t - B, axis=1)
    return {"R": R, "t": t, "yaw": ang, "inliers": inl, "res": res}


def _extent(uv):
    """RMS distance of pixels from their centre (0 for fewer than two)."""
    if len(uv) < 2:
        return 0.0
    return float(np.sqrt(np.mean(np.sum((uv - uv.mean(axis=0)) ** 2, axis=1))))


def estimate(model, ref, cur, B=None):
    """The planar motion D (reference frame B -> present frame B) of the world seen in two standard looks.

    ref, cur: {"img": H x W x 3 uint8, "q": joint pose of the look}. Returns {"ok": False, "why"} or
    {"ok": True, "D": pose (mm, rad), "inliers", "pairs", "rms_mm", "err": error model, "foot_mm", "camera_height_mm",
    "still"}."""
    uv0, uv1 = match(ref["img"], cur["img"])
    if len(uv0) < MIN_PAIRS:
        return {"ok": False, "why": f"only {len(uv0)} features matched between the two standard looks",
                "pairs": int(len(uv0))}
    cam0, cam1 = camera(model, ref["q"]), camera(model, cur["q"])
    P0, ok0 = on_plane(model, cam0, uv0)
    P1, ok1 = on_plane(model, cam1, uv1)
    ok = ok0 & ok1
    stuck = np.linalg.norm(uv1 - uv0, axis=1) < STUCK_PX
    if int(ok.sum()) < MIN_PAIRS:
        return {"ok": False, "why": "too few matched features lie on the surface below the camera",
                "pairs": int(ok.sum())}
    bound = B
    A, target = P0[ok, :2] * MM, P1[ok, :2] * MM
    # Image numerics scaled by the camera's height, relative to the sensor's metre unit.
    image_scale = abs(float(cam0[0][2]))
    fit = _ransac(A, target, INLIER_MM * image_scale, MIN_SPAN_MM * image_scale)
    use = ok.copy()
    moving = ok & ~stuck
    still = ok & stuck
    if int(moving.sum()) >= MIN_PAIRS:
        fit2 = _ransac(P0[moving, :2] * MM, P1[moving, :2] * MM, INLIER_MM * image_scale, MIN_SPAN_MM * image_scale)
        if fit2 is not None:
            n2 = int(fit2["inliers"].sum())
            wide = _extent(uv1[np.flatnonzero(moving)[fit2["inliers"]]]) >= 0.5 * _extent(uv1[still])
            if n2 >= max(MIN_INLIERS, 0.5 * int(still.sum())) and wide:
                fit, use = fit2, moving
    if fit is None or int(fit["inliers"].sum()) < MIN_INLIERS:
        return {"ok": False, "why": "no consistent motion of the surface below the camera", "pairs": int(ok.sum())}
    idx = np.flatnonzero(use)
    inl_idx = idx[fit["inliers"]]
    D = pose(fit["t"][0], fit["t"][1], fit["yaw"])
    n_in = int(len(inl_idx))
    rms = float(np.sqrt(np.mean(fit["res"][fit["inliers"]] ** 2)))
    Bin = P1[inl_idx, :2] * MM
    centre = Bin.mean(axis=0)
    spread = float(np.sqrt(np.mean(np.sum((Bin - centre) ** 2, axis=1)))) or 1.0
    foot = foot_point(cam0) * MM
    nadir = np.asarray(cam0[0][:2], dtype=float) * MM      # a plane offset scales the plane points about this point
    _, disp_nadir = motion_size(D, nadir)
    height = max(float(cam0[0][2]) * MM, 1.0)
    support = bound.space.support() if bound is not None else None
    sigma_z = support[1] if support and support[1] is not None else rms / MM
    stat = rms
    yaw_error = stat / spread
    pixel_error = abs(float(cam1[0][2])) * MM / model.f_px * STUCK_PX
    floor = max(pixel_error, bound.quantum(cur["q"], [0, 0, 1]) * MM if bound is not None else 0)
    err = {"plane_mm": sigma_z * MM / height * disp_nadir, "stat_mm": stat,
           "centre_mm": [float(centre[0]), float(centre[1])], "spread_mm": spread,
           "yaw_err_deg": math.degrees(yaw_error), "floor_mm": floor,
           "source": support[2] if support and support[1] is not None else "seen"}
    return {"ok": True, "D": D, "inliers": n_in, "pairs": int(ok.sum()), "rms_mm": rms, "err": err,
            "foot_mm": [float(foot[0]), float(foot[1])], "camera_height_mm": height, "still": int(still.sum())}


def error_at(err, p_mm):
    """The estimate's error (mm) at a point in frame B now (err from estimate; None = exact)."""
    if not err:
        return 0.0
    c = err["centre_mm"]
    dist = math.hypot(p_mm[0] - c[0], p_mm[1] - c[1])
    turn = math.radians(err.get("yaw_err_deg", 0)) * dist
    return float(err["plane_mm"] + err["stat_mm"] * (1.0 + dist / err["spread_mm"]) + turn + err["floor_mm"])
