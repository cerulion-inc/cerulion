# SPDX-License-Identifier: AGPL-3.0-only
"""What a survey sees (package survey, I32): the floor patch of each view and the raised things in it.

Inputs are the frames and the body model only (RobotModel.fk, pixel_ray, project, and the robot-only MuJoCo model
for the robot's own geometry); nothing reads a scene or ground truth, and no robot or device is named.

- floor_view(model, q, z): the patch of the plane z (the surface the robot stands on) a look covers: the image
  centre and corners there, in base mm; a corner whose ray never meets the plane (above the horizon) is None.
- robot_mask(model, q): pixels where the robot's own geometry is in the line of sight (rays through a pixel grid
  against the robot-only model).
- find(views, z): RAISED regions by plane-sweep stereo between neighbouring views. For each view the frames of its
  neighbours are warped onto planes z + h (homographies from the body model's camera poses) and compared: by local
  normalised cross-correlation where there is texture, and by shadow-free colour (chromaticity) where there is
  none. A pixel that matches the plane z itself is floor; one that matches only a plane at least RAISED_MIN_M
  above it, in every neighbour that sees it, is raised. Pixels that look the same in two views at the same place
  ride with the camera (the robot's own hand) and are left out. Raised pixels are grouped into regions, each with
  a pixel box, a height and a floor position from the matched plane; a region that stays at the same pixels in a
  neighbour view rides with the camera and is dropped.
- merge(regions): the same thing seen from several views is one candidate (floor positions within MERGE_M).

A candidate is a region that does not lie on the floor plane in two views. An object, a part of the robot that
rides with the camera, or a moving shadow can make one: the frames are the evidence, the candidates a pointer.
"""

import math

import cv2
import mujoco
import numpy as np

MM = 1000.0
SCALE = 2                  # frames are compared at half size
WIN = 9                    # comparison window, pixels at half size
N_PLANES = 24              # heights swept above the floor plane
RAISED_MIN_M = 0.025       # a pixel that matches only a plane this far above the floor or more is raised
TOP_FRAC = 0.8             # the highest plane swept: this share of the camera's height above the floor
NCC_MATCH = 0.85           # a textured pixel matches a plane at this correlation or more
NCC_MARGIN = 0.3           # and by this much more than it matches the floor
TEXTURE_VAR = 1e-3         # grey variance (0..1 units) a window needs for the correlation to count
CHROMA_FLOOR = 0.12        # a pixel whose colour differs from the floor's warp by this much (chromaticity) ...
CHROMA_RATIO = 0.4         # ... and matches a raised plane this much better is raised by colour
SAME_TOL = 0.03            # mean colour difference under which a pixel is the same in two views (rides along)
MIN_AREA_FRAC = 0.002      # a region smaller than this share of the frame is dropped
RIDES_IOU = 0.5            # a region at the same pixels in a neighbour view (this overlap) rides with the camera
MERGE_M = 0.05             # regions this close on the floor (or within half their size) are one candidate
MASK_STEP = 4              # robot mask ray grid, full-size pixels
NEIGHBOURS = 2             # views compared on each side
SHADOW_DIFF = 0.03         # a region whose colour (chromaticity) differs from the floor's warp less than this is a
                           # shadow or a shading change, not a thing
MERGE_PAD_M = 0.02         # floor footprints this close count as touching when regions are merged


def _cam(model, q):
    pc, Rc = model.fk(np.asarray(q, dtype=float))["camera"]
    return np.asarray(pc, dtype=float), np.asarray(Rc, dtype=float)


def rays(model, q, us, vs):
    """(origin (3,), unit directions (n, 3)) in frame B for pixels (us, vs) through the body model's camera (the
    same pinhole as RobotModel.pixel_ray)."""
    pc, Rc = _cam(model, q)
    us, vs = np.asarray(us, dtype=float).reshape(-1), np.asarray(vs, dtype=float).reshape(-1)
    dc = np.stack([(us - model.cam_w / 2.0) / model.f_px, -(vs - model.cam_h / 2.0) / model.f_px,
                   -np.ones_like(us)], axis=1)
    dw = dc @ Rc.T
    return pc, dw / np.linalg.norm(dw, axis=1, keepdims=True)


def on_plane(model, q, us, vs, z):
    """Points (n, 3) where the pixels' rays meet the plane z (frame B, m); nan where a ray never meets it."""
    pc, d = rays(model, q, us, vs)
    zz = np.asarray(z, dtype=float).reshape(-1)          # one plane, or one height per pixel
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (zz - pc[2]) / d[:, 2]
    ok = (d[:, 2] < -1e-6) & (t > 0)
    p = pc[None, :] + t[:, None] * d
    p[~ok] = np.nan
    return p


def floor_view(model, q, z):
    """{centre, corners (top-left, top-right, bottom-right, bottom-left)} on the plane z, base mm (x, y) rounded to
    the mm; a point above the horizon is None."""
    w, h = float(model.cam_w), float(model.cam_h)
    pts = on_plane(model, q, [w / 2, 0, w, w, 0], [h / 2, 0, 0, h, h], z)

    def xy(p):
        return None if not np.all(np.isfinite(p)) else [int(round(p[0] * MM)), int(round(p[1] * MM))]

    return {"centre": xy(pts[0]), "corners": [xy(p) for p in pts[1:]]}


def footprint_bearings(model, q, z, n=24):
    """[(radius m, bearing rad)] of points spread over the floor patch of a look (rows and columns of the image)."""
    w, h = float(model.cam_w), float(model.cam_h)
    us, vs = np.meshgrid(np.linspace(0, w, n), np.linspace(0, h, n))
    p = on_plane(model, q, us, vs, z)
    p = p[np.all(np.isfinite(p), axis=1)]
    return [(float(math.hypot(x, y)), float(math.atan2(y, x))) for x, y, _ in p]


# ---------------------------------------------------------------- the robot's own pixels
_MASKS = {}


def robot_mask(model, q):
    """uint8 H x W: 1 where the robot's own geometry (robot-only model) is the first thing along the pixel's ray."""
    m = model.mj
    key = id(m)
    if key not in _MASKS:
        qadr = np.array([m.jnt_qposadr[m.joint(n).id] for n in model.joint_names])
        _MASKS[key] = (mujoco.MjData(m), qadr, m.camera("wrist").id)
    d, qadr, cid = _MASKS[key]
    d.qpos[:] = m.qpos0
    d.qpos[qadr] = np.asarray(q, dtype=float)
    mujoco.mj_kinematics(m, d)
    mujoco.mj_camlight(m, d)
    W, H = int(model.cam_w), int(model.cam_h)
    us, vs = np.meshgrid(np.arange(MASK_STEP / 2, W, MASK_STEP), np.arange(MASK_STEP / 2, H, MASK_STEP))
    pc = d.cam_xpos[cid].copy()
    Rc = d.cam_xmat[cid].reshape(3, 3)
    dc = np.stack([(us - W / 2.0) / model.f_px, -(vs - H / 2.0) / model.f_px, -np.ones_like(us, dtype=float)], -1)
    dw = dc.reshape(-1, 3) @ Rc.T
    dw /= np.linalg.norm(dw, axis=1, keepdims=True)
    n = len(dw)
    gid = np.zeros(n, np.int32)
    dist = np.zeros(n)
    mujoco.mj_multiRay(m, d, pc, np.ascontiguousarray(dw.reshape(-1)), None, 1, -1, gid, dist, None, n, 10.0)
    hit = (gid >= 0).reshape(us.shape).astype(np.uint8)
    return cv2.resize(hit, (W, H), interpolation=cv2.INTER_NEAREST)


# ---------------------------------------------------------------- plane sweep
def homography(model, qa, qb, z):
    """3 x 3 map from view a's pixels to view b's for points on the plane z, or None when the plane is not seen in
    both (fewer than four grid pixels of a land on it in front of b's camera)."""
    W, H = float(model.cam_w), float(model.cam_h)
    us, vs = np.meshgrid(np.linspace(0.1 * W, 0.9 * W, 4), np.linspace(0.1 * H, 0.9 * H, 4))
    pts = on_plane(model, qa, us, vs, z)
    src, dst = [], []
    for (u, v), X in zip(zip(us.reshape(-1), vs.reshape(-1)), pts):
        if not np.all(np.isfinite(X)):
            continue
        ub, vb, dep = model.project(qb, X)
        if dep > 1e-3 and math.isfinite(ub) and abs(ub) < 1e5 and abs(vb) < 1e5:
            src.append([u, v])
            dst.append([ub, vb])
    if len(src) < 4:
        return None
    Hm, _ = cv2.findHomography(np.float32(src), np.float32(dst), 0)
    return Hm


def _prep(img):
    """Half-size grey, chromaticity and colour (float 0..1) of an RGB frame."""
    rgb = np.asarray(img)[:, :, :3].astype(np.float32) / 255.0
    H, W = rgb.shape[:2]
    small = cv2.resize(rgb, (W // SCALE, H // SCALE), interpolation=cv2.INTER_AREA)
    grey = cv2.cvtColor(small, cv2.COLOR_RGB2GRAY)
    chroma = small / np.maximum(small.sum(axis=2, keepdims=True), 0.05)
    return {"rgb": small, "grey": grey, "chroma": chroma}


def _box(x):
    return cv2.boxFilter(x, -1, (WIN, WIN))


def _ncc(a, b):
    ma, mb = _box(a), _box(b)
    va = _box(a * a) - ma * ma
    vb = _box(b * b) - mb * mb
    cov = _box(a * b) - ma * mb
    return cov / np.sqrt(np.maximum(va * vb, 1e-8)), va, vb


def compare(model, va, vb, z):
    """Per pixel of view a (half size), what neighbour b says: {seen, raised, height} (height in m above z)."""
    A, B = va["prep"], vb["prep"]
    h2, w2 = A["grey"].shape
    S = np.diag([1.0 / SCALE, 1.0 / SCALE, 1.0])
    Si = np.linalg.inv(S)
    mask_a = cv2.resize(va["mask"], (w2, h2), interpolation=cv2.INTER_NEAREST).astype(np.float32)
    mask_b = cv2.resize(vb["mask"], (w2, h2), interpolation=cv2.INTER_NEAREST).astype(np.float32)
    same = (_box(np.abs(A["rgb"] - B["rgb"]).sum(axis=2)) < SAME_TOL).astype(np.float32)   # rides along
    cam_h = float(_cam(model, va["q"])[0][2] - z)
    heights = _heights(cam_h)
    shape = (h2, w2)
    best, best_h = np.full(shape, -1.0, np.float32), np.zeros(shape, np.float32)
    ncc0, d0 = np.full(shape, -1.0, np.float32), np.full(shape, 9.0, np.float32)
    dmin, dmin_h = np.full(shape, 9.0, np.float32), np.zeros(shape, np.float32)
    seen = np.zeros(shape, bool)
    var_a = None
    kernel = np.ones((WIN, WIN), np.uint8)
    flags = cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP
    for h in heights:
        Hm = homography(model, va["q"], vb["q"], z + h)
        if Hm is None:
            continue
        Hs = S @ Hm @ Si
        wg = cv2.warpPerspective(B["grey"], Hs, (w2, h2), flags=flags)
        wc = cv2.warpPerspective(B["chroma"], Hs, (w2, h2), flags=flags)
        inside = cv2.warpPerspective(np.ones(shape, np.float32), Hs, (w2, h2),
                                     flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP)
        wmask = cv2.warpPerspective(mask_b, Hs, (w2, h2), flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP)
        valid = cv2.erode((inside * (1 - wmask) * (1 - mask_a) * (1 - same) > 0.5).astype(np.uint8), kernel) > 0
        n, va_, vb_ = _ncc(A["grey"], wg)
        var_a = va_ if var_a is None else var_a
        n = np.where(valid & (vb_ > TEXTURE_VAR), n, -1.0)
        dcol = np.where(valid, _box(np.abs(A["chroma"] - wc).sum(axis=2)), 9.0)
        seen |= valid                 # b sees the pixel on some plane (the floor's may fall outside b's frame)
        if h == 0.0:
            ncc0, d0 = n, dcol
            continue
        up = n > best
        best, best_h = np.where(up, n, best), np.where(up, h, best_h)
        up = dcol < dmin
        dmin, dmin_h = np.where(up, dcol, dmin), np.where(up, h, dmin_h)
    if var_a is None:
        return {"seen": seen, "raised": np.zeros(shape, bool), "height": np.zeros(shape, np.float32),
                "floor_diff": np.zeros(shape, np.float32)}
    textured = var_a > TEXTURE_VAR
    by_ncc = seen & textured & (best > NCC_MATCH) & (best - ncc0 > NCC_MARGIN)
    by_col = seen & ~by_ncc & (d0 > CHROMA_FLOOR) & (dmin < CHROMA_RATIO * d0)
    return {"seen": seen, "raised": by_ncc | by_col, "height": np.where(by_ncc, best_h, dmin_h),
            "floor_diff": np.where(d0 < 9.0, d0, 0.0)}


def _iou(a, b):
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def regions(model, views, k, z, reach=NEIGHBOURS):
    """The raised regions of view k against its neighbours up to `reach` views away on each side (those that
    exist and share part of its patch: the hand hides the lower middle of each frame, so a further neighbour often
    sees what the next one cannot)."""
    va = views[k]
    said = [compare(model, va, views[j], z) for j in range(k - reach, k + reach + 1)
            if j != k and 0 <= j < len(views)]
    said = [s for s in said if s["seen"].any()]
    if not said:
        return []
    seen = np.sum([s["seen"] for s in said], axis=0)
    votes = np.sum([s["raised"] for s in said], axis=0)
    raised = (votes >= 1) & (2 * votes >= seen)     # at least half the neighbours that see the pixel call it raised
    hs = np.stack([np.where(s["raised"], s["height"], np.nan) for s in said])
    count = np.sum(np.isfinite(hs), axis=0)
    total = np.nansum(hs, axis=0)
    height = np.where(count > 0, total / np.maximum(count, 1), 0.0)
    spread = np.where(count > 1, np.nanmax(np.where(np.isfinite(hs), hs, -np.inf), axis=0)
                      - np.nanmin(np.where(np.isfinite(hs), hs, np.inf), axis=0), 0.0)
    raised &= spread < np.maximum(0.03, 0.25 * height)    # the neighbours that call it raised agree on how high
    m = cv2.morphologyEx(raised.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
    out = []
    for i in range(1, n):
        x, y, w, h, area = (int(v) for v in stats[i])
        if area < MIN_AREA_FRAC * m.size:
            continue
        sel = (lab == i) & (height > 0)
        if not sel.any():
            continue
        # a thing differs in colour from the floor it hides; a moving shadow keeps the floor's colour (chromaticity)
        diff = np.max([np.where(s["seen"], s["floor_diff"], 0.0) for s in said], axis=0)
        if float(np.median(diff[sel])) < SHADOW_DIFF:
            continue
        hv = height[sel]
        vv, uu = np.nonzero(sel)
        pts = on_plane(model, va["q"], (uu + 0.5) * SCALE, (vv + 0.5) * SCALE, z + hv)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if len(pts) == 0:
            continue
        lo, hi = np.percentile(pts[:, :2], 10, axis=0), np.percentile(pts[:, :2], 90, axis=0)
        where = np.median(pts[:, :2], axis=0)
        top = float(np.percentile(hv, 90))
        camera = _cam(model, va["q"])[0]
        e_z = (_heights(float(camera[2] - z))[-1] - RAISED_MIN_M) / (N_PLANES - 2)
        e_xy = e_z * float(np.linalg.norm(camera[:2] - where)) / float(camera[2] - z - top)
        out.append({"err_m": [e_xy, e_z], "view": k, "look_id": va["look_id"],
                    "box_px": [x * SCALE, y * SCALE, (x + w) * SCALE, (y + h) * SCALE], "area_px": area * SCALE ** 2,
                    "where_m": where, "height_m": float(np.median(hv)),
                    "top_m": top, "size_m": np.maximum(hi - lo, 0.0),
                    "agree": int(min(len(said), np.max(votes[lab == i])))})
    return out


def rides_along(regs_by_view):
    """Drop regions that stay at the same pixels in a neighbouring view: they ride with the camera."""
    keep = []
    for k, regs in enumerate(regs_by_view):
        others = [r for j in (k - 1, k + 1) if 0 <= j < len(regs_by_view) for r in regs_by_view[j]]
        keep.append([r for r in regs if not any(_iou(r["box_px"], o["box_px"]) > RIDES_IOU for o in others)])
    return keep


def merge(regs):
    """Candidates: regions grouped by floor position: two regions are one when their floor footprints (where, grown
    by half their size and MERGE_PAD_M) overlap, or their centres lie within MERGE_M."""
    n = len(regs)
    parent = list(range(n))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        for j in range(i + 1, n):
            a, b = regs[i], regs[j]
            gap = np.abs(a["where_m"] - b["where_m"]) - 0.5 * (a["size_m"] + b["size_m"]) - MERGE_PAD_M
            if bool(np.all(gap < 0)) or float(np.linalg.norm(a["where_m"] - b["where_m"])) < MERGE_M:
                parent[root(i)] = root(j)
    groups = {}
    for i in range(n):
        groups.setdefault(root(i), []).append(regs[i])
    out = []
    for members in groups.values():
        wts = np.array([r["area_px"] for r in members], dtype=float)
        where = np.sum([r["where_m"] * w for r, w in zip(members, wts)], axis=0) / wts.sum()
        lo = np.min([r["where_m"] - 0.5 * r["size_m"] for r in members], axis=0)
        hi = np.max([r["where_m"] + 0.5 * r["size_m"] for r in members], axis=0)
        per_look = {}
        for r in members:                    # one box per look: the union of that look's regions
            b = per_look.get(r["look_id"])
            per_look[r["look_id"]] = (r["view"], r["box_px"] if b is None else
                                      [min(b[1][0], r["box_px"][0]), min(b[1][1], r["box_px"][1]),
                                       max(b[1][2], r["box_px"][2]), max(b[1][3], r["box_px"][3])])
        errors = [r.get("err_m") or [0.0, 0.0] for r in members]
        err = [max(max(e[0] for e in errors), max(float(np.linalg.norm(r["where_m"] - where)) for r in members)),
               max(max(e[1] for e in errors), max(r["top_m"] for r in members) - min(r["top_m"] for r in members))]
        out.append({"err_m": err, "where_m": where, "top_m": max(r["top_m"] for r in members),
                    "height_m": float(np.median([r["height_m"] for r in members])),
                    "size_m": np.maximum(hi - lo, 0.0),
                    "seen_in": sorted(({"look_id": lid, "view": v, "box_px": b} for lid, (v, b) in per_look.items()),
                                      key=lambda x: x["view"]),
                    "area_px": int(wts.sum())})
    out.sort(key=lambda c: (-len({s["view"] for s in c["seen_in"]}), -c["area_px"]))
    return out


def draw(img, k, n, bearing_deg, boxes, look_id=""):
    """A copy of the frame with its look id, the view's number and bearing and each candidate's box and id."""
    out = np.ascontiguousarray(np.asarray(img)[:, :, :3]).copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(out, f"{look_id}  view {k + 1} of {n}  bearing {bearing_deg:.0f} deg", (6, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    for cid, (x0, y0, x1, y1) in boxes:
        cv2.rectangle(out, (int(x0), int(y0)), (int(x1), int(y1)), (255, 40, 40), 2)
        cv2.putText(out, cid, (int(x0) + 3, max(36, int(y0) + 16)), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 0), 2, cv2.LINE_AA)
    return out


def sheet(tiles, cols=4, width=320):
    """One contact sheet of the survey frames (already drawn), in view order, `cols` to a row."""
    if not tiles:
        return None
    h0, w0 = tiles[0].shape[:2]
    height = int(round(h0 * width / float(w0)))
    rows = int(math.ceil(len(tiles) / float(cols)))
    out = np.zeros((rows * height, min(cols, len(tiles)) * width, 3), np.uint8)
    for i, t in enumerate(tiles):
        r, c = divmod(i, cols)
        out[r * height:(r + 1) * height, c * width:(c + 1) * width] = cv2.resize(
            np.asarray(t)[:, :, :3], (width, height), interpolation=cv2.INTER_AREA)
    return out


def _heights(cam_height):
    """The shared plane sweep heights; its adjacent raised-plane step is this view's height error."""
    return [0.0] + list(np.linspace(RAISED_MIN_M, max(RAISED_MIN_M, TOP_FRAC * cam_height), N_PLANES - 1))
