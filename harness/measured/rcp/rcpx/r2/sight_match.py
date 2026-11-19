# SPDX-License-Identifier: AGPL-3.0-only
"""Pure image and ray functions of locate (package sight). Frames, pixels and
camera poses in; numbers out. No motion, no executor, no scene; nothing names a robot or a device.

    Cam(pc, Rc, f, w, h)                 a pinhole as the body model gives it: project(X), ray(u, v)
    features(img)                        chroma (two channels, a shading or exposure change keeps them) plus the
                                         grey gradient magnitude: what NCC compares
    pick_patch(img, px)                  the smallest of PATCH_SIZES_PX whose gradient energy passes GRAD_RMS_MIN
    dominant_dir(img, px, size)          the patch's dominant gradient direction (structure tensor), unit pixels
    band_search(...)                     the template slid along the epipolar curve of the naming ray in another view,
                                         EPI_BAND_PX each side, under the local affine of each depth (so a turned or
                                         nearer view still compares like with like)
    window_search(...)                   the same over a small window round a predicted pixel (view B)
    triangulate(rays)                    least-squares point of rays, each ray's miss, the span of the rays
    uncertainty(cams, pxs, sigmas, P)    the point's error from pixel errors through the geometry (largest axis)

A match is accepted when its peak correlation is NCC_PEAK or more and the best correlation more than NCC_RUNNER_PX
from it is at most NCC_RUNNER_FRAC of the peak (rcpx.r3_limits).
"""

import math

import cv2
import numpy as np

from rcpx import r3_limits as L

MM = 1000.0
RMS_SCALE = 0.25         # grad_rms reads the Sobel magnitude / 4 (grey levels per pixel)
GRAD_RMS_MIN = 6.0       # grey levels per pixel (RMS of the Sobel magnitude / 4): a patch with less is plain (the
                         # sim's plain cabinet faces read under 2, a part's edge in a 21 px patch over 15)
GRID_MAX = 31            # a template is sampled on at most this many points a side (a 61 px patch every 2 px)
CHROMA_SCALE = 255.0     # chroma channels in grey-level units, so they weigh like the gradient channel
CHROMA_DARK = 20.0       # added to r + g + b before the chroma division: a dark pixel has no reliable hue
GRAD_SCALE = 0.1         # the gradient channel: Sobel magnitude x this (a shape seen from another side changes its
                         # edges more than its colours: measured on the test parts, chroma carries the match)
CURVE_STEP_PX = 0.7      # the epipolar curve is sampled this finely in the searched view
GRAZING_COS = 0.17       # a surface seen within 80 deg of edge-on from the naming camera gives no usable map
NCC_SIGMA_FRAC = 0.4     # the correlation's Gaussian weight: this share of the patch's half side
CURVE_SAMPLES = 3000     # depths tried along the bracket before resampling by arc length
WINDOW_PX = 10           # view B's search window, each side of the predicted pixel
FD_PX = 0.5              # finite-difference step of the uncertainty propagation


class Cam:
    """A pinhole: centre pc (3,), rotation Rc (columns image right, image up, back along the view), focal f px."""

    def __init__(self, pc, Rc, f, w, h):
        self.pc, self.R = np.asarray(pc, dtype=float), np.asarray(Rc, dtype=float)
        self.f, self.w, self.h = float(f), int(w), int(h)

    def project(self, X):
        """(n, 2) pixels and (n,) depths of points X (n, 3); depth <= 0 is behind."""
        X = np.asarray(X, dtype=float).reshape(-1, 3)
        c = (X - self.pc) @ self.R
        dep = -c[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = self.w / 2.0 + self.f * c[:, 0] / dep
            v = self.h / 2.0 - self.f * c[:, 1] / dep
        return np.stack([u, v], axis=1), dep

    def ray(self, u, v):
        dc = np.array([(float(u) - self.w / 2.0) / self.f, -(float(v) - self.h / 2.0) / self.f, -1.0])
        dw = self.R @ dc
        return self.pc.copy(), dw / np.linalg.norm(dw)

    def rays(self, uv):
        """(n, 3) unit directions of the pixels uv (n, 2)."""
        uv = np.asarray(uv, dtype=float).reshape(-1, 2)
        dc = np.stack([(uv[:, 0] - self.w / 2.0) / self.f, -(uv[:, 1] - self.h / 2.0) / self.f,
                       -np.ones(len(uv))], axis=1)
        dw = dc @ self.R.T
        return dw / np.linalg.norm(dw, axis=1, keepdims=True)

    @property
    def axis(self):
        return -self.R[:, 2]


def _rgb(img):
    return np.asarray(img)[:, :, :3].astype(np.float32)


def grey(img):
    return cv2.cvtColor(np.ascontiguousarray(np.asarray(img)[:, :, :3]).astype(np.uint8), cv2.COLOR_RGB2GRAY
                        ).astype(np.float32)


def _grad(g):
    gb = cv2.GaussianBlur(g, (3, 3), 0)
    gx = cv2.Sobel(gb, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gb, cv2.CV_32F, 0, 1, ksize=3)
    return gx, gy


def features(img):
    """H x W x 3 float32: chroma r, chroma g (CHROMA_SCALE units) and the grey gradient magnitude x GRAD_SCALE."""
    rgb = _rgb(img)
    s = rgb.sum(axis=2) + CHROMA_DARK
    gx, gy = _grad(grey(img))
    mag = np.sqrt(gx * gx + gy * gy) * GRAD_SCALE
    return np.dstack([rgb[:, :, 0] / s * CHROMA_SCALE, rgb[:, :, 1] / s * CHROMA_SCALE, mag]).astype(np.float32)


def _box(img, px, size):
    H, W = img.shape[:2]
    r = size // 2
    u, v = int(round(px[0])), int(round(px[1]))
    return max(0, u - r), max(0, v - r), min(W, u + r + 1), min(H, v + r + 1)


def grad_rms(img, px, size):
    """RMS gradient (grey levels per pixel) of the size x size patch round px."""
    g = grey(img)
    x0, y0, x1, y1 = _box(g, px, size)
    gx, gy = _grad(g)
    m = np.sqrt(gx[y0:y1, x0:x1] ** 2 + gy[y0:y1, x0:x1] ** 2) * RMS_SCALE
    return float(np.sqrt(np.mean(m * m))) if m.size else 0.0


def pick_patch(img, px):
    """(size px, rms) of the smallest patch of PATCH_SIZES_PX whose gradient RMS passes GRAD_RMS_MIN; (None, rms of
    the largest) when none does (a plain patch: no depth from pictures)."""
    rms = 0.0
    for size in L.PATCH_SIZES_PX:
        rms = grad_rms(img, px, size)
        if rms >= GRAD_RMS_MIN:
            return int(size), round(rms, 2)
    return None, round(rms, 2)


def dominant_dir(img, px, size):
    """(unit gradient direction in pixels (u, v), coherence 0..1) of the patch: the structure tensor's main axis."""
    g = grey(img)
    x0, y0, x1, y1 = _box(g, px, size)
    gx, gy = _grad(g)
    gx, gy = gx[y0:y1, x0:x1], gy[y0:y1, x0:x1]
    a, b, c = float(np.sum(gx * gx)), float(np.sum(gx * gy)), float(np.sum(gy * gy))
    tr = a + c
    if tr < 1e-6:
        return np.array([1.0, 0.0]), 0.0
    disc = math.sqrt(max(0.0, (a - c) ** 2 / 4.0 + b * b))
    l1, l2 = tr / 2.0 + disc, tr / 2.0 - disc
    ang = 0.5 * math.atan2(2.0 * b, a - c)
    return np.array([math.cos(ang), math.sin(ang)]), round((l1 - l2) / max(l1 + l2, 1e-9), 3)


def grid(size):
    """(G, 2) template sample offsets (pixels of the naming image) for a size x size patch."""
    n = min(int(size), GRID_MAX)
    r = (size - 1) / 2.0
    s = np.linspace(-r, r, n)
    uu, vv = np.meshgrid(s, s)
    return np.stack([uu.reshape(-1), vv.reshape(-1)], axis=1).astype(np.float32)


def sample(F, centres, jacs, offs):
    """(N, G, C) samples of feature image F at centres (N, 2) + jacs (N, 2, 2) @ offs (G, 2); outside reads nan."""
    centres = np.asarray(centres, dtype=np.float32).reshape(-1, 2)
    jacs = np.asarray(jacs, dtype=np.float32).reshape(-1, 2, 2)
    pts = centres[:, None, :] + np.einsum("nij,gj->ngi", jacs, offs)
    mx, my = np.ascontiguousarray(pts[..., 0]), np.ascontiguousarray(pts[..., 1])
    out = cv2.remap(F, mx, my, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                    borderValue=(np.nan, np.nan, np.nan, np.nan))
    return out.reshape(len(centres), offs.shape[0], -1)


def ncc(T, P):
    """Centre-weighted normalised cross-correlation of template T (G, C) with each of P (N, G, C): the means taken
    per channel, every sample weighted by a Gaussian of NCC_SIGMA_FRAC of the patch's half side (the middle of a patch
    is the pointed part; its rim is often background that moves with the view). A candidate with any sample
    outside its frame reads -1."""
    T = np.asarray(T, dtype=np.float64)
    P = np.asarray(P, dtype=np.float64)
    G = T.shape[0]
    n = int(round(math.sqrt(G)))
    s = np.linspace(-1.0, 1.0, n)
    xx, yy = np.meshgrid(s, s)
    w = np.exp(-(xx ** 2 + yy ** 2) / (2.0 * NCC_SIGMA_FRAC ** 2)).reshape(-1)
    w = w / w.sum()
    bad = ~np.all(np.isfinite(P), axis=(1, 2))
    P = np.where(np.isfinite(P), P, 0.0)
    Tz = T - (w[:, None] * T).sum(axis=0, keepdims=True)
    Pz = P - (w[None, :, None] * P).sum(axis=1, keepdims=True)
    num = np.einsum("g,gc,ngc->n", w, Tz, Pz)
    den = np.sqrt(float(np.einsum("g,gc,gc->", w, Tz, Tz)) * np.einsum("g,ngc,ngc->n", w, Pz, Pz))
    out = num / np.maximum(den, 1e-12)
    out[bad] = -1.0
    return out


def _affine(cam0, camk, P, normal=None, du_dv=1.0):
    """(n, 2, 2): how one pixel of the naming image round each point P (n, 3) moves in camera k's image, for a
    surface through P with the given unit normal (None: facing camera 0). None when that plane is seen too near
    edge-on from camera 0 (GRAZING_COS) for the map to mean anything."""
    P = np.asarray(P, dtype=float).reshape(-1, 3)
    uv0, d0 = cam0.project(P)
    nrm = cam0.axis if normal is None else _unit3(normal)
    out = np.zeros((len(P), 2, 2))
    pk, _ = camk.project(P)
    for col, off in enumerate(((du_dv, 0.0), (0.0, du_dv))):
        rays = cam0.rays(uv0 + np.asarray(off)[None, :])
        den = rays @ nrm
        if float(np.min(np.abs(den))) < GRAZING_COS:
            return None
        depth = ((P - cam0.pc) @ nrm) / den
        X = cam0.pc[None, :] + rays * depth[:, None]
        xk, _ = camk.project(X)
        out[:, :, col] = (xk - pk) / du_dv
    return out


def _unit3(v):
    v = np.asarray(v, dtype=float)
    return v / max(float(np.linalg.norm(v)), 1e-12)


def surface_normals(cam0, d, radial):
    """The surfaces a pointed patch is tried on: facing camera 0, level (a top face), upright facing camera 0's
    side and upright facing the robot (a front face); each map is the local affine of that plane."""
    out = [None, np.array([0.0, 0.0, 1.0])]
    h = -np.array([d[0], d[1], 0.0])
    if float(np.linalg.norm(h)) > 1e-6:
        out.append(_unit3(h))
    r = -np.array([radial[0], radial[1], 0.0])
    if float(np.linalg.norm(r)) > 1e-6 and (len(out) < 3 or abs(float(_unit3(r) @ out[-1])) < 0.97):
        out.append(_unit3(r))
    return out


def curve(cam0, camk, o, d, t_lo, t_hi, margin):
    """The naming ray's depths [t_lo, t_hi] seen in camera k: (pts (N, 2), t (N,), normals (N, 2)), sampled every
    CURVE_STEP_PX, only where the point is in front of camera k and `margin` px inside its frame."""
    ts = np.geomspace(max(t_lo, 1e-3), max(t_hi, t_lo + 1e-3), CURVE_SAMPLES)
    X = o[None, :] + ts[:, None] * d[None, :]
    uv, dep = camk.project(X)
    ok = (dep > 1e-3) & (uv[:, 0] >= margin) & (uv[:, 0] <= camk.w - 1 - margin) & \
         (uv[:, 1] >= margin) & (uv[:, 1] <= camk.h - 1 - margin) & np.all(np.isfinite(uv), axis=1)
    if ok.sum() < 2:
        return np.zeros((0, 2)), np.zeros(0), np.zeros((0, 2))
    uv, ts = uv[ok], ts[ok]
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(uv, axis=0), axis=1))]
    n = max(2, int(seg[-1] / CURVE_STEP_PX) + 1)
    s = np.linspace(0.0, seg[-1], n)
    t = np.interp(s, seg, ts)
    pts = np.stack([np.interp(s, seg, uv[:, 0]), np.interp(s, seg, uv[:, 1])], axis=1)
    tan = np.gradient(pts, axis=0)
    tn = np.linalg.norm(tan, axis=1, keepdims=True)
    tan = np.where(tn > 1e-9, tan / np.maximum(tn, 1e-9), np.array([[1.0, 0.0]]))
    return pts, t, np.stack([-tan[:, 1], tan[:, 0]], axis=1)


def _peak(score, centres, runner_px=L.NCC_RUNNER_PX):
    i = int(np.argmax(score))
    best = float(score[i])
    far = np.linalg.norm(centres - centres[i], axis=1) > runner_px
    runner = float(np.max(score[far])) if far.any() else -1.0
    return i, best, runner


def _accept(best, runner):
    return best >= L.NCC_PEAK and runner <= L.NCC_RUNNER_FRAC * best


def _alone(best, runner):
    """A peak under NCC_PEAK that still stands alone (no rival within NCC_RUNNER_FRAC of it): a weak match. It
    counts only after back_check finds the pointed pixel again from it (a part whose surroundings change with the
    view, a thin part against the floor, correlates less than NCC_PEAK at the right place and nowhere else)."""
    return 0.0 < best < L.NCC_PEAK and runner <= L.NCC_RUNNER_FRAC * best


def _parabola(a, b, c):
    den = a - 2.0 * b + c
    return 0.0 if abs(den) < 1e-12 else max(-0.5, min(0.5, 0.5 * (a - c) / den))


def band_search(F0, Fk, cam0, camk, px, size, o, d, t_lo, t_hi, band=L.EPI_BAND_PX, normals=(None,)):
    """The naming patch (px, size) found in camera k along the naming ray's epipolar curve, band px each side, the
    template warped by the local affine of each surface in normals (the better correlation counts).
    {ok, px (2,), t (depth along d, m), ncc, runner, n} (ok False with why when nothing passes)."""
    offs = grid(size)
    T = sample(F0, [px], [np.eye(2)], offs)[0]
    pts, ts, nrm = curve(cam0, camk, o, d, t_lo, t_hi, margin=size / 2.0 + band + 1)
    if len(pts) < 3:
        return {"ok": False, "why": "the line of sight is not in this view", "n": 0}
    P = o[None, :] + ts[:, None] * d[None, :]
    bands = np.arange(-band, band + 1, 1.0)
    cen = (pts[:, None, :] + bands[None, :, None] * nrm[:, None, :]).reshape(-1, 2)
    score = None
    for n in normals:
        J = _affine(cam0, camk, P, n)
        if J is None:
            continue
        sc = ncc(T, sample(Fk, cen, np.repeat(J, len(bands), axis=0), offs))
        score = sc if score is None else np.maximum(score, sc)
    if score is None:
        return {"ok": False, "why": "no surface to compare on", "n": 0}
    k, best, runner = _peak(score, cen)
    i, j = divmod(k, len(bands))
    S = score.reshape(len(pts), len(bands))
    di = _parabola(S[i - 1, j], S[i, j], S[i + 1, j]) if 0 < i < len(pts) - 1 else 0.0
    dj = _parabola(S[i, j - 1], S[i, j], S[i, j + 1]) if 0 < j < len(bands) - 1 else 0.0
    ii = min(len(pts) - 1.0, max(0.0, i + di))
    i0 = int(math.floor(ii))
    w = ii - i0
    i1 = min(i0 + 1, len(pts) - 1)
    base = pts[i0] * (1 - w) + pts[i1] * w
    at = base + (bands[j] + dj) * nrm[i]
    t = float(ts[i0] * (1 - w) + ts[i1] * w)
    out = {"ok": _accept(best, runner), "px": at, "t": t, "ncc": round(best, 3), "runner": round(runner, 3),
           "n": int(len(cen)), "weak": _alone(best, runner)}
    if not out["ok"]:
        out["why"] = (f"no clear match (best {best:.2f}, the next best {runner:.2f} elsewhere)" if best >= L.NCC_PEAK
                      else f"no match passed {L.NCC_PEAK:g} (best {best:.2f})")
    return out


def back_check(F0, Fk, cam0, camk, px0, pxk, size, t_lo, t_hi, band=L.EPI_BAND_PX, normals=(None,)):
    """The match the other way round: the patch found at pxk in camera k is searched for in the naming image along
    ITS line of sight (the whole bracket, the same band). (ok, off px, ncc): ok when the best place in the naming
    image is the pointed pixel px0 to within the pointing error plus one match error (POINT_ERR_PX + MATCH_ERR_PX)
    and nothing else along that line rivals it. A patch that is not the pointed part (the floor behind it, a thing
    in front of it) finds its own place in the naming image, which is somewhere else."""
    ok_, dk = camk.ray(*pxk)
    m = band_search(Fk, F0, camk, cam0, pxk, size, ok_, dk, t_lo, t_hi, band=band, normals=normals)
    if m.get("px") is None:
        return False, None, None
    off = float(np.linalg.norm(np.asarray(m["px"], dtype=float) - np.asarray(px0, dtype=float)))
    good = (m["ok"] or m.get("weak", False)) and off <= L.POINT_ERR_PX + L.MATCH_ERR_PX
    return bool(good), round(off, 1), m.get("ncc")


def window_search(sources, Fb, camb, size, P, half=WINDOW_PX, normals=(None,)):
    """The patch found in camera b round the projection of P, half px each side. sources: [(F, cam, px)] the
    templates to try (the one re-cut in view A, and the naming one): each is warped into b by its own local affine
    for each surface in normals, and the best correlation counts at each pixel. {ok, px, ncc, runner, pred}."""
    offs = grid(size)
    pred, dep = camb.project([P])
    if not (dep[0] > 1e-3 and np.all(np.isfinite(pred))):
        return {"ok": False, "why": "the point is behind this view"}
    s = np.arange(-half, half + 1, 1.0)
    du, dv = np.meshgrid(s, s)
    cen = pred[0][None, :] + np.stack([du.reshape(-1), dv.reshape(-1)], axis=1)
    score = None
    for Fa, cama, pa in sources:
        T = sample(Fa, [pa], [np.eye(2)], offs)[0]
        for n in normals:
            J = _affine(cama, camb, [P], n)
            if J is None:
                continue
            sc = ncc(T, sample(Fb, cen, np.repeat(J, len(cen), axis=0), offs))
            score = sc if score is None else np.maximum(score, sc)
    if score is None:
        return {"ok": False, "why": "no surface to compare on"}
    k, best, runner = _peak(score, cen)
    S = score.reshape(len(s), len(s))
    r, c = divmod(k, len(s))
    dr = _parabola(S[r - 1, c], S[r, c], S[r + 1, c]) if 0 < r < len(s) - 1 else 0.0
    dc = _parabola(S[r, c - 1], S[r, c], S[r, c + 1]) if 0 < c < len(s) - 1 else 0.0
    at = cen[k] + np.array([dc, dr])
    out = {"ok": _accept(best, runner), "px": at, "ncc": round(best, 3), "runner": round(runner, 3),
           "pred": pred[0]}
    if not out["ok"]:
        out["why"] = f"no clear match near the crossing (best {best:.2f}, next {runner:.2f})"
    return out


def line_dist(cam, o, d, t, px, span=0.05):
    """Distance (px) of pixel px from the projection of the ray o + t' d near depth t (the line through the
    projections at t - span and t + span)."""
    a, _ = cam.project([o + (t - span) * d])
    b, _ = cam.project([o + (t + span) * d])
    a, b, p = a[0], b[0], np.asarray(px, dtype=float)
    ab = b - a
    n = float(np.linalg.norm(ab))
    if n < 1e-9:
        return float(np.linalg.norm(p - a))
    return abs(float(ab[0] * (p - a)[1] - ab[1] * (p - a)[0])) / n


def triangulate(rays):
    """(point (3,), misses (k,) m, span deg) of the least-squares point of rays [(o, d unit)]; point None when the
    rays are parallel."""
    A, b = np.zeros((3, 3)), np.zeros(3)
    for o, d in rays:
        P = np.eye(3) - np.outer(d, d)
        A += P
        b += P @ o
    if abs(float(np.linalg.det(A))) < 1e-12:
        return None, None, 0.0
    p = np.linalg.solve(A, b)
    miss = np.array([float(np.linalg.norm((p - o) - float((p - o) @ d) * d)) for o, d in rays])
    span = max(math.degrees(math.acos(max(-1.0, min(1.0, abs(float(a[1] @ c[1])))))) for a in rays for c in rays)
    return p, miss, span


def on_ray(o, d, rays):
    """The pointed spot lies on the naming ray (o, d) by definition: (point (3,), t (m along d), per-ray depths t_k,
    misses (m) of every ray including the naming one, span deg). Each other ray gives t_k, the point of the naming
    ray nearest it; the combined t is their least-squares mean (weights sin^2 of each ray's angle to the naming ray,
    the depth each one fixes); None when every other ray is parallel to it."""
    o, d = np.asarray(o, dtype=float), np.asarray(d, dtype=float)
    ts, ws = [], []
    for ok_, dk in rays:
        w = o - ok_
        b = float(d @ dk)
        den = 1.0 - b * b
        if den < 1e-9:
            continue
        ts.append((b * float(dk @ w) - float(d @ w)) / den)
        ws.append(den)
    if not ts:
        return None, None, [], None, 0.0
    t = float(np.dot(ws, ts) / np.sum(ws))
    p = o + t * d
    allr = [(o, d)] + list(rays)
    miss = np.array([float(np.linalg.norm((p - ok_) - float((p - ok_) @ dk) * dk)) for ok_, dk in allr])
    span = max(math.degrees(math.acos(max(-1.0, min(1.0, abs(float(a[1] @ c[1])))))) for a in allr for c in allr)
    return p, t, ts, miss, span


def uncertainty(cams, pxs, sigmas):
    """(largest axis mm, its unit direction) of the located point's error from independent pixel errors sigmas
    (px) on each camera's pixel (the first is the naming camera), propagated through the estimate itself (on_ray:
    the point on the naming ray nearest the other rays)."""
    def point(pxl):
        o, d = cams[0].ray(*pxl[0])
        return on_ray(o, d, [c.ray(*p) for c, p in zip(cams[1:], pxl[1:])])[0]

    base = [np.asarray(p, dtype=float) for p in pxs]
    P0 = point(base)
    if P0 is None:
        return float("inf"), np.array([0.0, 0.0, 1.0])
    C = np.zeros((3, 3))
    for k, s in enumerate(sigmas):
        for ax in (0, 1):
            pk = [b.copy() for b in base]
            pk[k][ax] += FD_PX
            P1 = point(pk)
            if P1 is None:
                continue
            g = (P1 - P0) / FD_PX
            C += (float(s) ** 2) * np.outer(g, g)
    w, V = np.linalg.eigh(C)
    return float(math.sqrt(max(0.0, w[-1]))) * MM, V[:, -1]


def crop(img, px, size, out=160, label=""):
    """A size x size crop round px scaled to out x out, a cross at the point and a label: one tile of the
    composite locate returns."""
    a = np.ascontiguousarray(np.asarray(img)[:, :, :3]).astype(np.uint8)
    H, W = a.shape[:2]
    r = max(8, int(size))
    u, v = int(round(px[0])), int(round(px[1]))
    pad = cv2.copyMakeBorder(a, r, r, r, r, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    c = pad[v:v + 2 * r + 1, u:u + 2 * r + 1]
    c = cv2.resize(c, (out, out), interpolation=cv2.INTER_NEAREST)
    m = out // 2
    cv2.line(c, (m - 8, m), (m + 8, m), (255, 255, 0), 1)
    cv2.line(c, (m, m - 8), (m, m + 8), (255, 255, 0), 1)
    cv2.rectangle(c, (0, 0), (out, 16), (0, 0, 0), -1)
    cv2.putText(c, label[:22], (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
    return c


def composite(tiles):
    """Tiles side by side (at most 640 px wide: three 160 px tiles leave room)."""
    if not tiles:
        return None
    return np.concatenate(tiles, axis=1)


__all__ = ["Cam", "on_ray", "back_check", "band_search", "composite", "crop", "curve", "dominant_dir", "features", "grad_rms", "grey",
           "line_dist", "ncc", "pick_patch", "sample", "triangulate", "uncertainty", "window_search"]
