# SPDX-License-Identifier: AGPL-3.0-only
"""Image side of the selfcheck package: how the picture of the surface under the arm moved between two looks.

measure(ref, img, inits) returns the homography (pixels of the reference look -> pixels of the other look) that the
flat surface in view followed, found by direct alignment: edge pixels of the reference are warped into the other
look and the warp is refined until colours agree (Gauss-Newton on the eight homography numbers, coarse to fine,
with a per-channel brightness fit). Why direct alignment and not corner tracks: a table seen from a wrist camera is
mostly large smooth marks, and an edge constrains the motion along its normal wherever it is.

What is not the surface is handled four ways:
- the robot's own body is masked from the body model (robot_mask: rays from the camera against the robot's geoms)
  and so is the mount the robot stands on (it is not at the floor's height);
- colour edges are followed alone when the look has enough of them: the arm's shadow moves with the arm and is an
  edge in lightness only; lightness edges join when colour is scarce (a grey surface);
- anything else off the surface (a device, clutter) moves differently, and a redescending weight (Tukey) drops
  what misses by more than a pixel at the fine level;
- each residual is divided by the local colour gradient, so it reads in pixels along the edge normal and strong
  edges do not outvote weak ones.
Nothing here reads simulator ground truth: inputs are frames and the body model.
"""

import cv2
import mujoco
import numpy as np

LEVELS = ((5.0, 12.0), (2.5, 5.0), (1.2, 1.5), (1.2, 0.8))   # (blur sigma px, Tukey cut-off px), coarse to fine
ITERS = 14                # Gauss-Newton steps per level at most
N_MAX = 5000              # edge pixels used
CELL = 3                  # one edge pixel per CELL x CELL block
EDGE_FRAC = 0.30          # an edge pixel's gradient is at least this share of the frame's strong gradients
BORDER = 6
MASK_STRIDE = 8           # the robot mask is ray cast on every MASK_STRIDE th pixel
MASK_GROW = 2             # and grown by this many coarse cells
MIN_INLIERS = 250         # fewer edge pixels than this following one motion: the look is not used
MIN_SPREAD = 0.12         # the inliers must cover at least this share of the frame (bounding box)
LAMBDA = 1e-4
COLOUR_EDGES_MIN = 1200   # fewer colour edge pixels than this: lightness edges are used too
GRAD_FLOOR = 2e-4         # a channel whose strong gradients are under this carries no edges (per pixel, 0 to 1)
BLACK = 0.06              # summed r + g + b (0 to 3) under which a pixel's colour shares are not trusted


def channels(img, sigma, light=True):
    """H x W x 3 float32 channels, blurred: two chromaticities (the red and green shares of the pixel's light,
    which a shadow or a dimmer lamp does not change; pulled to neutral only where the pixel is near black and its
    shares are noise) and, when `light` is set, the lightness."""
    a = np.ascontiguousarray(np.asarray(img)[:, :, :3]).astype(np.float32) / 255.0
    total = a.sum(axis=2)
    keep = total * total / (total * total + BLACK * BLACK)
    out = np.empty_like(a)
    for c in (0, 1):
        out[:, :, c] = (1.0 / 3.0) + (a[:, :, c] / np.maximum(total, 1e-4) - (1.0 / 3.0)) * keep
    out[:, :, 2] = (total / 3.0) if light else 0.0
    return cv2.GaussianBlur(out, (0, 0), sigma)


def _grads(C):
    return cv2.Sobel(C, cv2.CV_32F, 1, 0, ksize=3, scale=0.125), cv2.Sobel(C, cv2.CV_32F, 0, 1, ksize=3, scale=0.125)


def robot_mask(model, q, floor_z, mount_half=None, mount_margin=0.02):
    """True where the reference look may show the surface under the arm: not the robot's own body (rays from the
    body model's camera against its geoms, stopped at the floor) and not the mount the robot stands on."""
    m, d = model.mj, model.d
    pc, Rc = model.fk(q)["camera"]
    W, H, f = model.cam_w, model.cam_h, model.f_px
    us = np.arange(MASK_STRIDE // 2, W, MASK_STRIDE)
    vs = np.arange(MASK_STRIDE // 2, H, MASK_STRIDE)
    ok = np.ones((len(vs), len(us)), bool)
    gid = np.zeros(1, np.int32)
    for i, v in enumerate(vs):
        for j, u in enumerate(us):
            dc = np.array([(u - W / 2.0) / f, -(v - H / 2.0) / f, -1.0])
            dw = Rc @ dc
            dw /= np.linalg.norm(dw)
            t_floor = (floor_z - pc[2]) / dw[2] if dw[2] < -1e-6 else np.inf
            if not np.isfinite(t_floor):
                ok[i, j] = False          # this ray never meets the floor
                continue
            dist = mujoco.mj_ray(m, d, pc, dw, None, 1, -1, gid)
            if 0.0 <= dist < t_floor:
                ok[i, j] = False
                continue
            if mount_half is not None:
                p = pc + dw * t_floor
                if abs(p[0]) <= mount_half + mount_margin and abs(p[1]) <= mount_half + mount_margin:
                    ok[i, j] = False
    bad = (~ok).astype(np.uint8)
    k = 2 * MASK_GROW + 1
    bad = cv2.dilate(bad, np.ones((k, k), np.uint8))
    return cv2.resize(bad, (W, H), interpolation=cv2.INTER_NEAREST) == 0


class Reference:
    """The reference look, prepared once: per level its blurred channels and the edge pixels to follow. Colour
    edges alone are used when the look has enough of them (a shadow edge is a lightness edge and moves with the
    arm); a look without colour falls back to lightness too. light: None chooses, True or False forces."""

    def __init__(self, img, mask=None, light=None):
        self.h, self.w = np.asarray(img).shape[:2]
        mask = np.ones((self.h, self.w), bool) if mask is None else mask
        self.usable = mask.copy()
        b = BORDER
        self.usable[:b], self.usable[-b:], self.usable[:, :b], self.usable[:, -b:] = False, False, False, False
        self.light = False if light is None else bool(light)
        self._build(img)
        if light is None and self.n_edges < COLOUR_EDGES_MIN:
            self.light = True
            self._build(img)

    def _build(self, img):
        self.levels = []
        for sigma, cut in LEVELS:
            C = channels(img, sigma, self.light)
            gx, gy = _grads(C)
            g = np.sqrt(gx * gx + gy * gy)                                   # H x W x 3
            strong = np.array([max(GRAD_FLOOR, float(np.percentile(g[:, :, c], 97))) for c in range(3)])
            G = np.sqrt(((g / strong) ** 2).sum(axis=2))
            G[~self.usable] = 0.0
            hh, ww = (self.h // CELL) * CELL, (self.w // CELL) * CELL
            blocks = G[:hh, :ww].reshape(hh // CELL, CELL, ww // CELL, CELL).transpose(0, 2, 1, 3)
            blocks = blocks.reshape(hh // CELL, ww // CELL, CELL * CELL)
            arg = blocks.argmax(axis=2)
            best = blocks.max(axis=2)
            by, bx = np.nonzero(best >= EDGE_FRAC)
            v = by * CELL + arg[by, bx] // CELL
            u = bx * CELL + arg[by, bx] % CELL
            if len(u) > N_MAX:
                keep = np.linspace(0, len(u) - 1, N_MAX).round().astype(int)
                u, v = u[keep], v[keep]
            self.levels.append({"sigma": sigma, "cut": cut, "C": C, "uv": np.stack([u, v], 1).astype(np.float64),
                                "val": C[v, u].astype(np.float64), "norm": 1.0 / (g[v, u] + 0.3 * strong)})
        self.n_edges = int(len(self.levels[-1]["uv"]))


def _warp(H, uv):
    z = uv @ H[:2, :2].T + H[:2, 2]
    w = uv @ H[2, :2] + H[2, 2]
    return z / w[:, None], w


def _sample(img3, xy):
    mx = xy[:, 0].astype(np.float32).reshape(-1, 1)
    my = xy[:, 1].astype(np.float32).reshape(-1, 1)
    return cv2.remap(img3, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).reshape(len(xy), -1)


def _tukey(e, c):
    x = np.clip(np.abs(e) / c, 0.0, 1.0)
    return (1.0 - x * x) ** 2


def _refine(level, C, gx, gy, H, w_img, h_img, iters=ITERS):
    """Gauss-Newton on H at one level. Returns (H, weights, pixel residual per edge, inside mask)."""
    uv, val, norm, cut = level["uv"], level["val"], level["norm"], level["cut"]
    n = len(uv)
    w = np.zeros(n)
    e_px = np.full(n, np.inf)
    inside = np.zeros(n, bool)
    for _ in range(iters):
        xy, den = _warp(H, uv)
        inside = (xy[:, 0] >= 1) & (xy[:, 0] <= w_img - 2) & (xy[:, 1] >= 1) & (xy[:, 1] <= h_img - 2) & (den > 1e-9)
        got = _sample(C, xy)
        # brightness: reference = gain * look + bias per channel, from the pixels now trusted
        trust = inside & (w > 0.3) if w.any() else inside
        res = np.zeros((n, 3))
        gain = np.ones(3)
        for c in range(3):
            if trust.sum() > 50:
                A = np.stack([got[trust, c], np.ones(int(trust.sum()))], 1)
                sol = np.linalg.lstsq(A, val[trust, c], rcond=None)[0]
                gain[c] = float(np.clip(sol[0], 0.5, 2.0))
                bias = float(np.mean(val[trust, c] - gain[c] * got[trust, c]))
            else:
                bias = 0.0
            res[:, c] = gain[c] * got[:, c] + bias - val[:, c]
        e = res * norm                                            # about pixels along each channel's edge normal
        e_px = np.abs(e).max(axis=1)
        w = _tukey(e_px, cut) * inside
        if w.sum() < 30:
            break
        ix, iy = _sample(gx, xy), _sample(gy, xy)                 # n x 3 each
        # d(xy)/d(delta) for H (I + delta): delta[r, c] moves z by H[:, r] * x_c
        xh = np.concatenate([uv, np.ones((n, 1))], 1)
        J = np.zeros((n, 3, 8))
        k = 0
        for r in range(3):
            for cc in range(3):
                if r == 2 and cc == 2:
                    continue
                dz = np.outer(xh[:, cc], H[:, r])                 # n x 3
                dxy = (dz[:, :2] - xy * dz[:, 2:3]) / den[:, None]
                J[:, :, k] = (ix * dxy[:, 0:1] + iy * dxy[:, 1:2]) * gain * norm
                k += 1
        Jw = (J * w[:, None, None]).reshape(-1, 8)
        A = Jw.T @ J.reshape(-1, 8)
        b = Jw.T @ e.reshape(-1)
        scale = np.sqrt(np.maximum(np.diag(A), 1e-12))
        A = A / np.outer(scale, scale) + LAMBDA * np.eye(8)
        try:
            step = -np.linalg.solve(A, b / scale) / scale
        except np.linalg.LinAlgError:
            break
        D = np.eye(3)
        k = 0
        for r in range(3):
            for cc in range(3):
                if r == 2 and cc == 2:
                    continue
                D[r, cc] += step[k]
                k += 1
        H = H @ D
        H = H / H[2, 2]
        corners = np.array([[0.0, 0.0], [w_img, 0.0], [0.0, h_img], [w_img, h_img]])
        if float(np.abs(_warp(H @ np.linalg.inv(D), corners)[0] - _warp(H, corners)[0]).max()) < 0.003:
            break
    return H, w, e_px, inside


def measure(ref, img, inits):
    """The surface's motion from the reference look to `img`.

    inits: list of (tag, 3x3 homography guess). Each guess is refined at the coarsest level; the one most edge
    pixels follow goes on to the fine levels. Returns {"ok", "H", "tag", "inliers", "edges", "rms_px", "spread",
    "why"}; ok false means no single motion explains enough of the look (too dark, no texture, covered)."""
    imgs = {}
    for sigma, _ in LEVELS:
        if sigma not in imgs:
            C = channels(img, sigma, ref.light)
            imgs[sigma] = (C,) + _grads(C)
    lv0 = ref.levels[0]
    if len(lv0["uv"]) < MIN_INLIERS:
        return {"ok": False, "why": "too few edges on the surface in the reference look", "edges": len(lv0["uv"]),
                "inliers": 0, "H": None, "tag": None, "rms_px": None, "spread": 0.0}
    best = None
    for tag, H0 in inits:
        H0 = np.asarray(H0, float) / float(H0[2][2])
        H, w, e, inside = _refine(lv0, *imgs[lv0["sigma"]], H0, ref.w, ref.h)
        score = float(w.sum())
        if best is None or score > best[0]:
            best = (score, tag, H)
    _, tag, H = best
    w = e = inside = None
    for lv in ref.levels[1:]:
        H, w, e, inside = _refine(lv, *imgs[lv["sigma"]], H, ref.w, ref.h)
    lv = ref.levels[-1]
    inl = (w > 0.25)
    n_in = int(inl.sum())
    out = {"H": H, "tag": tag, "edges": int(len(lv["uv"])), "inliers": n_in, "rms_px": None, "spread": 0.0,
           "ok": False, "why": ""}
    if n_in < MIN_INLIERS:
        out["why"] = f"only {n_in} edge pixels follow one motion"
        return out
    pts = lv["uv"][inl]
    span = (pts.max(axis=0) - pts.min(axis=0))
    out["spread"] = round(float(span[0] * span[1]) / float(ref.w * ref.h), 3)
    out["rms_px"] = round(float(np.sqrt(np.mean(e[inl] ** 2))), 4)
    if out["spread"] < MIN_SPREAD:
        out["why"] = "the edges that follow one motion cover too little of the frame"
        return out
    out["ok"] = True
    out["inlier_uv"] = pts
    return out
