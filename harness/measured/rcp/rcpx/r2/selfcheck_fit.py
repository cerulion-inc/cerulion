# SPDX-License-Identifier: AGPL-3.0-only
"""The body fit of the selfcheck package: which body explains how the picture moved.

The hypothesis is theta = [offset per joint, gain per joint, camera mount turn r (3), camera mount shift s (3)]:
the joint's true value is q_ref + offset + gain x (reading - q_ref), and the camera sits at the body model's camera
pose moved by s and turned by r in its own frame. For the flat surface the robot stands on (z = floor in frame B)
each look pair then predicts one homography; the fit matches the predictions to the pictures.

Three steps, all on the same hypothesis:
1. geometric(): least squares of the predicted against the measured homographies (selfcheck_image.measure) at edge
   pixels that followed the surface. Cheap; it starts step 2.
2. direct(): the same parameters refined on the pictures themselves (every look's edge pixels warped by the
   predicted homography, Tukey weights), so an off-surface edge that bent one look's free homography cannot bend a
   body that must explain every look at once. It runs twice: with every gain free, to read each joint's sign and
   gain; then with the gains that are 1 held there, to read the offsets and the mount.
3. the spread of each estimate from the direct fit's normal matrix, widened for neighbouring pixels not being
   independent (CORRELATED_PIXELS).

What a camera cannot tell is found, not assumed (observable()): an offset whose effect on every prediction is nil
(a turn about the surface normal through the base, a joint beyond the camera) or the same as a mount change (the
joint that carries the camera) is held at its present value and reported as not seen.

What this fit takes on trust: the height of the surface (floor_z) and that it is level with frame B. The camera
cannot tell a surface a few mm higher from joint zeros that lower the camera by as much (measured on a right body:
1 mm of height error reads as about 0.15 deg on a 0.8 m arm and 0.45 deg on a 0.3 m arm). The only trace a wrong
height leaves is that the picture moves a little more or less than predicted, which is a gain off 1; the caller
applies nothing when a gain is off (GAIN_TOL).
No ground truth is read here; inputs are homographies, frames, joint readings and the body model.
"""

import math

import numpy as np
from scipy.optimize import least_squares

from rcpx.r2 import selfcheck_image as si

DEG = math.pi / 180.0
CORRELATED_PIXELS = 60.0   # edge pixels per independent measurement (edges are sampled every 3 px along smooth
                           # marks several blur widths long); widens every reported spread by its square root
SIGMA_FLOOR = (0.05, 0.001, 0.05, 0.3)   # least spread ever claimed: offset deg, gain, mount deg, shift mm
BLIND_PX = 0.02            # an effect under this many px (per degree, per 10 mm, per 1 % gain) is no effect
SAME_FRAC = 0.04           # an effect that other parameters reproduce to within this share is not separable
GRID = (9, 7)              # points per frame for the observability test
POINTS_PER_LOOK = 90       # edge pixels per look in the geometric fit
ROBUST_PX = 0.3
GAIN_SPREADS = 4.0  # PB-133: image gain significance


def need_px():
    return GAIN_SPREADS * ROBUST_PX / (GAIN_TOL * math.sqrt(si.N_MAX / CORRELATED_PIXELS))


def picture_gain(model, q, floor_z):
    body = Body(model, q, floor_z)
    grid = _grid(model)
    theta = body.theta0()
    gains = []
    for j in range(model.n):
        turned = np.asarray(q, float).copy()
        turned[j] += DEG
        shift = si._warp(body.homography(turned, theta), grid)[0] - grid
        gains.append(float(np.sqrt(np.mean(np.sum(shift ** 2, axis=1)))) / DEG)
    return np.asarray(gains)



def rot(v):
    v = np.asarray(v, dtype=float)
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


class Body:
    """Predicted homographies of the body model under theta, for one reference reading and a list of looks."""

    def __init__(self, model, q_ref, floor_z):
        self.model, self.n = model, model.n
        self.q_ref = np.asarray(q_ref, float).copy()
        self.floor_z = float(floor_z)
        f, W, H = float(model.f_px), float(model.cam_w), float(model.cam_h)
        # camera coordinates (x right, y up, looking along -z) to continuous pixels, then to pixel indices (a
        # pixel's centre is half a pixel inside it)
        K = np.array([[f, 0.0, -W / 2.0], [0.0, -f, -H / 2.0], [0.0, 0.0, -1.0]])
        T = np.array([[1.0, 0.0, -0.5], [0.0, 1.0, -0.5], [0.0, 0.0, 1.0]])
        self.K = T @ K
        self.Kinv = np.linalg.inv(self.K)
        self._cache = {}
        self.scale = np.concatenate([np.full(self.n, DEG), np.full(self.n, 0.01), np.full(3, DEG),
                                     np.full(3, 0.01)])    # one natural unit: 1 deg, 1 % gain, 1 deg, 10 mm

    def theta0(self):
        return np.concatenate([np.zeros(self.n), np.ones(self.n), np.zeros(6)])

    def pose(self, q_read, theta):
        n = self.n
        q = self.q_ref + theta[:n] + theta[n:2 * n] * (np.asarray(q_read, float) - self.q_ref)
        key = q.tobytes()
        hit = self._cache.get(key)
        if hit is None:
            pc, Rc = self.model.fk(q)["camera"]
            hit = (pc.copy(), Rc.copy())
            if len(self._cache) > 6000:
                self._cache.clear()
            self._cache[key] = hit
        pc, Rc = hit
        return pc + Rc @ theta[2 * n + 3:2 * n + 6], Rc @ rot(theta[2 * n:2 * n + 3])

    def homography(self, q_read, theta):
        """Pixel indices of the reference look -> pixel indices of the look at q_read, for the floor."""
        c0, R0 = self.pose(self.q_ref, theta)
        c1, R1 = self.pose(q_read, theta)
        n0 = R0[2]                                   # R0^T applied to the floor normal (0, 0, 1)
        dd = self.floor_z - c0[2]
        A = R1.T @ R0 + np.outer(R1.T @ (c0 - c1), n0) / dd
        H = self.K @ A @ self.Kinv
        return H / H[2, 2]

    def sees_floor(self, theta=None):
        """Share of the frame whose rays meet the floor in front of the camera within 20 camera heights."""
        theta = self.theta0() if theta is None else theta
        c0, R0 = self.pose(self.q_ref, theta)
        if c0[2] <= self.floor_z + 0.01:
            return 0.0
        W, H = self.model.cam_w, self.model.cam_h
        gx, gy = np.meshgrid(np.linspace(0, W - 1, 16), np.linspace(0, H - 1, 12))
        px = np.stack([gx.ravel(), gy.ravel(), np.ones(gx.size)], 1)
        d = (px @ self.Kinv.T) @ R0.T                 # K^-1 of a pixel is a camera ray with z = -1 (forward)
        d = d / np.linalg.norm(d, axis=1, keepdims=True)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (self.floor_z - c0[2]) / d[:, 2]
        return float(np.mean((d[:, 2] < -1e-6) & (t < 20.0 * (c0[2] - self.floor_z))))


def _grid(model):
    gx, gy = np.meshgrid(np.linspace(40, model.cam_w - 40, GRID[0]), np.linspace(40, model.cam_h - 40, GRID[1]))
    return np.stack([gx.ravel(), gy.ravel()], 1)


def _predict(body, looks, theta, pts):
    return np.concatenate([si._warp(body.homography(lk["q"], theta), p)[0].ravel() for lk, p in zip(looks, pts)])


def _jac(body, looks, theta, pts, cols):
    r0 = _predict(body, looks, theta, pts)
    J = np.zeros((len(r0), len(cols)))
    for k, c in enumerate(cols):
        e = 1e-4 * body.scale[c]
        tp, tm = theta.copy(), theta.copy()
        tp[c] += e
        tm[c] -= e
        J[:, k] = (_predict(body, looks, tp, pts) - _predict(body, looks, tm, pts)) / (2e-4)   # per natural unit
    return J


def observable(body, looks, theta=None):
    """Which parameters these looks can tell: {"free": bool mask over theta, "why": {index: text}}. Mount and
    gains first, then offsets from the base outward; an offset joins only if the others cannot imitate it."""
    n = body.n
    theta = body.theta0() if theta is None else theta
    grid = _grid(body.model)
    pts = [grid] * len(looks)
    J = _jac(body, looks, theta, pts, list(range(2 * n + 6)))
    rms = np.sqrt(np.mean(J * J, axis=0)) if len(looks) else np.zeros(2 * n + 6)
    free = np.zeros(2 * n + 6, bool)
    why = {}
    free[2 * n:] = rms[2 * n:] > BLIND_PX
    for j in range(n):
        if rms[n + j] > BLIND_PX:
            free[n + j] = True
        else:
            why[n + j] = "moving this joint does not move the camera over the surface"
    for j in range(n):
        if rms[j] <= BLIND_PX:
            why[j] = ("turning this joint does not change what the camera can measure (a turn about the surface "
                      "normal, or a joint beyond the camera)")
            continue
        B = J[:, np.flatnonzero(free)]
        col = J[:, j]
        if B.shape[1]:
            coef = np.linalg.lstsq(B, col, rcond=1e-10)[0]
            left = float(np.linalg.norm(col - B @ coef) / max(np.linalg.norm(col), 1e-12))
        else:
            left = 1.0
        if left < SAME_FRAC:
            why[j] = ("an offset here looks the same to the camera as a change of the camera mount: it is counted "
                      "in the camera estimate")
            continue
        free[j] = True
    return {"free": free, "why": why}


def geometric(body, looks, theta0, free):
    """Least squares of predicted against measured homographies at each look's own surface pixels (robust)."""
    pts, meas = [], []
    for lk in looks:
        p = lk["uv"]
        if len(p) > POINTS_PER_LOOK:
            p = p[np.linspace(0, len(p) - 1, POINTS_PER_LOOK).round().astype(int)]
        pts.append(p)
        meas.append(si._warp(lk["H"], p)[0].ravel())
    meas = np.concatenate(meas)
    cols = np.flatnonzero(free)
    sc = body.scale[cols]

    def theta_of(x):
        th = np.asarray(theta0, float).copy()
        th[cols] = theta0[cols] + x * sc
        return th

    def res(x):
        return _predict(body, looks, theta_of(x), pts) - meas

    sol = least_squares(res, np.zeros(len(cols)), loss="soft_l1", f_scale=ROBUST_PX, method="trf",
                        diff_step=1e-4, max_nfev=60)
    r = res(sol.x)
    per = []
    k = 0
    for p in pts:
        seg = r[k:k + 2 * len(p)].reshape(-1, 2)
        per.append(float(np.sqrt(np.mean(np.sum(seg * seg, axis=1)))))
        k += 2 * len(p)
    return theta_of(sol.x), per


def direct(body, ref, looks, theta0, free, level=-1, iters=12):
    """Refine theta on the pictures: every look's edge pixels warped by its predicted homography.

    looks carry "chan" = (C, gx, gy) of the look at the level's blur. Returns {"theta", "cov" (free block, natural
    units squared), "cols", "rms_px" per look, "used" per look, "s_px"}."""
    lv = ref.levels[level]
    uv, val, norm, cut = lv["uv"], lv["val"], lv["norm"], lv["cut"]
    cols = np.flatnonzero(free)
    sc = body.scale[cols]
    theta = np.asarray(theta0, float).copy()
    n, w_img, h_img = len(uv), ref.w, ref.h
    weights = [None] * len(looks)
    lam = 1e-3

    def look_terms(lk, th, w_prev, want_jac):
        H = body.homography(lk["q"], th)
        xy, den = si._warp(H, uv)
        inside = (xy[:, 0] >= 1) & (xy[:, 0] <= w_img - 2) & (xy[:, 1] >= 1) & (xy[:, 1] <= h_img - 2) & (den > 1e-9)
        C, gx, gy = lk["chan"]
        got = si._sample(C, xy)
        trust = inside if w_prev is None else (inside & (w_prev > 0.3))
        res = np.zeros((n, 3))
        gain = np.ones(3)
        for c in range(3):
            bias = 0.0
            if int(trust.sum()) > 50:
                A = np.stack([got[trust, c], np.ones(int(trust.sum()))], 1)
                s0 = np.linalg.lstsq(A, val[trust, c], rcond=None)[0]
                gain[c] = float(np.clip(s0[0], 0.5, 2.0))
                bias = float(np.mean(val[trust, c] - gain[c] * got[trust, c]))
            res[:, c] = gain[c] * got[:, c] + bias - val[:, c]
        e = res * norm
        w = si._tukey(np.abs(e).max(axis=1), cut) * inside
        if not want_jac:
            return e, w, None
        ix, iy = si._sample(gx, xy), si._sample(gy, xy)
        J = np.zeros((n, 3, len(cols)))
        for k, c in enumerate(cols):
            d = 1e-4 * body.scale[c]
            tp = th.copy()
            tp[c] += d
            dxy = (si._warp(body.homography(lk["q"], tp), uv)[0] - xy) / 1e-4      # px per natural unit
            J[:, :, k] = (ix * dxy[:, 0:1] + iy * dxy[:, 1:2]) * gain * norm
        return e, w, J

    def cost(th, ws):
        tot = 0.0
        for lk, w in zip(looks, ws):
            e, w2, _ = look_terms(lk, th, w, False)
            # the capped cost a Tukey weight minimises, summed over edge pixels
            x = np.clip(np.abs(e).max(axis=1) / cut, 0.0, 1.0)
            tot += float(np.sum(np.where(w2 > 0, 1.0 - (1.0 - x * x) ** 3, 1.0)))
        return tot

    A = np.eye(len(cols))
    for it in range(iters):
        A = np.zeros((len(cols), len(cols)))
        b = np.zeros(len(cols))
        for i, lk in enumerate(looks):
            e, w, J = look_terms(lk, theta, weights[i], True)
            weights[i] = w
            Jw = (J * w[:, None, None]).reshape(-1, len(cols))
            A += Jw.T @ J.reshape(-1, len(cols))
            b += Jw.T @ e.reshape(-1)
        c0 = cost(theta, weights)
        moved = False
        for _ in range(6):
            try:
                step = -np.linalg.solve(A + lam * np.diag(np.maximum(np.diag(A), 1e-9)), b)
            except np.linalg.LinAlgError:
                lam *= 10.0
                continue
            trial = theta.copy()
            trial[cols] += step * sc
            if cost(trial, weights) <= c0 + 1e-9:
                theta, lam, moved = trial, max(lam * 0.3, 1e-6), True
                break
            lam *= 10.0
        if not moved or float(np.max(np.abs(step))) < 2e-4:
            break
    rms, used, num, den_ = [], [], 0.0, 0.0
    A = np.zeros((len(cols), len(cols)))
    for i, lk in enumerate(looks):
        e, w, J = look_terms(lk, theta, weights[i], True)
        Jw = (J * w[:, None, None]).reshape(-1, len(cols))
        A += Jw.T @ J.reshape(-1, len(cols))
        emax = np.abs(e).max(axis=1)
        inl = w > 0.25
        used.append(int(inl.sum()))
        rms.append(float(np.sqrt(np.mean(emax[inl] ** 2))) if inl.any() else float("inf"))
        num += float(np.sum(w * emax ** 2))
        den_ += float(np.sum(w))
    s2 = num / max(den_, 1.0)
    try:
        cov = np.linalg.inv(A + 1e-12 * np.eye(len(cols))) * s2 * CORRELATED_PIXELS
    except np.linalg.LinAlgError:
        cov = np.full((len(cols), len(cols)), np.inf)
    return {"theta": theta, "cov": cov, "cols": cols, "rms_px": rms, "used": used, "s_px": math.sqrt(s2)}


def sigmas(body, fit):
    """Spread of every parameter in report units (deg, gain as a ratio, deg, mm); inf where it was held."""
    n = body.n
    out = np.full(2 * n + 6, np.inf)
    d = np.sqrt(np.maximum(np.diag(fit["cov"]), 0.0))
    for k, c in enumerate(fit["cols"]):
        if c < n:
            out[c] = max(d[k], SIGMA_FLOOR[0])
        elif c < 2 * n:
            out[c] = max(d[k] * 0.01, SIGMA_FLOOR[1])
        elif c < 2 * n + 3:
            out[c] = max(d[k], SIGMA_FLOOR[2])
        else:
            out[c] = max(d[k] * 10.0, SIGMA_FLOOR[3])
    return out


# ------------------------------------------------------------------------------------------------ the whole check
GAIN_TOL = 0.005           # a gain within this of 1 (or four of its own spreads) is 1: held there for the offsets.
                           # Kept tight on purpose: the offsets are read with the gains held at 1, so a picture that
                           # moves a few percent more or less than predicted (a joint's scale, or a surface that is
                           # not at the configured height) would otherwise be explained by joint zeros that are wrong
                           # by degrees. The gains are judged on the direct fit, whose spread holds in a view
                           # with much off-surface clutter; the geometric fit's gains wander by percents there.
SIGN_BAND = 0.2            # a joint that turns the wrong way turns -1 times its reading, give or take this
MIN_FLOOR_SHARE = 0.25


def _first_gain(body, lk, grid):
    """Gain that best scales the nominal prediction onto the measured motion of one look (least squares)."""
    th = body.theta0()
    fm = (si._warp(lk["H"], grid)[0] - grid).ravel()
    fp = (si._warp(body.homography(lk["q"], th), grid)[0] - grid).ravel()
    den = float(fp @ fp)
    return 1.0 if den < 1e-9 else float(fm @ fp) / den


def analyse(model, q_ref, ref_img, looks, floor_z, mount_half=None):
    """The whole camera check. looks: [{"joint": index, "q": reading (rad), "img": frame}] taken around q_ref.

    Returns {"ok": False, "why"} when the pictures cannot carry a check, else {"ok": True, "theta", "sigma"
    (natural units), "free", "why_held", "gain_state" per joint (ok | sign | gain | none | unseen), "look_rms_px",
    "looks_used", "looks_dropped", "edges", "floor_share", "light_edges"}."""
    n = model.n
    body = Body(model, q_ref, floor_z)
    share = body.sees_floor()
    if share < MIN_FLOOR_SHARE:
        return {"ok": False, "why": "the camera does not look at the surface the robot stands on from this pose",
                "floor_share": round(share, 2)}
    mask = si.robot_mask(model, q_ref, floor_z, mount_half)
    ref = si.Reference(ref_img, mask)
    if ref.n_edges < si.MIN_INLIERS:
        return {"ok": False, "why": "the surface in view carries too few marks to follow", "edges": ref.n_edges,
                "floor_share": round(share, 2)}
    th0 = body.theta0()
    used, dropped = [], []
    sigma_fine = ref.levels[-1]["sigma"]
    for lk in looks:
        j = lk["joint"]
        inits = []
        for g in (1.0, -1.0, 0.0):
            th = th0.copy()
            th[n + j] = g
            inits.append((g, body.homography(lk["q"], th)))
        m = si.measure(ref, lk["img"], inits)
        if not m["ok"]:
            dropped.append({"joint": j, "why": m["why"]})
            continue
        C = si.channels(lk["img"], sigma_fine, ref.light)
        used.append({"joint": j, "q": np.asarray(lk["q"], float), "H": m["H"], "uv": m["inlier_uv"],
                     "chan": (C,) + si._grads(C), "h_rms": m["rms_px"]})
    if len(used) < 2:
        return {"ok": False, "why": "no look could be matched to the reference look", "edges": ref.n_edges,
                "floor_share": round(share, 2), "looks_dropped": dropped}
    obs = observable(body, used)
    free = obs["free"].copy()
    grid = _grid(model)
    th = th0.copy()
    for j in range(n):
        gs = [_first_gain(body, lk, grid) for lk in used if lk["joint"] == j]
        if gs and free[n + j]:
            th[n + j] = float(np.mean(gs))
    th_a, _ = geometric(body, used, th, free)
    fit_a = direct(body, ref, used, th_a, free)      # everything free, on the pictures: the gains and their spread
    th_g, sig_a = fit_a["theta"], sigmas(body, fit_a)
    state = {}
    for j in range(n):
        if not free[n + j]:
            state[j] = "unseen"
            continue
        g = float(th_g[n + j])
        if abs(g - 1.0) <= max(GAIN_TOL, GAIN_SPREADS * sig_a[n + j]):
            state[j] = "ok"
        elif abs(g + 1.0) <= SIGN_BAND:
            state[j] = "sign"
        else:                                        # neither the described turn nor its mirror image
            state[j] = "none" if g < 0.3 else "gain"
    free_b = free.copy()
    th_b = th_a.copy()
    for j in range(n):
        if state[j] == "ok":
            free_b[n + j] = False
            th_b[n + j] = 1.0
    th_b, _ = geometric(body, used, th_b, free_b)
    fit = direct(body, ref, used, th_b, free_b)
    sig = sigmas(body, fit)
    for j in range(n):
        if state[j] == "ok":
            sig[n + j] = sig_a[n + j]
    return {"ok": True, "theta": fit["theta"], "theta_gains_free": th_g, "sigma": sig, "free": free_b,
            "why_held": obs["why"], "gain_state": state, "look_rms_px": fit["rms_px"], "looks_used": used,
            "look_pixels": fit["used"], "looks_dropped": dropped, "edges": ref.n_edges,
            "floor_share": round(share, 2), "light_edges": bool(ref.light), "s_px": fit["s_px"]}
