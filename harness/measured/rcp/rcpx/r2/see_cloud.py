# SPDX-License-Identifier: AGPL-3.0-only
"""A depth picture as points, and what stands at a pixel of it. Pure geometry on
numpy, OpenCV and scipy: no executor, no robot, no truth, no motion. rcpx/r2/see_spot.py is the service over it.

Units: metres, base frame. `up` is the unit vector against gravity (the stated support's normal). Pixel (u, v)
covers [u, u + 1) x [v, v + 1): its centre is at (u + MID, v + MID).

    View(fx, fy, cx, cy, w, h, pos, R)        a pinhole view; R: the camera's axes in the base frame as columns,
                                              image right, image down, along the view
    cloud(view, depth)                        H x W x 3 points of a depth picture (metres along the optical axis)
    hull_faces(points), inside_hulls(P, bodies, tol)   which points lie inside a body's convex hulls (the robot's own)
    extent(P, valid, view, u, v, up, hand, sigma_m)   what stands at pixel (u, v), see below
    component(P, valid, view, u, v, up, hand, sigma_m)   the points extent() takes as the thing at (u, v)
    march(view_a, u, v, view_r, P_r, valid_r, up)     where the ray of a pixel of ANOTHER view first goes under
                                                      the surface a view shows
    ray_at_height(o, d, up, z)                where a ray crosses a level height

Every size that decides something is the hand's own (hand = {"open_m": the widest opening, "pad_m": the pad's
width, "finger_m": how deep the fingers reach}), the pixel's own footprint at its range (range over the focal
length: the lateral grain of every figure), the depth's stated sigma, or a ratio of rcpx/bounds_ratios.py:
- a surface is LEVEL when its slope is under REL;
- two neighbouring pixels are ONE surface unless their ranges differ by more than BAND footprints, by more than
  GIVE of a pad, or by more than QUANT readings each BAND sigma off;
- the level a thing stands on is looked for within one and a half openings (1 + GIVE), the thing itself may run
  BAND openings: past that it is wider than any hand of this size spans ("border");
- a level counts when it shows a pad's own area (and at least QUANT x QUANT pixels);
- a thing is what stands over that level by more than BAND sigma or one footprint;
- bearings step so that the end of a chord half an opening long moves by one footprint;
- a picture is read no finer than the hand can use: the hand tells sizes to REL of a pad's width (the accepted
  relative error of the smallest size that decides), and telling a size takes QUANT points across it. A camera
  whose pixels stand closer than that at the pointed point is read at every k-th pixel, the pointed one among
  them (plain picking, nothing is averaged), so what one resolve costs is bounded by the hand's own shape and
  not by the camera's size or how close it stands. The pointed point and its edge are always read at the
  camera's own pixel; "grain_m" says the spacing the extent was read at;
- the top is the level of the highest points, not the highest point: the middle of the points within BAND sigma of
  it, found in QUANT rounds from the highest one (the highest of many noisy readings stands over the surface by
  two to three sigma; with an exact depth this is the highest point itself).

extent() returns {"kind": "thing" | "bare" | None, "point", "foot_m", "grain_m", "range_m", "pixel": [u, v] used,
"snapped",
"edge": {"other_point", "behind_m"} when another surface lies within one pixel, "why" when kind is None, and for a
thing: "base_z", "top_z", "top_near_z", "narrow_m", "bearing_rad", "wide_m", "centre_xy", "cut", "border",
"sigma_m", "points", "part", "hollow_m", "wall", "way_in_z", "slot"} with the meanings of C11. Levels and heights are along
`up`; centre_xy and every bearing are in the level axes axes_of(up) gives (the base x and y when up is the base z).
- hollow_m: the thing's rim (its points at the height of its top) stands over the pointed point on every side of
  it (no quarter turn around the point is free of rim): a rim around the point, not a higher part beside or
  over it (a handle's own upper arm stands over a point on its lower arm and is no rim);
- wall: the rim point nearest the pointed ray; "inward_xy" points from it to where the pointed ray passes the
  rim's height (a ray that lands right under the rim point gives no direction by its landing point);
- way_in_z: the top of what the pointed ray passes over on its way in (the wall on the lens's side of the pointed
  point), when that stands over the point: the height at which a place into the thing is entered. It is not
  top_near_z: a ray that lands on a tall back wall came in over the lower wall in front of it, and one that lands
  on the face of a shallow dish came in over a rim that stands further than a pad from the point;
- slot: {"span_m", "rim_z", "middle_xy"} when the ray came in ABOVE that wall and landed on a wall that rises over
  the point (the two walls of a slot whose opening the ray does not show a height for): the level middle between
  the wall it landed on and the wall in front of it, along the ray's track.
"""

import math

import cv2
import numpy as np

from rcpx.bounds_ratios import BAND, GIVE, QUANT, REL

MID = math.ldexp(1, -1)        # one half exactly: the middle of a pixel, of two ends, of a strip. A fact of geometry,
                               # not the ratio GIVE: a retune of GIVE must not move a lens's middle
DIAGONAL = math.hypot(1, 1)    # the step between two pixels that touch at a corner, in pixels
REACH = 1 + GIVE               # openings within which the level a thing stands on is looked for
RING = (-1, 0, 1)               # a pixel and its neighbours, along a row or a column
NEIGHBOURS = ((0, 1, 1.0), (1, 0, 1.0), (1, 1, DIAGONAL), (1, -1, DIAGONAL))   # (dv, du, step)


class View:
    def __init__(self, fx, fy, cx, cy, w, h, pos, R, name=""):
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)
        self.f = math.sqrt(self.fx * self.fy)
        self.w, self.h = int(w), int(h)
        self.pos = np.asarray(pos, dtype=float).reshape(-1)
        self.R = np.asarray(R, dtype=float).reshape(len(self.pos), len(self.pos))
        self.name = name

    def ray(self, x, y):
        """Unit direction through image point (x, y) (continuous: a pixel's centre is its index plus MID)."""
        d = self.R @ np.array([(x - self.cx) / self.fx, (y - self.cy) / self.fy, 1.0])
        return d / np.linalg.norm(d)

    def project(self, X):
        """Image coordinates (x, y) and the depth along the axis of points X (..., 3); the pixel is the floor."""
        Xc = (np.asarray(X, dtype=float) - self.pos) @ self.R
        z = Xc[..., -1]
        with np.errstate(divide="ignore", invalid="ignore"):
            return self.fx * Xc[..., 0] / z + self.cx, self.fy * Xc[..., 1] / z + self.cy, z

    def dirs(self):
        """H x W x 3 directions of all pixel centres, scaled so that the part along the view is 1."""
        us = (np.arange(self.w) + MID - self.cx) / self.fx
        vs = (np.arange(self.h) + MID - self.cy) / self.fy
        g = np.stack(np.broadcast_arrays(us[None, :], vs[:, None], np.ones((self.h, self.w))), -1)
        return g @ self.R.T

    def inside(self, x, y):
        return bool(np.isfinite(x) and np.isfinite(y) and 0.0 <= x < self.w and 0.0 <= y < self.h)


def cloud(view, depth):
    return view.pos + view.dirs() * np.asarray(depth, dtype=float)[..., None]


def axes_of(up):
    """(e1, e2): level axes for an up vector: the base x made level (the base y when up is along x), and up x e1."""
    up = np.asarray(up, dtype=float)
    for k in range(len(up)):
        e = np.zeros(len(up))
        e[k] = 1.0
        e1 = e - (e @ up) * up
        if np.linalg.norm(e1) > GIVE:
            break
    e1 = e1 / np.linalg.norm(e1)
    return e1, np.cross(up, e1)


def ray_at_height(o, d, up, z):
    """Where the ray from o along d crosses the level height z (None when it runs level or the crossing is behind)."""
    den = float(np.asarray(d) @ np.asarray(up))
    if abs(den) < np.finfo(float).eps:
        return None
    t = (float(z) - float(np.asarray(o) @ np.asarray(up))) / den
    return None if t <= 0.0 else np.asarray(o, dtype=float) + t * np.asarray(d, dtype=float)


def hull_faces(points):
    """A convex hull's faces (A, b) with A x + b <= 0 inside, and its box (lo, hi), from points in one frame."""
    from scipy.spatial import ConvexHull
    pts = np.asarray(points, dtype=float)
    eq = ConvexHull(pts).equations
    return eq[:, :-1], eq[:, -1], pts.min(0), pts.max(0)


def inside_hulls(P, bodies, tol):
    """Which points of P (..., 3) lie inside one of the convex hulls. bodies: [{"R", "p": the body's pose in the
    base frame, "A", "b", "lo", "hi": hull_faces of its hull in its own frame, "tol_m": the hull's own tolerance}].
    tol: how far outside a face still counts (the measure's own grain). A 3D test: nothing is grown in the picture."""
    flat = np.asarray(P, dtype=float).reshape(-1, P.shape[-1])
    own = np.zeros(len(flat), dtype=bool)
    for body in bodies:
        slack = tol + body["tol_m"]
        local = (flat - body["p"]) @ body["R"]
        near = np.flatnonzero(np.all((local >= body["lo"] - slack) & (local <= body["hi"] + slack), axis=1) & ~own)
        if len(near):
            own[near[np.all(local[near] @ body["A"].T + body["b"] <= slack, axis=1)]] = True
    return own.reshape(P.shape[:-1])


def _one_surface(rng_a, rng_b, foot, step, gap_min):
    return np.abs(rng_a - rng_b) < np.maximum(BAND * step * foot, gap_min)


def _components(mask, rng, ft, gap_min):
    """Labels (0: not in mask) of the pixels of mask joined through their 8 neighbours where both are one surface."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    H, W = mask.shape
    idx = np.arange(H * W).reshape(H, W)
    ii, jj = [], []
    for dv, du, step in NEIGHBOURS:
        a = (slice(0, H - dv), slice(max(0, -du), W - max(0, du)))
        b = (slice(dv, H), slice(max(0, du), W - max(0, -du)))
        ok = mask[a] & mask[b] & _one_surface(rng[a], rng[b], np.minimum(ft[a], ft[b]), step, gap_min)
        ii.append(idx[a][ok])
        jj.append(idx[b][ok])
    ii, jj = np.concatenate(ii), np.concatenate(jj)
    g = coo_matrix((np.ones(len(ii), np.int8), (ii, jj)), shape=(H * W, H * W))
    _, lab = connected_components(g, directed=False)
    lab = lab.reshape(H, W) + 1
    lab[~mask] = 0
    return lab


def bearings(hand, foot):
    """The bearings in [0, pi) a footprint is read over: the end of a chord half an opening long moves one footprint."""
    n = max(QUANT * QUANT, int(math.ceil(math.pi * MID * hand["open_m"] / foot)))
    return np.arange(n) * math.pi / n


def widths(xy, foot, th):
    """Extents of level points xy (N x 2): (narrow, its bearing, wide, centre xy). First over the bearings th, then
    again around the narrowest of them in steps that move the far end of the wide side by under one footprint (a
    long thin thing turns its narrow side by its length times the step). One footprint is added to every extent
    (points are pixel centres: half a footprint is missing on each side)."""
    def over(angles):
        D = np.stack([np.cos(angles), np.sin(angles)], 1)
        pr = xy @ D.T
        return D, pr.min(0), pr.max(0)

    D, lo, hi = over(th)
    k = int(np.argmin(hi - lo))
    step = th[1] - th[0] if len(th) > 1 else math.pi
    across = float(np.ptp(xy @ np.array([-D[k][1], D[k][0]])))
    fine = th[k] + np.linspace(-step, step, int(math.ceil(QUANT * step * MID * across / foot)) + QUANT)
    D, lo, hi = over(fine)
    k = int(np.argmin(hi - lo))
    d, n = D[k], np.array([-D[k][1], D[k][0]])
    wn = xy @ n
    c = d * (lo[k] + hi[k]) * MID + n * (wn.min() + wn.max()) * MID
    return float(hi[k] - lo[k]) + foot, float(fine[k] % math.pi), float(np.ptp(wn)) + foot, c


def top_of(z, sigma):
    """The level of the highest points (module docstring): no higher than the highest point."""
    top = float(np.max(z))
    for _ in range(QUANT):
        top = float(np.median(z[np.abs(z - top) <= BAND * sigma]))
    return top


def _run(t, gap):
    """(lo, hi) of the contiguous run of the sorted values t that holds the value nearest zero (gaps over gap cut)."""
    k = int(np.argmin(np.abs(t)))
    cut = np.flatnonzero(np.diff(t) > gap)
    lo = cut[cut < k].max() + 1 if (cut < k).any() else 0
    hi = cut[cut >= k].min() if (cut >= k).any() else len(t) - 1
    return int(lo), int(hi)


def chord(xy, z, at, th, half, gap, pad, sigma):
    """The narrowest chord of the level points xy through the point `at`: over the bearings th, the contiguous run of
    the points inside a strip of half width `half` along that bearing, nearest the point. Returns {"narrow_m",
    "bearing_rad", "centre_xy", "top_z"} or None when no strip holds a point."""
    best = None
    rel = xy - at
    for a in th:
        d, n = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        strip = np.abs(rel @ n) <= half
        if not strip.any():
            continue
        t = rel[strip] @ d
        order = np.argsort(t)
        lo, hi = _run(t[order], gap)
        length = float(t[order][hi] - t[order][lo]) + pad
        if best is None or length < best[0]:
            best = (length, float(a), at + d * float(t[order][hi] + t[order][lo]) * MID,
                    top_of(z[strip][order][lo:hi + 1], sigma))
    if best is None:
        return None
    return {"narrow_m": best[0], "bearing_rad": best[1], "centre_xy": [float(v) for v in best[2]], "top_z": best[3]}


def _levels(Q, ok, rng, ft, up, foot, sigma_m, gap_min):
    """Which pixels of the window show a level surface: the normal from the cloud's own differences over a span
    wide enough that the depth's noise tilts it by under half of what LEVEL allows, within LEVEL of up, away
    from every jump of the range."""
    sp = int(max(1, math.ceil(QUANT * sigma_m / (foot * REL))))
    level = np.zeros(ok.shape, dtype=bool)
    if not (Q.shape[0] > QUANT * sp + 1 and Q.shape[1] > QUANT * sp + 1):
        return level
    Qs = Q if sp == 1 else cv2.blur(Q, (sp, sp))
    dx = Qs[sp:-sp, QUANT * sp:] - Qs[sp:-sp, :-QUANT * sp]
    dy = Qs[QUANT * sp:, sp:-sp] - Qs[:-QUANT * sp, sp:-sp]
    nn = np.cross(dx, dy)
    ln = np.linalg.norm(nn, axis=-1)
    with np.errstate(divide="ignore", invalid="ignore"):
        cosine = np.abs(nn @ up) / ln
    jump = np.zeros(ok.shape, dtype=np.uint8)
    jump[:, :-1] |= ~_one_surface(rng[:, 1:], rng[:, :-1], np.minimum(ft[:, 1:], ft[:, :-1]), 1.0, gap_min)
    jump[:-1, :] |= ~_one_surface(rng[1:, :], rng[:-1, :], np.minimum(ft[1:, :], ft[:-1, :]), 1.0, gap_min)
    k = QUANT * sp + 1
    smooth = cv2.dilate(jump, np.ones((k, k), np.uint8)) == 0
    level[sp:-sp, sp:-sp] = (ok & smooth)[sp:-sp, sp:-sp] & (cosine > 1.0 / math.sqrt(1.0 + REL * REL))
    return level


def _base(zl, z0, margin, n_min):
    """The highest level below the pixel: from the highest height bin down, the first run of filled bins that
    holds n_min level pixels; its height is the middle of its fullest bin. None when there is none."""
    if not zl.size:
        return None
    hist, edges = np.histogram(zl, np.arange(zl.min() - margin, z0 + margin, margin))
    k = len(hist) - 1
    while k >= 0:
        if hist[k] == 0:
            k -= 1
            continue
        j = k
        while j - 1 >= 0 and hist[j - 1] > 0:
            j -= 1
        if hist[j:k + 1].sum() >= n_min:
            full = j + int(np.argmax(hist[j:k + 1]))
            sel = zl[(zl >= edges[full]) & (zl <= edges[full + 1])]
            return float(np.median(sel))
        k = j - 1
    return None


def _nearest_surface(P, valid, view, u, v, gap_min):
    """The pixel to read at (u, v): the nearer surface within one pixel when the pixel lies on a depth edge (an
    edge pixel can show the surface behind), and the other surface when one lies within one pixel at all:
    (u, v, snapped (du, dv) or None, {"other_point", "behind_m"} or None)."""
    H, W = valid.shape
    ring = [(u + du, v + dv) for dv in RING for du in RING
            if 0 <= u + du < W and 0 <= v + dv < H and valid[v + dv, u + du]]
    rng = {q: float(np.linalg.norm(P[q[1], q[0]] - view.pos)) for q in ring}
    here = (u, v)
    for q in ring:
        if rng[q] < rng[here] - max(BAND * rng[q] / view.f, gap_min):
            here = q
    far = max(ring, key=lambda q: rng[q])
    edge = None
    if rng[far] - rng[here] > max(BAND * rng[here] / view.f, gap_min):
        edge = {"other_point": [float(x) for x in P[far[1], far[0]]], "behind_m": rng[far] - rng[here]}
    return here[0], here[1], (None if here == (u, v) else [here[0] - u, here[1] - v]), edge


def stride(hand, foot):
    """How many of the camera's pixels one read point stands for (module docstring): 1 until QUANT of them fit
    between two points that stand REL of a pad's width over QUANT apart."""
    return max(1, int(math.floor(REL * hand["pad_m"] / (QUANT * foot))))


def _thin(P, valid, view, u, v, k):
    """Every k-th pixel of a picture, pixel (u, v) among them: (points, valid, the view of those pixels, u, v)."""
    ou, ov = u % k, v % k
    Pk, ok = P[ov::k, ou::k], valid[ov::k, ou::k]
    thin = View(view.fx / k, view.fy / k, (view.cx - ou - MID) / k + MID, (view.cy - ov - MID) / k + MID,
                Pk.shape[1], Pk.shape[0], view.pos, view.R, view.name)
    return Pk, ok, thin, u // k, v // k


def _resolve(P, valid, view, u, v, up, hand, sigma_m, snap=True, beside=False):
    """The first part of extent(): (out, S). out: the extent as far as it is known before the thing's own sizes
    are read (complete when S is None: no depth, no level below, bare). S: what extent() and component() read of
    the thing at the pixel: {"Q", "ok", "z", "rng", "ft": the window's points, validity, heights, ranges and
    footprints; "C": the thing's pixels in the window; "u", "v": the pixel read, in the window's picture; "win":
    (u0, u1, v0, v1) of the window in that picture; "edges": whether the window ends at the picture's own
    (top, bottom, left, right); "view": the view of that picture; "k", "ou", "ov": that picture holds every k-th
    pixel of the camera's, from (ou, ov); "foot", "margin", "gap_min", "base", "p0", "z0"}.
    snap False: the pixel is read as it is (no nearer surface within one pixel is taken for it). beside True: S is
    given for a pixel that lies on a level too ("C" then None), with "lab": every thing that stands over that level
    in the window, labelled (0: the level itself, the robot's own, no depth)."""
    H, W = valid.shape
    out = {"kind": None, "pixel": [int(u), int(v)]}
    if not (0 <= u < W and 0 <= v < H) or not valid[v, u]:
        out["why"] = "the pixel has no depth"
        return out, None
    gap_min = max(GIVE * hand["pad_m"], QUANT * BAND * sigma_m)
    us, vs, out["snapped"], edge = _nearest_surface(P, valid, view, u, v, gap_min)
    if snap:
        u, v = us, vs
    else:
        out["snapped"] = None
    if edge:
        out["edge"] = edge
    p0 = P[v, u]
    rng0 = float(np.linalg.norm(p0 - view.pos))
    foot = rng0 / view.f
    out.update(point=[float(x) for x in p0], foot_m=foot, range_m=rng0, pixel=[int(u), int(v)])
    k, ou, ov = stride(hand, foot), 0, 0
    if k > 1:                                   # finer than the hand can use: every k-th pixel is read from here on
        ou, ov = u % k, v % k
        P, valid, view, u, v = _thin(P, valid, view, u, v, k)
        H, W = valid.shape
        foot = rng0 / view.f
    out["grain_m"] = foot
    r_near = int(math.ceil(REACH * hand["open_m"] / foot)) + QUANT
    r = int(math.ceil(BAND * hand["open_m"] / foot)) + QUANT
    u0, u1, v0, v1 = max(0, u - r), min(W, u + r + 1), max(0, v - r), min(H, v + r + 1)
    Q, ok = P[v0:v1, u0:u1], valid[v0:v1, u0:u1]
    z = Q @ up
    rng = np.linalg.norm(Q - view.pos, axis=-1)
    ft = rng / view.f
    level = _levels(Q, ok, rng, ft, up, foot, sigma_m, gap_min)
    vv, uu = np.indices(ok.shape)
    near_px = (np.abs(vv - (v - v0)) <= r_near) & (np.abs(uu - (u - u0)) <= r_near)
    margin = max(BAND * sigma_m, foot)
    z0 = float(z[v - v0, u - u0])
    base = _base(z[level & near_px & (z < z0 - margin)], z0, margin, max(QUANT * QUANT, (hand["pad_m"] / foot) ** 2))
    if base is None:
        n_min = max(QUANT * QUANT, (hand["pad_m"] / foot) ** 2)
        on = _base(z[level & near_px & (z < z0 + margin)], z0 + margin, margin, n_min) if beside else None
        if level[v - v0, u - u0] or (on is not None and abs(z0 - on) <= margin):
            out.update(kind="bare", base_z=z0)      # (beside: a pixel next to a jump lies on the level beside it)
        else:
            out["why"] = "no level surface of a pad's size is seen below the pixel"
        if not (beside and out["kind"] == "bare"):
            return out, None
        base = z0
    lab = _components(ok & (z > base + margin), rng, ft, gap_min)
    me = lab[v - v0, u - u0]
    if me == 0:
        out.update(kind="bare", base_z=base)
        if not beside:
            return out, None
    return out, {"Q": Q, "ok": ok, "z": z, "rng": rng, "ft": ft, "C": lab == me if me else None, "lab": lab,
                 "u": u, "v": v,
                 "win": (u0, u1, v0, v1), "edges": (v0 == 0, v1 == H, u0 == 0, u1 == W), "view": view, "k": k,
                 "ou": ou, "ov": ov, "foot": foot, "margin": margin, "gap_min": gap_min, "base": base, "p0": p0,
                 "z0": z0}


def component(P, valid, view, u, v, up, hand, sigma_m):
    """The points extent() takes as the thing at pixel (u, v), or None when extent() makes
    out no thing there (no depth, no level of a pad's size below the pixel, or the pixel lies on that level).
    {"pts": N x 3 (base frame), "xy": N x 2 (the level axes of axes_of(up)), "z": N heights along up, "mask": the
    thing's pixels in the picture given (H x W; every k-th pixel where the picture is finer than the hand can
    use: "stride"), "top": the level of its highest points, "base": the level it stands on, "foot": the spacing
    the points were read at, "window": [u0, u1, v0, v1] of the picture given that was read, "stride": k,
    "pixel": [u, v] the pixel read (the nearer surface within one pixel of the one given)}. The thing may run on
    past the window (extent()'s "border")."""
    up = np.asarray(up, dtype=float)
    out, S = _resolve(P, valid, view, u, v, up, hand, sigma_m)
    if S is None:
        return None
    e1, e2 = axes_of(up)
    pts, zc = S["Q"][S["C"]], S["z"][S["C"]]
    k, (u0, u1, v0, v1) = S["k"], S["win"]
    mask = np.zeros(valid.shape, dtype=bool)
    vv, uu = np.nonzero(S["C"])
    mask[S["ov"] + (vv + v0) * k, S["ou"] + (uu + u0) * k] = True
    return {"pts": pts, "xy": pts @ np.stack([e1, e2], 1), "z": zc, "mask": mask, "top": top_of(zc, sigma_m),
            "base": S["base"], "foot": S["foot"], "stride": k, "pixel": list(out["pixel"]),
            "window": [S["ou"] + u0 * k, min(valid.shape[1], S["ou"] + u1 * k), S["ov"] + v0 * k,
                       min(valid.shape[0], S["ov"] + v1 * k)]}


def extent(P, valid, view, u, v, up, hand, sigma_m, ray=None, snap=True):
    """What stands at pixel (u, v) of a view whose points are P (H x W x 3) where valid (module docstring). ray:
    (origin, unit direction) of the pointed ray when it is another view's (default: this view's ray to the point).
    snap False: the pixel is read as it is (no nearer surface within one pixel is taken for it)."""
    up = np.asarray(up, dtype=float)
    out, S = _resolve(P, valid, view, u, v, up, hand, sigma_m, snap=snap)
    if S is None:
        return out
    Q, ok, z, C, view = S["Q"], S["ok"], S["z"], S["C"], S["view"]
    foot, margin, p0, z0, base = S["foot"], S["margin"], S["p0"], S["z0"], S["base"]
    grown = cv2.dilate(C.astype(np.uint8), np.ones((len(RING), len(RING)), np.uint8)) > 0
    at_top, at_bottom, at_left, at_right = S["edges"]
    at_edge = (at_top and C[0, :].any()) or (at_bottom and C[-1, :].any()) or (at_left and C[:, 0].any()) or \
        (at_right and C[:, -1].any())
    e1, e2 = axes_of(up)
    A = np.stack([e1, e2], 1)
    pts, zc = Q[C], z[C]
    xy, xy0 = pts @ A, p0 @ A
    top = top_of(zc, sigma_m)
    near = np.linalg.norm(xy - xy0, axis=1) <= hand["pad_m"]
    top_near = top_of(zc[near], sigma_m) if near.any() else top
    slab = zc >= top - hand["finger_m"]
    th = bearings(hand, foot)
    nar, bear, wide, c = widths(xy[slab], foot, th)
    out.update(kind="thing", base_z=base, top_z=top, top_near_z=top_near, narrow_m=nar, bearing_rad=bear, wide_m=wide,
               centre_xy=[float(x) for x in c], points=int(C.sum()), sigma_m=max(sigma_m, MID * foot),
               border=bool(C[0, :].any() or C[-1, :].any() or C[:, 0].any() or C[:, -1].any()),
               cut=bool(at_edge or (grown & ~ok).any()))
    gap = BAND * foot
    by_point = zc >= top_near - hand["finger_m"]
    out["part"] = chord(xy[by_point], zc[by_point], xy0, th, MID * hand["pad_m"], gap, foot, sigma_m)
    out["hollow_m"], out["wall"] = None, None
    o = view.pos if ray is None else np.asarray(ray[0], dtype=float)
    d = p0 - o if ray is None else np.asarray(ray[1], dtype=float)
    d = d / np.linalg.norm(d)
    higher = (zc >= top - margin) & (zc > z0 + margin)       # the thing's rim (the points at its top) over the point
    if higher.any() and _surrounded(xy[higher] - xy0):
        out["hollow_m"] = top - z0
        at_top = ray_at_height(o, d, up, top)
        aim = xy0 if at_top is None else at_top @ A
        out["wall"] = _wall(pts, xy, zc, aim, np.asarray(c), o, d, top, margin, slab, hand, gap, foot)
    out["way_in_z"], out["slot"] = None, None
    came = _way_in(xy, zc, xy0, z0, -(d @ A), hand, foot, margin, sigma_m)
    if came is not None and came[0] > z0 + margin:
        out["way_in_z"] = came[0]
    elif came is not None and top_near > z0 + margin:
        g = -(d @ A) / np.linalg.norm(d @ A)
        out["slot"] = {"span_m": came[1], "rim_z": came[0], "middle_xy": [float(x) for x in xy0 + g * came[1] * MID]}
    return out


def _surrounded(rel):
    """Whether level directions rel (N x 2, from a point) stand on every side of it: no quarter turn around the
    point is free of them."""
    ang = np.sort(np.arctan2(rel[:, 1], rel[:, 0]))
    gaps = np.diff(np.concatenate([ang, ang[:1] + math.tau]))            # around the whole turn, back to the first
    return bool(len(ang) > 1 and float(gaps.max()) < math.pi * MID)      # a quarter turn


def _wall(pts, xy, zc, aim, centre, o, d, top, margin, slab, hand, gap, foot):
    """The wall of a hollow thing nearest the pointed ray: the rim point (a point at the thing's top) nearest the
    ray from o along d, the level direction from it to where the pointed ray passes the rim's height (`aim`; to
    the thing's middle when that is no further than one footprint from the wall point), and the top slab's
    thickness along that direction through it."""
    rim = zc >= top - margin
    off = pts[rim] - o
    k = int(np.argmin(np.linalg.norm(off - np.outer(off @ d, d), axis=1)))
    at, at_xy = pts[rim][k], xy[rim][k]
    inward = aim - at_xy
    if np.linalg.norm(inward) <= foot:
        inward = centre - at_xy
    n = float(np.linalg.norm(inward))
    if n <= np.finfo(float).eps:
        return None
    inward = inward / n
    rel = xy[slab] - at_xy
    strip = np.abs(rel @ np.array([-inward[1], inward[0]])) <= MID * hand["pad_m"]
    t = np.sort(rel[strip] @ inward)
    lo, hi = _run(t, gap)
    return {"point": [float(x) for x in at], "inward_xy": [float(x) for x in inward],
            "thick_m": float(t[hi] - t[lo]) + foot}


def _way_in(xy, zc, xy0, z0, back, hand, foot, margin, sigma):
    """What the pointed ray passed over on its way in: (the top of the wall on the lens's side of the pointed
    point, how far that wall stands from the point along the ray's track), or None when nothing of the thing
    stands on that side (or the ray comes down near plumb). Along the track on the level, from the pointed point
    back toward the lens (`back`, level), in a strip one pad wide, the thing's height is read footprint by
    footprint. The surface the ray landed on is left (the first steps that stand over the point: a wall it
    landed on). Then the wall is where the thing first stands over the point again, or rises over the lowest
    height read since, or shows again after a stretch the lens cannot see (what hides a floor from the lens is
    the wall on the lens's side of it); its top is read while the heights climb."""
    n = float(np.linalg.norm(back))
    if n <= REL:                              # a ray within REL of plumb passes over nothing on its way in
        return None
    g = np.asarray(back, dtype=float) / n
    rel = xy - xy0
    along = rel @ g
    strip = (np.abs(rel @ np.array([-g[1], g[0]])) <= MID * hand["pad_m"]) & (along > 0.0)
    if not strip.any():
        return None
    bins = np.floor(along[strip] / foot).astype(int)
    prof = np.full(int(bins.max()) + 1, -np.inf)
    np.maximum.at(prof, bins, zc[strip])
    k = 0
    while k < len(prof) and prof[k] > z0 + margin:
        k += 1
    low, hidden, start = np.inf, 0, None
    for j in range(k, len(prof)):
        if not np.isfinite(prof[j]):
            hidden += 1
            continue
        if prof[j] > z0 + margin or prof[j] > low + margin or (hidden > BAND and j > k):
            start = j
            break
        low, hidden = min(low, prof[j]), 0
    if start is None:
        return None
    end, top, hidden = start, prof[start], 0
    while end + 1 < len(prof) and hidden <= BAND:
        nxt = prof[end + 1]
        if not np.isfinite(nxt):
            hidden += 1
        elif nxt < top - margin:
            break
        else:
            top, hidden = max(top, nxt), 0
        end += 1
    inside = strip.copy()
    inside[strip] = (bins >= start) & (bins <= end)
    return top_of(zc[inside & (zc >= top - margin)], sigma), start * foot


def march(view_a, x, y, view_r, P_r, valid_r, up, step):
    """The first point along the ray of image point (x, y) of view a that lies at or under the surface view r
    shows (P_r where valid_r; that surface is read as heights along up, which holds for a view that looks down on
    it). step: the spacing of the ray's samples. Returns (point, (u, v) of view r) or (None, None)."""
    up = np.asarray(up, dtype=float)
    d = view_a.ray(x, y)
    far = float(np.max(np.linalg.norm(P_r[valid_r] - view_a.pos, axis=-1))) if valid_r.any() else 0.0
    if far <= step:
        return None, None
    ts = np.arange(step, far + step, step)
    X = view_a.pos + ts[:, None] * d
    px, py, zc = view_r.project(X)
    with np.errstate(invalid="ignore"):
        inside = (zc > 0) & (px >= 0) & (px < view_r.w) & (py >= 0) & (py < view_r.h)
    ui = np.where(inside, np.floor(np.nan_to_num(px)), 0).astype(int)
    vi = np.where(inside, np.floor(np.nan_to_num(py)), 0).astype(int)
    seen = inside & valid_r[vi, ui]
    under = seen & ((X @ up) <= P_r[vi, ui] @ up)
    if not under.any() or under[0]:
        return None, None
    k = int(np.argmax(under))
    return X[k], (int(ui[k]), int(vi[k]))
