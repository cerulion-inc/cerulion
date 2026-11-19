# SPDX-License-Identifier: AGPL-3.0-only
"""Pure image functions of the witness package: two frames from the same camera pose in, what
changed out. No model, no scene, no robot: images (H x W x 3 uint8 RGB), masks and numbers.

    align(a, b, valid)               ECC (Euclidean) on grey half-size frames, the own body masked out: the warp
                                     that lines b up on a, its shift at the image centre and its turn; ok when the
                                     shift is ALIGN_SHIFT_PX or less and the turn ALIGN_TURN_DEG or less
    warp(b, align_result)            b lined up on a (and the mask of pixels b covers there)
    gain(a, b, valid)                the median of b over a on low-gradient pixels (the light's change)
    threshold(a, b, valid)           (t, m): t = max(DIFF_MIN, DIFF_SCALE x m), m the median grey of both frames
    change_mask(a, b, t, valid)      a pixel changed when its largest channel difference to EVERY pixel within
                                     DIFF_NEAR_PX in the other frame is over t, both ways; then a 3 x 3 opening
    components(mask)                 connected changes of COMPONENT_MIN_PX or more, largest first
    describe(comp, a, b, ...)        box, area, colour before and after, brighter or darker, glowing, kind
    region(shape, spec)              a target's region and ring masks (a disc, or a box)
    judge(...)                       changed yes | no | unclear at one region (the rule below)
    patch_offset(a, b, box)          where a's patch is found again in b (NCC), within MOVED_SEARCH_PX
    device_offset(a, b, box, mask)   the same for a whole device's outline, also turned a few degrees, with the
                                     correlation where it was (the device stayed) beside the best (it moved)
    moved_shift(a, b, mask, t)       the shift that explains the most changed pixels (a part that moved)
    static_mask(a, b, valid)         what did not change even before alignment (ECC lines the frames up on it)
    gain_spread(a, b, valid, g)      how even the gain is over the frame (an exposure, or the light)
    triptych(a, b, mask, marks)      before | after | changes, TRIPTYCH_W wide

The rule at a target: changed yes when the region's changed share is CHANGED_SHARE or more and at least
CHANGED_RING_MULT x the ring's share + CHANGED_RING_ADD (or, for a part LARGE_PART_PX or more across, whose own
motion can fill a ring narrower than itself, at least that over the share of the frame beyond the ring: the
change is still local to the part, not the whole frame); no (a "solid no") when the share is NO_SHARE or less, the
part is SOLID_MIN_PX or more across and the frames are not dim (median grey at rcpx.perception.dim_level of the
camera's own noise or more); anything else unclear. So in dim frames or on a small region the answer is yes or
unclear, never no.
Every number is in rcpx.r3_limits with its reason, except the few defined here with theirs.
"""

import math

import cv2
import numpy as np

from rcpx import colour_words
from rcpx import r3_limits as L
from rcpx import perception

ECC_ITERS, ECC_EPS = 80, 1e-5   # ECC stops after this many iterations or this small a step (half-size pixels)
ECC_BLUR = 5                    # ECC's own Gaussian pre-filter (odd, half-size pixels)
STRETCH_PCT = (2.0, 98.0)       # ECC sees each frame stretched between these grey percentiles (dim frames)
LARGE_PART_PX = 2 * L.RING_PX   # a part this wide or more can fill its own ring when it moves (the large rule)
STATIC_GROW_PX = 15             # ECC leaves out what changed before alignment, grown this much ...
STATIC_MIN_FRAC = 0.3           # ... unless that leaves under this share of the usable pixels
GAIN_UNIFORM_IQR = 0.08         # a gain this even over the frame (the ratio's interquartile spread), with both
                                # frames still ok, is the camera's own exposure (auto exposure scales every pixel
                                # alike and holds the frame bright), not the light going down
MOVED_EXPLAIN_FRAC = 0.6        # a shift that explains this share of the changed pixels is a move (moved_px)
MOVED_STEP_PX = 6               # the move search: this coarse step over MOVED_SEARCH_PX, then 1 px around the best
MOVED_SAMPLE_MAX = 1200         # changed pixels sampled for the move search (evenly, so it is deterministic)
MOVED_COARSE_EVERY = 4          # the coarse pass of the move search reads every 4th sample, the fine pass all
DEVICE_TURNS_DEG = (-8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0)   # device_moved also tries these turns:
                                # a device someone knocks or moves goes a few cm and turns a few degrees
DEVICE_TURN_MIN_DEG = 4.0       # a device found turned this much has moved even with a small offset
GAIN_GRAD_PCT = 50.0            # the light's gain is read on the flatter half of the pixels (both frames) ...
GAIN_GREY_LO, GAIN_GREY_HI = 8.0, 250.0   # ... that are neither black nor saturated, where a ratio says nothing
MIN_VALID_FRAC = 0.05           # under this share of usable pixels a frame pair cannot be read at all
MOVED_MIN_PX = 2.0              # a patch found again this far off has moved (under it: the same place)
FLAT_STD = 2.0                  # a patch flatter than this (grey std) cannot be found again by correlation
CONTRAST_GROW_PX = 10           # appeared or gone: a change is compared with the ring this wide around it
DESCRIBE_MAX = 12               # components described in full (the largest); the rest are counted
PANEL_DIM = 0.45                # the changes panel shows the after frame dimmed to this ...
CHANGE_RGB = (255, 0, 255)      # ... with changed pixels painted this colour
MARK_RGB = (255, 255, 0)        # target outlines on the changes panel


def rgb(img):
    a = np.asarray(img)
    if a.ndim == 2:
        a = np.stack([a, a, a], axis=-1)
    return np.ascontiguousarray(a[..., :3]).astype(np.uint8)


def grey(img):
    return cv2.cvtColor(rgb(img), cv2.COLOR_RGB2GRAY).astype(np.float32)


# ---------------------------------------------------------------- alignment and light
def _stretch(g, sel):
    """A grey frame stretched so its STRETCH_PCT percentiles (over sel) span 0..255: ECC on a dim, flat frame
    otherwise wanders off (p1b/08's dim pair, grey std 12 of 255)."""
    lo, hi = np.percentile(g[sel], STRETCH_PCT) if sel.any() else (0.0, 255.0)
    return np.clip((g - lo) * (255.0 / max(float(hi - lo), 1.0)), 0, 255).astype(np.float32)


def static_mask(a, b, valid):
    """The pixels that did not change even before any alignment (the changes grown by STATIC_GROW_PX left out),
    or valid itself when that leaves under STATIC_MIN_FRAC of it: ECC then lines the frames up on what stood
    still, not on a part that moved across much of the frame (a large sliding part seen from above)."""
    v = np.asarray(valid, dtype=bool)
    a, b = rgb(a), rgb(b)
    h, w = a.shape[:2]
    hw, hh = w // 2, h // 2            # read at half size (ECC runs there too): a quarter of the pixels
    a2 = cv2.resize(a, (hw, hh), interpolation=cv2.INTER_AREA)
    b2 = cv2.resize(b, (hw, hh), interpolation=cv2.INTER_AREA)
    v2 = cv2.resize(v.astype(np.uint8), (hw, hh), interpolation=cv2.INTER_NEAREST).astype(bool)
    g = gain(a2, b2, v2)
    bn = scaled(b2, None if not g else 1.0 / g)
    t, _ = threshold(a2, bn, v2)
    ch = change_mask(a2, bn, t, v2, near=max(1, int(L.DIFF_NEAR_PX) // 2))
    k = 2 * (STATIC_GROW_PX // 2) + 1
    grown = cv2.dilate(ch.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))).astype(bool)
    still = v2 & ~grown
    still = still if still.sum() >= STATIC_MIN_FRAC * max(1, int(v2.sum())) else v2
    return cv2.resize(still.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST).astype(bool)


def align(a, b, valid=None):
    """ECC (Euclidean) of b onto a on half-size grey frames: the own body (outside valid) and what changed even
    before alignment (static_mask) left out."""
    a, b = rgb(a), rgb(b)
    h, w = a.shape[:2]
    hw, hh = w // 2, h // 2
    ga = cv2.resize(grey(a), (hw, hh), interpolation=cv2.INTER_AREA)
    gb = cv2.resize(grey(b), (hw, hh), interpolation=cv2.INTER_AREA)
    mask = None
    if valid is not None:
        still = static_mask(a, b, valid)
        mask = cv2.resize(still.astype(np.uint8), (hw, hh), interpolation=cv2.INTER_NEAREST)
        if mask.sum() < MIN_VALID_FRAC * hw * hh:
            return {"ok": False, "why": "the robot's own body covers nearly all of the frame", "shift_px": None,
                    "turn_deg": None, "warp": None}
    sel = np.ones(ga.shape, dtype=bool) if mask is None else mask > 0
    ga, gb = _stretch(ga, sel), _stretch(gb, sel)
    W = np.eye(2, 3, dtype=np.float32)
    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, ECC_ITERS, ECC_EPS)
    try:
        _, W = cv2.findTransformECC(ga, gb, W, cv2.MOTION_EUCLIDEAN, crit, mask, ECC_BLUR)
    except cv2.error:
        return {"ok": False, "why": "could not line the frames up (the alignment did not settle)",
                "shift_px": None, "turn_deg": None, "warp": None}
    Wf = W.astype(np.float64).copy()
    Wf[:, 2] *= 2.0
    c = np.array([w / 2.0, h / 2.0])
    shift = float(np.linalg.norm(Wf[:, :2] @ c + Wf[:, 2] - c))
    turn = math.degrees(math.atan2(float(Wf[1, 0]), float(Wf[0, 0])))
    ok = shift <= L.ALIGN_SHIFT_PX and abs(turn) <= L.ALIGN_TURN_DEG
    return {"ok": bool(ok), "why": None if ok else "could not line the frames up",
            "shift_px": round(shift, 2), "turn_deg": round(turn, 3), "warp": Wf.tolist()}


def warp(b, al):
    """(b lined up on a, bool mask of the pixels b covers there) for an align() result."""
    b = rgb(b)
    h, w = b.shape[:2]
    if not al or al.get("warp") is None:
        return b, np.ones((h, w), dtype=bool)
    Wf = np.asarray(al["warp"], dtype=np.float64)
    if np.allclose(Wf, np.eye(2, 3), atol=1e-9):
        return b, np.ones((h, w), dtype=bool)
    flags = cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP
    out = cv2.warpAffine(b, Wf, (w, h), flags=flags, borderMode=cv2.BORDER_REPLICATE)
    cover = cv2.warpAffine(np.ones((h, w), dtype=np.uint8), Wf, (w, h), flags=cv2.INTER_NEAREST
                           + cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return out, cover.astype(bool)


def _grad(g):
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    return np.sqrt(gx * gx + gy * gy)


def gain(a, b, valid):
    """The median of b over a (grey) on low-gradient pixels of both frames that are neither black nor saturated:
    1.0 when the light did not change. None when too few pixels can be read."""
    ga, gb = grey(a), grey(b)
    sel = np.asarray(valid, dtype=bool) & (ga >= GAIN_GREY_LO) & (gb >= GAIN_GREY_LO) & (ga <= GAIN_GREY_HI) \
        & (gb <= GAIN_GREY_HI)
    if sel.sum() < MIN_VALID_FRAC * sel.size:
        return None
    gr = np.maximum(_grad(ga), _grad(gb))
    thr = float(np.percentile(gr[sel], GAIN_GRAD_PCT))
    sel &= gr <= thr
    if not sel.any():
        return None
    return float(np.median(gb[sel] / ga[sel]))


def gain_spread(a, b, valid, g):
    """The interquartile spread of b over a (grey, divided by the gain g) on the pixels gain() reads: under
    GAIN_UNIFORM_IQR the change is one scale for the whole frame (an exposure), else the light changed unevenly."""
    if g is None:
        return None
    ga, gb = grey(a), grey(b)
    sel = np.asarray(valid, dtype=bool) & (ga >= GAIN_GREY_LO) & (gb >= GAIN_GREY_LO) & (ga <= GAIN_GREY_HI) \
        & (gb <= GAIN_GREY_HI)
    if sel.sum() < MIN_VALID_FRAC * sel.size:
        return None
    gr = np.maximum(_grad(ga), _grad(gb))
    sel &= gr <= float(np.percentile(gr[sel], GAIN_GRAD_PCT))
    r = gb[sel] / ga[sel] / float(g)
    q1, q3 = np.percentile(r, (25.0, 75.0))
    return float(q3 - q1)


def scaled(img, k):
    """img with every channel scaled by k (the light's gain undone), clipped to 0..255."""
    if k is None or abs(float(k) - 1.0) < 1e-6:
        return rgb(img)
    return np.clip(rgb(img).astype(np.float32) * float(k), 0, 255).astype(np.uint8)


def threshold(a, b, valid):
    """(t, m): the change threshold max(DIFF_MIN, DIFF_SCALE x m) and m, the median grey of both frames."""
    v = np.asarray(valid, dtype=bool)
    vals = np.concatenate([grey(a)[v], grey(b)[v]]) if v.any() else np.concatenate([grey(a).ravel(),
                                                                                  grey(b).ravel()])
    m = float(np.median(vals))
    return max(L.DIFF_MIN, L.DIFF_SCALE * m), m


# ---------------------------------------------------------------- changes
def _nearest(x, y, r):
    """Per pixel of x: the smallest, over pixels of y within r (a square), of the largest channel difference."""
    h, w = x.shape[:2]
    yp = cv2.copyMakeBorder(y, r, r, r, r, cv2.BORDER_REPLICATE)
    best = np.full((h, w), 255, dtype=np.uint8)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            ys = yp[r + dy:r + dy + h, r + dx:r + dx + w]
            d = cv2.absdiff(x, np.ascontiguousarray(ys))
            m = cv2.max(cv2.max(d[..., 0], d[..., 1]), d[..., 2])
            best = cv2.min(best, m)
    return best


def change_mask(a, b, t, valid, near=None):
    """bool H x W: changed pixels (both ways over t, see the module text), opened 3 x 3, inside valid. near: the
    neighbourhood in pixels (DIFF_NEAR_PX; half of it on half-size frames)."""
    a, b = rgb(a), rgb(b)
    r = int(L.DIFF_NEAR_PX if near is None else near)
    lim = int(math.floor(float(t)))
    ch = (_nearest(a, b, r) > lim) & (_nearest(b, a, r) > lim) & np.asarray(valid, dtype=bool)
    k = np.ones((3, 3), dtype=np.uint8)
    return cv2.morphologyEx(ch.astype(np.uint8), cv2.MORPH_OPEN, k).astype(bool)


def components(mask, min_px=L.COMPONENT_MIN_PX):
    """[{label, box [u0, v0, u1, v1], area, centroid}] of 8-connected changes of min_px or more, largest first
    (ties by position), and the label image."""
    n, lab, stats, cents = cv2.connectedComponentsWithStats(np.asarray(mask, dtype=np.uint8), connectivity=8)
    out = []
    for i in range(1, n):
        x, y, ww, hh, area = (int(v) for v in stats[i])
        if area < min_px:
            continue
        out.append({"label": i, "box": [x, y, x + ww, y + hh], "area": area,
                    "centroid": [round(float(cents[i][0]), 1), round(float(cents[i][1]), 1)]})
    out.sort(key=lambda c: (-c["area"], c["box"][1], c["box"][0]))
    return out, lab


def _pix(img, m):
    return rgb(img)[m].reshape(-1, 1, 3)


def grow_box(box, k, w, h):
    return [max(0, box[0] - k), max(0, box[1] - k), min(w, box[2] + k), min(h, box[3] + k)]


def patch_offset(a, b, box, search=L.MOVED_SEARCH_PX):
    """(dx, dy, ncc) where a's patch inside box is found best in b within search px (normalised correlation on
    grey), or None when the patch is too flat or too small to find (the whole device's box: device_moved)."""
    ga, gb = grey(a), grey(b)
    h, w = ga.shape
    u0, v0, u1, v1 = (int(x) for x in box)
    u0, v0, u1, v1 = max(0, u0), max(0, v0), min(w, u1), min(h, v1)
    if u1 - u0 < 4 or v1 - v0 < 4:
        return None
    tpl = ga[v0:v1, u0:u1]
    if float(tpl.std()) < FLAT_STD:
        return None
    s0, t0 = max(0, u0 - search), max(0, v0 - search)
    s1, t1 = min(w, u1 + search), min(h, v1 + search)
    area = gb[t0:t1, s0:s1]
    if area.shape[0] < tpl.shape[0] or area.shape[1] < tpl.shape[1]:
        return None
    res = cv2.matchTemplate(area, tpl, cv2.TM_CCOEFF_NORMED)
    res = np.nan_to_num(res, nan=-1.0, posinf=-1.0, neginf=-1.0)
    _, best, _, loc = cv2.minMaxLoc(res)
    return float(s0 + loc[0] - u0), float(t0 + loc[1] - v0), float(best)


def _masked_offset(ga, gb, box, tm, search):
    """(dx, dy, ncc, ncc at no offset) of ga's patch in box (tm: its outline mask, or None) found best in gb."""
    u0, v0, u1, v1 = box
    h, w = ga.shape
    tpl = ga[v0:v1, u0:u1]
    vals = tpl[tm] if tm is not None else tpl
    if vals.size < L.COMPONENT_MIN_PX or float(vals.std()) < FLAT_STD:
        return None
    s0, t0 = max(0, u0 - search), max(0, v0 - search)
    s1, t1 = min(w, u1 + search), min(h, v1 + search)
    area = gb[t0:t1, s0:s1]
    if area.shape[0] < tpl.shape[0] or area.shape[1] < tpl.shape[1]:
        return None
    if tm is None:
        res = cv2.matchTemplate(area, tpl, cv2.TM_CCOEFF_NORMED)
    else:
        res = cv2.matchTemplate(area, tpl, cv2.TM_CCOEFF_NORMED, mask=tm.astype(np.uint8))
    res = np.nan_to_num(res, nan=-1.0, posinf=-1.0, neginf=-1.0)
    _, best, _, loc = cv2.minMaxLoc(res)
    return float(s0 + loc[0] - u0), float(t0 + loc[1] - v0), float(best), float(res[v0 - t0, u0 - s0])


def device_offset(a, b, box, mask=None, search=L.MOVED_SEARCH_PX):
    """(dx, dy, ncc, turn_deg, stayed_ncc): the device's patch of a (its box; mask: the pixels the device's outline
    covers) found best in b by normalised correlation, also with the patch turned by each of DEVICE_TURNS_DEG about
    its centre (a device someone moved is often turned a few degrees too), and stayed_ncc, the correlation of the
    same patch where it was (no offset, no turn): the two hypotheses a caller weighs. Read on half-size grey
    frames (area averaged): a whole outline needs no pixel detail, the averaging takes out what differs between
    two renderers' edges, and it is a quarter of the work. Offsets are full-size pixels. None when it cannot be
    read."""
    ga, gb = grey(a), grey(b)
    h, w = ga.shape
    u0, v0, u1, v1 = (int(x) for x in box)
    u0, v0, u1, v1 = max(0, u0), max(0, v0), min(w, u1), min(h, v1)
    if u1 - u0 < 8 or v1 - v0 < 8:
        return None
    full = np.ones((h, w), dtype=bool) if mask is None else np.asarray(mask, dtype=bool)
    hw, hh = w // 2, h // 2
    ga = cv2.resize(ga, (hw, hh), interpolation=cv2.INTER_AREA)
    gb = cv2.resize(gb, (hw, hh), interpolation=cv2.INTER_AREA)
    full = cv2.resize(full.astype(np.uint8), (hw, hh), interpolation=cv2.INTER_NEAREST).astype(bool)
    u0, v0, u1, v1 = u0 // 2, v0 // 2, min(hw, (u1 + 1) // 2), min(hh, (v1 + 1) // 2)
    best, stayed = None, None
    for turn in (0.0,) + tuple(t for t in DEVICE_TURNS_DEG if t != 0.0):
        ra, rm = ga, full
        if turn != 0.0:
            M = cv2.getRotationMatrix2D(((u0 + u1) / 2.0, (v0 + v1) / 2.0), float(turn), 1.0)
            ra = cv2.warpAffine(ga, M, (hw, hh), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
            inside = cv2.warpAffine(np.ones((hh, hw), dtype=np.uint8), M, (hw, hh), flags=cv2.INTER_NEAREST,
                                    borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
            rm = cv2.warpAffine(full.astype(np.uint8), M, (hw, hh), flags=cv2.INTER_NEAREST,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=0).astype(bool) & inside
        tm = rm[v0:v1, u0:u1]
        off = _masked_offset(ra, gb, (u0, v0, u1, v1), None if tm.all() else tm, max(1, int(search) // 2))
        if off is None:
            continue
        if turn == 0.0:
            stayed = off[3]
        if best is None or off[2] > best[2]:
            best = (2.0 * off[0], 2.0 * off[1], off[2], float(turn))
    if best is None:
        return None
    return best + (stayed if stayed is not None else -1.0,)


def moved_shift(a, b, mask, t, search=L.MOVED_SEARCH_PX):
    """(dx, dy, share): the shift d (|d| up to search px) that explains the most changed pixels, a pixel explained
    when a there matches b at p + d or b there matches a at p - d (largest channel difference t or less): a part
    that moved shows where it was in a and where it went in b, and the shift that carries one onto the other
    explains both. A plain part (a single-colour block) has no texture for a correlation to lock on, but its colour
    still matches. None when fewer than COMPONENT_MIN_PX pixels changed."""
    a, b = rgb(a).astype(np.int16), rgb(b).astype(np.int16)
    h, w = a.shape[:2]
    ys, xs = np.nonzero(np.asarray(mask, dtype=bool))
    if len(xs) < L.COMPONENT_MIN_PX:
        return None
    step = max(1, len(xs) // MOVED_SAMPLE_MAX)
    ys, xs = ys[::step], xs[::step]
    pa, pb = a[ys, xs], b[ys, xs]
    lim = float(t)
    r = int(search)
    # the frames padded by the search radius with a value nothing matches: a shift off the frame explains nothing
    ap = np.pad(a, ((r, r), (r, r), (0, 0)), constant_values=-4096)
    bp = np.pad(b, ((r, r), (r, r), (0, 0)), constant_values=-4096)
    yp, xp = ys + r, xs + r

    def score(dx, dy, k=1):
        """(rank, share) of shift (dx, dy) on every k-th sample (the coarse pass reads a quarter of them)."""
        m1 = np.max(np.abs(pa[::k] - bp[yp[::k] + dy, xp[::k] + dx]), axis=1) <= lim
        m0 = np.max(np.abs(pb[::k] - ap[yp[::k] - dy, xp[::k] - dx]), axis=1) <= lim
        # ranked by both ways together (a plain part's colour matches over a plateau of shifts one way alone);
        # the share explained either way is what is reported
        return float(np.mean(m1)) + float(np.mean(m0)), float(np.mean(m1 | m0))

    best = (0, 0, -1.0, 0.0)
    coarse = MOVED_COARSE_EVERY if len(xs) >= MOVED_COARSE_EVERY * L.COMPONENT_MIN_PX else 1
    for dy in range(-r, r + 1, MOVED_STEP_PX):
        for dx in range(-r, r + 1, MOVED_STEP_PX):
            if dx * dx + dy * dy <= r * r and math.hypot(dx, dy) >= MOVED_MIN_PX:
                rank, share = score(dx, dy, coarse)
                if rank > best[2]:
                    best = (dx, dy, rank, share)
    best = (best[0], best[1], -1.0, 0.0)        # the fine pass ranks on every sample
    cx, cy = best[0], best[1]
    for dy in range(cy - MOVED_STEP_PX + 1, cy + MOVED_STEP_PX):
        for dx in range(cx - MOVED_STEP_PX + 1, cx + MOVED_STEP_PX):
            if dx * dx + dy * dy <= r * r and math.hypot(dx, dy) >= MOVED_MIN_PX:
                rank, share = score(dx, dy)
                if rank > best[2]:
                    best = (dx, dy, rank, share)
    return float(best[0]), float(best[1]), round(best[3], 3)


def _contrast(img, m, ring):
    if not ring.any() or not m.any():
        return 0.0
    a = rgb(img).astype(np.float32)
    return float(np.max(np.abs(np.median(a[m], axis=0) - np.median(a[ring], axis=0))))


def _glows_here(p_now, p_other, img_now, ring, compact):
    """A compact patch glows by its own light in a bright frame: colour_words.glowing measures a patch against the
    frame's median value, which a lamp at full value cannot beat by GLOW_REL once the frame's median is high (a
    bright floor). Here the same test is made against the patch's OWN value in the other frame (it is GLOW_REL
    times brighter than it was, or will be, and passes the same brightness and saturation rules), and the patch
    must also stand over the ring around it by more than DIFF_MIN (the smallest difference that counts as a
    change), so a bright floor a dark thing left bare (as bright as the floor round it) does not glow."""
    if not compact or not ring.any():
        return False
    v_now = float(colour_words.hsv(p_now)[2])
    v_ring = float(colour_words.hsv(_pix(img_now, ring))[2])
    return v_now - v_ring > L.DIFF_MIN and bool(colour_words.glowing(p_now, p_other, compact=True))


def describe(comp, lab, a, b, gains_a, gains_b, frame_shape, t, find_move=True):
    """One change: box, area, colour before and after (floor gains), brighter or darker, glowing (after, else
    before) and its kind: lit, went_dark, moved (a's patch found again elsewhere in b: moved_px), colour_changed,
    appeared or gone (which frame's patch stands out more from the ring around it). t: the change threshold;
    find_move False skips the move search (a change too small to be reported)."""
    m = lab == comp["label"]
    h, w = frame_shape[:2]
    pa, pb = _pix(a, m), _pix(b, m)
    ca, cb = colour_words.name(pa, gains_a), colour_words.name(pb, gains_b)
    va, vb = colour_words.hsv(pa)[2], colour_words.hsv(pb)[2]
    cmp = colour_words.compact(m, frame_shape)
    gb = grow_box(comp["box"], CONTRAST_GROW_PX, w, h)
    ring = np.zeros((h, w), dtype=bool)
    ring[gb[1]:gb[3], gb[0]:gb[2]] = True
    ring &= ~m
    ga_ = colour_words.glowing(pa, a, compact=cmp) or _glows_here(pa, pb, a, ring, cmp)
    gb_ = colour_words.glowing(pb, b, compact=cmp) or _glows_here(pb, pa, b, ring, cmp)
    out = {"box_px": comp["box"], "area_px": comp["area"], "colour_before": ca, "colour_after": cb,
           "way": "brighter" if vb > va else "darker", "glowing": bool(gb_), "compact": bool(cmp)}
    if gb_ and not ga_ and vb > va:
        kind = "lit"
    elif ga_ and not gb_ and vb < va:
        kind = "went_dark"
    else:
        kind = None
        off = moved_shift(a, b, m, t) if find_move else None
        if off is not None and off[2] >= MOVED_EXPLAIN_FRAC:
            kind = "moved"
            out["moved_px"] = [round(off[0], 1), round(off[1], 1)]
        elif ca != cb:
            kind = "colour_changed"
        else:
            kind = "appeared" if _contrast(b, m, ring) > _contrast(a, m, ring) else "gone"
    out["kind"] = kind
    return out


# ---------------------------------------------------------------- targets
def region(shape, spec, grow=L.RING_PX):
    """(region, ring) bool masks for a target spec {"disc": [u, v, r]} or {"box": [u0, v0, u1, v1]}: the ring is
    the region grown by grow px, less the region."""
    h, w = shape[:2]
    reg = np.zeros((h, w), dtype=np.uint8)
    big = np.zeros((h, w), dtype=np.uint8)
    if "disc" in spec:
        u, v, r = (float(x) for x in spec["disc"])
        c = (int(round(u)), int(round(v)))
        cv2.circle(reg, c, max(1, int(round(r))), 1, -1)
        cv2.circle(big, c, max(1, int(round(r + grow))), 1, -1)
    else:
        u0, v0, u1, v1 = (int(round(x)) for x in spec["box"])
        reg[max(0, v0):max(0, v1), max(0, u0):max(0, u1)] = 1
        big[max(0, v0 - grow):max(0, v1 + grow), max(0, u0 - grow):max(0, u1 + grow)] = 1
    reg, big = reg.astype(bool), big.astype(bool)
    return reg, big & ~reg


def across_px(reg):
    """The smaller extent of a region's bounding box (px), 0 when empty."""
    ys, xs = np.nonzero(reg)
    if not len(xs):
        return 0
    return int(min(xs.max() - xs.min() + 1, ys.max() - ys.min() + 1))


def judge(reg, ring, changed, valid, m, part_px=None, light_changed=False, dim=False, cam=None):
    """{changed: yes | no | unclear, solid, share, ring_share, why}: the module's rule at one region. reg, ring:
    masks; changed: the change mask to count (under a light change: only the compact changes whose colour name
    changed); valid: pixels usable in both frames (not the own body); m: the median grey of the two frames;
    part_px: the part's own size in this frame when the sheet gives one (else the region's extent)."""
    rv = reg & valid
    n = int(rv.sum())
    if n == 0 or n < 0.5 * max(1, int(reg.sum())):
        return {"changed": "unclear", "solid": False, "share": None, "ring_share": None,
                "why": "the robot's own body or the frame's edge covers it"}
    share = float(changed[rv].mean())
    rr = ring & valid
    ring_share = float(changed[rr].mean()) if rr.any() else 0.0
    far = valid & ~reg & ~ring
    far_share = float(changed[far].mean()) if far.any() else 0.0
    across = across_px(rv) if part_px is None else min(float(part_px), across_px(rv))
    solid = across >= L.SOLID_MIN_PX and m >= perception.dim_level(cam) and not dim and not light_changed

    def stands_out(other):
        return share >= L.CHANGED_RING_MULT * other + L.CHANGED_RING_ADD
    large = across >= LARGE_PART_PX and stands_out(far_share)
    if share >= L.CHANGED_SHARE and (stands_out(ring_share) or large):
        verdict, why = "yes", None
    elif share <= L.NO_SHARE and solid:
        verdict, why = "no", None
    else:
        verdict = "unclear"
        if share <= L.NO_SHARE:
            why = ("the light changed between the frames" if light_changed else "the frames are dim" if
                   (dim or m < perception.dim_level(cam)) else "the part is small in the frame")
        elif share >= L.CHANGED_SHARE:
            why = "the ring around it changed as much"
        else:
            why = "a small change only"
    return {"changed": verdict, "solid": bool(solid), "share": round(share, 3), "ring_share": round(ring_share, 3),
            "far_share": round(far_share, 3), "why": why}


# ---------------------------------------------------------------- the picture
def triptych(a, b, changed, marks=(), width=L.TRIPTYCH_W):
    """before | after | changes, width px wide (3 panels at the frames' aspect): the third is the after frame
    dimmed, changed pixels painted, each mark (a disc [u, v, r] or a box) outlined. Deterministic."""
    a, b = rgb(a), rgb(b)
    h, w = a.shape[:2]
    ws = [width // 3, width // 3, width - 2 * (width // 3)]
    ph = int(round(h * ws[0] / float(w)))
    third = (b.astype(np.float32) * PANEL_DIM).astype(np.uint8)
    third[np.asarray(changed, dtype=bool)] = CHANGE_RGB
    for mk in marks:
        if "disc" in mk:
            u, v, r = mk["disc"]
            cv2.circle(third, (int(round(u)), int(round(v))), max(2, int(round(r))), MARK_RGB, 2)
        elif "box" in mk:
            u0, v0, u1, v1 = (int(round(x)) for x in mk["box"])
            cv2.rectangle(third, (u0, v0), (u1, v1), MARK_RGB, 2)
    panels = [cv2.resize(img, (pw, ph), interpolation=cv2.INTER_AREA) for img, pw in zip((a, b, third), ws)]
    out = np.concatenate(panels, axis=1)
    for x in (ws[0], ws[0] + ws[1]):
        out[:, x - 1:x + 1] = 255
    return out
