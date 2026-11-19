# SPDX-License-Identifier: AGPL-3.0-only
"""Package see_place:
where a held thing can STAND at a place a look shows, and what the looks show OVER a place. Pure geometry on numpy
and OpenCV over points a look gave (rcpx/r2/see_spot.py cloud_of, read only): no executor, no robot, no truth, no
motion. rcpx/r2/place_spot.py is the service over it.

Units: metres. Points are given in the level frame: columns (x, y) on the level axes (see_cloud.axes_of) and the
height along up.

    heights(points, lo_xy, hi_xy, cell_m)       the look's height map on a level grid: the highest point per cell
    stand(H, cell_m, origin_xy, at_xy, radius_m, tol_m, floor_z=None, under_z=None, on=None)
                                                 where a footprint stands fully on ONE level the look shows
    walls(H), fill(H), basins(H, F, deep_m, own, base_z, least)   the hollows the look shows: where the floor lies
                                                 under the walls around it, seen or hidden by the wall in front
    nearest_basin(found, cell_m, origin_xy, at_xy), inside(H, basin, cell_m, origin_xy, at_xy, radius_m, tol_m)
                                                 where a footprint goes into the hollow a point lies in or nearest to
    behind(H, cell_m, origin_xy, at_xy, away_xy, z0, radius_m, pad_m, tol_m)   the hollow a wall hides from the look
    level(H, at_ij, z, band_m)                   the level region a point lies on
    dish(H, walled, at_ij, band_m, gap_cells, within), room_in(H, mask, cell_m)
                                                 the inside of a dish a point lies in (under the pour height of
                                                 its rim), how far each cell lies from what the look shows
                                                 outside it
    outline(own)                                 the cells inside the outline (convex hull) of a thing's cells
    roof(points, at_xy, half_m, low_z, high_z)   the lowest thing the looks show over a place, or None

A footprint is a disc: the held thing's half width across (what the hold's facts give) and the spot's own error.
It STANDS on a cell when every cell of the disc around it is seen and the heights under the disc differ by no
more than tol_m: nothing stands there, no rim crosses it, no edge of the top lies under it, and nothing of it is
hidden from the look. tol_m is the caller's: the look's own band (what tells a thing from the level it stands
on) and the tilt a level surface may have under the disc (the ratio REL of see_cloud's LEVEL rule), never a
typed length. A grid cell is as wide as QUANT of the look's footprints, so that a level surface seen at a slant
has no cell without a point; a single cell the look left out between seen ones takes the lower of its
neighbours' heights (a ray that passes between two samples of one surface saw no wall there).
"""

import math

import cv2
import numpy as np

HALF = math.ldexp(1, -1)       # numeric: one half exactly
RING = np.ones((3, 3), np.uint8)


def heights(points, lo_xy, hi_xy, cell_m):
    """(H, origin_xy): the highest point of each cell of a level grid over [lo_xy, hi_xy] (rows along y, columns
    along x; a cell with no point is NaN; cell (0, 0) has its middle at origin_xy)."""
    P = np.asarray(points, dtype=float)
    lo, hi = np.asarray(lo_xy, dtype=float), np.asarray(hi_xy, dtype=float)
    n = np.maximum(1, np.ceil((hi - lo) / cell_m).astype(int) + 1)
    H = np.full((int(n[1]), int(n[0])), -np.inf)
    ij = np.rint((P[:, :2] - lo) / cell_m).astype(int)
    ok = np.all((ij >= 0) & (ij < n), axis=1)
    np.maximum.at(H, (ij[ok, 1], ij[ok, 0]), P[ok, 2])
    seen = np.isfinite(H)
    # a single cell left out between seen ones: the lower of what stands around it
    low = cv2.erode(np.where(seen, H, np.inf).astype(np.float32), RING)
    most = RING.size - RING.shape[0]          # seen on all sides but one row or column of its ring
    fill = ~seen & np.isfinite(low) & (cv2.blur(seen.astype(np.float32), RING.shape) * RING.size >= most - HALF)
    H = np.where(fill, low, H)
    H[~np.isfinite(H)] = np.nan
    return H, lo


def _disc(radius_m, cell_m):
    k = max(0, int(math.ceil(radius_m / cell_m)))
    g = np.arange(-k, k + 1)
    X, Y = np.meshgrid(g, g)
    return (np.hypot(X, Y) * cell_m <= radius_m + HALF * cell_m).astype(np.uint8)


def spans(H, radius_m, cell_m):
    """Per cell: how much the heights under a disc of radius_m around it differ (inf where a cell of the disc is
    not seen or lies off the grid), and the lowest height under it."""
    seen = np.isfinite(H)
    K = _disc(radius_m, cell_m)
    hi = cv2.dilate(np.where(seen, H, np.inf).astype(np.float32), K, borderType=cv2.BORDER_CONSTANT,
                    borderValue=float("inf"))
    lo = cv2.erode(np.where(seen, H, -np.inf).astype(np.float32), K, borderType=cv2.BORDER_CONSTANT,
                   borderValue=float("-inf"))
    with np.errstate(invalid="ignore"):
        span = np.where(np.isfinite(hi) & np.isfinite(lo), hi - lo, np.inf)
    return span, lo


def stand(H, cell_m, origin_xy, at_xy, radius_m, tol_m, floor_z=None, under_z=None, on=None):
    """Where a footprint of radius_m stands fully on one level of the height map H, nearest at_xy (module
    docstring). floor_z: only levels higher than this count (onto a THING: over the level it stands on);
    under_z: only levels lower than this count (INTO a thing: under its rim); on: only these cells count as the
    footprint's middle (the cells that hold a point of the thing). Returns None when no cell holds it,
    else {"xy", "z": the level there, "moved_m": how far from at_xy, "here": whether the cell under at_xy itself
    holds it, "span_m": how much the heights under the footprint differ there, "cells": how many cells hold it}."""
    span, low = spans(H, radius_m, cell_m)
    with np.errstate(invalid="ignore"):
        good = span <= tol_m
        if floor_z is not None:
            good &= low > floor_z
        if under_z is not None:
            good &= (low + span) < under_z
        if on is not None:
            good &= np.asarray(on, dtype=bool)
    if not good.any():
        return None
    origin, at = np.asarray(origin_xy, dtype=float), np.asarray(at_xy, dtype=float)
    rows, cols = np.nonzero(good)
    xy = origin + np.column_stack([cols, rows]) * cell_m
    d = np.linalg.norm(xy - at, axis=1)
    k = int(np.argmin(d))
    j, i = int(rows[k]), int(cols[k])
    here = np.rint((at - origin) / cell_m).astype(int)
    inside = 0 <= here[0] < H.shape[1] and 0 <= here[1] < H.shape[0]
    at_good = bool(inside and good[here[1], here[0]])
    return {"xy": [float(at[0]), float(at[1])] if at_good else [float(xy[k][0]), float(xy[k][1])],
            "z": float(low[here[1], here[0]] + HALF * span[here[1], here[0]]) if at_good else
            float(low[j, i] + HALF * span[j, i]), "moved_m": 0.0 if at_good else float(d[k]), "here": at_good,
            "span_m": float(span[here[1], here[0]] if at_good else span[j, i]), "cells": int(good.sum())}


def fill(H):
    """Each cell's pour height: how high water standing on it would rise before it ran off the grid's edge (a
    priority flood from the edge inward; a cell the look does not show is bottomless: it holds whatever the
    walls around it hold; a cell given as plus infinity is a wall of any height: what lies outside the picture
    may be one). A cell whose pour height stands over its own height lies in a hollow."""
    import heapq
    Hx = np.where(np.isnan(H), -np.inf, H)          # (plus infinity stays: a cell given as a wall of any height)
    rows, cols = Hx.shape
    F = np.full(Hx.shape, np.inf)
    done = np.zeros(Hx.shape, dtype=bool)
    heap = []
    for r in range(rows):
        for c in ((0, cols - 1) if 0 < r < rows - 1 else range(cols)):
            if not done[r, c]:
                done[r, c] = True
                F[r, c] = Hx[r, c]
                heapq.heappush(heap, (float(Hx[r, c]), r, c))
    while heap:
        h, r, c = heapq.heappop(heap)
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n, m = r + dr, c + dc
            if 0 <= n < rows and 0 <= m < cols and not done[n, m]:
                done[n, m] = True
                F[n, m] = max(float(Hx[n, m]), h)
                heapq.heappush(heap, (float(F[n, m]), n, m))
    return F


def walls(H, gap_cells=1):
    """The height map with the gaps between seen cells closed that are no wider than gap_cells (a wall the look
    samples cell by cell has no holes, and a gap narrower than the hand's pad lets nothing of the hand through):
    a cell the look does not show stays bottomless (minus infinity) unless seen cells stand that near on both
    sides of it; a closed cell stands as high as the lower of the two sides."""
    Hx = np.where(np.isfinite(H), H, -np.inf).astype(np.float32)
    k = max(1, int(gap_cells)) + 1
    K = np.ones((k, k), np.uint8)
    return np.maximum(Hx, cv2.erode(cv2.dilate(Hx, K), K)).astype(float)


def basins(H, F, deep_m, own=None, base_z=None, least=1):
    """The hollows a height map shows: [{"mask", "rim_z": the pour height (the lowest wall around it), "floor_z":
    the lowest height seen in it (None: its floor is not seen), "seen": how many of its cells are seen,
    "room_cells": how far its innermost cell stands from its walls, in cells}]. F: the
    pour heights (fill) of the map the hollows are read in (walls(H)). A hollow starts from SEEN cells that lie
    deeper than deep_m under their pour height, and from cells the look does not show whose pour height stands
    more than deep_m over base_z (walls all around hold them: the inside of a thing whose floor the wall on the
    lens's side hides); it takes in the cells beside them the look does not show. A stretch of unseen cells that
    runs off to the grid's edge or to the level the thing stands on is a shadow, not a hollow. own: only hollows
    that touch one of these cells count (the thing's own cells); least: the fewest cells a hollow starts from."""
    seen = np.isfinite(H)
    Hc = np.where(seen, H, -np.inf)
    with np.errstate(invalid="ignore"):
        seed = seen & np.isfinite(F) & (F - np.where(seen, H, 0.0) > deep_m)
        dark = ~seen & np.isfinite(F) & ((F > float(base_z) + deep_m) if base_z is not None else False)
    grow = seed | (~seen & np.isfinite(F))
    from scipy import ndimage
    lab, n = ndimage.label(grow)                    # (cells joined along rows and columns)
    out = []
    for k in range(1, n + 1):
        C = lab == k
        mine = C & (seed | dark)
        if int(mine.sum()) < least or (own is not None and not (
                (cv2.dilate(C.astype(np.uint8), RING) > 0) & np.asarray(own, dtype=bool)).any()):
            continue
        rim = float(np.median(F[mine]))
        with np.errstate(invalid="ignore"):
            C = C & (seed | (~seen & (F >= rim - deep_m)))      # the cells beside it only where the same walls hold
        low = Hc[C & seed]
        room = float(cv2.distanceTransform(np.pad(C.astype(np.uint8), 1), cv2.DIST_L2,
                                           cv2.DIST_MASK_PRECISE).max())
        out.append({"mask": C, "rim_z": rim, "floor_z": float(low.min()) if low.size else None,
                    "seen": int((C & seen).sum()), "room_cells": room})
    return out


def nearest_basin(found, cell_m, origin_xy, at_xy):
    """(the hollow nearest a level point, how far its nearest cell's square lies from the point) or (None, inf)."""
    best, least = None, np.inf
    at = (np.asarray(at_xy, dtype=float) - np.asarray(origin_xy, dtype=float)) / cell_m
    for b in found:
        rows, cols = np.nonzero(b["mask"])
        d = float(np.min(np.hypot(np.maximum(np.abs(cols - at[0]) - HALF, 0.0),
                                  np.maximum(np.abs(rows - at[1]) - HALF, 0.0)))) * cell_m
        if d < least:
            best, least = b, d
    return best, least


def inside(H, basin, cell_m, origin_xy, at_xy, radius_m, tol_m):
    """Where a footprint of radius_m goes into a hollow, nearest at_xy: a cell whose disc lies within the hollow's
    cells and under which the heights the look shows differ by no more than tol_m (nothing stands there; cells
    of the floor the look does not show are taken as the floor). When no cell holds it: where the footprint lies
    within the hollow, the cell under whose disc what the look shows stands lowest (a thing already inside is
    gone round as far as the hollow allows), nearest at_xy; where it lies within the hollow nowhere (the hollow
    is tighter than the footprint): the cell of the hollow furthest from its walls. Returns {"xy", "z": the lowest
    height seen under the footprint there (None: none is seen), "moved_m", "here": at_xy itself holds it, "how":
    "fits" | "lowest" | "middle", "room_m": how far that cell stands from the hollow's walls}."""
    C = basin["mask"]
    seen = np.isfinite(H) & C
    K = _disc(radius_m, cell_m)
    fits = cv2.erode(C.astype(np.uint8), K, borderType=cv2.BORDER_CONSTANT, borderValue=0) > 0
    hi = cv2.dilate(np.where(seen, H, -np.inf).astype(np.float32), K)
    lo = cv2.erode(np.where(seen, H, np.inf).astype(np.float32), K)
    with np.errstate(invalid="ignore", over="ignore"):
        flat = ~np.isfinite(hi) | (hi - lo <= tol_m)
    good = fits & flat
    origin, at = np.asarray(origin_xy, dtype=float), np.asarray(at_xy, dtype=float)
    inner = cv2.distanceTransform(np.pad(C.astype(np.uint8), 1), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    here = np.rint((at - origin) / cell_m).astype(int)
    at_in = bool(0 <= here[0] < H.shape[1] and 0 <= here[1] < H.shape[0])
    if at_in and good[here[1], here[0]]:
        j, i, how, xy = int(here[1]), int(here[0]), "fits", at
    else:
        ref = at
        if good.any():
            how, pick = "fits", good
        elif fits.any() and np.isfinite(hi[fits]).any():
            # the footprint lies within the hollow but stands on nothing flat (something already stands inside):
            # where what the look shows under it stands lowest, nearest the pointed place
            low = float(np.min(np.where(fits, hi, np.inf)))
            how, pick = "lowest", fits & (hi <= low + tol_m)
        else:
            how, pick = "middle", inner >= float(inner.max()) - HALF
            ref = origin + np.array([np.nonzero(C)[1].mean(), np.nonzero(C)[0].mean()]) * cell_m
        rows, cols = np.nonzero(pick)
        cand = origin + np.column_stack([cols, rows]) * cell_m
        k = int(np.argmin(np.linalg.norm(cand - ref, axis=1)))
        j, i, xy = int(rows[k]), int(cols[k]), cand[k]
    z = float(lo[j, i]) if np.isfinite(lo[j, i]) else None
    return {"xy": [float(xy[0]), float(xy[1])], "z": z, "moved_m": float(np.linalg.norm(xy - at)),
            "here": bool(at_in and good[here[1], here[0]]), "how": how, "room_m": float(inner[j, i]) * cell_m}


def behind(H, cell_m, origin_xy, at_xy, away_xy, z0, radius_m, pad_m, tol_m):
    """The hollow a WALL hides: for a point on a wall's face (level point at_xy at height z0, seen along the level
    direction away_xy, away from the lens), where the held thing goes when the look does not show what lies past
    the wall. Walking away from the lens, the wall is the seen cells that stand no lower than z0 within a pad's
    width (pad_m) of the point; its top is the highest of them. Then:
    - "behind": every cell from the wall's top on, as far as the footprint is wide (2 radius_m), is not seen or
      lies under the top by more than tol_m: the wall hides a hollow; the place is the footprint's radius and a
      cell past the wall's last cell;
    - "before": no such stretch behind, and more than half of the cells that far TOWARD the lens are not seen
      (the pointed ray came in over a wall that hides the floor in front of the face it landed on: an inner
      face): the place is the footprint's radius and a cell before the point.
    Returns None when neither holds (the top of a solid thing is seen behind its face; the support is seen in
    front of it), else {"xy", "rim_z": the wall's top, "side": "behind" | "before", "moved_m"}."""
    g = np.asarray(away_xy, dtype=float)
    n = float(np.linalg.norm(g))
    if n <= 0.0:
        return None
    g = g / n
    origin, at = np.asarray(origin_xy, dtype=float), np.asarray(at_xy, dtype=float)

    def cell(k):
        ij = np.rint((at + g * k * cell_m - origin) / cell_m).astype(int)
        if not (0 <= ij[0] < H.shape[1] and 0 <= ij[1] < H.shape[0]):
            return None, False
        h = H[ij[1], ij[0]]
        return (float(h) if np.isfinite(h) else None), True

    reach = int(math.ceil(pad_m / cell_m)) + 1
    wide = int(math.ceil((radius_m + radius_m) / cell_m))
    top, last = float(z0), 0
    for k in range(0, reach + 1):
        h, _ = cell(k)
        if h is not None and h >= float(z0) - tol_m:
            top, last = max(top, h), k
    past = [cell(k) for k in range(last + 1, last + 1 + wide)]
    if all(inside and (h is None or h < top - tol_m) for h, inside in past) and any(h is None for h, _ in past):
        xy = at + g * (last * cell_m + radius_m + cell_m)
        return {"xy": [float(xy[0]), float(xy[1])], "rim_z": top, "side": "behind",
                "moved_m": float(np.linalg.norm(xy - at))}
    front = [cell(-k) for k in range(1, wide + 1)]
    unseen = sum(1 for h, inside in front if inside and h is None)
    if all(inside for _, inside in front) and unseen + unseen > len(front) and \
            all(h is None or h < top - tol_m for h, _ in front):
        xy = at - g * (radius_m + cell_m)
        return {"xy": [float(xy[0]), float(xy[1])], "rim_z": top, "side": "before",
                "moved_m": float(np.linalg.norm(xy - at))}
    return None


def level(H, at_ij, z, band_m):
    """The level region a point lies on: the seen cells at height z (within band_m) joined to the cell at_ij
    (column, row) along rows and columns, or the region of such cells nearest it when its own cell is none (a
    point on a face beside the level). Returns a mask, or None when the look shows no cell at that height."""
    from scipy import ndimage
    with np.errstate(invalid="ignore"):
        lvl = np.isfinite(H) & (np.abs(H - float(z)) <= float(band_m))
    if not lvl.any():
        return None
    lab, _ = ndimage.label(lvl)
    i, j = int(at_ij[0]), int(at_ij[1])
    if 0 <= i < H.shape[1] and 0 <= j < H.shape[0] and lab[j, i] > 0:
        return lab == lab[j, i]
    rows, cols = np.nonzero(lab)
    k = int(np.argmin(np.hypot(rows - j, cols - i)))
    return lab == lab[rows[k], cols[k]]


def dish(H, walled, at_ij, band_m, gap_cells=1, within=None):
    """The inside of a dish a point lies in or on: the seen cells that lie under the map's pour height (fill over
    walls(H, gap_cells), the cells given in walled standing as walls of any height: what the picture does not hold
    may be one) by more than band_m, within the cells given (the outline of the thing the point lies on: a pixel on
    a dish's rim or wall names its inside), the region of them joined along rows and columns that the cell at_ij
    (column, row) lies in, else the one nearest it. A floor that is not flat (a plate's) is one inside: every
    cell of it lies under the rim the water would pour over. Returns {"mask", "rim_z": the median pour height
    over it, "floor_z": the lowest height seen in it, "pour": the pour heights} or None (no such inside: the top
    of a thing, the bare support)."""
    from scipy import ndimage
    held_by = walls(H, gap_cells)
    held_by[np.asarray(walled, dtype=bool)] = np.inf
    F = fill(held_by)
    with np.errstate(invalid="ignore"):
        inner = np.isfinite(H) & np.isfinite(F) & (F - H > float(band_m))
    keep = np.ones(H.shape, dtype=bool) if within is None else np.asarray(within, dtype=bool)
    inner &= keep
    if not inner.any():
        return None
    # parts of one inside that cells the look does not show split (a shadow, what hides the floor) are joined
    # through those cells, where the same walls hold them
    with np.errstate(invalid="ignore"):
        hidden = ~np.isfinite(H) & keep & np.isfinite(F) & (F >= float(np.min(F[inner])))
    lab, _ = ndimage.label(inner | hidden)
    i, j = int(at_ij[0]), int(at_ij[1])
    if 0 <= i < H.shape[1] and 0 <= j < H.shape[0] and lab[j, i] > 0 and (lab == lab[j, i])[inner].any():
        m = (lab == lab[j, i]) & inner          # the point lies in this inside (the hollow it lies in, deep or not)
    else:                                       # the point lies on a rim or a wall: the inside nearest it
        rows, cols = np.nonzero(lab * inner)
        k = int(np.argmin(np.hypot(rows - j, cols - i)))
        m = (lab == lab[rows[k], cols[k]]) & inner
    return {"mask": m, "rim_z": float(np.median(F[m])), "floor_z": float(np.min(H[m])), "pour": F}


def room_in(H, mask, cell_m):
    """Per cell: how far it lies from the nearest cell the look shows outside mask (m; infinite where it shows
    none). A cell the look does not show is no wall."""
    walls_ = np.isfinite(H) & ~np.asarray(mask, dtype=bool)
    if not walls_.any():
        return np.full(H.shape, np.inf)
    return cv2.distanceTransform((~walls_).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE).astype(float) * cell_m


def cells_of(xy, shape, cell_m, origin_xy):
    """The cells of a grid that hold one of the level points xy, grown by one cell (a thing's own cells)."""
    ij = np.rint((np.asarray(xy, dtype=float) - np.asarray(origin_xy, dtype=float)) / cell_m).astype(int)
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < shape[0])
    m = np.zeros(shape, np.uint8)
    m[ij[ok, 1], ij[ok, 0]] = 1
    return cv2.dilate(m, RING) > 0


def outline(own):
    """The cells inside the outline of a thing's own cells on the grid (their convex hull, filled)."""
    m = np.asarray(own, dtype=bool)
    rows, cols = np.nonzero(m)
    out = np.zeros(m.shape, np.uint8)
    if not len(rows):
        return m.copy()
    hull = cv2.convexHull(np.column_stack([cols, rows]).astype(np.int32))
    cv2.fillConvexPoly(out, hull, 1)
    return (out > 0) | m


def roof(points, at_xy, half_m, low_z, high_z):
    """The lowest thing the looks show over a place: of the points within half_m of at_xy on the level and
    between the heights low_z and high_z, the lowest. Returns None when there is none, else {"z": its height,
    "points": how many points stand in that column, "top_z": the highest of them}."""
    P = np.asarray(points, dtype=float)
    if not len(P):
        return None
    near = (np.linalg.norm(P[:, :2] - np.asarray(at_xy, dtype=float), axis=1) <= half_m) & (P[:, 2] > low_z) & \
        (P[:, 2] < high_z)
    if not near.any():
        return None
    z = P[near, 2]
    return {"z": float(z.min()), "points": int(near.sum()), "top_z": float(z.max())}
