# SPDX-License-Identifier: AGPL-3.0-only
"""Geometry of the obstacles package: obstacle shapes as signed distance functions, the robot's parts as
sampled volumes (collision capsules from RobotModel plus the mounted parts), and the path check.

SI inside (m, rad). A signed distance is negative inside the solid. Every function is a pure function of its inputs;
nothing here reads the scene or names a robot.

Shapes (one dict each, "shape" key):
- sphere     {c, R}: a ball (a triangulated point, a located point, a contact without a normal estimate).
- half_ball  {c, n, R}: the ball of radius R around a contact point cut by the touched surface: n is the surface
             normal toward the robot's side, the solid is the half behind the surface. The robot was on the n side
             when it touched, so leaving the surface is free and going through it is not.
- box        {c, Rb, half}: a located box (columns of Rb are its axes).
- sector     {h, a, u, A, r, w}: the volume a part swings through about a hinge: hinge point h, unit axis a, unit
             start direction u (perpendicular to a, hinge toward the moving edge at rest), arc A (rad, turning by
             the right-hand rule about a), radius r, width w along the axis centred on h.
"""

import math

import numpy as np



# ---------------------------------------------------------------------------------------------------- distances
def d_sphere(P, c, R):
    return np.linalg.norm(P - c, axis=1) - R


def d_half_ball(P, c, n, R):
    w = P - c
    h = w @ n                               # height above the touched surface, robot side positive
    lat = np.linalg.norm(w - np.outer(h, n), axis=1)
    r = np.linalg.norm(w, axis=1)
    out = np.empty(len(P))
    below = h <= 0.0
    out[below] = r[below] - R
    inside = below & (r < R)
    out[inside] = -np.minimum(R - r[inside], -h[inside])
    disc = (~below) & (lat <= R)
    out[disc] = h[disc]
    rim = (~below) & (lat > R)
    out[rim] = np.hypot(lat[rim] - R, h[rim])
    return out


def d_box(P, c, Rb, half):
    q = np.abs((P - c) @ Rb) - half
    return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(np.max(q, axis=1), 0.0)


def d_sector(P, h, a, u, A, r, w):
    v = np.cross(a, u)
    X = P - h
    s = X @ a
    x, y = X @ u, X @ v
    rho = np.hypot(x, y)
    phi = np.mod(np.arctan2(y, x), 2.0 * math.pi)
    dr, ds = rho - r, np.abs(s) - w / 2.0
    out = np.empty(len(P))
    arc = phi <= A
    outside = np.hypot(np.maximum(dr, 0.0), np.maximum(ds, 0.0))
    ang = np.minimum(phi, A - phi)
    depth_ang = np.where(ang < math.pi / 2.0, rho * np.sin(np.clip(ang, 0.0, None)), rho)
    inner = -np.minimum(np.minimum(-dr, -ds), depth_ang)
    is_in = arc & (dr < 0) & (ds < 0)
    out[arc] = outside[arc]
    out[is_in] = inner[is_in]
    rest = ~arc
    if rest.any():
        best = np.full(int(rest.sum()), np.inf)
        Xr, sr = X[rest], np.clip(s[rest], -w / 2.0, w / 2.0)
        for e in (u, math.cos(A) * u + math.sin(A) * v):
            t = np.clip(Xr @ e, 0.0, r)
            best = np.minimum(best, np.linalg.norm(Xr - np.outer(t, e) - np.outer(sr, a), axis=1))
        out[rest] = best
    return out


def distance(ob, P):
    """Signed distance (m) from every row of P to obstacle ob."""
    sh = ob["shape"]
    if sh == "sphere":
        return d_sphere(P, ob["c"], ob["R"])
    if sh == "half_ball":
        return d_half_ball(P, ob["c"], ob["n"], ob["R"])
    if sh == "box":
        return d_box(P, ob["c"], ob["Rb"], ob["half"])
    if sh == "sector":
        return d_sector(P, ob["h"], ob["a"], ob["u"], ob["A"], ob["r"], ob["w"])
    raise ValueError(f"unknown obstacle shape {sh!r}")


# ---------------------------------------------------------------------------------------------------- mounted parts
def housing_in_camera(size_m):
    """The camera housing box in the camera frame (x image right, y image up, z out of the back): the lens sits on
    the housing's front face."""
    w, h, d = size_m
    return {"c": np.array([0.0, 0.0, d / 2.0]), "half": np.array([w, h, d]) / 2.0}


def box_surface_points(half, step=None):
    """Points over the six faces of a box with half extents half, in its own frame."""
    step = min(x for x in half if x > 0) if step is None else step
    pts = []
    for ax in range(3):
        o1, o2 = [i for i in range(3) if i != ax]
        n1 = max(2, int(math.ceil(2 * half[o1] / step)) + 1)
        n2 = max(2, int(math.ceil(2 * half[o2] / step)) + 1)
        g1, g2 = np.meshgrid(np.linspace(-half[o1], half[o1], n1), np.linspace(-half[o2], half[o2], n2))
        for sgn in (-1.0, 1.0):
            p = np.zeros((g1.size, 3))
            p[:, ax] = sgn * half[ax]
            p[:, o1], p[:, o2] = g1.ravel(), g2.ravel()
            pts.append(p)
    return np.unique(np.round(np.vstack(pts), 9), axis=0)


class Body:
    """The robot as sampled volumes at a joint pose: each moving collision capsule (axis points with its radius),
    the tool tip (a point) and each mounted part (surface points of its box). The part list is fixed at
    construction; poses come from RobotModel.fk, so this works for any robot the model loads."""

    def __init__(self, model, tool_bodies, mounted, step=None):
        self.model = model
        self.root = model.mj.body(model._root).name   # the base rests on the plane and never moves
        caps = [c for c in model.collision_capsules(model.q_home) if c[0] != self.root]
        step = min(c[3] for c in caps if c[3] > 0) if step is None else step
        self.step = step
        self.parts = []
        for name, p0, p1, r in caps:
            n = max(2, int(math.ceil(float(np.linalg.norm(p1 - p0)) / step)) + 1)
            self.parts.append({"name": name, "kind": "tool" if name in tool_bodies else "link", "n": n, "r": r})
        self.parts.append({"name": "tip", "kind": "tool", "n": 1, "r": 0.0})   # the touching point itself
        self.mounted = list(mounted)
        self.travel_scale = sum(float(np.linalg.norm(p1 - p0)) + 2.0 * r for _, p0, p1, r in caps)
        self.travel_scale += max((float(np.linalg.norm(mp["c"])) + float(np.linalg.norm(mp["half"])) for mp in self.mounted), default=0)
        for mp in self.mounted:
            mp["local_pts"] = (np.zeros((1, 3)) if mp.get("shape") == "sphere"
                               else box_surface_points(mp["half"], step)) + mp["c"]
            self.parts.append({"name": mp["name"], "kind": "mounted", "n": len(mp["local_pts"]), "r": mp.get("radius", 0.0)})
        self.names = [p["name"] for p in self.parts]
        self.kinds = [p["kind"] for p in self.parts]
        counts = [p["n"] for p in self.parts]
        self.starts = np.concatenate([[0], np.cumsum(counts)[:-1]]).astype(int)
        self.radius = np.concatenate([np.full(p["n"], p["r"]) for p in self.parts])

    def mounted_pose(self, mp, fk):
        """(centre, axes) of a mounted part in frame B from an fk result."""
        pc, Rc = fk["camera"]
        return pc + Rc @ mp["c"], Rc

    def points(self, q):
        """(stacked sample points in frame B, per point radius) at joint pose q."""
        fk = self.model.fk(q)
        links = [v for k, v in fk["links"].items() if k != self.root]
        out = []
        for part, (p0, p1, _r) in zip(self.parts, links):
            t = np.linspace(0.0, 1.0, part["n"])[:, None]
            out.append(p0 + t * (p1 - p0))
        out.append(np.asarray(fk["tip"][0], dtype=float)[None, :])
        pc, Rc = fk["camera"]
        for mp in self.mounted:
            out.append(pc + mp["local_pts"] @ Rc.T)
        return np.vstack(out)

    def clearances(self, q, obstacles):
        """(n_parts, n_obstacles) clearance in m: the least signed distance of the part's volume to the obstacle."""
        return self.clearances_points(self.points(q), obstacles)

    def clearances_points(self, P, obstacles):
        """The same distances using already sampled body points."""
        C = np.empty((len(self.parts), len(obstacles)))
        for j, ob in enumerate(obstacles):
            d = distance(ob, P) - self.radius
            C[:, j] = np.minimum.reduceat(d, self.starts)
        return C


# ---------------------------------------------------------------------------------------------------- paths
def straight_joint_path(q0, q1, length_scale, step=None, body=None):
    """Return every sample, bounding travel of the fastest moving point.

    The sum of joint travel times the chain length bounds point travel between
    samples, including curved motion between equal endpoints. No samples are dropped.
    """
    q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
    if step is None:
        return [q0.copy(), q1.copy()]
    if step <= 0:
        step = getattr(body, "step", length_scale)
    length_scale = max(length_scale, getattr(body, "travel_scale", length_scale))
    travel = float(np.sum(np.abs(q1 - q0))) * length_scale
    n = max(1, math.ceil(travel / step))
    return [q0 + (k / n) * (q1 - q0) for k in range(n + 1)]


def thin_path(path, length_scale, step=None, body=None):
    """Compatibility name: preserve all poses and subdivide each stretch."""
    if not path:
        return []
    out = [np.asarray(path[0], dtype=float)]
    for q in path[1:]:
        out.extend(straight_joint_path(out[-1], q, length_scale, step, body)[1:])
    return out


def iter_path(path, length_scale, spacing, body=None):
    """Every sampled pose, processing one original stretch at a time."""
    if not path:
        return
    yield np.asarray(path[0])
    for q0, q1 in path_pieces(path):
        yield from straight_joint_path(q0, q1, length_scale, spacing, body)[1:]


def path_pieces(path):
    """Consecutive complete stretches, with shared endpoints."""
    for q0, q1 in zip(path, path[1:]):
        yield q0, q1


def check_path(body, path, obstacles, margin, tol, gives=None, spacing=None):
    """Check every sample in consecutive pieces. margin may vary per sample and pair.

    Return (worst, entered), retaining the public result keys of the path check.
    worst is None or the first part and obstacle pair that comes within its margin
    closer than it was at the start by more than tol. It records part, kind,
    obstacle index, least clearance, path percentage, start clearance and margin.
    gives marks volumes drawn behind a touched surface. A part already inside one
    deeper than tol may go up to its margin deeper than the start before it counts:
    the way out of a part the tool just pushed is not refused for grazing its old place.
    entered lists obstacle indices some part went inside, deeper than at the start.
    """
    if not obstacles or len(path) < 2:
        return None, []
    variable = callable(margin)
    reuse_points = hasattr(body, "clearances_points") and hasattr(margin, "for_points")
    first_points = body.points(path[0]) if reuse_points else None
    first = body.clearances_points(first_points, obstacles) if reuse_points else body.clearances(path[0], obstacles)
    first_margin = np.asarray(margin(path[0]) if variable else margin)
    if spacing is None:
        spacing = float(np.min(first_margin)) / 2.0
    if spacing <= 0:
        spacing = body.step
    length = getattr(getattr(body, "model", None), "length_scale", None)
    scale = max(length or 0, getattr(body, "travel_scale", length or 0))
    # Each original stretch is a separate complete piece. Only that piece is
    # materialized; no long-path cap or thinning can remove a sample.
    def pieces():
        for q0, q1 in path_pieces(path):
            if length is None:
                yield [q0, q1]
            else:
                yield straight_joint_path(q0, q1, scale, spacing, body)
    total = sum(max(1, math.ceil(float(np.sum(np.abs(np.asarray(q1) - q0))) * scale / spacing))
                if scale and spacing > 0 else 1 for q0, q1 in path_pieces(path))
    gave = np.zeros(first.shape[1], dtype=bool) if gives is None else np.asarray(gives, dtype=bool)
    entered, worst = set(), None
    k = 0
    for piece in pieces():
        previous = piece[0]
        previous_points = body.points(previous) if reuse_points else None
        for q in piece[1:]:
            k += 1
            for_step = getattr(margin, "for_step", None)
            if reuse_points:
                points = body.points(q)
                C = body.clearances_points(points, obstacles)
                m = np.asarray(margin.for_points(previous, q, previous_points, points))
                previous_points = points
            else:
                C = body.clearances(q, obstacles)
                m = np.asarray(for_step(previous, q) if for_step else margin(q) if variable else margin)
            closer = C < np.where((first < -tol) & gave[None, :], first - m, first - tol)
            entered.update(int(j) for j in np.nonzero(((C < first - tol) & (C < 0)).any(axis=0))[0])
            bad = closer & (C < m)
            if bad.any():
                i, j = min(np.argwhere(bad).tolist(), key=lambda ij: C[ij[0], ij[1]])
                if worst is None or float(C[i, j]) < worst["clearance"]:
                    chosen_margin = float(np.broadcast_to(m, C.shape)[i, j])
                    worst = {"part": body.names[i], "kind": body.kinds[i], "obstacle": int(j),
                             "clearance": float(C[i, j]), "pct": int(round(100.0 * k / total)),
                             "start": float(first[i, j]), "margin": chosen_margin, "q": q.copy(),
                             "previous_q": np.asarray(previous).copy()}
            previous = q
    return worst, sorted(entered)
