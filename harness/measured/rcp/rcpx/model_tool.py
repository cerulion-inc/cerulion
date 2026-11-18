# SPDX-License-Identifier: AGPL-3.0-only
"""What on the tool can touch (the tool part of robot://body, I1), split out of rcpx/model_body.py.

Everything that moves rigidly with the tip as far as the arm joints go (the last arm joint's body and all bodies
under it: the last link, a flange, a hand, its jaws, a paddle) is sampled on the model's own collision geometry, as
the physics collides it (a mesh is its convex hull), and expressed in the tool frame T in mm: x along the finger
(0 at the tip, negative behind it), y the hand's left, z the hand's up. From those points: a box per body, the
tool's outline as cross-sections along the finger, what stands out past each named surface, how far the tool can
tilt before something other than the tip meets a flat surface, the gripper's shape when open, and sentences built
from those numbers. Nothing is typed per robot.

In the document a box or a cross-section is one row of text, "x A..B | y A..B | z A..B" (mm, tool frame), because
the client prints the document one number per line; code reads the numbers through tool_numbers(model) and
parse_row(text), never by hand.
"""

import copy
import math

import mujoco
import numpy as np

MM = 1000.0
SAMPLE_M = 0.002  # the surface samples are at most this far apart
SLAB_MM = 10.0  # one outline cross-section per this much of tool x (half that within FINE_MM of the tip)
FINE_MM = 20.0
MAX_SECTIONS = 8  # the outline is merged down to at most this many sections
MERGE_FREE_MM = 1.0  # neighbouring cross-sections this alike are always merged
NEAR_TIP_MM = 40.0  # a difference this far behind the tip counts a quarter of one at the tip when merging
STEP_OUT_MM = 3.0  # a part "stands out" once it is this much past the tip's own cross-section or a surface
_G = mujoco.mjtGeom
_AXES = (("+y", 1, 1.0, "left"), ("-y", 1, -1.0, "right"), ("+z", 2, 1.0, "top"), ("-z", 2, -1.0, "underside"))


# ---------- surface samples ----------
def _grid(a, b, step):
    n = max(2, int(math.ceil((b - a) / step)) + 1)
    return np.linspace(a, b, n)


def _tri_points(V, F, step):
    """Points on triangles F of vertices V, at most about step apart; every corner and edge is sampled."""
    out = [V]
    A, B, C = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    edge = np.maximum.reduce([np.linalg.norm(B - A, axis=1), np.linalg.norm(C - B, axis=1),
                              np.linalg.norm(A - C, axis=1)])
    n = np.minimum(np.ceil(edge / step).astype(int), 80)
    for k in np.unique(n):
        if k <= 1:
            continue
        sel = n == k
        i, j = np.meshgrid(np.arange(k + 1), np.arange(k + 1), indexing="ij")
        keep = (i + j) <= k
        u, v = i[keep] / k, j[keep] / k
        w = 1.0 - u - v
        P = (A[sel][:, None, :] * w[None, :, None] + B[sel][:, None, :] * u[None, :, None]
             + C[sel][:, None, :] * v[None, :, None])
        out.append(P.reshape(-1, 3))
    return np.vstack(out)


def _hull_faces(m, mid):
    """Triangles (indices into the mesh's own vertices) of the convex hull the physics collides; the mesh's own
    faces when the model holds no hull for it."""
    adr = int(m.mesh_graphadr[mid])
    if adr < 0:
        return m.mesh_face[m.mesh_faceadr[mid]: m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
    g = m.mesh_graph[adr:]
    nv, nf = int(g[0]), int(g[1])
    start = 2 + 3 * nv + 3 * nf  # numvert, numface, vert_edgeadr, vert_globalid, edge_localid, then face_globalid
    return np.asarray(g[start: start + 3 * nf]).reshape(nf, 3)


def _sphere(step_rad):
    out = [[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]]
    for lat in _grid(-math.pi / 2, math.pi / 2, step_rad)[1:-1]:
        n = max(4, int(math.ceil(2 * math.pi * math.cos(lat) / step_rad)))
        for lon in np.linspace(0, 2 * math.pi, n, endpoint=False):
            out.append([math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)])
    return np.array(out)


def _local_points(m, g, step):
    """Surface points of geom g in its own frame."""
    t, s = int(m.geom_type[g]), m.geom_size[g]
    if t == int(_G.mjGEOM_MESH):
        mid = int(m.geom_dataid[g])
        V = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        return _tri_points(np.asarray(V, dtype=float), np.asarray(_hull_faces(m, mid), dtype=int), step)
    if t == int(_G.mjGEOM_BOX):
        out = []
        for k in range(3):
            i, j = (k + 1) % 3, (k + 2) % 3
            a, b = np.meshgrid(_grid(-s[i], s[i], step), _grid(-s[j], s[j], step), indexing="ij")
            for sign in (-1.0, 1.0):
                P = np.zeros((a.size, 3))
                P[:, i], P[:, j], P[:, k] = a.ravel(), b.ravel(), sign * s[k]
                out.append(P)
        return np.vstack(out)
    if t in (int(_G.mjGEOM_CAPSULE), int(_G.mjGEOM_CYLINDER)):
        r, h = float(s[0]), float(s[1])
        ring = np.linspace(0, 2 * math.pi, max(8, int(math.ceil(2 * math.pi * r / step))), endpoint=False)
        side = np.array([[r * math.cos(a), r * math.sin(a), z] for z in _grid(-h, h, step) for a in ring])
        if t == int(_G.mjGEOM_CAPSULE):
            cap = _sphere(step / max(r, 1e-6)) * r
            ends = np.vstack([cap[cap[:, 2] >= 0] + [0, 0, h], cap[cap[:, 2] <= 0] - [0, 0, h]])
        else:
            disk = np.array([[rr * math.cos(a), rr * math.sin(a), 0.0] for rr in _grid(0.0, r, step) for a in ring])
            ends = np.vstack([disk + [0, 0, h], disk - [0, 0, h]])
        return np.vstack([side, ends])
    if t == int(_G.mjGEOM_SPHERE):
        return _sphere(step / max(float(s[0]), 1e-6)) * float(s[0])
    if t == int(_G.mjGEOM_ELLIPSOID):
        return _sphere(step / max(float(max(s)), 1e-6)) * np.asarray(s, dtype=float)
    r = float(m.geom_rbound[g])  # anything else: its bounding sphere
    return _sphere(step / max(r, 1e-6)) * r


def tool_body_ids(model):
    """Bodies that move rigidly with the tip as far as the arm joints go: the last arm joint's body and all under."""
    m, last = model.mj, int(model._jbody[-1])
    out = []
    for b in range(1, m.nbody):
        k = b
        while k not in (0, last):
            k = int(m.body_parentid[k])
        if k == last:
            out.append(b)
    return out


def tool_points(model, gripper=None):
    """{body id: points (mm, tool frame)} of every tool body at the home pose; gripper: joint values to set (the
    spec's closed values when None)."""
    m, d = model.mj, model.d
    model._set(model.q_home)
    g = model.spec.get("gripper")
    if g and gripper is not None:
        for j, v in zip(g["joints"], gripper):
            d.qpos[m.jnt_qposadr[m.joint(j).id]] = float(v)
        mujoco.mj_kinematics(m, d)
    ptip, Rtip = d.site_xpos[model._tip].copy(), d.site_xmat[model._tip].reshape(3, 3).copy()
    out = {}
    for b in tool_body_ids(model):
        gs = [k for k in range(m.ngeom) if m.geom_bodyid[k] == b]
        use = [k for k in gs if m.geom_contype[k] or m.geom_conaffinity[k]] or gs
        if not use:
            continue
        P = np.vstack([_local_points(m, k, SAMPLE_M) @ d.geom_xmat[k].reshape(3, 3).T + d.geom_xpos[k] for k in use])
        out[b] = ((P - ptip) @ Rtip) * MM
    model._set(model.q_home)
    return out


# ---------- numbers ----------
def _lo(v):
    return math.floor(float(v) * 10.0 + 1e-9) / 10.0


def _hi(v):
    return math.ceil(float(v) * 10.0 - 1e-9) / 10.0


def row(lo, hi):
    """'x A..B | y A..B | z A..B' for a box from lo to hi (mm), rounded outward to 0.1 mm."""
    return " | ".join(f"{ax} {_lo(a):.1f}..{_hi(b):.1f}" for ax, a, b in zip("xyz", lo, hi))


def parse_row(text):
    """(lo, hi) arrays of a row written by row(); text after a third '|' (the bodies) is ignored."""
    lo, hi = [], []
    for part in text.split("|")[:3]:
        a, b = part.split()[1].split("..")
        lo.append(float(a))
        hi.append(float(b))
    return np.array(lo), np.array(hi)


def _gripper_joints_over(model, b):
    """Names of the gripper joints body b hangs under (None when it is fixed to the tip's own chain)."""
    g = model.spec.get("gripper")
    if not g:
        return None
    m = model.mj
    jb = {int(m.jnt_bodyid[m.joint(j).id]): j for j in g["joints"]}
    names = []
    while b != 0:
        if b in jb:
            names.append(jb[b])
        b = int(m.body_parentid[b])
    return names or None


def _sections(clouds, names):
    """Cross-sections along tool x, nearest the tip first: [{x, y, z: [lo, hi] mm, bodies}], merged to at most
    MAX_SECTIONS, the least different neighbours first and differences near the tip counting most. Each bound is
    taken over the slab widened by one sample step, so a point between two samples is still inside, and a merged
    section is the envelope of its slabs."""
    P = np.vstack(list(clouds.values()))
    who = np.concatenate([np.full(len(c), b) for b, c in clouds.items()])
    pad = SAMPLE_M * MM
    x_top, x_end = max(float(P[:, 0].max()), 0.0), float(P[:, 0].min())
    runs, hi, lo = [], x_top, -SLAB_MM / 2
    while hi > x_end - 1e-9:
        sel = (P[:, 0] >= lo - pad) & (P[:, 0] <= hi + pad)
        core = (P[:, 0] >= lo) & (P[:, 0] <= hi)
        if core.any():
            Q = P[sel]
            runs.append({"x": [max(lo, x_end), hi], "y": [float(Q[:, 1].min()), float(Q[:, 1].max())],
                         "z": [float(Q[:, 2].min()), float(Q[:, 2].max())], "bodies": set(who[core].tolist())})
        hi, lo = lo, lo - (SLAB_MM / 2 if lo > -FINE_MM + 1e-9 else SLAB_MM)

    def cost(i):
        a, b = runs[i], runs[i + 1]
        diff = max(abs(a[k][j] - b[k][j]) for k in ("y", "z") for j in (0, 1))
        return diff, diff / (1.0 + abs(a["x"][0]) / NEAR_TIP_MM) ** 2

    while len(runs) > 1:
        costs = [cost(i) for i in range(len(runs) - 1)]
        i = int(np.argmin([c[1] for c in costs]))
        if len(runs) <= MAX_SECTIONS and costs[i][0] > MERGE_FREE_MM:
            break
        a, b = runs[i], runs.pop(i + 1)
        a["x"][0] = b["x"][0]
        a["y"] = [min(a["y"][0], b["y"][0]), max(a["y"][1], b["y"][1])]
        a["z"] = [min(a["z"][0], b["z"][0]), max(a["z"][1], b["z"][1])]
        a["bodies"] |= b["bodies"]
    return [{"x": [_lo(r["x"][0]), _hi(r["x"][1])], "y": [_lo(r["y"][0]), _hi(r["y"][1])],
             "z": [_lo(r["z"][0]), _hi(r["z"][1])], "bodies": [names[b] for b in sorted(r["bodies"])]} for r in runs]


def _stand_out(P, who, names, axis, sign, v0):
    """What stands out past v0 (mm, the tip's own cross-section) on one side of the tool: None, or
    {from_behind_tip_mm: where the tool first gets wider, tilt_deg: the tilt toward that side at which a part
    other than the tip meets a flat surface before the tip does, body, behind_tip_mm, out_mm: that part and where
    its touching point is}."""
    rho = sign * P[:, axis]
    out = rho > v0 + STEP_OUT_MM
    if not out.any():
        return None
    s = np.maximum(-P[out, 0], 0.0)
    tilt = np.degrees(np.arctan2(s, rho[out] - v0))
    k = int(np.argmin(tilt))
    return {"from_behind_tip_mm": round(float(s.min()), 1), "tilt_deg": round(float(tilt[k]), 1),
            "body": names[int(who[out][k])], "behind_tip_mm": round(float(s[k]), 1),
            "out_mm": round(float(rho[out][k]), 1)}


def _past_surface(P, surf):
    """(how far other tool parts stand out past a named surface's plane, how far behind the tip that starts), mm;
    (0.0, None) when nothing does or the surface looks along the finger."""
    n = np.asarray(surf["normal_in_tool"], dtype=float)
    if abs(n[0]) > 0.5:
        return 0.0, None
    h = P @ n - float(np.asarray(surf["centre_in_tool_mm"], dtype=float) @ n)
    out = h > STEP_OUT_MM
    if not out.any():
        return 0.0, None
    return _hi(h[out].max()), round(max(float((-P[out, 0]).min()), 0.0), 1)


def _words(sec0, steps, lead_body, ahead_mm):
    y, z = sec0["y"], sec0["z"]
    out = [f"Within {abs(sec0['x'][0]):.0f} mm of the tip the tool is {y[1] - y[0]:.0f} mm wide (tool y "
           f"{y[0]:.0f} to {y[1]:.0f}) and {z[1] - z[0]:.0f} mm deep (tool z {z[0]:.0f} to {z[1]:.0f})."]
    groups, free = [], []
    for key, _, _, word in _AXES:
        st = steps[key]
        if st is None:
            free.append(word)
            continue
        for g in groups:  # sides that read the same once rounded share a sentence
            if (g["body"] == st["body"] and round(g["tilt"]) == round(st["tilt_deg"])
                    and round(g["out"]) == round(st["out_mm"]) and round(g["s"]) == round(st["behind_tip_mm"])):
                g["sides"].append(f"{word} (tool {key})")
                break
        else:
            groups.append({"body": st["body"], "out": st["out_mm"], "s": st["behind_tip_mm"],
                           "tilt": st["tilt_deg"], "sides": [f"{word} (tool {key})"]})
    for g in groups:
        out.append(f"Tilted {g['tilt']:.0f} deg or more toward its {' or '.join(g['sides'])}, the {g['body']} "
                   f"({g['out']:.0f} mm out from the finger axis, {g['s']:.0f} mm behind the tip) meets a flat surface "
                   f"before the tip does.")
    if free:
        out.append("Nothing stands out past the tip's own cross-section on its " + ", ".join(free) + ".")
    first = ("the tip" if ahead_mm < 0.5 else f"the {lead_body}, {ahead_mm:.1f} mm ahead of the tip point,")
    starts = [(s["from_behind_tip_mm"], k) for k, s in steps.items() if s]
    if starts:
        n, k = min(starts)
        out.append(f"Pointing straight at a surface {first} touches first and the finger is the only part within "
                   f"{n:.0f} mm of the tip; behind that the tool gets wider (first toward tool {k}).")
    else:
        out.append(f"Pointing straight at a surface, or tilted, {first} touches first.")
    return " ".join(out)


def tool_numbers(model):
    """The tool's shape as numbers (mm, tool frame, gripper closed): {bodies: [{name, lo, hi, moved_by}], lo, hi,
    sections: [{x, y, z, bodies}], stands_out: {'+y' | '-y' | '+z' | '-z': None | {...}}, lead_body, ahead_mm,
    points, who, names}; None when the tool has no geometry. Worked out once per model and shared: read only."""
    if getattr(model, "_tool_numbers", None) is not None:
        return model._tool_numbers or None
    m = model.mj
    clouds = tool_points(model)
    if not clouds:
        model._tool_numbers = {}
        return None
    names = {b: m.body(b).name for b in clouds}
    P = np.vstack(list(clouds.values()))
    who = np.concatenate([np.full(len(c), b) for b, c in clouds.items()])
    near = P[P[:, 0] >= -SLAB_MM / 2]
    steps = {key: _stand_out(P, who, names, axis, sign, float((sign * near[:, axis]).max()))
             for key, axis, sign, _ in _AXES}
    model._tool_numbers = {
        "bodies": [{"name": names[b], "lo": c.min(axis=0), "hi": c.max(axis=0),
                    "moved_by": _gripper_joints_over(model, b)} for b, c in clouds.items()],
        "lo": P.min(axis=0), "hi": P.max(axis=0), "sections": _sections(clouds, names), "stands_out": steps,
        "lead_body": names[int(who[int(np.argmax(P[:, 0]))])], "ahead_mm": max(float(P[:, 0].max()), 0.0),
        "points": P, "who": who, "names": names, "clouds": clouds}
    return model._tool_numbers


def tool_shape(model, surfaces):
    """Keys for the tool block of robot://body: bodies, moved_by_gripper, extent, outline, shape_in_words (the
    sentences carry what stands out on each side and the tilt at which it touches first; the numbers behind them
    are tool_numbers(model)["stands_out"]); each surface in `surfaces` (the tool block's own list) gains
    overhang_mm and overhang_from_behind_tip_mm."""
    N = tool_numbers(model)
    if N is None:
        return {}
    for s in surfaces:
        s["overhang_mm"], s["overhang_from_behind_tip_mm"] = _past_surface(N["points"], s)
    return {
        "shape_note": "everything that moves with the tip (gripper closed), from the model's collision geometry, "
                      "tool frame, mm. A row reads x from..to | y from..to | z from..to; x is 0 at the tip and "
                      "negative behind it. bodies: the box around each body (moved_by_gripper: the ones a gripper "
                      "joint moves). outline: over each stretch of x, nearest the tip first, the whole tool lies "
                      "inside the y and z ranges (the bodies there are named last). A surface's overhang_mm is "
                      "how far other tool parts stand out past its plane, starting overhang_from_behind_tip_mm "
                      "behind the tip",
        "bodies": {b["name"]: row(b["lo"], b["hi"]) for b in N["bodies"]},
        "moved_by_gripper": {b["name"]: ", ".join(b["moved_by"]) for b in N["bodies"] if b["moved_by"]},
        "extent": row(N["lo"], N["hi"]),
        "outline": [" | ".join([f"x {s['x'][0]:.1f}..{s['x'][1]:.1f}", f"y {s['y'][0]:.1f}..{s['y'][1]:.1f}",
                                f"z {s['z'][0]:.1f}..{s['z'][1]:.1f}", ", ".join(s["bodies"])]) for s in N["sections"]],
        "shape_in_words": _words(N["sections"][0], N["stands_out"], N["lead_body"], N["ahead_mm"]),
    }


def touch_points(model):
    """[(label, body id, point in that body's own frame, m)]: for every tool body, the middle of the part that
    sticks out farthest along the finger and on each side of the tool frame ("hand +y", "wrist_3_link -z"): the
    places a contact off the tip is most likely to be. A contact locator can try each point's Jacobian against the
    measured joint torques, beside the spec's named surfaces."""
    m, d = model.mj, model.d
    N = tool_numbers(model)
    clouds = N["clouds"] if N else {}
    model._set(model.q_home)
    ptip, Rtip = d.site_xpos[model._tip].copy(), d.site_xmat[model._tip].reshape(3, 3).copy()
    out = []
    for b, P in clouds.items():
        for key, axis, sign in (("+x", 0, 1.0),) + tuple((k, a, sg) for k, a, sg, _ in _AXES):
            v = sign * P[:, axis]
            far = P[v >= v.max() - 0.5].mean(axis=0)  # the middle of the face, edge or corner that leads that way
            pw = ptip + Rtip @ (far / MM)
            out.append((f"{m.body(b).name} {key}", int(b), d.xmat[b].reshape(3, 3).T @ (pw - d.xpos[b])))
    return out


def gripper_shapes(model):
    """Keys for the gripper block: closed_shape and open_shape (a box per body the gripper joints move; open also
    the whole tool's extent) and shape_in_words."""
    g = model.spec.get("gripper")
    if not g:
        return {}
    if getattr(model, "_gripper_shapes", None) is not None:
        return copy.deepcopy(model._gripper_shapes)
    m = model.mj
    out, clouds_by = {}, {}
    for state in ("closed", "open"):
        clouds = tool_points(model, [float(x) for x in g[state]])
        moved = {b: c for b, c in clouds.items() if _gripper_joints_over(model, b)}
        if not moved:
            model._gripper_shapes = {}
            return {}
        P = np.vstack(list(clouds.values()))
        clouds_by[state] = (P, np.vstack(list(moved.values())))
        out[state + "_shape"] = {"jaws": {m.body(b).name: row(c.min(axis=0), c.max(axis=0)) for b, c in moved.items()}}
        if state == "open":
            out["open_shape"]["tool_extent"] = row(P.min(axis=0), P.max(axis=0))
    (Pc, Jc), (Po, Jo) = clouds_by["closed"], clouds_by["open"]
    k = 1 + int(np.argmax(np.abs(Jo.max(axis=0) - Jc.max(axis=0))[1:] + np.abs(Jo.min(axis=0) - Jc.min(axis=0))[1:]))
    ax = "xyz"[k]
    ahead = float(Jo[:, 0].max())
    where = ("end level with the tip" if abs(ahead) < 0.5 else f"reach {ahead:.0f} mm past the tip" if ahead > 0
             else f"end {-ahead:.0f} mm behind the tip")
    out["shape_in_words"] = (
        f"The jaws open along tool {ax}. Closed, the bodies the gripper moves span tool {ax} {Jc[:, k].min():.0f} to "
        f"{Jc[:, k].max():.0f} mm; open, {Jo[:, k].min():.0f} to {Jo[:, k].max():.0f} mm, where they {where} and the "
        f"whole tool spans tool {ax} {Po[:, k].min():.0f} to {Po[:, k].max():.0f} mm "
        f"({Po[:, k].max() - Po[:, k].min():.0f} mm). The tool block describes the closed shape.")
    model._gripper_shapes = out
    return copy.deepcopy(out)


# ---------- hulls, fit errors and the tool's own ratings (SPEC-R4 A8.4, Q2, Q4) ----------
def _curve_tol(m, g):
    """How far the sampled points of a curved shape can stand inside its true surface (m): the sagitta of two
    samples SAMPLE_M apart on its largest radius. A box and a mesh are their own corners: zero."""
    t = int(m.geom_type[g])
    if t in (int(_G.mjGEOM_BOX), int(_G.mjGEOM_MESH)):
        return 0.0
    r = float(max(m.geom_size[g])) if t == int(_G.mjGEOM_ELLIPSOID) else float(m.geom_size[g][0])
    r = r if r > 0 else float(m.geom_rbound[g])
    return r * (1.0 - math.cos(min(SAMPLE_M / (2.0 * r), math.pi))) if r > 0 else 0.0


def body_hulls(model):
    """{body name: {"points_m": N x 3 in that body's own frame, "tol_m"}} for the base and every body the arm
    moves: points whose convex hull is the body's own collision shape (a mesh: its vertices, which the physics
    collides as their hull; a box: its corners; a curved shape: surface samples, tol_m then says how far they can
    stand inside the true surface). A mesh states no tolerance of its own in these files: zero. Worked out once."""
    if getattr(model, "_hulls", None) is None:
        m, d = model.mj, model.d
        model._set(model.q_home)
        out = {}
        for name, b, _, _, _ in model._capsule_defs:
            gs = [k for k in range(m.ngeom) if m.geom_bodyid[k] == b]
            use = [k for k in gs if m.geom_contype[k] or m.geom_conaffinity[k]] or gs
            Rb, pb = d.xmat[b].reshape(3, 3), d.xpos[b]
            pts, tol = [], 0.0
            for k in use:
                if int(m.geom_type[k]) == int(_G.mjGEOM_MESH):
                    mid = int(m.geom_dataid[k])
                    P = np.asarray(m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]], float)
                else:
                    P = _local_points(m, k, SAMPLE_M)
                pts.append(((P @ d.geom_xmat[k].reshape(3, 3).T + d.geom_xpos[k]) - pb) @ Rb)
                tol = max(tol, _curve_tol(m, k))
            if pts:
                out[name] = {"points_m": np.vstack(pts), "tol_m": tol}
        model._hulls = out
    return model._hulls


def _to_segment(P, p0, p1):
    """Distance (m) of each point of P from the segment p0 to p1."""
    d = p1 - p0
    L2 = float(d @ d)
    t = np.clip((P - p0) @ d / L2, 0.0, 1.0) if L2 > 0 else np.zeros(len(P))
    return np.linalg.norm(P - (p0 + np.outer(t, d)), axis=1)


def fit_errors(model):
    """{body name: m}: the farthest a hull point of the body stands out of that body's capsule (zero when the
    capsule covers it), plus the hull's own tolerance."""
    hulls = body_hulls(model)
    out = {}
    for name, _, p0, p1, r in model._capsule_defs:
        if name in hulls:
            far = float(np.max(_to_segment(hulls[name]["points_m"], np.asarray(p0), np.asarray(p1)))) - float(r)
            out[name] = max(far, 0.0) + float(hulls[name]["tol_m"])
    return out


def tool_lever_m(model):
    """The tool's lever arm (m): from the wrist centre (the anchor of the last arm joint; the joint before it when the
    tip stands on that anchor) to the tip, at home."""
    tip = model.fk_tip(model.q_home)[0]
    for anchor, _ in reversed(model.joint_frames(model.q_home)):
        lever = float(np.linalg.norm(tip - anchor))
        if lever > 0:
            return lever
    return float(model.length_scale)


def tool_force(model):
    """(F_tool in N or None, how): the tool's rating, only where its maker or owner states one (spec tool_rating_n).
    With none stated the answer is None: a jaw motor's force range is a grip force (jaw_force) and the wrist's
    torque over the tool's lever is the arm's own push; neither is a contact or collision rating. Nothing
    the robot measures raises it."""
    spec = model.spec
    if spec.get("tool_rating_n") is not None:
        return float(spec["tool_rating_n"]), "the tool's stated rating"
    return None, "no tool rating is stated (spec tool_rating_n): the jaws' own force is a grip force, not a rating"


def jaw_force(model):
    """(N or None, how): the squeeze the jaw actuator can give at the pad, the weakest jaw (a sliding jaw: its force
    range; a hinged jaw: its torque range over the pad's lever arm, the tip's distance from the hinge axis). A grip
    force, never a rating of the tool. None for a hand with no jaws."""
    spec, m = model.spec, model.mj
    g = spec.get("gripper")
    if not g or not g.get("forcerange"):
        return None, "no jaws"
    model._set(model.q_home)
    tip = model.d.site_xpos[model._tip].copy()
    forces = []
    for j, fr in zip(g["joints"], g["forcerange"]):
        jid = m.joint(j).id
        if int(m.jnt_type[jid]) == int(mujoco.mjtJoint.mjJNT_SLIDE):
            forces.append(float(fr))
            continue
        rel = tip - model.d.xanchor[jid]
        axis = model.d.xaxis[jid]
        lever = float(np.linalg.norm(rel - axis * float(rel @ axis)))
        if lever > 0:
            forces.append(float(fr) / lever)
    if not forces:
        return None, "no jaw with a lever"
    return min(forces), "the jaw actuator's force range carried to the pad through the finger's lever"


def hand_weight(model):
    """(F_hand in N, mass kg): g times the mass of every body the last arm joint moves."""
    m = model.mj
    mass = float(sum(m.body_mass[b] for b in tool_body_ids(model)))
    return mass * float(np.linalg.norm(m.opt.gravity)), mass
