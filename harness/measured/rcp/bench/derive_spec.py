# SPDX-License-Identifier: AGPL-3.0-only
"""derive_spec(mjcf_path, name) -> robot_spec from the model file alone.

No robot is named here and no robot number is typed. Every rule below runs the same way on any serial arm:

- chain: the path from the root body to the deepest body; its hinge and slide joints in order. The end of the chain
  is a gripper when it is a pair of sibling slide fingers, or a hinge finger whose geometry swings off its axis
  while the parent body carries a fixed finger reaching as far (a jaw). Arm joints are the rest of the path.
- tool direction: the last arm joint's axis when the distal geometry lies along it (a roll), else the direction
  from that joint to the distal geometry, square to its axis.
- tip: the centre of the most distal slab of geometry along the tool direction: the fixed jaw for a hinge jaw, the
  closed fingers for slide fingers. An arm with no finger gets the bench's standard paddle on the flange (the
  flange is the far face of the last link on the tool axis) and the tip is the paddle's end.
- tool surfaces: the faces of the bounding box of the distal geometry in the tool frame, near the tip.
- camera: behind and above the tip, clear of the hand's own geometry, through bench.robots._build.camera_on_tool.
- q_home: a seeded search over joint space for a pose with no contact, every moving geom above the mounting plane,
  the tip out in front, the tool pointing forward and down, the arm well away from singularities and limits; the
  best candidates are then checked with rcpx.model.RobotModel (ik with the tool pointing down at the home tip).
- kp, kd, torque limits: the MJCF actuators (position gains, forcerange, motor ctrlrange) on each joint, made
  stable for the explicit servo law at 0.002 s physics by the joint inertia; joints with no actuator get gains
  and limits from their inertia and the gravity load they carry.

The returned spec carries spec["source"] (field -> "derived" | "person"), spec["derivation"] (field -> how) and
spec["overrides"] (every field a person supplied, with why). A model file the loader cannot take as it is (an
unbounded hinge, a name the loader adds) is copied to runs/derived_mjcf/ with the fix listed in spec["model_fixes"].
"""

import hashlib
import math
import os

import mujoco
import numpy as np

from bench.robots import _build
from bench.robots import _gains as _gains_mod

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CACHE_DIR = os.path.join(ROOT, "runs", "derived_mjcf")
PADDLE_MM = (90.0, 20.0, 12.0)  # the bench's standard bolt-on paddle for a flange with no finger
PADDLE_KG = 0.05
CAMERA_FOVY_DEG = 65.0  # the bench's standard wrist lens
CAMERA_CLEAR_M = 0.01  # camera clearance above the hand's geometry
CAMERA_UP_OVER_BACK = 0.6  # camera height over setback: puts the tip low in the frame, axis along the finger
SLAB_M = 0.003  # thickness of the distal slab whose centre is the tip
PAD_MAX_M = 0.04  # tool surface pads cover at most this much behind the tip
HOME_SAMPLES = 3000
RESERVED = {"camera": ("wrist",), "site": ("tip",), "body": ("finger_tool",)}

_HINGE, _SLIDE = int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)
_MESH, _BOX = int(mujoco.mjtGeom.mjGEOM_MESH), int(mujoco.mjtGeom.mjGEOM_BOX)
_CAPSULE, _CYLINDER = int(mujoco.mjtGeom.mjGEOM_CAPSULE), int(mujoco.mjtGeom.mjGEOM_CYLINDER)


# ---------- the model file ----------
def _prepare_mjcf(mjcf_path, name):
    """Path the loader can take, plus the fixes made. Unbounded hinges get +-pi; reserved names get a suffix."""
    s = mujoco.MjSpec.from_file(mjcf_path)
    m = s.compile()
    fixes = []
    half = 180.0 if s.compiler.degree else math.pi
    for j in s.joints:
        jid = m.joint(j.name).id
        if m.jnt_type[jid] == _HINGE and not m.jnt_limited[jid]:
            j.range = [-half, half]
            j.limited = mujoco.mjtLimited.mjLIMITED_TRUE
            fixes.append({"joint": j.name, "fix": "range +-180 deg", "continuous": True,
                          "why": "a continuous joint has no travel limit (the spec lists it under continuous); the "
                                 "bench's scene builder and loader read a range from the file, so this copy carries "
                                 "one turn as a working range, which is not a limit of the robot"})
    for kind, names in RESERVED.items():
        for el in getattr(s, kind + "s") if kind != "body" else s.bodies:
            if el.name in names:
                fixes.append({kind: el.name, "fix": "renamed " + el.name + "_mjcf",
                              "why": "the loader adds a " + kind + " of this name"})
                el.name = el.name + "_mjcf"
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    pairs = set()
    for c in d.contact[: d.ncon]:
        b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
        if m.body_parentid[b1] == b2 or m.body_parentid[b2] == b1:
            pairs.add((m.body(min(b1, b2)).name, m.body(max(b1, b2)).name))
    for b1, b2 in sorted(pairs):  # a jointed pair touching at its own bearing (the parent is welded to the world)
        e = s.add_exclude()
        e.bodyname1, e.bodyname2 = b1, b2
        fixes.append({"exclude": [b1, b2], "fix": "contact exclude", "why": "parent and child touch at the joint "
                      "at the reference pose; the parent is fixed to the world so MuJoCo's parent filter skips it"})
    if not fixes:
        return os.path.abspath(mjcf_path), fixes
    base = os.path.dirname(os.path.abspath(mjcf_path))
    s.meshdir = os.path.join(base, s.meshdir or "")
    s.texturedir = os.path.join(base, s.texturedir or "")
    s.compile()
    xml = s.to_xml()
    os.makedirs(CACHE_DIR, exist_ok=True)
    out = os.path.join(CACHE_DIR, f"{name}_{hashlib.sha256(xml.encode()).hexdigest()[:10]}.xml")
    if not os.path.exists(out):
        tmp = f"{out}.{os.getpid()}.tmp"
        with open(tmp, "w") as f:
            f.write(xml)
        os.replace(tmp, out)  # in one step: a process started beside this one never loads a half-written file
    return out, fixes


# ---------- geometry ----------
def _collides(m, g):
    return m.geom_contype[g] != 0 or m.geom_conaffinity[g] != 0


def _geom_points(m, d, g):
    """Points (world) of one geom: mesh vertices or box/primitive corners."""
    R, p = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
    t, s = int(m.geom_type[g]), m.geom_size[g]
    if t == _MESH:
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
    else:
        if t == _BOX:
            h = s
        elif t in (_CAPSULE, _CYLINDER):
            h = [s[0], s[0], s[1] + (s[0] if t == _CAPSULE else 0.0)]
        else:
            h = [m.geom_rbound[g]] * 3
        v = np.array([[a * h[0], b * h[1], c * h[2]] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])
    return v @ R.T + p


def _bodies_points(m, d, bodies):
    gs = [g for g in range(m.ngeom) if m.geom_bodyid[g] in bodies]
    use = [g for g in gs if _collides(m, g)] or gs
    return np.vstack([_geom_points(m, d, g) for g in use]) if use else np.zeros((0, 3))


def _subtree(m, b):
    out, todo = [], [b]
    while todo:
        x = todo.pop()
        out.append(x)
        todo.extend(int(c) for c in range(m.nbody) if m.body_parentid[c] == x and c != x)
    return out


def _depth(m, b):
    k = 0
    while b != 0:
        b, k = m.body_parentid[b], k + 1
    return k


def _body_joints(m, b):
    return [int(j) for j in range(m.body_jntadr[b], m.body_jntadr[b] + m.body_jntnum[b])
            if m.jnt_type[j] in (_HINGE, _SLIDE)] if m.body_jntnum[b] > 0 else []


def _to_body(d, b, p_world=None, v_world=None):
    R, p = d.xmat[b].reshape(3, 3), d.xpos[b]
    if p_world is not None:
        return R.T @ (np.asarray(p_world) - p)
    return R.T @ np.asarray(v_world)


# ---------- chain and gripper ----------
def _chain(m, d):
    moving = [b for b in range(1, m.nbody) if any(_body_joints(m, a) for a in _ancestry(m, b))]
    deepest = min(moving, key=lambda b: (-_depth(m, b), b))
    path = list(reversed(_ancestry(m, deepest)))  # root first
    joints = [j for b in path for j in _body_joints(m, b)]
    last = joints[-1]
    L = int(m.jnt_bodyid[last])
    P = int(m.body_parentid[L])
    fingers, how = [], None
    if m.jnt_type[last] == _SLIDE:
        sib = [c for c in range(m.nbody) if m.body_parentid[c] == P and c != P]
        fingers = [c for c in sib if len(_body_joints(m, c)) == 1 and m.jnt_type[_body_joints(m, c)[0]] == _SLIDE]
        how = "slide fingers on one palm"
    else:
        a, ax = d.xanchor[last], d.xaxis[last]
        F = _bodies_points(m, d, _subtree(m, L))
        Pp = _bodies_points(m, d, [P])
        if len(F) and len(Pp):
            c = F.mean(axis=0) - a
            dirf = c / max(np.linalg.norm(c), 1e-9)
            swings = abs(float(ax @ dirf)) < 0.6  # the finger lies across its hinge axis, not along it
            ext_f = float(((F - a) @ dirf).max())
            ext_p = float(((Pp - a) @ dirf).max())
            if swings and ext_p > 0.5 * ext_f and ext_f > 0:
                fingers, how = [L], "hinge jaw against a fixed finger"
    gj = [j for f in fingers for j in _body_joints(m, f)]
    arm = [j for j in joints if j not in gj]
    return arm, gj, fingers, (P if fingers else int(m.jnt_bodyid[arm[-1]])), how


def _ancestry(m, b):
    out = []
    while b != 0:
        out.append(int(b))
        b = m.body_parentid[b]
    return out


def _finger_gap(m, d, fingers, palm, gj):
    """Gap (m) between the gripper's fingertips: the distal halves of the two opposing fingers' geometry."""
    if len(fingers) >= 2:
        A, B = _bodies_points(m, d, [fingers[0]]), _bodies_points(m, d, fingers[1:])
        a = (A.mean(axis=0) + B.mean(axis=0)) / 2.0
        ax = d.xaxis[gj[0]]
        return float(np.min(np.abs((A - a) @ ax)) + np.min(np.abs((B - a) @ ax)))
    a = d.xanchor[gj[0]]
    A = _bodies_points(m, d, _subtree(m, fingers[0]))
    B = _bodies_points(m, d, [palm])
    dirf = A.mean(axis=0) - a
    dirf /= np.linalg.norm(dirf)
    sa, sb = (A - a) @ dirf, (B - a) @ dirf
    A, B = A[sa >= 0.5 * sa.max()], B[sb >= 0.5 * sa.max()]
    if not len(A) or not len(B):
        return 1.0
    return float(np.min(np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2)))


def _gripper_ends(m, d, fingers, palm, gj):
    """(open, closed) per gripper joint: open has the widest fingertip gap, closed is the first touch from there."""
    lo = np.array([m.jnt_range[j][0] for j in gj])
    hi = np.array([m.jnt_range[j][1] for j in gj])
    qa = [m.jnt_qposadr[j] for j in gj]
    gaps = []
    for t in np.linspace(0.0, 1.0, 41):
        d.qpos[qa] = lo + t * (hi - lo)
        mujoco.mj_kinematics(m, d)
        gaps.append((t, _finger_gap(m, d, fingers, palm, gj)))
    t_open = max(gaps, key=lambda x: x[1])[0]
    sweep = gaps[::-1] if t_open > 0.5 else gaps  # from the open end toward the closing end
    shut = [t for t, g in sweep if g <= 0.002]  # the first fingertip touch
    t_closed = shut[0] if shut else min(gaps, key=lambda x: x[1])[0]
    d.qpos[qa] = lo + t_closed * (hi - lo)
    mujoco.mj_kinematics(m, d)
    return list(lo + t_open * (hi - lo)), list(lo + t_closed * (hi - lo))


# ---------- tool ----------
def _tool_direction(m, d, arm, group_pts):
    """Tool x in world. A roll points away from the joint before it (along the axis); with no offset along the
    axis to tell, toward the distal geometry. A last joint the geometry lies across points toward that geometry."""
    ja = arm[-1]
    a, ax = d.xanchor[ja], d.xaxis[ja]
    v = group_pts.mean(axis=0) - a
    along = float(v @ ax)
    if abs(along) > 0.5 * np.linalg.norm(v):
        step = float((a - d.xanchor[arm[-2]]) @ ax) if len(arm) > 1 else 0.0
        if abs(step) > 0.001:
            return ax * (1.0 if step > 0 else -1.0), "last joint axis (a roll), away from the joint before it"
        return ax * (1.0 if along > 0 else -1.0), "last joint axis (a roll), toward the distal geometry"
    w = v - ax * along
    return w / np.linalg.norm(w), "from the last joint toward the distal geometry, square to its axis"


def _slab_tip(pts, origin, direction, on_axis=False):
    s = (pts - origin) @ direction
    smax = float(s.max())
    slab = pts[s >= smax - SLAB_M]
    if on_axis:
        return origin + direction * smax
    c = slab.mean(axis=0)
    return c + direction * (smax - float((c - origin) @ direction))


def _opening_axis(m, d, gj, fingers, tip_w):
    j = gj[0]
    if m.jnt_type[j] == _SLIDE:
        return d.xaxis[j].copy()
    return np.cross(d.xaxis[j], tip_w - d.xanchor[j])


def _surfaces(points_tool, body, tip_pos, R_tool):
    """Tool faces from the bounding box of the distal geometry near the tip, in the tip body."""
    P = points_tool
    depth = float(-P[:, 0].min())
    pad = min(PAD_MAX_M, 0.5 * depth) if depth > 1e-4 else PAD_MAX_M
    near = P[P[:, 0] >= -pad]
    if len(near) < 4:
        near = P
    (y0, z0), (y1, z1) = near[:, 1:].min(axis=0), near[:, 1:].max(axis=0)
    yc, zc, xc = (y0 + y1) / 2, (z0 + z1) / 2, -pad / 2
    faces = [("tip", [0, 0, 0], [1, 0, 0], [y1 - y0, z1 - z0]),
             ("pad_top", [xc, yc, z1], [0, 0, 1], [pad, y1 - y0]),
             ("pad_bottom", [xc, yc, z0], [0, 0, -1], [pad, y1 - y0]),
             ("side_left", [xc, y1, zc], [0, 1, 0], [pad, z1 - z0]),
             ("side_right", [xc, y0, zc], [0, -1, 0], [pad, z1 - z0])]
    out = []
    for nm, p, n, size in faces:
        pb = np.asarray(tip_pos) + R_tool @ np.asarray(p, dtype=float)
        nb = R_tool @ np.asarray(n, dtype=float)
        out.append({"name": nm, "body": body, "pos": [round(float(v), 5) for v in pb],
                    "normal": [round(float(v), 6) for v in nb],
                    "size_mm": [round(max(float(v) * 1000.0, 1.0), 1) for v in size]})
    return out


def _camera(tip, points_tool):
    """camera_on_tool with setback and height taken from the hand's geometry behind the tip."""
    up = 0.02
    for _ in range(4):
        back = max(up / CAMERA_UP_OVER_BACK, 0.05)
        behind = points_tool[(points_tool[:, 0] >= -back - 0.01) & (points_tool[:, 0] <= 0.0)]
        top = float(behind[:, 2].max()) if len(behind) else 0.0
        up = max(top + CAMERA_CLEAR_M, 0.02)
    back = max(up / CAMERA_UP_OVER_BACK, 0.05)
    return _build.camera_on_tool(tip, back_m=back, up_m=up, fovy_deg=CAMERA_FOVY_DEG), back, up


# ---------- home pose ----------
def _lowest_z(m, d, g, need=None):
    """Lowest z of geom g: the bounding box first, the exact vertices when the bound is under need."""
    c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
    R = d.geom_xmat[g].reshape(3, 3)
    z = float((d.geom_xpos[g] + R @ c)[2] - np.abs(R[2]) @ h)
    if need is not None and z >= need:
        return z
    return float(_geom_points(m, d, g)[:, 2].min()) if int(m.geom_type[g]) == _MESH else z


def _home_candidates(spec, lscale, want_z_up):
    """Seeded joint-space samples that pass the hard rules, best score first: [(score, q)]."""
    from bench.robots import home_qpos, load_robot_mjcf

    m = load_robot_mjcf(spec).compile()
    d = mujoco.MjData(m)
    jids = [m.joint(j).id for j in spec["arm_joints"]]
    qa = [m.jnt_qposadr[j] for j in jids]
    da = [m.jnt_dofadr[j] for j in jids]
    lo = np.array([m.jnt_range[j][0] for j in jids])
    hi = np.array([m.jnt_range[j][1] for j in jids])
    qfull = home_qpos(m, spec)
    tip = m.site("tip").id
    root = int(m.body_rootid[m.jnt_bodyid[jids[0]]])
    moving = [g for g in range(m.ngeom) if m.geom_bodyid[g] != root and _collides(m, g)]
    # geoms the first joint alone carries cannot change height: they only need to stay above the plane
    first_only = {g for g in moving if {jj for b in _ancestry(m, int(m.geom_bodyid[g])) for jj in _body_joints(m, b)}
                  == {jids[0]}}
    rng = np.random.default_rng(0)
    jp = np.zeros((3, m.nv))
    out = []
    for _ in range(HOME_SAMPLES):
        q = lo + (hi - lo) * (0.1 + 0.8 * rng.random(len(jids)))
        d.qpos[:] = qfull
        d.qpos[qa] = q
        mujoco.mj_forward(m, d)
        if d.ncon:
            continue
        p = d.site_xpos[tip]
        a0 = d.xanchor[jids[0]]
        r = math.hypot(p[0] - a0[0], p[1] - a0[1])
        if p[2] < 0.08 * lscale or r < 0.3 * lscale:
            continue
        clear = 0.04 * lscale
        if any(_lowest_z(m, d, g, 0.0 if g in first_only else clear) < (0.0 if g in first_only else clear)
               for g in moving):
            continue
        Rt = d.site_xmat[tip].reshape(3, 3)
        e_r = np.array([p[0] - a0[0], p[1] - a0[1], 0.0]) / r
        down_fwd = 0.5 * e_r - math.sqrt(0.75) * np.array([0, 0, 1.0])  # 60 deg below the horizon
        mujoco.mj_jacSite(m, d, jp, None, tip)
        smin = float(np.linalg.svd(jp[:, da], compute_uv=False)[-1]) / lscale
        margin = float(np.min(np.minimum(q - lo, hi - q) / (hi - lo))) * 2.0
        score = (smin + 0.5 * float(Rt[:, 0] @ down_fwd) + 0.3 * margin - 0.3 * abs(r / lscale - 0.4)
                 - 0.5 * abs(p[2] / lscale - 0.2))
        if want_z_up:
            score += 0.3 * abs(float(Rt[:, 2][2]))
        out.append((score, q.copy()))
    out.sort(key=lambda x: -x[0])
    return out


# ---------- gains (bench/robots/_gains.py; the old names stay importable from here) ----------
_joint_actuator, _file_gains, _inertia_and_gravity, _stable = (_gains_mod.joint_actuator, _gains_mod.file_gains,
                                                              _gains_mod.inertia_and_gravity, _gains_mod.stable)
DT_PHYS, KD_CAP, ZETA_MIN = _gains_mod.DT_PHYS, _gains_mod.KD_CAP, _gains_mod.ZETA_MIN


def _gains(m_raw, spec, joints):
    """(kp, kd, torque limits, how): bench.robots._gains.gains without its list of assumed joints."""
    return _gains_mod.gains(m_raw, spec, joints)[:4]


# ---------- the spec ----------
def _assemble(name, path, arm_names, gripper, tip, finger_tool, q_home, pts_tool, gains):
    cam, back, up = _camera(tip, pts_tool)
    Rt = _build.mat_from_xz(tip["x_axis"], tip["z_axis"])
    spec = {"name": name, "mjcf": path, "arm_joints": arm_names, "gripper": gripper, "tip": tip,
            "tool_surfaces": _surfaces(pts_tool, tip["body"], tip["pos"], Rt), "finger_tool": finger_tool,
            "camera": cam, "q_home": [round(float(v), 5) for v in q_home]}
    spec.update(gains)
    return spec, back, up


def derive_spec(mjcf_path, name, overrides=None, ratings=None):
    """robot_spec of a serial arm from its MJCF alone. overrides: {field: (value, why)} a person supplies. ratings:
    {"speed_limit": ([rad/s per arm joint], "the document and the figure")}: the maker's RATED joint speeds with the
    data sheet named (spec["source"]["speed_limit"] = "rating: ..."); with none, the file's own (a velocity servo) or
    None per joint, which the robot's sweep then measures. Continuous joints are listed in spec["continuous"]; a
    torque limit or gain the file does not state is named in spec["assumed"] and in spec["source"]."""
    from rcpx.model import RobotModel

    path, fixes = _prepare_mjcf(mjcf_path, name)
    m = mujoco.MjSpec.from_file(path).compile()
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    arm, gj, fingers, palm, grip_how = _chain(m, d)
    gripper = None
    if gj:
        g_open, g_closed = _gripper_ends(m, d, fingers, palm, gj)
    ja = arm[-1]
    jb = int(m.jnt_bodyid[ja])
    group = _bodies_points(m, d, _subtree(m, jb))
    xw, dir_how = _tool_direction(m, d, arm, group)
    roll = abs(float(xw @ d.xaxis[ja])) > 0.99
    if gj:
        src = _bodies_points(m, d, fingers if m.jnt_type[gj[0]] == _SLIDE else [palm])
        tip_w = _slab_tip(src, d.xanchor[ja], xw)
        yw = _opening_axis(m, d, gj, fingers, tip_w)
        yw = yw - xw * float(yw @ xw)
        zw = np.cross(xw, yw / np.linalg.norm(yw))
        tip = {"body": m.body(palm).name, "pos": _to_body(d, palm, p_world=tip_w),
               "x_axis": _to_body(d, palm, v_world=xw), "z_axis": _to_body(d, palm, v_world=zw)}
        finger_tool, tip_how = None, "centre of the distal slab of the " + (
            "closed fingers" if m.jnt_type[gj[0]] == _SLIDE else "fixed jaw")
        names = [m.joint(j).name for j in gj]
        gripper = {"joints": names, "actuators": ["gripper_" + n for n in names],
                   "open": [round(float(v), 5) for v in g_open], "closed": [round(float(v), 5) for v in g_closed]}
    else:
        flange_w = _slab_tip(group, d.xanchor[ja], xw, on_axis=roll)
        ref = np.array([0, 0, 1.0]) if abs(xw[2]) < 0.9 else np.array([1.0, 0, 0])
        zw = ref - xw * float(ref @ xw)
        finger_tool = {"parent": m.body(jb).name, "pos": _to_body(d, jb, p_world=flange_w),
                       "x_axis": _to_body(d, jb, v_world=xw), "z_axis": _to_body(d, jb, v_world=zw),
                       "size_mm": list(PADDLE_MM), "mass_kg": PADDLE_KG}
        tip = {"body": "finger_tool", "pos": np.array([PADDLE_MM[0] / 1000.0, 0, 0]), "x_axis": np.array([1.0, 0, 0]),
               "z_axis": np.array([0, 0, 1.0])}
        tip_how = "end of the standard paddle on the flange (the arm has no finger)"
    arm_names = [m.joint(j).name for j in arm]
    anchors = [d.xanchor[j] for j in arm] + [group.mean(axis=0)]
    lscale = float(sum(np.linalg.norm(anchors[i + 1] - anchors[i]) for i in range(len(arm))))
    rounded = lambda v: [round(float(x), 6) for x in v]  # noqa: E731
    tip = {k: (rounded(v) if k != "body" else v) for k, v in tip.items()}
    if finger_tool:
        finger_tool = {k: (rounded(v) if k in ("pos", "x_axis", "z_axis") else v) for k, v in finger_tool.items()}
    lo = np.array([m.jnt_range[j][0] for j in arm])
    hi = np.array([m.jnt_range[j][1] for j in arm])
    gains0 = {"kp": [1.0] * len(arm), "kd": [0.1] * len(arm), "torque_limit": [1.0] * len(arm)}
    if gripper:
        gripper.update({"kp": [1.0] * len(gj), "forcerange": [1.0] * len(gj)})
    prov, _, _ = _assemble(name, path, arm_names, gripper, tip, finger_tool, (lo + hi) / 2, np.zeros((1, 3)), gains0)
    cands = _home_candidates(prov, lscale, want_z_up=bool(gj))
    # gains need a pose-independent model only; take them from the provisional spec
    kp, kd, lim, gain_how, assumed = _gains_mod.gains(m, prov, arm_names)
    gains = {"kp": kp, "kd": kd, "torque_limit": lim}
    if gripper:
        gkp, gkd, glim, _, gass = _gains_mod.gains(m, prov, gripper["joints"])
        gripper.update({"kp": gkp, "forcerange": glim, "speed": None})  # speed: None unless the maker states one
        assumed = {f: assumed[f] + gass[f] for f in assumed}
    chosen, tried = None, 0
    for score, q in cands[:30]:
        tried += 1
        sp = _orient_tool_up(name, path, arm_names, gripper, tip, finger_tool, q, gains)
        if sp is None:
            continue
        spec, back, up = sp
        M = RobotModel(spec)
        target = M.fk_tip(M.q_home)[0] + np.array([0.02, 0.02, -0.02])
        _, ok, _ = M.ik(M.q_home, target, keep_axis=("x", [0, 0, -1]))
        u, v, depth = M.project(M.q_home, M.fk_tip(M.q_home)[0])
        if ok and M.plane_clearance(M.q_home) > 0 and not M.self_collisions(M.q_home) and depth > 0:
            chosen = (spec, back, up, score)
            break
    if chosen is None:
        raise ValueError(f"derive_spec({name}): no home pose passed the checks in {tried} candidates")
    spec, back, up, score = chosen
    spec["model_fixes"] = fixes
    spec["continuous"] = [f["joint"] for f in fixes if f.get("continuous") and f["joint"] in arm_names]
    spec["assumed"] = {f: v for f, v in assumed.items() if v}
    speeds = [_gains_mod.file_speed(m, j) for j in arm]
    stated = (ratings or {}).get("speed_limit")
    spec["speed_limit"] = [float(v) for v in stated[0]] if stated else speeds
    spec["mjcf_original"] = os.path.abspath(mjcf_path)
    src = {k: "derived" for k in ("arm_joints", "gripper", "tip", "tool_surfaces", "finger_tool", "camera",
                                   "q_home", "kp", "kd", "torque_limit")}
    src["kp"], src["torque_limit"] = (_gains_mod.source_word(assumed, f) for f in ("kp", "torque_limit"))
    src["speed_limit"] = ("rating: " + stated[1] if stated else
                          "model: a velocity servo's control range in the file" if all(v is not None for v in speeds)
                          else "self_test: the file states no joint speed and no data sheet is named; the robot's "
                               "own sweep measures what it tracks")
    spec["derivation"] = {
        "arm_joints": "hinge and slide joints on the root to deepest body path, gripper joints removed",
        "gripper": grip_how or "none: the chain ends in a roll or a link with no finger",
        "tip": tip_how + "; tool x " + dir_how,
        "tool_surfaces": "bounding box faces of the distal geometry within the pads' depth of the tip",
        "finger_tool": "standard paddle on the flange" if finger_tool else "none: the arm has its own finger",
        "camera": f"camera_on_tool back {back * 1000:.0f} mm, up {up * 1000:.0f} mm (clear of the hand), "
                  f"fovy {CAMERA_FOVY_DEG:.0f}",
        "q_home": f"seeded search, best of {len(cands)} clear candidates (score {score:.3f}, rank {tried})",
        "kp": gain_how, "kd": "kd capped at the explicit law limit, damping ratio >= 0.9",
        "torque_limit": "file actuator force range where stated",
    }
    spec["overrides"] = []
    for field, (value, why) in (overrides or {}).items():
        spec[field] = value
        src[field] = "person"
        spec["overrides"].append({"field": field, "value": value, "why": why})
    spec["source"] = src
    return spec


def _orient_tool_up(name, path, arm_names, gripper, tip, finger_tool, q, gains):
    """Final spec at home q: tool z turned (paddle) or signed (gripper) to point up at home; camera and surfaces."""
    from bench.robots import home_qpos, load_robot_mjcf

    tip = dict(tip)
    ft = dict(finger_tool) if finger_tool else None
    prov, _, _ = _assemble(name, path, arm_names, gripper, tip, ft, q, np.zeros((1, 3)), gains)
    m = load_robot_mjcf(prov).compile()
    d = mujoco.MjData(m)
    d.qpos[:] = home_qpos(m, prov)
    mujoco.mj_kinematics(m, d)
    tb = m.body(tip["body"]).id
    Rb = d.xmat[tb].reshape(3, 3)
    up_b = Rb.T @ np.array([0, 0, 1.0])
    if ft:  # the paddle frame: x fixed, z = up at home square to x, in the parent body
        pb = m.body(ft["parent"]).id
        Rp = d.xmat[pb].reshape(3, 3)
        x = _build.unit(ft["x_axis"])
        up_p = Rp.T @ np.array([0, 0, 1.0])
        z = up_p - x * float(up_p @ x)
        if np.linalg.norm(z) < 0.2:
            return None  # tool points straight up or down at home: no "up" to mount the camera by
        ft["z_axis"] = [round(float(v), 6) for v in _build.unit(z)]
    else:
        z = np.asarray(tip["z_axis"], dtype=float)
        if float(z @ up_b) < 0:
            tip["z_axis"] = [round(-float(v), 6) for v in z]
    prov, _, _ = _assemble(name, path, arm_names, gripper, tip, ft, q, np.zeros((1, 3)), gains)
    m = load_robot_mjcf(prov).compile()
    d = mujoco.MjData(m)
    d.qpos[:] = home_qpos(m, prov)
    mujoco.mj_kinematics(m, d)
    site = m.site("tip").id
    Rs, ps = d.site_xmat[site].reshape(3, 3), d.site_xpos[site]
    bodies = _subtree(m, int(m.jnt_bodyid[m.joint(arm_names[-1]).id]))
    P = (_bodies_points(m, d, bodies) - ps) @ Rs
    return _assemble(name, path, arm_names, gripper, tip, ft, q, P, gains)
