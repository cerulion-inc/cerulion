# SPDX-License-Identifier: AGPL-3.0-only
"""The robot://body document and the reach summary, split out of rcpx/model.py.

Everything here is computed from a RobotModel; nothing is typed per robot. Units in the document: mm, degrees,
newtons, newton metres.

No kind is hard-coded: each joint's kind (revolute or prismatic) and unit come from the loaded model;
the camera's optical axis and image up in the tool frame come from the spec's camera and tip; a gripper is described
as a gripper (its joints and the jaw opening in mm, measured on the model at its open and closed commands); the base
kind is the robot model's own (derived: whether its root body carries a joint) until the installation is given
(with_installation); and "sources" says for every hand-written spec field whether a person typed it, it was derived from
the model file, or it is measured at run time.

What can touch: the tool block carries the shape of everything that moves with the tip (a box
per body, the outline along the finger, what stands out past each surface and at what tilt it touches first, in
words too) and the gripper block its closed and open shapes, all from rcpx/model_tool.py; each collision capsule
gives its two ends at home and the joint that carries it; self_collision says how the arm is kept off itself.
base.standing_surface is the floor under the robot: unknown to the robot model, filled from the installation
(with_installation below).
"""

import math

import mujoco
import numpy as np

from rcpx import model_tool

MM = 1000.0
DIRECTIONS = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}
SPEC_FIELDS = ("arm_joints", "gripper", "tip", "tool_surfaces", "finger_tool", "camera", "q_home", "kp", "kd",
               "torque_limit", "speed_limit", "collision_excludes")
NO_RATING = "none stated: no maker's document is named for it; the robot's own sweep measures what the joint tracks"
NEAR_PAIR_MM = 10.0  # link pairs nearer than this at home are listed in self_collision
_KINDS = {int(mujoco.mjtJoint.mjJNT_HINGE): ("revolute", "deg"), int(mujoco.mjtJoint.mjJNT_SLIDE): ("prismatic", "mm")}


def _mm(v):
    return [round(float(x) * MM, 1) for x in v]


def _r(v, nd=3):
    return [round(float(x), nd) for x in v]


def _deg(x):
    return round(math.degrees(float(x)), 2)


def _n(model, q, direction):
    """tip_force_capability rounded for the document; None when the robot's ratings do not bound that push."""
    force = model.tip_force_capability(q, direction)
    return None if force is None else round(force, 1)


def _valid(model, q):
    return model.plane_clearance(q) > 0.0 and not model.self_collisions(q)


def reach_summary(model, n_samples=4000, seed=0):
    rng = np.random.default_rng(seed)
    pts = []
    for _ in range(n_samples):
        q = rng.uniform(model.q_lo, model.q_hi)
        if _valid(model, q):
            pts.append(model.fk_tip(q)[0])
    P = np.array(pts) if pts else np.zeros((1, 3))
    rad = np.hypot(P[:, 0], P[:, 1])
    dist = np.linalg.norm(P, axis=1)
    z_top = float(P[:, 2].max())
    heights = [float(h) for h in np.linspace(0.05 * z_top, 0.8 * z_top, 5)]
    rows = []
    q_seed = model.q_home.copy()
    for h in heights:
        band = np.abs(P[:, 2] - h) < 0.03 * max(z_top, 0.1)
        lo = float(rad[band].max()) * 0.8 if band.any() else 0.0
        hi = float(rad.max()) + 0.05
        best, q_best = None, None
        q_lo_ok = None
        q_try, ok, _ = model.ik(q_seed, [lo, 0.0, h], iters=150, restarts=4)
        if ok and model.plane_clearance(q_try) > -1e-3:
            best, q_lo_ok = lo, q_try
        if best is not None:
            a, b = lo, hi
            q_b = q_lo_ok
            for _ in range(9):
                mid = 0.5 * (a + b)
                q_try, ok, _ = model.ik(q_b, [mid, 0.0, h], iters=150, restarts=3)
                if ok and model.plane_clearance(q_try) > -1e-3:
                    a, q_b = mid, q_try
                else:
                    b = mid
            best, q_best = a, q_b
        rows.append({"height_mm": round(h * MM, 1), "max_forward_mm": None if best is None else round(best * MM, 1),
                     "tip_force_down_n": None if q_best is None else _n(model, q_best, (0, 0, -1))})
    near = P[P[:, 2] < 0.1 * max(z_top, 0.1)] if len(P) else P
    near_rad = np.hypot(near[:, 0], near[:, 1]) if len(near) else rad
    return {
        "samples": int(n_samples),
        "valid_samples": int(len(pts)),
        "max_reach_mm": round(float(dist.max()) * MM, 1),
        "max_height_mm": round(z_top * MM, 1),
        "forward_reach_by_height": rows,
        "workspace_box_mm": {"min": _mm(P.min(axis=0)), "max": _mm(P.max(axis=0))},
        "annulus_near_plane_mm": {
            "inner": round(float(np.percentile(near_rad, 2)) * MM, 1) if len(near_rad) else None,
            "outer": round(float(near_rad.max()) * MM, 1) if len(near_rad) else None,
            "band_height_mm": round(0.1 * max(z_top, 0.1) * MM, 1),
        },
        "method": "random joint samples inside the limits with the arm clear of the plane and itself; forward reach "
                  "per height refined by position-only ik bisection along +x",
    }


def _chain_up(m, b):
    out = []
    while b > 0:
        out.append(int(b))
        b = int(m.body_parentid[b])
    return out


def _rigid_between(m, a, b):
    """True when no joint lies on the body path between bodies a and b (they move as one)."""
    up_a, up_b = _chain_up(m, a), _chain_up(m, b)
    common = set(up_a) & set(up_b)
    path = [x for x in up_a if x not in common] + [x for x in up_b if x not in common]
    return all(int(m.body_jntnum[x]) == 0 for x in path)


def _toward(v, names=("x", "y", "z")):
    """The tool axis a direction leans toward, away from tool x: '+z', '-y' and so on."""
    rest = [abs(float(v[1])), abs(float(v[2]))]
    k = 1 + int(np.argmax(rest))
    return ("+" if float(v[k]) >= 0 else "-") + names[k]


def _camera_from_spec(model):
    """The camera's optical axis, image up and right and position in the tool frame T, from the spec's camera
    (body, pos, xyaxes) and tip (body, pos, x_axis, z_axis); the two bodies' relative pose from the model at home.
    rigid_with_tool says whether that relative pose is the same at every joint angle."""
    from bench.robots import _build

    spec, m = model.spec, model.mj
    cam, tip = spec["camera"], spec["tip"]
    cb, tb = m.body(cam["body"]).id, m.body(tip["body"]).id
    rigid = _rigid_between(m, cb, tb)
    model._set(model.q_home)
    d = model.d
    Rtb, ptb = d.xmat[tb].reshape(3, 3), d.xpos[tb]
    Rcb, pcb = d.xmat[cb].reshape(3, 3), d.xpos[cb]
    R_rel = np.eye(3) if cb == tb else Rtb.T @ Rcb
    p_rel = np.zeros(3) if cb == tb else Rtb.T @ (pcb - ptb)
    Rt = _build.mat_from_xz(tip["x_axis"], tip["z_axis"])       # tool axes in the tip body
    Rc = _build.mat_from_xyaxes(cam["xyaxes"])                   # camera axes in the camera body (looks along -z)
    to_tool = Rt.T @ R_rel
    axis = to_tool @ (-Rc[:, 2])
    up = to_tool @ Rc[:, 1]
    right = to_tool @ Rc[:, 0]
    pos = Rt.T @ (R_rel @ np.asarray(cam["pos"], float) + p_rel - np.asarray(tip["pos"], float))
    off = math.degrees(math.acos(max(-1.0, min(1.0, float(axis[0])))))
    lean = "" if off < 1.0 else f", leaning toward tool {_toward(axis)}"
    mount = ("rigid with the tool" if rigid else
             "NOT rigid with the tool: joints lie between them, so these axes hold at home only")
    text = (f"camera on body {cam['body']} ({mount}): image u right, v down; the optical axis is {_r(axis)} in the "
            f"tool frame, {off:.1f} deg off tool x{lean}; image up is {_r(up)} in the tool frame")
    return {"optical_axis_in_tool": _r(axis), "up_in_tool": _r(up), "right_in_tool": _r(right),
            "position_in_tool_mm_from_spec": _mm(pos), "optical_axis_off_tool_x_deg": round(off, 2),
            "rigid_with_tool": bool(rigid),
            "axes_source": "the spec's camera xyaxes and tip axes" + ("" if rigid else ", at the home pose"),
            "description": text}


def _camera_block(model):
    ptip, Rtip = model.fk_tip(model.q_home)
    pc, Rc = model.fk(model.q_home)["camera"]
    pos_tool = Rtip.T @ (pc - ptip)
    look_tool = Rtip.T @ (-Rc[:, 2])
    up_tool = Rtip.T @ Rc[:, 1]
    u, v, depth = model.project(model.q_home, ptip)
    fovx = 2 * math.degrees(math.atan((model.cam_w / 2.0) / model.f_px))
    out = {
        "name": "wrist",
        "mounted_on": model.spec["camera"]["body"],
        "position_in_tool_mm": _mm(pos_tool),
        "looks_along_in_tool": _r(look_tool),
        "image_up_in_tool": _r(up_tool),
        "angle_off_tool_x_deg": _deg(math.acos(max(-1.0, min(1.0, float(look_tool[0]))))),
        "fov_vertical_deg": _deg(model.fovy),
        "fov_horizontal_deg": round(fovx, 2),
        "width_px": model.cam_w,
        "height_px": model.cam_h,
        "focal_px": round(model.f_px, 2),
        "pixel_convention": "u right, v down, origin top left; principal point at the image centre",
        "tip_pixel": [round(u, 1), round(v, 1)],
        "tip_depth_mm": round(depth * MM, 1),
        "mm_per_px_at_tip": round(depth * MM / model.f_px, 3),
    }
    out.update(_camera_from_spec(model))
    return out


def _tool_kind(spec):
    if spec.get("finger_tool"):
        return "fingertip paddle on the flange"
    if spec.get("gripper"):
        return "gripper: the tool tip is a point on the hand's jaws (the jaws start closed)"
    return "the last link's own end (no gripper, no paddle)"


def _tool_block(model):
    spec = model.spec
    q = model.q_home
    ptip, Rtip = model.fk_tip(q)
    model._set(q)
    d, m = model.d, model.mj
    surfaces = []
    for s in spec["tool_surfaces"]:
        sid = m.site("surface_" + s["name"]).id
        ps, Rs = d.site_xpos[sid], d.site_xmat[sid].reshape(3, 3)
        surfaces.append({
            "name": s["name"],
            "centre_in_tool_mm": _mm(Rtip.T @ (ps - ptip)),
            "normal_in_tool": _r(Rtip.T @ Rs[:, 2]),
            "normal_at_home_base": _r(Rs[:, 2]),
            "size_mm": [float(x) for x in s["size_mm"]],
        })
    out = {
        "kind": _tool_kind(spec),
        "tip_at_home_mm": _mm(ptip),
        "tool_x_at_home_base": _r(Rtip[:, 0]),
        "tool_z_at_home_base": _r(Rtip[:, 2]),
        "frame": "T: origin at the tip, x along the finger's pointing direction, z the hand's up, y = z cross x",
        "surfaces": surfaces,
    }
    if spec.get("finger_tool"):
        out["paddle_size_mm"] = [float(x) for x in spec["finger_tool"]["size_mm"]]
    out.update(model_tool.tool_shape(model, surfaces))
    return out


def _joint_kind(model, i):
    return _KINDS.get(int(model.mj.jnt_type[model._jids[i]]), ("other", None))


def _in_unit(x, unit):
    if unit == "deg":
        return _deg(x)
    if unit == "mm":
        return round(float(x) * MM, 2)
    return round(float(x), 4)


def _joint_entry(model, i, anchor, axis):
    spec, q = model.spec, model.q_home
    kind, unit = _joint_kind(model, i)
    lo, hi = model.q_lo[i], model.q_hi[i]
    rev = kind == "revolute"
    entry = {
        "index": i,
        "name": model.joint_names[i],
        "type": kind,
        "unit": unit,
        "limit": [_in_unit(lo, unit), _in_unit(hi, unit)],
        "home": _in_unit(q[i], unit),
        "limit_deg": [_deg(lo), _deg(hi)] if rev else None,
        "home_deg": _deg(q[i]) if rev else None,
        "axis_at_home_base": _r(axis),
        "anchor_at_home_mm": _mm(anchor),
        "sign": ("a positive angle turns the distal links counter-clockwise about the axis (right-hand rule)" if rev
                 else "a positive value slides the distal links along the axis" if kind == "prismatic"
                 else "see the model file"),
        "speed_limit": _speed_entry(model, i, unit),
        "continuous": model.joint_names[i] in (spec.get("continuous") or ()),
        "torque_limit_nm": float(model.torque_limit[i]),
        "effort_limit": {"value": float(model.torque_limit[i]), "unit": "N m" if rev else "N"},
        "servo_kp_nm_per_rad": float(spec["kp"][i]),
        "servo_kd_nms_per_rad": float(spec["kd"][i]),
        "servo_units": "kp N m per rad, kd N m s per rad" if rev else "kp N per m, kd N s per m",
        "kind_source": "derived: the joint type in the model file",
    }
    if kind == "prismatic":
        entry["limit_mm"], entry["home_mm"] = entry["limit"], entry["home"]
    return entry


def _speed_entry(model, i, unit, value=None, source=None):
    """joints[].speed_limit: {value (deg/s or mm/s; None when no speed is stated), unit, source}. The source is the
    spec's own word for it (a rating names its document) until the robot's sweep gives a lower or a first value."""
    rated = model.speed_limit[i] if value is None else value
    told = (model.spec.get("source") or {}).get("speed_limit")
    if source is None:
        source = (told or "rating") if model.speed_limit[i] is not None else NO_RATING
    per = "deg/s" if unit == "deg" else "mm/s" if unit == "mm" else "per s"
    return {"value": None if rated is None else _in_unit(rated, unit), "unit": per, "source": source}


def with_self(doc, know, model=None):
    """The body document with the robot's own measured numbers: a block self holding, each with its
    source and the stamp of when it was read, the hidden level and noise of the force estimate at rest, the
    detection delay, the lag and stopping tables, the stop slope, the quantum, e_tau, F_tool and F_hand; and
    joints[].speed_limit lowered to what the sweep showed the joint tracks (source self_test). know is a
    rcpx.bounds_self.SelfKnow (anything with get(name, *args) -> (value, source, at) and rows(table)); model, when
    given, supplies F_tool and F_hand where know has none. Returns a new document."""
    out = dict(doc)

    def item(name, *args, scale=1.0, unit=None):
        value, source, at = know.get(name, *args)
        if value is None:
            return None
        if isinstance(value, (list, tuple)):
            value = [None if v is None else round(float(v) * scale, 6) for v in value]
        elif isinstance(value, (int, float)):
            value = round(float(value) * scale, 6)
        return {"value": value, "unit": unit, "source": source, "at": at}

    def table(name, scale=MM, unit="mm"):
        return [{"speed_mm_s": round(float(r["x"][0]) * MM, 3), unit: _scaled(r["y"], scale), "source": r.get("source"),
                 "at": r.get("at")} for r in know.rows(name)]

    block = {"ready": bool(getattr(know, "ready", False)), "stored": getattr(know, "stored", None),
             "f_hidden_n": item("f_hidden", 0.0, 0.0, unit="N"), "sigma_f_n": item("sigma_f", 0.0, 0.0, unit="N"),
             "t_det_s": item("t_det", unit="s"), "t_settle_s": item("t_settle", unit="s"),
             "lag": table("lag"), "d_stop": table("d_stop"), "slope": item("slope"),
             "quantum": item("quantum", unit="rad per joint"), "e_tau": item("e_tau"),
             "f_tool_n": item("f_tool", unit="N"), "f_hand_n": item("f_hand", unit="N"),
             "took_s": item("took_s", unit="s")}
    if model is not None:
        for key, got, how in (("f_tool_n", model.f_tool(), None), ("f_hand_n", model.f_hand(), "the model's masses")):
            if block[key] is None and got[0] is not None:   # a tool rating is there only when one is stated
                block[key] = {"value": round(float(got[0]), 3), "unit": "N", "source": "model", "at": None,
                              "evidence": how or got[1]}
    out["self"] = block
    qd, _, at = know.get("qd_max")
    sources = know.get("qd_sources") if qd is not None else None
    if qd is not None and model is not None:
        joints = [dict(j) for j in out.get("joints") or []]
        for i, j in enumerate(joints[:len(qd)]):
            if qd[i] is not None and sources and sources[i] == "self_test":
                j["speed_limit"] = _speed_entry(model, i, j.get("unit"), value=qd[i],
                                                source=f"self_test: the largest speed the joint tracked in its own "
                                                       f"sweep ({at})")
        out["joints"] = joints
    return out


def _scaled(y, scale):
    return [round(float(v) * scale, 4) for v in y] if isinstance(y, (list, tuple)) else round(float(y) * scale, 4)


def _geoms_under(m, body):
    """Collision geoms of a body and its descendants (every geom when none collides)."""
    under = [g for g in range(m.ngeom) if body in _chain_up(m, int(m.geom_bodyid[g]))]
    hit = [g for g in under if m.geom_contype[g] or m.geom_conaffinity[g]]
    return hit or under


def _gaps(model, gadr, values, sides):
    """Per geom of the moving jaw: the smallest distance (m) to the other jaw, with the jaws at `values`."""
    m, d = model.mj, model.d
    model._set(model.q_home)
    d.qpos[gadr] = values
    mujoco.mj_kinematics(m, d)
    fromto = np.zeros(6)
    return [min(float(mujoco.mj_geomDistance(m, d, int(ga), int(gb), 1.0, fromto)) for gb in sides[1])
            for ga in sides[0]]


def _gripper_block(model):
    """The gripper as a gripper: joints and kinds, open and closed commands, and the jaw opening in mm measured
    on the model at those two commands, at the part of the moving jaw whose gap to the other jaw changes most
    between them (the fingertips; parts that overlap at a hinge never change)."""
    g = model.spec.get("gripper")
    if not g:
        return None
    m = model.mj
    jids = [m.joint(j).id for j in g["joints"]]
    fingers = list(dict.fromkeys(int(m.jnt_bodyid[j]) for j in jids))
    if len(fingers) >= 2:
        pair = (fingers[0], fingers[1])
        how = f"between bodies {m.body(pair[0]).name} and {m.body(pair[1]).name}"
    else:
        pair = (fingers[0], int(m.body_parentid[fingers[0]]))
        how = f"between moving jaw {m.body(pair[0]).name} and the body it hangs on, {m.body(pair[1]).name}"
    sides = [_geoms_under(m, pair[0]),
             [x for x in _geoms_under(m, pair[1]) if pair[0] not in _chain_up(m, int(m.geom_bodyid[x]))]]
    opened, closed = None, None
    if sides[0] and sides[1]:
        gadr = [int(m.jnt_qposadr[j]) for j in jids]
        at_open = _gaps(model, gadr, [float(x) for x in g["open"]], sides)
        at_closed = _gaps(model, gadr, [float(x) for x in g["closed"]], sides)
        k = int(np.argmax(np.asarray(at_open) - np.asarray(at_closed)))
        opened = round(max(0.0, at_open[k]) * MM, 1)
        closed = round(max(0.0, at_closed[k]) * MM, 1)
    model._set(model.q_home)
    kinds = [_KINDS.get(int(m.jnt_type[j]), ("other", None))[0] for j in jids]
    out = {"kind": "gripper", "joints": list(g["joints"]), "joint_kinds": kinds,
           "state": "closed at the start of an episode; the gripper tool sets the opening (0 closed, 1 open)",
           "open": [float(x) for x in g["open"]], "closed": [float(x) for x in g["closed"]],
           "open_width_mm": opened, "closed_width_mm": closed,
           "width_method": "smallest distance between the jaws' collision geometry on the model, " + how +
                           ", at the jaw part whose gap changes most from closed to open",
           "source": "derived: measured on the model at the spec's open and closed commands"}
    out.update(model_tool.gripper_shapes(model))
    return out


def _base_block(model):
    """What the robot model alone knows about its base: nothing about how it is installed. A root body with a free
    joint is a loose base; a root with no joint says only that the model has no mobile base. The installation is
    filled in by with_installation."""
    m, root = model.mj, model._root
    free = any(int(m.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE)
               for j in range(m.njnt) if int(m.jnt_bodyid[j]) == root)
    return {"kind": "loose" if free else "unknown", "give": None,
            "kind_source": "derived",
            "kind_how": ("the robot model's root body carries a free joint" if free else
                         "the robot model's root body carries no joint of its own, which does not say how the "
                         "robot is fastened down") +
                        "; how it is installed is not given",
            "note": BASE_NOTE_UNKNOWN,
            "standing_surface": {"floor_z_mm": None, "mount_top_z_mm": 0.0, "mount_half_side_mm": None,
                                 "source": "unknown to the robot model: the installation is not given",
                                 "note": "the surface the robot stands on, in the base frame: z = 0 is the mounting "
                                         "plane (the top of whatever the base is fastened to); floor_z_mm is the "
                                         "height of the floor or table around that mount"}}


BASE_NOTE_UNKNOWN = "not measured yet: whether the base gives under a push, and by how much, is not known"
BASE_NOTE = {"fixed": "fastened down: a push at the hand does not move the base",
             "loose": "standing free: a push at the hand can tip or turn the base a little"}
GIVEN_BY = "as the person who set the robot up gave it"


CAMERA_UNCERTAINTY_NOTE = ("how far the mounted camera may sit and turn from the pose the body model gives, as the "
                           "person who mounted it states")


def with_installation(doc, kind=None, standing_surface=None, measured_give=None, kind_how=None,
                      camera_uncertainty=None, encoder_counts=None):
    """The body document with what only the installation knows: kind ("fixed" | "loose"), standing_surface
    (Executor.standing_surface(): floor_z_mm, mount_top_z_mm, mount_half_side_mm, note), measured_give
    (Executor.base_give), camera_uncertainty ({mm, deg}: how far the mounted camera may sit and turn from the
    model's pose; served as camera.pose_uncertainty {mm, deg, source "person", note}) and encoder_counts (counts
    per turn, one number or one per joint, the robot spec's optional key of that name: each revolute joint gains
    resolution_deg = 360 / counts). Returns a new document; keys not given stay as the model left them. The words
    name no tool and no harness (the texts an operator reads)."""
    out = dict(doc)
    base = dict(out.get("base") or {})
    sources = dict(out.get("sources") or {})
    if kind is not None:
        base.update(kind=kind, kind_source="person", kind_how=kind_how or f"how the robot is installed, {GIVEN_BY}")
        if base.get("note") in (None, BASE_NOTE_UNKNOWN) and kind in BASE_NOTE:
            base["note"] = BASE_NOTE[kind]
        sources["base.kind"] = "person"
    if standing_surface is not None:
        base["standing_surface"] = dict(base.get("standing_surface") or {}, **standing_surface,
                                        source=f"the installation, {GIVEN_BY}")
        sources["base.standing_surface"] = "person"
    if measured_give is not None:
        base["measured_give"] = measured_give
    if encoder_counts:
        joints = [dict(j) if isinstance(j, dict) else j for j in out.get("joints") or []]
        per = list(encoder_counts) if isinstance(encoder_counts, (list, tuple)) else [encoder_counts] * len(joints)
        for j, counts in zip(joints, per):
            if isinstance(j, dict) and j.get("type", "revolute") == "revolute" and counts:
                j["resolution_deg"] = round(360.0 / float(counts), 6)
        out["joints"] = joints
        sources["joints.resolution_deg"] = "person"
    if camera_uncertainty is not None:
        cam = dict(out.get("camera") or {})
        cam["pose_uncertainty"] = {"mm": float(camera_uncertainty["mm"]), "deg": float(camera_uncertainty["deg"]),
                                   "source": "person", "note": CAMERA_UNCERTAINTY_NOTE}
        out["camera"] = cam
        sources["camera.pose_uncertainty"] = "person"
    out["base"], out["sources"] = base, sources
    return out


def _capsule_rows(model, q):
    """Each collision capsule with its two ends at home (base frame) and the joint that carries it."""
    m, rows = model.mj, []
    for name, p0, p1, r in model.collision_capsules(q):
        k = model._arm_key(m.body(name).id)
        rows.append({"name": name, "radius_mm": round(r * MM, 1),
                     "length_mm": round(float(np.linalg.norm(p1 - p0)) * MM, 1),
                     "ends_at_home_mm": f"{_mm(p0)} to {_mm(p1)}",
                     "carried_by": model.joint_names[k] if k >= 0 else "the base (does not move)"})
    return rows


def _self_collision_block(model):
    near = [{"links": [a, b], "clearance_at_home_mm": round(float(h) * MM, 1)}
            for (a, b), h in zip(model._sp_names, model._sp_home) if h * MM < NEAR_PAIR_MM]
    return {"method": "a move is refused (check path_self) when two links that are not neighbours would come "
                      "nearer than the margin, on the model's own collision geometry; neighbours are kept apart "
                      f"by their joint limits. near_at_home: pairs under {NEAR_PAIR_MM:.0f} mm apart at home, "
                      "refused only once nearer than they are there",
            "near_at_home": near,
            "excluded": [{"links": list(e["bodies"]), "why": e.get("why", "")}
                         for e in model.spec.get("collision_excludes") or ()]}


def _sources(spec, has_gripper):
    told = spec.get("source") or {}
    out = {f: told.get(f, "person") for f in SPEC_FIELDS if f in spec}
    out.update({"joints.type_unit_limits": "derived", "links": "derived", "collision_capsules": "derived",
                "reach": "derived", "tip_force_at_home_n": "derived", "camera.axes": told.get("camera", "person"),
                "base.kind": "derived", "base.give": "measured", "base.standing_surface": "unknown",
                "tool.shape": "derived", "self_collision": "derived"})
    if has_gripper:
        out["gripper.widths"] = "derived"
    return out


def describe(model):
    spec = model.spec
    q = model.q_home
    frames = model.joint_frames(q)
    ptip = model.fk_tip(q)[0]
    joints = [_joint_entry(model, i, a, ax) for i, (a, ax) in enumerate(frames)]
    links = []
    pts = [f[0] for f in frames] + [ptip]
    for i in range(model.n):
        to = model.joint_names[i + 1] if i + 1 < model.n else "tip"
        links.append({"from": model.joint_names[i], "to": to,
                      "length_mm": round(float(np.linalg.norm(pts[i + 1] - pts[i])) * MM, 1)})
    caps = _capsule_rows(model, q)
    force = {k: _n(model, q, v) for k, v in DIRECTIONS.items()}
    camera = _camera_block(model)
    gripper = _gripper_block(model)
    units = sorted({j["unit"] for j in joints if j["unit"]})
    out = {
        "uri": "robot://body",
        "source": "derived from the loaded robot model",
        "name": model.name,
        "units": {"length": "mm", "angle": "deg", "force": "N", "torque": "N m", "time": "s"},
        "frames": {
            "base": "B: origin on the mounting plane under the first joint axis, x forward, y left, z up",
            "tool": "T: origin at the tool tip, x along the finger, z the hand's up",
            "camera": camera["description"],
        },
        "dof": model.n,
        "joint_order": list(model.joint_names),
        "joint_units": units,
        "joints": joints,
        "links": links,
        "collision_capsules": caps,
        "collision_capsules_note": "a coarse capsule per link, for the floor check and the picture: its two ends at "
                                   "home in the base frame and the joint that carries it; the tool's true shape "
                                   "is under tool",
        "self_collision": _self_collision_block(model),
        "home": {"q_deg": [_deg(x) for x in q], "q": [j["home"] for j in joints],
                 "q_units": [j["unit"] for j in joints], "tip_mm": _mm(ptip)},
        "tool": _tool_block(model),
        "camera": camera,
        "reach": model.reach(),
        "tip_force_at_home_n": force,
        "tip_force_note": "largest steady push the tip can make on the world along each base direction at home: the "
                          "smaller of what the joint torque limits allow after holding gravity and what the "
                          "position servo holds without the arm folding",
        "gripper": gripper,
        "end_effector": {"kind": "gripper" if gripper else "paddle" if spec.get("finger_tool") else "fixed end",
                         "gripper": "present" if gripper else "none", "tool": _tool_kind(spec)},
        "base": _base_block(model),
        "sources": _sources(spec, gripper is not None),
        "sources_note": "person: typed by a person in the robot spec; derived: computed from the model file; "
                        "measured: measured on the running robot",
    }
    if spec.get("derivation"):
        out["derivation"] = dict(spec["derivation"])
    return out
