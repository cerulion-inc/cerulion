# SPDX-License-Identifier: AGPL-3.0-only
"""The rcpx robot spec of the benchmark's arm, built per task from the env host's robot_xml() (build plan S6).

The env host serves the arm's own model file: the merged robot, gripper and mount of the task's robot class, with the
base placed where the task puts it (the mounted and the on-the-ground variants differ in base height). This module
turns that file into what rcpx loads (bench.robots.load_robot_mjcf, rcpx.model.RobotModel):

- arm_xml(text): the arm alone. The mount bodies are dropped (the mount is part of the installation: the work
  surface height goes to the executor cfg instead), and the first arm link becomes the root body at the base's place,
  because rcpx puts frame B on the lowest face of the root body's own geometry and the wrapper body has none. Stdlib
  only, so the env host's Python can call it too.
- build_spec(text, ...): bench.derive_spec.derive_spec on that file, with overrides a person would type from the
  robot's documentation, each with its reason: the home pose (the arm's start pose), the wrist camera (the arm's own
  hand camera, its pose from the same file), and the servo gains (the arm's joint position controller: an inertia
  weighted PD, so the joint stiffness rcpx plans with is the controller's kp times the joint inertia at home). The
  parallel gripper is derived as a gripper. Fixed scene cameras ride in spec["scene_cameras"] with their
  installation pose in frame B (rcpx.r2.scene_camera reads them through the backend).
- frame_b(spec, base_pos, base_quat): frame B in the env's world (a 4x4), so the backend can state a scene camera's
  pose in frame B and a test can read world truth in frame B.

- executor_cfg(facts, spec_facts): the installation rcpx is told (a fixed base, the work surface height, the
  gripper's own tracking).

Never here: object poses, object names, contact names or anything from the scene but the robot's own file, the
cameras' installation poses and the work surface height.
"""

import hashlib
import math
import os
import xml.etree.ElementTree as ET

try:                      # the env host's Python imports arm_xml only; the rest needs numpy
    import numpy as np
except ImportError:       # pragma: no cover
    np = None

BASE_BODY = "robot0_base"         # the wrapper body the robot class places in the world
FIRST_LINK = "robot0_link0"       # the arm's first link: the root rcpx loads
MOUNT_PREFIX = "mount0_"          # the mount's bodies (installation, not arm)
WRIST_CAMERA = "robot0_eye_in_hand"
EEF_SITE = "gripper0_grip_site"   # the env's end-effector site (the pose the env reports)
FT_SITE = "gripper0_ft_frame"     # the wrist force/torque sensor site
SPEC_NAME = "libero_panda"
WRIST_PX = (256, 256)             # (width, height): the size the plain interface sees, unless asked otherwise

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
CACHE_DIR = os.path.join(ROOT, "runs", "derived_mjcf")


def _floats(text, n=None, default=None):
    if text is None:
        return list(default) if default is not None else None
    out = [float(x) for x in str(text).split()]
    if n is not None and len(out) != n:
        raise ValueError(f"expected {n} numbers, got {text!r}")
    return out


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def _quat_rot(q, v):
    """Rotate vector v by unit quaternion q (w, x, y, z)."""
    w, x, y, z = q
    p = _quat_mul(_quat_mul(q, [0.0] + list(v)), [w, -x, -y, -z])
    return p[1:]


def quat_to_mat(q):
    """3x3 rotation (row lists) of a quaternion (w, x, y, z), normalised first."""
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    w, x, y, z = (c / n for c in q)
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def _find_body(root, name):
    for b in root.iter("body"):
        if b.get("name") == name:
            return b
    return None


def _parent_map(root):
    return {c: p for p in root.iter() for c in p}


WORLD_COMPILER = {"angle": "radian", "inertiagrouprange": "0 0"}
WORLD_OPTION = {"impratio": "20", "cone": "elliptic", "density": "1.2", "viscosity": "0.00002"}


def arm_xml(xml_text, base_dir=None):
    """(xml text of the arm alone, facts). facts: base_pos and base_quat (the base's place in the env's world, as
    the robot class put it), mount_bodies (dropped), dropped_cameras (cameras on the wrapper body), wrist_camera
    ({"body", "pos", "quat", "fovy"} from the file), finger_ctrlrange ({finger joint: [lo, hi]} of the gripper's
    position actuators), added (what was filled in). base_dir: the folder the file came from, so relative mesh and
    texture folders still resolve when the result is written elsewhere. A file with no compiler or option element
    (the robot class's own merged file) gets the world file's: every scene of this benchmark is built on robosuite's
    world file (robosuite/models/assets/base.xml: radians, inertia from group 0, air of density 1.2 and viscosity
    2e-5, which drags every moving body). Raises ValueError when the file has no arm wrapper or link."""
    root = ET.fromstring(xml_text)
    added = []
    comp = root.find("compiler")
    if comp is None:
        comp = ET.Element("compiler", dict(WORLD_COMPILER))
        root.insert(0, comp)
        added.append("compiler of the world file")
    if root.find("option") is None:
        root.insert(1, ET.Element("option", dict(WORLD_OPTION)))
        added.append("option of the world file (air)")
    if base_dir is not None:
        for attr in ("meshdir", "texturedir", "assetdir"):
            v = comp.get(attr)
            if v is not None and not os.path.isabs(v):
                comp.set(attr, os.path.join(os.path.abspath(base_dir), v))
        if comp.get("meshdir") is None and comp.get("assetdir") is None:
            comp.set("meshdir", os.path.abspath(base_dir))
    wb = root.find("worldbody")
    if wb is None:
        raise ValueError("robot xml has no worldbody")
    parents = _parent_map(root)
    for b in list(root.iter("body")):        # the mount (and anything under it) is installation, not arm
        if str(b.get("name", "")).startswith(MOUNT_PREFIX) and b in parents:
            parents[b].remove(b)
    parents = _parent_map(root)
    base = _find_body(wb, BASE_BODY)
    link0 = _find_body(wb, FIRST_LINK)
    if link0 is None:
        raise ValueError(f"robot xml has no body {FIRST_LINK!r}")
    if base is None:    # the file's root is the first link already
        if parents[link0] is not wb:
            raise ValueError(f"robot xml body {FIRST_LINK!r} is neither under {BASE_BODY!r} nor a root body")
        base = ET.Element("body", {"name": BASE_BODY})
        wb.remove(link0)
        base.append(link0)
        wb.append(base)
        parents = _parent_map(root)
    if parents[link0] is not base:
        raise ValueError(f"robot xml body {BASE_BODY!r} has no child {FIRST_LINK!r}")
    holder = parents[base]
    bpos = _floats(base.get("pos"), 3, (0.0, 0.0, 0.0))
    bquat = _floats(base.get("quat"), 4, (1.0, 0.0, 0.0, 0.0))
    lpos = _floats(link0.get("pos"), 3, (0.0, 0.0, 0.0))
    lquat = _floats(link0.get("quat"), 4, (1.0, 0.0, 0.0, 0.0))
    # the link's place in the world: base pose composed with the link's pose in the base
    off = _quat_rot(bquat, lpos)
    link0.set("pos", " ".join(repr(round(bpos[i] + off[i], 9)) for i in range(3)))
    link0.set("quat", " ".join(repr(round(c, 9)) for c in _quat_mul(bquat, lquat)))
    mounts = sorted({str(b.get("name")) for b in ET.fromstring(xml_text).iter("body")
                     if str(b.get("name", "")).startswith(MOUNT_PREFIX)})
    dropped_cams = [c.get("name") for c in base if c.tag == "camera"]
    idx = list(holder).index(base)
    holder.remove(base)
    holder.insert(idx, link0)
    # the dropped mount's assets stay (unused meshes are harmless); its contact pairs and excludes would name
    # bodies that are gone, so they go
    gone = set(mounts)
    finger_range = {}
    act = root.find("actuator")
    for a in (list(act) if act is not None else []):   # <position> in a robot file, <general> once compiled
        if a.get("joint") and a.get("ctrlrange") and (a.tag == "position" or a.get("biastype") == "affine"):
            finger_range[a.get("joint")] = _floats(a.get("ctrlrange"), 2)
    for sec in ("contact", "equality"):
        el = root.find(sec)
        if el is None:
            continue
        for c in list(el):
            names = {c.get("body1"), c.get("body2"), c.get("bodyname1"), c.get("bodyname2")}
            if any(n and (n == BASE_BODY or n.startswith(MOUNT_PREFIX)) for n in names) or names & gone:
                el.remove(c)
    cam = None
    for b in root.iter("body"):
        for c in b:
            if c.tag == "camera" and c.get("name") == WRIST_CAMERA:
                cam = {"body": b.get("name"), "pos": _floats(c.get("pos"), 3, (0.0, 0.0, 0.0)),
                       "quat": _floats(c.get("quat"), 4, (1.0, 0.0, 0.0, 0.0)), "fovy": float(c.get("fovy", 45.0))}
    facts = {"base_pos": bpos, "base_quat": bquat, "mount_bodies": mounts, "dropped_cameras": dropped_cams,
             "wrist_camera": cam, "first_link": FIRST_LINK, "added": added, "finger_ctrlrange": finger_range}
    return ET.tostring(root, encoding="unicode"), facts


def _stable_assets(text, cache_dir):
    """The arm file with its mesh and texture folder moved to cache_dir/assets: the env host writes a fresh copy of
    the same meshes beside every episode's file, so without the move no two episodes share a cached derivation. The
    host names each copy by its source path (the same name is the same file), so a file already there is kept."""
    import shutil

    root = ET.fromstring(text)
    comp = root.find("compiler")
    if comp is None:
        return text
    dst = os.path.join(cache_dir, "assets")
    os.makedirs(dst, exist_ok=True)
    for attr in ("meshdir", "texturedir"):
        src = comp.get(attr)
        if not src or not os.path.isabs(src) or not os.path.isdir(src):
            continue
        for fn in sorted(os.listdir(src)):
            out = os.path.join(dst, fn)
            if not os.path.exists(out):
                tmp = f"{out}.{os.getpid()}.tmp"
                shutil.copyfile(os.path.join(src, fn), tmp)
                os.replace(tmp, out)
        comp.set(attr, dst)
    return ET.tostring(root, encoding="unicode")


def _write_cached(text, name, cache_dir):
    os.makedirs(cache_dir, exist_ok=True)
    path = os.path.join(cache_dir, f"{name}_{hashlib.sha256(text.encode()).hexdigest()[:12]}.xml")
    if not os.path.exists(path):
        tmp = f"{path}.{os.getpid()}.tmp"
        with open(tmp, "w") as f:
            f.write(text)
        os.replace(tmp, path)   # in one step: a process started beside this one never reads half a file
    return path


def wrist_camera_spec(cam, width, height):
    """The spec camera dict (body, pos, xyaxes, fovy, width, height) of the arm's own hand camera, from the file."""
    R = quat_to_mat(cam["quat"])
    x = [R[0][0], R[1][0], R[2][0]]
    y = [R[0][1], R[1][1], R[2][1]]
    return {"body": cam["body"], "pos": [float(v) for v in cam["pos"]], "xyaxes": [float(v) for v in x + y],
            "fovy": float(cam["fovy"]), "width": int(width), "height": int(height)}


# the robot classes' own home pose (their init_qpos): rcpx's home and frame B come from it (frame B's x points at
# the tip at home, so a home with joints 1, 3 and 5 at zero puts frame B on the base's own axes)
HOME_QPOS = {"MountedPanda": [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, math.pi / 4],
             "OnTheGroundPanda": [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, math.pi / 4]}


def home_pose(info, state):
    """(q_home, how): the host's init_qpos when it gives one, else the robot class's own (HOME_QPOS), else the
    start pose (frame B then follows the start pose's tip, a few degrees off the base's axes)."""
    q = info.get("init_qpos")
    if isinstance(q, (list, tuple)) and len(q) == 7:
        return [float(v) for v in q], "the robot's init_qpos, from the env host"
    cls = info.get("robot_class")
    if cls in HOME_QPOS:
        return list(HOME_QPOS[cls]), f"the {cls} class's init_qpos"
    return [float(v) for v in state["q"]], "the start pose (the robot class has no known home)"


def camera_from_host(c):
    """The host's cam answer (OpenCV axes: x right, y down, z forward) as build_spec takes a fixed camera: world
    position and the MuJoCo-convention rotation (columns image right, image up, toward the viewer)."""
    r_cv = np.asarray(c["mat_world_from_cam"], float).reshape(3, 3)
    return {"pos": [float(v) for v in c["pos"]], "mat": (r_cv @ np.diag([1.0, -1.0, -1.0])).tolist(),
            "fovy": float(c["fovy_deg"]), "width": int(c["width"]), "height": int(c["height"])}


def surface_z_world(facts):
    """The work surface height in the env's world (m): the cell's surface_z (the same fact the plain arm's
    workspace bounds come from), else the problem's workspace_offset height, else None."""
    facts = facts or {}
    z = facts.get("surface_z")
    if isinstance(z, (int, float)) and not isinstance(z, bool):
        return float(z)
    off = facts.get("workspace_offset")
    if isinstance(off, (list, tuple)) and len(off) == 3:
        return float(off[2])
    return None


GRIP_BLOCK_FRAC = 0.15   # this gripper moves in eighths of its span per control step (a direction only, slewed)
                         # and its fingers trail the servo target by a tenth of the span: a guarded close counts a
                         # lag past this share as the jaws meeting something (rcpx's default, 0.01, fires on air)


def executor_cfg(facts, spec_facts):
    """The installation: a fixed base, the work surface under the arm as the floor rcpx keeps the links above
    (without one, the plane the base stands on), and the gripper's own tracking (GRIP_BLOCK_FRAC)."""
    cfg = {"base_kind": "fixed", "grip_block_frac": GRIP_BLOCK_FRAC}
    z = surface_z_world(facts)
    if z is not None:
        cfg["floor_z_m"] = round(float(world_to_b([0.0, 0.0, z], spec_facts["T_world_B"])[2]), 4)
    return cfg


def full_inertia(m, d, out):
    """Dense inertia matrix after a forward pass (mujoco 4 takes (m, d, dst); 3.x took (m, dst, qM))."""
    import mujoco

    try:
        mujoco.mj_fullM(m, d, out)
    except TypeError:
        mujoco.mj_fullM(m, out, d.qM)
    return out


def servo_gains(spec, law):
    """(kp, kd) per arm joint that rcpx plans with for an inertia weighted joint PD: tau = M(q) (kp_c e - kd_c qd)
    + bias. At home the joint stiffness is kp_c times the joint's own inertia (the diagonal of M, rotor armature
    included), and likewise the damping. A diagonal is what rcpx's compliance and speed rules take; the force
    observer uses the full law (bench.ext.libero.backend.ServoLawMixin)."""
    import mujoco
    import numpy as np

    from bench.robots import home_qpos, load_robot_mjcf

    m = load_robot_mjcf(spec).compile()
    d = mujoco.MjData(m)
    d.qpos[:] = home_qpos(m, spec)
    mujoco.mj_forward(m, d)
    M = full_inertia(m, d, np.zeros((m.nv, m.nv)))
    da = [m.jnt_dofadr[m.joint(j).id] for j in spec["arm_joints"]]
    diag = np.diag(M)[da]
    kp = [round(float(law["kp"][i] * diag[i]), 3) for i in range(len(da))]
    kd = [round(float(law["kd"][i] * diag[i]), 3) for i in range(len(da))]
    return kp, kd, [round(float(v), 5) for v in diag]


def _jsonable(x):
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if hasattr(x, "tolist"):
        return _jsonable(x.tolist())
    if hasattr(x, "item"):
        return x.item()
    return x


def derived_spec(path, name, wrist, cache_dir=CACHE_DIR):
    """derive_spec of the arm file with its own hand camera, cached beside the file by the file's text: the
    derivation (a seeded search) is the same for every episode of a robot class, and takes seconds."""
    import json

    from bench.derive_spec import derive_spec

    with open(path) as f:
        text = f.read()
    key = hashlib.sha256((text + json.dumps(wrist, sort_keys=True)).encode()).hexdigest()[:12]
    cpath = os.path.join(cache_dir, f"{name}_{key}.spec.json")
    if os.path.exists(cpath):
        with open(cpath) as f:
            return json.load(f)
    over = {"camera": (wrist, f"the arm's own hand camera {WRIST_CAMERA}, its mount and lens from the model file")}
    spec = _jsonable(derive_spec(path, name, overrides=over))
    tmp = f"{cpath}.{os.getpid()}.tmp"
    with open(tmp, "w") as f:
        json.dump(spec, f, sort_keys=True)
    os.replace(tmp, cpath)
    return spec


def fix_gripper(spec, finger_ctrlrange):
    """The gripper's open and closed ends from its finger servos, and the tip and tool surfaces again with the
    fingers closed. derive_spec sweeps both slide fingers from the low to the high end of their ranges together; on
    a hand whose fingers mirror each other (one range 0 to s, the other -s to 0) that sweep keeps the gap the same,
    so its ends and the tip it centres on the 'closed' fingers are off. Each finger servo here commands 0 to close
    (the end of its range nearer 0) and the far end to open, which is what the env's own gripper command does."""
    import mujoco

    from bench import derive_spec as DS

    g = dict(spec.get("gripper") or {})
    if not g or any(j not in finger_ctrlrange for j in g.get("joints", [])):
        return spec
    rng = [finger_ctrlrange[j] for j in g["joints"]]
    g["closed"] = [float(min(r, key=abs)) for r in rng]
    g["open"] = [float(max(r, key=abs)) for r in rng]
    m = mujoco.MjSpec.from_file(spec["mjcf"]).compile()
    d = mujoco.MjData(m)
    for j, v in zip(g["joints"], g["closed"]):
        d.qpos[m.jnt_qposadr[m.joint(j).id]] = v
    mujoco.mj_kinematics(m, d)
    palm = m.body(spec["tip"]["body"]).id
    fingers = [int(m.jnt_bodyid[m.joint(j).id]) for j in g["joints"]]
    last = m.joint(spec["arm_joints"][-1]).id
    Rp = d.xmat[palm].reshape(3, 3)
    xw = Rp @ np.asarray(spec["tip"]["x_axis"], dtype=float)
    tip_w = DS._slab_tip(DS._bodies_points(m, d, fingers), d.xanchor[last], xw)
    tip = dict(spec["tip"], pos=[round(float(v), 6) for v in DS._to_body(d, palm, p_world=tip_w)])
    gains = {k: spec[k] for k in ("kp", "kd", "torque_limit")}
    out = DS._orient_tool_up(spec["name"], spec["mjcf"], spec["arm_joints"], g, tip, spec.get("finger_tool"),
                             spec["q_home"], gains)
    if out is None:
        return dict(spec, gripper=g, tip=tip)
    fixed = out[0]
    new = dict(spec, gripper=g, tip=fixed["tip"], tool_surfaces=fixed["tool_surfaces"])
    new["derivation"] = dict(spec["derivation"], gripper=spec["derivation"].get("gripper", "") + "; ends from the "
                             "finger servos' ranges (0 closes each finger)", tip="centre of the distal slab of the "
                             "closed fingers")
    return _jsonable(new)


def build_spec(xml_text, q_home, law, wrist_px=WRIST_PX, scene_cameras=None, name=SPEC_NAME, cache_dir=CACHE_DIR,
               base_dir=None):
    """(spec, facts) for one task's arm. xml_text: the env host's robot_xml() file text (base_dir: its folder);
    q_home: the robot's home pose (rad; frame B's x points at the tip there); law: the env servo law
    ({"kp": [..], "kd": [..], ...}, see the backend);
    scene_cameras: {name: {"pos", "mat" (3x3 rows, MuJoCo camera convention: columns image right, image up, toward
    the viewer), "fovy", "width", "height"}} in the env's world, or None. The spec's scene_cameras give each one in
    frame B."""
    text, facts = arm_xml(xml_text, base_dir=base_dir)
    if base_dir is not None:
        text = _stable_assets(text, cache_dir)
    path = _write_cached(text, name, cache_dir)
    cam = facts["wrist_camera"]
    if cam is None:
        raise ValueError(f"robot xml has no camera {WRIST_CAMERA!r}")
    spec = derived_spec(path, name, wrist_camera_spec(cam, wrist_px[0], wrist_px[1]), cache_dir)
    q_home = [round(float(v), 6) for v in q_home]
    spec["q_home"] = q_home
    spec["source"]["q_home"] = "person"
    spec["overrides"] = [o for o in spec.get("overrides", []) if o.get("field") != "q_home"] + [
        {"field": "q_home", "value": q_home,
         "why": "the robot's own home pose: home, park and the standard look return there"}]
    spec["derivation"]["q_home"] = "the robot class's home pose (its init_qpos), as the env host builds it"
    spec = fix_gripper(spec, facts["finger_ctrlrange"])
    kp, kd, inertia = servo_gains(spec, law)
    spec["kp"], spec["kd"] = kp, kd
    spec["source"]["kp"] = spec["source"]["kd"] = "derived"
    spec["derivation"]["kp"] = ("the arm's joint position controller is inertia weighted (tau = M (kp e - kd qd) "
                                f"+ gravity): kp {law['kp'][0]:g} 1/s^2 times each joint's inertia at home")
    spec["derivation"]["kd"] = f"likewise: kd {law['kd'][0]:.3g} 1/s times each joint's inertia at home"
    spec["joint_inertia_home"] = inertia
    T = frame_b(spec, facts["base_pos"], facts["base_quat"])
    facts["T_world_B"] = T.tolist()
    spec["scene_cameras"] = {}
    for cname, c in (scene_cameras or {}).items():
        spec["scene_cameras"][cname] = camera_in_b(c, T)
    facts["xml_path"] = path
    return spec, facts


def frame_b(spec, base_pos, base_quat):
    """4x4 pose of frame B in the env's world. rcpx re-poses the root body so frame B holds (bench.robots
    load_robot_mjcf); the root body's pose in frame B, against its pose in the world (the base's place composed with
    the link's place in the base, as arm_xml put it), gives frame B."""
    import mujoco
    import numpy as np

    from bench.robots import home_qpos, load_robot_mjcf

    m = load_robot_mjcf(spec).compile()
    d = mujoco.MjData(m)
    d.qpos[:] = home_qpos(m, spec)
    mujoco.mj_kinematics(m, d)
    rid = m.body(FIRST_LINK).id
    T_B_root = np.eye(4)
    T_B_root[:3, :3] = d.xmat[rid].reshape(3, 3)
    T_B_root[:3, 3] = d.xpos[rid]
    # the root's world pose: arm_xml wrote it into the file it built from the base and link poses
    s = mujoco.MjSpec.from_file(spec["mjcf"])
    b = s.body(FIRST_LINK)
    T_W_root = np.eye(4)
    T_W_root[:3, :3] = np.array(quat_to_mat(list(b.quat)))
    T_W_root[:3, 3] = np.array(list(b.pos), dtype=float)
    return T_W_root @ np.linalg.inv(T_B_root)


def camera_in_b(cam, T_W_B):
    """A fixed camera's installation pose in frame B: {"position_m", "R" (3x3 rows, MuJoCo convention: columns image
    right, image up, toward the viewer), "looks_along", "image_right", "image_down", "fovy", "width", "height", "fx",
    "fy", "cx", "cy"}; the intrinsics are the pinhole of a MuJoCo camera of that fovy (square pixels, centred)."""
    import numpy as np

    T = np.asarray(T_W_B, dtype=float)
    Rw = np.asarray(cam["mat"], dtype=float).reshape(3, 3)
    pw = np.asarray(cam["pos"], dtype=float)
    R = T[:3, :3].T @ Rw
    p = T[:3, :3].T @ (pw - T[:3, 3])
    w, h, fovy = int(cam["width"]), int(cam["height"]), float(cam["fovy"])
    f = (h / 2.0) / math.tan(math.radians(fovy) / 2.0)
    return {"position_m": [round(float(v), 6) for v in p], "R": [[round(float(v), 8) for v in row] for row in R],
            "looks_along": [round(float(v), 6) for v in -R[:, 2]], "image_right": [round(float(v), 6) for v in R[:, 0]],
            "image_down": [round(float(v), 6) for v in -R[:, 1]], "fovy": fovy, "width": w, "height": h,
            "fx": round(f, 4), "fy": round(f, 4), "cx": w / 2.0, "cy": h / 2.0}


def world_to_b(p_world, T_W_B):
    """A world point (m) in frame B (m)."""
    import numpy as np

    T = np.asarray(T_W_B, dtype=float)
    return T[:3, :3].T @ (np.asarray(p_world, dtype=float) - T[:3, 3])


def b_to_world(p_b, T_W_B):
    import numpy as np

    T = np.asarray(T_W_B, dtype=float)
    return T[:3, :3] @ np.asarray(p_b, dtype=float) + T[:3, 3]


__all__ = ["BASE_BODY", "EEF_SITE", "FIRST_LINK", "FT_SITE", "GRIP_BLOCK_FRAC", "HOME_QPOS", "WRIST_CAMERA", "arm_xml",
           "b_to_world", "build_spec", "camera_from_host", "camera_in_b", "executor_cfg", "fix_gripper", "frame_b",
           "home_pose", "quat_to_mat", "servo_gains", "surface_z_world", "world_to_b", "wrist_camera_spec"]
