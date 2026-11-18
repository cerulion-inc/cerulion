# SPDX-License-Identifier: AGPL-3.0-only
"""Robot registry and the robot loader.

ROBOTS = {name: SPEC}: the four hand-written arms, plus the arms whose spec is derived from the model file alone
(bench.robots_extra: gen3, koch). A derived spec is built the first time it is asked for (ROBOTS[name], iteration,
items), so importing this module costs nothing for them and a file that cannot be derived cannot break the others.
A hand-written spec types no servo gain and no torque limit: the first read of it through ROBOTS fills kp, kd and
torque_limit (and a gripper block's kp and forcerange where it states none) from its model file
(bench/robots/_gains.py), in place, so the module's own SPEC object is complete from then on. Every spec carries
speed_limit: the maker's RATED joint speed per arm joint with the document named in spec["source"], or None where no
document is named (the robot's own sweep then measures it).

load_robot_mjcf(spec) -> mujoco.MjSpec of the robot alone:
- every menagerie actuator and keyframe removed; one torque motor per arm joint ("motor_<joint>", ctrl = N m,
  ctrlrange = +-torque_limit) and one position actuator per gripper joint (names from spec["gripper"]["actuators"]);
- a fingertip paddle body "finger_tool" fixed to the flange when spec["finger_tool"] is set;
- a site "tip" at the tool tip (frame T), sites "surface_<name>" for every tool surface, a camera "wrist";
- a contact exclude for every body pair in spec["collision_excludes"] ([{"bodies": [a, b], "why": text}]): pairs
  whose collision hulls meet where the links themselves do not. The scene and RobotModel both load through here,
  so the simulator and the self-collision model always agree on which pairs can meet;
- the root body re-posed so frame B holds: origin on the mounting plane under the first joint axis, x toward the
  tip at q_home, z up. The re-pose is computed from the compiled model, never typed per robot.
"""

import math

import mujoco
import numpy as np

from . import _build
from .lite6 import SPEC as LITE6
from .panda import SPEC as PANDA
from .so100 import SPEC as SO100
from .ur5e import SPEC as UR5E

DERIVED_ROBOTS = ("gen3", "koch")  # specs bench.robots_extra derives from the model file (EXTENSIONS.md)


class _Registry(dict):
    """{name: SPEC} whose derived entries are built on first use. Every way of reading goes through _fill, and
    __iter__ is overridden so dict(ROBOTS) and {**ROBOTS} read through it too."""

    def __init__(self, specs, derived):
        super().__init__(specs)
        self._pending = [n for n in derived if n not in specs]
        for n in self._pending:
            super().__setitem__(n, None)

    def _gains(self, name=None):
        """kp, kd and torque_limit of the hand-written specs, from their model files, the first time one is read."""
        for n in ([name] if name is not None else list(super().keys())):
            spec = super().get(n)
            if isinstance(spec, dict) and "kp" not in spec:
                from ._gains import fill_from_file

                fill_from_file(spec)

    def _fill(self, name=None):
        self._gains(name)
        if not self._pending or (name is not None and name not in self._pending):
            return
        from bench.robots_extra import EXTRA_ROBOTS  # derives the specs: seeded, no wall clock

        for n in list(self._pending):
            self._pending.remove(n)
            super().__setitem__(n, EXTRA_ROBOTS[n])

    def __getitem__(self, name):
        self._fill(name)
        return super().__getitem__(name)

    def get(self, name, default=None):
        self._fill(name)
        return super().get(name, default)

    def __setitem__(self, name, spec):
        if name in self._pending:
            self._pending.remove(name)
        super().__setitem__(name, spec)

    def update(self, *args, **kw):
        for name, spec in dict(*args, **kw).items():
            self[name] = spec

    def __iter__(self):
        return super().__iter__()

    def items(self):
        self._fill()
        return super().items()

    def values(self):
        self._fill()
        return super().values()

    def copy(self):
        self._fill()
        return dict(super().items())


ROBOTS = _Registry({"so100": SO100, "ur5e": UR5E, "panda": PANDA, "lite6": LITE6}, DERIVED_ROBOTS)


def _root_body(s, joint_name):
    b = s.joint(joint_name).parent
    while b.parent is not None and b.parent.name != "world":
        b = b.parent
    return b


def _add_finger_tool(s, ft):
    R = _build.mat_from_xz(ft["x_axis"], ft["z_axis"])
    body = s.body(ft["parent"]).add_body(name="finger_tool", pos=list(ft["pos"]), quat=list(_build.quat_from_mat(R)))
    L, W, T = (v / 1000.0 for v in ft["size_mm"])
    r = T / 2.0
    rgba = [0.85, 0.55, 0.15, 1.0]
    # Rounded box: a box from the flange to L - r and a capsule across the width that rounds the end.
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, name="finger_tool_box", size=[(L - r) / 2.0, W / 2.0, T / 2.0],
                  pos=[(L - r) / 2.0, 0, 0], rgba=rgba, mass=ft.get("mass_kg", 0.05))
    cap_q = _build.quat_from_mat(np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]]))  # capsule axis along y
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, name="finger_tool_end", size=[r, max(W / 2.0 - r, 1e-4), 0],
                  pos=[L - r, 0, 0], quat=list(cap_q), rgba=rgba, mass=0.0)


def _add_motors(s, spec):
    for j, lim in zip(spec["arm_joints"], spec["torque_limit"]):
        a = s.add_actuator(name="motor_" + j, target=j)
        a.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a.set_to_motor()
        a.ctrllimited = 1
        a.ctrlrange = [-float(lim), float(lim)]
        a.forcelimited = 1
        a.forcerange = [-float(lim), float(lim)]
    g = spec.get("gripper")
    if g:
        for j, name, kp, fr in zip(g["joints"], g["actuators"], g["kp"], g["forcerange"]):
            a = s.add_actuator(name=name, target=j)
            a.trntype = mujoco.mjtTrn.mjTRN_JOINT
            a.set_to_position(kp=float(kp), dampratio=1.0)
            a.forcelimited = 1
            a.forcerange = [-float(fr), float(fr)]


def _add_sites_and_camera(s, spec):
    tip = spec["tip"]
    Rt = _build.mat_from_xz(tip["x_axis"], tip["z_axis"])
    s.body(tip["body"]).add_site(name="tip", pos=list(tip["pos"]), quat=list(_build.quat_from_mat(Rt)),
                                 size=[0.003, 0, 0], rgba=[1, 0, 0, 0.6], group=4)
    for surf in spec["tool_surfaces"]:
        n = _build.unit(surf["normal"])
        # site z along the outward normal; x any perpendicular
        ref = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
        x = _build.unit(ref - n * float(ref @ n))
        Rs = np.column_stack([x, np.cross(n, x), n])
        s.body(surf["body"]).add_site(name="surface_" + surf["name"], pos=list(surf["pos"]),
                                      quat=list(_build.quat_from_mat(Rs)), size=[0.002, 0, 0], group=4)
    cam = spec["camera"]
    Rc = _build.mat_from_xyaxes(cam["xyaxes"])
    c = s.body(cam["body"]).add_camera(name="wrist", pos=list(cam["pos"]), quat=list(_build.quat_from_mat(Rc)))
    c.fovy = float(cam["fovy"])
    c.resolution = [int(cam["width"]), int(cam["height"])]


def _add_excludes(s, spec):
    for ex in spec.get("collision_excludes") or ():
        e = s.add_exclude()
        e.bodyname1, e.bodyname2 = ex["bodies"]


def _set_gripper_closed(m, d, spec):
    g = spec.get("gripper")
    if g:
        for j, v in zip(g["joints"], g["closed"]):
            d.qpos[m.jnt_qposadr[m.joint(j).id]] = v


def _geom_lowest_z(m, d, gid):
    if m.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH:
        mid = m.geom_dataid[gid]
        v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        w = v @ d.geom_xmat[gid].reshape(3, 3).T + d.geom_xpos[gid]
        return float(w[:, 2].min())
    return float(d.geom_xpos[gid][2] - m.geom_rbound[gid])


def home_qpos(m, spec):
    """Full qpos of model m at the spec home pose with the gripper closed."""
    d = mujoco.MjData(m)
    for j, v in zip(spec["arm_joints"], spec["q_home"]):
        d.qpos[m.jnt_qposadr[m.joint(j).id]] = v
    _set_gripper_closed(m, d, spec)
    return d.qpos.copy()


def load_robot_mjcf(spec):
    """mujoco.MjSpec of the robot alone, in frame B, with motors, tool, tip site and wrist camera."""
    s = mujoco.MjSpec.from_file(spec["mjcf"])
    for a in list(s.actuators):
        s.delete(a)
    for k in list(s.keys):
        s.delete(k)
    if spec.get("finger_tool"):
        _add_finger_tool(s, spec["finger_tool"])
    _add_sites_and_camera(s, spec)
    _add_motors(s, spec)
    _add_excludes(s, spec)
    root = _root_body(s, spec["arm_joints"][0])
    m = s.compile()
    d = mujoco.MjData(m)
    d.qpos[:] = home_qpos(m, spec)
    mujoco.mj_kinematics(m, d)
    j0 = m.joint(spec["arm_joints"][0]).id
    a = d.xanchor[j0].copy()
    tip = d.site_xpos[m.site("tip").id].copy()
    yaw = -math.atan2(tip[1] - a[1], tip[0] - a[0])
    rid = m.body(root.name).id
    zmin = min(_geom_lowest_z(m, d, g) for g in range(m.ngeom) if m.geom_bodyid[g] == rid)
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1.0]])
    ra = Rz @ a
    t = np.array([-ra[0], -ra[1], -zmin])
    R_old = _build.mat_from_quat(d.xquat[rid])
    p_old = d.xpos[rid].copy()
    root.pos = list(Rz @ p_old + t)
    root.quat = list(_build.quat_from_mat(Rz @ R_old))
    return s
