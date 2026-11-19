# SPDX-License-Identifier: AGPL-3.0-only
"""Servo gains, torque limits and joint speed limits read from a robot's model file (moved out of bench/derive_spec.py;
that module still offers the same names).

Nothing here is typed per robot. gains(m_raw, spec, joints) reads each joint's actuator in the file (position gain,
velocity gain, force range), makes the pair stable for the bench's explicit servo law at its physics step, and where the
file states nothing falls back on the joint's own inertia and gravity load: a fallback is an ASSUMPTION, listed by joint
in the fifth value it returns, never a rating (nothing downstream may treat it as one). file_speed(m, jid) is the file's
own joint speed limit where it has one (a velocity servo's control range); a file with none gives None and the robot
measures what it tracks. fill_from_file(spec) completes a hand-written spec: kp, kd, torque_limit of the arm, kp and
forcerange of a gripper block that states none, with spec["source"], spec["derivation"] and spec["assumed"] saying so.
"""

import math

import mujoco
import numpy as np

DT_PHYS = 0.002  # backend physics step the servo law runs at (SPEC 3)
KD_CAP = 0.3  # kd <= KD_CAP * inertia / DT_PHYS keeps the explicit damping stable
ZETA_MIN = 0.9  # damping ratio floor at the smallest inertia
ASSUMED_LIMIT = "ASSUMED, not a rating: limit from inertia and gravity load (file states none)"
ASSUMED_KP = "ASSUMED: kp from inertia and gravity load (file states none)"


def joint_actuator(m, jid):
    """(actuator id, transmission coefficient) of the first actuator driving joint jid, or (None, 0)."""
    for a in range(m.nu):
        if int(m.actuator_trntype[a]) == int(mujoco.mjtTrn.mjTRN_JOINT) and m.actuator_trnid[a, 0] == jid:
            return a, float(m.actuator_gear[a, 0])
        if int(m.actuator_trntype[a]) == int(mujoco.mjtTrn.mjTRN_TENDON):
            t = m.actuator_trnid[a, 0]
            for w in range(m.tendon_adr[t], m.tendon_adr[t] + m.tendon_num[t]):
                if int(m.wrap_type[w]) == int(mujoco.mjtWrap.mjWRAP_JOINT) and m.wrap_objid[w] == jid:
                    return a, float(m.actuator_gear[a, 0] * m.wrap_prm[w])
    return None, 0.0


def file_gains(m, jid):
    """(kp, kv, torque limit) the file states for joint jid; None where it states none."""
    a, c = joint_actuator(m, jid)
    kp = kv = lim = None
    if a is not None:
        aff = int(m.actuator_biastype[a]) == int(mujoco.mjtBias.mjBIAS_AFFINE)
        if aff and int(m.actuator_gaintype[a]) == int(mujoco.mjtGain.mjGAIN_FIXED) and m.actuator_biasprm[a, 1] < 0:
            kp = float(-m.actuator_biasprm[a, 1] * abs(c))
            kv = float(-m.actuator_biasprm[a, 2] * abs(c)) or None
        if m.actuator_forcelimited[a]:
            lim = float(np.abs(m.actuator_forcerange[a]).max() * abs(c))
        elif int(m.actuator_biastype[a]) == int(mujoco.mjtBias.mjBIAS_NONE) and m.actuator_ctrllimited[a]:
            lim = float(np.abs(m.actuator_ctrlrange[a]).max() * m.actuator_gainprm[a, 0] * abs(c))
    if m.jnt_actfrclimited[jid]:
        jl = float(np.abs(m.jnt_actfrcrange[jid]).max())
        lim = jl if lim is None else min(lim, jl)
    return kp, kv, lim


def file_speed(m, jid):
    """The joint speed limit (rad/s, or m/s for a slide) the file states for joint jid, or None. MJCF has no joint
    speed attribute; the one place a file states a speed is a velocity servo on the joint (force = kv (ctrl - speed),
    no position term) with a limited control range: its largest command is the fastest the file lets the joint be
    asked to go."""
    a, c = joint_actuator(m, jid)
    if a is None or not m.actuator_ctrllimited[a] or abs(c) <= 0.0:
        return None
    aff = int(m.actuator_biastype[a]) == int(mujoco.mjtBias.mjBIAS_AFFINE)
    fixed = int(m.actuator_gaintype[a]) == int(mujoco.mjtGain.mjGAIN_FIXED)
    if aff and fixed and m.actuator_biasprm[a, 1] == 0 and m.actuator_biasprm[a, 2] < 0:
        return float(np.abs(m.actuator_ctrlrange[a]).max() / abs(c))
    return None


def inertia_and_gravity(spec, joints, n_poses=48):
    """Per joint: smallest and largest joint-space inertia and the largest gravity torque over seeded poses."""
    from bench.robots import home_qpos, load_robot_mjcf

    m = load_robot_mjcf(spec).compile()
    d = mujoco.MjData(m)
    arm = [m.joint(j).id for j in spec["arm_joints"]]
    lo = np.array([m.jnt_range[j][0] for j in arm])
    hi = np.array([m.jnt_range[j][1] for j in arm])
    da = [m.jnt_dofadr[m.joint(j).id] for j in joints]
    rng = np.random.default_rng(1)
    M = np.zeros((m.nv, m.nv))
    I, G = [], []
    for k in range(n_poses):
        d.qpos[:] = home_qpos(m, spec)
        if k:
            d.qpos[[m.jnt_qposadr[j] for j in arm]] = rng.uniform(lo, hi)
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        mujoco.mj_fullM(m, d, M)
        I.append(np.diag(M)[da])
        G.append(np.abs(d.qfrc_bias[da]))
    I, G = np.array(I), np.array(G)
    return I.min(axis=0), I.max(axis=0), G.max(axis=0)


def stable(kp, kv, i_min):
    """kd capped for the explicit law at DT_PHYS, raised toward ZETA_MIN of critical; kp lowered to keep ZETA_MIN."""
    crit = 2.0 * ZETA_MIN * math.sqrt(kp * i_min)
    kd = min(max(kv if kv else crit, crit), KD_CAP * i_min / DT_PHYS)
    if kd < crit:
        kp = (kd / (2.0 * ZETA_MIN)) ** 2 / i_min
    return round(kp, 1), round(kd, 2)


def gains(m_raw, spec, joints):
    """(kp, kd, torque limits, how per joint, assumed): assumed = {"torque_limit": [joints], "kp": [joints]} names
    the joints whose value the file does not state (a fallback from inertia and gravity load, said so in how)."""
    i_min, i_max, g_max = inertia_and_gravity(spec, joints)
    kps, kds, lims, how = [], [], [], []
    assumed = {"torque_limit": [], "kp": []}
    for j, a, b, g in zip(joints, i_min, i_max, g_max):
        kp, kv, lim = file_gains(m_raw, m_raw.joint(j).id)
        h = []
        if lim is None:
            lim = max(2.5 * g, 20.0 * b)
            h.append(ASSUMED_LIMIT)
            assumed["torque_limit"].append(j)
        if kp is None:
            kp = max(g / math.radians(0.5), b * (2 * math.pi * 2.0) ** 2)
            h.append(ASSUMED_KP)
            assumed["kp"].append(j)
        kp, kd = stable(kp, kv, a)
        kps.append(float(kp))
        kds.append(float(kd))
        lims.append(round(float(lim), 3))
        how.append("; ".join(h) or "file actuator, made stable at the joint inertia")
    return kps, kds, lims, how, assumed


def source_word(assumed, field):
    """The spec.source word of a gain field: derived, with the joints whose value is an assumption named."""
    names = assumed.get(field) or []
    return "derived" if not names else "derived; ASSUMED (the file states none) for " + ", ".join(names)


def fill_from_file(spec):
    """Complete a hand-written spec in place from its model file: kp, kd and torque_limit of the arm joints, and kp
    and forcerange of a gripper block that states none. Seeded, no wall clock; the file changed, the spec changes."""
    m = mujoco.MjSpec.from_file(spec["mjcf"]).compile()
    n = len(spec["arm_joints"])
    g = spec.get("gripper")
    prov = dict(spec, kp=[1.0] * n, kd=[1.0] * n, torque_limit=[1.0] * n)
    if g:
        k = len(g["joints"])
        prov["gripper"] = dict(g, kp=g.get("kp") or [1.0] * k, forcerange=g.get("forcerange") or [1.0] * k)
    kp, kd, lim, how, assumed = gains(m, prov, spec["arm_joints"])
    spec.update(kp=kp, kd=kd, torque_limit=lim)
    src = dict(spec.get("source") or {})
    src.update(kp=source_word(assumed, "kp"), kd="derived", torque_limit=source_word(assumed, "torque_limit"))
    how_all = dict(spec.get("derivation") or {})
    how_all.update(kp=how, kd="kd capped at the explicit law limit, damping ratio >= 0.9",
                   torque_limit="file actuator force range where stated")
    if g and (g.get("kp") is None or g.get("forcerange") is None):
        gkp, _, glim, ghow, gass = gains(m, prov, g["joints"])
        g["kp"] = gkp if g.get("kp") is None else g["kp"]
        g["forcerange"] = glim if g.get("forcerange") is None else g["forcerange"]
        src.setdefault("gripper", "person; kp and forcerange derived from the file's actuator")
        how_all["gripper.kp"] = ghow
        assumed = {f: assumed[f] + gass[f] for f in assumed}
    spec["source"], spec["derivation"] = src, how_all
    spec["assumed"] = {f: v for f, v in assumed.items() if v}
    return spec
