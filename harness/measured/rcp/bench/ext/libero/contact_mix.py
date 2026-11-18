# SPDX-License-Identifier: AGPL-3.0-only
"""The contact rate the robot's own bodies can meet the scene with, by the simulator's own mixing rule (the design rulings,
the design rulings): pure over a compiled MjModel, no stepping. The env host calls rates() on the whole scene and answers by the
robot's own body names only (bench/ext/libero/envhost.py op contact_rates); the backend folds the answer into its
contact_model (bench/ext/libero/backend.py). Nothing here names a scene item in what it returns.

A contact between two geoms takes its solver parameters by MuJoCo's rule (mj_contactParam, engine_collision_driver.c):
  - different priority: the solref and solimp of the geom with the higher priority;
  - the same priority: mix = solmix1 / (solmix1 + solmix2) (0.5 when both are under mjMINVAL, 0 or 1 when one is);
    solref mixed by mix where both are standard (time constant first, positive), else the smaller of each element;
    solimp mixed by mix;
  - an explicit contact pair carries its own solref and solimp; the global override (mjENBL_OVERRIDE) replaces all.
The rate of the damped spring that contact is (the law the round's figures read, rcpx/bounds_know_point.py): with d
the top of the impedance (solimp[1], at most 1), b = 2 / (d timeconst) for a standard solref, the time constant kept
at two physics steps or more where the simulator's refsafe is on; b = damping / d for a direct one (-stiffness,
-damping).

    rates(m, is_robot_body) -> {"by_body": {robot body: the stiffest rate any of its geoms can meet a scene geom
                                with}, "own_by_body": {robot body: its own geoms' rate alone}, "pairs": count read,
                                "override": bool, "refsafe": bool, "timestep_s"}
"""

import numpy as np

MINVAL = 1e-15          # mjMINVAL
ENBL_OVERRIDE = 1 << 0  # mjENBL_OVERRIDE
DSBL_REFSAFE = 1 << 14  # mjDSBL_REFSAFE (MuJoCo 2.3 and later)


def _refsafe(m):
    try:
        import mujoco
        bit = int(mujoco.mjtDisableBit.mjDSBL_REFSAFE)
    except Exception:
        bit = DSBL_REFSAFE
    return not (int(m.opt.disableflags) & bit)


def _override(m):
    try:
        import mujoco
        bit = int(mujoco.mjtEnableBit.mjENBL_OVERRIDE)
    except Exception:
        bit = ENBL_OVERRIDE
    return bool(int(m.opt.enableflags) & bit)


def mixed(m, g1, g2):
    """(solref, solimp) of a contact between geoms g1 and g2 by the simulator's rule (module text)."""
    if _override(m):
        return np.asarray(m.opt.o_solref, dtype=float), np.asarray(m.opt.o_solimp, dtype=float)
    p1, p2 = int(m.geom_priority[g1]), int(m.geom_priority[g2])
    if p1 != p2:
        g = g1 if p1 > p2 else g2
        return np.asarray(m.geom_solref[g], dtype=float), np.asarray(m.geom_solimp[g], dtype=float)
    s1, s2 = float(m.geom_solmix[g1]), float(m.geom_solmix[g2])
    if s1 >= MINVAL and s2 >= MINVAL:
        mix = s1 / (s1 + s2)
    elif s1 < MINVAL and s2 < MINVAL:
        mix = 0.5
    elif s1 < MINVAL:
        mix = 0.0
    else:
        mix = 1.0
    r1, r2 = np.asarray(m.geom_solref[g1], dtype=float), np.asarray(m.geom_solref[g2], dtype=float)
    ref = mix * r1 + (1.0 - mix) * r2 if (r1[0] > 0 and r2[0] > 0) else np.minimum(r1, r2)
    imp = mix * np.asarray(m.geom_solimp[g1], dtype=float) + (1.0 - mix) * np.asarray(m.geom_solimp[g2], dtype=float)
    return ref, imp


def rate(m, ref, imp, safe=None):
    """The damped spring's rate b (1/s) of solref ref and solimp imp (module text); None where the impedance top is
    not above 0."""
    d = min(float(imp[1]), 1.0)
    if not d > 0.0:
        return None
    if float(ref[0]) > 0.0:
        safe = _refsafe(m) if safe is None else safe
        tc = max(float(ref[0]), 2.0 * float(m.opt.timestep)) if safe else float(ref[0])
        return 2.0 / (d * tc)
    return -float(ref[1]) / d


def can_touch(m, g1, g2):
    """The two geoms' bitmasks let them collide (contype of one against conaffinity of the other)."""
    return bool((int(m.geom_contype[g1]) & int(m.geom_conaffinity[g2]))
                or (int(m.geom_contype[g2]) & int(m.geom_conaffinity[g1])))


def rates(m, is_robot_body):
    """Module text. is_robot_body(body id) -> bool."""
    safe = _refsafe(m)
    live = [g for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]]
    mine = [g for g in live if is_robot_body(int(m.geom_bodyid[g]))]
    world = [g for g in live if not is_robot_body(int(m.geom_bodyid[g]))]

    def name(g):
        return m.body(int(m.geom_bodyid[g])).name

    by_body, own = {}, {}
    for g in mine:
        b = rate(m, np.asarray(m.geom_solref[g], dtype=float), np.asarray(m.geom_solimp[g], dtype=float), safe)
        if b is not None:
            own[name(g)] = max(own.get(name(g), 0.0), b)
    n = 0
    for g1 in mine:
        for g2 in world:
            if not can_touch(m, g1, g2):
                continue
            ref, imp = mixed(m, g1, g2)
            b = rate(m, ref, imp, safe)
            n += 1
            if b is not None:
                by_body[name(g1)] = max(by_body.get(name(g1), 0.0), b)
    for p in range(int(getattr(m, "npair", 0))):
        g1, g2 = int(m.pair_geom1[p]), int(m.pair_geom2[p])
        r1, r2 = is_robot_body(int(m.geom_bodyid[g1])), is_robot_body(int(m.geom_bodyid[g2]))
        if r1 == r2:
            continue
        g = g1 if r1 else g2
        ref = m.opt.o_solref if _override(m) else m.pair_solref[p]
        imp = m.opt.o_solimp if _override(m) else m.pair_solimp[p]
        b = rate(m, np.asarray(ref, dtype=float), np.asarray(imp, dtype=float), safe)
        n += 1
        if b is not None:
            by_body[name(g)] = max(by_body.get(name(g), 0.0), b)
    return {"by_body": by_body, "own_by_body": own, "pairs": n, "override": _override(m), "refsafe": safe,
            "timestep_s": float(m.opt.timestep)}
