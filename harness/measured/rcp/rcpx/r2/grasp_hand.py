# SPDX-License-Identifier: AGPL-3.0-only
"""The hand of package grasp: jaw geometry and the gripper reading, derived from the robot's own model.

Nothing here is ground truth about the world. Everything comes from the robot_spec's gripper block (joints, open and
closed values, and the servo gain and force range when the spec gives them) and the loaded kinematic model:

- jaw sides: the body of each gripper joint is a moving jaw; a gripper with one moving jaw closes against the body
  that carries that joint (the fixed jaw);
- jaw points: the tool tip site, expressed in each side's body frame with the gripper closed. With the jaws open
  those points sit on each jaw at the tips, so the opening at the jaw tips is the distance between them (a part held
  deeper in a hinged jaw reads wider than it is; a parallel gripper reads its true gap). The opening reads 0 at the
  spec's closed value, so a spec whose closed value leaves a gap between the jaws under-reads by that gap;
- grip force: a position servo pushes kp times its lag (target minus measured) up to its force range, less the jaw
  joint's dry friction from the model (a lag inside that band is stiction, not a part); on a hinge that torque acts
  at the jaw point's distance from the axis. Own jaw gravity is included from the model at the current arm pose.

SI inside (m, rad, N); the Mixin turns results into mm and newtons.
"""

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from bench.robots import home_qpos
from rcpx import bounds_ratios as R

MIN_LAG_FRAC = 0.01    # kept: lag in a jaw whose spec gives no gain
N_TABLE = 81           # geometry table, not a motion limit
MM = 1000.0            # unit: m to mm


def finite(*xs):
    """Every argument is a finite number."""
    try:
        return all(math.isfinite(float(x)) for x in xs)
    except (TypeError, ValueError):
        return False


class Hand:
    """Jaw geometry and gripper readings for one robot model (RobotModel); None-safe readers."""

    def __init__(self, model):
        """Build the hands geometry table and servo facts from its own robot model."""
        spec = model.spec
        g = spec["gripper"]
        m = model.mj
        self.m, self.d = m, mujoco.MjData(m)
        self.last_width_m, self.step_m = None, 0.0
        self.arm_q = None
        self.joints = list(g["joints"])
        jids = [m.joint(j).id for j in self.joints]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jids])
        self.arm_qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in spec["arm_joints"]])
        self.closed = np.asarray(g["closed"], dtype=float)
        self.open = np.asarray(g["open"], dtype=float)
        self.span = self.open - self.closed
        self.kp = np.asarray(g["kp"], dtype=float) if g.get("kp") is not None else None
        self.force_range = np.asarray(g["forcerange"], dtype=float) if g.get("forcerange") is not None else None
        # the jaw joints' dry friction in the loaded model: a servo lag inside it is the jaw's own stiction
        self.friction = np.asarray([float(m.dof_frictionloss[m.jnt_dofadr[j]]) for j in jids])
        bodies = [int(m.jnt_bodyid[j]) for j in jids]
        self.hinge = [int(m.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_HINGE) for j in jids]
        self.sides = list(bodies) if len(bodies) > 1 else [bodies[0], int(m.body_parentid[bodies[0]])]
        self.moving_side = 0
        self.q_home_full = home_qpos(m, spec)
        d = self.d
        d.qpos[:] = self.q_home_full
        d.qpos[self.qadr] = self.closed
        mujoco.mj_kinematics(m, d)
        tip = d.site_xpos[m.site("tip").id].copy()
        self.local = [d.xmat[b].reshape(3, 3).T @ (tip - d.xpos[b]) for b in self.sides]
        self.lever = []
        for j, b, is_hinge in zip(jids, bodies, self.hinge):
            if not is_hinge:
                self.lever.append(1.0)
                continue
            a, ax = d.xanchor[j].copy(), d.xaxis[j].copy()
            r = tip - a
            self.lever.append(max(float(np.linalg.norm(r - (r @ ax) * ax)), 1e-3))
        self.lever = np.asarray(self.lever)
        # Project the nearest collision pad onto the finger's radius from its hinge.
        # Its inner end reads the opening at the hinge side, without a pad side-edge offset.
        self.depth_local = []
        moving_anchor = d.xanchor[jids[0]].copy()
        for b, loc in zip(self.sides, self.local):
            anchor = d.xmat[b].reshape(3, 3).T @ (moving_anchor - d.xpos[b])
            radial = loc - anchor
            norm = float(np.linalg.norm(radial))
            radial = radial / norm if norm > 0 else np.zeros_like(radial)
            pads = []
            for gid in range(m.ngeom):
                if m.geom_bodyid[gid] != b or not (m.geom_contype[gid] or m.geom_conaffinity[gid]):
                    continue
                if m.geom_type[gid] == mujoco.mjtGeom.mjGEOM_BOX:
                    rot = np.zeros(9)
                    mujoco.mju_quat2Mat(rot, m.geom_quat[gid])
                    corners = [m.geom_pos[gid] + rot.reshape(3, 3) @ (m.geom_size[gid] * [x, y, z])
                               for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
                    pads.append(corners)
            if pads:
                pad = min(pads, key=lambda corners: min(float(np.linalg.norm(p - loc)) for p in corners))
                inner = min(float((p - anchor) @ radial) for p in pad)
                self.depth_local.append(anchor + inner * radial)
            else:
                self.depth_local.append(loc)

        fracs = np.linspace(0.0, 1.0, N_TABLE)
        self.table = np.array([self.width_m(self.q_of_frac(f)) for f in fracs])
        self.fracs = fracs
        self.max_m = float(self.table.max())
        self.monotone = bool(np.all(np.diff(self.table) >= -1e-6))

    # ---------------------------------------------------------------- geometry
    def q_of_frac(self, f):
        """Convert a tip opening fraction to the gripper joint values."""
        return self.closed + float(f) * self.span

    def frac_of_q(self, q):
        """Read the mean opening fraction of the moving jaw joints."""
        q = np.asarray(q, dtype=float)
        span = np.where(np.abs(self.span) > 1e-9, self.span, 1.0)
        return float(np.mean((q - self.closed) / span))

    def _points(self, q_grip, q_arm=None):
        """Tip points of every jaw from the robot model at the stated or current arm pose."""
        d = self.d
        d.qpos[:] = self.q_home_full
        arm = self.arm_q if q_arm is None else q_arm
        if arm is not None:
            d.qpos[self.arm_qadr] = np.asarray(arm, dtype=float)
        d.qpos[self.qadr] = np.asarray(q_grip, dtype=float)
        mujoco.mj_kinematics(self.m, d)
        return [d.xpos[b] + d.xmat[b].reshape(3, 3) @ loc for b, loc in zip(self.sides, self.local)]

    def width_m(self, q_grip):
        """Opening at the jaw tips (m) for gripper joint values q_grip; more than two jaws: the mean pair gap."""
        pts = self._points(q_grip)
        gaps = [float(np.linalg.norm(pts[i] - pts[k])) for i in range(len(pts)) for k in range(i + 1, len(pts))]
        return float(np.mean(gaps))

    def frac_for_width(self, w_m):
        """The opening fraction whose tip width is w_m (clipped to the table); the table must be monotone."""
        w = min(max(float(w_m), float(self.table[0])), float(self.table[-1]))
        if self.monotone:
            return float(np.interp(w, self.table, self.fracs))
        return float(self.fracs[int(np.argmin(np.abs(self.table - w)))])

    def jaw_point(self, q_arm, q_grip, side=None):
        """Base-frame point (m) of a jaw's tip point at arm pose q_arm (the model's frame is frame B)."""
        return self._points(q_grip, q_arm)[self.moving_side if side is None else side]

    # ---------------------------------------------------------------- readings
    def reading(self, g, cmd_frac=None):
        """(q, target) from a Backend.read()["gripper"] block; target falls back to the commanded fraction."""
        if not isinstance(g, dict):
            return None, None
        q = g.get("q")
        if q is None and g.get("open_frac") is not None:
            q = self.q_of_frac(g["open_frac"])
        if q is None:
            return None, None
        q = np.asarray(q, dtype=float).reshape(-1)
        target = g.get("target")
        if target is None and cmd_frac is not None:
            target = self.q_of_frac(cmd_frac)
        return q, None if target is None else np.asarray(target, dtype=float).reshape(-1)

    def lag(self, q, target):
        """Per joint: how far the jaw is held open of its target, in joint units (positive = squeezing)."""
        if q is None or target is None:
            return None
        return (q - target) * np.sign(np.where(np.abs(self.span) > 1e-9, self.span, 1.0))

    def squeeze_n(self, q, target):
        """Estimated squeeze on a held part (N): the mean over moving jaws of the servo force at the jaw point,
        less the jaw joint's dry friction (a jaw in stiction holds a lag up to friction / kp with nothing between
        the jaws). None when the spec gives no servo gain or force range."""
        lag = self.lag(q, target)
        if lag is None or self.kp is None or self.force_range is None:
            return None
        f = np.minimum(np.maximum(self.kp * lag - self.friction, 0.0), self.force_range)
        return float(np.mean(f / self.lever))

    def squeeze_at_part_n(self, q, target, part_m):
        """Squeeze at a stated part depth from its stall opening and hinge lever."""
        tip_m = self.width_m(q)
        if tip_m <= 0 or part_m <= 0:
            return None
        lag = self.lag(q, target)
        if lag is None or self.kp is None or self.force_range is None:
            return None
        levers = np.array([lever * part_m / tip_m if hinge else lever
                           for lever, hinge in zip(self.lever, self.hinge)])
        force = np.minimum(np.maximum(self.kp * lag - self.friction, 0.0), self.force_range)
        return float(np.mean(force / levers))

    def loaded(self, q, target, min_n):
        """Is the servo pressing the jaws on something: squeeze over min_n, or (no gain in the spec) a lag."""
        s = self.squeeze_n(q, target)
        if s is not None:
            return s > min_n
        lag = self.lag(q, target)
        return lag is not None and bool(np.all(lag >= MIN_LAG_FRAC * np.abs(self.span)))

    def note_width(self, width):
        """Learn the smallest nonzero reading change; zero remains valid."""
        if self.last_width_m is not None:
            step = abs(width - self.last_width_m)
            if step > 0:
                self.step_m = step if self.step_m == 0 else min(self.step_m, step)
        self.last_width_m = self.seen_width_m = width

    def weight_load(self):
        """Per jaw joint, the most the jaw's own weight can load its servo in any arm pose (N on a slide, N m on a
        hinge): the mass the joint carries times gravity, on a hinge times how far that mass sits from the axis.
        From the loaded model; it is what a jaw at rest can lag by with nothing between the jaws, beside its dry
        friction (a hidden level is what the reading shows when the thing it detects is absent)."""
        m, d = self.m, self.d
        self._points(self.closed)
        mujoco.mj_comPos(m, d)
        g = float(np.linalg.norm(m.opt.gravity))
        out = []
        for joint, is_hinge in zip(self.joints, self.hinge):
            j = m.joint(joint).id
            b = int(m.jnt_bodyid[j])
            load = float(m.body_subtreemass[b]) * g
            if is_hinge:
                r = d.subtree_com[b] - d.xanchor[j]
                ax = d.xaxis[j]
                load *= float(np.linalg.norm(r - (r @ ax) * ax))
            out.append(load)
        return np.asarray(out)

    def rest_band_m(self):
        """How far from closed a jaw can rest with nothing between the jaws (m at the tips): its dry friction and
        its own weight over the servo gain, through the hand's geometry. 0.0 without a gain. Source: model."""
        if self.kp is None:
            return 0.0
        rest = self.friction + self.weight_load()
        delta = np.divide(rest, self.kp, out=np.zeros_like(rest), where=self.kp > 0)
        return abs(self.width_m(self.closed + np.sign(self.span) * delta) - self.width_m(self.closed))

    def rest_squeeze_n(self):
        """The squeeze estimate a jaw can show with nothing between the jaws (N at the tips): its own weight on
        the servo (the estimate already takes the dry friction off). 0.0 without a gain. Source: model."""
        if self.kp is None or self.force_range is None:
            return 0.0
        return float(np.max(np.minimum(self.weight_load(), self.force_range) / self.lever))

    def depth_error_m(self, q):
        """Tip opening minus the hinge-side pad opening at the same angle."""
        if not any(self.hinge):
            return 0.0
        self._points(q)
        pts = [self.d.xpos[b] + self.d.xmat[b].reshape(3, 3) @ loc
               for b, loc in zip(self.sides, self.depth_local)]
        gaps = [float(np.linalg.norm(pts[i] - pts[j])) for i in range(len(pts)) for j in range(i + 1, len(pts))]
        return abs(self.width_m(q) - float(np.mean(gaps)))

    def sweep_m(self, part_m):
        """Chord between each hinged tips open and part-width positions."""
        if not any(self.hinge):
            return 0.0
        q0, q1 = self.open, self.q_of_frac(self.frac_for_width(part_m))
        before = self._points(q0)
        after = self._points(q1)
        return max((float(np.linalg.norm(after[i] - before[i]))
                    for i, hinge in enumerate(self.hinge) if hinge), default=0.0)

    def lead_frac(self, force_frac):
        """How far (opening fraction) the close command may lead the jaws for force_frac of the force range;
        None when the spec gives no gain (the close is then bounded only by the servo's own force range)."""
        if self.kp is None or self.force_range is None:
            return None
        lead = float(force_frac) * self.force_range / np.maximum(self.kp, 1e-9)
        return float(np.min(lead / np.maximum(np.abs(self.span), 1e-9)))

    def full_squeeze_n(self):
        """The most the servo can squeeze (N): its force range, or kp times the whole joint span when that is less
        (a position servo commanded fully closed presses only kp times how far the jaw stands open of closed, so on
        a stiff-limited gripper the force range is never reached). None without a force range."""
        if self.force_range is None:
            return None
        top = self.force_range if self.kp is None else np.minimum(self.force_range, self.kp * np.abs(self.span))
        return float(np.mean(np.maximum(top - self.friction, 0.0) / self.lever))

    def describe(self):
        """Report tip opening, servo range and the model interpretation of each reading."""
        return {"jaws": len(self.sides), "moving_jaws": len(self.joints),
                "max_opening_mm": round(self.max_m * MM, 2),
                "full_squeeze_n_est": None if self.full_squeeze_n() is None else round(self.full_squeeze_n(), 2),
                "width_note": "widths are the opening at the jaw tips, from the gripper joints through the model; "
                              "a part held deeper in a hinged jaw reads wider than it is",
                "squeeze_note": "full_squeeze_n_est is the most the gripper servo can press, with the jaws fully "
                                "open; on a narrower part it can be less: each grasp result gives squeeze_n_est"}


@dataclass
class JawFacts:
    """Jaw geometry, measured speed and hidden squeeze, with each values source."""
    step_m: float = 0.0
    closed_gap_m: float = 0.0
    rest_band_m: float = 0.0
    speed_m_s: float | None = None
    squeeze_hidden_n: float = 0.0
    squeeze_noise_n: float = 0.0
    lag: object = None
    sources: dict = field(default_factory=dict)


def jaw_facts(ex):
    """The jaws' own facts in this package's keys (m and N at the jaw tips), from the core's numbers
    (rcpx/control_hold.py numbers: the self-test's steady rates, else the body's description) and the hand's model.
    Nothing is learned inside a task."""
    from rcpx import control_hold

    hand = ex._grasp_hand
    N = control_hold.numbers(ex)
    facts = JawFacts(rest_band_m=hand.rest_band_m())
    facts.step_m = hand.step_m
    facts.closed_gap_m = max(hand.width_m(hand.closed) + facts.rest_band_m,
                             hand.width_m(hand.q_of_frac(N["closed_gap"])))
    facts.speed_m_s = float(N["speed"]) * float(N["gap_per_span_m"])
    facts.squeeze_hidden_n = hand.rest_squeeze_n()
    facts.squeeze_noise_n = 0.0
    facts.lag = None
    facts.sources = {"rest_band_m": "model", "step_m": "self_test", "closed_gap_m": N["source"],
                     "speed_m_s": N["source"], "squeeze_hidden_n": "model", "squeeze_noise_n": "model"}
    return facts


def width_err_m(hand, facts, stated_err_m=0.0):
    """Tip width error from jaw quanta, rest band and hinge depth geometry."""
    q = hand.q_of_frac(hand.frac_for_width(getattr(hand, "seen_width_m", None) or 0.0))
    return R.QUANT * facts.step_m + facts.rest_band_m + stated_err_m + hand.depth_error_m(q)


def release_width_m(hand, facts, B, part_m, sigma_p_m=None):
    """Opening advice from the part error and each jaw's geometric sweep."""
    error = (width_err_m(hand, facts) + B.lag(facts.speed_m_s or 0.0)
             if sigma_p_m is None else sigma_p_m)
    return min(hand.max_m, part_m + len(hand.sides) * error + hand.sweep_m(part_m))
