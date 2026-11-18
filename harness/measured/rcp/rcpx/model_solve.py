# SPDX-License-Identifier: AGPL-3.0-only
"""Whole-posture solves for the RobotModel (rcpx/model.py), for primitives that turn or place the hand and need to say
what the arm CAN do when the straight way stops.

postures(model, q0, p, R_target=None, keep_axis=None, ...): every distinct arm posture (from the present pose, home
and seeded starts) that puts the tip at p with the hand held at R_target (or the tool axis along keep_axis), inside
the joint limits and clear of the arm itself, best first: the largest margin to the nearest joint limit, then the
smallest joint change from q0. A primitive that turns the hand step by step (orient_tool) can stop on a joint limit
while the arm has other postures for the same hand pose (a seven joint arm, or a wrist that can flip): this is
where it finds them, and the q_deg it can offer as a joint move.

axis_options(model, q0, p, want): for an arm that cannot point its tool axis everywhere (a five joint
arm points its finger only in its own vertical plane), the nearest direction it can hold at p, how far that is from
the wanted one, the pose, and which tool surface then leads along the wanted direction: so reach_check can answer
"the tip leads with the finger 40 deg off the approach" instead of a posture whose jaw meets the part first.

No margin here is a typed number (SPEC-R4): motion_of states what the model knows of its own motion until the robot
has measured it; limit_stop is how near a limit a joint can no longer be kept off it (the servo's deflection at its
torque rating, plus the joint's stopping angle at the speed the next move uses); self_margin is the error of the
distance between two links (zero on the model's own collision geometry, plus what tracking lag moves a link at
speed); the tilt up to which axis_options answers is the tool's own (model_tool stands_out).

Nothing here names a robot; everything comes from the loaded model. Deterministic (seeded starts).
"""

import math

import mujoco
import numpy as np

SAME_POSTURE_RAD = 0.05   # two solutions whose joints all differ by less than this are one posture

IN_USE = [None]  # the RobotModel that last posed itself: what a number that used to be a constant here reads


class OfModel:
    """A number that used to be a typed constant and is now the loaded model's own (SPEC-R4: no limit is typed).
    Modules that still import the old name get this: it reads, where it is used in arithmetic or a comparison, as
    the value of the model that last posed itself (the one whose Jacobian the caller has just taken)."""

    __array_ufunc__ = None   # numpy leaves the arithmetic to this class

    def __init__(self, read):
        self._read = read

    def __float__(self):
        if IN_USE[0] is None:
            raise RuntimeError("no robot model is loaded yet: this number is the model's own")
        return float(self._read(IN_USE[0]))

    def __mul__(self, other):
        return float(self) * other

    __rmul__ = __mul__

    def __truediv__(self, other):
        return float(self) / other

    def __rtruediv__(self, other):
        return other / float(self)

    def __lt__(self, other):
        return float(self) < other

    def __le__(self, other):
        return float(self) <= other

    def __gt__(self, other):
        return float(self) > other

    def __ge__(self, other):
        return float(self) >= other



# the nearest any joint may stand to a limit in a posture at rest (an older name some callers still import)
MIN_LIMIT_MARGIN_RAD = OfModel(lambda model: float(np.min(limit_stop(model))))


def motion_of(model, qd=None, acc=None, lag_s=None):
    """{"qd", "qd_known", "acc", "lag_s", "lever_m", "source"}: per joint, what bounds the joint's own motion (qd_known
    is qd where a maker rated it or the robot measured it, and zero where it is only the until-known rule, which
    sizes the self-test's own passes and nothing else). Until a session
    gives measured values: acc = (torque limit - gravity torque) / the joint's own inertia at home (rotor included);
    qd = the maker's rated speed, and for a joint with none the fastest it could still stop from inside its range,
    sqrt(2 acc range to the nearer limit from home); lag_s = kd / kp, the servo's own time constant. lever_m: how
    far from the joint the farthest thing it carries stands at home."""
    n = model.n
    g = np.abs(model.gravity_torque(model.q_home))   # leaves the model's data forwarded at home
    M = np.zeros((model.mj.nv, model.mj.nv))
    mujoco.mj_fullM(model.mj, model.d, M)
    room = np.maximum(model.torque_limit - g, 0.0)
    own_acc = room / np.diag(M)[model._dadr]
    acc = _known(acc, own_acc)
    span = np.minimum(model.q_home - model.q_lo, model.q_hi - model.q_home)
    reach = np.sqrt(np.maximum(2 * acc * span, 0.0))   # v^2 = 2 a s: the speed a joint can stop from within s
    rated = [np.nan if v is None else v for v in model.speed_limit]
    own_qd = np.where(np.isnan(rated), reach, rated)
    got = _known(qd, np.asarray(rated, dtype=float))   # a speed a maker rated or the robot measured, else nan
    qd = _known(qd, own_qd)
    kp, kd = (np.asarray(model.spec[k], dtype=float) for k in ("kp", "kd"))
    lag_s = _known(lag_s, kd / kp)
    frames = model.joint_frames(model.q_home)
    ends = [p for _, p0, p1, _ in model.collision_capsules(model.q_home)[1:] for p in (p0, p1)]
    ends.append(model.fk_tip(model.q_home)[0])
    keys = [model._arm_key(model.mj.body(name).id) for name, _, _, _ in model.collision_capsules(model.q_home)[1:]]
    keys = [k for k in keys for _ in (0, 1)] + [n - 1]
    lever = np.array([max([float(np.linalg.norm(p - frames[j][0])) for p, k in zip(ends, keys) if k >= j] or [0.0])
                      for j in range(n)])
    return {"qd": qd, "qd_known": np.where(np.isnan(got), 0.0, got), "acc": acc, "lag_s": lag_s, "lever_m": lever,
            "source": ["rating" if v is not None else "model" for v in model.speed_limit]}


def _known(given, own):
    """Per joint: the given value where one is given (not None, not nan), else the model's own."""
    if given is None:
        return own
    got = np.array([np.nan if v is None else float(v) for v in given])
    return np.where(np.isnan(got), own, got)


def joint_inertia(model, q):
    """Each arm joint's own inertia (kg m^2, rotor included) at pose q: the diagonal of the joint-space mass matrix."""
    model.gravity_torque(q)   # leaves the model's data forwarded at q
    M = np.zeros((model.mj.nv, model.mj.nv))
    mujoco.mj_fullM(model.mj, model.d, M)
    return np.diag(M)[model._dadr]


def limit_stop(model, qd=None):
    """Per joint (rad): how near a limit the joint can no longer be kept off it. At rest: the deflection of its
    servo at its torque rating (torque limit / kp), which is how far a load inside the rating can push it. Moving
    at joint speeds qd (rad/s): that plus its stopping angle, qd^2 / (2 acc) + qd lag (the model's own values, or
    the session's measured ones: RobotModel.set_motion)."""
    hold = model.torque_limit / np.asarray(model.spec["kp"], dtype=float)
    if qd is None:
        return hold
    mo = model.motion
    qd = np.abs(np.asarray(qd, dtype=float))
    with np.errstate(divide="ignore", invalid="ignore"):
        brake = np.where(mo["acc"] > 0, qd ** 2 / (2 * mo["acc"]), np.where(qd > 0, np.inf, 0.0))
    return hold + brake + qd * mo["lag_s"]


def self_margin(model, qd=None):
    """How far apart (m) two links must stay. The distance is measured on the model's own collision geometry, as
    the physics collides it, so its fit error is zero; at joint speeds qd each joint's tracking lag (qd lag_s) moves
    what it carries by up to that angle times its lever, and the sum over the joints is the margin."""
    if qd is None:
        return 0.0
    mo = model.motion
    return float(np.sum(np.abs(np.asarray(qd, dtype=float)) * mo["lag_s"] * mo["lever_m"]))


def limit_margin(model, q):
    """Smallest distance (rad) from any joint to its nearer limit."""
    q = np.asarray(q, float)
    return float(np.min(np.minimum(q - model.q_lo, model.q_hi - q)))


def postures(model, q0, p, R_target=None, keep_axis=None, starts=24, seed=0, rot_tol=None, keep=4, qd=None):
    """Up to `keep` distinct postures (q, limit margin rad, largest joint change rad from q0), best first; [] when
    none. A posture with a joint nearer a limit than limit_stop(model, qd) is left out (no way round a limit), and
    one whose links are nearer than self_margin(model, qd); qd: the joint speeds the next move will use (None: the
    posture is judged at rest). rot_tol (rad): the orientation error a solve may end with (default: the solver's
    own)."""
    need, apart = limit_stop(model, qd), self_margin(model, qd)
    p = np.asarray(p, float)
    q0 = model.clip(q0)
    rng = np.random.default_rng(seed)
    seeds = [q0, model.clip(model.q_home)] + [rng.uniform(model.q_lo, model.q_hi) for _ in range(starts)]
    found = []
    for k, qs in enumerate(seeds):
        q, ok, _ = model.ik(qs, p, R_target=R_target, keep_axis=keep_axis, restarts=0, seed=k, rot_tol=rot_tol)
        if not ok or not model.within_limits(q) or model.self_collisions(q, apart):
            continue
        if np.any(np.minimum(q - model.q_lo, model.q_hi - q) < need):
            continue
        if any(float(np.max(np.abs(q - f[0]))) < SAME_POSTURE_RAD for f in found):
            continue
        found.append((q, limit_margin(model, q), float(np.max(np.abs(q - q0)))))
    found.sort(key=lambda f: (-round(f[1], 3), f[2]))
    return found[:keep]


def leading_surface(model, q, direction):
    """Which named tool surface (robot spec tool_surfaces, the tip included) stands foremost along `direction` at
    pose q, and by how much (mm) it leads the tip (0 when the tip leads)."""
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    tip = model.fk_tip(q)[0]
    best = ("tip", 0.0)
    for surf in model.spec.get("tool_surfaces", []):
        if surf["name"] == "tip":
            continue
        c = model.d.site_xpos[model.mj.site("surface_" + surf["name"]).id]
        lead = float((c - tip) @ d) * 1000.0
        if lead > best[1]:
            best = (surf["name"], lead)
    return best[0], round(best[1], 1)


AXIS_STARTS = 24          # position-only solves from seeded starts: the postures the arm has at the point
AXIS_REFINE = (0.0, 0.25, 0.5, 0.75)   # then from the nearest one, the axis asked this far back toward it


def _angle(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(a, b))))))


def lead_tilt_deg(model):
    """The tilt (deg) of the tool off its approach beyond which a part other than the tip leads whichever way it
    leans: the largest of the per-side tilts model_tool.tool_numbers works out (a side nothing stands out on never
    leads: a quarter turn)."""
    from rcpx.model_tool import tool_numbers

    N = tool_numbers(model)
    sides = [math.degrees(math.pi / 2) if s is None else float(s["tilt_deg"])
             for s in (N["stands_out"].values() if N else [None])]
    return max(sides)


def axis_options(model, q0, p, want, max_off_deg=None, axis="x", seed=0):
    """The nearest tool-axis direction the arm can hold with the tip at p: {q, held, off_deg, leading,
    leading_by_mm}, or None when no posture within max_off_deg exists (default: lead_tilt_deg(model), the tilt
    past which another tool part leads on every side). The postures the arm has at p (position-only
    solves from q0, home and seeded starts, inside the limits and clear of itself) give the nearest held axis; then
    the solve asks the wanted axis itself and, failing that, axes part of the way back toward the nearest held one,
    so the answer is the arm's own nearest pose. leading names the tool surface that stands foremost along `want`
    there (the tip, or a jaw or flange that would meet the part first)."""
    want = np.asarray(want, float)
    want = want / np.linalg.norm(want)
    max_off_deg = lead_tilt_deg(model) if max_off_deg is None else float(max_off_deg)
    apart = self_margin(model)
    idx = "xyz".index(axis)
    p = np.asarray(p, float)
    rng = np.random.default_rng(seed)
    starts = [model.clip(q0), model.clip(model.q_home)] + [rng.uniform(model.q_lo, model.q_hi)
                                                           for _ in range(AXIS_STARTS)]
    best = None
    for k, qs in enumerate(starts):
        q, ok, _ = model.ik(qs, p, restarts=0, seed=k)
        if not ok or not model.within_limits(q) or model.self_collisions(q, apart):
            continue
        off = _angle(model.fk_tip(q)[1][:, idx], want)
        if best is None or off < best[1]:
            best = (q, off)
    if best is None:
        return None
    held0 = model.fk_tip(best[0])[1][:, idx]
    for f in AXIS_REFINE:
        t = (1.0 - f) * want + f * held0
        t = t / np.linalg.norm(t)
        q, ok, _ = model.ik(best[0], p, keep_axis=(axis, t), restarts=0)
        if ok and model.within_limits(q) and not model.self_collisions(q, apart):
            off = _angle(model.fk_tip(q)[1][:, idx], want)
            if off < best[1]:
                best = (q, off)
            break
    if best[1] > max_off_deg:
        return None
    q = best[0]
    lead, by = leading_surface(model, q, want)
    return {"q": q, "held": model.fk_tip(q)[1][:, idx], "off_deg": round(best[1], 2), "leading": lead,
            "leading_by_mm": by}
