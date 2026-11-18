# SPDX-License-Identifier: AGPL-3.0-only
"""Model and backend reads, with a duck-typed model path, A1 and A8.1."""

from dataclasses import dataclass, field
from contextlib import contextmanager
from functools import wraps
import inspect
import math
import numpy as np
from rcpx import bounds_laws as laws
from rcpx.bounds_report import MM

FD_STEP = 1e-5  # numeric: finite-difference step of the point Jacobians, rad or m


def pinv(J):
    """Pseudo-inverse of a point Jacobian. A singular value under the finite-difference step's own share of the
    largest one is that step's error, not motion (a joint past the point's body cannot move it), and is cut:
    without the cut the rounding of a zero column answers with joint speeds of 1e8 per m/s."""
    return np.linalg.pinv(np.asarray(J, dtype=float), rcond=FD_STEP)


def quantity(method):
    """Preserve the frozen signature while routing any test override through _q."""
    signature = inspect.signature(method)
    @wraps(method)
    def read(self, *args, **kwargs):
        if method.__name__ in self._override:
            bound = signature.bind(self, *args, **kwargs)
            bound.apply_defaults()
            return self._q(method.__name__, *list(bound.arguments.values())[1:])[0]
        return method(self, *args, **kwargs)
    return read


@contextmanager
def point_model(B, q, point):
    """Use this moving point's Jacobian only while evaluating its bounds."""
    previous = getattr(B, "_point_model", None), getattr(B, "_point_body", None)
    B._point_model, B._point_body = (np.asarray(q), B.robot.point_jacobian(q, point)), point.body
    try:
        yield
    finally:
        B._point_model, B._point_body = previous


@dataclass
class Point:
    name: str
    p: np.ndarray
    radius: float
    tool: bool
    body: str


# The records of A8.1 (rcpx.bounds exports them).
@dataclass
class Thing:
    id: str = None
    point: object = None
    sigma_p: float = None
    extent: object = None
    k_c: float = None
    f_bear: float = None
    f_gave: float = None
    f_moved: float = None
    f_worked: float = None
    end: object = None
    mass_kg: float = None
    touched: bool = False


@dataclass
class Plan:
    path: list = field(default_factory=list)
    guard: object = None
    items: list = field(default_factory=list)
    time_s: float = 0.0
    slowed: int = 0
    free_share: float = 1.0
    refused: object = None
    unseen: str = None
    periods: int = 0
    profile: list = field(default_factory=list)
    kinds: list = field(default_factory=list)
    legs: list = field(default_factory=list)
    looks: list = field(default_factory=list)
    strike: object = None      # the price of the motion (rcpx.bounds_strike.price, contract C2)
    limit: object = None       # the limit somebody stated on the motion as a whole: {limit_n, source, shown}


@dataclass
class GuardSpec:
    force_n: float
    one_reading_n: float
    tau: object
    direction: object
    cap_n: float
    readings: int
    source: str = None         # what the threshold and the cap rest on, in words
    hold: str = None           # None, "first" (a hold whose load is not read yet) or "known"
    expect: object = None      # the expected contact of the leg (contract C9) or None


@dataclass
class ForceSpec:
    first_n: float
    rate_n_s: object
    ceiling_n: float
    items: list
    refused: object = None


@dataclass
class TravelSpec:
    past_m: float
    search_m: float
    backoff_m: float
    items: list


@dataclass
class WaitSpec:
    until: str
    force_n: float
    need_m: float
    t_det: float
    limit_s: float


class Robot:
    def __init__(self, ex):
        self.ex = ex
        self.model = ex.model

    @property
    def n(self):
        return self.model.n

    @property
    def q_lo(self):
        return np.asarray(self.model.q_lo, dtype=float)

    @property
    def q_hi(self):
        return np.asarray(self.model.q_hi, dtype=float)

    @property
    def q_home(self):
        return np.asarray(self.model.q_home, dtype=float)

    @property
    def kp(self):
        return np.asarray(self.model.spec["kp"], dtype=float)

    @property
    def kd(self):
        return np.asarray(self.model.spec["kd"], dtype=float)

    @property
    def tau_limit(self):
        return np.asarray(self.model.torque_limit, dtype=float)

    def speed_limit(self):
        value = self.model.spec.get("speed_limit")
        return (None if value is None else np.asarray(value, dtype=float)), "rating"

    def dt(self):
        for name in ("dt_s", "dt"):
            value = getattr(self.ex.backend, name, None)
            if value is not None and value > 0:
                return float(value)
        raise ValueError("the backend states no control period")

    def g(self):
        if hasattr(self.model, "mj"):
            return float(np.linalg.norm(self.model.mj.opt.gravity))
        return float(self.model.gravity_m_s2)

    def standard_q(self):
        value = self.ex.cfg.get("standard_q")
        return self.q_home if value is None else np.asarray(value, dtype=float)

    def tip(self, q):
        return self.model.fk_tip(q)

    def ik(self, q, p, R):
        return self.model.ik(q, p, R_target=R)

    @property
    def joint_names(self):
        return getattr(self.model, "joint_names", ["joint:" + str(i) for i in range(self.n)])

    def points(self, q):
        out = []
        for body, p0, p1, radius in self.model.collision_capsules(q)[1:]:
            for suffix, p in zip(("start", "end"), (p0, p1)):
                out.append(Point(body + ":" + suffix, np.asarray(p, dtype=float), float(radius),
                                 body in getattr(self.ex, "_tool_bodies", ()), body))
        out.append(Point("tip", np.asarray(self.model.fk_tip(q)[0], dtype=float), 0.0, True, "tip"))
        return out

    def held_points(self, q, held):
        if held is None or held.get("extent_m") is None:
            return []
        p, R = self.model.fk_tip(q)
        return [Point("held:" + str(i), p + R @ np.asarray(c, dtype=float), 0.0, True, "held")
                for i, c in enumerate(held["extent_m"])]

    def point_jacobians(self, q):
        q = np.asarray(q, dtype=float)
        p = np.asarray([v.p for v in self.points(q)])
        J = np.zeros(p.shape + (self.n,))
        for j in range(self.n):
            dq = np.zeros_like(q)
            dq[j] = FD_STEP
            J[..., j] = (np.asarray([v.p for v in self.points(q + dq)]) - p) / FD_STEP
        return J

    def jac_tip(self, q):
        return np.asarray(self.model.jac(q), dtype=float)

    def point_jacobian(self, q, point):
        tip, _ = self.model.fk_tip(q)
        J = self.jac_tip(q)
        if point.tool or point.body in ("tip", "held"):
            return J[:tip.size] + np.cross(J[tip.size:].T, point.p - tip).T
        q = np.asarray(q, dtype=float)
        if not hasattr(self.model, "mj"):
            points = self.points(q)
            index = next((i for i, p in enumerate(points) if p.name == point.name), None)
            return self.point_jacobians(q)[index] if index is not None else J[:tip.size]
        self.model._set(q)
        body = self.model.mj.body(point.body).id
        p = self.model.d.xpos[body].copy()
        R = self.model.d.xmat[body].reshape((p.size, p.size)).copy()
        local = R.T @ (point.p - p)
        out = np.zeros((p.size, q.size))
        for j in range(q.size):
            dq = np.zeros_like(q)
            dq[j] = FD_STEP
            self.model._set(q + dq)
            p1 = self.model.d.xpos[body]
            R1 = self.model.d.xmat[body].reshape((p.size, p.size))
            out[:, j] = (p1 + R1 @ local - point.p) / FD_STEP
        self.model._set(q)
        return out

    def mass_matrix(self, q):
        if hasattr(self.model, "mass_matrix"):
            return np.asarray(self.model.mass_matrix(q), dtype=float)
        if not hasattr(self.model, "mj"):
            raise ValueError("the model states no joint mass matrix")
        import mujoco
        m = self.model.mj
        d = getattr(self.ex, "_dd", None)
        if d is None:
            d = mujoco.MjData(m)
        d.qpos[:] = getattr(self.ex, "_qfull", self.model._qfull)
        qa = getattr(self.ex, "_qadr", self.model._qadr)
        da = getattr(self.ex, "_dadr", self.model._dadr)
        d.qpos[qa] = q
        mujoco.mj_forward(m, d)
        M = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, d, M)
        return M[np.ix_(da, da)]

    def gravity_torque(self, q):
        return np.asarray(self.model.gravity_torque(q), dtype=float)

    def hand_mass(self):
        if hasattr(self.model, "hand_mass_kg"):
            return float(self.model.hand_mass_kg)
        m = self.model.mj
        root = self.model._jbody[-1]
        def descends(b):
            while b and b != root:
                b = int(m.body_parentid[b])
            return b == root
        return sum(float(m.body_mass[b]) for b in range(m.nbody) if descends(b))

    def f_cap(self, q, u):
        return self.model.tip_force_capability(q, u)

    def f_tool_model(self):
        """(N or None, source, how): the tool's rating, only where its maker or owner states one (spec
        tool_rating_n, or a model that states f_tool itself as a rating). A jaw motor's force range is a grip
        force and the wrist's torque over a lever is the arm's own push: neither is a rating."""
        stated = self.model.spec.get("tool_rating_n") if hasattr(self.model, "spec") else None
        if stated is not None:
            return float(stated), "rating", "the tool's stated rating"
        if hasattr(self.model, "f_tool"):
            value = self.model.f_tool()
            force, text = (value[0], str(value[1])) if isinstance(value, tuple) else (value, "the model's tool force")
            if force is not None and "rating" in text:
                return float(force), "rating", text
        return None, "model", "no tool rating is stated (spec tool_rating_n): nothing bounds a force by the tool"

    def w_tool(self):
        surfaces = self.model.spec.get("tool_surfaces") or []
        entries = surfaces.items() if isinstance(surfaces, dict) else ((v.get("name"), v) for v in surfaces)
        for name, surface in entries:
            if name == "tip" and surface.get("size_mm") is not None:
                return float(min(surface["size_mm"]) / MM / laws.TWO)
        return float(getattr(self.model, "w_tool_m", 0.0))

    def e_fit(self):
        """{body: how far its own shape stands out of its capsule}: a fact of the model, worked out once."""
        if getattr(self, "_e_fit", None) is None:
            self._e_fit = self._e_fit_read()
        return self._e_fit

    def _e_fit_read(self):
        if hasattr(self.model, "e_fit"):
            return dict(self.model.e_fit())
        if not hasattr(self.model, "_capsule_defs") or not hasattr(self.model, "_body_points"):
            return {}
        from rcpx.bounds_book import point_segment
        self.model._set(self.q_home)
        return {name: max((max(0, point_segment(p, p0, p1) - r) for p in self.model._body_points(b)), default=0.0)
                for name, b, p0, p1, r in self.model._capsule_defs}

    def _hulls(self):
        """The model's hulls (body frame), read once: they are facts of the model file."""
        if getattr(self, "_hull_cache", None) is None:
            self._hull_cache = self.model.hulls()
        return self._hull_cache

    def _pose_bodies(self, q):
        """Pose the model's bodies at q with the jaws where set_jaws last put them (as collision_capsules does)."""
        geom = getattr(self.model, "_qgeom", None)
        if geom is None:
            self.model._set(q)
            return
        full, self.model._qfull = self.model._qfull, geom
        try:
            self.model._set(q)
        finally:
            self.model._qfull = full

    def hull_points(self, q, body):
        """N x 3 world points of that body's own collision shape at q; None for a model that has none."""
        if not hasattr(self.model, "hulls") and not hasattr(self.model, "_body_points"):
            return None
        if not hasattr(self.model, "mj"):
            hull = self._hulls().get(body)
            return None if hull is None else np.asarray(hull.get("points_m") if isinstance(hull, dict) else hull, dtype=float)
        if hasattr(self.model, "hulls"):
            entry = self._hulls().get(body)
            if entry is None:
                return None
            pts = np.asarray(entry["points_m"], dtype=float)
            self._pose_bodies(q)
            b = self.model.mj.body(body).id
        else:
            self._pose_bodies(q)
            b = self.model.mj.body(body).id
            pts = np.asarray(self.model._body_points(b), dtype=float)
        p = self.model.d.xpos[b]
        R = self.model.d.xmat[b].reshape((p.size, p.size))
        return pts @ R.T + p

    def contact_model(self):
        value = getattr(self.ex.backend, "contact_model", None)
        return value() if callable(value) else value

    def servo_law(self):
        value = getattr(self.ex.backend, "servo_law", None)
        return value() if callable(value) else value

    def joint_resolution(self):
        spec = self.model.spec
        if spec.get("resolution_rad") is not None:
            return np.broadcast_to(spec["resolution_rad"], self.q_home.shape).copy(), "stated"
        if spec.get("resolution_deg") is not None:
            return np.broadcast_to(np.deg2rad(spec["resolution_deg"]), self.q_home.shape).copy(), "stated"
        return None, "model"


class ModelKnow:
    """Until-known answers from robot facts, never commissioning a place, A1."""
    def __init__(self, robot):
        self.robot = robot
        self.ready = False
        self.values = {}
        self._stops = []

    def add_stop(self, stop, where="task", at=None, store=None):
        """Keep a stop measured by rcpx.bounds_stops.measure (contract C5); a false stop is never handed here.
        store is SelfKnow's argument: nothing here is ever written to a file."""
        if not isinstance(stop, dict) or "det_periods" not in stop:
            return False
        self._stops.append(dict(stop, where=stop.get("where", where), at=stop.get("at", at)))
        return True

    def stops(self):
        return [dict(s) for s in self._stops]

    def _servo_tau(self):
        """The joint servo's own time constant (s): the largest kd over kp. It is the model of the tracking lag
        until a free line was measured; it is no part of a detection time."""
        r = self.robot
        return max((d / p for d, p in zip(r.kd, r.kp) if p > 0), default=0.0)

    def set(self, name, value, source="self_test"):
        self.values[name] = value, source

    def get(self, name, *args):
        if name in self.values:
            value, source = self.values[name]
            return (value(*args) if callable(value) else value), source, "measured self-test"
        r = self.robot
        source, evidence = "model", "from the body model"
        if name == "qd_max":
            value, source = r.speed_limit()
            evidence = "the maker's rated joint speed"
            if value is None or not np.all(np.isfinite(value)):
                # a joint with no rating: the fastest it can still stop from inside its own range (Q1)
                sized = laws.qd_until_known(self.get("qdd_max", r.q_home)[0], r.q_hi - r.q_lo)
                value = sized if value is None else np.where(np.isfinite(value), value, sized)
                source = "model"
                evidence = "no rating and no sweep yet: sized for the self-test's own passes only"
        elif name == "qdd_max":
            q = args[0] if args else r.q_home
            value = laws.qdd_model(r.tau_limit, r.gravity_torque(q), np.diag(r.mass_matrix(q)))
        elif name == "tau_max":
            value, source, evidence = r.tau_limit, "rating", "the model file's force range"
        elif name == "f_tool":
            return r.f_tool_model()
        elif name in ("quantum", "joint_quantum"):
            value, source = r.joint_resolution()
            if value is None:
                value, evidence = np.zeros_like(r.q_home), "no resolution stated and none measured yet"
        elif name == "tau_hidden":
            # a level nobody measured is not zero: what the estimate can hide is not known, so no contact
            # threshold stands on it (the guard then runs on its cap) until a still hold or a free leg measures it
            value, evidence = np.full_like(r.q_home, math.inf), "not known: not measured yet, no threshold stands on it"
        elif name == "f_hidden":
            value, evidence = math.inf, "not known: not measured yet, no threshold stands on it"
        elif name == "sigma_tau":
            value, evidence = np.zeros_like(r.q_home), "not measured yet: the self-test has not run"
        elif name in ("jitter", "sigma_f", "e_tau"):
            value, evidence = 0.0, "not measured yet: the self-test has not run"
        elif name == "rise":
            value, evidence = 0.0, "no rise term: a detection time is the guard's readings, then what the stops show"
        elif name == "t_det":
            from rcpx import bounds_stops
            value, evidence, _ = bounds_stops.t_det(self._stops, r.dt(), *args[:1])
        elif name in ("lag", "e_cross"):
            value = args[0] * self._servo_tau()
            evidence = "servo lag until a free line was measured"
        elif name == "t_settle":
            diag = np.diag(r.mass_matrix(r.q_home))
            value = max(((m / d if d > 0 else 0) + (d / p if p > 0 else 0) for m, d, p in zip(diag, r.kd, r.kp)), default=0.0)
            evidence = "one time constant of each stage of the joint servo"
        elif name == "a_brake":
            value = None
        elif name in ("dt", "t_sample"):
            value, evidence = r.dt(), "the backend's control period"
        elif name == "slope":
            from rcpx import bounds_stops
            got = bounds_stops.fit(self._stops)
            value = dict(got, c=max(1.0, got["c"] or 1.0), c2=None, kind="fitted" if got["n"] else "model")
            source = "contact" if got["n"] else "model"
            evidence = f"{got['seen']} resolved stops of this session"
        else:
            value, evidence = None, "not known"
        return value, source, evidence

    def note_false_stop(self, stop):
        """A stop no thing made: the level it showed stands for this session (never in a stored file)."""
        old = self.get("f_hidden", stop.get("v", 0.0), 0.0, stop.get("held"))[0]
        if old is not None and math.isfinite(old):
            self.set("f_hidden", max(old, stop["peak_n"]), "self_test")

    def refit(self, stop):
        """Contract C5: a stop measured by rcpx.bounds_stops.measure is kept, and the slope is read from the kept
        stops (bounds_stops.fit). Any other record changes nothing: a peak made of a deceleration and the speed of
        the reading that tripped are no measurement. Returns {"stored", "n": resolved stops kept, "c"}."""
        from rcpx import bounds_stops
        stored = self.add_stop(stop)
        got = bounds_stops.fit(self._stops)
        return {"stored": stored, "n": got["seen"], "c": got["c"]}


def model_hidden(B, tau_hidden=None):
    """What the torque model's own error hides (A1.1: a hidden level is what the model and the arithmetic leave),
    from the measured share e_tau. With tau_hidden: per joint, the larger of the stored level and e_tau of that
    joint's rating. Without: at the tool, e_tau of the force the arm can give at its pose along its travel.
    Zero until e_tau was measured."""
    try:
        share = B._value("e_tau") or 0.0
        if tau_hidden is not None:
            hidden = np.asarray(tau_hidden, dtype=float)
            return np.maximum(hidden, share * np.asarray(B.tau_max(), dtype=float)) if share else hidden
        if not share:
            return 0.0
        q, u = B._pose(getattr(B.ex, "_q_des", None), getattr(B.ex, "_travel_direction", None))
        cap = B.robot.f_cap(q, u)
        return float(share * cap) if cap is not None and math.isfinite(cap) else 0.0
    except Exception:
        return np.asarray(tau_hidden, dtype=float) if tau_hidden is not None else 0.0


def note_joint_hidden(know, v, ext_peak):
    """After a false stop: the joint torque levels that tripped it rise for this session (never in the stored
    file: a hidden level is stored only from readings where no contact is proven)."""
    note = getattr(know, "note_session", None)
    if ext_peak is None or note is None:
        return
    try:
        note("tau_hidden", [v], np.abs(np.asarray(ext_peak, dtype=float)).tolist())
    except Exception:
        pass
