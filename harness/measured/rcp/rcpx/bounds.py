# SPDX-License-Identifier: AGPL-3.0-only
"""The SI bounds interface, with session-only thing evidence. Speed is the
body's. Nothing here lowers a motion but a stated limit (stated_limit and arrival, rcpx/bounds_limit.py); doubt
is priced on the plan, never slowed or refused for."""

import importlib
import math
import numpy as np
from rcpx import bounds_know_point as know_point
from rcpx import bounds_know_solve as solve
from rcpx import bounds_laws as laws
from rcpx import bounds_limit as limit
from rcpx import bounds_report as report
from rcpx import bounds_stops
from rcpx import bounds_strike
from rcpx.bounds_ratios import QUANT, REL
from rcpx.bounds_robot import Robot, ModelKnow, pinv, quantity, model_hidden
from rcpx.bounds_robot import Thing, Plan, GuardSpec, ForceSpec, TravelSpec, WaitSpec  # noqa: F401
from rcpx.bounds_book import note_give, note_stop, point_clearance, segment_clearance, thing_of, thing_sheet
from rcpx.bounds_space import SpaceBook
from rcpx.context_base import refusal


def of(ex):
    if not isinstance(getattr(ex, "bounds", None), Bounds):
        ex.bounds = Bounds(ex)
    return ex.bounds


class Bounds:
    def __init__(self, ex):
        self.ex, self.robot = ex, Robot(ex)
        self.site = ex.cfg.get("site") or None
        self.notes, self._override, self._sources, self._mods = [], {}, {}, {}
        self._records, self.stops, self.gives = {}, [], []
        self.false_stops, self._word_info = 0, None
        self._solving = None      # what the arrivals being solved keep: "limit", "ends" or None (see slope)
        self.model_know = ModelKnow(self.robot)
        self.know = self.model_know
        self._load_know()
        self.space = SpaceBook(ex, self.robot)
        self.space._bounds = self

    def _load_know(self):
        """The stored self-knowledge of this robot, read at session open with no motion; else an empty one."""
        try:
            know = importlib.import_module("rcpx.bounds_self").of(self.ex)
            if know is not None:
                self.know = know
        except Exception as error:
            self._note("bounds_self gave no SelfKnow for this robot (" + type(error).__name__ + "); the model's rules stand in")

    def _note(self, text):
        self.notes += [] if text in self.notes else [text]

    def _optional(self, name):
        """A module of another package that may not be merged yet (asked once)."""
        if name not in self._mods:
            try:
                self._mods[name] = importlib.import_module("rcpx." + name)
            except ImportError:
                self._note(name + " absent; local fallback stands in")
                self._mods[name] = None
        return self._mods[name]

    def _q(self, name, *args):
        if name in self._override:
            value = self._override[name]
            answer = value(*args) if callable(value) else value
            result = answer, "stated", "set by a test"
        else:
            result = None
            try:
                getter = getattr(self.know, "get", None)
                actual_name = "quantum" if name == "joint_quantum" else name
                ask = (lambda *a: getter(actual_name, *a)) if callable(getter) else getattr(self.know, actual_name)
                try:
                    answer = ask(*args)
                except TypeError:
                    answer = ask()   # a quantity the self-knowledge keeps without a pose or a speed
                if isinstance(answer, tuple) and len(answer) > 1 and (answer[0] is None or isinstance(answer[1], str)):
                    value, source, *extra = answer
                    words = [w for w in report.SOURCES if w in str(source)]
                    source = "self_test" if not words else ("self_test" if "self_test" in words else words[0])
                    stamp = extra[0] if extra and extra[0] is not None else None
                    if isinstance(getattr(self.know, "_base", self.know), ModelKnow):
                        evidence = report.WORDS[source] if stamp is None else str(stamp)
                    else:
                        evidence = report.WORDS[source] + ("" if stamp is None else ", stored " + str(stamp))
                else:
                    value, source, evidence = answer, "self_test", report.WORDS["self_test"]
                if value is not None:
                    result = value, source, evidence
            except Exception:
                pass
            if result is None:
                result = self.model_know.get(name, *args)
        self._sources[name] = result[1:]
        return result

    def _value(self, name, *args):
        return self._q(name, *args)[0]

    def _derived(self, name, value, source="model", evidence="derived from body facts"):
        self._sources[name] = source, evidence
        return value

    def _pose(self, q, u=None):
        return (self.robot.standard_q() if q is None else np.asarray(q, dtype=float),
                np.asarray((0, 0, -1) if u is None else u, dtype=float))

    def _jac(self, q, u):
        point = getattr(self, "_point_model", None)
        if point is not None and np.array_equal(q, point[0]):
            return point[1]
        return self.robot.jac_tip(q)[:len(u)]

    def src(self, name):
        return self._sources.get(name, ("model", "not read yet"))

    def dt(self):
        return self._derived("dt", self.robot.dt(), "model", "the backend's control period")

    def qd_max(self):
        value, source, evidence = self._q("qd_max")
        value = np.asarray([np.nan if v is None else v for v in np.asarray(value, dtype=object).reshape(-1)], dtype=float)
        if not np.all(np.isfinite(value)):
            sized, source, evidence = self.model_know.get("qd_max")
            value = np.where(np.isfinite(value), value, sized)
            self._sources["qd_max"] = source, evidence
        return value

    def qdd_max(self, q):
        """Per joint the lesser of the model's acceleration (the torque limit over the inertia) and what a
        self-test pass showed (accel_j); where no pass ran, the model's. Never a stored share of it."""
        value, source, evidence = self.model_know.get("qdd_max", self._pose(q)[0])
        value = np.asarray(value, dtype=float)
        shown = self._shown("accel_j", value.shape)
        if shown is not None and np.any(shown < value):
            value, source, evidence = np.minimum(value, shown), "self_test", "the largest acceleration a self-test pass ended still with"
        return self._derived("qdd_max", value, source, evidence)

    def _shown(self, name, shape):
        """A per joint entry of the self-knowledge (brake_j, accel_j) as an array, inf where no pass ran; None
        when the file has none."""
        try:
            stored = self._value(name)
        except Exception:
            stored = None
        if stored is None:
            return None
        out = np.asarray([np.inf if v is None else float(v) for v in np.asarray(stored, dtype=object).reshape(-1)])
        return out if out.shape == tuple(shape) else None

    def tau_max(self):
        return np.asarray(self._value("tau_max"), dtype=float)

    def joint_quantum(self):
        return np.asarray(self._value("joint_quantum"), dtype=float)

    def quantum(self, q, u):
        if "quantum" in self._override and callable(self._override["quantum"]):
            return self._value("quantum", q, u)
        q, u = self._pose(q, u)
        value = float(np.linalg.norm((self._jac(q, u).T @ u) * self.joint_quantum()))
        return self._derived("quantum", value, *self.src("joint_quantum"))

    def rigid(self, q, u, force_n=None):
        """The tip travel the readings cannot tell from none (m): resolution, jitter and the servo's give at force_n;
        with none, at the still arm's own level (bounds_know_solve.rest: no false stop of this session raises it)."""
        if "rigid" in self._override:
            return self._value("rigid", q, u, force_n)
        q, u = self._pose(q, u)
        force_n = solve.rest(self) if force_n is None else force_n
        return self._derived("rigid", laws.rigid(self.quantum(q, u), self._value("jitter"), force_n, self.k_servo(q, u)))

    def f_hidden(self, v=0.0, a=0.0, held=None):
        return self._value("f_hidden", v, a, held)

    def sigma_f(self, v=0.0, a=0.0, held=None):
        return self._value("sigma_f", v, a, held)

    def f_thr(self, v=0.0, a=0.0, readings=2, held=None):
        if "f_thr" in self._override:
            return self._value("f_thr", v, a, readings, held)
        hidden, source = self.f_hidden(v, a, held), self.src("f_hidden")
        value = laws.f_thr(max(hidden, model_hidden(self)), self.sigma_f(v, a, held), readings)
        if not value > 0:   # a level of exactly zero was never measured: the guard runs on its cap, so every reader
            return self._derived("f_thr", math.inf, "model", "not known: no level over zero measured yet")   # is told inf
        return self._derived("f_thr", value, *source)

    def tau_thr(self, v=0.0):
        hidden, source = model_hidden(self, self._value("tau_hidden", v)), self.src("tau_hidden")
        return self._derived("tau_thr", laws.tau_thr(hidden, self._value("sigma_tau", v)), *source)

    def _stops(self):    # the stops this robot keeps: the self-knowledge's own list, else the session's
        kept = getattr(self.know, "stops", None)
        return list(kept() if callable(kept) else self.stops)

    def t_det(self, readings=2):
        """The detection time: the largest over this robot's stops; with none, the guard's own readings."""
        seconds, text, n = bounds_stops.t_det(self._stops(), self.dt(), readings)
        return self._derived("t_det", seconds, "contact" if n else "model", text)

    def lag(self, v):
        return self._value("lag", v)

    def e_cross(self, v):
        return self._value("e_cross", v)

    def t_settle(self):
        return self._value("t_settle")

    def a_brake(self, q, u):
        """Braking along u from the joints that carry the motion: the model's (the torque left after gravity
        over the directional inertia), and per joint no more than a self-test pass showed (brake_j). The stored
        scalar of round four (the least lever times braking over all joints) is never read."""
        q, u = self._pose(q, u)
        joints = pinv(self._jac(q, u)) @ u
        value = laws.a_brake_model(self.tau_max(), self.robot.gravity_torque(q), self.robot.mass_matrix(q) @ joints)
        shown = self._shown("brake_j", joints.shape)
        measured = math.inf if shown is None else laws.v_free(shown, joints)
        if measured < value:
            return self._derived("a_brake", measured, "self_test", "the largest braking a self-test pass ended still with, along this travel")
        return self._derived("a_brake", value, "model", "the torque left after gravity over the inertia along this travel")

    def d_stop(self, v, q=None, u=None, readings=2):
        q, u = self._pose(q, u)
        return self._derived("d_stop", laws.d_stop(v, self.t_det(readings), self.a_brake(q, u)))

    def v_free(self, q, u):
        q, u = self._pose(q, u)
        value = laws.v_free(self.qd_max(), pinv(self._jac(q, u)) @ u)
        return self._derived("v_free", value, *self.src("qd_max"))

    def w_free(self, q, axis):
        q, axis = self._pose(q, axis)
        value = laws.v_free(self.qd_max(), pinv(self.robot.jac_tip(q)[len(axis):]) @ axis)
        return self._derived("w_free", value, *self.src("qd_max"))

    def f_cap(self, q, u):
        q, u = self._pose(q, u)
        if getattr(self, "_point_model", None) is not None:
            margin = np.maximum(0, self.tau_max() - np.abs(self.robot.gravity_torque(q)))
            cap = self.robot.f_cap(q, u)
            value = laws.v_free(margin, self._jac(q, u).T @ u)
            return self._derived("f_cap", value if cap is None else min(value, cap))
        return self._value("f_cap", q, u) if "f_cap" in self._override else self._derived("f_cap", self.robot.f_cap(q, u))

    def f_tool(self):
        """The tool's stated contact rating (spec tool_rating_n), else None: a grip force is no rating."""
        rated = self.robot.model.spec.get("tool_rating_n")
        if rated is None:
            return self._derived("f_tool", None, "model", "no contact rating is stated for the tool")
        return self._derived("f_tool", float(rated), "rating", "the tool's stated contact rating")

    def f_hand(self, held_n=0.0):
        return self._derived("f_hand", laws.f_hand(self.robot.hand_mass(), self.robot.g(), held_n))

    def e_tau(self):
        return self._value("e_tau")

    def m_eff(self, q, u, held_kg=0.0, part_kg=0.0):
        q, u = self._pose(q, u)
        J = self._jac(q, u)
        tip_J = self.robot.jac_tip(q)[:u.size]
        M = self.robot.mass_matrix(q) + held_kg * (tip_J.T @ tip_J)
        return self._derived("m_eff", laws.m_eff(J.T @ u, np.linalg.pinv(M)) + part_kg)

    def k_servo(self, q, u):
        q, u = self._pose(q, u)
        servo = self.robot.servo_law()
        kp = servo.get("kp", self.robot.kp) if servo and servo.get("kind") == "joint_pd" else self.robot.kp
        return self._derived("k_servo", laws.k_servo(self._jac(q, u).T @ u, kp))

    def _contact_rate(self):
        """The contact rate the backend states (A8.4): of the moving point's own body where it gives one per body,
        else of the tool."""
        model = self.robot.contact_model()
        if not model:
            return None
        body = (model.get("by_body") or {}).get(getattr(self, "_point_body", None))
        b = model.get("b_per_s") if body is None else body
        return max(b.values(), default=None) if isinstance(b, dict) else b

    def model_z(self, q, u, thing=None, part_kg=0.0):
        """The body's model of the stop slope, N per (m/s) (rcpx/bounds_know_point.py): the contact
        rate times the mass that moves along the travel at the point (held fast), plus the servo's stiffness there
        times (the detection time plus the lag); a link's point (bounds_robot.point_model) gives its own, by the
        energy bound at the leg's joint step where one is handed (bounds_limit.moving_point). None with no contact
        model. It carries "said" (bounds_stops.Slope): the slope a figure is stated with here and the range the
        stops bear it in, the last arrival's figures ("before") while the hand stands where that stop left it,
        and at the tool "moving": every moving body's energy bound for a stop's joint speeds."""
        q, u = self._pose(q, u)
        body = getattr(self, "_point_body", None) if getattr(self, "_point_model", None) is not None else None
        got = know_point.terms(self, q, np.asarray(self._jac(q, u), dtype=float), u, body,
                               qd=None if body is None else getattr(self, "_point_qd", None))
        E = self.thing(thing) if thing is not None else None
        if got["impact"] is None or got["m_kg"] is None or got["k_n_m"] is None:
            return None
        k = laws.series(got["k_n_m"], None if E is None else E.k_c)
        if not math.isfinite(k):
            return None
        z = float(got["b_per_s"]) * (got["m_kg"] + part_kg) + k * got["lead_s"]
        fit = bounds_stops.of_kind(bounds_stops.fit(self._stops()), body is not None)
        said = bounds_stops.said_of(*bounds_stops.slope(z, fit), fit, q, u)
        last = getattr(self, "_arrived", None)
        if last is not None and body is None:
            moved = float(np.linalg.norm(self.robot.tip(q)[0] - last["tip"]))
            if moved <= QUANT * last["v"] * self.dt():       # the hand stands where that stop left it
                said["before"] = last["ran"]
        return bounds_stops.Slope(z, said=said, **({} if body is not None else {"moving": bounds_strike.moving_at(self, q)}))

    def slope(self, q, u, thing=None, part_kg=0.0):
        """(the stop slope N per (m/s) for SOLVING a speed from a force, what it rests on); (None,
        why) with no contact model. Under a stated limit, and for a link's point: bounds_stops.slope, the slope
        the motion's figure is stated with (never under the body's model). Else (the force a touch, a close, a
        landing or a press ends at; a caller outside a plan): bounds_stops.estimate, what this body's stops
        showed. rcpx/bounds_know_solve.py says who sets which (self._solving). Both carry bounds_stops.solving."""
        z_model = self.model_z(q, u, thing, part_kg)
        if z_model is None:
            return self._derived("slope", (None, "unresolved: the backend states no contact model"))
        link = getattr(self, "_point_model", None) is not None and getattr(self, "_point_body", None) is not None
        whole = bounds_stops.fit(self._stops())
        fit = bounds_stops.of_kind(whole, True) if link and (whole.get("link") or {}).get("n") else whole
        stated = link or self._solving == "limit"
        z, text = (bounds_stops.slope if stated else bounds_stops.estimate)(float(z_model), fit)
        text = solve.at_link(text) if link and fit is whole else text
        tell = laws.tell_speed(self.rigid(q, u), self.t_det(), self.a_brake(q, u)) if fit.get("e0") else None
        z, text = bounds_stops.solving(z, text, fit, tell)
        return self._derived("slope", (z, text), "contact" if fit.get("c") is not None else "model", text)

    def thing(self, ref):
        E = thing_of(self, ref)
        return self._derived("thing", E, "contact" if E.touched else "seen", "session thing evidence")

    def distance(self, q, thing, body=None):
        E, q = self.thing(thing), self._pose(q)[0]
        fits = self.robot.e_fit()
        values, pairs = [], {}
        for point in self.robot.points(q):
            if body is not None and point.body != body:
                continue
            pairs.setdefault(point.body, []).append(point)
        for name, points in pairs.items():
            c = segment_clearance(points[0].p, points[-1].p, E, points[0].radius)
            if c <= fits.get(name, 0) + laws.band(E.sigma_p or 0):
                hull = self.robot.hull_points(q, name) if name != "tip" else None
                if hull is not None and len(hull):
                    c = min(point_clearance(p, E) for p in hull)
            values.append(c)
        return self._derived("distance", min(values, default=math.inf))

    def _sheet(self, E, intent):
        return thing_sheet(self.ex.world, E, intent)

    def _entry(self, ref):   # the world entry a call names by id or label, or None
        if not isinstance(ref, str):
            ref = ref.get("id", ref.get("label")) if isinstance(ref, dict) else getattr(ref, "id", None)
        if ref is None:
            return None
        return next((v for v in self.ex.world.query(include_stale=True) if ref in (v.get("id"), v.get("label"))), None)

    def stated_limit(self, thing=None, intent=None, expected=False):
        """The least limit somebody STATED (rcpx/bounds_limit.py gather_limit); limit_n None when none is."""
        got = solve.noted(self, limit.gather_limit(self, thing, intent, expected))   # a limit, or a force to end at
        return self._derived("stated_limit", got, got["source"] if got["limit_n"] is not None else "model", got["evidence"])

    def arrival(self, limit_n, source, q, u, held=None, floor=None):
        """{"v_m_s", "kept", "shown_by", "seek", "strike_n", "f_guard_n", "text"}: the speed at which a contact
        under limit_n is met at q along u, solved from the contact threshold of the arm at rest at that speed
        (under a known hold no less than the hold's level) and the stop slope here (rcpx/bounds_limit.py arrival
        and gather_arrival, which a caller that has derived its leg's guard asks itself). floor: stated_limit's
        "floor" when the number is the force at which a touch ends: never under the tell speed. A limit,
        or an ending force with a limit beside it, is solved to be kept; nothing is held to the fastest stop
        measured: above it the energy bound states the strike (bounds_know_solve.above says so)."""
        mode = "limit" if floor is None or floor[0] is not None else (self._solving or "ends")
        got = solve.above(self, solve.run(self, mode, lambda: limit.gather_arrival(self, limit_n, source, q, u, held, floor=floor)))
        return self._derived("arrival", got, source, got["text"])

    def f_allow(self, thing, q, u, intent=None, meant=False):
        """The force a contact with the thing may see, as a bound item: the stated limit when under what the
        robot itself can push there, else the robot's own side. meant (the move is to move the thing): only the operator's."""
        got = self.stated_limit(thing, intent, expected=True)
        own, term = laws.f_allow(None, self.f_cap(q, u), self.e_tau(), self.f_tool())
        if got["limit_n"] is not None and got["limit_n"] < own and not (meant and got["source"] != "intent"):
            value, source, evidence, lift = got["limit_n"], got["source"], got["evidence"], "state another limit on the call (max_n)"
        else:
            value, source = own, "rating" if term == "tool" else "model"
            evidence = "nothing stated lowers it: " + ("the tool's stated contact rating" if term == "tool" else "what the arm can push here")
            lift = "nothing: the robot's own rating"
        return self._derived("f_allow", report.item("force", value, source, evidence, lift), source, evidence)

    def v_in(self, thing, q, u, intent=None, held_n=0.0):
        """(the speed a contact with the thing is met at, its bound item): B.arrival's under a stated limit, else the body's."""
        q, u = self._pose(q, u)
        got = self.stated_limit(thing, intent, expected=True)
        free = self.v_free(q, u)
        if got["limit_n"] is None:
            source, evidence = self.src("qd_max")
            return self._derived("v_in", (free, report.item("speed", free, source, "nothing stated lowers this arrival: " + evidence,
                                                            "nothing: the robot's own rating")), source, evidence)
        arr = self.arrival(got["limit_n"], got["source"], q, u, floor=got.get("floor"))
        v = free if arr["v_m_s"] is None else min(free, arr["v_m_s"])
        evidence = got["evidence"] + "; " + arr["text"]
        return self._derived("v_in", (v, report.item("speed", v, got["source"], evidence, "state another limit on the call (max_n)")),
                             got["source"], evidence)

    def note_give(self, thing, force_n, why):
        """A give of this session: it bounds later contacts with that thing and things of its stated label."""
        return note_give(self, thing, force_n, why)

    def note_stop(self, thing, q, u, v, peak_n, impulse_ns, resolved, moved_m, held=False, rest_n=None, k_c=None,
                  ext_peak=None, *, det_periods=None, expect=None, stop=None):
        """A contact stop (bounds_book.note_stop). It keeps its pose; an arrival's figures are remembered."""
        if isinstance(stop, dict):
            stop.setdefault("q", [float(x) for x in np.asarray(q, dtype=float)])
            before = (stop.get("said") or {}).get("before")
            if stop.get("v") and not (before and stop["v"] <= REL * (before.get("v_m_s") or 0.0)):
                from rcpx import bounds_strike
                self._arrived = {"tip": np.asarray(self.robot.tip(q)[0], dtype=float), "v": float(stop["v"]),
                                 "ran": bounds_strike.ran(dict(stop, said=dict(stop.get("said") or {}, before=None)))}
        return note_stop(self, thing, q, u, v, peak_n, impulse_ns, resolved, moved_m, held, rest_n, k_c, ext_peak,
                         det_periods, expect, stop)

    def guard(self, kind, v=0.0, a=0.0, held=None, away_from=None, readings=2):
        q = getattr(self.ex, "_q_des", None)
        q, u = self._pose(q, getattr(self.ex, "_travel_direction", None))
        direction = None
        if kind in ("lift", "retreat", "release") and away_from is not None:
            if not isinstance(away_from, (str, dict, Thing)):
                vec = np.asarray(away_from, dtype=float)
                if abs(np.linalg.norm(vec) - 1) <= laws.ROUND:
                    direction = vec
            if direction is None:
                E = self.thing(away_from)
                direction = np.asarray(E.extent["plane"][0], dtype=float) if E.extent and "plane" in E.extent else (self.robot.tip(q)[0] - E.point if E.point is not None else None)
            norm = np.linalg.norm(direction) if direction is not None else 0
            direction = direction / norm if norm > 0 else None
        cap, term = laws.f_allow(None, self.f_cap(q, u), self.e_tau(), self.f_tool())
        cap_source = "the tool's stated contact rating" if term == "tool" else "what the arm can push along this travel"
        hold = None if not (isinstance(held, dict) and "weight_n" in held) else ("first" if held["weight_n"] is None else "known")
        if hold == "first":
            # the load is not read yet, so no contact threshold can tell it from a contact: for this ONE leg
            # only the cap judges (the least of the arm's own push and the hold's stated limit)
            if held.get("limit_n") is not None and held["limit_n"] < cap:
                cap, cap_source = held["limit_n"], f"the hold's stated limit ({held.get('limit_source')})"
            spec = GuardSpec(None, None, self.tau_thr(v), direction, cap, readings)
            source = f"hold first: no contact threshold on this one leg, the load is not read yet; cap {cap:.3g} N: {cap_source}"
        else:
            force, one = self.f_thr(v, a, held=held), self.f_thr(v, a, 1, held)
            source = f"the contact threshold at this speed and acceleration; cap {cap:.3g} N: {cap_source}"
            level = held.get("level_n") if hold == "known" else None
            if level is not None and level > force:
                force, one = level, max(one, level)
                source = f"the hold's own level ({held.get('level_source')}); cap {cap:.3g} N: {cap_source}"
            spec = GuardSpec(force, one, self.tau_thr(v), direction, cap, readings)
        spec.source, spec.hold, spec.expect = source, hold, None     # plain attributes: KNOW adds the fields
        return self._derived("guard", spec)

    def force(self, kind, thing, q, u, intent=None):
        """The force ramp of a stroke: its first force, its rate, its ceiling with every term's source. The ceiling
        is the least of the arm's own rating, a sheet's highest working force, the call's max_n or force_n, a
        hold's measured slip fuse and the force that already moved this part or its device. With no force stated
        and none known the ramp runs to the arm's own rating and the bound offers max_n: never a refusal."""
        intent = intent or {}
        E = self.thing(thing)
        own, term = laws.f_allow(None, self.f_cap(q, u), self.e_tau(), self.f_tool())
        terms = [(own, "rating" if term == "tool" else "model",
                  "the tool's stated contact rating" if term == "tool" else "the arm's own rating: what it can push here")]
        sheet = self._sheet(E, intent)
        high = sheet.get("max_n") if sheet.get("max_n") is not None else sheet.get("force_high_n")
        if high is not None:
            terms.append((high + laws.TWO_READ * self.sigma_f(), "sheet", "sheet's highest working force plus noise"))
        for key in ("max_n", "force_n"):
            if intent.get(key) is not None:
                terms.append((intent[key], "intent", "on your word: " + key))
        fuse = (intent.get("held") or {}).get("g_max_n")
        if fuse is not None:
            terms.append((fuse, "contact", "the hold's measured slip fuse"))
        moving = kind == "work_part" or intent.get("move") or intent.get("firm")
        stated = high is not None or any(intent.get(k) is not None for k in ("max_n", "force_n"))
        moved = [f for f in (E.f_moved, self._device_moved(E)) if f is not None]
        if not stated and not moving and moved:
            terms.append((max(moved), "contact", "the largest force that moved this part or another part of its device"))
        terms.sort(key=lambda t: t[0])
        ceiling = terms[0][0]
        word = "; on your word to move it" if moving else ""
        if not stated and not moving and not moved:
            word = "; no force was stated or known: the ramp stops when the part moves; state max_n to bound it"
        items = [report.item("force", f, source, evidence + word, "repeat this call with max_n stated") for f, source, evidence in terms]
        rate = lambda f: laws.ramp_rate(f, self.sigma_f(), self.e_tau(), self.t_det(), ceiling)
        spec = ForceSpec(self.f_thr(), rate, ceiling, items, None)
        spec.offer = None if stated or moving or moved else "max_n"
        return self._derived("force", spec)

    def _device_moved(self, E):
        """The largest force that moved another part of the same device entry (A4.5): world entries whose data
        names the same device, and the device's own entry."""
        entries = self.ex.world.query(include_stale=True)
        own = next((v for v in entries if v.get("id") == E.id), None)
        device = ((own or {}).get("data") or {}).get("device")
        if device is None:
            return None
        forces = [(v.get("data") or {}).get("f_moved_n") for v in entries
                  if v.get("id") == device or (v.get("data") or {}).get("device") == device]
        return max((f for f in forces if f is not None), default=None)

    def travel(self, kind, thing, q, u, intent=None):
        E = self.thing(thing)
        support = self.space.support()
        past = laws.reach_past(E.sigma_p or 0, self.rigid(q, u))
        search = laws.BAND * (support[1] or 0) if support else 0.0
        back = self.robot.w_tool() + (float(np.linalg.norm(E.end)) if E.end is not None else 0)
        return self._derived("travel", TravelSpec(past, search, back, [report.item("travel", past, "model", "position band plus rigid travel", "touch again to refine the position error")]))

    def grip(self, held):
        capacity = laws.grip_capacity(held.get("proofs", []), held.get("squeeze_n", 0))
        accel = laws.accel_ok(capacity, held["mass_kg"], self.robot.g())
        up = laws.accel_ok(capacity, held["mass_kg"], self.robot.g(), cos_up=1.0)
        evidence = f"proved hold capacity; level acceleration {accel}; straight-up acceleration {up} m/s^2"
        return self._derived("grip", (capacity, accel, [report.item("force", capacity, "contact", evidence, "prove the lift at a higher squeeze")]))

    def wait(self, kind="settle", need_m=None):
        return self._derived("wait", WaitSpec("none" if need_m is None else "quiet", self.f_thr(0, 0), need_m, self.t_det(), self.limit_s(self.t_settle())))

    def limit_s(self, planned_s):
        return self._derived("limit_s", laws.limit_s(planned_s))

    def standoff(self, v, thing=None, q=None, u=None):
        E = self.thing(thing) if thing is not None else None
        return self._derived("standoff", laws.standoff(self.d_stop(v, q, u), E.sigma_p or 0 if E else 0, self.robot.w_tool()))

    def clearance(self, v, thing=None, body=None):
        E = self.thing(thing) if thing is not None else None
        fit = self.robot.e_fit().get(body, 0) if body is not None else 0
        return self._derived("clearance", laws.clearance(fit, self.e_cross(v), E.sigma_p or 0 if E else 0, v * self.dt()))

    def spacing(self, margin_m):
        return self._derived("spacing", laws.spacing(margin_m))

    def report(self, result, items, detail="brief"):
        """Attach the bounds; a cold session's first result also carries "selfknow"."""
        cold = getattr(self.ex, "_selfknow_cold", None)
        if cold is not None and not getattr(self, "_cold_said", False) and isinstance(result, dict):
            result["selfknow"], self._cold_said = cold, True
        return report.attach(result, items, detail)

    def begin(self, what="call"):
        """A call, or a phase of one, begins. Nothing is re-seen and nothing ages: no changes; the word goes."""
        if what == "call":
            self._word_info = None
            solve.new_call(self)          # the numbers of the call before; a cold session's first line
        return []

    def word(self, intent, path=None):
        """The operator's word: {"look", "given", "text", "refused"}: recorded; refused only for an unknown look."""
        intent = intent or {}
        clear = intent.get("clear")
        named = clear.get("look") if isinstance(clear, dict) else (clear if isinstance(clear, str) else intent.get("look"))
        if not clear and named is None:
            return {"look": None, "given": False, "covered": None, "text": "", "refused": None}
        looks = self.space.looks
        if named is not None and named not in looks and named not in (getattr(self.ex, "_looks", None) or {}):
            return {"look": None, "given": True, "covered": None, "text": "",
                    "refused": refusal("unknown_look", f"clear names look {named}, which this session does not hold",
                                       "name a look of this session, or say clear: true")}
        look = named if named is not None else (self.space.newest_look() or {}).get("id")
        event, rec = self.space.last_event(), looks.get(look)
        text = "given; recorded: your word changes no speed and no guard"
        if look is None:
            text = "given with no look of this session; recorded"
        elif event is not None and rec is not None and rec["order"] <= event[-1]:
            text = f"given; your look {look} is older than the last event ({event[0]}); recorded as information"
        self._word_info = {"look": look, "given": True, "covered": None, "text": text, "refused": None}
        return self._word_info

    def plan(self, q0, path, kind, target=None, intent=None, held=None, away_from=None, expect=None):
        """expect: the expected contact of the path's last stretch, or one per path segment (None where a
        segment expects none); intent expect "contact" marks the last stretch when no expect is given."""
        from rcpx.bounds_plan import plan
        return solve.planned(self, [intent], held, expect, lambda: plan(
            self, q0, path, kind, target, intent, held, away_from, expect=expect))

    def plan_time(self, q0, path, kind, target=None, intent=None, held=None, away_from=None, expect=None):
        """plan without the joint vector per period (path is empty): the same bounds, items, time_s and periods,
        for a caller that only weighs how long a way takes."""
        from rcpx.bounds_plan import plan
        return solve.planned(self, [intent], held, expect, lambda: plan(
            self, q0, path, kind, target, intent, held, away_from, materialize=False, expect=expect))

    def plan_legs(self, q0, legs, held=None, looks=None, intent=None):
        from rcpx.bounds_plan import plan_legs
        return solve.planned(self, [intent] + [leg.get("intent") for leg in legs], held,
                             [leg.get("expect") for leg in legs], lambda: plan_legs(self, q0, legs, held, looks, intent))

for _name in ("dt", "qd_max", "qdd_max", "tau_max", "quantum", "joint_quantum", "rigid", "f_hidden", "sigma_f",
              "f_thr", "tau_thr", "t_det", "lag", "e_cross", "t_settle", "a_brake", "d_stop", "v_free", "w_free",
              "f_cap", "f_tool", "f_hand", "e_tau", "m_eff", "k_servo", "model_z", "slope", "standoff", "clearance", "spacing",
              "limit_s", "stated_limit", "arrival", "f_allow", "v_in", "guard", "force", "travel", "grip", "wait"):
    setattr(Bounds, _name, quantity(getattr(Bounds, _name)))
