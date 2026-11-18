# SPDX-License-Identifier: AGPL-3.0-only
"""Poses and moves of package place (N4), version two (SPEC-R4 B1, B4, C2).

Poses: the hand pose that points the tool along an approach with the jaws across a chosen bearing, and its solvers
(pure kinematics; their tolerances are solver numerics).

Moves: two calls, and neither passes a speed, a force or a limit (the speed of each is the plan of rcpx.bounds for
the kind of its legs; the contact stop is armed on every one):
- _pl_stream(legs, names, held): ONE stream (the executor's _stream), its result in this package's words. The
  legs of a way to a place are made in rcpx/r2/place_carry.py.
- _pl_line(dp_m, kind, target, away_from, intent): one straight tool line of the executor's own Cartesian move.
The operator's word changes no move here: it is passed on as information and nothing is run again for it.

What is carried is the core's one record (rcpx/control_hold.py, contract C8); this package adds nothing to it.
hold_facts reads what a carry and a set down need of it: how far the held thing reaches below the tip and its
half width, from the extent the take stated on the record, else from where the jaws closed over the support.

Phases: phase(ex, name, did, t0, items) appends the phase record of SPEC-R4 A5 (periods split into moving, waiting
and jaw, seconds, free_share, the binding bound) for what ran since world time t0 and re-sees the space (bounds
begin). The split comes from a log this package keeps of every control period. No robot is named here.
"""

import math

import numpy as np

from rcpx import bounds
from rcpx import bounds_report as report
from rcpx import control_hold
from rcpx.context import refusal
from rcpx.model import ROT_WEIGHT_M, _axis_error, _rotvec

MM = 1000.0
UP = np.array([0.0, 0.0, 1.0])
IK_RESTARTS = 4            # solver numerics (kept): restarts, rotation tolerances and the step clamp below
FULL_HAND_JOINTS = 6       # fact: an arm needs six joints to hold a whole hand rotation at a point


def unit(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def _skew(r):
    return np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])


def horizontal(v):
    v = np.asarray(v, dtype=float).copy()
    v[2] = 0.0
    return unit(v)


def pose(p, R):
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(p, dtype=float)
    return T


# ---------------------------------------------------------------- the period log and the phase record
def _place(ex):
    return ex.state.setdefault("place", {})


def log_of(ex):
    return _place(ex).setdefault("log", {"rows": [], "moves": [], "last": None})


def period_tick(ex):
    """A tick hook: one row per control period the backend ran (time, what the period was for; a backend that counts
    its own control steps, steps_run, and ran none in a period, its budget spent, logs none: the receipt's periods
    are the step counter's, final review 2, medium 4). The executor's own word for
    the period (moving, waiting, jaw) when it keeps one; else this log's reading of it: the arm's commanded pose
    changed (moving), or it stood while the jaws' command changed since the arm last moved (jaw), or neither."""
    def fn(s):
        log = log_of(ex)
        ran = getattr(ex.backend, "steps_run", None)
        if isinstance(ran, int) and ran == log.get("steps_run"):
            return None      # the backend ran no control step (its budget is spent, the world frozen): no period
        log["steps_run"] = ran if isinstance(ran, int) else None
        q = None if ex._q_des is None else np.asarray(ex._q_des, dtype=float)
        last, cmd = log["last"], ex._gripper_cmd
        moved = bool(last is not None and q is not None and last[0].shape == q.shape and np.any(last[0] != q))
        acting = bool(last is not None and (cmd != last[1] or (last[2] and not moved)))
        stand = ex.__dict__.get("_stand")
        if (last is not None and cmd != last[1]) or (stand is not None and stand is not log.get("stand")):
            _place(ex)["scene_t"] = float(s.t)    # the scene was acted on: the jaws were told to move, or a guard
        log["stand"] = stand                      # wrote where the tool bore on something (place_near.scene_t)
        log["last"] = (None if q is None else q.copy(), cmd, acting) if q is not None else None
        kind = getattr(ex, "_period_kind", None)
        if kind not in ("moving", "waiting", "jaw"):
            kind = "moving" if moved else ("jaw" if acting else "waiting")
        log["rows"].append((float(s.t), kind))
        return None
    return fn


def hold_tick(ex):
    """A tick hook: when the core's record of a hold is new, keep where the tip stood as the jaws closed (the
    record's own judged_at, before the test lift's rest read moves it): what a hold that no take described is
    read from (PlaceMotion._pl_hold_facts)."""
    def fn(s):
        rec = ex.__dict__.get("_hold")
        if rec is None:
            ex.__dict__["_pl_hold_seen"] = None
        elif ex.__dict__.get("_pl_hold_seen") is not rec:
            ex.__dict__["_pl_hold_seen"] = rec
            at = dict.get(rec, "judged_at")
            _place(ex).update(closed_at=None if at is None else [float(x) for x in at],
                              closed_R=[[float(x) for x in row] for row in np.asarray(s.R, dtype=float)])
        return None
    return fn


def reset_log(ex):
    """A new operator call: the log of the last one goes (phases never span calls)."""
    log = log_of(ex)
    del log["rows"][:], log["moves"][:]
    log.pop("phase_end_t", None)


def split(ex, t0, t1=None):
    """(moving, waiting, jaw) control periods logged after world time t0 (and up to t1)."""
    n = {"moving": 0, "waiting": 0, "jaw": 0}
    for t, kind in log_of(ex)["rows"]:
        if t > t0 and (t1 is None or t <= t1):
            n[kind] += 1
    return n["moving"], n["waiting"], n["jaw"]


def free_share(ex, t0, t1=None):
    """The share of the moving periods after t0 that ran at the free speed, from the moves this package made
    (each move's own plan says how many of its periods a bound slowed); None when no move said."""
    total = free = 0.0
    for m in log_of(ex)["moves"]:
        if m["t"] > t0 and (t1 is None or m["t"] <= t1) and m.get("free") is not None:
            total += m["moving"]
            free += m["free"]
    return None if total <= 0 else round(free / total, 3)


def binding(items):
    """The binding bound of a phase: the first that is named in a list of items, move results or bounds blocks."""
    if isinstance(items, dict):
        items = [items]
    for it in items or ():
        if not isinstance(it, dict):
            continue
        if isinstance(it.get("binding"), dict):
            return it["binding"]
        if "what" in it and "source" in it:
            return it
        b = it.get("bounds")
        if isinstance(b, dict) and isinstance(b.get("binding"), dict):
            return b["binding"]
        if isinstance(b, (list, tuple)) and b and isinstance(b[0], dict):
            return b[0]
    return None


def phase(ex, name, did, t0, items=None):
    """Append the phase record of SPEC-R4 A5 for what ran since world time t0 and re-see the space; returns it.
    The phases PARTITION the call's control periods IN ORDER (contract C12; final review 2, medium 4 and severe 3
    (c)): a period an earlier phase of this call already counted is never counted again, and a period no phase
    was open for (between the end of the phase recorded before and this one's t0, or before the call's first
    phase) is this phase's, the leg that follows it (work_part's rule, rcpx/r2/fixture_strike.open_phase): so a
    phase counts from the end of the phase recorded before it, or from the call's start, and every period a
    contact can be read in belongs to the leg whose figure covers it. t0 is the phase's own start, kept for the
    record's caller."""
    log = log_of(ex)
    t_from = float(log.get("phase_end_t", float("-inf")))
    moving, waiting, jaw = split(ex, t_from)
    if log["rows"]:
        log["phase_end_t"] = max(float(log["rows"][-1][0]), t_from)
    rec = report.phase_record(str(name), str(did), moving, waiting, jaw,
                              round((moving + waiting + jaw) * float(ex.dt), 3), free_share(ex, t_from), binding(items))
    _place(ex).setdefault("phases", []).append(rec)
    try:
        changed = bounds.of(ex).begin("phase")
    except Exception as exc:        # a re-see that fails must not lose the phase; the record says so
        changed = None
        rec["resee"] = f"not re-seen: {type(exc).__name__}"
    if changed:
        rec["changed"] = changed
    return rec


def note_move(ex, result, t0):
    """Remember what one move's plan said of its own periods at the free speed (for free_share); None when it
    said nothing."""
    if not isinstance(result, dict) or "refused" in result:
        return
    moving, _, _ = split(ex, t0)
    plan = result.get("plan")
    share = getattr(plan, "free_share", None) if plan is not None else result.get("free_share")
    slowed = result.get("slowed")
    free = None
    if isinstance(share, (int, float)):
        free = float(share) * moving
    elif isinstance(slowed, (int, float)):
        free = max(0.0, moving - float(slowed))
    log_of(ex)["moves"].append({"t": ex.sim_time(), "moving": moving, "free": free})


def move_bounds(r):
    """The bounds block and the unseen note of an executor move result, wherever that result keeps them."""
    keys = r.get("speed") if isinstance(r.get("speed"), dict) else {}
    return r.get("bounds", keys.get("bounds")), r.get("unseen", keys.get("unseen"))


class PlaceMotion:
    """Helpers of the place Mixin (every name is prefixed: all package Mixins share one class)."""

    # ---------------------------------------------------------------- orientation
    def _pl_rot(self, t, yaw_rad):
        """The hand rotation with the tool's pointing axis (tool x) along t and the jaws closing along the
        horizontal bearing yaw_rad (base frame), or the nearest that t allows."""
        t = unit(t)
        n = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0.0])
        n = unit(n - float(n @ t) * t)
        if float(np.linalg.norm(n)) < 0.5:              # t horizontal along the bearing: close across it
            n = unit(np.cross(UP, t))
        a = unit(np.array([0.0, self._pl_jaws.axis_T[1], self._pl_jaws.axis_T[2]]))
        b = np.cross(np.array([1.0, 0.0, 0.0]), a)
        world = np.column_stack([t, n, np.cross(t, n)])
        tool = np.column_stack([np.array([1.0, 0.0, 0.0]), a, b])
        return world @ tool.T

    def _pl_jaw_bearing(self, R):
        """The horizontal bearing (rad) the jaws close along at hand rotation R."""
        n = R @ self._pl_jaws.axis_T
        return math.atan2(float(n[1]), float(n[0]))

    def _pl_tip_for(self, centre, R, frac, obj_w=None):
        """The tip point that puts an object's middle (width obj_w, m) on centre with the jaws open at frac
        (place_hand.Jaws.hold_centre_T)."""
        return np.asarray(centre, dtype=float) - R @ self._pl_jaws.hold_centre_T(frac, obj_w)

    # ---------------------------------------------------------------- solving
    def _pl_solve(self, p_tip, R, t, seeds):
        """A joint pose with the tip at p_tip: the whole hand rotation R when the arm can hold it, else (an arm of
        fewer than six joints) the pointing axis t only. (q, R_reached) or (None, None)."""
        M = self.model
        tries = [("R", R)] if M.n >= FULL_HAND_JOINTS else [("R", R), ("axis", t)]
        for kind, target in tries:
            for q0 in seeds:
                if q0 is None:
                    continue
                kw = {"R_target": target} if kind == "R" else {"keep_axis": ("x", target)}
                rt = math.radians(3.0) if kind == "R" and M.n < FULL_HAND_JOINTS else None
                q, ok, _ = M.ik(np.asarray(q0, dtype=float), np.asarray(p_tip, dtype=float), restarts=IK_RESTARTS,
                                rot_tol=rt, **kw)
                if ok and M.within_limits(q):
                    return q, M.fk_tip(q)[1]
        return None, None

    def _pl_ik_held(self, q0, p_centre, c_T, t, R=None, iters=150, restarts=4, seed=0):
        """Joints that put the hand-fixed point c_T (tool frame, m) on p_centre with the hand rotation R (an arm of
        six joints or more) or the pointing axis along t (fewer): damped least squares on the point's Jacobian,
        limits respected, from q0, home and seeded random starts. q or None."""
        M = self.model
        rng = np.random.default_rng(seed)
        starts = [M.clip(q0), M.clip(M.q_home)] + [rng.uniform(M.q_lo, M.q_hi) for _ in range(restarts)]
        w = ROT_WEIGHT_M
        target = np.asarray(p_centre, dtype=float)
        rot_tol = 0.01 if R is not None else math.radians(1.0)
        for q in starts:
            q = q.copy()
            for _ in range(iters):
                p, Rt = M.fk_tip(q)
                r = Rt @ c_T
                J = M.jac(q)
                Jp = J[:3] - _skew(r) @ J[3:]
                e_p = target - (p + r)
                if R is not None:
                    e_r = _rotvec(np.asarray(R) @ Rt.T)
                else:
                    e_r = _axis_error(Rt[:, 0], t, Rt[:, 1])
                if float(np.linalg.norm(e_p)) < 1e-3 and float(np.linalg.norm(e_r)) < rot_tol:
                    return q if M.within_limits(q) else None
                dq = M._dls(np.vstack([Jp, w * J[3:]]), np.concatenate([e_p, w * e_r]), q)
                big = float(np.max(np.abs(dq)))
                if big > 0.3:
                    dq *= 0.3 / big
                q = M.clip(q + dq)
        return None

    def _pl_best_pose(self, p_centre, t, yaws, frac, seeds, obj_w=None):
        """Over the candidate jaw bearings, the reachable pose (q, R reached, yaw, squared) whose joints lie nearest
        the first seed, or (None, None, None, False). The whole hand rotation is solved first (squared: the jaws
        close along the bearing asked); an arm of fewer than six joints that reaches none of them is solved for the
        pointing axis alone, its jaws turned as its pose allows (squared false)."""
        J = self._pl_jaws
        c_T = J.hold_centre_T(frac, obj_w)
        best = None
        q_ref = np.asarray(seeds[0], dtype=float)
        for yaw in yaws:
            R = self._pl_rot(t, yaw)
            q = None
            for q0 in seeds:
                if q0 is not None:
                    q = self._pl_ik_held(np.asarray(q0, dtype=float), p_centre, c_T, unit(t), R)
                if q is not None:
                    break
            if q is None:
                continue
            cost = float(np.max(np.abs(q - q_ref)))
            if best is None or cost < best[0] - 1e-6:
                best = (cost, q, self.model.fk_tip(q)[1], yaw, True)
        if best is None and self.model.n < FULL_HAND_JOINTS:
            for q0 in seeds:
                q = None if q0 is None else self._pl_ik_held(np.asarray(q0, dtype=float), p_centre, c_T, unit(t))
                if q is not None:
                    Rq = self.model.fk_tip(q)[1]
                    best = (0.0, q, Rq, self._pl_jaw_bearing(Rq), False)
                    break
        if best is None:
            return None, None, None, False
        return best[1], best[2], best[3], best[4]

    # ---------------------------------------------------------------- what is carried
    def _pl_gravity(self):
        return float(np.linalg.norm(np.asarray(self.model.mj.opt.gravity, dtype=float)))

    def _pl_held(self):
        """What the hand holds: the core's one record (rcpx/control_hold.py), or None. Nothing is added to it."""
        return control_hold.held(self)

    def _pl_hold_facts(self):
        """What a carry and a set down read of the hold, or None with nothing held: {"below_m": how far the held
        thing reaches below the tip along the tool, "below_source", "half_m": its half width across the tool,
        "above_m": how far it reaches up the fingers, "offset_T": the held thing's middle in the tool frame (the
        middle of the jaws' gap; the thing's own seen middle when this call's take saw it whole), "middle":
        which of the two, "gap_m", "below_max_m": the most it can hang (how far the fingertips stood over the
        stated support as the jaws closed: what a level stretch clears), "doubt": text or None}. From the extent the take stated on the record ({"below_tip_m",
        "half_width_m"}, or its corners in the tool frame); with none stated, from where the jaws closed: the
        thing stood on something at or over the stated support, so it reaches no further below the tip than the
        tip stood over that support then (said as a doubt)."""
        rec = control_hold.held(self)
        if rec is None:
            return None
        J = self._pl_jaws
        q, _ = J.reading(self.backend.read().get("gripper"), self._gripper_cmd)
        frac = float(rec["frac"]) if q is None else float(J.frac_of_q(q))
        gap = float(rec["gap_m"]) if dict.get(rec, "gap_m") is not None else float(J.width_m(J.q_of_frac(frac)))
        out = {"offset_T": np.asarray(J.gap_centre_T(frac), dtype=float), "gap_m": gap, "half_m": gap / 2,
               "above_m": float(J.finger_len_m or J.max_m), "weight_n": dict.get(rec, "weight_n"), "doubt": None,
               "middle": "the middle of the jaws' gap"}
        st = self.state["place"]
        took = st.get("taken")      # what the look showed of the thing this call took (place.py _pl_pick_run)
        if took and st.get("closed_at") is not None and st.get("closed_R") is not None:
            # the thing's own middle, where the hand took it off its middle (a rim, a handle): its seen middle in
            # the tool frame as the jaws closed; never further from the gap than the thing is wide
            off = np.asarray(st["closed_R"], dtype=float).T @ (
                np.array([took["centre_xy"][0], took["centre_xy"][1], st["closed_at"][2]])
                - np.asarray(st["closed_at"], dtype=float))
            off[0] = out["offset_T"][0]
            if float(np.linalg.norm(off - out["offset_T"])) <= float(took["wide_m"]):
                out.update(offset_T=off, middle="the middle of the thing as the look showed it")
            out["half_m"] = max(out["half_m"], float(took["narrow_m"]) / 2)
        at, floor = st.get("closed_at"), self._pl_support_z()
        if at is not None and floor is not None:       # the most the thing can hang: it stood at or over the support
            out["below_max_m"] = max(0.0, float(at[2]) - floor)
        ext, corners = dict.get(rec, "extent"), dict.get(rec, "extent_m")
        if isinstance(ext, dict) and ext.get("below_tip_m") is not None:
            out.update(below_m=max(0.0, float(ext["below_tip_m"])), below_source="the take's own reading")
            if ext.get("half_width_m") is not None:
                out["half_m"] = max(out["half_m"], float(ext["half_width_m"]))
            return out
        if corners is None and isinstance(ext, (list, tuple)):
            corners = ext
        if corners is not None and len(corners):
            C = np.asarray(corners, dtype=float).reshape(-1, 3)
            out.update(below_m=max(0.0, float(C[:, 0].max())), below_source="the take's own reading",
                       half_m=max(out["half_m"], float(np.hypot(C[:, 1], C[:, 2]).max())),
                       above_m=max(0.0, float(-C[:, 0].min())))
            return out
        below = None if at is None or floor is None else max(0.0, float(at[2]) - floor)
        if below is None:
            out.update(below_m=0.0, below_source="not known",
                       doubt="how far the held thing reaches below the fingertips is not known: taken as nothing")
        else:
            out.update(below_m=below, below_source="where the jaws closed over the stated support",
                       doubt=f"how far the held thing reaches below the fingertips was not read by a take: at most "
                             f"{below * MM:.0f} mm (the fingertips stood that far over the stated support when "
                             "the jaws closed)")
        return out

    def _pl_support_z(self):
        """The height of the stated support (m, base frame), or None when none is stated."""
        try:
            sup = bounds.of(self).space.support()
        except Exception:
            return None
        if not sup or sup[0] is None:
            return None
        n = np.asarray(sup[0][0], dtype=float)
        return float(sup[0][1]) / float(n[2]) if abs(float(n[2])) > 0 else None

    def _pl_res(self, B, q, u=UP):
        """The tip's own resolution along u (m): its quanta and its jitter, with no give under a force in it. It
        does not follow the session's contact level (B.rigid at its default does: one stop the bounds call false
        raises that level, and with it that figure, many times over)."""
        return float(B.rigid(q, u, 0.0))

    def _pl_below_tip(self, held=None):
        """How far the lowest carried point hangs below the tip along the tool's pointing axis (m)."""
        facts = self._pl_hold_facts()
        return 0.0 if facts is None else float(facts["below_m"])

    def _pl_half_width(self, held=None):
        """The half extent across the way of what moves: the hand's own hull about its pointing axis (place_set
        _pl_hand_half; the open jaws where no hull is read), or the held thing when it is wider."""
        fn = getattr(self, "_pl_hand_half", None)
        half = fn() if callable(fn) else self._pl_jaws.max_m / 2
        facts = self._pl_hold_facts()
        return half if facts is None else max(half, float(facts["half_m"]))

    # ---------------------------------------------------------------- moves
    def _pl_ref(self, ref):
        """A target as the bounds' own record: never a bare list (the core reads one as a target per waypoint)."""
        if ref is None or isinstance(ref, (str, bounds.Thing)):
            return ref
        return bounds.of(self).thing(np.asarray(ref, dtype=float))

    @staticmethod
    def _pl_core_intent(intent):
        """The intent a core move gets: the operator's word and wishes, without this package's own route notes."""
        if intent is None:
            return None
        return {k: v for k, v in intent.items() if k not in ("over_z_m", "clearance_m", "standoff_m", "need_m")}

    def _pl_stream(self, legs, names=None, held=None, looks=None, need_m=None):
        """The executor's stream, its result in this package's words: {stopped, leg, leg_name, contact, why,
        periods, looks, bounds, tip} or a refusal unchanged."""
        t0 = self.sim_time()
        legs = [dict(leg, target=self._pl_ref(leg.get("target")), away_from=self._pl_ref(leg.get("away_from")))
                for leg in legs]
        r = self._stream(legs, held=held, looks=looks, need_m=need_m)
        if not isinstance(r, dict):
            return refusal("motion", "the move did not run (an internal fault of the stream)", "a shorter move")
        if "refused" in r:
            return r
        note_move(self, r, t0)
        stop = r.get("stopped", r.get("stop"))
        i = r.get("leg")
        b, _ = move_bounds(r)
        return {"stopped": stop, "leg": i, "leg_name": names[i] if names and isinstance(i, int) and
                0 <= i < len(names) else None, "contact": r.get("contact"), "why": r.get("why", ""),
                "periods": r.get("periods"), "ended": r.get("ended"), "looks": r.get("looks") or [], "bounds": b,
                "strike": r.get("strike"), "predicted_peak_n": r.get("predicted_peak_n"),
                "peak_force_n": r.get("peak_force_n", r.get("peak_n")), "tip": self._sense().tip.copy()}

    def _pl_line(self, dp_m, kind, target=None, away_from=None, intent=None, held=False):
        """One straight tool line by dp_m (base frame) keeping the hand's rotation: the executor's own Cartesian
        move of the given kind (its speed and guard are the bounds' for that kind). held: the held block the
        bounds get (default: what the pick stored). Returns {stopped, tip (m), contact, peak_force_n, why,
        bounds, periods} or a refusal."""
        dp = np.asarray(dp_m, dtype=float)
        t0 = self.sim_time()
        s0 = self._told()
        intent = self._pl_core_intent(intent)
        held = self._pl_held() if held is False else held
        target, away_from = self._pl_ref(target), self._pl_ref(away_from)
        r = self._cart_move(s0, dp, None, None, True, True, kind=kind, target=target, intent=intent,
                            away_from=away_from, held=held)
        if "refused" in r:
            return r
        note_move(self, r, t0)
        b, _ = move_bounds(r)
        keys = r.get("speed") if isinstance(r.get("speed"), dict) else {}
        return {"stopped": r["stop"], "tip": r["s"].tip.copy(), "contact": r.get("contact"),
                "peak_force_n": None if r.get("peak_n") is None else round(float(r["peak_n"]), 2),
                "why": r.get("why", ""), "bounds": b, "strike": r.get("strike", keys.get("strike")),
                "periods": r.get("periods", keys.get("periods"))}

    def _pl_open_to(self, width_m):
        """Open (or close without squeezing) to width_m at the jaw tips; returns the width reached (m)."""
        J = self._pl_jaws
        self.gripper(J.frac_for_width(width_m), hold=True)
        return self._pl_width()

    def _pl_width(self):
        q, _ = self._pl_jaws.reading(self.backend.read().get("gripper"), self._gripper_cmd)
        return None if q is None else self._pl_jaws.width_m(q)

    def _pl_refuse_motion(self, r, what):
        """A primitive's refusal, with what this tool was doing when it came."""
        if isinstance(r, dict) and "refused" in r:
            body = dict(r["refused"])
            body["why"] = f"{what}: {body.get('why', '')}"
            return {"refused": body}
        return refusal("motion", f"{what}: the move did not run (an internal fault)", "a shorter move")
