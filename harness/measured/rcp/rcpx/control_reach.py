# SPDX-License-Identifier: AGPL-3.0-only
"""Reach and base give primitives of the Executor: reach_check and measure_base_give, and what follows a
stroke of control_contact.py: the back-off that follows the part, the path a retreat may retrace, and the stored
points a moved part made stale.

No length, force, speed or time here is typed: each comes from the bounds module (rcpx/bounds.py). No robot is named
here."""

import math

import numpy as np

from rcpx import bounds
from rcpx import bounds_laws as laws
from rcpx import control_expect, control_move_passed, control_move_press
from rcpx.bounds_ratios import GIVE, REL
from rcpx.context import refusal
from rcpx.control_base import Guard, _deg, _mm, primitive

MM = 1000.0
LEADING = (("x", 1.0, "tip (tool x leads)"), ("z", -1.0, "underside (tool -z leads)"),
           ("z", 1.0, "top (tool z leads)"), ("y", 1.0, "left side (tool y leads)"),
           ("y", -1.0, "right side (tool -y leads)"))
NAMES = {(1, 0, 0): "forward", (-1, 0, 0): "back", (0, 1, 0): "left", (0, -1, 0): "right", (0, 0, 1): "up",
         (0, 0, -1): "down"}
NAMED_COS = 0.95      # numeric: a direction is given a name when it lies this close to an axis (words only)
MARGIN_HALVINGS = 12  # numeric: halvings of the reach in the margin search (its resolution, not a limit)
LINE_TOL_RAD = math.radians(2.0)   # numeric: how far a line's plan may turn the finger off its hold (planner)


def _dir_name(v):
    for k, name in NAMES.items():
        if float(np.dot(v, k)) > NAMED_COS:
            return name
    return None


def ramp_seconds(rate, f_from, f_to, dt):
    """Time the ramp rate(F) (N/s) takes from f_from to f_to, stepped as the stroke steps it."""
    f, n = float(f_from), 0
    while f < f_to:
        step = rate(f) * dt
        if not step > 0.0:
            break
        f, n = f + step, n + 1
    return n * dt


def _refused(what, s0, q_des):
    """A stroke the bounds refuse: the refusal, nothing moved."""
    return {"stop": "refused", "refused": what["refused"], "s": s0, "q_des": q_des, "done": [], "free": [],
            "periods": 0}


class ReachMixin:
    # ------------------------------------------------------------------ reach_check
    def _valid(self, q):
        return self.floor_clearance(q) > 0.0 and not self.model.self_collisions(q, bounds.of(self).clearance(0.0))

    def _ik_valid(self, q0, p, keep_axis, restarts, tries=4):
        """An ik solution clear of the plane and of the arm itself. Starts: the present pose, then (for an axis
        constraint) a position-only solution as the warm start, then seeded random poses, so a converged pose
        that collides makes the next try start elsewhere."""
        rng = np.random.default_rng(7)
        starts = [np.asarray(q0, dtype=float)]
        if keep_axis is not None:
            qp, okp, _ = self.model.ik(q0, p, restarts=restarts)
            if okp:
                starts.append(qp)
        while len(starts) < tries:
            starts.append(rng.uniform(self.model.q_lo, self.model.q_hi))
        for k, start in enumerate(starts):
            q, ok, _ = self.model.ik(start, p, keep_axis=keep_axis, restarts=restarts if k == 0 else 2, seed=k)
            if ok and self._valid(q):
                return q
        return None

    def _push_rating(self, q, u):
        """What the robot may push along u at q: {"force_n": its joints' rating less the torque error, "tool_n":
        what its tool is rated for} (N; A4.3)."""
        B = bounds.of(self)
        return {"force_n": round(B.f_cap(q, u) * (1.0 - B.e_tau()), 1), "tool_force_n": None if B.f_tool() is None else round(B.f_tool(), 1)}

    @primitive
    def reach_check(self, point_mm, approach=None):
        m = self.model
        s = self._sense()
        p = np.asarray(point_mm, dtype=float).reshape(-1) / MM
        if p.size != len(NAMES) // 2 or not np.all(np.isfinite(p)):
            return refusal("point_mm", "point_mm must be three numbers", "[x, y, z] in base frame mm")
        a = None
        if approach is not None:
            a, bad = self._unit_in_base(approach, "base", s)
            if bad is not None:
                return bad
        options = [(None, "any orientation")] if a is None else [((ax, sg * a), what) for ax, sg, what in LEADING]
        sol = None
        for i, (ka, what) in enumerate(options):
            q = self._ik_valid(s.q, p, ka, 12 if i == 0 else 4)
            if q is not None:
                sol = (q, ka, what)
                break
        down = np.array(next(k for k, name in NAMES.items() if name == "down"), dtype=float)
        if sol is None:
            q, ok, err = m.ik(s.q, p, restarts=8)
            p_best = m.fk_tip(q)[0]
            if ok:
                why = ("the point is reachable but not arriving along that approach with any tool face"
                       if a is not None else "the arm poses that reach the point hit the mounting plane or the arm")
                return self._rating_words({"reachable": False, "margin_mm": 0.0, "short_by_mm": 0.0,
                                           "direction": None, "why": why,
                                           **self._push_rating(q, a if a is not None else down)})
            gap = p - p_best
            dist = float(np.linalg.norm(gap))
            u = gap / dist if dist > 0.0 else down
            return self._rating_words({
                "reachable": False, "margin_mm": 0.0, "short_by_mm": round(dist * MM, 1),
                "direction": [round(float(x), 3) for x in u], "direction_name": _dir_name(u),
                "closest_mm": _mm(p_best), **self._push_rating(q, a if a is not None else u),
                "why": f"out of reach by {dist * MM:.0f} mm" + (f" ({_dir_name(u)})" if _dir_name(u) else "")})
        q, ka, what = sol
        flat = float(np.hypot(p[0], p[1]))
        out_dir = a if a is not None else (np.array([p[0], p[1], 0.0]) / flat if flat > 0.0 else -down)
        # how much further along the stroke the tip could still go: searched to the edge of the arm's own reach
        lo, hi, q_b = 0.0, float(m.length_scale), q
        for _ in range(MARGIN_HALVINGS):
            mid = lo + (hi - lo) * GIVE
            q2, ok2, _ = m.ik(q_b, p + out_dir * mid, keep_axis=ka, restarts=1)
            if ok2 and self._valid(q2):
                lo, q_b = mid, q2
            else:
                hi = mid
        if a is not None:
            rating = self._push_rating(q, a)
        else:
            each = {name: self._push_rating(q, np.array(k, float)) for k, name in NAMES.items()}
            rating = {"force_n": {name: r["force_n"] for name, r in each.items()},
                      "tool_force_n": each["down"]["tool_force_n"]}
        return self._rating_words({
            "reachable": True, "margin_mm": round(lo * MM, 1), "margin_along": [round(float(x), 3) for x in out_dir],
            "short_by_mm": 0.0, "direction": None, "leading": what, **rating,
            "q_deg": {n: _deg(v) for n, v in zip(m.joint_names, q)}})

    @staticmethod
    def _rating_words(out):
        """A reach result whose tool has no stated contact rating says so (the jaw motor's force is none)."""
        if out.get("tool_force_n") is None:
            out["tool_force_note"] = "no contact rating is stated for the tool: what the arm can push stands alone"
        return out

    # ------------------------------------------------------------------ measure_base_give
    def _point_error_m(self):
        """The smallest error of the points robot://world holds (m), or None with none stored: what a shift of
        the base may not pass before those points are wrong."""
        errs = []
        for kind in ("contact", "located"):
            for e in self.world.query(kind=kind, include_stale=False):
                sigma = bounds.of(self).thing(e["id"]).sigma_p
                if sigma is None and (e.get("data") or {}).get("error_mm") is not None:
                    sigma = float(e["data"]["error_mm"]) / MM
                if sigma:
                    errs.append(float(sigma))
        return min(errs) if errs else None

    def _thing_at(self, p):
        """The touched thing the tip stands on at p (m): the stored contact it is on, or None."""
        B = bounds.of(self)
        best = None
        for e in self.world.query(kind="contact", include_stale=False):
            if e.get("point_mm") is None:
                continue
            t = B.thing(e["id"])
            gap = float(np.linalg.norm(np.asarray(e["point_mm"], dtype=float) / MM - p))
            # on it: within the stored point's own band and the give of arm and surface under the force that
            # contact showed (A4.3: reach past a target = BAND x sigma + rigid)
            own = B.rigid(self._sense().q, None, t.f_bear or t.f_gave or 0.0)
            if gap <= laws.reach_past(t.sigma_p or 0.0, own) and (best is None or gap < best[0]):
                best = (gap, t)
        return None if best is None else best[1]

    @primitive
    def measure_base_give(self, direction=None, force_n=None):
        """Push the touched surface both ways along direction by the ramp of A4.5 until the tip's shift reads
        (it passes what the arm's own give explains), at most to force_n (the force the operator plans next) or to
        what that surface has borne. The shift per newton bounds later pushes."""
        B = bounds.of(self)
        dark = self._dark_refusal_now()
        if dark is not None:
            return dark
        s = self._ease_impl()
        here = self._thing_at(s.tip)
        none = {"tilt_deg": None, "yaw_deg": None, "shift_mm": None}
        at_rest = B.f_thr()
        if here is None and s.Fmag <= at_rest:
            return {**none, "verdict": "not_measurable",
                    "why": "the tip is not on a surface (no touched point where it stands and no force felt)",
                    "hint": "touch a surface first, then measure"}
        up = np.array(next(k for k, name in NAMES.items() if name == "up"), dtype=float)
        d, bad = self._unit_in_base(up if direction is None else direction, "base", s)
        if bad is not None:
            return bad
        # the ramp's ceiling: the force the operator plans next, else what this surface has already borne
        top = force_n if force_n is not None else (here.f_bear if here is not None else None)
        intent = {} if top is None else {"max_n": float(top)}
        pushes, tops = [], []
        for sgn in (1.0, -1.0):
            s = self._sense()
            ramp = B.force("press", here, s.q, sgn * d, intent)
            if ramp.refused:   # no force stated and none known for this surface
                continue
            tops.append(ramp.ceiling_n)
            reach = B.rigid(s.q, d, ramp.ceiling_n) + ramp.ceiling_n / B.k_servo(s.q, d)
            r = self._push(s, sgn * d, reach, None, ramp=ramp, thing=here, intent=intent, until_moved=True)
            self._ease_impl()
            if r["stop"] == "refused":
                continue
            self._retrace_now(r["done"], here)
            if r["stop"] in ("moved", "force", "lead") and r["peak_n"] > at_rest:
                pushes.append({"dir": "with" if sgn > 0 else "against", "force_n": r["peak_n"],
                               "delta": r["tip_at_peak"] - s.tip, "moved": bool(r["moved"])})
        if not pushes:
            return {**none, "verdict": "not_measurable",
                    "why": "neither push met a surface" + (f" (ramped to {max(tops):.1f} N)" if tops else
                                                           ": no force is known for it"),
                    "hint": "touch a firm surface, then measure; state force_n (the force you plan to push with)"}
        best = max(pushes, key=lambda x: np.linalg.norm(x["delta"]) / x["force_n"])
        tip = self._sense().tip
        rxy = float(np.hypot(tip[0], tip[1]))
        radial = np.array([tip[0], tip[1], 0.0]) / rxy if rxy > 0.0 else np.array([1.0, 0.0, 0.0])
        tang = np.array([-radial[1], radial[0], 0.0])
        dl = best["delta"]
        shift = float(np.linalg.norm(dl))
        per_n = shift / best["force_n"]   # m per N
        tol = self._point_error_m()
        gives = bool(best["moved"])   # the shift passed what the arm's own give explains at that force
        bound = tol / per_n if gives and tol is not None and per_n > 0.0 else None
        verdict = "gives" if gives else "rigid"
        self.base_give = {"mm_per_n": per_n * MM, "g_base_m_per_n": per_n if gives else 0.0, "bound_n": bound,
                          "verdict": verdict}
        out = {"tilt_deg": round(math.degrees(math.atan2(float(dl[2]), rxy)), 3) if rxy > 0.0 else 0.0,
               "yaw_deg": round(math.degrees(float(dl @ tang) / rxy), 3) if rxy > 0.0 else 0.0,
               "shift_mm": round(shift * MM, 3), "mm_per_n": round(per_n * MM, 4), "verdict": verdict,
               "push_bound_n": None if bound is None else round(bound, 1),
               "pushes": [{"dir": x["dir"], "force_n": round(x["force_n"], 2),
                           "tip_shift_mm": [round(float(v) * MM, 3) for v in x["delta"]]} for x in pushes],
               "note": "the shift includes the touched surface's own give"}
        if bound is not None:
            out["push_bound_why"] = (f"at {per_n * MM:.3f} mm per N a push over {bound:.1f} N shifts the base more "
                                     f"than the {tol * MM:.1f} mm the stored points are good to")
        self.world.event("base_give", data={k: out[k] for k in ("tilt_deg", "yaw_deg", "shift_mm", "verdict")})
        return out

    # ------------------------------------------------------------------ a stroke's line and what it expects ahead
    def _line(self, q0, d, length, hold=None):
        """The joint waypoints of a straight tip line from q0 along d (unit, base), one per control period of the
        robot's own full speed there, as far as the arm can run it: (path, "goal" or "limit", why, hold). A
        waypoint is solved until the solve gains nothing more; one it does not reach (the tip off its line by the
        tool's own half width, or the finger off its hold) is tried at half the step, a set number of times:
        where even that fails the line ends. An arm
        that cannot keep its finger's direction along the line keeps its pitch alone, if that goes further (as
        PlanMixin._plan_hold). The bounds time the waypoints; nothing is timed here."""
        B = bounds.of(self)
        p0, R0 = self.model.fk_tip(q0)
        tol = B.standoff(0.0)   # the tool's own half width

        def run(how):
            path, q, c = [], np.asarray(q0, dtype=float).copy(), 0.0
            step, halved = B.v_free(q0, d) * B.dt(), 0
            while c < length and halved <= MARGIN_HALVINGS:
                at = min(float(length), c + step)
                q2, err, er = q, math.inf, math.inf
                for _ in range(MARGIN_HALVINGS):
                    was = err
                    q2, err, er = self._track(q2, p0 + d * at, R0, how)
                    if err >= was * (1.0 - REL):   # the solve gains under a fifth of what was left: it is done
                        break
                if err > tol or er > LINE_TOL_RAD:
                    step, halved = step * GIVE, halved + 1
                    continue
                path.append(q2.copy())
                q, c = q2, at
            return path, c

        hold = self.model.n < 6 if hold is None else hold   # see PlanMixin._rot_err
        path, got = run(hold)
        if got < length and hold is True:
            alt, far = run("pitch")
            if far > got:
                path, got, hold = alt, far, "pitch"
        if got < length:
            return path, "limit", self._limit_why(path[-1] if path else np.asarray(q0, dtype=float)), hold
        return path, "goal", "", hold

    def _ahead(self, B, p, d, length):
        """The thing the space book expects a stroke from p along d to meet within length (with what the session
        has learned of it), or None: the way is free or unseen."""
        kind, thing, _, _ = B.space.kind(p, p + d * length, 0.0)
        if kind != "expected" or thing is None:
            return None
        return B.thing(thing.id) if getattr(thing, "id", None) is not None else thing

    @staticmethod
    def _gap(thing, p):
        """How far the point p (m) stands off a thing: its height over a plane, its distance from a point; None
        for a thing with neither."""
        plane = (thing.extent or {}).get("plane") if isinstance(thing.extent, dict) else None
        if plane is not None:
            n = np.asarray(plane[0], dtype=float)
            return float(n @ p - plane[1]) / float(np.linalg.norm(n))
        return None if thing.point is None else float(np.linalg.norm(p - np.asarray(thing.point, dtype=float)))

    def _same_spot(self, B, thing, point_mm, sigma_m):
        """True when a contact at point_mm (its own error sigma_m) is at the thing the stroke expected."""
        gap = None if thing is None or point_mm is None else self._gap(thing, np.asarray(point_mm, dtype=float) / MM)
        if gap is None:
            return False
        return abs(gap) <= (laws.band(thing.sigma_p or 0.0)
                            + laws.same_spot(thing.sigma_p or 0.0, sigma_m, B.standoff(0.0)))

    # ------------------------------------------------------------------ a stroke's helpers
    def _lead_for(self, force_n, q, d):
        """How far the command must lead the tip along d for the servo to push force_n (m)."""
        return float(force_n) / bounds.of(self).k_servo(q, d)

    def _track_mask(self, q, p_des, mask):
        J = self.model.jac(q)[:3][:, mask]
        e = p_des - self.model.fk_tip(q)[0]
        lam2 = (0.01 * self.model.length_scale) ** 2 * 0.01   # numeric: the damping of the masked solve
        dqm = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(3), e)
        step = np.asarray(bounds.of(self).qd_max(), dtype=float)[mask] * self.dt   # a joint's own speed, one period
        over = float(np.max(np.abs(dqm) / step)) if dqm.size else 0.0
        if over > 1.0:
            dqm = dqm / over
        q2 = np.asarray(q, dtype=float).copy()
        q2[mask] += dqm
        return self.model.clip(q2)

    def _arrival(self, B, q, d, intent, thing=None, stroke=False):
        """What a stated limit makes of a stroke's speed: {"expect": the expected contact record,
        "v": the arrival speed in m/s or None (nothing stated: the body's own speed), "kept", "note", "refused"}.
        stroke: the bound of a press's stroke once the part yields (max_n, a sheet, the site; never the force the
        press is asked to reach). A limit under what this arm can tell is acted on at the tell speed when it is
        the operator's own (not shown kept) and refused limit_not_keepable when a sheet, a rating, the site or a
        give states it."""
        own = {k: v for k, v in intent.items() if not (stroke and k == "force_n")}
        expect = control_expect.of_intent(self, own, d, thing)
        out = {"expect": expect, "v": None, "kept": None, "note": None, "refused": None}
        limit, source = expect["bound_n"], expect["bound_source"]
        if limit is None:
            return out
        a = B.arrival(limit, source, q, d, floor=expect.get("floor"))
        out.update(v=a.get("v_m_s"), kept=a.get("kept"), note=a.get("text"))
        if not a.get("seek", True):
            out["refused"] = refusal("limit_not_keepable", a.get("text") or f"the limit of {limit:g} N ({source}) "
                                     "is under what this arm can tell", "this contact cannot be sought by force "
                                     "under that limit here: go to it by position (move_tip), or come at it "
                                     "another way")
        return out

    def _stroke_plan(self, s, d, length):
        """(refusal or None, hold) for a stroke of `length` along d: the path check of its plan and the hold the
        arm can keep along it (PlanMixin._plan_hold)."""
        path, _, _, hold = self._line(s.q, d, length)
        return self._path_check([s.q] + path, kind="press"), hold

    def _stroke_check(self, s, d, length):
        """The path check of a stroke's plan: a refusal or None."""
        return self._stroke_plan(s, d, length)[0]

    # ------------------------------------------------------------------ what a press needs beside its stroke
    def _rounds_split(self, q, d):
        """The joints of a rounds press: (those that take up the load, those that finish it): the chain's own
        order, its base half and its tool half (a fact of the body; no robot is named)."""
        k = int(math.ceil(self.model.n * GIVE))
        return list(range(k)), list(range(k, self.model.n))

    def _stroke_keys(self, B, legs, top, arr, strk, peak_n=0.0):
        """What a press says of its own speed and strike: the limit its approach ran under and whether
        it is shown kept (never when a reading of the call passed it), the fastest the part ran once it yielded,
        and the largest strike an end stop could have taken (over the stroke: the force on the part plus the
        stop slope there times the tip's speed; unresolved where the body has no slope)."""
        v = max((float(lg.get("v_part") or 0.0) for lg in legs), default=0.0)
        z = next((lg["z_end"] for lg in legs if lg.get("z_end") is not None), None)
        hit = max((float(lg.get("strike_n") or 0.0) for lg in legs), default=0.0)
        keys = self._move_keys(legs[0].get("plan"), 0)
        out = {k: keys[k] for k in ("told_mm", "tip_mm", "behind_mm", "strike", "predicted_peak_n", "tail_contact")
               if k in keys}
        lim, src, kept = control_move_press.limit_keys(arr, peak_n, hit)   # never the force the approach ends at
        out.update(approach_mm_s=None if arr["v"] is None else round(arr["v"] * MM, 2),
                   limit_n=lim, limit_source=src, limit_kept=kept, stroke_limit_n=strk["expect"]["bound_n"],
                   fastest_mm_s=round(v * MM, 2),
                   end_stop_strike_n=None if z is None else round(hit, 2))
        if z is None:
            out["end_stop_note"] = "peak unresolved: no stop slope is known for this body here"
        notes = [n for n in (arr["note"], strk["note"]) if n]
        if notes:
            out["limit_notes"] = notes
        return out

    # ------------------------------------------------------------------ after a stroke (A4.8)
    def _retrace_now(self, done, thing=None):
        """Go back along a stroke's own measured path (space the stroke swept: free), away from the thing it
        pushed: a cap or a stop request ends it, the force of leaving does not."""
        if len(done) < len(("from", "to")):
            return self._sense()
        d = self.model.fk_tip(done[0])[0] - self.model.fk_tip(done[-1])[0]
        size = float(np.linalg.norm(d))
        g = Guard(self, kind="retreat", away_from=d / size if size > 0.0 else thing)
        for q in reversed(done[:-1]):
            if g.check(self._period(np.asarray(q, dtype=float))):
                break
        if g.asked:
            self._stop_asked()
        return self._ease_impl()

    def _retrace(self, free, e, d, peak_n=0.0):
        """The path a retreat may retrace after a push along d that ended eased at reading e: the measured poses
        from before anything resisted that stand at least `lift` back from where the tip rests, then the tool
        lifted that far back along the push as it rests, then the resting pose. The joints wind up under a push,
        so going straight from the resting pose to a free pose next to the part would swing the tip through the
        part; lifted first, by what the servo's lead under that push was plus what the tip can tell, the swing
        happens clear of it."""
        B = bounds.of(self)
        lift = B.rigid(e.q, d, peak_n) + float(peak_n) / B.k_servo(e.q, d)
        q = e.q.copy()
        for _ in range(self.cfg["corrections"] + self.cfg["corrections"]):
            q, err, _ = self._track(q, e.tip - d * lift, e.R, self.model.n < 6)
            if err <= B.rigid(q, d):
                break
        keep = len(free)
        while keep > 0 and float((e.tip - self.model.fk_tip(free[keep - 1])[0]) @ d) < lift:
            keep -= 1
        return list(free[:keep]) + [q, e.q.copy()]

    def _lift_after_give(self, r, d, hold, thing=None, intent=None):
        """A part let go under the stroke: back along the stroke's own line to where it began and the tool's own
        back-off further (kind lift: the way the tool came, its guard directional). Returns a leg record."""
        s = self._sense()
        back = max(0.0, r["travel_m"]) + bounds.of(self).travel("press", thing, s.q, d, intent).backoff_m
        m = self._cart_move(s, -d * back, None, None, True, True, kind="lift", away_from=thing, intent=intent)
        e = self._sense()
        if "refused" in m:
            return {"stopped": "refused", "why": m["refused"].get("why"), "tip_mm": _mm(e.tip), "peak_force_n": 0.0}
        c = m["contact"] if m["stop"] != "goal" else None
        return {"stopped": m["stop"], "tip_mm": _mm(e.tip), "peak_force_n": round(m["peak_n"], 2),
                **({"felt": {k: c.get(k) for k in ("point_mm", "force_n", "surface")}} if c else {})}

    def _release(self, d, max_m, hold, until_n=None, extra_m=None, k_c=None):
        """Ease, then back the tool off along -d while the force reads over until_n (default: the contact
        threshold at rest), following the part: the command recedes at F / (k_w x t_det), k_w the servo's stiffness
        in series with the part's (k_c, read in the stroke just made; None: rigid), so the force falls by a share
        of itself per detection time and a sprung part comes back with the tool, not along its sides; never
        faster than the free speed of leaving. Once the force is gone (under until_n, or under the least share of
        its start the ramp resolves) it goes on extra_m (default: the least travel the tip can tell from none) and
        stops. At most max_m. (backed_m, followed_m, peak N): followed is how far the tip went back while
        something pushed on it, in the ease and in the back-off."""
        B = bounds.of(self)
        dt, t_det = B.dt(), B.t_det()
        until = B.f_thr() if until_n is None else float(until_n)
        s0 = self._sense()
        s = self._ease_impl()
        followed = max(0.0, float((s0.tip - s.tip) @ d)) if s0.Fmag > until else 0.0
        if s.Fmag <= until:
            return 0.0, followed, max(s0.Fmag, s.Fmag)
        gone_n = max(until, REL * REL * s.Fmag)
        k_s = B.k_servo(s.q, d)
        k_w = k_s if k_c is None or k_c <= 0.0 else k_s * k_c / (k_s + k_c)
        v_free = B.v_free(s.q, -d)
        extra = B.rigid(s.q, d) if extra_m is None else float(extra_m)
        q_des, p0, R0, prev = s.q.copy(), s.tip.copy(), s.R.copy(), s.tip.copy()
        c, quiet, peak, c_quiet = 0.0, 0, s.Fmag, None
        g = Guard(self, kind="release", away_from=-d)   # leaving: only a force that opposes the way back stops it
        n_det = max(1, int(math.ceil(t_det / dt)))
        # its time: the whole back-off at the slowest speed the law gives before the force counts as gone
        planned = max_m * k_w * t_det / gone_n + max_m / v_free + B.t_settle()
        for _ in range(int(math.ceil(B.limit_s(planned) / dt))):
            v = v_free if c_quiet is not None else min(v_free, s.Fmag / (k_w * t_det))
            f_was, c_was = s.Fmag, c
            c = min(max_m, c + v * dt)
            q_des, _, _ = self._track(q_des, p0 - d * c, R0, hold)
            s = self._period(q_des)
            if c > c_was and f_was > s.Fmag:   # the part as the back-off itself reads it: softer, so faster
                k_w = min(k_w, (f_was - s.Fmag) / (c - c_was))
            peak = max(peak, s.Fmag)
            if s.Fmag > until:
                followed += max(0.0, float((prev - s.tip) @ d))
            prev = s.tip.copy()
            quiet = quiet + 1 if s.Fmag <= gone_n else 0
            if quiet >= n_det and c_quiet is None:
                c_quiet = c
            if (c_quiet is not None and c >= c_quiet + extra) or g.check(s) or c >= max_m:
                break
        self._ease_impl()
        return float((p0 - self._sense().tip) @ d), followed, peak

    def _after_stroke(self, r, d, s0, hold, give_lift, thing=None, k_c=None, intent=None):
        """Lift after a give (give_lift), then the release and the keys every push result carries about it."""
        B = bounds.of(self)
        stop_s = r["s"]
        lifted = (self._lift_after_give(r, d, hold, thing, intent)
                  if give_lift and self.cfg["give_lift"] == "back" else None)
        gone = max(0.0, float((stop_s.tip - s0.tip) @ d)) if lifted is None else 0.0
        reach = gone + r["peak_n"] / B.k_servo(stop_s.q, d) + B.travel("press", thing, stop_s.q, d, intent).backoff_m
        backed, followed, peak_rel = self._release(d, reach, hold, k_c=k_c)
        e = self._sense()
        at_rest = B.f_thr()
        # how far the part pushed the tool back between the stop and now
        sprang = 0.0 if lifted is not None else max(0.0, followed)
        resisted = r["felt"].resisted
        tell = B.rigid(e.q, d)   # the least travel the tip can tell from none
        out = {"tip_mm_at_stop": _mm(stop_s.tip), "along_push_mm": round(r["travel_m"] * MM, 2),
               "across_mm": round(r["across_m"] * MM, 2), "force_after_n": round(e.Fmag, 2),
               "eased": bool(e.Fmag <= at_rest), "still_touching": bool(e.Fmag > at_rest),
               "released_mm": round(backed * MM, 2),
               "sprang_back": bool(resisted > tell and sprang > tell and sprang >= GIVE * resisted
                                   and not r["gave"]),
               "sprang_back_mm": round(sprang * MM, 2)}
        if not out["eased"]:
            out["eased_why"] = (f"the tool still reads {e.Fmag:.1f} N after backing off {backed * MM:.1f} mm: "
                                "something holds against it; retreat before the next push")
        if lifted is not None:
            out["lifted"] = dict(lifted, why="the part gave way: the tool went back along its stroke to where the "
                                             "press began and its own back-off further, out of the part's way")
        return out, max(peak_rel, (lifted or {}).get("peak_force_n", 0.0))

    def _stale_near(self, point_mm, why, travel_m=0.0, along=None):
        """A part moved at point_mm by travel_m: the located and contact points within that travel and their own
        "same spot" distance of it may no longer be where the part is: robot://world marks them
        stale. The part the act set moving voids the arm's passage on the side it went (along: the act's
        travel, else the last stroke's). Returns the receipt's "void" block, or None."""
        if point_mm is None or not hasattr(self.world, "mark_stale"):
            return None
        B = bounds.of(self)
        p = np.asarray(point_mm, dtype=float) / MM
        near = []
        for kind in ("contact", "located"):
            for e in self.world.query(kind=kind, include_stale=False):
                if e.get("point_mm") is None:
                    continue
                gap = float(np.linalg.norm(np.asarray(e["point_mm"], dtype=float) / MM - p))
                if gap <= float(travel_m) + B.standoff(0.0, B.thing(e["id"])):
                    near.append(e["id"])
        if near:
            self.world.mark_stale(near, why)
        along = getattr(self, "_travel_direction", None) if along is None else along
        return control_move_passed.void(self, p, along, why, ", ".join(near) or None)
